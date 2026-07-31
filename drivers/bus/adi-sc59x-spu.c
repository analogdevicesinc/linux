// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * System Protection Unit (SPU) for ADI SC59x processors
 *
 * The SPU is a peripheral firewall: it write-protects peripheral MMR space
 * from selected bus masters and, when secure registers are available, marks
 * peripherals as secure completers.  Protection is described in the device
 * tree and applied at probe.
 *
 * Copyright (C) 2025 - Analog Devices, Inc.
 * Author: Ozan Durgut <ozan.durgut@analog.com>
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/* Register offsets */
#define ADI_SPU_REG_CTL			0x000
#define ADI_SPU_REG_STAT		0x004
#define ADI_SPU_WP(n)			(0x400 + (n) * 0x04)

/* Secure register block (gated behind adi,secure-access) */
#define ADI_SPU_SECURE_REG_BASE		0x800
#define ADI_SPU_REG_SECURECTL		0x840
#define ADI_SPU_SECUREP(n)		(0xA00 + (n) * 0x04)

/* SPU_CTL register bits */
#define ADI_SPU_CTL_WPLCK		BIT(16)
#define ADI_SPU_CTL_PINTEN		BIT(14)
#define ADI_SPU_CTL_GLCK_MASK		GENMASK(7, 0)
#define ADI_SPU_CTL_GLCK_UNLOCK		0xAD

/* SPU_STAT register bits */
#define ADI_SPU_STAT_LWERR		BIT(31)		/* W1C */
#define ADI_SPU_STAT_ADDRERR		BIT(30)		/* W1C */
#define ADI_SPU_STAT_VIRQ		BIT(12)		/* W1C */
#define ADI_SPU_STAT_W1C_MASK	\
	(ADI_SPU_STAT_LWERR | ADI_SPU_STAT_ADDRERR | ADI_SPU_STAT_VIRQ)

/* SPU_SECURECTL register bits */
#define ADI_SPU_SECURECTL_SINTEN	BIT(14)

/* SPU_SECUREP[n] register bits */
#define ADI_SPU_SECUREP_SSEC		BIT(0)

/*
 * Number of write-protect / secure-peripheral registers: SPU_WP[n] spans
 * 0x400-0x754 and SPU_SECUREP[n] spans 0xA00-0xD54, both 214 entries.
 */
#define ADI_SPU_NUM_PERIPHERALS		214

/*
 * SPU_WP[n] master bits: CM[2:0] select core masters, SM[18:16] select system
 * masters. The device tree "adi,write-protect" property gives the mask
 * directly, so only the field positions are needed here.
 */
#define ADI_SPU_WP_CM_MASK		GENMASK(2, 0)
#define ADI_SPU_WP_SM_MASK		GENMASK(18, 16)
#define ADI_SPU_WP_MASK			(ADI_SPU_WP_CM_MASK | ADI_SPU_WP_SM_MASK)

/* struct adi_spu - SPU driver data */
struct adi_spu {
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;		/* serialises register read-modify-write */
	struct dentry *debugfs_root;
	bool secure_regs_available;
	unsigned long violations;
};

/* Register access helpers */
static u32 adi_spu_readl(const struct adi_spu *spu, u32 offset)
{
	return readl(spu->base + offset);
}

static void adi_spu_writel(const struct adi_spu *spu, u32 val, u32 offset)
{
	writel(val, spu->base + offset);
}

/*
 * Write to a secure register (offset >= ADI_SPU_SECURE_REG_BASE).
 * Returns 0 on success, -EOPNOTSUPP if secure registers are unavailable, or
 * -EINVAL on a bad offset.
 */
static int adi_spu_write_secure_reg(const struct adi_spu *spu, u32 val,
				    u32 offset)
{
	if (!spu->secure_regs_available) {
		dev_warn_ratelimited(spu->dev,
				     "Secure register access unavailable\n");
		return -EOPNOTSUPP;
	}

	if (offset < ADI_SPU_SECURE_REG_BASE)
		return -EINVAL;

	adi_spu_writel(spu, val, offset);
	return 0;
}

/* SPU violation interrupt handler (SEC ID 216). */
static irqreturn_t adi_spu_irq_handler(int irq, void *dev_id)
{
	struct adi_spu *spu = dev_id;
	u32 status;

	/* Already in hard IRQ context, so plain spin_lock() is enough. */
	spin_lock(&spu->lock);

	status = adi_spu_readl(spu, ADI_SPU_REG_STAT);
	if (!(status & ADI_SPU_STAT_W1C_MASK)) {
		spin_unlock(&spu->lock);
		return IRQ_NONE;
	}

	if (status & ADI_SPU_STAT_VIRQ) {
		spu->violations++;
		dev_err_ratelimited(spu->dev,
				    "protection/security violation (STAT=0x%08x)\n",
				    status);
	}

	if (status & ADI_SPU_STAT_LWERR)
		dev_err_ratelimited(spu->dev,
				    "write to locked SPU register blocked\n");

	if (status & ADI_SPU_STAT_ADDRERR)
		dev_err_ratelimited(spu->dev,
				    "invalid SPU MMR address access\n");

	adi_spu_writel(spu, status & ADI_SPU_STAT_W1C_MASK, ADI_SPU_REG_STAT);

	spin_unlock(&spu->lock);

	return IRQ_HANDLED;
}

/* Static configuration from the device tree */

/*
 * Parse and apply "adi,write-protect": a list of <peripheral-id, master-mask>
 * pairs. Each pair sets the SPU_WP[n] master bits so the listed bus masters
 * are blocked from writing that peripheral's MMR space. Runs at probe before
 * the IRQ is requested, so no locking is needed.
 */
static int adi_spu_parse_write_protect(struct adi_spu *spu)
{
	struct device *dev = spu->dev;
	u32 *pairs;
	int count, i, ret;

	count = device_property_count_u32(dev, "adi,write-protect");
	if (count <= 0)
		return 0;

	if (count % 2) {
		dev_err(dev, "adi,write-protect must be <id mask> pairs\n");
		return -EINVAL;
	}

	pairs = devm_kmalloc_array(dev, count, sizeof(*pairs), GFP_KERNEL);
	if (!pairs)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, "adi,write-protect", pairs,
					     count);
	if (ret)
		return ret;

	for (i = 0; i < count; i += 2) {
		u32 id = pairs[i];
		u32 mask = pairs[i + 1];

		if (id >= ADI_SPU_NUM_PERIPHERALS) {
			dev_err(dev, "write-protect peripheral %u out of range\n",
				id);
			return -EINVAL;
		}

		/*
		 * Reject master bits outside the valid CM/SM fields rather
		 * than silently masking them: a dropped bit would narrow the
		 * requested protection and could leave a master able to write
		 * the peripheral (fail-open) without any diagnostic.
		 */
		if (mask & ~ADI_SPU_WP_MASK) {
			dev_err(dev, "write-protect peripheral %u: invalid master mask %#x\n",
				id, mask);
			return -EINVAL;
		}

		adi_spu_writel(spu, mask, ADI_SPU_WP(id));
		dev_dbg(dev, "write-protect peripheral %u masters %#x\n", id, mask);
	}

	return 0;
}

/*
 * Parse and apply "adi,secure-peripherals": peripheral IDs to mark as secure
 * completers (SPU_SECUREP[n].SSEC). Only effective with secure register
 * access; ignored otherwise.
 */
static int adi_spu_parse_secure(struct adi_spu *spu)
{
	struct device *dev = spu->dev;
	u32 *ids;
	int count, i, ret;

	if (!spu->secure_regs_available)
		return 0;

	count = device_property_count_u32(dev, "adi,secure-peripherals");
	if (count <= 0)
		return 0;

	ids = devm_kmalloc_array(dev, count, sizeof(*ids), GFP_KERNEL);
	if (!ids)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, "adi,secure-peripherals", ids,
					     count);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		if (ids[i] >= ADI_SPU_NUM_PERIPHERALS) {
			dev_err(dev, "secure peripheral %u out of range\n", ids[i]);
			return -EINVAL;
		}

		adi_spu_write_secure_reg(spu, ADI_SPU_SECUREP_SSEC,
					 ADI_SPU_SECUREP(ids[i]));
	}

	return 0;
}

/* Debugfs interface */

static int spu_status_show(struct seq_file *s, void *data)
{
	struct adi_spu *spu = s->private;
	unsigned long flags;
	u32 ctl, stat;

	spin_lock_irqsave(&spu->lock, flags);
	ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
	stat = adi_spu_readl(spu, ADI_SPU_REG_STAT);
	spin_unlock_irqrestore(&spu->lock, flags);

	seq_printf(s, "Control:   0x%08x\n", ctl);
	seq_printf(s, "  Global lock:   %s\n",
		   (ctl & ADI_SPU_CTL_GLCK_MASK) == ADI_SPU_CTL_GLCK_UNLOCK ?
		   "off" : "on");
	seq_printf(s, "  WP lock:       %s\n",
		   ctl & ADI_SPU_CTL_WPLCK ? "yes" : "no");
	seq_printf(s, "  IRQ enabled:   %s\n",
		   ctl & ADI_SPU_CTL_PINTEN ? "yes" : "no");
	seq_printf(s, "Status:    0x%08x\n", stat);
	seq_printf(s, "  Violations:    %lu\n", spu->violations);
	seq_printf(s, "  Secure regs:   %s\n",
		   spu->secure_regs_available ? "available" : "unavailable");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(spu_status);

static int spu_write_protect_show(struct seq_file *s, void *data)
{
	struct adi_spu *spu = s->private;
	unsigned long flags;
	unsigned int i;

	seq_puts(s, "Peripherals with active write protection:\n");

	spin_lock_irqsave(&spu->lock, flags);
	for (i = 0; i < ADI_SPU_NUM_PERIPHERALS; i++) {
		u32 wp = adi_spu_readl(spu, ADI_SPU_WP(i));

		if (wp)
			seq_printf(s, "  WP[%u] = 0x%08x\n", i, wp);
	}
	spin_unlock_irqrestore(&spu->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(spu_write_protect);

static void adi_spu_debugfs_remove(void *data)
{
	struct adi_spu *spu = data;

	debugfs_remove_recursive(spu->debugfs_root);
}

static int adi_spu_debugfs_init(struct adi_spu *spu)
{
	int ret;

	spu->debugfs_root = debugfs_create_dir(dev_name(spu->dev), NULL);

	ret = devm_add_action_or_reset(spu->dev, adi_spu_debugfs_remove, spu);
	if (ret)
		return ret;

	debugfs_create_file("status", 0444, spu->debugfs_root, spu,
			    &spu_status_fops);
	debugfs_create_file("write-protect", 0444, spu->debugfs_root, spu,
			    &spu_write_protect_fops);

	return 0;
}

/* Platform driver */

static void adi_spu_disable_irq(void *data)
{
	struct adi_spu *spu = data;
	u32 ctl;

	ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
	ctl &= ~ADI_SPU_CTL_PINTEN;
	adi_spu_writel(spu, ctl, ADI_SPU_REG_CTL);

	/*
	 * The security-violation interrupt is enabled separately in
	 * SPU_SECURECTL.SINTEN; clear it too so a later violation cannot
	 * re-assert the shared line after the handler has been freed.
	 */
	if (spu->secure_regs_available)
		adi_spu_write_secure_reg(spu, 0, ADI_SPU_REG_SECURECTL);
}

static int adi_spu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_spu *spu;
	u32 ctl;
	int irq, ret;

	spu = devm_kzalloc(dev, sizeof(*spu), GFP_KERNEL);
	if (!spu)
		return -ENOMEM;

	spu->dev = dev;
	spin_lock_init(&spu->lock);
	dev_set_drvdata(dev, spu);

	spu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spu->base))
		return dev_err_probe(dev, PTR_ERR(spu->base),
				     "Failed to map SPU registers\n");

	spu->secure_regs_available =
		of_property_read_bool(np, "adi,secure-access");

	/* Clear any pending violations and mask the interrupt for now. */
	adi_spu_writel(spu, ADI_SPU_STAT_W1C_MASK, ADI_SPU_REG_STAT);
	ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
	ctl &= ~ADI_SPU_CTL_PINTEN;
	adi_spu_writel(spu, ctl, ADI_SPU_REG_CTL);

	/* Enable the security violation interrupt if secure regs are usable. */
	if (spu->secure_regs_available) {
		if (adi_spu_write_secure_reg(spu, ADI_SPU_SECURECTL_SINTEN,
					     ADI_SPU_REG_SECURECTL))
			spu->secure_regs_available = false;
	}

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	if (irq > 0) {
		ret = devm_request_irq(dev, irq, adi_spu_irq_handler,
				       0, dev_name(dev), spu);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to request IRQ %d\n", irq);

		ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
		ctl |= ADI_SPU_CTL_PINTEN;
		adi_spu_writel(spu, ctl, ADI_SPU_REG_CTL);

		/* Mask the interrupt before the handler is freed on unbind. */
		ret = devm_add_action_or_reset(dev, adi_spu_disable_irq, spu);
		if (ret)
			return ret;
	} else {
		dev_warn(dev, "No IRQ specified, violations will not be reported\n");
	}

	ret = adi_spu_parse_write_protect(spu);
	if (ret)
		return ret;

	ret = adi_spu_parse_secure(spu);
	if (ret)
		return ret;

	return adi_spu_debugfs_init(spu);
}

static const struct of_device_id adi_spu_match[] = {
	{ .compatible = "adi,sc59x-spu" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_spu_match);

static struct platform_driver adi_spu_driver = {
	.probe = adi_spu_probe,
	.driver = {
		.name = "adi-sc59x-spu",
		.of_match_table = adi_spu_match,
	},
};

module_platform_driver(adi_spu_driver);

MODULE_DESCRIPTION("System Protection Unit for ADI SC59x SoCs");
MODULE_AUTHOR("Ozan Durgut <ozan.durgut@analog.com>");
MODULE_LICENSE("GPL");
