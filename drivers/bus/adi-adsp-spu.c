// SPDX-License-Identifier: GPL-2.0-only
/*
 * System Protection Unit (SPU) for ADI ADSP processors
 *
 * The SPU is the peripheral firewall found on several Analog Devices
 * ADSP/SHARC-FX processors (SC59x, SC846, SC594): it write-protects a
 * peripheral's MMR space from selected bus masters and can mark
 * peripherals as secure completers. Register layout is identical across
 * these chips; only the size of the write-protect register array differs,
 * so it is carried as per-chip match data instead of a fixed constant.
 *
 * The protection policy is owned by the secure firmware that runs before
 * Linux, matching the model used by other in-tree access-control hardware.
 * This driver does not program any policy from the device tree; it maps the
 * SPU, reports protection/security violations through its interrupt, and
 * exposes the programmed write-protect state read-only through debugfs.
 *
 * Copyright (C) 2026 - Analog Devices, Inc.
 * Author: Ozan Durgut <ozan.durgut@analog.com>
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string_choices.h>
#include <linux/types.h>

/* Register offsets */
#define ADI_SPU_REG_CTL			0x000
#define ADI_SPU_REG_STAT		0x004
#define ADI_SPU_WP(n)			(0x400 + (n) * 0x04)

/* SPU_CTL register bits */
#define ADI_SPU_CTL_WPLCK		BIT(16)
#define ADI_SPU_CTL_PINTEN		BIT(14)
#define ADI_SPU_CTL_GLCK_MASK		GENMASK(7, 0)
#define ADI_SPU_CTL_GLCK_UNLOCK		0xAD

/* SPU_STAT register bits (all write-1-to-clear) */
#define ADI_SPU_STAT_LWERR		BIT(31)
#define ADI_SPU_STAT_ADDRERR		BIT(30)
#define ADI_SPU_STAT_VIRQ		BIT(12)
#define ADI_SPU_STAT_W1C_MASK	\
	(ADI_SPU_STAT_LWERR | ADI_SPU_STAT_ADDRERR | ADI_SPU_STAT_VIRQ)

/* Number of write-protect registers (SPU_WP[n]) differs per chip. */
struct adi_spu_chip_data {
	unsigned int num_peripherals;
};

/* SPU_WP[n] spans 0x400-0x754, 214 entries. */
static const struct adi_spu_chip_data sc59x_spu_data = {
	.num_peripherals = 214,
};

/* SPU_WP[n] spans 0x400-0x750, 213 entries. */
static const struct adi_spu_chip_data sc846_spu_data = {
	.num_peripherals = 213,
};

/* SPU_WP[n] spans 0x400-0x724, 202 entries. */
static const struct adi_spu_chip_data sc594_spu_data = {
	.num_peripherals = 202,
};

struct adi_spu {
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;		/* serialises register read-modify-write */
	struct dentry *debugfs_root;
	unsigned long violations;
	unsigned int num_peripherals;
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

/* SPU violation interrupt handler. */
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
	seq_printf(s, "  WP lock:       %s\n", str_yes_no(ctl & ADI_SPU_CTL_WPLCK));
	seq_printf(s, "  IRQ enabled:   %s\n", str_yes_no(ctl & ADI_SPU_CTL_PINTEN));
	seq_printf(s, "Status:    0x%08x\n", stat);
	seq_printf(s, "  Violations:    %lu\n", spu->violations);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(spu_status);

static int spu_write_protect_show(struct seq_file *s, void *data)
{
	struct adi_spu *spu = s->private;
	unsigned int i;

	seq_puts(s, "Peripherals with active write protection:\n");

	/*
	 * The write-protect registers are only read here and never written by
	 * this report-only driver, so they do not race the STAT-only interrupt
	 * handler; no locking is needed.
	 */
	for (i = 0; i < spu->num_peripherals; i++) {
		u32 wp = adi_spu_readl(spu, ADI_SPU_WP(i));

		if (wp)
			seq_printf(s, "  WP[%u] = 0x%08x\n", i, wp);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(spu_write_protect);

static void adi_spu_debugfs_remove(void *data)
{
	struct adi_spu *spu = data;

	debugfs_remove_recursive(spu->debugfs_root);
}

static void adi_spu_debugfs_init(struct adi_spu *spu)
{
	struct dentry *root;

	root = debugfs_create_dir(dev_name(spu->dev), NULL);
	spu->debugfs_root = root;

	if (devm_add_action_or_reset(spu->dev, adi_spu_debugfs_remove, spu))
		return;

	debugfs_create_file("status", 0444, root, spu, &spu_status_fops);
	debugfs_create_file("write-protect", 0444, root, spu,
			    &spu_write_protect_fops);
}

/* Mask the violation interrupt (devm unbind action). */
static void adi_spu_disable_irq(void *data)
{
	struct adi_spu *spu = data;
	unsigned long flags;
	u32 ctl;

	spin_lock_irqsave(&spu->lock, flags);
	ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
	ctl &= ~ADI_SPU_CTL_PINTEN;
	adi_spu_writel(spu, ctl, ADI_SPU_REG_CTL);
	spin_unlock_irqrestore(&spu->lock, flags);
}

static int adi_spu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct adi_spu_chip_data *chip_data;
	unsigned long flags;
	struct adi_spu *spu;
	u32 ctl;
	int irq, ret;

	chip_data = device_get_match_data(dev);
	if (!chip_data)
		return -ENODEV;

	spu = devm_kzalloc(dev, sizeof(*spu), GFP_KERNEL);
	if (!spu)
		return -ENOMEM;

	spu->dev = dev;
	spu->num_peripherals = chip_data->num_peripherals;
	spin_lock_init(&spu->lock);
	platform_set_drvdata(pdev, spu);

	spu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spu->base))
		return dev_err_probe(dev, PTR_ERR(spu->base),
				     "failed to map SPU registers\n");

	/* Clear any pending violation and enable the violation interrupt. */
	adi_spu_writel(spu, ADI_SPU_STAT_W1C_MASK, ADI_SPU_REG_STAT);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, adi_spu_irq_handler, 0,
			       dev_name(dev), spu);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n", irq);

	/*
	 * The handler is registered above, so serialise this CTL update
	 * against it (consistent with the SMPU driver).
	 */
	spin_lock_irqsave(&spu->lock, flags);
	ctl = adi_spu_readl(spu, ADI_SPU_REG_CTL);
	ctl |= ADI_SPU_CTL_PINTEN;
	adi_spu_writel(spu, ctl, ADI_SPU_REG_CTL);
	spin_unlock_irqrestore(&spu->lock, flags);

	ret = devm_add_action_or_reset(dev, adi_spu_disable_irq, spu);
	if (ret)
		return ret;

	adi_spu_debugfs_init(spu);

	return 0;
}

static const struct of_device_id adi_spu_match[] = {
	{ .compatible = "adi,sc59x-spu", .data = &sc59x_spu_data },
	{ .compatible = "adi,sc846-spu", .data = &sc846_spu_data },
	{ .compatible = "adi,sc594-spu", .data = &sc594_spu_data },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_spu_match);

static struct platform_driver adi_spu_driver = {
	.probe = adi_spu_probe,
	.driver = {
		.name = "adi-adsp-spu",
		.of_match_table = adi_spu_match,
	},
};
module_platform_driver(adi_spu_driver);

MODULE_DESCRIPTION("System Protection Unit for ADI ADSP SoCs");
MODULE_AUTHOR("Ozan Durgut <ozan.durgut@analog.com>");
MODULE_LICENSE("GPL");
