// SPDX-License-Identifier: GPL-2.0-only
/*
 * System Memory Protection Unit (SMPU) for ADI SC59x processors
 *
 * Copyright (C) 2026 - Analog Devices, Inc.
 * Author: Ozan Durgut <ozan.durgut@analog.com>
 *
 * The SMPU is the memory firewall of the ADSP-SC59x processors: it restricts
 * accesses to L2 SRAM / DDR / boot-ROM windows by bus-master transaction ID.
 *
 * The protection policy (which regions are protected, and for which masters)
 * is owned by the secure firmware that runs before Linux, matching the model
 * used by other in-tree access-control hardware. This driver does not program
 * any policy from the device tree; it maps the SMPU instances, reports
 * protection violations through the shared interrupt, and exposes the
 * programmed state read-only through debugfs.
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
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string_choices.h>
#include <linux/types.h>

/* Register offsets */
#define ADI_SMPU_REG_CTL		0x00
#define ADI_SMPU_REG_STAT		0x04
#define ADI_SMPU_REG_IADDR		0x08
#define ADI_SMPU_REG_IDTLS		0x0C
#define ADI_SMPU_REG_BADDR		0x10

/* Region configuration registers (stride = 0x18 per region), read-only here */
#define ADI_SMPU_RCTL(n)		(0x20 + (n) * 0x18)
#define ADI_SMPU_RADDR(n)		(0x24 + (n) * 0x18)
#define ADI_SMPU_RIDA(n)		(0x28 + (n) * 0x18)
#define ADI_SMPU_RIDB(n)		(0x30 + (n) * 0x18)

/* SMPU_CTL register bits */
#define ADI_SMPU_CTL_LOCK		BIT(31)
#define ADI_SMPU_CTL_RLOCK		BIT(4)
#define ADI_SMPU_CTL_PINTEN		BIT(3)

/* SMPU_STAT register bits (all write-1-to-clear) */
#define ADI_SMPU_STAT_LWERR		BIT(17)
#define ADI_SMPU_STAT_ADRERR		BIT(16)
#define ADI_SMPU_STAT_BEOVR		BIT(3)
#define ADI_SMPU_STAT_BERR		BIT(2)
#define ADI_SMPU_STAT_IOVR		BIT(1)
#define ADI_SMPU_STAT_IRQ		BIT(0)
#define ADI_SMPU_STAT_W1C_MASK		\
	(ADI_SMPU_STAT_LWERR | ADI_SMPU_STAT_ADRERR | \
	 ADI_SMPU_STAT_BEOVR | ADI_SMPU_STAT_BERR | ADI_SMPU_STAT_IOVR | \
	 ADI_SMPU_STAT_IRQ)

/* SMPU_RCTL register bits (read-only decode for debugfs) */
#define ADI_SMPU_RCTL_WPROTEN		BIT(10)
#define ADI_SMPU_RCTL_RPROTEN		BIT(8)
#define ADI_SMPU_RCTL_SIZE_SHIFT	1
#define ADI_SMPU_RCTL_SIZE_MASK		GENMASK(5, 1)
#define ADI_SMPU_RCTL_EN		BIT(0)

/* SMPU_IDTLS register bits */
#define ADI_SMPU_DTLS_ID_SHIFT		8
#define ADI_SMPU_DTLS_ID_MASK		GENMASK(20, 8)
#define ADI_SMPU_DTLS_RW		BIT(1)
#define ADI_SMPU_DTLS_SECURE		BIT(0)

/* Transaction IDs are 13-bit */
#define ADI_SMPU_TID_MASK		GENMASK(12, 0)
/* RADDR[n].BADDR holds the region base in bits [31:12] in place. */
#define ADI_SMPU_RADDR_MASK		GENMASK(31, 12)

/* Maximum number of SMPU instances described in the device tree */
#define ADI_SMPU_MAX_INSTANCES		9

/* Number of protection regions per SMPU instance */
#define ADI_SMPU_NUM_REGIONS		8

/* Violation history buffer size per instance */
#define ADI_SMPU_VIOL_HISTORY_SIZE	32

/* One entry in the per-instance circular violation history buffer. */
struct adi_smpu_violation {
	unsigned long timestamp;
	u32 addr;
	u16 id;
	bool is_write;
	bool is_secure;
};

struct adi_smpu_instance {
	void __iomem *base;
	const char *name;
	spinlock_t lock;		/* serialises register and history access */
	struct adi_smpu *smpu;
	struct adi_smpu_violation violations[ADI_SMPU_VIOL_HISTORY_SIZE];
	unsigned int viol_head;
	unsigned long viol_count;
};

struct adi_smpu {
	struct device *dev;
	struct adi_smpu_instance instances[ADI_SMPU_MAX_INSTANCES];
	int num_instances;
	struct dentry *debugfs_root;
};

/* Register access helpers */
static u32 adi_smpu_readl(const struct adi_smpu_instance *inst, u32 offset)
{
	return readl(inst->base + offset);
}

static void adi_smpu_writel(const struct adi_smpu_instance *inst,
			    u32 val, u32 offset)
{
	writel(val, inst->base + offset);
}

/* Record a violation in the per-instance circular history buffer. */
static void adi_smpu_store_violation(struct adi_smpu_instance *inst,
				     u32 addr, u32 details)
{
	struct adi_smpu_violation *viol;
	unsigned int idx;

	idx = inst->viol_head % ADI_SMPU_VIOL_HISTORY_SIZE;
	viol = &inst->violations[idx];

	viol->timestamp = jiffies;
	viol->addr = addr;
	viol->id = (details & ADI_SMPU_DTLS_ID_MASK) >> ADI_SMPU_DTLS_ID_SHIFT;
	viol->is_write = !!(details & ADI_SMPU_DTLS_RW);
	viol->is_secure = !!(details & ADI_SMPU_DTLS_SECURE);

	inst->viol_head++;
	inst->viol_count++;
}

/* Log a protection violation and store it in the history buffer. */
static void adi_smpu_log_violation(struct adi_smpu_instance *inst,
				   u32 addr, u32 details)
{
	u16 trans_id = (details & ADI_SMPU_DTLS_ID_MASK) >> ADI_SMPU_DTLS_ID_SHIFT;
	bool is_write = !!(details & ADI_SMPU_DTLS_RW);
	bool is_secure = !!(details & ADI_SMPU_DTLS_SECURE);

	dev_err_ratelimited(inst->smpu->dev,
			    "%s violation: addr=0x%08x id=0x%03x %s %s\n",
			    inst->name, addr, trans_id,
			    is_write ? "write" : "read",
			    is_secure ? "secure" : "non-secure");

	adi_smpu_store_violation(inst, addr, details);
}

/*
 * SMPU violation interrupt handler.  All instances share the same IRQ line
 * (SEC ID 217), so walk every instance and service the ones that fired.
 * Returns IRQ_HANDLED if any violation was processed, IRQ_NONE otherwise.
 */
static irqreturn_t adi_smpu_irq_handler(int irq, void *dev_id)
{
	struct adi_smpu *smpu = dev_id;
	irqreturn_t ret = IRQ_NONE;
	int i;

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		u32 status;

		/* Already in hard IRQ context, so plain spin_lock() is enough. */
		spin_lock(&inst->lock);

		status = adi_smpu_readl(inst, ADI_SMPU_REG_STAT);
		if (!(status & ADI_SMPU_STAT_W1C_MASK)) {
			spin_unlock(&inst->lock);
			continue;
		}

		if (status & ADI_SMPU_STAT_IRQ) {
			u32 iaddr = adi_smpu_readl(inst, ADI_SMPU_REG_IADDR);
			u32 idtls = adi_smpu_readl(inst, ADI_SMPU_REG_IDTLS);

			adi_smpu_log_violation(inst, iaddr, idtls);
		}

		if (status & ADI_SMPU_STAT_IOVR)
			dev_warn_ratelimited(inst->smpu->dev,
					     "%s: interrupt overrun\n", inst->name);

		if (status & ADI_SMPU_STAT_BERR) {
			u32 baddr = adi_smpu_readl(inst, ADI_SMPU_REG_BADDR);

			dev_err_ratelimited(inst->smpu->dev,
					    "%s: bus error at 0x%08x\n",
					    inst->name, baddr);
		}

		if (status & ADI_SMPU_STAT_LWERR)
			dev_err_ratelimited(inst->smpu->dev,
					    "%s: locked-register write error\n",
					    inst->name);

		if (status & ADI_SMPU_STAT_ADRERR)
			dev_err_ratelimited(inst->smpu->dev,
					    "%s: address error\n", inst->name);

		/* Acknowledge every pending W1C status bit. */
		adi_smpu_writel(inst, status & ADI_SMPU_STAT_W1C_MASK,
				ADI_SMPU_REG_STAT);
		ret = IRQ_HANDLED;

		spin_unlock(&inst->lock);
	}

	return ret;
}

/* Clear any pending violation and enable the violation interrupt. */
static void adi_smpu_enable_irq(struct adi_smpu_instance *inst)
{
	unsigned long flags;
	u32 val;

	/*
	 * The handler is already registered on the shared line when this runs,
	 * so serialise the STAT/CTL access against it.
	 */
	spin_lock_irqsave(&inst->lock, flags);

	adi_smpu_writel(inst, ADI_SMPU_STAT_W1C_MASK, ADI_SMPU_REG_STAT);

	val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	val |= ADI_SMPU_CTL_PINTEN;
	adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);

	spin_unlock_irqrestore(&inst->lock, flags);
}

/* Mask the violation interrupt on every instance (devm unbind action). */
static void adi_smpu_disable_irqs(void *data)
{
	struct adi_smpu *smpu = data;
	int i;

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		unsigned long flags;
		u32 val;

		spin_lock_irqsave(&inst->lock, flags);
		val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
		val &= ~ADI_SMPU_CTL_PINTEN;
		adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);
		spin_unlock_irqrestore(&inst->lock, flags);
	}
}

static int smpu_status_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	u32 ctl, stat;

	spin_lock_irqsave(&inst->lock, flags);
	ctl = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	stat = adi_smpu_readl(inst, ADI_SMPU_REG_STAT);
	spin_unlock_irqrestore(&inst->lock, flags);

	seq_printf(s, "Control:   0x%08x\n", ctl);
	seq_printf(s, "  Locked:        %s\n", str_yes_no(ctl & ADI_SMPU_CTL_LOCK));
	seq_printf(s, "  Region locked: %s\n", str_yes_no(ctl & ADI_SMPU_CTL_RLOCK));
	seq_printf(s, "  IRQ enabled:   %s\n", str_yes_no(ctl & ADI_SMPU_CTL_PINTEN));
	seq_printf(s, "\nStatus:    0x%08x\n", stat);
	seq_printf(s, "  IRQ pending:   %s\n", str_yes_no(stat & ADI_SMPU_STAT_IRQ));
	seq_printf(s, "  Bus error:     %s\n", str_yes_no(stat & ADI_SMPU_STAT_BERR));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_status);

static int smpu_regions_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	int i;

	seq_printf(s, "%-6s %-10s %-12s %-10s %-6s %-8s %-8s\n",
		   "Region", "Enabled", "Base", "Size", "Perms", "IDA", "IDB");
	seq_puts(s, "------------------------------------------------------------\n");

	spin_lock_irqsave(&inst->lock, flags);

	for (i = 0; i < ADI_SMPU_NUM_REGIONS; i++) {
		u32 rctl = adi_smpu_readl(inst, ADI_SMPU_RCTL(i));
		u32 raddr = adi_smpu_readl(inst, ADI_SMPU_RADDR(i));
		u32 rida = adi_smpu_readl(inst, ADI_SMPU_RIDA(i));
		u32 ridb = adi_smpu_readl(inst, ADI_SMPU_RIDB(i));

		if (rctl & ADI_SMPU_RCTL_EN) {
			int size_bits = (rctl & ADI_SMPU_RCTL_SIZE_MASK) >>
					ADI_SMPU_RCTL_SIZE_SHIFT;
			u64 size = 1ULL << (size_bits + 12);
			phys_addr_t base = raddr & ADI_SMPU_RADDR_MASK;
			char perms[3] = "--";

			if (rctl & ADI_SMPU_RCTL_RPROTEN)
				perms[0] = 'R';
			if (rctl & ADI_SMPU_RCTL_WPROTEN)
				perms[1] = 'W';

			seq_printf(s, "%-6d %-10s 0x%08llx 0x%-8llx %-6s 0x%04x 0x%04x\n",
				   i, "yes", (u64)base, size, perms,
				   (u32)(rida & ADI_SMPU_TID_MASK),
				   (u32)(ridb & ADI_SMPU_TID_MASK));
		} else {
			seq_printf(s, "%-6d %-10s\n", i, "no");
		}
	}

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_regions);

static int smpu_violations_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;
	unsigned long flags;
	unsigned int i, count, start;

	spin_lock_irqsave(&inst->lock, flags);

	seq_printf(s, "Total violations: %lu\n\n", inst->viol_count);

	if (inst->viol_count == 0) {
		spin_unlock_irqrestore(&inst->lock, flags);
		seq_puts(s, "No violations recorded\n");
		return 0;
	}

	seq_printf(s, "%-10s %-12s %-8s %-6s %-8s\n",
		   "Time", "Address", "ID", "Type", "Secure");
	seq_puts(s, "---------------------------------------------------\n");

	count = min_t(unsigned long, inst->viol_count, ADI_SMPU_VIOL_HISTORY_SIZE);
	if (inst->viol_count > ADI_SMPU_VIOL_HISTORY_SIZE)
		start = inst->viol_head % ADI_SMPU_VIOL_HISTORY_SIZE;
	else
		start = 0;

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % ADI_SMPU_VIOL_HISTORY_SIZE;
		struct adi_smpu_violation *viol = &inst->violations[idx];
		unsigned long age = jiffies - viol->timestamp;

		seq_printf(s, "%-10lu 0x%08x 0x%04x %-6s %-8s\n",
			   age / HZ, viol->addr, viol->id,
			   viol->is_write ? "write" : "read",
			   viol->is_secure ? "yes" : "no");
	}

	spin_unlock_irqrestore(&inst->lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_violations);

static int smpu_summary_show(struct seq_file *s, void *data)
{
	struct adi_smpu *smpu = s->private;
	unsigned long total_violations = 0;
	int i;

	seq_puts(s, "ADSP-SC59x System Memory Protection Unit\n");
	seq_printf(s, "Instances: %d\n", smpu->num_instances);

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		unsigned long flags;

		spin_lock_irqsave(&inst->lock, flags);
		total_violations += inst->viol_count;
		spin_unlock_irqrestore(&inst->lock, flags);
	}

	seq_printf(s, "Total violations: %lu\n", total_violations);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_summary);

static void adi_smpu_debugfs_remove(void *data)
{
	struct adi_smpu *smpu = data;

	debugfs_remove_recursive(smpu->debugfs_root);
}

static void adi_smpu_debugfs_init(struct adi_smpu *smpu)
{
	struct dentry *root;
	int i;

	root = debugfs_create_dir(dev_name(smpu->dev), NULL);
	smpu->debugfs_root = root;

	if (devm_add_action_or_reset(smpu->dev, adi_smpu_debugfs_remove, smpu))
		return;

	debugfs_create_file("summary", 0444, root, smpu, &smpu_summary_fops);

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		struct dentry *dir = debugfs_create_dir(inst->name, root);

		debugfs_create_file("status", 0444, dir, inst, &smpu_status_fops);
		debugfs_create_file("regions", 0444, dir, inst, &smpu_regions_fops);
		debugfs_create_file("violations", 0444, dir, inst,
				    &smpu_violations_fops);
	}
}

static int adi_smpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_smpu *smpu;
	int ret, irq, i;

	smpu = devm_kzalloc(dev, sizeof(*smpu), GFP_KERNEL);
	if (!smpu)
		return -ENOMEM;

	smpu->dev = dev;
	platform_set_drvdata(pdev, smpu);

	/* Map every SMPU instance described in reg. */
	for (i = 0; i < ADI_SMPU_MAX_INSTANCES; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		void __iomem *base;

		/* Stop at the first missing reg entry; the rest are optional. */
		if (!platform_get_resource(pdev, IORESOURCE_MEM, i))
			break;

		base = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(base))
			return dev_err_probe(dev, PTR_ERR(base),
					     "failed to map SMPU instance %d\n", i);

		inst->base = base;
		inst->smpu = smpu;
		inst->name = devm_kasprintf(dev, GFP_KERNEL, "smpu%d", i);
		if (!inst->name)
			return -ENOMEM;
		spin_lock_init(&inst->lock);
	}

	smpu->num_instances = i;
	if (smpu->num_instances == 0)
		return dev_err_probe(dev, -ENODEV, "no SMPU instances found\n");

	/*
	 * The shared violation interrupt (SEC ID 217) covers all instances.
	 * The protection policy itself is programmed by secure firmware; the
	 * driver only reports the violations it raises.
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, adi_smpu_irq_handler, 0,
			       dev_name(dev), smpu);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ %d\n", irq);

	for (i = 0; i < smpu->num_instances; i++)
		adi_smpu_enable_irq(&smpu->instances[i]);

	/* Mask all violation interrupts before the handler is freed. */
	ret = devm_add_action_or_reset(dev, adi_smpu_disable_irqs, smpu);
	if (ret)
		return ret;

	adi_smpu_debugfs_init(smpu);

	return 0;
}

static const struct of_device_id adi_smpu_match[] = {
	{ .compatible = "adi,sc59x-smpu" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_smpu_match);

static struct platform_driver adi_smpu_driver = {
	.probe = adi_smpu_probe,
	.driver = {
		.name = "adi-sc59x-smpu",
		.of_match_table = adi_smpu_match,
	},
};
module_platform_driver(adi_smpu_driver);

MODULE_DESCRIPTION("System Memory Protection Unit for ADI SC59x SoCs");
MODULE_AUTHOR("Ozan Durgut <ozan.durgut@analog.com>");
MODULE_LICENSE("GPL");
