// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * System Memory Protection Unit (SMPU) for ADI SC59x processors
 *
 * Copyright (C) 2025 - Analog Devices, Inc.
 * Author: Ozan Durgut <ozan.durgut@analog.com>
 */

#include <linux/align.h>
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
#include <linux/log2.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/sprintf.h>
#include <linux/string_choices.h>
#include <linux/types.h>

/* Register offsets */
#define ADI_SMPU_REG_CTL		0x00
#define ADI_SMPU_REG_STAT		0x04
#define ADI_SMPU_REG_IADDR		0x08
#define ADI_SMPU_REG_IDTLS		0x0C
#define ADI_SMPU_REG_BADDR		0x10
#define ADI_SMPU_REG_BDTLS		0x14

/* Region configuration registers (stride = 0x18 per region) */
#define ADI_SMPU_RCTL(n)		(0x20 + (n) * 0x18)
#define ADI_SMPU_RADDR(n)		(0x24 + (n) * 0x18)
#define ADI_SMPU_RIDA(n)		(0x28 + (n) * 0x18)
#define ADI_SMPU_RIDMSKA(n)		(0x2C + (n) * 0x18)
#define ADI_SMPU_RIDB(n)		(0x30 + (n) * 0x18)
#define ADI_SMPU_RIDMSKB(n)		(0x34 + (n) * 0x18)

/* Security registers (SMPU_SECURE block, Anomaly 20000003 affects 0x800-0xFFF) */
#define ADI_SMPU_SECURE_REG_BASE	0x800
#define ADI_SMPU_REG_SECURECTL		ADI_SMPU_SECURE_REG_BASE
#define ADI_SMPU_SECURERCTL(n)		(0x820 + (n) * 0x04)

/* SMPU_SECURECTL register bits */
#define ADI_SMPU_SECURECTL_WSECDIS	BIT(11)
#define ADI_SMPU_SECURECTL_WNSEN	BIT(10)
#define ADI_SMPU_SECURECTL_RSECDIS	BIT(9)
#define ADI_SMPU_SECURECTL_RNSEN	BIT(8)
#define ADI_SMPU_SECURECTL_SINTEN	BIT(2)

/* SMPU_SECURERCTL[n] register bits */
#define ADI_SMPU_SECURERCTL_WSECDIS	BIT(3)
#define ADI_SMPU_SECURERCTL_WNSEN	BIT(2)
#define ADI_SMPU_SECURERCTL_RSECDIS	BIT(1)
#define ADI_SMPU_SECURERCTL_RNSEN	BIT(0)

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

/* SMPU_RCTL register bits */
#define ADI_SMPU_RCTL_WIDCINV		BIT(11)
#define ADI_SMPU_RCTL_WPROTEN		BIT(10)
#define ADI_SMPU_RCTL_RIDCINV		BIT(9)
#define ADI_SMPU_RCTL_RPROTEN		BIT(8)
#define ADI_SMPU_RCTL_SIZE_SHIFT	1
#define ADI_SMPU_RCTL_SIZE_MASK		GENMASK(5, 1)
#define ADI_SMPU_RCTL_EN		BIT(0)

/* SMPU_IDTLS/BDTLS register bits */
#define ADI_SMPU_DTLS_ID_SHIFT		8
#define ADI_SMPU_DTLS_ID_MASK		GENMASK(20, 8)
#define ADI_SMPU_DTLS_RW		BIT(1)
#define ADI_SMPU_DTLS_SECURE		BIT(0)

/* Transaction IDs are 13-bit */
#define ADI_SMPU_TID_MASK		GENMASK(12, 0)
/*
 * RADDR[n].BADDR is the region base address held in bits [31:12] IN PLACE
 * (the 20 MSBs); bits [11:0] are reserved 0. The address is NOT right-shifted
 * into the low bits - store base & GENMASK(31,12) directly.
 */
#define ADI_SMPU_RADDR_MASK		GENMASK(31, 12)

/* Maximum number of SMPU instances */
#define ADI_SMPU_MAX_INSTANCES		9

/* Number of protection regions per SMPU instance */
#define ADI_SMPU_NUM_REGIONS		8

/* Violation history buffer size per instance */
#define ADI_SMPU_VIOL_HISTORY_SIZE	32

/* Region access permissions (DT "permissions" bitmap) */
#define ADI_SMPU_PERM_READ		BIT(0)
#define ADI_SMPU_PERM_WRITE		BIT(1)

/* Secure access modes for a region (TrustZone), used when secure regs exist */
enum adi_smpu_secure_mode {
	ADI_SMPU_SEC_BOTH = 0,		/* allow secure and non-secure (default) */
	ADI_SMPU_SEC_SECURE_ONLY,	/* allow only secure transactions */
	ADI_SMPU_SEC_NONSECURE_ONLY,	/* allow only non-secure transactions */
	ADI_SMPU_SEC_NONE,		/* block all transactions */
};

/* Parsed configuration for one static protection region from the device tree */
struct adi_smpu_region_config {
	phys_addr_t base;
	u64 size;
	u32 permissions;
	u16 allowed_ids[2];
	u16 id_masks[2];
	bool id_invert[2];
	enum adi_smpu_secure_mode secure_mode;
};

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
	int instance_id;
	const char *name;
	unsigned long region_bitmap;
	spinlock_t lock;		/* serialises register and history access */
	struct adi_smpu *smpu;
	struct adi_smpu_violation violations[ADI_SMPU_VIOL_HISTORY_SIZE];
	unsigned int viol_head;
	unsigned long viol_count;
	struct dentry *debugfs_dir;
};

struct adi_smpu {
	struct device *dev;
	struct adi_smpu_instance instances[ADI_SMPU_MAX_INSTANCES];
	int num_instances;
	struct dentry *debugfs_root;
	/* Can access the 0x800+ secure registers (see Anomaly 20000003). */
	bool secure_regs_available;
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

/*
 * Write to a secure register (offset >= ADI_SMPU_SECURE_REG_BASE).
 * Returns 0 on success, -EOPNOTSUPP if secure registers are unavailable
 * (Anomaly 20000003), or -EINVAL on a bad offset.
 */
static int adi_smpu_write_secure_reg(const struct adi_smpu_instance *inst,
				     u32 val, u32 offset)
{
	if (!inst->smpu->secure_regs_available) {
		dev_warn_ratelimited(inst->smpu->dev,
				     "Secure register access blocked (Anomaly 20000003)\n");
		return -EOPNOTSUPP;
	}

	if (offset < ADI_SMPU_SECURE_REG_BASE) {
		dev_err(inst->smpu->dev, "Invalid secure register offset 0x%x\n", offset);
		return -EINVAL;
	}

	adi_smpu_writel(inst, val, offset);
	return 0;
}

/*
 * Read from a secure register (offset >= ADI_SMPU_SECURE_REG_BASE).
 * Returns 0 on success, -EOPNOTSUPP if secure registers are unavailable,
 * or -EINVAL on a bad offset.
 */
static int adi_smpu_read_secure_reg(const struct adi_smpu_instance *inst,
				    u32 *val, u32 offset)
{
	if (!inst->smpu->secure_regs_available)
		return -EOPNOTSUPP;

	if (offset < ADI_SMPU_SECURE_REG_BASE)
		return -EINVAL;

	*val = adi_smpu_readl(inst, offset);
	return 0;
}

/* Look up an SMPU instance by its hardware instance ID. */
static struct adi_smpu_instance *
adi_smpu_get_instance(struct adi_smpu *smpu, int instance_id)
{
	int i;

	for (i = 0; i < smpu->num_instances; i++) {
		if (smpu->instances[i].instance_id == instance_id)
			return &smpu->instances[i];
	}

	return NULL;
}

static int adi_smpu_calc_securerctl(enum adi_smpu_secure_mode mode, u32 *out);

/*
 * Encode a region size as the power-of-2 bits used by the hardware:
 * 4KB -> 0 (1 << 12) ... 4GB -> 20 (1 << 32).  Returns the size bits or
 * a negative error if the size is out of range or not a power of two.
 */
static int adi_smpu_calc_size_bits(u64 size)
{
	int bits;

	if (size < SZ_4K || size > SZ_4G)
		return -EINVAL;

	if (!is_power_of_2(size))
		return -EINVAL;

	bits = ilog2(size) - 12;
	if (bits < 0 || bits > 20)
		return -EINVAL;

	return bits;
}

/*
 * Validate a region configuration and return the encoded size bits, or a
 * negative error.  Callers reuse the returned value instead of recomputing it.
 */
static int adi_smpu_validate_region(const struct adi_smpu_region_config *config)
{
	int size_bits;

	if (!config)
		return -EINVAL;

	size_bits = adi_smpu_calc_size_bits(config->size);
	if (size_bits < 0)
		return size_bits;

	/*
	 * RADDR only holds bits [31:12], so the base must fit in 32 bits;
	 * a higher base would be silently truncated and protect the wrong
	 * physical region.
	 */
	if (config->base > U32_MAX)
		return -EINVAL;

	if (!IS_ALIGNED(config->base, config->size))
		return -EINVAL;

	return size_bits;
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

		/* Acknowledge every pending W1C status bit. */
		adi_smpu_writel(inst, status & ADI_SMPU_STAT_W1C_MASK,
				ADI_SMPU_REG_STAT);
		ret = IRQ_HANDLED;

		spin_unlock(&inst->lock);
	}

	return ret;
}

/*
 * Reset an instance to a safe state: disable all regions, clear pending
 * violations and mask the violation interrupt.  Runs during probe before the
 * IRQ is requested, so no locking is needed - there is nothing to race with.
 */
static void adi_smpu_reset_instance(struct adi_smpu_instance *inst)
{
	u32 val;
	int i;

	/* Disable all regions */
	for (i = 0; i < ADI_SMPU_NUM_REGIONS; i++)
		adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(i));

	/* Clear any pending violations */
	adi_smpu_writel(inst, ADI_SMPU_STAT_W1C_MASK, ADI_SMPU_REG_STAT);

	/* Leave violation interrupts disabled until an IRQ is registered. */
	val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);
	val &= ~ADI_SMPU_CTL_PINTEN;
	adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);
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

/* Region configuration (driven from the device tree at probe) */

static int adi_smpu_configure_region(struct adi_smpu *smpu, int instance_id,
				     int region_num,
				     const struct adi_smpu_region_config *config)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 rctl, raddr;
	int size_bits;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	/* Validate once and reuse the encoded size bits below. */
	size_bits = adi_smpu_validate_region(config);
	if (size_bits < 0)
		return size_bits;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	/* Disable region during configuration */
	adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(region_num));

	/* Configure base address (held in bits [31:12] in place, not shifted) */
	raddr = config->base & ADI_SMPU_RADDR_MASK;
	adi_smpu_writel(inst, raddr, ADI_SMPU_RADDR(region_num));

	/* Configure transaction ID filters */
	adi_smpu_writel(inst, config->allowed_ids[0] & ADI_SMPU_TID_MASK,
			ADI_SMPU_RIDA(region_num));
	adi_smpu_writel(inst, config->id_masks[0] & ADI_SMPU_TID_MASK,
			ADI_SMPU_RIDMSKA(region_num));
	adi_smpu_writel(inst, config->allowed_ids[1] & ADI_SMPU_TID_MASK,
			ADI_SMPU_RIDB(region_num));
	adi_smpu_writel(inst, config->id_masks[1] & ADI_SMPU_TID_MASK,
			ADI_SMPU_RIDMSKB(region_num));

	/* Configure control register */
	rctl = (size_bits << ADI_SMPU_RCTL_SIZE_SHIFT) & ADI_SMPU_RCTL_SIZE_MASK;

	if (config->permissions & ADI_SMPU_PERM_READ)
		rctl |= ADI_SMPU_RCTL_RPROTEN;
	if (config->permissions & ADI_SMPU_PERM_WRITE)
		rctl |= ADI_SMPU_RCTL_WPROTEN;
	if (config->id_invert[0])
		rctl |= ADI_SMPU_RCTL_RIDCINV;
	if (config->id_invert[1])
		rctl |= ADI_SMPU_RCTL_WIDCINV;

	/* Write control register (region still disabled) */
	adi_smpu_writel(inst, rctl, ADI_SMPU_RCTL(region_num));

	/*
	 * Program per-region secure access when the secure registers are
	 * available. This must run for every mode including the default
	 * SEC_BOTH: SECURERCTL resets to 0 (non-secure read/write disabled),
	 * so leaving it unwritten would block all non-secure transactions
	 * even when the intent is to allow both. When the secure registers
	 * are unavailable this is skipped and transaction-ID filtering above
	 * still applies.
	 */
	if (inst->smpu->secure_regs_available) {
		u32 securerctl;

		if (!adi_smpu_calc_securerctl(config->secure_mode, &securerctl))
			adi_smpu_writel(inst, securerctl,
					ADI_SMPU_SECURERCTL(region_num));
	}

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_dbg(smpu->dev, "%s: configured region %d: base=0x%llx size=0x%llx\n",
		inst->name, region_num, (u64)config->base, config->size);

	return 0;
}

/* Translate a secure mode into the SECURERCTL[n] register value. */
static int adi_smpu_calc_securerctl(enum adi_smpu_secure_mode mode, u32 *out)
{
	switch (mode) {
	case ADI_SMPU_SEC_BOTH:
		/* Allow secure (default) and non-secure. */
		*out = ADI_SMPU_SECURERCTL_RNSEN | ADI_SMPU_SECURERCTL_WNSEN;
		break;
	case ADI_SMPU_SEC_SECURE_ONLY:
		/* Secure is enabled out of reset; block non-secure. */
		*out = 0;
		break;
	case ADI_SMPU_SEC_NONSECURE_ONLY:
		*out = ADI_SMPU_SECURERCTL_RNSEN | ADI_SMPU_SECURERCTL_WNSEN |
		       ADI_SMPU_SECURERCTL_RSECDIS | ADI_SMPU_SECURERCTL_WSECDIS;
		break;
	case ADI_SMPU_SEC_NONE:
		*out = ADI_SMPU_SECURERCTL_RSECDIS | ADI_SMPU_SECURERCTL_WSECDIS;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int adi_smpu_enable_region(struct adi_smpu *smpu, int instance_id,
				  int region_num)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	u32 rctl;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return -EINVAL;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	rctl = adi_smpu_readl(inst, ADI_SMPU_RCTL(region_num));
	rctl |= ADI_SMPU_RCTL_EN;
	adi_smpu_writel(inst, rctl, ADI_SMPU_RCTL(region_num));

	spin_unlock_irqrestore(&inst->lock, flags);

	dev_dbg(smpu->dev, "%s: enabled region %d\n", inst->name, region_num);

	return 0;
}

static int adi_smpu_allocate_region(struct adi_smpu *smpu, int instance_id)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;
	int region;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return -EINVAL;

	spin_lock_irqsave(&inst->lock, flags);

	region = find_first_zero_bit(&inst->region_bitmap, ADI_SMPU_NUM_REGIONS);
	if (region >= ADI_SMPU_NUM_REGIONS) {
		spin_unlock_irqrestore(&inst->lock, flags);
		return -ENOSPC;
	}

	set_bit(region, &inst->region_bitmap);

	spin_unlock_irqrestore(&inst->lock, flags);

	return region;
}

static void adi_smpu_free_region(struct adi_smpu *smpu, int instance_id,
				 int region_num)
{
	struct adi_smpu_instance *inst;
	unsigned long flags;

	if (region_num < 0 || region_num >= ADI_SMPU_NUM_REGIONS)
		return;

	inst = adi_smpu_get_instance(smpu, instance_id);
	if (!inst)
		return;

	spin_lock_irqsave(&inst->lock, flags);

	/* Disable region and mark it free. */
	adi_smpu_writel(inst, 0, ADI_SMPU_RCTL(region_num));
	clear_bit(region_num, &inst->region_bitmap);

	spin_unlock_irqrestore(&inst->lock, flags);
}

/* Debugfs interface */

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
	seq_printf(s, "  Locked:        %s\n", ctl & ADI_SMPU_CTL_LOCK ? "yes" : "no");
	seq_printf(s, "  Region locked: %s\n", ctl & ADI_SMPU_CTL_RLOCK ? "yes" : "no");
	seq_printf(s, "  IRQ enabled:   %s\n", ctl & ADI_SMPU_CTL_PINTEN ? "yes" : "no");
	seq_printf(s, "\nStatus:    0x%08x\n", stat);
	seq_printf(s, "  IRQ pending:   %s\n", stat & ADI_SMPU_STAT_IRQ ? "yes" : "no");
	seq_printf(s, "  Bus error:     %s\n", stat & ADI_SMPU_STAT_BERR ? "yes" : "no");

	/* Show secure control (if available) */
	if (inst->smpu->secure_regs_available) {
		u32 securectl;
		int ret = adi_smpu_read_secure_reg(inst, &securectl, ADI_SMPU_REG_SECURECTL);

		if (ret == 0) {
			seq_printf(s, "\nSECURECTL: 0x%08x\n", securectl);
			seq_printf(s, "  Secure reads:      %s\n",
				   str_disabled_enabled(securectl & ADI_SMPU_SECURECTL_RSECDIS));
			seq_printf(s, "  Secure writes:     %s\n",
				   str_disabled_enabled(securectl & ADI_SMPU_SECURECTL_WSECDIS));
			seq_printf(s, "  Non-secure reads:  %s\n",
				   str_enabled_disabled(securectl & ADI_SMPU_SECURECTL_RNSEN));
			seq_printf(s, "  Non-secure writes: %s\n",
				   str_enabled_disabled(securectl & ADI_SMPU_SECURECTL_WNSEN));
			seq_printf(s, "  Security IRQ:      %s\n",
				   str_enabled_disabled(securectl & ADI_SMPU_SECURECTL_SINTEN));
		}
	}

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
			char perms[4] = "---";

			if (rctl & ADI_SMPU_RCTL_RPROTEN)
				perms[0] = 'R';
			if (rctl & ADI_SMPU_RCTL_WPROTEN)
				perms[1] = 'W';

			seq_printf(s, "%-6d %-10s 0x%08llx 0x%-8llx %-6s 0x%04x 0x%04x",
				   i, "yes", (u64)base, size, perms,
				   (u32)(rida & ADI_SMPU_TID_MASK),
				   (u32)(ridb & ADI_SMPU_TID_MASK));

			/* Show secure control for this region (if available) */
			if (inst->smpu->secure_regs_available) {
				u32 sr;

				if (!adi_smpu_read_secure_reg(inst, &sr,
							      ADI_SMPU_SECURERCTL(i)))
					seq_printf(s, " | Sec:%c%c NS:%c%c",
						   (sr & ADI_SMPU_SECURERCTL_RSECDIS) ? '-' : 'R',
						   (sr & ADI_SMPU_SECURERCTL_WSECDIS) ? '-' : 'W',
						   (sr & ADI_SMPU_SECURERCTL_RNSEN) ? 'R' : '-',
						   (sr & ADI_SMPU_SECURERCTL_WNSEN) ? 'W' : '-');
			}
			seq_puts(s, "\n");
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

	seq_printf(s, "Total violations: %lu\n\n", inst->viol_count);

	if (inst->viol_count == 0) {
		seq_puts(s, "No violations recorded\n");
		return 0;
	}

	seq_printf(s, "%-10s %-12s %-8s %-6s %-8s\n",
		   "Time", "Address", "ID", "Type", "Secure");
	seq_puts(s, "---------------------------------------------------\n");

	spin_lock_irqsave(&inst->lock, flags);

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

static int smpu_stats_show(struct seq_file *s, void *data)
{
	struct adi_smpu_instance *inst = s->private;

	seq_printf(s, "Total violations: %lu\n", inst->viol_count);
	seq_printf(s, "History size:     %u\n", ADI_SMPU_VIOL_HISTORY_SIZE);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_stats);

static int smpu_instances_show(struct seq_file *s, void *data)
{
	struct adi_smpu *smpu = s->private;
	int i;

	seq_printf(s, "Total instances: %d\n\n", smpu->num_instances);
	seq_printf(s, "%-10s %-10s\n", "Name", "ID");
	seq_puts(s, "--------------------\n");

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];

		seq_printf(s, "%-10s %-10d\n", inst->name, inst->instance_id);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_instances);

static int smpu_summary_show(struct seq_file *s, void *data)
{
	struct adi_smpu *smpu = s->private;
	unsigned long total_violations = 0;
	int i;

	seq_puts(s, "ADSP-SC59x System Memory Protection Unit\n");
	seq_printf(s, "Instances: %d\n", smpu->num_instances);
	seq_printf(s, "Secure registers: %s\n",
		   smpu->secure_regs_available ? "available" : "unavailable (Errata, 20000003)");
	seq_puts(s, "\n");

	for (i = 0; i < smpu->num_instances; i++)
		total_violations += smpu->instances[i].viol_count;

	seq_printf(s, "Total violations: %lu\n", total_violations);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(smpu_summary);

static void adi_smpu_debugfs_remove(void *data)
{
	struct adi_smpu *smpu = data;

	debugfs_remove_recursive(smpu->debugfs_root);
}

static int adi_smpu_debugfs_init(struct adi_smpu *smpu)
{
	struct dentry *root;
	int i, ret;

	root = debugfs_create_dir("smpu", NULL);
	smpu->debugfs_root = root;

	ret = devm_add_action_or_reset(smpu->dev, adi_smpu_debugfs_remove, smpu);
	if (ret)
		return ret;

	/* Everybody can read, nobody can write */
	debugfs_create_file("instances", 0444, root, smpu,
			    &smpu_instances_fops);
	debugfs_create_file("summary", 0444, root, smpu,
			    &smpu_summary_fops);

	for (i = 0; i < smpu->num_instances; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		struct dentry *inst_dir;

		inst_dir = debugfs_create_dir(inst->name, root);
		inst->debugfs_dir = inst_dir;

		debugfs_create_file("status", 0444, inst_dir, inst,
				    &smpu_status_fops);
		debugfs_create_file("regions", 0444, inst_dir, inst,
				    &smpu_regions_fops);
		debugfs_create_file("violations", 0444, inst_dir, inst,
				    &smpu_violations_fops);
		debugfs_create_file("stats", 0444, inst_dir, inst,
				    &smpu_stats_fops);
	}

	return 0;
}

/* Static region parsing from device tree */

static int adi_smpu_parse_static_regions(struct adi_smpu *smpu)
{
	struct device *dev = smpu->dev;
	struct device_node *np = dev->of_node;
	struct device_node *regions_node, *child;

	/* Static regions are optional. */
	regions_node = of_get_child_by_name(np, "static-regions");
	if (!regions_node)
		return 0;

	for_each_child_of_node(regions_node, child) {
		struct adi_smpu_region_config config = {0};
		struct device_node *mem_node;
		struct resource res;
		u32 instance, permissions;
		int region_num, ret;
		u32 allowed_ids[2] = {0};
		u32 id_masks[2] = {0};
		int id_count, mask_count;

		/* Parse instance ID */
		ret = of_property_read_u32(child, "instance", &instance);
		if (ret) {
			dev_err(dev, "Missing 'instance' property in region %pOF\n",
				child);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		/* Parse memory-region phandle to get base and size */
		mem_node = of_parse_phandle(child, "memory-region", 0);
		if (mem_node) {
			ret = of_address_to_resource(mem_node, 0, &res);
			of_node_put(mem_node);
			if (ret) {
				dev_err(dev, "Failed to parse memory-region for %pOF\n",
					child);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}
			config.base = res.start;
			config.size = resource_size(&res);
		} else {
			/*
			 * Fallback: direct base/size properties. These are
			 * single-cell (u32) values; SC59x protected regions all
			 * live in the 32-bit address space.
			 */
			u32 base, size;

			ret = of_property_read_u32(child, "base", &base);
			if (ret) {
				dev_err(dev, "Missing 'base' or 'memory-region' in %pOF\n",
					child);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}

			ret = of_property_read_u32(child, "size", &size);
			if (ret) {
				dev_err(dev, "Missing 'size' in %pOF\n", child);
				of_node_put(child);
				of_node_put(regions_node);
				return ret;
			}

			config.base = base;
			config.size = size;
		}

		/* Parse permissions (default to read+write if not specified) */
		ret = of_property_read_u32(child, "permissions", &permissions);
		if (ret)
			permissions = ADI_SMPU_PERM_READ | ADI_SMPU_PERM_WRITE;
		config.permissions = permissions;

		/* Parse allowed transaction IDs (max 2) */
		id_count = of_property_count_u32_elems(child, "allowed-ids");
		if (id_count > 0) {
			if (id_count > 2)
				id_count = 2;
			of_property_read_u32_array(child, "allowed-ids", allowed_ids, id_count);
			config.allowed_ids[0] = (u16)allowed_ids[0];
			if (id_count > 1)
				config.allowed_ids[1] = (u16)allowed_ids[1];
		}

		/* Parse ID masks (default to exact match) */
		mask_count = of_property_count_u32_elems(child, "id-masks");
		if (mask_count > 0) {
			if (mask_count > 2)
				mask_count = 2;
			of_property_read_u32_array(child, "id-masks", id_masks, mask_count);
			config.id_masks[0] = (u16)id_masks[0];
			if (mask_count > 1)
				config.id_masks[1] = (u16)id_masks[1];
		} else {
			config.id_masks[0] = ADI_SMPU_TID_MASK;
			config.id_masks[1] = ADI_SMPU_TID_MASK;
		}

		/*
		 * The hardware has no per-comparator enable: the ID bypass is
		 * (comparator-A match OR comparator-B match). If fewer than two
		 * allowed IDs are described, comparator B would be left at
		 * RIDB=0 with an exact mask and would spuriously match
		 * transaction ID 0 (a real bus master), silently whitelisting
		 * it. Mirror comparator A into B so the unused comparator does
		 * not open a hole.
		 */
		if (id_count < 2) {
			config.allowed_ids[1] = config.allowed_ids[0];
			config.id_masks[1] = config.id_masks[0];
		}

		/* Parse ID inversion (default to false(no inversion)) */
		config.id_invert[0] = of_property_read_bool(child, "adi,read-id-invert");
		config.id_invert[1] = of_property_read_bool(child, "adi,write-id-invert");

		/* Parse secure mode (default: allow both secure and non-secure) */
		if (of_property_read_bool(child, "secure-only"))
			config.secure_mode = ADI_SMPU_SEC_SECURE_ONLY;
		else if (of_property_read_bool(child, "non-secure-only"))
			config.secure_mode = ADI_SMPU_SEC_NONSECURE_ONLY;
		else
			config.secure_mode = ADI_SMPU_SEC_BOTH;

		/* Allocate a region for this configuration */
		region_num = adi_smpu_allocate_region(smpu, instance);
		if (region_num < 0) {
			dev_err(dev, "Failed to allocate region for %pOF on SMPU%d: %d\n",
				child, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return region_num;
		}

		/* Configure the region */
		ret = adi_smpu_configure_region(smpu, instance, region_num, &config);
		if (ret) {
			dev_err(dev, "Failed to configure region %d on SMPU%d: %d\n",
				region_num, instance, ret);
			adi_smpu_free_region(smpu, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		/* Enable the region */
		ret = adi_smpu_enable_region(smpu, instance, region_num);
		if (ret) {
			dev_err(dev, "Failed to enable region %d on SMPU%d: %d\n",
				region_num, instance, ret);
			adi_smpu_free_region(smpu, instance, region_num);
			of_node_put(child);
			of_node_put(regions_node);
			return ret;
		}

		dev_dbg(dev, "region %d: SMPU%d [%#llx-%#llx] perm=%#x ids=[%#x,%#x]\n",
			region_num, instance, (u64)config.base,
			(u64)(config.base + config.size - 1), config.permissions,
			config.allowed_ids[0], config.allowed_ids[1]);
	}

	of_node_put(regions_node);

	return 0;
}

/* Platform driver */

static int adi_smpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_smpu *smpu;
	int ret, irq, i;

	smpu = devm_kzalloc(dev, sizeof(*smpu), GFP_KERNEL);
	if (!smpu)
		return -ENOMEM;

	smpu->dev = dev;
	dev_set_drvdata(dev, smpu);

	/* Check if secure registers are accessible (Errata, 20000003) */
	smpu->secure_regs_available =
		of_property_read_bool(np, "adi,secure-access");

	/* Parse and map all SMPU instances from device tree */
	for (i = 0; i < ADI_SMPU_MAX_INSTANCES; i++) {
		struct adi_smpu_instance *inst = &smpu->instances[i];
		const char *name;
		void __iomem *base;
		int instance_id;

		base = devm_platform_ioremap_resource(pdev, i);
		if (IS_ERR(base)) {
			/* -EINVAL means there is no resource at this index. */
			if (PTR_ERR(base) == -EINVAL)
				break;
			return dev_err_probe(dev, PTR_ERR(base),
					     "Failed to map SMPU instance %d\n", i);
		}

		ret = of_property_read_string_index(np, "reg-names", i, &name);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing reg-names entry %d\n", i);

		/* Extract instance ID from name (like, "smpu0" -> 0) */
		if (sscanf(name, "smpu%d", &instance_id) != 1)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid reg-name: %s\n", name);

		inst->base = base;
		inst->instance_id = instance_id;
		inst->name = devm_kasprintf(dev, GFP_KERNEL, "smpu%d", instance_id);
		if (!inst->name)
			return -ENOMEM;
		inst->smpu = smpu;
		spin_lock_init(&inst->lock);
	}

	smpu->num_instances = i;

	if (smpu->num_instances == 0)
		return dev_err_probe(dev, -ENODEV, "No SMPU instances found\n");

	/* Reset all instances to safe state */
	for (i = 0; i < smpu->num_instances; i++)
		adi_smpu_reset_instance(&smpu->instances[i]);

	/* Configure global secure control if available */
	if (smpu->secure_regs_available) {
		for (i = 0; i < smpu->num_instances; i++) {
			struct adi_smpu_instance *inst = &smpu->instances[i];
			u32 securectl;

			/*
			 * Enable both secure and non-secure access by default;
			 * secure access is enabled out of reset.
			 */
			securectl = ADI_SMPU_SECURECTL_RNSEN |
				    ADI_SMPU_SECURECTL_WNSEN |
				    ADI_SMPU_SECURECTL_SINTEN;

			ret = adi_smpu_write_secure_reg(inst, securectl,
							ADI_SMPU_REG_SECURECTL);
			if (ret) {
				dev_warn(dev, "%s: failed to init SECURECTL: %d\n",
					 inst->name, ret);
				smpu->secure_regs_available = false;
				break;
			}
		}
	}

	/* Request the shared violation IRQ (SEC ID 217, all instances). */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	if (irq > 0) {
		ret = devm_request_irq(dev, irq, adi_smpu_irq_handler,
				       0, dev_name(dev), smpu);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to request IRQ %d\n", irq);

		for (i = 0; i < smpu->num_instances; i++) {
			struct adi_smpu_instance *inst = &smpu->instances[i];
			u32 val = adi_smpu_readl(inst, ADI_SMPU_REG_CTL);

			val |= ADI_SMPU_CTL_PINTEN;
			adi_smpu_writel(inst, val, ADI_SMPU_REG_CTL);
		}

		/* Mask all violation interrupts before the handler is freed. */
		ret = devm_add_action_or_reset(dev, adi_smpu_disable_irqs, smpu);
		if (ret)
			return ret;
	} else {
		dev_warn(dev, "No IRQ specified, violations will not be reported\n");
	}

	/* Parse and configure static regions from device tree */
	ret = adi_smpu_parse_static_regions(smpu);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to parse static regions\n");

	ret = adi_smpu_debugfs_init(smpu);
	if (ret)
		return ret;

	if (!smpu->secure_regs_available)
		dev_warn(dev, "Security registers unavailable (Anomaly 20000003)\n");

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
