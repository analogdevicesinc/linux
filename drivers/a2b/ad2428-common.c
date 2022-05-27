// SPDX-License-Identifier: GPL-2.0-only

#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b-regs.h>
#include "ad2428.h"

static bool ad2428_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case A2B_SLOTFMT:
	case A2B_DATCTL:
	case A2B_CONTROL:
	case A2B_SWSTAT:
	case A2B_INTSTAT:
	case A2B_INTSRC:
	case A2B_INTTYPE:
	case A2B_INTPND0:
	case A2B_INTPND1:
	case A2B_INTPND2:
	case A2B_BECNT:
	case A2B_ERRCNT(0)... A2B_ERRCNT(3):
	case A2B_NODE:
	case A2B_DISCSTAT:
	case A2B_LINTTYPE:
	case A2B_I2SRRATE:
	case A2B_GPIODAT_SET:
	case A2B_GPIODAT_CLR:
	case A2B_GPIODAT_IN:
	case A2B_RAISE:
	case A2B_GENERR:
	case A2B_MBOXSTAT(0):
	case A2B_MBOXDATA(0, 0)... A2B_MBOXDATA(0, 3):
	case A2B_MBOXSTAT(1):
	case A2B_MBOXDATA(1, 0)... A2B_MBOXDATA(1, 3):
		return true;
	default:
		return false;
	}
}

static bool ad2428_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	/* Write-to-{set,clear} registers */
	case A2B_INTPND0:
	case A2B_INTPND1:
	case A2B_INTPND2:
	case A2B_BECNT:
	case A2B_GPIODAT_SET:
	case A2B_GPIODAT_CLR:
		return true;
	/* Self-clearing bits registers */
	case A2B_CONTROL:
	case A2B_RAISE:
	case A2B_GENERR:
		return true;
	/* Broadcast registers */
	case A2B_SLOTFMT:
	case A2B_DATCTL:
	case A2B_I2SRRATE:
		return true;
	/* Mailbox data registers */
	case A2B_MBOXDATA(0, 0)... A2B_MBOXDATA(0, 3):
	case A2B_MBOXDATA(1, 0)... A2B_MBOXDATA(1, 3):
		return true;
	case A2B_VENDOR:
	case A2B_PRODUCT:
	case A2B_VERSION:
	case A2B_CAPABILITY:
	case A2B_CHIPID(0)... A2B_CHIPID(5):
		return false;
	default:
		return !ad2428_is_volatile_reg(dev, reg);
	}
}

const struct regmap_config ad2428_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = ad2428_is_volatile_reg,
	.writeable_reg = ad2428_is_writeable_reg,
	.max_register = A2B_MAX_REG,
	.cache_type = REGCACHE_RBTREE,
};

MODULE_DESCRIPTION("AD2428 mainnode driver");
MODULE_LICENSE("GPL v2");
