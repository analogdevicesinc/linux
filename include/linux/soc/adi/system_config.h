/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implementation of adi_system_config, potential replacement for syscon that
 * generalizes it to support arbitrary regmap registration and requires the
 * driver to be initialized first
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#ifndef SOC_ADI_SYSTEM_CONFIG_H
#define SOC_ADI_SYSTEM_CONFIG_H

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct adi_system_register {
	u32 id;
	u32 offset;
	u32 mask;
	u8 shift;
	bool is_bits;
};

struct adi_system_config {
	/* User configured */
	struct adi_system_register *registers;
	unsigned int max_register;
	size_t len;

	/* Internal data populated during usage */
	const struct regmap_config config;
	struct regmap *mmio_regmap;
	struct device_node *np;
	struct list_head list;
	struct regmap *system_regmap;
};

/*
 * All possible system register IDs across all platforms supported by this
 * driver.
 */
enum adi_system_reg_id {
	ADI_SYSTEM_REG_EMAC0_PTPCLK0 = 0,
	ADI_SYSTEM_REG_EMAC0_EMACRESET,
	ADI_SYSTEM_REG_EMAC0_PHYISEL,
	ADI_SYSTEM_REG_CNT0UDSEL,
	ADI_SYSTEM_REG_CNT0DGSEL,
	ADI_SYSTEM_REG_TWI0VSEL,
	ADI_SYSTEM_REG_TWI1VSEL,
	ADI_SYSTEM_REG_TWI2VSEL,
	ADI_SYSTEM_REG_PUMSIDLC,
	ADI_SYSTEM_REG_PUMSIHL,
	ADI_SYSTEM_REG_PUTMS,
	ADI_SYSTEM_REG_EMAC0_AUXIE,
	ADI_SYSTEM_REG_FAULT_DIS,
	ADI_SYSTEM_REG_EMAC0_ENDIANNESS,
	ADI_SYSTEM_REG_EMAC1_ENDIANNESS,
	ADI_SYSTEM_REG_MSHC_CCLK_DIV_EN,
	ADI_SYSTEM_REG_DAI0_IE,
	ADI_SYSTEM_REG_DAI1_IE,
	__ADI_SYSTEM_REG_COUNT
};

#endif
