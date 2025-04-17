// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * PADS-related system config register driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Author: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adi_system_config.h>
#include <linux/soc/adi/pads_system.h>

#define ADI_SYSREG_BITS(_id, _offset, _width, _shift) \
	{ \
		.id = ADI_SYSTEM_REG_##_id, \
		.offset = _offset, \
		.mask = GENMASK(_width-1, 0) << _shift, \
		.shift = _shift, \
		.is_bits = true, \
	}

#define ADI_SYSREG(_id, _offset) \
	{ \
		.id = ADI_SYSTEM_REG_##_id, \
		.offset = _offset, \
		.is_bits = false, \
	}

// Fields in PADS CFG0 at offset +0x04
static struct system_register adi_pads_regs[] = {
	// PTP Clock Source 0
	ADI_SYSREG_BITS(EMAC0_PTPCLK0, 0x04, 2, 0),
	// Reset Enable for RGMII
	ADI_SYSREG_BITS(EMAC0_EMACRESET, 0x04, 1, 2),
	// Select PHY Interface RGMII/RMII/MII
	ADI_SYSREG_BITS(EMAC0_PHYISEL, 0x04, 2, 3),
	// CNT0 Down Input Select
	ADI_SYSREG_BITS(CNT0UDSEL, 0x04, 2, 6),
	// CNT0 Up Input Select
	ADI_SYSREG_BITS(CNT0DGSEL, 0x04, 2, 7),

#if defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
	// TWI2 Voltage Select
	ADI_SYSREG_BITS(TWI0VSEL, 0x04, 2, 8),
	// TWI1 Voltage Select
	ADI_SYSREG_BITS(TWI1VSEL, 0x04, 2, 9),
	// TWI0 Voltage Select
	ADI_SYSREG_BITS(TWI2VSEL, 0x04, 2, 10),
#endif

#if defined(CONFIG_ARCH_SC58X)
	// Pull-Up Enable for MSI DATA[3:0] bits and CMD Pin
	ADI_SYSREG_BITS(PUMSIDLC, 0x04, 2, 14),
	// Pull-Up Enable for MSI DATA[7:4] bits
	ADI_SYSREG_BITS(PUMSIHL, 0x04, 2, 15),
#endif

	// Pull-Up Enable for TMS/SWDIO (debug port)
	ADI_SYSREG_BITS(PUTMS, 0x04, 2, 16),
	// Input enable control for PTP_AUXIN pins
	ADI_SYSREG_BITS(EMAC0_AUXIE, 0x04, 1, 17),
	// FAULT does not exist
	ADI_SYSREG_BITS(FAULT_DIS, 0x04, 1, 18),

#if defined(CONFIG_ARCH_SC59X_64)
	// EMAC0 DMA transfer endian format
	ADI_SYSREG_BITS(EMAC0_ENDIANNESS, 0x04, 1, 19),
	// EMAC1 DMA transfer endian format
	ADI_SYSREG_BITS(EMAC1_ENDIANNESS, 0x04, 1, 20),
	// Enable MSHC Card Clock Divider
	ADI_SYSREG_BITS(MSHC_CCLK_DIV_EN, 0x04, 1, 22),
#endif

// DAIn port input enable registers
#if defined(CONFIG_ARCH_SC58X)
	ADI_SYSREG(DAI0_IE, 0x60),
	ADI_SYSREG(DAI1_IE, 0x64),
#endif

#if defined(CONFIG_ARCH_SC59X_64) || defined(CONFIG_ARCH_SC59X)
	ADI_SYSREG(DAI0_IE, 0x90),
	ADI_SYSREG(DAI1_IE, 0x94),
#endif
};

static struct system_config adi_pads_config = {
	.registers = adi_pads_regs,
	.len = ARRAY_SIZE(adi_pads_regs),
	.max_register = __ADI_SYSTEM_REG_COUNT,
};

int adi_pads_probe(struct platform_device *pdev)
{
	return system_config_probe(pdev, &adi_pads_config);
}

void adi_pads_remove(struct platform_device *pdev)
{
	system_config_remove(pdev);
}

static const struct of_device_id pads_dt_ids[] = {
	{ .compatible = "adi,pads-system-config", },
	{ }
};
MODULE_DEVICE_TABLE(of, pads_dt_ids);

static struct platform_driver pads_driver = {
	.driver = {
		.name = "adi-pads-system-config",
		.of_match_table = pads_dt_ids,
	},
	.probe = adi_pads_probe,
	.remove = adi_pads_remove,
};
module_platform_driver(pads_driver);

MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_DESCRIPTION("ADI ADSP PADS CFG-based System Configuration Driver");
MODULE_LICENSE("GPL");
