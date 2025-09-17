/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef SOC_ADI_ADI_SYSTEM_CONFIG_H
#define SOC_ADI_ADI_SYSTEM_CONFIG_H

#include <linux/soc/adi/system_config.h>

/*
 * All possible system register IDs across all platforms supported by this driver
 */
enum adi_system_reg_id {
	ADI_SYSTEM_REG_EMAC0_PTPCLK0 = 0,	/* PTP Clock Source 0 */
	ADI_SYSTEM_REG_EMAC0_EMACRESET,	/* Reset Enable for RGMII */
	ADI_SYSTEM_REG_EMAC0_PHYISEL,	/* Select PHY Interface RGMII/RMII/MII */
	ADI_SYSTEM_REG_CNT0UDSEL,	/* CNT0 Down Input Select */
	ADI_SYSTEM_REG_CNT0DGSEL,	/* CNT0 Up Input Select */
	ADI_SYSTEM_REG_TWI0VSEL,	/* TWI2 Voltage Select */
	ADI_SYSTEM_REG_TWI1VSEL,	/* TWI1 Voltage Select */
	ADI_SYSTEM_REG_TWI2VSEL,	/* TWI0 Voltage Select */
	ADI_SYSTEM_REG_PUMSIDLC,	/* Pull-Up Enable for MSI DATA[3:0] bits and CMD Pin */
	ADI_SYSTEM_REG_PUMSIHL,	/* Pull-Up Enable for MSI DATA[7:4] bits */
	ADI_SYSTEM_REG_PUTMS,	/* Pull-Up Enable for MSI DATA[7:4] bits */
	ADI_SYSTEM_REG_EMAC0_AUXIE,	/* Input enable control for PTP_AUXIN pins */
	ADI_SYSTEM_REG_FAULT_DIS,	/* FAULT does not exist */
	ADI_SYSTEM_REG_EMAC0_ENDIANNESS,	/* EMAC0 DMA transfer endian format */
	ADI_SYSTEM_REG_EMAC1_ENDIANNESS,	/* EMAC1 DMA transfer endian format */
	ADI_SYSTEM_REG_MSHC_CCLK_DIV_EN,	/* Enable MSHC Card Clock Divider */
	ADI_SYSTEM_REG_DAI0_IE,	/* Port input enable for DAI0 */
	ADI_SYSTEM_REG_DAI1_IE,	/* Port input enable for DAI1 */
	__ADI_SYSTEM_REG_COUNT
};

#endif
