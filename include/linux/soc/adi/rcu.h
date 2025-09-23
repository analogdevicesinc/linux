/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#ifndef SOC_ADI_RCU_H
#define SOC_ADI_RCU_H

/* Register offsets */
#define ADI_RCU_REG_CTL				0x00
#define ADI_RCU_REG_STAT			0x04
#define ADI_RCU_REG_CRCTL			0x08
#define ADI_RCU_REG_CRSTAT			0x0c

/* Register bit definitions */
#define ADI_RCU_CTL_SYSRST		BIT(0)

#endif
