/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_FSL_SIP_H
#define __SOC_FSL_SIP_H

/* SIP 0xC2000000 - 0xC200FFFF */
#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_MASK		0x00
#define FSL_SIP_CONFIG_GPC_UNMASK	0x01
#define FSL_SIP_CONFIG_GPC_SET_WAKE	0x02
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x03

#define FSL_SIP_CPUFREQ			0xC2000001
#define FSL_SIP_SET_CPUFREQ		0x00

#define IMX8MQ_PD_MIPI		0
#define IMX8MQ_PD_PCIE1		1
#define IMX8MQ_PD_OTG1		2
#define IMX8MQ_PD_OTG2		3
#define IMX8MQ_PD_GPU		4
#define IMX8MQ_PD_VPU		5
#define IMX8MQ_PD_HDMI		6
#define IMX8MQ_PD_DISP		7
#define IMX8MQ_PD_MIPI_CSI1	8
#define IMX8MQ_PD_MIPI_CSI2	9
#define IMX8MQ_PD_PCIE2		10

#endif
