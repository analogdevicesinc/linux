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

#define FSL_SIP_SRTC			0xC2000002
#define FSL_SIP_SRTC_SET_TIME		0x00
#define FSL_SIP_SRTC_START_WDOG		0x01
#define FSL_SIP_SRTC_STOP_WDOG		0x02
#define FSL_SIP_SRTC_SET_WDOG_ACT	0x03
#define FSL_SIP_SRTC_PING_WDOG		0x04
#define FSL_SIP_SRTC_SET_TIMEOUT_WDOG	0x05
#define FSL_SIP_SRTC_GET_WDOG_STAT	0x06
#define FSL_SIP_SRTC_SET_PRETIME_WDOG	0x07

#define FSL_SIP_DDR_DVFS		0xc2000004

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

#define FSL_SIP_DDR_DVFS		0xc2000004

#define FSL_SIP_SRC			0xc2000005
#define FSL_SIP_SRC_M4_START		0x00
#define FSL_SIP_SRC_M4_STARTED		0x01

#define FSL_SIP_GET_SOC_INFO		0xc2000006

#define FSL_SIP_NOC			0xc2000008
#define FSL_SIP_NOC_LCDIF		0x0
#define FSL_SIP_NOC_PRIORITY		0x1
#define NOC_GPU_PRIORITY		0x10
#define NOC_DCSS_PRIORITY		0x11
#define NOC_VPU_PRIORITY		0x12
#define NOC_CPU_PRIORITY		0x13
#define NOC_MIX_PRIORITY		0x14

#define FSL_SIP_WAKEUP_SRC		0xc2000009
#define FSL_SIP_WAKEUP_SRC_SCU		0x1
#define FSL_SIP_WAKEUP_SRC_IRQSTEER	0x2

#endif
