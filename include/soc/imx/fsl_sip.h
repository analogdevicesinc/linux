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

#define SC_TIMER_WDOG_ACTION_PARTITION      0   /*!< Reset partition */
#define SC_TIMER_WDOG_ACTION_WARM           1   /*!< Warm reset system */
#define SC_TIMER_WDOG_ACTION_COLD           2   /*!< Cold reset system */
#define SC_TIMER_WDOG_ACTION_BOARD          3   /*!< Reset board */

#define FSL_SIP_DDR_DVFS		0xc2000004

#define FSL_SIP_SRC			0xc2000005
#define FSL_SIP_SRC_M4_START		0x00
#define FSL_SIP_SRC_M4_STARTED		0x01

#define FSL_SIP_GET_SOC_INFO		0xc2000006

#endif
