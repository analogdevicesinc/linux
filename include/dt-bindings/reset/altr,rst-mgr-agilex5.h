/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Intel Corporation. All rights reserved
 *
 */

#ifndef _DT_BINDINGS_RESET_ALTR_RST_MGR_AGILEX5_H
#define _DT_BINDINGS_RESET_ALTR_RST_MGR_AGILEX5_H

/* PER0MODRST */
#define EMAC0_RESET		32
#define EMAC1_RESET		33
#define EMAC2_RESET		34
#define USB0_RESET		35
#define USB1_RESET		36
#define NAND_RESET		37
#define SOFT_PHY_RESET		38
#define SDMMC_RESET		39
#define EMAC0_OCP_RESET		40
#define EMAC1_OCP_RESET		41
#define EMAC2_OCP_RESET		42
#define USB0_OCP_RESET		43
#define USB1_OCP_RESET		44
#define NAND_OCP_RESET		45
/* 46 is empty */
#define SDMMC_OCP_RESET		47
#define DMA_RESET		48
#define SPIM0_RESET		49
#define SPIM1_RESET		50
#define SPIS0_RESET		51
#define SPIS1_RESET		52
#define DMA_OCP_RESET		53
#define EMAC_PTP_RESET		54
/* 55 is empty*/
#define DMAIF0_RESET		56
#define DMAIF1_RESET		57
#define DMAIF2_RESET		58
#define DMAIF3_RESET		59
#define DMAIF4_RESET		60
#define DMAIF5_RESET		61
#define DMAIF6_RESET		62
#define DMAIF7_RESET		63

/* PER1MODRST */
#define WATCHDOG0_RESET		64
#define WATCHDOG1_RESET		65
#define WATCHDOG2_RESET		66
#define WATCHDOG3_RESET		67
#define L4SYSTIMER0_RESET	68
#define L4SYSTIMER1_RESET	69
#define SPTIMER0_RESET		70
#define SPTIMER1_RESET		71
#define I2C0_RESET		72
#define I2C1_RESET		73
#define I2C2_RESET		74
#define I2C3_RESET		75
#define I2C4_RESET		76
#define I3C0_RESET		77
#define I3C1_RESET		78
/* 79 is empty */
#define UART0_RESET		80
#define UART1_RESET		81
/* 82-87 is empty */
#define GPIO0_RESET		88
#define GPIO1_RESET		89
#define WATCHDOG4_RESET		90

/* BRGMODRST */
#define SOC2FPGA_RESET		96
#define LWHPS2FPGA_RESET	97
#define FPGA2SOC_RESET		98
#define F2SSDRAM0_RESET		99
/* 100-101 is empty */
#define MPFE_RESET		102

/* DBGMODRST */
#define DBG_RESET		128

#endif
