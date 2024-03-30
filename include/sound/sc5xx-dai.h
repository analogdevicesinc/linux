/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices SC5XX DAI code
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __SC5XX_DAI_H_
#define __SC5XX_DAI_H_

#ifdef CONFIG_ARCH_SC59X_64
#define DAI0_BASE_ADDRESS (0x310C9000)
#define DAI1_BASE_ADDRESS (0x310CA000)
#endif

#define REG_DAI_EXTD_CLK0    0x000 /*  DAI0 Extended Clock Routing Control Register 0 */
#define REG_DAI_EXTD_CLK1    0x004 /*  DAI0 Extended Clock Routing Control Register 1 */
#define REG_DAI_EXTD_CLK2    0x008 /*  DAI0 Extended Clock Routing Control Register 2 */
#define REG_DAI_EXTD_CLK3    0x00C /*  DAI0 Extended Clock Routing Control Register 3 */
#define REG_DAI_EXTD_CLK4    0x010 /*  DAI0 Extended Clock Routing Control Register 4 */
#define REG_DAI_EXTD_CLK5    0x014 /*  DAI0 Extended Clock Routing Control Register 5 */
#define REG_DAI_EXTD_DAT0    0x018 /*  DAI0 Extended Serial Data Routing Control Register 0 */
#define REG_DAI_EXTD_DAT1    0x01C /*  DAI0 Extended Serial Data Routing Control Register 1 */
#define REG_DAI_EXTD_DAT2    0x020 /*  DAI0 Extended Serial Data Routing Control Register 2 */
#define REG_DAI_EXTD_DAT3    0x024 /*  DAI0 Extended Serial Data Routing Control Register 3 */
#define REG_DAI_EXTD_DAT4    0x028 /*  DAI0 Extended Serial Data Routing Control Register 4 */
#define REG_DAI_EXTD_DAT5    0x02C /*  DAI0 Extended Serial Data Routing Control Register 5 */
#define REG_DAI_EXTD_DAT6    0x030 /*  DAI0 Extended Serial Data Routing Control Register 6 */
#define REG_DAI_EXTD_FS0     0x034 /*  DAI0 Extended Frame Sync Routing Control Register 0 */
#define REG_DAI_EXTD_FS1     0x038 /*  DAI0 Extended Frame Sync Routing Control Register 1 */
#define REG_DAI_EXTD_FS2     0x03C /*  DAI0 Extended Frame Sync Routing Control Register 2 */
#define REG_DAI_EXTD_FS4     0x044 /*  DAI0 Extended Frame Sync Routing Control Register 4 */
#define REG_DAI_EXTD_PIN0    0x048 /*  DAI0 Extended Pin Buffer Assignment Register 0 */
#define REG_DAI_EXTD_PIN1    0x04C /*  DAI0 Extended Pin Buffer Assignment Register 1 */
#define REG_DAI_EXTD_PIN2    0x050 /*  DAI0 Extended Pin Buffer Assignment Register 2 */
#define REG_DAI_EXTD_PIN3    0x054 /*  DAI0 Extended Pin Buffer Assignment Register 3 */
#define REG_DAI_EXTD_PIN4    0x058 /*  DAI0 Extended Pin Buffer Assignment Register 4 */
#define REG_DAI_EXTD_MISC0   0x05C /*  DAI0 Extended Miscellaneous Control Register 0 */
#define REG_DAI_EXTD_MISC1   0x060 /*  DAI0 Extended Miscellaneous Control Register 1 */
#define REG_DAI_EXTD_MISC2   0x064 /*  DAI0 Extended Miscellaneous Control Register 2 */
#define REG_DAI_EXTD_PBEN0   0x068 /*  DAI0 Extended Pin Buffer Enable Register 0 */
#define REG_DAI_EXTD_PBEN1   0x06C /*  DAI0 Extended Pin Buffer Enable Register 1 */
#define REG_DAI_EXTD_PBEN2   0x070 /*  DAI0 Extended Pin Buffer Enable Register 2 */
#define REG_DAI_EXTD_PBEN3   0x074 /*  DAI0 Extended Pin Buffer Enable Register 3 */
#define REG_DAI_CLK0         0x0C0 /*  DAI0 Clock Routing Control Register 0 */
#define REG_DAI_CLK1         0x0C4 /*  DAI0 Clock Routing Control Register 1 */
#define REG_DAI_CLK2         0x0C8 /*  DAI0 Clock Routing Control Register 2 */
#define REG_DAI_CLK3         0x0CC /*  DAI0 Clock Routing Control Register 3 */
#define REG_DAI_CLK4         0x0D0 /*  DAI0 Clock Routing Control Register 4 */
#define REG_DAI_CLK5         0x0D4 /*  DAI0 Clock Routing Control Register 5 */
#define REG_DAI_DAT0         0x100 /*  DAI0 Serial Data Routing Control Register 0 */
#define REG_DAI_DAT1         0x104 /*  DAI0 Serial Data Routing Control Register 1 */
#define REG_DAI_DAT2         0x108 /*  DAI0 Serial Data Routing Control Register 2 */
#define REG_DAI_DAT3         0x10C /*  DAI0 Serial Data Routing Control Register 3 */
#define REG_DAI_DAT4         0x110 /*  DAI0 Serial Data Routing Control Register 4 */
#define REG_DAI_DAT5         0x114 /*  DAI0 Serial Data Routing Control Register 5 */
#define REG_DAI_DAT6         0x118 /*  DAI0 Serial Data Routing Control Register 6 */
#define REG_DAI_FS0          0x140 /*  DAI0 Frame Sync Routing Control Register 0 */
#define REG_DAI_FS1          0x144 /*  DAI0 Frame Sync Routing Control Register 1 */
#define REG_DAI_FS2          0x148 /*  DAI0 Frame Sync Routing Control Register 2 */
#define REG_DAI_FS4          0x150 /*  DAI0 Frame Sync Routing Control Register 4 */
#define REG_DAI_PIN0         0x180 /*  DAI0 Pin Buffer Assignment Register 0 */
#define REG_DAI_PIN1         0x184 /*  DAI0 Pin Buffer Assignment Register 1 */
#define REG_DAI_PIN2         0x188 /*  DAI0 Pin Buffer Assignment Register 2 */
#define REG_DAI_PIN3         0x18C /*  DAI0 Pin Buffer Assignment Register 3 */
#define REG_DAI_PIN4         0x190 /*  DAI0 Pin Buffer Assignment Register 4 */
#define REG_DAI_MISC0        0x1C0 /*  DAI0 Miscellaneous Control Register 0 */
#define REG_DAI_MISC1        0x1C4 /*  DAI0 Miscellaneous Control Register 1 */
#define REG_DAI_MISC2        0x1C8 /*  DAI0 Miscellaneous Control Register 1 */
#define REG_DAI_PBEN0        0x1E0 /*  DAI0 Pin Buffer Enable Register 0 */
#define REG_DAI_PBEN1        0x1E4 /*  DAI0 Pin Buffer Enable Register 1 */
#define REG_DAI_PBEN2        0x1E8 /*  DAI0 Pin Buffer Enable Register 2 */
#define REG_DAI_PBEN3        0x1EC /*  DAI0 Pin Buffer Enable Register 3 */
#define REG_DAI_IMSK_FE      0x200 /*  DAI0 Falling-Edge Interrupt Mask Register */
#define REG_DAI_IMSK_RE      0x204 /*  DAI0 Rising-Edge Interrupt Mask Register */
#define REG_DAI_IMSK_PRI     0x210 /*  DAI0 Core Interrupt Priority Assignment Register */
#define REG_DAI_IRPTL_H      0x220 /*  DAI0 High Priority Interrupt Latch Register */
#define REG_DAI_IRPTL_L      0x224 /*  DAI0 Low Priority Interrupt Latch Register */
#define REG_DAI_IRPTL_HS     0x230 /*  DAI0 Shadow High Priority Interrupt Latch Register */
#define REG_DAI_IRPTL_LS     0x234 /*  DAI0 Shadow Low Priority Interrupt Latch Register */
#define REG_DAI_PIN_STAT     0x2E4 /*  DAI0 Pin Status Register */
#define REG_DAI_GBL_SP_EN    0x2E8 /*  DAI0 Global SPORT Enable Register */
#define REG_DAI_GBL_INT_EN   0x2EC /*  DAI0 Global SPORT Interrupt Grouping Register */
#define REG_DAI_GBL_PCG_EN   0x2F0 /*  DAI0 Global PCG Enable Control Register */

/*	DAI0	*/
#define REG_DAI0_CLK0		0x310C90C0
#define REG_DAI0_CLK1		0x310C90C4
#define REG_DAI0_CLK2		0x310C90C8
#define REG_DAI0_CLK3		0x310C90CC
#define REG_DAI0_CLK4		0x310C90D0
#define REG_DAI0_CLK5		0x310C90D4
#define REG_DAI0_DAT0		0x310C9100
#define REG_DAI0_DAT1		0x310C9104
#define REG_DAI0_DAT2		0x310C9108
#define REG_DAI0_DAT3		0x310C910C
#define REG_DAI0_DAT4		0x310C9110
#define REG_DAI0_DAT5		0x310C9114
#define REG_DAI0_DAT6		0x310C9118
#define REG_DAI0_FS0		0x310C9140
#define REG_DAI0_FS1		0x310C9144
#define REG_DAI0_FS2		0x310C9148
#define REG_DAI0_FS4		0x310C9150
#define REG_DAI0_PIN0		0x310C9180
#define REG_DAI0_PIN1		0x310C9184
#define REG_DAI0_PIN2		0x310C9188
#define REG_DAI0_PIN3		0x310C918C
#define REG_DAI0_PIN4		0x310C9190
#define REG_DAI0_MISC0		0x310C91C0
#define REG_DAI0_MISC1		0x310C91C4
#define REG_DAI0_PBEN0		0x310C91E0
#define REG_DAI0_PBEN1		0x310C91E4
#define REG_DAI0_PBEN2		0x310C91E8
#define REG_DAI0_PBEN3		0x310C91EC
#define REG_DAI0_IMSK_FE	0x310C9200
#define REG_DAI0_IMSK_RE	0x310C9204
#define REG_DAI0_IMSK_PRI	0x310C9210
#define REG_DAI0_IRPTL_H	0x310C9220
#define REG_DAI0_IRPTL_L	0x310C9224
#define REG_DAI0_IRPTL_HS	0x310C9230
#define REG_DAI0_IRPTL_LS	0x310C9234
#define REG_DAI0_PIN_STAT	0x310C92E4

/*	DAI1	*/
#ifndef CONFIG_ARCH_SC59X
	#define REG_DAI1_CLK0          0x310CB0C0
	#define REG_DAI1_CLK1          0x310CB0C4
	#define REG_DAI1_CLK2          0x310CB0C8
	#define REG_DAI1_CLK3          0x310CB0CC
	#define REG_DAI1_CLK4          0x310CB0D0
	#define REG_DAI1_CLK5          0x310CB0D4
	#define REG_DAI1_DAT0          0x310CB100
	#define REG_DAI1_DAT1          0x310CB104
	#define REG_DAI1_DAT2          0x310CB108
	#define REG_DAI1_DAT3          0x310CB10C
	#define REG_DAI1_DAT4          0x310CB110
	#define REG_DAI1_DAT5          0x310CB114
	#define REG_DAI1_DAT6          0x310CB118
	#define REG_DAI1_FS0           0x310CB140
	#define REG_DAI1_FS1           0x310CB144
	#define REG_DAI1_FS2           0x310CB148
	#define REG_DAI1_FS4           0x310CB150
	#define REG_DAI1_PIN0          0x310CB180
	#define REG_DAI1_PIN1          0x310CB184
	#define REG_DAI1_PIN2          0x310CB188
	#define REG_DAI1_PIN3          0x310CB18C
	#define REG_DAI1_PIN4          0x310CB190
	#define REG_DAI1_MISC0         0x310CB1C0
	#define REG_DAI1_MISC1         0x310CB1C4
	#define REG_DAI1_PBEN0         0x310CB1E0
	#define REG_DAI1_PBEN1         0x310CB1E4
	#define REG_DAI1_PBEN2         0x310CB1E8
	#define REG_DAI1_PBEN3         0x310CB1EC
	#define REG_DAI1_IMSK_FE       0x310CB200
	#define REG_DAI1_IMSK_RE       0x310CB204
	#define REG_DAI1_IMSK_PRI      0x310CB210
	#define REG_DAI1_IRPTL_H       0x310CB220
	#define REG_DAI1_IRPTL_L       0x310CB224
	#define REG_DAI1_IRPTL_HS      0x310CB230
	#define REG_DAI1_IRPTL_LS      0x310CB234
	#define REG_DAI1_PIN_STAT      0x310CB2E4
#endif

#ifdef CONFIG_ARCH_SC59X
	#define REG_DAI1_CLK0          0x310CA0C0
	#define REG_DAI1_CLK1          0x310CA0C4
	#define REG_DAI1_CLK2          0x310CA0C8
	#define REG_DAI1_CLK3          0x310CA0CC
	#define REG_DAI1_CLK4          0x310CA0D0
	#define REG_DAI1_CLK5          0x310CA0D4
	#define REG_DAI1_DAT0          0x310CA100
	#define REG_DAI1_DAT1          0x310CA104
	#define REG_DAI1_DAT2          0x310CA108
	#define REG_DAI1_DAT3          0x310CA10C
	#define REG_DAI1_DAT4          0x310CA110
	#define REG_DAI1_DAT5          0x310CA114
	#define REG_DAI1_DAT6          0x310CA118
	#define REG_DAI1_FS0           0x310CA140
	#define REG_DAI1_FS1           0x310CA144
	#define REG_DAI1_FS2           0x310CA148
	#define REG_DAI1_FS4           0x310CA150
	#define REG_DAI1_PIN0          0x310CA180
	#define REG_DAI1_PIN1          0x310CA184
	#define REG_DAI1_PIN2          0x310CA188
	#define REG_DAI1_PIN3          0x310CA18C
	#define REG_DAI1_PIN4          0x310CA190
	#define REG_DAI1_MISC0         0x310CA1C0
	#define REG_DAI1_MISC1         0x310CA1C4
	#define REG_DAI1_PBEN0         0x310CA1E0
	#define REG_DAI1_PBEN1         0x310CA1E4
	#define REG_DAI1_PBEN2         0x310CA1E8
	#define REG_DAI1_PBEN3         0x310CA1EC
	#define REG_DAI1_IMSK_FE       0x310CA200
	#define REG_DAI1_IMSK_RE       0x310CA204
	#define REG_DAI1_IMSK_PRI      0x310CA210
	#define REG_DAI1_IRPTL_H       0x310CA220
	#define REG_DAI1_IRPTL_L       0x310CA224
	#define REG_DAI1_IRPTL_HS      0x310CA230
	#define REG_DAI1_IRPTL_LS      0x310CA234
	#define REG_DAI1_PIN_STAT      0x310CA2E4
#endif

/* Pads init function for dai  */
void pads_init(void);

#endif	/*__SC5XX_DAI_H_ */
