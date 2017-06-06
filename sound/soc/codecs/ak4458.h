/*
 * ak4458.h  --  audio driver for AK4458
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 * Author:  Tsuyoshi Mutsuro
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _AK4458_H
#define _AK4458_H

#include <linux/regmap.h>

/* Settings */
//#define AK4458_ACKS_USE_MANUAL_MODE

#define AK4458_00_CONTROL1			0x00
#define AK4458_01_CONTROL2			0x01
#define AK4458_02_CONTROL3			0x02
#define AK4458_03_LCHATT			0x03
#define AK4458_04_RCHATT			0x04
#define AK4458_05_CONTROL4			0x05
#define AK4458_06_DSD1				0x06
#define AK4458_07_CONTROL5			0x07
#define AK4458_08_SOUND_CONTROL			0x08
#define AK4458_09_DSD2				0x09
#define AK4458_0A_CONTROL6			0x0A
#define AK4458_0B_CONTROL7			0x0B
#define AK4458_0C_CONTROL8			0x0C
#define AK4458_0D_CONTROL9			0x0D
#define AK4458_0E_CONTROL10			0x0E
#define AK4458_0F_L2CHATT			0x0F
#define AK4458_10_R2CHATT			0x10
#define AK4458_11_L3CHATT			0x11
#define AK4458_12_R3CHATT			0x12
#define AK4458_13_L4CHATT			0x13
#define AK4458_14_R4CHATT			0x14

/* Bitfield Definitions */

/* AK4458_00_CONTROL1 (0x00) Fields */
//Addr Register Name  D7     D6    D5    D4    D3    D2    D1    D0
//00H  Control 1      ACKS   0     0     0     DIF2  DIF1  DIF0  RSTN

//MONO1 & SELLR1 bits
#define AK4458_DAC1_LR_MASK	0x0A
#define AK4458_DAC1_INV_MASK	0xC0

//MONO2 & SELLR2 bits
#define AK4458_DAC2_MASK1	0x20
#define AK4458_DAC2_MASK2	0x38

//MONO3 & SELLR3 bits
#define AK4458_DAC3_LR_MASK	0x44
#define AK4458_DAC3_INV_MASK	0x30

//MONO4 & SELLR4 bits
#define AK4458_DAC4_LR_MASK	0x88
#define AK4458_DAC4_INV_MASK	0xC0


//SDS2-0 bits
#define AK4458_SDS0__MASK	0x10
#define AK4458_SDS12_MASK	0x30

//Digital Filter (SD, SLOW, SSLOW)
#define AK4458_SD_MASK		0x20
#define AK4458_SLOW_MASK	0x01
#define AK4458_SSLOW_MASK	0x01

//DIF2 1 0
//  x  1 0 MSB justified  Figure 3 (default)
//  x  1 1 I2S Compliment  Figure 4
#define AK4458_DIF_MASK			0x06
#define AK4458_DIF_MSB_LOW_FS_MODE	(2 << 1)
#define AK4458_DIF_I2S_LOW_FS_MODE	(3 << 1)

// ACKS is Auto mode so disable the Manual feature
//#define AK4458_ACKS_USE_MANUAL_MODE
/* AK4458_00_CONTROL1 (0x00) D0 bit */
#define AK4458_RSTN_MASK		0x01
#define AK4458_RSTN			(0x1 << 0)


#ifdef AK4458_ACKS_USE_MANUAL_MODE
/* AK4458_01_CONTROL2 (0x01) and AK4458_05_CONTROL4 (0x05) Fields */
#define AK4458_DFS01_MASK		0x18
#define AK4458_DFS2__MASK		0x02
#define AK4458_DFS01_48KHZ		(0x0 << 3)  //  30kHz to 54kHz
#define AK4458_DFS2__48KHZ		(0x0 << 1)  //  30kHz to 54kHz

#define AK4458_DFS01_96KHZ		(0x1 << 3)  //  54kHz to 108kHz
#define AK4458_DFS2__96KHZ		(0x0 << 1)  //  54kHz to 108kHz

#define AK4458_DFS01_192KHZ		(0x2 << 3)  //  120kHz  to 216kHz
#define AK4458_DFS2__192KHZ		(0x0 << 1)  //  120kHz  to 216kHz

#define AK4458_DFS01_384KHZ		(0x0 << 3)	//	384kHz
#define AK4458_DFS2__384KHZ		(0x1 << 1)	//	384kHz

#define AK4458_DFS01_768KHZ		(0x1 << 3)	//	768kHz
#define AK4458_DFS2__768KHZ		(0x1 << 1)	//	768kHz
#endif

extern const struct regmap_config ak4458_i2c_regmap_config;
extern const struct regmap_config ak4458_spi_regmap_config;
extern const struct dev_pm_ops ak4458_pm;

int ak4458_probe(struct device *dev, struct regmap *regmap);
void ak4458_remove(struct device *dev);

#endif
