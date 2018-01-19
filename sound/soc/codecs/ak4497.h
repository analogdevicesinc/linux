/*
 * ak4497.h  --  audio driver for ak4497
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 * Copyright (C) 2017, NXP
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#define SND_SOC_DAIFMT_DSD		   0x101

#ifndef _AK4497_H
#define _AK4497_H

#define AK4497_00_CONTROL1          0x00
#define AK4497_01_CONTROL2          0x01
#define AK4497_02_CONTROL3          0x02
#define AK4497_03_LCHATT            0x03
#define AK4497_04_RCHATT            0x04
#define AK4497_05_CONTROL4          0x05
#define AK4497_06_DSD1              0x06
#define AK4497_07_CONTROL5          0x07
#define AK4497_08_SOUNDCONTROL      0x08
#define AK4497_09_DSD2              0x09
#define AK4497_0A_CONTROL7          0x0A
#define AK4497_0B_CONTROL8          0x0B
#define AK4497_0C_RESERVED          0x0C
#define AK4497_0D_RESERVED          0x0D
#define AK4497_0E_RESERVED          0x0E
#define AK4497_0F_RESERVED          0x0F
#define AK4497_10_RESERVED          0x10
#define AK4497_11_RESERVED          0x11
#define AK4497_12_RESERVED          0x12
#define AK4497_13_RESERVED          0x13
#define AK4497_14_RESERVED          0x14
#define AK4497_15_DFSREAD           0x15


#define AK4497_MAX_REGISTERS	(AK4497_15_DFSREAD)

/* Bitfield Definitions */

/* AK4497_00_CONTROL1 (0x00) Fields */
#define AK4497_DIF					0x0E
#define AK4497_DIF_MSB_MODE	    (2 << 1)
#define AK4497_DIF_I2S_MODE     (3 << 1)
#define AK4497_DIF_32BIT_MODE	(4 << 1)

#define AK4497_DIF_16BIT_LSB	(0 << 1)
#define AK4497_DIF_20BIT_LSB	(1 << 1)
#define AK4497_DIF_24BIT_MSB	(2 << 1)
#define AK4497_DIF_24BIT_I2S	(3 << 1)
#define AK4497_DIF_24BIT_LSB	(4 << 1)
#define AK4497_DIF_32BIT_LSB	(5 << 1)
#define AK4497_DIF_32BIT_MSB	(6 << 1)
#define AK4497_DIF_32BIT_I2S	(7 << 1)

/* AK4497_02_CONTROL3 (0x02) Fields */
#define AK4497_DIF_DSD				0x80
#define AK4497_DIF_DSD_MODE	    (1 << 7)


/* AK4497_01_CONTROL2 (0x01) Fields */
/* AK4497_05_CONTROL4 (0x05) Fields */
#define AK4497_DFS				0x18
#define AK4497_DFS_48KHZ		(0x0 << 3)  //  30kHz to 54kHz
#define AK4497_DFS_96KHZ		(0x1 << 3)  //  54kHz to 108kHz
#define AK4497_DFS_192KHZ		(0x2 << 3)  //  120kHz  to 216kHz
#define AK4497_DFS_384KHZ		(0x0 << 3)
#define AK4497_DFS_768KHZ		(0x1 << 3)

#define AK4497_DFS2				0x2
#define AK4497_DFS2_48KHZ		(0x0 << 1)  // 30kHz to 216kHz
#define AK4497_DFS2_384KHZ		(0x1 << 1)  // 384kHz, 768kHz to 108kHz

#endif
