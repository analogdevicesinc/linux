/*
 * ak5558.h  --  audio driver for AK5558
 *
 * Copyright (C) 2016 Asahi Kasei Microdevices Corporation
 * Copyright 2017 NXP
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

#ifndef _AK5558_H
#define _AK5558_H

#define AK5558_00_POWER_MANAGEMENT1    0x00
#define AK5558_01_POWER_MANAGEMENT2    0x01
#define AK5558_02_CONTROL1             0x02
#define AK5558_03_CONTROL2             0x03
#define AK5558_04_CONTROL3             0x04
#define AK5558_05_DSD                  0x05

/* Bitfield Definitions */

/* AK5558_02_CONTROL1 (0x03) Fields */
#define AK5558_DIF			0x02
#define AK5558_DIF_MSB_MODE		(0 << 1)
#define AK5558_DIF_I2S_MODE		(1 << 1)

#define AK5558_BITS			0x04
#define AK5558_DIF_24BIT_MODE		(0 << 2)
#define AK5558_DIF_32BIT_MODE		(1 << 2)

#define AK5558_CKS			0x78
#define AK5558_CKS_128FS_192KHZ		(0 << 3)
#define AK5558_CKS_192FS_192KHZ		(1 << 3)
#define AK5558_CKS_256FS_48KHZ		(2 << 3)
#define AK5558_CKS_256FS_96KHZ		(3 << 3)
#define AK5558_CKS_384FS_96KHZ		(4 << 3)
#define AK5558_CKS_384FS_48KHZ		(5 << 3)
#define AK5558_CKS_512FS_48KHZ		(6 << 3)
#define AK5558_CKS_768FS_48KHZ		(7 << 3)
#define AK5558_CKS_64FS_384KHZ		(8 << 3)
#define AK5558_CKS_32FS_768KHZ		(9 << 3)
#define AK5558_CKS_96FS_384KHZ		(10 << 3)
#define AK5558_CKS_48FS_768KHZ		(11 << 3)
#define AK5558_CKS_64FS_768KHZ		(12 << 3)
#define AK5558_CKS_1024FS_16KHZ		(13 << 3)
#define AK5558_CKS_AUTO			(15 << 3)

#endif
