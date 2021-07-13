/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * IT6161 MIPI to HDMI Converter driver
 *
 * Copyright (C) 2021 NXP
 */
#ifndef __IT6161_H__
#define __IT6161_H__

/* Video Configuration */
#define F_MODE_RGB444  0
#define F_MODE_YUV422 1
#define F_MODE_YUV444 2
#define F_MODE_CLRMOD_MASK 3

#define F_VIDMODE_ITU709  (1<<4)
#define F_VIDMODE_ITU601  0
#define F_VIDMODE_16_235  (1<<5)
#define F_VIDMODE_0_255   0

#define F_VIDMODE_EN_UDFILT (1<<6)
#define F_VIDMODE_EN_DITHER (1<<7)

#define INPUT_COLOR_MODE	F_MODE_RGB444
#define OUTPUT_COLOR_MODE	F_MODE_RGB444

/* Audio Configuration */

/* Audio interface */
#define I2S 0
#define SPDIF 1
#define TDM 2

/* Audio sample clock */
#define AUDFS_44p1KHz		0
#define AUDFS_88p2KHz		8
#define AUDFS_176p4KHz		12
#define AUDFS_32KHz			3
#define AUDFS_48KHz			2
#define AUDFS_96KHz			10
#define AUDFS_192KHz		14
#define AUDFS_768KHz		9

#define F_AUDIO_ON			(1<<7)
#define F_AUDIO_HBR			(1<<6)
#define F_AUDIO_DSD			(1<<5)
#define F_AUDIO_NLPCM		(1<<4)
#define F_AUDIO_LAYOUT_1	(1<<3)
#define F_AUDIO_LAYOUT_0	(0<<3)

#define T_AUDIO_HBR			(F_AUDIO_ON | F_AUDIO_HBR)
#define T_AUDIO_DSD			(F_AUDIO_ON | F_AUDIO_DSD)
#define T_AUDIO_NLPCM		(F_AUDIO_ON | F_AUDIO_NLPCM)
#define T_AUDIO_LPCM		(F_AUDIO_ON)

/* #define SUPPORT_HBR_AUDIO */
#ifndef SUPPORT_HBR_AUDIO
	#define INPUT_SAMPLE_FREQ				AUDFS_48KHz
	#define INPUT_SAMPLE_FREQ_HZ			48000L
	#define OUTPUT_CHANNEL					2    /* 3,4,5,6,7,8 */
	#define CNOFIG_INPUT_AUDIO_TYPE			T_AUDIO_LPCM
	#define CONFIG_INPUT_AUDIO_INTERFACE	SPDIF
	#define I2S_FORMAT						0x01 /* 32bit I2S audio */
#else /* SUPPORT_HBR_AUDIO */
	#define INPUT_SAMPLE_FREQ				AUDFS_768KHz
	#define INPUT_SAMPLE_FREQ_HZ			768000L
	#define OUTPUT_CHANNEL					8
	#define CNOFIG_INPUT_AUDIO_TYPE			T_AUDIO_HBR
	#define CONFIG_INPUT_AUDIO_INTERFACE	SPDIF
	#define I2S_FORMAT						0x47 /* 32bit audio */
#endif

/* MIPI Rx Configuration */
#define MIPI_RX_LANE_COUNT     2	/* 1~4 */

#define SUPPORT_AUDI_AudSWL 24

#if (SUPPORT_AUDI_AudSWL == 16)
	#define CHTSTS_SWCODE 0x02
#elif (SUPPORT_AUDI_AudSWL == 18)
	#define CHTSTS_SWCODE 0x04
#elif (SUPPORT_AUDI_AudSWL == 20)
	#define CHTSTS_SWCODE 0x03
#else
	#define CHTSTS_SWCODE 0x0B
#endif

#define DDC_FIFO_MAXREQ 0x20

/* I2C address */
#define DDC_EDID_ADDRESS		0xA0
#define CEC_I2C_SLAVE_ADDR		0x9C

/* HDMI TX Register offset */
#define REG_TX_SW_RST				0x04
#define B_TX_AREF_RST (1<<4)
#define B_HDMITX_VID_RST (1<<3)
#define B_HDMITX_AUD_RST (1<<2)
#define B_TX_HDMI_RST (1<<1)
#define B_TX_HDCP_RST_HDMITX (1<<0)

#define REG_TX_INT_STAT1			0x06
#define B_TX_INT_AUD_OVERFLOW  (1<<7)
#define B_TX_INT_DDCFIFO_ERR   (1<<4)
#define B_TX_INT_DDC_BUS_HANG  (1<<2)
#define B_TX_INT_HPD_PLUG  (1<<0)

#define REG_TX_INT_STAT3			0x08
#define B_TX_INT_VIDSTABLE (1<<4)

#define REG_TX_INT_MASK1			0x09
#define B_TX_AUDIO_OVFLW_MASK (1<<7)
#define B_TX_DDC_FIFO_ERR_MASK (1<<4)
#define B_TX_DDC_BUS_HANG_MASK (1<<2)
#define B_TX_RXSEN_MASK (1<<1)
#define B_TX_HPD_MASK (1<<0)

#define REG_TX_INT_MASK3			0x0B
#define B_TX_VIDSTABLE_MASK (1<<3)

#define REG_TX_INT_CLR0				0x0C
#define REG_TX_INT_CLR1				0x0D
#define REG_TX_SYS_STATUS			0x0E
#define B_TX_HPDETECT      (1<<6)
#define B_TX_RXSENDETECT   (1<<5)
#define B_TXVIDSTABLE      (1<<4)
#define B_TX_CLR_AUD_CTS   (1<<1)
#define B_TX_INTACTDONE    (1<<0)

/* DDC */
#define REG_TX_DDC_MASTER_CTRL		0x10
#define B_TX_MASTERROM (1<<1)
#define B_TX_MASTERDDC (0<<1)
#define B_TX_MASTERHOST (1<<0)

#define REG_TX_DDC_HEADER			0x11
#define REG_TX_DDC_REQOFF			0x12
#define REG_TX_DDC_REQCOUNT			0x13
#define REG_TX_DDC_EDIDSEG			0x14
#define REG_TX_DDC_CMD				0x15
#define CMD_EDID_READ   3
#define CMD_FIFO_CLR    9
#define CMD_GEN_SCLCLK  0xA
#define CMD_DDC_ABORT   0xF

#define REG_TX_DDC_STATUS			0x16
#define B_TX_DDC_DONE  (1<<7)
#define B_TX_DDC_NOACK (1<<5)
#define B_TX_DDC_WAITBUS   (1<<4)
#define B_TX_DDC_ARBILOSE  (1<<3)

#define REG_TX_DDC_READFIFO			0x17
#define REG_TX_CLK_CTRL0			0x58
#define B_TX_AUTO_OVER_SAMPLING_CLOCK (1<<4)
#define O_TX_EXT_MCLK_SEL  2
#define M_TX_EXT_MCLK_SEL  (3<<O_TX_EXT_MCLK_SEL)
#define B_TX_EXT_128FS (0<<O_TX_EXT_MCLK_SEL)
#define B_TX_EXT_256FS (1<<O_TX_EXT_MCLK_SEL)
#define B_TX_EXT_512FS (2<<O_TX_EXT_MCLK_SEL)
#define B_TX_EXT_1024FS (3<<O_TX_EXT_MCLK_SEL)

#define REG_TX_CLK_STATUS2			0x5F
#define B_TX_OSF_LOCK (1<<5)

#define REG_TX_AFE_DRV_CTRL			0x61
#define B_TX_AFE_DRV_PWD (1<<5)
#define B_TX_AFE_DRV_RST (1<<4)

/* Input Data Format Register */
#define REG_TX_INPUT_MODE			0x70
#define B_TX_PCLKDIV2  (1<<5)

#define REG_TX_CSC_CTRL				0x72
#define B_HDMITX_CSC_BYPASS  0
#define B_HDMITX_CSC_RGB2YUV 2
#define B_HDMITX_CSC_YUV2RGB 3
#define M_TX_CSC_SEL  3
#define B_TX_EN_DITHER   (1<<7)
#define B_TX_EN_UDFILTER (1<<6)
#define B_TX_DNFREE_GO   (1<<5)
#define SIZEOF_CSCMTX 21

#define REG_TX_CSC_YOFF				0x73
#define REG_TX_CSC_COFF				0x74
#define REG_TX_CSC_RGBOFF			0x75

/* HDMI General Control Registers */
#define REG_TX_HDMI_MODE			0xC0
#define B_TX_HDMI_MODE 1
#define B_TX_DVI_MODE  0
#define REG_TX_GCP					0xC1
#define B_TX_SETAVMUTE (1<<0)

#define O_TX_COLOR_DEPTH 4
#define M_TX_COLOR_DEPTH 7
#define B_TX_COLOR_DEPTH_MASK (M_TX_COLOR_DEPTH << O_TX_COLOR_DEPTH)
#define REG_TX_PKT_GENERAL_CTRL		0xC6

/* Audio Channel Control */
#define REG_TX_AUDIO_CTRL0			0xE0
#define M_TX_AUD_16BIT (0<<6)
#define M_TX_AUD_18BIT (1<<6)
#define M_TX_AUD_20BIT (2<<6)
#define M_TX_AUD_24BIT (3<<6)
#define B_TX_AUD_SPDIF (1<<4)
#define B_TX_AUD_I2S (0<<4)
#define B_TX_AUD_EN_I2S3 (1<<3)
#define B_TX_AUD_EN_I2S2 (1<<2)
#define B_TX_AUD_EN_I2S1 (1<<1)
#define B_TX_AUD_EN_I2S0 (1<<0)
#define B_TX_AUD_EN_SPDIF 1

#define REG_TX_AUDIO_CTRL1			0xE1
#define REG_TX_AUDIO_FIFOMAP		0xE2
#define REG_TX_AUDIO_CTRL3			0xE3
#define B_TX_CHSTSEL (1<<4)

#define REG_TX_AUD_SRCVALID_FLAT	0xE4

#define REG_TX_AUD_HDAUDIO			0xE5
#define B_TX_HBR   (1<<3)
#define B_TX_DSD   (1<<1)
#define B_TX_TDM   (1<<0)

/* HDMI TX reg Bank 1 */
#define REGPktAudCTS0			0x30	/* 7:0 */
#define REGPktAudCTS1			0x31	/* 15:8 */
#define REGPktAudCTS2			0x32	/* 19:16 */
#define REGPktAudN0				0x33	/* 7:0 */
#define REGPktAudN1				0x34	/* 15:8 */
#define REGPktAudN2				0x35	/* 19:16 */
#define REG_TX_AVIINFO_DB1		0x58
#define REG_TX_AVIINFO_SUM		0x5D
#define REG_TX_PKT_AUDINFO_CC	0x68		/* [2:0] */
#define REG_TX_PKT_AUDINFO_SF	0x69		/* [4:2] */
#define REG_TX_PKT_AUDINFO_CA	0x6B		/* [7:0] */
#define REG_TX_PKT_AUDINFO_DM_LSV	0x6C	/* [7][6:3] */
#define REG_TX_PKT_AUDINFO_SUM	0x6D		/* [7:0] */
#define REG_TX_AUDCHST_MODE		0x91	/* 191 REG_TX_AUD_CHSTD[2:0] 6:4 */
#define REG_TX_AUDCHST_CAT		0x92	/* 192 REG_TX_AUD_CHSTCAT 7:0 */
#define REG_TX_AUDCHST_SRCNUM	0x93	/* 193 REG_TX_AUD_CHSTSRC 3:0 */
#define REG_TX_AUD0CHST_CHTNUM	0x94	/* 194 REG_TX_AUD0_CHSTCHR 7:4 */
#define REG_TX_AUDCHST_CA_FS	0x98	/* 198 REG_TX_AUD_CHSTCA 5:4 */
#define REG_TX_AUDCHST_OFS_WL	0x99	/* 199 REG_TX_AUD_CHSTOFS 7:4 */

#define REG_TX_PKT_SINGLE_CTRL	0xC5
#define B_TX_SW_CTS		(1<<1)

#define REG_TX_AVI_INFOFRM_CTRL	0xCD
#define REG_TX_AUD_INFOFRM_CTRL	0xCE
#define B_TX_ENABLE_PKT	1
#define B_TX_REPEAT_PKT	(1<<1)

struct RegSetEntry {
	u8 offset;
	u8 mask;
	u8 value;
};

enum {
	PCLK_LOW = 0,
	PCLK_MEDIUM,
	PCLK_HIGH
};

u8 bCSCMtx_RGB2YUV_ITU601_16_235[] = {
	0x00, 0x80, 0x00,
	0xB2, 0x04, 0x65, 0x02, 0xE9, 0x00,
	0x93, 0x3C, 0x18, 0x04, 0x55, 0x3F,
	0x49, 0x3D, 0x9F, 0x3E, 0x18, 0x04
};

u8 bCSCMtx_RGB2YUV_ITU601_0_255[] = {
	0x10, 0x80, 0x10,
	0x09, 0x04, 0x0E, 0x02, 0xC9, 0x00,
	0x0F, 0x3D, 0x84, 0x03, 0x6D, 0x3F,
	0xAB, 0x3D, 0xD1, 0x3E, 0x84, 0x03
};

u8 bCSCMtx_RGB2YUV_ITU709_16_235[] = {
	0x00, 0x80, 0x00,
	0xB8, 0x05, 0xB4, 0x01, 0x94, 0x00,
	0x4a, 0x3C, 0x17, 0x04, 0x9F, 0x3F,
	0xD9, 0x3C, 0x10, 0x3F, 0x17, 0x04
};

u8 bCSCMtx_RGB2YUV_ITU709_0_255[] = {
	0x10, 0x80, 0x10,
	0xEa, 0x04, 0x77, 0x01, 0x7F, 0x00,
	0xD0, 0x3C, 0x83, 0x03, 0xAD, 0x3F,
	0x4B, 0x3D, 0x32, 0x3F, 0x83, 0x03
};

u8 bCSCMtx_YUV2RGB_ITU601_16_235[] = {
	0x00, 0x00, 0x00,
	0x00, 0x08, 0x6B, 0x3A, 0x50, 0x3D,
	0x00, 0x08, 0xF5, 0x0A, 0x02, 0x00,
	0x00, 0x08, 0xFD, 0x3F, 0xDA, 0x0D
};

u8 bCSCMtx_YUV2RGB_ITU601_0_255[] = {
	0x04, 0x00, 0xA7,
	0x4F, 0x09, 0x81, 0x39, 0xDD, 0x3C,
	0x4F, 0x09, 0xC4, 0x0C, 0x01, 0x00,
	0x4F, 0x09, 0xFD, 0x3F, 0x1F, 0x10
};

u8 bCSCMtx_YUV2RGB_ITU709_16_235[] = {
	0x00, 0x00, 0x00,
	0x00, 0x08, 0x55, 0x3C, 0x88, 0x3E,
	0x00, 0x08, 0x51, 0x0C, 0x00, 0x00,
	0x00, 0x08, 0x00, 0x00, 0x84, 0x0E
};

u8 bCSCMtx_YUV2RGB_ITU709_0_255[] = {
	0x04, 0x00, 0xA7,
	0x4F, 0x09, 0xBA, 0x3B, 0x4B, 0x3E,
	0x4F, 0x09, 0x57, 0x0E, 0x02, 0x00,
	0x4F, 0x09, 0xFE, 0x3F, 0xE8, 0x10
};

#endif
