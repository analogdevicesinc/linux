/*
 * fsl_acm.h - ALSA ACM interface for the Freescale i.MX SoC
 *
 * Copyright 2017 NXP
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _FSL_ACM_H
#define _FSL_ACM_H

/* The offset of ACM control registers */
#define AUD_CLK0_SEL_OFF		0x00000
#define AUD_CLK1_SEL_OFF		0x10000
#define MCLKOUT0_SEL_OFF		0x20000
#define MCLKOUT1_SEL_OFF		0x30000
#define ASRC0_CLK_SEL_OFF		0x40000

#define ESAI0_CLK_SEL_OFF		0x60000
#define ESAI1_CLK_SEL_OFF		0x70000
#define GPT0_CLK_SEL_OFF		0x80000
#define GPT0_CAPIN1_SEL_OFF		0x80004
#define GPT0_CAPIN2_SEL_OFF		0x80008
#define GPT1_CLK_SEL_OFF		0x90000
#define GPT1_CAPIN1_SEL_OFF		0x90004
#define GPT1_CAPIN2_SEL_OFF		0x90008
#define GPT2_CLK_SEL_OFF		0xA0000
#define GPT2_CAPIN1_SEL_OFF		0xA0004
#define GPT2_CAPIN2_SEL_OFF		0xA0008
#define GPT3_CLK_SEL_OFF		0xB0000
#define GPT3_CAPIN1_SEL_OFF		0xB0004
#define GPT3_CAPIN2_SEL_OFF		0xB0008
#define GPT4_CLK_SEL_OFF		0xC0000
#define GPT4_CAPIN1_SEL_OFF		0xC0004
#define GPT4_CAPIN2_SEL_OFF		0xC0008
#define GPT5_CLK_SEL_OFF		0xD0000
#define GPT5_CAPIN1_SEL_OFF		0xD0004
#define GPT5_CAPIN2_SEL_OFF		0xD0008
#define SAI0_MCLK_SEL_OFF		0xE0000
#define SAI1_MCLK_SEL_OFF		0xF0000
#define SAI2_MCLK_SEL_OFF		0x100000
#define SAI3_MCLK_SEL_OFF		0x110000
#define SAI_HDMIRX0_MCLK_SEL_OFF	0x120000
#define SAI_HDMITX0_MCLK_SEL_OFF	0x130000
#define SAI6_MCLK_SEL_OFF		0x140000
#define SAI7_MCLK_SEL_OFF		0x150000

/* in imx8qxp SAI6=>SAI4, SAI7=>SAI5 */
#define SAI4_MCLK_SEL_OFF		0x140000
#define SAI5_MCLK_SEL_OFF		0x150000

#define SPDIF0_TX_CLK_SEL_OFF		0x1A0000
#define SPDIF1_TX_CLK_SEL_OFF		0x1B0000
#define MQS_HMCLK_SEL_OFF		0x1C0000

/* GPT CAPTURE Event definition*/
#define IPI_USB0_SOF			0
#define IPI_USB1_SOF			1
#define IPI_USB30_ITP			2
#define IPI_ETHERNET0_EVENT		3
#define IPI_ETHERNET1_EVENT		4
#define IPI_MPEG0_EVENT			5
#define IPI_MPEG1_EVENT			6
#define ASRC0_DMA1_REQ			7
#define ASRC0_DMA2_REQ			8
#define ASRC0_DMA3_REQ			9
#define ASRC0_DMA4_REQ			10
#define ASRC0_DMA5_REQ			11
#define ASRC0_DMA6_REQ			12
#define ESAI0_IPD_ESAI_RX_B		13
#define ESAI0_IPD_ESAI_TX_B		14
#define SPDIF0_DRQ0_SPDIF_B		15
#define SPDIF0_DRQ1_SPDIF_B		16
#define SPDIF1_DRQ0_SPDIF_B		17
#define SPDIF1_DRQ1_SPDIF_B		18
#define SAI_HDMIRX0_IPD_REQ_SAI_RX	19
#define SAI_HDMITX0_IPD_REQ_SAI_TX	20
#define ASRC1_DMA1_REQ			21
#define ASRC1_DMA2_REQ			22
#define ASRC1_DMA3_REQ			23
#define ASRC1_DMA4_REQ			24
#define ASRC1_DMA5_REQ			25
#define ASRC1_DMA6_REQ			26
#define ESAI1_IPD_ESAI_RX_B		27
#define ESAI1_IPD_ESAI_TX_B		28
#define SAI6_IPD_REQ_SAI_RX		29
#define SAI6_IPD_REQ_SAI_TX		30
#define SAI7_IPD_REQ_SAI_TX		31


#endif /* _FSL_ACM_H */
