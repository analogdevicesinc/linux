/*
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FSL_SAI_H
#define __FSL_SAI_H

#include <linux/pm_qos.h>
#include <sound/dmaengine_pcm.h>

#define FSL_SAI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE |\
			 SNDRV_PCM_FMTBIT_DSD_U8 |\
			 SNDRV_PCM_FMTBIT_DSD_U16_LE |\
			 SNDRV_PCM_FMTBIT_DSD_U32_LE)

/* SAI Register Map Register */
#define FSL_SAI_VERID	0x00 /* SAI Version ID Register */
#define FSL_SAI_PARAM	0x04 /* SAI Parameter Register */
#define FSL_SAI_TCSR(offset) (0x00 + offset) /* SAI Transmit Control */
#define FSL_SAI_TCR1(offset) (0x04 + offset) /* SAI Transmit Configuration 1 */
#define FSL_SAI_TCR2(offset) (0x08 + offset) /* SAI Transmit Configuration 2 */
#define FSL_SAI_TCR3(offset) (0x0c + offset) /* SAI Transmit Configuration 3 */
#define FSL_SAI_TCR4(offset) (0x10 + offset) /* SAI Transmit Configuration 4 */
#define FSL_SAI_TCR5(offset) (0x14 + offset) /* SAI Transmit Configuration 5 */
#define FSL_SAI_TDR0    0x20 /* SAI Transmit Data */
#define FSL_SAI_TDR1    0x24 /* SAI Transmit Data */
#define FSL_SAI_TDR2    0x28 /* SAI Transmit Data */
#define FSL_SAI_TDR3    0x2C /* SAI Transmit Data */
#define FSL_SAI_TDR4    0x30 /* SAI Transmit Data */
#define FSL_SAI_TDR5    0x34 /* SAI Transmit Data */
#define FSL_SAI_TDR6    0x38 /* SAI Transmit Data */
#define FSL_SAI_TDR7    0x3C /* SAI Transmit Data */
#define FSL_SAI_TFR0    0x40 /* SAI Transmit FIFO */
#define FSL_SAI_TFR1    0x44 /* SAI Transmit FIFO */
#define FSL_SAI_TFR2    0x48 /* SAI Transmit FIFO */
#define FSL_SAI_TFR3    0x4C /* SAI Transmit FIFO */
#define FSL_SAI_TFR4    0x50 /* SAI Transmit FIFO */
#define FSL_SAI_TFR5    0x54 /* SAI Transmit FIFO */
#define FSL_SAI_TFR6    0x58 /* SAI Transmit FIFO */
#define FSL_SAI_TFR7    0x5C /* SAI Transmit FIFO */
#define FSL_SAI_TMR	0x60 /* SAI Transmit Mask */
#define FSL_SAI_TTCTL	0x70 /* SAI Transmit Timestamp Control Register */
#define FSL_SAI_TTCTN	0x74 /* SAI Transmit Timestamp Counter Register */
#define FSL_SAI_TBCTN	0x78 /* SAI Transmit Bit Counter Register */
#define FSL_SAI_TTCAP	0x7C /* SAI Transmit Timestamp Capture */

#define FSL_SAI_RCSR(offset) (0x80 + offset) /* SAI Receive Control */
#define FSL_SAI_RCR1(offset) (0x84 + offset) /* SAI Receive Configuration 1 */
#define FSL_SAI_RCR2(offset) (0x88 + offset) /* SAI Receive Configuration 2 */
#define FSL_SAI_RCR3(offset) (0x8c + offset) /* SAI Receive Configuration 3 */
#define FSL_SAI_RCR4(offset) (0x90 + offset) /* SAI Receive Configuration 4 */
#define FSL_SAI_RCR5(offset) (0x94 + offset) /* SAI Receive Configuration 5 */
#define FSL_SAI_RDR0    0xa0 /* SAI Receive Data */
#define FSL_SAI_RDR1    0xa4 /* SAI Receive Data */
#define FSL_SAI_RDR2    0xa8 /* SAI Receive Data */
#define FSL_SAI_RDR3    0xac /* SAI Receive Data */
#define FSL_SAI_RDR4    0xb0 /* SAI Receive Data */
#define FSL_SAI_RDR5    0xb4 /* SAI Receive Data */
#define FSL_SAI_RDR6    0xb8 /* SAI Receive Data */
#define FSL_SAI_RDR7    0xbc /* SAI Receive Data */
#define FSL_SAI_RFR0    0xc0 /* SAI Receive FIFO */
#define FSL_SAI_RFR1    0xc4 /* SAI Receive FIFO */
#define FSL_SAI_RFR2    0xc8 /* SAI Receive FIFO */
#define FSL_SAI_RFR3    0xcc /* SAI Receive FIFO */
#define FSL_SAI_RFR4    0xd0 /* SAI Receive FIFO */
#define FSL_SAI_RFR5    0xd4 /* SAI Receive FIFO */
#define FSL_SAI_RFR6    0xd8 /* SAI Receive FIFO */
#define FSL_SAI_RFR7    0xdc /* SAI Receive FIFO */
#define FSL_SAI_RMR	0xe0 /* SAI Receive Mask */
#define FSL_SAI_RTCTL	0xf0 /* SAI Receive Timestamp Control Register */
#define FSL_SAI_RTCTN	0xf4 /* SAI Receive Timestamp Counter Register */
#define FSL_SAI_RBCTN	0xf8 /* SAI Receive Bit Counter Register */
#define FSL_SAI_RTCAP	0xfc /* SAI Receive Timestamp Capture */

#define FSL_SAI_MCTL	0x100 /* SAI MCLK Control Register */
#define FSL_SAI_MDIV	0x104 /* SAI MCLK Divide Register */

#define FSL_SAI_xCSR(tx, off)	(tx ? FSL_SAI_TCSR(off) : FSL_SAI_RCSR(off))
#define FSL_SAI_xCR1(tx, off)	(tx ? FSL_SAI_TCR1(off) : FSL_SAI_RCR1(off))
#define FSL_SAI_xCR2(tx, off)	(tx ? FSL_SAI_TCR2(off) : FSL_SAI_RCR2(off))
#define FSL_SAI_xCR3(tx, off)	(tx ? FSL_SAI_TCR3(off) : FSL_SAI_RCR3(off))
#define FSL_SAI_xCR4(tx, off)	(tx ? FSL_SAI_TCR4(off) : FSL_SAI_RCR4(off))
#define FSL_SAI_xCR5(tx, off)	(tx ? FSL_SAI_TCR5(off) : FSL_SAI_RCR5(off))
#define FSL_SAI_xDR(tx)		(tx ? FSL_SAI_TDR : FSL_SAI_RDR)
#define FSL_SAI_xFR(tx)		(tx ? FSL_SAI_TFR : FSL_SAI_RFR)
#define FSL_SAI_xMR(tx)		(tx ? FSL_SAI_TMR : FSL_SAI_RMR)

/* SAI Transmit/Receive Control Register */
#define FSL_SAI_CSR_TERE	BIT(31)
#define FSL_SAI_CSR_SE		BIT(30)
#define FSL_SAI_CSR_FR		BIT(25)
#define FSL_SAI_CSR_SR		BIT(24)
#define FSL_SAI_CSR_xF_SHIFT	16
#define FSL_SAI_CSR_xF_W_SHIFT	18
#define FSL_SAI_CSR_xF_MASK	(0x1f << FSL_SAI_CSR_xF_SHIFT)
#define FSL_SAI_CSR_xF_W_MASK	(0x7 << FSL_SAI_CSR_xF_W_SHIFT)
#define FSL_SAI_CSR_WSF		BIT(20)
#define FSL_SAI_CSR_SEF		BIT(19)
#define FSL_SAI_CSR_FEF		BIT(18)
#define FSL_SAI_CSR_FWF		BIT(17)
#define FSL_SAI_CSR_FRF		BIT(16)
#define FSL_SAI_CSR_xIE_SHIFT	8
#define FSL_SAI_CSR_xIE_MASK	(0x1f << FSL_SAI_CSR_xIE_SHIFT)
#define FSL_SAI_CSR_WSIE	BIT(12)
#define FSL_SAI_CSR_SEIE	BIT(11)
#define FSL_SAI_CSR_FEIE	BIT(10)
#define FSL_SAI_CSR_FWIE	BIT(9)
#define FSL_SAI_CSR_FRIE	BIT(8)
#define FSL_SAI_CSR_FRDE	BIT(0)

/* SAI Transmit and Receive Configuration 1 Register */
#define FSL_SAI_CR1_RFW_MASK	0x1f

/* SAI Transmit and Receive Configuration 2 Register */
#define FSL_SAI_CR2_SYNC	BIT(30)
#define FSL_SAI_CR2_MSEL_MASK	(0x3 << 26)
#define FSL_SAI_CR2_MSEL_BUS	0
#define FSL_SAI_CR2_MSEL_MCLK1	BIT(26)
#define FSL_SAI_CR2_MSEL_MCLK2	BIT(27)
#define FSL_SAI_CR2_MSEL_MCLK3	(BIT(26) | BIT(27))
#define FSL_SAI_CR2_MSEL(ID)	((ID) << 26)
#define FSL_SAI_CR2_BCP		BIT(25)
#define FSL_SAI_CR2_BCD_MSTR	BIT(24)
#define FSL_SAI_CR2_BYP		BIT(23) /* BCLK bypass */
#define FSL_SAI_CR2_DIV_MASK	0xff

/* SAI Transmit and Receive Configuration 3 Register */
#define FSL_SAI_CR3_TRCE_MASK	(0xff << 16)
#define FSL_SAI_CR3_TRCE(x)	(x << 16)
#define FSL_SAI_CR3_WDFL(x)	(x)
#define FSL_SAI_CR3_WDFL_MASK	0x1f

/* SAI Transmit and Receive Configuration 4 Register */

#define FSL_SAI_CR4_FCONT	BIT(28)
#define FSL_SAI_CR4_FCOMB_SHIFT BIT(26)
#define FSL_SAI_CR4_FCOMB_SOFT  BIT(27)
#define FSL_SAI_CR4_FCOMB_MASK  (0x3 << 26)
#define FSL_SAI_CR4_FPACK_8     (0x2 << 24)
#define FSL_SAI_CR4_FPACK_16    (0x3 << 24)
#define FSL_SAI_CR4_FRSZ(x)	(((x) - 1) << 16)
#define FSL_SAI_CR4_FRSZ_MASK	(0x1f << 16)
#define FSL_SAI_CR4_SYWD(x)	(((x) - 1) << 8)
#define FSL_SAI_CR4_SYWD_MASK	(0x1f << 8)
#define FSL_SAI_CR4_CHMOD	(1 << 5)
#define FSL_SAI_CR4_CHMOD_MASK	(1 << 5)
#define FSL_SAI_CR4_MF		BIT(4)
#define FSL_SAI_CR4_FSE		BIT(3)
#define FSL_SAI_CR4_FSP		BIT(1)
#define FSL_SAI_CR4_FSD_MSTR	BIT(0)

/* SAI Transmit and Receive Configuration 5 Register */
#define FSL_SAI_CR5_WNW(x)	(((x) - 1) << 24)
#define FSL_SAI_CR5_WNW_MASK	(0x1f << 24)
#define FSL_SAI_CR5_W0W(x)	(((x) - 1) << 16)
#define FSL_SAI_CR5_W0W_MASK	(0x1f << 16)
#define FSL_SAI_CR5_FBT(x)	((x) << 8)
#define FSL_SAI_CR5_FBT_MASK	(0x1f << 8)

/* SAI MCLK Control Register */
#define FSL_SAI_MCTL_MCLK_EN	BIT(30)	/* MCLK Enable */
#define FSL_SAI_MCTL_MSEL_MASK	(0x3 << 24)
#define FSL_SAI_MCTL_MSEL(ID)   ((ID) << 24)
#define FSL_SAI_MCTL_MSEL_BUS	0
#define FSL_SAI_MCTL_MSEL_MCLK1	BIT(24)
#define FSL_SAI_MCTL_MSEL_MCLK2	BIT(25)
#define FSL_SAI_MCTL_MSEL_MCLK3	(BIT(24) | BIT(25))
#define FSL_SAI_MCTL_DIV_EN	BIT(23)
#define FSL_SAI_MCTL_DIV_MASK	0xFF

/* SAI VERID Register */
#define FSL_SAI_VER_ID_SHIFT	16
#define FSL_SAI_VER_ID_MASK	(0xFFFF << FSL_SAI_VER_ID_SHIFT)
#define FSL_SAI_VER_EFIFO_EN	BIT(0)
#define FSL_SAI_VER_TSTMP_EN	BIT(1)

/* SAI PARAM Register */
#define FSL_SAI_PAR_SPF_SHIFT	16
#define FSL_SAI_PAR_SPF_MASK	(0x0F << FSL_SAI_PAR_SPF_SHIFT)
#define FSL_SAI_PAR_WPF_SHIFT	8
#define FSL_SAI_PAR_WPF_MASK	(0x0F << FSL_SAI_PAR_WPF_SHIFT)
#define FSL_SAI_PAR_DLN_MASK	(0x0F)

/* SAI MCLK Divide Register */
#define FSL_SAI_MDIV_MASK	0xFFFFF

/* SAI type */
#define FSL_SAI_DMA		BIT(0)
#define FSL_SAI_USE_AC97	BIT(1)
#define FSL_SAI_NET		BIT(2)
#define FSL_SAI_TRA_SYN		BIT(3)
#define FSL_SAI_REC_SYN		BIT(4)
#define FSL_SAI_USE_I2S_SLAVE	BIT(5)

#define FSL_FMT_TRANSMITTER	0
#define FSL_FMT_RECEIVER	1

/* SAI clock sources */
#define FSL_SAI_CLK_BUS		0
#define FSL_SAI_CLK_MAST1	1
#define FSL_SAI_CLK_MAST2	2
#define FSL_SAI_CLK_MAST3	3

#define FSL_SAI_MCLK_MAX	4

/* SAI data transfer numbers per DMA request */
#define FSL_SAI_MAXBURST_TX 6
#define FSL_SAI_MAXBURST_RX 6

#define SAI_FLAG_PMQOS   BIT(0)

struct fsl_sai_soc_data {
	unsigned int fifo_depth;
	unsigned int fifos;
	unsigned int dataline;
	unsigned int flags;
	unsigned char reg_offset;
	bool imx;
	/* True for EDMA because it needs period size multiple of maxburst */
	bool constrain_period_size;
};

struct fsl_sai_verid {
	u32 id;
	bool timestamp_en;
	bool extfifo_en;
	bool loaded;
};

struct fsl_sai_param {
	u32 spf; /* max slots per frame */
	u32 wpf; /* words in fifo */
	u32 dln; /* number of datalines implemented */
};

struct fsl_sai {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct clk *bus_clk;
	struct clk *mclk_clk[FSL_SAI_MCLK_MAX];
	struct clk *pll8k_clk;
	struct clk *pll11k_clk;

	bool slave_mode[2];
	bool is_lsb_first;
	bool is_dsp_mode;
	bool is_multi_lane;
	bool synchronous[2];
	bool is_stream_opened[2];
	bool is_dsd;
	unsigned int dataline[2];
	unsigned int dataline_dsd[2];
	unsigned int dataline_off[2];
	unsigned int dataline_off_dsd[2];
	unsigned int masterflag[2];

	unsigned int mclk_id[2];
	unsigned int mclk_streams;
	unsigned int slots;
	unsigned int slot_width;
	unsigned int bitclk_ratio;

	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
	const struct fsl_sai_soc_data *soc;
	struct pm_qos_request pm_qos_req;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_state;

	struct fsl_sai_verid verid;
	struct fsl_sai_param param;
};

#define TX 1
#define RX 0

#endif /* __FSL_SAI_H */
