/*
 * fsl_dma_workaround.h - EDMA bits useful for DMA workaround
 *
 * Copyright (C) 2017 NXP
 *
 * Author: Daniel Baluta <daniel.baluta@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef _FSL_DMA_WORKAROUND_H
#define _FSL_DMA_WORKAROUND_H

#include <sound/pcm.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define EDMA_CH_CSR			0x00
#define EDMA_CH_ES			0x04
#define EDMA_CH_INT			0x08
#define EDMA_CH_SBR			0x0C
#define EDMA_CH_PRI			0x10
#define EDMA_TCD_SADDR			0x20
#define EDMA_TCD_SOFF			0x24
#define EDMA_TCD_ATTR			0x26
#define EDMA_TCD_NBYTES			0x28
#define EDMA_TCD_SLAST			0x2C
#define EDMA_TCD_DADDR			0x30
#define EDMA_TCD_DOFF			0x34
#define EDMA_TCD_CITER_ELINK		0x36
#define EDMA_TCD_CITER			0x36
#define EDMA_TCD_DLAST_SGA		0x38
#define EDMA_TCD_CSR			0x3C
#define EDMA_TCD_BITER_ELINK		0x3E
#define EDMA_TCD_BITER			0x3E

#define GPT_CR				0x00
#define GPT_PR				0x04
#define GPT_SR				0x08
#define GPT_IR				0x0C

#define GPT5_ADDR			0x590b0000
#define GPT6_ADDR			0x590c0000
#define GPT7_ADDR			0x590d0000
#define GPT8_ADDR			0x590e0000

#define EDMA_GPT6_ADDR			0x59360000
#define EDMA_GPT8_ADDR			0x59380000

#define FSL_DMA_WORKAROUND_ESAI		0
#define FSL_DMA_WORKAROUND_SPDIF	1

struct fsl_edma3_hw_tcd {
	__le32	saddr;
	__le16	soff;
	__le16	attr;
	__le32	nbytes;
	__le32	slast;
	__le32	daddr;
	__le16	doff;
	__le16	citer;
	__le32	dlast_sga;
	__le16	csr;
	__le16	biter;
};

struct fsl_edma3_sw_tcd {
	dma_addr_t			ptcd;
	struct fsl_edma3_hw_tcd		*vtcd;
};

struct fsl_dma_workaround_info {
	struct fsl_edma3_sw_tcd	tcd_sw[4];
	struct dma_pool	*tcd_pool;
	struct snd_dma_buffer buf;
	void __iomem *base_gpt0;
	void __iomem *base_gpt1;
	void __iomem *base_gpt2;
	void __iomem *base_gpt3;
	void __iomem *base_edma_gpt1;
	void __iomem *base_edma_gpt3;
	void __iomem *base_acm;
	int iface;
};

int configure_gpt_dma(struct snd_pcm_substream *substream,
		      struct fsl_dma_workaround_info *info);

int clear_gpt_dma(struct snd_pcm_substream *substream,
		  struct fsl_dma_workaround_info *info);

struct fsl_dma_workaround_info *
fsl_dma_workaround_alloc_info(const char *pool_name, struct device *dma_dev,
			      const char *base_acm_compat, int iface);

void fsl_dma_workaround_free_info(struct fsl_dma_workaround_info *info,
				  struct device *dma_dev);
#endif /* _FSL_EDMA_H */
