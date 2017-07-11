/*
 * fsl_dma_workaround.c - DMA workaround bits
 *
 * Copyright (C) 2017 NXP
 *
 * Author: Daniel Baluta <daniel.baluta@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/slab.h>
#include "fsl_acm.h"
#include "fsl_dma_workaround.h"

int gpt_events[2][2] = {
	{ESAI0_IPD_ESAI_TX_B, ESAI0_IPD_ESAI_RX_B},
	{SPDIF0_DRQ1_SPDIF_B, SPDIF0_DRQ0_SPDIF_B},
};

/*
 * configure_gpt_dma - configures GPT DMA for a given audio interface
 * @substream:	PCM substream
 * @info: DMA workaround specific info
 *
 */
int configure_gpt_dma(struct snd_pcm_substream *substream,
		      struct fsl_dma_workaround_info *info)
{
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	if (tx) {
		writel_relaxed(gpt_events[info->iface][0],
			       info->base_acm + GPT0_CAPIN1_SEL_OFF);
		writel_relaxed(gpt_events[info->iface][0],
			       info->base_acm + GPT1_CAPIN1_SEL_OFF);

		writel(le32_to_cpu(info->tcd_sw[0].vtcd->saddr),
				info->base_edma_gpt1 + EDMA_TCD_SADDR);
		writel(le32_to_cpu(info->tcd_sw[0].vtcd->daddr),
				info->base_edma_gpt1 + EDMA_TCD_DADDR);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->attr),
				info->base_edma_gpt1 + EDMA_TCD_ATTR);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->soff),
				info->base_edma_gpt1 + EDMA_TCD_SOFF);
		writel(le32_to_cpu(info->tcd_sw[0].vtcd->nbytes),
				info->base_edma_gpt1 + EDMA_TCD_NBYTES);
		writel(le32_to_cpu(info->tcd_sw[0].vtcd->slast),
				info->base_edma_gpt1 + EDMA_TCD_SLAST);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->citer),
				info->base_edma_gpt1 + EDMA_TCD_CITER);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->biter),
				info->base_edma_gpt1 + EDMA_TCD_BITER);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->doff),
				info->base_edma_gpt1 + EDMA_TCD_DOFF);
		writel(le32_to_cpu(info->tcd_sw[0].vtcd->dlast_sga),
				info->base_edma_gpt1 + EDMA_TCD_DLAST_SGA);
		writew(le16_to_cpu(info->tcd_sw[0].vtcd->csr),
				info->base_edma_gpt1 + EDMA_TCD_CSR);

		writel(0x0, info->base_edma_gpt1 + EDMA_CH_SBR);
		writel(0x1, info->base_edma_gpt1 + EDMA_CH_CSR);

		/* configure this gpt for dma tx */
		writel_relaxed(0x8, info->base_gpt0 + GPT_IR);
		writel_relaxed(0x7<<12, info->base_gpt0 + GPT_PR);
		writel_relaxed(0x20441, info->base_gpt0 + GPT_CR);

		/* configure this gpt for dma tx request clear */
		writel_relaxed(0x8, info->base_gpt1 + GPT_IR);
		writel_relaxed(0x7<<12, info->base_gpt1 + GPT_PR);
		writel_relaxed(0x10441, info->base_gpt1 + GPT_CR);

	} else {
		writel_relaxed(gpt_events[info->iface][1],
				info->base_acm + GPT2_CAPIN1_SEL_OFF);
		writel_relaxed(gpt_events[info->iface][1],
				info->base_acm + GPT3_CAPIN1_SEL_OFF);

		writel(le32_to_cpu(info->tcd_sw[2].vtcd->saddr),
				info->base_edma_gpt3 + EDMA_TCD_SADDR);
		writel(le32_to_cpu(info->tcd_sw[2].vtcd->daddr),
				info->base_edma_gpt3 + EDMA_TCD_DADDR);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->attr),
				info->base_edma_gpt3 + EDMA_TCD_ATTR);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->soff),
				info->base_edma_gpt3 + EDMA_TCD_SOFF);
		writel(le32_to_cpu(info->tcd_sw[2].vtcd->nbytes),
				info->base_edma_gpt3 + EDMA_TCD_NBYTES);
		writel(le32_to_cpu(info->tcd_sw[2].vtcd->slast),
				info->base_edma_gpt3 + EDMA_TCD_SLAST);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->citer),
				info->base_edma_gpt3 + EDMA_TCD_CITER);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->biter),
				info->base_edma_gpt3 + EDMA_TCD_BITER);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->doff),
				info->base_edma_gpt3 + EDMA_TCD_DOFF);
		writel(le32_to_cpu(info->tcd_sw[2].vtcd->dlast_sga),
				info->base_edma_gpt3 + EDMA_TCD_DLAST_SGA);
		writew(le16_to_cpu(info->tcd_sw[2].vtcd->csr),
				info->base_edma_gpt3 + EDMA_TCD_CSR);

		writel(0x0, info->base_edma_gpt3 + EDMA_CH_SBR);
		writel(0x1, info->base_edma_gpt3 + EDMA_CH_CSR);

		/* configure this gpt for dma rx */
		writel_relaxed(0x8, info->base_gpt2 + GPT_IR);
		writel_relaxed(0x7<<12, info->base_gpt2 + GPT_PR);
		writel_relaxed(0x20441, info->base_gpt2 + GPT_CR);

		/* configure this gpt for dma rx request clear*/
		writel_relaxed(0x8, info->base_gpt3 + GPT_IR);
		writel_relaxed(0x7<<12, info->base_gpt3 + GPT_PR);
		writel_relaxed(0x10441, info->base_gpt3 + GPT_CR);

	}

	return 0;
}


int clear_gpt_dma(struct snd_pcm_substream *substream,
			 struct fsl_dma_workaround_info *info)
{
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 val;

	if (tx) {
		val = readl(info->base_edma_gpt1 + EDMA_CH_CSR);
		val &= ~0x1;
		writel(val, info->base_edma_gpt1 + EDMA_CH_CSR);

		/* disable gpt */
		writel_relaxed(0, info->base_gpt0 + GPT_IR);
		writel_relaxed(0, info->base_gpt0 + GPT_PR);
		writel_relaxed(0, info->base_gpt0 + GPT_CR);

		writel_relaxed(0, info->base_gpt1 + GPT_IR);
		writel_relaxed(0, info->base_gpt1 + GPT_PR);
		writel_relaxed(0, info->base_gpt1 + GPT_CR);

	} else {
		val = readl(info->base_edma_gpt3 + EDMA_CH_CSR);
		val &= ~0x1;
		writel(val, info->base_edma_gpt3 + EDMA_CH_CSR);

		/* disable gpt */
		writel_relaxed(0, info->base_gpt2 + GPT_IR);
		writel_relaxed(0, info->base_gpt2 + GPT_PR);
		writel_relaxed(0, info->base_gpt2 + GPT_CR);

		writel_relaxed(0, info->base_gpt3 + GPT_IR);
		writel_relaxed(0, info->base_gpt3 + GPT_PR);
		writel_relaxed(0, info->base_gpt3 + GPT_CR);
	}

	return 0;
}

struct fsl_dma_workaround_info *
fsl_dma_workaround_alloc_info(const char *pool_name, struct device *dma_dev,
			      const char *base_acm_compat, int iface)
{
	struct fsl_dma_workaround_info *info;
	int *buffer;
	int i;

	info = devm_kzalloc(dma_dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return ERR_PTR(-ENOMEM);

	info->tcd_pool = dma_pool_create(pool_name, dma_dev,
					 sizeof(struct fsl_edma3_hw_tcd),
					 32, 0);

	info->buf.area = dma_alloc_writecombine(dma_dev, 0x1000,
						&info->buf.addr, GFP_KERNEL);

	buffer = (int *)info->buf.area;
	buffer[0] = 0x8;

	info->tcd_sw[0].vtcd = dma_pool_alloc(info->tcd_pool,
			GFP_ATOMIC, &info->tcd_sw[0].ptcd);
	info->tcd_sw[1].vtcd = dma_pool_alloc(info->tcd_pool,
			GFP_ATOMIC, &info->tcd_sw[1].ptcd);
	info->tcd_sw[2].vtcd = dma_pool_alloc(info->tcd_pool,
			GFP_ATOMIC, &info->tcd_sw[2].ptcd);
	info->tcd_sw[3].vtcd = dma_pool_alloc(info->tcd_pool,
			GFP_ATOMIC, &info->tcd_sw[3].ptcd);

	for (i = 0; i < 4; i++) {
		info->tcd_sw[i].vtcd->saddr = info->buf.addr;
		info->tcd_sw[i].vtcd->attr  = 0x0202;
		info->tcd_sw[i].vtcd->soff  = 0x0;
		info->tcd_sw[i].vtcd->nbytes = 0x4;
		info->tcd_sw[i].vtcd->slast     = 0x0;
		info->tcd_sw[i].vtcd->citer     = 0x1;
		info->tcd_sw[i].vtcd->biter     = 0x1;
		info->tcd_sw[i].vtcd->doff      = 0x0;
		info->tcd_sw[i].vtcd->csr       = 0x10;
	}

	info->tcd_sw[0].vtcd->daddr = GPT5_ADDR + GPT_SR;
	info->tcd_sw[1].vtcd->daddr = GPT6_ADDR + GPT_SR;
	info->tcd_sw[2].vtcd->daddr = GPT7_ADDR + GPT_SR;
	info->tcd_sw[3].vtcd->daddr = GPT8_ADDR + GPT_SR;

	info->tcd_sw[0].vtcd->dlast_sga =
				info->tcd_sw[1].ptcd;
	info->tcd_sw[1].vtcd->dlast_sga =
				info->tcd_sw[0].ptcd;
	info->tcd_sw[2].vtcd->dlast_sga =
				info->tcd_sw[3].ptcd;
	info->tcd_sw[3].vtcd->dlast_sga =
				info->tcd_sw[2].ptcd;

	info->base_gpt0 = ioremap(GPT5_ADDR, SZ_64K);
	info->base_gpt1 = ioremap(GPT6_ADDR, SZ_64K);
	info->base_gpt2 = ioremap(GPT7_ADDR, SZ_64K);
	info->base_gpt3 = ioremap(GPT8_ADDR, SZ_64K);

	info->base_edma_gpt1 = ioremap(EDMA_GPT6_ADDR, SZ_64K);
	info->base_edma_gpt3 = ioremap(EDMA_GPT8_ADDR, SZ_64K);

	info->base_acm = of_iomap(of_find_compatible_node(
				NULL, NULL, base_acm_compat), 0);

	info->iface = iface;
	return info;
}

void fsl_dma_workaround_free_info(struct fsl_dma_workaround_info *info,
				  struct device *dma_dev)
{
	dma_free_writecombine(dma_dev,
			      0x1000,
			      info->buf.area,
			      info->buf.addr);

	dma_pool_free(info->tcd_pool,
		      info->tcd_sw[0].vtcd,
		      info->tcd_sw[0].ptcd);
	dma_pool_free(info->tcd_pool,
		      info->tcd_sw[1].vtcd,
		      info->tcd_sw[1].ptcd);
	dma_pool_free(info->tcd_pool,
		      info->tcd_sw[2].vtcd,
		      info->tcd_sw[2].ptcd);
	dma_pool_free(info->tcd_pool,
		      info->tcd_sw[3].vtcd,
		      info->tcd_sw[3].ptcd);

	dma_pool_destroy(info->tcd_pool);
}
