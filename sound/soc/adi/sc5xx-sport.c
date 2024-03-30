// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SC5XX SPORT driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Scott Jiang <Scott.Jiang.Linux@gmail.com>
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/gpio.h>

#include <linux/soc/adi/cpu.h>
#include "sc5xx-sport.h"

int sport_set_tx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI)
		return -EBUSY;
	iowrite32(params->spctl | SPORT_CTL_SPTRAN, &sport->tx_regs->spctl);
	iowrite32(params->div, &sport->tx_regs->div);
	iowrite32(params->spmctl, &sport->tx_regs->spmctl);
	iowrite32(params->spcs0, &sport->tx_regs->spcs0);
	return 0;
}
EXPORT_SYMBOL(sport_set_tx_params);

int sport_set_rx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->rx_regs->spctl) & SPORT_CTL_SPENPRI)
		return -EBUSY;
	iowrite32(params->spctl & ~SPORT_CTL_SPTRAN, &sport->rx_regs->spctl);
	iowrite32(params->div, &sport->rx_regs->div);
	iowrite32(params->spmctl, &sport->rx_regs->spmctl);
	iowrite32(params->spcs0, &sport->rx_regs->spcs0);
	return 0;
}
EXPORT_SYMBOL(sport_set_rx_params);

int sport_tx_start(struct sport_device *sport)
{
	sport->tx_cookie = dmaengine_submit(sport->tx_desc);
	dma_async_issue_pending(sport->tx_dma_chan);
	iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	return 0;
}
EXPORT_SYMBOL(sport_tx_start);

int sport_rx_start(struct sport_device *sport)
{
	sport->rx_cookie = dmaengine_submit(sport->rx_desc);
	dma_async_issue_pending(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	return 0;
}
EXPORT_SYMBOL(sport_rx_start);

int sport_tx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	dmaengine_terminate_sync(sport->tx_dma_chan);
	sport->tx_cookie = 0;
	sport->tx_desc = NULL;
	return 0;
}
EXPORT_SYMBOL(sport_tx_stop);

int sport_rx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	dmaengine_terminate_sync(sport->rx_dma_chan);
	sport->rx_cookie = 0;
	sport->rx_desc = NULL;
	return 0;
}
EXPORT_SYMBOL(sport_rx_stop);

void sport_set_tx_callback(struct sport_device *sport,
		void (*tx_callback)(void *), void *tx_data)
{
	sport->tx_callback = tx_callback;
	sport->tx_data = tx_data;
}
EXPORT_SYMBOL(sport_set_tx_callback);

void sport_set_rx_callback(struct sport_device *sport,
		void (*rx_callback)(void *), void *rx_data)
{
	sport->rx_callback = rx_callback;
	sport->rx_data = rx_data;
}
EXPORT_SYMBOL(sport_set_rx_callback);

void sport_tx_dma_callback(void *ptr)
{
	struct sport_device *sport = ptr;

	sport->tx_count += 1;
	if (sport->tx_count >= sport->tx_frags)
		sport->tx_count = 0;
	sport->tx_callback(sport->tx_data);
}

void sport_rx_dma_callback(void *ptr)
{
	struct sport_device *sport = ptr;

	sport->rx_count += 1;
	if (sport->rx_count >= sport->rx_frags)
		sport->rx_count = 0;
	sport->rx_callback(sport->rx_data);
}

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	struct dma_slave_config dma_config = {0};
	size_t total = fragsize * fragcount;
	int ret;

	if (sport->tx_desc)
		dmaengine_terminate_sync(sport->tx_dma_chan);

	dma_config.direction = DMA_MEM_TO_DEV;
	dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.src_maxburst = sport->wdsize;
	dma_config.dst_maxburst = sport->wdsize;
	ret = dmaengine_slave_config(sport->tx_dma_chan, &dma_config);
	if (ret) {
		dev_err(&sport->pdev->dev, "tx dma slave config failed: %d\n", ret);
		return ret;
	}

	sport->tx_desc = dmaengine_prep_dma_cyclic(sport->tx_dma_chan,
		(dma_addr_t) buf, total, fragsize, DMA_MEM_TO_DEV,
		DMA_PREP_INTERRUPT);

	sport->tx_desc->callback = sport_tx_dma_callback;
	sport->tx_desc->callback_param = sport;

	sport->tx_buf = (dma_addr_t)buf;
	sport->tx_fragsize = fragsize;
	sport->tx_frags = fragcount;
	sport->tx_totalsize = total;
	sport->tx_count = 0;

	sport->tx_substream = substream;

	return 0;
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	struct dma_slave_config dma_config = {0};
	size_t total = fragcount * fragsize;
	int ret;

	if (sport->rx_desc)
		dmaengine_terminate_sync(sport->rx_dma_chan);

	dma_config.direction = DMA_DEV_TO_MEM;
	dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.src_maxburst = sport->wdsize;
	dma_config.dst_maxburst = sport->wdsize;
	ret = dmaengine_slave_config(sport->rx_dma_chan, &dma_config);
	if (ret) {
		dev_err(&sport->pdev->dev, "rx dma slave config failed: %d\n", ret);
		return ret;
	}

	sport->rx_desc = dmaengine_prep_dma_cyclic(sport->rx_dma_chan,
		(dma_addr_t) buf, total, fragsize, DMA_DEV_TO_MEM,
		DMA_PREP_INTERRUPT);

	sport->rx_desc->callback = sport_rx_dma_callback;
	sport->rx_desc->callback_param = sport;

	sport->rx_buf = (dma_addr_t)buf;
	sport->rx_fragsize = fragsize;
	sport->rx_frags = fragcount;
	sport->rx_totalsize = total;
	sport->rx_count = 0;

	sport->rx_substream = substream;

	return 0;
}
EXPORT_SYMBOL(sport_config_rx_dma);

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	return sport->tx_count * sport->tx_fragsize;
}
EXPORT_SYMBOL(sport_curr_offset_tx);

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	return sport->rx_count * sport->rx_fragsize;
}
EXPORT_SYMBOL(sport_curr_offset_rx);

static int sport_get_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	struct resource *res;

	if (!dev->of_node) {
		dev_err(dev, "No device tree node\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No tx MEM resource\n");
		return -ENODEV;
	}
	sport->tx_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(sport->tx_regs)) {
		dev_err(dev, "Failed to map tx registers\n");
		return PTR_ERR(sport->tx_regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "No rx MEM resource\n");
		return -ENODEV;
	}
	sport->rx_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(sport->rx_regs)) {
		dev_err(dev, "Failed to map rx registers\n");
		return PTR_ERR(sport->rx_regs);
	}

	return 0;
}

static int sport_request_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	sport->tx_dma_chan = dma_request_chan(dev, "tx");
	if (IS_ERR(sport->tx_dma_chan)) {
		dev_err(dev, "Missing `tx` dma channel: %ld\n", PTR_ERR(sport->tx_dma_chan));
		return PTR_ERR(sport->tx_dma_chan);
	}

	sport->rx_dma_chan = dma_request_chan(dev, "rx");
	if (IS_ERR(sport->rx_dma_chan)) {
		dev_err(dev, "Missing `rx` dma channel: %ld\n", PTR_ERR(sport->rx_dma_chan));
		ret = PTR_ERR(sport->rx_dma_chan);
		goto err_rx_dma;
	}

	return 0;

err_rx_dma:
	dma_release_channel(sport->tx_dma_chan);
	return ret;
}

static void sport_free_resource(struct sport_device *sport)
{
	dma_release_channel(sport->tx_dma_chan);
	dma_release_channel(sport->rx_dma_chan);
}
struct sport_device *sport_create(struct platform_device *pdev)
{
	struct sport_device *sport;
	int ret;

	sport = kzalloc(sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return ERR_PTR(-ENOMEM);

	sport->pdev = pdev;

	ret = sport_get_resource(sport);
	if (ret)
		goto err_free_data;

	ret = sport_request_resource(sport);
	if (ret)
		goto err_free_data;

	return sport;

err_free_data:
	kfree(sport);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(sport_create);

void sport_delete(struct sport_device *sport)
{
	dmaengine_terminate_sync(sport->tx_dma_chan);
	dmaengine_terminate_sync(sport->rx_dma_chan);
	sport_free_resource(sport);
	kfree(sport);
}
EXPORT_SYMBOL(sport_delete);

MODULE_DESCRIPTION("Analog Devices SC5XX SPORT driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
