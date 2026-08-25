// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SC5XX audio dma driver
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#include <sound/sc5xx-dai.h>

#include "sc5xx-sport.h"

static struct sport_params param;

static int sport_set_tx_params(struct sport_device *sport,
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

static int sport_set_rx_params(struct sport_device *sport,
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

static int sport_tx_start(struct sport_device *sport)
{
	sport->tx_cookie = dmaengine_submit(sport->tx_desc);
	dma_async_issue_pending(sport->tx_dma_chan);
	iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	return 0;
}

static int sport_rx_start(struct sport_device *sport)
{
	sport->rx_cookie = dmaengine_submit(sport->rx_desc);
	dma_async_issue_pending(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	return 0;
}

static int sport_tx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	dmaengine_terminate_sync(sport->tx_dma_chan);
	sport->tx_cookie = 0;
	sport->tx_desc = NULL;
	return 0;
}

static int sport_rx_stop(struct sport_device *sport)
{
	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	dmaengine_terminate_sync(sport->rx_dma_chan);
	sport->rx_cookie = 0;
	sport->rx_desc = NULL;
	return 0;
}

static void sport_set_tx_callback(struct sport_device *sport,
		void (*tx_callback)(void *), void *tx_data)
{
	sport->tx_callback = tx_callback;
	sport->tx_data = tx_data;
}

static void sport_set_rx_callback(struct sport_device *sport,
		void (*rx_callback)(void *), void *rx_data)
{
	sport->rx_callback = rx_callback;
	sport->rx_data = rx_data;
}

static void sport_tx_dma_callback(void *ptr)
{
	struct sport_device *sport = ptr;

	sport->tx_count += 1;
	if (sport->tx_count >= sport->tx_frags)
		sport->tx_count = 0;
	sport->tx_callback(sport->tx_data);
}

static void sport_rx_dma_callback(void *ptr)
{
	struct sport_device *sport = ptr;

	sport->rx_count += 1;
	if (sport->rx_count >= sport->rx_frags)
		sport->rx_count = 0;
	sport->rx_callback(sport->rx_data);
}

static int sport_config_tx_dma(struct sport_device *sport, void *buf,
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

static int sport_config_rx_dma(struct sport_device *sport, void *buf,
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

static unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	return sport->tx_count * sport->tx_fragsize;
}

static unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	return sport->rx_count * sport->rx_fragsize;
}

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
static struct sport_device *sport_create(struct platform_device *pdev)
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

static void sport_delete(struct sport_device *sport)
{
	dmaengine_terminate_sync(sport->tx_dma_chan);
	dmaengine_terminate_sync(sport->rx_dma_chan);
	sport_free_resource(sport);
	kfree(sport);
}

static void sc5xx_dma_irq(void *data)
{
	struct snd_pcm_substream *pcm = data;

	snd_pcm_period_elapsed(pcm);
}

static const struct snd_pcm_hardware sc5xx_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_MMAP_VALID |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 0x10000,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/32,
	.buffer_bytes_max	= 0x20000, /* 128 kbytes */
	.fifo_size		= 16,
};

static int sc5xx_pcm_prepare(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	int period_bytes = frames_to_bytes(runtime, runtime->period_size);
	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sport_set_tx_callback(sport, sc5xx_dma_irq, substream);
		ret = sport_config_tx_dma(sport, (void *)runtime->dma_addr,
			runtime->periods, period_bytes, substream);
	} else {
		sport_set_rx_callback(sport, sc5xx_dma_irq, substream);
		ret = sport_config_rx_dma(sport, (void *)runtime->dma_addr,
			runtime->periods, period_bytes, substream);
	}

	return ret;
}

static int sc5xx_pcm_trigger(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ret = sport_tx_start(sport);
		else
			ret = sport_rx_start(sport);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			ret = sport_tx_stop(sport);
		else
			ret = sport_rx_stop(sport);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t sc5xx_pcm_pointer(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sport_device *sport = runtime->private_data;
	unsigned int diff;
	snd_pcm_uframes_t frames;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		diff = sport_curr_offset_tx(sport);
	else
		diff = sport_curr_offset_rx(sport);

	/*
	 * TX at least can report one frame beyond the end of the
	 * buffer if we hit the wraparound case - clamp to within the
	 * buffer as the ALSA APIs require.
	 */
	if (diff == snd_pcm_lib_buffer_bytes(substream))
		diff = 0;

	frames = bytes_to_frames(substream->runtime, diff);

	return frames;
}

static int sc5xx_pcm_open(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sport_device *sport = snd_soc_dai_get_drvdata(cpu_dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &sc5xx_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sport->tx_buf = (dma_addr_t)buf->area;
	else
		sport->rx_buf = (dma_addr_t)buf->area;

	runtime->private_data = sport;
	return 0;
}

static int sc5xx_pcm_new(struct snd_soc_component *component,
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	size_t size = sc5xx_pcm_hardware.buffer_bytes_max;
	int ret = 0;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* Prefers iram pool, if not available it fallbacks to CMA */
	snd_pcm_set_managed_buffer_all(rtd->pcm,
				SNDRV_DMA_TYPE_DEV_IRAM, card->dev, size, size);
	return 0;
}

static int sc5xx_dai_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(cpu_dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~(SPORT_CTL_OPMODE | SPORT_CTL_CKRE | SPORT_CTL_FSR
			| SPORT_CTL_LFS | SPORT_CTL_LAFS);
	param.spmctl &= ~(SPORT_MCTL_MCE);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		param.spctl |= SPORT_CTL_OPMODE | SPORT_CTL_CKRE
			| SPORT_CTL_LFS;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		param.spctl |= SPORT_CTL_FSR;
		param.spmctl |= SPORT_MCTL_MCE | SPORT_MCTL_MCPDE
			| (0x10 & SPORT_MCTL_MFD);
		param.spcs0 = 0xff;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		param.spctl |= SPORT_CTL_OPMODE | SPORT_CTL_LFS
			| SPORT_CTL_LAFS;
		break;
	default:
		dev_err(dev, "%s: Unknown DAI format type\n", __func__);
		ret = -EINVAL;
		break;
	}

	param.spctl &= ~(SPORT_CTL_ICLK | SPORT_CTL_IFS);
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BC_FC:
		break;
	case SND_SOC_DAIFMT_BP_FP:
	case SND_SOC_DAIFMT_BC_FP:
	case SND_SOC_DAIFMT_BP_FC:
		ret = -EINVAL;
		dev_err(dev, "%s: clock provider modes are not implemented\n",
			__func__);
		break;
	default:
		dev_err(dev, "%s: Unknown DAI master type\n", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int sc5xx_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct sport_device *sport = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &sport->pdev->dev;
	int ret = 0;

	param.spctl &= ~SPORT_CTL_SLEN;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		param.spctl |= 0x70;
		sport->wdsize = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		param.spctl |= 0xf0;
		sport->wdsize = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		param.spctl |= 0x170;
		sport->wdsize = 3;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		param.spctl |= 0x1f0;
		sport->wdsize = 4;
		break;
	}

	/* set window size in SPORT_MCTL register */
	param.spmctl &= ~SPORT_MCTL_WSIZE;
	if (param.spmctl && SPORT_MCTL_MCE)
		param.spmctl |= (((params_channels(params) - 1) << 8)
					& SPORT_MCTL_WSIZE);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sport->tx_hw_params = *params;
		ret = sport_set_tx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT tx is busy!\n");
			return ret;
		}
	} else {
		sport->rx_hw_params = *params;
		ret = sport_set_rx_params(sport, &param);
		if (ret) {
			dev_err(dev, "SPORT rx is busy!\n");
			return ret;
		}
	}
	return 0;
}

#ifdef CONFIG_PM
static int sc5xx_dai_suspend(struct snd_soc_component *component)
{
	struct sport_device *sport = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;
	int stream;

	for_each_component_dais(component, dai) {
		for_each_pcm_streams(stream) {
			if (snd_soc_dai_stream_active(dai, stream)) {
				if (stream == SNDRV_PCM_STREAM_CAPTURE)
					sport_rx_stop(sport);
				else if (stream == SNDRV_PCM_STREAM_PLAYBACK)
					sport_tx_stop(sport);
			}
		}
	}

	return 0;
}

static int sc5xx_dai_resume(struct snd_soc_component *component)
{
	struct sport_device *sport = snd_soc_component_get_drvdata(component);
	struct device *dev = &sport->pdev->dev;
	int ret;

	ret = sport_set_tx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT tx is busy!\n");
		return ret;
	}
	ret = sport_set_rx_params(sport, &param);
	if (ret) {
		dev_err(dev, "SPORT rx is busy!\n");
		return ret;
	}

	return 0;
}
#else
#define sc5xx_dai_suspend NULL
#define sc5xx_dai_resume NULL
#endif

#define SC5XX_DAI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
		SNDRV_PCM_RATE_192000)

#define SC5XX_DAI_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops sc5xx_i2s_dai_ops = {
	.hw_params	= sc5xx_dai_hw_params,
	.set_fmt	= sc5xx_dai_set_dai_fmt,
};

static struct snd_soc_dai_driver sc5xx_i2s_dai = {
	.name = "sc5xx",
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SC5XX_DAI_RATES,
		.formats = SC5XX_DAI_FORMATS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		.rates = SC5XX_DAI_RATES,
		.formats = SC5XX_DAI_FORMATS,
	},
	.ops = &sc5xx_i2s_dai_ops,
};

static const struct snd_soc_component_driver sc5xx_pcm_component = {
	.name		= "sc5xx-i2s",
	.open		= sc5xx_pcm_open,
	.prepare	= sc5xx_pcm_prepare,
	.trigger	= sc5xx_pcm_trigger,
	.pointer	= sc5xx_pcm_pointer,
	/*
	 * Needs s/pcm_construct/pcm_new/ when rebased to v7.1-rc1 or later. See
	 * commit tags/v7.1-rc1~166^2~5^2~86^2 ("ASoC: soc-component: remove
	 * pcm_construct()/pcm_destruct()")
	 */
	.pcm_construct	= sc5xx_pcm_new,
	.suspend	= sc5xx_dai_suspend,
	.resume		= sc5xx_dai_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id sc5xx_audio_of_match[] = {
	{
		.compatible = "adi,sc5xx-i2s-dai",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sc5xx_audio_of_match);
#endif

static int sc5xx_dai_probe(struct platform_device *pdev)
{
	struct sport_device *sport;
	struct device *dev = &pdev->dev;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "sclk");
	if (IS_ERR(clk)) {
		dev_err(dev, "Missing clock node `sclk` for i2s\n");
		return PTR_ERR(clk);
	}

	sport = sport_create(pdev);
	if (IS_ERR(sport))
		return PTR_ERR(sport);

	sport->clk = clk;
	clk_prepare_enable(clk);

	ret = devm_snd_soc_register_component(dev, &sc5xx_pcm_component,
					 &sc5xx_i2s_dai, 1);
	if (ret)
		goto cleanup;

	platform_set_drvdata(pdev, sport);
	return 0;

cleanup:
	sport_delete(sport);
	clk_disable_unprepare(clk);
	return ret;
}

static void sc5xx_dai_remove(struct platform_device *pdev)
{
	struct sport_device *sport = platform_get_drvdata(pdev);

	sport_delete(sport);
	clk_disable_unprepare(sport->clk);
}

static struct platform_driver sc5xx_i2s_dai_driver = {
	.probe  = sc5xx_dai_probe,
	.remove = sc5xx_dai_remove,
	.driver = {
		.name = "sc5xx-i2s-dai",
		.of_match_table = of_match_ptr(sc5xx_audio_of_match),
	},
};

module_platform_driver(sc5xx_i2s_dai_driver);

MODULE_DESCRIPTION("Analog Devices SC5XX audio dma driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
