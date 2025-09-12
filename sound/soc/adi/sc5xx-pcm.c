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
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/rpmsg.h>

#include "sc5xx-sport.h"

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

static const struct snd_soc_component_driver sc5xx_pcm_component = {
	.open		= sc5xx_pcm_open,
	.prepare	= sc5xx_pcm_prepare,
	.trigger	= sc5xx_pcm_trigger,
	.pointer	= sc5xx_pcm_pointer,
	.pcm_construct	= sc5xx_pcm_new,
};

static int sc5xx_soc_platform_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev, &sc5xx_pcm_component, NULL, 0);
}

static struct platform_driver sc5xx_pcm_driver = {
	.driver = {
		.name = "sc5xx-pcm-audio",
	},
	.probe = sc5xx_soc_platform_probe,
};

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
static struct rpmsg_device_id rpmsg_icap_sport_id_table[] = {
	{ .name = "icap-sport0-core1" },
	{ .name = "icap-sport1-core1" },
	{ .name = "icap-sport2-core1" },
	{ .name = "icap-sport3-core1" },
	{ .name = "icap-sport4-core1" },
	{ .name = "icap-sport5-core1" },
	{ .name = "icap-sport6-core1" },
	{ .name = "icap-sport7-core1" },
	{ .name = "icap-sport0-core2" },
	{ .name = "icap-sport1-core2" },
	{ .name = "icap-sport2-core2" },
	{ .name = "icap-sport3-core2" },
	{ .name = "icap-sport4-core2" },
	{ .name = "icap-sport5-core2" },
	{ .name = "icap-sport6-core2" },
	{ .name = "icap-sport7-core2" },
	{ },
};
static struct rpmsg_driver rpmsg_icap_sport = {
	.drv.name  = KBUILD_MODNAME,
	.drv.owner = THIS_MODULE,
	.id_table  = rpmsg_icap_sport_id_table,
	.probe     = rpmsg_icap_sport_probe,
	.callback  = rpmsg_icap_sport_cb,
	.remove    = rpmsg_icap_sport_remove,
};
#endif

static struct platform_device *sc5xx_pcm_dev;

static int sc5xx_pcm_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&sc5xx_pcm_driver);
	if (ret < 0) {
		pr_err("sc5xx_pcm_driver: failed to register pcm driver\n");
		return ret;
	}

	sc5xx_pcm_dev =	platform_device_register_simple("sc5xx-pcm-audio", -1, NULL, 0);
	if (IS_ERR(sc5xx_pcm_dev)) {
		pr_err("sc5xx_pcm_driver: failed to register pcm device\n");
		platform_driver_unregister(&sc5xx_pcm_driver);
		return PTR_ERR(sc5xx_pcm_dev);
	}

#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
	ret = register_rpmsg_driver(&rpmsg_icap_sport);
	if (ret < 0) {
		pr_err("sc5xx_pcm_driver: failed to register rpmsg driver\n");
		platform_device_unregister(sc5xx_pcm_dev);
		platform_driver_unregister(&sc5xx_pcm_driver);
		return ret;
	}
#endif

	return 0;
}
module_init(sc5xx_pcm_driver_init);

static void sc5xx_pcm_driver_exit(void)
{
	platform_device_unregister(sc5xx_pcm_dev);
	platform_driver_unregister(&sc5xx_pcm_driver);
#if IS_ENABLED(CONFIG_SND_SC5XX_SPORT_SHARC)
	unregister_rpmsg_driver(&rpmsg_icap_sport);
#endif
}
module_exit(sc5xx_pcm_driver_exit);

MODULE_DESCRIPTION("Analog Devices SC5XX audio dma driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
