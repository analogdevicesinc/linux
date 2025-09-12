// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SHARC-ALSA ASoC card driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/rpmsg.h>
#include <linux/workqueue.h>

#include "icap/include/icap_application.h"

#define SHARC_ALSA_SUBDEV_MAX 16

enum sa_state {
	SHARC_ALSA_STOPPED = 0,
	SHARC_ALSA_RUNNING = 1,
	SHARC_ALSA_STOPPING = 2,
	SHARC_ALSA_STARTING = 3,
};

struct sa_subdev {
	spinlock_t buf_pos_lock;
	u32 state;
	size_t buf_frags;
	size_t buf_frag_pos;
	u32 buf_id;
	struct snd_pcm_substream *substream;
	struct wait_queue_head pending_stop_event;
};

struct sa_card_data {
	struct device *dev;
	struct rpmsg_device *rpdev;
	struct icap_instance icap;

	u32 subdev_num;
	struct sa_subdev subdevs[SHARC_ALSA_SUBDEV_MAX];

	u32 card_id;
	char card_name[64];
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_links[SHARC_ALSA_SUBDEV_MAX];

	struct snd_soc_dai_link_component cpu_links[SHARC_ALSA_SUBDEV_MAX];
	struct snd_soc_dai_link_component codec_links[SHARC_ALSA_SUBDEV_MAX];
	struct snd_soc_dai_link_component platform_links[SHARC_ALSA_SUBDEV_MAX];

	struct platform_device *asoc_cpu_devs[SHARC_ALSA_SUBDEV_MAX];
	struct platform_device *asoc_codec_devs[SHARC_ALSA_SUBDEV_MAX];
	struct platform_device *asoc_platform_devs[SHARC_ALSA_SUBDEV_MAX];

	u32 cpu_num;
	u32 codec_num;
	u32 platform_num;

	struct work_struct delayed_probe_work;
	struct work_struct send_start_work;
	struct work_struct send_stop_work;

	spinlock_t start_stop_spinlock;
	u32 stopping;
	u32 starting;

	int card_registered;
};

struct sa_comp_data {
	char dai_driver_name[64];
	struct snd_soc_dai_driver dai_driver;
	char component_driver_name[64];
	struct snd_soc_component_driver component_driver;
};

#define SHARC_ALSA_RATES	SNDRV_PCM_RATE_8000_192000
#define SHARC_ALSA_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE | \
			SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

static const struct snd_pcm_hardware sa_pcm_params = {
	.info =	SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats =	SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 0x10000,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/32,
	.buffer_bytes_max	= 0x20000, /* 128 kbytes */
	.fifo_size			= 16,
};

static int sa_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sa_card_data *sa = snd_soc_card_get_drvdata(card);
	union icap_remote_addr src_addr;
	int ret;

	src_addr.rpmsg_addr = src;
	ret = icap_parse_msg(&sa->icap, &src_addr, data, len);
	if (ret && (ret != -ICAP_ERROR_INIT)) {
		if (ret == -ICAP_ERROR_TIMEOUT)
			dev_notice_ratelimited(&rpdev->dev, "ICAP late response\n");
		else
			dev_err_ratelimited(&rpdev->dev, "ICAP parse msg error: %d\n", ret);
	}
	return 0;
}

static int sa_pcm_hw_params(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	size_t size = params_buffer_bytes(params);

	snd_pcm_lib_malloc_pages(substream, size);
	return 0;
}

static int sa_pcm_hw_free(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	snd_pcm_lib_free_pages(substream);
	return 0;
}

static snd_pcm_uframes_t sa_pcm_pointer(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sa_card_data *sa = runtime->private_data;
	int subdev_id = substream->pcm->device;
	struct sa_subdev *subdev = &sa->subdevs[subdev_id];
	unsigned long flags;
	unsigned int period;
	snd_pcm_uframes_t frames;

	spin_lock_irqsave(&subdev->buf_pos_lock, flags);
	period = subdev->buf_frag_pos;
	spin_unlock_irqrestore(&subdev->buf_pos_lock, flags);

	frames = period * runtime->period_size;
	return frames;
}

static int sa_pcm_open(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sa_card_data *sa = snd_soc_card_get_drvdata(rtd->card);

	snd_soc_set_runtime_hwparams(substream, &sa_pcm_params);

	runtime->private_data = sa;
	return 0;
}

static int sa_pcm_prepare(struct snd_soc_component *component,
	struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sa_card_data *sa = runtime->private_data;
	struct icap_instance *icap = &sa->icap;
	int period_bytes = frames_to_bytes(runtime, runtime->period_size);
	struct icap_buf_descriptor icap_buf;
	int subdev_id = substream->pcm->device;
	struct sa_subdev *subdev = &sa->subdevs[subdev_id];
	int ret = 0;

	ret = wait_event_interruptible(subdev->pending_stop_event,
				       subdev->state == SHARC_ALSA_STOPPED);
	if (ret)
		return ret;

	subdev->substream = substream;
	subdev->buf_frag_pos = 0;
	subdev->buf_frags = runtime->periods;

	icap_buf.subdev_id = (u32)subdev_id;
	icap_buf.buf = (u64)runtime->dma_addr;
	icap_buf.buf_size = runtime->periods * period_bytes;
	icap_buf.type = ICAP_BUF_CIRCURAL;
	icap_buf.gap_size = 0; // continuous circural
	icap_buf.frag_size = period_bytes;
	icap_buf.channels = runtime->channels;
	icap_buf.format = runtime->format;
	icap_buf.rate = runtime->rate;
	icap_buf.report_frags = 1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		snprintf(icap_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-playback-%d",
			 sa->card_name, subdev_id);

		if (subdev->buf_id != -1) {
			ret = icap_remove_src(icap, subdev->buf_id);
			if (ret)
				return ret;
			subdev->buf_id = -1;
		}
		ret = icap_add_src(icap, &icap_buf);
		if (ret < 0)
			return ret;
		subdev->buf_id = (u32)ret;

	} else {

		snprintf(icap_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-capture-%d",
			 sa->card_name, subdev_id);

		if (subdev->buf_id != -1) {
			ret = icap_remove_dst(icap, subdev->buf_id);
			if (ret)
				return ret;
			subdev->buf_id = -1;
		}
		ret = icap_add_dst(icap, &icap_buf);
		if (ret < 0)
			return ret;
		subdev->buf_id = (u32)ret;
	}

	return ret;
}

static int sa_pcm_trigger(struct snd_soc_component *component,
	struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sa_card_data *sa = runtime->private_data;
	int subdev_id = substream->pcm->device;
	struct sa_subdev *sa_subdev = &sa->subdevs[subdev_id];
	unsigned long flags;

	spin_lock_irqsave(&sa->start_stop_spinlock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		sa_subdev->state = SHARC_ALSA_STARTING;
		sa->starting = 1;
		queue_work(system_highpri_wq, &sa->send_start_work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		sa_subdev->state = SHARC_ALSA_STOPPING;
		sa->stopping = 1;
		queue_work(system_highpri_wq, &sa->send_stop_work);
		break;
	}

	spin_unlock_irqrestore(&sa->start_stop_spinlock, flags);

	return 0;
}

static int sa_pcm_new(struct snd_soc_component *component,
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	size_t size = sa_pcm_params.buffer_bytes_max;/* 128KiB */
	int ret = 0;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
				SNDRV_DMA_TYPE_DEV, card->dev, size, size);
	return 0;
}

static s32 sa_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *buf_frags)
{
	struct sa_card_data *sa = (struct sa_card_data *)icap->priv;
	struct sa_subdev *subdev;
	int i;
	unsigned long flags;

	for (i = 0; i < sa->subdev_num; i++) {
		subdev = &sa->subdevs[i];
		if (subdev->buf_id == buf_frags->buf_id) {
			spin_lock_irqsave(&subdev->buf_pos_lock, flags);
			subdev->buf_frag_pos += buf_frags->frags;
			if (subdev->buf_frag_pos >= subdev->buf_frags)
				subdev->buf_frag_pos = subdev->buf_frag_pos - subdev->buf_frags;
			spin_unlock_irqrestore(&subdev->buf_pos_lock, flags);
			snd_pcm_period_elapsed(subdev->substream);
		}
	}
	return 0;
}

struct icap_application_callbacks icap_application_callbacks = {
	.frag_ready = sa_frag_ready_cb,
};

static void sa_start_func(struct work_struct *work)
{
	struct sa_card_data *sa = container_of(work, struct sa_card_data, send_start_work);
	struct sa_subdev *subdev;
	unsigned long flags;
	int ret, i, reschedule;

	spin_lock_irqsave(&sa->start_stop_spinlock, flags);

	for (i = 0; i < sa->subdev_num; i++) {
		subdev = &sa->subdevs[i];
		if (subdev->state == SHARC_ALSA_STARTING) {
			subdev->state = SHARC_ALSA_RUNNING;
			spin_unlock_irqrestore(&sa->start_stop_spinlock, flags);
			ret = icap_start(&sa->icap, i);
			if (ret)
				dev_err(sa->dev, "ICAP subdev%d start error %d", i, ret);
			spin_lock_irqsave(&sa->start_stop_spinlock, flags);
		}
	}
	reschedule = sa->starting;
	sa->starting = 0;

	spin_unlock_irqrestore(&sa->start_stop_spinlock, flags);

	if (reschedule)
		queue_work(system_highpri_wq, &sa->send_stop_work);
}

static void sa_stop_func(struct work_struct *work)
{
	struct sa_card_data *sa = container_of(work, struct sa_card_data, send_stop_work);
	struct sa_subdev *subdev;
	unsigned long flags;
	int ret, i, reschedule;

	spin_lock_irqsave(&sa->start_stop_spinlock, flags);

	for (i = 0; i < sa->subdev_num; i++) {
		subdev = &sa->subdevs[i];
		if (subdev->state == SHARC_ALSA_STOPPING) {
			spin_unlock_irqrestore(&sa->start_stop_spinlock, flags);
			ret = icap_stop(&sa->icap, i);
			if (ret)
				dev_err(sa->dev, "ICAP subdev%d stop error %d", i, ret);
			spin_lock_irqsave(&sa->start_stop_spinlock, flags);
			subdev->state = SHARC_ALSA_STOPPED;
			wake_up_interruptible_all(&subdev->pending_stop_event);
		}
	}
	reschedule = sa->stopping;
	sa->stopping = 0;

	spin_unlock_irqrestore(&sa->start_stop_spinlock, flags);

	if (reschedule)
		queue_work(system_highpri_wq, &sa->send_stop_work);
}

static void sa_delayed_probe(struct work_struct *work)
{
	struct sa_card_data *sa = container_of(work, struct sa_card_data, delayed_probe_work);
	struct device *dev = sa->dev;
	struct rpmsg_device *rpdev = sa->rpdev;
	struct icap_instance *icap = &sa->icap;
	const u8 card_id = sa->card_id;
	struct sa_comp_data sa_comp;
	struct sa_comp_data *sa_comp_priv;
	struct icap_subdevice_features features;
	uint32_t subdev_num, i, link_id;
	char link_name[64];
	int32_t ret = 0;

	/* init Inter Core Audio protocol */
	icap->transport.rpdev = rpdev;
	ret = icap_application_init(icap, sa->card_name, &icap_application_callbacks, (void *)sa);
	if (ret) {
		dev_err(dev, "Failed to init ICAP: %d\n", ret);
		return;
	}

	ret = icap_get_subdevices(icap);
	if (ret < 0) {
		dev_err(dev, "Get subdevices error: %d", ret);
		goto sa_delayed_probe_fail;
	}
	subdev_num = (uint32_t)ret;
	subdev_num = subdev_num > SHARC_ALSA_SUBDEV_MAX ? SHARC_ALSA_SUBDEV_MAX : subdev_num;
	sa->subdev_num = subdev_num;

	/* Initialize ASoC dai_link, one for each subdevice */
	for (i = 0; i < subdev_num; i++) {

		spin_lock_init(&sa->subdevs[i].buf_pos_lock);
		init_waitqueue_head(&sa->subdevs[i].pending_stop_event);
		sa->subdevs[i].state = SHARC_ALSA_STOPPED;

		link_id = card_id * SHARC_ALSA_SUBDEV_MAX + i;

		ret = icap_get_subdevice_features(icap, i, &features);
		if (ret) {
			dev_err(dev, "Get subdev%d features error: %d", i, ret);
			goto sa_delayed_probe_fail;
		}

		if ((features.type != ICAP_DEV_PLAYBACK) && (features.type != ICAP_DEV_RECORD)) {
			dev_err(dev, "Unsupported subdev%d type %d", i, features.type);
			goto sa_delayed_probe_fail;
		}

		/* Initialize ASoC common component and dai driver for codec and cpu */
		memset(&sa_comp, 0, sizeof(sa_comp));
		if (features.type == ICAP_DEV_PLAYBACK) {
			if (features.src_buf_max < 1) {
				dev_err(dev, "Invalid max sourcebuf, playback subdev %d", i);
				ret = -EINVAL;
				goto sa_delayed_probe_fail;
			}
			sa_comp.dai_driver.playback.stream_name = "Playback";
			sa_comp.dai_driver.playback.channels_min = features.channels_min;
			sa_comp.dai_driver.playback.channels_max = features.channels_max;
			sa_comp.dai_driver.playback.rates = features.rates;
			sa_comp.dai_driver.playback.formats = features.formats;
		} else {
			/* Capture */
			if (features.dst_buf_max < 1) {
				dev_err(dev, "Invalid max destbuf, capt subdev %d", i);
				ret = -EINVAL;
				goto sa_delayed_probe_fail;
			}
			sa_comp.dai_driver.capture.stream_name = "Capture";
			sa_comp.dai_driver.capture.channels_min = features.channels_min;
			sa_comp.dai_driver.capture.channels_max = features.channels_max;
			sa_comp.dai_driver.capture.rates = features.rates;
			sa_comp.dai_driver.capture.formats = features.formats;
		}

		sa_comp.component_driver.idle_bias_on = 1;
		sa_comp.component_driver.use_pmdown_time = 1;
		sa_comp.component_driver.endianness = 1;

		/* Initialize ASoC codec component and dai driver one for each ICAP subdevice */
		sprintf(sa_comp.dai_driver_name, "sharc-alsa-codec-dai_%d", link_id);
		sa_comp.dai_driver.name = sa_comp.dai_driver_name;
		sprintf(sa_comp.component_driver_name, "sharc-alsa-codec_%d", link_id);
		sa_comp.component_driver.name = sa_comp.component_driver_name;

		sa->asoc_codec_devs[sa->codec_num] = platform_device_register_data(
			dev, "sharc-alsa-codec", link_id, &sa_comp, sizeof(sa_comp));
		if (IS_ERR(sa->asoc_codec_devs[sa->codec_num])) {
			dev_err(dev, "Failed to register codec component\n");
			ret = PTR_ERR(sa->asoc_codec_devs[sa->codec_num]);
			goto sa_delayed_probe_fail;
		}
		sa->codec_num++;

		/* Initialize ASoC cpu component and dai drivers */
		sprintf(sa_comp.dai_driver_name, "sharc-alsa-cpu-dai_%d", link_id);
		sa_comp.dai_driver.name = sa_comp.dai_driver_name;
		sprintf(sa_comp.component_driver_name, "sharc-alsa-cpu_%d", link_id);
		sa_comp.component_driver.name = sa_comp.component_driver_name;

		sa->asoc_cpu_devs[sa->cpu_num] = platform_device_register_data(
			dev, "sharc-alsa-cpu", link_id, &sa_comp, sizeof(sa_comp));
		if (IS_ERR(sa->asoc_cpu_devs[sa->cpu_num])) {
			dev_err(dev, "Failed to register cpu component\n");
			ret = PTR_ERR(sa->asoc_cpu_devs[sa->cpu_num]);
			goto sa_delayed_probe_fail;
		}
		sa->cpu_num++;

		/* Initialize ASoC platform driver */
		memset(&sa_comp, 0, sizeof(sa_comp));
		sprintf(sa_comp.component_driver_name, "sharc-alsa-platform_%d", link_id);
		sa_comp.component_driver.name = sa_comp.component_driver_name;
		sa_comp.component_driver.open = sa_pcm_open;
		sa_comp.component_driver.hw_params = sa_pcm_hw_params;
		sa_comp.component_driver.hw_free = sa_pcm_hw_free;
		sa_comp.component_driver.prepare = sa_pcm_prepare;
		sa_comp.component_driver.trigger = sa_pcm_trigger;
		sa_comp.component_driver.pointer = sa_pcm_pointer;
		sa_comp.component_driver.pcm_construct = sa_pcm_new;

		sa->asoc_platform_devs[sa->platform_num] = platform_device_register_data(
			dev, "sharc-alsa-platform", link_id, &sa_comp, sizeof(sa_comp));
		if (IS_ERR(sa->asoc_platform_devs[sa->platform_num])) {
			dev_err(dev, "Failed to register platform component\n");
			ret = PTR_ERR(sa->asoc_platform_devs[sa->platform_num]);
			goto sa_delayed_probe_fail;
		}
		sa->platform_num++;

		/* Initialize ASoC links */
		sa_comp_priv = dev_get_platdata(&sa->asoc_codec_devs[i]->dev);
		sa->codec_links[i].of_node = NULL;
		sa->codec_links[i].name = dev_name(&sa->asoc_codec_devs[i]->dev);
		sa->codec_links[i].dai_name = sa_comp_priv->dai_driver_name;

		sa_comp_priv = dev_get_platdata(&sa->asoc_cpu_devs[i]->dev);
		sa->cpu_links[i].of_node = NULL;
		sa->cpu_links[i].name = dev_name(&sa->asoc_cpu_devs[i]->dev);
		sa->cpu_links[i].dai_name = sa_comp_priv->dai_driver_name;

		sa->platform_links[i].of_node = NULL;
		sa->platform_links[i].name = dev_name(&sa->asoc_platform_devs[i]->dev);
		sa->platform_links[i].dai_name = NULL;

		if (features.type == ICAP_DEV_PLAYBACK) {
			sa->dai_links[i].playback_only = 1;
			snprintf(link_name, 64, "sharc-alsa-playback-%d", link_id);
		} else {
			/* Capture */
			sa->dai_links[i].capture_only = 1;
			snprintf(link_name, 64, "sharc-alsa-capture-%d", link_id);
		}
		sa->dai_links[i].name = devm_kstrdup(dev, link_name, GFP_KERNEL);
		sa->dai_links[i].stream_name = sa->dai_links[i].name;
		sa->dai_links[i].cpus = &sa->cpu_links[i];
		sa->dai_links[i].num_cpus = 1;
		sa->dai_links[i].codecs = &sa->codec_links[i];
		sa->dai_links[i].num_codecs = 1;
		sa->dai_links[i].platforms = &sa->platform_links[i];
		sa->dai_links[i].num_platforms = 1;
		sa->dai_links[i].init = NULL;
		sa->dai_links[i].ops = NULL;
	}

	/* Initialize ASoC card */
	sa->card.name = sa->card_name;
	sa->card.owner		= THIS_MODULE;
	sa->card.dev		= dev;
	sa->card.probe		= NULL;
	sa->card.dai_link	= sa->dai_links;
	sa->card.num_links	= subdev_num;

	ret = snd_soc_register_card(&sa->card);

	if (ret < 0)
		goto sa_delayed_probe_fail;

	sa->card_registered = 1;

	dev_info(dev, "sharc-alsa card probed for rpmsg endpoint addr: 0x%03x\n", rpdev->dst);

	return;

sa_delayed_probe_fail:
	for (i = 0; i < sa->codec_num; i++)
		platform_device_unregister(sa->asoc_codec_devs[i]);

	for (i = 0; i < sa->cpu_num; i++)
		platform_device_unregister(sa->asoc_cpu_devs[i]);

	for (i = 0; i < sa->platform_num; i++)
		platform_device_unregister(sa->asoc_platform_devs[i]);

	sa->codec_num = 0;
	sa->cpu_num = 0;
	sa->platform_num = 0;

	icap_application_deinit(&sa->icap);
}

static int sa_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	struct sa_card_data *sa;
	int i;
	int ret = 0;

	sa = devm_kzalloc(dev, sizeof(struct sa_card_data), GFP_KERNEL);
	if (!sa)
		return -ENOMEM;

	sa->card_id = rpdev->dst;
	sprintf(sa->card_name, "sharc-alsa-card_%d", sa->card_id);

	sa->rpdev = rpdev;
	sa->dev = dev;
	INIT_WORK(&sa->delayed_probe_work, sa_delayed_probe);
	INIT_WORK(&sa->send_start_work, sa_start_func);
	INIT_WORK(&sa->send_stop_work, sa_stop_func);

	for (i = 0; i < SHARC_ALSA_SUBDEV_MAX; i++)
		sa->subdevs[i].buf_id = -1;

	/*
	 * The snd_soc_register_card sets the driver_data in `struct device` for its own use
	 * Use drvdata of the `struct snd_soc_card` to keep the sa pointer, which is
	 * needed in remove.
	 */
	sa->card.dev = dev;
	dev_set_drvdata(dev, &sa->card);
	snd_soc_card_set_drvdata(&sa->card, sa);

	/*
	 * This probe function is in interrupt context (rpmsg handling)
	 * Can't wait for ICAP response here, do the rest of probe in the system_wq workqueue.
	 */
	ret = queue_work(system_highpri_wq, &sa->delayed_probe_work);

	return !ret;
}

static void sa_remove(struct rpmsg_device *rpdev)
{
	struct snd_soc_card *card = dev_get_drvdata(&rpdev->dev);
	struct sa_card_data *sa = snd_soc_card_get_drvdata(card);
	int32_t ret, i;

	/* Cancel probe work */
	cancel_work_sync(&sa->delayed_probe_work);

	if (sa->card_registered)
		snd_soc_unregister_card(&sa->card);

	for (i = 0; i < sa->codec_num; i++)
		platform_device_unregister(sa->asoc_codec_devs[i]);

	for (i = 0; i < sa->cpu_num; i++)
		platform_device_unregister(sa->asoc_cpu_devs[i]);

	for (i = 0; i < sa->platform_num; i++)
		platform_device_unregister(sa->asoc_platform_devs[i]);

	/* Cancel all other works */
	cancel_work_sync(&sa->send_start_work);
	cancel_work_sync(&sa->send_stop_work);

	ret = icap_application_deinit(&sa->icap);
	if (ret)
		dev_err(&rpdev->dev, "ICAP deinit failed %d\n", ret);

	dev_err(&rpdev->dev, "sharc-alsa card removed for rpmsg endpoint addr: 0x%03x\n",
		rpdev->dst);
}

static int sa_comp_probe(struct platform_device *pdev)
{
	struct sa_comp_data *sa_comp = (struct sa_comp_data *) dev_get_platdata(&pdev->dev);
	struct snd_soc_dai_driver *dai_drv;
	int dai_num;

	if (sa_comp->dai_driver.name) {
		dai_drv = &sa_comp->dai_driver;
		dai_num = 1;
	} else {
		dai_drv = NULL;
		dai_num = 0;
	}
	return devm_snd_soc_register_component(&pdev->dev,
					       &sa_comp->component_driver,
					       dai_drv, dai_num);
}

static struct rpmsg_device_id sa_id_table[] = {
	{ .name = "sharc-alsa" },
	{ },
};
static struct rpmsg_driver sa_rpmsg_driver = {
	.drv.name  = KBUILD_MODNAME,
	.drv.owner = THIS_MODULE,
	.id_table  = sa_id_table,
	.probe     = sa_probe,
	.callback  = sa_rpmsg_cb,
	.remove    = sa_remove,
};

static struct platform_driver sa_cpu_driver = {
	.driver = {
		.name = "sharc-alsa-cpu",
	},
	.probe = sa_comp_probe,
};

static struct platform_driver sa_codec_driver = {
	.driver = {
		.name = "sharc-alsa-codec",
	},
	.probe = sa_comp_probe,
};

static struct platform_driver sa_platform_driver = {
	.driver = {
		.name = "sharc-alsa-platform",
	},
	.probe = sa_comp_probe,
};

static int sa_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&sa_cpu_driver);
	if (ret != 0) {
		pr_err("sa: failed to register cpu driver\n");
		goto fail_cpu_driver;
	}

	ret = platform_driver_register(&sa_codec_driver);
	if (ret != 0) {
		pr_err("sa: failed to register codec driver\n");
		goto fail_codec_driver;
	}

	ret = platform_driver_register(&sa_platform_driver);
	if (ret != 0) {
		pr_err("sa: failed to register platform driver\n");
		goto fail_platform_driver;
	}

	ret = register_rpmsg_driver(&sa_rpmsg_driver);
	if (ret < 0) {
		pr_err("sa: failed to register rpmsg driver\n");
		goto fail_rpmsg_driver;
	}

	return 0;

fail_rpmsg_driver:
	platform_driver_unregister(&sa_platform_driver);
fail_platform_driver:
	platform_driver_unregister(&sa_codec_driver);
fail_codec_driver:
	platform_driver_unregister(&sa_cpu_driver);
fail_cpu_driver:
	return ret;
}
module_init(sa_driver_init);

static void sa_driver_exit(void)
{
	unregister_rpmsg_driver(&sa_rpmsg_driver);
	platform_driver_unregister(&sa_platform_driver);
	platform_driver_unregister(&sa_codec_driver);
	platform_driver_unregister(&sa_cpu_driver);
}
module_exit(sa_driver_exit);

MODULE_DESCRIPTION("Analog Devices SHARC-ALSA ASoC card driver");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("sharc-alsa");
