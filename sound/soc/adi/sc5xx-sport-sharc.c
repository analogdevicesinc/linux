// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SC5XX SHARC SPORT driver.
 * Data proccessed and feed into SPORT DMA buff by a SHARC core.
 * Code based on sc5xx-sport.c
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Author: Scott Jiang <Scott.Jiang.Linux@gmail.com>
 * Author: Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/atomic.h>

#include <linux/soc/adi/cpu.h>
#include <sound/sc5xx-dai.h>
#include <sound/pcm.h>
#include <linux/rpmsg.h>

#include "sc5xx-sport.h"

#define _DEBUG 0

#define SHARC_MSG_TIMEOUT usecs_to_jiffies(1000)

static struct sport_device *sport_devices[8];

void sharc_playback_underrun_uevent(struct sport_device *sport, int core)
{
	char _env[64];
	char *envp[] = {_env, NULL};

	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_UNDERRUN", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

void sharc_record_overrun_uevent(struct sport_device *sport, int core)
{
	char _env[64];
	char *envp[] = {_env, NULL};

	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_OVERRUN", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

void sharc_msg_dropped_uevent(struct sport_device *sport, int core)
{
	char _env[64];
	char *envp[] = {_env, NULL};

	snprintf(_env, sizeof(_env), "EVENT=SHARC%d_MSG_DROPPED", core);
	kobject_uevent_env(&sport->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static int sport_frag_ready_cb(struct icap_instance *icap, struct icap_buf_frags *frags)
{
	struct sport_device *sport = (struct sport_device *)icap->priv;
	unsigned long flags;

	if (frags->buf_id == sport->tx_alsa_icap_buf_id) {
		spin_lock_irqsave(&sport->sharc_tx_buf_pos_lock, flags);
		sport->sharc_tx_buf_pos += frags->frags * sport->tx_fragsize;
		if (sport->sharc_tx_buf_pos >= sport->sharc_tx_dma_buf.bytes)
			sport->sharc_tx_buf_pos = sport->sharc_tx_buf_pos -
						  sport->sharc_tx_dma_buf.bytes;
		spin_unlock_irqrestore(&sport->sharc_tx_buf_pos_lock, flags);
		sport->tx_callback(sport->tx_data);
	}

	if (frags->buf_id == sport->rx_alsa_icap_buf_id) {
		spin_lock_irqsave(&sport->sharc_rx_buf_pos_lock, flags);
		sport->sharc_rx_buf_pos += frags->frags * sport->rx_fragsize;
		if (sport->sharc_rx_buf_pos >= sport->sharc_rx_dma_buf.bytes)
			sport->sharc_rx_buf_pos = sport->sharc_rx_buf_pos -
						  sport->sharc_rx_dma_buf.bytes;
		spin_unlock_irqrestore(&sport->sharc_rx_buf_pos_lock, flags);
		sport->rx_callback(sport->rx_data);
	}

	return 0;
}

struct icap_application_callbacks sport_icap_callbacks = {
	.frag_ready = sport_frag_ready_cb,
};

int sport_set_tx_params(struct sport_device *sport,
			struct sport_params *params)
{
	if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI) {
		//try to stop tx
		dev_warn(&sport->pdev->dev, "tx pcm is running during playback init, stopping ...\n");
		sport_tx_stop(sport);
		if (ioread32(&sport->tx_regs->spctl) & SPORT_CTL_SPENPRI)
			return -EBUSY;
	}

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

void get_sharc_features(struct sport_device *sport, int sharc_core)
{
	struct icap_instance *icap = &sport->icap[sharc_core];
	struct icap_subdevice_features features;
	u32 dev_num, i;
	s32 ret;

	ret = icap_get_subdevices(icap);
	if (ret < 0)
		dev_err(&sport->pdev->dev, "Get sharc%d devices error: %d", sharc_core, ret);
		return;

	dev_num = (u32)ret;

	for (i = 0; i < dev_num; i++) {
		ret = icap_get_subdevice_features(icap, i, &features);
		if (ret) {
			dev_err(&sport->pdev->dev, "Get sharc%d dev%d features error: %d",
				sharc_core, i, ret);
			return;
		}
		if (features.type == ICAP_DEV_PLAYBACK) {
			sport->sharc_tx_core = sharc_core;
			sport->icap_tx_dev_id = i;
		} else if (features.type == ICAP_DEV_RECORD) {
			sport->sharc_rx_core = sharc_core;
			sport->icap_rx_dev_id = i;
		} else {
			dev_err(&sport->pdev->dev, "Get sharc%d unknown dev%d type %d",
				sharc_core, i, features.type);
			return;
		}
	}
}

void get_sharc1_feature_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work,
						  struct sport_device,
						  get_sharc1_feature_work);
	const int sharc_core = 0;

	get_sharc_features(sport, sharc_core);
}

void get_sharc2_feature_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work,
						  struct sport_device,
						  get_sharc2_feature_work);
	const int sharc_core = 1;

	get_sharc_features(sport, sharc_core);
}

void sport_tx_start_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work,
						  struct sport_device,
						  send_tx_start_work);
	unsigned long flags;
	u32 sharc_core, dev_id;
	s32 ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_tx_core;
	dev_id = sport->icap_tx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core != -1) && (dev_id != -1)) {
		ret = icap_start(&sport->icap[sharc_core], dev_id);
		if (ret)
			dev_err(&sport->pdev->dev, "tx_start error: %d", ret);
	}
}

int sport_tx_start(struct sport_device *sport)
{
	s32 ret;

	ret = queue_work(system_highpri_wq, &sport->send_tx_start_work);
	if (ret == 0)
		return -EIO;

	//enable DMA, after SHARC ACKs START
	sport->tx_cookie = dmaengine_submit(sport->tx_desc);
	dma_async_issue_pending(sport->tx_dma_chan);
	iowrite32(ioread32(&sport->tx_regs->spctl) | SPORT_CTL_SPENPRI, &sport->tx_regs->spctl);

	return 0;
}
EXPORT_SYMBOL(sport_tx_start);

void sport_rx_start_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_rx_start_work);
	unsigned long flags;
	u32 sharc_core, dev_id;
	s32 ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_rx_core;
	dev_id = sport->icap_rx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core != -1) && (dev_id != -1)) {
		ret = icap_start(&sport->icap[sharc_core], dev_id);
		if (ret)
			dev_err(&sport->pdev->dev, "rx_start error: %d", ret);
	}
}

int sport_rx_start(struct sport_device *sport)
{
	int ret;

	sport->rx_cookie = dmaengine_submit(sport->rx_desc);
	dma_async_issue_pending(sport->rx_dma_chan);
	iowrite32(ioread32(&sport->rx_regs->spctl) | SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);

	ret = queue_work(system_highpri_wq, &sport->send_rx_start_work);
	return !ret;
}
EXPORT_SYMBOL(sport_rx_start);

void sport_tx_stop_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_tx_stop_work);
	unsigned long flags;
	u32 sharc_core, dev_id;
	s32 ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_tx_core;
	dev_id = sport->icap_tx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core != -1) && (dev_id != -1)) {
		ret = icap_stop(&sport->icap[sharc_core], dev_id);
		if (ret)
			dev_err(&sport->pdev->dev, "tx_stop error: %d", ret);
	}
	sport->pending_tx_stop = 0;
	wake_up_interruptible_all(&sport->pending_tx_stop_event);
}

int sport_tx_stop(struct sport_device *sport)
{
	int ret;

	iowrite32(ioread32(&sport->tx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->tx_regs->spctl);
	dmaengine_terminate_sync(sport->tx_dma_chan);

	/*
	 * Can't send icap message and wait for response here as we can be in interrupt context.
	 * FRAG_READY callbacks are processed in the rpmsg interrupt context.
	 * The FRAG_READY cb calls snd_pcm_period_elapsed() which can call stop trigger on xrun
	 * event.  Waiting here blocks entire rpmsg communication on the rpdev.
	 */
	sport->pending_tx_stop = 1;
	ret = queue_work(system_highpri_wq, &sport->send_tx_stop_work);
	return !ret;
}
EXPORT_SYMBOL(sport_tx_stop);

void sport_rx_stop_work_func(struct work_struct *work)
{
	struct sport_device *sport = container_of(work, struct sport_device, send_rx_stop_work);
	unsigned long flags;
	u32 sharc_core, dev_id;
	s32 ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_rx_core;
	dev_id = sport->icap_rx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core != -1) && (dev_id != -1)) {
		ret = icap_stop(&sport->icap[sharc_core], dev_id);
		if (ret)
			dev_err(&sport->pdev->dev, "rx_stop error: %d", ret);
	}
	sport->pending_rx_stop = 0;
	wake_up_interruptible_all(&sport->pending_rx_stop_event);
}

int sport_rx_stop(struct sport_device *sport)
{
	int ret;

	iowrite32(ioread32(&sport->rx_regs->spctl) & ~SPORT_CTL_SPENPRI,
			&sport->rx_regs->spctl);
	dmaengine_terminate_sync(sport->rx_dma_chan);

	/*
	 * Can't send icap message and wait for response here as we can be in interrupt context.
	 * FRAG_READY callbacks are processed in the rpmsg interrupt context.
	 * The FRAG_READY cb calls snd_pcm_period_elapsed() which can call stop trigger on xrun
	 * event.  Waiting here blocks entire rpmsg communication on the rpdev.
	 */
	sport->pending_rx_stop = 1;
	ret = queue_work(system_highpri_wq, &sport->send_rx_stop_work);
	return !ret;
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

int sport_config_tx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	struct icap_buf_descriptor audio_buf;
	struct icap_subdevice_params params;
	unsigned long flags;
	u32 sharc_core, dev_id;
	struct dma_slave_config dma_config = {0};
	int ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_tx_core;
	dev_id = sport->icap_tx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core == -1) || (dev_id == -1))
		return -ENODEV;

	ret = wait_event_interruptible(sport->pending_tx_stop_event, !sport->pending_tx_stop);
	if (ret)
		return ret;

	/* Allocate buffer for SHARC output - DMA, prefers
	 * iram pool, if not available it fallbacks to CMA
	 */
	if (sport->sharc_tx_dma_buf.dev.type)
		snd_dma_free_pages(&sport->sharc_tx_dma_buf);
	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_IRAM,
				  &sport->pdev->dev, fragsize * fragcount,
				  &sport->sharc_tx_dma_buf);
	if (ret)
		return ret;

	sport->tx_buf = sport->sharc_tx_dma_buf.addr;
	sport->tx_fragsize = fragsize;
	sport->tx_frags = fragcount;
	sport->sharc_tx_buf_pos = 0;

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
		sport->sharc_tx_dma_buf.addr, fragsize * fragcount, fragsize, DMA_MEM_TO_DEV,
		DMA_PREP_INTERRUPT);

	sport->tx_substream = substream;

	memset(&params, 0, sizeof(params));

	/* Rest of the params don't care as
	 * sharc doesn't set the hardware params
	 */
	params.subdev_id = dev_id;

	ret = icap_subdevice_init(&sport->icap[sharc_core], &params);
	if (ret == -ERESTARTSYS || ret == -ETIMEDOUT)
		return ret;

	if (sport->tx_dma_icap_buf_id != -1) {
		ret = icap_remove_dst(&sport->icap[sharc_core], sport->tx_dma_icap_buf_id);
		if (ret)
			dev_err(&sport->pdev->dev, "tx_stop dst remove error: %d", ret);
		sport->tx_dma_icap_buf_id = -1;
	}

	if (sport->tx_alsa_icap_buf_id != -1) {
		ret = icap_remove_src(&sport->icap[sharc_core], sport->tx_alsa_icap_buf_id);
		if (ret)
			dev_err(&sport->pdev->dev, "tx_stop src remove error: %d", ret);
		sport->tx_alsa_icap_buf_id = -1;
	}

	// Set ALSA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-playback", sport->pdev->name);
	audio_buf.subdev_id = dev_id;
	audio_buf.buf = (u64)buf;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continuous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format
	audio_buf.channels = params_channels(&sport->tx_hw_params);
	audio_buf.format = params_format(&sport->tx_hw_params);
	audio_buf.rate = params_rate(&sport->tx_hw_params);
	audio_buf.report_frags = 1;

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"ALSA  playback buf PA:0x%p size:%d fragsize:%d\n",
		(void *)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_src(&sport->icap[sharc_core], &audio_buf);
	if (ret < 0)
		return ret;

	sport->tx_alsa_icap_buf_id = (u32)ret;

	// Set DMA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-dma-playback", sport->pdev->name);
	audio_buf.subdev_id = dev_id;
	audio_buf.buf = (u64)sport->sharc_tx_dma_buf.addr;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continuous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format - same as ALSA buffer
	audio_buf.channels = params_channels(&sport->tx_hw_params);
	audio_buf.format = params_format(&sport->tx_hw_params);
	audio_buf.rate = params_rate(&sport->tx_hw_params);
	audio_buf.report_frags = 0;

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"SHARC playback buf PA:0x%p size:%d fragsize:%d\n",
		(void *)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_dst(&sport->icap[sharc_core], &audio_buf);
	if (ret < 0) {
		icap_remove_src(&sport->icap[sharc_core], sport->tx_alsa_icap_buf_id);
		return ret;
	}
	sport->tx_dma_icap_buf_id = (u32)ret;

	return 0;
}
EXPORT_SYMBOL(sport_config_tx_dma);

int sport_config_rx_dma(struct sport_device *sport, void *buf,
		int fragcount, size_t fragsize, struct snd_pcm_substream *substream)
{
	struct icap_buf_descriptor audio_buf;
	struct icap_subdevice_params params;
	unsigned long flags;
	u32 sharc_core, dev_id;
	struct dma_slave_config dma_config = {0};
	int ret;

	spin_lock_irqsave(&sport->icap_spinlock, flags);
	sharc_core = sport->sharc_rx_core;
	dev_id = sport->icap_rx_dev_id;
	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if ((sharc_core == -1) || (dev_id == -1))
		return -ENODEV;

	ret = wait_event_interruptible(sport->pending_rx_stop_event, !sport->pending_rx_stop);
	if (ret)
		return ret;

	/* Allocate buffer for SHARC input - DMA, prefers
	 * iram pool, if not available it fallbacks to CMA
	 */
	if (sport->sharc_rx_dma_buf.dev.type)
		snd_dma_free_pages(&sport->sharc_rx_dma_buf);

	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_IRAM,
				  &sport->pdev->dev, fragsize * fragcount,
				  &sport->sharc_rx_dma_buf);
	if (ret)
		return ret;

	sport->rx_buf = sport->sharc_rx_dma_buf.addr;
	sport->rx_fragsize = fragsize;
	sport->rx_frags = fragcount;
	sport->sharc_rx_buf_pos = 0;

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
		sport->sharc_rx_dma_buf.addr, fragsize * fragcount, fragsize, DMA_DEV_TO_MEM,
		DMA_PREP_INTERRUPT);

	sport->rx_substream = substream;

	memset(&params, 0, sizeof(params));

	/* Rest of the params don't care as sharc
	 * doesn't set the hardware params
	 */
	params.subdev_id = dev_id;

	ret = icap_subdevice_init(&sport->icap[sharc_core], &params);
	if (ret == -ERESTARTSYS || ret == -ETIMEDOUT)
		return ret;

	if (sport->rx_alsa_icap_buf_id != -1) {
		ret = icap_remove_dst(&sport->icap[sharc_core], sport->rx_alsa_icap_buf_id);
		if (ret)
			dev_err(&sport->pdev->dev, "rx_stop dst remove error: %d", ret);
		sport->rx_alsa_icap_buf_id = -1;
	}

	if (sport->rx_dma_icap_buf_id != -1) {
		ret = icap_remove_src(&sport->icap[sharc_core], sport->rx_dma_icap_buf_id);
		if (ret)
			dev_err(&sport->pdev->dev, "rx_stop src remove error: %d", ret);
		sport->rx_dma_icap_buf_id = -1;
	}

	// Set ALSA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-alsa-record", sport->pdev->name);
	audio_buf.subdev_id = dev_id;
	audio_buf.buf = (u64)buf;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continuous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format
	audio_buf.channels = params_channels(&sport->rx_hw_params);
	audio_buf.format = params_format(&sport->rx_hw_params);
	audio_buf.rate = params_rate(&sport->rx_hw_params);
	audio_buf.report_frags = 1;

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"ALSA  record buf PA:0x%p size:%d fragsize:%d\n",
		(void *)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_dst(&sport->icap[sharc_core], &audio_buf);
	if (ret < 0)
		return ret;

	sport->rx_alsa_icap_buf_id = (u32)ret;

	// Set DMA buffer size and pointer
	snprintf(audio_buf.name, ICAP_BUF_NAME_LEN, "%s-dma-record", sport->pdev->name);
	audio_buf.subdev_id = dev_id;
	audio_buf.buf = (u64)sport->sharc_rx_dma_buf.addr;
	audio_buf.buf_size = fragsize * fragcount;
	audio_buf.type = ICAP_BUF_CIRCURAL;
	audio_buf.gap_size = 0; // continuous circural
	audio_buf.frag_size = fragsize;
	// Set audio data format - same as ALSA buffer
	audio_buf.channels = params_channels(&sport->rx_hw_params);
	audio_buf.format = params_format(&sport->rx_hw_params);
	audio_buf.rate = params_rate(&sport->rx_hw_params);
	audio_buf.report_frags = 0;

#if _DEBUG
	dev_info(&sport->pdev->dev,
		"SHARC record buf PA:0x%p size:%d fragsize:%d\n",
		(void *)audio_buf.buf, audio_buf.buf_size, audio_buf.frag_size);
#endif
	ret = icap_add_src(&sport->icap[sharc_core], &audio_buf);
	if (ret < 0) {
		icap_remove_dst(&sport->icap[sharc_core], sport->rx_alsa_icap_buf_id);
		return ret;
	}
	sport->rx_dma_icap_buf_id = (u32)ret;

	return 0;
}
EXPORT_SYMBOL(sport_config_rx_dma);

unsigned long sport_curr_offset_tx(struct sport_device *sport)
{
	unsigned long off;
	unsigned long flags;

	spin_lock_irqsave(&sport->sharc_tx_buf_pos_lock, flags);
	off = sport->sharc_tx_buf_pos;
	spin_unlock_irqrestore(&sport->sharc_tx_buf_pos_lock, flags);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_tx);

unsigned long sport_curr_offset_rx(struct sport_device *sport)
{
	unsigned long off;
	unsigned long flags;

	spin_lock_irqsave(&sport->sharc_rx_buf_pos_lock, flags);
	off = sport->sharc_rx_buf_pos;
	spin_unlock_irqrestore(&sport->sharc_rx_buf_pos_lock, flags);

	return off;
}
EXPORT_SYMBOL(sport_curr_offset_rx);

static int sport_get_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

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

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "No tx error irq resource\n");
		return -ENODEV;
	}
	sport->tx_err_irq = res->start;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!res) {
		dev_err(dev, "No rx error irq resource\n");
		return -ENODEV;
	}
	sport->rx_err_irq = res->start;

	return 0;
}

static int sport_request_resource(struct sport_device *sport)
{
	struct platform_device *pdev = sport->pdev;
	struct device *dev = &pdev->dev;
	int ret;

	sport->tx_dma_chan = dma_request_chan(dev, "tx");
	if (IS_ERR(sport->tx_dma_chan)) {
		ret = PTR_ERR(sport->tx_dma_chan);
		dev_err(dev, "Missing `tx` dma channel: %d\n", ret);
		return ret;
	}

	sport->rx_dma_chan = dma_request_chan(dev, "rx");
	if (IS_ERR(sport->rx_dma_chan)) {
		ret = PTR_ERR(sport->rx_dma_chan);
		dev_err(dev, "Missing `rx` dma channel: %d\n", ret);
		goto err_rx_dma;
	}

	/* NOTE: tx_irq, rx_irq and err_irqs handled by SHARC core*/

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

int rpmsg_icap_sport_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct sport_device **sport_p = (struct sport_device **)dev_get_drvdata(&rpdev->dev);
	struct sport_device *sport = *sport_p;
	union icap_remote_addr src_addr;
	int sharc_core;
	int ret;

	if (sport == NULL)
		return -ENODEV;

	/* Find corresponding core */
	for (sharc_core = 0; sharc_core < SHARC_CORES_NUM; sharc_core++) {
		if (sport->icap[sharc_core].transport.rpdev == rpdev)
			break;
	}

	if (sharc_core >= SHARC_CORES_NUM)
		return -ENODEV;

	src_addr.rpmsg_addr = src;
	ret = icap_parse_msg(&sport->icap[sharc_core], &src_addr, data, len);
	if (ret && (ret != -ICAP_ERROR_INIT)) {
		if (ret == -ICAP_ERROR_TIMEOUT)
			dev_notice_ratelimited(&rpdev->dev, "ICAP late response\n");
		else
			dev_err_ratelimited(&rpdev->dev, "ICAP parse msg error: %d\n", ret);
	}
	return 0;
}
EXPORT_SYMBOL(rpmsg_icap_sport_cb);

int rpmsg_icap_sport_probe(struct rpmsg_device *rpdev)
{
	struct sport_device **sport_p;
	struct sport_device *sport;
	int sharc_core;
	int ret;

	if (!strncmp(rpdev->id.name, "icap-sport4-core1", RPMSG_NAME_SIZE)) {
		sport_p = &sport_devices[4];
		sharc_core = 0;
	} else if (!strncmp(rpdev->id.name, "icap-sport4-core2", RPMSG_NAME_SIZE)) {
		sport_p = &sport_devices[4];
		sharc_core = 1;
	} else {
		/* Currently supports only sport4 on sharc core 1 or 2 */
		return -ENODEV;
	}

	sport = *sport_p;

	if (sport == NULL)
		return -ENODEV;

	dev_set_drvdata(&rpdev->dev, sport_p);

	sport->icap[sharc_core].transport.rpdev = rpdev;
	ret = icap_application_init(&sport->icap[sharc_core],
				    "sc5xx-sport",
				    &sport_icap_callbacks,
				    (void *)sport);

	if (ret)
		goto error_out;

	if (sharc_core == 0)
		ret = queue_work(system_highpri_wq, &sport->get_sharc1_feature_work);
	else
		ret = queue_work(system_highpri_wq, &sport->get_sharc2_feature_work);

	if (ret == 0) {
		icap_application_deinit(&sport->icap[sharc_core]);
		ret = -EIO;
		goto error_out;
	}

	dev_info(&sport->pdev->dev, "sharc-alsa client dev attached, addr: 0x%03x\n", rpdev->dst);
	return 0;

error_out:
	dev_err(&sport->pdev->dev, "sharc-alsa client dev err, addr: 0x%03x, %d\n",
		rpdev->dst, ret);
	return ret;
}
EXPORT_SYMBOL(rpmsg_icap_sport_probe);

void rpmsg_icap_sport_remove(struct rpmsg_device *rpdev)
{
	struct sport_device **sport_p = (struct sport_device **)dev_get_drvdata(&rpdev->dev);
	struct sport_device *sport = *sport_p;
	int sharc_core;
	unsigned long flags;

	if (sport == NULL)
		return;

	/* Find corresponding core */
	for (sharc_core = 0; sharc_core < SHARC_CORES_NUM; sharc_core++) {
		if (sport->icap[sharc_core].transport.rpdev == rpdev)
			break;
	}

	if (sharc_core >= SHARC_CORES_NUM)
		return;

	spin_lock_irqsave(&sport->icap_spinlock, flags);

	icap_application_deinit(&sport->icap[sharc_core]);

	if (sharc_core == sport->sharc_tx_core) {
		sport->sharc_tx_core = -1;
		sport->icap_tx_dev_id = -1;
	}

	if (sharc_core == sport->sharc_rx_core) {
		sport->sharc_rx_core = -1;
		sport->icap_rx_dev_id = -1;
	}

	spin_unlock_irqrestore(&sport->icap_spinlock, flags);

	if (sport->tx_substream &&
	    sport->tx_substream->runtime &&
	    snd_pcm_running(sport->tx_substream)) {
		snd_pcm_stream_lock_irq(sport->tx_substream);
		snd_pcm_stop(sport->tx_substream, SNDRV_PCM_STATE_DISCONNECTED);
		snd_pcm_stream_unlock_irq(sport->tx_substream);
	}

	if (sport->rx_substream &&
	    sport->rx_substream->runtime &&
	    snd_pcm_running(sport->rx_substream)) {
		snd_pcm_stream_lock_irq(sport->rx_substream);
		snd_pcm_stop(sport->rx_substream, SNDRV_PCM_STATE_DISCONNECTED);
		snd_pcm_stream_unlock_irq(sport->rx_substream);
	}

	dev_info(&rpdev->dev, "sharc-alsa client device is removed, addr: 0x%03x\n", rpdev->dst);
}
EXPORT_SYMBOL(rpmsg_icap_sport_remove);

struct sport_device *sport_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
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

	spin_lock_init(&sport->icap_spinlock);
	spin_lock_init(&sport->sharc_tx_buf_pos_lock);
	spin_lock_init(&sport->sharc_rx_buf_pos_lock);
	INIT_WORK(&sport->send_tx_start_work, sport_tx_start_work_func);
	INIT_WORK(&sport->send_rx_start_work, sport_rx_start_work_func);
	INIT_WORK(&sport->send_tx_stop_work, sport_tx_stop_work_func);
	INIT_WORK(&sport->send_rx_stop_work, sport_rx_stop_work_func);
	INIT_WORK(&sport->get_sharc1_feature_work, get_sharc1_feature_work_func);
	INIT_WORK(&sport->get_sharc2_feature_work, get_sharc2_feature_work_func);

	init_waitqueue_head(&sport->pending_tx_stop_event);
	init_waitqueue_head(&sport->pending_rx_stop_event);

	sport->tx_alsa_icap_buf_id = -1;
	sport->tx_dma_icap_buf_id = -1;
	sport->rx_alsa_icap_buf_id = -1;
	sport->rx_dma_icap_buf_id = -1;

	sport->sharc_tx_core = -1;
	sport->sharc_rx_core = -1;

	sport->icap_tx_dev_id = -1;
	sport->icap_rx_dev_id = -1;

	sport_devices[4] = sport;

	dev_info(dev, "SPORT create success, SHARC-ALSA (PCM steram send to sharc for processing)\n");
	return sport;

err_free_data:
	kfree(sport);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(sport_create);

void sport_delete(struct sport_device *sport)
{

	cancel_work_sync(&sport->get_sharc1_feature_work);
	cancel_work_sync(&sport->get_sharc2_feature_work);
	cancel_work_sync(&sport->send_tx_start_work);
	cancel_work_sync(&sport->send_rx_start_work);
	cancel_work_sync(&sport->send_tx_stop_work);
	cancel_work_sync(&sport->send_rx_stop_work);

	sport_devices[4] = NULL;

	snd_dma_free_pages(&sport->sharc_tx_dma_buf);
	snd_dma_free_pages(&sport->sharc_rx_dma_buf);

	dmaengine_terminate_sync(sport->tx_dma_chan);
	dmaengine_terminate_sync(sport->rx_dma_chan);
	sport_free_resource(sport);
	kfree(sport);
}
EXPORT_SYMBOL(sport_delete);

MODULE_DESCRIPTION("Analog Devices SC5XX SPORT driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
MODULE_LICENSE("GPL v2");
