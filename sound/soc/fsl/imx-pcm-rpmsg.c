/*
 * imx-rpmsg-platform.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/imx_rpmsg.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include "imx-pcm.h"
#include "fsl_rpmsg_i2s.h"

struct i2s_info *i2s_info_g;

static struct snd_pcm_hardware imx_rpmsg_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.buffer_bytes_max = IMX_SAI_DMABUF_SIZE,
	.period_bytes_min = 512,
	.period_bytes_max = 65532, /* Limited by SDMA engine */
	.periods_min = 2,
	.periods_max = 6000,
	.fifo_size = 0,
};

static int imx_rpmsg_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info  *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg *rpmsg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_HW_PARAM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_HW_PARAM];

	rpmsg->send_msg.param.rate = params_rate(params);

	if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
		rpmsg->send_msg.param.format   = RPMSG_S16_LE;
	else if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE)
		rpmsg->send_msg.param.format   = RPMSG_S24_LE;
	else
		rpmsg->send_msg.param.format   = RPMSG_S32_LE;

	if (params_channels(params) == 1)
		rpmsg->send_msg.param.channels = RPMSG_CH_LEFT;
	else
		rpmsg->send_msg.param.channels = RPMSG_CH_STEREO;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_HW_PARAM;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_HW_PARAM;

	i2s_info->send_message(rpmsg, i2s_info);

	return 0;
}

static int imx_rpmsg_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static snd_pcm_uframes_t imx_rpmsg_pcm_pointer(
				struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	unsigned int pos = 0;
	struct i2s_rpmsg *rpmsg;
	int buffer_tail = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_POINTER];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_POINTER];

	buffer_tail = rpmsg->recv_msg.param.buffer_offset /
				snd_pcm_lib_period_bytes(substream);
	pos = buffer_tail * snd_pcm_lib_period_bytes(substream);

	return bytes_to_frames(substream->runtime, pos);
}

static void imx_rpmsg_timer_callback(unsigned long data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	u8 index = i2s_info->work_index;
	int time_msec;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_POINTER];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_POINTER];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_POINTER;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_POINTER;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;

	if (rpmsg_i2s->force_lpa) {
		time_msec = min(500,
			    (int)(runtime->period_size*1000/runtime->rate));
		mod_timer(&i2s_info->stream_timer[substream->stream],
			     jiffies + msecs_to_jiffies(time_msec));
	}
}

static int imx_rpmsg_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	struct dmaengine_pcm_runtime_data *prtd;
	int cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_OPEN];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_OPEN];

	imx_rpmsg_pcm_hardware.buffer_bytes_max =
					i2s_info->prealloc_buffer_size;
	imx_rpmsg_pcm_hardware.period_bytes_max =
			imx_rpmsg_pcm_hardware.buffer_bytes_max / 2;

	snd_soc_set_runtime_hwparams(substream, &imx_rpmsg_pcm_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_OPEN;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_OPEN;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cmd = I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_TX_POINTER].recv_msg.param.buffer_offset = 0;
	} else {
		cmd = I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM;
		i2s_info->rpmsg[cmd].send_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[cmd].recv_msg.param.buffer_tail = 0;
		i2s_info->rpmsg[I2S_RX_POINTER].recv_msg.param.buffer_offset = 0;
	}

	i2s_info->send_message(rpmsg, i2s_info);

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	substream->runtime->private_data = prtd;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;


	/*create thread*/
	setup_timer(&i2s_info->stream_timer[substream->stream],
			imx_rpmsg_timer_callback, (unsigned long)substream);

	return ret;
}

static int imx_rpmsg_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	struct dmaengine_pcm_runtime_data *prtd =
					substream->runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_CLOSE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_CLOSE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_CLOSE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_CLOSE;
	flush_workqueue(i2s_info->rpmsg_wq);
	i2s_info->send_message(rpmsg, i2s_info);

	del_timer(&i2s_info->stream_timer[substream->stream]);

	kfree(prtd);

	return ret;
}

static int imx_rpmsg_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);

	/* NON-MMAP mode, NONBLOCK, Version 2, enable lpa in dts
	 * four condition to determine the lpa is enabled.
	 */
	if ((runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED ||
		runtime->access == SNDRV_PCM_ACCESS_RW_NONINTERLEAVED) &&
		(substream->f_flags & O_NONBLOCK) &&
			rpmsg_i2s->version == 2 &&
			rpmsg_i2s->enable_lpa)
		rpmsg_i2s->force_lpa = 1;
	else
		rpmsg_i2s->force_lpa = 0;

	return 0;
}

static int imx_rpmsg_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

static void imx_rpmsg_pcm_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg *rpmsg, *rpmsg2;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rpmsg = &i2s_info->rpmsg[I2S_TX_POINTER];
		rpmsg2 = &i2s_info->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];
	} else {
		rpmsg = &i2s_info->rpmsg[I2S_RX_POINTER];
		rpmsg2 = &i2s_info->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];
	}

	rpmsg->recv_msg.param.buffer_offset =
		rpmsg2->recv_msg.param.buffer_tail
				* snd_pcm_lib_period_bytes(substream);

	snd_pcm_period_elapsed(substream);
}

static int imx_rpmsg_pcm_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg   *rpmsg;
	u8 index = i2s_info->work_index;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_BUFFER];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_BUFFER];


	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_BUFFER;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_BUFFER;

	rpmsg->send_msg.param.buffer_addr = substream->runtime->dma_addr;
	rpmsg->send_msg.param.buffer_size = snd_pcm_lib_buffer_bytes(substream);
	rpmsg->send_msg.param.period_size = snd_pcm_lib_period_bytes(substream);
	rpmsg->send_msg.param.buffer_tail = 0;

	i2s_info->num_period[substream->stream] =
			rpmsg->send_msg.param.buffer_size /
				rpmsg->send_msg.param.period_size;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;

	i2s_info->callback[substream->stream] = imx_rpmsg_pcm_dma_complete;
	i2s_info->callback_param[substream->stream] = substream;
	return 0;
}

static void imx_rpmsg_async_issue_pending(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	u8 index = i2s_info->work_index;
	int time_msec;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_START];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_START];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_START;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_START;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;

	if (rpmsg_i2s->force_lpa) {
		time_msec = min(500,
			    (int)(runtime->period_size*1000/runtime->rate));
		mod_timer(&i2s_info->stream_timer[substream->stream],
			    jiffies + msecs_to_jiffies(time_msec));
	}
}

static int imx_rpmsg_restart(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	u8 index = i2s_info->work_index;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_RESTART];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_RESTART];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_RESTART;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_RESTART;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;

	return 0;
}

static int imx_rpmsg_pause(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	u8 index = i2s_info->work_index;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PAUSE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PAUSE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_PAUSE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_PAUSE;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;
	return 0;
}

static int imx_rpmsg_terminate_all(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg     *rpmsg;
	u8 index = i2s_info->work_index;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_TERMINATE];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_TERMINATE];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_TERMINATE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_TERMINATE;

	memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[index].work);
	i2s_info->work_index++;
	i2s_info->work_index %= WORK_MAX_NUM;

	del_timer(&i2s_info->stream_timer[substream->stream]);
	return 0;
}

int imx_rpmsg_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai     *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s   *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = imx_rpmsg_pcm_prepare_and_submit(substream);
		if (ret)
			return ret;
		imx_rpmsg_async_issue_pending(substream);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		if (rpmsg_i2s->force_lpa)
			break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		imx_rpmsg_restart(substream);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (!rpmsg_i2s->force_lpa) {
			if (runtime->info & SNDRV_PCM_INFO_PAUSE)
				imx_rpmsg_pause(substream);
			else
				imx_rpmsg_terminate_all(substream);
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		imx_rpmsg_pause(substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		imx_rpmsg_terminate_all(substream);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int imx_rpmsg_pcm_ack(struct snd_pcm_substream *substream)
{
	/*send the hw_avail size through rpmsg*/
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg           *rpmsg;
	u8 index = i2s_info->work_index;
	int buffer_tail = 0;

	if (!rpmsg_i2s->force_lpa)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg = &i2s_info->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];
	else
		rpmsg = &i2s_info->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->send_msg.header.cmd = I2S_TX_PERIOD_DONE;
	else
		rpmsg->send_msg.header.cmd = I2S_RX_PERIOD_DONE;

	rpmsg->send_msg.header.type = I2S_TYPE_C;

	buffer_tail = (frames_to_bytes(runtime, runtime->control->appl_ptr) %
				snd_pcm_lib_buffer_bytes(substream));
	buffer_tail = buffer_tail /  snd_pcm_lib_period_bytes(substream);

	if (buffer_tail != rpmsg->send_msg.param.buffer_tail) {
		rpmsg->send_msg.param.buffer_tail = buffer_tail;
		memcpy(&i2s_info->work_list[index].msg, rpmsg,
					sizeof(struct i2s_rpmsg_s));
		queue_work(i2s_info->rpmsg_wq,
					&i2s_info->work_list[index].work);
		i2s_info->work_index++;
		i2s_info->work_index %= WORK_MAX_NUM;
	}

	return 0;
}

static struct snd_pcm_ops imx_rpmsg_pcm_ops = {
	.open		= imx_rpmsg_pcm_open,
	.close		= imx_rpmsg_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= imx_rpmsg_pcm_hw_params,
	.hw_free	= imx_rpmsg_pcm_hw_free,
	.trigger	= imx_rpmsg_pcm_trigger,
	.pointer	= imx_rpmsg_pcm_pointer,
	.mmap		= imx_rpmsg_pcm_mmap,
	.ack		= imx_rpmsg_pcm_ack,
	.prepare	= imx_rpmsg_pcm_prepare,
};

static int imx_rpmsg_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream, int size)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void imx_rpmsg_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = SNDRV_PCM_STREAM_PLAYBACK;
			stream < SNDRV_PCM_STREAM_LAST; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int imx_rpmsg_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info *i2s_info =  &rpmsg_i2s->i2s_info;
	int ret;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK,
			i2s_info->prealloc_buffer_size);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE,
			i2s_info->prealloc_buffer_size);
		if (ret)
			goto out;
	}

out:
	/* free preallocated buffers in case of error */
	if (ret)
		imx_rpmsg_pcm_free_dma_buffers(pcm);

	return ret;
}

static struct snd_soc_platform_driver imx_rpmsg_soc_platform = {
	.ops		= &imx_rpmsg_pcm_ops,
	.pcm_new	= imx_rpmsg_pcm_new,
	.pcm_free	= imx_rpmsg_pcm_free_dma_buffers,
};

int imx_rpmsg_platform_register(struct device *dev)
{
	struct fsl_rpmsg_i2s  *rpmsg_i2s = dev_get_drvdata(dev);

	i2s_info_g	=  &rpmsg_i2s->i2s_info;

	return devm_snd_soc_register_platform(dev, &imx_rpmsg_soc_platform);
}
EXPORT_SYMBOL_GPL(imx_rpmsg_platform_register);

static int i2s_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct i2s_rpmsg_r *msg = (struct i2s_rpmsg_r *)data;
	struct i2s_rpmsg *rpmsg;
	unsigned long flags;

	dev_dbg(&rpdev->dev, "get from%d: cmd:%d. %d\n",
				src, msg->header.cmd, msg->param.resp);

	if (msg->header.type == I2S_TYPE_C) {
		if (msg->header.cmd == I2S_TX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[0], flags);
			rpmsg = &i2s_info_g->rpmsg[I2S_TX_PERIOD_DONE + I2S_TYPE_A_NUM];

			if (msg->header.major == 1 && msg->header.minor == 2)
				rpmsg->recv_msg.param.buffer_tail =
							msg->param.buffer_tail;
			else
				rpmsg->recv_msg.param.buffer_tail++;

			rpmsg->recv_msg.param.buffer_tail %=
						i2s_info_g->num_period[0];

			spin_unlock_irqrestore(&i2s_info_g->lock[0], flags);
			i2s_info_g->callback[0](i2s_info_g->callback_param[0]);

		} else if (msg->header.cmd == I2S_RX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[1], flags);
			rpmsg = &i2s_info_g->rpmsg[I2S_RX_PERIOD_DONE + I2S_TYPE_A_NUM];

			if (msg->header.major == 1 && msg->header.minor == 2)
				rpmsg->recv_msg.param.buffer_tail =
							msg->param.buffer_tail;
			else
				rpmsg->recv_msg.param.buffer_tail++;

			rpmsg->recv_msg.param.buffer_tail %=
						i2s_info_g->num_period[1];
			spin_unlock_irqrestore(&i2s_info_g->lock[1], flags);
			i2s_info_g->callback[1](i2s_info_g->callback_param[1]);
		}
	}

	if (msg->header.type == I2S_TYPE_B) {
		memcpy(&i2s_info_g->recv_msg, msg, sizeof(struct i2s_rpmsg_r));
		complete(&i2s_info_g->cmd_complete);
	}

	return 0;
}

static int i2s_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct platform_device *codec_pdev;
	struct fsl_rpmsg_i2s *rpmsg_i2s = NULL;
	struct fsl_rpmsg_codec  rpmsg_codec[2];
	int ret;

	if (!i2s_info_g)
		return 0;

	i2s_info_g->rpdev = rpdev;

	init_completion(&i2s_info_g->cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	rpmsg_i2s = container_of(i2s_info_g, struct fsl_rpmsg_i2s, i2s_info);

	if (rpmsg_i2s->codec_wm8960) {
		rpmsg_codec[0].audioindex = rpmsg_i2s->codec_wm8960 >> 16;
		rpmsg_codec[0].shared_lrclk = true;
		rpmsg_codec[0].capless = false;
		codec_pdev = platform_device_register_data(
					&rpmsg_i2s->pdev->dev,
					RPMSG_CODEC_DRV_NAME_WM8960,
					PLATFORM_DEVID_NONE,
					&rpmsg_codec[0], sizeof(struct fsl_rpmsg_codec));
		if (IS_ERR(codec_pdev)) {
			dev_err(&rpdev->dev,
				"failed to register rpmsg audio codec\n");
			ret = PTR_ERR(codec_pdev);
			return ret;
		}
	}

	if (rpmsg_i2s->codec_cs42888) {
		rpmsg_codec[1].audioindex = rpmsg_i2s->codec_cs42888 >> 16;
		strcpy(rpmsg_codec[1].name, "cs42888");
		rpmsg_codec[1].num_adcs = 2;

		codec_pdev = platform_device_register_data(
					&rpmsg_i2s->pdev->dev,
					RPMSG_CODEC_DRV_NAME_CS42888,
					PLATFORM_DEVID_NONE,
					&rpmsg_codec[1], sizeof(struct fsl_rpmsg_codec));
		if (IS_ERR(codec_pdev)) {
			dev_err(&rpdev->dev,
				"failed to register rpmsg audio codec\n");
			ret = PTR_ERR(codec_pdev);
			return ret;
		}
	}
	return 0;
}

static void i2s_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "i2s rpmsg driver is removed\n");
}

static struct rpmsg_device_id i2s_rpmsg_id_table[] = {
	{ .name	= "rpmsg-audio-channel" },
	{ },
};

static struct rpmsg_driver i2s_rpmsg_driver = {
	.drv.name	= "i2s_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= i2s_rpmsg_id_table,
	.probe		= i2s_rpmsg_probe,
	.callback	= i2s_rpmsg_cb,
	.remove		= i2s_rpmsg_remove,
};

static int __init i2s_rpmsg_init(void)
{
	return register_rpmsg_driver(&i2s_rpmsg_driver);
}

static void __exit i2s_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&i2s_rpmsg_driver);
}
module_init(i2s_rpmsg_init);
module_exit(i2s_rpmsg_exit);

MODULE_LICENSE("GPL");
