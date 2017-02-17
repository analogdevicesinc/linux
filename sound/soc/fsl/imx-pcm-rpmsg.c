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

static const struct snd_pcm_hardware imx_rpmsg_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.buffer_bytes_max = IMX_SAI_DMABUF_SIZE,
	.period_bytes_min = 128,
	.period_bytes_max = 65535, /* Limited by SDMA engine */
	.periods_min = 2,
	.periods_max = 255,
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
	struct i2s_rpmsg_s *rpmsg = &i2s_info->send_msg[substream->stream];

	rpmsg->param.rate     = params_rate(params);
	if (SNDRV_PCM_FORMAT_S16_LE ==  params_format(params))
		rpmsg->param.format   = 0;
	else
		rpmsg->param.format   = 1;

	if (params_channels(params) == 1)
		rpmsg->param.channels = 0;
	else
		rpmsg->param.channels = 2;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->header.cmd = I2S_TX_HW_PARAM;
	else
		rpmsg->header.cmd = I2S_RX_HW_PARAM;

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
	struct dma_tx_state state;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[substream->stream];
	unsigned int buf_size;
	unsigned int pos = 0;
	unsigned long flags;

	spin_lock_irqsave(&i2s_info->lock[substream->stream], flags);
	state.residue = (i2s_info->num_period[substream->stream] -
				  rpmsg->param.buffer_tail)
				* rpmsg->param.period_size;
	spin_unlock_irqrestore(&i2s_info->lock[substream->stream], flags);

	buf_size = snd_pcm_lib_buffer_bytes(substream);
	if (state.residue > 0 && state.residue <= buf_size)
		pos = buf_size - state.residue;

	return bytes_to_frames(substream->runtime, pos);
}

static int imx_rpmsg_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[substream->stream];
	struct dmaengine_pcm_runtime_data *prtd;

	snd_soc_set_runtime_hwparams(substream, &imx_rpmsg_pcm_hardware);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->header.cmd = I2S_TX_OPEN;
	else
		rpmsg->header.cmd = I2S_RX_OPEN;
	i2s_info->send_message(rpmsg, i2s_info);

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	substream->runtime->private_data = prtd;

	return ret;
}

static int imx_rpmsg_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[substream->stream];
	struct dmaengine_pcm_runtime_data *prtd =
					substream->runtime->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rpmsg->header.cmd = I2S_TX_CLOSE;
	else
		rpmsg->header.cmd = I2S_RX_CLOSE;

	flush_workqueue(i2s_info->rpmsg_wq);
	i2s_info->send_message(rpmsg, i2s_info);

	kfree(prtd);

	return ret;
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
	struct dmaengine_pcm_runtime_data *prtd
					= substream->runtime->private_data;

	prtd->pos += snd_pcm_lib_period_bytes(substream);
	if (prtd->pos >= snd_pcm_lib_buffer_bytes(substream))
		prtd->pos = 0;

	snd_pcm_period_elapsed(substream);
}

static int imx_rpmsg_pcm_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[substream->stream];
	u8 cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cmd = I2S_TX_BUFFER;
	else
		cmd = I2S_RX_BUFFER;

	rpmsg->header.cmd = cmd;
	rpmsg->param.buffer_addr    =  substream->runtime->dma_addr;
	rpmsg->param.buffer_size    =  snd_pcm_lib_buffer_bytes(substream);
	rpmsg->param.period_size    =  snd_pcm_lib_period_bytes(substream);
	rpmsg->param.buffer_tail    = 0;

	i2s_info->num_period[substream->stream] =
			rpmsg->param.buffer_size/rpmsg->param.period_size;

	memcpy(&i2s_info->work_list[cmd].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[cmd].work);

	i2s_info->callback[substream->stream] = imx_rpmsg_pcm_dma_complete;
	i2s_info->callback_param[substream->stream] = substream;
	return 0;
}

static void imx_rpmsg_async_issue_pending(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai   *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info      *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s   *rpmsg = &i2s_info->send_msg[substream->stream];
	u8 cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cmd = I2S_TX_START;
	else
		cmd = I2S_RX_START;

	rpmsg->header.cmd = cmd;
	memcpy(&i2s_info->work_list[cmd].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[cmd].work);
}

static int imx_rpmsg_resume(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s     *rpmsg = &i2s_info->send_msg[substream->stream];
	u8 cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cmd = I2S_TX_RESTART;
	else
		cmd = I2S_RX_RESTART;

	rpmsg->header.cmd = cmd;
	memcpy(&i2s_info->work_list[cmd].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[cmd].work);

	return 0;
}

static int imx_rpmsg_pause(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s     *rpmsg = &i2s_info->send_msg[substream->stream];
	u8 cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cmd = I2S_TX_PAUSE;
	else
		cmd = I2S_RX_PAUSE;

	rpmsg->header.cmd = cmd;
	memcpy(&i2s_info->work_list[cmd].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[cmd].work);

	return 0;
}

static int imx_rpmsg_terminate_all(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai         *cpu_dai = rtd->cpu_dai;
	struct fsl_rpmsg_i2s       *rpmsg_i2s = dev_get_drvdata(cpu_dai->dev);
	struct i2s_info            *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg_s     *rpmsg = &i2s_info->send_msg[substream->stream];
	u8 cmd;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cmd = I2S_TX_TERMINATE;
	else
		cmd = I2S_RX_TERMINATE;

	rpmsg->header.cmd = cmd;
	memcpy(&i2s_info->work_list[cmd].msg, rpmsg,
						sizeof(struct i2s_rpmsg_s));
	queue_work(i2s_info->rpmsg_wq, &i2s_info->work_list[cmd].work);

	return 0;
}

int imx_rpmsg_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = imx_rpmsg_pcm_prepare_and_submit(substream);
		if (ret)
			return ret;
		imx_rpmsg_async_issue_pending(substream);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		imx_rpmsg_resume(substream);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (runtime->info & SNDRV_PCM_INFO_PAUSE)
			imx_rpmsg_pause(substream);
		else
			imx_rpmsg_terminate_all(substream);
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

static struct snd_pcm_ops imx_rpmsg_pcm_ops = {
	.open		= imx_rpmsg_pcm_open,
	.close		= imx_rpmsg_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= imx_rpmsg_pcm_hw_params,
	.hw_free	= imx_rpmsg_pcm_hw_free,
	.trigger	= imx_rpmsg_pcm_trigger,
	.pointer	= imx_rpmsg_pcm_pointer,
	.mmap		= imx_rpmsg_pcm_mmap,
};

static int imx_rpmsg_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = imx_rpmsg_pcm_hardware.buffer_bytes_max;

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
	int ret;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = imx_rpmsg_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
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
	struct i2s_rpmsg_s *rpmsg;
	unsigned long flags;

	dev_dbg(&rpdev->dev, "get from%d: cmd:%d.\n", src, msg->header.cmd);

	memcpy(&i2s_info_g->recv_msg, msg, sizeof(struct i2s_rpmsg_r));

	if (msg->header.type == I2S_TYPE_C) {
		if (msg->header.cmd == I2S_TX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[0], flags);
			rpmsg = &i2s_info_g->send_msg[0];
			rpmsg->param.buffer_tail++;
			rpmsg->param.buffer_tail %= i2s_info_g->num_period[0];
			spin_unlock_irqrestore(&i2s_info_g->lock[0], flags);
			i2s_info_g->callback[0](i2s_info_g->callback_param[0]);
		} else if (msg->header.cmd == I2S_RX_PERIOD_DONE) {
			spin_lock_irqsave(&i2s_info_g->lock[1], flags);
			rpmsg = &i2s_info_g->send_msg[1];
			rpmsg->param.buffer_tail++;
			rpmsg->param.buffer_tail %= i2s_info_g->num_period[1];
			spin_unlock_irqrestore(&i2s_info_g->lock[1], flags);
			i2s_info_g->callback[1](i2s_info_g->callback_param[1]);
		}
	}

	if (msg->header.type == I2S_TYPE_B)
		complete(&i2s_info_g->cmd_complete);

	return 0;
}

static int i2s_rpmsg_probe(struct rpmsg_device *rpdev)
{
	i2s_info_g->rpdev = rpdev;

	init_completion(&i2s_info_g->cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);
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
