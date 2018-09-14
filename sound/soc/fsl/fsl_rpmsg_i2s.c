/*
 * Freescale ALSA SoC rpmsg i2s driver.
 *
 * Copyright 2017 NXP
 *
 * This program is free software, you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or(at your
 * option) any later version.
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>

#include "fsl_rpmsg_i2s.h"
#include "imx-pcm.h"

#define FSL_RPMSG_I2S_RATES	(SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|\
				SNDRV_PCM_RATE_48000)
#define FSL_RPMSG_I2S_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

static int i2s_send_message(struct i2s_rpmsg *msg,
			       struct i2s_info *info)
{
	int err = 0;

	mutex_lock(&info->tx_lock);
	if (!info->rpdev) {
		dev_err(info->dev, "rpmsg channel not ready, m4 image ready?\n");
		mutex_unlock(&info->tx_lock);
		return -EINVAL;
	}

	dev_dbg(&info->rpdev->dev, "send cmd %d\n", msg->send_msg.header.cmd);

	if (!(msg->send_msg.header.type == I2S_TYPE_C))
		reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)&msg->send_msg,
			 sizeof(struct i2s_rpmsg_s));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		mutex_unlock(&info->tx_lock);
		return err;
	}

	if (!(msg->send_msg.header.type == I2S_TYPE_C)) {
		/* wait response from rpmsg */
		err = wait_for_completion_timeout(&info->cmd_complete,
					  msecs_to_jiffies(RPMSG_TIMEOUT));
		if (!err) {
			dev_err(&info->rpdev->dev,
					"rpmsg_send cmd %d timeout!\n",
					msg->send_msg.header.cmd);
			mutex_unlock(&info->tx_lock);
			return -ETIMEDOUT;
		}
		memcpy(&msg->recv_msg, &info->recv_msg,
					sizeof(struct i2s_rpmsg_r));
		memcpy(&info->rpmsg[msg->recv_msg.header.cmd].recv_msg,
			&msg->recv_msg, sizeof(struct i2s_rpmsg_r));
	}

	dev_dbg(&info->rpdev->dev, "cmd:%d, resp %d\n",
						msg->send_msg.header.cmd,
						info->recv_msg.param.resp);
	mutex_unlock(&info->tx_lock);

	return 0;
}

static const unsigned int fsl_rpmsg_rates[] = {
	32000, 48000, 96000, 88200, 176400, 192000,
	352800, 384000, 705600, 768000, 1411200, 2822400,
};

static const struct snd_pcm_hw_constraint_list fsl_rpmsg_rate_constraints = {
	.count = ARRAY_SIZE(fsl_rpmsg_rates),
	.list = fsl_rpmsg_rates,
};

static int fsl_rpmsg_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &fsl_rpmsg_rate_constraints);

	return ret;
}

static const struct snd_soc_dai_ops fsl_rpmsg_dai_ops = {
	.startup	= fsl_rpmsg_startup,
};

static struct snd_soc_dai_driver fsl_rpmsg_i2s_dai = {
	.playback = {
		.stream_name = "CPU-Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_RPMSG_I2S_FORMATS,
	},
	.capture = {
		.stream_name = "CPU-Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_RPMSG_I2S_FORMATS,
	},
	.symmetric_rates      = 1,
	.symmetric_channels   = 1,
	.symmetric_samplebits = 1,
	.ops = &fsl_rpmsg_dai_ops,
};

static const struct snd_soc_component_driver fsl_component = {
	.name           = "fsl-rpmsg-i2s",
};

static const struct of_device_id fsl_rpmsg_i2s_ids[] = {
	{ .compatible = "fsl,imx7ulp-rpmsg-i2s"},
	{ .compatible = "fsl,imx8mq-rpmsg-i2s"},
	{ .compatible = "fsl,imx8qxp-rpmsg-i2s"},
	{ .compatible = "fsl,imx8qm-rpmsg-i2s"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_rpmsg_i2s_ids);

static void rpmsg_i2s_work(struct work_struct *work)
{
	struct work_of_rpmsg *work_of_rpmsg;
	struct i2s_info *i2s_info;

	work_of_rpmsg = container_of(work, struct work_of_rpmsg, work);
	i2s_info = work_of_rpmsg->i2s_info;

	i2s_send_message(&work_of_rpmsg->msg, i2s_info);
}

static int fsl_rpmsg_i2s_probe(struct platform_device *pdev)
{
	struct device_node *np     = pdev->dev.of_node;
	struct fsl_rpmsg_i2s         *rpmsg_i2s;
	struct i2s_info              *i2s_info;
	int audioindex = 0;
	int ret;
	int i;

	rpmsg_i2s = devm_kzalloc(&pdev->dev, sizeof(struct fsl_rpmsg_i2s),
								GFP_KERNEL);
	if (!rpmsg_i2s)
		return -ENOMEM;

	rpmsg_i2s->pdev = pdev;
	i2s_info =  &rpmsg_i2s->i2s_info;

	ret = of_property_read_u32(np, "fsl,audioindex", &audioindex);
	if (ret)
		audioindex = 0;

	/* Setup work queue */
	i2s_info->rpmsg_wq = create_singlethread_workqueue("rpmsg_i2s");
	if (i2s_info->rpmsg_wq == NULL) {
		dev_err(&pdev->dev, "workqueue create failed\n");
		return -ENOMEM;
	}

	i2s_info->send_message = i2s_send_message;

	for (i = 0; i < WORK_MAX_NUM; i++) {
		INIT_WORK(&i2s_info->work_list[i].work, rpmsg_i2s_work);
		i2s_info->work_list[i].i2s_info = i2s_info;
	}

	for (i = 0; i < I2S_CMD_MAX_NUM ; i++) {
		i2s_info->rpmsg[i].send_msg.header.cate  = IMX_RPMSG_AUDIO;
		i2s_info->rpmsg[i].send_msg.header.major = IMX_RMPSG_MAJOR;
		i2s_info->rpmsg[i].send_msg.header.minor = IMX_RMPSG_MINOR;
		i2s_info->rpmsg[i].send_msg.header.type  = I2S_TYPE_A;
		i2s_info->rpmsg[i].send_msg.param.audioindex = audioindex;
	}

	mutex_init(&i2s_info->tx_lock);
	mutex_init(&i2s_info->i2c_lock);

	if (of_device_is_compatible(pdev->dev.of_node,
				    "fsl,imx7ulp-rpmsg-i2s")) {
		rpmsg_i2s->codec_wm8960 = 1;
		rpmsg_i2s->version = 1;
		rpmsg_i2s->rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_48000;
		rpmsg_i2s->formats =  SNDRV_PCM_FMTBIT_S16_LE;
		fsl_rpmsg_i2s_dai.playback.rates = rpmsg_i2s->rates;
		fsl_rpmsg_i2s_dai.playback.formats = rpmsg_i2s->formats;
		fsl_rpmsg_i2s_dai.capture.rates = rpmsg_i2s->rates;
		fsl_rpmsg_i2s_dai.capture.formats = rpmsg_i2s->formats;
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "fsl,imx8qxp-rpmsg-i2s")) {
		rpmsg_i2s->codec_wm8960 = 1 + (1 << 16);
		rpmsg_i2s->version = 1;
		rpmsg_i2s->codec_cs42888 = 1 + (2 << 16);
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "fsl,imx8qm-rpmsg-i2s")) {
		rpmsg_i2s->codec_wm8960 = 0;
		rpmsg_i2s->version = 1;
		rpmsg_i2s->codec_cs42888 = 1 + (0 << 16);
	}

	if (of_device_is_compatible(pdev->dev.of_node,
				    "fsl,imx8mq-rpmsg-i2s")) {
		rpmsg_i2s->codec_dummy = 0;
		rpmsg_i2s->codec_ak4497 = 1;
		rpmsg_i2s->version = 2;
		rpmsg_i2s->rates = SNDRV_PCM_RATE_KNOT;
		rpmsg_i2s->formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_DSD_U8 |
					SNDRV_PCM_FMTBIT_DSD_U16_LE |
					SNDRV_PCM_FMTBIT_DSD_U32_LE;

		fsl_rpmsg_i2s_dai.playback.rates = rpmsg_i2s->rates;
		fsl_rpmsg_i2s_dai.playback.formats = rpmsg_i2s->formats;
		fsl_rpmsg_i2s_dai.capture.rates = rpmsg_i2s->rates;
		fsl_rpmsg_i2s_dai.capture.formats = rpmsg_i2s->formats;
	}

	if (of_property_read_bool(pdev->dev.of_node, "fsl,enable-lpa"))
		rpmsg_i2s->enable_lpa = 1;

	if (of_property_read_u32(np, "fsl,dma-buffer-size",
					&i2s_info->prealloc_buffer_size))
		i2s_info->prealloc_buffer_size = IMX_DEFAULT_DMABUF_SIZE;

	platform_set_drvdata(pdev, rpmsg_i2s);
	pm_runtime_enable(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_component,
			&fsl_rpmsg_i2s_dai, 1);
	if (ret)
		return ret;

	return imx_rpmsg_platform_register(&pdev->dev);
}

static int fsl_rpmsg_i2s_remove(struct platform_device *pdev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = platform_get_drvdata(pdev);
	struct i2s_info      *i2s_info  = &rpmsg_i2s->i2s_info;

	if (i2s_info->rpmsg_wq)
		destroy_workqueue(i2s_info->rpmsg_wq);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_rpmsg_i2s_runtime_resume(struct device *dev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(dev);

	pm_qos_add_request(&rpmsg_i2s->pm_qos_req, PM_QOS_CPU_DMA_LATENCY, 0);
	return 0;
}

static int fsl_rpmsg_i2s_runtime_suspend(struct device *dev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(dev);

	pm_qos_remove_request(&rpmsg_i2s->pm_qos_req);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int fsl_rpmsg_i2s_suspend(struct device *dev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(dev);
	struct i2s_info  *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg *rpmsg_tx;
	struct i2s_rpmsg *rpmsg_rx;

	flush_workqueue(i2s_info->rpmsg_wq);
	rpmsg_tx = &i2s_info->rpmsg[I2S_TX_SUSPEND];
	rpmsg_rx = &i2s_info->rpmsg[I2S_RX_SUSPEND];

	rpmsg_tx->send_msg.header.cmd = I2S_TX_SUSPEND;
	i2s_send_message(rpmsg_tx, i2s_info);

	rpmsg_rx->send_msg.header.cmd = I2S_RX_SUSPEND;
	i2s_send_message(rpmsg_rx, i2s_info);

	return 0;
}

static int fsl_rpmsg_i2s_resume(struct device *dev)
{
	struct fsl_rpmsg_i2s *rpmsg_i2s = dev_get_drvdata(dev);
	struct i2s_info  *i2s_info =  &rpmsg_i2s->i2s_info;
	struct i2s_rpmsg *rpmsg_tx;
	struct i2s_rpmsg *rpmsg_rx;

	rpmsg_tx = &i2s_info->rpmsg[I2S_TX_RESUME];
	rpmsg_rx = &i2s_info->rpmsg[I2S_RX_RESUME];

	rpmsg_tx->send_msg.header.cmd = I2S_TX_RESUME;
	i2s_send_message(rpmsg_tx, i2s_info);

	rpmsg_rx->send_msg.header.cmd = I2S_RX_RESUME;
	i2s_send_message(rpmsg_rx, i2s_info);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_rpmsg_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_rpmsg_i2s_runtime_suspend,
			   fsl_rpmsg_i2s_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_rpmsg_i2s_suspend, fsl_rpmsg_i2s_resume)
};

static struct platform_driver fsl_rpmsg_i2s_driver = {
	.probe  = fsl_rpmsg_i2s_probe,
	.remove	= fsl_rpmsg_i2s_remove,
	.driver = {
		.name = "fsl-rpmsg-i2s",
		.pm = &fsl_rpmsg_i2s_pm_ops,
		.of_match_table = fsl_rpmsg_i2s_ids,
	},
};

module_platform_driver(fsl_rpmsg_i2s_driver);

MODULE_DESCRIPTION("Freescale Soc rpmsg_i2s Interface");
MODULE_AUTHOR("Shengjiu Wang <shengjiu.wang@freescale.com>");
MODULE_ALIAS("platform:fsl-rpmsg_i2s");
MODULE_LICENSE("GPL");
