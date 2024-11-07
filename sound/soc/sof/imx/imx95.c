// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright 2024 NXP
//
// Author: Laurentiu Mihalcea <laurentiu.mihalcea@nxp.com>
//

#include <linux/module.h>
#include <linux/of_platform.h>
#include <sound/sof.h>
#include <linux/arm-smccc.h>
#include <linux/firmware/imx/dsp.h>
#include <linux/of_reserved_mem.h>
#include <linux/clk.h>

#include "../sof-of-dev.h"
#include "../ops.h"

#define IMX_SIP_SRC 0xC2000005
#define IMX_SIP_SRC_M_RESET_ADDR_SET 0x03

#define IMX95_CPU_VEC_FLAGS_BOOT BIT(29)

#define IMX_SIP_LMM 0xC200000F
#define IMX_SIP_LMM_BOOT 0x0
#define IMX_SIP_LMM_SHUTDOWN 0x1

#define IMX95_M7_LM_ID 0x1

#define MBOX_DSPBOX_OFFSET 0x1000

struct imx95_priv {
	struct platform_device *ipc_dev;
	struct imx_dsp_ipc *ipc_handle;
	resource_size_t bootaddr;
	struct clk_bulk_data *clks;
	int clk_num;
};

static void imx95_ipc_handle_reply(struct imx_dsp_ipc *ipc)
{
	unsigned long flags;
	struct snd_sof_dev *sdev = imx_dsp_get_data(ipc);

	spin_lock_irqsave(&sdev->ipc_lock, flags);
	snd_sof_ipc_process_reply(sdev, 0);
	spin_unlock_irqrestore(&sdev->ipc_lock, flags);
}

static void imx95_ipc_handle_request(struct imx_dsp_ipc *ipc)
{
	snd_sof_ipc_msgs_rx(imx_dsp_get_data(ipc));
}

static struct imx_dsp_ops ipc_ops = {
	.handle_reply = imx95_ipc_handle_reply,
	.handle_request = imx95_ipc_handle_request,
};

static int imx95_probe(struct snd_sof_dev *sdev)
{
	struct platform_device *pdev;
	struct imx95_priv *priv;
	struct resource *res;
	struct arm_smccc_res smc_ret;
	int ret;

	pdev = container_of(sdev->dev, struct platform_device, dev);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return dev_err_probe(&pdev->dev, -ENOMEM, "failed to alloc priv\n");

	sdev->pdata->hw_pdata = priv;

	/* map DRAM */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dram");
	if (!res)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "failed to fetch DRAM region\n");

	sdev->bar[SOF_FW_BLK_TYPE_DRAM] = devm_ioremap(&pdev->dev, res->start,
						       resource_size(res));
	if (IS_ERR(sdev->bar[SOF_FW_BLK_TYPE_DRAM]))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(sdev->bar[SOF_FW_BLK_TYPE_DRAM]),
				     "failed to map DRAM region\n");

	sdev->mmio_bar = SOF_FW_BLK_TYPE_DRAM;
	priv->bootaddr = res->start;

	/* map mailbox region */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mailbox");
	if (!res) {
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "failed to fetch MAILBOX region\n");
	}
	sdev->bar[SOF_FW_BLK_TYPE_SRAM] = devm_ioremap_wc(&pdev->dev, res->start,
							  resource_size(res));

	if (IS_ERR(sdev->bar[SOF_FW_BLK_TYPE_SRAM]))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(sdev->bar[SOF_FW_BLK_TYPE_SRAM]),
				     "failed to map mailbox region\n");

	sdev->mailbox_bar = SOF_FW_BLK_TYPE_SRAM;
	sdev->dsp_box.offset = MBOX_DSPBOX_OFFSET;

	ret = of_reserved_mem_device_init(sdev->dev);
	if (ret) {
		return dev_err_probe(&pdev->dev, ret,
				     "failed to bind DMA region\n");
	}

	priv->ipc_dev = platform_device_register_data(&pdev->dev, "imx-dsp",
						      PLATFORM_DEVID_NONE,
						      pdev, sizeof(*pdev));
	if (IS_ERR(priv->ipc_dev))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(priv->ipc_dev),
				     "failed to create IPC device\n");

	priv->ipc_handle = dev_get_drvdata(&priv->ipc_dev->dev);
	if (!priv->ipc_handle) {
		ret = -EPROBE_DEFER;
		dev_err(&pdev->dev, "failed to fetch ipc handle\n");
		goto err_unregister_ipc_dev;
	}
	priv->ipc_handle->ops = &ipc_ops;
	imx_dsp_set_data(priv->ipc_handle, sdev);

	/* set core boot reset address */
	arm_smccc_smc(IMX_SIP_SRC, IMX_SIP_SRC_M_RESET_ADDR_SET, priv->bootaddr,
		      IMX95_CPU_VEC_FLAGS_BOOT, 0, 0, 0, 0, &smc_ret);

	if ((int)smc_ret.a0 < 0) {
		dev_err_probe(&pdev->dev, smc_ret.a0, "failed to set boot addr\n");
		goto err_unregister_ipc_dev;
	}

	ret = devm_clk_bulk_get_all(sdev->dev, &priv->clks);
	if (ret < 0) {
		dev_err_probe(&pdev->dev, ret, "failed to fetch clocks\n");
		goto err_unregister_ipc_dev;
	}
	priv->clk_num = ret;

	ret = clk_bulk_prepare_enable(priv->clk_num, priv->clks);
	if (ret < 0) {
		dev_err_probe(&pdev->dev, ret, "failed to enable clocks\n");
		goto err_unregister_ipc_dev;
	}

	return 0;

err_unregister_ipc_dev:
	platform_device_unregister(priv->ipc_dev);

	return ret;
}

static int imx95_disable_enable_core(struct imx95_priv *priv, bool enable)
{
	struct arm_smccc_res res;

	if (enable)
		arm_smccc_smc(IMX_SIP_LMM, IMX_SIP_LMM_BOOT, IMX95_M7_LM_ID,
			      0, 0, 0, 0, 0, &res);
	else
		arm_smccc_smc(IMX_SIP_LMM, IMX_SIP_LMM_SHUTDOWN, IMX95_M7_LM_ID,
			      0, 0, 0, 0, 0, &res);

	return res.a0;
}

static void imx95_remove(struct snd_sof_dev *sdev)
{
	struct imx95_priv *priv;

	priv = sdev->pdata->hw_pdata;

	if (imx95_disable_enable_core(priv, false)) {
		dev_err(sdev->dev, "failed to stop core\n");
	}

	clk_bulk_disable_unprepare(priv->clk_num, priv->clks);

	platform_device_unregister(priv->ipc_dev);
}

static int imx95_run(struct snd_sof_dev *sdev)
{
	return imx95_disable_enable_core(sdev->pdata->hw_pdata, true);
}

static int imx95_send_msg(struct snd_sof_dev *sdev, struct snd_sof_ipc_msg *msg)
{
	struct imx95_priv *priv = sdev->pdata->hw_pdata;

	sof_mailbox_write(sdev, sdev->host_box.offset, msg->msg_data,
			  msg->msg_size);

	imx_dsp_ring_doorbell(priv->ipc_handle, 0);

	return 0;
}

static int imx95_get_mailbox_offset(struct snd_sof_dev *sdev)
{
	return MBOX_DSPBOX_OFFSET;
}

static int imx95_get_bar_index(struct snd_sof_dev *sdev, u32 type)
{
	switch (type) {
	case SOF_FW_BLK_TYPE_SRAM:
	case SOF_FW_BLK_TYPE_DRAM:
		return type;
	default:
		return -EINVAL;
	}
}

static int imx95_get_window_offset(struct snd_sof_dev *sdev, u32 id)
{
	/* no offset for window regions - they are already relative to
	 * MAILBOX memory region described in DTS
	 */
	return 0;
}

static int imx95_set_power_state(struct snd_sof_dev *sdev,
				 const struct sof_dsp_power_state *target_state)
{
	sdev->dsp_power_state = *target_state;

	return 0;
}

static int imx95_suspend_resume(struct snd_sof_dev *sdev, bool suspend)
{
	struct imx95_priv *priv;
	int ret, i;

	priv = sdev->pdata->hw_pdata;

	if (suspend) {
		ret = imx95_disable_enable_core(priv, false);
		if (ret) {
			dev_err(sdev->dev, "failed to stop core\n");
			return ret;
		}

		clk_bulk_disable_unprepare(priv->clk_num, priv->clks);

		for (i = 0; i < DSP_MU_CHAN_NUM; i++)
			imx_dsp_free_channel(priv->ipc_handle, i);
	} else {
		ret = clk_bulk_prepare_enable(priv->clk_num, priv->clks);
		if (ret < 0) {
			dev_err(sdev->dev, "failed to enable clocks: %d\n", ret);
			return ret;
		}

		for (i = 0; i < DSP_MU_CHAN_NUM; i++)
			imx_dsp_request_channel(priv->ipc_handle, i);
	}

	return 0;
}

static int imx95_runtime_resume(struct snd_sof_dev *sdev)
{
	int ret;
	const struct sof_dsp_power_state target_state = {
		.state = SOF_DSP_PM_D0,
	};

	ret = imx95_suspend_resume(sdev, false);
	if (ret < 0) {
		dev_err(sdev->dev, "failed to runtime resume: %d\n", ret);
		return ret;
	}

	return snd_sof_dsp_set_power_state(sdev, &target_state);
}


static int imx95_resume(struct snd_sof_dev *sdev)
{
	int ret;
	const struct sof_dsp_power_state target_state = {
		.state = SOF_DSP_PM_D0,
	};

	ret = imx95_suspend_resume(sdev, false);
	if (ret < 0) {
		dev_err(sdev->dev, "failed to resume: %d\n", ret);
		return ret;
	}

	if (pm_runtime_suspended(sdev->dev)) {
		pm_runtime_disable(sdev->dev);
		pm_runtime_set_active(sdev->dev);
		pm_runtime_mark_last_busy(sdev->dev);
		pm_runtime_enable(sdev->dev);
		pm_runtime_idle(sdev->dev);
	}

	return snd_sof_dsp_set_power_state(sdev, &target_state);
}

static int imx95_runtime_suspend(struct snd_sof_dev *sdev)
{
	int ret;
	const struct sof_dsp_power_state target_state = {
		.state = SOF_DSP_PM_D3,
	};

	ret = imx95_suspend_resume(sdev, true);
	if (ret < 0) {
		dev_err(sdev->dev, "failed to runtime suspend: %d\n", ret);
		return ret;
	}

	return snd_sof_dsp_set_power_state(sdev, &target_state);
}

static int imx95_suspend(struct snd_sof_dev *sdev, unsigned int target_state)
{
	int ret;
	const struct sof_dsp_power_state target_power_state = {
		.state = target_state,
	};

	if (!pm_runtime_suspended(sdev->dev)) {
		ret = imx95_suspend_resume(sdev, true);
		if (ret < 0) {
			dev_err(sdev->dev, "failed to suspend: %d\n", ret);
			return ret;
		}
	}

	return snd_sof_dsp_set_power_state(sdev, &target_power_state);
}

static struct snd_soc_dai_driver imx95_dai[] = {
	{
		.name = "sai3",
		.playback = {
			.channels_min = 1,
			.channels_max = 32,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 32,
		},
	}
};

static const struct snd_sof_dsp_ops sof_imx95_ops = {
	.probe = imx95_probe,
	.remove = imx95_remove,

	/* mandatory "DSP" ops */
	.run = imx95_run,
	.block_read = sof_block_read,
	.block_write = sof_block_write,
	.send_msg = imx95_send_msg,
	.load_firmware = snd_sof_load_firmware_memcpy,
	.ipc_msg_data = sof_ipc_msg_data,

	.mailbox_read = sof_mailbox_read,
	.mailbox_write = sof_mailbox_write,

	.get_mailbox_offset = imx95_get_mailbox_offset,
	.get_bar_index = imx95_get_bar_index, /* TODO: try to remove */
	.get_window_offset = imx95_get_window_offset,

	.pcm_open = sof_stream_pcm_open,
	.pcm_close = sof_stream_pcm_close,
	.set_stream_data_offset = sof_set_stream_data_offset,

	.runtime_suspend = imx95_runtime_suspend,
	.runtime_resume = imx95_runtime_resume,

	.resume = imx95_resume,
	.suspend = imx95_suspend,

	.set_power_state = imx95_set_power_state,

	.drv = imx95_dai,
	.num_drv = ARRAY_SIZE(imx95_dai),

	.hw_info = SNDRV_PCM_INFO_MMAP |
		   SNDRV_PCM_INFO_MMAP_VALID |
		   SNDRV_PCM_INFO_INTERLEAVED |
		   SNDRV_PCM_INFO_PAUSE |
		   SNDRV_PCM_INFO_BATCH |
		   SNDRV_PCM_INFO_NO_PERIOD_WAKEUP
};

static struct snd_sof_of_mach sof_imx95_machs[] = {
	{
		.compatible = "fsl,imx95-19x19-evk",
		.sof_tplg_filename = "sof-imx95-wm8962.tplg",
		.drv_name = "asoc-audio-graph-card2",
	},
	{
		/* sentinel */
	},
};

static struct sof_dev_desc sof_of_imx95_desc = {
	.of_machines = sof_imx95_machs,
	.ipc_supported_mask = BIT(SOF_IPC_TYPE_3),
	.ipc_default = SOF_IPC_TYPE_3,
	.default_fw_path = {
		[SOF_IPC_TYPE_3] = "imx/sof",
	},
	.default_tplg_path = {
		[SOF_IPC_TYPE_3] = "imx/sof-tplg",
	},
	.default_fw_filename = {
		[SOF_IPC_TYPE_3] = "sof-imx95.ri",
	},
	.ops = &sof_imx95_ops,
};

static const struct of_device_id sof_of_imx95_ids[] = {
	{ .compatible = "fsl,imx95-cm7-dsp", .data = &sof_of_imx95_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sof_of_imx95_ids);

static struct platform_driver snd_sof_of_imx95_driver = {
	.probe = sof_of_probe,
	.remove_new = sof_of_remove,
	.driver = {
		.name = "sof-audio-of-imx95",
		.pm = &sof_of_pm,
		.of_match_table = sof_of_imx95_ids,
	},
};
module_platform_driver(snd_sof_of_imx95_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("SOF support for i.MX95 platforms");
