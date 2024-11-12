// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/genalloc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "se_ctrl.h"

#define MBOX_TX_NAME			"tx"
#define MBOX_RX_NAME			"rx"
#define SE_TYPE_HSM			"hsm"

struct se_fw_load_info {
	const u8 *prim_fw_nm_in_rfs;
	const u8 *seco_fw_nm_in_rfs;
	struct mutex se_fw_load;
	bool is_fw_loaded;
	bool handle_susp_resm;
	struct se_imem_buf imem;
};

struct se_if_node_info {
	u8 se_if_id;
	u8 se_if_did;
	struct se_if_defines if_defs;
	u8 *se_name;
	u8 *pool_name;
	bool soc_register;
	bool reserved_dma_ranges;
	int (*se_fetch_soc_info)(struct se_if_priv *priv, u16 *soc_rev, u64 *serial_num);
};

struct se_if_node_info_list {
	const u8 num_mu;
	const u16 soc_id;
	struct se_fw_load_info load_hsm_fw;
	const struct se_if_node_info info[];
};

static u16 se_soc_rev;
static struct se_if_node_info_list imx8ulp_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX8ULP,
	.load_hsm_fw = {
		.prim_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2-ahab-container.img",
		.seco_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2ext-ahab-container.img",
		.is_fw_loaded = false,
		.handle_susp_resm = true,
		.imem = {
			.state = ELE_IMEM_STATE_OK,
		},
	},
	.info = {
			{
			.se_if_id = 0,
			.se_if_did = 7,
			.if_defs = {
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.se_name = SE_TYPE_HSM"1",
			.pool_name = "sram",
			.soc_register = true,
			.reserved_dma_ranges = true,
			.se_fetch_soc_info = ele_fetch_soc_info,
			},
	},
};

static struct se_if_node_info_list imx93_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX93,
	.load_hsm_fw = {
		.prim_fw_nm_in_rfs = NULL,
		.seco_fw_nm_in_rfs = NULL,
		.is_fw_loaded = true,
		.handle_susp_resm = false,
	},
	.info = {
			{
			.se_if_id = 2,
			.se_if_did = 3,
			.if_defs = {
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.se_name = SE_TYPE_HSM"1",
			.reserved_dma_ranges = true,
			.soc_register = true,
			},
	},
};

static const struct of_device_id se_match[] = {
	{ .compatible = "fsl,imx8ulp-se", .data = (void *)&imx8ulp_info},
	{ .compatible = "fsl,imx93-se", .data = (void *)&imx93_info},
	{},
};

static const struct se_if_node_info
	*get_se_if_node_info(const struct se_if_node_info_list *info_list,
			     const u32 idx)
{
	return &info_list->info[idx];
}

static int se_soc_info(struct se_if_priv *priv)
{
	const struct se_if_node_info *info;
	struct se_if_node_info_list *info_list;
	struct soc_device_attribute *attr;
	struct soc_device *sdev;
	u64 serial_num;
	int err = 0;

	info = container_of(priv->if_defs,
			typeof(*info),
			if_defs);
	info_list = container_of(info,
			typeof(*info_list),
			info[info->se_if_id]);

	/* This function should be called once.
	 * Check if the se_soc_rev is zero to continue.
	 */
	if (se_soc_rev)
		return err;

	if (info->se_fetch_soc_info) {
		err = info->se_fetch_soc_info(priv, &se_soc_rev, &serial_num);
		if (err < 0) {
			dev_err(priv->dev, "Failed to fetch SoC Info.");
			return err;
		}
	} else {
		dev_err(priv->dev, "Failed to fetch SoC revision.");
		if (info->soc_register)
			dev_err(priv->dev, "Failed to do SoC registration.");
		err = -EINVAL;
		return err;
	}

	if (!info->soc_register)
		return 0;

	attr = devm_kzalloc(priv->dev, sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	if (FIELD_GET(DEV_GETINFO_MIN_VER_MASK, se_soc_rev))
		attr->revision = devm_kasprintf(priv->dev, GFP_KERNEL, "%x.%x",
						FIELD_GET(DEV_GETINFO_MIN_VER_MASK,
							  se_soc_rev),
						FIELD_GET(DEV_GETINFO_MAJ_VER_MASK,
							  se_soc_rev));
	else
		attr->revision = devm_kasprintf(priv->dev, GFP_KERNEL, "%x",
						FIELD_GET(DEV_GETINFO_MAJ_VER_MASK,
							  se_soc_rev));

	switch (info_list->soc_id) {
	case SOC_ID_OF_IMX8ULP:
		attr->soc_id = devm_kasprintf(priv->dev, GFP_KERNEL,
					      "i.MX8ULP");
		break;
	case SOC_ID_OF_IMX93:
		attr->soc_id = devm_kasprintf(priv->dev, GFP_KERNEL,
					      "i.MX93");
		break;
	}

	err = of_property_read_string(of_root, "model",
				      &attr->machine);
	if (err)
		return -EINVAL;

	attr->family = devm_kasprintf(priv->dev, GFP_KERNEL, "Freescale i.MX");

	attr->serial_number
		= devm_kasprintf(priv->dev, GFP_KERNEL, "%016llX", serial_num);

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);

	return 0;
}

static struct se_fw_load_info *get_load_fw_instance(struct se_if_priv *priv)
{
	const struct se_if_node_info *info = container_of(priv->if_defs,
							typeof(*info),
							if_defs);
	struct se_if_node_info_list *info_list;
	struct se_fw_load_info *load_fw = NULL;

	info_list = container_of(info,
			typeof(*info_list),
			info[info->se_if_id]);

	if (!memcmp(SE_TYPE_HSM, info->se_name, strlen(SE_TYPE_HSM)))
		load_fw = &info_list->load_hsm_fw;
	else
		dev_err(priv->dev, "Invalid load fw configuration.");

	return load_fw;
}

static int se_load_firmware(struct se_if_priv *priv)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	const struct firmware *fw;
	phys_addr_t se_fw_phyaddr;
	const u8 *se_img_file_to_load;
	u8 *se_fw_buf;
	int ret;

	guard(mutex)(&load_fw->se_fw_load);
	if (load_fw->is_fw_loaded)
		return 0;

	se_img_file_to_load = load_fw->seco_fw_nm_in_rfs;
	if (load_fw->prim_fw_nm_in_rfs) {
		/* allocate buffer where SE store encrypted IMEM */
		load_fw->imem.buf = dmam_alloc_coherent(priv->dev, ELE_IMEM_SIZE,
							&load_fw->imem.phyaddr,
							GFP_KERNEL);
		if (!load_fw->imem.buf) {
			dev_err(priv->dev,
				"dmam-alloc-failed: To store encr-IMEM.\n");
			ret = -ENOMEM;
			goto exit;
		}
		if (load_fw->imem.state == ELE_IMEM_STATE_BAD)
			se_img_file_to_load
					= load_fw->prim_fw_nm_in_rfs;
	}

	do {
		ret = request_firmware(&fw, se_img_file_to_load, priv->dev);
		if (ret)
			goto exit;

		dev_info(priv->dev, "loading firmware %s\n", se_img_file_to_load);

		/* allocate buffer to store the SE FW */
		se_fw_buf = dma_alloc_coherent(priv->dev, fw->size,
				&se_fw_phyaddr, GFP_KERNEL);
		if (!se_fw_buf) {
			ret = -ENOMEM;
			goto exit;
		}

		memcpy(se_fw_buf, fw->data, fw->size);
		ret = ele_fw_authenticate(priv, se_fw_phyaddr);
		if (ret < 0) {
			dev_err(priv->dev,
					"Error %pe: Authenticate & load SE firmware %s.\n",
					ERR_PTR(ret),
					se_img_file_to_load);
			ret = -EPERM;
		}

		dma_free_coherent(priv->dev,
				  fw->size,
				  se_fw_buf,
				  se_fw_phyaddr);

		release_firmware(fw);

		if (!ret && load_fw->imem.state == ELE_IMEM_STATE_BAD &&
				se_img_file_to_load == load_fw->prim_fw_nm_in_rfs)
			se_img_file_to_load = load_fw->seco_fw_nm_in_rfs;
		else
			se_img_file_to_load = NULL;

	} while (se_img_file_to_load);

	if (!ret)
		load_fw->is_fw_loaded = true;

exit:
	return ret;
}

/* interface for managed res to free a mailbox channel */
static void if_mbox_free_channel(void *mbox_chan)
{
	mbox_free_channel(mbox_chan);
}

static int se_if_request_channel(struct device *dev,
				 struct mbox_chan **chan,
				 struct mbox_client *cl,
				 const char *name)
{
	struct mbox_chan *t_chan;
	int ret = 0;

	t_chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(t_chan)) {
		ret = PTR_ERR(t_chan);
		return dev_err_probe(dev, ret,
				     "Failed to request %s channel.", name);
	}

	ret = devm_add_action(dev, if_mbox_free_channel, t_chan);
	if (ret) {
		dev_err(dev, "failed to add devm removal of mbox %s\n", name);
		goto exit;
	}

	*chan = t_chan;

exit:
	return ret;
}

static void se_if_probe_cleanup(void *plat_dev)
{
	struct platform_device *pdev = plat_dev;
	struct device *dev = &pdev->dev;
	struct se_fw_load_info *load_fw;
	struct se_if_priv *priv;

	priv = dev_get_drvdata(dev);
	load_fw = get_load_fw_instance(priv);

	/* In se_if_request_channel(), passed the clean-up functional
	 * pointer reference as action to devm_add_action().
	 * No need to free the mbox channels here.
	 */

	/* free the buffer in se remove, previously allocated
	 * in se probe to store encrypted IMEM
	 */
	if (load_fw && load_fw->imem.buf) {
		dmam_free_coherent(dev,
				   ELE_IMEM_SIZE,
				   load_fw->imem.buf,
				   load_fw->imem.phyaddr);
		load_fw->imem.buf = NULL;
	}

	/* No need to check, if reserved memory is allocated
	 * before calling for its release. Or clearing the
	 * un-set bit.
	 */
	of_reserved_mem_device_release(dev);
}

static int se_if_probe(struct platform_device *pdev)
{
	const struct se_if_node_info_list *info_list;
	const struct se_if_node_info *info;
	struct device *dev = &pdev->dev;
	struct se_fw_load_info *load_fw;
	struct se_if_priv *priv;
	u32 idx;
	int ret;

	idx = GET_IDX_FROM_DEV_NODE_NAME(dev->of_node);
	info_list = device_get_match_data(dev);
	if (idx >= info_list->num_mu) {
		dev_err(dev,
			"Incorrect node name :%s\n",
			dev->of_node->full_name);
		dev_err(dev,
			"%s-<index>, acceptable index range is 0..%d\n",
			dev->of_node->name,
			info_list->num_mu - 1);
		ret = -EINVAL;
		return ret;
	}

	info = get_se_if_node_info(info_list, idx);
	if (!info) {
		ret = -EINVAL;
		goto exit;
	}


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto exit;
	}

	priv->dev = dev;
	priv->if_defs = &info->if_defs;
	dev_set_drvdata(dev, priv);

	ret = devm_add_action(dev, se_if_probe_cleanup, pdev);
	if (ret)
		goto exit;


	/* Mailbox client configuration */
	priv->se_mb_cl.dev		= dev;
	priv->se_mb_cl.tx_block		= false;
	priv->se_mb_cl.knows_txdone	= true;
	priv->se_mb_cl.rx_callback	= se_if_rx_callback;

	ret = se_if_request_channel(dev, &priv->tx_chan,
			&priv->se_mb_cl, MBOX_TX_NAME);
	if (ret)
		goto exit;

	ret = se_if_request_channel(dev, &priv->rx_chan,
			&priv->se_mb_cl, MBOX_RX_NAME);
	if (ret)
		goto exit;

	mutex_init(&priv->se_if_cmd_lock);

	init_completion(&priv->waiting_rsp_clbk_hdl.done);
	init_completion(&priv->cmd_receiver_clbk_hdl.done);

	if (info->pool_name) {
		priv->mem_pool = of_gen_pool_get(dev->of_node,
							 info->pool_name, 0);
		if (!priv->mem_pool) {
			dev_err(dev,
				"Unable to get sram pool = %s\n",
				info->pool_name);
			goto exit;
		}
	}

	if (info->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret) {
			dev_err(dev,
				"failed to init reserved memory region %d\n",
				ret);
			goto exit;
		}
	}

	ret = se_soc_info(priv);
	if (ret) {
		dev_err(dev,
			"failed[%pe] to fetch SoC Info\n", ERR_PTR(ret));
		goto exit;
	}

	load_fw = get_load_fw_instance(priv);
	/* By default, there is no pending FW to be loaded.*/
	if (load_fw->is_fw_loaded) {
		mutex_init(&load_fw->se_fw_load);
		ret = se_load_firmware(priv);
		if (ret)
			dev_warn(dev, "Failed to load firmware.");
		ret = 0;
	}
	dev_info(dev, "i.MX secure-enclave: %s interface to firmware, configured.\n",
		 info->se_name);
	return ret;

exit:
	/* if execution control reaches here, if probe fails.
	 */
	return dev_err_probe(dev, ret, "%s: Probe failed.", __func__);

	return ret;
}

static void se_if_remove(struct platform_device *pdev)
{
	se_if_probe_cleanup(pdev);
}

static int se_suspend(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	int ret = 0;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->handle_susp_resm) {
		ret = se_save_imem_state(priv, &load_fw->imem);
		if (ret < 0)
			goto exit;
		load_fw->imem.size = ret;
	}
exit:
	return ret;
}

static int se_resume(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->handle_susp_resm)
		se_restore_imem_state(priv, &load_fw->imem);

	return 0;
}

static const struct dev_pm_ops se_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(se_suspend, se_resume)
};

static struct platform_driver se_driver = {
	.driver = {
		.name = "fsl-se-fw",
		.of_match_table = se_match,
		.pm = &se_pm,
	},
	.probe = se_if_probe,
	.remove = se_if_remove,
};
MODULE_DEVICE_TABLE(of, se_match);

module_platform_driver(se_driver);
MODULE_AUTHOR("Pankaj Gupta <pankaj.gupta@nxp.com>");
MODULE_DESCRIPTION("iMX Secure Enclave Driver.");
MODULE_LICENSE("GPL");
