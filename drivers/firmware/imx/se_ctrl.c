// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-direct.h>
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

#define MAX_SOC_INFO_DATA_SZ		256
#define MBOX_TX_NAME			"tx"
#define MBOX_RX_NAME			"rx"

#define SE_TYPE_STR_DBG			"dbg"
#define SE_TYPE_STR_HSM			"hsm"

#define SE_TYPE_ID_DBG			0x1
#define SE_TYPE_ID_HSM			0x2

struct se_fw_img_name {
	const u8 *prim_fw_nm_in_rfs;
	const u8 *seco_fw_nm_in_rfs;
};

struct se_fw_load_info {
	const struct se_fw_img_name *se_fw_img_nm;
	bool is_fw_tobe_loaded;
	bool imem_mgmt;
	struct se_imem_buf imem;
};

struct se_var_info {
	u16 soc_rev;
	struct se_fw_load_info load_fw;
};

/* contains fixed information */
struct se_soc_info {
	const u16 soc_id;
	const bool soc_register;
	const struct se_fw_img_name se_fw_img_nm;
};

struct se_if_node {
	struct se_soc_info *se_info;
	u8 *pool_name;
	bool reserved_dma_ranges;
	struct se_if_defines if_defs;
};

/* common for all the SoC. */
static struct se_var_info var_se_info;

static struct se_soc_info se_imx8ulp_info = {
	.soc_id = SOC_ID_OF_IMX8ULP,
	.soc_register = true,
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2-ahab-container.img",
		.seco_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2ext-ahab-container.img",
	},
};

static struct se_if_node imx8ulp_se_ele_hsm = {
	.se_info = &se_imx8ulp_info,
	.pool_name = "sram",
	.reserved_dma_ranges = true,
	.if_defs = {
		.se_if_type = SE_TYPE_ID_HSM,
		.cmd_tag = 0x17,
		.rsp_tag = 0xe1,
		.success_tag = ELE_SUCCESS_IND,
		.base_api_ver = MESSAGING_VERSION_6,
		.fw_api_ver = MESSAGING_VERSION_7,
	},
};

static struct se_soc_info se_imx93_info = {
	.soc_id = SOC_ID_OF_IMX93,
};

static struct se_if_node imx93_se_ele_hsm = {
	.se_info = &se_imx93_info,
	.reserved_dma_ranges = true,
	.if_defs = {
		.se_if_type = SE_TYPE_ID_HSM,
		.cmd_tag = 0x17,
		.rsp_tag = 0xe1,
		.success_tag = ELE_SUCCESS_IND,
		.base_api_ver = MESSAGING_VERSION_6,
		.fw_api_ver = MESSAGING_VERSION_7,
	},
};

static const struct of_device_id se_match[] = {
	{ .compatible = "fsl,imx8ulp-se-ele-hsm", .data = &imx8ulp_se_ele_hsm},
	{ .compatible = "fsl,imx93-se-ele-hsm", .data = &imx93_se_ele_hsm},
	{},
};

char *get_se_if_name(u8 se_if_id)
{
	switch (se_if_id) {
	case SE_TYPE_ID_DBG: return SE_TYPE_STR_DBG;
	case SE_TYPE_ID_HSM: return SE_TYPE_STR_HSM;
	}

	return NULL;
}

static struct se_fw_load_info *get_load_fw_instance(struct se_if_priv *priv)
{
	return &var_se_info.load_fw;
}

static int get_se_soc_info(struct se_if_priv *priv, const struct se_soc_info *se_info)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	struct soc_device_attribute *attr;
	u8 data[MAX_SOC_INFO_DATA_SZ];
	struct ele_dev_info *s_info;
	struct soc_device *sdev;
	int err = 0;

	/*
	 * This function should be called once.
	 * Check if the se_soc_rev is zero to continue.
	 */
	if (var_se_info.soc_rev)
		return err;

	err = ele_fetch_soc_info(priv, &data);
	if (err < 0)
		return dev_err_probe(priv->dev, err, "Failed to fetch SoC Info.");
	s_info = (void *)data;
	var_se_info.soc_rev = s_info->d_info.soc_rev;
	load_fw->imem.state = s_info->d_addn_info.imem_state;

	if (!se_info->soc_register)
		return 0;

	attr = devm_kzalloc(priv->dev, sizeof(*attr), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	if (FIELD_GET(DEV_GETINFO_MIN_VER_MASK, var_se_info.soc_rev))
		attr->revision = devm_kasprintf(priv->dev, GFP_KERNEL, "%x.%x",
						FIELD_GET(DEV_GETINFO_MIN_VER_MASK,
							  var_se_info.soc_rev),
						FIELD_GET(DEV_GETINFO_MAJ_VER_MASK,
							  var_se_info.soc_rev));
	else
		attr->revision = devm_kasprintf(priv->dev, GFP_KERNEL, "%x",
						FIELD_GET(DEV_GETINFO_MAJ_VER_MASK,
							  var_se_info.soc_rev));

	switch (se_info->soc_id) {
	case SOC_ID_OF_IMX8ULP:
		attr->soc_id = "i.MX8ULP";
		break;
	case SOC_ID_OF_IMX93:
		attr->soc_id = "i.MX93";
		break;
	}

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err)
		return -EINVAL;

	attr->family = "Freescale i.MX";

	attr->serial_number = devm_kasprintf(priv->dev,
					     GFP_KERNEL, "%016llX",
					     GET_SERIAL_NUM_FROM_UID(s_info->d_info.uid,
								     MAX_UID_SIZE >> 2));

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);

	return 0;
}

static int init_misc_device_context(struct se_if_priv *priv, int ch_id,
				    struct se_if_device_ctx **new_dev_ctx)
{
	struct se_if_device_ctx *dev_ctx;
	int ret = 0;

	dev_ctx = devm_kzalloc(priv->dev, sizeof(*dev_ctx), GFP_KERNEL);

	if (!dev_ctx)
		return -ENOMEM;

	dev_ctx->devname = devm_kasprintf(priv->dev, GFP_KERNEL, "%s0_ch%d",
					  get_se_if_name(priv->if_defs->se_if_type),
					  ch_id);
	if (!dev_ctx->devname)
		return -ENOMEM;

	dev_ctx->priv = priv;
	*new_dev_ctx = dev_ctx;

	return ret;
}

/* interface for managed res to free a mailbox channel */
static void if_mbox_free_channel(void *mbox_chan)
{
	mbox_free_channel(mbox_chan);
}

static int se_if_request_channel(struct device *dev, struct mbox_chan **chan,
				 struct mbox_client *cl, const char *name)
{
	struct mbox_chan *t_chan;
	int ret = 0;

	t_chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(t_chan))
		return dev_err_probe(dev, PTR_ERR(t_chan),
				     "Failed to request %s channel.", name);

	ret = devm_add_action_or_reset(dev, if_mbox_free_channel, t_chan);
	if (ret)
		return dev_err_probe(dev, -EPERM,
				     "Failed to add-action for removal of mbox: %s\n",
				     name);
	*chan = t_chan;

	return ret;
}

static void se_if_probe_cleanup(void *plat_dev)
{
	struct platform_device *pdev = plat_dev;
	struct se_fw_load_info *load_fw;
	struct device *dev = &pdev->dev;
	struct se_if_priv *priv;

	priv = dev_get_drvdata(dev);
	load_fw = get_load_fw_instance(priv);

	/*
	 * In se_if_request_channel(), passed the clean-up functional
	 * pointer reference as action to devm_add_action_or_reset().
	 * No need to free the mbox channels here.
	 */

	/*
	 * free the buffer in se remove, previously allocated
	 * in se probe to store encrypted IMEM
	 */
	if (load_fw && load_fw->imem.buf) {
		dmam_free_coherent(dev, ELE_IMEM_SIZE, load_fw->imem.buf,
				   load_fw->imem.phyaddr);
		load_fw->imem.buf = NULL;
	}

	/*
	 * No need to check, if reserved memory is allocated
	 * before calling for its release. Or clearing the
	 * un-set bit.
	 */
	of_reserved_mem_device_release(dev);
}

static int se_if_probe(struct platform_device *pdev)
{
	const struct se_soc_info *se_info;
	const struct se_if_node *if_node;
	struct se_fw_load_info *load_fw;
	struct device *dev = &pdev->dev;
	struct se_if_priv *priv;
	dma_addr_t imem_dma_addr;
	int ret;

	if_node = device_get_match_data(dev);
	if (!if_node)
		return -EINVAL;

	se_info = if_node->se_info;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->if_defs = &if_node->if_defs;
	dev_set_drvdata(dev, priv);

	ret = devm_add_action_or_reset(dev, se_if_probe_cleanup, pdev);
	if (ret)
		return ret;

	/* Mailbox client configuration */
	priv->se_mb_cl.dev		= dev;
	priv->se_mb_cl.tx_block		= false;
	priv->se_mb_cl.knows_txdone	= true;
	priv->se_mb_cl.rx_callback	= se_if_rx_callback;

	ret = se_if_request_channel(dev, &priv->tx_chan, &priv->se_mb_cl, MBOX_TX_NAME);
	if (ret)
		return ret;

	ret = se_if_request_channel(dev, &priv->rx_chan, &priv->se_mb_cl, MBOX_RX_NAME);
	if (ret)
		return ret;

	mutex_init(&priv->se_if_cmd_lock);

	init_completion(&priv->waiting_rsp_clbk_hdl.done);
	init_completion(&priv->cmd_receiver_clbk_hdl.done);

	if (if_node->pool_name) {
		priv->mem_pool = of_gen_pool_get(dev->of_node, if_node->pool_name, 0);
		if (!priv->mem_pool)
			return dev_err_probe(dev, -ENOMEM,
					     "Unable to get sram pool = %s.",
					     if_node->pool_name);
	}

	if (if_node->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret,
					    "Failed to init reserved memory region.");
	}

	ret = init_misc_device_context(priv, 0, &priv->priv_dev_ctx);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed[0x%x] to create device contexts.",
				     ret);

	if (if_node->if_defs.se_if_type == SE_TYPE_ID_HSM) {
		ret = get_se_soc_info(priv, se_info);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to fetch SoC Info.");
	}

	/* By default, there is no pending FW to be loaded.*/
	if (se_info->se_fw_img_nm.seco_fw_nm_in_rfs) {
		load_fw = get_load_fw_instance(priv);
		load_fw->se_fw_img_nm = &se_info->se_fw_img_nm;
		load_fw->is_fw_tobe_loaded = true;

		if (load_fw->se_fw_img_nm->prim_fw_nm_in_rfs) {
			/* allocate buffer where SE store encrypted IMEM */
			imem_dma_addr = phys_to_dma(priv->dev, load_fw->imem.phyaddr);
			load_fw->imem.buf = dmam_alloc_coherent(priv->dev, ELE_IMEM_SIZE,
								&imem_dma_addr, GFP_KERNEL);
			if (!load_fw->imem.buf)
				return dev_err_probe(dev, -ENOMEM,
						     "dmam-alloc-failed: To store encr-IMEM.");
			load_fw->imem_mgmt = true;
		}
	}
	dev_info(dev, "i.MX secure-enclave: %s0 interface to firmware, configured.",
		 get_se_if_name(priv->if_defs->se_if_type));

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int se_suspend(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	int ret = 0;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		ret = se_save_imem_state(priv, &load_fw->imem);
		if (ret)
			dev_err(dev, "Failure saving IMEM state[0x%x]", ret);
	}

	return 0;
}

static int se_resume(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	int ret = 0;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		se_restore_imem_state(priv, &load_fw->imem);
		if (ret)
			dev_err(dev, "Failure restoring IMEM state[0x%x]", ret);
	}

	return 0;
}

static const struct dev_pm_ops se_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(se_suspend, se_resume)
};

#define SE_PM_OPS	(&se_pm)
#else
#define SE_PM_OPS	NULL
#endif

static struct platform_driver se_driver = {
	.driver = {
		.name = "fsl-se",
		.of_match_table = se_match,
		.pm = SE_PM_OPS,
	},
	.probe = se_if_probe,
};
MODULE_DEVICE_TABLE(of, se_match);

module_platform_driver(se_driver);
MODULE_AUTHOR("Pankaj Gupta <pankaj.gupta@nxp.com>");
MODULE_DESCRIPTION("iMX Secure Enclave Driver.");
MODULE_LICENSE("GPL");
