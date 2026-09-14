// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
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
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "se_ctrl.h"

#define MAX_SOC_INFO_DATA_SZ		256
#define SE_TYPE_STR_DBG			"dbg"
#define SE_TYPE_STR_HSM			"hsm"

#define SE_TYPE_ID_DBG			0x1

#define SE_TYPE_ID_HSM			0x2

struct se_soc_dev_regn {
	bool soc_dev_registered;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;
};

struct se_var_info {
	u16 soc_rev;
	struct se_soc_dev_regn soc_dev_regn;
	/* To serialize populating common SoC level info. */
	struct mutex se_var_info_lock;
};

/* contains fixed information */
struct se_soc_info {
	const u16 soc_id;
	const char *soc_name;
	const struct se_fw_img_name se_fw_img_nm;
	bool imem_state_mgmt;
};

struct se_if_node {
	struct se_soc_info *se_info;
	u8 *pool_name;
	bool reserved_dma_ranges;
	struct se_if_defines if_defs;
};

/* common for all the SoC. */
static struct se_var_info var_se_info = {
	.soc_rev = 0,
	.se_var_info_lock = __MUTEX_INITIALIZER(var_se_info.se_var_info_lock)
};

static struct se_soc_info se_imx8ulp_info = {
	.soc_id = SOC_ID_OF_IMX8ULP,
	.soc_name = "i.MX8ULP",
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2-ahab-container.img",
		.seco_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2ext-ahab-container.img",
	},
	.imem_state_mgmt = true,
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
	{ .compatible = "fsl,imx8ulp-se-ele-hsm", .data = &imx8ulp_se_ele_hsm },
	{ .compatible = "fsl,imx93-se-ele-hsm", .data = &imx93_se_ele_hsm },
	{ }
};
MODULE_DEVICE_TABLE(of, se_match);

/**
 * get_se_if_name() - return a human-readable string for a SE interface type.
 * @se_if_id: SE interface type identifier (e.g. SE_TYPE_ID_HSM).
 *
 * Return: pointer to a constant string naming the interface type, or "unknown"
 * if @se_if_id does not match any known type.
 */
char *get_se_if_name(u8 se_if_id)
{
	switch (se_if_id) {
	case SE_TYPE_ID_DBG: return SE_TYPE_STR_DBG;
	case SE_TYPE_ID_HSM: return SE_TYPE_STR_HSM;
	}

	return "unknown";
}

static struct se_fw_load_info *get_load_fw_instance(struct se_if_priv *priv)
{
	return &priv->load_fw;
}

static void se_soc_device_unregister(struct se_soc_dev_regn *soc_dev_regn)
{
	guard(mutex)(&var_se_info.se_var_info_lock);

	if (soc_dev_regn->soc_dev) {
		soc_device_unregister(soc_dev_regn->soc_dev);
		soc_dev_regn->soc_dev = NULL;
	}

	if (soc_dev_regn->soc_dev_attr) {
		/*
		 * revision and serial_number are the only kasprintf()-allocated
		 * strings. machine points into the DT, and soc_id/family are
		 * constants, so they must not be freed.
		 */
		kfree(soc_dev_regn->soc_dev_attr->revision);
		kfree(soc_dev_regn->soc_dev_attr->serial_number);
		kfree(soc_dev_regn->soc_dev_attr);
		soc_dev_regn->soc_dev_attr = NULL;
	}

	soc_dev_regn->soc_dev_registered = false;
}

/*
 * Build and register a soc_device entry for this SoC. Separated from
 * get_se_soc_info() so that the firmware-fetch path and the sysfs
 * registration path can be reasoned about independently.
 */
static int se_soc_dev_register(struct se_if_priv *priv, u16 soc_rev,
			       const char *soc_name, const u8 *uid)
{
	struct soc_device_attribute *attr;
	struct soc_device *sdev;
	int err;

	if (!soc_rev || !soc_name || !uid)
		return -EINVAL;

	attr = kzalloc_obj(*attr);
	if (!attr)
		return -ENOMEM;

	if (FIELD_GET(DEV_GETINFO_MIN_VER_MASK, soc_rev))
		attr->revision = kasprintf(GFP_KERNEL, "%x.%x",
					   FIELD_GET(DEV_GETINFO_MAJ_VER_MASK, soc_rev),
					   FIELD_GET(DEV_GETINFO_MIN_VER_MASK, soc_rev));
	else
		attr->revision = kasprintf(GFP_KERNEL, "%x",
					   FIELD_GET(DEV_GETINFO_MAJ_VER_MASK, soc_rev));

	if (!attr->revision) {
		err = -ENOMEM;
		goto err_free_attr;
	}

	attr->soc_id = soc_name;

	err = of_property_read_string(of_root, "model", &attr->machine);
	if (err) {
		err = -EINVAL;
		goto err_free_rev;
	}

	attr->family = "Freescale i.MX";

	attr->serial_number = kasprintf(GFP_KERNEL, "%016llX",
					GET_SERIAL_NUM_FROM_UID(uid, MAX_UID_SIZE >> 2));
	if (!attr->serial_number) {
		err = -ENOMEM;
		goto err_free_rev;
	}

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev)) {
		err = PTR_ERR(sdev);
		goto err_free_serial;
	}

	/*
	 * Publish the singleton. Freed once, at module unload, by
	 * se_soc_device_unregister(). Caller holds se_var_info_lock.
	 */
	var_se_info.soc_dev_regn.soc_dev = sdev;
	var_se_info.soc_dev_regn.soc_dev_attr = attr;

	/* Mark registration complete so get_se_soc_info() skips this path on retry. */
	var_se_info.soc_dev_regn.soc_dev_registered = true;

	return 0;

err_free_serial:
	kfree(attr->serial_number);
err_free_rev:
	kfree(attr->revision);
err_free_attr:
	kfree(attr);

	return err;
}

static int get_se_soc_info(struct se_if_priv *priv, const struct se_soc_info *se_info)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	u8 data[MAX_SOC_INFO_DATA_SZ];
	struct ele_dev_info *s_info;
	int err;

	guard(mutex)(&var_se_info.se_var_info_lock);

	/*
	 * Early exit: both objectives already complete, nothing to do.
	 * Do not exit early when imem_mgmt is active: load_fw is per-probe
	 * (embedded in priv) and starts zeroed on every probe, so imem.state
	 * must be refreshed from firmware on each probe even when soc_rev is
	 * already cached in the module-lifetime var_se_info.
	 */
	if (var_se_info.soc_rev &&
	    (!se_info->soc_name || var_se_info.soc_dev_regn.soc_dev_registered) &&
	    !load_fw->imem_mgmt)
		return 0;

	err = ele_fetch_soc_info(priv, &data);
	if (err < 0)
		return dev_err_probe(priv->dev, err, "Failed to fetch SoC Info.\n");

	s_info = (struct ele_dev_info *)data;

	if (!var_se_info.soc_rev)
		var_se_info.soc_rev = s_info->d_info.soc_rev;

	/*
	 * imem.state is per-probe state (lives in priv->load_fw which is
	 * zeroed on every probe). Update it unconditionally whenever the
	 * IMEM management path is active, regardless of whether soc_rev was
	 * already cached from a previous probe or a sibling interface.
	 */
	if (load_fw->imem_mgmt)
		load_fw->imem.state = s_info->d_addn_info.imem_state;

	if (se_info->soc_name && !var_se_info.soc_dev_regn.soc_dev_registered) {
		err = se_soc_dev_register(priv, var_se_info.soc_rev,
					  se_info->soc_name, s_info->d_info.uid);
		if (err < 0)
			return dev_err_probe(priv->dev, err,
					     "Failed to register SE SoC device.\n");
	}

	return 0;
}

static int se_if_request_channel(struct device *dev, struct mbox_chan **chan,
				 struct mbox_client *cl, const char *name)
{
	struct mbox_chan *t_chan;

	t_chan = mbox_request_channel_byname(cl, name);
	if (IS_ERR(t_chan))
		return dev_err_probe(dev, PTR_ERR(t_chan),
				     "Failed to request %s channel.\n", name);

	*chan = t_chan;

	return 0;
}

static void se_if_probe_cleanup(void *plat_dev)
{
	struct platform_device *pdev = plat_dev;
	struct device *dev = &pdev->dev;
	struct se_if_priv *priv;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return;

	if (priv->rx_chan)
		mbox_free_channel(priv->rx_chan);
	if (priv->tx_chan)
		mbox_free_channel(priv->tx_chan);

	/*
	 * Being device managed buffer, no need to free the buffer allocated
	 * in se probe to store encrypted IMEM.
	 */

	/*
	 * No need to check, if reserved memory is allocated
	 * before calling for its release. Or clearing the
	 * un-set bit.
	 */
	of_reserved_mem_device_release(dev);

	dev_set_drvdata(dev, NULL);
	mutex_destroy(&priv->load_fw.load_fw_lock);
	mutex_destroy(&priv->se_if_cmd_lock);
	kfree(priv);
}

static int se_if_probe(struct platform_device *pdev)
{
	const struct se_soc_info *se_info;
	const struct se_if_node *if_node;
	struct device *dev = &pdev->dev;
	struct se_fw_load_info *load_fw;
	struct se_if_priv *priv;
	int ret;

	if_node = device_get_match_data(dev);
	if (!if_node)
		return -EINVAL;

	se_info = if_node->se_info;

	priv = kzalloc_obj(*priv);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->if_defs = &if_node->if_defs;
	dev_set_drvdata(dev, priv);

	spin_lock_init(&priv->cmd_receiver_clbk_hdl.clbk_rx_lock);
	spin_lock_init(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock);
	atomic_set(&priv->fw_busy, 0);
	init_completion(&priv->waiting_rsp_clbk_hdl.done);
	init_completion(&priv->cmd_receiver_clbk_hdl.done);

	mutex_init(&priv->se_if_cmd_lock);

	load_fw = get_load_fw_instance(priv);
	mutex_init(&load_fw->load_fw_lock);
	if (se_info->se_fw_img_nm.seco_fw_nm_in_rfs) {
		load_fw->se_fw_img_nm = &se_info->se_fw_img_nm;
		load_fw->is_fw_tobe_loaded = true;
	}
	ret = devm_add_action_or_reset(dev, se_if_probe_cleanup, pdev);
	if (ret)
		return ret;

	/* Mailbox client configuration */
	priv->se_mb_cl.dev		= dev;
	priv->se_mb_cl.tx_block		= false;
	priv->se_mb_cl.knows_txdone	= false;
	priv->se_mb_cl.rx_callback	= se_if_rx_callback;

	ret = se_if_request_channel(dev, &priv->tx_chan, &priv->se_mb_cl, "tx");
	if (ret)
		return ret;

	ret = se_if_request_channel(dev, &priv->rx_chan, &priv->se_mb_cl, "rx");
	if (ret)
		return ret;

	if (if_node->pool_name) {
		priv->mem_pool = of_gen_pool_get(dev->of_node, if_node->pool_name, 0);
		if (!priv->mem_pool)
			return dev_err_probe(dev, -ENOMEM,
					     "Unable to get sram pool = %s.\n",
					     if_node->pool_name);
	}

	if (if_node->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to init reserved memory region.\n");
	}

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));

	/* By default, there is no pending FW to be loaded.*/
	if (se_info->imem_state_mgmt) {
		/* allocate buffer where SE store encrypted IMEM */
		load_fw->imem.buf = dmam_alloc_coherent(priv->dev, ELE_IMEM_SIZE,
							&load_fw->imem.daddr,
							GFP_KERNEL);
		if (!load_fw->imem.buf)
			return dev_err_probe(dev, -ENOMEM,
					     "dmam-alloc-failed: To store encr-IMEM.\n");
		load_fw->imem_mgmt = true;
	}

	if (if_node->if_defs.se_if_type == SE_TYPE_ID_HSM) {
		ret = get_se_soc_info(priv, se_info);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to fetch SoC Info.\n");
	}

	dev_info(dev, "i.MX secure-enclave: %s0 interface to firmware, configured.\n",
		 get_se_if_name(priv->if_defs->se_if_type));

	return ret;
}

static int se_suspend(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	unsigned int noio_flag;
	int ret = 0;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		/*
		 * Set PF_MEMALLOC_NOIO for the duration of the suspend
		 * callbacks. This covers all allocations in the call chain
		 * (ele_get_info, se_service_swap, se_get_mem_pool_buf) without
		 * requiring each site to pass GFP_NOIO explicitly. Without this,
		 * GFP_KERNEL allocations in those paths could trigger direct
		 * reclaim and attempt I/O to a storage device that is already
		 * suspended, causing a deadlock.
		 */
		noio_flag = memalloc_noio_save();
		ret = se_save_imem_state(priv, &load_fw->imem);
		memalloc_noio_restore(noio_flag);
		if (ret)
			dev_err(dev, "Failure saving IMEM state[0x%x]\n", ret);
	}

	return ret;
}

static int se_resume(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	unsigned int noio_flag;
	int ret = 0;

	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		noio_flag = memalloc_noio_save();
		ret = se_restore_imem_state(priv, &load_fw->imem);
		memalloc_noio_restore(noio_flag);
		if (ret)
			dev_err(dev, "Failure restoring IMEM state[0x%x]\n", ret);
	}

	return ret;
}

DEFINE_SIMPLE_DEV_PM_OPS(se_pm, se_suspend, se_resume);

static struct platform_driver se_driver = {
	.driver = {
		.name = "fsl-se",
		.of_match_table = se_match,
		.pm = pm_sleep_ptr(&se_pm),
	},
	.probe = se_if_probe,
};

static int __init se_init(void)
{
	return platform_driver_register(&se_driver);
}
module_init(se_init);

static void __exit se_exit(void)
{
	platform_driver_unregister(&se_driver);

	/*
	 * The soc_device is a module-scoped singleton that outlives any single
	 * MU interface bind/unbind. Release it here, once, after every interface
	 * has been unbound, so its lifetime is tied to the module rather than to
	 * the first-probed interface.
	 */
	se_soc_device_unregister(&var_se_info.soc_dev_regn);
}
module_exit(se_exit);

MODULE_AUTHOR("Pankaj Gupta <pankaj.gupta@nxp.com>");
MODULE_DESCRIPTION("iMX Secure Enclave Driver.");
MODULE_LICENSE("GPL");
