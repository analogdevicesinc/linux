// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024-2025 NXP
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/fs_struct.h>
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
#include <uapi/linux/se_ioctl.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "ele_fw_api.h"
#include "ele_trng.h"
#include "se_ctrl.h"
#include "seco_init.h"
#include "v2x_base_msg.h"
#include "v2x_common.h"

#define MAX_SOC_INFO_DATA_SZ		256
#define MBOX_TX_NAME			"tx"
#define MBOX_RX_NAME			"rx"
#define MBOX_TXDB_NAME			"txdb"
#define MBOX_RXDB_NAME			"rxdb"

#define IMX_SE_LOG_PATH "/var/lib/se_"
#define SE_RCV_MSG_DEFAULT_TIMEOUT	5000
#define SE_RCV_MSG_LONG_TIMEOUT		5000000

static int se_log;
static struct kobject *se_kobj;
u32 se_rcv_msg_timeout = SE_RCV_MSG_DEFAULT_TIMEOUT;

struct se_fw_img_name {
	const u8 *prim_fw_nm_in_rfs;
	const u8 *seco_fw_nm_in_rfs;
};

struct se_fw_load_info {
	const struct se_fw_img_name *se_fw_img_nm;
	bool is_fw_loaded;
	bool imem_mgmt;
	struct se_imem_buf imem;
};

struct se_if_node_info {
	u8 se_if_id;
	u8 se_if_did;
	struct se_if_defines if_defs;
	u8 *pool_name;
	uint32_t mu_buff_size;
	bool reserved_dma_ranges;
	int (*start_rng)(struct se_if_priv *priv);
	int (*init_trng)(struct se_if_priv *priv);
	int (*se_if_early_init)(struct se_if_priv *priv);
	int (*se_if_late_init)(struct se_if_priv *priv);
};

/* contains fixed information */
struct se_if_node_info_list {
	const u8 num_mu;
	const u16 soc_id;
	bool soc_register;
	int (*se_fetch_soc_info)(struct se_if_priv *priv, void *data);
	const struct se_fw_img_name se_fw_img_nm;
	const struct se_if_node_info info[];
};

struct se_var_info {
	uint8_t board_type;
	u16 soc_id;
	u16 soc_rev;
	struct se_fw_load_info load_fw;
};

static struct se_var_info var_se_info = {
	.board_type = 0,
	.soc_id = 0,
	.soc_rev = 0,
	.load_fw = {
		.is_fw_loaded = true,
		.imem_mgmt = false,
	},
};

static LIST_HEAD(priv_data_list);

static struct se_if_node_info_list imx8ulp_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX8ULP,
	.soc_register = true,
	.se_fetch_soc_info = ele_fetch_soc_info,
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2-ahab-container.img",
		.seco_fw_nm_in_rfs = IMX_ELE_FW_DIR
			"mx8ulpa2ext-ahab-container.img",
	},
	.info = {
			{
			.se_if_id = 0,
			.se_if_did = 7,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_HSM,
				.se_instance_id = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.pool_name = "sram",
			.reserved_dma_ranges = true,
			},
	},
};

static struct se_if_node_info_list imx93_info = {
	.num_mu = 1,
	.soc_id = SOC_ID_OF_IMX93,
	.soc_register = false,
	.se_fetch_soc_info = ele_fetch_soc_info,
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = NULL,
		.seco_fw_nm_in_rfs = NULL,
	},
	.info = {
			{
			.se_if_id = 0,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_HSM,
				.se_instance_id = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = true,
			.start_rng = ele_start_rng,
			.init_trng = ele_trng_init,
			.se_if_early_init = NULL,
			.se_if_late_init = ele_init_fw,
			},
	},
};

static struct se_if_node_info_list imx95_info = {
	.num_mu = 4,
	.soc_id = SOC_ID_OF_IMX95,
	.soc_register = false,
	.se_fetch_soc_info = ele_fetch_soc_info,
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = NULL,
		.seco_fw_nm_in_rfs = NULL,
	},
	.info = {
			{
			.se_if_id = 0,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_HSM,
				.se_instance_id = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = ele_start_rng,
			.init_trng = ele_trng_init,
			.se_if_early_init = NULL,
			.se_if_late_init = v2x_late_init,
			},
			{
			.se_if_id = 1,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_DBG,
				.se_instance_id = 0,
				.cmd_tag = 0x1a,
				.rsp_tag = 0xe4,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_2,
			},
			.reserved_dma_ranges = false,
			.start_rng = v2x_start_rng,
			.init_trng = NULL,
			.se_if_early_init = v2x_early_init,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 2,
			.mu_buff_size = 16,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SV,
				.se_instance_id = 0,
				.cmd_tag = 0x18,
				.rsp_tag = 0xe2,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_2,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = v2x_early_init,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 3,
			.mu_buff_size = 256,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SHE,
				.se_instance_id = 0,
				.cmd_tag = 0x1a,
				.rsp_tag = 0xe4,
				.success_tag = ELE_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_2,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = v2x_early_init,
			.se_if_late_init = NULL,
			},
	},
};

static struct se_if_node_info_list imx8dxl_info = {
	.num_mu = 7,
	.soc_id = SOC_ID_OF_IMX8DXL,
	.soc_register = false,
	.se_fetch_soc_info = seco_fetch_soc_info,
	.se_fw_img_nm = {
		.prim_fw_nm_in_rfs = NULL,
		.seco_fw_nm_in_rfs = NULL,
	},
	.info = {
			{
			.se_if_id = 0,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_SHE,
				.se_instance_id = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 1,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_HSM,
				.se_instance_id = 0,
				.cmd_tag = 0x17,
				.rsp_tag = 0xe1,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_6,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 2,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SV,
				.se_instance_id = 0,
				.cmd_tag = 0x18,
				.rsp_tag = 0xe2,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 3,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SV,
				.se_instance_id = 1,
				.cmd_tag = 0x19,
				.rsp_tag = 0xe3,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 4,
			.mu_buff_size = 16,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SHE,
				.se_instance_id = 0,
				.cmd_tag = 0x1a,
				.rsp_tag = 0xe4,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 5,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SG,
				.se_instance_id = 0,
				.cmd_tag = 0x1d,
				.rsp_tag = 0xe7,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
			{
			.se_if_id = 6,
			.mu_buff_size = 0,
			.if_defs = {
				.se_if_type = SE_TYPE_ID_V2X_SG,
				.se_instance_id = 1,
				.cmd_tag = 0x1e,
				.rsp_tag = 0xe8,
				.success_tag = SECO_SUCCESS_IND,
				.base_api_ver = MESSAGING_VERSION_2,
				.fw_api_ver = MESSAGING_VERSION_7,
			},
			.reserved_dma_ranges = false,
			.start_rng = NULL,
			.init_trng = NULL,
			.se_if_early_init = imx_scu_init_fw,
			.se_if_late_init = NULL,
			},
	},
};

static const struct of_device_id se_match[] = {
	{ .compatible = "fsl,imx8ulp-se", .data = (void *)&imx8ulp_info},
	{ .compatible = "fsl,imx93-se", .data = (void *)&imx93_info},
	{ .compatible = "fsl,imx95-se", .data = (void *)&imx95_info},
	{ .compatible = "fsl,imx8dxl-se", .data = (void *)&imx8dxl_info},
	{},
};

char *get_se_if_name(u8 se_if_id)
{
	switch (se_if_id) {
	case SE_TYPE_ID_DBG: return SE_TYPE_STR_DBG;
	case SE_TYPE_ID_HSM: return SE_TYPE_STR_HSM;
	case SE_TYPE_ID_SHE: return SE_TYPE_STR_SHE;
	case SE_TYPE_ID_V2X_DBG: return SE_TYPE_STR_V2X_DBG;
	case SE_TYPE_ID_V2X_SHE: return SE_TYPE_STR_V2X_SHE;
	case SE_TYPE_ID_V2X_SV: return SE_TYPE_STR_V2X_SV;
	case SE_TYPE_ID_V2X_SG: return SE_TYPE_STR_V2X_SG;
	}

	return NULL;
}

/*
 * Writing the sysfs entry for -
 *
 * se_log: to enable/disable the SE logging
 * echo 1 > /sys/kernel/se/se_log // enable logging
 * echo 0 > /sys/kernel/se/se_log // disable logging
 *
 * se_rcv_msg_timeout: to change the rcv_msg timeout value in jiffies for
 *                     the operations that take more time.
 * echo 180000 > /sys/kernel/se/se_rcv_msg_timeout
 */
static ssize_t se_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int var, ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0) {
		pr_err("Failed to convert to int\n");
		return ret;
	}

	if (strcmp(attr->attr.name, "se_log") == 0)
		se_log = var;
	else
		se_rcv_msg_timeout = var;

	return count;
}

/*
 * Reading the sysfs entry for -
 *
 * se_log: to know SE logging is enabled/disabled
 *
 * se_rcv_msg_timeout: to read the current rcv_msg timeout value in jiffies
 */
static ssize_t se_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	int var = 0;

	if (strcmp(attr->attr.name, "se_log") == 0)
		var = se_log;
	else
		var = se_rcv_msg_timeout;

	return sysfs_emit(buf, "%d\n", var);
}

struct kobj_attribute se_log_attr = __ATTR(se_log, 0664,
					   se_show,
					   se_store);

struct kobj_attribute se_rcv_msg_timeout_attr = __ATTR(se_rcv_msg_timeout,
						       0664, se_show,
						       se_store);

/* Exposing the variable se_log via sysfs to enable/disable logging */
static int  se_sysfs_log(void)
{
	int ret = 0;

	/* Create kobject "se" located under /sys/kernel */
	se_kobj = kobject_create_and_add("se", kernel_kobj);
	if (!se_kobj) {
		pr_warn("kobject creation failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Create file for the se_log attribute */
	if (sysfs_create_file(se_kobj, &se_log_attr.attr)) {
		pr_err("Failed to create se file\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Create file for the se_rcv_msg_timeout attribute */
	if (sysfs_create_file(se_kobj, &se_rcv_msg_timeout_attr.attr)) {
		pr_err("Failed to create se_rcv_msg_timeout file\n");
		ret = -ENOMEM;
		goto out;
	}

out:
	if (ret)
		kobject_put(se_kobj);
	return ret;
}

/*
 * get_se_soc_id() - to fetch the soc_id of the platform
 *
 * @priv  : reference to the private data per SE MU interface.
 *
 * This function returns the SoC ID.
 *
 * Context: Other module, requiring to access the secure services based on SoC Id.
 *
 * Return: SoC Id of the device.
 */
uint32_t get_se_soc_id(struct se_if_priv *priv)
{
	const struct se_if_node_info_list *info_list
			= device_get_match_data(priv->dev);

	if (var_se_info.soc_rev)
		return var_se_info.soc_id;
	else
		return info_list->soc_id;

}

void *imx_get_se_data_info(uint32_t soc_id, u32 idx)
{
	const struct se_if_node_info_list *info_list;
	struct se_if_priv *priv;

	switch (soc_id) {
	case SOC_ID_OF_IMX8ULP:
		info_list = &imx8ulp_info; break;
	case SOC_ID_OF_IMX8DXL:
	case SOC_ID_OF_IMX8QXP:
		info_list = &imx8dxl_info; break;
	case SOC_ID_OF_IMX93:
		info_list = &imx93_info; break;
	case SOC_ID_OF_IMX95:
		info_list = &imx95_info; break;
	default:
		return NULL;
	}

	if (idx >= info_list->num_mu) {
		pr_err("%s-<index>, acceptable index range is 0..%d\n",
			NODE_NAME,
			info_list->num_mu - 1);
		return NULL;
	}

	list_for_each_entry(priv, &priv_data_list, priv_data) {
		if (priv->if_defs == &info_list->info[idx].if_defs)
			return (void *)priv;
	}
	pr_err("No matching index found for soc_id = %d.", soc_id);

	return NULL;
}
EXPORT_SYMBOL_GPL(imx_get_se_data_info);

static struct se_fw_load_info *get_load_fw_instance(struct se_if_priv *priv)
{
	return &var_se_info.load_fw;
}

static int se_soc_info(struct se_if_priv *priv)
{
	const struct se_if_node_info_list *info_list
			= device_get_match_data(priv->dev);
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	struct soc_device_attribute *attr;
	struct ele_dev_info *s_info;
	struct soc_device *sdev;
	u8 data[MAX_SOC_INFO_DATA_SZ];
	int err = 0;

	/* This function should be called once.
	 * Check if the se_soc_rev is zero to continue.
	 */
	if (var_se_info.soc_rev)
		return err;

	if (info_list->se_fetch_soc_info) {
		err = info_list->se_fetch_soc_info(priv, &data);
		if (err < 0) {
			dev_err(priv->dev, "Failed to fetch SoC Info.");
			return err;
		}
		if (info_list->soc_id == SOC_ID_OF_IMX8DXL) {
			struct seco_soc_info *soc_data = (void *)data;

			var_se_info.board_type = soc_data->board_type;
			var_se_info.soc_id = soc_data->soc_id;
			var_se_info.soc_rev = soc_data->soc_rev;
		} else {
			s_info = (void *)data;

			var_se_info.board_type = 0;
			var_se_info.soc_id = info_list->soc_id;
			var_se_info.soc_rev = s_info->d_info.soc_rev;
			load_fw->imem.state = s_info->d_addn_info.imem_state;
		}
	} else {
		dev_err(priv->dev, "Failed to fetch SoC revision.");
		if (info_list->soc_register)
			dev_err(priv->dev, "Failed to do SoC registration.");
		err = -EINVAL;
		return err;
	}

	if (!info_list->soc_register)
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
		= devm_kasprintf(priv->dev, GFP_KERNEL, "%016llX",
				 GET_SERIAL_NUM_FROM_UID(s_info->d_info.uid, MAX_UID_SIZE >> 2));

	sdev = soc_device_register(attr);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);

	return 0;
}

static int se_load_firmware(struct se_if_priv *priv)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	const struct firmware *fw;
	phys_addr_t se_fw_phyaddr;
	const u8 *se_img_file_to_load;
	u8 *se_fw_buf;
	int ret;

	if (load_fw->is_fw_loaded)
		return 0;

	se_img_file_to_load = load_fw->se_fw_img_nm->seco_fw_nm_in_rfs;
	if (load_fw->imem.state == ELE_IMEM_STATE_BAD &&
			load_fw->se_fw_img_nm->prim_fw_nm_in_rfs)
		se_img_file_to_load = load_fw->se_fw_img_nm->prim_fw_nm_in_rfs;

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
		fw = NULL;

		if (!ret && load_fw->imem.state == ELE_IMEM_STATE_BAD &&
				se_img_file_to_load == load_fw->se_fw_img_nm->prim_fw_nm_in_rfs)
			se_img_file_to_load = load_fw->se_fw_img_nm->seco_fw_nm_in_rfs;
		else
			se_img_file_to_load = NULL;

	} while (se_img_file_to_load);

	if (!ret)
		load_fw->is_fw_loaded = true;

exit:
	return ret;
}

#define NANO_SEC_PRN_LEN	9
#define SEC_PRN_LEN		5

int se_dump_to_logfl(struct se_if_device_ctx *dev_ctx,
		     u8 caller_type, int buf_size,
		     const char *buf, ...)
{
	struct se_lg_fl_info *lg_fl_info = &dev_ctx->priv->lg_fl_info;
	u8 fmt_str[256] = "[%lld.%ld]: %s: %s: %s";
	const u8 *devname = dev_ctx->devname;
	int fmt_str_idx = strlen(fmt_str);
	const u8 *caller_type_str;
	u8 dump_ln[512] = {'\0'};
	u8 loc_buf[256] = {'\0'};
	u8 file_name[128] = {'\0'};
	struct timespec64 log_tm;
	bool is_hex = true;
	int dump_ln_len;
	ssize_t wret;
	int w_ct;
	va_list args;

	/* if logging is set to be disabled, return */
	if (!se_log)
		return 0;

	switch (caller_type) {
	case SE_DUMP_IOCTL_BUFS:
		caller_type_str = "_IOCTL";
		break;
	case SE_DUMP_MU_SND_BUFS:
		caller_type_str = "MU_SND";
		break;
	case SE_DUMP_MU_RCV_BUFS:
		caller_type_str = "MU_RCV";
		break;
	default:

		is_hex = false;
		caller_type_str = "SE_DBG";
		va_start(args, buf);
		buf_size = vsprintf(loc_buf, buf, args);
		va_end(args);
	}

	if (is_hex) {
		for (w_ct = 0; w_ct < buf_size >> 2; w_ct++) {
			fmt_str[fmt_str_idx] = '%';
			fmt_str_idx++;
			fmt_str[fmt_str_idx] = '0';
			fmt_str_idx++;
			fmt_str[fmt_str_idx] = '8';
			fmt_str_idx++;
			fmt_str[fmt_str_idx] = 'x';
			fmt_str_idx++;
			fmt_str[fmt_str_idx] = ' ';
			fmt_str_idx++;
		}
	}

	fmt_str[fmt_str_idx] = '\n';

	ktime_get_ts64(&log_tm);
	if (!lg_fl_info->lg_file) {
		task_lock(&init_task);
		get_fs_root(init_task.fs, &lg_fl_info->root);
		task_unlock(&init_task);

		sprintf(file_name, "%s%s_%d.%lld_%ld",
			IMX_SE_LOG_PATH,
			get_se_if_name(dev_ctx->priv->if_defs->se_if_type),
			dev_ctx->priv->if_defs->se_instance_id,
			log_tm.tv_sec,
			log_tm.tv_nsec);

		lg_fl_info->lg_file = file_open_root(&lg_fl_info->root,
						file_name,
						O_CREAT | O_WRONLY | O_SYNC,
						0);
		path_put(&lg_fl_info->root);
		if (IS_ERR(lg_fl_info->lg_file)) {
			dev_err(dev_ctx->priv->dev, "open file %s failed[%ld]\n",
				file_name, PTR_ERR(lg_fl_info->lg_file));

			wret = PTR_ERR(lg_fl_info->lg_file);
			lg_fl_info->lg_file = NULL;
			return wret;
		}
	}
	dump_ln_len = SEC_PRN_LEN + NANO_SEC_PRN_LEN +
			fmt_str_idx + strlen(devname) +
			strlen(caller_type_str) + buf_size;
	snprintf(dump_ln, dump_ln_len, fmt_str,
					log_tm.tv_sec,
					log_tm.tv_nsec,
					devname,
					caller_type_str,
					loc_buf,
					((uint32_t *)buf)[0], ((uint32_t *)buf)[1],
					((uint32_t *)buf)[2], ((uint32_t *)buf)[3],
					((uint32_t *)buf)[4], ((uint32_t *)buf)[5],
					((uint32_t *)buf)[6], ((uint32_t *)buf)[7],
					((uint32_t *)buf)[8], ((uint32_t *)buf)[9],
					((uint32_t *)buf)[10], ((uint32_t *)buf)[11],
					((uint32_t *)buf)[12], ((uint32_t *)buf)[13],
					((uint32_t *)buf)[14], ((uint32_t *)buf)[15]);

	wret = kernel_write(lg_fl_info->lg_file,
				dump_ln, dump_ln_len,
				&lg_fl_info->offset);
	if (wret < 0) {
		dev_err(dev_ctx->priv->dev,
			"Error writing log file: %s.\n",
			file_name);
	} else if (wret != dump_ln_len) {
		dev_err(dev_ctx->priv->dev,
			"Wrote only %ld bytes of %d writing to log file %s\n",
			wret, dump_ln_len, file_name);
	}

	return 0;
}

static int init_se_shared_mem(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;

	INIT_LIST_HEAD(&se_shared_mem_mgmt->pending_out);
	INIT_LIST_HEAD(&se_shared_mem_mgmt->pending_in);

	/*
	 * Allocate some memory for data exchanges with S40x.
	 * This will be used for data not requiring secure memory.
	 */
	se_shared_mem_mgmt->non_secure_mem.ptr
			= dma_alloc_coherent(priv->dev,
					     MAX_DATA_SIZE_PER_USER,
					     &se_shared_mem_mgmt->non_secure_mem.dma_addr,
					     GFP_KERNEL);
	if (!se_shared_mem_mgmt->non_secure_mem.ptr)
		return -ENOMEM;

	if (priv->flags & SCU_MEM_CFG) {
		if (imx_scu_mem_access(dev_ctx)) {
			dev_err(dev_ctx->priv->dev,
				"%s: Failed to share access to shared memory\n",
				dev_ctx->devname);
			return -EPERM;
		}
	}

	se_shared_mem_mgmt->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	se_shared_mem_mgmt->non_secure_mem.pos = 0;

	return 0;
}

static void cleanup_se_shared_mem(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;

	/* Unmap secure memory shared buffer. */
	if (se_shared_mem_mgmt->secure_mem.ptr)
		devm_iounmap(priv->dev,
				(void __iomem *)se_shared_mem_mgmt->secure_mem.ptr);

	se_shared_mem_mgmt->secure_mem.ptr = NULL;
	se_shared_mem_mgmt->secure_mem.dma_addr = 0;
	se_shared_mem_mgmt->secure_mem.size = 0;
	se_shared_mem_mgmt->secure_mem.pos = 0;

	/* Free non-secure shared buffer. */
	dma_free_coherent(priv->dev, MAX_DATA_SIZE_PER_USER,
			  se_shared_mem_mgmt->non_secure_mem.ptr,
			  se_shared_mem_mgmt->non_secure_mem.dma_addr);

	se_shared_mem_mgmt->non_secure_mem.ptr = NULL;
	se_shared_mem_mgmt->non_secure_mem.dma_addr = 0;
	se_shared_mem_mgmt->non_secure_mem.size = 0;
	se_shared_mem_mgmt->non_secure_mem.pos = 0;
}

/* Need to copy the output data to user-device context.
 */
static int se_dev_ctx_cpy_out_data(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_buf_desc *b_desc, *temp;
	bool do_cpy = true;

	list_for_each_entry_safe(b_desc, temp, &se_shared_mem_mgmt->pending_out, link) {
		if (b_desc->usr_buf_ptr && b_desc->shared_buf_ptr && do_cpy) {

			dev_dbg(priv->dev,
				"Copying output data to user.");
			if (do_cpy && copy_to_user(b_desc->usr_buf_ptr,
					 b_desc->shared_buf_ptr,
					 b_desc->size)) {
				dev_err(priv->dev,
					"Failure copying output data to user.");
				do_cpy = false;
			}
		}

		if (b_desc->shared_buf_ptr) {
			if (dev_ctx->priv->mu_mem.pos)
				memset_io(b_desc->shared_buf_ptr, 0, b_desc->size);
			else
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);
		}

		list_del(&b_desc->link);
		kfree(b_desc);
	}

	return do_cpy ? 0 : -EFAULT;
}

/*
 * Clean the used Shared Memory space,
 * whether its Input Data copied from user buffers, or
 * Data received from FW.
 */
static void se_dev_ctx_shared_mem_cleanup(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct list_head *pending_lists[] = {&se_shared_mem_mgmt->pending_in,
						&se_shared_mem_mgmt->pending_out};
	struct se_buf_desc *b_desc, *temp;
	int i;

	for (i = 0; i < 2; i++) {
		list_for_each_entry_safe(b_desc, temp,
					 pending_lists[i], link) {

			if (b_desc->shared_buf_ptr) {
				if (dev_ctx->priv->mu_mem.pos)
					memset_io(b_desc->shared_buf_ptr, 0, b_desc->size);
				else
					memset(b_desc->shared_buf_ptr, 0, b_desc->size);
			}

			list_del(&b_desc->link);
			kfree(b_desc);
		}
	}
	se_shared_mem_mgmt->secure_mem.pos = 0;
	se_shared_mem_mgmt->non_secure_mem.pos = 0;
}

static int add_b_desc_to_pending_list(void *shared_ptr_with_pos,
			       struct se_ioctl_setup_iobuf *io,
			       struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_buf_desc *b_desc = NULL;

	b_desc = kzalloc(sizeof(*b_desc), GFP_KERNEL);
	if (!b_desc)
		return -ENOMEM;

	b_desc->shared_buf_ptr = shared_ptr_with_pos;
	b_desc->usr_buf_ptr = io->user_buf;
	b_desc->size = io->length;

	if (io->flags & SE_IO_BUF_FLAGS_IS_INPUT) {
		/*
		 * buffer is input:
		 * add an entry in the "pending input buffers" list so
		 * that copied data can be cleaned from shared memory
		 * later.
		 */
		list_add_tail(&b_desc->link, &se_shared_mem_mgmt->pending_in);
	} else {
		/*
		 * buffer is output:
		 * add an entry in the "pending out buffers" list so data
		 * can be copied to user space when receiving Secure-Enclave
		 * response.
		 */
		list_add_tail(&b_desc->link, &se_shared_mem_mgmt->pending_out);
	}

	return 0;
}

/* interface for managed res to unregister a character device */
static void if_misc_deregister(void *miscdevice)
{
	misc_deregister(miscdevice);
}

static int init_device_context(struct se_if_priv *priv, int ch_id,
			struct se_if_device_ctx **new_dev_ctx,
			const struct file_operations *se_if_fops)
{
	struct se_if_device_ctx *dev_ctx;
	int ret = 0;

	if (ch_id)
		dev_ctx = kzalloc(sizeof(*dev_ctx), GFP_KERNEL);
	else
		dev_ctx = devm_kzalloc(priv->dev, sizeof(*dev_ctx), GFP_KERNEL);

	if (!dev_ctx) {
		ret = -ENOMEM;
		return ret;
	}

	dev_ctx->priv = priv;

	if (ch_id)
		dev_ctx->devname = kasprintf(GFP_KERNEL, "%s%d_ch%d",
					     get_se_if_name(priv->if_defs->se_if_type),
					     priv->if_defs->se_instance_id,
					     ch_id);
	else
		dev_ctx->devname = devm_kasprintf(priv->dev, GFP_KERNEL, "%s%d_ch%d",
					     get_se_if_name(priv->if_defs->se_if_type),
					     priv->if_defs->se_instance_id,
					     ch_id);
	if (!dev_ctx->devname) {
		ret = -ENOMEM;
		if (ch_id)
			kfree(dev_ctx);

		return ret;
	}

	mutex_init(&dev_ctx->fops_lock);

	*new_dev_ctx = dev_ctx;

	if (ch_id) {
		list_add_tail(&dev_ctx->link, &priv->dev_ctx_list);
		priv->active_devctx_count++;

		ret = init_se_shared_mem(dev_ctx);
		if (ret < 0) {
			kfree(dev_ctx->devname);
			kfree(dev_ctx);
			*new_dev_ctx = NULL;
			return ret;
		}

		return ret;
	}

	/* Only for ch_id = 0:
	 * - register the misc device.
	 * - add action
	 */
	dev_ctx->miscdev = devm_kzalloc(priv->dev, sizeof(*dev_ctx->miscdev), GFP_KERNEL);
	if (!dev_ctx->miscdev) {
		ret = -ENOMEM;
		*new_dev_ctx = NULL;
		return ret;
	}

	dev_ctx->miscdev->name = dev_ctx->devname;
	dev_ctx->miscdev->minor = MISC_DYNAMIC_MINOR;
	dev_ctx->miscdev->fops = se_if_fops;
	dev_ctx->miscdev->parent = priv->dev;
	ret = misc_register(dev_ctx->miscdev);
	if (ret) {
		dev_err(priv->dev, "failed to register misc device %d\n",
			ret);
		return ret;
	}

	ret = devm_add_action(priv->dev, if_misc_deregister,
			      dev_ctx->miscdev);
	if (ret) {
		dev_err(priv->dev,
			"failed[%d] to add action to the misc-dev\n",
			ret);
		misc_deregister(dev_ctx->miscdev);
	}

	return ret;
}

static int se_ioctl_cmd_snd_rcv_rsp_handler(struct se_if_device_ctx *dev_ctx,
					    u64 arg)
{
	struct se_ioctl_cmd_snd_rcv_rsp_info cmd_snd_rcv_rsp_info = {0};
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_api_msg *rx_msg __free(kfree) = NULL;
	int err = 0;

	if (copy_from_user(&cmd_snd_rcv_rsp_info, (u8 __user *)arg,
			   sizeof(cmd_snd_rcv_rsp_info))) {
		dev_err(priv->dev,
			"%s: Failed to copy cmd_snd_rcv_rsp_info from user\n",
			dev_ctx->devname);
		err = -EFAULT;
		goto exit;
	}

	if (cmd_snd_rcv_rsp_info.tx_buf_sz < SE_MU_HDR_SZ) {
		dev_err(priv->dev,
			"%s: User buffer too small(%d < %d)\n",
			dev_ctx->devname,
			cmd_snd_rcv_rsp_info.tx_buf_sz,
			SE_MU_HDR_SZ);
		err = -ENOSPC;
		goto exit;
	}

	rx_msg = kzalloc(cmd_snd_rcv_rsp_info.rx_buf_sz, GFP_KERNEL);
	if (!rx_msg) {
		err = -ENOMEM;
		goto exit;
	}

	tx_msg = memdup_user(cmd_snd_rcv_rsp_info.tx_buf,
			     cmd_snd_rcv_rsp_info.tx_buf_sz);
	if (IS_ERR(tx_msg)) {
		err = PTR_ERR(tx_msg);
		goto exit;
	}

	if (tx_msg->header.tag != priv->if_defs->cmd_tag) {
		err = -EINVAL;
		goto exit;
	}

	if (tx_msg->header.ver == priv->if_defs->fw_api_ver &&
		!get_load_fw_instance(priv)->is_fw_loaded) {
		err = se_load_firmware(priv);
		if (err) {
			dev_err(priv->dev, "Could not send the message as FW is not loaded.");
			err = -EPERM;
			goto exit;
		}
	}

	se_rcv_msg_timeout =
		(se_rcv_msg_timeout == SE_RCV_MSG_DEFAULT_TIMEOUT) ? SE_RCV_MSG_LONG_TIMEOUT
								   : se_rcv_msg_timeout;

	err = ele_msg_send_rcv(dev_ctx,
			       tx_msg,
			       cmd_snd_rcv_rsp_info.tx_buf_sz,
			       rx_msg,
			       cmd_snd_rcv_rsp_info.rx_buf_sz);
	if (err < 0)
		goto exit;

	dev_dbg(priv->dev,
		"%s: %s %s\n",
		dev_ctx->devname,
		__func__,
		"message received, start transmit to user");

	/* We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	err = se_dev_ctx_cpy_out_data(dev_ctx);
	if (err < 0)
		goto exit;

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     rx_msg,
			     cmd_snd_rcv_rsp_info.rx_buf_sz, false);

	if (copy_to_user(cmd_snd_rcv_rsp_info.rx_buf, rx_msg,
			 cmd_snd_rcv_rsp_info.rx_buf_sz)) {
		dev_err(priv->dev,
			"%s: Failed to copy to user\n",
			dev_ctx->devname);
		err = -EFAULT;
	}

exit:
	se_dev_ctx_shared_mem_cleanup(dev_ctx);
	priv->mu_mem.pos = 0;

	if (copy_to_user((void __user *)arg, &cmd_snd_rcv_rsp_info,
			 sizeof(cmd_snd_rcv_rsp_info))) {
		dev_err(priv->dev,
			"%s: Failed to copy cmd_snd_rcv_rsp_info from user\n",
			dev_ctx->devname);
		err = -EFAULT;
	}

	return err;
}

static int se_ioctl_get_mu_info(struct se_if_device_ctx *dev_ctx,
				u64 arg)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_if_node_info *info;
	struct se_ioctl_get_if_info if_info;
	int err = 0;

	info = container_of(priv->if_defs, typeof(*info), if_defs);

	if_info.se_if_id = 0;
	if_info.interrupt_idx = 0;
	if_info.tz = 0;
	if (get_se_soc_id(priv) == SOC_ID_OF_IMX8DXL ||
		get_se_soc_id(priv) == SOC_ID_OF_IMX8QXP) {
		if_info.se_if_id = info->se_if_id + 1;
		if (priv->if_defs->se_if_type > SE_TYPE_ID_SHE)
			if_info.se_if_id++;
	}

	if_info.did = info->se_if_did;
	if_info.cmd_tag = priv->if_defs->cmd_tag;
	if_info.rsp_tag = priv->if_defs->rsp_tag;
	if_info.success_tag = priv->if_defs->success_tag;
	if_info.base_api_ver = priv->if_defs->base_api_ver;
	if_info.fw_api_ver = priv->if_defs->fw_api_ver;

	dev_dbg(priv->dev,
		"%s: info [se_if_id: %d, irq_idx: %d, tz: 0x%x, did: 0x%x]\n",
			dev_ctx->devname,
			if_info.se_if_id, if_info.interrupt_idx,
			if_info.tz, if_info.did);

	if (copy_to_user((u8 __user *)arg, &if_info, sizeof(if_info))) {
		dev_err(priv->dev,
			"%s: Failed to copy mu info to user\n",
			dev_ctx->devname);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/*
 * Copy a buffer of data to/from the user and return the address to use in
 * messages
 */
static int se_ioctl_setup_iobuf_handler(struct se_if_device_ctx *dev_ctx,
					u64 arg)
{
	struct se_shared_mem *shared_mem = NULL;
	struct se_ioctl_setup_iobuf io = {0};
	int err = 0;
	u32 pos;

	if (copy_from_user(&io, (u8 __user *)arg, sizeof(io))) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed copy iobuf config from user\n",
			dev_ctx->devname);
		err = -EFAULT;
		goto exit;
	}

	dev_dbg(dev_ctx->priv->dev,
		"%s: io [buf: %p(%d) flag: %x]\n",
		dev_ctx->devname,
		io.user_buf, io.length, io.flags);

	if (io.length == 0 || !io.user_buf) {
		/*
		 * Accept NULL pointers since some buffers are optional
		 * in FW commands. In this case we should return 0 as
		 * pointer to be embedded into the message.
		 * Skip all data copy part of code below.
		 */
		io.ele_addr = 0;
		goto copy;
	}

	/* Select the shared memory to be used for this buffer. */
	if (io.flags & SE_IO_BUF_FLAGS_USE_MU_BUF)
		shared_mem = &dev_ctx->priv->mu_mem;
	else {
		if ((io.flags & SE_IO_BUF_FLAGS_USE_SEC_MEM) &&
				(dev_ctx->priv->flags & SCU_MEM_CFG)) {
			/* App requires to use secure memory for this buffer.*/
			shared_mem = &dev_ctx->se_shared_mem_mgmt.secure_mem;
		} else {
			/* No specific requirement for this buffer. */
			shared_mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;
		}
	}

	/* Check there is enough space in the shared memory. */
	dev_dbg(dev_ctx->priv->dev,
		"%s: req_size = %d, max_size= %d, curr_pos = %d",
		dev_ctx->devname,
		round_up(io.length, 8u),
		shared_mem->size, shared_mem->pos);

	if (shared_mem->size < shared_mem->pos ||
		round_up(io.length, 8u) > (shared_mem->size - shared_mem->pos)) {
		dev_err(dev_ctx->priv->dev,
			"%s: Not enough space in shared memory\n",
			dev_ctx->devname);
		err = -ENOMEM;
		goto exit;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;
	shared_mem->pos += round_up(io.length, 8u);
	io.ele_addr = (u64)shared_mem->dma_addr + pos;

	if (dev_ctx->priv->flags & SCU_MEM_CFG) {
		if ((io.flags & SE_IO_BUF_FLAGS_USE_SEC_MEM) &&
				!(io.flags & SE_IO_BUF_FLAGS_USE_SHORT_ADDR)) {
			/*Add base address to get full address.#TODO: Add API*/
			io.ele_addr += SECURE_RAM_BASE_ADDRESS_SCU;
		}
	}

	if (dev_ctx->priv->mu_mem.pos)
		memset_io(shared_mem->ptr + pos, 0, io.length);
	else
		memset(shared_mem->ptr + pos, 0, io.length);

	if ((io.flags & SE_IO_BUF_FLAGS_IS_INPUT) ||
	    (io.flags & SE_IO_BUF_FLAGS_IS_IN_OUT)) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		if (copy_from_user(shared_mem->ptr + pos, io.user_buf,
				   io.length)) {
			dev_err(dev_ctx->priv->dev,
				"%s: Failed copy data to shared memory\n",
				dev_ctx->devname);
			err = -EFAULT;
			goto exit;
		}
	}

	err = add_b_desc_to_pending_list(shared_mem->ptr + pos,
					 &io,
					 dev_ctx);
	if (err < 0)
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to allocate/link b_desc.",
			dev_ctx->devname);

copy:
	/* Provide the EdgeLock Enclave address to user space only if success.*/
	if (copy_to_user((u8 __user *)arg, &io, sizeof(io))) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy iobuff setup to user.",
			dev_ctx->devname);
		err = -EFAULT;
		goto exit;
	}
exit:
	return err;
}

/* IOCTL to provide SoC information */
static int se_ioctl_get_se_soc_info_handler(struct se_if_device_ctx *dev_ctx,
					     u64 arg)
{
	struct se_ioctl_get_soc_info soc_info;
	int err = -EINVAL;

	soc_info.soc_id = var_se_info.soc_id;
	soc_info.soc_rev = var_se_info.soc_rev;
	soc_info.board_type = var_se_info.board_type;

	err = (int)copy_to_user((u8 __user *)arg, (u8 *)(&soc_info), sizeof(soc_info));
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy soc info to user\n",
			dev_ctx->devname);
		err = -EFAULT;
		goto exit;
	}

exit:
	return err;
}

/* Configure the shared memory according to user config */
static int se_ioctl_shared_mem_cfg_handler(struct file *fp,
					   struct se_if_device_ctx *dev_ctx,
					   unsigned long arg)
{
	struct se_ioctl_shared_mem_cfg cfg;
	int err = -EINVAL;

	/* Check if not already configured. */
	if (dev_ctx->se_shared_mem_mgmt.secure_mem.dma_addr != 0u) {
		dev_err(dev_ctx->priv->dev, "Shared memory not configured\n");
		return err;
	}

	err = (int)copy_from_user(&cfg, (u8 *)arg, sizeof(cfg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Fail copy memory config\n");
		err = -EFAULT;
		return err;
	}

	dev_dbg(dev_ctx->priv->dev, "cfg offset: %u(%d)\n", cfg.base_offset, cfg.size);

	err = imx_scu_sec_mem_cfg(fp, cfg.base_offset, cfg.size);
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failt to map memory\n");
		err = -ENOMEM;
		return err;
	}

	return err;
}

static int se_ioctl_signed_msg_handler(struct file *fp,
				       struct se_if_device_ctx *dev_ctx,
				       unsigned long arg)
{
	struct se_ioctl_signed_message msg;
	int err;

	err = copy_from_user(&msg, (u8 *)arg, sizeof(msg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failed to copy from user: %d\n", err);
		return -EFAULT;
	}

	err = imx_scu_signed_msg(fp, msg.message, msg.msg_size, &msg.error_code);
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"Failed to send signed message: %d\n",
			err);
		return err;
	}

	err = copy_to_user((u8 *)arg, &msg, sizeof(msg));
	if (err) {
		dev_err(dev_ctx->priv->dev, "Failed to copy to user: %d\n", err);
		return -EFAULT;
	}

	return err;
}

/* IOCTL to provide request and response timestamps from FW for a crypto
 * operation
 */
static int se_ioctl_get_time(struct se_if_device_ctx *dev_ctx, unsigned long arg)
{
	struct se_if_priv *priv = dev_ctx->priv;
	int err = -EINVAL;
	struct se_time_frame time_frame;

	if (!priv) {
		err = -EINVAL;
		goto exit;
	}

	time_frame.t_start = priv->time_frame.t_start;
	time_frame.t_end = priv->time_frame.t_end;
	err = (int)copy_to_user((u8 *)arg, (u8 *)(&time_frame), sizeof(time_frame));
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"%s: Failed to copy timer to user\n",
			dev_ctx->devname);
		err  = -EFAULT;
		goto exit;
	}
exit:
	return err;
}


/*
 * File operations for user-space
 */

/* Write a message to the MU. */
static ssize_t se_if_fops_write(struct file *fp, const char __user *buf,
				size_t size, loff_t *ppos)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_api_msg *tx_msg __free(kfree) = NULL;
	struct se_if_priv *priv = dev_ctx->priv;
	int err;

	dev_dbg(priv->dev,
		"%s: write from buf (%p)%zu, ppos=%lld\n",
		dev_ctx->devname,
		buf, size, ((ppos) ? *ppos : 0));

	if (mutex_lock_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx) {
		err = -EINVAL;
		goto exit;
	}

	if (size < SE_MU_HDR_SZ) {
		dev_err(priv->dev,
			"%s: User buffer too small(%zu < %d)\n",
			dev_ctx->devname,
			size, SE_MU_HDR_SZ);
		err = -ENOSPC;
		goto exit;
	}

	tx_msg = memdup_user(buf, size);
	if (IS_ERR(tx_msg)) {
		err = PTR_ERR(tx_msg);
		goto exit;
	}

	print_hex_dump_debug("from user ", DUMP_PREFIX_OFFSET, 4, 4,
			     tx_msg, size, false);

	err = ele_msg_send(dev_ctx, tx_msg, size);
	if (err < 0)
		goto exit;
exit:
	mutex_unlock(&dev_ctx->fops_lock);
	return err;
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t se_if_fops_read(struct file *fp, char __user *buf,
			       size_t size, loff_t *ppos)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	int err;

	dev_dbg(priv->dev,
		"%s: read to buf %p(%zu), ppos=%lld\n",
		dev_ctx->devname,
		buf, size, ((ppos) ? *ppos : 0));

	if (mutex_lock_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx) {
		err = -EINVAL;
		goto exit;
	}

	err = ele_msg_rcv(dev_ctx, &priv->cmd_receiver_clbk_hdl);
	if (err < 0) {
		dev_err(priv->dev,
			"%s: Err[0x%x]:Interrupted by signal.\n",
			dev_ctx->devname, err);
		dev_dbg(priv->dev,
			"Current active dev-ctx count = %d.\n",
			dev_ctx->priv->active_devctx_count);
		goto exit;
	}

	/* We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	err = se_dev_ctx_cpy_out_data(dev_ctx);
	if (err < 0)
		goto exit;

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     priv->cmd_receiver_clbk_hdl.rx_msg,
			     priv->cmd_receiver_clbk_hdl.rx_msg_sz,
			     false);

	if (copy_to_user(buf, priv->cmd_receiver_clbk_hdl.rx_msg,
			 priv->cmd_receiver_clbk_hdl.rx_msg_sz)) {
		dev_err(priv->dev,
			"%s: Failed to copy to user\n",
			dev_ctx->devname);
		err = -EFAULT;
	}
	err = priv->cmd_receiver_clbk_hdl.rx_msg_sz;
exit:
	priv->cmd_receiver_clbk_hdl.rx_msg_sz = 0;

	se_dev_ctx_shared_mem_cleanup(dev_ctx);

	mutex_unlock(&dev_ctx->fops_lock);
	return err;
}

/* Open a character device. */
static int se_if_fops_open(struct inode *nd, struct file *fp)
{
	struct miscdevice *miscdev = fp->private_data;
	struct se_if_priv *priv = dev_get_drvdata(miscdev->parent);
	struct se_if_device_ctx *misc_dev_ctx = priv->priv_dev_ctx;
	struct se_if_device_ctx *dev_ctx;
	int err = 0;

	if (mutex_lock_interruptible(&misc_dev_ctx->fops_lock))
		return -EBUSY;

	priv->dev_ctx_mono_count++;
	err = init_device_context(priv,
				  priv->dev_ctx_mono_count ?
					priv->dev_ctx_mono_count
					: priv->dev_ctx_mono_count++,
				  &dev_ctx, NULL);
	if (err) {
		dev_err(priv->dev,
			"Failed[0x%x] to create device contexts.\n",
			err);
		goto exit;
	}
	se_dump_to_logfl(dev_ctx,
			 SE_DUMP_KDEBUG_BUFS, 0,
			 "IOCTL: %s", __func__);

	fp->private_data = dev_ctx;

exit:
	mutex_unlock(&misc_dev_ctx->fops_lock);
	return err;
}

/* Close a character device. */
static int se_if_fops_close(struct inode *nd, struct file *fp)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;

	if (mutex_lock_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	/* check if this device was registered as command receiver. */
	if (priv->cmd_receiver_clbk_hdl.dev_ctx == dev_ctx) {
		priv->cmd_receiver_clbk_hdl.dev_ctx = NULL;
		kfree(priv->cmd_receiver_clbk_hdl.rx_msg);
		priv->cmd_receiver_clbk_hdl.rx_msg = NULL;
	}

	se_dev_ctx_shared_mem_cleanup(dev_ctx);
	cleanup_se_shared_mem(dev_ctx);

	priv->active_devctx_count--;
	list_del(&dev_ctx->link);

	mutex_unlock(&dev_ctx->fops_lock);
	se_dump_to_logfl(dev_ctx,
			 SE_DUMP_KDEBUG_BUFS, 0,
			 "IOCTL: %s", __func__);
	kfree(dev_ctx->devname);
	kfree(dev_ctx);

	return 0;
}

/* IOCTL entry point of a character device */
static long se_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	int err = -EINVAL;

	/* Prevent race during change of device context */
	if (mutex_lock_interruptible(&dev_ctx->fops_lock))
		return -EBUSY;

	switch (cmd) {
	case SE_IOCTL_ENABLE_CMD_RCV:
		if (!priv->cmd_receiver_clbk_hdl.dev_ctx) {
			if (!priv->cmd_receiver_clbk_hdl.rx_msg) {
				priv->cmd_receiver_clbk_hdl.rx_msg
					= kzalloc(MAX_NVM_MSG_LEN,
						  GFP_KERNEL);
				if (!priv->cmd_receiver_clbk_hdl.rx_msg) {
					err = -ENOMEM;
					break;
				}
			}
			priv->cmd_receiver_clbk_hdl.rx_msg_sz = MAX_NVM_MSG_LEN;
			priv->cmd_receiver_clbk_hdl.dev_ctx = dev_ctx;
			se_dump_to_logfl(dev_ctx,
					SE_DUMP_KDEBUG_BUFS, 0,
					"IOCTL: %s", "SE_IOCTL_ENABLE_CMD_RCV");
			err = 0;
		} else {
			err = -EBUSY;
		}
		break;
	case SE_IOCTL_GET_MU_INFO:
		err = se_ioctl_get_mu_info(dev_ctx, arg);
		break;
	case SE_IOCTL_SETUP_IOBUF:
		err = se_ioctl_setup_iobuf_handler(dev_ctx, arg);
		break;
	case SE_IOCTL_GET_SOC_INFO:
		err = se_ioctl_get_se_soc_info_handler(dev_ctx, arg);
		break;
	case SE_IOCTL_CMD_SEND_RCV_RSP:
		err = se_ioctl_cmd_snd_rcv_rsp_handler(dev_ctx, arg);
		break;
	case SE_IOCTL_SHARED_BUF_CFG:
		if (priv->flags & SCU_MEM_CFG)
			err = se_ioctl_shared_mem_cfg_handler(fp, dev_ctx, arg);
		else
			err = -EPERM;
		break;
	case SE_IOCTL_SIGNED_MESSAGE:
		if (priv->flags & SCU_SIGNED_MSG_CFG)
			err = se_ioctl_signed_msg_handler(fp, dev_ctx, arg);
		else
			err = -EPERM;
		break;
	case SE_IOCTL_GET_TIMER:
		err = se_ioctl_get_time(dev_ctx, arg);
		break;
	default:
		err = -EINVAL;
		dev_dbg(priv->dev,
			"%s: IOCTL %.8x not supported\n",
			dev_ctx->devname,
			cmd);
	}

	mutex_unlock(&dev_ctx->fops_lock);

	return (long)err;
}

/* Char driver setup */
static const struct file_operations se_if_fops = {
	.open		= se_if_fops_open,
	.owner		= THIS_MODULE,
	.release	= se_if_fops_close,
	.unlocked_ioctl = se_ioctl,
	.read		= se_if_fops_read,
	.write		= se_if_fops_write,
};

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
	struct se_if_device_ctx *dev_ctx, *t_dev_ctx;
	struct platform_device *pdev = plat_dev;
	struct device *dev = &pdev->dev;
	struct se_fw_load_info *load_fw;
	struct se_if_priv *priv;
	int wret;

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

	if (priv->dev_ctx_mono_count) {
		list_for_each_entry_safe(dev_ctx, t_dev_ctx, &priv->dev_ctx_list, link) {
			list_del(&dev_ctx->link);
			priv->active_devctx_count--;
		}
	}

	if (priv->lg_fl_info.lg_file &&
		filp_close(priv->lg_fl_info.lg_file, NULL)) {
		wret = filp_close(priv->lg_fl_info.lg_file, NULL);
		if (wret)
			pr_err("Error %pe closing log file.\n",
				ERR_PTR(wret));
	}

	__list_del_entry(&priv->priv_data);
	/* No need to check, if reserved memory is allocated
	 * before calling for its release. Or clearing the
	 * un-set bit.
	 */
	of_reserved_mem_device_release(dev);

	/* Free Kobj created for logging */
	if (se_kobj)
		kobject_put(se_kobj);

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

	info = &info_list->info[idx];
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

	if (info->se_if_early_init) {
		/* start initializing ele fw */
		ret = info->se_if_early_init(priv);
		if (ret)
			goto exit;
	}

	list_add_tail(&priv->priv_data, &priv_data_list);

	ret = devm_add_action(dev, se_if_probe_cleanup, pdev);
	if (ret)
		goto exit;


	/* Mailbox client configuration */
	priv->se_mb_cl.dev		= dev;
	priv->se_mb_cl.tx_block		= false;
	priv->se_mb_cl.knows_txdone	= true;
	priv->se_mb_cl.rx_callback	= se_if_rx_callback;

	ret = se_if_request_channel(dev, &priv->tx_chan,
				    &priv->se_mb_cl,
				    (info_list->soc_id == SOC_ID_OF_IMX8DXL) ?
					MBOX_TXDB_NAME : MBOX_TX_NAME);
	if (ret)
		goto exit;

	ret = se_if_request_channel(dev,
				    &priv->rx_chan,
				    &priv->se_mb_cl,
				    (info_list->soc_id == SOC_ID_OF_IMX8DXL) ?
					MBOX_RXDB_NAME : MBOX_RX_NAME);
	if (ret)
		goto exit;

	if (info->mu_buff_size) {
		/* TODO: to get func get_mu_buf(), part of imx-mailbox.c */
		priv->mu_mem.ptr = get_mu_buf(priv->tx_chan);
		priv->mu_mem.size = info->mu_buff_size;
		priv->mu_mem.dma_addr = (u64)priv->mu_mem.ptr;
	}
	mutex_init(&priv->se_if_cmd_lock);
	mutex_init(&priv->se_msg_sq_ctl.se_msg_sq_lk);

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
	INIT_LIST_HEAD(&priv->dev_ctx_list);

	if (info->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret) {
			dev_err(dev,
				"failed to init reserved memory region %d\n",
				ret);
			goto exit;
		}
	}

	ret = init_device_context(priv, 0, &priv->priv_dev_ctx, &se_if_fops);
	if (ret) {
		dev_err(dev,
			"Failed[0x%x] to create device contexts.\n",
			ret);
		goto exit;
	}

	if (info->if_defs.se_if_type == SE_TYPE_ID_HSM) {
		ret = se_soc_info(priv);
		if (ret) {
			dev_err(dev,
				"failed[%pe] to fetch SoC Info\n", ERR_PTR(ret));
			goto exit;
		}
	}

	if (info->se_if_late_init) {
		ret = info->se_if_late_init(priv);
		if (ret)
			goto exit;
	}

	/* start ele rng */
	if (info->start_rng) {
		ret = info->start_rng(priv);
		if (ret)
			dev_err(dev, "Failed[0x%x] to start rng.\n", ret);
	}

	if (info->init_trng) {
		ret = info->init_trng(priv);
		if (ret)
			dev_err(dev, "Failed[0x%x] to init trng.\n", ret);
	}

	/* By default, there is no pending FW to be loaded.*/
	if (info_list->se_fw_img_nm.prim_fw_nm_in_rfs ||
			info_list->se_fw_img_nm.seco_fw_nm_in_rfs) {
		load_fw = get_load_fw_instance(priv);
		load_fw->se_fw_img_nm = &info_list->se_fw_img_nm;
		load_fw->is_fw_loaded = false;

		if (info_list->se_fw_img_nm.prim_fw_nm_in_rfs) {
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
			load_fw->imem_mgmt = true;
		}
	}

	/* exposing variables se_log and se_rcv_msg_timeout via sysfs */
	if (!se_kobj) {
		ret = se_sysfs_log();
		if (ret)
			pr_warn("Warn: Creating sysfs entry for se_log and  se_rcv_msg_timeout: %d\n", ret);
	}

	dev_info(dev, "i.MX secure-enclave: %s%d interface to firmware, configured.\n",
			get_se_if_name(priv->if_defs->se_if_type),
			priv->if_defs->se_instance_id);
	return ret;

exit:
	/* if execution control reaches here, if probe fails.
	 */
	return dev_err_probe(dev, ret, "%s: Probe failed.", __func__);
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

	se_rcv_msg_timeout = SE_RCV_MSG_DEFAULT_TIMEOUT;
	if (priv->if_defs->se_if_type == SE_TYPE_ID_V2X_DBG) {
		ret = v2x_suspend(priv);
		if (ret) {
			dev_err(dev, "Failure V2X-FW suspend[0x%x].", ret);
			goto exit;
		}
	}
	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		ret = se_save_imem_state(priv, &load_fw->imem);
		if (ret) {
			dev_err(dev, "Failure saving IMEM state[0x%x]", ret);
			goto exit;
		}
	}
exit:
	return ret;
}

static int se_resume(struct device *dev)
{
	struct se_if_priv *priv = dev_get_drvdata(dev);
	struct se_fw_load_info *load_fw;
	int ret = 0;

	if (priv->if_defs->se_if_type == SE_TYPE_ID_V2X_DBG) {
		ret = v2x_resume(priv);
		if (ret)
			dev_err(dev, "Failure V2X-FW resume[0x%x].", ret);
	}

	load_fw = get_load_fw_instance(priv);

	if (load_fw->imem_mgmt) {
		ret = se_restore_imem_state(priv, &load_fw->imem);
		if (ret)
			dev_err(dev, "Failure restoring IMEM state[0x%x]", ret);
	}

	return ret;
}

static const struct dev_pm_ops se_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(se_suspend, se_resume)
};

static struct platform_driver se_driver = {
	.driver = {
		.name = "fsl-se",
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
