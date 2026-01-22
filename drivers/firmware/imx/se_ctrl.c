// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
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
#include <uapi/linux/se_ioctl.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "se_ctrl.h"

#define MAX_SOC_INFO_DATA_SZ		256
#define MBOX_TX_NAME			"tx"
#define MBOX_RX_NAME			"rx"

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

static u32 get_se_soc_id(struct se_if_priv *priv)
{
	const struct se_soc_info *se_info = device_get_match_data(priv->dev);

	return se_info->soc_id;
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

static int load_firmware(struct se_if_priv *priv, const u8 *se_img_file_to_load)
{
	const struct firmware *fw = NULL;
	dma_addr_t se_fw_dma_addr;
	phys_addr_t se_fw_phyaddr;
	u8 *se_fw_buf;
	int ret;

	if (!se_img_file_to_load) {
		dev_err(priv->dev, "FW image is not provided.");
		return -EINVAL;
	}
	ret = request_firmware(&fw, se_img_file_to_load, priv->dev);
	if (ret)
		return ret;

	dev_info(priv->dev, "loading firmware %s.", se_img_file_to_load);

	/* allocate buffer to store the SE FW */
	se_fw_buf = dma_alloc_coherent(priv->dev, fw->size, &se_fw_dma_addr, GFP_KERNEL);
	if (!se_fw_buf) {
		ret = -ENOMEM;
		goto exit;
	}

	memcpy(se_fw_buf, fw->data, fw->size);
	se_fw_phyaddr = dma_to_phys(priv->dev, se_fw_dma_addr);
	ret = ele_fw_authenticate(priv, se_fw_phyaddr, se_fw_phyaddr);
	if (ret < 0) {
		dev_err(priv->dev,
			"Error %pe: Authenticate & load SE firmware %s.",
			ERR_PTR(ret), se_img_file_to_load);
		ret = -EPERM;
	}
	dma_free_coherent(priv->dev, fw->size, se_fw_buf, se_fw_dma_addr);
exit:
	release_firmware(fw);

	return ret;
}

static int se_load_firmware(struct se_if_priv *priv)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	int ret = 0;

	if (!load_fw->is_fw_tobe_loaded)
		return 0;

	if (load_fw->imem.state == ELE_IMEM_STATE_BAD) {
		ret = load_firmware(priv, load_fw->se_fw_img_nm->prim_fw_nm_in_rfs);
		if (ret) {
			dev_err(priv->dev, "Failed to load boot firmware.");
			return -EPERM;
		}
	}

	ret = load_firmware(priv, load_fw->se_fw_img_nm->seco_fw_nm_in_rfs);
	if (ret) {
		dev_err(priv->dev, "Failed to load runtime firmware.");
		return -EPERM;
	}

	load_fw->is_fw_tobe_loaded = false;

	return ret;
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
	se_shared_mem_mgmt->non_secure_mem.ptr =
			dma_alloc_coherent(priv->dev, MAX_DATA_SIZE_PER_USER,
					   &se_shared_mem_mgmt->non_secure_mem.dma_addr,
					   GFP_KERNEL);
	if (!se_shared_mem_mgmt->non_secure_mem.ptr)
		return -ENOMEM;

	se_shared_mem_mgmt->non_secure_mem.size = MAX_DATA_SIZE_PER_USER;
	se_shared_mem_mgmt->non_secure_mem.pos = 0;

	return 0;
}

static void cleanup_se_shared_mem(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;

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
			dev_dbg(priv->dev, "Copying output data to user.");
			if (do_cpy && copy_to_user(b_desc->usr_buf_ptr,
						   b_desc->shared_buf_ptr,
						   b_desc->size)) {
				dev_err(priv->dev, "Failure copying output data to user.");
				do_cpy = false;
			}
		}

		if (b_desc->shared_buf_ptr)
			memset(b_desc->shared_buf_ptr, 0, b_desc->size);

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

	for (i = 0; i < ARRAY_SIZE(pending_lists); i++) {
		list_for_each_entry_safe(b_desc, temp, pending_lists[i], link) {
			if (b_desc->shared_buf_ptr)
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);

			list_del(&b_desc->link);
			kfree(b_desc);
		}
	}
	se_shared_mem_mgmt->non_secure_mem.pos = 0;
}

static int add_b_desc_to_pending_list(void *shared_ptr_with_pos,
				      struct se_ioctl_setup_iobuf *io,
				      struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_buf_desc *b_desc;

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

static int init_misc_device_context(struct se_if_priv *priv, int ch_id,
				    struct se_if_device_ctx **new_dev_ctx,
				    const struct file_operations *se_if_fops)
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

	mutex_init(&dev_ctx->fops_lock);

	dev_ctx->priv = priv;
	*new_dev_ctx = dev_ctx;

	dev_ctx->miscdev = devm_kzalloc(priv->dev, sizeof(*dev_ctx->miscdev), GFP_KERNEL);
	if (!dev_ctx->miscdev) {
		*new_dev_ctx = NULL;
		return -ENOMEM;
	}

	dev_ctx->miscdev->name = dev_ctx->devname;
	dev_ctx->miscdev->minor = MISC_DYNAMIC_MINOR;
	dev_ctx->miscdev->fops = se_if_fops;
	dev_ctx->miscdev->parent = priv->dev;
	ret = misc_register(dev_ctx->miscdev);
	if (ret)
		return dev_err_probe(priv->dev, ret, "Failed to register misc device.");

	ret = devm_add_action_or_reset(priv->dev, if_misc_deregister, dev_ctx->miscdev);
	if (ret)
		return dev_err_probe(priv->dev, ret,
				     "Failed to add action to the misc-dev.");
	return ret;
}

static int init_device_context(struct se_if_priv *priv, int ch_id,
			       struct se_if_device_ctx **new_dev_ctx)
{
	struct se_if_device_ctx *dev_ctx;
	int ret = 0;

	dev_ctx = kzalloc(sizeof(*dev_ctx), GFP_KERNEL);

	if (!dev_ctx)
		return -ENOMEM;

	dev_ctx->devname = kasprintf(GFP_KERNEL, "%s0_ch%d",
				     get_se_if_name(priv->if_defs->se_if_type),
				     ch_id);
	if (!dev_ctx->devname) {
		kfree(dev_ctx);
		return -ENOMEM;
	}

	mutex_init(&dev_ctx->fops_lock);
	dev_ctx->priv = priv;
	*new_dev_ctx = dev_ctx;

	list_add_tail(&dev_ctx->link, &priv->dev_ctx_list);
	priv->active_devctx_count++;

	ret = init_se_shared_mem(dev_ctx);
	if (ret < 0) {
		kfree(dev_ctx->devname);
		kfree(dev_ctx);
		*new_dev_ctx = NULL;
	}

	return ret;
}

static int se_ioctl_cmd_snd_rcv_cleanup(struct se_if_device_ctx *dev_ctx, void __user *uarg,
					struct se_ioctl_cmd_snd_rcv_rsp_info *cmd_snd_rcv_rsp_info)
{
	/* shared memory is allocated before this IOCTL */
	se_dev_ctx_shared_mem_cleanup(dev_ctx);

	if (copy_to_user(uarg, cmd_snd_rcv_rsp_info, sizeof(*cmd_snd_rcv_rsp_info))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy cmd_snd_rcv_rsp_info from user.",
			dev_ctx->devname);
		return -EFAULT;
	}

	return 0;
}

static int se_ioctl_cmd_snd_rcv_rsp_handler(struct se_if_device_ctx *dev_ctx,
					    void __user *uarg)
{
	struct se_ioctl_cmd_snd_rcv_rsp_info cmd_snd_rcv_rsp_info = {0};
	struct se_if_priv *priv = dev_ctx->priv;
	int err = 0;

	if (copy_from_user(&cmd_snd_rcv_rsp_info, uarg,
			   sizeof(cmd_snd_rcv_rsp_info))) {
		dev_err(priv->dev,
			"%s: Failed to copy cmd_snd_rcv_rsp_info from user.",
			dev_ctx->devname);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -EFAULT;
	}

	if (cmd_snd_rcv_rsp_info.tx_buf_sz < SE_MU_HDR_SZ) {
		dev_err(priv->dev, "%s: User buffer too small(%d < %d)",
			dev_ctx->devname, cmd_snd_rcv_rsp_info.tx_buf_sz, SE_MU_HDR_SZ);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -ENOSPC;
	}

	err = se_chk_tx_msg_hdr(priv, (struct se_msg_hdr *)cmd_snd_rcv_rsp_info.tx_buf);
	if (err) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(cmd_snd_rcv_rsp_info.rx_buf_sz, GFP_KERNEL);
	if (!rx_msg) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -ENOMEM;
	}

	struct se_api_msg *tx_msg __free(kfree) =
		memdup_user(cmd_snd_rcv_rsp_info.tx_buf,
			    cmd_snd_rcv_rsp_info.tx_buf_sz);
	if (IS_ERR(tx_msg)) {
		err = PTR_ERR(tx_msg);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	if (tx_msg->header.tag != priv->if_defs->cmd_tag) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -EINVAL;
	}

	if (tx_msg->header.ver == priv->if_defs->fw_api_ver &&
	    get_load_fw_instance(priv)->is_fw_tobe_loaded) {
		err = se_load_firmware(priv);
		if (err) {
			dev_err(priv->dev, "Could not send msg as FW is not loaded.");
			se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
			return -EPERM;
		}
	}
	set_se_rcv_msg_timeout(priv, SE_RCV_MSG_LONG_TIMEOUT);

	err = ele_msg_send_rcv(dev_ctx, tx_msg, cmd_snd_rcv_rsp_info.tx_buf_sz,
			       rx_msg, cmd_snd_rcv_rsp_info.rx_buf_sz);
	if (err < 0) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	dev_dbg(priv->dev, "%s: %s %s.", dev_ctx->devname, __func__,
		"message received, start transmit to user");

	/* We may need to copy the output data to user before
	 * delivering the completion message.
	 */
	err = se_dev_ctx_cpy_out_data(dev_ctx);
	if (err < 0) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4, rx_msg,
			     cmd_snd_rcv_rsp_info.rx_buf_sz, false);

	if (copy_to_user(cmd_snd_rcv_rsp_info.rx_buf, rx_msg,
			 cmd_snd_rcv_rsp_info.rx_buf_sz)) {
		dev_err(priv->dev, "%s: Failed to copy to user.", dev_ctx->devname);
		err = -EFAULT;
	}

	err = se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);

	return err;
}

static int se_ioctl_get_mu_info(struct se_if_device_ctx *dev_ctx,
				void __user *uarg)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_ioctl_get_if_info if_info;
	struct se_if_node *if_node;
	int err = 0;

	if_node = container_of(priv->if_defs, typeof(*if_node), if_defs);

	if_info.se_if_id = 0;
	if_info.interrupt_idx = 0;
	if_info.tz = 0;
	if_info.did = 0;
	if_info.cmd_tag = priv->if_defs->cmd_tag;
	if_info.rsp_tag = priv->if_defs->rsp_tag;
	if_info.success_tag = priv->if_defs->success_tag;
	if_info.base_api_ver = priv->if_defs->base_api_ver;
	if_info.fw_api_ver = priv->if_defs->fw_api_ver;

	dev_dbg(priv->dev, "%s: info [se_if_id: %d, irq_idx: %d, tz: 0x%x, did: 0x%x].",
		dev_ctx->devname, if_info.se_if_id, if_info.interrupt_idx, if_info.tz,
		if_info.did);

	if (copy_to_user(uarg, &if_info, sizeof(if_info))) {
		dev_err(priv->dev, "%s: Failed to copy mu info to user.",
			dev_ctx->devname);
		err = -EFAULT;
	}

	return err;
}

/*
 * Copy a buffer of data to/from the user and return the address to use in
 * messages
 */
static int se_ioctl_setup_iobuf_handler(struct se_if_device_ctx *dev_ctx,
					void __user *uarg)
{
	struct se_shared_mem *shared_mem = NULL;
	struct se_ioctl_setup_iobuf io = {0};
	int err = 0;
	u32 pos;

	if (copy_from_user(&io, uarg, sizeof(io))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed copy iobuf config from user.",
			dev_ctx->devname);
		return -EFAULT;
	}

	dev_dbg(dev_ctx->priv->dev, "%s: io [buf: %p(%d) flag: %x].", dev_ctx->devname,
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

	/* No specific requirement for this buffer. */
	shared_mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;

	/* Check there is enough space in the shared memory. */
	dev_dbg(dev_ctx->priv->dev, "%s: req_size = %d, max_size= %d, curr_pos = %d",
		dev_ctx->devname, round_up(io.length, 8u), shared_mem->size,
		shared_mem->pos);

	if (shared_mem->size < shared_mem->pos ||
	    round_up(io.length, 8u) > (shared_mem->size - shared_mem->pos)) {
		dev_err(dev_ctx->priv->dev, "%s: Not enough space in shared memory.",
			dev_ctx->devname);
		return -ENOMEM;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;
	shared_mem->pos += round_up(io.length, 8u);
	io.ele_addr = (u64)shared_mem->dma_addr + pos;

	memset(shared_mem->ptr + pos, 0, io.length);
	if ((io.flags & SE_IO_BUF_FLAGS_IS_INPUT) ||
	    (io.flags & SE_IO_BUF_FLAGS_IS_IN_OUT)) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		if (copy_from_user(shared_mem->ptr + pos, io.user_buf, io.length)) {
			dev_err(dev_ctx->priv->dev,
				"%s: Failed copy data to shared memory.",
				dev_ctx->devname);
			return -EFAULT;
		}
	}

	err = add_b_desc_to_pending_list(shared_mem->ptr + pos, &io, dev_ctx);
	if (err < 0)
		dev_err(dev_ctx->priv->dev, "%s: Failed to allocate/link b_desc.",
			dev_ctx->devname);

copy:
	/* Provide the EdgeLock Enclave address to user space only if success.*/
	if (copy_to_user(uarg, &io, sizeof(io))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy iobuff setup to user.",
			dev_ctx->devname);
		err = -EFAULT;
	}

	return err;
}

/* IOCTL to provide SoC information */
static int se_ioctl_get_se_soc_info_handler(struct se_if_device_ctx *dev_ctx,
					    void __user *uarg)
{
	struct se_ioctl_get_soc_info soc_info;
	int err = -EINVAL;

	soc_info.soc_id = get_se_soc_id(dev_ctx->priv);
	soc_info.soc_rev = var_se_info.soc_rev;

	err = copy_to_user(uarg, (u8 *)(&soc_info), sizeof(soc_info));
	if (err) {
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy soc info to user.",
			dev_ctx->devname);
		err = -EFAULT;
	}

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
	struct se_if_priv *priv = dev_ctx->priv;
	int err;

	dev_dbg(priv->dev, "%s: write from buf (%p)%zu, ppos=%lld.", dev_ctx->devname,
		buf, size, ((ppos) ? *ppos : 0));

	scoped_cond_guard(mutex_intr, return -EBUSY, &dev_ctx->fops_lock) {
		if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx)
			return -EINVAL;

		err = se_chk_tx_msg_hdr(priv, (struct se_msg_hdr *)buf);
		if (err)
			return err;

		if (size < SE_MU_HDR_SZ) {
			dev_err(priv->dev, "%s: User buffer too small(%zu < %d).",
				dev_ctx->devname, size, SE_MU_HDR_SZ);
			return -ENOSPC;
		}

		struct se_api_msg *tx_msg __free(kfree) = memdup_user(buf, size);
		if (IS_ERR(tx_msg))
			return PTR_ERR(tx_msg);

		print_hex_dump_debug("from user ", DUMP_PREFIX_OFFSET, 4, 4,
				     tx_msg, size, false);

		err = ele_msg_send(dev_ctx, tx_msg, size);

		return err;
	}
}

/*
 * Read a message from the MU.
 * Blocking until a message is available.
 */
static ssize_t se_if_fops_read(struct file *fp, char __user *buf, size_t size,
			       loff_t *ppos)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	int err;

	dev_dbg(priv->dev, "%s: read to buf %p(%zu), ppos=%lld.", dev_ctx->devname,
		buf, size, ((ppos) ? *ppos : 0));

	scoped_cond_guard(mutex_intr, return -EBUSY, &dev_ctx->fops_lock) {
		if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx) {
			err = -EINVAL;
			goto exit;
		}

		err = ele_msg_rcv(dev_ctx, &priv->cmd_receiver_clbk_hdl);
		if (err < 0) {
			dev_err(priv->dev,
				"%s: Er[0x%x]: Signal Interrupted. Current act-dev-ctx count: %d.",
				dev_ctx->devname, err, dev_ctx->priv->active_devctx_count);
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
			dev_err(priv->dev, "%s: Failed to copy to user.",
				dev_ctx->devname);
			err = -EFAULT;
		} else {
			err = priv->cmd_receiver_clbk_hdl.rx_msg_sz;
		}
exit:
		priv->cmd_receiver_clbk_hdl.rx_msg_sz = 0;

		se_dev_ctx_shared_mem_cleanup(dev_ctx);

		return err;
	}
}

/* Open a character device. */
static int se_if_fops_open(struct inode *nd, struct file *fp)
{
	struct miscdevice *miscdev = fp->private_data;
	struct se_if_device_ctx *misc_dev_ctx;
	struct se_if_device_ctx *dev_ctx;
	struct se_if_priv *priv;
	int err = 0;

	priv = dev_get_drvdata(miscdev->parent);
	misc_dev_ctx = priv->priv_dev_ctx;

	scoped_cond_guard(mutex_intr, return -EBUSY, &misc_dev_ctx->fops_lock) {
		priv->dev_ctx_mono_count++;
		err = init_device_context(priv,
					  priv->dev_ctx_mono_count ?
					  priv->dev_ctx_mono_count
					  : priv->dev_ctx_mono_count++,
					  &dev_ctx);
		if (err)
			dev_err(priv->dev, "Failed[0x%x] to create dev-ctx.", err);
		else
			fp->private_data = dev_ctx;

		return err;
	}
}

/* Close a character device. */
static int se_if_fops_close(struct inode *nd, struct file *fp)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;

	scoped_cond_guard(mutex_intr, return -EBUSY, &dev_ctx->fops_lock) {
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

		kfree(dev_ctx->devname);
		kfree(dev_ctx);
	}

	return 0;
}

/* IOCTL entry point of a character device */
static long se_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	void __user *uarg = (void __user *)arg;
	long err;

	/* Prevent race during change of device context */
	scoped_cond_guard(mutex_intr, return -EBUSY, &dev_ctx->fops_lock) {
		switch (cmd) {
		case SE_IOCTL_ENABLE_CMD_RCV:
			if (!priv->cmd_receiver_clbk_hdl.dev_ctx) {
				if (!priv->cmd_receiver_clbk_hdl.rx_msg) {
					priv->cmd_receiver_clbk_hdl.rx_msg =
						kzalloc(MAX_NVM_MSG_LEN,
							GFP_KERNEL);
					if (!priv->cmd_receiver_clbk_hdl.rx_msg) {
						err = -ENOMEM;
						break;
					}
				}
				priv->cmd_receiver_clbk_hdl.rx_msg_sz = MAX_NVM_MSG_LEN;
				priv->cmd_receiver_clbk_hdl.dev_ctx = dev_ctx;
				err = 0;
			} else {
				err = -EBUSY;
			}
			break;
		case SE_IOCTL_GET_MU_INFO:
			err = se_ioctl_get_mu_info(dev_ctx, uarg);
			break;
		case SE_IOCTL_SETUP_IOBUF:
			err = se_ioctl_setup_iobuf_handler(dev_ctx, uarg);
			break;
		case SE_IOCTL_GET_SOC_INFO:
			err = se_ioctl_get_se_soc_info_handler(dev_ctx, uarg);
			break;
		case SE_IOCTL_CMD_SEND_RCV_RSP:
			err = se_ioctl_cmd_snd_rcv_rsp_handler(dev_ctx, uarg);
			break;
		default:
			err = -EINVAL;
			dev_dbg(priv->dev, "%s: IOCTL %.8x not supported.",
				dev_ctx->devname, cmd);
		}
	}

	return err;
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
				     "Failed to add-action for removal of mbox: %s.",
				     name);
	*chan = t_chan;

	return ret;
}

static void se_if_probe_cleanup(void *plat_dev)
{
	struct se_if_device_ctx *dev_ctx, *t_dev_ctx;
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

	if (priv->dev_ctx_mono_count) {
		list_for_each_entry_safe(dev_ctx, t_dev_ctx, &priv->dev_ctx_list, link) {
			list_del(&dev_ctx->link);
			priv->active_devctx_count--;
		}
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
	set_se_rcv_msg_timeout(priv, SE_RCV_MSG_DEFAULT_TIMEOUT);

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
	INIT_LIST_HEAD(&priv->dev_ctx_list);

	if (if_node->reserved_dma_ranges) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret,
					    "Failed to init reserved memory region.");
	}

	ret = init_misc_device_context(priv, 0, &priv->priv_dev_ctx, &se_if_fops);
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

	set_se_rcv_msg_timeout(priv, SE_RCV_MSG_DEFAULT_TIMEOUT);
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
