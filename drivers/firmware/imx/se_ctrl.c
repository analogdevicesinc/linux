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
#include <uapi/linux/se_ioctl.h>

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

	for (i = 0; i < 2; i++) {
		list_for_each_entry_safe(b_desc, temp,
					 pending_lists[i], link) {

			if (b_desc->shared_buf_ptr)
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);

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
	const struct se_if_node_info *info = container_of(priv->if_defs,
							typeof(*info),
							if_defs);
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
		dev_ctx->devname = kasprintf(GFP_KERNEL, "%s_ch%d",
					     info->se_name, ch_id);
	else
		dev_ctx->devname = devm_kasprintf(priv->dev,
						  GFP_KERNEL, "%s_ch%d",
						  info->se_name, ch_id);
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
	const struct se_if_node_info *info = container_of(dev_ctx->priv->if_defs,
							typeof(*info),
							if_defs);
	struct se_ioctl_cmd_snd_rcv_rsp_info cmd_snd_rcv_rsp_info;
	struct se_if_node_info_list *info_list
				= container_of(info,
						typeof(*info_list),
						info[info->se_if_id]);
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
		!info_list->load_hsm_fw.is_fw_loaded) {
		err = se_load_firmware(priv);
		if (err) {
			dev_err(priv->dev, "Could not send the message as FW is not loaded.");
			err = -EPERM;
			goto exit;
		}
	}
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

	if_info.se_if_id = info->se_if_id;
	if_info.interrupt_idx = 0;
	if_info.tz = 0;
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

	/* No specific requirement for this buffer. */
	shared_mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;

	/* Check there is enough space in the shared memory. */
	if (shared_mem->size < shared_mem->pos ||
		round_up(io.length, 8u) >= (shared_mem->size - shared_mem->pos)) {
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
	const struct se_if_node_info_list *info_list;
	struct se_ioctl_get_soc_info soc_info;
	int err = -EINVAL;

	info_list = device_get_match_data(dev_ctx->priv->dev);
	if (!info_list)
		goto exit;

	soc_info.soc_id = info_list->soc_id;
	soc_info.soc_rev = se_soc_rev;

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
	kfree(dev_ctx->devname);
	kfree(dev_ctx);

	return 0;
}

/* IOCTL entry point of a character device */
static long se_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	int err;

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

	if (priv->priv_dev_ctx && priv->priv_dev_ctx->miscdev) {
		devm_remove_action(dev, if_misc_deregister, &priv->priv_dev_ctx->miscdev);
		misc_deregister(priv->priv_dev_ctx->miscdev);
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
