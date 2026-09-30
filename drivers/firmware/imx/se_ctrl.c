// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
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
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>
#include <uapi/linux/se_ioctl.h>

#include "ele_base_msg.h"
#include "ele_common.h"
#include "ele_fw_api.h"
#include "se_ctrl.h"

/* Maximum response buffer size in bytes for debug-dump replies. */
#define MAX_ALLOWED_TX_MSG_SZ		SZ_4K

#define MAX_SOC_INFO_DATA_SZ		256

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

static u32 get_se_soc_id(struct se_if_priv *priv)
{
	const struct se_if_node *if_node = device_get_match_data(priv->dev);

	return if_node->se_info->soc_id;
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

static int load_firmware(struct se_if_priv *priv, const u8 *se_img_file_to_load)
{
	const struct firmware *fw = NULL;
	dma_addr_t se_fw_dma_addr;
	u32 se_fw_buf_len;
	void *se_fw_buf;
	int ret;

	if (!se_img_file_to_load) {
		dev_err(priv->dev, "FW image is not provided.\n");
		return -EINVAL;
	}
	ret = request_firmware(&fw, se_img_file_to_load, priv->dev);
	if (ret)
		return ret;

	if (fw->size > U32_MAX) {
		ret = -EFBIG;
		release_firmware(fw);
		return ret;
	}
	dev_info(priv->dev, "loading firmware %s.\n", se_img_file_to_load);

	/*
	 * Serialize access to priv_dev_ctx shared memory to prevent pos
	 * corruption if two driver-internal callers run concurrently (e.g.
	 * ele_get_info() racing with load_firmware()).
	 */
	scoped_guard(mutex, &priv->priv_dev_ctx->fops_lock) {
		se_fw_buf_len = fw->size;
		ret = get_shared_mem_slot(priv->priv_dev_ctx,
					  &se_fw_buf_len, &se_fw_dma_addr,
					  &se_fw_buf);
		if (ret) {
			dev_err(priv->dev, "Failed to allocate firmware shared buffer: %d\n",
				ret);
			release_firmware(fw);
			return ret;
		}

		memcpy(se_fw_buf, fw->data, fw->size);
		ret = ele_fw_authenticate(priv, se_fw_dma_addr, se_fw_dma_addr);
		if (ret < 0) {
			dev_err(priv->dev,
				"Error %pe: Authenticate & load SE firmware %s.",
				ERR_PTR(ret), se_img_file_to_load);
			ret = -EPERM;
		}
		if (!se_is_fw_busy_ctx(priv->priv_dev_ctx))
			se_dev_ctx_shared_mem_cleanup(priv->priv_dev_ctx);
	}

	release_firmware(fw);

	return ret;
}

static int se_load_firmware(struct se_if_priv *priv)
{
	struct se_fw_load_info *load_fw = get_load_fw_instance(priv);
	int ret = 0;

	guard(mutex)(&load_fw->load_fw_lock);
	if (!load_fw->is_fw_tobe_loaded)
		return 0;

	if (load_fw->imem.state == ELE_IMEM_STATE_BAD) {
		ret = load_firmware(priv, load_fw->se_fw_img_nm->prim_fw_nm_in_rfs);
		if (ret) {
			dev_err(priv->dev, "Failed to load boot firmware.\n");
			return -EPERM;
		}
	}

	ret = load_firmware(priv, load_fw->se_fw_img_nm->seco_fw_nm_in_rfs);
	if (ret) {
		dev_err(priv->dev, "Failed to load runtime firmware.\n");
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

	if (priv->mem_pool)
		INIT_LIST_HEAD(&se_shared_mem_mgmt->mem_pool_buf_list);

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

static void cleanup_se_shared_mem(struct se_if_device_ctx *dev_ctx, bool reclaim)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;
	bool free_dma_buf;

	/*
	 * mem_pool_buf_list is only initialised for interfaces that own a
	 * gen_pool (priv->mem_pool != NULL). On interfaces without a pool
	 * (e.g. imx93, which has no pool_name) the list head is left
	 * zero-filled, so se_cleanup_mem_pool_buf() must not walk it here or
	 * list_for_each_entry_safe() would dereference a NULL head and panic
	 * the kernel on close/teardown. Skip the pool cleanup entirely when
	 * there is no pool; there is nothing to reclaim in that case.
	 */
	if (priv->mem_pool)
		se_cleanup_mem_pool_buf(dev_ctx, reclaim);

	/* Guard against being called before shared memory was ever allocated
	 * (e.g. probe failure before dma_alloc_coherent succeeded).
	 */
	if (!se_shared_mem_mgmt->non_secure_mem.ptr)
		return;

	/*
	 * Decide whether the DMA buffer can be released before touching the
	 * pending lists. se_dev_ctx_shared_mem_cleanup() resets
	 * non_secure_mem.pos, so the "nothing staged" test must be sampled
	 * here first. When reclaim is false the buffer is released only if no
	 * data is still staged for the firmware; otherwise the enclave may
	 * still be DMA-ing into it and the buffer is deliberately leaked to
	 * avoid a DMA-after-free.
	 */
	free_dma_buf = reclaim || !se_shared_mem_mgmt->non_secure_mem.pos;

	/*
	 * Free any se_buf_desc items that were never consumed (e.g. when the
	 * fd is closed while pending I/O buffers are still listed). This must
	 * happen before the DMA backing memory is released to avoid a leak.
	 */
	se_dev_ctx_shared_mem_cleanup(dev_ctx);

	if (free_dma_buf) {
		dma_free_coherent(priv->dev, MAX_DATA_SIZE_PER_USER,
				  se_shared_mem_mgmt->non_secure_mem.ptr,
				  se_shared_mem_mgmt->non_secure_mem.dma_addr);
	}

	/*
	 * Drop the host-side tracking unconditionally. On the reclaim path the
	 * buffer has been freed. On the deliberate-leak path the buffer is
	 * abandoned on purpose, so clearing the pointer here guarantees a later
	 * cleanup pass (e.g. se_if_priv_release()) cannot double-free it.
	 */
	se_shared_mem_mgmt->non_secure_mem.ptr = NULL;
	se_shared_mem_mgmt->non_secure_mem.dma_addr = 0;
	se_shared_mem_mgmt->non_secure_mem.size = 0;
	se_shared_mem_mgmt->non_secure_mem.pos = 0;
}

static int se_dev_ctx_cpy_out_data(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_buf_desc *b_desc, *temp;
	bool do_cpy = true;

	list_for_each_entry_safe(b_desc, temp, &se_shared_mem_mgmt->pending_out, link) {
		if (b_desc->usr_buf_ptr && b_desc->shared_buf_ptr && do_cpy) {
			dev_dbg(priv->dev, "Copying output data to user.\n");
			if (do_cpy && copy_to_user(b_desc->usr_buf_ptr,
						   b_desc->shared_buf_ptr,
						   b_desc->size)) {
				dev_err(priv->dev, "Failure copying output data to user.\n");
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
void se_dev_ctx_shared_mem_cleanup(struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct list_head *pending_lists[] = {&se_shared_mem_mgmt->pending_in,
						&se_shared_mem_mgmt->pending_out};
	struct se_buf_desc *b_desc, *temp;
	bool is_fw_busy_dev_ctx;
	int i;

	/*
	 * If this context is the one that caused a firmware timeout the shared
	 * DMA buffers may still be actively read/written by the firmware.
	 */
	is_fw_busy_dev_ctx = se_is_fw_busy_ctx(dev_ctx);

	for (i = 0; i < ARRAY_SIZE(pending_lists); i++) {
		list_for_each_entry_safe(b_desc, temp, pending_lists[i], link) {
			if (!is_fw_busy_dev_ctx && b_desc->shared_buf_ptr)
				memset(b_desc->shared_buf_ptr, 0, b_desc->size);

			list_del(&b_desc->link);
			kfree(b_desc);
		}
	}

	/*
	 * Keep non_secure_mem.pos non-zero while this context still owns an
	 * outstanding firmware transaction. A non-zero pos is the marker that
	 * data is still staged for the enclave, which cleanup_se_shared_mem()
	 * uses to decide the buffer must be leaked rather than freed. Resetting
	 * it here would let a later teardown pass free a buffer the enclave may
	 * still be DMA-ing into.
	 */
	if (!is_fw_busy_dev_ctx)
		se_shared_mem_mgmt->non_secure_mem.pos = 0;
}

static struct se_buf_desc *add_b_desc_to_pending_list(void *shared_ptr_with_pos,
						      struct se_ioctl_setup_iobuf *io,
						      struct se_if_device_ctx *dev_ctx)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_buf_desc *b_desc = NULL;

	b_desc = kzalloc_obj(*b_desc);
	if (!b_desc)
		return ERR_PTR(-ENOMEM);

	b_desc->shared_buf_ptr = shared_ptr_with_pos;
	b_desc->usr_buf_ptr = u64_to_user_ptr(io->user_buf);
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

	return b_desc;
}

static void se_if_open_gate_release(struct kref *kref)
{
	struct se_if_open_gate *gate =
		container_of(kref, struct se_if_open_gate, refcount);

	kfree(gate);
}

static bool se_if_open_gate_get(struct se_if_open_gate *gate)
{
	if (!gate)
		return false;

	return kref_get_unless_zero(&gate->refcount);
}

static void se_if_open_gate_put(struct se_if_open_gate *gate)
{
	if (gate)
		kref_put(&gate->refcount, se_if_open_gate_release);
}

/*
 * Distinct lockdep class for the internal priv_dev_ctx fops_lock. Taking it
 * while an open context's fops_lock is held (for example a firmware load
 * triggered from an ioctl) is valid hierarchical locking, but shares the same
 * class as the per-open fops_lock and would otherwise be misreported as
 * recursive locking by lockdep.
 */
static struct lock_class_key se_priv_ctx_fops_key;

static int init_misc_device_context(struct se_if_priv *priv, int ch_id,
				    struct se_if_device_ctx **new_dev_ctx,
				    const struct file_operations *se_if_fops)
{
	struct se_if_open_gate *gate = NULL;
	struct se_if_device_ctx *dev_ctx;
	int ret = -ENOMEM;

	dev_ctx = kzalloc_obj(*dev_ctx);
	if (!dev_ctx)
		return -ENOMEM;

	dev_ctx->priv = priv;
	dev_ctx->devname = kasprintf(GFP_KERNEL, "%s0_ch%d",
				     get_se_if_name(priv->if_defs->se_if_type),
				     ch_id);
	if (!dev_ctx->devname) {
		kfree(dev_ctx);
		return -ENOMEM;
	}

	mutex_init(&dev_ctx->fops_lock);
	lockdep_set_class(&dev_ctx->fops_lock, &se_priv_ctx_fops_key);

	kref_init(&dev_ctx->refcount);
	dev_ctx->cleanup_done = false;
	*new_dev_ctx = dev_ctx;
	set_se_rcv_msg_timeout(dev_ctx, SE_RCV_MSG_DEFAULT_TIMEOUT_MS);

	ret = init_se_shared_mem(dev_ctx);
	if (ret < 0)
		goto exit;

	gate = kzalloc_obj(*gate);
	if (!gate) {
		ret = -ENOMEM;
		goto exit;
	}

	mutex_init(&gate->lock);
	kref_init(&gate->refcount);    /* device-owned reference */
	gate->priv = priv;
	gate->dying = false;
	priv->open_gate = gate;

	/*
	 * The miscdevice storage is now owned by the open gate object.
	 * priv->priv_dev_ctx still keeps a pointer to that miscdevice.
	 */
	dev_ctx->miscdev = &gate->miscdev;

	dev_ctx->miscdev->name = dev_ctx->devname;
	dev_ctx->miscdev->minor = MISC_DYNAMIC_MINOR;
	dev_ctx->miscdev->fops = se_if_fops;
	dev_ctx->miscdev->parent = priv->dev;

	return 0;
exit:
	*new_dev_ctx = NULL;

	if (gate) {
		priv->open_gate = NULL;
		se_if_open_gate_put(gate);
	}
	cleanup_se_shared_mem(dev_ctx, true);
	kfree(dev_ctx->devname);
	kfree(dev_ctx);
	return ret;
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

/*
 * Forward declarations. se_if_probe_cleanup() and se_if_probe() are kept
 * together as the teardown/probe pair, but several helpers, the file
 * operations table and the firmware-busy work handler they reference are
 * defined further down in this file.
 */
static void dlink_dev_ctx(struct se_if_device_ctx *dev_ctx);
static void cleanup_dev_ctx(struct se_if_device_ctx *dev_ctx, bool is_fclose);
static void se_clear_fw_busy(struct se_if_priv *priv);
static void se_if_dev_ctx_release(struct kref *kref);
static void se_if_priv_release(struct kref *kref);
static int se_if_misc_register(struct se_if_priv *priv);
static void se_fw_busy_work(struct work_struct *work);
static const struct file_operations se_if_fops;

static void se_if_probe_cleanup(void *plat_dev)
{
	struct platform_device *pdev = plat_dev;
	struct se_if_device_ctx *dev_ctx;
	struct device *dev = &pdev->dev;
	struct fw_busy_info *fbusy_info;
	struct se_if_priv *priv;

	priv = dev_get_drvdata(dev);
	if (!priv)
		return;

	fbusy_info = &priv->fw_busy_info;

	/*
	 * Announce teardown, then wake any in-flight waiter. going_away makes
	 * ele_msg_send_rcv() bail out instead of arming a new transaction and
	 * lets ele_msg_rcv() tell a teardown-forced completion apart from a
	 * real response; it must be set before complete_all().
	 *
	 * Set it under clbk_rx_lock, not se_if_cmd_lock: se_if_cmd_lock is held
	 * across the whole blocking transaction, so taking it here would stall
	 * unbind for a full receive-timeout. clbk_rx_lock is the short spinlock
	 * ele_msg_send_rcv() holds while arming, so this closes the lost-wakeup
	 * window - the sender either sees going_away and bails before arming, or
	 * armed first and this store (and complete_all()) is ordered after its
	 * reinit_completion() - and supplies the ordering the relaxed atomics do
	 * not.
	 */
	scoped_guard(spinlock_irqsave, &priv->waiting_rsp_clbk_hdl.clbk_rx_lock)
		atomic_set(&priv->going_away, 1);
	/*
	 * Wake the waiter before iterating the device-context list. It sleeps on
	 * this completion holding dev_ctx->fops_lock, which cleanup_dev_ctx()
	 * below also takes, so completing first avoids an unbind hang. Runs
	 * outside clbk_rx_lock; the going_away store above already orders it
	 * against the arming path.
	 */
	complete_all(&priv->waiting_rsp_clbk_hdl.done);

	/*
	 * Only now reserve the messaging interface for this teardown flow.
	 *
	 * se_reserve_msg_if() blocks on msg_excl_flow_lock, and the fw_busy
	 * recovery worker (se_fw_busy_work() -> se_clear_fw_busy()) may already
	 * hold that reservation while parked uninterruptibly in ele_msg_rcv()
	 * waiting on a possibly hung firmware for up to the full receive
	 * timeout. The only thing that cuts that wait short is the complete_all()
	 * above, so it MUST run before this reserve: otherwise teardown would
	 * sleep on the reservation the worker holds, the worker would stay
	 * blocked on firmware, and unbind would stall for the entire multi-
	 * thousand-second timeout (an unbind hang / hung-task).
	 *
	 * With going_away already set and the in-flight waiter already forced to
	 * unwind, the worker returns promptly (its send is failed with -ENODEV),
	 * drops the reservation, and this call acquires it without waiting on
	 * anything firmware-related. From here on teardown is the exclusive
	 * owner: ele_msg_send_rcv() lets only this task's priv_dev_ctx close
	 * traffic through and rejects every other caller.
	 */
	se_reserve_msg_if(priv);

	/*
	 * Mark the private device context as cleanup_done first.
	 * This prevents new device contexts from being created in open().
	 */
	if (priv->priv_dev_ctx) {
		/*
		 * Mark cleanup_done under fops_lock so that se_if_fops_open(),
		 * which checks cleanup_done while holding fops_lock, cannot
		 * race past this and add a new device context after teardown.
		 */
		scoped_guard(mutex, &priv->priv_dev_ctx->fops_lock)
			priv->priv_dev_ctx->cleanup_done = true;

		if (priv->open_gate) {
			scoped_guard(mutex, &priv->open_gate->lock) {
				priv->open_gate->dying = true;
				priv->open_gate->priv = NULL;
			}
		}

		/*
		 * misc_register() is deferred to the end of probe, so the
		 * device may have a miscdev set up but never registered if
		 * probe failed before se_if_misc_register(). Only deregister
		 * when registration actually succeeded.
		 */
		if (priv->open_gate && priv->open_gate->registered &&
		    priv->priv_dev_ctx->miscdev)
			misc_deregister(priv->priv_dev_ctx->miscdev);
	}

	while (true) {
		bool list_was_empty = false;

		dev_ctx = NULL;

		scoped_guard(mutex, &priv->modify_lock) {
			if (list_empty(&priv->dev_ctx_list)) {
				list_was_empty = true;
			} else {
				dev_ctx = list_first_entry(&priv->dev_ctx_list,
							   struct se_if_device_ctx, link);

				/* pin this context so close() cannot free it under us */
				kref_get(&dev_ctx->refcount);
				dlink_dev_ctx(dev_ctx);
			}
		}

		if (list_was_empty)
			break;

		/*
		 * Local cleanup outside the global lock avoids ABBA deadlock
		 * with paths that already take dev_ctx->fops_lock first.
		 */
		cleanup_dev_ctx(dev_ctx, false);
		kref_put(&dev_ctx->refcount, se_if_dev_ctx_release);
	}

	se_release_msg_if(priv);
	/*
	 * Release any dev_ctx retained by the firmware-busy circuit breaker.
	 * A synchronous command that timed out parks its dev_ctx in
	 * fbusy_info->fw_busy_dev_ctx so that a late firmware response can still be
	 * routed back. If no such response arrived before teardown, that
	 * retained reference must be dropped here to avoid a leak.
	 * se_clear_fw_busy() is safe to call unconditionally: it checks
	 * fbusy_info->fw_busy_dev_ctx under fw_busy_lock and is a no-op when
	 * nothing is parked.
	 */
	se_clear_fw_busy(priv);

	/*
	 * Free the mailbox channels under se_if_cmd_lock. ele_msg_send_rcv()
	 * holds se_if_cmd_lock for the full duration of a synchronous
	 * transaction, including the mbox_send_message() call on priv->tx_chan.
	 * going_away was set above and complete_all() has already woken any
	 * in-flight waiter, so any transaction in progress will unwind to
	 * -ENODEV and release the lock quickly. Acquiring se_if_cmd_lock here
	 * guarantees no caller is still touching the channels when they are
	 * freed, and nulling the pointers under the lock prevents any sender
	 * that races past the going_away check from accessing a freed channel.
	 */
	scoped_guard(mutex, &priv->se_if_cmd_lock) {
		if (priv->rx_chan) {
			mbox_free_channel(priv->rx_chan);
			priv->rx_chan = NULL;
		}
		if (priv->tx_chan) {
			mbox_free_channel(priv->tx_chan);
			priv->tx_chan = NULL;
		}
	}

	/*
	 * Cancel any pending fw_busy_work before dropping the initial priv
	 * reference. going_away was set above, so no new work can be scheduled
	 * after this point. Canceling here while probe_cleanup still holds its
	 * own priv reference prevents two races:
	 *
	 * 1. UAF: if fw_busy_work has dev_ctx == priv_dev_ctx, letting it run
	 *    past this point while se_if_priv_release() frees priv_dev_ctx
	 *    causes a use-after-free of dev_ctx->fops_lock in se_clear_fw_busy().
	 *
	 * 2. Deadlock: if fw_busy_work drops the last priv reference,
	 *    se_if_dev_ctx_release() -> se_if_priv_release() would call
	 *    cancel_work_sync() from inside the worker, causing the worker to
	 *    wait for its own completion.
	 *
	 * Both are avoided by canceling here: probe_cleanup still holds a priv
	 * reference so the worker cannot invoke se_if_priv_release(), and the
	 * cancel runs from a non-worker context.
	 */
	cancel_work_sync(&fbusy_info->fw_busy_work);

	/*
	 * Reclaim priv_dev_ctx shared memory before of_reserved_mem_device_release():
	 * cleanup_se_shared_mem() calls dma_free_coherent(), which must run while
	 * the DMA config is still active. fw_busy_work was canceled above, so no
	 * concurrent caller holds priv_dev_ctx->fops_lock.
	 *
	 * reclaim=true is safe even if FW hung at teardown (command timed out, pos
	 * still non-zero): the ELE region is no-map/shared-dma-pool, so freeing only
	 * drops the kernel VA/bitmap while the physical pages stay reserved (no
	 * DMA-after-free). The next probe also sends ELE_GET_INFO into a fresh
	 * buffer before accepting commands, so a stale FW write to the old buffer is
	 * never observed by the new driver instance.
	 */
	if (priv->priv_dev_ctx) {
		scoped_guard(mutex, &priv->priv_dev_ctx->fops_lock)
			cleanup_se_shared_mem(priv->priv_dev_ctx, true);
	}

	/*
	 * Release the reserved DMA memory configuration at unbind time, paired
	 * with of_reserved_mem_device_init() in se_if_probe(). This must not be
	 * deferred to se_if_priv_release(): that runs when the last file
	 * descriptor closes, which may be after a new driver instance has already
	 * called of_reserved_mem_device_init() on the same struct device. Calling
	 * the release at that point would corrupt the new instance's DMA setup.
	 */
	of_reserved_mem_device_release(dev);

	/*
	 * Being device managed buffer, no need to free the buffer allocated
	 * in se probe to store encrypted IMEM.
	 */

	dev_set_drvdata(dev, NULL);

	/* Drop the initial reference - priv will be freed when last fd closes */
	kref_put(&priv->refcount, se_if_priv_release);
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
	/*
	 * Pin the parent device for the lifetime of priv. A file descriptor may
	 * stay open after the device is unbound; close() then still passes
	 * priv->dev to dma_free_coherent()/dev_warn(). Without this reference
	 * the struct device could be freed while priv->dev still points at it,
	 * so the reference is dropped in se_if_priv_release() via put_device().
	 */
	get_device(priv->dev);
	kref_init(&priv->refcount);
	priv->if_defs = &if_node->if_defs;
	dev_set_drvdata(dev, priv);

	spin_lock_init(&priv->cmd_receiver_clbk_hdl.clbk_rx_lock);
	spin_lock_init(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock);
	priv->msg_excl_flow.msg_excl_owner = NULL;
	spin_lock_init(&priv->msg_excl_flow.msg_excl_lock);
	mutex_init(&priv->msg_excl_flow.msg_excl_flow_lock);
	struct fw_busy_info *fbusy_info = &priv->fw_busy_info;

	atomic_set(&fbusy_info->fw_busy, 0);

	spin_lock_init(&fbusy_info->fw_busy_lock);
	fbusy_info->fw_busy_dev_ctx = NULL;
	INIT_WORK(&fbusy_info->fw_busy_work, se_fw_busy_work);

	init_completion(&priv->waiting_rsp_clbk_hdl.done);
	init_completion(&priv->cmd_receiver_clbk_hdl.done);
	INIT_LIST_HEAD(&priv->dev_ctx_list);

	mutex_init(&priv->se_if_cmd_lock);
	mutex_init(&priv->modify_lock);

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

	ret = init_misc_device_context(priv, 0, &priv->priv_dev_ctx, &se_if_fops);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to create device contexts.\n");

	if (if_node->if_defs.se_if_type == SE_TYPE_ID_HSM) {
		ret = get_se_soc_info(priv, se_info);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to fetch SoC Info.\n");
	}

	/*
	 * All probe-time initialization is complete; expose the
	 * interface to userspace last so that an open()/ioctl cannot
	 * race against a not-yet-initialized device.
	 */
	ret = se_if_misc_register(priv);
	if (ret)
		return ret;

	dev_info(dev, "i.MX secure-enclave: %s0 interface to firmware, configured.\n",
		 get_se_if_name(priv->if_defs->se_if_type));

	return ret;
}

/*
 * Expose the interface to userspace. Deferred until the end of probe so
 * the device node only becomes openable after SoC info has been fetched
 * and, on SoCs with IMEM management, the encrypted-IMEM buffer has been
 * allocated. This prevents userspace from opening the node and issuing
 * commands against a partially initialized interface.
 */
static int se_if_misc_register(struct se_if_priv *priv)
{
	int ret;

	ret = misc_register(priv->priv_dev_ctx->miscdev);
	if (ret)
		return dev_err_probe(priv->dev, ret,
				     "Failed to register misc device.");

	priv->open_gate->registered = true;

	return 0;
}

static void se_if_priv_release(struct kref *kref)
{
	struct se_if_priv *priv = container_of(kref, struct se_if_priv, refcount);

	/*
	 * Free priv_dev_ctx if it still exists. se_if_priv_release() always
	 * runs after se_if_probe_cleanup() has completed: the initial kref
	 * held by probe_cleanup is the last one dropped by probe_cleanup
	 * itself, so no other kref_put() can reach zero -- and therefore
	 * trigger se_if_priv_release() -- until probe_cleanup's own
	 * kref_put() fires. By that time cleanup_se_shared_mem() and
	 * of_reserved_mem_device_release() have already run in
	 * probe_cleanup, so only the struct itself and its devname string
	 * need to be freed here. Calling cleanup_se_shared_mem() again
	 * would be a use-after-free of already-freed DMA memory.
	 */
	if (priv->priv_dev_ctx) {
		kfree(priv->priv_dev_ctx->devname);
		kfree(priv->priv_dev_ctx);
		priv->priv_dev_ctx = NULL;
	}
	/*
	 * Be defensive: if teardown did not already drop the device-owned
	 * gate reference for some reason, release it here.
	 */
	if (priv->open_gate) {
		se_if_open_gate_put(priv->open_gate);
		priv->open_gate = NULL;
	}

	/*
	 * Drop the reference on priv->dev taken in se_if_probe(). The device was
	 * pinned so that a file descriptor closed after device unbind can still
	 * safely pass priv->dev to dma_free_coherent()/dev_warn().
	 */
	put_device(priv->dev);
	mutex_destroy(&priv->load_fw.load_fw_lock);
	mutex_destroy(&priv->modify_lock);
	mutex_destroy(&priv->se_if_cmd_lock);
	mutex_destroy(&priv->msg_excl_flow.msg_excl_flow_lock);

	/* Free any remaining resources that weren't devm-managed */
	kfree(priv);
}

static void se_if_dev_ctx_release(struct kref *kref)
{
	struct se_if_device_ctx *dev_ctx =
		container_of(kref, struct se_if_device_ctx, refcount);
	struct se_if_priv *priv = dev_ctx->priv;

	kfree(dev_ctx);

	/* drop the priv reference owned by this device context */
	kref_put(&priv->refcount, se_if_priv_release);
}

/*
 * se_reserve_msg_if() - reserve the SE messaging interface for the current task.
 *
 * Blocks on msg_excl_flow_lock until this task owns the reservation, then
 * publishes current as msg_excl_owner under msg_excl_lock. While a reservation
 * is held, ele_msg_send_rcv() lets only the owning task issue transactions and
 * rejects every other caller with -EBUSY. Used by the fw_busy recovery flow in
 * se_clear_fw_busy() to drive its teardown-close messages through the otherwise
 * closed circuit breaker.
 *
 * If a second flow tries to reserve while the interface is already reserved,
 * it sleeps on msg_excl_flow_lock until the current owner calls
 * se_release_msg_if(). Must be called from process/workqueue context (it may
 * sleep) and every successful call must be balanced by se_release_msg_if().
 *
 * Return: 0 (the reservation is always acquired once this returns).
 */
int se_reserve_msg_if(struct se_if_priv *priv)
{
	unsigned long flags;

	mutex_lock(&priv->msg_excl_flow.msg_excl_flow_lock);
	/*
	 * The mutex guarantees this task is now the sole reserver, so
	 * msg_excl_owner is either NULL or already current. Publish current
	 * under msg_excl_lock so the lockless READ_ONCE in ele_msg_send_rcv()
	 * observes a consistent pointer.
	 */
	spin_lock_irqsave(&priv->msg_excl_flow.msg_excl_lock, flags);
	priv->msg_excl_flow.msg_excl_owner = current;
	spin_unlock_irqrestore(&priv->msg_excl_flow.msg_excl_lock, flags);

	return 0;
}

/* se_release_msg_if() - release a reservation taken by se_reserve_msg_if(). */
void se_release_msg_if(struct se_if_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->msg_excl_flow.msg_excl_lock, flags);
	priv->msg_excl_flow.msg_excl_owner = NULL;
	spin_unlock_irqrestore(&priv->msg_excl_flow.msg_excl_lock, flags);
	mutex_unlock(&priv->msg_excl_flow.msg_excl_flow_lock);
}

/* se_clear_fw_busy() - atomically clear fw_busy and reclaim the parked dev_ctx. */
static void se_clear_fw_busy(struct se_if_priv *priv)
{
	struct fw_busy_info *fbusy_info = &priv->fw_busy_info;
	struct se_if_device_ctx *dev_ctx = NULL;
	unsigned long flags;

	scoped_guard(spinlock_irqsave, &fbusy_info->fw_busy_lock) {
		dev_ctx = fbusy_info->fw_busy_dev_ctx;
		fbusy_info->fw_busy_dev_ctx = NULL;

		if (!dev_ctx) {
			/*
			 * No parked context: nothing to recover. Clear fw_busy
			 * and return without reserving the interface, so the
			 * no-op path never leaves a dangling recovery
			 * reservation. The scoped_guard releases fw_busy_lock
			 * on this return.
			 */
			atomic_set(&fbusy_info->fw_busy, 0);
			return;
		}
	}

	/*
	 * A context is parked and its handles must be recovered. Keep
	 * fw_busy set (breaker stays closed to all third parties) and
	 * reserve the SE interface exclusively for this recovery flow by
	 * publishing the current task as msg_excl_owner. ele_msg_send_rcv()
	 * then lets only this task's teardown-close messages through and
	 * rejects everyone else with -EBUSY. The reservation is assigned here,
	 * from outside ele_msg_send_rcv(), and released with se_release_msg_if()
	 * at the end, after which the interface is available for general
	 * se_if_cmd_lock message exchange.
	 *
	 * The scoped_guard above has already dropped fw_busy_lock before this
	 * se_reserve_msg_if() call: the reserve helper takes msg_excl_lock, and
	 * taking it while still holding fw_busy_lock would introduce a new
	 * fw_busy_lock -> msg_excl_lock nesting. The brief fw_busy == 1 /
	 * owner == NULL window that this opens is harmless - the breaker is
	 * fully closed, so every caller (including a would-be re-arm) is
	 * rejected with -EBUSY.
	 */
	se_reserve_msg_if(priv);

	scoped_guard(mutex, &dev_ctx->fops_lock) {
		/*
		 * Snapshot any orphaned late FW response. On the teardown
		 * path se_if_probe_cleanup calls se_clear_fw_busy before
		 * cancel_work_sync, so fw_busy is still 1 here and a late
		 * IRQ can write orphan_fw_rx_msg concurrently - take
		 * clbk_rx_lock. On the workqueue path the IRQ writer has
		 * already finished; the lock is a no-contention formality.
		 * Call fw_api_specific_ops() outside the spinlock since it
		 * may sleep.
		 */
		u8 late_rx_snap[MAX_ALLOWED_RX_MSG_SZ];
		bool have_snap;

		spin_lock_irqsave(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);
		have_snap = fbusy_info->orphan_fw_rx_msg[0] != 0;
		if (have_snap) {
			memcpy(late_rx_snap, fbusy_info->orphan_fw_rx_msg,
			       sizeof(late_rx_snap));
			memset(fbusy_info->orphan_fw_rx_msg, 0,
			       sizeof(fbusy_info->orphan_fw_rx_msg));
		}
		spin_unlock_irqrestore(&priv->waiting_rsp_clbk_hdl.clbk_rx_lock, flags);

		if (have_snap) {
			/*
			 * FW responded late. DMA staging buffer is no longer
			 * being written - safe to reclaim. Close any firmware
			 * resource handle carried in the response.
			 */
			fw_api_specific_ops(priv->priv_dev_ctx,
					    (struct se_api_msg *)late_rx_snap, true);

			if (dev_ctx == priv->priv_dev_ctx) {
				/*
				 * Internal context: probe-time static DMA buf;
				 * se_if_probe_cleanup reclaims it explicitly.
				 * Just reset logical pos and return gen_pool
				 * loan buffers for reuse.
				 */
				if (priv->mem_pool)
					se_cleanup_mem_pool_buf(dev_ctx, true);
				se_dev_ctx_shared_mem_cleanup(dev_ctx);
			} else if (dev_ctx->cleanup_done) {
				/*
				 * Userspace fd already closed while fw_busy was
				 * armed (e.g. SIGKILL). FW has now responded;
				 * close deferred handles and free the DMA buf.
				 *
				 * cleanup_dev_ctx() already freed dev_ctx->devname
				 * and set it to NULL, so use the stable snapshot
				 * captured at arm time (fbusy_info->devname) for
				 * these diagnostics rather than dev_ctx->devname.
				 */
				if (dev_ctx->strg_hdl &&
				    se_close_storage(priv->priv_dev_ctx,
						     dev_ctx->strg_hdl))
					dev_err(priv->dev,
						"%s: failed to close deferred storage handle\n",
						fbusy_info->devname);
				if (dev_ctx->sess_hdl &&
				    se_close_session(priv->priv_dev_ctx,
						     dev_ctx->sess_hdl))
					dev_err(priv->dev,
						"%s: failed to close deferred session handle\n",
						fbusy_info->devname);
				dev_ctx->strg_hdl = 0;
				dev_ctx->sess_hdl = 0;
				cleanup_se_shared_mem(dev_ctx, true);
			} else {
				/* Pure timeout, fd still open: reset pos only. */
				se_dev_ctx_shared_mem_cleanup(dev_ctx);
			}
		} else {
			/*
			 * have_snap=false only on teardown (FW never responded,
			 * or teardown beat the late IRQ, which going_away then
			 * drops). priv_dev_ctx is handled by probe_cleanup's
			 * single cleanup_se_shared_mem(reclaim=true) after
			 * cancel_work_sync, so nothing to do here. For a
			 * userspace dev_ctx (cleanup_done already true), use
			 * reclaim=false: the pos gate leaks the buffer if FW may
			 * still be writing, else frees it. reclaim=true would
			 * also be safe here since the region is no-map.
			 */
			if (dev_ctx != priv->priv_dev_ctx && dev_ctx->cleanup_done)
				cleanup_se_shared_mem(dev_ctx, false);
		}
	}

	/*
	 * Recovery flow is done: release the exclusive reservation, then clear
	 * the breaker. se_release_msg_if() drops msg_excl_owner; the fw_busy
	 * clear below reopens the interface. Ordering is safe either way: while
	 * fw_busy is still 1 a third party is rejected regardless of owner, and
	 * once fw_busy is 0 the owner is no longer consulted.
	 */
	se_release_msg_if(priv);

	spin_lock_irqsave(&fbusy_info->fw_busy_lock, flags);
	atomic_set(&fbusy_info->fw_busy, 0);
	spin_unlock_irqrestore(&fbusy_info->fw_busy_lock, flags);
	kref_put(&dev_ctx->refcount, se_if_dev_ctx_release);
}

void unset_dev_ctx_as_command_receiver(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_api_msg *old_rx_msg = NULL;
	struct se_clbk_handle *se_clbk_hdl;
	unsigned long flags;

	lockdep_assert_held(&priv->modify_lock);

	se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;

	if (se_clbk_hdl->dev_ctx == dev_ctx) {
		spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
		old_rx_msg = se_clbk_hdl->rx_msg;
		se_clbk_hdl->dev_ctx = NULL;
		se_clbk_hdl->rx_msg = NULL;
		se_clbk_hdl->rx_msg_sz = 0;
		spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

		kfree(old_rx_msg);
		complete_all(&se_clbk_hdl->done);
	}
}

/*
 * check_cmd_rcvr_status() - check whether dev_ctx can become the command
 * receiver or is already become the command receiver.
 *
 * Returns:
 *   0        - dev_ctx is already the registered receiver
 *   -EBUSY   - another context is already the receiver
 *   -EINVAL  - dev_ctx has no storage handle
 *   -ENXIO   - ready to proceed: no receiver set, strg_hdl present
 *
 * Caller must hold priv->modify_lock.
 */
static int check_cmd_rcvr_status(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_clbk_handle *se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;

	lockdep_assert_held(&priv->modify_lock);

	if (se_clbk_hdl->dev_ctx == dev_ctx)
		return 0;

	if (se_clbk_hdl->dev_ctx)
		return -EBUSY;

	if (!dev_ctx->strg_hdl)
		return -EINVAL;

	/* Reaching here means, with a valid storage handle and command-receiver as NULL,
	 * either the registration process is to be done or failed.
	 */
	return -ENXIO;
}

int set_dev_ctx_as_command_receiver(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_clbk_handle *se_clbk_hdl = &priv->cmd_receiver_clbk_hdl;
	struct se_api_msg *new_rx_msg = NULL;
	unsigned long flags;
	int ret;

	guard(mutex)(&priv->modify_lock);

	/*
	 * All state checks happen inside modify_lock so the result cannot
	 * go stale between the check and the arming below.
	 */
	ret = check_cmd_rcvr_status(dev_ctx);
	if (ret != -ENXIO)
		return ret;

	if (!se_clbk_hdl->rx_msg) {
		new_rx_msg = kzalloc(MAX_NVM_MSG_LEN, GFP_KERNEL);
		if (!new_rx_msg)
			return -ENOMEM;
	}
	spin_lock_irqsave(&se_clbk_hdl->clbk_rx_lock, flags);
	if (new_rx_msg)
		se_clbk_hdl->rx_msg = new_rx_msg;
	reinit_completion(&se_clbk_hdl->done);
	se_clbk_hdl->rx_msg_sz = MAX_NVM_MSG_LEN;
	se_clbk_hdl->dev_ctx = dev_ctx;
	dev_ctx->rcv_msg_timeout_jiffies = MAX_SCHEDULE_TIMEOUT;
	spin_unlock_irqrestore(&se_clbk_hdl->clbk_rx_lock, flags);

	return 0;
}

static void dlink_dev_ctx(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;

	unset_dev_ctx_as_command_receiver(dev_ctx);

	if (!list_empty(&dev_ctx->link)) {
		list_del_init(&dev_ctx->link);
		priv->active_devctx_count--;
	}
}

bool se_is_fw_busy_ctx(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	struct fw_busy_info *fbusy_info = &priv->fw_busy_info;
	unsigned long flags;
	bool match;

	spin_lock_irqsave(&fbusy_info->fw_busy_lock, flags);
	match = fbusy_info->fw_busy_dev_ctx == dev_ctx;
	spin_unlock_irqrestore(&fbusy_info->fw_busy_lock, flags);

	return match;
}

static void cleanup_dev_ctx(struct se_if_device_ctx *dev_ctx, bool is_fclose)
{
	struct fw_busy_info *fbusy_info = &dev_ctx->priv->fw_busy_info;
	bool already_done;

	scoped_guard(mutex, &dev_ctx->fops_lock) {
		already_done = dev_ctx->cleanup_done;
		if (!already_done) {
			/*
			 * Ask FW to drop this context's session and storage so
			 * the kernel and FW stay in sync. Done here, under this
			 * context's fops_lock only (not the global modify_lock),
			 * because both close requests block on a firmware
			 * round-trip; issuing them while modify_lock was held
			 * would stall every other context for the FW timeout.
			 *
			 * Skip the round-trips once the FW path is marked busy.
			 * fw_busy is armed when a synchronous transaction times
			 * out; while it is set ele_msg_send_rcv() rejects further
			 * commands with -EBUSY without waiting. It is only cleared
			 * by se_clear_fw_busy(), which during unbind runs once
			 * after this loop (or earlier from fw_busy_work only if a
			 * genuine late FW response arrives). On a hung FW no late
			 * response comes, so the breaker stays set for the rest of
			 * the loop and the remaining closes would just return
			 * -EBUSY and log spurious "failed to close" errors. Skip
			 * them and emit a single warning instead.
			 */
			if (atomic_read(&fbusy_info->fw_busy)) {
				if (dev_ctx->strg_hdl || dev_ctx->sess_hdl)
					dev_warn(dev_ctx->priv->dev,
						 "%s: skipping session/storage close, FW is busy\n",
						 dev_ctx->devname);
			} else {
				/*
				 * Choose which dev_ctx sends the close messages.
				 * fclose: use the caller's own dev_ctx so a race with
				 * unbind is rejected with -ENODEV instead of hitting a
				 * freed tx_chan. Teardown: use priv_dev_ctx; going_away
				 * is set but the reservation (msg_excl_owner == current)
				 * lets these closes through while tx_chan is still live.
				 */
				struct se_if_device_ctx *tx_ctx = is_fclose ? dev_ctx :
							dev_ctx->priv->priv_dev_ctx;

				if (dev_ctx->strg_hdl &&
				    se_close_storage(tx_ctx, dev_ctx->strg_hdl))
					dev_err(dev_ctx->priv->dev, "failed to close storage.\n");
				if (dev_ctx->sess_hdl &&
				    se_close_session(tx_ctx, dev_ctx->sess_hdl))
					dev_err(dev_ctx->priv->dev, "failed to close session.\n");
			}
			/*
			 * fw_busy is caused by one timed-out synchronous transaction.
			 * Only that transaction's dev_ctx may still have coherent
			 * memory referenced by FW. Do not skip cleanup for unrelated
			 * contexts while fw_busy is set.
			 */
			if (se_is_fw_busy_ctx(dev_ctx))
				dev_warn(dev_ctx->priv->dev,
					 "%s: deferring shared memory cleanup while FW is busy\n",
					 dev_ctx->devname);
			else
				cleanup_se_shared_mem(dev_ctx, true);

			kfree(dev_ctx->devname);
			dev_ctx->devname = NULL;
			dev_ctx->cleanup_done = true;
		}
	}

	if (is_fclose)
		kref_put(&dev_ctx->refcount, se_if_dev_ctx_release);
}

static void dlink_n_cleanup_dev_ctx(struct se_if_device_ctx *dev_ctx, bool is_fclose)
{
	struct se_if_priv *priv = dev_ctx->priv;

	if (is_fclose) {
		scoped_guard(mutex, &priv->modify_lock)
			dlink_dev_ctx(dev_ctx);
	}

	cleanup_dev_ctx(dev_ctx, is_fclose);
}

static int init_device_context(struct se_if_priv *priv, int ch_id,
			       struct se_if_device_ctx **new_dev_ctx)
{
	struct se_if_device_ctx *dev_ctx;
	int ret = 0;

	dev_ctx = kzalloc_obj(*dev_ctx);

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
	kref_init(&dev_ctx->refcount);
	dev_ctx->priv = priv;
	dev_ctx->cleanup_done = false;
	INIT_LIST_HEAD(&dev_ctx->link);
	set_se_rcv_msg_timeout(dev_ctx, SE_RCV_MSG_LONG_TIMEOUT_MS);
	*new_dev_ctx = dev_ctx;

	ret = init_se_shared_mem(dev_ctx);
	if (ret < 0) {
		kfree(dev_ctx->devname);
		kfree(dev_ctx);
		*new_dev_ctx = NULL;

		return ret;
	}

	/* Take a reference to priv for this device context */
	kref_get(&priv->refcount);

	scoped_guard(mutex, &priv->modify_lock) {
		list_add_tail(&dev_ctx->link, &priv->dev_ctx_list);
		priv->active_devctx_count++;
	}

	return ret;
}

static int se_ioctl_cmd_snd_rcv_cleanup(struct se_if_device_ctx *dev_ctx, void __user *uarg,
					struct se_ioctl_cmd_snd_rcv_rsp_info *cmd_snd_rcv_rsp_info)
{
	/* shared memory is allocated before this IOCTL */
	se_dev_ctx_shared_mem_cleanup(dev_ctx);

	if (cmd_snd_rcv_rsp_info->rx_buf_sz &&
	    copy_to_user(uarg, cmd_snd_rcv_rsp_info, sizeof(*cmd_snd_rcv_rsp_info))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy cmd_snd_rcv_rsp_info to user.\n",
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
	int rsp_status_err = 0;
	int act_rx_msg_sz = 0;
	int cleanup_err = 0;
	int err = 0;

	if (copy_from_user(&cmd_snd_rcv_rsp_info, uarg,
			   sizeof(cmd_snd_rcv_rsp_info))) {
		dev_err(priv->dev,
			"%s: Failed to copy cmd_snd_rcv_rsp_info from user.",
			dev_ctx->devname);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -EFAULT;
	}

	if (cmd_snd_rcv_rsp_info.tx_buf_sz < SE_MU_HDR_SZ ||
	    cmd_snd_rcv_rsp_info.tx_buf_sz > MAX_ALLOWED_TX_MSG_SZ) {
		dev_err(priv->dev, "%s: User buffer too small/large(%d < %d)\n",
			dev_ctx->devname, cmd_snd_rcv_rsp_info.tx_buf_sz,
			cmd_snd_rcv_rsp_info.tx_buf_sz < SE_MU_HDR_SZ ? SE_MU_HDR_SZ :
								MAX_ALLOWED_TX_MSG_SZ);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -ENOSPC;
	}

	struct se_api_msg *tx_msg __free(kfree) =
		memdup_user(u64_to_user_ptr(cmd_snd_rcv_rsp_info.tx_buf),
			    cmd_snd_rcv_rsp_info.tx_buf_sz);
	if (IS_ERR(tx_msg)) {
		err = PTR_ERR(tx_msg);
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	err = se_chk_tx_cmd_msg_hdr(dev_ctx, &tx_msg->header,
				    cmd_snd_rcv_rsp_info.tx_buf_sz,
				    cmd_snd_rcv_rsp_info.rx_buf_sz);
	if (err) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return err;
	}

	if (cmd_snd_rcv_rsp_info.rx_buf_sz < SE_MU_HDR_SZ ||
	    cmd_snd_rcv_rsp_info.rx_buf_sz > MAX_ALLOWED_RX_MSG_SZ) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -EINVAL;
	}

	if (tx_msg->header.tag != priv->if_defs->cmd_tag) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -EINVAL;
	}

	if (tx_msg->header.ver == priv->if_defs->fw_api_ver &&
	    get_load_fw_instance(priv)->is_fw_tobe_loaded) {
		err = se_load_firmware(priv);
		if (err) {
			dev_err(priv->dev, "Could not send msg as FW is not loaded.\n");
			se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
			return -EPERM;
		}
	}

	struct se_api_msg *rx_msg __free(kfree) =
		kzalloc(cmd_snd_rcv_rsp_info.rx_buf_sz, GFP_KERNEL);
	if (!rx_msg) {
		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
		return -ENOMEM;
	}

	err = ele_msg_send_rcv(dev_ctx, tx_msg, cmd_snd_rcv_rsp_info.tx_buf_sz,
			       rx_msg, cmd_snd_rcv_rsp_info.rx_buf_sz, &act_rx_msg_sz);
	if (err < 0) {
		/*
		 * -ERESTARTSYS here means the wait was interrupted by a signal
		 * after the command had already been handed to - and executed
		 * by - the firmware, with its response delivered into rx_msg
		 * (ele_msg_send_rcv() converts only a positive, i.e. successfully
		 * received, result to -ERESTARTSYS). If that response carried a
		 * freshly allocated session/storage handle, record it now via
		 * fw_api_specific_ops(): the handle is already live in firmware,
		 * so leaving it untracked would stop cleanup_dev_ctx() from ever
		 * closing it and leak the firmware resource. Validate the
		 * delivered response first, using its own declared length bounded
		 * by the caller's buffer, so a truncated or malformed reply is
		 * not acted upon.
		 */
		if (err == -ERESTARTSYS) {
			u32 rsp_sz = rx_msg->header.size << 2;

			if (rsp_sz && rsp_sz <= cmd_snd_rcv_rsp_info.rx_buf_sz &&
			    !se_val_rsp_hdr_n_status(dev_ctx, rx_msg,
						     tx_msg->header.command, act_rx_msg_sz,
						     tx_msg->header.ver)) {
				se_dev_ctx_cpy_out_data(dev_ctx);
				fw_api_specific_ops(dev_ctx, rx_msg, true);
			}
			/*
			 * NOTE: se_dev_ctx_cpy_out_data() above has already
			 * copied the firmware response payload to userspace before
			 * this point. Returning -EINTR here is intentional, not
			 * -ERESTARTSYS: the VFS would transparently restart the
			 * ioctl on -ERESTARTSYS, re-issuing the command with
			 * already-zeroed shared input buffers. -EINTR prevents
			 * auto-restart and lets userspace enter its signal handler
			 * to decide whether to reissue the command.
			 * See Documentation/driver-api/firmware/other_interfaces.rst,
			 * section "Signal handling after a completed hardware
			 * operation".
			 */
			err = -EINTR;
		}

		se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);

		return err;
	}

	/*
	 * ele_msg_send_rcv() returns a positive received-message size on
	 * success. Returning that raw size as the ioctl result would make a
	 * successful transaction look like a positive (non-zero) return value
	 * to userspace. Record the actual received size in rx_buf_sz for the
	 * response copied back to userspace, then normalise err to 0 so the
	 * ioctl reports plain success; the firmware status is conveyed to
	 * userspace inside the response buffer itself.
	 */
	cmd_snd_rcv_rsp_info.rx_buf_sz = act_rx_msg_sz;
	err = 0;

	dev_dbg(priv->dev, "%s: %s %s.\n", dev_ctx->devname, __func__,
		"message received, start transmit to user");

	rsp_status_err =
		se_val_rsp_hdr_n_status(dev_ctx, rx_msg, tx_msg->header.command,
					act_rx_msg_sz, tx_msg->header.ver);

	if (!rsp_status_err) {
		/*
		 * For msg IDs handled by fw_api_specific_ops(), the exact
		 * response size was already ensured in ele_uapi_allowed_fw_cmd().
		 */
		err = fw_api_specific_ops(dev_ctx, rx_msg, false);
		if (err) {
			se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
			return err;
		}

		err = se_dev_ctx_cpy_out_data(dev_ctx);
		if (err < 0) {
			se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);
			return err;
		}
	}

	/* Copy data from the buffer */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4, rx_msg,
			     cmd_snd_rcv_rsp_info.rx_buf_sz, false);

	if (copy_to_user(u64_to_user_ptr(cmd_snd_rcv_rsp_info.rx_buf), rx_msg,
			 cmd_snd_rcv_rsp_info.rx_buf_sz)) {
		dev_err(priv->dev, "%s: Failed to copy to user.\n", dev_ctx->devname);
		err = -EFAULT;
	}

	cleanup_err = se_ioctl_cmd_snd_rcv_cleanup(dev_ctx, uarg, &cmd_snd_rcv_rsp_info);

	if (cleanup_err && !err)
		err = cleanup_err;

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

	dev_dbg(priv->dev, "%s: info [se_if_id: %d, irq_idx: %d, tz: 0x%x, did: 0x%x].\n",
		dev_ctx->devname, if_info.se_if_id, if_info.interrupt_idx, if_info.tz,
		if_info.did);

	if (copy_to_user(uarg, &if_info, sizeof(if_info))) {
		dev_err(priv->dev, "%s: Failed to copy mu info to user.\n",
			dev_ctx->devname);
		err = -EFAULT;
	}

	return err;
}

static void rollback_shared_mem_pos(struct se_if_device_ctx *dev_ctx, u32 length)
{
	struct se_shared_mem *shared_mem = NULL;

	shared_mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;

	if (WARN_ON_ONCE(length > shared_mem->pos)) {
		shared_mem->pos = 0;
		return;
	}

	shared_mem->pos -= length;
}

int get_shared_mem_slot(struct se_if_device_ctx *dev_ctx,
			u32 *length, dma_addr_t *ele_dma_addr, void **ptr)
{
	struct se_shared_mem *shared_mem = NULL;
	bool is_fw_busy_dev_ctx;
	size_t aligned_len = 0;
	u32 pos;

	/*
	 * If this context is the one that caused a firmware timeout the shared
	 * DMA buffers may still be actively read/written by the firmware.
	 */
	is_fw_busy_dev_ctx = se_is_fw_busy_ctx(dev_ctx);
	if (is_fw_busy_dev_ctx)
		return -EBUSY;

	aligned_len = round_up((size_t)*length, 8);
	if (aligned_len < *length) {
		dev_err(dev_ctx->priv->dev, "%s: Invalid buffer length.\n",
			dev_ctx->devname);
		return -EINVAL;
	}

	/* No specific requirement for this buffer. */
	shared_mem = &dev_ctx->se_shared_mem_mgmt.non_secure_mem;

	/* Check there is enough space in the shared memory. */
	dev_dbg(dev_ctx->priv->dev, "%s: req_size = %zd, max_size= %d, curr_pos = %d\n",
		dev_ctx->devname, aligned_len, shared_mem->size,
		shared_mem->pos);

	if (shared_mem->size < shared_mem->pos ||
	    aligned_len > (shared_mem->size - shared_mem->pos)) {
		dev_err(dev_ctx->priv->dev, "%s: Not enough space in shared memory.\n",
			dev_ctx->devname);
		return -ENOMEM;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;
	shared_mem->pos += aligned_len;
	*ele_dma_addr = (u64)shared_mem->dma_addr + pos;
	*ptr = shared_mem->ptr + pos;
	*length = aligned_len;

	memset(shared_mem->ptr + pos, 0, aligned_len);

	return 0;
}

/*
 * Copy a buffer of data to/from the user and return the address to use in
 * messages
 */
static int se_ioctl_setup_iobuf_handler(struct se_if_device_ctx *dev_ctx,
					void __user *uarg)
{
	struct se_ioctl_setup_iobuf io = {0};
	struct se_buf_desc *b_desc = NULL;
	void *dma_buf_ptr = NULL;
	dma_addr_t ele_dma_addr;
	u32 aligned_len = 0;
	int err = 0;

	if (copy_from_user(&io, uarg, sizeof(io))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed copy iobuf config from user.\n",
			dev_ctx->devname);
		return -EFAULT;
	}

	dev_dbg(dev_ctx->priv->dev, "%s: io [buf: %p(%d) flag: %x].\n", dev_ctx->devname,
		u64_to_user_ptr(io.user_buf), io.length, io.flags);

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

	aligned_len = io.length;
	err = get_shared_mem_slot(dev_ctx, &aligned_len, &ele_dma_addr, &dma_buf_ptr);
	if (err)
		return err;

	io.ele_addr = ele_dma_addr;
	if ((io.flags & SE_IO_BUF_FLAGS_IS_INPUT) ||
	    (io.flags & SE_IO_BUF_FLAGS_IS_IN_OUT)) {
		/*
		 * buffer is input:
		 * copy data from user space to this allocated buffer.
		 */
		if (copy_from_user(dma_buf_ptr, u64_to_user_ptr(io.user_buf),
				   io.length)) {
			dev_err(dev_ctx->priv->dev,
				"%s: Failed copy data to shared memory.",
				dev_ctx->devname);
			err = -EFAULT;
			goto rollback;
		}
	}

	b_desc = add_b_desc_to_pending_list(dma_buf_ptr, &io, dev_ctx);
	if (IS_ERR(b_desc)) {
		err = PTR_ERR(b_desc);
		dev_err(dev_ctx->priv->dev, "%s: Failed to allocate/link b_desc.\n",
			dev_ctx->devname);
		goto rollback;
	}

copy:
	/* Provide the EdgeLock Enclave address to user space only if success.*/
	if (copy_to_user(uarg, &io, sizeof(io))) {
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy iobuff setup to user.\n",
			dev_ctx->devname);
		err = -EFAULT;
		goto rollback;
	}
	return err;

rollback:
	if (!IS_ERR_OR_NULL(b_desc)) {
		list_del(&b_desc->link);
		kfree(b_desc);
	}

	if (dma_buf_ptr && aligned_len) {
		memset(dma_buf_ptr, 0, aligned_len);
		rollback_shared_mem_pos(dev_ctx, aligned_len);
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
		dev_err(dev_ctx->priv->dev, "%s: Failed to copy soc info to user.\n",
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
	struct se_if_priv *priv;
	int err;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &dev_ctx->fops_lock) {
		if (dev_ctx->cleanup_done)
			return -ENODEV;

		priv = dev_ctx->priv;

		dev_dbg(priv->dev, "%s: write from buf (%p)%zu, ppos=%lld.\n", dev_ctx->devname,
			buf, size, ((ppos) ? *ppos : 0));

		if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx) {
			se_dev_ctx_shared_mem_cleanup(dev_ctx);
			return -EINVAL;
		}

		if (size < SE_MU_HDR_SZ || size > MAX_ALLOWED_TX_MSG_SZ) {
			dev_err(priv->dev, "%s: User buffer too small/large(%zu < %d)\n",
				dev_ctx->devname, size,
				size < SE_MU_HDR_SZ ? SE_MU_HDR_SZ :
								MAX_ALLOWED_TX_MSG_SZ);
			return -ENOSPC;
		}

		struct se_api_msg *tx_msg __free(kfree) = memdup_user(buf, size);
		if (IS_ERR(tx_msg))
			return PTR_ERR(tx_msg);

		err = se_chk_tx_rsp_msg_hdr(dev_ctx, &tx_msg->header, size);
		if (err)
			return err;

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
	u8 rx_msg_snap[MAX_NVM_MSG_LEN] = {};
	char devname_snap[32] = {};
	struct se_if_priv *priv;
	unsigned long flags;
	size_t copy_len;
	int err;

	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &dev_ctx->fops_lock) {
		priv = dev_ctx->priv;

		if (dev_ctx->cleanup_done)
			return -ENODEV;

		/*
		 * Snapshot devname once while fops_lock is held. After the
		 * scoped guard releases the lock, a concurrent cleanup_dev_ctx()
		 * could free dev_ctx->devname before the error path below runs.
		 */
		strscpy(devname_snap, dev_ctx->devname, sizeof(devname_snap));

		dev_dbg(priv->dev, "%s: read to buf %p(%zu), ppos=%lld.\n", devname_snap,
			buf, size, ((ppos) ? *ppos : 0));

		mutex_lock(&priv->modify_lock);
		if (dev_ctx != priv->cmd_receiver_clbk_hdl.dev_ctx) {
			mutex_unlock(&priv->modify_lock);
			se_dev_ctx_shared_mem_cleanup(dev_ctx);
			return -EINVAL;
		}
		mutex_unlock(&priv->modify_lock);
	}

	err = ele_msg_rcv(dev_ctx, &priv->cmd_receiver_clbk_hdl);
	if (err < 0) {
		if (err != -ERESTARTSYS)
			dev_err(priv->dev,
				"%s: Er[0x%x]: Signal Interrupted. Current act-dev-ctx count: %d.",
				devname_snap, err, dev_ctx->priv->active_devctx_count);
		return err;
	}

	/*
	 * Reacquire fops_lock before touching any dev_ctx state (pending lists,
	 * rx_msg) after the blocking wait. fops_lock was dropped before calling
	 * ele_msg_rcv(). If cleanup_dev_ctx() ran concurrently it could have
	 * freed the DMA buffers and the pending lists, leading to UAF and list
	 * corruption. Re-checking cleanup_done under fops_lock prevents that.
	 */
	mutex_lock(&dev_ctx->fops_lock);

	if (dev_ctx->cleanup_done) {
		mutex_unlock(&dev_ctx->fops_lock);
		return -ENODEV;
	}

	/*
	 * Snapshot the whole rx_msg under modify_lock + clbk_rx_lock, not just
	 * copy_len bytes: fw_api_specific_ops() reads data words (e.g. strg_hdl
	 * at data[1]) beyond the userspace read size; truncating would record a
	 * zero handle. Run fw_api_specific_ops() OUTSIDE modify_lock
	 * (ELE_STORAGE_OPEN_REQ re-takes it, else deadlock).
	 */
	scoped_guard(mutex, &priv->modify_lock) {
		spin_lock_irqsave(&priv->cmd_receiver_clbk_hdl.clbk_rx_lock, flags);
		if (priv->cmd_receiver_clbk_hdl.dev_ctx != dev_ctx ||
		    !priv->cmd_receiver_clbk_hdl.rx_msg ||
		    !priv->cmd_receiver_clbk_hdl.rx_msg_sz) {
			spin_unlock_irqrestore(&priv->cmd_receiver_clbk_hdl.clbk_rx_lock, flags);
			mutex_unlock(&dev_ctx->fops_lock);
			return -ENODEV;
		}
		copy_len = min(size, (size_t)priv->cmd_receiver_clbk_hdl.rx_msg_sz);
		memcpy(rx_msg_snap, priv->cmd_receiver_clbk_hdl.rx_msg,
		       priv->cmd_receiver_clbk_hdl.rx_msg_sz);
		priv->cmd_receiver_clbk_hdl.rx_msg_sz = 0;
		spin_unlock_irqrestore(&priv->cmd_receiver_clbk_hdl.clbk_rx_lock, flags);

		/* We may need to copy the output data to user before
		 * delivering the completion message.
		 */
		err = se_dev_ctx_cpy_out_data(dev_ctx);
		if (err < 0) {
			se_dev_ctx_shared_mem_cleanup(dev_ctx);
			mutex_unlock(&dev_ctx->fops_lock);
			return err;
		}
	}

	/* fw_api_specific_ops() runs outside modify_lock; see comment above. */
	print_hex_dump_debug("to user ", DUMP_PREFIX_OFFSET, 4, 4,
			     rx_msg_snap, copy_len, false);

	cmd_receiver_specific_ops(dev_ctx, (struct se_api_msg *)rx_msg_snap);
	err = copy_len;
	if (copy_to_user(buf, rx_msg_snap, copy_len))
		err = -EFAULT;

	se_dev_ctx_shared_mem_cleanup(dev_ctx);
	mutex_unlock(&dev_ctx->fops_lock);

	return err;
}

/* Open a character device. */
static int se_if_fops_open(struct inode *nd, struct file *fp)
{
	struct miscdevice *miscdev = fp->private_data;
	struct se_if_open_gate *gate;
	struct se_if_device_ctx *misc_dev_ctx;
	struct se_if_device_ctx *dev_ctx;
	struct se_if_priv *priv;
	int err = 0;

	gate = container_of(miscdev, struct se_if_open_gate, miscdev);

	if (!se_if_open_gate_get(gate))
		return -ENODEV;

	if (mutex_lock_interruptible(&gate->lock)) {
		se_if_open_gate_put(gate);
		return -ERESTARTSYS;
	}

	if (gate->dying || !gate->priv ||
	    !kref_get_unless_zero(&gate->priv->refcount)) {
		mutex_unlock(&gate->lock);
		se_if_open_gate_put(gate);
		return -ENODEV;
	}

	priv = gate->priv;
	mutex_unlock(&gate->lock);

	misc_dev_ctx = priv->priv_dev_ctx;

	if (mutex_lock_interruptible(&misc_dev_ctx->fops_lock)) {
		err = -ERESTARTSYS;
		goto out_put_priv;
	}

	if (misc_dev_ctx->cleanup_done) {
		err = -ENODEV;
		goto out_unlock_misc;
	}

	priv->dev_ctx_mono_count++;
	err = init_device_context(priv, priv->dev_ctx_mono_count, &dev_ctx);
	if (err) {
		dev_err(priv->dev, "Failed to create dev-ctx.\n");
		goto out_unlock_misc;
	}

	fp->private_data = dev_ctx;

out_unlock_misc:
	mutex_unlock(&misc_dev_ctx->fops_lock);
out_put_priv:
	kref_put(&priv->refcount, se_if_priv_release);
	se_if_open_gate_put(gate);
	return err;
}

/* Close a character device. */
static int se_if_fops_close(struct inode *nd, struct file *fp)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;

	dlink_n_cleanup_dev_ctx(dev_ctx, true);

	return 0;
}

/* IOCTL entry point of a character device */
static long se_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv;
	void __user *uarg = (void __user *)arg;
	long err;

	/* Prevent race during change of device context */
	scoped_cond_guard(mutex_intr, return -ERESTARTSYS, &dev_ctx->fops_lock) {
		if (dev_ctx->cleanup_done)
			return -ENODEV;

		priv = dev_ctx->priv;

		switch (cmd) {
		case SE_IOCTL_CHECK_CMD_RCV_REG_STATUS: {
			guard(mutex)(&priv->modify_lock);
			err = check_cmd_rcvr_status(dev_ctx);
		break;
		}
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
			err = -ENOTTY;
			dev_dbg(priv->dev, "%s: IOCTL %.8x not supported.\n",
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
	.compat_ioctl   = compat_ptr_ioctl,
	.read		= se_if_fops_read,
	.write		= se_if_fops_write,
};

int se_get_mem_pool_buf(struct se_if_device_ctx *dev_ctx, void **buf,
			dma_addr_t *daddr, u32 len)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_buf_desc *b_desc = NULL;

	lockdep_assert_held(&dev_ctx->fops_lock);

	if (se_is_fw_busy_ctx(dev_ctx))
		return -EBUSY;

	b_desc = kzalloc_obj(*b_desc);
	if (!b_desc)
		return -ENOMEM;

	/*
	 * gen_pool is internally thread-safe, so contexts may allocate
	 * concurrently. The buffer is tracked on this context's own
	 * mem_pool_buf_list and released on its cleanup path.
	 */
	*buf = gen_pool_dma_alloc(priv->mem_pool, len, daddr);
	if (!*buf) {
		dev_err(priv->dev, "Failed to alloc from gen_pool.\n");
		kfree(b_desc);
		return -ENOMEM;
	}

	/* gen_pool_dma_alloc() does not zero the buffer. */
	memset(*buf, 0, len);
	b_desc->shared_buf_ptr = *buf;
	b_desc->size = len;

	list_add_tail(&b_desc->link, &se_shared_mem_mgmt->mem_pool_buf_list);

	return 0;
}

void se_cleanup_mem_pool_buf(struct se_if_device_ctx *dev_ctx, bool reclaim)
{
	struct se_shared_mem_mgmt_info *se_shared_mem_mgmt = &dev_ctx->se_shared_mem_mgmt;
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_buf_desc *b_desc, *temp;

	/*
	 * Free only the buffers this context allocated. A context that never
	 * used the pool has an empty list, so this is a no-op for it.
	 *
	 * Unlike the coherent staging buffer, the pool path needs no
	 * "nothing staged" (pos) gate on the reclaim=false leg. Pool buffers
	 * are ephemeral, per-transaction allocations: se_get_mem_pool_buf()
	 * refuses to allocate once the context is fw_busy, ele_msg_send_rcv()
	 * refuses to start a new command while fw_busy, and the success path
	 * frees the whole list via se_cleanup_mem_pool_buf(reclaim=true)
	 * before returning. se_if_cmd_lock serialises synchronous commands, so
	 * at most one transaction is outstanding. The only way to reach here
	 * with reclaim=false and a non-empty list is the single fw_busy
	 * context still owning the buffer(s) from the one timed-out
	 * transaction. Those buffers are exactly the in-flight ones the
	 * enclave may still be DMA-ing into, so leaving them on the list (no
	 * gen_pool_free) deliberately leaks them to avoid a DMA-after-free -
	 * there are no already-consumed pool buffers to reclaim on this leg.
	 */
	list_for_each_entry_safe(b_desc, temp, &se_shared_mem_mgmt->mem_pool_buf_list, link) {
		if (reclaim)
			gen_pool_free(priv->mem_pool,
				      (unsigned long)b_desc->shared_buf_ptr,
				      b_desc->size);
		list_del(&b_desc->link);
		kfree(b_desc);
	}
}

static void se_fw_busy_work(struct work_struct *work)
{
	struct fw_busy_info *fbusy_info =
		container_of(work, struct fw_busy_info, fw_busy_work);
	struct se_if_priv *priv =
		container_of(fbusy_info, struct se_if_priv, fw_busy_info);

	se_clear_fw_busy(priv);
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
