// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2024, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <drm/gpu_scheduler.h>
#include <linux/cleanup.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/pci.h>
#include <linux/xarray.h>

#include "aie2_msg_priv.h"
#include "aie2_pci.h"
#include "aie2_solver.h"
#include "amdxdna_ctx.h"
#include "amdxdna_gem.h"
#include "amdxdna_mailbox.h"
#include "amdxdna_pci_drv.h"
#include "amdxdna_pm.h"

static int aie2_max_col = XRS_MAX_COL;
module_param(aie2_max_col, uint, 0600);
MODULE_PARM_DESC(aie2_max_col, "Maximum column could be used");

/*
 * The management mailbox channel is allocated by firmware.
 * The related register and ring buffer information is on SRAM BAR.
 * This struct is the register layout.
 */
#define MGMT_MBOX_MAGIC 0x55504e5f /* _NPU */
struct mgmt_mbox_chann_info {
	__u32	x2i_tail;
	__u32	x2i_head;
	__u32	x2i_buf;
	__u32	x2i_buf_sz;
	__u32	i2x_tail;
	__u32	i2x_head;
	__u32	i2x_buf;
	__u32	i2x_buf_sz;
	__u32	magic;
	__u32	msi_id;
	__u32	prot_major;
	__u32	prot_minor;
	__u32	rsvd[4];
};

static int aie2_check_protocol(struct amdxdna_dev_hdl *ndev, u32 fw_major, u32 fw_minor)
{
	struct amdxdna_dev *xdna = ndev->xdna;

	/*
	 * The driver supported mailbox behavior is defined by
	 * ndev->priv->protocol_major and protocol_minor.
	 *
	 * When protocol_major and fw_major are different, it means driver
	 * and firmware are incompatible.
	 */
	if (ndev->priv->protocol_major != fw_major) {
		XDNA_ERR(xdna, "Incompatible firmware protocol major %d minor %d",
			 fw_major, fw_minor);
		return -EINVAL;
	}

	/*
	 * When protocol_minor is greater then fw_minor, that means driver
	 * relies on operation the installed firmware does not support.
	 */
	if (ndev->priv->protocol_minor > fw_minor) {
		XDNA_ERR(xdna, "Firmware minor version smaller than supported");
		return -EINVAL;
	}
	return 0;
}

static void aie2_dump_chann_info_debug(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->xdna;

	XDNA_DBG(xdna, "i2x tail    0x%x", ndev->mgmt_i2x.mb_tail_ptr_reg);
	XDNA_DBG(xdna, "i2x head    0x%x", ndev->mgmt_i2x.mb_head_ptr_reg);
	XDNA_DBG(xdna, "i2x ringbuf 0x%x", ndev->mgmt_i2x.rb_start_addr);
	XDNA_DBG(xdna, "i2x rsize   0x%x", ndev->mgmt_i2x.rb_size);
	XDNA_DBG(xdna, "x2i tail    0x%x", ndev->mgmt_x2i.mb_tail_ptr_reg);
	XDNA_DBG(xdna, "x2i head    0x%x", ndev->mgmt_x2i.mb_head_ptr_reg);
	XDNA_DBG(xdna, "x2i ringbuf 0x%x", ndev->mgmt_x2i.rb_start_addr);
	XDNA_DBG(xdna, "x2i rsize   0x%x", ndev->mgmt_x2i.rb_size);
	XDNA_DBG(xdna, "x2i chann index 0x%x", ndev->mgmt_chan_idx);
	XDNA_DBG(xdna, "mailbox protocol major 0x%x", ndev->mgmt_prot_major);
	XDNA_DBG(xdna, "mailbox protocol minor 0x%x", ndev->mgmt_prot_minor);
}

static int aie2_get_mgmt_chann_info(struct amdxdna_dev_hdl *ndev)
{
	struct mgmt_mbox_chann_info info_regs;
	struct xdna_mailbox_chann_res *i2x;
	struct xdna_mailbox_chann_res *x2i;
	u32 addr, off;
	u32 *reg;
	int ret;
	int i;

	/*
	 * Once firmware is alive, it will write management channel
	 * information in SRAM BAR and write the address of that information
	 * at FW_ALIVE_OFF offset in SRMA BAR.
	 *
	 * Read a non-zero value from FW_ALIVE_OFF implies that firmware
	 * is alive.
	 */
	ret = readx_poll_timeout(readl, SRAM_GET_ADDR(ndev, FW_ALIVE_OFF),
				 addr, addr, AIE2_INTERVAL, AIE2_TIMEOUT);
	if (ret || !addr)
		return -ETIME;

	off = AIE2_SRAM_OFF(ndev, addr);
	reg = (u32 *)&info_regs;
	for (i = 0; i < sizeof(info_regs) / sizeof(u32); i++)
		reg[i] = readl(ndev->sram_base + off + i * sizeof(u32));

	if (info_regs.magic != MGMT_MBOX_MAGIC) {
		XDNA_ERR(ndev->xdna, "Invalid mbox magic 0x%x", info_regs.magic);
		ret = -EINVAL;
		goto done;
	}

	i2x = &ndev->mgmt_i2x;
	x2i = &ndev->mgmt_x2i;

	i2x->mb_head_ptr_reg = AIE2_MBOX_OFF(ndev, info_regs.i2x_head);
	i2x->mb_tail_ptr_reg = AIE2_MBOX_OFF(ndev, info_regs.i2x_tail);
	i2x->rb_start_addr   = AIE2_SRAM_OFF(ndev, info_regs.i2x_buf);
	i2x->rb_size         = info_regs.i2x_buf_sz;

	x2i->mb_head_ptr_reg = AIE2_MBOX_OFF(ndev, info_regs.x2i_head);
	x2i->mb_tail_ptr_reg = AIE2_MBOX_OFF(ndev, info_regs.x2i_tail);
	x2i->rb_start_addr   = AIE2_SRAM_OFF(ndev, info_regs.x2i_buf);
	x2i->rb_size         = info_regs.x2i_buf_sz;

	ndev->mgmt_chan_idx  = info_regs.msi_id;
	ndev->mgmt_prot_major = info_regs.prot_major;
	ndev->mgmt_prot_minor = info_regs.prot_minor;

	ret = aie2_check_protocol(ndev, ndev->mgmt_prot_major, ndev->mgmt_prot_minor);

done:
	aie2_dump_chann_info_debug(ndev);

	/* Must clear address at FW_ALIVE_OFF */
	writel(0, SRAM_GET_ADDR(ndev, FW_ALIVE_OFF));

	return ret;
}

int aie2_runtime_cfg(struct amdxdna_dev_hdl *ndev,
		     enum rt_config_category category, u32 *val)
{
	const struct rt_config *cfg;
	u32 value;
	int ret;

	for (cfg = ndev->priv->rt_config; cfg->type; cfg++) {
		if (cfg->category != category)
			continue;

		value = val ? *val : cfg->value;
		ret = aie2_set_runtime_cfg(ndev, cfg->type, value);
		if (ret) {
			XDNA_ERR(ndev->xdna, "Set type %d value %d failed",
				 cfg->type, value);
			return ret;
		}
	}

	return 0;
}

static int aie2_xdna_reset(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie2_suspend_fw(ndev);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Suspend firmware failed");
		return ret;
	}

	ret = aie2_resume_fw(ndev);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Resume firmware failed");
		return ret;
	}

	return 0;
}

static int aie2_mgmt_fw_init(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie2_runtime_cfg(ndev, AIE2_RT_CFG_INIT, NULL);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Runtime config failed");
		return ret;
	}

	ret = aie2_assign_mgmt_pasid(ndev, 0);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Can not assign PASID");
		return ret;
	}

	ret = aie2_xdna_reset(ndev);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Reset firmware failed");
		return ret;
	}

	return 0;
}

static int aie2_mgmt_fw_query(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie2_query_firmware_version(ndev, &ndev->xdna->fw_ver);
	if (ret) {
		XDNA_ERR(ndev->xdna, "query firmware version failed");
		return ret;
	}

	ret = aie2_query_aie_version(ndev, &ndev->version);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Query AIE version failed");
		return ret;
	}

	ret = aie2_query_aie_metadata(ndev, &ndev->metadata);
	if (ret) {
		XDNA_ERR(ndev->xdna, "Query AIE metadata failed");
		return ret;
	}

	ndev->total_col = min(aie2_max_col, ndev->metadata.cols);

	return 0;
}

static void aie2_mgmt_fw_fini(struct amdxdna_dev_hdl *ndev)
{
	if (aie2_suspend_fw(ndev))
		XDNA_ERR(ndev->xdna, "Suspend_fw failed");
	XDNA_DBG(ndev->xdna, "Firmware suspended");
}

static int aie2_xrs_load(void *cb_arg, struct xrs_action_load *action)
{
	struct amdxdna_hwctx *hwctx = cb_arg;
	struct amdxdna_dev *xdna;
	int ret;

	xdna = hwctx->client->xdna;

	hwctx->start_col = action->part.start_col;
	hwctx->num_col = action->part.ncols;
	ret = aie2_create_context(xdna->dev_handle, hwctx);
	if (ret)
		XDNA_ERR(xdna, "create context failed, ret %d", ret);

	return ret;
}

static int aie2_xrs_unload(void *cb_arg)
{
	struct amdxdna_hwctx *hwctx = cb_arg;
	struct amdxdna_dev *xdna;
	int ret;

	xdna = hwctx->client->xdna;

	ret = aie2_destroy_context(xdna->dev_handle, hwctx);
	if (ret)
		XDNA_ERR(xdna, "destroy context failed, ret %d", ret);

	return ret;
}

static int aie2_xrs_set_dft_dpm_level(struct drm_device *ddev, u32 dpm_level)
{
	struct amdxdna_dev *xdna = to_xdna_dev(ddev);
	struct amdxdna_dev_hdl *ndev;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	ndev = xdna->dev_handle;
	ndev->dft_dpm_level = dpm_level;
	if (ndev->pw_mode != POWER_MODE_DEFAULT || ndev->dpm_level == dpm_level)
		return 0;

	return ndev->priv->hw_ops.set_dpm(ndev, dpm_level);
}

static struct xrs_action_ops aie2_xrs_actions = {
	.load = aie2_xrs_load,
	.unload = aie2_xrs_unload,
	.set_dft_dpm_level = aie2_xrs_set_dft_dpm_level,
};

static void aie2_hw_stop(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;

	if (ndev->dev_status <= AIE2_DEV_INIT) {
		XDNA_ERR(xdna, "device is already stopped");
		return;
	}

	aie2_mgmt_fw_fini(ndev);
	xdna_mailbox_stop_channel(ndev->mgmt_chann);
	xdna_mailbox_destroy_channel(ndev->mgmt_chann);
	ndev->mgmt_chann = NULL;
	drmm_kfree(&xdna->ddev, ndev->mbox);
	ndev->mbox = NULL;
	aie2_psp_stop(ndev->psp_hdl);
	aie2_smu_fini(ndev);
	aie2_error_async_events_free(ndev);
	pci_disable_device(pdev);

	ndev->dev_status = AIE2_DEV_INIT;
}

static int aie2_hw_start(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct xdna_mailbox_res mbox_res;
	u32 xdna_mailbox_intr_reg;
	int mgmt_mb_irq, ret;

	if (ndev->dev_status >= AIE2_DEV_START) {
		XDNA_INFO(xdna, "device is already started");
		return 0;
	}

	ret = pci_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "failed to enable device, ret %d", ret);
		return ret;
	}
	pci_set_master(pdev);

	ret = aie2_smu_init(ndev);
	if (ret) {
		XDNA_ERR(xdna, "failed to init smu, ret %d", ret);
		goto disable_dev;
	}

	ret = aie2_psp_start(ndev->psp_hdl);
	if (ret) {
		XDNA_ERR(xdna, "failed to start psp, ret %d", ret);
		goto fini_smu;
	}

	ret = aie2_get_mgmt_chann_info(ndev);
	if (ret) {
		XDNA_ERR(xdna, "firmware is not alive");
		goto stop_psp;
	}

	mbox_res.ringbuf_base = ndev->sram_base;
	mbox_res.ringbuf_size = pci_resource_len(pdev, xdna->dev_info->sram_bar);
	mbox_res.mbox_base = ndev->mbox_base;
	mbox_res.mbox_size = MBOX_SIZE(ndev);
	mbox_res.name = "xdna_mailbox";
	ndev->mbox = xdnam_mailbox_create(&xdna->ddev, &mbox_res);
	if (!ndev->mbox) {
		XDNA_ERR(xdna, "failed to create mailbox device");
		ret = -ENODEV;
		goto stop_psp;
	}

	mgmt_mb_irq = pci_irq_vector(pdev, ndev->mgmt_chan_idx);
	if (mgmt_mb_irq < 0) {
		ret = mgmt_mb_irq;
		XDNA_ERR(xdna, "failed to alloc irq vector, ret %d", ret);
		goto stop_psp;
	}

	xdna_mailbox_intr_reg = ndev->mgmt_i2x.mb_head_ptr_reg + 4;
	ndev->mgmt_chann = xdna_mailbox_create_channel(ndev->mbox,
						       &ndev->mgmt_x2i,
						       &ndev->mgmt_i2x,
						       xdna_mailbox_intr_reg,
						       mgmt_mb_irq);
	if (!ndev->mgmt_chann) {
		XDNA_ERR(xdna, "failed to create management mailbox channel");
		ret = -EINVAL;
		goto stop_psp;
	}

	ret = aie2_pm_init(ndev);
	if (ret) {
		XDNA_ERR(xdna, "failed to init pm, ret %d", ret);
		goto destroy_mgmt_chann;
	}

	ret = aie2_mgmt_fw_init(ndev);
	if (ret) {
		XDNA_ERR(xdna, "initial mgmt firmware failed, ret %d", ret);
		goto destroy_mgmt_chann;
	}

	ret = aie2_mgmt_fw_query(ndev);
	if (ret) {
		XDNA_ERR(xdna, "failed to query fw, ret %d", ret);
		goto destroy_mgmt_chann;
	}

	ret = aie2_error_async_events_alloc(ndev);
	if (ret) {
		XDNA_ERR(xdna, "Allocate async events failed, ret %d", ret);
		goto destroy_mgmt_chann;
	}

	ndev->dev_status = AIE2_DEV_START;

	return 0;

destroy_mgmt_chann:
	xdna_mailbox_stop_channel(ndev->mgmt_chann);
	xdna_mailbox_destroy_channel(ndev->mgmt_chann);
stop_psp:
	aie2_psp_stop(ndev->psp_hdl);
fini_smu:
	aie2_smu_fini(ndev);
disable_dev:
	pci_disable_device(pdev);

	return ret;
}

static int aie2_hw_suspend(struct amdxdna_dev *xdna)
{
	struct amdxdna_client *client;

	guard(mutex)(&xdna->dev_lock);
	list_for_each_entry(client, &xdna->client_list, node)
		aie2_hwctx_suspend(client);

	aie2_hw_stop(xdna);

	return 0;
}

static int aie2_hw_resume(struct amdxdna_dev *xdna)
{
	struct amdxdna_client *client;
	int ret;

	ret = aie2_hw_start(xdna);
	if (ret) {
		XDNA_ERR(xdna, "Start hardware failed, %d", ret);
		return ret;
	}

	list_for_each_entry(client, &xdna->client_list, node) {
		ret = aie2_hwctx_resume(client);
		if (ret)
			break;
	}

	return ret;
}

static int aie2_init(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	void __iomem *tbl[PCI_NUM_RESOURCES] = {0};
	struct init_config xrs_cfg = { 0 };
	struct amdxdna_dev_hdl *ndev;
	struct psp_config psp_conf;
	const struct firmware *fw;
	unsigned long bars = 0;
	int i, nvec, ret;

	ndev = drmm_kzalloc(&xdna->ddev, sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	ndev->priv = xdna->dev_info->dev_priv;
	ndev->xdna = xdna;

	ret = request_firmware(&fw, ndev->priv->fw_path, &pdev->dev);
	if (ret) {
		XDNA_ERR(xdna, "failed to request_firmware %s, ret %d",
			 ndev->priv->fw_path, ret);
		return ret;
	}

	ret = pcim_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "pcim enable device failed, ret %d", ret);
		goto release_fw;
	}

	for (i = 0; i < PSP_MAX_REGS; i++)
		set_bit(PSP_REG_BAR(ndev, i), &bars);

	set_bit(xdna->dev_info->sram_bar, &bars);
	set_bit(xdna->dev_info->smu_bar, &bars);
	set_bit(xdna->dev_info->mbox_bar, &bars);

	for (i = 0; i < PCI_NUM_RESOURCES; i++) {
		if (!test_bit(i, &bars))
			continue;
		tbl[i] = pcim_iomap(pdev, i, 0);
		if (!tbl[i]) {
			XDNA_ERR(xdna, "map bar %d failed", i);
			ret = -ENOMEM;
			goto release_fw;
		}
	}

	ndev->sram_base = tbl[xdna->dev_info->sram_bar];
	ndev->smu_base = tbl[xdna->dev_info->smu_bar];
	ndev->mbox_base = tbl[xdna->dev_info->mbox_bar];

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		XDNA_ERR(xdna, "Failed to set DMA mask: %d", ret);
		goto release_fw;
	}

	nvec = pci_msix_vec_count(pdev);
	if (nvec <= 0) {
		XDNA_ERR(xdna, "does not get number of interrupt vector");
		ret = -EINVAL;
		goto release_fw;
	}

	ret = pci_alloc_irq_vectors(pdev, nvec, nvec, PCI_IRQ_MSIX);
	if (ret < 0) {
		XDNA_ERR(xdna, "failed to alloc irq vectors, ret %d", ret);
		goto release_fw;
	}

	psp_conf.fw_size = fw->size;
	psp_conf.fw_buf = fw->data;
	for (i = 0; i < PSP_MAX_REGS; i++)
		psp_conf.psp_regs[i] = tbl[PSP_REG_BAR(ndev, i)] + PSP_REG_OFF(ndev, i);
	ndev->psp_hdl = aie2m_psp_create(&xdna->ddev, &psp_conf);
	if (!ndev->psp_hdl) {
		XDNA_ERR(xdna, "failed to create psp");
		ret = -ENOMEM;
		goto release_fw;
	}
	xdna->dev_handle = ndev;

	ret = aie2_hw_start(xdna);
	if (ret) {
		XDNA_ERR(xdna, "start npu failed, ret %d", ret);
		goto release_fw;
	}

	xrs_cfg.clk_list.num_levels = ndev->max_dpm_level + 1;
	for (i = 0; i < xrs_cfg.clk_list.num_levels; i++)
		xrs_cfg.clk_list.cu_clk_list[i] = ndev->priv->dpm_clk_tbl[i].hclk;
	xrs_cfg.sys_eff_factor = 1;
	xrs_cfg.ddev = &xdna->ddev;
	xrs_cfg.actions = &aie2_xrs_actions;
	xrs_cfg.total_col = ndev->total_col;

	xdna->xrs_hdl = xrsm_init(&xrs_cfg);
	if (!xdna->xrs_hdl) {
		XDNA_ERR(xdna, "Initialize resolver failed");
		ret = -EINVAL;
		goto stop_hw;
	}

	release_firmware(fw);
	amdxdna_pm_init(xdna);
	return 0;

stop_hw:
	aie2_hw_stop(xdna);
release_fw:
	release_firmware(fw);

	return ret;
}

static void aie2_fini(struct amdxdna_dev *xdna)
{
	amdxdna_pm_fini(xdna);
	aie2_hw_stop(xdna);
}

static int aie2_get_aie_status(struct amdxdna_client *client,
			       struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_aie_status status;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;
	int ret;

	ndev = xdna->dev_handle;
	if (copy_from_user(&status, u64_to_user_ptr(args->buffer), sizeof(status))) {
		XDNA_ERR(xdna, "Failed to copy AIE request into kernel");
		return -EFAULT;
	}

	if (ndev->metadata.cols * ndev->metadata.size < status.buffer_size) {
		XDNA_ERR(xdna, "Invalid buffer size. Given Size: %u. Need Size: %u.",
			 status.buffer_size, ndev->metadata.cols * ndev->metadata.size);
		return -EINVAL;
	}

	ret = aie2_query_status(ndev, u64_to_user_ptr(status.buffer),
				status.buffer_size, &status.cols_filled);
	if (ret) {
		XDNA_ERR(xdna, "Failed to get AIE status info. Ret: %d", ret);
		return ret;
	}

	if (copy_to_user(u64_to_user_ptr(args->buffer), &status, sizeof(status))) {
		XDNA_ERR(xdna, "Failed to copy AIE request info to user space");
		return -EFAULT;
	}

	return 0;
}

static int aie2_get_aie_metadata(struct amdxdna_client *client,
				 struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_aie_metadata *meta;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;
	int ret = 0;

	ndev = xdna->dev_handle;
	meta = kzalloc(sizeof(*meta), GFP_KERNEL);
	if (!meta)
		return -ENOMEM;

	meta->col_size = ndev->metadata.size;
	meta->cols = ndev->metadata.cols;
	meta->rows = ndev->metadata.rows;

	meta->version.major = ndev->metadata.version.major;
	meta->version.minor = ndev->metadata.version.minor;

	meta->core.row_count = ndev->metadata.core.row_count;
	meta->core.row_start = ndev->metadata.core.row_start;
	meta->core.dma_channel_count = ndev->metadata.core.dma_channel_count;
	meta->core.lock_count = ndev->metadata.core.lock_count;
	meta->core.event_reg_count = ndev->metadata.core.event_reg_count;

	meta->mem.row_count = ndev->metadata.mem.row_count;
	meta->mem.row_start = ndev->metadata.mem.row_start;
	meta->mem.dma_channel_count = ndev->metadata.mem.dma_channel_count;
	meta->mem.lock_count = ndev->metadata.mem.lock_count;
	meta->mem.event_reg_count = ndev->metadata.mem.event_reg_count;

	meta->shim.row_count = ndev->metadata.shim.row_count;
	meta->shim.row_start = ndev->metadata.shim.row_start;
	meta->shim.dma_channel_count = ndev->metadata.shim.dma_channel_count;
	meta->shim.lock_count = ndev->metadata.shim.lock_count;
	meta->shim.event_reg_count = ndev->metadata.shim.event_reg_count;

	if (copy_to_user(u64_to_user_ptr(args->buffer), meta, sizeof(*meta)))
		ret = -EFAULT;

	kfree(meta);
	return ret;
}

static int aie2_get_aie_version(struct amdxdna_client *client,
				struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_aie_version version;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;

	ndev = xdna->dev_handle;
	version.major = ndev->version.major;
	version.minor = ndev->version.minor;

	if (copy_to_user(u64_to_user_ptr(args->buffer), &version, sizeof(version)))
		return -EFAULT;

	return 0;
}

static int aie2_get_firmware_version(struct amdxdna_client *client,
				     struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_firmware_version version;
	struct amdxdna_dev *xdna = client->xdna;

	version.major = xdna->fw_ver.major;
	version.minor = xdna->fw_ver.minor;
	version.patch = xdna->fw_ver.sub;
	version.build = xdna->fw_ver.build;

	if (copy_to_user(u64_to_user_ptr(args->buffer), &version, sizeof(version)))
		return -EFAULT;

	return 0;
}

static int aie2_get_power_mode(struct amdxdna_client *client,
			       struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_get_power_mode mode = {};
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;

	ndev = xdna->dev_handle;
	mode.power_mode = ndev->pw_mode;

	if (copy_to_user(u64_to_user_ptr(args->buffer), &mode, sizeof(mode)))
		return -EFAULT;

	return 0;
}

static int aie2_get_clock_metadata(struct amdxdna_client *client,
				   struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_clock_metadata *clock;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;
	int ret = 0;

	ndev = xdna->dev_handle;
	clock = kzalloc(sizeof(*clock), GFP_KERNEL);
	if (!clock)
		return -ENOMEM;

	snprintf(clock->mp_npu_clock.name, sizeof(clock->mp_npu_clock.name),
		 "MP-NPU Clock");
	clock->mp_npu_clock.freq_mhz = ndev->npuclk_freq;
	snprintf(clock->h_clock.name, sizeof(clock->h_clock.name), "H Clock");
	clock->h_clock.freq_mhz = ndev->hclk_freq;

	if (copy_to_user(u64_to_user_ptr(args->buffer), clock, sizeof(*clock)))
		ret = -EFAULT;

	kfree(clock);
	return ret;
}

static int aie2_hwctx_status_cb(struct amdxdna_hwctx *hwctx, void *arg)
{
	struct amdxdna_drm_hwctx_entry *tmp __free(kfree) = NULL;
	struct amdxdna_drm_get_array *array_args = arg;
	struct amdxdna_drm_hwctx_entry __user *buf;
	u32 size;

	if (!array_args->num_element)
		return -EINVAL;

	tmp = kzalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	tmp->pid = hwctx->client->pid;
	tmp->context_id = hwctx->id;
	tmp->start_col = hwctx->start_col;
	tmp->num_col = hwctx->num_col;
	tmp->command_submissions = hwctx->priv->seq;
	tmp->command_completions = hwctx->priv->completed;
	tmp->pasid = hwctx->client->pasid;
	tmp->priority = hwctx->qos.priority;
	tmp->gops = hwctx->qos.gops;
	tmp->fps = hwctx->qos.fps;
	tmp->dma_bandwidth = hwctx->qos.dma_bandwidth;
	tmp->latency = hwctx->qos.latency;
	tmp->frame_exec_time = hwctx->qos.frame_exec_time;
	tmp->state = AMDXDNA_HWCTX_STATE_ACTIVE;

	buf = u64_to_user_ptr(array_args->buffer);
	size = min(sizeof(*tmp), array_args->element_size);

	if (copy_to_user(buf, tmp, size))
		return -EFAULT;

	array_args->buffer += size;
	array_args->num_element--;

	return 0;
}

static int aie2_get_hwctx_status(struct amdxdna_client *client,
				 struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_get_array array_args;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_client *tmp_client;
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	array_args.element_size = sizeof(struct amdxdna_drm_query_hwctx);
	array_args.buffer = args->buffer;
	array_args.num_element = args->buffer_size / array_args.element_size;
	list_for_each_entry(tmp_client, &xdna->client_list, node) {
		ret = amdxdna_hwctx_walk(tmp_client, &array_args,
					 aie2_hwctx_status_cb);
		if (ret)
			break;
	}

	args->buffer_size -= (u32)(array_args.buffer - args->buffer);
	return ret;
}

static int aie2_get_info(struct amdxdna_client *client, struct amdxdna_drm_get_info *args)
{
	struct amdxdna_dev *xdna = client->xdna;
	int ret, idx;

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	ret = amdxdna_pm_resume_get(xdna);
	if (ret)
		goto dev_exit;

	switch (args->param) {
	case DRM_AMDXDNA_QUERY_AIE_STATUS:
		ret = aie2_get_aie_status(client, args);
		break;
	case DRM_AMDXDNA_QUERY_AIE_METADATA:
		ret = aie2_get_aie_metadata(client, args);
		break;
	case DRM_AMDXDNA_QUERY_AIE_VERSION:
		ret = aie2_get_aie_version(client, args);
		break;
	case DRM_AMDXDNA_QUERY_CLOCK_METADATA:
		ret = aie2_get_clock_metadata(client, args);
		break;
	case DRM_AMDXDNA_QUERY_HW_CONTEXTS:
		ret = aie2_get_hwctx_status(client, args);
		break;
	case DRM_AMDXDNA_QUERY_FIRMWARE_VERSION:
		ret = aie2_get_firmware_version(client, args);
		break;
	case DRM_AMDXDNA_GET_POWER_MODE:
		ret = aie2_get_power_mode(client, args);
		break;
	default:
		XDNA_ERR(xdna, "Not supported request parameter %u", args->param);
		ret = -EOPNOTSUPP;
	}

	amdxdna_pm_suspend_put(xdna);
	XDNA_DBG(xdna, "Got param %d", args->param);

dev_exit:
	drm_dev_exit(idx);
	return ret;
}

static int aie2_query_ctx_status_array(struct amdxdna_client *client,
				       struct amdxdna_drm_get_array *args)
{
	struct amdxdna_drm_get_array array_args;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_client *tmp_client;
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	if (args->element_size > SZ_4K || args->num_element > SZ_1K) {
		XDNA_DBG(xdna, "Invalid element size %d or number of element %d",
			 args->element_size, args->num_element);
		return -EINVAL;
	}

	array_args.element_size = min(args->element_size,
				      sizeof(struct amdxdna_drm_hwctx_entry));
	array_args.buffer = args->buffer;
	array_args.num_element = args->num_element * args->element_size /
				array_args.element_size;
	list_for_each_entry(tmp_client, &xdna->client_list, node) {
		ret = amdxdna_hwctx_walk(tmp_client, &array_args,
					 aie2_hwctx_status_cb);
		if (ret)
			break;
	}

	args->element_size = array_args.element_size;
	args->num_element = (u32)((array_args.buffer - args->buffer) /
				  args->element_size);

	return ret;
}

static int aie2_get_array(struct amdxdna_client *client,
			  struct amdxdna_drm_get_array *args)
{
	struct amdxdna_dev *xdna = client->xdna;
	int ret, idx;

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	ret = amdxdna_pm_resume_get(xdna);
	if (ret)
		goto dev_exit;

	switch (args->param) {
	case DRM_AMDXDNA_HW_CONTEXT_ALL:
		ret = aie2_query_ctx_status_array(client, args);
		break;
	default:
		XDNA_ERR(xdna, "Not supported request parameter %u", args->param);
		ret = -EOPNOTSUPP;
	}

	amdxdna_pm_suspend_put(xdna);
	XDNA_DBG(xdna, "Got param %d", args->param);

dev_exit:
	drm_dev_exit(idx);
	return ret;
}

static int aie2_set_power_mode(struct amdxdna_client *client,
			       struct amdxdna_drm_set_state *args)
{
	struct amdxdna_drm_set_power_mode power_state;
	enum amdxdna_power_mode_type power_mode;
	struct amdxdna_dev *xdna = client->xdna;

	if (copy_from_user(&power_state, u64_to_user_ptr(args->buffer),
			   sizeof(power_state))) {
		XDNA_ERR(xdna, "Failed to copy power mode request into kernel");
		return -EFAULT;
	}

	if (XDNA_MBZ_DBG(xdna, power_state.pad, sizeof(power_state.pad)))
		return -EINVAL;

	power_mode = power_state.power_mode;
	if (power_mode > POWER_MODE_TURBO) {
		XDNA_ERR(xdna, "Invalid power mode %d", power_mode);
		return -EINVAL;
	}

	return aie2_pm_set_mode(xdna->dev_handle, power_mode);
}

static int aie2_set_state(struct amdxdna_client *client,
			  struct amdxdna_drm_set_state *args)
{
	struct amdxdna_dev *xdna = client->xdna;
	int ret, idx;

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	ret = amdxdna_pm_resume_get(xdna);
	if (ret)
		goto dev_exit;

	switch (args->param) {
	case DRM_AMDXDNA_SET_POWER_MODE:
		ret = aie2_set_power_mode(client, args);
		break;
	default:
		XDNA_ERR(xdna, "Not supported request parameter %u", args->param);
		ret = -EOPNOTSUPP;
		break;
	}

	amdxdna_pm_suspend_put(xdna);
dev_exit:
	drm_dev_exit(idx);
	return ret;
}

const struct amdxdna_dev_ops aie2_ops = {
	.init = aie2_init,
	.fini = aie2_fini,
	.resume = aie2_hw_resume,
	.suspend = aie2_hw_suspend,
	.get_aie_info = aie2_get_info,
	.set_aie_state = aie2_set_state,
	.hwctx_init = aie2_hwctx_init,
	.hwctx_fini = aie2_hwctx_fini,
	.hwctx_config = aie2_hwctx_config,
	.cmd_submit = aie2_cmd_submit,
	.hmm_invalidate = aie2_hmm_invalidate,
	.get_array = aie2_get_array,
};
