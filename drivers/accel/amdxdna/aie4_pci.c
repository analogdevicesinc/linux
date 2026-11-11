// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <linux/debugfs.h>
#include <linux/firmware.h>
#include <linux/sizes.h>

#include "aie.h"
#include "aie4_msg_priv.h"
#include "aie4_pci.h"
#include "amdxdna_ctx.h"
#include "amdxdna_error.h"
#include "amdxdna_mailbox.h"
#include "amdxdna_mailbox_helper.h"
#include "amdxdna_pci_drv.h"
#include "amdxdna_pm.h"

#define NO_IOHUB		0
#define PSP_NOTIFY_INTR		0xD007BE11
#define AIE4_TOTAL_COLUMN	3

/*
 * The management mailbox channel is allocated by firmware.
 * The related register and ring buffer information is on SRAM BAR.
 * This struct is the register layout.
 */
struct mailbox_info {
	__u32 valid;
	__u32 protocol_major;
	__u32 protocol_minor;
	__u32 x2i_tail_offset;
	__u32 x2i_head_offset;
	__u32 x2i_buffer_addr;
	__u32 x2i_buffer_size;
	__u32 i2x_tail_offset;
	__u32 i2x_head_offset;
	__u32 i2x_buffer_addr;
	__u32 i2x_buffer_size;
	__u32 i2x_msi_idx;
	__u32 reserved[4];
};

static int aie4_fw_is_alive(struct amdxdna_dev *xdna)
{
	const struct amdxdna_dev_priv *npriv = xdna->dev_info->dev_priv;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	void __iomem *src;
	u32 fw_is_valid;
	int ret;

	src = ndev->rbuf_base + npriv->mbox_info_off;

	ret = readx_poll_timeout(readl, src + offsetof(struct mailbox_info, valid),
				 fw_is_valid, (fw_is_valid == 0x1),
				 AIE_INTERVAL, AIE_TIMEOUT);
	if (ret)
		XDNA_ERR(xdna, "fw_is_valid=%d after %d ms",
			 fw_is_valid, DIV_ROUND_CLOSEST(AIE_TIMEOUT, 1000000));

	return ret;
}

static void aie4_read_mbox_info(struct amdxdna_dev *xdna,
				struct mailbox_info *mbox_info)
{
	const struct amdxdna_dev_priv *npriv = xdna->dev_info->dev_priv;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	u32 *dst = (u32 *)mbox_info;
	u32 __iomem *src;
	int i;

	src = ndev->rbuf_base + npriv->mbox_info_off;

	for (i = 0; i < sizeof(*mbox_info) / sizeof(u32); i++)
		dst[i] = readl(&src[i]);
}

static int aie4_mailbox_info(struct amdxdna_dev *xdna,
			     struct mailbox_info *mbox_info)
{
	int ret;

	ret = aie4_fw_is_alive(xdna);
	if (ret)
		return ret;

	aie4_read_mbox_info(xdna, mbox_info);

	ret = aie_check_protocol(&xdna->dev_handle->aie,
				 mbox_info->protocol_major,
				 mbox_info->protocol_minor,
				 xdna->dev_info->fw_feature_tbl);
	if (ret)
		XDNA_ERR(xdna, "mailbox major.minor %d.%d is not supported",
			 mbox_info->protocol_major, mbox_info->protocol_minor);

	return ret;
}

static void aie4_mailbox_fini(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	aie_destroy_chann(&ndev->aie, &ndev->aie.mgmt_chann);
	drmm_kfree(&xdna->ddev, ndev->mbox);
	ndev->mbox = NULL;
}

static irqreturn_t cert_comp_isr(int irq, void *p)
{
	struct cert_comp *cert_comp = p;

	wake_up_all(&cert_comp->waitq);
	return IRQ_HANDLED;
}

/* Wire per-cert completion notification interrupt. */
int aie4_request_notification(struct cert_comp *comp)
{
	struct pci_dev *pdev = to_pci_dev(comp->ndev->aie.xdna->ddev.dev);
	int ret;

	ret = pci_irq_vector(pdev, comp->msix_idx);
	if (ret < 0)
		return ret;
	comp->irq = ret;

	ret = request_irq(comp->irq, cert_comp_isr, 0, "xdna_hsa", comp);
	if (ret) {
		comp->irq = -ENOENT;
		return ret;
	}

	return 0;
}

/* Tear down per-cert completion notification interrupt. */
void aie4_free_notification(struct cert_comp *comp)
{
	if (comp->irq >= 0)
		free_irq(comp->irq, comp);
}

/* Validate and configure hardware context doorbell target. */
int aie4_doorbell_setup(struct amdxdna_hwctx *hwctx,
			const struct aie4_msg_create_hw_context_resp *resp)
{
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	u64 db_off = (u64)ndev->priv->doorbell_off + resp->doorbell_offset;

	/* Validate doorbell offset against mapped BAR bounds. */
	if (db_off + sizeof(u32) >
	    pci_resource_len(pdev, xdna->dev_info->doorbell_bar)) {
		XDNA_ERR(xdna, "doorbell offset 0x%llx out of BAR", db_off);
		return -EINVAL;
	}

	priv->doorbell_addr = ndev->doorbell_base + ndev->priv->doorbell_off +
			      resp->doorbell_offset;
	return 0;
}

/* Ring context doorbell to notify CERT. */
void aie4_doorbell_ring(struct amdxdna_hwctx *hwctx)
{
	writel(0, hwctx->priv->doorbell_addr);
}

static int aie4_irq_init(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret, nvec;

	nvec = pci_msix_vec_count(pdev);
	XDNA_DBG(xdna, "irq vectors:%d", nvec);
	if (nvec <= 0) {
		XDNA_ERR(xdna, "does not get number of interrupt vector");
		return -EINVAL;
	}

	ret = pci_alloc_irq_vectors(pdev, nvec, nvec, PCI_IRQ_MSIX);
	if (ret < 0) {
		XDNA_ERR(xdna, "failed to alloc irq vector, ret: %d", ret);
		return ret;
	}

	return 0;
}

static int aie4_mailbox_start(struct amdxdna_dev *xdna,
			      struct mailbox_info *mbi)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	const struct amdxdna_dev_priv *npriv = xdna->dev_info->dev_priv;
	struct xdna_mailbox_chann_res *i2x;
	struct xdna_mailbox_chann_res *x2i;
	int mgmt_mb_irq;
	int ret;

	struct xdna_mailbox_res mbox_res = {
		.ringbuf_base = ndev->rbuf_base,
		.ringbuf_size = pci_resource_len(pdev, npriv->mbox_rbuf_bar),
		.mbox_base = ndev->mbox_base,
		.mbox_size = pci_resource_len(pdev, npriv->mbox_bar),
		.name = "xdna_aie4_mailbox",
	};

	i2x = &ndev->aie.mgmt_i2x;
	x2i = &ndev->aie.mgmt_x2i;

	x2i->mb_head_ptr_reg = mbi->x2i_head_offset;
	x2i->mb_tail_ptr_reg = mbi->x2i_tail_offset;
	x2i->rb_start_addr = mbi->x2i_buffer_addr;
	x2i->rb_size = mbi->x2i_buffer_size;

	i2x->rb_start_addr = mbi->i2x_buffer_addr;
	i2x->rb_size = mbi->i2x_buffer_size;
	i2x->mb_head_ptr_reg = mbi->i2x_head_offset;
	i2x->mb_tail_ptr_reg = mbi->i2x_tail_offset;

	ndev->aie.mgmt_chan_idx = mbi->i2x_msi_idx;
	aie_dump_mgmt_chann_debug(&ndev->aie);

	ndev->mbox = xdnam_mailbox_create(&xdna->ddev, &mbox_res);
	if (!ndev->mbox) {
		XDNA_ERR(xdna, "failed to create mailbox device");
		return -ENODEV;
	}

	ndev->aie.mgmt_chann = xdna_mailbox_alloc_channel(ndev->mbox);
	if (!ndev->aie.mgmt_chann) {
		XDNA_ERR(xdna, "failed to alloc mailbox channel");
		return -ENODEV;
	}

	mgmt_mb_irq = pci_irq_vector(pdev, ndev->aie.mgmt_chan_idx);
	if (mgmt_mb_irq < 0) {
		XDNA_ERR(xdna, "failed to alloc irq vector, return %d", mgmt_mb_irq);
		ret = mgmt_mb_irq;
		goto free_channel;
	}

	/*
	 * At any given time, at most AMDXDNA_MAX_ASYNC_EVENT_BUFS async event
	 * messages plus 1 other management command can be unresponded.
	 */
	ret = xdna_mailbox_start_channel(ndev->aie.mgmt_chann,
					 &ndev->aie.mgmt_x2i,
					 &ndev->aie.mgmt_i2x,
					 NO_IOHUB,
					 mgmt_mb_irq,
					 AMDXDNA_MAX_ASYNC_EVENT_BUFS + 1);
	if (ret) {
		XDNA_ERR(xdna, "failed to start management mailbox channel");
		ret = -EINVAL;
		goto free_channel;
	}

	XDNA_DBG(xdna, "Mailbox management channel created");
	return 0;

free_channel:
	xdna_mailbox_free_channel(ndev->aie.mgmt_chann);
	ndev->aie.mgmt_chann = NULL;
	return ret;
}

static int aie4_mailbox_init(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct mailbox_info mbox_info;
	int ret;

	ret = aie4_mailbox_info(xdna, &mbox_info);
	if (ret)
		return ret;

	return aie4_mailbox_start(xdna, &mbox_info);
}

static void aie4_fw_stop(struct amdxdna_dev_hdl *ndev)
{
	aie_psp_stop(ndev->aie.psp_hdl);
	aie_smu_fini(ndev->aie.smu_hdl);
}

static int aie4_fw_start(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie_smu_init(ndev->aie.smu_hdl);
	if (ret) {
		XDNA_ERR(ndev->aie.xdna, "failed to init smu, ret %d", ret);
		return ret;
	}

	ret = aie_psp_start(ndev->aie.psp_hdl);
	if (ret) {
		XDNA_ERR(ndev->aie.xdna, "failed to start psp, ret %d", ret);
		aie_smu_fini(ndev->aie.smu_hdl);
	}

	return ret;
}

static int aie4_partition_init(struct amdxdna_dev_hdl *ndev)
{
	DECLARE_AIE_MSG(aie4_msg_create_partition, AIE4_MSG_OP_CREATE_PARTITION);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	req.partition_col_start = 0;
	req.partition_col_count = AIE4_TOTAL_COLUMN;
	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret) {
		XDNA_ERR(xdna, "partition init failed: %d", ret);
		return ret;
	}

	ndev->partition_id = resp.partition_id;
	return 0;
}

static void aie4_partition_fini(struct amdxdna_dev_hdl *ndev)
{
	DECLARE_AIE_MSG(aie4_msg_destroy_partition, AIE4_MSG_OP_DESTROY_PARTITION);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	req.partition_id = ndev->partition_id;
	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_ERR(xdna, "partition fini failed: %d", ret);
}

/* Verify CERT protocol compatibility before starting hardware. */
static int aie4_query_fw(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	ret = aie4_query_npu_firmware_version(ndev, &xdna->fw_ver);
	if (ret)
		return ret;

	ret = aie4_query_cert_firmware_version(ndev, &ndev->cert_version);
	if (ret)
		return ret;

	return 0;
}

/* Restore cached power mode override across hardware start. */
int aie4_restore_power_mode(struct amdxdna_dev_hdl *ndev)
{
	if (ndev->pw_mode == POWER_MODE_DEFAULT)
		return 0;

	return aie4_msg_set_power_mode(ndev, ndev->pw_mode);
}

static int aie4_config_fw(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie4_attach_work_buffer(ndev);
	if (ret)
		return ret;

	/* Best-effort tuning knob; failure is warned inside and does not fail hw start */
	ret = aie4_set_ctx_hysteresis(ndev, ndev->ctx_switch_hysteresis_us);
	if (ret == -ETIME)
		return ret;

	/*
	 * Give the firmware high performance DRAM to log into with less
	 * verbose level.
	 */
	if (ndev->fw_log_buf) {
		ret = aie4_start_fw_log(ndev, AIE4_FW_LOG_LEVEL_ERR);
		if (ret == -ETIME)
			return ret;
	}

	return 0;
}

static int aie4_setup_aie(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie4_query_aie_version(ndev, &ndev->aie.version);
	if (ret)
		return ret;

	ret = aie4_query_aie_metadata(ndev, &ndev->aie.metadata);
	if (ret)
		return ret;

	ndev->total_col = min_t(u32, AIE4_TOTAL_COLUMN, ndev->aie.metadata.cols);

	ret = aie4_init_dpm_freq_table(ndev);
	if (ret) {
		if (ret == -ETIME)
			return ret;
		/* if query dpm from fw failed, using default value */
		if (ndev->priv->hw_ops && ndev->priv->hw_ops->set_dpm)
			(void)ndev->priv->hw_ops->set_dpm(&ndev->aie, 0);
	}

	return aie4_partition_init(ndev);
}

static int aie4_pf_hw_start(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie4_fw_start(ndev);
	if (ret)
		return ret;

	ret = aie4_mailbox_init(ndev);
	if (ret)
		goto stop_fw;

	ret = aie4_query_fw(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_config_fw(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_restore_power_mode(ndev);
	if (ret)
		goto mbox_fini;

	return 0;

mbox_fini:
	aie4_mailbox_fini(ndev);
stop_fw:
	aie4_fw_stop(ndev);

	return ret;
}

static void aie4_pf_hw_stop(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	aie4_suspend_fw(ndev);
	aie4_mailbox_fini(ndev);
	aie4_fw_stop(ndev);
}

static int aie4_vf_hw_start(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie4_mailbox_init(ndev);
	if (ret)
		return ret;

	ret = aie4_query_fw(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_setup_aie(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_restore_power_mode(ndev);
	if (ret)
		goto partition_fini;

	return 0;

partition_fini:
	aie4_partition_fini(ndev);
mbox_fini:
	aie4_mailbox_fini(ndev);
	return ret;
}

static void aie4_vf_hw_stop(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	aie4_partition_fini(ndev);
	aie4_mailbox_fini(ndev);
}

static int aie4_classic_hw_start(struct amdxdna_dev_hdl *ndev)
{
	int ret;

	ret = aie4_fw_start(ndev);
	if (ret)
		return ret;

	ret = aie4_mailbox_init(ndev);
	if (ret)
		goto stop_fw;

	ret = aie4_query_fw(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_config_fw(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_setup_aie(ndev);
	if (ret)
		goto mbox_fini;

	ret = aie4_restore_power_mode(ndev);
	if (ret)
		goto partition_fini;

	return 0;

partition_fini:
	aie4_partition_fini(ndev);
mbox_fini:
	aie4_mailbox_fini(ndev);
stop_fw:
	aie4_fw_stop(ndev);

	return ret;
}

static void aie4_classic_hw_stop(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	aie4_partition_fini(ndev);
	aie4_suspend_fw(ndev);
	aie4_mailbox_fini(ndev);
	aie4_fw_stop(ndev);
}

static int aie4_request_firmware(struct amdxdna_dev_hdl *ndev,
				 const struct firmware **npufw,
				 const struct firmware **certfw)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	char fw_name[128];
	int ret;

	ret = snprintf(fw_name, sizeof(fw_name), "amdnpu/%04x_%02x/%s",
		       pdev->device, pdev->revision, ndev->priv->npufw_path);
	if (ret >= sizeof(fw_name)) {
		XDNA_ERR(xdna, "npu firmware path is truncated");
		return -EINVAL;
	}

	ret = request_firmware(npufw, fw_name, &pdev->dev);
	if (ret) {
		XDNA_ERR(xdna, "failed to request_firmware %s, ret %d", fw_name, ret);
		return ret;
	}

	ret = snprintf(fw_name, sizeof(fw_name), "amdnpu/%04x_%02x/%s",
		       pdev->device, pdev->revision, ndev->priv->certfw_path);
	if (ret >= sizeof(fw_name)) {
		XDNA_ERR(xdna, "cert firmware path is truncated");
		ret = -EINVAL;
		goto release_npufw;
	}

	ret = request_firmware(certfw, fw_name, &pdev->dev);
	if (ret) {
		XDNA_ERR(xdna, "failed to request_firmware %s, ret %d", fw_name, ret);
		goto release_npufw;
	}

	return 0;

release_npufw:
	release_firmware(*npufw);

	return ret;
}

static void aie4_release_firmware(struct amdxdna_dev_hdl *ndev,
				  const struct firmware *npufw,
				  const struct firmware *certfw)
{
	release_firmware(certfw);
	release_firmware(npufw);
}

static int aie4_prepare_firmware(struct amdxdna_dev_hdl *ndev,
				 const struct firmware *npufw,
				 const struct firmware *certfw,
				 void __iomem *tbl[PCI_NUM_RESOURCES])
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct smu_config smu_conf = {};
	struct psp_config psp_conf;
	int i;

	psp_conf.fw_size = npufw->size;
	psp_conf.fw_buf = npufw->data;
	psp_conf.certfw_size = certfw->size;
	psp_conf.certfw_buf = certfw->data;
	psp_conf.arg2_mask = ~0;
	psp_conf.notify_val = PSP_NOTIFY_INTR;
	for (i = 0; i < PSP_MAX_REGS; i++)
		psp_conf.psp_regs[i] = tbl[PSP_REG_BAR(ndev, i)] + PSP_REG_OFF(ndev, i);
	ndev->aie.psp_hdl = aiem_psp_create(&xdna->ddev, &psp_conf);
	if (!ndev->aie.psp_hdl) {
		XDNA_ERR(xdna, "failed to create psp");
		return -ENOMEM;
	}

	for (i = 0; i < SMU_MAX_REGS; i++)
		smu_conf.smu_regs[i] = tbl[SMU_REG_BAR(ndev, i)] + SMU_REG_OFF(ndev, i);
	ndev->aie.smu_hdl = aiem_smu_create(&xdna->ddev, &smu_conf);
	if (!ndev->aie.smu_hdl) {
		XDNA_ERR(xdna, "failed to create smu");
		return -ENOMEM;
	}

	return 0;
}

static int aie4_load_fw(struct amdxdna_dev_hdl *ndev,
			void __iomem *tbl[PCI_NUM_RESOURCES])
{
	const struct firmware *npufw, *certfw;
	int ret;

	if (!ndev->priv->npufw_path && !ndev->priv->certfw_path)
		return 0;

	ret = aie4_request_firmware(ndev, &npufw, &certfw);
	if (ret)
		return ret;

	ret = aie4_prepare_firmware(ndev, npufw, certfw, tbl);
	aie4_release_firmware(ndev, npufw, certfw);

	return ret;
}

static int aie4m_pcidev_init(struct amdxdna_dev *xdna)
{
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	struct amdxdna_dev_hdl *ndev;
	void __iomem *tbl[PCI_NUM_RESOURCES] = {0};
	unsigned long bars = 0;
	int ret, i;

	ndev = drmm_kzalloc(&xdna->ddev, sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	ndev->priv = xdna->dev_info->dev_priv;
	ndev->aie.xdna = xdna;
	ndev->ctx_switch_hysteresis_us = AIE4_CTX_HYSTERESIS_US;
	ndev->pw_mode = POWER_MODE_DEFAULT;
	xdna->dev_handle = ndev;

	xa_init_flags(&ndev->cert_comp_xa, XA_FLAGS_ALLOC);
	mutex_init(&ndev->cert_comp_lock);

	/* Enable managed PCI device */
	ret = pcim_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "pcim enable device failed, ret %d", ret);
		return ret;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		XDNA_ERR(xdna, "failed to set DMA mask to 64:%d", ret);
		return ret;
	}

	for (i = 0; i < PSP_MAX_REGS; i++)
		set_bit(PSP_REG_BAR(ndev, i), &bars);
	for (i = 0; i < SMU_MAX_REGS; i++)
		set_bit(SMU_REG_BAR(ndev, i), &bars);
	set_bit(xdna->dev_info->mbox_bar, &bars);
	set_bit(xdna->dev_info->sram_bar, &bars);
	set_bit(xdna->dev_info->doorbell_bar, &bars);

	for (i = 0; i < PCI_NUM_RESOURCES; i++) {
		if (!test_bit(i, &bars))
			continue;
		tbl[i] = pcim_iomap(pdev, i, 0);
		if (!tbl[i]) {
			XDNA_ERR(xdna, "map bar %d failed", i);
			return -ENOMEM;
		}
	}

	ndev->mbox_base = tbl[xdna->dev_info->mbox_bar];
	ndev->rbuf_base = tbl[xdna->dev_info->sram_bar];
	ndev->doorbell_base = tbl[xdna->dev_info->doorbell_bar];

	pci_set_master(pdev);

	ret = aie4_load_fw(ndev, tbl);
	if (ret)
		return ret;

	ret = aie4_irq_init(xdna);
	if (ret)
		return ret;

	amdxdna_vbnv_init(xdna);
	XDNA_DBG(xdna, "init finished");

	return 0;
}

static int aie4_get_power_mode(struct amdxdna_client *client,
			       struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_get_power_mode mode = {};
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;
	u32 buf_sz;

	ndev = xdna->dev_handle;
	mode.power_mode = ndev->pw_mode;

	buf_sz = min_t(u32, args->buffer_size, sizeof(mode));
	if (copy_to_user(u64_to_user_ptr(args->buffer), &mode, buf_sz))
		return -EFAULT;

	return 0;
}

static int aie4_query_clock_metadata(struct amdxdna_client *client,
				     struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_query_clock_metadata *clock;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev;
	int ret = 0;
	u32 buf_sz;

	ndev = xdna->dev_handle;
	clock = kzalloc_obj(*clock);
	if (!clock)
		return -ENOMEM;

	aie_update_counters(ndev);
	snprintf(clock->mp_npu_clock.name, sizeof(clock->mp_npu_clock.name),
		 "MP-NPU Clock");
	clock->mp_npu_clock.freq_mhz = ndev->aie.npuclk_freq;
	snprintf(clock->h_clock.name, sizeof(clock->h_clock.name), "H Clock");
	clock->h_clock.freq_mhz = ndev->aie.hclk_freq;

	buf_sz = min_t(u32, args->buffer_size, sizeof(*clock));
	if (copy_to_user(u64_to_user_ptr(args->buffer), clock, buf_sz))
		ret = -EFAULT;

	kfree(clock);
	return ret;
}

static int aie4_query_resource_info(struct amdxdna_client *client,
				    struct amdxdna_drm_get_info *args)
{
	struct amdxdna_drm_get_resource_info res_info = {};
	struct amdxdna_dev_hdl *ndev;
	struct amdxdna_dev *xdna;
	u32 buf_sz;

	xdna = client->xdna;
	ndev = xdna->dev_handle;

	aie_update_counters(ndev);
	res_info.npu_clk_max = ndev->dpm_clk_tbl[ndev->max_npuhclk_level].hclk;
	res_info.npu_tops_max = ndev->aie.max_tops;
	res_info.npu_tops_curr = ndev->aie.curr_tops;
	/* Context accounting is populated in later patches in the series. */

	buf_sz = min_t(u32, args->buffer_size, sizeof(res_info));
	if (copy_to_user(u64_to_user_ptr(args->buffer), &res_info, buf_sz))
		return -EFAULT;

	return 0;
}

static int aie4_get_info(struct amdxdna_client *client, struct amdxdna_drm_get_info *args)
{
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	int ret, idx;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	ret = amdxdna_pm_resume_get_locked(xdna);
	if (ret)
		goto dev_exit;

	switch (args->param) {
	case DRM_AMDXDNA_QUERY_AIE_METADATA:
		ret = amdxdna_get_metadata(&ndev->aie, client, args);
		break;
	case DRM_AMDXDNA_QUERY_AIE_VERSION:
		ret = amdxdna_get_aie_version(client, args, &ndev->aie.version);
		break;
	case DRM_AMDXDNA_QUERY_CLOCK_METADATA:
		ret = aie4_query_clock_metadata(client, args);
		break;
	case DRM_AMDXDNA_QUERY_FIRMWARE_VERSION:
		ret = amdxdna_get_firmware_version(client, args, &xdna->fw_ver);
		break;
	case DRM_AMDXDNA_GET_POWER_MODE:
		ret = aie4_get_power_mode(client, args);
		break;
	case DRM_AMDXDNA_QUERY_RESOURCE_INFO:
		ret = aie4_query_resource_info(client, args);
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

static int aie4_set_power_mode(struct amdxdna_client *client,
			       struct amdxdna_drm_set_state *args)
{
	struct amdxdna_drm_set_power_mode power_state = { 0 };
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	u8 power_mode;
	int ret;

	if (args->buffer_size < sizeof(power_state))
		return -EINVAL;

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

	ret = aie4_msg_set_power_mode(xdna->dev_handle, power_mode);
	if (ret)
		return ret;

	ndev->pw_mode = power_mode;
	return 0;
}

static int aie4_set_state(struct amdxdna_client *client,
			  struct amdxdna_drm_set_state *args)
{
	struct amdxdna_dev *xdna = client->xdna;
	int ret, idx;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	ret = amdxdna_pm_resume_get_locked(xdna);
	if (ret)
		goto dev_exit;

	switch (args->param) {
	case DRM_AMDXDNA_SET_POWER_MODE:
		ret = aie4_set_power_mode(client, args);
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

static int aie4_alloc_work_buffer(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	u32 buf_size = AIE4_WORK_BUFFER_MIN_SIZE;

	ndev->work_buf = amdxdna_alloc_msg_buffer(xdna, &buf_size,
						  &ndev->work_buf_addr);
	if (IS_ERR(ndev->work_buf)) {
		int ret = PTR_ERR(ndev->work_buf);

		XDNA_ERR(xdna, "Failed to alloc work buffer, size 0x%x",
			 AIE4_WORK_BUFFER_MIN_SIZE);
		ndev->work_buf = NULL;
		return ret;
	}

	ndev->work_buf_size = buf_size;
	XDNA_DBG(xdna, "Work buffer allocated: size 0x%x", buf_size);

	return 0;
}

static void aie4_free_work_buffer(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	if (!ndev->work_buf)
		return;

	amdxdna_free_msg_buffer(xdna, ndev->work_buf_size, ndev->work_buf,
				ndev->work_buf_addr);
	ndev->work_buf = NULL;
}

/*
 * Firmware logging is best effort: a device that cannot spare the buffer still
 * runs, it just does not log. Never fail hw start or probe on this.
 */
static void aie4_alloc_fw_log_buffer(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	u32 buf_size = AIE4_FW_LOG_BUF_SIZE;

	ndev->fw_log_buf = amdxdna_alloc_msg_buffer(xdna, &buf_size,
						    &ndev->fw_log_buf_addr);
	if (IS_ERR(ndev->fw_log_buf)) {
		XDNA_WARN(xdna, "Failed to alloc fw log buffer, size 0x%x",
			  AIE4_FW_LOG_BUF_SIZE);
		ndev->fw_log_buf = NULL;
		return;
	}

	ndev->fw_log_buf_size = buf_size;
	XDNA_DBG(xdna, "FW log buffer allocated: size 0x%x", buf_size);
}

/* Free logging buffer after firmware execution has stopped. */
static void aie4_free_fw_log_buffer(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	if (!ndev->fw_log_buf)
		return;

	amdxdna_free_msg_buffer(xdna, ndev->fw_log_buf_size, ndev->fw_log_buf,
				ndev->fw_log_buf_addr);
	ndev->fw_log_buf = NULL;
}

static int aie4_pf_init(struct amdxdna_dev *xdna)
{
	int ret;

	ret = aie4m_pcidev_init(xdna);
	if (ret)
		return ret;

	ret = aie4_alloc_work_buffer(xdna->dev_handle);
	if (ret)
		return ret;

	aie4_alloc_fw_log_buffer(xdna->dev_handle);

	ret = aie4_pf_hw_start(xdna->dev_handle);
	if (ret)
		goto free_work_buf;

	return 0;

free_work_buf:
	aie4_free_fw_log_buffer(xdna->dev_handle);
	aie4_free_work_buffer(xdna->dev_handle);
	return ret;
}

static int aie4_vf_init(struct amdxdna_dev *xdna)
{
	int ret;

	ret = aie4m_pcidev_init(xdna);
	if (ret)
		return ret;

	return aie4_vf_hw_start(xdna->dev_handle);
}

static int aie4_classic_init(struct amdxdna_dev *xdna)
{
	int ret;

	ret = aie4m_pcidev_init(xdna);
	if (ret)
		return ret;

	ret = aie4_alloc_work_buffer(xdna->dev_handle);
	if (ret)
		return ret;

	aie4_alloc_fw_log_buffer(xdna->dev_handle);

	ret = aie4_classic_hw_start(xdna->dev_handle);
	if (ret)
		goto free_work_buf;

	amdxdna_pm_init(xdna);
	return 0;

free_work_buf:
	aie4_free_fw_log_buffer(xdna->dev_handle);
	aie4_free_work_buffer(xdna->dev_handle);
	return ret;
}

static void aie4_pf_fini(struct amdxdna_dev *xdna)
{
	aie4_sriov_stop(xdna->dev_handle);
	aie4_pf_hw_stop(xdna->dev_handle);
	aie4_free_fw_log_buffer(xdna->dev_handle);
	aie4_free_work_buffer(xdna->dev_handle);
}

static void aie4_vf_fini(struct amdxdna_dev *xdna)
{
	aie4_vf_hw_stop(xdna->dev_handle);
}

static void aie4_classic_fini(struct amdxdna_dev *xdna)
{
	amdxdna_pm_fini(xdna);
	aie4_classic_hw_stop(xdna->dev_handle);
	aie4_free_fw_log_buffer(xdna->dev_handle);
	aie4_free_work_buffer(xdna->dev_handle);
}

static int aie4_ctx_hysteresis_get(void *data, u64 *val)
{
	struct amdxdna_dev_hdl *ndev = data;
	struct amdxdna_dev *xdna = ndev->aie.xdna;

	guard(mutex)(&xdna->dev_lock);
	*val = ndev->ctx_switch_hysteresis_us;

	return 0;
}

static int aie4_ctx_hysteresis_set(void *data, u64 val)
{
	struct amdxdna_dev_hdl *ndev = data;
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret, idx;

	if (val > U32_MAX)
		return -EINVAL;

	if (!drm_dev_enter(&xdna->ddev, &idx))
		return -ENODEV;

	mutex_lock(&xdna->dev_lock);

	ret = amdxdna_pm_resume_get_locked(xdna);
	if (ret)
		goto unlock;

	ret = aie4_set_ctx_hysteresis(ndev, (u32)val);
	if (!ret)
		ndev->ctx_switch_hysteresis_us = (u32)val;

	amdxdna_pm_suspend_put(xdna);

unlock:
	mutex_unlock(&xdna->dev_lock);
	drm_dev_exit(idx);

	return ret;
}

/* Context switch hysteresis timeout in microseconds; 0 disables hysteresis. */
DEFINE_DEBUGFS_ATTRIBUTE(aie4_ctx_hysteresis_fops, aie4_ctx_hysteresis_get,
			 aie4_ctx_hysteresis_set, "%llu\n");

static void aie4_debugfs_init(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;

	/* Context switch hysteresis is only supported on PF and classic devices. */
	if (!to_pci_dev(xdna->ddev.dev)->is_virtfn)
		debugfs_create_file_unsafe("ctx_switch_hysteresis_us", 0600,
					   xdna->ddev.accel->debugfs_root, ndev,
					   &aie4_ctx_hysteresis_fops);
}

void aie4_hwctx_suspend_all(struct amdxdna_dev_hdl *ndev, int clean_jobs)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct amdxdna_client *client;
	struct amdxdna_hwctx *hwctx;
	unsigned long hwctx_id;
	int idx;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	amdxdna_for_each_client(xdna, client) {
		idx = srcu_read_lock(&client->hwctx_srcu);
		amdxdna_for_each_hwctx(client, hwctx_id, hwctx) {
			/* clean up workers and drain running jobs */
			if (clean_jobs) {
				int ret;

				aie4_hwctx_destroy(hwctx, AIE4_HWCTX_ERROR);
				ret = aie4_hwctx_wait_for_running(hwctx);
				if (ret)
					XDNA_WARN(xdna, "hwctx %s wait for running failed %d",
						  hwctx->name, ret);
			} else {
				aie4_hwctx_destroy(hwctx, AIE4_HWCTX_NORMAL);
			}
		}
		srcu_read_unlock(&client->hwctx_srcu, idx);
	}

	XDNA_DBG(xdna, "Finished hwctx suspend");
}

int aie4_hwctx_resume_all(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct amdxdna_client *client;
	struct amdxdna_hwctx *hwctx;
	unsigned long hwctx_id;
	int ret, idx;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	amdxdna_for_each_client(xdna, client) {
		idx = srcu_read_lock(&client->hwctx_srcu);
		amdxdna_for_each_hwctx(client, hwctx_id, hwctx) {
			ret = aie4_hwctx_create(hwctx);
			if (ret)
				goto error;
			aie4_hwctx_resume_jobs(hwctx);
		}
		srcu_read_unlock(&client->hwctx_srcu, idx);
	}

	XDNA_DBG(xdna, "Finished hwctx resume");
	return 0;
error:
	srcu_read_unlock(&client->hwctx_srcu, idx);
	XDNA_DBG(xdna, "Failed hwctx resume");
	return ret;
}

static int aie4_restore_sriov(struct amdxdna_dev_hdl *ndev)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret;

	if (ndev->num_vfs) {
		if (pci_num_vf(pdev) != ndev->num_vfs) {
			XDNA_ERR(xdna, "inconsistent vf number");
			return -EINVAL;
		}
		ret = aie4_create_vfs(ndev, ndev->num_vfs);
		if (ret) {
			XDNA_ERR(xdna, "create vfs failed, %d", ret);
			return ret;
		}
		XDNA_DBG(xdna, "restored num_vfs %d", ndev->num_vfs);
	}

	return 0;
}

static int aie4_pf_suspend(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));
	aie4_pf_hw_stop(ndev);
	pci_disable_device(pdev);

	XDNA_DBG(xdna, "pf suspend done");
	return 0;
}

static int aie4_pf_resume(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	ret = pci_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "enable pci device failed %d", ret);
		return ret;
	}
	pci_set_master(pdev);

	ret = aie4_pf_hw_start(ndev);
	if (ret) {
		XDNA_ERR(xdna, "hw_start failed %d", ret);
		goto pci_disable;
	}

	ret = aie4_restore_sriov(ndev);
	if (ret)
		goto hw_stop;

	XDNA_DBG(xdna, "pf resume done");
	return 0;
hw_stop:
	aie4_pf_hw_stop(ndev);
pci_disable:
	pci_disable_device(pdev);
	return ret;
}

static int aie4_vf_suspend(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));
	aie4_hwctx_suspend_all(ndev, false);
	/*
	 * partition_fini and mailbox messages should not be called here
	 * because PF suspend will do the cleanup for all VFs.
	 */
	aie4_mailbox_fini(ndev);
	pci_disable_device(pdev);

	XDNA_DBG(xdna, "vf suspend done");
	return 0;
}

static int aie4_vf_resume(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	ret = pci_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "enable pci device failed %d", ret);
		/* Contexts cannot reconnect; fail waiters and abort running jobs. */
		aie4_hwctx_suspend_all(ndev, true);
		return ret;
	}
	pci_set_master(pdev);

	ret = aie4_vf_hw_start(ndev);
	if (ret) {
		XDNA_ERR(xdna, "hw_start failed %d", ret);
		/* Contexts cannot reconnect; fail waiters and abort running jobs. */
		aie4_hwctx_suspend_all(ndev, true);
		goto pci_disable;
	}

	ret = aie4_hwctx_resume_all(ndev);
	if (ret) {
		XDNA_ERR(xdna, "hwctx_resume failed %d", ret);
		goto hw_clear;
	}

	XDNA_DBG(xdna, "vf resume done");
	return 0;

hw_clear:
	aie4_hwctx_suspend_all(ndev, true);
	aie4_vf_hw_stop(ndev);
pci_disable:
	pci_disable_device(pdev);
	return ret;
}

static int aie4_classic_suspend(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));
	aie4_hwctx_suspend_all(ndev, false);
	aie4_classic_hw_stop(ndev);
	pci_disable_device(pdev);

	XDNA_DBG(xdna, "classic suspend done");
	return 0;
}

static int aie4_classic_resume(struct amdxdna_dev *xdna)
{
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct pci_dev *pdev = to_pci_dev(xdna->ddev.dev);
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	ret = pci_enable_device(pdev);
	if (ret) {
		XDNA_ERR(xdna, "enable pci device failed %d", ret);
		/* Contexts cannot reconnect; fail waiters and abort running jobs. */
		aie4_hwctx_suspend_all(ndev, true);
		return ret;
	}
	pci_set_master(pdev);

	ret = aie4_classic_hw_start(ndev);
	if (ret) {
		XDNA_ERR(xdna, "hw_start failed %d", ret);
		/* Contexts cannot reconnect; fail waiters and abort running jobs. */
		aie4_hwctx_suspend_all(ndev, true);
		goto pci_disable;
	}

	ret = aie4_hwctx_resume_all(ndev);
	if (ret) {
		XDNA_ERR(xdna, "hwctx_resume failed %d", ret);
		goto hw_clear;
	}

	XDNA_DBG(xdna, "classic resume done");
	return 0;
hw_clear:
	aie4_hwctx_suspend_all(ndev, true);
	aie4_classic_hw_stop(ndev);
pci_disable:
	pci_disable_device(pdev);
	return ret;
}

const struct amdxdna_dev_ops aie4_pf_ops = {
	.init			= aie4_pf_init,
	.fini			= aie4_pf_fini,
	.debugfs_init		= aie4_debugfs_init,
	.sriov_configure        = aie4_sriov_configure,
	.resume			= pm_ptr(aie4_pf_resume),
	.suspend		= pm_ptr(aie4_pf_suspend),
};

const struct amdxdna_dev_ops aie4_vf_ops = {
	.init			= aie4_vf_init,
	.fini			= aie4_vf_fini,
	.debugfs_init		= aie4_debugfs_init,
	.hwctx_init		= aie4_hwctx_init,
	.hwctx_stop		= aie4_hwctx_stop,
	.hwctx_fini		= aie4_hwctx_fini,
	.cmd_submit		= aie4_cmd_submit,
	.cmd_wait		= aie4_cmd_wait,
	.get_aie_info		= aie4_get_info,
	.set_aie_state		= aie4_set_state,
	.resume			= pm_ptr(aie4_vf_resume),
	.suspend		= pm_ptr(aie4_vf_suspend),
};

const struct amdxdna_dev_ops aie4_classic_ops = {
	.init			= aie4_classic_init,
	.fini			= aie4_classic_fini,
	.debugfs_init		= aie4_debugfs_init,
	.hwctx_init		= aie4_hwctx_init,
	.hwctx_stop		= aie4_hwctx_stop,
	.hwctx_fini		= aie4_hwctx_fini,
	.cmd_submit		= aie4_cmd_submit,
	.cmd_wait		= aie4_cmd_wait,
	.get_aie_info		= aie4_get_info,
	.set_aie_state		= aie4_set_state,
	.resume			= pm_ptr(aie4_classic_resume),
	.suspend		= pm_ptr(aie4_classic_suspend),
	.runtime_resume		= pm_ptr(aie4_classic_resume),
	.runtime_suspend	= pm_ptr(aie4_classic_suspend),
};
