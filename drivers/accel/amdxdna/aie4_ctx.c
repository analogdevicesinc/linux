// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#include <drm/amdxdna_accel.h>
#include <drm/drm_device.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_print.h>
#include <drm/gpu_scheduler.h>
#include <linux/hmm.h>
#include <linux/iommu.h>
#include <linux/types.h>

#include "aie.h"
#include "aie4_host_queue.h"
#include "aie4_msg_priv.h"
#include "aie4_pci.h"
#include "amdxdna_ctx.h"
#include "amdxdna_gem.h"
#include "amdxdna_mailbox.h"
#include "amdxdna_mailbox_helper.h"
#include "amdxdna_pci_drv.h"
#include "amdxdna_pm.h"

#define CTX_INVALID_ID			(~0U)
#define CTX_INVALID_DOORBELL		AMDXDNA_INVALID_DOORBELL_OFFSET
#define AIE4_JOB_TIMEOUT_MS		2000

static void job_worker(struct work_struct *work);

static struct cert_comp *aie4_lookup_cert_comp(struct amdxdna_dev_hdl *ndev, u32 msix_idx)
{
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	struct cert_comp *cert_comp;
	int ret;

	mutex_lock(&ndev->cert_comp_lock);
	cert_comp = xa_load(&ndev->cert_comp_xa, msix_idx);
	if (cert_comp) {
		kref_get(&cert_comp->kref);
		mutex_unlock(&ndev->cert_comp_lock);
		return cert_comp;
	}

	cert_comp = kzalloc_obj(*cert_comp);
	if (!cert_comp) {
		ret = -ENOMEM;
		goto unlock;
	}

	cert_comp->ndev = ndev;
	cert_comp->msix_idx = msix_idx;
	cert_comp->irq = -ENOENT;
	init_waitqueue_head(&cert_comp->waitq);
	kref_init(&cert_comp->kref);

	/* Transport-specific: PCI wires an MSI-X irq, platform an IPI callback. */
	ret = aie4_request_notification(cert_comp);
	if (ret) {
		XDNA_ERR(xdna, "request notification for msix idx %u failed %d", msix_idx, ret);
		goto free_cert_comp;
	}

	ret = xa_err(xa_store(&ndev->cert_comp_xa, msix_idx, cert_comp, GFP_KERNEL));
	if (ret) {
		XDNA_ERR(xdna, "store cert_comp for msix idx %u failed %d", msix_idx, ret);
		goto free_irq;
	}
	mutex_unlock(&ndev->cert_comp_lock);

	return cert_comp;

free_irq:
	aie4_free_notification(cert_comp);
free_cert_comp:
	kfree(cert_comp);
unlock:
	mutex_unlock(&ndev->cert_comp_lock);
	return ERR_PTR(ret);
}

static void cert_comp_release(struct kref *kref)
{
	struct cert_comp *cert_comp = container_of(kref, struct cert_comp, kref);
	struct amdxdna_dev_hdl *ndev = cert_comp->ndev;

	xa_erase(&ndev->cert_comp_xa, cert_comp->msix_idx);
	aie4_free_notification(cert_comp);
	kfree(cert_comp);
}

static void aie4_put_cert_comp(struct cert_comp *cert_comp)
{
	struct amdxdna_dev_hdl *ndev = cert_comp->ndev;

	guard(mutex)(&ndev->cert_comp_lock);

	kref_put(&cert_comp->kref, cert_comp_release);
}

static struct cert_comp *aie4_get_cert_comp(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct cert_comp *cert_comp;

	/* Take io_lock to serialize against cert_comp link/unlink. */
	guard(mutex)(&priv->io_lock);

	cert_comp = READ_ONCE(priv->cert_comp);
	if (cert_comp)
		kref_get(&cert_comp->kref);

	return cert_comp;
}

static void aie4_msg_destroy_context(struct amdxdna_dev_hdl *ndev, u32 hw_context_id)
{
	DECLARE_AIE_MSG(aie4_msg_destroy_hw_context, AIE4_MSG_OP_DESTROY_HW_CONTEXT);
	struct amdxdna_dev *xdna = ndev->aie.xdna;
	int ret;

	if (hw_context_id == CTX_INVALID_ID)
		return;

	req.hw_context_id = hw_context_id;
	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret)
		XDNA_WARN(xdna, "destroy ctx id %d failed %d", hw_context_id, ret);
}

static u8 aie4_parse_priority_to_dev(u32 priority)
{
	switch (priority) {
	case AMDXDNA_QOS_LOW_PRIORITY:
		return AIE4_CONTEXT_PRIORITY_BAND_IDLE;
	case AMDXDNA_QOS_NORMAL_PRIORITY:
		return AIE4_CONTEXT_PRIORITY_BAND_NORMAL;
	case AMDXDNA_QOS_HIGH_PRIORITY:
		return AIE4_CONTEXT_PRIORITY_BAND_FOCUS;
	case AMDXDNA_QOS_REALTIME_PRIORITY:
		return AIE4_CONTEXT_PRIORITY_BAND_REAL_TIME;
	default:
		return AIE4_CONTEXT_PRIORITY_BAND_NORMAL;
	}
}

int aie4_hwctx_create(struct amdxdna_hwctx *hwctx)
{
	DECLARE_AIE_MSG(aie4_msg_create_hw_context, AIE4_MSG_OP_CREATE_HW_CONTEXT);
	struct amdxdna_client *client = hwctx->client;
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct cert_comp *cert_comp;
	int ret;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	if (!ndev->partition_id || !hwctx->num_tiles) {
		XDNA_ERR(xdna, "invalid request partition_id %u, num_tiles %d",
			 ndev->partition_id, hwctx->num_tiles);
		return -EINVAL;
	}

	req.partition_id = ndev->partition_id;
	req.request_num_tiles = hwctx->num_tiles;
	req.pasid = aie4_msg_pasid(client);
	req.pasid = req.pasid == IOMMU_PASID_INVALID ? 0 : req.pasid;
	req.priority_band = aie4_parse_priority_to_dev(hwctx->qos.priority);
	req.hsa_addr_high = upper_32_bits(amdxdna_gem_dev_addr(priv->umq_bo));
	req.hsa_addr_low = lower_32_bits(amdxdna_gem_dev_addr(priv->umq_bo));

	XDNA_DBG(xdna, "pasid 0x%x, num_tiles %d, hsa[0x%x 0x%x]",
		 req.pasid, req.request_num_tiles, req.hsa_addr_high, req.hsa_addr_low);

	ret = aie_send_mgmt_msg_wait(&ndev->aie, &msg);
	if (ret) {
		XDNA_ERR(xdna, "create ctx failed: %d", ret);
		return ret;
	}

	XDNA_DBG(xdna, "resp msix: %d, ctx id: %d, doorbell: %d",
		 resp.job_complete_msix_idx, resp.hw_context_id,
		 resp.doorbell_offset);

	/* setup interrupt completion per msix index */
	cert_comp = aie4_lookup_cert_comp(ndev, resp.job_complete_msix_idx);
	if (IS_ERR(cert_comp)) {
		aie4_msg_destroy_context(ndev, resp.hw_context_id);
		return PTR_ERR(cert_comp);
	}

	priv->hw_ctx_id = resp.hw_context_id;

	hwctx->fw_ctx_id = resp.hw_context_id;
	hwctx->start_col = 0;
	hwctx->num_col = ndev->total_col;

	/* Set up driver doorbell and return invalid offset to userspace. */
	mutex_lock(&priv->io_lock);
	ret = aie4_doorbell_setup(hwctx, &resp);
	if (ret) {
		mutex_unlock(&priv->io_lock);
		aie4_msg_destroy_context(ndev, resp.hw_context_id);
		aie4_put_cert_comp(cert_comp);
		priv->hw_ctx_id = CTX_INVALID_ID;
		hwctx->fw_ctx_id = -1;
		return ret;
	}
	WRITE_ONCE(priv->has_error, false);
	WRITE_ONCE(priv->cert_comp, cert_comp);
	mutex_unlock(&priv->io_lock);
	hwctx->doorbell_offset = CTX_INVALID_DOORBELL;
	wake_up_all(&priv->job_list_wq);

	return 0;
}

/* Linked cert_comp acts as connected sentinel for submit waiters. */
static bool aie4_hwctx_connected(struct amdxdna_hwctx *hwctx)
{
	return !!READ_ONCE(hwctx->priv->cert_comp);
}

static bool aie4_hwctx_has_error(struct amdxdna_hwctx *hwctx)
{
	return READ_ONCE(hwctx->priv->has_error);
}

void aie4_hwctx_destroy(struct amdxdna_hwctx *hwctx, enum aie4_hwctx_flags flags)
{
	struct amdxdna_client *client = hwctx->client;
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct cert_comp *cert_comp;
	bool has_error = false;

	drm_WARN_ON(&xdna->ddev, !mutex_is_locked(&xdna->dev_lock));

	if (flags == AIE4_HWCTX_DISCONNECT || flags == AIE4_HWCTX_ERROR)
		has_error = true;

	mutex_lock(&priv->io_lock);
	if (has_error)
		WRITE_ONCE(priv->has_error, true);
	cert_comp = priv->cert_comp;
	WRITE_ONCE(priv->cert_comp, NULL);
	mutex_unlock(&priv->io_lock);

	if (cert_comp)
		wake_up_all(&cert_comp->waitq);

	if (has_error)
		wake_up_all(&priv->job_list_wq);

	if (flags != AIE4_HWCTX_DISCONNECT && priv->hw_ctx_id != CTX_INVALID_ID)
		aie4_msg_destroy_context(ndev, priv->hw_ctx_id);

	if (cert_comp)
		aie4_put_cert_comp(cert_comp);

	priv->hw_ctx_id = CTX_INVALID_ID;
	hwctx->fw_ctx_id = -1;
	hwctx->doorbell_offset = CTX_INVALID_DOORBELL;

	/* Skip cancel_work_sync on error so worker can abort in-flight jobs. */
	if (!has_error)
		cancel_work_sync(&priv->job_work);
}

static void aie4_hwctx_umq_fini(struct amdxdna_hwctx *hwctx)
{
	if (hwctx->priv && hwctx->priv->umq_bo)
		drm_gem_object_put(to_gobj(hwctx->priv->umq_bo));
}

static int aie4_hwctx_umq_init(struct amdxdna_hwctx *hwctx)
{
	const size_t indir_pkts_sz = CTX_MAX_CMDS * HSA_MAX_LEVEL1_INDIRECT_ENTRIES *
				     sizeof(struct host_indirect_packet_data);
	const size_t pkts_sz = CTX_MAX_CMDS * sizeof(struct host_queue_packet);
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_gem_obj *umq_bo;
	struct host_queue_header *qhdr;
	u64 data_dev_addr;
	void *umq_va;
	int ret;
	int i;

	/* Queue lives in user-allocated BO so device can access it under PASID. */
	umq_bo = amdxdna_gem_get_obj(hwctx->client, hwctx->umq_bo_hdl, AMDXDNA_BO_SHARE);
	if (!umq_bo) {
		XDNA_ERR(xdna, "cannot find umq_bo handle %d", hwctx->umq_bo_hdl);
		return -ENOENT;
	}

	/* Ensure user BO can hold queue header and direct/indirect packet arrays. */
	if (umq_bo->mem.size < sizeof(*qhdr) ||
	    (umq_bo->mem.size < sizeof(*qhdr) + pkts_sz + indir_pkts_sz)) {
		XDNA_ERR(xdna, "umq_bo size %zu is too small",
			 (size_t)umq_bo->mem.size);
		ret = -EINVAL;
		goto put_umq_bo;
	}

	umq_va = amdxdna_gem_vmap(umq_bo);
	if (!umq_va) {
		ret = -ENOMEM;
		goto put_umq_bo;
	}
	qhdr = umq_va;

	priv->umq_bo = umq_bo;
	priv->umq_read_index = &qhdr->read_index;
	priv->umq_write_index = &qhdr->write_index;

	/* Lay out direct packets after header, followed by indirect packets. */
	data_dev_addr = amdxdna_gem_dev_addr(umq_bo) + sizeof(*qhdr);
	priv->umq_pkts = umq_va + sizeof(*qhdr);
	priv->umq_indirect_pkts = umq_va + sizeof(*qhdr) + pkts_sz;
	priv->umq_indirect_pkts_dev_addr = data_dev_addr + pkts_sz;

	/* Clear only the driver-managed header and packet regions. */
	memset(umq_va, 0, sizeof(*qhdr) + pkts_sz + indir_pkts_sz);
	priv->write_index = QUEUE_INDEX_START;
	qhdr->read_index = QUEUE_INDEX_START;
	qhdr->write_index = QUEUE_INDEX_START;
	qhdr->version.major = HOST_QUEUE_MAJOR_VERSION;
	qhdr->version.minor = HOST_QUEUE_MINOR_VERSION;
	qhdr->capacity = CTX_MAX_CMDS;
	qhdr->data_address = data_dev_addr;
	for (i = 0; i < CTX_MAX_CMDS; i++)
		priv->umq_pkts[i].pkt_header.common_header.opcode = OPCODE_EXEC_BUF;
	for (i = 0; i < CTX_MAX_CMDS * HSA_MAX_LEVEL1_INDIRECT_ENTRIES; i++) {
		priv->umq_indirect_pkts[i].header.opcode = OPCODE_EXEC_BUF;
		priv->umq_indirect_pkts[i].header.count = sizeof(struct exec_buf);
		priv->umq_indirect_pkts[i].header.distribute = 1;
	}

	return 0;

put_umq_bo:
	amdxdna_gem_put_obj(umq_bo);
	return ret;
}

int aie4_hwctx_init(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_client *client = hwctx->client;
	struct amdxdna_dev *xdna = client->xdna;
	struct amdxdna_dev_hdl *ndev = xdna->dev_handle;
	struct amdxdna_hwctx_priv *priv;
	int ret;

	if (!AIE_FEATURE_ON(&ndev->aie, AIE4_HSA_COMMAND))
		return -EOPNOTSUPP;

	priv = kzalloc_obj(*priv);
	if (!priv)
		return -ENOMEM;
	hwctx->priv = priv;
	priv->hwctx = hwctx;
	priv->hw_ctx_id = CTX_INVALID_ID;

	/* Initialize io_lock guarding cert_comp binding. */
	mutex_init(&priv->io_lock);

	INIT_LIST_HEAD(&priv->pending_job_list);
	INIT_LIST_HEAD(&priv->running_job_list);
	init_waitqueue_head(&priv->job_list_wq);
	INIT_WORK(&priv->job_work, job_worker);

	ret = aie4_hwctx_umq_init(hwctx);
	if (ret)
		goto destroy_lock;

	ret = amdxdna_pm_resume_get_locked(xdna);
	if (ret)
		goto umq_fini;

	ret = aie4_hwctx_create(hwctx);
	if (ret)
		goto suspend_put;

	priv->job_work_q = alloc_ordered_workqueue("aie4_job_%d_%d", 0,
						   client->pid, hwctx->fw_ctx_id);
	if (!priv->job_work_q) {
		XDNA_ERR(xdna, "Create job_work_q failed");
		ret = -ENOMEM;
		goto destroy_ctx;
	}
	amdxdna_pm_suspend_put(xdna);

	XDNA_DBG(xdna, "hwctx %d.%d init completed", client->pid, hwctx->fw_ctx_id);
	return 0;

destroy_ctx:
	aie4_hwctx_destroy(hwctx, AIE4_HWCTX_NORMAL);
suspend_put:
	amdxdna_pm_suspend_put(xdna);
umq_fini:
	aie4_hwctx_umq_fini(hwctx);
destroy_lock:
	mutex_destroy(&priv->io_lock);
	kfree(priv);
	hwctx->priv = NULL;
	return ret;
}

void aie4_hwctx_stop(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	if (priv->hw_ctx_id == CTX_INVALID_ID)
		return;

	/* Mark error to drain running jobs and unlink cert_comp to wake waiters. */
	aie4_hwctx_destroy(hwctx, AIE4_HWCTX_ERROR);
}

void aie4_hwctx_fini(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	if (priv->job_work_q) {
		aie4_hwctx_wait_for_running(hwctx);
		destroy_workqueue(priv->job_work_q);
	}
	aie4_hwctx_umq_fini(hwctx);
	mutex_destroy(&priv->io_lock);
	kfree(hwctx->priv);
}

static inline bool valid_queue_index(u64 read, u64 write, u32 capacity)
{
	return (write >= read) && ((write - read) <= capacity);
}

static u64 get_read_index(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	u64 ri, wi;

	/* Sample read_index before write_index to guarantee wi >= ri. */
	ri = READ_ONCE(*priv->umq_read_index);
	/* Order the read_index sample before the write_index sample. */
	smp_rmb();
	wi = READ_ONCE(priv->write_index);

	/* Non-atomic 64-bit counter update by CERT; re-sample once if invalid. */
	if (!valid_queue_index(ri, wi, CTX_MAX_CMDS)) {
		ri = READ_ONCE(*priv->umq_read_index);
		/* Order the read_index sample before the write_index sample. */
		smp_rmb();
		wi = READ_ONCE(priv->write_index);
		if (!valid_queue_index(ri, wi, CTX_MAX_CMDS)) {
			/* 0 <= any seq: callers always treat it as "not done" and retry. */
			XDNA_DBG(xdna, "Invalid index, ri %llu, wi %llu", ri, wi);
			return 0;
		}
	}

	return ri;
}

/* Verify cert_comp remains linked to detect disconnect or reset. */
static bool check_cert_comp_linked(struct amdxdna_hwctx *hwctx, struct cert_comp *comp)
{
	/* READ_ONCE pairs with the link/unlink WRITE_ONCE. */
	return comp == READ_ONCE(hwctx->priv->cert_comp);
}

/*
 * Lockless wait condition: wake on completion or disconnect. Record
 * completion, so callers do not re-read state that can change after the
 * wait.
 */
static inline bool cmd_done_or_unlinked(struct amdxdna_hwctx *hwctx, u64 seq,
					struct cert_comp *comp, bool *done)
{
	*done = get_read_index(hwctx) > seq;
	return *done || !check_cert_comp_linked(hwctx, comp);
}

/*
 * Return 0 once seq completed, -ENODEV if the context is lost, or -EAGAIN
 * if it is suspended by power management before seq completed.
 */
int aie4_cmd_wait(struct amdxdna_hwctx *hwctx, u64 seq, u32 timeout)
{
	unsigned long wait_jifs = MAX_SCHEDULE_TIMEOUT;
	struct cert_comp *cert_comp = aie4_get_cert_comp(hwctx);
	bool done = false;
	long ret;

	if (!cert_comp) {
		/* Not connected: read_index in host memory is still valid. */
		if (get_read_index(hwctx) > seq)
			return 0;
		return aie4_hwctx_has_error(hwctx) ? -ENODEV : -EAGAIN;
	}

	if (timeout)
		wait_jifs = msecs_to_jiffies(timeout);

	ret = wait_event_interruptible_timeout(cert_comp->waitq,
					       cmd_done_or_unlinked(hwctx, seq, cert_comp,
								    &done),
					       wait_jifs);
	aie4_put_cert_comp(cert_comp);

	if (ret < 0)
		return ret;
	if (done)
		return 0;
	if (!ret)
		return -ETIME;

	/* Woken by disconnect before seq completed. */
	return aie4_hwctx_has_error(hwctx) ? -ENODEV : -EAGAIN;
}

/* ---- kernel-mode submission (driver fills the queue and rings doorbell) ---- */

/* Publish a command to CERT and return the assigned command sequence (slot). */
static u64 publish_cmd(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	u64 wi = priv->write_index;

	/* Paired with the lockless READ_ONCE() readers of write_index. */
	WRITE_ONCE(priv->write_index, wi + 1);
	/* Order the packet-slot writes before CERT sees the new write_index. */
	wmb();
	WRITE_ONCE(*priv->umq_write_index, wi + 1);
	return wi;
}

/*
 * Return 0 once seq completed, -EAGAIN on disconnect, -ETIME on timeout,
 * -ERESTARTSYS on signal.
 */
static int wait_till_seq_completed(struct amdxdna_hwctx *hwctx, u64 seq,
				   unsigned long timeout)
{
	struct cert_comp *cert_comp;
	bool done = false;
	long ret;

	/* Not connected: read_index in host memory is still valid. */
	cert_comp = aie4_get_cert_comp(hwctx);
	if (!cert_comp)
		return get_read_index(hwctx) > seq ? 0 : -EAGAIN;

	/* Wait for queue slot; freezable for suspend, interruptible for signals. */
	ret = wait_event_freezable_timeout(cert_comp->waitq,
					   cmd_done_or_unlinked(hwctx, seq, cert_comp,
								&done),
					   timeout);
	aie4_put_cert_comp(cert_comp);
	if (ret < 0)
		return ret;
	if (done)
		return 0;

	return ret ? -EAGAIN : -ETIME;
}

static int wait_till_connected_hsa_not_full(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	u64 wi = READ_ONCE(priv->write_index);
	bool hsa_not_full = !!(wi < CTX_MAX_CMDS);
	int ret;

	do {
		mutex_unlock(&priv->io_lock);
		if (!hsa_not_full) {
			ret = wait_till_seq_completed(hwctx, wi - CTX_MAX_CMDS,
						      MAX_SCHEDULE_TIMEOUT);
			if (ret && ret != -EAGAIN) {
				mutex_lock(&priv->io_lock);
				return ret;
			}
			if (!ret)
				hsa_not_full = true;
		}
		ret = wait_event_freezable(priv->job_list_wq,
					   aie4_hwctx_connected(hwctx) ||
					   aie4_hwctx_has_error(hwctx));
		mutex_lock(&priv->io_lock);
		if (ret)
			return ret;
		if (aie4_hwctx_has_error(hwctx)) {
			XDNA_DBG(xdna, "ctx %s in error; unwinding -ENODEV",
				 hwctx->name);
			return -ENODEV;
		}
	} while (!hsa_not_full || !aie4_hwctx_connected(hwctx));

	return 0;
}

static int fill_indirect_pkt(struct amdxdna_hwctx_priv *priv, u64 slot_idx,
			     u32 total_slots, struct amdxdna_cmd_start_dpu *dpu,
			     u16 entries)
{
	struct host_queue_packet *pkt = &priv->umq_pkts[slot_idx];
	struct host_indirect_packet_entry *hipe =
		(struct host_indirect_packet_entry *)(pkt->data);
	u16 i;

	if (!entries || entries > HSA_MAX_LEVEL1_INDIRECT_ENTRIES) {
		XDNA_ERR(priv->hwctx->client->xdna, "Invalid indirect entries %u", entries);
		return -EINVAL;
	}

	for (i = 0; i < entries; i++, dpu++, hipe++) {
		struct host_indirect_packet_data *hipd;
		u64 indirect_pkt_dev_addr;
		u32 uci = READ_ONCE(dpu->uc_index);
		u64 dtrace_buf = READ_ONCE(dpu->dtrace_buffer);
		u64 inst_buf = READ_ONCE(dpu->instruction_buffer);
		u32 idx;

		/* Validate uc_index against indirect packet bounds before publication. */
		if (uci >= HSA_MAX_LEVEL1_INDIRECT_ENTRIES) {
			XDNA_ERR(priv->hwctx->client->xdna, "Invalid uc index %d", uci);
			return -EINVAL;
		}
		if (upper_32_bits(dtrace_buf) > U16_MAX) {
			XDNA_ERR(priv->hwctx->client->xdna,
				 "Invalid dtrace buffer address 0x%llx", dtrace_buf);
			return -EINVAL;
		}
		idx = uci * total_slots + slot_idx;
		hipd = &priv->umq_indirect_pkts[idx];
		indirect_pkt_dev_addr = priv->umq_indirect_pkts_dev_addr +
			sizeof(struct host_indirect_packet_data) * idx;

		/* Point the indirect entry at the indirect packet. */
		hipe->host_addr_low = lower_32_bits(indirect_pkt_dev_addr);
		hipe_set_host_addr_high(&hipe->host_addr_high_uc_index,
					upper_32_bits(indirect_pkt_dev_addr));
		hipe_set_uc_index(&hipe->host_addr_high_uc_index, uci);

		/* Fill in the indirect packet. */
		hipd->payload.dpu_control_code_host_addr_low =
			lower_32_bits(inst_buf);
		hipd->payload.dpu_control_code_host_addr_high =
			upper_32_bits(inst_buf);
		hipd->payload.dtrace_buf_host_addr_low =
			lower_32_bits(dtrace_buf);
		hipd->payload.dtrace_buf_host_addr_high =
			lower_16_bits(upper_32_bits(dtrace_buf));
	}
	pkt->pkt_header.common_header.distribute = 1;
	pkt->pkt_header.common_header.indirect = 1;
	pkt->pkt_header.common_header.count = entries * sizeof(*hipe);
	return 0;
}

static int fill_direct_pkt(struct amdxdna_hwctx_priv *priv, u64 slot_idx,
			   struct amdxdna_cmd_start_dpu *dpu)
{
	struct host_queue_packet *pkt = &priv->umq_pkts[slot_idx];
	struct exec_buf *ebuf = (struct exec_buf *)(pkt->data);
	u64 dtrace_buf = READ_ONCE(dpu->dtrace_buffer);
	u64 inst_buf = READ_ONCE(dpu->instruction_buffer);

	if (upper_32_bits(dtrace_buf) > U16_MAX) {
		XDNA_ERR(priv->hwctx->client->xdna,
			 "Invalid dtrace buffer address 0x%llx", dtrace_buf);
		return -EINVAL;
	}

	memset(pkt->data, 0, sizeof(pkt->data));
	ebuf->dpu_control_code_host_addr_low = lower_32_bits(inst_buf);
	ebuf->dpu_control_code_host_addr_high = upper_32_bits(inst_buf);
	ebuf->dtrace_buf_host_addr_low = lower_32_bits(dtrace_buf);
	ebuf->dtrace_buf_host_addr_high = lower_16_bits(upper_32_bits(dtrace_buf));
	pkt->pkt_header.common_header.distribute = 0;
	pkt->pkt_header.common_header.indirect = 0;
	pkt->pkt_header.common_header.count = sizeof(*ebuf);
	return 0;
}

static int validate_cmd_abo(struct amdxdna_dev *xdna, struct amdxdna_gem_obj *cmd_abo,
			    struct amdxdna_cmd_start_dpu **dpu_out, u16 *chained_cnt)
{
	struct amdxdna_cmd_start_dpu *dpu;
	u32 payload_size;
	u16 chained;
	u32 op;
	u16 i;

	op = amdxdna_cmd_get_op(cmd_abo);
	if (op != ERT_START_DPU) {
		XDNA_ERR(xdna, "Invalid exec buf op, %d", op);
		return -EINVAL;
	}

	dpu = amdxdna_cmd_get_payload(cmd_abo, &payload_size);
	if (!dpu || payload_size < sizeof(*dpu)) {
		XDNA_ERR(xdna, "Invalid DPU payload, size %u", payload_size);
		return -EINVAL;
	}
	chained = READ_ONCE(dpu->chained);
	if (chained >= HSA_MAX_LEVEL1_INDIRECT_ENTRIES ||
	    payload_size < (u32)(chained + 1) * sizeof(*dpu)) {
		XDNA_ERR(xdna, "Invalid DPU chained entries %u, payload %u",
			 chained, payload_size);
		return -EINVAL;
	}

	if (!chained) {
		u64 dtrace_buf = READ_ONCE(dpu->dtrace_buffer);

		if (upper_32_bits(dtrace_buf) > U16_MAX) {
			XDNA_ERR(xdna, "Invalid dtrace buffer address 0x%llx", dtrace_buf);
			return -EINVAL;
		}
	} else {
		for (i = 0; i <= chained; i++) {
			u32 uci = READ_ONCE(dpu[i].uc_index);
			u64 dtrace_buf = READ_ONCE(dpu[i].dtrace_buffer);

			if (uci >= HSA_MAX_LEVEL1_INDIRECT_ENTRIES) {
				XDNA_ERR(xdna, "Invalid uc index %u", uci);
				return -EINVAL;
			}
			if (upper_32_bits(dtrace_buf) > U16_MAX) {
				XDNA_ERR(xdna, "Invalid dtrace buffer address 0x%llx", dtrace_buf);
				return -EINVAL;
			}
		}
	}

	if (dpu_out)
		*dpu_out = dpu;
	if (chained_cnt)
		*chained_cnt = chained;

	return 0;
}

/* Build and submit one HSA command into the user host queue. Holds io_lock. */
static int submit_one_cmd(struct amdxdna_hwctx *hwctx,
			  struct amdxdna_gem_obj *cmd_abo, bool last_of_chain,
			  u64 *seq)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_cmd_start_dpu *dpu;
	struct host_queue_packet *pkt;
	u64 slot_idx;
	u16 chained;
	int ret;

	/* Use the validated payload pointer; the header is user-writable. */
	ret = validate_cmd_abo(xdna, cmd_abo, &dpu, &chained);
	if (ret)
		return ret;

	/* Wait for queue slot and connected state. Drops and re-acquires io_lock. */
	ret = wait_till_connected_hsa_not_full(hwctx);
	if (ret) {
		XDNA_DBG(xdna, "Wait for queue slot / ctx reconnect interrupted, ret %d", ret);
		return ret;
	}

	slot_idx = priv->write_index & (CTX_MAX_CMDS - 1);
	if (chained) {
		ret = fill_indirect_pkt(priv, slot_idx, CTX_MAX_CMDS, dpu, chained + 1);
		if (ret)
			return ret;
	} else {
		ret = fill_direct_pkt(priv, slot_idx, dpu);
		if (ret)
			return ret;
	}

	pkt = &priv->umq_pkts[slot_idx];
	pkt->pkt_header.common_header.opcode = OPCODE_EXEC_BUF;
	pkt->pkt_header.common_header.chain_flag =
		last_of_chain ? CHAIN_FLG_LAST_CMD : CHAIN_FLG_NOT_LAST_CMD;
	pkt->pkt_header.common_header.reserved = 0x0;
	pkt->pkt_header.completion_signal = amdxdna_gem_dev_addr(cmd_abo) +
					    offsetof(struct amdxdna_cmd, header);
	*seq = publish_cmd(hwctx);
	aie4_doorbell_ring(hwctx);
	XDNA_DBG(xdna, "Submitted one cmd, %s seq %lld", hwctx->name, *seq);
	return 0;
}

/* Peek head job without removing it from running list. */
static struct amdxdna_sched_job *peek_running_job(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_sched_job *job;

	mutex_lock(&priv->io_lock);
	job = list_first_entry_or_null(&priv->running_job_list,
				       struct amdxdna_sched_job, aie4_job_list);
	mutex_unlock(&priv->io_lock);
	return job;
}

/* Remove a job from the running list once it is completed or reaped. */
static void dequeue_running_job(struct amdxdna_hwctx *hwctx, struct amdxdna_sched_job *job)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	mutex_lock(&priv->io_lock);
	list_del(&job->aie4_job_list);
	mutex_unlock(&priv->io_lock);
}

static void aie4_job_release(struct kref *ref)
{
	struct amdxdna_sched_job *job =
		container_of(ref, struct amdxdna_sched_job, refcnt);
	u32 i;

	for (i = 0; i < job->aie4_cmd_bo_cnt; i++)
		amdxdna_gem_put_obj(job->aie4_cmd_bos[i]);
	kfree(job->aie4_cmd_bos);

	amdxdna_sched_job_cleanup(job);
	if (job->out_fence)
		dma_fence_put(job->out_fence);
	kfree(job);
}

static void job_done(struct amdxdna_sched_job *job)
{
	job->aie4_job_state = AIE4_JOB_STATE_DONE;
	dma_fence_signal(job->fence);
	/* Release submitter mm reference taken at submit. */
	mmput_async(job->mm);
	kref_put(&job->refcnt, aie4_job_release);
}

static void job_complete(struct amdxdna_sched_job *job)
{
	job_done(job);
}

/* Advance read_index when disconnected to unblock waiters. */
static void update_read_index(struct amdxdna_hwctx *hwctx, u64 idx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	/* Order cmd-bo state write before the waiter observes completion. */
	wmb();
	WRITE_ONCE(*priv->umq_read_index, idx);
}

static void job_abort(struct amdxdna_sched_job *job)
{
	struct amdxdna_hwctx *hwctx = job->hwctx;
	u32 i;

	XDNA_WARN(hwctx->client->xdna, "aborting %s job %lld", hwctx->name, job->seq);
	amdxdna_cmd_set_state(job->cmd_bo, ERT_CMD_STATE_ABORT);
	for (i = 0; i < job->aie4_cmd_bo_cnt; i++)
		amdxdna_cmd_set_state(job->aie4_cmd_bos[i], ERT_CMD_STATE_ABORT);
	dma_fence_set_error(job->fence, -ECANCELED);
	/* Advance read_index only if CERT has not already moved past this job. */
	if (get_read_index(hwctx) <= job->seq)
		update_read_index(hwctx, job->seq + 1);
	job_done(job);
}

/*
 * Stop a context whose running job timed out, so the NPU no longer
 * accesses the job BOs before the worker aborts the jobs and signals
 * their fences. Paths holding dev_lock flush this worker, so do not
 * block on dev_lock; if it is busy, wait for another timeout period.
 */
static void job_timeout(struct amdxdna_hwctx *hwctx, struct amdxdna_sched_job *job)
{
	struct amdxdna_dev *xdna = hwctx->client->xdna;

	if (!mutex_trylock(&xdna->dev_lock))
		return;

	/* The job may have completed or the context disconnected meanwhile. */
	if (aie4_hwctx_connected(hwctx) && get_read_index(hwctx) <= job->seq) {
		XDNA_ERR(xdna, "hwctx %s job %llu timed out", hwctx->name, job->seq);
		aie4_hwctx_stop(hwctx);
	}
	mutex_unlock(&xdna->dev_lock);
}

static void job_worker(struct work_struct *work)
{
	struct amdxdna_hwctx_priv *priv =
		container_of(work, struct amdxdna_hwctx_priv, job_work);
	struct amdxdna_hwctx *hwctx = priv->hwctx;
	struct amdxdna_sched_job *job;
	int ret;

	while ((job = peek_running_job(hwctx))) {
		ret = wait_till_seq_completed(hwctx, job->seq,
					      msecs_to_jiffies(AIE4_JOB_TIMEOUT_MS));
		if (ret == -ETIME) {
			job_timeout(hwctx, job);
			continue;
		}
		if (!ret) {
			dequeue_running_job(hwctx, job);
			/* Abort partially submitted jobs; complete fully submitted ones. */
			if (job->aie4_job_state != AIE4_JOB_STATE_SUBMITTED)
				job_abort(job);
			else
				job_complete(job);
		} else if (aie4_hwctx_has_error(hwctx)) {
			dequeue_running_job(hwctx, job);
			job_abort(job);
		} else {
			/* Disconnected by suspend; resume requeues the worker. */
			break;
		}
	}
}

int aie4_hwctx_wait_for_running(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_sched_job *job;
	long error;
	int ret = 0;

	mutex_lock(&priv->io_lock);
	job = READ_ONCE(priv->pending_head);
	if (job && job->aie4_job_state == AIE4_JOB_STATE_SUBMITTING) {
		mutex_unlock(&priv->io_lock);
		error = wait_event_timeout(priv->job_list_wq,
					   READ_ONCE(priv->pending_head) != job,
					   msecs_to_jiffies(2000));
		if (!error) {
			XDNA_WARN(xdna, "hwctx %s wait for submitting job timed out",
				  hwctx->name);
			ret = -ETIMEDOUT;
		}
	} else {
		mutex_unlock(&priv->io_lock);
	}

	queue_work(priv->job_work_q, &priv->job_work);
	flush_work(&priv->job_work);
	return ret;
}

void aie4_hwctx_resume_jobs(struct amdxdna_hwctx *hwctx)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	mutex_lock(&priv->io_lock);
	if (list_empty(&priv->running_job_list)) {
		mutex_unlock(&priv->io_lock);
		return;
	}
	aie4_doorbell_ring(hwctx);
	mutex_unlock(&priv->io_lock);

	queue_work(priv->job_work_q, &priv->job_work);
}

static void put_cmd_bos(struct amdxdna_sched_job *job)
{
	u32 i;

	for (i = 0; i < job->aie4_cmd_bo_cnt; i++)
		amdxdna_gem_put_obj(job->aie4_cmd_bos[i]);
	kfree(job->aie4_cmd_bos);
	job->aie4_cmd_bos = NULL;
	job->aie4_cmd_bo_cnt = 0;
}

/* Look up and validate all sub-command BOs of a command chain. */
static int get_cmd_bos(struct amdxdna_hwctx *hwctx, struct amdxdna_sched_job *job)
{
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	struct amdxdna_cmd_chain *payload;
	struct amdxdna_gem_obj *abo;
	u32 payload_len, ccnt;
	int ret;
	u32 i;

	payload = amdxdna_cmd_get_payload(job->cmd_bo, &payload_len);
	if (!payload) {
		XDNA_ERR(xdna, "Invalid cmd payload for chained cmd");
		return -EINVAL;
	}
	ccnt = READ_ONCE(payload->command_count);
	/* A command chain cannot exceed queue capacity. */
	if (!ccnt || ccnt > CTX_MAX_CMDS ||
	    payload_len < struct_size(payload, data, ccnt)) {
		XDNA_ERR(xdna, "Invalid command count %u", ccnt);
		return -EINVAL;
	}

	job->aie4_cmd_bos = kcalloc(ccnt, sizeof(*job->aie4_cmd_bos), GFP_KERNEL);
	if (!job->aie4_cmd_bos)
		return -ENOMEM;

	for (i = 0; i < ccnt; i++) {
		u32 boh = (u32)(payload->data[i]);

		abo = amdxdna_gem_get_obj(hwctx->client, boh, AMDXDNA_BO_SHARE);
		if (!abo) {
			XDNA_ERR(xdna, "Failed to find cmd BO %u at index %u", boh, i);
			ret = -ENOENT;
			goto put_bos;
		}
		job->aie4_cmd_bos[job->aie4_cmd_bo_cnt++] = abo;

		ret = validate_cmd_abo(xdna, abo, NULL, NULL);
		if (ret)
			goto put_bos;
	}

	return 0;

put_bos:
	put_cmd_bos(job);
	return ret;
}

/* Append a GEM object to the lock list unless it is already there. */
static void add_lock_obj(struct drm_gem_object **objs, u32 *cnt,
			 struct drm_gem_object *obj)
{
	u32 i;

	for (i = 0; i < *cnt; i++) {
		if (objs[i] == obj)
			return;
	}
	objs[(*cnt)++] = obj;
}

/*
 * Lock arg and command BOs, repopulate invalidated mappings and attach
 * the job fence. The NPU writes completion to command BO headers, so
 * they are fenced too. Duplicates are skipped as
 * drm_gem_lock_reservations() fails on them.
 */
static int fence_job_bos(struct amdxdna_dev *xdna, struct amdxdna_sched_job *job)
{
	struct ww_acquire_ctx acquire_ctx;
	struct drm_gem_object **objs;
	struct amdxdna_gem_obj *abo;
	unsigned long timeout = 0;
	u32 cnt = 0;
	int ret;
	u32 i;

	objs = kmalloc_array(job->bo_cnt + 1 + job->aie4_cmd_bo_cnt,
			     sizeof(*objs), GFP_KERNEL);
	if (!objs)
		return -ENOMEM;

	for (i = 0; i < job->bo_cnt; i++)
		add_lock_obj(objs, &cnt, job->bos[i]);
	add_lock_obj(objs, &cnt, to_gobj(job->cmd_bo));
	for (i = 0; i < job->aie4_cmd_bo_cnt; i++)
		add_lock_obj(objs, &cnt, to_gobj(job->aie4_cmd_bos[i]));

retry:
	ret = drm_gem_lock_reservations(objs, cnt, &acquire_ctx);
	if (ret) {
		XDNA_WARN(xdna, "Failed to lock BOs, ret %d", ret);
		goto free_objs;
	}

	for (i = 0; i < cnt; i++) {
		ret = dma_resv_reserve_fences(objs[i]->resv, 1);
		if (ret) {
			XDNA_WARN(xdna, "Failed to reserve fences %d", ret);
			goto unlock;
		}
	}

	down_read(&xdna->notifier_lock);
	for (i = 0; i < cnt; i++) {
		abo = to_xdna_obj(objs[i]);
		if (abo->mem.map_invalid) {
			up_read(&xdna->notifier_lock);
			drm_gem_unlock_reservations(objs, cnt, &acquire_ctx);
			if (!timeout) {
				timeout = jiffies +
					msecs_to_jiffies(HMM_RANGE_DEFAULT_TIMEOUT);
			} else if (time_after(jiffies, timeout)) {
				ret = -ETIME;
				goto free_objs;
			}

			ret = amdxdna_populate_range(abo);
			if (ret)
				goto free_objs;
			goto retry;
		}
	}

	job->out_fence = dma_fence_get(job->fence);
	for (i = 0; i < cnt; i++)
		dma_resv_add_fence(objs[i]->resv, job->out_fence, DMA_RESV_USAGE_WRITE);
	up_read(&xdna->notifier_lock);

unlock:
	drm_gem_unlock_reservations(objs, cnt, &acquire_ctx);
free_objs:
	kfree(objs);
	return ret;
}

/*
 * Submit job command(s) to host queue. Called with io_lock held.
 * Returns 0 on success or if partial chain is queued for worker abort.
 */
static int submit_job_cmds(struct amdxdna_hwctx *hwctx,
			   struct amdxdna_sched_job *job, u32 op)
{
	int ret = 0;
	u32 i;

	/* Single cmd. */
	if (op == ERT_START_DPU) {
		ret = submit_one_cmd(hwctx, job->cmd_bo, true, &job->seq);
		if (!ret)
			job->aie4_job_state = AIE4_JOB_STATE_SUBMITTED;
		return ret;
	}

	/* Cmd chain. Sub-command BOs were looked up and validated at submit. */
	for (i = 0; i < job->aie4_cmd_bo_cnt; i++) {
		ret = submit_one_cmd(hwctx, job->aie4_cmd_bos[i],
				     i + 1 == job->aie4_cmd_bo_cnt, &job->seq);
		if (ret)
			break;
		job->aie4_job_state = AIE4_JOB_STATE_SUBMITTING;
	}

	if (!ret) {
		job->aie4_job_state = AIE4_JOB_STATE_SUBMITTED;
		return 0;
	}

	/*
	 * If partial submission occurred, return 0 so the job is queued to
	 * running_job_list. The worker will wait for hardware to finish the
	 * published packets (up to job->seq), then abort the job safely.
	 */
	if (job->aie4_job_state == AIE4_JOB_STATE_SUBMITTING)
		return 0;

	return ret;
}

/* Pending list serializes job submissions on the hardware queue. */
/* Publish current pending-list head for lockless submit wait condition. */
static void update_pending_head(struct amdxdna_hwctx_priv *priv)
{
	WRITE_ONCE(priv->pending_head,
		   list_first_entry_or_null(&priv->pending_job_list,
					    struct amdxdna_sched_job, aie4_job_list));
}

static void enqueue_pending_job(struct amdxdna_hwctx *hwctx,
				struct amdxdna_sched_job *job)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	mutex_lock(&priv->io_lock);
	list_add_tail(&job->aie4_job_list, &priv->pending_job_list);
	job->aie4_job_state = AIE4_JOB_STATE_PENDING;
	update_pending_head(priv);
	mutex_unlock(&priv->io_lock);

	/* Let the next pending submitter re-check whether it is now first. */
	wake_up_all(&priv->job_list_wq);
}

static void cancel_pending_job(struct amdxdna_hwctx *hwctx,
			       struct amdxdna_sched_job *job)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;

	mutex_lock(&priv->io_lock);
	list_del(&job->aie4_job_list);
	job->aie4_job_state = AIE4_JOB_STATE_INIT;
	update_pending_head(priv);
	mutex_unlock(&priv->io_lock);
	/* Let the next pending submitter re-check whether it is now first. */
	wake_up_all(&priv->job_list_wq);
}

int aie4_cmd_submit(struct amdxdna_hwctx *hwctx, struct amdxdna_sched_job *job, u64 *seq)
{
	struct amdxdna_hwctx_priv *priv = hwctx->priv;
	struct amdxdna_dev *xdna = hwctx->client->xdna;
	u32 op;
	int ret;

	XDNA_DBG(xdna, "ctx %s job %p received", hwctx->name, job);

	if (!job->cmd_bo) {
		XDNA_ERR(xdna, "No command BO in job");
		return -EINVAL;
	}

	op = amdxdna_cmd_get_op(job->cmd_bo);
	if (op != ERT_START_DPU && op != ERT_CMD_CHAIN) {
		XDNA_ERR(xdna, "Invalid cmd opcode %d", op);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&job->aie4_job_list);

	/* Pin submitter's address space until job completion. */
	if (!mmget_not_zero(job->mm)) {
		XDNA_ERR(xdna, "Failed to get mm reference");
		return -ESRCH;
	}

	if (op == ERT_CMD_CHAIN) {
		ret = get_cmd_bos(hwctx, job);
		if (ret)
			goto put_mm;
	}

	ret = fence_job_bos(xdna, job);
	if (ret)
		goto put_mm;

	/* Wait until this job reaches head of pending list. */
	enqueue_pending_job(hwctx, job);
	ret = wait_event_freezable(priv->job_list_wq,
				   READ_ONCE(priv->pending_head) == job);
	if (ret) {
		cancel_pending_job(hwctx, job);
		goto signal_fence;
	}

	mutex_lock(&priv->io_lock);
	ret = submit_job_cmds(hwctx, job, op);
	if (ret) {
		/* No command was published; cancel pending job and signal fence error. */
		mutex_unlock(&priv->io_lock);
		cancel_pending_job(hwctx, job);
		goto signal_fence;
	}

	/* Move in-flight or partial job to running list for worker completion. */
	list_move_tail(&job->aie4_job_list, &priv->running_job_list);
	update_pending_head(priv);
	*seq = job->seq;
	mutex_unlock(&priv->io_lock);

	/* Release the next pending submitter and kick the reaper. */
	wake_up_all(&priv->job_list_wq);
	atomic64_inc(&hwctx->job_submit_cnt);
	queue_work(priv->job_work_q, &priv->job_work);
	return 0;

signal_fence:
	/* Map -ERESTARTSYS to -ECANCELED for exported fence error status. */
	dma_fence_set_error(job->fence, ret == -ERESTARTSYS ? -ECANCELED : ret);
	dma_fence_signal(job->fence);
	dma_fence_put(job->out_fence);
	job->out_fence = NULL;
put_mm:
	put_cmd_bos(job);
	mmput(job->mm);
	return ret;
}
