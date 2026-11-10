// SPDX-License-Identifier: MIT
/*
 * Copyright © 2023 Intel Corporation
 */

#include "xe_devcoredump.h"
#include "xe_devcoredump_types.h"

#include <linux/ascii85.h>
#include <linux/devcoredump.h>
#include <linux/utsname.h>

#include <drm/drm_managed.h>

#include "regs/xe_gt_regs.h"
#include "regs/xe_guc_regs.h"

#include "xe_device.h"
#include "xe_exec_queue.h"
#include "xe_force_wake.h"
#include "xe_gt_mcr.h"
#include "xe_gt_printk.h"
#include "xe_gt_types.h"
#include "xe_guc_capture.h"
#include "xe_guc_ct.h"
#include "xe_guc_log.h"
#include "xe_guc_submit.h"
#include "xe_hw_engine.h"
#include "xe_mmio.h"
#include "xe_pm.h"
#include "xe_sched_job.h"
#include "xe_vm.h"

/**
 * DOC: Xe device coredump
 *
 * Xe uses dev_coredump infrastructure for exposing the crash errors in a
 * standardized way. Once a crash occurs, devcoredump exposes a temporary
 * node under ``/sys/class/devcoredump/devcd<m>/``. The same node is also
 * accessible in ``/sys/class/drm/card<n>/device/devcoredump/``. The
 * ``failing_device`` symlink points to the device that crashed and created the
 * coredump.
 *
 * The following characteristics are observed by xe when creating a device
 * coredump:
 *
 * **Snapshot at hang**:
 *   The 'data' file contains a snapshot of the HW and driver states at the time
 *   the hang happened. Due to the driver recovering from resets/crashes, it may
 *   not correspond to the state of the system when the file is read by
 *   userspace.
 *
 * **Coredump release**:
 *   After a coredump is generated, it stays in kernel memory until released by
 *   userspace by writing anything to it, or after an internal timer expires. The
 *   exact timeout may vary and should not be relied upon. Example to release
 *   a coredump:
 *
 *   .. code-block:: shell
 *
 *	$ > /sys/class/drm/card0/device/devcoredump/data
 *
 * **First failure only**:
 *   In general, the first hang is the most critical one since the following
 *   hangs can be a consequence of the initial hang. For this reason a snapshot
 *   is taken only for the first failure. Until the devcoredump is released by
 *   userspace or kernel, all subsequent hangs do not override the snapshot nor
 *   create new ones. Devcoredump has a delayed work queue that will eventually
 *   delete the file node and free all the dump information.
 */

#ifdef CONFIG_DEV_COREDUMP

/* 1 hour timeout */
#define XE_COREDUMP_TIMEOUT_JIFFIES (60 * 60 * HZ)

#define XE_REG_DESC(_reg, _name)	{ .reg = (_reg), .name = (_name) }
#define XE_REG_DESC_MCR(_reg, _name)   { .mcr_reg = (_reg), .name = (_name) }

static const struct xe_reg_desc xe_dbg_gt_guc_reglist[] = {
	XE_REG_DESC(GUC_STATUS,	"GUC_STATUS"),
	XE_REG_DESC(GUC_WOPCM_SIZE,	"GUC_WOPCM_SIZE"),
	XE_REG_DESC(GUC_DEVICEID,	"GUC_DEVICEID"),
	XE_REG_DESC(DMA_CTRL,	"DMA_CTRL"),
	XE_REG_DESC(GUC_HW_FATAL_ERR,	"GUC_HW_FATAL_ERR"),
	XE_REG_DESC(GUC_HW_NOTIFY_ERR,	"GUC_HW_NOTIFY_ERR"),
	XE_REG_DESC(SHIM_ERR_TRAP,	"SHIM_ERR_TRAP"),
};

static const struct xe_reg_desc xe_dbg_gt_fuse_reglist[] = {
	XE_REG_DESC(XELP_GT_GEOMETRY_DSS_ENABLE, "GT_GEOMETRY_DSS_ENABLE"),
	XE_REG_DESC(XEHP_GT_COMPUTE_DSS_ENABLE, "GT_COMPUTE_DSS_ENABLE"),
	XE_REG_DESC(XELP_EU_ENABLE, "EU_ENABLE"),
	XE_REG_DESC(MIRROR_FUSE1, "MIRROR_FUSE1"),
	XE_REG_DESC(GT_VEBOX_VDBOX_DISABLE, "GT_VEBOX_VDBOX_DISABLE"),
	XE_REG_DESC(SERVICE_COPY_ENABLE, "SERVICE_COPY_ENABLE"),
	XE_REG_DESC(RESOURCE_COPY_ENABLE, "COPY_ENABLE_GTFS_GCD"),
	XE_REG_DESC(XE2_GT_GEOMETRY_DSS_1, "GT_GEOMETRY_DSS_1"),
	XE_REG_DESC(XE2_GT_GEOMETRY_DSS_2, "GT_GEOMETRY_DSS_2"),
	XE_REG_DESC(XE2_GT_COMPUTE_DSS_2, "GT_COMPUTE_DSS_2"),
	XE_REG_DESC(XEHPC_GT_COMPUTE_DSS_ENABLE_EXT, "GT_COMPUTE_DSS_ENABLE_EXT"),
	XE_REG_DESC(XE3P_XPC_GT_GEOMETRY_DSS_3, "GT_GEOMETRY_DSS_3"),
	XE_REG_DESC(XE3P_XPC_GT_COMPUTE_DSS_3, "GT_COMPUTE_DSS_3"),
};

static const struct xe_reg_desc xe_dbg_media_gt_fuse_reglist[] = {
	XE_REG_DESC(SERVICE_COPY_ENABLE, "SERVICE_COPY_ENABLE"),
	XE_REG_DESC(RESOURCE_COPY_ENABLE, "COPY_ENABLE_GTFS_GCD"),
};

static const struct xe_reg_desc xe_dbg_gt_rpm_reglist[] = {
	XE_REG_DESC(MIRROR_PM_DEBUG, "MIRROR_PM_DEBUG"),
	XE_REG_DESC(POWERGATE_ENABLE, "POWERGATE_ENABLE"),
};

static const struct xe_reg_desc xe_dbg_media_gt_rpm_reglist[] = {
	XE_REG_DESC(MIRROR_PM_DEBUG, "MIRROR_PM_DEBUG"),
};

static const struct xe_reg_desc xe_dbg_gam_pf_reglist[] = {
	XE_REG_DESC_MCR(PAGE_FLTRPTQ_HEAD, "PAGE_FLTRPTQ_HEAD"),
	XE_REG_DESC_MCR(PAGE_FLTRPTQ_TAIL, "PAGE_FLTRPTQ_TAIL"),
};

static const struct xe_reg_desc xe_dbg_xelp_l3_reglist[] = {
	XE_REG_DESC_MCR(L3STAT_LBCF_GFX, "L3STAT_LBCF_GFX"),
};

static const struct xe_reg_desc xe_dbg_l3_node_reglist[] = {
	XE_REG_DESC_MCR(L3_NODE_IDLE, "L3_NODE_IDLE"),
	XE_REG_DESC_MCR(L3STAT_LBCF_GFX, "L3STAT_LBCF_GFX"),
};

/**
 * struct xe_gt_regset - Struct for register descriptor list for GT
 *
 * This struct is used to reference the corresponding reglist of
 * Main GT and media GT
 */
struct xe_gt_regset {
	/**
	 * @gt: GT array indexed by enum xe_gt_type starting at 1
	 *	(UNINITIALIZED=0, MAIN=1, MEDIA=2)
	 */
	struct xe_reg_desc_list gt[XE_MAX_GT_PER_TILE + 1];
};

/**
 * enum xe_dbg_regset_type - debug register set types
 * @XE_DBG_REGSET_FUSE: Fuse register set
 * @XE_DBG_REGSET_RPM: RPM register set
 * @XE_DBG_REGSET_GUC: GUC register set
 * @XE_DBG_REGSET_GAM_PF: GAM Pagefault registers set
 * @XE_DBG_REGSET_XELP_L3: Xe_LP L3 register set
 * @XE_DBG_REGSET_L3_NODE: L3 Node register Set
 * @XE_DBG_REGSET_MAX: Number of debug register set types
 */
enum xe_dbg_regset_type {
	XE_DBG_REGSET_FUSE,
	XE_DBG_REGSET_RPM,
	XE_DBG_REGSET_GUC,
	XE_DBG_REGSET_GAM_PF,
	XE_DBG_REGSET_XELP_L3,
	XE_DBG_REGSET_L3_NODE,
	XE_DBG_REGSET_MAX,
};

static const struct xe_gt_regset xe_dbg_regsets[XE_DBG_REGSET_MAX] = {
	[XE_DBG_REGSET_FUSE] = {
		.gt[XE_GT_TYPE_MAIN]  = { xe_dbg_gt_fuse_reglist,
					  ARRAY_SIZE(xe_dbg_gt_fuse_reglist) },
		.gt[XE_GT_TYPE_MEDIA] = { xe_dbg_media_gt_fuse_reglist,
					  ARRAY_SIZE(xe_dbg_media_gt_fuse_reglist) },
	},
	[XE_DBG_REGSET_RPM] = {
		.gt[XE_GT_TYPE_MAIN]  = { xe_dbg_gt_rpm_reglist,
					  ARRAY_SIZE(xe_dbg_gt_rpm_reglist) },
		.gt[XE_GT_TYPE_MEDIA] = { xe_dbg_media_gt_rpm_reglist,
					  ARRAY_SIZE(xe_dbg_media_gt_rpm_reglist) },
	},
	[XE_DBG_REGSET_GUC] = {
		.gt[XE_GT_TYPE_MAIN] = { xe_dbg_gt_guc_reglist,
					ARRAY_SIZE(xe_dbg_gt_guc_reglist) },
		.gt[XE_GT_TYPE_MEDIA] = { xe_dbg_gt_guc_reglist,
					  ARRAY_SIZE(xe_dbg_gt_guc_reglist) },
	},
	[XE_DBG_REGSET_GAM_PF] = {
		.gt[XE_GT_TYPE_MAIN] = { xe_dbg_gam_pf_reglist,
					ARRAY_SIZE(xe_dbg_gam_pf_reglist) },
	},
	[XE_DBG_REGSET_XELP_L3] = {
		.gt[XE_GT_TYPE_MAIN] = { xe_dbg_xelp_l3_reglist,
					 ARRAY_SIZE(xe_dbg_xelp_l3_reglist) },
	},
	[XE_DBG_REGSET_L3_NODE] = {
		.gt[XE_GT_TYPE_MAIN] = { xe_dbg_l3_node_reglist,
					 ARRAY_SIZE(xe_dbg_l3_node_reglist) },
	},
};

static void xe_dbg_reg_snapshot_print(struct drm_printer *p,
				      const char *title,
				      const struct xe_dbg_reg_snapshot *snap)
{
	u32 i;

	if (!snap)
		return;

	if (title)
		drm_printf(p, "\n**** %s ****\n", title);

	for (i = 0; i < snap->num_regs; i++)
		drm_printf(p, " %s [0x%x] = 0x%08x\n",
			   snap->desc->regs[i].name,
			   snap->desc->regs[i].reg.addr,
			   snap->values[i]);
}

static void xe_dbg_reg_snapshot_free(struct xe_dbg_reg_snapshot *snap)
{
	kfree(snap);
}

static const u32 *xe_dbg_find_value(const struct xe_dbg_reg_snapshot *snap,
				    struct xe_reg reg)
{
	int i;

	if (!snap)
		return NULL;

	for (i = 0; i < snap->num_regs; i++)
		if (snap->desc->regs[i].reg.addr == reg.addr)
			return &snap->values[i];

	return NULL;
}

static void xe_gam_pf_snapshot_print(struct drm_printer *p,
				     const char *title,
				     const struct xe_dbg_reg_snapshot *snap)
{
	const u32 *val1, *val2;

	xe_dbg_reg_snapshot_print(p, title, snap);

	if (!snap)
		return;

	val1 = xe_dbg_find_value(snap, PAGE_FLTRPTQ_HEAD.__reg);
	val2 = xe_dbg_find_value(snap, PAGE_FLTRPTQ_TAIL.__reg);
	if ((val1 && val2) && (*val1 != *val2))
		drm_printf(p, "PF occurred, refer to dmesg log for PF info\n");
}

static void xe_gt_rpm_snapshot_print(struct drm_printer *p,
				     const char *title,
				     const struct xe_dbg_reg_snapshot *snap)
{
	const u32 *val;

	xe_dbg_reg_snapshot_print(p, title, snap);

	if (!snap)
		return;

	val = xe_dbg_find_value(snap, POWERGATE_ENABLE);
	if (val && !(*val & RENDER_POWERGATE_ENABLE))
		drm_puts(p, "render powergating appears not enabled\n");
}

static struct xe_dbg_reg_snapshot *
xe_dbg_reg_snapshot_capture_target(struct xe_gt *gt,
				   const struct xe_reg_desc_list *reglist)
{
	struct xe_dbg_reg_snapshot *snap;
	int i;

	if (IS_SRIOV_VF(gt_to_xe(gt)))
		return NULL;

	snap = kmalloc(struct_size(snap, values, reglist->num_regs), GFP_ATOMIC);
	if (!snap)
		return NULL;

	snap->desc	= reglist;
	snap->num_regs = reglist->num_regs;
	for (i = 0; i < reglist->num_regs; i++) {
		if (reglist->regs[i].reg.mcr)
			snap->values[i] = xe_gt_mcr_unicast_read_any(gt,
								     reglist->regs[i].mcr_reg);
		else
			snap->values[i] = xe_mmio_read32(&gt->mmio, reglist->regs[i].reg);
	}

	return snap;
}

static struct xe_dbg_reg_snapshot *
xe_dbg_gt_snapshot_capture(struct xe_gt *fault_gt, struct xe_gt *other_gt,
			   enum xe_dbg_regset_type type,
			   enum xe_gt_type gt_type)
{
	struct xe_gt *target_gt;

	if (!fault_gt || !fault_gt->tile)
		return NULL;

	target_gt = gt_type == XE_GT_TYPE_MEDIA ? fault_gt->tile->media_gt
						 : fault_gt->tile->primary_gt;
	if (!target_gt)
		return NULL;

	/* non-fault GT needs the caller's confirmed forcewake */
	if (target_gt != fault_gt && target_gt != other_gt)
		return NULL;

	/* Capture GAM fault registers only where MCR steering is defined. */
	if (type == XE_DBG_REGSET_GAM_PF &&
	    (!xe_gt_mcr_check_reg(target_gt, PAGE_FLTRPTQ_HEAD.__reg) ||
	     !xe_gt_mcr_check_reg(target_gt, PAGE_FLTRPTQ_TAIL.__reg)))
		return NULL;

	return xe_dbg_reg_snapshot_capture_target(target_gt,
						  &xe_dbg_regsets[type].gt[gt_type]);
}

static struct xe_device *coredump_to_xe(const struct xe_devcoredump *coredump)
{
	return container_of(coredump, struct xe_device, devcoredump);
}

static ssize_t __xe_devcoredump_read(char *buffer, ssize_t count,
				     ssize_t start,
				     struct xe_devcoredump *coredump)
{
	struct xe_device *xe;
	struct xe_devcoredump_snapshot *ss;
	struct drm_printer p;
	struct drm_print_iterator iter;
	struct timespec64 ts;
	int i;

	xe = coredump_to_xe(coredump);
	ss = &coredump->snapshot;

	iter.data = buffer;
	iter.start = start;
	iter.remain = count;

	p = drm_coredump_printer(&iter);

	drm_puts(&p, "**** Xe Device Coredump ****\n");
	drm_printf(&p, "Reason: %s\n", ss->reason);
	drm_printf(&p, "kernel: %s\n", init_utsname()->release);
	drm_puts(&p, "module: " KBUILD_MODNAME "\n");

	ts = ktime_to_timespec64(ss->snapshot_time);
	drm_printf(&p, "Snapshot time: %ptSp\n", &ts);
	ts = ktime_to_timespec64(ss->boot_time);
	drm_printf(&p, "Uptime: %ptSp\n", &ts);
	drm_printf(&p, "Process: %s [%d]\n", ss->process_name, ss->pid);
	xe_device_snapshot_print(xe, &p);

	drm_printf(&p, "\n**** GT #%d ****\n", ss->gt->info.id);
	drm_printf(&p, "\tTile: %d\n", ss->gt->tile->id);

	xe_dbg_reg_snapshot_print(&p, "GT Fuse Register Dump", ss->gt_fuse);
	xe_dbg_reg_snapshot_print(&p, "Media GT Fuse Register Dump", ss->media_fuse);
	xe_gt_rpm_snapshot_print(&p, "GT RPM Register Dump", ss->gt_rpm);
	xe_dbg_reg_snapshot_print(&p, "Media GT RPM Register Dump", ss->media_gt_rpm);
	xe_dbg_reg_snapshot_print(&p, "GT GuC Register Dump", ss->gt_guc_reg);
	xe_dbg_reg_snapshot_print(&p, "Media GT GuC Register Dump", ss->media_gt_guc_reg);

	drm_puts(&p, "\n**** GuC Log ****\n");
	xe_guc_log_snapshot_print(ss->guc.log, &p, ss->gt);
	drm_puts(&p, "\n**** GuC CT ****\n");
	xe_guc_ct_snapshot_print(ss->guc.ct, &p, ss->gt);

	xe_gam_pf_snapshot_print(&p, "GAM PF Report", ss->gam_pf_report);

	xe_dbg_reg_snapshot_print(&p, "Node Status", ss->ns);

	drm_puts(&p, "\n**** Contexts ****\n");
	xe_guc_exec_queue_snapshot_print(ss->ge, &p);

	drm_puts(&p, "\n**** Job ****\n");
	xe_sched_job_snapshot_print(ss->job, &p);

	drm_puts(&p, "\n**** HW Engines ****\n");
	for (i = 0; i < XE_NUM_HW_ENGINES; i++)
		if (ss->hwe[i]) {
			xe_engine_snapshot_print(ss->hwe[i], &p);
			drm_puts(&p, "\n");
		}

	drm_puts(&p, "\n**** VM state ****\n");
	xe_vm_snapshot_print(ss->vm, &p);

	return count - iter.remain;
}

static void xe_devcoredump_snapshot_free(struct xe_devcoredump_snapshot *ss)
{
	int i;

	kfree(ss->reason);
	ss->reason = NULL;

	xe_guc_log_snapshot_free(ss->guc.log);
	ss->guc.log = NULL;

	xe_guc_ct_snapshot_free(ss->guc.ct);
	ss->guc.ct = NULL;

	xe_dbg_reg_snapshot_free(ss->gt_fuse);
	ss->gt_fuse = NULL;

	xe_dbg_reg_snapshot_free(ss->media_fuse);
	ss->media_fuse = NULL;

	xe_dbg_reg_snapshot_free(ss->gt_rpm);
	ss->gt_rpm = NULL;

	xe_dbg_reg_snapshot_free(ss->media_gt_rpm);
	ss->media_gt_rpm = NULL;

	xe_dbg_reg_snapshot_free(ss->gt_guc_reg);
	ss->gt_guc_reg = NULL;

	xe_dbg_reg_snapshot_free(ss->media_gt_guc_reg);
	ss->media_gt_guc_reg = NULL;

	if (!IS_ERR_OR_NULL(ss->gt))
		xe_guc_capture_put_matched_nodes(&ss->gt->uc.guc);
	ss->matched_node = NULL;

	xe_dbg_reg_snapshot_free(ss->gam_pf_report);
	ss->gam_pf_report = NULL;

	xe_dbg_reg_snapshot_free(ss->ns);
	ss->ns = NULL;

	xe_guc_exec_queue_snapshot_free(ss->ge);
	ss->ge = NULL;

	xe_sched_job_snapshot_free(ss->job);
	ss->job = NULL;

	for (i = 0; i < XE_NUM_HW_ENGINES; i++)
		if (ss->hwe[i]) {
			xe_hw_engine_snapshot_free(ss->hwe[i]);
			ss->hwe[i] = NULL;
		}

	xe_vm_snapshot_free(ss->vm);
	ss->vm = NULL;
}

#define XE_DEVCOREDUMP_CHUNK_MAX	(SZ_512M + SZ_1G)

/**
 * xe_devcoredump_read() - Read data from the Xe device coredump snapshot
 * @buffer: Destination buffer to copy the coredump data into
 * @offset: Offset in the coredump data to start reading from
 * @count: Number of bytes to read
 * @data: Pointer to the xe_devcoredump structure
 * @datalen: Length of the data (unused)
 *
 * Reads a chunk of the coredump snapshot data into the provided buffer.
 * If the devcoredump is smaller than 1.5 GB (XE_DEVCOREDUMP_CHUNK_MAX),
 * it is read directly from a pre-written buffer. For larger devcoredumps,
 * the pre-written buffer must be periodically repopulated from the snapshot
 * state due to kmalloc size limitations.
 *
 * Return: Number of bytes copied on success, or a negative error code on failure.
 */
static ssize_t xe_devcoredump_read(char *buffer, loff_t offset,
				   size_t count, void *data, size_t datalen)
{
	struct xe_devcoredump *coredump = data;
	struct xe_devcoredump_snapshot *ss;
	ssize_t byte_copied = 0;
	u32 chunk_offset;
	ssize_t new_chunk_position;
	bool pm_needed = false;
	int ret = 0;

	if (!coredump)
		return -ENODEV;

	ss = &coredump->snapshot;

	/* Ensure delayed work is captured before continuing */
	flush_work(&ss->work);

	pm_needed = ss->read.size > XE_DEVCOREDUMP_CHUNK_MAX;
	if (pm_needed)
		xe_pm_runtime_get(gt_to_xe(ss->gt));

	mutex_lock(&coredump->lock);

	if (!ss->read.buffer) {
		ret = -ENODEV;
		goto unlock;
	}

	if (offset >= ss->read.size)
		goto unlock;

	new_chunk_position = div_u64_rem(offset,
					 XE_DEVCOREDUMP_CHUNK_MAX,
					 &chunk_offset);

	if (offset >= ss->read.chunk_position + XE_DEVCOREDUMP_CHUNK_MAX ||
	    offset < ss->read.chunk_position) {
		ss->read.chunk_position = new_chunk_position *
			XE_DEVCOREDUMP_CHUNK_MAX;

		__xe_devcoredump_read(ss->read.buffer,
				      XE_DEVCOREDUMP_CHUNK_MAX,
				      ss->read.chunk_position, coredump);
	}

	byte_copied = count < ss->read.size - offset ? count :
		ss->read.size - offset;
	memcpy(buffer, ss->read.buffer + chunk_offset, byte_copied);

unlock:
	mutex_unlock(&coredump->lock);

	if (pm_needed)
		xe_pm_runtime_put(gt_to_xe(ss->gt));

	return byte_copied ? byte_copied : ret;
}

static void xe_devcoredump_free(void *data)
{
	struct xe_devcoredump *coredump = data;

	/* Our device is gone. Nothing to do... */
	if (!data || !coredump_to_xe(coredump))
		return;

	if (coredump->captured)
		cancel_work_sync(&coredump->snapshot.work);

	mutex_lock(&coredump->lock);

	xe_devcoredump_snapshot_free(&coredump->snapshot);
	kvfree(coredump->snapshot.read.buffer);

	/* To prevent stale data on next snapshot, clear everything */
	memset(&coredump->snapshot, 0, sizeof(coredump->snapshot));
	coredump->captured = false;
	drm_info(&coredump_to_xe(coredump)->drm,
		 "Xe device coredump has been deleted.\n");

	mutex_unlock(&coredump->lock);
}

static void xe_devcoredump_deferred_snap_work(struct work_struct *work)
{
	struct xe_devcoredump_snapshot *ss = container_of(work, typeof(*ss), work);
	struct xe_devcoredump *coredump = container_of(ss, typeof(*coredump), snapshot);
	struct xe_device *xe = coredump_to_xe(coredump);

	/*
	 * NB: Despite passing a GFP_ flags parameter here, more allocations are done
	 * internally using GFP_KERNEL explicitly. Hence this call must be in the worker
	 * thread and not in the initial capture call.
	 */
	dev_coredumpm_timeout(gt_to_xe(ss->gt)->drm.dev, THIS_MODULE, coredump, 0, GFP_KERNEL,
			      xe_devcoredump_read, xe_devcoredump_free,
			      XE_COREDUMP_TIMEOUT_JIFFIES);

	guard(xe_pm_runtime)(xe);

	/* keep going if fw fails as we still want to save the memory and SW data */
	xe_with_force_wake(fw_ref, gt_to_fw(ss->gt), XE_FORCEWAKE_ALL) {
		if (!xe_force_wake_ref_has_domain(fw_ref.domains, XE_FORCEWAKE_ALL))
			xe_gt_info(ss->gt, "failed to get forcewake for coredump capture\n");
		xe_vm_snapshot_capture_delayed(ss->vm);
		xe_guc_exec_queue_snapshot_capture_delayed(ss->ge);
	}

	ss->read.chunk_position = 0;

	/* Calculate devcoredump size */
	ss->read.size = __xe_devcoredump_read(NULL, LONG_MAX, 0, coredump);

	if (ss->read.size > XE_DEVCOREDUMP_CHUNK_MAX) {
		ss->read.buffer = kvmalloc(XE_DEVCOREDUMP_CHUNK_MAX,
					   GFP_USER);
		if (!ss->read.buffer)
			return;

		__xe_devcoredump_read(ss->read.buffer,
				      XE_DEVCOREDUMP_CHUNK_MAX,
				      0, coredump);
	} else {
		ss->read.buffer = kvmalloc(ss->read.size, GFP_USER);
		if (!ss->read.buffer)
			return;

		__xe_devcoredump_read(ss->read.buffer, ss->read.size, 0,
				      coredump);
		xe_devcoredump_snapshot_free(ss);
	}
}

static void xe_devcoredump_capture_gt_regs(struct xe_devcoredump_snapshot *ss,
					   struct xe_gt *gt, struct xe_gt *other_gt)
{
	enum xe_dbg_regset_type l3_regset =
		GRAPHICS_VERx100(gt_to_xe(gt)) < 1250 ?
		XE_DBG_REGSET_XELP_L3 : XE_DBG_REGSET_L3_NODE;

	ss->gt_fuse      = xe_dbg_gt_snapshot_capture(gt,
						      other_gt,
						      XE_DBG_REGSET_FUSE,
						      XE_GT_TYPE_MAIN);
	ss->media_fuse   = xe_dbg_gt_snapshot_capture(gt,
						      other_gt,
						      XE_DBG_REGSET_FUSE,
						      XE_GT_TYPE_MEDIA);
	ss->gt_rpm       = xe_dbg_gt_snapshot_capture(gt,
						      other_gt,
						      XE_DBG_REGSET_RPM,
						      XE_GT_TYPE_MAIN);
	ss->media_gt_rpm = xe_dbg_gt_snapshot_capture(gt,
						      other_gt,
						      XE_DBG_REGSET_RPM,
						      XE_GT_TYPE_MEDIA);
	ss->gt_guc_reg = xe_dbg_gt_snapshot_capture(gt,
						    other_gt,
						    XE_DBG_REGSET_GUC,
						    XE_GT_TYPE_MAIN);
	ss->media_gt_guc_reg = xe_dbg_gt_snapshot_capture(gt,
							  other_gt,
							  XE_DBG_REGSET_GUC,
							  XE_GT_TYPE_MEDIA);
	ss->gam_pf_report = xe_dbg_gt_snapshot_capture(gt,
						       other_gt,
						       XE_DBG_REGSET_GAM_PF,
						       XE_GT_TYPE_MAIN);
	ss->ns = xe_dbg_gt_snapshot_capture(gt,
					    other_gt,
					    l3_regset,
					    XE_GT_TYPE_MAIN);
}

static void devcoredump_snapshot(struct xe_devcoredump *coredump,
				 struct xe_gt *gt,
				 struct xe_exec_queue *q,
				 struct xe_sched_job *job)
{
	struct xe_devcoredump_snapshot *ss = &coredump->snapshot;
	struct xe_guc *guc = &gt->uc.guc;
	const char *process_name = "no process";
	bool cookie;
	struct xe_gt *other_gt;

	ss->snapshot_time = ktime_get_real();
	ss->boot_time = ktime_get_boottime();

	if (q && q->vm && q->vm->xef) {
		process_name = q->vm->xef->process_name;
		ss->pid = q->vm->xef->pid;
	}

	strscpy(ss->process_name, process_name);

	ss->gt = gt;
	INIT_WORK(&ss->work, xe_devcoredump_deferred_snap_work);

	CLASS(xe_force_wake, fw_ref)(gt_to_fw(gt), XE_FORCEWAKE_ALL);

	cookie = dma_fence_begin_signalling();

	other_gt = gt->tile->media_gt == gt ? gt->tile->primary_gt
					    : gt->tile->media_gt;
	if (other_gt) {
		/* fw_ref_other must stay in scope through the register captures below */
		CLASS(xe_force_wake, fw_ref_other)(gt_to_fw(other_gt),
						   XE_FORCEWAKE_ALL);
		if (!xe_force_wake_ref_has_domain(fw_ref_other.domains, XE_FORCEWAKE_ALL))
			other_gt = NULL;
		xe_devcoredump_capture_gt_regs(ss, gt, other_gt);
	} else {
		xe_devcoredump_capture_gt_regs(ss, gt, NULL);
	}

	ss->guc.log = xe_guc_log_snapshot_capture(&guc->log, true);
	ss->guc.ct = xe_guc_ct_snapshot_capture(&guc->ct);

	if (q) {
		ss->ge = xe_guc_exec_queue_snapshot_capture(q);
		if (job)
			ss->job = xe_sched_job_snapshot_capture(job);
		ss->vm = xe_vm_snapshot_capture(q->vm);
		xe_engine_snapshot_capture_for_queue(q);
	}

	queue_work(system_dfl_wq, &ss->work);

	dma_fence_end_signalling(cookie);
}

/**
 * __xe_devcoredump - Take the required snapshots and initialize coredump device.
 * @gt: The GT where the issue was detected.
 * @q: The faulty xe_exec_queue, where the issue was detected, may be NULL for
 *     hangs that are not tied to an exec queue (e.g. TLB invalidation
 *     timeouts); in that case only the GT-level state (GuC log and CT state)
 *     is captured.
 * @job: The faulty xe_sched_job, where the issue was detected, may be NULL.
 * @fmt: Printf format + args to describe the reason for the core dump
 *
 * This function should be called at the crash time within the serialized
 * gt_reset. It is skipped if we still have the core dump device available
 * with the information of the 'first' snapshot.
 */
__printf(4, 5)
void __xe_devcoredump(struct xe_gt *gt, struct xe_exec_queue *q,
		      struct xe_sched_job *job, const char *fmt, ...)
{
	struct xe_device *xe = gt_to_xe(gt);
	struct xe_devcoredump *coredump = &xe->devcoredump;
	va_list varg;

	mutex_lock(&coredump->lock);

	if (coredump->captured) {
		drm_dbg(&xe->drm, "Multiple hangs are occurring, but only the first snapshot was taken\n");
		mutex_unlock(&coredump->lock);
		return;
	}

	coredump->captured = true;

	va_start(varg, fmt);
	coredump->snapshot.reason = kvasprintf(GFP_ATOMIC, fmt, varg);
	va_end(varg);

	devcoredump_snapshot(coredump, gt, q, job);

	drm_info(&xe->drm, "Xe device coredump has been created\n");
	drm_info(&xe->drm, "Check your /sys/class/drm/card%d/device/devcoredump/data\n",
		 xe->drm.primary->index);

	mutex_unlock(&coredump->lock);
}

static void xe_driver_devcoredump_fini(void *arg)
{
	struct drm_device *drm = arg;

	dev_coredump_put(drm->dev);
}

int xe_devcoredump_init(struct xe_device *xe)
{
	int err;

	err = drmm_mutex_init(&xe->drm, &xe->devcoredump.lock);
	if (err)
		return err;

	if (IS_ENABLED(CONFIG_LOCKDEP)) {
		fs_reclaim_acquire(GFP_KERNEL);
		might_lock(&xe->devcoredump.lock);
		fs_reclaim_release(GFP_KERNEL);
	}

	return devm_add_action_or_reset(xe->drm.dev, xe_driver_devcoredump_fini, &xe->drm);
}

#endif

/**
 * xe_print_blob_ascii85 - print a BLOB to some useful location in ASCII85
 *
 * The output is split into multiple calls to drm_puts() because some print
 * targets, e.g. dmesg, cannot handle arbitrarily long lines. These targets may
 * add newlines, as is the case with dmesg: each drm_puts() call creates a
 * separate line.
 *
 * There is also a scheduler yield call to prevent the 'task has been stuck for
 * 120s' kernel hang check feature from firing when printing to a slow target
 * such as dmesg over a serial port.
 *
 * @p: the printer object to output to
 * @prefix: optional prefix to add to output string
 * @suffix: optional suffix to add at the end. 0 disables it and is
 *          not added to the output, which is useful when using multiple calls
 *          to dump data to @p
 * @blob: the Binary Large OBject to dump out
 * @offset: offset in bytes to skip from the front of the BLOB, must be a multiple of sizeof(u32)
 * @size: the size in bytes of the BLOB, must be a multiple of sizeof(u32)
 */
void xe_print_blob_ascii85(struct drm_printer *p, const char *prefix, char suffix,
			   const void *blob, size_t offset, size_t size)
{
	const u32 *blob32 = (const u32 *)blob;
	char buff[ASCII85_BUFSZ], *line_buff;
	size_t line_pos = 0;

#define DMESG_MAX_LINE_LEN	800
	/* Always leave space for the suffix char and the \0 */
#define MIN_SPACE		(ASCII85_BUFSZ + 2)	/* 85 + "<suffix>\0" */

	if (size & 3)
		drm_printf(p, "Size not word aligned: %zu", size);
	if (offset & 3)
		drm_printf(p, "Offset not word aligned: %zu", offset);

	line_buff = kzalloc(DMESG_MAX_LINE_LEN, GFP_ATOMIC);
	if (!line_buff) {
		drm_printf(p, "Failed to allocate line buffer\n");
		return;
	}

	blob32 += offset / sizeof(*blob32);
	size /= sizeof(*blob32);

	if (prefix) {
		strscpy(line_buff, prefix, DMESG_MAX_LINE_LEN - MIN_SPACE - 2);
		line_pos = strlen(line_buff);

		line_buff[line_pos++] = ':';
		line_buff[line_pos++] = ' ';
	}

	while (size--) {
		u32 val = *(blob32++);

		strscpy(line_buff + line_pos, ascii85_encode(val, buff),
			DMESG_MAX_LINE_LEN - line_pos);
		line_pos += strlen(line_buff + line_pos);

		if ((line_pos + MIN_SPACE) >= DMESG_MAX_LINE_LEN) {
			line_buff[line_pos++] = 0;

			drm_puts(p, line_buff);

			line_pos = 0;

			/* Prevent 'stuck thread' time out errors */
			cond_resched();
		}
	}

	if (suffix)
		line_buff[line_pos++] = suffix;

	if (line_pos) {
		line_buff[line_pos++] = 0;
		drm_puts(p, line_buff);
	}

	kfree(line_buff);

#undef MIN_SPACE
#undef DMESG_MAX_LINE_LEN
}
