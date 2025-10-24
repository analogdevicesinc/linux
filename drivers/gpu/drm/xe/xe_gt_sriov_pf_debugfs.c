// SPDX-License-Identifier: MIT
/*
 * Copyright © 2023-2024 Intel Corporation
 */

#include <linux/debugfs.h>

#include <drm/drm_print.h>
#include <drm/drm_debugfs.h>

#include "xe_bo.h"
#include "xe_debugfs.h"
#include "xe_device.h"
#include "xe_gt.h"
#include "xe_gt_debugfs.h"
#include "xe_gt_sriov_pf_config.h"
#include "xe_gt_sriov_pf_control.h"
#include "xe_gt_sriov_pf_debugfs.h"
#include "xe_gt_sriov_pf_helpers.h"
#include "xe_gt_sriov_pf_migration.h"
#include "xe_gt_sriov_pf_monitor.h"
#include "xe_gt_sriov_pf_policy.h"
#include "xe_gt_sriov_pf_service.h"
#include "xe_pm.h"
#include "xe_sriov_pf.h"
#include "xe_sriov_pf_provision.h"

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov		# d_inode->i_private = (xe_device*)
 *      │   ├── pf		# d_inode->i_private = (xe_device*)
 *      │   │   ├── tile0	# d_inode->i_private = (xe_tile*)
 *      │   │   │   ├── gt0	# d_inode->i_private = (xe_gt*)
 *      │   │   │   ├── gt1	# d_inode->i_private = (xe_gt*)
 *      │   │   ├── tile1
 *      │   │   │   :
 *      │   ├── vf1		# d_inode->i_private = VFID(1)
 *      │   │   ├── tile0	# d_inode->i_private = (xe_tile*)
 *      │   │   │   ├── gt0	# d_inode->i_private = (xe_gt*)
 *      │   │   │   ├── gt1	# d_inode->i_private = (xe_gt*)
 *      │   │   ├── tile1
 *      │   │   │   :
 *      :   :
 *      │   ├── vfN		# d_inode->i_private = VFID(N)
 */

static void *extract_priv(struct dentry *d)
{
	return d->d_inode->i_private;
}

static struct xe_gt *extract_gt(struct dentry *d)
{
	return extract_priv(d);
}

static struct xe_device *extract_xe(struct dentry *d)
{
	return extract_priv(d->d_parent->d_parent->d_parent);
}

static unsigned int extract_vfid(struct dentry *d)
{
	void *priv = extract_priv(d->d_parent->d_parent);

	return priv == extract_xe(d) ? PFID : (uintptr_t)priv;
}

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── pf
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── contexts_provisioned
 *                      ├── doorbells_provisioned
 *                      ├── runtime_registers
 *                      ├── adverse_events
 */

static const struct drm_info_list pf_info[] = {
	{
		"contexts_provisioned",
		.show = xe_gt_debugfs_simple_show,
		.data = xe_gt_sriov_pf_config_print_ctxs,
	},
	{
		"doorbells_provisioned",
		.show = xe_gt_debugfs_simple_show,
		.data = xe_gt_sriov_pf_config_print_dbs,
	},
	{
		"runtime_registers",
		.show = xe_gt_debugfs_simple_show,
		.data = xe_gt_sriov_pf_service_print_runtime,
	},
	{
		"adverse_events",
		.show = xe_gt_debugfs_simple_show,
		.data = xe_gt_sriov_pf_monitor_print_events,
	},
};

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── pf
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── reset_engine
 *                      ├── sample_period
 *                      ├── sched_if_idle
 */

#define DEFINE_SRIOV_GT_POLICY_DEBUGFS_ATTRIBUTE(POLICY, TYPE, FORMAT)		\
										\
static int POLICY##_set(void *data, u64 val)					\
{										\
	struct xe_gt *gt = extract_gt(data);					\
	struct xe_device *xe = gt_to_xe(gt);					\
	int err;								\
										\
	if (val > (TYPE)~0ull)							\
		return -EOVERFLOW;						\
										\
	xe_pm_runtime_get(xe);							\
	err = xe_gt_sriov_pf_policy_set_##POLICY(gt, val);			\
	if (!err)								\
		xe_sriov_pf_provision_set_custom_mode(xe);			\
	xe_pm_runtime_put(xe);							\
										\
	return err;								\
}										\
										\
static int POLICY##_get(void *data, u64 *val)					\
{										\
	struct xe_gt *gt = extract_gt(data);					\
										\
	*val = xe_gt_sriov_pf_policy_get_##POLICY(gt);				\
	return 0;								\
}										\
										\
DEFINE_DEBUGFS_ATTRIBUTE(POLICY##_fops, POLICY##_get, POLICY##_set, FORMAT)

DEFINE_SRIOV_GT_POLICY_DEBUGFS_ATTRIBUTE(reset_engine, bool, "%llu\n");
DEFINE_SRIOV_GT_POLICY_DEBUGFS_ATTRIBUTE(sched_if_idle, bool, "%llu\n");
DEFINE_SRIOV_GT_POLICY_DEBUGFS_ATTRIBUTE(sample_period, u32, "%llu\n");

static void pf_add_policy_attrs(struct xe_gt *gt, struct dentry *parent)
{
	xe_gt_assert(gt, gt == extract_gt(parent));
	xe_gt_assert(gt, PFID == extract_vfid(parent));

	debugfs_create_file_unsafe("reset_engine", 0644, parent, parent, &reset_engine_fops);
	debugfs_create_file_unsafe("sched_if_idle", 0644, parent, parent, &sched_if_idle_fops);
	debugfs_create_file_unsafe("sample_period_ms", 0644, parent, parent, &sample_period_fops);
}

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── pf
 *          │   ├── tile0
 *          │   :   ├── gt0
 *          │       :   ├── doorbells_spare
 *          │           ├── contexts_spare
 *          │           ├── exec_quantum_ms
 *          │           ├── preempt_timeout_us
 *          │           ├── sched_priority
 *          ├── vf1
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── doorbells_quota
 *                      ├── contexts_quota
 *                      ├── exec_quantum_ms
 *                      ├── preempt_timeout_us
 *                      ├── sched_priority
 */

#define DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(CONFIG, TYPE, FORMAT)		\
										\
static int CONFIG##_set(void *data, u64 val)					\
{										\
	struct xe_gt *gt = extract_gt(data);					\
	unsigned int vfid = extract_vfid(data);					\
	struct xe_device *xe = gt_to_xe(gt);					\
	int err;								\
										\
	if (val > (TYPE)~0ull)							\
		return -EOVERFLOW;						\
										\
	xe_pm_runtime_get(xe);							\
	err = xe_sriov_pf_wait_ready(xe) ?:					\
	      xe_gt_sriov_pf_config_set_##CONFIG(gt, vfid, val);		\
	if (!err)								\
		xe_sriov_pf_provision_set_custom_mode(xe);			\
	xe_pm_runtime_put(xe);							\
										\
	return err;								\
}										\
										\
static int CONFIG##_get(void *data, u64 *val)					\
{										\
	struct xe_gt *gt = extract_gt(data);					\
	unsigned int vfid = extract_vfid(data);					\
										\
	*val = xe_gt_sriov_pf_config_get_##CONFIG(gt, vfid);			\
	return 0;								\
}										\
										\
DEFINE_DEBUGFS_ATTRIBUTE(CONFIG##_fops, CONFIG##_get, CONFIG##_set, FORMAT)

DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(ctxs, u32, "%llu\n");
DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(dbs, u32, "%llu\n");
DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(exec_quantum, u32, "%llu\n");
DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(preempt_timeout, u32, "%llu\n");
DEFINE_SRIOV_GT_CONFIG_DEBUGFS_ATTRIBUTE(sched_priority, u32, "%llu\n");

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── pf
 *          │   ├── tile0
 *          │   :   ├── gt0
 *          │       :   ├── threshold_cat_error_count
 *          │           ├── threshold_doorbell_time_us
 *          │           ├── threshold_engine_reset_count
 *          │           ├── threshold_guc_time_us
 *          │           ├── threshold_irq_time_us
 *          │           ├── threshold_page_fault_count
 *          ├── vf1
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── threshold_cat_error_count
 *                      ├── threshold_doorbell_time_us
 *                      ├── threshold_engine_reset_count
 *                      ├── threshold_guc_time_us
 *                      ├── threshold_irq_time_us
 *                      ├── threshold_page_fault_count
 */

static int set_threshold(void *data, u64 val, enum xe_guc_klv_threshold_index index)
{
	struct xe_gt *gt = extract_gt(data);
	unsigned int vfid = extract_vfid(data);
	struct xe_device *xe = gt_to_xe(gt);
	int err;

	if (val > (u32)~0ull)
		return -EOVERFLOW;

	xe_pm_runtime_get(xe);
	err = xe_gt_sriov_pf_config_set_threshold(gt, vfid, index, val);
	if (!err)
		xe_sriov_pf_provision_set_custom_mode(xe);
	xe_pm_runtime_put(xe);

	return err;
}

static int get_threshold(void *data, u64 *val, enum xe_guc_klv_threshold_index index)
{
	struct xe_gt *gt = extract_gt(data);
	unsigned int vfid = extract_vfid(data);

	*val = xe_gt_sriov_pf_config_get_threshold(gt, vfid, index);
	return 0;
}

#define DEFINE_SRIOV_GT_THRESHOLD_DEBUGFS_ATTRIBUTE(THRESHOLD, INDEX)		\
										\
static int THRESHOLD##_set(void *data, u64 val)					\
{										\
	return set_threshold(data, val, INDEX);					\
}										\
										\
static int THRESHOLD##_get(void *data, u64 *val)				\
{										\
	return get_threshold(data, val, INDEX);					\
}										\
										\
DEFINE_DEBUGFS_ATTRIBUTE(THRESHOLD##_fops, THRESHOLD##_get, THRESHOLD##_set, "%llu\n")

/* generate all threshold attributes */
#define define_threshold_attribute(TAG, NAME, ...) \
	DEFINE_SRIOV_GT_THRESHOLD_DEBUGFS_ATTRIBUTE(NAME, MAKE_XE_GUC_KLV_THRESHOLD_INDEX(TAG));
MAKE_XE_GUC_KLV_THRESHOLDS_SET(define_threshold_attribute)
#undef define_threshold_attribute

static void pf_add_config_attrs(struct xe_gt *gt, struct dentry *parent, unsigned int vfid)
{
	xe_gt_assert(gt, gt == extract_gt(parent));
	xe_gt_assert(gt, vfid == extract_vfid(parent));

	debugfs_create_file_unsafe(vfid ? "doorbells_quota" : "doorbells_spare",
				   0644, parent, parent, &dbs_fops);
	debugfs_create_file_unsafe(vfid ? "contexts_quota" : "contexts_spare",
				   0644, parent, parent, &ctxs_fops);
	debugfs_create_file_unsafe("exec_quantum_ms", 0644, parent, parent,
				   &exec_quantum_fops);
	debugfs_create_file_unsafe("preempt_timeout_us", 0644, parent, parent,
				   &preempt_timeout_fops);
	debugfs_create_file_unsafe("sched_priority", 0644, parent, parent,
				   &sched_priority_fops);

	/* register all threshold attributes */
#define register_threshold_attribute(TAG, NAME, ...) \
	debugfs_create_file_unsafe("threshold_" #NAME, 0644, parent, parent, \
				   &NAME##_fops);
	MAKE_XE_GUC_KLV_THRESHOLDS_SET(register_threshold_attribute)
#undef register_threshold_attribute
}

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── vf1
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── control { stop, pause, resume }
 */

static const struct {
	const char *cmd;
	int (*fn)(struct xe_gt *gt, unsigned int vfid);
} control_cmds[] = {
	{ "stop", xe_gt_sriov_pf_control_stop_vf },
	{ "pause", xe_gt_sriov_pf_control_pause_vf },
	{ "resume", xe_gt_sriov_pf_control_resume_vf },
#ifdef CONFIG_DRM_XE_DEBUG_SRIOV
	{ "restore!", xe_gt_sriov_pf_migration_restore_guc_state },
#endif
};

static ssize_t control_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	struct dentry *dent = file_dentry(file);
	struct dentry *parent = dent->d_parent;
	struct xe_gt *gt = extract_gt(parent);
	struct xe_device *xe = gt_to_xe(gt);
	unsigned int vfid = extract_vfid(parent);
	int ret = -EINVAL;
	char cmd[32];
	size_t n;

	xe_gt_assert(gt, vfid);
	xe_gt_sriov_pf_assert_vfid(gt, vfid);

	if (*pos)
		return -ESPIPE;

	if (count > sizeof(cmd) - 1)
		return -EINVAL;

	ret = simple_write_to_buffer(cmd, sizeof(cmd) - 1, pos, buf, count);
	if (ret < 0)
		return ret;
	cmd[ret] = '\0';

	for (n = 0; n < ARRAY_SIZE(control_cmds); n++) {
		xe_gt_assert(gt, sizeof(cmd) > strlen(control_cmds[n].cmd));

		if (sysfs_streq(cmd, control_cmds[n].cmd)) {
			xe_pm_runtime_get(xe);
			ret = control_cmds[n].fn ? (*control_cmds[n].fn)(gt, vfid) : 0;
			xe_pm_runtime_put(xe);
			break;
		}
	}

	return (ret < 0) ? ret : count;
}

static ssize_t control_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char help[128];
	size_t n;

	help[0] = '\0';
	for (n = 0; n < ARRAY_SIZE(control_cmds); n++) {
		strlcat(help, control_cmds[n].cmd, sizeof(help));
		strlcat(help, "\n", sizeof(help));
	}

	return simple_read_from_buffer(buf, count, ppos, help, strlen(help));
}

static const struct file_operations control_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.write		= control_write,
	.read		= control_read,
	.llseek		= default_llseek,
};

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── vf1
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── guc_state
 */

static ssize_t guc_state_read(struct file *file, char __user *buf,
			      size_t count, loff_t *pos)
{
	struct dentry *dent = file_dentry(file);
	struct dentry *parent = dent->d_parent;
	struct xe_gt *gt = extract_gt(parent);
	unsigned int vfid = extract_vfid(parent);

	return xe_gt_sriov_pf_migration_read_guc_state(gt, vfid, buf, count, pos);
}

static ssize_t guc_state_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *pos)
{
	struct dentry *dent = file_dentry(file);
	struct dentry *parent = dent->d_parent;
	struct xe_gt *gt = extract_gt(parent);
	unsigned int vfid = extract_vfid(parent);

	if (*pos)
		return -EINVAL;

	return xe_gt_sriov_pf_migration_write_guc_state(gt, vfid, buf, count);
}

static const struct file_operations guc_state_ops = {
	.owner		= THIS_MODULE,
	.read		= guc_state_read,
	.write		= guc_state_write,
	.llseek		= default_llseek,
};

/*
 *      /sys/kernel/debug/dri/BDF/
 *      ├── sriov
 *      :   ├── vf1
 *          :   ├── tile0
 *              :   ├── gt0
 *                  :   ├── config_blob
 */

struct config_blob_data {
	size_t size;
	u8 blob[];
};

static int config_blob_open(struct inode *inode, struct file *file)
{
	struct dentry *dent = file_dentry(file);
	struct dentry *parent = dent->d_parent;
	struct xe_gt *gt = extract_gt(parent);
	unsigned int vfid = extract_vfid(parent);
	struct config_blob_data *cbd;
	ssize_t ret;

	ret = xe_gt_sriov_pf_config_save(gt, vfid, NULL, 0);
	if (!ret)
		return -ENODATA;
	if (ret < 0)
		return ret;

	cbd = kzalloc(struct_size(cbd, blob, ret), GFP_KERNEL);
	if (!cbd)
		return -ENOMEM;

	ret = xe_gt_sriov_pf_config_save(gt, vfid, cbd->blob, ret);
	if (ret < 0) {
		kfree(cbd);
		return ret;
	}

	cbd->size = ret;
	file->private_data = cbd;
	return nonseekable_open(inode, file);
}

static ssize_t config_blob_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct config_blob_data *cbd = file->private_data;

	return simple_read_from_buffer(buf, count, pos, cbd->blob, cbd->size);
}

static ssize_t config_blob_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *pos)
{
	struct dentry *dent = file_dentry(file);
	struct dentry *parent = dent->d_parent;
	struct xe_gt *gt = extract_gt(parent);
	unsigned int vfid = extract_vfid(parent);
	ssize_t ret;
	void *tmp;

	if (*pos)
		return -EINVAL;

	if (!count)
		return -ENODATA;

	if (count > SZ_4K)
		return -EINVAL;

	tmp = kzalloc(count, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	if (copy_from_user(tmp, buf, count)) {
		ret = -EFAULT;
	} else {
		ret = xe_gt_sriov_pf_config_restore(gt, vfid, tmp, count);
		if (!ret)
			ret = count;
	}
	kfree(tmp);
	return ret;
}

static int config_blob_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static const struct file_operations config_blob_ops = {
	.owner		= THIS_MODULE,
	.open		= config_blob_open,
	.read		= config_blob_read,
	.write		= config_blob_write,
	.release	= config_blob_release,
};

static void pf_add_compat_attrs(struct xe_gt *gt, struct dentry *dent, unsigned int vfid)
{
	struct xe_device *xe = gt_to_xe(gt);

	if (!xe_gt_is_main_type(gt))
		return;

	if (vfid) {
		debugfs_create_symlink("ggtt_quota", dent, "../ggtt_quota");
		if (xe_device_has_lmtt(xe))
			debugfs_create_symlink("lmem_quota", dent, "../vram_quota");
	} else {
		debugfs_create_symlink("ggtt_spare", dent, "../ggtt_spare");
		debugfs_create_symlink("ggtt_available", dent, "../ggtt_available");
		debugfs_create_symlink("ggtt_provisioned", dent, "../ggtt_provisioned");
		if (xe_device_has_lmtt(xe)) {
			debugfs_create_symlink("lmem_spare", dent, "../vram_spare");
			debugfs_create_symlink("lmem_provisioned", dent, "../vram_provisioned");
		}
	}
}

static void pf_populate_gt(struct xe_gt *gt, struct dentry *dent, unsigned int vfid)
{
	struct xe_device *xe = gt_to_xe(gt);
	struct drm_minor *minor = xe->drm.primary;

	if (vfid) {
		pf_add_config_attrs(gt, dent, vfid);

		debugfs_create_file("control", 0600, dent, NULL, &control_ops);

		/* for testing/debugging purposes only! */
		if (IS_ENABLED(CONFIG_DRM_XE_DEBUG)) {
			debugfs_create_file("guc_state",
					    IS_ENABLED(CONFIG_DRM_XE_DEBUG_SRIOV) ? 0600 : 0400,
					    dent, NULL, &guc_state_ops);
			debugfs_create_file("config_blob",
					    IS_ENABLED(CONFIG_DRM_XE_DEBUG_SRIOV) ? 0600 : 0400,
					    dent, NULL, &config_blob_ops);
		}

	} else {
		pf_add_config_attrs(gt, dent, PFID);
		pf_add_policy_attrs(gt, dent);

		drm_debugfs_create_files(pf_info, ARRAY_SIZE(pf_info), dent, minor);
	}

	/* for backward compatibility only */
	pf_add_compat_attrs(gt, dent, vfid);
}

/**
 * xe_gt_sriov_pf_debugfs_populate() - Create SR-IOV GT-level debugfs directories and files.
 * @gt: the &xe_gt to register
 * @parent: the parent &dentry that represents a &xe_tile
 * @vfid: the VF identifier
 *
 * Add to the @parent directory new debugfs directory that will represent a @gt and
 * populate it with GT files that are related to the SR-IOV @vfid function.
 *
 * This function can only be called on PF.
 */
void xe_gt_sriov_pf_debugfs_populate(struct xe_gt *gt, struct dentry *parent, unsigned int vfid)
{
	struct dentry *dent;
	char name[8]; /* should be enough up to "gt%u\0" for 2^8 - 1 */

	xe_gt_assert(gt, IS_SRIOV_PF(gt_to_xe(gt)));
	xe_gt_assert(gt, extract_priv(parent) == gt->tile);
	xe_gt_assert(gt, extract_priv(parent->d_parent) == gt_to_xe(gt) ||
		     (uintptr_t)extract_priv(parent->d_parent) == vfid);

	/*
	 *      /sys/kernel/debug/dri/BDF/
	 *      ├── sriov
	 *      │   ├── pf
	 *      │   │   ├── tile0		# parent
	 *      │   │   │   ├── gt0		# d_inode->i_private = (xe_gt*)
	 *      │   │   │   ├── gt1
	 *      │   │   :   :
	 *      │   ├── vf1
	 *      │   │   ├── tile0		# parent
	 *      │   │   │   ├── gt0		# d_inode->i_private = (xe_gt*)
	 *      │   │   │   ├── gt1
	 *      │   :   :   :
	 */
	snprintf(name, sizeof(name), "gt%u", gt->info.id);
	dent = debugfs_create_dir(name, parent);
	if (IS_ERR(dent))
		return;
	dent->d_inode->i_private = gt;

	xe_gt_assert(gt, extract_gt(dent) == gt);
	xe_gt_assert(gt, extract_vfid(dent) == vfid);

	pf_populate_gt(gt, dent, vfid);
}

static void pf_add_links(struct xe_gt *gt, struct dentry *dent)
{
	unsigned int totalvfs = xe_gt_sriov_pf_get_totalvfs(gt);
	unsigned int vfid;
	char name[16];		/* should be more than enough for "vf%u\0" and VFID(UINT_MAX) */
	char symlink[64];	/* should be more enough for "../../sriov/vf%u/tile%u/gt%u\0" */

	for (vfid = 0; vfid <= totalvfs; vfid++) {
		if (vfid)
			snprintf(name, sizeof(name), "vf%u", vfid);
		else
			snprintf(name, sizeof(name), "pf");
		snprintf(symlink, sizeof(symlink), "../../sriov/%s/tile%u/gt%u",
			 name, gt->tile->id, gt->info.id);
		debugfs_create_symlink(name, dent, symlink);
	}
}

/**
 * xe_gt_sriov_pf_debugfs_register - Register SR-IOV PF specific entries in GT debugfs.
 * @gt: the &xe_gt to register
 * @dent: the &dentry that represents the GT directory
 *
 * Instead of actual files, create symlinks for PF and each VF to their GT specific
 * attributes that should be already exposed in the dedicated debugfs SR-IOV tree.
 */
void xe_gt_sriov_pf_debugfs_register(struct xe_gt *gt, struct dentry *dent)
{
	xe_gt_assert(gt, IS_SRIOV_PF(gt_to_xe(gt)));
	xe_gt_assert(gt, dent->d_inode->i_private == gt);

	pf_add_links(gt, dent);
}
