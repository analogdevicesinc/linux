// SPDX-License-Identifier: MIT
/*
 * Copyright © 2023-2024 Intel Corporation
 */

#include <drm/drm_debugfs.h>
#include <drm/drm_managed.h>

#include "xe_gt.h"
#include "xe_gt_sriov_vf.h"
#include "xe_guc.h"
#include "xe_sriov_printk.h"
#include "xe_sriov_vf.h"
#include "xe_sriov_vf_ccs.h"

/**
 * DOC: VF restore procedure in PF KMD and VF KMD
 *
 * Restoring previously saved state of a VF is one of core features of
 * SR-IOV. All major VM Management applications allow saving and restoring
 * the VM state, and doing that to a VM which uses SRIOV VF as one of
 * the accessible devices requires support from KMD on both PF and VF side.
 * VMM initiates all required operations through VFIO module, which then
 * translates them into PF KMD calls. This description will focus on these
 * calls, leaving out the module which initiates these steps (VFIO).
 *
 * In order to start the restore procedure, GuC needs to keep the VF in
 * proper state. The PF driver can ensure GuC set it to VF_READY state
 * by provisioning the VF, which in turn can be done after Function Level
 * Reset of said VF (or after it was freshly created - in that case FLR
 * is not needed). The FLR procedure ends with GuC sending message
 * `GUC_PF_NOTIFY_VF_FLR_DONE`, and then provisioning data is sent to GuC.
 * After the provisioning is completed, the VF needs to be paused, and
 * at that point the actual restore can begin.
 *
 * During VF Restore, state of several resources is restored. These may
 * include local memory content (system memory is restored by VMM itself),
 * values of MMIO registers, stateless compression metadata and others.
 * The final resource which also needs restoring is state of the VF
 * submission maintained within GuC. For that, `GUC_PF_OPCODE_VF_RESTORE`
 * message is used, with reference to the state blob to be consumed by
 * GuC.
 *
 * Next, when VFIO is asked to set the VM into running state, the PF driver
 * sends `GUC_PF_TRIGGER_VF_RESUME` to GuC. When sent after restore, this
 * changes VF state within GuC to `VF_RESFIX_BLOCKED` rather than the
 * usual `VF_RUNNING`. At this point GuC triggers an interrupt to inform
 * the VF KMD within the VM that it was migrated.
 *
 * As soon as Virtual GPU of the VM starts, the VF driver within receives
 * the MIGRATED interrupt and schedules post-migration recovery worker.
 * That worker queries GuC for new provisioning (using MMIO communication),
 * and applies fixups to any non-virtualized resources used by the VF.
 *
 * When the VF driver is ready to continue operation on the newly connected
 * hardware, it sends `VF2GUC_NOTIFY_RESFIX_DONE` which causes it to
 * enter the long awaited `VF_RUNNING` state, and therefore start handling
 * CTB messages and scheduling workloads from the VF::
 *
 *      PF                             GuC                              VF
 *     [ ]                              |                               |
 *     [ ] PF2GUC_VF_CONTROL(pause)     |                               |
 *     [ ]---------------------------> [ ]                              |
 *     [ ]                             [ ]  GuC sets new VF state to    |
 *     [ ]                             [ ]------- VF_READY_PAUSED       |
 *     [ ]                             [ ]      |                       |
 *     [ ]                             [ ] <-----                       |
 *     [ ] success                     [ ]                              |
 *     [ ] <---------------------------[ ]                              |
 *     [ ]                              |                               |
 *     [ ] PF loads resources from the  |                               |
 *     [ ]------- saved image supplied  |                               |
 *     [ ]      |                       |                               |
 *     [ ] <-----                       |                               |
 *     [ ]                              |                               |
 *     [ ] GUC_PF_OPCODE_VF_RESTORE     |                               |
 *     [ ]---------------------------> [ ]                              |
 *     [ ]                             [ ]  GuC loads contexts and CTB  |
 *     [ ]                             [ ]------- state from image      |
 *     [ ]                             [ ]      |                       |
 *     [ ]                             [ ] <-----                       |
 *     [ ]                             [ ]                              |
 *     [ ]                             [ ]  GuC sets new VF state to    |
 *     [ ]                             [ ]------- VF_RESFIX_PAUSED      |
 *     [ ]                             [ ]      |                       |
 *     [ ] success                     [ ] <-----                       |
 *     [ ] <---------------------------[ ]                              |
 *     [ ]                              |                               |
 *     [ ] GUC_PF_TRIGGER_VF_RESUME     |                               |
 *     [ ]---------------------------> [ ]                              |
 *     [ ]                             [ ]  GuC sets new VF state to    |
 *     [ ]                             [ ]------- VF_RESFIX_BLOCKED     |
 *     [ ]                             [ ]      |                       |
 *     [ ]                             [ ] <-----                       |
 *     [ ]                             [ ]                              |
 *     [ ]                             [ ] GUC_INTR_SW_INT_0            |
 *     [ ] success                     [ ]---------------------------> [ ]
 *     [ ] <---------------------------[ ]                             [ ]
 *      |                               |      VF2GUC_QUERY_SINGLE_KLV [ ]
 *      |                              [ ] <---------------------------[ ]
 *      |                              [ ]                             [ ]
 *      |                              [ ]        new VF provisioning  [ ]
 *      |                              [ ]---------------------------> [ ]
 *      |                               |                              [ ]
 *      |                               |       VF driver applies post [ ]
 *      |                               |      migration fixups -------[ ]
 *      |                               |                       |      [ ]
 *      |                               |                       -----> [ ]
 *      |                               |                              [ ]
 *      |                               |    VF2GUC_NOTIFY_RESFIX_DONE [ ]
 *      |                              [ ] <---------------------------[ ]
 *      |                              [ ]                             [ ]
 *      |                              [ ]  GuC sets new VF state to   [ ]
 *      |                              [ ]------- VF_RUNNING           [ ]
 *      |                              [ ]      |                      [ ]
 *      |                              [ ] <-----                      [ ]
 *      |                              [ ]                     success [ ]
 *      |                              [ ]---------------------------> [ ]
 *      |                               |                               |
 *      |                               |                               |
 */

/**
 * xe_sriov_vf_migration_supported - Report whether SR-IOV VF migration is
 * supported or not.
 * @xe: the &xe_device to check
 *
 * Returns: true if VF migration is supported, false otherwise.
 */
bool xe_sriov_vf_migration_supported(struct xe_device *xe)
{
	xe_assert(xe, IS_SRIOV_VF(xe));
	return xe->sriov.vf.migration.enabled;
}

static void vf_disable_migration(struct xe_device *xe, const char *fmt, ...)
{
	struct va_format vaf;
	va_list va_args;

	xe_assert(xe, IS_SRIOV_VF(xe));

	va_start(va_args, fmt);
	vaf.fmt = fmt;
	vaf.va  = &va_args;
	xe_sriov_notice(xe, "migration disabled: %pV\n", &vaf);
	va_end(va_args);

	xe->sriov.vf.migration.enabled = false;
}

static void vf_migration_init_early(struct xe_device *xe)
{
	/*
	 * TODO: Add conditions to allow specific platforms, when they're
	 * supported at production quality.
	 */
	if (!IS_ENABLED(CONFIG_DRM_XE_DEBUG))
		return vf_disable_migration(xe,
					    "experimental feature not available on production builds");

	if (GRAPHICS_VER(xe) < 20)
		return vf_disable_migration(xe, "requires gfx version >= 20, but only %u found",
					    GRAPHICS_VER(xe));

	if (!IS_DGFX(xe)) {
		struct xe_uc_fw_version guc_version;

		xe_gt_sriov_vf_guc_versions(xe_device_get_gt(xe, 0), NULL, &guc_version);
		if (MAKE_GUC_VER_STRUCT(guc_version) < MAKE_GUC_VER(1, 23, 0))
			return vf_disable_migration(xe,
						    "CCS migration requires GuC ABI >= 1.23 but only %u.%u found",
						    guc_version.major, guc_version.minor);
	}

	xe->sriov.vf.migration.enabled = true;
	xe_sriov_dbg(xe, "migration support enabled\n");
}

/**
 * xe_sriov_vf_init_early - Initialize SR-IOV VF specific data.
 * @xe: the &xe_device to initialize
 */
void xe_sriov_vf_init_early(struct xe_device *xe)
{
	vf_migration_init_early(xe);
}

/**
 * xe_sriov_vf_init_late() - SR-IOV VF late initialization functions.
 * @xe: the &xe_device to initialize
 *
 * This function initializes code for CCS migration.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int xe_sriov_vf_init_late(struct xe_device *xe)
{
	int err = 0;

	if (xe_sriov_vf_migration_supported(xe))
		err = xe_sriov_vf_ccs_init(xe);

	return err;
}

static int sa_info_vf_ccs(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct xe_device *xe = to_xe_device(node->minor->dev);
	struct drm_printer p = drm_seq_file_printer(m);

	xe_sriov_vf_ccs_print(xe, &p);
	return 0;
}

static const struct drm_info_list debugfs_list[] = {
	{ .name = "sa_info_vf_ccs", .show = sa_info_vf_ccs },
};

/**
 * xe_sriov_vf_debugfs_register - Register VF debugfs attributes.
 * @xe: the &xe_device
 * @root: the root &dentry
 *
 * Prepare debugfs attributes exposed by the VF.
 */
void xe_sriov_vf_debugfs_register(struct xe_device *xe, struct dentry *root)
{
	drm_debugfs_create_files(debugfs_list, ARRAY_SIZE(debugfs_list),
				 root, xe->drm.primary);
}
