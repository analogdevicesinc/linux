/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023-2024 Intel Corporation
 */

#ifndef _XE_GT_SRIOV_PF_CONFIG_H_
#define _XE_GT_SRIOV_PF_CONFIG_H_

#include <linux/types.h>

struct drm_printer;
struct xe_gt;

u64 xe_gt_sriov_pf_config_get_ggtt(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_ggtt(struct xe_gt *gt, unsigned int vfid, u64 size);
int xe_gt_sriov_pf_config_set_fair_ggtt(struct xe_gt *gt,
					unsigned int vfid, unsigned int num_vfs);
int xe_gt_sriov_pf_config_bulk_set_ggtt(struct xe_gt *gt,
					unsigned int vfid, unsigned int num_vfs, u64 size);

u32 xe_gt_sriov_pf_config_get_ctxs(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_ctxs(struct xe_gt *gt, unsigned int vfid, u32 num_ctxs);
int xe_gt_sriov_pf_config_set_fair_ctxs(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs);
int xe_gt_sriov_pf_config_bulk_set_ctxs(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs,
					u32 num_ctxs);

u32 xe_gt_sriov_pf_config_get_dbs(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_dbs(struct xe_gt *gt, unsigned int vfid, u32 num_dbs);
int xe_gt_sriov_pf_config_set_fair_dbs(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs);
int xe_gt_sriov_pf_config_bulk_set_dbs(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs,
				       u32 num_dbs);

u64 xe_gt_sriov_pf_config_get_lmem(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_lmem(struct xe_gt *gt, unsigned int vfid, u64 size);
int xe_gt_sriov_pf_config_set_fair_lmem(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs);
int xe_gt_sriov_pf_config_bulk_set_lmem(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs,
					u64 size);

u32 xe_gt_sriov_pf_config_get_exec_quantum(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_exec_quantum(struct xe_gt *gt, unsigned int vfid, u32 exec_quantum);

u32 xe_gt_sriov_pf_config_get_preempt_timeout(struct xe_gt *gt, unsigned int vfid);
int xe_gt_sriov_pf_config_set_preempt_timeout(struct xe_gt *gt, unsigned int vfid,
					      u32 preempt_timeout);

int xe_gt_sriov_pf_config_set_fair(struct xe_gt *gt, unsigned int vfid, unsigned int num_vfs);
int xe_gt_sriov_pf_config_release(struct xe_gt *gt, unsigned int vfid, bool force);
int xe_gt_sriov_pf_config_push(struct xe_gt *gt, unsigned int vfid, bool refresh);

int xe_gt_sriov_pf_config_print_ggtt(struct xe_gt *gt, struct drm_printer *p);
int xe_gt_sriov_pf_config_print_ctxs(struct xe_gt *gt, struct drm_printer *p);
int xe_gt_sriov_pf_config_print_dbs(struct xe_gt *gt, struct drm_printer *p);

int xe_gt_sriov_pf_config_print_available_ggtt(struct xe_gt *gt, struct drm_printer *p);

#endif
