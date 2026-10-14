/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2024 Intel Corporation
 */

#ifndef _XE_GUC_ID_MGR_H_
#define _XE_GUC_ID_MGR_H_

struct drm_printer;
struct xe_guc_id_mgr;

int xe_guc_id_mgr_init(struct xe_guc_id_mgr *idm, unsigned int limit);

unsigned int xe_guc_id_mgr_max_usable(struct xe_guc_id_mgr *idm);
unsigned int xe_guc_id_mgr_max_shareable(struct xe_guc_id_mgr *idm);

int xe_guc_id_mgr_reserve_usable_locked(struct xe_guc_id_mgr *idm, unsigned int count);
int xe_guc_id_mgr_reserve_usable(struct xe_guc_id_mgr *idm, unsigned int count);
int xe_guc_id_mgr_reserve_shareable_locked(struct xe_guc_id_mgr *idm, unsigned int count,
					   unsigned int spare);
int xe_guc_id_mgr_reserve_shareable(struct xe_guc_id_mgr *idm, unsigned int count,
				    unsigned int spare);
int xe_guc_id_mgr_reserve_locked(struct xe_guc_id_mgr *idm, unsigned int count);
int xe_guc_id_mgr_reserve(struct xe_guc_id_mgr *idm, unsigned int count, unsigned int retain);
void xe_guc_id_mgr_release_locked(struct xe_guc_id_mgr *idm, unsigned int id, unsigned int count);
int xe_guc_id_mgr_release_usable_locked(struct xe_guc_id_mgr *idm, unsigned int id,
					unsigned int count);
int xe_guc_id_mgr_release_shareable_locked(struct xe_guc_id_mgr *idm, unsigned int id,
					   unsigned int count);

void xe_guc_id_mgr_release(struct xe_guc_id_mgr *idm, unsigned int start, unsigned int count);
int xe_guc_id_mgr_release_usable(struct xe_guc_id_mgr *idm, unsigned int id, unsigned int count);
int xe_guc_id_mgr_release_shareable(struct xe_guc_id_mgr *idm, unsigned int id,
				    unsigned int count);

void xe_guc_id_mgr_print(struct xe_guc_id_mgr *idm, struct drm_printer *p, int indent);

#endif
