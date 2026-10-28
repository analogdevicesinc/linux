// SPDX-License-Identifier: MIT
/*
 * Copyright © 2024 Intel Corporation
 */

#include <linux/bitmap.h>
#include <linux/mutex.h>

#include <drm/drm_managed.h>

#include "xe_assert.h"
#include "xe_gt_printk.h"
#include "xe_guc.h"
#include "xe_guc_id_mgr.h"
#include "xe_guc_types.h"

static struct xe_guc *idm_to_guc(struct xe_guc_id_mgr *idm)
{
	return container_of(idm, struct xe_guc, submission_state.idm);
}

static struct xe_gt *idm_to_gt(struct xe_guc_id_mgr *idm)
{
	return guc_to_gt(idm_to_guc(idm));
}

static struct xe_device *idm_to_xe(struct xe_guc_id_mgr *idm)
{
	return gt_to_xe(idm_to_gt(idm));
}

#define idm_assert(idm, cond)		xe_gt_assert(idm_to_gt(idm), cond)
#define idm_mutex(idm)			(&idm_to_guc(idm)->submission_state.lock)

static void idm_print_locked(struct xe_guc_id_mgr *idm, struct drm_printer *p, int indent);

static unsigned int idm_total(struct xe_guc_id_mgr *idm)
{
	if (idm->usable > GUC_ID_MAX - idm->shareable)
		return GUC_ID_MAX;

	return idm->usable + idm->shareable;
}

static unsigned int idm_shareable_start(struct xe_guc_id_mgr *idm)
{
	return idm_total(idm) - idm->shareable;
}

static unsigned int idm_used_usable(struct xe_guc_id_mgr *idm)
{
	lockdep_assert_held(idm_mutex(idm));

	return bitmap_weight(idm->bitmap, idm->usable);
}

static unsigned int idm_used_shareable(struct xe_guc_id_mgr *idm)
{
	lockdep_assert_held(idm_mutex(idm));

	return bitmap_weight(idm->bitmap, idm_total(idm)) -
	       bitmap_weight(idm->bitmap, idm_shareable_start(idm));
}

static void __fini_idm(struct drm_device *drm, void *arg)
{
	struct xe_guc_id_mgr *idm = arg;

	mutex_lock(idm_mutex(idm));

	if (IS_ENABLED(CONFIG_DRM_XE_DEBUG)) {
		unsigned int weight = bitmap_weight(idm->bitmap, idm_total(idm));

		if (weight) {
			struct drm_printer p = xe_gt_info_printer(idm_to_gt(idm));

			xe_gt_err(idm_to_gt(idm), "GUC ID manager unclean (%u/%u)\n",
				  weight, idm_total(idm));
			idm_print_locked(idm, &p, 1);
		}
	}

	bitmap_free(idm->bitmap);
	idm->bitmap = NULL;
	idm->usable = 0;
	idm->shareable = 0;

	mutex_unlock(idm_mutex(idm));
}

static int idm_init(struct xe_guc_id_mgr *idm, unsigned int usable, unsigned int shareable)
{
	bool overlap = usable > GUC_ID_MAX - shareable;
	unsigned int total;
	int ret;

	if (!usable)
		return -EINVAL;
	if (usable > GUC_ID_MAX || shareable > GUC_ID_MAX)
		return -ERANGE;

	idm_assert(idm, !idm->bitmap);
	idm_assert(idm, !shareable || IS_SRIOV_PF(idm_to_xe(idm)));
	total = overlap ? GUC_ID_MAX : usable + shareable;

	idm->bitmap = bitmap_zalloc(total, GFP_KERNEL);
	if (!idm->bitmap)
		return -ENOMEM;

	idm->usable = usable;
	idm->shareable = shareable;

	ret = drmm_add_action_or_reset(&idm_to_xe(idm)->drm, __fini_idm, idm);
	if (ret)
		return ret;

	xe_gt_dbg(idm_to_gt(idm), "using %u GuC ID%s, %u GuC ID%s shareable with VFs\n",
		  idm->usable, str_plural(idm->usable), idm->shareable, str_plural(idm->shareable));
	return 0;
}

/**
 * xe_guc_id_mgr_init() - Initialize GuC ID manager for native.
 * @idm: the &xe_guc_id_mgr to initialize
 *
 * This function initializes the GuC ID manager to manage the full range of
 * GuC context IDs (0..GUC_ID_MAX - 1) for use by the GuC submission code.
 * No GuC context IDs will be reserved for sharing with VFs.
 *
 * Can only be called when the device is not in SR-IOV mode.
 *
 * Return: 0 on success or a negative error code on failure.
 */
int xe_guc_id_mgr_init(struct xe_guc_id_mgr *idm)
{
	xe_gt_assert(idm_to_gt(idm), !IS_SRIOV(idm_to_xe(idm)));

	return idm_init(idm, GUC_ID_MAX, 0);
}

/**
 * xe_guc_id_mgr_init_small() - Initialize GuC ID manager for small count.
 * @idm: the &xe_guc_id_mgr to initialize
 * @count: the number of usable GuC context IDs
 *
 * This function initializes the GuC ID manager to manage a small number of
 * GuC context IDs (0..count-1) for use by the GuC submission code.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_init_small(struct xe_guc_id_mgr *idm, unsigned int count)
{
	return idm_init(idm, count, 0);
}

#ifdef CONFIG_PCI_IOV
/**
 * xe_guc_id_mgr_init_shared() - Initialize GuC ID manager for PF.
 * @idm: the &xe_guc_id_mgr to initialize
 *
 * This function initializes the GuC ID manager to manage the full range of
 * GuC context IDs (0..GUC_ID_MAX - 1) for use by the GuC submission code.
 * All GuC context IDs will be shareable with VFs.
 *
 * Can only be called when the device is in SR-IOV PF mode.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_init_shared(struct xe_guc_id_mgr *idm)
{
	xe_gt_assert(idm_to_gt(idm), IS_SRIOV_PF(idm_to_xe(idm)));

	return idm_init(idm, GUC_ID_MAX, GUC_ID_MAX);
}
#endif /* CONFIG_PCI_IOV */

/**
 * xe_guc_id_mgr_max_usable() - Get maximum number of usable GuC context IDs.
 * @idm: the &xe_guc_id_mgr to query
 *
 * Return: maximum number of GuC context IDs that can be used by @idm for GuC submission.
 */
unsigned int xe_guc_id_mgr_max_usable(struct xe_guc_id_mgr *idm)
{
	return idm->usable;
}

/**
 * xe_guc_id_mgr_max_shareable() - Get maximum number of shareable GuC context IDs.
 * @idm: the &xe_guc_id_mgr to query
 *
 * Return: maximum number of GuC context IDs that can be shared with VFs.
 */
unsigned int xe_guc_id_mgr_max_shareable(struct xe_guc_id_mgr *idm)
{
	return idm->shareable;
}

static int idm_reserve_chunk_usable_locked(struct xe_guc_id_mgr *idm, unsigned int count)
{
	int id;

	idm_assert(idm, count);
	lockdep_assert_held(idm_mutex(idm));

	if (!idm->usable)
		return -ENODATA;

	id = bitmap_find_next_zero_area(idm->bitmap, idm->usable, 0, count, 0);
	if (id >= idm->usable)
		return -ENOSPC;

	bitmap_set(idm->bitmap, id, count);

	return id;
}

static unsigned int idm_find_last_zero_area_in_range(unsigned long *bitmap,
						     unsigned int range_start,
						     unsigned int range_end,
						     unsigned int count)
{
	unsigned int found = range_end;
	unsigned int rs, re;

	for_each_clear_bitrange(rs, re, bitmap, range_end) {
		unsigned int start = max(rs, range_start);
		unsigned int end = min(re, range_end);
		unsigned int range;

		if (start >= end)
			continue;

		range = end - start;
		if (range < count)
			continue;

		found = start + (range - count);
	}

	return found;
}

static int idm_check_quota(unsigned int used, unsigned int count,
			   unsigned int spare, unsigned int limit)
{
	return used + count + spare > limit ? -EDQUOT : 0;
}

static int idm_reserve_chunk_shareable_locked(struct xe_guc_id_mgr *idm, unsigned int count,
					      unsigned int spare)
{
	unsigned int shareable_start = idm_shareable_start(idm);
	unsigned int total = idm_total(idm);
	int id;

	idm_assert(idm, count);
	lockdep_assert_held(idm_mutex(idm));

	if (!idm->shareable)
		return -ENODATA;

	if (shareable_start < idm->usable)
		id = idm_find_last_zero_area_in_range(idm->bitmap, shareable_start, total, count);
	else
		id = bitmap_find_next_zero_area(idm->bitmap, total, shareable_start, count, 0);

	if (id >= total)
		return -ENOSPC;

	/*
	 * Spare is meaningful only for IDs that are shared between usable and
	 * shareable pools. Only IDs taken from the overlapping part reduce PF
	 * usable capacity, so a chunk located entirely in the VF-exclusive part
	 * (or any chunk from non-overlapping pools) is never limited by the spare.
	 */
	if (spare && id < idm->usable) {
		unsigned int need_overlap = min(id + count, idm->usable) - id;
		int err;

		err = idm_check_quota(idm_used_usable(idm), need_overlap, spare, idm->usable);
		if (err)
			return err;
	}

	bitmap_set(idm->bitmap, id, count);

	return id;
}

static void idm_release_chunk_locked(struct xe_guc_id_mgr *idm,
				     unsigned int start, unsigned int count)
{
	idm_assert(idm, count);
	idm_assert(idm, start < idm_total(idm));
	idm_assert(idm, start + count - 1 < idm_total(idm));
	lockdep_assert_held(idm_mutex(idm));

	if (IS_ENABLED(CONFIG_DRM_XE_DEBUG)) {
		unsigned int n;

		for (n = 0; n < count; n++)
			idm_assert(idm, test_bit(start + n, idm->bitmap));
	}
	bitmap_clear(idm->bitmap, start, count);
}

static int idm_release_chunk_in_range_locked(struct xe_guc_id_mgr *idm,
					     unsigned int start,
					     unsigned int count,
					     unsigned int range_start,
					     unsigned int range_end)
{
	lockdep_assert_held(idm_mutex(idm));

	if (!count)
		return -EINVAL;

	if (start < range_start || start >= range_end)
		return -ERANGE;

	if (count > range_end - start)
		return -ERANGE;

	idm_release_chunk_locked(idm, start, count);

	return 0;
}

/**
 * xe_guc_id_mgr_reserve_usable_locked() - Reserve one or more usable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @count: number of usable GuC context IDs to reserve (can't be 0)
 *
 * This function is dedicated for use by the GuC submission code when the
 * submission lock is already held.
 *
 * Return: ID of allocated GuC context or a negative error code on failure.
 */
int xe_guc_id_mgr_reserve_usable_locked(struct xe_guc_id_mgr *idm, unsigned int count)
{
	return idm_reserve_chunk_usable_locked(idm, count);
}

/**
 * xe_guc_id_mgr_reserve_shareable_locked() - Reserve one or more shareable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @count: number of shareable GuC context IDs to reserve (can't be 0)
 * @spare: number of usable IDs to keep available for the PF
 *
 * This function is dedicated for use by the PF driver, which expects the
 * reserved range of IDs to be contiguous. The function expects that the
 * caller has already taken the lock.
 *
 * Return: ID of allocated GuC context or a negative error code on failure.
 */
int xe_guc_id_mgr_reserve_shareable_locked(struct xe_guc_id_mgr *idm, unsigned int count,
					   unsigned int spare)
{
	return idm_reserve_chunk_shareable_locked(idm, count, spare);
}

/**
 * xe_guc_id_mgr_release_locked() - Release one or more GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @id: the GuC context ID to release
 * @count: number of IDs to release (can't be 0)
 *
 * This function is dedicated for the use by the GuC submission code,
 * where submission lock is already taken.
 */
void xe_guc_id_mgr_release_locked(struct xe_guc_id_mgr *idm, unsigned int id,
				  unsigned int count)
{
	return idm_release_chunk_locked(idm, id, count);
}

/**
 * xe_guc_id_mgr_release_usable_locked() - Release one or more usable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @id: the first usable GuC context ID to release
 * @count: number of usable GuC context IDs to release
 *
 * This function is dedicated for use by the GuC submission code when the
 * submission lock is already held.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_release_usable_locked(struct xe_guc_id_mgr *idm, unsigned int id,
					unsigned int count)
{
	return idm_release_chunk_in_range_locked(idm, id, count, 0, idm->usable);
}

/**
 * xe_guc_id_mgr_release_shareable_locked() - Release one or more shareable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @id: the first shareable GuC context ID to release
 * @count: number of shareable GuC context IDs to release
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_release_shareable_locked(struct xe_guc_id_mgr *idm, unsigned int id,
					   unsigned int count)
{
	return idm_release_chunk_in_range_locked(idm, id, count,
						 idm_shareable_start(idm), idm_total(idm));
}

/**
 * xe_guc_id_mgr_reserve_usable() - Reserve one or more usable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @count: number of usable GuC context IDs to reserve (can't be 0)
 *
 * Return: ID of allocated GuC context or a negative error code on failure.
 */
int xe_guc_id_mgr_reserve_usable(struct xe_guc_id_mgr *idm, unsigned int count)
{
	guard(mutex)(idm_mutex(idm));

	idm_assert(idm, count);

	return idm_reserve_chunk_usable_locked(idm, count);
}

/**
 * xe_guc_id_mgr_reserve_shareable() - Reserve one or more shareable GuC context IDs for VFs.
 * @idm: the &xe_guc_id_mgr
 * @count: number of shareable GuC context IDs to reserve (can't be 0)
 * @spare: number of usable IDs to keep available for the PF
 *
 * This function is dedicated for use by the PF driver, which expects the
 * reserved range of IDs to be contiguous.
 *
 * Return: starting ID of the allocated GuC context ID range or
 *         a negative error code on failure.
 */
int xe_guc_id_mgr_reserve_shareable(struct xe_guc_id_mgr *idm, unsigned int count,
				    unsigned int spare)
{
	guard(mutex)(idm_mutex(idm));

	idm_assert(idm, count);

	return idm_reserve_chunk_shareable_locked(idm, count, spare);
}

/**
 * xe_guc_id_mgr_release() - Release one or more GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @start: the first GuC context ID to release
 * @count: number of GuC context IDs to release
 */
void xe_guc_id_mgr_release(struct xe_guc_id_mgr *idm,
			   unsigned int start, unsigned int count)
{
	guard(mutex)(idm_mutex(idm));

	idm_release_chunk_locked(idm, start, count);
}

/**
 * xe_guc_id_mgr_release_usable() - Release one or more usable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @id: the first usable GuC context ID to release
 * @count: number of usable GuC context IDs to release
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_release_usable(struct xe_guc_id_mgr *idm, unsigned int id, unsigned int count)
{
	guard(mutex)(idm_mutex(idm));

	return xe_guc_id_mgr_release_usable_locked(idm, id, count);
}

/**
 * xe_guc_id_mgr_release_shareable() - Release one or more shareable GuC context IDs.
 * @idm: the &xe_guc_id_mgr
 * @id: the first shareable GuC context ID to release
 * @count: number of shareable GuC context IDs to release
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int xe_guc_id_mgr_release_shareable(struct xe_guc_id_mgr *idm, unsigned int id,
				    unsigned int count)
{
	guard(mutex)(idm_mutex(idm));

	return xe_guc_id_mgr_release_shareable_locked(idm, id, count);
}

static void idm_print_range_locked(struct xe_guc_id_mgr *idm,
				   struct drm_printer *p,
				   int indent,
				   const char *name,
				   unsigned int range_start,
				   unsigned int range_end,
				   unsigned int used)
{
	unsigned int rs, re;

	lockdep_assert_held(idm_mutex(idm));

	drm_printf_indent(p, indent, "%s used %u\n", name, used);
	for_each_set_bitrange(rs, re, idm->bitmap, range_end) {
		rs = max(rs, range_start);
		re = min(re, range_end);

		if (rs < re)
			drm_printf_indent(p, indent, "%s range %u..%u (%u)\n",
					  name, rs, re - 1, re - rs);
	}
}

static void idm_print_locked(struct xe_guc_id_mgr *idm, struct drm_printer *p, int indent)
{
	lockdep_assert_held(idm_mutex(idm));

	drm_printf_indent(p, indent, "total %u\n", idm_total(idm));
	if (!idm->bitmap)
		return;

	idm_print_range_locked(idm, p, indent, "usable", 0, idm->usable,
			       idm_used_usable(idm));
	idm_print_range_locked(idm, p, indent, "shareable",
			       idm_shareable_start(idm), idm_total(idm),
			       idm_used_shareable(idm));
}

/**
 * xe_guc_id_mgr_print() - Print status of GuC ID Manager.
 * @idm: the &xe_guc_id_mgr to print
 * @p: the &drm_printer to print to
 * @indent: tab indentation level
 */
void xe_guc_id_mgr_print(struct xe_guc_id_mgr *idm, struct drm_printer *p, int indent)
{
	guard(mutex)(idm_mutex(idm));
	idm_print_locked(idm, p, indent);
}

#if IS_BUILTIN(CONFIG_DRM_XE_KUNIT_TEST)
#include "tests/xe_guc_id_mgr_test.c"
#endif
