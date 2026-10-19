// SPDX-License-Identifier: MIT
/*
 * Copyright © 2024 Intel Corporation
 */

#include <linux/shrinker.h>

#include <drm/drm_managed.h>
#include <drm/ttm/ttm_backup.h>
#include <drm/ttm/ttm_bo.h>
#include <drm/ttm/ttm_tt.h>

#include "xe_bo.h"
#include "xe_pm.h"
#include "xe_shrinker.h"

/*
 * Reclaiming non-purgeable objects requires GPU idling and writeback to a
 * backup store, and restoring them requires reading the contents back.
 * Tell the shrinker core they're costlier to recreate than purgeable
 * objects, which use DEFAULT_SEEKS, so that purgeable objects are
 * preferentially reclaimed. The default is
 * CONFIG_DRM_XE_SHRINKER_NON_PURGEABLE_SEEKS, overridable through sysfs.
 * Zero is disallowed, since the shrinker core treats it specially and
 * reads the value unlocked.
 */
#define XE_SHRINKER_NON_PURGEABLE_SEEKS_MIN 1
#define XE_SHRINKER_NON_PURGEABLE_SEEKS_MAX 64

/**
 * struct xe_shrinker - per-device shrinker
 * @xe: Back pointer to the device.
 * @lock: Lock protecting accounting.
 * @shrinkable_pages: Number of pages that are currently shrinkable.
 * @purgeable_pages: Number of pages that are currently purgeable.
 * @populated_tts: Number of populated ttm_tts currently shrinkable or purgeable.
 * @batch_avg: Exponential moving average of the preferred non-purgeable
 * batch size, based on the average populated ttm_tt size.
 * @shrink: Pointer to the mm shrinker handling non-purgeable (shrinkable) objects.
 * @purge_shrink: Pointer to the mm shrinker handling purgeable objects.
 * @pm_worker: Worker to wake up the device if required.
 */
struct xe_shrinker {
	struct xe_device *xe;
	rwlock_t lock;
	long shrinkable_pages;
	long purgeable_pages;
	long populated_tts;
	long batch_avg;
	struct shrinker *shrink;
	struct shrinker *purge_shrink;
	struct work_struct pm_worker;
};

static struct xe_shrinker *to_xe_shrinker(struct shrinker *shrink)
{
	return shrink->private_data;
}

/**
 * xe_shrinker_mod_pages() - Modify shrinker page and object accounting
 * @shrinker: Pointer to the struct xe_shrinker.
 * @shrinkable: Shrinkable pages delta. May be negative.
 * @purgeable: Purgeable pages delta. May be negative.
 * @tts: Populated ttm_tt count delta. May be negative.
 *
 * Updates the shrinkable and purgeable page counts and the populated
 * ttm_tt count.
 */
void
xe_shrinker_mod_pages(struct xe_shrinker *shrinker, long shrinkable, long purgeable,
		      long tts)
{
	write_lock(&shrinker->lock);
	shrinker->shrinkable_pages += shrinkable;
	shrinker->purgeable_pages += purgeable;
	shrinker->populated_tts += tts;
	write_unlock(&shrinker->lock);
}

static bool __xe_shrinker_runtime_pm_get(struct xe_shrinker *shrinker)
{
	struct xe_device *xe = shrinker->xe;

	if (xe_pm_runtime_get_if_active(xe))
		return true;

	if (xe_rpm_reclaim_safe(xe) && !ttm_bo_shrink_avoid_wait()) {
		xe_pm_runtime_get(xe);
		return true;
	}

	queue_work(xe->unordered_wq, &shrinker->pm_worker);

	return false;
}

static void xe_shrinker_runtime_pm_put(struct xe_shrinker *shrinker, bool runtime_pm)
{
	if (runtime_pm)
		xe_pm_runtime_put(shrinker->xe);
}

static int __xe_shrinker_walk(struct xe_shrinker *shrinker,
			      struct ttm_operation_ctx *ctx,
			      const struct xe_bo_shrink_flags flags,
			      unsigned long to_scan, unsigned long *scanned,
			      unsigned long *freed)
{
	struct xe_device *xe = shrinker->xe;
	unsigned int mem_type;
	bool rpm = false;
	int ret = 0;
	s64 lret;

	for (mem_type = XE_PL_SYSTEM; mem_type <= XE_PL_TT; ++mem_type) {
		struct ttm_resource_manager *man = ttm_manager_type(&xe->ttm, mem_type);
		struct ttm_bo_lru_cursor curs;
		struct ttm_buffer_object *ttm_bo;
		struct ttm_lru_walk_arg arg = {
			.ctx = ctx,
			.trylock_only = true,
		};

		if (!man || !man->use_tt)
			continue;

		if (mem_type != XE_PL_SYSTEM && !rpm &&
		    xe_device_is_l2_flush_optimized(xe)) {
			if (!__xe_shrinker_runtime_pm_get(shrinker))
				break;
			rpm = true;
		}

		ttm_bo_lru_for_each_reserved_guarded(&curs, man, &arg, ttm_bo) {
			if (!ttm_bo_shrink_suitable(ttm_bo, ctx))
				continue;

			lret = xe_bo_shrink(ctx, ttm_bo, flags, scanned);
			/* Like the TTM LRU walk, skip bos we can't shrink now. */
			if (lret == -EBUSY)
				continue;
			if (lret < 0) {
				ret = lret;
				goto out;
			}

			*freed += lret;
			if (*scanned >= to_scan)
				goto out;
		}
		/* Trylocks should never error, just fail. */
		xe_assert(xe, !IS_ERR(ttm_bo));
	}

out:
	xe_shrinker_runtime_pm_put(shrinker, rpm);

	return ret;
}

/*
 * Try shrinking idle objects without writeback first, then if not sufficient,
 * try also non-idle objects and finally if that's not sufficient either,
 * add writeback. This avoids stalls and explicit writebacks with light or
 * moderate memory pressure.
 */
static int xe_shrinker_walk(struct xe_shrinker *shrinker,
			    struct ttm_operation_ctx *ctx,
			    const struct xe_bo_shrink_flags flags,
			    unsigned long to_scan, unsigned long *scanned,
			    unsigned long *freed)
{
	bool no_wait_gpu = true;
	struct xe_bo_shrink_flags save_flags = flags;
	int ret;

	swap(no_wait_gpu, ctx->no_wait_gpu);
	save_flags.writeback = false;
	ret = __xe_shrinker_walk(shrinker, ctx, save_flags, to_scan, scanned,
				 freed);
	swap(no_wait_gpu, ctx->no_wait_gpu);
	if (ret || *scanned >= to_scan)
		return ret;

	if (!ctx->no_wait_gpu) {
		ret = __xe_shrinker_walk(shrinker, ctx, save_flags, to_scan, scanned,
					 freed);
		if (ret || *scanned >= to_scan)
			return ret;
	}

	if (flags.writeback)
		ret = __xe_shrinker_walk(shrinker, ctx, flags, to_scan, scanned,
					 freed);

	return ret;
}

static unsigned long
xe_shrinker_count_purgeable(struct shrinker *shrink, struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);
	unsigned long num_pages;

	read_lock(&shrinker->lock);
	num_pages = shrinker->purgeable_pages;
	read_unlock(&shrinker->lock);

	return num_pages ? num_pages : SHRINK_EMPTY;
}

/**
 * DOC: Batch size calculation
 *
 * Update our preferred vmscan batch size for the non-purgeable shrinker
 * on count passes that allow backup (__GFP_FS), to avoid invoking
 * scan_objects() for requests too small to free even a single
 * average-sized GEM object. Our rough estimate for an effective batch
 * is twice the average number of pages per populated ttm_tt across
 * the full shrinkable and purgeable population. The factor of two
 * provides headroom so that most scan invocations can free at least one
 * GEM object despite variability in object sizes.
 *
 * The batch value is updated as an exponential moving average,
 * (old_batch + avg) / 2, to smooth out sudden changes in the object
 * population. It is floored at 128 pages, the kernel default
 * SHRINK_BATCH, to ensure the shrinker remains responsive when there
 * are very few objects.
 *
 * We only do this for the non-purgeable shrinker: purgeable objects are
 * reclaimed without writeback or GPU idling, so shrinking them is cheap
 * enough that the core default batch size is fine.
 */
static unsigned long
xe_shrinker_count(struct shrinker *shrink, struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);
	unsigned long num_pages;
	bool can_backup = !!(sc->gfp_mask & __GFP_FS);

	if (!can_backup)
		return SHRINK_EMPTY;

	num_pages = ttm_backup_bytes_avail() >> PAGE_SHIFT;
	/* Consistently use write_lock() for simplicity. */
	write_lock(&shrinker->lock);

	num_pages = min_t(unsigned long, num_pages, shrinker->shrinkable_pages);

	if (shrinker->populated_tts) {
		unsigned long avg = 2 * (shrinker->shrinkable_pages +
					 shrinker->purgeable_pages) /
				     shrinker->populated_tts;

		shrinker->batch_avg = max((shrinker->batch_avg + (long)avg) >> 1,
					  128L /* default SHRINK_BATCH */);
		WRITE_ONCE(shrink->batch, shrinker->batch_avg);
	}

	write_unlock(&shrinker->lock);

	return num_pages ? num_pages : SHRINK_EMPTY;
}

/*
 * Check if we need runtime pm, and if so try to grab a reference if
 * already active. If grabbing a reference fails, queue a worker that
 * does it for us outside of reclaim, but don't wait for it to complete.
 * If bo shrinking needs an rpm reference and we don't have it (yet),
 * that bo will be skipped anyway.
 */
static bool xe_shrinker_runtime_pm_get(struct xe_shrinker *shrinker)
{
	struct xe_device *xe = shrinker->xe;

	if (IS_DGFX(xe) || !xe_device_has_flat_ccs(xe) ||
	    !ttm_backup_bytes_avail())
		return false;

	return __xe_shrinker_runtime_pm_get(shrinker);
}

static unsigned long xe_shrinker_scan_purgeable(struct shrinker *shrink,
						struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);
	struct ttm_operation_ctx ctx = {
		.interruptible = false,
		.no_wait_gpu = ttm_bo_shrink_avoid_wait(),
	};
	unsigned long nr_to_scan = sc->nr_to_scan, nr_scanned = 0, freed = 0;
	struct xe_bo_shrink_flags shrink_flags = {
		.purge = true,
	};

	xe_shrinker_walk(shrinker, &ctx, shrink_flags, nr_to_scan, &nr_scanned, &freed);

	sc->nr_scanned = nr_scanned;

	return nr_scanned ? freed : SHRINK_STOP;
}

static unsigned long xe_shrinker_scan(struct shrinker *shrink, struct shrink_control *sc)
{
	struct xe_shrinker *shrinker = to_xe_shrinker(shrink);
	struct ttm_operation_ctx ctx = {
		.interruptible = false,
		.no_wait_gpu = ttm_bo_shrink_avoid_wait(),
	};
	unsigned long nr_to_scan = sc->nr_to_scan, nr_scanned = 0, freed = 0;
	struct xe_bo_shrink_flags shrink_flags = {
		.purge = false,
		/* Don't request writeback without __GFP_IO. */
		.writeback = !ctx.no_wait_gpu && (sc->gfp_mask & __GFP_IO),
	};
	bool runtime_pm;

	/* Might need runtime PM to access backed-up or CCS-compressed data. */
	runtime_pm = xe_shrinker_runtime_pm_get(shrinker);

	xe_shrinker_walk(shrinker, &ctx, shrink_flags, nr_to_scan, &nr_scanned, &freed);

	sc->nr_scanned = nr_scanned;
	xe_shrinker_runtime_pm_put(shrinker, runtime_pm);

	return nr_scanned ? freed : SHRINK_STOP;
}

/* Wake up the device for shrinking. */
static void xe_shrinker_pm(struct work_struct *work)
{
	struct xe_shrinker *shrinker =
		container_of(work, typeof(*shrinker), pm_worker);

	xe_pm_runtime_get(shrinker->xe);
	xe_pm_runtime_put(shrinker->xe);
}

static void xe_shrinker_fini(struct drm_device *drm, void *arg)
{
	struct xe_shrinker *shrinker = arg;

	xe_assert(shrinker->xe, !shrinker->shrinkable_pages);
	xe_assert(shrinker->xe, !shrinker->purgeable_pages);
	xe_assert(shrinker->xe, !shrinker->populated_tts);
	shrinker_free(shrinker->shrink);
	shrinker_free(shrinker->purge_shrink);
	flush_work(&shrinker->pm_worker);
	kfree(shrinker);
}

/**
 * xe_shrinker_non_purgeable_seeks() - Get the non-purgeable shrinker seeks
 * @shrinker: Pointer to the struct xe_shrinker.
 *
 * Return: The seeks value of the non-purgeable shrinker.
 */
int xe_shrinker_non_purgeable_seeks(struct xe_shrinker *shrinker)
{
	return READ_ONCE(shrinker->shrink->seeks);
}

/**
 * xe_shrinker_set_non_purgeable_seeks() - Set the non-purgeable shrinker seeks
 * @shrinker: Pointer to the struct xe_shrinker.
 * @seeks: The new seeks value.
 *
 * Set the cost of recreating non-purgeable objects relative to purgeable
 * objects, which use DEFAULT_SEEKS. Higher values reduce reclaim pressure
 * on non-purgeable objects.
 *
 * Return: %0 on success, -EINVAL if @seeks is out of range.
 */
int xe_shrinker_set_non_purgeable_seeks(struct xe_shrinker *shrinker,
					unsigned int seeks)
{
	if (seeks < XE_SHRINKER_NON_PURGEABLE_SEEKS_MIN ||
	    seeks > XE_SHRINKER_NON_PURGEABLE_SEEKS_MAX)
		return -EINVAL;

	WRITE_ONCE(shrinker->shrink->seeks, seeks);

	return 0;
}

/**
 * xe_shrinker_create() - Create an xe per-device shrinker
 * @xe: Pointer to the xe device.
 *
 * Return: %0 on success. Negative error code on failure.
 */
int xe_shrinker_create(struct xe_device *xe)
{
	struct xe_shrinker *shrinker = kzalloc_obj(*shrinker);

	if (!shrinker)
		return -ENOMEM;

	shrinker->purge_shrink = shrinker_alloc(0, "drm-xe_gem-purgeable:%s", xe->drm.unique);
	if (!shrinker->purge_shrink) {
		kfree(shrinker);
		return -ENOMEM;
	}

	shrinker->shrink = shrinker_alloc(0, "drm-xe_gem-shrinkable:%s", xe->drm.unique);
	if (!shrinker->shrink) {
		shrinker_free(shrinker->purge_shrink);
		kfree(shrinker);
		return -ENOMEM;
	}

	INIT_WORK(&shrinker->pm_worker, xe_shrinker_pm);
	shrinker->xe = xe;
	rwlock_init(&shrinker->lock);

	shrinker->purge_shrink->count_objects = xe_shrinker_count_purgeable;
	shrinker->purge_shrink->scan_objects = xe_shrinker_scan_purgeable;
	shrinker->purge_shrink->private_data = shrinker;
	shrinker_register(shrinker->purge_shrink);

	shrinker->shrink->count_objects = xe_shrinker_count;
	shrinker->shrink->scan_objects = xe_shrinker_scan;
	shrinker->shrink->private_data = shrinker;
	shrinker->shrink->seeks = CONFIG_DRM_XE_SHRINKER_NON_PURGEABLE_SEEKS;
	shrinker_register(shrinker->shrink);

	xe->mem.shrinker = shrinker;

	return drmm_add_action_or_reset(&xe->drm, xe_shrinker_fini, shrinker);
}
