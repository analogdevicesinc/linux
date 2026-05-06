// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

/**
 * DOC: display pinning helpers
 */

#include <drm/drm_print.h>

#include "display/intel_display_core.h"
#include "display/intel_display_types.h"
#include "display/intel_fb.h"
#include "display/intel_fb_pin.h"
#include "display/intel_plane.h"

#include "gem/i915_gem_domain.h"
#include "gem/i915_gem_object.h"

#include "i915_dpt.h"
#include "i915_drv.h"
#include "i915_vma.h"

static struct i915_vma *
intel_fb_pin_to_dpt(struct drm_gem_object *_obj, struct intel_dpt *dpt,
		    const struct intel_fb_pin_params *pin_params)
{
	struct drm_i915_private *i915 = to_i915(_obj->dev);
	struct drm_i915_gem_object *obj = to_intel_bo(_obj);
	struct i915_address_space *vm = i915_dpt_to_vm(dpt);
	struct i915_gem_ww_ctx ww;
	struct i915_vma *vma;
	int ret;

	/*
	 * We are not syncing against the binding (and potential migrations)
	 * below, so this vm must never be async.
	 */
	if (drm_WARN_ON(&i915->drm, vm->bind_async_flags))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(!i915_gem_object_is_framebuffer(obj)))
		return ERR_PTR(-EINVAL);

	atomic_inc(&i915->pending_fb_pin);

	for_i915_gem_ww(&ww, ret, true) {
		ret = i915_gem_object_lock(obj, &ww);
		if (ret)
			continue;

		if (HAS_LMEM(i915)) {
			unsigned int flags = obj->flags;

			/*
			 * For this type of buffer we need to able to read from the CPU
			 * the clear color value found in the buffer, hence we need to
			 * ensure it is always in the mappable part of lmem, if this is
			 * a small-bar device.
			 */
			if (pin_params->needs_cpu_lmem_access)
				flags &= ~I915_BO_ALLOC_GPU_ONLY;
			ret = __i915_gem_object_migrate(obj, &ww, INTEL_REGION_LMEM_0,
							flags);
			if (ret)
				continue;
		}

		ret = i915_gem_object_set_cache_level(obj, I915_CACHE_NONE);
		if (ret)
			continue;

		vma = i915_vma_instance(obj, vm, pin_params->view);
		if (IS_ERR(vma)) {
			ret = PTR_ERR(vma);
			continue;
		}

		if (i915_vma_misplaced(vma, 0, pin_params->alignment, 0)) {
			ret = i915_vma_unbind(vma);
			if (ret)
				continue;
		}

		ret = i915_vma_pin_ww(vma, &ww, 0, pin_params->alignment,
				      PIN_GLOBAL);
		if (ret)
			continue;
	}
	if (ret) {
		vma = ERR_PTR(ret);
		goto err;
	}

	vma->display_alignment = max(vma->display_alignment,
				     pin_params->alignment);

	i915_gem_object_flush_if_display(obj);

	i915_vma_get(vma);
err:
	atomic_dec(&i915->pending_fb_pin);

	return vma;
}

struct i915_vma *
intel_fb_pin_to_ggtt(struct drm_gem_object *_obj,
		     const struct intel_fb_pin_params *pin_params,
		     int *out_fence_id)
{
	struct drm_i915_private *i915 = to_i915(_obj->dev);
	struct drm_i915_gem_object *obj = to_intel_bo(_obj);
	intel_wakeref_t wakeref;
	struct i915_gem_ww_ctx ww;
	struct i915_vma *vma;
	unsigned int pinctl;
	int ret;

	if (drm_WARN_ON(&i915->drm, !i915_gem_object_is_framebuffer(obj)))
		return ERR_PTR(-EINVAL);

	if (drm_WARN_ON(&i915->drm, pin_params->alignment &&
			!is_power_of_2(pin_params->alignment)))
		return ERR_PTR(-EINVAL);

	/*
	 * Global gtt pte registers are special registers which actually forward
	 * writes to a chunk of system memory. Which means that there is no risk
	 * that the register values disappear as soon as we call
	 * intel_runtime_pm_put(), so it is correct to wrap only the
	 * pin/unpin/fence and not more.
	 */
	wakeref = intel_runtime_pm_get(&i915->runtime_pm);

	atomic_inc(&i915->pending_fb_pin);

	pinctl = 0;
	/* PIN_MAPPABLE limits the address to GMADR size */
	if (pin_params->needs_low_address)
		pinctl |= PIN_MAPPABLE;

	i915_gem_ww_ctx_init(&ww, true);
retry:
	ret = i915_gem_object_lock(obj, &ww);
	if (!ret && pin_params->needs_physical)
		ret = i915_gem_object_attach_phys(obj, pin_params->phys_alignment);
	else if (!ret && HAS_LMEM(i915))
		ret = i915_gem_object_migrate(obj, &ww, INTEL_REGION_LMEM_0);
	if (!ret)
		ret = i915_gem_object_pin_pages(obj);
	if (ret)
		goto err;

	vma = i915_gem_object_pin_to_display_plane(obj, &ww,
						   pin_params->alignment,
						   pin_params->vtd_guard,
						   pin_params->view, pinctl);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto err_unpin;
	}

	if (out_fence_id)
		*out_fence_id = -1;

	if (out_fence_id && i915_vma_is_map_and_fenceable(vma)) {
		/*
		 * Install a fence for tiled scan-out. Pre-i965 always needs a
		 * fence, whereas 965+ only requires a fence if using
		 * framebuffer compression.  For simplicity, we always, when
		 * possible, install a fence as the cost is not that onerous.
		 *
		 * If we fail to fence the tiled scanout, then either the
		 * modeset will reject the change (which is highly unlikely as
		 * the affected systems, all but one, do not have unmappable
		 * space) or we will not be able to enable full powersaving
		 * techniques (also likely not to apply due to various limits
		 * FBC and the like impose on the size of the buffer, which
		 * presumably we violated anyway with this unmappable buffer).
		 * Anyway, it is presumably better to stumble onwards with
		 * something and try to run the system in a "less than optimal"
		 * mode that matches the user configuration.
		 */
		ret = i915_vma_pin_fence(vma);
		if (ret != 0 && pin_params->needs_fence) {
			i915_vma_unpin(vma);
			goto err_unpin;
		}
		ret = 0;

		if (vma->fence)
			*out_fence_id = vma->fence->id;
	}

	i915_vma_get(vma);

err_unpin:
	i915_gem_object_unpin_pages(obj);
err:
	if (ret == -EDEADLK) {
		ret = i915_gem_ww_ctx_backoff(&ww);
		if (!ret)
			goto retry;
	}
	i915_gem_ww_ctx_fini(&ww);
	if (ret)
		vma = ERR_PTR(ret);

	atomic_dec(&i915->pending_fb_pin);
	intel_runtime_pm_put(&i915->runtime_pm, wakeref);
	return vma;
}

void intel_fb_unpin_vma(struct i915_vma *vma, int fence_id)
{
	if (fence_id >= 0)
		i915_vma_unpin_fence(vma);
	i915_vma_unpin(vma);
	i915_vma_put(vma);
}

static unsigned int
intel_plane_fb_min_alignment(const struct intel_plane_state *plane_state)
{
	const struct intel_framebuffer *fb = to_intel_framebuffer(plane_state->hw.fb);

	return fb->min_alignment;
}

static unsigned int
intel_plane_fb_min_phys_alignment(const struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	const struct drm_framebuffer *fb = plane_state->hw.fb;

	if (!intel_plane_needs_physical(plane))
		return 0;

	return plane->min_alignment(plane, fb, 0);
}

static unsigned int
intel_plane_fb_vtd_guard(const struct intel_plane_state *plane_state)
{
	return intel_fb_view_vtd_guard(plane_state->hw.fb,
				       &plane_state->view,
				       plane_state->hw.rotation);
}

int intel_plane_pin_fb(struct intel_plane_state *plane_state,
		       const struct intel_plane_state *old_plane_state)
{
	struct intel_display *display = to_intel_display(plane_state);
	struct drm_i915_private *i915 = to_i915(plane_state->uapi.plane->dev);
	struct intel_plane *plane = to_intel_plane(plane_state->uapi.plane);
	const struct intel_framebuffer *fb =
		to_intel_framebuffer(plane_state->hw.fb);
	struct i915_vma *vma;

	if (!intel_fb_uses_dpt(&fb->base)) {
		struct intel_fb_pin_params pin_params = {
			.view = &plane_state->view.gtt,
			.alignment = intel_plane_fb_min_alignment(plane_state),
			.phys_alignment = intel_plane_fb_min_phys_alignment(plane_state),
			.vtd_guard = intel_plane_fb_vtd_guard(plane_state),
			.needs_cpu_lmem_access = intel_fb_needs_cpu_access(&fb->base),
			.needs_low_address = intel_plane_needs_low_address(display),
			.needs_physical = intel_plane_needs_physical(plane),
			.needs_fence = intel_plane_needs_fence(display),
		};
		int fence_id = -1;

		vma = intel_fb_pin_to_ggtt(intel_fb_bo(&fb->base), &pin_params,
					   intel_plane_uses_fence(plane_state) ? &fence_id : NULL);
		if (IS_ERR(vma))
			return PTR_ERR(vma);

		plane_state->ggtt_vma = vma;
		plane_state->fence_id = fence_id;
	} else {
		struct intel_fb_pin_params pin_params = {
			.view = &plane_state->view.gtt,
			.alignment = intel_plane_fb_min_alignment(plane_state),
			.needs_cpu_lmem_access = intel_fb_needs_cpu_access(&fb->base),
		};

		vma = i915_dpt_pin_to_ggtt(fb->dpt, pin_params.alignment / 512);
		if (IS_ERR(vma))
			return PTR_ERR(vma);

		plane_state->ggtt_vma = vma;

		vma = intel_fb_pin_to_dpt(intel_fb_bo(&fb->base), fb->dpt, &pin_params);
		if (IS_ERR(vma)) {
			i915_dpt_unpin_from_ggtt(fb->dpt);
			plane_state->ggtt_vma = NULL;
			return PTR_ERR(vma);
		}

		plane_state->dpt_vma = vma;

		WARN_ON(plane_state->ggtt_vma == plane_state->dpt_vma);

		/*
		 * The DPT object contains only one vma, and there is no VT-d
		 * guard, so the VMA's offset within the DPT is always 0.
		 */
		drm_WARN_ON(&i915->drm, i915_dpt_offset(plane_state->dpt_vma));
	}

	/*
	 * Pre-populate the dma address before we enter the vblank
	 * evade critical section as i915_gem_object_get_dma_address()
	 * will trigger might_sleep() even if it won't actually sleep,
	 * which is the case when the fb has already been pinned.
	 */
	if (intel_plane_needs_physical(plane)) {
		struct drm_i915_gem_object *obj = to_intel_bo(intel_fb_bo(&fb->base));

		plane_state->surf = i915_gem_object_get_dma_address(obj, 0) +
			plane->surf_offset(plane_state);
	} else {
		plane_state->surf = i915_ggtt_offset(plane_state->ggtt_vma) +
			plane->surf_offset(plane_state);
	}

	return 0;
}

void intel_plane_unpin_fb(struct intel_plane_state *old_plane_state)
{
	const struct intel_framebuffer *fb =
		to_intel_framebuffer(old_plane_state->hw.fb);
	struct i915_vma *vma;

	if (!intel_fb_uses_dpt(&fb->base)) {
		vma = fetch_and_zero(&old_plane_state->ggtt_vma);
		if (vma) {
			intel_fb_unpin_vma(vma, old_plane_state->fence_id);
			old_plane_state->fence_id = -1;
		}
	} else {
		vma = fetch_and_zero(&old_plane_state->dpt_vma);
		if (vma)
			intel_fb_unpin_vma(vma, -1);

		vma = fetch_and_zero(&old_plane_state->ggtt_vma);
		if (vma)
			i915_dpt_unpin_from_ggtt(fb->dpt);
	}
}

void intel_fb_get_map(struct i915_vma *vma, struct iosys_map *map)
{
	iosys_map_set_vaddr_iomem(map, i915_vma_get_iomap(vma));
}
