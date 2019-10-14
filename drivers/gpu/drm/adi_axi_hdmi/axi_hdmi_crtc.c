/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma/xilinx_dma.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "axi_hdmi_drv.h"

struct axi_hdmi_crtc {
	struct drm_crtc drm_crtc;
	struct drm_plane plane;

	struct dma_chan *dma;
	struct dma_interleaved_template *dma_template;
};

static struct axi_hdmi_crtc *plane_to_axi_hdmi_crtc(struct drm_plane *plane)
{
	return container_of(plane, struct axi_hdmi_crtc, plane);
}

static struct axi_hdmi_crtc *to_axi_hdmi_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_hdmi_crtc, drm_crtc);
}

static struct dma_async_tx_descriptor *axi_hdmi_vdma_prep_interleaved_desc(
	struct drm_plane *plane)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = plane_to_axi_hdmi_crtc(plane);
	struct drm_framebuffer *fb = plane->state->fb;
	size_t offset, hw_row_size;
	struct drm_gem_cma_object *obj;

#if IS_ENABLED(CONFIG_XILINX_DMA)
	struct xilinx_vdma_config vdma_config;

	if (!strncmp(axi_hdmi_crtc->dma->device->dev->driver->name, "xilinx-vdma", 11)) {
		memset(&vdma_config, 0, sizeof(vdma_config));
		vdma_config.park = 1;
		vdma_config.coalesc = 0xff;
		xilinx_vdma_channel_set_config(axi_hdmi_crtc->dma, &vdma_config);
	}
#endif

	obj = drm_fb_cma_get_gem_obj(plane->state->fb, 0);

	offset = plane->state->crtc_x * fb->format->cpp[0] +
		plane->state->crtc_y * fb->pitches[0];

	/* Interleaved DMA is used that way:
	 * Each interleaved frame is a row (hsize) implemented in ONE
	 * chunk (sgl has len 1).
	 * The number of interleaved frames is the number of rows (vsize).
	 * The icg in used to pack data to the HW, so that the buffer len
	 * is fb->piches[0], but the actual size for the hw is somewhat less
	 */
	axi_hdmi_crtc->dma_template->dir = DMA_MEM_TO_DEV;
	axi_hdmi_crtc->dma_template->src_start = obj->paddr + offset;
	/* sgl list have just one entry (each interleaved frame have 1 chunk) */
	axi_hdmi_crtc->dma_template->frame_size = 1;
	/* the number of interleaved frame, each has the size specified in sgl */
	axi_hdmi_crtc->dma_template->numf = plane->state->crtc_h;
	axi_hdmi_crtc->dma_template->src_sgl = 1;
	axi_hdmi_crtc->dma_template->src_inc = 1;

	/* vdma IP does not provide any addr to the hdmi IP, so dst_inc
	 * and dst_sgl should make no any difference.
	 */
	axi_hdmi_crtc->dma_template->dst_inc = 0;
	axi_hdmi_crtc->dma_template->dst_sgl = 0;

	hw_row_size = plane->state->crtc_w * fb->format->cpp[0];
	axi_hdmi_crtc->dma_template->sgl[0].size = hw_row_size;

	/* the vdma driver seems to look at icg, and not src_icg */
	axi_hdmi_crtc->dma_template->sgl[0].icg =
		fb->pitches[0] - hw_row_size;

	return dmaengine_prep_interleaved_dma(axi_hdmi_crtc->dma,
				axi_hdmi_crtc->dma_template, DMA_CYCLIC);
}

static void axi_hdmi_plane_atomic_update(struct drm_plane *plane,
	struct drm_plane_state *old_state)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = plane_to_axi_hdmi_crtc(plane);
	struct dma_async_tx_descriptor *desc;

	if (!plane->state->crtc || !plane->state->fb)
		return;

	dmaengine_terminate_all(axi_hdmi_crtc->dma);

	desc = axi_hdmi_vdma_prep_interleaved_desc(plane);
	if (!desc) {
		pr_err("Failed to prepare DMA descriptor\n");
		return;
	}

	dmaengine_submit(desc);
	dma_async_issue_pending(axi_hdmi_crtc->dma);
}

static void axi_hdmi_crtc_enable(struct drm_crtc *crtc,
				 struct drm_crtc_state *old_state)
{
}

static void axi_hdmi_crtc_disable(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_state)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	dmaengine_terminate_all(axi_hdmi_crtc->dma);
}

static void axi_hdmi_crtc_atomic_begin(struct drm_crtc *crtc,
	struct drm_crtc_state *state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_crtc_helper_funcs axi_hdmi_crtc_helper_funcs = {
	.atomic_enable = axi_hdmi_crtc_enable,
	.atomic_disable = axi_hdmi_crtc_disable,
	.atomic_begin = axi_hdmi_crtc_atomic_begin,
};

static void axi_hdmi_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(axi_hdmi_crtc->dma_template);
	kfree(axi_hdmi_crtc);
}

static const struct drm_crtc_funcs axi_hdmi_crtc_funcs = {
	.destroy = axi_hdmi_crtc_destroy,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_plane_helper_funcs axi_hdmi_plane_helper_funcs = {
	.atomic_update = axi_hdmi_plane_atomic_update,
};

static void axi_hdmi_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane, NULL);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs axi_hdmi_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = axi_hdmi_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const u32 axi_hdmi_supported_formats[] = {
	DRM_FORMAT_XRGB8888,
};

struct drm_crtc *axi_hdmi_crtc_create(struct drm_device *dev)
{
	struct axi_hdmi_private *p = dev->dev_private;
	struct axi_hdmi_crtc *axi_hdmi_crtc;
	struct drm_crtc *crtc;
	struct drm_plane *plane;
	int ret;

	if (!dma_has_cap(DMA_INTERLEAVE, p->dma->device->cap_mask)) {
		DRM_ERROR("DMA needs to support interleaved transfers\n");
		return ERR_PTR(-EINVAL);
	}

	axi_hdmi_crtc = kzalloc(sizeof(*axi_hdmi_crtc), GFP_KERNEL);
	if (!axi_hdmi_crtc)
		return ERR_PTR(-ENOMEM);

	crtc = &axi_hdmi_crtc->drm_crtc;
	plane = &axi_hdmi_crtc->plane;

	/* we know we'll always use only one data chunk */
	axi_hdmi_crtc->dma_template = kzalloc(
		sizeof(struct dma_interleaved_template) +
		sizeof(struct data_chunk), GFP_KERNEL);
	if (!axi_hdmi_crtc->dma_template) {
		ret = -ENOMEM;
		goto err_free_crtc;
	}

	ret = drm_universal_plane_init(dev, plane, 0xff, &axi_hdmi_plane_funcs,
		axi_hdmi_supported_formats,
		ARRAY_SIZE(axi_hdmi_supported_formats), NULL,
		DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		goto err_free_dma_template;

	drm_plane_helper_add(plane, &axi_hdmi_plane_helper_funcs);

	axi_hdmi_crtc->dma = p->dma;

	ret = drm_crtc_init_with_planes(dev, crtc, plane, NULL,
		&axi_hdmi_crtc_funcs, NULL);
	if (ret)
		goto err_plane_destroy;
	drm_crtc_helper_add(crtc, &axi_hdmi_crtc_helper_funcs);

	return crtc;

err_plane_destroy:
	axi_hdmi_plane_destroy(plane);
err_free_dma_template:
	kfree(axi_hdmi_crtc->dma_template);
err_free_crtc:
	kfree(axi_hdmi_crtc);

	return ERR_PTR(ret);
}
