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
#include <linux/amba/xilinx_dma.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "axi_hdmi_crtc.h"
#include "axi_hdmi_drv.h"
#include "axi_hdmi_encoder.h"

struct axi_hdmi_crtc {
	struct drm_crtc drm_crtc;
	struct dma_chan *dma;
	struct dma_interleaved_template *dma_template;
	int mode;
};

static inline struct axi_hdmi_crtc *to_axi_hdmi_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_hdmi_crtc, drm_crtc);
}

static struct dma_async_tx_descriptor
*axi_hdmi_vdma_prep_single_desc(struct drm_crtc *crtc,
				struct drm_gem_cma_object *obj)
{
	struct xilinx_dma_config dma_config;
	size_t offset;
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;

	memset(&dma_config, 0, sizeof(dma_config));
	dma_config.hsize = mode->hdisplay * fb->bits_per_pixel / 8;
	dma_config.vsize = mode->vdisplay;
	dma_config.stride = fb->pitches[0];

	dmaengine_slave_config(axi_hdmi_crtc->dma,
			(struct dma_slave_config *)&dma_config);

	offset = crtc->x * fb->bits_per_pixel / 8 + crtc->y * fb->pitches[0];

	return dmaengine_prep_slave_single(axi_hdmi_crtc->dma,
					obj->paddr + offset,
					mode->vdisplay * fb->pitches[0],
					DMA_MEM_TO_DEV, 0);
}

static struct dma_async_tx_descriptor
*axi_hdmi_vdma_prep_interleaved_desc(struct drm_crtc *crtc,
				struct drm_gem_cma_object *obj)
{
	struct xilinx_vdma_config vdma_config;
	size_t offset, hw_row_size;
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;

	memset(&vdma_config, 0, sizeof(vdma_config));
	vdma_config.park = 1;
	xilinx_vdma_channel_set_config(axi_hdmi_crtc->dma, &vdma_config);

	offset = crtc->x * fb->bits_per_pixel / 8 +
		crtc->y * fb->pitches[0];

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
	axi_hdmi_crtc->dma_template->numf = mode->vdisplay;
	axi_hdmi_crtc->dma_template->src_sgl = 1;
	axi_hdmi_crtc->dma_template->src_inc = 1;

	/* vdma IP does not provide any addr to the hdmi IP, so dst_inc
	 * and dst_sgl should make no any difference.
	 */
	axi_hdmi_crtc->dma_template->dst_inc = 0;
	axi_hdmi_crtc->dma_template->dst_sgl = 0;

	hw_row_size = mode->hdisplay * fb->bits_per_pixel / 8;
	axi_hdmi_crtc->dma_template->sgl[0].size = hw_row_size;

	/* the vdma driver seems to look at icg, and not src_icg */
	axi_hdmi_crtc->dma_template->sgl[0].icg =
		fb->pitches[0] - hw_row_size;

	return dmaengine_prep_interleaved_dma(axi_hdmi_crtc->dma,
						axi_hdmi_crtc->dma_template, 0);
}

static int axi_hdmi_crtc_update(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct dma_async_tx_descriptor *desc;
	struct drm_gem_cma_object *obj;

	if (!mode || !fb)
		return -EINVAL;

	dmaengine_terminate_all(axi_hdmi_crtc->dma);

	if (axi_hdmi_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;


		if (dma_has_cap(DMA_INTERLEAVE,
					axi_hdmi_crtc->dma->device->cap_mask)) {
			desc = axi_hdmi_vdma_prep_interleaved_desc(crtc, obj);
		} else {
			desc = axi_hdmi_vdma_prep_single_desc(crtc, obj);
		}

		if (!desc) {
			pr_err("Failed to prepare DMA descriptor\n");
			return -ENOMEM;
		} else {
			dmaengine_submit(desc);
			dma_async_issue_pending(axi_hdmi_crtc->dma);
		}
	}

	return 0;
}

static void axi_hdmi_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	if (axi_hdmi_crtc->mode != mode) {
		axi_hdmi_crtc->mode = mode;
		axi_hdmi_crtc_update(crtc);
	}
}

static void axi_hdmi_crtc_prepare(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	dmaengine_terminate_all(axi_hdmi_crtc->dma);
}

static void axi_hdmi_crtc_commit(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	axi_hdmi_crtc->mode = DRM_MODE_DPMS_ON;
	axi_hdmi_crtc_update(crtc);
}

static bool axi_hdmi_crtc_mode_fixup(struct drm_crtc *crtc,
	const struct drm_display_mode *mode,
	struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int axi_hdmi_crtc_mode_set(struct drm_crtc *crtc,
	struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode,
	int x, int y, struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int axi_hdmi_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
	struct drm_framebuffer *old_fb)
{
	return axi_hdmi_crtc_update(crtc);
}

static void axi_hdmi_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs axi_hdmi_crtc_helper_funcs = {
	.dpms		= axi_hdmi_crtc_dpms,
	.prepare	= axi_hdmi_crtc_prepare,
	.commit		= axi_hdmi_crtc_commit,
	.mode_fixup	= axi_hdmi_crtc_mode_fixup,
	.mode_set	= axi_hdmi_crtc_mode_set,
	.mode_set_base	= axi_hdmi_crtc_mode_set_base,
	.load_lut	= axi_hdmi_crtc_load_lut,
};

static void axi_hdmi_crtc_destroy(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(axi_hdmi_crtc->dma_template);
	kfree(axi_hdmi_crtc);
}

static struct drm_crtc_funcs axi_hdmi_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_hdmi_crtc_destroy,
};

struct drm_crtc *axi_hdmi_crtc_create(struct drm_device *dev)
{
	struct axi_hdmi_private *p = dev->dev_private;
	struct axi_hdmi_crtc *axi_hdmi_crtc;
	struct drm_crtc *crtc;

	axi_hdmi_crtc = kzalloc(sizeof(*axi_hdmi_crtc), GFP_KERNEL);
	if (!axi_hdmi_crtc) {
		DRM_ERROR("failed to allocate axi_hdmi crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	/* we know we'll always use only one data chunk */
	axi_hdmi_crtc->dma_template = kzalloc(
		sizeof(struct dma_interleaved_template) +
		sizeof(struct data_chunk), GFP_KERNEL);

	if (!axi_hdmi_crtc->dma_template) {
		kfree(axi_hdmi_crtc);
		DRM_ERROR("failed to allocate dma_template crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	crtc = &axi_hdmi_crtc->drm_crtc;

	axi_hdmi_crtc->dma = p->dma;

	drm_crtc_init(dev, crtc, &axi_hdmi_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_hdmi_crtc_helper_funcs);

	return crtc;
}
