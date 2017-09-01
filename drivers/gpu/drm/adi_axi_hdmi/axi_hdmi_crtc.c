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

#include "axi_hdmi_drv.h"

struct axi_hdmi_crtc {
	struct drm_crtc drm_crtc;
	struct dma_chan *dma;
	struct xilinx_dma_config dma_config;
	int mode;
};

static inline struct axi_hdmi_crtc *to_axi_hdmi_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct axi_hdmi_crtc, drm_crtc);
}

static int axi_hdmi_crtc_update(struct drm_crtc *crtc)
{
	struct axi_hdmi_crtc *axi_hdmi_crtc = to_axi_hdmi_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->fb;
	struct dma_async_tx_descriptor *desc;
	struct drm_gem_cma_object *obj;
	size_t offset;

	if (!mode || !fb)
		return -EINVAL;

	dmaengine_terminate_all(axi_hdmi_crtc->dma);

	if (axi_hdmi_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = drm_fb_cma_get_gem_obj(fb, 0);
		if (!obj)
			return -EINVAL;

		axi_hdmi_crtc->dma_config.hsize = mode->hdisplay * fb->bits_per_pixel / 8;
		axi_hdmi_crtc->dma_config.vsize = mode->vdisplay;
		axi_hdmi_crtc->dma_config.stride = fb->pitches[0];

		dmaengine_device_control(axi_hdmi_crtc->dma, DMA_SLAVE_CONFIG,
			(unsigned long)&axi_hdmi_crtc->dma_config);

		offset = crtc->x * fb->bits_per_pixel / 8 + crtc->y * fb->pitches[0];

		desc = dmaengine_prep_slave_single(axi_hdmi_crtc->dma,
					obj->paddr + offset,
					mode->vdisplay * fb->pitches[0],
					DMA_MEM_TO_DEV, 0);
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
	dma_release_channel(axi_hdmi_crtc->dma);
	kfree(axi_hdmi_crtc);
}

static struct drm_crtc_funcs axi_hdmi_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= axi_hdmi_crtc_destroy,
};

static bool xlnx_pcm_filter(struct dma_chan *chan, void *param)
{
	struct xlnx_pcm_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

struct drm_crtc *axi_hdmi_crtc_create(struct drm_device *dev)
{
	struct axi_hdmi_private *p = dev->dev_private;
	struct axi_hdmi_crtc *axi_hdmi_crtc;
	struct drm_crtc *crtc;

	dma_cap_mask_t mask;

	axi_hdmi_crtc = kzalloc(sizeof(*axi_hdmi_crtc), GFP_KERNEL);
	if (!axi_hdmi_crtc) {
		DRM_ERROR("failed to allocate axi_hdmi crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	crtc = &axi_hdmi_crtc->drm_crtc;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);

	axi_hdmi_crtc->dma = dma_request_channel(mask, xlnx_pcm_filter,
						&p->dma_params);
	if (!axi_hdmi_crtc->dma) {
		kfree(axi_hdmi_crtc);
		return ERR_PTR(-EINVAL);
	}

	drm_crtc_init(dev, crtc, &axi_hdmi_crtc_funcs);
	drm_crtc_helper_add(crtc, &axi_hdmi_crtc_helper_funcs);

	return crtc;
}
