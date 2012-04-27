#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <linux/dmaengine.h>
#include <linux/amba/xilinx_dma.h>

#include "analog_drm_crtc.h"
#include "analog_drm_fbdev.h"
#include "analog_drm_drv.h"
#include "analog_drm_encoder.h"
#include "analog_drm_gem.h"

#define to_analog_crtc(x)	container_of(x, struct analog_drm_crtc,\
				drm_crtc)

struct analog_drm_crtc {
	struct drm_crtc			drm_crtc;
	struct dma_chan 		*dma;
	struct xilinx_dma_config dma_config;
	int mode;
};

static int analog_drm_crtc_update(struct drm_crtc *crtc)
{
	struct analog_drm_crtc *analog_crtc = to_analog_crtc(crtc);
	struct drm_display_mode *mode = &crtc->mode;
	struct drm_framebuffer *fb = crtc->fb;
	struct dma_async_tx_descriptor *desc;
	struct analog_drm_gem_obj *obj;
	size_t offset;

	if (!mode || !fb)
		return -EINVAL;

	if (analog_crtc->mode == DRM_MODE_DPMS_ON) {
		obj = analog_drm_fb_get_gem_obj(fb);
		if (!obj)
			return -EINVAL;

		analog_crtc->dma_config.hsize = mode->hdisplay * fb->bits_per_pixel / 8;
		analog_crtc->dma_config.vsize = mode->vdisplay;
		analog_crtc->dma_config.stride = fb->pitches[0];

		dmaengine_device_control(analog_crtc->dma, DMA_SLAVE_CONFIG,
			(unsigned long)&analog_crtc->dma_config);

		offset = crtc->x * fb->bits_per_pixel / 8 + crtc->y * fb->pitches[0];

		desc = dmaengine_prep_slave_single(analog_crtc->dma,
					obj->dma_addr + offset, mode->vdisplay * fb->pitches[0],
					DMA_MEM_TO_DEV, 0);
		if (!desc) {
			pr_err("Failed to prepare DMA descriptor\n");
			return -ENOMEM;
		} else {
			dmaengine_submit(desc);
			dma_async_issue_pending(analog_crtc->dma);
		}
	} else {
		dmaengine_terminate_all(analog_crtc->dma);
	}

	return 0;
}

static void analog_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct analog_drm_crtc *analog_crtc = to_analog_crtc(crtc);

	if (analog_crtc->mode != mode) {
		analog_crtc->mode = mode;
		analog_drm_crtc_update(crtc);
	}
}

static void analog_drm_crtc_prepare(struct drm_crtc *crtc)
{
	struct analog_drm_crtc *analog_crtc = to_analog_crtc(crtc);

	dmaengine_terminate_all(analog_crtc->dma);
}

static void analog_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct analog_drm_crtc *analog_crtc = to_analog_crtc(crtc);

	analog_crtc->mode = DRM_MODE_DPMS_ON;
	analog_drm_crtc_update(crtc);
}

static bool
analog_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int
analog_drm_crtc_mode_set(struct drm_crtc *crtc, struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode, int x, int y,
			  struct drm_framebuffer *old_fb)
{
	/* We do everything in commit() */
	return 0;
}

static int analog_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					  struct drm_framebuffer *old_fb)
{
	return analog_drm_crtc_update(crtc);
}

static void analog_drm_crtc_load_lut(struct drm_crtc *crtc)
{
	/* drm framework doesn't check NULL */
}

static struct drm_crtc_helper_funcs analog_crtc_helper_funcs = {
	.dpms		= analog_drm_crtc_dpms,
	.prepare	= analog_drm_crtc_prepare,
	.commit		= analog_drm_crtc_commit,
	.mode_fixup	= analog_drm_crtc_mode_fixup,
	.mode_set	= analog_drm_crtc_mode_set,
	.mode_set_base	= analog_drm_crtc_mode_set_base,
	.load_lut	= analog_drm_crtc_load_lut,
};

static void analog_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct analog_drm_crtc *analog_crtc = to_analog_crtc(crtc);

	drm_crtc_cleanup(crtc);
	kfree(analog_crtc);
}

static struct drm_crtc_funcs analog_crtc_funcs = {
	.set_config	= drm_crtc_helper_set_config,
	.destroy	= analog_drm_crtc_destroy,
};

static bool xlnx_pcm_filter(struct dma_chan *chan, void *param)
{
	struct xlnx_pcm_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

struct drm_crtc *analog_drm_crtc_create(struct drm_device *dev)
{
	struct analog_drm_private *p = dev->dev_private;
	struct analog_drm_crtc *analog_crtc;
	struct drm_crtc *crtc;
	dma_cap_mask_t mask;

	analog_crtc = kzalloc(sizeof(*analog_crtc), GFP_KERNEL);
	if (!analog_crtc) {
		DRM_ERROR("failed to allocate analog crtc\n");
		return ERR_PTR(-ENOMEM);
	}

	crtc = &analog_crtc->drm_crtc;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);

	analog_crtc->dma = dma_request_channel(mask, xlnx_pcm_filter, &p->dma_params);

	drm_crtc_init(dev, crtc, &analog_crtc_funcs);
	drm_crtc_helper_add(crtc, &analog_crtc_helper_funcs);

	return crtc;
}
