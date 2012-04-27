#ifndef _ANALOG_DRM_DRV_H_
#define _ANALOG_DRM_DRV_H_

#include <drm/drm.h>
#include <linux/of.h>

struct xlnx_pcm_dma_params {
	struct device_node *of_node;
	int chan_id;
};

struct analog_drm_fbdev;
struct analog_drm_encoder;

struct analog_drm_private {
	struct analog_drm_fbdev *fbdev;
	struct drm_crtc *crtc;
	struct analog_drm_encoder *encoder;
	struct i2c_adapter *ddc_adapter;
	struct i2c_adapter *slave_adapter;

	void __iomem *base;
	void __iomem *base_clock;

	struct xlnx_pcm_dma_params dma_params;
};

#endif
