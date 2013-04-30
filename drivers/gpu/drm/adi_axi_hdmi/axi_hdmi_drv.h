/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _AXI_HDMI_DRV_H_
#define _AXI_HDMI_DRV_H_

#include <drm/drm.h>
#include <drm/drm_fb_cma_helper.h>
#include <linux/of.h>
#include <linux/clk.h>

struct xlnx_pcm_dma_params {
	struct device_node *of_node;
	int chan_id;
};

struct axi_hdmi_encoder;

struct axi_hdmi_private {
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc *crtc;
	struct axi_hdmi_encoder *encoder;
	struct i2c_client *encoder_slave;

	void __iomem *base;

	struct clk *hdmi_clock;

	struct xlnx_pcm_dma_params dma_params;
	bool is_rgb;
	bool embedded_sync;
};

#endif
