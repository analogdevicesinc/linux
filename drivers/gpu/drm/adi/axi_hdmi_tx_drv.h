/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012-2020 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
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

struct axi_hdmi_tx_encoder;

struct axi_hdmi_tx_private {
	struct drm_device *drm_dev;
	struct drm_crtc *crtc;
	struct axi_hdmi_tx_encoder *encoder;
	struct i2c_client *encoder_slave;

	void __iomem *base;

	struct clk *hdmi_clock;
	bool clk_enabled;

	struct dma_chan *dma;

	bool is_rgb;
};

struct drm_crtc *axi_hdmi_tx_crtc_create(struct drm_device *dev);
struct drm_encoder *axi_hdmi_tx_encoder_create(struct drm_device *dev);

#endif
