/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _AXI_HDMI_CRTC_H_
#define _AXI_HDMI_CRTC_H_

struct drm_device;
struct drm_crtc;

struct drm_crtc* axi_hdmi_crtc_create(struct drm_device *dev);

#endif
