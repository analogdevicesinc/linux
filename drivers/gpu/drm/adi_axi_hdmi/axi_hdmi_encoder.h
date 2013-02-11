/*
 * Analog Devices AXI HDMI DRM driver.
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _AXI_HDMI_ENCODER_H_
#define _AXI_HDMI_ENCODER_H_

struct drm_encoder *axi_hdmi_encoder_create(struct drm_device *dev);

#endif
