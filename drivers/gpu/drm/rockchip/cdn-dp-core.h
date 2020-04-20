/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2016 Chris Zhong <zyw@rock-chips.com>
 * Copyright (C) 2016 ROCKCHIP, Inc.
 */

#ifndef _CDN_DP_CORE_H
#define _CDN_DP_CORE_H

#include <drm/display/drm_dp_helper.h>
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <sound/hdmi-codec.h>

#include "rockchip_drm_drv.h"

#define MAX_PHY		2

struct cdn_firmware_header {
	u32 size_bytes; /* size of the entire header+image(s) in bytes */
	u32 header_size; /* size of just the header in bytes */
	u32 iram_size; /* size of iram */
	u32 dram_size; /* size of dram */
};

struct cdn_dp_port {
	struct cdn_dp_device *dp;
	struct notifier_block event_nb;
	struct extcon_dev *extcon;
	struct phy *phy;
	u8 lanes;
	bool phy_enabled;
	u8 id;
};

struct cdn_dp_device {
	struct cdns_mhdp_device mhdp;
	struct drm_device *drm_dev;
	struct rockchip_encoder encoder;
	struct work_struct event_work;
	const struct drm_edid *drm_edid;

	struct mutex lock;
	bool connected;
	bool active;
	bool suspended;

	const struct firmware *fw;	/* cdn dp firmware */
	bool fw_loaded;

	struct regmap *grf;
	struct clk *core_clk;
	struct clk *pclk;
	struct clk *grf_clk;
	struct reset_control *dptx_rst;
	struct reset_control *apb_rst;
	struct reset_control *core_rst;
	struct cdn_dp_port *port[MAX_PHY];
	u8 ports;
	u8 lanes;
	int active_port;

	bool sink_has_audio;

	hdmi_codec_plugged_cb plugged_cb;
	struct device *codec_dev;
};
#endif  /* _CDN_DP_CORE_H */
