/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MXC_HDMI_RX_
#define _MXC_HDMI_RX_

#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <soc/imx8/sc/sci.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>

#include <uapi/linux/v4l2-dv-timings.h>

#include "../../../../mxc/hdp/all.h"
#include "../../../../mxc/hdp-cec/imx-hdp-cec.h"

#define state_to_mxc_hdmirx(env) \
	container_of(env, struct mxc_hdmi_rx_dev, state)

#define MXC_HDMI_RX_DRIVER_NAME	"mxc-hdmi-rx"
#define MXC_HDMI_RX_SUBDEV_NAME	MXC_HDMI_RX_DRIVER_NAME

#define MXC_HDMI_RX_NODE_NAME	"hdmi_rx"

#define MXC_HDMI_RX_MAX_DEVS		2
#define MXC_HDMI_RX_MAX_LANES		4

#define MXC_HDMI_RX_PAD_SINK		1
#define MXC_HDMI_RX_PAD_SOURCE		2
#define MXC_HDMI_RX_PADS_NUM		3

#define CSR_PIXEL_LINK_ENC_CTL		0x00
#define PL_ENC_CTL_PXL_VAL          15
#define PL_ENC_CTL_PXL_VPP          14
#define PL_ENC_CTL_PXL_HPP          13
#define PL_ENC_CTL_PXL_VCP          12
#define PL_ENC_CTL_PXL_HCP          11
#define PL_ENC_CTL_PXL_ADD          9
#define PL_ENC_CTL_PXL_EXT          7
#define PL_ENC_CTL_PXL_EN           6
#define PL_ENC_CTL_PXL_ITC          4
#define PL_ENC_CTL_PXL_ODD_EVEN     3
#define PL_ENC_CTL_PXL_TYP          1
#define PL_ENC_CTL_PXL_YUV          0

#define CSR_HDP_RX_CTRL_CTRL0		0x04
#define CSR_HDP_RX_CTRL_CTRL1		0x08

struct mxc_hdmi_rx_dev_video_standards {
	struct v4l2_dv_timings timings;
	u8 vic;
	u8 hdmi_vic;
	u8 fps;
};

struct mxc_hdmi_rx_dev {
	spinlock_t				slock;
	struct mutex			lock;
	wait_queue_head_t		irq_queue;
	struct media_pad pads[MXC_HDMI_RX_PADS_NUM];

	struct platform_device		*pdev;
	struct v4l2_device			*v4l2_dev;
	struct v4l2_subdev sd;
	struct v4l2_async_subdev asd;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_mbus_framefmt format;
	struct v4l2_fract aspect_ratio;
	struct {
		u8 edid[512];
		u32 present;
		u32 blocks;
	} edid;

	state_struct state;
	struct clk		*sclk;
	struct clk		*pclk;
	struct clk		*ref_clk;
	struct clk		*pxl_clk;
	struct clk		*enc_clk;
	struct clk		*i2s_clk;
	struct clk		*spdif_clk;
	struct clk		*pxl_link_clk;
	struct hdp_mem mem;

	u32 flags;
	sc_ipc_t ipcHndl;
	u32 mu_id;
    S_HDMI_SCDC_GET_MSG          scdcData;

	struct mxc_hdmi_rx_dev_video_standards *timings;
    u8 vic_code;
	u8 hdmi_vic;
    u8 pixel_encoding;
	u8 color_depth;

	u8 is_cec;
	struct imx_cec_dev cec;
	u32 sample_rate;
	u32 sample_width;
	u32 channels;
};

enum mxc_hdmi_rx_power_state {
	MXC_HDMI_RX_PM_SUSPENDED = 0x01,
	MXC_HDMI_RX_PM_POWERED = 0x02,
	MXC_HDMI_RX_RUNTIME_SUSPEND = 0x04,
};

int hdmirx_startup(state_struct *state);
void imx8qm_hdmi_phy_reset(state_struct *state, u8 reset);
int hdmi_rx_init(state_struct *state);
int mxc_hdmi_frame_timing(struct mxc_hdmi_rx_dev *hdmi_rx);
void mxc_hdmi_rx_register_audio_driver(struct device *dev);

#endif
