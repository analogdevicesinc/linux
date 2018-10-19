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
#include <linux/irq.h>
#include "mxc-hdmi-rx.h"
#include "API_AFE_ss28fdsoi_hdmirx.h"

#define MXC_HDMI_DRIVER_NAME "iMX HDMI RX"
#define MXC_HDMI_MIN_WIDTH		640
#define MXC_HDMI_MAX_WIDTH		3840
#define MXC_HDMI_MIN_HEIGHT		480
#define MXC_HDMI_MAX_HEIGHT		2160

/* V4L2_DV_BT_CEA_640X480P59_94 */
#define MXC_HDMI_MIN_PIXELCLOCK	24000000
/* V4L2_DV_BT_CEA_3840X2160P30 */
#define MXC_HDMI_MAX_PIXELCLOCK	297000000

#define imx_sd_to_hdmi(sd) container_of(sd, struct mxc_hdmi_rx_dev, sd)

static void mxc_hdmi_cec_init(struct mxc_hdmi_rx_dev *hdmi_rx);

static const struct v4l2_dv_timings_cap mxc_hdmi_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },

	V4L2_INIT_BT_TIMINGS(MXC_HDMI_MIN_WIDTH, MXC_HDMI_MAX_WIDTH,
			     MXC_HDMI_MIN_HEIGHT, MXC_HDMI_MAX_HEIGHT,
			     MXC_HDMI_MIN_PIXELCLOCK,
			     MXC_HDMI_MAX_PIXELCLOCK,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT,
			     V4L2_DV_BT_CAP_PROGRESSIVE)
};

struct mxc_hdmi_rx_dev_video_standards
mxc_hdmi_video_standards[] = {
	{V4L2_DV_BT_CEA_640X480P59_94, 1, 0, 60},
	{V4L2_DV_BT_CEA_720X480P59_94, 2, 0, 60},
	{V4L2_DV_BT_CEA_720X480P59_94, 3, 0, 60},
	{V4L2_DV_BT_CEA_720X576P50,   18, 0, 50},
	{V4L2_DV_BT_CEA_720X576P50,   17, 0, 50},
	{V4L2_DV_BT_CEA_1280X720P60,   4, 0, 60},
	{V4L2_DV_BT_CEA_1280X720P50,  19, 0, 50},
	{V4L2_DV_BT_CEA_1280X720P30,  62, 0, 30},
	{V4L2_DV_BT_CEA_1280X720P25,  61, 0, 25},
	{V4L2_DV_BT_CEA_1280X720P24,  60, 0, 24},
	{V4L2_DV_BT_CEA_1920X1080P60, 16, 0, 60},
	{V4L2_DV_BT_CEA_1920X1080P50, 31, 0, 50},
	{V4L2_DV_BT_CEA_1920X1080P30, 34, 0, 30},
	{V4L2_DV_BT_CEA_1920X1080P25, 33, 0, 25},
	{V4L2_DV_BT_CEA_1920X1080P24, 32, 0, 24},
	{V4L2_DV_BT_CEA_3840X2160P24, 93, 3, 24},
	{V4L2_DV_BT_CEA_3840X2160P25, 94, 2, 25},
	{V4L2_DV_BT_CEA_3840X2160P30, 95, 1, 30},
	{V4L2_DV_BT_CEA_4096X2160P24, 98, 4, 24},
	{V4L2_DV_BT_CEA_4096X2160P25, 99, 0, 25},
	{V4L2_DV_BT_CEA_4096X2160P30, 100, 0, 30},
	/* SVGA */
	{V4L2_DV_BT_DMT_800X600P56, 0x0, 0, 56},
	{V4L2_DV_BT_DMT_800X600P60, 0x0, 0, 60},
	{V4L2_DV_BT_DMT_800X600P72, 0x0, 0, 72},
	{V4L2_DV_BT_DMT_800X600P75, 0x0, 0, 75},
	{V4L2_DV_BT_DMT_800X600P85, 0x0, 0, 85},
	/* SXGA */
	{V4L2_DV_BT_DMT_1280X1024P60, 0x0, 0, 60},
	{V4L2_DV_BT_DMT_1280X1024P75, 0x0, 0, 75},
	/* VGA */
	{ V4L2_DV_BT_DMT_640X480P72, 0x0, 0, 72},
	{ V4L2_DV_BT_DMT_640X480P75, 0x0, 0, 75},
	{ V4L2_DV_BT_DMT_640X480P85, 0x0, 0, 85},
	/* XGA */
	{ V4L2_DV_BT_DMT_1024X768P60, 0x0, 0, 60},
	{ V4L2_DV_BT_DMT_1024X768P70, 0x0, 0, 70},
	{ V4L2_DV_BT_DMT_1024X768P75, 0x0, 0, 75},
	{ V4L2_DV_BT_DMT_1024X768P85, 0x0, 0, 85},
	/* UXGA */
	{ V4L2_DV_BT_DMT_1600X1200P60, 0x0, 0, 60},
};

int mxc_hdmi_frame_timing(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	struct mxc_hdmi_rx_dev_video_standards *stds;
	u32 i, vic, hdmi_vic;

	vic = hdmi_rx->vic_code;
	hdmi_vic = hdmi_rx->hdmi_vic;
	stds = mxc_hdmi_video_standards;

	if (vic == 0 && hdmi_vic != 0) {
		for (i = 0; i < ARRAY_SIZE(mxc_hdmi_video_standards); i++) {
			if (stds[i].hdmi_vic == hdmi_vic) {
				hdmi_rx->timings = &stds[i];
				return true;
			}
		}
	} else if (vic > 109) {
		dev_err(&hdmi_rx->pdev->dev,
				"Unsupported mode vic=%d, hdmi_vic=%d\n", vic, hdmi_vic);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(mxc_hdmi_video_standards); i++) {
		if (stds[i].vic == vic) {
			hdmi_rx->timings = &stds[i];
			return true;
		}
	}

	if (i >= ARRAY_SIZE(mxc_hdmi_video_standards))
		return -EINVAL;

	return true;
}

static int mxc_hdmi_clock_init(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	struct device *dev = &hdmi_rx->pdev->dev;

	hdmi_rx->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(hdmi_rx->ref_clk)) {
		dev_err(dev, "failed to get hdmi rx ref clk\n");
		return PTR_ERR(hdmi_rx->ref_clk);
	}

	hdmi_rx->pxl_clk = devm_clk_get(dev, "pxl_clk");
	if (IS_ERR(hdmi_rx->pxl_clk)) {
		dev_err(dev, "failed to get hdmi rx pxl clk\n");
		return PTR_ERR(hdmi_rx->pxl_clk);
	}
	hdmi_rx->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(hdmi_rx->pclk)) {
		dev_err(dev, "failed to get hdmi rx pclk\n");
		return PTR_ERR(hdmi_rx->pclk);
	}

	hdmi_rx->sclk = devm_clk_get(dev, "sclk");
	if (IS_ERR(hdmi_rx->sclk)) {
		dev_err(dev, "failed to get hdmi rx sclk\n");
		return PTR_ERR(hdmi_rx->sclk);
	}

	hdmi_rx->enc_clk = devm_clk_get(dev, "enc_clk");
	if (IS_ERR(hdmi_rx->enc_clk)) {
		dev_err(dev, "failed to get hdmi rx enc clk\n");
		return PTR_ERR(hdmi_rx->enc_clk);
	}

	hdmi_rx->i2s_clk = devm_clk_get(dev, "i2s_clk");
	if (IS_ERR(hdmi_rx->i2s_clk)) {
		dev_err(dev, "failed to get hdmi rx i2s clk\n");
		return PTR_ERR(hdmi_rx->i2s_clk);
	}

	hdmi_rx->spdif_clk = devm_clk_get(dev, "spdif_clk");
	if (IS_ERR(hdmi_rx->spdif_clk)) {
		dev_err(dev, "failed to get hdmi rx spdif clk\n");
		return PTR_ERR(hdmi_rx->spdif_clk);
	}

	hdmi_rx->pxl_link_clk = devm_clk_get(dev, "pxl_link_clk");
	if (IS_ERR(hdmi_rx->pxl_link_clk)) {
		dev_err(dev, "failed to get hdmi rx pixel link clk\n");
		return PTR_ERR(hdmi_rx->pxl_link_clk);
	}

	return 0;
}

static int mxc_hdmi_clock_enable(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	struct device *dev = &hdmi_rx->pdev->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	ret = clk_prepare_enable(hdmi_rx->ref_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre ref_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmi_rx->pxl_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pxl_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmi_rx->enc_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre enc_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmi_rx->sclk);
	if (ret < 0) {
		dev_err(dev, "%s, pre sclk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmi_rx->pclk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pclk error %d\n", __func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(hdmi_rx->i2s_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre i2s_clk error %d\n", __func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(hdmi_rx->spdif_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre spdif_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmi_rx->pxl_link_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pxl_link_clk error %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void mxc_hdmi_clock_disable(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);

	clk_disable_unprepare(hdmi_rx->ref_clk);
	clk_disable_unprepare(hdmi_rx->pxl_clk);
	clk_disable_unprepare(hdmi_rx->enc_clk);
	clk_disable_unprepare(hdmi_rx->sclk);
	clk_disable_unprepare(hdmi_rx->pclk);
	clk_disable_unprepare(hdmi_rx->i2s_clk);
	clk_disable_unprepare(hdmi_rx->spdif_clk);
	clk_disable_unprepare(hdmi_rx->pxl_link_clk);
}

static void mxc_hdmi_pixel_link_encoder(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	u32 val;

	switch (hdmi_rx->pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		val = 3;
		break;
	case PIXEL_ENCODING_YUV420:
		val = 4;
		break;
	case PIXEL_ENCODING_YUV444:
		val = 2;
		break;
	case PIXEL_ENCODING_RGB:
		val = 0;
		break;
	default:
		val = 0x6;
	}

	/* HDMI RX H/Vsync Polarity */
	if (hdmi_rx->timings->timings.bt.polarities & V4L2_DV_VSYNC_POS_POL)
		val |= 1 << PL_ENC_CTL_PXL_VCP;
	if (hdmi_rx->timings->timings.bt.polarities & V4L2_DV_HSYNC_POS_POL)
		val |= 1 << PL_ENC_CTL_PXL_HCP;

	writel(val, hdmi_rx->mem.ss_base);
}

/* -----------------------------------------------------------------------------
 * v4l2_subdev_video_ops
 */
static int mxc_hdmi_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	struct device *dev = &hdmi_rx->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static int mxc_hdmi_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_captureparm *cparm = &a->parm.capture;
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->timeperframe.denominator = hdmi_rx->timings->fps;
		cparm->timeperframe.numerator = 1;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;
	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int mxc_hdmi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	u32 val;

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);
	mxc_hdmi_pixel_link_encoder(hdmi_rx);

	if (enable) {
		val = readl(hdmi_rx->mem.ss_base);
		val |=  1 << PL_ENC_CTL_PXL_EN;
		writel(val, hdmi_rx->mem.ss_base);
		mdelay(17);

		val = readl(hdmi_rx->mem.ss_base);
		val |=  1 << PL_ENC_CTL_PXL_VAL;
		writel(val, hdmi_rx->mem.ss_base);
	} else {
		val = readl(hdmi_rx->mem.ss_base);
		val |=  ~(1 << PL_ENC_CTL_PXL_VAL);
		writel(val, hdmi_rx->mem.ss_base);
		mdelay(17);

		val = readl(hdmi_rx->mem.ss_base);
		val |=  ~(1 << PL_ENC_CTL_PXL_EN);
		writel(val, hdmi_rx->mem.ss_base);
	}

	return 0;
}

static const struct v4l2_subdev_video_ops imx_video_ops_hdmi = {
	.s_stream = mxc_hdmi_s_stream,
	.g_parm =	mxc_hdmi_g_parm,
	.s_parm =	mxc_hdmi_s_parm,
};

/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations hdmi_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * v4l2_subdev_pad_ops
 */
static int mxc_hdmi_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);

	if (fse->index > 1)
		return -EINVAL;

	fse->min_width = hdmi_rx->timings->timings.bt.width;
	fse->max_width = fse->min_width;

	fse->min_height = hdmi_rx->timings->timings.bt.height;
	fse->max_height = fse->min_height;
	return 0;
}
static int mxc_hdmi_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);

	if (fie->index < 0 || fie->index > 8)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	 /* TODO Reserved to extension */
	fie->interval.denominator = hdmi_rx->timings->fps;
	return 0;
}

static int mxc_hdmi_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_RGB888_1X24;

	return 0;
}

static int mxc_hdmi_get_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *sdformat)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	struct v4l2_mbus_framefmt *mbusformat = &sdformat->format;

	if (sdformat->pad != MXC_HDMI_RX_PAD_SOURCE)
		return -EINVAL;

	switch (hdmi_rx->pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		mbusformat->code = MEDIA_BUS_FMT_YUYV8_1X16;
		break;
	case PIXEL_ENCODING_YUV420:
		mbusformat->code = MEDIA_BUS_FMT_UV8_1X8;
		break;
	case PIXEL_ENCODING_YUV444:
		mbusformat->code = MEDIA_BUS_FMT_AYUV8_1X32;
		break;
	default:
		mbusformat->code = MEDIA_BUS_FMT_RGB888_1X24;
	}

	mbusformat->width =  hdmi_rx->timings->timings.bt.width;
	mbusformat->height = hdmi_rx->timings->timings.bt.height;
	mbusformat->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int mxc_hdmi_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (!hdmi_rx->edid.present)
		return -ENODATA;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = hdmi_rx->edid.blocks;
		return 0;
	}

	if (edid->start_block >= hdmi_rx->edid.blocks)
		return -EINVAL;

	if (edid->start_block + edid->blocks > hdmi_rx->edid.blocks)
		edid->blocks = hdmi_rx->edid.blocks - edid->start_block;

	memcpy(edid->edid, hdmi_rx->edid.edid + edid->start_block * 128,
			edid->blocks * 128);

	return 0;
}

static int mxc_hdmi_set_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	state_struct *state = &hdmi_rx->state;
	int err, i;

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks == 0) {
		/* Default EDID */
		hdmi_rx->edid.blocks = 2;
		hdmi_rx->edid.present = true;
	} else if (edid->blocks > 4) {
		edid->blocks = 4;
		return -E2BIG;
	} else {
		memcpy(hdmi_rx->edid.edid, edid->edid, 128 * edid->blocks);
		hdmi_rx->edid.blocks = edid->blocks;
		hdmi_rx->edid.present = true;
	}

	for (i = 0; i < hdmi_rx->edid.blocks; i++) {
		/* EDID block */
		err = CDN_API_HDMIRX_SET_EDID_blocking(state, 0, i, &hdmi_rx->edid.edid[i * 128]);
		if (err != CDN_OK) {
			v4l2_err(sd, "error %d writing edid pad %d\n", err, edid->pad);
			return -err;
		}
	}

	return 0;
}

static bool mxc_hdmi_check_dv_timings(const struct v4l2_dv_timings *timings,
					  void *hdl)
{
	const struct mxc_hdmi_rx_dev_video_standards *stds =
		mxc_hdmi_video_standards;
	u32 i;

	for (i = 0; stds[i].timings.bt.width; i++)
		if (v4l2_match_dv_timings(timings, &stds[i].timings, 0, false))
			return true;

	return false;
}

static int mxc_hdmi_enum_dv_timings(struct v4l2_subdev *sd,
					struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &mxc_hdmi_timings_cap,
					mxc_hdmi_check_dv_timings, NULL);
}

static int mxc_hdmi_dv_timings_cap(struct v4l2_subdev *sd,
				       struct v4l2_dv_timings_cap *cap)
{
	*cap = mxc_hdmi_timings_cap;
	return 0;
}

static const struct v4l2_subdev_pad_ops imx_pad_ops_hdmi = {
	.enum_mbus_code = mxc_hdmi_enum_mbus_code,
	.enum_frame_size = mxc_hdmi_enum_framesizes,
	.enum_frame_interval = mxc_hdmi_enum_frame_interval,
	.get_fmt = mxc_hdmi_get_format,
	.get_edid = mxc_hdmi_get_edid,
	.set_edid = mxc_hdmi_set_edid,
	.dv_timings_cap = mxc_hdmi_dv_timings_cap,
	.enum_dv_timings = mxc_hdmi_enum_dv_timings,
};

static int mxc_hdmi_s_power(struct v4l2_subdev *sd, int on)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = imx_sd_to_hdmi(sd);
	struct device *dev = &hdmi_rx->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}
static struct v4l2_subdev_core_ops imx_core_ops_hdmi = {
	.s_power = mxc_hdmi_s_power,
};

/* -----------------------------------------------------------------------------
 * v4l2_subdev_ops
 */
static const struct v4l2_subdev_ops imx_ops_hdmi = {
	.core = &imx_core_ops_hdmi,
	.video = &imx_video_ops_hdmi,
	.pad = &imx_pad_ops_hdmi,
};

void imx8qm_hdmi_phy_reset(state_struct *state, u8 reset)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	sc_err_t sciErr;

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);
	/* set the pixel link mode and pixel type */
	sciErr = sc_misc_set_control(hdmi_rx->ipcHndl, SC_R_HDMI_RX, SC_C_PHY_RESET, reset);
	if (sciErr != SC_ERR_NONE)
		DRM_ERROR("SC_R_HDMI PHY reset failed %d!\n", sciErr);
}

static int imx8qm_hdp_read(struct hdp_mem *mem, u32 addr, u32 *value)
{
	u32 temp;
	void *tmp_addr;
	void *off_addr;

	mutex_lock(&mem->mutex);
	tmp_addr = (addr & 0xfff) + mem->regs_base;
	off_addr = 0x4 + mem->ss_base;
	__raw_writel(addr >> 12, off_addr);
	temp = __raw_readl((volatile u32 *)tmp_addr);

	*value = temp;
	mutex_unlock(&mem->mutex);
	return 0;
}

static int imx8qm_hdp_write(struct hdp_mem *mem, u32 addr, u32 value)
{
	void *tmp_addr;
	void *off_addr;

	mutex_lock(&mem->mutex);
	tmp_addr = (addr & 0xfff) + mem->regs_base;
	off_addr = 0x4 + mem->ss_base;;
	__raw_writel(addr >> 12, off_addr);

	__raw_writel(value, (volatile u32 *) tmp_addr);
	mutex_unlock(&mem->mutex);

	return 0;
}

static int imx8qm_hdp_sread(struct hdp_mem *mem, u32 addr, u32 *value)
{
	u32 temp;
	void *tmp_addr;
	void *off_addr;

	mutex_lock(&mem->mutex);
	tmp_addr = (addr & 0xfff) + mem->regs_base;
	off_addr = 0xc + mem->ss_base;
	__raw_writel(addr >> 12, off_addr);

	temp = __raw_readl((volatile u32 *)tmp_addr);
	*value = temp;
	mutex_unlock(&mem->mutex);
	return 0;
}

static int imx8qm_hdp_swrite(struct hdp_mem *mem, u32 addr, u32 value)
{
	void *tmp_addr;
	void *off_addr;

	mutex_lock(&mem->mutex);
	tmp_addr = (addr & 0xfff) + mem->regs_base;
	off_addr = 0xc + mem->ss_base;
	__raw_writel(addr >> 12, off_addr);
	__raw_writel(value, (volatile u32 *)tmp_addr);
	mutex_unlock(&mem->mutex);

	return 0;
}
static struct hdp_rw_func imx8qm_rw = {
	.read_reg = imx8qm_hdp_read,
	.write_reg = imx8qm_hdp_write,
	.sread_reg = imx8qm_hdp_sread,
	.swrite_reg = imx8qm_hdp_swrite,
};

static void mxc_hdmi_state_init(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	state_struct *state = &hdmi_rx->state;

	memset(state, 0, sizeof(state_struct));
	mutex_init(&state->mutex);

	state->mem = &hdmi_rx->mem;
	state->rw = &imx8qm_rw;
}

#ifdef CONFIG_IMX_HDP_CEC
static void mxc_hdmi_cec_init(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	state_struct *state = &hdmi_rx->state;
	struct imx_cec_dev *cec = &hdmi_rx->cec;
	u32 clk_rate;

	memset(cec, 0, sizeof(struct imx_cec_dev));

	CDN_API_GetClock(state, &clk_rate);
	cec->clk_div = clk_rate * 10;
	cec->dev = &hdmi_rx->pdev->dev;
	cec->mem = &hdmi_rx->mem;
	cec->rw = &imx8qm_rw;
}
#endif


int mxc_hdmi_init(struct mxc_hdmi_rx_dev *hdmi_rx)
{
	sc_err_t sciErr;

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);
	mxc_hdmi_state_init(hdmi_rx);

	sciErr = sc_ipc_getMuID(&hdmi_rx->mu_id);
	if (sciErr != SC_ERR_NONE) {
		DRM_ERROR("Cannot obtain MU ID\n");
		return -EINVAL;
	}

	sciErr = sc_ipc_open(&hdmi_rx->ipcHndl, hdmi_rx->mu_id);
	if (sciErr != SC_ERR_NONE) {
		DRM_ERROR("sc_ipc_open failed! (sciError = %d)\n", sciErr);
		return -EINVAL;
	}


	return 0;
}

static void hpd5v_work_func(struct work_struct *work)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = container_of(work, struct mxc_hdmi_rx_dev,
								hpd5v_work.work);
	char event_string[32];
	char *envp[] = { event_string, NULL };
	u8 sts;
	u8 hpd;

	/* Check cable states before enable irq */
	hdmirx_get_hpd_state(&hdmi_rx->state, &hpd);
	if (hpd == 1) {
		pr_info("HDMI RX Cable Plug In\n");

		CDN_API_MainControl_blocking(&hdmi_rx->state, 1, &sts);
		hdmirx_hotplug_trigger(&hdmi_rx->state);
		hdmirx_startup(&hdmi_rx->state);
		enable_irq(hdmi_rx->irq[HPD5V_IRQ_OUT]);
		sprintf(event_string, "EVENT=hdmirxin");
		kobject_uevent_env(&hdmi_rx->pdev->dev.kobj, KOBJ_CHANGE, envp);
#ifdef CONFIG_IMX_HDP_CEC
		if (hdmi_rx->is_cec) {
			mxc_hdmi_cec_init(hdmi_rx);
			imx_cec_register(&hdmi_rx->cec);
		}
#endif
	} else if (hpd == 0){
		pr_info("HDMI RX Cable Plug Out\n");
		hdmirx_stop(&hdmi_rx->state);
#ifdef CONFIG_IMX_HDP_CEC
		if (hdmi_rx->is_cec) {
			imx_cec_unregister(&hdmi_rx->cec);
		}
#endif
		hdmirx_phy_pix_engine_reset(&hdmi_rx->state);
		sprintf(event_string, "EVENT=hdmirxout");
		kobject_uevent_env(&hdmi_rx->pdev->dev.kobj, KOBJ_CHANGE, envp);
		enable_irq(hdmi_rx->irq[HPD5V_IRQ_IN]);
		CDN_API_MainControl_blocking(&hdmi_rx->state, 0, &sts);
	} else
		pr_warn("HDMI RX Cable State unknow\n");

}

#define HOTPLUG_DEBOUNCE_MS		200
static irqreturn_t mxc_hdp5v_irq_thread(int irq, void *data)
{
	struct mxc_hdmi_rx_dev *hdmi_rx =  data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &hdmi_rx->hpd5v_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static int mxc_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxc_hdmi_rx_dev *hdmi_rx;
	struct resource *res;
	u8 hpd;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);
	hdmi_rx = devm_kzalloc(dev, sizeof(*hdmi_rx), GFP_KERNEL);
	if (!hdmi_rx)
		return -ENOMEM;

	hdmi_rx->pdev = pdev;

	mutex_init(&hdmi_rx->mem.mutex);
	/* register map */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi_rx->mem.regs_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi_rx->mem.regs_base)) {
		dev_err(dev, "Failed to get HDMI RX CTRL base register\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	hdmi_rx->mem.ss_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi_rx->mem.ss_base)) {
		dev_err(dev, "Failed to get HDMI RX CRS base register\n");
		return -EINVAL;
	}

	hdmi_rx->irq[HPD5V_IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (hdmi_rx->irq[HPD5V_IRQ_IN] < 0)
		dev_info(&pdev->dev, "No plug_in irq number\n");

	hdmi_rx->irq[HPD5V_IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (hdmi_rx->irq[HPD5V_IRQ_OUT] < 0)
		dev_info(&pdev->dev, "No plug_out irq number\n");

	INIT_DELAYED_WORK(&hdmi_rx->hpd5v_work, hpd5v_work_func);


	v4l2_subdev_init(&hdmi_rx->sd, &imx_ops_hdmi);
	/* sd.dev may use by match_of */
	hdmi_rx->sd.dev = dev;

	/* the owner is the same as the i2c_client's driver owner */
	hdmi_rx->sd.owner = THIS_MODULE;
	hdmi_rx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	hdmi_rx->sd.entity.function = MEDIA_ENT_F_IO_DTV;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&hdmi_rx->sd, pdev);

	/* initialize name */
	snprintf(hdmi_rx->sd.name, sizeof(hdmi_rx->sd.name), "%s",
		MXC_HDMI_RX_SUBDEV_NAME);

	hdmi_rx->pads[MXC_HDMI_RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	hdmi_rx->pads[MXC_HDMI_RX_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	hdmi_rx->sd.entity.ops = &hdmi_media_ops;
	ret = media_entity_pads_init(&hdmi_rx->sd.entity,
				     MXC_HDMI_RX_PADS_NUM, hdmi_rx->pads);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, hdmi_rx);
	ret = v4l2_async_register_subdev(&hdmi_rx->sd);
	if (ret < 0) {
		dev_err(dev,
					"%s--Async register failed, ret=%d\n", __func__, ret);
		media_entity_cleanup(&hdmi_rx->sd.entity);
	}

	hdmi_rx->is_cec = of_property_read_bool(pdev->dev.of_node, "fsl,cec");

	mxc_hdmi_clock_init(hdmi_rx);

	hdmi_rx->flags = MXC_HDMI_RX_PM_POWERED;

	mxc_hdmi_clock_enable(hdmi_rx);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	ret = mxc_hdmi_init(hdmi_rx);
	if (ret < 0) {
		dev_err(dev, "mxc hdmi init failed\n");
		goto failed;
	}
	ret = hdmirx_init(&hdmi_rx->state);
	if (ret < 0) {
		dev_err(dev, "mxc hdmi rx init failed\n");
		goto failed;
	}

	/* Check cable states before enable irq */
	hdmirx_get_hpd_state(&hdmi_rx->state, &hpd);

	/* Enable Hotplug Detect IRQ thread */
	if (hdmi_rx->irq[HPD5V_IRQ_IN] > 0) {
		irq_set_status_flags(hdmi_rx->irq[HPD5V_IRQ_IN], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdmi_rx->irq[HPD5V_IRQ_IN],
						NULL, mxc_hdp5v_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdmi_rx);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
							hdmi_rx->irq[HPD5V_IRQ_IN]);
			goto failed;
		}
		/* Cable Disconnedted, enable Plug in IRQ */
		if (hpd == 0)
			enable_irq(hdmi_rx->irq[HPD5V_IRQ_IN]);
	}
	if (hdmi_rx->irq[HPD5V_IRQ_OUT] > 0) {
		irq_set_status_flags(hdmi_rx->irq[HPD5V_IRQ_OUT], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdmi_rx->irq[HPD5V_IRQ_OUT],
						NULL, mxc_hdp5v_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdmi_rx);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
							hdmi_rx->irq[HPD5V_IRQ_OUT]);
			goto failed;
		}
		if (hpd == 1) {
			hdmirx_hotplug_trigger(&hdmi_rx->state);
			hdmirx_startup(&hdmi_rx->state);
			/* Cable Connected, enable Plug out IRQ */
			enable_irq(hdmi_rx->irq[HPD5V_IRQ_OUT]);
		}
	}

	mxc_hdmi_rx_register_audio_driver(dev);

	dev_info(dev, "iMX8 HDMI RX probe successfully\n");

	return ret;
failed:
	v4l2_async_unregister_subdev(&hdmi_rx->sd);
	media_entity_cleanup(&hdmi_rx->sd.entity);

	mxc_hdmi_clock_disable(hdmi_rx);
	pm_runtime_disable(dev);
	dev_info(dev, "mxc hdmi rx probe failed\n");
	return ret;
}

static int mxc_hdmi_remove(struct platform_device *pdev)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = platform_get_drvdata(pdev);
	state_struct *state = &hdmi_rx->state;
	struct device *dev = &pdev->dev;
	u8 sts;

	dev_dbg(dev, "%s\n", __func__);
	v4l2_async_unregister_subdev(&hdmi_rx->sd);
	media_entity_cleanup(&hdmi_rx->sd.entity);

#ifdef CONFIG_IMX_HDP_CEC
	if (hdmi_rx->is_cec)
		imx_cec_unregister(&hdmi_rx->cec);
#endif

	/* Reset HDMI RX PHY */
	CDN_API_HDMIRX_Stop_blocking(state);
	CDN_API_MainControl_blocking(state, 0, &sts);

	mxc_hdmi_clock_disable(hdmi_rx);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxc_hdmi_pm_suspend(struct device *dev)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if ((hdmi_rx->flags & MXC_HDMI_RX_PM_SUSPENDED) ||
		(hdmi_rx->flags & MXC_HDMI_RX_RUNTIME_SUSPEND))
		return 0;

	mxc_hdmi_clock_disable(hdmi_rx);
	hdmi_rx->flags |= MXC_HDMI_RX_PM_SUSPENDED;
	hdmi_rx->flags &= ~MXC_HDMI_RX_PM_POWERED;

	return 0;
}

static int mxc_hdmi_pm_resume(struct device *dev)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	if (hdmi_rx->flags & MXC_HDMI_RX_PM_POWERED)
		return 0;

	hdmi_rx->flags |= MXC_HDMI_RX_PM_POWERED;
	hdmi_rx->flags &= ~MXC_HDMI_RX_PM_SUSPENDED;

	ret = mxc_hdmi_clock_enable(hdmi_rx);
	return (ret) ? -EAGAIN : 0;
}
#endif

static int mxc_hdmi_runtime_suspend(struct device *dev)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (hdmi_rx->flags & MXC_HDMI_RX_RUNTIME_SUSPEND)
		return 0;

	if (hdmi_rx->flags & MXC_HDMI_RX_PM_POWERED) {
		mxc_hdmi_clock_disable(hdmi_rx);
		hdmi_rx->flags |= MXC_HDMI_RX_RUNTIME_SUSPEND;
		hdmi_rx->flags &= ~MXC_HDMI_RX_PM_POWERED;
	}
	return 0;
}

static int mxc_hdmi_runtime_resume(struct device *dev)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (hdmi_rx->flags & MXC_HDMI_RX_PM_POWERED)
		return 0;

	if (hdmi_rx->flags & MXC_HDMI_RX_RUNTIME_SUSPEND) {
		mxc_hdmi_clock_enable(hdmi_rx);
		hdmi_rx->flags |= MXC_HDMI_RX_PM_POWERED;
		hdmi_rx->flags &= ~MXC_HDMI_RX_RUNTIME_SUSPEND;
	}
	return 0;
}

static const struct dev_pm_ops mxc_hdmi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mxc_hdmi_pm_suspend, mxc_hdmi_pm_resume)
	SET_RUNTIME_PM_OPS(mxc_hdmi_runtime_suspend, mxc_hdmi_runtime_resume, NULL)
};

static const struct of_device_id mxc_hdmi_of_match[] = {
	{.compatible = "fsl,imx-hdmi-rx",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mxc_hdmi_of_match);

static struct platform_driver mxc_hdmi_driver = {
	.probe		= mxc_hdmi_probe,
	.remove		= mxc_hdmi_remove,
	.driver = {
		.of_match_table = mxc_hdmi_of_match,
		.name		= MXC_HDMI_RX_DRIVER_NAME,
		.pm		= &mxc_hdmi_pm_ops,
	}
};

module_platform_driver(mxc_hdmi_driver);
