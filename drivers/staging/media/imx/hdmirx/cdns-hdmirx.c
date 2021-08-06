/*
 * Copyright 2019-2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/sci.h>
#include <linux/freezer.h>
#include <linux/irq.h>
#include <linux/kthread.h>

#include "cdns-hdmirx-phy.h"
#include "cdns-mhdp-hdmirx.h"

#define HDMIRX_MIN_WIDTH		640
#define HDMIRX_MAX_WIDTH		3840
#define HDMIRX_MIN_HEIGHT		480
#define HDMIRX_MAX_HEIGHT		2160
/* V4L2_DV_BT_CEA_640X480P59_94 */
#define HDMIRX_MIN_PIXELCLOCK	24000000
/* V4L2_DV_BT_CEA_3840X2160P30 */
#define HDMIRX_MAX_PIXELCLOCK	297000000

#define VIDEO_MODE_CHANGE 0x40

#define cdns_sd_to_hdmi(sd) container_of(sd, struct cdns_hdmirx_device, sd)

/* Attibutes for sysfs */
static ssize_t HDCPRX_status_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t HDCPRX_request_reauthentication_show(struct device *dev,
			struct device_attribute *attr, char *buf);

static ssize_t HDCPRX_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute HDCPRX_status = __ATTR_RO(HDCPRX_status);
static struct device_attribute HDCPRX_request_reauthentication = __ATTR_RO(HDCPRX_request_reauthentication);
static struct device_attribute HDCPRX_enable = __ATTR_WO(HDCPRX_enable);

static ssize_t HDCPRX_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hdcprx_status status;
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);
	ssize_t size = 0;

	if (NULL != hdmirx) {
		if (hdmirx->initialized == true && hdmirx->hdcp_fw_loaded == true
					&& !cdns_hdcprx_get_status(hdmirx, &status))
			size = sprintf(buf, "HCDP key_arrived 0x%02x\nHCDP hdcp_ver 0x%02x\nHCDP error 0x%02x\n",
					status.flags & 1,
					(status.flags >> 1) & 0x3,
					(status.flags >> 4) & 0xF);
		else
			size = sprintf(buf, "HDCPRX status read failed, link is not up or HDCP not enabled.\n");
	} else
		size = sprintf(buf, "hdmirx is NULL\n");
	return size;
}

static ssize_t HDCPRX_request_reauthentication_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);
	ssize_t size;

	if (NULL != hdmirx && hdmirx->initialized == true &&
			hdmirx->hdcp_fw_loaded == true) {
		if (!cdns_hdcprx_reauth_req_wait(hdmirx, 2000))
			size = sprintf(buf, "cdns_hdcprx_reauth_req done\n");
		else {
			size = sprintf(buf, "HDCPRX cdns_hdcprx_reauth_req failed.\n");
			return size;
		}
	} else
		size = sprintf(buf, "hdmirx is NULL or HDCP not enabled\n");

	return size;
}

static ssize_t HDCPRX_enable_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
    int value;
    struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);

	if (hdmirx == NULL) {
		dev_dbg(&hdmirx->pdev->dev, "%s hdmirx\n", __func__);
		return -1;
	}

    sscanf(buf, "%d", &value);
    /* HDCP start/stop and load/unload of modules will be handled
       in the tmdsmon function. We just setup the flags here.
    */
	if (value == 1) {
		dev_dbg(&hdmirx->pdev->dev, "%s enable hdcp\n", __func__);
		hdmirx->allow_hdcp = true;
	} else if (value == 0) {
		dev_dbg(&hdmirx->pdev->dev, "%s disable hdcp\n", __func__);
		hdmirx->allow_hdcp = false;
	} else
		dev_dbg(&hdmirx->pdev->dev, "%s invalid hdcp command\n", __func__);

    return count;
}

static int hdmi_sysfs_init(struct device *dev)
{
	int retval;

	retval = device_create_file(dev, &HDCPRX_status);
	if (retval) {
		printk(KERN_ERR "Unable to create hdmirx status sysfs\n");
		device_remove_file(dev, &HDCPRX_status);
		return retval;
	}

	retval = device_create_file(dev, &HDCPRX_enable);
	if (retval) {
		printk(KERN_ERR "Unable to create hdmirx enable sysfs\n");
		device_remove_file(dev, &HDCPRX_enable);
		return retval;
	}

	retval = device_create_file(dev, &HDCPRX_request_reauthentication);
	if (retval) {
		printk(KERN_ERR "Unable to create HDCPRX_request_reauthentication sysfs\n");
		device_remove_file(dev, &HDCPRX_request_reauthentication);
		return retval;
	}

	return 0;
}

static const struct v4l2_dv_timings_cap hdmirx_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },

	V4L2_INIT_BT_TIMINGS(HDMIRX_MIN_WIDTH, HDMIRX_MAX_WIDTH,
			     HDMIRX_MIN_HEIGHT, HDMIRX_MAX_HEIGHT,
			     HDMIRX_MIN_PIXELCLOCK,
			     HDMIRX_MAX_PIXELCLOCK,
			     V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT,
			     V4L2_DV_BT_CAP_PROGRESSIVE)
};

struct cdns_hdmirx_dev_video_standards
cdns_hdmi_video_standards[] = {
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

int cdns_hdmirx_frame_timing(struct cdns_hdmirx_device *hdmirx)
{
	struct cdns_hdmirx_dev_video_standards *stds;
	u32 i, vic, hdmi_vic;

	vic = hdmirx->vic_code;
	hdmi_vic = hdmirx->hdmi_vic;
	stds = cdns_hdmi_video_standards;

	if (vic == 0 && hdmi_vic != 0) {
		for (i = 0; i < ARRAY_SIZE(cdns_hdmi_video_standards); i++) {
			if (stds[i].hdmi_vic == hdmi_vic) {
				hdmirx->timings = &stds[i];
				return true;
			}
		}
	} else if (vic > 109) {
		dev_err(&hdmirx->pdev->dev,
				"Unsupported mode vic=%d, hdmi_vic=%d\n", vic, hdmi_vic);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(cdns_hdmi_video_standards); i++) {
		if (stds[i].vic == vic) {
			hdmirx->timings = &stds[i];
			return true;
		}
	}

	if (i >= ARRAY_SIZE(cdns_hdmi_video_standards))
		return -EINVAL;

	return true;
}

static int hdmirx_clock_init(struct cdns_hdmirx_device *hdmirx)
{
	struct device *dev = &hdmirx->pdev->dev;

	hdmirx->ref_clk = devm_clk_get(dev, "ref_clk");
	if (IS_ERR(hdmirx->ref_clk)) {
		dev_err(dev, "failed to get hdmi rx ref clk\n");
		return PTR_ERR(hdmirx->ref_clk);
	}

	hdmirx->pxl_clk = devm_clk_get(dev, "pxl_clk");
	if (IS_ERR(hdmirx->pxl_clk)) {
		dev_err(dev, "failed to get hdmi rx pxl clk\n");
		return PTR_ERR(hdmirx->pxl_clk);
	}

	hdmirx->lpcg_pclk = devm_clk_get(dev, "lpcg_pclk");
	if (IS_ERR(hdmirx->lpcg_pclk)) {
		dev_err(dev, "failed to get hdmi rx pclk\n");
		return PTR_ERR(hdmirx->lpcg_pclk);
	}

	hdmirx->lpcg_sclk = devm_clk_get(dev, "lpcg_sclk");
	if (IS_ERR(hdmirx->lpcg_sclk)) {
		dev_err(dev, "failed to get hdmi rx lpcg_sclk\n");
		return PTR_ERR(hdmirx->lpcg_sclk);
	}

	hdmirx->lpcg_enc_clk = devm_clk_get(dev, "lpcg_enc_clk");
	if (IS_ERR(hdmirx->lpcg_enc_clk)) {
		dev_err(dev, "failed to get hdmi rx lpcg_enc clk\n");
		return PTR_ERR(hdmirx->lpcg_enc_clk);
	}

	hdmirx->i2s_clk = devm_clk_get(dev, "i2s_clk");
	if (IS_ERR(hdmirx->i2s_clk)) {
		dev_err(dev, "failed to get hdmi rx i2s clk\n");
		return PTR_ERR(hdmirx->i2s_clk);
	}

	hdmirx->spdif_clk = devm_clk_get(dev, "spdif_clk");
	if (IS_ERR(hdmirx->spdif_clk)) {
		dev_err(dev, "failed to get hdmi rx spdif clk\n");
		return PTR_ERR(hdmirx->spdif_clk);
	}

	hdmirx->lpcg_pxl_link_clk = devm_clk_get(dev, "lpcg_pxl_link_clk");
	if (IS_ERR(hdmirx->lpcg_pxl_link_clk)) {
		dev_err(dev, "failed to get hdmi rx lpcg_pixel link clk\n");
		return PTR_ERR(hdmirx->lpcg_pxl_link_clk);
	}

	return 0;
}

static int hdmirx_clock_enable(struct cdns_hdmirx_device *hdmirx)
{
	struct device *dev = &hdmirx->pdev->dev;
	int ret;

	dev_dbg(dev, "%s\n", __func__);
	ret = clk_prepare_enable(hdmirx->ref_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre ref_clk error %d\n", __func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(hdmirx->pxl_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pxl_clk error %d\n", __func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(hdmirx->i2s_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre i2s_clk error %d\n", __func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(hdmirx->spdif_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre spdif_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmirx->lpcg_enc_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre enc_clk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmirx->lpcg_sclk);
	if (ret < 0) {
		dev_err(dev, "%s, pre sclk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmirx->lpcg_pclk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pclk error %d\n", __func__, ret);
		return ret;
	}
	ret = clk_prepare_enable(hdmirx->lpcg_pxl_link_clk);
	if (ret < 0) {
		dev_err(dev, "%s, pre pxl_link_clk error %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static void hdmirx_clock_disable(struct cdns_hdmirx_device *hdmirx)
{
	dev_dbg(&hdmirx->pdev->dev, "%s\n", __func__);

	clk_disable_unprepare(hdmirx->ref_clk);
	clk_disable_unprepare(hdmirx->pxl_clk);
	clk_disable_unprepare(hdmirx->lpcg_enc_clk);
	clk_disable_unprepare(hdmirx->lpcg_sclk);
	clk_disable_unprepare(hdmirx->lpcg_pclk);
	clk_disable_unprepare(hdmirx->i2s_clk);
	clk_disable_unprepare(hdmirx->spdif_clk);
	clk_disable_unprepare(hdmirx->lpcg_pxl_link_clk);
}

static void hdmirx_pixel_link_encoder(struct cdns_hdmirx_device *hdmirx)
{
	u32 val;

	switch (hdmirx->pixel_encoding) {
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
	if (hdmirx->timings->timings.bt.polarities & V4L2_DV_VSYNC_POS_POL)
		val |= 1 << PL_ENC_CTL_PXL_VCP;
	if (hdmirx->timings->timings.bt.polarities & V4L2_DV_HSYNC_POS_POL)
		val |= 1 << PL_ENC_CTL_PXL_HCP;

	writel(val, hdmirx->regs_sec);
}

/* -----------------------------------------------------------------------------
 * v4l2_subdev_video_ops
 */
static int hdmirx_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	struct device *dev = &hdmirx->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}

static int hdmirx_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_captureparm *cparm = &a->parm.capture;
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	int ret = 0;

	if (hdmirx->cable_plugin == false) {
		dev_warn(&hdmirx->pdev->dev, "No Cable Connected!\n");
		return -EINVAL;
	}

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->timeperframe.denominator = hdmirx->timings->fps;
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
		pr_debug("type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int hdmirx_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	struct device *dev = &hdmirx->pdev->dev;
	u32 val;

	dev_dbg(dev, "%s\n", __func__);
	if (hdmirx->cable_plugin == false) {
		dev_warn(dev, "No Cable Connected!\n");
		return -EINVAL;
	}

	hdmirx_pixel_link_encoder(hdmirx);

	if (enable) {
		pm_runtime_get_sync(dev);
		if (!hdmirx->running++) {
			val = readl(hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			val |=  1 << PL_ENC_CTL_PXL_EN;
			writel(val, hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			mdelay(17);

			val = readl(hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			val |=  1 << PL_ENC_CTL_PXL_VAL;
			writel(val, hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
		}
	} else {
		if (!--hdmirx->running) {
			val = readl(hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			val |=  ~(1 << PL_ENC_CTL_PXL_VAL);
			writel(val, hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			mdelay(17);

			val = readl(hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
			val |=  ~(1 << PL_ENC_CTL_PXL_EN);
			writel(val, hdmirx->regs_sec + CSR_PIXEL_LINK_ENC_CTL);
		}
		pm_runtime_put(dev);
	}

	return 0;
}

static const struct v4l2_subdev_video_ops cdns_video_ops_hdmi = {
	.s_stream = hdmirx_s_stream,
	.g_parm =	hdmirx_g_parm,
	.s_parm =	hdmirx_s_parm,
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
static int hdmirx_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);

	/* Check for hdmirx->timings correctness,
	 * as sometimes it is null, and a kernel panic arises.
	 * For the sake of precision, also hdmirx is checked,
	 * but no crash was found due to it being null */
	if (fse->index > 1 || hdmirx == NULL ||
			hdmirx->timings == NULL || hdmirx->cable_plugin == false)
		return -EINVAL;

	fse->min_width = hdmirx->timings->timings.bt.width;
	fse->max_width = fse->min_width;

	fse->min_height = hdmirx->timings->timings.bt.height;
	fse->max_height = fse->min_height;
	return 0;
}
static int hdmirx_enum_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);

	if (fie->index > 8)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	 /* TODO Reserved to extension */
	fie->interval.denominator = hdmirx->timings->fps;
	return 0;
}

static int hdmirx_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_RGB888_1X24;

	return 0;
}

static int hdmirx_get_format(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_format *sdformat)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	struct v4l2_mbus_framefmt *mbusformat = &sdformat->format;

	if (hdmirx->cable_plugin == false) {
		dev_warn(&hdmirx->pdev->dev, "No Cable Connected!\n");
		return -EINVAL;
	}

	if (sdformat->pad != MXC_HDMI_RX_PAD_SOURCE)
		return -EINVAL;

	switch (hdmirx->pixel_encoding) {
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

	mbusformat->width =  hdmirx->timings->timings.bt.width;
	mbusformat->height = hdmirx->timings->timings.bt.height;
	mbusformat->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int hdmirx_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (!hdmirx->edid.present)
		return -ENODATA;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = hdmirx->edid.blocks;
		return 0;
	}

	if (edid->start_block >= hdmirx->edid.blocks)
		return -EINVAL;

	if (edid->start_block + edid->blocks > hdmirx->edid.blocks)
		edid->blocks = hdmirx->edid.blocks - edid->start_block;

	memcpy(edid->edid, hdmirx->edid.edid + edid->start_block * 128,
			edid->blocks * 128);

	return 0;
}

static int hdmirx_set_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	int err, i;

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks == 0) {
		/* Default EDID */
		hdmirx->edid.blocks = 2;
		hdmirx->edid.present = true;
	} else if (edid->blocks > 4) {
		edid->blocks = 4;
		return -E2BIG;
	} else {
		memcpy(hdmirx->edid.edid, edid->edid, 128 * edid->blocks);
		hdmirx->edid.blocks = edid->blocks;
		hdmirx->edid.present = true;
	}

	for (i = 0; i < hdmirx->edid.blocks; i++) {
		/* EDID block */
		err = cdns_hdmirx_set_edid(hdmirx, 0, i, &hdmirx->edid.edid[i * 128]);
		if (err) {
			v4l2_err(sd, "error %d writing edid pad %d\n", err, edid->pad);
			return -err;
		}
	}

	return 0;
}

static bool check_dv_timings(const struct v4l2_dv_timings *timings,
					  void *hdl)
{
	const struct cdns_hdmirx_dev_video_standards *stds =
		cdns_hdmi_video_standards;
	u32 i;

	for (i = 0; stds[i].timings.bt.width; i++)
		if (v4l2_match_dv_timings(timings, &stds[i].timings, 0, false))
			return true;

	return false;
}

static int hdmirx_enum_dv_timings(struct v4l2_subdev *sd,
					struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &hdmirx_timings_cap,
					check_dv_timings, NULL);
}

static int hdmirx_dv_timings_cap(struct v4l2_subdev *sd,
				       struct v4l2_dv_timings_cap *cap)
{
	*cap = hdmirx_timings_cap;
	return 0;
}

static const struct v4l2_subdev_pad_ops cdns_pad_ops_hdmi = {
	.enum_mbus_code = hdmirx_enum_mbus_code,
	.enum_frame_size = hdmirx_enum_framesizes,
	.enum_frame_interval = hdmirx_enum_frame_interval,
	.get_fmt = hdmirx_get_format,
	.get_edid = hdmirx_get_edid,
	.set_edid = hdmirx_set_edid,
	.dv_timings_cap = hdmirx_dv_timings_cap,
	.enum_dv_timings = hdmirx_enum_dv_timings,
};

static int hdmirx_s_power(struct v4l2_subdev *sd, int on)
{
	struct cdns_hdmirx_device *hdmirx = cdns_sd_to_hdmi(sd);
	struct device *dev = &hdmirx->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	return 0;
}
static struct v4l2_subdev_core_ops cdns_core_ops_hdmi = {
	.s_power = hdmirx_s_power,
};

/* -----------------------------------------------------------------------------
 * v4l2_subdev_ops
 */
static const struct v4l2_subdev_ops cdns_ops_hdmi = {
	.core = &cdns_core_ops_hdmi,
	.video = &cdns_video_ops_hdmi,
	.pad = &cdns_pad_ops_hdmi,
};

void imx8qm_hdmi_phy_reset(struct cdns_hdmirx_device *hdmirx, u8 reset)
{
	struct imx_sc_ipc *handle;
	int ret;

	dev_dbg(&hdmirx->pdev->dev, "%s\n", __func__);

	imx_scu_get_handle(&handle);
	/* set the pixel link mode and pixel type */
	ret = imx_sc_misc_set_control(handle, IMX_SC_R_HDMI_RX, IMX_SC_C_PHY_RESET, reset);
	if (ret)
		DRM_ERROR("SC_R_HDMI PHY reset failed %d!\n", ret);
}

#ifdef CONFIG_MHDP_HDMIRX_CEC
static void hdmirx_cec_init(struct cdns_hdmirx_device *hdmirx)
{
	struct cdns_mhdp_cec *cec = &hdmirx->cec;

	cec->dev = &hdmirx->pdev->dev;
	cec->iolock = &hdmirx->iolock;
	cec->regs_base = hdmirx->regs_base;
	cec->regs_sec = hdmirx->regs_sec;
	cec->bus_type = hdmirx->bus_type;
}
#endif

static void dump_debug_regs(struct cdns_hdmirx_device *hdmirx)
{
	u16 dbgp = 0xFFFF;
	u16 dbg;
	u32 reg;

	dev_info(&hdmirx->pdev->dev, "Checking debug registers...\n");
	reg = cdns_hdmirx_bus_read(hdmirx, SW_DEBUG_L);
	dbg = (u8)reg;
	reg = cdns_hdmirx_bus_read(hdmirx, SW_DEBUG_H);
	dbg |= (u8)reg << 8;

	if (dbgp != dbg) {
		u32 addr;
		u32 r1, r2, r3, r4;

		dev_info(&hdmirx->pdev->dev, "=== DBG: 0x%.4X\n", dbgp = dbg);

		r1 = cdns_hdmirx_bus_read(hdmirx, VER_LIB_L_ADDR);
		r2 = cdns_hdmirx_bus_read(hdmirx, VER_LIB_H_ADDR);
		r3 = cdns_hdmirx_bus_read(hdmirx, VER_L);
		r4 = cdns_hdmirx_bus_read(hdmirx, VER_H);

		addr = r1 << 24 | r2 << 16 | r3 << 8 | r4;
		dev_info(&hdmirx->pdev->dev, "=== DBG: addr 0x%.8X\n", addr);
	}
}

static void alive(struct cdns_hdmirx_device *hdmirx)
{
	u32 ret;

	/* Check if FW is still alive */
	ret = cdns_hdmirx_check_alive(hdmirx);
	if (ret == false) {
		dev_err(&hdmirx->pdev->dev, "NO HDMI RX FW running\n");
		dev_err(&hdmirx->pdev->dev, "Checking debug registers...\n");
		dump_debug_regs(hdmirx);
		/* BUG(); */
	}
}

static int tmdsmon_fn(void *data)
{
	struct cdns_hdmirx_device *hdmirx = (struct cdns_hdmirx_device *)data;
	int tmds;
	u8 startup_attempts;
	u8 avi_fail_cnt;
	u16 tmds_measure_cnt;
	u8 bad_tmds_cnt;

	dev_dbg(&hdmirx->pdev->dev, "= starting tmdsmon thread\n");

	startup_attempts = 0;
	avi_fail_cnt = 0;
	tmds_measure_cnt = 0;
	bad_tmds_cnt = 0;

	set_freezable();

	while (1) {
		bool change = false;

		cpu_relax();
		msleep(200);

		if (kthread_freezable_should_stop(NULL))
			break;

		if (hdmirx->tmdsmon_state != 1) {
			if (hdmirx->tmdsmon_state == 2)
				dev_dbg(&hdmirx->pdev->dev, "= stopping tmdsmon thread\n");
			hdmirx->tmdsmon_state = 0;
			startup_attempts = 0;
			avi_fail_cnt = 0;
			continue;
		}

		alive(hdmirx);

		if (phy_in_reset(hdmirx)) {
			if (cdns_hdmirx_phyinit(hdmirx))
				continue;
		}

		tmds = -1;
		if (pma_rx_clk_sig_detected(hdmirx)) {
			tmds = cdns_hdmirx_get_stable_tmds(hdmirx);
			if (tmds < 0) {
				bad_tmds_cnt++;
				if (bad_tmds_cnt > 50) {
					bad_tmds_cnt = 0;
					dev_dbg(&hdmirx->pdev->dev, "Re-initialising PHY as no valid clock from source");
					cdns_hdmirx_phyinit(hdmirx);
					continue;
				}
			}
		} else
			bad_tmds_cnt = 0;

		if (tmds < 0) {
			if (hdmirx->initialized)
				dev_info(&hdmirx->pdev->dev, "Lost TMDS clock\n\n");
			hdmirx->cable_plugin = false;
			hdmirx->initialized = false;
			hdmirx->tmds_clk = -1;
			startup_attempts = 0;
			avi_fail_cnt = 0;
			tmds_measure_cnt = 0;
			continue;
		}

		tmds_measure_cnt++;
		if (tmds_measure_cnt == 500) {
			tmds_measure_cnt = 0;
			dev_info(&hdmirx->pdev->dev, "Measured TMDS is %d\n\n", tmds);
		}

		if (tmds > (hdmirx->tmds_clk + 50) ||
		    tmds < (hdmirx->tmds_clk - 50)) {
			dev_info(&hdmirx->pdev->dev, "TMDS change detect: %d -> %d\n\n",
				hdmirx->tmds_clk, tmds);
			change = true;
		} else if ((tmds > 0) && (hdmirx->initialized)) {
			u32 val;
			hdmirx->tmds_clk = tmds;

			val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);
			if (val & 1) {
				/* Only if detected as HDMI mode */
				int avi_change;

				/* Try to get an AVI infoframe */
				avi_change = cdns_hdmirx_get_avi_infoframe(hdmirx, 200);

				/* First check to see if we are already authenticated in
				 * HDCP mode, if so we need to request re-authentication
				 * if there are too many errors...
				 * However if get_avi_infoframe was successful then skip
				 * as all good... */
				if ((avi_change < 0) && hdmirx->hdcp_fw_loaded) {
					struct hdcprx_status status;
					u8 key_arrived, hdcp_ver;

					cdns_hdcprx_get_status(hdmirx, &status);
					key_arrived = status.flags & 1;
					hdcp_ver = (status.flags >> 1) & 0x3,

					val = cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);
					/* Already authenticated */
					if (key_arrived) {
						if (val & 0xff) {
							dev_dbg(&hdmirx->pdev->dev, "Got PKT_ERR_CNT_HEADER 0x%08X\n", val);
							dev_info(&hdmirx->pdev->dev, "Requesting HDCP re-authentication\n");
							cdns_hdcprx_reauth_req_wait(hdmirx, 2000);
						}
					} else if (hdcp_ver > 0)
						/* Possibly authenticating */
						cdns_hdcprx_wait_auth_complete(hdmirx, 2000);
				}

				/* Clear out status register */
				val = cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);
				if (avi_change < 0) {
					avi_fail_cnt++;

					if (avi_fail_cnt >= 3) {
						uint32_t events;

						dev_info(&hdmirx->pdev->dev, "Failed to get AVI infoframe after %d attempts\n", avi_fail_cnt);
						dev_dbg(&hdmirx->pdev->dev, "Triggering re-init via HPD\n");
						events = cdns_hdmirx_bus_read(hdmirx, SW_EVENTS1);
						cdns_hdmirx_hotplug_trigger(hdmirx);
						cdns_hdmirx_wait_edid_read(hdmirx);
						change = true;
						avi_fail_cnt = 0;
					}
				} else {
					avi_fail_cnt = 0;
					if (avi_change == 1) {
						dev_info(&hdmirx->pdev->dev, "\nAVI change detect\n\n");
						change = true;
						if (hdmirx->vic_code == 0) {
							int vnd_change = cdns_hdmirx_get_vendor_infoframe(hdmirx, 250);
							if (vnd_change == 1) {
								dev_info(&hdmirx->pdev->dev, "\nVendor info change detect\n\n");
								cdns_hdmirx_frame_timing(hdmirx);
							}
						}
					}
				}
			} else
				/* DVI mode */
				avi_fail_cnt = 0;
		}

		/* Check in case about to shut down... */
		if (hdmirx->tmdsmon_state != 1)
			continue;

		if (change) {
			if (hdmirx->initialized) {
				if (hdmirx->hdcp_fw_loaded && !hdmirx->allow_hdcp)
					cdns_hdcprx_disable(hdmirx);
				/* We may not have a clock anymore at this point
				   so clear it later...
				hdmirx_phy_pix_engine_reset(hdmirx);
				hdmirx_stop(hdmirx);
				*/
			}
			hdmirx->initialized = false;
			hdmirx->cable_plugin = false;

			if (!hdmirx->allow_hdcp && hdmirx->hdcp_fw_loaded)
				cdns_hdmirx_general_unloadhdcprx(hdmirx);

			cdns_hdmirx_maincontrol(hdmirx, VIDEO_MODE_CHANGE);
			dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(0) with VIDEO_MODE_CHANGE\n",  __func__);

			cdns_hdmirx_maincontrol(hdmirx, 1);
			dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(1)\n", __func__);

			if (hdmirx->allow_hdcp && !hdmirx->hdcp_fw_loaded) {
				cdns_hdmirx_general_loadhdcprx(hdmirx);
				cdns_hdcprx_enable(hdmirx);
			}
			hdmirx->hdcp_fw_loaded = hdmirx->allow_hdcp;

			dev_dbg(&hdmirx->pdev->dev, "= TMDS Mon calling cdns_hdmirx_startup after seeing TMDS of %d\n", tmds);
			if (cdns_hdmirx_startup(hdmirx) < 0) {
				hdmirx->tmds_clk = -1;
				if (startup_attempts > 1) {
					dev_dbg(&hdmirx->pdev->dev, "= TMDS Mon re-initialising PHY due to %d failed startup attempts\n", startup_attempts);
					cdns_hdmirx_phyinit(hdmirx);
					startup_attempts = 0;
				} else
					startup_attempts++;
				continue;
			}
#ifdef CONFIG_MHDP_HDMIRX_CEC
			if (hdmirx->is_cec && !hdmirx->cec_running) {
				hdmirx_cec_init(hdmirx);
				cdns_mhdp_register_cec_driver(&hdmirx->cec);
				hdmirx->cec_running = true;
			}
#endif
			startup_attempts = 0;
			hdmirx->initialized = true;
			hdmirx->cable_plugin = true;
		} else {
			if (hdmirx->initialized) {

				if (!hdmirx->allow_hdcp && hdmirx->hdcp_fw_loaded) {
					cdns_hdcprx_disable(hdmirx);
					cdns_hdmirx_general_unloadhdcprx(hdmirx);
				}
				if (hdmirx->allow_hdcp && !hdmirx->hdcp_fw_loaded) {
					cdns_hdmirx_general_loadhdcprx(hdmirx);
					cdns_hdcprx_enable(hdmirx);
				}
				hdmirx->hdcp_fw_loaded = hdmirx->allow_hdcp;
			}
			startup_attempts = 0;
		}
	}

	dev_info(&hdmirx->pdev->dev, "= exiting tmdsmon thread\n");

	/* Avoid compiler warning */
	return 0;
}

static int tmdsmon_init(struct cdns_hdmirx_device *hdmirx)
{
	static const char name[] = "tmdsmon_th";

	if (hdmirx->tmdsmon_th) {
		dev_dbg(&hdmirx->pdev->dev, "Resuming thread for monitoring TMDS changes\n");
		hdmirx->tmdsmon_state = 1;
		return 0;
	}

	dev_dbg(&hdmirx->pdev->dev, "Creating thread for monitoring TMDS changes\n");

	hdmirx->tmdsmon_th = kthread_create(tmdsmon_fn, hdmirx, name);
	if (!IS_ERR(hdmirx->tmdsmon_th)) {
		hdmirx->tmdsmon_state = 1;
		wake_up_process(hdmirx->tmdsmon_th);
	} else {
		dev_err(&hdmirx->pdev->dev,
			"Failed to spawn TMDS monitor thread\n");
		return -1;
	}

	return 0;
}

static void tmdsmon_cleanup(struct cdns_hdmirx_device *hdmirx)
{
	dev_dbg(&hdmirx->pdev->dev, "= stopping tmds mon\n");
	hdmirx->tmdsmon_state = 2;
	while (hdmirx->tmdsmon_state == 2)
		schedule();
	dev_dbg(&hdmirx->pdev->dev, "= stopped tmds mon\n");
}

int cdns_hdmirx_wait_edid_read(struct cdns_hdmirx_device *hdmirx)
{
	uint32_t events;

	ktime_t timeout = ktime_timeout_ms(2000);
	dev_info(&hdmirx->pdev->dev, "hdmirx_wait_edid_read wait for EDID to be read\n");
	do {
		events = cdns_hdmirx_bus_read(hdmirx, SW_EVENTS1);
		if (ktime_after(ktime_get(), timeout))
			goto timeout_err;
		msleep(1);
	} while ((events & 0x1) == 0);
	dev_dbg(&hdmirx->pdev->dev, "hdmirx_wait_edid_read detected, now wait for status to be 0");

	timeout = ktime_timeout_ms(500);
	do {
		msleep(50);
		events = cdns_hdmirx_bus_read(hdmirx, SW_EVENTS1);
		if (ktime_after(ktime_get(), timeout))
			goto timeout_err;
	} while ((events & 0x1) == 1);

	dev_dbg(&hdmirx->pdev->dev, "hdmirx_wait_edid_read Finished successfully, final status 0x%08X\n", events);
	return 0;

timeout_err:
	dev_dbg(&hdmirx->pdev->dev, "hdmirx_wait_edid_read timed out waiting for EDID to be read\n");
	return -1;
}

static void hpd5v_work_func(struct work_struct *work)
{
	struct cdns_hdmirx_device *hdmirx = container_of(work,
						       struct cdns_hdmirx_device,
						       hpd5v_work.work);
	char event_string[32];
	char *envp[] = { event_string, NULL };
	u8 hpd = ~0;

	dev_dbg(&hdmirx->pdev->dev, "%s hello\n", __func__);

	/* Check cable hdmirxs before enable irq */
	hpd = cdns_hdmirx_read_hpd(hdmirx);
	if (hdmirx->last_5v_state == hpd)
		return;

	hdmirx->last_5v_state = hpd;
	if (hpd == 1) {
		dev_info(&hdmirx->pdev->dev, "HDMI RX Cable Plug In\n");

		dev_dbg(&hdmirx->pdev->dev, "hpd5v_work_func calling cdns_hdmirx_phyinit\n");
		if (cdns_hdmirx_phyinit(hdmirx))
			dev_err(&hdmirx->pdev->dev, "Failed to init PHY, try replugging.\n");
		if (hdmirx->allow_hdcp) {
			cdns_hdmirx_general_loadhdcprx(hdmirx);
			cdns_hdcprx_enable(hdmirx);
		}
		hdmirx->initialized = 0;
		hdmirx->tmds_clk = -1;

		sprintf(event_string, "EVENT=hdmirxin");
		kobject_uevent_env(&hdmirx->pdev->dev.kobj, KOBJ_CHANGE, envp);
		hdmirx->cable_plugin = true;
#ifdef CONFIG_MHDP_HDMIRX_CEC
		if (hdmirx->is_cec) {
			hdmirx_cec_init(hdmirx);
			cdns_mhdp_register_cec_driver(&hdmirx->cec);
			hdmirx->cec_running = true;
		}
#endif
		tmdsmon_init(hdmirx);
		enable_irq(hdmirx->irq[HPD5V_IRQ_OUT]);
	} else if (hpd == 0) {
		tmdsmon_cleanup(hdmirx);
		cdns_hdmirx_sethpd(hdmirx, 0);
		if (hdmirx->initialized) {
			/* We may not have a clock anymore at this point
			   so clear it later...
			hdmirx_phy_pix_engine_reset(hdmirx);
			hdmirx_stop(hdmirx);
			*/
		}
		hdmirx->initialized = false;
#ifdef CONFIG_MHDP_HDMIRX_CEC
		if (hdmirx->is_cec && hdmirx->cec_running) {
			cdns_mhdp_unregister_cec_driver(&hdmirx->cec);
			hdmirx->cec_running = false;
		}
#endif
		sprintf(event_string, "EVENT=hdmirxout");
		kobject_uevent_env(&hdmirx->pdev->dev.kobj, KOBJ_CHANGE, envp);
		hdmirx->cable_plugin = false;

		cdns_hdmirx_maincontrol(hdmirx, 0);
		dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl_blocking(0x0)\n", __func__);
		imx8qm_hdmi_phy_reset(hdmirx, 0);
		cdns_hdmirx_general_assertphyreset(hdmirx);

		enable_irq(hdmirx->irq[HPD5V_IRQ_IN]);
	} else
		dev_warn(&hdmirx->pdev->dev, "HDMI RX Cable State unknow\n");
}

#define HOTPLUG_DEBOUNCE_MS		200
static irqreturn_t hdp5v_irq_thread(int irq, void *data)
{
	struct cdns_hdmirx_device *hdmirx =  data;

	dev_dbg(&hdmirx->pdev->dev, "%s hello\n", __func__);

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &hdmirx->hpd5v_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void print_fw_ver(struct cdns_hdmirx_device *hdmirx)
{
	u8 r1, r2, r3, r4;

	r1 = cdns_hdmirx_bus_read(hdmirx, VER_LIB_L_ADDR);
	r2 = cdns_hdmirx_bus_read(hdmirx, VER_LIB_H_ADDR);
	r3 = cdns_hdmirx_bus_read(hdmirx, VER_L);
	r4 = cdns_hdmirx_bus_read(hdmirx, VER_H);
    printk("HDMI RX FIRMWARE VERSION: %d, LIB VERSION: %d\n", (r4 << 8) | r3, (r2 << 8) | r1);
}

static int hdmirx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cdns_hdmirx_device *hdmirx;
	struct resource *res;
	u8 hpd;
	int ret = 0;

	dev_dbg(dev, "%s\n", __func__);
	hdmirx = devm_kzalloc(dev, sizeof(*hdmirx), GFP_KERNEL);
	if (!hdmirx)
		return -ENOMEM;

	hdmirx->pdev = pdev;
	mutex_init(&hdmirx->iolock);
	mutex_init(&hdmirx->pif_mutex);

	/* register map */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		hdmirx->regs_base = devm_ioremap(dev, res->start, resource_size(res));
		if (IS_ERR(hdmirx->regs_base)) {
			dev_err(dev, "Failed to get HDMI RX CTRL base register\n");
			return -EINVAL;
		}
	} else
		dev_err(dev, "Error Not get memory resource");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		hdmirx->regs_sec = devm_ioremap(dev, res->start, resource_size(res));
		if (IS_ERR(hdmirx->regs_sec)) {
			dev_err(dev, "Failed to get HDMI RX CRS base register\n");
			return -EINVAL;
		}
	} else
		dev_err(dev, "Error Not get memory resource");

	hdmirx->irq[HPD5V_IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (hdmirx->irq[HPD5V_IRQ_IN] < 0)
		dev_info(&pdev->dev, "No plug_in irq number\n");

	hdmirx->irq[HPD5V_IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (hdmirx->irq[HPD5V_IRQ_OUT] < 0)
		dev_info(&pdev->dev, "No plug_out irq number\n");

	INIT_DELAYED_WORK(&hdmirx->hpd5v_work, hpd5v_work_func);

	v4l2_subdev_init(&hdmirx->sd, &cdns_ops_hdmi);
	/* sd.dev may use by match_of */
	hdmirx->sd.dev = dev;

	/* the owner is the same as the i2c_client's driver owner */
	hdmirx->sd.owner = THIS_MODULE;
	hdmirx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	hdmirx->sd.entity.function = MEDIA_ENT_F_IO_DTV;

	/* This allows to retrieve the platform device id by the host driver */
	v4l2_set_subdevdata(&hdmirx->sd, pdev);

	/* initialize name */
	snprintf(hdmirx->sd.name, sizeof(hdmirx->sd.name), "%s", HDMIRX_SUBDEV_NAME);

	hdmirx->pads[MXC_HDMI_RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	hdmirx->pads[MXC_HDMI_RX_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	hdmirx->sd.entity.ops = &hdmi_media_ops;
	ret = media_entity_pads_init(&hdmirx->sd.entity,
				     MXC_HDMI_RX_PADS_NUM, hdmirx->pads);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, hdmirx);
	ret = v4l2_async_register_subdev(&hdmirx->sd);
	if (ret < 0) {
		dev_err(dev,
					"%s--Async register failed, ret=%d\n", __func__, ret);
		media_entity_cleanup(&hdmirx->sd.entity);
	}

	hdmirx->is_cec = of_property_read_bool(pdev->dev.of_node, "fsl,cec");

	of_property_read_string(pdev->dev.of_node, "firmware-name",
					&hdmirx->firmware_name);

	hdmirx_clock_init(hdmirx);

	hdmirx->bus_type = BUS_TYPE_LOW4K_HDMI_RX;

	hdmirx->flags = HDMIRX_PM_POWERED;

	hdmirx->allow_hdcp = true;
	hdmirx->hdcp_fw_loaded = false;
	hdmirx->rescal_val = 0;
	hdmirx->slicer_tune_val = 0;
	hdmirx->running = 0;

	hdmirx_clock_enable(hdmirx);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	hdmirx->tmdsmon_th = NULL;
	hdmirx->tmdsmon_state = 0;

	ret = cdns_hdmirx_init(hdmirx);
	if (ret < 0) {
		dev_err(dev, "mxc hdmi rx init failed\n");
		goto failed;
	}
	hdmi_sysfs_init(dev);
	cdns_hdcprx_enable(hdmirx);

	/* Check cable states before enable irq */
	hpd = cdns_hdmirx_read_hpd(hdmirx);
	hdmirx->last_5v_state = hpd;

	/* Enable Hotplug Detect IRQ thread */
	if (hdmirx->irq[HPD5V_IRQ_IN] > 0) {
		irq_set_status_flags(hdmirx->irq[HPD5V_IRQ_IN], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdmirx->irq[HPD5V_IRQ_IN],
						NULL, hdp5v_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdmirx);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
							hdmirx->irq[HPD5V_IRQ_IN]);
			goto failed;
		}
		/* Cable Disconnedted, enable Plug in IRQ */
		if (hpd == 0) {
			cdns_hdmirx_sethpd(hdmirx, 0);
			hdmirx->cable_plugin = false;
			cdns_hdmirx_maincontrol(hdmirx, 0);
			dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl_blocking(0x0)\n", __func__);
			imx8qm_hdmi_phy_reset(hdmirx, 0);
			cdns_hdmirx_general_assertphyreset(hdmirx);
			enable_irq(hdmirx->irq[HPD5V_IRQ_IN]);
		}
	}

	if (hdmirx->irq[HPD5V_IRQ_OUT] > 0) {
		irq_set_status_flags(hdmirx->irq[HPD5V_IRQ_OUT], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdmirx->irq[HPD5V_IRQ_OUT],
						NULL, hdp5v_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdmirx);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
				hdmirx->irq[HPD5V_IRQ_OUT]);
			goto failed;
		}
		if (hpd == 1) {
			dev_dbg(&hdmirx->pdev->dev, "hdmirx_probe calling cdns_hdmirx_phyinit\n");
			if (cdns_hdmirx_phyinit(hdmirx))
				dev_err(&hdmirx->pdev->dev, "Failed to init PHY, try replugging.\n");
			if (hdmirx->allow_hdcp) {
				cdns_hdmirx_general_loadhdcprx(hdmirx);
				cdns_hdcprx_enable(hdmirx);
			}
			hdmirx->hdcp_fw_loaded = hdmirx->allow_hdcp;
			hdmirx->initialized = 0;
			hdmirx->tmds_clk = -1;
			hdmirx->cable_plugin = true;
#ifdef CONFIG_MHDP_HDMIRX_CEC
			if (hdmirx->is_cec) {
				hdmirx_cec_init(hdmirx);
				cdns_mhdp_register_cec_driver(&hdmirx->cec);
				hdmirx->cec_running = true;
			}
#endif
			tmdsmon_init(hdmirx);
			/* Cable Connected, enable Plug out IRQ */
			enable_irq(hdmirx->irq[HPD5V_IRQ_OUT]);
		}
	}

	cdns_hdmirx_register_audio_driver(dev);
	print_fw_ver(hdmirx);
	dev_dbg(dev, "iMX8 HDMI RX probe successfully\n");

	return ret;
failed:
	v4l2_async_unregister_subdev(&hdmirx->sd);
	media_entity_cleanup(&hdmirx->sd.entity);

	hdmirx_clock_disable(hdmirx);
	pm_runtime_disable(dev);
	dev_warn(dev, "mxc hdmi rx probe failed\n");
	return ret;
}

static int hdmirx_remove(struct platform_device *pdev)
{
	struct cdns_hdmirx_device *hdmirx = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	tmdsmon_cleanup(hdmirx);
	v4l2_async_unregister_subdev(&hdmirx->sd);
	media_entity_cleanup(&hdmirx->sd.entity);

#ifdef CONFIG_MHDP_HDMIRX_CEC
	if (hdmirx->is_cec)
		cdns_mhdp_unregister_cec_driver(&hdmirx->cec);
#endif

	cdns_hdmirx_sethpd(hdmirx, 0);

	cdns_hdmirx_maincontrol(hdmirx, 0x0);
	dev_dbg(dev, "%s(): called MainControl_blocking(0x0)\n", __func__);

	hdmirx_clock_disable(hdmirx);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hdmirx_pm_suspend(struct device *dev)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);
	if (hdmirx->running > 0) {
		dev_warn(dev, "running, prevent entering suspend.\n");
		return -EAGAIN;
	}

	if ((hdmirx->flags & HDMIRX_PM_SUSPENDED) ||
		(hdmirx->flags & HDMIRX_RUNTIME_SUSPEND))
		return 0;

	hdmirx_clock_disable(hdmirx);
	hdmirx->flags |= HDMIRX_PM_SUSPENDED;
	hdmirx->flags &= ~HDMIRX_PM_POWERED;

	return 0;
}

static int hdmirx_pm_resume(struct device *dev)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);
	int ret;

	if (hdmirx->flags & HDMIRX_PM_POWERED)
		return 0;

	hdmirx->flags |= HDMIRX_PM_POWERED;
	hdmirx->flags &= ~HDMIRX_PM_SUSPENDED;

	hdmirx_clock_enable(hdmirx);

	/* reset hdmi rx status flags */
	hdmirx->initialized = 0;
	hdmirx->tmds_clk = -1;
	hdmirx->hdcp_fw_loaded = false;

	ret = cdns_hdmirx_init(hdmirx);
	if (ret < 0)
		return -EAGAIN;

	ret = cdns_hdmirx_phyinit(hdmirx);
	return (ret) ? -EAGAIN : 0;
}
#endif

static int hdmirx_runtime_suspend(struct device *dev)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);

	if (hdmirx->flags & HDMIRX_RUNTIME_SUSPEND)
		return 0;

	if (hdmirx->flags & HDMIRX_PM_POWERED) {
		hdmirx_clock_disable(hdmirx);
		hdmirx->flags |= HDMIRX_RUNTIME_SUSPEND;
		hdmirx->flags &= ~HDMIRX_PM_POWERED;
	}
	return 0;
}

static int hdmirx_runtime_resume(struct device *dev)
{
	struct cdns_hdmirx_device *hdmirx = dev_get_drvdata(dev);

	if (hdmirx->flags & HDMIRX_PM_POWERED)
		return 0;

	if (hdmirx->flags & HDMIRX_RUNTIME_SUSPEND) {
		hdmirx_clock_enable(hdmirx);
		hdmirx->flags |= HDMIRX_PM_POWERED;
		hdmirx->flags &= ~HDMIRX_RUNTIME_SUSPEND;
	}
	return 0;
}

static const struct dev_pm_ops hdmirx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hdmirx_pm_suspend, hdmirx_pm_resume)
	SET_RUNTIME_PM_OPS(hdmirx_runtime_suspend, hdmirx_runtime_resume, NULL)
};

static const struct of_device_id hdmirx_of_match[] = {
	{.compatible = "cdn,imx8qm-hdmirx",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hdmirx_of_match);

static struct platform_driver hdmirx_driver = {
	.probe		= hdmirx_probe,
	.remove		= hdmirx_remove,
	.driver = {
		.of_match_table = hdmirx_of_match,
		.name		= HDMIRX_DRIVER_NAME,
		.pm		= &hdmirx_pm_ops,
	}
};

module_platform_driver(hdmirx_driver);
