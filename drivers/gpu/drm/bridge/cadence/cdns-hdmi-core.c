/*
 * Cadence High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright 2019-2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_hdcp.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_scdc_helper.h>
#include <drm/drm_vblank.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of_device.h>

#include "cdns-mhdp-hdcp.h"
#include "cdns-hdcp-common.h"

static void hdmi_sink_config(struct cdns_mhdp_device *mhdp)
{
	struct drm_scdc *scdc = &mhdp->connector.base.display_info.hdmi.scdc;
	u8 buff = 0;

	/* return if hdmi work in DVI mode */
	if (mhdp->hdmi.hdmi_type == MODE_DVI)
		return;

	/* check sink support SCDC or not */
	if (scdc->supported != true) {
		DRM_INFO("Sink Not Support SCDC\n");
		return;
	}

	if (mhdp->hdmi.char_rate > 340000) {
		/*
		 * TMDS Character Rate above 340MHz should working in HDMI2.0
		 * Enable scrambling and TMDS_Bit_Clock_Ratio
		 */
		buff = SCDC_TMDS_BIT_CLOCK_RATIO_BY_40 | SCDC_SCRAMBLING_ENABLE;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	} else  if (scdc->scrambling.low_rates) {
		/*
		 * Enable scrambling and HDMI2.0 when scrambling capability of sink
		 * be indicated in the HF-VSDB LTE_340Mcsc_scramble bit
		 */
		buff = SCDC_SCRAMBLING_ENABLE;
		mhdp->hdmi.hdmi_type = MODE_HDMI_2_0;
	}

	/* TMDS config */
	cdns_hdmi_scdc_write(mhdp, 0x20, buff);
}

static void hdmi_lanes_config(struct cdns_mhdp_device *mhdp)
{
	/* Line swaping */
	cdns_mhdp_reg_write(mhdp, LANES_CONFIG, 0x00400000 | mhdp->lane_mapping);
}

static int hdmi_avi_info_set(struct cdns_mhdp_device *mhdp,
			     struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	int format = mhdp->video_info.color_fmt;
	struct drm_connector_state *conn_state = mhdp->connector.base.state;
	struct drm_display_mode *adj_mode;
	enum hdmi_quantization_range qr;
	u8 buf[32];
	int ret;

	/* Initialise info frame from DRM mode */
	drm_hdmi_avi_infoframe_from_display_mode(&frame, &mhdp->connector.base,
						 mode);

	switch (format) {
	case YCBCR_4_4_4:
		frame.colorspace = HDMI_COLORSPACE_YUV444;
		break;
	case YCBCR_4_2_2:
		frame.colorspace = HDMI_COLORSPACE_YUV422;
		break;
	case YCBCR_4_2_0:
		frame.colorspace = HDMI_COLORSPACE_YUV420;
		break;
	default:
		frame.colorspace = HDMI_COLORSPACE_RGB;
		break;
	}

	drm_hdmi_avi_infoframe_colorspace(&frame, conn_state);

	adj_mode = &mhdp->bridge.base.encoder->crtc->state->adjusted_mode;

	qr = drm_default_rgb_quant_range(adj_mode);

	drm_hdmi_avi_infoframe_quant_range(&frame, &mhdp->connector.base,
					   adj_mode, qr);

	ret = hdmi_avi_infoframe_check(&frame);
	if (WARN_ON(ret))
		return false;

	ret = hdmi_avi_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 0, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_AVI);
	return 0;
}

static void hdmi_vendor_info_set(struct cdns_mhdp_device *mhdp,
				struct drm_display_mode *mode)
{
	struct hdmi_vendor_infoframe frame;
	u8 buf[32];
	int ret;

	/* Initialise vendor frame from DRM mode */
	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, &mhdp->connector.base, mode);
	if (ret < 0) {
		DRM_INFO("No vendor infoframe\n");
		return;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_WARN("Unable to pack vendor infoframe: %d\n", ret);
		return;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 3, sizeof(buf), buf, HDMI_INFOFRAME_TYPE_VENDOR);
}

static void hdmi_drm_info_set(struct cdns_mhdp_device *mhdp)
{
	struct drm_connector_state *conn_state;
	struct hdmi_drm_infoframe frame;
	u8 buf[32];
	int ret;

	conn_state = mhdp->connector.base.state;

	if (!conn_state->hdr_output_metadata)
		return;

	ret = drm_hdmi_infoframe_set_hdr_metadata(&frame, conn_state);
	if (ret < 0) {
		DRM_DEBUG_KMS("couldn't set HDR metadata in infoframe\n");
		return;
	}

	ret = hdmi_drm_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_DEBUG_KMS("couldn't pack HDR infoframe\n");
		return;
	}

	buf[0] = 0;
	cdns_mhdp_infoframe_set(mhdp, 3, sizeof(buf),
				buf, HDMI_INFOFRAME_TYPE_DRM);
}

void cdns_hdmi_mode_set(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	int ret;

	/* video mode valid check */
	if (mode->clock == 0 || mode->hdisplay == 0 ||  mode->vdisplay == 0)
		return;

	hdmi_lanes_config(mhdp);

	cdns_mhdp_plat_call(mhdp, pclk_rate);

	/* delay for HDMI FW stable after pixel clock relock */
	msleep(20);

	cdns_mhdp_plat_call(mhdp, phy_set);

	hdmi_sink_config(mhdp);

	ret = cdns_hdmi_ctrl_init(mhdp, mhdp->hdmi.hdmi_type, mhdp->hdmi.char_rate);
	if (ret < 0) {
		DRM_ERROR("%s, ret = %d\n", __func__, ret);
		return;
	}

	/* Config GCP */
	if (mhdp->video_info.color_depth == 8)
		cdns_hdmi_disable_gcp(mhdp);
	else
		cdns_hdmi_enable_gcp(mhdp);

	ret = hdmi_avi_info_set(mhdp, mode);
	if (ret < 0) {
		DRM_ERROR("%s ret = %d\n", __func__, ret);
		return;
	}

	/* vendor info frame is enable only  when HDMI1.4 4K mode */
	hdmi_vendor_info_set(mhdp, mode);

	hdmi_drm_info_set(mhdp);

	ret = cdns_hdmi_mode_config(mhdp, mode, &mhdp->video_info);
	if (ret < 0) {
		DRM_ERROR("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}
}

static void handle_plugged_change(struct cdns_mhdp_device *mhdp, bool plugged)
{
	if (mhdp->plugged_cb && mhdp->codec_dev)
		mhdp->plugged_cb(mhdp->codec_dev, plugged);
}

int cdns_hdmi_set_plugged_cb(struct cdns_mhdp_device *mhdp,
			     hdmi_codec_plugged_cb fn,
			     struct device *codec_dev)
{
	bool plugged;

	mutex_lock(&mhdp->lock);
	mhdp->plugged_cb = fn;
	mhdp->codec_dev = codec_dev;
	plugged = mhdp->last_connector_result == connector_status_connected;
	handle_plugged_change(mhdp, plugged);
	mutex_unlock(&mhdp->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_set_plugged_cb);

static enum drm_connector_status
cdns_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct cdns_mhdp_device *mhdp =
				container_of(connector, struct cdns_mhdp_device, connector.base);
	enum drm_connector_status result;

	u8 hpd = 0xf;

	hpd = cdns_mhdp_read_hpd(mhdp);

	if (hpd == 1)
		/* Cable Connected */
		result = connector_status_connected;
	else if (hpd == 0)
		/* Cable Disconnedted */
		result = connector_status_disconnected;
	else {
		/* Cable status unknown */
		DRM_INFO("Unknow cable status, hdp=%u\n", hpd);
		result = connector_status_unknown;
	}

	mutex_lock(&mhdp->lock);
	if (result != mhdp->last_connector_result) {
		handle_plugged_change(mhdp,
				      result == connector_status_connected);
		mhdp->last_connector_result = result;
	}
	mutex_unlock(&mhdp->lock);

	return result;
}

static int cdns_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct cdns_mhdp_device *mhdp =
				container_of(connector, struct cdns_mhdp_device, connector.base);
	int num_modes = 0;
	struct edid *edid;

	edid = drm_do_get_edid(&mhdp->connector.base,
				   cdns_hdmi_get_edid_block, mhdp);
	if (edid) {
		dev_info(mhdp->dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
			 edid->header[0], edid->header[1],
			 edid->header[2], edid->header[3],
			 edid->header[4], edid->header[5],
			 edid->header[6], edid->header[7]);
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		mhdp->hdmi.hdmi_type = drm_detect_hdmi_monitor(edid) ?
						MODE_HDMI_1_4 : MODE_DVI;
		kfree(edid);
	}

	if (num_modes == 0)
		DRM_ERROR("Invalid edid\n");
	return num_modes;
}

static bool blob_equal(const struct drm_property_blob *a,
		       const struct drm_property_blob *b)
{
	if (a && b)
		return a->length == b->length &&
			!memcmp(a->data, b->data, a->length);

	return !a == !b;
}

static void cdns_hdmi_bridge_disable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	cdns_hdcp_disable(mhdp);
}

static void cdns_hdmi_bridge_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_connector_state *conn_state = mhdp->connector.base.state;

	if (conn_state->content_protection == DRM_MODE_CONTENT_PROTECTION_DESIRED)
		cdns_hdcp_enable(mhdp);
}

static int cdns_hdmi_connector_atomic_check(struct drm_connector *connector,
					    struct drm_atomic_state *state)
{
	struct drm_connector_state *new_con_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_connector_state *old_con_state =
		drm_atomic_get_old_connector_state(state, connector);
	struct drm_crtc *crtc = new_con_state->crtc;
	struct drm_crtc_state *new_crtc_state;
	struct cdns_mhdp_device *mhdp =
		container_of(connector, struct cdns_mhdp_device, connector.base);

	cdns_hdcp_atomic_check(connector, old_con_state, new_con_state);
	if (!new_con_state->crtc)
		return 0;

	new_crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(new_crtc_state))
		return PTR_ERR(new_crtc_state);

	if (!blob_equal(new_con_state->hdr_output_metadata,
			old_con_state->hdr_output_metadata) ||
	    new_con_state->colorspace != old_con_state->colorspace) {

		new_crtc_state->mode_changed =
			!new_con_state->hdr_output_metadata ||
			!old_con_state->hdr_output_metadata ||
			new_con_state->colorspace != old_con_state->colorspace;
		/* save new connector state */
		memcpy(&mhdp->connector.new_state, new_con_state, sizeof(struct drm_connector_state));
	}

	/*
	 * These properties are handled by fastset, and might not end up in a
	 * modeset.
	 */
	if (new_con_state->picture_aspect_ratio !=
	    old_con_state->picture_aspect_ratio ||
	    new_con_state->content_type != old_con_state->content_type ||
	    new_con_state->scaling_mode != old_con_state->scaling_mode)
		new_crtc_state->mode_changed = true;
	return 0;
}

static const struct drm_connector_funcs cdns_hdmi_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = cdns_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs cdns_hdmi_connector_helper_funcs = {
	.get_modes = cdns_hdmi_connector_get_modes,
	.atomic_check = cdns_hdmi_connector_atomic_check,
};

static int cdns_hdmi_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_mode_config *config = &bridge->dev->mode_config;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &mhdp->connector.base;
	int ret;

	connector->interlace_allowed = 1;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	if (!strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11))
		connector->ycbcr_420_allowed = true;

	drm_connector_helper_add(connector, &cdns_hdmi_connector_helper_funcs);

	ret = drm_connector_init(bridge->dev, connector, &cdns_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize connector\n");
		return ret;
	}

	if (!strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11)) {
		drm_object_attach_property(&connector->base,
					   config->hdr_output_metadata_property,
					   0);

		if (!drm_mode_create_hdmi_colorspace_property(connector))
			drm_object_attach_property(&connector->base,
						connector->colorspace_property,
						0);
	}

	drm_connector_attach_encoder(connector, encoder);

	drm_connector_attach_content_protection_property(connector, true);
	return 0;
}

static enum drm_mode_status
cdns_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
			    const struct drm_display_info *info,
			  const struct drm_display_mode *mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	enum drm_mode_status mode_status = MODE_OK;
	u32 vic;
	int ret;

	/* We don't support double-clocked and Interlaced modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	/* MAX support pixel clock rate 594MHz */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	/* 5120 x 2160 is the maximum supported resolution */
	if (mode->hdisplay > 5120 || mode->vdisplay > 2160)
		return MODE_BAD_HVALUE;

	/* imx8mq-hdmi does not support non CEA modes */
	if (!strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11)) {
		vic = drm_match_cea_mode(mode);
		if (vic == 0)
			return MODE_BAD;
	}

	mhdp->valid_mode = mode;
	ret = cdns_mhdp_plat_call(mhdp, phy_video_valid);
	if (ret == false)
		return MODE_CLOCK_RANGE;

	return mode_status;
}

static void cdns_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *orig_mode,
				    const struct drm_display_mode *mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct video_info *video = &mhdp->video_info;

	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock);
	memcpy(&mhdp->mode, mode, sizeof(struct drm_display_mode));

	mutex_lock(&mhdp->lock);
	cdns_hdmi_mode_set(mhdp);
	mutex_unlock(&mhdp->lock);
}

bool cdns_hdmi_bridge_mode_fixup(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_connector_state *new_state = &mhdp->connector.new_state;
	struct drm_display_info *di = &mhdp->connector.base.display_info;
	struct video_info *video = &mhdp->video_info;
	int vic = drm_match_cea_mode(mode);

	video->color_depth = 8;
	video->color_fmt = PXL_RGB;

	/* for all other platforms, other than imx8mq */
	if (strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11)) {
		if (di->bpc == 10 || di->bpc == 6)
			video->color_depth = di->bpc;

		return true;
	}

	/* H20 Section 7.2.2, Colorimetry BT2020 for pixel encoding 10bpc or more */
	if (new_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_RGB) {
		if (drm_mode_is_420_only(di, mode))
			return false;

		/* BT2020_RGB for RGB 10bit or more  */
		/* 10b RGB is not supported for following VICs */
		if (vic == 97 || vic == 96 || vic == 95 || vic == 93 || vic == 94)
			return false;

		video->color_depth = 10;
	} else if (new_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_CYCC ||
	    new_state->colorspace == DRM_MODE_COLORIMETRY_BT2020_YCC) {
		/* BT2020_YCC/CYCC for YUV 10bit or more */
		if (drm_mode_is_420_only(di, mode) ||
				drm_mode_is_420_also(di, mode))
			video->color_fmt = YCBCR_4_2_0;
		else
			video->color_fmt = YCBCR_4_2_2;

		if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_36)
			video->color_depth = 12;
		else if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_30)
			video->color_depth = 10;
	} else if (new_state->colorspace == DRM_MODE_COLORIMETRY_SMPTE_170M_YCC ||
	    new_state->colorspace == DRM_MODE_COLORIMETRY_BT709_YCC ||
		new_state->colorspace == DRM_MODE_COLORIMETRY_XVYCC_601 ||
		new_state->colorspace == DRM_MODE_COLORIMETRY_XVYCC_709 ||
		new_state->colorspace == DRM_MODE_COLORIMETRY_SYCC_601) {
		/* Colorimetry for HD and SD YUV */
		if (drm_mode_is_420_only(di, mode) || drm_mode_is_420_also(di, mode))
			video->color_fmt = YCBCR_4_2_0;
		else
			video->color_fmt = YCBCR_4_4_4;
	} else if (new_state->colorspace == DRM_MODE_COLORIMETRY_DEFAULT)
		return !drm_mode_is_420_only(di, mode);

	return true;
}

static const struct drm_bridge_funcs cdns_hdmi_bridge_funcs = {
	.attach = cdns_hdmi_bridge_attach,
	.enable = cdns_hdmi_bridge_enable,
	.disable = cdns_hdmi_bridge_disable,
	.mode_set = cdns_hdmi_bridge_mode_set,
	.mode_valid = cdns_hdmi_bridge_mode_valid,
	.mode_fixup = cdns_hdmi_bridge_mode_fixup,
};

static void hotplug_work_func(struct work_struct *work)
{
	struct cdns_mhdp_device *mhdp = container_of(work,
					   struct cdns_mhdp_device, hotplug_work.work);
	struct drm_connector *connector = &mhdp->connector.base;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		DRM_INFO("HDMI Cable Plug In\n");
		mhdp->force_mode_set = true;
		enable_irq(mhdp->irq[IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI Cable Plug Out\n");
		/* force mode set for cable replugin to recovery HDMI2.0 video modes */
		mhdp->force_mode_set = true;
		enable_irq(mhdp->irq[IRQ_IN]);
	}
}

static irqreturn_t cdns_hdmi_irq_thread(int irq, void *data)
{
	struct cdns_mhdp_device *mhdp = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &mhdp->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void cdns_hdmi_parse_dt(struct cdns_mhdp_device *mhdp)
{
	struct device_node *of_node = mhdp->dev->of_node;
	int ret;

	ret = of_property_read_u32(of_node, "lane-mapping", &mhdp->lane_mapping);
	if (ret) {
		mhdp->lane_mapping = 0xc6;
		dev_warn(mhdp->dev, "Failed to get lane_mapping - using default 0xc6\n");
	}
	dev_info(mhdp->dev, "lane-mapping 0x%02x\n", mhdp->lane_mapping);
}

#ifdef CONFIG_DRM_CDNS_HDMI_CEC
static void cdns_mhdp_cec_init(struct cdns_mhdp_device *mhdp)
{
	struct cdns_mhdp_cec *cec = &mhdp->hdmi.cec;

	cec->dev = mhdp->dev;
	cec->iolock = &mhdp->iolock;
	cec->regs_base = mhdp->regs_base;
	cec->regs_sec = mhdp->regs_sec;
	cec->bus_type = mhdp->bus_type;
}
#endif

static int __cdns_hdmi_probe(struct platform_device *pdev,
		  struct cdns_mhdp_device *mhdp)
{
	struct device *dev = &pdev->dev;
	struct platform_device_info pdevinfo;
	struct resource *iores = NULL;
	int ret;

	mutex_init(&mhdp->lock);
	mutex_init(&mhdp->api_lock);
	mutex_init(&mhdp->iolock);

	INIT_DELAYED_WORK(&mhdp->hotplug_work, hotplug_work_func);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhdp->regs_base = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_base)) {
		dev_err(dev, "No regs_base memory\n");
		return -ENOMEM;
	}

	/* sec register base */
	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mhdp->regs_sec = devm_ioremap(dev, iores->start, resource_size(iores));
	if (IS_ERR(mhdp->regs_sec)) {
		dev_err(dev, "No regs_sec memory\n");
		return -ENOMEM;
	}

	mhdp->irq[IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (mhdp->irq[IRQ_IN] < 0) {
		dev_info(dev, "No plug_in irq number\n");
		return -EPROBE_DEFER;
	}

	mhdp->irq[IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (mhdp->irq[IRQ_OUT] < 0) {
		dev_info(dev, "No plug_out irq number\n");
		return -EPROBE_DEFER;
	}

	cdns_mhdp_plat_call(mhdp, power_on);

	/* Initialize FW */
	cdns_mhdp_plat_call(mhdp, firmware_init);

	/* HDMI FW alive check */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		dev_err(dev, "NO HDMI FW running\n");
		return -ENXIO;
	}

	/* Enable Hotplug Detect thread */
	irq_set_status_flags(mhdp->irq[IRQ_IN], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_IN],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					mhdp);
	if (ret < 0) {
		dev_err(dev, "can't claim irq %d\n",
						mhdp->irq[IRQ_IN]);
		return -EINVAL;
	}

	irq_set_status_flags(mhdp->irq[IRQ_OUT], IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_OUT],
					NULL, cdns_hdmi_irq_thread,
					IRQF_ONESHOT, dev_name(dev),
					mhdp);
	if (ret < 0) {
		dev_err(dev, "can't claim irq %d\n",
						mhdp->irq[IRQ_OUT]);
		return -EINVAL;
	}

	cdns_hdmi_parse_dt(mhdp);

	ret = cdns_hdcp_init(mhdp, pdev->dev.of_node);
	if (ret < 0)
		DRM_WARN("Failed to initialize HDCP\n");

	cnds_hdcp_create_device_files(mhdp);

	if (cdns_mhdp_read_hpd(mhdp))
		enable_irq(mhdp->irq[IRQ_OUT]);
	else
		enable_irq(mhdp->irq[IRQ_IN]);

	mhdp->bridge.base.driver_private = mhdp;
	mhdp->bridge.base.funcs = &cdns_hdmi_bridge_funcs;
#ifdef CONFIG_OF
	mhdp->bridge.base.of_node = dev->of_node;
#endif
	mhdp->last_connector_result = connector_status_disconnected;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent = dev;
	pdevinfo.id = PLATFORM_DEVID_AUTO;

	dev_set_drvdata(dev, mhdp);

	/* register audio driver */
	cdns_mhdp_register_audio_driver(dev);

	/* register cec driver */
#ifdef CONFIG_DRM_CDNS_HDMI_CEC
	cdns_mhdp_cec_init(mhdp);
	cdns_mhdp_register_cec_driver(&mhdp->hdmi.cec);
#endif

	return 0;
}

static void __cdns_hdmi_remove(struct cdns_mhdp_device *mhdp)
{
	/* unregister cec driver */
#ifdef CONFIG_DRM_CDNS_HDMI_CEC
	cdns_mhdp_unregister_cec_driver(&mhdp->hdmi.cec);
#endif
	cdns_mhdp_unregister_audio_driver(mhdp->dev);
}

/* -----------------------------------------------------------------------------
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
int cdns_hdmi_probe(struct platform_device *pdev,
		struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret  = __cdns_hdmi_probe(pdev, mhdp);
	if (ret < 0)
		return ret;

	drm_bridge_add(&mhdp->bridge.base);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_probe);

void cdns_hdmi_remove(struct platform_device *pdev)
{
	struct cdns_mhdp_device *mhdp = platform_get_drvdata(pdev);

	drm_bridge_remove(&mhdp->bridge.base);

	__cdns_hdmi_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_remove);

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_hdmi_bind(struct platform_device *pdev, struct drm_encoder *encoder,
			struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = __cdns_hdmi_probe(pdev, mhdp);
	if (ret)
		return ret;

	ret = drm_bridge_attach(encoder, &mhdp->bridge.base, NULL, 0);
	if (ret) {
		cdns_hdmi_remove(pdev);
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdmi_bind);

void cdns_hdmi_unbind(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	__cdns_hdmi_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_hdmi_unbind);

MODULE_AUTHOR("Sandor Yu <sandor.yu@nxp.com>");
MODULE_DESCRIPTION("Cadence HDMI transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdn-hdmi");
