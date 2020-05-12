/*
 * Cadence Display Port Interface (DP) driver
 *
 * Copyright (C) 2019 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>

/*
 * This function only implements native DPDC reads and writes
 */
static ssize_t dp_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(aux->dev);
	bool native = msg->request & (DP_AUX_NATIVE_WRITE & DP_AUX_NATIVE_READ);
	int ret;

	/* Ignore address only message */
	if ((msg->size == 0) || (msg->buffer == NULL)) {
		msg->reply = native ?
			DP_AUX_NATIVE_REPLY_ACK : DP_AUX_I2C_REPLY_ACK;
		return msg->size;
	}

	if (!native) {
		dev_err(mhdp->dev, "%s: only native messages supported\n", __func__);
		return -EINVAL;
	}

	/* msg sanity check */
	if (msg->size > DP_AUX_MAX_PAYLOAD_BYTES) {
		dev_err(mhdp->dev, "%s: invalid msg: size(%zu), request(%x)\n",
						__func__, msg->size, (unsigned int)msg->request);
		return -EINVAL;
	}

	if (msg->request == DP_AUX_NATIVE_WRITE) {
		const u8 *buf = msg->buffer;
		int i;
		for (i = 0; i < msg->size; ++i) {
			ret = cdns_mhdp_dpcd_write(mhdp,
						   msg->address + i, buf[i]);
			if (!ret)
				continue;

			DRM_DEV_ERROR(mhdp->dev, "Failed to write DPCD\n");

			return ret;
		}
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	}

	if (msg->request == DP_AUX_NATIVE_READ) {
		ret = cdns_mhdp_dpcd_read(mhdp, msg->address, msg->buffer, msg->size);
		if (ret < 0)
			return -EIO;
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	}
	return 0;
}

static int dp_aux_init(struct cdns_mhdp_device *mhdp,
		  struct device *dev)
{
	int ret;

	mhdp->dp.aux.name = "imx_dp_aux";
	mhdp->dp.aux.dev = dev;
	mhdp->dp.aux.transfer = dp_aux_transfer;

	ret = drm_dp_aux_register(&mhdp->dp.aux);

	return ret;
}

static int dp_aux_destroy(struct cdns_mhdp_device *mhdp)
{
	drm_dp_aux_unregister(&mhdp->dp.aux);
	return 0;
}

static void dp_pixel_clk_reset(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	/* reset pixel clk */
	val = cdns_mhdp_reg_read(mhdp, SOURCE_HDTX_CAR);
	cdns_mhdp_reg_write(mhdp, SOURCE_HDTX_CAR, val & 0xFD);
	cdns_mhdp_reg_write(mhdp, SOURCE_HDTX_CAR, val);
}

static void cdns_dp_mode_set(struct cdns_mhdp_device *mhdp)
{
	u32 lane_mapping = mhdp->lane_mapping;
	int ret;

	cdns_mhdp_plat_call(mhdp, pclk_rate);

	/* delay for DP FW stable after pixel clock relock */
	msleep(50);

	dp_pixel_clk_reset(mhdp);

	/* Get DP Caps  */
	ret = drm_dp_dpcd_read(&mhdp->dp.aux, DP_DPCD_REV, mhdp->dp.dpcd,
			       DP_RECEIVER_CAP_SIZE);
	if (ret < 0) {
		DRM_ERROR("Failed to get caps %d\n", ret);
		return;
	}

	mhdp->dp.rate = drm_dp_max_link_rate(mhdp->dp.dpcd);
	mhdp->dp.num_lanes = drm_dp_max_lane_count(mhdp->dp.dpcd);

	/* check the max link rate */
	if (mhdp->dp.rate > CDNS_DP_MAX_LINK_RATE)
		mhdp->dp.rate = CDNS_DP_MAX_LINK_RATE;

	/* Initialize link rate/num_lanes as panel max link rate/max_num_lanes */
	cdns_mhdp_plat_call(mhdp, phy_set);

	/* Video off */
	ret = cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_IDLE);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}

	/* Line swaping */
	cdns_mhdp_reg_write(mhdp, LANES_CONFIG, 0x00400000 | lane_mapping);

	/* Set DP host capability */
	ret = cdns_mhdp_set_host_cap(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to set host cap %d\n", ret);
		return;
	}

	ret = cdns_mhdp_config_video(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to config video %d\n", ret);
		return;
	}

	return;
}

/* -----------------------------------------------------------------------------
 * dp TX Setup
 */
static enum drm_connector_status
cdns_dp_connector_detect(struct drm_connector *connector, bool force)
{
	struct cdns_mhdp_device *mhdp = container_of(connector,
					struct cdns_mhdp_device, connector.base);
	u8 hpd = 0xf;

	hpd = cdns_mhdp_read_hpd(mhdp);
	if (hpd == 1)
		/* Cable Connected */
		return connector_status_connected;
	else if (hpd == 0)
		/* Cable Disconnedted */
		return connector_status_disconnected;
	else {
		/* Cable status unknown */
		DRM_INFO("Unknow cable status, hdp=%u\n", hpd);
		return connector_status_unknown;
	}
}

static int cdns_dp_connector_get_modes(struct drm_connector *connector)
{
	struct cdns_mhdp_device *mhdp = container_of(connector,
					struct cdns_mhdp_device, connector.base);
	int num_modes = 0;
	struct edid *edid;

	edid = drm_do_get_edid(&mhdp->connector.base,
				   cdns_mhdp_get_edid_block, mhdp);
	if (edid) {
		dev_info(mhdp->dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
			 edid->header[0], edid->header[1],
			 edid->header[2], edid->header[3],
			 edid->header[4], edid->header[5],
			 edid->header[6], edid->header[7]);
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	if (num_modes == 0)
		DRM_ERROR("Invalid edid\n");
	return num_modes;
}

static const struct drm_connector_funcs cdns_dp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = cdns_dp_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs cdns_dp_connector_helper_funcs = {
	.get_modes = cdns_dp_connector_get_modes,
};

static int cdns_dp_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector = &mhdp->connector.base;

	connector->interlace_allowed = 1;

	if (mhdp->is_hpd)
		connector->polled = DRM_CONNECTOR_POLL_HPD;
	else
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_helper_add(connector, &cdns_dp_connector_helper_funcs);

	drm_connector_init(bridge->dev, connector, &cdns_dp_connector_funcs,
			   DRM_MODE_CONNECTOR_DisplayPort);

	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static enum drm_mode_status
cdns_dp_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_mode *mode)
{
	enum drm_mode_status mode_status = MODE_OK;

	/* We don't support double-clocked modes */
	if (mode->flags & DRM_MODE_FLAG_DBLCLK ||
			mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_BAD;

	/* MAX support pixel clock rate 594MHz */
	if (mode->clock > 594000)
		return MODE_CLOCK_HIGH;

	/* 4096x2160 is not supported now */
	if (mode->hdisplay > 3840)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 2160)
		return MODE_BAD_VVALUE;

	return mode_status;
}

static void cdns_dp_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *orig_mode,
				    const struct drm_display_mode *mode)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	struct drm_display_info *display_info = &mhdp->connector.base.display_info;
	struct video_info *video = &mhdp->video_info;

	switch (display_info->bpc) {
	case 10:
		video->color_depth = 10;
		break;
	case 6:
		video->color_depth = 6;
		break;
	default:
		video->color_depth = 8;
		break;
	}

	video->color_fmt = PXL_RGB;
	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	DRM_INFO("Mode: %dx%dp%d\n", mode->hdisplay, mode->vdisplay, mode->clock); 
	memcpy(&mhdp->mode, mode, sizeof(struct drm_display_mode));

	mutex_lock(&mhdp->lock);
	cdns_dp_mode_set(mhdp);
	mutex_unlock(&mhdp->lock);

	/* reset force mode set flag */
	mhdp->force_mode_set = false;
}

static void cdn_dp_bridge_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge->driver_private;
	int ret;

	/* Link trainning */
	ret = cdns_mhdp_train_link(mhdp);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed link train %d\n", ret);
		return;
	}

	ret = cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_VALID);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to valid video %d\n", ret);
		return;
	}
}

static void cdn_dp_bridge_disable(struct drm_bridge *bridge)
{	
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	cdns_mhdp_set_video_status(mhdp, CONTROL_VIDEO_IDLE);
}

static const struct drm_bridge_funcs cdns_dp_bridge_funcs = {
	.attach = cdns_dp_bridge_attach,
	.enable = cdn_dp_bridge_enable,
	.disable = cdn_dp_bridge_disable,
	.mode_set = cdns_dp_bridge_mode_set,
	.mode_valid = cdns_dp_bridge_mode_valid,
};

static void hotplug_work_func(struct work_struct *work)
{
	struct cdns_mhdp_device *mhdp = container_of(work,
					   struct cdns_mhdp_device, hotplug_work.work);
	struct drm_connector *connector = &mhdp->connector.base;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		/* Cable connedted  */
		DRM_INFO("HDMI/DP Cable Plug In\n");
		enable_irq(mhdp->irq[IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI/DP Cable Plug Out\n");
		/* force mode set for cable replugin to recovery DP video modes */
		mhdp->force_mode_set = true;
		enable_irq(mhdp->irq[IRQ_IN]);
	}
}

static irqreturn_t cdns_dp_irq_thread(int irq, void *data)
{
	struct cdns_mhdp_device *mhdp = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &mhdp->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void cdns_dp_parse_dt(struct cdns_mhdp_device *mhdp)
{
	struct device_node *of_node = mhdp->dev->of_node;
	int ret;

	ret = of_property_read_u32(of_node, "lane-mapping",
						&mhdp->lane_mapping);
	if (ret) {
		mhdp->lane_mapping = 0xc6;
		dev_warn(mhdp->dev, "Failed to get lane_mapping - using default 0xc6\n");
	}
	dev_info(mhdp->dev, "lane-mapping 0x%02x\n", mhdp->lane_mapping);
}

static int __cdns_dp_probe(struct platform_device *pdev,
		struct cdns_mhdp_device *mhdp)
{
	struct device *dev = &pdev->dev;
	struct resource *iores = NULL;
	int ret;

	mutex_init(&mhdp->lock);
	mutex_init(&mhdp->iolock);

	INIT_DELAYED_WORK(&mhdp->hotplug_work, hotplug_work_func);

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (iores) {
		mhdp->regs_base = devm_ioremap(dev, iores->start,
					       resource_size(iores));
		if (IS_ERR(mhdp->regs_base))
			return -ENOMEM;
	}

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (iores) {
		mhdp->regs_sec = devm_ioremap(dev, iores->start,
					      resource_size(iores));
		if (IS_ERR(mhdp->regs_sec))
			return -ENOMEM;
	}

	mhdp->is_hpd = true;
	mhdp->is_ls1028a = false;

	mhdp->irq[IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (mhdp->irq[IRQ_IN] < 0) {
		mhdp->is_hpd = false;
		dev_info(dev, "No plug_in irq number\n");
	}

	mhdp->irq[IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (mhdp->irq[IRQ_OUT] < 0) {
		mhdp->is_hpd = false;
		dev_info(dev, "No plug_out irq number\n");
	}

	cdns_dp_parse_dt(mhdp);

	if (of_device_is_compatible(dev->of_node, "cdn,ls1028a-dp"))
		mhdp->is_ls1028a = true;

	cdns_mhdp_plat_call(mhdp, power_on);

	cdns_mhdp_plat_call(mhdp, firmware_init);

	/* DP FW alive check */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		DRM_ERROR("NO dp FW running\n");
		return -ENXIO;
	}

	/* DP PHY init before AUX init */
	cdns_mhdp_plat_call(mhdp, phy_set);

	/* Enable Hotplug Detect IRQ thread */
	if (mhdp->is_hpd) {
		irq_set_status_flags(mhdp->irq[IRQ_IN], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_IN],
						NULL, cdns_dp_irq_thread,
						IRQF_ONESHOT, dev_name(dev),
						mhdp);
	
		if (ret) {
			dev_err(dev, "can't claim irq %d\n",
					mhdp->irq[IRQ_IN]);
			return -EINVAL;
		}

		irq_set_status_flags(mhdp->irq[IRQ_OUT], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, mhdp->irq[IRQ_OUT],
						NULL, cdns_dp_irq_thread,
						IRQF_ONESHOT, dev_name(dev),
						mhdp);

		if (ret) {
			dev_err(dev, "can't claim irq %d\n",
					mhdp->irq[IRQ_OUT]);
			return -EINVAL;
		}

		if (cdns_mhdp_read_hpd(mhdp))
			enable_irq(mhdp->irq[IRQ_OUT]);
		else
			enable_irq(mhdp->irq[IRQ_IN]);
	}

	mhdp->bridge.base.driver_private = mhdp;
	mhdp->bridge.base.funcs = &cdns_dp_bridge_funcs;
#ifdef CONFIG_OF
	mhdp->bridge.base.of_node = dev->of_node;
#endif

	dev_set_drvdata(dev, mhdp);
	
	/* register audio driver */
	cdns_mhdp_register_audio_driver(dev);

	dp_aux_init(mhdp, dev);

	return 0;
}

static void __cdns_dp_remove(struct cdns_mhdp_device *mhdp)
{
	dp_aux_destroy(mhdp);
	cdns_mhdp_unregister_audio_driver(mhdp->dev);
}

/* -----------------------------------------------------------------------------
 * Probe/remove API, used from platforms based on the DRM bridge API.
 */
int cdns_dp_probe(struct platform_device *pdev,
		  struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = __cdns_dp_probe(pdev, mhdp);
	if (ret)
		return ret;

	drm_bridge_add(&mhdp->bridge.base);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_dp_probe);

void cdns_dp_remove(struct platform_device *pdev)
{
	struct cdns_mhdp_device *mhdp = platform_get_drvdata(pdev);

	drm_bridge_remove(&mhdp->bridge.base);

	__cdns_dp_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_dp_remove);

/* -----------------------------------------------------------------------------
 * Bind/unbind API, used from platforms based on the component framework.
 */
int cdns_dp_bind(struct platform_device *pdev, struct drm_encoder *encoder,
		struct cdns_mhdp_device *mhdp)
{
	int ret;

	ret = __cdns_dp_probe(pdev, mhdp);
	if (ret < 0)
		return ret;

	ret = drm_bridge_attach(encoder, &mhdp->bridge.base, NULL, 0);
	if (ret) {
		cdns_dp_remove(pdev);
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_dp_bind);

void cdns_dp_unbind(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	__cdns_dp_remove(mhdp);
}
EXPORT_SYMBOL_GPL(cdns_dp_unbind);

MODULE_AUTHOR("Sandor Yu <sandor.yu@nxp.com>");
MODULE_DESCRIPTION("Cadence Display Port transmitter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdn-dp");
