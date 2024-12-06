/*
 * Copyright 2019-2020 NXP
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license version 2 as
 * published by the free software foundation.
 */
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_of.h>
#include <drm/drm_vblank.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_modeset_helper_vtables.h>

#include "cdns-mhdp-imx.h"
#include "cdns-mhdp-phy.h"
#include "../imx-drm.h"

static void cdns_mhdp_imx_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_bridge *bridge = drm_bridge_chain_get_first_bridge(encoder);
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	if (mhdp->is_dp)
		cdns_dp_phy_shutdown(mhdp);
	else
		cdns_hdmi_phy_shutdown(mhdp);

	cdns_mhdp_plat_call(mhdp, plat_init);
}

static void cdns_mhdp_imx_encoder_enable(struct drm_encoder *encoder)
{
	struct drm_bridge *bridge = drm_bridge_chain_get_first_bridge(encoder);
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	cdns_mhdp_plat_call(mhdp, plat_init);
	if (mhdp->is_dp)
		cdns_dp_phy_power_up(mhdp);
	else
		cdns_hdmi_phy_power_up(mhdp);
}

static int cdns_mhdp_imx_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct drm_bridge *bridge = drm_bridge_chain_get_first_bridge(encoder);
	struct cdns_mhdp_device *mhdp = bridge->driver_private;

	if (mhdp->plat_data->video_format != 0)
		imx_crtc_state->bus_format = mhdp->plat_data->video_format;

	if (mhdp->force_mode_set) {
		crtc_state->mode_changed = true;
		/* reset force mode set flag */
		mhdp->force_mode_set = false;
	}

	return 0;
}

static const struct drm_encoder_helper_funcs cdns_mhdp_imx_encoder_helper_funcs = {
	.enable     = cdns_mhdp_imx_encoder_enable,
	.disable    = cdns_mhdp_imx_encoder_disable,
	.atomic_check = cdns_mhdp_imx_encoder_atomic_check,
};

static const struct drm_encoder_funcs cdns_mhdp_imx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct cdns_plat_data imx8mq_hdmi_drv_data = {
	.plat_name = "imx8mq-hdmi",
	.bind	= cdns_hdmi_bind,
	.unbind	= cdns_hdmi_unbind,
	.phy_set = cdns_hdmi_phy_set_imx8mq,
	.phy_video_valid = cdns_hdmi_phy_video_valid_imx8mq,
	.bus_type = BUS_TYPE_NORMAL_APB,
};

static struct cdns_plat_data imx8mq_dp_drv_data = {
	.plat_name = "imx8mq-dp",
	.bind	= cdns_dp_bind,
	.unbind	= cdns_dp_unbind,
	.phy_set = cdns_dp_phy_set_imx8mq,
	.bus_type = BUS_TYPE_NORMAL_APB,
};

static struct cdns_plat_data imx8qm_hdmi_drv_data = {
	.plat_name = "imx8qm-hdmi",
	.bind	= cdns_hdmi_bind,
	.unbind	= cdns_hdmi_unbind,
	.phy_set = cdns_hdmi_phy_set_imx8qm,
	.phy_video_valid = cdns_hdmi_phy_video_valid_imx8qm,
	.power_on = cdns_mhdp_power_on_imx8qm,
	.power_off = cdns_mhdp_power_off_imx8qm,
	.firmware_init = cdns_mhdp_firmware_init_imx8qm,
	.resume = cdns_mhdp_resume_imx8qm,
	.suspend = cdns_mhdp_suspend_imx8qm,
	.pclk_rate = cdns_mhdp_pclk_rate_imx8qm,
	.plat_init = cdns_mhdp_plat_init_imx8qm,
	.plat_deinit = cdns_mhdp_plat_deinit_imx8qm,
	.bus_type = BUS_TYPE_LOW4K_APB,
	.video_format =  MEDIA_BUS_FMT_RGB101010_1X30,
};

static struct cdns_plat_data imx8qm_dp_drv_data = {
	.plat_name = "imx8qm-dp",
	.bind	= cdns_dp_bind,
	.unbind	= cdns_dp_unbind,
	.phy_set = cdns_dp_phy_set_imx8qm,
	.power_on = cdns_mhdp_power_on_imx8qm,
	.power_off = cdns_mhdp_power_off_imx8qm,
	.firmware_init = cdns_mhdp_firmware_init_imx8qm,
	.resume = cdns_mhdp_resume_imx8qm,
	.suspend = cdns_mhdp_suspend_imx8qm,
	.pclk_rate = cdns_mhdp_pclk_rate_imx8qm,
	.plat_init = cdns_mhdp_plat_init_imx8qm,
	.plat_deinit = cdns_mhdp_plat_deinit_imx8qm,
	.bus_type = BUS_TYPE_LOW4K_APB,
	.video_format =  MEDIA_BUS_FMT_RGB101010_1X30,
	.is_dp = true,
};

static struct cdns_plat_data ls1028a_dp_drv_data = {
	.bind = cdns_dp_bind,
	.unbind = cdns_dp_unbind,
	.phy_set = cdns_dp_phy_set_imx8mq,
	.power_on = cdns_mhdp_power_on_ls1028a,
	.power_off = cdns_mhdp_power_off_ls1028a,
	.firmware_init = cdns_mhdp_firmware_init_imx8qm,
	.pclk_rate = cdns_mhdp_pclk_rate_ls1028a,
	.bus_type = BUS_TYPE_NORMAL_APB,
};

static const struct of_device_id cdns_mhdp_imx_dt_ids[] = {
	{ .compatible = "cdn,imx8mq-hdmi",
	  .data = &imx8mq_hdmi_drv_data
	},
	{ .compatible = "cdn,imx8mq-dp",
	  .data = &imx8mq_dp_drv_data
	},
	{ .compatible = "cdn,imx8qm-hdmi",
	  .data = &imx8qm_hdmi_drv_data
	},
	{ .compatible = "cdn,imx8qm-dp",
	  .data = &imx8qm_dp_drv_data
	},
	{ .compatible = "cdn,ls1028a-dp",
	  .data = &ls1028a_dp_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, cdns_mhdp_imx_dt_ids);

static int cdns_mhdp_imx_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct cdns_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct imx_mhdp_device *imx_mhdp;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	imx_mhdp = devm_kzalloc(&pdev->dev, sizeof(*imx_mhdp), GFP_KERNEL);
	if (!imx_mhdp)
		return -ENOMEM;

	match = of_match_node(cdns_mhdp_imx_dt_ids, pdev->dev.of_node);
	if (!match)
		return -EFAULT;

	plat_data = match->data;
	encoder = &imx_mhdp->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
					&imx_mhdp->firmware_name);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &cdns_mhdp_imx_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &cdns_mhdp_imx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);


	imx_mhdp->mhdp.plat_data = plat_data;
	imx_mhdp->mhdp.dev = dev;
	imx_mhdp->mhdp.drm_dev = drm;
	imx_mhdp->mhdp.bus_type = plat_data->bus_type;
	ret = plat_data->bind(pdev, encoder, &imx_mhdp->mhdp);
	/*
	 * If cdns_mhdp_bind() fails we'll never call cdns_mhdp_unbind(),
	 * which would have called the encoder cleanup.  Do it manually.
	 */
	if (ret < 0)
		drm_encoder_cleanup(encoder);

	return ret;
}

static void cdns_mhdp_imx_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	imx_mhdp->mhdp.plat_data->unbind(dev);
}

static const struct component_ops cdns_mhdp_imx_ops = {
	.bind	= cdns_mhdp_imx_bind,
	.unbind	= cdns_mhdp_imx_unbind,
};

static int cdns_mhdp_imx_suspend(struct device *dev)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	cdns_mhdp_plat_call(&imx_mhdp->mhdp, suspend);

	return 0;
}

static int cdns_mhdp_imx_resume(struct device *dev)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	cdns_mhdp_plat_call(&imx_mhdp->mhdp, resume);
	cdns_mhdp_plat_call(&imx_mhdp->mhdp, phy_set);

	return 0;
}

static int cdns_mhdp_imx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &cdns_mhdp_imx_ops);
}

static int cdns_mhdp_imx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cdns_mhdp_imx_ops);

	return 0;
}

static const struct dev_pm_ops cdns_mhdp_imx_pm_ops = {
        SET_LATE_SYSTEM_SLEEP_PM_OPS(cdns_mhdp_imx_suspend, cdns_mhdp_imx_resume)
};

static struct platform_driver cdns_mhdp_imx_platform_driver = {
	.probe  = cdns_mhdp_imx_probe,
	.remove = cdns_mhdp_imx_remove,
	.driver = {
		.name = "cdns-mhdp-imx",
		.of_match_table = cdns_mhdp_imx_dt_ids,
		.pm = &cdns_mhdp_imx_pm_ops,
	},
};

module_platform_driver(cdns_mhdp_imx_platform_driver);

MODULE_AUTHOR("Sandor YU <sandor.yu@nxp.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdnhdmi-imx");
