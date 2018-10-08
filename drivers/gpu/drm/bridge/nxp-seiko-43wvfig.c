/*
 * DRM driver for the legacy Freescale adapter card holding
 * Seiko RA169Z20 43WVFIG LCD panel
 *
 * Copyright (C) 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>

struct seiko_adapter {
	struct device		*dev;
	struct drm_panel	*panel;
	struct drm_bridge	bridge;
	struct drm_connector	connector;

	u32			bpc;
	u32			bus_format;
};

static enum drm_connector_status seiko_adapter_connector_detect(
	struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int seiko_adapter_connector_get_modes(struct drm_connector *connector)
{
	int num_modes;

	struct seiko_adapter *adap = container_of(connector,
						struct seiko_adapter,
						connector);

	num_modes = drm_panel_get_modes(adap->panel);

	/*
	 * The panel will populate the connector display_info properties with
	 * fixed numbers, but we need to change them according to our
	 * configuration.
	 */
	connector->display_info.bpc = adap->bpc;
	drm_display_info_set_bus_formats(&connector->display_info,
						 &adap->bus_format, 1);

	return num_modes;
}

static const struct drm_connector_funcs seiko_adapter_connector_funcs = {
	.detect = seiko_adapter_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs
	seiko_adapter_connector_helper_funcs = {
	.get_modes = seiko_adapter_connector_get_modes,
};

static int seiko_adapter_bridge_attach(struct drm_bridge *bridge)
{
	struct seiko_adapter *adap = bridge->driver_private;
	struct device *dev = adap->dev;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_device *drm;
	int ret = 0;

	if (!encoder) {
		DRM_DEV_ERROR(dev, "Parent encoder object not found\n");
		return -ENODEV;
	}

	drm = encoder->dev;

	/*
	 * Create the connector for our panel
	 */

	ret = drm_connector_init(drm, &adap->connector,
				 &seiko_adapter_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to init drm connector: %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&adap->connector,
				 &seiko_adapter_connector_helper_funcs);

	adap->connector.dpms = DRM_MODE_DPMS_OFF;
	drm_mode_connector_attach_encoder(&adap->connector, encoder);

	ret = drm_panel_attach(adap->panel, &adap->connector);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to attach panel: %d\n", ret);
		drm_connector_cleanup(&adap->connector);
		return ret;
	}

	return ret;
}

static void seiko_adapter_bridge_detach(struct drm_bridge *bridge)
{
	struct seiko_adapter *adap = bridge->driver_private;

	drm_panel_detach(adap->panel);
	drm_connector_cleanup(&adap->connector);
}

static void seiko_adapter_bridge_enable(struct drm_bridge *bridge)
{
	struct seiko_adapter *adap = bridge->driver_private;
	struct device *dev = adap->dev;

	if (drm_panel_prepare(adap->panel)) {
		DRM_DEV_ERROR(dev, "Failed to prepare panel\n");
		return;
	}

	if (drm_panel_enable(adap->panel)) {
		DRM_DEV_ERROR(dev, "Failed to enable panel\n");
		drm_panel_unprepare(adap->panel);
	}
}

static void seiko_adapter_bridge_disable(struct drm_bridge *bridge)
{
	struct seiko_adapter *adap = bridge->driver_private;
	struct device *dev = adap->dev;

	if (drm_panel_disable(adap->panel)) {
		DRM_DEV_ERROR(dev, "failed to disable panel\n");
		return;
	}

	if (drm_panel_unprepare(adap->panel))
		DRM_DEV_ERROR(dev, "failed to unprepare panel\n");
}

static const struct drm_bridge_funcs seiko_adapter_bridge_funcs = {
	.enable = seiko_adapter_bridge_enable,
	.disable = seiko_adapter_bridge_disable,
	.attach = seiko_adapter_bridge_attach,
	.detach = seiko_adapter_bridge_detach,
};

static int seiko_adapter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct seiko_adapter *adap;
	struct device_node *remote;
	u32 bus_mode;
	int ret, port;

	adap = devm_kzalloc(dev, sizeof(*adap), GFP_KERNEL);
	if (!adap)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "bus_mode", &bus_mode);
	if (bus_mode != 18 && bus_mode != 24) {
		dev_err(dev, "Invalid bus_mode: %d\n", bus_mode);
		return -EINVAL;
	}

	switch (bus_mode) {
	case 18:
		adap->bpc = 6;
		adap->bus_format = MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case 24:
		adap->bpc = 8;
		adap->bus_format = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	}

	for (port = 0; port < 2; port++) {
		remote = of_graph_get_remote_node(dev->of_node, port, -1);
		if (!remote) {
			pr_info("No remote for port %d\n", port);
			return -ENODEV;
		}
		adap->panel = of_drm_find_panel(remote);
		if (adap->panel)
			break;
	}
	if (!adap->panel) {
		dev_err(dev, "No panel found!\n");
		return -ENODEV;
	}

	adap->dev = dev;
	adap->bridge.driver_private = adap;
	adap->bridge.funcs = &seiko_adapter_bridge_funcs;
	adap->bridge.of_node = dev->of_node;

	ret = drm_bridge_add(&adap->bridge);
	if (ret < 0)
		dev_err(dev, "Failed to add seiko-adapter bridge (%d)\n", ret);

	dev_info(dev, "Seiko adapter driver probed\n");
	return ret;
}

static int seiko_adapter_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id seiko_adapter_dt_ids[] = {
	{ .compatible = "nxp,seiko-43wvfig" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, seiko_adapter_dt_ids);

static struct platform_driver seiko_adapter_driver = {
	.probe		= seiko_adapter_probe,
	.remove		= seiko_adapter_remove,
	.driver		= {
		.of_match_table = seiko_adapter_dt_ids,
		.name	= "nxp-seiko-adapter",
	},
};

module_platform_driver(seiko_adapter_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Seiko 43WVFIG adapter card driver");
MODULE_LICENSE("GPL");
