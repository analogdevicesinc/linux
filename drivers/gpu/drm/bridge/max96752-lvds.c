// SPDX-License-Identifier: GPL-2.0-only
/*
 * Core MFD support for Analog Devices MAX96752 MIPI-DSI serializer
 *
 * Copyright 2023 NXP
 */

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/maxim_serdes.h>
#include <linux/mfd/max96752.h>
#include <linux/module.h>
#include <linux/regmap.h>


#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_module.h>

enum max96752_lvds_id {
	LVDS_PORT_A,
	LVDS_PORT_B,
	MAX_LVDS_PORTS
};

enum max96752_lvds_bus_format {
	BUS_FORMAT_OLDI,
	BUS_FORMAT_VESA,
};

struct max96752_lvds {
	struct device *dev;
	struct max96752 *max96752_mfd;

	struct drm_bridge bridge;
	struct drm_bridge *panel_bridge;

	enum max96752_lvds_bus_format bus_format;
	bool dual_channel;
	u32 oldi_ssr;
};

static void max96752_lvds_bridge_atomic_pre_enable(struct drm_bridge *bridge,
						   struct drm_bridge_state *old_bridge_state)
{
	struct max96752_lvds *max96752_lvds = container_of(bridge, struct max96752_lvds, bridge);
	struct max96752 *mfd = max96752_lvds->max96752_mfd;
	unsigned int reg_mask, reg_val;


	reg_mask = (max96752_lvds->dual_channel ? OLDI_SPL_EN : 0) | OLDI_FORMAT;
	reg_val = (max96752_lvds->dual_channel ? OLDI_SPL_EN : 0) |
		  (max96752_lvds->bus_format ? OLDI_FORMAT : 0);
	regmap_update_bits(mfd->regmap, MAX96752_VRX_OLDI1, reg_mask, reg_val);
}

static int max96752_lvds_bridge_attach(struct drm_bridge *bridge,
				       enum drm_bridge_attach_flags flags)
{
	struct max96752_lvds *max96752_lvds = container_of(bridge, struct max96752_lvds, bridge);
	struct max96752 *mfd = max96752_lvds->max96752_mfd;
	struct device *dev = max96752_lvds->dev;

	max96752_lvds->panel_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 1);
	if (IS_ERR(max96752_lvds->panel_bridge)) {
		dev_err(dev, "Couldn't find next panel/bridge: %ld\n",
			PTR_ERR(max96752_lvds->panel_bridge));

		return PTR_ERR(max96752_lvds->panel_bridge);
	}

	/* enable spread spectrum, if needed */
	if (max96752_lvds->oldi_ssr) {
		regmap_update_bits(mfd->regmap, MAX96752_DPLL_OLDI_DPLL_3,
				   OLDI_CONFIG_SPREAD_BIT_RATIO_MASK,
				   max96752_lvds->oldi_ssr << OLDI_CONFIG_SPREAD_BIT_RATIO_SHIFT);
		regmap_update_bits(mfd->regmap, MAX96752_VRX_OLDI2, SSEN, SSEN);
	}

	return drm_bridge_attach(bridge->encoder, max96752_lvds->panel_bridge, bridge, flags);
}

static int max96752_lvds_atomic_check(struct drm_bridge *bridge,
				      struct drm_bridge_state *bridge_state,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct drm_display_info *di = &conn_state->connector->display_info;
	struct drm_bridge_state *next_bridge_state = NULL;
	struct drm_bridge *next_bridge;
	u32 bus_flags = 0;
	u32 bus_format = 0;
	struct max96752_lvds *max96752_lvds = container_of(bridge, struct max96752_lvds, bridge);

	next_bridge = drm_bridge_get_next_bridge(bridge);
	if (next_bridge)
		next_bridge_state = drm_atomic_get_new_bridge_state(crtc_state->state,
								    next_bridge);

	if (next_bridge_state) {
		bus_flags = next_bridge_state->input_bus_cfg.flags;
		bus_format = next_bridge_state->input_bus_cfg.format;
	} else if (di->num_bus_formats) {
		bus_flags = di->bus_flags;
		bus_format = di->bus_formats[0];
	}

	bridge_state->output_bus_cfg.flags = bus_flags;

	if (bus_format == MEDIA_BUS_FMT_RGB888_1X7X4_SPWG)
		max96752_lvds->bus_format = BUS_FORMAT_VESA;
	else if (bus_format == MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA)
		max96752_lvds->bus_format = BUS_FORMAT_OLDI;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_bridge_funcs max96752_lvds_bridge_funcs = {
	.atomic_check = max96752_lvds_atomic_check,
	.atomic_pre_enable = max96752_lvds_bridge_atomic_pre_enable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = max96752_lvds_bridge_attach,
};

static int max96752_lvds_dt_parse(struct max96752_lvds *max96752_lvds)
{
	struct device *dev = max96752_lvds->dev;
	struct device_node *dt_port;
	int ret;

	dt_port = of_graph_get_port_by_id(dev->of_node, 0);
	if (!dt_port) {
		dev_err(dev, "port definition is missing in DT\n");
		return -ENODEV;
	}

	of_node_put(dt_port);

	max96752_lvds->dual_channel = of_property_read_bool(dev->of_node, "maxim,dual-channel");

	ret = of_property_read_u32(dev->of_node, "maxim,oldi-ssr", &max96752_lvds->oldi_ssr);
	if (ret && ret != -EINVAL)
		return ret;

	if (max96752_lvds->oldi_ssr > 5) {
		dev_err(dev, "SSR value provided is out of range.\n");
		return -EINVAL;
	}

	max96752_lvds->bridge.driver_private = max96752_lvds;
	max96752_lvds->bridge.funcs = &max96752_lvds_bridge_funcs;
	max96752_lvds->bridge.of_node = dev->of_node;

	drm_bridge_add(&max96752_lvds->bridge);

	return 0;
}

static const struct of_device_id max96752_of_match[] = {
	{ .compatible = "maxim,max96752", .data = (void *)ID_MAX96752 },
	{}
};
MODULE_DEVICE_TABLE(of, max96752_of_match);

static int max96752_lvds_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max96752_lvds *max96752_lvds;
	struct max96752 *mfd = dev_get_drvdata(dev->parent);
	const struct of_device_id *match;

	match = of_match_node(max96752_of_match, dev->parent->of_node);
	if (!match) {
		dev_err(dev, "invalid compatible string\n");
		return -ENODEV;
	}

	if (!mfd->link_setup_finished)
		return -EPROBE_DEFER;

	max96752_lvds = devm_kzalloc(dev, sizeof(*max96752_lvds), GFP_KERNEL);
	if (!max96752_lvds)
		return -ENOMEM;

	max96752_lvds->dev = dev;
	max96752_lvds->max96752_mfd = mfd;

	dev_set_drvdata(dev, max96752_lvds);

	return max96752_lvds_dt_parse(max96752_lvds);
}

static const struct of_device_id max96752_lvds_of_match[] = {
	{ .compatible = "maxim,max96752-lvds", .data = (void *)ID_MAX96752 },
	{}
};
MODULE_DEVICE_TABLE(of, max96752_lvds_of_match);

static struct platform_driver max96752_lvds_platform_driver = {
	.probe	= max96752_lvds_drm_probe,
	.driver	= {
		.name	= "max96752-lvds",
		.of_match_table = of_match_ptr(max96752_lvds_of_match),
	},
};

drm_module_platform_driver(max96752_lvds_platform_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_DESCRIPTION("MAX96752 deserializer LVDS bridge driver");
MODULE_LICENSE("GPL v2");
