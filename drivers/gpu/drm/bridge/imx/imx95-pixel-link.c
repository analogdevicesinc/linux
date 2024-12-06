// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2023 NXP
 */

#include <linux/bits.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>

#define CTRL		0x8
#define  PL_VALID(n)	BIT(1 + 4 * (n))
#define  PL_ENABLE(n)	BIT(4 * (n))

#define STREAMS		2
#define OUT_ENDPOINTS	2

#define DRIVER_NAME	"imx95-pixel-link"

struct imx95_pl_bridge {
	struct drm_bridge base;
	struct imx95_pl *pl;
	u32 id;
};

struct imx95_pl {
	struct device *dev;
	struct regmap *regmap;
	struct imx95_pl_bridge bridge[STREAMS];
	struct drm_bridge *next_bridge[STREAMS];
};

static int imx95_pl_bridge_attach(struct drm_bridge *bridge,
				  enum drm_bridge_attach_flags flags)
{
	struct imx95_pl_bridge *pl_bridge = bridge->driver_private;
	struct imx95_pl *pl = pl_bridge->pl;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		dev_err(pl->dev, "do not support creating a drm_connector\n");
		return -EINVAL;
	}

	if (!bridge->encoder) {
		dev_err(pl->dev, "missing encoder\n");
		return -ENODEV;
	}

	return drm_bridge_attach(bridge->encoder,
				 pl->next_bridge[pl_bridge->id], bridge,
				 DRM_BRIDGE_ATTACH_NO_CONNECTOR);
}

static void imx95_pl_bridge_disable(struct drm_bridge *bridge)
{
	struct imx95_pl_bridge *pl_bridge = bridge->driver_private;
	struct imx95_pl *pl = pl_bridge->pl;
	unsigned int id = pl_bridge->id;

	regmap_update_bits(pl->regmap, CTRL, PL_ENABLE(id), 0);
	regmap_update_bits(pl->regmap, CTRL, PL_VALID(id), 0);
}

static void imx95_pl_bridge_enable(struct drm_bridge *bridge)
{
	struct imx95_pl_bridge *pl_bridge = bridge->driver_private;
	struct imx95_pl *pl = pl_bridge->pl;
	unsigned int id = pl_bridge->id;

	regmap_update_bits(pl->regmap, CTRL, PL_VALID(id), PL_VALID(id));
	regmap_update_bits(pl->regmap, CTRL, PL_ENABLE(id), PL_ENABLE(id));
}

static u32 *
imx95_pl_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
					  struct drm_bridge_state *bridge_state,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state,
					  u32 output_fmt,
					  unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	if (output_fmt != MEDIA_BUS_FMT_RGB888_1X36_CPADLO &&
	    output_fmt != MEDIA_BUS_FMT_FIXED)
		return NULL;

	*num_input_fmts = 1;

	input_fmts = kmalloc(sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;

	return input_fmts;
}

static const struct drm_bridge_funcs imx95_pl_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		= drm_atomic_helper_bridge_reset,
	.attach			= imx95_pl_bridge_attach,
	.disable		= imx95_pl_bridge_disable,
	.enable			= imx95_pl_bridge_enable,
	.atomic_get_input_bus_fmts =
				imx95_pl_bridge_atomic_get_input_bus_fmts,
};

static int
imx95_pl_find_next_bridge_per_output_port(struct imx95_pl *pl, u32 out_port)
{
	struct device *dev = pl->dev;
	struct device_node *np = dev->of_node;
	u32 in_port = out_port - STREAMS;
	struct device_node *remote;
	u32 ep;
	int i;

	remote = of_graph_get_remote_node(np, in_port, 0);
	if (!remote) {
		dev_dbg(dev, "no remote node for input port%u\n", in_port);
		return -ENODEV;
	}
	of_node_put(remote);

	for (i = 0; i < OUT_ENDPOINTS; i++) {
		ep = OUT_ENDPOINTS - 1 - i;

		remote = of_graph_get_remote_node(np, out_port, ep);
		if (!remote) {
			dev_dbg(dev, "no remote node for port%u ep%u\n",
				out_port, ep);
			continue;
		}

		pl->next_bridge[in_port] = of_drm_find_bridge(remote);
		if (!pl->next_bridge[in_port]) {
			dev_dbg(dev, "failed to find next bridge for port%u ep%u\n",
				out_port, ep);
			of_node_put(remote);
			return -EPROBE_DEFER;
		}

		of_node_put(remote);

		if (pl->next_bridge[in_port] == pl->next_bridge[in_port ^ 1]) {
			dev_err(dev, "do not support sharing next bridge(%pOF) between pixel links\n",
				remote);
			return -ENODEV;
		}

		return 0;
	}

	return -ENODEV;
}

static void imx95_pl_bridge_remove(struct imx95_pl *pl)
{
	struct imx95_pl_bridge *bridge;
	int i;

	for (i = 0; i < STREAMS; i++) {
		if (!pl->next_bridge[i])
			continue;

		bridge = &pl->bridge[i];
		drm_bridge_remove(&bridge->base);
	}
}

static int imx95_pl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *port;
	struct imx95_pl_bridge *bridge;
	u32 in_port, out_port;
	struct imx95_pl *pl;
	int ret;

	pl = devm_kzalloc(dev, sizeof(*pl), GFP_KERNEL);
	if (!pl)
		return -ENOMEM;

	pl->dev = dev;
	platform_set_drvdata(pdev, pl);

	pl->regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(pl->regmap))
		return dev_err_probe(dev, PTR_ERR(pl->regmap),
				     "failed to get regmap\n");

	for (in_port = 0; in_port < STREAMS; in_port++) {
		out_port = in_port + STREAMS;

		ret = imx95_pl_find_next_bridge_per_output_port(pl, out_port);
		if (ret == -ENODEV)
			continue;
		else if (ret)
			goto err;

		port = of_graph_get_port_by_id(np, in_port);
		if (!port) {
			dev_err(dev, "failed to get port@%u\n", in_port);
			ret = -ENODEV;
			goto err;
		}

		bridge = &pl->bridge[in_port];
		bridge->pl = pl;
		bridge->id = in_port;

		bridge->base.driver_private = bridge;
		bridge->base.funcs = &imx95_pl_bridge_funcs;
		bridge->base.of_node = port;

		of_node_put(port);

		drm_bridge_add(&bridge->base);
	}

	return 0;

err:
	imx95_pl_bridge_remove(pl);

	return ret;
}

static int imx95_pl_remove(struct platform_device *pdev)
{
	struct imx95_pl *pl = platform_get_drvdata(pdev);

	imx95_pl_bridge_remove(pl);

	return 0;
}

static const struct of_device_id imx95_pl_dt_ids[] = {
	{ .compatible = "nxp,imx95-dc-pixel-link", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx95_pl_dt_ids);

static struct platform_driver imx95_pl_driver = {
	.probe	= imx95_pl_probe,
	.remove	= imx95_pl_remove,
	.driver	= {
		.of_match_table = imx95_pl_dt_ids,
		.name = DRIVER_NAME,
	},
};

module_platform_driver(imx95_pl_driver);

MODULE_DESCRIPTION("i.MX95 pixel link driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
