// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 * Copyright 2020 NXP
 */

#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <drm/bridge/fsl_imx_ldb.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#define LDB_CH0_MODE_EN_TO_DI0		(1 << 0)
#define LDB_CH0_MODE_EN_TO_DI1		(3 << 0)
#define LDB_CH0_MODE_EN_MASK		(3 << 0)
#define LDB_CH1_MODE_EN_TO_DI0		(1 << 2)
#define LDB_CH1_MODE_EN_TO_DI1		(3 << 2)
#define LDB_CH1_MODE_EN_MASK		(3 << 2)
#define LDB_SPLIT_MODE_EN		(1 << 4)
#define LDB_DATA_WIDTH_CH0_24		(1 << 5)
#define LDB_BIT_MAP_CH0_JEIDA		(1 << 6)
#define LDB_DATA_WIDTH_CH1_24		(1 << 7)
#define LDB_BIT_MAP_CH1_JEIDA		(1 << 8)
#define LDB_DI0_VS_POL_ACT_LOW		(1 << 9)
#define LDB_DI1_VS_POL_ACT_LOW		(1 << 10)

struct ldb_bit_mapping {
	u32 bus_format;
	u32 datawidth;
	const char * const mapping;
};

static const struct ldb_bit_mapping ldb_bit_mappings[] = {
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,  18, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,  24, "spwg" },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA, 24, "jeida" },
};

static u32 of_get_bus_format(struct device *dev, struct device_node *np)
{
	const char *bm;
	u32 datawidth = 0;
	int ret, i;

	ret = of_property_read_string(np, "fsl,data-mapping", &bm);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "fsl,data-width", &datawidth);

	for (i = 0; i < ARRAY_SIZE(ldb_bit_mappings); i++) {
		if (!strcasecmp(bm, ldb_bit_mappings[i].mapping) &&
		    datawidth == ldb_bit_mappings[i].datawidth)
			return ldb_bit_mappings[i].bus_format;
	}

	dev_err(dev, "invalid data mapping: %d-bit \"%s\"\n", datawidth, bm);

	return -ENOENT;
}

static inline struct ldb_channel *bridge_to_ldb_ch(struct drm_bridge *b)
{
	return container_of(b, struct ldb_channel, bridge);
}

static void ldb_ch_set_bus_format(struct ldb_channel *ldb_ch, u32 bus_format)
{
	struct ldb *ldb = ldb_ch->ldb;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		if (ldb_ch->chno == 0 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH0_24 |
					 LDB_BIT_MAP_CH0_JEIDA;
		if (ldb_ch->chno == 1 || ldb->dual)
			ldb->ldb_ctrl |= LDB_DATA_WIDTH_CH1_24 |
					 LDB_BIT_MAP_CH1_JEIDA;
		break;
	}
}

static void ldb_bridge_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *mode,
				const struct drm_display_mode *adjusted_mode)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	/* FIXME - assumes straight connections DI0 --> CH0, DI1 --> CH1 */
	if (ldb_ch == ldb->channel[0] || ldb->dual) {
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI0_VS_POL_ACT_LOW;
		else if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI0_VS_POL_ACT_LOW;
	}
	if (ldb_ch == ldb->channel[1] || ldb->dual) {
		if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC)
			ldb->ldb_ctrl |= LDB_DI1_VS_POL_ACT_LOW;
		else if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			ldb->ldb_ctrl &= ~LDB_DI1_VS_POL_ACT_LOW;
	}

	ldb_ch_set_bus_format(ldb_ch, ldb_ch->bus_format);
}

static void ldb_bridge_enable(struct drm_bridge *bridge)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	if (pm_runtime_enabled(ldb->dev))
		pm_runtime_get_sync(ldb->dev);

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);
}

static void ldb_bridge_disable(struct drm_bridge *bridge)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	if (ldb_ch == ldb->channel[0] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH0_MODE_EN_MASK;
	if (ldb_ch == ldb->channel[1] || ldb->dual)
		ldb->ldb_ctrl &= ~LDB_CH1_MODE_EN_MASK;

	regmap_write(ldb->regmap, ldb->ctrl_reg, ldb->ldb_ctrl);

	if (pm_runtime_enabled(ldb->dev))
		pm_runtime_put(ldb->dev);
}

static int ldb_bridge_attach(struct drm_bridge *bridge,
			     enum drm_bridge_attach_flags flags)
{
	struct ldb_channel *ldb_ch = bridge_to_ldb_ch(bridge);
	struct ldb *ldb = ldb_ch->ldb;

	if (!bridge->encoder) {
		dev_err(ldb->dev, "failed to find encoder object\n");
		return -ENODEV;
	}

	if (!ldb_ch->next_bridge)
		return 0;

	return drm_bridge_attach(bridge->encoder,
				ldb_ch->next_bridge, &ldb_ch->bridge, flags);
}

static int ldb_bridge_atomic_check(struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct drm_display_info *di = &conn_state->connector->display_info;
	struct drm_bridge_state *next_bridge_state = NULL;
	struct drm_bridge *next_bridge;
	u32 bus_flags = 0;

	next_bridge = drm_bridge_get_next_bridge(bridge);
	if (next_bridge)
		next_bridge_state = drm_atomic_get_new_bridge_state(crtc_state->state,
								    next_bridge);

	if (next_bridge_state)
		bus_flags = next_bridge_state->input_bus_cfg.flags;
	else if (di->num_bus_formats)
		bus_flags = di->bus_flags;

	bridge_state->output_bus_cfg.flags = bus_flags;
	bridge_state->input_bus_cfg.flags = bus_flags;

	return 0;
}

static u32 *ldb_bridge_atomic_get_output_bus_fmts(struct drm_bridge *bridge,
						  struct drm_bridge_state *bridge_state,
						  struct drm_crtc_state *crtc_state,
						  struct drm_connector_state *conn_state,
						  unsigned int *num_output_fmts)
{
	u32 *output_fmts;
	int i;

	*num_output_fmts = 0;

	output_fmts = kcalloc(ARRAY_SIZE(ldb_bit_mappings),
			      sizeof(*output_fmts), GFP_KERNEL);
	if (!output_fmts)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(ldb_bit_mappings); i++)
		output_fmts[i] = ldb_bit_mappings[i].bus_format;

	*num_output_fmts = ARRAY_SIZE(ldb_bit_mappings);

	return output_fmts;
}

static const struct drm_bridge_funcs ldb_bridge_funcs = {
	.mode_set		    = ldb_bridge_mode_set,
	.atomic_duplicate_state     = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	    = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset		    = drm_atomic_helper_bridge_reset,
	.enable			    = ldb_bridge_enable,
	.disable		    = ldb_bridge_disable,
	.attach			    = ldb_bridge_attach,
	.atomic_check		    = ldb_bridge_atomic_check,
	.atomic_get_output_bus_fmts = ldb_bridge_atomic_get_output_bus_fmts,
};

int ldb_bind(struct ldb *ldb, struct drm_encoder **encoder)
{
	struct device *dev = ldb->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	int ret = 0;
	int i;

	ldb->regmap = syscon_regmap_lookup_by_phandle(np, "gpr");
	if (IS_ERR(ldb->regmap)) {
		dev_err(dev, "failed to get parent regmap\n");
		return PTR_ERR(ldb->regmap);
	}

	if (pm_runtime_enabled(dev))
		pm_runtime_get_sync(dev);

	/* disable LDB by resetting the control register to POR default */
	regmap_write(ldb->regmap, ldb->ctrl_reg, 0);

	if (pm_runtime_enabled(dev))
		pm_runtime_put(dev);

	ldb->dual = of_property_read_bool(np, "fsl,dual-channel");
	if (ldb->dual)
		ldb->ldb_ctrl |= LDB_SPLIT_MODE_EN;

	for_each_child_of_node(np, child) {
		struct ldb_channel *ldb_ch;
		int bus_format;

		ret = of_property_read_u32(child, "reg", &i);
		if (ret || i < 0 || i > 1) {
			ret = -EINVAL;
			goto free_child;
		}

		if (!of_device_is_available(child))
			continue;

		if (ldb->dual && i > 0) {
			dev_warn(dev, "dual-channel mode, ignoring second output\n");
			continue;
		}

		ldb_ch = ldb->channel[i];
		ldb_ch->ldb = ldb;
		ldb_ch->chno = i;
		ldb_ch->is_valid = false;

		ret = drm_of_find_panel_or_bridge(child,
						  ldb->output_port, 0,
						  &ldb_ch->panel,
						  &ldb_ch->next_bridge);
		if (ret && ret != -ENODEV)
			goto free_child;

		bus_format = of_get_bus_format(dev, child);
		if (bus_format == -EINVAL) {
			/*
			 * If no bus format was specified in the device tree,
			 * we can still get it from the connected panel later.
			 */
			if (ldb_ch->panel && ldb_ch->panel->funcs &&
			    ldb_ch->panel->funcs->get_modes)
				bus_format = 0;
		}
		if (bus_format < 0) {
			dev_warn(dev, "No data-mapping in DT, will use negotiated bus format.\n");
			bus_format = 0;
		}
		ldb_ch->bus_format = bus_format;
		ldb_ch->child = child;

		if (ldb_ch->panel) {
			ldb_ch->next_bridge = devm_drm_panel_bridge_add(dev,
								ldb_ch->panel);
			if (IS_ERR(ldb_ch->next_bridge)) {
				ret = PTR_ERR(ldb_ch->next_bridge);
				goto free_child;
			}
		}

		ldb_ch->bridge.driver_private = ldb_ch;
		ldb_ch->bridge.funcs = &ldb_bridge_funcs;
		ldb_ch->bridge.of_node = child;

		ret = drm_bridge_attach(encoder[i], &ldb_ch->bridge, NULL, 0);
		if (ret) {
			dev_err(dev,
				"failed to attach bridge with encoder: %d\n",
				ret);
			goto free_child;
		}

		ldb_ch->is_valid = true;
	}

	return 0;

free_child:
	of_node_put(child);
	return ret;
}
EXPORT_SYMBOL_GPL(ldb_bind);

MODULE_DESCRIPTION("Freescale i.MX LVDS display bridge driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: fsl-imx-ldb");
