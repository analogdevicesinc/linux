// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Deserializer Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include "max_des.h"
#include "max_ser.h"
#include "max_serdes.h"

#define MAX_DES_LINK_FREQUENCY_MIN		100000000ull
#define MAX_DES_LINK_FREQUENCY_DEFAULT		750000000ull
#define MAX_DES_LINK_FREQUENCY_MAX		1250000000ull

const struct regmap_config max_des_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1f00,
};

static struct max_des_subdev_priv *next_subdev(struct max_des_priv *priv,
					       struct max_des_subdev_priv *sd_priv)
{
	if (!sd_priv)
		sd_priv = &priv->sd_privs[0];
	else
		sd_priv++;

	for (; sd_priv < priv->sd_privs + priv->num_subdevs; sd_priv++) {
		if (sd_priv->fwnode)
			return sd_priv;
	}

	return NULL;
}

#define for_each_subdev(priv, sd_priv) \
	for ((sd_priv) = NULL; ((sd_priv) = next_subdev((priv), (sd_priv))); )

static inline struct max_des_subdev_priv *sd_to_max_des(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_des_subdev_priv, sd);
}

static int __max_des_mipi_update(struct max_des_priv *priv)
{
	struct max_des_subdev_priv *sd_priv;
	bool enable = 0;

	for_each_subdev(priv, sd_priv) {
		if (sd_priv->active) {
			enable = 1;
			break;
		}
	}

	if (enable == priv->active)
		return 0;

	priv->active = enable;

	return priv->ops->mipi_enable(priv, enable);
}

static int max_des_ch_enable(struct max_des_subdev_priv *sd_priv, bool enable)
{
	struct max_des_priv *priv = sd_priv->priv;
	int ret = 0;

	mutex_lock(&priv->lock);

	if (sd_priv->active == enable)
		goto exit;

	sd_priv->active = enable;

	ret = __max_des_mipi_update(priv);

exit:
	mutex_unlock(&priv->lock);

	return ret;
}

static int max_des_update_pipe_remaps(struct max_des_priv *priv,
				      struct max_des_pipe *pipe)
{
	struct max_des_link *link = &priv->links[pipe->link_id];
	struct max_des_subdev_priv *sd_priv;
	unsigned int i;

	pipe->num_remaps = 0;

	if (link->tunnel_mode)
		return 0;

	for_each_subdev(priv, sd_priv) {
		unsigned int num_remaps;

		if (sd_priv->pipe_id != pipe->index)
			continue;

		if (!sd_priv->fmt)
			continue;

		if (sd_priv->fmt->dt == MAX_DT_EMB8)
			num_remaps = 1;
		else
			num_remaps = 3;

		for (i = 0; i < num_remaps; i++) {
			struct max_des_dt_vc_remap *remap;
			unsigned int dt;

			if (pipe->num_remaps == MAX_DES_REMAPS_NUM) {
				dev_err(priv->dev, "Too many remaps\n");
				return -EINVAL;
			}

			remap = &pipe->remaps[pipe->num_remaps++];

			if (i == 0)
				dt = sd_priv->fmt->dt;
			else if (i == 1)
				dt = MAX_DT_FS;
			else
				dt = MAX_DT_FE;

			remap->from_dt = dt;
			remap->from_vc = sd_priv->src_vc_id;
			remap->to_dt = dt;
			remap->to_vc = sd_priv->dst_vc_id;
			remap->phy = sd_priv->phy_id;
		}
	}

	return priv->ops->update_pipe_remaps(priv, pipe);
}

static int max_des_init_link_ser_xlate(struct max_des_priv *priv,
				       struct max_des_link *link,
				       u8 power_up_addr, u8 new_addr)
{
	u8 addrs[] = { power_up_addr, new_addr };
	struct i2c_client *client;
	struct regmap *regmap;
	int ret;

	client = i2c_new_dummy_device(priv->client->adapter, power_up_addr);
	if (IS_ERR(client)) {
		ret = PTR_ERR(client);
		dev_err(priv->dev,
			"Failed to create I2C client: %d\n", ret);
		return ret;
	}

	regmap = regmap_init_i2c(client, &max_ser_i2c_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(priv->dev,
			"Failed to create I2C regmap: %d\n", ret);
		goto err_unregister_client;
	}

	ret = priv->ops->select_links(priv, BIT(link->index));
	if (ret)
		goto err_regmap_exit;

	ret = max_ser_wait_for_multiple(client, regmap, addrs, ARRAY_SIZE(addrs));
	if (ret) {
		dev_err(priv->dev,
			"Failed waiting for serializer with new or old address: %d\n", ret);
		goto err_regmap_exit;
	}

	ret = max_ser_reset(regmap);
	if (ret) {
		dev_err(priv->dev, "Failed to reset serializer: %d\n", ret);
		goto err_regmap_exit;
	}

	ret = max_ser_wait(client, regmap, power_up_addr);
	if (ret) {
		dev_err(priv->dev,
			"Failed waiting for serializer with new address: %d\n", ret);
		goto err_regmap_exit;
	}

	ret = max_ser_change_address(client, regmap, new_addr, priv->ops->fix_tx_ids);
	if (ret) {
		dev_err(priv->dev, "Failed to change serializer address: %d\n", ret);
		goto err_regmap_exit;
	}

err_regmap_exit:
	regmap_exit(regmap);

err_unregister_client:
	i2c_unregister_device(client);

	return ret;
}

static int max_des_init(struct max_des_priv *priv)
{
	unsigned int i;
	int ret;

	ret = __max_des_mipi_update(priv);
	if (ret)
		return ret;

	ret = priv->ops->init(priv);
	if (ret)
		return ret;

	for (i = 0; i < priv->ops->num_phys; i++) {
		struct max_des_phy *phy = &priv->phys[i];

		if (!phy->enabled)
			continue;

		if (!phy->bus_config_parsed) {
			dev_err(priv->dev, "Cannot turn on unconfigured PHY\n");
			return -EINVAL;
		}

		ret = priv->ops->init_phy(priv, phy);
		if (ret)
			return ret;
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &priv->pipes[i];

		if (!pipe->enabled)
			continue;

		ret = priv->ops->init_pipe(priv, pipe);
		if (ret)
			return ret;

		ret = max_des_update_pipe_remaps(priv, pipe);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_post_init(struct max_des_priv *priv)
{
	unsigned int i, mask = 0;
	int ret;

	for (i = 0; i < priv->ops->num_links; i++) {
		struct max_des_link *link = &priv->links[i];

		if (!link->enabled)
			continue;

		mask |= BIT(link->index);
	}

	ret = priv->ops->select_links(priv, mask);
	if (ret)
		return ret;

	if (priv->ops->post_init) {
		ret = priv->ops->post_init(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_ser_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
					 const struct i2c_client *client, u16 alias)
{
	struct max_des_priv *priv = i2c_atr_get_driver_data(atr);
	struct max_des_link *link = &priv->links[chan_id];

	if (link->ser_xlate_enabled) {
		dev_err(priv->dev, "Serializer for link %u already bound\n", link->index);
		return -EINVAL;
	}

	link->ser_xlate.src = alias;
	link->ser_xlate.dst = client->addr;
	link->ser_xlate_enabled = true;

	return max_des_init_link_ser_xlate(priv, link, client->addr, alias);
}

static void max_des_ser_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
					  const struct i2c_client *client)
{
	/* Don't do anything. */
}

static const struct i2c_atr_ops max_des_i2c_atr_ops = {
	.attach_client = max_des_ser_atr_attach_client,
	.detach_client = max_des_ser_atr_detach_client,
};

static void max_des_i2c_atr_deinit(struct max_des_priv *priv)
{
	unsigned int i;

	for (i = 0; i < priv->ops->num_links; i++) {
		struct max_des_link *link = &priv->links[i];

		/* Deleting adapters that haven't been added does no harm. */
		i2c_atr_del_adapter(priv->atr, link->index);
	}

	i2c_atr_delete(priv->atr);
}

static int max_des_i2c_atr_init(struct max_des_priv *priv)
{
	unsigned int i;
	int ret;

	if (!i2c_check_functionality(priv->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	priv->atr = i2c_atr_new(priv->client->adapter, priv->dev,
				&max_des_i2c_atr_ops, priv->ops->num_links);
	if (!priv->atr)
		return -ENOMEM;

	i2c_atr_set_driver_data(priv->atr, priv);

	for (i = 0; i < priv->ops->num_links; i++) {
		struct max_des_link *link = &priv->links[i];

		if (!link->enabled)
			continue;

		ret = i2c_atr_add_adapter(priv->atr, link->index, NULL, NULL);
		if (ret)
			goto err_add_adapters;
	}

	return 0;

err_add_adapters:
	max_des_i2c_atr_deinit(priv);

	return ret;
}

static int max_des_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max_des_subdev_priv *sd_priv = sd_to_max_des(sd);

	return max_des_ch_enable(sd_priv, enable);
}

static int max_des_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct max_des_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);

	if (format->pad == MAX_SER_SINK_PAD) {
		format->format.code = MEDIA_BUS_FMT_FIXED;
		return 0;
	}

	if (!sd_priv->fmt)
		return -EINVAL;

	format->format.code = sd_priv->fmt->code;

	return 0;
}

static u64 max_des_get_pixel_rate(struct max_des_subdev_priv *sd_priv)
{
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_phy *phy = &priv->phys[sd_priv->phy_id];
	u8 bpp = 8;

	if (sd_priv->fmt)
		bpp = sd_priv->fmt->bpp;

	return phy->link_frequency * 2 * phy->mipi.num_data_lanes / bpp;
}

static int max_des_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct max_des_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_pipe *pipe = &priv->pipes[sd_priv->pipe_id];
	const struct max_format *fmt;
	int ret;

	if (format->pad != MAX_DES_SOURCE_PAD)
		return -EINVAL;

	fmt = max_format_by_code(format->format.code);
	if (!fmt)
		return -EINVAL;

	sd_priv->fmt = fmt;

	v4l2_ctrl_s_ctrl_int64(sd_priv->pixel_rate_ctrl,
			       max_des_get_pixel_rate(sd_priv));

	mutex_lock(&priv->lock);

	ret = max_des_update_pipe_remaps(priv, pipe);

	mutex_unlock(&priv->lock);

	return ret;
}

static int max_des_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	const struct max_format *fmt;

	if (code->pad != MAX_DES_SOURCE_PAD)
		return -EINVAL;

	fmt = max_format_by_index(code->index);
	if (!fmt)
		return -EINVAL;

	code->code = fmt->code;

	return 0;
}

static int max_des_log_status(struct v4l2_subdev *sd)
{
	struct max_des_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_des_priv *priv = sd_priv->priv;
	unsigned int i, j;
	int ret;

	v4l2_info(sd, "active: %u\n", priv->active);
	v4l2_info(sd, "pipe_stream_autoselect: %u\n", priv->pipe_stream_autoselect);
	if (priv->ops->log_status) {
		ret = priv->ops->log_status(priv, sd->name);
		if (ret)
			return ret;
	}
	v4l2_info(sd, "\n");

	for (i = 0; i < priv->ops->num_links; i++) {
		struct max_des_link *link = &priv->links[i];

		v4l2_info(sd, "link: %u\n", link->index);
		v4l2_info(sd, "\tenabled: %u\n", link->enabled);
		v4l2_info(sd, "\ttunnel_mode: %u\n", link->tunnel_mode);
		v4l2_info(sd, "\tser_xlate_enabled: %u\n", link->ser_xlate_enabled);
		v4l2_info(sd, "\tser_xlate: src: 0x%02x dst: 0x%02x\n",
			  link->ser_xlate.src, link->ser_xlate.dst);
		v4l2_info(sd, "\n");
	}

	for_each_subdev(priv, sd_priv) {
		v4l2_info(sd, "channel: %u\n", sd_priv->index);
		v4l2_info(sd, "\tfwnode: %pfw\n", sd_priv->fwnode);
		v4l2_info(sd, "\tlabel: %s\n", sd_priv->label);
		v4l2_info(sd, "\tactive: %u\n", sd_priv->active);
		v4l2_info(sd, "\tfmt: %s\n", sd_priv->fmt ? sd_priv->fmt->name : NULL);
		v4l2_info(sd, "\tdt: 0x%02x\n", sd_priv->fmt ? sd_priv->fmt->dt : 0);
		v4l2_info(sd, "\tpipe_id: %u\n", sd_priv->pipe_id);
		v4l2_info(sd, "\tphy_id: %u\n", sd_priv->phy_id);
		v4l2_info(sd, "\tsrc_vc_id: %u\n", sd_priv->src_vc_id);
		v4l2_info(sd, "\tdst_vc_id: %u\n", sd_priv->dst_vc_id);
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &priv->pipes[i];

		v4l2_info(sd, "pipe: %u\n", pipe->index);
		v4l2_info(sd, "\tenabled: %u\n", pipe->enabled);
		v4l2_info(sd, "\tphy_id: %u\n", pipe->phy_id);
		v4l2_info(sd, "\tstream_id: %u\n", pipe->stream_id);
		v4l2_info(sd, "\tlink_id: %u\n", pipe->link_id);
		v4l2_info(sd, "\tdbl8: %u\n", pipe->dbl8);
		v4l2_info(sd, "\tdbl8mode: %u\n", pipe->dbl8mode);
		v4l2_info(sd, "\tdbl10: %u\n", pipe->dbl10);
		v4l2_info(sd, "\tdbl10mode: %u\n", pipe->dbl10mode);
		v4l2_info(sd, "\tdbl12: %u\n", pipe->dbl12);
		v4l2_info(sd, "\tremaps: %u\n", pipe->num_remaps);
		for (j = 0; j < pipe->num_remaps; j++) {
			struct max_des_dt_vc_remap *remap = &pipe->remaps[j];

			v4l2_info(sd, "\t\tremap: from: vc: %u, dt: 0x%02x\n",
				  remap->from_vc, remap->from_dt);
			v4l2_info(sd, "\t\t       to:   vc: %u, dt: 0x%02x, phy: %u\n",
				  remap->to_vc, remap->to_dt, remap->phy);
		}
		if (priv->ops->log_pipe_status) {
			ret = priv->ops->log_pipe_status(priv, pipe, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < priv->ops->num_phys; i++) {
		struct max_des_phy *phy = &priv->phys[i];

		v4l2_info(sd, "phy: %u\n", phy->index);
		v4l2_info(sd, "\tenabled: %u\n", phy->enabled);
		v4l2_info(sd, "\tlink_frequency: %llu\n", phy->link_frequency);
		v4l2_info(sd, "\tnum_data_lanes: %u\n", phy->mipi.num_data_lanes);
		v4l2_info(sd, "\tclock_lane: %u\n", phy->mipi.clock_lane);
		v4l2_info(sd, "\talt_mem_map8: %u\n", phy->alt_mem_map8);
		v4l2_info(sd, "\talt2_mem_map8: %u\n", phy->alt2_mem_map8);
		v4l2_info(sd, "\talt_mem_map10: %u\n", phy->alt_mem_map10);
		v4l2_info(sd, "\talt_mem_map12: %u\n", phy->alt_mem_map12);
		if (priv->ops->log_phy_status) {
			ret = priv->ops->log_phy_status(priv, phy, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max_des_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct max_des_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_des_priv *priv = sd_priv->priv;
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int max_des_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct max_des_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_des_priv *priv = sd_priv->priv;
	int ret;

	ret = regmap_write(priv->regmap, reg->reg, reg->val);
	if (ret)
		return ret;

	return 0;
}
#endif

static const struct v4l2_subdev_core_ops max_des_core_ops = {
	.log_status = max_des_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = max_des_g_register,
	.s_register = max_des_s_register,
#endif
};

static const struct v4l2_subdev_video_ops max_des_video_ops = {
	.s_stream = max_des_s_stream,
};

static const struct v4l2_subdev_pad_ops max_des_pad_ops = {
	.get_fmt = max_des_get_fmt,
	.set_fmt = max_des_set_fmt,
	.enum_mbus_code = max_des_enum_mbus_code,
};

static const struct v4l2_subdev_ops max_des_subdev_ops = {
	.core = &max_des_core_ops,
	.video = &max_des_video_ops,
	.pad = &max_des_pad_ops,
};

static const struct media_entity_operations max_des_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static void max_des_set_sd_name(struct max_des_subdev_priv *sd_priv)
{
	struct max_des_priv *priv = sd_priv->priv;
	struct i2c_client *client = priv->client;

	if (sd_priv->label) {
		strscpy(sd_priv->sd.name, sd_priv->label, sizeof(sd_priv->sd.name));
		return;
	}

	snprintf(sd_priv->sd.name, sizeof(sd_priv->sd.name), "%s %d-%04x:%u",
		 client->dev.driver->name, i2c_adapter_id(client->adapter),
		 client->addr, sd_priv->index);
}

static int max_des_v4l2_register_sd(struct max_des_subdev_priv *sd_priv)
{
	struct v4l2_ctrl_handler *hdl = &sd_priv->ctrl_handler;
	u64 max_pixel_rate = max_des_get_pixel_rate(sd_priv);
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_phy *phy = &priv->phys[sd_priv->phy_id];
	int ret;

	v4l2_i2c_subdev_init(&sd_priv->sd, priv->client, &max_des_subdev_ops);
	max_des_set_sd_name(sd_priv);
	sd_priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd_priv->sd.entity.ops = &max_des_media_ops;
	sd_priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd_priv->sd.fwnode = sd_priv->fwnode;
	fwnode_handle_get(sd_priv->sd.fwnode);

	sd_priv->sd.ctrl_handler = hdl;
	ret = v4l2_ctrl_handler_init(hdl, 1);
	if (ret)
		goto error;

	v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0, &phy->link_frequency);
	sd_priv->pixel_rate_ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
						     0, max_pixel_rate,
						     1, max_pixel_rate);

	sd_priv->pads[MAX_DES_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	sd_priv->pads[MAX_DES_SINK_PAD].flags = MEDIA_PAD_FL_SINK;

	v4l2_set_subdevdata(&sd_priv->sd, sd_priv);

	ret = media_entity_pads_init(&sd_priv->sd.entity, MAX_DES_PAD_NUM, sd_priv->pads);
	if (ret)
		goto error;

	ret = v4l2_async_register_subdev(&sd_priv->sd);
	if (ret)
		goto error;

	return 0;

error:
	media_entity_cleanup(&sd_priv->sd.entity);
	v4l2_ctrl_handler_free(&sd_priv->ctrl_handler);
	fwnode_handle_put(sd_priv->sd.fwnode);

	return ret;
}

static void max_des_v4l2_unregister_sd(struct max_des_subdev_priv *sd_priv)
{
	v4l2_async_unregister_subdev(&sd_priv->sd);
	media_entity_cleanup(&sd_priv->sd.entity);
	v4l2_ctrl_handler_free(&sd_priv->ctrl_handler);
	fwnode_handle_put(sd_priv->sd.fwnode);
}

static int max_des_v4l2_register(struct max_des_priv *priv)
{
	struct max_des_subdev_priv *sd_priv;
	int ret;

	for_each_subdev(priv, sd_priv) {
		ret = max_des_v4l2_register_sd(sd_priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void max_des_v4l2_unregister(struct max_des_priv *priv)
{
	struct max_des_subdev_priv *sd_priv;

	for_each_subdev(priv, sd_priv)
		max_des_v4l2_unregister_sd(sd_priv);
}

static int max_des_parse_phy_dt(struct max_des_priv *priv,
				struct max_des_phy *phy,
				struct fwnode_handle *fwnode)
{
	phy->alt_mem_map8 = fwnode_property_read_bool(fwnode, "maxim,alt-mem-map8");
	phy->alt2_mem_map8 = fwnode_property_read_bool(fwnode, "maxim,alt2-mem-map8");
	phy->alt_mem_map10 = fwnode_property_read_bool(fwnode, "maxim,alt-mem-map10");
	phy->alt_mem_map12 = fwnode_property_read_bool(fwnode, "maxim,alt-mem-map12");

	return 0;
}

static int max_des_parse_pipe_link_remap_dt(struct max_des_priv *priv,
					    struct max_des_pipe *pipe,
					    struct fwnode_handle *fwnode)
{
	u32 val;
	int ret;

	val = pipe->link_id;
	ret = fwnode_property_read_u32(fwnode, "maxim,link-id", &val);
	if (!ret && !priv->ops->supports_pipe_link_remap) {
		dev_err(priv->dev, "Pipe link remapping is not supported\n");
		return -EINVAL;
	}

	if (val >= priv->ops->num_links) {
		dev_err(priv->dev, "Invalid link %u\n", val);
		return -EINVAL;
	}

	pipe->link_id = val;

	return 0;
}

static int max_des_parse_pipe_dt(struct max_des_priv *priv,
				 struct max_des_pipe *pipe,
				 struct fwnode_handle *fwnode)
{
	u32 val;
	int ret;

	val = pipe->phy_id;
	fwnode_property_read_u32(fwnode, "maxim,phy-id", &val);
	if (val >= priv->ops->num_phys) {
		dev_err(priv->dev, "Invalid PHY %u\n", val);
		return -EINVAL;
	}
	pipe->phy_id = val;

	val = pipe->stream_id;
	ret = fwnode_property_read_u32(fwnode, "maxim,stream-id", &val);
	if (!ret && priv->pipe_stream_autoselect) {
		dev_err(priv->dev, "Cannot select stream when using autoselect\n");
		return -EINVAL;
	}

	if (val >= MAX_SERDES_STREAMS_NUM) {
		dev_err(priv->dev, "Invalid stream %u\n", val);
		return -EINVAL;
	}

	pipe->stream_id = val;

	pipe->dbl8 = fwnode_property_read_bool(fwnode, "maxim,dbl8");
	pipe->dbl10 = fwnode_property_read_bool(fwnode, "maxim,dbl10");
	pipe->dbl12 = fwnode_property_read_bool(fwnode, "maxim,dbl12");

	pipe->dbl8mode = fwnode_property_read_bool(fwnode, "maxim,dbl8-mode");
	pipe->dbl10mode = fwnode_property_read_bool(fwnode, "maxim,dbl10-mode");

	ret = max_des_parse_pipe_link_remap_dt(priv, pipe, fwnode);
	if (ret)
		return ret;

	return 0;
}

static int max_des_parse_ch_dt(struct max_des_subdev_priv *sd_priv,
			       struct fwnode_handle *fwnode)
{
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_pipe *pipe;
	struct max_des_link *link;
	struct max_des_phy *phy;
	u32 val;

	fwnode_property_read_string(fwnode, "label", &sd_priv->label);

	/* TODO: implement extended Virtual Channel. */
	val = sd_priv->src_vc_id;
	fwnode_property_read_u32(fwnode, "maxim,src-vc-id", &val);
	if (val >= MAX_SERDES_VC_ID_NUM) {
		dev_err(priv->dev, "Invalid source virtual channel %u\n", val);
		return -EINVAL;
	}
	sd_priv->src_vc_id = val;

	/* TODO: implement extended Virtual Channel. */
	val = sd_priv->dst_vc_id;
	fwnode_property_read_u32(fwnode, "maxim,dst-vc-id", &val);
	if (val >= MAX_SERDES_VC_ID_NUM) {
		dev_err(priv->dev, "Invalid destination virtual channel %u\n", val);
		return -EINVAL;
	}
	sd_priv->dst_vc_id = val;

	val = sd_priv->pipe_id;
	fwnode_property_read_u32(fwnode, "maxim,pipe-id", &val);
	if (val >= priv->ops->num_pipes) {
		dev_err(priv->dev, "Invalid pipe %u\n", val);
		return -EINVAL;
	}
	sd_priv->pipe_id = val;

	pipe = &priv->pipes[val];
	pipe->enabled = true;

	val = pipe->phy_id;
	fwnode_property_read_u32(fwnode, "maxim,phy-id", &val);
	if (val >= priv->ops->num_phys) {
		dev_err(priv->dev, "Invalid PHY %u\n", val);
		return -EINVAL;
	}
	sd_priv->phy_id = val;

	if (fwnode_property_read_bool(fwnode, "maxim,embedded-data"))
		sd_priv->fmt = max_format_by_dt(MAX_DT_EMB8);

	phy = &priv->phys[val];
	phy->enabled = true;

	link = &priv->links[pipe->link_id];
	link->enabled = true;

	return 0;
}

static int max_des_parse_sink_dt_endpoint(struct max_des_subdev_priv *sd_priv,
					  struct fwnode_handle *fwnode)
{
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_pipe *pipe = &priv->pipes[sd_priv->pipe_id];
	struct max_des_link *link = &priv->links[pipe->link_id];
	struct fwnode_handle *ep, *channel_fwnode, *device_fwnode;
	u32 val;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, MAX_DES_SINK_PAD, 0, 0);
	if (!ep) {
		dev_err(priv->dev, "Not connected to subdevice\n");
		return -EINVAL;
	}

	channel_fwnode = fwnode_graph_get_remote_port_parent(ep);
	fwnode_handle_put(ep);
	if (!channel_fwnode) {
		dev_err(priv->dev, "Not connected to remote subdevice\n");
		return -EINVAL;
	}

	device_fwnode = fwnode_get_parent(channel_fwnode);
	fwnode_handle_put(channel_fwnode);
	if (!device_fwnode) {
		dev_err(priv->dev, "Not connected to remote subdevice\n");
		return -EINVAL;
	}

	val = fwnode_property_read_bool(device_fwnode, "maxim,tunnel-mode");
	fwnode_handle_put(device_fwnode);
	if (val && !priv->ops->supports_tunnel_mode) {
		dev_err(priv->dev, "Tunnel mode is not supported\n");
		return -EINVAL;
	}
	link->tunnel_mode = val;

	return 0;
}

static int max_des_parse_src_dt_endpoint(struct max_des_subdev_priv *sd_priv,
					 struct fwnode_handle *fwnode)
{
	struct max_des_priv *priv = sd_priv->priv;
	struct max_des_phy *phy = &priv->phys[sd_priv->phy_id];
	struct v4l2_fwnode_endpoint v4l2_ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct v4l2_fwnode_bus_mipi_csi2 *mipi = &v4l2_ep.bus.mipi_csi2;
	struct fwnode_handle *ep;
	u64 link_frequency;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, MAX_DES_SOURCE_PAD, 0, 0);
	if (!ep) {
		fwnode_handle_put(ep);
		return 0;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &v4l2_ep);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse v4l2 endpoint\n");
		return ret;
	}

	ret = 0;
	if (v4l2_ep.nr_of_link_frequencies == 0)
		link_frequency = MAX_DES_LINK_FREQUENCY_DEFAULT;
	else if (v4l2_ep.nr_of_link_frequencies == 1)
		link_frequency = v4l2_ep.link_frequencies[0];
	else
		ret = -EINVAL;

	v4l2_fwnode_endpoint_free(&v4l2_ep);

	if (ret) {
		dev_err(priv->dev, "PHY configured with invalid number of link frequencies\n");
		return -EINVAL;
	}

	if (link_frequency < MAX_DES_LINK_FREQUENCY_MIN ||
	    link_frequency > MAX_DES_LINK_FREQUENCY_MAX) {
		dev_err(priv->dev, "PHY configured with out of range link frequency\n");
		return -EINVAL;
	}

	for (i = 0; i < mipi->num_data_lanes; i++) {
		if (mipi->data_lanes[i] > mipi->num_data_lanes) {
			dev_err(priv->dev, "PHY configured with data lanes out of range\n");
			return -EINVAL;
		}
	}

	if (!phy->bus_config_parsed) {
		phy->mipi = v4l2_ep.bus.mipi_csi2;
		phy->link_frequency = link_frequency;
		phy->bus_config_parsed = true;

		return 0;
	}

	if (phy->link_frequency != link_frequency) {
		dev_err(priv->dev, "PHY configured with differing link frequency\n");
		return -EINVAL;
	}

	if (phy->mipi.num_data_lanes != mipi->num_data_lanes) {
		dev_err(priv->dev, "PHY configured with differing number of data lanes\n");
		return -EINVAL;
	}

	for (i = 0; i < mipi->num_data_lanes; i++) {
		if (phy->mipi.data_lanes[i] != mipi->data_lanes[i]) {
			dev_err(priv->dev, "PHY configured with differing data lanes\n");
			return -EINVAL;
		}
	}

	if (phy->mipi.clock_lane != mipi->clock_lane) {
		dev_err(priv->dev, "PHY configured with differing clock lane\n");
		return -EINVAL;
	}

	return 0;
}

static int max_des_parse_dt(struct max_des_priv *priv)
{
	const char *channel_node_name = "channel";
	const char *pipe_node_name = "pipe";
	const char *phy_node_name = "phy";
	struct max_des_subdev_priv *sd_priv;
	struct fwnode_handle *fwnode;
	struct max_des_link *link;
	struct max_des_pipe *pipe;
	struct max_des_phy *phy;
	unsigned int i;
	u32 index;
	u32 val;
	int ret;

	val = device_property_read_bool(priv->dev, "maxim,pipe-stream-autoselect");
	if (val && !priv->ops->supports_pipe_stream_autoselect) {
		dev_err(priv->dev, "Pipe stream autoselect is not supported\n");
		return -EINVAL;
	}
	priv->pipe_stream_autoselect = val;

	for (i = 0; i < priv->ops->num_phys; i++) {
		phy = &priv->phys[i];
		phy->index = i;
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		pipe = &priv->pipes[i];
		pipe->index = i;
		pipe->phy_id = i % priv->ops->num_phys;
		pipe->stream_id = i % MAX_SERDES_STREAMS_NUM;
		pipe->link_id = i;
	}

	for (i = 0; i < priv->ops->num_links; i++) {
		link = &priv->links[i];
		link->index = i;
	}

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, phy_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		if (index >= priv->ops->num_phys) {
			dev_err(priv->dev, "Invalid PHY %u\n", index);
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		phy = &priv->phys[index];

		ret = max_des_parse_phy_dt(priv, phy, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, pipe_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		if (index >= priv->ops->num_pipes) {
			dev_err(priv->dev, "Invalid pipe %u\n", index);
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		pipe = &priv->pipes[index];

		ret = max_des_parse_pipe_dt(priv, pipe, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, channel_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		priv->num_subdevs++;
	}

	priv->sd_privs = devm_kcalloc(priv->dev, priv->num_subdevs,
				      sizeof(*priv->sd_privs), GFP_KERNEL);
	if (!priv->sd_privs)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, channel_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		sd_priv = &priv->sd_privs[i++];
		sd_priv->fwnode = fwnode;
		sd_priv->priv = priv;
		sd_priv->index = index;
		sd_priv->src_vc_id = 0;
		sd_priv->dst_vc_id = index % MAX_SERDES_VC_ID_NUM;
		sd_priv->pipe_id = index % priv->ops->num_pipes;

		ret = max_des_parse_ch_dt(sd_priv, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		ret = max_des_parse_sink_dt_endpoint(sd_priv, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		ret = max_des_parse_src_dt_endpoint(sd_priv, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	return 0;
}

static int max_des_allocate(struct max_des_priv *priv)
{
	priv->phys = devm_kcalloc(priv->dev, priv->ops->num_phys,
				  sizeof(*priv->phys), GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	priv->pipes = devm_kcalloc(priv->dev, priv->ops->num_pipes,
				   sizeof(*priv->pipes), GFP_KERNEL);
	if (!priv->pipes)
		return -ENOMEM;

	priv->links = devm_kcalloc(priv->dev, priv->ops->num_links,
				   sizeof(*priv->links), GFP_KERNEL);
	if (!priv->links)
		return -ENOMEM;

	return 0;
}

int max_des_probe(struct max_des_priv *priv)
{
	int ret;

	mutex_init(&priv->lock);

	ret = max_des_allocate(priv);
	if (ret)
		return ret;

	ret = max_des_parse_dt(priv);
	if (ret)
		return ret;

	ret = max_des_init(priv);
	if (ret)
		return ret;

	ret = max_des_i2c_atr_init(priv);
	if (ret)
		return ret;

	ret = max_des_post_init(priv);
	if (ret)
		return ret;

	return max_des_v4l2_register(priv);
}
EXPORT_SYMBOL_GPL(max_des_probe);

int max_des_remove(struct max_des_priv *priv)
{
	max_des_v4l2_unregister(priv);

	max_des_i2c_atr_deinit(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(max_des_remove);

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(I2C_ATR);
