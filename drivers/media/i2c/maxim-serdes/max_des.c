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

static inline struct max_des_asd *to_max_des_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct max_des_asd, base);
}

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
	struct max_des_subdev_priv *sd_priv;
	unsigned int i;

	pipe->num_remaps = 0;

	for_each_subdev(priv, sd_priv) {
		if (sd_priv->pipe_id != pipe->index)
			continue;

		for (i = 0; i < 3; i++) {
			struct max_des_dt_vc_remap *remap;
			unsigned int dt;

			if (pipe->num_remaps == MAX_DES_REMAPS_NUM) {
				dev_err(priv->dev, "Too many remaps\n");
				return -EINVAL;
			}

			remap = &pipe->remaps[pipe->num_remaps++];

			if (i == 0)
				dt = MAX_DT_FS;
			else if (i == 1)
				dt = MAX_DT_FE;
			else
				dt = sd_priv->dt;

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
	}

	return 0;
}

static int max_des_post_init(struct max_des_priv *priv)
{
	unsigned int i, mask;
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
	const struct max_format *fmt;

	if (format->pad != MAX_DES_SOURCE_PAD)
		return -EINVAL;

	fmt = max_format_by_dt(sd_priv->dt);
	if (!fmt)
		return -EINVAL;

	format->format.code = fmt->code;

	return 0;
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

	sd_priv->dt = fmt->dt;

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

static int max_des_v4l2_register_sd(struct max_des_subdev_priv *sd_priv)
{
	struct max_des_priv *priv = sd_priv->priv;
	unsigned int index = sd_priv->index;
	char postfix[3];
	int ret;

	snprintf(postfix, sizeof(postfix), ":%d", index);

	v4l2_i2c_subdev_init(&sd_priv->sd, priv->client, &max_des_subdev_ops);
	v4l2_i2c_subdev_set_name(&sd_priv->sd, priv->client, NULL, postfix);
	sd_priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd_priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd_priv->sd.fwnode = sd_priv->fwnode;

	sd_priv->pads[MAX_DES_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	sd_priv->pads[MAX_DES_SINK_PAD].flags = MEDIA_PAD_FL_SINK;

	ret = media_entity_pads_init(&sd_priv->sd.entity, MAX_DES_PAD_NUM, sd_priv->pads);
	if (ret)
		goto error;

	v4l2_set_subdevdata(&sd_priv->sd, sd_priv);

	return v4l2_async_register_subdev(&sd_priv->sd);

error:
	media_entity_cleanup(&sd_priv->sd.entity);
	fwnode_handle_put(sd_priv->sd.fwnode);

	return ret;
}

static void max_des_v4l2_unregister_sd(struct max_des_subdev_priv *sd_priv)
{
	v4l2_async_unregister_subdev(&sd_priv->sd);
	media_entity_cleanup(&sd_priv->sd.entity);
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
	phy->alt_mem_map8 = fwnode_property_read_bool(fwnode, "max,alt-mem-map8");
	phy->alt_mem_map10 = fwnode_property_read_bool(fwnode, "max,alt-mem-map10");
	phy->alt_mem_map12 = fwnode_property_read_bool(fwnode, "max,alt-mem-map12");

	return 0;
}

static int max_des_parse_pipe_link_remap_dt(struct max_des_priv *priv,
					    struct max_des_pipe *pipe,
					    struct fwnode_handle *fwnode)
{
	u32 val;
	int ret;

	val = pipe->link_id;
	ret = fwnode_property_read_u32(fwnode, "max,link-id", &val);
	if (!ret && priv->ops->supports_pipe_link_remap) {
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
	fwnode_property_read_u32(fwnode, "max,phy-id", &val);
	if (val >= priv->ops->num_phys) {
		dev_err(priv->dev, "Invalid PHY %u\n", val);
		return -EINVAL;
	}
	pipe->phy_id = val;

	/* TODO: implement auto select stream. */
	val = pipe->stream_id;
	fwnode_property_read_u32(fwnode, "max,stream-id", &val);
	if (val >= MAX_SERDES_STREAMS_NUM) {
		dev_err(priv->dev, "Invalid stream %u\n", val);
		return -EINVAL;
	}
	pipe->stream_id = val;

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

	/* TODO: implement extended Virtual Channel. */
	val = sd_priv->src_vc_id;
	fwnode_property_read_u32(fwnode, "max,src-vc-id", &val);
	if (val >= MAX_SERDES_VC_ID_NUM) {
		dev_err(priv->dev, "Invalid source virtual channel %u\n", val);
		return -EINVAL;
	}
	sd_priv->src_vc_id = val;

	/* TODO: implement extended Virtual Channel. */
	val = sd_priv->dst_vc_id;
	fwnode_property_read_u32(fwnode, "max,dst-vc-id", &val);
	if (val >= MAX_SERDES_VC_ID_NUM) {
		dev_err(priv->dev, "Invalid destination virtual channel %u\n", val);
		return -EINVAL;
	}
	sd_priv->dst_vc_id = val;

	val = sd_priv->pipe_id;
	fwnode_property_read_u32(fwnode, "max,pipe-id", &val);
	if (val >= priv->ops->num_pipes) {
		dev_err(priv->dev, "Invalid pipe %u\n", val);
		return -EINVAL;
	}
	sd_priv->pipe_id = val;

	pipe = &priv->pipes[val];
	pipe->enabled = true;

	val = pipe->phy_id;
	fwnode_property_read_u32(fwnode, "max,phy-id", &val);
	if (val >= priv->ops->num_phys) {
		dev_err(priv->dev, "Invalid PHY %u\n", val);
		return -EINVAL;
	}
	sd_priv->phy_id = val;

	phy = &priv->phys[val];
	phy->enabled = true;

	link = &priv->links[pipe->link_id];
	link->enabled = true;

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
	struct fwnode_handle *ep, *remote_ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, MAX_DES_SOURCE_PAD, 0, 0);
	if (!ep) {
		dev_err(priv->dev, "Not connected to subdevice\n");
		return -EINVAL;
	}

	remote_ep = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);
	if (!remote_ep) {
		dev_err(priv->dev, "Not connected to remote endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(remote_ep, &v4l2_ep);
	fwnode_handle_put(remote_ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse v4l2 endpoint\n");
		return ret;
	}

	/* TODO: check the rest of the MIPI configuration. */
	if (phy->mipi.num_data_lanes && phy->mipi.num_data_lanes !=
	    v4l2_ep.bus.mipi_csi2.num_data_lanes) {
		dev_err(priv->dev, "PHY configured with differing number of data lanes\n");
		return -EINVAL;
	}

	phy->mipi = v4l2_ep.bus.mipi_csi2;

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
	int ret;

	for (i = 0; i < priv->ops->num_phys; i++) {
		phy = &priv->phys[i];
		phy->index = i;
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		pipe = &priv->pipes[i];
		pipe->index = i;
		pipe->phy_id = i;
		pipe->stream_id = i;
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

		if (index >= priv->ops->num_pipes) {
			dev_err(priv->dev, "Invalid phy %u\n", index);
			return -EINVAL;
		}

		phy = &priv->phys[index];

		ret = max_des_parse_phy_dt(priv, phy, fwnode);
		if (ret)
			return ret;
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
			return -EINVAL;
		}

		pipe = &priv->pipes[index];

		ret = max_des_parse_pipe_dt(priv, pipe, fwnode);
		if (ret)
			return ret;
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

		if (index + 1 < priv->num_subdevs)
			continue;

		priv->num_subdevs = index + 1;
	}

	priv->sd_privs = devm_kcalloc(priv->dev, priv->num_subdevs,
				      sizeof(*priv->sd_privs), GFP_KERNEL);
	if (!priv->sd_privs)
		return -ENOMEM;

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, channel_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		sd_priv = &priv->sd_privs[index];
		sd_priv->fwnode = fwnode;
		sd_priv->priv = priv;
		sd_priv->index = index;
		sd_priv->src_vc_id = 0;
		sd_priv->dst_vc_id = index;
		sd_priv->pipe_id = index;

		ret = max_des_parse_ch_dt(sd_priv, fwnode);
		if (ret)
			return ret;

		ret = max_des_parse_src_dt_endpoint(sd_priv, fwnode);
		if (ret)
			return ret;
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
