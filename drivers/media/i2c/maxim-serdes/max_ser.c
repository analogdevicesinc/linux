// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include "max_ser.h"

#include <linux/delay.h>
#include <linux/module.h>

#include "max_ser.h"
#include "max_serdes.h"

const struct regmap_config max_ser_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x1f00,
};
EXPORT_SYMBOL_GPL(max_ser_i2c_regmap);

static struct max_ser_subdev_priv *next_subdev(struct max_ser_priv *priv,
						struct max_ser_subdev_priv *sd_priv)
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

static inline struct max_ser_subdev_priv *sd_to_max_ser(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_ser_subdev_priv, sd);
}

static int max_ser_i2c_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
					 const struct i2c_client *client, u16 alias)
{
	struct max_ser_priv *priv = i2c_atr_get_driver_data(atr);
	struct max_i2c_xlate *xlate;

	if (priv->num_i2c_xlates == priv->ops->num_i2c_xlates) {
		dev_err(priv->dev,
			"Reached maximum number of I2C translations\n");
		return -EINVAL;
	}

	xlate = &priv->i2c_xlates[priv->num_i2c_xlates++];
	xlate->src = alias;
	xlate->dst = client->addr;

	return priv->ops->init_i2c_xlate(priv);
}

static void max_ser_i2c_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
					  const struct i2c_client *client)
{
	struct max_ser_priv *priv = i2c_atr_get_driver_data(atr);
	struct max_i2c_xlate *xlate;
	unsigned int i;

	/* Find index of matching I2C translation. */
	for (i = 0; i < priv->num_i2c_xlates; i++) {
		xlate = &priv->i2c_xlates[i];

		if (xlate->dst == client->addr)
			break;
	}

	WARN_ON(i == priv->num_i2c_xlates);

	/* Starting from index + 1, copy index translation into index - 1. */
	for (i++; i < priv->num_i2c_xlates; i++) {
		priv->i2c_xlates[i - 1].src = priv->i2c_xlates[i].src;
		priv->i2c_xlates[i - 1].dst = priv->i2c_xlates[i].dst;
	}

	/* Zero out last index translation. */
	priv->i2c_xlates[priv->num_i2c_xlates].src = 0;
	priv->i2c_xlates[priv->num_i2c_xlates].dst = 0;

	/* Decrease number of translations. */
	priv->num_i2c_xlates--;

	priv->ops->init_i2c_xlate(priv);
}

static const struct i2c_atr_ops max_ser_i2c_atr_ops = {
	.attach_client = max_ser_i2c_atr_attach_client,
	.detach_client = max_ser_i2c_atr_detach_client,
};

static void max_ser_i2c_atr_deinit(struct max_ser_priv *priv)
{
	/* Deleting adapters that haven't been added does no harm. */
	i2c_atr_del_adapter(priv->atr, 0);

	i2c_atr_delete(priv->atr);
}

static int max_ser_i2c_atr_init(struct max_ser_priv *priv)
{
	if (!i2c_check_functionality(priv->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	priv->atr = i2c_atr_new(priv->client->adapter, priv->dev,
				&max_ser_i2c_atr_ops, 1);
	if (!priv->atr)
		return -ENOMEM;

	i2c_atr_set_driver_data(priv->atr, priv);

	return i2c_atr_add_adapter(priv->atr, 0, NULL, NULL);
}

static int max_ser_update_pipe_dts(struct max_ser_priv *priv,
				   struct max_ser_pipe *pipe)
{
	struct max_ser_subdev_priv *sd_priv;

	pipe->num_dts = 0;

	if (priv->tunnel_mode)
		return 0;

	for_each_subdev(priv, sd_priv) {
		if (sd_priv->pipe_id != pipe->index)
			continue;

		if (pipe->num_dts == priv->ops->num_dts_per_pipe) {
			dev_err(priv->dev, "Too many data types per pipe\n");
			return -EINVAL;
		}

		if (!sd_priv->fmt)
			continue;

		/* TODO: optimize by checking for existing filters. */
		pipe->dts[pipe->num_dts++] = sd_priv->fmt->dt;
	}

	return priv->ops->update_pipe_dts(priv, pipe);
}

static int max_ser_update_pipe_vcs(struct max_ser_priv *priv,
				   struct max_ser_pipe *pipe)
{
	struct max_ser_subdev_priv *sd_priv;

	pipe->vcs = 0;

	if (priv->tunnel_mode)
		return 0;

	for_each_subdev(priv, sd_priv) {
		if (sd_priv->pipe_id != pipe->index)
			continue;

		pipe->vcs |= BIT(sd_priv->vc_id);
	}

	return priv->ops->update_pipe_vcs(priv, pipe);
}

static int max_ser_update_pipe_active(struct max_ser_priv *priv,
				      struct max_ser_pipe *pipe)
{
	struct max_ser_subdev_priv *sd_priv;
	bool enable = 0;

	for_each_subdev(priv, sd_priv) {
		if (sd_priv->pipe_id == pipe->index && sd_priv->active) {
			enable = 1;
			break;
		}
	}

	if (enable == pipe->active)
		return 0;

	pipe->active = enable;

	return priv->ops->set_pipe_enable(priv, pipe, enable);
}

static int max_ser_ch_enable(struct max_ser_subdev_priv *sd_priv, bool enable)
{
	struct max_ser_priv *priv = sd_priv->priv;
	struct max_ser_pipe *pipe = max_ser_ch_pipe(sd_priv);
	int ret = 0;

	mutex_lock(&priv->lock);

	if (sd_priv->active == enable)
		goto exit;

	sd_priv->active = enable;

	ret = max_ser_update_pipe_active(priv, pipe);

exit:
	mutex_unlock(&priv->lock);

	return ret;
}

static int max_ser_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct max_ser_subdev_priv *sd_priv = sd_to_max_ser(sd);

	return max_ser_ch_enable(sd_priv, enable);
}

static int max_ser_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct max_ser_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);

	if (format->pad == MAX_SER_SOURCE_PAD) {
		format->format.code = MEDIA_BUS_FMT_FIXED;
		return 0;
	}

	if (!sd_priv->fmt)
		return -EINVAL;

	format->format.code = sd_priv->fmt->code;

	return 0;
}

static int max_ser_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct max_ser_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_ser_priv *priv = sd_priv->priv;
	struct max_ser_pipe *pipe = &priv->pipes[sd_priv->pipe_id];
	const struct max_format *fmt;
	int ret;

	if (format->pad != MAX_SER_SINK_PAD)
		return -EINVAL;

	fmt = max_format_by_code(format->format.code);
	if (!fmt)
		return -EINVAL;

	sd_priv->fmt = fmt;

	mutex_lock(&priv->lock);

	ret = max_ser_update_pipe_dts(priv, pipe);

	mutex_unlock(&priv->lock);

	return ret;
}

static int max_ser_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	const struct max_format *fmt;

	if (code->pad != MAX_SER_SINK_PAD)
		return -EINVAL;

	fmt = max_format_by_index(code->index);
	if (!fmt)
		return -EINVAL;

	code->code = fmt->code;

	return 0;
}

static int max_ser_log_status(struct v4l2_subdev *sd)
{
	struct max_ser_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_ser_priv *priv = sd_priv->priv;
	unsigned int i, j;
	int ret;

	v4l2_info(sd, "tunnel_mode: %u\n", priv->tunnel_mode);
	v4l2_info(sd, "i2c_xlates: %u\n", priv->num_i2c_xlates);
	for (i = 0; i < priv->num_i2c_xlates; i++)
		v4l2_info(sd, "\tsrc: 0x%02x dst: 0x%02x\n",
			  priv->i2c_xlates[i].src, priv->i2c_xlates[i].dst);
	if (priv->ops->log_status) {
		ret = priv->ops->log_status(priv, sd->name);
		if (ret)
			return ret;
	}
	v4l2_info(sd, "\n");

	for_each_subdev(priv, sd_priv) {
		v4l2_info(sd, "channel: %u\n", sd_priv->index);
		v4l2_info(sd, "\tfwnode: %pfw\n", sd_priv->fwnode);
		v4l2_info(sd, "\tlabel: %s\n", sd_priv->label);
		v4l2_info(sd, "\tactive: %u\n", sd_priv->active);
		v4l2_info(sd, "\tfmt: %s\n", sd_priv->fmt ? sd_priv->fmt->name : NULL);
		v4l2_info(sd, "\tdt: 0x%02x\n", sd_priv->fmt ? sd_priv->fmt->dt : 0);
		v4l2_info(sd, "\tpipe_id: %u\n", sd_priv->pipe_id);
		v4l2_info(sd, "\tvc_id: %u\n", sd_priv->vc_id);
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &priv->pipes[i];

		v4l2_info(sd, "pipe: %u\n", pipe->index);
		v4l2_info(sd, "\tenabled: %u\n", pipe->enabled);
		v4l2_info(sd, "\tactive: %u\n", pipe->enabled);
		v4l2_info(sd, "\tphy_id: %u\n", pipe->phy_id);
		v4l2_info(sd, "\tstream_id: %u\n", pipe->stream_id);
		v4l2_info(sd, "\tdts: %u\n", pipe->num_dts);
		for (j = 0; j < pipe->num_dts; j++)
			v4l2_info(sd, "\t\tdt: 0x%02x\n", pipe->dts[j]);
		v4l2_info(sd, "\tvcs: 0x%08x\n", pipe->vcs);
		v4l2_info(sd, "\tdbl8: %u\n", pipe->dbl8);
		v4l2_info(sd, "\tdbl10: %u\n", pipe->dbl10);
		v4l2_info(sd, "\tdbl12: %u\n", pipe->dbl12);
		v4l2_info(sd, "\tsoft_bpp: %u\n", pipe->soft_bpp);
		v4l2_info(sd, "\tbpp: %u\n", pipe->bpp);
		if (priv->ops->log_pipe_status) {
			ret = priv->ops->log_pipe_status(priv, pipe, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < priv->ops->num_phys; i++) {
		struct max_ser_phy *phy = &priv->phys[i];

		v4l2_info(sd, "phy: %u\n", phy->index);
		v4l2_info(sd, "\tenabled: %u\n", phy->enabled);
		v4l2_info(sd, "\tnum_data_lanes: %u\n", phy->mipi.num_data_lanes);
		v4l2_info(sd, "\tclock_lane: %u\n", phy->mipi.clock_lane);
		v4l2_info(sd, "\tnoncontinuous_clock: %u\n",
			  !!(phy->mipi.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK));
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
static int max_ser_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct max_ser_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_ser_priv *priv = sd_priv->priv;
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int max_ser_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct max_ser_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct max_ser_priv *priv = sd_priv->priv;
	int ret;

	ret = regmap_write(priv->regmap, reg->reg, reg->val);
	if (ret)
		return ret;

	return 0;
}
#endif

static const struct v4l2_subdev_core_ops max_ser_core_ops = {
	.log_status = max_ser_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = max_ser_g_register,
	.s_register = max_ser_s_register,
#endif
};

static const struct v4l2_subdev_video_ops max_ser_video_ops = {
	.s_stream	= max_ser_s_stream,
};

static const struct v4l2_subdev_pad_ops max_ser_pad_ops = {
	.get_fmt = max_ser_get_fmt,
	.set_fmt = max_ser_set_fmt,
	.enum_mbus_code = max_ser_enum_mbus_code,
};

static const struct v4l2_subdev_ops max_ser_subdev_ops = {
	.core = &max_ser_core_ops,
	.video = &max_ser_video_ops,
	.pad = &max_ser_pad_ops,
};

static const struct media_entity_operations max_ser_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int max_ser_init(struct max_ser_priv *priv)
{
	unsigned int i;
	int ret;

	ret = priv->ops->init(priv);
	if (ret)
		return ret;

	for (i = 0; i < priv->ops->num_phys; i++) {
		struct max_ser_phy *phy = &priv->phys[i];

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
		struct max_ser_pipe *pipe = &priv->pipes[i];

		if (!pipe->enabled)
			continue;

		ret = priv->ops->init_pipe(priv, pipe);
		if (ret)
			return ret;

		ret = max_ser_update_pipe_vcs(priv, pipe);
		if (ret)
			return ret;

		ret = max_ser_update_pipe_dts(priv, pipe);
		if (ret)
			return ret;
	}

	ret = priv->ops->post_init(priv);
	if (ret)
		return ret;

	return 0;
}

static void max_ser_set_sd_name(struct max_ser_subdev_priv *sd_priv)
{
	struct max_ser_priv *priv = sd_priv->priv;
	struct i2c_client *client = priv->client;

	if (sd_priv->label) {
		strscpy(sd_priv->sd.name, sd_priv->label, sizeof(sd_priv->sd.name));
		return;
	}

	snprintf(sd_priv->sd.name, sizeof(sd_priv->sd.name), "%s %d-%04x:%u",
		 client->dev.driver->name, i2c_adapter_id(client->adapter),
		 client->addr, sd_priv->index);
}

static int max_ser_v4l2_register_sd(struct max_ser_subdev_priv *sd_priv)
{
	struct max_ser_priv *priv = sd_priv->priv;
	int ret;

	v4l2_i2c_subdev_init(&sd_priv->sd, priv->client, &max_ser_subdev_ops);
	max_ser_set_sd_name(sd_priv);
	sd_priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd_priv->sd.entity.ops = &max_ser_media_ops;
	sd_priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd_priv->sd.fwnode = sd_priv->fwnode;
	fwnode_handle_get(sd_priv->sd.fwnode);

	sd_priv->pads[MAX_SER_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	sd_priv->pads[MAX_SER_SINK_PAD].flags = MEDIA_PAD_FL_SINK;

	v4l2_set_subdevdata(&sd_priv->sd, sd_priv);

	ret = media_entity_pads_init(&sd_priv->sd.entity, MAX_SER_PAD_NUM, sd_priv->pads);
	if (ret)
		goto error;

	ret = v4l2_async_register_subdev(&sd_priv->sd);
	if (ret)
		goto error;

	return 0;

error:
	media_entity_cleanup(&sd_priv->sd.entity);
	fwnode_handle_put(sd_priv->sd.fwnode);

	return ret;
}

static void max_ser_v4l2_unregister_sd(struct max_ser_subdev_priv *sd_priv)
{
	v4l2_async_unregister_subdev(&sd_priv->sd);
	media_entity_cleanup(&sd_priv->sd.entity);
	fwnode_handle_put(sd_priv->sd.fwnode);
}

static int max_ser_v4l2_register(struct max_ser_priv *priv)
{
	struct max_ser_subdev_priv *sd_priv;
	int ret;

	for_each_subdev(priv, sd_priv) {
		ret = max_ser_v4l2_register_sd(sd_priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void max_ser_v4l2_unregister(struct max_ser_priv *priv)
{
	struct max_ser_subdev_priv *sd_priv;

	for_each_subdev(priv, sd_priv)
		max_ser_v4l2_unregister_sd(sd_priv);
}

static int max_ser_parse_pipe_dt(struct max_ser_priv *priv,
				 struct max_ser_pipe *pipe,
				 struct fwnode_handle *fwnode)
{
	unsigned int val;

	val = pipe->phy_id;
	fwnode_property_read_u32(fwnode, "maxim,phy-id", &val);
	if (val >= priv->ops->num_phys) {
		dev_err(priv->dev, "Invalid PHY %u\n", val);
		return -EINVAL;
	}
	pipe->phy_id = val;

	val = pipe->stream_id;
	fwnode_property_read_u32(fwnode, "maxim,stream-id", &val);
	if (val >= MAX_SERDES_STREAMS_NUM) {
		dev_err(priv->dev, "Invalid stream %u\n", val);
		return -EINVAL;
	}
	pipe->stream_id = val;

	val = 0;
	fwnode_property_read_u32(fwnode, "maxim,soft-bpp", &val);
	if (val > 24) {
		dev_err(priv->dev, "Invalid soft bpp %u\n", val);
		return -EINVAL;
	}
	pipe->soft_bpp = val;

	val = 0;
	fwnode_property_read_u32(fwnode, "maxim,bpp", &val);
	if (val > 24) {
		dev_err(priv->dev, "Invalid bpp %u\n", val);
		return -EINVAL;
	}
	pipe->bpp = val;

	pipe->dbl8 = fwnode_property_read_bool(fwnode, "maxim,dbl8");
	pipe->dbl10 = fwnode_property_read_bool(fwnode, "maxim,dbl10");
	pipe->dbl12 = fwnode_property_read_bool(fwnode, "maxim,dbl12");

	return 0;
}

static int max_ser_parse_ch_dt(struct max_ser_subdev_priv *sd_priv,
			       struct fwnode_handle *fwnode)
{
	struct max_ser_priv *priv = sd_priv->priv;
	struct max_ser_pipe *pipe;
	struct max_ser_phy *phy;
	u32 val;

	fwnode_property_read_string(fwnode, "label", &sd_priv->label);

	val = sd_priv->pipe_id;
	fwnode_property_read_u32(fwnode, "maxim,pipe-id", &val);
	if (val >= priv->ops->num_pipes) {
		dev_err(priv->dev, "Invalid pipe %u\n", val);
		return -EINVAL;
	}
	sd_priv->pipe_id = val;

	val = 0;
	fwnode_property_read_u32(fwnode, "maxim,vc-id", &val);
	if (val >= MAX_SERDES_VC_ID_NUM) {
		dev_err(priv->dev, "Invalid virtual channel %u\n", val);
		return -EINVAL;
	}
	sd_priv->vc_id = val;

	if (fwnode_property_read_bool(fwnode, "maxim,embedded-data"))
		sd_priv->fmt = max_format_by_dt(MAX_DT_EMB8);

	pipe = &priv->pipes[val];
	pipe->enabled = true;

	phy = &priv->phys[pipe->phy_id];
	phy->enabled = true;

	return 0;
}

static int max_ser_parse_sink_dt_endpoint(struct max_ser_subdev_priv *sd_priv,
					  struct fwnode_handle *fwnode)
{
	struct max_ser_priv *priv = sd_priv->priv;
	struct max_ser_pipe *pipe = max_ser_ch_pipe(sd_priv);
	struct max_ser_phy *phy = max_ser_pipe_phy(priv, pipe);
	struct v4l2_fwnode_endpoint v4l2_ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct v4l2_fwnode_bus_mipi_csi2 *mipi = &v4l2_ep.bus.mipi_csi2;
	struct fwnode_handle *ep, *remote_ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, MAX_SER_SINK_PAD, 0, 0);
	if (!ep) {
		fwnode_handle_put(ep);
		return 0;
	}

	remote_ep = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);
	if (!remote_ep) {
		dev_err(priv->dev, "Not connected to subdevice\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(remote_ep, &v4l2_ep);
	fwnode_handle_put(remote_ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse v4l2 endpoint\n");
		return ret;
	}

	if (mipi->flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK &&
	    !priv->ops->supports_noncontinuous_clock) {
		dev_err(priv->dev, "Clock non-continuous mode is not supported\n");
		return -EINVAL;
	}

	if (!phy->bus_config_parsed) {
		phy->mipi = v4l2_ep.bus.mipi_csi2;
		phy->bus_config_parsed = true;

		return 0;
	}

	if (phy->mipi.num_data_lanes != mipi->num_data_lanes) {
		dev_err(priv->dev, "PHY configured with differing number of data lanes\n");
		return -EINVAL;
	}

	if ((phy->mipi.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK) !=
	    (mipi->flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK)) {
		dev_err(priv->dev, "PHY configured with differing clock continuity\n");
		return -EINVAL;
	}

	return 0;
}

static int max_ser_parse_dt(struct max_ser_priv *priv)
{
	const char *channel_node_name = "channel";
	const char *pipe_node_name = "pipe";
	struct max_ser_subdev_priv *sd_priv;
	struct fwnode_handle *fwnode;
	struct max_ser_pipe *pipe;
	struct max_ser_phy *phy;
	unsigned int i;
	u32 index;
	u32 val;
	int ret;

	val = device_property_read_bool(priv->dev, "maxim,tunnel-mode");
	if (val && !priv->ops->supports_tunnel_mode) {
		dev_err(priv->dev, "Tunnel mode is not supported\n");
		return -EINVAL;
	}
	priv->tunnel_mode = val;

	for (i = 0; i < priv->ops->num_phys; i++) {
		phy = &priv->phys[i];
		phy->index = i;
	}

	for (i = 0; i < priv->ops->num_pipes; i++) {
		pipe = &priv->pipes[i];
		pipe->index = i;
		pipe->phy_id = i % priv->ops->num_phys;
		pipe->stream_id = i % MAX_SERDES_STREAMS_NUM;
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
			dev_err(priv->dev, "Invalid pipe number %u\n", index);
			fwnode_handle_put(fwnode);
			return -EINVAL;
		}

		pipe = &priv->pipes[index];

		ret = max_ser_parse_pipe_dt(priv, pipe, fwnode);
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
		sd_priv->pipe_id = index % priv->ops->num_pipes;

		ret = max_ser_parse_ch_dt(sd_priv, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}

		ret = max_ser_parse_sink_dt_endpoint(sd_priv, fwnode);
		if (ret) {
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	return 0;
}

static int max_ser_allocate(struct max_ser_priv *priv)
{
	unsigned int i;

	priv->phys = devm_kcalloc(priv->dev, priv->ops->num_phys,
				  sizeof(*priv->phys), GFP_KERNEL);
	if (!priv->phys)
		return -ENOMEM;

	priv->pipes = devm_kcalloc(priv->dev, priv->ops->num_pipes,
				   sizeof(*priv->pipes), GFP_KERNEL);
	if (!priv->pipes)
		return -ENOMEM;

	priv->i2c_xlates = devm_kcalloc(priv->dev, priv->ops->num_i2c_xlates,
					sizeof(*priv->i2c_xlates), GFP_KERNEL);
	if (!priv->i2c_xlates)
		return -ENOMEM;

	for (i = 0; i < priv->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &priv->pipes[i];

		pipe->dts = devm_kcalloc(priv->dev, priv->ops->num_dts_per_pipe,
					 sizeof(*pipe->dts), GFP_KERNEL);
	}

	return 0;
}

int max_ser_probe(struct max_ser_priv *priv)
{
	int ret;

	mutex_init(&priv->lock);

	ret = max_ser_allocate(priv);
	if (ret)
		return ret;

	ret = max_ser_parse_dt(priv);
	if (ret)
		return ret;

	ret = max_ser_init(priv);
	if (ret)
		return ret;

	ret = max_ser_i2c_atr_init(priv);
	if (ret)
		return ret;

	return max_ser_v4l2_register(priv);
}
EXPORT_SYMBOL_GPL(max_ser_probe);

int max_ser_remove(struct max_ser_priv *priv)
{
	max_ser_v4l2_unregister(priv);

	max_ser_i2c_atr_deinit(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_remove);

int max_ser_reset(struct regmap *regmap)
{
	int ret;

	ret = regmap_update_bits(regmap, 0x10, 0x80, 0x80);
	if (ret)
		return ret;

	msleep(50);

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_reset);

int max_ser_wait_for_multiple(struct i2c_client *client, struct regmap *regmap,
			      u8 *addrs, unsigned int num_addrs)
{
	unsigned int i, j, val;
	int ret;

	for (i = 0; i < 10; i++) {
		for (j = 0; j < num_addrs; j++) {
			client->addr = addrs[j];

			ret = regmap_read(regmap, 0x0, &val);
			if (ret >= 0)
				return 0;
		}

		msleep(100);

		dev_err(&client->dev, "Retry %u waiting for serializer: %d\n", i, ret);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(max_ser_wait_for_multiple);

int max_ser_wait(struct i2c_client *client, struct regmap *regmap, u8 addr)
{
	return max_ser_wait_for_multiple(client, regmap, &addr, 1);
}
EXPORT_SYMBOL_GPL(max_ser_wait);

static int max_ser_get_dev_id(struct regmap *regmap, unsigned int *dev_id)
{
	return regmap_read(regmap, 0xd, dev_id);
}

static int max_ser_fix_tx_ids(struct regmap *regmap, u8 addr)
{
	unsigned int addr_regs[] = { 0x7b, 0x83, 0x8b, 0x93, 0xa3, 0xab };
	unsigned int dev_id;
	unsigned int i;
	int ret;

	ret = max_ser_get_dev_id(regmap, &dev_id);
	if (ret)
		return ret;

	switch (dev_id) {
	case MAX_SER_MAX9265A_DEV_ID:
		for (i = 0; i < ARRAY_SIZE(addr_regs); i++) {
			ret = regmap_write(regmap, addr_regs[i], addr);
			if (ret)
				return ret;
		}

		break;
	default:
		return 0;
	}

	return 0;
}

int max_ser_change_address(struct i2c_client *client, struct regmap *regmap, u8 addr,
			   bool fix_tx_ids)
{
	int ret;

	ret = regmap_write(regmap, 0x0, addr << 1);
	if (ret)
		return ret;

	client->addr = addr;

	if (fix_tx_ids) {
		ret = max_ser_fix_tx_ids(regmap, addr);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_change_address);

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(I2C_ATR);
