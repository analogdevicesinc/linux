// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/i2c-atr.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max_ser.h"
#include "max_serdes.h"

#define MAX_SER_NUM_LINKS	1

struct max_ser_priv {
	struct max_ser *ser;
	struct device *dev;
	struct i2c_client *client;

	struct i2c_atr *atr;
	struct i2c_mux_core *mux;

	struct media_pad *pads;
	struct max_source *sources;
	u64 *streams_masks;
	u32 double_bpps;

	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;
};

static inline struct max_ser_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_ser_priv, sd);
}

static inline struct max_ser_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_ser_priv, nf);
}

static inline bool max_ser_pad_is_sink(struct max_ser *ser, u32 pad)
{
	return pad < ser->ops->num_phys;
}

static inline bool max_ser_pad_is_source(struct max_ser *ser, u32 pad)
{
	return pad >= ser->ops->num_phys;
}

static inline unsigned int max_ser_phy_to_pad(struct max_ser *ser,
					      struct max_ser_phy *phy)
{
	return phy->index;
}

static inline unsigned int max_ser_num_pads(struct max_ser *ser)
{
	return ser->ops->num_phys + MAX_SER_NUM_LINKS;
}

static struct max_ser_phy *max_ser_pad_to_phy(struct max_ser *ser, u32 pad)
{
	if (!max_ser_pad_is_sink(ser, pad))
		return NULL;

	return &ser->phys[pad];
}

static struct max_ser_pipe *
max_ser_find_phy_pipe(struct max_ser *ser, struct max_ser_phy *phy)
{
	unsigned int i;

	for (i = 0; i < ser->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &ser->pipes[i];

		if (pipe->phy_id == phy->index)
			return pipe;
	}

	return NULL;
}

static struct max_source *
max_ser_find_phy_source(struct max_ser_priv *priv, struct max_ser_phy *phy)
{
	return &priv->sources[phy->index];
}

static int max_ser_phy_set_active(struct max_ser *ser, struct max_ser_phy *phy,
				  bool active)
{
	int ret;

	ret = ser->ops->set_phy_active(ser, phy, active);
	if (ret)
		return ret;

	phy->active = active;

	return 0;
}

static int max_ser_set_pipe_enable(struct max_ser *ser, struct max_ser_pipe *pipe,
				   bool enable)
{
	int ret;

	ret = ser->ops->set_pipe_enable(ser, pipe, enable);
	if (ret)
		return ret;

	pipe->enabled = enable;

	return 0;
}

static int max_ser_set_pipe_dts(struct max_ser_priv *priv, struct max_ser_pipe *pipe,
				unsigned int *dts, unsigned int num_dts)
{
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	for (i = 0; i < num_dts; i++) {
		ret = ser->ops->set_pipe_dt(ser, pipe, i, dts[i]);
		if (ret)
			return ret;

		ret = ser->ops->set_pipe_dt_en(ser, pipe, i, true);
		if (ret)
			return ret;
	}

	if (num_dts == pipe->num_dts)
		return 0;

	for (i = num_dts; i < ser->ops->num_dts_per_pipe; i++) {
		ret = ser->ops->set_pipe_dt_en(ser, pipe, i, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_ser_set_pipe_mode(struct max_ser_priv *priv, struct max_ser_pipe *pipe,
				 struct max_ser_pipe_mode *mode)
{
	struct max_ser *ser = priv->ser;

	if (mode->bpp == pipe->mode.bpp &&
	    mode->soft_bpp == pipe->mode.soft_bpp &&
	    mode->dbl8 == pipe->mode.dbl8 &&
	    mode->dbl10 == pipe->mode.dbl10 &&
	    mode->dbl12 == pipe->mode.dbl12)
		return 0;

	return ser->ops->set_pipe_mode(ser, pipe, mode);
}

static int max_ser_i2c_atr_attach_addr(struct i2c_atr *atr, u32 chan_id,
				       u16 addr, u16 alias)
{
	struct max_i2c_xlate xlate = { .src = alias, .dst = addr, .en = true };
	struct max_ser_priv *priv = i2c_atr_get_driver_data(atr);
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	for (i = 0; i < ser->ops->num_i2c_xlates; i++)
		if (!ser->i2c_xlates[i].en)
			break;

	if (i == ser->ops->num_i2c_xlates) {
		dev_err(priv->dev,
			"Reached maximum number of I2C translations\n");
		return -EINVAL;
	}

	ret = ser->ops->set_i2c_xlate(ser, i, &xlate);
	if (ret)
		return ret;

	ser->i2c_xlates[i] = xlate;

	return 0;
}

static void max_ser_i2c_atr_detach_addr(struct i2c_atr *atr, u32 chan_id, u16 addr)
{
	struct max_ser_priv *priv = i2c_atr_get_driver_data(atr);
	struct max_ser *ser = priv->ser;
	struct max_i2c_xlate xlate = { 0 };
	unsigned int i;

	/* Find index of matching I2C translation. */
	for (i = 0; i < ser->ops->num_i2c_xlates; i++)
		if (ser->i2c_xlates[i].dst == addr)
			break;

	WARN_ON(i == ser->ops->num_i2c_xlates);

	ser->ops->set_i2c_xlate(ser, i, &xlate);
	ser->i2c_xlates[i] = xlate;
}

static const struct i2c_atr_ops max_ser_i2c_atr_ops = {
	.attach_addr = max_ser_i2c_atr_attach_addr,
	.detach_addr = max_ser_i2c_atr_detach_addr,
};

static void max_ser_i2c_atr_deinit(struct max_ser_priv *priv)
{
	/* Deleting adapters that haven't been added does no harm. */
	i2c_atr_del_adapter(priv->atr, 0);

	i2c_atr_delete(priv->atr);
}

static int max_ser_i2c_atr_init(struct max_ser_priv *priv)
{
	struct i2c_atr_adap_desc desc = {
		.chan_id = 0,
	};

	if (!i2c_check_functionality(priv->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	priv->atr = i2c_atr_new(priv->client->adapter, priv->dev,
				&max_ser_i2c_atr_ops, 1, 0);
	if (IS_ERR(priv->atr))
		return PTR_ERR(priv->atr);

	i2c_atr_set_driver_data(priv->atr, priv);

	return i2c_atr_add_adapter(priv->atr, &desc);
}

static int max_ser_i2c_mux_select(struct i2c_mux_core *mux, u32 chan)
{
	return 0;
}

static void max_ser_i2c_mux_deinit(struct max_ser_priv *priv)
{
	i2c_mux_del_adapters(priv->mux);
}

static int max_ser_i2c_mux_init(struct max_ser_priv *priv)
{
	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  1, 0, I2C_MUX_LOCKED | I2C_MUX_GATE,
				  max_ser_i2c_mux_select, NULL);
	if (!priv->mux)
		return -ENOMEM;

	return i2c_mux_add_adapter(priv->mux, 0, 0);
}

static int max_ser_i2c_adapter_init(struct max_ser_priv *priv)
{
	if (device_get_named_child_node(priv->dev, "i2c-gate"))
		return max_ser_i2c_mux_init(priv);
	else
		return max_ser_i2c_atr_init(priv);
}

static void max_ser_i2c_adapter_deinit(struct max_ser_priv *priv)
{
	if (device_get_named_child_node(priv->dev, "i2c-gate"))
		max_ser_i2c_mux_deinit(priv);
	else
		max_ser_i2c_atr_deinit(priv);
}

static int max_ser_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	struct v4l2_mbus_framefmt *fmt;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && ser->active)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (max_ser_pad_is_source(ser, format->pad))
		return v4l2_subdev_get_fmt(sd, state, format);

	fmt = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = format->format;

	return 0;
}

static int max_ser_log_status(struct v4l2_subdev *sd)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	unsigned int i, j;
	int ret;

	v4l2_info(sd, "tunnel: %u\n", ser->tunnel);
	if (ser->ops->log_status) {
		ret = ser->ops->log_status(ser, sd->name);
		if (ret)
			return ret;
	}
	for (i = 0; i < ser->ops->num_i2c_xlates; i++)
		v4l2_info(sd, "\ten: %u, src: 0x%02x dst: 0x%02x\n",
			  ser->i2c_xlates[i].en, ser->i2c_xlates[i].src,
			  ser->i2c_xlates[i].dst);
	v4l2_info(sd, "\n");

	for (i = 0; i < ser->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &ser->pipes[i];

		v4l2_info(sd, "pipe: %u\n", pipe->index);
		v4l2_info(sd, "\tenabled: %u\n", pipe->enabled);

		if (!pipe->enabled) {
			v4l2_info(sd, "\n");
			continue;
		}

		v4l2_info(sd, "\tphy_id: %u\n", pipe->phy_id);
		v4l2_info(sd, "\tstream_id: %u\n", pipe->stream_id);
		v4l2_info(sd, "\tdts: %u\n", pipe->num_dts);
		for (j = 0; j < pipe->num_dts; j++)
			v4l2_info(sd, "\t\tdt: 0x%02x\n", pipe->dts[j]);
		v4l2_info(sd, "\tvcs: 0x%08x\n", pipe->vcs);
		v4l2_info(sd, "\tdbl8: %u\n", pipe->mode.dbl8);
		v4l2_info(sd, "\tdbl10: %u\n", pipe->mode.dbl10);
		v4l2_info(sd, "\tdbl12: %u\n", pipe->mode.dbl12);
		v4l2_info(sd, "\tsoft_bpp: %u\n", pipe->mode.soft_bpp);
		v4l2_info(sd, "\tbpp: %u\n", pipe->mode.bpp);
		if (ser->ops->log_pipe_status) {
			ret = ser->ops->log_pipe_status(ser, pipe, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];

		v4l2_info(sd, "phy: %u\n", phy->index);
		v4l2_info(sd, "\tenabled: %u\n", phy->enabled);

		if (!phy->enabled) {
			v4l2_info(sd, "\n");
			continue;
		}

		v4l2_info(sd, "\tactive: %u\n", phy->active);
		v4l2_info(sd, "\tnum_data_lanes: %u\n", phy->mipi.num_data_lanes);
		v4l2_info(sd, "\tclock_lane: %u\n", phy->mipi.clock_lane);
		v4l2_info(sd, "\tnoncontinuous_clock: %u\n",
			  !!(phy->mipi.flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK));
		if (ser->ops->log_phy_status) {
			ret = ser->ops->log_phy_status(ser, phy, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	return 0;
}

static int max_ser_get_frame_desc_state(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_mbus_frame_desc *fd,
					unsigned int pad)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	struct v4l2_subdev_route *route;
	int ret;

	if (!max_ser_pad_is_source(ser, pad))
		return -ENOENT;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		struct max_source *source;
		struct max_ser_phy *phy;

		if (pad != route->source_pad)
			continue;

		phy = max_ser_pad_to_phy(ser, route->sink_pad);
		if (!phy) {
			dev_err(priv->dev, "Failed to find link for pad %u\n",
				route->sink_pad);
			return -ENOENT;
		}

		source = max_ser_find_phy_source(priv, phy);
		if (!source)
			return -ENOENT;

		if (!source->sd)
			continue;

		ret = max_get_fd_stream_entry(source->sd, source->pad,
					      route->sink_stream, &entry);
		if (ret) {
			dev_err(priv->dev,
				"Failed to find frame desc entry for stream %u:%u: %d\n",
				route->sink_pad, route->sink_stream, ret);
			return ret;
		}

		entry.stream = route->source_stream;

		fd->entry[fd->num_entries++] = entry;
	}

	return 0;
}

static int max_ser_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(&priv->sd);

	ret = max_ser_get_frame_desc_state(sd, state, fd, pad);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int max_ser_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	int ret;

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && ser->active)
		return -EBUSY;

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -E2BIG;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1 |
					   V4L2_SUBDEV_ROUTING_NO_SINK_STREAM_MIX);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing(sd, state, routing);
}

static int max_ser_get_vcs_dts(struct max_ser_priv *priv,
			       const struct v4l2_subdev_krouting *routing,
			       struct max_source *source,
			       unsigned int *vcs,
			       unsigned int *dts, unsigned int *num_dts,
			       u32 sink_pad, u64 streams_mask)
{
	struct max_ser *ser = priv->ser;
	struct v4l2_subdev_route *route;
	unsigned int i;
	int ret;

	*vcs = 0;
	*num_dts = 0;

	if (ser->tunnel)
		return 0;

	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		unsigned int vc, dt;

		if (sink_pad != route->sink_pad)
			continue;

		if (!(BIT_ULL(route->sink_stream) & streams_mask))
			continue;

		ret = max_get_fd_stream_entry(source->sd, source->pad,
					      route->sink_stream, &entry);
		if (ret) {
			dev_err(priv->dev,
				"Failed to find frame desc entry for stream %u:%u: %d\n",
				route->sink_pad, route->sink_stream, ret);
			return ret;
		}

		vc = entry.bus.csi2.vc;
		dt = entry.bus.csi2.dt;

		if (vc >= MAX_SERDES_VC_ID_NUM)
			return -E2BIG;

		*vcs |= BIT(vc);

		/* Skip already added DT. */
		for (i = 0; i < *num_dts; i++)
			if (dts[i] == dt)
				break;

		if (i < *num_dts)
			continue;

		dts[*num_dts] = dt;
		(*num_dts)++;
	}

	/*
	 * Hardware cannot distinguish between different pairs of VC and DT,
	 * issue a warning.
	 */
	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		unsigned int vc, dt;

		if (sink_pad != route->sink_pad)
			continue;

		/*
		 * Skip enabled streams, we only want to check for leaks
		 * among the disabled streams.
		 */
		if ((BIT_ULL(route->sink_stream) & streams_mask))
			continue;

		ret = max_get_fd_stream_entry(source->sd, source->pad,
					      route->sink_stream, &entry);
		if (ret) {
			dev_err(priv->dev,
				"Failed to find frame desc entry for stream %u:%u: %d\n",
				route->sink_pad, route->sink_stream, ret);
			return ret;
		}

		vc = entry.bus.csi2.vc;
		dt = entry.bus.csi2.dt;

		if (vc >= MAX_SERDES_VC_ID_NUM)
			return -E2BIG;

		if (!(*vcs & BIT(vc)))
			continue;

		for (i = 0; i < *num_dts; i++)
			if (dts[i] == dt)
				break;

		if (i == *num_dts)
			continue;

		dev_warn(priv->dev, "Leaked disabled stream %u:%u with VC: %u, DT: %u",
			 route->source_pad, route->source_stream, vc, dt);
	}

	return 0;
}

static int max_ser_get_mode(struct max_ser_priv *priv,
			    struct max_ser_pipe_mode *mode,
			    const struct v4l2_subdev_krouting *routing, u32 pad)
{
	struct max_ser *ser = priv->ser;
	unsigned int doubled_bpp;
	unsigned int min_bpp;
	unsigned int max_bpp;
	u32 bpps;
	int ret;

	if (ser->tunnel)
		return 0;

	ret = max_get_bpps(priv->sources, 0, &bpps, routing, pad, ~0ULL);
	if (ret)
		return ret;

	ret = max_process_bpps(priv->dev, bpps, priv->double_bpps, &doubled_bpp);
	if (ret)
		return ret;

	if (doubled_bpp == 8)
		mode->dbl8 = true;
	else if (doubled_bpp == 10)
		mode->dbl10 = true;
	else if (doubled_bpp == 12)
		mode->dbl12 = true;

	if (doubled_bpp) {
		bpps &= ~BIT(doubled_bpp);
		bpps |= BIT(doubled_bpp * 2);
	}

	min_bpp = __ffs(bpps);
	max_bpp = __fls(bpps);

	if (doubled_bpp)
		mode->soft_bpp = min_bpp;

	if (min_bpp != max_bpp)
		mode->bpp = max_bpp;

	return 0;
}

static int max_ser_update_pipe(struct max_ser_priv *priv,
			       struct max_source *source,
			       struct max_ser_pipe *pipe,
			       const struct v4l2_subdev_krouting *routing,
			       u32 pad, u64 streams_mask)
{
	struct max_ser *ser = priv->ser;
	struct max_ser_pipe_mode mode = { 0 };
	unsigned int num_dts;
	unsigned int *dts;
	unsigned int vcs;
	int ret;

	dts = devm_kcalloc(priv->dev, ser->ops->num_dts_per_pipe, sizeof(*dts),
			   GFP_KERNEL);
	if (!dts)
		return -ENOMEM;

	ret = max_ser_get_vcs_dts(priv, routing, source, &vcs, dts, &num_dts,
				  pad, streams_mask);
	if (ret)
		goto err_free_dts;

	ret = max_ser_get_mode(priv, &mode, routing, pad);
	if (ret)
		goto err_free_dts;

	ret = ser->ops->set_pipe_vcs(ser, pipe, vcs);
	if (ret)
		goto err_free_dts;

	ret = max_ser_set_pipe_mode(priv, pipe, &mode);
	if (ret)
		goto err_revert_vcs;

	ret = max_ser_set_pipe_dts(priv, pipe, dts, num_dts);
	if (ret)
		goto err_revert_mode;

	pipe->vcs = vcs;
	pipe->mode = mode;

	if (pipe->dts)
		devm_kfree(priv->dev, pipe->dts);

	pipe->dts = dts;
	pipe->num_dts = num_dts;

	return 0;

err_revert_mode:
	max_ser_set_pipe_mode(priv, pipe, &pipe->mode);

err_revert_vcs:
	ser->ops->set_pipe_vcs(ser, pipe, pipe->vcs);

err_free_dts:
	devm_kfree(priv->dev, dts);

	return ret;
}

static int max_ser_update_phy(struct max_ser_priv *priv,
			      const struct v4l2_subdev_krouting *routing,
			      struct max_ser_phy *phy, u64 streams_mask)
{
	struct max_ser *ser = priv->ser;
	u32 pad = max_ser_phy_to_pad(ser, phy);
	bool enable_changed = !streams_mask != !priv->streams_masks[pad];
	bool enable = !!streams_mask;
	struct max_source *source;
	struct max_ser_pipe *pipe;
	int ret;

	pipe = max_ser_find_phy_pipe(ser, phy);
	if (!pipe)
		return -ENOENT;

	source = max_ser_find_phy_source(priv, phy);
	if (!source)
		return -ENOENT;

	if (!source->sd)
		return 0;

	if (!enable && enable_changed) {
		ret = max_ser_set_pipe_enable(ser, pipe, enable);
		if (ret)
			return -ENOENT;

		ret = max_ser_phy_set_active(ser, phy, enable);
		if (ret)
			goto err_revert_pipe_disable;
	}

	ret = max_ser_update_pipe(priv, source, pipe, routing, pad, streams_mask);
	if (ret)
		goto err_revert_phy_disable;

	if (enable && enable_changed) {
		ret = max_ser_phy_set_active(ser, phy, enable);
		if (ret)
			goto err_revert_pipe_update;

		ret = max_ser_set_pipe_enable(ser, pipe, enable);
		if (ret)
			goto err_revert_phy_enable;
	}

	return 0;

err_revert_phy_enable:
	if (enable && enable_changed)
		max_ser_set_pipe_enable(ser, pipe, !enable);

err_revert_pipe_update:
	max_ser_update_pipe(priv, source, pipe, routing, pad, priv->streams_masks[pad]);

err_revert_phy_disable:
	if (!enable && enable_changed)
		max_ser_phy_set_active(ser, phy, !enable);

err_revert_pipe_disable:
	if (!enable && enable_changed)
		max_ser_set_pipe_enable(ser, pipe, !enable);

	return ret;
}

static int max_ser_update_phys(struct max_ser_priv *priv,
			       const struct v4l2_subdev_krouting *routing,
			       u64 *streams_masks)
{
	struct max_ser *ser = priv->ser;
	unsigned int failed_update_phy_id = ser->ops->num_phys;
	unsigned int i;
	int ret;

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		u32 sink_pad = max_ser_phy_to_pad(ser, phy);

		ret = max_ser_update_phy(priv, routing, phy, streams_masks[sink_pad]);
		if (ret) {
			failed_update_phy_id = i;
			goto err;
		}
	}

	return 0;

err:
	for (i = 0; i < failed_update_phy_id; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		u32 sink_pad = max_ser_phy_to_pad(ser, phy);

		max_ser_update_phy(priv, routing, phy, priv->streams_masks[sink_pad]);
	}

	return ret;
}

static int max_ser_enable_disable_streams(struct max_ser_priv *priv,
					  struct v4l2_subdev_state *state,
					  u32 pad, u64 updated_streams_mask,
					  bool enable)
{
	struct max_ser *ser = priv->ser;

	return max_xlate_enable_disable_streams(priv->sources, 0, state,
						pad, updated_streams_mask, 0,
						ser->ops->num_phys, enable);
}

static int max_ser_update_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 updated_streams_mask, bool enable)
{
	struct max_ser_priv *priv = v4l2_get_subdevdata(sd);
	struct max_ser *ser = priv->ser;
	unsigned int num_pads = max_ser_num_pads(ser);
	u64 *streams_masks;
	int ret;

	ret = max_get_streams_masks(priv->dev, state, pad, updated_streams_mask,
				    num_pads, 0, ser->ops->num_phys,
				    priv->streams_masks, &streams_masks, enable);
	if (ret)
		return ret;

	if (!enable) {
		ret = max_ser_enable_disable_streams(priv, state, pad,
						     updated_streams_mask, enable);
		if (ret)
			goto err_free_streams_masks;
	}

	ret = max_ser_update_phys(priv, &state->routing, streams_masks);
	if (ret)
		goto err_revert_streams_disable;

	if (enable) {
		ret = max_ser_enable_disable_streams(priv, state, pad,
						     updated_streams_mask, enable);
		if (ret)
			goto err_revert_phys_update;
	}

	devm_kfree(priv->dev, priv->streams_masks);
	priv->streams_masks = streams_masks;
	ser->active = !!streams_masks[pad];

	return 0;

err_revert_phys_update:
	max_ser_update_phys(priv, &state->routing, priv->streams_masks);

err_revert_streams_disable:
	if (!enable)
		max_ser_enable_disable_streams(priv, state, pad,
					       updated_streams_mask, !enable);

err_free_streams_masks:
	devm_kfree(priv->dev, streams_masks);

	return ret;
}

static int max_ser_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	return max_ser_update_streams(sd, state, pad, streams_mask, true);
}

static int max_ser_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	return max_ser_update_streams(sd, state, pad, streams_mask, false);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max_ser_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	unsigned int val;
	int ret;

	ret = ser->ops->reg_read(ser, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int max_ser_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;

	return ser->ops->reg_write(ser, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops max_ser_core_ops = {
	.log_status = max_ser_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = max_ser_g_register,
	.s_register = max_ser_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops max_ser_pad_ops = {
	.enable_streams = max_ser_enable_streams,
	.disable_streams = max_ser_disable_streams,

	.set_routing = max_ser_set_routing,
	.get_frame_desc = max_ser_get_frame_desc,

	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = max_ser_set_fmt,
};

static const struct v4l2_subdev_ops max_ser_subdev_ops = {
	.core = &max_ser_core_ops,
	.pad = &max_ser_pad_ops,
};

static const struct media_entity_operations max_ser_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int max_ser_init(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	ret = ser->ops->init(ser);
	if (ret)
		return ret;

	if (ser->ops->set_tunnel_enable) {
		ret = ser->ops->set_tunnel_enable(ser, false);
		if (ret)
			return ret;
	}

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];

		if (phy->enabled) {
			ret = ser->ops->init_phy(ser, phy);
			if (ret)
				return ret;
		}

		ret = ser->ops->set_phy_active(ser, phy, false);
		if (ret)
			return ret;
	}

	for (i = 0; i < ser->ops->num_pipes; i++) {
		struct max_ser_pipe *pipe = &ser->pipes[i];
		struct max_ser_phy *phy = &ser->phys[pipe->phy_id];

		ret = ser->ops->set_pipe_enable(ser, pipe, false);
		if (ret)
			return ret;

		ret = ser->ops->set_pipe_stream_id(ser, pipe, pipe->stream_id);
		if (ret)
			return ret;

		ret = ser->ops->set_pipe_phy(ser, pipe, phy);
		if (ret)
			return ret;

		ret = ser->ops->set_pipe_vcs(ser, pipe, 0);
		if (ret)
			return ret;

		ret = ser->ops->set_pipe_mode(ser, pipe, &pipe->mode);
		if (ret)
			return ret;

		ret = max_ser_set_pipe_dts(priv, pipe, NULL, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_ser_notify_bound(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct max_ser_priv *priv = nf_to_priv(nf);
	struct max_asc *asc = asc_to_max(base_asc);
	struct max_source *source = asc->source;
	u32 pad = source->index;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
					  source->ep_fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	source->sd = subdev;
	source->pad = ret;

	ret = media_create_pad_link(&source->sd->entity, source->pad,
				    &priv->sd.entity, pad,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(priv->dev, "Unable to link %s:%u -> %s:%u\n",
			source->sd->name, source->pad, priv->sd.name, pad);
		return ret;
	}

	return 0;
}

static void max_ser_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct max_asc *asc = asc_to_max(base_asc);
	struct max_source *source = asc->source;

	source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max_ser_notify_ops = {
	.bound = max_ser_notify_bound,
	.unbind = max_ser_notify_unbind,
};

static int max_ser_v4l2_notifier_register(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	unsigned int i;
	int ret;

	v4l2_async_subdev_nf_init(&priv->nf, &priv->sd);

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		struct max_source *source;
		struct max_asc *asc;

		source = max_ser_find_phy_source(priv, phy);
		if (!source)
			return -ENOENT;

		if (!source->ep_fwnode)
			continue;

		asc = v4l2_async_nf_add_fwnode(&priv->nf, source->ep_fwnode,
					       struct max_asc);
		if (IS_ERR(asc)) {
			dev_err(priv->dev,
				"Failed to add subdev for source %u: %pe", i,
				asc);

			v4l2_async_nf_cleanup(&priv->nf);

			return PTR_ERR(asc);
		}

		asc->source = source;
	}

	priv->nf.ops = &max_ser_notify_ops;

	ret = v4l2_async_nf_register(&priv->nf);
	if (ret) {
		dev_err(priv->dev, "Failed to register subdev notifier");
		v4l2_async_nf_cleanup(&priv->nf);
		return ret;
	}

	return 0;
}

static void max_ser_v4l2_notifier_unregister(struct max_ser_priv *priv)
{
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
}

static int max_ser_v4l2_register(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct max_ser *ser = priv->ser;
	void *data = i2c_get_clientdata(priv->client);
	unsigned int num_pads = max_ser_num_pads(ser);
	unsigned int i;
	int ret;

	v4l2_i2c_subdev_init(sd, priv->client, &max_ser_subdev_ops);
	i2c_set_clientdata(priv->client, data);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_ser_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;

	priv->pads = devm_kcalloc(priv->dev, num_pads, sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads)
		return -ENOMEM;

	for (i = 0; i < num_pads; i++) {
		if (max_ser_pad_is_sink(ser, i))
			priv->pads[i].flags = MEDIA_PAD_FL_SINK;
		else
			priv->pads[i].flags = MEDIA_PAD_FL_SOURCE;
	}

	v4l2_set_subdevdata(sd, priv);

	ret = media_entity_pads_init(&sd->entity, num_pads, priv->pads);
	if (ret)
		return ret;

	ret = max_ser_v4l2_notifier_register(priv);
	if (ret)
		goto err_media_entity_cleanup;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_nf_cleanup;

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_sd_cleanup;

	return 0;

err_sd_cleanup:
	v4l2_subdev_cleanup(sd);
err_nf_cleanup:
	max_ser_v4l2_notifier_unregister(priv);
err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);

	return ret;
}

static void max_ser_v4l2_unregister(struct max_ser_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;

	max_ser_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
}

static int max_ser_parse_sink_dt_endpoint(struct max_ser_priv *priv,
					  struct max_ser_phy *phy,
					  struct max_source *source,
					  struct fwnode_handle *fwnode)
{
	struct max_ser *ser = priv->ser;
	u32 pad = max_ser_phy_to_pad(ser, phy);
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_mbus_config_mipi_csi2 *mipi = &v4l2_ep.bus.mipi_csi2;
	struct fwnode_handle *ep;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, pad, 0, 0);
	if (!ep)
		return 0;

	source->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
	if (!source->ep_fwnode) {
		dev_err(priv->dev,
			"Failed to get remote endpoint on port %u\n", pad);
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(ep, &v4l2_ep);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse endpoint on port %u\n", pad);
		return ret;
	}

	if (mipi->flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK &&
	    !ser->ops->supports_noncontinuous_clock) {
		dev_err(priv->dev,
			"Clock non-continuous mode is not supported on port %u\n", pad);
		return -EINVAL;
	}

	phy->mipi = v4l2_ep.bus.mipi_csi2;
	phy->enabled = true;

	return 0;
}

static int max_ser_find_phys_config(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	const struct max_phys_configs *configs = &ser->ops->phys_configs;
	struct max_ser_phy *phy;
	unsigned int i, j;

	if (!configs->num_configs)
		return 0;

	for (i = 0; i < configs->num_configs; i++) {
		const struct max_phys_config *config = &configs->configs[i];
		bool matching = true;

		for (j = 0; j < ser->ops->num_phys; j++) {
			phy = &ser->phys[j];

			if (!phy->enabled)
				continue;

			if (phy->mipi.num_data_lanes <= config->lanes[j])
				continue;

			matching = false;

			break;
		}

		if (matching)
			break;
	}

	if (i == configs->num_configs) {
		dev_err(priv->dev, "Invalid lane configuration\n");
		return -EINVAL;
	}

	ser->phys_config = i;

	return 0;
}

static int max_ser_parse_dt(struct max_ser_priv *priv)
{
	struct fwnode_handle *fwnode = dev_fwnode(priv->dev);
	struct max_ser *ser = priv->ser;
	struct max_ser_pipe *pipe;
	struct max_ser_phy *phy;
	unsigned int i;
	int ret;

	for (i = 0; i < ser->ops->num_phys; i++) {
		phy = &ser->phys[i];
		phy->index = i;
	}

	for (i = 0; i < ser->ops->num_pipes; i++) {
		pipe = &ser->pipes[i];
		pipe->index = i;
		pipe->phy_id = i % ser->ops->num_phys;
		pipe->stream_id = i % MAX_SERDES_STREAMS_NUM;
	}

	for (i = 0; i < ser->ops->num_phys; i++) {
		struct max_ser_phy *phy = &ser->phys[i];
		struct max_source *source;

		source = max_ser_find_phy_source(priv, phy);
		if (!source)
			return -ENOENT;

		source->index = i;

		ret = max_ser_parse_sink_dt_endpoint(priv, phy, source, fwnode);
		if (ret)
			return ret;
	}

	return max_ser_find_phys_config(priv);
}

static int max_ser_allocate(struct max_ser_priv *priv)
{
	struct max_ser *ser = priv->ser;
	unsigned int num_pads = max_ser_num_pads(ser);

	ser->phys = devm_kcalloc(priv->dev, ser->ops->num_phys,
				 sizeof(*ser->phys), GFP_KERNEL);
	if (!ser->phys)
		return -ENOMEM;

	ser->pipes = devm_kcalloc(priv->dev, ser->ops->num_pipes,
				  sizeof(*ser->pipes), GFP_KERNEL);
	if (!ser->pipes)
		return -ENOMEM;

	ser->i2c_xlates = devm_kcalloc(priv->dev, ser->ops->num_i2c_xlates,
				       sizeof(*ser->i2c_xlates), GFP_KERNEL);
	if (!ser->i2c_xlates)
		return -ENOMEM;

	priv->sources = devm_kcalloc(priv->dev, ser->ops->num_phys,
				     sizeof(*priv->sources), GFP_KERNEL);
	if (!priv->sources)
		return -ENOMEM;

	priv->streams_masks = devm_kcalloc(priv->dev, num_pads,
					   sizeof(*priv->streams_masks),
					   GFP_KERNEL);
	if (!priv->streams_masks)
		return -ENOMEM;

	return 0;
}

int max_ser_probe(struct i2c_client *client, struct max_ser *ser)
{
	struct device *dev = &client->dev;
	struct max_ser_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	priv->dev = dev;
	priv->ser = ser;
	ser->priv = priv;

	ret = max_ser_allocate(priv);
	if (ret)
		return ret;

	ret = max_ser_parse_dt(priv);
	if (ret)
		return ret;

	ret = max_ser_init(priv);
	if (ret)
		return ret;

	ret = max_ser_i2c_adapter_init(priv);
	if (ret)
		return ret;

	ret = max_ser_v4l2_register(priv);
	if (ret)
		goto err_i2c_adapter_deinit;

	return 0;

err_i2c_adapter_deinit:
	max_ser_i2c_adapter_deinit(priv);

	return ret;
}
EXPORT_SYMBOL_GPL(max_ser_probe);

int max_ser_remove(struct max_ser *ser)
{
	struct max_ser_priv *priv = ser->priv;

	max_ser_v4l2_unregister(priv);

	max_ser_i2c_adapter_deinit(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_remove);

int max_ser_set_double_bpps(struct v4l2_subdev *sd, u32 double_bpps)
{
	struct max_ser_priv *priv = sd_to_priv(sd);

	priv->double_bpps = double_bpps;

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_set_double_bpps);

int max_ser_set_stream_id(struct v4l2_subdev *sd, unsigned int stream_id)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	struct max_ser_pipe *pipe = &ser->pipes[0];

	return ser->ops->set_pipe_stream_id(ser, pipe, stream_id);
}
EXPORT_SYMBOL_GPL(max_ser_set_stream_id);

bool max_ser_supports_tunnel_mode(struct v4l2_subdev *sd)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;

	if (!ser->ops->set_tunnel_enable)
		return false;

	return true;
}
EXPORT_SYMBOL(max_ser_supports_tunnel_mode);

int max_ser_set_tunnel_enable(struct v4l2_subdev *sd, bool enable)
{
	struct max_ser_priv *priv = sd_to_priv(sd);
	struct max_ser *ser = priv->ser;
	int ret;

	if (!ser->ops->set_tunnel_enable)
		return 0;

	if (ser->tunnel == enable)
		return 0;

	ret = ser->ops->set_tunnel_enable(ser, enable);
	if (ret)
		return ret;

	ser->tunnel = enable;

	return 0;
}
EXPORT_SYMBOL(max_ser_set_tunnel_enable);

static int max_ser_read_reg(struct i2c_adapter *adapter, u8 addr,
			    u16 reg, u8 *val)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg[2] = {
		{
			.addr = addr,
			.flags = 0,
			.buf = buf,
			.len = sizeof(buf),
		},
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = 1,
		},
	};
	int ret;

	ret = i2c_transfer(adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	*val = buf[0];

	return 0;
}

static int max_ser_write_reg(struct i2c_adapter *adapter, u8 addr,
			     u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_msg msg[1] = {
		{
			.addr = addr,
			.flags = 0,
			.buf = buf,
			.len = sizeof(buf),
		},
	};
	int ret;

	ret = i2c_transfer(adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

int max_ser_reset(struct i2c_adapter *adapter, u8 addr)
{
	int ret;
	u8 val;

	ret = max_ser_read_reg(adapter, addr, MAX_SER_CTRL0, &val);
	if (ret)
		return ret;

	val |= MAX_SER_CTRL0_RESET_ALL;

	return max_ser_write_reg(adapter, addr, MAX_SER_CTRL0, val);
}
EXPORT_SYMBOL_GPL(max_ser_reset);

int max_ser_wait_for_multiple(struct i2c_adapter *adapter, u8 *addrs,
			      unsigned int num_addrs, u8 *current_addr)
{
	unsigned int i, j;
	int ret = 0;
	u8 val;

	for (i = 0; i < 10; i++) {
		for (j = 0; j < num_addrs; j++) {
			ret = max_ser_read_reg(adapter, addrs[j], MAX_SER_REG0, &val);
			if (!ret && val) {
				*current_addr = addrs[j];
				return 0;
			}

			msleep(100);
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(max_ser_wait_for_multiple);

int max_ser_wait(struct i2c_adapter *adapter, u8 addr)
{
	u8 current_addr;

	return max_ser_wait_for_multiple(adapter, &addr, 1, &current_addr);
}
EXPORT_SYMBOL_GPL(max_ser_wait);

int max_ser_fix_tx_ids(struct i2c_adapter *adapter, u8 addr)
{
	unsigned int addr_regs[] = {
		MAX_SER_CFGI_INFOFR_TR3,
		MAX_SER_CFGL_SPI_TR3,
		MAX_SER_CFGC_CC_TR3,
		MAX_SER_CFGC_GPIO_TR3,
		MAX_SER_CFGL_IIC_X_TR3,
		MAX_SER_CFGL_IIC_Y_TR3,
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(addr_regs); i++) {
		ret = max_ser_write_reg(adapter, addr, addr_regs[i], addr);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max_ser_fix_tx_ids);

int max_ser_change_address(struct i2c_adapter *adapter, u8 addr, u8 new_addr)
{
	int ret;
	u8 val;

	ret = max_ser_read_reg(adapter, addr, MAX_SER_REG0, &val);
	if (ret)
		return ret;

	val |= FIELD_PREP(MAX_SER_REG0_DEV_ADDR, new_addr);

	return max_ser_write_reg(adapter, addr, MAX_SER_REG0, val);
}
EXPORT_SYMBOL_GPL(max_ser_change_address);

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("I2C_ATR");
