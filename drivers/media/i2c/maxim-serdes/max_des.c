// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Deserializer Driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/i2c-atr.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max_des.h"
#include "max_ser.h"
#include "max_serdes.h"

#define MAX_DES_LINK_FREQUENCY_MIN 100000000ull
#define MAX_DES_LINK_FREQUENCY_DEFAULT 750000000ull
#define MAX_DES_LINK_FREQUENCY_MAX 1250000000ull

#define MAX_DES_PHYS_NUM		4
#define MAX_DES_PIPES_NUM		8

struct max_des_priv {
	struct max_des *des;

	struct device *dev;
	struct i2c_client *client;
	struct i2c_atr *atr;
	struct i2c_mux_core *mux;

	struct media_pad *pads;
	struct regulator **pocs;
	struct max_source *sources;
	u64 *streams_masks;

	struct notifier_block i2c_nb;
	struct v4l2_subdev sd;
	struct v4l2_async_notifier nf;

	struct max_des_phy *unused_phy;
};

struct max_des_remap_context {
	/* Mark whether chip can function in tunnel mode. */
	bool tunnel_enable;
	/* Mark the PHYs to which each pipe is mapped. */
	unsigned long pipe_phy_masks[MAX_DES_PIPES_NUM];
	/* Mark whether pipe has remapped VC ids. */
	bool vc_ids_remapped[MAX_DES_PIPES_NUM];
	/* Map between pipe VC ids and PHY VC ids. */
	unsigned int vc_ids_map[MAX_DES_PIPES_NUM][MAX_DES_PHYS_NUM][MAX_SERDES_VC_ID_NUM];
	/* Mark whether a pipe VC id has been mapped to a PHY VC id. */
	unsigned long vc_ids_masks[MAX_DES_PIPES_NUM][MAX_DES_PHYS_NUM];
	/* Mark whether a PHY VC id has been mapped. */
	unsigned long dst_vc_ids_masks[MAX_DES_PHYS_NUM];
};

struct max_des_mode_context {
	bool phys_bpp8_shared_with_16[MAX_DES_PHYS_NUM];
	bool pipes_bpp8_shared_with_16[MAX_DES_PIPES_NUM];
	u32 phys_double_bpps[MAX_DES_PHYS_NUM];
	u32 pipes_double_bpps[MAX_DES_PIPES_NUM];
};

static const char *max_des_gmsl_versions[] = {
	[MAX_GMSL_2_3Gbps] = "GMSL2 3Gbps",
	[MAX_GMSL_2_6Gbps] = "GMSL2 6Gbps",
	[MAX_GMSL_3] = "GMSL3",
};

static inline struct max_des_priv *sd_to_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max_des_priv, sd);
}

static inline struct max_des_priv *nf_to_priv(struct v4l2_async_notifier *nf)
{
	return container_of(nf, struct max_des_priv, nf);
}

static inline bool max_des_pad_is_sink(struct max_des *des, u32 pad)
{
	return pad < des->ops->num_links;
}

static inline bool max_des_pad_is_source(struct max_des *des, u32 pad)
{
	return pad >= des->ops->num_links;
}

static inline unsigned int max_des_link_to_pad(struct max_des *des,
					       struct max_des_link *link)
{
	return link->index;
}

static inline unsigned int max_des_phy_to_pad(struct max_des *des,
					      struct max_des_phy *phy)
{
	return phy->index + des->ops->num_links;
}

static inline unsigned int max_des_num_pads(struct max_des *des)
{
	return des->ops->num_links + des->ops->num_phys;
}

static struct max_des_phy *max_des_pad_to_phy(struct max_des *des, u32 pad)
{
	if (!max_des_pad_is_source(des, pad))
		return NULL;

	return &des->phys[pad - des->ops->num_links];
}

static struct max_des_link *max_des_pad_to_link(struct max_des *des, u32 pad)
{
	if (!max_des_pad_is_sink(des, pad))
		return NULL;

	return &des->links[pad];
}

static struct max_des_pipe *
max_des_find_link_pipe(struct max_des *des, struct max_des_link *link)
{
	unsigned int i;

	for (i = 0; i < des->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &des->pipes[i];

		if (pipe->link_id == link->index)
			return pipe;
	}

	return NULL;
}

static struct max_source *
max_des_find_link_source(struct max_des_priv *priv, struct max_des_link *link)
{
	return &priv->sources[link->index];
}

static int max_des_set_pipe_remaps(struct max_des_priv *priv,
				   struct max_des_pipe *pipe,
				   struct max_des_remap *remaps,
				   unsigned int num_remaps)
{
	struct max_des *des = priv->des;
	unsigned int mask = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < num_remaps; i++) {
		struct max_des_remap *remap = &remaps[i];

		ret = des->ops->set_pipe_remap(des, pipe, i, remap);
		if (ret)
			return ret;

		mask |= BIT(i);
	}

	return des->ops->set_pipe_remaps_enable(des, pipe, mask);
}

static int max_des_set_phy_active(struct max_des *des, struct max_des_phy *phy,
				  bool active)
{
	int ret;

	ret = des->ops->set_phy_active(des, phy, active);
	if (ret)
		return ret;

	phy->active = active;

	return 0;
}

static int max_des_set_pipe_enable(struct max_des *des, struct max_des_pipe *pipe,
				   bool enable)
{
	int ret;

	ret = des->ops->set_pipe_enable(des, pipe, enable);
	if (ret)
		return ret;

	pipe->enabled = enable;

	return 0;
}

static int max_des_map_src_dst_vc_id(struct max_des_remap_context *context,
				     unsigned int pipe_id, unsigned int phy_id,
				     unsigned int src_vc_id, unsigned int *dst_vc_id)
{
	unsigned int vc_id;

	if (src_vc_id >= MAX_SERDES_VC_ID_NUM)
		return -E2BIG;

	if (context->vc_ids_masks[pipe_id][phy_id] & BIT(src_vc_id)) {
		*dst_vc_id = context->vc_ids_map[pipe_id][phy_id][src_vc_id];
		return 0;
	}

	if (!(context->dst_vc_ids_masks[phy_id] & BIT(src_vc_id))) {
		vc_id = src_vc_id;
	} else {
		context->vc_ids_remapped[pipe_id] = true;
		vc_id = ffz(context->dst_vc_ids_masks[phy_id]);
	}

	if (vc_id >= MAX_SERDES_VC_ID_NUM)
		return -E2BIG;

	context->pipe_phy_masks[pipe_id] |= BIT(phy_id);
	context->dst_vc_ids_masks[phy_id] |= BIT(vc_id);

	context->vc_ids_map[pipe_id][phy_id][src_vc_id] = vc_id;
	context->vc_ids_masks[pipe_id][phy_id] |= BIT(src_vc_id);

	*dst_vc_id = vc_id;

	return 0;
}

static int max_des_populate_remap_context(struct max_des_priv *priv,
					  struct max_des_remap_context *context,
					  const struct v4l2_subdev_krouting *routing)
{
	struct max_des *des = priv->des;
	struct v4l2_subdev_route *route;
	unsigned int link_id;
	bool tunnel_enable;
	int ret;

	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		struct max_source *source;
		struct max_des_link *link;
		struct max_des_pipe *pipe;
		struct max_des_phy *phy;
		unsigned int vc_id;

		link = max_des_pad_to_link(des, route->sink_pad);
		if (!link) {
			dev_err(priv->dev, "Failed to find link for pad %u\n",
				route->sink_pad);
			return -ENOENT;
		}

		phy = max_des_pad_to_phy(des, route->source_pad);
		if (!phy) {
			dev_err(priv->dev, "Failed to find PHY for pad %u\n",
				route->source_pad);
			return -ENOENT;
		}

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		source = max_des_find_link_source(priv, link);
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

		ret = max_des_map_src_dst_vc_id(context, pipe->index, phy->index,
						entry.bus.csi2.vc, &vc_id);
		if (ret)
			return ret;
	}

	if (!des->ops->set_pipe_tunnel_enable)
		return 0;

	tunnel_enable = true;

	for (link_id = 0; link_id < des->ops->num_links; link_id++) {
		struct max_des_link *link = &des->links[link_id];
		struct max_des_pipe *pipe;
		struct max_source *source;

		if (!link->enabled)
			continue;

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		source = max_des_find_link_source(priv, link);
		if (!source)
			return -ENOENT;

		if (!source->sd)
			continue;

		if (max_ser_supports_tunnel_mode(source->sd) &&
		    hweight_long(context->pipe_phy_masks[pipe->index]) <= 1 &&
		    !context->vc_ids_remapped[pipe->index])
			continue;

		tunnel_enable = false;
	}

	context->tunnel_enable = tunnel_enable;

	return 0;
}

static int max_des_populate_mode_context(struct max_des_priv *priv,
					 struct max_des_mode_context *context,
					 const struct v4l2_subdev_krouting *routing)
{
	bool bpp8_not_shared_with_16_phys[MAX_DES_PHYS_NUM] = { 0 };
	u32 undoubled_bpps_phys[MAX_DES_PHYS_NUM] = { 0 };
	struct max_des *des = priv->des;
	struct v4l2_subdev_route *route;
	struct max_des_link *link;
	struct max_des_pipe *pipe;
	struct max_des_phy *phy;
	unsigned int doubled_bpp;
	unsigned int bpp;
	unsigned int i;
	u32 stream_bpps;
	u32 sink_bpps;
	int ret;

	/*
	 * Go over all streams and check if the current stream is doubled.
	 *
	 * If the current stream is doubled, add it to a doubled mask for both
	 * the pipe and the PHY.
	 *
	 * If the current stream is not doubled, add it to a local undoubled
	 * mask for the PHY.
	 *
	 * Also, track whether an 8bpp stream is shared with any bpp > 8 on both
	 * the PHYs and the pipes, since that needs to be special cased.
	 *
	 * After going over all the streams, remove the undoubled streams from
	 * the doubled ones. Doubled and undoubled streams cannot be streamed
	 * over the same PHY.
	 *
	 * Then, do a second pass to remove the undoubled streams from the pipes.
	 *
	 * This operation cannot be done in a single pass because any pipe might
	 * generate an undoubled stream for a specific bpp, causing already
	 * processed pipes to need to have their doubled bpps updated.
	 */

	for_each_active_route(routing, route) {
		unsigned int min_bpp;
		unsigned int max_bpp;

		phy = max_des_pad_to_phy(des, route->source_pad);
		if (!phy)
			return -ENOENT;

		link = max_des_pad_to_link(des, route->sink_pad);
		if (!link)
			return -ENOENT;

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		ret = max_get_bpps(priv->sources, 0, &stream_bpps,
				   routing, route->sink_pad,
				   BIT_ULL(route->sink_stream));
		if (ret)
			return ret;

		ret = max_get_bpps(priv->sources, 0, &sink_bpps,
				   routing, route->sink_pad, ~0ULL);
		if (ret)
			return ret;

		ret = max_process_bpps(priv->dev, sink_bpps, ~0U, &doubled_bpp);
		if (ret)
			return ret;

		bpp = __ffs(stream_bpps);
		min_bpp = __ffs(sink_bpps);
		max_bpp = __fls(sink_bpps);

		if (bpp == doubled_bpp) {
			context->phys_double_bpps[phy->index] |= BIT(bpp);
			context->pipes_double_bpps[pipe->index] |= BIT(bpp);
		} else {
			undoubled_bpps_phys[phy->index] |= BIT(bpp);
		}

		if (min_bpp == 8 && max_bpp > 8) {
			context->phys_bpp8_shared_with_16[phy->index] = true;
			context->pipes_bpp8_shared_with_16[pipe->index] = true;
		} else if (min_bpp == 8 && max_bpp == 8) {
			bpp8_not_shared_with_16_phys[phy->index] = true;
		}
	}

	for (i = 0; i < des->ops->num_phys; i++) {
		if (context->phys_bpp8_shared_with_16[i] && bpp8_not_shared_with_16_phys[i]) {
			dev_err(priv->dev,
				"Cannot stream 8bpp coming from pipes padded to 16bpp "
				"and pipes not padded to 16bpp on the same PHY\n");
			return -EINVAL;
		}
	}

	for (i = 0; i < des->ops->num_phys; i++)
		context->phys_double_bpps[i] &= ~undoubled_bpps_phys[i];

	for_each_active_route(routing, route) {
		struct max_des_link *link;
		struct max_des_pipe *pipe;
		struct max_des_phy *phy;

		phy = max_des_pad_to_phy(des, route->source_pad);
		if (!phy)
			return -ENOENT;

		link = max_des_pad_to_link(des, route->sink_pad);
		if (!link)
			return -ENOENT;

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		context->pipes_double_bpps[pipe->index] &=
			context->phys_double_bpps[phy->index];
	}

	return 0;
}

static void max_des_get_pipe_mode(struct max_des_mode_context *context,
				  struct max_des_pipe *pipe,
				  struct max_des_pipe_mode *mode)
{
	u32 double_bpps = context->pipes_double_bpps[pipe->index];

	if ((double_bpps & BIT(8)) &&
	    !context->pipes_bpp8_shared_with_16[pipe->index]) {
		mode->dbl8 = true;
		mode->dbl8mode = true;
	}
}

static void max_des_get_phy_mode(struct max_des_mode_context *context,
				 struct max_des_phy *phy,
				 struct max_des_phy_mode *mode)
{
	bool bpp8_pipe_shared_with_16 = context->phys_bpp8_shared_with_16[phy->index];
	u32 double_bpps = context->phys_double_bpps[phy->index];

	if (BIT(8) & double_bpps) {
		if (bpp8_pipe_shared_with_16)
			mode->alt2_mem_map8 = true;
		else
			mode->alt_mem_map8 = true;
	}

	if (BIT(10) & double_bpps)
		mode->alt_mem_map10 = true;

	if (BIT(12) & double_bpps)
		mode->alt_mem_map12 = true;
}

static int max_des_set_modes(struct max_des_priv *priv,
			     struct max_des_remap_context *context,
			     struct max_des_mode_context *mode_context)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	for (i = 0; i < des->ops->num_phys; i++) {
		struct max_des_phy *phy = &des->phys[i];
		struct max_des_phy_mode mode = { 0 };

		if (!context->tunnel_enable)
			max_des_get_phy_mode(mode_context, phy, &mode);

		if (phy->mode.alt_mem_map8 == mode.alt_mem_map8 &&
		    phy->mode.alt_mem_map10 == mode.alt_mem_map10 &&
		    phy->mode.alt_mem_map12 == mode.alt_mem_map12 &&
		    phy->mode.alt2_mem_map8 == mode.alt2_mem_map8)
			continue;

		ret = des->ops->set_phy_mode(des, phy, &mode);
		if (ret)
			return ret;

		phy->mode = mode;
	}

	for (i = 0; i < des->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &des->pipes[i];
		struct max_des_pipe_mode mode = { 0 };

		if (!context->tunnel_enable)
			max_des_get_pipe_mode(mode_context, pipe, &mode);

		if (pipe->mode.dbl8 == mode.dbl8 &&
		    pipe->mode.dbl10 == mode.dbl10 &&
		    pipe->mode.dbl12 == mode.dbl12 &&
		    pipe->mode.dbl8mode == mode.dbl8mode &&
		    pipe->mode.dbl10mode == mode.dbl10mode)
			continue;

		ret = des->ops->set_pipe_mode(des, pipe, &mode);
		if (ret)
			return ret;

		pipe->mode = mode;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct max_des_pipe *pipe;
		struct max_source *source;
		u32 pipe_double_bpps = 0;

		if (!link->enabled)
			continue;

		source = max_des_find_link_source(priv, link);
		if (!source)
			return -ENOENT;

		if (!source->sd)
			continue;

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		if (!source->sd)
			continue;

		if (!context->tunnel_enable)
			pipe_double_bpps = mode_context->pipes_double_bpps[pipe->index];

		ret = max_ser_set_double_bpps(source->sd, pipe_double_bpps);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_set_tunnel(struct max_des_priv *priv,
			      struct max_des_remap_context *context)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	if (des->tunnel == context->tunnel_enable)
		return 0;

	if (des->ops->set_pipe_tunnel_enable) {
		for (i = 0; i < des->ops->num_pipes; i++) {
			struct max_des_pipe *pipe = &des->pipes[i];

			ret = des->ops->set_pipe_tunnel_enable(des, pipe,
							       context->tunnel_enable);
			if (ret)
				return ret;
		}
	}

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct max_source *source;

		if (!link->enabled)
			continue;

		source = max_des_find_link_source(priv, link);
		if (!source)
			return -ENOENT;

		if (!source->sd)
			continue;

		ret = max_ser_set_tunnel_enable(source->sd, context->tunnel_enable);
		if (ret)
			return ret;
	}

	des->tunnel = context->tunnel_enable;

	return 0;
}

static int max_des_set_pipes_stream_id(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	if (!des->ops->needs_unique_stream_id)
		return 0;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct max_des_pipe *pipe;
		struct max_source *source;

		if (!link->enabled)
			continue;

		source = max_des_find_link_source(priv, link);
		if (!source)
			return -ENOENT;

		if (!source->sd)
			continue;

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		if (!source->sd)
			continue;

		ret = max_ser_set_stream_id(source->sd, pipe->stream_id);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_set_pipes_phy(struct max_des_priv *priv,
				 struct max_des_remap_context *context)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	if (!des->ops->set_pipe_phy)
		return 0;

	for (i = 0; i < des->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &des->pipes[i];
		struct max_des_phy *phy;
		unsigned int phy_id;

		phy_id = find_first_bit(&context->pipe_phy_masks[pipe->index],
					des->ops->num_phys);

		if (priv->unused_phy &&
		    (!context->tunnel_enable || phy_id == des->ops->num_phys))
			phy_id = priv->unused_phy->index;

		if (phy_id != des->ops->num_phys) {
			phy = &des->phys[phy_id];

			ret = des->ops->set_pipe_phy(des, pipe, phy);
			if (ret)
				return ret;
		}

		pipe->phy_id = phy_id;
	}

	return 0;
}

static int max_des_add_remap(struct max_des_remap *remaps,
			     unsigned int *num_remaps, unsigned int phy_id,
			     unsigned int src_vc_id, unsigned int dst_vc_id,
			     unsigned int dt)
{
	struct max_des_remap *remap = &remaps[*num_remaps];

	remap->from_dt = dt;
	remap->from_vc = src_vc_id;
	remap->to_dt = dt;
	remap->to_vc = dst_vc_id;
	remap->phy = phy_id;

	(*num_remaps)++;

	return 0;
}

static int max_des_get_pipe_remaps(struct max_des_priv *priv,
				   struct max_des_remap_context *context,
				   struct max_des_pipe *pipe,
				   struct max_source *source,
				   struct max_des_remap *remaps,
				   unsigned int *num_remaps,
				   const struct v4l2_subdev_krouting *routing,
				   u32 pad, u64 streams_mask)
{
	struct max_des *des = priv->des;
	struct v4l2_subdev_route *route;
	unsigned long vc_ids_masks = 0;
	unsigned int phy_id;
	int ret;

	*num_remaps = 0;

	if (context->tunnel_enable)
		return 0;

	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		struct max_des_phy *phy;
		unsigned int src_vc_id, dst_vc_id;

		if (pad != route->sink_pad)
			continue;

		if (!(BIT_ULL(route->sink_stream) & streams_mask))
			continue;

		if (*num_remaps + 1 > des->ops->num_remaps_per_pipe) {
			dev_err(priv->dev, "Too many streams for pipe %u\n",
				pipe->index);
			return -E2BIG;
		}

		phy = max_des_pad_to_phy(des, route->source_pad);
		if (!phy) {
			dev_err(priv->dev, "Failed to find PHY for pad %u\n",
				route->source_pad);
			return -ENOENT;
		}

		ret = max_get_fd_stream_entry(source->sd, source->pad,
					      route->sink_stream, &entry);
		if (ret) {
			dev_err(priv->dev,
				"Failed to find frame desc entry for stream %u:%u: %d\n",
				route->sink_pad, route->sink_stream, ret);
			return ret;
		}

		src_vc_id = entry.bus.csi2.vc;
		vc_ids_masks |= BIT(src_vc_id);

		ret = max_des_map_src_dst_vc_id(context, pipe->index, phy->index,
						src_vc_id, &dst_vc_id);
		if (ret)
			return ret;

		ret = max_des_add_remap(remaps, num_remaps, phy->index,
					src_vc_id, dst_vc_id,
					entry.bus.csi2.dt);
		if (ret)
			return ret;
	}

	for (phy_id = 0; phy_id < des->ops->num_phys; phy_id++) {
		unsigned long mask = context->vc_ids_masks[pipe->index][phy_id];
		unsigned int src_vc_id;

		for_each_set_bit(src_vc_id, &mask, MAX_SERDES_VC_ID_NUM) {
			unsigned int dst_vc_id;

			if (!(vc_ids_masks & BIT(src_vc_id)))
				continue;

			if (*num_remaps + 2 > des->ops->num_remaps_per_pipe) {
				dev_err(priv->dev, "Too many streams for pipe %u\n",
					pipe->index);
				return -E2BIG;
			}

			ret = max_des_map_src_dst_vc_id(context, pipe->index, phy_id,
							src_vc_id, &dst_vc_id);
			if (ret)
				return ret;

			ret = max_des_add_remap(remaps, num_remaps, phy_id,
						src_vc_id, dst_vc_id,
						MIPI_CSI2_DT_FS);
			if (ret)
				return ret;

			ret = max_des_add_remap(remaps, num_remaps, phy_id,
						src_vc_id, dst_vc_id,
						MIPI_CSI2_DT_FE);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int max_des_update_pipe(struct max_des_priv *priv,
			       struct max_des_remap_context *context,
			       struct max_source *source,
			       struct max_des_pipe *pipe,
			       const struct v4l2_subdev_krouting *routing,
			       u32 pad, u64 streams_mask)
{
	struct max_des *des = priv->des;
	struct max_des_remap *remaps;
	unsigned int num_remaps;
	int ret;

	remaps = devm_kcalloc(priv->dev, des->ops->num_remaps_per_pipe,
			      sizeof(*remaps), GFP_KERNEL);
	if (!remaps)
		return -ENOMEM;

	ret = max_des_get_pipe_remaps(priv, context, pipe, source, remaps,
				      &num_remaps, routing, pad, streams_mask);
	if (ret)
		goto err_free_new_remaps;

	ret = max_des_set_pipe_remaps(priv, pipe, remaps, num_remaps);
	if (ret)
		goto err_free_new_remaps;

	if (pipe->remaps)
		devm_kfree(priv->dev, pipe->remaps);

	pipe->remaps = remaps;
	pipe->num_remaps = num_remaps;

	return 0;

err_free_new_remaps:
	devm_kfree(priv->dev, remaps);

	return ret;
}

static int max_des_init_link_ser_xlate(struct max_des_priv *priv,
				       struct max_des_link *link,
				       struct i2c_adapter *adapter,
				       u8 power_up_addr, u8 new_addr)
{
	struct max_des *des = priv->des;
	u8 addrs[] = { power_up_addr, new_addr };
	u8 current_addr;
	int ret;

	ret = des->ops->select_links(des, BIT(link->index));
	if (ret)
		return ret;

	ret = max_ser_wait_for_multiple(adapter, addrs, ARRAY_SIZE(addrs),
					&current_addr);
	if (ret) {
		dev_err(priv->dev,
			"Failed to wait for serializer at 0x%02x or 0x%02x: %d\n",
			power_up_addr, new_addr, ret);
		return ret;
	}

	ret = max_ser_reset(adapter, current_addr);
	if (ret) {
		dev_err(priv->dev, "Failed to reset serializer: %d\n", ret);
		return ret;
	}

	ret = max_ser_wait(adapter, power_up_addr);
	if (ret) {
		dev_err(priv->dev,
			"Failed to wait for serializer at 0x%02x: %d\n",
			power_up_addr, ret);
		return ret;
	}

	ret = max_ser_change_address(adapter, power_up_addr, new_addr);
	if (ret) {
		dev_err(priv->dev,
			"Failed to change serializer from 0x%02x to 0x%02x: %d\n",
			power_up_addr, new_addr, ret);
		return ret;
	}

	ret = max_ser_wait(adapter, new_addr);
	if (ret) {
		dev_err(priv->dev,
			"Failed to wait for serializer at 0x%02x: %d\n",
			new_addr, ret);
		return ret;
	}

	if (des->ops->fix_tx_ids) {
		ret = max_ser_fix_tx_ids(adapter, new_addr);
		if (ret)
			return ret;
	}

	return ret;
}

static int max_des_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	if (des->ops->init) {
		ret = des->ops->init(des);
		if (ret)
			return ret;
	}

	ret = des->ops->set_enable(des, false);
	if (ret)
		return ret;

	for (i = 0; i < des->ops->num_phys; i++) {
		struct max_des_phy *phy = &des->phys[i];

		if (phy->enabled) {
			ret = des->ops->init_phy(des, phy);
			if (ret)
				return ret;
		}

		ret = des->ops->set_phy_active(des, phy, false);
		if (ret)
			return ret;
	}

	for (i = 0; i < des->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &des->pipes[i];

		ret = des->ops->set_pipe_enable(des, pipe, false);
		if (ret)
			return ret;

		if (des->ops->set_pipe_tunnel_enable) {
			ret = des->ops->set_pipe_tunnel_enable(des, pipe, false);
			if (ret)
				return ret;
		}

		ret = des->ops->set_pipe_stream_id(des, pipe, pipe->stream_id);
		if (ret)
			return ret;

		ret = max_des_set_pipe_remaps(priv, pipe, pipe->remaps,
					      pipe->num_remaps);
		if (ret)
			return ret;
	}

	if (!des->ops->init_link)
		return 0;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		if (!link->enabled)
			continue;

		ret = des->ops->init_link(des, link);
		if (ret)
			return ret;
	}

	return 0;
}

static void max_des_ser_find_version_range(struct max_des *des,
					   enum max_gmsl_version *min,
					   enum max_gmsl_version *max)
{
	unsigned int i;

	*min = MAX_GMSL_MIN;
	*max = MAX_GMSL_MAX;

	if (!des->ops->needs_single_link_version)
		return;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		if (!link->enabled)
			continue;

		if (!link->ser_xlate.en)
			continue;

		*min = *max = link->version;

		return;
	}
}

static int max_des_ser_attach_addr(struct max_des_priv *priv, u32 chan_id,
				   u16 addr, u16 alias)
{
	struct max_des *des = priv->des;
	struct max_des_link *link = &des->links[chan_id];
	enum max_gmsl_version max;
	enum max_gmsl_version min;
	int ret;
	int i;

	max_des_ser_find_version_range(des, &min, &max);

	if (link->ser_xlate.en) {
		dev_err(priv->dev, "Serializer for link %u already bound\n",
			link->index);
		return -EINVAL;
	}

	for (i = max; i >= min; i--) {
		if (!(des->ops->versions & BIT(i)))
			continue;

		if (des->ops->set_link_version) {
			ret = des->ops->set_link_version(des, link, i);
			if (ret)
				return ret;
		}

		ret = max_des_init_link_ser_xlate(priv, link, priv->client->adapter,
						  addr, alias);
		if (!ret)
			break;
	}

	if (ret) {
		dev_err(priv->dev, "Cannot find serializer for link %u\n",
			link->index);
		return -ENOENT;
	}

	link->version = i;
	link->ser_xlate.src = alias;
	link->ser_xlate.dst = addr;
	link->ser_xlate.en = true;

	return 0;
}

static int max_des_ser_atr_attach_addr(struct i2c_atr *atr, u32 chan_id,
				       u16 addr, u16 alias)
{
	struct max_des_priv *priv = i2c_atr_get_driver_data(atr);

	return max_des_ser_attach_addr(priv, chan_id, addr, alias);
}

static void max_des_ser_atr_detach_addr(struct i2c_atr *atr, u32 chan_id, u16 addr)
{
	/* Don't do anything. */
}

static const struct i2c_atr_ops max_des_i2c_atr_ops = {
	.attach_addr = max_des_ser_atr_attach_addr,
	.detach_addr = max_des_ser_atr_detach_addr,
};

static void max_des_i2c_atr_deinit(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int i;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		/* Deleting adapters that haven't been added does no harm. */
		i2c_atr_del_adapter(priv->atr, link->index);
	}

	i2c_atr_delete(priv->atr);
}

static int max_des_i2c_atr_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int mask = 0;
	unsigned int i;
	int ret;

	if (!i2c_check_functionality(priv->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	priv->atr = i2c_atr_new(priv->client->adapter, priv->dev,
				&max_des_i2c_atr_ops, des->ops->num_links,
				I2C_ATR_F_STATIC | I2C_ATR_F_PASSTHROUGH);
	if (IS_ERR(priv->atr))
		return PTR_ERR(priv->atr);

	i2c_atr_set_driver_data(priv->atr, priv);

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct i2c_atr_adap_desc desc = {
			.chan_id = i,
		};

		if (!link->enabled)
			continue;

		ret = i2c_atr_add_adapter(priv->atr, &desc);
		if (ret)
			goto err_add_adapters;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		if (!link->enabled)
			continue;

		mask |= BIT(link->index);
	}

	return des->ops->select_links(des, mask);

err_add_adapters:
	max_des_i2c_atr_deinit(priv);

	return ret;
}

static void max_des_i2c_mux_deinit(struct max_des_priv *priv)
{
	i2c_mux_del_adapters(priv->mux);
	bus_unregister_notifier(&i2c_bus_type, &priv->i2c_nb);
}

static int max_des_i2c_mux_bus_notifier_call(struct notifier_block *nb,
					     unsigned long event, void *device)
{
	struct max_des_priv *priv = container_of(nb, struct max_des_priv, i2c_nb);
	struct device *dev = device;
	struct i2c_client *client;
	u32 chan_id;

	/*
	 * Ideally, we would want to negotiate the GMSL version on
	 * BUS_NOTIFY_ADD_DEVICE, but the adapters list is only populated with
	 * the new adapter after BUS_NOTIFY_ADD_DEVICE is issued.
	 */
	if (event != BUS_NOTIFY_BIND_DRIVER)
		return NOTIFY_DONE;

	client = i2c_verify_client(dev);
	if (!client)
		return NOTIFY_DONE;

	for (chan_id = 0; chan_id < priv->mux->max_adapters; ++chan_id) {
		if (client->adapter == priv->mux->adapter[chan_id])
			break;
	}

	if (chan_id == priv->mux->max_adapters)
		return NOTIFY_DONE;

	max_des_ser_attach_addr(priv, chan_id, client->addr, client->addr);

	return NOTIFY_DONE;
}

static int max_des_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max_des_priv *priv = i2c_mux_priv(muxc);
	struct max_des *des = priv->des;

	if (!des->ops->select_links)
		return 0;

	return des->ops->select_links(des, BIT(chan));
}

static int max_des_i2c_mux_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	u32 flags = I2C_MUX_LOCKED;
	unsigned int i;
	int ret;

	if (des->ops->num_links == 1)
		flags |= I2C_MUX_GATE;

	priv->mux = i2c_mux_alloc(priv->client->adapter, priv->dev,
				  des->ops->num_links, 0, flags,
				  max_des_i2c_mux_select, NULL);
	if (!priv->mux)
		return -ENOMEM;

	priv->mux->priv = priv;

	priv->i2c_nb.notifier_call = max_des_i2c_mux_bus_notifier_call;
	ret = bus_register_notifier(&i2c_bus_type, &priv->i2c_nb);
	if (ret)
		return ret;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		if (!link->enabled)
			continue;

		ret = i2c_mux_add_adapter(priv->mux, 0, i);
		if (ret)
			goto err_add_adapters;
	}

	return 0;

err_add_adapters:
	max_des_i2c_mux_deinit(priv);

	return ret;
}

static void max_des_i2c_adapter_deinit(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;

	if (des->ops->use_atr)
		return max_des_i2c_atr_deinit(priv);
	else
		return max_des_i2c_mux_deinit(priv);
}

static int max_des_i2c_adapter_init(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;

	if (des->ops->use_atr)
		return max_des_i2c_atr_init(priv);
	else
		return max_des_i2c_mux_init(priv);

	return 0;
}

static int max_des_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;
	struct v4l2_mbus_framefmt *fmt;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && des->active)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (max_des_pad_is_source(des, format->pad))
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

static int max_des_log_status(struct v4l2_subdev *sd)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;
	unsigned int i, j;
	int ret;

	v4l2_info(sd, "active: %u\n", des->active);
	v4l2_info(sd, "tunnel: %u", des->tunnel);
	if (des->ops->log_status) {
		ret = des->ops->log_status(des, sd->name);
		if (ret)
			return ret;
	}
	v4l2_info(sd, "\n");

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		v4l2_info(sd, "link: %u\n", link->index);
		v4l2_info(sd, "\tenabled: %u\n", link->enabled);

		if (!link->enabled) {
			v4l2_info(sd, "\n");
			continue;
		}

		v4l2_info(sd, "\tversion: %s\n", max_des_gmsl_versions[link->version]);
		v4l2_info(sd, "\tser_xlate: en: %u, src: 0x%02x dst: 0x%02x\n",
			  link->ser_xlate.en, link->ser_xlate.src,
			  link->ser_xlate.dst);
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < des->ops->num_pipes; i++) {
		struct max_des_pipe *pipe = &des->pipes[i];

		v4l2_info(sd, "pipe: %u\n", pipe->index);
		v4l2_info(sd, "\tenabled: %u\n", pipe->enabled);
		if (pipe->phy_id == des->ops->num_phys)
			v4l2_info(sd, "\tphy_id: invalid\n");
		else
			v4l2_info(sd, "\tphy_id: %u\n", pipe->phy_id);
		v4l2_info(sd, "\tstream_id: %u\n", pipe->stream_id);
		v4l2_info(sd, "\tlink_id: %u\n", pipe->link_id);
		v4l2_info(sd, "\tdbl8: %u\n", pipe->mode.dbl8);
		v4l2_info(sd, "\tdbl8mode: %u\n", pipe->mode.dbl8mode);
		v4l2_info(sd, "\tdbl10: %u\n", pipe->mode.dbl10);
		v4l2_info(sd, "\tdbl10mode: %u\n", pipe->mode.dbl10mode);
		v4l2_info(sd, "\tdbl12: %u\n", pipe->mode.dbl12);
		v4l2_info(sd, "\tremaps: %u\n", pipe->num_remaps);
		for (j = 0; j < pipe->num_remaps; j++) {
			struct max_des_remap *remap = &pipe->remaps[j];

			v4l2_info(sd, "\t\tremap: from: vc: %u, dt: 0x%02x\n",
				  remap->from_vc, remap->from_dt);
			v4l2_info(sd, "\t\t       to:   vc: %u, dt: 0x%02x, phy: %u\n",
				  remap->to_vc, remap->to_dt, remap->phy);
		}
		if (des->ops->log_pipe_status) {
			ret = des->ops->log_pipe_status(des, pipe, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	for (i = 0; i < des->ops->num_phys; i++) {
		struct max_des_phy *phy = &des->phys[i];

		v4l2_info(sd, "phy: %u\n", phy->index);
		v4l2_info(sd, "\tenabled: %u\n", phy->enabled);

		if (!phy->enabled) {
			v4l2_info(sd, "\n");
			continue;
		}

		v4l2_info(sd, "\tactive: %u\n", phy->active);
		v4l2_info(sd, "\tlink_frequency: %llu\n", phy->link_frequency);
		v4l2_info(sd, "\tnum_data_lanes: %u\n", phy->mipi.num_data_lanes);
		v4l2_info(sd, "\tclock_lane: %u\n", phy->mipi.clock_lane);
		v4l2_info(sd, "\talt_mem_map8: %u\n", phy->mode.alt_mem_map8);
		v4l2_info(sd, "\talt2_mem_map8: %u\n", phy->mode.alt2_mem_map8);
		v4l2_info(sd, "\talt_mem_map10: %u\n", phy->mode.alt_mem_map10);
		v4l2_info(sd, "\talt_mem_map12: %u\n", phy->mode.alt_mem_map12);
		if (des->ops->log_phy_status) {
			ret = des->ops->log_phy_status(des, phy, sd->name);
			if (ret)
				return ret;
		}
		v4l2_info(sd, "\n");
	}

	return 0;
}

static int max_des_get_frame_desc_state(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_mbus_frame_desc *fd,
					unsigned int pad)
{
	struct max_des_remap_context context = { 0 };
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des *des = priv->des;
	struct v4l2_subdev_route *route;
	struct max_des_phy *phy;
	int ret;

	phy = max_des_pad_to_phy(des, pad);
	if (!phy) {
		dev_err(priv->dev, "Failed to find PHY for pad %u\n", pad);
		return -ENOENT;
	}

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	ret = max_des_populate_remap_context(priv, &context, &state->routing);
	if (ret)
		return ret;

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		struct max_source *source;
		struct max_des_link *link;
		struct max_des_pipe *pipe;
		unsigned int dst_vc_id;

		if (pad != route->source_pad)
			continue;

		link = max_des_pad_to_link(des, route->sink_pad);
		if (!link) {
			dev_err(priv->dev, "Failed to find link for pad %u\n",
				route->sink_pad);
			return -ENOENT;
		}

		pipe = max_des_find_link_pipe(des, link);
		if (!pipe)
			return -ENOENT;

		source = max_des_find_link_source(priv, link);
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

		ret = max_des_map_src_dst_vc_id(&context, pipe->index, phy->index,
						entry.bus.csi2.vc, &dst_vc_id);
		if (ret)
			return ret;

		entry.bus.csi2.vc = dst_vc_id;
		entry.stream = route->source_stream;

		fd->entry[fd->num_entries++] = entry;
	}

	return 0;
}

static int max_des_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct v4l2_subdev_state *state;
	int ret;

	state = v4l2_subdev_lock_and_get_active_state(&priv->sd);

	ret = max_des_get_frame_desc_state(sd, state, fd, pad);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int max_des_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_config *cfg)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des *des = priv->des;
	struct max_des_phy *phy;

	phy = max_des_pad_to_phy(des, pad);
	if (!phy)
		return -EINVAL;

	cfg->type = phy->bus_type;
	cfg->bus.mipi_csi2 = phy->mipi;
	cfg->link_freq = phy->link_frequency;

	return 0;
}

static int max_des_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       enum v4l2_subdev_format_whence which,
			       struct v4l2_subdev_krouting *routing)
{
	struct max_des_priv *priv = sd_to_priv(sd);
	struct max_des *des = priv->des;
	int ret;

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && des->active)
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

static int max_des_update_link(struct max_des_priv *priv,
			       struct max_des_remap_context *context,
			       struct max_des_link *link,
			       const struct v4l2_subdev_krouting *routing,
			       u64 streams_mask)
{
	struct max_source *source;
	struct max_des *des = priv->des;
	u32 pad = max_des_link_to_pad(des, link);
	bool enable_changed = !streams_mask != !priv->streams_masks[pad];
	bool enable = !!streams_mask;
	struct max_des_pipe *pipe;
	int ret;

	pipe = max_des_find_link_pipe(des, link);
	if (!pipe)
		return -ENOENT;

	source = max_des_find_link_source(priv, link);
	if (!source)
		return -ENOENT;

	if (!source->sd)
		return 0;

	if (!enable && enable_changed) {
		ret = max_des_set_pipe_enable(des, pipe, enable);
		if (ret)
			return ret;
	}

	ret = max_des_update_pipe(priv, context, source, pipe,
				  routing, pad, streams_mask);
	if (ret)
		goto err_revert_pipe_disable;

	if (enable && enable_changed) {
		ret = max_des_set_pipe_enable(des, pipe, enable);
		if (ret)
			goto err_revert_update_pipe;
	}

	return 0;

err_revert_update_pipe:
	max_des_update_pipe(priv, context, source, pipe,
			    routing, pad, priv->streams_masks[pad]);

err_revert_pipe_disable:
	if (!enable && enable_changed)
		max_des_set_pipe_enable(des, pipe, !enable);

	return ret;
}

static int max_des_update_phy(struct max_des_priv *priv,
			      const struct v4l2_subdev_krouting *routing,
			      u32 pad, u64 *streams_masks)
{
	bool enable_changed = !streams_masks[pad] != !priv->streams_masks[pad];
	bool enable = !!streams_masks[pad];
	struct max_des *des = priv->des;
	struct max_des_phy *phy;
	int ret;

	phy = max_des_pad_to_phy(des, pad);
	if (!phy)
		return -EINVAL;

	if (enable_changed) {
		ret = max_des_set_phy_active(des, phy, enable);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_update_active(struct max_des_priv *priv, u64 *streams_masks,
				 bool expected_active)
{
	struct max_des *des = priv->des;
	bool active = false;
	unsigned int i;
	int ret;

	for (i = 0; i < des->ops->num_phys; i++) {
		struct max_des_phy *phy = &des->phys[i];
		u32 pad = max_des_phy_to_pad(des, phy);

		if (streams_masks[pad]) {
			active = true;
			break;
		}
	}

	if (active != expected_active || des->active == active)
		return 0;

	ret = des->ops->set_enable(des, active);
	if (ret)
		return ret;

	des->active = active;

	return 0;
}

static int max_des_update_links(struct max_des_priv *priv,
				struct max_des_remap_context *context,
				const struct v4l2_subdev_krouting *routing,
				u64 *streams_masks)
{
	struct max_des *des = priv->des;
	unsigned int failed_update_link_id = des->ops->num_links;
	unsigned int i;
	int ret;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		u32 sink_pad = max_des_link_to_pad(des, link);

		ret = max_des_update_link(priv, context, link, routing,
					  streams_masks[sink_pad]);
		if (ret) {
			failed_update_link_id = i;
			goto err;
		}
	}

	return 0;

err:
	for (i = 0; i < failed_update_link_id; i++) {
		struct max_des_link *link = &des->links[i];
		u32 sink_pad = max_des_link_to_pad(des, link);

		max_des_update_link(priv, context, link, routing,
				    priv->streams_masks[sink_pad]);
	}

	return ret;
}

static int max_des_enable_disable_streams(struct max_des_priv *priv,
					  struct v4l2_subdev_state *state,
					  u32 pad, u64 updated_streams_mask,
					  bool enable)
{
	struct max_des *des = priv->des;

	return max_xlate_enable_disable_streams(priv->sources, 0, state,
						pad, updated_streams_mask, 0,
						des->ops->num_links, enable);
}

static int max_des_update_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 updated_streams_mask, bool enable)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des_remap_context context = { 0 };
	struct max_des_mode_context mode_context = { 0 };
	struct max_des *des = priv->des;
	unsigned int num_pads = max_des_num_pads(des);
	u64 *streams_masks;
	int ret;

	ret = max_des_populate_remap_context(priv, &context, &state->routing);
	if (ret)
		return ret;

	ret = max_des_populate_mode_context(priv, &mode_context, &state->routing);
	if (ret)
		return ret;

	ret = max_get_streams_masks(priv->dev, state, pad, updated_streams_mask,
				    num_pads, 0, des->ops->num_links,
				    priv->streams_masks, &streams_masks, enable);
	if (ret)
		return ret;

	ret = max_des_set_pipes_phy(priv, &context);
	if (ret)
		goto err_free_streams_masks;

	ret = max_des_set_tunnel(priv, &context);
	if (ret)
		goto err_free_streams_masks;

	ret = max_des_set_modes(priv, &context, &mode_context);
	if (ret)
		goto err_free_streams_masks;

	ret = max_des_set_pipes_stream_id(priv);
	if (ret)
		goto err_free_streams_masks;

	if (!enable) {
		ret = max_des_enable_disable_streams(priv, state, pad,
						     updated_streams_mask, enable);
		if (ret)
			goto err_free_streams_masks;
	}

	ret = max_des_update_active(priv, streams_masks, false);
	if (ret)
		goto err_revert_streams_disable;

	ret = max_des_update_links(priv, &context, &state->routing, streams_masks);
	if (ret)
		goto err_revert_active_disable;

	ret = max_des_update_phy(priv, &state->routing, pad, streams_masks);
	if (ret)
		goto err_revert_links_update;

	ret = max_des_update_active(priv, streams_masks, true);
	if (ret)
		goto err_revert_phy_update;

	if (enable) {
		ret = max_des_enable_disable_streams(priv, state, pad,
						     updated_streams_mask, enable);
		if (ret)
			goto err_revert_active_enable;
	}

	devm_kfree(priv->dev, priv->streams_masks);
	priv->streams_masks = streams_masks;

	return 0;

err_revert_active_enable:
	max_des_update_active(priv, priv->streams_masks, false);

err_revert_phy_update:
	max_des_update_phy(priv, &state->routing, pad, priv->streams_masks);

err_revert_links_update:
	max_des_update_links(priv, &context, &state->routing, priv->streams_masks);

err_revert_active_disable:
	max_des_update_active(priv, priv->streams_masks, true);

err_revert_streams_disable:
	if (!enable)
		max_des_enable_disable_streams(priv, state, pad,
					       updated_streams_mask, !enable);

err_free_streams_masks:
	devm_kfree(priv->dev, streams_masks);

	return ret;
}

static int max_des_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	return max_des_update_streams(sd, state, pad, streams_mask, true);
}

static int max_des_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	return max_des_update_streams(sd, state, pad, streams_mask, false);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int max_des_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;
	unsigned int val;
	int ret;

	ret = des->ops->reg_read(des, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int max_des_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct max_des_priv *priv = v4l2_get_subdevdata(sd);
	struct max_des *des = priv->des;

	return des->ops->reg_write(des, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops max_des_core_ops = {
	.log_status = max_des_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = max_des_g_register,
	.s_register = max_des_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops max_des_pad_ops = {
	.enable_streams = max_des_enable_streams,
	.disable_streams = max_des_disable_streams,

	.set_routing = max_des_set_routing,
	.get_frame_desc = max_des_get_frame_desc,

	.get_mbus_config = max_des_get_mbus_config,

	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = max_des_set_fmt,
};

static const struct v4l2_subdev_ops max_des_subdev_ops = {
	.core = &max_des_core_ops,
	.pad = &max_des_pad_ops,
};

static const struct media_entity_operations max_des_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int max_des_notify_bound(struct v4l2_async_notifier *nf,
				struct v4l2_subdev *subdev,
				struct v4l2_async_connection *base_asc)
{
	struct max_des_priv *priv = nf_to_priv(nf);
	struct max_asc *asc = asc_to_max(base_asc);
	struct max_source *source = asc->source;
	struct max_des *des = priv->des;
	struct max_des_link *link = &des->links[source->index];
	u32 pad = max_des_link_to_pad(des, link);
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

static void max_des_notify_unbind(struct v4l2_async_notifier *nf,
				  struct v4l2_subdev *subdev,
				  struct v4l2_async_connection *base_asc)
{
	struct max_asc *asc = asc_to_max(base_asc);
	struct max_source *source = asc->source;

	source->sd = NULL;
}

static const struct v4l2_async_notifier_operations max_des_notify_ops = {
	.bound = max_des_notify_bound,
	.unbind = max_des_notify_unbind,
};

static int max_des_v4l2_notifier_register(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	v4l2_async_subdev_nf_init(&priv->nf, &priv->sd);

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct max_source *source;
		struct max_asc *asc;

		if (!link->enabled)
			continue;

		source = max_des_find_link_source(priv, link);
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

	priv->nf.ops = &max_des_notify_ops;

	ret = v4l2_async_nf_register(&priv->nf);
	if (ret) {
		dev_err(priv->dev, "Failed to register subdev notifier");
		v4l2_async_nf_cleanup(&priv->nf);
		return ret;
	}

	return 0;
}

static void max_des_v4l2_notifier_unregister(struct max_des_priv *priv)
{
	v4l2_async_nf_unregister(&priv->nf);
	v4l2_async_nf_cleanup(&priv->nf);
}

static int max_des_v4l2_register(struct max_des_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct max_des *des = priv->des;
	void *data = i2c_get_clientdata(priv->client);
	unsigned int num_pads = max_des_num_pads(des);
	unsigned int i;
	int ret;

	v4l2_i2c_subdev_init(sd, priv->client, &max_des_subdev_ops);
	i2c_set_clientdata(priv->client, data);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max_des_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;

	for (i = 0; i < num_pads; i++) {
		if (max_des_pad_is_sink(des, i))
			priv->pads[i].flags = MEDIA_PAD_FL_SINK;
		else
			priv->pads[i].flags = MEDIA_PAD_FL_SOURCE;
	}

	v4l2_set_subdevdata(sd, priv);

	ret = media_entity_pads_init(&sd->entity, num_pads, priv->pads);
	if (ret)
		return ret;

	ret = max_des_v4l2_notifier_register(priv);
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
	max_des_v4l2_notifier_unregister(priv);
err_media_entity_cleanup:
	media_entity_cleanup(&sd->entity);

	return ret;
}

static void max_des_v4l2_unregister(struct max_des_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	max_des_v4l2_notifier_unregister(priv);
	media_entity_cleanup(&sd->entity);
}

static int max_des_update_pocs(struct max_des_priv *priv, bool enable)
{
	struct max_des *des = priv->des;
	unsigned int i;
	int ret;

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];

		if (!link->enabled)
			continue;

		if (!priv->pocs[i])
			continue;

		if (enable)
			ret = regulator_enable(priv->pocs[i]);
		else
			ret = regulator_disable(priv->pocs[i]);

		if (ret) {
			dev_err(priv->dev,
				"Failed to set POC supply to %u: %u\n",
				enable, ret);
			return ret;
		}
	}

	return 0;
}

static int max_des_parse_sink_dt_endpoint(struct max_des_priv *priv,
					  struct max_des_link *link,
					  struct max_source *source,
					  struct fwnode_handle *fwnode)
{
	struct max_des *des = priv->des;
	u32 pad = max_des_link_to_pad(des, link);
	unsigned int index = link->index;
	struct fwnode_handle *ep;
	char poc_name[10];
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, pad, 0, 0);
	if (!ep)
		return 0;

	source->ep_fwnode = fwnode_graph_get_remote_endpoint(ep);
	fwnode_handle_put(ep);
	if (!source->ep_fwnode) {
		dev_err(priv->dev,
			"Failed to get remote endpoint on port %u\n", pad);
		return -ENODEV;
	}

	snprintf(poc_name, sizeof(poc_name), "port%u-poc", index);
	priv->pocs[index] = devm_regulator_get_optional(priv->dev, poc_name);
	if (IS_ERR(priv->pocs[index])) {
		ret = PTR_ERR(priv->pocs[index]);
		if (ret != -ENODEV) {
			dev_err(priv->dev,
				"Failed to get POC supply on port %u: %d\n",
				index, ret);
			goto err_put_source_ep_fwnode;
		}

		priv->pocs[index] = NULL;
	}

	link->enabled = true;

	return 0;

err_put_source_ep_fwnode:
	fwnode_handle_put(source->ep_fwnode);

	return ret;
}

static int max_des_parse_src_dt_endpoint(struct max_des_priv *priv,
					 struct max_des_phy *phy,
					 struct fwnode_handle *fwnode)
{
	struct max_des *des = priv->des;
	u32 pad = max_des_phy_to_pad(des, phy);
	struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = V4L2_MBUS_UNKNOWN };
	struct v4l2_mbus_config_mipi_csi2 *mipi = &v4l2_ep.bus.mipi_csi2;
	enum v4l2_mbus_type bus_type;
	struct fwnode_handle *ep;
	u64 link_frequency;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(fwnode, pad, 0, 0);
	if (!ep)
		return 0;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &v4l2_ep);
	fwnode_handle_put(ep);
	if (ret) {
		dev_err(priv->dev, "Could not parse endpoint on port %u\n", pad);
		return ret;
	}

	bus_type = v4l2_ep.bus_type;
	if (bus_type != V4L2_MBUS_CSI2_DPHY &&
	    bus_type != V4L2_MBUS_CSI2_CPHY) {
		v4l2_fwnode_endpoint_free(&v4l2_ep);
		dev_err(priv->dev, "Unsupported bus-type %u on port %u\n",
			pad, bus_type);
		return -EINVAL;
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
		dev_err(priv->dev, "Invalid link frequencies %u on port %u\n",
			v4l2_ep.nr_of_link_frequencies, pad);
		return -EINVAL;
	}

	if (link_frequency < MAX_DES_LINK_FREQUENCY_MIN ||
	    link_frequency > MAX_DES_LINK_FREQUENCY_MAX) {
		dev_err(priv->dev, "Invalid link frequency %llu on port %u\n",
			link_frequency, pad);
		return -EINVAL;
	}

	for (i = 0; i < mipi->num_data_lanes; i++) {
		if (mipi->data_lanes[i] > mipi->num_data_lanes) {
			dev_err(priv->dev, "Invalid data lane %u on port %u\n",
				mipi->data_lanes[i], pad);
			return -EINVAL;
		}
	}

	phy->bus_type = bus_type;
	phy->mipi = *mipi;
	phy->link_frequency = link_frequency;
	phy->enabled = true;

	return 0;
}

int max_des_phy_hw_data_lanes(struct max_des *des, struct max_des_phy *phy)
{
	const struct max_phys_configs *configs = &des->ops->phys_configs;
	const struct max_phys_config *config = &configs->configs[des->phys_config];

	return config->lanes[phy->index];
}
EXPORT_SYMBOL(max_des_phy_hw_data_lanes);

static int max_des_find_phys_config(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	const struct max_phys_configs *configs = &des->ops->phys_configs;
	struct max_des_phy *phy;
	unsigned int i, j;

	if (!configs->num_configs)
		return 0;

	for (i = 0; i < configs->num_configs; i++) {
		const struct max_phys_config *config = &configs->configs[i];
		bool matching = true;

		for (j = 0; j < des->ops->num_phys; j++) {
			phy = &des->phys[j];

			if (!phy->enabled)
				continue;

			if (phy->mipi.num_data_lanes <= config->lanes[j] &&
			    phy->mipi.clock_lane == config->clock_lane[j])
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

	des->phys_config = i;

	return 0;
}

static int max_des_parse_dt(struct max_des_priv *priv)
{
	struct fwnode_handle *fwnode = dev_fwnode(priv->dev);
	struct max_des *des = priv->des;
	struct max_des_link *link;
	struct max_des_pipe *pipe;
	struct max_des_phy *phy;
	unsigned int i;
	int ret;

	for (i = 0; i < des->ops->num_phys; i++) {
		phy = &des->phys[i];
		phy->index = i;

		ret = max_des_parse_src_dt_endpoint(priv, phy, fwnode);
		if (ret)
			return ret;
	}

	ret = max_des_find_phys_config(priv);
	if (ret)
		return ret;

	/* Find an unsed PHY to send unampped data to. */
	for (i = 0; i < des->ops->num_phys; i++) {
		phy = &des->phys[i];

		if (!phy->enabled) {
			priv->unused_phy = phy;
			break;
		}
	}

	for (i = 0; i < des->ops->num_pipes; i++) {
		pipe = &des->pipes[i];
		pipe->index = i;

		/*
		 * Serializers can send data on different stream ids over the
		 * same link, and some deserializers support stream id autoselect
		 * allowing them to receive data from all stream ids.
		 * Deserializers that support that feature should enable it.
		 * Deserializers that support per-link stream ids do not need
		 * to assign unique stream ids to each serializer.
		 */
		if (des->ops->needs_unique_stream_id)
			pipe->stream_id = i;
		else
			pipe->stream_id = 0;

		/*
		 * We already checked that num_pipes >= num_links.
		 * Set up pipe to receive data from the link with the same index.
		 * This is already the default for most chips, and some of them
		 * don't even support receiving pipe data from a different link.
		 */
		pipe->link_id = i;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		link = &des->links[i];
		link->index = i;
	}

	for (i = 0; i < des->ops->num_links; i++) {
		struct max_des_link *link = &des->links[i];
		struct max_source *source;

		source = max_des_find_link_source(priv, link);
		if (!source)
			return -ENOENT;

		source->index = i;

		ret = max_des_parse_sink_dt_endpoint(priv, link, source, fwnode);
		if (ret)
			return ret;
	}

	return 0;
}

static int max_des_allocate(struct max_des_priv *priv)
{
	struct max_des *des = priv->des;
	unsigned int num_pads = max_des_num_pads(des);

	des->phys = devm_kcalloc(priv->dev, des->ops->num_phys,
				 sizeof(*des->phys), GFP_KERNEL);
	if (!des->phys)
		return -ENOMEM;

	des->pipes = devm_kcalloc(priv->dev, des->ops->num_pipes,
				  sizeof(*des->pipes), GFP_KERNEL);
	if (!des->pipes)
		return -ENOMEM;

	des->links = devm_kcalloc(priv->dev, des->ops->num_links,
				  sizeof(*des->links), GFP_KERNEL);
	if (!des->links)
		return -ENOMEM;

	priv->sources = devm_kcalloc(priv->dev, des->ops->num_links,
				     sizeof(*priv->sources), GFP_KERNEL);
	if (!priv->sources)
		return -ENOMEM;

	priv->pocs = devm_kcalloc(priv->dev, des->ops->num_links,
				  sizeof(*priv->pocs), GFP_KERNEL);
	if (!priv->pocs)
		return -ENOMEM;

	priv->pads = devm_kcalloc(priv->dev, num_pads,
				  sizeof(*priv->pads), GFP_KERNEL);
	if (!priv->pads)
		return -ENOMEM;

	priv->streams_masks = devm_kcalloc(priv->dev, num_pads,
					   sizeof(*priv->streams_masks),
					   GFP_KERNEL);
	if (!priv->streams_masks)
		return -ENOMEM;

	return 0;
}

int max_des_probe(struct i2c_client *client, struct max_des *des)
{
	struct device *dev = &client->dev;
	struct max_des_priv *priv;
	int ret;

	if (des->ops->num_phys > MAX_DES_PHYS_NUM)
		return -E2BIG;

	if (des->ops->num_pipes > MAX_DES_PIPES_NUM)
		return -E2BIG;

	if (des->ops->num_links > des->ops->num_pipes)
		return -E2BIG;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (des->ops->set_link_version && !des->ops->select_links) {
		dev_err(dev,
			"Cannot implement .select_link_version() without .select_links()\n");
		return -EINVAL;
	}

	if (hweight_long(des->ops->versions) >= 1 &&
	    !des->ops->set_link_version) {
		dev_err(dev, "Multiple version without .select_link_version()\n");
		return -EINVAL;
	}

	priv->client = client;
	priv->dev = dev;
	priv->des = des;
	des->priv = priv;

	ret = max_des_allocate(priv);
	if (ret)
		return ret;

	ret = max_des_parse_dt(priv);
	if (ret)
		return ret;

	ret = max_des_init(priv);
	if (ret)
		return ret;

	ret = max_des_update_pocs(priv, true);
	if (ret)
		return ret;

	ret = max_des_i2c_adapter_init(priv);
	if (ret)
		goto err_disable_pocs;

	ret = max_des_v4l2_register(priv);
	if (ret)
		goto err_i2c_adapter_deinit;

	return 0;

err_i2c_adapter_deinit:
	max_des_i2c_adapter_deinit(priv);

err_disable_pocs:
	max_des_update_pocs(priv, false);

	return ret;
}
EXPORT_SYMBOL_GPL(max_des_probe);

int max_des_remove(struct max_des *des)
{
	struct max_des_priv *priv = des->priv;

	max_des_v4l2_unregister(priv);

	max_des_i2c_adapter_deinit(priv);

	max_des_update_pocs(priv, false);

	return 0;
}
EXPORT_SYMBOL_GPL(max_des_remove);

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("I2C_ATR");
