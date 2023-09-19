// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/i2c-atr.h>
#include <linux/regmap.h>

#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max_serdes.h"

#ifndef MAX_SER_H
#define MAX_SER_H

#define MAX_SER_SOURCE_PAD	0
#define MAX_SER_SINK_PAD	1
#define MAX_SER_PAD_NUM		2

#define MAX_SER_MAX96717_DEV_ID			0xbf
#define MAX_SER_MAX9265A_DEV_ID			0x91

extern const struct regmap_config max_ser_i2c_regmap;

struct max_ser_asd {
	struct v4l2_async_subdev base;
	struct max_ser_subdev_priv *sd_priv;
};

struct max_ser_subdev_priv {
	struct v4l2_subdev sd;
	unsigned int index;
	struct fwnode_handle *fwnode;

	struct max_ser_priv *priv;
	const struct max_format *fmt;
	const char *label;

	struct media_pad pads[MAX_SER_PAD_NUM];

	bool active;
	unsigned int pipe_id;
	unsigned int vc_id;
};

struct max_ser_phy {
	unsigned int index;
	struct v4l2_mbus_config_mipi_csi2 mipi;
	bool bus_config_parsed;
	bool enabled;
};

struct max_ser_pipe {
	unsigned int index;
	unsigned int phy_id;
	unsigned int stream_id;
	unsigned int *dts;
	unsigned int num_dts;
	unsigned int vcs;
	unsigned int soft_bpp;
	unsigned int bpp;
	bool dbl8;
	bool dbl10;
	bool dbl12;
	bool enabled;
	bool active;
};

struct max_ser_ops {
	unsigned int num_pipes;
	unsigned int num_dts_per_pipe;
	unsigned int num_phys;
	unsigned int num_i2c_xlates;
	bool supports_tunnel_mode;
	bool supports_noncontinuous_clock;

	int (*log_status)(struct max_ser_priv *priv, const char *name);
	int (*log_pipe_status)(struct max_ser_priv *priv, struct max_ser_pipe *pipe,
			       const char *name);
	int (*log_phy_status)(struct max_ser_priv *priv, struct max_ser_phy *phy,
			      const char *name);
	int (*set_pipe_enable)(struct max_ser_priv *priv, struct max_ser_pipe *pipe, bool enable);
	int (*update_pipe_dts)(struct max_ser_priv *priv, struct max_ser_pipe *pipe);
	int (*update_pipe_vcs)(struct max_ser_priv *priv, struct max_ser_pipe *pipe);
	int (*init)(struct max_ser_priv *priv);
	int (*init_i2c_xlate)(struct max_ser_priv *priv);
	int (*init_phy)(struct max_ser_priv *priv, struct max_ser_phy *phy);
	int (*init_pipe)(struct max_ser_priv *priv, struct max_ser_pipe *pipe);
	int (*post_init)(struct max_ser_priv *priv);
};

struct max_ser_priv {
	const struct max_ser_ops *ops;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct i2c_atr *atr;

	unsigned int num_subdevs;
	struct mutex lock;
	bool tunnel_mode;

	struct max_i2c_xlate *i2c_xlates;
	unsigned int num_i2c_xlates;

	struct max_ser_phy *phys;
	struct max_ser_pipe *pipes;
	struct max_ser_subdev_priv *sd_privs;
};

int max_ser_probe(struct max_ser_priv *priv);

int max_ser_remove(struct max_ser_priv *priv);

int max_ser_reset(struct regmap *regmap);

int max_ser_wait(struct i2c_client *client, struct regmap *regmap, u8 addr);
int max_ser_wait_for_multiple(struct i2c_client *client, struct regmap *regmap,
			      u8 *addrs, unsigned int num_addrs);

int max_ser_change_address(struct i2c_client *client, struct regmap *regmap, u8 addr,
			   bool fix_tx_ids);

static inline struct max_ser_pipe *max_ser_pipe_by_id(struct max_ser_priv *priv,
						      unsigned int index)
{
	return &priv->pipes[index];
}

static inline struct max_ser_pipe *max_ser_ch_pipe(struct max_ser_subdev_priv *sd_priv)
{
	return max_ser_pipe_by_id(sd_priv->priv, sd_priv->pipe_id);
}

static inline struct max_ser_phy *max_ser_phy_by_id(struct max_ser_priv *priv,
						    unsigned int index)
{
	return &priv->phys[index];
}

static inline struct max_ser_phy *max_ser_pipe_phy(struct max_ser_priv *priv,
						   struct max_ser_pipe *pipe)
{
	return max_ser_phy_by_id(priv, pipe->phy_id);
}

static inline struct max_ser_phy *max_ser_phy_by_pipe_id(struct max_ser_priv *priv,
							 unsigned int index)
{
	struct max_ser_pipe *pipe = max_ser_pipe_by_id(priv, index);

	return max_ser_pipe_phy(priv, pipe);
}

#endif // MAX_SER_H
