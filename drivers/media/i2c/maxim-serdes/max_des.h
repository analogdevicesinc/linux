// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/i2c-atr.h>

#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max_serdes.h"

#ifndef MAX_DES_H
#define MAX_DES_H

/* TODO: remove */
#define MAX_DES_REMAPS_NUM		16

#define MAX_DES_SOURCE_PAD		0
#define MAX_DES_SINK_PAD		1
#define MAX_DES_PAD_NUM			2

extern const struct regmap_config max_des_i2c_regmap;

struct max_des_asd {
	struct v4l2_async_subdev base;
	struct max_des_subdev_priv *sd_priv;
};

#define MAX_DES_DT_VC(dt, vc) (((vc) & 0x3) << 6 | ((dt) & 0x3f))

struct max_des_dt_vc_remap {
	u8 from_dt;
	u8 from_vc;
	u8 to_dt;
	u8 to_vc;
	u8 phy;
};

struct max_des_subdev_priv {
	struct v4l2_subdev sd;
	unsigned int index;
	struct fwnode_handle *fwnode;

	struct max_des_priv *priv;

	struct v4l2_subdev *slave_sd;
	struct fwnode_handle *slave_fwnode;
	struct v4l2_subdev_state *slave_sd_state;
	unsigned int slave_sd_pad_id;

	struct v4l2_async_notifier notifier;
	struct media_pad pads[MAX_DES_PAD_NUM];

	bool active;
	unsigned int pipe_id;
	unsigned int phy_id;
	struct max_des_dt_vc_remap remaps[MAX_DES_REMAPS_NUM];
	unsigned int src_vc_id;
	unsigned int dst_vc_id;
	unsigned int num_remaps;
	unsigned int dt;
};

struct max_des_link {
	unsigned int index;
	bool enabled;
	struct max_i2c_xlate ser_xlate;
	bool ser_xlate_enabled;
};

struct max_des_pipe {
	unsigned int index;
	unsigned int phy_id;
	unsigned int stream_id;
	unsigned int link_id;
	struct max_des_dt_vc_remap remaps[MAX_DES_REMAPS_NUM];
	unsigned int num_remaps;
	bool enabled;
};

struct max_des_phy {
	unsigned int index;
	struct v4l2_fwnode_bus_mipi_csi2 mipi;
	bool alt_mem_map8;
	bool alt_mem_map10;
	bool alt_mem_map12;
	bool enabled;
};

struct max_des_ops {
	unsigned int num_phys;
	unsigned int num_pipes;
	unsigned int num_links;
	bool fix_tx_ids;
	bool supports_pipe_link_remap;

	int (*mipi_enable)(struct max_des_priv *priv, bool enable);
	int (*init)(struct max_des_priv *priv);
	int (*init_phy)(struct max_des_priv *priv, struct max_des_phy *phy);
	int (*init_pipe)(struct max_des_priv *priv, struct max_des_pipe *pipe);
	int (*update_pipe_remaps)(struct max_des_priv *priv, struct max_des_pipe *pipe);
	int (*select_links)(struct max_des_priv *priv, unsigned int mask);
	int (*post_init)(struct max_des_priv *priv);
};

struct max_des_priv {
	const struct max_des_ops *ops;

	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct i2c_atr *atr;

	unsigned int num_subdevs;
	struct mutex lock;
	bool active;

	struct max_des_phy *phys;
	struct max_des_pipe *pipes;
	struct max_des_link *links;
	struct max_des_subdev_priv *sd_privs;
};

int max_des_probe(struct max_des_priv *priv);

int max_des_remove(struct max_des_priv *priv);

static inline struct max_des_phy *max_des_phy_by_id(struct max_des_priv *priv,
						    unsigned int index)
{
	return &priv->phys[index];
}

#endif // MAX_DES_H
