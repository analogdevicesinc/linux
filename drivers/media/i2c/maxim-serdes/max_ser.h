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

struct max_ser;

struct max_ser_ops {
	unsigned int num_pipes;
	unsigned int num_dts_per_pipe;
	unsigned int num_phys;
	unsigned int num_i2c_xlates;
	bool supports_tunnel_mode;
	bool supports_noncontinuous_clock;

	int (*log_status)(struct max_ser *ser, const char *name);
	int (*log_pipe_status)(struct max_ser *ser, struct max_ser_pipe *pipe,
			       const char *name);
	int (*log_phy_status)(struct max_ser *ser, struct max_ser_phy *phy,
			      const char *name);
	int (*set_pipe_enable)(struct max_ser *ser, struct max_ser_pipe *pipe, bool enable);
	int (*update_pipe_dts)(struct max_ser *ser, struct max_ser_pipe *pipe);
	int (*update_pipe_vcs)(struct max_ser *ser, struct max_ser_pipe *pipe);
	int (*init)(struct max_ser *ser);
	int (*init_i2c_xlate)(struct max_ser *ser);
	int (*init_phy)(struct max_ser *ser, struct max_ser_phy *phy);
	int (*init_pipe)(struct max_ser *ser, struct max_ser_pipe *pipe);
	int (*post_init)(struct max_ser *ser);
};

struct max_ser_priv;

struct max_ser {
	struct max_ser_priv *priv;

	const struct max_ser_ops *ops;

	struct max_i2c_xlate *i2c_xlates;
	unsigned int num_i2c_xlates;

	struct max_ser_phy *phys;
	struct max_ser_pipe *pipes;

	bool tunnel_mode;
};

int max_ser_probe(struct i2c_client *client, struct max_ser *ser);

int max_ser_remove(struct max_ser *ser);

#endif // MAX_SER_H
