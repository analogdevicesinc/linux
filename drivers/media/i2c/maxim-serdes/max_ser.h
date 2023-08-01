/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Analog Devices Inc.
 */

#ifndef MAX_SER_H
#define MAX_SER_H

#include <linux/i2c.h>

#include <media/v4l2-mediabus.h>

#include "max_serdes.h"

#define MAX_SER_REG0				0x0
#define MAX_SER_REG0_DEV_ADDR			GENMASK(7, 1)

#define MAX_SER_CTRL0				0x10
#define MAX_SER_CTRL0_RESET_ALL			BIT(7)

#define MAX_SER_CFGI_INFOFR_TR3			0x7b
#define MAX_SER_CFGL_SPI_TR3			0x83
#define MAX_SER_CFGC_CC_TR3			0x8b
#define MAX_SER_CFGC_GPIO_TR3			0x93
#define MAX_SER_CFGL_IIC_X_TR3			0xa3
#define MAX_SER_CFGL_IIC_Y_TR3			0xab

struct max_ser_phy {
	unsigned int index;
	struct v4l2_mbus_config_mipi_csi2 mipi;
	bool enabled;
	bool active;
};

struct max_ser_pipe_mode {
	unsigned int soft_bpp;
	unsigned int bpp;
	bool dbl8;
	bool dbl10;
	bool dbl12;
};

struct max_ser_pipe {
	unsigned int index;
	unsigned int phy_id;
	unsigned int stream_id;
	unsigned int *dts;
	unsigned int num_dts;
	unsigned int vcs;
	struct max_ser_pipe_mode mode;
	bool enabled;
};

struct max_ser;

struct max_ser_ops {
	unsigned int num_pipes;
	unsigned int num_dts_per_pipe;
	unsigned int num_phys;
	unsigned int num_i2c_xlates;
	bool supports_noncontinuous_clock;

	struct max_phys_configs phys_configs;

	int (*reg_read)(struct max_ser *ser, unsigned int reg, unsigned int *val);
	int (*reg_write)(struct max_ser *ser, unsigned int reg, unsigned int val);
	int (*log_status)(struct max_ser *ser, const char *name);
	int (*log_pipe_status)(struct max_ser *ser, struct max_ser_pipe *pipe,
			       const char *name);
	int (*log_phy_status)(struct max_ser *ser, struct max_ser_phy *phy,
			      const char *name);
	int (*init)(struct max_ser *ser);
	int (*set_i2c_xlate)(struct max_ser *ser, unsigned int i,
			     struct max_i2c_xlate *i2c_xlate);
	int (*set_tunnel_enable)(struct max_ser *ser, bool enable);
	int (*init_phy)(struct max_ser *ser, struct max_ser_phy *phy);
	int (*set_phy_active)(struct max_ser *ser, struct max_ser_phy *phy,
			      bool enable);
	int (*set_pipe_enable)(struct max_ser *ser, struct max_ser_pipe *pipe,
			       bool enable);
	int (*set_pipe_dt)(struct max_ser *ser, struct max_ser_pipe *pipe,
			   unsigned int i, unsigned int dt);
	int (*set_pipe_dt_en)(struct max_ser *ser, struct max_ser_pipe *pipe,
			      unsigned int i, bool enable);
	int (*set_pipe_vcs)(struct max_ser *ser, struct max_ser_pipe *pipe,
			    unsigned int vcs);
	int (*set_pipe_mode)(struct max_ser *ser, struct max_ser_pipe *pipe,
			     struct max_ser_pipe_mode *mode);
	int (*set_pipe_stream_id)(struct max_ser *ser, struct max_ser_pipe *pipe,
				  unsigned int stream_id);
	int (*set_pipe_phy)(struct max_ser *ser, struct max_ser_pipe *pipe,
			    struct max_ser_phy *phy);
};

struct max_ser_priv;

struct max_ser {
	struct max_ser_priv *priv;

	const struct max_ser_ops *ops;

	struct max_i2c_xlate *i2c_xlates;

	struct max_ser_phy *phys;
	struct max_ser_pipe *pipes;

	unsigned int phys_config;
	unsigned int active;
	bool tunnel;
};

int max_ser_probe(struct i2c_client *client, struct max_ser *ser);

int max_ser_remove(struct max_ser *ser);

int max_ser_set_double_bpps(struct v4l2_subdev *sd, u32 double_bpps);
bool max_ser_supports_tunnel_mode(struct v4l2_subdev *sd);
int max_ser_set_tunnel_enable(struct v4l2_subdev *sd, bool enable);
int max_ser_set_stream_id(struct v4l2_subdev *sd, unsigned int stream_id);

int max_ser_reset(struct i2c_adapter *adapter, u8 addr);
int max_ser_wait(struct i2c_adapter *adapter, u8 addr);
int max_ser_wait_for_multiple(struct i2c_adapter *adapter, u8 *addrs,
			      unsigned int num_addrs, u8 *current_addr);

int max_ser_change_address(struct i2c_adapter *adapter, u8 addr, u8 new_addr);
int max_ser_fix_tx_ids(struct i2c_adapter *adapter, u8 addr);

#endif // MAX_SER_H
