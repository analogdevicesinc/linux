// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 */

#ifndef __FSL_IMX_LDB__
#define __FSL_IMX_LDB__

#include <drm/drm_bridge.h>

#define LDB_CH_NUM	2

struct ldb_channel {
	struct ldb *ldb;

	struct drm_bridge bridge;

	struct drm_panel *panel;
	struct drm_bridge *next_bridge;

	struct device_node *child;
	int chno;
	u32 bus_format;
	bool is_valid;
};

struct ldb {
	struct regmap *regmap;
	struct device *dev;
	struct ldb_channel *channel[LDB_CH_NUM];
	unsigned int ctrl_reg;
	u32 ldb_ctrl;
	int output_port;
	bool dual;
};

int ldb_bind(struct ldb *ldb, struct drm_encoder **encoder);

#endif /* __FSL_IMX_LDB__ */
