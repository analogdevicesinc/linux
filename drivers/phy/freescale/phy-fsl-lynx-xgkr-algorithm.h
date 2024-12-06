/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 NXP
 */
#ifndef _FSL_LYNX_XGKR_ALGORITHM_H
#define _FSL_LYNX_XGKR_ALGORITHM_H

#include <linux/phy/phy.h>
#include <linux/types.h>

struct lynx_xgkr_algorithm;
struct phy;

struct lynx_xgkr_tx_eq {
	u32 ratio_preq;
	u32 ratio_post1q;
	u32 adapt_eq;
	u32 amp_reduction;
};

enum lynx_bin_type {
	BIN_1,
	BIN_2,
	BIN_3,
	BIN_4,
	BIN_OFFSET,
	BIN_BLW,
	BIN_DATA_AVG,
	BIN_M1,
	BIN_LONG,
};

struct lynx_xgkr_algorithm_ops {
	void (*read_tx_eq)(struct phy *phy, struct lynx_xgkr_tx_eq *tx_eq);
	void (*tune_tx_eq)(struct phy *phy, const struct lynx_xgkr_tx_eq *tx_eq);
	int (*snapshot_rx_eq_gains)(struct phy *phy, u8 *gaink2, u8 *gaink3,
				    u8 *eq_offset);
	int (*snapshot_rx_eq_bin)(struct phy *phy, enum lynx_bin_type bin_type,
				  s16 *snapshot);
};

struct lynx_xgkr_algorithm *
lynx_xgkr_algorithm_create(struct phy *phy,
			   const struct lynx_xgkr_algorithm_ops *ops);
void lynx_xgkr_algorithm_destroy(struct lynx_xgkr_algorithm *algorithm);
int lynx_xgkr_algorithm_configure(struct lynx_xgkr_algorithm *algorithm,
				  struct phy_configure_opts_ethernet *opts);

#endif
