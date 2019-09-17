/*
 * Xilinx High-speed tranceiver dynamic reconfiguration
 *
 * Copyright 2016-2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 */

#ifndef XILINX_XCVR_H
#define XILINX_XCVR_H

#include <linux/fpga/adi-axi-common.h>

enum xilinx_xcvr_type {
	XILINX_XCVR_TYPE_S7_GTX2 = 2,
	XILINX_XCVR_TYPE_US_GTH3 = 5,
	XILINX_XCVR_TYPE_US_GTH4 = 8,
	XILINX_XCVR_TYPE_US_GTY4 = 9,
};

enum xilinx_xcvr_legacy_type {
	XILINX_XCVR_LEGACY_TYPE_S7_GTX2,
	XILINX_XCVR_LEGACY_TYPE_US_GTH3,
	XILINX_XCVR_LEGACY_TYPE_US_GTH4,
	XILINX_XCVR_LEGACY_TYPE_US_GTY4 = 4,
};

enum xilinx_xcvr_refclk_ppm {
	PM_200,
	PM_700,
	PM_1250,
};

struct xilinx_xcvr;
struct device;

struct xilinx_xcvr_drp_ops {
	int (*write)(struct xilinx_xcvr *xcvr, unsigned int drp_port,
		unsigned int reg, unsigned int val);
	int (*read)(struct xilinx_xcvr *xcvr, unsigned int drp_port,
		unsigned int reg);
};

struct xilinx_xcvr {
	struct device *dev;

	const struct xilinx_xcvr_drp_ops *drp_ops;

	enum xilinx_xcvr_type type;
	enum xilinx_xcvr_refclk_ppm refclk_ppm;
	unsigned int encoding;
	unsigned int version;
	enum adi_axi_fgpa_technology tech;
	enum adi_axi_fpga_family family;
	enum adi_axi_fpga_speed_grade speed_grade;
	enum adi_axi_fpga_dev_pack dev_package;
	unsigned int voltage;
};

#define ENC_8B10B					810

struct xilinx_xcvr_cpll_config {
	unsigned int refclk_div;
	unsigned int fb_div_N1;
	unsigned int fb_div_N2;
};

struct xilinx_xcvr_qpll_config {
	unsigned int refclk_div;
	unsigned int fb_div;
	unsigned int band;
	unsigned int qpll;
};

int xilinx_xcvr_configure_cdr(struct xilinx_xcvr *xcvr, unsigned int drp_port,
	unsigned int lane_rate, unsigned int out_div, bool lpm);
int xilinx_xcvr_configure_lpm_dfe_mode(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, bool lpm);

int xilinx_xcvr_calc_cpll_config(struct xilinx_xcvr *xcvr,
	unsigned int refclk_hz, unsigned int lanerate_khz,
	struct xilinx_xcvr_cpll_config *conf,
	unsigned int *out_div);
int xilinx_xcvr_cpll_read_config(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, struct xilinx_xcvr_cpll_config *conf);
int xilinx_xcvr_cpll_write_config(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, const struct xilinx_xcvr_cpll_config *conf);
int xilinx_xcvr_cpll_calc_lane_rate(struct xilinx_xcvr *xcvr,
	unsigned int refclk_hz, const struct xilinx_xcvr_cpll_config *conf,
	unsigned int out_div);

int xilinx_xcvr_calc_qpll_config(struct xilinx_xcvr *xcvr,
	unsigned int refclk_hz, unsigned int lanerate_khz,
	struct xilinx_xcvr_qpll_config *conf,
	unsigned int *out_div);
int xilinx_xcvr_qpll_read_config(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, struct xilinx_xcvr_qpll_config *conf);
int xilinx_xcvr_qpll_write_config(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, const struct xilinx_xcvr_qpll_config *conf);
int xilinx_xcvr_qpll_calc_lane_rate(struct xilinx_xcvr *xcvr,
	unsigned int ref_clk_hz, const struct xilinx_xcvr_qpll_config *conf,
	unsigned int out_div);

int xilinx_xcvr_read_out_div(struct xilinx_xcvr *xcvr, unsigned int drp_port,
	unsigned int *rx_out_div, unsigned int *tx_out_div);
int xilinx_xcvr_write_out_div(struct xilinx_xcvr *xcvr, unsigned int drp_port,
	int rx_out_div, int tx_out_div);

int xilinx_xcvr_write_rx_clk25_div(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, unsigned int div);
int xilinx_xcvr_write_tx_clk25_div(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, unsigned int div);

int xilinx_xcvr_drp_update(struct xilinx_xcvr *xcvr,
	unsigned int drp_port, unsigned int reg, unsigned int mask,
	unsigned int val);

#endif
