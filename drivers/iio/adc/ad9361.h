/*
 * AD9361
 *
 * Copyright 2013-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9361_H_
#define IIO_FREQUENCY_AD9361_H_

#include "ad9361_regs.h"

enum ad9361_clocks {
	BB_REFCLK,
	RX_REFCLK,
	TX_REFCLK,
	BBPLL_CLK,
	ADC_CLK,
	R2_CLK,
	R1_CLK,
	CLKRF_CLK,
	RX_SAMPL_CLK,
	DAC_CLK,
	T2_CLK,
	T1_CLK,
	CLKTF_CLK,
	TX_SAMPL_CLK,
	RX_RFPLL_INT,
	TX_RFPLL_INT,
	RX_RFPLL_DUMMY,
	TX_RFPLL_DUMMY,
	RX_RFPLL,
	TX_RFPLL,
	NUM_AD9361_CLKS,
};

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_INIT,
	DBGFS_LOOPBACK,
	DBGFS_BIST_PRBS,
	DBGFS_BIST_TONE,
	DBGFS_BIST_DT_ANALYSIS,
	DBGFS_RXGAIN_1,
	DBGFS_RXGAIN_2,
	DBGFS_MCS,
	DBGFS_CAL_SW_CTRL,
	DBGFS_DIGITAL_TUNE,
};

enum dig_tune_flags {
	BE_VERBOSE = 1,
	BE_MOREVERBOSE = 2,
	DO_IDELAY = 4,
	DO_ODELAY = 8,
	SKIP_STORE_RESULT = 16,
	RESTORE_DEFAULT = 32,
};

enum ad9361_bist_mode {
	BIST_DISABLE,
	BIST_INJ_TX,
	BIST_INJ_RX,
};

enum {
	ID_AD9361,
	ID_AD9364,
	ID_AD9361_2,
	ID_AD9363A,
};

enum digital_tune_skip_mode {
	TUNE_RX_TX,
	SKIP_TX,
	SKIP_ALL,
};

struct ad9361_rf_phy;
struct ad9361_debugfs_entry {
	struct ad9361_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

struct ad9361_dig_tune_data {
	u32 bist_loopback_mode;
	u32 bist_config;
	u32 ensm_state;
	u8 skip_mode;
};

struct refclk_scale {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct ad9361_rf_phy	*phy;
	unsigned long		rate;
	u32			mult;
	u32			div;
	enum ad9361_clocks 	source;
};

struct ad9361_rf_phy_state;
struct ad9361_rf_phy {
	struct spi_device 	*spi;
	struct clk 		*clk_refin;
	struct clk 		*clk_ext_lo_rx;
	struct clk 		*clk_ext_lo_tx;
	struct clk 		*clks[NUM_AD9361_CLKS];
	struct notifier_block   clk_nb_tx;
	struct notifier_block   clk_nb_rx;
	struct refclk_scale	clk_priv[NUM_AD9361_CLKS];
	struct clk_onecell_data	clk_data;
	struct ad9361_phy_platform_data *pdata;
	struct ad9361_debugfs_entry debugfs_entry[180];
	struct bin_attribute 	bin;
	struct bin_attribute 	bin_gt;
	struct iio_dev 		*indio_dev;
	struct work_struct 	work;
	struct completion       complete;
	struct gain_table_info  *gt_info;
	char			*bin_attr_buf;
	u32 			ad9361_debugfs_entry_index;

	struct ad9361_rf_phy_state	*state;
};


ssize_t ad9361_dig_interface_timing_analysis(struct ad9361_rf_phy *phy,
						   char *buf, unsigned buflen);
int ad9361_hdl_loopback(struct ad9361_rf_phy *phy, bool enable);
int ad9361_register_axi_converter(struct ad9361_rf_phy *phy);
struct ad9361_rf_phy* ad9361_spi_to_phy(struct spi_device *spi);
int ad9361_spi_read(struct spi_device *spi, u32 reg);
int ad9361_spi_write(struct spi_device *spi, u32 reg, u32 val);
int ad9361_bist_loopback(struct ad9361_rf_phy *phy, unsigned mode);
int ad9361_bist_prbs(struct ad9361_rf_phy *phy, enum ad9361_bist_mode mode);
int ad9361_find_opt(u8 *field, u32 size, u32 *ret_start);
int ad9361_ensm_mode_disable_pinctrl(struct ad9361_rf_phy *phy);
int ad9361_ensm_mode_restore_pinctrl(struct ad9361_rf_phy *phy);
void ad9361_ensm_force_state(struct ad9361_rf_phy *phy, u8 ensm_state);
void ad9361_ensm_restore_state(struct ad9361_rf_phy *phy, u8 ensm_state);
void ad9361_ensm_restore_prev_state(struct ad9361_rf_phy *phy);
int ad9361_set_trx_clock_chain_freq(struct ad9361_rf_phy *phy,
					  unsigned long freq);
int ad9361_set_trx_clock_chain_default(struct ad9361_rf_phy *phy);
int ad9361_dig_tune(struct ad9361_rf_phy *phy, unsigned long max_freq,
			   enum dig_tune_flags flags);
int ad9361_tx_mute(struct ad9361_rf_phy *phy, u32 state);
int ad9361_write_bist_reg(struct ad9361_rf_phy *phy, u32 val);
bool ad9361_uses_rx2tx2(struct ad9361_rf_phy *phy);
int ad9361_get_dig_tune_data(struct ad9361_rf_phy *phy,
			     struct ad9361_dig_tune_data *data);
int ad9361_read_clock_data_delays(struct ad9361_rf_phy *phy);
int ad9361_write_clock_data_delays(struct ad9361_rf_phy *phy);
bool ad9361_uses_lvds_mode(struct ad9361_rf_phy *phy);

#endif

