/*
 * AD9371
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_TRX_AD9371_H_
#define IIO_TRX_AD9371_H_

#include "mykonos/t_mykonos.h"

#define MIN_GAIN_mdB		0
#define MAX_RX_GAIN_mdB		30000
#define MAX_OBS_RX_GAIN_mdB	18000
#define MAX_OBS_SNRX_GAIN_mdB	52000

#define RX_GAIN_STEP_mdB	500
#define OBS_RX_GAIN_STEP_mdB	1000
#define SNRX_GAIN_STEP_mdB	1000

#define PRODUCT_ID_9371		0x08
#define REV_MASK		0x07

struct ad9371_phy_platform_data {

	u64			rx_synth_freq;
	u64			tx_synth_freq;
	struct gpio_desc			*reset_gpio;

};

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_INIT,
	DBGFS_LOOPBACK,
	DBGFS_BIST_PRBS,
	DBGFS_BIST_TONE,
};


enum ad9371_bist_mode {
	BIST_DISABLE,
	BIST_INJ_TX,
	BIST_INJ_RX,
};

enum ad9371_rx_ext_info {
	RSSI,
	RX_QEC,
	TEMPCOMP_GAIN,
	RX_RF_BANDWIDTH,
};

enum ad9371_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_RF_BANDWIDTH,
};

enum ad9371_iio_voltage_in {
	CHAN_RX1,
	CHAN_RX2,
	CHAN_OBS,
	CHAN_AUXADC0,
	CHAN_AUXADC1,
	CHAN_AUXADC2,
	CHAN_AUXADC3,
};

enum ad9371_iio_voltage_out {
	CHAN_TX1,
	CHAN_TX2,
	CHAN_AUXDAC0,
	CHAN_AUXDAC1,
	CHAN_AUXDAC2,
	CHAN_AUXDAC3,
	CHAN_AUXDAC4,
	CHAN_AUXDAC5,
	CHAN_AUXDAC6,
	CHAN_AUXDAC7,
	CHAN_AUXDAC8,
	CHAN_AUXDAC9,
};

enum {
	ID_AD9371,
};

enum ad9371_sysref_req_mode {
	SYSREF_CONT_ON,
	SYSREF_CONT_OFF,
	SYSREF_PULSE,
};

struct ad9371_rf_phy;
struct ad9371_debugfs_entry {
	struct ad9371_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

enum ad9371_clocks {
	RX_SAMPL_CLK,
	OBS_SAMPL_CLK,
	TX_SAMPL_CLK,
	NUM_AD9371_CLKS,
};

enum ad9371_radio_states {
	RADIO_OFF,
	RADIO_ON,
	RADIO_FORCE_OFF,
	RADIO_RESTORE_STATE,
};

struct ad9371_clock {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct ad9371_rf_phy	*phy;
	unsigned long		rate;
	enum ad9371_clocks 	source;
};

#define to_clk_priv(_hw) container_of(_hw, struct ad9371_clock, hw)

#define MAX_NUM_GAIN_TABLES 10

struct gain_table_info {
	u64 start;
	u64 end;
	u8 max_index;
	u8 dest;
	s32 *abs_gain_tbl;
	u8 (*tab)[4];
};

struct ad9371_rf_phy {
	struct spi_device 	*spi;
	const struct firmware 	*fw;
	mykonosDevice_t 	*mykDevice;
	struct clk 		*dev_clk;
	struct clk 		*jesd_rx_clk;
	struct clk 		*jesd_tx_clk;
	struct clk 		*jesd_rx_os_clk;
	struct clk 		*clk_ext_lo_rx;
	struct clk 		*clk_ext_lo_tx;
	struct clk 		*clks[NUM_AD9371_CLKS];
	struct ad9371_clock	clk_priv[NUM_AD9371_CLKS];
	struct clk_onecell_data	clk_data;
	struct ad9371_phy_platform_data *pdata;
	struct ad9371_debugfs_entry debugfs_entry[300];
	struct bin_attribute 	bin;
	struct bin_attribute 	bin_gt;
	struct iio_dev 		*indio_dev;

	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*sysref_req_gpio;
	struct gain_table_info  gt_info[LOOPBACK_GT + 1];

	char			*bin_gt_attr_buf;
	char			*bin_attr_buf;
	u32 			ad9371_debugfs_entry_index;
	u8			obs_rx_path_source;
	u32			agc_mode[2];
	u32			tracking_cal_mask;
	bool			radio_state;
	bool			saved_radio_state;
	u32			init_cal_mask;
	u32			rf_bandwith[3];
};

int ad9371_hdl_loopback(struct ad9371_rf_phy *phy, bool enable);
int ad9371_register_axi_converter(struct ad9371_rf_phy *phy);
struct ad9371_rf_phy* ad9371_spi_to_phy(struct spi_device *spi);
int ad9371_spi_read(struct spi_device *spi, u32 reg);
int ad9371_spi_write(struct spi_device *spi, u32 reg, u32 val);

#endif

