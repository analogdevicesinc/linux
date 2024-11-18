/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9025
 *
 * Copyright 2020-2023 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_TRX_ADRV9025_H_
#define IIO_TRX_ADRV9025_H_

#include "adi_adrv9025_agc.h"
#include "adi_adrv9025_arm.h"
#include "adi_adrv9025_cals.h"
#include "adi_adrv9025_dfe.h"
#include "adi_adrv9025_error.h"
#include "adi_adrv9025_gpio.h"
#include "adi_adrv9025_hal.h"
#include "adi_adrv9025_radioctrl.h"
#include "adi_adrv9025_rx.h"
#include "adi_adrv9025_tx.h"
#include "adi_adrv9025_user.h"
#include "adi_adrv9025_utilities.h"
#include "adi_adrv9025_version.h"
#include "adi_adrv9025_data_interface.h"
#include "adi_platform.h"

#define MIN_GAIN_mdB 0
#define MAX_RX_GAIN_mdB 30000
#define MAX_OBS_RX_GAIN_mdB 30000
#define RX_GAIN_STEP_mdB 500

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_BIST_FRAMER_0_PRBS,
	DBGFS_BIST_FRAMER_LOOPBACK,
	DBGFS_BIST_TONE,
};

enum adrv9025_rx_ext_info {
	RSSI,
	RX_QEC,
	RX_HD2,
	RX_DIG_DC,
	RX_RF_BANDWIDTH,
};

enum adrv9025_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_RF_BANDWIDTH,
};

enum adrv9025_iio_voltage_in {
	CHAN_RX1,
	CHAN_RX2,
	CHAN_RX3,
	CHAN_RX4,
	CHAN_OBS_RX1,
	CHAN_OBS_RX2,
	CHAN_OBS_RX3,
	CHAN_OBS_RX4,}
;

enum adrv9025_iio_voltage_out {
	CHAN_TX1,
	CHAN_TX2,
	CHAN_TX3,
	CHAN_TX4,
};

enum adrv9025_device_id {
	ID_ADRV9025,
	ID_ADRV9026,
	ID_ADRV9029,
};

struct adrv9025_rf_phy;
struct adrv9025_debugfs_entry {
	struct adrv9025_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

enum adrv9025_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	NUM_ADRV9025_CLKS,
};

struct adrv9025_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct adrv9025_rf_phy *phy;
	unsigned long rate;
	enum adrv9025_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv9025_clock, hw)
#define MAX_NUM_GAIN_TABLES 10

struct adrv9025_rf_phy {
	struct spi_device *spi;
	adi_adrv9025_Device_t adi_adrv9025_device;
	adi_adrv9025_Device_t *madDevice;
	adi_adrv9025_SpiSettings_t spiSettings;
	adi_adrv9025_Init_t deviceInitStruct;
	adi_adrv9025_PlatformFiles_t platformFiles;
	adi_adrv9025_PostMcsInit_t adrv9025PostMcsInitInst;
	adi_adrv9025_InitCals_t cal_mask;

	adi_adrv9025_AgcCfg_t  *agcConfig;

	struct jesd204_dev	*jdev;
	/* protect against device accesses */
	struct mutex		lock;

	u32 tx_iqRate_kHz;
	u32 rx_iqRate_kHz;

	adi_hal_Cfg_t linux_hal;
	struct clk *dev_clk;

	struct clk *clk_ext_lo_rx;
	struct clk *clk_ext_lo_tx;
	struct clk *clks[NUM_ADRV9025_CLKS];
	struct adrv9025_clock clk_priv[NUM_ADRV9025_CLKS];
	struct clk_onecell_data clk_data;
	struct adrv9025_debugfs_entry debugfs_entry[342];
	struct iio_dev *indio_dev;

	struct gpio_desc *sysref_req_gpio;

	u8 device_id;

	u32 adrv9025_debugfs_entry_index;
	u32 tracking_cal_mask;

	bool is_initialized;
	int spi_device_id;
};

int adrv9025_hdl_loopback(struct adrv9025_rf_phy *phy, bool enable);
int adrv9025_register_axi_converter(struct adrv9025_rf_phy *phy);
struct adrv9025_rf_phy *adrv9025_spi_to_phy(struct spi_device *spi);
int adrv9025_spi_read(struct spi_device *spi, u32 reg);
int adrv9025_spi_write(struct spi_device *spi, u32 reg, u32 val);

#endif
