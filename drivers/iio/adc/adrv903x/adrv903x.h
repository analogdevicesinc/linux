/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV903X
 *
 * Copyright 2020-2026 Analog Devices Inc.
 */

#ifndef IIO_TRX_ADRV903X_H_
#define IIO_TRX_ADRV903X_H_

#include <linux/clk-provider.h>
#include <linux/iio/iio.h>

#include "adi_adrv903x_utilities_types.h"
#include "adi_adrv903x_datainterface.h"
#include "adi_adrv903x_utilities.h"
#include "adi_adrv903x_radioctrl.h"
#include "adi_common_error_types.h"
#include "adi_adrv903x_version.h"
#include "adi_adrv903x_error.h"
#include "adi_adrv903x_cals.h"
#include "adi_adrv903x_core.h"
#include "adi_adrv903x_gpio.h"
#include "adi_adrv903x_user.h"
#include "adi_adrv903x_agc.h"
#include "adi_adrv903x_hal.h"
#include "adi_adrv903x_rx.h"
#include "adi_adrv903x_tx.h"
#include "adi_platform.h"

#define MIN_GAIN_mdB 0
#define MAX_RX_GAIN_mdB 32000
#define MAX_OBS_RX_GAIN_mdB 32000
#define RX_GAIN_STEP_mdB 500

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_BIST_FRAMER_0_PRBS,
	DBGFS_BIST_FRAMER_1_PRBS,
	DBGFS_BIST_FRAMER_0_LOOPBACK,
	DBGFS_BIST_FRAMER_1_LOOPBACK,
	DBGFS_BIST_TONE,
	DBGFS_TX0_QEC_STATUS,
	DBGFS_TX1_QEC_STATUS,
	DBGFS_TX2_QEC_STATUS,
	DBGFS_TX3_QEC_STATUS,
	DBGFS_TX4_QEC_STATUS,
	DBGFS_TX5_QEC_STATUS,
	DBGFS_TX6_QEC_STATUS,
	DBGFS_TX7_QEC_STATUS,
	DBGFS_TX0_LOL_STATUS,
	DBGFS_TX1_LOL_STATUS,
	DBGFS_TX2_LOL_STATUS,
	DBGFS_TX3_LOL_STATUS,
	DBGFS_TX4_LOL_STATUS,
	DBGFS_TX5_LOL_STATUS,
	DBGFS_TX6_LOL_STATUS,
	DBGFS_TX7_LOL_STATUS,
	DBGFS_RX0_QEC_STATUS,
	DBGFS_RX1_QEC_STATUS,
	DBGFS_RX2_QEC_STATUS,
	DBGFS_RX3_QEC_STATUS,
	DBGFS_RX4_QEC_STATUS,
	DBGFS_RX5_QEC_STATUS,
	DBGFS_RX6_QEC_STATUS,
	DBGFS_RX7_QEC_STATUS,
	DBGFS_RX0_ADC_STATUS,
	DBGFS_RX1_ADC_STATUS,
	DBGFS_RX2_ADC_STATUS,
	DBGFS_RX3_ADC_STATUS,
	DBGFS_RX4_ADC_STATUS,
	DBGFS_RX5_ADC_STATUS,
	DBGFS_RX6_ADC_STATUS,
	DBGFS_RX7_ADC_STATUS,
	DBGFS_ORX0_ADC_STATUS,
	DBGFS_ORX1_ADC_STATUS,
};

enum adrv903x_rx_ext_info {
	RX_QEC,
	RX_DIG_DC,
	RX_RF_BANDWIDTH,
	RX_ADC,
	RX_NCO_EN,
	RX_NCO_FREQ,
	RX_NCO_PHASE,
	RX_NCO_BAND_SEL,
	ORX_NCO_EN,
	ORX_NCO_FREQ,
	ORX_NCO_PHASE,
	ORX_NCO_SEL,
};

enum adrv903x_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_RF_BANDWIDTH,
	TX_LB_ADC,
	TX_TEST_TONE_EN,
	TX_TEST_TONE_FREQ,
	TX_TEST_TONE_PHASE,
	TX_TEST_TONE_NCO_SEL,
	TX_TEST_TONE_ATTEN,
};

enum adrv903x_iio_voltage_in {
	CHAN_RX1,
	CHAN_RX2,
	CHAN_RX3,
	CHAN_RX4,
	CHAN_RX5,
	CHAN_RX6,
	CHAN_RX7,
	CHAN_RX8,
	CHAN_OBS_RX1,
	CHAN_OBS_RX2,
};

enum adrv903x_iio_voltage_out {
	CHAN_TX1,
	CHAN_TX2,
	CHAN_TX3,
	CHAN_TX4,
	CHAN_TX5,
	CHAN_TX6,
	CHAN_TX7,
	CHAN_TX8,
};

struct adrv903x_chip_info {
	const char *name;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	u8 device_id;
	bool is_adrv9032r;
};

struct adrv903x_debugfs_entry {
	struct adrv903x_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

enum adrv903x_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	OBS_SAMPL_CLK,
	NUM_ADRV903X_CLKS,
};

struct adrv903x_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct adrv903x_rf_phy *phy;
	unsigned long rate;
	enum adrv903x_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv903x_clock, hw)
#define MAX_NUM_GAIN_TABLES 10

struct adrv903x_rf_phy {
	struct spi_device *spi;
	adi_adrv903x_Device_t adi_adrv903x_device;
	adi_adrv903x_Device_t *palauDevice;
	adi_adrv903x_SpiConfigSettings_t spiSettings;
	adi_adrv903x_SpiOptions_t spiOptions;
	adi_adrv903x_Init_t deviceInitStruct;
	adi_adrv903x_TrxFileInfo_t trxBinaryInfoPtr;
	adi_adrv903x_PostMcsInit_t adrv903xPostMcsInitInst;
	adi_adrv903x_InitCals_t cal_mask;

	struct jesd204_dev	*jdev;
	/* protect against device accesses */
	struct mutex		lock;

	u32 tx_iqRate_kHz;
	u32 rx_iqRate_kHz;
	u32 orx_iqRate_kHz;

	adi_hal_Cfg_t linux_hal;
	struct clk *dev_clk;

	struct clk *clks[NUM_ADRV903X_CLKS];
	struct adrv903x_clock clk_priv[NUM_ADRV903X_CLKS];
	struct clk_hw_onecell_data *clk_data;
	struct adrv903x_debugfs_entry debugfs_entry[342];
	struct iio_dev *indio_dev;

	struct gpio_desc *sysref_req_gpio;

	u8 device_id;

	u32 adrv903x_debugfs_entry_index;
	u32 tracking_cal_mask;

	bool is_initialized;
	const struct adrv903x_chip_info *chip_info;
	bool tx_test_tone_en[8];
	u32 tx_test_tone_freq_khz[8];
	u32 tx_test_tone_phase[8];
	u8 tx_test_tone_nco_sel[8];
	u8 tx_test_tone_atten[8];
	bool rx_nco_en[8];
	s32 rx_nco_freq_khz[8];
	u32 rx_nco_phase[8];
	u8 rx_nco_band_sel[8];
	bool orx_nco_en[2];
	s32 orx_nco_freq_khz[2];
	u32 orx_nco_phase[2];
	u8 orx_nco_sel[2];
};

int adrv903x_hdl_loopback(struct adrv903x_rf_phy *phy, bool enable);
int adrv903x_register_axi_converter(struct adrv903x_rf_phy *phy);
struct adrv903x_rf_phy *adrv903x_spi_to_phy(struct spi_device *spi);
int adrv903x_spi_read(struct spi_device *spi, u32 reg);
int adrv903x_spi_write(struct spi_device *spi, u32 reg, u32 val);

#endif
