/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV904X
 *
 * Copyright 2020-2026 Analog Devices Inc.
 */

#ifndef IIO_TRX_ADRV904X_H_
#define IIO_TRX_ADRV904X_H_

#include "adi_adrv904x_utilities_types.h"
#include "adi_adrv904x_utilities.h"
#include "adi_adrv904x_radioctrl.h"
#include "adi_common_error_types.h"
#include "adi_adrv904x_version.h"
#include "adi_adrv904x_error.h"
#include "adi_adrv904x_cals.h"
#include "adi_adrv904x_gpio.h"
#include "adi_adrv904x_user.h"
#include "adi_adrv904x_agc.h"
#include "adi_adrv904x_hal.h"
#include "adi_adrv904x_rx.h"
#include "adi_adrv904x_tx.h"
#include "adi_platform.h"

#include <linux/clk-provider.h>
#include <linux/iio/iio.h>

#define MIN_GAIN_mdB 0
#define MAX_RX_GAIN_mdB 32000
#define MAX_OBS_RX_GAIN_mdB 32000
#define RX_GAIN_STEP_mdB 500

enum debugfs_cmd {
	DBGFS_NONE,
	DBGFS_BIST_FRAMER_0_PRBS,
	DBGFS_BIST_FRAMER_LOOPBACK,
	DBGFS_BIST_TONE,
};

enum adrv904x_rx_ext_info {
	RX_QEC,
	RX_DIG_DC,
	RX_RF_BANDWIDTH,
	RX_ADC,
};

enum adrv904x_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_RF_BANDWIDTH,
	TX_LB_ADC,
};

enum adrv904x_iio_voltage_in {
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

enum adrv904x_iio_voltage_out {
	CHAN_TX1,
	CHAN_TX2,
	CHAN_TX3,
	CHAN_TX4,
	CHAN_TX5,
	CHAN_TX6,
	CHAN_TX7,
	CHAN_TX8,
};

struct adrv904x_chip_info {
	const char *name;
};

struct adrv904x_rf_phy;
struct adrv904x_debugfs_entry {
	struct adrv904x_rf_phy *phy;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

enum adrv904x_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	OBS_SAMPL_CLK,
	NUM_ADRV904X_CLKS,
};

struct adrv904x_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct adrv904x_rf_phy *phy;
	unsigned long rate;
	enum adrv904x_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv904x_clock, hw)
#define MAX_NUM_GAIN_TABLES 10

struct adrv904x_rf_phy {
	struct spi_device *spi;
	adi_adrv904x_Device_t adi_adrv904x_device;
	adi_adrv904x_Device_t *kororDevice;
	adi_adrv904x_SpiConfigSettings_t spiSettings;
	adi_adrv904x_SpiOptions_t spiOptions;
	adi_adrv904x_Init_t deviceInitStruct;
	adi_adrv904x_TrxFileInfo_t trxBinaryInfoPtr;
	adi_adrv904x_PostMcsInit_t adrv904xPostMcsInitInst;
	adi_adrv904x_InitCals_t cal_mask;

	struct jesd204_dev	*jdev;
	/* protect against device accesses */
	struct mutex		lock;

	u32 tx_iqRate_kHz;
	u32 rx_iqRate_kHz;
	u32 orx_iqRate_kHz;

	adi_hal_Cfg_t linux_hal;
	struct clk *dev_clk;

	struct clk *clks[NUM_ADRV904X_CLKS];
	struct adrv904x_clock clk_priv[NUM_ADRV904X_CLKS];
	struct clk_hw_onecell_data *clk_data;
	struct adrv904x_debugfs_entry debugfs_entry[342];
	struct iio_dev *indio_dev;

	struct gpio_desc *sysref_req_gpio;

	const struct adrv904x_chip_info *chip_info;

	u32 adrv904x_debugfs_entry_index;
	u32 tracking_cal_mask;

	bool is_initialized;
};

int __adrv904x_dev_err(struct adrv904x_rf_phy *phy, const char *function, int line);

#define adrv904x_api_call(phy, func, args...) ({				\
	int __ret = func((phy)->kororDevice, ##args);				\
	if (__ret)								\
		__ret = __adrv904x_dev_err(phy, __func__, __LINE__);		\
	__ret;									\
})

int adrv904x_register_axi_converter(struct adrv904x_rf_phy *phy);
int adrv904x_spi_read(struct spi_device *spi, u32 reg);
int adrv904x_spi_write(struct spi_device *spi, u32 reg, u32 val);

struct iio_dev;
#ifdef CONFIG_DEBUG_FS
void adrv904x_register_debugfs(struct iio_dev *indio_dev);
#else
static inline void adrv904x_register_debugfs(struct iio_dev *indio_dev) {}
#endif

#endif
