/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9002
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_TRX_ADRV9002_H_
#define IIO_TRX_ADRV9002_H_

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/types.h>

#include "adi_common_log.h"
#include "adi_adrv9001_user.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#ifdef CONFIG_DEBUG_FS
#include "adi_adrv9001_ssi.h"
#endif
#include "adi_platform_types.h"

#define ADRV_ADDRESS(port, chan)	((port) << 8 | (chan))
#define ADRV_ADDRESS_PORT(addr)		((addr) >> 8)
#define ADRV_ADDRESS_CHAN(addr)		((addr) & 0xFF)

enum {
	ADRV9002_CHANN_1,
	ADRV9002_CHANN_2,
	ADRV9002_CHANN_MAX,
};

#ifdef ADI_COMMON_VERBOSE
/*
 * Setting logEnable enables error reports which are only available if
 * ADI_COMMON_VERBOSE is defined
 */
#define	adrv9002_set_loglevel(common, level)	\
	adi_common_LogLevelSet(common, level); \
	(common)->error.logEnable = true
#else
#define	adrv9002_set_loglevel(common, level)	\
	adi_common_LogLevelSet(common, level)
#endif

enum ad900x_device_id {
	ID_ADRV9002,
	ID_ADRV9002_RX2TX2,
};

enum adrv9002_clocks {
	RX1_SAMPL_CLK,
	RX2_SAMPL_CLK,
	TX1_SAMPL_CLK,
	TX2_SAMPL_CLK,
	NUM_ADRV9002_CLKS,
};

enum adrv9002_rx_ext_info {
	RX_QEC_FIC,
	RX_QEC_W_POLY,
	RX_AGC,
	RX_TRACK_BBDC,
	RX_HD2,
	RX_RSSI_CAL,
	RX_RFDC,
	RX_RSSI,
	RX_DECIMATION_POWER,
	RX_RF_BANDWIDTH,
	RX_POWERDOWN,
	RX_GAIN_CTRL_PIN_MODE,
	RX_ENSM_MODE,
	RX_NCO_FREQUENCY,
	RX_ADC_SWITCH,
	RX_BBDC,
};

enum adrv9002_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_LB_PD,
	TX_PAC,
	TX_CLGC,
	TX_RF_BANDWIDTH,
	TX_POWERDOWN,
	TX_ATTN_CTRL_PIN_MODE,
	TX_ENSM_MODE,
	TX_NCO_FREQUENCY
};

struct adrv9002_clock {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct adrv9002_rf_phy	*phy;
	unsigned long		rate;
	enum adrv9002_clocks	source;
};

struct adrv9002_chan {
	adi_adrv9001_ChannelState_e cached_state;
	adi_common_ChannelNumber_e number;
	u32 power;
	int nco_freq;
	u8 enabled;
};

struct adrv9002_rx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_GainControlCfg *agc;
	struct adi_adrv9001_RxGainControlPinCfg *pin_cfg;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_RxSsiTestModeCfg ssi_test;
	struct adi_adrv9001_GainControlCfg debug_agc;
#endif
};

struct adrv9002_tx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_TxAttenuationPinControlCfg *pin_cfg;
	u8 dac_boost_en;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_TxSsiTestModeCfg ssi_test;
#endif
};

struct adrv9002_gpio {
	struct adi_adrv9001_GpioCfg gpio;
	u32 signal;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv9002_clock, hw)

struct adrv9002_rf_phy {
	struct spi_device		*spi;
	struct iio_dev			*indio_dev;
	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*ssi_sync;
	/* Protect against concurrent accesses to the device */
	struct mutex			lock;
	struct clk			*clks[NUM_ADRV9002_CLKS];
	struct adrv9002_clock		clk_priv[NUM_ADRV9002_CLKS];
	struct clk_onecell_data		clk_data;
	struct bin_attribute		bin;
	char				*bin_attr_buf;
	struct adrv9002_rx_chan		rx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_tx_chan		tx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_gpio 		*adrv9002_gpios;
	struct adi_adrv9001_Device	adrv9001_device;
	struct adi_adrv9001_Device	*adrv9001;
	struct adi_hal_Cfg		hal;
	struct adi_adrv9001_Init	*curr_profile;
	struct adi_adrv9001_Init	profile;
	struct adi_adrv9001_InitCals	init_cals;
	int				spi_device_id;
	int				ngpios;
	u8				rx2tx2;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_SsiCalibrationCfg ssi_delays;
#endif
};

int adrv9002_hdl_loopback(struct adrv9002_rf_phy *phy, bool enable);
int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy);
int adrv9002_axi_interface_set(struct adrv9002_rf_phy *phy, const u8 n_lanes,
			       const u8 ssi_intf, const bool cmos_ddr,
			       const int channel);
struct adrv9002_rf_phy *adrv9002_spi_to_phy(struct spi_device *spi);
int adrv9002_axi_intf_tune(struct adrv9002_rf_phy *phy, const bool tx, const int chann,
			   const adi_adrv9001_SsiType_e ssi_type, u8 *clk_delay, u8 *data_delay);
void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan, const bool en);
int adrv9002_spi_read(struct spi_device *spi, u32 reg);
int adrv9002_spi_write(struct spi_device *spi, u32 reg, u32 val);
void adrv9002_get_ssi_interface(struct adrv9002_rf_phy *phy, const int channel,
				u8 *ssi_intf, u8 *n_lanes, bool *cmos_ddr_en);
int adrv9002_ssi_configure(struct adrv9002_rf_phy *phy);
int adrv9002_post_init(struct adrv9002_rf_phy *phy);
void adrv9002_cmos_default_set(void);
int adrv9002_intf_tuning(struct adrv9002_rf_phy *phy);
int adrv9002_intf_test_cfg(struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop, const adi_adrv9001_SsiType_e ssi_type);
int adrv9002_check_tx_test_pattern(struct adrv9002_rf_phy *phy, const int chann,
				   const adi_adrv9001_SsiType_e ssi_type);
int adrv9002_intf_change_delay(struct adrv9002_rf_phy *phy, const int channel, u8 clk_delay,
			       u8 data_delay, const bool tx,
			       const adi_adrv9001_SsiType_e ssi_type);
adi_adrv9001_SsiType_e adrv9002_axi_ssi_type_get(struct adrv9002_rf_phy *phy);
/* get init structs */
struct adi_adrv9001_SpiSettings *adrv9002_spi_settings_get(void);
struct adi_adrv9001_Init *adrv9002_init_get(void);
struct adi_adrv9001_RadioCtrlInit *adrv9002_radio_ctrl_init_get(void);
struct adi_adrv9001_PlatformFiles *adrv9002_platform_files_get(void);

static inline void adrv9002_sync_gpio_toogle(const struct adrv9002_rf_phy *phy)
{
	if (phy->rx2tx2) {
		/* toogle ssi sync gpio */
		gpiod_set_value_cansleep(phy->ssi_sync, 1);
		usleep_range(5000, 5005);
		gpiod_set_value_cansleep(phy->ssi_sync, 0);
	}
}
#endif
