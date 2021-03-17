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
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_ssi_types.h"
#include "linux_platform.h"

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
 * Enable log if ADI_COMMON_VERBOSE is defined
 */
#define	adrv9002_log_enable(common)	\
	(common)->error.logEnable = true
#else
#define	adrv9002_log_enable(common)
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
	TDD1_INTF_CLK,
	TDD2_INTF_CLK,
	NUM_ADRV9002_CLKS,
};

enum adrv9002_rx_ext_info {
	RX_QEC_FIC,
	RX_QEC_W_POLY,
	ORX_QEC_W_POLY,
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

#define rx_to_phy(rx, nr)	\
	container_of(rx, struct adrv9002_rf_phy, rx_channels[nr])

#define tx_to_phy(tx, nr)	\
	container_of(tx, struct adrv9002_rf_phy, tx_channels[nr])

#define chan_to_tx(c)		\
	container_of(c, struct adrv9002_tx_chan, channel)

#define chan_to_rx(c)		\
	container_of(c, struct adrv9002_rx_chan, channel)

#define chan_to_phy(c) ({						\
	struct adrv9002_chan *__c = (c);				\
	struct adrv9002_rf_phy *__phy;					\
									\
	if (__c->port == ADI_RX)					\
		__phy = rx_to_phy(chan_to_rx(__c), __c->idx);	\
	else								\
		__phy = tx_to_phy(chan_to_tx(__c), __c->idx);	\
									\
	__phy;								\
})

struct adrv9002_clock {
	struct clk_hw		hw;
	struct spi_device	*spi;
	struct adrv9002_rf_phy	*phy;
	unsigned long		rate;
	enum adrv9002_clocks	source;
};

struct adrv9002_chan {
	struct clk *clk;
	/*
	 * These values are in nanoseconds. They need to be converted with
	 * @adrv9002_chan_ns_to_en_delay() before passing them to the API.
	 */
	struct adi_adrv9001_ChannelEnablementDelays en_delays_ns;
	unsigned long rate;
	adi_adrv9001_ChannelState_e cached_state;
	adi_common_ChannelNumber_e number;
	adi_common_Port_e port;
	u32 power;
	int nco_freq;
	u8 idx;
	u8 enabled;;
};

struct adrv9002_rx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_GainControlCfg agc;
	struct adi_adrv9001_RxGainControlPinCfg *pin_cfg;
	struct clk *tdd_clk;
	struct gpio_desc *orx_gpio;
	u8 orx_en;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_RxSsiTestModeCfg ssi_test;
#endif
};

struct adrv9002_tx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_TxAttenuationPinControlCfg *pin_cfg;
	u8 dac_boost_en;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_TxSsiTestModeCfg ssi_test;
	u8 loopback;
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
	char				*bin_attr_buf;
	size_t				bin_attr_sz;
	u8				*stream_buf;
	u16				stream_size;
	struct adrv9002_rx_chan		rx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_tx_chan		tx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_chan		*channels[ADRV9002_CHANN_MAX * 2];
	struct adrv9002_gpio 		*adrv9002_gpios;
	struct adi_adrv9001_Device	adrv9001_device;
	struct adi_adrv9001_Device	*adrv9001;
	struct adrv9002_hal_cfg		hal;
	struct adi_adrv9001_Init	*curr_profile;
	struct adi_adrv9001_Init	profile;
	struct adi_adrv9001_InitCals	init_cals;
	u32				n_clks;
	int				spi_device_id;
	int				ngpios;
	u8				rx2tx2;
	/* ssi type of the axi cores - cannot really change at runtime */
	enum adi_adrv9001_SsiType	ssi_type;
	/*
	 * Tells if TX only profiles are valid. If not set, it means that TX1/TX2 SSI clocks are
	 * derived from RX1/RX2 which means that TX cannot be enabled if RX is not...
	 */
	u8				tx_only;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_SsiCalibrationCfg ssi_delays;
#endif
};

void adrv9002_en_delays_ns_to_arm(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d_ns,
				  struct adi_adrv9001_ChannelEnablementDelays *d);
void adrv9002_en_delays_arm_to_ns(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d,
				  struct adi_adrv9001_ChannelEnablementDelays *d_ns);
/* phy lock must be held before entering the API */
int adrv9002_channel_to_state(struct adrv9002_rf_phy *phy, struct adrv9002_chan *chann,
			      const adi_adrv9001_ChannelState_e state, const bool cache_state);
int adrv9002_init(struct adrv9002_rf_phy *phy, struct adi_adrv9001_Init *profile);
int __adrv9002_dev_err(const struct adrv9002_rf_phy *phy, const char *function, const int line);
#define adrv9002_dev_err(phy)	__adrv9002_dev_err(phy, __func__, __LINE__)

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy);
int adrv9002_axi_interface_set(struct adrv9002_rf_phy *phy, const u8 n_lanes,
			       const bool cmos_ddr, const int channel, const bool tx);
int adrv9002_axi_intf_tune(struct adrv9002_rf_phy *phy, const bool tx, const int chann,
			   u8 *clk_delay, u8 *data_delay);
void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan, const bool tx,
				   const bool en);
int __maybe_unused adrv9002_axi_tx_test_pattern_cfg(struct adrv9002_rf_phy *phy, const int channel,
						    const adi_adrv9001_SsiTestModeData_e data);
int adrv9002_post_init(struct adrv9002_rf_phy *phy);
int adrv9002_intf_test_cfg(struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop);
int adrv9002_check_tx_test_pattern(struct adrv9002_rf_phy *phy, const int chann);
int adrv9002_intf_change_delay(struct adrv9002_rf_phy *phy, const int channel, u8 clk_delay,
			       u8 data_delay, const bool tx);
u32 adrv9002_axi_dds_rate_get(struct adrv9002_rf_phy *phy, const int chan);
void adrv9002_axi_hdl_loopback(struct adrv9002_rf_phy *phy, int channel, bool enable);

static inline void adrv9002_sync_gpio_toogle(const struct adrv9002_rf_phy *phy)
{
	if (phy->rx2tx2) {
		/* toogle ssi sync gpio */
		gpiod_set_value_cansleep(phy->ssi_sync, 1);
		usleep_range(5000, 5005);
		gpiod_set_value_cansleep(phy->ssi_sync, 0);
	}
}

#ifdef CONFIG_DEBUG_FS
void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy, struct dentry *d);
#else
void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy, struct dentry *d) {}
#endif
#endif
