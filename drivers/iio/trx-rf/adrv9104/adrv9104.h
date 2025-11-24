/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9104 and similar RF Transceivers
 *
 * Copyright 2025 Analog Devices Inc.
 *
 */

#ifndef IIO_ADRV9104_H_
#define IIO_ADRV9104_H_

#include <linux/mutex.h>
#include <linux/types.h>

#include "adi_adrv910x_common_types.h"
#include "adi_adrv910x_radio_types.h"
#include "adi_adrv910x_rx_types.h"
#include "adi_adrv910x_types.h"
#include "adrv9104-linux.h"
#include "device_profile_bundle_t.h"

enum {
	ADRV9104_RX1,
	ADRV9104_RX2,
	ADRV9104_RX_MAX,
	ADRV9104_TX = ADRV9104_RX_MAX,
	ADRV9104_CHAN_MAX
};

enum {
	ADRV9104_RX1_I_SCAN,
	ADRV9104_RX1_Q_SCAN,
	ADRV9104_RX2_I_SCAN,
	ADRV9104_RX2_Q_SCAN,
	ADRV9104_TX_I_SCAN,
	ADRV9104_TX_Q_SCAN,
	ADRV9104_MAX_SCAN
};

#define adrv9104_rx_to_phy(rx, nr)	\
	container_of(rx, struct adrv9104_rf_phy, rx_channels[nr])

#define adrv9104_tx_to_phy(tx)	\
	container_of(tx, struct adrv9104_rf_phy, tx_channel)

#define adrv9104_chan_to_tx(c)		\
	container_of(c, struct adrv9104_tx, channel)

#define adrv9104_chan_to_rx(c)		\
	container_of(c, struct adrv9104_rx, channel)

#define adrv9104_chan_to_phy(c) ({						\
	const struct adrv9104_chan *__c = (c);					\
	struct adrv9104_rf_phy *__phy;						\
										\
	if (__c->port == ADI_RX)						\
		__phy = adrv9104_rx_to_phy(adrv9104_chan_to_rx(__c), __c->idx);	\
	else									\
		__phy = adrv9104_tx_to_phy(adrv9104_chan_to_tx(__c));		\
										\
	__phy;									\
})

#define adrv9104_api_call(phy, func, args...)	({			\
	int __ret = func(&(phy)->phy_dev, ##args);			\
									\
	if (__ret)							\
		__ret = __adrv9104_dev_err(phy, __func__, __LINE__);	\
									\
	__ret;								\
})

/*
 * Exactly like adrv9104_api_call() but for APIs that have to be in the calibrated state to be
 * called. For those, we have three typical steps:
 *   1) Move it device to calibrated with adrv9104_channel_to_state_cache() and cache the current
 *      state.
 *   2) Do the API call.
 *   3) Move back the device to the previous state.
 */
#define adrv9104_calibrated_api_call(phy, chan, func, args...)	({	\
	adi_adrv910x_ChannelState_e cached_state;						\
	int ____ret;										\
												\
	____ret = adrv9104_channel_to_state_cache(phy, chan, ADI_ADRV910X_CHANNEL_CALIBRATED,	\
						&cached_state);					\
	if (!____ret) {										\
		____ret = adrv9104_api_call(phy, func, args);					\
		if (!____ret)									\
			____ret = adrv9104_channel_to_state(phy, chan, cached_state);		\
	}											\
												\
	____ret;										\
})

#define adrv9104_for_each_rx(phy, rx)					\
	for (unsigned int ____c = 0;					\
	     ____c < ARRAY_SIZE((phy)->rx_channels) && (rx = &(phy)->rx_channels[____c], true); \
	     ____c++)

#define adrv9104_for_each_enabled_rx(phy, rx)	\
	adrv9104_for_each_rx(phy, rx)		\
		if (rx->channel.enabled)

#define adrv9104_for_each_chan(phy, chan)				\
	for (unsigned int ____c = 0;					\
	     ____c < ARRAY_SIZE((phy)->channels) && (chan = (phy)->channels[____c], true); \
	     ____c++)

#define adrv9104_for_each_enabled_chan(phy, chan)	\
	adrv9104_for_each_chan(phy, chan)		\
		if (chan->enabled)

#ifdef ADI_COMMON_VERBOSE
/*
 * Enable log if ADI_COMMON_VERBOSE is defined
 */
#define	adrv9104_log_enable(common)	\
	(common)->error.logEnable = true
#else
#define	adrv9104_log_enable(...)
#endif

struct adrv9104_chan {
	struct iio_backend *back;
	u32 rate;
	adi_common_Port_e port;
	adi_common_ChannelNumber_e number;
	bool power_down;
	u8 idx;
	u8 enabled;
};

struct adrv9104_rx {
	struct adrv9104_chan channel;
	struct adi_adrv910x_GainControlCfg agc;
};

struct adrv9104_tx {
	struct adrv9104_chan channel;
	bool dac_boost_en;
	bool txnb;
};

struct adrv9104_rf_phy {
	struct device *dev;
	struct adrv9104_chan *channels[ADRV9104_CHAN_MAX];
	/*
	 * I'm being optimistic so we'll have a way to easily (even if the old json way) get the
	 * complete profile (profile + pfirBuffer). But the below might change when all of that is
	 * figured.
	 */
	union {
		deviceProfileBundle_t profile_bundle;
		struct {
			deviceProfile_t profile;
			pfirBuffer_t pfirBuffer;
		};
	};

	adi_adrv910x_Device_t phy_dev;
	struct adrv9104_rx rx_channels[ADRV9104_RX_MAX];
	struct adrv9104_tx tx_channel;
	struct adrv9104_hal_cfg	hal;
	/* Protect against concurrent accesses to the device */
	struct mutex lock;
};

int __adrv9104_dev_err(struct adrv9104_rf_phy *phy, const char *function, const int line);
int adrv9104_channel_to_state_cache(struct adrv9104_rf_phy *phy, const struct adrv9104_chan *chan,
				    adi_adrv910x_ChannelState_e state,
				    adi_adrv910x_ChannelState_e *curr);
#define adrv9104_channel_to_state(phy, chan, state)	\
	adrv9104_channel_to_state_cache(phy, chan, state, NULL)

#endif
