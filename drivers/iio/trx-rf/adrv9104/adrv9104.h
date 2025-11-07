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

#include "adi_adrv910x_common_types.h"
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

#define adrv9104_rx_to_phy(rx, nr)	\
	container_of(rx, struct adrv9104_rf_phy, rx_channels[nr])

#define adrv9104_tx_to_phy(tx)	\
	container_of(tx, struct adrv9104_rf_phy, tx_channel)

#define adrv9104_chan_to_tx(c)		\
	container_of(c, struct adrv9104_tx_chan, channel)

#define adrv9104_chan_to_rx(c)		\
	container_of(c, struct adrv9104_rx_chan, channel)

#define adrv9104_chan_to_phy(c) ({					\
	const struct adrv9104_chan *__c = (c);				\
	struct adrv9104_rf_phy *__phy;					\
									\
	if (__c->port == ADI_RX)					\
		__phy = adrv9104_rx_to_phy(chan_to_rx(__c), __c->idx);	\
	else								\
		__phy = adrv9104_tx_to_phy(chan_to_tx(__c));		\
									\
	__phy;								\
})

#define adrv9104_api_call(phy, func, args...)	({			\
	int __ret = func(&(phy)->phy_dev, ##args);			\
									\
	if (__ret)							\
		__ret = __adrv9104_dev_err(phy, __func__, __LINE__);	\
									\
	__ret;								\
})

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
	adi_common_Port_e port;
	adi_common_ChannelNumber_e channel_number;
	u8 idx;
	u8 enabled;
};

struct adrv9104_rx {
	struct adrv9104_chan channel;
};

struct adrv9104_tx {
	struct adrv9104_chan channel;
};

struct adrv9104_rf_phy {
	struct device *dev;
	struct adrv9104_chan *channels[ADRV9104_CHAN_MAX];
	/*
	 * I'm being optimistic so we'll have a way to easily (even if the old json way) get the
	 * complete profile (profile + pfirBuffer). But the below might change when all of that is
	 * figured.
	 */
	deviceProfileBundle_t profile;
	adi_adrv910x_Device_t phy_dev;
	struct adrv9104_rx rx_channels[ADRV9104_RX_MAX];
	struct adrv9104_tx tx_channel;
	struct adrv9104_hal_cfg	hal;
	/* Protect against concurrent accesses to the device */
	struct mutex lock;
};

int __adrv9104_dev_err(struct adrv9104_rf_phy *phy, const char *function, const int line);

#endif
