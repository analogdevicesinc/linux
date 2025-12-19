// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 and similar RF Transceiver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include "linux/dev_printk.h"
#include "linux/iio/types.h"
#include <linux/array_size.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/kstrtox.h>
#include <linux/lockdep.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/units.h>
#include <linux/wordpart.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>

#include "adi_adrv910x.h"
#include "adi_adrv910x_arm.h"
#include "adi_adrv910x_cals.h"
#include "adi_adrv910x_radio.h"
#include "adi_adrv910x_rx.h"
#include "adi_adrv910x_spi.h"
#include "adi_adrv910x_stream.h"
#include "adi_adrv910x_tx.h"

#include "adi_common_error.h"

#include "device_profile_rx_dp_t.h"

#include "adrv9104.h"
#include "adrv9104-backend.h"
#include "adrv9104-profile.h"
#include "adrv9104-rx-gain-table.h"
#include "adrv9104-tx-atten-table.h"

#define ADRV9104_ARM_FW_SIZE_BYTES	393216
#define ADRV9104_STREAM_FW_SIZE_BYTES	32768
#define ADRV9104_RX_GAIN_STEP_mDB	500
#define ADRV9104_RX_MAX_GAIN_mdB	\
	((ADI_ADRV910X_RX_GAIN_INDEX_MAX - ADI_ADRV910X_RX_GAIN_INDEX_MIN) *	\
	 ADRV9104_RX_GAIN_STEP_mDB)

#define ADRV9104_RX_BIT_START		(ffs(ADI_ADRV910X_RX1) - 1)
#define ADRV9104_TX_BIT			(ffs(ADI_ADRV910X_TX1) - 1)
#define ADRV9104_TXNB_BIT		(ffs(ADI_ADRV910X_TXNB) - 1)

/* Gives the port position in an array of ADRV9104_CHAN_MAX + 1 */
#define ADRV9104_PORT_POS(chan)		((chan)->port * 2 + (chan)->idx)

enum {
	ADRV9104_IIO_RX1,
	ADRV9104_IIO_RX2,
	ADRV9104_IIO_RX_MAX,
	ADRV9104_IIO_TX = 0,
};

int __adrv9104_dev_err(struct adrv9104_rf_phy *phy, const char *function, int line)
{
	dev_err(phy->dev, "%s, %d: failed with \"%s\" (%d)\n", function, line,
		phy->phy_dev.common.error.errormessage[0] != '\0' ?
		phy->phy_dev.common.error.errormessage : "",
		phy->phy_dev.common.error.errCode);

	adi_common_ErrorClear(&phy->phy_dev.common);

	switch (phy->phy_dev.common.error.errCode) {
	case ADI_COMMON_ERR_INV_PARAM:
	case ADI_COMMON_ERR_NULL_PARAM:
		return -EINVAL;
	case ADI_COMMON_ERR_API_FAIL:
		/*
		 * Not really a meaningfull translation. Just something to be different from the
		 * default case.
		 */
		return -ENXIO;
	case ADI_COMMON_ERR_SPI_FAIL:
		return -EIO;
	case ADI_COMMON_ERR_MEM_ALLOC_FAIL:
		return -ENOMEM;
	default:
		return -EFAULT;
	}
}

static struct adrv9104_chan *adrv9104_get_rf_chan_from_iio(struct adrv9104_rf_phy *phy,
							   const struct iio_chan_spec *iio_chan)
{
	struct adrv9104_chan *chan;

	if (iio_chan->type != IIO_VOLTAGE)
		return ERR_PTR(-EINVAL);

	if (iio_chan->output && iio_chan->channel == ADRV9104_IIO_TX)
		chan = &phy->tx_channel.channel;
	else if (!iio_chan->output && iio_chan->channel < ADRV9104_IIO_RX_MAX)
		chan = &phy->rx_channels[iio_chan->channel].channel;
	else
		return ERR_PTR(-ENOENT);

	lockdep_assert_held(&phy->lock);
	if (!chan->enabled)
		return ERR_PTR(-ENODEV);

	return chan;
}

int adrv9104_channel_to_state_cache(struct adrv9104_rf_phy *phy, const struct adrv9104_chan *chan,
				    adi_adrv910x_ChannelState_e state,
				    adi_adrv910x_ChannelState_e *curr)
{
	enum adi_adrv910x_ChannelEnableMode pin_mode;
	adi_adrv910x_ChannelState_e curr_state;
	int ret;

	lockdep_assert_held(&phy->lock);

	/*
	 * We can only move state each state at time. So we need to care if for example we want
	 * to move to RF_ENABLED from CALIBRATED. Hence, get the current state and act accordingly.
	 */
	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_State_Get, chan->port,
				chan->number, &curr_state);
	if (ret)
		return ret;
	if (state == curr_state)
		return 0;

	if (curr)
		*curr = curr_state;

	/*
	 * Check if pin mode is enabled. If so, still allow to control the radio state if we are
	 * in control of the enable GPIO. Also note that in pin mode we can only move between
	 * primed and rf_enabled so error out if we try to move to calibrated.
	 */
	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_ChannelEnableMode_Get, chan->port,
				chan->number, &pin_mode);
	if (ret)
		return ret;

	if (pin_mode == ADI_ADRV910X_PIN_MODE) {
		if (!chan->ensm) {
			dev_err(phy->dev,
				"Can't change ensm state! Pin mode enabled and no enable pin for p:%u c:%u\n",
				chan->port, chan->number);
			return -EPERM;
		}

		if (state == ADI_ADRV910X_CHANNEL_CALIBRATED) {
			dev_err(phy->dev,
				"Cannot move p:%u, c:%u to calibrated in pin mode! Use SPI mode instead.\n",
				chan->port, chan->number);
			return -EINVAL;
		}

		gpiod_set_value_cansleep(chan->ensm, state == ADI_ADRV910X_CHANNEL_RF_ENABLED);
		return 0;
	}

	switch (state) {
	case ADI_ADRV910X_CHANNEL_CALIBRATED:
		if (curr_state == ADI_ADRV910X_CHANNEL_RF_ENABLED) {
			ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_EnableRf,
						chan->port, chan->number, false);
			if (ret)
				return ret;
		}

		return adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_Prime,
					 chan->port, chan->number, false);
	case ADI_ADRV910X_CHANNEL_PRIMED:
		/* Either I need to prime or to disable RF */
		if (curr_state == ADI_ADRV910X_CHANNEL_RF_ENABLED) {
			ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_EnableRf,
						chan->port, chan->number, false);
			if (ret)
				return ret;
		}

		return adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_Prime,
					 chan->port, chan->number, true);
	case ADI_ADRV910X_CHANNEL_RF_ENABLED:
		if (curr_state == ADI_ADRV910X_CHANNEL_CALIBRATED) {
			ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_Prime,
						chan->port, chan->number, true);
			if (ret)
				return ret;
		}

		/* at this point, I must be primed */
		ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_EnableRf,
					chan->port, chan->number, true);
		if (ret)
			return ret;
		/*
		 * Transitioning to RF_ENABLED might take some time. So let's poll the state so
		 * that we are sure we are on proper state when leaving.
		 */
		return read_poll_timeout(adi_adrv910x_Radio_Channel_State_Get, ret,
					 (ret || curr_state == state), MILLI, 10 * MILLI, false,
					 &phy->phy_dev, chan->port, chan->number, &curr_state);
	default:
		return -EINVAL;
	}
}

static int adrv9104_carrier_set(struct adrv9104_rf_phy *phy, const struct adrv9104_chan *chan,
				u64 freq_hz, u64 *old_freq_hz)
{
	struct adi_adrv910x_Carrier carrier = {};
	int ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Carrier_Inspect, chan->port,
				chan->number, &carrier);
	if (ret)
		return ret;

	if (old_freq_hz)
		*old_freq_hz = carrier.carrierFrequency_Hz;

	carrier.carrierFrequency_Hz = freq_hz;

	return adrv9104_api_call(phy, adi_adrv910x_Radio_Carrier_Configure,
				 chan->port, chan->number, &carrier);
}

static int adrv9104_hardware_gain_set(struct adrv9104_rf_phy *phy,
				      const struct iio_chan_spec *iio_chan, int val, int val2)
{
	struct adrv9104_chan *chan;
	int ret, gain;
	u8 idx;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	if (chan->port == ADI_TX) {
		u16 atten_mdb;

		if (val > 0 || (val == 0 && val2 > 0))
			return -EINVAL;

		atten_mdb = -1 * (val * 1000 + val2 / 1000);
		ret = adrv9104_api_call(phy, adi_adrv910x_Tx_Attenuation_Set, atten_mdb);
		if (ret)
			return ret;

		return 0;
	}

	gain = val * 1000 + val2 / 1000;
	clamp(gain, 0, ADRV9104_RX_MAX_GAIN_mdB);
	idx = DIV_ROUND_CLOSEST(gain, ADRV9104_RX_GAIN_STEP_mDB) + ADI_ADRV910X_RX_GAIN_INDEX_MIN;

	return adrv9104_api_call(phy, adi_adrv910x_Rx_Gain_Set, chan->number, idx);
}

static int adrv9104_rerun_init_cals(struct adrv9104_rf_phy *phy, const struct adrv9104_chan *chan,
				    initCals_t *init_cals)
{
	/*
	 * In theory we only need two states as we can only have two channels on a different LO but
	 * we do it like this to be consistent.
	 * Also, +1 because when TXNB is enabled we set the index to 1 for API calls.
	 */
	adi_adrv910x_ChannelState_e state[ADRV9104_CHAN_MAX + 1];
	struct adrv9104_chan *__chan;
	u8 error;
	int ret;

	adrv9104_for_each_enabled_chan(phy, __chan) {
		adrv9104_backend_disable(__chan->back);

		/* if on the same lo, it must be moved already */
		if (__chan->lo == chan->lo)
			continue;

		ret = adrv9104_channel_to_state_cache(phy, __chan, ADI_ADRV910X_CHANNEL_CALIBRATED,
						      &state[ADRV9104_PORT_POS(__chan)]);
		if (ret)
			return ret;
	}

	ret = adrv9104_api_call(phy, adi_adrv910x_cals_InitCals_Run, init_cals, 6000, &error);
	if (ret)
		return ret;

	adrv9104_for_each_enabled_chan(phy, __chan) {
		if (__chan->lo != chan->lo) {
			ret = adrv9104_channel_to_state(phy, __chan,
							state[ADRV9104_PORT_POS(__chan)]);
			if (ret)
				return ret;
		}

		ret = adrv9104_backend_enable(__chan->back);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Handling carrier frequency is complex due to the device's two Local Oscillators (LOs),
 * despite the apparent availability of three independent controls. This behavior is
 * dictated by the LO mappings defined in the current profile.
 *
 * First, for a carrier frequency change (and subsequent PLL re-tuning) to take effect,
 * all ports sharing the same LO must first be moved into the calibrated state before
 * modification. Failing to do so can lead to an inconsistent state.
 *
 * For example, consider the following steps in an FDD profile where RX1 and RX2 share LO1
 * (assuming both ports initially operate at 2.4GHz):
 *   1) Move RX1 carrier to 2.45GHz -> LO1 re-tunes.
 *   2) Move RX1 back to 2.4GHz -> LO1 does not re-tune, and the actual carrier remains at 2.45GHz.
 * Consequently, both ports would *appear* to be at 2.4GHz, but the *actual* carrier frequency
 * would remain 2.45GHz.
 *
 * Second, when multiple ports of the same type share a single LO, as is common in FDD and
 * TDD (with diversity) profiles, their carrier frequencies must be changed simultaneously.
 * It is not possible for multiple enabled ports sharing the same LO to operate at different
 * carrier frequencies.
 *
 * In TDD profiles, TX and RX ports are never moved together, even if they share an LO.
 * The assumption is that there is sufficient time to re-tune between RX and TX frames.
 * If this is not the case, TX and RX carriers must be manually set to the same value
 * before operation begins.
 */
static int adrv9104_carrier_freq_set(struct adrv9104_rf_phy *phy,
				     const struct iio_chan_spec *iio_chan, int val, int val2)
{
	adi_adrv910x_ChannelState_e state[ADRV9104_CHAN_MAX + 1];
	struct adrv9104_chan *chan, *__chan;
	u64 carrier_freq_hz, old_hz;
	initCals_t init_cals = {};
	int ret;

	if (val < 0 || val2 < 0)
		return -EINVAL;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	carrier_freq_hz = ((u64)val2 << 32) | val;

	adrv9104_for_each_enabled_chan(phy, __chan) {
		if (__chan->lo != chan->lo)
			continue;

		ret = adrv9104_channel_to_state_cache(phy, __chan, ADI_ADRV910X_CHANNEL_CALIBRATED,
						      &state[ADRV9104_PORT_POS(__chan)]);
		if (ret)
			return ret;
	}

	/*
	 * Now move the carrier for all channels of the same port. Naturally, for TX
	 * we only have 1.
	 */
	if (chan->port == ADI_TX) {
		ret = adrv9104_carrier_set(phy, chan, carrier_freq_hz, &old_hz);
		if (ret)
			return ret;

		/* Run on RX/TX */
		init_cals.chanInitCalMask[__chan->idx] = INIT_LO_RETUNE;
	} else {
		/* Only run on RX */
		initCalibrations_e rx_cals = INIT_LO_RETUNE & ~INIT_CAL_TX_ALL;
		struct adrv9104_rx *rx;

		adrv9104_for_each_enabled_rx(phy, rx) {
			if (rx->channel.lo != chan->lo)
				continue;

			ret = adrv9104_carrier_set(phy, &rx->channel, carrier_freq_hz, &old_hz);
			if (ret)
				return ret;

			init_cals.chanInitCalMask[rx->channel.idx] |= rx_cals;
		}
	}

	if (abs(carrier_freq_hz - old_hz) >= 100 * MEGA && phy->rerun_calls) {
		ret = adrv9104_rerun_init_cals(phy, chan, &init_cals);
		if (ret)
			return ret;
	}

	adrv9104_for_each_enabled_chan(phy, __chan) {
		/* If it's me... defer. See below */
		if (__chan->port == chan->port && __chan->idx == chan->idx)
			continue;
		if (__chan->lo != chan->lo)
			continue;

		ret = adrv9104_channel_to_state(phy, __chan, state[ADRV9104_PORT_POS(__chan)]);
		if (ret)
			return ret;
	}

	/*
	 * This ordering ensures that the channel whose carrier is being changed is the
	 * last one to transition state. This is critical in TDD profiles because a
	 * transition from 'calibrated' to 'prime' also causes the PLL to re-lock to
	 * the port's carrier.
	 *
	 * Consider a scenario where RX1 and TX1 are both on LO1:
	 *	1. TX1 carrier set to 2.45GHz and primed.
	 *	2. RX1 carrier set to 2.4GHz and primed.
	 *	3. Change RX1 to rf_enabled and set its carrier to 2.5GHz.
	 *
	 * If TX1 were moved from 'calibrated' to 'prime' after RX1's change, it would
	 * trigger a re-lock, potentially causing LO1 to end up at 2.4GHz, which is
	 * unexpected.
	 */
	return adrv9104_channel_to_state(phy, chan, state[ADRV9104_PORT_POS(chan)]);
}

static int adrv9104_phy_write_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adrv9104_hardware_gain_set(phy, chan, val, val2);
	case IIO_CHAN_INFO_FREQUENCY:
		return adrv9104_carrier_freq_set(phy, chan, val, val2);
	default:
		return -EINVAL;
	}
}

static int adrv9104_write_raw_get_fmt(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		return IIO_VAL_INT_64;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
}

static int adrv9104_hardware_gain_get(struct adrv9104_rf_phy *phy,
				      const struct iio_chan_spec *iio_chan,
				      int *val, int *val2)
{
	struct adrv9104_chan *chan;
	int ret, temp;
	u8 index;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	/* Make sure this actually works! For now it's just a copy from adrv9002 */
	if (chan->port == ADI_TX) {
		u16 atten_mdb;

		ret = adrv9104_api_call(phy, adi_adrv910x_Tx_Attenuation_Get, &atten_mdb);
		if (ret)
			return ret;

		*val = -1 * (atten_mdb / 1000);
		*val2 = (atten_mdb % 1000) * 1000;
		if (!*val)
			*val2 *= -1;

		return IIO_VAL_INT_PLUS_MICRO_DB;
	}

	ret = adrv9104_api_call(phy, adi_adrv910x_Rx_Gain_Get, chan->number, &index);
	if (ret)
		return ret;

	clamp(index, ADI_ADRV910X_RX_GAIN_INDEX_MIN, ADI_ADRV910X_RX_GAIN_INDEX_MAX);
	temp = (index - ADI_ADRV910X_RX_GAIN_INDEX_MIN) * ADRV9104_RX_GAIN_STEP_mDB;

	*val = temp / 1000;
	*val2 = temp % 1000 * 1000;

	return IIO_VAL_INT_PLUS_MICRO_DB;
}

static int adrv9104_phy_read_sample_rate(struct adrv9104_rf_phy *phy,
					 const struct iio_chan_spec *iio_chan, int *val)
{
	struct adrv9104_chan *chan;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	*val = chan->rate;

	return IIO_VAL_INT;
}

static int adrv9104_carrier_freq_get(struct adrv9104_rf_phy *phy,
				     const struct iio_chan_spec *iio_chan, int *val, int *val2)
{
	struct adi_adrv910x_Carrier carrier = {};
	struct adrv9104_chan *chan;
	int ret;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Carrier_Inspect, chan->port,
				chan->number, &carrier);
	if (ret)
		return ret;

	*val = lower_32_bits(carrier.carrierFrequency_Hz);
	*val2 = upper_32_bits(carrier.carrierFrequency_Hz);

	return IIO_VAL_INT_64;
}

static int adrv9104_phy_read_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return adrv9104_phy_read_sample_rate(phy, chan, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adrv9104_hardware_gain_get(phy, chan, val, val2);
	case IIO_CHAN_INFO_FREQUENCY:
		return adrv9104_carrier_freq_get(phy, chan, val, val2);
	default:
		return -EINVAL;
	}
}

static int adrv9104_read_label(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
			       char *label)
{
	const char *port = chan->output ? "TX" : "RX";

	if (chan->type == IIO_VOLTAGE) {
		if (chan->scan_index == -1)
			return sysfs_emit(label, "%s%d\n", port, chan->channel + 1);

		if (chan->channel2 == IIO_MOD_I)
			return sysfs_emit(label, "%s%d_I_BUF\n", port, chan->channel + 1);

		return sysfs_emit(label, "%s%d_Q_BUF\n", port, chan->channel + 1);
	}

	if (chan->modified) {
		/* if altvoltage and modified we already know it's TX DDS*/
		if (chan->channel2 == IIO_MOD_I)
			return sysfs_emit(label, "TX1_DDS_I\n");

		return sysfs_emit(label, "TX1_DDS_Q\n");
	}

	return sysfs_emit(label, "%s%d_LO\n", port, chan->channel + 1);
}

static int adrv9104_phy_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	int ret;
	u8 val;

	guard(mutex)(&phy->lock);
	if (!readval)
		return adrv9104_api_call(phy, adi_adrv910x_spi_Byte_Write, reg, writeval);

	ret = adrv9104_api_call(phy, adi_adrv910x_spi_Byte_Read, reg, &val);
	if (ret)
		return ret;

	*readval = val;

	return 0;
}

enum {
	ADRV9104_EXT_TRACK_MIN,
	ADRV9104_EXT_TRACK_RX_HD2 = ADRV9104_EXT_TRACK_MIN,
	ADRV9104_EXT_TRACK_RX_QEC_WPOLY,
	ADRV9104_EXT_TRACK_RX_BBDC,
	ADRV9104_EXT_TRACK_RX_RFDC,
	ADRV9104_EXT_TRACK_RX_QEC_FIC,
	ADRV9104_EXT_TRACK_RX_GAIN_CTL_DETECT,
	ADRV9104_EXT_TRACK_RX_RSSI,
	ADRV9104_EXT_TRACK_MAX,
	ADRV9104_EXT_RSSI,
	ADRV9104_EXT_RF_BANDWIDTH,
};

static ssize_t adrv9104_rssi_get(struct adrv9104_rf_phy *phy,
				 const struct iio_chan_spec *iio_chan, char *buf)
{
	struct adrv9104_chan *chan;
	u32 rssi_mdB;
	int ret;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Rx_Rssi_Read, chan->number, &rssi_mdB);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u.%03u dB\n", rssi_mdB / 1000, rssi_mdB % 1000);
}

static ssize_t adrv9104_rf_bandwidth_get(struct adrv9104_rf_phy *phy,
					 const struct iio_chan_spec *iio_chan, char *buf)
{
	struct adrv9104_chan *chan;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	if (chan->port == ADI_RX)
		return sysfs_emit(buf, "%d\n", phy->profile.rxConfig[chan->idx].primaryBw_Hz);

	return sysfs_emit(buf, "%d\n", phy->profile.txConfig[chan->idx].primaryBw_Hz);
}

static const u32 adrv9104_track_cal_masks[ADRV9104_EXT_TRACK_MAX] = {
	[ADRV9104_EXT_TRACK_RX_HD2] = TRACKING_CAL_RX_HD2,
	[ADRV9104_EXT_TRACK_RX_QEC_WPOLY] = TRACKING_CAL_RX_QEC_WBPOLY,
	[ADRV9104_EXT_TRACK_RX_BBDC] = TRACKING_CAL_RX_BBDC,
	[ADRV9104_EXT_TRACK_RX_RFDC] = TRACKING_CAL_RX_RFDC,
	[ADRV9104_EXT_TRACK_RX_QEC_FIC] = TRACKING_CAL_RX_QEC_FIC,
	[ADRV9104_EXT_TRACK_RX_GAIN_CTL_DETECT] = TRACKING_CAL_RX_GAIN_CONTROL_DETECTORS,
	[ADRV9104_EXT_TRACK_RX_RSSI] = TRACKING_CAL_RX_RSSI,
};

static ssize_t adrv9104_track_cals_get(struct adrv9104_rf_phy *phy,
				       const struct iio_chan_spec *iio_chan, u32 mask, char *buf)
{
	trackingCals_t track_calls;
	struct adrv9104_chan *chan;
	int ret;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_cals_Tracking_Get, &track_calls);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", !!(mask & track_calls.chanTrackingCalMask[chan->idx]));
}

static ssize_t adrv9104_phy_ext_read(struct iio_dev *indio_dev, uintptr_t private,
				     const struct iio_chan_spec *iio_chan, char *buf)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	switch (private) {
	case ADRV9104_EXT_RSSI:
		return adrv9104_rssi_get(phy, iio_chan, buf);
	case ADRV9104_EXT_RF_BANDWIDTH:
		return adrv9104_rf_bandwidth_get(phy, iio_chan, buf);
	case ADRV9104_EXT_TRACK_MIN ... ADRV9104_EXT_TRACK_MAX - 1:
		return adrv9104_track_cals_get(phy, iio_chan, adrv9104_track_cal_masks[private],
					       buf);
	default:
		return -EINVAL;
	}
}

static ssize_t adrv9104_track_cals_set(struct adrv9104_rf_phy *phy,
				       const struct iio_chan_spec *iio_chan,
				       u32 mask, const char *buf, size_t len)
{
	adi_adrv910x_ChannelState_e state[ADRV9104_CHAN_MAX + 1];
	struct adrv9104_chan *chan, *__chan;
	trackingCals_t track_calls;
	bool enable;
	int ret;

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_cals_Tracking_Get, &track_calls);
	if (ret)
		return ret;

	adrv9104_for_each_enabled_chan(phy, __chan) {
		ret = adrv9104_channel_to_state_cache(phy, __chan, ADI_ADRV910X_CHANNEL_CALIBRATED,
						      &state[ADRV9104_PORT_POS(__chan)]);
		if (ret)
			return ret;
	}

	if (enable)
		track_calls.chanTrackingCalMask[chan->idx] |= mask;
	else
		track_calls.chanTrackingCalMask[chan->idx] &= ~mask;

	ret = adrv9104_api_call(phy, adi_adrv910x_cals_Tracking_Set, &track_calls);
	if (ret)
		return ret;

	adrv9104_for_each_enabled_chan(phy, __chan) {
		ret = adrv9104_channel_to_state(phy, __chan, state[ADRV9104_PORT_POS(__chan)]);
		if (ret)
			return ret;
	}

	return len;
}

static ssize_t adrv9104_phy_ext_write(struct iio_dev *indio_dev, uintptr_t private,
				      const struct iio_chan_spec *iio_chan, const char *buf,
				      size_t len)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	switch (private) {
	case ADRV9104_EXT_TRACK_MIN ... ADRV9104_EXT_TRACK_MAX - 1:
		return adrv9104_track_cals_set(phy, iio_chan, adrv9104_track_cal_masks[private],
					       buf, len);
	default:
		return -EINVAL;
	}
}

static const char *const adrv9104_port_en_mode[] = {
	"spi", "pin"
};

static int adrv9104_get_port_en_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *iio_chan)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	enum adi_adrv910x_ChannelEnableMode en_mode;
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_ChannelEnableMode_Get, chan->port,
				chan->number, &en_mode);
	return ret ?: en_mode;
}

static int adrv9104_set_port_en_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *iio_chan, u32 mode)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9104_chan *chan;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	return adrv9104_api_call(phy, adi_adrv910x_Radio_ChannelEnableMode_Set, chan->port,
				 chan->number, mode);
}

static const char * const adrv9104_ensm_modes[] = {
	"calibrated", "primed", "rf_enabled"
};

static int adrv9104_get_ensm_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *iio_chan)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	enum adi_adrv910x_ChannelState radio_state;
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Channel_State_Get, chan->port,
				chan->number, &radio_state);
	if (ret)
		return ret;

	/* ADI_ADRV910X_CHANNEL_STANDBY does not count */
	return ret ?: radio_state - 1;
}

static int adrv9104_set_ensm_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *iio_chan, u32 mode)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9104_chan *chan;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	return adrv9104_channel_to_state(phy, chan, mode + 1);
}

static const char * const adrv9104_agc_modes[] = {
	"spi", "pin", "automatic"
};

static int adrv9104_get_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *iio_chan)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	enum adi_adrv910x_RxGainControlMode agc_mode;
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Rx_GainControl_Mode_Get, chan->number,
				&agc_mode);

	return ret ?: agc_mode;
}

static int adrv9104_set_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *iio_chan, u32 mode)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9104_chan *chan;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	/* To check: This needs to be adrv9104_calibrated_api_call()? */
	return adrv9104_api_call(phy, adi_adrv910x_Rx_GainControl_Mode_Set, chan->number, mode);
}

static const char * const adrv9104_digital_gain_ctl_modes[] = {
	"automatic", "spi"
};

static int adrv9104_get_digital_gain_ctl_mode(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *iio_chan)
{
	struct adi_adrv910x_RxInterfaceGainCtrl rx_digi_gain_ctrl = {};
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv910x_RxGainTableType_e dummy;
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Rx_InterfaceGain_Inspect, chan->number,
				&rx_digi_gain_ctrl, &dummy);

	return ret ?: rx_digi_gain_ctrl.controlMode;
}

static int adrv9104_set_digital_gain_ctl_mode(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *iio_chan, u32 mode)
{
	struct adi_adrv910x_RxInterfaceGainCtrl rx_digi_gain_ctrl = {};
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv910x_RxGainTableType_e dummy;
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Rx_InterfaceGain_Inspect, chan->number,
				&rx_digi_gain_ctrl, &dummy);
	if (ret)
		return ret;

	rx_digi_gain_ctrl.controlMode = mode;

	return adrv9104_calibrated_api_call(phy, chan, adi_adrv910x_Rx_InterfaceGain_Configure,
					    chan->number, &rx_digi_gain_ctrl);
}

static const char *const adrv9104_atten_control_mode[] = {
	"bypass", "spi", "pin"
};

static int adrv9104_get_atten_control_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *iio_chan)
{
	enum adi_adrv910x_TxAttenuationControlMode atten_mode;
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9104_chan *chan;
	int ret;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = adrv9104_api_call(phy, adi_adrv910x_Tx_AttenuationMode_Get, &atten_mode);
	if (ret)
		return ret;

	if (atten_mode == ADI_ADRV910X_TX_ATTENUATION_CONTROL_MODE_PIN)
		atten_mode = 2;

	return atten_mode;
}

static int adrv9104_set_atten_control_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *iio_chan, u32 mode)
{
	enum adi_adrv910x_TxAttenuationControlMode atten_mode;
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9104_chan *chan;

	guard(mutex)(&phy->lock);

	chan = adrv9104_get_rf_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	if (mode == 2)
		atten_mode = ADI_ADRV910X_TX_ATTENUATION_CONTROL_MODE_PIN;

	return adrv9104_calibrated_api_call(phy, chan, adi_adrv910x_Tx_AttenuationMode_Set,
					    atten_mode);
}

static const struct iio_enum adrv9104_ensm_modes_available = {
	.items = adrv9104_ensm_modes,
	.num_items = ARRAY_SIZE(adrv9104_ensm_modes),
	.get = adrv9104_get_ensm_mode,
	.set = adrv9104_set_ensm_mode,
};

static const struct iio_enum adrv9104_agc_modes_available = {
	.items = adrv9104_agc_modes,
	.num_items = ARRAY_SIZE(adrv9104_agc_modes),
	.get = adrv9104_get_agc_mode,
	.set = adrv9104_set_agc_mode,
};

static const struct iio_enum adrv9104_digital_gain_ctl_modes_available = {
	.items = adrv9104_digital_gain_ctl_modes,
	.num_items = ARRAY_SIZE(adrv9104_digital_gain_ctl_modes),
	.get = adrv9104_get_digital_gain_ctl_mode,
	.set = adrv9104_set_digital_gain_ctl_mode,
};

static const struct iio_enum adrv9104_port_en_modes_available = {
	.items = adrv9104_port_en_mode,
	.num_items = ARRAY_SIZE(adrv9104_port_en_mode),
	.get = adrv9104_get_port_en_mode,
	.set = adrv9104_set_port_en_mode,
};

#define ADRV9104_EXT_INFO(_name, _private) {		\
	.name = _name,					\
	.read = adrv9104_phy_ext_read,			\
	.write = adrv9104_phy_ext_write,		\
	.private = _private,				\
}

static const struct iio_chan_spec_ext_info adrv9104_phy_rx_ext_info[] = {
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM_AVAILABLE("gain_control_mode", IIO_SEPARATE, &adrv9104_agc_modes_available),
	IIO_ENUM("gain_control_mode", IIO_SEPARATE, &adrv9104_agc_modes_available),
	IIO_ENUM_AVAILABLE("digital_gain_control_mode", IIO_SEPARATE,
			   &adrv9104_digital_gain_ctl_modes_available),
	IIO_ENUM("digital_gain_control_mode", IIO_SEPARATE,
		 &adrv9104_digital_gain_ctl_modes_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	IIO_ENUM("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	ADRV9104_EXT_INFO("rssi", ADRV9104_EXT_RSSI),
	ADRV9104_EXT_INFO("bandwidth", ADRV9104_EXT_RF_BANDWIDTH),
	ADRV9104_EXT_INFO("track_quadrature_err_wpoly_en", ADRV9104_EXT_TRACK_RX_QEC_WPOLY),
	ADRV9104_EXT_INFO("track_rf_dc_en", ADRV9104_EXT_TRACK_RX_RFDC),
	ADRV9104_EXT_INFO("track_quadrature_err_fic_en", ADRV9104_EXT_TRACK_RX_QEC_FIC),
	ADRV9104_EXT_INFO("track_gain_ctl_detect_en", ADRV9104_EXT_TRACK_RX_GAIN_CTL_DETECT),
	ADRV9104_EXT_INFO("track_rssi_en", ADRV9104_EXT_TRACK_RX_RSSI),
	{ }
};

static const struct iio_chan_spec_ext_info adrv9104_phy_rx2_ext_info[] = {
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM_AVAILABLE("gain_control_mode", IIO_SEPARATE, &adrv9104_agc_modes_available),
	IIO_ENUM("gain_control_mode", IIO_SEPARATE, &adrv9104_agc_modes_available),
	IIO_ENUM_AVAILABLE("digital_gain_control_mode", IIO_SEPARATE,
			   &adrv9104_digital_gain_ctl_modes_available),
	IIO_ENUM("digital_gain_control_mode", IIO_SEPARATE,
		 &adrv9104_digital_gain_ctl_modes_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	IIO_ENUM("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	ADRV9104_EXT_INFO("rssi", ADRV9104_EXT_RSSI),
	ADRV9104_EXT_INFO("bandwidth", ADRV9104_EXT_RF_BANDWIDTH),
	ADRV9104_EXT_INFO("track_harmonic_distortion_en", ADRV9104_EXT_TRACK_RX_HD2),
	ADRV9104_EXT_INFO("track_rf_dc_en", ADRV9104_EXT_TRACK_RX_RFDC),
	ADRV9104_EXT_INFO("track_baseband_dc_reject_en", ADRV9104_EXT_TRACK_RX_BBDC),
	ADRV9104_EXT_INFO("track_quadrature_err_fic_en", ADRV9104_EXT_TRACK_RX_QEC_FIC),
	ADRV9104_EXT_INFO("track_gain_ctl_detect_en", ADRV9104_EXT_TRACK_RX_GAIN_CTL_DETECT),
	ADRV9104_EXT_INFO("track_rssi_en", ADRV9104_EXT_TRACK_RX_RSSI),
	{ }
};

static const struct iio_enum adrv9104_atten_control_mode_available = {
	.items = adrv9104_atten_control_mode,
	.num_items = ARRAY_SIZE(adrv9104_atten_control_mode),
	.get = adrv9104_get_atten_control_mode,
	.set = adrv9104_set_atten_control_mode,
};

static const struct iio_chan_spec_ext_info adrv9104_phy_tx_ext_info[] = {
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM("ensm_mode", IIO_SEPARATE, &adrv9104_ensm_modes_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	IIO_ENUM("port_en_mode", IIO_SEPARATE, &adrv9104_port_en_modes_available),
	IIO_ENUM_AVAILABLE("atten_control_mode", IIO_SEPARATE,
			   &adrv9104_atten_control_mode_available),
	IIO_ENUM("atten_control_mode", IIO_SEPARATE,
		 &adrv9104_atten_control_mode_available),
	ADRV9104_EXT_INFO("bandwidth_hz", ADRV9104_EXT_RF_BANDWIDTH),
	{ }
};

#define ADRV9104_RX_CHAN(idx, __ext_info) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.ext_info = __ext_info,					\
	.scan_index = -1,					\
}

#define ADRV9104_RX_BUF_CHAN(idx, _si, _mod, _buf_idx) {	\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel = idx,						\
	.channel2 = _mod,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _si,					\
	.buffer_index = _buf_idx,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
	},							\
}

#define ADRV9104_TX_CHAN {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.ext_info = adrv9104_phy_tx_ext_info,			\
	.scan_index = -1,					\
}

#define ADRV9104_TX_BUF_CHAN(_si, _mod, _buf_idx) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel2 = _mod,					\
	.output = 1,						\
	.scan_index = _si,					\
	.buffer_index = _buf_idx,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
	},							\
}

#define ADRV9104_TX_DDS_CHAN(_mod) {			\
	.type = IIO_ALTVOLTAGE,				\
	.indexed = 1,					\
	.modified = 1,					\
	.channel2 = _mod,				\
	.output = 1,					\
	.scan_index = -1,				\
}

/* Speak with Michael about LO's for RX being output channels */
#define ADRV9104_LO_CHAN(idx, out) {				\
	.type = IIO_ALTVOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),	\
	.output = out,						\
	.scan_index = -1,					\
}

static struct iio_chan_spec adrv9104_phy_chan[] = {
	ADRV9104_RX_CHAN(ADRV9104_IIO_RX1, adrv9104_phy_rx_ext_info),
	ADRV9104_LO_CHAN(ADRV9104_IIO_RX1, false),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX1, ADRV9104_RX1_I_SCAN, IIO_MOD_I, 0),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX1, ADRV9104_RX1_Q_SCAN, IIO_MOD_Q, 0),
	ADRV9104_RX_CHAN(ADRV9104_IIO_RX2, adrv9104_phy_rx2_ext_info),
	ADRV9104_LO_CHAN(ADRV9104_IIO_RX2, false),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX2, ADRV9104_RX2_I_SCAN, IIO_MOD_I, 1),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX2, ADRV9104_RX2_Q_SCAN, IIO_MOD_Q, 1),
	ADRV9104_TX_CHAN,
	ADRV9104_LO_CHAN(ADRV9104_IIO_TX, true),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_I),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_Q),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_I_SCAN, IIO_MOD_I, 2),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_Q_SCAN, IIO_MOD_Q, 2),
};

static int adrv9104_load_firmware(struct adrv9104_rf_phy *phy, const char *fw_name, u32 fw_size)
{
	int ret;

	const struct firmware *fw __free(firmware) = NULL;
	ret = request_firmware(&fw, fw_name, phy->dev);
	if (ret)
		return ret;
	if (fw->size != fw_size) {
		dev_err(phy->dev, "Unexpected firmware size (%zd != %u)\n", fw->size, fw_size);
		return -EINVAL;
	}

	/* For now use the size as the choosing factor */
	if (fw_size == ADRV9104_STREAM_FW_SIZE_BYTES)
		return adrv9104_api_call(phy, adi_adrv910x_Stream_Image_Write, 0, fw->data,
					 fw->size,
					 ADI_ADRV910X_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4);

	return adrv9104_api_call(phy, adi_adrv910x_arm_Image_Write, 0, fw->data, fw->size,
				 ADI_ADRV910X_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4, ADI_PS1);
}

static int adrv9104_validate_rx_profile(struct adrv9104_rf_phy *phy, const struct adrv9104_rx *rx)
{
	const rxConfig_t *cfg = &phy->profile.rxConfig[rx->channel.idx];

	if (phy->ssi_type != cfg->rxSsiConfig.ssiType) {
		dev_err(phy->dev, "SSI type mismatch HDL=%d Profile=%d\n",
			phy->ssi_type, cfg->rxSsiConfig.ssiType);
		return -EINVAL;
	}

	if (phy->ssi_type == SSI_TYPE_LVDS && !cfg->rxSsiConfig.ddrEn) {
		dev_err(phy->dev, "RX%d: Single Data Rate port not supported for LVDS\n",
			rx->channel.idx + 1);
		return -EINVAL;
	}

	if (cfg->rxSsiConfig.strobeType == SSI_LONG_STROBE) {
		dev_err(phy->dev, "RX%d: SSI Long strobe not supported\n",
			rx->channel.idx + 1);
		return -EINVAL;
	}

	return 0;
}

static int adrv9104_validate_tx_profile(struct adrv9104_rf_phy *phy, const struct adrv9104_tx *tx)
{
	const txConfig_t *cfg = &phy->profile.txConfig[tx->channel.idx];
	struct adrv9104_chan *rx;

	if (phy->ssi_type != cfg->txSsiConfig.ssiType) {
		dev_err(phy->dev, "SSI type mismatch HDL=%d Profile=%d\n",
			phy->ssi_type, cfg->txSsiConfig.ssiType);
		return -EINVAL;
	}

	if (phy->ssi_type == SSI_TYPE_LVDS && !cfg->txSsiConfig.ddrEn) {
		dev_err(phy->dev, "TX%d: Single Data Rate port not supported for LVDS\n",
			tx->channel.idx + 1);
		return -EINVAL;
	}

	if (cfg->txSsiConfig.strobeType == SSI_LONG_STROBE) {
		dev_err(phy->dev, "TX%d: SSI Long strobe not supported\n",
			tx->channel.idx + 1);
		return -EINVAL;
	}

	if (!tx->tx_ref_clock)
		return 0;

	/* Alright, RX clock is driving us... */
	rx = &phy->rx_channels[tx->tx_ref_clock - 1].channel;
	if (!rx->enabled) {
		dev_err(phy->dev, "TX clock driven by RX%d and it is disabled\n",
			rx->idx + 1);
		return -EINVAL;
	}

	if (cfg->txInputRate != rx->rate) {
		dev_err(phy->dev, "TX clock driven by RX%d and rate mismatch RX=%u TX=%u\n",
			rx->idx + 1, rx->rate, cfg->txInputRate);
		return -EINVAL;
	}

	return 0;
}

static int adrv9104_validate_profile(struct adrv9104_rf_phy *phy)
{
	unsigned long chan_mask = phy->profile.chanConfig;
	struct adrv9104_tx *tx = &phy->tx_channel;
	struct adrv9104_rx *rx;
	int ret;

	adrv9104_for_each_rx(phy, rx) {
		if (!test_bit(ADRV9104_RX_BIT_START + rx->channel.idx, &chan_mask))
			continue;

		ret = adrv9104_validate_rx_profile(phy, rx);
		if (ret)
			return ret;

		rx->channel.enabled = true;
		rx->channel.rate = phy->profile.rxConfig[rx->channel.idx].rxOutputRate;
		rx->channel.lo = phy->profile.rxConfig[rx->channel.idx].LoSelect;

		dev_dbg(phy->dev, "RX%d enabled with rate %u\n", rx->channel.idx + 1,
			rx->channel.rate);
	}

	if (!test_bit(ADRV9104_TX_BIT, &chan_mask) && !test_bit(ADRV9104_TXNB_BIT, &chan_mask))
		return 0;

	/* Let's just set the txnb flag as that matters for validating the TX profile */
	tx->txnb = test_bit(ADRV9104_TXNB_BIT, &chan_mask);
	if (tx->txnb) {
		/*
		 * Then let's pretend we have idx 1 for TX as that seems the logic the driver API
		 * follows.
		 */
		tx->channel.idx = 1;
		tx->channel.number = ADI_CHANNEL_2;
	} else {
		tx->channel.idx = 0;
		tx->channel.number = ADI_CHANNEL_1;
	}

	ret = adrv9104_validate_tx_profile(phy, tx);
	if (ret)
		return ret;

	tx->channel.enabled = true;
	tx->channel.rate = phy->profile.txConfig[tx->channel.idx].txInputRate;
	tx->channel.lo = phy->profile.txConfig[tx->channel.idx].LoSelect;

	dev_dbg(phy->dev, "TX (narrowband=%d) enabled with rate %u\n", tx->txnb, tx->channel.rate);

	return 0;
}

static int adrv9104_digital_init(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_rx *rx;
	int ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_AhbSpiBridge_Enable);
	if (ret)
		return ret;

	ret = adrv9104_load_firmware(phy, "adrv9104_stream.bin", ADRV9104_STREAM_FW_SIZE_BYTES);
	if (ret)
		return ret;

	ret = adrv9104_load_firmware(phy, "adrv9104_arm_fw.bin", ADRV9104_ARM_FW_SIZE_BYTES);
	if (ret)
		return ret;

	/*
	 * Make sure the two sizes are identical given that deviceProfileBundle_t is defined with
	 * ADI_NEVIS_PACK_START and ADI_NEVIS_PACK_END which should pack the structure. And the
	 * sizes better match so that I can do the cast below.
	 */
	static_assert(sizeof_field(deviceProfileBundle_t, profile) ==
		      sizeof_field(struct adi_adrv910x_Profiles, profilePS1));

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_Profile_Write,
				(adi_adrv910x_Profiles_t *)&phy->profile, ADI_PS1);
	if (ret)
		return ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_PfirProfiles_Write, &phy->pfirBuffer);
	if (ret)
		return ret;

	adrv9104_for_each_enabled_rx(phy, rx) {
		rxConfig_t *rx_cfg = &phy->profile.rxConfig[rx->channel.idx];
		const adi_adrv910x_RxGainTableRow_t *table;
		adi_adrv910x_RxGainTableType_e table_type;
		u32 table_size;

		/*
		 * Most likely correction and compensated tables will always have the same size.
		 * Nevertheless better not to assume that!
		 */
		if (rx_cfg->gainTableType == RX_GAIN_CORRECTION_TABLE) {
			table = adrv9104_rx_gain_table;
			table_size = ARRAY_SIZE(adrv9104_rx_gain_table);
			table_type = ADI_ADRV910X_RX_GAIN_CORRECTION_TABLE;
		} else {
			table = adrv9104_rx_gain_table_gain_compensated;
			table_size = ARRAY_SIZE(adrv9104_rx_gain_table_gain_compensated);
			table_type = ADI_ADRV910X_RX_GAIN_COMPENSATION_TABLE;
		}

		ret = adrv9104_api_call(phy, adi_adrv910x_Rx_GainTable_Write, ADI_RX,
					rx->channel.number, ADRV9104_RX_GAIN_TABLE_MAX_GAIN_INDEX,
					table, table_size, table_type);
		if (ret)
			return ret;
	}

	if (phy->tx_channel.channel.enabled) {
		ret = adrv9104_api_call(phy, adi_adrv910x_Tx_AttenuationTable_Write,
					ADI_ADRV910X_TX1, ADRV9104_TX_ATTEN_TABLE_MIN_INDEX,
					adrv9104_tx_atten_table,
					ARRAY_SIZE(adrv9104_tx_atten_table));
		if (ret)
			return ret;
	}

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_Start, ADI_PS1);
	if (ret)
		return ret;

	return adrv9104_api_call(phy, adi_adrv910x_arm_StartStatus_Check, 5000000);
}

static void adrv9104_get_init_carrier(const struct adrv9104_chan *chan, u64 *carrier_hz)
{
	if (chan->carrier_hz)
		*carrier_hz = chan->carrier_hz;
	else
		*carrier_hz = chan->port == ADI_TX ? 2450 * MEGA : 2400 * MEGA;
}

static int adrv9104_radio_init(struct adrv9104_rf_phy *phy)
{
	struct adi_adrv910x_PllLoopFilterCfg pll_loop_filter = {
		.effectiveLoopBandwidth_kHz = 0,
		.loopBandwidth_kHz = 300,
		.phaseMargin_degrees = 60,
		.powerScale = 5
	};
	struct adrv9104_chan *chan;
	u64 carrier_hz;
	int ret;

	adrv9104_for_each_enabled_chan(phy, chan) {
		adrv9104_get_init_carrier(chan, &carrier_hz);
		ret = adrv9104_carrier_set(phy, chan, carrier_hz, NULL);
		if (ret)
			return ret;
	}

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_PllLoopFilter_Set,
				ADI_ADRV910X_PLL_LO1, &pll_loop_filter);
	if (ret)
		return ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_PllLoopFilter_Set,
				ADI_ADRV910X_PLL_LO2, &pll_loop_filter);
	if (ret)
		return ret;

	return adrv9104_api_call(phy, adi_adrv910x_arm_System_Program, phy->profile.chanConfig);
}

static int adrv9104_tx_set_dac_full_scale(struct adrv9104_rf_phy *phy)
{
	const struct adrv9104_tx *tx = &phy->tx_channel;

	if (!tx->channel.enabled || !tx->dac_boost_en)
		return 0;

	return adrv9104_api_call(phy, adi_adrv910x_Tx_OutputPowerBoost_Set, true);
}

static const u32 adrv9002_init_cals_mask[16][2] = {
	/* Not a valid case. At least one channel should be enabled */
	[0] = { 0, 0 },
	/* txnb:0 rx2:0 tx1:0 rx1:1 */
	[1] = { 0x1bc400, 0 },
	/* txnb:0 rx2:0 tx1:1 rx1:0 */
	[2] = { 0x1bc5f7, 0 },
	/* txnb:0 rx2:0 tx1:1 rx1:1 */
	[3] = { 0x1bc5f7, 0 },
	/* txnb:0 rx2:1 tx1:0 rx1:0 */
	[4] = { 0, 0x1ac400 },
	/* txnb:0 rx2:1 tx1:0 rx1:1 */
	[5] = { 0x1bc400, 0x1ac400 },
	/* txnb:0 rx2:1 tx1:1 rx1:0 */
	[6] = { 0x1bc5f7, 0x1ac400 },
	/* txnb:0 rx2:1 tx1:1 rx1:1 */
	[7] = { 0x1bc5f7, 0x1ac400 },
	/* txnb:1 rx2:0 tx1:0 rx1:0 */
	[8] = { 0, 0x1ac5f7 },
	/* txnb:1 rx2:0 tx1:0 rx1:1 */
	[9] = { 0x1bc400, 0x1ac5f7 },
	/* txnb:1 rx2:0 tx1:1 rx1:0 */
	[10] = { 0x1bc5f7, 0x1ac5f7  },
	/* txnb:1 rx2:0 tx1:1 rx1:1 */
	[11] = { 0x1bc5f7, 0x1ac5f7  },
	/* txnb:1 rx2:1 tx1:0 rx1:0 */
	[12] = { 0, 0x1ac5f7 },
	/* txnb:1 rx2:1 tx1:0 rx1:1 */
	[13] = { 0x1bc400, 0x1ac5f7 },
	/* txnb:1 rx2:1 tx1:1 rx1:0 */
	[14] = { 0x1bc5f7, 0x1ac5f7  },
	/* txnb:1 rx2:1 tx1:1 rx1:1 */
	[15] = { 0x1bc5f7, 0x1ac5f7  },
};

static int adrv9104_run_init_cals(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_chan *chan;
	initCals_t init_cals = {};
	u32 pos = 0;
	u8 errors;

	/*
	 * For the below to work we need to evaluate the possibility of putting txnb as channel
	 * index 1.
	 */
	adrv9104_for_each_enabled_chan(phy, chan)
		pos |= BIT(chan->idx * 2 + chan->port);

	init_cals.chanInitCalMask[0] = adrv9002_init_cals_mask[pos][0];
	init_cals.chanInitCalMask[1] = adrv9002_init_cals_mask[pos][1];

	return adrv9104_api_call(phy, adi_adrv910x_cals_InitCals_Run, &init_cals, 60000, &errors);
}

static int adrv9104_rx_path_config(struct adrv9104_rf_phy *phy,
				   const adi_adrv910x_ChannelState_e state)
{
	struct adrv9104_rx *rx;
	int ret;

	adrv9104_for_each_enabled_rx(phy, rx) {
		ret = adrv9104_api_call(phy, adi_adrv910x_Rx_GainControl_Configure,
					rx->channel.number, &rx->agc);
		if (ret)
			return ret;

		ret = adrv9104_channel_to_state(phy, &rx->channel, state);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9104_rf_init(struct adrv9104_rf_phy *phy)
{
	struct adi_adrv910x_SpiSettings adrv9104_spi = {
		.msbFirst = 1,
		.autoIncAddrUp = 1,
		.fourWireMode = 1,
		.cmosPadDrvStrength = ADI_ADRV910X_CMOSPAD_DRV_STRONG,
	};
	adi_adrv910x_ChannelState_e init_state;
	int ret;

	adi_common_ErrorClear(&phy->phy_dev.common);

	ret = adrv9104_api_call(phy, adi_adrv910x_HwOpen, &adrv9104_spi);
	if (ret)
		return ret;

	ret = adrv9104_validate_profile(phy);
	if (ret)
		return ret;

	adrv9104_log_enable(&phy->phy_dev.common);

	ret = adrv9104_api_call(phy, adi_adrv910x_InitAnalog, &phy->profile,
				ADI_ADRV910X_DEVICECLOCKDIVISOR_BYPASS);
	if (ret)
		return ret;

	ret = adrv9104_digital_init(phy);
	if (ret)
		return ret;

	ret = adrv9104_radio_init(phy);
	if (ret)
		return ret;

	/* Must be called before init cals */
	ret = adrv9104_tx_set_dac_full_scale(phy);
	if (ret)
		return ret;

	ret = adrv9104_run_init_cals(phy);
	if (ret)
		return ret;

	/* in TDD we cannot start with all ports enabled as RX/TX cannot be on at the same time */
	if (phy->profile.sysConfig.duplexMode == TDD)
		init_state = ADI_ADRV910X_CHANNEL_PRIMED;
	else
		init_state = ADI_ADRV910X_CHANNEL_RF_ENABLED;

	ret = adrv9104_rx_path_config(phy, init_state);
	if (ret)
		return ret;

	return adrv9104_channel_to_state(phy, &phy->tx_channel.channel, init_state);
}

static void adrv9104_cleanup(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_chan *chan;

	adrv9104_for_each_chan(phy, chan) {
		chan->enabled = false;
		chan->lo = MAX_NUM_LOS;
	}

	memset(&phy->phy_dev.devStateInfo, 0, sizeof(phy->phy_dev.devStateInfo));
}

static int __adrv9104_init(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_chan *chan;
	int ret;

	adrv9104_cleanup(phy);

	/* Disable all the cores as it might interfere with init calibrations */
	adrv9104_for_each_chan(phy, chan)
		adrv9104_backend_disable(chan->back);

	ret = adrv9104_rf_init(phy);
	if (ret)
		goto err;

	adrv9104_for_each_enabled_chan(phy, chan) {
		ret = adrv9104_backend_setup(phy, chan);
		if (ret)
			return ret;

		ret = adrv9104_backend_enable(chan->back);
		if (ret)
			return ret;
	}

	return 0;

err:
	/*
	 * Leave the device in a reset state in case of error. There's not much we can do if
	 * the API call fails, so we are just being verbose about it...
	 */
	adrv9104_api_call(phy, adi_adrv910x_HwReset);
	adrv9104_cleanup(phy);
	return ret;
}

int adr9104_init(struct adrv9104_rf_phy *phy)
{
	guard(mutex)(&phy->lock);
	return __adrv9104_init(phy);
}

static const struct iio_info adrv9104_phy_info = {
	.read_raw = &adrv9104_phy_read_raw,
	.write_raw = &adrv9104_phy_write_raw,
	.read_label = &adrv9104_read_label,
	.get_iio_backend = &adrv9104_backend_get_from_chan,
	.write_raw_get_fmt = &adrv9104_write_raw_get_fmt,
	.debugfs_reg_access = &adrv9104_phy_reg_access,
};

static int adrv9104_probe(struct spi_device *spi)
{
	struct adrv9104_rf_phy *phy;
	struct iio_dev *indio_dev;
	struct clk *dev_clk;
	int ret, c;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->dev = &spi->dev;
	phy->phy_dev.common.devHalInfo = &phy->hal;
	phy->hal.spi = spi;

	phy->hal.reset_gpio = devm_gpiod_get_optional(phy->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->hal.reset_gpio))
		return PTR_ERR(phy->hal.reset_gpio);

	phy->iio_chans = devm_kmemdup(phy->dev, adrv9104_phy_chan, sizeof(adrv9104_phy_chan),
				      GFP_KERNEL);
	if (!phy->iio_chans)
		return -ENOMEM;

	indio_dev->name = "adrv9104";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adrv9104_phy_info;
	indio_dev->channels = phy->iio_chans;
	indio_dev->num_channels = ARRAY_SIZE(adrv9104_phy_chan);

	for (c = 0; c < ARRAY_SIZE(phy->rx_channels); c++) {
		phy->channels[c] = &phy->rx_channels[c].channel;
		phy->rx_channels[c].channel.idx = c;
		phy->rx_channels[c].channel.port = ADI_RX;
		phy->rx_channels[c].channel.number = c + ADI_CHANNEL_1;

		ret = adrv9104_backend_get(phy, &phy->rx_channels[c].channel, indio_dev, NULL);
		if (ret)
			return ret;
	}

	/*
	 * Index and number are not set as those depend on whether tx or txnb is enabled. It looks
	 * like the API treats txnb as ADI_CHANNEL_2.
	 */
	phy->channels[ADRV9104_TX] = &phy->tx_channel.channel;
	phy->tx_channel.channel.port = ADI_TX;

	ret = adrv9104_backend_get(phy, &phy->tx_channel.channel, indio_dev, &phy->iio_chans[10]);
	if (ret)
		return ret;

	ret = adrv9104_backend_get_ssi_type(phy);
	if (ret)
		return ret;

	ret = adrv9104_backend_get_tx_ref_clock(phy);
	if (ret)
		return ret;

	dev_clk = devm_clk_get_enabled(phy->dev, NULL);
	if (IS_ERR(dev_clk))
		return PTR_ERR(dev_clk);

	adrv9104_profile_copy_default(phy);

	ret = adrv9104_fw_parse(phy);
	if (ret)
		return ret;

	ret = __adrv9104_init(phy);
	if (ret)
		return ret;

	ret = devm_mutex_init(phy->dev, &phy->lock);
	if (ret)
		return ret;

	ret = devm_iio_device_register(phy->dev, indio_dev);
	if (ret)
		return ret;

	adrv9104_debugfs_create(phy, indio_dev);

	return 0;
}

static const struct of_device_id adrv9104_of_match[] = {
	{ .compatible = "adi,adrv9104" },
	{ }
};
MODULE_DEVICE_TABLE(of, adrv9104_of_match);

static const struct spi_device_id adrv9104_ids[] = {
	{ "adrv9104" },
	{ }
};
MODULE_DEVICE_TABLE(spi, adrv9104_ids);

static struct spi_driver adrv9104_driver = {
	.driver = {
		.name	= "adrv9104",
		.of_match_table = adrv9104_of_match,
	},
	.probe		= adrv9104_probe,
	.id_table	= adrv9104_ids,
};
module_spi_driver(adrv9104_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9104 and similar RF Transceivers Driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_BACKEND);
