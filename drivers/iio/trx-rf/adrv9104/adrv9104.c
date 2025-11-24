// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 and similar RF Transceiver
 *
 * Copyright 2025 Analog Devices Inc.
 */
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
#include <linux/gpio/consumer.h>
#include <linux/lockdep.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>
#include <linux/units.h>
#include <linux/wordpart.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>

#include "adi_adrv910x_arm.h"
#include "adi_adrv910x_cals.h"
#include "adi_adrv910x_radio.h"
#include "adi_adrv910x_rx.h"
#include "adi_adrv910x_spi.h"
#include "adi_adrv910x_stream.h"
#include "adi_adrv910x_tx.h"

#include "adi_common_error.h"

#include "adrv9104.h"
#include "adrv9104-backend.h"
#include "adrv9104-profile.h"
#include "adrv9104-rx-gain-table.h"
#include "adrv9104-tx-atten-table.h"

#include "device_profile_rx_dp_t.h"

#define ADRV9104_ARM_FW_SIZE_BYTES	393216
#define ADRV9104_STREAM_FW_SIZE_BYTES	32768
#define ADRV9104_RX_GAIN_STEP_mDB	500
#define ADRV9104_RX_MAX_GAIN_mdB	\
	((ADI_ADRV910X_RX_GAIN_INDEX_MAX - ADI_ADRV910X_RX_GAIN_INDEX_MIN) *	\
	 ADRV9104_RX_GAIN_STEP_mDB)

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

static struct adrv9104_chan *adrv9104_get_chan_from_iio(struct adrv9104_rf_phy *phy,
							const struct iio_chan_spec *chan_spec)
{
	if (chan_spec->output)
		return &phy->tx_channel.channel;

	if (chan_spec->channel < ADRV9104_IIO_RX_MAX)
		return &phy->rx_channels[chan_spec->channel].channel;

	return ERR_PTR(-ENOENT);
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

int adrv9104_channel_to_state_cache(struct adrv9104_rf_phy *phy, const struct adrv9104_chan *chan,
				    adi_adrv910x_ChannelState_e state,
				    adi_adrv910x_ChannelState_e *curr)
{
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
				u64 freq_hz, bool on_init)
{
	struct adi_adrv910x_Carrier carrier = {};
	int ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_Radio_Carrier_Inspect, chan->port,
				chan->number, &carrier);
	if (ret)
		return ret;

	carrier.carrierFrequency_Hz = freq_hz;

	if (on_init)
		return adrv9104_api_call(phy, adi_adrv910x_Radio_Carrier_Configure,
					    chan->port, chan->number, &carrier);

	return adrv9104_calibrated_api_call(phy, chan, adi_adrv910x_Radio_Carrier_Configure,
					    chan->port, chan->number, &carrier);
}

static int adrv9104_hardware_gain_set(struct adrv9104_rf_phy *phy,
				      const struct iio_chan_spec *iio_chan, int val, int val2)
{
	struct adrv9104_chan *chan;
	int ret, gain;
	u8 idx;

	chan = adrv9104_get_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	guard(mutex)(&phy->lock);
	if (!chan->enabled)
		return -ENODEV;

	/* Make sure this actually works! For now it's just a copy from adrv9002 */
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

static int adrv9104_carrier_freq_set(struct adrv9104_rf_phy *phy,
				     const struct iio_chan_spec *iio_chan, int val, int val2)
{
	struct adrv9104_chan *chan;
	u64 carrier_freq_hz;

	if (val < 0 || val2 < 0)
		return -EINVAL;

	chan = adrv9104_get_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	guard(mutex)(&phy->lock);
	if (!chan->enabled)
		return -ENODEV;

	/*
	 * Keeping it simple for now but this needs more handling with respect to which LO is
	 * driving a specific channel. For instance, both RX1 and RX2 can be driven by LO1 and so
	 * they need to be updated together.
	 */
	carrier_freq_hz = ((u64)val2 << 32) | val;

	return adrv9104_carrier_set(phy, chan, carrier_freq_hz, false);
}

static int adrv9104_phy_write_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

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
	u8 index;
	int ret;

	chan = adrv9104_get_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	guard(mutex)(&phy->lock);
	if (!chan->enabled)
		return -ENODEV;

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
	*val = (index - ADI_ADRV910X_RX_GAIN_INDEX_MIN) * ADRV9104_RX_GAIN_STEP_mDB;

	*val = *val / 1000;
	*val2 = (*val % 1000) * 1000;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int adrv9104_phy_read_sample_rate(struct adrv9104_rf_phy *phy,
					 const struct iio_chan_spec *iio_chan, int *val)
{
	struct adrv9104_chan *chan = adrv9104_get_chan_from_iio(phy, iio_chan);

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

	chan = adrv9104_get_chan_from_iio(phy, iio_chan);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	guard(mutex)(&phy->lock);
	if (!chan->enabled)
		return -ENODEV;

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

/* Do I need scan_index = -1? Navassa does not have it! */
#define ADRV9104_RX_CHAN(idx) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_ENABLE) |		\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.scan_index = -1,					\
}

#define ADRV9104_RX_BUF_CHAN(idx, _si, _mod) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel = idx,						\
	.channel2 = _mod,					\
	.scan_index = _si,					\
	.scan_type = {						\
		.sign = 'S',					\
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
	.scan_index = -1,					\
}

#define ADRV9104_TX_BUF_CHAN(_si, _mod) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.modified = 1,						\
	.channel2 = _mod,					\
	.output = 1,						\
	.scan_index = _si,					\
	.scan_type = {						\
		.sign = 'S',					\
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
	ADRV9104_RX_CHAN(ADRV9104_IIO_RX1),
	ADRV9104_LO_CHAN(ADRV9104_IIO_RX1, false),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX1, ADRV9104_RX1_I_SCAN, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX1, ADRV9104_RX1_Q_SCAN, IIO_MOD_Q),
	ADRV9104_RX_CHAN(ADRV9104_IIO_RX2),
	ADRV9104_LO_CHAN(ADRV9104_IIO_RX2, false),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX2, ADRV9104_RX2_I_SCAN, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(ADRV9104_IIO_RX2, ADRV9104_RX2_Q_SCAN, IIO_MOD_Q),
	ADRV9104_TX_CHAN,
	ADRV9104_LO_CHAN(ADRV9104_IIO_TX, true),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_I),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_Q),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_I_SCAN, IIO_MOD_I),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_Q_SCAN, IIO_MOD_Q),
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

static int adrv9104_radio_init(struct adrv9104_rf_phy *phy)
{
	struct adi_adrv910x_PllLoopFilterCfg pll_loop_filter = {
		.effectiveLoopBandwidth_kHz = 0,
		.loopBandwidth_kHz = 300,
		.phaseMargin_degrees = 60,
		.powerScale = 5
	};
	struct adrv9104_chan *chan;
	int ret;

	adrv9104_for_each_enabled_chan(phy, chan) {
		ret = adrv9104_carrier_set(phy, chan,
					   chan->port == ADI_TX ? 2450 * MEGA : 2400 * MEGA, true);
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

/* !TODO: Validate MASKS!!!! */
static const u32 adrv9002_init_cals_mask[16][2] = {
	/* Not a valid case. At least one channel should be enabled */
	[0] = { 0, 0 },
	/* txnb:0 rxnb:0 tx1:0 rx1:1 */
	[1] = { 0x1BE400, 0 },
	/* txnb:0 rxnb:0 tx1:1 rx1:0 */
	[2] = { 0x1BE5F7, 0 },
	/* txnb:0 rxnb:0 tx1:1 rx1:1 */
	[3] = { 0x1BE5F7, 0 },
	/* txnb:0 rxnb:1 tx1:0 rx1:0 */
	[4] = { 0, 0x11E400 },
	/* txnb:0 rxnb:1 tx1:0 rx1:1 */
	[5] = { 0x1BE400, 0x1BE400 },
	/* txnb:0 rxnb:1 tx1:1 rx1:0 */
	[6] = { 0x1BE5F7, 0x1BE400 },
	/* txnb:0 rxnb:1 tx1:1 rx1:1 */
	[7] = { 0x1BE5F7, 0x1BE400 },
	/* txnb:1 rxnb:0 tx1:0 rx1:0 */
	[8] = { 0, 0x11E5F0 },
	/* txnb:1 rxnb:0 tx1:0 rx1:1 */
	[9] = { 0x1BE400, 0x1BE5F0 },
	/* txnb:1 rxnb:0 tx1:1 rx1:0 */
	[10] = { 0x1BE5F7, 0x1BE5F7 },
	/* txnb:1 rxnb:0 tx1:1 rx1:1 */
	[11] = { 0x1BE5F7, 0x1BE5F7 },
	/* txnb:1 rxnb:1 tx1:0 rx1:0 */
	[12] = { 0, 0x11E5F0 },
	/* txnb:1 rxnb:1 tx1:0 rx1:1 */
	[13] = { 0x1BE400, 0x1BE5F0 },
	/* txnb:1 rxnb:1 tx1:1 rx1:0 */
	[14] = { 0x1BE5F7, 0x1BE5F7 },
	/* txnb:1 rxnb:1 tx1:1 rx1:1 */
	[15] = { 0x1BE5F7, 0x1BE5F7 },
};

static int adrv9104_run_init_cals(struct adrv9104_rf_phy *phy)
{
	initCals_t init_cals = {};
	struct adrv9104_rx *rx;
	u32 pos = 0;
	u8 errors;

	adrv9104_for_each_enabled_rx(phy, rx)
		pos |= BIT(rx->channel.idx * 2 + 1);

	if (phy->tx_channel.channel.enabled) {
		if (phy->tx_channel.txnb)
			pos |= BIT(3);
		else
			pos |= BIT(1);
	}

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

static struct adi_adrv910x_SpiSettings adrv9104_spi = {
	.msbFirst = 1,
	.enSpiStreaming = 0,
	.autoIncAddrUp = 1,
	.fourWireMode = 1,
	.cmosPadDrvStrength = ADI_ADRV910X_CMOSPAD_DRV_STRONG,
};

static int adr9104_rf_init(struct adrv9104_rf_phy *phy)
{
	adi_adrv910x_ChannelState_e init_state;
	int ret;

	adi_common_ErrorClear(&phy->phy_dev.common);

	ret = adrv9104_api_call(phy, adi_adrv910x_HwOpen, &adrv9104_spi);
	if (ret)
		return ret;

	/* !\TODO: validate profile */
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

	/* must be called before init cals */
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

static int adr9104_init(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_chan *chan;
	int ret;

	/* Disable all the cores as it might interfere with init calibrations */
	adrv9104_for_each_chan(phy, chan)
		adrv9104_backend_disable(chan->back);

	ret = adr9104_rf_init(phy);
	if (ret)
		return ret;

	/*
	 * \TODO:
	 * - Set the base rate for the DDS
	 * - Configure SSI interface
	 * - Properly unwind in case of errors
	 * - Only enable the backend if the channel is enabled
	 * - Interface tuning (maybe in a later point in time)
	 */

	adrv9104_for_each_enabled_chan(phy, chan) {
		ret = adrv9104_backend_init(phy, chan);
		if (ret)
			return ret;

		ret = adrv9104_backend_enable(chan->back);
		if (ret)
			return ret;
	}
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
	struct adrv9104_chan *chan;
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

	phy->hal.reset_gpio = devm_gpiod_get_optional(&spi->dev,
						      "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->hal.reset_gpio))
		return PTR_ERR(phy->hal.reset_gpio);

	indio_dev->name = "adrv9104";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adrv9104_phy_info;
	indio_dev->channels = adrv9104_phy_chan;
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

	phy->channels[ADRV9104_TX] = &phy->tx_channel.channel;
	phy->tx_channel.channel.port = ADI_TX;

	ret = adrv9104_backend_get(phy, &phy->tx_channel.channel, indio_dev,
				   &adrv9104_phy_chan[10]);
	if (ret)
		return ret;

	ret = devm_mutex_init(phy->dev, &phy->lock);
	if (ret)
		return ret;

	dev_clk = devm_clk_get_enabled(phy->dev, NULL);
	if (IS_ERR(dev_clk))
		return PTR_ERR(dev_clk);

	adrv9104_profile_copy_default(phy);

	ret = adr9104_init(phy);
	if (ret)
		return ret;

	ret = devm_iio_device_register(phy->dev, indio_dev);
	if (ret)
		return ret;

	adrv9104_for_each_chan(phy, chan)
		iio_backend_debugfs_add(chan->back, indio_dev);

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
