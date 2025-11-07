// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 and similar RF Transceiver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/array_size.h>
#include <linux/bitmap.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/stddef.h>
#include <linux/sysfs.h>

#include <linux/iio/backend.h>
#include <linux/iio/iio.h>

#include "adi_adrv910x_arm.h"
#include "adi_adrv910x_common_types.h"
#include "adi_adrv910x_rx.h"
#include "adi_adrv910x_spi.h"
#include "adi_adrv910x_types.h"
#include "adi_adrv910x_tx.h"

#include "adi_common_error.h"

#include "adrv9104.h"
#include "adrv9104-rx-gain-table.h"
#include "adrv9104-tx-atten-table.h"

#include "device_profile_rx_dp_t.h"

#define ADRV9104_ARM_FW_SIZE_BYTES	393216

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
		return -EFAULT;
	case ADI_COMMON_ERR_SPI_FAIL:
		return -EIO;
	case ADI_COMMON_ERR_MEM_ALLOC_FAIL:
		return -ENOMEM;
	default:
		return -EFAULT;
	}
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

static int adrv9104_phy_write_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	return 0;
}

static int adrv9104_phy_read_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	return 0;
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

enum {
	ADRV9104_RX1_I_SCAN,
	ADRV9104_RX1_Q_SCAN,
	ADRV9104_RX2_I_SCAN,
	ADRV9104_RX2_Q_SCAN,
	ADRV9104_TX_I_SCAN,
	ADRV9104_TX_Q_SCAN,
	ADRV9104_MAX_SCAN
};

static const unsigned long adrv9104_valid_scan_masks[ADRV9104_CHAN_MAX] = {
	GENMASK(ADRV9104_RX1_Q_SCAN, ADRV9104_RX1_I_SCAN),
	GENMASK(ADRV9104_RX2_Q_SCAN, ADRV9104_RX2_I_SCAN),
	GENMASK(ADRV9104_TX_Q_SCAN, ADRV9104_TX_I_SCAN),
};

static int adrv9104_backend_channel_set(struct iio_dev *indio_dev,
					const struct adrv9104_chan *chan,
					const unsigned long buf_mask, bool enable)
{
	unsigned int bit;
	int ret;

	iio_for_each_active_channel(indio_dev, bit) {
		if (!test_bit(bit, &buf_mask))
			continue;

		/* Even bits are for I channels which is index 0. Odd bits are for Q channels which
		 * is index 1. Take care that if we do add support for rx2tx this will not hold
		 * true!
		 */
		if (enable) {
			if (chan->port == ADI_RX) {
				ret = iio_backend_chan_enable(chan->back, bit % 2);
				if (ret)
					return ret;
			} else {
				ret = iio_backend_data_source_set(chan->back, bit % 2,
								  IIO_BACKEND_EXTERNAL);
				if (ret)
					return ret;
			}

			continue;
		}

		if (chan->port == ADI_RX) {
			ret = iio_backend_chan_disable(chan->back, bit % 2);
			if (ret)
				return ret;

			continue;
		}

		ret = iio_backend_data_source_set(chan->back, bit % 2,
						  IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9104_tx_preenable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->tx_channel.channel,
					    adrv9104_valid_scan_masks[ADRV9104_TX], true);
}

static int adrv9104_tx_postdisable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->tx_channel.channel,
					    adrv9104_valid_scan_masks[ADRV9104_TX], false);
}

static int adrv9104_rx1_preenable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->rx_channels[ADRV9104_RX1].channel,
					    adrv9104_valid_scan_masks[ADRV9104_RX1], true);
}

static int adrv9104_rx1_postdisable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->rx_channels[ADRV9104_RX1].channel,
					    adrv9104_valid_scan_masks[ADRV9104_RX1], false);
}

static int adrv9104_rx2_preenable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->rx_channels[ADRV9104_RX2].channel,
					    adrv9104_valid_scan_masks[ADRV9104_RX2], true);
}

static int adrv9104_rx2_postdisable(struct iio_dev *indio_dev)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	return adrv9104_backend_channel_set(indio_dev, &phy->rx_channels[ADRV9104_RX2].channel,
					    adrv9104_valid_scan_masks[ADRV9104_RX2], false);
}

static bool adrv9104_validate_scan_mask(const unsigned long *scan_mask,
					const unsigned long *allowed_mask)
{
	if (bitmap_weight_andnot(scan_mask, allowed_mask, ADRV9104_MAX_SCAN))
		return false;

	return true;
}

static bool adrv9104_tx_validate_scan_mask(struct iio_dev *indio_dev,
					   const unsigned long *scan_mask)
{
	return adrv9104_validate_scan_mask(scan_mask, &adrv9104_valid_scan_masks[ADRV9104_TX]);
}

static bool adrv9104_rx1_validate_scan_mask(struct iio_dev *indio_dev,
					    const unsigned long *scan_mask)
{
	return adrv9104_validate_scan_mask(scan_mask, &adrv9104_valid_scan_masks[ADRV9104_RX1]);
}

static bool adrv9104_rx2_validate_scan_mask(struct iio_dev *indio_dev,
					    const unsigned long *scan_mask)
{
	return adrv9104_validate_scan_mask(scan_mask, &adrv9104_valid_scan_masks[ADRV9104_RX2]);
}

static struct iio_backend *adrv9104_get_backend(struct iio_dev *indio_dev, uintptr_t private,
						const struct iio_chan_spec *chan)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	/* Only get's called for the TX DDS channel extension */
	return phy->tx_channel.channel.back;
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
			BIT(IIO_CHAN_INFO_ENABLE) |		\
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
	ADRV9104_RX_CHAN(0),
	ADRV9104_LO_CHAN(0, false),
	ADRV9104_RX_BUF_CHAN(0, ADRV9104_RX1_I_SCAN, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(0, ADRV9104_RX1_Q_SCAN, IIO_MOD_Q),
	ADRV9104_RX_CHAN(1),
	ADRV9104_LO_CHAN(1, false),
	ADRV9104_RX_BUF_CHAN(1, ADRV9104_RX2_I_SCAN, IIO_MOD_I),
	ADRV9104_RX_BUF_CHAN(1, ADRV9104_RX2_Q_SCAN, IIO_MOD_Q),
	ADRV9104_TX_CHAN,
	ADRV9104_LO_CHAN(0, true),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_I),
	ADRV9104_TX_DDS_CHAN(IIO_MOD_Q),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_I_SCAN, IIO_MOD_I),
	ADRV9104_TX_BUF_CHAN(ADRV9104_TX_Q_SCAN, IIO_MOD_Q),
};

static const char * const adrv9104_rx_backend_names[] = {
	"rx1-backend",
	"rx2-backend",
};

static const struct iio_buffer_setup_ops adrv9104_tx_buffer_ops = {
	.preenable = adrv9104_tx_preenable,
	.postdisable = adrv9104_tx_postdisable,
	.validate_scan_mask = adrv9104_tx_validate_scan_mask,
};

static const struct iio_buffer_setup_ops adrv9104_rx_buffer_ops[] = {
	{
		.preenable = adrv9104_rx1_preenable,
		.postdisable = adrv9104_rx1_postdisable,
		.validate_scan_mask = adrv9104_rx1_validate_scan_mask,
	},
	{
		.preenable = adrv9104_rx2_preenable,
		.postdisable = adrv9104_rx2_postdisable,
		.validate_scan_mask = adrv9104_rx2_validate_scan_mask,
	}
};

static int adrv9104_backend_get(struct adrv9104_rf_phy *phy,
				struct adrv9104_chan *chan,
				struct iio_dev *indio_dev)
{
	const struct iio_buffer_setup_ops *setup_ops;
	const char *back_name;
	int ret;

	if (chan->port == ADI_TX) {
		setup_ops = &adrv9104_tx_buffer_ops;
		back_name = "tx-backend";
	} else {
		setup_ops = &adrv9104_rx_buffer_ops[chan->idx];
		back_name = adrv9104_rx_backend_names[chan->idx];
	}

	chan->back = devm_iio_backend_get(phy->dev, back_name);
	if (IS_ERR(chan->back))
		return PTR_ERR(chan->back);

	ret = devm_iio_backend_request_buffer_with_ops(phy->dev, chan->back, indio_dev, setup_ops);
	if (ret)
		return ret;

	if (chan->port == ADI_RX)
		return 0;

	/* Position for ADRV9104_TX_DDS_CHAN() channels */
	ret = iio_backend_extend_chan_spec(chan->back, &adrv9104_phy_chan[10]);
	if (ret)
		return ret;

	return iio_backend_extend_chan_spec(chan->back, &adrv9104_phy_chan[11]);
}

static int adrv9104_load_arm_fw(struct adrv9104_rf_phy *phy)
{
	int ret;

	const struct firmware *fw __free(firmware) = NULL;
	ret = request_firmware(&fw, "adrv9104_arm_fw.bin", phy->dev);
	if (ret)
		return ret;

	if (fw->size != ADRV9104_ARM_FW_SIZE_BYTES) {
		dev_err(phy->dev, "Unexpected ARM FW size (%zd != %u)\n", fw->size,
			ADRV9104_ARM_FW_SIZE_BYTES);
		return -ENOENT;
	}

	return adrv9104_api_call(phy, adi_adrv910x_arm_Image_Write, 0, fw->data, fw->size,
				 ADI_ADRV910X_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4, ADI_PS1);
}

static int adrv9104_digital_init(struct adrv9104_rf_phy *phy)
{
	int ret;
	u32 c;

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_AhbSpiBridge_Enable);
	if (ret)
		return ret;

	ret = adrv9104_load_arm_fw(phy);
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
				(adi_adrv910x_Profiles_t *)&phy->profile.profile, ADI_PS1);
	if (ret)
		return ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_PfirProfiles_Write, &phy->profile.pfirBuffer);
	if (ret)
		return ret;

	for (c = 0; c < ARRAY_SIZE(phy->rx_channels); c++) {
		const struct adrv9104_chan *rx = &phy->rx_channels[c].channel;
		rxConfig_t *rx_cfg = &phy->profile.profile.rxConfig[c];
		const adi_adrv910x_RxGainTableRow_t *table;
		adi_adrv910x_RxGainTableType_e table_type;
		u32 table_size;

		if (!rx->enabled)
			continue;

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

		/*
		 * !TODO: Do we support ORx in Nevis?! It does not look like we do!
		 * Do we want to properly mark the argument gainTableRows as const?!
		 */
		ret = adrv9104_api_call(phy, adi_adrv910x_Rx_GainTable_Write, ADI_RX,
					rx->channel_number, ADRV9104_RX_GAIN_TABLE_MAX_GAIN_INDEX,
					(adi_adrv910x_RxGainTableRow_t *)table, table_size,
					table_type);
		if (ret)
			return ret;
	}

	/* same const story */
	if (phy->tx_channel.channel.enabled) {
		ret = adrv9104_api_call(phy, adi_adrv910x_Tx_AttenuationTable_Write,
					ADI_ADRV910X_TX1, ADRV9104_TX_ATTEN_TABLE_MIN_INDEX,
					(adi_adrv910x_TxAttenTableRow_t *)adrv9104_tx_atten_table,
					ARRAY_SIZE(adrv9104_tx_atten_table));
		if (ret)
			return ret;
	}

	ret = adrv9104_api_call(phy, adi_adrv910x_arm_Start, ADI_PS1);
	if (ret)
		return ret;

	return adrv9104_api_call(phy, adi_adrv910x_arm_StartStatus_Check, 5000000);
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
	int ret;

	adi_common_ErrorClear(&phy->phy_dev.common);

	ret = adrv9104_api_call(phy, adi_adrv910x_HwOpen, &adrv9104_spi);
	if (ret)
		return ret;

	/* !\TODO: validate profile and compute init cals */
	adrv9104_log_enable(&phy->phy_dev.common);

	ret = adrv9104_api_call(phy, adi_adrv910x_InitAnalog, &phy->profile.profile,
				ADI_ADRV910X_DEVICECLOCKDIVISOR_BYPASS);
	if (ret)
		return ret;

	ret = adrv9104_digital_init(phy);
	if (ret)
		return ret;

	return 0;
}

static int adr9104_init(struct adrv9104_rf_phy *phy)
{
	int ret, c;

	/* Disable all the cores as it might interfere with init calibrations */
	for (c = 0; c < ARRAY_SIZE(phy->rx_channels); c++)
		iio_backend_disable(phy->rx_channels[c].channel.back);

	iio_backend_disable(phy->tx_channel.channel.back);

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

	for (c = 0; c < ARRAY_SIZE(phy->rx_channels); c++) {
		ret = iio_backend_enable(phy->rx_channels[c].channel.back);
		if (ret)
			return ret;
	}

	return iio_backend_enable(phy->tx_channel.channel.back);
}

static const struct iio_info adrv9104_phy_info = {
	.read_raw = &adrv9104_phy_read_raw,
	.write_raw = &adrv9104_phy_write_raw,
	.read_label = &adrv9104_read_label,
	.get_iio_backend = &adrv9104_get_backend,
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

		ret = adrv9104_backend_get(phy, &phy->rx_channels[c].channel, indio_dev);
		if (ret)
			return ret;
	}

	phy->channels[ADRV9104_TX] = &phy->tx_channel.channel;
	phy->tx_channel.channel.idx = 0;
	phy->tx_channel.channel.port = ADI_TX;

	ret = adrv9104_backend_get(phy, &phy->tx_channel.channel, indio_dev);
	if (ret)
		return ret;

	ret = devm_mutex_init(phy->dev, &phy->lock);
	if (ret)
		return ret;

	dev_clk = devm_clk_get_enabled(phy->dev, NULL);
	if (IS_ERR(dev_clk))
		return PTR_ERR(dev_clk);

	ret = adr9104_init(phy);
	if (ret)
		return ret;

	ret = devm_iio_device_register(phy->dev, indio_dev);
	if (ret)
		return ret;

	for (c = 0; c < ADRV9104_RX_MAX; c++)
		iio_backend_debugfs_add(phy->rx_channels[c].channel.back, indio_dev);
	iio_backend_debugfs_add(phy->tx_channel.channel.back, indio_dev);

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
