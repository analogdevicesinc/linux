// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include "adrv9002.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_bbdc.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_common_types.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_powermanagement.h"
#include "adi_adrv9001_powermanagement_types.h"
#include "adi_adrv9001_profile_types.h"
#include "adi_adrv9001_profileutil.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_spi.h"
#include "adi_adrv9001_ssi.h"
#include "adi_adrv9001_ssi_types.h"
#include "adi_adrv9001_stream.h"
#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_txSettings_types.h"
#include "adi_adrv9001_utilities.h"
#include "adi_adrv9001_version.h"
#include "adi_common_error_types.h"

/* gpio0 starts at 1 in the API enum */
#define ADRV9002_DGPIO_MIN	(ADI_ADRV9001_GPIO_DIGITAL_00 - 1)
#define ADRV9002_DGPIO_MAX	(ADI_ADRV9001_GPIO_DIGITAL_15 - 1)

#define ALL_RX_CHANNEL_MASK	(ADI_ADRV9001_RX1 | ADI_ADRV9001_RX2 | \
				 ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2)

#define ADRV9002_RX_EN(nr)	BIT(((nr) * 2) & 0x3)
#define ADRV9002_TX_EN(nr)	BIT(((nr) * 2 + 1) & 0x3)

#define ADRV9002_RX_MAX_GAIN_mdB	\
	((ADI_ADRV9001_RX_GAIN_INDEX_MAX - ADI_ADRV9001_RX_GAIN_INDEX_MIN) * ADRV9002_RX_GAIN_STEP_mDB)
#define ADRV9002_RX_GAIN_STEP_mDB	500
#define ADRV9002_RX_MIN_GAIN_IDX	ADI_ADRV9001_RX_GAIN_INDEX_MIN
#define ADRV9002_RX_MAX_GAIN_IDX	ADI_ADRV9001_RX_GAIN_INDEX_MAX

#define ADRV9002_STREAM_BINARY_SZ 	ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES

/* IRQ Masks */
#define ADRV9002_GP_MASK_RX_DP_RECEIVE_ERROR		0x08000000
#define ADRV9002_GP_MASK_TX_DP_TRANSMIT_ERROR		0x04000000
#define ADRV9002_GP_MASK_RX_DP_READ_REQUEST_FROM_BBIC	0x02000000
#define ADRV9002_GP_MASK_TX_DP_WRITE_REQUEST_TO_BBIC	0x01000000
#define ADRV9002_GP_MASK_STREAM_PROCESSOR_3_ERROR	0x00100000
#define ADRV9002_GP_MASK_STREAM_PROCESSOR_2_ERROR	0x00080000
#define ADRV9002_GP_MASK_STREAM_PROCESSOR_1_ERROR	0x00040000
#define ADRV9002_GP_MASK_STREAM_PROCESSOR_0_ERROR	0x00020000
#define ADRV9002_GP_MASK_MAIN_STREAM_PROCESSOR_ERROR	0x00010000
#define ADRV9002_GP_MASK_LSSI_RX2_CLK_MCS		0x00008000
#define ADRV9002_GP_MASK_LSSI_RX1_CLK_MCS		0x00004000
#define ADRV9002_GP_MASK_CLK_1105_MCS_SECOND		0x00002000
#define ADRV9002_GP_MASK_CLK_1105_MCS			0x00001000
#define ADRV9002_GP_MASK_CLK_PLL_LOCK			0x00000800
#define ADRV9002_GP_MASK_AUX_PLL_LOCK			0x00000400
#define ADRV9002_GP_MASK_RF2_SYNTH_LOCK			0x00000200
#define ADRV9002_GP_MASK_RF_SYNTH_LOCK			0x00000100
#define ADRV9002_GP_MASK_CLK_PLL_LOW_POWER_LOCK		0x00000080
#define ADRV9002_GP_MASK_TX2_PA_PROTECTION_ERROR	0x00000040
#define ADRV9002_GP_MASK_TX1_PA_PROTECTION_ERROR	0x00000020
#define ADRV9002_GP_MASK_CORE_ARM_MONITOR_ERROR		0x00000010
#define ADRV9002_GP_MASK_CORE_ARM_CALIBRATION_ERROR	0x00000008
#define ADRV9002_GP_MASK_CORE_ARM_SYSTEM_ERROR		0x00000004
#define ADRV9002_GP_MASK_CORE_FORCE_GP_INTERRUPT	0x00000002
#define ADRV9002_GP_MASK_CORE_ARM_ERROR			0x00000001

#define ADRV9002_IRQ_MASK					\
	(ADRV9002_GP_MASK_CORE_ARM_ERROR |			\
	 ADRV9002_GP_MASK_CORE_FORCE_GP_INTERRUPT |		\
	 ADRV9002_GP_MASK_CORE_ARM_SYSTEM_ERROR |		\
	 ADRV9002_GP_MASK_CORE_ARM_CALIBRATION_ERROR |		\
	 ADRV9002_GP_MASK_CORE_ARM_MONITOR_ERROR |		\
	 ADRV9002_GP_MASK_TX1_PA_PROTECTION_ERROR |		\
	 ADRV9002_GP_MASK_TX2_PA_PROTECTION_ERROR |		\
	 ADRV9002_GP_MASK_CLK_PLL_LOW_POWER_LOCK |		\
	 ADRV9002_GP_MASK_RF_SYNTH_LOCK |			\
	 ADRV9002_GP_MASK_RF2_SYNTH_LOCK |			\
	 ADRV9002_GP_MASK_AUX_PLL_LOCK |			\
	 ADRV9002_GP_MASK_CLK_PLL_LOCK |			\
	 ADRV9002_GP_MASK_MAIN_STREAM_PROCESSOR_ERROR |		\
	 ADRV9002_GP_MASK_STREAM_PROCESSOR_0_ERROR |		\
	 ADRV9002_GP_MASK_STREAM_PROCESSOR_1_ERROR |		\
	 ADRV9002_GP_MASK_STREAM_PROCESSOR_2_ERROR |		\
	 ADRV9002_GP_MASK_STREAM_PROCESSOR_3_ERROR |		\
	 ADRV9002_GP_MASK_TX_DP_WRITE_REQUEST_TO_BBIC |		\
	 ADRV9002_GP_MASK_RX_DP_READ_REQUEST_FROM_BBIC |	\
	 ADRV9002_GP_MASK_TX_DP_TRANSMIT_ERROR |		\
	 ADRV9002_GP_MASK_RX_DP_RECEIVE_ERROR)

int __adrv9002_dev_err(const struct adrv9002_rf_phy *phy, const char *function, const int line)
{
	int ret;

	dev_err(&phy->spi->dev, "%s, %d: failed with \"%s\" (%d)\n", function, line,
		phy->adrv9001->common.error.errormessage ?
		phy->adrv9001->common.error.errormessage : "",
		phy->adrv9001->common.error.errCode);

	switch (phy->adrv9001->common.error.errCode) {
	case ADI_COMMON_ERR_INV_PARAM:
	case ADI_COMMON_ERR_NULL_PARAM:
		ret = -EINVAL;
		break;
	case ADI_COMMON_ERR_API_FAIL:
		ret = -EFAULT;
		break;
	case ADI_COMMON_ERR_SPI_FAIL:
		ret = -EIO;
		break;
	case ADI_COMMON_ERR_MEM_ALLOC_FAIL:
		ret = -ENOMEM;
		break;
	default:
		ret = -EFAULT;
		break;
	}

	adi_common_ErrorClear(&phy->adrv9001->common);

	return ret;
}

#define adrv9002_dev_err(phy)	__adrv9002_dev_err(phy, __func__, __LINE__)

static void adrv9002_get_ssi_interface(struct adrv9002_rf_phy *phy, const int chann,
				       const bool tx, u8 *ssi_intf, u8 *n_lanes, bool *cmos_ddr_en)
{
	if (tx) {
		adi_adrv9001_TxProfile_t *tx_cfg;

		tx_cfg = &phy->curr_profile->tx.txProfile[chann];
		*ssi_intf = tx_cfg->txSsiConfig.ssiType;
		*n_lanes = tx_cfg->txSsiConfig.numLaneSel;
		*cmos_ddr_en = tx_cfg->txSsiConfig.ddrEn;
	} else {
		adi_adrv9001_RxProfile_t *rx_cfg;

		rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chann].profile;
		*ssi_intf = rx_cfg->rxSsiConfig.ssiType;
		*n_lanes = rx_cfg->rxSsiConfig.numLaneSel;
		*cmos_ddr_en = rx_cfg->rxSsiConfig.ddrEn;
	}
}

static int adrv9002_ssi_configure(struct adrv9002_rf_phy *phy)
{
	bool cmos_ddr;
	u8 n_lanes, ssi_intf;
	int c, ret;
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	unsigned long rate;

	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[c];
		struct adrv9002_chan *tx = &phy->tx_channels[c].channel;

		/* RX2/TX2 can only be enabled if RX1/TX1 are also enabled */
		if (phy->rx2tx2 && c > ADRV9002_CHANN_1)
			break;

		if (!rx->channel.enabled)
			goto tx;

		adrv9002_sync_gpio_toogle(phy);
		adrv9002_get_ssi_interface(phy, c, false, &ssi_intf, &n_lanes, &cmos_ddr);
		ret = adrv9002_axi_interface_set(phy, n_lanes, ssi_intf, cmos_ddr, c, false);
		if (ret)
			return ret;

		rate = adrv9002_axi_dds_rate_get(phy, c) * rx_cfg[c].profile.rxOutputRate_Hz;
		clk_set_rate(rx->tdd_clk, rate);

tx:
		if (!tx->enabled)
			continue;

		adrv9002_sync_gpio_toogle(phy);
		adrv9002_get_ssi_interface(phy, c, true, &ssi_intf, &n_lanes, &cmos_ddr);
		ret = adrv9002_axi_interface_set(phy, n_lanes, ssi_intf, cmos_ddr, c, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9002_phy_reg_access(struct iio_dev *indio_dev,
				   u32 reg, u32 writeval,
				   u32 *readval)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (!readval) {
		ret = adi_adrv9001_spi_Byte_Write(phy->adrv9001, reg, writeval);
	} else {
		u8 val;

		ret = adi_adrv9001_spi_Byte_Read(phy->adrv9001, reg, &val);
		*readval = val;
	}
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

#define ADRV9002_MAX_CLK_NAME 79

static char *adrv9002_clk_set_dev_name(struct adrv9002_rf_phy *phy,
				       char *dest, const char *name)
{
	size_t len = 0;

	if (!name)
		return NULL;

	if (*name == '-')
		len = strlcpy(dest, dev_name(&phy->spi->dev),
			      ADRV9002_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, ADRV9002_MAX_CLK_NAME - len);
}

static unsigned long adrv9002_bb_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int adrv9002_bb_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long adrv9002_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct adrv9002_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = adrv9002_bb_round_rate,
	.set_rate = adrv9002_bb_set_rate,
	.recalc_rate = adrv9002_bb_recalc_rate,
};

static struct clk *adrv9002_clk_register(struct adrv9002_rf_phy *phy, const char *name,
					 const unsigned long flags, const u32 source)
{
	struct adrv9002_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV9002_MAX_CLK_NAME + 1];

	/* struct adrv9002_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	init.name = adrv9002_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.num_parents = 0;
	init.ops = &bb_clk_ops;

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	if (IS_ERR(clk)) {
		dev_err(&phy->spi->dev, "Error registering clock=%d, err=%ld\n", source,
			PTR_ERR(clk));
		return ERR_CAST(clk);
	}

	phy->clks[phy->n_clks++] = clk;

	return clk;
}

static void adrv9002_set_clk_rates(const struct adrv9002_rf_phy *phy)
{
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	const struct adi_adrv9001_TxProfile *tx_cfg = phy->curr_profile->tx.txProfile;
	int c;

	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		const struct adrv9002_chan *rx = &phy->rx_channels[c].channel;
		const struct adrv9002_chan *tx = &phy->tx_channels[c].channel;
		/* RX2/TX2 can only be enabled if RX1/TX1 are also enabled */
		if (phy->rx2tx2 && c > ADRV9002_CHANN_1)
			break;

		if (!rx->enabled)
			goto tx_clk;

		clk_set_rate(rx->clk, rx_cfg[c].profile.rxOutputRate_Hz);
tx_clk:
		if (!tx->enabled)
			continue;

		clk_set_rate(tx->clk, tx_cfg[c].txInputRate_Hz);
	}
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static int adrv9002_gainidx_to_gain(int idx)
{
	idx = clamp(idx, ADRV9002_RX_MIN_GAIN_IDX, ADRV9002_RX_MAX_GAIN_IDX);

	return (idx - ADRV9002_RX_MIN_GAIN_IDX) * ADRV9002_RX_GAIN_STEP_mDB;
}

static int adrv9002_gain_to_gainidx(int gain)
{
	int temp;

	gain = clamp(gain, 0, ADRV9002_RX_MAX_GAIN_mdB);
	temp = DIV_ROUND_CLOSEST(gain, ADRV9002_RX_GAIN_STEP_mDB);

	return temp + ADRV9002_RX_MIN_GAIN_IDX;
}

/* lock is assumed to be held... */
static int adrv9002_channel_to_state(struct adrv9002_rf_phy *phy,
				     struct adrv9002_chan *chann,
				     const int port,
				     const adi_adrv9001_ChannelState_e state,
				     const bool cache_state)
{
	int ret;
	adi_adrv9001_ChannelEnableMode_e mode;

	/* nothing to do */
	if (!chann->enabled)
		return 0;

	ret = adi_adrv9001_Radio_ChannelEnableMode_Get(phy->adrv9001, port,
						       chann->number, &mode);
	if (ret)
		return adrv9002_dev_err(phy);

	/* we need to set it to spi */
	if (mode == ADI_ADRV9001_PIN_MODE) {
		ret = adi_adrv9001_Radio_ChannelEnableMode_Set(phy->adrv9001, port,
							       chann->number,
							       ADI_ADRV9001_SPI_MODE);
		if (ret)
			return adrv9002_dev_err(phy);

		/*
		 * When moving from pin to spi mode, the device might change the ensm automatically.
		 * Hence, we need to wait to cache the right value and to make sure that the device
		 * is in a stable state so that @adi_adrv9001_Radio_Channel_ToState() actually
		 * works.
		 */
		usleep_range(2000, 3000);
	}

	if (cache_state) {
		ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, port,
							   chann->number,
							   &chann->cached_state);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, port,
						 chann->number, state);
	if (ret)
		return adrv9002_dev_err(phy);

	/*
	 * We also need to wait here so that the device goes into the right state. This is important
	 * when an operation requiring these state transitions is called multiple times in a row...
	 */
	usleep_range(2000, 3000);
	if (mode == ADI_ADRV9001_SPI_MODE)
		return 0;

	/* restore enable mode */
	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(phy->adrv9001, port,
						       chann->number, mode);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static struct
adrv9002_chan *adrv9002_get_channel(struct adrv9002_rf_phy *phy,
				    const int port, const int chann)
{
	if (port == ADI_TX)
		return &phy->tx_channels[chann].channel;

	return &phy->rx_channels[chann].channel;
}

static ssize_t adrv9002_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	int ret = 0;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);
	struct adi_adrv9001_Carrier lo_freq;
	u64 freq;

	mutex_lock(&phy->lock);

	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case LOEXT_FREQ:
		ret = kstrtoull(buf, 10, &freq);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, port,
							 chann->number, &lo_freq);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		lo_freq.carrierFrequency_Hz = freq;
		ret = adrv9002_channel_to_state(phy, chann, port,
						ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Radio_Carrier_Configure(phy->adrv9001, port,
							   chann->number, &lo_freq);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		ret = adrv9002_channel_to_state(phy, chann, port,
						chann->cached_state, false);
		break;
	default:
		ret = -EINVAL;
	}

unlock:
	mutex_unlock(&phy->lock);
	return ret ? ret : len;
}

static ssize_t adrv9002_phy_lo_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, channel);
	struct adi_adrv9001_Carrier lo_freq;

	mutex_lock(&phy->lock);

	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case LOEXT_FREQ:
		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, port,
							 chann->number, &lo_freq);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%llu\n", lo_freq.carrierFrequency_Hz);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
}

#define _ADRV9002_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrv9002_phy_lo_read, \
	.write = adrv9002_phy_lo_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info adrv9002_phy_ext_lo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV9002_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{ },
};

static int adrv9002_set_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_ChannelNumber_e chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;

	if (mode > ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO)
		return -EINVAL;

	mutex_lock(&phy->lock);

	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_GainControl_Mode_Set(phy->adrv9001,
						   rx->channel.number, mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_get_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RxGainControlMode_e gain_ctrl_mode;
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_GainControl_Mode_Get(phy->adrv9001,
						   rx->channel.number,
						   &gain_ctrl_mode);
	mutex_unlock(&phy->lock);

	return gain_ctrl_mode;
}

static const char * const adrv9002_agc_modes[] = {
	"spi", "pin", "automatic"
};

static const struct iio_enum adrv9002_agc_modes_available = {
	.items = adrv9002_agc_modes,
	.num_items = ARRAY_SIZE(adrv9002_agc_modes),
	.get = adrv9002_get_agc_mode,
	.set = adrv9002_set_agc_mode,
};

static int adrv9002_set_ensm_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, channel);

	mutex_lock(&phy->lock);
	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}
	/*
	 * In TDD, we cannot have TX and RX enabled at the same time on the same
	 * channel (due to TDD nature). Hence, we will return -EPERM if that is
	 * attempted...
	 */
	if (phy->curr_profile->sysConfig.duplexMode == ADI_ADRV9001_TDD_MODE &&
	    mode + 1 == ADI_ADRV9001_CHANNEL_RF_ENABLED) {
		enum adi_adrv9001_ChannelState state;
		/* just the last bit matters as RX is 0 and TX is 1 */
		adi_common_Port_e __port = ~port & 0x1;

		ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, __port,
							   chann->number, &state);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		if (state == ADI_ADRV9001_CHANNEL_RF_ENABLED) {
			ret = -EPERM;
			goto unlock;
		}
	}
	ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, port,
						 chann->number, mode + 1);
	if (ret)
		ret = adrv9002_dev_err(phy);
unlock:
	mutex_unlock(&phy->lock);

	return ret;
}

static int adrv9002_get_ensm_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	enum adi_adrv9001_ChannelState state;
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, channel);

	mutex_lock(&phy->lock);
	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, port,
						   chann->number, &state);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return state - 1;
}

static const char * const adrv9002_ensm_modes[] = {
	"calibrated", "primed", "rf_enabled"
};

static const struct iio_enum adrv9002_ensm_modes_available = {
	.items = adrv9002_ensm_modes,
	.num_items = ARRAY_SIZE(adrv9002_ensm_modes),
	.get = adrv9002_get_ensm_mode,
	.set = adrv9002_set_ensm_mode,
};

static int adrv9002_set_digital_gain_ctl_mode(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan,
					      u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	struct adi_adrv9001_RxInterfaceGainCtrl rx_intf_gain_mode = {0};
	u32 gain_table_type;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Inspect(phy->adrv9001, rx->channel.number,
						    &rx_intf_gain_mode, &gain_table_type);
	if (ret) {
		mutex_unlock(&phy->lock);
		return adrv9002_dev_err(phy);
	}

	rx_intf_gain_mode.controlMode = mode;

	ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
					ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Rx_InterfaceGain_Configure(phy->adrv9001,
						      rx->channel.number,
						      &rx_intf_gain_mode);
	if (ret) {
		ret = adrv9002_dev_err(phy);
		goto unlock;
	}

	ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
					rx->channel.cached_state, false);
unlock:
	mutex_unlock(&phy->lock);

	return ret;
}

static int adrv9002_get_digital_gain_ctl_mode(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	struct adi_adrv9001_RxInterfaceGainCtrl rx_intf_gain_mode;
	u32 gain_table_type;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Inspect(phy->adrv9001,
						    rx->channel.number,
						    &rx_intf_gain_mode,
						    &gain_table_type);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return rx_intf_gain_mode.controlMode;
}

static int adrv9002_get_intf_gain(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	adi_adrv9001_RxInterfaceGain_e gain;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Get(phy->adrv9001,
						rx->channel.number, &gain);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return gain;
}

static int adrv9002_set_intf_gain(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Rx_InterfaceGain_Set(phy->adrv9001,
						rx->channel.number, mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_get_port_en_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	adi_adrv9001_ChannelEnableMode_e mode;
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);

	mutex_lock(&phy->lock);
	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Get(phy->adrv9001, port,
						       chann->number, &mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return mode;
}

static int adrv9002_set_port_en_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);

	mutex_lock(&phy->lock);
	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(phy->adrv9001, port,
						       chann->number, mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_update_tracking_calls(struct adrv9002_rf_phy *phy,
					  const u32 mask, const int chann,
					  const bool enable)
{
	int ret, i;
	struct adi_adrv9001_TrackingCals tracking_cals;

	ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001, &tracking_cals);
	if (ret)
		return adrv9002_dev_err(phy);

	/* all channels need to be in calibrated state...*/
	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
						ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;

		ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_TX,
						ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;
	}

	if (enable)
		tracking_cals.chanTrackingCalMask[chann] |= mask;
	else
		tracking_cals.chanTrackingCalMask[chann] &= ~mask;

	ret = adi_adrv9001_cals_Tracking_Set(phy->adrv9001, &tracking_cals);
	if (ret)
		return adrv9002_dev_err(phy);

	/* restore state */
	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		if (!rx->channel.enabled)
			goto tx_restore;

		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
						rx->channel.cached_state, false);
		if (ret)
			return ret;

tx_restore:
		if (!tx->channel.enabled)
			continue;

		ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_TX,
						tx->channel.cached_state, false);
		if (ret)
			return ret;
	}

	return 0;
}

static const u32 rx_track_calls[] = {
	[RX_QEC_FIC] = ADI_ADRV9001_TRACKING_CAL_RX_QEC_FIC,
	[RX_QEC_W_POLY] = ADI_ADRV9001_TRACKING_CAL_RX_QEC_WBPOLY,
	[RX_AGC] = ADI_ADRV9001_TRACKING_CAL_RX_AGC,
	[RX_TRACK_BBDC] = ADI_ADRV9001_TRACKING_CAL_RX_BBDC,
	[RX_HD2] = ADI_ADRV9001_TRACKING_CAL_RX_HD2,
	[RX_RSSI_CAL] = ADI_ADRV9001_TRACKING_CAL_RX_RSSI,
	[RX_RFDC] = ADI_ADRV9001_TRACKING_CAL_RX_RFDC
};

static ssize_t adrv9002_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	int ret = 0, freq_offset_hz;
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[channel];
	bool enable;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;

		ret = adrv9002_update_tracking_calls(phy,
						     rx_track_calls[private],
						     channel, enable);
		break;
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn) {
			ret = -ENOTSUPP;
			goto unlock;
		}

		ret = kstrtoint(buf, 10, &freq_offset_hz);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Rx_FrequencyCorrection_Set(phy->adrv9001,
							      rx->channel.number,
							      freq_offset_hz,
							      true);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		rx->channel.nco_freq = freq_offset_hz;
		break;
	case RX_ADC_SWITCH:
		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;

		/* we must be in calibrated state */
		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
						ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Rx_AdcSwitchEnable_Set(phy->adrv9001,
							  rx->channel.number,
							  enable);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_RX,
						rx->channel.cached_state,
						false);
		break;
	case RX_BBDC:
		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;
		/*
		 * Disabling the bbdc will completely disable the algorithm and set the correction
		 * value to 0. The difference with the tracking cal is that disabling it, just
		 * disables the algorithm but the last used correction value is still applied...
		 */
		ret = adi_adrv9001_bbdc_RejectionEnable_Set(phy->adrv9001, ADI_RX,
							    rx->channel.number, enable);
		if (ret)
			ret = adrv9002_dev_err(phy);

		break;
	default:
		ret = -EINVAL;
	}

unlock:
	mutex_unlock(&phy->lock);
	return ret ? ret : len;
}

static ssize_t adrv9002_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u16 dec_pwr_mdb;
	u32 rssi_pwr_mdb;
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[channel];
	adi_adrv9001_BbdcRejectionStatus_e bbdc;
	bool enable;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001,
						     &tracking_cals);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%d\n", calls_mask[channel] & rx_track_calls[private] ? 1 : 0);
		break;
	case RX_DECIMATION_POWER:
		/* it might depend on proper AGC parameters */
		ret = adi_adrv9001_Rx_DecimatedPower_Get(phy->adrv9001,
							 rx->channel.number,
							 &dec_pwr_mdb);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000, dec_pwr_mdb % 1000);
		break;
	case RX_RSSI:
		ret = adi_adrv9001_Rx_Rssi_Read(phy->adrv9001,
						rx->channel.number, &rssi_pwr_mdb);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%u.%02u dB\n", rssi_pwr_mdb / 1000, rssi_pwr_mdb % 1000);
		break;
	case RX_RF_BANDWIDTH:
		rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chan->channel];
		ret = sprintf(buf, "%u\n", rx_cfg->profile.primarySigBandwidth_Hz);
		break;
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn) {
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}

		ret = sprintf(buf, "%d\n", rx->channel.nco_freq);
		break;
	case RX_ADC_SWITCH:
		ret = adi_adrv9001_Rx_AdcSwitchEnable_Get(phy->adrv9001,
							  rx->channel.number,
							  &enable);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%d\n", enable);
		break;
	case RX_BBDC:
		ret = adi_adrv9001_bbdc_RejectionEnable_Get(phy->adrv9001, ADI_RX,
							    rx->channel.number, &bbdc);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sprintf(buf, "%d\n", bbdc);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
}

#define _ADRV9002_EXT_RX_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrv9002_phy_rx_read, \
	.write = adrv9002_phy_rx_write, \
	.private = _ident, \
}

static const u32 tx_track_calls[] = {
	[TX_QEC] = ADI_ADRV9001_TRACKING_CAL_TX_QEC,
	[TX_LOL] = ADI_ADRV9001_TRACKING_CAL_TX_LO_LEAKAGE,
	[TX_LB_PD] = ADI_ADRV9001_TRACKING_CAL_TX_LB_PD,
	[TX_PAC] = ADI_ADRV9001_TRACKING_CAL_TX_PAC,
	[TX_CLGC] = ADI_ADRV9001_TRACKING_CAL_TX_DPD_CLGC
};

static int adrv9002_set_atten_control_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[chann];
	adi_adrv9001_TxAttenuationControlMode_e tx_mode;
	int ret;

	switch (mode) {
	case 0:
		tx_mode = ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_BYPASS;
		break;
	case 1:
		tx_mode = ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI;
		break;
	case 2:
		tx_mode = ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_PIN;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}
	/* we must be in calibrated state */
	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_TX,
					ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Tx_AttenuationMode_Set(phy->adrv9001,
						  tx->channel.number, tx_mode);
	if (ret) {
		ret = adrv9002_dev_err(phy);
		goto unlock;
	}

	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_TX,
					tx->channel.cached_state, false);
unlock:
	mutex_unlock(&phy->lock);

	return ret;
}

static int adrv9002_get_atten_control_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[chann];
	adi_adrv9001_TxAttenuationControlMode_e tx_mode;
	int mode, ret;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Tx_AttenuationMode_Get(phy->adrv9001,
						  tx->channel.number, &tx_mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	switch (tx_mode) {
	case ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_BYPASS:
		mode = 0;
		break;
	case ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI:
		mode = 1;
		break;
	case ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_PIN:
		mode = 2;
		break;
	default:
		return -EINVAL;
	}

	return mode;
}

static ssize_t adrv9002_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[channel];
	struct adi_adrv9001_TxProfile *tx_cfg = &phy->curr_profile->tx.txProfile[channel];
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	int val = 0, ret = 0;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001,
						     &tracking_cals);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		val = calls_mask[channel] & tx_track_calls[private] ? 1 : 0;
		ret = sprintf(buf, "%d\n", val);
		break;
	case TX_RF_BANDWIDTH:
		ret = sprintf(buf, "%d\n", tx_cfg->primarySigBandwidth_Hz);
		break;
	case TX_NCO_FREQUENCY:
		/*
		 * This field seems to be the only thing that changes on TX profiles when nco
		 * is enabled.
		 */
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2) {
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}

		ret = sprintf(buf, "%d\n", tx->channel.nco_freq);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
}

static ssize_t adrv9002_phy_tx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[channel];
	struct adi_adrv9001_TxProfile *tx_cfg = &phy->curr_profile->tx.txProfile[channel];
	bool enable;
	int ret = 0, nco_freq_hz;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;

		ret = adrv9002_update_tracking_calls(phy, tx_track_calls[private], channel, enable);
		break;
	case TX_NCO_FREQUENCY:
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2) {
			ret = -ENOTSUPP;
			goto unlock;
		}

		ret = kstrtoint(buf, 10, &nco_freq_hz);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Tx_FrequencyCorrection_Set(phy->adrv9001, tx->channel.number,
							      nco_freq_hz, true);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		tx->channel.nco_freq = nco_freq_hz;
		break;
	default:
		ret = -EINVAL;
	}

unlock:
	mutex_unlock(&phy->lock);
	return ret ? ret : len;
}

#define _ADRV9002_EXT_TX_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrv9002_phy_tx_read, \
	.write = adrv9002_phy_tx_write, \
	.private = _ident, \
}

static const char * const adrv9002_digital_gain_ctl_modes[] = {
	"automatic", "spi"
};

static const struct iio_enum adrv9002_digital_gain_ctl_modes_available = {
	.items = adrv9002_digital_gain_ctl_modes,
	.num_items = ARRAY_SIZE(adrv9002_digital_gain_ctl_modes),
	.get = adrv9002_get_digital_gain_ctl_mode,
	.set = adrv9002_set_digital_gain_ctl_mode,
};

static const char * const adrv9002_intf_gain[] = {
	"18dB", "12dB", "6dB", "0dB", "-6dB", "-12dB", "-18dB",
	"-24dB", "-30dB", "-36dB"
};

static const struct iio_enum adrv9002_intf_gain_available = {
	.items = adrv9002_intf_gain,
	.num_items = ARRAY_SIZE(adrv9002_intf_gain),
	.get = adrv9002_get_intf_gain,
	.set = adrv9002_set_intf_gain,
};

static const char *const adrv9002_port_en_mode[] = {
	"spi", "pin"
};

static const struct iio_enum adrv9002_port_en_modes_available = {
	.items = adrv9002_port_en_mode,
	.num_items = ARRAY_SIZE(adrv9002_port_en_mode),
	.get = adrv9002_get_port_en_mode,
	.set = adrv9002_set_port_en_mode,
};

static const char *const adrv9002_atten_control_mode[] = {
	"bypass", "spi", "pin"
};

static const struct iio_enum adrv9002_atten_control_mode_available = {
	.items = adrv9002_atten_control_mode,
	.num_items = ARRAY_SIZE(adrv9002_atten_control_mode),
	.get = adrv9002_get_atten_control_mode,
	.set = adrv9002_set_atten_control_mode,
};

static const struct iio_chan_spec_ext_info adrv9002_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE_SHARED("ensm_mode", 0,
				  &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", 0, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("gain_control_mode", 0,
				  &adrv9002_agc_modes_available),
	IIO_ENUM("gain_control_mode", 0, &adrv9002_agc_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("digital_gain_control_mode", 0,
				  &adrv9002_digital_gain_ctl_modes_available),
	IIO_ENUM("digital_gain_control_mode", 0,
		 &adrv9002_digital_gain_ctl_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("interface_gain", 0,
				  &adrv9002_intf_gain_available),
	IIO_ENUM("interface_gain", 0,
		 &adrv9002_intf_gain_available),
	IIO_ENUM_AVAILABLE_SHARED("port_en_mode", 0,
				  &adrv9002_port_en_modes_available),
	IIO_ENUM("port_en_mode", 0, &adrv9002_port_en_modes_available),
	_ADRV9002_EXT_RX_INFO("rssi", RX_RSSI),
	_ADRV9002_EXT_RX_INFO("decimated_power", RX_DECIMATION_POWER),
	_ADRV9002_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV9002_EXT_RX_INFO("nco_frequency", RX_NCO_FREQUENCY),
	_ADRV9002_EXT_RX_INFO("quadrature_fic_tracking_en", RX_QEC_FIC),
	_ADRV9002_EXT_RX_INFO("quadrature_w_poly_tracking_en", RX_QEC_W_POLY),
	_ADRV9002_EXT_RX_INFO("agc_tracking_en", RX_AGC),
	_ADRV9002_EXT_RX_INFO("bbdc_rejection_tracking_en", RX_TRACK_BBDC),
	_ADRV9002_EXT_RX_INFO("hd_tracking_en", RX_HD2),
	_ADRV9002_EXT_RX_INFO("rssi_tracking_en", RX_RSSI_CAL),
	_ADRV9002_EXT_RX_INFO("rfdc_tracking_en", RX_RFDC),
	_ADRV9002_EXT_RX_INFO("dynamic_adc_switch_en", RX_ADC_SWITCH),
	_ADRV9002_EXT_RX_INFO("bbdc_rejection_en", RX_BBDC),
	{ },
};

static struct iio_chan_spec_ext_info adrv9002_phy_tx_ext_info[] = {
	IIO_ENUM_AVAILABLE_SHARED("ensm_mode", 0,
				  &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", 0, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("port_en_mode", 0,
				  &adrv9002_port_en_modes_available),
	IIO_ENUM("port_en_mode", 0, &adrv9002_port_en_modes_available),
	IIO_ENUM_AVAILABLE_SHARED("atten_control_mode", 0,
				  &adrv9002_atten_control_mode_available),
	IIO_ENUM("atten_control_mode", 0,
		 &adrv9002_atten_control_mode_available),
	_ADRV9002_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	_ADRV9002_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_ADRV9002_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_ADRV9002_EXT_TX_INFO("loopback_delay_tracking_en", TX_LB_PD),
	_ADRV9002_EXT_TX_INFO("pa_correction_tracking_en", TX_PAC),
	_ADRV9002_EXT_TX_INFO("close_loop_gain_tracking_en", TX_CLGC),
	_ADRV9002_EXT_TX_INFO("nco_frequency", TX_NCO_FREQUENCY),
	{ },
};

static int adrv9002_channel_power_set(struct adrv9002_rf_phy *phy,
				      struct adrv9002_chan *channel,
				      const adi_common_Port_e port,
				      const int val)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Set power: %d, chan: %d, port: %d\n",
		val, channel->number, port);

	if (!val && channel->power) {
		ret = adrv9002_channel_to_state(phy, channel, port,
						ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;

		ret = adi_adrv9001_Radio_Channel_PowerDown(phy->adrv9001, port,
							   channel->number);
		if (ret)
			return adrv9002_dev_err(phy);

		channel->power = false;
	} else if (val && !channel->power) {
		ret = adi_adrv9001_Radio_Channel_PowerUp(phy->adrv9001, port,
							 channel->number);
		if (ret)
			return adrv9002_dev_err(phy);

		ret = adrv9002_channel_to_state(phy, channel, port,
						channel->cached_state, false);
		if (ret)
			return ret;

		channel->power = true;
	}

	return 0;
}

static int adrv9002_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long m)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);
	u16 temp;
	int ret;
	u8 index;

	/* we can still read the device temperature... */
	mutex_lock(&phy->lock);
	if (m != IIO_CHAN_INFO_PROCESSED) {
		if (!chann->enabled) {
			mutex_unlock(&phy->lock);
			return -ENODEV;
		}
	}

	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			u16 atten_mdb;

			ret = adi_adrv9001_Tx_Attenuation_Get(phy->adrv9001,
							      chann->number,
							      &atten_mdb);
			if (ret) {
				mutex_unlock(&phy->lock);
				return adrv9002_dev_err(phy);
			}

			*val = -1 * (atten_mdb / 1000);
			*val2 = (atten_mdb % 1000) * 1000;
			if (!*val)
				*val2 *= -1;
		} else {

			ret = adi_adrv9001_Rx_Gain_Get(phy->adrv9001, chann->number,
						       &index);
			if (ret) {
				mutex_unlock(&phy->lock);
				return adrv9002_dev_err(phy);
			}

			temp = adrv9002_gainidx_to_gain(index);
			*val = temp / 1000;
			*val2 = temp % 1000 * 1000;
		}

		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(chann->clk);
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_ENABLE:
		*val = chann->power;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		ret = adi_adrv9001_Temperature_Get(phy->adrv9001, &temp);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		*val = temp * 1000;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
};

static int adrv9002_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int val,
				  int val2, long mask)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);
	u32 code;
	int ret = 0;

	mutex_lock(&phy->lock);
	if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0)) {
				mutex_unlock(&phy->lock);
				return -EINVAL;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			ret = adi_adrv9001_Tx_Attenuation_Set(phy->adrv9001,
							      chann->number,
							      code);
		} else {
			u8 idx;
			int gain;

			gain = val * 1000 + val2 / 1000;
			idx = adrv9002_gain_to_gainidx(gain);
			ret = adi_adrv9001_Rx_Gain_Set(phy->adrv9001,
						       chann->number, idx);
		}

		if (ret)
			ret = adrv9002_dev_err(phy);

		break;
	case IIO_CHAN_INFO_ENABLE:
		ret = adrv9002_channel_power_set(phy, chann, port, val);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
}

#define ADRV9002_IIO_LO_CHAN(idx, name, port, chan) {	\
	.type = IIO_ALTVOLTAGE,				\
	.indexed = 1,					\
	.output = 1,					\
	.channel = idx,					\
	.extend_name = name,				\
	.ext_info = adrv9002_phy_ext_lo_info,		\
	.address = ADRV_ADDRESS(port, chan),		\
}

#define ADRV9002_IIO_TX_CHAN(idx, port, chan) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_ENABLE) |		\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.ext_info = adrv9002_phy_tx_ext_info,			\
	.address = ADRV_ADDRESS(port, chan),			\
}

#define ADRV9002_IIO_RX_CHAN(idx, port, chan) {			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
			BIT(IIO_CHAN_INFO_ENABLE) |		\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.ext_info = adrv9002_phy_rx_ext_info,			\
	.address = ADRV_ADDRESS(port, chan),			\
}

static const struct iio_chan_spec adrv9002_phy_chan[] = {
	ADRV9002_IIO_LO_CHAN(0, "RX1_LO", ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_LO_CHAN(1, "RX2_LO", ADI_RX, ADRV9002_CHANN_2),
	ADRV9002_IIO_LO_CHAN(2, "TX1_LO", ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_LO_CHAN(3, "TX2_LO", ADI_TX, ADRV9002_CHANN_2),
	ADRV9002_IIO_TX_CHAN(0, ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_TX_CHAN(1, ADI_TX, ADRV9002_CHANN_2),
	ADRV9002_IIO_RX_CHAN(0, ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_RX_CHAN(1, ADI_RX, ADRV9002_CHANN_2),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info adrv9002_phy_info = {
	.read_raw = &adrv9002_phy_read_raw,
	.write_raw = &adrv9002_phy_write_raw,
	.debugfs_reg_access = &adrv9002_phy_reg_access,
};

static const struct {
	char *irq_source;
	int action;
} adrv9002_irqs[] = {
	{"ARM error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Force GP interrupt(Set by firmware to send an interrupt to BBIC)",
	 ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"ARM System error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"ARM Calibration error", ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL},
	{"ARM monitor interrupt", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"TX1 PA protection error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"TX2 PA protection error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Low-power PLL lock indicator", ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"RF PLL1 lock indicator", ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"RF PLL2 lock indicator", ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"Auxiliary Clock PLL lock indicator", ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"Clock PLL lock indicator", ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR},
	{"Main clock 1105 MCS", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Main clock 1105 second MCS", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"RX1 LSSI MCS", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"RX2 LSSI MCS", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Core stream processor error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Stream0 error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Stream1 error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Stream2 error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Stream3 error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"Unknown GP Interrupt source"},
	{"Unknown GP Interrupt source"},
	{"Unknown GP Interrupt source"},
	{"TX DP write request error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"RX DP write request error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"SPI interface transmit error", ADI_COMMON_ACT_ERR_RESET_FULL},
	{"SPI interface receive error", ADI_COMMON_ACT_ERR_RESET_FULL},
};

static irqreturn_t adrv9002_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret, bit;
	u32 status;
	unsigned long active_irq;
	u8 error;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_gpio_GpIntStatus_Get(phy->adrv9001, &status);
	if (ret)
		goto irq_done;

	dev_dbg(&phy->spi->dev, "GP Interrupt Status 0x%08X Mask 0x%08X\n",
		status, ADRV9002_IRQ_MASK);

	active_irq = status & ADRV9002_IRQ_MASK;
	for_each_set_bit(bit, &active_irq, ARRAY_SIZE(adrv9002_irqs)) {
		/* check if there's something to be done */
		switch (adrv9002_irqs[bit].action) {
		case ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL:
			dev_warn(&phy->spi->dev, "Re-running tracking calibrations\n");
			ret = adi_adrv9001_cals_InitCals_Run(phy->adrv9001, &phy->init_cals,
							     60000, &error);
			if (ret)
				/* just log the error */
				adrv9002_dev_err(phy);
			break;
		case ADI_COMMON_ACT_ERR_RESET_FULL:
			dev_warn(&phy->spi->dev, "[%s]: Reset might be needed...\n",
				 adrv9002_irqs[bit].irq_source);
			break;
		case ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR:
			/* just log the irq */
			dev_dbg(&phy->spi->dev, "%s\n", adrv9002_irqs[bit].irq_source);
			break;
		default:
			/* no defined action. print out interrupt source */
			dev_warn(&phy->spi->dev, "%s\n", adrv9002_irqs[bit].irq_source);
			break;
		}
	}

irq_done:
	mutex_unlock(&phy->lock);
	return IRQ_HANDLED;
}

static int adrv9002_dgpio_config(struct adrv9002_rf_phy *phy)
{
	struct adrv9002_gpio *dgpio = phy->adrv9002_gpios;
	int i, ret;

	for (i = 0; i < phy->ngpios; i++) {
		dev_dbg(&phy->spi->dev, "Set dpgio: %d, signal: %d\n",
			dgpio[i].gpio.pin, dgpio[i].signal);

		ret = adi_adrv9001_gpio_Configure(phy->adrv9001,
						  dgpio[i].signal,
						  &dgpio[i].gpio);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	return 0;
}

#define ADRV9001_BF_EQUAL(mask, value) ((value) == ((value) & (mask)))

static int adrv9001_rx_path_config(struct adrv9002_rf_phy *phy,
				   const adi_adrv9001_ChannelState_e state)
{
	struct adi_adrv9001_Device *adrv9001_dev = phy->adrv9001;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(phy->rx_channels); i++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		/* For each rx channel enabled */
		if (!rx->channel.enabled)
			continue;

		if (!rx->pin_cfg)
			goto agc_cfg;

		ret = adi_adrv9001_Rx_GainControl_PinMode_Configure(phy->adrv9001,
								    rx->channel.number,
								    rx->pin_cfg);
		if (ret)
			return adrv9002_dev_err(phy);

agc_cfg:
		if (!rx->agc)
			goto rf_enable;

		ret = adi_adrv9001_Rx_GainControl_Configure(phy->adrv9001,
							    rx->channel.number,
							    rx->agc);
		if (ret)
			return adrv9002_dev_err(phy);

rf_enable:
		ret = adi_adrv9001_Radio_Channel_ToState(adrv9001_dev, ADI_RX,
							 rx->channel.number, state);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	return 0;
}

static int adrv9002_tx_set_dac_full_scale(struct adrv9002_rf_phy *phy)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];

		if (!tx->channel.enabled || !tx->dac_boost_en)
			continue;

		ret = adi_adrv9001_Tx_OutputPowerBoost_Set(phy->adrv9001,
							   tx->channel.number,
							   true);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	return ret;
}

static int adrv9002_tx_path_config(struct adrv9002_rf_phy *phy,
				   const adi_adrv9001_ChannelState_e state)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];

		/* For each tx channel enabled */
		if (!tx->channel.enabled)
			continue;

		if (!tx->pin_cfg)
			goto rf_enable;

		ret = adi_adrv9001_Tx_Attenuation_PinControl_Configure(phy->adrv9001,
								       tx->channel.number,
								       tx->pin_cfg);
		if (ret)
			return adrv9002_dev_err(phy);

rf_enable:
		ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, ADI_TX,
							 tx->channel.number, state);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	return 0;
}

static const u32 adrv9002_init_cals_mask[16][2] = {
	/* Not a valid case. At least one channel should be enabled */
	[0] = {0, 0},
	/* tx2:0 rx2:0 tx1:0 rx1:1 */
	[1] = {0x1BE400, 0},
	/* tx2:0 rx2:0 tx1:1 rx1:0 */
	[2] = {0x1BE5F7, 0},
	/* tx2:0 rx2:0 tx1:1 rx1:1 */
	[3] = {0x1BE5F7, 0},
	/* tx2:0 rx2:1 tx1:0 rx1:0 */
	[4] = {0, 0x11E400},
	/* tx2:0 rx2:1 tx1:0 rx1:1 */
	[5] = {0x1BE400, 0x1BE400},
	/* tx2:0 rx2:1 tx1:1 rx1:0 */
	[6] = {0x1BE5F7, 0x1BE400},
	/* tx2:0 rx2:1 tx1:1 rx1:1 */
	[7] = {0x1BE5F7, 0x1BE400},
	/* tx2:1 rx2:0 tx1:0 rx1:0 */
	[8] = {0, 0x11E5F0},
	/* tx2:1 rx2:0 tx1:0 rx1:1 */
	[9] = {0x1BE400, 0x1BE5F0},
	/* tx2:1 rx2:0 tx1:1 rx1:0 */
	[10] = {0x1BE5F7, 0x1BE5F7},
	/* tx2:1 rx2:0 tx1:1 rx1:1 */
	[11] = {0x1BE5F7, 0x1BE5F7},
	/* tx2:1 rx2:1 tx1:0 rx1:0 */
	[12] = {0, 0x11E5F0},
	/* tx2:1 rx2:1 tx1:0 rx1:1 */
	[13] = {0x1BE400, 0x1BE5F0},
	/* tx2:1 rx2:1 tx1:1 rx1:0 */
	[14] = {0x1BE5F7, 0x1BE5F7},
	/* tx2:1 rx2:1 tx1:1 rx1:1 */
	[15] = {0x1BE5F7, 0x1BE5F7},
};

static int adrv9002_compute_init_cals(struct adrv9002_rf_phy *phy)
{
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	const struct adi_adrv9001_TxProfile *tx_cfg = phy->curr_profile->tx.txProfile;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	const u32 tx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_TX1, ADI_ADRV9001_TX2
	};
	const u32 rx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_RX1, ADI_ADRV9001_RX2
	};
	int i, pos = 0;

	phy->init_cals.sysInitCalMask = 0;
	phy->init_cals.calMode = ADI_ADRV9001_INIT_CAL_MODE_ALL;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		if (!ADRV9001_BF_EQUAL(phy->curr_profile->rx.rxInitChannelMask, rx_channels[i]))
			goto tx;

		if (phy->rx2tx2 && i &&
		    rx_cfg[i].profile.rxOutputRate_Hz != rx_cfg[0].profile.rxOutputRate_Hz) {
			dev_err(&phy->spi->dev, "In rx2tx2, RX%d rate=%u must be equal to RX1, rate=%u\n",
				i + 1, rx_cfg[i].profile.rxOutputRate_Hz,
				rx_cfg[0].profile.rxOutputRate_Hz);
			return -EINVAL;
		} else if (phy->rx2tx2 && i && !phy->rx_channels[0].channel.enabled) {
			dev_err(&phy->spi->dev, "In rx2tx2, RX%d cannot be enabled while RX1 is disabled",
				i + 1);
			return -EINVAL;
		} else if (ssi_type != rx_cfg[i].profile.rxSsiConfig.ssiType) {
			dev_err(&phy->spi->dev, "SSI interface mismatch. PHY=%d, RX%d=%d\n",
				ssi_type, i + 1, rx_cfg[i].profile.rxSsiConfig.ssiType);
			return -EINVAL;
		} else if (rx_cfg[i].profile.rxSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
			dev_err(&phy->spi->dev, "SSI interface Long Strobe not supported\n");
			return -EINVAL;
		} else if (ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS &&
			   !rx_cfg[i].profile.rxSsiConfig.ddrEn) {
			dev_err(&phy->spi->dev, "RX%d: Single Data Rate port not supported for LVDS\n",
				i + 1);
			return -EINVAL;
		}

		dev_dbg(&phy->spi->dev, "RX%d enabled\n", i);
		pos |= ADRV9002_RX_EN(i);
		rx->channel.power = true;
		rx->channel.enabled = true;
		rx->channel.nco_freq = 0;
tx:
		/* tx validations */
		if (!ADRV9001_BF_EQUAL(phy->curr_profile->tx.txInitChannelMask, tx_channels[i]))
			continue;

		/* check @tx_only comments in adrv9002.h to better understand the next checks */
		if (ssi_type != tx_cfg[i].txSsiConfig.ssiType) {
			dev_err(&phy->spi->dev, "SSI interface mismatch. PHY=%d, TX%d=%d\n",
				ssi_type, i + 1,  tx_cfg[i].txSsiConfig.ssiType);
			return -EINVAL;
		} else if (tx_cfg[i].txSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
			dev_err(&phy->spi->dev, "SSI interface Long Strobe not supported\n");
			return -EINVAL;
		} else if (ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS && !tx_cfg[i].txSsiConfig.ddrEn) {
			dev_err(&phy->spi->dev, "TX%d: Single Data Rate port not supported for LVDS\n",
				i + 1);
			return -EINVAL;
		} else if (phy->rx2tx2) {
			if (!phy->tx_only && !phy->rx_channels[0].channel.enabled) {
				/*
				 * pretty much means that in this case either all channels are
				 * disabled, which obviously does not make sense, or RX1 must
				 * be enabled...
				 */
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d cannot be enabled while RX1 is disabled",
					i + 1);
				return -EINVAL;
			} else if (i && !phy->tx_channels[0].channel.enabled) {
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d cannot be enabled while TX1 is disabled",
					i + 1);
				return -EINVAL;
			} else if (!phy->tx_only &&
				   tx_cfg[i].txInputRate_Hz != rx_cfg[0].profile.rxOutputRate_Hz) {
				/*
				 * pretty much means that in this case, all ports must have
				 * the same rate. We match against RX1 since RX2 can be disabled
				 * even if it does not make much sense to disable it in rx2tx2 mode
				 */
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d rate=%u must be equal to RX1, rate=%u\n",
					i + 1, tx_cfg[i].txInputRate_Hz,
					rx_cfg[0].profile.rxOutputRate_Hz);
				return -EINVAL;
			} else if (phy->tx_only && i &&
				   tx_cfg[i].txInputRate_Hz != tx_cfg[0].txInputRate_Hz) {
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d rate=%u must be equal to TX1, rate=%u\n",
					i + 1, tx_cfg[i].txInputRate_Hz, tx_cfg[0].txInputRate_Hz);
				return -EINVAL;
			}
		} else if (!phy->tx_only && !rx->channel.enabled) {
			dev_err(&phy->spi->dev, "TX%d cannot be enabled while RX%d is disabled",
				i + 1, i + 1);
		} else if (!phy->tx_only &&
			   tx_cfg[i].txInputRate_Hz != rx_cfg[i].profile.rxOutputRate_Hz) {
			dev_err(&phy->spi->dev, "TX%d rate=%u must be equal to RX%d, rate=%u\n",
				i + 1, tx_cfg[i].txInputRate_Hz, i + 1,
				rx_cfg[i].profile.rxOutputRate_Hz);
			return -EINVAL;
		}

		dev_dbg(&phy->spi->dev, "TX%d enabled\n", i);
		pos |= ADRV9002_TX_EN(i);
		tx->channel.power = true;
		tx->channel.enabled = true;
		tx->channel.nco_freq = 0;
	}

	phy->init_cals.chanInitCalMask[0] = adrv9002_init_cals_mask[pos][0];
	phy->init_cals.chanInitCalMask[1] = adrv9002_init_cals_mask[pos][1];

	dev_dbg(&phy->spi->dev, "pos: %u, Chan1:%X, Chan2:%X", pos,
		phy->init_cals.chanInitCalMask[0],
		phy->init_cals.chanInitCalMask[1]);

	return 0;
}

static int adrv9002_power_mgmt_config(struct adrv9002_rf_phy *phy)
{
	int ret;
	struct adi_adrv9001_PowerManagementSettings power_mgmt = {
		.ldoPowerSavingModes = {
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1, ADI_ADRV9001_LDO_POWER_SAVING_MODE_1,
			ADI_ADRV9001_LDO_POWER_SAVING_MODE_1
		}
	};

	ret = adi_adrv9001_powermanagement_Configure(phy->adrv9001, &power_mgmt);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_digital_init(struct adrv9002_rf_phy *phy)
{
	int ret;
	u8 tx_mask = 0;
	int c;
	/*
	 * There's still no way of getting the gain table type from the profile. We
	 * always get the correction one (which was the one we were using already).
	 * There were some talks and this might change in future versions of the DDAPI so,
	 * let's force this to correction for now and wait a few release cycles for
	 * proper support. If we do not get it, we might just add a devicetree attribute
	 * or some runtime sysfs attr. Not ideal but we won't have any choice if we can't
	 * get this info from the profile.
	 */
	adi_adrv9001_RxGainTableType_e t_type = ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE;
	adi_adrv9001_RxChannelCfg_t *rx_cfg = phy->curr_profile->rx.rxChannelCfg;

	ret = adi_adrv9001_arm_AhbSpiBridge_Enable(phy->adrv9001);
	if (ret)
		return adrv9002_dev_err(phy);

	/*
	 * If we find a custom stream, we will load that. Otherwise we will load the default one.
	 * Note that if we are in the middle of filling @phy->stream_buf with a new stream and we
	 * get somehow here, the default one will be used. Either way, after filling the stream, we
	 * __must__ write a new profile which will get us here again and we can then load then new
	 * stream.
	 */
	if (phy->stream_size == ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES)
		ret = adi_adrv9001_Stream_Image_Write(phy->adrv9001, 0, phy->stream_buf,
						      phy->stream_size,
						      ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252);
	else
		ret = adi_adrv9001_Utilities_StreamImage_Load(phy->adrv9001, "Navassa_Stream.bin",
					ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252);
	if (ret)
		return adrv9002_dev_err(phy);

	/* program arm firmware */
	ret = adi_adrv9001_Utilities_ArmImage_Load(phy->adrv9001, "Navassa_EvaluationFw.bin",
					ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_arm_Profile_Write(phy->adrv9001, phy->curr_profile);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_arm_PfirProfiles_Write(phy->adrv9001, phy->curr_profile);
	if (ret)
		return adrv9002_dev_err(phy);

	/* Load gain tables */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[c];
		struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
		adi_adrv9001_RxProfile_t *p = &rx_cfg[c].profile;

		if (tx->channel.enabled) {
			ret = adi_adrv9001_Utilities_RxGainTable_Load(phy->adrv9001, ADI_ORX,
								      "ORxGainTable.csv",
								      rx->channel.number,
								      &p->lnaConfig, t_type);
			if (ret)
				return adrv9002_dev_err(phy);
		}

		if (tx->channel.enabled)
			tx_mask |= tx->channel.number;

		if (!rx->channel.enabled)
			continue;

		ret = adi_adrv9001_Utilities_RxGainTable_Load(phy->adrv9001, ADI_RX,
							      "RxGainTable.csv", rx->channel.number,
							      &p->lnaConfig, t_type);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	if (tx_mask) {
		ret = adi_adrv9001_Utilities_TxAttenTable_Load(phy->adrv9001, "TxAttenTable.csv",
							       tx_mask);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	ret = adrv9002_power_mgmt_config(phy);
	if (ret)
		return ret;

	ret = adi_adrv9001_arm_Start(phy->adrv9001);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_arm_StartStatus_Check(phy->adrv9001, 5000000);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

/*
 * All of these structures are taken from TES when exporting the default profile to C code. Consider
 * about having all of these configurable through devicetree.
 */
static int adrv9002_radio_init(struct adrv9002_rf_phy *phy)
{
	int ret;
	int chan;
	u8 channel_mask = (phy->curr_profile->tx.txInitChannelMask |
			   phy->curr_profile->rx.rxInitChannelMask) & 0xFF;
	struct adi_adrv9001_PllLoopFilterCfg pll_loop_filter = {
		.effectiveLoopBandwidth_kHz = 0,
		.loopBandwidth_kHz = 300,
		.phaseMargin_degrees = 60,
		.powerScale = 5
	};
	struct adi_adrv9001_Carrier carrier = {0};

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_LO1, &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_LO2, &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_AUX, &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	for (chan = 0; chan < ADRV9002_CHANN_MAX; chan++) {
		struct adrv9002_chan *tx = &phy->tx_channels[chan].channel;
		struct adrv9002_chan *rx = &phy->rx_channels[chan].channel;

		if (!rx->enabled)
			goto tx;

		/*
		 * For some low rate profiles, the intermediate frequency is non 0.
		 * In these cases, forcing it 0, will cause a firmware error. Hence, we need to
		 * read what we have and make sure we just change the carrier frequency...
		 */
		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, ADI_RX, rx->number,
							 &carrier);
		if (ret)
			return adrv9002_dev_err(phy);

		carrier.carrierFrequency_Hz = 2400000000ULL;
		ret = adi_adrv9001_Radio_Carrier_Configure(phy->adrv9001, ADI_RX, rx->number,
							   &carrier);
		if (ret)
			return adrv9002_dev_err(phy);

tx:
		if (!tx->enabled)
			continue;

		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, ADI_TX, tx->number,
							 &carrier);
		if (ret)
			return adrv9002_dev_err(phy);

		carrier.carrierFrequency_Hz = 2450000000ULL;
		ret = adi_adrv9001_Radio_Carrier_Configure(phy->adrv9001, ADI_TX, tx->number,
							   &carrier);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	ret = adi_adrv9001_arm_System_Program(phy->adrv9001, channel_mask);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_setup(struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_Device *adrv9001_device = phy->adrv9001;
	u8 init_cals_error = 0;
	int ret;
	adi_adrv9001_ChannelState_e init_state;
	struct adi_adrv9001_SpiSettings spi = {
		.msbFirst = 1,
		.enSpiStreaming = 0,
		.autoIncAddrUp = 1,
		.fourWireMode = 1,
		.cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
	};

	/* in TDD we cannot start with all ports enabled as RX/TX cannot be on at the same time */
	if (phy->curr_profile->sysConfig.duplexMode == ADI_ADRV9001_TDD_MODE)
		init_state = ADI_ADRV9001_CHANNEL_PRIMED;
	else
		init_state = ADI_ADRV9001_CHANNEL_RF_ENABLED;

	adi_common_ErrorClear(&phy->adrv9001->common);
	ret = adi_adrv9001_HwOpen(adrv9001_device, &spi);
	if (ret)
		return adrv9002_dev_err(phy);

	/* compute init call and does some profile validations... */
	ret = adrv9002_compute_init_cals(phy);
	if (ret)
		return ret;

	adrv9002_log_enable(&adrv9001_device->common);

	ret = adi_adrv9001_InitAnalog(adrv9001_device, phy->curr_profile,
				      ADI_ADRV9001_DEVICECLOCKDIVISOR_2);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adrv9002_digital_init(phy);
	if (ret)
		return ret;

	ret = adrv9002_radio_init(phy);
	if (ret)
		return ret;

	/* should be done before init calibrations */
	ret = adrv9002_tx_set_dac_full_scale(phy);
	if (ret)
		return ret;

	ret = adi_adrv9001_cals_InitCals_Run(adrv9001_device, &phy->init_cals,
					     60000, &init_cals_error);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adrv9001_rx_path_config(phy, init_state);
	if (ret)
		return ret;

	ret = adrv9002_tx_path_config(phy, init_state);
	if (ret)
		return ret;

	/* unmask IRQs */
	ret = adi_adrv9001_gpio_GpIntMask_Set(adrv9001_device, ~ADRV9002_IRQ_MASK);
	if (ret)
		return adrv9002_dev_err(phy);

	return adrv9002_dgpio_config(phy);
}

int adrv9002_intf_change_delay(struct adrv9002_rf_phy *phy, const int channel, u8 clk_delay,
			       u8 data_delay, const bool tx, const adi_adrv9001_SsiType_e ssi_type)
{
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};
	int ret;

	dev_dbg(&phy->spi->dev, "Set intf delay clk:%u, d:%u, tx:%d c:%d\n", clk_delay,
		data_delay, tx, channel);

	if (tx) {
		delays.txClkDelay[channel] = clk_delay;
		delays.txIDataDelay[channel] = data_delay;
		delays.txQDataDelay[channel] = data_delay;
		delays.txStrobeDelay[channel] = data_delay;
		if (phy->rx2tx2 && !channel) {
			delays.txClkDelay[channel + 1] = clk_delay;
			delays.txIDataDelay[channel + 1] = data_delay;
			delays.txQDataDelay[channel + 1] = data_delay;
			delays.txStrobeDelay[channel + 1] = data_delay;
		}
	} else {
		delays.rxClkDelay[channel] = clk_delay;
		delays.rxIDataDelay[channel] = data_delay;
		delays.rxQDataDelay[channel] = data_delay;
		delays.rxStrobeDelay[channel] = data_delay;
		if (phy->rx2tx2) {
			delays.rxClkDelay[channel + 1] = clk_delay;
			delays.rxIDataDelay[channel + 1] = data_delay;
			delays.rxQDataDelay[channel + 1] = data_delay;
			delays.rxStrobeDelay[channel + 1] = data_delay;
		}
	}

	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, ssi_type, &delays);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

int adrv9002_check_tx_test_pattern(struct adrv9002_rf_phy *phy, const int chann,
				   const adi_adrv9001_SsiType_e ssi_type)
{
	int ret;
	struct adrv9002_chan *chan = &phy->tx_channels[chann].channel;
	adi_adrv9001_SsiTestModeData_e test_data = ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ?
						ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE :
						ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7;
	struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};
	struct adi_adrv9001_TxSsiTestModeStatus status = {0};

	cfg.testData = test_data;

	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001, chan->number, ssi_type,
							  ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							  &cfg, &status);
	if (ret)
		return adrv9002_dev_err(phy);

	dev_dbg(&phy->spi->dev, "[c%d]: d_e:%u, f_f:%u f_e:%u, s_e:%u", chan->number, status.dataError,
		status.fifoFull, status.fifoEmpty, status.strobeAlignError);

	/* only looking for data errors for now */
	if (status.dataError)
		return 1;

	if (!phy->rx2tx2 || chann)
		return 0;

	chan = &phy->tx_channels[chann + 1].channel;
	if (!chan->enabled)
		return 0;

	memset(&status, 0, sizeof(status));
	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001, chan->number, ssi_type,
							  ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							  &cfg, &status);
	if (ret)
		return adrv9002_dev_err(phy);

	dev_dbg(&phy->spi->dev, "[c%d]: d_e:%u, f_f:%u f_e:%u, s_e:%u", chan->number,
		status.dataError, status.fifoFull, status.fifoEmpty, status.strobeAlignError);

	if (status.dataError)
		return 1;

	return 0;
}

int adrv9002_intf_test_cfg(struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop, const adi_adrv9001_SsiType_e ssi_type)
{
	int ret;
	struct adrv9002_chan *chan;

	dev_dbg(&phy->spi->dev, "cfg test stop:%u, ssi:%d, c:%d, tx:%d\n", stop, ssi_type, chann,
		tx);

	if (tx) {
		struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};
		chan = &phy->tx_channels[chann].channel;

		if (stop)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL;
		else if (ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
			/*
			 * Some low rate profiles don't play well with prbs15. The reason is
			 * still unclear. We suspect that the chip error checker might have
			 * some time constrains and cannot reliable validate prbs15 full
			 * sequences in the test time. Using a shorter sequence fixes the
			 * problem...
			 */
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7;
		else
			/* CMOS */
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE;

		ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, chan->number, ssi_type,
							     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							     &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

		if (!phy->rx2tx2 || chann)
			return 0;

		chan = &phy->tx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, chan->number, ssi_type,
							     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							     &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

	} else {
		struct adi_adrv9001_RxSsiTestModeCfg cfg = {0};
		chan = &phy->rx_channels[chann].channel;

		if (stop)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL;
		else if (ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;
		else
			/* CMOS */
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE;

		ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, chan->number, ssi_type,
							     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							     &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

		if (!phy->rx2tx2)
			return 0;

		/* on rx2tx2 RX1 must be enabled so we are fine in assuming chan=0 at this point */
		chan = &phy->rx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, chan->number, ssi_type,
							     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							     &cfg);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	return 0;
}

static int adrv9002_intf_tuning(struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};
	int ret;
	u8 clk_delay, data_delay;
	struct adrv9002_chan *chan;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int i;

	for (i = 0; i < ARRAY_SIZE(phy->rx_channels); i++) {
		chan = &phy->rx_channels[i].channel;

		if (!chan->enabled)
			continue;

		ret = adrv9002_axi_intf_tune(phy, false, i, ssi_type, &clk_delay, &data_delay);
		if (ret)
			return ret;

		dev_dbg(&phy->spi->dev, "RX: Got clk: %u, data: %u\n", clk_delay, data_delay);
		delays.rxClkDelay[i] = clk_delay;
		delays.rxIDataDelay[i] = data_delay;
		delays.rxQDataDelay[i] = data_delay;
		delays.rxStrobeDelay[i] = data_delay;
		/*
		 * In rx2tx2 we should treat RX1/RX2 as the same. Hence, we will run
		 * the test simultaneosly for both ports and configure the same delays.
		 * Moreover rx2 cannot be enabled while rx1 is disabled...
		 */
		if (phy->rx2tx2) {
			/* set RX2 delays */
			delays.rxClkDelay[i + 1] = clk_delay;
			delays.rxIDataDelay[i + 1] = data_delay;
			delays.rxQDataDelay[i + 1] = data_delay;
			delays.rxStrobeDelay[i + 1] = data_delay;
			break;
		}
	}

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		chan = &phy->tx_channels[i].channel;

		if (!chan->enabled)
			continue;

		ret = adrv9002_axi_intf_tune(phy, true, i, ssi_type, &clk_delay, &data_delay);
		if (ret)
			return ret;

		dev_dbg(&phy->spi->dev, "TX: Got clk: %u, data: %u\n", clk_delay, data_delay);
		delays.txClkDelay[i] = clk_delay;
		delays.txIDataDelay[i] = data_delay;
		delays.txQDataDelay[i] = data_delay;
		delays.txStrobeDelay[i] = data_delay;
		/*
		 * In rx2tx2 we should treat TX1/TX2 as the same. Hence, we will run
		 * the test simultaneosly for both ports and configure the same delays.
		 */
		if (phy->rx2tx2) {
			if (!i) {
				/* set TX2 delays */
				delays.txClkDelay[i + 1] = clk_delay;
				delays.txIDataDelay[i + 1] = data_delay;
				delays.txQDataDelay[i + 1] = data_delay;
				delays.txStrobeDelay[i + 1] = data_delay;
			}
			break;
		}
	}

	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, ssi_type, &delays);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static void adrv9002_cleanup(struct adrv9002_rf_phy *phy)
{
	int i;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		phy->rx_channels[i].channel.enabled = 0;
		phy->tx_channels[i].channel.enabled = 0;
	}

	memset(&phy->adrv9001->devStateInfo, 0,
	       sizeof(phy->adrv9001->devStateInfo));
}

#define ADRV9002_OF_U32_GET_VALIDATE(dev, node, key, def, min, max,	\
				     val, mandatory) ({			\
	u32 tmp;							\
	int ret, __ret = 0;						\
	const char *__key = key;					\
									\
	val = def;							\
	ret = of_property_read_u32(node, __key, &tmp);			\
	if (!ret) {							\
		if (tmp < (min) || tmp > (max)) {			\
			dev_err(dev, "Invalid value(%d) for \"%s\"\n",	\
				tmp, __key);				\
			__ret = -EINVAL;				\
		} else {						\
			val = tmp;					\
		}							\
	} else if (mandatory) {						\
		dev_err(dev, "Missing mandatory prop: \"%s\"\n",	\
			__key);						\
		__ret = ret;						\
	}								\
									\
	__ret;								\
})

#define OF_ADRV9002_PINCTL(key, def, min, max, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, pinctlr, key, def, min, \
				     max, val, mandatory)

static int adrv9002_parse_tx_pin_dt(struct adrv9002_rf_phy *phy,
				    struct device_node *node,
				    struct adrv9002_tx_chan *tx)
{
	struct device_node *pinctlr;
	int ret;

	pinctlr = of_parse_phandle(node, "adi,pinctrl", 0);
	if (!pinctlr)
		return 0;

	tx->pin_cfg = devm_kzalloc(&phy->spi->dev, sizeof(*tx->pin_cfg),
				   GFP_KERNEL);
	if (!tx->pin_cfg) {
		of_node_put(pinctlr);
		return -ENOMEM;
	}

	ret = OF_ADRV9002_PINCTL("adi,increment-pin", 0, ADRV9002_DGPIO_MIN,
				 ADRV9002_DGPIO_MAX, tx->pin_cfg->incrementPin,
				 true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_PINCTL("adi,decrement-pin", 0, ADRV9002_DGPIO_MIN,
				 ADRV9002_DGPIO_MAX, tx->pin_cfg->decrementPin,
				 true);
	if (ret)
		goto of_pinctrl_put;

	/* set the pins correct for the API */
	tx->pin_cfg->incrementPin++;
	tx->pin_cfg->decrementPin++;

	ret = OF_ADRV9002_PINCTL("adi,step-size-mdB", 50, 50, 1550,
				 tx->pin_cfg->stepSize_mdB, false);
	/* extra validation since the value needs to be multiple of 50 */
	if (tx->pin_cfg->stepSize_mdB % 50 != 0) {
		dev_err(&phy->spi->dev, "adi,step-size-mdB must be multiple of 50\n");
		ret = -EINVAL;
	}

of_pinctrl_put:
	of_node_put(pinctlr);
	return ret;
}

static int adrv9002_parse_tx_dt(struct adrv9002_rf_phy *phy,
				struct device_node *node, const int channel)
{
	struct adrv9002_tx_chan *tx = &phy->tx_channels[channel];

	if (of_property_read_bool(node, "adi,dac-full-scale-boost"))
		tx->dac_boost_en = true;

	return adrv9002_parse_tx_pin_dt(phy, node, tx);
}

static int adrv9002_parse_rx_pinctl_dt(struct adrv9002_rf_phy *phy,
				       const struct device_node *node,
				       struct adrv9002_rx_chan *rx)
{
	struct device_node *pinctlr;
	int ret;

	/* get pinctrl properties if any */
	pinctlr = of_parse_phandle(node, "adi,pinctrl", 0);
	if (!pinctlr)
		return 0;

	rx->pin_cfg = devm_kzalloc(&phy->spi->dev, sizeof(*rx->pin_cfg),
				   GFP_KERNEL);
	if (!rx->pin_cfg) {
		ret = -ENOMEM;
		goto of_pinctrl_put;
	}

	ret = OF_ADRV9002_PINCTL("adi,increment-step-size", 1, 1, 7,
				 rx->pin_cfg->incrementStepSize, false);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_PINCTL("adi,decrement-step-size", 1, 1, 7,
				 rx->pin_cfg->decrementStepSize, false);
	if (ret)
		goto of_pinctrl_put;

	/*
	 * Get gpios. This are mandatory properties. It makes no sense to
	 * pin crtl gain and no pins congigured to control it!
	 */
	ret = OF_ADRV9002_PINCTL("adi,increment-pin", 0, ADRV9002_DGPIO_MIN,
				 ADRV9002_DGPIO_MAX, rx->pin_cfg->incrementPin,
				 true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_PINCTL("adi,decrement-pin", 0, ADRV9002_DGPIO_MIN,
				 ADRV9002_DGPIO_MAX, rx->pin_cfg->decrementPin,
				 true);
	if (ret)
		goto of_pinctrl_put;

	rx->pin_cfg->incrementPin++;
	rx->pin_cfg->decrementPin++;

of_pinctrl_put:
	of_node_put(pinctlr);
	return ret;
}

#define AGC_OFFSETOF(member)	\
	offsetof(struct adi_adrv9001_GainControlCfg, member)

static const struct {
	const char *key;
	const u32 __off;
	const u32 def;
	const u32 min;
	const u32 max;
	const u8 size;
} of_agc_props[] = {
	{"adi,peak-wait-time", AGC_OFFSETOF(peakWaitTime), 4, 0, 31, 1},
	{"adi,gain-update-counter",
	 AGC_OFFSETOF(gainUpdateCounter), 11520, 0, 4194303, 4},
	{"adi,attack-delax-us",
	 AGC_OFFSETOF(attackDelay_us), 10, 0, 63, 1},
	{"adi,slow-loop-settling-delay",
	 AGC_OFFSETOF(slowLoopSettlingDelay), 16, 0, 127, 1},
	{"adi,change-gain-threshold-high",
	 AGC_OFFSETOF(changeGainIfThreshHigh), 3, 0, 3, 1},
	{"adi,agc-mode", AGC_OFFSETOF(agcMode), 1, 0, 1, 1},
	{"adi,reset-on-rx-on-gain-index",
	 AGC_OFFSETOF(resetOnRxonGainIndex), ADRV9002_RX_MAX_GAIN_IDX, ADRV9002_RX_MIN_GAIN_IDX,
	 ADRV9002_RX_MAX_GAIN_IDX, 1},
	/* power detector */
	{"adi,power-under-range-high-threshold",
	 AGC_OFFSETOF(power.underRangeHighPowerThresh), 10, 0, 127, 1},
	{"adi,power-under-range-low-threshold",
	 AGC_OFFSETOF(power.underRangeLowPowerThresh), 4, 0, 15, 1},
	{"adi,power-under-range-high-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeHighPowerGainStepRecovery), 2, 0, 31, 1},
	{"adi,power-under-range-low-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeLowPowerGainStepRecovery), 4, 0, 31, 1},
	{"adi,power-measurement-duration",
	 AGC_OFFSETOF(power.powerMeasurementDuration), 10, 0, 31, 1},
	{"adi,power-measurement-delay",
	 AGC_OFFSETOF(power.powerMeasurementDelay), 2, 0, 255, 1},
	{"adi,power-rx-tdd-measurement-duration",
	 AGC_OFFSETOF(power.rxTddPowerMeasDuration), 0, 0, 65535, 2},
	{"adi,power-rx-tdd-measurement-delay",
	 AGC_OFFSETOF(power.rxTddPowerMeasDelay), 0, 0, 65535, 2},
	{"adi,power-over-range-high-threshold",
	 AGC_OFFSETOF(power.overRangeHighPowerThresh), 0, 0, 15, 1},
	{"adi,power-over-range-low-threshold",
	 AGC_OFFSETOF(power.overRangeLowPowerThresh), 7, 0, 127, 1},
	{"adi,power-over-range-high-gain-step-attack",
	 AGC_OFFSETOF(power.overRangeHighPowerGainStepAttack), 4, 0, 31, 1},
	{"adi,power-over-range-low-gain-step-attack",
	 AGC_OFFSETOF(power.overRangeLowPowerGainStepAttack), 4, 0, 31, 1},
	/* peak detector */
	{"adi,peak-agc-under-range-low-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeLowInterval), 50, 0, 65535, 2},
	{"adi,peak-agc-under-range-mid-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeMidInterval), 2, 0, 63, 1},
	{"adi,peak-agc-under-range-high-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeHighInterval), 4, 0, 63, 1},
	{"adi,peak-apd-high-threshold",
	 AGC_OFFSETOF(peak.apdHighThresh), 21, 0, 63, 1},
	{"adi,peak-apd-low-threshold",
	 AGC_OFFSETOF(peak.apdLowThresh), 12, 0, 63, 1},
	{"adi,peak-apd-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-apd-lower-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdLowerThreshPeakExceededCount), 3, 0, 255, 1},
	{"adi,peak-apd-gain-step-attack",
	 AGC_OFFSETOF(peak.apdGainStepAttack), 2, 0, 31, 1},
	{"adi,peak-apd-gain-step-recovery",
	 AGC_OFFSETOF(peak.apdGainStepRecovery), 0, 0, 31, 1},
	{"adi,peak-hb-overload-duration-count",
	 AGC_OFFSETOF(peak.hbOverloadDurationCount), 1, 0, 7, 1},
	{"adi,peak-hb-overload-threshold-count",
	 AGC_OFFSETOF(peak.hbOverloadThreshCount), 1, 1, 15, 1},
	{"adi,peak-hb-high-threshold",
	 AGC_OFFSETOF(peak.hbHighThresh), 13044, 0, 16383, 2},
	{"adi,peak-hb-under-range-low-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeLowThresh), 5826, 0, 16383, 2},
	{"adi,peak-hb-under-range-mid-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeMidThresh), 8230, 0, 16383, 2},
	{"adi,peak-hb-under-range-high-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThresh), 7335, 0, 16383, 2},
	{"adi,peak-hb-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-hb-under-range-high-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThreshExceededCount), 3, 0, 255, 1},
	{"adi,peak-hb-gain-step-high-recover",
	 AGC_OFFSETOF(peak.hbGainStepHighRecovery), 2, 0, 31, 1},
	{"adi,peak-hb-gain-step-low-recovery",
	 AGC_OFFSETOF(peak.hbGainStepLowRecovery), 6, 0, 31, 1},
	{"adi,peak-hb-gain-step-mid-recovery",
	 AGC_OFFSETOF(peak.hbGainStepMidRecovery), 4, 0, 31, 1},
	{"adi,peak-hb-gain-step-attack",
	 AGC_OFFSETOF(peak.hbGainStepAttack), 2, 0, 31, 1},
	{"adi,peak-hb-overload-power-mode",
	 AGC_OFFSETOF(peak.hbOverloadPowerMode), 0, 0, 1, 1},
	{"adi,peak-hb-under-range-mid-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeMidThreshExceededCount), 3, 0, 255, 1},
	{"adi,peak-hb-under-range-low-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeLowThreshExceededCount), 3, 0, 255, 1},
};

static int adrv9002_parse_rx_agc_dt(struct adrv9002_rf_phy *phy,
				    const struct device_node *node,
				    struct adrv9002_rx_chan *rx)
{
	struct device_node *agc;
	int ret, prop;

#define ADRV9002_OF_AGC_PIN(key, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, key, \
				     ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED, \
				     min, max, val, false)

	/* get AGC properties if any */
	agc = of_parse_phandle(node, "adi,agc", 0);
	if (!agc)
		return 0;

	rx->agc = devm_kzalloc(&phy->spi->dev, sizeof(*rx->agc), GFP_KERNEL);
	if (!rx->agc) {
		of_node_put(agc);
		return -ENOMEM;
	}

	for (prop = 0; prop < ARRAY_SIZE(of_agc_props); prop++) {
		u32 temp, __off = of_agc_props[prop].__off;

		ret = of_property_read_u32(agc, of_agc_props[prop].key, &temp);
		if (ret) {
			temp = of_agc_props[prop].def;
		} else if (temp < of_agc_props[prop].min ||
			   temp > of_agc_props[prop].max) {
			dev_err(&phy->spi->dev, "%s not in valid range [%d %d]\n",
				of_agc_props[prop].key, of_agc_props[prop].min,
				of_agc_props[prop].max);
			of_node_put(agc);
			return -EINVAL;
		}

		/* assign value */
		switch (of_agc_props[prop].size) {
		case 1:
			*(u8 *)((void *)rx->agc + __off) = temp;
			break;
		case 2:
			*(u16 *)((void *)rx->agc + __off) = temp;
			break;
		case 4:
			*(u32 *)((void *)rx->agc + __off) = temp;
			break;
		};
	}

	/* boolean properties */
	if (of_property_read_bool(agc, "adi,low-threshold-prevent-gain-inc"))
		rx->agc->lowThreshPreventGainInc = true;

	if (of_property_read_bool(agc, "adi,sync-pulse-gain-counter-en"))
		rx->agc->enableSyncPulseForGainCounter = true;

	if (of_property_read_bool(agc, "adi,fast-recovery-loop-en"))
		rx->agc->enableFastRecoveryLoop = true;

	if (of_property_read_bool(agc, "adi,reset-on-rx-on"))
		rx->agc->resetOnRxon = true;

	if (of_property_read_bool(agc, "adi,power-measurement-en"))
		rx->agc->power.powerEnableMeasurement = true;

	if (of_property_read_bool(agc, "adi,peak-hb-overload-en"))
		rx->agc->peak.enableHbOverload = true;

	/* check if there are any gpios */
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-high-thres-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->power.feedback_inner_high_inner_low);
	if (ret)
		goto out;

	/* feedback pins*/
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-low-thres-gain-change",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->power.feedback_apd_high_apd_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-high-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->peak.feedback_apd_low_hb_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-low-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->peak.feedback_apd_high_hb_high);
out:
	of_node_put(agc);
	return ret;
}

static int adrv9002_parse_rx_dt(struct adrv9002_rf_phy *phy,
				const struct device_node *node,
				const int channel)
{
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	int ret;
	u32 min_gain, max_gain;

	ret = adrv9002_parse_rx_agc_dt(phy, node, rx);
	if (ret)
		return ret;

	ret = adrv9002_parse_rx_pinctl_dt(phy, node, rx);
	if (ret)
		return ret;

	/* check min/max gain and assign to pinctrl and agc if there */
#define ADRV9002_OF_RX_OPTIONAL(key, def, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, key, def, \
				     min, max, val, false)

	ret = ADRV9002_OF_RX_OPTIONAL("adi,min-gain-index", ADRV9002_RX_MIN_GAIN_IDX,
				      ADRV9002_RX_MIN_GAIN_IDX,
				      ADRV9002_RX_MAX_GAIN_IDX, min_gain);
	if (ret)
		return ret;

	ret = ADRV9002_OF_RX_OPTIONAL("adi,max-gain-index", ADRV9002_RX_MAX_GAIN_IDX,
				      min_gain, ADRV9002_RX_MAX_GAIN_IDX, max_gain);
	if (ret)
		return ret;

	if (rx->agc) {
		rx->agc->maxGainIndex = max_gain;
		rx->agc->minGainIndex = min_gain;
	}

	if (rx->pin_cfg) {
		rx->pin_cfg->maxGainIndex = max_gain;
		rx->pin_cfg->minGainIndex = min_gain;
	}

	return 0;
}

static int adrv9002_parse_dt(struct adrv9002_rf_phy *phy)
{
	int ret, idx = 0;
	struct device_node *of_channels, *of_gpios = NULL;
	struct device_node *parent = phy->spi->dev.of_node, *child;

	/* handle channels */
	of_channels = of_get_child_by_name(parent, "adi,channels");
	if (!of_channels)
		goto of_gpio;

	for_each_available_child_of_node(of_channels, child) {
		u32 chann, port;

		ret = of_property_read_u32(child, "reg", &chann);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No reg property defined for channel\n");
			goto of_child_put;
		} else if (chann > 1) {
			dev_err(&phy->spi->dev,
				"Invalid value for channel: %d\n", chann);
			ret = -EINVAL;
			goto of_child_put;
		}

		ret = of_property_read_u32(child, "adi,port", &port);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No port property defined for channel\n");
			goto of_child_put;
		}

		switch (port) {
		case ADI_TX:
			ret = adrv9002_parse_tx_dt(phy, child, chann);
			break;
		case ADI_RX:
			ret = adrv9002_parse_rx_dt(phy, child, chann);
			break;
		default:
			dev_err(&phy->spi->dev, "Unknown port: %d\n", port);
			ret = -EINVAL;
			break;
		};

		if (ret)
			goto of_child_put;
	}

of_gpio:
	/* handle gpios */
	of_gpios = of_get_child_by_name(parent, "adi,gpios");
	if (!of_gpios)
		goto of_channels_put;

	phy->ngpios = of_get_child_count(of_gpios);
	if (!phy->ngpios)
		goto of_gpio_put;

	phy->adrv9002_gpios = devm_kcalloc(&phy->spi->dev, phy->ngpios,
					   sizeof(*phy->adrv9002_gpios),
					   GFP_KERNEL);
	if (!phy->adrv9002_gpios) {
		ret = -ENOMEM;
		goto of_gpio_put;
	}

	for_each_available_child_of_node(of_gpios, child) {
		u32 gpio, polarity, master, signal;

		ret = of_property_read_u32(child, "reg", &gpio);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No reg property defined for gpio\n");
			goto of_child_put;
		} else if (gpio >= ADI_ADRV9001_GPIO_ANALOG_11) {
			dev_err(&phy->spi->dev,
				"Invalid gpio number: %d\n", gpio);
			ret = -EINVAL;
			goto of_child_put;
		}
		/* index 0 is not valid */
		phy->adrv9002_gpios[idx].gpio.pin = gpio + 1;

		ret = of_property_read_u32(child, "adi,signal", &signal);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No adi,signal property defined for gpio%d\n",
				gpio);
			goto of_child_put;
		} else if (signal > ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL2) {
			dev_err(&phy->spi->dev,
				"Invalid gpio signal: %d\n", signal);
			ret = -EINVAL;
			goto of_child_put;
		}
		phy->adrv9002_gpios[idx].signal = signal;

		ret = of_property_read_u32(child, "adi,polarity", &polarity);
		if (!ret) {
			if (polarity > ADI_ADRV9001_GPIO_POLARITY_INVERTED) {
				dev_err(&phy->spi->dev,
					"Invalid gpio polarity: %d\n",
					polarity);

				ret = -EINVAL;
				goto of_child_put;
			}
			phy->adrv9002_gpios[idx].gpio.polarity = polarity;
		}

		ret = of_property_read_u32(child, "adi,master", &master);
		if (!ret) {
			if (master != ADI_ADRV9001_GPIO_MASTER_ADRV9001 &&
			    master != ADI_ADRV9001_GPIO_MASTER_BBIC) {
				dev_err(&phy->spi->dev,
					"Invalid gpio master: %d\n",
					master);

				ret = -EINVAL;
				goto of_child_put;
			}
			phy->adrv9002_gpios[idx].gpio.master = master;
		} else {
			ret = 0;
		}

		idx++;
	}

of_child_put:
	of_node_put(child);
of_gpio_put:
	of_node_put(of_gpios);
of_channels_put:
	of_node_put(of_channels);
	return ret;
}

int adrv9002_init(struct adrv9002_rf_phy *phy, struct adi_adrv9001_Init *profile)
{
	int ret, c;

	adrv9002_cleanup(phy);
	/*
	 * Disable all the cores as it might interfere with init calibrations.
	 */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		/* rx */
		adrv9002_axi_interface_enable(phy, c, false, false);
		/* tx */
		adrv9002_axi_interface_enable(phy, c, true, false);
		if (phy->rx2tx2)
			break;
	}

	phy->curr_profile = profile;
	ret = adrv9002_setup(phy);
	if (ret) {
		/* try one more time */
		ret = adrv9002_setup(phy);
		if (ret) {
			adrv9002_cleanup(phy);
			return ret;
		}
	}

	adrv9002_set_clk_rates(phy);

	ret = adrv9002_ssi_configure(phy);
	if (ret)
		return ret;

	/* re-enable the cores */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		struct adrv9002_chan *rx = &phy->rx_channels[c].channel;
		struct adrv9002_chan *tx = &phy->tx_channels[c].channel;

		if (!rx->enabled)
			goto tx;

		adrv9002_axi_interface_enable(phy, c, false, true);
tx:
		if (tx->enabled)
			adrv9002_axi_interface_enable(phy, c, true, true);

		if (phy->rx2tx2)
			break;
	}

	return adrv9002_intf_tuning(phy);
}

static ssize_t adrv9002_stream_bin_write(struct file *filp, struct kobject *kobj,
					 struct bin_attribute *bin_attr, char *buf, loff_t off,
					 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);

	if (off + count <= bin_attr->size) {
		mutex_lock(&phy->lock);
		if (!off)
			phy->stream_size = 0;
		memcpy(phy->stream_buf + off, buf, count);
		phy->stream_size += count;
		mutex_unlock(&phy->lock);
	} else {
		dev_err(&phy->spi->dev, "Invalid stream image size:%lld!\n", count + off);
		return -EINVAL;
	}

	return count;
}

static ssize_t adrv9002_profile_bin_write(struct file *filp, struct kobject *kobj,
					  struct bin_attribute *bin_attr, char *buf, loff_t off,
					  size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	if (off == 0)
		memset(phy->bin_attr_buf, 0, bin_attr->size);

	memcpy(phy->bin_attr_buf + off, buf, count);

	if (!strnstr(phy->bin_attr_buf, "\n}", off + count))
		return count;

	dev_dbg(&phy->spi->dev, "%s:%d: size %lld\n", __func__, __LINE__,
		off + count);

	mutex_lock(&phy->lock);

	memset(&phy->profile, 0, sizeof(phy->profile));
	ret = adi_adrv9001_profileutil_Parse(phy->adrv9001, &phy->profile, phy->bin_attr_buf,
					     off + count);
	if (ret)
		goto out;

	ret = adrv9002_init(phy, &phy->profile);
out:
	mutex_unlock(&phy->lock);

	return (ret < 0) ? ret : count;
}

static int adrv9002_profile_load(struct adrv9002_rf_phy *phy, const char *profile)
{
	int ret;
	const struct firmware *fw;
	void *buf;

	ret = request_firmware(&fw, profile, &phy->spi->dev);
	if (ret)
		return ret;

	buf = kzalloc(fw->size, GFP_KERNEL);
	if (!buf) {
		release_firmware(fw);
		return -ENOMEM;
	}

	memcpy(buf, fw->data, fw->size);
	ret = adi_adrv9001_profileutil_Parse(phy->adrv9001, &phy->profile, buf, fw->size);
	if (ret)
		ret = adrv9002_dev_err(phy);
	release_firmware(fw);
	kfree(buf);

	return ret;
}

static void adrv9002_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void adrv9002_of_clk_del_provider(void *data)
{
	struct device *dev = data;

	of_clk_del_provider(dev->of_node);
}

static BIN_ATTR(stream_config, 0222, NULL, adrv9002_stream_bin_write, ADRV9002_STREAM_BINARY_SZ);

int adrv9002_post_init(struct adrv9002_rf_phy *phy)
{
	struct adi_common_ApiVersion api_version;
	struct adi_adrv9001_ArmVersion arm_version;
	struct adi_adrv9001_SiliconVersion silicon_version;
	struct adi_adrv9001_StreamVersion stream_version;
	int ret, c;
	struct spi_device *spi = phy->spi;
	struct iio_dev *indio_dev = phy->indio_dev;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	const char * const clk_names[NUM_ADRV9002_CLKS] = {
		[RX1_SAMPL_CLK] = "-rx1_sampl_clk",
		[RX2_SAMPL_CLK] = "-rx2_sampl_clk",
		[TX1_SAMPL_CLK] = "-tx1_sampl_clk",
		[TX2_SAMPL_CLK] = "-tx2_sampl_clk",
		[TDD1_INTF_CLK] = "-tdd1_intf_clk",
		[TDD2_INTF_CLK] = "-tdd2_intf_clk"
	};

	if (ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS) {
		ret = adrv9002_profile_load(phy, "Navassa_LVDS_profile.json");
		if (ret)
			return ret;
	}

	/* register channels clocks */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[c];
		struct adrv9002_chan *tx = &phy->tx_channels[c].channel;

		rx->channel.clk = adrv9002_clk_register(phy, clk_names[c],
							CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
							c);
		if (IS_ERR(rx->channel.clk))
			return PTR_ERR(rx->channel.clk);

		tx->clk = adrv9002_clk_register(phy, clk_names[c + TX1_SAMPL_CLK],
						CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
						c + TX1_SAMPL_CLK);
		if (IS_ERR(tx->clk))
			return PTR_ERR(tx->clk);

		rx->tdd_clk = adrv9002_clk_register(phy, clk_names[c + TDD1_INTF_CLK],
						    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
						    c + TDD1_INTF_CLK);
		if (IS_ERR(rx->tdd_clk))
			return PTR_ERR(rx->tdd_clk);

		if (phy->rx2tx2) {
			/* just point RX2/TX2 to RX1/TX1*/
			phy->rx_channels[c + 1].channel.clk = rx->channel.clk;
			phy->tx_channels[c + 1].channel.clk = tx->clk;
			break;
		}
	}

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = phy->n_clks;
	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_onecell_get,
				  &phy->clk_data);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adrv9002_of_clk_del_provider,
				       &spi->dev);
	if (ret)
		return ret;

	ret = adrv9002_init(phy, &phy->profile);
	if (ret)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adrv9002_phy_info;
	indio_dev->channels = adrv9002_phy_chan;
	indio_dev->num_channels = ARRAY_SIZE(adrv9002_phy_chan);

	if (spi->irq) {
		const unsigned long mask = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

		ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
						adrv9002_irq_handler, mask,
						indio_dev->name, indio_dev);
		if (ret)
			return ret;
	}

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		return ret;

	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "profile_config";
	phy->bin.attr.mode = 0200;
	phy->bin.write = adrv9002_profile_bin_write;
	phy->bin.size = 73728;

	phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev, phy->bin.size, GFP_KERNEL);
	if (!phy->bin_attr_buf)
		return -ENOMEM;

	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		return ret;

	phy->stream_buf = devm_kzalloc(&phy->spi->dev, ADRV9002_STREAM_BINARY_SZ, GFP_KERNEL);
	if (!phy->stream_buf)
		return -ENOMEM;

	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &bin_attr_stream_config);
	if (ret < 0)
		return ret;

	adi_adrv9001_ApiVersion_Get(phy->adrv9001, &api_version);
	adi_adrv9001_arm_Version(phy->adrv9001, &arm_version);
	adi_adrv9001_SiliconVersion_Get(phy->adrv9001, &silicon_version);
	adi_adrv9001_Stream_Version(phy->adrv9001, &stream_version);

	dev_info(&spi->dev,
		 "%s Rev %d.%d, Firmware %u.%u.%u.%u,  Stream %u.%u.%u.%u,  API version: %u.%u.%u successfully initialized",
		 indio_dev->name, silicon_version.major, silicon_version.minor,
		 arm_version.majorVer, arm_version.minorVer, arm_version.maintVer,
		 arm_version.rcVer, stream_version.majorVer, stream_version.minorVer,
		 stream_version.maintVer, stream_version.buildVer, api_version.major,
		 api_version.minor, api_version.patch);

	adrv9002_debugfs_create(phy, iio_get_debugfs_dentry(indio_dev));

	return 0;
}

static int adrv9002_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9002_rf_phy *phy;
	struct clk *clk = NULL;
	int ret, c;
	const int *id;

	id = of_device_get_match_data(&spi->dev);
	if (!id)
		return -EINVAL;

	clk = devm_clk_get(&spi->dev, "adrv9002_ext_refclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->spi_device_id = *id;
	if (phy->spi_device_id == ID_ADRV9002_RX2TX2)
		phy->rx2tx2 = true;

	mutex_init(&phy->lock);
	phy->adrv9001 = &phy->adrv9001_device;
	phy->hal.spi = spi;
	phy->adrv9001->common.devHalInfo = &phy->hal;

	ret = adrv9002_profile_load(phy, "Navassa_CMOS_profile.json");
	if (ret)
		return ret;

	/* initialize channel numbers here since these will never change */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		phy->rx_channels[c].channel.number = c + ADI_CHANNEL_1;
		phy->tx_channels[c].channel.number = c + ADI_CHANNEL_1;
	}

	phy->hal.reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->hal.reset_gpio))
		return PTR_ERR(phy->hal.reset_gpio);

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adrv9002_clk_disable, clk);
	if (ret)
		return ret;

	ret = adrv9002_parse_dt(phy);
	if (ret)
		return ret;

	if (phy->rx2tx2) {
		phy->ssi_sync = devm_gpiod_get(&spi->dev, "ssi-sync", GPIOD_OUT_LOW);
		if (IS_ERR(phy->ssi_sync))
			return PTR_ERR(phy->ssi_sync);
	}

	return adrv9002_register_axi_converter(phy);
}

static const int adrv9002_id = ID_ADRV9002;
static const int adrv9002_rx2tx2_id = ID_ADRV9002_RX2TX2;

static const struct of_device_id adrv9002_of_match[] = {
	{.compatible = "adi,adrv9002", .data = &adrv9002_id},
	{.compatible = "adi,adrv9002-rx2tx2", .data = &adrv9002_rx2tx2_id},
	{}
};

MODULE_DEVICE_TABLE(of, adrv9002_of_match);

static struct spi_driver adrv9002_driver = {
	.driver = {
		.name	= "adrv9002",
		.of_match_table = adrv9002_of_match,
	},
	.probe		= adrv9002_probe,
};
module_spi_driver(adrv9002_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9002 ADC");
MODULE_LICENSE("GPL v2");
