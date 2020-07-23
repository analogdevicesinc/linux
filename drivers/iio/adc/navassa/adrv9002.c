// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>

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
#include "adi_adrv9001_profile_types.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_txSettings_types.h"
#include "adi_adrv9001_utilities.h"
#include "adi_adrv9001_utilities_types.h"
#include "adi_adrv9001_version.h"
#include "adi_common_error_types.h"
#include "adi_platform_types.h"

/* gpio0 starts at 1 in the API enum */
#define ADRV9002_DGPIO_MIN	(ADI_ADRV9001_GPIO_DIGITAL_00 - 1)
#define ADRV9002_DGPIO_MAX	(ADI_ADRV9001_GPIO_DIGITAL_15 - 1)

#define ALL_RX_CHANNEL_MASK	(ADI_ADRV9001_RX1 | ADI_ADRV9001_RX2 | \
				 ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2)

#define ADRV9002_RX_EN(nr)	BIT(((nr) * 2) & 0x3)
#define ADRV9002_TX_EN(nr)	BIT(((nr) * 2 + 1) & 0x3)

#define ADRV9002_RX_MAX_GAIN_mdB	36000
#define ADRV9002_RX_GAIN_STEP_mDB	500
#define ADRV9002_RX_MIN_GAIN_IDX	183
#define ADRV9002_RX_MAX_GAIN_IDX	255

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

int adrv9002_spi_read(struct spi_device *spi, u32 reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;
	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, buf[2], ret);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	return buf[2];
}

int adrv9002_spi_write(struct spi_device *spi, u32 reg, u32 val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

	return 0;
}

void adrv9002_get_ssi_interface(struct adrv9002_rf_phy *phy, const int chann,
				u8 *ssi_intf, u8 *n_lanes, bool *cmos_ddr_en)
{
	/*
	 * Using the RX profile since with TX, we can have, for example, TX1 disabled
	 * while RX1 is enabled (the other way around is not permitted). Since this API
	 * only looks to the channel, we would return invalid values in such a case...
	 */
	adi_adrv9001_RxProfile_t *rx_cfg;
	/*
	 * We only look for one port. Although theoretical possible, we are
	 * assuming that ports on the same channel have the same number of lanes
	 * and, obviously, the same interface type
	 */
	rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chann].profile;
	*ssi_intf = rx_cfg->rxSsiConfig.ssiType;
	*n_lanes = rx_cfg->rxSsiConfig.numLaneSel;
	*cmos_ddr_en = rx_cfg->rxSsiConfig.cmosDdrEn;
}
EXPORT_SYMBOL(adrv9002_get_ssi_interface);

int adrv9002_ssi_configure(struct adrv9002_rf_phy *phy)
{
	bool cmos_ddr;
	u8 n_lanes, ssi_intf;
	int c, ret;

	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		/*
		 * Care only about RX because TX cannot be enabled while the RX on the
		 * same channel is disabled. This will also work in rx2tx2 mode since we
		 * only care at channel 1 and RX1 must be enabled. However, TX1 can be
		 * disabled which would lead to problems since we would no configure channel 1...
		 */
		struct adrv9002_chan *chan = &phy->rx_channels[c].channel;

		if (!chan->enabled)
			continue;

		/* the SSI settings should be done with the core in reset */
		adrv9002_axi_interface_enable(phy, c, false);
		if (phy->rx2tx2)
			adrv9002_sync_gpio_toogle(phy);

		adrv9002_get_ssi_interface(phy, c, &ssi_intf, &n_lanes, &cmos_ddr);
		ret = adrv9002_axi_interface_set(phy, n_lanes, ssi_intf, cmos_ddr, c);
		if (ret)
			return ret;

		adrv9002_axi_interface_enable(phy, c, true);

		if (phy->rx2tx2)
			break;
	}

	return 0;
}
EXPORT_SYMBOL(adrv9002_ssi_configure);

static int adrv9002_phy_reg_access(struct iio_dev *indio_dev,
				   u32 reg, u32 writeval,
				   u32 *readval)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (!readval) {
		ret = adrv9002_spi_write(phy->spi, reg, writeval);
	} else {
		*readval = adrv9002_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret;
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

static int adrv9002_clk_register(struct adrv9002_rf_phy *phy,
				 const char *name, const char *parent_name,
				 const char *parent_name2, unsigned long flags,
				 u32 source)
{
	struct adrv9002_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV9002_MAX_CLK_NAME + 1],
		p_name[2][ADRV9002_MAX_CLK_NAME + 1];
	const char *_parent_name[2];
	struct adi_adrv9001_TxProfile *tx_profile;
	struct adi_adrv9001_RxChannelCfg *rx_cfg;

	tx_profile = phy->curr_profile->tx.txProfile;
	rx_cfg = phy->curr_profile->rx.rxChannelCfg;

	/* struct adrv9002_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] = adrv9002_clk_set_dev_name(phy, p_name[0],
						    parent_name);
	_parent_name[1] = adrv9002_clk_set_dev_name(phy, p_name[1],
						    parent_name2);

	init.name = adrv9002_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
	case RX1_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		/* TODO: check indexing */
		clk_priv->rate = rx_cfg[0].profile.rxOutputRate_Hz;
		break;
	case RX2_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = rx_cfg[1].profile.rxOutputRate_Hz;
		break;
	case TX1_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = tx_profile[0].txInputRate_Hz;
		break;
	case TX2_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = tx_profile[1].txInputRate_Hz;
		break;
	default:
		return -EINVAL;
	}

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

static void adrv9002_set_clk_rates(const struct adrv9002_rf_phy *phy)
{
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	const struct adi_adrv9001_TxProfile *tx_cfg = phy->curr_profile->tx.txProfile;
	int c;

	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		const struct adrv9002_chan *rx = &phy->rx_channels[c].channel;
		const struct adrv9002_chan *tx = &phy->tx_channels[c].channel;

		if (!rx->enabled)
			/* TX cannot be enabled without the corresponding RX */
			continue;
		else if (phy->rx2tx2 && c)
			/* if in rx2tx2 we only care about RX1 clk */
			goto tx_clk;

		clk_set_rate(phy->clks[c], rx_cfg[c].profile.rxOutputRate_Hz);
tx_clk:
		if (!tx->enabled)
			continue;
		/*
		 * the point here is that TX2 can be enabled and TX1 not. If we are in
		 * rx2tx2 only TX1 is instantiated in the dds driver (with all 4 channels),
		 * so we need to set the TX1 clk rate.
		 */
		if (phy->rx2tx2) {
			clk_set_rate(phy->clks[TX1_SAMPL_CLK], tx_cfg[c].txInputRate_Hz);
			/* if TX0 is enabled then there's nothing else todo.. */
			if (!c)
				break;
		} else {
			clk_set_rate(phy->clks[c + TX1_SAMPL_CLK], tx_cfg[c].txInputRate_Hz);
		}
	}
}

static int __adrv9002_dev_err(const struct adrv9002_rf_phy *phy,
			      const char *function, const int line)
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
	case ADI_COMMON_ERR_API_FAIL:
		ret = -EFAULT;
	case ADI_COMMON_ERR_SPI_FAIL:
		ret = -EIO;
	case ADI_COMMON_ERR_MEM_ALLOC_FAIL:
		ret = -ENOMEM;
	default:
		ret = -EFAULT;
	}

	adi_common_ErrorClear(&phy->adrv9001->common);

	return ret;
}

#define adrv9002_dev_err(phy)	__adrv9002_dev_err(phy, __func__, __LINE__)

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
	struct adi_adrv9001_Carrier lo_freq = {
		.pllCalibration = ADI_ADRV9001_PLL_CALIBRATION_NORMAL,
		.loGenOptimization = ADI_ADRV9001_LO_GEN_OPTIMIZATION_PHASE_NOISE,
		.pllPower = ADI_ADRV9001_PLL_POWER_MEDIUM
	};

	if (!chann->enabled)
		return -ENODEV;

	switch (private) {
	case LOEXT_FREQ:
		ret = kstrtoull(buf, 10, &lo_freq.carrierFrequency_Hz);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);

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
unlock:
		mutex_unlock(&phy->lock);

		return ret ? ret : len;
	default:
		return -EINVAL;
	}
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

	if (!chann->enabled)
		return -ENODEV;

	switch (private) {
	case LOEXT_FREQ:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, port,
							 chann->number, &lo_freq);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%llu\n", lo_freq.carrierFrequency_Hz);
	default:
		return -EINVAL;
	}
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

	if (!rx->channel.enabled)
		return -ENODEV;

	if (mode > ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO)
		return -EINVAL;

	mutex_lock(&phy->lock);
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

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!chann->enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!chann->enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

enum {
	ADRV9002_INTF_GAIN_MANUAL_CORRECTION,
	ADRV9002_INTF_GAIN_MANUAL_COMPENSATION,
	ADRV9002_INTF_GAIN_AUTO_CORRECTION,
	ADRV9002_INTF_GAIN_AUTO_COMPENSATION,
};

static int adrv9002_set_digital_gain_ctl_mode(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan,
					      u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	struct adi_adrv9001_RxInterfaceGainCtrl rx_intf_gain_mode = {
		.updateInstance = ADI_ADRV9001_RX_INTERFACE_GAIN_UPDATE_TIMING_NOW,
		/*
		 * Reset gain to 0db. The reason is that depending on the gain
		 * table and the profile being used, some gains that make sense
		 * in one mode, might not make sense in the mode we are trying
		 * to change to.
		 */
		.gain = ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB,
	};

	if (!rx->channel.enabled)
		return -ENODEV;

	switch (mode) {
	case ADRV9002_INTF_GAIN_MANUAL_CORRECTION:
		rx_intf_gain_mode.controlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL;
		rx_intf_gain_mode.gainTableType = ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE;
		break;
	case ADRV9002_INTF_GAIN_MANUAL_COMPENSATION:
		rx_intf_gain_mode.controlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL;
		rx_intf_gain_mode.gainTableType = ADI_ADRV9001_RX_GAIN_COMPENSATION_TABLE;
		break;
	case ADRV9002_INTF_GAIN_AUTO_CORRECTION:
		rx_intf_gain_mode.controlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC;
		rx_intf_gain_mode.gainTableType = ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE;
		break;
	case ADRV9002_INTF_GAIN_AUTO_COMPENSATION:
		rx_intf_gain_mode.controlMode = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC;
		rx_intf_gain_mode.gainTableType = ADI_ADRV9001_RX_GAIN_COMPENSATION_TABLE;
		break;
	default:
		return -EINVAL;
	};

	mutex_lock(&phy->lock);

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
	int ret, mode;
	struct adi_adrv9001_RxInterfaceGainCtrl rx_intf_gain_mode;
	const int manual = ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL;
	const int correction_tbl = ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_InterfaceGain_Inspect(phy->adrv9001,
						    rx->channel.number,
						    &rx_intf_gain_mode);
	mutex_unlock(&phy->lock);

	if (ret)
		return adrv9002_dev_err(phy);

	if (rx_intf_gain_mode.controlMode == manual) {
		if (rx_intf_gain_mode.gainTableType == correction_tbl)
			mode = ADRV9002_INTF_GAIN_MANUAL_CORRECTION;
		else
			mode = ADRV9002_INTF_GAIN_MANUAL_COMPENSATION;
	} else {
		if (rx_intf_gain_mode.gainTableType == correction_tbl)
			mode = ADRV9002_INTF_GAIN_AUTO_CORRECTION;
		else
			mode = ADRV9002_INTF_GAIN_AUTO_COMPENSATION;
	}

	return mode;
}

static int adrv9002_get_intf_gain(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	adi_adrv9001_RxInterfaceGain_e gain;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!chann->enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!chann->enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!rx->channel.enabled)
		return -ENODEV;

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
			return ret;

		mutex_lock(&phy->lock);
		ret = adrv9002_update_tracking_calls(phy,
						     rx_track_calls[private],
						     channel, enable);
		mutex_unlock(&phy->lock);
		if (ret)
			return ret;
		return len;
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn)
			return -ENOTSUPP;

		ret = kstrtoint(buf, 10, &freq_offset_hz);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Rx_FrequencyCorrection_Set(phy->adrv9001,
							      rx->channel.number,
							      freq_offset_hz,
							      true);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		rx->channel.nco_freq = freq_offset_hz;
		return len;
	case RX_ADC_SWITCH:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);
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
unlock:
		mutex_unlock(&phy->lock);

		return ret ? ret : len;
	case RX_BBDC:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;
		/*
		 * Disabling the bbdc will completely disable the algorithm and set the correction
		 * value to 0. The difference with the tracking cal is that disabling it, just
		 * disables the algorithm but the last used correction value is still applied...
		 */
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_bbdc_RejectionEnable_Set(phy->adrv9001, ADI_RX,
							    rx->channel.number, enable);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return len;
	default:
		return -EINVAL;
	}
}

static ssize_t adrv9002_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u16 dec_pwr_mdb;
	struct adi_adrv9001_RxRssiStatus rssi;
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[channel];
	adi_adrv9001_BbdcRejectionStatus_e bbdc;
	bool enable;

	if (!rx->channel.enabled)
		return -ENODEV;

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001,
						     &tracking_cals);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%d\n", calls_mask[channel] &
			       rx_track_calls[private] ? 1 : 0);
	case RX_DECIMATION_POWER:
		/* it might depend on proper AGC parameters */
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Rx_DecimatedPower_Get(phy->adrv9001,
							 rx->channel.number,
							 &dec_pwr_mdb);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000,
			       dec_pwr_mdb % 1000);
	case RX_RSSI:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Rx_Rssi_Read(phy->adrv9001,
						rx->channel.number, &rssi);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%u.%02u dB\n", rssi.power_mdB / 1000,
			       rssi.power_mdB % 1000);
	case RX_RF_BANDWIDTH:
		rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chan->channel];
		return sprintf(buf, "%u\n",
			       rx_cfg->profile.primarySigBandwidth_Hz);
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn)
			return -ENOTSUPP;
		return sprintf(buf, "%d\n", rx->channel.nco_freq);
	case RX_ADC_SWITCH:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Rx_AdcSwitchEnable_Get(phy->adrv9001,
							  rx->channel.number,
							  &enable);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%d\n", enable);
	case RX_BBDC:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_bbdc_RejectionEnable_Get(phy->adrv9001, ADI_RX,
							    rx->channel.number, &bbdc);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return sprintf(buf, "%d\n", bbdc);
	default:
		return -EINVAL;
	}
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
	[TX_CLGC] = ADI_ADRV9001_TRACKING_CAL_TX_CLGC
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

	if (!tx->channel.enabled)
		return -ENODEV;

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

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
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

	if (!tx->channel.enabled)
		return -ENODEV;

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001,
						     &tracking_cals);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		val = calls_mask[channel] & tx_track_calls[private] ? 1 : 0;
		return sprintf(buf, "%d\n", val);
	case TX_RF_BANDWIDTH:
		return sprintf(buf, "%d\n", tx_cfg->primarySigBandwidth_Hz);
	case TX_NCO_FREQUENCY:
		/*
		 * This field seems to be the only thing that changes on TX profiles when nco
		 * is enabled.
		 */
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2)
			return -ENOTSUPP;
		return sprintf(buf, "%d\n", tx->channel.nco_freq);
	default:
		return -EINVAL;
	}
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

	if (!tx->channel.enabled)
		return -ENODEV;

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);
		ret = adrv9002_update_tracking_calls(phy,
						     tx_track_calls[private],
						     channel, enable);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		return len;
	case TX_NCO_FREQUENCY:
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2)
			return -ENOTSUPP;

		ret = kstrtoint(buf, 10, &nco_freq_hz);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Tx_FrequencyCorrection_Set(phy->adrv9001, tx->channel.number,
							      nco_freq_hz, true);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		tx->channel.nco_freq = nco_freq_hz;

		return len;
	default:
		return -EINVAL;
	}
}

#define _ADRV9002_EXT_TX_INFO(_name, _ident) { \
	.name = _name, \
	.read = adrv9002_phy_tx_read, \
	.write = adrv9002_phy_tx_write, \
	.private = _ident, \
}

static const char * const adrv9002_digital_gain_ctl_modes[] = {
	"Gain_Correction_manual_control",
	"Gain_Compensation_manual_control",
	"Gain_Correction_automatic_control",
	"Gain_Compensation_automatic_control",
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
	if (m != IIO_CHAN_INFO_PROCESSED)
		if (!chann->enabled)
			return -ENODEV;

	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			u16 atten_mdb;

			mutex_lock(&phy->lock);
			ret = adi_adrv9001_Tx_Attenuation_Get(phy->adrv9001,
							      chann->number,
							      &atten_mdb);
			mutex_unlock(&phy->lock);
			if (ret)
				return adrv9002_dev_err(phy);

			*val = -1 * (atten_mdb / 1000);
			*val2 = (atten_mdb % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

			return IIO_VAL_INT_PLUS_MICRO_DB;
		}

		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Rx_Gain_Get(phy->adrv9001, chann->number,
					       &index);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		temp = adrv9002_gainidx_to_gain(index);
		*val = temp / 1000;
		*val2 = temp % 1000 * 1000;

		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output) {
			if (phy->rx2tx2) {
				*val = clk_get_rate(phy->clks[TX1_SAMPL_CLK]);
			} else {
				switch (chann->number) {
				case ADI_CHANNEL_1:
					*val = clk_get_rate(phy->clks[TX1_SAMPL_CLK]);
					break;
				case ADI_CHANNEL_2:
					*val = clk_get_rate(phy->clks[TX2_SAMPL_CLK]);
					break;
				}
			}
		} else {
			if (phy->rx2tx2) {
				*val = clk_get_rate(phy->clks[RX1_SAMPL_CLK]);
			} else {
				switch (chann->number) {
				case ADI_CHANNEL_1:
					*val = clk_get_rate(phy->clks[RX1_SAMPL_CLK]);
					break;
				case ADI_CHANNEL_2:
					*val = clk_get_rate(phy->clks[RX2_SAMPL_CLK]);
					break;
				}
			}
		}

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		*val = chann->power;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&phy->lock);
		ret = adi_adrv9001_Temperature_Get(phy->adrv9001, &temp);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		*val = temp * 1000;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
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

	if (!chann->enabled)
		return -ENODEV;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0))
				return -EINVAL;

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			mutex_lock(&phy->lock);
			ret = adi_adrv9001_Tx_Attenuation_Set(phy->adrv9001,
							      chann->number,
							      code);
			mutex_unlock(&phy->lock);
		} else {
			u8 idx;
			int gain;

			gain = val * 1000 + val2 / 1000;
			idx = adrv9002_gain_to_gainidx(gain);

			mutex_lock(&phy->lock);
			ret = adi_adrv9001_Rx_Gain_Set(phy->adrv9001,
						       chann->number, idx);
			mutex_unlock(&phy->lock);
		}

		if (ret)
			return adrv9002_dev_err(phy);

		return 0;
	case IIO_CHAN_INFO_ENABLE:
		mutex_lock(&phy->lock);
		ret = adrv9002_channel_power_set(phy, chann, port, val);
		mutex_unlock(&phy->lock);
		return ret;
	default:
		return -EINVAL;
	}
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

static const char * const adrv9002_irqs[] = {
	"ARM error",
	"Force GP interrupt(Set by firmware to send an interrupt to BBIC)",
	"ARM System error",
	"ARM Calibration error",
	"ARM monitor interrupt",
	"TX1 PA protection error",
	"TX2 PA protection error",
	"low-power PLL lock indicator",
	"RF PLL1 lock indicator",
	"RF PLL2 lock indicator",
	"auxiliary Clock PLL lock indicator",
	"Clock PLL lock indicator",
	"main clock 1105 MCS",
	"main clock 1105 second MCS",
	"RX1 LSSI MCS",
	"RX2 LSSI MCS",
	"core stream processor error",
	"stream0 error",
	"stream1 error",
	"stream2 error",
	"stream3 error",
	"Unknown Interrupt",
	"Unknown Interrupt",
	"Unknown Interrupt",
	"TX DP write request error",
	"RX DP read request error",
	"SPI interface transmit error",
	"SPI interface receive error",
};

static irqreturn_t adrv9002_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_gpIntStatus_t gp_stat;
	int ret, bit;
	unsigned long active_irq;
	u8 error;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_gpio_GpIntHandler(phy->adrv9001, &gp_stat);
	mutex_unlock(&phy->lock);

	/* clear the logger */
	adi_common_ErrorClear(&phy->adrv9001->common);

	dev_warn(&phy->spi->dev, "GP Interrupt Status 0x%08X Mask 0x%08X\n",
		 gp_stat.gpIntStatus, ~gp_stat.gpIntSaveIrqMask & 0x0FFFFFFF);

	active_irq = gp_stat.gpIntActiveSources;
	for_each_set_bit(bit, &active_irq, ARRAY_SIZE(adrv9002_irqs)) {
		dev_warn(&phy->spi->dev, "%d: %s\n",bit, adrv9002_irqs[bit]);
	}

	/* check if there's something to be done */
	switch (ret) {
	case ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL:
		dev_warn(&phy->spi->dev, "Re-running tracking calibrations\n");
		ret = adi_adrv9001_cals_InitCals_Run(phy->adrv9001,
						     &phy->init_cals, 60000,
						     &error);
		if (ret)
			/* just log the error */
			adrv9002_dev_err(phy);
		break;
	case ADI_COMMON_ACT_ERR_RESET_FULL:
		dev_warn(&phy->spi->dev, "Reset might be needed...\n");
		break;
	case ADI_ADRV9001_ACT_ERR_BBIC_LOG_ERROR:
		/* nothing to do. IRQ already logged... */
		break;
	default:
		dev_err(&phy->spi->dev, "Unkonwn action: %d", ret);
		break;
	}

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
	struct adi_adrv9001_TxProfile *profi = phy->curr_profile->tx.txProfile;

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adi_adrv9001_Info *info = &phy->adrv9001->devStateInfo;
		/* For each tx channel enabled */
		if (!tx->channel.enabled)
			continue;
		/*
		 * Should this be done by the API? This seems to be needed for
		 * the NCO tone generation. We need to clarify if this will be
		 * done by the API in future releases.
		 */
		info->txInputRate_kHz[i] = profi[i].txInputRate_Hz / 1000;
		info->outputSignaling[i] = profi[i].outputSignaling;

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
	const u32 tx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_TX1, ADI_ADRV9001_TX2
	};
	const u32 rx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_RX1, ADI_ADRV9001_RX2
	};
	const u32 channels[ADRV9002_CHANN_MAX] = {
		ADI_CHANNEL_1, ADI_CHANNEL_2
	};
	int i, pos = 0;

	phy->init_cals.sysInitCalMask = 0;
	phy->init_cals.calMode = ADI_ADRV9001_INIT_CAL_MODE_ALL;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		if (ADRV9001_BF_EQUAL(phy->curr_profile->rx.rxInitChannelMask,
				      rx_channels[i])) {
			dev_dbg(&phy->spi->dev, "RX%d enabled\n", i);
			pos |= ADRV9002_RX_EN(i);
			rx->channel.power = true;
			rx->channel.enabled = true;
			rx->channel.number = channels[i];
		} else if (phy->rx2tx2 && i == ADRV9002_CHANN_1 ) {
			/*
			 * In rx2tx2 mode RX1 must be always enabled because RX2 cannot be
			 * on without RX1. On top of this, TX cannot be enabled without the
			 * corresponding RX. Hence, RX1 cannot really be disabled...
			 */
			dev_err(&phy->spi->dev, "In rx2tx2 mode RX1 must be always enabled...\n");
			return -EINVAL;
		}

		if (ADRV9001_BF_EQUAL(phy->curr_profile->tx.txInitChannelMask,
				      tx_channels[i])) {
			if (!rx->channel.enabled) {
				dev_err(&phy->spi->dev, "TX%d cannot be enabled while RX%d is disabled",
					i + 1, i + 1);
				return -EINVAL;
			}
			dev_dbg(&phy->spi->dev, "TX%d enabled\n", i);
			pos |= ADRV9002_TX_EN(i);
			tx->channel.power = true;
			tx->channel.enabled = true;
			tx->channel.number = channels[i];
		}

	}

	phy->init_cals.chanInitCalMask[0] = adrv9002_init_cals_mask[pos][0];
	phy->init_cals.chanInitCalMask[1] = adrv9002_init_cals_mask[pos][1];

	dev_dbg(&phy->spi->dev, "pos: %u, Chan1:%X, Chan2:%X", pos,
		phy->init_cals.chanInitCalMask[0],
		phy->init_cals.chanInitCalMask[1]);

	return 0;
}

static int adrv9002_setup(struct adrv9002_rf_phy *phy,
			  adi_adrv9001_Init_t *adrv9002_init)
{
	struct adi_adrv9001_Device *adrv9001_device = phy->adrv9001;
	struct adi_adrv9001_RadioCtrlInit *adrv9002_radio_init =
						adrv9002_radio_ctrl_init_get();
	struct adi_adrv9001_ResourceCfg adrv9001_resource_cfg = {
		adrv9002_init,
		adrv9002_radio_init,
		adrv9002_platform_files_get()
	};
	u8 init_cals_error = 0;
	u8 channel_mask = 0;
	int ret;
	adi_adrv9001_gpMaskArray_t gp_mask;
	adi_adrv9001_ChannelState_e init_state;

	phy->curr_profile = adrv9002_init;

	channel_mask = adrv9002_init->tx.txInitChannelMask |
		(adrv9002_init->rx.rxInitChannelMask & ALL_RX_CHANNEL_MASK);

	/* in TDD we cannot start with all ports enabled as RX/TX cannot be on at the same time */
	if (phy->curr_profile->sysConfig.duplexMode == ADI_ADRV9001_TDD_MODE)
		init_state = ADI_ADRV9001_CHANNEL_PRIMED;
	else
		init_state = ADI_ADRV9001_CHANNEL_RF_ENABLED;

	/* compute init call and does some profile validations... */
	ret = adrv9002_compute_init_cals(phy);
	if (ret)
		return ret;

	adi_common_ErrorClear(&phy->adrv9001->common);
	ret = adi_adrv9001_HwOpen(adrv9001_device, adrv9002_spi_settings_get());
	if (ret)
		return adrv9002_dev_err(phy);

	adrv9002_set_loglevel(&adrv9001_device->common, ADI_HAL_LOG_ERR);

	ret = adi_adrv9001_InitAnalog(adrv9001_device, adrv9002_init,
			adrv9002_radio_init->adrv9001DeviceClockOutputDivisor);
	if (ret)
		return adrv9002_dev_err(phy);

	/* needs to be done before loading the ARM image */
	ret = adrv9002_tx_set_dac_full_scale(phy);
	if (ret)
		return ret;

	ret = adi_adrv9001_Utilities_Resources_Load(adrv9001_device,
						    &adrv9001_resource_cfg);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_Utilities_InitRadio_Load(adrv9001_device,
						    &adrv9001_resource_cfg,
						    channel_mask);
	if (ret)
		return adrv9002_dev_err(phy);

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
	gp_mask.gpIntMask = ~ADRV9002_IRQ_MASK;
	ret = adi_adrv9001_gpio_GpIntMask_Set(adrv9001_device,
					      ADI_ADRV9001_GPINT, &gp_mask);
	if (ret)
		return adrv9002_dev_err(phy);

	adrv9002_set_clk_rates(phy);
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
EXPORT_SYMBOL(adrv9002_intf_change_delay);

int adrv9002_check_tx_test_pattern(struct adrv9002_rf_phy *phy, const int chann,
				   const adi_adrv9001_SsiType_e ssi_type)
{
	int ret;
	struct adrv9002_chan *chan = &phy->tx_channels[chann].channel;
	adi_adrv9001_SsiTestModeData_e test_data = ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ?
						ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE :
						ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;
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
EXPORT_SYMBOL(adrv9002_check_tx_test_pattern);

int adrv9002_intf_test_cfg(struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop, const adi_adrv9001_SsiType_e ssi_type)
{
	int ret;
	struct adrv9002_chan *chan;
	adi_adrv9001_SsiTestModeData_e test_data = ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ?
						ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE :
						ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;

	dev_dbg(&phy->spi->dev, "cfg test stop:%u, ssi:%d, c:%d, tx:%d\n", stop, ssi_type, chann,
		tx);

	if (tx) {
		struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};
		chan = &phy->tx_channels[chann].channel;

		cfg.testData = stop ? ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL : test_data;
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

		cfg.testData = stop ? ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL : test_data;
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
EXPORT_SYMBOL(adrv9002_intf_test_cfg);

static int adrv9002_intf_tuning_unlocked(struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};
	int ret;
	u8 clk_delay, data_delay;
	struct adrv9002_chan *chan;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int i;

	/* FIXME: do not tune for CMOS for now */
	if (ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		return 0;

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

int adrv9002_intf_tuning(struct adrv9002_rf_phy *phy)
{
	int ret;

	mutex_lock(&phy->lock);
	ret = adrv9002_intf_tuning_unlocked(phy);
	mutex_unlock(&phy->lock);

	return ret;
}
EXPORT_SYMBOL(adrv9002_intf_tuning);

static void adrv9002_cleanup(struct adrv9002_rf_phy *phy)
{
	int i;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		memset(&phy->rx_channels[i].channel, 0,
		       sizeof(struct adrv9002_chan));

		memset(&phy->tx_channels[i].channel, 0,
		       sizeof(struct adrv9002_chan));
	}

	memset(&phy->adrv9001->devStateInfo, 0,
	       sizeof(phy->adrv9001->devStateInfo));
}

#define rx_to_phy(rx, nr)	\
	container_of(rx, struct adrv9002_rf_phy, rx_channels[nr])

#define tx_to_phy(tx, nr)	\
	container_of(tx, struct adrv9002_rf_phy, tx_channels[nr])

#ifdef CONFIG_DEBUG_FS
static ssize_t adrv9002_rx_adc_type_get(struct file *file, char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	char buf[8];
	adi_adrv9001_AdcType_e adc_type;
	int ret, len;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_AdcType_Get(phy->adrv9001, rx->channel.number,
					  &adc_type);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	len = snprintf(buf, sizeof(buf), "%s\n",
		       adc_type == ADI_ADRV9001_ADC_HP ? "HP" : "LP");

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations adrv9002_channel_adc_type_fops = {
	.open = simple_open,
	.read = adrv9002_rx_adc_type_get,
	.llseek = default_llseek,
};

#define adrv9002_seq_printf(seq, ptr, member) \
	seq_printf(seq, "%s: %u\n", #member, (ptr)->member)

static int adrv9002_rx_gain_control_pin_mode_show(struct seq_file *s,
						  void *ignored)
{
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	int ret;
	struct adi_adrv9001_RxGainControlPinCfg cfg = {0};

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_PinMode_Inspect(phy->adrv9001,
							  rx->channel.number,
							  &cfg);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	seq_printf(s, "min_gain_index: %u\n", cfg.minGainIndex);
	seq_printf(s, "max_gain_index: %u\n", cfg.maxGainIndex);
	seq_printf(s, "increment_step_size: %u\n", cfg.incrementStepSize);
	seq_printf(s, "decrement_step_size: %u\n", cfg.decrementStepSize);
	seq_printf(s, "increment_pin: dgpio%d\n", cfg.incrementPin - 1);
	seq_printf(s, "decrement_pin: dgpio%d\n", cfg.decrementPin - 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_rx_gain_control_pin_mode);

static int adrv9002_rx_agc_config_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	struct adi_adrv9001_GainControlCfg agc = {0};
	int ret;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_Inspect(phy->adrv9001,
						  rx->channel.number, &agc);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

#define adrv9002_agc_seq_printf(member) \
	adrv9002_seq_printf(s, &agc, member)

	adrv9002_agc_seq_printf(peakWaitTime);
	adrv9002_agc_seq_printf(maxGainIndex);
	adrv9002_agc_seq_printf(minGainIndex);
	adrv9002_agc_seq_printf(gainUpdateCounter);
	adrv9002_agc_seq_printf(attackDelay_us);
	adrv9002_agc_seq_printf(slowLoopSettlingDelay);
	adrv9002_agc_seq_printf(lowThreshPreventGainInc);
	adrv9002_agc_seq_printf(changeGainIfThreshHigh);
	adrv9002_agc_seq_printf(agcMode);
	adrv9002_agc_seq_printf(resetOnRxon);
	adrv9002_agc_seq_printf(resetOnRxonGainIndex);
	adrv9002_agc_seq_printf(enableSyncPulseForGainCounter);
	adrv9002_agc_seq_printf(enableFastRecoveryLoop);
	/* power parameters */
	adrv9002_agc_seq_printf(power.powerEnableMeasurement);
	adrv9002_agc_seq_printf(power.underRangeHighPowerThresh);
	adrv9002_agc_seq_printf(power.underRangeLowPowerThresh);
	adrv9002_agc_seq_printf(power.underRangeHighPowerGainStepRecovery);
	adrv9002_agc_seq_printf(power.underRangeLowPowerGainStepRecovery);
	adrv9002_agc_seq_printf(power.powerMeasurementDuration);
	adrv9002_agc_seq_printf(power.powerMeasurementDelay);
	adrv9002_agc_seq_printf(power.rxTddPowerMeasDuration);
	adrv9002_agc_seq_printf(power.rxTddPowerMeasDelay);
	adrv9002_agc_seq_printf(power.overRangeHighPowerThresh);
	adrv9002_agc_seq_printf(power.overRangeLowPowerThresh);
	adrv9002_agc_seq_printf(power.overRangeHighPowerGainStepAttack);
	adrv9002_agc_seq_printf(power.overRangeLowPowerGainStepAttack);
	adrv9002_agc_seq_printf(power.feedback_lowThreshold_gainChange);
	adrv9002_agc_seq_printf(power.feedback_high_threshold_exceeded);
	/* peak parameters */
	adrv9002_agc_seq_printf(peak.agcUnderRangeLowInterval);
	adrv9002_agc_seq_printf(peak.agcUnderRangeMidInterval);
	adrv9002_agc_seq_printf(peak.agcUnderRangeHighInterval);
	adrv9002_agc_seq_printf(peak.apdHighThresh);
	adrv9002_agc_seq_printf(peak.apdLowThresh);
	adrv9002_agc_seq_printf(peak.apdUpperThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.apdLowerThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.apdGainStepAttack);
	adrv9002_agc_seq_printf(peak.apdGainStepRecovery);
	adrv9002_agc_seq_printf(peak.enableHbOverload);
	adrv9002_agc_seq_printf(peak.hbOverloadDurationCount);
	adrv9002_agc_seq_printf(peak.hbOverloadThreshCount);
	adrv9002_agc_seq_printf(peak.hbHighThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeLowThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeMidThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeHighThresh);
	adrv9002_agc_seq_printf(peak.hbUpperThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.hbUnderRangeHighThreshExceededCount);
	adrv9002_agc_seq_printf(peak.hbGainStepHighRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepLowRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepMidRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepMidRecovery);
	adrv9002_agc_seq_printf(peak.hbOverloadPowerMode);
	adrv9002_agc_seq_printf(peak.hbUnderRangeMidThreshExceededCount);
	adrv9002_agc_seq_printf(peak.hbUnderRangeLowThreshExceededCount);
	adrv9002_agc_seq_printf(peak.feedback_low_threshold_counter_exceeded);
	adrv9002_agc_seq_printf(peak.feedback_high_threshold_counter_exceeded);

	return 0;
}

static ssize_t adrv9002_rx_agc_config_write(struct file *file, const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_Configure(phy->adrv9001, rx->channel.number,
						    &rx->debug_agc);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return count;
}

static int adrv9002_rx_agc_config_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv9002_rx_agc_config_show, inode->i_private);
}

static const struct file_operations adrv9002_rx_agc_config_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv9002_rx_agc_config_open,
	.read		= seq_read,
	.write		= adrv9002_rx_agc_config_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void adrv9002_debugfs_agc_config_create(struct adrv9002_rx_chan *rx, struct dentry *d)
{
#define adrv9002_agc_get_attr(nr, member) ((nr) == ADI_CHANNEL_1 ? \
					"rx0_agc_" #member : "rx1_agc_" #member)

#define adrv9002_agc_add_file_u8(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u8(attr, 0600, d, (u8 *)&rx->debug_agc.member); \
}

#define adrv9002_agc_add_file_u16(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u16(attr, 0600, d, &rx->debug_agc.member); \
}

#define adrv9002_agc_add_file_u32(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u32(attr, 0600, d, &rx->debug_agc.member); \
}

#define adrv9002_agc_add_file_bool(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_bool(attr, 0600, d, &rx->debug_agc.member); \
}
	adrv9002_agc_add_file_u8(peakWaitTime);
	adrv9002_agc_add_file_u8(maxGainIndex);
	adrv9002_agc_add_file_u8(minGainIndex);
	adrv9002_agc_add_file_u32(gainUpdateCounter);
	adrv9002_agc_add_file_u8(attackDelay_us);
	adrv9002_agc_add_file_u8(slowLoopSettlingDelay);
	adrv9002_agc_add_file_bool(lowThreshPreventGainInc);
	adrv9002_agc_add_file_u8(changeGainIfThreshHigh);
	adrv9002_agc_add_file_u8(agcMode);
	adrv9002_agc_add_file_bool(resetOnRxon);
	adrv9002_agc_add_file_u8(resetOnRxonGainIndex);
	adrv9002_agc_add_file_bool(enableSyncPulseForGainCounter);
	adrv9002_agc_add_file_bool(enableFastRecoveryLoop);
	/* power parameters */
	adrv9002_agc_add_file_bool(power.powerEnableMeasurement);
	adrv9002_agc_add_file_u8(power.underRangeHighPowerThresh);
	adrv9002_agc_add_file_u8(power.underRangeLowPowerThresh);
	adrv9002_agc_add_file_u8(power.underRangeHighPowerGainStepRecovery);
	adrv9002_agc_add_file_u8(power.underRangeLowPowerGainStepRecovery);
	adrv9002_agc_add_file_u8(power.powerMeasurementDuration);
	adrv9002_agc_add_file_u8(power.powerMeasurementDelay);
	adrv9002_agc_add_file_u16(power.rxTddPowerMeasDuration);
	adrv9002_agc_add_file_u16(power.rxTddPowerMeasDelay);
	adrv9002_agc_add_file_u8(power.overRangeHighPowerThresh);
	adrv9002_agc_add_file_u8(power.overRangeLowPowerThresh);
	adrv9002_agc_add_file_u8(power.overRangeHighPowerGainStepAttack);
	adrv9002_agc_add_file_u8(power.overRangeLowPowerGainStepAttack);
	adrv9002_agc_add_file_u8(power.feedback_lowThreshold_gainChange);
	adrv9002_agc_add_file_u8(power.feedback_high_threshold_exceeded);
	/* peak parameters */
	adrv9002_agc_add_file_u16(peak.agcUnderRangeLowInterval);
	adrv9002_agc_add_file_u8(peak.agcUnderRangeMidInterval);
	adrv9002_agc_add_file_u8(peak.agcUnderRangeHighInterval);
	adrv9002_agc_add_file_u8(peak.apdHighThresh);
	adrv9002_agc_add_file_u8(peak.apdLowThresh);
	adrv9002_agc_add_file_u8(peak.apdUpperThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.apdLowerThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.apdGainStepAttack);
	adrv9002_agc_add_file_u8(peak.apdGainStepRecovery);
	adrv9002_agc_add_file_bool(peak.enableHbOverload);
	adrv9002_agc_add_file_u8(peak.hbOverloadDurationCount);
	adrv9002_agc_add_file_u8(peak.hbOverloadThreshCount);
	adrv9002_agc_add_file_u16(peak.hbHighThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeLowThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeMidThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeHighThresh);
	adrv9002_agc_add_file_u8(peak.hbUpperThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeHighThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.hbGainStepHighRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepLowRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepMidRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepMidRecovery);
	adrv9002_agc_add_file_u8(peak.hbOverloadPowerMode);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeMidThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeLowThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.feedback_low_threshold_counter_exceeded);
	adrv9002_agc_add_file_u8(peak.feedback_high_threshold_counter_exceeded);
}

static int adrv9002_tx_dac_full_scale_get(void *arg, u64 *val)
{
	struct adrv9002_tx_chan	*tx = arg;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.number - 1);
	int ret;
	bool enable;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Tx_OutputPowerBoost_Get(phy->adrv9001,
						   tx->channel.number, &enable);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	*val = enable;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_tx_dac_full_scale_fops,
			 adrv9002_tx_dac_full_scale_get,
			 NULL, "%lld\n");

static int adrv9002_tx_pin_atten_control_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_tx_chan	*tx = s->private;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.number - 1);
	struct adi_adrv9001_TxAttenuationPinControlCfg cfg = {0};
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Tx_Attenuation_PinControl_Inspect(phy->adrv9001,
							     tx->channel.number,
							     &cfg);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	seq_printf(s, "step_size_mdB: %u\n", cfg.stepSize_mdB);
	seq_printf(s, "increment_pin: dgpio%d\n", cfg.incrementPin - 1);
	seq_printf(s, "decrement_pin: dgpio%d\n", cfg.decrementPin - 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_tx_pin_atten_control);

static int adrv9002_pll_status_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rf_phy *phy = s->private;
	int ret;
	bool lo1, lo2, aux, clk, clk_lp;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
					       ADI_ADRV9001_PLL_LO1, &lo1);

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_LO2, &lo2);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_AUX, &aux);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_CLK, &clk);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_CLK_LP,
	                                       &clk_lp);
	if (ret)
		goto error;
	mutex_unlock(&phy->lock);

	seq_printf(s, "Clock: %s\n", clk ? "Locked" : "Unlocked");
	seq_printf(s, "Clock LP: %s\n", clk_lp ? "Locked" : "Unlocked");
	seq_printf(s, "LO1: %s\n", lo1 ? "Locked" : "Unlocked");
	seq_printf(s, "LO2: %s\n", lo2 ? "Locked" : "Unlocked");
	seq_printf(s, "AUX: %s\n", aux ? "Locked" : "Unlocked");

	return 0;
error:
	mutex_unlock(&phy->lock);
	return adrv9002_dev_err(phy);
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_pll_status);

static const char *const adrv9002_ssi_test_mode_data_avail[] = {
	"TESTMODE_DATA_NORMAL",
	"TESTMODE_DATA_FIXED_PATTERN",
	"TESTMODE_DATA_RAMP_NIBBLE",
	"TESTMODE_DATA_RAMP_16_BIT",
	"TESTMODE_DATA_PRBS15",
	"TESTMODE_DATA_PRBS7",
};

#define ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE	6
#define ADRV9002_RX_SSI_TEST_DATA_LVDS_MASK	0x3b
#define ADRV9002_RX_SSI_TEST_DATA_CMOS_MASK	GENMASK(2, 0)
#define ADRV9002_TX_SSI_TEST_DATA_LVDS_MASK	0x33
#define ADRV9002_TX_SSI_TEST_DATA_CMOS_MASK	GENMASK(1, 0)

static unsigned long rx_ssi_avail_mask;
static unsigned long tx_ssi_avail_mask;

static ssize_t adrv9002_ssi_test_mode_data_show(char __user *userbuf,
						size_t count, loff_t *ppos,
						const char *item)
{
	char buf[32];
	int len;

	len = scnprintf(buf, sizeof(buf), "%s\n", item);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static int adrv9002_ssi_test_mode_data_set(const char __user *userbuf,
					   size_t count, loff_t *ppos,
					   const unsigned long mask)
{
	char buf[32] = {0};
	int bit = 0, ret;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf,
				     count);
	if (ret < 0)
		return ret;

	for_each_set_bit(bit, &mask, ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE) {
		if (sysfs_streq(buf, adrv9002_ssi_test_mode_data_avail[bit]))
			break;
	}

	if (bit == ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE)
		return -EINVAL;

	return bit;
}

static int adrv9002_ssi_mode_avail_show(struct seq_file *s, void *ignored)
{
	int bit = 0;
	const unsigned long *mask = s->private;

	for_each_set_bit(bit, mask, ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE) {
		seq_printf(s, "%s\n", adrv9002_ssi_test_mode_data_avail[bit]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_ssi_mode_avail);

static ssize_t adrv9002_rx_ssi_test_mode_data_show(struct file *file,
						   char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	int idx = rx->ssi_test.testData;
	const char *data = adrv9002_ssi_test_mode_data_avail[idx];

	return adrv9002_ssi_test_mode_data_show(userbuf, count, ppos, data);
}

static ssize_t adrv9002_rx_ssi_test_mode_data_set(struct file *file,
						  const char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	int ret;

	ret = adrv9002_ssi_test_mode_data_set(userbuf, count, ppos, rx_ssi_avail_mask);
	if (ret < 0)
		return ret;

	rx->ssi_test.testData = ret;

	return count;
}

static const struct file_operations adrv9002_rx_ssi_test_mode_data_fops = {
	.open = simple_open,
	.read = adrv9002_rx_ssi_test_mode_data_show,
	.write = adrv9002_rx_ssi_test_mode_data_set,
	.llseek = default_llseek,
};

static int adrv9002_rx_ssi_test_mode_fixed_pattern_get(void *arg, u64 *val)
{
	struct adrv9002_rx_chan	*rx = arg;

	*val = rx->ssi_test.fixedDataPatternToTransmit;

	return 0;
};

static int adrv9002_rx_ssi_test_mode_fixed_pattern_set(void *arg, const u64 val)
{
	struct adrv9002_rx_chan	*rx = arg;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int val_max = ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ? 0xf : U16_MAX;
	int __val;

	__val = clamp_val(val, 0, val_max);
	rx->ssi_test.fixedDataPatternToTransmit = __val;

	return 0;
};

DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_rx_ssi_test_mode_fixed_pattern_fops,
			 adrv9002_rx_ssi_test_mode_fixed_pattern_get,
			 adrv9002_rx_ssi_test_mode_fixed_pattern_set,
			 "%llu\n");

static int adrv9002_ssi_rx_test_mode_set(void *arg, const u64 val)
{
	struct adrv9002_rx_chan	*rx = arg;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.number - 1);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int ret;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, rx->channel.number,
						     ssi_type,
						     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						     &rx->ssi_test);
	mutex_unlock(&phy->lock);
	if (ret)
		adrv9002_dev_err(phy);

	return 0;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_ssi_rx_test_mode_config_fops,
			 NULL, adrv9002_ssi_rx_test_mode_set, "%llu");

static ssize_t adrv9002_tx_ssi_test_mode_data_show(struct file *file,
						   char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct adrv9002_tx_chan	*tx = file->private_data;
	int idx = tx->ssi_test.testData;
	const char *data = adrv9002_ssi_test_mode_data_avail[idx];

	return adrv9002_ssi_test_mode_data_show(userbuf, count, ppos, data);
}

static ssize_t adrv9002_tx_ssi_test_mode_data_set(struct file *file,
						  const char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct adrv9002_tx_chan	*tx = file->private_data;
	int ret;

	ret = adrv9002_ssi_test_mode_data_set(userbuf, count, ppos, tx_ssi_avail_mask);
	if (ret < 0)
		return ret;

	tx->ssi_test.testData = ret;

	return count;
}

static const struct file_operations adrv9002_tx_ssi_test_mode_data_fops = {
	.open = simple_open,
	.read = adrv9002_tx_ssi_test_mode_data_show,
	.write = adrv9002_tx_ssi_test_mode_data_set,
	.llseek = default_llseek,
};

static int adrv9002_tx_ssi_test_mode_fixed_pattern_get(void *arg, u64 *val)
{
	struct adrv9002_tx_chan	*tx = arg;

	*val = tx->ssi_test.fixedDataPatternToCheck;

	return 0;
};

static int adrv9002_tx_ssi_test_mode_fixed_pattern_set(void *arg, const u64 val)
{
	struct adrv9002_tx_chan	*tx = arg;
	u32 __val;

	__val = clamp_val(val, 0, U16_MAX);
	tx->ssi_test.fixedDataPatternToCheck = __val;

	return 0;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_tx_ssi_test_mode_fixed_pattern_fops,
			 adrv9002_tx_ssi_test_mode_fixed_pattern_get,
			 adrv9002_tx_ssi_test_mode_fixed_pattern_set,
			 "%llu\n");

static int adrv9002_init_set(void *arg, const u64 val)
{
	struct adrv9002_rf_phy *phy = arg;
	int ret;

	if (!val)
		return -EINVAL;

	mutex_lock(&phy->lock);
	adrv9002_cleanup(phy);
	ret = adrv9002_setup(phy, phy->curr_profile);
	mutex_unlock(&phy->lock);

	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_init_fops,
			 NULL, adrv9002_init_set, "%llu");

static int adrv9002_ssi_tx_test_mode_set(void *arg, const u64 val)
{
	struct adrv9002_tx_chan	*tx = arg;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.number - 1);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, tx->channel.number,
						     ssi_type,
						     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						     &tx->ssi_test);
	mutex_unlock(&phy->lock);
	if (ret)
		adrv9002_dev_err(phy);

	return 0;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_ssi_tx_test_mode_config_fops,
			 NULL, adrv9002_ssi_tx_test_mode_set, "%llu");

static int adrv9002_ssi_tx_test_mode_status_show(struct seq_file *s,
						 void *ignored)
{
	struct adrv9002_tx_chan	*tx = s->private;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.number - 1);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	adi_adrv9001_TxSsiTestModeStatus_t ssi_status = {0};
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001,
							  tx->channel.number,
							  ssi_type,
							  ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							  &tx->ssi_test,
							  &ssi_status);
	mutex_unlock(&phy->lock);
	if (ret)
		adrv9002_dev_err(phy);

	seq_printf(s, "dataError: %u\n", ssi_status.dataError);
	seq_printf(s, "fifoFull: %u\n", ssi_status.fifoFull);
	seq_printf(s, "fifoEmpty: %u\n", ssi_status.fifoEmpty);
	seq_printf(s, "strobeAlignError: %u\n", ssi_status.strobeAlignError);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_ssi_tx_test_mode_status);

static int adrv9002_ssi_delays_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rf_phy *phy = s->private;
	int ret, i;
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Inspect(phy->adrv9001, ssi_type, &delays);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		seq_printf(s, "rx%d_ClkDelay: %u\n", i, delays.rxClkDelay[i]);
		seq_printf(s, "rx%d_StrobeDelay: %u\n", i, delays.rxStrobeDelay[i]);
		seq_printf(s, "rx%d_rxIDataDelay: %u\n", i, delays.rxIDataDelay[i]);
		seq_printf(s, "rx%d_rxQDataDelay: %u\n", i, delays.rxQDataDelay[i]);
		seq_printf(s, "tx%d_ClkDelay: %u\n", i, delays.txClkDelay[i]);
		seq_printf(s, "tx%d_RefClkDelay: %u\n", i, delays.txRefClkDelay[i]);
		seq_printf(s, "tx%d_StrobeDelay: %u\n", i, delays.txStrobeDelay[i]);
		seq_printf(s, "tx%d_rxIDataDelay: %u\n", i, delays.txIDataDelay[i]);
		seq_printf(s, "tx%d_rxQDataDelay: %u\n", i, delays.txQDataDelay[i]);
	}

	return 0;
}

static ssize_t adrv9002_ssi_delays_write(struct file *file, const char __user *userbuf,
					 size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9002_rf_phy *phy = s->private;
	int ret;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, ssi_type, &phy->ssi_delays);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return count;
}

static int adrv9002_ssi_delays_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv9002_ssi_delays_show, inode->i_private);
}

static const struct file_operations adrv9002_ssi_delays_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv9002_ssi_delays_open,
	.read		= seq_read,
	.write		= adrv9002_ssi_delays_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy)
{
	int chan;
	char attr[64];
	struct iio_dev *indio_dev = iio_priv_to_dev(phy);
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	if (!d)
		return;

	if (ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS) {
		rx_ssi_avail_mask = ADRV9002_RX_SSI_TEST_DATA_CMOS_MASK;
		tx_ssi_avail_mask = ADRV9002_TX_SSI_TEST_DATA_CMOS_MASK;
	} else {
		rx_ssi_avail_mask = ADRV9002_RX_SSI_TEST_DATA_LVDS_MASK;
		tx_ssi_avail_mask = ADRV9002_TX_SSI_TEST_DATA_LVDS_MASK;
	}

	debugfs_create_file_unsafe("initialize", 0600, d, phy,
				   &adrv9002_init_fops);

	debugfs_create_file("pll_status", 0400, d, phy,
			    &adrv9002_pll_status_fops);

	debugfs_create_file("rx_ssi_test_mode_data_available", 0400, d,
			    &rx_ssi_avail_mask, &adrv9002_ssi_mode_avail_fops);

	debugfs_create_file("tx_ssi_test_mode_data_available", 0400, d,
			    &tx_ssi_avail_mask, &adrv9002_ssi_mode_avail_fops);

	debugfs_create_file("ssi_delays", 0600, d, phy, &adrv9002_ssi_delays_fops);

	for (chan = 0; chan < ARRAY_SIZE(phy->tx_channels); chan++) {
		sprintf(attr, "tx%d_attenuation_pin_control", chan);
		debugfs_create_file(attr, 0400, d, &phy->tx_channels[chan],
				    &adrv9002_tx_pin_atten_control_fops);
		sprintf(attr, "tx%d_dac_boost_en", chan);
		debugfs_create_file_unsafe(attr, 0400, d,
					   &phy->tx_channels[chan],
					   &adrv9002_tx_dac_full_scale_fops);
		sprintf(attr, "tx%d_ssi_test_mode_data", chan);
		debugfs_create_file(attr, 0600, d, &phy->tx_channels[chan],
				    &adrv9002_tx_ssi_test_mode_data_fops);
		sprintf(attr, "tx%d_ssi_test_mode_fixed_pattern", chan);
		debugfs_create_file_unsafe(attr, 0600, d,
					   &phy->tx_channels[chan],
					   &adrv9002_tx_ssi_test_mode_fixed_pattern_fops);
		sprintf(attr, "tx%d_ssi_test_mode_configure", chan);
		debugfs_create_file(attr, 0200, d,
				    &phy->tx_channels[chan],
				    &adrv9002_ssi_tx_test_mode_config_fops);
		sprintf(attr, "tx%d_ssi_test_mode_status", chan);
		debugfs_create_file(attr, 0400, d,
				    &phy->tx_channels[chan],
				    &adrv9002_ssi_tx_test_mode_status_fops);

		sprintf(attr, "tx%d_ssi_clk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txClkDelay[chan]);
		sprintf(attr, "tx%d_ssi_refclk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txRefClkDelay[chan]);
		sprintf(attr, "tx%d_ssi_strobe_delay_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txStrobeDelay[chan]);
		sprintf(attr, "tx%d_ssi_i_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txIDataDelay[chan]);
		sprintf(attr, "tx%d_ssi_q_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txQDataDelay[chan]);
	}

	for (chan = 0; chan < ARRAY_SIZE(phy->rx_channels); chan++) {
		sprintf(attr, "rx%d_adc_type", chan);
		debugfs_create_file(attr, 0400, d, &phy->rx_channels[chan],
				    &adrv9002_channel_adc_type_fops);
		sprintf(attr, "rx%d_gain_control_pin_mode", chan);
		debugfs_create_file(attr, 0400, d, &phy->rx_channels[chan],
				    &adrv9002_rx_gain_control_pin_mode_fops);
		sprintf(attr, "rx%d_agc_config", chan);
		debugfs_create_file(attr, 0600, d, &phy->rx_channels[chan],
				    &adrv9002_rx_agc_config_fops);
		sprintf(attr, "rx%d_ssi_test_mode_data", chan);
		debugfs_create_file(attr, 0600, d, &phy->rx_channels[chan],
				    &adrv9002_rx_ssi_test_mode_data_fops);
		sprintf(attr, "rx%d_ssi_test_mode_fixed_pattern", chan);
		debugfs_create_file_unsafe(attr, 0600, d,
					   &phy->rx_channels[chan],
					   &adrv9002_rx_ssi_test_mode_fixed_pattern_fops);
		sprintf(attr, "rx%d_ssi_test_mode_configure", chan);
		debugfs_create_file(attr, 0200, d,
				    &phy->rx_channels[chan],
				    &adrv9002_ssi_rx_test_mode_config_fops);

		sprintf(attr, "rx%d_ssi_clk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxClkDelay[chan]);
		sprintf(attr, "rx%d_ssi_strobe_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxStrobeDelay[chan]);
		sprintf(attr, "rx%d_ssi_i_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxIDataDelay[chan]);
		sprintf(attr, "rx%d_ssi_q_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxQDataDelay[chan]);
		adrv9002_debugfs_agc_config_create(&phy->rx_channels[chan], d);
	}
}
#else
static void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy)
{
}
#endif

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
	 AGC_OFFSETOF(resetOnRxonGainIndex), 255, 183, 255, 1},
	/* power detector */
	{"adi,power-under-range-high-threshold",
	 AGC_OFFSETOF(power.underRangeHighPowerThresh), 4, 0, 127, 1},
	{"adi,power-under-range-low-threshold",
	 AGC_OFFSETOF(power.underRangeLowPowerThresh), 0, 0, 31, 1},
	{"adi,power-under-range-high-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeHighPowerGainStepRecovery), 4, 0, 31, 1},
	{"adi,power-under-range-low-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeLowPowerGainStepRecovery), 4, 0, 31, 1},
	{"adi,power-measurement-duration",
	 AGC_OFFSETOF(power.powerMeasurementDuration), 10, 0, 31, 1},
	{"adi,power-measurement-delay",
	 AGC_OFFSETOF(power.powerMeasurementDelay), 3, 0, 255, 1},
	{"adi,power-rx-tdd-measurement-duration",
	 AGC_OFFSETOF(power.rxTddPowerMeasDuration), 0, 0, 65535, 2},
	{"adi,power-rx-tdd-measurement-delay",
	 AGC_OFFSETOF(power.rxTddPowerMeasDelay), 92, 0, 65535, 2},
	{"adi,power-over-range-high-threshold",
	 AGC_OFFSETOF(power.overRangeHighPowerThresh), 0, 0, 15, 1},
	{"adi,power-over-range-low-threshold",
	 AGC_OFFSETOF(power.overRangeLowPowerThresh), 1, 0, 127, 1},
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
	 AGC_OFFSETOF(peak.apdHighThresh), 38, 0, 63, 1},
	{"adi,peak-apd-low-threshold",
	 AGC_OFFSETOF(peak.apdLowThresh), 27, 10, 63, 1},
	{"adi,peak-apd-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-apd-lower-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdLowerThreshPeakExceededCount), 3, 0, 255, 1},
	{"adi,peak-apd-gain-step-attack",
	 AGC_OFFSETOF(peak.apdGainStepAttack), 4, 0, 31, 1},
	{"adi,peak-apd-gain-step-recovery",
	 AGC_OFFSETOF(peak.apdGainStepRecovery), 0, 0, 31, 1},
	{"adi,peak-hb-overload-duration-count",
	 AGC_OFFSETOF(peak.hbOverloadDurationCount), 1, 0, 7, 1},
	{"adi,peak-hb-overload-threshold-count",
	 AGC_OFFSETOF(peak.hbOverloadThreshCount), 1, 0, 15, 1},
	{"adi,peak-hb-high-threshold",
	 AGC_OFFSETOF(peak.hbHighThresh), 16383, 0, 16383, 2},
	{"adi,peak-hb-under-range-low-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeLowThresh), 3768, 0, 16383, 2},
	{"adi,peak-hb-under-range-mid-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeMidThresh), 7209, 0, 16383, 2},
	{"adi,peak-hb-under-range-high-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThresh), 10321, 0, 16383, 2},
	{"adi,peak-hb-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-hb-under-range-high-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThreshExceededCount), 3, 0, 255, 1},
	{"adi,peak-hb-gain-step-high-recover",
	 AGC_OFFSETOF(peak.hbGainStepHighRecovery), 4, 0, 31, 1},
	{"adi,peak-hb-gain-step-low-recovery",
	 AGC_OFFSETOF(peak.hbGainStepLowRecovery), 6, 0, 31, 1},
	{"adi,peak-hb-gain-step-mid-recovery",
	 AGC_OFFSETOF(peak.hbGainStepMidRecovery), 4, 0, 31, 1},
	{"adi,peak-hb-gain-step-attack",
	 AGC_OFFSETOF(peak.hbGainStepAttack), 4, 0, 31, 1},
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
				  rx->agc->power.feedback_high_threshold_exceeded);
	if (ret)
		goto out;

	/* feedback pins*/
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-low-thres-gain-change",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->power.feedback_lowThreshold_gainChange);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-high-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->peak.feedback_high_threshold_counter_exceeded);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-low-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc->peak.feedback_low_threshold_counter_exceeded);
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

	ret = ADRV9002_OF_RX_OPTIONAL("adi,min-gain-index", 183, 183, 255,
				      min_gain);
	if (ret)
		return ret;

	ret = ADRV9002_OF_RX_OPTIONAL("adi,max-gain-index", 255, min_gain, 255,
				      max_gain);
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
	struct device_node *of_channels, *of_gpios;
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
			goto of_channels_put;
		} else if (chann > 1) {
			dev_err(&phy->spi->dev,
				"Invalid value for channel: %d\n", chann);
			ret = -EINVAL;
			goto of_channels_put;
		}

		ret = of_property_read_u32(child, "adi,port", &port);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No port property defined for channel\n");
			goto of_channels_put;
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
			goto of_channels_put;
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
			goto of_gpio_put;
		} else if (gpio >= ADI_ADRV9001_GPIO_ANALOG_11) {
			dev_err(&phy->spi->dev,
				"Invalid gpio number: %d\n", gpio);
			ret = -EINVAL;
			goto of_gpio_put;
		}
		/* index 0 is not valid */
		phy->adrv9002_gpios[idx].gpio.pin = gpio + 1;

		ret = of_property_read_u32(child, "adi,signal", &signal);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No adi,signal property defined for gpio%d\n",
				gpio);
			goto of_gpio_put;
		} else if (signal > ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL2) {
			dev_err(&phy->spi->dev,
				"Invalid gpio signal: %d\n", signal);
			ret = -EINVAL;
			goto of_gpio_put;
		}
		phy->adrv9002_gpios[idx].signal = signal;

		ret = of_property_read_u32(child, "adi,polarity", &polarity);
		if (!ret) {
			if (polarity > ADI_ADRV9001_GPIO_POLARITY_INVERTED) {
				dev_err(&phy->spi->dev,
					"Invalid gpio polarity: %d\n",
					polarity);

				ret = -EINVAL;
				goto of_gpio_put;
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
				goto of_gpio_put;
			}
			phy->adrv9002_gpios[idx].gpio.master = master;
		} else {
			ret = 0;
		}

		idx++;
	}

of_gpio_put:
	of_node_put(of_gpios);
of_channels_put:
	of_node_put(of_channels);
	return ret;
}

#ifdef ADI_DYNAMIC_PROFILE_LOAD
static int adrv9002_profile_update(struct adrv9002_rf_phy *phy)
{
	int ret;

	adrv9002_cleanup(phy);
	ret = adrv9002_setup(phy, &phy->profile);
	if (ret) {
		/* try one more time */
		ret = adrv9002_setup(phy, &phy->profile);
		if (ret)
			return ret;
	}

	ret = adrv9002_ssi_configure(phy);
	if (ret)
		return ret;

	return adrv9002_intf_tuning_unlocked(phy);
}

static ssize_t
adrv9002_profile_bin_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	if (off == 0) {
		if (!phy->bin_attr_buf) {
			phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev,
							 bin_attr->size,
							 GFP_KERNEL);
			if (!phy->bin_attr_buf)
				return -ENOMEM;
		} else {
			memset(phy->bin_attr_buf, 0, bin_attr->size);
		}
	}

	memcpy(phy->bin_attr_buf + off, buf, count);

	if (!strnstr(phy->bin_attr_buf, "\n}", off + count))
		return count;

	dev_dbg(&phy->spi->dev, "%s:%d: size %lld\n", __func__, __LINE__,
		off + count);

	mutex_lock(&phy->lock);

	memset(&phy->profile, 0, sizeof(phy->profile));
	ret = adi_adrv9001_Utilities_DeviceProfile_Parse(phy->adrv9001,
							 &phy->profile,
							 phy->bin_attr_buf,
							 off + count);
	if (ret)
		goto out;

	ret = adrv9002_profile_update(phy);
out:
	mutex_unlock(&phy->lock);

	return (ret < 0) ? ret : count;
}
#else
static ssize_t
adrv9002_profile_bin_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	return -ENOTSUPP;
}
#endif

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

int adrv9002_post_init(struct adrv9002_rf_phy *phy)
{
	struct adi_common_ApiVersion api_version;
	struct adi_adrv9001_ArmVersion arm_version;
	struct adi_adrv9001_SiliconVersion silicon_version;
	int ret;
	struct spi_device *spi = phy->spi;
	struct iio_dev *indio_dev = phy->indio_dev;

	ret = adrv9002_setup(phy, adrv9002_init_get());
	if (ret < 0)
		return ret;

	adrv9002_clk_register(phy, "-rx1_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      RX1_SAMPL_CLK);

	adrv9002_clk_register(phy, "-rx2_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      RX2_SAMPL_CLK);

	adrv9002_clk_register(phy, "-tx1_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      TX1_SAMPL_CLK);

	adrv9002_clk_register(phy, "-tx2_sampl_clk", NULL, NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      TX2_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_ADRV9002_CLKS;

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_onecell_get,
				  &phy->clk_data);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adrv9002_of_clk_del_provider,
				       &spi->dev);
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
	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		return ret;

	adi_adrv9001_ApiVersion_Get(phy->adrv9001, &api_version);
	adi_adrv9001_arm_Version(phy->adrv9001, &arm_version);
	adi_adrv9001_SiliconVersion_Get(phy->adrv9001, &silicon_version);

	dev_info(&spi->dev,
		 "%s Rev %d.%d, Firmware %u.%u.%u.%u API version: %u.%u.%u successfully initialized",
		 indio_dev->name, silicon_version.major, silicon_version.minor,
		 arm_version.majorVer, arm_version.minorVer,
		 arm_version.maintVer, arm_version.rcVer, api_version.major,
		 api_version.minor, api_version.patch);

	adrv9002_debugfs_create(phy);

	return 0;
}
EXPORT_SYMBOL(adrv9002_post_init);

static int adrv9002_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9002_rf_phy *phy;
	struct clk *clk = NULL;
	int ret;
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
	phy->hal.phy = phy;
	phy->spi_device_id = *id;
	if (phy->spi_device_id == ID_ADRV9002_RX2TX2)
		phy->rx2tx2 = true;

	mutex_init(&phy->lock);
	phy->adrv9001 = &phy->adrv9001_device;
	phy->hal.spi = spi;
	phy->adrv9001->common.devHalInfo = &phy->hal;

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
