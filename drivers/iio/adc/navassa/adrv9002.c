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
#include <linux/iio/sysfs.h>
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
#include "adi_adrv9001_auxadc.h"
#include "adi_adrv9001_auxadc_types.h"
#include "adi_adrv9001_bbdc.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_common_types.h"
#include "adi_adrv9001_auxdac.h"
#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_fh.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_orx.h"
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

#define ALL_RX_CHANNEL_MASK	(ADI_ADRV9001_RX1 | ADI_ADRV9001_RX2 | \
				 ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2)

#define ADRV9002_RX_EN(nr)	BIT(((nr) * 2) & 0x3)
#define ADRV9002_TX_EN(nr)	BIT(((nr) * 2 + 1) & 0x3)

#define ADRV9002_RX_MAX_GAIN_mdB	\
	((ADI_ADRV9001_RX_GAIN_INDEX_MAX - ADI_ADRV9001_RX_GAIN_INDEX_MIN) *	\
	 ADRV9002_RX_GAIN_STEP_mDB)
#define ADRV9002_RX_GAIN_STEP_mDB	500
#define ADRV9002_RX_MIN_GAIN_IDX	ADI_ADRV9001_RX_GAIN_INDEX_MIN
#define ADRV9002_RX_MAX_GAIN_IDX	ADI_ADRV9001_RX_GAIN_INDEX_MAX
/* ORx gain defines */
#define ADRV9002_ORX_GAIN_STEP_mDB	5000
#define ADRV9002_ORX_MIN_GAIN_IDX	ADI_ADRV9001_ORX_GAIN_INDEX_MIN
#define ADRV9002_ORX_MAX_GAIN_IDX	ADI_ADRV9001_ORX_GAIN_INDEX_MAX
/*
 * the Orx tables indexes are the same in a x2 step. And only the even index will actually
 * take effect in the device. That's why we divide by 2...
 */
#define ADRV9002_ORX_MAX_GAIN_DROP_mdB	\
	((ADI_ADRV9001_ORX_GAIN_INDEX_MAX - ADI_ADRV9001_ORX_GAIN_INDEX_MIN) / 2 \
	 * ADRV9002_ORX_GAIN_STEP_mDB)
#define ADRV9002_ORX_MIN_GAIN_mdB	({ \
	BUILD_BUG_ON(ADRV9002_ORX_MAX_GAIN_DROP_mdB > ADRV9002_RX_MAX_GAIN_mdB); \
	(ADRV9002_RX_MAX_GAIN_mdB - ADRV9002_ORX_MAX_GAIN_DROP_mdB); \
})

#define ADRV9002_STREAM_BINARY_SZ	ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES
#define ADRV9002_HP_CLK_PLL_DAHZ	884736000

/* Frequency hopping */
#define ADRV9002_FH_TABLE_COL_SZ	7

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

static void adrv9002_get_ssi_interface(struct adrv9002_rf_phy *phy, const int chann,
				       const bool tx, u8 *n_lanes, bool *cmos_ddr_en)
{
	if (tx) {
		adi_adrv9001_TxProfile_t *tx_cfg;

		tx_cfg = &phy->curr_profile->tx.txProfile[chann];
		*n_lanes = tx_cfg->txSsiConfig.numLaneSel;
		*cmos_ddr_en = tx_cfg->txSsiConfig.ddrEn;
	} else {
		adi_adrv9001_RxProfile_t *rx_cfg;

		rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chann].profile;
		*n_lanes = rx_cfg->rxSsiConfig.numLaneSel;
		*cmos_ddr_en = rx_cfg->rxSsiConfig.ddrEn;
	}
}

static int adrv9002_ssi_configure(struct adrv9002_rf_phy *phy)
{
	bool cmos_ddr;
	u8 n_lanes;
	int c, ret;

	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		struct adrv9002_chan *chann = phy->channels[c];

		/* RX2/TX2 can only be enabled if RX1/TX1 are also enabled */
		if (phy->rx2tx2 && chann->idx > ADRV9002_CHANN_1)
			break;

		if (!chann->enabled)
			continue;

		adrv9002_sync_gpio_toggle(phy);

		adrv9002_get_ssi_interface(phy, chann->idx, chann->port == ADI_TX, &n_lanes,
					   &cmos_ddr);
		ret = adrv9002_axi_interface_set(phy, n_lanes, cmos_ddr, chann->idx,
						 chann->port == ADI_TX);
		if (ret)
			return ret;

		/*
		 * We should set the tdd rate on TX's iterations since only at this point we
		 * have the up to date dds rate. Moreover it does not make sense to do any
		 * tdd configuration if both TX/RX on the same channel are not enabled.
		 */
		if (chann->port == ADI_TX) {
			struct adrv9002_rx_chan *rx = &phy->rx_channels[chann->idx];
			unsigned long rate;

			if (!rx->channel.enabled)
				continue;

			rate = adrv9002_axi_dds_rate_get(phy, chann->idx) * rx->channel.rate;
			clk_set_rate(rx->tdd_clk, rate);
		}
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
		len = strscpy(dest, dev_name(&phy->spi->dev),
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
	int c;

	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		const struct adrv9002_chan *chan = phy->channels[c];

		if (!chan->enabled)
			continue;

		clk_set_rate(chan->clk, chan->rate);
	}
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static int adrv9002_gainidx_to_gain(int idx, int port)
{
	int gain;

	if (port  == ADI_RX) {
		idx = clamp(idx, ADRV9002_RX_MIN_GAIN_IDX, ADRV9002_RX_MAX_GAIN_IDX);
		gain = (idx - ADRV9002_RX_MIN_GAIN_IDX) * ADRV9002_RX_GAIN_STEP_mDB;
	} else {
		/* ADI_ORX - look at the ORX defines for why we have the div/mult by 2 */
		idx = clamp(idx, ADRV9002_ORX_MIN_GAIN_IDX, ADRV9002_ORX_MAX_GAIN_IDX);
		gain = (idx - ADRV9002_ORX_MIN_GAIN_IDX) / 2 * ADRV9002_ORX_GAIN_STEP_mDB +
			ADRV9002_ORX_MIN_GAIN_mdB;
	}

	return gain;
}

static int adrv9002_gain_to_gainidx(int gain, int port)
{
	int temp;

	if (port  == ADI_RX) {
		gain = clamp(gain, 0, ADRV9002_RX_MAX_GAIN_mdB);
		temp = DIV_ROUND_CLOSEST(gain, ADRV9002_RX_GAIN_STEP_mDB);
		temp += ADRV9002_RX_MIN_GAIN_IDX;
	} else {
		/* ADI_ORX */
		gain = clamp(gain, ADRV9002_ORX_MIN_GAIN_mdB, ADRV9002_RX_MAX_GAIN_mdB);
		temp = DIV_ROUND_CLOSEST(gain - ADRV9002_ORX_MIN_GAIN_mdB,
					 ADRV9002_ORX_GAIN_STEP_mDB) * 2;
		temp += ADRV9002_ORX_MIN_GAIN_IDX;
	}

	return temp;
}

static int adrv9002_chan_to_state_poll(struct adrv9002_rf_phy *phy,
				       struct adrv9002_chan *c,
				       const adi_adrv9001_ChannelState_e state,
				       const int n_tries)
{
	int ret;
	adi_adrv9001_ChannelState_e __state;
	int try = 0;

	do {
		ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, c->port, c->number,
							   &__state);
		if (ret)
			return adrv9002_dev_err(phy);

		if (__state == state)
			break;
		usleep_range(1000, 1005);
	} while (++try < n_tries);

	if (try == n_tries)
		return -EBUSY;

	return 0;
}

static bool adrv9002_orx_enabled(const struct adrv9002_rf_phy *phy, const struct adrv9002_chan *c)
{
	const struct adrv9002_rx_chan *rx;

	if (c->port == ADI_RX)
		rx = chan_to_rx(c);
	else
		rx = &phy->rx_channels[c->idx];

	if (!rx->orx_gpio)
		return false;

	return !!gpiod_get_value_cansleep(rx->orx_gpio);
}

int adrv9002_channel_to_state(struct adrv9002_rf_phy *phy, struct adrv9002_chan *chann,
			      const adi_adrv9001_ChannelState_e state, const bool cache_state)
{
	int ret;
	adi_adrv9001_ChannelEnableMode_e mode;

	/* nothing to do */
	if (!chann->enabled)
		return 0;
	/*
	 * if ORx is enabled we are not expected to do any state transition on RX/TX in the
	 * same channel as that might have the non explicit side effect of breaking the
	 * capture in the ORx port. Hence, we should protect against that...
	 */
	if (adrv9002_orx_enabled(phy, chann))
		return -EPERM;

	ret = adi_adrv9001_Radio_ChannelEnableMode_Get(phy->adrv9001, chann->port,
						       chann->number, &mode);
	if (ret)
		return adrv9002_dev_err(phy);

	/* we need to set it to spi */
	if (mode == ADI_ADRV9001_PIN_MODE) {
		ret = adi_adrv9001_Radio_ChannelEnableMode_Set(phy->adrv9001, chann->port,
							       chann->number,
							       ADI_ADRV9001_SPI_MODE);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	if (cache_state) {
		ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, chann->port,
							   chann->number,
							   &chann->cached_state);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, chann->port,
						 chann->number, state);
	if (ret)
		return adrv9002_dev_err(phy);
	/*
	 * Make sure that the channel is really in the state we want as it might take time
	 * for the device to actually do the change (mainly when moving to rf_enabled).
	 */
	ret = adrv9002_chan_to_state_poll(phy, chann, state, 7);
	if (ret) {
		/*
		 * This is important when the device is in PIN mode as changing it to SPI
		 * might trigger a state change to rf_enabled. In that case it looks like the
		 * first call to @adi_adrv9001_Radio_Channel_ToState() is just ignored as the
		 * device is still busy. Hence we try one last time to move the channel to the
		 * desired state and double up the number of tries...
		 */
		dev_dbg(&phy->spi->dev, "Try to change to state(%d) again...\n", state);
		ret = adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, chann->port,
							 chann->number, state);
		if (ret)
			return adrv9002_dev_err(phy);

		ret = adrv9002_chan_to_state_poll(phy, chann, state, 14);
		if (ret)
			return ret;
	}

	if (mode == ADI_ADRV9001_SPI_MODE)
		return 0;

	/* restore enable mode */
	ret = adi_adrv9001_Radio_ChannelEnableMode_Set(phy->adrv9001, chann->port,
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

enum {
	ADRV9002_HOP_1_TABLE_SEL,
	ADRV9002_HOP_2_TABLE_SEL,
	ADRV9002_HOP_1_TRIGGER,
	ADRV9002_HOP_2_TRIGGER
};

static const char * const adrv9002_hop_table[ADRV9002_FH_TABLES_NR + 1] = {
	"TABLE_A",
	"TABLE_B",
	"Unknown"
};

static ssize_t adrv9002_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	adi_adrv9001_FhHopTable_e table;
	adi_adrv9001_FhMode_e mode = phy->fh.mode;
	int ret;

	switch (iio_attr->address) {
	case ADRV9002_HOP_1_TABLE_SEL:
	case ADRV9002_HOP_2_TABLE_SEL:
		mutex_lock(&phy->lock);
		if (!phy->curr_profile->sysConfig.fhModeOn) {
			dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}

		if (iio_attr->address &&
		    mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
			dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}

		ret = adi_adrv9001_fh_HopTable_Get(phy->adrv9001, iio_attr->address, &table);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9002_dev_err(phy);

		if (table >= ADRV9002_FH_TABLES_NR)
			table = ADRV9002_FH_TABLES_NR;

		return sysfs_emit(buf, "%s\n", adrv9002_hop_table[table]);
	default:
		return -EINVAL;
	}
}

static ssize_t adrv9002_attr_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	int ret, tbl;

	mutex_lock(&phy->lock);
	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
		mutex_unlock(&phy->lock);
		return -ENOTSUPP;
	}

	if (iio_attr->address == ADRV9002_HOP_2_TABLE_SEL ||
	    iio_attr->address == ADRV9002_HOP_2_TRIGGER) {
		if (phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
			dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}
	}

	switch (iio_attr->address) {
	case ADRV9002_HOP_1_TABLE_SEL:
	case ADRV9002_HOP_2_TABLE_SEL:
		for (tbl = 0; tbl < ARRAY_SIZE(adrv9002_hop_table) - 1; tbl++) {
			if (sysfs_streq(buf, adrv9002_hop_table[tbl]))
				break;
		}

		if (tbl == ARRAY_SIZE(adrv9002_hop_table) - 1) {
			dev_err(&phy->spi->dev, "Unknown table %s\n", buf);
			mutex_unlock(&phy->lock);
			return -EINVAL;
		}

		ret = adi_adrv9001_fh_HopTable_Set(phy->adrv9001, iio_attr->address, tbl);
		if (ret)
			ret = adrv9002_dev_err(phy);
		break;
	case ADRV9002_HOP_1_TRIGGER:
		ret = adi_adrv9001_fh_Hop(phy->adrv9001, ADI_ADRV9001_FH_HOP_SIGNAL_1);
		break;
	case ADRV9002_HOP_2_TRIGGER:
		ret = adi_adrv9001_fh_Hop(phy->adrv9001, ADI_ADRV9001_FH_HOP_SIGNAL_2);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&phy->lock);
	return ret ? ret : len;
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
	};

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
		ret = adrv9002_channel_to_state(phy, chann, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
		if (ret)
			goto unlock;

		ret = adi_adrv9001_Radio_Carrier_Configure(phy->adrv9001, port,
							   chann->number, &lo_freq);
		if (ret) {
			ret = adrv9002_dev_err(phy);
			goto unlock;
		}

		ret = adrv9002_channel_to_state(phy, chann, chann->cached_state, false);
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

		ret = sysfs_emit(buf, "%llu\n", lo_freq.carrierFrequency_Hz);
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

	if (adrv9002_orx_enabled(phy, chann)) {
		mutex_unlock(&phy->lock);
		return -EPERM;
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

	ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Rx_InterfaceGain_Configure(phy->adrv9001,
						      rx->channel.number,
						      &rx_intf_gain_mode);
	if (ret) {
		ret = adrv9002_dev_err(phy);
		goto unlock;
	}

	ret = adrv9002_channel_to_state(phy, &rx->channel, rx->channel.cached_state, false);
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

	if (adrv9002_orx_enabled(phy, chann)) {
		/*
		 * Don't allow changing port enable mode if ORx is enabled, because it
		 * might trigger an ensm state transition which can potentially break ORx
		 */
		mutex_unlock(&phy->lock);
		return -EPERM;
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
	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		ret = adrv9002_channel_to_state(phy, c, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
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
	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		ret = adrv9002_channel_to_state(phy, c, c->cached_state, false);
		if (ret)
			return ret;
	}

	return 0;
}

static const u32 rx_track_calls[] = {
	[RX_QEC_FIC] = ADI_ADRV9001_TRACKING_CAL_RX_QEC_FIC,
	[RX_QEC_W_POLY] = ADI_ADRV9001_TRACKING_CAL_RX_QEC_WBPOLY,
	[ORX_QEC_W_POLY] = ADI_ADRV9001_TRACKING_CAL_ORX_QEC_WBPOLY,
	[RX_AGC] = ADI_ADRV9001_TRACKING_CAL_RX_GAIN_CONTROL_DETECTORS,
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
	const int port = ADRV_ADDRESS_PORT(chan->address);
	int ret = 0, freq_offset_hz;
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[channel];
	bool enable;
	u32 val;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled && port == ADI_RX) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case ORX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		if (private == ORX_QEC_W_POLY && !rx->orx_en) {
			ret = -ENODEV;
			goto unlock;
		}

		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;

		ret = adrv9002_update_tracking_calls(phy, rx_track_calls[private], channel, enable);
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
		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED,
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

		ret = adrv9002_channel_to_state(phy, &rx->channel, rx->channel.cached_state, false);
		break;
	case RX_BBDC:
		if (!rx->orx_en && port == ADI_ORX) {
			ret = -ENODEV;
			goto unlock;
		}

		ret = kstrtobool(buf, &enable);
		if (ret)
			goto unlock;
		/*
		 * Disabling the bbdc will completely disable the algorithm and set the correction
		 * value to 0. The difference with the tracking cal is that disabling it, just
		 * disables the algorithm but the last used correction value is still applied...
		 */
		ret = adi_adrv9001_bbdc_RejectionEnable_Set(phy->adrv9001, port,
							    rx->channel.number, enable);
		if (ret)
			ret = adrv9002_dev_err(phy);
		break;
	case RX_BBDC_LOOP_GAIN:
		ret = kstrtou32(buf, 10, &val);
		if (ret)
			goto unlock;

		ret = adi_adrv9010_bbdc_LoopGain_Set(phy->adrv9001, rx->channel.number, val);
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
	u32 rssi_pwr_mdb, val;
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	const int port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[channel];
	adi_adrv9001_BbdcRejectionStatus_e bbdc;
	bool enable;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled && port == ADI_RX) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case ORX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		if (private == ORX_QEC_W_POLY && !rx->orx_en) {
			mutex_unlock(&phy->lock);
			return -ENODEV;
		}

		ret = adi_adrv9001_cals_Tracking_Get(phy->adrv9001,
						     &tracking_cals);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sysfs_emit(buf, "%d\n",
				 calls_mask[channel] & rx_track_calls[private] ? 1 : 0);
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

		ret = sysfs_emit(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000, dec_pwr_mdb % 1000);
		break;
	case RX_RSSI:
		ret = adi_adrv9001_Rx_Rssi_Read(phy->adrv9001,
						rx->channel.number, &rssi_pwr_mdb);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sysfs_emit(buf, "%u.%02u dB\n", rssi_pwr_mdb / 1000, rssi_pwr_mdb % 1000);
		break;
	case RX_RF_BANDWIDTH:
		rx_cfg = &phy->curr_profile->rx.rxChannelCfg[chan->channel];
		ret = sysfs_emit(buf, "%u\n", rx_cfg->profile.primarySigBandwidth_Hz);
		break;
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn) {
			mutex_unlock(&phy->lock);
			return -ENOTSUPP;
		}

		ret = sysfs_emit(buf, "%d\n", rx->channel.nco_freq);
		break;
	case RX_ADC_SWITCH:
		ret = adi_adrv9001_Rx_AdcSwitchEnable_Get(phy->adrv9001,
							  rx->channel.number,
							  &enable);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sysfs_emit(buf, "%d\n", enable);
		break;
	case RX_BBDC:
		if (!rx->orx_en && port == ADI_ORX) {
			mutex_unlock(&phy->lock);
			return -ENODEV;
		}

		ret = adi_adrv9001_bbdc_RejectionEnable_Get(phy->adrv9001, port,
							    rx->channel.number, &bbdc);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sysfs_emit(buf, "%d\n", bbdc);
		break;
	case RX_BBDC_LOOP_GAIN:
		ret = adi_adrv9010_bbdc_LoopGain_Get(phy->adrv9001, rx->channel.number, &val);
		if (ret) {
			mutex_unlock(&phy->lock);
			return adrv9002_dev_err(phy);
		}

		ret = sysfs_emit(buf, "%u\n", val);
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
	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Tx_AttenuationMode_Set(phy->adrv9001,
						  tx->channel.number, tx_mode);
	if (ret) {
		ret = adrv9002_dev_err(phy);
		goto unlock;
	}

	ret = adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);
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
		ret = sysfs_emit(buf, "%d\n", val);
		break;
	case TX_RF_BANDWIDTH:
		ret = sysfs_emit(buf, "%d\n", tx_cfg->primarySigBandwidth_Hz);
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

		ret = sysfs_emit(buf, "%d\n", tx->channel.nco_freq);
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
	_ADRV9002_EXT_RX_INFO("bbdc_loop_gain_raw", RX_BBDC_LOOP_GAIN),
	{ },
};

static const struct iio_chan_spec_ext_info adrv9002_phy_orx_ext_info[] = {
	_ADRV9002_EXT_RX_INFO("bbdc_rejection_en", RX_BBDC),
	_ADRV9002_EXT_RX_INFO("quadrature_w_poly_tracking_en", ORX_QEC_W_POLY),
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

static int adrv9002_channel_power_set(struct adrv9002_rf_phy *phy, struct adrv9002_chan *channel,
				      const int val)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Set power: %d, chan: %d, port: %d\n",
		val, channel->number, channel->port);

	if (!val && channel->power) {
		ret = adrv9002_channel_to_state(phy, channel, ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;

		ret = adi_adrv9001_Radio_Channel_PowerDown(phy->adrv9001, channel->port,
							   channel->number);
		if (ret)
			return adrv9002_dev_err(phy);

		channel->power = false;
	} else if (val && !channel->power) {
		ret = adi_adrv9001_Radio_Channel_PowerUp(phy->adrv9001, channel->port,
							 channel->number);
		if (ret)
			return adrv9002_dev_err(phy);

		ret = adrv9002_channel_to_state(phy, channel, channel->cached_state, false);
		if (ret)
			return ret;

		channel->power = true;
	}

	return 0;
}

static int adrv9002_phy_read_raw_no_rf_chan(struct adrv9002_rf_phy *phy,
					    struct iio_chan_spec const *chan,
					    int *val, int *val2, long m)
{
	int ret;
	bool en;
	u16 temp;

	switch (m) {
	case IIO_CHAN_INFO_ENABLE:
		mutex_lock(&phy->lock);
		if (chan->output)
			ret = adi_adrv9001_AuxDac_Inspect(phy->adrv9001, chan->address, &en);
		else
			ret = adi_adrv9001_AuxAdc_Inspect(phy->adrv9001, chan->address, &en);
		if (ret)
			goto error;
		mutex_unlock(&phy->lock);
		*val = en;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_TEMP:
			mutex_lock(&phy->lock);
			ret = adi_adrv9001_Temperature_Get(phy->adrv9001, &temp);
			if (ret)
				goto error;
			mutex_unlock(&phy->lock);
			*val = temp * 1000;
			return IIO_VAL_INT;
		case IIO_VOLTAGE:
			mutex_lock(&phy->lock);
			if (!chan->output) {
				ret = adi_adrv9001_AuxAdc_Voltage_Get(phy->adrv9001, chan->address,
								      &temp);
				if (ret)
					goto error;
				mutex_unlock(&phy->lock);

				*val = temp;
				return IIO_VAL_INT;
			}

			ret = adi_adrv9001_AuxDac_Code_Get(phy->adrv9001, chan->address, &temp);
			if (ret)
				goto error;
			mutex_unlock(&phy->lock);

			*val = 900 + DIV_ROUND_CLOSEST((temp - 2048) * 1700, 4096);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	};

error:
	mutex_unlock(&phy->lock);
	return adrv9002_dev_err(phy);
}

static int adrv9002_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long m)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9002_chan *chann;
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	u16 temp;
	int ret;
	u8 index;

	if (chan->type != IIO_VOLTAGE || chan->channel > ADRV9002_CHANN_2)
		return adrv9002_phy_read_raw_no_rf_chan(phy, chan, val, val2, m);

	chann = adrv9002_get_channel(phy, port, chan_nr);
	mutex_lock(&phy->lock);
	if (port == ADI_ORX) {
		struct adrv9002_rx_chan *rx = chan_to_rx(chann);

		if (!rx->orx_en) {
			mutex_unlock(&phy->lock);
			return -ENODEV;
		}
	} else if (!chann->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
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
			if (port == ADI_ORX)
				ret = adi_adrv9001_ORx_Gain_Get(phy->adrv9001, chann->number,
								&index);
			else
				ret = adi_adrv9001_Rx_Gain_Get(phy->adrv9001, chann->number,
							       &index);
			if (ret) {
				mutex_unlock(&phy->lock);
				return adrv9002_dev_err(phy);
			}

			temp = adrv9002_gainidx_to_gain(index, port);
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
		if (port == ADI_ORX) {
			struct adrv9002_rx_chan *rx = chan_to_rx(chann);

			if (!rx->orx_gpio) {
				mutex_unlock(&phy->lock);
				return -ENOTSUPP;
			}

			*val = gpiod_get_value_cansleep(rx->orx_gpio);
		} else {
			*val = chann->power;
		}
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);
	return ret;
};

static int adrv9002_phy_write_raw_no_rf_chan(struct adrv9002_rf_phy *phy,
					     struct iio_chan_spec const *chan, int val,
					     int val2, long mask)
{
	int ret;
	u16 code;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		mutex_lock(&phy->lock);
		if (chan->output)
			ret = adi_adrv9001_AuxDac_Configure(phy->adrv9001, chan->address, val);
		else
			ret = adi_adrv9001_AuxAdc_Configure(phy->adrv9001, chan->address, val);
		if (ret)
			goto error;
		mutex_unlock(&phy->lock);
		return 0;
	case IIO_CHAN_INFO_PROCESSED:
		code = clamp_val(val, 50, 1750);
		code = 2048 + DIV_ROUND_CLOSEST((code - 900) * 4096, 1700);
		if (code == 4096)
			code = 4095;

		mutex_lock(&phy->lock);
		ret = adi_adrv9001_AuxDac_Code_Set(phy->adrv9001, chan->address, code);
		if (ret)
			goto error;
		mutex_unlock(&phy->lock);
		return 0;
	default:
		return -EINVAL;
	}

error:
	mutex_unlock(&phy->lock);
	return adrv9002_dev_err(phy);
}

static bool adrv9002_orx_can_enable(const struct adrv9002_rf_phy *phy,
				    const struct adrv9002_chan *rx)
{
	adi_adrv9001_ChannelState_e state;
	int ret;

	/* We can't enable ORx if RX is already enabled */
	ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, ADI_RX, rx->number, &state);
	if (ret) {
		adrv9002_dev_err(phy);
		return false;
	}

	if (state == ADI_ADRV9001_CHANNEL_RF_ENABLED) {
		dev_err(&phy->spi->dev, "RX%d cannot be enabled in order to enable ORx%d\n",
			rx->number, rx->number);
		return false;
	}

	/*
	 * TX must be enabled in order to enable ORx. We just use the rx object as we are
	 * interested in the TX on the same channel
	 */
	ret = adi_adrv9001_Radio_Channel_State_Get(phy->adrv9001, ADI_TX, rx->number, &state);
	if (ret) {
		adrv9002_dev_err(phy);
		return false;
	}

	if (state != ADI_ADRV9001_CHANNEL_RF_ENABLED) {
		dev_err(&phy->spi->dev, "TX%d must be enabled in order to enable ORx%d\n",
			rx->number, rx->number);
		return false;
	}

	return true;
}

static int adrv9002_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int val,
				  int val2, long mask)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_chan *chann;
	u32 code;
	int ret = 0;

	if (chan->type != IIO_VOLTAGE || chan->channel > ADRV9002_CHANN_2)
		return adrv9002_phy_write_raw_no_rf_chan(phy, chan, val, val2, mask);

	mutex_lock(&phy->lock);
	chann = adrv9002_get_channel(phy, port, chan_nr);
	if (port == ADI_ORX) {
		struct adrv9002_rx_chan *rx = chan_to_rx(chann);

		if (!rx->orx_en) {
			mutex_unlock(&phy->lock);
			return -ENODEV;
		}
	} else if (!chann->enabled) {
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
			idx = adrv9002_gain_to_gainidx(gain, port);
			if (port == ADI_RX)
				ret = adi_adrv9001_Rx_Gain_Set(phy->adrv9001, chann->number, idx);
			else
				ret = adi_adrv9001_ORx_Gain_Set(phy->adrv9001, chann->number, idx);
		}

		if (ret)
			ret = adrv9002_dev_err(phy);
		break;
	case IIO_CHAN_INFO_ENABLE:
		if (port == ADI_ORX) {
			struct adrv9002_rx_chan *rx = chan_to_rx(chann);

			if (!rx->orx_gpio) {
				mutex_unlock(&phy->lock);
				return -ENOTSUPP;
			}

			if (val && !adrv9002_orx_can_enable(phy, chann)) {
				mutex_unlock(&phy->lock);
				return -EPERM;
			}
			gpiod_set_value_cansleep(rx->orx_gpio, !!val);
		} else {
			ret = adrv9002_channel_power_set(phy, chann, val);
		}
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

#define ADRV9002_IIO_ORX_CHAN(idx, port, chan) {		\
	.type = IIO_VOLTAGE,                                    \
	.indexed = 1,                                           \
	.channel = idx,                                         \
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
			BIT(IIO_CHAN_INFO_ENABLE),		\
	.extend_name = "orx",                                   \
	.ext_info = adrv9002_phy_orx_ext_info,                  \
	.address = ADRV_ADDRESS(port, chan),                    \
}

#define ADRV9002_IIO_AUX_CONV_CHAN(idx, out, chan) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = idx,						\
	.output = out,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |	\
			BIT(IIO_CHAN_INFO_ENABLE),		\
	.address = chan,					\
}

static const struct iio_chan_spec adrv9002_phy_chan[] = {
	ADRV9002_IIO_LO_CHAN(0, "RX1_LO", ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_LO_CHAN(1, "RX2_LO", ADI_RX, ADRV9002_CHANN_2),
	ADRV9002_IIO_LO_CHAN(2, "TX1_LO", ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_LO_CHAN(3, "TX2_LO", ADI_TX, ADRV9002_CHANN_2),
	ADRV9002_IIO_TX_CHAN(0, ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_TX_CHAN(1, ADI_TX, ADRV9002_CHANN_2),
	ADRV9002_IIO_AUX_CONV_CHAN(2, true, ADI_ADRV9001_AUXDAC0),
	ADRV9002_IIO_AUX_CONV_CHAN(3, true, ADI_ADRV9001_AUXDAC1),
	ADRV9002_IIO_AUX_CONV_CHAN(4, true, ADI_ADRV9001_AUXDAC2),
	ADRV9002_IIO_AUX_CONV_CHAN(5, true, ADI_ADRV9001_AUXDAC3),
	ADRV9002_IIO_RX_CHAN(0, ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_RX_CHAN(1, ADI_RX, ADRV9002_CHANN_2),
	/*
	 * as Orx shares the same data path as Rx, let's just point
	 * them to the same Rx index...
	 */
	ADRV9002_IIO_ORX_CHAN(0, ADI_ORX, ADRV9002_CHANN_1),
	ADRV9002_IIO_ORX_CHAN(1, ADI_ORX, ADRV9002_CHANN_2),
	ADRV9002_IIO_AUX_CONV_CHAN(2, false, ADI_ADRV9001_AUXADC0),
	ADRV9002_IIO_AUX_CONV_CHAN(3, false, ADI_ADRV9001_AUXADC1),
	ADRV9002_IIO_AUX_CONV_CHAN(4, false, ADI_ADRV9001_AUXADC2),
	ADRV9002_IIO_AUX_CONV_CHAN(5, false, ADI_ADRV9001_AUXADC3),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static IIO_CONST_ATTR(frequency_hopping_hop_table_select_available, "TABLE_A TABLE_B");
static IIO_DEVICE_ATTR(frequency_hopping_hop1_table_select, 0600, adrv9002_attr_show,
		       adrv9002_attr_store, ADRV9002_HOP_1_TABLE_SEL);
static IIO_DEVICE_ATTR(frequency_hopping_hop2_table_select, 0600, adrv9002_attr_show,
		       adrv9002_attr_store, ADRV9002_HOP_2_TABLE_SEL);
static IIO_DEVICE_ATTR(frequency_hopping_hop1_signal_trigger, 0200, NULL, adrv9002_attr_store,
		       ADRV9002_HOP_1_TRIGGER);
static IIO_DEVICE_ATTR(frequency_hopping_hop2_signal_trigger, 0200, NULL, adrv9002_attr_store,
		       ADRV9002_HOP_2_TRIGGER);

static struct attribute *adrv9002_sysfs_attrs[] = {
	&iio_const_attr_frequency_hopping_hop_table_select_available.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop1_table_select.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop2_table_select.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop1_signal_trigger.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop2_signal_trigger.dev_attr.attr,
	NULL
};

static const struct attribute_group adrv9002_sysfs_attrs_group = {
	.attrs = adrv9002_sysfs_attrs,
};

static const struct iio_info adrv9002_phy_info = {
	.attrs = &adrv9002_sysfs_attrs_group,
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
		ret = adi_adrv9001_Rx_GainControl_Configure(phy->adrv9001,
							    rx->channel.number, &rx->agc);
		if (ret)
			return adrv9002_dev_err(phy);

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

static void adrv9002_compute_init_cals(struct adrv9002_rf_phy *phy)
{
	int i, pos = 0;

	phy->init_cals.sysInitCalMask = 0;
	phy->init_cals.calMode = ADI_ADRV9001_INIT_CAL_MODE_ALL;

	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		if (!c->enabled)
			continue;

		if (c->port == ADI_RX)
			pos |= ADRV9002_RX_EN(c->idx);
		else
			pos |= ADRV9002_TX_EN(c->idx);
	}

	phy->init_cals.chanInitCalMask[0] = adrv9002_init_cals_mask[pos][0];
	phy->init_cals.chanInitCalMask[1] = adrv9002_init_cals_mask[pos][1];

	dev_dbg(&phy->spi->dev, "pos: %u, Chan1:%X, Chan2:%X", pos,
		phy->init_cals.chanInitCalMask[0],
		phy->init_cals.chanInitCalMask[1]);
}

static int adrv9002_validate_profile(struct adrv9002_rf_phy *phy)
{
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	const struct adi_adrv9001_TxProfile *tx_cfg = phy->curr_profile->tx.txProfile;
	const u32 tx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_TX1, ADI_ADRV9001_TX2
	};
	const u32 rx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_RX1, ADI_ADRV9001_RX2
	};
	const u32 orx_channels[ADRV9002_CHANN_MAX] = {
		ADI_ADRV9001_ORX1, ADI_ADRV9001_ORX2
	};
	int i;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_chan *tx = &phy->tx_channels[i].channel;
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		/* rx validations */
		if (!ADRV9001_BF_EQUAL(phy->curr_profile->rx.rxInitChannelMask, rx_channels[i]))
			goto tx;

		if (phy->rx2tx2 && i &&
		    rx_cfg[i].profile.rxOutputRate_Hz != phy->rx_channels[0].channel.rate) {
			dev_err(&phy->spi->dev, "In rx2tx2, RX%d rate=%u must be equal to RX1, rate=%ld\n",
				i + 1, rx_cfg[i].profile.rxOutputRate_Hz,
				phy->rx_channels[0].channel.rate);
			return -EINVAL;
		}

		if (phy->rx2tx2 && i && !phy->rx_channels[0].channel.enabled) {
			dev_err(&phy->spi->dev, "In rx2tx2, RX%d cannot be enabled while RX1 is disabled",
				i + 1);
			return -EINVAL;
		}

		if (phy->ssi_type != rx_cfg[i].profile.rxSsiConfig.ssiType) {
			dev_err(&phy->spi->dev, "SSI interface mismatch. PHY=%d, RX%d=%d\n",
				phy->ssi_type, i + 1, rx_cfg[i].profile.rxSsiConfig.ssiType);
			return -EINVAL;
		}

		if (rx_cfg[i].profile.rxSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
			dev_err(&phy->spi->dev, "SSI interface Long Strobe not supported\n");
			return -EINVAL;
		}

		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS &&
		    !rx_cfg[i].profile.rxSsiConfig.ddrEn) {
			dev_err(&phy->spi->dev, "RX%d: Single Data Rate port not supported for LVDS\n",
				i + 1);
			return -EINVAL;
		}

		dev_dbg(&phy->spi->dev, "RX%d enabled\n", i + 1);
		rx->channel.power = true;
		rx->channel.enabled = true;
		rx->channel.nco_freq = 0;
		rx->channel.rate = rx_cfg[i].profile.rxOutputRate_Hz;
tx:
		/* tx validations*/
		if (!ADRV9001_BF_EQUAL(phy->curr_profile->tx.txInitChannelMask, tx_channels[i]))
			continue;

		/* check @tx_only comments in adrv9002.h to better understand the next checks */
		if (phy->ssi_type != tx_cfg[i].txSsiConfig.ssiType) {
			dev_err(&phy->spi->dev, "SSI interface mismatch. PHY=%d, TX%d=%d\n",
				phy->ssi_type, i + 1,  tx_cfg[i].txSsiConfig.ssiType);
			return -EINVAL;
		}

		if (tx_cfg[i].txSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
			dev_err(&phy->spi->dev, "SSI interface Long Strobe not supported\n");
			return -EINVAL;
		}

		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS &&
		    !tx_cfg[i].txSsiConfig.ddrEn) {
			dev_err(&phy->spi->dev, "TX%d: Single Data Rate port not supported for LVDS\n",
				i + 1);
			return -EINVAL;
		}

		if (phy->rx2tx2) {
			if (!phy->tx_only && !phy->rx_channels[0].channel.enabled) {
				/*
				 * pretty much means that in this case either all channels are
				 * disabled, which obviously does not make sense, or RX1 must
				 * be enabled...
				 */
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d cannot be enabled while RX1 is disabled",
					i + 1);
				return -EINVAL;
			}

			if (i && !phy->tx_channels[0].channel.enabled) {
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d cannot be enabled while TX1 is disabled",
					i + 1);
				return -EINVAL;
			}

			if (!phy->tx_only &&
			    tx_cfg[i].txInputRate_Hz != phy->rx_channels[0].channel.rate) {
				/*
				 * pretty much means that in this case, all ports must have
				 * the same rate. We match against RX1 since RX2 can be disabled
				 * even if it does not make much sense to disable it in rx2tx2 mode
				 */
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d rate=%u must be equal to RX1, rate=%ld\n",
					i + 1, tx_cfg[i].txInputRate_Hz,
					phy->rx_channels[0].channel.rate);
				return -EINVAL;
			}

			if (phy->tx_only && i &&
			    tx_cfg[i].txInputRate_Hz != phy->tx_channels[0].channel.rate) {
				dev_err(&phy->spi->dev, "In rx2tx2, TX%d rate=%u must be equal to TX1, rate=%ld\n",
					i + 1, tx_cfg[i].txInputRate_Hz,
					phy->tx_channels[0].channel.rate);
				return -EINVAL;
			}
		} else if (!phy->tx_only && !rx->channel.enabled) {
			dev_err(&phy->spi->dev, "TX%d cannot be enabled while RX%d is disabled",
				i + 1, i + 1);
			return -EINVAL;
		} else if (!phy->tx_only && tx_cfg[i].txInputRate_Hz != rx->channel.rate) {
			dev_err(&phy->spi->dev, "TX%d rate=%u must be equal to RX%d, rate=%ld\n",
				i + 1, tx_cfg[i].txInputRate_Hz, i + 1, rx->channel.rate);
			return -EINVAL;
		}

		dev_dbg(&phy->spi->dev, "TX%d enabled\n", i + 1);
		/* orx actually depends on whether or not TX is enabled and not RX */
		rx->orx_en = ADRV9001_BF_EQUAL(phy->curr_profile->rx.rxInitChannelMask,
					       orx_channels[i]);
		tx->power = true;
		tx->enabled = true;
		tx->nco_freq = 0;
		tx->rate = tx_cfg[i].txInputRate_Hz;
	}

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
	int spi_mode = ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252;
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
						      phy->stream_size, spi_mode);
	else
		ret = adi_adrv9001_Utilities_StreamImage_Load(phy->adrv9001, "Navassa_Stream.bin",
							      spi_mode);
	if (ret)
		return adrv9002_dev_err(phy);

	/* program arm firmware */
	ret = adi_adrv9001_Utilities_ArmImage_Load(phy->adrv9001, "Navassa_EvaluationFw.bin",
						   spi_mode);
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
		const char *rx_table;

		if (rx->orx_en || tx->channel.enabled) {
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

		rx_table = p->gainTableType ? "RxGainTable_GainCompensated.csv" : "RxGainTable.csv";
		ret = adi_adrv9001_Utilities_RxGainTable_Load(phy->adrv9001, ADI_RX, rx_table,
							      rx->channel.number, &p->lnaConfig,
							      t_type);
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

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_LO1,
						   &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_LO2,
						   &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adi_adrv9001_Radio_PllLoopFilter_Set(phy->adrv9001, ADI_ADRV9001_PLL_AUX,
						   &pll_loop_filter);
	if (ret)
		return adrv9002_dev_err(phy);

	for (chan = 0; chan < ARRAY_SIZE(phy->channels); chan++) {
		const struct adrv9002_chan *c = phy->channels[chan];
		struct adi_adrv9001_ChannelEnablementDelays en_delays;

		if (!c->enabled)
			continue;

		/*
		 * For some low rate profiles, the intermediate frequency is non 0.
		 * In these cases, forcing it 0, will cause a firmware error. Hence, we need to
		 * read what we have and make sure we just change the carrier frequency...
		 */
		ret = adi_adrv9001_Radio_Carrier_Inspect(phy->adrv9001, c->port, c->number,
							 &carrier);
		if (ret)
			return adrv9002_dev_err(phy);

		if (c->port == ADI_RX)
			carrier.carrierFrequency_Hz = 2400000000ULL;
		else
			carrier.carrierFrequency_Hz = 2450000000ULL;

		ret = adi_adrv9001_Radio_Carrier_Configure(phy->adrv9001, c->port, c->number,
							   &carrier);
		if (ret)
			return adrv9002_dev_err(phy);

		adrv9002_en_delays_ns_to_arm(phy, &c->en_delays_ns, &en_delays);
		ret = adi_adrv9001_Radio_ChannelEnablementDelays_Configure(phy->adrv9001, c->port,
									   c->number, &en_delays);
		if (ret)
			return adrv9002_dev_err(phy);
	}

	ret = adi_adrv9001_arm_System_Program(phy->adrv9001, channel_mask);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static struct adi_adrv9001_SpiSettings adrv9002_spi = {
	.msbFirst = 1,
	.enSpiStreaming = 0,
	.autoIncAddrUp = 1,
	.fourWireMode = 1,
	.cmosPadDrvStrength = ADI_ADRV9001_CMOSPAD_DRV_STRONG,
};

static int adrv9002_setup(struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_Device *adrv9001_device = phy->adrv9001;
	u8 init_cals_error = 0;
	int ret;
	adi_adrv9001_ChannelState_e init_state;

	/* in TDD we cannot start with all ports enabled as RX/TX cannot be on at the same time */
	if (phy->curr_profile->sysConfig.duplexMode == ADI_ADRV9001_TDD_MODE)
		init_state = ADI_ADRV9001_CHANNEL_PRIMED;
	else
		init_state = ADI_ADRV9001_CHANNEL_RF_ENABLED;

	adi_common_ErrorClear(&phy->adrv9001->common);
	ret = adi_adrv9001_HwOpen(adrv9001_device, &adrv9002_spi);
	if (ret)
		return adrv9002_dev_err(phy);

	ret = adrv9002_validate_profile(phy);
	if (ret)
		return ret;

	adrv9002_compute_init_cals(phy);

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

	if (phy->curr_profile->sysConfig.fhModeOn) {
		ret = adi_adrv9001_fh_Configure(phy->adrv9001, &phy->fh);
		if (ret)
			return adrv9002_dev_err(phy);
	}

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
			       u8 data_delay, const bool tx)
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
		if (phy->rx2tx2) {
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

	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, phy->ssi_type, &delays);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

int adrv9002_check_tx_test_pattern(struct adrv9002_rf_phy *phy, const int chann)
{
	int ret;
	struct adrv9002_chan *chan = &phy->tx_channels[chann].channel;
	adi_adrv9001_SsiTestModeData_e test_data = phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ?
						ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE :
						ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7;
	struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};
	struct adi_adrv9001_TxSsiTestModeStatus status = {0};
	adi_adrv9001_SsiDataFormat_e data_fmt = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA;

	cfg.testData = test_data;

	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001, chan->number,
							  phy->ssi_type, data_fmt, &cfg, &status);
	if (ret)
		return adrv9002_dev_err(phy);

	dev_dbg(&phy->spi->dev, "[c%d]: d_e:%u, f_f:%u f_e:%u, s_e:%u", chan->number,
		status.dataError, status.fifoFull, status.fifoEmpty, status.strobeAlignError);

	/* only looking for data errors for now */
	if (status.dataError)
		return 1;

	if (!phy->rx2tx2)
		return 0;

	/* on rx2tx2 we will only get here on index 0 so the following is fine */
	chan = &phy->tx_channels[chann + 1].channel;
	if (!chan->enabled)
		return 0;

	memset(&status, 0, sizeof(status));
	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001, chan->number,
							  phy->ssi_type, data_fmt, &cfg, &status);
	if (ret)
		return adrv9002_dev_err(phy);

	dev_dbg(&phy->spi->dev, "[c%d]: d_e:%u, f_f:%u f_e:%u, s_e:%u", chan->number,
		status.dataError, status.fifoFull, status.fifoEmpty, status.strobeAlignError);

	if (status.dataError)
		return 1;

	return 0;
}

int adrv9002_intf_test_cfg(struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop)
{
	int ret;
	struct adrv9002_chan *chan;
	adi_adrv9001_SsiDataFormat_e data_fmt = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA;

	dev_dbg(&phy->spi->dev, "cfg test stop:%u, ssi:%d, c:%d, tx:%d\n", stop,
		phy->ssi_type, chann, tx);

	if (tx) {
		struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};

		chan = &phy->tx_channels[chann].channel;

		if (stop)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL;
		else if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
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

		ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, chan->number,
							     phy->ssi_type, data_fmt, &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

		if (!phy->rx2tx2)
			return 0;

		/* on rx2tx2 we will only get here on index 0 so the following is fine */
		chan = &phy->tx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, chan->number,
							     phy->ssi_type, data_fmt, &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

	} else {
		struct adi_adrv9001_RxSsiTestModeCfg cfg = {0};

		chan = &phy->rx_channels[chann].channel;

		if (stop)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL;
		else if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;
		else
			/* CMOS */
			cfg.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE;

		ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, chan->number,
							     phy->ssi_type, data_fmt, &cfg);
		if (ret)
			return adrv9002_dev_err(phy);

		if (!phy->rx2tx2)
			return 0;

		/* on rx2tx2 we will only get here on index 0 so the following is fine */
		chan = &phy->rx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, chan->number,
							     phy->ssi_type, data_fmt, &cfg);
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
	int i;

	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		if (!c->enabled)
			continue;
		if (phy->rx2tx2 && c->idx) {
			/*
			 * In rx2tx2 we should treat both channels as the same. Hence, we will run
			 * the test simultaneosly for both and configure the same delays.
			 */
			if (c->port == ADI_RX) {
				/* RX0 must be enabled, hence we can safely skip further tuning */
				delays.rxClkDelay[c->idx] = delays.rxClkDelay[0];
				delays.rxIDataDelay[c->idx] = delays.rxIDataDelay[0];
				delays.rxQDataDelay[c->idx] = delays.rxQDataDelay[0];
				delays.rxStrobeDelay[c->idx] = delays.rxStrobeDelay[0];
			} else {
				/* TX0 must be enabled, hence we can safely skip further tuning */
				delays.txClkDelay[c->idx] = delays.txClkDelay[0];
				delays.txIDataDelay[c->idx] = delays.txIDataDelay[0];
				delays.txQDataDelay[c->idx] = delays.txQDataDelay[0];
				delays.txStrobeDelay[c->idx] = delays.txStrobeDelay[0];
			}

			continue;
		}

		ret = adrv9002_axi_intf_tune(phy, c->port == ADI_TX, c->idx, &clk_delay,
					     &data_delay);
		if (ret)
			return ret;

		if (c->port == ADI_RX) {
			dev_dbg(&phy->spi->dev, "RX: Got clk: %u, data: %u\n", clk_delay,
				data_delay);
			delays.rxClkDelay[c->idx] = clk_delay;
			delays.rxIDataDelay[c->idx] = data_delay;
			delays.rxQDataDelay[c->idx] = data_delay;
			delays.rxStrobeDelay[c->idx] = data_delay;
		} else {
			dev_dbg(&phy->spi->dev, "TX: Got clk: %u, data: %u\n", clk_delay,
				data_delay);
			delays.txClkDelay[c->idx] = clk_delay;
			delays.txIDataDelay[c->idx] = data_delay;
			delays.txQDataDelay[c->idx] = data_delay;
			delays.txStrobeDelay[c->idx] = data_delay;
		}
	}

	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, phy->ssi_type, &delays);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static void adrv9002_cleanup(struct adrv9002_rf_phy *phy)
{
	int i;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		phy->rx_channels[i].orx_en = 0;
		/* make sure we have the ORx GPIO low */
		if (phy->rx_channels[i].orx_gpio)
			gpiod_set_value_cansleep(phy->rx_channels[i].orx_gpio, 0);
		phy->rx_channels[i].channel.enabled = 0;
		phy->tx_channels[i].channel.enabled = 0;
	}

	memset(&phy->adrv9001->devStateInfo, 0,
	       sizeof(phy->adrv9001->devStateInfo));
}

static u32 adrv9002_get_arm_clk(const struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_ClockSettings *clks = &phy->curr_profile->clocks;
	u32 sys_clk;

	/* HP clk PLL is 8.8GHz and LP is 4.4GHz */
	if (clks->clkPllVcoFreq_daHz == ADRV9002_HP_CLK_PLL_DAHZ)
		sys_clk = clks->clkPllVcoFreq_daHz / 48 * 10;
	else
		sys_clk = clks->clkPllVcoFreq_daHz / 24 * 10;

	return DIV_ROUND_CLOSEST(sys_clk, clks->armPowerSavingClkDiv);
}

void adrv9002_en_delays_ns_to_arm(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d_ns,
				  struct adi_adrv9001_ChannelEnablementDelays *d)
{
	u32 arm_clk = adrv9002_get_arm_clk(phy);

	d->fallToOffDelay = DIV_ROUND_CLOSEST_ULL((u64)arm_clk * d_ns->fallToOffDelay, 1000000000);
	d->guardDelay = DIV_ROUND_CLOSEST_ULL((u64)arm_clk * d_ns->guardDelay, 1000000000);
	d->holdDelay = DIV_ROUND_CLOSEST_ULL((u64)arm_clk * d_ns->holdDelay, 1000000000);
	d->riseToAnalogOnDelay = DIV_ROUND_CLOSEST_ULL((u64)arm_clk * d_ns->riseToAnalogOnDelay,
						       1000000000);
	d->riseToOnDelay = DIV_ROUND_CLOSEST_ULL((u64)arm_clk * d_ns->riseToOnDelay, 1000000000);
}

void adrv9002_en_delays_arm_to_ns(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d,
				  struct adi_adrv9001_ChannelEnablementDelays *d_ns)
{
	u32 arm_clk = adrv9002_get_arm_clk(phy);

	d_ns->fallToOffDelay = DIV_ROUND_CLOSEST_ULL(d->fallToOffDelay * 1000000000ULL, arm_clk);
	d_ns->guardDelay = DIV_ROUND_CLOSEST_ULL(d->guardDelay * 1000000000ULL, arm_clk);
	d_ns->holdDelay = DIV_ROUND_CLOSEST_ULL(d->holdDelay * 1000000000ULL, arm_clk);
	d_ns->riseToAnalogOnDelay = DIV_ROUND_CLOSEST_ULL(d->riseToAnalogOnDelay * 1000000000ULL,
							  arm_clk);
	d_ns->riseToOnDelay = DIV_ROUND_CLOSEST_ULL(d->riseToOnDelay * 1000000000ULL, arm_clk);
}

#define ADRV9002_OF_U32_GET_VALIDATE(dev, node, key, def, min, max,		\
				     val, mandatory) ({				\
	u32 tmp;								\
	int ret, __ret = 0;							\
	const char *__key = key;						\
										\
	val = def;								\
	ret = of_property_read_u32(node, __key, &tmp);				\
	if (!ret) {								\
		if (tmp < (min) || tmp > (max)) {				\
			dev_err(dev, "Invalid value(%d) for \"%s\"\n",		\
				tmp, __key);					\
			__ret = -EINVAL;					\
		} else {							\
			val = tmp;						\
		}								\
	} else if (mandatory) {							\
		dev_err(dev, "Failed to get mandatory prop: \"%s\", ret=%d\n",	\
			__key, ret);						\
		__ret = ret;							\
	}									\
										\
	__ret;									\
})

#define OF_ADRV9002_PINCTL(key, def, min, max, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, pinctlr, key, def, min, \
				     max, val, mandatory)

#define OF_ADRV9002_DGPIO(node, key, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, key, ADI_ADRV9001_GPIO_UNASSIGNED, \
				     ADI_ADRV9001_GPIO_DIGITAL_00, ADI_ADRV9001_GPIO_DIGITAL_15, \
				     val, mandatory)

#define OF_ADRV9002_FH(key, def, min, max, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, fh, key, def, min, max, val, mandatory)

static int adrv9002_parse_hop_signal_ports(struct adrv9002_rf_phy *phy,
					   const struct device_node *node, int hop, bool tx)
{
	const char *prop = tx ? "adi,fh-hop-tx-ports" : "adi,fh-hop-rx-ports";
	const enum adi_adrv9001_FhHopSignal hop_signals[] = {
		ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FH_HOP_SIGNAL_2
	};
	int p, nports;

	nports = of_property_count_elems_of_size(node, prop, sizeof(u32));
	if (nports > ADRV9002_CHANN_MAX) {
		dev_err(&phy->spi->dev, "More than %d elements in %s\n", ADRV9002_CHANN_MAX, prop);
		return -EINVAL;
	}

	for (p = 0; p < nports; p++) {
		u32 port;

		of_property_read_u32_index(node, prop, p, &port);
		if (port > ADRV9002_CHANN_2) {
			dev_err(&phy->spi->dev, "Invalid port:%d given in %s\n", port, prop);
			return -EINVAL;
		}

		if (tx)
			phy->fh.txPortHopSignals[port] = hop_signals[hop];
		else
			phy->fh.rxPortHopSignals[port] = hop_signals[hop];
	}

	return 0;
}

static int adrv9002_parse_hop_signal(struct adrv9002_rf_phy *phy,
				     const struct device_node *node, int hop)
{
	struct device_node *child;
	adi_adrv9001_FhhopTableSelectCfg_t *hop_tbl = &phy->fh.hopTableSelectConfig;
	int ret;
	const char *hop_str = hop ? "adi,fh-hop-signal-2" : "adi,fh-hop-signal-1";

	child = of_get_child_by_name(node, hop_str);
	if (!child)
		return 0;

	/* check that hop2 is actually supported and valid */
	if (hop && phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
		dev_err(&phy->spi->dev, "adi,fh-hop-signal-2 given but adi,fh-mode not in dual hop");
		ret = -EINVAL;
		goto of_put;
	}

	ret = OF_ADRV9002_DGPIO(child, "adi,fh-hop-pin",
				phy->fh.hopSignalGpioConfig[hop].pin, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_DGPIO(child, "adi,fh-hop-table-select-pin",
				hop_tbl->hopTableSelectGpioConfig[hop].pin, false);
	if (ret)
		goto of_put;

	/*
	 * On commom mode, gpio[0] is used to control both hop tables select. Thus,
	 * error out already if we try to assign a gpio to hop_2
	 */
	if (hop_tbl->hopTableSelectMode == ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON && hop) {
		if (hop_tbl->hopTableSelectGpioConfig[hop].pin != ADI_ADRV9001_GPIO_UNASSIGNED) {
			dev_err(&phy->spi->dev,
				"Table select mode set to common. Cannot assign gpio for hop signal 2\n");
			ret = -EINVAL;
			goto of_put;
		}
	}

	ret = adrv9002_parse_hop_signal_ports(phy, child, hop, false);
	if (ret)
		goto of_put;

	ret = adrv9002_parse_hop_signal_ports(phy, child, hop, true);
of_put:
	of_node_put(child);
	return ret;
}

static int adrv9002_fh_parse_table_control(struct adrv9002_rf_phy *phy,
					   const struct device_node *fh)
{
	int ret;
	u32 pin, p;

	if (phy->fh.tableIndexCtrl != ADI_ADRV9001_TABLEINDEXCTRL_GPIO)
		return 0;

	ret = OF_ADRV9002_DGPIO(fh, "adi,fh-table-control-pin", pin, true);
	if (ret)
		return ret;

	ret = OF_ADRV9002_FH("adi,fh-table-control-npins", 0, 1,
			     ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS,
			     phy->fh.numTableIndexPins, true);
	if (ret)
		return ret;

	/* assign gpios now */
	for (p = 0; p < phy->fh.numTableIndexPins; p++) {
		if ((pin + p) > ADI_ADRV9001_GPIO_DIGITAL_15) {
			dev_err(&phy->spi->dev, "Invalid DGPIO given for tbl ctl: %d\n", pin + p);
			return -EINVAL;
		}
		phy->fh.tableIndexGpioConfig[p].pin = pin + p;
	}

	return 0;
}

#define assign_fh_gpio_ctl_table(node, prop, sz, tbl) {			\
	typeof(tbl) __tbl = (tbl);					\
	u32 p;								\
									\
	for (p = 0; p < (sz); p++) {					\
		u32 temp;						\
									\
		of_property_read_u32_index(node, (prop), p, &temp);	\
		__tbl[p] = temp;					\
	}								\
}

static int adrv9002_fh_parse_chan_gpio_gain_control_chan(struct adrv9002_rf_phy *phy,
							 const struct device_node *fh, int chan)
{
	int ret;
	adi_adrv9001_FhGainSetupByPinCfg_t *gpio_gain = &phy->fh.gainSetupByPinConfig[chan];
	u32 pin, p;
	int size;

	ret = OF_ADRV9002_DGPIO(fh, "adi,fh-gain-select-pin", pin, true);
	if (ret)
		return ret;

	ret = OF_ADRV9002_FH("adi,fh-gain-select-npins", 0, 1,
			     ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS,
			     gpio_gain->numGainCtrlPins, true);
	if (ret)
		return ret;

	/* assign gpios now */
	for (p = 0; p < gpio_gain->numGainCtrlPins; p++) {
		if ((pin + p) > ADI_ADRV9001_GPIO_DIGITAL_15) {
			dev_err(&phy->spi->dev, "Invalid DGPIO given for gain ctl: %d\n", pin + p);
			return -EINVAL;
		}
		gpio_gain->gainSelectGpioConfig[p].pin = pin + p;
	}

	/*
	 * The table size depends on the number of gpios used to control it. Hence, it cannot
	 * be bigger than 1 << gpio_gain->numGainCtrlPins
	 */
	size = of_property_count_elems_of_size(fh, "adi,fh-rx-gain-table", sizeof(u32));
	if (size <= 0 || size > (1 << gpio_gain->numGainCtrlPins)) {
		dev_err(&phy->spi->dev, "Invalid size:%d for fh rx gain table\n", size);
		return -EINVAL;
	}

	assign_fh_gpio_ctl_table(fh, "adi,fh-rx-gain-table", size, &gpio_gain->rxGainTable[0]);
	gpio_gain->numRxGainTableEntries = size;

	size = of_property_count_elems_of_size(fh, "adi,fh-tx-atten-table", sizeof(u32));
	if (size <= 0 || size > (1 << gpio_gain->numGainCtrlPins)) {
		dev_err(&phy->spi->dev, "Invalid size:%d for fh tx gain table\n", size);
		return -EINVAL;
	}

	assign_fh_gpio_ctl_table(fh, "adi,fh-tx-atten-table", size, &gpio_gain->txAttenTable[0]);
	gpio_gain->numTxAttenTableEntries = size;

	return 0;
}

static int adrv9002_fh_parse_gpio_gain_control(struct adrv9002_rf_phy *phy,
					       const struct device_node *node)
{
	struct device_node *gain, *child;
	int ret = 0, n_chann;

	gain = of_get_child_by_name(node, "adi,fh-gain-setup-by-pin");
	if (!gain)
		return 0;

	n_chann = of_get_available_child_count(gain);
	if (n_chann != ADRV9002_CHANN_MAX) {
		dev_err(&phy->spi->dev, "If set, Gain setup by pin must be set for both channels!\n");
		of_node_put(gain);
		return -EINVAL;
	}

	for_each_available_child_of_node(gain, child) {
		u32 chann;

		ret = of_property_read_u32(child, "reg", &chann);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No reg property defined for gain pin setup channel\n");
			goto of_child_put;
		}

		if (chann > ADRV9002_CHANN_2) {
			dev_err(&phy->spi->dev,
				"Invalid value for gain pin setup channel: %d\n", chann);
			ret = -EINVAL;
			goto of_child_put;
		}

		ret = adrv9002_fh_parse_chan_gpio_gain_control_chan(phy, child, chann);
		if (ret)
			goto of_child_put;
	}

	phy->fh.gainSetupByPin = true;

	of_node_put(gain);
	return 0;

of_child_put:
	of_node_put(child);
	of_node_put(gain);
	return ret;
}

static int adrv9002_parse_fh_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	adi_adrv9001_FhhopTableSelectCfg_t *hop_tbl = &phy->fh.hopTableSelectConfig;
	struct device_node *fh;
	int ret;
	u64 lo;
	int hop;

	fh = of_get_child_by_name(node, "adi,frequency-hopping");
	if (!fh) {
		/* set default parameters */
		phy->fh.minRxGainIndex = ADI_ADRV9001_RX_GAIN_INDEX_MIN;
		phy->fh.maxRxGainIndex = ADI_ADRV9001_RX_GAIN_INDEX_MAX;
		phy->fh.maxTxAtten_mdB = ADRV9001_TX_MAX_ATTENUATION_MDB;
		phy->fh.minOperatingFrequency_Hz = ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ;
		phy->fh.maxOperatingFrequency_Hz = ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ;
		return 0;
	}

	ret = OF_ADRV9002_FH("adi,fh-mode", ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS,
			     ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS,
			     ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP,
			     phy->fh.mode, false);
	if (ret)
		goto of_put;

	if (of_property_read_bool(fh, "adi,fh-hop-table-select-common-en"))
		hop_tbl->hopTableSelectMode = ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON;

	for (hop = 0; hop < ADRV9002_FH_HOP_SIGNALS_NR; hop++) {
		ret = adrv9002_parse_hop_signal(phy, fh, hop);
		if (ret)
			goto of_put;
	}

	ret = OF_ADRV9002_FH("adi,fh-min-rx-gain-idx", ADI_ADRV9001_RX_GAIN_INDEX_MIN,
			     ADI_ADRV9001_RX_GAIN_INDEX_MIN, ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.minRxGainIndex, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_FH("adi,fh-max-rx-gain-idx", ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.minRxGainIndex, ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.maxRxGainIndex, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_FH("adi,fh-min-tx-atten-mdb", 0, 0, ADRV9001_TX_MAX_ATTENUATION_MDB,
			     phy->fh.minTxAtten_mdB, false);
	if (ret)
		goto of_put;

	if (phy->fh.minTxAtten_mdB % ADRV9001_TX_ATTENUATION_RESOLUTION_MDB) {
		dev_err(&phy->spi->dev, "adi,fh-min-tx-atten-mdb must have %d resolution\n",
			ADRV9001_TX_ATTENUATION_RESOLUTION_MDB);
		ret = -EINVAL;
		goto of_put;
	}

	ret = OF_ADRV9002_FH("adi,fh-max-tx-atten-mdb", ADRV9001_TX_MAX_ATTENUATION_MDB,
			     phy->fh.minTxAtten_mdB, ADRV9001_TX_MAX_ATTENUATION_MDB,
			     phy->fh.maxTxAtten_mdB, false);
	if (ret)
		goto of_put;

	if (phy->fh.maxTxAtten_mdB % ADRV9001_TX_ATTENUATION_RESOLUTION_MDB) {
		dev_err(&phy->spi->dev, "adi,fh-max-tx-atten-mdb must have %d resolution\n",
			ADRV9001_TX_ATTENUATION_RESOLUTION_MDB);
		ret = -EINVAL;
		goto of_put;
	}

	ret = of_property_read_u64(fh, "adi,fh-min-frequency-hz", &lo);
	if (!ret) {
		if (lo < ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid val(%llu) for adi,fh-min-frequency-hz\n",
				lo);
			ret = -EINVAL;
			goto of_put;
		}
		phy->fh.minOperatingFrequency_Hz = lo;
	} else {
		phy->fh.minOperatingFrequency_Hz = ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ;
	}

	ret = of_property_read_u64(fh, "adi,fh-max-frequency-hz", &lo);
	if (!ret) {
		if (lo < phy->fh.minOperatingFrequency_Hz ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid val(%llu) for adi,fh-max-frequency-hz\n",
				lo);
			ret = -EINVAL;
			goto of_put;
		}
		phy->fh.maxOperatingFrequency_Hz = lo;
	} else {
		phy->fh.maxOperatingFrequency_Hz = ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ;
	}

	ret = OF_ADRV9002_FH("adi,fh-tx-analog-power-o-frame-delay", 0, 0,
			     ADI_ADRV9001_FH_MAX_TX_FE_POWERON_FRAME_DELAY,
			     phy->fh.txAnalogPowerOnFrameDelay, false);
	if (ret)
		goto of_put;

	phy->fh.rxZeroIfEnable = of_property_read_bool(fh, "adi-fh-rx-zero-if-en");
	of_property_read_u32(fh, "adi,fh-min-frame-us", &phy->fh.minFrameDuration_us);

	ret = OF_ADRV9002_FH("adi,fh-table-control", ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP,
			     ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP,
			     ADI_ADRV9001_TABLEINDEXCTRL_GPIO, phy->fh.tableIndexCtrl, false);
	if (ret)
		goto of_put;

	ret = adrv9002_fh_parse_table_control(phy, fh);
	if (ret)
		goto of_put;

	ret = adrv9002_fh_parse_gpio_gain_control(phy, fh);
of_put:
	of_node_put(fh);
	return ret;
}

/* as stated in adi_adrv9001_radio_types.h */
#define EN_DELAY_MAX		(BIT(24) - 1ULL)

static int adrv9002_parse_en_delays(const struct adrv9002_rf_phy *phy,
				    const struct device_node *node,
				    struct adrv9002_chan *chan)
{
	int ret;
	struct device_node *en_delay;
	struct adi_adrv9001_ChannelEnablementDelays *delays = &chan->en_delays_ns;
	u32 arm_clk = adrv9002_get_arm_clk(phy);
	u32 max_delay_ns = DIV_ROUND_CLOSEST_ULL(EN_DELAY_MAX * 1000000000, arm_clk);

	en_delay = of_parse_phandle(node, "adi,en-delays", 0);
	if (!en_delay)
		return 0;

#define OF_ADRV9002_EN_DELAY(key, min, max, val) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, en_delay, key, 0, min, \
				     max, val, false)

	ret = OF_ADRV9002_EN_DELAY("adi,rise-to-on-delay-ns", 0, max_delay_ns,
				   delays->riseToOnDelay);
	if (ret)
		goto of_en_delay_put;

	ret = OF_ADRV9002_EN_DELAY("adi,rise-to-analog-on-delay-ns", 0, delays->riseToOnDelay,
				   delays->riseToAnalogOnDelay);
	if (ret)
		goto of_en_delay_put;

	if (chan->port == ADI_RX) {
		ret = OF_ADRV9002_EN_DELAY("adi,fall-to-off-delay-ns", 0, max_delay_ns,
					   delays->fallToOffDelay);
		if (ret)
			goto of_en_delay_put;

		ret = OF_ADRV9002_EN_DELAY("adi,hold-delay-ns", 0, delays->fallToOffDelay,
					   delays->holdDelay);
		if (ret)
			goto of_en_delay_put;
	} else {
		ret = OF_ADRV9002_EN_DELAY("adi,hold-delay-ns", 0, max_delay_ns, delays->holdDelay);
		if (ret)
			goto of_en_delay_put;

		ret = OF_ADRV9002_EN_DELAY("adi,fall-to-off-delay-ns", 0, delays->holdDelay,
					   delays->fallToOffDelay);
		if (ret)
			goto of_en_delay_put;
	}

	ret = OF_ADRV9002_EN_DELAY("adi,guard-delay-ns", 0, max_delay_ns, delays->guardDelay);
	if (ret)
		goto of_en_delay_put;

of_en_delay_put:
	of_node_put(en_delay);

	return ret;
}

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

	ret = OF_ADRV9002_DGPIO(pinctlr, "adi,increment-pin", tx->pin_cfg->incrementPin, true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_DGPIO(pinctlr, "adi,decrement-pin", tx->pin_cfg->decrementPin, true);
	if (ret)
		goto of_pinctrl_put;

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
	int ret;

	if (of_property_read_bool(node, "adi,dac-full-scale-boost"))
		tx->dac_boost_en = true;

	ret = adrv9002_parse_en_delays(phy, node, &tx->channel);
	if (ret)
		return ret;

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
	ret = OF_ADRV9002_DGPIO(pinctlr, "adi,increment-pin", rx->pin_cfg->incrementPin, true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_DGPIO(pinctlr, "adi,decrement-pin", rx->pin_cfg->decrementPin, true);
	if (ret)
		goto of_pinctrl_put;

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

#define agc_assign_value(agc, prop, val) {			\
	typeof(prop) __prop = (prop);				\
	u32 __off = of_agc_props[__prop].__off;			\
								\
	switch (of_agc_props[__prop].size) {			\
	case 1:							\
		*(u8 *)((void *)(agc) + __off) = (val);		\
		break;						\
	case 2:							\
		*(u16 *)((void *)(agc) + __off) = (val);	\
		break;						\
	case 4:							\
		*(u32 *)((void *)(agc) + __off) = (val);	\
		break;						\
	};							\
}

static void adrv9002_set_agc_defaults(struct adi_adrv9001_GainControlCfg *agc)
{
	int prop;

	for (prop = 0; prop < ARRAY_SIZE(of_agc_props); prop++) {
		u32 temp = of_agc_props[prop].def;

		agc_assign_value(agc, prop, temp);
	}
	/*
	 * Since the enum is 0 (for now), we could just skipp this but I'm being paranoid and not
	 * trusting that this can't ever change...
	 */
	agc->power.feedback_apd_high_apd_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->power.feedback_inner_high_inner_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_low_hb_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_high_hb_high = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
}

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

	/* set boolean settings that are enabled by default */
	rx->agc.power.powerEnableMeasurement = true;
	rx->agc.peak.enableHbOverload = true;

	agc = of_parse_phandle(node, "adi,agc", 0);
	if (!agc) {
		adrv9002_set_agc_defaults(&rx->agc);
		return 0;
	}

	for (prop = 0; prop < ARRAY_SIZE(of_agc_props); prop++) {
		u32 temp;

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

		agc_assign_value(&rx->agc, prop, temp);
	}

	/* boolean properties */
	if (of_property_read_bool(agc, "adi,low-threshold-prevent-gain-inc"))
		rx->agc.lowThreshPreventGainInc = true;

	if (of_property_read_bool(agc, "adi,sync-pulse-gain-counter-en"))
		rx->agc.enableSyncPulseForGainCounter = true;

	if (of_property_read_bool(agc, "adi,fast-recovery-loop-en"))
		rx->agc.enableFastRecoveryLoop = true;

	if (of_property_read_bool(agc, "adi,reset-on-rx-on"))
		rx->agc.resetOnRxon = true;

	if (of_property_read_bool(agc, "adi,no-power-measurement-en"))
		rx->agc.power.powerEnableMeasurement = false;

	if (of_property_read_bool(agc, "adi,no-peak-hb-overload-en"))
		rx->agc.peak.enableHbOverload = false;

	/* check if there are any gpios */
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-high-thres-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.power.feedback_inner_high_inner_low);
	if (ret)
		goto out;

	/* feedback pins*/
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-low-thres-gain-change",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.power.feedback_apd_high_apd_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-high-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.peak.feedback_apd_low_hb_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-low-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.peak.feedback_apd_high_hb_high);
out:
	of_node_put(agc);
	return ret;
}

static int adrv9002_parse_rx_dt(struct adrv9002_rf_phy *phy,
				struct device_node *node,
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

	ret = adrv9002_parse_en_delays(phy, node, &rx->channel);
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

	rx->agc.maxGainIndex = max_gain;
	rx->agc.minGainIndex = min_gain;

	if (rx->pin_cfg) {
		rx->pin_cfg->maxGainIndex = max_gain;
		rx->pin_cfg->minGainIndex = min_gain;
	}

	/* there's no optional variant for this, so we need to check for -ENOENT */
	rx->orx_gpio = devm_fwnode_get_index_gpiod_from_child(&phy->spi->dev, "orx", 0,
							      of_fwnode_handle(node),
							      GPIOD_OUT_LOW, NULL);
	if (IS_ERR(rx->orx_gpio)) {
		if (PTR_ERR(rx->orx_gpio) != -ENOENT)
			return PTR_ERR(rx->orx_gpio);

		rx->orx_gpio = NULL;
	}

	return 0;
}

static int adrv9002_parse_gpios_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	struct device_node *of_gpios, *child;
	struct device *dev = &phy->spi->dev;
	int ret = 0, idx = 0;

	of_gpios = of_get_child_by_name(node, "adi,gpios");
	if (!of_gpios)
		return 0;

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
			dev_err(dev, "No reg property defined for gpio\n");
			goto of_child_put;
		}

		if (gpio < ADI_ADRV9001_GPIO_DIGITAL_00 ||
		    gpio > ADI_ADRV9001_GPIO_ANALOG_11) {
			dev_err(dev, "Invalid gpio number: %d\n", gpio);
			ret = -EINVAL;
			goto of_child_put;
		}

		phy->adrv9002_gpios[idx].gpio.pin = gpio;

		ret = of_property_read_u32(child, "adi,signal", &signal);
		if (ret) {
			dev_err(dev, "No adi,signal property defined for gpio%d\n", gpio);
			goto of_child_put;
		}

		if (signal >= ADI_ADRV9001_GPIO_NUM_SIGNALS) {
			dev_err(dev, "Invalid gpio signal: %d\n", signal);
			ret = -EINVAL;
			goto of_child_put;
		}

		phy->adrv9002_gpios[idx].signal = signal;

		ret = of_property_read_u32(child, "adi,polarity", &polarity);
		if (!ret) {
			if (polarity > ADI_ADRV9001_GPIO_POLARITY_INVERTED) {
				dev_err(dev, "Invalid gpio polarity: %d\n", polarity);
				ret = -EINVAL;
				goto of_child_put;
			}

			phy->adrv9002_gpios[idx].gpio.polarity = polarity;
		}

		ret = of_property_read_u32(child, "adi,master", &master);
		if (!ret) {
			if (master != ADI_ADRV9001_GPIO_MASTER_ADRV9001 &&
			    master != ADI_ADRV9001_GPIO_MASTER_BBIC) {
				dev_err(dev, "Invalid gpio master: %d\n", master);
				ret = -EINVAL;
				goto of_child_put;
			}

			phy->adrv9002_gpios[idx].gpio.master = master;
		}

		idx++;
	}

	of_node_put(of_gpios);
	return 0;

of_child_put:
	of_node_put(child);
of_gpio_put:
	of_node_put(of_gpios);
	return ret;
}

static int adrv9002_parse_channels_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	struct device_node *of_channels, *child;
	struct device *dev = &phy->spi->dev;
	int ret;

	of_channels = of_get_child_by_name(node, "adi,channels");
	if (!of_channels)
		return 0;

	for_each_available_child_of_node(of_channels, child) {
		u32 chann, port;

		ret = of_property_read_u32(child, "reg", &chann);
		if (ret) {
			dev_err(dev, "No reg property defined for channel\n");
			goto of_error_put;
		}

		if (chann >= ADRV9002_CHANN_MAX) {
			dev_err(dev, "Invalid value for channel: %d\n", chann);
			ret = -EINVAL;
			goto of_error_put;
		}

		ret = of_property_read_u32(child, "adi,port", &port);
		if (ret) {
			dev_err(dev, "No port property defined for channel\n");
			goto of_error_put;
		}

		switch (port) {
		case ADI_TX:
			ret = adrv9002_parse_tx_dt(phy, child, chann);
			break;
		case ADI_RX:
			ret = adrv9002_parse_rx_dt(phy, child, chann);
			break;
		default:
			dev_err(dev, "Unknown port: %d\n", port);
			ret = -EINVAL;
			break;
		};

		if (ret)
			goto of_error_put;
	}

	of_node_put(of_channels);
	return 0;

of_error_put:
	of_node_put(child);
	of_node_put(of_channels);
	return ret;
}

static int adrv9002_parse_dt(struct adrv9002_rf_phy *phy)
{
	struct device_node *parent = phy->spi->dev.of_node;
	int ret;

	ret = adrv9002_parse_channels_dt(phy, parent);
	if (ret)
		return ret;

	ret = adrv9002_parse_gpios_dt(phy, parent);
	if (ret)
		return ret;

	return adrv9002_parse_fh_dt(phy, parent);
}

int adrv9002_init(struct adrv9002_rf_phy *phy, struct adi_adrv9001_Init *profile)
{
	int ret, c;
	struct adrv9002_chan *chan;

	adrv9002_cleanup(phy);
	/*
	 * Disable all the cores as it might interfere with init calibrations.
	 */
	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		chan = phy->channels[c];

		if (phy->rx2tx2 && chan->idx > ADRV9002_CHANN_1)
			break;
		adrv9002_axi_interface_enable(phy, chan->idx, chan->port == ADI_TX, false);
	}

	phy->curr_profile = profile;
	ret = adrv9002_setup(phy);
	if (ret) {
		/* try one more time */
		ret = adrv9002_setup(phy);
		if (ret)
			goto error;
	}

	adrv9002_set_clk_rates(phy);

	ret = adrv9002_ssi_configure(phy);
	if (ret)
		goto error;

	/* re-enable the cores */
	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		chan = phy->channels[c];

		if (phy->rx2tx2 && chan->idx > ADRV9002_CHANN_1)
			break;

		if (!chan->enabled)
			continue;

		adrv9002_axi_interface_enable(phy, chan->idx, chan->port == ADI_TX, true);
	}

	ret = adrv9002_intf_tuning(phy);
	if (ret) {
		dev_err(&phy->spi->dev, "Interface tuning failed: %d\n", ret);
		goto error;
	}

	return 0;
error:
	/*
	 * Leave the device in a reset state in case of error. There's not much we can do if
	 * the API call fails, so we are just being verbose about it...
	 */
	if (adi_adrv9001_HwOpen(phy->adrv9001, &adrv9002_spi))
		adrv9002_dev_err(phy);
	adrv9002_cleanup(phy);

	return ret;
}

static ssize_t adrv9002_stream_bin_write(struct file *filp, struct kobject *kobj,
					 struct bin_attribute *bin_attr, char *buf, loff_t off,
					 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);

	if (off + count > bin_attr->size) {
		dev_err(&phy->spi->dev, "Invalid stream image size:%lld!\n", count + off);
		return -EINVAL;
	}

	mutex_lock(&phy->lock);
	if (!off)
		phy->stream_size = 0;
	memcpy(phy->stream_buf + off, buf, count);
	phy->stream_size += count;
	mutex_unlock(&phy->lock);

	return count;
}

static ssize_t adrv9002_profile_bin_write(struct file *filp, struct kobject *kobj,
					  struct bin_attribute *bin_attr, char *buf, loff_t off,
					  size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	if (off + count > phy->bin_attr_sz)
		return -EFBIG;

	if (off == 0)
		memset(phy->bin_attr_buf, 0, phy->bin_attr_sz);

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

const char *const lo_maps[] = {
	"Unknown",
	"L01",
	"L02",
	"AUX LO"
};

const char *const duplex[] = {
	"TDD",
	"FDD"
};

const char *const ssi[] = {
	"Disabled",
	"CMOS",
	"LVDS"
};

const char *const mcs[] = {
	"Disabled",
	"Enabled",
	"Enabled RFPLL Phase"
};

const char *const rx_gain_type[] = {
	"Correction",
	"Compensated"
};

static ssize_t adrv9002_profile_bin_read(struct file *filp, struct kobject *kobj,
					 struct bin_attribute *bin_attr, char *buf, loff_t pos,
					 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct adi_adrv9001_ClockSettings *clks = &phy->curr_profile->clocks;
	struct adi_adrv9001_RxSettings *rx = &phy->curr_profile->rx;
	struct adi_adrv9001_TxSettings *tx = &phy->curr_profile->tx;
	struct adi_adrv9001_DeviceSysConfig *sys = &phy->curr_profile->sysConfig;
	char info[350];
	size_t len;

	len = scnprintf(info, sizeof(info), "Device clk(Hz): %d\n"
		       "Clk PLL VCO(Hz): %lld\n"
		       "ARM Power Saving Clk Divider: %d\n"
		       "RX1 LO: %s\n"
		       "RX2 LO: %s\n"
		       "TX1 LO: %s\n"
		       "TX2 LO: %s\n"
		       "RX1 Gain Table Type: %s\n"
		       "RX2 Gain Table Type: %s\n"
		       "RX Channel Mask: 0x%x\n"
		       "TX Channel Mask: 0x%x\n"
		       "Duplex Mode: %s\n"
		       "FH enable: %d\n"
		       "MCS mode: %s\n"
		       "SSI interface: %s\n", clks->deviceClock_kHz * 1000,
		       clks->clkPllVcoFreq_daHz * 10ULL, clks->armPowerSavingClkDiv,
		       lo_maps[clks->rx1LoSelect], lo_maps[clks->rx2LoSelect],
		       lo_maps[clks->tx1LoSelect], lo_maps[clks->tx2LoSelect],
		       rx_gain_type[rx->rxChannelCfg[ADRV9002_CHANN_1].profile.gainTableType],
		       rx_gain_type[rx->rxChannelCfg[ADRV9002_CHANN_2].profile.gainTableType],
		       rx->rxInitChannelMask, tx->txInitChannelMask, duplex[sys->duplexMode],
		       sys->fhModeOn, mcs[sys->mcsMode], ssi[phy->ssi_type]);

	return memory_read_from_buffer(buf, count, &pos, info, len);
}

static ssize_t adrv9002_fh_bin_table_write(struct adrv9002_rf_phy *phy, char *buf, loff_t off,
					   size_t count, int hop, int table)
{
	struct adrv9002_fh_bin_table *tbl = &phy->fh_table_bin_attr[hop * 2 + table];
	static adi_adrv9001_FhHopFrame_t hop_tbl[ADI_ADRV9001_FH_MAX_HOP_TABLE_SIZE];
	char *p, *line;
	int entry = 0, ret, max_sz = ARRAY_SIZE(hop_tbl);

	mutex_lock(&phy->lock);
	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
		mutex_unlock(&phy->lock);
		return -ENOTSUPP;
	}

	if (hop && phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
		dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
		mutex_unlock(&phy->lock);
		return -ENOTSUPP;
	}

	if (!off)
		memset(tbl->bin_table, 0, sizeof(tbl->bin_table));

	memcpy(tbl->bin_table + off, buf, count);

	if (!strnstr(tbl->bin_table, "</table>", off + count)) {
		mutex_unlock(&phy->lock);
		return count;
	}

	if (phy->fh.mode == ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP)
		max_sz /= 2;

	p = tbl->bin_table;
	while ((line = strsep(&p, "\n")) && p) {
		u64 lo;
		u32 rx10_if, rx20_if, rx1_gain, tx1_atten, rx2_gain, tx2_atten;

		 /* skip comment lines or blank lines */
		if (line[0] == '#' || !line[0])
			continue;
		if (strstr(line, "table>"))
			continue;

		ret = sscanf(line, "%llu,%u,%u,%u,%u,%u,%u", &lo, &rx10_if, &rx20_if, &rx1_gain,
			     &tx1_atten, &rx2_gain, &tx2_atten);
		if (ret != ADRV9002_FH_TABLE_COL_SZ) {
			dev_err(&phy->spi->dev, "Failed to parse hop:%d table:%d line:%s\n",
				hop, table, line);
			mutex_unlock(&phy->lock);
			return -EINVAL;
		}

		if (entry > max_sz) {
			dev_err(&phy->spi->dev, "Hop:%d table:%d too big:%d\n", hop, table, entry);
			mutex_unlock(&phy->lock);
			return -EINVAL;
		}

		if (lo < ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid value for lo:%llu, in table entry:%d\n",
				lo, entry);
			mutex_unlock(&phy->lock);
			return -EINVAL;
		}

		hop_tbl[entry].hopFrequencyHz = lo;
		hop_tbl[entry].rx1OffsetFrequencyHz = rx10_if;
		hop_tbl[entry].rx2OffsetFrequencyHz = rx10_if;
		hop_tbl[entry].rx1GainIndex = rx1_gain;
		hop_tbl[entry].tx1Attenuation_fifthdB = tx1_atten;
		hop_tbl[entry].rx2GainIndex = rx1_gain;
		hop_tbl[entry].tx2Attenuation_fifthdB = tx2_atten;
		entry++;
	}

	dev_dbg(&phy->spi->dev, "Load hop:%d table:%d with %d entries\n", hop, table, entry);
	ret = adi_adrv9001_fh_HopTable_Static_Configure(phy->adrv9001, phy->fh.mode,
							hop, table, hop_tbl, entry);
	if (ret)
		ret = adrv9002_dev_err(phy);
	mutex_unlock(&phy->lock);

	return ret ? ret : count;
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

#define ADRV9002_HOP_TABLE_BIN_ATTR(h, t, hop, table) \
static ssize_t adrv9002_fh_hop##h##_bin_table_##t##_write(struct file *filp,			\
							  struct kobject *kobj,			\
							  struct bin_attribute *bin_attr,	\
							  char *buf, loff_t off, size_t count)	\
{												\
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));				\
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);					\
												\
	return adrv9002_fh_bin_table_write(phy, buf, off, count, hop, table);			\
}												\
static BIN_ATTR(frequency_hopping_hop##h##_table_##t, 0222, NULL,				\
		adrv9002_fh_hop##h##_bin_table_##t##_write, PAGE_SIZE)				\

ADRV9002_HOP_TABLE_BIN_ATTR(1, a, ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FHHOPTABLE_A);
ADRV9002_HOP_TABLE_BIN_ATTR(1, b, ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FHHOPTABLE_B);
ADRV9002_HOP_TABLE_BIN_ATTR(2, a, ADI_ADRV9001_FH_HOP_SIGNAL_2, ADI_ADRV9001_FHHOPTABLE_A);
ADRV9002_HOP_TABLE_BIN_ATTR(2, b, ADI_ADRV9001_FH_HOP_SIGNAL_2, ADI_ADRV9001_FHHOPTABLE_B);
static BIN_ATTR(stream_config, 0222, NULL, adrv9002_stream_bin_write, ADRV9002_STREAM_BINARY_SZ);
static BIN_ATTR(profile_config, 0644, adrv9002_profile_bin_read, adrv9002_profile_bin_write, 0);

int adrv9002_post_init(struct adrv9002_rf_phy *phy)
{
	struct adi_common_ApiVersion api_version;
	struct adi_adrv9001_ArmVersion arm_version;
	struct adi_adrv9001_SiliconVersion silicon_version;
	struct adi_adrv9001_StreamVersion stream_version;
	int ret, c;
	struct spi_device *spi = phy->spi;
	struct iio_dev *indio_dev = phy->indio_dev;
	const char * const clk_names[NUM_ADRV9002_CLKS] = {
		[RX1_SAMPL_CLK] = "-rx1_sampl_clk",
		[RX2_SAMPL_CLK] = "-rx2_sampl_clk",
		[TX1_SAMPL_CLK] = "-tx1_sampl_clk",
		[TX2_SAMPL_CLK] = "-tx2_sampl_clk",
		[TDD1_INTF_CLK] = "-tdd1_intf_clk",
		[TDD2_INTF_CLK] = "-tdd2_intf_clk"
	};
	const struct bin_attribute *hop_attrs[] = {
		&bin_attr_frequency_hopping_hop1_table_a,
		&bin_attr_frequency_hopping_hop1_table_b,
		&bin_attr_frequency_hopping_hop2_table_a,
		&bin_attr_frequency_hopping_hop2_table_b
	};

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

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS) {
		ret = adrv9002_profile_load(phy, "Navassa_LVDS_profile.json");
		if (ret)
			return ret;
	}

	ret = adrv9002_init(phy, &phy->profile);
	if (ret)
		return ret;

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

	phy->bin_attr_sz = 73728;
	phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev, phy->bin_attr_sz, GFP_KERNEL);
	if (!phy->bin_attr_buf)
		return -ENOMEM;

	ret = device_create_bin_file(&indio_dev->dev, &bin_attr_profile_config);
	if (ret < 0)
		return ret;

	phy->stream_buf = devm_kzalloc(&phy->spi->dev, ADRV9002_STREAM_BINARY_SZ, GFP_KERNEL);
	if (!phy->stream_buf)
		return -ENOMEM;

	ret = device_create_bin_file(&indio_dev->dev, &bin_attr_stream_config);
	if (ret < 0)
		return ret;

	for (c = 0; c < ARRAY_SIZE(hop_attrs); c++) {
		ret = device_create_bin_file(&indio_dev->dev, hop_attrs[c]);
		if (ret < 0)
			return ret;
	}

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

	/* set current profile now as it's needed in @adrv9002_parse_dt() */
	phy->curr_profile = &phy->profile;
	phy->ssi_type = ADI_ADRV9001_SSI_TYPE_CMOS;

	/* initialize channel numbers and ports here since these will never change */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		phy->rx_channels[c].channel.idx = c;
		phy->rx_channels[c].channel.number = c + ADI_CHANNEL_1;
		phy->rx_channels[c].channel.port = ADI_RX;
		phy->channels[c * 2] = &phy->rx_channels[c].channel;
		phy->tx_channels[c].channel.idx = c;
		phy->tx_channels[c].channel.number = c + ADI_CHANNEL_1;
		phy->tx_channels[c].channel.port = ADI_TX;
		phy->channels[c * 2 + 1] = &phy->tx_channels[c].channel;
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
