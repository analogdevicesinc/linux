// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/cleanup.h>
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
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/units.h>

#include "adrv9002.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_auxadc.h"
#include "adi_adrv9001_auxadc_types.h"
#include "adi_adrv9001_bbdc.h"
#include "adi_adrv9001_cals.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_dpd.h"
#include "adi_adrv9001_dpd_types.h"
#include "adi_common_types.h"
#include "adi_adrv9001_auxdac.h"
#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_fh.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_mcs.h"
#include "adi_adrv9001_orx.h"
#include "adi_adrv9001_powermanagement.h"
#include "adi_adrv9001_powermanagement_types.h"
#include "adi_adrv9001_profile_types.h"
#include "adi_adrv9001_profileutil.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
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
#include "adi_common_error_types.h"

#define ALL_RX_CHANNEL_MASK	(ADI_ADRV9001_RX1 | ADI_ADRV9001_RX2 | \
				 ADI_ADRV9001_ORX1 | ADI_ADRV9001_ORX2)

#define ADRV9002_PORT_BIT(c)	(((c)->idx * 2 + (c)->port) & 0x3)
#define ADRV9002_PORT_MASK(c)	BIT(ADRV9002_PORT_BIT(c))

#define ADRV9002_RX_MAX_GAIN_mdB	\
	((ADI_ADRV9001_RX_GAIN_INDEX_MAX - ADI_ADRV9001_RX_GAIN_INDEX_MIN) *	\
	 ADRV9002_RX_GAIN_STEP_mDB)
#define ADRV9002_RX_GAIN_STEP_mDB	500
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
#define ADRV9002_PROFILE_MAX_SZ		73728
#define ADRV9002_HP_CLK_PLL_DAHZ	884736000
#define ADRV9002_NO_EXT_LO		0xff
#define ADRV9002_EXT_LO_FREQ_MIN	60000000
#define ADRV9002_EXT_LO_FREQ_MAX	12000000000ULL
#define ADRV9002_DEV_CLKOUT_MIN		(10 * MEGA)
#define ADRV9002_DEV_CLKOUT_MAX		(80 * MEGA)

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

#define ADRV9002_RX_BIT_START		(ffs(ADI_ADRV9001_RX1) - 1)
#define ADRV9002_TX_BIT_START		(ffs(ADI_ADRV9001_TX1) - 1)
#define ADRV9002_ORX_BIT_START		(ffs(ADI_ADRV9001_ORX1) - 1)
#define ADRV9002_ELB_BIT_START		(ffs(ADI_ADRV9001_ELB1) - 1)

enum {
	ADRV9002_TX_A,
	ADRV9002_TX_B,
};

int __adrv9002_dev_err(const struct adrv9002_rf_phy *phy, const char *function, const int line)
{
	int ret;

	dev_err(&phy->spi->dev, "%s, %d: failed with \"%s\" (%d)\n", function, line,
		phy->adrv9001->common.error.errormessage[0] != '\0' ?
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

static void adrv9002_get_ssi_interface(const struct adrv9002_rf_phy *phy, const int chann,
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

static int adrv9002_ssi_configure(const struct adrv9002_rf_phy *phy)
{
	bool cmos_ddr;
	u8 n_lanes;
	int c, ret;

	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		const struct adrv9002_chan *chann = phy->channels[c];

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
			const struct adrv9002_rx_chan *rx = &phy->rx_channels[chann->idx];
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
	u8 val;

	guard(mutex)(&phy->lock);
	if (!readval)
		return api_call(phy, adi_adrv9001_spi_Byte_Write, reg, writeval);

	ret = api_call(phy, adi_adrv9001_spi_Byte_Read, reg, &val);
	if (ret)
		return ret;

	*readval = val;

	return 0;
}

#define ADRV9002_MAX_CLK_NAME 79

static char *adrv9002_clk_set_dev_name(const struct adrv9002_rf_phy *phy,
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

static int adrv9002_chan_to_state_poll(const struct adrv9002_rf_phy *phy,
				       const struct adrv9002_chan *c,
				       const adi_adrv9001_ChannelState_e state,
				       const int n_tries)
{
	int ret, tmp;
	adi_adrv9001_ChannelState_e __state;

	tmp = read_poll_timeout(adi_adrv9001_Radio_Channel_State_Get, ret,
				ret || (__state == state), 1000, n_tries * 1000, false,
				phy->adrv9001, c->port, c->number, &__state);

	/* so that we keep the same behavior as before introducing read_poll_timeout() */
	if (tmp == -ETIMEDOUT)
		return -EBUSY;

	return ret ? __adrv9002_dev_err(phy, __func__, __LINE__) : 0;
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

int adrv9002_channel_to_state(const struct adrv9002_rf_phy *phy, struct adrv9002_chan *chann,
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

	ret = api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Get, chann->port,
		       chann->number, &mode);
	if (ret)
		return ret;

	/* we need to set it to spi */
	if (mode == ADI_ADRV9001_PIN_MODE) {
		ret = api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Set, chann->port,
			       chann->number, ADI_ADRV9001_SPI_MODE);
		if (ret)
			return ret;
	}

	if (cache_state) {
		ret = api_call(phy, adi_adrv9001_Radio_Channel_State_Get, chann->port,
			       chann->number, &chann->cached_state);
		if (ret)
			return ret;
	}

	ret = api_call(phy, adi_adrv9001_Radio_Channel_ToState, chann->port, chann->number, state);
	if (ret)
		return ret;
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
		ret = api_call(phy, adi_adrv9001_Radio_Channel_ToState, chann->port,
			       chann->number, state);
		if (ret)
			return ret;

		ret = adrv9002_chan_to_state_poll(phy, chann, state, 14);
		if (ret)
			return ret;
	}

	if (mode == ADI_ADRV9001_SPI_MODE)
		return 0;

	/* restore enable mode */
	return api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Set,
			chann->port, chann->number, mode);
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
	ADRV9002_HOP_2_TRIGGER,
	ADRV9002_INIT_CALS_RUN,
	ADRV9002_WARMBOOT_SEL,
	ADRV9002_MCS,
};

static const char * const adrv9002_hop_table[ADRV9002_FH_TABLES_NR + 1] = {
	"TABLE_A",
	"TABLE_B",
	"Unknown"
};

static int adrv9002_fh_table_show(struct adrv9002_rf_phy *phy, char *buf, u64 address)
{
	adi_adrv9001_FhMode_e mode = phy->fh.mode;
	adi_adrv9001_FhHopTable_e table;
	int ret;

	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
		return -ENOTSUPP;
	}

	if (address && mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
		dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
		return -ENOTSUPP;
	}

	ret = api_call(phy, adi_adrv9001_fh_HopTable_Get, address, &table);
	if (ret)
		return ret;

	if (table >= ADRV9002_FH_TABLES_NR)
		table = ADRV9002_FH_TABLES_NR;

	return sysfs_emit(buf, "%s\n", adrv9002_hop_table[table]);
}

static const char * const adrv9002_init_cals_modes[] = {
	"off",
	"auto",
	"run"
};

static ssize_t adrv9002_attr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);

	guard(mutex)(&phy->lock);

	switch (iio_attr->address) {
	case ADRV9002_HOP_1_TABLE_SEL:
	case ADRV9002_HOP_2_TABLE_SEL:
		return adrv9002_fh_table_show(phy, buf, iio_attr->address);
	case ADRV9002_INIT_CALS_RUN:
		return sysfs_emit(buf, "%s\n", adrv9002_init_cals_modes[phy->run_cals]);
	case ADRV9002_WARMBOOT_SEL:
		return sysfs_emit(buf, "%s\n", phy->warm_boot.coeffs_name);
	default:
		return -EINVAL;
	}
}

static void adrv9002_port_enable(const struct adrv9002_rf_phy *phy,
				 const struct adrv9002_chan *c, bool enable)
{
	/*
	 * Handle port muxes. Always terminate the ports at 50ohm when disabling and only
	 * mux them again if the channel is enabled.
	 */
	if (!enable) {
		if (c->mux_ctl)
			gpiod_set_value_cansleep(c->mux_ctl, 0);
		if (c->mux_ctl_2)
			gpiod_set_value_cansleep(c->mux_ctl_2, 0);
	} else if (c->enabled) {
		bool ctl_assert = true;

		/* Make sure to respect any possible TX port selection given by userspace. */
		if (c->port == ADI_TX && chan_to_tx(c)->port_sel == ADRV9002_TX_B)
			ctl_assert = false;

		if (c->mux_ctl && ctl_assert)
			gpiod_set_value_cansleep(c->mux_ctl, 1);
		if (c->mux_ctl_2)
			gpiod_set_value_cansleep(c->mux_ctl_2, 1);
	}

	/*
	 * Nothing to do for channel 2 in rx2tx2 mode. The check is useful to have
	 * it in here if the outer loop is looping through all the channels.
	 */
	if (phy->rx2tx2 && c->idx > ADRV9002_CHANN_1)
		return;
	/*
	 * We always disable but let's not enable a port that is not enable in the profile.
	 * Same as above, the condition is needed if the outer loop is looping through all
	 * channels to enable/disable them. Might be a redundant in some cases where the
	 * caller already knows the state of the port.
	 */
	if (enable && !c->enabled)
		return;

	adrv9002_axi_interface_enable(phy, c->idx, c->port == ADI_TX, enable);
}

static int adrv9002_phy_rerun_cals_setup(struct adrv9002_rf_phy *phy, unsigned long port_mask,
					 bool after)
{
	int ret, c;

	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		adi_adrv9001_ChannelState_e state = after ? phy->channels[c]->cached_state
				: ADI_ADRV9001_CHANNEL_CALIBRATED;

		adrv9002_port_enable(phy, phy->channels[c], after);

		/* if the bit is set, then we already moved the port */
		if (test_bit(ADRV9002_PORT_BIT(phy->channels[c]), &port_mask))
			continue;

		ret = adrv9002_channel_to_state(phy, phy->channels[c], state, !after);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9002_phy_rerun_cals(struct adrv9002_rf_phy *phy,
				   struct adi_adrv9001_InitCals *init_cals, unsigned long port_mask)
{
	u8 error;
	int ret;

	if (!init_cals->chanInitCalMask[0] && !init_cals->chanInitCalMask[1])
		return 0;

	/* move the missing ports to calibrated state and disable all the axi cores */
	ret = adrv9002_phy_rerun_cals_setup(phy, port_mask, false);
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "Re-run init cals: mask: %08X, %08X\n",
		init_cals->chanInitCalMask[0], init_cals->chanInitCalMask[1]);

	ret = api_call(phy, adi_adrv9001_cals_InitCals_Run, init_cals, 60000, &error);
	if (ret)
		return ret;

	/* re-enable cores and re-set port states (those where the carrier is not being changed) */
	return adrv9002_phy_rerun_cals_setup(phy, port_mask, true);
}

static int adrv9002_init_cals_set(struct adrv9002_rf_phy *phy, const char *buf)
{
	struct adi_adrv9001_InitCals cals = {0};
	int c, ret;

	ret = sysfs_match_string(adrv9002_init_cals_modes, buf);
	if (ret < 0)
		return ret;

	if (strcmp(adrv9002_init_cals_modes[ret], "run")) {
		if (!strcmp(adrv9002_init_cals_modes[ret], "auto"))
			phy->run_cals = true;
		else
			phy->run_cals = false;

		return 0;
	}

	for (c = 0; c < ARRAY_SIZE(phy->channels); c++)
		cals.chanInitCalMask[phy->channels[c]->idx] |= phy->channels[c]->lo_cals;

	return adrv9002_phy_rerun_cals(phy, &cals, 0);
}

static int adrv9002_warm_boot_name_save(struct adrv9002_rf_phy *phy, const char *buf)
{
	int ret;
	char *p;

	ret = strscpy(phy->warm_boot.coeffs_name, buf, sizeof(phy->warm_boot.coeffs_name));
	if (ret < 0)
		return ret;

	/* Strip trailing newline */
	p = phy->warm_boot.coeffs_name + strlen(phy->warm_boot.coeffs_name);
	if (*--p == '\n')
		*p = '\0';

	return 0;
}

static int adrv9002_fh_set(const struct adrv9002_rf_phy *phy, const char *buf, u64 address)
{
	int ret;

	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
		return -ENOTSUPP;
	}

	if (address == ADRV9002_HOP_2_TABLE_SEL || address == ADRV9002_HOP_2_TRIGGER) {
		if (phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
			dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
			return -ENOTSUPP;
		}
	}

	switch (address) {
	case ADRV9002_HOP_1_TABLE_SEL:
	case ADRV9002_HOP_2_TABLE_SEL:
		ret = __sysfs_match_string(adrv9002_hop_table,
					   ARRAY_SIZE(adrv9002_hop_table) - 1, buf);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "Unknown table %s\n", buf);
			return ret;
		}

		return api_call(phy, adi_adrv9001_fh_HopTable_Set, address, ret);
	case ADRV9002_HOP_1_TRIGGER:
		return api_call(phy, adi_adrv9001_fh_Hop, ADI_ADRV9001_FH_HOP_SIGNAL_1);
	case ADRV9002_HOP_2_TRIGGER:
		return api_call(phy, adi_adrv9001_fh_Hop, ADI_ADRV9001_FH_HOP_SIGNAL_2);
	default:
		return  -EINVAL;
	}
}

static int adrv9002_mcs_run(struct adrv9002_rf_phy *phy, const char *buf)
{
	adi_adrv9001_RadioState_t radio = {0};
	unsigned int i;
	int ret, tmp;

	if (!phy->curr_profile->sysConfig.mcsMode) {
		dev_err(&phy->spi->dev, "Multi chip sync not enabled\n");
		return -ENOTSUPP;
	}

	if (phy->mcs_run) {
		/*
		 * !\FIXME: Ugly hack but MCS only runs successful once and then always fails
		 * (get's stuck). Hence let's just not allow running more than once and print
		 * something so people can see this is a known thing.
		 */
		dev_err(&phy->spi->dev, "Multi chip sync can only run once for now...\n");
		return -EPERM;
	}

	/* all channels need to be in calibrated state...*/
	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		ret = adrv9002_channel_to_state(phy, c, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
		if (ret)
			return ret;
	}

	ret = api_call(phy, adi_adrv9001_Radio_ToMcsReady);
	if (ret)
		return ret;

	tmp = read_poll_timeout(adi_adrv9001_Radio_State_Get, ret,
				ret || (radio.mcsState == ADI_ADRV9001_ARMMCSSTATES_DONE),
				20 * USEC_PER_MSEC, 10 * USEC_PER_SEC, false, phy->adrv9001,
				&radio);
	if (ret)
		return __adrv9002_dev_err(phy, __func__, __LINE__);
	if (tmp)
		return tmp;

	phy->mcs_run = true;

	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		struct adrv9002_chan *c = phy->channels[i];

		ret = adrv9002_channel_to_state(phy, c, c->cached_state, false);
		if (ret)
			return ret;
	}

	return 0;
}

static ssize_t adrv9002_attr_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct iio_dev_attr *iio_attr = to_iio_dev_attr(attr);
	int ret;

	guard(mutex)(&phy->lock);
	switch (iio_attr->address) {
	case ADRV9002_INIT_CALS_RUN:
		ret = adrv9002_init_cals_set(phy, buf);
		break;
	case ADRV9002_WARMBOOT_SEL:
		ret = adrv9002_warm_boot_name_save(phy, buf);
		break;
	case ADRV9002_MCS:
		ret = adrv9002_mcs_run(phy, buf);
		break;
	default:
		ret = adrv9002_fh_set(phy, buf, iio_attr->address);
	}

	return ret ? ret : len;
}

static int adrv9002_set_ext_lo(const struct adrv9002_chan *c, u64 freq)
{
	u64 lo_freq;

	if (!c->ext_lo)
		return 0;

	lo_freq = freq * c->ext_lo->divider;
	if (lo_freq < ADRV9002_EXT_LO_FREQ_MIN || lo_freq > ADRV9002_EXT_LO_FREQ_MAX) {
		const struct adrv9002_rf_phy *phy = chan_to_phy(c);

		dev_err(&phy->spi->dev, "Ext LO freq not in the [%d %llu] range\n",
			ADRV9002_EXT_LO_FREQ_MIN, ADRV9002_EXT_LO_FREQ_MAX);
		return -EINVAL;
	}

	return clk_set_rate_scaled(c->ext_lo->clk, freq, &c->ext_lo->scale);
}

static int adrv9002_phy_lo_set(struct adrv9002_rf_phy *phy, struct adrv9002_chan *c,
			       struct adi_adrv9001_InitCals *init_cals, u64 freq)
{
	struct adi_adrv9001_Carrier lo_freq;
	int ret;

	ret = api_call(phy, adi_adrv9001_Radio_Carrier_Inspect, c->port, c->number, &lo_freq);
	if (ret)
		return ret;

	ret = adrv9002_set_ext_lo(c, freq);
	if (ret)
		return ret;

	if (abs(freq - lo_freq.carrierFrequency_Hz) >= 100 * MEGA && phy->run_cals)
		init_cals->chanInitCalMask[c->idx] |= c->lo_cals;

	lo_freq.carrierFrequency_Hz = freq;
	/*
	 * !\FIXME: This is an ugly workaround for being able to control carriers when a MCS
	 * enabled profile is loaded. When MCS is on, we get 'loGenOptimization = 2' which is
	 * not a valid value and hence the API call will fail...
	 */
	if (phy->curr_profile->sysConfig.mcsMode &&
	    lo_freq.loGenOptimization > ADI_ADRV9001_LO_GEN_OPTIMIZATION_POWER_CONSUMPTION)
		lo_freq.loGenOptimization = ADI_ADRV9001_LO_GEN_OPTIMIZATION_POWER_CONSUMPTION;

	return api_call(phy, adi_adrv9001_Radio_Carrier_Configure, c->port, c->number, &lo_freq);
}

static int adrv9002_phy_lo_set_ports(struct adrv9002_rf_phy *phy, struct adrv9002_chan *c,
				     struct adi_adrv9001_InitCals *init_cals, u64 freq)
{
	int ret;
	int tx;

	if (c->port == ADI_RX) {
		int rx;

		for (rx = 0; rx < ARRAY_SIZE(phy->rx_channels); rx++) {
			if (c->lo != phy->rx_channels[rx].channel.lo)
				continue;

			ret = adrv9002_phy_lo_set(phy, &phy->rx_channels[rx].channel,
						  init_cals, freq);
			if (ret)
				return ret;
		}

		return 0;
	}

	for (tx = 0; tx < phy->chip->n_tx; tx++) {
		if (c->lo != phy->tx_channels[tx].channel.lo)
			continue;

		ret = adrv9002_phy_lo_set(phy, &phy->tx_channels[tx].channel, init_cals, freq);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Handling the carrier frequency is not really straight because it looks like we have 4
 * independent controls when in reality they are not as the device only has 2 LOs. It
 * all the depends on the LO mappings present in the current profile. First, the only way
 * a carrier is actually applied (PLL re-tunes) is if all ports on the same LO, are moved
 * into the calibrated state before changing it. Not doing so means that we may end up in
 * an inconsistent state. For instance, consider the following steps in a FDD profile where
 * RX1=RX2=LO1 (assuming both ports start at 2.4GHz):
 *   1) Move RX1 carrier to 2.45GHz -> LO1 re-tunes
 *   2) Move RX1 back to 2.4GHz -> LO1 does not re-tune and our carrier is at 2.45GHz
 * With the above steps we are left with both ports, __apparently__, at 2.4GHz but in __reality__
 * our carrier is at 2.45GHz.
 *
 * Secondly, when both ports of the same type are at the same LO, which happens on FDD and TTD
 * (with diversity) profiles, we should move the carrier of these ports together, because it's
 * just not possible to have both ports enabled with different carriers (they are on the same LO!).
 * On TDD profiles, we never move TX/RX ports together even being on the same LO. The assumption
 * is that we might have time to re-tune between RX and TX frames. If we don't, we need to manually
 * set TX and RX carriers to the same value before starting operating...
 */
static ssize_t adrv9002_phy_lo_do_write(struct adrv9002_rf_phy *phy, struct adrv9002_chan *c,
					const char *buf, size_t len)
{
	struct adi_adrv9001_InitCals init_cals = {0};
	unsigned long port_mask = 0;
	int ret, i;
	u64 freq;

	ret = kstrtoull(buf, 10, &freq);
	if (ret)
		return ret;

	guard(mutex)(&phy->lock);
	if (!c->enabled)
		return -ENODEV;

	/* move all channels on the same lo to the calibrated state */
	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		if (phy->channels[i]->lo != c->lo)
			continue;

		ret = adrv9002_channel_to_state(phy, phy->channels[i],
						ADI_ADRV9001_CHANNEL_CALIBRATED, true);
		if (ret)
			return ret;

		port_mask |= ADRV9002_PORT_MASK(phy->channels[i]);
	}

	ret = adrv9002_phy_lo_set_ports(phy, c, &init_cals, freq);
	if (ret)
		return ret;

	ret = adrv9002_phy_rerun_cals(phy, &init_cals, port_mask);
	if (ret)
		return ret;

	/* move all channels on the same lo to the cached state */
	for (i = 0; i < ARRAY_SIZE(phy->channels); i++) {
		/* If it's me... defer. See below */
		if (phy->channels[i]->port == c->port && phy->channels[i]->idx == c->idx)
			continue;

		if (phy->channels[i]->lo != c->lo)
			continue;

		ret = adrv9002_channel_to_state(phy, phy->channels[i],
						phy->channels[i]->cached_state, false);
		if (ret)
			return ret;
	}

	/*
	 * This is needed so that we are sure that the port where we are changing the
	 * carrier is the last one to transition state. This might matter in TDD profiles
	 * because a transition from calibrated to prime also causes the PLL to re-lock
	 * to the port carrier. So let's say that RX1 and TX1 are both on LO1 and start:
	 *	1. TX1 carrier set to 2.45GHz and primed
	 *	2. RX1 carrier set to 2.4GHz and primed
	 *	3. Change RX1 to rf_enabled and set carrier to 2.5GHz
	 * With the last steps we end up with 2.4GHz on LO1 which is not expected. The
	 * reason is that we would move TX1 from calibrated to prime after changing RX1
	 * which causes a re-lock.
	 */
	ret = adrv9002_channel_to_state(phy, c, c->cached_state, false);

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
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);

	switch (private) {
	case LOEXT_FREQ:
		return adrv9002_phy_lo_do_write(phy, chann, buf, len);
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
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, channel);
	struct adi_adrv9001_Carrier lo_freq;
	int ret;

	switch (private) {
	case LOEXT_FREQ:
		scoped_guard(mutex, &phy->lock) {
			if (!chann->enabled)
				return -ENODEV;

			ret = api_call(phy, adi_adrv9001_Radio_Carrier_Inspect, port,
				       chann->number, &lo_freq);
			if (ret)
				return ret;
		}
		return sysfs_emit(buf, "%llu\n", lo_freq.carrierFrequency_Hz);
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

	if (mode > ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO)
		return -EINVAL;

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	return api_call(phy, adi_adrv9001_Rx_GainControl_Mode_Set, rx->channel.number, mode);
}

static int adrv9002_get_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9001_RxGainControlMode_e gain_ctrl_mode;
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Rx_GainControl_Mode_Get,
		       rx->channel.number, &gain_ctrl_mode);

	return ret ? ret : gain_ctrl_mode;
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
	adi_adrv9001_ChannelEnableMode_e pin_mode;
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	int ret;
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, channel);

	guard(mutex)(&phy->lock);
	if (!chann->enabled)
		return -ENODEV;

	if (adrv9002_orx_enabled(phy, chann))
		return -EPERM;

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

		ret = api_call(phy, adi_adrv9001_Radio_Channel_State_Get, __port,
			       chann->number, &state);
		if (ret)
			return ret;

		if (state == ADI_ADRV9001_CHANNEL_RF_ENABLED)
			return -EPERM;
	}

	/*
	 * Still allow to control the radio state if the enable mode is set to pin
	 * and if we are in control of that GPIO. Also note that in pin mode we can
	 * only move between primed and rf_enabled. To keep the same behavior as
	 * before, if calibrated state is requested we go and fail in
	 * adi_adrv9001_Radio_Channel_ToState().
	 */
	ret = api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Get, chann->port,
		       chann->number, &pin_mode);
	if (ret)
		return ret;

	if (pin_mode == ADI_ADRV9001_PIN_MODE && chann->ensm && mode) {
		gpiod_set_value_cansleep(chann->ensm, mode - 1);
		return 0;
	}

	return api_call(phy, adi_adrv9001_Radio_Channel_ToState, port, chann->number, mode + 1);
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

	guard(mutex)(&phy->lock);
	if (!chann->enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Radio_Channel_State_Get, port, chann->number, &state);

	return ret ? ret : state - 1;
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

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Rx_InterfaceGain_Inspect, rx->channel.number,
		       &rx_intf_gain_mode, &gain_table_type);
	if (ret)
		return ret;

	rx_intf_gain_mode.controlMode = mode;

	ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_Rx_InterfaceGain_Configure,
		       rx->channel.number, &rx_intf_gain_mode);
	if (ret)
		return ret;

	return adrv9002_channel_to_state(phy, &rx->channel, rx->channel.cached_state, false);
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

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Rx_InterfaceGain_Inspect, rx->channel.number,
		       &rx_intf_gain_mode, &gain_table_type);

	return ret ? ret : rx_intf_gain_mode.controlMode;
}

static int adrv9002_get_intf_gain(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];
	int ret;
	adi_adrv9001_RxInterfaceGain_e gain;

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Rx_InterfaceGain_Get, rx->channel.number, &gain);

	return ret ? ret : gain;
}

static int adrv9002_set_intf_gain(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[chann];

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled)
		return -ENODEV;

	return api_call(phy, adi_adrv9001_Rx_InterfaceGain_Set, rx->channel.number, mode);
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

	guard(mutex)(&phy->lock);
	if (!chann->enabled)
		return -ENODEV;

	ret = api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Get, port, chann->number, &mode);

	return ret ? ret : mode;
}

static int adrv9002_set_port_en_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_chan *chann = adrv9002_get_channel(phy, port, chan_nr);

	guard(mutex)(&phy->lock);
	if (!chann->enabled)
		return -ENODEV;

	if (adrv9002_orx_enabled(phy, chann))
		/*
		 * Don't allow changing port enable mode if ORx is enabled, because it
		 * might trigger an ensm state transition which can potentially break ORx
		 */
		return -EPERM;

	return api_call(phy, adi_adrv9001_Radio_ChannelEnableMode_Set, port, chann->number, mode);
}

static int adrv9002_get_port_select(struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int c = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_chan *tx = &phy->tx_channels[c].channel;
	int mux_ctl, mux_ctl2;
	u32 mode;

	scoped_guard(mutex, &phy->lock) {
		if (!tx->enabled)
			return -ENODEV;

		mux_ctl = gpiod_get_value_cansleep(tx->mux_ctl);
		if (mux_ctl < 0)
			return mux_ctl;

		mux_ctl2 = gpiod_get_value_cansleep(tx->mux_ctl_2);
		if (mux_ctl2 < 0)
			return mux_ctl2;
	}

	if (mux_ctl && mux_ctl2)
		mode = ADRV9002_TX_A;
	else if (!mux_ctl && mux_ctl2)
		mode = ADRV9002_TX_B;
	else
		mode = -EFAULT;

	return mode;
}

static int adrv9002_set_port_select(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	int c = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[c];

	guard(mutex)(&phy->lock);
	if (!tx->channel.enabled)
		return -ENODEV;

	if (mode == ADRV9002_TX_A) {
		gpiod_set_value_cansleep(tx->channel.mux_ctl, 1);
		gpiod_set_value_cansleep(tx->channel.mux_ctl_2, 1);
	} else if (mode == ADRV9002_TX_B) {
		gpiod_set_value_cansleep(tx->channel.mux_ctl, 0);
		gpiod_set_value_cansleep(tx->channel.mux_ctl_2, 1);
	} else {
		return -EINVAL;
	}

	tx->port_sel = mode;

	return 0;
}

static int adrv9002_update_tracking_calls(const struct adrv9002_rf_phy *phy,
					  const u32 mask, const int chann,
					  const bool enable)
{
	int ret, i;
	struct adi_adrv9001_TrackingCals tracking_cals;

	ret = api_call(phy, adi_adrv9001_cals_Tracking_Get, &tracking_cals);
	if (ret)
		return ret;

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

	ret = api_call(phy, adi_adrv9001_cals_Tracking_Set, &tracking_cals);
	if (ret)
		return ret;

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

static int adrv9002_phy_rx_do_write(const struct adrv9002_rf_phy *phy, struct adrv9002_rx_chan *rx,
				    adi_common_Port_e port, uintptr_t private, const char *buf)
{
	struct adi_adrv9001_RxSettings *rx_settings = &phy->curr_profile->rx;
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &rx_settings->rxChannelCfg[rx->channel.idx];
	int ret, freq_offset_hz;
	bool enable;
	u32 val;

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case ORX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		if (private == ORX_QEC_W_POLY && !rx->orx_en)
			return -ENODEV;

		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;

		return adrv9002_update_tracking_calls(phy, rx_track_calls[private],
						      rx->channel.idx, enable);
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn)
			return -ENOTSUPP;

		ret = kstrtoint(buf, 10, &freq_offset_hz);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_Rx_FrequencyCorrection_Set, rx->channel.number,
			       freq_offset_hz, true);
		if (ret)
			return ret;

		rx->channel.nco_freq = freq_offset_hz;
		return 0;
	case RX_ADC_SWITCH:
		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;

		/* we must be in calibrated state */
		ret = adrv9002_channel_to_state(phy, &rx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_Rx_AdcSwitchEnable_Set,
			       rx->channel.number, enable);
		if (ret)
			return ret;

		return adrv9002_channel_to_state(phy, &rx->channel, rx->channel.cached_state,
						 false);
	case RX_BBDC:
		if (!rx->orx_en && port == ADI_ORX)
			return -ENODEV;

		ret = kstrtobool(buf, &enable);
		if (ret)
			return ret;
		/*
		 * Disabling the bbdc will completely disable the algorithm and set the correction
		 * value to 0. The difference with the tracking cal is that disabling it, just
		 * disables the algorithm but the last used correction value is still applied...
		 */
		return api_call(phy, adi_adrv9001_bbdc_RejectionEnable_Set, port,
				rx->channel.number, enable);
	case RX_BBDC_LOOP_GAIN:
		ret = kstrtou32(buf, 10, &val);
		if (ret)
			return ret;

		return api_call(phy, adi_adrv9010_bbdc_LoopGain_Set, rx->channel.number, val);
	default:
		return -EINVAL;
	}
}

static ssize_t adrv9002_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	int ret;

	guard(mutex)(&phy->lock);
	if (!rx->channel.enabled && port == ADI_RX)
		return -ENODEV;

	ret = adrv9002_phy_rx_do_write(phy, rx, port, private, buf);

	return ret ? ret : len;
}

static const char * const adrv9002_intf_gain[] = {
	"18dB", "12dB", "6dB", "0dB", "-6dB", "-12dB", "-18dB",
	"-24dB", "-30dB", "-36dB"
};

static int adrv9002_intf_gain_avail(const struct adrv9002_rf_phy *phy,
				    const struct adrv9002_rx_chan *rx, char *buf)
{
	adi_adrv9001_RxChannelCfg_t *rx_cfg = &phy->curr_profile->rx.rxChannelCfg[rx->channel.idx];
	/* typical case: only 0db allowed for correction table (rate >= 1Mhz) */
	u32 off = 3, nelem = 1, i;
	int sz = 0;

	/*
	 * Yes, this could be done at setup time just once
	 * (calculating the offset and number of elements) but this is also not
	 * a fastpath and like this there's no need to add more elements to
	 * struct adrv9002_rx_chan and simplifies the changes.
	 */
	if (rx_cfg->profile.gainTableType) {
		if (rx->channel.rate >= MEGA) {
			nelem = 7;
		} else {
			off = 0;
			nelem = ARRAY_SIZE(adrv9002_intf_gain);
		}
	} else {
		if (rx->channel.rate < MEGA) {
			off = 0;
			nelem = 4;
		}
	}

	for (i = off; i < off + nelem; i++)
		sz += sysfs_emit_at(buf, sz, "%s ", adrv9002_intf_gain[i]);

	buf[sz - 1] = '\n';

	return sz;
}

static int adrv9002_phy_rx_do_read(const struct adrv9002_rf_phy *phy,
				   const struct adrv9002_rx_chan *rx, adi_common_Port_e port,
				   uintptr_t private, char *buf)
{
	struct adi_adrv9001_RxSettings *rx_settings = &phy->curr_profile->rx;
	struct adi_adrv9001_RxChannelCfg *rx_cfg = &rx_settings->rxChannelCfg[rx->channel.idx];
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	adi_adrv9001_BbdcRejectionStatus_e bbdc;
	u32 rssi_pwr_mdb, val;
	u16 dec_pwr_mdb;
	bool enable;
	int ret;

	switch (private) {
	case RX_QEC_FIC:
	case RX_QEC_W_POLY:
	case ORX_QEC_W_POLY:
	case RX_HD2:
	case RX_TRACK_BBDC:
	case RX_AGC:
	case RX_RSSI_CAL:
	case RX_RFDC:
		if (private == ORX_QEC_W_POLY && !rx->orx_en)
			return -ENODEV;

		ret = api_call(phy, adi_adrv9001_cals_Tracking_Get, &tracking_cals);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%d\n",
				  !!(calls_mask[rx->channel.idx] & rx_track_calls[private]));
	case RX_DECIMATION_POWER:
		/* it might depend on proper AGC parameters */
		ret = api_call(phy, adi_adrv9001_Rx_DecimatedPower_Get,
			       rx->channel.number, &dec_pwr_mdb);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000, dec_pwr_mdb % 1000);
	case RX_RSSI:
		ret = api_call(phy, adi_adrv9001_Rx_Rssi_Read, rx->channel.number, &rssi_pwr_mdb);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u.%02u dB\n", rssi_pwr_mdb / 1000, rssi_pwr_mdb % 1000);
	case RX_RF_BANDWIDTH:
		return  sysfs_emit(buf, "%u\n", rx_cfg->profile.primarySigBandwidth_Hz);
	case RX_NCO_FREQUENCY:
		if (!rx_cfg->profile.rxDpProfile.rxNbDem.rxNbNco.rxNbNcoEn)
			return -ENOTSUPP;

		return sysfs_emit(buf, "%d\n", rx->channel.nco_freq);
	case RX_ADC_SWITCH:
		ret = api_call(phy, adi_adrv9001_Rx_AdcSwitchEnable_Get,
			       rx->channel.number, &enable);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%d\n", enable);
	case RX_BBDC:
		if (!rx->orx_en && port == ADI_ORX)
			return -ENODEV;

		ret = api_call(phy, adi_adrv9001_bbdc_RejectionEnable_Get, port,
			       rx->channel.number, &bbdc);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%d\n", bbdc);
	case RX_BBDC_LOOP_GAIN:
		ret = api_call(phy, adi_adrv9010_bbdc_LoopGain_Get, rx->channel.number, &val);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", val);
	case RX_INTERFACE_GAIN_AVAIL:
		return adrv9002_intf_gain_avail(phy, rx, buf);
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
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];

	guard(mutex)(&phy->lock);
	/*
	 * We still want to be able to get the available interface gain values to keep
	 * the same behavior as with IIO_ENUMS.
	 */
	if (!rx->channel.enabled && port == ADI_RX && private != RX_INTERFACE_GAIN_AVAIL)
		return -ENODEV;

	return adrv9002_phy_rx_do_read(phy, rx, port, private, buf);
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

	if (tx->dpd_init && tx->dpd_init->clgcEnable) {
		dev_err(&phy->spi->dev,
			"Cannot change attenuation when closed loop gain control is enabled\n");
		return -EPERM;
	}

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

	guard(mutex)(&phy->lock);
	if (!tx->channel.enabled)
		return -ENODEV;

	/* we must be in calibrated state */
	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_Tx_AttenuationMode_Set, tx->channel.number, tx_mode);
	if (ret)
		return ret;

	return adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);
}

static int adrv9002_get_atten_control_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chann = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[chann];
	adi_adrv9001_TxAttenuationControlMode_e tx_mode;
	int mode, ret;

	scoped_guard(mutex, &phy->lock) {
		if (!tx->channel.enabled)
			return -ENODEV;

		ret = api_call(phy, adi_adrv9001_Tx_AttenuationMode_Get,
			       tx->channel.number, &tx_mode);
		if (ret)
			return ret;
	}

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
	case ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC:
		mode = 3;
		break;
	default:
		return -EINVAL;
	}

	return mode;
}

static int adrv9002_phy_tx_do_read(const struct adrv9002_rf_phy *phy,
				   const struct adrv9002_chan *tx, uintptr_t private, char *buf)
{
	struct adi_adrv9001_TxProfile *tx_cfg = &phy->curr_profile->tx.txProfile[tx->idx];
	struct adi_adrv9001_TrackingCals tracking_cals;
	const u32 *calls_mask = tracking_cals.chanTrackingCalMask;
	int ret;

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		ret = api_call(phy, adi_adrv9001_cals_Tracking_Get, &tracking_cals);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%d\n", !!(calls_mask[tx->idx] & tx_track_calls[private]));
	case TX_RF_BANDWIDTH:
		return sysfs_emit(buf, "%d\n", tx_cfg->primarySigBandwidth_Hz);
	case TX_NCO_FREQUENCY:
		/*
		 * This field seems to be the only thing that changes on TX profiles when nco
		 * is enabled.
		 */
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2)
			return -ENOTSUPP;

		return sysfs_emit(buf, "%d\n", tx->nco_freq);
	default:
		return -EINVAL;
	}
}

static ssize_t adrv9002_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int channel = ADRV_ADDRESS_CHAN(chan->address);
	struct adrv9002_tx_chan *tx = &phy->tx_channels[channel];

	guard(mutex)(&phy->lock);
	if (!tx->channel.enabled)
		return -ENODEV;

	return adrv9002_phy_tx_do_read(phy, &tx->channel, private, buf);
}

static int adrv9002_phy_tx_do_write(const struct adrv9002_rf_phy *phy, struct adrv9002_chan *tx,
				    uintptr_t private, const char *buf)
{
	struct adi_adrv9001_TxProfile *tx_cfg = &phy->curr_profile->tx.txProfile[tx->idx];
	int ret, nco_freq_hz;
	bool en;

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_LB_PD:
	case TX_PAC:
	case TX_CLGC:
		ret = kstrtobool(buf, &en);
		if (ret)
			return ret;

		return adrv9002_update_tracking_calls(phy, tx_track_calls[private], tx->idx, en);
	case TX_NCO_FREQUENCY:
		if (tx_cfg->txDpProfile.txIqdmDuc.iqdmDucMode != ADI_ADRV9001_TX_DP_IQDMDUC_MODE2)
			return -ENOTSUPP;

		ret = kstrtoint(buf, 10, &nco_freq_hz);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_Tx_FrequencyCorrection_Set,
			       tx->number, nco_freq_hz, true);
		if (ret)
			return ret;

		tx->nco_freq = nco_freq_hz;
		return 0;
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
	int ret;

	guard(mutex)(&phy->lock);
	if (!tx->channel.enabled)
		return -ENODEV;

	ret = adrv9002_phy_tx_do_write(phy, &tx->channel, private, buf);

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

static const char *const adrv9002_port_select[] = {
	"tx_a", "tx_b"
};

static const struct iio_enum adrv9002_port_select_available = {
	.items = adrv9002_port_select,
	.num_items = ARRAY_SIZE(adrv9002_port_select),
	.get = adrv9002_get_port_select,
	.set = adrv9002_set_port_select,
};

static const char *const adrv9002_atten_control_mode[] = {
	"bypass", "spi", "pin", "closed_loop_gain"
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
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", 0, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE("gain_control_mode", IIO_SEPARATE, &adrv9002_agc_modes_available),
	IIO_ENUM("gain_control_mode", 0, &adrv9002_agc_modes_available),
	IIO_ENUM_AVAILABLE("digital_gain_control_mode", IIO_SEPARATE,
			   &adrv9002_digital_gain_ctl_modes_available),
	IIO_ENUM("digital_gain_control_mode", 0,
		 &adrv9002_digital_gain_ctl_modes_available),
	_ADRV9002_EXT_RX_INFO("interface_gain_available", RX_INTERFACE_GAIN_AVAIL),
	IIO_ENUM("interface_gain", 0,
		 &adrv9002_intf_gain_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9002_port_en_modes_available),
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

static const struct iio_chan_spec_ext_info adrv9002_phy_tx_mux_ext_info[] = {
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", IIO_SEPARATE, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9002_port_en_modes_available),
	IIO_ENUM("port_en_mode", IIO_SEPARATE, &adrv9002_port_en_modes_available),
	IIO_ENUM_AVAILABLE("atten_control_mode", IIO_SEPARATE,
			   &adrv9002_atten_control_mode_available),
	IIO_ENUM("port_select", IIO_SEPARATE, &adrv9002_port_select_available),
	IIO_ENUM_AVAILABLE("port_select", IIO_SEPARATE, &adrv9002_port_select_available),
	IIO_ENUM("atten_control_mode", IIO_SEPARATE,
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

static const struct iio_chan_spec_ext_info adrv9002_phy_tx_ext_info[] = {
	IIO_ENUM_AVAILABLE("ensm_mode", IIO_SEPARATE, &adrv9002_ensm_modes_available),
	IIO_ENUM("ensm_mode", IIO_SEPARATE, &adrv9002_ensm_modes_available),
	IIO_ENUM_AVAILABLE("port_en_mode", IIO_SEPARATE, &adrv9002_port_en_modes_available),
	IIO_ENUM("port_en_mode", IIO_SEPARATE, &adrv9002_port_en_modes_available),
	IIO_ENUM_AVAILABLE("atten_control_mode", IIO_SEPARATE,
			   &adrv9002_atten_control_mode_available),
	IIO_ENUM("atten_control_mode", IIO_SEPARATE,
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

static int adrv9002_channel_power_set(const struct adrv9002_rf_phy *phy,
				      struct adrv9002_chan *channel, const int val)
{
	int ret;

	dev_dbg(&phy->spi->dev, "Set power: %d, chan: %d, port: %d\n",
		val, channel->number, channel->port);

	if (!val && channel->power) {
		ret = adrv9002_channel_to_state(phy, channel, ADI_ADRV9001_CHANNEL_CALIBRATED,
						true);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_Radio_Channel_PowerDown,
			       channel->port, channel->number);
		if (ret)
			return ret;

		channel->power = false;
	} else if (val && !channel->power) {
		ret = api_call(phy, adi_adrv9001_Radio_Channel_PowerUp,
			       channel->port, channel->number);
		if (ret)
			return ret;

		ret = adrv9002_channel_to_state(phy, channel, channel->cached_state, false);
		if (ret)
			return ret;

		channel->power = true;
	}

	return 0;
}

static int adrv9002_phy_read_raw_no_rf_chan(const struct adrv9002_rf_phy *phy,
					    const struct iio_chan_spec *chan,
					    int *val, int *val2, long m)
{
	int ret;
	bool en;
	u16 temp;

	switch (m) {
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output)
			ret = api_call(phy, adi_adrv9001_AuxDac_Inspect, chan->address, &en);
		else
			ret = api_call(phy, adi_adrv9001_AuxAdc_Inspect, chan->address, &en);
		if (ret)
			return ret;

		*val = en;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_TEMP:
			ret = api_call(phy, adi_adrv9001_Temperature_Get, &temp);
			if (ret)
				return ret;

			*val = temp * 1000;
			return IIO_VAL_INT;
		case IIO_VOLTAGE:
			if (!chan->output) {
				ret = api_call(phy, adi_adrv9001_AuxAdc_Voltage_Get,
					       chan->address, &temp);
				if (ret)
					return ret;

				*val = temp;
				return IIO_VAL_INT;
			}

			ret = api_call(phy, adi_adrv9001_AuxDac_Code_Get, chan->address, &temp);
			if (ret)
				return ret;

			*val = 900 + DIV_ROUND_CLOSEST((temp - 2048) * 1700, 4096);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	};
}

static int adrv9002_hardware_gain_get(const struct adrv9002_rf_phy *phy,
				      const struct adrv9002_chan *c, adi_common_Port_e port,
				      int *val, int *val2)
{
	int temp, ret;
	u8 index;

	if (port == ADI_TX) {
		u16 atten_mdb;

		ret = api_call(phy, adi_adrv9001_Tx_Attenuation_Get, c->number, &atten_mdb);
		if (ret)
			return ret;

		*val = -1 * (atten_mdb / 1000);
		*val2 = (atten_mdb % 1000) * 1000;
		if (!*val)
			*val2 *= -1;

		return IIO_VAL_INT_PLUS_MICRO_DB;
	}

	if (port == ADI_ORX)
		ret = api_call(phy, adi_adrv9001_ORx_Gain_Get, c->number, &index);
	else
		ret = api_call(phy, adi_adrv9001_Rx_Gain_Get, c->number, &index);
	if (ret)
		return ret;

	temp = adrv9002_gainidx_to_gain(index, port);
	*val = temp / 1000;
	*val2 = temp % 1000 * 1000;

	return IIO_VAL_INT_PLUS_MICRO_DB;
}

static int adrv9002_phy_read_raw_rf_chan(const struct adrv9002_rf_phy *phy,
					 const struct adrv9002_chan *chann, adi_common_Port_e port,
					 int *val, int *val2, long m)
{
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adrv9002_hardware_gain_get(phy, chann, port, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(chann->clk);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		if (port == ADI_ORX) {
			struct adrv9002_rx_chan *rx = chan_to_rx(chann);

			if (!rx->orx_gpio)
				return -ENOTSUPP;

			*val = gpiod_get_value_cansleep(rx->orx_gpio);
			return IIO_VAL_INT;
		}

		*val = chann->power;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adrv9002_phy_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan, int *val,
				 int *val2, long m)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	struct adrv9002_chan *chann;
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);

	guard(mutex)(&phy->lock);
	if (chan->type != IIO_VOLTAGE || chan->channel > ADRV9002_CHANN_2)
		return adrv9002_phy_read_raw_no_rf_chan(phy, chan, val, val2, m);

	chann = adrv9002_get_channel(phy, port, chan_nr);
	if (port == ADI_ORX && !chan_to_rx(chann)->orx_en)
		return -ENODEV;
	if (!chann->enabled)
		return -ENODEV;

	return adrv9002_phy_read_raw_rf_chan(phy, chann, port, val, val2, m);
};

static int adrv9002_phy_write_raw_no_rf_chan(const struct adrv9002_rf_phy *phy,
					     const struct iio_chan_spec *chan, int val,
					     int val2, long mask)
{
	u16 code;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output)
			return api_call(phy, adi_adrv9001_AuxDac_Configure, chan->address, val);

		return api_call(phy, adi_adrv9001_AuxAdc_Configure, chan->address, val);
	case IIO_CHAN_INFO_PROCESSED:
		code = clamp_val(val, 50, 1750);
		code = 2048 + DIV_ROUND_CLOSEST((code - 900) * 4096, 1700);
		if (code == 4096)
			code = 4095;

		return api_call(phy, adi_adrv9001_AuxDac_Code_Set, chan->address, code);
	default:
		return -EINVAL;
	}
}

static bool adrv9002_orx_can_enable(const struct adrv9002_rf_phy *phy,
				    const struct adrv9002_chan *rx)
{
	adi_adrv9001_ChannelState_e state;
	int ret;

	/* We can't enable ORx if RX is already enabled */
	ret = api_call(phy, adi_adrv9001_Radio_Channel_State_Get, ADI_RX, rx->number, &state);
	if (ret)
		return false;

	if (state == ADI_ADRV9001_CHANNEL_RF_ENABLED) {
		dev_err(&phy->spi->dev, "RX%d cannot be enabled in order to enable ORx%d\n",
			rx->number, rx->number);
		return false;
	}

	/*
	 * TX must be enabled in order to enable ORx. We just use the rx object as we are
	 * interested in the TX on the same channel
	 */
	ret = api_call(phy, adi_adrv9001_Radio_Channel_State_Get, ADI_TX, rx->number, &state);
	if (ret)
		return false;

	if (state != ADI_ADRV9001_CHANNEL_RF_ENABLED) {
		dev_err(&phy->spi->dev, "TX%d must be enabled in order to enable ORx%d\n",
			rx->number, rx->number);
		return false;
	}

	return true;
}

static int adrv9002_hardware_gain_set(const struct adrv9002_rf_phy *phy,
				      const struct adrv9002_chan *c, adi_common_Port_e port,
				      int val, int val2)
{
	int gain;
	u32 code;
	u8 idx;

	if (port == ADI_TX) {
		if (val > 0 || (val == 0 && val2 > 0))
			return -EINVAL;

		code = ((abs(val) * 1000) + (abs(val2) / 1000));
		return api_call(phy, adi_adrv9001_Tx_Attenuation_Set, c->number, code);
	}

	gain = val * 1000 + val2 / 1000;
	idx = adrv9002_gain_to_gainidx(gain, port);

	if (port == ADI_RX)
		return api_call(phy, adi_adrv9001_Rx_Gain_Set, c->number, idx);

	return api_call(phy, adi_adrv9001_ORx_Gain_Set, c->number, idx);
}

static int adrv9002_phy_write_raw_rf_chan(const struct adrv9002_rf_phy *phy,
					  struct adrv9002_chan *chann, adi_common_Port_e port,
					  int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adrv9002_hardware_gain_set(phy, chann, port, val, val2);
	case IIO_CHAN_INFO_ENABLE:
		if (port == ADI_ORX) {
			struct adrv9002_rx_chan *rx = chan_to_rx(chann);

			if (!rx->orx_gpio)
				return -ENOTSUPP;

			if (val && !adrv9002_orx_can_enable(phy, chann))
				return -EPERM;

			gpiod_set_value_cansleep(rx->orx_gpio, !!val);
			return 0;
		}

		return adrv9002_channel_power_set(phy, chann, val);
	default:
		return -EINVAL;
	}
}

static int adrv9002_phy_write_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, int val,
				  int val2, long mask)
{
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	const int chan_nr = ADRV_ADDRESS_CHAN(chan->address);
	const adi_common_Port_e port = ADRV_ADDRESS_PORT(chan->address);
	struct adrv9002_chan *chann;

	guard(mutex)(&phy->lock);
	if (chan->type != IIO_VOLTAGE || chan->channel > ADRV9002_CHANN_2)
		return adrv9002_phy_write_raw_no_rf_chan(phy, chan, val, val2, mask);

	chann = adrv9002_get_channel(phy, port, chan_nr);
	if (port == ADI_ORX && !chan_to_rx(chann)->orx_en)
		return -ENODEV;
	if (!chann->enabled)
		return -ENODEV;

	return adrv9002_phy_write_raw_rf_chan(phy, chann, port, val, val2, mask);
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

static const struct iio_chan_spec adrv9003_phy_chan[] = {
	ADRV9002_IIO_LO_CHAN(0, "RX1_LO", ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_LO_CHAN(1, "RX2_LO", ADI_RX, ADRV9002_CHANN_2),
	ADRV9002_IIO_LO_CHAN(2, "TX1_LO", ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_TX_CHAN(0, ADI_TX, ADRV9002_CHANN_1),
	ADRV9002_IIO_AUX_CONV_CHAN(1, true, ADI_ADRV9001_AUXDAC0),
	ADRV9002_IIO_AUX_CONV_CHAN(2, true, ADI_ADRV9001_AUXDAC1),
	ADRV9002_IIO_AUX_CONV_CHAN(3, true, ADI_ADRV9001_AUXDAC2),
	ADRV9002_IIO_AUX_CONV_CHAN(4, true, ADI_ADRV9001_AUXDAC3),
	ADRV9002_IIO_RX_CHAN(0, ADI_RX, ADRV9002_CHANN_1),
	ADRV9002_IIO_RX_CHAN(1, ADI_RX, ADRV9002_CHANN_2),
	/*
	 * as Orx shares the same data path as Rx, let's just point
	 * them to the same Rx index...
	 */
	ADRV9002_IIO_ORX_CHAN(0, ADI_ORX, ADRV9002_CHANN_1),
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
static IIO_CONST_ATTR(initial_calibrations_available, "off auto run");
static IIO_DEVICE_ATTR(frequency_hopping_hop1_table_select, 0600, adrv9002_attr_show,
		       adrv9002_attr_store, ADRV9002_HOP_1_TABLE_SEL);
static IIO_DEVICE_ATTR(frequency_hopping_hop2_table_select, 0600, adrv9002_attr_show,
		       adrv9002_attr_store, ADRV9002_HOP_2_TABLE_SEL);
static IIO_DEVICE_ATTR(frequency_hopping_hop1_signal_trigger, 0200, NULL, adrv9002_attr_store,
		       ADRV9002_HOP_1_TRIGGER);
static IIO_DEVICE_ATTR(frequency_hopping_hop2_signal_trigger, 0200, NULL, adrv9002_attr_store,
		       ADRV9002_HOP_2_TRIGGER);
static IIO_DEVICE_ATTR(initial_calibrations, 0600, adrv9002_attr_show, adrv9002_attr_store,
		       ADRV9002_INIT_CALS_RUN);
static IIO_DEVICE_ATTR(warmboot_coefficients_file, 0600, adrv9002_attr_show, adrv9002_attr_store,
		       ADRV9002_WARMBOOT_SEL);
static IIO_DEVICE_ATTR(multi_chip_sync, 0200, NULL, adrv9002_attr_store, ADRV9002_MCS);

static struct attribute *adrv9002_sysfs_attrs[] = {
	&iio_const_attr_initial_calibrations_available.dev_attr.attr,
	&iio_const_attr_frequency_hopping_hop_table_select_available.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop1_table_select.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop2_table_select.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop1_signal_trigger.dev_attr.attr,
	&iio_dev_attr_frequency_hopping_hop2_signal_trigger.dev_attr.attr,
	&iio_dev_attr_initial_calibrations.dev_attr.attr,
	&iio_dev_attr_warmboot_coefficients_file.dev_attr.attr,
	&iio_dev_attr_multi_chip_sync.dev_attr.attr,
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

	guard(mutex)(&phy->lock);
	ret = api_call(phy, adi_adrv9001_gpio_GpIntStatus_Get, &status);
	if (ret)
		return IRQ_HANDLED;

	dev_dbg(&phy->spi->dev, "GP Interrupt Status 0x%08X Mask 0x%08X\n",
		status, ADRV9002_IRQ_MASK);

	active_irq = status & ADRV9002_IRQ_MASK;
	for_each_set_bit(bit, &active_irq, ARRAY_SIZE(adrv9002_irqs)) {
		/* check if there's something to be done */
		switch (adrv9002_irqs[bit].action) {
		case ADI_ADRV9001_ACT_WARN_RERUN_TRCK_CAL:
			dev_warn(&phy->spi->dev, "Re-running tracking calibrations\n");
			api_call(phy, adi_adrv9001_cals_InitCals_Run,
				 &phy->init_cals, 60000, &error);
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

	return IRQ_HANDLED;
}

static int adrv9002_dgpio_config(const struct adrv9002_rf_phy *phy)
{
	struct adrv9002_gpio *dgpio = phy->adrv9002_gpios;
	int i, ret;

	for (i = 0; i < phy->ngpios; i++) {
		dev_dbg(&phy->spi->dev, "Set dpgio: %d, signal: %d\n",
			dgpio[i].gpio.pin, dgpio[i].signal);

		ret = api_call(phy, adi_adrv9001_gpio_Configure, dgpio[i].signal, &dgpio[i].gpio);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9002_init_cals_handle(struct adrv9002_rf_phy *phy)
{
	const struct firmware *fw;
	int ret;
	u8 errors;

	if (!phy->curr_profile->sysConfig.warmBootEnable || phy->warm_boot.coeffs_name[0] == '\0')
		goto run_cals;

	dev_dbg(&phy->spi->dev, "Requesting warmboot coefficients: \"%s\"n",
		phy->warm_boot.coeffs_name);

	ret = request_firmware(&fw, phy->warm_boot.coeffs_name, &phy->spi->dev);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_UniqueArray_Set,
		       (u8 *)fw->data, phy->init_cals.chanInitCalMask[0],
		       phy->init_cals.chanInitCalMask[1]);
	release_firmware(fw);
	if (ret)
		return ret;

run_cals:
	return api_call(phy, adi_adrv9001_cals_InitCals_Run, &phy->init_cals, 60000, &errors);
}

static int adrv9001_rx_path_config(struct adrv9002_rf_phy *phy,
				   const adi_adrv9001_ChannelState_e state)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(phy->rx_channels); i++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		/* For each rx channel enabled */
		if (!rx->channel.enabled)
			continue;

		if (!rx->pin_cfg)
			goto agc_cfg;

		ret = api_call(phy, adi_adrv9001_Rx_GainControl_PinMode_Configure,
			       rx->channel.number, rx->pin_cfg);
		if (ret)
			return ret;

agc_cfg:
		ret = api_call(phy, adi_adrv9001_Rx_GainControl_Configure,
			       rx->channel.number, &rx->agc);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_Radio_Channel_ToState, ADI_RX,
			       rx->channel.number, state);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9002_tx_set_dac_full_scale(const struct adrv9002_rf_phy *phy)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		const struct adrv9002_tx_chan *tx = &phy->tx_channels[i];

		if (!tx->channel.enabled || !tx->dac_boost_en)
			continue;

		ret = api_call(phy, adi_adrv9001_Tx_OutputPowerBoost_Set, tx->channel.number, true);
		if (ret)
			return ret;
	}

	return ret;
}

static int adrv9002_dpd_ext_path_set(const struct adrv9002_rf_phy *phy)
{
	u8 init_calls_error;
	u32 delay, c;
	int ret;

	for (c = 0; c < phy->chip->n_tx; c++) {
		const struct adrv9002_tx_chan *tx = &phy->tx_channels[c];

		if (!tx->channel.enabled || !tx->ext_path_calib)
			continue;

		/* let's first measure the delay */
		ret = api_call(phy, adi_adrv9001_cals_ExternalPathDelay_Calibrate,
			       tx->channel.number, 60000, &init_calls_error, &delay);
		if (ret)
			return ret;

		ret = api_call(phy, adi_adrv9001_cals_ExternalPathDelay_Set,
			       tx->channel.number, delay);
		if (ret)
			return ret;
	}

	return 0;
}

static int adrv9002_tx_path_config(const struct adrv9002_rf_phy *phy,
				   const adi_adrv9001_ChannelState_e state)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(phy->tx_channels); i++) {
		const struct adrv9002_tx_chan *tx = &phy->tx_channels[i];

		/* For each tx channel enabled */
		if (!tx->channel.enabled)
			continue;

		if (!tx->elb_en || !tx->dpd_init || !tx->dpd_init->enable)
			goto pin_cfg;

		ret = api_call(phy, adi_adrv9001_dpd_Configure, tx->channel.number, tx->dpd);
		if (ret)
			return ret;

pin_cfg:
		if (!tx->pin_cfg)
			goto rf_enable;

		ret = api_call(phy, adi_adrv9001_Tx_Attenuation_PinControl_Configure,
			       tx->channel.number, tx->pin_cfg);
		if (ret)
			return ret;

rf_enable:
		ret = api_call(phy, adi_adrv9001_Radio_Channel_ToState, ADI_TX,
			       tx->channel.number, state);
		if (ret)
			return ret;
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
			pos |= ADRV9002_PORT_MASK(c);
		else
			pos |= ADRV9002_PORT_MASK(c);
	}

	phy->init_cals.chanInitCalMask[0] = adrv9002_init_cals_mask[pos][0];
	phy->init_cals.chanInitCalMask[1] = adrv9002_init_cals_mask[pos][1];

	dev_dbg(&phy->spi->dev, "pos: %u, Chan1:%X, Chan2:%X", pos,
		phy->init_cals.chanInitCalMask[0],
		phy->init_cals.chanInitCalMask[1]);
}

static int adrv9002_ext_lo_validate(struct adrv9002_rf_phy *phy, int idx, bool tx)
{
	adi_adrv9001_ClockSettings_t *clocks = &phy->curr_profile->clocks;
	adi_adrv9001_LoSel_e lo_selects[] = {
		clocks->rx1LoSelect, clocks->tx1LoSelect, clocks->rx2LoSelect, clocks->tx2LoSelect
	};
	adi_adrv9001_PllLoMode_e modes[] = { clocks->rfPll1LoMode, clocks->rfPll2LoMode };
	u16 dividers[] = { clocks->extLo1Divider, clocks->extLo2Divider };
	/* -1 since the enums start at 1 */
	unsigned int lo = lo_selects[idx * 2 + tx] - 1;
	struct device *dev = &phy->spi->dev;

	if (lo >= ARRAY_SIZE(modes)) {
		/*
		 * Anything other than ADI_ADRV9001_LOSEL_LO{1|2} should be wrong...
		 */
		dev_err(dev, "Unknown LO(%u) on %s%d\n", lo, tx ? "TX" : "RX", idx + 1);
		return -EINVAL;
	}

	if (modes[lo] == ADI_ADRV9001_INT_LO1)
		return ADRV9002_NO_EXT_LO;

	/*
	 * Alright, if external LO is being set on the profile for this port, we need to have
	 * a matching clk to control.
	 */
	if (!phy->ext_los[lo].clk) {
		dev_err(dev, "Ext LO%d set for %s%d but not controlling clk provided via dts\n",
			lo + 1, tx ? "TX" : "RX", idx + 1);
		return -EINVAL;
	}

	/* should never happen but we should also not blindly trust in the loaded profile... */
	if (!dividers[lo]) {
		dev_err(dev, "LO%d cannot have a divider of 0!\n", lo + 1);
		return -EINVAL;
	}

	phy->ext_los[lo].divider = dividers[lo];

	dev_dbg(dev, "EXT LO%d being used for %s%d with div(%u)\n", lo + 1, tx ? "TX" : "RX",
		idx + 1, dividers[lo]);

	return lo;
}

static int adrv9002_rx_validate_profile(struct adrv9002_rf_phy *phy, unsigned int idx,
					const struct adi_adrv9001_RxChannelCfg *rx_cfg)
{
	struct device *dev = &phy->spi->dev;

	if (phy->ssi_type != rx_cfg[idx].profile.rxSsiConfig.ssiType) {
		dev_err(dev, "SSI interface mismatch. PHY=%d, RX%d=%d\n",
			phy->ssi_type, idx + 1, rx_cfg[idx].profile.rxSsiConfig.ssiType);
		return -EINVAL;
	}

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS && !rx_cfg[idx].profile.rxSsiConfig.ddrEn) {
		dev_err(dev, "RX%d: Single Data Rate port not supported for LVDS\n",
			idx + 1);
		return -EINVAL;
	}

	if (rx_cfg[idx].profile.rxSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
		dev_err(dev, "SSI interface Long Strobe not supported\n");
		return -EINVAL;
	}

	if (!phy->rx2tx2 || !idx)
		return 0;

	if (rx_cfg[idx].profile.rxOutputRate_Hz != phy->rx_channels[0].channel.rate) {
		dev_err(dev, "In rx2tx2, RX%d rate=%u must be equal to RX1, rate=%ld\n", idx + 1,
			rx_cfg[idx].profile.rxOutputRate_Hz, phy->rx_channels[0].channel.rate);
		return -EINVAL;
	}

	if (!phy->rx_channels[0].channel.enabled) {
		dev_err(dev, "In rx2tx2, RX%d cannot be enabled while RX1 is disabled", idx + 1);
		return -EINVAL;
	}

	return 0;
}

static int adrv9002_tx_validate_profile(struct adrv9002_rf_phy *phy, unsigned int idx,
					const struct adi_adrv9001_TxProfile *tx_cfg)
{
	struct adrv9002_tx_chan *tx = &phy->tx_channels[idx];
	struct device *dev = &phy->spi->dev;
	struct adrv9002_rx_chan *rx;

	/* check @tx_only comments in adrv9002.h to better understand the next checks */
	if (phy->ssi_type != tx_cfg[idx].txSsiConfig.ssiType) {
		dev_err(dev, "SSI interface mismatch. PHY=%d, TX%d=%d\n",
			phy->ssi_type, idx + 1,  tx_cfg[idx].txSsiConfig.ssiType);
		return -EINVAL;
	}

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS && !tx_cfg[idx].txSsiConfig.ddrEn) {
		dev_err(dev, "TX%d: Single Data Rate port not supported for LVDS\n", idx + 1);
		return -EINVAL;
	}

	if (tx_cfg[idx].txSsiConfig.strobeType == ADI_ADRV9001_SSI_LONG_STROBE) {
		dev_err(dev, "SSI interface Long Strobe not supported\n");
		return -EINVAL;
	}

	if (phy->rx2tx2) {
		struct adrv9002_chan *rx1 = &phy->rx_channels[0].channel;
		struct adrv9002_chan *tx1 = &phy->tx_channels[0].channel;

		/*
		 * In rx2tx2 mode, if TX uses RX as the reference clock, we just need to
		 * validate against RX1 since in this mode RX2 cannot be enabled without RX1. The
		 * same goes for the rate that must be the same.
		 */
		if (tx->rx_ref_clk && !rx1->enabled) {
			/*
			 * pretty much means that in this case either all channels are
			 * disabled, which obviously does not make sense, or RX1 must
			 * be enabled...
			 */
			dev_err(dev, "In rx2tx2, TX%d cannot be enabled while RX1 is disabled",
				idx + 1);
			return -EINVAL;
		}

		if (tx->rx_ref_clk  && tx_cfg[idx].txInputRate_Hz != rx1->rate) {
			/*
			 * pretty much means that in this case, all ports must have
			 * the same rate. We match against RX1 since RX2 can be disabled
			 * even if it does not make much sense to disable it in rx2tx2 mode
			 */
			dev_err(dev, "In rx2tx2, TX%d rate=%u must be equal to RX1, rate=%ld\n",
				idx + 1, tx_cfg[idx].txInputRate_Hz, rx1->rate);
			return -EINVAL;
		}

		if (!tx->rx_ref_clk  && idx && tx_cfg[idx].txInputRate_Hz != tx1->rate) {
			dev_err(dev, "In rx2tx2, TX%d rate=%u must be equal to TX1, rate=%ld\n",
				idx + 1, tx_cfg[idx].txInputRate_Hz, tx1->rate);
			return -EINVAL;
		}

		if (idx && !tx1->enabled) {
			dev_err(dev, "In rx2tx2, TX%d cannot be enabled while TX1 is disabled",
				idx + 1);
			return -EINVAL;
		}

		return 0;
	}

	if (!tx->rx_ref_clk)
		return 0;

	/* Alright, RX clock is driving us... */
	rx = &phy->rx_channels[tx->rx_ref_clk - 1];
	if (!rx->channel.enabled) {
		dev_err(dev, "TX%d cannot be enabled while RX%d is disabled", idx + 1,
			rx->channel.number);
		return -EINVAL;
	}

	if (tx_cfg[idx].txInputRate_Hz != rx->channel.rate) {
		dev_err(dev, "TX%d rate=%u must be equal to RX%d, rate=%ld\n", idx + 1,
			tx_cfg[idx].txInputRate_Hz, rx->channel.number, rx->channel.rate);
		return -EINVAL;
	}

	return 0;
}

static void adrv9002_validate_device_clkout(struct adrv9002_rf_phy *phy, u32 devclk)
{
	unsigned long out_rate;

	/* validated internally by the API for disabled case */
	if (phy->dev_clkout_div == ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS ||
	    phy->dev_clkout_div == ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED)
		return;

	/*
	 * Ideally, this would be implemented with registering a clock provider for
	 * dev clkout. But given that there's no API to easily change the divider or
	 * to even get it and that ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED pretty
	 * much makes the internal API to decide the divider to use, it would be
	 * cumbersome and far from ideal to implement this through CCF - we probably
	 * would have to not allow the DISABLED option and only have a fixed clock
	 * after profile load. Given all the limitations go the easy way. If there's
	 * enough motivation to implement this through CCF later on, we can propose
	 * some new internal APIs.
	 */
	out_rate = devclk >> phy->dev_clkout_div;
	if (out_rate < ADRV9002_DEV_CLKOUT_MIN || out_rate > ADRV9002_DEV_CLKOUT_MAX) {
		dev_dbg(&phy->spi->dev, "Invalid device output clk(%lu) not in [%lu %lu]\n",
			out_rate, ADRV9002_DEV_CLKOUT_MIN, ADRV9002_DEV_CLKOUT_MAX);
		/*
		 * If we can't get a valid rate with the new devclk + divider, let's defer
		 * to the internal API to try and get a valid divider that puts us in the
		 * supported range. Not ideal if someone wants an exact output clocks but
		 * better than failing probe. A runtime parameter for a divider does not
		 * make sense either. Therefore, a workaround for those wanting to dynamically
		 * change the output clock (in an exact way) is to overwrite phy->dev_clkout_div
		 * in debugfs.
		 */
		phy->dev_clkout_div = ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED;
	}
}

static int adrv9002_validate_device_clk(struct adrv9002_rf_phy *phy,
					const struct adi_adrv9001_ClockSettings *clk_ctrl)
{
	unsigned long rate;
	long new_rate;
	int ret;

	rate = clk_get_rate(phy->dev_clk);
	if (rate == clk_ctrl->deviceClock_kHz * KILO)
		return 0;

	/*
	 * If they don't match let's try to set the desired ref clk. Furthermore, let's
	 * be strict about not rounding it. If someones specifies some clk in the
	 * profile, then we should be capable of getting exactly that exact rate.
	 *
	 * !NOTE: we may need some small hysteris though... but let's add one when and
	 * if we really need one.
	 */
	new_rate = clk_round_rate(phy->dev_clk, clk_ctrl->deviceClock_kHz * KILO);
	if (new_rate < 0 || new_rate != clk_ctrl->deviceClock_kHz * KILO) {
		dev_err(&phy->spi->dev, "Cannot set ref_clk to (%lu), got (%ld)\n",
			clk_ctrl->deviceClock_kHz * KILO, new_rate);
		return new_rate < 0 ? new_rate : -EINVAL;
	}

	ret = clk_set_rate(phy->dev_clk, new_rate);
	if (ret)
		return ret;

	adrv9002_validate_device_clkout(phy, new_rate);

	return 0;
}

static int adrv9002_validate_profile(struct adrv9002_rf_phy *phy)
{
	const struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	const struct adi_adrv9001_TxProfile *tx_cfg = phy->curr_profile->tx.txProfile;
	struct adi_adrv9001_ClockSettings *clks = &phy->curr_profile->clocks;
	unsigned long rx_mask = phy->curr_profile->rx.rxInitChannelMask;
	unsigned long tx_mask = phy->curr_profile->tx.txInitChannelMask;
	int i, lo, ret;

	ret = adrv9002_validate_device_clk(phy, clks);
	if (ret)
		return ret;

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		/* rx validations */
		if (!test_bit(ADRV9002_RX_BIT_START + i, &rx_mask))
			continue;

		lo = adrv9002_ext_lo_validate(phy, i, false);
		if (lo < 0)
			return lo;

		ret = adrv9002_rx_validate_profile(phy, i, rx_cfg);
		if (ret)
			return ret;

		dev_dbg(&phy->spi->dev, "RX%d enabled\n", i + 1);
		rx->channel.power = true;
		rx->channel.enabled = true;
		rx->channel.nco_freq = 0;
		rx->channel.rate = rx_cfg[i].profile.rxOutputRate_Hz;
		if (lo < ADI_ADRV9001_LOSEL_LO2)
			rx->channel.ext_lo = &phy->ext_los[lo];
		rx->channel.lo = i ? clks->rx2LoSelect : clks->rx1LoSelect;
		rx->channel.lo_cals = ADI_ADRV9001_INIT_LO_RETUNE & ~ADI_ADRV9001_INIT_CAL_TX_ALL;
	}

	for (i = 0; i < phy->chip->n_tx; i++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[i];
		struct adrv9002_rx_chan *rx = &phy->rx_channels[i];

		if (!test_bit(ADRV9002_TX_BIT_START + i, &tx_mask))
			continue;

		lo = adrv9002_ext_lo_validate(phy, i, true);
		if (lo < 0)
			return lo;

		ret = adrv9002_tx_validate_profile(phy, i, tx_cfg);
		if (ret)
			return ret;

		dev_dbg(&phy->spi->dev, "TX%d enabled\n", i + 1);
		/* orx actually depends on whether or not TX is enabled and not RX */
		rx->orx_en = test_bit(ADRV9002_ORX_BIT_START + i, &rx_mask);
		tx->channel.power = true;
		tx->channel.enabled = true;
		tx->channel.nco_freq = 0;
		tx->channel.rate = tx_cfg[i].txInputRate_Hz;
		tx->elb_en = test_bit(ADRV9002_ELB_BIT_START + i, &rx_mask);
		if (lo < ADI_ADRV9001_LOSEL_LO2)
			tx->channel.ext_lo = &phy->ext_los[lo];
		tx->channel.lo = i ? clks->tx2LoSelect : clks->tx1LoSelect;
		tx->channel.lo_cals = ADI_ADRV9001_INIT_LO_RETUNE & ~ADI_ADRV9001_INIT_CAL_RX_ALL;
	}

	return 0;
}

static int adrv9002_power_mgmt_config(const struct adrv9002_rf_phy *phy)
{
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

	return api_call(phy, adi_adrv9001_powermanagement_Configure, &power_mgmt);
}

static int adrv9002_digital_init(const struct adrv9002_rf_phy *phy)
{
	int spi_mode = ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252;
	int ret;
	u8 tx_mask = 0;
	int c;
	adi_adrv9001_RxChannelCfg_t *rx_cfg = phy->curr_profile->rx.rxChannelCfg;

	ret = adi_adrv9001_arm_AhbSpiBridge_Enable(phy->adrv9001);
	if (ret)
		return ret;

	/*
	 * If we find a custom stream, we will load that. Otherwise we will load the default one.
	 * Note that if we are in the middle of filling @phy->stream_buf with a new stream and we
	 * get somehow here, the default one will be used. Either way, after filling the stream, we
	 * __must__ write a new profile which will get us here again and we can then load then new
	 * stream.
	 */
	if (phy->stream_size == ADI_ADRV9001_STREAM_BINARY_IMAGE_FILE_SIZE_BYTES)
		ret = api_call(phy, adi_adrv9001_Stream_Image_Write, 0,
			       phy->stream_buf, phy->stream_size, spi_mode);
	else
		ret = api_call(phy, adi_adrv9001_Utilities_StreamImage_Load,
			       "Navassa_Stream.bin", spi_mode);
	if (ret)
		return ret;

	/* program arm firmware */
	ret = api_call(phy, adi_adrv9001_Utilities_ArmImage_Load,
		       "Navassa_EvaluationFw.bin", spi_mode);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_arm_Profile_Write, phy->curr_profile);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_arm_PfirProfiles_Write, phy->curr_profile);
	if (ret)
		return ret;

	/* Load gain tables */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		const struct adrv9002_rx_chan *rx = &phy->rx_channels[c];
		const struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
		adi_adrv9001_RxProfile_t *p = &rx_cfg[c].profile;
		adi_adrv9001_RxGainTableType_e t_type;
		const char *rx_table;

		if (p->gainTableType) {
			rx_table = "RxGainTable_GainCompensated.csv";
			t_type = ADI_ADRV9001_RX_GAIN_COMPENSATION_TABLE;
		} else {
			rx_table = "RxGainTable.csv";
			t_type = ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE;
		}

		if (rx->orx_en || tx->channel.enabled) {
			ret = api_call(phy, adi_adrv9001_Utilities_RxGainTable_Load, ADI_ORX,
				       "ORxGainTable.csv", rx->channel.number, &p->lnaConfig,
				       t_type);
			if (ret)
				return ret;
		}

		if (tx->channel.enabled)
			tx_mask |= tx->channel.number;

		if (!rx->channel.enabled)
			continue;

		ret = api_call(phy, adi_adrv9001_Utilities_RxGainTable_Load, ADI_RX, rx_table,
			       rx->channel.number, &p->lnaConfig, t_type);
		if (ret)
			return ret;
	}

	if (tx_mask) {
		ret = api_call(phy, adi_adrv9001_Utilities_TxAttenTable_Load,
			       "TxAttenTable.csv", tx_mask);
		if (ret)
			return ret;
	}

	ret = adrv9002_power_mgmt_config(phy);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_arm_Start);
	if (ret)
		return ret;

	return api_call(phy, adi_adrv9001_arm_StartStatus_Check, 5000000);
}

static u64 adrv9002_get_init_carrier(const struct adrv9002_chan *c)
{
	u64 lo_freq;

	if (!c->ext_lo) {
		if (c->carrier)
			return c->carrier;

		/* If no external LO, keep the same values as before */
		if (c->port == ADI_RX)
			return 2400000000ULL;

		return 2450000000ULL;
	}

	lo_freq = clk_get_rate_scaled(c->ext_lo->clk, &c->ext_lo->scale);
	return DIV_ROUND_CLOSEST_ULL(lo_freq, c->ext_lo->divider);
}

static int adrv9002_ext_lna_set(const struct adrv9002_rf_phy *phy,
				struct adrv9002_rx_chan *rx)
{
	struct adi_adrv9001_RxChannelCfg *rx_cfg = phy->curr_profile->rx.rxChannelCfg;
	struct adi_adrv9001_RxProfile *p = &rx_cfg[rx->channel.idx].profile;
	int ret;

	if (!p->lnaConfig.externalLnaPresent)
		return 0;

	ret = api_call(phy, adi_adrv9001_Rx_ExternalLna_Configure, rx->channel.number,
		       &p->lnaConfig, p->gainTableType);
	if (ret)
		return ret;

	/* min gain index may have changed */
	rx->agc.minGainIndex =  p->lnaConfig.minGainIndex;
	/*
	 * Also make sure to update the AGC LNA settling delay. Otherwise we would overwrite it
	 * when configuring AGC.
	 */
	rx->agc.extLna.settlingDelay = p->lnaConfig.settlingDelay;
	return 0;
}

static int adrv9002_init_dpd(const struct adrv9002_rf_phy *phy, const struct adrv9002_tx_chan *tx)
{
	if (!tx->elb_en || !tx->dpd_init || !tx->dpd_init->enable)
		return 0;

	return api_call(phy, adi_adrv9001_dpd_Initial_Configure, tx->channel.number, tx->dpd_init);
}

/*
 * All of these structures are taken from TES when exporting the default profile to C code. Consider
 * about having all of these configurable through devicetree.
 */
static int adrv9002_radio_init(const struct adrv9002_rf_phy *phy)
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

	ret = api_call(phy, adi_adrv9001_Radio_PllLoopFilter_Set,
		       ADI_ADRV9001_PLL_LO1, &pll_loop_filter);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_Radio_PllLoopFilter_Set,
		       ADI_ADRV9001_PLL_LO2, &pll_loop_filter);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_Radio_PllLoopFilter_Set,
		       ADI_ADRV9001_PLL_AUX, &pll_loop_filter);
	if (ret)
		return ret;

	for (chan = 0; chan < ARRAY_SIZE(phy->channels); chan++) {
		struct adrv9002_chan *c = phy->channels[chan];
		struct adi_adrv9001_ChannelEnablementDelays en_delays;

		if (!c->enabled)
			continue;

		if (c->port == ADI_RX) {
			ret = adrv9002_ext_lna_set(phy, chan_to_rx(c));
			if (ret)
				return ret;
		}
		/*
		 * For some low rate profiles, the intermediate frequency is non 0.
		 * In these cases, forcing it 0, will cause a firmware error. Hence, we need to
		 * read what we have and make sure we just change the carrier frequency...
		 */
		ret = api_call(phy, adi_adrv9001_Radio_Carrier_Inspect, c->port,
			       c->number, &carrier);
		if (ret)
			return ret;

		carrier.carrierFrequency_Hz = adrv9002_get_init_carrier(c);
		ret = api_call(phy, adi_adrv9001_Radio_Carrier_Configure,
			       c->port, c->number, &carrier);
		if (ret)
			return ret;

		adrv9002_en_delays_ns_to_arm(phy, &c->en_delays_ns, &en_delays);

		ret = api_call(phy, adi_adrv9001_Radio_ChannelEnablementDelays_Configure,
			       c->port, c->number, &en_delays);
		if (ret)
			return ret;

		if (c->port == ADI_TX) {
			ret = adrv9002_init_dpd(phy, chan_to_tx(c));
			if (ret)
				return ret;
		}

		if (!phy->curr_profile->sysConfig.mcsMode)
			continue;

		ret = api_call(phy, adi_adrv9001_Mcs_ChannelMcsDelay_Set, c->port,
			       c->number, &c->mcs_delay);
		if (ret)
			return ret;
	}

	return api_call(phy, adi_adrv9001_arm_System_Program, channel_mask);
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
	int ret;
	adi_adrv9001_ChannelState_e init_state;

	/* in TDD we cannot start with all ports enabled as RX/TX cannot be on at the same time */
	if (phy->curr_profile->sysConfig.duplexMode == ADI_ADRV9001_TDD_MODE)
		init_state = ADI_ADRV9001_CHANNEL_PRIMED;
	else
		init_state = ADI_ADRV9001_CHANNEL_RF_ENABLED;

	adi_common_ErrorClear(&phy->adrv9001->common);
	ret = api_call(phy, adi_adrv9001_HwOpen, &adrv9002_spi);
	if (ret)
		return ret;

	ret = adrv9002_validate_profile(phy);
	if (ret)
		return ret;

	adrv9002_compute_init_cals(phy);

	adrv9002_log_enable(&phy->adrv9001->common);

	ret = api_call(phy, adi_adrv9001_InitAnalog, phy->curr_profile, phy->dev_clkout_div);
	if (ret)
		return ret;

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
		ret = api_call(phy, adi_adrv9001_fh_Configure, &phy->fh);
		if (ret)
			return ret;
	}

	ret = adrv9002_init_cals_handle(phy);
	if (ret)
		return ret;

	ret = adrv9002_dpd_ext_path_set(phy);
	if (ret)
		return ret;

	ret = adrv9001_rx_path_config(phy, init_state);
	if (ret)
		return ret;

	ret = adrv9002_tx_path_config(phy, init_state);
	if (ret)
		return ret;

	/* unmask IRQs */
	ret = api_call(phy, adi_adrv9001_gpio_GpIntMask_Set, ~ADRV9002_IRQ_MASK);
	if (ret)
		return ret;

	return adrv9002_dgpio_config(phy);
}

int adrv9002_intf_change_delay(const struct adrv9002_rf_phy *phy, const int channel, u8 clk_delay,
			       u8 data_delay, const bool tx)
{
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};

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

	return api_call(phy, adi_adrv9001_Ssi_Delay_Configure, phy->ssi_type, &delays);
}

adi_adrv9001_SsiTestModeData_e adrv9002_get_test_pattern(const struct adrv9002_rf_phy *phy,
							 unsigned int chan, bool rx, bool stop)
{
	const struct adrv9002_chan *tx = &phy->tx_channels[chan].channel;

	if (stop)
		return ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL;
	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		return ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE;
	if (rx)
		return ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;

	/*
	 * Some low rate profiles don't play well with prbs15. The reason is
	 * still unclear. We suspect that the chip error checker might have
	 * some time constrains and cannot reliable validate prbs15 full
	 * sequences in the test time. Using a shorter sequence fixes the
	 * problem...
	 *
	 * We use the same threshold as in the rx interface gain for narrow band.
	 */
	if (tx->rate < 1 * MEGA)
		return ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7;

	return ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15;
}

int adrv9002_check_tx_test_pattern(const struct adrv9002_rf_phy *phy, const int chann)
{
	int ret;
	const struct adrv9002_chan *chan = &phy->tx_channels[chann].channel;
	struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};
	struct adi_adrv9001_TxSsiTestModeStatus status = {0};
	adi_adrv9001_SsiDataFormat_e data_fmt = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA;

	cfg.testData = adrv9002_get_test_pattern(phy, chann, false, false);

	ret = api_call(phy, adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect,
		       chan->number, phy->ssi_type, data_fmt, &cfg, &status);
	if (ret)
		return ret;

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
	ret = api_call(phy, adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect,
		       chan->number, phy->ssi_type, data_fmt, &cfg, &status);
	if (ret)
		return ret;

	dev_dbg(&phy->spi->dev, "[c%d]: d_e:%u, f_f:%u f_e:%u, s_e:%u", chan->number,
		status.dataError, status.fifoFull, status.fifoEmpty, status.strobeAlignError);

	if (status.dataError)
		return 1;

	return 0;
}

int adrv9002_intf_test_cfg(const struct adrv9002_rf_phy *phy, const int chann, const bool tx,
			   const bool stop)
{
	int ret;
	const struct adrv9002_chan *chan;
	adi_adrv9001_SsiDataFormat_e data_fmt = ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA;

	dev_dbg(&phy->spi->dev, "cfg test stop:%u, ssi:%d, c:%d, tx:%d\n", stop,
		phy->ssi_type, chann, tx);

	if (tx) {
		struct adi_adrv9001_TxSsiTestModeCfg cfg = {0};

		chan = &phy->tx_channels[chann].channel;

		cfg.testData = adrv9002_get_test_pattern(phy, chann, false, stop);
		ret = api_call(phy, adi_adrv9001_Ssi_Tx_TestMode_Configure,
			       chan->number, phy->ssi_type, data_fmt, &cfg);
		if (ret)
			return ret;

		if (!phy->rx2tx2)
			return 0;

		/* on rx2tx2 we will only get here on index 0 so the following is fine */
		chan = &phy->tx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = api_call(phy, adi_adrv9001_Ssi_Tx_TestMode_Configure,
			       chan->number, phy->ssi_type, data_fmt, &cfg);
	} else {
		struct adi_adrv9001_RxSsiTestModeCfg cfg = {0};

		chan = &phy->rx_channels[chann].channel;

		cfg.testData = adrv9002_get_test_pattern(phy, chann, true, stop);
		ret = api_call(phy, adi_adrv9001_Ssi_Rx_TestMode_Configure,
			       chan->number, phy->ssi_type, data_fmt, &cfg);
		if (ret)
			return ret;

		if (!phy->rx2tx2)
			return 0;

		/* on rx2tx2 we will only get here on index 0 so the following is fine */
		chan = &phy->rx_channels[chann + 1].channel;
		if (!chan->enabled)
			return 0;

		ret = api_call(phy, adi_adrv9001_Ssi_Rx_TestMode_Configure,
			       chan->number, phy->ssi_type, data_fmt, &cfg);
	}

	return ret;
}

static int adrv9002_intf_tuning(const struct adrv9002_rf_phy *phy)
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

	return api_call(phy, adi_adrv9001_Ssi_Delay_Configure, phy->ssi_type, &delays);

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
		phy->rx_channels[i].channel.ext_lo = NULL;
		phy->rx_channels[i].channel.lo = 0;
		phy->rx_channels[i].channel.lo_cals = 0;
		phy->tx_channels[i].channel.enabled = 0;
		phy->tx_channels[i].channel.ext_lo = NULL;
		phy->tx_channels[i].channel.lo = 0;
		phy->tx_channels[i].channel.lo_cals = 0;
	}

	phy->profile_len = scnprintf(phy->profile_buf, sizeof(phy->profile_buf),
				     "No profile loaded...\n");
	memset(&phy->adrv9001->devStateInfo, 0,
	       sizeof(phy->adrv9001->devStateInfo));

	/*
	 * By default, let's not run init cals for LO changes >= 100MHz to keep
	 * the same behavior as before (as doing it automatically is time consuming).
	 */
	phy->run_cals = false;
	phy->mcs_run = false;
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

static const char *const lo_maps[] = {
	"Unknown",
	"L01",
	"L02",
	"AUX LO"
};

static const char *const duplex[] = {
	"TDD",
	"FDD"
};

static const char *const ssi[] = {
	"Disabled",
	"CMOS",
	"LVDS"
};

static const char *const mcs[] = {
	"Disabled",
	"Enabled",
	"Enabled RFPLL Phase"
};

static const char *const rx_gain_type[] = {
	"Correction",
	"Compensated"
};

static const char *const warm_boot[] = {
	"Disabled",
	"Enabled"
};

static void adrv9002_fill_profile_read(struct adrv9002_rf_phy *phy)
{
	struct adi_adrv9001_DeviceSysConfig *sys = &phy->curr_profile->sysConfig;
	struct adi_adrv9001_ClockSettings *clks = &phy->curr_profile->clocks;
	struct adi_adrv9001_RxSettings *rx = &phy->curr_profile->rx;
	struct adi_adrv9001_RxChannelCfg *rx_cfg = rx->rxChannelCfg;
	struct adi_adrv9001_TxSettings *tx = &phy->curr_profile->tx;

	phy->profile_len = scnprintf(phy->profile_buf, sizeof(phy->profile_buf),
				     "Device clk(Hz): %d\n"
				     "Clk PLL VCO(Hz): %lld\n"
				     "ARM Power Saving Clk Divider: %d\n"
				     "RX1 LO: %s\n"
				     "RX2 LO: %s\n"
				     "TX1 LO: %s\n"
				     "TX1 DPD enable: %d\n"
				     "TX2 LO: %s\n"
				     "TX2 DPD enable: %d\n"
				     "RX1 Gain Table Type: %s\n"
				     "RX2 Gain Table Type: %s\n"
				     "RX Channel Mask: 0x%x\n"
				     "TX Channel Mask: 0x%x\n"
				     "Duplex Mode: %s\n"
				     "FH enable: %d\n"
				     "MCS mode: %s\n"
				     "WarmBoot: %s\n"
				     "SSI interface: %s\n", clks->deviceClock_kHz * 1000,
				     clks->clkPllVcoFreq_daHz * 10ULL, clks->armPowerSavingClkDiv,
				     lo_maps[clks->rx1LoSelect], lo_maps[clks->rx2LoSelect],
				     lo_maps[clks->tx1LoSelect],
				     phy->tx_channels[0].dpd_init && phy->tx_channels[0].elb_en,
				     lo_maps[clks->tx2LoSelect],
				     phy->tx_channels[1].dpd_init && phy->tx_channels[1].elb_en,
				     rx_gain_type[rx_cfg[ADRV9002_CHANN_1].profile.gainTableType],
				     rx_gain_type[rx_cfg[ADRV9002_CHANN_2].profile.gainTableType],
				     rx->rxInitChannelMask, tx->txInitChannelMask,
				     duplex[sys->duplexMode], sys->fhModeOn, mcs[sys->mcsMode],
				     warm_boot[sys->warmBootEnable], ssi[phy->ssi_type]);
}

/*
 * !\FIXME
 *
 * There's a very odd issue where the tx2 power is significantly higher than
 * tx1. The reason is far from being clear but it looks somehow to be related with
 * tuning and the SSI delays. Some workarounds tested were:
 *	issuing a sync (reg 0x44) on the DDS;
 *	re-enabling the DDS core.
 * Both options had to be done after tuning but they were only half fixing the issue.
 * Meaning that TX2 power decreased to a level closer to TX1 but still around 6dbs
 * higher. Hence, what seems to really fix the issue is to read the TX SSI status
 * on the device side and with testdata set to FIXED_PATTERN. Somehow that is making
 * hdl happy. Another thing that was noted was that doing this at every calibration
 * point (after configuring the delays), on TX2, lead to more reliable tuning results
 * (more noticeable on the LTE40 profile).
 *
 * Obviuosly, this is an awful workaround and we need to understand the root cause of
 * the issue and properly fix things. Hopefully this won't one those things where
 * "we fix it later" means never!
 */
int adrv9002_tx2_fixup(const struct adrv9002_rf_phy *phy)
{
	const struct adrv9002_chan *tx = &phy->tx_channels[ADRV9002_CHANN_2].channel;
	struct  adi_adrv9001_TxSsiTestModeCfg ssi_cfg = {
		.testData = ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN,
	};
	struct adi_adrv9001_TxSsiTestModeStatus dummy;

	if (phy->chip->n_tx < ADRV9002_CHANN_MAX || phy->rx2tx2)
		return 0;

	return api_call(phy, adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect, tx->number, phy->ssi_type,
			ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA, &ssi_cfg, &dummy);
}

int adrv9002_init(struct adrv9002_rf_phy *phy, struct adi_adrv9001_Init *profile)
{
	int ret, c;
	struct adrv9002_chan *chan;

	adrv9002_cleanup(phy);
	/*
	 * Disable all the cores as it might interfere with init calibrations
	 * and mux all ports to 50ohms (when aplicable).
	 */
	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		chan = phy->channels[c];

		/* nothing else to do if there's no TX2 */
		if (chan->port == ADI_TX && chan->idx >= phy->chip->n_tx)
			break;

		adrv9002_port_enable(phy, chan, false);
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

	/* re-enable the cores and port muxes */
	for (c = 0; c < ARRAY_SIZE(phy->channels); c++) {
		chan = phy->channels[c];

		adrv9002_port_enable(phy, chan, true);
	}

	ret = adrv9002_intf_tuning(phy);
	if (ret) {
		dev_err(&phy->spi->dev, "Interface tuning failed: %d\n", ret);
		goto error;
	}

	adrv9002_fill_profile_read(phy);

	return adrv9002_tx2_fixup(phy);
error:
	/*
	 * Leave the device in a reset state in case of error. There's not much we can do if
	 * the API call fails, so we are just being verbose about it...
	 */
	api_call(phy, adi_adrv9001_HwOpen, &adrv9002_spi);
	adrv9002_cleanup(phy);

	return ret;
}

static ssize_t adrv9002_stream_bin_write(struct file *filp, struct kobject *kobj,
					 struct bin_attribute *bin_attr, char *buf, loff_t off,
					 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	if (!off)
		phy->stream_size = 0;
	memcpy(phy->stream_buf + off, buf, count);
	phy->stream_size += count;

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

	guard(mutex)(&phy->lock);

	memset(&phy->profile, 0, sizeof(phy->profile));
	ret = api_call(phy, adi_adrv9001_profileutil_Parse, &phy->profile,
		       phy->bin_attr_buf, off + count);
	if (ret)
		return ret;

	ret = adrv9002_init(phy, &phy->profile);

	return (ret < 0) ? ret : count;
}

static ssize_t adrv9002_profile_bin_read(struct file *filp, struct kobject *kobj,
					 struct bin_attribute *bin_attr, char *buf, loff_t pos,
					 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);

	guard(mutex)(&phy->lock);
	return memory_read_from_buffer(buf, count, &pos, phy->profile_buf, phy->profile_len);
}

static ssize_t adrv9002_fh_bin_table_write(struct adrv9002_rf_phy *phy, char *buf, loff_t off,
					   size_t count, int hop, int table)
{
	struct adrv9002_fh_bin_table *tbl = &phy->fh_table_bin_attr;
	char *p, *line;
	int entry = 0, ret, max_sz = ARRAY_SIZE(tbl->hop_tbl);

	/* force a one write() call as it simplifies things a lot */
	if (off) {
		dev_err(&phy->spi->dev, "Hop table must be set in one write() call\n");
		return -EINVAL;
	}

	guard(mutex)(&phy->lock);
	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(&phy->spi->dev, "Frequency hopping not enabled\n");
		return -ENOTSUPP;
	}

	if (hop && phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
		dev_err(&phy->spi->dev, "HOP2 not supported! FH mode not in dual hop.\n");
		return -ENOTSUPP;
	}

	memcpy(tbl->bin_table, buf, count);
	/* The bellow is always safe as @bin_table is bigger (by 1 byte) than the bin attribute */
	tbl->bin_table[count] = '\0';

	if (phy->fh.mode == ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP)
		max_sz /= 2;

	p = tbl->bin_table;
	while ((line = strsep(&p, "\n"))) {
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
			return -EINVAL;
		}

		if (entry > max_sz) {
			dev_err(&phy->spi->dev, "Hop:%d table:%d too big:%d\n", hop, table, entry);
			return -EINVAL;
		}

		if (lo < ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid value for lo:%llu, in table entry:%d\n",
				lo, entry);
			return -EINVAL;
		}

		tbl->hop_tbl[entry].hopFrequencyHz = lo;
		tbl->hop_tbl[entry].rx1OffsetFrequencyHz = rx10_if;
		tbl->hop_tbl[entry].rx2OffsetFrequencyHz = rx10_if;
		tbl->hop_tbl[entry].rx1GainIndex = rx1_gain;
		tbl->hop_tbl[entry].tx1Attenuation_fifthdB = tx1_atten;
		tbl->hop_tbl[entry].rx2GainIndex = rx1_gain;
		tbl->hop_tbl[entry].tx2Attenuation_fifthdB = tx2_atten;
		entry++;
	}

	dev_dbg(&phy->spi->dev, "Load hop:%d table:%d with %d entries\n", hop, table, entry);
	ret = api_call(phy, adi_adrv9001_fh_HopTable_Static_Configure,
		       phy->fh.mode, hop, table, tbl->hop_tbl, entry);

	return ret ? ret : count;
}

static char fh_table[PAGE_SIZE + 1];

static ssize_t adrv9002_dpd_tx_fh_regions_read(struct adrv9002_rf_phy *phy, char *buf,
					       loff_t off, size_t count, int c)
{
	struct adi_adrv9001_DpdFhRegions fh_regions[ADRV9002_DPD_FH_MAX_REGIONS];
	struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
	int ret, f, sz = 0;

	guard(mutex)(&phy->lock);

	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_dpd_fh_regions_Inspect, tx->channel.number,
		       fh_regions, ADRV9002_DPD_FH_MAX_REGIONS);
	if (ret)
		return ret;

	ret = adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);
	if (ret)
		return ret;

	for (f = 0; f < ARRAY_SIZE(fh_regions); f++) {
		/* We ask for all the possible entries and identify 0,0 as end of table */
		if (!fh_regions[f].startFrequency_Hz && !fh_regions[f].endFrequency_Hz)
			break;

		sz += sprintf(fh_table + sz, "%llu,%llu\n", fh_regions[f].startFrequency_Hz,
			      fh_regions[f].endFrequency_Hz);
	}

	return memory_read_from_buffer(buf, count, &off, fh_table, sz);
}

static ssize_t adrv9002_dpd_tx_fh_regions_write(struct adrv9002_rf_phy *phy, char *buf,
						loff_t off, size_t count, int c)
{
	struct adi_adrv9001_DpdFhRegions fh_regions[ADRV9002_DPD_FH_MAX_REGIONS];
	struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
	struct device *dev = &phy->spi->dev;
	u8 n_regions = 0;
	char *line, *p;
	int ret;

	/* force a one write() call as it simplifies things a lot */
	if (off) {
		dev_err(dev, "FH regions must be set in one write() call\n");
		return -EINVAL;
	}

	guard(mutex)(&phy->lock);

	if (!tx->elb_en || !tx->dpd_init || !tx->dpd_init->enable) {
		dev_err(dev, "DPD is not enabled for tx%u\n", tx->channel.number);
		return -ENOTSUPP;
	}

	if (!phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(dev, "Frequency hopping not enabled\n");
		return -ENOTSUPP;
	}

	memcpy(fh_table, buf, count);
	/* terminate it */
	fh_table[count] = '\0';

	p = fh_table;
	while ((line = strsep(&p, "\n"))) {
		 /* skip comment lines or blank lines */
		if (line[0] == '#' || !line[0])
			continue;

		if (n_regions == ADRV9002_DPD_FH_MAX_REGIONS) {
			dev_err(dev, "Max number of regions(%u) reached\n", n_regions);
			return  -E2BIG;
		}

		ret = sscanf(line, "%llu,%llu", &fh_regions[n_regions].startFrequency_Hz,
			     &fh_regions[n_regions].endFrequency_Hz);
		if (ret != 2) {
			dev_err(dev, "Failed to parse fh region, tx%u line: %s\n",
				tx->channel.idx + 1, line);
			return -EINVAL;
		}

		n_regions++;
	}

	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_dpd_fh_regions_Configure, tx->channel.number,
		       fh_regions, n_regions);
	if (ret)
		return ret;

	ret = adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);

	return ret ? ret : count;
}

static int adrv9002_dpd_coeficcients_get_line(const struct device *dev,
					      struct adi_adrv9001_DpdCoefficients *dpd_coeffs,
					      u8 *off, char *line)
{
	int ret = -EINVAL;
	char *coeff;

	while ((coeff = strsep(&line, ","))) {
		if (*off == ARRAY_SIZE(dpd_coeffs->coefficients)) {
			dev_err(dev, "Max number of coefficients(%u) reached\n", *off);
			return -E2BIG;
		}

		ret = kstrtou8(coeff, 16, &dpd_coeffs->coefficients[*off]);
		if (ret) {
			dev_err(dev, "Failed to get coeficcient: %s\n", coeff);
			return ret;
		}

		(*off)++;
	}

	return ret;
}

static char coeffs[PAGE_SIZE + 1];

static ssize_t adrv9002_dpd_tx_coeficcients_read(struct adrv9002_rf_phy *phy, char *buf,
						 loff_t off, size_t count, int c, int region)
{
	struct adi_adrv9001_DpdCoefficients dpd_coeffs = {0};
	struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
	int ret, i, sz = 0;

	dpd_coeffs.region = region;

	guard(mutex)(&phy->lock);

	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	ret = api_call(phy, adi_adrv9001_dpd_coefficients_Get, tx->channel.number, &dpd_coeffs);
	if (ret)
		return ret;

	ret = adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(dpd_coeffs.coefficients); i++) {
		/* 16 coefficients per line */
		if (!((i + 1) % 16))
			sz += sprintf(coeffs + sz, "0x%x\n", dpd_coeffs.coefficients[i]);
		else
			sz += sprintf(coeffs + sz, "0x%x,", dpd_coeffs.coefficients[i]);
	}

	return memory_read_from_buffer(buf, count, &off, coeffs, sz);
}

static ssize_t adrv9002_dpd_tx_coeficcients_write(struct adrv9002_rf_phy *phy, char *buf,
						  loff_t off, size_t count, int c, int region)
{
	struct adi_adrv9001_DpdCoefficients dpd_coeffs = {0};
	struct adrv9002_tx_chan *tx = &phy->tx_channels[c];
	struct device *dev = &phy->spi->dev;
	u8 n_coeff = 0;
	char *line, *p;
	int ret;

	/* force a one write() call as it simplifies things a lot */
	if (off) {
		dev_err(dev, "DPD coeficcients must be set in one write() call\n");
		return -EINVAL;
	}

	guard(mutex)(&phy->lock);

	if (!tx->elb_en || !tx->dpd_init || !tx->dpd_init->enable) {
		dev_err(dev, "DPD is not enabled for tx%u\n", tx->channel.number);
		return -ENOTSUPP;
	}

	/*
	 * If FH is not enabled, only region 0 is supported which basically refers to the
	 * complete spectrum. If FH is enabled, region 7 will be the one used for the "rest"
	 * of the sprectrum (the spectrum not defined in the FH regions table).
	 *
	 * \note: It is also allowed to have multiple regions with dynamic profiles having one
	 * set of coefficients per profile. Have this in mind when adding support for dymanic
	 * profiles!
	 */
	if (region > 0 && !phy->curr_profile->sysConfig.fhModeOn) {
		dev_err(dev, "Multiple regions not allowed...\n");
		return -ENOTSUPP;
	}

	memcpy(coeffs, buf, count);
	/* terminate it */
	coeffs[count] = '\0';

	p = coeffs;
	while ((line = strsep(&p, "\n")) != NULL) {
		 /* skip comment lines or blank lines */
		if (line[0] == '#' || !line[0])
			continue;

		ret = adrv9002_dpd_coeficcients_get_line(dev, &dpd_coeffs, &n_coeff, line);
		if (ret < 0)
			return ret;
	}

	/* no table?! */
	if (!n_coeff) {
		dev_err(dev, "No coeficcients found for tx%u\n", tx->channel.number + 1);
		return -EINVAL;
	}

	ret = adrv9002_channel_to_state(phy, &tx->channel, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		return ret;

	dpd_coeffs.region = region;
	ret = api_call(phy, adi_adrv9001_dpd_coefficients_Set, tx->channel.number, &dpd_coeffs);
	if (ret)
		return ret;

	ret = adrv9002_channel_to_state(phy, &tx->channel, tx->channel.cached_state, false);

	return ret ? ret : count;
}

static ssize_t adrv9002_init_cals_bin_read(struct file *filp, struct kobject *kobj,
					   struct bin_attribute *bin_attr, char *buf, loff_t off,
					   size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);
	off_t curr_off = off;
	int ret = 0, len;

	guard(mutex)(&phy->lock);
	if (!off) {
		struct adi_adrv9001_Warmboot_CalNumbers cals;

		if (phy->warm_boot.cals)
			/*
			 * Someone stop reading the coefficients in the middle of it or we have
			 * concurrent cals! As we cannot know which one is it just error out...
			 */
			return -EBUSY;

		ret = api_call(phy, adi_adrv9001_cals_InitCals_WarmBoot_UniqueEnabledCals_Get,
			       &cals, phy->init_cals.chanInitCalMask[0],
			       phy->init_cals.chanInitCalMask[1]);
		if (ret)
			return ret;

		/*
		 * These coefficients buffers are huge and adrv9002_rf_phy is already quite big.
		 * That's why we are going with this trouble to allocate + free the memory every
		 * time one wants to save the current coefficients.
		 */
		phy->warm_boot.cals = devm_kzalloc(&phy->spi->dev, cals.warmbootMemoryNumBytes,
						   GFP_KERNEL);
		if (!phy->warm_boot.cals)
			return -ENOMEM;

		ret = api_call(phy,
			       adi_adrv9001_cals_InitCals_WarmBoot_Coefficients_UniqueArray_Get,
			       phy->warm_boot.cals, phy->init_cals.chanInitCalMask[0],
			       phy->init_cals.chanInitCalMask[1]);
		if (ret)
			return ret;

		phy->warm_boot.size = cals.warmbootMemoryNumBytes;
	}

	len = memory_read_from_buffer(buf, count, &off, phy->warm_boot.cals, phy->warm_boot.size);
	if (curr_off + count >= phy->warm_boot.size && phy->warm_boot.cals) {
		/*
		 * We are done with it. Even if userspace tries to read more,
		 * @memory_read_from_buffer() will return 0 (EOF) without trying to copy into the
		 * buffer.
		 */
		dev_dbg(&phy->spi->dev, "Freeing memory(%u)...\n", phy->warm_boot.size);
		devm_kfree(&phy->spi->dev, phy->warm_boot.cals);
		phy->warm_boot.cals = NULL;
	}

	return ret ? ret : len;
}

static int adrv9002_profile_load(struct adrv9002_rf_phy *phy)
{
	int ret;
	const struct firmware *fw;
	const char *profile;
	void *buf;

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		profile = phy->chip->cmos_profile;
	else
		profile = phy->chip->lvd_profile;

	ret = request_firmware(&fw, profile, &phy->spi->dev);
	if (ret)
		return ret;

	buf = kzalloc(fw->size, GFP_KERNEL);
	if (!buf) {
		release_firmware(fw);
		return -ENOMEM;
	}

	memcpy(buf, fw->data, fw->size);
	ret = api_call(phy, adi_adrv9001_profileutil_Parse, &phy->profile, buf, fw->size);
	release_firmware(fw);
	kfree(buf);

	return ret;
}

static int adrv9002_init_cals_coeffs_name_get(struct adrv9002_rf_phy *phy)
{
	const char *init_cals;

	/* Ignore if the profile has no warmboot */
	if (!phy->profile.sysConfig.warmBootEnable)
		return 0;

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		init_cals = phy->chip->cmos_cals;
	else
		init_cals = phy->chip->lvds_cals;

	return strscpy(phy->warm_boot.coeffs_name, init_cals, sizeof(phy->warm_boot.coeffs_name));
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

#define ADRV9002_DPD_FH_REGIONS(nr)								\
static ssize_t adrv9002_dpd_tx##nr##_fh_regions_write(struct file *filp, struct kobject *kobj,	\
						      struct bin_attribute *bin_attr,		\
						      char *buf, loff_t off, size_t count)	\
{												\
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));				\
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);					\
												\
	return adrv9002_dpd_tx_fh_regions_write(phy, buf, off, count, nr);			\
}												\
												\
static ssize_t adrv9002_dpd_tx##nr##_fh_regions_read(struct file *filp, struct kobject *kobj,	\
						      struct bin_attribute *bin_attr,		\
						      char *buf, loff_t off, size_t count)	\
{												\
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));				\
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);					\
												\
	return adrv9002_dpd_tx_fh_regions_read(phy, buf, off, count, nr);			\
}												\
static BIN_ATTR(out_voltage##nr##_dpd_frequency_hopping_regions, 0644,				\
		adrv9002_dpd_tx##nr##_fh_regions_read,						\
		adrv9002_dpd_tx##nr##_fh_regions_write, PAGE_SIZE)				\

ADRV9002_DPD_FH_REGIONS(0);
ADRV9002_DPD_FH_REGIONS(1);

#define ADRV9002_DPD_COEFICCIENTS(nr, r)							\
static ssize_t adrv9002_dpd_tx##nr##_region##r##_write(struct file *filp, struct kobject *kobj,	\
						       struct bin_attribute *bin_attr,		\
						       char *buf, loff_t off, size_t count)	\
{												\
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));				\
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);					\
												\
	return adrv9002_dpd_tx_coeficcients_write(phy, buf, off, count, nr, r);			\
}												\
												\
static ssize_t adrv9002_dpd_tx##nr##_region##r##_read(struct file *filp, struct kobject *kobj,	\
						      struct bin_attribute *bin_attr,		\
						      char *buf, loff_t off, size_t count)	\
{												\
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));				\
	struct adrv9002_rf_phy *phy = iio_priv(indio_dev);					\
												\
	return adrv9002_dpd_tx_coeficcients_read(phy, buf, off, count, nr, r);			\
}												\
static BIN_ATTR(out_voltage##nr##_dpd_region##r##_coefficients, 0644,				\
		adrv9002_dpd_tx##nr##_region##r##_read,						\
		adrv9002_dpd_tx##nr##_region##r##_write, PAGE_SIZE)				\

ADRV9002_DPD_COEFICCIENTS(0, 0);
ADRV9002_DPD_COEFICCIENTS(0, 1);
ADRV9002_DPD_COEFICCIENTS(0, 2);
ADRV9002_DPD_COEFICCIENTS(0, 3);
ADRV9002_DPD_COEFICCIENTS(0, 4);
ADRV9002_DPD_COEFICCIENTS(0, 5);
ADRV9002_DPD_COEFICCIENTS(0, 6);
ADRV9002_DPD_COEFICCIENTS(0, 7);
ADRV9002_DPD_COEFICCIENTS(1, 0);
ADRV9002_DPD_COEFICCIENTS(1, 1);
ADRV9002_DPD_COEFICCIENTS(1, 2);
ADRV9002_DPD_COEFICCIENTS(1, 3);
ADRV9002_DPD_COEFICCIENTS(1, 4);
ADRV9002_DPD_COEFICCIENTS(1, 5);
ADRV9002_DPD_COEFICCIENTS(1, 6);
ADRV9002_DPD_COEFICCIENTS(1, 7);

static BIN_ATTR(stream_config, 0222, NULL, adrv9002_stream_bin_write, ADRV9002_STREAM_BINARY_SZ);
static BIN_ATTR(profile_config, 0644, adrv9002_profile_bin_read, adrv9002_profile_bin_write,
		ADRV9002_PROFILE_MAX_SZ);
static BIN_ATTR(warmboot_coefficients, 0400, adrv9002_init_cals_bin_read, NULL,
		ADRV9002_INIT_CALS_COEFFS_MAX);

static int adrv9002_iio_channels_get(struct adrv9002_rf_phy *phy)
{
	/*
	 * The fist channels are the LO's. Hence we always have ADRV9002_CHANN_MAX
	 * for RX's and 'n_tx' for TX's. Of course this assumes not channel will
	 * be added in between but that should be easy enough to maintain!
	 */
	unsigned int off = ADRV9002_CHANN_MAX + phy->chip->n_tx;
	unsigned int tx;

	phy->iio_chan = devm_kmemdup(&phy->spi->dev, phy->chip->channels,
				     sizeof(*phy->chip->channels) * phy->chip->num_channels,
				     GFP_KERNEL);
	if (!phy->iio_chan)
		return -ENOMEM;

	for (tx = 0; tx < phy->chip->n_tx; tx++) {
		struct adrv9002_chan *c = &phy->tx_channels[tx].channel;

		if (!c->mux_ctl || !c->mux_ctl_2)
			continue;

		/* both muxes are available so we can select between TX's */
		phy->iio_chan[off + tx].ext_info = adrv9002_phy_tx_mux_ext_info;
	}

	return 0;
}

static const char * const clk_names[NUM_ADRV9002_CLKS] = {
	[RX1_SAMPL_CLK] = "-rx1_sampl_clk",
	[RX2_SAMPL_CLK] = "-rx2_sampl_clk",
	[TX1_SAMPL_CLK] = "-tx1_sampl_clk",
	[TX2_SAMPL_CLK] = "-tx2_sampl_clk",
	[TDD1_INTF_CLK] = "-tdd1_intf_clk",
	[TDD2_INTF_CLK] = "-tdd2_intf_clk"
};

static const struct bin_attribute *hop_attrs[] = {
	&bin_attr_frequency_hopping_hop1_table_a,
	&bin_attr_frequency_hopping_hop1_table_b,
	&bin_attr_frequency_hopping_hop2_table_a,
	&bin_attr_frequency_hopping_hop2_table_b
};

static const struct bin_attribute *dpd_fh_regions[] = {
	&bin_attr_out_voltage0_dpd_frequency_hopping_regions,
	&bin_attr_out_voltage1_dpd_frequency_hopping_regions,
};

static const struct bin_attribute *dpd_coeffs[ADRV9002_CHANN_MAX][ADRV9002_DPD_MAX_REGIONS] = {
	[ADRV9002_CHANN_1] = {
		&bin_attr_out_voltage0_dpd_region0_coefficients,
		&bin_attr_out_voltage0_dpd_region1_coefficients,
		&bin_attr_out_voltage0_dpd_region2_coefficients,
		&bin_attr_out_voltage0_dpd_region3_coefficients,
		&bin_attr_out_voltage0_dpd_region4_coefficients,
		&bin_attr_out_voltage0_dpd_region5_coefficients,
		&bin_attr_out_voltage0_dpd_region6_coefficients,
		&bin_attr_out_voltage0_dpd_region7_coefficients
	},
	[ADRV9002_CHANN_2] = {
		&bin_attr_out_voltage1_dpd_region0_coefficients,
		&bin_attr_out_voltage1_dpd_region1_coefficients,
		&bin_attr_out_voltage1_dpd_region2_coefficients,
		&bin_attr_out_voltage1_dpd_region3_coefficients,
		&bin_attr_out_voltage1_dpd_region4_coefficients,
		&bin_attr_out_voltage1_dpd_region5_coefficients,
		&bin_attr_out_voltage1_dpd_region6_coefficients,
		&bin_attr_out_voltage1_dpd_region7_coefficients
	}
};

int adrv9002_post_init(struct adrv9002_rf_phy *phy)
{
	struct adi_common_ApiVersion api_version;
	struct adi_adrv9001_ArmVersion arm_version;
	struct adi_adrv9001_SiliconVersion silicon_version;
	struct adi_adrv9001_StreamVersion stream_version;
	int ret, c, r;
	struct spi_device *spi = phy->spi;
	struct iio_dev *indio_dev = phy->indio_dev;

	/* register channels clocks */
	for (c = 0; c < ADRV9002_CHANN_MAX; c++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[c];
		struct adrv9002_chan *tx = &phy->tx_channels[c].channel;

		rx->channel.clk = adrv9002_clk_register(phy, clk_names[c],
							CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
							c);
		if (IS_ERR(rx->channel.clk))
			return PTR_ERR(rx->channel.clk);

		/*
		 * We do not support more TXs so break now. Note that in rx2tx2 mode, we
		 * set the clk pointer of a non existing channel but since it won't ever
		 * be used it's not a problem so let's keep the code simple in here.
		 */
		if (c >= phy->chip->n_tx)
			break;

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

	ret = adrv9002_profile_load(phy);
	if (ret)
		return ret;

	/*
	 * Validate the output devclk for the default profile. Done once in here so that we don't
	 * have to do it everytime in adrv9002_validate_device_clk() even if the device clock did
	 * not changed between profiles.
	 */
	adrv9002_validate_device_clkout(phy, phy->profile.clocks.deviceClock_kHz * KILO);

	ret = adrv9002_init_cals_coeffs_name_get(phy);
	if (ret < 0)
		return ret;

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

	/* it might be possible to mux between TX's and so iio_chan_spec can change */
	ret = adrv9002_iio_channels_get(phy);
	if (ret)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = phy->chip->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adrv9002_phy_info;
	indio_dev->channels = phy->iio_chan;
	indio_dev->num_channels = phy->chip->num_channels;

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

	phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev, ADRV9002_PROFILE_MAX_SZ, GFP_KERNEL);
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

	for (c = 0; c < phy->chip->n_tx; c++) {
		/* do not expose the interface if dpd is not available */
		if (!phy->tx_channels[c].dpd_init)
			continue;

		ret = device_create_bin_file(&indio_dev->dev, dpd_fh_regions[c]);
		if (ret)
			return ret;

		for (r = 0; r < ADRV9002_DPD_MAX_REGIONS; r++) {
			ret = device_create_bin_file(&indio_dev->dev, dpd_coeffs[c][r]);
			if (ret)
				return ret;
		}
	}

	ret = device_create_bin_file(&indio_dev->dev, &bin_attr_warmboot_coefficients);
	if (ret)
		return ret;

	api_call(phy, adi_adrv9001_ApiVersion_Get, &api_version);
	api_call(phy, adi_adrv9001_arm_Version, &arm_version);
	api_call(phy, adi_adrv9001_SiliconVersion_Get, &silicon_version);
	api_call(phy, adi_adrv9001_Stream_Version, &stream_version);

	dev_info(&spi->dev,
		 "%s Rev %d.%d, Firmware %u.%u.%u,  Stream %u.%u.%u.%u,  API version: %u.%u.%u successfully initialized",
		 indio_dev->name, silicon_version.major, silicon_version.minor,
		 arm_version.majorVer, arm_version.minorVer, arm_version.maintVer,
		 stream_version.majorVer, stream_version.minorVer,
		 stream_version.maintVer, stream_version.buildVer, api_version.major,
		 api_version.minor, api_version.patch);

	adrv9002_debugfs_create(phy, iio_get_debugfs_dentry(indio_dev));

	return 0;
}

static const struct adrv9002_chip_info adrv9002_info[] = {
	[ID_ADRV9002] = {
		.channels = adrv9002_phy_chan,
		.num_channels = ARRAY_SIZE(adrv9002_phy_chan),
		.cmos_profile = "Navassa_CMOS_profile.json",
		.lvd_profile = "Navassa_LVDS_profile.json",
		.cmos_cals = "Navassa_CMOS_init_cals.bin",
		.lvds_cals = "Navassa_LVDS_init_cals.bin",
		.name = "adrv9002-phy",
		.n_tx = ADRV9002_CHANN_MAX,
	},
	[ID_ADRV9002_RX2TX2] = {
		.channels = adrv9002_phy_chan,
		.num_channels = ARRAY_SIZE(adrv9002_phy_chan),
		.cmos_profile = "Navassa_CMOS_profile.json",
		.lvd_profile = "Navassa_LVDS_profile.json",
		.cmos_cals = "Navassa_CMOS_init_cals.bin",
		.lvds_cals = "Navassa_LVDS_init_cals.bin",
		.name = "adrv9002-phy",
		.n_tx = ADRV9002_CHANN_MAX,
		.rx2tx2 = true,
	},
	[ID_ADRV9003] = {
		.channels = adrv9003_phy_chan,
		.num_channels = ARRAY_SIZE(adrv9003_phy_chan),
		.cmos_profile = "Navassa_CMOS_profile_adrv9003.json",
		.lvd_profile = "Navassa_LVDS_profile_adrv9003.json",
		.cmos_cals = "Navassa_CMOS_init_cals_adrv9003.bin",
		.lvds_cals = "Navassa_LVDS_init_cals_adrv9003.bin",
		.name = "adrv9003-phy",
		.n_tx = 1,
	},
	[ID_ADRV9003_RX2TX2] = {
		.channels = adrv9003_phy_chan,
		.num_channels = ARRAY_SIZE(adrv9003_phy_chan),
		.cmos_profile = "Navassa_CMOS_profile_adrv9003.json",
		.lvd_profile = "Navassa_LVDS_profile_adrv9003.json",
		.cmos_cals = "Navassa_CMOS_init_cals_adrv9003.bin",
		.lvds_cals = "Navassa_LVDS_init_cals_adrv9003.bin",
		.name = "adrv9003-phy",
		.n_tx = 1,
		.rx2tx2 = true,
	},
};

static int adrv9002_get_external_los(struct adrv9002_rf_phy *phy)
{
	static const char *const ext_los[] = { "ext_lo1", "ext_lo2" };
	struct device_node *np = phy->spi->dev.of_node;
	struct device *dev = &phy->spi->dev;
	int ret, lo;

	for (lo = 0; lo < ARRAY_SIZE(phy->ext_los); lo++) {
		phy->ext_los[lo].clk = devm_clk_get_optional_enabled(dev, ext_los[lo]);
		if (IS_ERR(phy->ext_los[lo].clk))
			return PTR_ERR(phy->ext_los[lo].clk);
		if (!phy->ext_los[lo].clk)
			continue;

		ret = of_clk_get_scale(np, ext_los[lo], &phy->ext_los[lo].scale);
		if (ret) {
			phy->ext_los[lo].scale.div = 10;
			phy->ext_los[lo].scale.mult = 1;
		}
	}

	return 0;
}

static int adrv9002_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9002_rf_phy *phy;
	int ret, c;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;

	phy->chip = of_device_get_match_data(&spi->dev);
	if (!phy->chip)
		phy->chip = (const struct adrv9002_chip_info *)spi_get_device_id(spi)->driver_data;
	if (!phy->chip)
		return -EINVAL;

	/* in the future we might want to get rid of 'phy->rx2tx2' and just use chip_info  */
	if (phy->chip->rx2tx2)
		phy->rx2tx2 = true;

	mutex_init(&phy->lock);
	phy->adrv9001 = &phy->adrv9001_device;
	phy->hal.spi = spi;
	phy->adrv9001->common.devHalInfo = &phy->hal;

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

	phy->dev_clk = devm_clk_get_enabled(&spi->dev, "adrv9002_ext_refclk");
	if (IS_ERR(phy->dev_clk))
		return PTR_ERR(phy->dev_clk);

	phy->hal.reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(phy->hal.reset_gpio))
		return PTR_ERR(phy->hal.reset_gpio);

	ret = adrv9002_get_external_los(phy);
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

static const struct of_device_id adrv9002_of_match[] = {
	{.compatible = "adi,adrv9002", .data = &adrv9002_info[ID_ADRV9002]},
	{.compatible = "adi,adrv9002-rx2tx2", .data = &adrv9002_info[ID_ADRV9002_RX2TX2]},
	{.compatible = "adi,adrv9003", .data = &adrv9002_info[ID_ADRV9003]},
	{.compatible = "adi,adrv9003-rx2tx2", .data = &adrv9002_info[ID_ADRV9003_RX2TX2]},
	{}
};
MODULE_DEVICE_TABLE(of, adrv9002_of_match);

static const struct spi_device_id adrv9002_ids[] = {
	{"adrv9002", (kernel_ulong_t)&adrv9002_info[ID_ADRV9002]},
	{"adrv9002-rx2tx2", (kernel_ulong_t)&adrv9002_info[ID_ADRV9002_RX2TX2]},
	{"adrv9003", (kernel_ulong_t)&adrv9002_info[ID_ADRV9003]},
	{"adrv9003-rx2tx2", (kernel_ulong_t)&adrv9002_info[ID_ADRV9003_RX2TX2]},
	{}
};
MODULE_DEVICE_TABLE(spi, adrv9002_ids);

static struct spi_driver adrv9002_driver = {
	.driver = {
		.name	= "adrv9002",
		.of_match_table = adrv9002_of_match,
	},
	.probe		= adrv9002_probe,
	.id_table	= adrv9002_ids,
};
module_spi_driver(adrv9002_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9002 ADC");
MODULE_LICENSE("GPL v2");
