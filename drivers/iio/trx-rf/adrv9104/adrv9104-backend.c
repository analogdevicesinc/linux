// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Analog Devices Inc.
 */
#include "linux/dev_printk.h"
#include "linux/stddef.h"
#include <linux/bitfield.h>
#include <linux/bitmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/backend.h>

#include "adrv9104.h"
#include "adrv9104-backend.h"

#include "device_profile_ssi_t.h"

#define ADRV9104_BACK_REG_CONFIG		0xc
#define   ADRV9104_BACK_IS_CMOS_MSK		BIT(7)
#define   ADRV9104_BACK_TX_REF_CLK_MSK		GENMASK(11, 10)
#define ADRV9104_BACK_RX_REG_CTRL		0x44
#define   ADRV9104_BACK_R1_MODE_MSK		BIT(2)
#define ADRV9104_BACK_TX_REG_CTRL2		0x48
#define   ADRV9104_BACK_SSI_N_LANES_MSK		GENMASK(12, 8)
#define   ADRV9104_BACK_SSI_N_DDR_MSK		BIT(16)
#define ADRV9104_BACK_TX_REG_RATE		0x4c
#define   ADRV9104_BACK_SSI_RATE_MSK		GENMASK(7, 0)
#define ADRV9104_BACK_REG_STATUS1		0x58

enum {
	ADRV9104_TX_REF_CLK_OWN,
	ADRV9104_TX_REF_CLK_RX1,
	ADRV9104_TX_REF_CLK_RX2,
	ADRV9104_TX_REF_CLK_MAX
};

static const unsigned long adrv9104_valid_scan_masks[ADRV9104_CHAN_MAX] = {
	GENMASK(ADRV9104_RX1_Q_SCAN, ADRV9104_RX1_I_SCAN),
	GENMASK(ADRV9104_RX2_Q_SCAN, ADRV9104_RX2_I_SCAN),
	GENMASK(ADRV9104_TX_Q_SCAN, ADRV9104_TX_I_SCAN),
};

enum {
	ADRV9104_TX_DDS_I,
	ADRV9104_TX_DDS_Q,
};

static int adrv9104_backend_channel_set(struct iio_dev *indio_dev, const struct adrv9104_chan *chan,
					const unsigned long buf_mask, bool enable)
{
	unsigned int bit;
	int ret;

	iio_for_each_active_channel(indio_dev, bit) {
		if (!test_bit(bit, &buf_mask))
			continue;

		/*
		 * Even bits are for I channels which is index 0. Odd bits are for Q channels which
		 * is index 1. Take care that if we do add support for rx2tx this will not hold
		 * true!
		 */
		dev_info(indio_dev->dev.parent, "Handle chan: %u, port: %u, scan: %u, enable:%u\n",
			 chan->idx, chan->port, bit, enable);

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

static int adrv9104_backend_set_ssi_type(struct adrv9104_rf_phy *phy, struct adrv9104_chan *chan)
{
	unsigned int val, reg, divider;
	ssiNumLane_e n_lanes;
	bool ddr_en;
	int ret;

	if (chan->port == ADI_TX) {
		n_lanes = phy->profile.txConfig[chan->idx].txSsiConfig.numLaneSel;
		ddr_en = phy->profile.txConfig[chan->idx].txSsiConfig.ddrEn;
		reg = ADRV9104_BACK_TX_REG_CTRL2;
	} else {
		n_lanes = phy->profile.rxConfig[chan->idx].rxSsiConfig.numLaneSel;
		ddr_en = phy->profile.rxConfig[chan->idx].rxSsiConfig.ddrEn;
		reg = ADRV9104_BACK_RX_REG_CTRL;
	}

	ret = iio_backend_reg_read(chan->back, ADRV9104_BACK_REG_STATUS1, &val);
	if (ret)
		return ret;

	val &= ~(ADRV9104_BACK_SSI_N_LANES_MSK | ADRV9104_BACK_SSI_N_DDR_MSK);

	switch (n_lanes) {
	case SSI_1_LANE:
		val |= FIELD_PREP(ADRV9104_BACK_SSI_N_LANES_MSK, 1);
		break;
	case SSI_2_LANE:
		if (phy->ssi_type == SSI_TYPE_CMOS)
			return -EINVAL;
		val |= FIELD_PREP(ADRV9104_BACK_SSI_N_LANES_MSK, 2);
		break;
	case SSI_4_LANE:
		if (phy->ssi_type != SSI_TYPE_CMOS)
			return -EINVAL;
		val |= FIELD_PREP(ADRV9104_BACK_SSI_N_LANES_MSK, 4);
		break;
	default:
		return -EINVAL;
	}

	if (phy->ssi_type == SSI_TYPE_CMOS)
		val |= FIELD_PREP(ADRV9104_BACK_SSI_N_DDR_MSK, ddr_en);

	ret = iio_backend_reg_write(chan->back, reg, val);
	if (ret)
		return ret;

	if (chan->port == ADI_RX)
		return 0;

	ret = iio_backend_reg_read(chan->back, ADRV9104_BACK_REG_STATUS1, &divider);
	if (ret)
		return ret;

	/* in LVDS, data type is always DDR */
	if (phy->ssi_type == SSI_TYPE_LVDS)
		ddr_en = true;

	val = 32 / ((1 << n_lanes) * (1 + ddr_en) * divider) - 1;

	return iio_backend_reg_write(chan->back, ADRV9104_BACK_TX_REG_RATE,
				     FIELD_PREP(ADRV9104_BACK_SSI_RATE_MSK, val));
}

int adrv9104_backend_get(struct adrv9104_rf_phy *phy, struct adrv9104_chan *chan,
			 struct iio_dev *indio_dev, struct iio_chan_spec *dds_base_chan)
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
		/* For now just go with split mode */
		return iio_backend_reg_write(chan->back, ADRV9104_BACK_RX_REG_CTRL,
					     FIELD_PREP(ADRV9104_BACK_R1_MODE_MSK, 1));

	/* Position for ADRV9104_TX_DDS_CHAN() channels */
	ret = iio_backend_extend_chan_spec(chan->back, dds_base_chan);
	if (ret)
		return ret;

	return iio_backend_extend_chan_spec(chan->back, &dds_base_chan[1]);
}

static const char * const adrv9104_tx_clk[ADRV9104_TX_REF_CLK_MAX] = {
	"OWN REF", "RX1 REF", "RX2 REF"
};

int adrv9104_backend_get_tx_ref_clock(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_tx *tx = &phy->tx_channel;
	unsigned int val;
	int ret;

	/* We can't have backends with different SSI interfaces. So, let's just read for TX. */
	ret = iio_backend_reg_read(tx->channel.back, ADRV9104_BACK_REG_CONFIG, &val);
	if (ret)
		return ret;

	tx->tx_ref_clock = FIELD_GET(ADRV9104_BACK_TX_REF_CLK_MSK, val);
	if (tx->tx_ref_clock > ADRV9104_TX_REF_CLK_RX2)
		return dev_err_probe(phy->dev, -EINVAL, "Got Invalid TX ref clock(%u) from HW\n",
				     tx->tx_ref_clock);

	dev_dbg(phy->dev, "Tx SSI clk driven by %s\n", adrv9104_tx_clk[tx->tx_ref_clock]);
	return 0;
}

int adrv9104_backend_get_ssi_type(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_rx *rx = &phy->rx_channels[0];
	enum iio_backend_interface_type type;
	int ret;

	/* the ssi type is the same on all ports so just get it for one */
	ret = iio_backend_interface_type_get(rx->channel.back, &type);
	if (ret)
		return ret;

	phy->ssi_type = type == IIO_BACKEND_INTERFACE_SERIAL_CMOS ? SSI_TYPE_CMOS : SSI_TYPE_LVDS;

	return 0;
}

struct iio_backend *adrv9104_backend_get_from_chan(struct iio_dev *indio_dev,
						   uintptr_t private,
						   const struct iio_chan_spec *chan)
{
	struct adrv9104_rf_phy *phy = iio_priv(indio_dev);

	/* Only get's called for the TX DDS channel extension */
	return phy->tx_channel.channel.back;
}

int adrv9104_backend_cfg(struct adrv9104_rf_phy *phy, struct adrv9104_chan *chan)
{
	int ret;

	if (chan->port == ADI_TX) {
		ret = iio_backend_set_sampling_freq(chan->back, ADRV9104_TX_DDS_I, chan->rate);
		if (ret)
			return ret;

		ret = iio_backend_set_sampling_freq(chan->back, ADRV9104_TX_DDS_Q, chan->rate);
		if (ret)
			return ret;
	}

	return adrv9104_backend_set_ssi_type(phy, chan);
}
