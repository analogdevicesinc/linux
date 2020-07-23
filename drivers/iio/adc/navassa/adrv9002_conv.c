// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "adrv9002.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_ssi_types.h"

#if IS_ENABLED(CONFIG_CF_AXI_ADC)
#include "../cf_axi_adc.h"

#define ADI_RX2_REG_OFF			0x1000
#define ADI_TX1_REG_OFF			0x2000
#define ADI_TX2_REG_OFF			0x4000
#define ADI_TX_REG_RATE			0x4c
#define ADI_TX_REG_CTRL_2		0x48
#define ADI_TX_REG_CHAN_CTRL_7(c)	(0x0418 + (c) * 0x40)
#define ADI_TX_REG_CTRL_1		0x44
#define R1_MODE				BIT(2)
#define TX_R1_MODE				BIT(5)

#define AIM_AXI_REG(off, addr)		((off) + (addr))
#define	NUM_LANES_MASK			GENMASK(12, 8)
#define NUM_LANES(x)			FIELD_PREP(NUM_LANES_MASK, x)
#define SDR_DDR_MASK			BIT(16)
#define SDR_DDR(x)			FIELD_PREP(SDR_DDR_MASK, x)
#define TX_ONLY_MASK			BIT(10)
#define TX_ONLY(x)			FIELD_GET(TX_ONLY_MASK, x)

#define IS_CMOS(cfg)			((cfg) & (ADI_CMOS_OR_LVDS_N))

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .modified = 1,						\
	  .channel = _chan,						\
	  .channel2 = _mod,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	  .scan_index = _si,						\
	  .scan_type = {						\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 0,						\
	  },								\
	}

static const unsigned long adrv9002_rx2tx2_available_scan_masks[] = {
	0x01, 0x02, 0x03, 0x04, 0x08, 0x0C, 0x0F,
	0x00
};

static const unsigned long adrv9002_available_scan_masks[] = {
	0x01, 0x02, 0x03, 0x00
};

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_ADRV9002_RX2TX2] = {
		.name = "ADRV9002",
		.max_rate = 245760000,
		.max_testmode = 0,
		.num_channels = 4,
		.scan_masks = adrv9002_rx2tx2_available_scan_masks,
		.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
		.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
		.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
		.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
	},
	[ID_ADRV9002] = {
		.name = "ADRV9002",
		.max_rate = 245760000,
		.max_testmode = 0,
		.num_channels = 2,
		.scan_masks = adrv9002_available_scan_masks,
		.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
		.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),

	},
};

static int adrv9002_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adrv9002_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val,
			      int val2,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -ENODEV;
	default:
		return -EINVAL;
	}
}

static int adrv9002_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval,
			       u32 *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	if (!readval)
		axiadc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(st, reg & 0xFFFF);

	return 0;
}

int adrv9002_axi_interface_set(struct adrv9002_rf_phy *phy, const u8 n_lanes,
			       const bool cmos_ddr, const int channel, const bool tx)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	u32 reg_ctrl, reg_value = 0, off, divider;
	u8 rate;

	if (tx) {
		off = channel ? ADI_TX2_REG_OFF : ADI_TX1_REG_OFF;
		reg_ctrl = ADI_TX_REG_CTRL_2;
	} else {
		off = channel ? ADI_RX2_REG_OFF : 0;
		reg_ctrl = ADI_REG_CNTRL;
	}

	reg_value = axiadc_read(st, AIM_AXI_REG(off, reg_ctrl));
	reg_value &= ~(NUM_LANES_MASK | SDR_DDR_MASK);

	switch (n_lanes) {
	case ADI_ADRV9001_SSI_1_LANE:
		reg_value |= NUM_LANES(1);
		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
			reg_value |= SDR_DDR(!cmos_ddr);
		break;
	case ADI_ADRV9001_SSI_2_LANE:
		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
			return -EINVAL;

		reg_value |= NUM_LANES(2);
		break;
	case ADI_ADRV9001_SSI_4_LANE:
		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
			return -EINVAL;

		reg_value |= NUM_LANES(4);
		reg_value |= SDR_DDR(!cmos_ddr);
		break;
	default:
		return -EINVAL;
	}

	axiadc_write(st, AIM_AXI_REG(off, reg_ctrl), reg_value);
	if (tx) {
		u32 ddr = cmos_ddr;

		divider = axiadc_read(st, AIM_AXI_REG(off, ADI_REG_CLK_RATIO));
		/* in LVDS, data type is always DDR */
		if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_LVDS)
			ddr = 1;
		rate = 32 / ((1 << n_lanes) * (1 + ddr) * divider) - 1;
		axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_RATE), rate);
	}

	return 0;
}

static int adrv9002_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct adrv9002_rf_phy *phy = conv->phy;
	u32 num_chan, axi_config = 0;
	int i, ret;

	num_chan = conv->chip_info->num_channels;

	conv->indio_dev = indio_dev;

	if (!phy->rx2tx2) {
		/* set R1_MODE to 1 rf channel in all channels */
		axiadc_write(st, ADI_REG_CNTRL, R1_MODE);
		axiadc_write(st, AIM_AXI_REG(ADI_RX2_REG_OFF, ADI_REG_CNTRL), R1_MODE);
		axiadc_write(st, AIM_AXI_REG(ADI_TX1_REG_OFF, ADI_TX_REG_CTRL_2), TX_R1_MODE);
		axiadc_write(st, AIM_AXI_REG(ADI_TX2_REG_OFF, ADI_TX_REG_CTRL_2), TX_R1_MODE);
	}

	for (i = 0; i < num_chan; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(i),
			     ADI_DCFILT_OFFSET(0));
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_ENABLE | ADI_IQCOR_ENB);
	}

	axi_config = axiadc_read(st, ADI_REG_CONFIG);
	if (!IS_CMOS(axi_config))
		/*
		 * FIXME: Update the current ssi type to LVDS (it will load the LVDS default
		 * profile). This was set during adrv9002 probe() as we might need some profile
		 * clock information when parsing the DT. This is ok since both default profiles
		 * have the same clock settings but, obviously, this ping pong on the profile
		 * is not nice. Furthermore this whole mechanism of doing the device setup() in this
		 * post_setup() hook is ugly. We need to find a mechanism to find out the AXI
		 * interface type when probing the transceiver...
		 */
		phy->ssi_type = ADI_ADRV9001_SSI_TYPE_LVDS;

	/*
	 * Get tx core config to check if we support tx only profiles. 1 means that it's not
	 * supported...
	 */
	axi_config = axiadc_read(st, AIM_AXI_REG(ADI_TX1_REG_OFF, ADI_REG_CONFIG));
	phy->tx_only = !TX_ONLY(axi_config);

	ret = adrv9002_post_init(phy);
	if (ret)
		return ret;

	/* get adc rate now */
	conv->clk = phy->clks[RX1_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	return 0;
}

#ifdef DEBUG
void adrv9002_axi_digital_tune_verbose(struct adrv9002_rf_phy *phy, u8 field[][8], const bool tx,
				       const int channel)
{
	int i, j;
	char c;
	struct adrv9002_chan *ch;

	if (tx)
		ch = &phy->tx_channels[channel].channel;
	else
		ch = &phy->tx_channels[channel].channel;

	pr_info("SAMPL CLK: %lu tuning: %s%d\n",
	        clk_get_rate(ch->clk), tx ? "TX" : "RX",
		channel ? 2 : 1);
	pr_info("  ");
	for (i = 0; i < 8; i++)
		pr_cont("%x%s", i, i == 7 ? "" : ":");
	pr_cont("\n");

	for (i = 0; i < 8; i++) {
		pr_info("%x:", i);
		for (j = 0; j < 8; j++) {
			if (field[i][j])
			    c = '#';
			else
			    c = 'o';
			pr_cont("%c ", c);
		}
		pr_cont("\n");
	}
}
#else
void adrv9002_axi_digital_tune_verbose(struct adrv9002_rf_phy *phy, u8 field[][8], const bool tx,
				       const int channel)
{
}
#endif

static int adrv9002_axi_find_point(const u8 *field, const u8 sz, u8 *data_start)
{
	int i = sz;
	int cnt = 0, start = -1, max_cnt = 0;

	for (i = 0; i < sz; i++) {
		if (!field[i]) {
			if (start == -1)
				start = i;

			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				*data_start = start;
			}

			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		*data_start = start;
	}

	if (!max_cnt)
		return -EIO;

	return max_cnt;
}

static int adrv9002_axi_pn_check(const struct axiadc_converter *conv, const int off,
				 const int n_chan)
{
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int chan;
	struct adrv9002_rf_phy *phy = conv->phy;
	u32 reg;

	/* reset result */
	for (chan = 0; chan < n_chan; chan++)
		axiadc_write(st, AIM_AXI_REG(off, ADI_REG_CHAN_STATUS(chan)),
			     ADI_PN_ERR | ADI_PN_OOS);

	usleep_range(5000, 5005);

	/* check for errors in any channel */
	for (chan = 0; chan < n_chan; chan++) {
		reg = axiadc_read(st, AIM_AXI_REG(off, ADI_REG_CHAN_STATUS(chan)));
		if (reg) {
			dev_dbg(&phy->spi->dev, "pn error in c:%d, reg: %02X\n", chan, reg);
			return 1;
		}
	}

	return 0;
}

static void adrv9002_axi_tx_test_pattern_set(const struct axiadc_converter *conv, const int off,
					     const int n_chan, u32 *ctrl_7)
{
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct adrv9002_rf_phy *phy = conv->phy;
	int c, sel;

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		/* RAMP nibble */
		sel = 10;
	else
		/* pn7 */
		sel = 6;

	for (c = 0; c < n_chan; c++) {
		ctrl_7[c] = axiadc_read(st, AIM_AXI_REG(off, ADI_TX_REG_CHAN_CTRL_7(c)));
		axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_CHAN_CTRL_7(c)), sel);
		axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_CTRL_1), 1);
	}
}

static void adrv9002_axi_tx_test_pattern_restore(const struct axiadc_converter *conv, const int off,
						 const int n_chan, const u32 *saved_ctrl_7)
{
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int c;

	for (c = 0; c < n_chan; c++)
		axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_CHAN_CTRL_7(c)),
			     saved_ctrl_7[c]);
}

static void adrv9002_axi_rx_test_pattern_pn_sel(const struct axiadc_converter *conv, const int off,
						const int n_chan)
{
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct adrv9002_rf_phy *phy = conv->phy;
	int c;
	enum adc_pn_sel sel;

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS)
		sel = ADC_PN_RAMP_NIBBLE;
	else
		sel = ADC_PN15;

	for (c = 0; c < n_chan; c++)
		/*
		 * We are deliberately overwriting the DATA_SEL bits to DMA. If it's set to loopback
		 * (if some user was playing with near end loopback before updating the profile or
		 * reinitialize the device), TX tuning will fail...
		 */
		axiadc_write(st, AIM_AXI_REG(off, ADI_REG_CHAN_CNTRL_3(c)), ADI_ADC_PN_SEL(sel));
}

static void adrv9002_axi_get_channel_range(struct axiadc_converter *conv, bool tx, int *end)
{
	struct adrv9002_rf_phy *phy = conv->phy;
	/*
	 * The point here is that we only want to generate and check test patterns for enabled
	 * channels. If in !rx2tx2 we only get here if the channel is enabled so just use
	 * all the @conv channels for the test. In rx2tx2 mode, we will run the test
	 * at the same time for both channels if both are enabled. However, if RX2/TX2 is
	 * disabled we do not want to check for that so that we tweak @end to only go over
	 * the first channel (1 phy channel == 2 hdl channels). RX2/TX2 start at index 2
	 * in the channels array, so we use @tx to get the right one...
	 */
	if (phy->rx2tx2 && !phy->channels[tx + 2]->enabled)
		*end = 2;
	else
		*end = conv->chip_info->num_channels;
}

int adrv9002_axi_intf_tune(struct adrv9002_rf_phy *phy, const bool tx, const int chann,
			   u8 *clk_delay, u8 *data_delay)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	int ret, cnt, max_cnt = 0, off;
	u8 field[8][8] = {0};
	u8 clk, data;
	u32 saved_ctrl_7[4];
	int n_chan;

	adrv9002_axi_get_channel_range(conv, tx, &n_chan);
	if (tx) {
		off = chann ? ADI_TX2_REG_OFF : ADI_TX1_REG_OFF;
		/* generate test pattern for tx test  */
		adrv9002_axi_tx_test_pattern_set(conv, off, n_chan, saved_ctrl_7);
	} else {
		off = chann ? ADI_RX2_REG_OFF : 0;
		adrv9002_axi_rx_test_pattern_pn_sel(conv, off, n_chan);
		/* start test */
		ret = adrv9002_intf_test_cfg(phy, chann, tx, false);
		if (ret)
			return ret;
	}

	for (clk = 0; clk < ARRAY_SIZE(field); clk++) {
		for (data = 0; data < sizeof(*field); data++) {
			ret = adrv9002_intf_change_delay(phy, chann, clk, data, tx);
			if (ret < 0)
				return ret;

			if (tx) {
				/*
				 * we need to restart the tx test for every iteration since it's
				 * the only way to reset the counters.
				 */
				ret = adrv9002_intf_test_cfg(phy, chann, tx, false);
				if (ret)
					return ret;
			}
			/* check result */
			if (!tx)
				ret = adrv9002_axi_pn_check(conv, off, n_chan);
			else
				ret = adrv9002_check_tx_test_pattern(phy, chann);

			field[clk][data] |= ret;
		}
	}

	adrv9002_axi_digital_tune_verbose(phy, field, tx, chann);

	/* stop test */
	ret = adrv9002_intf_test_cfg(phy, chann, tx, true);
	if (ret)
		return ret;

	/* stop tx pattern */
	if (tx)
		adrv9002_axi_tx_test_pattern_restore(conv, off, n_chan, saved_ctrl_7);

	for (clk = 0; clk < ARRAY_SIZE(field); clk++) {
		cnt = adrv9002_axi_find_point(&field[clk][0], sizeof(*field), &data);
		if (cnt < 0)
			continue;

		if (cnt > max_cnt) {
			max_cnt = cnt;
			*clk_delay = clk;
			*data_delay = data + max_cnt / 2;
		}
	}

	return max_cnt ? 0 : -EIO;
}

void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan, const bool tx,
				   const bool en)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int off = 0;

	if (tx)
		off = chan ? ADI_TX2_REG_OFF : ADI_TX1_REG_OFF;
	else
		off = chan ? ADI_RX2_REG_OFF : 0;

	if (en)
		/* bring axi core out of reset */
		axiadc_write(st, AIM_AXI_REG(off, ADI_REG_RSTN), ADI_RSTN | ADI_MMCM_RSTN);
	else
		/* reset axi core*/
		axiadc_write(st, AIM_AXI_REG(off, ADI_REG_RSTN), 0);
}

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	conv->chip_info = &axiadc_chip_info_tbl[phy->spi_device_id];
	conv->write_raw = adrv9002_write_raw;
	conv->read_raw = adrv9002_read_raw;
	conv->post_setup = adrv9002_post_setup;
	conv->reg_access = adrv9002_reg_access;
	conv->spi = spi;
	conv->phy = phy;

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
}

int __maybe_unused adrv9002_axi_tx_test_pattern_cfg(struct adrv9002_rf_phy *phy, const int channel,
						    const adi_adrv9001_SsiTestModeData_e data)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int off, start, n_chan, c, sel;

	if (phy->rx2tx2) {
		off = ADI_TX1_REG_OFF;
		start = channel * 2;
		/* I and Q channels */
		n_chan = start + 2;
	} else {
		off = channel ? ADI_TX2_REG_OFF : ADI_TX1_REG_OFF;
		start = 0;
		n_chan = conv->chip_info->num_channels;
	}

	switch (data) {
	case ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL:
		/* DATA_SEL_DDS */
		sel = 0;
		break;
	case ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN:
		/* DATA_SEL_SED */
		sel = 1;
		break;
	case ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE:
		/* DATA_SEL_RAMP_NIBBLE */
		sel = 10;
		break;
	case ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT:
		/* DATA_SEL_RAMP_16 */;
		sel = 11;
		break;
	case ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15:
		/* DATA_SEL_PN15 */
		sel = 7;
		break;
	case ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7:
		/* DATA_SEL_PN7 */
		sel = 6;
		break;
	default:
		return -EINVAL;
	}

	for (c = start; c < n_chan; c++)
		axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_CHAN_CTRL_7(c)), sel);

	axiadc_write(st, AIM_AXI_REG(off, ADI_TX_REG_CTRL_1), 1);

	return 0;
}

void adrv9002_axi_hdl_loopback(struct adrv9002_rf_phy *phy, int channel, bool enable)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int off, start, n_chan, c, reg;

	if (phy->rx2tx2) {
		start = channel * 2;
		/* I and Q channels */
		n_chan = start + 2;
		off = 0;
	} else {
		start = 0;
		n_chan = conv->chip_info->num_channels;
		off = channel ? ADI_RX2_REG_OFF : 0;
	}

	for (c = start; c < n_chan; c++) {
		reg = axiadc_read(st, AIM_AXI_REG(off, ADI_REG_CHAN_CNTRL_3(c)));
		reg = (reg & ~ADI_ADC_DATA_SEL(~0)) | ADI_ADC_DATA_SEL(enable);
		axiadc_write(st, AIM_AXI_REG(off, ADI_REG_CHAN_CNTRL_3(c)), reg);
	}
}

u32 adrv9002_axi_dds_rate_get(struct adrv9002_rf_phy *phy, const int chan)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	const int off = chan ? ADI_TX2_REG_OFF : ADI_TX1_REG_OFF;

	/* the rate is decremented by one when configured on the core */
	return axiadc_read(st, AIM_AXI_REG(off, ADI_TX_REG_RATE)) + 1;
}

#else  /* CONFIG_CF_AXI_ADC */

u32 adrv9002_axi_dds_rate_get(struct adrv9002_rf_phy *phy, const int chan)
{
	return -ENODEV;
}

int adrv9002_axi_interface_set(struct adrv9002_rf_phy *phy, const u8 n_lanes,
			       const bool cmos_ddr, const int channel, const bool tx)
{
	return -ENODEV;
}

int adrv9002_axi_intf_tune(struct adrv9002_rf_phy *phy, const bool tx, const int chann,
			   u8 *clk_delay, u8 *data_delay)
{
	return -ENODEV;
}

void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan, const bool tx,
				   const bool en)
{
	return -ENODEV;
}

int adrv9002_axi_tx_test_pattern_cfg(struct adrv9002_rf_phy *phy, const int channel,
				     const adi_adrv9001_SsiTestModeData_e data)
{
	return -ENODEV;
}

int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;

	spi_set_drvdata(spi, phy); /* Take care here */

	return 0;
}

#endif /* CONFIG_CF_AXI_ADC */
