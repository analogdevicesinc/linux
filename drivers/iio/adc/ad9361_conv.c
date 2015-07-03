/*
 * AD9361 Agile RF Transceiver
 *
 * Copyright 2013-2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

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

#include "ad9361.h"

#ifdef CONFIG_CF_AXI_ADC
#include "cf_axi_adc.h"

ssize_t ad9361_dig_interface_timing_analysis(struct ad9361_rf_phy *phy,
						   char *buf, unsigned buflen)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st;
	int ret, i, j, chan, len = 0;
	u8 field[16][16];
	u8 rx;

	if (!conv)
		return -ENODEV;

	st = iio_priv(conv->indio_dev);

	rx = ad9361_spi_read(phy->spi, REG_RX_CLOCK_DATA_DELAY);

	ad9361_bist_prbs(phy, BIST_INJ_RX);

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 16; j++) {
			ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY,
					DATA_CLK_DELAY(j) | RX_DATA_DELAY(i));
			for (chan = 0; chan < 4; chan++)
				axiadc_write(st, ADI_REG_CHAN_STATUS(chan),
					ADI_PN_ERR | ADI_PN_OOS);

			mdelay(1);

			if (axiadc_read(st, ADI_REG_STATUS) & ADI_STATUS) {
				for (chan = 0, ret = 0; chan < 4; chan++)
					ret |= axiadc_read(st, ADI_REG_CHAN_STATUS(chan));
			} else {
				ret = 1;
			}

			field[i][j] = ret;
		}
	}

	ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY, rx);

	ad9361_bist_prbs(phy, BIST_DISABLE);

	len += snprintf(buf + len, buflen, "CLK: %lu Hz 'o' = PASS\n",
		       clk_get_rate(phy->clks[RX_SAMPL_CLK]));
	len += snprintf(buf + len, buflen, "DC");
	for (i = 0; i < 16; i++)
		len += snprintf(buf + len, buflen, "%x:", i);
	len += snprintf(buf + len, buflen, "\n");

	for (i = 0; i < 16; i++) {
		len += snprintf(buf + len, buflen, "%x:", i);
		for (j = 0; j < 16; j++) {
			len += snprintf(buf + len, buflen, "%c ",
					(field[i][j] ? '.' : 'o'));
		}
		len += snprintf(buf + len, buflen, "\n");
	}
	len += snprintf(buf + len, buflen, "\n");

	return len;
}
EXPORT_SYMBOL(ad9361_dig_interface_timing_analysis);

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
			BIT(IIO_CHAN_INFO_CALIBPHASE),			\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	/*.ext_info = axiadc_ext_info,*/			\
	  .scan_index = _si,						\
	  .scan_type = {						\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 0,						\
	  },								\
	}

#define AIM_MC_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .scan_index = _si,						\
	  .scan_type = {						\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 0,						\
	  },								\
	}


static const unsigned long ad9361_2x2_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, /* 1 & 2 chan */
	0x10, 0x20, 0x40, 0x80, 0x30, 0xC0, /* 1 & 2 chan */
	0x33, 0xCC, 0xC3, 0x3C, 0x0F, 0xF0, /* 4 chan */
	0xFF,				   /* 8 chan */
	0x00,
};

static const unsigned long ad9361_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, 0x0F,
	0x00,
};

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9361] = {
		.name = "AD9361",
		.max_rate = 61440000UL,
		.max_testmode = 0,
		.num_channels = 4,
		.scan_masks = ad9361_available_scan_masks,
		.channel[0] = AIM_CHAN(0, 0, 12, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 12, 'S'),
		.channel[2] = AIM_CHAN(2, 2, 12, 'S'),
		.channel[3] = AIM_CHAN(3, 3, 12, 'S'),
	},
	[ID_AD9361_2] = { /* MCS/MIMO 2x AD9361 */
		.name = "AD9361-2",
		.max_rate = 61440000UL,
		.max_testmode = 0,
		.num_channels = 8,
		.num_shadow_slave_channels = 4,
		.scan_masks = ad9361_2x2_available_scan_masks,
		.channel[0] = AIM_CHAN(0, 0, 12, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 12, 'S'),
		.channel[2] = AIM_CHAN(2, 2, 12, 'S'),
		.channel[3] = AIM_CHAN(3, 3, 12, 'S'),
		.channel[4] = AIM_MC_CHAN(4, 4, 12, 'S'),
		.channel[5] = AIM_MC_CHAN(5, 5, 12, 'S'),
		.channel[6] = AIM_MC_CHAN(6, 6, 12, 'S'),
		.channel[7] = AIM_MC_CHAN(7, 7, 12, 'S'),
	},
	[ID_AD9364] = {
		.name = "AD9361",
		.max_rate = 61440000UL,
		.max_testmode = 0,
		.num_channels = 2,
		.channel[0] = AIM_CHAN(0, 0, 12, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 12, 'S'),
	},

};

static int ad9361_read_raw(struct iio_dev *indio_dev,
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

static int ad9361_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				"Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		return 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int ad9361_hdl_loopback(struct ad9361_rf_phy *phy, bool enable)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st;
	unsigned reg, addr, chan, version;

	if (!conv)
		return -ENODEV;

	st = iio_priv(conv->indio_dev);
	version = axiadc_read(st, 0x4000);

	/* Still there but implemented a bit different */
	if (PCORE_VERSION_MAJOR(version) > 7)
		addr = 0x4418;
	else
		addr = 0x4414;

	for (chan = 0; chan < conv->chip_info->num_channels; chan++) {
		reg = axiadc_read(st, addr + (chan) * 0x40);

		if (PCORE_VERSION_MAJOR(version) > 7) {
		/* FIXME: May cause problems if DMA is selected */
			if (enable)
				reg = 0x8;
			else
				reg = 0x0;
		} else {
		/* DAC_LB_ENB If set enables loopback of receive data */
			if (enable)
				reg |= BIT(1);
			else
				reg &= ~BIT(1);
		}
		axiadc_write(st, addr + (chan) * 0x40, reg);
	}

	return 0;
}
EXPORT_SYMBOL(ad9361_hdl_loopback);

static int ad9361_iodelay_set(struct axiadc_state *st, unsigned lane,
			      unsigned val, bool tx)
{
	if (tx) {
		if (PCORE_VERSION_MAJOR(st->pcore_version) > 8)
			axiadc_write(st, 0x4000 + ADI_REG_DELAY(lane), val);
		else
			return -ENODEV;
	} else {
		axiadc_idelay_set(st, lane, val);
	}

	return 0;
}

static int ad9361_midscale_iodelay(struct ad9361_rf_phy *phy, bool tx)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int ret = 0, i;

	for (i = 0; i < 7; i++)
		ret |= ad9361_iodelay_set(st, i, 15, tx);

	return 0;
}

static int ad9361_dig_tune_iodelay(struct ad9361_rf_phy *phy, bool tx)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	int ret, i, j, chan, num_chan;
	u32 s0, c0;
	u8 field[32];

	num_chan = (conv->chip_info->num_channels > 4) ? 4 : conv->chip_info->num_channels;

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 32; j++) {
			ad9361_iodelay_set(st, i, j, tx);
			mdelay(1);

			for (chan = 0; chan < num_chan; chan++)
				axiadc_write(st, ADI_REG_CHAN_STATUS(chan),
					ADI_PN_ERR | ADI_PN_OOS);
			mdelay(10);

			for (chan = 0, ret = 0; chan < num_chan; chan++)
				ret |= axiadc_read(st, ADI_REG_CHAN_STATUS(chan));

			field[j] = ret;
		}

		c0 = ad9361_find_opt(&field[0], 32, &s0);
		ad9361_iodelay_set(st, i, s0 + c0 / 2, tx);

		dev_info(&phy->spi->dev,
			 "%s Lane %d, window cnt %d , start %d, IODELAY set to %d\n",
			 tx ? "TX" :"RX",  i , c0, s0, s0 + c0 / 2);

	}

	return 0;
}

static void ad9361_dig_tune_verbose_print(struct ad9361_rf_phy *phy,
					  u8 field[][16], bool tx)
{
	int i, j;

	printk("SAMPL CLK: %lu tuning: %s\n",
	       clk_get_rate(phy->clks[RX_SAMPL_CLK]), tx ? "TX" : "RX");
	printk("  ");
	for (i = 0; i < 16; i++)
		printk("%x:", i);
	printk("\n");

	for (i = 0; i < 2; i++) {
		printk("%x:", i);
		for (j = 0; j < 16; j++) {
			printk("%c ", (field[i][j] ? '#' : 'o'));
		}
		printk("\n");
	}
	printk("\n");
}

int ad9361_dig_tune(struct ad9361_rf_phy *phy, unsigned long max_freq,
			   enum dig_tune_flags flags)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st;
	int ret, i, j, k, chan, t, num_chan, err = 0;
	u32 s0, s1, c0, c1, tmp, saved = 0;
	u8 field[2][16];
	u32 saved_dsel[4], saved_chan_ctrl6[4];
	u32 rates[3] = {25000000U, 40000000U, 61440000U};
	unsigned hdl_dac_version;

	if (!conv)
		return -ENODEV;

	st = iio_priv(conv->indio_dev);
	hdl_dac_version = axiadc_read(st, 0x4000);

	if ((phy->pdata->dig_interface_tune_skipmode == 2) ||
		(flags & RESTORE_DEFAULT)) {
	/* skip completely and use defaults */
		ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY,
				phy->pdata->port_ctrl.rx_clk_data_delay);

		ad9361_spi_write(phy->spi, REG_TX_CLOCK_DATA_DELAY,
				phy->pdata->port_ctrl.tx_clk_data_delay);

		return 0;
	}

	if (flags & DO_IDELAY)
		ad9361_midscale_iodelay(phy, 0);

	if (flags & DO_ODELAY)
		ad9361_midscale_iodelay(phy, 1);


	if (!phy->pdata->fdd) {
		ad9361_set_ensm_mode(phy, true, false);
		ad9361_ensm_force_state(phy, ENSM_STATE_FDD);
	} else {
		ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);
		ad9361_ensm_restore_prev_state(phy);
	}

	num_chan = (conv->chip_info->num_channels > 4) ? 4 :
		conv->chip_info->num_channels;

	ad9361_bist_prbs(phy, BIST_INJ_RX);

	for (t = 0; t < 2; t++) {
		memset(field, 0, 32);
		for (k = 0; k < (max_freq ? ARRAY_SIZE(rates) : 1); k++) {
			if (max_freq)
				ad9361_set_trx_clock_chain_freq(phy, rates[k]);
		for (i = 0; i < 2; i++) {
			for (j = 0; j < 16; j++) {
				ad9361_spi_write(phy->spi,
						REG_RX_CLOCK_DATA_DELAY + t,
						RX_DATA_DELAY(i == 0 ? j : 0) |
						DATA_CLK_DELAY(i ? j : 0));
				for (chan = 0; chan < num_chan; chan++)
					axiadc_write(st, ADI_REG_CHAN_STATUS(chan),
						ADI_PN_ERR | ADI_PN_OOS);
				mdelay(4);

				if ((t == 1) || (axiadc_read(st, ADI_REG_STATUS) & ADI_STATUS)) {
					for (chan = 0, ret = 0; chan < num_chan; chan++) {
						ret |= axiadc_read(st, ADI_REG_CHAN_STATUS(chan));
					}
				} else {
					ret = 1;
				}

				field[i][j] |= ret;
			}
		}

		if ((flags & BE_MOREVERBOSE) && max_freq) {
			ad9361_dig_tune_verbose_print(phy, field, t);
		}
		}

		c0 = ad9361_find_opt(&field[0][0], 16, &s0);
		c1 = ad9361_find_opt(&field[1][0], 16, &s1);

		if (!c0 && !c1) {
			ad9361_dig_tune_verbose_print(phy, field, t);
			dev_err(&phy->spi->dev, "%s: Tuning %s FAILED!", __func__,
				t ? "TX" : "RX");
			err |= -EIO;
		} else if (flags & BE_VERBOSE) {
			ad9361_dig_tune_verbose_print(phy, field, t);
		}

		if (c1 > c0)
			ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY + t,
					DATA_CLK_DELAY(s1 + c1 / 2) |
					RX_DATA_DELAY(0));
		else
			ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY + t,
					DATA_CLK_DELAY(0) |
					RX_DATA_DELAY(s0 + c0 / 2));

		if (t == 0) {
			if (flags & DO_IDELAY)
				ad9361_dig_tune_iodelay(phy, 0);

			/* Now do the loopback and tune the digital out */
			ad9361_bist_prbs(phy, BIST_DISABLE);

			axiadc_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);
			axiadc_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

			if (phy->pdata->dig_interface_tune_skipmode == 1) {
			/* skip TX */
				if (!(flags & SKIP_STORE_RESULT))
					phy->pdata->port_ctrl.rx_clk_data_delay =
						ad9361_spi_read(phy->spi, REG_RX_CLOCK_DATA_DELAY);

				if (!phy->pdata->fdd) {
					ad9361_set_ensm_mode(phy, phy->pdata->fdd,
							     phy->pdata->ensm_pin_ctrl);
					ad9361_ensm_restore_prev_state(phy);
				}
				return 0;
			}

			ad9361_bist_loopback(phy, 1);
			axiadc_write(st, 0x4000 + ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

			for (chan = 0; chan < num_chan; chan++) {
				axiadc_write(st, ADI_REG_CHAN_CNTRL(chan),
					ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
					ADI_ENABLE | ADI_IQCOR_ENB);
				axiadc_set_pnsel(st, chan, ADC_PN_CUSTOM);
				saved_chan_ctrl6[chan] = axiadc_read(st, 0x4414 + (chan) * 0x40);
				if (PCORE_VERSION_MAJOR(hdl_dac_version) > 7) {
					saved_dsel[chan] = axiadc_read(st, 0x4418 + (chan) * 0x40);
					axiadc_write(st, 0x4418 + (chan) * 0x40, 9);
					axiadc_write(st, 0x4414 + (chan) * 0x40, 0); /* !IQCOR_ENB */
					axiadc_write(st, 0x4044, 1);
				} else {
					axiadc_write(st, 0x4414 + (chan) * 0x40, 1); /* DAC_PN_ENB */
				}
			}
			if (PCORE_VERSION_MAJOR(hdl_dac_version) < 8) {
				saved = tmp = axiadc_read(st, 0x4048);
				tmp &= ~0xF;
				tmp |= 1;
				axiadc_write(st, 0x4048, tmp);

			}
		} else {

			if (flags & DO_ODELAY)
				ad9361_dig_tune_iodelay(phy, 1);

			ad9361_bist_loopback(phy, 0);

			if (PCORE_VERSION_MAJOR(hdl_dac_version) < 8)
				axiadc_write(st, 0x4048, saved);

			for (chan = 0; chan < num_chan; chan++) {
				axiadc_write(st, ADI_REG_CHAN_CNTRL(chan),
					ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
					ADI_ENABLE | ADI_IQCOR_ENB);
				axiadc_set_pnsel(st, chan, ADC_PN9);
				if (PCORE_VERSION_MAJOR(hdl_dac_version) > 7) {
					axiadc_write(st, 0x4418 + (chan) * 0x40, saved_dsel[chan]);
					axiadc_write(st, 0x4044, 1);
				}

				axiadc_write(st, 0x4414 + (chan) * 0x40, saved_chan_ctrl6[chan]);
			}

			if (err == -EIO) {
				ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY,
						phy->pdata->port_ctrl.rx_clk_data_delay);

				ad9361_spi_write(phy->spi, REG_TX_CLOCK_DATA_DELAY,
						phy->pdata->port_ctrl.tx_clk_data_delay);
				if (!max_freq)
					err = 0;
			} else if (!(flags & SKIP_STORE_RESULT)) {
				phy->pdata->port_ctrl.rx_clk_data_delay =
					ad9361_spi_read(phy->spi, REG_RX_CLOCK_DATA_DELAY);
				phy->pdata->port_ctrl.tx_clk_data_delay =
					ad9361_spi_read(phy->spi, REG_TX_CLOCK_DATA_DELAY);
			}

			if (!phy->pdata->fdd) {
				ad9361_set_ensm_mode(phy, phy->pdata->fdd, phy->pdata->ensm_pin_ctrl);
				ad9361_ensm_restore_prev_state(phy);
			}

			axiadc_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);
			axiadc_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

			return err;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(ad9361_dig_tune);

static int ad9361_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9361_rf_phy *phy = conv->phy;
	unsigned rx2tx2 = phy->pdata->rx2tx2;
	unsigned tmp, num_chan, flags;
	int i, ret;

	num_chan = (conv->chip_info->num_channels > 4) ? 4 : conv->chip_info->num_channels;

	conv->indio_dev = indio_dev;
	axiadc_write(st, ADI_REG_CNTRL, rx2tx2 ? 0 : ADI_R1_MODE);
	tmp = axiadc_read(st, 0x4048);

	if (!rx2tx2) {
		axiadc_write(st, 0x4048, tmp | BIT(5)); /* R1_MODE */
		axiadc_write(st, 0x404c, 1); /* RATE */
	} else {
		tmp &= ~BIT(5);
		axiadc_write(st, 0x4048, tmp);
		axiadc_write(st, 0x404c, 3); /* RATE */
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

	flags = 0;

	ret = ad9361_dig_tune(phy, (axiadc_read(st, ADI_REG_ID)) ?
		0 : 61440000, flags);
	if (ret < 0)
		goto error;

	if (flags & (DO_IDELAY | DO_ODELAY)) {
		ret = ad9361_dig_tune(phy, (axiadc_read(st, ADI_REG_ID)) ?
			0 : 61440000, flags & BE_VERBOSE);
		if (ret < 0)
			goto error;
	}

	ret = ad9361_set_trx_clock_chain(phy,
					 phy->pdata->rx_path_clks,
					 phy->pdata->tx_path_clks);

	ad9361_ensm_force_state(phy, ENSM_STATE_ALERT);
	ad9361_ensm_restore_prev_state(phy);

	return 0;

error:
	spi_set_drvdata(phy->spi, NULL);
	return ret;
}

int ad9361_register_axi_converter(struct ad9361_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->id = ad9361_spi_read(spi, REG_PRODUCT_ID) & PRODUCT_ID_MASK;
	if (conv->id != PRODUCT_ID_9361) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
  		ret = -ENODEV;
  		goto out;
	}

	conv->chip_info = &axiadc_chip_info_tbl[
		(spi_get_device_id(spi)->driver_data == ID_AD9361_2) ?
		ID_AD9361_2 : phy->pdata->rx2tx2 ? ID_AD9361 : ID_AD9364];
	conv->adc_output_mode = OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->write = ad9361_spi_write;
	conv->read = ad9361_spi_read;
	conv->write_raw = ad9361_write_raw;
	conv->read_raw = ad9361_read_raw;
	conv->post_setup = ad9361_post_setup;
	conv->spi = spi;
	conv->phy = phy;

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
out:
	spi_set_drvdata(spi, NULL);
	return ret;
}
EXPORT_SYMBOL(ad9361_register_axi_converter);

struct ad9361_rf_phy* ad9361_spi_to_phy(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	return conv->phy;
}
EXPORT_SYMBOL(ad9361_spi_to_phy);

#else  /* CONFIG_CF_AXI_ADC */

int ad9361_dig_tune(struct ad9361_rf_phy *phy, unsigned long max_freq,
			   enum dig_tune_flags flags)
{
	return -ENODEV;
}
EXPORT_SYMBOL(ad9361_dig_tune);

ssize_t ad9361_dig_interface_timing_analysis(struct ad9361_rf_phy *phy,
						   char *buf, unsigned buflen)
{
	return 0;
}
EXPORT_SYMBOL(ad9361_dig_interface_timing_analysis);

int ad9361_hdl_loopback(struct ad9361_rf_phy *phy, bool enable)
{
	return -ENODEV;
}
EXPORT_SYMBOL(ad9361_hdl_loopback);

int ad9361_register_axi_converter(struct ad9361_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	spi_set_drvdata(spi, phy); /* Take care here */

	return 0;
}
EXPORT_SYMBOL(ad9361_register_axi_converter);

struct ad9361_rf_phy* ad9361_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}
EXPORT_SYMBOL(ad9361_spi_to_phy);

#endif /* CONFIG_CF_AXI_ADC */
