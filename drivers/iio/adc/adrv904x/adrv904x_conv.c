// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV904X RF Transceiver
 *
 * Copyright 2020-2024 Analog Devices Inc.
 *
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

#include "adrv904x.h"

#if IS_ENABLED(CONFIG_CF_AXI_ADC)
#include "../cf_axi_adc.h"

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)                                   \
	{                                                                          \
		.type = IIO_VOLTAGE, .indexed = 1, .modified = 1,                  \
		.channel = _chan, .channel2 = _mod,                                \
		.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |              \
				      BIT(IIO_CHAN_INFO_CALIBBIAS) |               \
				      BIT(IIO_CHAN_INFO_CALIBPHASE),               \
		.info_mask_shared_by_type = BIT(                                   \
			IIO_CHAN_INFO_SAMP_FREQ), /*.ext_info = axiadc_ext_info,*/ \
			.scan_index = _si,                                         \
		.scan_type = {                                                     \
			.sign = _sign,                                             \
			.realbits = _bits,                                         \
			.storagebits = 16,                                         \
			.shift = 0,                                                \
		},                                                                 \
	}

#define AIM_MC_CHAN(_chan, _si, _bits, _sign)                                  \
	{                                                                      \
		.type = IIO_VOLTAGE, .indexed = 1, .channel = _chan,           \
		.scan_index = _si,                                             \
		.scan_type = {                                                 \
			.sign = _sign,                                         \
			.realbits = _bits,                                     \
			.storagebits = 16,                                     \
			.shift = 0,                                            \
		},                                                             \
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_ADRV9040] =
		{
			.name = "ADRV904X",
			.max_rate = 245760000,
			.max_testmode = 0,
			.num_channels = 16,
			.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
			.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
			.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
			.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
			.channel[4] = AIM_CHAN(2, IIO_MOD_I, 4, 16, 'S'),
			.channel[5] = AIM_CHAN(2, IIO_MOD_Q, 5, 16, 'S'),
			.channel[6] = AIM_CHAN(3, IIO_MOD_I, 6, 16, 'S'),
			.channel[7] = AIM_CHAN(3, IIO_MOD_Q, 7, 16, 'S'),
			.channel[8] = AIM_CHAN(4, IIO_MOD_I, 8, 16, 'S'),
			.channel[9] = AIM_CHAN(4, IIO_MOD_Q, 9, 16, 'S'),
			.channel[10] = AIM_CHAN(5, IIO_MOD_I, 10, 16, 'S'),
			.channel[11] = AIM_CHAN(5, IIO_MOD_Q, 11, 16, 'S'),
			.channel[12] = AIM_CHAN(6, IIO_MOD_I, 12, 16, 'S'),
			.channel[13] = AIM_CHAN(6, IIO_MOD_Q, 13, 16, 'S'),
			.channel[14] = AIM_CHAN(7, IIO_MOD_I, 14, 16, 'S'),
			.channel[15] = AIM_CHAN(7, IIO_MOD_Q, 15, 16, 'S'),
		},
};

static int adrv904x_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long m)
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

static int adrv904x_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -ENODEV;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int adrv904x_hdl_loopback(struct adrv904x_rf_phy *phy, bool enable)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	struct axiadc_state *st;
	unsigned reg, addr, chan, version;

	if (!conv)
		return -ENODEV;

	st = iio_priv(conv->indio_dev);
	version = axiadc_read(st, 0x4000);

	addr = 0x4418;

	for (chan = 0; chan < conv->chip_info->num_channels; chan++) {
		reg = axiadc_read(st, addr + (chan)*0x40);

		if (enable && reg != 0x8) {
			conv->scratch_reg[chan] = reg;
			reg = 0x8;
		} else if (reg == 0x8)
			reg = conv->scratch_reg[chan];

		axiadc_write(st, addr + (chan)*0x40, reg);
	}

	return 0;
}
EXPORT_SYMBOL(adrv904x_hdl_loopback);

static int adrv904x_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	unsigned tmp, num_chan;
	int i;

	num_chan = conv->chip_info->num_channels;

	conv->indio_dev = indio_dev;
	axiadc_write(st, ADI_REG_CNTRL, 0);
	tmp = axiadc_read(st, 0x4048);

	tmp &= ~BIT(5);
	axiadc_write(st, 0x4048, tmp);
	axiadc_write(st, 0x404c, 3); /* RATE */

	for (i = 0; i < num_chan; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(i), ADI_DCFILT_OFFSET(0));
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
				     ADI_ENABLE | ADI_IQCOR_ENB);
	}

	return 0;
}

int adrv904x_register_axi_converter(struct adrv904x_rf_phy *phy)
{
	struct axiadc_converter *conv;
	struct spi_device *spi = phy->spi;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->chip_info = &axiadc_chip_info_tbl[phy->spi_device_id];
	conv->write_raw = adrv904x_write_raw;
	conv->read_raw = adrv904x_read_raw;
	conv->post_setup = adrv904x_post_setup;
	conv->spi = spi;
	conv->phy = phy;

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->adc_clk = clk_get_rate(conv->clk);

	spi_set_drvdata(spi, conv); /* Take care here */

	return 0;
}
EXPORT_SYMBOL(adrv904x_register_axi_converter);

struct adrv904x_rf_phy *adrv904x_spi_to_phy(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	return conv->phy;
}
EXPORT_SYMBOL(adrv904x_spi_to_phy);

#else /* CONFIG_CF_AXI_ADC */

int adrv904x_hdl_loopback(struct adrv904x_rf_phy *phy, bool enable)
{
	return -ENODEV;
}
EXPORT_SYMBOL(adrv904x_hdl_loopback);

int adrv904x_register_axi_converter(struct adrv904x_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	spi_set_drvdata(spi, phy); /* Take care here */

	return 0;
}
EXPORT_SYMBOL(adrv904x_register_axi_converter);

struct adrv904x_rf_phy *adrv904x_spi_to_phy(struct spi_device *spi)
{
	return spi_get_drvdata(spi);
}
EXPORT_SYMBOL(adrv904x_spi_to_phy);

#endif /* CONFIG_CF_AXI_ADC */
