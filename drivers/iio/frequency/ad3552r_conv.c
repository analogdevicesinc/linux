// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AD3552R SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2023 Analog Devices Inc.
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

#include "../dac/ad3552r.h"

#include "cf_axi_dds.h"

unsigned long long ad3552r_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static void axi_ad3552r_update_bits(struct cf_axi_dds_state *st, u32 reg,
				    u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = dds_read(st, reg);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		dds_write(st, reg, tmp);
}

static int ad3552r_set_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	return ad3552r_update_reg_field(conv->phy, AD3552R_REG_OUTPUT_RANGE,
				    AD3552R_MASK_OUT_RANGE,
				    SET_CH1_RANGE(mode) | SET_CH0_RANGE(mode));
}

static int ad3552r_get_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	u16 tmp_val;
	int ret;

	ret = ad3552r_read_reg(conv->phy, AD3552R_REG_OUTPUT_RANGE, &tmp_val);
	if (ret < 0)
		return ret;

	return GET_CH0_RANGE(tmp_val);
}

static int ad3552r_set_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	dds_write(st, ADI_REG_CHAN_CNTRL_7(0), mode);
	dds_write(st, ADI_REG_CHAN_CNTRL_7(1), mode);

	return 0;
}

static int ad3552r_get_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	return dds_read(st, ADI_REG_CHAN_CNTRL_7(0));
}

static int ad3552r_set_stream_state(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	int ret;

	if (mode == 2) {
		axi_ad3552r_update_bits(st, ADI_REG_CONFIG, ADI_DDS_DISABLE, 0);
		cf_axi_dds_start_sync(st, true);
	} else if (mode == 1) {
		axi_ad3552r_update_bits(st, ADI_REG_CONFIG, ADI_DDS_DISABLE, 0);
	} else {
		axi_ad3552r_update_bits(st, ADI_REG_CONFIG, ADI_DDS_DISABLE, 1);
	}

	return 0;
}

static int ad3552r_get_stream_state(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	return cf_axi_dds_dma_fifo_en(st);
}

static const struct iio_enum ad35525_source_enum = {
	.items = input_source,
	.num_items = ARRAY_SIZE(input_source),
	.get = ad3552r_get_input_source,
	.set = ad3552r_set_input_source,
};

static const struct iio_enum ad35525_output_enum = {
	.items = output_range,
	.num_items = ARRAY_SIZE(output_range),
	.get = ad3552r_get_output_range,
	.set = ad3552r_set_output_range,
};

static const struct iio_enum ad35525_stream_enum = {
	.items = stream_status,
	.num_items = ARRAY_SIZE(stream_status),
	.get = ad3552r_get_stream_state,
	.set = ad3552r_set_stream_state,
};

static const struct iio_chan_spec_ext_info ad3552r_ext_info[] = {
	IIO_ENUM("input_source", IIO_SHARED_BY_ALL, &ad35525_source_enum),
	IIO_ENUM_AVAILABLE_SHARED("input_source", IIO_SHARED_BY_ALL,
				  &ad35525_source_enum),
	IIO_ENUM("stream_status", IIO_SHARED_BY_ALL, &ad35525_stream_enum),
	IIO_ENUM_AVAILABLE_SHARED("stream_status", IIO_SHARED_BY_ALL,
				  &ad35525_stream_enum),
	IIO_ENUM("output_range", IIO_SHARED_BY_ALL, &ad35525_output_enum),
	IIO_ENUM_AVAILABLE_SHARED("output_range", IIO_SHARED_BY_ALL,
				  &ad35525_output_enum),
	{},
};

int conv_ad3552r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad3552r_desc *dac = conv->phy;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (dac->single_channel)
			*val = DIV_ROUND_UP(ad3552r_get_data_clk(conv), 4);
		else
			*val = DIV_ROUND_UP(ad3552r_get_data_clk(conv), 8);
		return IIO_VAL_INT;
	default:
		return ad3552r_read_raw(indio_dev, chan, val, val2, mask);
	}
}



static int conv_ad3552r_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long mask)
{
	return ad3552r_write_raw(indio_dev, chan, val, val2, mask);
}

static int conv_ad3552r_setup(struct cf_axi_converter *conv)
{
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	int ret;

	ret = ad3552r_write_reg(conv->phy, AD3552R_REG_REF_CONFIG,
				AD3552R_REF_INIT);
	if (ret < 0)
		return ret;

	dds_write(dds, ADI_REG_CHAN_CNTRL_7(0), AXI_SEL_SRC_ADC);
	dds_write(dds, ADI_REG_CHAN_CNTRL_7(1), AXI_SEL_SRC_ADC);

	return 0;
}


static void ad3552r_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

int ad3552r_register_axi_converter(struct ad3552r_desc *dac)
{
	struct cf_axi_converter *conv;
	struct spi_device *spi = dac->spi;
	int ret;

	dev_info(&spi->dev, "Start ad3552r_conv registered converter ...");
	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	dac->ref_clk = devm_clk_get(&spi->dev, "dac_clk");
	if (IS_ERR(dac->ref_clk))
		return PTR_ERR(dac->ref_clk);

	ret = clk_prepare_enable(dac->ref_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad3552r_clk_disable,
				       dac->ref_clk);
	if (ret < 0)
		return ret;

	// Product ID was checked previously at ad3552r_init().
	conv->id = ID_AD3552R;
	conv->phy = dac;
	conv->spi = spi;
	conv->write_raw = conv_ad3552r_write_raw;
	conv->read_raw = conv_ad3552r_read_raw;
	conv->setup = conv_ad3552r_setup;
	conv->clk[CLK_DAC] = dac->ref_clk;
	conv->get_data_clk = ad3552r_get_data_clk;

	spi_set_drvdata(spi, conv);

	dev_info(&spi->dev, "Registered ad3552r_conv converter");
	return 0;
}
EXPORT_SYMBOL(ad3552r_register_axi_converter);

struct ad3552r_desc* ad3552r_spi_to_phy(struct spi_device *spi)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	return conv->phy;
}
EXPORT_SYMBOL(ad3552r_spi_to_phy);
