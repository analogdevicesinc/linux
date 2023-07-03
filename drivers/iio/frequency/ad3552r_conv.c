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
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "../dac/ad3552r.h"

#include "cf_axi_dds.h"

static u32 axi_ad3552r_read_wrapper(struct reg_addr_poll *addr)
{
	struct cf_axi_converter *conv = to_converter(addr->st->dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);

	return dds_read(dds, addr->reg);
}

static void axi_ad3552r_update_bits(struct cf_axi_dds_state *dds, u32 reg,
				    u32 mask, u32 val)
{
	u32 tmp, orig;

	orig = dds_read(dds, reg);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		dds_write(dds, reg, tmp);
}

static void axi_ad3552r_spi_write(struct axi_ad3552r_state *st, u32 reg, u32 val,
				  u32 transfer_params)
{
	struct cf_axi_converter *conv = to_converter(st->dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	struct reg_addr_poll addr;
	u32 check;

	if (!st->has_lock)
		mutex_lock(&st->lock);

	if (transfer_params & AXI_MSK_SYMB_8B)
		dds_write(dds, AXI_REG_CNTRL_DATA_WR, CNTRL_DATA_WR_8(val));
	else
		dds_write(dds, AXI_REG_CNTRL_DATA_WR, CNTRL_DATA_WR_16(val));

	dds_write(dds, AXI_REG_CNTRL_2, transfer_params);

	axi_ad3552r_update_bits(dds, AXI_REG_CNTRL_CSTM, AXI_MSK_ADDRESS,
				CNTRL_CSTM_ADDR(reg));
	axi_ad3552r_update_bits(dds, AXI_REG_CNTRL_CSTM, AXI_MSK_TRANSFER_DATA,
				AXI_MSK_TRANSFER_DATA);
	addr.st = st;
	addr.reg = AXI_REG_UI_STATUS;
	readx_poll_timeout(axi_ad3552r_read_wrapper, &addr, check,
			   check == AXI_MSK_BUSY, 10, 100);

	axi_ad3552r_update_bits(dds, AXI_REG_CNTRL_CSTM, AXI_MSK_TRANSFER_DATA, 0);

	if (!st->has_lock)
		mutex_unlock(&st->lock);
}

static u32 axi_ad3552r_spi_read(struct axi_ad3552r_state *st, u32 reg,
				u32 transfer_params)
{
	struct cf_axi_converter *conv = to_converter(st->dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	u32 val;

	mutex_lock(&st->lock);
	st->has_lock = true;

	axi_ad3552r_spi_write(st, RD_ADDR(reg), 0x00, transfer_params);
	val = dds_read(dds, AXI_REG_CNTRL_DATA_RD);

	st->has_lock = false;
	mutex_unlock(&st->lock);

	return val;
}

static void axi_ad3552r_spi_update_bits(struct axi_ad3552r_state *st, u32 reg,
					u32 mask, u32 val, u32 transfer_params)
{
	u32 tmp, orig;

	orig = axi_ad3552r_spi_read(st, reg, transfer_params);
	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig)
		axi_ad3552r_spi_write(st, reg, tmp, transfer_params);
}

static int axi_ad3552r_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2,
				long mask)
{

	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axi_ad3552r_state *st = conv->phy;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->single_channel)
			*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk), 4);
		else
			*val = DIV_ROUND_UP(clk_get_rate(st->ref_clk), 8);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->channel) {
			*val = axi_ad3552r_spi_read(st, AD3552R_REG_CH1_DAC_16B,
						    AD3552R_TFER_16BIT_SDR);
		} else {
			*val = axi_ad3552r_spi_read(st, AD3552R_REG_CH0_DAC_16B,
						    AD3552R_TFER_16BIT_SDR);
		}
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int axi_ad3552r_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val,
				 int val2,
				 long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axi_ad3552r_state *st = conv->phy;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			axi_ad3552r_spi_write(st, AD3552R_REG_CH1_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		else
			axi_ad3552r_spi_write(st, AD3552R_REG_CH0_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		return 0;
	}
	return -EINVAL;
}

static int axi_ad3552r_write(struct device *dev, u32 reg, u32 val)
{
	struct cf_axi_converter *conv = to_converter(dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);

	dds_write(dds, reg, val);
	return 0;
}

static int axi_ad3552r_read(struct device *dev, u32 reg)
{
	struct cf_axi_converter *conv = to_converter(dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);

	return dds_read(dds, reg);
}

unsigned long long ad3552r_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad3552r_set_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	axi_ad3552r_spi_update_bits(conv->phy, AD3552R_REG_OUTPUT_RANGE,
				    AD3552R_MASK_OUT_RANGE,
				    SET_CH1_RANGE(mode) | SET_CH0_RANGE(mode),
				    AD3552R_TFER_8BIT_SDR);
	mdelay(100);

	return 0;

}

static int ad3552r_get_output_range(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	u32 val;

	val = axi_ad3552r_spi_read(conv->phy, AD3552R_REG_OUTPUT_RANGE,
				   AD3552R_TFER_8BIT_SDR);
	return GET_CH0_RANGE(val);
}

static int ad3552r_set_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);

	dds_write(dds, ADI_REG_CHAN_CNTRL_7(0), mode);
	dds_write(dds, ADI_REG_CHAN_CNTRL_7(1), mode);

	return 0;
}

static int ad3552r_get_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);

	return dds_read(dds, ADI_REG_CHAN_CNTRL_7(0));
}

static int ad3552r_set_stream_state(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);

	if (mode == 2) {
		axi_ad3552r_update_bits(dds, ADI_REG_CONFIG, ADI_DDS_DISABLE, 0);
		cf_axi_dds_start_sync(dds, true);
	} else if (mode == 1) {
		axi_ad3552r_update_bits(dds, ADI_REG_CONFIG, ADI_DDS_DISABLE, 0);
	} else {
		axi_ad3552r_update_bits(dds, ADI_REG_CONFIG, ADI_DDS_DISABLE, 1);
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

static int axi_ad3552r_reset(struct axi_ad3552r_state *st)
{
	struct cf_axi_converter *conv = to_converter(st->dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);

	axi_ad3552r_update_bits(dds, AXI_REG_RSTN, AXI_RST, 0x00);
	axi_ad3552r_update_bits(dds, AXI_REG_RSTN, AXI_RST, AXI_RST);

	st->reset_gpio = devm_gpiod_get_optional(st->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		usleep_range(1, 10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	} else {
		axi_ad3552r_spi_update_bits(st, AD3552R_REG_INTERFACE_CONFIG_A,
					    AD3552R_MASK_SW_RST,
					    AD3552R_MASK_SW_RST,
					    AD3552R_TFER_8BIT_SDR);
	}
	msleep_interruptible(100);

	return 0;
}

static int axi_ad3552r_setup(struct cf_axi_converter *conv)
{
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	struct axi_ad3552r_state *st = conv->phy;
	u8 val;
	u16 id;
	int ret;

	ret = axi_ad3552r_reset(st);
	if (ret)
		return ret;

	axi_ad3552r_spi_write(st, AD3552R_REG_SCRATCH_PAD,
			      AD3552R_SCRATCH_PAD_TEST_VAL,
			      AD3552R_TFER_8BIT_SDR);
	val = axi_ad3552r_spi_read(st, AD3552R_REG_SCRATCH_PAD,
				   AD3552R_TFER_8BIT_SDR);

	if (val != AD3552R_SCRATCH_PAD_TEST_VAL)
		return -EINVAL;

	val = axi_ad3552r_spi_read(st, AD3552R_REG_PRODUCT_ID_L,
				   AD3552R_TFER_8BIT_SDR);

	id = val;
	mdelay(100);
	val = axi_ad3552r_spi_read(st, AD3552R_REG_PRODUCT_ID_H,
				   AD3552R_TFER_8BIT_SDR);

	id |= val << 8;
	if (id != AD3552R_ID)
		dev_warn(st->dev,
			 "Chip ID mismatch. Expected 0x%x, Reported 0x%x\n",
			 AD3552R_ID, id);

	axi_ad3552r_spi_write(st, AD3552R_REG_REF_CONFIG, AD3552R_REF_INIT,
			      AD3552R_TFER_8BIT_SDR);
	axi_ad3552r_spi_write(st, AD3552R_REG_TRANSFER, AD3552R_TRANSFER_INIT,
			      AD3552R_TFER_8BIT_SDR);

	dds_write(dds, ADI_REG_CHAN_CNTRL_7(0), AXI_SEL_SRC_ADC);
	dds_write(dds, ADI_REG_CHAN_CNTRL_7(1), AXI_SEL_SRC_ADC);

	return 0;
}

static void ad3552r_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int axi_ad3552r_probe(struct platform_device *pdev)
{
	struct cf_axi_dds_chip_info *chip_info;
	struct cf_axi_converter *conv;
	struct axi_ad3552r_state *st;
	int ret;

	dev_info(&pdev->dev, "Start ad3552r_conv converter ...");
	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->dev = &pdev->dev;

	conv = devm_kzalloc(&pdev->dev, sizeof(*conv), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(conv->dev, ad3552r_clk_disable,
				       st->ref_clk);
	if (ret < 0)
		return ret;

	mutex_init(&st->lock);
	st->has_lock = false;

	conv->id = ID_AD3552R;
	conv->dev = &pdev->dev;
	conv->phy = st;
	conv->write_raw = axi_ad3552r_write_raw;
	conv->read_raw = axi_ad3552r_read_raw;
	conv->write = axi_ad3552r_write;
	conv->read = axi_ad3552r_read;
	conv->setup = axi_ad3552r_setup;
	conv->clk[CLK_DAC] = st->ref_clk;
	conv->get_data_clk = ad3552r_get_data_clk;
	chip_info = &cf_axi_dds_chip_info_tbl[ID_AD3552R];
	chip_info->channel[0].ext_info = ad3552r_ext_info;
	chip_info->channel[1].ext_info = ad3552r_ext_info;

	dev_set_drvdata(conv->dev, conv);

	dev_info(conv->dev, "Probed ad3552r_conv converter");

	return 0;
}

static const struct of_device_id axi_ad3552r_of_id[] = {
	{ .compatible = "adi,axi-ad3552r" },
	{}
};
MODULE_DEVICE_TABLE(of, axi_ad3552r_of_id);

static struct platform_driver axi_ad3552r_driver = {
	.driver = {
		.name = "axi-ad3552r",
		.owner = THIS_MODULE,
		.of_match_table = axi_ad3552r_of_id,
	},
	.probe = axi_ad3552r_probe,
};
module_platform_driver(axi_ad3552r_driver);
