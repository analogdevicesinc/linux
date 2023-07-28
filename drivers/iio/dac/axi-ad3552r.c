// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AXI AD3552R DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "../frequency/cf_axi_dds.h"

#define AXI_REG_RSTN				0x40
#define   AXI_MSK_RSTN				BIT(0)
#define   AXI_MSK_MMCM_RSTN			BIT(1)
#define   AXI_MSK_USIGN_DATA			BIT(4)
#define   AXI_MSK_SYMB_8B			BIT(14)
#define   AXI_MSK_SDR_DDR_N			BIT(16)

/* Masks for accessing DAC Common REG_DAC_CUSTOM_CTRL register */
#define   AXI_MSK_TRANSFER_DATA			BIT(0)
#define   AXI_MSK_STREAM			BIT(1)
#define   AXI_MSK_SYNCED_TRANSFER		BIT(2)
#define   AXI_MSK_ADDRESS			GENMASK(31, 24)

#define   AXI_EXT_SYNC_ARM		0x02

#define   AXI_SEL_SRC_DDS			0x00
#define   AXI_SEL_SRC_DMA			0x02
#define   AXI_SEL_SRC_ADC			0x08
#define   AXI_SEL_SRC_16_B_RAMP			0x0b

#define AD3552R_REG_INTERFACE_CONFIG_A		0x00
#define   AD3552R_MASK_SW_RST			(BIT(7) | BIT(0))
#define AD3552R_REG_PRODUCT_ID_L		0x04
#define AD3552R_REG_PRODUCT_ID_H		0x05
#define AD3552R_REG_SCRATCH_PAD			0x0A
#define AD3552R_REG_STREAM_MODE			0x0E
#define AD3552R_REG_TRANSFER			0x0F
#define   AD3552R_MASK_STREAM_LENGTH_KEEP	BIT(2)
#define   AD3552R_MASK_MULTI_IO_MODE		GENMASK(7, 6)
#define AD3552R_REG_INTERFACE_CONFIG_D		0x14
#define   AD3552R_MASK_SPI_CONFIG_DDR		BIT(0)
#define   AD3552R_MASK_DUAL_SPI_SYNC_EN		BIT(1)
#define   AD3552R_MASK_SDO_DRIVE_STRENGTH	GENMASK(3, 2)
#define   AD3552R_MASK_MEM_CRC_EN		BIT(4)
#define   AD3552R_MASK_ALERT_ENABLE_PULLUP	BIT(6)
#define	AD3552R_REG_REF_CONFIG			0x15
#define AD3552R_REG_OUTPUT_RANGE		0x19
#define   AD3552R_MASK_OUT_RANGE        GENMASK(7, 0)
#define   AD3552R_MASK_CH0_RANGE		GENMASK(2, 0)
#define   AD3552R_MASK_CH1_RANGE		GENMASK(6, 4)
#define AD3552R_REG_CH0_DAC_16B			0x2A
#define AD3552R_REG_CH1_DAC_16B			0x2C

#define AD3552R_TFER_8BIT_SDR			(AXI_MSK_SYMB_8B | \
						AXI_MSK_SDR_DDR_N)
#define AD3552R_TFER_8BIT_DDR			AXI_MSK_SYMB_8B
#define AD3552R_TFER_16BIT_SDR			AXI_MSK_SDR_DDR_N
#define AD3552R_TFER_16BIT_DDR			0x00

#define AD3552R_SINGLE_SPI			0x00
#define AD3552R_DUAL_SPI			0x01
#define AD3552R_QUAD_SPI			0x02

#define AD3552R_SCRATCH_PAD_TEST_VAL		0x5A
#define AD3552R_ID				0x4008

#define AD3552R_REF_INIT			0x00
#define AD3552R_TRANSFER_INIT			(FIELD_PREP(AD3552R_MASK_MULTI_IO_MODE,\
							    AD3552R_QUAD_SPI) |\
						AD3552R_MASK_STREAM_LENGTH_KEEP)

#define AD3552R_STREAM_2BYTE_LOOP		0x02
#define AD3552R_STREAM_4BYTE_LOOP		0x04
#define AD3552R_STREAM_SATRT			(AXI_MSK_TRANSFER_DATA | \
						AXI_MSK_STREAM)

#define AD3552R_CH0_ACTIVE			BIT(0)
#define AD3552R_CH1_ACTIVE			BIT(1)
#define AD3552R_CH0_CH1_ACTIVE			(AD3552R_CH0_ACTIVE | \
						 AD3552R_CH1_ACTIVE)

#define AXI_RST					(AXI_MSK_RSTN | AXI_MSK_MMCM_RSTN)

#define CNTRL_CSTM_ADDR(x)			FIELD_PREP(AXI_MSK_ADDRESS, x)

#define RD_ADDR(x)				(BIT(7) | (x))

#define SET_CH0_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH0_RANGE, x)
#define SET_CH1_RANGE(x)			FIELD_PREP(AD3552R_MASK_CH1_RANGE, x)
#define GET_CH0_RANGE(x)			FIELD_GET(AD3552R_MASK_CH0_RANGE, x)
#define GET_CH1_RANGE(x)			FIELD_GET(AD3552R_MASK_CH1_RANGE, x)

enum ad35525_out_range {
	AD3552R_0_2_5,
	AD3552R_0_5,
	AD3552R_0_10,
	AD3552R_5_5,
	AD3552R_10_10
};

enum ad35525_source {
	AD3552R_DDS	= AXI_SEL_SRC_DDS,
	AD3552R_ADC	= AXI_SEL_SRC_ADC,
	AD3552R_DMA	= AXI_SEL_SRC_DMA,
	AD3552R_RAMP	= AXI_SEL_SRC_16_B_RAMP
};

enum ad35525_stream_status {
	AD3552R_STOP_STREAM,
	AD3552R_START_STREAM,
	AD3552R_START_STREAM_SYNCED,
};

static const char *const stream_status[] = {
	[AD3552R_STOP_STREAM] = "stop_stream",
	[AD3552R_START_STREAM] = "start_stream",
	[AD3552R_START_STREAM_SYNCED] = "start_stream_synced"
};

static const char * const input_source[] = {
	[AD3552R_DDS]	= "dds_input",
	[AD3552R_ADC]	= "adc_input",
	[AD3552R_DMA]	= "dma_input",
	[AD3552R_RAMP]	= "ramp_input"
};

static const char * const output_range[] = {
	[AD3552R_0_2_5]	= "0/2.5V",
	[AD3552R_0_5]	= "0/5V",
	[AD3552R_0_10]	= "0/10V",
	[AD3552R_5_5]	= "-5/+5V",
	[AD3552R_10_10]	= "-10/+10V"
};

struct axi_ad3552r_state {
	struct gpio_desc *reset_gpio;
	struct clk *ref_clk;
	struct device *dev;
	/* protect device accesses */
	struct mutex lock;
	bool has_lock;
	bool ddr;
	bool single_channel;
	bool synced_transfer;
};

struct reg_addr_poll {
	struct axi_ad3552r_state *st;
	u8 reg;
};



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
		dds_write(dds, ADI_REG_DAC_CUSTOM_WR, CNTRL_DATA_WR_8(val));
	else
		dds_write(dds, ADI_REG_DAC_CUSTOM_WR, CNTRL_DATA_WR_16(val));

	dds_write(dds, ADI_REG_CNTRL_2, transfer_params);

	axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_ADDRESS,
				CNTRL_CSTM_ADDR(reg));
	axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_TRANSFER_DATA,
				AXI_MSK_TRANSFER_DATA);
	addr.st = st;
	addr.reg = ADI_REG_UI_STATUS;
	readx_poll_timeout(axi_ad3552r_read_wrapper, &addr, check,
			   check == ADI_AXI_MSK_BUSY, 10, 100);

	axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_TRANSFER_DATA, 0);

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
	val = dds_read(dds, ADI_REG_DAC_CUSTOM_RD);

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


static int ad3552r_set_sample_rate(struct cf_axi_converter *conv, u64 sample_rate)
{
	int ret;

	sample_rate = clk_round_rate(conv->clk[CLK_DAC], sample_rate);
	clk_disable_unprepare(conv->clk[CLK_DAC]);
	ret = clk_set_rate(conv->clk[CLK_DAC], sample_rate);
	if (ret < 0)
		return ret;

	return clk_prepare_enable(conv->clk[CLK_REF]);
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
	u64 sample_rate;
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		sample_rate = (u64)val2 << 32 | val;
		if (st->single_channel)
			sample_rate *= 4;
		else
			sample_rate *= 8;
		ret = ad3552r_set_sample_rate(conv, sample_rate);
		if (ret < 0)
			dev_err(conv->dev, "Failed to set sample rate: %d\n", ret);
		break;

	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			axi_ad3552r_spi_write(st, AD3552R_REG_CH1_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		else
			axi_ad3552r_spi_write(st, AD3552R_REG_CH0_DAC_16B,
					      (u32)val, AD3552R_TFER_16BIT_SDR);
		break;
	default:
		return -EINVAL;
	}

	return ret;
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

	dev_info(&indio_dev->dev, "ad3552r set_output_range, mode: %u", mode);
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
	dev_info(&indio_dev->dev, "ad3552r get_output_range, GET_CH0_RANGE(val): %lu", GET_CH0_RANGE(val));
	return GET_CH0_RANGE(val);
}

static int ad3552r_set_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);

	dev_info(&indio_dev->dev, "ad3552r set_input_source, mode: %u", mode);
	dds_write(dds, ADI_REG_CHAN_CNTRL_7(0), mode);
	dds_write(dds, ADI_REG_CHAN_CNTRL_7(1), mode);

	return 0;
}

static int ad3552r_get_input_source(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);
	int source;

	source = dds_read(dds, ADI_REG_CHAN_CNTRL_7(0));

	dev_info(&indio_dev->dev, "ad3552r get_input_source, source: %d", source);
	return dds_read(dds, ADI_REG_CHAN_CNTRL_7(0));
}

// TODO Prevent user from setting output range while ad3552r is streaming.

static int ad3552r_set_stream_state(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct cf_axi_dds_state *dds = iio_priv(indio_dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axi_ad3552r_state *st = conv->phy;

	dev_info(&indio_dev->dev, "ad3552r set_stream_state, mode: %u", mode);
	if (mode == 2) {
		st->synced_transfer = true;
		axi_ad3552r_write(conv->dev, ADI_REG_CNTRL_1, AXI_EXT_SYNC_ARM);

		axi_ad3552r_write(conv->dev, ADI_REG_CNTRL_2,
				  (u32)(AXI_MSK_USIGN_DATA | ~AXI_MSK_SDR_DDR_N));

		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL,
					AD3552R_STREAM_SATRT,
					AD3552R_STREAM_SATRT);
	} else if (mode == 1) {
		st->synced_transfer = false;
		axi_ad3552r_write(conv->dev, ADI_REG_CNTRL_2,
				  (u32)(AXI_MSK_USIGN_DATA | ~AXI_MSK_SDR_DDR_N));
		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL,
					AD3552R_STREAM_SATRT,
					AD3552R_STREAM_SATRT);
	} else {
		st->synced_transfer = false;
		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL,
					AD3552R_STREAM_SATRT, 0);
	}

	return 0;
}

static int ad3552r_get_stream_state(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axi_ad3552r_state *st = conv->phy;
	u32 val;

	val = axi_ad3552r_read(conv->dev, ADI_REG_DAC_CUSTOM_CTRL);

	dev_info(&indio_dev->dev, "ad3552r get_stream_state, val: %u", val);
	dev_info(&indio_dev->dev, "ad3552r get_stream_state, (val & MSK): %lu",
		val & AXI_MSK_STREAM);
	if ((val & AXI_MSK_STREAM) == 2 && st->synced_transfer)
		return AD3552R_START_STREAM_SYNCED;
	else if ((val & AXI_MSK_STREAM) == 2)
		return AD3552R_START_STREAM;
	else
		return AD3552R_STOP_STREAM;
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

static int axi_ad3552r_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *active_scan_mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct cf_axi_dds_state *dds = iio_priv(conv->indio_dev);
	struct axi_ad3552r_state *st = conv->phy;

	dev_info(conv->dev, "ad3552r update_scan_mode, mask: %lu", *active_scan_mask);
	axi_ad3552r_spi_update_bits(st, AD3552R_REG_INTERFACE_CONFIG_D,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AD3552R_MASK_SPI_CONFIG_DDR,
				    AD3552R_TFER_8BIT_SDR);

	switch (*active_scan_mask) {
	case AD3552R_CH0_ACTIVE:
		st->single_channel = true;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_2BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH0_DAC_16B));
		return 0;
	case AD3552R_CH1_ACTIVE:
		st->single_channel = true;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_2BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH1_DAC_16B));
		return 0;
	case AD3552R_CH0_CH1_ACTIVE:
		st->single_channel = false;
		axi_ad3552r_spi_write(st, AD3552R_REG_STREAM_MODE,
				      AD3552R_STREAM_4BYTE_LOOP,
				      AD3552R_TFER_8BIT_DDR);
		axi_ad3552r_update_bits(dds, ADI_REG_DAC_CUSTOM_CTRL, AXI_MSK_ADDRESS,
					CNTRL_CSTM_ADDR(AD3552R_REG_CH1_DAC_16B));
		return 0;
	}

	dev_info(conv->dev, "ad3552r invalid active_scan_mask");
	return -EINVAL;
}

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
	if (conv == NULL)
		return -ENOMEM;

	st->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, ad3552r_clk_disable,
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
	conv->update_scan_mode = axi_ad3552r_update_scan_mode;
	conv->clk[CLK_DAC] = st->ref_clk;
	conv->get_data_clk = ad3552r_get_data_clk;
	chip_info = &cf_axi_dds_chip_info_tbl[ID_AD3552R];
	chip_info->channel[0].ext_info = ad3552r_ext_info;
	chip_info->channel[1].ext_info = ad3552r_ext_info;

	dev_set_drvdata(&pdev->dev, conv);

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

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("AXI AD3552R Driver");
MODULE_LICENSE("GPL v2");
