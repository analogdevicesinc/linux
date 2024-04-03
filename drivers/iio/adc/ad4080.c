// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD4080 SPI ADC driver
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>

/** Register Definition */
#define AD4080_REG_INTERFACE_CONFIG_A		0x00
#define AD4080_REG_INTERFACE_CONFIG_B		0x01
#define AD4080_REG_DEVICE_CONFIG		0x02
#define AD4080_REG_CHIP_TYPE			0x03
#define AD4080_REG_PRODUCT_ID_L			0x04
#define AD4080_REG_PRODUCT_ID_H			0x05
#define AD4080_REG_CHIP_GRADE			0x06
#define AD4080_REG_SCRATCH_PAD			0x0A
#define AD4080_REG_SPI_REVISION			0x0B
#define AD4080_REG_VENDOR_L			0x0C
#define AD4080_REG_VENDOR_H			0x0D
#define AD4080_REG_STREAM_MODE			0x0E
#define AD4080_REG_TRANSFER_CONFIG		0x0F
#define AD4080_REG_INTERFACE_CONFIG_C		0x10
#define AD4080_REG_INTERFACE_STATUS_A		0x11
#define AD4080_REG_DEVICE_STATUS		0x14
#define AD4080_REG_ADC_DATA_INTF_CONFIG_A	0x15
#define AD4080_REG_ADC_DATA_INTF_CONFIG_B	0x16
#define AD4080_REG_ADC_DATA_INTF_CONFIG_C	0x17
#define AD4080_REG_PWR_CTRL			0x18
#define AD4080_REG_GPIO_CONFIG_A		0x19
#define AD4080_REG_GPIO_CONFIG_B		0x1A
#define AD4080_REG_GPIO_CONFIG_C		0x1B
#define AD4080_REG_GENERAL_CONFIG		0x1C
#define AD4080_REG_FIFO_WATERMARK_LSB		0x1D
#define AD4080_REG_FIFO_WATERMARK_MSB		0x1E
#define AD4080_REG_EVENT_HYSTERESIS_LSB		0x1F
#define AD4080_REG_EVENT_HYSTERESIS_MSB		0x20
#define AD4080_REG_EVENT_DETECTION_HI_LSB	0x21
#define AD4080_REG_EVENT_DETECTION_HI_MSB	0x22
#define AD4080_REG_EVENT_DETECTION_LO_LSB	0x23
#define AD4080_REG_EVENT_DETECTION_LO_MSB	0x24
#define AD4080_REG_OFFSET_LSB			0x25
#define AD4080_REG_OFFSET_MSB			0x26
#define AD4080_REG_GAIN_LSB			0x27
#define AD4080_REG_GAIN_MSB			0x28
#define AD4080_REG_FILTER_CONFIG		0x29

/** AD4080_REG_INTERFACE_CONFIG_A Bit Definition */
#define AD4080_SW_RESET_MSK			(BIT(7) | BIT(0))
#define AD4080_ADDR_ASC_MSK			BIT(5)
#define AD4080_SDO_ENABLE_MSK			BIT(4)

/** AD4080_REG_INTERFACE_CONFIG_B Bit Definition */
#define AD4080_SINGLE_INST_MSK			BIT(7)
#define AD4080_SHORT_INST_MSK			BIT(3)

/** AD4080_REG_DEVICE_CONFIG Bit Definition */
#define AD4080_OPERATING_MODES_MSK		GENMASK(1, 0)

/** AD4080_REG_TRANSFER_CONFIG Bit Definition */
#define AD4080_KEEP_STREAM_LENGTH_VAL_MSK	BIT(2)

/** AD4080_REG_INTERFACE_CONFIG_C Bit Definition */
#define AD4080_STRICT_REG_ACCESS_MSK		BIT(5)

/** AD4080_REG_ADC_DATA_INTF_CONFIG_A Bit Definition */
#define AD4080_RESERVED_CONFIG_A_MSK		BIT(6)
#define AD4080_INTF_CHK_EN_MSK			BIT(4)
#define AD4080_SPI_LVDS_LANES_MSK		BIT(2)
#define AD4080_DATA_INTF_MODE_MSK		BIT(0)

/** AD4080_REG_ADC_DATA_INTF_CONFIG_B Bit Definition */
#define AD4080_LVDS_CNV_CLK_CNT_MSK		GENMASK(7, 4)
#define AD4080_LVDS_SELF_CLK_MODE_MSK		BIT(3)
#define AD4080_LVDS_CNV_EN_MSK			BIT(0)

/** AD4080_REG_ADC_DATA_INTF_CONFIG_C Bit Definition */
#define AD4080_LVDS_VOD_MSK			GENMASK(6, 4)

/** AD4080_REG_PWR_CTRL Bit Definition */
#define AD4080_ANA_DIG_LDO_PD_MSK		BIT(1)
#define AD4080_INTF_LDO_PD_MSK			BIT(0)

/** AD4080_REG_GPIO_CONFIG_A Bit Definition */
#define AD4080_GPO_1_EN				BIT(1)
#define AD4080_GPO_0_EN				BIT(0)

/** AD4080_REG_GPIO_CONFIG_B Bit Definition */
#define AD4080_GPIO_1_SEL			GENMASK(7, 4)
#define AD4080_GPIO_0_SEL			GENMASK(3, 0)

/** AD4080_REG_FIFO_CONFIG Bit Definition */
#define AD4080_FIFO_MODE_MSK			GENMASK(1, 0)

/** AD4080_REG_FILTER_CONFIG Bit Definition */
#define AD4080_SINC_DEC_RATE_MSK		GENMASK(6, 3)
#define AD4080_FILTER_SEL_MSK			GENMASK(1, 0)

/** Miscellaneous Definitions */
#define AD4080_SW_RESET				(BIT(7) | BIT(0))
#define AD4080_SPI_READ				BIT(7)
#define BYTE_ADDR_H				GENMASK(15, 8)
#define BYTE_ADDR_L				GENMASK(7, 0)
#define AD4080_CHIP_ID				GENMASK(2, 0)

#define AD4080_MAX_SAMP_FREQ			40000000
#define AD4080_MIN_SAMP_FREQ			1250000

#define AXI_AD4080_ENABLE_FILTER_BIT		BIT(0)
#define AXI_AD4080_SELF_SYNC_BIT		BIT(1)

enum ad4080_filter_sel {
	FILTER_DISABLE,
	SINC_1,
	SINC_5,
	SINC_5_COMP
};

enum ad4080_dec_rate {
	DEC_2,
	DEC_4,
	DEC_8,
	DEC_16,
	DEC_32,
	DEC_64,
	DEC_128,
	DEC_256,
	DEC_512,
	DEC_1024
};

static const int ad4080_scale_table[][2] = {
	{6000, 0},
};

static const int ad4080_dec_rate_value[] = {
	[DEC_2]    = 2,
	[DEC_4]    = 4,
	[DEC_8]    = 8,
	[DEC_16]   = 16,
	[DEC_32]   = 32,
	[DEC_64]   = 64,
	[DEC_128]  = 128,
	[DEC_256]  = 256,
	[DEC_512]  = 512,
	[DEC_1024] = 1024,
};

static const char *const ad4080_filter_mode_iio_enum[] = {
	[FILTER_DISABLE]   = "FILTER_DISABLE",
	[SINC_1]           = "SINC_1",
	[SINC_5]           = "SINC_5",
	[SINC_5_COMP] = "SINC_5_+COMP",
};

static const char *const ad4080_dec_rate_iio_enum[] = {
	[DEC_2]    = "DEC_2",
	[DEC_4]    = "DEC_4",
	[DEC_8]    = "DEC_8",
	[DEC_16]   = "DEC_16",
	[DEC_32]   = "DEC_32",
	[DEC_64]   = "DEC_64",
	[DEC_128]  = "DEC_128",
	[DEC_256]  = "DEC_256",
	[DEC_512]  = "DEC_512",
	[DEC_1024] = "DEC_1024",
};

struct ad4080_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct clk			*clk;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex			lock;
	unsigned int			num_lanes;
	enum ad4080_dec_rate		dec_rate;
	enum ad4080_filter_sel		filter_mode;
	bool				filter_enabled;
};

static const struct regmap_config ad4080_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = 0x29,
};

static struct ad4080_state *ad4080_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
}

static int ad4080_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4080_state *st = ad4080_get_data(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4080_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned int tmp;

	tmp = (conv->chip_info->scale_table[0][0] * 1000000ULL) >>
		    conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000000;
	*val2 = tmp % 1000000000;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ad4080_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	int dec_rate;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ad4080_get_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->filter_mode == SINC_5_COMP)
			dec_rate = ad4080_dec_rate_value[st->dec_rate] * 2;
		else
			dec_rate = ad4080_dec_rate_value[st->dec_rate];
		if (st->filter_enabled)
			*val = DIV_ROUND_CLOSEST(clk_get_rate(st->clk), dec_rate);
		else
			*val = clk_get_rate(st->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4080_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	unsigned long s_clk;
	int dec_rate = 1;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return -EINVAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		s_clk = clk_round_rate(st->clk, val);

		if (st->filter_enabled) {
			if (st->filter_mode == SINC_5_COMP)
				dec_rate = ad4080_dec_rate_value[st->dec_rate] * 2;
			else
				dec_rate = ad4080_dec_rate_value[st->dec_rate];
		}

		s_clk *= dec_rate;

		if (s_clk < AD4080_MIN_SAMP_FREQ)
			s_clk = AD4080_MIN_SAMP_FREQ;
		if (s_clk > AD4080_MAX_SAMP_FREQ)
			s_clk = AD4080_MAX_SAMP_FREQ;

		return clk_set_rate(st->clk, s_clk);
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t ad4080_lvds_sync_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	return sprintf(buf, "enable\n");
}

static ssize_t ad4080_lvds_sync_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	unsigned int reg_cntrl, reg_cntrl_3;
	unsigned int timeout = 100;
	int ret;

	mutex_lock(&st->lock);
	if (st->num_lanes == 1)
		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
				   AD4080_RESERVED_CONFIG_A_MSK |
				   AD4080_INTF_CHK_EN_MSK);
	else
		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
				   AD4080_RESERVED_CONFIG_A_MSK |
				   AD4080_INTF_CHK_EN_MSK |
				   AD4080_SPI_LVDS_LANES_MSK);
	if (ret)
		return ret;

	// set bit 2 of ADI_REG_CNTRL_3 to let the HDL know that the CNV is not used
	reg_cntrl_3 = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
	reg_cntrl_3 |= AXI_AD4080_SELF_SYNC_BIT;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl_3);

	// enable the sync
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	reg_cntrl |= ADI_SYNC;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);

	do {
		if (axiadc_read(axi_adc_st, ADI_REG_SYNC_STATUS) == 0)
			dev_info(&st->spi->dev, "Not Locked: Running Bit Slip\n");
		else
			break;
	} while (--timeout);

	if (timeout) {
		dev_info(&st->spi->dev, "Success: Pattern correct and Locked!\n");
		if (st->num_lanes == 1)
			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
					   AD4080_RESERVED_CONFIG_A_MSK);
		else
			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
					   AD4080_RESERVED_CONFIG_A_MSK |
					   AD4080_SPI_LVDS_LANES_MSK);
	} else {
		dev_info(&conv->spi->dev, "LVDS Sync Timeout.\n");
		if (st->num_lanes == 1)
			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
					   AD4080_RESERVED_CONFIG_A_MSK);
		else
			ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_A,
					   AD4080_RESERVED_CONFIG_A_MSK |
					   AD4080_SPI_LVDS_LANES_MSK);
		ret = -ETIME;
	}

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad4080_lvds_cnv_en_read(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       char *buf)
{
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	unsigned int data;
	int ret;

	ret = regmap_read(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B, &data);
	if (ret < 0)
		return ret;

	data = FIELD_GET(AD4080_LVDS_CNV_EN_MSK, data);

	return ret < 0 ? ret : sysfs_emit(buf, "%u\n", data);
}

static ssize_t ad4080_lvds_cnv_en_write(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					const char *buf, size_t len)
{
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	bool en;
	unsigned int data;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = regmap_read(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B, &data);
	if (ret)
		goto mutex_unlock;

	data &= ~AD4080_LVDS_CNV_EN_MSK;
	data |= en;

	ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B, data);

mutex_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad4080_get_filter_mode(struct iio_dev *dev,
				      const struct iio_chan_spec *chan)
{
	struct ad4080_state *st = ad4080_get_data(dev);
	unsigned int data;
	int ret;

	ret = regmap_read(st->regmap, AD4080_REG_FILTER_CONFIG, &data);
	if (ret < 0)
		return ret;

	data = FIELD_GET(GENMASK(1, 0), data);

	return data;
}

static int ad4080_set_filter_mode(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int mode)
{
	struct ad4080_state *st = ad4080_get_data(dev);
	struct axiadc_state *axi_adc_st = iio_priv(dev);
	int ret;
	unsigned int data;
	unsigned int reg_val;
	unsigned int reg_cntrl;

	if (mode >= SINC_5 && st->dec_rate >= DEC_512)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (mode) {
		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B,
				   FIELD_PREP(AD4080_LVDS_CNV_CLK_CNT_MSK, 2) |
				   AD4080_LVDS_CNV_EN_MSK);
		if (ret)
			return ret;

		reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl | AXI_AD4080_ENABLE_FILTER_BIT);

		st->filter_enabled = true;
	} else {
		ret = regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B,
				   FIELD_PREP(AD4080_LVDS_CNV_CLK_CNT_MSK, 4) |
				   AD4080_LVDS_CNV_EN_MSK);
		if (ret)
			return ret;

		reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
		axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl & ~AXI_AD4080_ENABLE_FILTER_BIT);

		st->filter_enabled = false;
	}
	ret = regmap_read(st->regmap, AD4080_REG_FILTER_CONFIG, &reg_val);
	data = ((reg_val & AD4080_SINC_DEC_RATE_MSK)) |
	       (mode & AD4080_FILTER_SEL_MSK);
	ret = regmap_write(st->regmap, AD4080_REG_FILTER_CONFIG, data);
	st->filter_mode = mode;
	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t ad4080_get_dec_rate(struct iio_dev *dev,
				   const struct iio_chan_spec *chan)
{
	struct ad4080_state *st = ad4080_get_data(dev);
	int ret;
	unsigned int data;

	ret = regmap_read(st->regmap, AD4080_REG_FILTER_CONFIG, &data);
	if (ret)
		return ret;

	data = FIELD_GET(AD4080_SINC_DEC_RATE_MSK, data);

	return data;
}

static int ad4080_set_dec_rate(struct iio_dev *dev,
			       const struct iio_chan_spec *chan,
			       unsigned int mode)
{
	struct ad4080_state *st = ad4080_get_data(dev);
	int ret;
	unsigned int data;
	unsigned int reg_val;

	if (st->filter_mode >= SINC_5 && mode >= DEC_512)
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4080_REG_FILTER_CONFIG, &reg_val);
	if (ret)
		return ret;

	data = (mode << 3) | (reg_val & AD4080_FILTER_SEL_MSK);
	ret = regmap_write(st->regmap, AD4080_REG_FILTER_CONFIG, data);
	if (ret)
		return ret;

	st->dec_rate = mode;
	mutex_unlock(&st->lock);
	return ret;
}

static const struct iio_enum ad4080_dec_rate_enum = {
	.items = ad4080_dec_rate_iio_enum,
	.num_items = ARRAY_SIZE(ad4080_dec_rate_iio_enum),
	.set = ad4080_set_dec_rate,
	.get = ad4080_get_dec_rate,
};

static const struct iio_enum ad4080_filter_mode_enum = {
	.items = ad4080_filter_mode_iio_enum,
	.num_items = ARRAY_SIZE(ad4080_filter_mode_iio_enum),
	.set = ad4080_set_filter_mode,
	.get = ad4080_get_filter_mode,
};

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	{
	 .name = "lvds_sync",
	 .write = ad4080_lvds_sync_write,
	 .read = ad4080_lvds_sync_read,
	 },
	{
	 .name = "lvds_cnv",
	 .read = ad4080_lvds_cnv_en_read,
	 .write = ad4080_lvds_cnv_en_write,
	},
	IIO_ENUM("dec_rate",
		 IIO_SHARED_BY_ALL,
		 &ad4080_dec_rate_enum),
	IIO_ENUM_AVAILABLE("dec_rate",
			   IIO_SHARED_BY_ALL,
			   &ad4080_dec_rate_enum),
	IIO_ENUM("filter_mode",
		 IIO_SHARED_BY_ALL,
		 &ad4080_filter_mode_enum),
	IIO_ENUM_AVAILABLE("filter_mode",
			   IIO_SHARED_BY_ALL,
			   &ad4080_filter_mode_enum),
	{},
};

#define AD4080_CHAN(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),		\
	  .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 32,				\
			.shift = _shift,				\
	  },								\
	}

static const struct axiadc_chip_info ad4080_chip_info = {
	.name = "AD4080",
	.id = AD4080_CHIP_ID,
	.max_rate = 40000000UL,
	.scale_table = ad4080_scale_table,
	.num_scales = ARRAY_SIZE(ad4080_scale_table),
	.num_channels = 1,
	.channel[0] = AD4080_CHAN(0, 0, 20, 'S', 0),
};

static void ad4080_clk_disable(void *data)
{
	struct axiadc_converter *st = data;

	clk_disable_unprepare(st->clk);
}

static int ad4080_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct ad4080_state *st = ad4080_get_data(indio_dev);
	unsigned int reg_cntrl, reg_cntrl_3;

	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);

	reg_cntrl_3 = axiadc_read(axi_adc_st, ADI_REG_CNTRL_3);
	reg_cntrl_3 |= AXI_AD4080_SELF_SYNC_BIT;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL_3, reg_cntrl_3); // in the default mode the CNV is not used as a start of transfer flag

	return 0;
}

static int ad4080_setup(struct ad4080_state *st)
{
	unsigned int id;
	int ret;

	ret = regmap_write(st->regmap, AD4080_REG_INTERFACE_CONFIG_A,
			   AD4080_SW_RESET);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4080_REG_INTERFACE_CONFIG_A,
			   AD4080_SDO_ENABLE_MSK);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4080_REG_CHIP_TYPE, &id);
	if (ret)
		return ret;

	if (id != AD4080_CHIP_ID) {
		dev_err(&st->spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		return -EINVAL;
	}

	if (st->num_lanes)
		regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B,
			     FIELD_PREP(AD4080_LVDS_CNV_CLK_CNT_MSK, 4) |
			     AD4080_LVDS_CNV_EN_MSK);
	else
		regmap_write(st->regmap, AD4080_REG_ADC_DATA_INTF_CONFIG_B,
			     AD4080_LVDS_CNV_EN_MSK);

	ret = regmap_write(st->regmap, AD4080_REG_GPIO_CONFIG_A,
			   AD4080_GPO_1_EN | AD4080_GPO_0_EN); // GPO_1_EN and GPO_0_EN as output.
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4080_REG_GPIO_CONFIG_B,
			    FIELD_PREP(AD4080_GPIO_1_SEL, 3)); // GPIO_1_SEL to 0011: Filter Result Ready (Active Low).
}

static int ad4080_properties_parse(struct ad4080_state *st)
{
	struct spi_device *spi = st->spi;
	unsigned int val;
	int ret;

	st->clk = devm_clk_get(&spi->dev, "adc_clk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	ret = of_property_read_u32(spi->dev.of_node, "num_lanes", &val);
	if (!ret)
		st->num_lanes = val;
	else
		st->num_lanes = 1;

	return 0;
}

static int ad4080_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct axiadc_converter *conv;
	struct ad4080_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ad4080_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);
	st->regmap = regmap;
	st->spi = spi;

	mutex_init(&st->lock);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	ret = ad4080_properties_parse(st);
	if (ret)
		return ret;

	ret = clk_prepare_enable(st->clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad4080_clk_disable, st);
	if (ret)
		return ret;

	conv->spi = st->spi;
	conv->clk = st->clk;
	conv->chip_info = &ad4080_chip_info;
	conv->reg_access = ad4080_reg_access;
	conv->read_raw = ad4080_read_raw;
	conv->write_raw = ad4080_write_raw;
	conv->post_setup = ad4080_post_setup;
	conv->phy = st;

	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	return ad4080_setup(st);
}

static const struct spi_device_id ad4080_id[] = {
	{ "ad4080", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4080_id);

static const struct of_device_id ad4080_of_match[] = {
	{ .compatible = "adi,ad4080" },
	{},
};
MODULE_DEVICE_TABLE(of, ad4080_of_match);

static struct spi_driver ad4080_driver = {
	.driver = {
		.name = "ad4080",
		.of_match_table = ad4080_of_match,
	},
	.probe = ad4080_probe,
	.id_table = ad4080_id,
};
module_spi_driver(ad4080_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices AD4080");
MODULE_LICENSE("GPL");
