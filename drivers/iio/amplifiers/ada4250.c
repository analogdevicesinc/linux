// SPDX-License-Identifier: GPL-2.0+
/*
 * ADA4250 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

/* ADA4250 Register Map */
#define ADA4250_REG_GAIN_MUX        0x00
#define ADA4250_REG_REFBUF_EN       0x01
#define ADA4250_REG_RESET           0x02
#define ADA4250_REG_SNSR_CAL_VAL    0x04
#define ADA4250_REG_SNSR_CAL_CNFG   0x05
#define ADA4250_REG_DIE_REV         0x18
#define ADA4250_REG_CHIP_ID1        0x19
#define ADA4250_REG_CHIP_ID2        0x1a

/* ADA4250_REG_GAIN_MUX Map */
#define ADA4250_GAIN_MUX_MSK        GENMASK(2, 0)
#define ADA4250_GAIN_MUX(x)         FIELD_PREP(ADA4250_GAIN_MUX_MSK, x)

/* ADA4250_REG_REFBUF Map */
#define ADA4250_REFBUF_MSK          BIT(0)
#define ADA4250_REFBUF(x)           FIELD_PREP(ADA4250_REFBUF_MSK, x)

/* ADA4250_REG_RESET Map */
#define ADA4250_RESET_MSK           BIT(0)
#define ADA4250_RESET(x)            FIELD_PREP(ADA4250_RESET_MSK, x)

/* ADA4250_REG_SNSR_CAL_VAL Map */
#define ADA4250_SNSR_CAL_VAL_MSK    GENMASK(7, 0)
#define ADA4250_SNSR_CAL_VAL(x)     FIELD_PREP(ADA4250_SNSR_CAL_VAL_MSK, x)

/* ADA4250_REG_SNSR_CAL_CNFG Bit Definition */
#define ADA4250_BIAS_SET_MSK        GENMASK(3, 2)
#define ADA4250_BIAS_SET(x)         FIELD_PREP(ADA4250_BIAS_SET_MSK, x)
#define ADA4250_RANGE_SET_MSK       GENMASK(1, 0)
#define ADA4250_RANGE_SET(x)        FIELD_PREP(ADA4250_RANGE_SET_MSK, x)

#define ADA4250_CHIP_ID             0x4250

enum supported_parts {
	ADA4250,
};

enum ada4250_bias {
	ADA4250_BIAS_DISABLE,
	ADA4250_BIAS_BANDGAP_REF,
	ADA4250_BIAS_AVDD
};

enum ada4250_offset_range {
	ADA4250_RANGE1,
	ADA4250_RANGE2,
	ADA4250_RANGE3,
	ADA4250_RANGE4,
};

enum ada4250_gain {
	ADA4250_GAIN_1,
	ADA4250_GAIN_2,
	ADA4250_GAIN_4,
	ADA4250_GAIN_8,
	ADA4250_GAIN_16,
	ADA4250_GAIN_32,
	ADA4250_GAIN_64,
	ADA4250_GAIN_128,
};

struct ada4250 {
	struct regmap		*regmap;
	enum ada4250_bias	bias;
	enum ada4250_gain	gain;
	struct regulator	*reg;
	int			offset_uv;
};

static const int calibbias_table[] = {
	ADA4250_BIAS_DISABLE,
	ADA4250_BIAS_BANDGAP_REF,
	ADA4250_BIAS_AVDD,
};

static const struct regmap_config ada4250_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = 0x1A,
};

static int ada4250_set_offset(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   int offset)
{
	struct ada4250 *dev = iio_priv(indio_dev);

	int i, ret, x[8], vlsb, max_vos, min_vos, voltage_v;
	u8 offset_raw;
	enum ada4250_offset_range range;

	if (dev->bias == ADA4250_BIAS_DISABLE)
		return -EINVAL;

	voltage_v = regulator_get_voltage(dev->reg);
	voltage_v = DIV_ROUND_CLOSEST(voltage_v, 1000000);

	x[0] = (dev->bias == ADA4250_BIAS_AVDD) ? voltage_v : 5;

	x[1] = 126 * (x[0] - 1);
	x[2] = DIV_ROUND_CLOSEST(x[1] * 1000, 1333);
	x[3] = DIV_ROUND_CLOSEST(x[1] * 1000, 2301);
	x[4] = DIV_ROUND_CLOSEST(x[1] * 1000, 4283);
	x[5] = DIV_ROUND_CLOSEST(x[1] * 1000, 8289);
	x[6] = DIV_ROUND_CLOSEST(x[1] * 1000, 16311);
	x[7] = DIV_ROUND_CLOSEST(x[1] * 1000, 31599);

	if (dev->gain != ADA4250_GAIN_1) {
		for (i = ADA4250_RANGE1; i <= ADA4250_RANGE4; i++) {
			max_vos = x[dev->gain] *  127 * ((1 << (i + 1)) - 1);
			min_vos = (-1) * max_vos;
			if(offset > min_vos && offset < max_vos) {
				range = i;
				vlsb = x[dev->gain] * ((1 << (i + 1)) - 1);
				break;
			}
		}
	} else
		return -EINVAL;

	offset_raw = DIV_ROUND_CLOSEST(abs(offset), vlsb);

	ret = regmap_update_bits(dev->regmap, ADA4250_REG_SNSR_CAL_CNFG,
		ADA4250_RANGE_SET_MSK, ADA4250_RANGE_SET(range));
	if (ret < 0)
		return ret;

	dev->offset_uv = offset_raw * vlsb;

	if (offset < 0) {
		offset_raw |= 1 << 8;
		dev->offset_uv *= (-1);
	}

	return regmap_write(dev->regmap, ADA4250_REG_SNSR_CAL_VAL, offset_raw));
}

static int ada4250_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ada4250 *dev = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = regmap_read(dev->regmap, ADA4250_REG_GAIN_MUX, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:

		*val = dev->offset_uv;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = regmap_read(dev->regmap, ADA4250_REG_SNSR_CAL_CNFG, val);
		if (ret < 0)
			return ret;

		*val = *val >> 2;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ada4250_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ada4250 *dev = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = regmap_write(dev->regmap, ADA4250_REG_GAIN_MUX,
			ADA4250_GAIN_MUX(val));
		if (ret < 0)
			return ret;
		dev->gain = val;

		return ret;
	case IIO_CHAN_INFO_OFFSET:
		return ada4250_set_offset(indio_dev, chan, val);

	case IIO_CHAN_INFO_CALIBBIAS:
		ret = regmap_update_bits(dev->regmap, ADA4250_REG_SNSR_CAL_CNFG,
			ADA4250_BIAS_SET_MSK, ADA4250_BIAS_SET(val));
		if (ret < 0)
			return ret;

		dev->bias = val;

		return ret;

	default:
		return -EINVAL;
	}

	return 0;
}


static int ada4250_read_avail(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       const int **vals, int *type, int *length,
			       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		*vals = (const int *)calibbias_table;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(calibbias_table);

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ada4250_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int write_val,
	unsigned int *read_val)
{
	struct ada4250 *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info ada4250_info = {
	.read_raw = ada4250_read_raw,
	.write_raw = ada4250_write_raw,
	.read_avail = &ada4250_read_avail,
	.debugfs_reg_access = &ada4250_reg_access,
};

static ssize_t ada4250_read_refbuf_en(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  char *buf)
{
	struct ada4250 *dev = iio_priv(indio_dev);

	int ret;
	unsigned int val;

	ret = regmap_read(dev->regmap, ADA4250_REG_REFBUF_EN, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static ssize_t ada4250_write_refbuf_en(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   const char *buf, size_t len)
{
	struct ada4250 *dev = iio_priv(indio_dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(dev->regmap, ADA4250_REG_REFBUF_EN, ADA4250_REFBUF_MSK,
				 ADA4250_REFBUF(val));
	if(ret < 0)
		return ret;

	return len;
}

static const struct iio_chan_spec_ext_info ada4250_ext_info[] = {
	{
		.name = "refbuf",
		.read = ada4250_read_refbuf_en,
		.write = ada4250_write_refbuf_en,
		.shared = IIO_SEPARATE,
	},
	{ }
};

#define ADA4250_CHAN(_channel) {				\
	.type = IIO_VOLTAGE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_OFFSET) | 			\
		BIT(IIO_CHAN_INFO_CALIBBIAS), 			\
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_CALIBBIAS), \
	.ext_info = ada4250_ext_info,				\
}

static const struct iio_chan_spec ada4250_channels[] = {
	ADA4250_CHAN(0),
};

static int ada4250_soft_reset(struct ada4250 *dev)
{
	return regmap_write(dev->regmap, ADA4250_REG_RESET,
		ADA4250_RESET(1));
}

static int ada4250_init(struct ada4250 *dev)
{
	int ret;
	u16 chip_id;
	u32 data;

	ret = ada4250_soft_reset(dev);
	if (ret < 0)
		return ret;

	ret = regmap_read(dev->regmap, ADA4250_REG_CHIP_ID2, &data);
	if (ret < 0)
		return ret;

	chip_id = data;

	ret = regmap_read(dev->regmap, ADA4250_REG_CHIP_ID1, &data);
	if (ret < 0)
		return ret;

	chip_id = (chip_id << 8 | data);

	if (chip_id != ADA4250_CHIP_ID)
		return -EINVAL;

	return ret;
}

static int ada4250_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct ada4250 *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ada4250_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	spi_set_drvdata(spi, indio_dev);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	dev->reg = devm_regulator_get(&spi->dev, "avdd");
	if (IS_ERR(dev->reg))
		return PTR_ERR(dev->reg);

	ret = regulator_enable(dev->reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable specified AVDD supply\n");
		return ret;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ada4250_info;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->channels = ada4250_channels;
	indio_dev->num_channels = ARRAY_SIZE(ada4250_channels);

	ret = ada4250_init(dev);
	if (ret < 0) {
		dev_err(&spi->dev, "ADA4250 init failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);

	dev_info(&spi->dev, "%s probed\n", indio_dev->name);

	return ret;
}

static int ada4250_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ada4250 *dev = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	regulator_disable(dev->reg);

	return 0;
}

static const struct spi_device_id ada4250_id[] = {
	{ "ada4250", ADA4250 },
	{}
};
MODULE_DEVICE_TABLE(spi, ada4250_id);

static const struct of_device_id ada4250_of_match[] = {
	{ .compatible = "adi,ada4250" },
	{},
};
MODULE_DEVICE_TABLE(of, ada4250_of_match);

static struct spi_driver ada4250_driver = {
	.driver = {
			.name = "ada4250",
			.of_match_table = of_match_ptr(ada4250_of_match),
		},
	.probe = ada4250_probe,
	.remove = ada4250_remove,
	.id_table = ada4250_id,
};
module_spi_driver(ada4250_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADA4250");
MODULE_LICENSE("GPL v2");
