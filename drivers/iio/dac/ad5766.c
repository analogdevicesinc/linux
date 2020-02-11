// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD5766, AD5767
 * Digital to Analog Converters driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#define AD5766_CMD_NOP_MUX_OUT			0x00
#define AD5766_CMD_SDO_CNTRL			0x01
#define AD5766_CMD_WR_IN_REG(x)			(0x10 | ((x) & 0xF))
#define AD5766_CMD_WR_DAC_REG(x)		(0x20 | ((x) & 0xF))
#define AD5766_CMD_SW_LDAC			0x30
#define AD5766_CMD_SPAN_REG			0x40
#define AD5766_CMD_WR_PWR_DITHER		0x51
#define AD5766_CMD_WR_DAC_REG_ALL		0x60
#define AD5766_CMD_SW_FULL_RESET		0x70
#define AD5766_CMD_READBACK_REG(x)		(0x80 | ((x) & 0xF))
#define AD5766_CMD_DITHER_SIG_1			0x90
#define AD5766_CMD_DITHER_SIG_2			0xA0
#define AD5766_CMD_INV_DITHER			0xB0
#define AD5766_CMD_DITHER_SCALE_1		0xC0
#define AD5766_CMD_DITHER_SCALE_2		0xD0

#define AD5766_FULL_RESET_CODE			0x1234

enum ad5766_type {
	ID_AD5766,
	ID_AD5767,
};

#define AD576x_CHANNEL(_chan, _bits) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_chan),					\
	.address = (_chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET) |	\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (_bits),				\
		.storagebits = 16,				\
		.shift = 16 - (_bits),				\
	},							\
}

#define DECLARE_AD576x_CHANNELS(_name, _bits)			\
const struct iio_chan_spec _name[] = {				\
	AD576x_CHANNEL(0, (_bits)),				\
	AD576x_CHANNEL(1, (_bits)),				\
	AD576x_CHANNEL(2, (_bits)),				\
	AD576x_CHANNEL(3, (_bits)),				\
	AD576x_CHANNEL(4, (_bits)),				\
	AD576x_CHANNEL(5, (_bits)),				\
	AD576x_CHANNEL(6, (_bits)),				\
	AD576x_CHANNEL(7, (_bits)),				\
	AD576x_CHANNEL(8, (_bits)),				\
	AD576x_CHANNEL(9, (_bits)),				\
	AD576x_CHANNEL(10, (_bits)),				\
	AD576x_CHANNEL(11, (_bits)),				\
	AD576x_CHANNEL(12, (_bits)),				\
	AD576x_CHANNEL(13, (_bits)),				\
	AD576x_CHANNEL(14, (_bits)),				\
	AD576x_CHANNEL(15, (_bits)),				\
}

/**
 * struct ad5766_chip_info - chip specific information
 * @num_channels:	number of channels
 * @channel:	        channel specification
 */

struct ad5766_chip_info {
	unsigned int			num_channels;
	const struct iio_chan_spec	*channels;
};

/**
 * struct ad5766_state - driver instance specific data
 * @spi:		Spi device
 * @lock:		Mutex lock
 * @chip_info:		Chip model specific constants
 * @data:		Spi transfer buffers
 */

struct ad5766_state {
	struct spi_device		*spi;
	struct mutex			lock;
	const struct ad5766_chip_info 	*chip_info;
	union {
		u32	d32;
		u16	w16[2];
		u8	b8[4];
	} data[3] ____cacheline_aligned;
};

static int ad5766_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info);

static int ad5766_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m);

static const struct iio_info ad5766_info = {
	.read_raw = ad5766_read_raw,
	.write_raw = ad5766_write_raw,
};

static DECLARE_AD576x_CHANNELS(ad5766_channels, 16);
static DECLARE_AD576x_CHANNELS(ad5767_channels, 12);

static const struct ad5766_chip_info ad5766_chip_infos[] = {
	[ID_AD5766] = {
		.num_channels = ARRAY_SIZE(ad5766_channels),
		.channels = ad5766_channels,
	},
	[ID_AD5767] = {
		.num_channels = ARRAY_SIZE(ad5767_channels),
		.channels = ad5767_channels,
	},
};

static void _ad5766_get_span_range(int *min, int *max)
{
	*min = -5;
	*max = 5;
}

static int _ad5766_spi_write(struct ad5766_state *st,
			     u8 command,
			     u16 data)
{
	st->data[0].b8[0] = command;
	st->data[0].b8[1] = (data & 0xFF00) >> 8;
	st->data[0].b8[2] = (data & 0x00FF) >> 0;

	return spi_write(st->spi, &st->data[0].b8[0], 3);
}

static int ad5766_write(struct iio_dev *indio_dev, u8 dac, u16 data)
{
	struct ad5766_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = _ad5766_spi_write(st, AD5766_CMD_WR_DAC_REG(dac), data);
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5766_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	const int max_val = (1 << chan->scan_type.realbits);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val >= max_val || val < 0)
			return -EINVAL;
		val <<= chan->scan_type.shift;
		break;
	default:
		return -EINVAL;
	}

	return ad5766_write(indio_dev, chan->address, val);
}

static int _ad5766_spi_read(struct ad5766_state *st, u8 dac, int *val)
{
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->data[0].d32,
			.bits_per_word = 8,
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d32,
			.rx_buf = &st->data[2].d32,
			.bits_per_word = 8,
			.len = 3,
		},
	};

	st->data[0].d32 = AD5766_CMD_READBACK_REG(dac);
	st->data[1].d32 = AD5766_CMD_NOP_MUX_OUT;

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));

	if (ret)
		return ret;

	*val = st->data[2].w16[1];

	return ret;
}

static int ad5766_read(struct iio_dev *indio_dev, u8 dac, int *val)
{
	struct ad5766_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = _ad5766_spi_read(st, dac, val);
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5766_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret, min, max;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5766_read(indio_dev, chan->address, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		_ad5766_get_span_range(&min, &max);
		*val = max - min;
		*val2 = indio_dev->channels->scan_type.realbits;
		
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_OFFSET:
		_ad5766_get_span_range(&min, &max);
		*val = min;
		
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad5766_setup(struct ad5766_state *st)
{
	return 0;
}

static int ad5766_probe(struct spi_device *spi)
{
	enum ad5766_type type = spi_get_device_id(spi)->driver_data;
	struct iio_dev *indio_dev;
	struct ad5766_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	mutex_init(&st->lock);

	st->spi = spi;
	st->chip_info = &ad5766_chip_infos[type];

	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad5766_info;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5766_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad5766_dt_match[] = {
	{ .compatible = "adi,ad5766" },
	{ .compatible = "adi,ad5767" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5766_dt_match);

static const struct spi_device_id ad5766_spi_ids[] = {
	{ "ad5766", ID_AD5766 },
	{ "ad5767", ID_AD5767 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5766_spi_ids);

static struct spi_driver ad5766_driver = {
	.driver = {
		.name = "ad5766",
		.of_match_table = ad5766_dt_match,
	},
	.probe = ad5766_probe,
	.id_table = ad5766_spi_ids,
};
module_spi_driver(ad5766_driver);

MODULE_AUTHOR("Denis-Gabriel Gheorghescu <denis.gheorghescu@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5766/AD5767 DACs");
MODULE_LICENSE("GPL v2");
