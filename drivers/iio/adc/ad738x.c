/*
 * Analog Devices AD738x Differential Input, Simultaneous Sampling,
 * 16/14/12-BIT, SAR ADC driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/bitops.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/*
 * AD738X registers definition
 */
#define AD738X_REG_NOP			0x00
#define AD738X_REG_CONFIG1		0x01
#define AD738X_REG_CONFIG2		0x02
#define AD738X_REG_ALERT		0x03
#define AD738X_REG_ALERT_LOW_TH		0x04
#define AD738X_REG_ALERT_HIGH_TH	0x05

/*
 * AD738X_REG_CONFIG1
 */
#define AD738X_CONFIG1_OS_MODE_MSK	BIT(9)
#define AD738X_CONFIG1_OS_MODE(x)	(((x) & 0x1) << 9)
#define AD738X_CONFIG1_OSR_MSK		GENMASK(8, 6)
#define AD738X_CONFIG1_OSR(x)		(((x) & 0x7) << 6)
#define AD738X_CONFIG1_CRC_W_MSK	BIT(5)
#define AD738X_CONFIG1_CRC_W(x)		(((x) & 0x1) << 5)
#define AD738X_CONFIG1_CRC_R_MSK	BIT(4)
#define AD738X_CONFIG1_CRC_R(x)		(((x) & 0x1) << 4)
#define AD738X_CONFIG1_ALERTEN_MSK	BIT(3)
#define AD738X_CONFIG1_ALERTEN(x)	(((x) & 0x1) << 3)
#define AD738X_CONFIG1_RES_MSK		BIT(2)
#define AD738X_CONFIG1_RES(x)		(((x) & 0x1) << 2)
#define AD738X_CONFIG1_REFSEL_MSK	BIT(1)
#define AD738X_CONFIG1_REFSEL(x)	(((x) & 0x1) << 1)
#define AD738X_CONFIG1_PMODE_MSK	BIT(0)
#define AD738X_CONFIG1_PMODE(x)		(((x) & 0x1) << 0)

/*
 * AD738X_REG_CONFIG2
 */
#define AD738X_CONFIG2_SDO2_MSK		BIT(8)
#define AD738X_CONFIG2_SDO2(x)		(((x) & 0x1) << 8)
#define AD738X_CONFIG2_SDO4_MSK		GENMASK(9, 8)
#define AD738X_CONFIG2_SDO4(x)		(((x) & 0x3) << 8)
#define AD738X_CONFIG2_RESET_MSK	GENMASK(7, 0)
#define AD738X_CONFIG2_RESET(x)      	(((x) & 0xFF) << 0)

/*
 * AD738X_REG_ALERT_LOW_TH
 */
#define AD738X_ALERT_LOW_MSK		GENMASK(11, 0)
#define AD738X_ALERT_LOW(x) 		(((x) & 0xFFF) << 0)

/*
 * AD738X_REG_ALERT_HIGH_TH
 */
#define AD738X_ALERT_HIGH_MSK		GENMASK(11, 0)
#define AD738X_ALERT_HIGH(x) 		(((x) & 0xFFF) << 0)

/* Write to register x */
#define AD738X_REG_WRITE(x)		((1 << 7) | ((x & 0x07) << 4))

enum ad738x_ids {
	ID_7380,
	ID_7381,
	ID_7382,
	ID_7383,
	ID_7384,
	ID_7385,
	ID_7386,
	ID_7387,
	ID_7388,
	ID_7380_4,
	ID_7381_4,
	ID_7382_4,
	ID_7383_4,
	ID_7384_4,
	ID_7385_4,
	ID_7386_4,
	ID_7387_4,
	ID_7388_4
};

struct ad738x_chip_info {
	struct iio_chan_spec	*channels;
	unsigned int 		num_channels;
};

#define AD738X_CHAN(index, bits)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = (bits),				\
			.storagebits = bits,				\
			.endianness = IIO_BE,				\
		},							\
	}

#define DECLARE_AD738X_CHANNELS(name, bits) 	\
static struct iio_chan_spec name[] = { 		\
		AD738X_CHAN(0, bits), 		\
		AD738X_CHAN(1, bits), 		\
}

#define DECLARE_AD738X_4_CHANNELS(name, bits) \
static struct iio_chan_spec name[] = { \
		AD738X_CHAN(0, bits), \
		AD738X_CHAN(1, bits), \
		AD738X_CHAN(2, bits), \
		AD738X_CHAN(3, bits), \
}

DECLARE_AD738X_CHANNELS(ad7380_channels, 16);
DECLARE_AD738X_CHANNELS(ad7381_channels, 14);
DECLARE_AD738X_CHANNELS(ad7382_channels, 12);
DECLARE_AD738X_CHANNELS(ad7383_channels, 16);
DECLARE_AD738X_CHANNELS(ad7384_channels, 14);
DECLARE_AD738X_CHANNELS(ad7385_channels, 12);
DECLARE_AD738X_CHANNELS(ad7386_channels, 16);
DECLARE_AD738X_CHANNELS(ad7387_channels, 14);
DECLARE_AD738X_CHANNELS(ad7388_channels, 12);
DECLARE_AD738X_4_CHANNELS(ad7380_4_channels, 16);
DECLARE_AD738X_4_CHANNELS(ad7381_4_channels, 14);
DECLARE_AD738X_4_CHANNELS(ad7382_4_channels, 12);
DECLARE_AD738X_4_CHANNELS(ad7383_4_channels, 16);
DECLARE_AD738X_4_CHANNELS(ad7384_4_channels, 14);
DECLARE_AD738X_4_CHANNELS(ad7385_4_channels, 12);
DECLARE_AD738X_4_CHANNELS(ad7386_4_channels, 16);
DECLARE_AD738X_4_CHANNELS(ad7387_4_channels, 14);
DECLARE_AD738X_4_CHANNELS(ad7388_4_channels, 12);

static struct ad738x_chip_info ad738x_chip_info_tbl[] = {
	[ID_7380] = {
		.channels = ad7380_channels,
		.num_channels = ARRAY_SIZE(ad7380_channels),
	},
	[ID_7381] = {
		.channels = ad7381_channels,
		.num_channels = ARRAY_SIZE(ad7381_channels),
	},
	[ID_7382] = {
		.channels = ad7382_channels,
		.num_channels = ARRAY_SIZE(ad7382_channels),
	},
	[ID_7383] = {
		.channels = ad7383_channels,
		.num_channels = ARRAY_SIZE(ad7383_channels),
	},
	[ID_7384] = {
		.channels = ad7384_channels,
		.num_channels = ARRAY_SIZE(ad7384_channels),
	},
	[ID_7385] = {
		.channels = ad7385_channels,
		.num_channels = ARRAY_SIZE(ad7385_channels),
	},
	[ID_7386] = {
		.channels = ad7386_channels,
		.num_channels = ARRAY_SIZE(ad7386_channels),
	},
	[ID_7387] = {
		.channels = ad7387_channels,
		.num_channels = ARRAY_SIZE(ad7387_channels),
	},
	[ID_7388] = {
		.channels = ad7388_channels,
		.num_channels = ARRAY_SIZE(ad7388_channels),
	},
	[ID_7380_4] = {
		.channels = ad7380_4_channels,
		.num_channels = ARRAY_SIZE(ad7380_4_channels),
	},
	[ID_7381_4] = {
		.channels = ad7381_4_channels,
		.num_channels = ARRAY_SIZE(ad7381_4_channels),
	},
	[ID_7382_4] = {
		.channels = ad7382_4_channels,
		.num_channels = ARRAY_SIZE(ad7382_4_channels),
	},
	[ID_7383_4] = {
		.channels = ad7383_4_channels,
		.num_channels = ARRAY_SIZE(ad7383_4_channels),
	},
	[ID_7384_4] = {
		.channels = ad7384_4_channels,
		.num_channels = ARRAY_SIZE(ad7384_4_channels),
	},
	[ID_7385_4] = {
		.channels = ad7385_4_channels,
		.num_channels = ARRAY_SIZE(ad7385_4_channels),
	},
	[ID_7386_4] = {
		.channels = ad7386_4_channels,
		.num_channels = ARRAY_SIZE(ad7386_4_channels),
	},
	[ID_7387_4] = {
		.channels = ad7387_4_channels,
		.num_channels = ARRAY_SIZE(ad7387_4_channels),
	},
	[ID_7388_4] = {
		.channels = ad7388_4_channels,
		.num_channels = ARRAY_SIZE(ad7388_4_channels),
	},
};

struct ad738x_state {
	struct spi_device		*spi;
	struct spi_transfer		t[5];
	struct regulator		*vref;
	const struct ad738x_chip_info 	*chip_info;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * Make the buffer large enough for one 64 bit sample and one 64 bit
	 * aligned 64 bit timestamp.
	 */
	unsigned char data[ALIGN(8, sizeof(s64)) + sizeof(s64)]
			____cacheline_aligned;
};

static int ad738x_spi_reg_read(struct ad738x_state *st,
			       u8 reg_addr,
			       u8 *reg_data)
{
	u8 tx_buf[2];
	u8 rx_buf[2];
	u8 n_tx = 2;
	u8 n_rx = 2;
	int ret;

	tx_buf[0] = (reg_addr & 0x07) << 4;
	tx_buf[1] = 0x00;

	/*
	 * A register read is performed by issuing a register read
	 * command followed by an additional SPI command
	 */
	ret = spi_write_then_read(st->spi, &tx_buf[0], n_tx, NULL, 0);
	ret |= spi_write_then_read(st->spi, NULL, 0, &rx_buf[0], n_rx);
	if (ret < 0) {
		dev_err(&st->spi->dev, "Reg Read Error %d", ret);
		return ret;
	}

	reg_data[0] = rx_buf[0];
	reg_data[1] = rx_buf[1];

	return ret;
}

static int ad738x_spi_reg_write(struct ad738x_state *st,
			        u8 reg_addr,
			        u32 reg_data)
{
	u8 tx_buf[4];
	u8 n_tx = 2;
	int ret;

	tx_buf[0] = AD738X_REG_WRITE(reg_addr) | ((reg_data & 0xF00) >> 8);
	tx_buf[1] = (reg_data & 0xFF);
	tx_buf[2] = 0x00;
	tx_buf[3] = 0x00;

	ret = spi_write_then_read(st->spi, &tx_buf[0], n_tx, NULL, 0);
	ret |= spi_write_then_read(st->spi, &tx_buf[2], n_tx, NULL, 0);

	return ret;
}

static int ad738x_spi_write_mask(struct ad738x_state *st,
			         u8 reg_addr,
			         u16 mask,
			         u16 data)
{
	u8 spi_buf[2];
	u16 reg_data;
	int ret;

	ret = ad738x_spi_reg_read(st, reg_addr, spi_buf);
	reg_data = (spi_buf[0] << 8) | spi_buf[1];
	reg_data &= ~mask;
	reg_data |= data;
	ret |= ad738x_spi_reg_write(st, reg_addr, reg_data);

	return ret;
}

static int ad738x_reg_access(struct iio_dev *indio_dev,
			     u32 reg, u32 writeval,
			     u32 *readval)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 reg_data[2];
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad738x_spi_reg_write(st, reg, writeval);
	} else {
		ad738x_spi_reg_read(st, reg, reg_data);
		*readval = (reg_data[0] << 8) | reg_data[1];
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ad738x_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *active_scan_mask)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 num_bytes = st->chip_info->num_channels * 2;
	int i;
	int j = 0;

	st->t[0].tx_buf = &st->data[0];
	st->t[0].len = num_bytes;
	st->t[0].cs_change = 1;

	/* Find which channels are active and overwrite the data
	   for the disabled channels*/
	for (i = 0; i < st->chip_info->num_channels; i++) {
		st->t[i+1].rx_buf = &st->data[j];
		st->t[i+1].len = 2;
		if(test_bit(i, active_scan_mask))
			j+=2;
	}

	return 0;
}

static irqreturn_t ad738x_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 num_bytes = st->chip_info->num_channels * 2;
	int i;
	int ret;

	/* Send NOP */
	for (i = 0; i < num_bytes; i++)
		st->data[i] = cpu_to_be16(AD738X_REG_NOP);

	ret = spi_sync_transfer(st->spi, st->t, ARRAY_SIZE(st->t));

	if (ret == 0) {
		iio_push_to_buffers_with_timestamp(indio_dev, st->data,
						   iio_get_time_ns(indio_dev));
	}
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ad738x_scan_direct(struct ad738x_state *st)
{
	u8 num_bytes = st->chip_info->num_channels * 2;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0],
			.len = num_bytes,
			.cs_change = 1,
		}, {
			.rx_buf = &st->data[0],
			.len = num_bytes,
		},
	};
	int i;
	int ret;

	for (i = 0; i < num_bytes; i++)
		st->data[i] = cpu_to_be16(AD738X_REG_NOP);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

	if (ret < 0)
		return ret;

	ret = num_bytes;

	return ret;
}

static int ad738x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad738x_state *st = iio_priv(indio_dev);
	u8 index = 2 * chan->scan_index;
	u8 buf[8];
	int scale_uv;
	int i;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		if (iio_buffer_enabled(indio_dev))
			ret = -EBUSY;
		else
			ret = ad738x_scan_direct(st);
		mutex_unlock(&indio_dev->mlock);
		if (ret < 0)
			return ret;

		for (i = 0; i < ret; i++)
			buf[i] = st->data[i];

		*val = (buf[index] << 8) | buf[index+1];

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		scale_uv = regulator_get_voltage(st->vref);
		if (scale_uv < 0)
			return scale_uv;

		*val = scale_uv * 2 / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

static const struct iio_info ad738x_info = {
	.read_raw = &ad738x_read_raw,
	.debugfs_reg_access = &ad738x_reg_access,
	.update_scan_mode = ad738x_update_scan_mode,
};

static int ad738x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad738x_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->chip_info =
		&ad738x_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);
	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad738x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 &ad738x_trigger_handler, NULL);
	if (ret)
		goto error_disable_vref;

	/* 1-wire mode */
	ret = ad738x_spi_write_mask(st,
				    AD738X_REG_CONFIG2,
				    AD738X_CONFIG2_SDO2_MSK,
				    AD738X_CONFIG2_SDO2(1));
	if (ret < 0)
		goto error_buffer_cleanup;

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	return 0;

error_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
error_disable_vref:
	regulator_disable(st->vref);

	return ret;
}

static int ad738x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad738x_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad738x_id[] = {
	{ "ad7380", ID_7380 },
	{ "ad7381", ID_7381 },
	{ "ad7382", ID_7382 },
	{ "ad7383", ID_7383 },
	{ "ad7384", ID_7384 },
	{ "ad7385", ID_7385 },
	{ "ad7386", ID_7386 },
	{ "ad7387", ID_7387 },
	{ "ad7388", ID_7388 },
	{ "ad7380-4", ID_7380_4 },
	{ "ad7381-4", ID_7381_4 },
	{ "ad7382-4", ID_7382_4 },
	{ "ad7383-4", ID_7383_4 },
	{ "ad7384-4", ID_7384_4 },
	{ "ad7385-4", ID_7385_4 },
	{ "ad7386-4", ID_7386_4 },
	{ "ad7387-4", ID_7387_4 },
	{ "ad7388-4", ID_7388_4 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad738x_id);

static struct spi_driver ad738x_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad738x_probe,
	.remove = ad738x_remove,
	.id_table = ad738x_id,
};
module_spi_driver(ad738x_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD738x ADC driver");
MODULE_LICENSE("GPL v2");
