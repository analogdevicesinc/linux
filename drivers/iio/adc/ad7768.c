/*
 * Analog Devices AD7768-1 SPI ADC driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/ad_sigma_delta.h>

/*
 * AD7768 registers definition
 */
#define	AD7768_REG_CHIP_TYPE 		0x3
#define	AD7768_REG_PROD_ID_L		0x4
#define	AD7768_REG_PROD_ID_H		0x5
#define	AD7768_REG_CHIP_GRADE		0x6
#define	AD7768_REG_SCRATCH_PAD		0x0A
#define	AD7768_REG_VENDOR_L		0x0C
#define	AD7768_REG_VENDOR_H		0x0D
#define	AD7768_REG_INTERFACE_FORMAT	0x14
#define AD7768_REG_POWER_CLOCK		0x15
#define AD7768_REG_ANALOG		0x16
#define AD7768_REG_ANALOG2		0x17
#define AD7768_REG_CONVERSION		0x18
#define AD7768_REG_DIGITAL_FILTER	0x19
#define AD7768_REG_SINC3_DEC_RATE_MSB	0x1A
#define AD7768_REG_SINC3_DEC_RATE_LSB	0x1B
#define AD7768_REG_DUTY_CYCLE_RATIO	0x1C
#define AD7768_REG_SYNC_RESET		0x1D
#define AD7768_REG_GPIO_CONTROL		0x1E
#define AD7768_REG_GPIO_WRITE		0x1F
#define AD7768_REG_GPIO_READ		0x20
#define AD7768_REG_OFFSET_HI		0x21
#define AD7768_REG_OFFSET_MID		0x22
#define AD7768_REG_OFFSET_LO		0x23
#define AD7768_REG_GAIN_HI		0x24
#define AD7768_REG_GAIN_MID		0x25
#define AD7768_REG_GAIN_LO		0x26
#define AD7768_REG_SPI_DIAG_ENABLE	0x28
#define AD7768_REG_ADC_DIAG_ENABLE	0x29
#define AD7768_REG_DIG_DIAG_ENABLE	0x2A
#define	AD7768_REG_ADC_DATA		0x2C
#define	AD7768_REG_MASTER_STATUS	0x2D
#define	AD7768_REG_SPI_DIAG_STATUS	0x2E
#define	AD7768_REG_ADC_DIAG_STATUS	0x2F
#define	AD7768_REG_DIG_DIAG_STATUS	0x30
#define	AD7768_REG_MCLK_COUNTER		0x31

/*
 * AD7768_REG_CONVERSION
 */
#define AD7768_CONVERSION_MODE_MSK	(0x7 << 0)
#define AD7768_CONVERSION_MODE(x) 	(((x) & 0x7) << 0)

/* ID Register Bit Designations (AD7768_REG_PROD_ID_L) */
#define AD7768_1_ID			0x01

static const struct iio_chan_spec ad7768_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 24,
			.storagebits = 32,
			.shift = 0,
			.endianness = IIO_BE,
		},
	},
};

struct ad7768_chip_info {
	unsigned int id;
};

struct ad7768_state {
	const struct ad7768_chip_info	*chip_info;
	struct regulator		*vref;
	struct ad_sigma_delta 		sd;
	u16				mode;
};

enum ad7768_supported_device_ids {
	ID_AD7768_1,
};

static struct ad7768_chip_info ad7768_chip_info_tbl[] = {
	[ID_AD7768_1] = {
		.id = AD7768_1_ID,
	},
};

static struct ad7768_state *ad_sigma_delta_to_ad7768(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct ad7768_state, sd);
}

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     u32 reg, u32 writeval,
			     u32 *readval)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad_sd_write_reg(&st->sd, reg, 1, writeval);
	} else {
		ad_sd_read_reg(&st->sd, reg, 1, readval);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ad7768_set_mode(struct ad_sigma_delta *sd,
			   enum ad_sigma_delta_mode mode)
{
	struct ad7768_state *st = ad_sigma_delta_to_ad7768(sd);

	st->mode &= ~AD7768_CONVERSION_MODE_MSK;
	st->mode |= AD7768_CONVERSION_MODE(mode);

	return ad_sd_write_reg(&st->sd, AD7768_REG_CONVERSION, 1, st->mode);
}

static const struct ad_sigma_delta_info ad7768_sigma_delta_info = {
	.has_registers = true,
	.data_reg = AD7768_REG_ADC_DATA,
	.addr_shift = 0,
	.read_mask = BIT(6),
};

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int scale_uv;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ad7768_set_mode(&st->sd, AD_SD_MODE_SINGLE);
		ret = ad_sigma_delta_single_conversion(indio_dev, chan, val);
		if (ret < 0)
			return ret;

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

static const struct iio_info ad7768_info = {
	.read_raw = &ad7768_read_raw,
	.debugfs_reg_access = &ad7768_reg_access,
	.driver_module = THIS_MODULE,
};

static int ad7768_probe(struct spi_device *spi)
{
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	u32 id;
	int ret;

	if (!spi->irq) {
		dev_err(&spi->dev, "No IRQ specified\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info =
		&ad7768_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	ad_sd_init(&st->sd, indio_dev, spi, &ad7768_sigma_delta_info);

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	spi_set_drvdata(spi, indio_dev);

	indio_dev->channels = ad7768_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7768_channels);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad7768_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad_sd_setup_buffer_and_trigger(indio_dev);
	if (ret)
		goto error_disable_vref;

	/* read test for device presence */
	ret = ad_sd_read_reg(&st->sd, AD7768_REG_PROD_ID_L, 1, &id);
	if (ret)
		goto error_remove_trigger;

	id &= AD7768_1_ID;

	if (id != st->chip_info->id) {
		dev_err(&st->sd.spi->dev, "device ID query failed\n");
		goto error_remove_trigger;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;

	return 0;

error_remove_trigger:
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
error_disable_vref:
	regulator_disable(st->vref);

	return ret;
}

static int ad7768_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7768_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad7768_id[] = {
	{ "ad7768-1", ID_AD7768_1 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad7768_id);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad7768_probe,
	.remove = ad7768_remove,
	.id_table = ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768-1 ADC driver");
MODULE_LICENSE("GPL v2");
