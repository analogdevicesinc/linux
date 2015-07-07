/*
 * HMC1044 SPI Digital Programmable Harmonic Low Pass Filter
 *
 * Copyright 2015 Analog Devices Inc.
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
#include <linux/bitrev.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

enum hmc1044_type {
	ID_HMC1044,
};

enum hmc1044_connection_type {
	SINGLE_ENDED,
	DIFFERENTIAL,
	NUM_CONNECTION_TYPES,
};

static const u16 hmc1044_cutoffs_3dB_MHz[][NUM_CONNECTION_TYPES] = {
	{1025, 970},
	{1050, 1000},
	{1075, 1030},
	{1105, 1055},
	{1130, 1085},
	{1160, 1120},
	{1195, 1155},
	{1225, 1195},
	{2230, 2335},
	{2300, 2430},
	{2380, 2530},
	{2465, 2655},
	{2550, 2770},
	{2675, 2940},
	{2805, 3145},
	{3060, 3400},
};

struct hmc1044_state {
	struct spi_device		*spi;
	struct regulator			*reg;
	unsigned char			ch[2];
	enum hmc1044_type		type;
	enum hmc1044_connection_type 	con_type;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data[2] ____cacheline_aligned;
};

static int hmc1044_get_index(u32 conn_type, u32 cutoff_frequency_Hz)
{
	int i;

	//cutoff_frequency_Hz /= 1000000UL;

	for (i = 0; i < ARRAY_SIZE(hmc1044_cutoffs_3dB_MHz); i++)
		if (hmc1044_cutoffs_3dB_MHz[i][conn_type] >= cutoff_frequency_Hz)
			return i;

	return 0;
}

static int hmc1044_write(struct iio_dev *indio_dev, unsigned reg, unsigned val)
{
	struct hmc1044_state *st = iio_priv(indio_dev);
	int ret;

	st->data[0] = val >> 1;
	st->data[1] = ((val & 1) << 7) | (reg << 3) | 0x6; /* Chip Address + Register */

	pr_err("%s: %d : 0x%X 0x%X\n", __func__,val,  st->data[0], st->data[1]);

	ret = spi_write(st->spi, st->data, 2);
	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int hmc1044_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct hmc1044_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);

	*val = hmc1044_cutoffs_3dB_MHz[st->ch[chan->channel]][st->con_type];
	ret = IIO_VAL_INT;;

	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int hmc1044_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct hmc1044_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);

	st->ch[chan->channel] = hmc1044_get_index(st->con_type, val);
	ret = hmc1044_write(indio_dev, 1, st->ch[chan->channel]);

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info hmc1044_info = {
	.read_raw = &hmc1044_read_raw,
	.write_raw = &hmc1044_write_raw,
	.driver_module = THIS_MODULE,
};

#define HMC1044_CHAN(_channel) {				\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = _channel,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),\
}

static const struct iio_chan_spec hmc1044_channels[] = {
	HMC1044_CHAN(0),
};

static int hmc1044_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct hmc1044_state *st;
	int ret;
	u32 tmp;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;

	/* try to get a unique name */
	if (spi->dev.platform_data)
		indio_dev->name = spi->dev.platform_data;
	else if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	st->type = spi_get_device_id(spi)->driver_data;
	switch (st->type) {
	case ID_HMC1044:
		indio_dev->channels = hmc1044_channels;
		indio_dev->num_channels = ARRAY_SIZE(hmc1044_channels);
		break;
	default:
		dev_err(&spi->dev, "Invalid device ID\n");
		return -EINVAL;
	}

	indio_dev->info = &hmc1044_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (of_property_read_bool(spi->dev.of_node,
		"adi,connection-type-single-ended-enable"))
		st->con_type = SINGLE_ENDED;
	else
		st->con_type = DIFFERENTIAL;

	tmp = 3400;
	of_property_read_u32(spi->dev.of_node,
			     "adi,default-cutoff-frequency-mhz", &tmp);

	st->ch[0] = hmc1044_get_index(st->con_type, tmp);
	ret = hmc1044_write(indio_dev, 1, st->ch[0]);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int hmc1044_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct hmc1044_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg))
		regulator_disable(reg);

	return 0;
}

static const struct spi_device_id hmc1044_id[] = {
	{"hmc1044", ID_HMC1044},
	{}
};

static struct spi_driver hmc1044_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
	},
	.probe		= hmc1044_probe,
	.remove		= hmc1044_remove,
	.id_table	= hmc1044_id,
};

module_spi_driver(hmc1044_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMC1044 LPF");
MODULE_LICENSE("GPL v2");
