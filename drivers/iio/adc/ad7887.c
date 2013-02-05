/*
 * AD7887, AD7888 SPI ADC driver
 *
 * Copyright 2010-2014 Analog Devices Inc.
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
#include <linux/interrupt.h>
#include <linux/bitops.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/platform_data/ad7887.h>

#define AD7887_REF_DIS		BIT(5)	/* on-chip reference disable */
#define AD7887_DUAL		BIT(4)	/* dual-channel mode */
#define AD7887_CH(x)		((x) << 3)
#define AD7887_PM_MODE1		(0)	 /* CS based shutdown */
#define AD7887_PM_MODE2		(1)	 /* full on */
#define AD7887_PM_MODE3		(2)	 /* auto shutdown after conversion */
#define AD7887_PM_MODE4		(3)	 /* standby mode */

#define AD7888_REF_DIS		(1 << 2) /* on-chip reference disable */

/**
 * struct ad7887_chip_info - chip specifc information
 * @int_vref_mv:	the internal reference voltage
 * @ref_dis_mask:
 * @channel:		channel specification
 */
struct ad7887_chip_info {
	u16				int_vref_mv;
	bool				shared_vref;
	unsigned int			ref_dis_mask;
	const struct iio_chan_spec	*channels;
	unsigned int			num_channels;
};

struct ad7887_state {
	struct spi_device		*spi;
	const struct ad7887_chip_info	*chip_info;
	struct regulator		*reg;
	struct spi_transfer		*xfers;
	struct spi_message		message;
	uint8_t				mode;
	 __be16				*data;
};

enum ad7887_supported_device_ids {
	ID_AD7887,
	ID_AD7888,
};

static int ad7887_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *active_scan_mask)
{
	struct ad7887_state *st = iio_priv(indio_dev);
	unsigned int n, i, ch;
	__be16 *tx, *rx;

	n = bitmap_weight(active_scan_mask, indio_dev->masklength);

	kfree(st->data);
	st->data = kzalloc(sizeof(*tx) * n + indio_dev->scan_bytes, GFP_KERNEL);
	if (!st->data)
		return -ENOMEM;

	kfree(st->xfers);
	st->xfers = kcalloc(sizeof(*st->xfers), n, GFP_KERNEL);
	if (!st->xfers)
		return -ENOMEM;

	tx = st->data;
	rx = st->data + n;

	spi_message_init(&st->message);

	i = 0;
	for_each_set_bit(ch, active_scan_mask, indio_dev->masklength) {
		st->xfers[i].tx_buf = &tx[i];
		st->xfers[i].rx_buf = &rx[i];
		st->xfers[i].len = sizeof(*tx);
		if (i != n - 1)
			st->xfers[i].cs_change = 1;
		if (i == 0)
			tx[n - 1] = cpu_to_be16((st->mode | AD7887_CH(ch)) << 8);
		else
			tx[i - 1] = cpu_to_be16((st->mode | AD7887_CH(ch)) << 8);
		spi_message_add_tail(&st->xfers[i], &st->message);
		i++;
	}

	return 0;
}

static int ad7887_ring_preenable(struct iio_dev *indio_dev)
{
	struct ad7887_state *st = iio_priv(indio_dev);
	unsigned int i;

	/*
	 * Select the first channel for sampling. The command for selecting the
	 * first channel is stored in the last tx buffer element.
	 */
	i = bitmap_weight(indio_dev->active_scan_mask, indio_dev->masklength) - 1;
	return spi_write(st->spi, &st->data[i], sizeof(*st->data));
}

/**
 * ad7887_trigger_handler() bh of trigger launched polling to ring buffer
 *
 * Currently there is no option in this driver to disable the saving of
 * timestamps within the ring.
 **/
static irqreturn_t ad7887_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7887_state *st = iio_priv(indio_dev);
	int b_sent;

	b_sent = spi_sync(st->spi, &st->message);
	if (b_sent)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, st->data,
		iio_get_time_ns(indio_dev));
done:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops ad7887_ring_setup_ops = {
	.preenable = &ad7887_ring_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static int ad7887_scan_direct(struct ad7887_state *st, unsigned ch)
{
	struct spi_transfer t[] = {
		{
			.len	    = sizeof(*st->data),
			.cs_change  = 1,
		},
		{
			.len	    = sizeof(*st->data),
		}
	};
	int ret;

	if (!st->data) {
		st->data = kmalloc(sizeof(*st->data) * 2, GFP_KERNEL);
		if (!st->data)
			return -ENOMEM;
	}

	t[0].tx_buf = &st->data[0];
	t[1].tx_buf = &st->data[0];
	t[1].rx_buf = &st->data[1];

	st->data[0] = cpu_to_be16((st->mode | AD7887_CH(ch)) << 8);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	return be16_to_cpu(st->data[1]);
}

static int ad7887_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct ad7887_state *st = iio_priv(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = ad7887_scan_direct(st, chan->address);
		iio_device_release_direct_mode(indio_dev);

		if (ret < 0)
			return ret;
		*val = ret >> chan->scan_type.shift;
		*val &= GENMASK(chan->scan_type.realbits - 1, 0);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (st->reg) {
			*val = regulator_get_voltage(st->reg);
			if (*val < 0)
				return *val;
			*val /= 1000;
		} else {
			*val = st->chip_info->int_vref_mv;
		}

		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

#define AD7887_CHANNEL(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.address = (x), \
	.scan_index = (x), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_BE, \
	}, \
}

static const struct iio_chan_spec ad7888_channels[] = {
    IIO_CHAN_SOFT_TIMESTAMP(8),
    AD7887_CHANNEL(0),
    AD7887_CHANNEL(1),
    AD7887_CHANNEL(2),
    AD7887_CHANNEL(3),
    AD7887_CHANNEL(4),
    AD7887_CHANNEL(5),
    AD7887_CHANNEL(6),
    AD7887_CHANNEL(7),
};

static const struct ad7887_chip_info ad7887_chip_info_tbl[] = {
	/*
	 * More devices added in future
	 */
	[ID_AD7887] = {
		.channels = ad7888_channels,
		.num_channels = 3,
		.int_vref_mv = 2500,
		.shared_vref = true,
		.ref_dis_mask = AD7887_REF_DIS,
	},
	[ID_AD7888] = {
		.channels = ad7888_channels,
		.num_channels = ARRAY_SIZE(ad7888_channels),
		.int_vref_mv = 2500,
		.shared_vref = false,
		.ref_dis_mask = AD7888_REF_DIS,
	},
};

static const struct iio_info ad7887_info = {
	.read_raw = &ad7887_read_raw,
	.update_scan_mode = &ad7887_update_scan_mode,
};

static int ad7887_probe(struct spi_device *spi)
{
	struct ad7887_platform_data *pdata = spi->dev.platform_data;
	struct ad7887_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	if (!pdata || !pdata->use_onchip_ref) {
		st->reg = devm_regulator_get(&spi->dev, "vref");
		if (IS_ERR(st->reg))
			return PTR_ERR(st->reg);

		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	st->chip_info =
		&ad7887_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	/* Estabilish that the iio_dev is a child of the spi device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad7887_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	st->mode = AD7887_PM_MODE4;
	if (!pdata || !pdata->use_onchip_ref)
		st->mode |= st->chip_info->ref_dis_mask;
	if (st->chip_info->shared_vref) {
		if (pdata && pdata->en_dual)
			st->mode |= AD7887_DUAL;
		else
			indio_dev->num_channels--;
	}

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
			&ad7887_trigger_handler, &ad7887_ring_setup_ops);
	if (ret)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_unregister_ring;

	return 0;
error_unregister_ring:
	iio_triggered_buffer_cleanup(indio_dev);
error_disable_reg:
	if (st->reg)
		regulator_disable(st->reg);

	return ret;
}

static int ad7887_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7887_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);
	if (st->reg)
		regulator_disable(st->reg);
	kfree(st->xfers);
	kfree(st->data);

	return 0;
}

static const struct spi_device_id ad7887_id[] = {
	{"ad7887", ID_AD7887},
	{"ad7888", ID_AD7888},
	{}
};
MODULE_DEVICE_TABLE(spi, ad7887_id);

static struct spi_driver ad7887_driver = {
	.driver = {
		.name	= "ad7887",
	},
	.probe		= ad7887_probe,
	.remove		= ad7887_remove,
	.id_table	= ad7887_id,
};
module_spi_driver(ad7887_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7887/AD7888 ADC");
MODULE_LICENSE("GPL v2");
