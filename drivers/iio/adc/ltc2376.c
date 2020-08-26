// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linear Technology LTC2376 Low Power SAR ADC
 *
 * Copyright (C) 2020 Analog Devices, Inc.
 */

#include <asm/unaligned.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define DEFAULT_SAMPLING_FREQUENCY 20000
#define DEFAULT_DUTY_CYCLE 30
#define MINIMUM_PERIOD 108

struct ltc2376_state {
	struct spi_device       *spi;
	struct regulator        *reg;
	struct pwm_device       *pwm;

	struct iio_trigger      *trig;
	struct irq_chip         irq;

	struct spi_transfer     xfer;
	struct spi_message      msg;

	struct mutex            lock;
	unsigned int            num_bits;

	unsigned int            frequency;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * Make the buffer large enough for two 16 bit sample and one 64 bit
	 * aligned 64 bit timestamp.
	 */
	struct {
		__be16 sample[2];
		s64 timestamp;
	} data ____cacheline_aligned;
};

ssize_t ltc2376_show_freq(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ltc2376_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->frequency);
}

ssize_t ltc2376_store_freq(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ltc2376_state *st = iio_priv(indio_dev);

	int period;
	int ret;
	unsigned int freq;

	mutex_lock(&st->lock);

	ret = kstrtouint(buf, 10, &freq);

	if (ret < 0) {
		mutex_unlock(&st->lock);
		return ret;
	}

	period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);
	if (period < MINIMUM_PERIOD) {
		mutex_unlock(&st->lock);
		return 0;
	}
	pwm_config(st->pwm, DEFAULT_DUTY_CYCLE, period);

	st->frequency = freq;
	mutex_unlock(&st->lock);

	return count;
}

static IIO_DEV_ATTR_SAMP_FREQ(0644, ltc2376_show_freq, ltc2376_store_freq);

static struct attribute *ltc2376_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL
};

static const struct attribute_group ltc2376_attribute_group = {
	.attrs = ltc2376_attributes,
};

static int ltc2376_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	int ret;
	struct ltc2376_state *st = iio_priv(indio_dev);

	switch (info) {

	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = spi_sync(st->spi, &st->msg);
		iio_device_release_direct_mode(indio_dev);
		if (ret < 0)
			return ret;
		*val = sign_extend32(be16_to_cpu(st->data.sample[0]),
				st->num_bits-1);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->reg);
		if (ret < 0)
			return ret;
		*val = ret/1000;
		*val2 = st->num_bits;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_OFFSET:
		*val = (1 << (st->num_bits - 1));
		return IIO_VAL_INT;

	default:
		return -EINVAL;

	}
}

static const struct iio_info ltc2376_info = {
	.attrs = &ltc2376_attribute_group,
	.read_raw = &ltc2376_read_raw,
};

#define LTC2376_DIFF_CHANNEL(real_bits)                    \
{                                                            \
	.type = IIO_VOLTAGE,                                 \
	.indexed = 1,                                        \
	.differential = 1,                                   \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),        \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) \
		| BIT(IIO_CHAN_INFO_OFFSET),                 \
	.scan_index = 0,                                     \
	.scan_type = {                                       \
		.sign = 's',                                 \
		.endianness = IIO_BE,                        \
		.realbits = real_bits,                       \
		.shift = 0,                                  \
		.storagebits = 16,                           \
	},                                                   \
}                                                        \

static const struct iio_chan_spec ltc2376_channels[] = {
	LTC2376_DIFF_CHANNEL(16)
};

static irqreturn_t ltc2376_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ltc2376_state *st = iio_priv(indio_dev);
	int b_sent;

	mutex_lock(&st->lock);

	b_sent = spi_sync(st->spi, &st->msg);
	if (b_sent < 0)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, &st->data,
		iio_get_time_ns(indio_dev));
done:
	iio_trigger_notify_done(indio_dev->trig);

	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

static const struct iio_trigger_ops ltc2376_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static int ltc2376_pwm_probe(struct iio_dev *indio_dev)
{
	int ret;
	unsigned int period;
	struct pwm_state state = {};
	struct ltc2376_state *st = iio_priv(indio_dev);

	st->frequency = DEFAULT_SAMPLING_FREQUENCY;
	period = DIV_ROUND_CLOSEST(NSEC_PER_SEC, st->frequency);

	st->pwm = devm_pwm_get(&st->spi->dev, NULL);
	if (IS_ERR(st->pwm)) {
		ret = PTR_ERR(st->pwm);
		if (ret != -EPROBE_DEFER)
			dev_err(&st->spi->dev, "pwm get failed: %d\n", ret);
			return ret;
	}

	pwm_init_state(st->pwm, &state);
	state.period = period;
	state.duty_cycle = DEFAULT_DUTY_CYCLE;
	state.enabled = true;

	ret = pwm_apply_state(st->pwm, &state);
	if (ret) {
		dev_err(&st->spi->dev, "failed to configure PWM: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ltc2376_trigger_probe(struct iio_dev *indio_dev)
{
	int ret;
	struct ltc2376_state *st = iio_priv(indio_dev);

	st->trig = devm_iio_trigger_alloc(&st->spi->dev, "%s-dev%d",
		indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ltc2376_trigger_ops;
	st->trig->dev.parent = &st->spi->dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(&st->spi->dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	ret = devm_request_irq(&st->spi->dev, st->spi->irq,
			&iio_trigger_generic_data_rdy_poll,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"ltc2376irq", st->trig);

	if (ret < 0) {
		dev_err(&st->spi->dev, "failed requesting irq %d\n",
				st->spi->irq);
		return ret;
	}

	return 0;
}

static int ltc2376_probe(struct spi_device *spi)
{
	struct ltc2376_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, indio_dev);

	ret = ltc2376_pwm_probe(indio_dev);
	if (ret)
		return ret;

	st->reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ltc2376";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2376_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2376_channels);
	indio_dev->info = &ltc2376_info;

	ret = ltc2376_trigger_probe(indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
			&iio_pollfunc_store_time,
			&ltc2376_trigger_handler,
			NULL);
	if (ret)
		return ret;

	st->num_bits = indio_dev->channels->scan_type.realbits;
	mutex_init(&st->lock);
	st->xfer.rx_buf = &st->data;
	st->xfer.len = 2;
	spi_message_init(&st->msg);
	spi_message_add_tail(&st->xfer, &st->msg);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int ltc2376_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ltc2376_state *st = iio_priv(indio_dev);

	regulator_disable(st->reg);
	pwm_disable(st->pwm);

	return 0;
}

static const struct of_device_id ltc2376_of_match[] = {
	{ .compatible = "adi,ltc2376" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2376_of_match);

static struct spi_driver ltc2376_driver = {
	.driver = {
		.name = "ltc2376",
		.of_match_table = ltc2376_of_match,
	},
	.probe = ltc2376_probe,
	.remove = ltc2376_remove,
};
module_spi_driver(ltc2376_driver);

MODULE_AUTHOR("Meenal Parakh <meenalparakh18@gmail.com>");
MODULE_DESCRIPTION("Linear Technology LTC2376 SAR ADC driver");
MODULE_LICENSE("GPL");
