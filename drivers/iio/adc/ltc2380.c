#include <asm/unaligned.h>

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/sysfs.h>

#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define LTC2380_FRAME_SIZE	3
#define LTC2380_MAX_FREQ	2000000
#define MHz 				1000000

struct ltc2380_chip_info {
	const char *name;
};

struct ltc2380_state {
	struct spi_device *spi;
	struct regulator *vref;

	struct ltc2380_chip_info *chip_info;
	struct pwm_device *cnv;

	u32 sampling_freq;
	u32 ref_clk_rate;
	struct clk *ref_clk;
	struct iio_trigger *trig;
	struct completion adc_data_completion;

	struct {
		u8 rx_buff[LTC2380_FRAME_SIZE];
		u64 timestamp;
	} adc_samples_buff ____cacheline_aligned;

	struct spi_message adc_samples_msg;
	u8 buff[LTC2380_FRAME_SIZE];
};

static struct ltc2380_chip_info ltc2380_data = {
	.name = "ltc2380",
};

static const struct iio_chan_spec ltc2380_channel = {
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
					  BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_type = {
			.sign = 's',
			.realbits = 24,
			.storagebits = 24,
		},
};

static const struct iio_trigger_ops ltc2380_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static int ltc2380_set_sampling_freq(struct ltc2380_state *st, u32 freq)
{
	u64 target, ref_clk_period_us;
	struct pwm_state cnv_state;
	int ret;

	target = DIV_ROUND_CLOSEST_ULL(1000000000ULL, freq);
	//ref_clk_period_us = DIV_ROUND_CLOSEST_ULL(1000000ULL, st->ref_clk_rate);

	cnv_state.period = 1000000000;
	cnv_state.duty_cycle = 1000000000 / 2;
	cnv_state.phase = 0;
	cnv_state.time_unit = PWM_UNIT_NSEC;
	cnv_state.polarity = PWM_POLARITY_NORMAL;
	cnv_state.enabled = 1;

	ret = pwm_apply_state(st->cnv, &cnv_state);
	if (ret)
		return ret;

	st->sampling_freq = 100;

	return 0;
}

static int ltc2380_read_sample(struct ltc2380_state *st, u32 *val)
{
	struct spi_transfer xfer = {
		.rx_buf = st->buff,
		.len = LTC2380_FRAME_SIZE,
	};
	int ret;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	*val = get_unaligned_be24(st->buff);

	return 0;
}

static int ltc2380_single_conversion(struct ltc2380_state *st, u32 *val)
{
	struct iio_dev *indio_dev = spi_get_drvdata(st->spi);
	int ret;

	// ret = iio_device_claim_direct_mode(indio_dev);
	// if (ret)
	// 	return ret;

	// spi_bus_lock(st->spi->master);

	/* Read the last result in order to start a new conversion */
	ret = ltc2380_read_sample(st, val);
	if (ret)
		goto err;

	mdelay(100);
	wait_for_completion_timeout(&st->adc_data_completion, msecs_to_jiffies(1000));
	ret = ltc2380_read_sample(st, val);
	mdelay(100);
err:
	// spi_bus_unlock(st->spi->master);
	// iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ltc2380_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ltc2380_state *st = iio_priv(indio_dev);
	int ret;

	switch(info) {
	case IIO_CHAN_INFO_RAW:
		ret = ltc2380_single_conversion(st, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;

		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ltc2380_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ltc2380_state *st = iio_priv(indio_dev);

	switch(info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val > LTC2380_MAX_FREQ)
			return -EINVAL;

		return ltc2380_set_sampling_freq(st, val);
	default:
		return -EINVAL;
	}
}

static irqreturn_t ltc2380_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ltc2380_state *st = iio_priv(indio_dev);
	int ret;

	ret = spi_sync(st->spi, &st->adc_samples_msg);
	if (ret)
		goto err;

	iio_push_to_buffers_with_timestamp(indio_dev, &st->adc_samples_buff,
					   iio_get_time_ns(indio_dev));

err:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t ltc2380_adc_data_interrupt(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ltc2380_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->adc_data_completion);

	return IRQ_HANDLED;
}

static void ltc2380_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static void ltc2380_pwm_disable(void *data)
{
	pwm_disable(data);
}

static void ltc2380_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct iio_info ltc2380_info = {
	.read_raw = &ltc2380_read_raw,
	.write_raw = &ltc2380_write_raw,
};

static int ltc2380_probe(struct spi_device *spi)
{
	const struct ltc2380_chip_info *ddata;
	struct iio_dev *indio_dev;
	struct ltc2380_state *st;
	int ret;

	printk("LTC2380 probing\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return dev_err_probe(&spi->dev, PTR_ERR(st->vref),
				     "Failed to get vref");

	spi_set_drvdata(spi, indio_dev);
	printk("LTC2380 vref\n");

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ltc2380_regulator_disable, st->vref);
	if (ret)
		return ret;

	ddata = device_get_match_data(&spi->dev);
	if (!ddata)
		ddata = (const struct ltc2380_chip_info *)spi_get_drvdata(spi);

	st->chip_info = ddata;
	printk("LTC2380 data\n");

	st->cnv = devm_pwm_get(&spi->dev, "pwm-trigger");
	if (IS_ERR(st->cnv))
		return PTR_ERR(st->cnv);

	ret = ltc2380_set_sampling_freq(st, 1000000);
	printk("LTC2380 pwm setup: %d\n", ret);
	printk("LTC2380 pwm period: %lld\n", pwm_get_period(st->cnv));
	// st->ref_clk = devm_clk_get(&spi->dev, NULL);
	// if (IS_ERR(st->ref_clk))
	// 	return PTR_ERR(st->ref_clk);

	// printk("LTC2380 data\n");

	// ret = clk_prepare_enable(st->ref_clk);
	// if (ret)
	// 	return ret;

	// printk("LTC2380 clk enable\n");

	// ret = devm_add_action_or_reset(&spi->dev, ltc2380_clk_disable, st->ref_clk);
	// if (ret)
	// 	return ret;

	ret = devm_add_action_or_reset(&spi->dev, ltc2380_pwm_disable, st->cnv);
	if (ret)
		return ret;

	printk("LTC2380 pwm disable action\n");

	st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d", st->chip_info->name,
					  indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	printk("LTC2380 trigger alloc\n");

	ret = devm_request_threaded_irq(&spi->dev, spi->irq, ltc2380_adc_data_interrupt,
			    		0, st->chip_info->name, st->trig);
	if(ret)
		return ret;
	printk("LTC2380 request irq\n");

	st->trig->ops = &ltc2380_trigger_ops;
	st->trig->dev.parent = &spi->dev;
	iio_trigger_set_drvdata(st->trig, st);

	ret = devm_iio_trigger_register(&spi->dev, st->trig);
	if (ret)
		return ret;

	printk("LTC2380 trigger register\n");

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ltc2380_trigger_handler,
					      NULL);
	if(ret)
		return ret;

	printk("LTC2380 trigger buffer setup\n");

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2380_info;
	indio_dev->channels = &ltc2380_channel;
	indio_dev->num_channels = 1;
	indio_dev->trig = iio_trigger_get(st->trig);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2380_of_match[] = {
	{ .compatible = "lltc,ltc2380", .data = &ltc2380_data, },
	{}
};
MODULE_DEVICE_TABLE(of, ltc2380_of_match);

static const struct spi_device_id ltc2380_id[] = {
	{ .name = "ltc2380", .driver_data = (kernel_ulong_t)&ltc2380_data },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc2380_id);

static struct spi_driver ltc2380_driver = {
	.driver = {
		.name   = "ltc2380",
		.of_match_table = ltc2380_of_match,
	},
	.probe          = ltc2380_probe,
	.id_table       = ltc2380_id,
};
module_spi_driver(ltc2380_driver);

MODULE_AUTHOR("Ciprian Regus <ciprian.regus@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2380 ADC driver");
MODULE_LICENSE("GPL v2");
