// SPDX-License-Identifier: GPL-2.0+
/*
 * AD400x SPI ADC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */
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
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AD400X_READ_COMMAND	0x54
#define AD400X_WRITE_COMMAND	0x14
#define AD400X_RESERVED_MSK	0xE0

#define ADAQ4003_MAX_FREQ	2000000

#define AD400X_TURBO_MODE(x)	FIELD_PREP(BIT_MASK(1), x)
#define AD400X_HIGH_Z_MODE(x)	FIELD_PREP(BIT_MASK(2), x)

#define AD400X_CHANNEL(real_bits)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE) |			\
			BIT(IIO_CHAN_INFO_OFFSET),			\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = real_bits,				\
			.storagebits = 32,				\
		},							\
	}								\

enum ad400x_ids {
	ID_AD4000,
	ID_AD4001,
	ID_AD4002,
	ID_AD4003,
	ID_AD4004,
	ID_AD4005,
	ID_AD4006,
	ID_AD4007,
	ID_AD4008,
	ID_AD4010,
	ID_AD4011,
	ID_AD4020,
	ID_AD4021,
	ID_AD4022,
	ID_ADAQ4003,
};

struct ad400x_chip_info {
	struct iio_chan_spec chan_spec;
	int max_rate;
};

static const struct ad400x_chip_info ad400x_chips[] = {
	[ID_AD4000] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
	},
	[ID_AD4001] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
	},
	[ID_AD4002] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
	},
	[ID_AD4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
	},
	[ID_AD4004] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
	},
	[ID_AD4005] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
	},
	[ID_AD4006] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
	},
	[ID_AD4007] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
	},
	[ID_AD4008] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  =  500000,
	},
	[ID_AD4010] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
	},
	[ID_AD4011] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
	},
	[ID_AD4020] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1800000,
	},
	[ID_AD4021] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1000000,
	},
	[ID_AD4022] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  =  500000,
	},
	[ID_ADAQ4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
	},
};

struct ad400x_state {
	struct spi_device *spi;
	struct regulator *vref;
	/* protect device accesses */
	struct mutex lock;
	bool bus_locked;

	unsigned long ref_clk_rate;
	struct pwm_device *cnv_trigger;
	const struct ad400x_chip_info *chip;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	bool turbo_mode;
	bool high_z_mode;

	unsigned int num_bits;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	uint8_t	data[4] ____cacheline_aligned;
};

static int ad400x_get_sampling_freq(struct ad400x_state *st)
{
	struct pwm_state conversion_state;

	pwm_get_state(st->cnv_trigger, &conversion_state);
	return DIV_ROUND_CLOSEST_ULL(1000000000000, conversion_state.period);
}

static int __ad400x_set_sampling_freq(struct ad400x_state *st, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnv_state;

	/* Sync up PWM state and prepare for pwm_apply_state(). */
	pwm_init_state(st->cnv_trigger, &cnv_state);

	freq = clamp(freq, 0, st->chip->max_rate);
	target = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000,
						  st->ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = ref_clk_period_ps;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	return pwm_apply_state(st->cnv_trigger, &cnv_state);
}

static int ad400x_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	if (!st->cnv_trigger)
		return -EINVAL;

	if (!freq || freq > st->chip->max_rate)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ad400x_set_sampling_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad400x_write_reg(struct ad400x_state *st, uint8_t val)
{
	struct spi_transfer t = {
		.tx_buf		= st->data,
		.len		= 4,
		.bits_per_word	= st->num_bits,
	};
	struct spi_message m;

	st->data[0] = AD400X_WRITE_COMMAND;
	st->data[1] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (st->bus_locked)
		return spi_sync_locked(st->spi, &m);

	return spi_sync(st->spi, &m);
}

static int ad400x_read_reg(struct ad400x_state *st, unsigned int *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;

	st->data[0] = AD400X_READ_COMMAND;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 2;
	t.bits_per_word = 16; /* reg reads are only 16 clocks pulses */

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->bus_locked)
		ret = spi_sync_locked(st->spi, &m);
	else
		ret = spi_sync(st->spi, &m);

	if (ret < 0)
		return ret;

	*val = st->data[0];

	return ret;
}

static int ad400x_read_sample(struct ad400x_state *st, uint32_t *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	uint8_t input[4] = {0};
	int ret;

	t.rx_buf = input;
	t.len = 4;
	t.bits_per_word = st->num_bits;
	if (st->turbo_mode) {
		t.word_delay.value = 500;
		t.word_delay.unit = SPI_DELAY_UNIT_NSECS;
	}

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->bus_locked)
		ret = spi_sync_locked(st->spi, &m);
	else
		ret = spi_sync(st->spi, &m);

	if (ret < 0)
		return ret;

	dev_info(&st->spi->dev, "read: %u", *val);
	*val = (input[2] << 16) | (input[1] << 8) | input[0];

	return ret;
}

static int ad400x_set_mode(struct ad400x_state *st)
{
	uint8_t mode;
	int ret;

	mode = AD400X_TURBO_MODE(st->turbo_mode) |
		AD400X_HIGH_Z_MODE(st->high_z_mode);

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = ad400x_write_reg(st, mode);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);

	return ret;
}

static int ad400x_single_conversion(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	unsigned int sample, raw_sample;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = ad400x_read_sample(st, &raw_sample);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);
	iio_device_release_direct_mode(indio_dev);

	if (ret)
		return ret;

	sample = raw_sample >> chan->scan_type.shift;
	sample &= (1 << chan->scan_type.realbits) - 1;

	*val = sample;

	return IIO_VAL_INT;
}

static int ad400x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad400x_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad400x_get_sampling_freq(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = -(1 << chan->scan_type.realbits);

		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ad400x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	if (readval)
		ret = ad400x_read_reg(st, readval);
	else
		ret = ad400x_write_reg(st, writeval);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);
	mutex_unlock(&st->lock);

	return ret;
}

static int ad400x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad400x_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad400x_info = {
	.read_raw = &ad400x_read_raw,
	.write_raw = &ad400x_write_raw,
	.debugfs_reg_access = &ad400x_reg_access,
};

static const struct spi_device_id ad400x_id[] = {
	{ "ad4001", (kernel_ulong_t)&ad400x_chips[ID_AD4001] },
	{ "ad4002", (kernel_ulong_t)&ad400x_chips[ID_AD4002] },
	{ "ad4003", (kernel_ulong_t)&ad400x_chips[ID_AD4003] },
	{ "ad4004", (kernel_ulong_t)&ad400x_chips[ID_AD4004] },
	{ "ad4005", (kernel_ulong_t)&ad400x_chips[ID_AD4005] },
	{ "ad4006", (kernel_ulong_t)&ad400x_chips[ID_AD4006] },
	{ "ad4008", (kernel_ulong_t)&ad400x_chips[ID_AD4008] },
	{ "ad4010", (kernel_ulong_t)&ad400x_chips[ID_AD4010] },
	{ "ad4020", (kernel_ulong_t)&ad400x_chips[ID_AD4020] },
	{ "ad4021", (kernel_ulong_t)&ad400x_chips[ID_AD4021] },
	{ "ad4022", (kernel_ulong_t)&ad400x_chips[ID_AD4022] },
	{ "adaq4003", (kernel_ulong_t)&ad400x_chips[ID_ADAQ4003] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad400x_id);

static int ad400x_enable_cnv_trigger(struct ad400x_state *st, bool enable)
{
	struct pwm_state cnv_state;

	pwm_get_state(st->cnv_trigger, &cnv_state);
	cnv_state.enabled = enable;
	return pwm_apply_state(st->cnv_trigger, &cnv_state);
}

static int ad400x_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = (void *)-1;
	st->spi_transfer.len = 4;
	st->spi_transfer.bits_per_word = st->num_bits;

	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;

	spi_engine_offload_enable(st->spi, true);

	return ad400x_enable_cnv_trigger(st, true);
}

static int ad400x_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi, false);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);

	return ad400x_enable_cnv_trigger(st, false);
}

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
			   struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad400x_buffer_setup_ops = {
	.preenable = &ad400x_buffer_preenable,
	.postdisable = &ad400x_buffer_postdisable,
};

static void ad400x_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static void ad400x_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void ad400x_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static int pwm_gen_setup(struct spi_device *spi, struct ad400x_state *st)
{
	struct clk *ref_clk;
	int ret;

	ref_clk = devm_clk_get(&spi->dev, "ref_clk");
	if (IS_ERR(ref_clk))
		return PTR_ERR(ref_clk);

	ret = clk_prepare_enable(ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad400x_clk_disable, ref_clk);
	if (ret)
		return ret;

	st->ref_clk_rate = clk_get_rate(ref_clk);

	st->cnv_trigger = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(st->cnv_trigger))
		return PTR_ERR(st->cnv_trigger);

	ret = devm_add_action_or_reset(&spi->dev, ad400x_pwm_diasble,
				       st->cnv_trigger);
	if (ret)
		return ret;

	return __ad400x_set_sampling_freq(st, st->chip->max_rate);
}

static int ad400x_probe(struct spi_device *spi)
{
	const struct ad400x_chip_info *chip;
	struct ad400x_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	const char *p_compat;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	chip = (const struct ad400x_chip_info *)device_get_match_data(&spi->dev);
	if (!chip)
		return -EINVAL;

	st = iio_priv(indio_dev);
	st->chip = chip;
	st->spi = spi;
	mutex_init(&st->lock);

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad400x_regulator_disable, st->vref);
	if (ret)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->info = &ad400x_info;

	indio_dev->channels = &st->chip->chan_spec;
	indio_dev->num_channels = 1;

	st->num_bits = indio_dev->channels->scan_type.realbits;

	/* Set turbo mode */
	st->turbo_mode = true;
	ret = ad400x_set_mode(st);
	if (ret < 0)
		return ret;

	/* We support DMA with the SPI engine, otherwise we actually don't */
	ret = device_property_read_string(spi->dev.parent, "compatible", &p_compat);
	if (ret < 0)
		return ret;

	if (strcmp(p_compat, "adi,axi-spi-engine-1.00.a") == 0) {
		buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
							 "rx", &dma_buffer_ops,
							 indio_dev);
		if (IS_ERR(buffer))
			return dev_err_probe(&spi->dev, PTR_ERR(buffer),
					     "Error initializing buffer: %ld\n",
					     PTR_ERR(buffer));

		ret = iio_device_attach_buffer(indio_dev, buffer);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Error attaching buffer: %d\n",
					     ret);

		indio_dev->setup_ops = &ad400x_buffer_setup_ops;
	}

	if (device_property_present(&spi->dev, "pwms"))
		pwm_gen_setup(spi, st);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad400x_of_match[] = {
	{ .compatible = "adi,ad4001", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4001] },
	{ .compatible = "adi,ad4002", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4002] },
	{ .compatible = "adi,ad4003", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4003] },
	{ .compatible = "adi,ad4004", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4004] },
	{ .compatible = "adi,ad4005", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4005] },
	{ .compatible = "adi,ad4006", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4006] },
	{ .compatible = "adi,ad4008", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4008] },
	{ .compatible = "adi,ad4010", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4010] },
	{ .compatible = "adi,ad4020", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4020] },
	{ .compatible = "adi,ad4021", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4021] },
	{ .compatible = "adi,ad4022", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4022] },
	{ .compatible = "adi,adaq4003", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_ADAQ4003] },
	{ },
};
MODULE_DEVICE_TABLE(of, ad400x_of_match);

static struct spi_driver ad400x_driver = {
	.driver = {
		.name   = "ad400x",
		.of_match_table = ad400x_of_match,
	},
	.probe          = ad400x_probe,
	.id_table       = ad400x_id,
};
module_spi_driver(ad400x_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD400x ADC driver");
MODULE_LICENSE("GPL v2");
