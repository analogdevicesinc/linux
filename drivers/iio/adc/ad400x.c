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
#include <linux/units.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AD400X_READ_COMMAND	0x54
#define AD400X_WRITE_COMMAND	0x14
#define AD400X_RESERVED_MSK	0xE0

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

enum ad400x_input_type {
	SINGLE_ENDED,
	DIFFERENTIAL,
};

/* Gains stored as fractions of 1000 so they can be expressed by integers. */
static const int ad4000_gains[] = {
	454, 909, 1000, 1900,
};

struct ad400x_chip_info {
	struct iio_chan_spec chan_spec;
	int max_rate;
	enum ad400x_input_type input_type;
	bool has_hardware_gain;
};

static const struct ad400x_chip_info ad400x_chips[] = {
	[ID_AD4000] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4001] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4002] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4004] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4005] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4006] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4007] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4008] = {
		.chan_spec = AD400X_CHANNEL(16),
		.max_rate  =  500000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4010] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
		.input_type = SINGLE_ENDED,
	},
	[ID_AD4011] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  =  500000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4020] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1800000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4021] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  = 1000000,
		.input_type = DIFFERENTIAL,
	},
	[ID_AD4022] = {
		.chan_spec = AD400X_CHANNEL(20),
		.max_rate  =  500000,
		.input_type = DIFFERENTIAL,
	},
	[ID_ADAQ4003] = {
		.chan_spec = AD400X_CHANNEL(18),
		.max_rate  = 2000000,
		.input_type = DIFFERENTIAL,
		.has_hardware_gain = true,
	},
};

struct ad400x_state {
	struct spi_device *spi;
	int vref_mv;
	/* protect device accesses */
	struct mutex lock;
	bool bus_locked;

	unsigned long ref_clk_rate;
	struct pwm_device *cnv_trigger;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	const struct ad400x_chip_info *chip;
	bool span_comp;
	bool turbo_mode;
	bool high_z_mode;
	u16 gain_milli;
	int scale_tbl[2][2];

	unsigned int num_bits;
	int read_offset;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	uint8_t	data[4] ____cacheline_aligned;
};

static int ad400x_get_sampling_freq(struct ad400x_state *st)
{
	return DIV_ROUND_CLOSEST_ULL(1000000000000,
				     pwm_get_period(st->cnv_trigger));
}

static int __ad400x_set_sampling_freq(struct ad400x_state *st, int freq)
{
	struct pwm_state cnv_state;
	u32 rem;

	/* Sync up PWM state and prepare for pwm_apply_state(). */
	pwm_init_state(st->cnv_trigger, &cnv_state);

	/*
	 * The goal here is that the PWM is configured with a minimal period not
	 * less than 1 / freq (with freq measured in Hz). It should not be less
	 * because freq is usually st->chip->max_rate which is a hard limit.
	 *
	 * When a period P (measured in ns) is passed to pwm_apply_state(), the
	 * actually implemented period is:
	 *
	 * 	round_down(P * R / NSEC_PER_SEC) / R
	 *
	 * (measured in s) with R = st->ref_clk_rate. So we have:
	 *
	 * 	  round_down(P * R / NSEC_PER_SEC) / R ≥ 1 / freq
	 * 	⟺ round_down(P * R / NSEC_PER_SEC) ≥ R / freq
	 *
	 * With the LHS being integer this is equivalent to:
	 *
	 * 	  round_down(P * R / NSEC_PER_SEC) ≥ round_up(R / freq)
	 * 	⟺ P * R / NSEC_PER_SEC ≥ round_up(R / freq)
	 * 	⟺ P ≥ round_up(R / freq) * NSEC_PER_SEC / R
	 */

	cnv_state.period = div_u64_rem((u64)DIV_ROUND_UP(st->ref_clk_rate, freq) * NSEC_PER_SEC,
				       st->ref_clk_rate, &rem);
	if (rem)
		cnv_state.period += 1;

	cnv_state.duty_cycle = DIV_ROUND_UP(NSEC_PER_SEC, st->ref_clk_rate);

	return pwm_apply_state(st->cnv_trigger, &cnv_state);
}

static int ad400x_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	if (!st->cnv_trigger)
		return -ENODEV;

	if (!freq || freq > st->chip->max_rate)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ad400x_set_sampling_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static void ad4000_fill_scale_tbl(struct ad400x_state *st,
				  struct iio_chan_spec const *chan)
{
	int val, tmp0, tmp1;
	int scale_bits;
	u64 tmp2;

	/*
	 * ADCs that output two's complement code have one less bit to express
	 * voltage magnitude.
	 */
	if (chan->scan_type.sign == 's')
		scale_bits = chan->scan_type.realbits - 1;
	else
		scale_bits = chan->scan_type.realbits;

	/*
	 * The gain is stored as a fraction of 1000 and, as we need to
	 * divide vref_mv by the gain, we invert the gain/1000 fraction.
	 * Also multiply by an extra MILLI to preserve precision.
	 * Thus, we have MILLI * MILLI equals MICRO as fraction numerator.
	 */
	val = mult_frac(st->vref_mv, MICRO, st->gain_milli);

	/* Would multiply by NANO here but we multiplied by extra MILLI */
	tmp2 = shift_right((u64)val * MICRO, scale_bits);
	tmp0 = div_s64_rem(tmp2, NANO, &tmp1);

	/* Store scale for when span compression is disabled */
	st->scale_tbl[0][0] = tmp0; /* Integer part */
	st->scale_tbl[0][1] = abs(tmp1); /* Fractional part */

	/* Store scale for when span compression is enabled */
	st->scale_tbl[1][0] = tmp0;

	/* The integer part is always zero so don't bother to divide it. */
	if (chan->differential)
		st->scale_tbl[1][1] = DIV_ROUND_CLOSEST(abs(tmp1) * 4, 5);
	else
		st->scale_tbl[1][1] = DIV_ROUND_CLOSEST(abs(tmp1) * 9, 10);
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
	int ret;

	t.rx_buf = st->data;
	t.len = 4;
	t.bits_per_word = st->num_bits;
	t.delay.value = 60;
	t.delay.unit = SPI_DELAY_UNIT_NSECS;

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->bus_locked)
		ret = spi_sync_locked(st->spi, &m);
	else
		ret = spi_sync(st->spi, &m);

	if (ret < 0)
		return ret;

	memcpy(val, &st->data[0], 4);

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
	if (st->chip->input_type == DIFFERENTIAL)
		*val = sign_extend32(sample, st->num_bits - 1);

	return IIO_VAL_INT;
}

static int ad400x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad400x_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad400x_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		*val = st->scale_tbl[st->span_comp][0];
		*val2 = st->scale_tbl[st->span_comp][1];
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad400x_get_sampling_freq(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = st->read_offset;
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
	struct ad400x_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad400x_set_sampling_freq(indio_dev, val);
	case IIO_CHAN_INFO_OFFSET:
		st->read_offset = val;
		return 0;
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
	{ "ad4000", (kernel_ulong_t)&ad400x_chips[ID_AD4000] },
	{ "ad4001", (kernel_ulong_t)&ad400x_chips[ID_AD4001] },
	{ "ad4002", (kernel_ulong_t)&ad400x_chips[ID_AD4002] },
	{ "ad4003", (kernel_ulong_t)&ad400x_chips[ID_AD4003] },
	{ "ad4004", (kernel_ulong_t)&ad400x_chips[ID_AD4004] },
	{ "ad4005", (kernel_ulong_t)&ad400x_chips[ID_AD4005] },
	{ "ad4006", (kernel_ulong_t)&ad400x_chips[ID_AD4006] },
	{ "ad4007", (kernel_ulong_t)&ad400x_chips[ID_AD4007] },
	{ "ad4008", (kernel_ulong_t)&ad400x_chips[ID_AD4008] },
	{ "ad4010", (kernel_ulong_t)&ad400x_chips[ID_AD4010] },
	{ "ad4011", (kernel_ulong_t)&ad400x_chips[ID_AD4011] },
	{ "ad4020", (kernel_ulong_t)&ad400x_chips[ID_AD4020] },
	{ "ad4021", (kernel_ulong_t)&ad400x_chips[ID_AD4021] },
	{ "ad4022", (kernel_ulong_t)&ad400x_chips[ID_AD4022] },
	{ "adaq4003", (kernel_ulong_t)&ad400x_chips[ID_ADAQ4003] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad400x_id);

static int ad400x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = (void *)-1;
	st->spi_transfer.len = 4;
	st->spi_transfer.bits_per_word = st->num_bits;
	st->spi_transfer.delay.value = 60;
	st->spi_transfer.delay.unit = SPI_DELAY_UNIT_NSECS;

	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;

	spi_engine_offload_enable(st->spi, true);

	return pwm_enable(st->cnv_trigger);
}

static int ad400x_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);

	pwm_disable(st->cnv_trigger);

	spi_engine_offload_enable(st->spi, false);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);

	return 0;
}

static const struct iio_buffer_setup_ops ad400x_buffer_setup_ops = {
	.postenable = &ad400x_buffer_postenable,
	.postdisable = &ad400x_buffer_postdisable,
};

static void ad400x_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static void ad400x_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static int ad400x_pwm_setup(struct spi_device *spi, struct ad400x_state *st)
{
	struct clk *ref_clk;
	int ret;

	ref_clk = devm_clk_get_enabled(&spi->dev, "ref_clk");
	if (IS_ERR(ref_clk))
		return PTR_ERR(ref_clk);

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
	struct regulator *vref_reg;
	struct ad400x_state *st;
	struct iio_dev *indio_dev;
	int gain_idx, ret;

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

	vref_reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(vref_reg))
		return PTR_ERR(vref_reg);

	ret = regulator_enable(vref_reg);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad400x_regulator_disable, vref_reg);
	if (ret)
		return ret;

	ret = regulator_get_voltage(vref_reg);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "Failed to get vref\n");

	st->vref_mv = ret / 1000;

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
	st->span_comp = 0; /* Span Compression not supported yet */
	st->gain_milli = 1000;
	if (chip->has_hardware_gain) {
		ret = device_property_read_u16(&spi->dev, "adi,gain-milli",
					       &st->gain_milli);
		if (!ret) {
			/* Match gain value from dt to one of supported gains */
			gain_idx = find_closest(st->gain_milli, ad4000_gains,
						ARRAY_SIZE(ad4000_gains));
			st->gain_milli = ad4000_gains[gain_idx];
		} else {
			return dev_err_probe(&spi->dev, ret,
					     "Failed to read gain property\n");
		}
	}

	ad4000_fill_scale_tbl(st, indio_dev->channels);

	if (spi_engine_offload_supported(spi)) {
		ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
						      indio_dev, "rx",
						      IIO_BUFFER_DIRECTION_IN);
		if (ret)
			return ret;

		indio_dev->setup_ops = &ad400x_buffer_setup_ops;
	}

	if (device_property_present(&spi->dev, "pwms"))
		ad400x_pwm_setup(spi, st);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad400x_of_match[] = {
	{ .compatible = "adi,ad4000", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4000] },
	{ .compatible = "adi,ad4001", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4001] },
	{ .compatible = "adi,ad4002", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4002] },
	{ .compatible = "adi,ad4003", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4003] },
	{ .compatible = "adi,ad4004", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4004] },
	{ .compatible = "adi,ad4005", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4005] },
	{ .compatible = "adi,ad4006", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4006] },
	{ .compatible = "adi,ad4007", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4007] },
	{ .compatible = "adi,ad4008", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4008] },
	{ .compatible = "adi,ad4010", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4010] },
	{ .compatible = "adi,ad4011", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4011] },
	{ .compatible = "adi,ad4020", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4020] },
	{ .compatible = "adi,ad4021", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4021] },
	{ .compatible = "adi,ad4022", .data = (struct ad400x_chip_info *)&ad400x_chips[ID_AD4022] },
	{ .compatible = "adi,adaq4003",
	  .data = (struct ad400x_chip_info *)&ad400x_chips[ID_ADAQ4003] },
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
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
