// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5760, AD5780, AD5781, AD5790, AD5791 Voltage Output Digital to Analog
 * Converter
 *
 * Copyright 2011 Analog Devices Inc.
 */

#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/dac/ad5791.h>

#define AD5791_DAC_MASK			GENMASK(19, 0)

#define AD5791_CMD_READ			BIT(23)
#define AD5791_CMD_WRITE		0
#define AD5791_ADDR(addr)		((addr) << 20)

/* Registers */
#define AD5791_ADDR_NOOP		0
#define AD5791_ADDR_DAC0		1
#define AD5791_ADDR_CTRL		2
#define AD5791_ADDR_CLRCODE		3
#define AD5791_ADDR_SW_CTRL		4

/* Control Register */
#define AD5791_CTRL_RBUF		BIT(1)
#define AD5791_CTRL_OPGND		BIT(2)
#define AD5791_CTRL_DACTRI		BIT(3)
#define AD5791_CTRL_BIN2SC		BIT(4)
#define AD5791_CTRL_SDODIS		BIT(5)
#define AD5761_CTRL_LINCOMP(x)		((x) << 6)

#define AD5791_LINCOMP_0_10		0
#define AD5791_LINCOMP_10_12		1
#define AD5791_LINCOMP_12_16		2
#define AD5791_LINCOMP_16_19		3
#define AD5791_LINCOMP_19_20		12

#define AD5780_LINCOMP_0_10		0
#define AD5780_LINCOMP_10_20		12

/* Software Control Register */
#define AD5791_SWCTRL_LDAC		BIT(0)
#define AD5791_SWCTRL_CLR		BIT(1)
#define AD5791_SWCTRL_RESET		BIT(2)

#define AD5791_DAC_PWRDN_6K		0
#define AD5791_DAC_PWRDN_3STATE		1

/* Arbitrary sane max sampling rate. datasheet only mentions
 * max SPI bauderate of 35Mbps (x 24 bits)
 */
#define AD5791_MAX_SAMPLING_RATE	1000000

/**
 * struct ad5791_chip_info - chip specific information
 * @get_lin_comp:	function pointer to the device specific function
 */

struct ad5791_chip_info {
	int (*get_lin_comp)	(unsigned int span);
};

/**
 * struct ad5791_state - driver instance specific data
 * @spi:			spi_device
 * @reg_vdd:		positive supply regulator
 * @reg_vss:		negative supply regulator
 * @chip_info:		chip model specific constants
 * @vref_mv:		actual reference voltage used
 * @vref_neg_mv:	voltage of the negative supply
 * @ctrl:		control register cache
 * @pwr_down_mode:	current power down mode
 * @pwr_down:		true if device is powered down
 * @data:		spi transfer buffers
 */
struct ad5791_state {
	struct spi_device		*spi;
	struct regulator		*reg_vdd;
	struct regulator		*reg_vss;
	struct gpio_desc		*gpio_reset;
	struct gpio_desc		*gpio_clear;
	struct gpio_desc		*gpio_ldac;
	const struct ad5791_chip_info	*chip_info;
	struct spi_message		spi_msg;
	struct spi_transfer		spi_transfer;
	struct pwm_device		*cnv_trigger;
	unsigned long			ref_clk_rate;
	unsigned short			vref_mv;
	unsigned int			vref_neg_mv;
	unsigned			ctrl;
	unsigned			pwr_down_mode;
	bool				pwr_down;

	union {
		__be32 d32;
		u8 d8[4];
	} data[3] __aligned(IIO_DMA_MINALIGN);
};

enum ad5791_supported_device_ids {
	ID_AD5760,
	ID_AD5780,
	ID_AD5781,
	ID_AD5791,
};

static int ad5791_spi_write(struct ad5791_state *st, u8 addr, u32 val)
{
	st->data[0].d32 = cpu_to_be32(AD5791_CMD_WRITE |
			      AD5791_ADDR(addr) |
			      (val & AD5791_DAC_MASK));

	return spi_write(st->spi, &st->data[0].d8[1], 3);
}

static int ad5791_spi_read(struct ad5791_state *st, u8 addr, u32 *val)
{
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.bits_per_word = 8,
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d8[1],
			.rx_buf = &st->data[2].d8[1],
			.bits_per_word = 8,
			.len = 3,
		},
	};

	st->data[0].d32 = cpu_to_be32(AD5791_CMD_READ |
			      AD5791_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(AD5791_ADDR(AD5791_ADDR_NOOP));

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));

	*val = be32_to_cpu(st->data[2].d32);

	return ret;
}

static const char * const ad5791_powerdown_modes[] = {
	"6kohm_to_gnd",
	"three_state",
};

static int ad5791_get_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	return st->pwr_down_mode;
}

static int ad5791_set_powerdown_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	st->pwr_down_mode = mode;

	return 0;
}

static const struct iio_enum ad5791_powerdown_mode_enum = {
	.items = ad5791_powerdown_modes,
	.num_items = ARRAY_SIZE(ad5791_powerdown_modes),
	.get = ad5791_get_powerdown_mode,
	.set = ad5791_set_powerdown_mode,
};

static int __ad5791_dac_powerdown(struct iio_dev *indio_dev,
		bool pwr_down)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	if (!pwr_down) {
		st->ctrl &= ~(AD5791_CTRL_OPGND | AD5791_CTRL_DACTRI);
	} else {
		if (st->pwr_down_mode == AD5791_DAC_PWRDN_6K)
			st->ctrl |= AD5791_CTRL_OPGND;
		else if (st->pwr_down_mode == AD5791_DAC_PWRDN_3STATE)
			st->ctrl |= AD5791_CTRL_DACTRI;
	}
	st->pwr_down = pwr_down;

	return ad5791_spi_write(st, AD5791_ADDR_CTRL, st->ctrl);
}

static ssize_t ad5791_read_dac_powerdown(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->pwr_down);
}

static ssize_t ad5791_write_dac_powerdown(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan, const char *buf,
	 size_t len)
{
	bool pwr_down;
	int ret;

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;

	ret = __ad5791_dac_powerdown(indio_dev, pwr_down);

	return ret ? ret : len;
}

static int ad5791_get_lin_comp(unsigned int span)
{
	if (span <= 10000)
		return AD5791_LINCOMP_0_10;
	else if (span <= 12000)
		return AD5791_LINCOMP_10_12;
	else if (span <= 16000)
		return AD5791_LINCOMP_12_16;
	else if (span <= 19000)
		return AD5791_LINCOMP_16_19;
	else
		return AD5791_LINCOMP_19_20;
}

static int ad5780_get_lin_comp(unsigned int span)
{
	if (span <= 10000)
		return AD5780_LINCOMP_0_10;
	else
		return AD5780_LINCOMP_10_20;
}
static const struct ad5791_chip_info ad5791_chip_info_tbl[] = {
	[ID_AD5760] = {
		.get_lin_comp = ad5780_get_lin_comp,
	},
	[ID_AD5780] = {
		.get_lin_comp = ad5780_get_lin_comp,
	},
	[ID_AD5781] = {
		.get_lin_comp = ad5791_get_lin_comp,
	},
	[ID_AD5791] = {
		.get_lin_comp = ad5791_get_lin_comp,
	},
};

static int ad5791_get_sampling_freq(struct ad5791_state *st)
{
	return DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC,
				     pwm_get_period(st->cnv_trigger));
}

static int __ad5791_set_sampling_freq(struct ad5791_state *st, int freq)
{
	struct pwm_state cnv_state;
	u32 rem;

	pwm_init_state(st->cnv_trigger, &cnv_state);
	cnv_state.period = div_u64_rem((u64)DIV_ROUND_UP(st->ref_clk_rate, freq) * NSEC_PER_SEC,
					st->ref_clk_rate, &rem);
	if (rem)
		cnv_state.period += 1;

	cnv_state.duty_cycle = DIV_ROUND_UP(NSEC_PER_SEC, st->ref_clk_rate);

	return pwm_apply_state(st->cnv_trigger, &cnv_state);
}

static int ad5791_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
	struct ad5791_state *st = iio_priv(indio_dev);
	int ret;

	if (!st->cnv_trigger)
		return -ENODEV;

	if (!freq || freq > AD5791_MAX_SAMPLING_RATE)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ad5791_set_sampling_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad5791_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad5791_state *st = iio_priv(indio_dev);
	u64 val64;
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5791_spi_read(st, chan->address, val);
		if (ret)
			return ret;
		*val &= AD5791_DAC_MASK;
		*val >>= chan->scan_type.shift;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = (1 << chan->scan_type.realbits) - 1;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_OFFSET:
		val64 = (((u64)st->vref_neg_mv) << chan->scan_type.realbits);
		do_div(val64, st->vref_mv);
		*val = -val64;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad5791_get_sampling_freq(st);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

};

static const struct iio_chan_spec_ext_info ad5791_ext_info[] = {
	{
		.name = "powerdown",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ad5791_read_dac_powerdown,
		.write = ad5791_write_dac_powerdown,
	},
	IIO_ENUM("powerdown_mode", IIO_SHARED_BY_TYPE,
		 &ad5791_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE, &ad5791_powerdown_mode_enum),
	{ },
};

#define AD5791_CHAN(bits, _shift) {			\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.address = AD5791_ADDR_DAC0,			\
	.channel = 0,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
		BIT(IIO_CHAN_INFO_OFFSET),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = (bits),			\
		.storagebits = 24,			\
		.shift = (_shift),			\
	},						\
	.ext_info = ad5791_ext_info,			\
}

#define AD5791_OFFLOAD_CHAN(bits, _shift) {		\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.address = AD5791_ADDR_DAC0,			\
	.channel = 0,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
		BIT(IIO_CHAN_INFO_OFFSET),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = (bits),			\
		.storagebits = 32,			\
		.shift = (_shift),			\
	},						\
	.ext_info = ad5791_ext_info,			\
}

static const struct iio_chan_spec ad5791_channels[] = {
	[ID_AD5760] = AD5791_CHAN(16, 4),
	[ID_AD5780] = AD5791_CHAN(18, 2),
	[ID_AD5781] = AD5791_CHAN(18, 2),
	[ID_AD5791] = AD5791_CHAN(20, 0)
};

static const struct iio_chan_spec ad5791_offload_channels[] = {
	[ID_AD5760] = AD5791_OFFLOAD_CHAN(16, 4),
	[ID_AD5780] = AD5791_OFFLOAD_CHAN(18, 2),
	[ID_AD5781] = AD5791_OFFLOAD_CHAN(18, 2),
	[ID_AD5791] = AD5791_OFFLOAD_CHAN(20, 0)
};

static int ad5791_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		val &= GENMASK(chan->scan_type.realbits - 1, 0);
		val <<= chan->scan_type.shift;

		return ad5791_spi_write(st, chan->address, val);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad5791_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad5791_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad5791_state *st = iio_priv(indio_dev);
	int ret;

	ret = __ad5791_dac_powerdown(indio_dev, false);
	if (ret)
		return ret;

	ret =  pwm_enable(st->cnv_trigger);
	if (ret) {
		__ad5791_dac_powerdown(indio_dev, true);
		return ret;
	}

	spi_bus_lock(st->spi->master);
	spi_engine_ex_offload_enable(st->spi, true);

	return 0;
}

static int ad5791_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad5791_state *st = iio_priv(indio_dev);

	spi_engine_ex_offload_enable(st->spi, false);
	pwm_disable(st->cnv_trigger);
	spi_bus_unlock(st->spi->master);

	return __ad5791_dac_powerdown(indio_dev, true);
}

static const struct iio_buffer_setup_ops ad5791_buffer_setup_ops = {
	.preenable = &ad5791_buffer_preenable,
	.postdisable = &ad5791_buffer_postdisable,
};

static void ad5791_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static int ad5791_pwm_setup(struct spi_device *spi, struct ad5791_state *st)
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

	ret = devm_add_action_or_reset(&spi->dev, ad5791_pwm_diasble,
				       st->cnv_trigger);
	if (ret)
		return ret;

	return __ad5791_set_sampling_freq(st, AD5791_MAX_SAMPLING_RATE);
}

static const struct iio_info ad5791_info = {
	.read_raw = &ad5791_read_raw,
	.write_raw = &ad5791_write_raw,
};

static int ad5791_probe(struct spi_device *spi)
{
	struct ad5791_platform_data *pdata = spi->dev.platform_data;
	struct iio_dev *indio_dev;
	struct ad5791_state *st;
	int ret, pos_voltage_uv = 0, neg_voltage_uv = 0;
	bool use_rbuf_gain2;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	st->reg_vdd = devm_regulator_get(&spi->dev, "vdd");
	if (!IS_ERR(st->reg_vdd)) {
		ret = regulator_enable(st->reg_vdd);
		if (ret)
			return ret;

		ret = regulator_get_voltage(st->reg_vdd);
		if (ret < 0)
			goto error_disable_reg_pos;

		pos_voltage_uv = ret;
	}

	st->reg_vss = devm_regulator_get(&spi->dev, "vss");
	if (!IS_ERR(st->reg_vss)) {
		ret = regulator_enable(st->reg_vss);
		if (ret)
			goto error_disable_reg_pos;

		ret = regulator_get_voltage(st->reg_vss);
		if (ret < 0)
			goto error_disable_reg_neg;

		neg_voltage_uv = ret;
	}

	st->gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_clear = devm_gpiod_get_optional(&spi->dev, "clear",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_clear))
		return PTR_ERR(st->gpio_clear);

	st->gpio_ldac = devm_gpiod_get_optional(&spi->dev, "ldac",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_ldac))
		return PTR_ERR(st->gpio_ldac);

	st->pwr_down = true;
	st->spi = spi;

	if (pdata)
		use_rbuf_gain2 = pdata->use_rbuf_gain2;
	else
		use_rbuf_gain2 = device_property_read_bool(&spi->dev,
							   "adi,rbuf-gain2-en");

	if (!IS_ERR(st->reg_vss) && !IS_ERR(st->reg_vdd)) {
		st->vref_mv = (pos_voltage_uv + neg_voltage_uv) / 1000;
		st->vref_neg_mv = neg_voltage_uv / 1000;
	} else if (pdata) {
		st->vref_mv = pdata->vref_pos_mv + pdata->vref_neg_mv;
		st->vref_neg_mv = pdata->vref_neg_mv;
	} else {
		dev_warn(&spi->dev, "reference voltage unspecified\n");
	}

	ret = ad5791_spi_write(st, AD5791_ADDR_SW_CTRL, AD5791_SWCTRL_RESET);
	if (ret)
		goto error_disable_reg_neg;

	st->chip_info =	&ad5791_chip_info_tbl[spi_get_device_id(spi)
					      ->driver_data];


	st->ctrl = AD5761_CTRL_LINCOMP(st->chip_info->get_lin_comp(st->vref_mv))
		  | (use_rbuf_gain2 ? 0 : AD5791_CTRL_RBUF) |
		  AD5791_CTRL_BIN2SC;

	ret = ad5791_spi_write(st, AD5791_ADDR_CTRL, st->ctrl |
		AD5791_CTRL_OPGND | AD5791_CTRL_DACTRI);
	if (ret)
		goto error_disable_reg_neg;

	spi_set_drvdata(spi, indio_dev);
	indio_dev->info = &ad5791_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels
		= &ad5791_channels[spi_get_device_id(spi)->driver_data];
	indio_dev->num_channels = 1;
	indio_dev->name = spi_get_device_id(st->spi)->name;

	if (spi_engine_ex_offload_supported(spi)) {
		indio_dev->channels =
			&ad5791_channels[spi_get_device_id(spi)->driver_data];

		st->spi_transfer.len = 4;
		st->spi_transfer.bits_per_word = 24;
		st->spi_transfer.tx_buf = (void *)-1; /* steaming tx */

		spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

		ret = spi_optimize_message(st->spi, &st->spi_msg);
		if (ret)
			goto error_disable_reg_neg;

		ret = spi_engine_ex_offload_load_msg(st->spi, &st->spi_msg);
		if (ret < 0)
			goto error_disable_reg_neg;

		ret = ad5791_pwm_setup(spi, st);
		if (ret)
			goto error_disable_reg_neg;

		ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
						      indio_dev, "tx",
						      IIO_BUFFER_DIRECTION_OUT);
		if (ret)
			goto error_disable_reg_neg;

		indio_dev->setup_ops = &ad5791_buffer_setup_ops;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg_neg;

	return 0;

error_disable_reg_neg:
	if (!IS_ERR(st->reg_vss))
		regulator_disable(st->reg_vss);
error_disable_reg_pos:
	if (!IS_ERR(st->reg_vdd))
		regulator_disable(st->reg_vdd);
	return ret;
}

static void ad5791_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad5791_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!IS_ERR(st->reg_vdd))
		regulator_disable(st->reg_vdd);

	if (!IS_ERR(st->reg_vss))
		regulator_disable(st->reg_vss);
}

static const struct spi_device_id ad5791_id[] = {
	{"ad5760", ID_AD5760},
	{"ad5780", ID_AD5780},
	{"ad5781", ID_AD5781},
	{"ad5790", ID_AD5791},
	{"ad5791", ID_AD5791},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5791_id);

static struct spi_driver ad5791_driver = {
	.driver = {
		   .name = "ad5791",
		   },
	.probe = ad5791_probe,
	.remove = ad5791_remove,
	.id_table = ad5791_id,
};
module_spi_driver(ad5791_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5760/AD5780/AD5781/AD5790/AD5791 DAC");
MODULE_LICENSE("GPL v2");
