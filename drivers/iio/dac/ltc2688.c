// SPDX-License-Identifier: GPL-2.0+
/*
 * LTC2688 16 channel, 16 bit Voltage Output SoftSpan DAC driver
 *
 * Copyright 2021 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned/be_byteshift.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define LTC2688_DAC_CHANNELS	16

#define LTC2688_CMD_CH_CODE(x)			(0x00 + x)
#define LTC2688_CMD_CH_SETTING(x)		(0x10 + x)
#define LTC2688_CMD_CH_OFFSET(x)		(0X20 + x)
#define LTC2688_CMD_CH_GAIN(x)			(0x30 + x)
#define LTC2688_CMD_CH_CODE_UPDATE(x)		(0x40 + x)
#define LTC2688_CMD_CH_CODE_UPDATE_ALL(x)	(0x50 + x)
#define LTC2688_CMD_CH_UPDATE(x)		(0x60 + x)

#define LTC2688_CMD_CONFIG_REG			0x70
#define LTC2688_CMD_POWERDOWN_REG		0x71
#define LTC2688_CMD_A_B_SELECT_REG		0x72
#define LTC2688_CMD_SW_TOGGLE_REG		0x73
#define LTC2688_CMD_TOGGLE_DITHER_EN_REG	0x74
#define LTC2688_CMD_MUX_CTRL_REG		0x75
#define LTC2688_CMD_FAULT_REG			0x76
#define LTC2688_CMD_CODE_ALL			0x78
#define LTC2688_CMD_CODE_UPDATE_ALL		0x79
#define LTC2688_CMD_SETTING_ALL			0x7A
#define LTC2688_CMD_SETTING_UPDATE_ALL		0x7B
#define LTC2688_CMD_UPDATE_ALL			0x7C
#define LTC2688_CMD_NOOP			0xFF

#define LTC2688_READ_OPERATION			0x80

/* Channel Settings */
#define LTC2688_CH_SPAN_MSK			GENMASK(3, 0)
#define LTC2688_CH_SPAN(x)			FIELD_PREP(LTC2688_CH_SPAN_MSK, x)
#define LTC2688_CH_OVERRANGE			BIT(3)
#define LTC2688_CH_TD_SEL_MSK			GENMASK(5, 4)
#define LTC2688_CH_TD_SEL(x)			FIELD_PREP(LTC2688_CH_TD_SEL_MSK, x)
#define LTC2688_CH_DIT_PER_MSK			GENMASK(8, 6)
#define LTC2688_CH_DIT_PER(x)			FIELD_PREP(LTC2688_CH_DIT_PER_MSK, x)
#define LTC2688_CH_DIT_PH_MSK			GENMASK(10, 9)
#define LTC2688_CH_DIT_PH(x)			FIELD_PREP(LTC2688_CH_DIT_PH_MSK, x)
#define LTC2688_CH_MODE				BIT(11)


/* Configuration register */
#define LTC2688_CONFIG_RST			BIT(15)
#define LTC2688_CONFIG_EXT_REF			BIT(1)

enum ltc2688_voltage_range {
	LTC2688_VOLTAGE_RANGE_0V_5V,
	LTC2688_VOLTAGE_RANGE_0V_10V,
	LTC2688_VOLTAGE_RANGE_M5V_5V,
	LTC2688_VOLTAGE_RANGE_M10V_10V,
	LTC2688_VOLTAGE_RANGE_M15V_15V,
};


/**
 * struct ltc2688_state - driver instance specific data
 * @spi:		spi_device
 * @vref_reg:		fixed regulator for reference configuration
 * @lock:		lock for spi access
 * @crt_range:		channel voltage range
 * @overrange:		5% overrange spans
 * @data:		transfer data
 */
struct ltc2688_state {
	struct spi_device		*spi;
	struct regulator		*vref_reg;
	struct mutex			lock; /* protect spi access */
	enum ltc2688_voltage_range	crt_range[16];
	bool				overrange[16];

	union {
		u32	d32;
		u8	b8[4];
	} data[3] ____cacheline_aligned;
};

struct ltc2688_span_tbl {
	int min;
	int max;
};

static const struct ltc2688_span_tbl ltc2688_span_tbl[] = {
	[LTC2688_VOLTAGE_RANGE_0V_5V] = {0, 5000},
	[LTC2688_VOLTAGE_RANGE_0V_10V] = {0, 10000},
	[LTC2688_VOLTAGE_RANGE_M5V_5V] = {-5000, 5000},
	[LTC2688_VOLTAGE_RANGE_M10V_10V] = {-10000, 10000},
	[LTC2688_VOLTAGE_RANGE_M15V_15V] = {-15000, 15000},
};

static const char * const ltc2688_dither_period[] = {
	"4",
	"8",
	"16",
	"32",
	"64",
};

static const char * const ltc2688_dither_phase[] = {
	"0",
	"90",
	"180",
	"270",
};

static const char * const  ltc2688_a_b_register[] = {
	"select_a_reg",
	"select_b_reg",
};

static int ltc2688_spi_read(struct ltc2688_state *st, u8 reg, int *val)
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

	st->data[0].d32 = reg | LTC2688_READ_OPERATION;
	st->data[1].d32 = LTC2688_CMD_NOOP;

	mutex_lock(&st->lock);
	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		goto err_unlock;

	*val = (st->data[2].b8[1] << 8) + st->data[2].b8[2];

err_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int ltc2688_spi_write(struct ltc2688_state *st, u8 command, u16 data)
{
	int ret;

	st->data[0].b8[0] = command;
	put_unaligned_be16(data, &st->data[0].b8[1]);

	mutex_lock(&st->lock);
	ret = spi_write(st->spi, &st->data[0].b8[0], 3);
	mutex_unlock(&st->lock);

	return ret;
}

static int ltc2688_spi_update_bits(struct ltc2688_state *st, u8 reg, u16 mask, u16 val)
{
	int regval;
	int ret;

	ret = ltc2688_spi_read(st, reg, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return ltc2688_spi_write(st, reg, regval);
}

static int ltc2688_get_vref(struct ltc2688_state *st, int *vref)
{
	int ret;

	if (st->vref_reg) {
		ret = regulator_get_voltage(st->vref_reg);
		if (ret < 0)
			return ret;

		*vref = ret / 1000;
	} else {
		*vref = 4096; /* internal vref in uV */
	}

	return 0;
}

static int ltc2688_get_full_scale(struct ltc2688_state *st, int channel)
{
	int fs;

	fs = ltc2688_span_tbl[st->crt_range[channel]].max -
		ltc2688_span_tbl[st->crt_range[channel]].min;

	if (st->overrange[channel]) {
		fs = mult_frac(fs * 1000UL, 105, 100);
		fs /= 1000;
	}

	return fs;
}

static int ltc2688_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long m)
{
	struct ltc2688_state *st = iio_priv(indio_dev);
	int ret, vref, fs;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ltc2688_spi_read(st, LTC2688_CMD_CH_CODE(chan->address), val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		if (ltc2688_span_tbl[st->crt_range[chan->address]].min < 0)
			*val = -32767;
		else
			*val = 0;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = ltc2688_get_vref(st, &vref);
		if (ret < 0)
			return ret;

		fs = ltc2688_get_full_scale(st, chan->address);
		*val =  DIV_ROUND_CLOSEST(fs * vref, 4096);
		*val2 = 16;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ltc2688_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ltc2688_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		return ltc2688_spi_write(st,
					 LTC2688_CMD_CH_CODE_UPDATE(chan->address), val);
	default:
		return -EINVAL;
	}
}

static ssize_t ltc2688_read_dac_powerdown(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  char *buf)
{
	struct ltc2688_state *st = iio_priv(indio_dev);
	int val, ret;

	ret = ltc2688_spi_read(st, LTC2688_CMD_POWERDOWN_REG, &val);

	return sprintf(buf, "%d\n", !!(val & (1 << chan->channel)));
}

static ssize_t ltc2688_write_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   const char *buf,
					   size_t len)
{
	bool pwr_down;
	int ret;
	struct ltc2688_state *st = iio_priv(indio_dev);
	u16 powerdown = 0;

	ret = strtobool(buf, &pwr_down);
	if (ret)
		return ret;

	if (pwr_down)
		powerdown |= (1 << chan->channel);
	else
		powerdown &= ~(1 << chan->channel);

	ret = ltc2688_spi_update_bits(st, LTC2688_CMD_POWERDOWN_REG,
				      (1 << chan->channel), powerdown);

	return ret ? ret : len;
}

static int ltc2688_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct ltc2688_state *st = iio_priv(indio_dev);

	if (readval)
		return ltc2688_spi_read(st, reg, readval);
	else
		return ltc2688_spi_write(st, reg, writeval);
}

static const struct iio_info ltc2688_info = {
	.write_raw	= ltc2688_write_raw,
	.read_raw	= ltc2688_read_raw,
	.debugfs_reg_access = ltc2688_reg_access,
};

enum {
	LTC2688_DITHER_ENABLE,
	LTC2688_DITHER_MODE,
	LTC2688_SOFT_TOGGLE,
};

static ssize_t ltc2688_read_ext(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				char *buf)
{
	struct ltc2688_state *st = iio_priv(indio_dev);
	int ret, regval;
	u8 reg;
	u16 mask;

	switch (private) {
	case LTC2688_DITHER_ENABLE:
		reg = LTC2688_CMD_TOGGLE_DITHER_EN_REG;
		mask = BIT(chan->channel);
		break;
	case LTC2688_DITHER_MODE:
		reg = LTC2688_CMD_CH_SETTING(chan->channel);
		mask = LTC2688_CH_MODE;
		break;
	case LTC2688_SOFT_TOGGLE:
		reg = LTC2688_CMD_SW_TOGGLE_REG;
		mask = BIT(chan->channel);
		break;
	default:
		return -EINVAL;
	}

	ret = ltc2688_spi_read(st, reg, &regval);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", !!(regval & mask));
}

static ssize_t ltc2688_write_ext(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	struct ltc2688_state *st = iio_priv(indio_dev);
	bool readin;
	int ret;
	u8 reg;
	u16 mask, val = 0;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	switch (private) {
	case LTC2688_DITHER_ENABLE:
		reg = LTC2688_CMD_TOGGLE_DITHER_EN_REG;
		mask = BIT(chan->channel);
		break;
	case LTC2688_DITHER_MODE:
		reg = LTC2688_CMD_CH_SETTING(chan->channel);
		mask = LTC2688_CH_MODE;
		break;
	case LTC2688_SOFT_TOGGLE:
		reg = LTC2688_CMD_SW_TOGGLE_REG;
		mask = BIT(chan->channel);
		break;
	default:
		return -EINVAL;
	}

	if (readin)
		val = mask;

	ret = ltc2688_spi_update_bits(st, reg, mask, val);

	return ret ? ret : len;
}

static int ltc2688_get_dither_period(struct iio_dev *dev,
				     const struct iio_chan_spec *chan)
{
	struct ltc2688_state *st = iio_priv(dev);
	int ret, regval;
	u8 period;

	ret = ltc2688_spi_read(st, LTC2688_CMD_CH_SETTING(chan->address), &regval);
	if (ret < 0)
		return ret;

	period = FIELD_GET(LTC2688_CH_DIT_PER_MSK, regval);

	return period;
}

static int ltc2688_set_dither_period(struct iio_dev *dev,
				     const struct iio_chan_spec *chan,
				     unsigned int period)
{
	struct ltc2688_state *st = iio_priv(dev);

	return ltc2688_spi_update_bits(st, LTC2688_CMD_CH_SETTING(chan->address),
				       LTC2688_CH_DIT_PER_MSK, LTC2688_CH_DIT_PER(period));
}

static const struct iio_enum ltc2688_dither_period_enum = {
	.items = ltc2688_dither_period,
	.num_items = ARRAY_SIZE(ltc2688_dither_period),
	.set = ltc2688_set_dither_period,
	.get = ltc2688_get_dither_period,
};

static int ltc2688_get_dither_phase(struct iio_dev *dev,
				    const struct iio_chan_spec *chan)
{
	struct ltc2688_state *st = iio_priv(dev);
	int ret, regval;
	u8 period;

	ret = ltc2688_spi_read(st, LTC2688_CMD_CH_SETTING(chan->address), &regval);
	if (ret < 0)
		return ret;

	period = FIELD_GET(LTC2688_CH_DIT_PH_MSK, regval);

	return period;
}

static int ltc2688_set_dither_phase(struct iio_dev *dev,
				    const struct iio_chan_spec *chan,
				    unsigned int phase)
{
	struct ltc2688_state *st = iio_priv(dev);

	return ltc2688_spi_update_bits(st, LTC2688_CMD_CH_SETTING(chan->address),
				       LTC2688_CH_DIT_PH_MSK, LTC2688_CH_DIT_PH(phase));
}

static const struct iio_enum ltc2688_dither_phase_enum = {
	.items = ltc2688_dither_phase,
	.num_items = ARRAY_SIZE(ltc2688_dither_phase),
	.set = ltc2688_set_dither_phase,
	.get = ltc2688_get_dither_phase,
};

static int ltc2688_get_reg_select(struct iio_dev *dev,
				  const struct iio_chan_spec *chan)
{
	struct ltc2688_state *st = iio_priv(dev);
	int ret, regval;

	ret = ltc2688_spi_read(st, LTC2688_CMD_A_B_SELECT_REG, &regval);
	if (ret < 0)
		return ret;

	return !!(regval & BIT(chan->address));
}

static int ltc2688_set_reg_select(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int reg)
{
	struct ltc2688_state *st = iio_priv(dev);

	return ltc2688_spi_update_bits(st, LTC2688_CMD_A_B_SELECT_REG,
				       BIT(chan->address), reg << chan->address);
}

static const struct iio_enum ltc2688_a_b_reg_enum = {
	.items = ltc2688_a_b_register,
	.num_items = ARRAY_SIZE(ltc2688_a_b_register),
	.set = ltc2688_set_reg_select,
	.get = ltc2688_get_reg_select,
};


#define _LTC2688_CHAN_EXT_INFO(_name, _what, _shared) {	\
	.name = _name,					\
	.read = ltc2688_read_ext,			\
	.write = ltc2688_write_ext,			\
	.private = _what,				\
	.shared = _shared,				\
}

#define _LTC2688_POWERDOWN {			\
	.name = "powerdown",			\
	.read = ltc2688_read_dac_powerdown,	\
	.write = ltc2688_write_dac_powerdown,	\
	.shared = IIO_SEPARATE,			\
}

#define IIO_ENUM_AVAILABLE_SHARED(_name, _shared, _e) {	\
	.name = (_name "_available"),			\
	.shared = _shared,				\
	.read = iio_enum_available_read,		\
	.private = (uintptr_t)(_e),			\
}

static const struct iio_chan_spec_ext_info ltc2688_ext_info[] = {
	_LTC2688_POWERDOWN,
	IIO_ENUM("a_b_register", IIO_SEPARATE, &ltc2688_a_b_reg_enum),
	IIO_ENUM_AVAILABLE_SHARED("a_b_register",
			  IIO_SHARED_BY_ALL, &ltc2688_a_b_reg_enum),
	IIO_ENUM("dither_period", IIO_SEPARATE, &ltc2688_dither_period_enum),
	IIO_ENUM_AVAILABLE_SHARED("dither_period",
				  IIO_SHARED_BY_ALL, &ltc2688_dither_period_enum),
	IIO_ENUM("dither_phase", IIO_SEPARATE, &ltc2688_dither_phase_enum),
	IIO_ENUM_AVAILABLE_SHARED("dither_phase",
				  IIO_SHARED_BY_ALL, &ltc2688_dither_phase_enum),
	_LTC2688_CHAN_EXT_INFO("dither_toggle_en", LTC2688_DITHER_ENABLE, IIO_SEPARATE),
	_LTC2688_CHAN_EXT_INFO("dither_mode", LTC2688_DITHER_MODE, IIO_SEPARATE),
	_LTC2688_CHAN_EXT_INFO("soft_toggle", LTC2688_SOFT_TOGGLE, IIO_SEPARATE),
	{}
};

#define LTC2688_CHANNEL(_chan) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (_chan),						\
	.address = (_chan),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),	\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 16,						\
		.storagebits = 16,					\
	},								\
	.ext_info = ltc2688_ext_info,					\
}

const struct iio_chan_spec ltc2688_channels[] = {
	LTC2688_CHANNEL(0),
	LTC2688_CHANNEL(1),
	LTC2688_CHANNEL(2),
	LTC2688_CHANNEL(3),
	LTC2688_CHANNEL(4),
	LTC2688_CHANNEL(5),
	LTC2688_CHANNEL(6),
	LTC2688_CHANNEL(7),
	LTC2688_CHANNEL(8),
	LTC2688_CHANNEL(9),
	LTC2688_CHANNEL(10),
	LTC2688_CHANNEL(11),
	LTC2688_CHANNEL(12),
	LTC2688_CHANNEL(13),
	LTC2688_CHANNEL(14),
	LTC2688_CHANNEL(15),
};

static int ltc2688_store_output_range(struct ltc2688_state *st, int min, int max, int index)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ltc2688_span_tbl); i++) {
		if (ltc2688_span_tbl[i].min != min ||
		    ltc2688_span_tbl[i].max != max)
			continue;
		st->crt_range[index] = i;

		return 0;
	}

	return -EINVAL;
}

static int ltc2688_channel_config(struct ltc2688_state *st)
{
	int ret, tmp[2], min, max, clk_input;
	unsigned int reg;
	struct fwnode_handle *child;
	uint8_t span;

	device_for_each_child_node(&st->spi->dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return ret;
		if (reg >= LTC2688_DAC_CHANNELS)
			return -EINVAL;

		ret = fwnode_property_read_u32_array(child,
						     "adi,output-range-millivolt",
						     tmp, 2);
		if (ret)
			return ret;

		min = tmp[0];
		max = tmp[1];
		ret = ltc2688_store_output_range(st, min, max, reg);
		if (ret)
			return ret;

		ret = fwnode_property_read_u32(child, "adi,toggle-dither-clk-input", &clk_input);
		if (ret)
			return ret;

		st->overrange[reg] = fwnode_property_read_bool(child, "adi,overrange");

		if (st->overrange[reg])
			span = LTC2688_CH_SPAN(st->crt_range[reg]) | LTC2688_CH_OVERRANGE;
		else
			span = LTC2688_CH_SPAN(st->crt_range[reg]);

		ret = ltc2688_spi_update_bits(st, LTC2688_CMD_CH_SETTING(reg),
					      LTC2688_CH_SPAN_MSK | LTC2688_CH_TD_SEL_MSK,
					      span | LTC2688_CH_TD_SEL(clk_input));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ltc2688_setup(struct ltc2688_state *st)
{
	int ret;

	/* Reset device */
	ret = ltc2688_spi_update_bits(st, LTC2688_CMD_CONFIG_REG,
				      LTC2688_CONFIG_RST, LTC2688_CONFIG_RST);
	if (ret < 0)
		return ret;

	/* Configure channels */
	ret = ltc2688_channel_config(st);
	if (ret < 0)
		return ret;

	/* Setup reference */
	if (st->vref_reg) {
		ret = ltc2688_spi_update_bits(st, LTC2688_CMD_CONFIG_REG,
					      LTC2688_CONFIG_EXT_REF, LTC2688_CONFIG_EXT_REF);
		if (ret < 0)
			return ret;
	}

	/* Update all channel configurations */
	return ltc2688_spi_write(st, LTC2688_CMD_UPDATE_ALL, 0);
}

static void ltc2688_disable_regulator(void *data)
{
	struct ltc2688_state *st = data;

	regulator_disable(st->vref_reg);
}

static int ltc2688_probe(struct spi_device *spi)
{
	struct ltc2688_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	mutex_init(&st->lock);

	st->vref_reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(st->vref_reg)) {
		ret = regulator_enable(st->vref_reg);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to enable vref regulators: %d\n", ret);
			return ret;
		}

		ret = devm_add_action_or_reset(&spi->dev,
					       ltc2688_disable_regulator,
					       st);
		if (ret < 0)
			return ret;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ltc2688_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2688_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2688_channels);

	ret = ltc2688_setup(st);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static const struct of_device_id ltc2688_of_id[] = {
	{ .compatible = "adi,ltc2688", },
	{},
};
MODULE_DEVICE_TABLE(of, ltc2688_of_id);

static const struct spi_device_id ltc2688_id[] = {
	{ "ltc2688", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, ltc2688_id);

static struct spi_driver ltc2688_driver = {
	.driver = {
		.name = "ltc2688",
		.of_match_table = ltc2688_of_id,
	},
	.probe = ltc2688_probe,
	.id_table = ltc2688_id,
};

module_spi_driver(ltc2688_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2688 DAC");
MODULE_LICENSE("GPL v2");
