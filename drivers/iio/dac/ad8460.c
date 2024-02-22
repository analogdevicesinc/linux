// SPDX-License-Identifier: GPL-2.0
/*
 * AD8460 Waveform generator DAC Driver
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define AD8460_CTRL_REG(x)                      (x)
#define AD8460_HVDAC_DATA_WORD_LOW(x)           (0x60 + (2 * x))
#define AD8460_HVDAC_DATA_WORD_HIGH(x)          (0x61 + (2 * x))

#define AD8460_HV_RESET_MSK                     BIT(7)
#define AD8460_HV_SLEEP_MSK                     BIT(4)
#define AD8460_WAVE_GEN_MODE_MSK                BIT(0)

#define AD8460_HVDAC_SLEEP_MSK                  BIT(3)

#define AD8460_APG_MODE_ENABLE_MSK              BIT(5)
#define AD8460_PATTERN_DEPTH_MSK                GENMASK(3, 0)

#define AD8460_FAULT_ARM_MSK			BIT(7)
#define AD8460_FAULT_LIMIT_MSK			GENMASK(6, 0)

#define AD8460_SHUTDOWN_FLAG_MSK                BIT(7)
#define AD8460_DATA_BYTE_LOW_MSK                GENMASK(7, 0)
#define AD8460_DATA_BYTE_HIGH_MSK               GENMASK(5, 0)

#define AD8460_PATTERN_MEMORY_AVAIL_N		16
#define AD8460_MAX_OVERCURRENT_MICROAMP		1000000
#define AD8460_MAX_OVERVOLTAGE_MICROVOLT	55000000
#define AD8460_MAX_OVERTEMP_MILLICELSIUS	150000

struct ad8460_state {
	struct regmap *regmap;
	struct clk *sync_clk;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex lock;
	unsigned int input_mode;
	bool pwr_down;
	bool enable;
	int vref_mv;
	u32 rset_ohms;
};

enum ad8460_apg_settings {
	AD8460_PATTERN_DEPTH,
	AD8460_PATTERN_MEMORY,
};

enum ad8460_fault_categories {
	AD8460_OVERCURRENT_SRC,
	AD8460_OVERCURRENT_SNK,
	AD8460_OVERVOLTAGE_POS,
	AD8460_OVERVOLTAGE_NEG,
	AD8460_OVERTEMPERATURE,
};

enum ad8460_input_mode {
	AD8460_INPUT_MODE_AWG,
	AD8460_INPUT_MODE_APG
};

static const char * const ad8460_input_modes[] = {
	[AD8460_INPUT_MODE_AWG] = "awg",
	[AD8460_INPUT_MODE_APG] = "apg"
};

static int ad8460_hv_reset(struct ad8460_state *state)
{
	int ret;

	ret = regmap_update_bits(state->regmap,	AD8460_CTRL_REG(0x00),
				 AD8460_HV_RESET_MSK,
				 FIELD_PREP(AD8460_HV_RESET_MSK, 1));
	if (ret)
		return ret;

	return regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_HV_RESET_MSK,
				 FIELD_PREP(AD8460_HV_RESET_MSK, 0));
}

static int ad8460_chan_enable(struct ad8460_state *state, int val)
{
	int sdn_flag;
	int ret;

	val = val & 0x1;

	mutex_lock(&state->lock);
	if (val) {
		ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x0E),
				  &sdn_flag);
		if (ret)
			goto out_unlock;

		sdn_flag = FIELD_GET(AD8460_SHUTDOWN_FLAG_MSK, sdn_flag);

		if (sdn_flag) {
			ret = ad8460_hv_reset(state);
			if (ret)
				goto out_unlock;
		}
	}

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_HV_SLEEP_MSK,
				 FIELD_PREP(AD8460_HV_SLEEP_MSK, val));
	if (ret)
		goto out_unlock;

	state->enable = val;
out_unlock:
	mutex_unlock(&state->lock);
	return ret;
}

static int ad8460_input_mode_write(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int item)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	int ret;

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x02),
				 AD8460_APG_MODE_ENABLE_MSK,
				 FIELD_PREP(AD8460_APG_MODE_ENABLE_MSK, item));
	if (ret)
		return ret;

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_WAVE_GEN_MODE_MSK,
				 FIELD_PREP(AD8460_WAVE_GEN_MODE_MSK, item));
	if (ret)
		return ret;

	state->input_mode = item;

	return 0;
}

static int ad8460_input_mode_read(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad8460_state *state = iio_priv(indio_dev);

	return state->input_mode;
}

static const struct iio_enum ad8460_input_mode_enum = {
	.items = ad8460_input_modes,
	.num_items = ARRAY_SIZE(ad8460_input_modes),
	.set = ad8460_input_mode_write,
	.get = ad8460_input_mode_read,
};

static int ad8460_get_hvdac_byte(struct ad8460_state *state,
				 int index,
				 int *val)
{
	unsigned int high, low;
	int ret;

	ret = regmap_read(state->regmap, AD8460_HVDAC_DATA_WORD_HIGH(index),
			  &high);
	if (ret)
		return ret;

	ret = regmap_read(state->regmap, AD8460_HVDAC_DATA_WORD_LOW(index),
			  &low);
	if (ret)
		return ret;

	*val = (FIELD_GET(AD8460_DATA_BYTE_HIGH_MSK, high) << 8) | low;

	return ret;
}

static int ad8460_set_hvdac_byte(struct ad8460_state *state,
				 int index,
				 int val)
{
	int ret;

	ret = regmap_write(state->regmap, AD8460_HVDAC_DATA_WORD_LOW(index),
			   (val & 0xFF));
	if (ret)
		return ret;

	return regmap_write(state->regmap, AD8460_HVDAC_DATA_WORD_HIGH(index),
			    ((val >> 8) & 0xFF));
}

static int ad8460_read_powerdown(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 char *buf)
{
	struct ad8460_state *state = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", state->pwr_down);
}

static int ad8460_write_powerdown(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf,
				  size_t len)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	bool pwr_down;
	int ret;

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;

	mutex_lock(&state->lock);
	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x01),
				 AD8460_HVDAC_SLEEP_MSK,
				 FIELD_PREP(AD8460_HVDAC_SLEEP_MSK, !pwr_down));
	if (ret)
		goto out_unlock;

	state->pwr_down = pwr_down;
out_unlock:
	mutex_unlock(&state->lock);
	return ret ? ret : len;
}

static ssize_t ad8460_pattern_memory_get(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int reg;
	int sz = 0;
	long mem;
	int ret;
	u32 m;

	switch (private) {
	case AD8460_PATTERN_MEMORY:
		for (m = 0; m < AD8460_PATTERN_MEMORY_AVAIL_N; m++) {
			ret = ad8460_get_hvdac_byte(state, m, &reg);
			if (ret)
				return ret;

			sz += sysfs_emit_at(buf, sz, "%ld ", (long)reg);
		}
		buf[sz - 1] = '\n';

		return sz;
	case AD8460_PATTERN_DEPTH:
		ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x02), &reg);
		if (ret)
			return ret;

		mem = FIELD_GET(AD8460_PATTERN_DEPTH_MSK, reg);
		return sysfs_emit(buf, "%ld\n", mem);
	default:
		return -EINVAL;
	}

	return sysfs_emit(buf, "%ld\n", mem);
}

static ssize_t ad8460_pattern_memory_set(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 const char *buf, size_t len)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int reg;
	char bufcpy[100];
	char *p, *q;
	long val;
	int ret;

	p = strcpy(bufcpy, buf);

	if (private == AD8460_PATTERN_MEMORY) {
		q = strsep(&p, ",");

		ret = kstrtol(p, 10, &val);
		if (ret)
			return ret;

		ret = kstrtou32(q, 10, &reg);
		if (ret)
			return ret;

		ret = ad8460_set_hvdac_byte(state, reg, val);
		if (ret)
			return ret;

		return len;
	}

	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;

	if (private == AD8460_PATTERN_DEPTH) {
		reg = FIELD_PREP(AD8460_PATTERN_DEPTH_MSK, val);
		ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x02),
					 AD8460_PATTERN_DEPTH_MSK, reg);
		if (ret)
			return ret;
	}

	return ret ? ret : len;
}

static int ad8460_arm(struct ad8460_state *state,
		      unsigned int fault_type,
		      unsigned int arm, int limit)
{
	int reg;

	switch (fault_type) {
	case AD8460_OVERCURRENT_SRC:
		if (limit > AD8460_MAX_OVERCURRENT_MICROAMP || limit < 0)
			return -EINVAL;

		reg = (arm << 7) | (limit / 15625);
		break;
	case AD8460_OVERCURRENT_SNK:
		if (limit > AD8460_MAX_OVERCURRENT_MICROAMP || limit < 0)
			return -EINVAL;

		reg = (arm << 7) | (limit / 15625);
		break;
	case AD8460_OVERVOLTAGE_POS:
		if (limit > AD8460_MAX_OVERVOLTAGE_MICROVOLT || limit < 0)
			return -EINVAL;

		reg = (arm << 7) | (limit / 1953000);
		break;
	case AD8460_OVERVOLTAGE_NEG:
		limit = -limit;
		if (limit > AD8460_MAX_OVERVOLTAGE_MICROVOLT || limit < 0)
			return -EINVAL;

		reg = (arm << 7) | (limit / 1953000);
		break;
	case AD8460_OVERTEMPERATURE:
		if (limit > AD8460_MAX_OVERTEMP_MILLICELSIUS || limit < 0)
			return -EINVAL;

		reg = (arm << 7) | ((limit + 266640) / 6510);
		break;
	default:
		return -EINVAL;
	}

	return regmap_write(state->regmap,
			    AD8460_CTRL_REG((0x08 + fault_type)),
			    reg);
}

static ssize_t ad8460_span_get(struct iio_dev *indio_dev,
			       uintptr_t private,
			       const struct iio_chan_spec *chan,
			       char *buf)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	long span;

	span = 80 * (2000 / state->rset_ohms) * (state->vref_mv / 1000);

	return sysfs_emit(buf, "%ld\n", span);
}

static int ad8460_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val =  state->enable;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = ad8460_get_hvdac_byte(state, 0, &data);
		if (ret)
			return ret;

		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(state->sync_clk);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad8460_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		return ad8460_chan_enable(state, val);
	case IIO_CHAN_INFO_RAW:
		ret = ad8460_set_hvdac_byte(state, 0, val);
		if (ret)
			return ret;

		reg = FIELD_PREP(AD8460_PATTERN_DEPTH_MSK, 0);
		return regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x02),
					  AD8460_PATTERN_DEPTH_MSK, reg);
	default:
		return -EINVAL;
	}
}

static int ad8460_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct ad8460_state *state = iio_priv(indio_dev);

	if (readval)
		return regmap_read(state->regmap, reg, readval);

	return regmap_write(state->regmap, reg, writeval);
}

static const struct iio_info ad8460_info = {
	.read_raw = &ad8460_read_raw,
	.write_raw = &ad8460_write_raw,
	.debugfs_reg_access = &ad8460_reg_access,
};

#define AD8460_CHAN_EXT_INFO(_name, _what, _shared, _read, _write) {	\
	.name = _name,							\
	.read = (_read),						\
	.write = (_write),						\
	.private = (_what),						\
	.shared = (_shared),						\
}

static struct iio_chan_spec_ext_info ad8460_ext_info[] = {
	IIO_ENUM("input_mode", IIO_SEPARATE, &ad8460_input_mode_enum),
	IIO_ENUM_AVAILABLE("input_mode", IIO_SEPARATE, &ad8460_input_mode_enum),
	AD8460_CHAN_EXT_INFO("powerdown", 0, IIO_SEPARATE,
			     ad8460_read_powerdown, ad8460_write_powerdown),
	AD8460_CHAN_EXT_INFO("pattern_memory", AD8460_PATTERN_MEMORY,
			     IIO_SHARED_BY_ALL, ad8460_pattern_memory_get,
			     ad8460_pattern_memory_set),
	AD8460_CHAN_EXT_INFO("pattern_depth", AD8460_PATTERN_DEPTH,
			     IIO_SHARED_BY_ALL, ad8460_pattern_memory_get,
			     ad8460_pattern_memory_set),
	AD8460_CHAN_EXT_INFO("span", 0, IIO_SHARED_BY_ALL, ad8460_span_get,
			     NULL),
	{},
};

#define AD8460_VOLTAGE_CHAN(_channel) {				\
	.type = IIO_ALTVOLTAGE,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),	\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.scan_index = _channel,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 14,					\
		.storagebits = 16,				\
		.shift = 0,					\
		.endianness = IIO_LE,				\
	},                                                      \
	.ext_info = ad8460_ext_info,                            \
}

static const struct iio_chan_spec ad8460_channels[] = {
	AD8460_VOLTAGE_CHAN(0)
};

static const struct regmap_config ad8460_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
};

static void ad8460_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad8460_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad8460_probe(struct spi_device *spi)
{
	struct ad8460_state *state;
	struct iio_dev *indio_dev;
	struct regulator *vrefio;
	struct clk *sync_clk;
	u32 tmp[2], temp;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	mutex_init(&state->lock);

	state->enable = true;
	state->regmap = devm_regmap_init_spi(spi, &ad8460_regmap_config);
	if (IS_ERR(state->regmap))
		return PTR_ERR(state->regmap);
	state->input_mode = AD8460_INPUT_MODE_AWG;

	indio_dev->name = "ad8460";
	indio_dev->channels = ad8460_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad8460_channels);
	indio_dev->info = &ad8460_info;

	sync_clk = devm_clk_get(&spi->dev, "sync_clk");
	if (IS_ERR(sync_clk))
		return dev_err_probe(&spi->dev, PTR_ERR(sync_clk),
				     "Failed to get sync clk\n");

	ret = clk_prepare_enable(sync_clk);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "failed to enable trigger clk\n");

	ret = devm_add_action_or_reset(&spi->dev, ad8460_clk_disable, sync_clk);
	if (ret)
		return ret;

	state->sync_clk = sync_clk;

	vrefio = devm_regulator_get_optional(&spi->dev, "vrefio");
	if (IS_ERR(vrefio)) {
		if (PTR_ERR(vrefio) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(vrefio),
					"Failed to get vref regulator\n");

		/* internal reference */
		state->vref_mv = 1200;

	} else {
		ret = regulator_enable(vrefio);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					"Failed to enable vrefio regulator\n");

		ret = devm_add_action_or_reset(&spi->dev,
					       ad8460_regulator_disable,
					       vrefio);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vrefio);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to get vrefio\n");

		if (ret < 120000 || ret > 1200000)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "Invalid vrefio voltage\n");

		state->vref_mv = ret / 1000;
	}

	ret = device_property_read_u32(&spi->dev, "adi,rset-ohms",
				       &state->rset_ohms);
	if (ret)
		state->rset_ohms = 2000;

	/* Soft reset */
	ret = regmap_write(state->regmap, AD8460_CTRL_REG(0x03), 1);
	if (ret)
		return ret;

	/* Arm the device by default */
	ret = device_property_read_u32_array(&spi->dev, "adi,ilim-microamp",
					     tmp, ARRAY_SIZE(tmp));
	if (!ret) {
		ret = ad8460_arm(state, AD8460_OVERCURRENT_SNK, 0x1, tmp[0]);
		if (ret)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "overcurrent snk not valid: %d",
					     tmp[0]);

		ret = ad8460_arm(state, AD8460_OVERCURRENT_SRC, 0x1, tmp[1]);
		if (ret)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "overcurrent src not valid: %d",
					     tmp[1]);
	}

	ret = device_property_read_u32_array(&spi->dev, "adi,vlim-microvolt",
					     tmp, ARRAY_SIZE(tmp));
	if (!ret) {
		ret = ad8460_arm(state, AD8460_OVERVOLTAGE_NEG, 0x1, tmp[0]);
		if (ret)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "positive overvoltage not valid: %d",
					     tmp[0]);

		ret = ad8460_arm(state, AD8460_OVERVOLTAGE_POS, 0x1, tmp[1]);
		if (ret)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "negative overvoltage not valid: %d",
					     tmp[1]);
	}

	ret = device_property_read_u32(&spi->dev, "adi,temp-lim-millicelsius",
				       &temp);
	if (!ret) {
		ret = ad8460_arm(state, AD8460_OVERTEMPERATURE, 0x1, temp);
		if (ret)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "overtemperature not valid: %d",
					     temp);
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad8460_of_match[] = {
	{ .compatible = "adi, ad8460" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad8460_of_match);

static struct spi_driver ad8460_driver = {
	.driver = {
		.name = "ad8460",
	},
	.probe = ad8460_probe,
};
module_spi_driver(ad8460_driver);

MODULE_AUTHOR("Mariel Tinaco <mariel.tinaco@analog.com");
MODULE_DESCRIPTION("AD8460 DAC driver");
MODULE_LICENSE("GPL v2");
