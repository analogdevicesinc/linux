// SPDX-License-Identifier: GPL-2.0
/*
 * AD8460 Waveform generator DAC Driver
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define AD8460_CTRL_REG(x)                      (x)
#define AD8460_HVDAC_DATA_WORD_LOW(x)           (0x60 + (2 * (x)))
#define AD8460_HVDAC_DATA_WORD_HIGH(x)          (0x61 + (2 * (x)))

#define AD8460_HV_RESET_MSK                     BIT(7)
#define AD8460_HV_SLEEP_MSK                     BIT(4)
#define AD8460_WAVE_GEN_MODE_MSK                BIT(0)

#define AD8460_HVDAC_SLEEP_MSK                  BIT(3)

#define AD8460_APG_MODE_ENABLE_MSK              BIT(5)
#define AD8460_PATTERN_DEPTH_MSK                GENMASK(3, 0)

#define AD8460_SHUTDOWN_FLAG_MSK                BIT(7)
#define AD8460_DATA_BYTE_LOW_MSK                GENMASK(7, 0)
#define AD8460_DATA_BYTE_HIGH_MSK               GENMASK(5, 0)

struct ad8460_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *sync_clk;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex lock;
	u32 cache_apg_idx;
	u32 rset_ohms;
	int vref_mv;
};

static int ad8460_hv_reset(struct ad8460_state *state)
{
	int ret;

	ret = regmap_update_bits(state->regmap,	AD8460_CTRL_REG(0x00),
				 AD8460_HV_RESET_MSK,
				 FIELD_PREP(AD8460_HV_RESET_MSK, 1));
	if (ret)
		return ret;

	fsleep(20);

	return regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_HV_RESET_MSK,
				 FIELD_PREP(AD8460_HV_RESET_MSK, 0));
}

static int ad8460_reset(const struct ad8460_state *state)
{
	struct device *dev = &state->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return dev_err_probe(dev, PTR_ERR(gpio),
				     "Failed to get reset gpio");
	if (gpio) {
		fsleep(100);
		gpiod_set_value_cansleep(gpio, 1);
	} else {
		return regmap_write(state->regmap, AD8460_CTRL_REG(0x03), 1);
	}
	fsleep(100);

	return 0;
}

static int ad8460_enable_apg_mode(struct ad8460_state *state, int val)
{
	int ret;

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x02),
				 AD8460_APG_MODE_ENABLE_MSK,
				 FIELD_PREP(AD8460_APG_MODE_ENABLE_MSK, val));
	if (ret)
		return ret;

	return regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_WAVE_GEN_MODE_MSK,
				 FIELD_PREP(AD8460_WAVE_GEN_MODE_MSK, val));
}

static ssize_t ad8460_read_powerdown(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	int ret, reg;

	ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x01), &reg);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%ld\n", FIELD_GET(AD8460_HVDAC_SLEEP_MSK, reg));
}

static ssize_t ad8460_write_powerdown(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf,
				      size_t len)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	bool pwr_down;
	int sdn_flag;
	int ret;

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;

	guard(mutex)(&state->lock);

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x01),
				 AD8460_HVDAC_SLEEP_MSK,
				 FIELD_PREP(AD8460_HVDAC_SLEEP_MSK, pwr_down));
	if (ret)
		return ret;

	if (!pwr_down) {
		ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x0E),
				  &sdn_flag);
		if (ret)
			return ret;

		sdn_flag = FIELD_GET(AD8460_SHUTDOWN_FLAG_MSK, sdn_flag);

		if (sdn_flag) {
			ret = ad8460_hv_reset(state);
			if (ret)
				return ret;
		}
	}

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x00),
				 AD8460_HV_SLEEP_MSK,
				 FIELD_PREP(AD8460_HV_SLEEP_MSK, !pwr_down));
	if (ret)
		return ret;

	return len;
}

static const char * const ad8460_powerdown_modes[] = {
	"three_state",
};

static int ad8460_get_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	return 0;
}

static int ad8460_set_powerdown_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int type)
{
	return 0;
}

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

	*val = FIELD_GET(AD8460_DATA_BYTE_HIGH_MSK, high) << 8 | low;

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
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad8460_enable_apg_mode(state, 1);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;

		scoped_guard(mutex, &state->lock) {
			ret = ad8460_set_hvdac_byte(state, 0, val);
			if (ret)
				return ret;

			reg = FIELD_PREP(AD8460_PATTERN_DEPTH_MSK, 0);
			ret = regmap_update_bits(state->regmap,
						 AD8460_CTRL_REG(0x02),
						 AD8460_PATTERN_DEPTH_MSK, reg);
		}

		return ret;
	default:
		return -EINVAL;
	}
}

static int ad8460_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int num, denom;
	int data, ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		scoped_guard(mutex, &state->lock) {
			ret = ad8460_get_hvdac_byte(state, 0, &data);
			if (ret)
				return ret;
		}

		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(state->sync_clk);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* vCONV = 80V * (DAC_CODE / 2**14) - 40V
		 * vMAX = 80V * (2**14 / 2**14) - 40V
		 * vMIN = 80V * (0 / 2**14) - 40V
		 * vADJ = vCONV * (2000 / rSET) * (vREF / 1.2)
		 * vSPAN = vADJ_MAX - vADJ_MIN
		 * See datasheet page 49, section FULL-SCALE REDUCTION
		 */
		num = 80 * 2000 * state->vref_mv;
		denom = state->rset_ohms * 1200;
		*val = DIV_ROUND_CLOSEST_ULL(num, denom);

		return IIO_VAL_INT;
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

static int ad8460_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad8460_state *state = iio_priv(indio_dev);

	return ad8460_enable_apg_mode(state, 0);
}

static int ad8460_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad8460_state *state = iio_priv(indio_dev);

	return ad8460_enable_apg_mode(state, 1);
}

static const struct iio_buffer_setup_ops ad8460_buffer_setup_ops = {
	.preenable = &ad8460_buffer_preenable,
	.postdisable = &ad8460_buffer_postdisable,
};

static const struct iio_info ad8460_info = {
	.read_raw = &ad8460_read_raw,
	.write_raw = &ad8460_write_raw,
	.debugfs_reg_access = &ad8460_reg_access,
};

static const struct iio_enum ad8460_powerdown_mode_enum = {
	.items = ad8460_powerdown_modes,
	.num_items = ARRAY_SIZE(ad8460_powerdown_modes),
	.get = ad8460_get_powerdown_mode,
	.set = ad8460_set_powerdown_mode,
};

#define AD8460_CHAN_EXT_INFO(_name, _what, _shared, _read, _write) {	\
	.name = _name,							\
	.read = (_read),						\
	.write = (_write),						\
	.private = (_what),						\
	.shared = (_shared),						\
}

static struct iio_chan_spec_ext_info ad8460_ext_info[] = {
	AD8460_CHAN_EXT_INFO("powerdown", 0, IIO_SEPARATE,
			     ad8460_read_powerdown, ad8460_write_powerdown),
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad8460_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE,
			   &ad8460_powerdown_mode_enum),
	{}
};

#define AD8460_ALTVOLTAGE_CHAN(_chan) {				\
	.type = IIO_ALTVOLTAGE,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
			      BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
	.output = 1,						\
	.indexed = 1,						\
	.channel = (_chan),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 14,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},                                                      \
	.ext_info = ad8460_ext_info,                            \
}

static const struct iio_chan_spec ad8460_channels[] = {
	AD8460_ALTVOLTAGE_CHAN(0),
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

static int ad8460_set_apg_pattern_depth(void *arg, u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct ad8460_state *state = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x02),
				 AD8460_PATTERN_DEPTH_MSK,
				 FIELD_PREP(AD8460_PATTERN_DEPTH_MSK, val));
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad8460_show_apg_pattern_depth(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad8460_state *state = iio_priv(indio_dev);
	u32 reg;
	int ret;

	ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x02), &reg);
	if (ret)
		return ret;

	*val = FIELD_GET(AD8460_PATTERN_DEPTH_MSK, reg);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ad8460_apg_pattern_depth_fops,
			 ad8460_show_apg_pattern_depth,
			 ad8460_set_apg_pattern_depth, "%llu\n");

static ssize_t ad8460_apg_pattern_memory_write(struct file *file,
					       const char __user *userbuf,
					       size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct ad8460_state *state = iio_priv(indio_dev);
	unsigned int reg;
	char data[16];
	int ret, val;

	ret = simple_write_to_buffer(data, sizeof(data) - 1, ppos,
				     userbuf, count);
	if (ret <= 0)
		return ret;

	ret = sscanf(data, "%i 0x%X", &reg, &val);

	switch (ret) {
	case 1:
		state->cache_apg_idx = reg;
		break;
	case 2:
		state->cache_apg_idx = reg;
		scoped_guard(mutex, &state->lock) {
			ret = ad8460_set_hvdac_byte(state, reg, val);
			if (ret) {
				dev_err(indio_dev->dev.parent, "%s: write failed\n",
					__func__);
				return ret;
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t ad8460_apg_pattern_memory_read(struct file *file,
					      char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct ad8460_state *state = iio_priv(indio_dev);
	int ret, val;
	char data[16];

	scoped_guard(mutex, &state->lock) {
		ret = ad8460_get_hvdac_byte(state, state->cache_apg_idx, &val);
		if (ret)
			return ret;
	}

	ret = scnprintf(data, sizeof(data), "%i 0x%X\n", state->cache_apg_idx, val);

	return simple_read_from_buffer(userbuf, count, ppos, data, ret);
}

static const struct file_operations ad8460_apg_pattern_memory_fops = {
	.open = simple_open,
	.read = ad8460_apg_pattern_memory_read,
	.write = ad8460_apg_pattern_memory_write,
};

static int ad8460_show_shutdown_flag(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad8460_state *state = iio_priv(indio_dev);
	u32 flag;
	int ret;

	ret = regmap_read(state->regmap, AD8460_CTRL_REG(0x0E), &flag);
	if (ret)
		return ret;

	*val = FIELD_GET(AD8460_SHUTDOWN_FLAG_MSK, flag);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ad8460_shutdown_flag_fops, ad8460_show_shutdown_flag,
			 NULL, "%llu\n");

static void ad8460_debugfs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_create_file_unsafe("apg_pattern_depth", 0600, d,
				   indio_dev, &ad8460_apg_pattern_depth_fops);
	debugfs_create_file_unsafe("shutdown_flag", 0600, d,
				   indio_dev, &ad8460_shutdown_flag_fops);
	debugfs_create_file("apg_pattern_memory", 0644, d,
			    indio_dev, &ad8460_apg_pattern_memory_fops);
}

static int ad8460_probe(struct spi_device *spi)
{
	struct ad8460_state *state;
	struct iio_dev *indio_dev;
	struct regulator *vrefio;
	struct clk *sync_clk;
	u32 temp;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	mutex_init(&state->lock);

	state->spi = spi;

	state->regmap = devm_regmap_init_spi(spi, &ad8460_regmap_config);
	if (IS_ERR(state->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(state->regmap),
				     "Failed to initialize regmap");

	indio_dev->name = "ad8460";
	indio_dev->channels = ad8460_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad8460_channels);
	indio_dev->info = &ad8460_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad8460_buffer_setup_ops;

	ret = devm_iio_dmaengine_buffer_setup(&spi->dev, indio_dev, "tx",
					      IIO_BUFFER_DIRECTION_OUT);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to get DMA buffer\n");

	sync_clk = devm_clk_get_enabled(&spi->dev, "sync_clk");
	if (IS_ERR(sync_clk))
		return dev_err_probe(&spi->dev, PTR_ERR(sync_clk),
				     "Failed to get sync clk\n");

	state->sync_clk = sync_clk;

	vrefio = devm_regulator_get_optional(&spi->dev, "vrefio");
	if (IS_ERR(vrefio)) {
		if (PTR_ERR(vrefio) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(vrefio),
					"Failed to get vref regulator\n");

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

	ret = device_property_read_u32(&spi->dev, "adi,rset-ohms", &temp);
	if (!ret) {
		if (temp < 2000 || temp > 20000)
			return dev_err_probe(&spi->dev, -EINVAL,
					"Invalid value for rset: %u\n", temp);
		state->rset_ohms = temp;
	}

	ret = ad8460_reset(state);
	if (ret)
		return ret;

	/* Enables DAC by default */
	ret = regmap_update_bits(state->regmap, AD8460_CTRL_REG(0x01),
				 AD8460_HVDAC_SLEEP_MSK,
				 FIELD_PREP(AD8460_HVDAC_SLEEP_MSK, 0));
	if (ret)
		return ret;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	ad8460_debugfs_init(indio_dev);

	return 0;
}

static const struct of_device_id ad8460_of_match[] = {
	{ .compatible = "adi, ad8460" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad8460_of_match);

static struct spi_driver ad8460_driver = {
	.driver = {
		.name = "ad8460",
		.of_match_table = ad8460_of_match,
	},
	.probe = ad8460_probe,
};
module_spi_driver(ad8460_driver);

MODULE_AUTHOR("Mariel Tinaco <mariel.tinaco@analog.com");
MODULE_DESCRIPTION("AD8460 DAC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
