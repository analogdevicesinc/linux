// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD4040/AD4044/AD4046/AD4048 ADC driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/log2.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define AD4040_REG_INTERFACE_CONFIG_B			0x01
#define   AD4040_INTERFACE_CONFIG_B_SHORT_INST		BIT(3)
#define AD4040_REG_TEST_PATTERN				0x21
#define   AD4040_TEST_PATTERN_SEL_MSK			GENMASK(6, 5)
#define   AD4040_TEST_PATTERN_EN			BIT(4)
#define AD4040_REG_SYNC_CTRL				0x22
#define   AD4040_SYNC_CTRL_SYNCB			BIT(0)
#define AD4040_REG_FILTER_CONFIG			0x29
#define   AD4040_FILTER_CONFIG_SINC_DEC_RATE_MSK	GENMASK(6, 3)
#define   AD4040_FILTER_CONFIG_FILTER_SEL_MSK		GENMASK(1, 0)
#define AD4040_REG_MAX					0x1154

#define AD4040_SPI_READ					BIT(7)
/* Highest register address reachable with short (8-bit) instructions */
#define AD4040_SHORT_INST_REG_MAX			0x7f

#define AD4040_NUM_CHANNELS				8
#define AD4040_FULL_SCALE_MV				6000
#define AD4040_SYNC_TIMEOUT_US				10000

enum ad4040_filter_type {
	AD4040_FILTER_NONE,
	AD4040_FILTER_SINC1,
	AD4040_FILTER_SINC5,
	AD4040_FILTER_SINC5_PF1,
};

static const char * const ad4040_filter_types[] = {
	[AD4040_FILTER_NONE] = "none",
	[AD4040_FILTER_SINC1] = "sinc1",
	[AD4040_FILTER_SINC5] = "sinc5",
	[AD4040_FILTER_SINC5_PF1] = "sinc5+pf1",
};

static const int ad4040_osr_avail[] = {
	2, 4, 8, 16, 32, 64, 128, 256, 512, 1024,
};

static const int ad4040_osr_none[] = { 1 };

static const char * const ad4040_lane_modes[] = {
	[IIO_BACKEND_LANE_MODE_DUAL] = "dual",
	[IIO_BACKEND_LANE_MODE_SINGLE] = "single",
	[IIO_BACKEND_LANE_MODE_HALF] = "half",
	[IIO_BACKEND_LANE_MODE_QUARTER] = "quarter",
};

struct ad4040_chip_info {
	const char *name;
	unsigned int resolution;
};

struct ad4040_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct iio_backend *back;
	const struct ad4040_chip_info *info;
	unsigned long clk_rate;
	/* Serializes multi-step device configuration sequences. */
	struct mutex lock;
	/*
	 * Used as the regmap lock. Serializes register accesses with changes
	 * of the SPI instruction mode.
	 */
	struct mutex bus_lock;
	enum ad4040_filter_type filter_type;
	enum iio_backend_lane_mode lane_mode;
	unsigned int num_lanes;
	/*
	 * When true, register accesses use 8-bit instructions (16 SCLK cycles),
	 * otherwise 16-bit instructions (24 SCLK cycles). Must always match the
	 * instruction mode of the device.
	 */
	bool short_instruction;
};

/*
 * Short instructions carry a 7-bit register address in one byte, long
 * instructions a 15-bit address in two bytes. Bit 7 of the first byte is the
 * read flag. spi_write_then_read() uses DMA safe bounce buffers.
 */
static int ad4040_spi_reg_write(void *context, unsigned int reg,
				unsigned int val)
{
	struct ad4040_state *st = context;
	u8 tx[3];

	if (st->short_instruction) {
		if (reg > AD4040_SHORT_INST_REG_MAX)
			return -EINVAL;

		tx[0] = reg;
		tx[1] = val;
		return spi_write_then_read(st->spi, tx, 2, NULL, 0);
	}

	put_unaligned_be16(reg & ~(AD4040_SPI_READ << 8), tx);
	tx[2] = val;

	return spi_write_then_read(st->spi, tx, 3, NULL, 0);
}

static int ad4040_spi_reg_read(void *context, unsigned int reg,
			       unsigned int *val)
{
	struct ad4040_state *st = context;
	unsigned int tx_len;
	u8 tx[2], rx;
	int ret;

	if (st->short_instruction) {
		if (reg > AD4040_SHORT_INST_REG_MAX)
			return -EINVAL;

		tx[0] = reg | AD4040_SPI_READ;
		tx_len = 1;
	} else {
		put_unaligned_be16(reg | (AD4040_SPI_READ << 8), tx);
		tx_len = 2;
	}

	ret = spi_write_then_read(st->spi, tx, tx_len, &rx, 1);
	if (ret)
		return ret;

	*val = rx;

	return 0;
}

static void ad4040_regmap_lock(void *arg)
{
	struct ad4040_state *st = arg;

	mutex_lock(&st->bus_lock);
}

static void ad4040_regmap_unlock(void *arg)
{
	struct ad4040_state *st = arg;

	mutex_unlock(&st->bus_lock);
}

static const struct regmap_config ad4040_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = AD4040_REG_MAX,
	.reg_read = ad4040_spi_reg_read,
	.reg_write = ad4040_spi_reg_write,
	.lock = ad4040_regmap_lock,
	.unlock = ad4040_regmap_unlock,
};

static int ad4040_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4040_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4040_get_osr(struct ad4040_state *st, int *val)
{
	unsigned int reg;
	int ret;

	if (st->filter_type == AD4040_FILTER_NONE) {
		*val = 1;
		return IIO_VAL_INT;
	}

	ret = regmap_read(st->regmap, AD4040_REG_FILTER_CONFIG, &reg);
	if (ret)
		return ret;

	*val = BIT(FIELD_GET(AD4040_FILTER_CONFIG_SINC_DEC_RATE_MSK, reg) + 1);

	return IIO_VAL_INT;
}

static int ad4040_osr_max(enum ad4040_filter_type type)
{
	return type >= AD4040_FILTER_SINC5 ? 256 : 1024;
}

static int ad4040_set_osr(struct ad4040_state *st, int val)
{
	guard(mutex)(&st->lock);

	if (st->filter_type == AD4040_FILTER_NONE)
		return val == 1 ? 0 : -EINVAL;

	if (val < 2 || val > ad4040_osr_max(st->filter_type) ||
	    !is_power_of_2(val))
		return -EINVAL;

	return regmap_update_bits(st->regmap, AD4040_REG_FILTER_CONFIG,
				  AD4040_FILTER_CONFIG_SINC_DEC_RATE_MSK,
				  FIELD_PREP(AD4040_FILTER_CONFIG_SINC_DEC_RATE_MSK,
					     ilog2(val) - 1));
}

static int ad4040_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad4040_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = AD4040_FULL_SCALE_MV;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->clk_rate;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4040_get_osr(st, val);
	default:
		return -EINVAL;
	}
}

static int ad4040_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct ad4040_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4040_set_osr(st, val);
	default:
		return -EINVAL;
	}
}

static int ad4040_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	struct ad4040_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (st->filter_type == AD4040_FILTER_NONE) {
			*vals = ad4040_osr_none;
			*length = ARRAY_SIZE(ad4040_osr_none);
		} else {
			*vals = ad4040_osr_avail;
			*length = ilog2(ad4040_osr_max(st->filter_type));
		}
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad4040_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad4040_state *st = iio_priv(indio_dev);
	unsigned int ch;
	int ret;

	for (ch = 0; ch < indio_dev->num_channels; ch++) {
		if (test_bit(ch, scan_mask))
			ret = iio_backend_chan_enable(st->back, ch);
		else
			ret = iio_backend_chan_disable(st->back, ch);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct iio_info ad4040_iio_info = {
	.read_raw = ad4040_read_raw,
	.write_raw = ad4040_write_raw,
	.read_avail = ad4040_read_avail,
	.update_scan_mode = ad4040_update_scan_mode,
	.debugfs_reg_access = ad4040_reg_access,
};

static int ad4040_get_filter_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad4040_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(st->regmap, AD4040_REG_FILTER_CONFIG, &reg);
	if (ret)
		return ret;

	return FIELD_GET(AD4040_FILTER_CONFIG_FILTER_SEL_MSK, reg);
}

static int ad4040_set_filter_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int type)
{
	struct ad4040_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD4040_REG_FILTER_CONFIG, &reg);
	if (ret)
		return ret;

	/* The sinc5 filters do not support the two highest decimation rates */
	if (type >= AD4040_FILTER_SINC5 &&
	    FIELD_GET(AD4040_FILTER_CONFIG_SINC_DEC_RATE_MSK, reg) + 1 >
	    ilog2(ad4040_osr_max(type)))
		return -EINVAL;

	ret = iio_backend_filter_type_set(st->back, type);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4040_REG_FILTER_CONFIG,
				 AD4040_FILTER_CONFIG_FILTER_SEL_MSK,
				 FIELD_PREP(AD4040_FILTER_CONFIG_FILTER_SEL_MSK,
					    type));
	if (ret)
		return ret;

	st->filter_type = type;

	return 0;
}

static const struct iio_enum ad4040_filter_type_enum = {
	.items = ad4040_filter_types,
	.num_items = ARRAY_SIZE(ad4040_filter_types),
	.get = ad4040_get_filter_type,
	.set = ad4040_set_filter_type,
};

static int ad4040_get_lane_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct ad4040_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return st->lane_mode;
}

static int ad4040_set_lane_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int mode)
{
	struct ad4040_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	ret = iio_backend_lane_mode_set(st->back, mode);
	if (ret)
		return ret;

	st->lane_mode = mode;

	return 0;
}

static const struct iio_enum ad4040_lane_mode_enum = {
	.items = ad4040_lane_modes,
	.num_items = ARRAY_SIZE(ad4040_lane_modes),
	.get = ad4040_get_lane_mode,
	.set = ad4040_set_lane_mode,
};

static const struct iio_chan_spec_ext_info ad4040_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ad4040_filter_type_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_ALL,
			   &ad4040_filter_type_enum),
	IIO_ENUM("lane_mode", IIO_SHARED_BY_ALL, &ad4040_lane_mode_enum),
	IIO_ENUM_AVAILABLE("lane_mode", IIO_SHARED_BY_ALL,
			   &ad4040_lane_mode_enum),
	{ }
};

/*
 * Align the backend to the data lanes: make the device send its fixed test
 * pattern, restart the output framing by toggling SYNCB and let the backend
 * lock onto the pattern.
 */
static int ad4040_lvds_sync(struct ad4040_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int pattern;
	int ret, ret2;

	guard(mutex)(&st->lock);

	/* The bring-up sequence discards the first read of this register */
	ret = regmap_read(st->regmap, AD4040_REG_TEST_PATTERN, &pattern);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4040_REG_TEST_PATTERN, &pattern);
	if (ret)
		return ret;

	pattern &= ~(AD4040_TEST_PATTERN_SEL_MSK | AD4040_TEST_PATTERN_EN);
	ret = regmap_write(st->regmap, AD4040_REG_TEST_PATTERN,
			   pattern | AD4040_TEST_PATTERN_EN);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4040_REG_SYNC_CTRL,
			   AD4040_SYNC_CTRL_SYNCB);
	if (ret)
		goto restore_pattern;

	ret = regmap_write(st->regmap, AD4040_REG_SYNC_CTRL, 0);
	if (ret)
		goto restore_pattern;

	ret = regmap_write(st->regmap, AD4040_REG_SYNC_CTRL,
			   AD4040_SYNC_CTRL_SYNCB);
	if (ret)
		goto restore_pattern;

	ret = iio_backend_interface_data_align(st->back, AD4040_SYNC_TIMEOUT_US);
	if (ret)
		dev_err(dev, "Data alignment process failed: %d\n", ret);

restore_pattern:
	ret2 = regmap_write(st->regmap, AD4040_REG_TEST_PATTERN, pattern);

	return ret ?: ret2;
}

static int ad4040_set_short_instruction(struct ad4040_state *st, bool enable)
{
	int ret;

	guard(mutex)(&st->bus_lock);

	if (st->short_instruction == enable)
		return 0;

	/*
	 * The mode switch is written with the current instruction length. Only
	 * the following accesses use the new one.
	 */
	ret = ad4040_spi_reg_write(st, AD4040_REG_INTERFACE_CONFIG_B,
				   enable ? AD4040_INTERFACE_CONFIG_B_SHORT_INST : 0);
	if (ret)
		return ret;

	st->short_instruction = enable;

	return 0;
}

static int ad4040_debugfs_lvds_sync_set(void *arg, u64 val)
{
	if (!val)
		return -EINVAL;

	return ad4040_lvds_sync(arg);
}
DEFINE_DEBUGFS_ATTRIBUTE(ad4040_lvds_sync_fops, NULL,
			 ad4040_debugfs_lvds_sync_set, "%llu\n");

static int ad4040_debugfs_long_inst_get(void *arg, u64 *val)
{
	struct ad4040_state *st = arg;

	guard(mutex)(&st->bus_lock);
	*val = !st->short_instruction;

	return 0;
}

static int ad4040_debugfs_long_inst_set(void *arg, u64 val)
{
	return ad4040_set_short_instruction(arg, !val);
}
DEFINE_DEBUGFS_ATTRIBUTE(ad4040_long_inst_fops, ad4040_debugfs_long_inst_get,
			 ad4040_debugfs_long_inst_set, "%llu\n");

/* Interface bring-up controls */
static void ad4040_debugfs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct ad4040_state *st = iio_priv(indio_dev);

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_create_file_unsafe("lvds_sync", 0200, d, st,
				   &ad4040_lvds_sync_fops);
	debugfs_create_file_unsafe("long_instruction", 0600, d, st,
				   &ad4040_long_inst_fops);

	iio_backend_debugfs_add(st->back, indio_dev);
}

static int ad4040_setup_channels(struct iio_dev *indio_dev)
{
	struct ad4040_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *channels;
	unsigned int ch;

	channels = devm_kcalloc(dev, AD4040_NUM_CHANNELS, sizeof(*channels),
				GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (ch = 0; ch < AD4040_NUM_CHANNELS; ch++) {
		channels[ch] = (struct iio_chan_spec) {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = ch,
			.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
			.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
			.info_mask_shared_by_all_available =
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
			.ext_info = ad4040_ext_info,
			.scan_index = ch,
			.scan_type = {
				.sign = 's',
				.realbits = st->info->resolution,
				.storagebits = 32,
			},
		};
	}

	indio_dev->channels = channels;
	indio_dev->num_channels = AD4040_NUM_CHANNELS;

	return 0;
}

static int ad4040_probe(struct spi_device *spi)
{
	struct regmap_config regmap_config = ad4040_regmap_config;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4040_state *st;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &st->bus_lock);
	if (ret)
		return ret;

	regmap_config.lock_arg = st;
	st->regmap = devm_regmap_init(dev, NULL, st, &regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	st->num_lanes = 1;
	device_property_read_u32(dev, "adi,num-lanes", &st->num_lanes);
	if (!st->num_lanes || st->num_lanes > 2)
		return dev_err_probe(dev, -EINVAL,
				     "invalid 'adi,num-lanes' value: %u\n",
				     st->num_lanes);

	clk = devm_clk_get_enabled(dev, "cnv");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get conversion clock\n");

	st->clk_rate = clk_get_rate(clk);

	st->back = devm_iio_backend_get(dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(dev, PTR_ERR(st->back),
				     "failed to get backend\n");

	ret = devm_iio_backend_enable(dev, st->back);
	if (ret)
		return ret;

	ret = iio_backend_num_lanes_set(st->back, st->num_lanes);
	if (ret)
		return ret;

	ret = iio_backend_data_size_set(st->back, st->info->resolution);
	if (ret)
		return ret;

	/* Start in dual lane mode, without filter */
	ret = iio_backend_lane_mode_set(st->back, IIO_BACKEND_LANE_MODE_DUAL);
	if (ret)
		return ret;

	st->lane_mode = IIO_BACKEND_LANE_MODE_DUAL;

	ret = iio_backend_filter_type_set(st->back,
					  IIO_BACKEND_FILTER_TYPE_DISABLED);
	if (ret)
		return ret;

	st->filter_type = AD4040_FILTER_NONE;

	indio_dev->name = st->info->name;
	indio_dev->info = &ad4040_iio_info;

	ret = ad4040_setup_channels(indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_request_buffer(dev, st->back, indio_dev);
	if (ret)
		return ret;

	/*
	 * The device is not accessed here. On the evaluation setup it is
	 * released from reset and clocked from user space, so the interface
	 * alignment (lvds_sync) is triggered at runtime.
	 */
	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	ad4040_debugfs_init(indio_dev);

	return 0;
}

static const struct ad4040_chip_info ad4040_chip_info = {
	.name = "ad4040",
	.resolution = 20,
};

static const struct ad4040_chip_info ad4044_chip_info = {
	.name = "ad4044",
	.resolution = 14,
};

static const struct ad4040_chip_info ad4046_chip_info = {
	.name = "ad4046",
	.resolution = 16,
};

static const struct ad4040_chip_info ad4048_chip_info = {
	.name = "ad4048",
	.resolution = 18,
};

static const struct spi_device_id ad4040_id[] = {
	{ "ad4040", (kernel_ulong_t)&ad4040_chip_info },
	{ "ad4044", (kernel_ulong_t)&ad4044_chip_info },
	{ "ad4046", (kernel_ulong_t)&ad4046_chip_info },
	{ "ad4048", (kernel_ulong_t)&ad4048_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4040_id);

static const struct of_device_id ad4040_of_match[] = {
	{ .compatible = "adi,ad4040", .data = &ad4040_chip_info },
	{ .compatible = "adi,ad4044", .data = &ad4044_chip_info },
	{ .compatible = "adi,ad4046", .data = &ad4046_chip_info },
	{ .compatible = "adi,ad4048", .data = &ad4048_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4040_of_match);

static struct spi_driver ad4040_driver = {
	.driver = {
		.name = "ad4040",
		.of_match_table = ad4040_of_match,
	},
	.probe = ad4040_probe,
	.id_table = ad4040_id,
};
module_spi_driver(ad4040_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4040 ADC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
