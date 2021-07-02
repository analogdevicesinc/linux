// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * ADL5960 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/sysfs.h>

/* ADL5960 Register Map */

#define ADL5960_REG_ADI_SPI_CONFIG		0x00
#define ADL5960_REG_SPI_CONFIG_B		0x01
#define ADL5960_REG_DEVICE_CONFIG		0x02
#define ADL5960_REG_CHIPTYPE			0x03
#define ADL5960_REG_PRODUCT_ID_L		0x04
#define ADL5960_REG_PRODUCT_ID_H		0x05
#define ADL5960_REG_LO_CONFIG			0x20
#define ADL5960_REG_CT2				0x21
#define ADL5960_REG_CT4				0x22
#define ADL5960_REG_IGAIN			0x23
#define ADL5960_REG_RGAIN			0x24
#define ADL5960_REG_CIF2_CIF1			0x25
#define ADL5960_REG_TDEG			0x26

/* ADL5960_REG_DEVICE_CONFIG */
#define ADL5960_OPERATING_MODE_MSK	GENMASK(3, 2)
#define ADL5960_POWER_MODE_MSK		GENMASK(1, 0)

/* ADL5960_REG_LO_CONFIG */
#define ADL5960_BYPASS_MSK		BIT(4)
#define ADL5960_IFMODE_MSK		GENMASK(3, 2)
#define ADL5960_LOMODE_MSK		GENMASK(1, 0)

/* ADL5960_REG_CT2 */
#define ADL5960_CT2_MSK			GENMASK(4, 0)

/* ADL5960_REG_CT4 */
#define ADL5960_CT4_MSK			GENMASK(4, 0)

/* ADL5960_REG_IGAIN */
#define ADL5960_IGAIN_MSK		GENMASK(6, 0)

/* ADL5960_REG_RGAIN */
#define ADL5960_RGAIN_MSK		GENMASK(6, 0)

/* ADL5960_REG_CIF2_CIF1 */
#define ADL5960_CIF2_MSK		GENMASK(7, 4)
#define ADL5960_CIF1_MSK		GENMASK(3, 0)

/* ADL5960_REG_TDEG */
#define ADL5960_TDEG_MSK		GENMASK(4, 0)

enum supported_parts {
	ADL5960,
};

enum {
	ADL5960_EXTI_LO_FREQ,
	ADL5960_EXTI_LO_NOTIFY_EN,
	ADL5960_EXTI_OFS_FREQ,
	ADL5960_EXTI_IF_FREQ,
	ADL5960_EXTI_CIF1,
	ADL5960_EXTI_CIF2,
};

static const unsigned int adl5920_cif_cutoff_lut[][2] = {
	{7000000, 700000},
	{7600000, 800000},
	{8100000, 860000},
	{8800000, 990000},
	{9500000, 1000000},
	{10500000, 1200000},
	{11600000, 1300000},
	{13000000, 1500000},
	{14500000, 1600000},
	{16700000, 2000000},
	{19700000, 2300000},
	{24100000, 3100000},
	{30100000, 3800000},
	{41000000, 5900000},
	{64000000, 11100000},
	{126000000, 126000000},
};

static const unsigned char tdeg_to_celcius[] = {
	-48, -41, -34, -27, -20, -14, -7, 0, 6, 13, 19,
	25, 32, 39, 45, 52, 58, 65, 72, 78, 85, 91, 97,
	104, 110, 117, 123, 130, 136, 142, 149, 155,
};

static const unsigned char ct2_settings_vs_freq_ghz[] = {
	31, 31, 31, 31, 31, 31, 20, 12, 9,
	6, 4, 3, 2, 2, 1, 0, 0, 0, 0, 0, 0
};

static const unsigned char ct4_settings_vs_freq_ghz[] = {
	255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
	255, 255, 255, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9,
	9, 6, 5, 5, 4, 3, 3, 3, 2, 2, 2, 2, 2, 1, 1, 0, 0
};

static const char * const adl5960_lo_modes[] = {
	[0] = "bypass",
	[1] = "/2",
	[2] = "x1",
	[3] = "x2",
	[4] = "x4",
};

static const char * const adl5960_if_modes[] = {
	[0] = "x1",
	[1] = "/2",
	[2] = "/4",
	[3] = "off",
	[4] = "bypass",
};

struct adl5960_dev {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clkin_lo;
	struct clk		*clkin_offs;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	struct regulator	*reg;
	u64			lo_freq;
	bool			notify_en;
	u8			lo_freq_mult;
	u8			lo_freq_div;
	u8			offs_freq_mult;
	u8			offs_freq_div;
	u8			ct2_cached;
	u8			ct4_cached;
};

static const struct reg_sequence adl5960_reg_defaults[] = {
	{ ADL5960_REG_ADI_SPI_CONFIG, 0x99 },
	{ ADL5960_REG_ADI_SPI_CONFIG, 0x0 },
	{ ADL5960_REG_SPI_CONFIG_B, 0x00 },
	{ ADL5960_REG_DEVICE_CONFIG, 0x00 },
};

static const struct regmap_config adl5960_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.use_single_read = true,
};

static unsigned int adl5960_update_cif_filters(u32 freq)
{
	int i, c1 = 255, c2 = 255;

	for (i = 0; i < ARRAY_SIZE(adl5920_cif_cutoff_lut); i++) {
		if (freq <= adl5920_cif_cutoff_lut[i][0] && c1 == 255)
			c1 = 15 - i;
		if (freq <= adl5920_cif_cutoff_lut[i][1] && c2 == 255)
			c2 = 15 - i;
	}

	if (c1 == 255)
		c1 = 0;

	if (c2 == 255)
		c2 = 0;

	return FIELD_PREP(ADL5960_CIF1_MSK, c1) |
		FIELD_PREP(ADL5960_CIF2_MSK, c2);
}

static int adl5960_update_ct2_filter(struct adl5960_dev *dev, u32 freq)
{
	int ret;
	u8 val;

	clamp_t(u32, freq, 0U, ARRAY_SIZE(ct2_settings_vs_freq_ghz) - 1);
	val = ct2_settings_vs_freq_ghz[freq];
	if (val == dev->ct2_cached)
		return 0;

	ret = regmap_write(dev->regmap,
			ADL5960_REG_CT2,
			FIELD_PREP(ADL5960_CT2_MSK, val));
	if (!ret)
		dev->ct2_cached = val;

	return ret;
}

static int adl5960_update_ct4_filter(struct adl5960_dev *dev, u32 freq)
{
	int ret;
	u8 val;

	clamp_t(u32, freq, 0U, ARRAY_SIZE(ct4_settings_vs_freq_ghz) - 1);
	val = ct4_settings_vs_freq_ghz[freq];
	if (val == dev->ct4_cached)
		return 0;

	/* x4 output has higher attenuation below 14GHz */
	if (val == 255) {
		dev_err(&dev->spi->dev,
			"Skipping CT4 for %u GHz, use CT2 instead\n", freq);

		return -EINVAL;
	}

	ret = regmap_write(dev->regmap,
			ADL5960_REG_CT4,
			FIELD_PREP(ADL5960_CT4_MSK, val));
	if (!ret)
		dev->ct4_cached = val;

	return ret;
}

static int adl5960_update_ct_filters(struct adl5960_dev *dev, u64 freq)
{
	int ret = 0;
	u32 val;

	if (dev->lo_freq_mult == 1)
		return 0;

	mutex_lock(&dev->lock);

	val = DIV_ROUND_UP_ULL(freq * dev->lo_freq_mult, 1000000000U);

	switch (dev->lo_freq_mult) {
	case 2:
		ret = adl5960_update_ct2_filter(dev, val);
		break;
	case 4:
		ret = adl5960_update_ct2_filter(dev, val / 2);
		ret |= adl5960_update_ct4_filter(dev, val);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&dev->lock);

	return ret;
}

static int adl5960_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);
	unsigned int data, reg, mask, c1, c2;
	int ret = 0;

	mutex_lock(&dev->lock);

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->channel) {
		case 0:
			reg = ADL5960_REG_IGAIN;
			mask = ADL5960_IGAIN_MSK;
			break;
		case 1:
			reg = ADL5960_REG_RGAIN;
			mask = ADL5960_RGAIN_MSK;
			break;
		default:
			ret = -EINVAL;
			goto out;
		}

		ret = regmap_read(dev->regmap, reg, &data);
		if (ret < 0)
			goto out;

		*val = FIELD_GET(mask, data);
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = regmap_read(dev->regmap,
			ADL5960_REG_CIF2_CIF1,
			&data);
		c1 = FIELD_GET(ADL5960_CIF1_MSK, data);
		c2 = FIELD_GET(ADL5960_CIF2_MSK, data);

		c1 = adl5920_cif_cutoff_lut[15 - c1][0];
		c2 = adl5920_cif_cutoff_lut[15 - c2][1];

		if (c1 < c2)
			*val = c1;
		else
			*val = c2;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		ret = regmap_read(dev->regmap, ADL5960_REG_TDEG, &data);
		if (ret < 0)
			goto out;

		c1 = FIELD_GET(ADL5960_TDEG_MSK, data);
		*val = tdeg_to_celcius[c1] * 1000;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&dev->lock);
	return ret;
}

static int adl5960_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		val = clamp_val(val, 0, 127);

		switch (chan->channel) {
		case 0:
			return regmap_write(dev->regmap,
				ADL5960_REG_IGAIN,
				FIELD_PREP(ADL5960_IGAIN_MSK, val));
		case 1:
			return regmap_write(dev->regmap,
				ADL5960_REG_RGAIN,
				FIELD_PREP(ADL5960_RGAIN_MSK, val));
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return regmap_write(dev->regmap,
			ADL5960_REG_CIF2_CIF1,
			adl5960_update_cif_filters(val));
	default:
		return -EINVAL;
	}
}

static int ad5960_phy_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	static const int gain[] = {0, 6, 48};

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*vals = gain;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	}

	return -EINVAL;
}

static int adl5960_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info adl5960_info = {
	.read_raw = adl5960_read_raw,
	.write_raw = adl5960_write_raw,
	.read_avail = ad5960_phy_read_avail,
	.debugfs_reg_access = &adl5960_reg_access,
};

static int adl5960_freq_change(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct adl5960_dev *dev = container_of(nb, struct adl5960_dev, nb);
	struct clk_notifier_data *cnd = data;

	if (action == POST_RATE_CHANGE) {
		/* cache the new rate */
		dev->lo_freq = clk_get_rate_scaled(cnd->clk, &dev->clkscale);
		return notifier_from_errno(
			adl5960_update_ct_filters(dev, dev->lo_freq));
	}

	return NOTIFY_OK;
}

static void adl5960_clk_notifier_unreg(void *data)
{
	struct adl5960_dev *dev = data;

	clk_notifier_unregister(dev->clkin_lo, &dev->nb);
}

static ssize_t adl5960_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    char *buf)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);
	unsigned long long val = 0;
	int ret = 0;

	switch ((u32)private) {
	case ADL5960_EXTI_LO_FREQ:
		val = (dev->lo_freq * dev->lo_freq_mult);
		do_div(val, dev->lo_freq_div);
		break;
	case ADL5960_EXTI_IF_FREQ:
		val = clk_get_rate(dev->clkin_offs) * dev->offs_freq_mult;
		do_div(val, dev->offs_freq_div);
		break;
	case ADL5960_EXTI_OFS_FREQ:
		val = clk_get_rate(dev->clkin_offs);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t adl5960_write(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);
	u64 freq;
	int ret;

	ret = kstrtoull(buf, 10, &freq);
	if (ret)
		return ret;

	switch ((u32)private) {
	case ADL5960_EXTI_LO_FREQ:
		return adl5960_update_ct_filters(dev, freq);
	case ADL5960_EXTI_IF_FREQ:
		mutex_lock(&dev->lock);
		freq *= dev->offs_freq_div;
		do_div(freq, dev->offs_freq_mult);
		ret = clk_set_rate(dev->clkin_offs, freq);
		mutex_unlock(&dev->lock);
		break;
	case ADL5960_EXTI_OFS_FREQ:
		ret = clk_set_rate(dev->clkin_offs, freq);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : len;
}

static int adl5960_get_mode(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);
	unsigned int readval;
	int ret;

	mutex_lock(&dev->lock);

	ret = regmap_read(dev->regmap,
		ADL5960_REG_LO_CONFIG, &readval);
	if (ret < 0)
		goto out;

	switch (chan->channel) {
	case 0:
		if (FIELD_GET(ADL5960_BYPASS_MSK, readval))
			ret = 0;
		else
			ret = FIELD_GET(ADL5960_LOMODE_MSK, readval) + 1;
		break;
	case 1:
		if (FIELD_GET(ADL5960_BYPASS_MSK, readval))
			ret = 4;
		else
			ret = FIELD_GET(ADL5960_IFMODE_MSK, readval);
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&dev->lock);

	return ret;
}

static int adl5960_set_mode(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int mode)
{
	struct adl5960_dev *dev = iio_priv(indio_dev);
	u8 bypass, val;
	int ret;

	mutex_lock(&dev->lock);

	switch (chan->channel) {
	case 0:
		switch (mode) {
		case 0:
			bypass = 1;
			val = 0;
			dev->lo_freq_mult = 1;
			dev->lo_freq_div = 1;
			break;
		case 1:
			bypass = 0;
			val = mode - 1;
			dev->lo_freq_mult = 1;
			dev->lo_freq_div = 2;
			break;
		default:
			bypass = 0;
			val = mode - 1;
			dev->lo_freq_mult = 1 << (mode - 2);
			dev->lo_freq_div = 1;
			break;
		}

		ret = regmap_update_bits(dev->regmap,
				ADL5960_REG_LO_CONFIG,
				ADL5960_BYPASS_MSK | ADL5960_LOMODE_MSK,
				FIELD_PREP(ADL5960_BYPASS_MSK, bypass) |
				FIELD_PREP(ADL5960_LOMODE_MSK, val));
		break;
	case 1:
		switch (mode) {
		case 3:
			bypass = 0;
			dev->offs_freq_mult = 0;
			dev->offs_freq_div = 1;
			break;
		case 4:
			bypass = 1;
			dev->offs_freq_mult = 0;
			dev->offs_freq_div = 1;
			break;
		default:
			bypass = 0;
			dev->offs_freq_mult = 1;
			dev->offs_freq_div = 1 << mode;
			break;
		}

		ret = regmap_update_bits(dev->regmap,
				ADL5960_REG_LO_CONFIG,
				ADL5960_BYPASS_MSK | ADL5960_IFMODE_MSK,
				FIELD_PREP(ADL5960_BYPASS_MSK, bypass) |
				FIELD_PREP(ADL5960_IFMODE_MSK, mode));
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&dev->lock);

	return ret;
}

static const struct iio_enum adl5960_lo_mode_enum = {
	.items = adl5960_lo_modes,
	.num_items = ARRAY_SIZE(adl5960_lo_modes),
	.get = adl5960_get_mode,
	.set = adl5960_set_mode,
};

static const struct iio_enum adl5960_if_mode_enum = {
	.items = adl5960_if_modes,
	.num_items = ARRAY_SIZE(adl5960_if_modes),
	.get = adl5960_get_mode,
	.set = adl5960_set_mode,
};

#define _ADL5960_EXT_INFO(_name, _shared, _ident) { \
		.name = _name, \
		.read = adl5960_read, \
		.write = adl5960_write, \
		.private = _ident, \
		.shared = _shared, \
}

static const struct iio_chan_spec_ext_info adl5960_ext_lo_info[] = {
	/*
	 * Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADL5960_EXT_INFO("frequency", IIO_SEPARATE,
		ADL5960_EXTI_LO_FREQ),
	IIO_ENUM("mode", IIO_SEPARATE, &adl5960_lo_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("mode", IIO_SEPARATE, &adl5960_lo_mode_enum),
	{ },
};

static const struct iio_chan_spec_ext_info adl5960_ext_if_info[] = {
	/*
	 * Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADL5960_EXT_INFO("if_frequency", IIO_SEPARATE,
		ADL5960_EXTI_IF_FREQ),
	_ADL5960_EXT_INFO("offset_frequency", IIO_SEPARATE,
		ADL5960_EXTI_OFS_FREQ),
	IIO_ENUM("offset_mode", IIO_SEPARATE, &adl5960_if_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("offset_mode", IIO_SEPARATE,
		&adl5960_if_mode_enum),
	{ },
};

#define ADL5960_CHAN_OUT(_channel, _extend_name) { \
	.type = IIO_VOLTAGE, \
	.output = 1, \
	.indexed = 1, \
	.channel = _channel, \
	.extend_name = (_extend_name), \
	.info_mask_shared_by_type_available = \
		BIT(IIO_CHAN_INFO_HARDWAREGAIN), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN), \
	.info_mask_shared_by_type = \
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
}

#define ADL5960_CHAN_IN(_channel, _extend_name, _adl5960_ext_info) { \
	.type = IIO_ALTVOLTAGE, \
	.indexed = 1, \
	.channel = _channel, \
	.ext_info = _adl5960_ext_info, \
	.extend_name = (_extend_name), \
}

static const struct iio_chan_spec adl5960_channels[] = {
	ADL5960_CHAN_OUT(0, "forward"),
	ADL5960_CHAN_OUT(1, "reverse"),
	ADL5960_CHAN_IN(0, "lo", adl5960_ext_lo_info),
	ADL5960_CHAN_IN(1, NULL, adl5960_ext_if_info),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int adl5960_init(struct adl5960_dev *dev)
{
	struct spi_device *spi = dev->spi;
	unsigned int reg_val;
	int ret;

	dev->lo_freq_mult = 1;
	dev->lo_freq_div = 1;
	dev->offs_freq_mult = 0;
	dev->offs_freq_div = 1;

	dev->regmap = devm_regmap_init_spi(spi, &adl5960_regmap_config);
	if (IS_ERR(dev->regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(dev->regmap));
		return PTR_ERR(dev->regmap);
	}

	ret = regmap_multi_reg_write(dev->regmap, adl5960_reg_defaults,
		ARRAY_SIZE(adl5960_reg_defaults));
	if (ret < 0)
		return ret;

	/* Fixme check PRODUCT_ID in future */
	ret = regmap_read(dev->regmap, ADL5960_REG_CHIPTYPE, &reg_val);
	if (ret < 0)
		return ret;

	if (reg_val != 1) {
		dev_err(&spi->dev, "Failed to verify device presence (%x)\n",
			reg_val);
		return -ENODEV;
	}

	return 0;
}

static void adl5960_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void adl5960_reg_disable(void *data)
{
	regulator_disable(data);
}

static int adl5960_dt_parse(struct adl5960_dev *dev)
{
	struct spi_device *spi = dev->spi;

	dev->reg = devm_regulator_get(&spi->dev, "avcc");
	if (IS_ERR(dev->reg))
		return PTR_ERR(dev->reg);

	dev->clkin_lo = devm_clk_get(&spi->dev, "lo_in");
	if (IS_ERR(dev->clkin_lo))
		return PTR_ERR(dev->clkin_lo);

	dev->clkin_offs = devm_clk_get(&spi->dev, "offs_in");
	if (IS_ERR(dev->clkin_offs))
		return PTR_ERR(dev->clkin_offs);

	return of_clk_get_scale(spi->dev.of_node, "lo_in", &dev->clkscale);
}

static int adl5960_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adl5960_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	dev = iio_priv(indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &adl5960_info;
	indio_dev->name = "adl5960";
	indio_dev->channels = adl5960_channels;
	indio_dev->num_channels = ARRAY_SIZE(adl5960_channels);

	dev->spi = spi;

	ret = adl5960_dt_parse(dev);
	if (ret < 0)
		return ret;

	ret = regulator_enable(dev->reg);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable regulator\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, adl5960_reg_disable,
					dev->reg);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(dev->clkin_lo);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		adl5960_clk_disable, dev->clkin_lo);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(dev->clkin_offs);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		adl5960_clk_disable, dev->clkin_offs);
	if (ret < 0)
		return ret;

	dev->lo_freq = clk_get_rate_scaled(dev->clkin_lo, &dev->clkscale);

	dev->nb.notifier_call = adl5960_freq_change;
	ret = clk_notifier_register(dev->clkin_lo, &dev->nb);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		adl5960_clk_notifier_unreg, dev);
	if (ret < 0)
		return ret;

	mutex_init(&dev->lock);

	ret = adl5960_init(dev);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adl5960_id[] = {
	{ "adl5960", ADL5960 },
	{}
};
MODULE_DEVICE_TABLE(spi, adl5960_id);

static const struct of_device_id adl5960_of_match[] = {
	{ .compatible = "adi,adl5960" },
	{},
};
MODULE_DEVICE_TABLE(of, adl5960_of_match);

static struct spi_driver adl5960_driver = {
	.driver = {
			.name = "adl5960",
			.of_match_table = adl5960_of_match,
		},
	.probe = adl5960_probe,
	.id_table = adl5960_id,
};
module_spi_driver(adl5960_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADL5960");
MODULE_LICENSE("Dual BSD/GPL");
