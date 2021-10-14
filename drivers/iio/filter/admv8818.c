// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADMV8818 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

/* ADMV8818 Register Map */
#define ADMV8818_REG_SPI_CONFIG_A		0x0
#define ADMV8818_REG_SPI_CONFIG_B		0x1
#define ADMV8818_REG_CHIPTYPE			0x3
#define ADMV8818_REG_PRODUCT_ID_L		0x4
#define ADMV8818_REG_PRODUCT_ID_H		0x5
#define ADMV8818_REG_FAST_LATCH_POINTER		0x10
#define ADMV8818_REG_FAST_LATCH_STOP		0x11
#define ADMV8818_REG_FAST_LATCH_START		0x12
#define ADMV8818_REG_FAST_LATCH_DIRECTION	0x13
#define ADMV8818_REG_FAST_LATCH_STATE		0x14
#define ADMV8818_REG_WR0_SW			0x20
#define ADMV8818_REG_WR0_FILTER			0x21
#define ADMV8818_REG_WR1_SW			0x22
#define ADMV8818_REG_WR1_FILTER			0x23
#define ADMV8818_REG_WR2_SW			0x24
#define ADMV8818_REG_WR2_FILTER			0x25
#define ADMV8818_REG_WR3_SW			0x26
#define ADMV8818_REG_WR3_FILTER			0x27
#define ADMV8818_REG_WR4_SW			0x28
#define ADMV8818_REG_WR4_FILTER			0x29
#define ADMV8818_REG_LUT0_SW			0x100
#define ADMV8818_REG_LUT0_FILTER		0x101
#define ADMV8818_REG_LUT127_SW			0x1FE
#define ADMV8818_REG_LUT127_FILTER		0x1FF

/* ADMV8818_REG_SPI_CONFIG_A Map */
#define ADMV8818_SOFTRESET_N_MSK		BIT(7)
#define ADMV8818_LSB_FIRST_N_MSK		BIT(6)
#define ADMV8818_ENDIAN_N_MSK			BIT(5)
#define ADMV8818_SDOACTIVE_N_MSK		BIT(4)
#define ADMV8818_SDOACTIVE_MSK			BIT(3)
#define ADMV8818_ENDIAN_MSK			BIT(2)
#define ADMV8818_LSBFIRST_MSK			BIT(1)
#define ADMV8818_SOFTRESET_MSK			BIT(0)

/* ADMV8818_REG_SPI_CONFIG_B Map */
#define ADMV8818_SINGLE_INSTRUCTION_MSK		BIT(7)
#define ADMV8818_CSB_STALL_MSK			BIT(6)
#define ADMV8818_MASTER_SLAVE_RB_MSK		BIT(5)
#define ADMV8818_MASTER_SLAVE_TRANSFER_MSK	BIT(0)

/* ADMV8818_REG_WR0_SW Map */
#define ADMV8818_SW_IN_SET_WR0_MSK		BIT(7)
#define ADMV8818_SW_OUT_SET_WR0_MSK		BIT(6)
#define ADMV8818_SW_IN_WR0_MSK			GENMASK(5, 3)
#define ADMV8818_SW_OUT_WR0_MSK			GENMASK(2, 0)

/* ADMV8818_REG_WR0_FILTER Map */
#define ADMV8818_HPF_WR0_MSK			GENMASK(7, 4)
#define ADMV8818_LPF_WR0_MSK			GENMASK(3, 0)

struct admv8818_dev {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clkin;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	struct mutex		lock;
	unsigned int		freq_scale;
	u64			clkin_freq;
	u32			tolerance;
};

static const unsigned long long freq_range_hpf[4][2] = {
	{1750000000ULL, 3550000000ULL},
	{3400000000ULL, 7250000000ULL},
	{6600000000, 12000000000},
	{12500000000, 19900000000}
};

static const unsigned long long freq_range_lpf[4][2] = {
	{2050000000ULL, 3850000000ULL},
	{3350000000ULL, 7250000000ULL},
	{7000000000, 13000000000},
	{12550000000, 18500000000}
};

static const struct regmap_config admv8818_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	.max_register = 0x1FF,
};

static int admv8818_hpf_select(struct admv8818_dev *dev, u64 freq)
{
	unsigned int hpf_step = 0, hpf_band = 0, i, j;
	u64 freq_step;
	int ret;

	if (freq < freq_range_hpf[0][0])
		goto hpf_write;

	if (freq > freq_range_hpf[3][1]) {
		hpf_step = 15;
		hpf_band = 4;

		goto hpf_write;
	}

	for (i = 0; i < 4; i++) {
		freq_step = div_u64((freq_range_hpf[i][1] -
			freq_range_hpf[i][0]), 15);

		if ((freq > freq_range_hpf[i][0]) &&
			(freq < freq_range_hpf[i][1] + freq_step)) {
			hpf_band = i + 1;

			for (j = 1; j <= 16; j++) {
				if (freq < (freq_range_hpf[i][0] + (freq_step * j))) {
					hpf_step = j - 1;
					break;
				}
			}
			break;
		}
	}

	/* Close HPF frequency gap between 12 and 12.5 GHz */
	if (freq >= 12000000000 && freq <= 12500000000) {
		hpf_band = 3;
		hpf_step = 15;
	}

hpf_write:
	mutex_lock(&dev->lock);

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_WR0_SW,
				ADMV8818_SW_IN_SET_WR0_MSK |
				ADMV8818_SW_IN_WR0_MSK,
				FIELD_PREP(ADMV8818_SW_IN_SET_WR0_MSK, 1) |
				FIELD_PREP(ADMV8818_SW_IN_WR0_MSK, hpf_band));
	if (ret)
		goto exit;

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_WR0_FILTER,
				ADMV8818_HPF_WR0_MSK,
				FIELD_PREP(ADMV8818_HPF_WR0_MSK, hpf_step));

exit:
	mutex_unlock(&dev->lock);

	return ret;
}

static int admv8818_lpf_select(struct admv8818_dev *dev, u64 freq)
{
	unsigned int lpf_step = 0, lpf_band = 0, i, j;
	u64 freq_step;
	int ret;

	if (freq > freq_range_lpf[3][1])
		goto lpf_write;

	if (freq < freq_range_lpf[0][0]) {
		lpf_band = 1;

		goto lpf_write;
	}

	for (i = 0; i < 4; i++) {
		if ((freq > freq_range_lpf[i][0]) && freq < freq_range_lpf[i][1]) {
			lpf_band = i + 1;
			freq_step = div_u64((freq_range_lpf[i][1] - freq_range_lpf[i][0]), 15);

			for (j = 0; j <= 15; j++) {
				if (freq < (freq_range_lpf[i][0] + (freq_step * j))) {
					lpf_step = j;
					break;
				}
			}
			break;
		}
	}

lpf_write:
	mutex_lock(&dev->lock);

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_WR0_SW,
				ADMV8818_SW_OUT_SET_WR0_MSK |
				ADMV8818_SW_OUT_WR0_MSK,
				FIELD_PREP(ADMV8818_SW_OUT_SET_WR0_MSK, 1) |
				FIELD_PREP(ADMV8818_SW_OUT_WR0_MSK, lpf_band));
	if (ret)
		goto exit;

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_WR0_FILTER,
				ADMV8818_LPF_WR0_MSK,
				FIELD_PREP(ADMV8818_LPF_WR0_MSK, lpf_step));
exit:
	mutex_unlock(&dev->lock);

	return ret;
}

static int admv8818_rfin_band_select(struct admv8818_dev *dev)
{
	int ret;

	ret = admv8818_hpf_select(dev, DIV_ROUND_DOWN_ULL(
		dev->clkin_freq * (100 - dev->tolerance), 100));
	if (ret)
		return ret;

	return admv8818_lpf_select(dev, DIV_ROUND_UP_ULL(
		dev->clkin_freq * (100 + dev->tolerance), 100));
}

static int admv8818_read_hpf_freq(struct admv8818_dev *dev, unsigned int *hpf_freq)
{
	unsigned int data, hpf_band, hpf_state;
	int ret;

	mutex_lock(&dev->lock);

	ret = regmap_read(dev->regmap, ADMV8818_REG_WR0_SW, &data);
	if (ret)
		goto exit;

	hpf_band = FIELD_GET(ADMV8818_SW_IN_WR0_MSK, data);
	if (!hpf_band) {
		*hpf_freq = 0;
		goto exit;
	}

	ret = regmap_read(dev->regmap, ADMV8818_REG_WR0_FILTER, &data);
	if (ret)
		goto exit;

	hpf_state = FIELD_GET(ADMV8818_HPF_WR0_MSK, data);

	*hpf_freq = div_u64(freq_range_hpf[hpf_band-1][1] - freq_range_hpf[hpf_band-1][0], dev->freq_scale * 15);
	*hpf_freq = div_u64(freq_range_hpf[hpf_band-1][0], dev->freq_scale) + (*hpf_freq * hpf_state);

exit:
	mutex_unlock(&dev->lock);

	return ret;
}


static int admv8818_read_lpf_freq(struct admv8818_dev *dev, unsigned int *lpf_freq)
{
	unsigned int data, lpf_band, lpf_state;
	int ret;

	mutex_lock(&dev->lock);

	ret = regmap_read(dev->regmap, ADMV8818_REG_WR0_SW, &data);
	if (ret)
		goto exit;

	lpf_band = FIELD_GET(ADMV8818_SW_OUT_WR0_MSK, data);
	if (!lpf_band) {
		*lpf_freq = 0;
		goto exit;
	}

	ret = regmap_read(dev->regmap, ADMV8818_REG_WR0_FILTER, &data);
	if (ret)
		goto exit;

	lpf_state = FIELD_GET(ADMV8818_LPF_WR0_MSK, data);

	*lpf_freq = div_u64(freq_range_lpf[lpf_band-1][1] - freq_range_lpf[lpf_band-1][0], dev->freq_scale * 15);
	*lpf_freq = div_u64(freq_range_lpf[lpf_band-1][0], dev->freq_scale) + (*lpf_freq * lpf_state);

exit:
	mutex_unlock(&dev->lock);

	return ret;
}

static int admv8818_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct admv8818_dev *dev = iio_priv(indio_dev);
	u64 freq = (u64)val * dev->freq_scale;

	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return admv8818_lpf_select(dev, freq);
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		return admv8818_hpf_select(dev, freq);
	case IIO_CHAN_INFO_SCALE:
		dev->freq_scale = val;
		return 0;
	default:
		return -EINVAL;
	}
}

static int admv8818_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct admv8818_dev *dev = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = admv8818_read_lpf_freq(dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		ret = admv8818_read_hpf_freq(dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = dev->freq_scale;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int admv8818_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int write_val,
				unsigned int *read_val)
{
	struct admv8818_dev *dev = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(dev->regmap, reg, read_val);
	else
		return regmap_write(dev->regmap, reg, write_val);
}

static const struct iio_info admv8818_info = {
	.write_raw = admv8818_write_raw,
	.read_raw = admv8818_read_raw,
	.debugfs_reg_access = &admv8818_reg_access,
};

#define ADMV8818_CHAN(_channel) {				\
	.type = IIO_ALTVOLTAGE,					\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.info_mask_separate =					\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY) | \
		BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY) | \
		BIT(IIO_CHAN_INFO_SCALE) \
}

static const struct iio_chan_spec admv8818_channels[] = {
	ADMV8818_CHAN(0),
};

static int admv8818_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv8818_dev *dev = container_of(nb, struct admv8818_dev, nb);
	struct clk_notifier_data *cnd = data;
	unsigned long long rate;

	if (action == PRE_RATE_CHANGE) {
		/* cache the new rate */
		rate = from_ccf_scaled(cnd->new_rate, &dev->clkscale);

		if (rate == dev->clkin_freq)
			return NOTIFY_OK;

		dev->clkin_freq = rate;

		return notifier_from_errno(admv8818_rfin_band_select(dev));
	}

	return NOTIFY_OK;
}

static void admv8818_clk_notifier_unreg(void *data)
{
	struct admv8818_dev *dev = data;

	clk_notifier_unregister(dev->clkin, &dev->nb);
}

static void admv8818_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int admv8818_init(struct admv8818_dev *dev)
{
	int ret;
	struct spi_device *spi = dev->spi;
	unsigned int chip_id;

	dev->freq_scale = 1000000;

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_SPI_CONFIG_A,
					ADMV8818_SOFTRESET_N_MSK |
					ADMV8818_SOFTRESET_MSK,
					FIELD_PREP(ADMV8818_SOFTRESET_N_MSK, 1) |
					FIELD_PREP(ADMV8818_SOFTRESET_MSK, 1));
	if (ret) {
		dev_err(&spi->dev, "ADMV8818 Soft Reset failed.\n");
		return ret;
	}

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_SPI_CONFIG_A,
					ADMV8818_SDOACTIVE_N_MSK |
					ADMV8818_SDOACTIVE_MSK,
					FIELD_PREP(ADMV8818_SDOACTIVE_N_MSK, 1) |
					FIELD_PREP(ADMV8818_SDOACTIVE_MSK, 1));
	if (ret) {
		dev_err(&spi->dev, "ADMV8818 SDO Enable failed.\n");
		return ret;
	}

	ret = regmap_read(dev->regmap, ADMV8818_REG_CHIPTYPE, &chip_id);
	if (ret) {
		dev_err(&spi->dev, "ADMV8818 Chip ID read failed.\n");
		return ret;
	}

	if (chip_id != 0x1) {
		dev_err(&spi->dev, "ADMV8818 Invalid Chip ID.\n");
		return -EINVAL;
	}

	ret = regmap_update_bits(dev->regmap, ADMV8818_REG_SPI_CONFIG_B,
					ADMV8818_SINGLE_INSTRUCTION_MSK,
					FIELD_PREP(ADMV8818_SINGLE_INSTRUCTION_MSK, 1));
	if (ret) {
		dev_err(&spi->dev, "ADMV8818 Single Instruction failed.\n");
		return ret;
	}

	if (dev->clkin)
		return admv8818_rfin_band_select(dev);
	else
		return 0;
}

static int admv8818_clk_setup(struct admv8818_dev *dev)
{
	struct spi_device *spi = dev->spi;
	int ret;

	dev->clkin = devm_clk_get_optional(&spi->dev, "rf_in");
	if (IS_ERR(dev->clkin))
		return PTR_ERR(dev->clkin);
	else if (!dev->clkin)
		return 0;

	ret = of_clk_get_scale(spi->dev.of_node, NULL, &dev->clkscale);
	if (ret)
		return ret;

	of_property_read_u32(spi->dev.of_node,
		"adi,tolerance-percent", &dev->tolerance);

	dev->tolerance = clamp(dev->tolerance, 0U, 50U);

	ret = clk_prepare_enable(dev->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv8818_clk_disable, dev->clkin);
	if (ret)
		return ret;

	dev->clkin_freq = clk_get_rate_scaled(dev->clkin, &dev->clkscale);

	dev->nb.notifier_call = admv8818_freq_change;
	ret = clk_notifier_register(dev->clkin, &dev->nb);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&spi->dev, admv8818_clk_notifier_unreg, dev);
}

static int admv8818_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct admv8818_dev *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &admv8818_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &admv8818_info;
	indio_dev->name = "admv8818";
	indio_dev->channels = admv8818_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv8818_channels);

	dev->spi = spi;

	ret = admv8818_clk_setup(dev);
	if (ret)
		return ret;

	mutex_init(&dev->lock);

	ret = admv8818_init(dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admv8818_id[] = {
	{ "admv8818", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv8818_id);

static const struct of_device_id admv8818_of_match[] = {
	{ .compatible = "adi,admv8818" },
	{}
};
MODULE_DEVICE_TABLE(of, admv8818_of_match);

static struct spi_driver admv8818_driver = {
	.driver = {
		.name = "admv8818",
		.of_match_table = admv8818_of_match,
	},
	.probe = admv8818_probe,
	.id_table = admv8818_id,
};
module_spi_driver(admv8818_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADMV8818");
MODULE_LICENSE("GPL v2");
