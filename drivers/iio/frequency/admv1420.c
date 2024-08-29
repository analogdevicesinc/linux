// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1420 Microwave Downconverter Driver
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/gpio/consumer.h>
#include <linux/clk/clkscale.h>
#include <linux/notifier.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/clk.h>

#define ADMV1420_REG_PRODUCT_ID_LSB	0x004

#define ADMV1420_REG0000	0x000
#define ADMV1420_REG013E	0x13E
#define ADMV1420_REG0202	0x202
#define ADMV1420_REG0208	0x208
#define ADMV1420_REG028A	0x28A
#define ADMV1420_REG028B	0x28B
#define ADMV1420_REG02A0	0x2A0
#define ADMV1420_REG02A1	0x2A1
#define ADMV1420_REG0603	0x603
#define ADMV1420_REG0800	0x800
#define ADMV1420_REG0801	0x801
#define ADMV1420_REG0802	0x802
#define ADMV1420_REG0804	0x804
#define ADMV1420_REG0805	0x805
#define ADMV1420_REG0806	0x806
#define ADMV1420_REG080A	0x80A
#define ADMV1420_REG080C	0x80C

struct admv1420_priv {
	struct spi_device	*spi;
	struct gpio_desc	*cen_gpio;
	struct gpio_desc	*reset_gpio;
	struct regmap		*regmap;
	struct clk		*lo_input;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	/* Protect against concurrent accesses to the device and to data */
	struct mutex		lock;
};

static int admv1420_debug_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int write_val, unsigned int *read_val)
{
	struct admv1420_priv *priv = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(priv->regmap, reg, read_val);
	else
		return regmap_write(priv->regmap, reg, write_val);
}

static const struct iio_info admv1420_info = {
	.debugfs_reg_access = &admv1420_debug_reg_access,
};

static int admv1420_set_filters(struct admv1420_priv *priv, u64 rate_khz)
{
	u8 lo_x3_filter;
	int ret;

	// LO X3 FILTER REG0800
	if (rate_khz >= (8000 * HZ_PER_KHZ) && rate_khz < (10000 * HZ_PER_KHZ)) {
		lo_x3_filter = 0x0D;
	} else if (rate_khz >= (10000 * HZ_PER_KHZ) && rate_khz < (12000 * HZ_PER_KHZ)) {;
		lo_x3_filter = 0x0C;
	} else if (rate_khz >= (12000 * HZ_PER_KHZ) && rate_khz < (14000 * HZ_PER_KHZ)) {
		lo_x3_filter = 0x0B;
	} else if (rate_khz >= (14000 * HZ_PER_KHZ) && rate_khz < (18000 * HZ_PER_KHZ)) {
		lo_x3_filter = 0x09;
	} else if (rate_khz >= (18000 * HZ_PER_KHZ) && rate_khz < (24000 * HZ_PER_KHZ)) {
		lo_x3_filter = 0x08;
	} else {
		// reset value, 24-28 GHz
		lo_x3_filter = 0x00;
	}

	ret = regmap_update_bits(priv->regmap, ADMV1420_REG0800, 0x0F, lo_x3_filter);
	if (ret)
		return ret;

	return 0;
}

static int admv1420_update(struct admv1420_priv *priv)
{
	int ret;
	u64 rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1420_set_filters(priv, rate);
	if (ret)
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);

	return 0;
}

static int admv1420_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv1420_priv *priv = container_of(nb, struct admv1420_priv, nb);
	int ret;

	if (action == POST_RATE_CHANGE) {
		mutex_lock(&priv->lock);
		ret = notifier_from_errno(admv1420_update(priv));
		mutex_unlock(&priv->lock);
		return ret;
	}

	return NOTIFY_OK;
}

static int admv1420_setup(struct iio_dev *indio_dev)
{
	struct admv1420_priv *priv = iio_priv(indio_dev);
	unsigned int product_id;
	int ret;

	// Set SPI 4 wire
	ret = regmap_write(priv->regmap, ADMV1420_REG0000, 0x18);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap,
		ADMV1420_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	pr_err("%s: PRODUCT_ID: 0x%X\n", priv->spi->dev.of_node->name, product_id);

	// Disable filter table
	ret = regmap_write(priv->regmap, ADMV1420_REG0202, 0x00);
	if (ret)
		return ret;

	// Disable filter load
	ret = regmap_write(priv->regmap, ADMV1420_REG0208, 0x00);
	if (ret)
		return ret;

	// Enable gain table bypass
	ret = regmap_write(priv->regmap, ADMV1420_REG028A, 0x01);
	if (ret)
		return ret;

	// DSA1 and DSA2 bypass values
	ret = regmap_write(priv->regmap, ADMV1420_REG028B, 0x00);
	if (ret)
		return ret;

	// LPF state determined by SPI
	ret = regmap_write(priv->regmap, ADMV1420_REG02A0, 0x10);
	if (ret)
		return ret;

	// HPF determined by SPI, cutoff frequenct 0x3F (reset value)
	ret = regmap_write(priv->regmap, ADMV1420_REG02A1, 0xBF);
	if (ret)
		return ret;

	// Set GPIO for gain table
	ret = regmap_write(priv->regmap, ADMV1420_REG0603, 0x08);
	if (ret)
		return ret;

return 0;
// Setting direct RF band (0 - low, 1 - high)
	ret = regmap_update_bits(priv->regmap, ADMV1420_REG0805, 0x80, 0);
	if (ret)
		return ret;

// Setting LO band, mixer band, LO sideband and direct LO Phase
	ret = regmap_write(priv->regmap, ADMV1420_REG0806, 0xF0);
	if (ret)
		return ret;

// Set direct IF gain
	ret = regmap_write(priv->regmap, ADMV1420_REG080A, 0x00);
	if (ret)
		return ret;

	return 0;
}

static void admv1420_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct regmap_config admv1420_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv1420_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv1420_priv *priv;
	u64 rate = 0;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	priv->spi = spi;

	priv->cen_gpio = devm_gpiod_get_optional(&spi->dev,
						 "chip-enable",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(priv->cen_gpio))
		return PTR_ERR(priv->cen_gpio);

	priv->reset_gpio = devm_gpiod_get_optional(&spi->dev,
						  "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	priv->regmap = devm_regmap_init_spi(spi, &admv1420_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->lo_input = devm_clk_get_optional(&spi->dev, "lo-input");
	if (IS_ERR(priv->lo_input))
		return dev_err_probe(&spi->dev, PTR_ERR(priv->lo_input),
				     "failed to get the LO input clock\n");

	ret = of_clk_get_scale(spi->dev.of_node, NULL, &priv->clkscale);
	if (ret)
		return ret;

	ret = clk_prepare_enable(priv->lo_input);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, admv1420_clk_disable, priv->lo_input);
	if (ret)
		return ret;

	priv->nb.notifier_call = admv1420_freq_change;
	ret = devm_clk_notifier_register(&spi->dev, priv->lo_input, &priv->nb);
	if (ret)
		return ret;

	mutex_init(&priv->lock);

	ret = admv1420_setup(indio_dev);
	if (ret)
		return ret;

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1420_set_filters(priv, rate);
	if (ret) {
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);
		return ret;
	}

	//indio_dev->name = "admv1420";
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv1420_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver admv1420_driver = {
	.driver = {
		.name = "admv1420",
	},
	.probe = admv1420_probe,
};
module_spi_driver(admv1420_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1420 Microwave Downconverter Driver");
MODULE_LICENSE("GPL v2");
