// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1455 Microwave Downconverter Driver
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

#define ADMV1455_REG_PRODUCT_ID_LSB	0x004

#define ADMV1455_REG0000	0x000
#define ADMV1455_REG0202	0x202
#define ADMV1455_REG0208	0x208
#define ADMV1455_REG028A	0x28A
#define ADMV1455_REG028B	0x28B
#define ADMV1455_REG02A0	0x2A0
#define ADMV1455_REG02A1	0x2A1
#define ADMV1455_REG0603	0x603
#define ADMV1455_REG0800	0x800
#define ADMV1455_REG0801	0x801
#define ADMV1455_REG0802	0x802
#define ADMV1455_REG0803	0x803
#define ADMV1455_REG0804	0x804
#define ADMV1455_REG0805	0x805
#define ADMV1455_REG0806	0x806
#define ADMV1455_REG080A	0x80A
#define ADMV1455_REG080C	0x80C

struct admv1455_priv {
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

static int admv1455_debug_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int write_val, unsigned int *read_val)
{
	struct admv1455_priv *priv = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(priv->regmap, reg, read_val);
	else
		return regmap_write(priv->regmap, reg, write_val);
}

static const struct iio_info admv1455_info = {
	.debugfs_reg_access = &admv1455_debug_reg_access,
};

static int admv1455_set_filters(struct admv1455_priv *priv, u64 rate_khz)
{
	u8 lo_trap_filter_7_0;
	u8 lo_trap_filter_9_8;
	u8 lo_doubler_band;
	u16 lo_trap_filter;
	u8 lo_x1_filter;
	u8 lo_x3_filter;
	u8 lo_x4_filter;
	int ret;

	// LO X1 FILTER REG0800
	// LO X3 FILTER REG0801
	// LO X4 FILTER REG0802
	// LO TRAP FILTER 7:0 REG0804
	// LO TRAP FILTER 9:8 REG0805
	// LO DOUBLED BAND REG080C
	if (rate_khz < (10000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x08;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1F;
	} else if (rate_khz >= (10000 * HZ_PER_KHZ) && rate_khz < (12000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1D;
	} else if (rate_khz >= (12000 * HZ_PER_KHZ) && rate_khz < (14000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x04;
	} else if (rate_khz >= (14000 * HZ_PER_KHZ) && rate_khz < (14500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x08;
	} else if (rate_khz >= (14500 * HZ_PER_KHZ) && rate_khz < (15000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz >= (15000 * HZ_PER_KHZ) && rate_khz < (17000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x05;
		lo_x1_filter = 0x05;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz >= (17000 * HZ_PER_KHZ) && rate_khz < (18000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz >= (18000 * HZ_PER_KHZ) && rate_khz < (18500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x80;
	} else if (rate_khz >= (18500 * HZ_PER_KHZ) && rate_khz < (19000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	}else if (rate_khz >= (19000 * HZ_PER_KHZ) && rate_khz < (21000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x03;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	} else if (rate_khz >= (21000 * HZ_PER_KHZ) && rate_khz < (24000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x02;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0x80;
	} else {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x00;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0xC0;
	}

	lo_trap_filter_7_0 = lo_trap_filter & 0xff;
	lo_trap_filter_9_8 = (lo_trap_filter >> 8) & 0x03;

	ret = regmap_update_bits(priv->regmap, ADMV1455_REG0800, 0x1F, lo_x1_filter);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1455_REG0801, 0x1F, lo_x3_filter);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1455_REG0802, 0X0F, lo_x4_filter);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1455_REG0803, lo_trap_filter_7_0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1455_REG0804, 0x03, lo_trap_filter_9_8);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1455_REG080C, lo_doubler_band);
	if (ret)
		return ret;

	return 0;
}

static int admv1455_update(struct admv1455_priv *priv)
{
	int ret;
	u64 rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1455_set_filters(priv, rate);
	if (ret)
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);

	return 0;
}

static int admv1455_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv1455_priv *priv = container_of(nb, struct admv1455_priv, nb);
	int ret;

	if (action == POST_RATE_CHANGE) {
		mutex_lock(&priv->lock);
		ret = notifier_from_errno(admv1455_update(priv));
		mutex_unlock(&priv->lock);
		return ret;
	}

	return NOTIFY_OK;
}

static int admv1455_setup(struct iio_dev *indio_dev)
{
	struct admv1455_priv *priv = iio_priv(indio_dev);
	unsigned int product_id;
	int ret;

	// Set SPI 4 wire
	ret = regmap_write(priv->regmap, ADMV1455_REG0000, 0x18);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap,
		ADMV1455_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	pr_err("%s: PRODUCT_ID: 0x%X\n", priv->spi->dev.of_node->name, product_id);

	// Disable filter table
	ret = regmap_write(priv->regmap, ADMV1455_REG0202, 0x00);
	if (ret)
		return ret;

	// Disable filter load
	ret = regmap_write(priv->regmap, ADMV1455_REG0208, 0x00);
	if (ret)
		return ret;

	// Enable gain table bypass
	ret = regmap_write(priv->regmap, ADMV1455_REG028A, 0x01);
	if (ret)
		return ret;

	// DSA1 and DSA2 bypass values
	ret = regmap_write(priv->regmap, ADMV1455_REG028B, 0x00);
	if (ret)
		return ret;

	// LPF state determined by SPI
	ret = regmap_write(priv->regmap, ADMV1455_REG02A0, 0x10);
	if (ret)
		return ret;

	// HPF determined by SPI, cutoff frequenct 0x3F (reset value)
	ret = regmap_write(priv->regmap, ADMV1455_REG02A1, 0xBF);
	if (ret)
		return ret;

	// Set GPIO for gain table
	ret = regmap_write(priv->regmap, ADMV1455_REG0603, 0x08);
	if (ret)
		return ret;

return 0;
// Setting direct RF band (0 - low, 1 - high)

// Setting LO band, mixer band, LO sideband and direct LO Phase

// Set direct IF gain

// Setting LO double band

	return 0;
}

static void admv1455_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct regmap_config admv1455_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv1455_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv1455_priv *priv;
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

	priv->regmap = devm_regmap_init_spi(spi, &admv1455_regmap_config);
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

	ret = devm_add_action_or_reset(&spi->dev, admv1455_clk_disable, priv->lo_input);
	if (ret)
		return ret;

	priv->nb.notifier_call = admv1455_freq_change;
	ret = devm_clk_notifier_register(&spi->dev, priv->lo_input, &priv->nb);
	if (ret)
		return ret;

	mutex_init(&priv->lock);

	ret = admv1455_setup(indio_dev);
	if (ret)
		return ret;

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1455_set_filters(priv, rate);
	if (ret) {
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);
		return ret;
	}

	//indio_dev->name = "admv1455";
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv1455_info;

	// return devm_iio_device_register(&spi->dev, indio_dev);
	return 0;
}

static struct spi_driver admv1455_driver = {
	.driver = {
		.name = "admv1455",
	},
	.probe = admv1455_probe,
};
module_spi_driver(admv1455_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1455 Microwave Downconverter Driver");
MODULE_LICENSE("GPL v2");
