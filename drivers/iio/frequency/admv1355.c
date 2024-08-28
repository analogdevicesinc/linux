// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1355 Microwave Upconverter Driver
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

#define ADMV1355_REG_PRODUCT_ID_LSB	0x004

#define ADMV1355_REG0000	0x000//	0x18	Enable SDO pin.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x000, 0x18);
#define ADMV1355_REG013E	0x13E//	0x02	RF_HS_PD enabled, so unused band powered down.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x13E, 0x02);
#define ADMV1355_REG0202	0x202//	0x00	Disable filter lookup table.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x202, 0x00);
#define ADMV1355_REG0208	0x208//	0x00	Disable filter load enable bit.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x208, 0x00);
#define ADMV1355_REG028A	0x28A//	0x01	Gain table bypass enabled, so gain can be set with register 0x28B.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x28A, 0x01);
#define ADMV1355_REG028B	0x28B//	0x00	Set DSA1 and DSA2 for max gain.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x28B, 0x00);
#define ADMV1355_REG02A0	0x2A0//	0x10	Set RF chain LPF filter to highest cutoff.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x2A0, 0x10);
#define ADMV1355_REG02A1	0x2A1//	0xBF	Set RF chain HPF filter to lowest cutoff.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x2A1, 0xBF);
#define ADMV1355_REG0601	0x601//	0x08	Set IF chain for BB IQ input.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x601, 0x08);
#define ADMV1355_REG0800	0x800//	0x00	Set LOx4 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x800, 0x00);
#define ADMV1355_REG0801	0x801//	0x1C	Set LOx3 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x801, 0x1C);
#define ADMV1355_REG0802	0x802//	0x1F	Set LOx1 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x802, 0x1F);
#define ADMV1355_REG0804	0x804//	0x18	Set LO trap filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x804, 0x18);
#define ADMV1355_REG0805	0x805//	0xC3	Set RF chain, the phase of LO Q path to 16, and the upper portion of LO trap filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x805, 0xC3);
#define ADMV1355_REG0806	0x806//	0xF0	Set the LO band, mixer band, the sideband, and the phase LO I path to 16.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x806, 0xF0);
#define ADMV1355_REG080A	0x80A//	0x00	Set the IF gain adjust to 0dB, adjust this as needed for SBR.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x80A, 0x00);
#define ADMV1355_REG080C	0x80C//	0x1D	Set the LO doubler band.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x80C, 0x1D);

struct admv1355_priv {
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

static int admv1355_debug_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int write_val, unsigned int *read_val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(priv->regmap, reg, read_val);
	else
		return regmap_write(priv->regmap, reg, write_val);
}

static const struct iio_info admv1355_info = {
	.debugfs_reg_access = &admv1355_debug_reg_access,
};

static int admv1355_set_filters(struct admv1355_priv *priv, u64 rate_khz)
{
	u8 lo_trap_filter_7_0;
	u8 lo_trap_filter_9_8;
	u8 lo_doubler_band;
	u16 lo_trap_filter;
	u8 lo_x4_filter;
	u8 lo_x3_filter;
	u8 lo_x1_filter;
	int ret;

	// LO X4 FILTER REG0800
	// LO X3 FILTER REG0801
	// LO X1 FILTER REG0802
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

	ret = regmap_write(priv->regmap, ADMV1355_REG0800, lo_x4_filter);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1355_REG0801, lo_x3_filter);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1355_REG0802, lo_x1_filter);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1355_REG0804, lo_trap_filter_7_0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1355_REG0805, 0x03, lo_trap_filter_9_8);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADMV1355_REG080C, lo_doubler_band);
	if (ret)
		return ret;

	return 0;
}

static int admv1355_update(struct admv1355_priv *priv)
{
	int ret;
	u64 rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1355_set_filters(priv, rate);
	if (ret)
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);

	return 0;
}

static int admv1355_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct admv1355_priv *priv = container_of(nb, struct admv1355_priv, nb);
	int ret;

	if (action == POST_RATE_CHANGE) {
		mutex_lock(&priv->lock);
		ret = notifier_from_errno(admv1355_update(priv));
		mutex_unlock(&priv->lock);
		return ret;
	}

	return NOTIFY_OK;
}

#if 0
0x000	0x18	Enable SDO pin.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x000, 0x18);
0x13E	0x02	RF_HS_PD enabled, so unused band powered down.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x13E, 0x02);
0x202	0x00	Disable filter lookup table.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x202, 0x00);
0x208	0x00	Disable filter load enable bit.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x208, 0x00);
0x28A	0x01	Gain table bypass enabled, so gain can be set with register 0x28B.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x28A, 0x01);
0x28B	0x00	Set DSA1 and DSA2 for max gain.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x28B, 0x00);
0x2A0	0x10	Set RF chain LPF filter to highest cutoff.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x2A0, 0x10);
0x2A1	0xBF	Set RF chain HPF filter to lowest cutoff.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x2A1, 0xBF);
0x601	0x08	Set IF chain for BB IQ input.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x601, 0x08);
0x800	0x00	Set LOx4 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x800, 0x00);
0x801	0x1C	Set LOx3 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x801, 0x1C);
0x802	0x1F	Set LOx1 filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x802, 0x1F);
0x804	0x18	Set LO trap filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x804, 0x18);
0x805	0xC3	Set RF chain, the phase of LO Q path to 16, and the upper portion of LO trap filter.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x805, 0xC3);
0x806	0xF0	Set the LO band, mixer band, the sideband, and the phase LO I path to 16.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x806, 0xF0);
0x80A	0x00	Set the IF gain adjust to 0dB, adjust this as needed for SBR.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x80A, 0x00);
0x80C	0x1D	Set the LO doubler band.	@Subsystem_1.ADMV1355 Board.ADMV1355: Evaluation.Control.WriteRegister(0x80C, 0x1D);
#endif

static int admv1355_setup(struct iio_dev *indio_dev)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int product_id;
	int ret;

	// Set SPI 4 wire
	ret = regmap_write(priv->regmap, ADMV1355_REG0000, 0x18);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap,
		ADMV1355_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	pr_err("%s: PRODUCT_ID: 0x%X\n", priv->spi->dev.of_node->name, product_id);

	// Power down unused components
	ret = regmap_write(priv->regmap, ADMV1355_REG013E, 0x02);
	if (ret)
		return ret;

	// Disable filter table
	ret = regmap_write(priv->regmap, ADMV1355_REG0202, 0x00);
	if (ret)
		return ret;

	// Disable filter load
	ret = regmap_write(priv->regmap, ADMV1355_REG0208, 0x00);
	if (ret)
		return ret;

	// Enable gain table bypass
	ret = regmap_write(priv->regmap, ADMV1355_REG028A, 0x01);
	if (ret)
		return ret;

	// DSA1 and DSA2 bypass values
	ret = regmap_write(priv->regmap, ADMV1355_REG028B, 0x00);
	if (ret)
		return ret;

	// LPF state determined by SPI
	ret = regmap_write(priv->regmap, ADMV1355_REG02A0, 0x10);
	if (ret)
		return ret;

	// HPF determined by SPI, cutoff frequenct 0x3F (reset value)
	ret = regmap_write(priv->regmap, ADMV1355_REG02A1, 0xBF);
	if (ret)
		return ret;

	// Set GPIO for gain table
	ret = regmap_write(priv->regmap, ADMV1355_REG0601, 0x08);
	if (ret)
		return ret;

return 0;
// Setting direct RF band (0 - low, 1 - high)
	ret = regmap_update_bits(priv->regmap, ADMV1355_REG0805, 0x80, 0);
	if (ret)
		return ret;

// Setting LO band, mixer band, LO sideband and direct LO Phase
	ret = regmap_write(priv->regmap, ADMV1355_REG0806, 0xF0);
	if (ret)
		return ret;

// Set direct IF gain
	ret = regmap_write(priv->regmap, ADMV1355_REG080A, 0x00);
	if (ret)
		return ret;

	return 0;
}

static void admv1355_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct regmap_config admv1355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv1355_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admv1355_priv *priv;
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

	priv->regmap = devm_regmap_init_spi(spi, &admv1355_regmap_config);
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

	ret = devm_add_action_or_reset(&spi->dev, admv1355_clk_disable, priv->lo_input);
	if (ret)
		return ret;

	priv->nb.notifier_call = admv1355_freq_change;
	ret = devm_clk_notifier_register(&spi->dev, priv->lo_input, &priv->nb);
	if (ret)
		return ret;

	mutex_init(&priv->lock);

	ret = admv1355_setup(indio_dev);
	if (ret)
		return ret;

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	ret = admv1355_set_filters(priv, rate);
	if (ret) {
		pr_err("Cannot apply settings for LO rate %llu.\n", rate);
		return ret;
	}

	//indio_dev->name = "admv1355";
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv1355_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver admv1355_driver = {
	.driver = {
		.name = "admv1355",
	},
	.probe = admv1355_probe,
};
module_spi_driver(admv1355_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1355 Microwave Upconverter Driver");
MODULE_LICENSE("GPL v2");
