// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1355 Microwave Upconverter Driver
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/notifier.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>

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

static int admv1355_update(struct admv1355_priv *priv)
{
	u64 rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);

	pr_err("Frequency changed. New rate: %llu.\n", rate);

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

	ret = regmap_write(priv->regmap, 0x000, 0x18);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap,
		ADMV1355_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	pr_err("%s: PRODUCT_ID: 0x%X\n", priv->spi->dev.of_node->name, product_id);

return 0;
	ret = regmap_write(priv->regmap, 0x13E, 0x02);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x202, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x208, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x28A, 0x01);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x28B, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x2A0, 0x10);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x2A1, 0xBF);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x601, 0x08);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x800, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x801, 0x1C);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x802, 0x1F);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x804, 0x18);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x805, 0xC3);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x806, 0xF0);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x80A, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, 0x80C, 0x1D);
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
