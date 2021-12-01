// SPDX-License-Identifier: GPL-2.0+
/*
 * LTC6946 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/clk-provider.h>

enum supported_parts {
	LTC6946,
};

struct ltc6946 {
    struct regmap *regmap;
    struct clk_hw clk_hw;
};

static const struct regmap_config ltc6946_regmap_config = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.max_register = 0x0B,
};

static int ltc6946_setup(struct iio_dev *indio_dev)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	regmap_write(dev->regmap, 0x01,	0x00);
	regmap_write(dev->regmap, 0x02,	0x0A);

	/* ref = 250 MHz, vco = 3750 MHz, rf = 3750 MHz*/
	regmap_write(dev->regmap, 0x03,	0x00);
	regmap_write(dev->regmap, 0x04,	0x19);
	regmap_write(dev->regmap, 0x05,	0x01);
	regmap_write(dev->regmap, 0x06,	0x77);

	/* Enable ALC Monitor for Status Flags Only, Start VCO Calibration */
	regmap_write(dev->regmap, 0x07,	0x63);

	// Output Divider Value = 1
	regmap_write(dev->regmap, 0x08,	0x81);
	regmap_write(dev->regmap, 0x09,	0xF0);
	regmap_write(dev->regmap, 0x0A,	0x00);

	// Power down PLL and just use the VCO portion to lock the LTC6952
	regmap_write(dev->regmap, 0x02,	0x08);

	return 0;
}

static int ltc6946_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int tx_val,
	unsigned int *rx_val)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	if (rx_val)
		regmap_read(dev->regmap, reg, rx_val);
	else
		regmap_write(dev->regmap, reg, tx_val);

	return 0;
}

static const struct iio_info ltc6946_info = {
	.debugfs_reg_access = &ltc6946_reg_access,
};

static unsigned long ltc6946_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	return 0;
}

static long ltc6946_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	return 0;
}

static int ltc6946_set_rate(struct clk_hw *clk_hw, unsigned long rate,
	unsigned long parent_rate)
{
	return 0;
}

static const struct clk_ops ltc6946_clk_ops = {
	.recalc_rate = ltc6946_recalc_rate,
	.round_rate = ltc6946_round_rate,
	.set_rate = ltc6946_set_rate,
};

static int ltc6946_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct clk_init_data init;
	struct regmap *regmap;
	struct ltc6946 *dev;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ltc6946_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	spi_set_drvdata(spi, indio_dev);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ltc6946_info;
	indio_dev->name = spi->dev.of_node->name;

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	ltc6946_setup(indio_dev);

	init.name = spi->dev.of_node->name;
	init.ops = &ltc6946_clk_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;

	dev->clk_hw.init = &init;

	clk = devm_clk_register(&spi->dev, &dev->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	dev_info(&spi->dev, "%s probed\n", indio_dev->name);

	return of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
}

static int ltc6946_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ltc6946_id[] = {
	{ "ltc6946", LTC6946 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6946_id);

static const struct of_device_id ltc6946_of_match[] = {
	{ .compatible = "adi,ltc6946" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc6946_of_match);

static struct spi_driver ltc6946_driver = {
	.driver = {
			.name = "ltc6946",
			.of_match_table = of_match_ptr(ltc6946_of_match),
		},
	.probe = ltc6946_probe,
	.remove = ltc6946_remove,
	.id_table = ltc6946_id,
};
module_spi_driver(ltc6946_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6946");
MODULE_LICENSE("GPL v2");