/*
 * JESD204B Altera Configuration Driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

struct jesd204b_altera_state {
	struct device 		*dev;
	void __iomem		*regs;
	struct clk 		*clk;
	unsigned long		rate;

	struct gpio_desc	*rx_sw_rstn_s;
	struct gpio_desc	*rx_sysref_s;
};

static inline void jesd204b_altera_write(struct jesd204b_altera_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int jesd204b_altera_read(struct jesd204b_altera_state *st,
	unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int jesd204b_altera_probe(struct platform_device *pdev)
{
	struct jesd204b_altera_state *st;
	struct resource *mem;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return -EPROBE_DEFER;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev = &pdev->dev;
	st->clk = clk;
	platform_set_drvdata(pdev, st);

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	st->rate = clk_get_rate(clk);

	st->rx_sw_rstn_s = devm_gpiod_get(&pdev->dev, "rx_sw_rstn_s");
	if (!IS_ERR(st->rx_sw_rstn_s)) {
		gpiod_direction_output(st->rx_sw_rstn_s, 0);
	}
	st->rx_sysref_s = devm_gpiod_get(&pdev->dev, "rx_sysref_s");
	if (!IS_ERR(st->rx_sysref_s)) {
		gpiod_direction_output(st->rx_sysref_s, 0);
	}

	mdelay(10);
	gpiod_set_value(st->rx_sw_rstn_s, 1);
	mdelay(10);
	gpiod_set_value(st->rx_sysref_s, 1);
	mdelay(10);
	gpiod_set_value(st->rx_sysref_s, 0);

	dev_info(&pdev->dev, "JESD204B Altera probed\n");

	return 0;
}

static int jesd204b_altera_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id jesd204b_altera_of_match[] = {
	{ .compatible = "adi,jesd204b-altera-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, jesd204b_altera_of_match);

static struct platform_driver jesd204b_altera_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_altera_of_match,
	},
	.probe	= jesd204b_altera_probe,
	.remove	= jesd204b_altera_remove,
};

module_platform_driver(jesd204b_altera_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("JESD204B Altera Configuration Driver");
MODULE_LICENSE("GPL v2");
