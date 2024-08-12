// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADdummy clock
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/mutex.h>
#include <linux/clk.h>

static unsigned long dummy_pll_rate = 50000000;  // Default 50 MHz
static struct clk *dummy_pll_clk;
static DEFINE_MUTEX(dummy_pll_lock);

// Recalculate the rate based on the internal stored rate
static unsigned long dummy_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
    return dummy_pll_rate;
}

// Round the rate to the nearest valid rate (if needed)
static long dummy_pll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *parent_rate)
{
    return rate;
}

// Set the rate to a new value
static int dummy_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
    mutex_lock(&dummy_pll_lock);
    dummy_pll_rate = rate;
    mutex_unlock(&dummy_pll_lock);
    return 0;  // Return 0 to indicate success
}

static const struct clk_ops dummy_pll_ops = {
    .recalc_rate = dummy_pll_recalc_rate,
    .round_rate  = dummy_pll_round_rate,
    .set_rate    = dummy_pll_set_rate,
};

static struct clk_hw dummy_pll_hw = {
    .init = &(struct clk_init_data){
        .name = "dummy_pll",
        .ops = &dummy_pll_ops,
        .flags = 0,  // Optional flags if needed
        .parent_names = NULL,  // No parent
        .num_parents = 0,
    },
};

// IIO Attribute for reading and setting the PLL frequency
static ssize_t dummy_pll_frequency_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    unsigned long rate;

    mutex_lock(&dummy_pll_lock);
    rate = dummy_pll_rate;
    mutex_unlock(&dummy_pll_lock);

    return sprintf(buf, "%lu\n", rate);
}

static ssize_t dummy_pll_frequency_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t len)
{
    unsigned long rate;
    int ret;

    ret = kstrtoul(buf, 10, &rate);
    if (ret)
        return ret;

    // Set the new PLL rate
    clk_set_rate(dummy_pll_clk, rate);

    return len;
}

static IIO_DEVICE_ATTR(frequency, 0644,
                       dummy_pll_frequency_show,
                       dummy_pll_frequency_store, 0);

static struct attribute *dummy_pll_attributes[] = {
    &iio_dev_attr_frequency.dev_attr.attr,
    NULL,
};

static const struct attribute_group dummy_pll_attribute_group = {
    .attrs = dummy_pll_attributes,
};

static const struct iio_info dummy_pll_info = {
    .attrs = &dummy_pll_attribute_group,
};

static int dummy_pll_probe(struct platform_device *pdev)
{
    struct iio_dev *indio_dev;
    struct clk_hw *hw = &dummy_pll_hw;

    // Register the PLL clock
    dummy_pll_clk = clk_register(NULL, hw);
    if (IS_ERR(dummy_pll_clk))
        return PTR_ERR(dummy_pll_clk);

    // Make clk provider
    if (!IS_ERR(dummy_pll_clk))
	of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, dummy_pll_clk);

    // Allocate and register the IIO device
    indio_dev = devm_iio_device_alloc(&pdev->dev, 0);
    if (!indio_dev) {
        clk_unregister(dummy_pll_clk);
        return -ENOMEM;
    }

    indio_dev->name = "dummy_pll";
    indio_dev->info = &dummy_pll_info;
    indio_dev->modes = INDIO_DIRECT_MODE;

    platform_set_drvdata(pdev, indio_dev);

    pr_err("Dummy input clock probed.\n");

    return devm_iio_device_register(&pdev->dev, indio_dev);
}

static int dummy_pll_remove(struct platform_device *pdev)
{
    struct clk *clk = platform_get_drvdata(pdev);
    clk_unregister(clk);
    return 0;
}

// Device ID table
static const struct of_device_id dummy_pll_of_match[] = {
    { .compatible = "myvendor,dummy-pll", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dummy_pll_of_match);

static struct platform_driver dummy_pll_driver = {
    .probe  = dummy_pll_probe,
    .remove = dummy_pll_remove,
    .driver = {
        .name = "dummy_pll",
	.of_match_table = dummy_pll_of_match,
        .owner = THIS_MODULE,
	
    },
};

module_platform_driver(dummy_pll_driver);

MODULE_AUTHOR("ADI");
MODULE_DESCRIPTION("Analog Devices Clk Dummy");
MODULE_LICENSE("GPL v2");
