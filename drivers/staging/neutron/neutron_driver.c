// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */
/****************************************************************************/

#include <linux/dma-mapping.h>
#include <linux/bitmap.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>

#include "neutron_device.h"

/****************************************************************************/

#define NEUTRON_DRIVER_VERSION "1.0"
#define NEUTRON_DRIVER_NAME    "neutron"

#define MINOR_BASE          (0) /* Minor version starts at 0 */
#define MINOR_COUNT         (64) /* Allocate minor versions */

/****************************************************************************/

static struct class *neutron_class;
static dev_t devt;
static DECLARE_BITMAP(minors, MINOR_COUNT);

/****************************************************************************/

static int neutron_pdev_probe(struct platform_device *pdev)
{
	struct neutron_device *ndev;
	int minor, irq;
	int ret = -ENOMEM;

	dev_dbg(&pdev->dev, "Probe\n");

	minor = find_first_zero_bit(minors, MINOR_COUNT);
	if (minor >= MINOR_COUNT) {
		dev_err(&pdev->dev, "No more minor numbers.\n");
		return -ENOMEM;
	}

	/* Allocate memory for NXP Neutron device */
	ndev = devm_kzalloc(&pdev->dev, sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);

	ndev->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ndev->reg_base))
		goto err_free_dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq: %d\n", irq);
		goto err_free_dev;
	}

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to resume: %d\n", ret);
		goto err_put_pm;
	}

	/* Initialize device */
	ret = neutron_dev_init(ndev, &pdev->dev, irq, neutron_class,
			       MKDEV(MAJOR(devt), minor));
	if (ret)
		goto err_put_pm;

	set_bit(minor, minors);

	return 0;

err_put_pm:
	pm_runtime_disable(&pdev->dev);
err_free_dev:
	devm_kfree(&pdev->dev, ndev);

	return ret;
}

static int neutron_pdev_remove(struct platform_device *pdev)
{
	struct neutron_device *ndev = platform_get_drvdata(pdev);

	neutron_rproc_shutdown(ndev);
	clear_bit(MINOR(ndev->devt), minors);
	neutron_dev_deinit(ndev);
	pm_runtime_disable(ndev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int neutron_runtime_suspend(struct device *dev)
{
	struct neutron_device *ndev = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ndev->num_clks, ndev->clks);

	return 0;
}

static int neutron_runtime_resume(struct device *dev)
{
	struct neutron_device *ndev = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(ndev->num_clks, ndev->clks);
	if (ret)
		dev_err(ndev->dev, "failed to enable clock\n");

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int neutron_suspend(struct device *dev)
{
	struct neutron_device *ndev = dev_get_drvdata(dev);

	if (ndev->power_state == NEUTRON_POWER_ON)
		neutron_rproc_shutdown(ndev);

	pm_runtime_force_suspend(dev);

	return 0;
}

static int neutron_resume(struct device *dev)
{
	struct neutron_device *ndev = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		pr_err("neutron: failed to resume\n");

	/* Start the neutron core only when it is ON state before sleeping */
	if (ndev->power_state == NEUTRON_POWER_ON)
		neutron_rproc_boot(ndev, NULL);

	return 0;
}

#endif

static const struct dev_pm_ops neutron_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(neutron_suspend, neutron_resume)
	SET_RUNTIME_PM_OPS(neutron_runtime_suspend, neutron_runtime_resume, NULL)
};

static const struct of_device_id neutron_pdev_match[] = {
	{ .compatible = "fsl,imx95-neutron" },
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, neutron_pdev_match);

static struct platform_driver neutron_pdev_driver = {
	.probe		  = &neutron_pdev_probe,
	.remove		 = &neutron_pdev_remove,
	.driver		 = {
		.name	   = NEUTRON_DRIVER_NAME,
		.owner	  = THIS_MODULE,
		.of_match_table = of_match_ptr(neutron_pdev_match),
		.pm = &neutron_dev_pm_ops,
	},
};

/****************************************************************************/

static int __init neutron_init(void)
{
	int ret;

	neutron_class = class_create(NEUTRON_DRIVER_NAME);
	if (IS_ERR(neutron_class)) {
		pr_err("Failed to create class '%s'.\n", NEUTRON_DRIVER_NAME);
		return PTR_ERR(neutron_class);
	}

	ret = alloc_chrdev_region(&devt, MINOR_BASE, MINOR_COUNT,
				  NEUTRON_DRIVER_NAME);
	if (ret) {
		pr_err("Failed to allocate chrdev region.\n");
		goto destroy_class;
	}

	ret = platform_driver_register(&neutron_pdev_driver);
	if (ret) {
		pr_err("Failed to register imx neutron npu driver.\n");
		goto region_unregister;
	}

	return 0;

region_unregister:
	unregister_chrdev_region(devt, MINOR_COUNT);

destroy_class:
	class_destroy(neutron_class);

	return ret;
}

static void __exit neutron_exit(void)
{
	platform_driver_unregister(&neutron_pdev_driver);
	unregister_chrdev_region(devt, MINOR_COUNT);
	class_destroy(neutron_class);
}

module_init(neutron_init)
module_exit(neutron_exit)
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("i.MX Neutron NPU Driver");
MODULE_VERSION(NEUTRON_DRIVER_VERSION);
