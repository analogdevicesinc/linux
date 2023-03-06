// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic IIO fake device driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>

struct adi_iio_fakedev {
	struct device_node *np;
	struct device *dev;
};

static int adi_iio_fakedev_attach_client(struct device *dev, void *data)
{
	struct adi_iio_fakedev *fdev = data;

	if ((fdev->np == dev->of_node) && dev->driver) {
		fdev->dev = dev;
		return 1;
	}

	return 0;
}

static void adi_iio_fakedev_cleanup(void *data)
{
	struct device *dev = data;

	put_device(dev);
	module_put(dev->driver->owner);
}

static void adi_iio_fakedev_cleanup_links(void *data)
{
	struct platform_device *pdev = data;
	struct property *prop;
	const char *name;

	of_property_for_each_string(pdev->dev.of_node,
		"adi,attribute-names", prop, name)
		sysfs_remove_link(&pdev->dev.kobj, name);
}

static const struct iio_info adi_iio_fakedev_info = {
};

/* Match table for of_platform binding */
static const struct of_device_id adi_iio_fakedev_of_match[] = {
	{ .compatible = "adi,iio-fake-platform-device", .data = &platform_bus_type},
	{ .compatible = "adi,iio-fake-spi-device", .data = &spi_bus_type},
	{ .compatible = "adi,iio-fake-i2c-device", .data = &i2c_bus_type},
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, adi_iio_fakedev_of_match);

static int adi_iio_fakedev_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct adi_iio_fakedev fdev;
	const struct of_device_id *id;
	struct bus_type *bus;
	struct property *prop;
	const char *name;
	int ret, cnt = 0;

	id = of_match_device(adi_iio_fakedev_of_match, &pdev->dev);
	if (!id || !id->data)
		return -ENODEV;

	bus = (struct bus_type *) id->data;

	/* Defer driver probe until matching driver is registered */
	fdev.np = of_parse_phandle(pdev->dev.of_node,
						 "adi,faked-dev", 0);
	if (!fdev.np) {
		dev_err(&pdev->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(bus, NULL, &fdev, adi_iio_fakedev_attach_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(fdev.dev->driver->owner))
		return -ENODEV;

	get_device(fdev.dev);

	ret = devm_add_action_or_reset(&pdev->dev, adi_iio_fakedev_cleanup, fdev.dev);
	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, 0);
	if (indio_dev == NULL)
		return -ENOMEM;

	indio_dev->name = "adi-iio-fakedev";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &adi_iio_fakedev_info;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	of_property_for_each_string(pdev->dev.of_node, "adi,attribute-names", prop, name) {
		ret = compat_only_sysfs_link_entry_to_kobj(&indio_dev->dev.kobj,
			&fdev.dev->kobj, name, NULL);
		if (!ret)
			cnt++;
	}

	ret = devm_add_action_or_reset(&pdev->dev, adi_iio_fakedev_cleanup_links, pdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Faking %d attributes from %s", cnt, dev_name(fdev.dev));

	return 0;
}

static struct platform_driver adi_iio_fakedev_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = adi_iio_fakedev_of_match,
	},
	.probe = adi_iio_fakedev_probe,
};
module_platform_driver(adi_iio_fakedev_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic IIO fake device driver");
MODULE_LICENSE("GPL v2");
