// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX2035x RAM Driver
 */
#include <linux/module.h>
#include <linux/platform_device.h>

#include "max2035x.h"
#include "max2035x_registers.h"
#include "max2035x_ram.h"

static const struct regmap_config max2035x_ram_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
	.cache_type = REGCACHE_NONE,
};

int max2035x_write_ram_data(struct max2035x *chip, const u8 *data, size_t len)
{
	struct max2035x_ram *ram;
	int ret;

	if (!chip || !chip->ram_data) {
		pr_err("%s: RAM driver not ready yet\n", __func__);
		return -EPROBE_DEFER;
	}

	ram = chip->ram_data;

	if (len > 128) {
		dev_err(ram->dev, "%s : Data length %zu exceeds RAM size (128B)\n", __func__, len);
		return -EINVAL;
	}

	ret = regmap_bulk_write(ram->regmap, 0x00, data, len);
	if (ret) {
		dev_err(ram->dev, "%s : Failed to write data to RAM data (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(ram->dev, "%s : Successfully write RAM with %zu bytes\n", __func__, len);

	return 0;
}
EXPORT_SYMBOL_GPL(max2035x_write_ram_data);

int max2035x_read_ram_data(struct max2035x *chip, u8 *data, size_t len)
{
	struct max2035x_ram *ram;
	int ret;

	if (!chip || !chip->ram_data) {
		pr_err("%s: RAM driver not ready yet\n", __func__);
		return -EPROBE_DEFER;
	}

	ram = chip->ram_data;

	if (len > 128) {
		dev_err(ram->dev, "%s : Data length %zu exceeds RAM size (128B)\n", __func__, len);
		return -EINVAL;
	}

	ret = regmap_bulk_read(ram->regmap, 0x00, data, len);
	if (ret) {
		dev_err(ram->dev, "%s : Failed to read data from RAM data (ret: %d)\n", __func__, ret);
		return ret;
	}

	dev_info(ram->dev, "%s : Successfully read %zu bytes from RAM\n", __func__, len);
	print_hex_dump(KERN_INFO, "max2035x-ram read: ", DUMP_PREFIX_OFFSET, 16, 1, data, len, false);

	return 0;
}
EXPORT_SYMBOL_GPL(max2035x_read_ram_data);

static int max2035x_ram_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct max2035x_ram *ram;

	ram = devm_kzalloc(&pdev->dev, sizeof(*ram), GFP_KERNEL);
	if (!ram)
		return -ENOMEM;

	ram->dev = &pdev->dev;
	ram->i2c = chip->ram;
	ram->chip = chip;

	ram->regmap = devm_regmap_init_i2c(ram->i2c, &max2035x_ram_regmap_cfg);
	if (IS_ERR(ram->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(ram->regmap), "%s : Failed to initialize regmap\n", __func__);

	chip->ram_data = ram;

	platform_set_drvdata(pdev, ram);

	dev_info(&pdev->dev, "%s : %s RAM driver Probed\n",
		__func__, (chip->type == MAX20355) ? "MAX20355" : "MAX20357");

	return 0;
}

static void max2035x_ram_remove(struct platform_device *pdev)
{
	struct max2035x_ram *ram = platform_get_drvdata(pdev);

	if (ram && ram->chip) {
		ram->chip->ram_data = NULL;
	}

	dev_info(&pdev->dev, "%s : RAM driver removed\n", __func__);
}

static struct platform_driver max20355_ram_driver = {
	.driver = {
		.name = "max20355-ram",
	},
	.probe = max2035x_ram_probe,
	.remove = max2035x_ram_remove,
};

static struct platform_driver max20357_ram_driver = {
	.driver = {
		.name = "max20357-ram",
	},
	.probe = max2035x_ram_probe,
	.remove = max2035x_ram_remove,
};

static struct platform_driver * const ram_drivers[] = {
	&max20355_ram_driver,
	&max20357_ram_driver,
};

int __init max2035x_ram_init(void)
{
	return platform_register_drivers(ram_drivers, ARRAY_SIZE(ram_drivers));
}

void max2035x_ram_exit(void)
{
	platform_unregister_drivers(ram_drivers, ARRAY_SIZE(ram_drivers));
}
