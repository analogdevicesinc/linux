// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 I2C Negative Voltage Hot Swap Controller
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include <linux/mfd/ltc4283.h>

static const struct mfd_cell ltc4283_cells[] = {
	MFD_CELL_OF("ltc4283-hwmon", NULL, NULL, 0, 0, "adi,ltc4283-hwmon"),
	MFD_CELL_OF("ltc4283-gpio", NULL, NULL, 0, 0, "adi,ltc4283-gpio"),
};

static bool ltc4283_writable_reg(struct device *dev, unsigned int reg)
{
	/* All read only reserved registers */
	switch (reg) {
	case 0x00 ... 0x03:
		return false;
	case 0x3c:
		return false;
	case 0x86 ... 0x8f:
		return false;
	case 0x91 ... 0xa1:
		return false;
	case 0xa3:
		return false;
	case 0xac:
		return false;
	case 0xf1 ... 0xff:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config ltc4283_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.writeable_reg = ltc4283_writable_reg,
};

static int ltc4283_get_gpio_pins(struct i2c_client *client, u32 *n_cells)
{
	struct device *dev = &client->dev;
	u32 pins[LTC4283_GPIO_MAX], pin;
	unsigned long *gpio_mask;
	int n_pins, ret;

	n_pins = device_property_count_u32(dev, "adi,gpio-pins");
	if (n_pins < 0)
		return 0;
	if (n_pins >= LTC4283_GPIO_MAX)
		return dev_err_probe(dev, -EINVAL, "Too many GPIO pins specified (%d), max is %d\n",
				     n_pins, LTC4283_GPIO_MAX);

	ret = device_property_read_u32_array(dev, "adi,gpio-pins", pins, n_pins);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read GPIO pins\n");

	gpio_mask = devm_bitmap_zalloc(dev, LTC4283_GPIO_MAX, GFP_KERNEL);
	if (!gpio_mask)
		return -ENOMEM;

	for (pin = 0; pin < n_pins; pin++) {
		if (pins[pin] >= LTC4283_GPIO_MAX)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid GPIO pin specified (%u), max is %d\n",
					     pins[pin], LTC4283_GPIO_MAX);

		__set_bit(pins[pin], gpio_mask);
	}

	/* Add the GPIO cell */
	*n_cells += 1;
	i2c_set_clientdata(client, gpio_mask);

	return 0;
}

static int ltc4283_probe(struct i2c_client *client)
{
	u32 n_cells = ARRAY_SIZE(ltc4283_cells) - 1;
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &ltc4283_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = ltc4283_get_gpio_pins(client, &n_cells);
	if (ret)
		return ret;

	return devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				    ltc4283_cells, n_cells, NULL, 0, NULL);
}

static const struct of_device_id ltc4283_of_match[] = {
	{ .compatible = "adi,ltc4283" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4283_of_match);

static const struct i2c_device_id ltc4283_i2c_id[] = {
	{ "ltc4283" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4283_i2c_id);

static struct i2c_driver ltc4283_driver = {
	.driver = {
		.name = "ltc4283",
		.of_match_table = ltc4283_of_match,
	},
	.probe = ltc4283_probe,
	.id_table = ltc4283_i2c_id,
};
module_i2c_driver(ltc4283_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("LTC4283 MFD I2C driver");
MODULE_LICENSE("GPL v2");
