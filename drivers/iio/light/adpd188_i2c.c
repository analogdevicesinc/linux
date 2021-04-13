// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADPD188 I2C driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * 7-bit I2C slave address: 0x64
 */

#include <linux/i2c.h>
#include <linux/module.h>

#include "adpd188.h"

#define ADPD188_REG_I2CS_ID			0x09
#define ADPD188_REG_SLAVE_ADDR_KEY		0x0D
#define ADPD188_REG_EXT_SYNC_SEL		0x4F

/** ADPD188_REG_I2CS_ID */
#define ADPD188_ADDRESS_WRITE_KEY_MASK		GENMASK(15, 8)
#define ADPD188_SLAVE_ADDRESS_MASK		GENMASK(7, 1)

/** ADPD188_REG_SLAVE_ADDR_KEY */
#define EN_I2C_ADDR_CHANGE_ALWAYS		0x04AD
#define EN_I2C_ADDR_CHANGE_IF_GPIO0_HIGH	0x44AD
#define EN_I2C_ADDR_CHANGE_IF_GPIO1_HIGH	0x84AD

/** ADPD188_REG_EXT_SYNC_SEL */
#define ADPD188_GPIO1_OE			BIT(6)
#define ADPD188_GPIO1_IE			BIT(5)
#define ADPD188_EXT_SYNC_SEL			GENMASK(3, 2)
#define ADPD188_GPIO0_IE			BIT(1)

struct adpd188_i2c_buff {
	u8 addr;
	__be16 val;
} __packed;

static int adpd188_sub_reg_write(void *bus, int i2c_addr,
			   int reg_addr, int value)
{
	struct i2c_msg msg;
	struct adpd188_i2c_buff buf;
	struct i2c_client *i2c = (struct i2c_client *)bus;

	buf.addr = reg_addr;
	buf.val = cpu_to_be16(value);
	msg.addr = i2c_addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = (u8 *)&buf;

	return i2c_transfer(i2c->adapter, &msg, 1);
}

static int adpd188_sub_reg_read(void *bus, int i2c_addr, int reg_addr,
			  int *value)
{
	int ret;
	struct i2c_msg msg[2];
	struct adpd188_i2c_buff buf;
	struct i2c_client *i2c = (struct i2c_client *)bus;

	msg[0].addr = i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	buf.addr = reg_addr;
	msg[0].buf = &buf.addr;
	msg[1].addr = i2c_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *)&buf.val;

	ret = i2c_transfer(i2c->adapter, msg, 2);
	if (ret < 0)
		return ret;

	*value = be16_to_cpu(buf.val);

	return 0;
}

static int adpd188_sub_reg_write_mask(void *bus, int i2c_addr,
				int reg_addr, int value, int mask)
{
	int ret;
	u16 reg_val;
	struct i2c_client *i2c = (struct i2c_client *)bus;

	ret = adpd188_sub_reg_read(i2c, i2c_addr, reg_addr, (int *)&reg_val);
	if (ret < 0)
		return ret;

	reg_val &= ~mask;
	reg_val |= (value << (ffs(mask) - 1)) & mask;

	return adpd188_sub_reg_write(i2c, i2c_addr, reg_addr, reg_val);
}

static int adpd188_prime_reg_write_mask(void *bus, int addr,
				  int value, int mask)
{
	struct i2c_client *i2c = (struct i2c_client *)bus;

	return adpd188_sub_reg_write_mask(bus, i2c->addr, addr, value, mask);
}

static int adpd188_prime_reg_read(void *bus, int addr, int *value)
{
	struct i2c_client *i2c = (struct i2c_client *)bus;

	return adpd188_sub_reg_read(bus, i2c->addr, addr, value);
}

static int adpd188_prime_reg_write(void *bus, int addr, int value)
{
	struct i2c_client *i2c = (struct i2c_client *)bus;

	return adpd188_sub_reg_write(bus, i2c->addr, addr, value);
}

static int adpd188_i2c_addr_change(struct i2c_client *i2c, int no_devices)
{
	int ret;
	int val;
	int i;

	ret = adpd188_prime_reg_read(i2c, ADPD188_REG_DEVID, &val);
	if (ret < 0) {
		dev_err(&i2c->dev, "i2c read failed. ret=%d\n", ret);
		return ret;
	}
	if ((val & 0xFF) != 0x16) {
		dev_err(&i2c->dev, "i2c read failed. ret=%d val=%x\n", ret, val);
		return ret;
	}

	ret = adpd188_prime_reg_write_mask(i2c, ADPD188_REG_GPIO_DRV,
				  1, ADPD188_GPIO0_ENA);
	if (ret < 0)  {
		dev_err(&i2c->dev, "Error GPIO 0 enable: %d\n", ret);
		return ret;
	}
	ret = adpd188_prime_reg_write_mask(i2c, ADPD188_REG_GPIO_CTRL,
				  0x11, ADPD188_GPIO0_ALT_CONFIG);
	if (ret < 0)  {
		dev_err(&i2c->dev, "Error GPIO 0 state to high: %d\n", ret);
		return ret;
	}

	ret = adpd188_prime_reg_write_mask(i2c, ADPD188_REG_EXT_SYNC_SEL,
				1, ADPD188_GPIO1_IE);
	if (ret < 0)  {
		dev_err(&i2c->dev, "Error 0x4F not accessible: %d\n", ret);
		return ret;
	}

	for (i = 0; i < (no_devices - 1); i++) {
		val = (0xAD << (ffs(ADPD188_ADDRESS_WRITE_KEY_MASK) - 1)) |
			((i2c->addr + i + 1) << (ffs(ADPD188_SLAVE_ADDRESS_MASK) - 1));
		ret = adpd188_sub_reg_write(i2c, (i2c->addr + i), ADPD188_REG_SLAVE_ADDR_KEY,
					EN_I2C_ADDR_CHANGE_IF_GPIO1_HIGH);
		if (ret < 0)  {
			dev_err(&i2c->dev, "Error changing slave address key: %d\n", ret);
			return ret;
		}
		ret = adpd188_sub_reg_write(i2c, (i2c->addr + i), ADPD188_REG_I2CS_ID, val);
		if (ret < 0)  {
			dev_err(&i2c->dev, "Error changing slave address: %d\n", i);
			return ret;
		}
		ret = adpd188_sub_reg_read(i2c, (i2c->addr + i + 1), ADPD188_REG_DEVID, &val);
		if ((ret < 0) || ((val & 0xFF) != 0x16)) {
			dev_err(&i2c->dev, "Failed to access device after reg change.\n");
			return ret;
		}
		ret = adpd188_sub_reg_write_mask(i2c, (i2c->addr + i), ADPD188_REG_GPIO_CTRL,
						0x10, ADPD188_GPIO0_ALT_CONFIG);
		if (ret < 0) {
			dev_err(&i2c->dev, "Error changing GPIO 0 to low: %d\n", i);
			return ret;
		}
	}

	dev_info(&i2c->dev, "adpd188 IDs changed.\n");

	return 0;
}

static int adpd188_i2c_probe(struct i2c_client *i2c)
{
	int ret;
	u32 no_devices;
	struct adpd188_ops *phy;

	ret = of_property_read_u32(i2c->dev.of_node,
				   "adi,no-of-devices", &no_devices);
	if (ret)
		return ret;

	if (no_devices > 1) {
		ret = adpd188_i2c_addr_change(i2c, no_devices);
		if (ret < 0)
			return ret;
	}

	phy = devm_kzalloc(&i2c->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;
	phy->reg_write = adpd188_prime_reg_write;
	phy->reg_read = adpd188_prime_reg_read;
	phy->i2c_sub_write = adpd188_sub_reg_write;
	phy->i2c_sub_read = adpd188_sub_reg_read;

	return adpd188_core_probe(i2c, phy, ADPD188_I2C, no_devices, i2c->name, i2c->irq);
}

static const struct i2c_device_id adpd188_i2c_id[] = {
	{ "adpd188", ADPD188 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adpd188_i2c_id);

static const struct of_device_id adpd188_of_match[] = {
	{ .compatible = "adi,adpd188" },
	{},
};
MODULE_DEVICE_TABLE(of, adpd188_of_match);

static struct i2c_driver adpd188_i2c_driver = {
	.driver = {
			.name = "adpd188_i2c",
			.of_match_table = adpd188_of_match,
		},
	.probe_new = adpd188_i2c_probe,
	.id_table = adpd188_i2c_id,
};
module_i2c_driver(adpd188_i2c_driver);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188 I2C driver");
MODULE_LICENSE("GPL v2");

