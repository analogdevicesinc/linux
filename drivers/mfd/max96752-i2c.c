// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for Analog Devices MAX96752 deserializer over I2C
 *
 * Copyright 2023 NXP
 */

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include <linux/mfd/maxim_serdes.h>
#include <linux/mfd/max96752.h>

struct max96752_i2c {
	struct max96752 mfd;

	struct i2c_client *client;
	struct i2c_mux_core *gate;
};

static int max96752_i2c_change_addr(struct i2c_client *i2c)
{
	struct max96752 *max96752 = i2c_get_clientdata(i2c);
	struct device *dev = max96752->dev;
	unsigned int addr[2];
	int ret;

	ret = of_property_count_u32_elems(dev->of_node, "reg");
	if (ret == 1 || ret < 0)
		return 0;

	ret = of_property_read_u32_array(dev->of_node, "reg", addr, 2);
	if (ret < 0) {
		dev_err(&i2c->dev, "no reg property set in DT\n");
		return ret;
	}

	ret = regmap_write(max96752->regmap, MAX96752_DEV_REG0, addr[1] << 1);
	if (ret) {
		dev_err(&i2c->dev, "could not change device I2C address to 0x%02x\n", addr[1]);
		return ret;
	}

	i2c->addr = addr[1];

	dev_info(dev, "device I2C address changed to 0x%02x\n", addr[1]);

	return 0;
}

static int max96752_i2c_get_link_no(struct i2c_client *i2c)
{
	struct device_node *parent_node;
	unsigned int bus_no;
	int ret;

	parent_node = of_get_parent(i2c->dev.of_node);
	ret = of_property_read_u32(parent_node, "reg", &bus_no);
	if (ret < 0) {
		dev_err(&i2c->dev, "could not get i2c bus no\n");
		of_node_put(parent_node);
		return ret;
	}
	of_node_put(parent_node);

	return bus_no;
}

static int max96752_i2c_gate_select(struct i2c_mux_core *gate, u32 chan)
{
	struct max96752_i2c *max96752_i2c = i2c_mux_priv(gate);
	struct max96752 *mfd = &max96752_i2c->mfd;

	return regmap_update_bits(mfd->regmap, MAX96752_DEV_REG2, DIS_LOCAL_CC, 0);
}

static int max96752_i2c_gate_deselect(struct i2c_mux_core *gate, u32 chan)
{
	struct max96752_i2c *max96752_i2c = i2c_mux_priv(gate);
	struct max96752 *mfd = &max96752_i2c->mfd;

	return regmap_update_bits(mfd->regmap, MAX96752_DEV_REG2, DIS_LOCAL_CC, DIS_LOCAL_CC);
}

static int max96752_i2c_init(struct max96752_i2c *max96752_i2c)
{
	struct i2c_client *i2c = max96752_i2c->client;
	int ret;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	max96752_i2c->gate = i2c_mux_alloc(i2c->adapter, &i2c->dev, 1, 0,
					   I2C_MUX_LOCKED | I2C_MUX_GATE,
					   max96752_i2c_gate_select, max96752_i2c_gate_deselect);
	if (!max96752_i2c->gate)
		return -ENOMEM;

	max96752_i2c->gate->priv = max96752_i2c;

	ret = i2c_mux_add_adapter(max96752_i2c->gate, 0, 0);
	if (ret < 0)
		goto error;

	return 0;

error:
	i2c_mux_del_adapters(max96752_i2c->gate);
	return ret;
}

static void late_probe_cb(void *data)
{
	struct max96752 *max96752 = data;
	struct max96752_i2c *max96752_i2c = container_of(max96752, struct max96752_i2c, mfd);
	int ret;

	ret = max96752_i2c_init(max96752_i2c);
	if (ret) {
		dev_err(max96752->dev, "i2c initialization failed: %d\n", ret);
		return;
	}

	max96752->link_setup_finished = true;
}

static int max96752_i2c_probe(struct i2c_client *i2c)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(i2c);
	struct max96752_i2c *max96752_i2c;
	int ret;
	const void *of_data;
	ulong expected_dev_id;

	of_data = of_device_get_match_data(&i2c->dev);
	if (of_data)
		expected_dev_id = (ulong)of_data;
	else
		expected_dev_id = id->driver_data;

	max96752_i2c = devm_kzalloc(&i2c->dev, sizeof(*max96752_i2c), GFP_KERNEL);
	if (!max96752_i2c)
		return -ENOMEM;

	max96752_i2c->mfd.regmap = devm_regmap_init_i2c(i2c, &max96752_regmap_cfg);
	if (IS_ERR(max96752_i2c->mfd.regmap)) {
		ret = PTR_ERR(max96752_i2c->mfd.regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	max96752_i2c->mfd.dev = &i2c->dev;

	i2c_set_clientdata(i2c, &max96752_i2c->mfd);

	max96752_i2c->client = i2c;

	ret = max96752_dev_check(&max96752_i2c->mfd, expected_dev_id);
	if (ret)
		return ret;

	max96752_i2c->mfd.link_id = max96752_i2c_get_link_no(i2c);
	if (max96752_i2c->mfd.link_id < 0)
		return max96752_i2c->mfd.link_id;

	ret = max96752_i2c_change_addr(i2c);
	if (ret)
		return ret;

	ret = max96752_dev_init(&max96752_i2c->mfd);
	if (ret < 0) {
		dev_err(&i2c->dev, "unable to setup gmsl link: %d\n", ret);
		return ret;
	}

	return maxim_serdes_chain_register_remote(&i2c->dev, max96752_i2c->mfd.link_id,
						  late_probe_cb, &max96752_i2c->mfd);
}

static void max96752_i2c_remove(struct i2c_client *i2c)
{
}

static struct i2c_device_id max96752_i2c_ids[] = {
	{ "max96752", ID_MAX96752},
	{ }
};
MODULE_DEVICE_TABLE(i2c, max96752_i2c_ids);

static const struct of_device_id max96752_of_match[] = {
	{ .compatible = "maxim,max96752", .data = (void *)ID_MAX96752 },
	{}
};
MODULE_DEVICE_TABLE(of, max96752_of_match);

static struct i2c_driver max96752_i2c_driver = {
	.driver = {
		.name		= "max96752-i2c",
		.of_match_table = of_match_ptr(max96752_of_match),
	},
	.probe		= max96752_i2c_probe,
	.remove		= max96752_i2c_remove,
	.id_table	= max96752_i2c_ids,
};
module_i2c_driver(max96752_i2c_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_DESCRIPTION("MAX96752 GMSL deserializer with I2C");
MODULE_LICENSE("GPL v2");
