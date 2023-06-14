// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for Analog Devices MAX96789 MIPI-DSI serializer over I2C
 *
 * Copyright 2023 NXP
 */

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <linux/mfd/max96789.h>

struct max96789_i2c {
	struct max96789 mfd;

	struct i2c_client *client;
	struct i2c_mux_core *mux;
};

static int max96789_i2c_parse_dt(struct max96789 *max96789)
{
	struct device *dev = max96789->dev;
	struct device_node *i2c_mux;
	struct device_node *node = NULL;

	i2c_mux = of_find_node_by_name(dev->of_node, "i2c-mux");
	if (!i2c_mux) {
		dev_err(dev, "Failed to find i2c-mux node\n");
		return -EINVAL;
	}

	/* Identify which i2c-mux channels are enabled */
	for_each_child_of_node(i2c_mux, node) {
		u32 id = 0;

		of_property_read_u32(node, "reg", &id);
		if (id >= GMSL_MAX_LINKS)
			continue;

		if (!of_device_is_available(node)) {
			dev_dbg(dev, "Skipping disabled I2C bus port %u\n", id);
			continue;
		}

		max96789->gmsl_link_mask |= BIT(id);
		max96789->gmsl_links_used++;
	}
	of_node_put(node);
	of_node_put(i2c_mux);
	of_node_put(dev->of_node);

	return 0;
};

static int max96789_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max96789_i2c *max96789_i2c = i2c_mux_priv(muxc);
	struct max96789 *mfd = &max96789_i2c->mfd;
	int ret;
	unsigned int reg_val;
	unsigned int lock_flag;

	if (mfd->link_setup_finished || mfd->gmsl2_dual_link)
		return 0;

	lock_flag = chan ? LOCK_B : LOCK_A;

	regmap_read(mfd->regmap, MAX96789_TCTRL_INTR7, &reg_val);
	if (reg_val & lock_flag)
		return 0;

	regmap_update_bits(mfd->regmap, MAX96789_DEV_LINK,
				LINK_EN_B | LINK_EN_A, chan ? LINK_EN_B : LINK_EN_A);

	ret = regmap_read_poll_timeout(mfd->regmap, MAX96789_TCTRL_INTR7, reg_val,
				       reg_val & lock_flag, 500, 1000000);
	if (ret < 0) {
		dev_err(mfd->dev, "%s: GMSL link not locked, MAX96789_TCTRL_INTR7 = 0x%02x\n",
			__func__, reg_val);
		return ret;
	}

	return 0;
}

static int max96789_i2c_init(struct max96789_i2c *max96789_i2c)
{
	struct max96789 *mfd = &max96789_i2c->mfd;
	int link;
	int ret;

	if (!i2c_check_functionality(max96789_i2c->client->adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	max96789_i2c->mux = i2c_mux_alloc(max96789_i2c->client->adapter, &max96789_i2c->client->dev,
					  mfd->gmsl_links_used, 0, I2C_MUX_LOCKED,
					  max96789_i2c_mux_select, NULL);
	if (!max96789_i2c->mux)
		return -ENOMEM;

	max96789_i2c->mux->priv = max96789_i2c;

	for (link = 0; link < GMSL_MAX_LINKS; link++) {
		if (!(mfd->gmsl_link_mask & BIT(link)))
			continue;

		ret = i2c_mux_add_adapter(max96789_i2c->mux, 0, link);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	i2c_mux_del_adapters(max96789_i2c->mux);
	return ret;
}

static int max96789_i2c_probe(struct i2c_client *i2c)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(i2c);
	struct max96789_i2c *max96789_i2c;
	int ret;
	const void *of_data;
	ulong expected_dev_id;

	of_data = of_device_get_match_data(&i2c->dev);
	if (of_data)
		expected_dev_id = (ulong)of_data;
	else
		expected_dev_id = id->driver_data;

	max96789_i2c = devm_kzalloc(&i2c->dev, sizeof(*max96789_i2c), GFP_KERNEL);
	if (!max96789_i2c)
		return -ENOMEM;

	max96789_i2c->mfd.dev = &i2c->dev;

	ret = max96789_i2c_parse_dt(&max96789_i2c->mfd);
	if (ret)
		return -ENODEV;

	max96789_i2c->mfd.regmap = devm_regmap_init_i2c(i2c, &max96789_regmap_cfg);
	if (IS_ERR(max96789_i2c->mfd.regmap)) {
		ret = PTR_ERR(max96789_i2c->mfd.regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, &max96789_i2c->mfd);

	max96789_i2c->client = i2c;

	ret = max96789_dev_init(&max96789_i2c->mfd, expected_dev_id);
	if (ret)
		return ret;

	return max96789_i2c_init(max96789_i2c);
}

static void max96789_i2c_remove(struct i2c_client *i2c)
{
}

static struct i2c_device_id max96789_i2c_ids[] = {
	{ "max96789", ID_MAX96789},
	{ }
};
MODULE_DEVICE_TABLE(i2c, max96789_i2c_ids);

static const struct of_device_id max96789_of_match[] = {
	{ .compatible = "maxim,max96789", .data = (void *)ID_MAX96789 },
	{}
};
MODULE_DEVICE_TABLE(of, max96789_of_match);

static struct i2c_driver max96789_i2c_driver = {
	.driver = {
		.name		= "max96789-i2c",
		.of_match_table = of_match_ptr(max96789_of_match),
	},
	.probe		= max96789_i2c_probe,
	.remove		= max96789_i2c_remove,
	.id_table	= max96789_i2c_ids,
};
module_i2c_driver(max96789_i2c_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_DESCRIPTION("MAX96789 MIPI-DSI Serializer with I2C");
MODULE_LICENSE("GPL v2");
