// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

struct ad242x_i2c {
	struct device *dev;
	struct a2b_node *node;
	struct i2c_adapter adap;
};

static int ad242x_set_addr(struct a2b_mainnode *mainnode, uint8_t node_id,
			   uint8_t addr)
{
	int ret;
	uint8_t buf[2] = { A2B_CHIP, addr };

	ret = regmap_update_bits(mainnode->node.regmap, A2B_NODEADR,
				 A2B_NODEADR_PERI | A2B_NODEADR_MASK, node_id);
	if (ret < 0)
		return ret;

	/*
	 * We can't use the slave's regmap here as it holds the same
	 * lock we also need to guard this context.
	 */
	ret = i2c_transfer_buffer_flags(mainnode->bus_client, buf, sizeof(buf),
					0);
	if (ret < 0)
		return ret;

	return regmap_update_bits(mainnode->node.regmap, A2B_NODEADR,
				  A2B_NODEADR_PERI, A2B_NODEADR_PERI);
}

static int ad242x_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			   int num)
{
	struct ad242x_i2c *i2c = adap->algo_data;
	struct a2b_mainnode *mainnode;
	int ret, i, current_addr = -1;

	mainnode = a2b_node_to_subnode(i2c->node)->mainnode;

	mutex_lock(&mainnode->bus_lock);

	for (i = 0; i < num; i++) {
		struct i2c_msg *msg = msgs + i;

		if (msg->addr != current_addr) {
			ret = ad242x_set_addr(mainnode, i2c->node->id,
					      msg->addr);
			if (ret < 0) {
				dev_err(&i2c->node->dev,
					"Cannot set address: %d\n", ret);
				break;
			}

			current_addr = msg->addr;
		}

		ret = i2c_transfer_buffer_flags(mainnode->bus_client, msg->buf,
						msg->len, msg->flags);
		if (ret < 0)
			break;
	}

	mutex_unlock(&mainnode->bus_lock);

	return ret < 0 ? ret : num;
}

static u32 ad242x_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ad242x_i2c_algorithm = {
	.master_xfer = ad242x_i2c_xfer,
	.functionality = ad242x_i2c_functionality,
};

static int ad242x_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ad242x_i2c *i2c;
	struct a2b_node *node;
	u32 freq, val = 0;
	int ret;

	if (!dev->of_node)
		return -ENODEV;

	node = dev_to_a2b_node(dev->parent);
	if (!a2b_node_has_capability(node, A2B_CAPABILITY_I2C)) {
		dev_err(dev, "Node %d has no I2C capability", node->id);
		return -ENODEV;
	}

	if (a2b_node_is_main(node))
		return -ENODEV;

	freq = clk_get_rate(a2b_node_get_sync_clk(node));
	if (freq == 44100)
		val |= A2B_I2CCFG_FRAMERATE;

	if (!of_property_read_u32(dev->of_node, "clock-frequency", &freq)) {
		if (freq == 400000)
			val |= A2B_I2CCFG_DATARATE;
		else if (freq != 100000)
			dev_warn(dev, "Unsupported frequency %d\n", freq);
	}

	ret = regmap_write(node->regmap, A2B_I2CCFG, val);
	if (ret < 0) {
		dev_err(&pdev->dev, "%d\n", __LINE__);
		return ret;
	}

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->node = node;
	i2c->adap.algo = &ad242x_i2c_algorithm;
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = dev;
	i2c->adap.dev.of_node = dev->of_node;
	i2c_set_adapdata(&i2c->adap, i2c);
	strlcpy(i2c->adap.name, "ad242x remote I2C bus",
		sizeof(i2c->adap.name));

	ret = i2c_add_adapter(&i2c->adap);
	if (ret < 0) {
		dev_err(dev, "error registering adapter: %d\n", ret);
		return ret;
	}

	dev_info(dev, "ad242x i2c driver, node ID %d\n", node->id);
	platform_set_drvdata(pdev, i2c);

	return 0;
}

static int ad242x_i2c_remove(struct platform_device *dev)
{
	struct ad242x_i2c *i2c = platform_get_drvdata(dev);

	i2c_del_adapter(&i2c->adap);

	return 0;
}

static const struct of_device_id ad242x_i2c_of_match[] = {
	{ .compatible = "adi,ad2428w-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, ad242x_i2c_of_match);

static struct platform_driver ad242x_i2c_driver = {
	.driver = {
		.name = "ad242x-i2c",
		.of_match_table = ad242x_i2c_of_match,
	},
	.probe = ad242x_i2c_probe,
	.remove = ad242x_i2c_remove,
};

module_platform_driver(ad242x_i2c_driver);
MODULE_LICENSE("GPL");
