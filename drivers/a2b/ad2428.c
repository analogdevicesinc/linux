static const struct regmap_config ad242x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= ad242x_is_volatile_reg,
	.writeable_reg	= ad242x_is_writeable_reg,
	.max_register	= AD242X_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
};

static int ad242x_master_probe(struct i2c_client *i2c)
{
	a2b_main_node_register(&i2c->dev, i2c->irq);
}

{static const struct of_device_id ad2428_main_of_match[] = {
	{ .compatible = "adi,ad2428w-main" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad2428_main_of_match);


static const struct i2c_device_id ad2428_main_i2c_id[] = {
	{"ad2428-main", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad2428_main_i2c_id);

static struct i2c_driver ad2428_main_i2c_driver = {
	.driver	= {
		.name = "ad2428-main",
		.of_match_table	= ad2428_main_of_match,
	},
	.probe_new = ad2428_main_probe,
	.remove = ad2428_main_remove,
	.id_table = ad2428_main_i2c_id,
};

module_i2c_driver(ad2428_main_i2c_driver);

MODULE_DESCRIPTION("AD2428 main-node driver");
MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_LICENSE("GPL v2");
