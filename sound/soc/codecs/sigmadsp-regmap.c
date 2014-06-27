/*
 * Load Analog Devices SigmaStudio firmware files
 *
 * Copyright 2009-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/regmap.h>
#include <linux/export.h>
#include <linux/module.h>

#include "sigmadsp.h"

static int sigmadsp_write_regmap(void *control_data,
	unsigned int addr, const uint8_t data[], size_t len)
{
	return regmap_raw_write(control_data, addr,
		data, len);
}

static int sigmadsp_read_regmap(void *control_data,
	unsigned int addr, uint8_t data[], size_t len)
{
	return regmap_raw_read(control_data, addr,
		data, len);
}

void sigmadsp_init_regmap(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops, struct regmap *regmap)
{
	sigmadsp->control_data = regmap;
	sigmadsp->write = sigmadsp_write_regmap;
	sigmadsp->read = sigmadsp_read_regmap;
	sigmadsp_init(sigmadsp, ops);
}
EXPORT_SYMBOL_GPL(sigmadsp_init_regmap);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("SigmaDSP regmap firmware loader");
MODULE_LICENSE("GPL");
