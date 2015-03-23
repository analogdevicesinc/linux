/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include "ad5592r.h"

#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/module.h>

#define MODE_CONF		(0 << 4)
#define MODE_DAC_WRITE		(1 << 4)
#define MODE_ADC_READBACK	(4 << 4)
#define MODE_DAC_READBACK	(5 << 4)
#define MODE_GPIO_READBACK	(6 << 4)
#define MODE_REG_READBACK	(7 << 4)

static int ad5593r_dac_write(struct ad5592r_state *st, unsigned chan, u16 value)
{
	struct i2c_client *i2c = container_of(st->dev, struct i2c_client, dev);

	value = cpu_to_be16(value);
	return i2c_smbus_write_word_data(i2c, MODE_DAC_WRITE | chan, value);
}

static int ad5593r_adc_read(struct ad5592r_state *st, unsigned chan, u16 *value)
{
	struct i2c_client *i2c = container_of(st->dev, struct i2c_client, dev);
	s32 val = i2c_smbus_write_word_data(i2c,
			MODE_CONF | AD5592R_REG_ADC_SEQ,
			cpu_to_be16(BIT(chan)));
	if (val < 0)
		return (int) val;

	i2c_smbus_read_word_data(i2c, MODE_ADC_READBACK); /* Invalid data */

	val = i2c_smbus_read_word_data(i2c, MODE_ADC_READBACK);
	if (val < 0)
		return (int) val;

	*value = be16_to_cpu((u16) val);
	return 0;
}

static int ad5593r_reg_write(struct ad5592r_state *st, u8 reg, u16 value)
{
	struct i2c_client *i2c = container_of(st->dev, struct i2c_client, dev);

	value = cpu_to_be16(value);
	return i2c_smbus_write_word_data(i2c, MODE_CONF | reg, value);
}

static int ad5593r_reg_read(struct ad5592r_state *st, u8 reg, u16 *value)
{
	struct i2c_client *i2c = container_of(st->dev, struct i2c_client, dev);
	s32 val = i2c_smbus_read_word_data(i2c, MODE_REG_READBACK | reg);
	if (val < 0)
		return (int) val;

	*value = be16_to_cpu((u16) val);
	return 0;
}

static const struct ad5592r_rw_ops ad5593r_rw_ops = {
	.dac_write = ad5593r_dac_write,
	.adc_read = ad5593r_adc_read,
	.reg_write = ad5593r_reg_write,
	.reg_read = ad5593r_reg_read,
};

static int ad5593r_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	return ad5592r_probe(&i2c->dev, id->driver_data,
			id->name, &ad5593r_rw_ops);
}

static int ad5593r_i2c_remove(struct i2c_client *i2c)
{
	return ad5592r_remove(&i2c->dev);
}

static const struct i2c_device_id ad5593r_i2c_ids[] = {
	{"ad5593r", ID_AD5593R},
	{}
};
MODULE_DEVICE_TABLE(i2c, ad5593r_i2c_ids);

static struct i2c_driver ad5593r_driver = {
	.driver = {
		.name = "ad5593r",
		.owner = THIS_MODULE,
	},
	.probe = ad5593r_i2c_probe,
	.remove = ad5593r_i2c_remove,
	.id_table = ad5593r_i2c_ids,
};
module_i2c_driver(ad5593r_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
