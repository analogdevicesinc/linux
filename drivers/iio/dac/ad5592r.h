/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRIVERS_IIO_DAC_AD5592R_H__
#define __DRIVERS_IIO_DAC_AD5592R_H__

#include <linux/types.h>

struct device;
struct ad5592r_state;

enum ad5592r_type {
	ID_AD5592R,
};

enum ad5592r_registers {
	AD5592R_REG_NOOP		= 0x0,
	AD5592R_REG_DAC_READBACK	= 0x1,
	AD5592R_REG_ADC_SEQ		= 0x2,
	AD5592R_REG_CTRL		= 0x3,
	AD5592R_REG_ADC_EN		= 0x4,
	AD5592R_REG_DAC_EN		= 0x5,
	AD5592R_REG_PULLDOWN		= 0x6,
	AD5592R_REG_LDAC		= 0x7,
	AD5592R_REG_GPIO_OUT_EN	= 0x8,
	AD5592R_REG_GPIO_SET		= 0x9,
	AD5592R_REG_GPIO_IN_EN		= 0xA,
	AD5592R_REG_PD			= 0xB,
	AD5592R_REG_OPEN_DRAIN		= 0xC,
	AD5592R_REG_TRISTATE		= 0xD,
	AD5592R_REG_RESET		= 0xF,
};

enum ad5592r_channel_mode {
	AD5592R_MODE_UNUSED,
	AD5592R_MODE_DAC,
	AD5592R_MODE_ADC,
	AD5592R_MODE_GPIO_OUT,
	AD5592R_MODE_GPIO_IN,
	AD5592R_MODE_GPIO_OPEN_DRAIN,
	AD5592R_MODE_GPIO_TRISTATE,
};

struct ad5592r_chip_info {
	unsigned num_channels;
};

struct ad5592r_rw_ops {
	int (*dac_write)(struct ad5592r_state *st, unsigned chan, u16 value);
	int (*adc_read)(struct ad5592r_state *st, unsigned chan, u16 *value);
	int (*reg_write)(struct ad5592r_state *st, u8 reg, u16 value);
	int (*reg_read)(struct ad5592r_state *st, u8 reg, u16 *value);
};

struct ad5592r_state {
	struct device *dev;
	const struct ad5592r_chip_info *chip_info;
	const struct ad5592r_rw_ops *ops;
	enum ad5592r_channel_mode channel_info[8];
};

int ad5592r_probe(struct device *dev, enum ad5592r_type type,
		const char *name, const struct ad5592r_rw_ops *rw_ops);
int ad5592r_remove(struct device *dev);

#endif /* __DRIVERS_IIO_DAC_AD5592R_H__ */
