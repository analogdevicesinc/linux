/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is part of AD5686 DAC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#ifndef __DRIVERS_IIO_DAC_AD5686_H__
#define __DRIVERS_IIO_DAC_AD5686_H__

#include <linux/bits.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <linux/iio/iio.h>

#define AD5310_CMD(x)				((x) << 12)

#define AD5683_DATA(x)				((x) << 4)

#define AD5686_ADDR(x)				((x) << 16)
#define AD5686_CMD(x)				((x) << 20)

#define AD5686_ADDR_DAC(chan)			(0x1 << (chan))
#define AD5686_ADDR_ALL_DAC			0xF
#define AD5686_MAX_CHANNELS			16

#define AD5686_CMD_NOOP				0x0
#define AD5686_CMD_WRITE_INPUT_N		0x1
#define AD5686_CMD_UPDATE_DAC_N			0x2
#define AD5686_CMD_WRITE_INPUT_N_UPDATE_N	0x3
#define AD5686_CMD_POWERDOWN_DAC		0x4
#define AD5686_CMD_LDAC_MASK			0x5
#define AD5686_CMD_RESET			0x6
#define AD5686_CMD_INTERNAL_REFER_SETUP		0x7
#define AD5686_CMD_DAISY_CHAIN_ENABLE		0x8
#define AD5686_CMD_READBACK_ENABLE		0x9

#define AD5686_LDAC_PWRDN_NONE			0x0
#define AD5686_LDAC_PWRDN_1K			0x1
#define AD5686_LDAC_PWRDN_100K			0x2
#define AD5686_LDAC_PWRDN_3STATE		0x3

#define AD5686_CMD_CONTROL_REG			0x4
#define AD5686_CMD_READBACK_ENABLE_V2		0x5

#define AD5310_GAIN_BIT_MSK			BIT(7)
#define AD5310_REF_BIT_MSK			BIT(8)
#define AD5310_PD_MSK				GENMASK(10, 9)
#define AD5683_GAIN_BIT_MSK			BIT(11)
#define AD5683_REF_BIT_MSK			BIT(12)
#define AD5683_PD_MSK				GENMASK(14, 13)

enum ad5686_regmap_type {
	AD5310_REGMAP,
	AD5683_REGMAP,
	AD5686_REGMAP,
};

struct ad5686_state;

/**
 * ad5686_bus_ops - bus specific read/write operations
 * @read: read a register value at the given address
 * @write: write a command, address and value to the device
 * @sync: ensure the completion of the write operation (optional)
 */
struct ad5686_bus_ops {
	int (*read)(struct ad5686_state *st, u8 addr);
	int (*write)(struct ad5686_state *st, u8 cmd, u8 addr, u16 val);
	int (*sync)(struct ad5686_state *st);
};

/**
 * struct ad5686_chip_info - chip specific information
 * @int_vref_mv:	AD5620/40/60: the internal reference voltage
 * @num_channels:	number of channels
 * @channel:		channel specification
 * @regmap_type:	register map layout variant
 */

struct ad5686_chip_info {
	u16				int_vref_mv;
	unsigned int			num_channels;
	const struct iio_chan_spec	*channels;
	enum ad5686_regmap_type		regmap_type;
};

/* single-channel instances */
extern const struct ad5686_chip_info ad5310r_chip_info;
extern const struct ad5686_chip_info ad5311r_chip_info;
extern const struct ad5686_chip_info ad5681r_chip_info;
extern const struct ad5686_chip_info ad5682r_chip_info;
extern const struct ad5686_chip_info ad5683_chip_info;
extern const struct ad5686_chip_info ad5683r_chip_info;

/* dual-channel instances */
extern const struct ad5686_chip_info ad5337r_chip_info;
extern const struct ad5686_chip_info ad5338r_chip_info;
extern const struct ad5686_chip_info ad5687_chip_info;
extern const struct ad5686_chip_info ad5687r_chip_info;
extern const struct ad5686_chip_info ad5689_chip_info;
extern const struct ad5686_chip_info ad5689r_chip_info;

/* quad-channel instances */
extern const struct ad5686_chip_info ad5317r_chip_info;
extern const struct ad5686_chip_info ad5684_chip_info;
extern const struct ad5686_chip_info ad5684r_chip_info;
extern const struct ad5686_chip_info ad5685r_chip_info;
extern const struct ad5686_chip_info ad5686_chip_info;
extern const struct ad5686_chip_info ad5686r_chip_info;

/* 8-channel instances */
extern const struct ad5686_chip_info ad5672r_chip_info;
extern const struct ad5686_chip_info ad5676_chip_info;
extern const struct ad5686_chip_info ad5676r_chip_info;

/* 16-channel instances */
extern const struct ad5686_chip_info ad5674_chip_info;
extern const struct ad5686_chip_info ad5674r_chip_info;
extern const struct ad5686_chip_info ad5679_chip_info;
extern const struct ad5686_chip_info ad5679r_chip_info;

/**
 * struct ad5686_state - driver instance specific data
 * @dev:		device instance
 * @chip_info:		chip model specific constants, available modes etc
 * @ops:		bus specific operations
 * @ldac_gpio:		LDAC pin GPIO descriptor
 * @gain_gpio:		GAIN pin GPIO descriptor
 * @vref_mv:		actual reference voltage used
 * @pwr_down_mask:	power down mask
 * @pwr_down_mode:	current power down mode
 * @scale_avail:	pre-calculated available scale values
 * @double_scale:	flag to indicate the gain multiplier is applied
 * @use_internal_vref:	set to true if the internal reference voltage is used
 * @lock:		lock to protect the data buffer during regmap ops
 * @bus_data:		bus specific data
 * @data:		transfer buffers
 */
struct ad5686_state {
	struct device			*dev;
	const struct ad5686_chip_info	*chip_info;
	const struct ad5686_bus_ops	*ops;
	struct gpio_desc		*ldac_gpio;
	struct gpio_desc		*gain_gpio;
	unsigned short			vref_mv;
	unsigned int			pwr_down_mask;
	unsigned int			pwr_down_mode;
	int				scale_avail[4];
	bool				double_scale;
	bool				use_internal_vref;
	struct mutex			lock;
	void				*bus_data;

	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */

	union {
		__be32 d32;
		__be16 d16;
		u8 d8[4];
	} data[AD5686_MAX_CHANNELS] __aligned(IIO_DMA_MINALIGN);
};


int ad5686_probe(struct device *dev,
		 const struct ad5686_chip_info *chip_info,
		 const char *name, const struct ad5686_bus_ops *ops,
		 void *bus_data);

static inline int ad5686_write(struct ad5686_state *st, u8 cmd, u8 addr, u16 val)
{
	int ret;

	ret = st->ops->write(st, cmd, addr, val);
	if (ret)
		return ret;

	return st->ops->sync ? st->ops->sync(st) : 0;
}

static inline int ad5686_read(struct ad5686_state *st, u8 addr)
{
	return st->ops->read(st, addr);
}

#endif /* __DRIVERS_IIO_DAC_AD5686_H__ */
