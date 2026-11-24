/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * AD9508 1.65 GHz Clock Fanout Buffer with Output Dividers and Delay Adjust
 *
 * Copyright 2026 Analog Devices Inc.
 */

#ifndef __CLK_AD9508_H__
#define __CLK_AD9508_H__

#include <linux/types.h>

struct device;

/**
 * struct ad9508_bus_ops - register access over the control interface
 * @read: read @len bytes starting at address @reg. The byte at @reg is the
 *	  least significant byte of @val.
 * @write: write @len bytes starting at address @reg, with the same layout.
 */
struct ad9508_bus_ops {
	int (*read)(void *context, unsigned int reg, unsigned int len,
		    unsigned int *val);
	int (*write)(void *context, unsigned int reg, unsigned int len,
		     unsigned int val);
};

int ad9508_probe(struct device *dev, const struct ad9508_bus_ops *bus_ops,
		 void *bus_context, bool sdo_enable);

#endif /* __CLK_AD9508_H__ */
