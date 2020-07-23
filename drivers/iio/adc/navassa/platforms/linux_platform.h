/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#ifndef LINUX_PLATFORM_H_
#define LINUX_PLATFORM_H_

struct spi_device;
struct gpio_desc;

struct adrv9002_hal_cfg {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
};

#endif
