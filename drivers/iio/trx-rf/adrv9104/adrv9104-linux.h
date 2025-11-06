/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9104 Linux Kernel Interface Header
 *
 * Copyright 2025 Analog Devices Inc.
 *
 */
#ifndef ADRV9104_LINUX_H_
#define ADRV9104_LINUX_H_

struct spi_device;
struct gpio_desc;

struct adrv9104_hal_cfg {
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
};

#endif
