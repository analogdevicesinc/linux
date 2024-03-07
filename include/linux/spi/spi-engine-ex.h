/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * SPI-Engine SPI controller driver
 * Copyright 2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */

#ifndef _INCLUDE_LINUX_SPI_SPI_ENGINE_EX_H_
#define _INCLUDE_LINUX_SPI_SPI_ENGINE_EX_H_

struct spi_device;
struct spi_message;

#ifdef CONFIG_SPI_AXI_SPI_ENGINE

bool spi_engine_ex_offload_supported(struct spi_device *spi);
void spi_engine_ex_offload_enable(struct spi_device *spi, bool enable);
int spi_engine_ex_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg);

#else

static inline bool spi_engine_ex_offload_supported(struct spi_device *spi)
{
	return false;
}

static inline void spi_engine_ex_offload_enable(struct spi_device *spi, bool enable)
{
}

static inline int spi_engine_ex_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg)
{
	return -ENODEV;
}

#endif

#endif
