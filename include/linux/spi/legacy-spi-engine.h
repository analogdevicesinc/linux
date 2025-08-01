/*
 * SPI-Engine SPI controller driver
 * Copyright 2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _INCLUDE_LINUX_SPI_SPI_ENGINE_H_
#define _INCLUDE_LINUX_SPI_SPI_ENGINE_H_

#if IS_REACHABLE(CONFIG_SPI_AXI_SPI_ENGINE)

bool legacy_spi_engine_offload_supported(struct spi_device *spi);
void legacy_spi_engine_offload_enable(struct spi_device *spi, bool enable);
int legacy_spi_engine_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg);

#else

static inline bool legacy_spi_engine_offload_supported(struct spi_device *spi)
{
	return false;
}

static inline void legacy_spi_engine_offload_enable(struct spi_device *spi, bool enable)
{
}

static inline int legacy_spi_engine_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg)
{
	return -ENODEV;
}

#endif

#endif
