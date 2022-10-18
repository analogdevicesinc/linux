/*
 * SPI-Engine SPI controller driver
 * Copyright 2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef _INCLUDE_LINUX_SPI_SPI_ENGINE_H_
#define _INCLUDE_LINUX_SPI_SPI_ENGINE_H_

struct spi_engine_transfer {
	struct spi_transfer xfer;
	bool has_container;
	bool one_shot;
	bool ddr;
	bool stream;
};

enum {
	SPI_ENGINE_OFFLOAD_SRC0,
	SPI_ENGINE_OFFLOAD_SRC1,
};

/**
 * spi_engine_message_init_with_transfers - Initialize spi_message and append
 * spi_engine transfers
 * @m: spi_message to be initialized
 * @exfers: An array of spi_engine transfers
 * @num_xfers: Number of items in the xfer array
 *
 * This function initializes the given spi_message and adds each spi_transfer in
 * the given array to the message.
 */
static inline void
spi_engine_message_init_with_transfers(struct spi_message *m,
struct spi_engine_transfer *exfers, unsigned int num_xfers)
{
	unsigned int i;

	spi_message_init(m);
	for (i = 0; i < num_xfers; ++i)
		spi_message_add_tail(&exfers[i].xfer, m);
}

#ifdef CONFIG_SPI_AXI_SPI_ENGINE

bool spi_engine_offload_supported(struct spi_device *spi);
void spi_engine_offload_enable(struct spi_device *spi, bool enable);
int spi_engine_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg);
void spi_engine_offload_source_set(struct spi_device *spi, unsigned int val);
unsigned int spi_engine_offload_source_get(struct spi_device *spi);

#else

static inline bool spi_engine_offload_supported(struct spi_device *spi)
{
	return false;
}

static inline void spi_engine_offload_enable(struct spi_device *spi, bool enable)
{
}

static inline int spi_engine_offload_load_msg(struct spi_device *spi,
	struct spi_message *msg)
{
	return -ENODEV;
}

#endif

#endif
