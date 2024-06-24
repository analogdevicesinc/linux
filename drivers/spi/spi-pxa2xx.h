/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2005 Stephen Street / StreetFire Sound Labs
 * Copyright (C) 2013, 2021 Intel Corporation
 */

#ifndef SPI_PXA2XX_H
#define SPI_PXA2XX_H

#include <linux/dmaengine.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/sizes.h>

#include <linux/pxa2xx_ssp.h>

struct gpio_desc;

/*
 * The platform data for SSP controller devices
 * (resides in device.platform_data).
 */
struct pxa2xx_spi_controller {
	u8 num_chipselect;
	u8 enable_dma;
	u8 dma_burst_size;
	bool is_target;

	/* DMA engine specific config */
	dma_filter_fn dma_filter;
	void *tx_param;
	void *rx_param;

	/* For non-PXA arches */
	struct ssp_device ssp;
};

struct spi_controller;
struct spi_device;
struct spi_transfer;

struct driver_data {
	/* SSP Info */
	struct ssp_device *ssp;

	/* SPI framework hookup */
	enum pxa_ssp_type ssp_type;
	struct spi_controller *controller;

	/* PXA hookup */
	struct pxa2xx_spi_controller *controller_info;

	/* SSP masks*/
	u32 dma_cr1;
	u32 int_cr1;
	u32 clear_sr;
	u32 mask_sr;

	/* DMA engine support */
	atomic_t dma_running;

	/* Current transfer state info */
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;
	u8 n_bytes;
	int (*write)(struct driver_data *drv_data);
	int (*read)(struct driver_data *drv_data);
	irqreturn_t (*transfer_handler)(struct driver_data *drv_data);

	void __iomem *lpss_base;

	/* Optional slave FIFO ready signal */
	struct gpio_desc *gpiod_ready;
};

static inline u32 pxa2xx_spi_read(const struct driver_data *drv_data, u32 reg)
{
	return pxa_ssp_read_reg(drv_data->ssp, reg);
}

static inline void pxa2xx_spi_write(const struct driver_data *drv_data, u32 reg, u32 val)
{
	pxa_ssp_write_reg(drv_data->ssp, reg, val);
}

#define DMA_ALIGNMENT		8

static inline int pxa25x_ssp_comp(const struct driver_data *drv_data)
{
	switch (drv_data->ssp_type) {
	case PXA25x_SSP:
	case CE4100_SSP:
	case QUARK_X1000_SSP:
		return 1;
	default:
		return 0;
	}
}

static inline void clear_SSCR1_bits(const struct driver_data *drv_data, u32 bits)
{
	pxa2xx_spi_write(drv_data, SSCR1, pxa2xx_spi_read(drv_data, SSCR1) & ~bits);
}

static inline u32 read_SSSR_bits(const struct driver_data *drv_data, u32 bits)
{
	return pxa2xx_spi_read(drv_data, SSSR) & bits;
}

static inline void write_SSSR_CS(const struct driver_data *drv_data, u32 val)
{
	if (drv_data->ssp_type == CE4100_SSP ||
	    drv_data->ssp_type == QUARK_X1000_SSP)
		val |= read_SSSR_bits(drv_data, SSSR_ALT_FRM_MASK);

	pxa2xx_spi_write(drv_data, SSSR, val);
}

extern int pxa2xx_spi_flush(struct driver_data *drv_data);

#define MAX_DMA_LEN		SZ_64K
#define DEFAULT_DMA_CR1		(SSCR1_TSRE | SSCR1_RSRE | SSCR1_TRAIL)

extern irqreturn_t pxa2xx_spi_dma_transfer(struct driver_data *drv_data);
extern int pxa2xx_spi_dma_prepare(struct driver_data *drv_data,
				  struct spi_transfer *xfer);
extern void pxa2xx_spi_dma_start(struct driver_data *drv_data);
extern void pxa2xx_spi_dma_stop(struct driver_data *drv_data);
extern int pxa2xx_spi_dma_setup(struct driver_data *drv_data);
extern void pxa2xx_spi_dma_release(struct driver_data *drv_data);

#endif /* SPI_PXA2XX_H */
