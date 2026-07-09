/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Public header for consumers of the ADI SC5xx DMA controller.
 *
 * (C) Copyright 2026 - Analog Devices, Inc.
 */

#ifndef __LINUX_DMA_ADI_DMA_H__
#define __LINUX_DMA_ADI_DMA_H__

#include <linux/bits.h>
#include <linux/types.h>

/**
 * struct adi_dma_peripheral_config - Private slave-config extension
 * @flags: bitmask of ADI_DMA_PC_* flags
 *
 * Pass a pointer to a stack-allocated instance of this struct through
 * ``dma_slave_config.peripheral_config`` (with ``peripheral_size =
 * sizeof(struct adi_dma_peripheral_config)``) to enable ADI-specific DMA
 * behaviour that isn't expressible through the generic dmaengine API.
 */
struct adi_dma_peripheral_config {
	unsigned int flags;
};

/**
 * ADI_DMA_PC_WAIT_FOR_TRIGGER - Gate every DMA work unit on a TRU trigger
 *
 * When set, the DMA controller programs ``DMA_CFG.TWAIT = 1`` on the
 * channel, which makes the DMA engine wait for a peripheral-trigger
 * pulse from the TRU (Trigger Routing Unit) before consuming each
 * subsequent work unit.
 *
 * Only meaningful in AUTOBUFFER, STOP, or DEMAND flow modes (i.e. the
 * modes that :c:func:`dmaengine_prep_dma_cyclic` currently produces on
 * this controller). Per the SC5xx HRM the FIRST work unit still fires
 * without waiting for a trigger — the consumer is responsible for
 * discarding the resulting first sample if that matters for its
 * protocol.
 *
 * The consumer is responsible for programming the TRU RSR entry that
 * connects a trigger generator (e.g. a GP timer configured in EXTCLK
 * mode reading a data-ready GPIO) to the SPI TX/RX DMA receiver IDs.
 * See ``drivers/spi/spi-offload-trigger-adi-sc-tru.c`` for the pattern.
 */
#define ADI_DMA_PC_WAIT_FOR_TRIGGER	BIT(0)

#endif /* __LINUX_DMA_ADI_DMA_H__ */
