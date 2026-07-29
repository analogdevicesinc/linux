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
 * @trigger_granule: bytes transferred per TRU trigger (descriptor-list mode)
 *
 * Pass a pointer to a stack-allocated instance of this struct through
 * ``dma_slave_config.peripheral_config`` (with ``peripheral_size =
 * sizeof(struct adi_dma_peripheral_config)``) to enable ADI-specific DMA
 * behaviour that isn't expressible through the generic dmaengine API.
 *
 * When @trigger_granule is non-zero together with
 * %ADI_DMA_PC_WAIT_FOR_TRIGGER, a subsequent
 * :c:func:`dmaengine_prep_dma_cyclic` on the channel is implemented as a
 * hardware descriptor-list ring instead of an AUTOBUFFER: one descriptor
 * per @trigger_granule bytes, each gated on a TRU trigger, with the
 * completion interrupt raised only by the last descriptor of each
 * ``period_len`` (so the callback rate is trigger-rate ×
 * granule ÷ period_len rather than one interrupt per granule). The
 * granule must divide ``period_len`` and ``period_len`` must divide the
 * total buffer length. In descriptor-list mode even the FIRST work unit
 * waits for a trigger (unlike AUTOBUFFER TWAIT, which lets the first
 * unit fire immediately).
 */
struct adi_dma_peripheral_config {
	unsigned int flags;
	unsigned int trigger_granule;
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

/**
 * ADI_DMA_PC_DISCARD_FIRST - Discard the first trigger-gated work unit
 *
 * Only meaningful together with %ADI_DMA_PC_WAIT_FOR_TRIGGER and a
 * non-zero @trigger_granule (descriptor-list ring). The ring is built
 * with one extra head descriptor that steers the FIRST granule into the
 * ring's final slot (which the loop rewrites with real data long before
 * that slot's period completes) and raises no interrupt. Use when the
 * peripheral's protocol makes the first response spurious — e.g. the
 * ADEMA127 pipelines long-frame reads so transaction N returns the data
 * requested in transaction N-1, making the first RX frame garbage.
 */
#define ADI_DMA_PC_DISCARD_FIRST	BIT(1)

#endif /* __LINUX_DMA_ADI_DMA_H__ */
