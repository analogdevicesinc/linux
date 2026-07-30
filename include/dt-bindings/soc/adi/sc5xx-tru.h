/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Trigger Routing Unit (TRU) master (generator) and slave (receiver) IDs
 * for the ADI ADSP-SC84x.
 *
 * IDs verified against the ADSP-SC84x HRM trigger lists (TRU chapter
 * "Trigger List Masters", SPI chapter Table "SPI Trigger List Receivers",
 * TIMER chapter Table "TIMER Trigger List Generators"). Trigger IDs are
 * assigned per die and differ between family members (including between
 * the SC59x variants), so these values must not be reused for other SoCs.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#ifndef _DT_BINDINGS_SOC_ADI_SC5XX_TRU_H
#define _DT_BINDINGS_SOC_ADI_SC5XX_TRU_H

/* Master ID 0 is reserved as the null source: routing it disconnects. */
#define ADI_TRU_MST_NULL		0

/* Software-driven trigger masters SOFT0..SOFT5 */
#define ADI_TRU_MST_SOFT(n)		(167 + (n))

/* GP timer trigger generators TIMER0_TMR00_GEN..TMR15_GEN */
#define ADI_TRU_MST_TIMER0_TMR(n)	(173 + (n))

/* SPI TX/RX DMA channel trigger receivers */
#define ADI_TRU_RCV_SPI0_TXDMA		149
#define ADI_TRU_RCV_SPI0_RXDMA		150
#define ADI_TRU_RCV_SPI1_TXDMA		151
#define ADI_TRU_RCV_SPI1_RXDMA		152
#define ADI_TRU_RCV_SPI5_TXDMA		153
#define ADI_TRU_RCV_SPI5_RXDMA		154
#define ADI_TRU_RCV_SPI2_TXDMA		155
#define ADI_TRU_RCV_SPI2_RXDMA		156

#endif /* _DT_BINDINGS_SOC_ADI_SC5XX_TRU_H */
