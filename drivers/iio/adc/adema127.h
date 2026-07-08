/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADEMA124/ADEMA127 poly-phase energy metering ADC driver — shared
 * definitions between the core driver (adema127-core.c) and the optional
 * SPI Offload streaming backend (adema127-spi-offload.c).
 *
 * Copyright 2026 Analog Devices Inc.
 */
#ifndef _ADEMA127_H_
#define _ADEMA127_H_

#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

struct gpio_desc;
struct regmap;

/* --- Frame lengths ------------------------------------------------------- */
#define ADEMA_CMD_LEN			4
#define ADEMA_SHORT_FRAME_LEN		6
#define ADEMA127_LONG_FRAME_LEN		32
#define ADEMA124_LONG_FRAME_LEN		20

#define ADEMA_MAX_CHANNELS		7

#define ADEMA_REG_STATUS2		0x022
/* Long-frame reads always request STATUS2 as the command address. */
#define ADEMA_STREAM_CMD_ADDR		ADEMA_REG_STATUS2

/* --- Per-variant description --------------------------------------------- */

struct adema_chip_info {
	const char *name;
	u8 product_id;
	u8 num_channels;
	u8 long_frame_len;
	const struct iio_chan_spec *channels;
	u8 num_iio_channels;
	const unsigned long *available_scan_masks;
};

struct adema_state {
	struct spi_device *spi;
	const struct adema_chip_info *chip_info;
	struct regmap *regmap;

	struct mutex lock;		/* serialises reg + long-frame paths */
	/*
	 * Serialises the DSP-RAM window (ACCESS_EXTENDED_MMAP request ->
	 * accesses -> release). Distinct from `lock`, which the regmap bus
	 * ops take per short-frame INSIDE the window; ordering is always
	 * dsp_lock -> lock, never the reverse.
	 */
	struct mutex dsp_lock;

	unsigned int sampling_freq;
	unsigned int xtal_hz;
	/*
	 * Supported sample rates at the ACTUAL XTAL rate, ascending —
	 * adema_datarates[] scaled by xtal_hz/16.384 MHz at probe. Both the
	 * available-list and the rate lookup use this table so sysfs never
	 * advertises a rate the DATARATE lookup would reject.
	 */
	int sampling_freqs[9];
	unsigned int scan_push_bytes;	/* payload + timestamp size */

	struct gpio_desc *reset_gpio;
	int dready_irq;

	/* per-channel hardware-input gain (1 or 2), for scale computation */
	u8 adc_input_gain[ADEMA_MAX_CHANNELS];

	/*
	 * DMA-touched buffers live at the tail of the struct, each on its own
	 * cacheline, so that non-coherent DMA maintenance doesn't clobber the
	 * rest of the state or the other buffer.
	 */
	u8 reg_tx[ADEMA_SHORT_FRAME_LEN] __aligned(IIO_DMA_MINALIGN);
	u8 reg_rx[ADEMA_SHORT_FRAME_LEN] __aligned(IIO_DMA_MINALIGN);
	u8 long_tx[ADEMA127_LONG_FRAME_LEN] __aligned(IIO_DMA_MINALIGN);
	u8 long_rx[ADEMA127_LONG_FRAME_LEN] __aligned(IIO_DMA_MINALIGN);

	/*
	 * Unpacked scan buffer: chip-sized channel payload followed by an
	 * 8-byte timestamp. Total pushed size is st->scan_push_bytes.
	 * Sized for the largest supported chip (ADEMA127: 7×4 payload + pad + 8).
	 */
	u8 scan[ALIGN(ADEMA_MAX_CHANNELS * sizeof(s32), sizeof(s64)) + sizeof(s64)]
		__aligned(IIO_DMA_MINALIGN);
};

#endif /* _ADEMA127_H_ */
