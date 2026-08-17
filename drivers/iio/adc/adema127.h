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

struct dma_chan;
struct gpio_desc;
struct regmap;
struct spi_offload;
struct spi_offload_trigger;

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

	/* SPI Offload framework handles (optional) */
	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_message offload_msg;
	struct spi_transfer offload_xfer;
	bool offload_streaming;
	/*
	 * TX side of the offload path. The command word ("READ STATUS2 long
	 * frame") never changes, so we allocate one coherent buffer sized to
	 * the chip's long frame, fill it once at setup, and hand its bus
	 * address to a cyclic DMA descriptor that replays it forever. The
	 * DMA_CFG.TWAIT bit (enabled via peripheral_config) gates each period
	 * on a TRU trigger derived from DREADY.
	 */
	struct dma_chan *offload_tx_chan;
	void *offload_tx_buf;
	dma_addr_t offload_tx_dma;
	struct dma_async_tx_descriptor *offload_tx_desc;

	/*
	 * RX side of the offload path: a coherent ping-pong buffer of
	 * ADEMA_OFFLOAD_RX_BANKS × watermark long-frames
	 * behind a hardware descriptor-list DMA ring (one descriptor per
	 * long-frame, TRU-trigger gated, IRQ only on each bank's last
	 * descriptor). The completion callback (adema_offload_rx_bank_done)
	 * pushes the finished bank frame-by-frame into the IIO kfifo.
	 *
	 * This bypasses industrialio-buffer-dmaengine's block-mode slave-sg
	 * path (which fails to complete blocks under our SPI3 TWCR-gated
	 * one-frame-per-trigger operating mode -- see followups.md §6).
	 */
	struct dma_chan *offload_rx_chan;
	void *offload_rx_buf;
	dma_addr_t offload_rx_dma;
	size_t offload_rx_total;
	struct dma_async_tx_descriptor *offload_rx_desc;
	unsigned int offload_rx_def_bank_frames;
	unsigned int offload_rx_bank_frames;
	unsigned int offload_rx_bank_idx;
	struct iio_dev *offload_indio_dev;

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

/* Command-word builder shared with the offload TX path. */
void adema_build_cmd(u8 buf[ADEMA_CMD_LEN], bool read, bool long_frame,
		     u16 addr, u8 data);

/* Channel ext_info (gain/filter/DSP-RAM controls), reused by offload mode. */
extern const struct iio_chan_spec_ext_info adema_ext_info[];

#if IS_ENABLED(CONFIG_ADEMA127_SPI_OFFLOAD)
/*
 * Probe-time hook: claim the controller's SPI Offload services and switch
 * the device to hardware-triggered streaming. Returns -ENODEV when the
 * controller does not provide offload for this device, in which case the
 * caller falls back to the software triggered-buffer path.
 */
int adema_setup_offload(struct iio_dev *indio_dev, struct adema_state *st);
#else
static inline int adema_setup_offload(struct iio_dev *indio_dev,
				      struct adema_state *st)
{
	return -ENODEV;
}
#endif

#endif /* _ADEMA127_H_ */
