// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADEMA124/ADEMA127 SPI Offload streaming backend.
 *
 * Hardware-triggered capture path: every DREADY pulse is routed (via the
 * controller's SPI Offload trigger provider) to a pair of trigger-gated
 * DMA descriptor rings that autonomously clock one long-frame command out
 * and one long-frame response in, with a completion interrupt only once
 * per bank of frames. The core driver (adema127-core.c) falls back to a
 * per-DREADY software triggered buffer when this backend is compiled out
 * or the controller offers no offload services.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dma/adi-dma.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/property.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#include "adema127.h"

/*
 * RX descriptor-ring geometry. The adi-dma controller runs a hardware
 * descriptor-list ring: one descriptor per long-frame (each gated on a
 * DREADY-derived TRU pulse), with the completion interrupt raised only
 * by the LAST descriptor of each bank. The bank size follows the IIO
 * buffer watermark set by userspace (clamped to the cap below), so the
 * interrupt rate is sample-rate / watermark and userspace owns the
 * latency-vs-IRQ-rate trade-off through the standard ABI; the callback
 * pushes the whole just-completed bank into the IIO kfifo while the
 * DMA fills the other bank (classic ping-pong).
 */
#define ADEMA_OFFLOAD_MAX_BANK_FRAMES	4096
#define ADEMA_OFFLOAD_DEF_BANK_FRAMES	128
#define ADEMA_OFFLOAD_RX_BANKS		2

/*
 * Offload-mode channel spec. The SPI Offload DMA hands the CPU raw long-frame
 * responses with the ADEMA's own framing intact:
 *
 *   ADEMA127 (32 B):  echo(1)  V0(3) STAT0(1) V1(3) STAT1(1)
 *                     V2(3) rsvd(1) V3(3) rsvd(1)
 *                     V4(3) rsvd(1) V5(3) rsvd(1) V6(3)
 *                     RDD1(1) RDD0(1) CRC(2)
 *   ADEMA124 (20 B):  echo(1)  V0(3) STAT0(1) V1(3) STAT1(1)
 *                     V2(3) rsvd(1) V3(3) RDD1(1) RDD0(1) CRC(2)
 *
 * Every phase sample is 1 byte of housekeeping (echo, STATUS0, STATUS1, or
 * rsvd) followed by the 3 waveform bytes transmitted WAV_LO, WAV_MD, WAV_HI
 * (little-endian, per the datasheet's Long Format Operation section). Each
 * phase therefore lives in a 4-byte slot whose little-endian 32-bit
 * representation is (value_24bit << 8) | hk, i.e. IIO scan_type sign='s',
 * realbits=24, storagebits=32, shift=8, endianness=IIO_LE -- the framework
 * shifts the housekeeping byte away and sign-extends bit 23 for the
 * consumer, so userspace reads clean s32 samples.
 *
 * The trailer slot (RDD1 RDD0 CRC_hi CRC_lo) is exposed as an extra
 * IIO_VOLTAGE channel with sign='u', realbits=32, storagebits=32, BE (the
 * trailer is not a sample, so it keeps the frame's byte order: RDDATA1 in
 * bits 31:24, RDDATA0 in bits 23:16, CRC-CCITT in bits 15:0). It's NOT a
 * real voltage -- the type is chosen so the framework's per-channel
 * bookkeeping accepts it uniformly with the phase channels.
 *
 * available_scan_masks below is "all channels or none" (GENMASK(N,0) plus 0)
 * because the DMA emits the whole 20/32-byte frame every DREADY -- there's
 * no way to selectively omit a channel without changing the on-wire frame,
 * and IIO's per-record bytesize must match the DMA transfer size exactly or
 * industrialio-buffer-dmaengine's read alignment goes wrong.
 */
/*
 * Offload phase channels carry the SAME control attributes as the
 * software-path channels (raw/scale/calib/phase/sampling_frequency plus
 * the adema_ext_info gain/filter controls) — attribute access uses the
 * regmap short-frame path and works whenever the buffer isn't streaming.
 * Only the scan_type differs: the DMA delivers the chip's native
 * little-endian long-frame layout instead of repacked CPU-endian s32s.
 */
#define ADEMA_OFFLOAD_PHASE_CHAN(_idx) {				\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.scan_index = (_idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |		\
			      BIT(IIO_CHAN_INFO_CALIBBIAS) |		\
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			      BIT(IIO_CHAN_INFO_PHASE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_all_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 24,						\
		.storagebits = 32,					\
		.shift = 8,						\
		.endianness = IIO_LE,					\
	},								\
	.ext_info = adema_ext_info,					\
}

#define ADEMA_OFFLOAD_TRAILER_CHAN(_idx) {				\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = (_idx),						\
	.scan_index = (_idx),						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_BE,					\
	},								\
}

static const struct iio_chan_spec adema127_offload_channels[] = {
	ADEMA_OFFLOAD_PHASE_CHAN(0),
	ADEMA_OFFLOAD_PHASE_CHAN(1),
	ADEMA_OFFLOAD_PHASE_CHAN(2),
	ADEMA_OFFLOAD_PHASE_CHAN(3),
	ADEMA_OFFLOAD_PHASE_CHAN(4),
	ADEMA_OFFLOAD_PHASE_CHAN(5),
	ADEMA_OFFLOAD_PHASE_CHAN(6),
	ADEMA_OFFLOAD_TRAILER_CHAN(7),
};

static const struct iio_chan_spec adema124_offload_channels[] = {
	ADEMA_OFFLOAD_PHASE_CHAN(0),
	ADEMA_OFFLOAD_PHASE_CHAN(1),
	ADEMA_OFFLOAD_PHASE_CHAN(2),
	ADEMA_OFFLOAD_PHASE_CHAN(3),
	ADEMA_OFFLOAD_TRAILER_CHAN(4),
};

static const unsigned long adema127_offload_scan_masks[] = {
	GENMASK(7, 0),  /* 7 phase channels + 1 trailer = 8 elements × 4 B = 32 B */
	0,
};

static const unsigned long adema124_offload_scan_masks[] = {
	GENMASK(4, 0),  /* 4 phase channels + 1 trailer = 5 elements × 4 B = 20 B */
	0,
};

static const struct spi_offload_config adema_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA |
			    SPI_OFFLOAD_CAP_TX_STREAM_DMA,
};

/*
 * Configure a handed-off SPI-offload DMA channel to wait for a TRU
 * trigger between work units. This is the ADI-specific hint that the
 * ADSP-SC5xx DMA driver honours by setting DMA_CFG.TWAIT on the
 * channel. See include/linux/dma/adi-dma.h.
 */
static int adema_offload_config_dma_twait(struct adema_state *st,
					  struct dma_chan *chan,
					  enum dma_transfer_direction dir)
{
	struct adi_dma_peripheral_config pc = {
		/*
		 * RX discards its first granule in hardware (an extra
		 * non-interrupting head descriptor): ADEMA long-frame reads
		 * are pipelined, so the first RX frame after enable answers
		 * a stale pre-stream command. Discarding in the ring keeps
		 * every completed bank a full bank of scans.
		 */
		.flags = ADI_DMA_PC_WAIT_FOR_TRIGGER |
			 (dir == DMA_DEV_TO_MEM ? ADI_DMA_PC_DISCARD_FIRST : 0),
		/*
		 * Both directions run a descriptor-list ring with one
		 * trigger-gated descriptor per long-frame; in list mode even
		 * the first fetch waits for a trigger, so no untriggered TX
		 * frame can leave RX permanently one frame stale.
		 * RX preps with DMA_PREP_INTERRUPT (IRQ on each bank's last
		 * descriptor), TX without (no TX IRQs at all).
		 */
		.trigger_granule = st->chip_info->long_frame_len,
	};
	struct dma_slave_config cfg = {
		.direction		= dir,
		.src_addr_width		= DMA_SLAVE_BUSWIDTH_1_BYTE,
		.dst_addr_width		= DMA_SLAVE_BUSWIDTH_1_BYTE,
		.src_maxburst		= 1,
		.dst_maxburst		= 1,
		.peripheral_config	= &pc,
		.peripheral_size	= sizeof(pc),
	};

	return dmaengine_slave_config(chan, &cfg);
}

/*
 * Fill the offload TX coherent buffer with the "READ STATUS2 long" command
 * word placed at the end of the frame (bytes n-4..n-1); the preceding bytes
 * stay at zero. Same layout as adema_build_long_stream_cmd() but writes
 * into the DMA-coherent scratch that feeds the TX DMA channel.
 */
static void adema_offload_fill_tx_buf(struct adema_state *st)
{
	unsigned int len = st->chip_info->long_frame_len;
	unsigned int cmd_off = len - ADEMA_CMD_LEN;

	memset(st->offload_tx_buf, 0, len);
	adema_build_cmd(st->offload_tx_buf + cmd_off, true, true,
			ADEMA_STREAM_CMD_ADDR, 0);
}

static int adema_offload_start_rx_stream(struct adema_state *st);
static void adema_offload_stop_rx_stream(struct adema_state *st);

static int adema_offload_start_tx_stream(struct adema_state *st)
{
	unsigned int len = st->chip_info->long_frame_len;
	dma_cookie_t cookie;

	adema_offload_fill_tx_buf(st);

	/*
	 * Single self-looping hardware descriptor: every DREADY-gated
	 * fetch re-sends the same one-frame command buffer.
	 */
	st->offload_tx_desc = dmaengine_prep_dma_cyclic(st->offload_tx_chan,
			st->offload_tx_dma, len, len,
			DMA_MEM_TO_DEV, 0);
	if (!st->offload_tx_desc)
		return -ENOMEM;

	cookie = dmaengine_submit(st->offload_tx_desc);
	if (dma_submit_error(cookie))
		return -EIO;

	dma_async_issue_pending(st->offload_tx_chan);
	return 0;
}

static int adema_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adema_state *st = iio_priv(indio_dev);

	/*
	 * The ADC is free-running (XTALIN + DATARATE set the cadence), so
	 * the trigger must follow DREADY rather than a periodic tick;
	 * DATA_READY carries no config parameters.
	 */
	struct spi_offload_trigger_config trig = {
		.type = SPI_OFFLOAD_TRIGGER_DATA_READY,
	};
	struct spi_transfer *xfer = &st->offload_xfer;
	int ret;

	memset(xfer, 0, sizeof(*xfer));
	xfer->len = st->chip_info->long_frame_len;
	xfer->bits_per_word = 8;
	/*
	 * Both TX and RX are streamed through the offload DMA channels — the
	 * SPI controller reads TX from the DMA-fed FIFO and writes RX into
	 * the DMA-drained FIFO, all without CPU involvement per sample.
	 */
	xfer->offload_flags = SPI_OFFLOAD_XFER_RX_STREAM |
			      SPI_OFFLOAD_XFER_TX_STREAM;

	spi_message_init_with_transfers(&st->offload_msg, xfer, 1);
	st->offload_msg.offload = st->offload;

	ret = spi_optimize_message(st->spi, &st->offload_msg);
	if (ret)
		return ret;

	/*
	 * Order matters: arm the RX ring first, then TX, then the TRU
	 * route that gates both — a TX frame clocked out before RX is
	 * armed would land in the RFIFO with no consumer.
	 */
	ret = adema_offload_start_rx_stream(st);
	if (ret)
		goto err_unoptimize;

	ret = adema_offload_start_tx_stream(st);
	if (ret)
		goto err_terminate_rx;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger, &trig);
	if (ret)
		goto err_terminate_tx;

	st->offload_streaming = true;
	return 0;

err_terminate_tx:
	dmaengine_terminate_sync(st->offload_tx_chan);
err_terminate_rx:
	adema_offload_stop_rx_stream(st);
err_unoptimize:
	spi_unoptimize_message(&st->offload_msg);
	return ret;
}

static int adema_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adema_state *st = iio_priv(indio_dev);

	if (st->offload_streaming) {
		/*
		 * Reverse order of postenable: cut the trigger first so no new
		 * DREADY-driven work unit can fire while we're tearing down,
		 * then stop TX (which stops SPI clocking with no consumer),
		 * then RX.
		 */
		spi_offload_trigger_disable(st->offload, st->offload_trigger);
		dmaengine_terminate_sync(st->offload_tx_chan);
		adema_offload_stop_rx_stream(st);
		spi_unoptimize_message(&st->offload_msg);
		st->offload_streaming = false;
	}
	return 0;
}

static const struct iio_buffer_setup_ops adema_offload_buffer_setup_ops = {
	.postenable  = adema_offload_buffer_postenable,
	.predisable  = adema_offload_buffer_predisable,
};

static void adema_offload_free_tx_buf(void *data)
{
	struct adema_state *st = data;

	dma_free_coherent(st->offload_tx_chan->device->dev,
			  st->chip_info->long_frame_len,
			  st->offload_tx_buf, st->offload_tx_dma);
}

/*
 * Tear down the RX stream: stop the descriptor-list ring, then release
 * the per-enable ping-pong buffer (its size follows the buffer
 * watermark, so it cannot be a probe-time allocation).
 */
static void adema_offload_stop_rx_stream(struct adema_state *st)
{
	dmaengine_terminate_sync(st->offload_rx_chan);
	if (st->offload_rx_buf) {
		dma_free_coherent(st->offload_rx_chan->device->dev,
				  st->offload_rx_total, st->offload_rx_buf,
				  st->offload_rx_dma);
		st->offload_rx_buf = NULL;
	}
}

/*
 * Bank-completion callback. Fires once per bank (= IIO watermark) of
 * frames — the DMA descriptor ring raises its interrupt only from the
 * last descriptor of each bank. Push the whole finished bank into the
 * IIO kfifo, one scan per long-frame, while the hardware fills the other
 * bank.
 *
 * The dmaengine API doesn't tell us which bank just completed; the ring
 * advances strictly in order and wraps, so a driver-side counter mirrors
 * the hardware position.
 */
static void adema_offload_rx_bank_done(void *arg)
{
	struct adema_state *st = arg;
	unsigned int len = st->chip_info->long_frame_len;
	const u8 *bank = (const u8 *)st->offload_rx_buf +
			 (size_t)st->offload_rx_bank_idx *
			 st->offload_rx_bank_frames * len;
	unsigned int i;

	/*
	 * All frames in a completed bank are valid: the spurious first
	 * (pipeline) frame is discarded in hardware by the RX ring's head
	 * descriptor (ADI_DMA_PC_DISCARD_FIRST), so every bank is a full
	 * watermark's worth of scans and the IIO poller wakes on the very
	 * first bank interrupt.
	 */
	for (i = 0; i < st->offload_rx_bank_frames; i++)
		iio_push_to_buffers(st->offload_indio_dev, bank + (size_t)i * len);

	st->offload_rx_bank_idx++;
	if (st->offload_rx_bank_idx >= ADEMA_OFFLOAD_RX_BANKS)
		st->offload_rx_bank_idx = 0;
}

static int adema_offload_start_rx_stream(struct adema_state *st)
{
	unsigned int len = st->chip_info->long_frame_len;
	dma_cookie_t cookie;
	int ret;

	st->offload_rx_bank_idx = 0;

	/*
	 * One bank = one IIO watermark of frames: userspace's standard
	 * buffer0/watermark setting (frozen by the core for the duration of
	 * the enable) decides how many trigger-gated frames accumulate per
	 * completion interrupt, i.e. IRQ rate = sample rate / watermark.
	 * A watermark left at the kernel default of 1 means userspace
	 * expressed no preference — use the board default from
	 * "adi,offload-frames-per-interrupt" (128 when absent) instead of
	 * degenerating to one interrupt per frame. Clamped so a
	 * pathological watermark cannot exhaust CMA.
	 */
	st->offload_rx_bank_frames = st->offload_indio_dev->buffer->watermark;
	if (st->offload_rx_bank_frames <= 1)
		st->offload_rx_bank_frames = st->offload_rx_def_bank_frames;
	st->offload_rx_bank_frames = clamp_t(unsigned int,
			st->offload_rx_bank_frames, 1,
			ADEMA_OFFLOAD_MAX_BANK_FRAMES);
	st->offload_rx_total = (size_t)len * st->offload_rx_bank_frames *
			       ADEMA_OFFLOAD_RX_BANKS;

	st->offload_rx_buf = dma_alloc_coherent(st->offload_rx_chan->device->dev,
			st->offload_rx_total, &st->offload_rx_dma, GFP_KERNEL);
	if (!st->offload_rx_buf)
		return -ENOMEM;

	/*
	 * period_len = one bank. The adi-dma controller sees trigger_granule
	 * = long_frame_len on this channel and implements the request as a
	 * hardware descriptor-list ring: one trigger-gated descriptor per
	 * frame, DI_EN_X only on the last descriptor of each period
	 * (DMA_PREP_INTERRUPT), so the callback below runs at
	 * trigger-rate / watermark.
	 */
	st->offload_rx_desc = dmaengine_prep_dma_cyclic(st->offload_rx_chan,
			st->offload_rx_dma,
			(size_t)len * st->offload_rx_bank_frames *
				ADEMA_OFFLOAD_RX_BANKS,
			(size_t)len * st->offload_rx_bank_frames,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!st->offload_rx_desc) {
		ret = -ENOMEM;
		goto err_free_ring;
	}

	st->offload_rx_desc->callback = adema_offload_rx_bank_done;
	st->offload_rx_desc->callback_param = st;

	cookie = dmaengine_submit(st->offload_rx_desc);
	if (dma_submit_error(cookie)) {
		ret = -EIO;
		goto err_free_ring;
	}

	dma_async_issue_pending(st->offload_rx_chan);
	return 0;

err_free_ring:
	dma_free_coherent(st->offload_rx_chan->device->dev,
			  st->offload_rx_total, st->offload_rx_buf,
			  st->offload_rx_dma);
	st->offload_rx_buf = NULL;
	return ret;
}

int adema_setup_offload(struct iio_dev *indio_dev, struct adema_state *st)
{
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma, *tx_dma;
	int ret;

	st->offload = devm_spi_offload_get(dev, st->spi, &adema_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);
	if (ret == -ENODEV) {
		st->offload = NULL;
		return -ENODEV;
	} else if (ret) {
		return dev_err_probe(dev, ret, "failed to get SPI offload\n");
	}

	st->offload_trigger = devm_spi_offload_trigger_get(dev, st->offload,
			SPI_OFFLOAD_TRIGGER_DATA_READY);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
				     "failed to get offload trigger\n");

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
				     "failed to get offload RX DMA\n");

	tx_dma = devm_spi_offload_tx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(tx_dma))
		return dev_err_probe(dev, PTR_ERR(tx_dma),
				     "failed to get offload TX DMA\n");
	st->offload_tx_chan = tx_dma;

	/*
	 * Both DMA channels have to gate on the same TRU-routed DREADY
	 * pulse — TX so it feeds exactly one command word into MOSI per
	 * trigger, RX so it drains exactly one long-frame response from
	 * MISO per trigger. Program TWAIT=1 on both.
	 */
	ret = adema_offload_config_dma_twait(st, rx_dma, DMA_DEV_TO_MEM);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable TWAIT on RX DMA\n");
	ret = adema_offload_config_dma_twait(st, tx_dma, DMA_MEM_TO_DEV);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable TWAIT on TX DMA\n");

	st->offload_tx_buf = dma_alloc_coherent(tx_dma->device->dev,
			st->chip_info->long_frame_len, &st->offload_tx_dma,
			GFP_KERNEL);
	if (!st->offload_tx_buf)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to allocate offload TX buffer\n");

	ret = devm_add_action_or_reset(dev, adema_offload_free_tx_buf, st);
	if (ret)
		return ret;

	/*
	 * RX side: a kfifo IIO buffer fed by a per-enable ping-pong
	 * descriptor-list DMA ring sized from the buffer watermark
	 * (see adema_offload_start_rx_stream()).
	 */
	st->offload_rx_chan = rx_dma;
	st->offload_indio_dev = indio_dev;

	st->offload_rx_def_bank_frames = ADEMA_OFFLOAD_DEF_BANK_FRAMES;
	device_property_read_u32(dev, "adi,offload-frames-per-interrupt",
				 &st->offload_rx_def_bank_frames);
	st->offload_rx_def_bank_frames = clamp_t(unsigned int,
			st->offload_rx_def_bank_frames, 1,
			ADEMA_OFFLOAD_MAX_BANK_FRAMES);

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &adema_offload_buffer_setup_ops);
	if (ret)
		return dev_err_probe(dev, ret, "cannot set up kfifo buffer\n");
	/*
	 * Replace the software-path channel list (N × s32 + timestamp) with
	 * the raw-frame layout (one u8 scan element of long_frame_len
	 * bytes). Consumer userspace unpacks the interleaved channel /
	 * status / CRC bytes — see Documentation/iio/adema127.rst.
	 */
	if (st->chip_info->num_channels == 7) {
		indio_dev->channels = adema127_offload_channels;
		indio_dev->num_channels = ARRAY_SIZE(adema127_offload_channels);
		indio_dev->available_scan_masks = adema127_offload_scan_masks;
	} else {
		indio_dev->channels = adema124_offload_channels;
		indio_dev->num_channels = ARRAY_SIZE(adema124_offload_channels);
		indio_dev->available_scan_masks = adema124_offload_scan_masks;
	}
	return 0;
}
