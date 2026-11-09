// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ADRV904X RAMC (RAM Capture) interface
 *
 * Copyright 2020-2026 Analog Devices Inc.
 *
 * This driver provides access to the RX/ORX data capture RAM for debugging
 * and signal analysis. The hardware can only capture from one channel at a
 * time, so only one I/Q pair can be enabled simultaneously.
 *
 * Capture is single-shot: enabling the buffer triggers one capture, fills
 * the buffer with samples, then stops. To capture again, disable and
 * re-enable the buffer (or use iio_readdev which handles this automatically).
 */

#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>

#include "adrv904x.h"
#include "adi_adrv904x_datainterface.h"
#include "adi_adrv904x_datainterface_types.h"

/* Max capture is 32768 samples × 4 bytes = 128KB */
#define ADRV904X_MAX_RAM_SIZE_BYTES	(32768 * sizeof(u32))
#define ADRV904X_RAMC_TIMEOUT_US	1000000

#define ADRV904X_RAMC_NUM_RX_CHANNELS	8
#define ADRV904X_RAMC_NUM_ORX_CHANNELS	2
#define ADRV904X_RAMC_NUM_CHANNELS \
	(ADRV904X_RAMC_NUM_RX_CHANNELS + ADRV904X_RAMC_NUM_ORX_CHANNELS)

/* Total IIO channels: (RX0-7 + ORX0-1) * 2 (I/Q) + timestamp */
#define ADRV904X_RAMC_NUM_IIO_CHANNELS	(ADRV904X_RAMC_NUM_CHANNELS * 2 + 1)

struct adrv904x_ramc_state {
	struct device *dev;
	/* sample_count and active_channel* are protected by phy->lock */
	struct adrv904x_rf_phy *phy;
	struct iio_dev *indio_dev;
	u32 sample_count;
	u32 *channel_buffer;
	unsigned long active_channel;
	int active_channel_idx;

	IIO_DECLARE_DMA_BUFFER_WITH_TS(s32, scan_data, 2);
};

/* Map channel index to API channel mask */
static const u32 adrv904x_ramc_channel_map[] = {
	ADI_ADRV904X_RX0,	/* 0 */
	ADI_ADRV904X_RX1,	/* 1 */
	ADI_ADRV904X_RX2,	/* 2 */
	ADI_ADRV904X_RX3,	/* 3 */
	ADI_ADRV904X_RX4,	/* 4 */
	ADI_ADRV904X_RX5,	/* 5 */
	ADI_ADRV904X_RX6,	/* 6 */
	ADI_ADRV904X_RX7,	/* 7 */
	ADI_ADRV904X_ORX0,	/* 8 */
	ADI_ADRV904X_ORX1,	/* 9 */
};

/*
 * Valid scan masks - only one I/Q pair at a time.
 * Each mask enables both I and Q channels for one RX/ORX channel.
 * Channels are: RX0_I(0), RX0_Q(1), RX1_I(2), RX1_Q(3), ..., ORX1_Q(19)
 */
static const unsigned long adrv904x_ramc_scan_masks[] = {
	BIT(0)  | BIT(1),	/* RX0 I+Q */
	BIT(2)  | BIT(3),	/* RX1 I+Q */
	BIT(4)  | BIT(5),	/* RX2 I+Q */
	BIT(6)  | BIT(7),	/* RX3 I+Q */
	BIT(8)  | BIT(9),	/* RX4 I+Q */
	BIT(10) | BIT(11),	/* RX5 I+Q */
	BIT(12) | BIT(13),	/* RX6 I+Q */
	BIT(14) | BIT(15),	/* RX7 I+Q */
	BIT(16) | BIT(17),	/* ORX0 I+Q */
	BIT(18) | BIT(19),	/* ORX1 I+Q */
	0
};

static inline s32 sign_extend_28bit(u32 val)
{
	return sign_extend32(val, 27);
}

static void adrv904x_ramc_push_sample(struct adrv904x_ramc_state *st,
				      const u32 *word)
{
	/* Q at even index, I at odd index; Q needs negation */
	st->scan_data[0] = sign_extend_28bit(word[1]);
	st->scan_data[1] = -sign_extend_28bit(word[0]);

	iio_push_to_buffers_with_ts(st->indio_dev, st->scan_data,
				    sizeof(st->scan_data),
				    iio_get_time_ns(st->indio_dev));
}

static int adrv904x_ramc_capture_and_push(struct adrv904x_ramc_state *st)
{
	adi_adrv904x_Device_t *device = st->phy->kororDevice;
	u32 *buf = st->channel_buffer;
	u32 word_count;
	bool is_orx;
	int ret;

	guard(mutex)(&st->phy->lock);

	word_count = st->sample_count;
	is_orx = st->active_channel_idx >= ADRV904X_RAMC_NUM_RX_CHANNELS;

	/* ORx capture RAM is limited to 12K (3 banks x 4K each) */
	if (is_orx && word_count > ADI_ADRV904X_CAPTURE_SIZE_12K)
		return -EINVAL;

	ret = adi_adrv904x_RxOrxDataCaptureStart(device, st->active_channel,
						 ADI_ADRV904X_CAPTURE_LOC_DDC0,
						 buf, word_count, 0,
						 ADRV904X_RAMC_TIMEOUT_US);
	if (ret) {
		dev_err(st->dev, "Failed to capture channel 0x%lx: %d\n",
			st->active_channel, ret);
		return -EIO;
	}

	if (is_orx) {
		/*
		 * ORx capture RAM: 3 banks of 4K words each, read sequentially
		 * by the API, so samples are contiguous in the buffer.
		 */
		for (unsigned int i = 0; i < word_count / 2; i++)
			adrv904x_ramc_push_sample(st, &buf[2 * i]);
	} else {
		/*
		 * RX/DPD capture RAM: 2 banks of 8K words each, read back to
		 * back by the API. Samples are interleaved across banks:
		 * sample[2n] from bank 0, sample[2n + 1] from bank 1.
		 */
		u32 *bank0 = buf;
		u32 *bank1 = buf + word_count / 2;

		for (unsigned int i = 0; i < word_count / 4; i++) {
			adrv904x_ramc_push_sample(st, &bank0[2 * i]);
			adrv904x_ramc_push_sample(st, &bank1[2 * i]);
		}
	}

	return 0;
}

static int adrv904x_ramc_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val, int *val2, long mask)
{
	struct adrv904x_ramc_state *st = iio_priv(indio_dev);
	u64 freq;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		guard(mutex)(&st->phy->lock);
		freq = st->phy->rx_iqRate_kHz * 1000ULL;
		*val = lower_32_bits(freq);
		*val2 = upper_32_bits(freq);
		return IIO_VAL_INT_64;
	}
	default:
		return -EINVAL;
	}
}

static ssize_t adrv904x_ramc_sample_count_show(struct iio_dev *indio_dev,
					       uintptr_t private,
					       const struct iio_chan_spec *chan,
					       char *buf)
{
	struct adrv904x_ramc_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->phy->lock);

	return sysfs_emit(buf, "%u\n", st->sample_count);
}

static ssize_t adrv904x_ramc_sample_count_store(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						const char *buf, size_t len)
{
	struct adrv904x_ramc_state *st = iio_priv(indio_dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	/* Validate capture size against hardware-supported values */
	switch (val) {
	case ADI_ADRV904X_CAPTURE_SIZE_32:
	case ADI_ADRV904X_CAPTURE_SIZE_64:
	case ADI_ADRV904X_CAPTURE_SIZE_128:
	case ADI_ADRV904X_CAPTURE_SIZE_256:
	case ADI_ADRV904X_CAPTURE_SIZE_512:
	case ADI_ADRV904X_CAPTURE_SIZE_1K:
	case ADI_ADRV904X_CAPTURE_SIZE_2K:
	case ADI_ADRV904X_CAPTURE_SIZE_4K:
	case ADI_ADRV904X_CAPTURE_SIZE_8K:
	case ADI_ADRV904X_CAPTURE_SIZE_12K:
		break;
	case ADI_ADRV904X_CAPTURE_SIZE_16K:
	case ADI_ADRV904X_CAPTURE_SIZE_32K:
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&st->phy->lock);

	/* ORx channels limited to 12K max (3 banks x 4K each) */
	if (st->active_channel_idx >= ADRV904X_RAMC_NUM_RX_CHANNELS &&
	    (val == ADI_ADRV904X_CAPTURE_SIZE_16K ||
	     val == ADI_ADRV904X_CAPTURE_SIZE_32K))
		return -EINVAL;

	st->sample_count = val;

	return len;
}

static const struct iio_chan_spec_ext_info adrv904x_ramc_ext_info[] = {
	{
		.name = "sample_count",
		.read = adrv904x_ramc_sample_count_show,
		.write = adrv904x_ramc_sample_count_store,
		.shared = IIO_SHARED_BY_ALL,
	},
	{ }
};

static int adrv904x_ramc_update_scan_mode(struct iio_dev *indio_dev,
					  const unsigned long *scan_mask)
{
	struct adrv904x_ramc_state *st = iio_priv(indio_dev);

	for (unsigned int i = 0; i < ADRV904X_RAMC_NUM_CHANNELS; i++) {
		if (*scan_mask == adrv904x_ramc_scan_masks[i]) {
			guard(mutex)(&st->phy->lock);
			st->active_channel_idx = i;
			st->active_channel = adrv904x_ramc_channel_map[i];
			return 0;
		}
	}

	return -EINVAL;
}

static const struct iio_info adrv904x_ramc_info = {
	.read_raw = &adrv904x_ramc_read_raw,
	.update_scan_mode = &adrv904x_ramc_update_scan_mode,
};

static int adrv904x_ramc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adrv904x_ramc_state *st = iio_priv(indio_dev);

	return adrv904x_ramc_capture_and_push(st);
}

static const struct iio_buffer_setup_ops adrv904x_ramc_buffer_ops = {
	.postenable = adrv904x_ramc_buffer_postenable,
};

int adrv904x_ramc_probe(struct adrv904x_rf_phy *phy)
{
	struct iio_dev *indio_dev;
	struct adrv904x_ramc_state *st;
	struct device *dev = &phy->spi->dev;
	struct iio_chan_spec *channels;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->indio_dev = indio_dev;
	st->dev = dev;
	st->phy = phy;
	st->sample_count = ADI_ADRV904X_CAPTURE_SIZE_2K;
	st->active_channel = ADI_ADRV904X_RX0;
	st->active_channel_idx = 0;

	st->channel_buffer = devm_kzalloc(dev, ADRV904X_MAX_RAM_SIZE_BYTES, GFP_KERNEL);
	if (!st->channel_buffer)
		return -ENOMEM;

	channels = devm_kcalloc(dev, ADRV904X_RAMC_NUM_IIO_CHANNELS, sizeof(*channels),
				GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	/* Create I/Q channel pairs for RX0-7 and ORX0-1 */
	for (unsigned int i = 0; i < ADRV904X_RAMC_NUM_CHANNELS; i++) {
		/* I channel */
		channels[i * 2].type = IIO_VOLTAGE;
		channels[i * 2].indexed = 1;
		channels[i * 2].modified = 1;
		channels[i * 2].channel = i;
		channels[i * 2].channel2 = IIO_MOD_I;
		channels[i * 2].scan_index = i * 2;
		channels[i * 2].info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		channels[i * 2].ext_info = adrv904x_ramc_ext_info;
		channels[i * 2].scan_type.sign = 's';
		channels[i * 2].scan_type.realbits = 28;
		channels[i * 2].scan_type.storagebits = 32;
		channels[i * 2].scan_type.endianness = IIO_LE;

		/* Q channel */
		channels[i * 2 + 1].type = IIO_VOLTAGE;
		channels[i * 2 + 1].indexed = 1;
		channels[i * 2 + 1].modified = 1;
		channels[i * 2 + 1].channel = i;
		channels[i * 2 + 1].channel2 = IIO_MOD_Q;
		channels[i * 2 + 1].scan_index = i * 2 + 1;
		channels[i * 2 + 1].info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		channels[i * 2 + 1].ext_info = adrv904x_ramc_ext_info;
		channels[i * 2 + 1].scan_type.sign = 's';
		channels[i * 2 + 1].scan_type.realbits = 28;
		channels[i * 2 + 1].scan_type.storagebits = 32;
		channels[i * 2 + 1].scan_type.endianness = IIO_LE;
	}

	/* Timestamp channel */
	channels[ADRV904X_RAMC_NUM_CHANNELS * 2] = (struct iio_chan_spec)
		IIO_CHAN_SOFT_TIMESTAMP(ADRV904X_RAMC_NUM_CHANNELS * 2);

	indio_dev->info = &adrv904x_ramc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = channels;
	indio_dev->num_channels = ADRV904X_RAMC_NUM_IIO_CHANNELS;
	indio_dev->available_scan_masks = adrv904x_ramc_scan_masks;
	indio_dev->name = "adrv904x-ramc";

	ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev, &adrv904x_ramc_buffer_ops, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}
