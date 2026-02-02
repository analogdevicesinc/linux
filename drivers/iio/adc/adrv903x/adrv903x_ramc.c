// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ADRV903X RAMC (RAM Capture) interface
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include "adrv903x.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer_impl.h>

#include "adi_adrv903x_datainterface.h"

#define ADRV903X_MAX_RAM_SIZE_BYTES 32768

#define TIMEOUT_US 1000

struct adrv903x_ramc_state {
	struct device *dev;
	struct mutex lock;
	struct adrv903x_rf_phy *phy;
	struct delayed_work capture_work;
	struct iio_dev *indio_dev;

	u32 sample_count;

	/* Buffer for captured data */
	__le32 *channel_buffer;  /* One buffer, single channel capture */

	/* Scan buffer for pushing to IIO (I + Q samples + timestamp) */
	u8 scan_data[ALIGN(2 * sizeof(s32), sizeof(s64)) + sizeof(s64)] __aligned(IIO_DMA_MINALIGN);

	/* Track which channel is enabled for capture */
	unsigned long active_channel;

	/* Flag to control capture work scheduling */
	bool capture_enabled;
};

/* Convert 28-bit two's complement to signed 32-bit */
static inline s32 twos_comp_28bit(u32 val)
{
	if (val & BIT(27))
		return (s32)(val | 0xF0000000);
	return (s32)(val & 0x0FFFFFFF);
}

static int adrv903x_ramc_read_samples(struct adrv903x_ramc_state *st)
{
	adi_adrv903x_Device_t *device = st->phy->palauDevice;
	u32 i, word_count, offset, bank_samples;
	u32 *bank0, *bank1;
	s32 *scan_s32;
	int ret;

	/* Get number of 32-bit words to read */
	word_count = st->sample_count;

	guard(mutex)(&st->phy->lock);

	dev_dbg(st->dev, "%s: word_count=%u channel=0x%lx\n",
		__func__, word_count, st->active_channel);

	ret = adi_adrv903x_RxOrxDataCaptureStart(device,
						 st->active_channel,
						 ADI_ADRV903X_CAPTURE_LOC_DDC0,
						 (u32 *)st->channel_buffer,
						 word_count,
						 0, /* trigger = 0 (immediate) */
						 TIMEOUT_US);
	if (ret) {
		dev_err(st->dev, "Failed to read channel 0x%lx: %d\n",
			st->active_channel, ret);
		return -EIO;
	}

	/*
	 * Step 2: Demultiplex data according to bank/I/Q layout
	 *
	 * Data layout from capture RAM:
	 *   - First half (Bank 0): Q at even indices, I at odd indices
	 *   - Second half (Bank 1): Q at even indices, I at odd indices
	 *   - Interleave: sample[2n] from Bank0, sample[2n+1] from Bank1
	 *   - Each sample is 28-bit two's complement
	 *   - Q needs to be negated
	 */
	offset = word_count / 2;
	bank0 = (u32 *)st->channel_buffer;
	bank1 = (u32 *)st->channel_buffer + offset;
	bank_samples = offset / 2;  /* Number of I/Q pairs per bank */

	scan_s32 = (s32 *)st->scan_data;

	for (i = 0; i < bank_samples; i++) {
		s32 I_bank0, Q_bank0, I_bank1, Q_bank1;

		/* Extract from Bank 0: Q at even index, I at odd index */
		Q_bank0 = -twos_comp_28bit(bank0[2 * i]);
		I_bank0 = twos_comp_28bit(bank0[2 * i + 1]);

		/* Extract from Bank 1: Q at even index, I at odd index */
		Q_bank1 = -twos_comp_28bit(bank1[2 * i]);
		I_bank1 = twos_comp_28bit(bank1[2 * i + 1]);

		/* Push interleaved samples: Bank0 sample, then Bank1 sample */
		/* Sample from Bank 0 */
		scan_s32[0] = I_bank0;
		scan_s32[1] = Q_bank0;
		iio_push_to_buffers_with_timestamp(st->indio_dev, st->scan_data,
						   iio_get_time_ns(st->indio_dev));

		/* Sample from Bank 1 */
		scan_s32[0] = I_bank1;
		scan_s32[1] = Q_bank1;
		iio_push_to_buffers_with_timestamp(st->indio_dev, st->scan_data,
						   iio_get_time_ns(st->indio_dev));
	}

	return 0;
}

static void adrv903x_ramc_capture_work_func(struct work_struct *work)
{
	struct adrv903x_ramc_state *st =
		container_of(work, struct adrv903x_ramc_state, capture_work.work);
	int ret;

	if (!st->capture_enabled)
		return;

	ret = adrv903x_ramc_read_samples(st);
	if (ret) {
		dev_err(st->dev, "Error reading Rx RAM samples: %d\n", ret);
		return;  /* Don't reschedule on error */
	}

	/* Reschedule if still enabled */
	if (st->capture_enabled)
		queue_delayed_work(system_freezable_wq, &st->capture_work,
				   msecs_to_jiffies(10));
}

static int adrv903x_ramc_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val, int *val2, long mask)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);
	int ret = 0;
	u64 freq;

	guard(mutex)(&st->phy->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		freq = st->phy->rx_iqRate_kHz * 1000;
		*val = lower_32_bits(freq);
		*val2 = upper_32_bits(freq);
		ret = IIO_VAL_INT_64;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t adrv903x_ramc_ext_info_read(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);
	int ret = 0;

	guard(mutex)(&st->phy->lock);

	switch (private) {
	case 0: /* active channel */
		ret = sprintf(buf, "0x%lx\n", st->active_channel);
		break;
	case 1: /* sample_count */
		ret = sprintf(buf, "%u\n", st->sample_count);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t adrv903x_ramc_ext_info_write(struct iio_dev *indio_dev,
					    uintptr_t private,
					    const struct iio_chan_spec *chan,
					    const char *buf, size_t len)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);
	unsigned long readin;
	int ret;

	guard(mutex)(&st->phy->lock);

	switch (private) {
	case 0: /* active channel */
		ret = kstrtoul(buf, 10, &readin);
		if (ret)
			return ret;
		/* range check the channel selection parameter */
		if (readin != ADI_ADRV903X_RX0 &&
		    readin != ADI_ADRV903X_RX1 &&
		    readin != ADI_ADRV903X_RX2 &&
		    readin != ADI_ADRV903X_RX3 &&
		    readin != ADI_ADRV903X_RX4 &&
		    readin != ADI_ADRV903X_RX5 &&
		    readin != ADI_ADRV903X_RX6 &&
		    readin != ADI_ADRV903X_RX7 &&
		    readin != ADI_ADRV903X_ORX0 &&
		    readin != ADI_ADRV903X_ORX1)
			return -EINVAL;
		st->active_channel = readin;
		break;
	case 1: /* sample_count */
		ret = kstrtoul(buf, 10, &readin);
		if (ret)
			return ret;
		/* range check the capture data length parameter */
		if ((st->active_channel == ADI_ADRV903X_ORX0 ||
		     st->active_channel == ADI_ADRV903X_ORX1) &&
		    (readin == ADI_ADRV903X_CAPTURE_SIZE_16K ||
		     readin == ADI_ADRV903X_CAPTURE_SIZE_32K))
			return -EINVAL;
		if (readin != ADI_ADRV903X_CAPTURE_SIZE_16K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_32K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_12K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_8K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_4K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_2K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_1K &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_512 &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_256 &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_128 &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_64 &&
		    readin != ADI_ADRV903X_CAPTURE_SIZE_32)
			return -EINVAL;
		st->sample_count = readin;
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info adrv903x_ramc_ext_info[] = {
	{
		.name = "active_channel",
		.read = adrv903x_ramc_ext_info_read,
		.write = adrv903x_ramc_ext_info_write,
		.shared = IIO_SHARED_BY_ALL,
		.private = 0,
	},
	{
		.name = "sample_count",
		.read = adrv903x_ramc_ext_info_read,
		.write = adrv903x_ramc_ext_info_write,
		.shared = IIO_SHARED_BY_ALL,
		.private = 1,
	},
	{},
};

/* Channel template for dynamic channel allocation */
static const struct iio_chan_spec adrv903x_ramc_channel_template = {
	.type = IIO_VOLTAGE,
	.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = adrv903x_ramc_ext_info,
	.indexed = 1,
	.scan_type = {
		.sign = 's',
		.realbits = 28,
		.storagebits = 32,
		.shift = 0,
		.endianness = IIO_LE,
	},
};

static int adrv903x_ramc_update_scan_mode(struct iio_dev *indio_dev,
					  const unsigned long *scan_mask)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s: scan_mask=0x%lx\n", __func__, *scan_mask);

	return 0;
}

static const struct iio_info adrv903x_ramc_info = {
	.read_raw = &adrv903x_ramc_read_raw,
	.update_scan_mode = &adrv903x_ramc_update_scan_mode,
};

static int adrv903x_ramc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s: buffer length: %d\n", __func__, indio_dev->buffer->length);

	st->capture_enabled = true;
	queue_delayed_work(system_freezable_wq, &st->capture_work, msecs_to_jiffies(100));

	return 0;
}

static int adrv903x_ramc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adrv903x_ramc_state *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s:%d\n", __func__, __LINE__);

	st->capture_enabled = false;
	cancel_delayed_work_sync(&st->capture_work);

	return 0;
}

static const struct iio_buffer_setup_ops adrv903x_ramc_buffer_ops = {
	.postenable = adrv903x_ramc_buffer_postenable,
	.predisable = adrv903x_ramc_buffer_predisable,
};

int adrv903x_ramc_probe(struct adrv903x_rf_phy *phy)
{
	struct iio_dev *indio_dev;
	struct adrv903x_ramc_state *st;
	struct device *dev = &phy->spi->dev;
	struct iio_chan_spec *channels;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(dev, "Can't allocate iio device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->indio_dev = indio_dev;
	st->dev = dev;
	st->phy = phy;

	/* Initialize default configuration */
	st->sample_count = 2048;
	st->active_channel = (unsigned long)ADI_ADRV903X_RX0;

	mutex_init(&st->lock);
	INIT_DELAYED_WORK(&st->capture_work, adrv903x_ramc_capture_work_func);

	/* Allocate buffer for captured samples */
	st->channel_buffer = devm_kzalloc(dev, ADRV903X_MAX_RAM_SIZE_BYTES, GFP_KERNEL);
	if (!st->channel_buffer)
		return -ENOMEM;

	/* Allocate mem for 3 channels (I, Q, and timestamp) */
	channels = devm_kcalloc(dev, 3, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	/* Channel 0: I (in-phase) */
	channels[0] = adrv903x_ramc_channel_template;
	channels[0].channel = 0;
	channels[0].scan_index = 0;

	/* Channel 1: Q (quadrature) */
	channels[1] = adrv903x_ramc_channel_template;
	channels[1].channel = 1;
	channels[1].scan_index = 1;

	/* Channel 2: Timestamp */
	channels[2] = (struct iio_chan_spec)IIO_CHAN_SOFT_TIMESTAMP(2);

	indio_dev->info = &adrv903x_ramc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = channels;
	indio_dev->num_channels = 3;
	indio_dev->name = "adrv903x-ramc";

	dev_info(dev, "%s: 3 channels (I/Q/timestamp)\n", indio_dev->name);

	ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev, &adrv903x_ramc_buffer_ops, NULL);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(adrv903x_ramc_probe);
