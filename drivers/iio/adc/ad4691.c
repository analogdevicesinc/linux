// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024-2026 Analog Devices, Inc.
 * Author: Radu Sabau <radu.sabau@analog.com>
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/offload/provider.h>
#include <linux/units.h>
#include <linux/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define AD4691_VREF_uV_MIN			2400000
#define AD4691_VREF_uV_MAX			5250000
#define AD4691_VREF_2P5_uV_MAX			2750000
#define AD4691_VREF_3P0_uV_MAX			3250000
#define AD4691_VREF_3P3_uV_MAX			3750000
#define AD4691_VREF_4P096_uV_MAX		4500000

#define AD4691_CNV_DUTY_CYCLE_NS		380
#define AD4691_CNV_HIGH_TIME_NS			430
/*
 * Conservative default for the manual offload periodic trigger. Low enough
 * to work safely out of the box across all OSR and channel count combinations.
 */
#define AD4691_OFFLOAD_INITIAL_TRIGGER_HZ	(100 * HZ_PER_KHZ)

#define AD4691_SPI_CONFIG_A_REG			0x000
#define AD4691_SW_RESET				(BIT(7) | BIT(0))

#define AD4691_STATUS_REG			0x014
#define AD4691_CLAMP_STATUS1_REG		0x01A
#define AD4691_CLAMP_STATUS2_REG		0x01B
#define AD4691_DEVICE_SETUP			0x020
#define AD4691_MANUAL_MODE			BIT(2)
#define AD4691_LDO_EN				BIT(4)
#define AD4691_REF_CTRL				0x021
#define AD4691_REF_CTRL_MASK			GENMASK(4, 2)
#define AD4691_REFBUF_EN			BIT(0)
#define AD4691_OSC_FREQ_REG			0x023
#define AD4691_OSC_FREQ_MASK			GENMASK(3, 0)
#define AD4691_STD_SEQ_CONFIG			0x025
#define AD4691_SEQ_ALL_CHANNELS_OFF		0x00
#define AD4691_SPARE_CONTROL			0x02A

#define AD4691_NOOP				0x00
#define AD4691_ADC_CHAN(ch)			((0x10 + (ch)) << 3)

#define AD4691_OSC_EN_REG			0x180
#define AD4691_STATE_RESET_REG			0x181
#define AD4691_STATE_RESET_ALL			0x01
#define AD4691_ADC_SETUP			0x182
#define AD4691_ADC_MODE_MASK			GENMASK(1, 0)
#define AD4691_CNV_BURST_MODE			0x01
#define AD4691_AUTONOMOUS_MODE			0x02
/*
 * ACC_MASK_REG covers both mask bytes via ADDR_DESCENDING SPI: writing a
 * 16-bit BE value to 0x185 auto-decrements to 0x184 for the second byte.
 */
#define AD4691_ACC_MASK_REG			0x185
#define AD4691_ACC_DEPTH_IN(n)			(0x186 + (n))
#define AD4691_GPIO_MODE1_REG			0x196
#define AD4691_GPIO_MODE2_REG			0x197
#define AD4691_GP_MODE_MASK			GENMASK(3, 0)
#define AD4691_GP_MODE_DATA_READY		0x06
#define AD4691_GPIO_READ			0x1A0
#define AD4691_ACC_STATUS_FULL1_REG		0x1B0
#define AD4691_ACC_STATUS_FULL2_REG		0x1B1
#define AD4691_ACC_STATUS_OVERRUN1_REG		0x1B2
#define AD4691_ACC_STATUS_OVERRUN2_REG		0x1B3
#define AD4691_ACC_STATUS_SAT1_REG		0x1B4
#define AD4691_ACC_STATUS_SAT2_REG		0x1BE
#define AD4691_ACC_SAT_OVR_REG(n)		(0x1C0 + (n))
#define AD4691_AVG_IN(n)			(0x201 + (2 * (n)))
#define AD4691_AVG_STS_IN(n)			(0x222 + (3 * (n)))
#define AD4691_ACC_IN(n)			(0x252 + (3 * (n)))
#define AD4691_ACC_STS_DATA(n)			(0x283 + (4 * (n)))

static const char * const ad4691_supplies[] = { "avdd", "vio" };

enum ad4691_ref_ctrl {
	AD4691_VREF_2P5   = 0,
	AD4691_VREF_3P0   = 1,
	AD4691_VREF_3P3   = 2,
	AD4691_VREF_4P096 = 3,
	AD4691_VREF_5P0   = 4,
};

struct ad4691_channel_info {
	const struct iio_chan_spec *channels;
	const struct iio_chan_spec *manual_channels;
	unsigned int num_channels;
};

struct ad4691_chip_info {
	const char *name;
	unsigned int max_rate;
	const struct ad4691_channel_info *sw_info;
	const struct ad4691_channel_info *offload_info;
};

/* CNV burst mode channel — exposes oversampling ratio. */
#define AD4691_CHANNEL(ch)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) | \
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.info_mask_separate_available =				\
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) | \
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
		.channel = ch,						\
		.scan_index = ch,					\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.endianness = IIO_BE,				\
		},							\
	}

/*
 * Manual mode channel — no oversampling ratio attribute. OSR is not
 * supported in manual mode; ACC_DEPTH_IN is not configured during manual
 * buffer enable.
 */
#define AD4691_MANUAL_CHANNEL(ch)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.info_mask_separate_available =				\
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),	\
		.channel = ch,						\
		.scan_index = ch,					\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.endianness = IIO_BE,				\
		},							\
	}

static const struct iio_chan_spec ad4691_channels[] = {
	AD4691_CHANNEL(0),
	AD4691_CHANNEL(1),
	AD4691_CHANNEL(2),
	AD4691_CHANNEL(3),
	AD4691_CHANNEL(4),
	AD4691_CHANNEL(5),
	AD4691_CHANNEL(6),
	AD4691_CHANNEL(7),
	AD4691_CHANNEL(8),
	AD4691_CHANNEL(9),
	AD4691_CHANNEL(10),
	AD4691_CHANNEL(11),
	AD4691_CHANNEL(12),
	AD4691_CHANNEL(13),
	AD4691_CHANNEL(14),
	AD4691_CHANNEL(15),
	IIO_CHAN_SOFT_TIMESTAMP(16),
};

static const struct iio_chan_spec ad4693_channels[] = {
	AD4691_CHANNEL(0),
	AD4691_CHANNEL(1),
	AD4691_CHANNEL(2),
	AD4691_CHANNEL(3),
	AD4691_CHANNEL(4),
	AD4691_CHANNEL(5),
	AD4691_CHANNEL(6),
	AD4691_CHANNEL(7),
	IIO_CHAN_SOFT_TIMESTAMP(8),
};

static const struct iio_chan_spec ad4691_manual_channels[] = {
	AD4691_MANUAL_CHANNEL(0),
	AD4691_MANUAL_CHANNEL(1),
	AD4691_MANUAL_CHANNEL(2),
	AD4691_MANUAL_CHANNEL(3),
	AD4691_MANUAL_CHANNEL(4),
	AD4691_MANUAL_CHANNEL(5),
	AD4691_MANUAL_CHANNEL(6),
	AD4691_MANUAL_CHANNEL(7),
	AD4691_MANUAL_CHANNEL(8),
	AD4691_MANUAL_CHANNEL(9),
	AD4691_MANUAL_CHANNEL(10),
	AD4691_MANUAL_CHANNEL(11),
	AD4691_MANUAL_CHANNEL(12),
	AD4691_MANUAL_CHANNEL(13),
	AD4691_MANUAL_CHANNEL(14),
	AD4691_MANUAL_CHANNEL(15),
	IIO_CHAN_SOFT_TIMESTAMP(16),
};

static const struct iio_chan_spec ad4693_manual_channels[] = {
	AD4691_MANUAL_CHANNEL(0),
	AD4691_MANUAL_CHANNEL(1),
	AD4691_MANUAL_CHANNEL(2),
	AD4691_MANUAL_CHANNEL(3),
	AD4691_MANUAL_CHANNEL(4),
	AD4691_MANUAL_CHANNEL(5),
	AD4691_MANUAL_CHANNEL(6),
	AD4691_MANUAL_CHANNEL(7),
	IIO_CHAN_SOFT_TIMESTAMP(8),
};

static const int ad4691_oversampling_ratios[] = { 1, 2, 4, 8, 16, 32 };

static const struct ad4691_channel_info ad4691_sw_info = {
	.channels = ad4691_channels,
	.manual_channels = ad4691_manual_channels,
	.num_channels = ARRAY_SIZE(ad4691_channels),
};

static const struct ad4691_channel_info ad4693_sw_info = {
	.channels = ad4693_channels,
	.manual_channels = ad4693_manual_channels,
	.num_channels = ARRAY_SIZE(ad4693_channels),
};

static const struct ad4691_channel_info ad4691_offload_info = {
	.channels = ad4691_channels,
	/*
	 * Offload paths share the SW channel arrays. num_channels caps access
	 * before the soft timestamp entry, so no separate array is needed.
	 */
	.manual_channels = ad4691_manual_channels,
	.num_channels = ARRAY_SIZE(ad4691_channels) - 1,
};

static const struct ad4691_channel_info ad4693_offload_info = {
	.channels = ad4693_channels,
	.manual_channels = ad4693_manual_channels,
	.num_channels = ARRAY_SIZE(ad4693_channels) - 1,
};

/*
 * Internal oscillator frequency table. Index is the OSC_FREQ_REG[3:0] value.
 * Index 0 (1 MHz) is only valid for AD4692/AD4694; AD4691/AD4693 support
 * up to 500 kHz and use index 1 as their highest valid rate.
 */
static const int ad4691_osc_freqs_Hz[] = {
	[0x0] = 1000000,
	[0x1] = 500000,
	[0x2] = 400000,
	[0x3] = 250000,
	[0x4] = 200000,
	[0x5] = 167000,
	[0x6] = 133000,
	[0x7] = 125000,
	[0x8] = 100000,
	[0x9] = 50000,
	[0xA] = 25000,
	[0xB] = 12500,
	[0xC] = 10000,
	[0xD] = 5000,
	[0xE] = 2500,
	[0xF] = 1250,
};

static const char * const ad4691_gp_names[] = { "gp0", "gp1", "gp2", "gp3" };

static const struct ad4691_chip_info ad4691_chip_info = {
	.name = "ad4691",
	.max_rate = 500 * HZ_PER_KHZ,
	.sw_info = &ad4691_sw_info,
	.offload_info = &ad4691_offload_info,
};

static const struct ad4691_chip_info ad4692_chip_info = {
	.name = "ad4692",
	.max_rate = 1 * HZ_PER_MHZ,
	.sw_info = &ad4691_sw_info,
	.offload_info = &ad4691_offload_info,
};

static const struct ad4691_chip_info ad4693_chip_info = {
	.name = "ad4693",
	.max_rate = 500 * HZ_PER_KHZ,
	.sw_info = &ad4693_sw_info,
	.offload_info = &ad4693_offload_info,
};

static const struct ad4691_chip_info ad4694_chip_info = {
	.name = "ad4694",
	.max_rate = 1 * HZ_PER_MHZ,
	.sw_info = &ad4693_sw_info,
	.offload_info = &ad4693_offload_info,
};

struct ad4691_offload_state {
	struct spi_offload *offload;
	struct spi_offload_trigger *trigger;
	u64 trigger_hz;
	u8 tx_cmd[17][2];
	u8 tx_reset[4];
};

struct ad4691_state {
	const struct ad4691_chip_info *info;
	struct regmap *regmap;
	struct spi_device *spi;

	struct pwm_device *conv_trigger;
	int irq;
	int vref_uV;
	u32 cnv_period_ns;
	/*
	 * Snapped oscillator frequency (Hz) shared by all channels. Set when
	 * sampling_frequency or oversampling_ratio is written; written to
	 * OSC_FREQ_REG at buffer enable and single-shot time so both attributes
	 * can be set in any order. Reading in_voltageN_sampling_frequency
	 * returns target_osc_freq_Hz / osr[N] — the effective rate for that
	 * channel given its oversampling ratio.
	 */
	u32 target_osc_freq_Hz;
	/* Per-channel oversampling ratio; always 1 in manual mode. */
	u8 osr[16];
	/* Scratch buffer for read_avail SAMP_FREQ; content is OSR-dependent. */
	int samp_freq_avail[ARRAY_SIZE(ad4691_osc_freqs_Hz)];

	bool manual_mode;
	bool refbuf_en;
	bool ldo_en;
	/*
	 * Synchronize access to members of the driver state, and ensure
	 * atomicity of consecutive SPI operations.
	 */
	struct mutex lock;
	/*
	 * Per-buffer-enable lifetime resources:
	 * Manual Mode - a pre-built SPI message that clocks out N+1
	 *		 transfers in one go.
	 * CNV Burst Mode - a pre-built SPI message that clocks out 2*N
	 *		    transfers in one go.
	 */
	struct spi_message scan_msg;
	/*
	 * Non-offload: max 16 + 1 NOOP (manual) or 2*16 + 2 (CNV burst).
	 * Offload reuses this array — both paths are mutually exclusive.
	 */
	struct spi_transfer scan_xfers[34];
	/*
	 * CNV burst: 16 AVG_IN addresses + state-reset address + state-reset
	 * value = 18.  Manual: 16 channel cmds + 1 NOOP = 17.
	 */
	__be16 scan_tx[18] __aligned(IIO_DMA_MINALIGN);
	/*
	 * Scan buffer: one BE16 slot per active channel, plus timestamp.
	 * DMA-aligned because scan_xfers point rx_buf directly into vals[].
	 */
	IIO_DECLARE_DMA_BUFFER_WITH_TS(__be16, vals, 16);
	/* NULL when no SPI offload hardware is present */
	struct ad4691_offload_state *offload;
};

/*
 * Configure the given GP pin (0-3) as DATA_READY output.
 * GP0/GP1 → GPIO_MODE1_REG, GP2/GP3 → GPIO_MODE2_REG.
 * Even pins occupy bits [3:0], odd pins bits [7:4].
 */
static int ad4691_gpio_setup(struct ad4691_state *st, unsigned int gp_num)
{
	unsigned int bit_off = gp_num % 2;
	unsigned int reg_off = gp_num / 2;
	unsigned int shift = 4 * bit_off;

	return regmap_update_bits(st->regmap,
				  AD4691_GPIO_MODE1_REG + reg_off,
				  AD4691_GP_MODE_MASK << shift,
				  AD4691_GP_MODE_DATA_READY << shift);
}

static const struct spi_offload_config ad4691_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static bool ad4691_offload_trigger_match(struct spi_offload_trigger *trigger,
					 enum spi_offload_trigger_type type,
					 u64 *args, u32 nargs)
{
	return type == SPI_OFFLOAD_TRIGGER_DATA_READY &&
	       nargs == 1 && args[0] <= 3;
}

static int ad4691_offload_trigger_request(struct spi_offload_trigger *trigger,
					  enum spi_offload_trigger_type type,
					  u64 *args, u32 nargs)
{
	struct ad4691_state *st = spi_offload_trigger_get_priv(trigger);

	if (nargs != 1)
		return -EINVAL;

	return ad4691_gpio_setup(st, args[0]);
}

static int ad4691_offload_trigger_validate(struct spi_offload_trigger *trigger,
					   struct spi_offload_trigger_config *config)
{
	if (config->type != SPI_OFFLOAD_TRIGGER_DATA_READY)
		return -EINVAL;

	return 0;
}

static const struct spi_offload_trigger_ops ad4691_offload_trigger_ops = {
	.match    = ad4691_offload_trigger_match,
	.request  = ad4691_offload_trigger_request,
	.validate = ad4691_offload_trigger_validate,
};

static int ad4691_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	u8 tx[2], rx[4];
	int ret;

	/* Set bit 15 to mark the operation as READ. */
	put_unaligned_be16(0x8000 | reg, tx);

	switch (reg) {
	case 0 ... AD4691_OSC_FREQ_REG:
	case AD4691_SPARE_CONTROL ... AD4691_ACC_SAT_OVR_REG(15):
		ret = spi_write_then_read(spi, tx, sizeof(tx), rx, 1);
		if (ret)
			return ret;
		*val = rx[0];
		return 0;
	case AD4691_STD_SEQ_CONFIG:
	case AD4691_AVG_IN(0) ... AD4691_AVG_IN(15):
		ret = spi_write_then_read(spi, tx, sizeof(tx), rx, 2);
		if (ret)
			return ret;
		*val = get_unaligned_be16(rx);
		return 0;
	case AD4691_AVG_STS_IN(0) ... AD4691_AVG_STS_IN(15):
	case AD4691_ACC_IN(0) ... AD4691_ACC_IN(15):
		ret = spi_write_then_read(spi, tx, sizeof(tx), rx, 3);
		if (ret)
			return ret;
		*val = get_unaligned_be24(rx);
		return 0;
	case AD4691_ACC_STS_DATA(0) ... AD4691_ACC_STS_DATA(15):
		ret = spi_write_then_read(spi, tx, sizeof(tx), rx, 4);
		if (ret)
			return ret;
		*val = get_unaligned_be32(rx);
		return 0;
	default:
		return -EINVAL;
	}
}

static int ad4691_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct spi_device *spi = context;
	u8 tx[4];

	put_unaligned_be16(reg, tx);

	switch (reg) {
	case 0 ... AD4691_OSC_FREQ_REG:
	case AD4691_SPARE_CONTROL ... AD4691_ACC_MASK_REG - 1:
	case AD4691_ACC_MASK_REG + 1 ... AD4691_GPIO_MODE2_REG:
		if (val > U8_MAX)
			return -EINVAL;
		tx[2] = val;
		return spi_write_then_read(spi, tx, 3, NULL, 0);
	case AD4691_ACC_MASK_REG:
	case AD4691_STD_SEQ_CONFIG:
		if (val > U16_MAX)
			return -EINVAL;
		put_unaligned_be16(val, &tx[2]);
		return spi_write_then_read(spi, tx, 4, NULL, 0);
	default:
		return -EINVAL;
	}
}

static bool ad4691_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD4691_STATUS_REG:
	case AD4691_CLAMP_STATUS1_REG:
	case AD4691_CLAMP_STATUS2_REG:
	case AD4691_GPIO_READ:
	case AD4691_ACC_STATUS_FULL1_REG ... AD4691_ACC_STATUS_SAT2_REG:
	case AD4691_ACC_SAT_OVR_REG(0) ... AD4691_ACC_SAT_OVR_REG(15):
	case AD4691_AVG_IN(0) ... AD4691_AVG_IN(15):
	case AD4691_AVG_STS_IN(0) ... AD4691_AVG_STS_IN(15):
	case AD4691_ACC_IN(0) ... AD4691_ACC_IN(15):
	case AD4691_ACC_STS_DATA(0) ... AD4691_ACC_STS_DATA(15):
		return true;
	default:
		return false;
	}
}

static bool ad4691_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... AD4691_OSC_FREQ_REG:
	case AD4691_SPARE_CONTROL ... AD4691_ACC_SAT_OVR_REG(15):
	case AD4691_STD_SEQ_CONFIG:
	case AD4691_AVG_IN(0) ... AD4691_AVG_IN(15):
	case AD4691_AVG_STS_IN(0) ... AD4691_AVG_STS_IN(15):
	case AD4691_ACC_IN(0) ... AD4691_ACC_IN(15):
	case AD4691_ACC_STS_DATA(0) ... AD4691_ACC_STS_DATA(15):
		return true;
	default:
		return false;
	}
}

static bool ad4691_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0 ... AD4691_OSC_FREQ_REG:
	case AD4691_STD_SEQ_CONFIG:
	case AD4691_SPARE_CONTROL ... AD4691_GPIO_MODE2_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ad4691_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_read = ad4691_reg_read,
	.reg_write = ad4691_reg_write,
	.volatile_reg = ad4691_volatile_reg,
	.readable_reg = ad4691_readable_reg,
	.writeable_reg = ad4691_writeable_reg,
	.max_register = AD4691_ACC_STS_DATA(15),
	.cache_type = REGCACHE_MAPLE,
};

/* Write target_osc_freq_Hz to OSC_FREQ_REG. Called at use time. */
static int ad4691_write_osc_freq(struct ad4691_state *st)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ad4691_osc_freqs_Hz); i++) {
		if (ad4691_osc_freqs_Hz[i] == st->target_osc_freq_Hz)
			return regmap_write(st->regmap, AD4691_OSC_FREQ_REG, i);
	}
	return -EINVAL;
}

/*
 * Index 0 in ad4691_osc_freqs_Hz is 1 MHz — valid only for AD4692/AD4694
 * (max_rate == 1 MHz). AD4691/AD4693 cap at 500 kHz so their valid range
 * starts at index 1.
 */
static unsigned int ad4691_samp_freq_start(const struct ad4691_chip_info *info)
{
	return (info->max_rate == 1 * HZ_PER_MHZ) ? 0 : 1;
}

/*
 * Find the largest oscillator table entry that is both <= needed_osc and
 * evenly divisible by osr (guaranteeing an integer effective rate on
 * read-back). Returns 0 if no such entry exists in the chip's valid range.
 */
static unsigned int ad4691_find_osc_freq(struct ad4691_state *st,
					 unsigned int needed_osc,
					 unsigned int osr)
{
	unsigned int start = ad4691_samp_freq_start(st->info);

	for (unsigned int i = start; i < ARRAY_SIZE(ad4691_osc_freqs_Hz); i++) {
		if ((unsigned int)ad4691_osc_freqs_Hz[i] > needed_osc)
			continue;
		if (ad4691_osc_freqs_Hz[i] % osr != 0)
			continue;
		return ad4691_osc_freqs_Hz[i];
	}
	return 0;
}

static int ad4691_get_sampling_freq(struct ad4691_state *st, u8 osr, int *val)
{
	*val = st->target_osc_freq_Hz / osr;
	return IIO_VAL_INT;
}

static int ad4691_set_sampling_freq(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, int freq)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	unsigned int osr = st->osr[chan->channel];
	unsigned int found;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (freq <= 0 || (unsigned int)freq > st->info->max_rate / osr) {
		iio_device_release_direct_mode(indio_dev);
		return -EINVAL;
	}

	found = ad4691_find_osc_freq(st, (unsigned int)freq * osr, osr);
	if (!found) {
		iio_device_release_direct_mode(indio_dev);
		return -EINVAL;
	}

	/*
	 * Store the snapped oscillator frequency; OSC_FREQ_REG is written at
	 * buffer enable and single-shot time so that sampling_frequency and
	 * oversampling_ratio can be set in any order.
	 */
	st->target_osc_freq_Hz = found;
	iio_device_release_direct_mode(indio_dev);
	return 0;
}

static int ad4691_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type,
			     int *length, long mask)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	unsigned int start = ad4691_samp_freq_start(st->info);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		unsigned int osr = st->osr[chan->channel];
		int n = 0;

		/*
		 * Only oscillator frequencies evenly divisible by the channel's
		 * OSR yield an integer effective rate; expose those as effective
		 * rates (osc / osr) so the user works entirely in output-sample
		 * space.
		 */
		for (unsigned int i = start; i < ARRAY_SIZE(ad4691_osc_freqs_Hz); i++) {
			if (ad4691_osc_freqs_Hz[i] % osr != 0)
				continue;
			st->samp_freq_avail[n++] = ad4691_osc_freqs_Hz[i] / osr;
		}
		*vals = st->samp_freq_avail;
		*type = IIO_VAL_INT;
		*length = n;
		return IIO_AVAIL_LIST;
	}
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = ad4691_oversampling_ratios;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(ad4691_oversampling_ratios);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad4691_single_shot_read(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, int *val)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	unsigned int reg_val, period_us;
	int ret;

	guard(mutex)(&st->lock);

	/* Use AUTONOMOUS mode for single-shot reads. */
	ret = regmap_write(st->regmap, AD4691_STATE_RESET_REG,
			   AD4691_STATE_RESET_ALL);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4691_STD_SEQ_CONFIG,
			   BIT(chan->channel));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4691_ACC_MASK_REG,
			   ~BIT(chan->channel) & GENMASK(15, 0));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4691_ACC_DEPTH_IN(chan->channel),
			   st->osr[chan->channel]);
	if (ret)
		return ret;

	ret = ad4691_write_osc_freq(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4691_OSC_EN_REG, 1);
	if (ret)
		return ret;

	/*
	 * Wait osr + 1 oscillator periods: osr for accumulation, +1 for the
	 * pipeline margin (one extra period ensures the final result is ready).
	 */
	period_us = DIV_ROUND_UP((unsigned long)(st->osr[chan->channel] + 1) * USEC_PER_SEC,
				 st->target_osc_freq_Hz);
	fsleep(period_us);

	ret = regmap_write(st->regmap, AD4691_OSC_EN_REG, 0);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4691_AVG_IN(chan->channel), &reg_val);
	if (ret)
		return ret;

	*val = reg_val;

	ret = regmap_write(st->regmap, AD4691_STATE_RESET_REG, AD4691_STATE_RESET_ALL);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static int ad4691_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4691_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW: {
		int ret;

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad4691_single_shot_read(indio_dev, chan, val);
		iio_device_release_direct_mode(indio_dev);
		return ret;
	}
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4691_get_sampling_freq(st, st->osr[chan->channel], val);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->osr[chan->channel];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_uV / (MICRO / MILLI);
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4691_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct ad4691_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4691_set_sampling_freq(indio_dev, chan, val);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO: {
		int ret;

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = -EINVAL;
		for (unsigned int i = 0; i < ARRAY_SIZE(ad4691_oversampling_ratios); i++) {
			if (ad4691_oversampling_ratios[i] != val)
				continue;
			/*
			 * Store the new OSR; target_osc_freq_Hz is unchanged.
			 * The effective rate read back via in_voltageN_sampling_frequency
			 * becomes target_osc_freq_Hz / new_osr automatically.
			 */
			st->osr[chan->channel] = val;
			ret = 0;
			break;
		}
		iio_device_release_direct_mode(indio_dev);
		return ret;
	}
	default:
		return -EINVAL;
	}
}

static int ad4691_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4691_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4691_set_pwm_freq(struct ad4691_state *st, int freq)
{
	if (!freq)
		return -EINVAL;

	st->cnv_period_ns = DIV_ROUND_UP(NSEC_PER_SEC, freq);
	return 0;
}

static int ad4691_sampling_enable(struct ad4691_state *st, bool enable)
{
	struct pwm_state conv_state = {
		.period     = st->cnv_period_ns,
		.duty_cycle = AD4691_CNV_DUTY_CYCLE_NS,
		.polarity   = PWM_POLARITY_NORMAL,
		.enabled    = enable,
	};

	return pwm_apply_might_sleep(st->conv_trigger, &conv_state);
}

/*
 * ad4691_enter_conversion_mode - Switch the chip to its buffer conversion mode.
 *
 * Configures the ADC hardware registers for the mode selected at probe
 * (CNV_BURST or MANUAL). Called from buffer preenable before starting
 * sampling. The chip is in AUTONOMOUS mode during idle (for read_raw).
 */
static int ad4691_enter_conversion_mode(struct ad4691_state *st)
{
	int ret;

	if (st->manual_mode)
		return regmap_update_bits(st->regmap, AD4691_DEVICE_SETUP,
					  AD4691_MANUAL_MODE, AD4691_MANUAL_MODE);

	ret = ad4691_write_osc_freq(st);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4691_ADC_SETUP,
				 AD4691_ADC_MODE_MASK, AD4691_CNV_BURST_MODE);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4691_STATE_RESET_REG,
			    AD4691_STATE_RESET_ALL);
}

/*
 * ad4691_exit_conversion_mode - Return the chip to AUTONOMOUS mode.
 *
 * Called from buffer postdisable to restore the chip to the
 * idle state used by read_raw. Clears the sequencer and resets state.
 */
static int ad4691_exit_conversion_mode(struct ad4691_state *st)
{
	if (st->manual_mode)
		return regmap_update_bits(st->regmap, AD4691_DEVICE_SETUP,
					  AD4691_MANUAL_MODE, 0);

	return regmap_update_bits(st->regmap, AD4691_ADC_SETUP,
				  AD4691_ADC_MODE_MASK, AD4691_AUTONOMOUS_MODE);
}

static int ad4691_manual_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	unsigned int prev_i, k, i;
	bool first;
	int ret;

	memset(st->scan_xfers, 0, sizeof(st->scan_xfers));
	memset(st->scan_tx, 0, sizeof(st->scan_tx));

	spi_message_init(&st->scan_msg);

	first = true;
	prev_i = 0;
	k = 0;
	iio_for_each_active_channel(indio_dev, i) {
		st->scan_tx[k] = cpu_to_be16(AD4691_ADC_CHAN(i));
		st->scan_xfers[k].tx_buf = &st->scan_tx[k];
		/*
		 * The pipeline means xfer[0] receives the residual from the
		 * previous sequence, not a valid sample for channel i. Point
		 * it at vals[i] anyway; xfer[1] (or the NOOP when only one
		 * channel is active) will overwrite that slot with the real
		 * result, so no separate dummy buffer is needed.
		 */
		if (first) {
			st->scan_xfers[k].rx_buf = &st->vals[i];
			first = false;
		} else {
			st->scan_xfers[k].rx_buf = &st->vals[prev_i];
		}
		st->scan_xfers[k].len = sizeof(__be16);
		st->scan_xfers[k].cs_change = 1;
		spi_message_add_tail(&st->scan_xfers[k], &st->scan_msg);
		prev_i = i;
		k++;
	}

	/* Final NOOP transfer retrieves the last channel's result. */
	st->scan_xfers[k].tx_buf = &st->scan_tx[k]; /* scan_tx[k] == 0 == NOOP */
	st->scan_xfers[k].rx_buf = &st->vals[prev_i];
	st->scan_xfers[k].len = sizeof(__be16);
	spi_message_add_tail(&st->scan_xfers[k], &st->scan_msg);

	ret = spi_optimize_message(st->spi, &st->scan_msg);
	if (ret)
		return ret;

	ret = ad4691_enter_conversion_mode(st);
	if (ret) {
		spi_unoptimize_message(&st->scan_msg);
		return ret;
	}

	return 0;
}

static int ad4691_manual_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad4691_exit_conversion_mode(st);
	spi_unoptimize_message(&st->scan_msg);
	return ret;
}

static const struct iio_buffer_setup_ops ad4691_manual_buffer_setup_ops = {
	.preenable = &ad4691_manual_buffer_preenable,
	.postdisable = &ad4691_manual_buffer_postdisable,
};

static int ad4691_cnv_burst_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	unsigned int acc_mask;
	unsigned int k, i;
	int ret;

	memset(st->scan_xfers, 0, sizeof(st->scan_xfers));
	memset(st->scan_tx, 0, sizeof(st->scan_tx));

	spi_message_init(&st->scan_msg);

	/*
	 * Each AVG_IN read needs two transfers: a 2-byte address write phase
	 * followed by a 2-byte data read phase. CS toggles between channels
	 * (cs_change=1 on the read phase of all but the last channel).
	 */
	k = 0;
	iio_for_each_active_channel(indio_dev, i) {
		st->scan_tx[k] = cpu_to_be16(0x8000 | AD4691_AVG_IN(i));
		st->scan_xfers[2 * k].tx_buf = &st->scan_tx[k];
		st->scan_xfers[2 * k].len = sizeof(__be16);
		spi_message_add_tail(&st->scan_xfers[2 * k], &st->scan_msg);
		st->scan_xfers[2 * k + 1].rx_buf = &st->vals[i];
		st->scan_xfers[2 * k + 1].len = sizeof(__be16);
		st->scan_xfers[2 * k + 1].cs_change = 1;
		spi_message_add_tail(&st->scan_xfers[2 * k + 1], &st->scan_msg);
		k++;
	}

	st->scan_tx[k] = cpu_to_be16(AD4691_STATE_RESET_REG);
	st->scan_xfers[2 * k].tx_buf = &st->scan_tx[k];
	st->scan_xfers[2 * k].len = sizeof(__be16);
	spi_message_add_tail(&st->scan_xfers[2 * k], &st->scan_msg);
	st->scan_tx[k + 1] = cpu_to_be16(AD4691_STATE_RESET_ALL << 8);
	st->scan_xfers[2 * k + 1].tx_buf = &st->scan_tx[k + 1];
	st->scan_xfers[2 * k + 1].len = sizeof(__be16);
	st->scan_xfers[2 * k + 1].cs_change = 1;
	spi_message_add_tail(&st->scan_xfers[2 * k + 1], &st->scan_msg);

	ret = spi_optimize_message(st->spi, &st->scan_msg);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4691_STD_SEQ_CONFIG,
			   bitmap_read(indio_dev->active_scan_mask, 0,
				       iio_get_masklength(indio_dev)));
	if (ret)
		goto err_unoptimize;

	acc_mask = ~bitmap_read(indio_dev->active_scan_mask, 0,
				iio_get_masklength(indio_dev)) & GENMASK(15, 0);
	ret = regmap_write(st->regmap, AD4691_ACC_MASK_REG, acc_mask);
	if (ret)
		goto err_unoptimize;

	iio_for_each_active_channel(indio_dev, i) {
		ret = regmap_write(st->regmap, AD4691_ACC_DEPTH_IN(i), st->osr[i]);
		if (ret)
			goto err_unoptimize;
	}

	ret = ad4691_enter_conversion_mode(st);
	if (ret)
		goto err_unoptimize;

	ret = ad4691_sampling_enable(st, true);
	if (ret)
		goto err_exit_conv;

	enable_irq(st->irq);
	return 0;

err_exit_conv:
	ad4691_exit_conversion_mode(st);
err_unoptimize:
	spi_unoptimize_message(&st->scan_msg);
	return ret;
}

static int ad4691_cnv_burst_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	int ret;

	disable_irq(st->irq);

	ret = ad4691_sampling_enable(st, false);
	if (ret)
		return ret;

	ret = ad4691_exit_conversion_mode(st);
	spi_unoptimize_message(&st->scan_msg);
	return ret;
}

static const struct iio_buffer_setup_ops ad4691_cnv_burst_buffer_setup_ops = {
	.preenable = &ad4691_cnv_burst_buffer_preenable,
	.postdisable = &ad4691_cnv_burst_buffer_postdisable,
};

static int ad4691_manual_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	struct ad4691_offload_state *offload = st->offload;
	struct device *dev = regmap_get_device(st->regmap);
	struct spi_device *spi = to_spi_device(dev);
	struct spi_offload_trigger_config config = {
		.type = SPI_OFFLOAD_TRIGGER_PERIODIC,
	};
	unsigned int bpw = indio_dev->channels[0].scan_type.realbits;
	unsigned int bit, k;
	int ret;

	ret = ad4691_enter_conversion_mode(st);
	if (ret)
		return ret;

	memset(st->scan_xfers, 0, sizeof(st->scan_xfers));

	/*
	 * N+1 transfers for N channels. Each CS-low period triggers
	 * a conversion AND returns the previous result (pipelined).
	 *   TX: [AD4691_ADC_CHAN(n), 0x00]
	 *   RX: [data_hi, data_lo]     (storagebits=16, shift=0)
	 * Transfer 0 RX is garbage; transfers 1..N carry real data.
	 */
	k = 0;
	iio_for_each_active_channel(indio_dev, bit) {
		offload->tx_cmd[k][0] = AD4691_ADC_CHAN(bit);
		st->scan_xfers[k].tx_buf = offload->tx_cmd[k];
		st->scan_xfers[k].len = sizeof(offload->tx_cmd[k]);
		st->scan_xfers[k].bits_per_word = bpw;
		st->scan_xfers[k].cs_change = 1;
		st->scan_xfers[k].cs_change_delay.value = AD4691_CNV_HIGH_TIME_NS;
		st->scan_xfers[k].cs_change_delay.unit = SPI_DELAY_UNIT_NSECS;
		/* First transfer RX is garbage — skip it. */
		if (k > 0)
			st->scan_xfers[k].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
		k++;
	}

	/* Final NOOP to flush pipeline and capture last channel. */
	offload->tx_cmd[k][0] = AD4691_NOOP;
	st->scan_xfers[k].tx_buf = offload->tx_cmd[k];
	st->scan_xfers[k].len = sizeof(offload->tx_cmd[k]);
	st->scan_xfers[k].bits_per_word = bpw;
	st->scan_xfers[k].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	k++;

	spi_message_init_with_transfers(&st->scan_msg, st->scan_xfers, k);
	st->scan_msg.offload = offload->offload;

	ret = spi_optimize_message(spi, &st->scan_msg);
	if (ret)
		goto err_exit_conversion;

	config.periodic.frequency_hz = offload->trigger_hz;
	ret = spi_offload_trigger_enable(offload->offload, offload->trigger, &config);
	if (ret)
		goto err_unoptimize;

	return 0;

err_unoptimize:
	spi_unoptimize_message(&st->scan_msg);
err_exit_conversion:
	ad4691_exit_conversion_mode(st);
	return ret;
}

static int ad4691_manual_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	struct ad4691_offload_state *offload = st->offload;

	spi_offload_trigger_disable(offload->offload, offload->trigger);
	spi_unoptimize_message(&st->scan_msg);

	return ad4691_exit_conversion_mode(st);
}

static const struct iio_buffer_setup_ops ad4691_manual_offload_buffer_setup_ops = {
	.postenable = &ad4691_manual_offload_buffer_postenable,
	.predisable = &ad4691_manual_offload_buffer_predisable,
};

static int ad4691_cnv_burst_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	struct ad4691_offload_state *offload = st->offload;
	struct device *dev = regmap_get_device(st->regmap);
	struct spi_device *spi = to_spi_device(dev);
	struct spi_offload_trigger_config config = {
		.type = SPI_OFFLOAD_TRIGGER_DATA_READY,
	};
	unsigned int bpw = indio_dev->channels[0].scan_type.realbits;
	unsigned int acc_mask;
	unsigned int bit, k;
	int ret;

	ret = regmap_write(st->regmap, AD4691_STD_SEQ_CONFIG,
			   bitmap_read(indio_dev->active_scan_mask, 0,
				       iio_get_masklength(indio_dev)));
	if (ret)
		return ret;

	acc_mask = ~bitmap_read(indio_dev->active_scan_mask, 0,
				iio_get_masklength(indio_dev)) & GENMASK(15, 0);
	ret = regmap_write(st->regmap, AD4691_ACC_MASK_REG, acc_mask);
	if (ret)
		return ret;

	iio_for_each_active_channel(indio_dev, bit) {
		ret = regmap_write(st->regmap, AD4691_ACC_DEPTH_IN(bit), st->osr[bit]);
		if (ret)
			return ret;
	}

	ret = ad4691_enter_conversion_mode(st);
	if (ret)
		return ret;

	memset(st->scan_xfers, 0, sizeof(st->scan_xfers));

	/*
	 * Each AVG_IN register read uses two 16-bit transfers:
	 *   TX: [reg_hi | 0x80, reg_lo]  (address, CS stays asserted)
	 *   RX: [data_hi, data_lo]       (data, storagebits=16, shift=0)
	 * The state reset is also split into two 16-bit transfers
	 * (address then value) to keep bits_per_word uniform throughout.
	 */
	k = 0;
	iio_for_each_active_channel(indio_dev, bit) {
		put_unaligned_be16(0x8000 | AD4691_AVG_IN(bit), offload->tx_cmd[k]);

		/* TX: address phase, CS stays asserted into data phase */
		st->scan_xfers[2 * k].tx_buf = offload->tx_cmd[k];
		st->scan_xfers[2 * k].len = sizeof(offload->tx_cmd[k]);
		st->scan_xfers[2 * k].bits_per_word = bpw;

		/* RX: data phase, CS toggles after to delimit the next register op */
		st->scan_xfers[2 * k + 1].len = sizeof(offload->tx_cmd[k]);
		st->scan_xfers[2 * k + 1].bits_per_word = bpw;
		st->scan_xfers[2 * k + 1].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
		st->scan_xfers[2 * k + 1].cs_change = 1;
		k++;
	}

	/* State reset to re-arm DATA_READY for the next scan. */
	put_unaligned_be16(AD4691_STATE_RESET_REG, offload->tx_reset);
	offload->tx_reset[2] = AD4691_STATE_RESET_ALL;

	st->scan_xfers[2 * k].tx_buf = offload->tx_reset;
	st->scan_xfers[2 * k].len = sizeof(offload->tx_cmd[k]);
	st->scan_xfers[2 * k].bits_per_word = bpw;

	st->scan_xfers[2 * k + 1].tx_buf = &offload->tx_reset[2];
	st->scan_xfers[2 * k + 1].len = sizeof(offload->tx_cmd[k]);
	st->scan_xfers[2 * k + 1].bits_per_word = bpw;
	st->scan_xfers[2 * k + 1].cs_change = 1;

	spi_message_init_with_transfers(&st->scan_msg, st->scan_xfers, 2 * k + 2);
	st->scan_msg.offload = offload->offload;

	ret = spi_optimize_message(spi, &st->scan_msg);
	if (ret)
		goto err_exit_conversion;

	ret = ad4691_sampling_enable(st, true);
	if (ret)
		goto err_unoptimize;

	ret = spi_offload_trigger_enable(offload->offload, offload->trigger, &config);
	if (ret)
		goto err_sampling_disable;

	return 0;

err_sampling_disable:
	ad4691_sampling_enable(st, false);
err_unoptimize:
	spi_unoptimize_message(&st->scan_msg);
err_exit_conversion:
	ad4691_exit_conversion_mode(st);
	return ret;
}

static int ad4691_cnv_burst_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	struct ad4691_offload_state *offload = st->offload;
	int ret;

	spi_offload_trigger_disable(offload->offload, offload->trigger);

	ret = ad4691_sampling_enable(st, false);
	if (ret)
		return ret;

	spi_unoptimize_message(&st->scan_msg);

	return ad4691_exit_conversion_mode(st);
}

static const struct iio_buffer_setup_ops ad4691_cnv_burst_offload_buffer_setup_ops = {
	.postenable = &ad4691_cnv_burst_offload_buffer_postenable,
	.predisable = &ad4691_cnv_burst_offload_buffer_predisable,
};

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4691_state *st = iio_priv(indio_dev);

	if (st->manual_mode && st->offload)
		return sysfs_emit(buf, "%llu\n", st->offload->trigger_hz);

	return sysfs_emit(buf, "%lu\n", NSEC_PER_SEC / st->cnv_period_ns);
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4691_state *st = iio_priv(indio_dev);
	int freq, ret;

	ret = kstrtoint(buf, 10, &freq);
	if (ret)
		return ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (st->manual_mode && st->offload) {
		struct spi_offload_trigger_config config = {
			.type = SPI_OFFLOAD_TRIGGER_PERIODIC,
			.periodic = { .frequency_hz = freq },
		};

		ret = spi_offload_trigger_validate(st->offload->trigger, &config);
		if (ret) {
			iio_device_release_direct_mode(indio_dev);
			return ret;
		}

		st->offload->trigger_hz = config.periodic.frequency_hz;
		iio_device_release_direct_mode(indio_dev);
		return len;
	}

	ret = ad4691_set_pwm_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);
	if (ret)
		return ret;

	return len;
}

static IIO_DEVICE_ATTR_RW(sampling_frequency, 0);

static const struct iio_dev_attr *ad4691_buffer_attrs[] = {
	&iio_dev_attr_sampling_frequency,
	NULL
};

static irqreturn_t ad4691_irq(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ad4691_state *st = iio_priv(indio_dev);

	iio_trigger_poll(indio_dev->trig);
	/*
	 * Keep the DATA_READY IRQ disabled until the trigger handler has
	 * finished reading the scan, to prevent a new assertion mid-transfer.
	 * The PWM continues running uninterrupted; the IRQ is re-enabled in
	 * ad4691_trigger_handler once spi_sync completes.
	 *
	 * IRQF_ONESHOT already masks the hardware line during this threaded
	 * handler, so disable_irq_nosync here ensures the IRQ stays disabled
	 * even after IRQF_ONESHOT unmasks on return.
	 */
	disable_irq_nosync(st->irq);

	return IRQ_HANDLED;
}

static const struct iio_trigger_ops ad4691_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static int ad4691_read_scan(struct iio_dev *indio_dev, s64 timestamp)
{
	struct ad4691_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	ret = spi_sync(st->spi, &st->scan_msg);
	if (ret)
		return ret;

	/*
	 * rx_buf pointers in scan_xfers point directly into scan.vals, so no
	 * copy is needed. The scan_msg already includes a STATE_RESET at the
	 * end (appended in preenable), so no explicit reset is needed here.
	 */
	iio_push_to_buffers_with_timestamp(indio_dev, st->vals, timestamp);
	return 0;
}

static irqreturn_t ad4691_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4691_state *st = iio_priv(indio_dev);

	ad4691_read_scan(indio_dev, pf->timestamp);
	if (!st->manual_mode)
		enable_irq(st->irq);
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static const struct iio_info ad4691_info = {
	.read_raw = &ad4691_read_raw,
	.write_raw = &ad4691_write_raw,
	.read_avail = &ad4691_read_avail,
	.debugfs_reg_access = &ad4691_reg_access,
};

static int ad4691_pwm_setup(struct ad4691_state *st)
{
	struct device *dev = regmap_get_device(st->regmap);

	st->conv_trigger = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->conv_trigger))
		return dev_err_probe(dev, PTR_ERR(st->conv_trigger),
				     "Failed to get cnv pwm\n");

	return ad4691_set_pwm_freq(st, st->info->max_rate);
}

static int ad4691_regulator_setup(struct ad4691_state *st)
{
	struct device *dev = regmap_get_device(st->regmap);
	int ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad4691_supplies),
					     ad4691_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get and enable supplies\n");

	/*
	 * vdd-supply and ldo-in-supply are mutually exclusive:
	 *   vdd-supply present  → external 1.8V VDD; disable internal LDO.
	 *   vdd-supply absent   → enable internal LDO fed from ldo-in-supply.
	 * Having both simultaneously is strongly inadvisable per the datasheet.
	 */
	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret == -ENODEV) {
		ret = devm_regulator_get_enable(dev, "ldo-in");
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to get and enable LDO-IN\n");
		st->ldo_en = true;
	} else if (ret) {
		return dev_err_probe(dev, ret, "Failed to get and enable VDD\n");
	}

	st->vref_uV = devm_regulator_get_enable_read_voltage(dev, "ref");
	if (st->vref_uV == -ENODEV) {
		st->vref_uV = devm_regulator_get_enable_read_voltage(dev, "refin");
		st->refbuf_en = true;
	}
	if (st->vref_uV < 0)
		return dev_err_probe(dev, st->vref_uV,
				     "Failed to get reference supply\n");

	if (st->vref_uV < AD4691_VREF_uV_MIN || st->vref_uV > AD4691_VREF_uV_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "vref(%d) must be in the range [%u...%u]\n",
				     st->vref_uV, AD4691_VREF_uV_MIN,
				     AD4691_VREF_uV_MAX);

	return 0;
}

static int ad4691_reset(struct ad4691_state *st)
{
	struct device *dev = regmap_get_device(st->regmap);
	struct reset_control *rst;

	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "Failed to get reset\n");

	if (rst) {
		/*
		 * reset_gpio_probe() already drives the pin asserted, so the
		 * device is held in reset before we get here.
		 * devm_reset_control_get_optional_exclusive_deasserted() cannot
		 * be used because it deasserts immediately without delay; the
		 * datasheet (Table 5) requires a ≥300 µs reset pulse width
		 * before deassertion.
		 */
		fsleep(300);
		return reset_control_deassert(rst);
	}

	/* No hardware reset available, fall back to software reset. */
	return regmap_write(st->regmap, AD4691_SPI_CONFIG_A_REG,
			    AD4691_SW_RESET);
}

static int ad4691_config(struct ad4691_state *st)
{
	struct device *dev = regmap_get_device(st->regmap);
	enum ad4691_ref_ctrl ref_val;
	unsigned int val;
	int ret;

	/*
	 * Determine buffer conversion mode from DT: if a PWM is provided it
	 * drives the CNV pin (CNV_BURST_MODE); otherwise CNV is tied to CS
	 * and each SPI transfer triggers a conversion (MANUAL_MODE).
	 * Both modes idle in AUTONOMOUS mode so that read_raw can use the
	 * internal oscillator without disturbing the hardware configuration.
	 */
	if (device_property_present(dev, "pwms")) {
		st->manual_mode = false;
		ret = ad4691_pwm_setup(st);
		if (ret)
			return ret;
	} else {
		st->manual_mode = true;
	}

	switch (st->vref_uV) {
	case AD4691_VREF_uV_MIN ... AD4691_VREF_2P5_uV_MAX:
		ref_val = AD4691_VREF_2P5;
		break;
	case AD4691_VREF_2P5_uV_MAX + 1 ... AD4691_VREF_3P0_uV_MAX:
		ref_val = AD4691_VREF_3P0;
		break;
	case AD4691_VREF_3P0_uV_MAX + 1 ... AD4691_VREF_3P3_uV_MAX:
		ref_val = AD4691_VREF_3P3;
		break;
	case AD4691_VREF_3P3_uV_MAX + 1 ... AD4691_VREF_4P096_uV_MAX:
		ref_val = AD4691_VREF_4P096;
		break;
	case AD4691_VREF_4P096_uV_MAX + 1 ... AD4691_VREF_uV_MAX:
		ref_val = AD4691_VREF_5P0;
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "Unsupported vref voltage: %d uV\n",
				     st->vref_uV);
	}

	val = FIELD_PREP(AD4691_REF_CTRL_MASK, ref_val);
	if (st->refbuf_en)
		val |= AD4691_REFBUF_EN;

	ret = regmap_write(st->regmap, AD4691_REF_CTRL, val);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to write REF_CTRL\n");

	ret = regmap_assign_bits(st->regmap, AD4691_DEVICE_SETUP,
				 AD4691_LDO_EN, st->ldo_en);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to write DEVICE_SETUP\n");

	/*
	 * Set the internal oscillator to the highest rate this chip supports.
	 * Index 0 (1 MHz) exceeds the 500 kHz max of AD4691/AD4693, so those
	 * chips start at index 1 (500 kHz).
	 */
	ret = regmap_write(st->regmap, AD4691_OSC_FREQ_REG,
			   ad4691_samp_freq_start(st->info));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to write OSC_FREQ\n");

	st->target_osc_freq_Hz = ad4691_osc_freqs_Hz[ad4691_samp_freq_start(st->info)];

	ret = regmap_update_bits(st->regmap, AD4691_ADC_SETUP,
				 AD4691_ADC_MODE_MASK, AD4691_AUTONOMOUS_MODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to write ADC_SETUP\n");

	return 0;
}

static int ad4691_setup_triggered_buffer(struct iio_dev *indio_dev,
					 struct ad4691_state *st)
{
	struct device *dev = regmap_get_device(st->regmap);
	struct iio_trigger *trig;
	unsigned int i;
	int irq, ret;

	trig = devm_iio_trigger_alloc(dev, "%s-dev%d", indio_dev->name,
				      iio_device_id(indio_dev));
	if (!trig)
		return -ENOMEM;

	trig->ops = &ad4691_trigger_ops;
	iio_trigger_set_drvdata(trig, st);

	ret = devm_iio_trigger_register(dev, trig);
	if (ret)
		return dev_err_probe(dev, ret, "IIO trigger register failed\n");

	indio_dev->trig = iio_trigger_get(trig);

	if (st->manual_mode)
		return devm_iio_triggered_buffer_setup(dev, indio_dev,
						       &iio_pollfunc_store_time,
						       &ad4691_trigger_handler,
						       &ad4691_manual_buffer_setup_ops);

	/*
	 * The GP pin named in interrupt-names asserts at end-of-conversion.
	 * The IRQ handler stops conversions and fires the IIO trigger so
	 * the trigger handler can read and push the sample to the buffer.
	 * The IRQ is kept disabled until the buffer is enabled.
	 */
	irq = -ENXIO;
	for (i = 0; i < ARRAY_SIZE(ad4691_gp_names); i++) {
		irq = fwnode_irq_get_byname(dev_fwnode(dev),
					    ad4691_gp_names[i]);
		if (irq > 0)
			break;
	}
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get GP interrupt\n");

	st->irq = irq;

	ret = ad4691_gpio_setup(st, i);
	if (ret)
		return ret;

	/*
	 * IRQ is kept disabled until the buffer is enabled to prevent
	 * spurious DATA_READY events before the SPI message is set up.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL,
					&ad4691_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					indio_dev->name, indio_dev);
	if (ret)
		return ret;

	return devm_iio_triggered_buffer_setup_ext(dev, indio_dev,
						   &iio_pollfunc_store_time,
						   &ad4691_trigger_handler,
						   IIO_BUFFER_DIRECTION_IN,
						   &ad4691_cnv_burst_buffer_setup_ops,
						   ad4691_buffer_attrs);
}

static int ad4691_setup_offload(struct iio_dev *indio_dev,
				struct ad4691_state *st,
				struct spi_offload *spi_offload)
{
	struct device *dev = regmap_get_device(st->regmap);
	struct ad4691_offload_state *offload;
	struct dma_chan *rx_dma;
	int ret;

	offload = devm_kzalloc(dev, sizeof(*offload), GFP_KERNEL);
	if (!offload)
		return -ENOMEM;

	offload->offload = spi_offload;
	st->offload = offload;

	if (st->manual_mode) {
		offload->trigger =
			devm_spi_offload_trigger_get(dev, offload->offload,
						     SPI_OFFLOAD_TRIGGER_PERIODIC);
		if (IS_ERR(offload->trigger))
			return dev_err_probe(dev, PTR_ERR(offload->trigger),
					     "Failed to get periodic offload trigger\n");

		offload->trigger_hz = AD4691_OFFLOAD_INITIAL_TRIGGER_HZ;
	} else {
		struct spi_offload_trigger_info trigger_info = {
			.fwnode = dev_fwnode(dev),
			.ops    = &ad4691_offload_trigger_ops,
			.priv   = st,
		};

		ret = devm_spi_offload_trigger_register(dev, &trigger_info);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to register offload trigger\n");

		offload->trigger =
			devm_spi_offload_trigger_get(dev, offload->offload,
						     SPI_OFFLOAD_TRIGGER_DATA_READY);
		if (IS_ERR(offload->trigger))
			return dev_err_probe(dev, PTR_ERR(offload->trigger),
					     "Failed to get DATA_READY offload trigger\n");
	}

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, offload->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
				     "Failed to get offload RX DMA channel\n");

	if (st->manual_mode)
		indio_dev->setup_ops = &ad4691_manual_offload_buffer_setup_ops;
	else
		indio_dev->setup_ops = &ad4691_cnv_burst_offload_buffer_setup_ops;

	ret = devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev, rx_dma,
							  IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

	indio_dev->buffer->attrs = ad4691_buffer_attrs;

	return 0;
}

static int ad4691_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct spi_offload *spi_offload;
	struct iio_dev *indio_dev;
	struct ad4691_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->info = spi_get_device_match_data(spi);
	memset(st->osr, 1, sizeof(st->osr));

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(dev, NULL, spi, &ad4691_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	ret = ad4691_regulator_setup(st);
	if (ret)
		return ret;

	ret = ad4691_reset(st);
	if (ret)
		return ret;

	ret = ad4691_config(st);
	if (ret)
		return ret;

	spi_offload = devm_spi_offload_get(dev, spi, &ad4691_offload_config);
	ret = PTR_ERR_OR_ZERO(spi_offload);
	if (ret == -ENODEV)
		spi_offload = NULL;
	else if (ret)
		return dev_err_probe(dev, ret, "Failed to get SPI offload\n");

	indio_dev->name = st->info->name;
	indio_dev->info = &ad4691_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (spi_offload) {
		if (st->manual_mode)
			indio_dev->channels = st->info->offload_info->manual_channels;
		else
			indio_dev->channels = st->info->offload_info->channels;
		indio_dev->num_channels = st->info->offload_info->num_channels;
		ret = ad4691_setup_offload(indio_dev, st, spi_offload);
	} else {
		if (st->manual_mode)
			indio_dev->channels = st->info->sw_info->manual_channels;
		else
			indio_dev->channels = st->info->sw_info->channels;
		indio_dev->num_channels = st->info->sw_info->num_channels;
		ret = ad4691_setup_triggered_buffer(indio_dev, st);
	}
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4691_of_match[] = {
	{ .compatible = "adi,ad4691", .data = &ad4691_chip_info },
	{ .compatible = "adi,ad4692", .data = &ad4692_chip_info },
	{ .compatible = "adi,ad4693", .data = &ad4693_chip_info },
	{ .compatible = "adi,ad4694", .data = &ad4694_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4691_of_match);

static const struct spi_device_id ad4691_id[] = {
	{ "ad4691", (kernel_ulong_t)&ad4691_chip_info },
	{ "ad4692", (kernel_ulong_t)&ad4692_chip_info },
	{ "ad4693", (kernel_ulong_t)&ad4693_chip_info },
	{ "ad4694", (kernel_ulong_t)&ad4694_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4691_id);

static struct spi_driver ad4691_driver = {
	.driver = {
		.name = "ad4691",
		.of_match_table = ad4691_of_match,
	},
	.probe = ad4691_probe,
	.id_table = ad4691_id,
};
module_spi_driver(ad4691_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4691 Family ADC Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMA_BUFFER);
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
