// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR3000, ADAR3001, ADAR3002, ADAR3003 device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADAR3000_REG_SPI_CONFIG			0x00
#define ADAR3000_REG_1				0x01
#define ADAR3000_REG_CHIPTYPE			0x03
#define ADAR3000_REG_PRODUCT_ID_L		0x04
#define ADAR3000_REG_PRODUCT_ID_H		0x05
#define ADAR3000_REG_ADDRESS_PAGE		0x08
#define ADAR3000_REG_SCRATCHPAD			0x0A
#define ADAR3000_REG_SPI_REV			0x0B
#define	ADAR3000_REG_BEAM0_MAP			0x10
#define	ADAR3000_REG_BEAM1_MAP			0x11
#define	ADAR3000_REG_BEAM2_MAP			0x12
#define	ADAR3000_REG_BEAM3_MAP			0x13
#define ADAR3000_REG_BEAMFORMER_MODE		0x14
#define ADAR3000_REG_BEAMSTATE_MODE		0x15
#define ADAR3000_REG_BEAM_SLEEP			0x16
#define ADAR3000_REG_BM0_MEM_SEQPTR_START	0x17
#define ADAR3000_REG_BM0_MEM_SEQPTR_STOP	0x18
#define ADAR3000_REG_BM1_MEM_SEQPTR_START	0x19
#define ADAR3000_REG_BM1_MEM_SEQPTR_STOP	0x1A
#define ADAR3000_REG_BM2_MEM_SEQPTR_START	0x1B
#define ADAR3000_REG_BM2_MEM_SEQPTR_STOP	0x1C
#define ADAR3000_REG_BM3_MEM_SEQPTR_START	0x1D
#define ADAR3000_REG_BM3_MEM_SEQPTR_STOP	0x1E
#define ADAR3000_REG_ADC_CONTROL		0x20
#define ADAR3000_REG_ADC_CONTROL2		0x21
#define ADAR3000_REG_ADC_DATA_OUT		0x22
#define ADAR3000_REG_DAC_DATA_MSB		0x23
#define ADAR3000_REG_DAC_DATA_LSB		0x24
#define ADAR3000_REG_DAC_CONTROL		0x25
#define ADAR3000_REG_BM0_FIFO_WRITE_POINTER	0x50
#define ADAR3000_REG_BM0_FIFO_READ_POINTER	0x51
#define ADAR3000_REG_BM1_FIFO_WRITE_POINTER	0x52
#define ADAR3000_REG_BM1_FIFO_READ_POINTER	0x53
#define ADAR3000_REG_BM2_FIFO_WRITE_POINTER	0x54
#define ADAR3000_REG_BM2_FIFO_READ_POINTER	0x55
#define ADAR3000_REG_BM3_FIFO_WRITE_POINTER	0x56
#define ADAR3000_REG_BM3_FIFO_READ_POINTER	0x57

/* Beam state Reset for BEAM0 - BEAM3 */
#define ADAR3000_REG_RESET_DELAY_EL0(x)		(0x80 + (x << 3))
#define ADAR3000_REG_RESET_ATTN_EL0(x)		(0x81 + (x << 3))
#define ADAR3000_REG_RESET_DELAY_EL1(x)		(0x82 + (x << 3))
#define ADAR3000_REG_RESET_ATTN_EL1(x)		(0x83 + (x << 3))
#define ADAR3000_REG_RESET_DELAY_EL2(x)		(0x84 + (x << 3))
#define ADAR3000_REG_RESET_ATTN_EL2(x)		(0x85 + (x << 3))
#define ADAR3000_REG_RESET_DELAY_EL3(x)		(0x86 + (x << 3))
#define ADAR3000_REG_RESET_ATTN_EL3(x)		(0x87 + (x << 3))

/* Beam state Mute for BEAM0 - BEAM3 */
#define ADAR3000_REG_MUTE_DELAY_BM0_EL0		(0xA0 + (x << 3))
#define ADAR3000_REG_MUTE_ATTN_BM0_EL0		(0xA1 + (x << 3))
#define ADAR3000_REG_MUTE_DELAY_BM0_EL1		(0xA2 + (x << 3))
#define ADAR3000_REG_MUTE_ATTN_BM0_EL1		(0xA3 + (x << 3))
#define ADAR3000_REG_MUTE_DELAY_BM0_EL2		(0xA4 + (x << 3))
#define ADAR3000_REG_MUTE_ATTN_BM0_EL2		(0xA5 + (x << 3))
#define ADAR3000_REG_MUTE_DELAY_BM0_EL3		(0xA6 + (x << 3))
#define ADAR3000_REG_MUTE_ATTN_BM0_EL3		(0xA7 + (x << 3))

#define ADAR3000_REG_RESET_BM0_AMP		0xC0
#define ADAR3000_REG_RESET_BM1_AMP		0xC1
#define ADAR3000_REG_RESET_BM2_AMP		0xC2
#define ADAR3000_REG_RESET_BM3_AMP		0xC3

#define ADAR3000_REG_OPERATIONAL_BM0_AMP	0xC4
#define ADAR3000_REG_OPERATIONAL_BM1_AMP	0xC5
#define ADAR3000_REG_OPERATIONAL_BM2_AMP	0xC6
#define ADAR3000_REG_OPERATIONAL_BM3_AMP	0xC7

#define ADAR3000_REG_MUTE_BM0_AMP		0xC8
#define ADAR3000_REG_MUTE_BM1_AMP		0xC9
#define ADAR3000_REG_MUTE_BM2_AMP		0xCA
#define ADAR3000_REG_MUTE_BM3_AMP		0xCB

#define ADAR3000_REG_SLEEP_BM0_AMP		0xCC
#define ADAR3000_REG_SLEEP_BM1_AMP		0xCD
#define ADAR3000_REG_SLEEP_BM2_AMP		0xCE
#define ADAR3000_REG_SLEEP_BM3_AMP		0xCF

#define ADAR3000_REG_OPERATIONAL_EL0_AMP	0xD0
#define ADAR3000_REG_OPERATIONAL_EL1_AMP	0xD1
#define ADAR3000_REG_OPERATIONAL_EL2_AMP	0xD2
#define ADAR3000_REG_OPERATIONAL_EL3_AMP	0xD3

#define ADAR3000_REG_SLEEP_EL0_AMP		0xD4
#define ADAR3000_REG_SLEEP_EL1_AMP		0xD5
#define ADAR3000_REG_SLEEP_EL2_AMP		0xD6
#define ADAR3000_REG_SLEEP_EL3_AMP		0xD7

/* ADAR3000/3001 direct control registers  BEAM0 to BEAM3 */
#define ADAR3000_REG_DRCT_CNTRL_DELAY_EL0(x)	(0x100 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_ATTN_EL0(x)	(0x101 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_DELAY_EL1(x)	(0x102 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_ATTN_EL1(x)	(0x103 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_DELAY_EL2(x)	(0x104 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_ATTN_EL2(x)	(0x105 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_DELAY_EL3(x)	(0x106 + (x << 3))
#define ADAR3000_REG_DRCT_CNTRL_ATTN_EL3(x)	(0x107 + (x << 3))

/* ADAR3002 direct control registers H/V BEAM0 to BEAM1 */
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0H(x)	(0x100 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0H(x)	(0x101 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1H(x)	(0x102 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1H(x)	(0x103 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2H(x)	(0x104 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2H(x)	(0x105 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3H(x)	(0x106 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3H(x)	(0x107 + (x) * (0x03 << 3))

#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0V(x)	(0x108 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0V(x)	(0x109 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1V(x)	(0x10A + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1V(x)	(0x10B + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2V(x)	(0x10C + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2V(x)	(0x10D + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3V(x)	(0x10E + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3V(x)	(0x10F + (x) * 0x08)

/* ADAR3003 direct control registers only one BEAM from EL0 to EL3 */
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELV(x)	(0x100 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELV(x)	(0x101 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELH(x)	(0x102 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELH(x)	(0x103 + (x) * 0x08)


#define ADAR3000_REG_BM0_SEQ_PTR(x)	(0x200 + x)
#define ADAR3000_REG_BM1_SEQ_PTR(x)	(0x240 + x)
#define ADAR3000_REG_BM2_SEQ_PTR(x)	(0x280 + x)
#define ADAR3000_REG_BM3_SEQ_PTR(x)	(0x2C0 + x)

#define ADAR3000_REG_RESET_BM0_STREAM_IN(x)	(0x300 + x)
#define ADAR3000_REG_RESET_BM1_STREAM_IN(x)	(0x306 + x)
#define ADAR3000_REG_RESET_BM2_STREAM_IN(x)	(0x30C + x)
#define ADAR3000_REG_RESET_BM3_STREAM_IN(x)	(0x312 + x)

#define ADAR3000_REG_MUTE_BM0_STREAM_IN(x)	(0x318 + x)
#define ADAR3000_REG_MUTE_BM1_STREAM_IN(x)	(0x31E + x)
#define ADAR3000_REG_MUTE_BM2_STREAM_IN(x)	(0x324 + x)
#define ADAR3000_REG_MUTE_BM3_STREAM_IN(x)	(0x32A + x)

#define ADAR3000_REG_DRCT_CNTRL_BM0_STREAM_IN(x)	(0x330 + x)
#define ADAR3000_REG_DRCT_CNTRL_BM1_STREAM_IN(x)	(0x336 + x)
#define ADAR3000_REG_DRCT_CNTRL_BM2_STREAM_IN(x)	(0x33C + x)
#define ADAR3000_REG_DRCT_CNTRL_BM3_STREAM_IN(x)	(0x342 + x)

/* Beam statei RAM
 * state - beam state number
 * pos - position in memory location 0 - 63
 */
#define ADAR3000_RAM_BEAM_STATE(state, pos)		(0x100 + (state * 6) + pos)

/* Beam state FIFO
 * state - beam state number
 * pos - position in memory location 0 - 15
 */
#define ADAR3000_FIFO_BEAM_STATE(state, pos)		(0x100 + (state * 0x10) + pos)

/* ADAR3000_REG_SPI_CONFIG */
#define ADAR3000_SPI_CONFIG_RESET_	BIT(7)
#define ADAR3000_SPI_CONFIG_BIG_ENDIAN_	BIT(5)
#define ADAR3000_SPI_CONFIG_SDOACTIVE_	BIT(4)
#define ADAR3000_SPI_CONFIG_SDOACTIVE	BIT(3)
#define ADAR3000_SPI_CONFIG_BIG_ENDIAN	BIT(2)
#define ADAR3000_SPI_CONFIG_RESET	BIT(0)

/* ADAR3000_REG_BEAMSTATE_MODE */
#define ADAR3000_BM0_MODE	0x03
#define ADAR3000_BM1_MODE	0x0C
#define ADAR3000_BM2_MODE	0x30
#define ADAR3000_BM3_MODE	0xC0

#define ADAR3000_SPI_ADDR_MSK		GENMASK(13, 10)
#define ADAR3000_SPI_ADDR(x)		FIELD_PREP(ADAR3000_SPI_ADDR_MSK, x)

#define ADAR3000_MAX_DEV	16

enum adar3000_ids {
	ID_ADAR3000,
	ID_ADAR3002,
	ID_ADAR3003,
};

/* Beam state composed of 48 bits ADAR3000/3001/3002
 * For ADAR3003 only 23 bits are used */
struct adar3000_beam_state {
	u8 delay0 : 6;
	u8 atten0 : 6;
	u8 delay1 : 6;
	u8 atten1 : 6;
	u8 delay2 : 6;
	u8 atten2 : 6;
	u8 delay3 : 6;
	u8 atten3 : 6;
};

struct adar3000_chip_info {
	unsigned int			num_channels;
	const struct iio_chan_spec	*channels;
};

struct adar3000_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	const struct adar3000_chip_info	*chip_info;
	u8				dev_addr;
	struct adar3000_beam_state	*beam_st;
};

static const struct regmap_config adar3000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int adar3000_reg_read(struct adar3000_state *st, u32 reg, u32 *val)
{
	return regmap_read(st->regmap, st->dev_addr | reg, val);
}

static int adar3000_reg_write(struct adar3000_state *st, u32 reg, u32 val)
{
	return regmap_write(st->regmap, st->dev_addr | reg, val);
}

static int adar3000_reg_update(struct adar3000_state *st, u32 reg, u32 mask, u32 val)
{
	int ret;
	u32 readval;

	ret = adar3000_reg_read(st, reg, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	readval |= val;

	return adar3000_reg_write(st, reg, val);
}

static int adar3000_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct adar3000_state *st = iio_priv(indio_dev);

	if (readval)
		return adar3000_reg_read(st, reg, readval);
	else
		return adar3000_reg_write(st, reg, writeval);
}

static int adar3000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	// struct adar3000_state *st = iio_priv(indio_dev);
	// int ret, ch;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static int adar3000_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	// struct adar3000_state *st = iio_priv(indio_dev);
	// u32 code;
	// int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_PHASE:
	default:
		return -EINVAL;
	};
}

static int adar3000_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

#define ADAR3000_CHANNEL(_num, name)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
	.extend_name = name,					\
}

/* Maybe ditch the entire channel to element mapping and just use the
 * beamstates as a reference attribute. Suggestion may include a new type of
 * raw value similar go hardwaregain, phase -> beamstate */
#define DECLARE_ADAR3000_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM0_EL0"),			\
	ADAR3000_CHANNEL(1, "BEAM0_EL1"),			\
	ADAR3000_CHANNEL(2, "BEAM0_EL2"),			\
	ADAR3000_CHANNEL(3, "BEAM0_EL3"),			\
	ADAR3000_CHANNEL(4, "BEAM1_EL0"),			\
	ADAR3000_CHANNEL(5, "BEAM1_EL1"),			\
	ADAR3000_CHANNEL(6, "BEAM1_EL2"),			\
	ADAR3000_CHANNEL(7, "BEAM1_EL3"),			\
	ADAR3000_CHANNEL(8, "BEAM2_EL0"),			\
	ADAR3000_CHANNEL(9, "BEAM2_EL1"),			\
	ADAR3000_CHANNEL(10, "BEAM2_EL2"),			\
	ADAR3000_CHANNEL(11, "BEAM2_EL3"),			\
	ADAR3000_CHANNEL(12, "BEAM3_EL0"),			\
	ADAR3000_CHANNEL(13, "BEAM3_EL1"),			\
	ADAR3000_CHANNEL(14, "BEAM3_EL2"),			\
	ADAR3000_CHANNEL(15, "BEAM3_EL3"),			\
};

#define DECLARE_ADAR3002_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM0_EL0H"),			\
	ADAR3000_CHANNEL(1, "BEAM0_EL1H"),			\
	ADAR3000_CHANNEL(2, "BEAM0_EL2H"),			\
	ADAR3000_CHANNEL(3, "BEAM0_EL3H"),			\
	ADAR3000_CHANNEL(4, "BEAM0_EL0V"),			\
	ADAR3000_CHANNEL(5, "BEAM0_EL1V"),			\
	ADAR3000_CHANNEL(6, "BEAM0_EL2V"),			\
	ADAR3000_CHANNEL(7, "BEAM0_EL3V"),			\
	ADAR3000_CHANNEL(8, "BEAM1_EL0V"),			\
	ADAR3000_CHANNEL(9, "BEAM1_EL1V"),			\
	ADAR3000_CHANNEL(10, "BEAM1_EL2V"),			\
	ADAR3000_CHANNEL(11, "BEAM1_EL3V"),			\
	ADAR3000_CHANNEL(12, "BEAM1_EL0H"),			\
	ADAR3000_CHANNEL(13, "BEAM1_EL1H"),			\
	ADAR3000_CHANNEL(14, "BEAM1_EL2H"),			\
	ADAR3000_CHANNEL(15, "BEAM1_EL3H"),			\
};

#define DECLARE_ADAR3003_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM_EL0V"),			\
	ADAR3000_CHANNEL(1, "BEAM_EL0H"),			\
	ADAR3000_CHANNEL(2, "BEAM_EL1V"),			\
	ADAR3000_CHANNEL(3, "BEAM_EL1H"),			\
	ADAR3000_CHANNEL(4, "BEAM_EL2V"),			\
	ADAR3000_CHANNEL(5, "BEAM_EL2H"),			\
	ADAR3000_CHANNEL(6, "BEAM_EL3V"),			\
	ADAR3000_CHANNEL(7, "BEAM_EL3H"),			\
};

DECLARE_ADAR3000_CHANNELS(adar3000_channels);
DECLARE_ADAR3002_CHANNELS(adar3002_channels);
DECLARE_ADAR3003_CHANNELS(adar3003_channels);

static const struct adar3000_chip_info adar3000_chip_info_tbl[] = {
	[ID_ADAR3000] = {
		.channels = adar3000_channels,
		.num_channels = 16,
	},
	[ID_ADAR3002] = {
		.channels = adar3002_channels,
		.num_channels = 16,
	},
	[ID_ADAR3003] = {
		.channels = adar3003_channels,
		.num_channels = 8,
	},
};

enum adar3000_beamstate_mode_ctrl {
	ADAR3000_DIRECT_CTRL,
	ADAR3000_MEMORY_CTRL,
	ADAR3000_FIFO_CTRL,
	ADAR3000_INST_DIRECT_CTRL
};

static const char *const adar3000_mode_ctrl[] = {
	[ADAR3000_DIRECT_CTRL] = "direct",
	[ADAR3000_MEMORY_CTRL] = "memory",
	[ADAR3000_FIFO_CTRL] = "fifo",
	[ADAR3000_INST_DIRECT_CTRL] = "instant_direct"
};

enum adar1000_iio_dev_attr {
	ADAR3000_MODE_CTRL,
};

/* The beam mode is develop with beamstate as the central point
 * For this to work beamstate are used for IIO channels. This proposal seems
 * to make more sense */
static ssize_t adar3000_beam_mode_available(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	// struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adar3000_mode_ctrl); ++i) {
		if (adar3000_mode_ctrl[i])
			len += sprintf(buf + len, "%s ", adar3000_mode_ctrl[i]);
	}
	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t adar3000_beam_mode_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adar3000_state *st = iio_device_get_drvdata(indio_dev);
	u32 readval;
	int ret;

	ret = adar3000_reg_read(st, ADAR3000_REG_BEAMSTATE_MODE, &readval);
	if (ret <0)
		return ret;

	readval = FIELD_GET(ADAR3000_BM0_MODE << chan->address, readval);

	return sprintf(buf, "%s\n", adar3000_mode_ctrl[readval]);
}

static ssize_t adar3000_beam_mode_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adar3000_state *st = iio_device_get_drvdata(indio_dev);
	unsigned int mode = 0, i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(adar3000_mode_ctrl); ++i) {
		if (adar3000_mode_ctrl[i] && sysfs_streq(buf, adar3000_mode_ctrl[i])) {
			mode = i;
			break;
		}
	}

	ret = adar3000_reg_update(st, ADAR3000_REG_BEAMSTATE_MODE,
				  ADAR3000_BM0_MODE << chan->address, mode); 

	return ret ? ret : len;
}

// static struct iio_chan_spec_ext_info adar3000_ext_info[] = {
// 	{
// 	 .name = "mode_ctrl",
// 	 .read = adar3000_beam_mode_read,
// 	 .write = adar3000_beam_mode_write,
// 	 .shared = IIO_SHARED_BY_TYPE,
// 	 },
// 	{
// 	 .name = "mode_ctrl_available",
// 	 .read = adar3000_beam_mode_available,
// 	 .shared = IIO_SHARED_BY_TYPE,
// 	 },
// 	{},
// };

static const struct iio_info adar3000_info = {
	.read_raw = &adar3000_read_raw,
	.write_raw = &adar3000_write_raw,
	.write_raw_get_fmt = &adar3000_write_raw_get_fmt,
	.debugfs_reg_access = &adar3000_reg_access,
};


static int adar3000_setup(struct iio_dev *indio_dev)
{
	struct adar3000_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret;
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	/* Software reset and activate SDO */
	
	ret = regmap_write(st->regmap, 0x0A, 0xAD);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, 0x0A, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0xAD) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}

	ret = regmap_write(st->regmap, 0x0A, 0x5A);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, 0x0A, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0x5A) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}
	dev_err(indio_dev->dev.parent, "ADMV4420 to read/write scratchpad %x ", val);
	
	return 0;
}

static int adar3000_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adar3000_state *st;
	struct regmap *regmap;
	const struct adar3000_chip_info *info;
	int ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);	
	regmap = devm_regmap_init_spi(spi, &adar3000_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error  ADMV4420 initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	
	info = of_device_get_match_data(&spi->dev);
	if (!info)
		return -ENODEV;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = info;
	st->regmap = regmap;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "admv4420";
	indio_dev->info = &adar3000_info;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = adar3000_setup(indio_dev);

	if (ret < 0) {
		dev_err(&spi->dev, "Setup ADMV4420 failed (%d)\n", ret);
		return ret;
	}
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	return devm_iio_device_register(&spi->dev, indio_dev);
}

// static int adar3000_probe(struct spi_device *spi)
// {
// 	struct iio_dev *indio_dev;
// 	struct adar3000_state *st;
// 	const struct adar3000_chip_info *info;
// 	struct regmap *regmap;
// 	int ret;
// 	// u32 tmp;

// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

// 	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
// 	if (!indio_dev)
// 		return -ENOMEM;

// 	regmap = devm_regmap_init_spi(spi, &adar3000_regmap_config);
// 	if (IS_ERR(regmap)) {
// 		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
// 			PTR_ERR(regmap));
// 		return PTR_ERR(regmap);
// 	}
// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	info = of_device_get_match_data(&spi->dev);
// 	if (!info)
// 		return -ENODEV;
// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	st = iio_priv(indio_dev);
// 	st->chip_info = info;
// 	spi_set_drvdata(spi, indio_dev);
// 	st->spi = spi;
// 	st->regmap = regmap;
// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->dev.parent = &spi->dev;
// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->name = "";
// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->info = &adar3000_info;
// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->modes = INDIO_DIRECT_MODE;
// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->channels = st->chip_info->channels;
// 	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	indio_dev->num_channels = st->chip_info->num_channels;

// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

// 	/* Do setup for each device */
// 	ret = adar3000_setup(indio_dev);
// 	if (ret < 0) {
// 		dev_err(&spi->dev, "Setup failed (%d)\n", ret);
// 		return ret;
// 	}
// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
// 	ret = devm_iio_device_register(&spi->dev, indio_dev);
// 	if (ret < 0)
// 		return ret;

// pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

// 	return 0;
// }

static const struct of_device_id adar3000_of_match[] = {
	{ .compatible = "adi,admv4420",
		.data = &adar3000_chip_info_tbl[ID_ADAR3000], },
	{ }
};
MODULE_DEVICE_TABLE(of, adar3000_of_match);

static struct spi_driver adar3000_driver = {
	.driver = {
		.name	= "admv4420",
		.of_match_table = adar3000_of_match,
	},
	.probe		= adar3000_probe,
};
module_spi_driver(adar3000_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADMV44200 K Band Downconverter");
MODULE_LICENSE("GPL v2");
