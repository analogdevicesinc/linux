// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADMV1355 17.7 GHz to 55 GHz Wideband Microwave Upconverter
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/device-id/of.h>
#include <linux/device-id/spi.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kstrtox.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include <linux/iio/iio.h>

#define ADMV1355_REG_SPI_CONFIG		0x000
#define ADMV1355_REG_PRODUCT_ID_L	0x004
#define ADMV1355_REG_PRODUCT_ID_H	0x005
#define ADMV1355_REG_SCRATCH_PAD	0x00A
#define ADMV1355_REG_NVM_OSC_CONFIG	0x078
#define ADMV1355_REG_NVM_READ_ADDR	0x07B
#define ADMV1355_REG_NVM_READ_DATA	0x07C
#define ADMV1355_REG_NVM_REFRESH	0x07E
#define ADMV1355_REG_POWER_DOWN		0x13E
#define ADMV1355_REG_FILTER_LUT_EN	0x202
#define ADMV1355_REG_FILTER_LOAD_EN	0x208
#define ADMV1355_REG_GAIN_LUT_BYPASS	0x28A
#define ADMV1355_REG_DSA_BYPASS		0x28B
#define ADMV1355_REG_RF_LPF_BYPASS	0x2A0
#define ADMV1355_REG_RF_HPF_BYPASS	0x2A1
#define ADMV1355_REG_ADC_MUX		0x2A7
#define ADMV1355_REG_ADC_CTRL		0x300
#define ADMV1355_REG_ADC_LON_CTRL	0x302
#define ADMV1355_REG_LON_SELECT		0x305
#define ADMV1355_REG_LON_TAP		0x306
#define ADMV1355_REG_ADC_OUT		0x30D
#define ADMV1355_REG_IF_ENABLE		0x602
#define ADMV1355_REG_MIXER_GATE		0x700
#define ADMV1355_REG_BB_SW_NMOS		0x702
#define ADMV1355_REG_BB_SW_PMOS		0x703
#define ADMV1355_REG_LO_X4_FILTER	0x800
#define ADMV1355_REG_LO_X3_FILTER	0x801
#define ADMV1355_REG_LO_X1_FILTER	0x802
#define ADMV1355_REG_LO_TRAP_L		0x804
#define ADMV1355_REG_RF_BAND_PHASE_Q	0x805
#define ADMV1355_REG_LO_BAND_PHASE_I	0x806
#define ADMV1355_REG_LON_OFFSET_I	0x808
#define ADMV1355_REG_LON_OFFSET_Q	0x809
#define ADMV1355_REG_IF_DSA		0x80A
#define ADMV1355_REG_LO_DOUBLER_BAND	0x80C

/* ADMV1355_REG_SPI_CONFIG: each field is mirrored in the upper nibble */
#define ADMV1355_SOFT_RESET_MSK		(BIT(7) | BIT(0))
#define ADMV1355_SDO_ACTIVE_MSK		(BIT(4) | BIT(3))

/* ADMV1355_REG_NVM_OSC_CONFIG / ADMV1355_REG_NVM_REFRESH */
#define ADMV1355_NVM_OSC_CFG_LOAD	0x08
#define ADMV1355_NVM_OSC_CFG_DONE	0x09
#define ADMV1355_NVM_REFRESH_STEP1	0x40
#define ADMV1355_NVM_REFRESH_STEP2	0x54

/* ADMV1355_REG_POWER_DOWN */
#define ADMV1355_RF_HS_PD_MSK		BIT(1)
#define ADMV1355_CHIP_PD_MSK		BIT(0)

/* ADMV1355_REG_GAIN_LUT_BYPASS */
#define ADMV1355_GAIN_LUT_BYPASS_EN_MSK	BIT(0)

/* ADMV1355_REG_DSA_BYPASS */
#define ADMV1355_DSA1_MSK		GENMASK(3, 0)
#define ADMV1355_DSA2_MSK		GENMASK(7, 4)

/* ADMV1355_REG_RF_LPF_BYPASS / ADMV1355_REG_RF_HPF_BYPASS */
#define ADMV1355_LPF_SELECT_MSK		BIT(4)
#define ADMV1355_HPF_SELECT_MSK		BIT(7)
#define ADMV1355_HPF_BYPASS_VALUE_MSK	GENMASK(5, 0)

/* ADMV1355_REG_ADC_MUX */
#define ADMV1355_ADC_MUX_TEMP		0x01

/* ADMV1355_REG_ADC_CTRL */
#define ADMV1355_ADC_EN_MSK		BIT(3)
#define ADMV1355_ADC_CLK_EN_MSK		BIT(0)

/* ADMV1355_REG_ADC_LON_CTRL */
#define ADMV1355_LON_AUTO_EN_MSK	BIT(2)
#define ADMV1355_ADC_RESET_MSK		BIT(1)

/* ADMV1355_REG_LON_SELECT */
#define ADMV1355_LON_EN_MSK		BIT(5)
#define ADMV1355_LON_Q_SELECT_MSK	BIT(1)
#define ADMV1355_LON_I_SELECT_MSK	BIT(0)

/* ADMV1355_REG_LON_TAP */
#define ADMV1355_LON_RESET_MSK		BIT(1)

/* ADMV1355_REG_IF_ENABLE */
#define ADMV1355_IF_HYBRID_EN_MSK	BIT(3)
#define ADMV1355_IF_HIGH_FREQ_EN_MSK	BIT(2)
#define ADMV1355_IF_LOW_FREQ_EN_MSK	BIT(1)
#define ADMV1355_BB_EN_MSK		BIT(0)
#define ADMV1355_IF_COMPLEX_EN_MSK	(ADMV1355_IF_HIGH_FREQ_EN_MSK | \
					 ADMV1355_IF_LOW_FREQ_EN_MSK)
#define ADMV1355_INPUT_EN_MSK		GENMASK(3, 0)

/* ADMV1355_REG_MIXER_GATE */
#define ADMV1355_MIXER_GATE_SELECT_MSK	BIT(5)
#define ADMV1355_MIXER_GATE_MSK		GENMASK(4, 0)

/* ADMV1355_REG_BB_SW_NMOS / ADMV1355_REG_BB_SW_PMOS */
#define ADMV1355_BB_SW_SELECT_MSK	BIT(5)
#define ADMV1355_BB_SW_MSK		GENMASK(4, 0)

/* ADMV1355_REG_RF_BAND_PHASE_Q */
#define ADMV1355_RF_BAND_SELECT_MSK	BIT(7)
#define ADMV1355_LO_PHASE_Q_MSK		GENMASK(6, 2)
#define ADMV1355_LO_TRAP_H_MSK		GENMASK(1, 0)

/* ADMV1355_REG_LO_BAND_PHASE_I */
#define ADMV1355_LO_BAND_MSK		BIT(7)
#define ADMV1355_MIXER_BAND_MSK		BIT(6)
#define ADMV1355_LO_SIDEBAND_MSK	BIT(5)
#define ADMV1355_LO_PHASE_I_MSK		GENMASK(4, 0)

/* ADMV1355_REG_IF_DSA */
#define ADMV1355_DSAQ_MSK		GENMASK(7, 4)
#define ADMV1355_DSAI_MSK		GENMASK(3, 0)

/* SPI frame header: R/W bit, broadcast bit, chip address, register address */
#define ADMV1355_SPI_READ		BIT(15)
#define ADMV1355_SPI_CHIP_ADDR_MSK	GENMASK(13, 12)
#define ADMV1355_SPI_ADDR_MSK		GENMASK(11, 0)
#define ADMV1355_SPI_CHIP_ADDR_MAX	3

#define ADMV1355_SCRATCH_TEST_VAL	0xA5

/* RF DSA1/DSA2: 0 dB to -15 dB in 1 dB steps */
#define ADMV1355_DSA_STEP_UDB		MICRO
/* RF attenuation range of DSA1 and DSA2 combined */
#define ADMV1355_RF_ATTEN_MAX_DB	30
/* IF DSAI/DSAQ: 0 dB to -1.5 dB in 0.1 dB steps */
#define ADMV1355_IF_DSA_STEP_UDB	(MICRO / 10)

/* Temperature in milli degrees Celsius: 173000 - 5000 / 6 * ADC_CODE */
#define ADMV1355_TEMP_OFFSET_MDEG	173000
#define ADMV1355_TEMP_SCALE_NUM		5000
#define ADMV1355_TEMP_SCALE_DEN		6

/* Baseband input common-mode voltage */
#define ADMV1355_BB_VICM_MAX_UV		1550000
#define ADMV1355_BB_VICM_MANUAL_MAX_UV	500000
#define ADMV1355_BB_VICM_STEP_UV	50000

#define ADMV1355_LO_FREQ_MIN_HZ		(8850ULL * HZ_PER_MHZ)
#define ADMV1355_LO_FREQ_MAX_HZ		(27500ULL * HZ_PER_MHZ)

enum admv1355_band {
	ADMV1355_BAND_LOW,
	ADMV1355_BAND_HIGH,
};

enum admv1355_chan_idx {
	ADMV1355_CHAN_RF,
	ADMV1355_CHAN_BB,
	ADMV1355_CHAN_IF_COMPLEX,
	ADMV1355_CHAN_IF_HYBRID,
	ADMV1355_CHAN_LO,
	ADMV1355_CHAN_LO_I,
	ADMV1355_CHAN_LO_Q,
	ADMV1355_CHAN_IF_I,
	ADMV1355_CHAN_IF_Q,
	ADMV1355_CHAN_TEMP,
};

static const char * const admv1355_chan_labels[] = {
	[ADMV1355_CHAN_RF] = "rf",
	[ADMV1355_CHAN_BB] = "baseband",
	[ADMV1355_CHAN_IF_COMPLEX] = "if_complex",
	[ADMV1355_CHAN_IF_HYBRID] = "if_hybrid",
	[ADMV1355_CHAN_LO] = "lo",
	[ADMV1355_CHAN_LO_I] = "lo_i",
	[ADMV1355_CHAN_LO_Q] = "lo_q",
	[ADMV1355_CHAN_IF_I] = "if_i",
	[ADMV1355_CHAN_IF_Q] = "if_q",
	[ADMV1355_CHAN_TEMP] = "temp",
};

static const char * const admv1355_supply_names[] = {
	"dvdd", "vdd-bg", "vdd-lo", "vdd-mix", "vdd-vga", "vdd-drv",
};

struct admv1355_lo_filter {
	u32 max_mhz;
	u16 trap;
	u8 x4;
	u8 x3;
	u8 x1;
	u8 doubler;
};

/*
 * Narrow-band LO tunable filter settings, using the first entry with LO
 * rate < max_mhz. See datasheet Rev. A Table 17.
 */
static const struct admv1355_lo_filter admv1355_lo_filters_narrow[] = {
	{ 10000, 0x318, 0x08, 0x1C, 0x1F, 0x1F },
	{ 12000, 0x318, 0x00, 0x1C, 0x1F, 0x1D },
	{ 14000, 0x3AA, 0x00, 0x07, 0x0A, 0x04 },
	{ 14500, 0x3AA, 0x05, 0x07, 0x0A, 0x08 },
	{ 15000, 0x1A3, 0x05, 0x07, 0x0A, 0x08 },
	{ 17000, 0x1A3, 0x05, 0x05, 0x05, 0x08 },
	{ 18000, 0x1A3, 0x05, 0x04, 0x01, 0x08 },
	{ 18500, 0x1A3, 0x07, 0x04, 0x01, 0x80 },
	{ 19000, 0x1A7, 0x07, 0x04, 0x01, 0x80 },
	{ 21000, 0x1A7, 0x07, 0x03, 0x01, 0x80 },
	{ 24000, 0x1E7, 0x07, 0x02, 0x00, 0x80 },
	{ U32_MAX, 0x1E7, 0x07, 0x00, 0x00, 0xC0 },
};

struct admv1355_lo_filter_table {
	const struct admv1355_lo_filter *filters;
	unsigned int num_filters;
	/* LO band selection threshold */
	u32 band_high_mhz;
};

static const struct admv1355_lo_filter_table admv1355_lo_filter_table_narrow = {
	.filters = admv1355_lo_filters_narrow,
	.num_filters = ARRAY_SIZE(admv1355_lo_filters_narrow),
	.band_high_mhz = 18000,
};

struct admv1355_mixer_gate {
	u8 gate;
	u8 nmos;
	u8 pmos;
};

/*
 * Manual mixer gate bias for the baseband input, needed for a common-mode
 * voltage of 0.5 V or less, indexed by VICM in 50 mV steps. See datasheet
 * Rev. A Table 19.
 */
static const struct admv1355_mixer_gate admv1355_bb_mixer_gates[] = {
	{ 0x02, 0x0A, 0x00 },	/* 0 V */
	{ 0x03, 0x0B, 0x00 },	/* 0.05 V */
	{ 0x04, 0x0B, 0x00 },	/* 0.1 V */
	{ 0x05, 0x0C, 0x00 },	/* 0.15 V */
	{ 0x06, 0x0C, 0x00 },	/* 0.2 V */
	{ 0x07, 0x0D, 0x00 },	/* 0.25 V */
	{ 0x08, 0x0D, 0x00 },	/* 0.3 V */
	{ 0x09, 0x0E, 0x00 },	/* 0.35 V */
	{ 0x0A, 0x0E, 0x00 },	/* 0.4 V */
	{ 0x0B, 0x0F, 0x00 },	/* 0.45 V */
	{ 0x0C, 0x0F, 0x00 },	/* 0.5 V */
};

struct admv1355_state {
	struct spi_device *spi;
	struct regmap *regmap;
	/*
	 * LO input rate, updated by the clock notifier. The notifier runs with
	 * the clock prepare lock held and takes @lock, so do not call
	 * clk_get_rate() with @lock held.
	 */
	unsigned long lo_rate;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;
	/* Manual mixer gate bias for the baseband input, NULL if automatic */
	const struct admv1355_mixer_gate *bb_mixer_gate;
	struct notifier_block nb;
	/* Serializes multi-register sequences and read-modify-write cycles */
	struct mutex lock;
	/* Complex IF path (enum admv1355_band) for when the input gets enabled */
	u8 if_complex_band;
	/* Chip address set by the CHIP_ADD0 and CHIP_ADD1 balls */
	u8 chip_addr;
	bool half_duplex;
	u8 buf[3] __aligned(IIO_DMA_MINALIGN);
};

/*
 * A register access is a 24-bit frame: a 16-bit header followed by 8 bits of
 * data. Full-duplex links clock the read data in on the last byte of a single
 * transfer. Half-duplex links (e.g. 3-wire) need a separate receive transfer.
 */
static int admv1355_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct admv1355_state *st = context;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->buf,
			.len = 2,
		}, {
			.rx_buf = &st->buf[2],
			.len = 1,
		},
	};
	struct spi_transfer fd_xfer = {
		.tx_buf = st->buf,
		.rx_buf = st->buf,
		.len = sizeof(st->buf),
	};
	int ret;

	put_unaligned_be16(ADMV1355_SPI_READ |
			   FIELD_PREP(ADMV1355_SPI_CHIP_ADDR_MSK, st->chip_addr) |
			   FIELD_PREP(ADMV1355_SPI_ADDR_MSK, reg), st->buf);
	st->buf[2] = 0;

	if (st->half_duplex)
		ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	else
		ret = spi_sync_transfer(st->spi, &fd_xfer, 1);
	if (ret)
		return ret;

	*val = st->buf[2];

	return 0;
}

static int admv1355_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct admv1355_state *st = context;

	put_unaligned_be16(FIELD_PREP(ADMV1355_SPI_CHIP_ADDR_MSK, st->chip_addr) |
			   FIELD_PREP(ADMV1355_SPI_ADDR_MSK, reg), st->buf);
	st->buf[2] = val;

	return spi_write(st->spi, st->buf, sizeof(st->buf));
}

static const struct regmap_config admv1355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xE85,
	.reg_read = admv1355_reg_read,
	.reg_write = admv1355_reg_write,
};

/* ADC configuration for the temperature sensor, see datasheet Rev. A p. 76 */
static const struct reg_sequence admv1355_temp_start_seq[] = {
	{ ADMV1355_REG_ADC_LON_CTRL, ADMV1355_ADC_RESET_MSK },
	{ ADMV1355_REG_ADC_CTRL, ADMV1355_ADC_EN_MSK | ADMV1355_ADC_CLK_EN_MSK },
	{ ADMV1355_REG_LON_TAP, ADMV1355_LON_RESET_MSK },
	{ ADMV1355_REG_POWER_DOWN, 0x00 },
	{ ADMV1355_REG_ADC_MUX, ADMV1355_ADC_MUX_TEMP },
	{ ADMV1355_REG_ADC_LON_CTRL,
	  ADMV1355_LON_AUTO_EN_MSK | ADMV1355_ADC_RESET_MSK, 1000 },
};

static const unsigned int admv1355_temp_saved_regs[] = {
	ADMV1355_REG_ADC_LON_CTRL,
	ADMV1355_REG_ADC_CTRL,
	ADMV1355_REG_LON_TAP,
	ADMV1355_REG_POWER_DOWN,
	ADMV1355_REG_ADC_MUX,
};

static int admv1355_read_temp(struct admv1355_state *st, int *val)
{
	unsigned int saved[ARRAY_SIZE(admv1355_temp_saved_regs)];
	unsigned int code, i;
	int ret, ret2 = 0;

	guard(mutex)(&st->lock);

	for (i = 0; i < ARRAY_SIZE(admv1355_temp_saved_regs); i++) {
		ret = regmap_read(st->regmap, admv1355_temp_saved_regs[i],
				  &saved[i]);
		if (ret)
			return ret;
	}

	ret = regmap_multi_reg_write(st->regmap, admv1355_temp_start_seq,
				     ARRAY_SIZE(admv1355_temp_start_seq));
	if (!ret)
		ret = regmap_read(st->regmap, ADMV1355_REG_ADC_OUT, &code);

	/* Restore the previous state even if the conversion failed */
	for (i = 0; i < ARRAY_SIZE(admv1355_temp_saved_regs) && !ret2; i++)
		ret2 = regmap_write(st->regmap, admv1355_temp_saved_regs[i],
				    saved[i]);
	if (ret)
		return ret;
	if (ret2)
		return ret2;

	/* T = 173 - 5/6 * ADC_CODE, in degrees Celsius */
	*val = ADMV1355_TEMP_OFFSET_MDEG -
	       (int)DIV_ROUND_CLOSEST(code * ADMV1355_TEMP_SCALE_NUM,
				      ADMV1355_TEMP_SCALE_DEN);

	return IIO_VAL_INT;
}

static bool admv1355_lo_rate_valid(u64 rate)
{
	return rate >= ADMV1355_LO_FREQ_MIN_HZ && rate <= ADMV1355_LO_FREQ_MAX_HZ;
}

static const struct admv1355_lo_filter *
admv1355_find_lo_filter(const struct admv1355_lo_filter_table *table,
			u32 rate_mhz)
{
	unsigned int i;

	for (i = 0; i < table->num_filters - 1; i++)
		if (rate_mhz < table->filters[i].max_mhz)
			break;

	return &table->filters[i];
}

static int admv1355_set_lo_filters(struct admv1355_state *st,
				   unsigned long rate)
{
	const struct admv1355_lo_filter_table *table = &admv1355_lo_filter_table_narrow;
	u32 rate_mhz = div_u64(rate, HZ_PER_MHZ);
	const struct admv1355_lo_filter *f = admv1355_find_lo_filter(table, rate_mhz);
	const struct reg_sequence seq[] = {
		{ ADMV1355_REG_LO_X4_FILTER, f->x4 },
		{ ADMV1355_REG_LO_X3_FILTER, f->x3 },
		{ ADMV1355_REG_LO_X1_FILTER, f->x1 },
		{ ADMV1355_REG_LO_TRAP_L, f->trap & 0xFF },
		{ ADMV1355_REG_LO_DOUBLER_BAND, f->doubler },
	};
	int ret;

	ret = regmap_multi_reg_write(st->regmap, seq, ARRAY_SIZE(seq));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADMV1355_REG_RF_BAND_PHASE_Q,
				 ADMV1355_LO_TRAP_H_MSK,
				 FIELD_PREP(ADMV1355_LO_TRAP_H_MSK, f->trap >> 8));
	if (ret)
		return ret;

	return regmap_assign_bits(st->regmap, ADMV1355_REG_LO_BAND_PHASE_I,
				  ADMV1355_LO_BAND_MSK,
				  rate_mhz >= table->band_high_mhz);
}

static const char * const admv1355_bands[] = {
	[ADMV1355_BAND_LOW] = "low",
	[ADMV1355_BAND_HIGH] = "high",
};

static int admv1355_get_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_RF_BAND_PHASE_Q, &val);
	if (ret)
		return ret;

	return FIELD_GET(ADMV1355_RF_BAND_SELECT_MSK, val);
}

static int admv1355_set_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int band)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	/* The mixer band must follow the selected RF signal chain */
	ret = regmap_assign_bits(st->regmap, ADMV1355_REG_RF_BAND_PHASE_Q,
				 ADMV1355_RF_BAND_SELECT_MSK, band);
	if (ret)
		return ret;

	return regmap_assign_bits(st->regmap, ADMV1355_REG_LO_BAND_PHASE_I,
				  ADMV1355_MIXER_BAND_MSK, band);
}

static const struct iio_enum admv1355_rf_band_enum = {
	.items = admv1355_bands,
	.num_items = ARRAY_SIZE(admv1355_bands),
	.get = admv1355_get_rf_band,
	.set = admv1355_set_rf_band,
};

static unsigned int admv1355_if_complex_msk(enum admv1355_band band)
{
	return band == ADMV1355_BAND_HIGH ? ADMV1355_IF_HIGH_FREQ_EN_MSK :
					    ADMV1355_IF_LOW_FREQ_EN_MSK;
}

static int admv1355_get_if_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return st->if_complex_band;
}

static int admv1355_set_if_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int band)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, ADMV1355_REG_IF_ENABLE, &val);
	if (ret)
		return ret;

	/* Switch paths right away if the complex IF input is enabled */
	if (val & ADMV1355_IF_COMPLEX_EN_MSK) {
		ret = regmap_update_bits(st->regmap, ADMV1355_REG_IF_ENABLE,
					 ADMV1355_IF_COMPLEX_EN_MSK,
					 admv1355_if_complex_msk(band));
		if (ret)
			return ret;
	}

	st->if_complex_band = band;

	return 0;
}

static const struct iio_enum admv1355_if_band_enum = {
	.items = admv1355_bands,
	.num_items = ARRAY_SIZE(admv1355_bands),
	.get = admv1355_get_if_band,
	.set = admv1355_set_if_band,
};

static const char * const admv1355_sidebands[] = {
	"lower",
	"upper",
};

static int admv1355_get_sideband(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_LO_BAND_PHASE_I, &val);
	if (ret)
		return ret;

	return FIELD_GET(ADMV1355_LO_SIDEBAND_MSK, val);
}

static int admv1355_set_sideband(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int sideband)
{
	struct admv1355_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return regmap_assign_bits(st->regmap, ADMV1355_REG_LO_BAND_PHASE_I,
				  ADMV1355_LO_SIDEBAND_MSK, sideband);
}

static const struct iio_enum admv1355_sideband_enum = {
	.items = admv1355_sidebands,
	.num_items = ARRAY_SIZE(admv1355_sidebands),
	.get = admv1355_get_sideband,
	.set = admv1355_set_sideband,
};

static int admv1355_calibphase_field(const struct iio_chan_spec *chan,
				     unsigned int *reg, unsigned int *mask)
{
	switch (chan->address) {
	case ADMV1355_CHAN_LO_I:
		*reg = ADMV1355_REG_LO_BAND_PHASE_I;
		*mask = ADMV1355_LO_PHASE_I_MSK;
		return 0;
	case ADMV1355_CHAN_LO_Q:
		*reg = ADMV1355_REG_RF_BAND_PHASE_Q;
		*mask = ADMV1355_LO_PHASE_Q_MSK;
		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t admv1355_read_calibphase(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					char *buf)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int reg, mask, val;
	int ret;

	ret = admv1355_calibphase_field(chan, &reg, &mask);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, reg, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", field_get(mask, val));
}

static ssize_t admv1355_write_calibphase(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 const char *buf, size_t len)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int reg, mask, val;
	int ret;

	ret = admv1355_calibphase_field(chan, &reg, &mask);
	if (ret)
		return ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val > field_get(mask, mask))
		return -EINVAL;

	guard(mutex)(&st->lock);

	ret = regmap_update_bits(st->regmap, reg, mask, field_prep(mask, val));
	if (ret)
		return ret;

	return len;
}

static const struct iio_chan_spec_ext_info admv1355_rf_ext_info[] = {
	IIO_ENUM("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM_AVAILABLE("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_if_complex_ext_info[] = {
	IIO_ENUM("band", IIO_SEPARATE, &admv1355_if_band_enum),
	IIO_ENUM_AVAILABLE("band", IIO_SEPARATE, &admv1355_if_band_enum),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_lo_ext_info[] = {
	IIO_ENUM("sideband", IIO_SEPARATE, &admv1355_sideband_enum),
	IIO_ENUM_AVAILABLE("sideband", IIO_SEPARATE, &admv1355_sideband_enum),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_lo_iq_ext_info[] = {
	{
		.name = "calibphase",
		.shared = IIO_SEPARATE,
		.read = admv1355_read_calibphase,
		.write = admv1355_write_calibphase,
	},
	{ }
};

#define ADMV1355_INPUT_CHAN(_idx, _channel, _ext_info) {		\
	.type = IIO_ALTVOLTAGE,						\
	.indexed = 1,							\
	.channel = _channel,						\
	.address = _idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),		\
	.ext_info = _ext_info,						\
}

#define ADMV1355_LO_IQ_CHAN(_idx, _mod) {				\
	.type = IIO_ALTVOLTAGE,						\
	.indexed = 1,							\
	.modified = 1,							\
	.channel = 3,							\
	.channel2 = _mod,						\
	.address = _idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBBIAS),		\
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_CALIBBIAS),	\
	.ext_info = admv1355_lo_iq_ext_info,				\
	.parent = &admv1355_channels[ADMV1355_CHAN_LO],			\
}

#define ADMV1355_IF_IQ_CHAN(_idx, _mod) {				\
	.type = IIO_ALTVOLTAGE,						\
	.indexed = 1,							\
	.modified = 1,							\
	.channel = 1,							\
	.channel2 = _mod,						\
	.address = _idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_HARDWAREGAIN),\
	.parent = &admv1355_channels[ADMV1355_CHAN_IF_COMPLEX],		\
}

static const struct iio_chan_spec admv1355_channels[] = {
	[ADMV1355_CHAN_RF] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.address = ADMV1355_CHAN_RF,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.ext_info = admv1355_rf_ext_info,
	},
	[ADMV1355_CHAN_BB] = ADMV1355_INPUT_CHAN(ADMV1355_CHAN_BB, 0, NULL),
	[ADMV1355_CHAN_IF_COMPLEX] = ADMV1355_INPUT_CHAN(ADMV1355_CHAN_IF_COMPLEX, 1,
							 admv1355_if_complex_ext_info),
	[ADMV1355_CHAN_IF_HYBRID] = ADMV1355_INPUT_CHAN(ADMV1355_CHAN_IF_HYBRID, 2, NULL),
	[ADMV1355_CHAN_LO] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 3,
		.address = ADMV1355_CHAN_LO,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY),
		.ext_info = admv1355_lo_ext_info,
	},
	[ADMV1355_CHAN_LO_I] = ADMV1355_LO_IQ_CHAN(ADMV1355_CHAN_LO_I, IIO_MOD_I),
	[ADMV1355_CHAN_LO_Q] = ADMV1355_LO_IQ_CHAN(ADMV1355_CHAN_LO_Q, IIO_MOD_Q),
	[ADMV1355_CHAN_IF_I] = ADMV1355_IF_IQ_CHAN(ADMV1355_CHAN_IF_I, IIO_MOD_I),
	[ADMV1355_CHAN_IF_Q] = ADMV1355_IF_IQ_CHAN(ADMV1355_CHAN_IF_Q, IIO_MOD_Q),
	[ADMV1355_CHAN_TEMP] = {
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.address = ADMV1355_CHAN_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int admv1355_if_gain_field(const struct iio_chan_spec *chan,
				  unsigned int *reg, unsigned int *mask)
{
	switch (chan->address) {
	case ADMV1355_CHAN_IF_I:
		*reg = ADMV1355_REG_IF_DSA;
		*mask = ADMV1355_DSAI_MSK;
		return 0;
	case ADMV1355_CHAN_IF_Q:
		*reg = ADMV1355_REG_IF_DSA;
		*mask = ADMV1355_DSAQ_MSK;
		return 0;
	default:
		return -EINVAL;
	}
}

/* Attenuation in micro dB, from a gain in dB between -max_db and 0 */
static int admv1355_gain_to_atten(int val, int val2, int max_db,
				  int *atten_udb)
{
	/* Reject values far outside of the range before scaling */
	if (val > 0 || val < -max_db)
		return -EINVAL;

	if (val < 0)
		*atten_udb = -val * (int)MICRO + abs(val2);
	else
		*atten_udb = -val2;

	if (*atten_udb < 0)
		return -EINVAL;

	return 0;
}

/*
 * The RF attenuators DSA1 and DSA2, in this order in the RF signal path, are
 * set as a single gain, following the datasheet RF gain policy: attenuate with
 * DSA1 first, then DSA2, to optimize the output P1dB and IP3.
 */
static int admv1355_read_rf_gain(struct admv1355_state *st, int *val,
				 int *val2)
{
	unsigned int data;
	int ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_DSA_BYPASS, &data);
	if (ret)
		return ret;

	*val = -(int)(FIELD_GET(ADMV1355_DSA1_MSK, data) +
		      FIELD_GET(ADMV1355_DSA2_MSK, data));
	*val2 = 0;

	return IIO_VAL_INT_PLUS_MICRO_DB;
}

static int admv1355_write_rf_gain(struct admv1355_state *st, int val,
				  int val2)
{
	unsigned int atten, dsa1;
	int ret, atten_udb;

	ret = admv1355_gain_to_atten(val, val2, ADMV1355_RF_ATTEN_MAX_DB,
				     &atten_udb);
	if (ret)
		return ret;

	atten = DIV_ROUND_CLOSEST(atten_udb, ADMV1355_DSA_STEP_UDB);
	if (atten > ADMV1355_RF_ATTEN_MAX_DB)
		return -EINVAL;

	dsa1 = min_t(unsigned int, atten, FIELD_MAX(ADMV1355_DSA1_MSK));

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, ADMV1355_REG_DSA_BYPASS,
				  ADMV1355_DSA1_MSK | ADMV1355_DSA2_MSK,
				  FIELD_PREP(ADMV1355_DSA1_MSK, dsa1) |
				  FIELD_PREP(ADMV1355_DSA2_MSK, atten - dsa1));
}

static int admv1355_read_if_gain(struct admv1355_state *st,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2)
{
	unsigned int reg, mask, data;
	int ret, gain_udb;

	ret = admv1355_if_gain_field(chan, &reg, &mask);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, reg, &data);
	if (ret)
		return ret;

	/* Each code step is an attenuation, so the gain is negative */
	gain_udb = -(int)(field_get(mask, data) * ADMV1355_IF_DSA_STEP_UDB);
	*val = gain_udb / (int)MICRO;
	*val2 = gain_udb % (int)MICRO;

	return IIO_VAL_INT_PLUS_MICRO_DB;
}

static int admv1355_write_if_gain(struct admv1355_state *st,
				  const struct iio_chan_spec *chan,
				  int val, int val2)
{
	unsigned int reg, mask, code;
	int ret, atten_udb;

	ret = admv1355_if_gain_field(chan, &reg, &mask);
	if (ret)
		return ret;

	ret = admv1355_gain_to_atten(val, val2, 1, &atten_udb);
	if (ret)
		return ret;

	code = DIV_ROUND_CLOSEST(atten_udb, ADMV1355_IF_DSA_STEP_UDB);
	if (code > field_get(mask, mask))
		return -EINVAL;

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, reg, mask, field_prep(mask, code));
}

static int admv1355_read_enable(struct admv1355_state *st,
				const struct iio_chan_spec *chan, int *val)
{
	unsigned int data;
	int ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_IF_ENABLE, &data);
	if (ret)
		return ret;

	switch (chan->address) {
	case ADMV1355_CHAN_BB:
		*val = !!(data & ADMV1355_BB_EN_MSK);
		return IIO_VAL_INT;
	case ADMV1355_CHAN_IF_COMPLEX:
		*val = !!(data & ADMV1355_IF_COMPLEX_EN_MSK);
		return IIO_VAL_INT;
	case ADMV1355_CHAN_IF_HYBRID:
		*val = !!(data & ADMV1355_IF_HYBRID_EN_MSK);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/*
 * The input enables are one-hot: enabling an input disables the other ones.
 * Disabling an input leaves all of them disabled.
 */
static int admv1355_set_manual_mixer_gate(struct admv1355_state *st,
					  const struct admv1355_mixer_gate *mg)
{
	const struct reg_sequence seq[] = {
		{ ADMV1355_REG_MIXER_GATE, ADMV1355_MIXER_GATE_SELECT_MSK |
		  FIELD_PREP(ADMV1355_MIXER_GATE_MSK, mg->gate) },
		{ ADMV1355_REG_BB_SW_NMOS, ADMV1355_BB_SW_SELECT_MSK |
		  FIELD_PREP(ADMV1355_BB_SW_MSK, mg->nmos) },
		{ ADMV1355_REG_BB_SW_PMOS, FIELD_PREP(ADMV1355_BB_SW_MSK, mg->pmos) },
	};

	return regmap_multi_reg_write(st->regmap, seq, ARRAY_SIZE(seq));
}

/*
 * The mixer gate bias is shared by all inputs, so the manual baseband bias
 * only applies while the baseband input is enabled.
 */
static int admv1355_set_mixer_gate(struct admv1355_state *st, bool baseband)
{
	int ret;

	if (!st->bb_mixer_gate)
		return 0;

	if (baseband)
		return admv1355_set_manual_mixer_gate(st, st->bb_mixer_gate);

	ret = regmap_clear_bits(st->regmap, ADMV1355_REG_MIXER_GATE,
				ADMV1355_MIXER_GATE_SELECT_MSK);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, ADMV1355_REG_BB_SW_NMOS,
				 ADMV1355_BB_SW_SELECT_MSK);
}

static int admv1355_write_enable(struct admv1355_state *st,
				 const struct iio_chan_spec *chan, int val)
{
	bool baseband = chan->address == ADMV1355_CHAN_BB;
	unsigned int mask;
	int ret;

	if (val != 0 && val != 1)
		return -EINVAL;

	guard(mutex)(&st->lock);

	/*
	 * The manual baseband mixer gate bias only applies with baseband on,
	 * which disabling another input does not change.
	 */
	if (val || baseband) {
		ret = admv1355_set_mixer_gate(st, baseband && val);
		if (ret)
			return ret;
	}

	switch (chan->address) {
	case ADMV1355_CHAN_BB:
		mask = ADMV1355_BB_EN_MSK;
		break;
	case ADMV1355_CHAN_IF_COMPLEX:
		mask = admv1355_if_complex_msk(st->if_complex_band);
		break;
	case ADMV1355_CHAN_IF_HYBRID:
		mask = ADMV1355_IF_HYBRID_EN_MSK;
		break;
	default:
		return -EINVAL;
	}

	if (val)
		return regmap_update_bits(st->regmap, ADMV1355_REG_IF_ENABLE,
					  ADMV1355_INPUT_EN_MSK, mask);

	if (chan->address == ADMV1355_CHAN_IF_COMPLEX)
		mask = ADMV1355_IF_COMPLEX_EN_MSK;

	return regmap_clear_bits(st->regmap, ADMV1355_REG_IF_ENABLE, mask);
}

static int admv1355_read_calibbias(struct admv1355_state *st,
				   const struct iio_chan_spec *chan,
				   unsigned int *val)
{
	unsigned int reg = chan->address == ADMV1355_CHAN_LO_I ?
			   ADMV1355_REG_LON_OFFSET_I : ADMV1355_REG_LON_OFFSET_Q;

	return regmap_read(st->regmap, reg, val);
}

static int admv1355_read_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int *val, int *val2, long mask)
{
	struct admv1355_state *st = iio_priv(indio_dev);
	unsigned int data;
	u64 rate;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		return admv1355_read_temp(st, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->address == ADMV1355_CHAN_RF)
			return admv1355_read_rf_gain(st, val, val2);

		return admv1355_read_if_gain(st, chan, val, val2);
	case IIO_CHAN_INFO_ENABLE:
		return admv1355_read_enable(st, chan, val);
	case IIO_CHAN_INFO_FREQUENCY:
		scoped_guard(mutex, &st->lock)
			rate = st->lo_rate;

		*val = lower_32_bits(rate);
		*val2 = upper_32_bits(rate);
		return IIO_VAL_INT_64;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = admv1355_read_calibbias(st, chan, &data);
		if (ret)
			return ret;

		*val = sign_extend32(data, 7);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int admv1355_write_calibbias(struct admv1355_state *st,
				    const struct iio_chan_spec *chan, int val)
{
	unsigned int reg = chan->address == ADMV1355_CHAN_LO_I ?
			   ADMV1355_REG_LON_OFFSET_I : ADMV1355_REG_LON_OFFSET_Q;
	int ret;

	if (val < S8_MIN || val > S8_MAX)
		return -EINVAL;

	guard(mutex)(&st->lock);

	ret = regmap_write(st->regmap, reg, (u8)val);
	if (ret)
		return ret;

	/*
	 * Manual LO nulling, see datasheet Rev. A p. 72: disable the loop and
	 * take both I/Q offsets from the registers, so that they match their
	 * calibbias attributes.
	 */
	return regmap_update_bits(st->regmap, ADMV1355_REG_LON_SELECT,
				  ADMV1355_LON_EN_MSK |
				  ADMV1355_LON_I_SELECT_MSK |
				  ADMV1355_LON_Q_SELECT_MSK,
				  ADMV1355_LON_I_SELECT_MSK |
				  ADMV1355_LON_Q_SELECT_MSK);
}

static int admv1355_write_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int val, int val2, long mask)
{
	struct admv1355_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->address == ADMV1355_CHAN_RF)
			return admv1355_write_rf_gain(st, val, val2);

		return admv1355_write_if_gain(st, chan, val, val2);
	case IIO_CHAN_INFO_ENABLE:
		return admv1355_write_enable(st, chan, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		return admv1355_write_calibbias(st, chan, val);
	default:
		return -EINVAL;
	}
}

static int admv1355_write_raw_get_fmt(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_ENABLE:
	case IIO_CHAN_INFO_CALIBBIAS:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/* [min, step, max] */
static const int admv1355_rf_gain_avail[] = { -30, 0, 1, 0, 0, 0 };
static const int admv1355_if_dsa_gain_avail[] = { -1, -500000, 0, 100000, 0, 0 };
static const int admv1355_calibbias_avail[] = { S8_MIN, 1, S8_MAX };

static int admv1355_read_avail(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       const int **vals, int *type, int *length,
			       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->address == ADMV1355_CHAN_RF) {
			*vals = admv1355_rf_gain_avail;
			*length = ARRAY_SIZE(admv1355_rf_gain_avail);
		} else {
			*vals = admv1355_if_dsa_gain_avail;
			*length = ARRAY_SIZE(admv1355_if_dsa_gain_avail);
		}
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_CALIBBIAS:
		*vals = admv1355_calibbias_avail;
		*length = ARRAY_SIZE(admv1355_calibbias_avail);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int admv1355_read_label(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan, char *label)
{
	return sysfs_emit(label, "%s\n", admv1355_chan_labels[chan->address]);
}

static int admv1355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			       unsigned int writeval, unsigned int *readval)
{
	struct admv1355_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info admv1355_info = {
	.read_raw = admv1355_read_raw,
	.write_raw = admv1355_write_raw,
	.write_raw_get_fmt = admv1355_write_raw_get_fmt,
	.read_avail = admv1355_read_avail,
	.read_label = admv1355_read_label,
	.debugfs_reg_access = admv1355_reg_access,
};

static int admv1355_lo_clk_notifier(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	struct admv1355_state *st = container_of(nb, struct admv1355_state, nb);
	struct clk_notifier_data *cnd = data;

	switch (action) {
	case PRE_RATE_CHANGE:
		return admv1355_lo_rate_valid(cnd->new_rate) ? NOTIFY_OK :
							       NOTIFY_BAD;
	case POST_RATE_CHANGE: {
		guard(mutex)(&st->lock);

		st->lo_rate = cnd->new_rate;
		return notifier_from_errno(admv1355_set_lo_filters(st,
								   cnd->new_rate));
	}
	default:
		return NOTIFY_DONE;
	}
}

static int admv1355_check_spi(struct admv1355_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int val;
	int ret;

	ret = regmap_write(st->regmap, ADMV1355_REG_SCRATCH_PAD,
			   ADMV1355_SCRATCH_TEST_VAL);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_SCRATCH_PAD, &val);
	if (ret)
		return ret;

	if (val != ADMV1355_SCRATCH_TEST_VAL)
		return dev_err_probe(dev, -EIO,
				     "scratchpad mismatch: wrote 0x%02x, read 0x%02x\n",
				     ADMV1355_SCRATCH_TEST_VAL, val);

	return 0;
}

/* NVM health check, see datasheet Rev. A p. 66 */
static int admv1355_nvm_loaded(struct admv1355_state *st, bool *loaded)
{
	static const u8 shadow_regs[] = { 0x33, 0x34, 0x37, 0x38 };
	struct device *dev = &st->spi->dev;
	unsigned int val, i;
	int ret;

	*loaded = false;

	for (i = 0; i < ARRAY_SIZE(shadow_regs); i++) {
		ret = regmap_write(st->regmap, ADMV1355_REG_NVM_READ_ADDR,
				   shadow_regs[i]);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, ADMV1355_REG_NVM_READ_ADDR, &val);
		if (ret)
			return ret;

		if (val != shadow_regs[i])
			return dev_err_probe(dev, -EIO,
					     "NVM address 0x%02x readback mismatch: 0x%02x\n",
					     shadow_regs[i], val);

		ret = regmap_read(st->regmap, ADMV1355_REG_NVM_READ_DATA, &val);
		if (ret)
			return ret;

		if (!val)
			return 0;
	}

	*loaded = true;

	return 0;
}

static const struct reg_sequence admv1355_nvm_load_seq[] = {
	{ ADMV1355_REG_NVM_OSC_CONFIG, ADMV1355_NVM_OSC_CFG_LOAD, 1000 },
	{ ADMV1355_REG_NVM_REFRESH, ADMV1355_NVM_REFRESH_STEP1, 1000 },
	{ ADMV1355_REG_NVM_REFRESH, ADMV1355_NVM_REFRESH_STEP2, 1000 },
	{ ADMV1355_REG_NVM_OSC_CONFIG, ADMV1355_NVM_OSC_CFG_DONE },
};

static int admv1355_nvm_init(struct admv1355_state *st)
{
	struct device *dev = &st->spi->dev;
	bool loaded;
	int ret;

	ret = admv1355_nvm_loaded(st, &loaded);
	if (ret || loaded)
		return ret;

	ret = regmap_multi_reg_write(st->regmap, admv1355_nvm_load_seq,
				     ARRAY_SIZE(admv1355_nvm_load_seq));
	if (ret)
		return ret;

	ret = admv1355_nvm_loaded(st, &loaded);
	if (ret)
		return ret;

	if (!loaded)
		return dev_err_probe(dev, -EIO, "NVM calibration not loaded\n");

	return 0;
}

/* Minimum settings from datasheet Rev. A Tables 20 and 21 */
static const struct reg_sequence admv1355_init_seq[] = {
	{ ADMV1355_REG_POWER_DOWN, ADMV1355_RF_HS_PD_MSK },
	{ ADMV1355_REG_FILTER_LUT_EN, 0x00 },
	{ ADMV1355_REG_FILTER_LOAD_EN, 0x00 },
	{ ADMV1355_REG_RF_LPF_BYPASS, ADMV1355_LPF_SELECT_MSK },
	{ ADMV1355_REG_RF_HPF_BYPASS, ADMV1355_HPF_SELECT_MSK |
				      ADMV1355_HPF_BYPASS_VALUE_MSK },
	{ ADMV1355_REG_GAIN_LUT_BYPASS, ADMV1355_GAIN_LUT_BYPASS_EN_MSK },
	{ ADMV1355_REG_DSA_BYPASS, 0x00 },
	{ ADMV1355_REG_IF_DSA, 0x00 },
};

static int admv1355_setup(struct admv1355_state *st)
{
	unsigned int val, id_l, id_h;
	int ret;

	/* Soft reset self-clears */
	ret = regmap_write(st->regmap, ADMV1355_REG_SPI_CONFIG,
			   ADMV1355_SOFT_RESET_MSK);
	if (ret)
		return ret;

	fsleep(1000);

	if (!(st->spi->mode & SPI_3WIRE)) {
		ret = regmap_write(st->regmap, ADMV1355_REG_SPI_CONFIG,
				   ADMV1355_SDO_ACTIVE_MSK);
		if (ret)
			return ret;
	}

	ret = admv1355_check_spi(st);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_PRODUCT_ID_L, &id_l);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_PRODUCT_ID_H, &id_h);
	if (ret)
		return ret;

	dev_dbg(&st->spi->dev, "product ID: 0x%04x\n", id_h << 8 | id_l);

	ret = admv1355_nvm_init(st);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(st->regmap, admv1355_init_seq,
				     ARRAY_SIZE(admv1355_init_seq));
	if (ret)
		return ret;

	ret = admv1355_set_lo_filters(st, st->lo_rate);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADMV1355_REG_IF_ENABLE, &val);
	if (ret)
		return ret;

	st->if_complex_band = val & ADMV1355_IF_HIGH_FREQ_EN_MSK ?
			      ADMV1355_BAND_HIGH : ADMV1355_BAND_LOW;

	return admv1355_set_mixer_gate(st, val & ADMV1355_BB_EN_MSK);
}

static int admv1355_get_chip_addr(struct admv1355_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 addr = 0;
	int ret;

	if (!device_property_present(dev, "spi-device-addr"))
		return 0;

	ret = device_property_read_u32(dev, "spi-device-addr", &addr);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read spi-device-addr\n");

	if (addr > ADMV1355_SPI_CHIP_ADDR_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "spi-device-addr %u out of range [0, %u]\n",
				     addr, ADMV1355_SPI_CHIP_ADDR_MAX);

	st->chip_addr = addr;

	return 0;
}

static int admv1355_get_bb_vicm(struct admv1355_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 vicm;
	int ret;

	ret = device_property_read_u32(dev, "adi,baseband-vicm-microvolt", &vicm);
	if (ret == -EINVAL)
		return 0;
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read adi,baseband-vicm-microvolt\n");

	if (vicm > ADMV1355_BB_VICM_MAX_UV)
		return dev_err_probe(dev, -EINVAL,
				     "baseband VICM %u uV out of range\n", vicm);

	/* The mixer gate bias is set automatically above 0.5 V */
	if (vicm > ADMV1355_BB_VICM_MANUAL_MAX_UV)
		return 0;

	st->bb_mixer_gate =
		&admv1355_bb_mixer_gates[DIV_ROUND_CLOSEST(vicm,
							   ADMV1355_BB_VICM_STEP_UV)];

	return 0;
}

static void admv1355_power_off(void *data)
{
	struct admv1355_state *st = data;

	/* Let the RF circuits power down before asserting reset (t23) */
	gpiod_set_value_cansleep(st->enable_gpio, 0);
	fsleep(3);
	gpiod_set_value_cansleep(st->reset_gpio, 1);
}

static int admv1355_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct admv1355_state *st;
	struct iio_dev *indio_dev;
	unsigned long rate;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, indio_dev);
	st->half_duplex = spi->mode & SPI_3WIRE ||
			  spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX;

	ret = admv1355_get_chip_addr(st);
	if (ret)
		return ret;

	ret = admv1355_get_bb_vicm(st);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(dev, NULL, st, &admv1355_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init regmap\n");

	/* All logic inputs must stay low until the supplies are stable */
	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "failed to get reset GPIO\n");

	st->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(st->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(st->enable_gpio),
				     "failed to get enable GPIO\n");

	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(admv1355_supply_names),
					     admv1355_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, admv1355_power_off, st);
	if (ret)
		return ret;

	clk = devm_clk_get_enabled(dev, "lo-input");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get LO input clock\n");

	rate = clk_get_rate(clk);
	if (!admv1355_lo_rate_valid(rate))
		return dev_err_probe(dev, -EINVAL,
				     "LO input rate %lu Hz out of range\n", rate);

	st->lo_rate = rate;

	/* Release reset, then wait for the digital logic to settle (t3) */
	gpiod_set_value_cansleep(st->reset_gpio, 0);
	fsleep(1);

	ret = admv1355_setup(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up device\n");

	/* The RF circuits are enabled only after the initialization (t16) */
	gpiod_set_value_cansleep(st->enable_gpio, 1);

	st->nb.notifier_call = admv1355_lo_clk_notifier;
	ret = devm_clk_notifier_register(dev, clk, &st->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register clock notifier\n");

	indio_dev->name = "admv1355";
	indio_dev->info = &admv1355_info;
	indio_dev->channels = admv1355_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1355_channels);

	return devm_iio_device_register(dev, indio_dev);
}

/*
 * Disable the chip while keeping the supplies on, so the register contents
 * are retained across suspend.
 */
static int admv1355_suspend(struct device *dev)
{
	struct admv1355_state *st = iio_priv(dev_get_drvdata(dev));

	guard(mutex)(&st->lock);

	gpiod_set_value_cansleep(st->enable_gpio, 0);

	return regmap_set_bits(st->regmap, ADMV1355_REG_POWER_DOWN,
			       ADMV1355_CHIP_PD_MSK);
}

static int admv1355_resume(struct device *dev)
{
	struct admv1355_state *st = iio_priv(dev_get_drvdata(dev));
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_clear_bits(st->regmap, ADMV1355_REG_POWER_DOWN,
				ADMV1355_CHIP_PD_MSK);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(st->enable_gpio, 1);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(admv1355_pm_ops, admv1355_suspend,
				admv1355_resume);

static const struct spi_device_id admv1355_id[] = {
	{ "admv1355" },
	{ }
};
MODULE_DEVICE_TABLE(spi, admv1355_id);

static const struct of_device_id admv1355_of_match[] = {
	{ .compatible = "adi,admv1355" },
	{ }
};
MODULE_DEVICE_TABLE(of, admv1355_of_match);

static struct spi_driver admv1355_driver = {
	.driver = {
		.name = "admv1355",
		.of_match_table = admv1355_of_match,
		.pm = pm_sleep_ptr(&admv1355_pm_ops),
	},
	.probe = admv1355_probe,
	.id_table = admv1355_id,
};
module_spi_driver(admv1355_driver);

MODULE_AUTHOR("George Mois <george.mois@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADMV1355 Microwave Upconverter");
MODULE_LICENSE("GPL");
