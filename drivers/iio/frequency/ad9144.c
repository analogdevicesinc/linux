// SPDX-License-Identifier: GPL-2.0
/*
 * AD9144 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2014-2022 Analog Devices Inc.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>

#define JESD204_OF_PREFIX "adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9144.h"
#include "cf_axi_dds.h"

#define AD9144_CHIPID(product_idh, product_idl, product_grade) \
	(((product_idh) << 16) | ((product_idl) << 8) | (product_grade))

#define AD9144_ID_GET_PRODUCT_ID(x) ((x) >> 8)

enum chip_id {
	CHIPID_AD9135 = AD9144_CHIPID(0x91, 0x44, 0x4),
	CHIPID_AD9136 = AD9144_CHIPID(0x91, 0x44, 0x6),
	CHIPID_AD9144 = AD9144_CHIPID(0x91, 0x44, 0x0),
	CHIPID_AD9152 = AD9144_CHIPID(0x91, 0x52, 0x0),
	CHIPID_AD9154 = AD9144_CHIPID(0x91, 0x54, 0x9),
};

#define AD9144_MOD_TYPE_NONE		(0x0 << 2)
#define AD9144_MOD_TYPE_FINE		(0x1 << 2)
#define AD9144_MOD_TYPE_COARSE4		(0x2 << 2)
#define AD9144_MOD_TYPE_COARSE8		(0x3 << 2)
#define AD9144_MOD_TYPE_MASK		(0x3 << 2)

struct ad9144_platform_data {
	u8 xbar_lane_sel[8];
	u8 interpolation;
	unsigned int fcenter_shift;
	bool spi4wire;

	u8 jesd_link_mode;
	u8 jesd_subclass;

	unsigned int pll_frequency;
	bool pll_enable;

	unsigned int sync_mode;
};

struct ad9144_jesd204_priv {
	struct ad9144_state *st;
};

struct ad9144_state {
	struct jesd204_dev *jdev;
	struct jesd204_link link_config;
	struct cf_axi_converter conv;
	unsigned int interpolation;
	unsigned int fcenter_shift;
	enum chip_id id;
	struct regmap *map;

	unsigned int num_lanes;
	unsigned int num_converters;
	unsigned int octets_per_frame;
	u8 lane_mux[8];
	unsigned int pll_frequency;
	bool pll_enable;
};

static const struct {
	u8 m, l, s, f, hd, n, np;
} ad9144_jesd_modes[] = {
	/* 00 */ {4, 8, 1, 1, 1, 16, 16},
	/* 01 */ {4, 8, 2, 2, 0, 16, 16},
	/* 02 */ {4, 4, 1, 2, 0, 16, 16},
	/* 03 */ {4, 2, 1, 4, 0, 16, 16},
	/* 04 */ {2, 4, 1, 1, 1, 16, 16},
	/* 05 */ {2, 4, 2, 2, 0, 16, 16},
	/* 06 */ {2, 2, 1, 2, 0, 16, 16},
	/* 07 */ {2, 1, 1, 4, 0, 16, 16},
	/* 08 */ {1, 4, 2, 1, 1, 16, 16},
	/* 09 */ {1, 2, 1, 1, 1, 16, 16},
	/* 10 */ {1, 1, 1, 2, 0, 16, 16},
	/* 11 */ {2, 8, 2, 1, 1, 16, 16},
	/* 12 */ {2, 4, 1, 1, 1, 16, 16},
	/* 13 */ {2, 2, 1, 2, 0, 16, 16}
};

static const char * const clk_names[] = {
	[CLK_DATA] = "jesd_dac_clk",
	[CLK_DAC] = "dac_clk",
	[CLK_REF] = "dac_sysref"
};

static int ad9144_read(struct spi_device *spi, unsigned reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9144_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);

	return regmap_write(st->map, reg, val);
}

static int ad9144_get_temperature_code(struct cf_axi_converter *conv)
{
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	unsigned val1, val2;

	regmap_write(st->map, REG_DIE_TEMP_CTRL0, 1);
	regmap_write(st->map, REG_DIE_TEMP_UPDATE, 1);

	regmap_read(st->map, REG_DIE_TEMP0, &val1);
	regmap_read(st->map, REG_DIE_TEMP1, &val2);
	return ((val2 & 0xFF) << 8) | (val1 & 0xFF);
}

/* PLL fixed register writes according to datasheet */
static const struct reg_sequence ad9144_pll_fixed_writes[] = {
	{ 0x87, 0x62 },
	{ 0x88, 0xc0 },
	{ 0x89, 0x0e },
	{ 0x8a, 0x12 },
	{ 0x8d, 0x7b },
	{ 0x1b0, 0x00 },
	{ 0x1b9, 0x24 },
	{ 0x1bc, 0x0d },
	{ 0x1be, 0x02 },
	{ 0x1bf, 0x8e },
	{ 0x1c0, 0x2a },
	{ 0x1c1, 0x2a },
	{ 0x1c4, 0x7e },
};

static unsigned int ad9144_round_pll_rate(struct ad9144_state *st,
	unsigned int fdac)
{
	unsigned int fref;

	fref = clk_get_rate(st->conv.clk[CLK_DAC]);
	if (fref == 0)
		return fdac;

	if (fdac > 2800000000U)
		fdac = 2800000000U;
	else if (fdac < 420000000U)
		fdac = 420000000U;

	while (fref > 80000000U)
		fref /= 2;

	return DIV_ROUND_CLOSEST(fdac, 2 * fref) * 2 * fref;
}

static int ad9144_setup_pll(struct ad9144_state *st)
{
	struct regmap *map = st->map;
	unsigned int fref, fdac;
	unsigned int lo_div_mode;
	unsigned int ref_div_mode = 0;
	unsigned int vco_param[3];
	unsigned int bcount;
	unsigned int fvco;

	fref = clk_get_rate(st->conv.clk[CLK_DAC]) / 1000;
	fdac = st->pll_frequency / 1000;

	if (fref > 1000000 || fref < 35000)
		return -EINVAL;

	if (fdac > 2800000 || fdac < 420000)
		return -EINVAL;

	if (fdac >= 1500000)
		lo_div_mode = 1;
	else if (fdac >= 750000)
		lo_div_mode = 2;
	else
		lo_div_mode = 3;

	while (fref > 80000) {
		ref_div_mode++;
		fref /= 2;
	}

	fvco = fdac << (lo_div_mode + 1);
	bcount = fdac / (2 * fref);
	if (bcount < 6) {
		bcount *= 2;
		ref_div_mode++;
	}

	if (fvco < 6300000) {
		vco_param[0] = 0x8;
		vco_param[1] = 0x3;
		vco_param[2] = 0x7;
	} else if (fvco < 7250000) {
		vco_param[0] = 0x9;
		vco_param[1] = 0x3;
		vco_param[2] = 0x6;
	} else {
		vco_param[0] = 0x9;
		vco_param[1] = 0x13;
		vco_param[2] = 0x6;
	}

	regmap_multi_reg_write(map, ad9144_pll_fixed_writes,
		ARRAY_SIZE(ad9144_pll_fixed_writes));

	regmap_write(map, REG_DACLOGENCNTRL, lo_div_mode);
	regmap_write(map, REG_DACLDOCNTRL1, ref_div_mode);
	regmap_write(map, REG_DACINTEGERWORD0, bcount);

	regmap_write(map, REG_DACPLLT5, vco_param[0]);
	regmap_write(map, REG_DACPLLTB, vco_param[1]);
	regmap_write(map, REG_DACPLLT18, vco_param[2]);

	regmap_write(map, REG_DACPLLCNTRL, 0x10);

	return 0;
}

static int ad9144_setup_link(struct ad9144_state *st,
	struct jesd204_link *config)
{
	struct regmap *map = st->map;
	unsigned int lane_mask;
	bool ad9136_dual_mode;
	unsigned int M, L;
	unsigned int val;
	unsigned int i, j;

	/*
	 * Datasheet calls this mode 11, 12, 13. L and M need to be
	 * programmed to half their actual values.
	 */
	ad9136_dual_mode = (st->id == CHIPID_AD9136 ||
			    st->id == CHIPID_AD9135) &&
			   config->num_converters == 2;

	if (ad9136_dual_mode) {
		M = 0;
		L = (config->num_lanes / 2) - 1;
		lane_mask = (1 << (config->num_lanes / 2)) - 1;
		lane_mask |= lane_mask << 4;
	} else {
		M = config->num_converters - 1;
		L = config->num_lanes - 1;
		lane_mask = (1 << config->num_lanes) - 1;
	}

	for (i = 0; i < 4; i++) {
		if (ad9136_dual_mode && i >= 2)
			j = config->num_lanes / 2 + 2*(i-2);
		else
			j = 2*i;

		val = st->lane_mux[j];
		val |= st->lane_mux[j + 1] << 3;
		regmap_write(map, REG_XBAR(i), val);
	}

	val = 0;
	if (!config->subclass)
		val |= BIT(4);
	if (!config->sysref.capture_falling_edge)
		val |= BIT(2);
	regmap_write(map, REG_SYSREF_ACTRL0, val);

	regmap_write(map, REG_GENERAL_JRX_CTRL_1, config->subclass);

	regmap_write(map, REG_ILS_DID, config->device_id);
	regmap_write(map, REG_ILS_BID, config->bank_id);

	val = L; /* L */
	if (config->scrambling)
		val |= BIT(7);
	regmap_write(map, REG_ILS_SCR_L, val);

	regmap_write(map, REG_ILS_F, config->octets_per_frame - 1); /* F */
	regmap_write(map, REG_ILS_K, config->frames_per_multiframe - 1); /* K */
	regmap_write(map, REG_ILS_M, M); /* M */
	regmap_write(map, REG_ILS_CS_N, 15); /* N */

	val = 15; /* NP */
	val |= config->subclass << 5; /* SUBCLASSV */
	regmap_write(map, REG_ILS_NP, val);

	val = config->samples_per_conv_frame - 1; /* S */
	val |= BIT(5); /* JESDVER */
	regmap_write(map, REG_ILS_S, val);

	val = config->high_density ? BIT(7) : 0x0; /* HD */
	regmap_write(map, REG_ILS_HD_CF, val);

	/* Static for now */
	regmap_write(map, REG_KVAL, 0x01);

	regmap_write(map, REG_LANEDESKEW, lane_mask);
	regmap_write(map, REG_CTRLREG1, config->octets_per_frame);
	regmap_write(map, REG_LANEENABLE, lane_mask);

	/*
	 * Length of the SYNC~ error pulse in PCLK cycles. According to the
	 * JESD204 standard the pulse length should be two frame clock cycles.
	 *
	 * 1 PCLK cycle = 4 octets
	 *   => SYNC~ pulse length = 2 * octets_per_frame / 4
	 */
	switch (config->octets_per_frame) {
	case 1:
		/* 0.5 PCLK cycles */
		val = 0x0;
		break;
	case 2:
		/* 1 PCLK cycle */
		val = 0x1;
		break;
	default:
		/* 2 PCLK cycles */
		val = 0x2;
		break;
	}
	regmap_write(map, REG_SYNCB_GEN_1, val << 4);

	return 0;
}

static void ad9144_set_nco_freq(struct ad9144_state *st, uint32_t sample_rate,
	uint32_t nco_freq)
{
	unsigned int mod_type;
	unsigned int i;
	uint64_t ftw;

	if (nco_freq == 0 || nco_freq >= sample_rate) {
		mod_type = AD9144_MOD_TYPE_NONE;
	} else if (sample_rate == nco_freq * 4) {
		mod_type = AD9144_MOD_TYPE_COARSE4;
	} else if (sample_rate == nco_freq * 8) {
		mod_type = AD9144_MOD_TYPE_COARSE8;
	} else {
		mod_type = AD9144_MOD_TYPE_FINE;
		ftw = mul_u64_u32_div(1ULL << 48, nco_freq, sample_rate);

		for (i = 0; i < 6; i++) {
			regmap_write(st->map, 0x114 + i, ftw & 0xff);
			ftw >>= 8;
		}
	}

	regmap_update_bits(st->map, 0x111, AD9144_MOD_TYPE_MASK, mod_type);

	if (mod_type == AD9144_MOD_TYPE_FINE)
		regmap_write(st->map, 0x113, 1);
}

static unsigned long ad9144_get_lane_rate(struct ad9144_state *st,
	unsigned int sample_rate)
{
	/*
	 * lanerate_khz = ((samplerate_hz / interpolation) * 20 * M / L) / 1000
	 *
	 * Slightly reordered here to avoid loss of precession or overflows.
	 */
	return DIV_ROUND_CLOSEST(sample_rate,
		50 * st->num_lanes * st->interpolation / st->num_converters);
}

static unsigned long ad9144_get_lmfc(struct ad9144_state *st,
	unsigned int sample_rate)
{
	/*
	 * lmfc = ((samplerate_hz / interpolation) * 20 * M / (K * K * F * 10))
	 * K is always 32
	 */
	return DIV_ROUND_CLOSEST(sample_rate,
		16 * st->num_lanes * st->interpolation * st->octets_per_frame /
		st->num_converters);
}

static bool ad9144_check_sysref_rate(unsigned int lmfc, unsigned int sysref)
{
	unsigned int div, mod;

	div = lmfc / sysref;
	mod = lmfc % sysref;

	/* Ignore minor deviations that can be introduced by rounding. */
	return mod <= div || mod >= sysref - div;
}

static int ad9144_update_sysref(struct ad9144_state *st,
	unsigned int sample_rate)
{
	unsigned int lmfc = ad9144_get_lmfc(st, sample_rate);
	unsigned int n;
	int rate;
	int ret;

	/* No clock, no problem */
	if (!st->conv.clk[CLK_REF])
		return 0;

	rate = clk_get_rate(st->conv.clk[CLK_REF]);
	if (rate < 0)
		return rate;

	/* If the current rate is OK, keep it */
	if (ad9144_check_sysref_rate(lmfc, rate))
		return 0;

	/*
	 * Try to find a rate that integer divides the LMFC. Starting with a low
	 * rate is a good idea and then slowly go up in case the clock generator
	 * can't generate such slow rates.
	 */
	for (n = 32; n > 0; n--) {
		rate = clk_round_rate(st->conv.clk[CLK_REF], lmfc / n);
		if (ad9144_check_sysref_rate(lmfc, rate))
			break;
	}

	if (n == 0) {
		dev_err(&st->conv.spi->dev,
			"Could not find suitable SYSREF rate for samplerate of %u and LMFC of %u\n",
			sample_rate, lmfc);
		return -EINVAL;
	}

	ret = clk_set_rate(st->conv.clk[CLK_REF], rate);
	if (ret)
		dev_err(&st->conv.spi->dev, "Failed to set SYSREF rate to %d kHz: %d\n",
			rate, ret);

	return ret;
}

static int ad9144_dac_calibrate(struct ad9144_state *st)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	unsigned int dac_mask;
	unsigned int timeout;
	unsigned int val;
	unsigned int i;

	switch (st->id) {
	case CHIPID_AD9144:
	case CHIPID_AD9154:
		dac_mask = GENMASK(st->num_converters - 1, 0);
		break;
	default: /* AD9135/AD9136 */
		dac_mask = BIT(0);
		if (st->num_converters == 2)
			dac_mask |= BIT(2);
		break;
	}

	/*
	 * DAC calibration sequence as per table 86 AD9144 datasheet Rev B.
	 */

	/* Set calibration clock */
	regmap_write(map, REG_CAL_CLKDIV, 0x38);
	/* Set initial value */
	regmap_write(map, REG_CAL_INIT, 0xa2);
	/* Select all DACs */
	regmap_write(map, REG_CAL_INDX, dac_mask);

	/* Start calibration */
	regmap_write(map, REG_CAL_CTRL, 0x01);
	regmap_write(map, REG_CAL_CTRL, 0x03);
	mdelay(10);

	for (i = 0; i < st->num_converters; i++) {
		switch (st->id) {
		case CHIPID_AD9144:
		case CHIPID_AD9154:
			dac_mask = BIT(i);
			break;
		default:
			dac_mask = BIT(2*i);
			break;
		}

		/* Select DAC N */
		regmap_write(map, REG_CAL_INDX, dac_mask);

		timeout = 30;
		do {
			mdelay(1);
			regmap_read(map, REG_CAL_CTRL, &val);
		} while ((val & CAL_ACTIVE) && timeout--);

		if ((val & (CAL_FIN | CAL_ERRHI | CAL_ERRLO)) != CAL_FIN)
			dev_err(dev, "DAC-%d calibration failed (0x%X)\n",
				i, val);
		else
			dev_dbg(dev, "DAC-%d calibration successful\n", i);
	}

	/* Turn off calibration clock */
	regmap_write(map, REG_CAL_CLKDIV, 0x30);

	return 0;
}

static unsigned int ad9144_get_sample_rate(struct ad9144_state *st)
{
	if (st->pll_enable)
		return st->pll_frequency;
	else
		return clk_get_rate(st->conv.clk[CLK_DAC]);
}

static void ad9144_setup_samplerate(struct ad9144_state *st)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	unsigned int sample_rate;
	unsigned int serdes_plldiv, serdes_cdr;
	unsigned int serdes_vco_freq;
	unsigned int val;
	unsigned long lane_rate_kHz;

	sample_rate = ad9144_get_sample_rate(st);
	lane_rate_kHz = ad9144_get_lane_rate(st, sample_rate);

	ad9144_set_nco_freq(st, sample_rate, st->fcenter_shift);

	/*
	 * Based on table 4 of the AD9144 datasheet Rev. B.
	 */
	if (lane_rate_kHz < 2880000) {
		serdes_cdr = 0x0a;
		serdes_plldiv = 0x06;
		serdes_vco_freq = lane_rate_kHz * 4;
	} else if (lane_rate_kHz < 5750000) {
		serdes_cdr = 0x08;
		serdes_plldiv = 0x05;
		serdes_vco_freq = lane_rate_kHz * 2;
	} else {
		serdes_cdr = 0x28;
		serdes_plldiv = 0x04;
		serdes_vco_freq = lane_rate_kHz;
	}

	if (st->id == CHIPID_AD9152) {
		/*
		 * VCO filter and charge pump setting.
		 * Based on Table 36 of the AD9152 datasheet Rev. B
		 */
		if (serdes_vco_freq < 7150000) {
			regmap_write(map, 0x296, 0x02);
			regmap_write(map, 0x291, 0x49);
			regmap_write(map, 0x28a, 0x7b);
		} else {
			regmap_write(map, 0x296, 0x03);
			regmap_write(map, 0x291, 0x4c);
			regmap_write(map, 0x28a, 0x2b);
		}
	}

	// physical layer

	regmap_write(map, 0x280, 0x00);	// disable serdes pll

	regmap_write(map, 0x2a7, 0x01);	// input termination calibration
	if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144 ||
	    AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9154)
		regmap_write(map, 0x2ae, 0x01);	// input termination calibration

	regmap_write(map, 0x230, serdes_cdr);

	regmap_write(map, 0x206, 0x00);	// cdr reset
	regmap_write(map, 0x206, 0x01);	// cdr reset

	regmap_write(map, 0x289, serdes_plldiv);

	regmap_write(map, 0x280, 0x01);	// enable serdes pll
	mdelay(20);

	regmap_read(map, 0x281, &val);
	if ((val & 0x01) == 0x00)
		dev_err(dev, "SERDES PLL not locked.\n");

	if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144)
		ad9144_dac_calibrate(st);
}

/*
 * Required device configuration as per table 16 from the AD9144
 * datasheet Rev B.
 */
static const struct reg_sequence ad9144_required_device_config[] = {
	{ 0x12d, 0x8b },
	{ 0x146, 0x01 },
	{ 0x2a4, 0xff },
	{ 0x232, 0xff },
	{ 0x333, 0x01 },
};

/*
 * Optimal settings for the SERDES PLL, as per table 39 of the AD9144 datasheet.
 * and table 16 of the AD9152 datasheet.
 */
static const struct reg_sequence ad9144_optimal_serdes_settings[] = {
	{ 0x284, 0x62 },
	{ 0x285, 0xc9 },
	{ 0x286, 0x0e },
	{ 0x287, 0x12 },
	{ 0x28b, 0x00 },
	{ 0x290, 0x89 },
	{ 0x294, 0x24 },
	{ 0x297, 0x0d },
	{ 0x299, 0x02 },
	{ 0x29a, 0x8e },
	{ 0x29c, 0x2a },
	{ 0x29f, 0x78 },
	{ 0x2a0, 0x06 },
};

static int ad9144_link_status_get(struct ad9144_state *st)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	int ret, i;
	unsigned int regs[4];

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = regmap_read(map, REG_CODEGRPSYNCFLG + i, &regs[i]);
		if (ret != 0) {
			dev_err(dev, "Get Link0 status failed\n");
			return -EIO;
		}
	}

	dev_info(dev, "Link0 code grp sync: %x\n", regs[0]);
	dev_info(dev, "Link0 frame sync stat: %x\n", regs[1]);
	dev_info(dev, "Link0 good checksum stat: %x\n", regs[2]);
	dev_info(dev, "Link0 init lane_sync stat: %x\n", regs[3]);
	dev_info(dev, "Link0 %d lanes @ %lu kBps\n", st->num_lanes,
		 ad9144_get_lane_rate(st, ad9144_get_sample_rate(st)));

	if (hweight8(regs[0]) != st->num_lanes ||
		regs[0] != regs[1] || regs[0] != regs[3])
		ret = -EFAULT;

	return 0;
}

static int ad9144_setup(struct ad9144_state *st,
	struct jesd204_link *link_config)
{
	struct regmap *map = st->map;
	unsigned int sync_mode;
	unsigned int phy_mask;
	unsigned int pd_dac;
	unsigned int pd_clk;
	unsigned int val;
	unsigned int i;

	regmap_write(map, 0x300, 0x00);	// single link - link 0

	// power-up and dac initialization

	switch (st->id) {
	case CHIPID_AD9152:
		if (st->num_converters == 1)
			pd_dac = BIT(5) | BIT(2);
		else
			pd_dac = 0x00;
		pd_clk = 0x04;
		break;
	case CHIPID_AD9144:
	case CHIPID_AD9154:
		pd_clk = GENMASK(7 - DIV_ROUND_UP(st->num_converters, 2), 6);
		if (st->id == CHIPID_AD9154)
			pd_clk |= 2;
		pd_dac = GENMASK(6 - st->num_converters, 3);
		break;
	default: /* AD9135/AD9136 */
		if (st->num_converters == 1) {
			pd_dac = GENMASK(5, 3);
			pd_clk = BIT(5);
		} else {
			pd_dac = BIT(5) | BIT(3);
			pd_clk = 0x00;
		}
		break;
	}

	regmap_write(map, REG_PWRCNTRL0, pd_dac); /* Power-up DACs */
	regmap_write(map, REG_CLKCFG0, pd_clk); /* Power-up clocks */
	regmap_write(map, 0x081, 0x00);	// sysref - power up/falling edge

	regmap_write(map, 0x2aa, 0xb7);	// jesd termination
	regmap_write(map, 0x2ab, 0x87);	// jesd termination

	regmap_write(map, 0x314, 0x01);	// pclk == qbd master clock

	if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144 ||
	    AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9154) {
		regmap_multi_reg_write(map, ad9144_required_device_config,
			ARRAY_SIZE(ad9144_required_device_config));

		if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144) {
			/*
			 * SERDES optimization according to table 39 AD9144 Rev. B
			 * datasheet.
			 */
			regmap_write(map, 0x296, 0x03);
			regmap_write(map, 0x28a, 0x7b);

			regmap_write(map, 0x2b1, 0xb7);	// jesd termination
			regmap_write(map, 0x2b2, 0x87);	// jesd termination
		} else {
			regmap_write(map, 0x28a, 0x7b);
			regmap_write(map, 0x291, 0x4c);
			regmap_write(map, 0x296, 0x1b);
		}
	}

	regmap_multi_reg_write(map, ad9144_optimal_serdes_settings,
		ARRAY_SIZE(ad9144_optimal_serdes_settings));

	if (st->pll_enable)
		ad9144_setup_pll(st);

	// digital data path

	switch (st->interpolation) {
	case 2:
		val = 0x01;
		break;
	case 4:
		if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144 ||
		    AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9154)
			val = 0x03;
		else
			val = 0x02;
		break;
	case 8:
		if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144 ||
		    AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9154)
			val = 0x04;
		else
			val = 0x03;
		break;
	default:
		val = 0x00;
		break;
	}
	regmap_write(map, 0x112, val);	// interpolation
	regmap_write(map, 0x110, 0x00);	// 2's complement

	// transport layer

	phy_mask = 0xff;
	for (i = 0; i < link_config->num_lanes; i++)
		phy_mask &= ~BIT(st->lane_mux[i]);

	regmap_write(map, REG_MASTER_PD, 0x00);
	regmap_write(map, REG_PHY_PD, phy_mask);

	ad9144_setup_link(st, link_config);

	regmap_write(map, 0x268, 0x62);	// equalizer

	// data link layer

	/* LMFC settings for link 0 */
	regmap_write(map, 0x304, 0x00);	// lmfc delay
	regmap_write(map, 0x306, 0x0a);	// receive buffer delay
	if (AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9144 ||
	    AD9144_ID_GET_PRODUCT_ID(st->id) == 0x9154) {
		/* LMFC settings for link 1 */
		regmap_write(map, 0x305, 0x00);	// lmfc delay
		regmap_write(map, 0x307, 0x0a);	// receive buffer delay
	}

	if (link_config->sysref.mode == JESD204_SYSREF_ONESHOT)
		sync_mode = 0x1;
	else
		sync_mode = 0x2;

	regmap_write(map, REG_SYNC_CTRL, sync_mode);
	regmap_write(map, REG_SYNC_CTRL, sync_mode | SYNCENABLE);
	regmap_write(map, REG_SYNC_CTRL, sync_mode | SYNCENABLE | SYNCARM);

	ad9144_setup_samplerate(st);

	if (st->jdev)
		return 0;

	regmap_write(map, 0x300, 0x01);	// enable link

	return 0;
}

static int ad9144_get_clks(struct cf_axi_converter *conv)
{
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	int ret;

	conv->clk[CLK_DAC] = devm_clk_get(&conv->spi->dev, clk_names[CLK_DAC]);
	if (IS_ERR(conv->clk[CLK_DAC]))
		return PTR_ERR(conv->clk[CLK_DAC]);

	ret = clk_prepare_enable(conv->clk[CLK_DAC]);
	if (ret < 0)
		return ret;

	if (!st->jdev) {
		conv->clk[CLK_DATA] = devm_clk_get(&conv->spi->dev, clk_names[CLK_DATA]);
		if (IS_ERR(conv->clk[CLK_DATA]))
			return PTR_ERR(conv->clk[CLK_DATA]);
	}

	conv->clk[CLK_REF] = devm_clk_get(&conv->spi->dev, clk_names[CLK_REF]);
	if (IS_ERR(conv->clk[CLK_REF])) {
		if (PTR_ERR(conv->clk[CLK_REF]) == -ENOENT) {
			conv->clk[CLK_REF] = NULL;
			return 0;
		} else {
			return PTR_ERR(conv->clk[CLK_REF]);
		}
	}

	return  clk_prepare_enable(conv->clk[CLK_REF]);
}

static unsigned long long ad9144_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);

	return ad9144_get_sample_rate(st) / st->interpolation;
}

static int ad9144_set_sample_rate(struct cf_axi_converter *conv,
	unsigned int sample_rate)
{
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	struct regmap *map = st->map;
	unsigned long lane_rate_kHz;
	unsigned long sysref_rate;
	unsigned int max_sample_rate;
	unsigned int min_sample_rate;
	unsigned int max_lane_rate_khz;
	unsigned int bits_per_sample_per_lane;
	unsigned int tmp;
	int ret;

	switch (st->id) {
	case CHIPID_AD9136:
	case CHIPID_AD9135:
		max_lane_rate_khz = 12400000;
		switch (st->interpolation) {
		case 1:
		case 2:
			max_sample_rate = 2120000000U;
			break;
		default:
			max_sample_rate = 2800000000U;
			break;
		}
		break;
	case CHIPID_AD9144:
	case CHIPID_AD9154:
		max_lane_rate_khz = 12400000;
		switch (st->interpolation) {
		case 1:
			max_sample_rate = 1060000000U;
			break;
		case 2:
			max_sample_rate = 2120000000U;
			break;
		default:
			max_sample_rate = 2800000000U;
			break;
		}
		break;
	default:
		max_lane_rate_khz = 12380000;
		switch (st->interpolation) {
		case 1:
			max_sample_rate = 1238000000U;
			break;
		default:
			max_sample_rate = 2250000000U;
			break;
		}
		break;
	}

	/*
	 * The lane rate limits can put further constraints on the sample rate
	 * depending on converter to lane ratio and interpolation factor.
	 */

	/* 5, 10, 20 or 40 */
	bits_per_sample_per_lane = (20 * st->num_converters) / st->num_lanes;

	/* Be careful not to overflow */
	tmp = max_lane_rate_khz * (1000 / bits_per_sample_per_lane);
	if (tmp < max_sample_rate / st->interpolation)
		max_sample_rate = tmp * st->interpolation;

	/* Based on minimum lane rate */
	min_sample_rate = 1440000000U / bits_per_sample_per_lane;
	min_sample_rate *= st->interpolation;

	sample_rate = clamp(sample_rate, min_sample_rate, max_sample_rate);
	if (st->pll_enable)
		sample_rate = ad9144_round_pll_rate(st, sample_rate);
	else
		sample_rate = clk_round_rate(conv->clk[CLK_DAC], sample_rate);

	if (st->jdev) {
		jesd204_fsm_stop(st->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(st->jdev, JESD204_LINKS_ALL);

		if (!st->pll_enable) {
			ret = clk_set_rate(conv->clk[CLK_DAC], sample_rate);
			if (ret < 0) {
				dev_err(&conv->spi->dev,
					"Failed to set sample rate: %d\n", ret);
				return ret;
			}
		} else {
			st->pll_frequency = sample_rate;
		}

		return jesd204_fsm_start(st->jdev, JESD204_LINKS_ALL);
	}

	sysref_rate = DIV_ROUND_CLOSEST(sample_rate, 128);
	lane_rate_kHz = ad9144_get_lane_rate(st, sample_rate);

	regmap_write(map, 0x300, 0x00);	// disable link

	if (!st->pll_enable)
		clk_disable_unprepare(conv->clk[CLK_DAC]);
	clk_disable_unprepare(conv->clk[CLK_DATA]);
	clk_disable_unprepare(conv->clk[CLK_REF]);

	ret = ad9144_update_sysref(st, sample_rate);
	if (ret < 0)
		return ret;

	ret = clk_set_rate(conv->clk[CLK_DATA], lane_rate_kHz);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set lane rate to %ld kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->clk[CLK_REF]);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to enable SYSREF clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->clk[CLK_DATA]);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	if (!st->pll_enable) {
		ret = clk_set_rate(conv->clk[CLK_DAC], sample_rate);
		if (ret < 0) {
			dev_err(&conv->spi->dev,
				"Failed to set sample rate: %d\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(conv->clk[CLK_DAC]);
		if (ret < 0) {
			dev_err(&conv->spi->dev,
					"Failed to enable sample rate clock: %d\n",
					ret);
			return ret;
		}
	} else {
		st->pll_frequency = sample_rate;
		ad9144_setup_pll(st);
	}

	ad9144_setup_samplerate(st);
	regmap_write(map, 0x300, 0x01);	// enable link

	return 0;
}

static int ad9144_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned tmp;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad9144_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = conv->temp_calib_code;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		if (!conv->temp_calib_code)
			return -EINVAL;

		tmp = ad9144_get_temperature_code(conv);

		*val = ((tmp - conv->temp_calib_code) * 77
			+ conv->temp_calib * 10 + 10000) / 10;

		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9144_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad9144_set_sample_rate(conv, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		conv->temp_calib_code = val;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		/*
		 * Writing in_temp0_input with the device temperature in milli
		 * degrees Celsius triggers the calibration.
		 */
		conv->temp_calib_code = ad9144_get_temperature_code(conv);
		conv->temp_calib = val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9144_prepare(struct cf_axi_converter *conv)
{
	return 0;
}

#ifdef CONFIG_OF
static struct ad9144_platform_data *ad9144_parse_dt(struct device *dev)
{
	char prop_name[] = "adi,jesd-xbar-lane0-sel";
	struct device_node *np = dev->of_node;
	struct ad9144_platform_data *pdata;
	unsigned int tmp;
	unsigned int i;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	for (i = 0; i < 8; i++) {
		prop_name[sizeof("adi,jesd-xbar-lane")-1] = '0' + i;

		tmp = i;
		of_property_read_u32(np, prop_name, &tmp);
		pdata->xbar_lane_sel[i] = tmp;
	}

	tmp = 1;
	of_property_read_u32(np, "adi,interpolation", &tmp);
	pdata->interpolation = tmp;

	tmp = 0;
	of_property_read_u32(np, "adi,frequency-center-shift", &tmp);
	pdata->fcenter_shift = tmp;

	tmp = 4;
	of_property_read_u32(np, "adi,jesd-link-mode", &tmp);
	pdata->jesd_link_mode = (tmp > 13 ? 4 : tmp);

	tmp = JESD204_SUBCLASS_1;
	of_property_read_u32(np, "adi,subclass", &tmp);
	pdata->jesd_subclass = (tmp > JESD204_SUBCLASS_1 ?
		JESD204_SUBCLASS_1 : tmp);

	pdata->pll_enable = of_property_read_bool(np, "adi,pll-enable");

	tmp = 0;
	of_property_read_u32(np, "adi,pll-frequency", &tmp);
	pdata->pll_frequency = tmp;

	if (pdata->pll_enable && !pdata->pll_frequency)
		dev_err(dev, "DAC pll enabled but missing 'adi,pll-frequency'\n");

	tmp = JESD204_SYSREF_ONESHOT;
	of_property_read_u32(np, "adi,sysref-mode", &tmp);
	pdata->sync_mode = tmp;

	if (pdata->sync_mode == JESD204_SYSREF_CONTINUOUS && !pdata->jesd_subclass)
		dev_warn(dev, "Continuous sync mode can only be used in Subclass 1\n");

	/*
	 * DO NOT copy this. It is as wrong as it gets, we have to do it to
	 * preserve backwards compatibility with earlier versions of the driver
	 * that only supported 3 wire mode.
	 */
	pdata->spi4wire = of_property_read_bool(np, "adi,spi-4wire");

	return pdata;
}
#else
static
struct ad9144_platform_data *ad9144_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static const struct regmap_config ad9144_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	/* TODO: Add volatile/writeable registers tables */
	.cache_type = REGCACHE_NONE,
};

static int ad9144_reset(struct ad9144_state *st, bool spi4wire)
{
	int ret;

	msleep(5);

	ret = regmap_write(st->map, REG_SPI_INTFCONFA, SOFTRESET_M | SOFTRESET);
	if (ret < 0)
		return ret;
	ret = regmap_write(st->map, REG_SPI_INTFCONFA, spi4wire ? 0x18 : 0x00);

	msleep(4);

	return ret;
}

static int ad9144_jesd204_link_init(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason,
				    struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9144_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9144_state *st = priv->st;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	jesd204_copy_link_params(lnk, &st->link_config);

	lnk->sample_rate = ad9144_get_sample_rate(st);
	lnk->sample_rate_div = st->interpolation;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;
	lnk->jesd_version = JESD204_VERSION_B;
	lnk->is_transmit = true;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9144_jesd204_link_setup(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9144_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9144_state *st = priv->st;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	/*Enable Link*/

	ret = ad9144_setup(st, lnk);
	if (ret != 0) {
		dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
		return -EFAULT;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9144_jesd204_link_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9144_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9144_state *st = priv->st;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	/*Enable Link*/
	ret = regmap_write(st->map, 0x300, reason == JESD204_STATE_OP_REASON_INIT);
	if (ret != 0) {
		dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
		return -EFAULT;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9144_jesd204_link_running(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9144_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9144_state *st = priv->st;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	ret = ad9144_link_status_get(st);
	if (ret) {
		dev_err(dev, "Failed JESD204 link status (%d)\n", ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9144_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9144_jesd204_link_init,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_link = ad9144_jesd204_link_setup,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9144_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9144_jesd204_link_running,
		},
	},

	.max_num_links = 1,
	.num_retries = 2,
	.sizeof_priv = sizeof(struct ad9144_jesd204_priv),
};

static int ad9144_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9144_platform_data *pdata;
	struct ad9144_state *st;
	unsigned long lane_rate_kHz;
	bool spi4wire;
	unsigned int idl, idh, grade;
	unsigned int i;
	int ret;

	if (spi->dev.of_node)
		pdata = ad9144_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	switch (pdata->interpolation) {
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		dev_err(&spi->dev, "Invalid interpolation factor: %u\n",
			pdata->interpolation);
		return -EINVAL;
	}

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9144_init);
	if (IS_ERR(st->jdev))
		return PTR_ERR(st->jdev);

	if (st->jdev) {
		struct ad9144_jesd204_priv *priv;

		priv = jesd204_dev_priv(st->jdev);
		priv->st = st;
	}

	st->id = (enum chip_id) dev_id->driver_data;
	st->interpolation = pdata->interpolation;
	st->fcenter_shift = pdata->fcenter_shift;
	conv = &st->conv;

	st->pll_enable = pdata->pll_enable;
	st->pll_frequency = pdata->pll_frequency;

	switch (st->id) {
	case CHIPID_AD9144:
	case CHIPID_AD9152:
		/* Crazy mode to keep backwards compatibility */
		spi4wire = pdata->spi4wire;
		break;
	default:
		/* Normal mode for all other devices */
		spi4wire = !(spi->mode & SPI_3WIRE);
		break;
	}

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	for (i = 0; i < 2; i++) {
		conv->txen_gpio[i] = devm_gpiod_get_index_optional(&spi->dev,
			"txen", i, GPIOD_OUT_HIGH);
		if (IS_ERR(conv->txen_gpio[i]))
			return PTR_ERR(conv->txen_gpio[i]);
	}

	st->map = devm_regmap_init_spi(spi, &ad9144_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	ret = ad9144_reset(st, spi4wire);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->map, REG_SPI_PRODIDL, &idl);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->map, REG_SPI_PRODIDH, &idh);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->map, REG_SPI_CHIPGRADE, &grade);
	if (ret < 0)
		return ret;
	grade >>= 4; /* grade is in the 4 MSBs */

	if (st->id != AD9144_CHIPID(idh, idl, grade)) {
		dev_err(&spi->dev,
		    "Unrecognized CHIP_ID 0x%.2X%.2X and CHIP_GRADE 0x%x\n",
		    idh, idl, grade);
		ret = -ENODEV;
		goto out;
	}

	regmap_write(st->map, REG_SPI_SCRATCHPAD, 0xAD);
	regmap_read(st->map, REG_SPI_SCRATCHPAD, &idl);
	if (idl != 0xAD)
		return -EIO;

	conv->write = ad9144_write;
	conv->read = ad9144_read;
	conv->setup = ad9144_prepare;

	conv->get_data_clk = ad9144_get_data_clk;
	conv->write_raw = ad9144_write_raw;
	conv->read_raw = ad9144_read_raw;
	conv->spi = spi;
	conv->id = ID_AUTO_SYNTH_PARAM; /* generate channel list automatically */

	ret = ad9144_get_clks(conv);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	st->num_lanes = ad9144_jesd_modes[pdata->jesd_link_mode].l;
	st->num_converters = ad9144_jesd_modes[pdata->jesd_link_mode].m;
	st->octets_per_frame = ad9144_jesd_modes[pdata->jesd_link_mode].f;

	st->link_config.device_id = 0;
	st->link_config.bank_id = 0;
	st->link_config.num_lanes = st->num_lanes;
	st->link_config.num_converters = st->num_converters;
	st->link_config.octets_per_frame = st->octets_per_frame;
	st->link_config.frames_per_multiframe = 32;
	st->link_config.samples_per_conv_frame = ad9144_jesd_modes[pdata->jesd_link_mode].s;

	st->link_config.high_density = ad9144_jesd_modes[pdata->jesd_link_mode].hd;
	st->link_config.scrambling = true;
	st->link_config.subclass = pdata->jesd_subclass;
	st->link_config.sysref.mode = pdata->sync_mode;
	st->link_config.bits_per_sample = ad9144_jesd_modes[pdata->jesd_link_mode].np;
	st->link_config.converter_resolution = ad9144_jesd_modes[pdata->jesd_link_mode].n;

	for (i = 0; i < 8; i++)
		st->lane_mux[i] = pdata->xbar_lane_sel[i];

	ret = ad9144_update_sysref(st, ad9144_get_sample_rate(st));
	if (ret < 0)
		return ret;

	/* No need to continue here when jesd204-fsm enabled */
	if (st->jdev)
		goto done;

	lane_rate_kHz = ad9144_get_lane_rate(st, ad9144_get_sample_rate(st));
	dev_dbg(&spi->dev, "Setting lane rate %ld kHz\n", lane_rate_kHz);

	ret = clk_set_rate(conv->clk[0], lane_rate_kHz);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to set lane rate to %ld kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->clk[0]);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	ret = ad9144_setup(st, &st->link_config);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	ad9144_link_status_get(st);

done:
	spi_set_drvdata(spi, conv);
	dev_dbg(&spi->dev, "Probed.\n");
	return jesd204_fsm_start(st->jdev, JESD204_LINKS_ALL);
out:
	return ret;
}

static const struct spi_device_id ad9144_id[] = {
	{ "ad9135", CHIPID_AD9135 },
	{ "ad9136", CHIPID_AD9136 },
	{ "ad9144", CHIPID_AD9144 },
	{ "ad9152", CHIPID_AD9152 },
	{ "ad9154", CHIPID_AD9154 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad9144_id);

static struct spi_driver ad9144_driver = {
	.driver = {
		   .name = "ad9144",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9144_probe,
	.id_table = ad9144_id,
};

module_spi_driver(ad9144_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9144 DAC");
MODULE_LICENSE("GPL v2");
