// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ltc6948 SPI Fractional-N Synthesizer with Integrated VCO
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gcd.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* Register map defines */
/* REG 0x0 */
#define LTC6948_MSK_UNLOCK              BIT(5)
#define LTC6948_MSK_ALCHI               BIT(4)
#define LTC6948_MSK_ALCLO               BIT(3)
#define LTC6948_MSK_LOCK                BIT(2)
#define LTC6948_MSK_THI                 BIT(1)
#define LTC6948_MSK_TLO                 BIT(0)

/* REG 0x1 */
#define LTC6948_MSK_X                   GENMASK(5, 0)

/* REG 0x2 */
#define LTC6948_MSK_PDALL               BIT(7)
#define LTC6948_MSK_PDPLL               BIT(6)
#define LTC6948_MSK_PDVCO               BIT(5)
#define LTC6948_MSK_PDOUT               BIT(4)
#define LTC6948_MSK_PDFN                BIT(3)
#define LTC6948_MSK_MTCAL               BIT(2)
#define LTC6948_MSK_OMUTE               BIT(1)
#define LTC6948_MSK_POR                 BIT(0)

/* REG 0x3 */
#define LTC6948_MSK_ALCEN               BIT(7)
#define LTC6948_MSK_ALCMON              BIT(6)
#define LTC6948_MSK_ALCCAL              BIT(5)
#define LTC6948_MSK_ALCULOK             BIT(4)
#define LTC6948_MSK_AUTOCAL             BIT(3)
#define LTC6948_MSK_AUTORST             BIT(2)
#define LTC6948_MSK_DITHEN              BIT(1)
#define LTC6948_MSK_INTN                BIT(0)

/* REG 0x4 */
#define LTC6948_MSK_BD                  GENMASK(7, 4)
#define LTC6948_MSK_CPLE                BIT(3)
#define LTC6948_MSK_LDOEN               BIT(2)
#define LTC6948_MSK_LDOV                GENMASK(1, 0)

/* REG 0x6 .. 0x7 */
#define LTC6948_MSK_RD                  GENMASK(7, 3)
#define LTC6948_MSK_ND_98               GENMASK(1, 0)

/* REG 0x8 .. 0xA */
#define LTC6948_MSK_NUM_1712            GENMASK(5, 0)
#define LTC6948_MSK_NUM_114             GENMASK(7, 0)
#define LTC6948_MSK_NUM_03              GENMASK(7, 4)
#define LTC6948_MSK_RSTFN               BIT(1)
#define LTC6948_MSK_CAL                 BIT(0)

/* REG 0xB */
#define LTC6948_MSK_BST                 BIT(7)
#define LTC6948_MSK_FILT                GENMASK(6, 5)
#define LTC6948_MSK_RFO                 GENMASK(4, 3)
#define LTC6948_MSK_OD                  GENMASK(2, 0)

/* REG 0xC */
#define LTC6948_MSK_LKWIN               GENMASK(7, 5)
#define LTC6948_MSK_LKCT                GENMASK(4, 3)
#define LTC6948_MSK_CP                  GENMASK(2, 0)

/* REG 0xD */
#define LTC6948_MSK_CPCHI               BIT(7)
#define LTC6948_MSK_CPCLO               BIT(6)
#define LTC6948_MSK_CPMID               BIT(5)
#define LTC6948_MSK_CPINV               BIT(4)
#define LTC6948_MSK_CPWIDE              BIT(3)
#define LTC6948_MSK_CPRST               BIT(2)
#define LTC6948_MSK_CPUP                BIT(1)
#define LTC6948_MSK_CPDN                BIT(0)

/* REG 0xE */
#define LTC6948_MSK_REV                 GENMASK(7, 4)
#define LTC6948_MSK_PART                GENMASK(3, 0)

#define LTC6948_BREG(x)                 (st->buf[LTC6948_REG(x)])

/* Registers address macro */
#define LTC6948_REG(x)                  (x)
#define LTC6948_REG_HW(x)               ((x) << 1)
#define LTC6948_MODULUS                 BIT(18)

#define LTC6948_1_MAXFREQ 3740000000ULL  //!< LTC6948-1 upper freq limit
#define LTC6948_2_MAXFREQ 4910000000ULL   //!< LTC6948-2 upper freq limit
#define LTC6948_3_MAXFREQ 5790000000ULL  //!< LTC6948-3 upper freq limit
#define LTC6948_4_MAXFREQ 6390000000ULL   //!< LTC6948-4 upper freq limit

#define LTC6948_1_MINFREQ 2240000000ULL   //!< LTC6948-1 lower freq limit
#define LTC6948_2_MINFREQ 3080000000ULL   //!< LTC6948-2 lower freq limit
#define LTC6948_3_MINFREQ 3840000000ULL   //!< LTC6948-3 lower freq limit
#define LTC6948_4_MINFREQ 4200000000ULL   //!< LTC6948-4 lower freq limit

#define LTC6948_1_FCALMAXFREQ 1000000   //!< LTC6948-1 cal-max freq limit
#define LTC6948_2_FCALMAXFREQ 1330000   //!< LTC6948-2 cal-max  freq limit
#define LTC6948_3_FCALMAXFREQ 1700000   //!< LTC6948-3 cal-max  freq limit
#define LTC6948_4_FCALMAXFREQ 1800000   //!< LTC6948-3 cal-max  freq limit

#define LTC6948_MIN_REF_FREQ 10000000  //!< LTC6948 lower reference frequency limit
#define LTC6948_MAX_REF_FREQ 425000000 //!< LTC6948 upper reference frequency limit

#define MAX_FPFD_INTEGER 100000000UL
#define MAX_FPFD_FRACT 76100000UL

#define MAX_INT_INTEGER 1023
#define MAX_INT_FRACT   1019
#define MIN_INT_INTEGER 32
#define MIN_INT_FRACT   35

static const struct regmap_config ltc6948_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
};

struct child_clk {
	struct clk_hw           hw;
	struct ltc6948_state    *st;
	bool                    enabled;
	struct clock_scale      scale;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

struct ltc6948_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *clkin;
	unsigned long clkin_freq;

	unsigned long long max_vco_freq;
	unsigned long long min_vco_freq;
	unsigned long max_fcal_freq;

	u8 part_id, rev;

	u8 reg_1_default;
	u8 reg_3_default;
	u8 reg_4_default;
	u8 reg_b_default;
	u8 reg_c_default;
	u8 reg_d_default;

	u8 buf[15] __aligned(IIO_DMA_MINALIGN);
};

static unsigned int ltc6948_fref_valid(u32 fref)
{
	if (fref < LTC6948_MIN_REF_FREQ || fref > LTC6948_MAX_REF_FREQ)
		return -EINVAL;

	return 0;
};

static unsigned int ltc6948_filt(u32 fref)
{
	if (fref < 20000000)
		return 3;
	else if (fref > 50000000)
		return 0;
	else
		return 1;
};

static int ltc6948_get_b_and_bd(unsigned int b, unsigned int *bd, unsigned int *b_value)
{
	switch (b) {
	case 0 ... 8:
		*bd = 0;
		*b_value = 8;
		break;
	case 9 ... 12:
		*bd = 1;
		*b_value = 12;
		break;
	case 13 ... 16:
		*bd = 2;
		*b_value = 16;
		break;
	case 17 ... 24:
		*bd = 3;
		*b_value = 24;
		break;
	case 25 ... 32:
		*bd = 4;
		*b_value = 32;
		break;
	case 33 ... 48:
		*bd = 5;
		*b_value = 48;
		break;
	case 49 ... 64:
		*bd = 6;
		*b_value = 64;
		break;
	case 65 ... 96:
		*bd = 7;
		*b_value = 96;
		break;
	case 97 ... 128:
		*bd = 8;
		*b_value = 128;
		break;
	case 129 ... 192:
		*bd = 9;
		*b_value = 192;
		break;
	case 193 ... 256:
		*bd = 10;
		*b_value = 256;
		break;
	case 257 ... 384:
		*bd = 11;
		*b_value = 384;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned int ltc6948_lkwin_fract(unsigned int fvco, bool cple)
{
	if (cple) {
		if (fvco >= 2970000000UL)
			return 0;
		else if (fvco >= 2000000000UL)
			return 1;
		else if (fvco >= 1390000000UL)
			return 2;
		else if (fvco >= 941000000UL)
			return 3;
		else if (fvco >= 646000000UL)
			return 4;
		else if (fvco >= 431000000UL)
			return 5;
		else if (fvco >= 294000000UL)
			return 6;
		else
			return 7;
	} else {
		if (fvco >= 1350000000UL)
			return 0;
		else if (fvco >= 919000000UL)
			return 1;
		else if (fvco >= 632000000UL)
			return 2;
		else if (fvco >= 428000000UL)
			return 3;
		else if (fvco >= 294000000UL)
			return 4;
		else if (fvco >= 196000000UL)
			return 5;
		else if (fvco >= 134000000UL)
			return 6;
		else
			return 7;
	}
}

static unsigned int ltc6948_lkwin_integer(unsigned int fpfd)
{
	if (fpfd > 6800000)
		return 0;
	else if (fpfd > 4700000)
		return 1;
	else if (fpfd > 3200000)
		return 2;
	else if (fpfd > 2200000)
		return 3;
	else if (fpfd > 1500000)
		return 4;
	else if (fpfd > 1000000)
		return 5;
	else if (fpfd > 660000)
		return 6;
	else
		return 7;
}

static void ltc6948_get_ldov_ldoen(unsigned int fpfd, unsigned int *ldov, bool *ldoen)
{
	if (fpfd <= 34300000UL) {
		*ldov = 0;
		*ldoen = true;
	} else if (fpfd <= 45900000UL) {
		*ldov = 1;
		*ldoen = true;
	} else if (fpfd <= 56100000UL) {
		*ldov = 2;
		*ldoen = true;
	} else if (fpfd <= 66300000UL) {
		*ldov = 3;
		*ldoen = true;
	} else {
		*ldov = 0;
		*ldoen = false;
	}
}

static unsigned int ltc6948_compute_cp(unsigned int icp)
{
	unsigned int cp;

	if (icp < 1400)
		cp = 0;
	else if (icp < 2000)
		cp = 1;
	else if (icp < 2800)
		cp = 2;
	else if (icp < 4000)
		cp = 3;
	else if (icp < 5600)
		cp = 4;
	else if (icp < 8000)
		cp = 5;
	else if (icp < 11200)
		cp = 6;
	else
		cp = 7; // Default value

	return cp;
}

static int ltc6948_set_freq(struct ltc6948_state *st, unsigned long parent_rate,
			    unsigned long long freq, unsigned long long *round_rate)
{
	u64 vco_freq, integer;
	u32 b_min, bd_val, b_value, ldov, lkwin, odiv, rdiv, fpfd, fract;
	int ret;
	bool ldoen;

	ret = ltc6948_fref_valid(parent_rate);
	if (ret < 0) {
		dev_err(&st->spi->dev, "Fref frequency out of range %lu Hz\n",  parent_rate);
		return ret;
	}

	odiv = DIV_ROUND_UP_ULL(st->min_vco_freq, freq);
	vco_freq = freq * odiv;

	if (vco_freq < st->min_vco_freq || vco_freq > st->max_vco_freq) {
		dev_err(&st->spi->dev, "VCO frequency out of range %llu Hz\n", vco_freq);
		return -EINVAL;
	}

	rdiv = DIV_ROUND_UP(parent_rate, MAX_FPFD_INTEGER); /* Assume integer mode for now */
	fpfd = parent_rate / rdiv;

	integer = vco_freq;
	fract = do_div(integer, fpfd);
	if (fract) {
		rdiv = DIV_ROUND_UP(parent_rate, MAX_FPFD_FRACT); /* Fract mode max fpfd = 76.1 MHz */
		fpfd = parent_rate / rdiv;

		integer = vco_freq;
		fract = do_div(integer, fpfd);
		fract = DIV_ROUND_CLOSEST_ULL(fract * LTC6948_MODULUS, fpfd);
		if (fract > LTC6948_MODULUS) {
			dev_err(&st->spi->dev, "Fract-N %u modulus out of range\n", fract);
			return -EINVAL;
		}
	}

	if (fract == 0) {
		if (integer > MAX_INT_INTEGER || integer < MIN_INT_INTEGER) {
			dev_err(&st->spi->dev, "N-Divider %llu out of range for integer mode\n", integer);
			return -EINVAL;
		}
		st->buf[LTC6948_REG(0x3)] = st->reg_3_default | LTC6948_MSK_INTN;
		lkwin = ltc6948_lkwin_integer(fpfd);
	} else {
		if (integer > MAX_INT_FRACT || integer < MIN_INT_FRACT) {
			dev_err(&st->spi->dev, "N-Divider %llu out of range for fractional mode\n", integer);
			return -EINVAL;
		}
		st->buf[LTC6948_REG(0x3)] = st->reg_3_default;
		lkwin = ltc6948_lkwin_fract(vco_freq, !!(st->reg_4_default & LTC6948_MSK_CPLE));
	}

	if (round_rate) {
		u64 val;

		val = ((integer * LTC6948_MODULUS) + fract) * fpfd;
		do_div(val, LTC6948_MODULUS * odiv);

		*round_rate = val;

		return 0;
	}

	b_min = DIV_ROUND_UP(fpfd, st->max_fcal_freq);
	ret = ltc6948_get_b_and_bd(b_min, &bd_val, &b_value);
	if (ret < 0) {
		dev_err(&st->spi->dev, "B value out of range\n");
		return ret;
	}

	ltc6948_get_ldov_ldoen(fpfd, &ldov, &ldoen);
	st->buf[LTC6948_REG(0x4)] = FIELD_PREP(LTC6948_MSK_BD, bd_val) |
				    FIELD_PREP(LTC6948_MSK_LDOV, ldov) |
				    FIELD_PREP(LTC6948_MSK_LDOEN, ldoen) |
				    st->reg_4_default;

	st->buf[LTC6948_REG(0x5)] = 0x11 /* default SEED */;
	st->buf[LTC6948_REG(0x6)] = FIELD_PREP(LTC6948_MSK_ND_98, integer >> 8) | (rdiv << 3);
	st->buf[LTC6948_REG(0x7)] = integer & 0xFF;
	st->buf[LTC6948_REG(0x8)] = FIELD_PREP(LTC6948_MSK_NUM_1712, fract >> 12);
	st->buf[LTC6948_REG(0x9)] = FIELD_PREP(LTC6948_MSK_NUM_114, fract >> 4);
	st->buf[LTC6948_REG(0xA)] = FIELD_PREP(LTC6948_MSK_NUM_03, fract);

	st->buf[LTC6948_REG(0xB)] = st->reg_b_default |
				    FIELD_PREP(LTC6948_MSK_FILT, ltc6948_filt(parent_rate)) |
				    FIELD_PREP(LTC6948_MSK_OD, odiv);

	st->buf[LTC6948_REG(0xC)] = st->reg_c_default |
				    FIELD_PREP(LTC6948_MSK_LKWIN, lkwin);

	ret = regmap_raw_write(st->regmap, LTC6948_REG_HW(0x3), &st->buf[LTC6948_REG(0x3)], 10);
	if (ret < 0)
		return ret;

	dev_dbg(&st->spi->dev,
		"odiv %d rdiv %d fpfd %d integer %llu fract %d lkwin %d ldov %d ldoen %d\n",
		odiv, rdiv, fpfd, integer, fract, lkwin, ldov, ldoen);

	return 0;
}

static int ltc6948_pll_fract_n_get_rate(struct ltc6948_state *st, unsigned long parent_rate, u64 *freq)
{
	unsigned long long val = 0;
	u32 o_div, fpfd, ref_div;
	u64 integer, fract;
	int ret;

	ret = regmap_raw_read(st->regmap, LTC6948_REG_HW(0x6), &st->buf[LTC6948_REG(0x6)], 6);
	if (ret < 0)
		return ret;

	o_div = FIELD_GET(LTC6948_MSK_OD, LTC6948_BREG(0xB));
	ref_div = FIELD_GET(LTC6948_MSK_RD, LTC6948_BREG(0x6));
	integer = (FIELD_GET(LTC6948_MSK_ND_98, LTC6948_BREG(0x6)) << 8) |
		  LTC6948_BREG(0x7);
	fract = (FIELD_GET(LTC6948_MSK_NUM_1712, LTC6948_BREG(0x8)) << 12) |
		LTC6948_BREG(0x9) << 4 |
		(FIELD_GET(LTC6948_MSK_NUM_03, LTC6948_BREG(0xA)));

	if (!ref_div || !o_div)
		return -EINVAL;

	fpfd = st->clkin_freq / ref_div;

	val = ((integer * LTC6948_MODULUS) + fract) * fpfd;
	do_div(val, LTC6948_MODULUS * o_div);

	*freq = val;

	dev_dbg(&st->spi->dev, "o_div %d ref_div %d integer %llu fract %llu fpfd %d freq %llu\n",
		o_div, ref_div, integer, fract, fpfd, *freq);

	return 0;
}

static long ltc6948_clock_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	unsigned long long scaled_rate;
	int ret;

	scaled_rate = from_ccf_scaled(rate, &to_clk_priv(hw)->scale);
	ret = ltc6948_set_freq(to_clk_priv(hw)->st, *parent_rate, scaled_rate, &scaled_rate);
	if (ret < 0)
		return ret;

	return to_ccf_scaled(scaled_rate, &to_clk_priv(hw)->scale);
}

static unsigned long ltc6948_clock_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	unsigned long long rate;

	ltc6948_pll_fract_n_get_rate(to_clk_priv(hw)->st, parent_rate, &rate);

	return to_ccf_scaled(rate, &to_clk_priv(hw)->scale);
}

static int ltc6948_clock_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	return ltc6948_set_freq(to_clk_priv(hw)->st, parent_rate,
				from_ccf_scaled(rate, &to_clk_priv(hw)->scale), NULL);
}

static int ltc6948_clock_enable(struct clk_hw *hw)
{
	struct ltc6948_state *st = to_clk_priv(hw)->st;
	int ret;

	ret = regmap_clear_bits(st->regmap, LTC6948_REG_HW(0x2), LTC6948_MSK_OMUTE);
	if (!ret)
		to_clk_priv(hw)->enabled = true;

	return ret;
}

void ltc6948_clock_disable(struct clk_hw *hw)
{
	struct ltc6948_state *st = to_clk_priv(hw)->st;
	int ret;

	ret = regmap_set_bits(st->regmap, LTC6948_REG_HW(0x2), LTC6948_MSK_OMUTE);
	if (!ret)
		to_clk_priv(hw)->enabled = false;
}

static int ltc6948_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static int ltc6948_debugfs_show(struct seq_file *s, void *p)
{
	struct ltc6948_state *st = s->private;
	u32 status;
	int ret;

	ret = regmap_read(st->regmap, LTC6948_REG_HW(0), &status);
	if (ret < 0)
		return ret;

	seq_printf(s, "LTC6948-%u Rev.%u Status: PLL %s%s%s%s%s\n", st->part_id, st->rev,
		   (status & LTC6948_MSK_LOCK) ? " Locked" : "Unlocked",
		   (status & LTC6948_MSK_ALCHI) ? " ALC too high" : "",
		   (status & LTC6948_MSK_ALCLO) ? " ALC too low" : "",
		   (status & LTC6948_MSK_THI) ? " CP too high" : "",
		   (status & LTC6948_MSK_TLO) ? " CP too low" : "");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ltc6948_debugfs);

static void ltc6948_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	debugfs_create_file("status", 0444, dentry, to_clk_priv(hw)->st, &ltc6948_debugfs_fops);
}
#else
#define ltc6948_debug_init NULL
#endif

static const struct clk_ops ltc6948_clock_ops = {
	.set_rate = ltc6948_clock_set_rate,
	.recalc_rate = ltc6948_clock_recalc_rate,
	.round_rate = ltc6948_clock_round_rate,
	.enable = ltc6948_clock_enable,
	.disable = ltc6948_clock_disable,
	.is_enabled = ltc6948_clk_is_enabled,
	.debug_init = ltc6948_debug_init,
};

static void ltc6948_of_clk_del_provider(void *data)
{
	struct device *dev = data;

	of_clk_del_provider(dev->of_node);
}

static int ltc6948_clk_register(struct ltc6948_state *st)
{
	struct child_clk *clk_priv;
	struct clk_init_data init;
	const char *parent_name;
	const char *clk_name;
	struct clk *clk_out;
	struct spi_device *spi = st->spi;
	int ret;

	if (!IS_ENABLED(CONFIG_OF))
		return 0;

	clk_priv = devm_kzalloc(&spi->dev, sizeof(*clk_priv), GFP_KERNEL);
	if (!clk_priv)
		return -ENOMEM;

	/* struct child_clk assignments */
	clk_priv->hw.init = &init;
	clk_priv->st = st;
	clk_name = spi->dev.of_node->name;
	of_property_read_string(spi->dev.of_node, "clock-output-names",
				&clk_name);

	ret = of_clk_get_scale(spi->dev.of_node,
			       NULL, &clk_priv->scale);
	if (ret < 0) {
		clk_priv->scale.mult = 1;
		clk_priv->scale.div = 10;
	}

	init.name = clk_name;
	init.ops = &ltc6948_clock_ops;
	init.flags = 0;
	parent_name = __clk_get_name(st->clkin);
	init.parent_names = &parent_name;
	init.num_parents = 1;
	clk_out = devm_clk_register(&spi->dev, &clk_priv->hw);
	if (IS_ERR(clk_out))
		return PTR_ERR(clk_out);

	ret = of_clk_add_provider(spi->dev.of_node,
				  of_clk_src_simple_get, clk_out);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&spi->dev,
					ltc6948_of_clk_del_provider, &spi->dev);
}

static int ltc6948_parse_dt(struct ltc6948_state *st)
{
	u32 val;

	if (device_property_read_bool(&st->spi->dev, "adi,ref-boost-enable"))
		st->reg_b_default |= LTC6948_MSK_BST;

	val = 3; /* default to max */
	device_property_read_u32(&st->spi->dev, "adi,output-power", &val);
	st->reg_b_default |= FIELD_PREP(LTC6948_MSK_RFO, val);

	val = 1; /* 32 */
	device_property_read_u32(&st->spi->dev, "adi,lock-count", &val);
	st->reg_c_default = FIELD_PREP(LTC6948_MSK_LKCT, val);

	val = 5600;
	device_property_read_u32(&st->spi->dev, "adi,charge-pump-current-uA", &val);
	st->reg_c_default |= FIELD_PREP(LTC6948_MSK_CP, ltc6948_compute_cp(val));

	st->reg_3_default = LTC6948_MSK_ALCEN | LTC6948_MSK_ALCMON |
			    LTC6948_MSK_ALCCAL | LTC6948_MSK_AUTOCAL |
			    LTC6948_MSK_AUTORST;

	if (device_property_read_bool(&st->spi->dev, "adi,dither-enable"))
		st->reg_3_default |= LTC6948_MSK_DITHEN;

	val = 0; /* disabled */
	device_property_read_u32(&st->spi->dev, "adi,status-output-or-mask", &val);
	st->reg_1_default = FIELD_PREP(LTC6948_MSK_X, val);

	st->reg_4_default = LTC6948_MSK_CPLE;
	if (device_property_read_bool(&st->spi->dev, "adi,cp-linearizer-disable"))
		st->reg_4_default = 0;

	if (device_property_read_bool(&st->spi->dev, "adi,cp-extended-pulse-width-enable"))
		st->reg_d_default |= LTC6948_MSK_CPWIDE;

	if (device_property_read_bool(&st->spi->dev, "adi,cp-phase-invert-enable"))
		st->reg_d_default |= LTC6948_MSK_CPINV;

	return 0;
}

static int ltc6948_probe(struct spi_device *spi)
{
	struct ltc6948_state *st;
	struct regmap *regmap;
	int ret;
	u32 val;

	regmap = devm_regmap_init_spi(spi, &ltc6948_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	spi_set_drvdata(spi, st);
	st->spi = spi;
	st->regmap = regmap;

	ret = ltc6948_parse_dt(st);
	if (ret < 0)
		return ret;

	st->clkin = devm_clk_get_enabled(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return PTR_ERR(st->clkin);

	st->clkin_freq = clk_get_rate(st->clkin);

	/* Force Power-On-Reset */
	regmap_write(st->regmap, LTC6948_REG_HW(0x2), LTC6948_MSK_POR |
		     LTC6948_MSK_OMUTE | LTC6948_MSK_MTCAL);
	fsleep(10);
	regmap_write(st->regmap, LTC6948_REG_HW(0x2), LTC6948_MSK_OMUTE |
		     LTC6948_MSK_MTCAL);

	regmap_read(st->regmap, LTC6948_REG_HW(0xE), &val);

	st->part_id = FIELD_GET(LTC6948_MSK_PART, val);
	st->rev = FIELD_GET(LTC6948_MSK_REV, val);

	switch (st->part_id) {
	case 1:
		st->max_vco_freq = LTC6948_1_MAXFREQ;
		st->min_vco_freq = LTC6948_1_MINFREQ;
		st->max_fcal_freq = LTC6948_1_FCALMAXFREQ;
		break;
	case 2:
		st->max_vco_freq = LTC6948_2_MAXFREQ;
		st->min_vco_freq = LTC6948_2_MINFREQ;
		st->max_fcal_freq = LTC6948_2_FCALMAXFREQ;
		break;
	case 3:
		st->max_vco_freq = LTC6948_3_MAXFREQ;
		st->min_vco_freq = LTC6948_3_MINFREQ;
		st->max_fcal_freq = LTC6948_3_FCALMAXFREQ;
		break;
	case 4:
		st->max_vco_freq = LTC6948_4_MAXFREQ;
		st->min_vco_freq = LTC6948_4_MINFREQ;
		st->max_fcal_freq = LTC6948_4_FCALMAXFREQ;
		break;
	default:
		dev_err(&spi->dev, "Invalid part id\n");
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, LTC6948_REG_HW(0x1), st->reg_1_default);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, LTC6948_REG_HW(0xD), st->reg_d_default);
	if (ret < 0)
		return ret;

	ret = ltc6948_clk_register(st);
	if (ret < 0)
		return ret;

	dev_info(&spi->dev, "LTC6948-%u Rev.%u successfully initialized\n", st->part_id, st->rev);

	return ret;
}

static const struct spi_device_id ltc6948_id_table[] = {
	{ "ltc6948", 0 },
	{ "ltc6948-1", 1 },
	{ "ltc6948-2", 2 },
	{ "ltc6948-3", 3 },
	{ "ltc6948-4", 4 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6948_id_table);

static const struct of_device_id ltc6948_of_match[] = {
	{ .compatible = "adi,ltc6948" },
	{ .compatible = "adi,ltc6948-1" },
	{ .compatible = "adi,ltc6948-2" },
	{ .compatible = "adi,ltc6948-3" },
	{ .compatible = "adi,ltc6948-4" },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc6948_of_match);

static struct spi_driver ltc6948_driver = {
	.driver = {
		.name = "ltc6948",
		.of_match_table = ltc6948_of_match,
	},
	.probe = ltc6948_probe,
	.id_table = ltc6948_id_table,
};
module_spi_driver(ltc6948_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6948 Synthesizer");
MODULE_LICENSE("GPL");
