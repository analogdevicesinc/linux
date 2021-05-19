// SPDX-License-Identifier: GPL-2.0
/*
 * ADF4159 SPI Wideband Synthesizer driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/clk-provider.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/clk/clkscale.h>

#include <dt-bindings/iio/frequency/adf4159.h>

/* REG0 Bit Definitions */
#define ADF4159_REG0_INT(x)			(((x) & 0xFFF) << 15)
#define ADF4159_REG0_FRACT_MSB(x)		(((x) & 0xFFF) << 3)
#define ADF4159_REG0_MUXOUT(x)			(((x) & 0xF) << 27)
#define ADF4159_REG0_RAMP_ON(x)			(((x) & 0x1) << 31)

/* REG1 Bit Definitions */
#define ADF4159_REG1_FRACT_LSB(x)		(((x) & 0x1FFF) << 15)
#define ADF4159_REG1_PHASE(x)			(((x) & 0xFFF) << 3)
#define ADF4159_REG1_PHASE_ADJ_EN(x)		(((x) & 0x1) << 28)

/* REG2 Bit Definitions */
#define ADF4159_REG2_CLK1_DIV(x)		(((x) & 0xFFF) << 3)
#define ADF4159_REG2_R_CNT(x)			(((x) & 0x1F) << 15)
#define ADF4159_REG2_REF_DOUBLER_EN(x)		(((x) & 0x1) << 20)
#define ADF4159_REG2_REF_DIV2_EN(x)		(((x) & 0x1) << 21)
#define ADF4159_REG2_PRESCALER_89_EN(x)		(((x) & 0x1) << 22)
#define ADF4159_REG2_CP_CURRENT(x)		(((x) & 0xF) << 24)
#define ADF4159_REG2_CSR_EN(x)			(((x) & 0x1) << 28)

/* REG3 Bit Definitions */
#define ADF4159_REG3_CNT_RESET_EN(x)		(((x) & 0x1) << 3)
#define ADF4159_REG3_CP_THREE_STATE_EN(x)	(((x) & 0x1) << 4)
#define ADF4159_REG3_POWER_DOWN_EN(x)		(((x) & 0x1) << 5)
#define ADF4159_REG3_PD_POLARITY_POS_EN(x)	(((x) & 0x1) << 6)
#define ADF4159_REG3_LDP_6NS_EN(x)		(((x) & 0x1) << 7)
#define ADF4159_REG3_FSK_EN(x)			(((x) & 0x1) << 8)
#define ADF4159_REG3_PSK_EN(x)			(((x) & 0x1) << 9)
#define ADF4159_REG3_RAMP_MODE(x)		(((x) & 0x3) << 10)
#define ADF4159_REG3_SD_RESET_DIS(x)		(((x) & 0x1) << 14)
#define ADF4159_REG3_LOL_DIS(x)			(((x) & 0x1) << 16)
#define ADF4159_REG3_NEG_BLEED_EN(x)		(((x) & 0x1) << 21)
#define ADF4159_REG3_NEG_BLEED_CURR(x)		(((x) & 0x7) << 22)

/* REG4 Bit Definitions */
#define ADF4159_REG4_CLK_DIV2_SEL(x)		(((x) & 0x1) << 6)
#define ADF4159_REG4_CLK2_DIV(x)		(((x) & 0xFFF) << 7)
#define ADF4159_REG4_CLK_DIV_MODE(x)		(((x) & 0x3) << 19)
#define ADF4159_REG4_RAMP_STATUS(x)		(((x) & 0x1F) << 21)
#define ADF4159_REG4_SD_MOD_MODE(x)		(((x) & 0x1F) << 26)
#define ADF4159_REG4_LE_SEL_SYNC_REFIN_EN(x)	(((x) & 0x1) << 31)

/* REG5 Bit Definitions */
#define ADF4159_REG5_DEVIATION(x)		(((x) & 0xFFFF) << 3)
#define ADF4159_REG5_DEVIATION_OFFSET(x)	(((x) & 0xF) << 19)
#define ADF4159_REG5_DEV_SEL(x)			(((x) & 0x1) << 23)
#define ADF4159_REG5_DUAL_RAMP_EN(x)		(((x) & 0x1) << 24)
#define ADF4159_REG5_FSK_RAMP_EN(x)		(((x) & 0x1) << 25)
#define ADF4159_REG5_INTERRUPT_MODE(x)		(((x) & 0x3) << 26)
#define ADF4159_REG5_PARABOLIC_RAMP_EN(x)	(((x) & 0x1) << 28)
#define ADF4159_REG5_TXDATA_RAMP_CLK(x)		(((x) & 0x1) << 29)
#define ADF4159_REG5_TXDATA_INVERT(x)		(((x) & 0x1) << 30)

/* REG6 Bit Definitions */
#define ADF4159_REG6_STEP(x)			(((x) & 0xFFFFF) << 3)
#define ADF4159_REG6_STEP_SEL(x)		(((x) & 0x1) << 23)

/* REG7 Bit Definitions */
#define ADF4159_REG7_DELAY_START(x)		(((x) & 0xFFF) << 3)
#define ADF4159_REG7_DELAY_START_EN(x)		(((x) & 0x1) << 15)
#define ADF4159_REG7_DEL_CLK_SEL(x)		(((x) & 0x1) << 16)
#define ADF4159_REG7_RAMP_DEL_EN(x)		(((x) & 0x1) << 17)
#define ADF4159_REG7_RAMP_DEL_FL_EN(x)		(((x) & 0x1) << 18)
#define ADF4159_REG7_FAST_RAMP_EN(x)		(((x) & 0x1) << 19)
#define ADF4159_REG7_TXDATA_TRIG_EN(x)		(((x) & 0x1) << 20)
#define ADF4159_REG7_SING_FULL_TRI_EN(x)	(((x) & 0x1) << 21)
#define ADF4159_REG7_TRI_DELAY_EN(x)		(((x) & 0x1) << 22)
#define ADF4159_REG7_TXDATA_TRIG_DELAY_EN(x)	(((x) & 0x1) << 23)

/* Specifications */
#define ADF4159_MIN_FREQ		500000000ULL /* Hz */
#define ADF4159_MAX_FREQ		13000000000ULL /* Hz */
#define ADF4159_MAX_FREQ_PFD		110000000UL /* Hz */

/* Specifications */
#define ADF4169_MIN_FREQ		500000000ULL /* Hz */
#define ADF4169_MAX_FREQ		13500000000ULL /* Hz */
#define ADF4169_MAX_FREQ_PFD		130000000UL /* Hz */

#define ADF4159_MAX_FREQ_REFIN		260000000UL /* 260 MHz */
#define ADF4159_MAX_R_CNT		32
#define ADF4159_MAX_INT			4095
#define ADF4159_MIN_INT			23
#define ADF4159_MIN_INT_PRESCALER_89	75
#define ADF4159_MODULUS			33554432ULL
#define ADF4159_MAX_FRAC		(ADF4159_MODULUS - 1)
#define ADF4159_MAX_FREQ_PRESCALER_45	8000000000ULL /* 8 GHz */
#define ADF4159_MIN_CLKIN_DOUB_FREQ	10000000ULL /* Hz */
#define ADF4159_MAX_CLKIN_DOUB_FREQ	50000000ULL /* Hz */

/* Registers */
enum adf4159_reg {
	ADF4159_REG0,
	ADF4159_REG1,
	ADF4159_REG2,
	ADF4159_REG3,
	ADF4159_REG4,
	ADF4159_REG5,
	ADF4159_REG6,
	ADF4159_REG7,
	ADF4159_REG4_SEL1,
	ADF4159_REG5_SEL1,
	ADF4159_REG6_SEL1,
	ADF4159_REG_NUM,
};

enum {
	ADF4159_FREQ,
	ADF4159_PWRDOWN,
	ADF4159_FREQ_DEV_STEP,
	ADF4159_FREQ_DEV_RANGE,
	ADF4159_FREQ_DEV_TIME,
};

enum {
	ADF4159,
	ADF4169,
};

struct adf4159_config {
	u64 frequency;
	s32 frequency_deviation_step;
	u32 frequency_deviation_range;
	u32 frequency_deviation_time;

	/* REG2 */
	u32 clkin;
	u32 ref_div_factor;
	u32 muxout;
	u32 clk1_div;
	bool ramp_en;
	u32 phase_val;
	u32 cp_curr_uA;
	bool ref_doubler_en;
	bool ref_div2_en;
	bool csr_en;

	/* REG3 */
	bool pwdn_en;
	bool pd_pol_pos_en;
	bool ldp_6ns_en;
	bool fsk_en;
	bool psk_en;
	u32 ramp_mode;
	bool neg_bleed_en;
	u32 neg_bleed_curr;

	/* REG4 */
	u32 clk2_div[2];
	u32 clk_div_mode;
	u32 ramp_status;
	bool le_sel_sync_refin_en;

	/* REG5 */
	u32 deviation[2];
	u32 deviation_offs;
	bool dual_ramp_en;
	bool fsk_ramp_en;
	u32 interrupt_mode;
	bool parabolic_ramp_en;
	bool txdata_ramp_clk_txdata_en;
	bool single_full_tri_en;
	bool txdata_invert_en;

	/* REG6 */
	u32 step_word[2];

	/* REG7 */
	u32 delay_start_word;
	bool delay_start_en;
	bool delay_clk_sel_pfd_x_clk1_en;
	bool ramp_delay_en;
	bool ramp_delay_fl_en;
	bool fast_ramp_en;
	bool txdata_trig_en;
	bool txdata_trig_delay_en;
};

struct adf4159_state {
	struct spi_device	*spi;
	struct regulator		*reg;
	struct adf4159_config	*conf;
	struct clk		*clk;
	struct dentry		*dent;
	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	unsigned long		clkin;
	unsigned long long	fpfd; /* Phase Frequency Detector */
	unsigned long long	max_out_freq;
	unsigned long long	max_pfd_freq;

	u32			integer;
	u32			fract;
	u32			regs[ADF4159_REG_NUM];
	u32			clock_shift;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be32			val ____cacheline_aligned;
};

struct child_clk {
	struct clk_hw		hw;
	struct adf4159_state	*st;
	bool			enabled;
	struct clock_scale	scale;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

/*
 * Factorize, approximate N = CLK1 * CLK2 where CLK1,2 are 12-bit registers
 * Either CLK1 or CLK2 must be greater than 1,
 * that is, CLK1 = CLK2 = 1 is not allowed.
 */

static int adf4159_factorize_clk_divs(u32 n, u32 *clk1, u32 *clk2)
{
	int i, c1, c2, n_calc, delta, delta_max = 0xFFFFFFU;

	n = clamp(n, 2U, 0xFFFFFFU);

	for (i = 0; i <= 12; i++) {
		c1 = BIT(i);
		if (c1 > 0xFFF)
			c1--;

		c2 = DIV_ROUND_CLOSEST(n, c1);

		if (c2 > 0xFFF)
			continue;

		n_calc = c1 * c2;

		if (n == n_calc) {
			*clk1 = c1;
			*clk2 = c2;
			return 0;
		}

		delta = abs(n - n_calc);
		if (delta < delta_max) {
			*clk1 = c1;
			*clk2 = c2;
			delta_max = delta;
		}
	}

	return delta_max;
}

static int adf4159_spi_write(struct adf4159_state *st, u32 val)
{
	st->val = cpu_to_be32(val);

	dev_dbg(&st->spi->dev, "[%d] 0x%.8X\n", val & 0x7, val);

	return spi_write(st->spi, &st->val, 4);
}

static int adf4159_sync_config(struct adf4159_state *st)
{
	int ret, i;

	for (i = ADF4159_REG7; i >= ADF4159_REG0; i--) {
		ret = adf4159_spi_write(st, st->regs[i] | i);
		if (ret < 0)
			return ret;

		switch (i) {
		case ADF4159_REG4:
			ret = adf4159_spi_write(st,
				st->regs[ADF4159_REG4_SEL1] |
				ADF4159_REG4_CLK_DIV2_SEL(1) | i);
			if (ret < 0)
				return ret;
			break;
		case ADF4159_REG5:
			ret = adf4159_spi_write(st,
				st->regs[ADF4159_REG5_SEL1] |
				ADF4159_REG5_DEV_SEL(1) | i);
			if (ret < 0)
				return ret;
			break;
		case ADF4159_REG6:
			ret = adf4159_spi_write(st,
				st->regs[ADF4159_REG6_SEL1] |
				ADF4159_REG6_STEP_SEL(1) | i);
			if (ret < 0)
				return ret;
			break;
		default:
			break;
		}
	}

	return 0;
}

static int adf4159_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg, unsigned int writeval,
			      unsigned int *readval)
{
	struct adf4159_state *st = iio_priv(indio_dev);
	int ret;

	if (reg >= ADF4159_REG_NUM)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		st->regs[reg] = writeval & ~GENMASK(2, 0);
		ret = adf4159_sync_config(st);
	} else {
		*readval =  st->regs[reg];
		ret = 0;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int adf4159_pll_fract_n_compute(unsigned long long vco,
					unsigned int pfd,
					unsigned int *integer,
					unsigned int *fract)
{
	u64 tmp;

	tmp = do_div(vco, pfd);
	tmp = tmp * ADF4159_MODULUS;
	do_div(tmp, pfd);

	*integer = vco;
	*fract = tmp;

	return 0;
}

static unsigned long long adf4159_pll_fract_n_get_rate(struct adf4159_state *st)
{
	u64 val, tmp;

	val = (u64)st->integer * st->fpfd;
	tmp = (u64)st->fract * st->fpfd;
	do_div(tmp, ADF4159_MODULUS);

	val += tmp;

	return val;
}

static int adf4159_setup(struct adf4159_state *st,
	unsigned long parent_rate, unsigned long long freq)
{
	struct adf4159_config *conf = st->conf;
	u32 ref_div_factor, tmp;
	bool prescaler_89;
	u64 val;

	if (parent_rate)
		st->clkin = parent_rate;
	else
		st->clkin = clk_get_rate(st->clk);

	ref_div_factor = conf->ref_div_factor;

	if (conf->ref_doubler_en &&
		((st->clkin > ADF4159_MAX_CLKIN_DOUB_FREQ) ||
		(st->clkin < ADF4159_MIN_CLKIN_DOUB_FREQ)))
		conf->ref_doubler_en = false;

	/* Calculate and maximize PFD frequency */
	do {
		st->fpfd = (st->clkin * (conf->ref_doubler_en ? 2 : 1)) /
			   (ref_div_factor++ * (conf->ref_div2_en ? 2 : 1));
	} while (st->fpfd > st->max_pfd_freq);

	adf4159_pll_fract_n_compute(freq, st->fpfd, &st->integer, &st->fract);

	prescaler_89 = (freq > ADF4159_MAX_FREQ_PRESCALER_45);

	if (prescaler_89 && st->integer < ADF4159_MIN_INT_PRESCALER_89) {
		dev_err(&st->spi->dev,
			"N INT MIN vilolation %u < 75 for prescaler 8/9 (PFD %llu)",
			st->integer, st->fpfd);
		return -EINVAL;
	}

	conf->frequency = freq;

	if (conf->frequency_deviation_step) {
		bool neg = conf->frequency_deviation_step < 0;
		u32 step = abs(conf->frequency_deviation_step);
		int i;

		for (i = 0; i < 10; i++) {
			val = step * ADF4159_MODULUS;
			val = div64_u64(val, st->fpfd * (1 << i));
			if (val <= SHRT_MAX)
				break;
		}

		if (i == 10) {
			i = 9;
			val = SHRT_MAX;
		}

		conf->deviation[0] = (s16)(val * (neg ? -1 : 1));
		conf->deviation_offs = i;
	}

	if (conf->frequency_deviation_range && conf->deviation[0]) {
		val = st->fpfd * abs((s16)conf->deviation[0] & 0xFFFF) *
			(1 << (conf->deviation_offs & 0xF));
		do_div(val, ADF4159_MODULUS);
		val = DIV_ROUND_CLOSEST_ULL(
			(u64)conf->frequency_deviation_range, val);
		conf->step_word[0] = clamp(val, 0ULL, 0xFFFFFULL);
	}

	if (conf->frequency_deviation_time && conf->step_word[0]) {
		val = DIV_ROUND_CLOSEST_ULL((u64)conf->frequency_deviation_time,
			conf->step_word[0]);
		val = DIV_ROUND_CLOSEST_ULL(val * st->fpfd, 1000000U);
		adf4159_factorize_clk_divs(val, &conf->clk1_div,
			&conf->clk2_div[0]);
	}

	/* REG0 Bit Definitions */
	st->regs[ADF4159_REG0] =
		ADF4159_REG0_INT(st->integer) |
		ADF4159_REG0_FRACT_MSB(st->fract >> 13) |
		ADF4159_REG0_MUXOUT(conf->muxout) |
		ADF4159_REG0_RAMP_ON(conf->ramp_en);

	/* REG1 Bit Definitions */
	st->regs[ADF4159_REG1] =
		ADF4159_REG1_FRACT_LSB(st->fract & 0x1FFFF) |
		ADF4159_REG1_PHASE(conf->phase_val) |
		ADF4159_REG1_PHASE_ADJ_EN(!!conf->phase_val);

		tmp = DIV_ROUND_CLOSEST(conf->cp_curr_uA - 315, 315U);
		tmp = clamp(tmp, 0U, 15U);

	/* REG2 Bit Definitions */
	st->regs[ADF4159_REG2] =
		ADF4159_REG2_CLK1_DIV(conf->clk1_div) |
		ADF4159_REG2_R_CNT(conf->ref_div_factor) |
		ADF4159_REG2_REF_DOUBLER_EN(conf->ref_doubler_en) |
		ADF4159_REG2_REF_DIV2_EN(conf->ref_div2_en) |
		ADF4159_REG2_PRESCALER_89_EN(prescaler_89) |
		ADF4159_REG2_CP_CURRENT(tmp) |
		ADF4159_REG2_CSR_EN(conf->csr_en);

	/* REG3 Bit Definitions */
	st->regs[ADF4159_REG3] =
		ADF4159_REG3_CNT_RESET_EN(0) |
		ADF4159_REG3_CP_THREE_STATE_EN(0) |
		ADF4159_REG3_POWER_DOWN_EN(conf->pwdn_en) |
		ADF4159_REG3_PD_POLARITY_POS_EN(conf->pd_pol_pos_en) |
		ADF4159_REG3_LDP_6NS_EN(conf->ldp_6ns_en) |
		ADF4159_REG3_FSK_EN(conf->fsk_en) |
		ADF4159_REG3_PSK_EN(conf->psk_en) |
		ADF4159_REG3_RAMP_MODE(conf->ramp_mode) |
		ADF4159_REG3_SD_RESET_DIS(0) |
		ADF4159_REG3_LOL_DIS(conf->neg_bleed_en) |
		ADF4159_REG3_NEG_BLEED_EN(conf->neg_bleed_en) |
		ADF4159_REG3_NEG_BLEED_CURR(conf->neg_bleed_curr);

	/* REG4 Bit Definitions */
	st->regs[ADF4159_REG4] =
		ADF4159_REG4_CLK_DIV2_SEL(0) |
		ADF4159_REG4_CLK2_DIV(conf->clk2_div[0]) |
		ADF4159_REG4_CLK_DIV_MODE(conf->clk_div_mode) |
		ADF4159_REG4_RAMP_STATUS(conf->ramp_status) |
		ADF4159_REG4_SD_MOD_MODE(0) |
		ADF4159_REG4_LE_SEL_SYNC_REFIN_EN(conf->le_sel_sync_refin_en);

	st->regs[ADF4159_REG4_SEL1] =
		ADF4159_REG4_CLK_DIV2_SEL(1) |
		ADF4159_REG4_CLK2_DIV(conf->clk2_div[1]) |
		ADF4159_REG4_CLK_DIV_MODE(conf->clk_div_mode) |
		ADF4159_REG4_RAMP_STATUS(conf->ramp_status) |
		ADF4159_REG4_SD_MOD_MODE(0) |
		ADF4159_REG4_LE_SEL_SYNC_REFIN_EN(conf->le_sel_sync_refin_en);

	/* REG5 Bit Definitions */
	st->regs[ADF4159_REG5] =
		ADF4159_REG5_DEVIATION(conf->deviation[0]) |
		ADF4159_REG5_DEVIATION_OFFSET(conf->deviation_offs) |
		ADF4159_REG5_DEV_SEL(0) |
		ADF4159_REG5_DUAL_RAMP_EN(conf->dual_ramp_en) |
		ADF4159_REG5_FSK_RAMP_EN(conf->fsk_ramp_en) |
		ADF4159_REG5_INTERRUPT_MODE(conf->interrupt_mode) |
		ADF4159_REG5_PARABOLIC_RAMP_EN(conf->parabolic_ramp_en) |
		ADF4159_REG5_TXDATA_RAMP_CLK(conf->txdata_ramp_clk_txdata_en) |
		ADF4159_REG5_TXDATA_INVERT(conf->txdata_invert_en);

	st->regs[ADF4159_REG5_SEL1] =
		ADF4159_REG5_DEVIATION(conf->deviation[1]) |
		ADF4159_REG5_DEVIATION_OFFSET(conf->deviation_offs) |
		ADF4159_REG5_DEV_SEL(1) |
		ADF4159_REG5_DUAL_RAMP_EN(conf->dual_ramp_en) |
		ADF4159_REG5_FSK_RAMP_EN(conf->fsk_ramp_en) |
		ADF4159_REG5_INTERRUPT_MODE(conf->interrupt_mode) |
		ADF4159_REG5_PARABOLIC_RAMP_EN(conf->parabolic_ramp_en) |
		ADF4159_REG5_TXDATA_RAMP_CLK(conf->txdata_ramp_clk_txdata_en) |
		ADF4159_REG5_TXDATA_INVERT(conf->txdata_invert_en);

	/* REG6 Bit Definitions */
	st->regs[ADF4159_REG6] =
		ADF4159_REG6_STEP(conf->step_word[0]) |
		ADF4159_REG6_STEP_SEL(0);

	st->regs[ADF4159_REG6_SEL1] =
		ADF4159_REG6_STEP(conf->step_word[1]) |
		ADF4159_REG6_STEP_SEL(1);

	/* REG7 Bit Definitions */
	st->regs[ADF4159_REG7] =
		ADF4159_REG7_DELAY_START(conf->delay_start_word) |
		ADF4159_REG7_DELAY_START_EN(conf->delay_start_en) |
		ADF4159_REG7_DEL_CLK_SEL(conf->delay_clk_sel_pfd_x_clk1_en) |
		ADF4159_REG7_RAMP_DEL_EN(conf->ramp_delay_en) |
		ADF4159_REG7_RAMP_DEL_FL_EN(conf->ramp_delay_fl_en) |
		ADF4159_REG7_FAST_RAMP_EN(conf->fast_ramp_en) |
		ADF4159_REG7_TXDATA_TRIG_EN(conf->txdata_trig_en) |
		ADF4159_REG7_SING_FULL_TRI_EN(conf->single_full_tri_en) |
		ADF4159_REG7_TRI_DELAY_EN(conf->txdata_trig_delay_en);

	dev_dbg(&st->spi->dev, "VCO: %llu Hz, PFD %llu Hz\n"
		"INT %d, FRACT %d PRESCALER %s\n",
		freq, st->fpfd, st->integer, st->fract,
		prescaler_89 ? "8/9" : "4/5");

	return adf4159_sync_config(st);
}

static ssize_t adf4159_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct adf4159_state *st = iio_priv(indio_dev);
	struct adf4159_config *conf = st->conf;
	long long readin;
	int ret;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF4159_FREQ:
		conf->frequency = readin;
		break;
	case ADF4159_PWRDOWN:
		conf->pwdn_en = !!readin;
		break;
	case ADF4159_FREQ_DEV_STEP:
		conf->frequency_deviation_step = readin;
		break;
	case ADF4159_FREQ_DEV_RANGE:
		conf->frequency_deviation_range = readin;
		break;
	case ADF4159_FREQ_DEV_TIME:
		conf->frequency_deviation_time = readin;
		break;
	default:
		ret = -EINVAL;
	}

	ret = adf4159_setup(st, 0, conf->frequency);

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t adf4159_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct adf4159_state *st = iio_priv(indio_dev);
	struct adf4159_config *conf = st->conf;
	long long val;
	u64 uval;
	int ret = 0;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF4159_FREQ:
		val = adf4159_pll_fract_n_get_rate(st);
		break;
	case ADF4159_PWRDOWN:
		val = !!(st->regs[ADF4159_REG3] &
			ADF4159_REG3_POWER_DOWN_EN(1));
		break;
	case ADF4159_FREQ_DEV_STEP:
		val = st->fpfd * (s16)(conf->deviation[0] & 0xFFFF) *
			(u64)(1 << (conf->deviation_offs & 0xF));
		val = div_s64(val, ADF4159_MODULUS);
		break;
	case ADF4159_FREQ_DEV_RANGE:
		uval = st->fpfd * abs((s16)conf->deviation[0] & 0xFFFF) *
			(1 << (conf->deviation_offs & 0xF));
		do_div(uval, ADF4159_MODULUS);
		val = uval * (conf->step_word[0] & 0xFFFFF);
		break;
	case ADF4159_FREQ_DEV_TIME:
		uval = (conf->clk1_div & 0xFFF) *
			(conf->clk2_div[0] & 0xFFF) *
			(conf->step_word[0] & 0xFFFFF) *
			1000000ULL; /* us */
		do_div(uval, st->fpfd);
		val = uval;
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&st->lock);

	return ret < 0 ? ret : sprintf(buf, "%lld\n", val);
}

static int adf4159_set_ramp_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, u32 mode)
{
	struct adf4159_state *st = iio_priv(indio_dev);
	struct adf4159_config *conf = st->conf;

	if (mode) {
		conf->ramp_en = true;
		conf->ramp_mode = mode - 1;
		conf->clk_div_mode = CLK_DIV_MODE_RAMP;
	} else {
		conf->ramp_en = false;
		conf->clk_div_mode = CLK_DIV_MODE_OFF;
	}

	return adf4159_setup(st, 0, conf->frequency);
}

static int adf4159_get_ramp_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct adf4159_state *st = iio_priv(indio_dev);

	return st->conf->ramp_en ? st->conf->ramp_mode + 1 : 0;
}

static const char * const adf4159_ramp_modes[] = {
	"disabled",
	"continuous_sawtooth",
	"continuous_triangular",
	"single_sawtooth_burst",
	"single_ramp_burst"
};

static const struct iio_enum adf4159_ramp_modes_available = {
	.items = adf4159_ramp_modes,
	.num_items = ARRAY_SIZE(adf4159_ramp_modes),
	.get = adf4159_get_ramp_mode,
	.set = adf4159_set_ramp_mode,
};

#define _ADF4159_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = adf4159_read, \
	.write = adf4159_write, \
	.private = _ident, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info adf4159_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADF4159_EXT_INFO("frequency", ADF4159_FREQ, IIO_SEPARATE),
	_ADF4159_EXT_INFO("frequency_deviation_step",
			  ADF4159_FREQ_DEV_STEP, IIO_SEPARATE),
	_ADF4159_EXT_INFO("frequency_deviation_range",
			  ADF4159_FREQ_DEV_RANGE, IIO_SEPARATE),
	_ADF4159_EXT_INFO("frequency_deviation_time",
			  ADF4159_FREQ_DEV_TIME, IIO_SEPARATE),
	_ADF4159_EXT_INFO("powerdown", ADF4159_PWRDOWN, IIO_SEPARATE),
	IIO_ENUM_AVAILABLE("ramp_mode", &adf4159_ramp_modes_available),
	IIO_ENUM("ramp_mode", false, &adf4159_ramp_modes_available),
	{ },
};

static const struct iio_chan_spec adf4159_chan[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf4159_ext_info,
	}
};

static const struct iio_info adf4159_info = {
	.debugfs_reg_access = &adf4159_reg_access,
};

static void adf4159_property_u32(struct adf4159_state *st,
	const char *name, u32 def, u32 *val)
{
	int ret;

	ret = device_property_read_u32(&st->spi->dev, name, val);
	if (ret)
		*val = def;

	if (st->dent)
		debugfs_create_u32(name, 0644, st->dent, val);
}

static void adf4159_property_u64(struct adf4159_state *st,
	const char *name, u64 def, u64 *val)
{
	int ret;

	ret = device_property_read_u64(&st->spi->dev, name, val);
	if (ret)
		*val = def;

	if (st->dent)
		debugfs_create_u64(name, 0644, st->dent, val);
}

static void adf4159_property_bool(struct adf4159_state *st,
	const char *name, bool *val)
{
	*val = device_property_read_bool(&st->spi->dev, name);

	if (st->dent)
		debugfs_create_bool(name, 0644, st->dent, val);
}

static struct adf4159_config *adf4159_parse_dt(struct adf4159_state *st)
{
	struct adf4159_config *conf;

	conf = devm_kzalloc(&st->spi->dev, sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return NULL;

	adf4159_property_u64(st, "adi,power-up-frequency-hz", 0,
			     &conf->frequency);

	adf4159_property_u32(st, "adi,frequency-deviation-step-hz", 0,
			     &conf->frequency_deviation_step);
	adf4159_property_u32(st, "adi,frequency-deviation-range-hz", 0,
			     &conf->frequency_deviation_range);
	adf4159_property_u32(st, "adi,frequency-deviation-time-us", 0,
			     &conf->frequency_deviation_time);

	adf4159_property_u32(st, "adi,reference-div-factor", 1,
			     &conf->ref_div_factor);
	adf4159_property_u32(st, "adi,clk1-div", 1, &conf->clk1_div);
	adf4159_property_u32(st, "adi,clkin-hz", 0, &conf->clkin);
	adf4159_property_u32(st, "adi,muxout-select", MUXOUT_THREE_STATE_OUTPUT,
			     &conf->muxout);
	adf4159_property_bool(st, "adi,ramp-enable", &conf->ramp_en);

	adf4159_property_u32(st, "adi,phase", 0, &conf->phase_val);
	adf4159_property_u32(st, "adi,charge-pump-current-microamp", 900,
			     &conf->cp_curr_uA);
	adf4159_property_bool(st, "adi,reference-doubler-enable",
			      &conf->ref_doubler_en);
	adf4159_property_bool(st, "adi,reference-div2-enable",
			      &conf->ref_div2_en);
	adf4159_property_bool(st, "adi,cycle-slip-reduction-enable",
			      &conf->csr_en);

	adf4159_property_bool(st, "adi,powerdown-enable", &conf->pwdn_en);
	adf4159_property_bool(st, "adi,phase-detector-polarity-positive-enable",
			      &conf->pd_pol_pos_en);
	adf4159_property_bool(st, "adi,lock-detect-precision-6ns-enable",
			      &conf->ldp_6ns_en);
	adf4159_property_bool(st, "adi,fsk-modulation-enable", &conf->fsk_en);
	adf4159_property_bool(st, "adi,psk-modulation-enable", &conf->psk_en);
	adf4159_property_bool(st, "adi,charge-pump-negative-bleed-enable",
			      &conf->neg_bleed_en);

	adf4159_property_u32(st, "adi,ramp-mode-select", 0, &conf->ramp_mode);
	adf4159_property_u32(st, "adi,negative-bleed-current-microamp", 0,
			     &conf->neg_bleed_curr);

	adf4159_property_u32(st, "adi,clk2-timer-div", 0, &conf->clk2_div[0]);
	adf4159_property_u32(st, "adi,clk2-timer-div-2", 0, &conf->clk2_div[1]);
	adf4159_property_u32(st, "adi,clk-div-mode", 0, &conf->clk_div_mode);
	adf4159_property_u32(st, "adi,ramp-status-mode", 0, &conf->ramp_status);
	adf4159_property_bool(st, "adi,le-sync-refin-enable",
			      &conf->le_sel_sync_refin_en);

	adf4159_property_u32(st, "adi,deviation", 0, &conf->deviation[0]);
	adf4159_property_u32(st, "adi,deviation-2", 0, &conf->deviation[1]);
	adf4159_property_u32(st, "adi,deviation-offset", 0,
			     &conf->deviation_offs);
	adf4159_property_u32(st, "adi,interrupt-mode-select", 0,
			     &conf->interrupt_mode);
	adf4159_property_bool(st, "adi,dual-ramp-enable", &conf->dual_ramp_en);
	adf4159_property_bool(st, "adi,fsk-ramp-enable", &conf->fsk_ramp_en);
	adf4159_property_bool(st, "adi,parabolic-ramp-enable",
			      &conf->parabolic_ramp_en);
	adf4159_property_bool(st, "adi,txdata-ramp-clk-txdata-enable",
			      &conf->txdata_ramp_clk_txdata_en);
	adf4159_property_bool(st, "adi,txdata-invert-enable",
			      &conf->txdata_invert_en);

	adf4159_property_u32(st, "adi,step-word", 0, &conf->step_word[0]);
	adf4159_property_u32(st, "adi,step-word-2", 0, &conf->step_word[1]);

	adf4159_property_u32(st, "adi,delay-start-word", 0,
			     &conf->delay_start_word);
	adf4159_property_bool(st, "adi,delay-start-enable",
			      &conf->delay_start_en);
	adf4159_property_bool(st, "adi,delay-clk-sel-pfd-x-clk1-enable",
			      &conf->delay_clk_sel_pfd_x_clk1_en);
	adf4159_property_bool(st, "adi,ramp-delay-enable",
			      &conf->ramp_delay_en);
	adf4159_property_bool(st, "adi,ramp-dealy-fl-enable",
			      &conf->ramp_delay_fl_en);
	adf4159_property_bool(st, "adi,fast-ramp-enable", &conf->fast_ramp_en);
	adf4159_property_bool(st, "adi,txdata-trigger-enable",
			      &conf->txdata_trig_en);
	adf4159_property_bool(st, "adi,single-full-triangle-enable",
			      &conf->single_full_tri_en);
	adf4159_property_bool(st, "adi,txdata-trigger-delay-enable",
			      &conf->txdata_trig_delay_en);

	return conf;
}

static unsigned long adf4159_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long long rate;

	rate = adf4159_pll_fract_n_get_rate(to_clk_priv(hw)->st);

	return to_ccf_scaled(rate, &to_clk_priv(hw)->scale);
}

static long adf4159_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static int adf4159_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct adf4159_state *st = to_clk_priv(hw)->st;

	return adf4159_setup(st, parent_rate,
		from_ccf_scaled(rate, &to_clk_priv(hw)->scale));
}

static int adf4159_clk_enable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = true;

	return 0;
}

static void adf4159_clk_disable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = false;
}

static int adf4159_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = adf4159_clk_recalc_rate,
	.round_rate = adf4159_clk_round_rate,
	.set_rate = adf4159_clk_set_rate,
	.enable = adf4159_clk_enable,
	.disable = adf4159_clk_disable,
	.is_enabled = adf4159_clk_is_enabled,
};

static void adf4159_of_clk_del_provider(void *data)
{
	struct device *dev = data;

	of_clk_del_provider(dev->of_node);
}

static int adf4159_clk_register(struct adf4159_state *st)
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
		clk_priv->scale.div = 1;
	}

	init.name = clk_name;
	init.ops = &clkout_ops;
	init.flags = 0;
	parent_name = __clk_get_name(st->clk);
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
		adf4159_of_clk_del_provider, &spi->dev);
}

static void adf4159_action_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void adf4159_action_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static void adf4159_powerdown(void *data)
{
	struct adf4159_state *st = data;

	st->regs[ADF4159_REG3] |= ADF4159_REG3_POWER_DOWN_EN(1);
	adf4159_sync_config(st);
}

static int adf4159_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct adf4159_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->clk = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clk))
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(st->clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		adf4159_action_clk_disable, st->clk);
	if (ret)
		return ret;

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (IS_ERR(st->reg))
		return PTR_ERR(st->reg);

	ret = regulator_enable(st->reg);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		adf4159_action_regulator_disable, st->reg);
	if (ret)
		return ret;

	st->spi = spi;
	mutex_init(&st->lock);

	switch (id->driver_data) {
	case ADF4159:
		st->max_out_freq = ADF4159_MAX_FREQ;
		st->max_pfd_freq = ADF4159_MAX_FREQ_PFD;
		break;
	case ADF4169:
		st->max_out_freq = ADF4169_MAX_FREQ;
		st->max_pfd_freq = ADF4169_MAX_FREQ_PFD;
		break;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = id->name;
	indio_dev->info = &adf4159_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adf4159_chan;
	indio_dev->num_channels = 1;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	st->dent = iio_get_debugfs_dentry(indio_dev);

	st->conf = adf4159_parse_dt(st);
	if (!st->conf)
		return -ENOMEM;

	if (st->conf->frequency) {
		ret = adf4159_setup(st, 0, st->conf->frequency);
		if (ret)
			return ret;
	}

	ret = adf4159_clk_register(st);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&spi->dev, adf4159_powerdown, st);
}

static const struct spi_device_id adf4159_id[] = {
	{"adf4159", ADF4159},
	{"adf4169", ADF4169},
	{}
};

static struct spi_driver adf4159_driver = {
	.driver = {
		.name	= "adf4159",
	},
	.probe		= adf4159_probe,
	.id_table	= adf4159_id,
};
module_spi_driver(adf4159_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4159 PLL");
MODULE_LICENSE("GPL v2");
