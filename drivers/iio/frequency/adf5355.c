/*
 * ADF5355 SPI Wideband Synthesizer driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gcd.h>
#include <linux/gpio.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/frequency/adf5355.h>

#include <linux/clk/clkscale.h>

/* REG0 Bit Definitions */
#define ADF5355_REG0_INT(x)			(((x) & 0xFFFF) << 4)
#define ADF5355_REG0_PRESCALER(x)		((x) << 20)
#define ADF5355_REG0_AUTOCAL(x)			((x) << 21)

/* REG1 Bit Definitions */
#define ADF5355_REG1_FRACT(x)			(((x) & 0xFFFFFF) << 4)

/* REG2 Bit Definitions */
#define ADF5355_REG2_MOD2(x)			(((x) & 0x3FFF) << 4)
#define ADF5355_REG2_FRAC2(x)			(((x) & 0x3FFF) << 18)

/* REG3 Bit Definitions */
#define ADF5355_REG3_PHASE(x)			(((x) & 0xFFFFFF) << 4)
#define ADF5355_REG3_PHASE_ADJUST(x)		((x) << 28)
#define ADF5355_REG3_PHASE_RESYNC(x)		((x) << 29)
#define ADF5355_REG3_EXACT_SDLOAD_RESET(x)	((x) << 30)

/* REG4 Bit Definitions */
#define ADF5355_REG4_COUNTER_RESET_EN(x)	((x) << 4)
#define ADF5355_REG4_CP_THREESTATE_EN(x)	((x) << 5)
#define ADF5355_REG4_POWER_DOWN_EN(x)		((x) << 6)
#define ADF5355_REG4_PD_POLARITY_POS(x)		((x) << 7)
#define ADF5355_REG4_MUX_LOGIC(x)		((x) << 8)
#define ADF5355_REG4_REFIN_MODE_DIFF(x)		((x) << 9)
#define ADF5355_REG4_CHARGE_PUMP_CURR(x)		(((x) & 0xF) << 10)
#define ADF5355_REG4_DOUBLE_BUFF_EN(x)		((x) << 14)
#define ADF5355_REG4_10BIT_R_CNT(x)		(((x) & 0x3FF) << 15)
#define ADF5355_REG4_RDIV2_EN(x)		((x) << 25)
#define ADF5355_REG4_RMULT2_EN(x)		((x) << 26)
#define ADF5355_REG4_MUXOUT(x)			(((x) & 0x7) << 27)
#define ADF5355_MUXOUT_THREESTATE		0
#define ADF5355_MUXOUT_DVDD			1
#define ADF5355_MUXOUT_GND			2
#define ADF5355_MUXOUT_R_DIV_OUT		3
#define ADF5355_MUXOUT_N_DIV_OUT		4
#define ADF5355_MUXOUT_ANALOG_LOCK_DETECT	5
#define ADF5355_MUXOUT_DIGITAL_LOCK_DETECT	6

/* REG5 Bit Definitions */
#define ADF5355_REG5_DEFAULT			0x00800025

/* REG6 Bit Definitions */
#define ADF4355_REG6_OUTPUTB_PWR(x)		(((x) & 0x7) << 4)
#define ADF4355_REG6_RF_OUTB_EN(x)		((x) << 9)
#define ADF5355_REG6_OUTPUT_PWR(x)		(((x) & 0x3) << 4)
#define ADF5355_REG6_RF_OUT_EN(x)		((x) << 6)
#define ADF5355_REG6_RF_OUTB_EN(x)		((x) << 10)
#define ADF5355_REG6_MUTE_TILL_LOCK_EN(x)	((x) << 11)
#define ADF5355_REG6_CP_BLEED_CURR(x)		(((x) & 0xFF) << 13)
#define ADF5355_REG6_RF_DIV_SEL(x)		(((x) & 0x7) << 21)
#define ADF5355_REG6_FEEDBACK_FUND(x)		((x) << 24)
#define ADF5355_REG6_NEG_BLEED_EN(x)		((x) << 29)
#define ADF5355_REG6_GATED_BLEED_EN(x)		((x) << 30)
#define ADF5355_REG6_DEFAULT			0x14000006


/* REG7 Bit Definitions */
#define ADF5355_REG7_LD_MODE_INT_N_EN(x)		((x) << 4)
#define ADF5355_REG7_FACT_N_LD_PRECISION(x)	(((x) & 0x3) << 5)
#define ADF5355_REG7_LOL_MODE_EN(x)		((x) << 7)
#define ADF5355_REG7_LD_CYCLE_CNT(x)		(((x) & 0x3) << 8)
#define ADF5355_REG7_LE_SYNCED_REFIN_EN(x)	((x) << 25)
#define ADF5355_REG7_DEFAULT			0x10000007

/* REG8 Bit Definitions */
#define ADF5355_REG8_DEFAULT			0x102D0428

/* REG9 Bit Definitions */
#define ADF5355_REG9_SYNTH_LOCK_TIMEOUT(x)	(((x) & 0x1F) << 4)
#define ADF5355_REG9_ALC_TIMEOUT(x)		(((x) & 0x1F) << 9)
#define ADF5355_REG9_TIMEOUT(x)			(((x) & 0x3FF) << 14)
#define ADF5355_REG9_VCO_BAND_DIV(x)		(((x) & 0xFF) << 24)

/* REG10 Bit Definitions */
#define ADF5355_REG10_ADC_EN(x)			((x) << 4)
#define ADF5355_REG10_ADC_CONV_EN(x)		((x) << 5)
#define ADF5355_REG10_ADC_CLK_DIV(x)		(((x) & 0xFF) << 6)
#define ADF5355_REG10_DEFAULT			0x00C0000A

/* REG11 Bit Definitions */
#define ADF5355_REG11_DEFAULT			0x0061300B

/* REG12 Bit Definitions */
#define ADF5355_REG12_PHASE_RESYNC_CLK_DIV(x)	(((x) & 0xFFFF) << 16)
#define ADF5355_REG12_DEFAULT			0x0000041C


/* Specifications */
#define ADF5355_MIN_VCO_FREQ		3400000000ULL /* Hz */
#define ADF5355_MAX_VCO_FREQ		6800000000ULL /* Hz */
#define ADF5355_MAX_OUT_FREQ		ADF5355_MAX_VCO_FREQ /* Hz */
#define ADF5355_MIN_OUT_FREQ		(ADF5355_MIN_VCO_FREQ / 64) /* Hz */
#define ADF5355_MAX_OUTB_FREQ		(ADF5355_MAX_VCO_FREQ * 2) /* Hz */
#define ADF5355_MIN_OUTB_FREQ		(ADF5355_MIN_VCO_FREQ * 2) /* Hz */


#define ADF4355_MIN_VCO_FREQ		3400000000ULL /* Hz */
#define ADF4355_MAX_VCO_FREQ		6800000000ULL /* Hz */
#define ADF4355_MAX_OUT_FREQ		ADF4355_MAX_VCO_FREQ /* Hz */
#define ADF4355_MIN_OUT_FREQ		(ADF4355_MIN_VCO_FREQ / 64) /* Hz */


#define ADF4355_3_MIN_VCO_FREQ		3300000000ULL /* Hz */
#define ADF4355_3_MAX_VCO_FREQ		6600000000ULL /* Hz */
#define ADF4355_3_MAX_OUT_FREQ		ADF4355_3_MAX_VCO_FREQ /* Hz */
#define ADF4355_3_MIN_OUT_FREQ		(ADF4355_3_MIN_VCO_FREQ / 64) /* Hz */


#define ADF4355_2_MIN_VCO_FREQ		3400000000ULL /* Hz */
#define ADF4355_2_MAX_VCO_FREQ		6800000000ULL /* Hz */
#define ADF4355_2_MAX_OUT_FREQ		4400000000ULL /* Hz */
#define ADF4355_2_MIN_OUT_FREQ		(ADF4355_2_MIN_VCO_FREQ / 64) /* Hz */


#define ADF5355_MAX_FREQ_PFD		125000000UL /* Hz */
#define ADF5355_MAX_FREQ_REFIN		600000000UL /* Hz */
#define ADF5355_MAX_MODULUS2		16384
#define ADF5355_MAX_R_CNT		1023

#define ADF5355_MODULUS1			16777216ULL
#define ADF5355_MIN_INT_PRESCALER_89	75

/* Registers */
enum adf5433_reg {
	ADF5355_REG0,
	ADF5355_REG1,
	ADF5355_REG2,
	ADF5355_REG3,
	ADF5355_REG4,
	ADF5355_REG5,
	ADF5355_REG6,
	ADF5355_REG7,
	ADF5355_REG8,
	ADF5355_REG9,
	ADF5355_REG10,
	ADF5355_REG11,
	ADF5355_REG12,
	ADF5355_REG_NUM,
};

enum {
	ADF5355_FREQ,
	ADF5355_FREQ_REFIN,
	ADF5355_PWRDOWN,
};

enum {
	ADF5355,
	ADF4355,
	ADF4355_2,
	ADF4355_3,
};

struct adf5355_state {
	struct spi_device	*spi;
	struct regulator		*reg;
	struct adf5355_platform_data	*pdata;
	struct clk		*clk;
	unsigned long long	freq_req;
	unsigned long		clkin;
	unsigned long		fpfd; /* Phase Frequency Detector */
	unsigned long long	min_vco_freq;
	unsigned long long	min_out_freq;
	unsigned long long	max_out_freq;
	u32			freq_req_chan;
	u32			integer;
	u32			fract1;
	u32			fract2;
	u32			mod2;
	u32			rf_div_sel;
	u32			delay_us;
	u32			regs[ADF5355_REG_NUM];
	u32			clock_shift;
	bool			all_synced;
	bool			is_5355;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be32			val ____cacheline_aligned;
};

struct child_clk {
	struct clk_hw		hw;
	struct adf5355_state	*st;
	bool			enabled;
	struct clock_scale 	scale;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

static int adf5355_spi_write(struct adf5355_state *st, u32 val)
{
	st->val = cpu_to_be32(val);

	dev_dbg(&st->spi->dev, "[%d] 0x%.8X\n", val & 0xF, val);

	return spi_write(st->spi, &st->val, 4);
}

static int adf5355_sync_config(struct adf5355_state *st, bool sync_all)
{
	int ret, i;

	if (sync_all || !st->all_synced) {
		for (i = ADF5355_REG12; i >= ADF5355_REG0; i--) {
				ret = adf5355_spi_write(st, st->regs[i] | i);
				if (ret < 0)
					return ret;
		}
		st->all_synced = true;
	} else {
			ret = adf5355_spi_write(st, st->regs[6] | 6);
			if (ret < 0)
				return ret;
			ret = adf5355_spi_write(st, st->regs[4] |
				ADF5355_REG4_COUNTER_RESET_EN(1) | 4);
			if (ret < 0)
				return ret;
			ret = adf5355_spi_write(st, st->regs[2] | 2);
			if (ret < 0)
				return ret;
			ret = adf5355_spi_write(st, st->regs[1] | 1);
			if (ret < 0)
				return ret;
			ret = adf5355_spi_write(st, st->regs[0] &
				~ADF5355_REG0_AUTOCAL(1));
			if (ret < 0)
				return ret;
			ret = adf5355_spi_write(st, st->regs[4] | 4);
			if (ret < 0)
				return ret;

			udelay(st->delay_us);

			ret = adf5355_spi_write(st, st->regs[0]);
			if (ret < 0)
				return ret;

	}

	return 0;
}

static int adf5355_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct adf5355_state *st = iio_priv(indio_dev);
	int ret;

	if (reg > ADF5355_REG12)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		st->regs[reg] = writeval & ~(BIT(0) | BIT(1) | BIT(2) | BIT(3));
		ret = adf5355_sync_config(st, true);
	} else {
		*readval =  st->regs[reg];
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int adf5355_pll_fract_n_compute(unsigned long long vco,
				       unsigned long long pfd,
				       unsigned *integer, unsigned *fract1,
				       unsigned *fract2, unsigned *mod2)
{
	unsigned long long tmp;
	u32 gcd_div;

	tmp = do_div(vco, pfd);
	tmp = tmp * ADF5355_MODULUS1;
	*fract2 = do_div(tmp, pfd);

	*integer = vco;
	*fract1 = tmp;

	*mod2 = pfd;

	while (*mod2 > ADF5355_MAX_MODULUS2) {
		*mod2 >>= 1;
		*fract2 >>=1;
	}

	gcd_div = gcd(*fract2, *mod2);
	*mod2 /= gcd_div;
	*fract2 /= gcd_div;

	return 0;
}

static unsigned long long adf5355_pll_fract_n_get_rate(struct adf5355_state *st,
						       u32 channel)
{
	unsigned long long val, tmp;

	val = (((u64)st->integer * ADF5355_MODULUS1) + st->fract1) * st->fpfd;
	tmp = (u64)st->fract2 * st->fpfd;
	do_div(tmp, st->mod2);
	val += tmp + ADF5355_MODULUS1 / 2;
	do_div(val, ADF5355_MODULUS1 *
		(1 << (channel == 1 ? 0 : st->rf_div_sel)));
	if (channel == 1)
		val <<= 1;

	return val;
}

static int adf5355_setup(struct adf5355_state *st, unsigned long parent_rate)
{
	struct adf5355_platform_data *pdata = st->pdata;
	u32 ref_div_factor, tmp;

	if (parent_rate) {
		st->clkin = parent_rate;
	} else if (st->clk) {
		st->clkin = clk_get_rate(st->clk);
	} else {
		st->clkin = pdata->clkin;
	}

	ref_div_factor = pdata->ref_div_factor;

	/* Calculate and maximize PFD frequency */
	do {
		ref_div_factor++;
		st->fpfd = (st->clkin * (pdata->ref_doubler_en ? 2 : 1)) /
			   (ref_div_factor * (pdata->ref_div2_en ? 2 : 1));
	} while (st->fpfd > ADF5355_MAX_FREQ_PFD);


	tmp = DIV_ROUND_CLOSEST(pdata->cp_curr_uA - 315, 315U);
	tmp = clamp(tmp, 0U, 15U);

	st->regs[ADF5355_REG4] =
		ADF5355_REG4_COUNTER_RESET_EN(0) |
		ADF5355_REG4_CP_THREESTATE_EN(0) |
		ADF5355_REG4_POWER_DOWN_EN(0) |
		ADF5355_REG4_PD_POLARITY_POS(!pdata->phase_detector_polarity_neg) |
		ADF5355_REG4_MUX_LOGIC(pdata->mux_out_3V3_en) |
		ADF5355_REG4_REFIN_MODE_DIFF(pdata->ref_diff_en) |
		ADF5355_REG4_CHARGE_PUMP_CURR(tmp) |
		ADF5355_REG4_DOUBLE_BUFF_EN(1) |
		ADF5355_REG4_10BIT_R_CNT(ref_div_factor) |
		ADF5355_REG4_RDIV2_EN(pdata->ref_div2_en) |
		ADF5355_REG4_RMULT2_EN(pdata->ref_doubler_en) |
		ADF5355_REG4_MUXOUT(pdata->mux_out_sel);

	st->regs[ADF5355_REG5] = ADF5355_REG5_DEFAULT;

	st->regs[ADF5355_REG7] = ADF5355_REG7_LD_MODE_INT_N_EN(0) |
		ADF5355_REG7_FACT_N_LD_PRECISION(3) |
		ADF5355_REG7_LOL_MODE_EN(0) |
		ADF5355_REG7_LD_CYCLE_CNT(0) |
		ADF5355_REG7_LE_SYNCED_REFIN_EN(1) |
		ADF5355_REG7_DEFAULT;

	st->regs[ADF5355_REG8] = ADF5355_REG8_DEFAULT;

	/* Calculate Timeouts */
	tmp = DIV_ROUND_UP(st->fpfd, 20000U * 30U);
	tmp = clamp(tmp, 1U, 1023U);

	st->regs[ADF5355_REG9] = ADF5355_REG9_TIMEOUT(tmp) |
		ADF5355_REG9_SYNTH_LOCK_TIMEOUT(DIV_ROUND_UP(st->fpfd * 2U, 100000U * tmp)) |
		ADF5355_REG9_ALC_TIMEOUT(DIV_ROUND_UP(st->fpfd * 5U, 100000U * tmp)) |
		ADF5355_REG9_VCO_BAND_DIV(DIV_ROUND_UP(st->fpfd, 2400000U));

	tmp = DIV_ROUND_UP(st->fpfd / 100000U - 2, 4);
	tmp = clamp(tmp, 1U, 255U);

	/* Delay > 16 ADC_CLK cycles */
	st->delay_us = DIV_ROUND_UP(16000000UL, st->fpfd / (4 * tmp + 2));

	st->regs[ADF5355_REG10] = ADF5355_REG10_ADC_EN(1) |
		ADF5355_REG10_ADC_CONV_EN(1) |
		ADF5355_REG10_ADC_CLK_DIV(tmp) |
		ADF5355_REG10_DEFAULT;

	st->regs[ADF5355_REG11] = ADF5355_REG11_DEFAULT;

	st->regs[ADF5355_REG12] = ADF5355_REG12_PHASE_RESYNC_CLK_DIV(0) |
		ADF5355_REG12_DEFAULT;

	st->all_synced = false;

	return 0;
}

static int adf5355_set_freq(struct adf5355_state *st, unsigned long long freq,
			    u32 channel)
{
	struct adf5355_platform_data *pdata = st->pdata;
	bool prescaler;
	u32 cp_bleed;

	if (channel == 0) {
		if ((freq > st->max_out_freq) || (freq < st->min_out_freq))
			return -EINVAL;

		st->rf_div_sel = 0;

		while (freq < st->min_vco_freq) {
			freq <<= 1;
			st->rf_div_sel++;
		}
	} else {
		/* ADF5355 RFoutB 6800...13600 MHz */
		if ((freq > ADF5355_MAX_OUTB_FREQ) || (freq < ADF5355_MIN_OUTB_FREQ))
			return -EINVAL;

		freq >>= 1;
	}

	adf5355_pll_fract_n_compute(freq, st->fpfd, &st->integer, &st->fract1,
			&st->fract2, &st->mod2);

	prescaler = (st->integer >= ADF5355_MIN_INT_PRESCALER_89);

	/* Tests have shown that the optimal bleed set is the following:
	 * 4/N < IBLEED/ICP < 10/N
	 */
	cp_bleed = DIV_ROUND_UP(400 * pdata->cp_curr_uA , st->integer * 375);
	cp_bleed = clamp(cp_bleed, 1U, 255U);

	st->regs[ADF5355_REG0] = ADF5355_REG0_INT(st->integer) |
				 ADF5355_REG0_PRESCALER(prescaler) |
				 ADF5355_REG0_AUTOCAL(1);

	st->regs[ADF5355_REG1] = ADF5355_REG1_FRACT(st->fract1);
	st->regs[ADF5355_REG2] = ADF5355_REG2_MOD2(st->mod2) |
				ADF5355_REG2_FRAC2(st->fract2);

	st->regs[ADF5355_REG6] =
		ADF5355_REG6_OUTPUT_PWR(pdata->outa_power) |
		ADF5355_REG6_RF_OUT_EN(pdata->outa_en) |
		(st->is_5355 ? ADF5355_REG6_RF_OUTB_EN(pdata->outb_en) :
			ADF4355_REG6_OUTPUTB_PWR(pdata->outa_power) |
			ADF4355_REG6_RF_OUTB_EN(pdata->outb_en)) |
		ADF5355_REG6_MUTE_TILL_LOCK_EN(pdata->mute_till_lock_detect_en) |
		ADF5355_REG6_CP_BLEED_CURR(cp_bleed) |
		ADF5355_REG6_RF_DIV_SEL(st->rf_div_sel) |
		ADF5355_REG6_FEEDBACK_FUND(1) |
		ADF5355_REG6_NEG_BLEED_EN(pdata->cp_neg_bleed_en) |
		ADF5355_REG6_GATED_BLEED_EN(pdata->cp_gated_bleed_en) |
		ADF5355_REG6_DEFAULT;

	st->freq_req = freq;
	st->freq_req_chan = channel;

	dev_dbg(&st->spi->dev, "VCO: %llu Hz, PFD %lu Hz\n"
		"INT %d, FRACT1 %d, FRACT2 %d\n"
		"MOD2 %d, RF_DIV %d\nPRESCALER %s\n",
		freq, st->fpfd, st->integer, st->fract1,st->fract2, st->mod2,
		1 << st->rf_div_sel, prescaler ? "8/9" : "4/5");

	return adf5355_sync_config(st, false);
}

static ssize_t adf5355_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct adf5355_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned long tmp;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case ADF5355_FREQ:
		ret = adf5355_set_freq(st, readin, chan->channel);
		break;
	case ADF5355_FREQ_REFIN:
		if (readin > ADF5355_MAX_FREQ_REFIN) {
			ret = -EINVAL;
			break;
		}

		if (st->clk) {
			tmp = clk_round_rate(st->clk, readin);
			if (tmp != readin) {
				ret = -EINVAL;
				break;
			}
			ret = clk_set_rate(st->clk, tmp);
			if (ret < 0)
				break;
		}

		adf5355_setup(st, tmp);
		ret = adf5355_set_freq(st, st->freq_req, st->freq_req_chan);
		break;
	case ADF5355_PWRDOWN:
		if (chan->channel == 0) {
			st->regs[ADF5355_REG6] &= ~ADF5355_REG6_RF_OUT_EN(1);
			st->regs[ADF5355_REG6] |= ADF5355_REG6_RF_OUT_EN(!!!readin);
		} else {
			st->regs[ADF5355_REG6] &= ~(st->is_5355 ?
				ADF5355_REG6_RF_OUTB_EN(1) :
				ADF4355_REG6_RF_OUTB_EN(1));
			st->regs[ADF5355_REG6] |= (st->is_5355 ?
				ADF5355_REG6_RF_OUTB_EN(!!!readin) :
				ADF4355_REG6_RF_OUTB_EN(!!!readin));
		}
		adf5355_sync_config(st, false);

break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t adf5355_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct adf5355_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case ADF5355_FREQ:
		val = adf5355_pll_fract_n_get_rate(st, chan->channel);

		/* PLL unlocked? return error */
		if (gpio_is_valid(st->pdata->gpio_lock_detect))
			if (!gpio_get_value(st->pdata->gpio_lock_detect)) {
				dev_dbg(&st->spi->dev, "PLL un-locked\n");
				ret = -EBUSY;
			}
		break;
	case ADF5355_FREQ_REFIN:
		if (st->clk)
			st->clkin = clk_get_rate(st->clk);

		val = st->clkin;
		break;
	case ADF5355_PWRDOWN:

		if (chan->channel == 0) {
			val = !(st->regs[ADF5355_REG6] & ADF5355_REG6_RF_OUT_EN(1));
		} else {
			val = ! (st->regs[ADF5355_REG6] & (st->is_5355 ?
				ADF5355_REG6_RF_OUTB_EN(1) :
				ADF4355_REG6_RF_OUTB_EN(1)));
		}
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

#define _ADF5355_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = adf5355_read, \
	.write = adf5355_write, \
	.private = _ident, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info adf5355_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADF5355_EXT_INFO("frequency", ADF5355_FREQ, IIO_SEPARATE),
	_ADF5355_EXT_INFO("refin_frequency", ADF5355_FREQ_REFIN, IIO_SHARED_BY_TYPE),
	_ADF5355_EXT_INFO("powerdown", ADF5355_PWRDOWN, IIO_SEPARATE),
	{ },
};

static const struct iio_chan_spec adf5355_chan[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf5355_ext_info,
	}, {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.ext_info = adf5355_ext_info,
	}
};

static const struct iio_chan_spec_ext_info adf4355_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADF5355_EXT_INFO("frequency", ADF5355_FREQ, IIO_SHARED_BY_TYPE),
	_ADF5355_EXT_INFO("refin_frequency", ADF5355_FREQ_REFIN, IIO_SHARED_BY_TYPE),
	_ADF5355_EXT_INFO("powerdown", ADF5355_PWRDOWN, IIO_SEPARATE),
	{ },
};

static const struct iio_chan_spec adf4355_chan[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf4355_ext_info,
	}, {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.ext_info = adf4355_ext_info,
	}
};

static const struct iio_info adf5355_info = {
	.debugfs_reg_access = &adf5355_reg_access,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static struct adf5355_platform_data *adf5355_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct adf5355_platform_data *pdata;
	unsigned int tmp;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	strncpy(&pdata->name[0], np->name, SPI_NAME_SIZE - 1);

	of_property_read_u64(np, "adi,power-up-frequency", &pdata->power_up_frequency);

	tmp = 0;
	of_property_read_u32(np, "adi,reference-div-factor", &tmp);
	if (tmp > 0)
		tmp--;
	pdata->ref_div_factor = tmp;

	ret = of_get_gpio(np, 0);
	if (ret < 0)
		pdata->gpio_lock_detect = -1;
	else
		pdata->gpio_lock_detect = ret;

	pdata->ref_doubler_en = of_property_read_bool(np,
			"adi,reference-doubler-enable");
	pdata->ref_div2_en = of_property_read_bool(np,
			"adi,reference-div2-enable");

	pdata->ref_diff_en = of_property_read_bool(np,
			"adi,reference-differential-input-enable");

	pdata->phase_detector_polarity_neg = of_property_read_bool(np,
			"adi,phase-detector-polarity-negative-enable");

	pdata->cp_curr_uA = 900;
	of_property_read_u32(np, "adi,charge-pump-current", &pdata->cp_curr_uA);


	pdata->mux_out_sel = 0;
	of_property_read_u32(np, "adi,muxout-select", &pdata->mux_out_sel);


	pdata->mux_out_3V3_en = of_property_read_bool(np,
			"adi,muxout-level-3v3-enable");

	pdata->mute_till_lock_detect_en = of_property_read_bool(np,
			"adi,mute-till-lock-enable");

	pdata->outa_power = 2;
	of_property_read_u32(np, "adi,output-a-power", &pdata->outa_power);

	pdata->outb_power = 2;
	of_property_read_u32(np, "adi,output-b-power", &pdata->outb_power);

	pdata->outb_en = of_property_read_bool(np, "adi,output-b-enable");
	pdata->outa_en = of_property_read_bool(np, "adi,output-a-enable");

	pdata->cp_neg_bleed_en = of_property_read_bool(np, "adi,charge-pump-negative-bleed-enable");
	pdata->cp_gated_bleed_en = of_property_read_bool(np, "adi,charge-pump-gated-bleed-enable");

	pdata->clock_shift = 1;
	of_property_read_u32(np, "adi,clock-shift", &pdata->clock_shift);

	return pdata;
}
#else
static
struct adf5355_platform_data *adf5355_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static unsigned long adf5355_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long long rate;

	rate = adf5355_pll_fract_n_get_rate(to_clk_priv(hw)->st, 0);

	return to_ccf_scaled(rate, &to_clk_priv(hw)->scale);
}

static long adf5355_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static int adf5355_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct adf5355_state *st = to_clk_priv(hw)->st;

	if (parent_rate != st->clkin)
		adf5355_setup(st, parent_rate);

	return adf5355_set_freq(st, from_ccf_scaled(rate, &to_clk_priv(hw)->scale), 0);
}

static int adf5355_clk_enable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = true;

	return 0;
}

static void adf5355_clk_disable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = false;
}

static int adf5355_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = adf5355_clk_recalc_rate,
	.round_rate = adf5355_clk_round_rate,
	.set_rate = adf5355_clk_set_rate,
	.enable = adf5355_clk_enable,
	.disable = adf5355_clk_disable,
	.is_enabled = adf5355_clk_is_enabled,
};

static int adf5355_probe(struct spi_device *spi)
{
	struct adf5355_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct adf5355_state *st;
	struct clk *clk = NULL;
	int ret;

	if (spi->dev.of_node)
		pdata = adf5355_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_warn(&spi->dev, "Error: no platform data\n");
		return -EINVAL;
	}

	if (!pdata->clkin) {
		clk = devm_clk_get(&spi->dev, "clkin");
		if (IS_ERR(clk))
			return -EPROBE_DEFER;

		ret = clk_prepare_enable(clk);
		if (ret < 0)
			return ret;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret =  -ENOMEM;
		goto error_disable_clk;
	}

	st = iio_priv(indio_dev);

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_disable_clk;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;
	st->clk = clk;

	st->is_5355 = (spi_get_device_id(spi)->driver_data == 5355);

	switch (spi_get_device_id(spi)->driver_data) {
	case ADF5355:
		st->is_5355 = true;
		st->max_out_freq = ADF5355_MAX_OUT_FREQ;
		st->min_out_freq = ADF5355_MIN_OUT_FREQ;
		st->min_vco_freq = ADF5355_MIN_VCO_FREQ;
		break;
	case ADF4355:
		st->is_5355 = false;
		st->max_out_freq = ADF4355_MAX_OUT_FREQ;
		st->min_out_freq = ADF4355_MIN_OUT_FREQ;
		st->min_vco_freq = ADF4355_MIN_VCO_FREQ;
		break;
	case ADF4355_2:
		st->is_5355 = false;
		st->max_out_freq = ADF4355_2_MAX_OUT_FREQ;
		st->min_out_freq = ADF4355_2_MIN_OUT_FREQ;
		st->min_vco_freq = ADF4355_2_MIN_VCO_FREQ;
		break;
	case ADF4355_3:
		st->is_5355 = false;
		st->max_out_freq = ADF4355_3_MAX_OUT_FREQ;
		st->min_out_freq = ADF4355_3_MIN_OUT_FREQ;
		st->min_vco_freq = ADF4355_3_MIN_VCO_FREQ;
		break;
	}

	st->max_out_freq = st->is_5355 ? ADF5355_MAX_OUT_FREQ : ADF4355_MAX_OUT_FREQ;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (pdata->name[0] != 0) ? pdata->name :
		spi_get_device_id(spi)->name;

	indio_dev->info = &adf5355_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->is_5355 ? adf5355_chan : adf4355_chan;
	indio_dev->num_channels = 2;

	ret = adf5355_setup(st, 0);
	if (ret)
		goto error_disable_reg;

	if (gpio_is_valid(pdata->gpio_lock_detect)) {
		ret = devm_gpio_request(&spi->dev, pdata->gpio_lock_detect,
					indio_dev->name);
		if (ret) {
			dev_err(&spi->dev, "fail to request lock detect GPIO-%d",
				pdata->gpio_lock_detect);
			goto error_disable_reg;
		}
		gpio_direction_input(pdata->gpio_lock_detect);
	}

	if (pdata->power_up_frequency) {
		ret = adf5355_set_freq(st, pdata->power_up_frequency, 0);
		if (ret)
			goto error_disable_reg;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	if (IS_ENABLED(CONFIG_OF)) {
		struct child_clk *clk_priv;
		struct clk_init_data init;
		const char *parent_name;
		const char *clk_name;
		struct clk *clk_out;

		clk_priv = devm_kzalloc(&spi->dev, sizeof(*clk_priv), GFP_KERNEL);
		if (!clk_priv) {
			dev_err(&spi->dev, "%s: could not allocate fixed factor clk\n", __func__);
			ret = -ENOMEM;
			goto error_disable_reg;
		}

		/* struct child_clk assignments */
		clk_priv->hw.init = &init;
		clk_priv->st = st;

		clk_name = spi->dev.of_node->name;
		of_property_read_string(spi->dev.of_node, "clock-output-names",
			&clk_name);

		ret = of_clk_get_scale(spi->dev.of_node, NULL, &clk_priv->scale);

		if (ret < 0) {
			clk_priv->scale.mult = 1;
			clk_priv->scale.div = (1 << pdata->clock_shift);
		}

		init.name = clk_name;
		init.ops = &clkout_ops;
		init.flags = 0;

		parent_name = __clk_get_name(clk);
		init.parent_names = &parent_name;
		init.num_parents = 1;

		clk_out = clk_register(&spi->dev, &clk_priv->hw);
		if (IS_ERR(clk_out))
			kfree(clk_priv);

		if (!IS_ERR(clk_out))
			of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk_out);
	}

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
error_disable_clk:
	if (clk)
		clk_disable_unprepare(clk);

	return ret;
}

static int adf5355_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct adf5355_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	st->regs[ADF5355_REG4] |= ADF5355_REG4_POWER_DOWN_EN(1);
	adf5355_sync_config(st, true);

	iio_device_unregister(indio_dev);

	if (st->clk)
		clk_disable_unprepare(st->clk);

	if (IS_ENABLED(CONFIG_OF))
		of_clk_del_provider(spi->dev.of_node);

	if (!IS_ERR(reg)) {
		regulator_disable(reg);
	}

	return 0;
}


static const struct spi_device_id adf5355_id[] = {
	{"adf5355", ADF5355},
	{"adf4355", ADF4355},
	{"adf4355-2", ADF4355_2},
	{"adf4355-3", ADF4355_3},
	{}
};

static struct spi_driver adf5355_driver = {
	.driver = {
		.name	= "adf5355",
		.owner	= THIS_MODULE,
	},
	.probe		= adf5355_probe,
	.remove		= adf5355_remove,
	.id_table	= adf5355_id,
};
module_spi_driver(adf5355_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF5355 PLL");
MODULE_LICENSE("GPL v2");
