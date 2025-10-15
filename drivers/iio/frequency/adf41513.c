// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADF41513 SPI PLL Frequency Synthesizer driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/frequency/adf41513.h>

enum {
	ADF41513_FREQ,
	ADF41513_POWER_DOWN,
	ADF41513_FREQ_RESOLUTION,
	ADF41513_FREQ_REFIN
};

enum {
	ADF41510_ID,
	ADF41513_ID,
};

enum adf41513_pll_mode {
	ADF41513_MODE_INTEGER_N,
	ADF41513_MODE_FIXED_MODULUS,
	ADF41513_MODE_VARIABLE_MODULUS,
	ADF41513_MODE_INVALID
};

struct adf41513_pll_settings {
	enum adf41513_pll_mode		mode;

	u64				target_frequency_uhz;
	u64				actual_frequency_uhz;
	u64				pfd_frequency_uhz;

	/* pll parameters */
	u16				int_value;
	u32				frac1;
	u32				frac2;
	u32				mod2;

	/* reference path parameters */
	u8				r_counter;
	u8				ref_doubler;
	u8				ref_div2;
	u8				prescaler;
};

struct adf41513_state {
	struct spi_device		*spi;
	struct gpio_desc		*lock_detect;
	struct gpio_desc		*chip_enable;
	struct clk			*ref_clk;
	struct adf41513_platform_data	*pdata;

	struct clk_hw			clk_hw;
	struct clk			*clk_out;

	u32				device_id;
	u64				ref_freq_hz;
	u64				freq_resolution_uhz;

	/*
	 * Lock for accessing device registers. Some operations require
	 * multiple consecutive R/W operations, during which the device
	 * shouldn't be interrupted. The buffers are also shared across
	 * all operations so need to be protected on stand alone reads and
	 * writes.
	 */
	struct mutex			lock;

	/* Cached register values */
	u32				regs[14];
	u32				regs_hw[14];

	/* PLL configuration */
	struct adf41513_pll_settings	settings;

	/*
	 * DMA (thus cache coherency maintenance) may require that
	 * transfer buffers live in their own cache lines.
	 */
	__be32				buf __aligned(IIO_DMA_MINALIGN);
};

#define to_adf41513_state(_hw) container_of(_hw, struct adf41513_state, clk_hw)

static const u32 adf41513_cp_voltage_mv[] = {
	810, 1620, 2430, 3240, 4050, 4860, 5670, 6480, 7290, 8100,
	8910, 9720, 10530, 11340, 12150, 12960
};

static int adf41513_parse_uhz(const char *str, u64 *freq_uhz)
{
	u64 uhz = 0;
	int f_count = ADF41513_HZ_DECIMAL_PRECISION;
	bool frac_part = false;

	if (str[0] == '+')
		str++;

	while (*str && f_count > 0) {
		if ('0' <= *str && *str <= '9') {
			uhz = uhz * 10 + *str - '0';
			if (frac_part)
				f_count--;
		} else if (*str == '\n') {
			if (*(str + 1) == '\0')
				break;
			return -EINVAL;
		} else if (*str == '.' && !frac_part) {
			frac_part = true;
		} else {
			return -EINVAL;
		}
		str++;
	}

	for (; f_count > 0; f_count--)
		uhz *= 10;

	*freq_uhz = uhz;

	return 0;
}

static int adf41513_uhz_to_str(u64 freq_uhz, char *buf)
{
	u64 frac_part;
	u64 int_part = div64_u64_rem(freq_uhz, ADF41513_HZ_TO_UHZ, &frac_part);

	if (frac_part == 0)
		return sprintf(buf, "%llu\n", int_part);
	else
		return sprintf(buf, "%llu.%06llu\n", int_part, frac_part);
}

static int adf41513_sync_config(struct adf41513_state *st, u16 sync_mask)
{
	int ret;
	int i;

	/* write registers in reverse order (R13 to R0)*/
	for (i = ADF41513_REG13; i >= ADF41513_REG0; i--) {
		if (st->regs_hw[i] != st->regs[i] ||
		    sync_mask & (1 << i)) {
			st->buf = cpu_to_be32(st->regs[i] | i);
			ret = spi_write(st->spi, &st->buf, 4);
			if (ret < 0)
				return ret;
			st->regs_hw[i] = st->regs[i];
			dev_dbg(&st->spi->dev, "REG%d <= 0x%08X\n",
				i, st->regs[i] | i);
		}
	}

	return 0;
}

static u64 adf41513_pll_get_rate(struct adf41513_state *st)
{
	if (st->settings.mode == ADF41513_MODE_INVALID) {
		/* get pll settings from regs_hw */
		st->settings.int_value =
			FIELD_GET(ADF41513_REG0_INT_MSK, st->regs_hw[ADF41513_REG0]);
		st->settings.frac1 =
			FIELD_GET(ADF41513_REG1_FRAC1_MSK, st->regs_hw[ADF41513_REG1]);
		st->settings.frac2 =
			FIELD_GET(ADF41513_REG3_FRAC2_MSK, st->regs_hw[ADF41513_REG3]);
		st->settings.mod2 =
			FIELD_GET(ADF41513_REG4_MOD2_MSK, st->regs_hw[ADF41513_REG4]);
		st->settings.r_counter =
			FIELD_GET(ADF41513_REG5_R_CNT_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.ref_doubler =
			FIELD_GET(ADF41513_REG5_REF_DOUBLER_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.ref_div2 =
			FIELD_GET(ADF41513_REG5_RDIV2_MSK, st->regs_hw[ADF41513_REG5]);
		st->settings.prescaler =
			FIELD_GET(ADF41513_REG5_PRESCALER_MSK, st->regs_hw[ADF41513_REG5]);

		/* calculate pfd frequency */
		st->settings.pfd_frequency_uhz = st->ref_freq_hz * ADF41513_HZ_TO_UHZ;
		if (st->settings.ref_doubler)
			st->settings.pfd_frequency_uhz *= 2;
		if (st->settings.ref_div2)
			st->settings.pfd_frequency_uhz /= 2;
		st->settings.pfd_frequency_uhz /= st->settings.r_counter;
		st->settings.actual_frequency_uhz =
			(u64)st->settings.int_value * st->settings.pfd_frequency_uhz;

		/* check if int mode is selected */
		if (FIELD_GET(ADF41513_REG6_INT_MODE_MSK, st->regs_hw[ADF41513_REG6])) {
			st->settings.mode = ADF41513_MODE_INTEGER_N;
		} else {
			st->settings.actual_frequency_uhz +=
				mul_u64_u64_div_u64(st->settings.frac1,
						    st->settings.pfd_frequency_uhz,
						    ADF41513_FIXED_MODULUS);

			/* check if variable modulus is selected */
			if (FIELD_GET(ADF41513_REG0_VAR_MOD_MSK, st->regs_hw[ADF41513_REG0])) {
				st->settings.actual_frequency_uhz +=
					mul_u64_u64_div_u64(st->settings.frac2,
							    st->settings.pfd_frequency_uhz,
						ADF41513_FIXED_MODULUS * st->settings.mod2);

				st->settings.mode = ADF41513_MODE_VARIABLE_MODULUS;
			} else {
				st->settings.mode = ADF41513_MODE_FIXED_MODULUS;
			}
		}

		st->settings.target_frequency_uhz = st->settings.actual_frequency_uhz;
	}

	return st->settings.actual_frequency_uhz;
}

static int adf41513_calc_pfd_frequency(struct adf41513_state *st,
				       struct adf41513_pll_settings *result,
				       u64 fpfd_limit_uhz)
{
	/* set initial values from platform data */
	result->ref_div2 = st->pdata->ref_div2_en ? 1 : 0;
	result->ref_doubler = st->pdata->ref_doubler_en ? 1 : 0;

	if (st->pdata->ref_doubler_en &&
	    st->ref_freq_hz > ADF41513_MAX_REF_FREQ_DOUBLER) {
		result->ref_doubler = 0;
		dev_warn(&st->spi->dev, "Disabling ref doubler due to high reference frequency\n");
	}

	/* set R counter starting with the div factor from platform data */
	result->r_counter = st->pdata->ref_div_factor - 1;

	do {
		result->r_counter++;
		result->pfd_frequency_uhz = (st->ref_freq_hz * ADF41513_HZ_TO_UHZ *
					    (result->ref_doubler ? 2 : 1)) /
					    ((result->r_counter) *
					    (result->ref_div2 ? 2 : 1));
	} while (result->pfd_frequency_uhz > fpfd_limit_uhz);

	if (result->r_counter > ADF41513_MAX_R_CNT) {
		dev_err(&st->spi->dev, "Cannot optimize PFD frequency\n");
		return -ERANGE;
	}

	return 0;
}

static int adf41513_calc_integer_n(struct adf41513_state *st,
				   struct adf41513_pll_settings *result)
{
	const u16 max_int = (st->device_id == ADF41513_ID) ?
			    ADF41513_MAX_INT_8_9 : ADF41513_MAX_INT_4_5;

	u64 fractional_part_uhz, freq_error_uhz;
	u16 int_value = (u16)div64_u64_rem(result->target_frequency_uhz,
		result->pfd_frequency_uhz, &fractional_part_uhz);

	/* check if N is close to an integer (within tolerance) */
	if (fractional_part_uhz > (result->pfd_frequency_uhz / 2) && int_value < max_int)
		int_value++;

	/* set prescaler */
	if (st->device_id == ADF41513_ID &&
	    int_value >= ADF41513_MIN_INT_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_4_5 &&
		 int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	result->actual_frequency_uhz = (u64)int_value * result->pfd_frequency_uhz;
	freq_error_uhz = (result->actual_frequency_uhz > result->target_frequency_uhz) ?
			 (result->actual_frequency_uhz - result->target_frequency_uhz) :
			 (result->target_frequency_uhz - result->actual_frequency_uhz);

	if (freq_error_uhz > st->freq_resolution_uhz)
		return -ERANGE;

	result->mode = ADF41513_MODE_INTEGER_N;
	result->int_value = int_value;
	result->frac1 = 0;
	result->frac2 = 0;
	result->mod2 = 0;

	return 0;
}

static int adf41513_calc_fractional_n(struct adf41513_state *st,
				      struct adf41513_pll_settings *result)
{
	u64 fractional_part_uhz, tmp, remaining_frac;
	u32 frac1, frac2, mod2;
	u16 int_value = (u16)div64_u64_rem(result->target_frequency_uhz,
		result->pfd_frequency_uhz, &fractional_part_uhz);

	/* set prescaler */
	if (st->device_id == ADF41513_ID &&
	    int_value >= ADF41513_MIN_INT_FRAC_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_FRAC_4_5 &&
		 int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* calculate required MOD2 based on target resolution / 2 (enhanced for uHz) */
	mod2 = (u32)min_t(u64, DIV64_U64_ROUND_CLOSEST(result->pfd_frequency_uhz * 2,
					  st->freq_resolution_uhz * ADF41513_FIXED_MODULUS),
					  ADF41513_MAX_MOD2);

	/* ensure MOD2 is at least 1 */
	if (mod2 < 1)
		mod2 = 1;

	/* calculate frac1 and frac2 */
	frac1 = (u32)mul_u64_u64_div_u64(fractional_part_uhz, ADF41513_FIXED_MODULUS,
		result->pfd_frequency_uhz);
	remaining_frac = fractional_part_uhz -
		mul_u64_u64_div_u64((u64)frac1, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS);
	frac2 = (u32)mul_u64_u64_div_u64(remaining_frac, ADF41513_FIXED_MODULUS * mod2,
		result->pfd_frequency_uhz);

	/* calculate actual frequency in uHz */
	tmp = (u64)frac1 * mod2 + frac2;
	result->actual_frequency_uhz = (u64)int_value * result->pfd_frequency_uhz +
		mul_u64_u64_div_u64(tmp, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS * mod2);

	result->mode = (frac2 == 0) ? ADF41513_MODE_FIXED_MODULUS : ADF41513_MODE_VARIABLE_MODULUS;
	result->int_value = int_value;
	result->frac1 = frac1;
	result->frac2 = frac2;
	result->mod2 = mod2;

	return 0;
}

static int adf41513_calculate_pll_settings(struct adf41513_state *st,
					   struct adf41513_pll_settings *result,
					   u64 rf_out_uhz)
{
	const u64 max_rf_freq_uhz = ADF41513_HZ_TO_UHZ * ((st->device_id == ADF41513_ID) ?
				    ADF41513_MAX_RF_FREQ : ADF41510_MAX_RF_FREQ);
	const u64 min_rf_freq_uhz = ADF41513_MIN_RF_FREQ * ADF41513_HZ_TO_UHZ;

	u64 pfd_freq_limit_uhz;
	int ret;

	/* input validation */
	if (rf_out_uhz < min_rf_freq_uhz ||
	    rf_out_uhz > max_rf_freq_uhz) {
		dev_err(&st->spi->dev, "RF frequency %llu uHz out of range [%llu, %llu] Hz\n",
			rf_out_uhz,
			min_rf_freq_uhz / ADF41513_HZ_TO_UHZ,
			max_rf_freq_uhz / ADF41513_HZ_TO_UHZ);
		return -EINVAL;
	}
	result->target_frequency_uhz = rf_out_uhz;

	/* try integer-N first (best phase noise performance) */
	pfd_freq_limit_uhz = min_t(u64, rf_out_uhz / ADF41513_MIN_INT_4_5,
				   ADF41513_MAX_PFD_FREQ_INT_N_UHZ);
	ret = adf41513_calc_pfd_frequency(st, result, pfd_freq_limit_uhz);
	if (ret < 0)
		return ret;

	ret = adf41513_calc_integer_n(st, result);
	if (ret < 0) {
		/* try fractional-N: recompute pfd frequency if necessary */
		pfd_freq_limit_uhz = min_t(u64, rf_out_uhz / ADF41513_MIN_INT_FRAC_4_5,
					   ADF41513_MAX_PFD_FREQ_FRAC_N_UHZ);
		if (pfd_freq_limit_uhz < result->pfd_frequency_uhz) {
			ret = adf41513_calc_pfd_frequency(st, result, pfd_freq_limit_uhz);
			if (ret < 0)
				return ret;
		}

		ret = adf41513_calc_fractional_n(st, result);
		if (ret < 0) {
			dev_err(&st->spi->dev,
				"no valid PLL configuration found for %llu uHz\n", rf_out_uhz);
			return -EINVAL;
		}
	}

	return 0;
}

static int adf41513_set_frequency(struct adf41513_state *st, u64 freq_uhz, u16 sync_mask)
{
	struct adf41513_pll_settings result;
	u32 bleed_value = 0;
	int ret;

	/* calculate pll settings candidate */
	ret = adf41513_calculate_pll_settings(st, &result, freq_uhz);
	if (ret < 0)
		return ret;

	/* apply computed results to state pll settings */
	memcpy(&st->settings, &result, sizeof(struct adf41513_pll_settings));

	/* log calculation result */
	dev_dbg(&st->spi->dev, "%s mode: int=%u, frac1=%u, frac2=%u, mod2=%u, fpdf=%llu Hz, prescaler=%s\n",
		(result.mode == ADF41513_MODE_INTEGER_N) ? "integer-n" :
		(result.mode == ADF41513_MODE_FIXED_MODULUS) ? "fixed-modulus" : "variable-modulus",
		result.int_value, result.frac1, result.frac2, result.mod2,
		result.pfd_frequency_uhz / ADF41513_HZ_TO_UHZ,
		result.prescaler ? "8/9" : "4/5");

	/* int */
	st->regs[ADF41513_REG0] = ADF41513_REG0_INT(st->settings.int_value);
	if (st->settings.mode == ADF41513_MODE_VARIABLE_MODULUS)
		st->regs[ADF41513_REG0] |= ADF41513_REG0_VAR_MOD_MSK;
	/* frac1 */
	st->regs[ADF41513_REG1] = ADF41513_REG1_FRAC1(st->settings.frac1);
	if (st->settings.mode != ADF41513_MODE_INTEGER_N)
		st->regs[ADF41513_REG1] |= ADF41513_REG1_DITHER2_MSK;

	/* frac2 */
	st->regs[ADF41513_REG3] = ADF41513_REG3_FRAC2(st->settings.frac2);
	/* mod2 */
	st->regs[ADF41513_REG4] &= ADF41513_REG4_MOD2_MSK;
	st->regs[ADF41513_REG4] |= ADF41513_REG4_MOD2(st->settings.mod2);

	/* r-cnt | doubler | rdiv2 | prescaler */
	st->regs[ADF41513_REG5] &= ~(ADF41513_REG5_R_CNT_MSK |
				     ADF41513_REG5_REF_DOUBLER_MSK |
				     ADF41513_REG5_RDIV2_MSK |
				     ADF41513_REG5_PRESCALER_MSK);
	st->regs[ADF41513_REG5] |= ADF41513_REG5_R_CNT(st->settings.r_counter % 32) |
				   ADF41513_REG5_REF_DOUBLER(st->settings.ref_doubler) |
				   ADF41513_REG5_RDIV2(st->settings.ref_div2) |
				   ADF41513_REG5_PRESCALER(st->settings.prescaler);

	/* Enable integer mode if no fractional part */
	if (st->settings.frac1 == 0 &&
	    st->settings.frac2 == 0) {
		/* integer mode */
		st->regs[ADF41513_REG6] |= ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_ENABLE_MSK;
	} else {
		/* fractional mode */
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_ENABLE_MSK;
	}

	/* set bleed current value */
	if (st->pdata->phase_detector_polarity)
		bleed_value = 90;
	else
		bleed_value = 144;

	bleed_value *= FIELD_GET(ADF41513_REG5_CP_CURRENT_MSK, st->regs[ADF41513_REG5]) + 1;
	bleed_value = (u32)mul_u64_u64_div_u64(st->settings.pfd_frequency_uhz,
		(u64)bleed_value, 16 * 100000000000000ULL /* 100 MHz in uHz */);

	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_CURRENT_MSK;
	st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_CURRENT(bleed_value);

	return adf41513_sync_config(st, sync_mask | ADF41513_SYNC_REG0);
}

static int adf41513_suspend(struct adf41513_state *st)
{
	st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN(1);
	if (st->pdata->load_enable_sync)
		st->regs[ADF41513_REG12] &= ~ADF41513_REG12_LE_SELECT_MSK;

	return adf41513_sync_config(st, ADF41513_SYNC_DIFF);
}

static int adf41513_resume(struct adf41513_state *st)
{
	int ret;

	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_POWER_DOWN_MSK;
	ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	if (ret < 0)
		return ret;

	/* check if load enable sync needs to be set */
	if (st->pdata->load_enable_sync) {
		st->regs[ADF41513_REG12] |= ADF41513_REG12_LE_SELECT_MSK;
		ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	}

	return ret;
}

static ssize_t adf41513_read(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     char *buf)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	u64 val = 0;
	int ret = 0;

	guard(mutex)(&st->lock);

	switch ((u32)private) {
	case ADF41513_FREQ:
		ret = adf41513_uhz_to_str(adf41513_pll_get_rate(st), buf);
		if (st->lock_detect)
			if (!gpiod_get_value(st->lock_detect)) {
				dev_dbg(&st->spi->dev, "PLL un-locked\n");
				ret = -EBUSY;
			}
		break;
	case ADF41513_FREQ_REFIN:
		if (st->ref_clk)
			st->ref_freq_hz = clk_get_rate(st->ref_clk);

		val = st->ref_freq_hz;
		ret = sprintf(buf, "%llu\n", val);
		break;
	case ADF41513_FREQ_RESOLUTION:
		ret = adf41513_uhz_to_str(st->freq_resolution_uhz, buf);
		break;
	case ADF41513_POWER_DOWN:
		val = (u64)FIELD_GET(ADF41513_REG6_POWER_DOWN_MSK, st->regs_hw[ADF41513_REG6]);
		ret = sprintf(buf, "%llu\n", val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t adf41513_write(struct iio_dev *indio_dev,
			      uintptr_t private,
			      const struct iio_chan_spec *chan,
			      const char *buf, size_t len)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned long tmp;
	u64 freq_uhz;
	int ret;

	guard(mutex)(&st->lock);

	switch ((u32)private) {
	case ADF41513_FREQ:
		ret = adf41513_parse_uhz(buf, &freq_uhz);
		if (ret)
			return ret;
		ret = adf41513_set_frequency(st, freq_uhz, ADF41513_SYNC_DIFF);
		break;
	case ADF41513_FREQ_REFIN:
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		if (readin < ADF41513_MIN_REF_FREQ || readin > ADF41513_MAX_REF_FREQ) {
			ret = -EINVAL;
			break;
		}

		if (st->ref_clk) {
			tmp = clk_round_rate(st->ref_clk, readin);
			if (tmp != readin) {
				ret = -EINVAL;
				break;
			}
			ret = clk_set_rate(st->ref_clk, tmp);
			if (ret < 0)
				break;
		}
		st->ref_freq_hz = readin;
		/* update RF out */
		ret = adf41513_set_frequency(st,
					     st->settings.target_frequency_uhz,
					     ADF41513_SYNC_DIFF);
		break;
	case ADF41513_FREQ_RESOLUTION:
		ret = adf41513_parse_uhz(buf, &freq_uhz);
		if (ret)
			return ret;
		if (freq_uhz == 0)
			ret = -EINVAL;
		else
			st->freq_resolution_uhz = freq_uhz;
		break;
	case ADF41513_POWER_DOWN:
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		if (readin)
			ret = adf41513_suspend(st);
		else
			ret = adf41513_resume(st);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : len;
}

#define _ADF41513_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = adf41513_read, \
	.write = adf41513_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adf41513_ext_info[] = {
	/*
	 * Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 with sub-Hz resolution
	 */
	_ADF41513_EXT_INFO("frequency", ADF41513_FREQ),
	_ADF41513_EXT_INFO("frequency_resolution", ADF41513_FREQ_RESOLUTION),
	_ADF41513_EXT_INFO("refin_frequency", ADF41513_FREQ_REFIN),
	_ADF41513_EXT_INFO("powerdown", ADF41513_POWER_DOWN),
	{ },
};

static const struct iio_chan_spec adf41513_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.ext_info = adf41513_ext_info,
};

static int adf41513_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int writeval,
			       unsigned int *readval)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	int ret;

	if (reg > ADF41513_REG13)
		return -EINVAL;

	guard(mutex)(&st->lock);

	if (!readval) {
		/* direct register access invalidates cached pll settings */
		st->settings.mode = ADF41513_MODE_INVALID;

		st->regs[reg] = writeval & ~0xF; /* Clear control bits */
		ret = adf41513_sync_config(st, 1 << reg);
	} else {
		*readval = st->regs_hw[reg];
		ret = 0;
	}

	return ret;
}

static const struct iio_info adf41513_info = {
	.debugfs_reg_access = &adf41513_reg_access,
};

#ifdef CONFIG_OF
static void adf41513_clk_del_provider(void *data)
{
	struct adf41513_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}
#endif

static unsigned long adf41513_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return (unsigned long)adf41513_pll_get_rate(st) / ADF41513_HZ_TO_UHZ;
}

static long adf41513_clk_round_rate(struct clk_hw *hw,
				    unsigned long rate,
				    unsigned long *parent_rate)
{
	return rate;
}

static int adf41513_clk_set_rate(struct clk_hw *hw,
				 unsigned long rate,
				 unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	u64 freq_uhz = (u64)rate * ADF41513_HZ_TO_UHZ;

	if (parent_rate < ADF41513_MIN_REF_FREQ ||
	    parent_rate > ADF41513_MAX_REF_FREQ)
		return -EINVAL;

	guard(mutex)(&st->lock);
	st->ref_freq_hz = parent_rate;
	return adf41513_set_frequency(st, freq_uhz, ADF41513_SYNC_DIFF);
}

static int adf41513_clk_prepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return adf41513_resume(st);
}

static void adf41513_clk_unprepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	adf41513_suspend(st);
}

static int adf41513_clk_is_enabled(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return (st->regs_hw[ADF41513_REG6] & ADF41513_REG6_POWER_DOWN_MSK) == 0;
}

static const struct clk_ops adf41513_clk_ops = {
	.recalc_rate = adf41513_clk_recalc_rate,
	.round_rate = adf41513_clk_round_rate,
	.set_rate = adf41513_clk_set_rate,
	.prepare = adf41513_clk_prepare,
	.unprepare = adf41513_clk_unprepare,
	.is_enabled = adf41513_clk_is_enabled,
};

static int adf41513_clk_register(struct adf41513_state *st)
{
#ifdef CONFIG_OF
	struct spi_device *spi = st->spi;
	struct clk_init_data init;
	struct clk *clk = NULL;
	const char *parent_name;
	int ret;

	if (!device_property_present(&spi->dev, "#clock-cells"))
		return 0;

	if (device_property_read_string(&spi->dev, "clock-output-names", &init.name)) {
		init.name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s-clk",
					   fwnode_get_name(dev_fwnode(&spi->dev)));
		if (!init.name)
			return -ENOMEM;
	}

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.ops = &adf41513_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	st->clk_hw.init = &init;
	clk = devm_clk_register(&spi->dev, &st->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
	if (ret)
		return ret;

	st->clk_out = clk;

	return devm_add_action_or_reset(&spi->dev, adf41513_clk_del_provider, st);
#else
	return 0;
#endif
}

static struct adf41513_platform_data *adf41513_parse_dt(struct device *dev)
{
	struct adf41513_platform_data *pdata __free(kfree) = NULL;
	u32 tmp;
	u32 cp_resistance;
	u32 cp_current;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	snprintf(pdata->name, sizeof(pdata->name), "%pfw", dev_fwnode(dev));

	/* power-up frequency - optional */
	pdata->power_up_frequency = 0;
	device_property_read_u64(dev, "adi,power-up-frequency", &pdata->power_up_frequency);

	/* reference divider factor - optional minimal value */
	pdata->ref_div_factor = ADF41513_MIN_R_CNT; /* Default R = 1 */
	if (!device_property_read_u32(dev, "adi,reference-div-factor", &tmp)) {
		if (tmp >= ADF41513_MIN_R_CNT && tmp <= ADF41513_MAX_R_CNT)
			pdata->ref_div_factor = (u8)tmp;
		else
			dev_warn(dev, "Invalid reference div factor %u\n", tmp);
	}

	/* reference controls */
	pdata->ref_doubler_en = device_property_read_bool(dev, "adi,reference-doubler-enable");
	pdata->ref_div2_en = device_property_read_bool(dev, "adi,reference-div2-enable");

	/* charge pump resistor */
	cp_resistance = 2700; /* Default 2.7 kOhms */
	if (!device_property_read_u32(dev, "adi,charge-pump-resistor", &tmp)) {
		if (tmp < 1800) {
			dev_warn(dev, "RSET value too low %u Ohms\n", tmp);
			tmp = 1800;
		} else if (tmp > 10000) {
			dev_warn(dev, "RSET value too high %u Ohms\n", tmp);
			tmp = 10000;
		}
		cp_resistance = tmp;
	}

	/* charge pump current */
	pdata->charge_pump_voltage_mv = 6480; /* Default 2.7 kOhm * 2.4 mA = 6.48 V */
	if (!device_property_read_u32(dev, "adi,charge-pump-current", &cp_current)) {
		tmp = (cp_current * cp_resistance) / 1000; /* Convert to mV */
		 /* Validate charge pump voltage */
		if (tmp < ADF41513_MIN_CP_VOLTAGE_MV) {
			dev_warn(dev, "Charge pump current too low %u uA (R_set = %u Ohms)\n",
				 cp_current, cp_resistance);
			tmp = ADF41513_MIN_CP_VOLTAGE_MV;
		} else if (tmp > ADF41513_MAX_CP_VOLTAGE_MV) {
			dev_warn(dev, "Charge pump current too high %u uA (R_set = %u Ohms)\n",
				 cp_current, cp_resistance);
			tmp = ADF41513_MAX_CP_VOLTAGE_MV;
		}
		pdata->charge_pump_voltage_mv = tmp;
	}

	/* phase detector polarity */
	pdata->phase_detector_polarity =
		device_property_read_bool(dev, "adi,phase-detector-polarity-positive");

	/* muxout selection */
	pdata->muxout_select = ADF41513_MUXOUT_TRISTATE; /* Default to tristate */
	if (!device_property_read_u32(dev, "adi,muxout-select", &tmp)) {
		if (tmp <= ADF41513_MUXOUT_N_DIV2)
			pdata->muxout_select = (u8)tmp;
		else
			dev_warn(dev, "Invalid muxout select: %u\n", tmp);
	}

	/* muxout logic level: default 3v3 */
	pdata->muxout_1v8_en = device_property_read_bool(dev, "adi,muxout-level-1v8-enable");

	/* lock detector settings */
	pdata->lock_detect_bias = 0; /* default 0 */
	if (!device_property_read_u32(dev, "adi,lock-detect-bias", &tmp)) {
		if (tmp > 3)
			dev_warn(dev, "Invalid lock detect bias: %u\n", tmp);
		else
			pdata->lock_detect_bias = (u8)tmp;
	}

	pdata->lock_detect_precision = 0; /* default 0 */
	if (!device_property_read_u32(dev, "adi,lock-detect-precision", &tmp)) {
		if (tmp > 3)
			dev_warn(dev, "Invalid lock detect precision: %u\n", tmp);
		else
			pdata->lock_detect_precision = (u8)tmp;
	}

	pdata->lock_detect_count = 0; /* default 0 */
	if (!device_property_read_u32(dev, "adi,lock-detect-count", &tmp)) {
		if (tmp > 7) {
			dev_warn(dev, "Lock detect count too high %u\n", tmp);
			tmp = 7;
		}
		pdata->lock_detect_count = (u8)tmp;
	}

	/* lock detect clk select */
	pdata->fast_lock_en = device_property_read_bool(dev, "adi,fast-lock-enable");

	/* phase resync configuration */
	pdata->phase_resync_en = device_property_read_bool(dev, "adi,phase-resync-enable");

	pdata->phase_resync_clk_div[0] = 0;
	pdata->phase_resync_clk_div[1] = 0;
	if (!device_property_read_u16_array(dev,
					    "adi,12bit-clk-divider",
					    pdata->phase_resync_clk_div, 2)) {
		if (pdata->phase_resync_clk_div[0] > ADF41513_MAX_CLK_DIVIDER) {
			dev_warn(dev, "Invalid clk1 divider: %u\n", pdata->phase_resync_clk_div[0]);
			pdata->phase_resync_clk_div[0] = 0;
		}
		if (pdata->phase_resync_clk_div[1] > ADF41513_MAX_CLK_DIVIDER) {
			dev_warn(dev, "Invalid clk2 divider: %u\n", pdata->phase_resync_clk_div[1]);
			pdata->phase_resync_clk_div[1] = 0;
		}
	}

	/* load enable sync with ref input */
	pdata->load_enable_sync = device_property_read_bool(dev, "adi,load-enable-sync");

	/* initial frequency resolution: Default to 1 Hz */
	pdata->freq_resolution_uhz = ADF41513_HZ_TO_UHZ;
	device_property_read_u64(dev, "adi,freq-resolution-uhz", &pdata->freq_resolution_uhz);
	if (pdata->freq_resolution_uhz == 0)
		pdata->freq_resolution_uhz = ADF41513_HZ_TO_UHZ;

	return no_free_ptr(pdata);
}

static int adf41513_setup(struct adf41513_state *st)
{
	int ret;
	u32 cp_index;

	memset(st->regs_hw, 0xFF, sizeof(st->regs_hw));
	memset(st->regs, 0x00, sizeof(st->regs));

	/* assume DLD pin is used for digital lock detect */
	st->regs[ADF41513_REG5] |= ADF41513_REG5_DLD_MODES(ADF41513_DLD_DIG_LD);

	/* configure charge pump current settings */
	cp_index = find_closest(st->pdata->charge_pump_voltage_mv,
				adf41513_cp_voltage_mv,
				ARRAY_SIZE(adf41513_cp_voltage_mv));
	st->regs[ADF41513_REG5] |= ADF41513_REG5_CP_CURRENT(cp_index);

	/* Configure phase detector polarity */
	if (st->pdata->phase_detector_polarity)
		st->regs[ADF41513_REG6] |= ADF41513_REG6_PD_POLARITY_MSK;

	/* narrow ABP | loss of lock detect enable | SD reset | LDP from pdata */
	st->regs[ADF41513_REG6] |= ADF41513_REG6_ABP_MSK |
				   ADF41513_REG6_LOL_ENABLE_MSK |
				   ADF41513_REG6_SD_RESET_MSK |
				   ADF41513_REG6_LDP(st->pdata->lock_detect_precision);

	/* LD count from pdata | PS bias */
	st->regs[ADF41513_REG7] |= ADF41513_REG7_LD_COUNT(st->pdata->lock_detect_count) |
				   ADF41513_REG7_LD_CLK_SEL(st->pdata->fast_lock_en) |
				   ADF41513_REG7_PS_BIAS(2);

	/* enable phase resync and configure the clk divs */
	st->regs[ADF41513_REG5] |= ADF41513_REG5_CLK1_DIV(st->pdata->phase_resync_clk_div[0]);
	st->regs[ADF41513_REG7] |= ADF41513_REG7_CLK_DIV_MODE(st->pdata->phase_resync_en ? 2 : 0) |
				   ADF41513_REG7_CLK2_DIV(st->pdata->phase_resync_clk_div[1]);

	/* lock detect bias */
	st->regs[ADF41513_REG9] |= ADF41513_REG9_LD_BIAS(st->pdata->lock_detect_bias);

	/* power down select */
	st->regs[ADF41513_REG11] |= ADF41513_REG11_POWER_DOWN_SEL_MSK;

	/* muxout */
	st->regs[ADF41513_REG12] |= ADF41513_REG12_MUXOUT(st->pdata->muxout_select) |
				    ADF41513_REG12_LOGIC_LEVEL(st->pdata->muxout_1v8_en ? 0 : 1) |
				    ADF41513_REG12_LE_SELECT(st->pdata->phase_resync_en);

	/* set power-up frequency if specified */
	if (st->pdata->power_up_frequency) {
		ret = adf41513_set_frequency(st,
					     (u64)st->pdata->power_up_frequency *
						ADF41513_HZ_TO_UHZ,
					     ADF41513_SYNC_ALL);
		if (ret < 0) {
			dev_err(&st->spi->dev, "Failed to set power-up frequency: %d\n", ret);
			return ret;
		}
	} else {
		/* init powered down */
		st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN(1);
		st->regs[ADF41513_REG12] &= ~ADF41513_REG12_LE_SELECT_MSK;

		/* write initial configuration */
		ret = adf41513_sync_config(st, ADF41513_SYNC_ALL);
		if (ret < 0) {
			dev_err(&st->spi->dev, "Failed to write initial config: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static void adf41513_power_down(void *data)
{
	struct adf41513_state *st = data;

	guard(mutex)(&st->lock);
	adf41513_suspend(st);

	if (st->chip_enable)
		gpiod_set_value_cansleep(st->chip_enable, 0);
}

static int adf41513_pm_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adf41513_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return adf41513_suspend(st);
}

static int adf41513_pm_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adf41513_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return adf41513_resume(st);
}

static int adf41513_probe(struct spi_device *spi)
{
	struct adf41513_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct adf41513_state *st;
	struct clk *ref_clk;

	int ret;

	/* Parse device tree or platform data */
	if (dev_fwnode(&spi->dev)) {
		pdata = adf41513_parse_dt(&spi->dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	} else {
		pdata = spi->dev.platform_data;
		if (!pdata) {
			dev_err(&spi->dev, "No platform data provided\n");
			return -EINVAL;
		}
	}

	/* Get reference clock */
	if (!pdata->clkin) {
		ref_clk = devm_clk_get_prepared(&spi->dev, "clkin");
		if (IS_ERR(ref_clk))
			return -EPROBE_DEFER;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->pdata = pdata;
	st->ref_clk = ref_clk;
	st->freq_resolution_uhz = pdata->freq_resolution_uhz;
	st->device_id = (u32)spi_get_device_id(spi)->driver_data;

	spi_set_drvdata(spi, indio_dev);

	/* vcc regulator */
	ret = devm_regulator_get_enable(&spi->dev, "vcc");
	if (ret) {
		return dev_err_probe(&spi->dev, ret,
				     "failed to get and enable vcc regulator\n");
	}

	/* get chip enable gpio */
	st->chip_enable = devm_gpiod_get_optional(&spi->dev, "chip-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(st->chip_enable)) {
		dev_err(&spi->dev, "fail to request chip enable GPIO\n");
		return PTR_ERR(st->chip_enable);
	}

	/* get lock detect gpio */
	st->lock_detect = devm_gpiod_get_optional(&spi->dev, "lock-detect", GPIOD_IN);
	if (IS_ERR(st->lock_detect)) {
		dev_err(&spi->dev, "fail to request lock detect GPIO\n");
		return PTR_ERR(st->lock_detect);
	}

	/* set reference frequency */
	if (st->ref_clk)
		st->ref_freq_hz = clk_get_rate(st->ref_clk);
	else
		st->ref_freq_hz = pdata->clkin;

	/* validate reference frequency */
	if (st->ref_freq_hz < ADF41513_MIN_REF_FREQ ||
	    st->ref_freq_hz > ADF41513_MAX_REF_FREQ) {
		dev_err(&spi->dev, "Reference frequency %llu Hz out of range\n",
			st->ref_freq_hz);
		return -EINVAL;
	}

	mutex_init(&st->lock);

	ret = adf41513_clk_register(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register clock: %d\n", ret);
		return ret;
	}

	/* configure IIO device */
	indio_dev->name = pdata->name[0] ? pdata->name : "adf41513";
	indio_dev->info = &adf41513_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adf41513_chan;
	indio_dev->num_channels = 1;

	/* initialize device */
	ret = adf41513_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Device setup failed: %d\n", ret);
		return ret;
	}

	/* add power down action */
	ret = devm_add_action_or_reset(&spi->dev, adf41513_power_down, st);
	if (ret) {
		dev_err(&spi->dev, "Failed to add power down action: %d\n", ret);
		return ret;
	}

	/* register IIO device */
	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register IIO device: %d\n", ret);
		return ret;
	}

	dev_info(&spi->dev, "ADF41513 PLL synthesizer registered\n");

	return 0;
}

static const struct of_device_id adf41513_of_match[] = {
	{ .compatible = "adi,adf41510" },
	{ .compatible = "adi,adf41513" },
	{ }
};
MODULE_DEVICE_TABLE(of, adf41513_of_match);

static const struct spi_device_id adf41513_id[] = {
	{"adf41510", ADF41510_ID},
	{"adf41513", ADF41513_ID},
	{}
};
MODULE_DEVICE_TABLE(spi, adf41513_id);

DEFINE_SIMPLE_DEV_PM_OPS(adf41513_pm_ops,
			 adf41513_pm_suspend,
			 adf41513_pm_resume);

static struct spi_driver adf41513_driver = {
	.driver = {
		.name = "adf41513",
		.pm = pm_ptr(&adf41513_pm_ops),
		.of_match_table = adf41513_of_match,
	},
	.probe = adf41513_probe,
	.id_table = adf41513_id,
};
module_spi_driver(adf41513_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF41513 PLL Frequency Synthesizer");
MODULE_LICENSE("GPL");
