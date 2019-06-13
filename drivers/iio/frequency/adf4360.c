// SPDX-License-Identifier: GPL-2.0
/*
 *
 * ADF4360 PLL with Integrated Synthesizer and VCO
 *
 * Copyright © 2014-2018 Analog Devices Inc.
 * Copyright © 2019 Edward Kigwana.
 *
 * Parts lifted from clk/clk-adf4360.c by Lars-Peter Clausen.
 * Parts lifted from iio/frequency/adf4350.c by Michael Hennerich.
 * Parts lifted from iio/frequency/adf4371.c by Stefan Popa.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

/* Registers */
#define ADF4360_REG_CTRL	0x00
#define ADF4360_REG_RDIV	0x01
#define ADF4360_REG_NDIV	0x02

/* Control Bit Definitions */
#define ADF4360_GEN1_CTRL_PC_5		0
#define ADF4360_GEN1_CTRL_PC_10		1
#define ADF4360_GEN1_CTRL_PC_15		2
#define ADF4360_GEN1_CTRL_PC_20		3
#define ADF4360_CTRL_CPL(x)		(((x) & 0x3) << 2)

#define ADF4360_GEN2_CTRL_PC_2_5	(0x0 << 2)
#define ADF4360_GEN2_CTRL_PC_5		(0x1 << 2)
#define ADF4360_GEN2_CTRL_PC_7_5	(0x2 << 2)
#define ADF4360_GEN2_CTRL_PC_10		(0x3 << 2)

#define ADF4360_CTRL_COUNTER_RESET	BIT(4)
#define ADF4360_CTRL_PDP		BIT(8)
#define ADF4360_CTRL_MTLD		BIT(11)
#define ADF4360_CTRL_POWER_DOWN_EN	(1 << 20)
#define ADF4360_CTRL_PL_3_5		0
#define ADF4360_CTRL_PL_5		1
#define ADF4360_CTRL_PL_7_5		2
#define ADF4360_CTRL_PL_11		3
#define ADF4360_CTRL_PL(x)		(((x) & 0x3) << 12)
#define ADF4360_CTRL_CPI1(x)		(((x) & 0x7) << 14)
#define ADF4360_CTRL_CPI2(x)		(((x) & 0x7) << 17)
#define ADF4360_CTRL_PRESCALER_8	(0 << 22)
#define ADF4360_CTRL_PRESCALER_16	(1 << 22)
#define ADF4360_CTRL_PRESCALER_32	(2 << 22)

#define ADF4360_CTRL_MUXOUT(x)		(((x) & 7) << 5)
#define ADF4360_CTRL_MUXOUT_THREE_STATE	0
#define ADF4360_CTRL_MUXOUT_LOCK_DETECT	1
#define ADF4360_CTRL_MUXOUT_NDIV	2
#define ADF4360_CTRL_MUXOUT_DVDD	3
#define ADF4360_CTRL_MUXOUT_RDIV	4
#define ADF4360_CTRL_MUXOUT_OD_LD	5
#define ADF4360_CTRL_MUXOUT_SDO		6
#define ADF4360_CTRL_MUXOUT_GND		7

#define ADF4360_CPI_0_31	0
#define ADF4360_CPI_0_62	1
#define ADF4360_CPI_0_93	2
#define ADF4360_CPI_1_25	3
#define ADF4360_CPI_1_56	4
#define ADF4360_CPI_1_87	5
#define ADF4360_CPI_2_18	6
#define ADF4360_CPI_2_50	7

/* N Counter Bit Definitions */
#define ADF4360_NDIV_A_COUNTER(x)	((x) << 2)
#define ADF4360_NDIV_B_COUNTER(x)	((x) << 8)
#define ADF4360_NDIV_OUT_DIV2		BIT(22)
#define ADF4360_NDIV_PRESCALER_DIV2	BIT(23)

/* R Counter Bit Definitions */
#define ADF4360_RDIV_R_COUNTER(x)	((x) << 2)
#define ADF4360_RDIV_ABP(x)		(((x) & 0x3) << 16)
#define ADF4360_RDIV_ABP_3_0NS		0
#define ADF4360_RDIV_ABP_1_3NS		1
#define ADF4360_RDIV_ABP_6_0NS		2
#define ADF4360_RDIV_BSC_1		(0x0 << 20)
#define ADF4360_RDIV_BSC_2		(0x1 << 20)
#define ADF4360_RDIV_BSC_4		(0x2 << 20)
#define ADF4360_RDIV_BSC_8		(0x3 << 20)

/* Specifications */
#define ADF4360_MIN_FREQ_REFIN		10000000 /* Hz */
#define ADF4360_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF4360_MAX_PFD_RATE		8000000 /* 8 MHz */
#define ADF4360_MAX_COUNTER_RATE	300000000 /* 300 MHz */

/**
 * struct adf4360_platform_data - platform specific information.
 * @clkin:		REFin frequency in Hz.
 * @power_up_frequency:	Optional, if set in Hz the PLL tunes to the desired
 *			frequency on probe.
 * @vco_min:		Optional mininum VCO frequency in Hz for devices
 *			configured using external hardware components.
 * @vco_max:		Optional maximum VCO frequency in Hz for devices
 *			configured using external hardware components.
 * @pfd_freq:		Phase frequency detector frequency in Hz.
 * @cpl:		Core power level setting.
 * @cpi:		Loop filter charge pump current.
 * @opl:		Output power level setting.
 * @pdp:		Phase detector polarity positive enable.
 * @mtld:		Optional, mutes output until PLL lock is detected.
 * @mux_out_ctrl:	Output multiplexer configuration.
 *			Defaults to lock detect if not set.
 * @abp:		Anti backlash setting.
 * @gpio_lock_detect:	Optional, if set with a valid GPIO number,
 *			pll lock state is tested upon read.
 *			If not used - set to -1.
 */

struct adf4360_platform_data {
	unsigned long power_up_frequency;
	unsigned int vco_min;
	unsigned int vco_max;
	unsigned int pfd_freq;
	unsigned int cpl;
	unsigned int cpi;
	unsigned int opl;
	bool pdp;
	bool mtld;
	unsigned int mux_out_ctrl;
	unsigned int abp;
	int gpio_lock_detect;
};

enum {
	ADF4360_FREQ,
	ADF4360_FREQ_REFIN,
	ADF4360_PWRDOWN,
};

struct adf4360_outputs {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
};

#define to_adf4360_outputs(_hw) container_of(_hw, struct adf4360_outputs, hw)

struct adf4360_state {
	struct spi_device *spi;
	struct regulator *reg;
	struct adf4360_platform_data *pdata;
	struct clk *clkin;
	struct adf4360_outputs outputs;
	unsigned long r;
	unsigned long n;
	unsigned long clkin_freq;
	unsigned int part_id;
	unsigned long regs[3];
	unsigned long regs_hw[3];
	unsigned long freq_req;

	u8 spi_data[3] ____cacheline_aligned;
};

static struct adf4360_platform_data default_pdata = {
	.pfd_freq = 25000,
	.cpi = ADF4360_CPI_1_25,
	.opl = ADF4360_CTRL_PL_11,
	.pdp = 1,
	.mtld = 1,
	.mux_out_ctrl = ADF4360_CTRL_MUXOUT_LOCK_DETECT,
	.abp = ADF4360_RDIV_ABP_3_0NS,
	.gpio_lock_detect = -1,
};

static int adf4360_sync_config(struct adf4360_state *st, bool sync_all)
{
	int ret, i = 0;
	int reg[] = {ADF4360_REG_RDIV, ADF4360_REG_CTRL, ADF4360_REG_NDIV};
	unsigned int val = 0;

	for (i = 0; i < sizeof(st->regs); ++i) {
		if ((st->regs_hw[reg[i]] != st->regs[reg[i]]) || sync_all) {
			val |= reg[i];

			st->spi_data[0] = (val >> 16) & 0xff;
			st->spi_data[1] = (val >> 8) & 0xff;
			st->spi_data[2] = val & 0xff;

			if (reg[i] == ADF4360_REG_NDIV)
				usleep_range(15000, 20000);

			ret = spi_write(st->spi, st->spi_data, sizeof(st->spi_data));
			if (ret < 0)
				return ret;
			st->regs_hw[reg[i]] = st->regs[reg[i]];
			dev_dbg(&st->spi->dev, "[%d] 0x%X\n",
				i, (u32)st->regs[reg[i]] | reg[i]);
		}
	}

	return 0;
}

static int adf4360_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct adf4360_state *st = iio_priv(indio_dev);
	int ret;

	if (reg > ADF4360_REG_NDIV)
		return -EINVAL;

	if (readval == NULL) {
		st->regs[reg] = writeval & ~(BIT(0) | BIT(1));
		ret = adf4360_sync_config(st, true);
	} else {
		*readval = st->regs_hw[reg];
		ret = 0;
	}

	return ret;
}

struct adf4360_part_info {
	unsigned int vco_min;
	unsigned int vco_max;
	unsigned int default_cpl;
};

static const struct adf4360_part_info adf4360_part_info[] = {
	{	/* ADF4360-0 */
		.vco_min = 2400000000U,
		.vco_max = 2725000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_10,
	}, {	/* ADF4360-1 */
		.vco_min = 2050000000U,
		.vco_max = 2450000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_15,
	}, {	/* ADF4360-2 */
		.vco_min = 1850000000U,
		.vco_max = 2170000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_15,
	}, {	/* ADF4360-3 */
		.vco_min = 1600000000U,
		.vco_max = 1950000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_15,
	}, {	/* ADF4360-4 */
		.vco_min = 1450000000U,
		.vco_max = 1750000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_15,
	}, {	/* ADF4360-5 */
		.vco_min = 1200000000U,
		.vco_max = 1400000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_10,
	}, {	/* ADF4360-6 */
		.vco_min = 1050000000U,
		.vco_max = 1250000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_10,
	}, {	/* ADF4360-7 */
		.vco_min = 350000000U,
		.vco_max = 1800000000U,
		.default_cpl = ADF4360_GEN1_CTRL_PC_5,
	}, {	/* ADF4360-8 */
		.vco_min = 65000000U,
		.vco_max = 400000000U,
		.default_cpl = ADF4360_GEN2_CTRL_PC_5,
	}, {	/* ADF4360-9 */
		.vco_min = 65000000U,
		.vco_max = 400000000U,
		.default_cpl = ADF4360_GEN2_CTRL_PC_5,
	}
};

static int adf4360_write_reg(struct adf4360_state *st, unsigned int reg,
	unsigned int val)
{
	val |= reg;

	st->spi_data[0] = (val >> 16) & 0xff;
	st->spi_data[1] = (val >> 8) & 0xff;
	st->spi_data[2] = val & 0xff;

	return spi_write(st->spi, st->spi_data, 3);
}

static unsigned int adf4360_calc_prescaler(unsigned int pfd_freq,
		unsigned int n, unsigned int *out_p, unsigned int *out_a,
		unsigned int *out_b)
{
	unsigned int rate = pfd_freq * n;
	unsigned int p, a, b;

	/* Make sure divider counter input frequency is low enough */
	p = 8;
	while (p < 32 && rate / p > ADF4360_MAX_COUNTER_RATE)
		p *= 2;

	/*
	 * The range of dividers that can be produced using the dual-modulus
	 * pre-scaler is not continuous for values of n < p*(p-1). If we end up
	 * with a non supported divider value, pick the next closest one.
	 */
	a = n % p;
	b = n / p;

	if (b < 3) {
		b = 3;
		a = 0;
	} else if (a > b) {
		if (a - b < p - a) {
			a = b;
		} else {
			a = 0;
			b++;
		}
	}

	if (out_p)
		*out_p = p;
	if (out_a)
		*out_a = a;
	if (out_b)
		*out_b = b;

	return p * b + a;
}

static int adf4360_set_freq(struct adf4360_state *st, unsigned long freq)
{
	struct adf4360_platform_data *pdata = st->pdata;
	unsigned int ret, val_r, val_n, val_ctrl;
	unsigned int pfd_freq;
	unsigned long r, n;

	if (st->clkin_freq == 0)
		return -EINVAL;

	r = DIV_ROUND_CLOSEST(st->clkin_freq, pdata->pfd_freq);
	pfd_freq = st->clkin_freq / r;
	n = DIV_ROUND_CLOSEST(freq, pfd_freq);

	val_ctrl = ADF4360_CTRL_CPL(pdata->cpl);
	val_ctrl |= ADF4360_CTRL_CPI1(pdata->cpi);
	val_ctrl |= ADF4360_CTRL_CPI2(pdata->cpi);
	val_ctrl |= ADF4360_CTRL_PL(pdata->opl);

	if (!pdata->mtld)
		val_ctrl |= ADF4360_CTRL_MTLD;

	if (!pdata->pdp)
		val_ctrl |= ADF4360_CTRL_PDP;

	val_ctrl |= ADF4360_CTRL_MUXOUT(pdata->mux_out_ctrl);

	/* ADF4360-0 to ADF4360-7 have a dual-modulous prescaler */
	if (st->part_id <= 7) {
		unsigned int p, a, b;

		n = adf4360_calc_prescaler(pfd_freq, n, &p, &a, &b);

		switch (p) {
		case 8:
			val_ctrl |= ADF4360_CTRL_PRESCALER_8;
			break;
		case 16:
			val_ctrl |= ADF4360_CTRL_PRESCALER_16;
			break;
		default:
			val_ctrl |= ADF4360_CTRL_PRESCALER_32;
			break;
		}

		val_n = ADF4360_NDIV_A_COUNTER(a);
		val_n |= ADF4360_NDIV_B_COUNTER(b);

		if (freq < pdata->vco_min)
			val_n |= ADF4360_NDIV_PRESCALER_DIV2 |
				 ADF4360_NDIV_OUT_DIV2;
	} else {
		val_n = ADF4360_NDIV_B_COUNTER(n);
	}

	/*
	 * Always use BSC divider of 8, see Analog Devices AN-1347.
	 * http://www.analog.com/media/en/technical-documentation/application-notes/AN-1347.pdf
	 */
	val_r = ADF4360_RDIV_R_COUNTER(r) | ADF4360_RDIV_BSC_8;
	val_r |= ADF4360_RDIV_ABP(pdata->abp);

	dev_dbg(&st->spi->dev, "VCO: %lu Hz, PFD %u Hz\n", freq, pfd_freq);

	st->regs[ADF4360_REG_RDIV] = val_r;
	st->regs[ADF4360_REG_CTRL] = val_ctrl;
	st->regs[ADF4360_REG_NDIV] = val_n;

	ret = adf4360_sync_config(st, true);
	if (ret < 0)
		return ret;

	st->freq_req = freq;
	st->n = n;
	st->r = r;

	return 0;
}

static ssize_t adf4360_write(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len)
{
	struct adf4360_state *st = iio_priv(indio_dev);
	struct adf4360_platform_data *pdata = st->pdata;
	unsigned long readin;
	unsigned long tmp;
	int ret = 0;

	ret = kstrtoul(buf, 10, &readin);
	if (ret)
		return ret;

	switch ((u32)private) {
	case ADF4360_FREQ:
		if ((readin < pdata->vco_min) || (readin > pdata->vco_max))
			ret = -EINVAL;
		else
			ret = adf4360_set_freq(st, readin);
		break;
	case ADF4360_FREQ_REFIN:
		if ((readin > ADF4360_MAX_FREQ_REFIN) ||
			(readin < ADF4360_MIN_FREQ_REFIN)) {
			ret = -EINVAL;
			break;
		}

		if (st->clkin) {
			tmp = clk_round_rate(st->clkin, readin);
			if (tmp != readin) {
				ret = -EINVAL;
				break;
			}
			ret = clk_set_rate(st->clkin, tmp);
			if (ret < 0)
				break;
		}
		st->clkin_freq = readin;
		ret = adf4360_set_freq(st, st->freq_req);
		break;
	case ADF4360_PWRDOWN:
		if (readin)
			st->regs[ADF4360_REG_CTRL] |= ADF4360_CTRL_POWER_DOWN_EN;
		else
			st->regs[ADF4360_REG_CTRL] &= ~ADF4360_CTRL_POWER_DOWN_EN;

		adf4360_sync_config(st, true);
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t adf4360_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    char *buf)
{
	struct adf4360_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	switch ((u32)private) {
	case ADF4360_FREQ:
		val = (u64)(st->freq_req);
		/* PLL unlocked? return error */
		if (gpio_is_valid(st->pdata->gpio_lock_detect))
			if (!gpio_get_value(st->pdata->gpio_lock_detect)) {
				dev_dbg(&st->spi->dev, "PLL un-locked\n");
				ret = -EBUSY;
			}
		break;
	case ADF4360_FREQ_REFIN:
		if (st->clkin)
			st->clkin_freq = clk_get_rate(st->clkin);

		val = st->clkin_freq;
		break;
	case ADF4360_PWRDOWN:
		val = !!(st->regs[ADF4360_REG_CTRL] & ADF4360_CTRL_POWER_DOWN_EN);
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static void adf4360_m2k_setup(struct adf4360_state *st)
{
	struct adf4360_platform_data *pdata = st->pdata;
	unsigned int val_r, val_ctrl, val_b;

	st->n = 20;
	st->r = 4;

	val_ctrl = ADF4360_CTRL_CPL(pdata->cpl);
	val_ctrl |= ADF4360_CTRL_CPI1(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_CPI2(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_PL(pdata->opl);
	val_ctrl |= ADF4360_CTRL_PL(1);
	val_ctrl |= 5 << 5;
	val_ctrl |= 1 << 8;

	val_r = ADF4360_RDIV_R_COUNTER(st->r);
	val_r |= ADF4360_RDIV_BSC_8;
	val_b = ADF4360_NDIV_B_COUNTER(st->n) | (2<<2);

	adf4360_write_reg(st, ADF4360_REG_RDIV, val_r);
	adf4360_write_reg(st, ADF4360_REG_CTRL, val_ctrl);
	msleep(15);
	adf4360_write_reg(st, ADF4360_REG_NDIV, val_b);
}

/* fVCO = B * fREFIN / R */
static unsigned long adf4360_clk_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct adf4360_outputs *out = to_adf4360_outputs(hw);
	struct iio_dev *indio_dev = out->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);

	if (st->r == 0)
		return 0;

	/*
	 * The result is garuanteed to fit in 32-bit, but the intermediate
	 * result might require 64-bit.
	 */
	return DIV_ROUND_CLOSEST_ULL((uint64_t)parent_rate * st->n,
				     st->r);
}

static long adf4360_clk_round_rate(struct clk_hw *hw,
	unsigned long rate, unsigned long *parent_rate)
{
	struct adf4360_outputs *out = to_adf4360_outputs(hw);
	struct iio_dev *indio_dev = out->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);
	struct adf4360_platform_data *pdata = st->pdata;
	unsigned int r, n, pfd_freq;

	if (*parent_rate == 0)
		return 0;

	if (st->part_id == 9)
		return *parent_rate * st->n / st->r;

	if (rate > pdata->vco_max)
		return pdata->vco_max;

	/* ADF4360-0 to AD4370-7 have an optional by two divider */
	if (st->part_id <= 7) {
		if (rate < pdata->vco_min / 2)
			return pdata->vco_min / 2;
		if (rate < pdata->vco_min && rate > pdata->vco_max / 2) {
			if (pdata->vco_min - rate < rate - pdata->vco_max / 2)
				return pdata->vco_min;
			else
				return pdata->vco_max / 2;
		}
	} else {
		if (rate < pdata->vco_min)
			return pdata->vco_min;
	}

	r = DIV_ROUND_CLOSEST(*parent_rate, pdata->pfd_freq);
	pfd_freq = *parent_rate / r;
	n = DIV_ROUND_CLOSEST(rate, pfd_freq);

	if (st->part_id <= 7)
		n = adf4360_calc_prescaler(pfd_freq, n, NULL, NULL, NULL);

	return pfd_freq * n;
}

static int adf4360_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_name)
{
	struct adf4360_outputs *out = to_adf4360_outputs(hw);
	struct iio_dev *indio_dev = out->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);

	return adf4360_set_freq(st, rate);
}

static const struct clk_ops adf4360_clk_ops = {
	.recalc_rate = adf4360_clk_recalc_rate,
	.round_rate = adf4360_clk_round_rate,
	.set_rate = adf4360_clk_set_rate,
};

static void adf4360_clk_disable(void *data)
{
	struct adf4360_state *st = data;

	if (st->clkin)
		clk_disable_unprepare(st->clkin);
}

static void adf4360_clk_del_provider(void *data)
{
	struct adf4360_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static int adf4360_clk_register(struct iio_dev *indio_dev)
{
	struct adf4360_state *st = iio_priv(indio_dev);
	struct device_node *of_node = st->spi->dev.of_node;
	struct clk_init_data init;
	struct clk *clk_out;
	const char *parent_name, *clk_name;
	int ret;

	parent_name = of_clk_get_parent_name(of_node, 0);
	if (!parent_name)
		return -EINVAL;

	clk_name = st->spi->dev.of_node->name;
	of_property_read_string(st->spi->dev.of_node, "clock-output-names",
				&clk_name);

	init.name = clk_name;
	init.ops = &adf4360_clk_ops;
	init.flags = CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	st->outputs.hw.init = &init;
	st->outputs.indio_dev = indio_dev;

	clk_out = devm_clk_register(&st->spi->dev, &st->outputs.hw);
	if (IS_ERR(clk_out))
		return PTR_ERR(clk_out);

	ret = of_clk_add_provider(st->spi->dev.of_node,
				  of_clk_src_simple_get, clk_out);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&st->spi->dev,
					adf4360_clk_del_provider, st);
}

static void adf4360_regulator_disable(void *data)
{
	struct adf4360_state *st = data;

	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
}

static void adf4360_power_down(void *data)
{
	struct adf4360_state *st = data;

	if (!IS_ERR(st->spi)) {
		st->regs[ADF4360_REG_CTRL] |= ADF4360_CTRL_POWER_DOWN_EN;
		adf4360_sync_config(st, false);
	}
}

#define _ADF4360_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = adf4360_read, \
	.write = adf4360_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adf4360_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADF4360_EXT_INFO("frequency", ADF4360_FREQ),
	_ADF4360_EXT_INFO("refin_frequency", ADF4360_FREQ_REFIN),
	_ADF4360_EXT_INFO("powerdown", ADF4360_PWRDOWN),
	{ },
};

static const struct iio_chan_spec adf4360_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.ext_info = adf4360_ext_info,
};

static const struct iio_info adf4360_info = {
	.debugfs_reg_access = &adf4360_reg_access,
	.driver_module = THIS_MODULE,
};

#ifdef CONFIG_OF
static struct adf4360_platform_data *adf4360_parse_dt(struct device *dev,
						      unsigned int part_id)
{
	const struct adf4360_part_info *info = &adf4360_part_info[part_id];
	struct device_node *np = dev->of_node;
	struct adf4360_platform_data *pdata;
	unsigned int tmp;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	tmp = 0;
	of_property_read_u32(np, "adi,power-up-frequency", &tmp);
	pdata->power_up_frequency = tmp;

	ret = of_get_gpio(np, 0);
	if (ret < 0)
		pdata->gpio_lock_detect = -1;
	else
		pdata->gpio_lock_detect = ret;

	tmp = info->default_cpl;
	of_property_read_u32(np, "adi,core-power-level", &tmp);
	pdata->cpl = tmp;

	tmp = (part_id == 9) ? ADF4360_CPI_2_50 : ADF4360_CPI_1_25;
	of_property_read_u32(np, "adi,loop-filter-charge-pump-current", &tmp);
	pdata->cpi = tmp;

	tmp = (part_id == 9) ? ADF4360_CTRL_PL_5 : ADF4360_CTRL_PL_11;
	of_property_read_u32(np, "adi,output-power-level", &tmp);
	pdata->opl = tmp;

	pdata->mtld = of_property_read_bool(np, "adi,mute-till-lock-enable");
	pdata->pdp = of_property_read_bool(np, "adi,loop-filter-inverting");

	ret = of_property_read_u32(np, "adi,loop-filter-pfd-frequency-hz", &tmp);
	if (ret == 0)
		pdata->pfd_freq = tmp;

	tmp = ADF4360_CTRL_MUXOUT_LOCK_DETECT;
	of_property_read_u32(np, "adi,muxout-control", &tmp);
	pdata->mux_out_ctrl = tmp;

	/* rdiv_user_settings */
	tmp = ADF4360_RDIV_ABP_3_0NS;
	of_property_read_u32(np, "adi,antibacklash-pulse-width", &tmp);
	pdata->abp = tmp;

	if (part_id >= 7) {
		/*
		 * ADF4360-7 to ADF4360-9 have a VCO that is tuned to a specific
		 * range using an external inductor. These properties describe
		 * the range selected by the external inductor.
		 */
		ret = of_property_read_u32(np, "adi,vco-minimum-frequency-hz",
					   &tmp);
		if (ret == 0)
			pdata->vco_min = max(info->vco_min, tmp);
		else
			pdata->vco_min = info->vco_min;

		ret = of_property_read_u32(np, "adi,vco-maximum-frequency-hz",
					   &tmp);
		if (ret == 0)
			pdata->vco_max = min(info->vco_max, tmp);
		else
			pdata->vco_max = info->vco_max;
	} else {
		pdata->vco_min = info->vco_min;
		pdata->vco_max = info->vco_max;
	}

	return pdata;
}
#else
static
struct adf4360_platform_data *adf4360_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int adf4360_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct adf4360_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct adf4360_state *st;
	int ret;

	if (spi->dev.of_node) {
		pdata = adf4360_parse_dt(&spi->dev, id->driver_data);
		if (pdata == NULL)
			return -EINVAL;
	} else {
		pdata = spi->dev.platform_data;
	}

	if (!pdata) {
		dev_warn(&spi->dev, "no platform data? using default\n");
		pdata = &default_pdata;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = devm_add_action_or_reset(&spi->dev, adf4360_power_down, st);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4360_regulator_disable, st);
	if (ret)
		return ret;

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;
	st->part_id = id->driver_data;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = id->name;
	indio_dev->info = &adf4360_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adf4360_chan;
	indio_dev->num_channels = 1;

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return PTR_ERR(st->clkin);

	ret = clk_prepare_enable(st->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4360_clk_disable, st);
	if (ret)
		return ret;

	st->clkin_freq = clk_get_rate(st->clkin);

	memset(st->regs_hw, 0xFF, sizeof(st->regs_hw));

	if (gpio_is_valid(pdata->gpio_lock_detect)) {
		int i;
		ret = devm_gpio_request(&spi->dev, pdata->gpio_lock_detect,
					indio_dev->name);
		if (ret) {
			dev_err(&spi->dev, "fail to request lock detect GPIO-%d",
				pdata->gpio_lock_detect);
			return ret;
		}
		gpio_direction_input(pdata->gpio_lock_detect);

		/* ADF4360 synthesizers are write only devices, try to probe using gpio */
		for (i = 0; i < 4; i++) {
			st->regs[ADF4360_REG_CTRL] = ADF4360_CTRL_MUXOUT((i & 1) ?
				ADF4360_CTRL_MUXOUT_DVDD : ADF4360_CTRL_MUXOUT_GND);
			adf4360_sync_config(st, false);
			ret = gpio_get_value(st->pdata->gpio_lock_detect);
			if (ret != ((i & 1) ? 1 : 0)) {
				ret = -ENODEV;
				dev_err(&spi->dev, "Probe failed (muxout)");
				return ret;
			}
		}
	}

	/*
	 * Backwards compatibility for old M2K devicetrees, remove this
	 * eventually.
	 */
	if (st->part_id == 9)
		adf4360_m2k_setup(st);

	if (pdata->power_up_frequency) {
		ret = adf4360_set_freq(st, pdata->power_up_frequency);
		if (ret)
			return ret;
	}

	ret = adf4360_clk_register(indio_dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Clock provider register failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id adf4360_of_match[] = {
	{ .compatible = "adi,adf4360-0", },
	{ .compatible = "adi,adf4360-1", },
	{ .compatible = "adi,adf4360-2", },
	{ .compatible = "adi,adf4360-3", },
	{ .compatible = "adi,adf4360-4", },
	{ .compatible = "adi,adf4360-5", },
	{ .compatible = "adi,adf4360-6", },
	{ .compatible = "adi,adf4360-7", },
	{ .compatible = "adi,adf4360-8", },
	{ .compatible = "adi,adf4360-9", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, adf4360_of_match);

static const struct spi_device_id adf4360_id[] = {
	{"adf4360-0", 0},
	{"adf4360-1", 1},
	{"adf4360-2", 2},
	{"adf4360-3", 3},
	{"adf4360-4", 4},
	{"adf4360-5", 5},
	{"adf4360-6", 6},
	{"adf4360-7", 7},
	{"adf4360-8", 8},
	{"adf4360-9", 9},
	{}
};
MODULE_DEVICE_TABLE(spi, adf4360_id);

static struct spi_driver adf4360_driver = {
	.driver = {
		.name = "adf4360",
		.of_match_table = adf4360_of_match,
	},
	.probe = adf4360_probe,
	.id_table = adf4360_id,
};
module_spi_driver(adf4360_driver);

MODULE_AUTHOR("Edward Kigwana <ekigwana@gmail.com>");
MODULE_DESCRIPTION("Analog Devices ADF4360 PLL");
MODULE_LICENSE("GPL v2");
