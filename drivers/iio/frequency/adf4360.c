/*
 * ADF4360 PLL with Integrated Synthesizer and VCO
 *
 * Copyright 2014-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

/* Registers */
#define ADF4360_REG_CTRL		0x00
#define ADF4360_REG_RDIV		0x01
#define ADF4360_REG_NDIV		0x02

/* ADF4360_CTRL */
#define ADF4360_GEN1_CTRL_PC_5		(0x0 << 2)
#define ADF4360_GEN1_CTRL_PC_10		(0x1 << 2)
#define ADF4360_GEN1_CTRL_PC_15		(0x2 << 2)
#define ADF4360_GEN1_CTRL_PC_20		(0x3 << 2)

#define ADF4360_GEN2_CTRL_PC_2_5	(0x0 << 2)
#define ADF4360_GEN2_CTRL_PC_5		(0x1 << 2)
#define ADF4360_GEN2_CTRL_PC_7_5	(0x2 << 2)
#define ADF4360_GEN2_CTRL_PC_10		(0x3 << 2)

#define ADF4360_CTRL_COUNTER_RESET	BIT(4)
#define ADF4360_CTRL_PDP		BIT(8)
#define ADF4360_CTRL_MTLD		BIT(11)
#define ADF4360_CTRL_PL_3_5		(0x0 << 12)
#define ADF4360_CTRL_PL_5		(0x1 << 12)
#define ADF4360_CTRL_PL_7_5		(0x2 << 12)
#define ADF4360_CTRL_PL_11		(0x3 << 12)
#define ADF4360_CTRL_CPI1(x)		((x) << 14)
#define ADF4360_CTRL_CPI2(x)		((x) << 17)
#define ADF4360_CTRL_PRESCALER_8	(0 << 22)
#define ADF4360_CTRL_PRESCALER_16	(1 << 22)
#define ADF4360_CTRL_PRESCALER_32	(2 << 22)

#define ADF4360_CTRL_MUXOUT_THREE_STATE	(0x0 << 5)
#define ADF4360_CTRL_MUXOUT_LOCK_DETECT	(0x1 << 5)
#define ADF4360_CTRL_MUXOUT_NDIV	(0x2 << 5)
#define ADF4360_CTRL_MUXOUT_DVDD	(0x3 << 5)
#define ADF4360_CTRL_MUXOUT_RDIV	(0x4 << 5)
#define ADF4360_CTRL_MUXOUT_OD_LD	(0x5 << 5)
#define ADF4360_CTRL_MUXOUT_SDO		(0x6 << 5)
#define ADF4360_CTRL_MUXOUT_GND		(0x7 << 5)

#define ADF4360_CPI_0_31		0
#define ADF4360_CPI_0_62		1
#define ADF4360_CPI_0_93		2
#define ADF4360_CPI_1_25		3
#define ADF4360_CPI_1_56		4
#define ADF4360_CPI_1_87		5
#define ADF4360_CPI_2_18		6
#define ADF4360_CPI_2_50		7

/* ADF4360_NDIV */
#define ADF4360_NDIV_A_COUNTER(x)	((x) << 2)
#define ADF4360_NDIV_B_COUNTER(x)	((x) << 8)
#define ADF4360_NDIV_OUT_DIV2		BIT(22)
#define ADF4360_NDIV_PRESCALER_DIV2	BIT(23)

/* ADF4360_RDIV */
#define ADF4360_RDIV_R_COUNTER(x)	((x) << 2)
#define ADF4360_RDIV_ABP_3_0NS		(0x0 << 16)
#define ADF4360_RDIV_ABP_1_3NS		(0x1 << 16)
#define ADF4360_RDIV_ABP_6_0NS		(0x2 << 16)
#define ADF4360_RDIV_BSC_1		(0x0 << 20)
#define ADF4360_RDIV_BSC_2		(0x1 << 20)
#define ADF4360_RDIV_BSC_4		(0x2 << 20)
#define ADF4360_RDIV_BSC_8		(0x3 << 20)

/* Specifications */
#define ADF4360_MAX_PFD_RATE		8000000 /* 8 MHz */
#define ADF4360_MAX_COUNTER_RATE	300000000 /* 300 MHz */

struct adf4360_output {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
};

#define to_output(_hw) container_of(_hw, struct adf4360_output, hw)

struct adf4360_chip_info {
	unsigned int vco_min;
	unsigned int vco_max;
	unsigned int default_cpl;
};

struct adf4360_state {
	struct spi_device *spi;
	const struct adf4360_chip_info *info;
	struct adf4360_output output;
	struct clk *clkin;
	unsigned int part_id;
	unsigned long clkin_freq;
	unsigned long r;
	unsigned long n;
	unsigned int vco_min;
	unsigned int vco_max;
	unsigned int pfd_freq;
	unsigned int cpi;
	bool pdp;
	const char *clk_out_name;
	u8 spi_data[3] ____cacheline_aligned;
};

static const struct adf4360_chip_info adf4360_chip_info_tbl[] = {
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

	return spi_write(st->spi, st->spi_data, ARRAY_SIZE(st->spi_data));
}

/* fVCO = B * fREFIN / R */

static unsigned long adf4360_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct iio_dev *indio_dev = to_output(hw)->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);

	if (st->r == 0)
		return 0;

	/*
	 * The result is guaranteed to fit in 32-bit, but the intermediate
	 * result might require 64-bit.
	 */
	return DIV_ROUND_CLOSEST_ULL((uint64_t)parent_rate * st->n, st->r);
}

static unsigned int adf4360_calc_prescaler(unsigned int pfd_freq,
					   unsigned int n,
					   unsigned int *out_p,
					   unsigned int *out_a,
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

static long adf4360_clk_round_rate(struct clk_hw *hw,
				   unsigned long rate,
				   unsigned long *parent_rate)
{
	struct iio_dev *indio_dev = to_output(hw)->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);
	unsigned int r, n;
	unsigned int pfd_freq;

	if (*parent_rate == 0)
		return 0;

	if (st->part_id == 9)
		return *parent_rate * st->n / st->r;

	if (rate > st->vco_max)
		return st->vco_max;

	/* ADF4360-0 to AD4370-7 have an optional by two divider */
	if (st->part_id <= 7) {
		if (rate < st->vco_min / 2)
			return st->vco_min / 2;
		if (rate < st->vco_min && rate > st->vco_max / 2) {
			if (st->vco_min - rate < rate - st->vco_max / 2)
				return st->vco_min;
			else
				return st->vco_max / 2;
		}
	} else {
		if (rate < st->vco_min)
			return st->vco_min;
	}

	r = DIV_ROUND_CLOSEST(*parent_rate, st->pfd_freq);
	pfd_freq = *parent_rate / r;
	n = DIV_ROUND_CLOSEST(rate, pfd_freq);

	if (st->part_id <= 7)
		n = adf4360_calc_prescaler(pfd_freq, n, NULL, NULL, NULL);

	return pfd_freq * n;
}

static int adf4360_clk_set_rate(struct clk_hw *hw,
				unsigned long rate,
				unsigned long parent_rate)
{
	struct iio_dev *indio_dev = to_output(hw)->indio_dev;
	struct adf4360_state *st = iio_priv(indio_dev);
	unsigned int val_r, val_n, val_ctrl;
	unsigned int pfd_freq;
	unsigned long r, n;

	if (parent_rate == 0)
		return -EINVAL;

	r = DIV_ROUND_CLOSEST(parent_rate, st->pfd_freq);
	pfd_freq = parent_rate / r;
	n = DIV_ROUND_CLOSEST(rate, pfd_freq);

	val_ctrl = st->info->default_cpl;
	val_ctrl |= ADF4360_CTRL_CPI1(st->cpi);
	val_ctrl |= ADF4360_CTRL_CPI2(st->cpi);
	val_ctrl |= ADF4360_CTRL_PL_11;
	val_ctrl |= ADF4360_CTRL_MTLD;

	if (!st->pdp)
		val_ctrl |= ADF4360_CTRL_PDP;
	val_ctrl |= ADF4360_CTRL_MUXOUT_LOCK_DETECT;

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

		if (rate < st->vco_min)
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

	adf4360_write_reg(st, ADF4360_REG_RDIV, val_r);
	adf4360_write_reg(st, ADF4360_REG_CTRL, val_ctrl);
	usleep_range(15000, 20000);
	adf4360_write_reg(st, ADF4360_REG_NDIV, val_n);

	st->n = n;
	st->r = r;

	return 0;
}

static const struct clk_ops adf4360_clk_ops = {
	.recalc_rate = adf4360_clk_recalc_rate,
	.round_rate = adf4360_clk_round_rate,
	.set_rate = adf4360_clk_set_rate,
};

static void adf4360_m2k_setup(struct adf4360_state *st)
{
	unsigned int val_r, val_ctrl, val_b;

	st->n = 20;
	st->r = 4;

	val_ctrl = ADF4360_GEN2_CTRL_PC_5;
	val_ctrl |= ADF4360_CTRL_CPI1(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_CPI2(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_PL_5;
	val_ctrl |= 5 << 5;
	val_ctrl |= 1 << 8;
//	val_ctrl |= BIT(11);
//	val_ctrl |= BIT(20);

	val_r = ADF4360_RDIV_R_COUNTER(st->r);
	val_r |= ADF4360_RDIV_BSC_8;
	val_b = ADF4360_NDIV_B_COUNTER(st->n) | (2<<2);

	adf4360_write_reg(st, ADF4360_REG_RDIV, val_r);
	adf4360_write_reg(st, ADF4360_REG_CTRL, val_ctrl);
	msleep(15);
	adf4360_write_reg(st, ADF4360_REG_NDIV, val_b);
}

static const struct iio_chan_spec adf4360_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
};

static const struct iio_info adf4360_iio_info = {
	.driver_module = THIS_MODULE,
};

static void adf4360_clkin_disable(void *data)
{
	struct adf4360_state *st = data;

	clk_disable_unprepare(st->clkin);
}

static int adf4360_get_clkin(struct adf4360_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "clkin");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, adf4360_clkin_disable, st);
	if (ret)
		return ret;

	st->clkin = clk;
	st->clkin_freq = clk_get_rate(clk);

	return 0;
}

static void adf4360_clk_del_provider(void *data)
{
	struct adf4360_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static int adf4360_clk_register(struct adf4360_state *st)
{
	struct spi_device *spi = st->spi;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_name;
	int ret;

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.name = st->clk_out_name;
	init.ops = &adf4360_clk_ops;
	init.flags = CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	st->output.hw.init = &init;

	clk = devm_clk_register(&spi->dev, &st->output.hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&spi->dev, adf4360_clk_del_provider, st);
}

static int adf4360_parse_dt(struct adf4360_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	ret = device_property_read_string(dev, "clock-output-names",
					  &st->clk_out_name);
	if ((ret < 0) && dev->of_node)
		st->clk_out_name = dev->of_node->name;

	if (st->part_id >= 7) {
		/*
		 * ADF4360-7 to ADF4360-9 have a VCO that is tuned to a specific
		 * range using an external inductor. These properties describe
		 * the range selected by the external inductor.
		 */
		ret = device_property_read_u32(dev,
					       "adi,vco-minimum-frequency-hz",
					       &tmp);
		if (ret == 0)
			st->vco_min = max(st->info->vco_min, tmp);
		else
			st->vco_min = st->info->vco_min;

		ret = device_property_read_u32(dev,
					       "adi,vco-maximum-frequency-hz",
					       &tmp);
		if (ret == 0)
			st->vco_max = min(st->info->vco_max, tmp);
		else
			st->vco_max = st->info->vco_max;
	} else {
		st->vco_min = st->info->vco_min;
		st->vco_max = st->info->vco_max;
	}

	st->pdp = device_property_read_bool(dev, "adi,loop-filter-inverting");

	ret = device_property_read_u32(dev,
				       "adi,loop-filter-pfd-frequency-hz",
				       &tmp);
	if (ret == 0) {
		st->pfd_freq = tmp;
	} else {
		dev_err(dev, "PFD frequency property missing\n");
		return ret;
	}

	ret = device_property_read_u32(dev,
				       "adi,loop-filter-charge-pump-current",
				       &tmp);
	if (ret == 0) {
		st->cpi = tmp;
	} else {
		dev_err(dev, "CPI property missing\n");
		return ret;
	}

	return 0;
}

static int adf4360_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct adf4360_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	st->info = &adf4360_chip_info_tbl[id->driver_data];
	st->part_id = id->driver_data;

	ret = adf4360_parse_dt(st);
	if (ret) {
		dev_err(&spi->dev, "Parsing properties failed (%d)\n", ret);
		return -ENODEV;
	}

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	indio_dev->info = &adf4360_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adf4360_chan;
	indio_dev->num_channels = 1;
	st->output.indio_dev = indio_dev;

	/*
	 * Backwards compatibility for old M2K devicetrees, remove this
	 * eventually.
	 */
	if (id->driver_data == 9)
		adf4360_m2k_setup(st);

	ret = adf4360_get_clkin(st);
	if (ret)
		return ret;
	ret = adf4360_clk_register(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

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

static struct spi_driver adf4360_driver = {
	.driver = {
		.name	= "adf4360",
		.owner	= THIS_MODULE,
	},
	.probe		= adf4360_probe,
	.id_table	= adf4360_id,
};
module_spi_driver(adf4360_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Analog Devices ADF4360 PLL");
