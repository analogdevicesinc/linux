// SPDX-License-Identifier: GPL-2.0+
/*
 * ADF5610 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */


#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/iio/iio.h>

/* PLL Register Map */

#define ADF5610_REG_CHIP_ID	0x0
#define ADF5610_CHIP_ID		0xA7975

#define ADF5610_REG_RDIV	0x02
#define ADF5610_REG_INTG	0x03
#define ADF5610_REG_FRAC	0x04

#define ADF5610_REG_VCO_SPI	0x05
#define ADF5610_VCO_ID(x)	(((x) & 0x3) << 0)
#define ADF5610_VCO_REGADDR(x)	(((x) & 0xf) << 3)
#define ADF5610_VCO_DATA(x)	(((x) & 0x1ff) << 7)

#define ADF5610_REG_DS		0x06
#define ADF5610_SEED_SEL(x)	((x) & 0x3)
#define ADF5610_FRAC_BYPASS	BIT(7)
#define ADF5610_SD_ENABLE	BIT(11)
#define ADF5610_AUTO_CLK_CFG	BIT(21)

#define ADF5610_REG_LKD			0x07
#define ADF5610_LKD_WINCNT_MAX(x)	(((x) & 0x7) << 0)
#define ADF5610_LKD_ENABLE		BIT(3)
#define ADF5610_LKD_ONESHOT_SEL		BIT(6)
#define ADF5610_LKD_ONESHOT_DURATION(x)	(((x) & 0x7) << 7)
#define ADF5610_LKD_LKD_CONFIG(x)	(((x) & 0x3) << 10)
#define ADF5610_LKD_UNLOCKED_AUTOCAL	BIT(13)

#define ADF5610_REG_AN_EN	0x08
#define ADF5610_REFBUF_EN	BIT(3)
#define ADF5610_DIV_SEL		BIT(19)

#define ADF5610_REG_CP		0x09
#define ADF5610_CP_DOWN_MAG(x)	(((x) & 0x7f) << 0)
#define ADF5610_CP_UP_MAG(x)	(((x) & 0x7f) << 7)
#define ADF5610_CP_LEAK_MAG(x)	(((x) & 0x7f) << 14)
#define ADF5610_CP_LEAK_UP_EN	BIT(21)
#define ADF5610_CP_LEAK_DN_EN	BIT(22)
#define ADF5610_CP_HIK_EN	BIT(23)

#define ADF5610_REG_VCO_CAL	0x0A
#define ADF5610_VCO_CAL_RECOMM	0x2047

#define ADF5610_REG_GPO2	0x12
#define ADF5610_GPO_STATE	BIT(0)
#define ADF5610_LOCK_DETECT	BIT(1)

/* VCO Subsystem Register Map */

#define ADF5610_VCO_SYS_ID	0x0

#define ADF5610_VCO_REG_TUN	0x0
#define ADF5610_VCO_VT_SEL	BIT(0)
#define ADF5610_VCO_CAPS(x)	(((x) & 0x7f) << 1)

#define ADF5610_VCO_REG_PWR		0x1
#define ADF5610_RFOUT_PWR_CNTL(x)	(((x) & 0x3) << 0)
#define ADF5610_VCO_BIAS_CNTL(x)	(((x) & 0x7) << 2)
#define ADF5610_DIV_PWR_CNTL(x)		(((x) & 0x3) << 5)
#define ADF5610_DIV_EN(x)		BIT(8)

#define ADF5610_VCO_REG_OUT		0x2
#define ADF5610_DIFF_DIV_RATIO(x)	(((x) & 0x7) << 0)
#define ADF5610_PD_VCO			BIT(3)


#define ADF5610_RD		BIT(7)
#define ADF5610_WR		0

#define ADF5610_XREF_MAX_HZ	350000000

#define ADF5610_PD_MAX_HZ	100000000

#define ADF5610_RFOUT_MAX_HZ	14600000000
#define ADF5610_RFOUT_MIN_HZ	7300000000

#define ADF5610_VCO_MAX_MHZ	7300000000
#define ADF5610_VCO_MIN_MHZ	3650000000
#define ADF5610_VCO_DIV_LIMIT	4000000000ull

enum supported_parts {
	ADF5610,
};

enum serial_port_mode {
	LEGACY,
	OPEN,
};

enum ext_info {
	ADF5610_FREQUENCY,
};

struct adf5610 {
	struct spi_device	*spi;
	enum serial_port_mode	serial_port_mode;
	struct regmap		*regmap;
	struct clk		*xref;
	struct clk_hw		clk_hw;
	struct clock_scale	scale;
	u64			rf_out_freq;
	u64			div_out_freq;
	u8			div_out_ratio;
	u16			cp_down_mag;
	u16			cp_up_mag;
	u16			cp_leak_mag;
	bool			cp_leak_up_en;
	bool			cp_leak_down_en;
	bool			cp_hik_en;
	u64			vco_freq;
	u32			pd_freq;
	u32			r;
	u32			n_int;
	u32			n_frac;
};

#define to_adf_clk(_hw)	container_of(_hw, struct adf5610, clk_hw)

static int adf5610_pll_read(struct adf5610 *adf,
			    unsigned int reg, unsigned int *val)
{
	u8 tx[4];
	u8 rx[4];
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 4,
	};
	int ret;

	if (adf->serial_port_mode == LEGACY) {
		tx[0] = ADF5610_RD | ((reg & 0x1F) << 1);

		ret = spi_sync_transfer(adf->spi, &t, 1);

		*val = (rx[1] << 16) | (rx[2] << 8) | (rx[3] >> 0);
	} else {
		tx[2] = reg & 0x1F;

		ret = spi_sync_transfer(adf->spi, &t, 1);
		ret |= spi_sync_transfer(adf->spi, &t, 1);

		*val = (rx[0] << 17) | (rx[1] << 9) |
		       (rx[2] << 1) | (rx[3] >> 7);
	}

	return ret;
}

static int adf5610_pll_write(struct adf5610 *adf,
			     unsigned int reg, unsigned int val)
{
	u8 tx[4];
	u8 rx[4];
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 4,
	};

	if (adf->serial_port_mode == LEGACY) {
		tx[0] = ADF5610_WR | ((reg & 0x1F) << 1) |
			((val & 0x800000) >> 23);
		tx[1] = ((val & 0x7f8000) >> 15);
		tx[2] = ((val & 0x7f80) >> 7);
		tx[3] = ((val & 0x7f) << 1);
	} else {
		tx[0] = ((val & 0xff0000) >> 16);
		tx[1] = ((val & 0x00ff00) >> 8);
		tx[2] = ((val & 0x0000ff) >> 0);
		tx[3] = ((reg & 0x1F) << 3);
	}

	return spi_sync_transfer(adf->spi, &t, 1);
}

static int adf5610_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct adf5610 *adf = context;

	return adf5610_pll_write(adf, reg, val);
}

static int adf5610_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct adf5610 *adf = context;

	return adf5610_pll_read(adf, reg, val);
}

static const struct regmap_config adf5610_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,
	.reg_write = adf5610_reg_write,
	.reg_read = adf5610_reg_read,
	.max_register = 0x13,
};

static int adf5610_pll_update_bits(struct adf5610 *adf,
				   unsigned int reg,  unsigned int mask,
				   unsigned int val)
{
	unsigned int old;
	unsigned int new;
	int ret;

	ret = adf5610_pll_read(adf, reg, &old);
	if (ret != 0)
		return ret;

	new = old & ~mask; 
    	new |= val & mask;

	if (new != old)
		return adf5610_pll_write(adf, reg, new);

	return 0;
}

static int adf5610_vco_write(struct adf5610 *adf,
			     unsigned int reg, unsigned int val)
{
	unsigned int pll_val;

	pll_val = ADF5610_VCO_DATA(val) | ADF5610_VCO_REGADDR(reg) |
			ADF5610_VCO_ID(ADF5610_VCO_SYS_ID);

	return adf5610_pll_write(adf, ADF5610_REG_VCO_SPI, pll_val);
}


static int adf5610_setup(struct adf5610 *adf)
{
	bool frac_bypass = 0;
	u64 prescaled_vco;
	bool div_sel = 0;
	bool ref_buf = 0;
	u32 xref_freq;
	u64 n;

	xref_freq = clk_get_rate(adf->xref);

	if (xref_freq > ADF5610_XREF_MAX_HZ) {
		dev_err(&adf->spi->dev, "XREF out of range (%u > %u)",
			xref_freq, ADF5610_XREF_MAX_HZ);
		return -EINVAL;
	}

	adf->r = 1;
	adf->pd_freq = xref_freq;
	while (adf->pd_freq > ADF5610_PD_MAX_HZ) {
		adf->r++;
		adf->pd_freq = xref_freq / adf->r;
	}

	adf->div_out_ratio = 7;

	adf->rf_out_freq = adf->div_out_freq;

	while (adf->rf_out_freq < ADF5610_RFOUT_MIN_HZ) {
		adf->rf_out_freq *= 2;
		adf->div_out_ratio--;
	}

	adf->vco_freq = adf->rf_out_freq / 2;
	prescaled_vco = adf->vco_freq;

	if (adf->vco_freq > ADF5610_VCO_DIV_LIMIT) {
		div_sel = 1;
		prescaled_vco /= 2;
	}

	/* 24-bit fractional modulus */
	n = (prescaled_vco << 24) + (adf->pd_freq / 2);
	do_div(n, adf->pd_freq);
	adf->n_frac = n & 0xffffff;
	adf->n_int = n >> 24;

	adf5610_pll_write(adf, ADF5610_REG_RDIV, adf->r);
	if (adf->r > 1)
		/* Using the divider requires the reference path buffer
		   to be enabled */
		ref_buf = 1;

	ref_buf = 1; // TODO

	if (adf->n_frac)
		adf5610_pll_write(adf, ADF5610_REG_INTG, adf->n_int);
	else {
		adf5610_pll_write(adf, ADF5610_REG_FRAC, adf->n_frac);
		frac_bypass = 1;
	}

	adf5610_pll_update_bits(adf, ADF5610_REG_DS,
				ADF5610_FRAC_BYPASS | ADF5610_SD_ENABLE | ADF5610_AUTO_CLK_CFG,
				(frac_bypass ? ADF5610_FRAC_BYPASS : ADF5610_SD_ENABLE) | 0);

	adf5610_pll_write(adf, ADF5610_REG_LKD,
			ADF5610_LKD_WINCNT_MAX(5) |
			ADF5610_LKD_ENABLE |
			ADF5610_LKD_ONESHOT_SEL |
			ADF5610_LKD_ONESHOT_DURATION(2)); // TODO

	adf5610_pll_update_bits(adf, ADF5610_REG_AN_EN,
				ADF5610_REFBUF_EN | ADF5610_DIV_SEL,
				(ref_buf ? ADF5610_REFBUF_EN : 0) |
				(div_sel ? ADF5610_DIV_SEL : 0));

	adf5610_pll_write(adf, ADF5610_REG_VCO_CAL, ADF5610_VCO_CAL_RECOMM);


	adf5610_pll_write(adf, ADF5610_REG_CP,
		ADF5610_CP_DOWN_MAG(adf->cp_down_mag) | 
		ADF5610_CP_UP_MAG(adf->cp_up_mag) |
		ADF5610_CP_LEAK_MAG(adf->cp_leak_mag) |
		(adf->cp_leak_up_en ? ADF5610_CP_LEAK_UP_EN : 0) |
		(adf->cp_leak_down_en ? ADF5610_CP_LEAK_DN_EN : 0) |
		(adf->cp_hik_en ? ADF5610_CP_HIK_EN : 0));

	adf5610_vco_write(adf, ADF5610_VCO_REG_PWR,
		ADF5610_DIV_EN(1) | ADF5610_VCO_BIAS_CNTL(7) | ADF5610_DIV_PWR_CNTL(3));

	adf5610_vco_write(adf, ADF5610_VCO_REG_OUT,
		ADF5610_DIFF_DIV_RATIO(adf->div_out_ratio));

	/* Trigger the FSM - force ADF5610 to relock whether in integer mode
	   or fractional mode */
	adf5610_vco_write(adf, ADF5610_VCO_REG_TUN, 0x0);

	if (adf->n_frac)
		adf5610_pll_write(adf, ADF5610_REG_FRAC, adf->n_frac);
	else
		adf5610_pll_write(adf, ADF5610_REG_INTG, adf->n_int);

	return 0;
}

static unsigned long adf5610_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct adf5610 *adf = to_adf_clk(clk_hw);

	return to_ccf_scaled(adf->div_out_freq, &adf->scale);
}

static long adf5610_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	return rate;
}

static int adf5610_set_rate(struct clk_hw *clk_hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct adf5610 *adf = to_adf_clk(clk_hw);

	adf->div_out_freq = from_ccf_scaled(rate,  &adf->scale);

	return adf5610_setup(adf);
}

static int adf5610_debugfs_show(struct seq_file *s, void *p)
{
	struct adf5610 *adf = s->private;
	unsigned int status;

	adf5610_pll_read(adf, ADF5610_REG_GPO2, &status);

	seq_printf(s, "Lock detect status: %s\n", (status & ADF5610_LOCK_DETECT) ?
			"locked" : "unlocked");
	seq_printf(s, "PFD: %u Hz\n", adf->pd_freq);
	seq_printf(s, "VCO: %llu Hz\n", adf->vco_freq);
	seq_printf(s, "RF_OUT: %llu Hz\n", adf->rf_out_freq);
	seq_printf(s, "DIV_OUT: %llu Hz\n", adf->div_out_freq);
	seq_printf(s, "DIV_OUT ratio: %d\n", adf->div_out_ratio);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adf5610_debugfs);

static void adf5610_debug_init(struct clk_hw *clk_hw, struct dentry *dentry)
{
	struct adf5610 *adf = to_adf_clk(clk_hw);

	debugfs_create_file("status", 0444, dentry, adf,
				&adf5610_debugfs_fops);
}

static const struct clk_ops adf5610_clk_ops = {
	.recalc_rate = adf5610_recalc_rate,
	.round_rate = adf5610_round_rate,
	.set_rate = adf5610_set_rate,
	.debug_init = adf5610_debug_init,
};

static int adf5610_parse_dt(struct device *dev,
			    struct adf5610 *adf)
{
	if (device_property_read_bool(dev, "adi,legacy-serial-port-mode"))
		adf->serial_port_mode = LEGACY;
	else
		adf->serial_port_mode = OPEN;

	device_property_read_u64(dev, "adi,power-up-div-out-frequency-hz",
				 &adf->div_out_freq);

	device_property_read_u16(dev, "adi,charge-pump-down-gain-ua",
				 &adf->cp_down_mag);
	if (adf->cp_down_mag)
		adf->cp_down_mag /= 20;

	device_property_read_u16(dev, "adi,charge-pump-up-gain-ua",
				 &adf->cp_up_mag);
	if (adf->cp_up_mag)
		adf->cp_up_mag /= 20;

	device_property_read_u16(dev, "adi,charge-pump-offset-ua",
				 &adf->cp_leak_mag);
	if (adf->cp_leak_mag)
		adf->cp_leak_mag /= 5;

	adf->cp_leak_up_en = device_property_read_bool(dev,
				"adi,charge-pump-offset-up-enable");
	adf->cp_leak_down_en = device_property_read_bool(dev,
				"adi,charge-pump-offset-down-enable");
	adf->cp_hik_en = device_property_read_bool(dev,
				"adi,charge-pump-high-current-enable");

	return 0;
}

static int adf5610_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int tx_val,
			      unsigned int *rx_val)
{
	struct adf5610 *adf = iio_priv(indio_dev);

	if (rx_val)
		regmap_read(adf->regmap, reg, rx_val);
	else
		regmap_write(adf->regmap, reg, tx_val);

	return 0;
}

static ssize_t adf5610_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    char *buf)
{
	struct adf5610 *adf = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	switch ((u32)private) {
	case ADF5610_FREQUENCY:
		val = adf->div_out_freq;
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t adf5610_write(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len)
{
	struct adf5610 *adf = iio_priv(indio_dev);
	unsigned long long val;
	int ret;

	switch ((u32)private) {
	case ADF5610_FREQUENCY:
		ret = kstrtoull(buf, 10, &val);
		if (ret)
			break;
		adf->div_out_freq = val;
		adf5610_setup(adf);
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info adf5610_frequency[] = {
	{
		.name = "frequency",
		.read = adf5610_read,
		.write = adf5610_write,
		.private = ADF5610_FREQUENCY,
		.shared = IIO_SEPARATE,
	},
	{ },
};

#define ADF5610_CHANNEL(chan) {					\
	.type = IIO_ALTVOLTAGE,					\
	.indexed = 1,						\
	.channel = (chan),					\
	.ext_info = adf5610_frequency,				\
}

static const struct iio_chan_spec adf5610_channels[] = {
	ADF5610_CHANNEL(0),
};

static const struct iio_info adf5610_info = {
	.debugfs_reg_access = &adf5610_reg_access,
};

static int adf5610_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct clk_init_data init;
	struct regmap *regmap;
	unsigned int chip_id;
	struct adf5610 *adf;
	struct clk *clk;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adf));
	if (!indio_dev)
		return -ENOMEM;

	adf = iio_priv(indio_dev);

	spi_set_drvdata(spi, indio_dev);

	adf->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &adf5610_info;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->channels = adf5610_channels;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	regmap = devm_regmap_init(&spi->dev, NULL, adf, &adf5610_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	adf->regmap = regmap;

	adf5610_pll_read(adf, ADF5610_REG_CHIP_ID, &chip_id);
	if (chip_id != ADF5610_CHIP_ID)
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", chip_id);
	else
		dev_err(&spi->dev, "Recognized CHIP_ID 0x%X\n", chip_id);

	adf->xref = devm_clk_get(&spi->dev, "xref");
	if (IS_ERR(adf->xref))
		return PTR_ERR(adf->xref);

	ret = clk_prepare_enable(adf->xref);
	if (ret < 0)
		return ret;

	ret = adf5610_parse_dt(&spi->dev, adf);
	if (ret < 0)
		return ret;


	init.name = spi->dev.of_node->name;
	init.ops = &adf5610_clk_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;

	adf->clk_hw.init = &init;

	clk = devm_clk_register(&spi->dev, &adf->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	adf->scale.mult = 1;
	adf->scale.div = 10;

	adf5610_setup(adf);

	dev_info(&spi->dev, "Probed\n");

	return of_clk_add_provider(spi->dev.of_node,
			of_clk_src_simple_get, clk);
}

static const struct spi_device_id adf5610_id[] = {
	{ "adf5610", ADF5610 },
	{}
};
MODULE_DEVICE_TABLE(spi, adf5610_id);

static const struct of_device_id adf5610_of_match[] = {
	{ .compatible = "adi,adf5610" },
	{},
};
MODULE_DEVICE_TABLE(of, adf5610_of_match);

static struct spi_driver adf5610_driver = {
	.driver = {
			.name = "adf5610",
			.of_match_table = of_match_ptr(adf5610_of_match),
		},
	.probe = adf5610_probe,
	.id_table = adf5610_id,
};
module_spi_driver(adf5610_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF5610");
MODULE_LICENSE("GPL v2");