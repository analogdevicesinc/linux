// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Skyworks Si5391A
 * Copyright (C) 2025
 *
 * The Si5391A is a 10-output clock generator with integrated PLL.
 * It's similar to the Si5341 but with a simpler architecture.
 * This driver is based on the clk-si5341.c driver structure.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gcd.h>
#include <linux/math64.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#define SI5391A_NUM_INPUTS	4
#define SI5391A_MAX_NUM_OUTPUTS	10

/* Input clock names */
static const char * const si5391a_input_clock_names[] = {
	"in0", "in1", "in2", "xtal"
};

/* The chip can get its input clock from 3 input pins or an XTAL */
/* There is one PLL running at 13500–14256 MHz similar to SI5341 */
#define SI5391A_PLL_VCO_MIN	13500000000ull
#define SI5391A_PLL_VCO_MAX	14256000000ull

/* Output stages */
struct clk_si5391a_output {
	struct clk_hw hw;
	struct clk_si5391a *data;
	struct regulator *vddo_reg;
	u8 index;
};
#define to_clk_si5391a_output(_hw) \
	container_of(_hw, struct clk_si5391a_output, hw)

struct clk_si5391a {
	struct clk_hw hw;
	struct regmap *regmap;
	struct i2c_client *i2c_client;
	struct clk_si5391a_output clk[SI5391A_MAX_NUM_OUTPUTS];
	struct clk *input_clk[SI5391A_NUM_INPUTS];
	const char *input_clk_name[SI5391A_NUM_INPUTS];
	const u16 *reg_output_offset;
	const u16 *reg_rdiv_offset;
	u64 freq_vco; /* 13500–14256 MHz */
	u8 num_outputs;
	u16 chip_id;
	bool xaxb_ext_clk;
	bool iovdd_33;
};
#define to_clk_si5391a(_hw)	container_of(_hw, struct clk_si5391a, hw)

struct clk_si5391a_output_config {
	u8 out_format_drv_bits;
	u8 out_cm_ampl_bits;
	u8 vdd_sel_bits;
	bool always_on;
};

/* Register definitions */
#define SI5391A_PAGE		0x0001
#define SI5391A_PN_BASE		0x0002
#define SI5391A_DEVICE_REV	0x0005
#define SI5391A_STATUS		0x000C
#define SI5391A_LOS		0x000D
#define SI5391A_STATUS_STICKY	0x0011
#define SI5391A_LOS_STICKY	0x0012
#define SI5391A_SOFT_RST	0x001C
#define SI5391A_IN_SEL		0x0021
#define SI5391A_DEVICE_READY	0x00FE
#define SI5391A_XAXB_CFG	0x090E
#define SI5391A_IO_VDD_SEL	0x0943
#define SI5391A_IN_EN		0x0949
#define SI5391A_INX_TO_PFD_EN	0x094A

/* Status bits */
#define SI5391A_STATUS_SYSINCAL	BIT(0)
#define SI5391A_STATUS_LOSXAXB	BIT(1)
#define SI5391A_STATUS_LOSREF	BIT(2)
#define SI5391A_STATUS_LOL	BIT(3)

/* Input selection */
#define SI5391A_IN_SEL_MASK	0x06
#define SI5391A_IN_SEL_SHIFT	1
#define SI5391A_IN_SEL_REGCTRL	0x01
#define SI5391A_INX_TO_PFD_SHIFT	4

/* XTAL config bits */
#define SI5391A_XAXB_CFG_EXTCLK_EN	BIT(0)
#define SI5391A_XAXB_CFG_PDNB		BIT(1)

/* Input dividers (48-bit) */
#define SI5391A_IN_PDIV(x)	(0x0208 + ((x) * 10))
#define SI5391A_IN_PSET(x)	(0x020E + ((x) * 10))
#define SI5391A_PX_UPD		0x0230

/* PLL configuration */
#define SI5391A_PLL_M_NUM	0x0235
#define SI5391A_PLL_M_DEN	0x023B

/* Output configuration */
#define SI5391A_OUT_CONFIG(output)	\
			((output)->data->reg_output_offset[(output)->index])
#define SI5391A_OUT_FORMAT(output)	(SI5391A_OUT_CONFIG(output) + 1)
#define SI5391A_OUT_CM(output)		(SI5391A_OUT_CONFIG(output) + 2)
#define SI5391A_OUT_MUX_SEL(output)	(SI5391A_OUT_CONFIG(output) + 3)
#define SI5391A_OUT_R_REG(output)	\
			((output)->data->reg_rdiv_offset[(output)->index])

#define SI5391A_OUT_MUX_VDD_SEL_MASK 0x38

/* Output R divider */
#define SI5391A_OUT_R_DIV(output)	(SI5391A_OUT_R_REG(output))

/* Output enable/disable */
#define SI5391A_OUT_EN		0x0A03

/* SI5391A_OUT_CONFIG bits */
#define SI5391A_OUT_CFG_PDN		BIT(0)
#define SI5391A_OUT_CFG_OE		BIT(1)
#define SI5391A_OUT_CFG_RDIV_FORCE2	BIT(2)

#define SI5391A_REGISTER_MAX	0xBFF

/* Output configuration registers 0..9 */
static const u16 si5391a_reg_output_offset[] = {
	0x0108,	/* OUT0 */
	0x010D,	/* OUT1 */
	0x0112,	/* OUT2 */
	0x0117,	/* OUT3 */
	0x011C,	/* OUT4 */
	0x0121,	/* OUT5 */
	0x0126,	/* OUT6 */
	0x012B,	/* OUT7 */
	0x0130,	/* OUT8 */
	0x013A,	/* OUT9 */
};

/* The location of the R divider registers for each output */
static const u16 si5391a_reg_rdiv_offset[] = {
	0x024A,	/* OUT0 */
	0x024D,	/* OUT1 */
	0x0250,	/* OUT2 */
	0x0253,	/* OUT3 */
	0x0256,	/* OUT4 */
	0x0259,	/* OUT5 */
	0x025C,	/* OUT6 */
	0x025F,	/* OUT7 */
	0x0262,	/* OUT8 */
	0x0268,	/* OUT9 */
};

/*
 * Register access functions
 */
static int si5391a_reg_read(struct clk_si5391a *data, u16 reg, u8 *val)
{
	int err;
	unsigned int tmp;
	u8 reg_high = reg >> 8;

	/* Set page */
	err = regmap_write(data->regmap, SI5391A_PAGE, reg_high);
	if (err < 0)
		return err;

	err = regmap_read(data->regmap, reg & 0xff, &tmp);
	*val = tmp;

	return err;
}

static int si5391a_reg_write(struct clk_si5391a *data, u16 reg, u8 val)
{
	int err;
	u8 reg_high = reg >> 8;

	/* Set page */
	err = regmap_write(data->regmap, SI5391A_PAGE, reg_high);
	if (err < 0)
		return err;

	return regmap_write(data->regmap, reg & 0xff, val);
}

static int si5391a_bulk_read(struct clk_si5391a *data, u16 reg,
			      u8 *buf, unsigned int count)
{
	int err;
	u8 reg_high = reg >> 8;

	/* Set page */
	err = regmap_write(data->regmap, SI5391A_PAGE, reg_high);
	if (err < 0)
		return err;

	return regmap_bulk_read(data->regmap, reg & 0xff, buf, count);
}

static int si5391a_bulk_write(struct clk_si5391a *data, u16 reg,
			       const u8 *buf, unsigned int count)
{
	int err;
	u8 reg_high = reg >> 8;

	/* Set page */
	err = regmap_write(data->regmap, SI5391A_PAGE, reg_high);
	if (err < 0)
		return err;

	return regmap_bulk_write(data->regmap, reg & 0xff, buf, count);
}

/*
 * Output clock functions
 */
static int si5391a_output_clk_is_on(struct clk_hw *hw)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	int err;
	u8 val;

	err = si5391a_reg_read(output->data, SI5391A_OUT_CONFIG(output), &val);
	if (err < 0)
		return err;

	/* Check OE and PDN bits */
	return (val & SI5391A_OUT_CFG_OE) && !(val & SI5391A_OUT_CFG_PDN);
}

static int si5391a_output_clk_prepare(struct clk_hw *hw)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	int err;
	u8 val;

	/* Clear PDN bit, set OE bit */
	err = si5391a_reg_read(output->data, SI5391A_OUT_CONFIG(output), &val);
	if (err < 0)
		return err;

	val &= ~SI5391A_OUT_CFG_PDN;
	val |= SI5391A_OUT_CFG_OE;

	return si5391a_reg_write(output->data, SI5391A_OUT_CONFIG(output), val);
}

static void si5391a_output_clk_unprepare(struct clk_hw *hw)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	u8 val;

	/* Set PDN bit */
	si5391a_reg_read(output->data, SI5391A_OUT_CONFIG(output), &val);
	val |= SI5391A_OUT_CFG_PDN;
	si5391a_reg_write(output->data, SI5391A_OUT_CONFIG(output), val);
}

static unsigned long si5391a_output_clk_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	int err;
	u32 r_divider;
	u8 buf[3];

	/* Read R divider value (24-bit) */
	err = si5391a_bulk_read(output->data, SI5391A_OUT_R_REG(output),
				 buf, sizeof(buf));
	if (err < 0)
		return 0;

	/* Convert to 32-bit value */
	r_divider = get_unaligned_le24(buf);

	if (!r_divider)
		r_divider = 1;

	dev_dbg(&output->data->i2c_client->dev,
		"Output %u divider: %u, parent_rate: %lu\n",
		output->index, r_divider, parent_rate);

	return parent_rate / r_divider;
}

static long si5391a_output_clk_round_rate(struct clk_hw *hw,
					   unsigned long rate,
					   unsigned long *parent_rate)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	u32 r_divider;

	r_divider = DIV_ROUND_CLOSEST(*parent_rate, rate);

	/* Clamp to valid range */
	if (r_divider < 1)
		r_divider = 1;
	if (r_divider > 0xffffff)
		r_divider = 0xffffff;

	dev_dbg(&output->data->i2c_client->dev,
		"Output %u round rate: %lu (div %u)\n",
		output->index, *parent_rate / r_divider, r_divider);

	return *parent_rate / r_divider;
}

static int si5391a_output_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct clk_si5391a_output *output = to_clk_si5391a_output(hw);
	u32 r_divider;
	u8 buf[3];

	r_divider = DIV_ROUND_CLOSEST(parent_rate, rate);

	if (r_divider < 1)
		r_divider = 1;
	if (r_divider > 0xffffff)
		r_divider = 0xffffff;

	put_unaligned_le24(r_divider, buf);

	dev_info(&output->data->i2c_client->dev,
		 "Setting output %u to %lu Hz (div %u from %lu Hz)\n",
		 output->index, parent_rate / r_divider, r_divider,
		 parent_rate);

	return si5391a_bulk_write(output->data, SI5391A_OUT_R_REG(output),
				  buf, sizeof(buf));
}

static const struct clk_ops si5391a_output_clk_ops = {
	.prepare = si5391a_output_clk_prepare,
	.unprepare = si5391a_output_clk_unprepare,
	.is_prepared = si5391a_output_clk_is_on,
	.recalc_rate = si5391a_output_clk_recalc_rate,
	.round_rate = si5391a_output_clk_round_rate,
	.set_rate = si5391a_output_clk_set_rate,
};

/*
 * PLL functions
 */
static unsigned long si5391a_pll_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_si5391a *data = to_clk_si5391a(hw);
	u64 res;
	u32 m_num, m_den;
	u8 buf[6];
	int err;

	/* Read M divider numerator and denominator */
	err = si5391a_bulk_read(data, SI5391A_PLL_M_NUM, buf, sizeof(buf));
	if (err < 0)
		return 0;

	m_num = get_unaligned_le32(buf);
	m_den = get_unaligned_le32(buf + 4);

	if (!m_den)
		m_den = 1;

	/* Calculate VCO frequency */
	res = parent_rate;
	res *= m_num;
	do_div(res, m_den);

	dev_dbg(&data->i2c_client->dev,
		"PLL: input %lu Hz, M=%u/%u, VCO %llu Hz\n",
		parent_rate, m_num, m_den, res);

	data->freq_vco = res;

	return res;
}

static long si5391a_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	/* VCO frequency range check */
	if (rate < SI5391A_PLL_VCO_MIN)
		rate = SI5391A_PLL_VCO_MIN;
	if (rate > SI5391A_PLL_VCO_MAX)
		rate = SI5391A_PLL_VCO_MAX;

	return rate;
}

static int si5391a_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct clk_si5391a *data = to_clk_si5391a(hw);
	u32 m_num, m_den;
	u8 buf[8];
	int err;

	/* Calculate M divider to achieve desired VCO frequency */
	m_den = 1000000;  /* Use fixed denominator for simplicity */
	m_num = div64_u64((u64)rate * m_den, parent_rate);

	put_unaligned_le32(m_num, buf);
	put_unaligned_le32(m_den, buf + 4);

	err = si5391a_bulk_write(data, SI5391A_PLL_M_NUM, buf, sizeof(buf));
	if (err < 0)
		return err;

	data->freq_vco = rate;

	dev_info(&data->i2c_client->dev,
		 "PLL set to %lu Hz (M=%u/%u from %lu Hz input)\n",
		 rate, m_num, m_den, parent_rate);

	return 0;
}

static const struct clk_ops si5391a_pll_ops = {
	.recalc_rate = si5391a_pll_recalc_rate,
	.round_rate = si5391a_pll_round_rate,
	.set_rate = si5391a_pll_set_rate,
};

/*
 * Device initialization
 */
static int si5391a_initialize_chip(struct clk_si5391a *data)
{
	int err;
	u8 status;

	/* Perform soft reset */
	err = si5391a_reg_write(data, SI5391A_SOFT_RST, 0x02);
	if (err < 0)
		return err;

	/* Wait for device to be ready */
	msleep(25);

	/* Check device ready status */
	err = si5391a_reg_read(data, SI5391A_DEVICE_READY, &status);
	if (err < 0)
		return err;

	if (!(status & 0x0F)) {
		dev_err(&data->i2c_client->dev, "Device not ready after reset\n");
		return -ENODEV;
	}

	/* Configure I/O voltage */
	if (data->iovdd_33) {
		err = si5391a_reg_write(data, SI5391A_IO_VDD_SEL, 0x01);
		if (err < 0)
			return err;
	}

	/* Enable XTAL or external clock */
	if (data->xaxb_ext_clk) {
		err = si5391a_reg_write(data, SI5391A_XAXB_CFG,
					 SI5391A_XAXB_CFG_EXTCLK_EN |
					 SI5391A_XAXB_CFG_PDNB);
	} else {
		err = si5391a_reg_write(data, SI5391A_XAXB_CFG,
					 SI5391A_XAXB_CFG_PDNB);
	}
	if (err < 0)
		return err;

	return 0;
}

static bool si5391a_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	return reg <= SI5391A_REGISTER_MAX;
}

static bool si5391a_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SI5391A_STATUS:
	case SI5391A_STATUS_STICKY:
	case SI5391A_LOS:
	case SI5391A_LOS_STICKY:
	case SI5391A_DEVICE_READY:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config si5391a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0xff,
	.writeable_reg = si5391a_regmap_is_writeable,
	.volatile_reg = si5391a_regmap_is_volatile,
};

/*
 * I2C probe function
 */
static int si5391a_dt_parse_dt(struct clk_si5391a *data,
				struct clk_si5391a_output_config *config)
{
	struct device_node *np = data->i2c_client->dev.of_node;
	struct device_node *child;
	u32 val;
	int i;

	/* Parse input clocks */
	for (i = 0; i < SI5391A_NUM_INPUTS; ++i) {
		data->input_clk[i] = devm_clk_get(&data->i2c_client->dev,
						   si5391a_input_clock_names[i]);
		if (!IS_ERR(data->input_clk[i])) {
			data->input_clk_name[i] = __clk_get_name(data->input_clk[i]);
		} else if (PTR_ERR(data->input_clk[i]) == -EPROBE_DEFER) {
			return -EPROBE_DEFER;
		} else {
			data->input_clk[i] = NULL;
			data->input_clk_name[i] = si5391a_input_clock_names[i];
		}
	}

	/* Parse properties */
	data->xaxb_ext_clk = of_property_read_bool(np, "skyworks,xaxb-ext-clk");
	data->iovdd_33 = of_property_read_bool(np, "skyworks,iovdd-33");

	/* Parse output configuration */
	for_each_child_of_node(np, child) {
		if (of_property_read_u32(child, "reg", &val)) {
			dev_warn(&data->i2c_client->dev,
				 "Missing reg property in output node\n");
			continue;
		}

		if (val >= data->num_outputs) {
			dev_warn(&data->i2c_client->dev,
				 "Invalid output index %u\n", val);
			continue;
		}

		i = val;

		if (!of_property_read_u32(child, "skyworks,format", &val)) {
			config[i].out_format_drv_bits = val & 0x07;
		}

		if (!of_property_read_u32(child, "skyworks,common-mode", &val)) {
			config[i].out_cm_ampl_bits = val;
		}

		if (!of_property_read_u32(child, "skyworks,amplitude", &val)) {
			config[i].out_cm_ampl_bits |= (val << 4);
		}

		config[i].always_on = of_property_read_bool(child, "always-on");
	}

	return 0;
}

static struct clk_hw *si5391a_of_clk_get(struct of_phandle_args *clkspec,
					  void *data)
{
	struct clk_si5391a *si5391a = data;
	unsigned int index = clkspec->args[0];

	if (index >= si5391a->num_outputs) {
		dev_err(&si5391a->i2c_client->dev,
			"Invalid output index %u\n", index);
		return ERR_PTR(-EINVAL);
	}

	return &si5391a->clk[index].hw;
}

static int si5391a_probe(struct i2c_client *client)
{
	struct clk_si5391a *data;
	struct clk_init_data init;
	struct clk_si5391a_output_config config[SI5391A_MAX_NUM_OUTPUTS] = {};
	int err;
	unsigned int i;
	u8 id[3];

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->i2c_client = client;
	i2c_set_clientdata(client, data);

	data->regmap = devm_regmap_init_i2c(client, &si5391a_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/* Set defaults */
	data->num_outputs = SI5391A_MAX_NUM_OUTPUTS;
	data->reg_output_offset = si5391a_reg_output_offset;
	data->reg_rdiv_offset = si5391a_reg_rdiv_offset;

	/* Read chip ID */
	err = si5391a_bulk_read(data, SI5391A_PN_BASE, id, sizeof(id));
	if (err < 0) {
		dev_err(&client->dev, "Failed to read chip ID\n");
		return err;
	}

	data->chip_id = (id[2] << 8) | id[1];

	dev_info(&client->dev, "Si5391A chip detected, ID=0x%04x, rev=%u\n",
		 data->chip_id, id[0]);

	/* Parse device tree */
	err = si5391a_dt_parse_dt(data, config);
	if (err < 0)
		return err;

	/* Initialize chip */
	err = si5391a_initialize_chip(data);
	if (err < 0) {
		dev_err(&client->dev, "Failed to initialize chip\n");
		return err;
	}

	/* Register PLL */
	init.name = kasprintf(GFP_KERNEL, "%s.pll", client->dev.of_node->name);
	if (!init.name)
		return -ENOMEM;

	init.ops = &si5391a_pll_ops;
	init.parent_names = data->input_clk_name;
	init.num_parents = SI5391A_NUM_INPUTS;
	init.flags = 0;

	data->hw.init = &init;
	err = devm_clk_hw_register(&client->dev, &data->hw);
	kfree(init.name);
	if (err) {
		dev_err(&client->dev, "PLL registration failed\n");
		return err;
	}

	/* Register output clocks */
	init.ops = &si5391a_output_clk_ops;
	init.num_parents = 1;
	init.parent_names = &init.name;  /* Use PLL as parent */
	init.flags = 0;

	for (i = 0; i < data->num_outputs; ++i) {
		init.name = kasprintf(GFP_KERNEL, "%s.%d",
				      client->dev.of_node->name, i);
		if (!init.name)
			return -ENOMEM;

		data->clk[i].index = i;
		data->clk[i].data = data;
		data->clk[i].hw.init = &init;

		/* Apply output configuration if specified */
		if (config[i].out_format_drv_bits & 0x07) {
			si5391a_reg_write(data, SI5391A_OUT_FORMAT(&data->clk[i]),
					  config[i].out_format_drv_bits);
			si5391a_reg_write(data, SI5391A_OUT_CM(&data->clk[i]),
					  config[i].out_cm_ampl_bits);
		}

		err = devm_clk_hw_register(&client->dev, &data->clk[i].hw);
		kfree(init.name);
		if (err) {
			dev_err(&client->dev,
				"Output %u registration failed\n", i);
			return err;
		}

		if (config[i].always_on)
			clk_prepare_enable(data->clk[i].hw.clk);
	}

	/* Register clock provider */
	err = devm_of_clk_add_hw_provider(&client->dev, si5391a_of_clk_get, data);
	if (err) {
		dev_err(&client->dev, "Unable to add clock provider\n");
		return err;
	}

	dev_info(&client->dev, "Si5391A clock generator registered\n");
	return 0;
}

static void si5391a_remove(struct i2c_client *client)
{
	/* Nothing to do, devm takes care of cleanup */
}

static const struct i2c_device_id si5391a_id[] = {
	{ "si5391a", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5391a_id);

static const struct of_device_id si5391a_of_match[] = {
	{ .compatible = "skyworks,si5391a" },
	{ }
};
MODULE_DEVICE_TABLE(of, si5391a_of_match);

static struct i2c_driver si5391a_driver = {
	.driver = {
		.name = "si5391a",
		.of_match_table = si5391a_of_match,
	},
	.probe = si5391a_probe,
	.remove = si5391a_remove,
	.id_table = si5391a_id,
};
module_i2c_driver(si5391a_driver);

MODULE_AUTHOR("Linux Kernel Developer");
MODULE_DESCRIPTION("Skyworks Si5391A Clock Generator Driver");
MODULE_LICENSE("GPL v2");