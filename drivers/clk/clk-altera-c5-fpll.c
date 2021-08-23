// SPDX-License-Identifier: GPL-2.0
/*
 * Altera FPLL driver
 *
 * Copyright 2021 Analog Devices Inc.
 *  Author: Liviu Adace <liviu.adace@analog.com>
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define C5_FPLL_MODE_REG_ADDR                      0x00
#define C5_FPLL_STATUS_REG_ADDR                    0x04
#define C5_FPLL_START_REG_ADDR                     0x08
#define C5_FPLL_N_CNTR_ADDR                        0x0C
#define C5_FPLL_M_CNTR_ADDR                        0x10
#define C5_FPLL_C_CNTR_ADDR                        0x14
#define C5_FPLL_DYNAMIC_PHASE_SHIFT_ADDR           0x18
#define C5_FPLL_M_CNTR_FRACTIONAL_VAL_ADDR         0x1C
#define C5_FPLL_BANDWIDTH_SETTINGS_ADDR            0x20
#define C5_FPLL_CHARGE_PUMP_SETTINGS_ADDR          0x24
#define C5_FPLL_VCO_POST_DEVIDE_CNTR_SETTINGS_ADDR 0x70
#define C5_FPLL_MIF_BASE_ADDR                      0x7C

#define C5_FPLL_REG_C_CNTR(x)	((x) << 18)
#define C5_FPLL_VCO_MAX		1600000000ull
#define C5_FPLL_NUM_CHAN	9

struct altera_c5_fpll_chan {
	struct altera_c5_fpll	*fpll;
	struct clk_hw		hw;
	u32			num;
	u8			c;
};

#define to_channel(_hw) container_of(_hw, struct altera_c5_fpll_chan, hw)

struct altera_c5_fpll {
	void __iomem			*base;
	struct device			*dev;
	struct clk			*ref_clk;
	struct clk			*clks[C5_FPLL_NUM_CHAN];
	struct altera_c5_fpll_chan	channels[C5_FPLL_NUM_CHAN];
	struct clk_onecell_data		clk_data;
	const char			*clk_out_names[C5_FPLL_NUM_CHAN];
	u64				vco;
	u32				fractional_carry_bit;
	u8				num_channels;
};

static long altera_c5_fpll_round_rate(struct clk_hw *hw,
				      unsigned long rate,
				      unsigned long *parent_rate)
{
	return rate;
}

static int altera_c5_fpll_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct altera_c5_fpll_chan *chan = to_channel(hw);
	struct altera_c5_fpll *fpll = chan->fpll;
	u32 in, reg, m_frac, m_int, status, last_rate;
	u64 m;
	u8 i, c;

	if (!rate)
		return -EINVAL;

	c = DIV_ROUND_CLOSEST_ULL(C5_FPLL_VCO_MAX, rate);
	fpll->vco = rate * c;
	if (fpll->vco > C5_FPLL_VCO_MAX) {
		c--;
		fpll->vco = rate * c;
	}

	in = clk_get_rate(fpll->ref_clk);

	m = (fpll->vco << fpll->fractional_carry_bit) + (in / 2);
	do_div(m, in);

	switch (fpll->fractional_carry_bit) {
	case 32:
		m_frac = m & GENMASK(31, 0);
		break;
	case 24:
		m_frac = m & GENMASK(23, 0);
		break;
	case 16:
		m_frac = m & GENMASK(15, 0);
		break;
	case 8:
		m_frac = m & GENMASK(7, 0);
		break;
	default:
		return -EINVAL;
	}

	m_int = m >> fpll->fractional_carry_bit;

	reg = m_int / 2;
	reg |= ((m_int / 2) + (m_int % 2)) << 8;

	if (m_int % 2)
		reg |= BIT(17);

	writel(0x01, fpll->base + C5_FPLL_MODE_REG_ADDR);
	writel(0x10101, fpll->base + C5_FPLL_N_CNTR_ADDR);
	writel(reg, fpll->base + C5_FPLL_M_CNTR_ADDR);
	writel(m_frac, fpll->base + C5_FPLL_M_CNTR_FRACTIONAL_VAL_ADDR);

	for (i = 0; i < fpll->num_channels; i++) {
		if (i == chan->num)
			last_rate = rate;
		else
			last_rate = clk_get_rate(fpll->clks[i]);

		if (!last_rate)
			continue;

		c = DIV_ROUND_CLOSEST_ULL(fpll->vco, last_rate);
		fpll->channels[i].c = c;

		reg = c / 2;
		reg |= ((c / 2) + (c % 2)) << 8;
		reg |= C5_FPLL_REG_C_CNTR(fpll->channels[i].num);

		if (c % 2)
			reg |= BIT(17);

		writel(reg, fpll->base + C5_FPLL_C_CNTR_ADDR);
	}

	writel(0x00, fpll->base + C5_FPLL_START_REG_ADDR);

	return readl_poll_timeout(fpll->base + C5_FPLL_STATUS_REG_ADDR,
				  status, status & 0x00000001, 1000, 30000);
}

static unsigned long altera_c5_fpll_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct altera_c5_fpll_chan *chan = to_channel(hw);
	struct altera_c5_fpll *fpll = chan->fpll;

	if (!chan->c)
		return 0;

	return DIV_ROUND_CLOSEST_ULL(fpll->vco, chan->c);
}

static const struct clk_ops altera_c5_fpll_ops = {
	.recalc_rate = altera_c5_fpll_recalc_rate,
	.round_rate = altera_c5_fpll_round_rate,
	.set_rate = altera_c5_fpll_set_rate,
};

static int altera_c5_fpll_parse_dt(struct altera_c5_fpll *fpll)
{
	struct device_node *np = fpll->dev->of_node, *chan_np;
	unsigned int cnt = 0;
	int ret;

	fpll->ref_clk = devm_clk_get(fpll->dev, NULL);
	if (IS_ERR(fpll->ref_clk))
		return PTR_ERR(fpll->ref_clk);

	fpll->num_channels = of_get_available_child_count(np);
	if (fpll->num_channels > C5_FPLL_NUM_CHAN)
		return -EINVAL;

	for_each_available_child_of_node(np, chan_np) {
		ret = of_property_read_u32(chan_np, "reg",
					   &fpll->channels[cnt].num);
		if (ret < 0) {
			dev_err(fpll->dev, "%s: Unable to parse channel DT property \"reg\"",
				__func__);
			return ret;
		}

		if (fpll->channels[cnt].num >= C5_FPLL_NUM_CHAN) {
			dev_err(fpll->dev, "%s: Invalid channed number %u",
				__func__, fpll->channels[cnt].num);
			return -EINVAL;
		}

		cnt++;
	};

	ret = of_property_read_u32(np, "adi,fractional-carry-bit",
				   &fpll->fractional_carry_bit);
	if (ret < 0) {
		dev_err(fpll->dev, "%s: Unable to parse DT property \"adi,fractional-carry-bit\"",
			__func__);
		return ret;
	}

	switch (fpll->fractional_carry_bit) {
	case 8:
	case 16:
	case 24:
	case 32:
		break;
	default:
		dev_err(fpll->dev, "%s: Invalid fractional-carry-bit value %u defined. Should be 8, 16, 24 or 32",
			__func__, fpll->fractional_carry_bit);
		return -EINVAL;
	}

	return of_property_read_string_array(np, "clock-output-names",
					    fpll->clk_out_names,
					    ARRAY_SIZE(fpll->clk_out_names));
};

static int altera_c5_fpll_outputs_setup(struct altera_c5_fpll *fpll)
{
	struct clk_init_data init;
	struct clk *clk;
	int i;

	for (i = 0; i < fpll->num_channels; i++) {
		init.name = fpll->clk_out_names[i];
		init.ops = &altera_c5_fpll_ops;
		init.flags = 0;
		init.parent_names = NULL;
		init.num_parents = 0;

		fpll->channels[i].fpll = fpll;
		fpll->channels[i].hw.init = &init;
		clk = devm_clk_register(fpll->dev, &fpll->channels[i].hw);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		fpll->clks[i] = clk;
	}

	return 0;
}

static void altera_c5_fpll_del_provider(void *data)
{
	of_clk_del_provider(data);
}

static const struct of_device_id altera_c5_fpll_ids[] = {
	{ .compatible = "altr,c5-fpll", },
	{ }
};
MODULE_DEVICE_TABLE(of, altera_c5_fpll_ids);

static int altera_c5_fpll_probe(struct platform_device *pdev)
{
	struct altera_c5_fpll *fpll;
	int ret;

	fpll = devm_kzalloc(&pdev->dev, sizeof(*fpll), GFP_KERNEL);
	if (!fpll)
		return -ENOMEM;

	fpll->dev = &pdev->dev;

	fpll->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fpll->base))
		return PTR_ERR(fpll->base);

	ret = altera_c5_fpll_parse_dt(fpll);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(fpll->ref_clk);
	if (ret < 0)
		return ret;

	ret = altera_c5_fpll_outputs_setup(fpll);
	if (ret < 0)
		return ret;

	fpll->clk_data.clks = fpll->clks;
	fpll->clk_data.clk_num = fpll->num_channels;

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				  &fpll->clk_data);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&pdev->dev,
					altera_c5_fpll_del_provider,
					pdev->dev.of_node);
}

static struct platform_driver altera_c5_fpll_driver = {
	.driver = {
		.name = "altera-c5-fpll",
		.of_match_table = altera_c5_fpll_ids,
	},
	.probe = altera_c5_fpll_probe
};
module_platform_driver(altera_c5_fpll_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Liviu Adace <liviu.adace@analog.com>");
MODULE_DESCRIPTION("Driver for the Altera Cyclone5 FPLL");
