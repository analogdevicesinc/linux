/*
 * Altera FPLL driver
 *
 * Copyright 2017 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 *
 */

#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>

#define FPLL_REG_C_COUNTER0			0x10D
#define FPLL_REG_C_COUNTER1			0x10E
#define FPLL_REG_C_COUNTER2			0x10F
#define FPLL_REG_C_COUNTER3			0x110
#define FPLL_REG_C_COUNTER4			0x111

#define FPLL_REG_DIV_COUNTER0		0x12B
#define FPLL_REG_DIV_COUNTER1		0x12C

struct altera_a10_fpll {
	void __iomem *base;
	struct clk_hw clk_hw;
	struct device *dev;
	bool initial_recalc;
};

static struct altera_a10_fpll *clk_hw_to_altera_a10_fpll(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct altera_a10_fpll, clk_hw);
}

static unsigned int altera_a10_fpll_read(struct altera_a10_fpll *fpll,
	unsigned int addr)
{
	return readl(fpll->base + addr * 4);
}

static void altera_a10_fpll_write(struct altera_a10_fpll *fpll,
	unsigned int addr, unsigned int val)
{
	writel(val, fpll->base + addr * 4);
}

static void altera_a10_fpll_update(struct altera_a10_fpll *fpll,
	unsigned int addr, unsigned int mask, unsigned int val)
{
	unsigned int rval;

	rval = altera_a10_fpll_read(fpll, addr);
	rval &= ~mask;
	rval |= val;
	altera_a10_fpll_write(fpll, addr, rval);
}

static unsigned int altera_a10_acquire_arbitration(struct altera_a10_fpll *fpll)
{
	unsigned int timeout = 0;
	unsigned int status;

	altera_a10_fpll_write(fpll, 0x00, 0x2);

	do {
		status = altera_a10_fpll_read(fpll, 0x280);
		if ((status & BIT(2)) == 0) {
			dev_dbg(fpll->dev, "Acquired arbitration: %d us\n",
				timeout * 10);
			return 0;
		}
		udelay(10);
	} while (timeout++ < 10000);

	dev_err(fpll->dev, "Failed to acquire arbitration\n");

	return 0;
}

static void altera_a10_release_arbitration(struct altera_a10_fpll *fpll,
	bool run_calibration)
{
	altera_a10_fpll_write(fpll, 0x00, run_calibration ? 0x1 : 0x3);
}

static unsigned int altera_a10_fpll_lookup_lf_resistance(unsigned int fvco,
	unsigned long m)
{
	if (fvco < 9000000) {
		switch (m) {
		case 0 ... 15:
			return 2;
		case 16 ... 31:
			return 0;
		case 32 ... 71:
			return 1;
		default:
			return 3;
		}
	} else if (fvco < 12000000) {
		switch (m) {
		case 0 ... 15:
			return 2;
		case 16 ... 31:
			return 0;
		case 32 ... 71:
			return 1;
		default:
			return 3;
		}
	} else {
		switch (m) {
		case 0 ... 23:
			return 0;
		case 24 ... 55:
			return 1;
		default:
			return 3;
		}
	}
}

static int altera_a10_fpll_lookup_cp_current(unsigned long fvco,
	unsigned long m)
{
	if (fvco < 9000000) {
		switch (m) {
		case 0 ... 15:
			return 29;
		case 16 ... 23:
			return 28;
		case 24 ... 31:
			return 29;
		case 32 ... 47:
			return 28;
		case 48 ... 55:
			return 29;
		case 56 ... 71:
			return 36;
		case 72 ... 103:
			return 29;
		default:
			return 36;
		}
	} else if (fvco < 12000000) {
		switch (m) {
		case 0 ... 15:
			return 36;
		case 16 ... 23:
			return 29;
		case 24 ... 31:
			return 36;
		case 32 ... 39:
			return 28;
		case 40 ... 55:
			return 29;
		case 56 ... 79:
			return 36;
		case 80 ... 87:
			return 29;
		default:
			return 36;
		}
	} else {
		switch (m) {
		case 0 ... 15:
			return 28;
		case 16 ... 23:
			return 36;
		case 24 ... 39:
			return 29;
		case 40 ... 55:
			return 36;
		case 56 ... 63:
			return 29;
		default:
			return 36;
		}
	}
}

#define A10_FPLL_PFD_MIN 25000    /* 25.000 MHz */
#define A10_FPLL_PFD_MAX 60000    /* 60.000 MHz */
#define A10_FPLL_VCO_MIN 4800000  /*  4.800 GHz */
#define A10_FPLL_VCO_MAX 14025000 /* 14.025 GHz */

static int altera_a10_fpll_calc_params(unsigned long fref,
	unsigned long fout, unsigned int *best_n, unsigned int *best_m,
	unsigned int *best_c0, unsigned long *best_fvco)
{
	unsigned long n, n_min, n_max, _n_min, _n_max;
	unsigned long m, m_min, m_max;
	unsigned long c0;
	unsigned long f, fvco, best_f;

	*best_n = *best_m = *best_c0 = *best_fvco = 0;

	fref /= 1000;
	fout /= 1000;

	best_f = ULONG_MAX;

	n_min = max_t(unsigned long, DIV_ROUND_UP(fref, A10_FPLL_PFD_MAX), 1);
	n_max = min_t(unsigned long, fref / A10_FPLL_PFD_MIN, 31);

	m_min = max_t(unsigned long, DIV_ROUND_UP(A10_FPLL_VCO_MIN / 2, fref) * n_min, 8);
	m_max = min_t(unsigned long, A10_FPLL_VCO_MAX / 2 * n_max / fref, 127);
	m_min = round_up(m_min, 2);

	for (m = m_min; m <= m_max; m += 2) {
		_n_min = max_t(unsigned long, n_min, DIV_ROUND_UP(fref * m, A10_FPLL_VCO_MAX / 2));
		_n_max = min_t(unsigned long, n_max, fref * m / (A10_FPLL_VCO_MIN / 2));

		for (n = _n_min; n <= _n_max; n++) {
			fvco = fref * m * 2 / n;

			c0 = DIV_ROUND_CLOSEST(fvco, fout * 4);
			c0 = clamp_t(unsigned long, c0, 1, 512);
			f = fvco / (c0 * 4);

			if (abs(f - fout) < abs(best_f - fout)) {
				best_f = f;
				*best_n = n;
				*best_m = m;
				*best_c0 = c0;
				*best_fvco = fvco;
				if (f == fout)
					return 0;
			}
		}
	}

	return -EINVAL;
}

static long altera_a10_fpll_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	unsigned int n, m, c0;
	unsigned long fvco;
	unsigned long long tmp;

	altera_a10_fpll_calc_params(*parent_rate, rate, &n, &m, &c0, &fvco);

	if (n == 0 || m == 0 || c0 == 0)
		return -EINVAL;

	tmp = (unsigned long long)*parent_rate * m;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, c0 * n * 2);

	return min_t(unsigned long long, tmp, LONG_MAX);
}

static int altera_a10_fpll_pll_calibration_check(struct altera_a10_fpll *fpll)
{
	unsigned int timeout = 0;
	unsigned int val;

	/* Wait max 100ms for cal_busy to de-assert */
	do {
		udelay(200);

		/* Read FPLL calibration status from capability register */
		val = altera_a10_fpll_read(fpll, 0x280);
		if ((val & 0x02) == 0x00) {
			dev_info(fpll->dev, "FPLL PLL calibration OK (%d us)\n",
				timeout * 200);
			return 0;
		}
	} while (timeout++ < 50);

	dev_err(fpll->dev, "FPLL PLL calibration FAILED\n");

	return 1;
}

static int altera_a10_fpll_set_rate(struct clk_hw *clk_hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct altera_a10_fpll *fpll = clk_hw_to_altera_a10_fpll(clk_hw);
	unsigned int n, m, c0;
	unsigned long fvco;
	unsigned int div0, div1;
	unsigned int lfr, cpc;

	altera_a10_fpll_calc_params(parent_rate, rate, &n, &m, &c0, &fvco);

	if (n == 0 || m == 0 || c0 == 0)
		return -EINVAL;

	lfr = altera_a10_fpll_lookup_lf_resistance(fvco, m);
	cpc = altera_a10_fpll_lookup_cp_current(fvco, m);

	if (c0 >= 512)
		c0 = 0;

	div0 = m & 0xff;
	div1 = ((n << 3) & 0xf8);

	altera_a10_acquire_arbitration(fpll);

	altera_a10_fpll_write(fpll, FPLL_REG_DIV_COUNTER0, div0);
	altera_a10_fpll_write(fpll, FPLL_REG_DIV_COUNTER1, div1);

	div0 = c0 & 0xff;
	div1 = (c0 & 0x100) >> 4;

	altera_a10_fpll_write(fpll, FPLL_REG_C_COUNTER2, m / 2);
	altera_a10_fpll_write(fpll, FPLL_REG_C_COUNTER3, div0);
	altera_a10_fpll_update(fpll, FPLL_REG_C_COUNTER4, 0x0f, div1);

	altera_a10_fpll_update(fpll, 0x133, 0x0c, lfr << 2);
	altera_a10_fpll_update(fpll, 0x134, 0x70, (cpc & 0x7) << 4);
	altera_a10_fpll_update(fpll, 0x135, 0x07, (cpc & 0x38) >> 3);

	altera_a10_fpll_update(fpll, 0x100, 0x2, 0x2);
	altera_a10_release_arbitration(fpll, true);
	altera_a10_fpll_pll_calibration_check(fpll);

	fpll->initial_recalc = false;

	return 0;
}

static unsigned long altera_a10_fpll_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct altera_a10_fpll *fpll = clk_hw_to_altera_a10_fpll(clk_hw);
	unsigned int m, n, c0;
	unsigned long long tmp;
	unsigned int div0, div1;

	altera_a10_acquire_arbitration(fpll);

	div0 = altera_a10_fpll_read(fpll, FPLL_REG_DIV_COUNTER0);
	div1 = altera_a10_fpll_read(fpll, FPLL_REG_DIV_COUNTER1);

	n = (div1 >> 3) & 0x1f;
	m = div0 | ((div1 & 0x1) << 8);

	if (m == 0)
		m = 512;

	div0 = altera_a10_fpll_read(fpll, FPLL_REG_C_COUNTER3);
	div1 = altera_a10_fpll_read(fpll, FPLL_REG_C_COUNTER4);

	altera_a10_release_arbitration(fpll, false);

	c0 = div0 | ((div1 & 0x8) << 5);

	if (c0 == 0 || n == 0)
		return 0;

	tmp = (unsigned long long)parent_rate * m;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, c0 * n * 2);

	/*
	 * Recalc configuration in case ref clock is not the same as in the FPGA
	 * project.
	 */
	if (tmp != 0 && fpll->initial_recalc)
		altera_a10_fpll_set_rate(clk_hw, tmp, parent_rate);

	return min_t(unsigned long long, tmp, ULONG_MAX);
}


static int altera_a10_fpll_is_enabled(struct clk_hw *clk_hw)
{
	struct altera_a10_fpll *fpll = clk_hw_to_altera_a10_fpll(clk_hw);

	return altera_a10_fpll_read(fpll, 0x2e0) != 0x3;
}


static int altera_a10_fpll_enable(struct clk_hw *clk_hw)
{
	struct altera_a10_fpll *fpll = clk_hw_to_altera_a10_fpll(clk_hw);

	altera_a10_fpll_write(fpll, 0x2e0, 0x2);

	return 0;
}

static void altera_a10_fpll_disable(struct clk_hw *clk_hw)
{
	struct altera_a10_fpll *fpll = clk_hw_to_altera_a10_fpll(clk_hw);

	altera_a10_fpll_write(fpll, 0x2e0, 0x3);
}

static const struct clk_ops altera_a10_fpll_ops = {
	.recalc_rate = altera_a10_fpll_recalc_rate,
	.is_enabled = altera_a10_fpll_is_enabled,
	.enable = altera_a10_fpll_enable,
	.disable = altera_a10_fpll_disable,
	.set_rate = altera_a10_fpll_set_rate,
	.round_rate = altera_a10_fpll_round_rate,
};

static const struct of_device_id altera_a10_fpll_ids[] = {
	{ .compatible = "altr,a10-fpll", },
	{ },
};
MODULE_DEVICE_TABLE(of, altera_a10_fpll_ids);

static int altera_a10_fpll_probe(struct platform_device *pdev)
{
	struct altera_a10_fpll *fpll;
	struct clk_init_data init;
	const char *parent_names[2];
	const char *clk_name;
	struct resource *mem;
	struct clk *clk;
	unsigned int i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	fpll = devm_kzalloc(&pdev->dev, sizeof(*fpll), GFP_KERNEL);
	if (!fpll)
		return -ENOMEM;

	fpll->dev = &pdev->dev;
	fpll->initial_recalc = true;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fpll->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(fpll->base))
		return PTR_ERR(fpll->base);

	init.num_parents = of_clk_get_parent_count(pdev->dev.of_node);
	if (init.num_parents != 1)
		return -EINVAL;

	for (i = 0; i < init.num_parents; i++) {
		parent_names[i] = of_clk_get_parent_name(pdev->dev.of_node, i);
		if (!parent_names[i])
			return -EINVAL;
	}

	clk_name = pdev->dev.of_node->name;
	of_property_read_string(pdev->dev.of_node, "clock-output-names",
		&clk_name);

	init.name = clk_name;
	init.ops = &altera_a10_fpll_ops;
	init.flags = CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE;
	init.parent_names = parent_names;

	fpll->clk_hw.init = &init;
	clk = devm_clk_register(&pdev->dev, &fpll->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get,
				    clk);
}

static int altera_a10_fpll_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static struct platform_driver altera_a10_fpll_driver = {
	.driver = {
		.name = "altera-a10-fpll",
		.of_match_table = altera_a10_fpll_ids,
	},
	.probe = altera_a10_fpll_probe,
	.remove = altera_a10_fpll_remove,
};
module_platform_driver(altera_a10_fpll_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Altera Arria10 FPLL");
