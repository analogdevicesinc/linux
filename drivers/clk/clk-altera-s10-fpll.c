
#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>

#define A10_REG_AVMM_ARBITRATION	0x000
#define A10_ARBITER_MSK			BIT(0)
#define A10_ARBITER(x)			(((x) & 0x1) << 0)
#define A10_PRESICE			1
#define A10_USER			0
#define A10_CALIB_MSK			BIT(1)
#define A10_CALIB(x)			(((x) & 0x1) << 1)
#define A10_CALIB_DONE			1
#define A10_CALIB_TRIGGER		0

#define S10_REG_AVMM_ARBITRATION	0x000
#define S10_ARBITER_MSK			BIT(0)
#define S10_ARBITER(x)			(((x) & 0x1) << 0)
#define S10_PRESICE			1
#define S10_USER			0
#define S10_CALIB_MSK			BIT(1)
#define S10_CALIB(x)			(((x) & 0x1) << 1)
#define S10_CALIB_DONE			1
#define S10_CALIB_TRIGGER		0

#define A10_REG_FPLL_CALIB_EN		0x100
#define A10_CALIB_EN_MSK		BIT(1)
#define A10_CALIB_EN(x)			(((x) & 0x1) << 1)
#define A10_CALIB_ENA			1
#define A10_CALIB_DIS			0

#define S10_REG_FPLL_CALIB_EN		0x100
#define S10_CALIB_EN_MSK		BIT(1)
#define S10_CALIB_EN(x)			(((x) & 0x1) << 1)
#define S10_CALIB_ENA			1
#define S10_CALIB_DIS			0

#define A10_REG_FPLL_STAT		0x280
#define A10_CALIB_STAT_MSK		BIT(1)
#define A10_CALIB_STAT(x)		(((x) & 0x1) << 1)
#define A10_CALIB_STAT_BUSY		1
#define A10_CALIB_STAT_DONE		0
#define A10_ARBITER_STAT_MSK		BIT(2)
#define A10_ARBITER_STAT(x)		(((x) & 0x1) << 2)
#define A10_STAT_PRESICE		1
#define A10_STAT_USER			0

#define S10_REG_FPLL_STAT		0x480
#define S10_CALIB_STAT_MSK		BIT(1)
#define S10_CALIB_STAT(x)		(((x) & 0x1) << 1)
#define S10_CALIB_STAT_BUSY		1
#define S10_CALIB_STAT_DONE		0
#define S10_ARBITER_STAT_MSK		BIT(2)
#define S10_ARBITER_STAT(x)		(((x) & 0x1) << 2)
#define S10_STAT_PRESICE		1
#define S10_STAT_USER			0

struct altera_fpll {
	void __iomem *base;
	struct clk_hw clk_hw;
	struct device *dev;
	unsigned long lane_rate;
	bool initial_recalc;
	bool is_s10;
};

static struct altera_fpll *clk_hw_to_altera_fpll(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct altera_fpll, clk_hw);
}

static unsigned int altera_fpll_read(struct altera_fpll *fpll,
	unsigned int addr)
{
	return readl(fpll->base + addr * 4);
}

static void altera_fpll_write(struct altera_fpll *fpll,
	unsigned int addr, unsigned int val)
{
	writel(val, fpll->base + addr * 4);
}

static void altera_fpll_update(struct altera_fpll *fpll,
	unsigned int addr, unsigned int mask, unsigned int val)
{
	unsigned int rval;

	rval = altera_fpll_read(fpll, addr);
	rval &= ~mask;
	rval |= val;
	altera_fpll_write(fpll, addr, rval);
}

static void request_avmm_user_control(struct altera_fpll *fpll)
{
	unsigned int timeout = 0;
	unsigned int status;

	altera_fpll_write(fpll, A10_REG_AVMM_ARBITRATION,
		A10_ARBITER(A10_USER) | A10_CALIB(A10_CALIB_DONE));

	do {
		status = altera_fpll_read(fpll, A10_REG_FPLL_STAT);
		if ((status & A10_ARBITER_STAT_MSK) ==
				A10_ARBITER_STAT(A10_STAT_USER)) {
			dev_dbg(fpll->dev, "Acquired AVMM user control: %d us\n",
				timeout * 10);
			return;
		}
		udelay(10);
	} while (timeout++ < 10000);

	dev_err(fpll->dev, "Failed to acquire AVMM user control\n");
}

static void release_avmm_user_control(struct altera_fpll *fpll,
	bool run_calibration)
{
	altera_fpll_write(fpll, A10_REG_AVMM_ARBITRATION, run_calibration ?
		A10_ARBITER(A10_PRESICE) | A10_CALIB(A10_CALIB_TRIGGER) :
		A10_ARBITER(A10_PRESICE) | A10_CALIB(A10_CALIB_DONE));
}

static int altera_fpll_calibration(struct altera_fpll *fpll)
{
	unsigned int timeout = 0;
	unsigned int val;

	request_avmm_user_control(fpll);
	altera_fpll_update(fpll, A10_REG_FPLL_CALIB_EN, 0x2, 0x2);
	release_avmm_user_control(fpll, true);

	/* Wait max 100ms for cal_busy to de-assert */
	do {
		udelay(200);

		/* Read FPLL calibration status from capability register */
		val = altera_fpll_read(fpll, fpll->is_s10 ? 0x480 : 0x280);
		if ((val & 0x02) == 0x00) {
			dev_info(fpll->dev, "Altera FPLL %s calibration OK (%d us)\n",
				fpll->is_s10 ? "S10" : "A10", timeout * 200);
			return 0;
		}
	} while (timeout++ < 50);

	dev_err(fpll->dev, "Altera FPLL calibration FAILED\n");

	return 1;
}


static long altera_fpll_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	return rate;
}

static int altera_fpll_set_rate(struct clk_hw *clk_hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct altera_fpll *fpll = clk_hw_to_altera_fpll(clk_hw);

	altera_fpll_calibration(fpll);

	fpll->initial_recalc = false;

	fpll->lane_rate = rate;

	return 0;
}

static unsigned long altera_fpll_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct altera_fpll *fpll = clk_hw_to_altera_fpll(clk_hw);

	/*
	 * Recalc configuration in case ref clock is not the same as in the FPGA
	 * project.
	 */
	if (fpll->initial_recalc)
		altera_fpll_set_rate(clk_hw, parent_rate, parent_rate);

	return fpll->lane_rate;
}

static int altera_fpll_is_enabled(struct clk_hw *clk_hw)
{
	struct altera_fpll *fpll = clk_hw_to_altera_fpll(clk_hw);

	return altera_fpll_read(fpll, 0x2e0) != 0x3;
}


static int altera_fpll_enable(struct clk_hw *clk_hw)
{
	struct altera_fpll *fpll = clk_hw_to_altera_fpll(clk_hw);

	altera_fpll_write(fpll, 0x2e0, 0x2);

	return 0;
}

static void altera_fpll_disable(struct clk_hw *clk_hw)
{
	struct altera_fpll *fpll = clk_hw_to_altera_fpll(clk_hw);

	altera_fpll_write(fpll, 0x2e0, 0x3);
}

static const struct clk_ops altera_fpll_ops = {
	.recalc_rate = altera_fpll_recalc_rate,
	.is_enabled = altera_fpll_is_enabled,
	.enable = altera_fpll_enable,
	.disable = altera_fpll_disable,
	.set_rate = altera_fpll_set_rate,
	.round_rate = altera_fpll_round_rate,
};

static const struct of_device_id altera_fpll_ids[] = {
	{ .compatible = "altr,fpll", },
	{ },
};
MODULE_DEVICE_TABLE(of, altera_fpll_ids);

static int altera_fpll_probe(struct platform_device *pdev)
{
	struct altera_fpll *fpll;
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

	fpll->is_s10 = true;

	init.name = clk_name;
	init.ops = &altera_fpll_ops;
	init.flags = CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE;
	init.parent_names = parent_names;

	fpll->clk_hw.init = &init;
	clk = devm_clk_register(&pdev->dev, &fpll->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get,
				    clk);
}

static int altera_fpll_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static struct platform_driver altera_fpll_driver = {
	.driver = {
		.name = "altera-fpll",
		.of_match_table = altera_fpll_ids,
	},
	.probe = altera_fpll_probe,
	.remove = altera_fpll_remove,
};
module_platform_driver(altera_fpll_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Altera FPLL");