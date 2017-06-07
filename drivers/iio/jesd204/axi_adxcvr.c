/*
 * ADI AXI-ADXCVR Module
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_adxcvr
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "xilinx_transceiver.h"

#define PCORE_VER(major, minor, letter)	((major << 16) | (minor << 8) | letter)
#define PCORE_VER_MAJOR(version)		(version >> 16)
#define PCORE_VER_MINOR(version)		((version >> 8) & 0xff)
#define PCORE_VER_LETTER(version)		(version & 0xff)

#define ADXCVR_REG_VERSION			0x0000
#define ADXCVR_VERSION(x)			(((x) & 0xffffffff) << 0)
#define ADXCVR_VERSION_IS(x, y, z)	((x) << 16 | (y) << 8 | (z))
#define ADXCVR_VERSION_MAJOR(x)		((x) >> 16)

#define ADXCVR_REG_ID				0x0004

#define ADXCVR_REG_SCRATCH			0x0008

#define ADXCVR_REG_RESETN			0x0010
#define ADXCVR_RESETN				(1 << 0)

#define ADXCVR_REG_STATUS			0x0014
#define ADXCVR_STATUS				(1 << 0)

#define ADXCVR_REG_CONTROL			0x0020
#define ADXCVR_LPM_DFE_N			(1 << 12)
#define ADXCVR_RATE(x)				(((x) & 0x7) << 8)
#define ADXCVR_SYSCLK_SEL(x)		(((x) & 0x3) << 4)
#define ADXCVR_OUTCLK_SEL(x)		(((x) & 0x7) << 0)

#define ADXCVR_REG_DRP_SEL(x)		(0x0040 + (x))

#define ADXCVR_REG_DRP_CTRL(x)		(0x0044 + (x))
#define ADXCVR_DRP_CTRL_WR			(1 << 28)
#define ADXCVR_DRP_CTRL_ADDR(x)		(((x) & 0xFFF) << 16)
#define ADXCVR_DRP_CTRL_WDATA(x)	(((x) & 0xFFFF) << 0)

#define ADXCVR_REG_DRP_STATUS(x)	(0x0048 + (x))
#define ADXCVR_DRP_STATUS_BUSY		(1 << 16)
#define ADXCVR_DRP_STATUS_RDATA(x)	(((x) & 0xFFFF) << 0)

#define ADXCVR_DRP_PORT_COMMON		0x00
#define ADXCVR_DRP_PORT_CHANNEL		0x20

#define ADXCVR_BROADCAST			0xff

struct adxcvr_state {
	struct device		*dev;
	void __iomem		*regs;
	struct clk			*conv_clk;
	struct clk			*lane_rate_div40_clk;
	struct clk_hw		out_clk_hw;
	bool				out_clk_enabled;
	struct work_struct	work;
	unsigned long		lane_rate;
	bool				tx_enable;
	u32					sys_clk_sel;
	u32					out_clk_sel;

	struct xilinx_xcvr	xcvr;

	bool				cpll_enable;
	bool				lpm_enable;
};

static struct adxcvr_state *xcvr_to_adxcvr(struct xilinx_xcvr *xcvr)
{
	return container_of(xcvr, struct adxcvr_state, xcvr);
}

static inline unsigned int adxcvr_read(struct adxcvr_state *st,
									   unsigned int reg)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
			 reg, ioread32(st->regs + reg));

	return ioread32(st->regs + reg);
}

static inline void adxcvr_write(struct adxcvr_state *st,
								unsigned int reg,
								unsigned int val)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
			 reg, val);

	iowrite32(val, st->regs + reg);
}

static int adxcvr_drp_wait_idle(struct adxcvr_state *st, unsigned int drp_port)
{
	unsigned int val;
	int timeout = 20;

	do {
		val = adxcvr_read(st, ADXCVR_REG_DRP_STATUS(drp_port));
		if (!(val & ADXCVR_DRP_STATUS_BUSY))
			return ADXCVR_DRP_STATUS_RDATA(val);

		mdelay(1);
	} while (timeout--);

	dev_err(st->dev, "%s: Timeout!", __func__);

	return -ETIMEDOUT;
}

static int adxcvr_drp_read(struct xilinx_xcvr *xcvr, unsigned int drp_port,
	unsigned int reg)
{
	struct adxcvr_state *st = xcvr_to_adxcvr(xcvr);
	int ret;

	adxcvr_write(st, ADXCVR_REG_DRP_SEL(drp_port), ADXCVR_BROADCAST);
	adxcvr_write(st, ADXCVR_REG_DRP_CTRL(drp_port), ADXCVR_DRP_CTRL_ADDR(reg));

	ret = adxcvr_drp_wait_idle(st, drp_port);
	if (ret < 0)
		return ret;

	return ret & 0xffff;
}

static int adxcvr_drp_write(struct xilinx_xcvr *xcvr, unsigned int drp_port,
	unsigned int reg, unsigned int val)
{
	struct adxcvr_state *st = xcvr_to_adxcvr(xcvr);
	int ret;

	adxcvr_write(st, ADXCVR_REG_DRP_SEL(drp_port), ADXCVR_BROADCAST);
	adxcvr_write(st, ADXCVR_REG_DRP_CTRL(drp_port), (ADXCVR_DRP_CTRL_WR |
		ADXCVR_DRP_CTRL_ADDR(reg) | ADXCVR_DRP_CTRL_WDATA(val)));

	ret = adxcvr_drp_wait_idle(st, drp_port);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct xilinx_xcvr_drp_ops adxcvr_drp_ops = {
	.read = adxcvr_drp_read,
	.write = adxcvr_drp_write,
};

static int adxcvr_status_error(struct device *dev)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	int timeout = 100;
	unsigned int status;

	do {
		mdelay(1);
		status = adxcvr_read(st, ADXCVR_REG_STATUS);
	} while ((timeout--) && (status == 0));

	if (!status) {
		dev_err(dev, "%s Error: %x", st->tx_enable ? "TX" : "RX", status);
		return -EIO;
	}

	return 0;
}

static void adxcvr_work_func(struct work_struct *work)
{
	struct adxcvr_state *st =
		container_of(work, struct adxcvr_state, work);

	unsigned long div40_rate;
	int ret;

	if (!IS_ERR(st->lane_rate_div40_clk)) {

		div40_rate = st->lane_rate * (1000 / 40);

		dev_dbg(st->dev, "%s: setting MMCM on %s rate %lu\n",
			__func__, st->tx_enable ? "TX" : "RX", div40_rate);

		if (__clk_is_enabled(st->lane_rate_div40_clk))
			clk_disable_unprepare(st->lane_rate_div40_clk);

		ret = clk_set_rate(st->lane_rate_div40_clk, div40_rate);
		if (ret < 0)
			dev_err(st->dev, "%s: setting MMCM on %s rate %lu failed (%d)\n",
				__func__, st->tx_enable ? "TX" : "RX", div40_rate, ret);

		ret = clk_prepare_enable(st->lane_rate_div40_clk);
		if (ret < 0)
			dev_err(st->dev, "%s: enabling MMCM rate %lu failed (%d)\n",
				__func__, div40_rate, ret);
	}
}

static int adxcvr_clk_enable(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	int ret;

	dev_dbg(st->dev, "%s: %s", __func__, st->tx_enable ? "TX" : "RX");

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	if (!st->tx_enable)
		xilinx_xcvr_configure_lpm_dfe_mode(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
				st->lpm_enable);

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);

	mdelay(100);

	ret = adxcvr_status_error(st->dev);

	return ret;
}

static void adxcvr_clk_disable(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);

	st->out_clk_enabled = false;
}

static int adxcvr_clk_is_enabled(struct clk_hw *hw)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);

	return st->out_clk_enabled;
}

static unsigned long adxcvr_clk_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	unsigned int *rx_out_div;
	unsigned int *tx_out_div;
	unsigned int out_div;

	dev_dbg(st->dev, "%s: Parent Rate %lu Hz",
		__func__, parent_rate);

	if (st->tx_enable) {
		rx_out_div = NULL;
		tx_out_div = &out_div;
	} else {
		rx_out_div = &out_div;
		tx_out_div = NULL;
	}

	xilinx_xcvr_read_out_div(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
		rx_out_div, tx_out_div);

	if (st->cpll_enable) {
		struct xilinx_xcvr_cpll_config cpll_conf;

		xilinx_xcvr_cpll_read_config(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
			&cpll_conf);
		return xilinx_xcvr_cpll_calc_lane_rate(&st->xcvr, parent_rate,
			&cpll_conf, out_div);

	} else {
		struct xilinx_xcvr_qpll_config qpll_conf;

		xilinx_xcvr_qpll_read_config(&st->xcvr, ADXCVR_DRP_PORT_COMMON,
			&qpll_conf);
		return xilinx_xcvr_qpll_calc_lane_rate(&st->xcvr, parent_rate,
			&qpll_conf, out_div);
	}
}

static long adxcvr_clk_round_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long *prate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	int ret;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, *prate);

	/* Just check if we can support the requested rate */
	if (st->cpll_enable)
		ret = xilinx_xcvr_calc_cpll_config(&st->xcvr, *prate, rate,
			NULL, NULL);
	else
		ret = xilinx_xcvr_calc_qpll_config(&st->xcvr, *prate, rate,
			NULL, NULL);

	return ret < 0 ? ret : rate;
}

static int adxcvr_clk_set_rate(struct clk_hw *hw, unsigned long rate,
							   unsigned long parent_rate)
{
	struct adxcvr_state *st =
		container_of(hw, struct adxcvr_state, out_clk_hw);
	struct xilinx_xcvr_cpll_config cpll_conf;
	struct xilinx_xcvr_qpll_config qpll_conf;
	unsigned int out_div, clk25_div;
	int ret;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, parent_rate);

	clk25_div = DIV_ROUND_CLOSEST(parent_rate, 25000000);

	if (st->cpll_enable)
		ret = xilinx_xcvr_calc_cpll_config(&st->xcvr, parent_rate, rate,
			&cpll_conf, &out_div);
	else
		ret = xilinx_xcvr_calc_qpll_config(&st->xcvr, parent_rate, rate,
			&qpll_conf, &out_div);
	if (ret < 0)
		return ret;

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	if (st->cpll_enable)
		ret = xilinx_xcvr_cpll_write_config(&st->xcvr,
			ADXCVR_DRP_PORT_CHANNEL, &cpll_conf);
	else
		ret = xilinx_xcvr_qpll_write_config(&st->xcvr,
			ADXCVR_DRP_PORT_COMMON, &qpll_conf);
	if (ret < 0)
		return ret;

	ret = xilinx_xcvr_write_out_div(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
		st->tx_enable ? -1 : out_div,
		st->tx_enable ? out_div : -1);
	if (ret < 0)
		return ret;

	if (!st->tx_enable) {
		ret = xilinx_xcvr_configure_cdr(&st->xcvr,
			ADXCVR_DRP_PORT_CHANNEL, rate, out_div, st->lpm_enable);
		if (ret < 0)
			return ret;

		ret = xilinx_xcvr_write_rx_clk25_div(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
			clk25_div);
	} else {
		ret = xilinx_xcvr_write_tx_clk25_div(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL,
			clk25_div);
	}
	if (ret < 0)
		return ret;

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);

	st->lane_rate = rate;

	if (!IS_ERR(st->lane_rate_div40_clk))
		schedule_work(&st->work);

	return 0;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = adxcvr_clk_recalc_rate,
	.enable = adxcvr_clk_enable,
	.disable = adxcvr_clk_disable,
	.is_enabled = adxcvr_clk_is_enabled,
	.round_rate = adxcvr_clk_round_rate,
	.set_rate = adxcvr_clk_set_rate,

};

static int adxcvr_clk_register(struct device *dev, struct device_node *node,
	const char *parent_name)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	struct clk_init_data init;
	struct clk *clk;
	const char *clk_name;
	int ret;

	ret = of_property_read_string_index(node, "clock-output-names",
		0, &clk_name);
	if (ret < 0)
		return ret;

	init.name = clk_name;
	init.ops = &clkout_ops;
	init.flags = 0;

	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	st->out_clk_hw.init = &init;

	/* register the clock */
	clk = devm_clk_register(dev, &st->out_clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

static int adxcvr_parse_dt(struct adxcvr_state *st,
						   struct device_node *np)
{
	bool gth_enable;

	gth_enable = of_property_read_bool(np, "adi,transceiver-gth-enable");
	st->tx_enable =
		of_property_read_bool(np, "adi,link-is-transmit-enable");

	of_property_read_u32(np, "adi,sys-clk-select",
				&st->sys_clk_sel);
	of_property_read_u32(np, "adi,out-clk-select",
				&st->out_clk_sel);

	st->cpll_enable = of_property_read_bool(np,
				"adi,use-cpll-enable");
	st->lpm_enable = of_property_read_bool(np,
				"adi,use-lpm-enable");

	INIT_WORK(&st->work, adxcvr_work_func);

	if (gth_enable)
		st->xcvr.type = XILINX_XCVR_TYPE_US_GTH3;
	else
		st->xcvr.type = XILINX_XCVR_TYPE_S7_GTX2;
	st->xcvr.encoding = ENC_8B10B;
	st->xcvr.refclk_ppm = PM_200; /* TODO use clock accuracy */

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id adxcvr_of_match[] = {
	{ .compatible = "adi,axi-adxcvr-1.0", .data = (void *) 1},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adxcvr_of_match);

static void adxcvr_enforce_settings(struct adxcvr_state *st)
{
	unsigned long lane_rate, parent_rate;

	/*
	 * Make sure filter settings, etc. are correct for the supplied reference
	 * clock. This is done since the reference clock might differ from the
	 * one specified during HDL synthesis.
	 */

	parent_rate = clk_get_rate(st->conv_clk);

	lane_rate = adxcvr_clk_recalc_rate(&st->out_clk_hw, parent_rate);

	adxcvr_clk_set_rate(&st->out_clk_hw, lane_rate, parent_rate);
}

static int adxcvr_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct adxcvr_state *st;
	struct resource *mem; /* IO mem resources */
	unsigned int version;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->conv_clk = devm_clk_get(&pdev->dev, "conv");
	if (IS_ERR(st->conv_clk))
		return PTR_ERR(st->conv_clk);

	st->lane_rate_div40_clk = devm_clk_get(&pdev->dev, "div40");
	if (IS_ERR(st->lane_rate_div40_clk)) {
		if (PTR_ERR(st->lane_rate_div40_clk) != -ENOENT)
			return PTR_ERR(st->lane_rate_div40_clk);
	}

	ret = clk_prepare_enable(st->conv_clk);
	if (ret < 0)
		return ret;

	st->xcvr.dev = &pdev->dev;
	st->xcvr.drp_ops = &adxcvr_drp_ops;

	ret = adxcvr_parse_dt(st, np);
	if (ret < 0)
		goto disable_unprepare;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs)) {
		ret = PTR_ERR(st->regs);
		goto disable_unprepare;
	}

	st->dev = &pdev->dev;
	version = adxcvr_read(st, ADXCVR_REG_VERSION);
	platform_set_drvdata(pdev, st);

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	adxcvr_write(st, ADXCVR_REG_CONTROL,
				 ((st->lpm_enable ? ADXCVR_LPM_DFE_N : 0) |
				  ADXCVR_SYSCLK_SEL(st->sys_clk_sel) |
				  ADXCVR_OUTCLK_SEL(st->out_clk_sel)));

	adxcvr_enforce_settings(st);

	ret = adxcvr_clk_register(&pdev->dev, np, __clk_get_name(st->conv_clk));
	if (ret)
		return ret;

	dev_info(&pdev->dev, "AXI-ADXCVR (%d.%.2d.%c) at 0x%08llX mapped to 0x%p,",
		PCORE_VER_MAJOR(version),
		PCORE_VER_MINOR(version),
		PCORE_VER_LETTER(version),
		(unsigned long long)mem->start, st->regs);

	return 0;

disable_unprepare:
	clk_disable_unprepare(st->conv_clk);

	return ret;
}

/**
 * adxcvr_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int adxcvr_remove(struct platform_device *pdev)
{
	struct adxcvr_state *st = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	clk_disable_unprepare(st->conv_clk);

	return 0;
}

static struct platform_driver adxcvr_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = adxcvr_of_match,
	},
	.probe		= adxcvr_probe,
	.remove		= adxcvr_remove,
};

module_platform_driver(adxcvr_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-ADXCVR Module");
MODULE_LICENSE("GPL v2");
