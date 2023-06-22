// SPDX-License-Identifier: GPL-2.0-only
/*
 * Flexio core driver
 *
 * Copyright 2023 NXP
 * Copyright 2022 NXP
 *
 */

#include <linux/export.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mfd/imx-flexio.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

void flexio_writel(void *base, u32 val, unsigned int reg)
{
	writel_relaxed(val, base + reg);
}
EXPORT_SYMBOL_GPL(flexio_writel);

u32 flexio_readl(void *base, unsigned int reg)
{
	return readl_relaxed(base + reg);
}
EXPORT_SYMBOL_GPL(flexio_readl);

void flexio_sw_reset(void *base, unsigned int reg)
{
	u32 val = 0;

	val = flexio_readl(base, reg) | FLEXIO_CTRL_SWRST_MASK;
	flexio_writel(base, val, reg);

	flexio_writel(base, 0, reg);
}
EXPORT_SYMBOL_GPL(flexio_sw_reset);

void flexio_get_default_ctrl(struct flexio_control *ctrl)
{
	memset(ctrl, 0, sizeof(*ctrl));

	ctrl->dozen     = false;
	ctrl->dbge      = true;
	ctrl->fastacc   = false;
	ctrl->flexen    = true;
}
EXPORT_SYMBOL_GPL(flexio_get_default_ctrl);

void flexio_setup_ctrl(void *base, struct flexio_control *ctrl,
		       unsigned int reg)
{
	u32 val = 0;

	val &= ~(FLEXIO_CTRL_DOZEN_MASK | FLEXIO_CTRL_DBGE_MASK |
		 FLEXIO_CTRL_FASTACC_MASK | FLEXIO_CTRL_FLEXEN_MASK);
	val |= (FLEXIO_CTRL_DBGE(ctrl->dbge) |
		FLEXIO_CTRL_FASTACC(ctrl->fastacc) |
		FLEXIO_CTRL_FLEXEN(ctrl->flexen));

	if (!ctrl->dozen)
		val |= FLEXIO_CTRL_DOZEN_MASK;

	flexio_writel(base, val, reg);
}
EXPORT_SYMBOL_GPL(flexio_setup_ctrl);

void flexio_setup_shiftctl(void *base, struct flexio_shifter_control *ctl,
			   unsigned int reg)
{
	u32 val = 0;

	val = SHIFTCTL_TIMSEL(ctl->timsel) | SHIFTCTL_TIMPOL(ctl->timpol) |
	      SHIFTCTL_PINCFG(ctl->pincfg) | SHIFTCTL_PINSEL(ctl->pinsel) |
	      SHIFTCTL_PINPOL(ctl->pinpol) | SHIFTCTL_SMOD(ctl->smod);

	flexio_writel(base, val, reg);
}
EXPORT_SYMBOL_GPL(flexio_setup_shiftctl);

void flexio_setup_shiftcfg(void *base, struct flexio_shifter_config *cfg,
			   unsigned int reg)
{
	u32 val = 0;

	val = SHIFTCFG_INSRC(cfg->insrc) | SHIFTCFG_SSTOP(cfg->sstop) |
	      SHIFTCFG_SSTART(cfg->sstart);

	flexio_writel(base, val, reg);
}
EXPORT_SYMBOL_GPL(flexio_setup_shiftcfg);

void flexio_setup_timerctl(void *base, struct flexio_timer_control *ctl,
			   unsigned int reg)
{
	u32 val = 0;

	val = TIMCTL_TRGSEL(ctl->trgsel) | TIMCTL_TRGPOL(ctl->trgpol) |
	      TIMCTL_TRGSRC(ctl->trgsrc) | TIMCTL_PINCFG(ctl->pincfg) |
	      TIMCTL_PINSEL(ctl->pinsel) | TIMCTL_PINPOL(ctl->pinpol) |
	      TIMCTL_TIMOD(ctl->timod);

	flexio_writel(base, val, reg);
}
EXPORT_SYMBOL_GPL(flexio_setup_timerctl);

void flexio_setup_timercfg(void *base, struct flexio_timer_config *cfg,
			   unsigned int reg)
{
	u32 val = 0;

	val = TIMCFG_TIMOUT(cfg->timout) | TIMCFG_TIMDEC(cfg->timdec) |
	      TIMCFG_TIMRST(cfg->timrst) | TIMCFG_TIMDIS(cfg->timdis) |
	      TIMCFG_TIMENA(cfg->timena) | TIMCFG_TSTOP(cfg->tstop) |
	      TIMCFG_TSTART(cfg->tstart);

	flexio_writel(base, val, reg);
}
EXPORT_SYMBOL_GPL(flexio_setup_timercfg);

static int imx_flexio_probe(struct platform_device *pdev)
{
	struct flexio_ddata *ddata;
	struct resource *res;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->irq = platform_get_irq(pdev, 0);
	if (ddata->irq < 0)
		return -EINVAL;

	ddata->per_clk = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(ddata->per_clk))
		return PTR_ERR(ddata->per_clk);

	ddata->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(ddata->ipg_clk))
		return PTR_ERR(ddata->ipg_clk);

	platform_set_drvdata(pdev, ddata);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id imx_flexio_of_match[] = {
	{ .compatible = "nxp,imx-flexio" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx_flexio_of_match);

static struct platform_driver imx_flexio_driver = {
	.probe = imx_flexio_probe,
	.driver = {
		.name = "imx-flexio",
		.of_match_table = imx_flexio_of_match,
	},
};
module_platform_driver(imx_flexio_driver);

MODULE_DESCRIPTION("NXP I.MX FlexIO Core driver");
MODULE_AUTHOR("NXP Semiconductors");
MODULE_ALIAS("platform:imx-flexio");
MODULE_LICENSE("GPL v2");
