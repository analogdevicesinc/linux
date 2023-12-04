// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2023 NXP
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"
#include "imx_rproc.h"

/* imx95 Reset and Control register*/
#define IMX95_SRC_SCR		0x00
#define IMX95_NEUTRON_START	0x1F11
#define IMX95_NEUTRON_STOP	0x1F00
#define IMX95_NEUTRON_RST_MASK	0xFFFF

/* att flags: lower 16 bits specifying core, higher 16 bits for flags  */
#define ATT_OWN			BIT(31)
#define ATT_IOMEM		BIT(30)

#define IMX_RPROC_MEM_MAX	32

static const struct imx_rproc_att neutron_rproc_att_imx95[] = {
	/* dev addr , sys addr  , size	    , flags */

	/* DTCM-Data NON-SECURE 16K */
	{ 0x00040000, 0x4AB08000, 0x00004000, ATT_OWN | ATT_IOMEM },

	/* DTCM-Ring NON-SECURE 16K */
	{ 0x00044000, 0x4AB0C000, 0x00004000, ATT_OWN | ATT_IOMEM },

	/* ITCM NON-SECURE 64K */
	{ 0x00000000, 0x4AB10000, 0x00010000, ATT_OWN | ATT_IOMEM },

	/*TODO if need to add SECURE memmap*/
};

static const struct imx_rproc_dcfg neutron_rproc_cfg_imx95 = {
	.src_reg	= IMX95_SRC_SCR,
	.src_mask	= IMX95_NEUTRON_RST_MASK,
	.src_start	= IMX95_NEUTRON_START,
	.src_stop	= IMX95_NEUTRON_STOP,
	.att		= neutron_rproc_att_imx95,
	.att_size	= ARRAY_SIZE(neutron_rproc_att_imx95),
};

/**
 * struct neutron_rproc_mem - slim internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @sys_addr: Bus address used to access the memory region
 * @size: Size of the memory region
 */
struct neutron_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t sys_addr;
	size_t size;
};

/**
 * struct neutron_rproc - Neutron remote processor state
 * @rproc: rproc handle
 * @pdev: pointer to platform device
 * @mem: Neutron memory information
 * @rsts: reset control
 */
struct neutron_rproc {
	struct device			*dev;
	void __iomem			*regbase;
	struct rproc			*rproc;
	struct platform_device		*pdev;
	struct neutron_rproc_mem	mem[IMX_RPROC_MEM_MAX];
	const struct			imx_rproc_dcfg *dcfg;
};

static int neutron_rproc_start(struct rproc *rproc)
{
	struct neutron_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;

	writel(dcfg->src_start, priv->regbase + dcfg->src_reg);

	if (readl(priv->regbase + dcfg->src_reg) != dcfg->src_start)
		return -EINVAL;

	return 0;
}

static int neutron_rproc_stop(struct rproc *rproc)
{
	struct neutron_rproc *priv = rproc->priv;
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;

	writel(dcfg->src_stop, priv->regbase + dcfg->src_reg);

	if (readl(priv->regbase + dcfg->src_reg) != dcfg->src_stop)
		return -EINVAL;

	return 0;
}

static int neutron_rproc_da_to_sys(struct neutron_rproc *priv, u64 da,
				   size_t len, u64 *sys, bool *is_iomem)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	int i;

	/* parse address translation table */
	for (i = 0; i < dcfg->att_size; i++) {
		const struct imx_rproc_att *att = &dcfg->att[i];

		if (da >= att->da && da + len < att->da + att->size) {
			unsigned int offset = da - att->da;

			*sys = att->sa + offset;
			if (is_iomem)
				*is_iomem = att->flags & ATT_IOMEM;
			return 0;
		}
	}

	dev_warn(priv->dev, "Translation failed: da = 0x%llx len = 0x%zx\n",
		 da, len);
	return -ENOENT;
}

static void *neutron_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct neutron_rproc *priv = rproc->priv;
	void *va = NULL;
	u64 sys;
	int i;

	/*
	 * On device side we have many aliases, so we need to convert device
	 * address (Zen-V) to system bus address first.
	 */
	if (len == 0 || neutron_rproc_da_to_sys(priv, da, len, &sys, is_iomem))
		return NULL;

	for (i = 0; i < IMX_RPROC_MEM_MAX; i++) {
		if (sys >= priv->mem[i].sys_addr && sys + len <
		    priv->mem[i].sys_addr +  priv->mem[i].size) {
			unsigned int offset = sys - priv->mem[i].sys_addr;
			/* __force to make sparse happy with type conversion */
			va = (__force void *)(priv->mem[i].cpu_addr + offset);
			break;
		}
	}

	return va;
}

static const struct rproc_ops neutron_rproc_ops = {
	.start		= neutron_rproc_start,
	.stop		= neutron_rproc_stop,
	.da_to_va	= neutron_rproc_da_to_va,
};

static int neutron_rproc_addr_init(struct neutron_rproc *priv,
				   struct platform_device *pdev)
{
	const struct imx_rproc_dcfg *dcfg = priv->dcfg;
	struct device *dev = &pdev->dev;
	int a, b = 0;

	/* remap required addresses */
	for (a = 0; a < dcfg->att_size; a++) {
		const struct imx_rproc_att *att = &dcfg->att[a];

		if (!(att->flags & ATT_OWN))
			continue;

		if (b >= IMX_RPROC_MEM_MAX)
			break;

		if (att->flags & ATT_IOMEM)
			priv->mem[b].cpu_addr = devm_ioremap(&pdev->dev,
							     att->sa, att->size);
		else
			priv->mem[b].cpu_addr = devm_ioremap_wc(&pdev->dev,
								att->sa, att->size);
		if (!priv->mem[b].cpu_addr) {
			dev_err(dev, "failed to remap %#x bytes from %#x\n", att->size, att->sa);
			return -ENOMEM;
		}
		priv->mem[b].sys_addr = att->sa;
		priv->mem[b].size = att->size;
		b++;
	}

	writel(dcfg->src_stop, priv->regbase + dcfg->src_reg);

	return 0;
}

static int neutron_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct neutron_rproc *priv;
	struct rproc *rproc;
	const struct imx_rproc_dcfg *dcfg;
	int ret;

	/* set some other name then imx */
	rproc = rproc_alloc(dev, "neutron-rproc", &neutron_rproc_ops,
			    NULL, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	dcfg = of_device_get_match_data(dev);
	if (!dcfg) {
		ret = -EINVAL;
		goto err_put_rproc;
	}

	rproc->auto_boot = false;
	rproc->sysfs_read_only = true;

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dcfg = dcfg;
	priv->dev = dev;

	priv->regbase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regbase)) {
		ret = PTR_ERR(priv->regbase);
		dev_err(&pdev->dev, "failed on platform_ioremap\n");
		goto err_put_rproc;
	}

	dev_set_drvdata(dev, rproc);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "failed to get pm: %d\n", ret);
		goto err_dis_pm;
	}

	ret = neutron_rproc_addr_init(priv, pdev);
	if (ret) {
		dev_err(dev, "failed on neutron_rproc_addr_init\n");
		goto err_put_pm;
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto err_put_pm;
	}

	pm_runtime_put_sync(dev);
	return 0;

err_put_pm:
	pm_runtime_put_sync(dev);
err_dis_pm:
	pm_runtime_disable(dev);
err_put_rproc:
	rproc_free(rproc);

	return ret;
}

static int neutron_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id neutron_rproc_of_match[] = {
	{ .compatible = "fsl,imx95-neutron-rproc", .data = &neutron_rproc_cfg_imx95 },
	{},
};
MODULE_DEVICE_TABLE(of, neutron_rproc_of_match);

static struct platform_driver neutron_rproc_driver = {
	.probe = neutron_rproc_probe,
	.remove = neutron_rproc_remove,
	.driver = {
		.name = "neutron_rproc",
		.of_match_table = neutron_rproc_of_match,
	},
};

module_platform_driver(neutron_rproc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i.MX remote processor neutron driver");
MODULE_AUTHOR("Feng Guo <feng.guo@nxp.com>");
