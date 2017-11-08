/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_domain.h>
#include <soc/imx/fsl_sip.h>

#define GPC_MAX_IRQS		(4 * 32)

struct imx_gpc_pm_domain {
	const char name[30];
	struct device *dev;
	struct generic_pm_domain pd;
	u32 gpc_domain_id;
	struct clk **clks;
	unsigned int num_clks;
	struct regulator *reg;
};

enum imx_gpc_pm_domain_state {
	GPC_PD_STATE_OFF,
	GPC_PD_STATE_ON,
};

#define to_imx_gpc_pm_domain(_genpd) container_of(_genpd, struct imx_gpc_pm_domain, pd)

static DEFINE_SPINLOCK(gpc_psci_lock);

static void imx_gpc_psci_irq_unmask(struct irq_data *d)
{
	struct arm_smccc_res res;

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_UNMASK, d->hwirq,
		      0, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	irq_chip_unmask_parent(d);
}

static void imx_gpc_psci_irq_mask(struct irq_data *d)
{
	struct arm_smccc_res res;

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_MASK, d->hwirq,
		      0, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	irq_chip_mask_parent(d);
}
static int imx_gpc_psci_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct arm_smccc_res res;

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_SET_WAKE, d->hwirq,
		      on, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	return 0;
}

static int imx_gpc_psci_irq_set_affinity(struct irq_data *d,
					 const struct cpumask *dest, bool force)
{
	/* parse the cpu of irq affinity */
	struct arm_smccc_res res;
	unsigned int cpu = cpumask_any_and(dest, cpu_online_mask);

	irq_chip_set_affinity_parent(d, dest, force);

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, 0x4, d->hwirq,
		      cpu, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	return 0;
}

static struct irq_chip imx_gpc_psci_chip = {
	.name			= "GPC-PSCI",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= imx_gpc_psci_irq_mask,
	.irq_unmask		= imx_gpc_psci_irq_unmask,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_wake		= imx_gpc_psci_irq_set_wake,
	.irq_set_affinity	= imx_gpc_psci_irq_set_affinity,
};

static int imx_gpc_psci_domain_translate(struct irq_domain *d,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;

		/* No PPI should point to this domain */
		if (fwspec->param[0] != 0)
			return -EINVAL;

		*hwirq = fwspec->param[1];
		*type = fwspec->param[2];
		return 0;
	}

	return -EINVAL;
}

static int imx_gpc_psci_domain_alloc(struct irq_domain *domain,
				  unsigned int irq,
				  unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	int i;

	if (fwspec->param_count != 3)
		return -EINVAL;	/* Not GIC compliant */
	if (fwspec->param[0] != 0)
		return -EINVAL;	/* No PPI should point to this domain */

	hwirq = fwspec->param[1];
	if (hwirq >= GPC_MAX_IRQS)
		return -EINVAL;	/* Can't deal with this */

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, irq + i, hwirq + i,
					      &imx_gpc_psci_chip, NULL);

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;

	return irq_domain_alloc_irqs_parent(domain, irq, nr_irqs,
					    &parent_fwspec);
}

static struct irq_domain_ops imx_gpc_psci_domain_ops = {
	.translate = imx_gpc_psci_domain_translate,
	.alloc	= imx_gpc_psci_domain_alloc,
	.free	= irq_domain_free_irqs_common,
};

static int __init imx_gpc_psci_init(struct device_node *node,
			       struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;

	if (!parent) {
		pr_err("%s: no parent, giving up\n", node->full_name);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%s: unable to obtain parent domain\n", node->full_name);
		return -ENXIO;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, GPC_MAX_IRQS,
					  node, &imx_gpc_psci_domain_ops,
					  NULL);
	if (!domain)
		return -ENOMEM;

	return 0;
}
IRQCHIP_DECLARE(imx_gpc_psci, "fsl,imx8mq-gpc", imx_gpc_psci_init);

static int imx_gpc_pd_power_on(struct generic_pm_domain *domain)
{
	struct imx_gpc_pm_domain *pd = to_imx_gpc_pm_domain(domain);
	struct arm_smccc_res res;
	int index, ret = 0;

	/* power on the external supply */
	if (!IS_ERR(pd->reg)) {
		ret = regulator_enable(pd->reg);
		if (ret) {
			dev_warn(pd->dev, "failed to power up the reg%d\n", ret);
			return ret;
		}
	}

	/* enable the necessary clks needed by the power domain */
	if (pd->num_clks) {
		for (index = 0; index < pd->num_clks; index++)
			clk_prepare_enable(pd->clks[index]);
	}

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, pd->gpc_domain_id,
		      GPC_PD_STATE_ON, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	return 0;
}

static int imx_gpc_pd_power_off(struct generic_pm_domain *domain)
{
	struct imx_gpc_pm_domain *pd = to_imx_gpc_pm_domain(domain);
	struct arm_smccc_res res;
	int index, ret = 0;

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, pd->gpc_domain_id,
		      GPC_PD_STATE_OFF, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	/* power off the external supply */
	if (!IS_ERR(pd->reg)) {
		ret = regulator_disable(pd->reg);
		if (ret) {
			dev_warn(pd->dev, "failed to power off the reg%d\n", ret);
			return ret;
		}
	}

	/* disable the necessary clks when power domain on finished */
	if (pd->num_clks) {
		for (index = 0; index < pd->num_clks; index++)
			clk_disable_unprepare(pd->clks[index]);
	}

	return ret;
};

static int imx_gpc_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_gpc_pm_domain *imx_pm_domain;
	struct property *pp;
	struct clk **clks;
	int index, clk_num, ret;

	if (!np) {
		dev_err(dev, "power domain device tree node not found\n");
		return -ENODEV;
	}

	imx_pm_domain = devm_kzalloc(dev, sizeof(*imx_pm_domain), GFP_KERNEL);
	if (!imx_pm_domain)
		return -ENOMEM;
	imx_pm_domain->dev = dev;

	ret = of_property_read_string(np, "domain-name", &imx_pm_domain->pd.name);
	if (ret) {
		dev_err(dev, "get domain name failed\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "domain-id", &imx_pm_domain->gpc_domain_id);
	if (ret) {
		dev_err(dev, "get domain id failed\n");
		return -EINVAL;
	}

	imx_pm_domain->reg = devm_regulator_get_optional(dev, "power");
	if (PTR_ERR(imx_pm_domain->reg) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	pp = of_find_property(np, "clocks", NULL);
	if (pp) {
		clk_num = pp->length / 8;
		imx_pm_domain->num_clks = clk_num;
	} else {
		clk_num = 0;
		imx_pm_domain->num_clks = clk_num;
	}

	if (clk_num) {
		clks = devm_kcalloc(dev, clk_num, sizeof(*clks), GFP_KERNEL);
		imx_pm_domain->clks = clks;
		for (index = 0; index < clk_num; index++) {
			clks[index] = of_clk_get(np, index);
			if (IS_ERR(clks[index])) {
				ret = -ENODEV;
				goto exit;
			}
		}
	}

	imx_pm_domain->pd.power_off = imx_gpc_pd_power_off;
	imx_pm_domain->pd.power_on = imx_gpc_pd_power_on;
	/* all power domains as off at boot */
	pm_genpd_init(&imx_pm_domain->pd, NULL, true);

	ret = of_genpd_add_provider_simple(np,
				 &imx_pm_domain->pd);
	return ret;

exit:
	for (index = 0; index < clk_num; index++) {
		if (!IS_ERR(clks[index]))
			clk_put(clks[index]);
	}

	return ret;
}

static const struct of_device_id imx_gpc_pm_domain_ids[] = {
	{.compatible = "fsl,imx8mq-pm-domain"},
	{},
};

static struct platform_driver imx_gpc_pm_domain_driver = {
	.driver = {
		.name	= "imx8m_gpc_pm_domain",
		.owner	= THIS_MODULE,
		.of_match_table = imx_gpc_pm_domain_ids,
	},
	.probe = imx_gpc_pm_domain_probe,
};

module_platform_driver(imx_gpc_pm_domain_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP i.MX8M GPC power domain driver");
MODULE_LICENSE("GPL v2");
