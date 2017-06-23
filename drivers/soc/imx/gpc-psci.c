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
#include <linux/pm_domain.h>
#include <soc/imx/fsl_sip.h>

#define GPC_MAX_IRQS		(4 * 32)

struct imx_gpc_pm_domain {
	char name[30];
	struct generic_pm_domain pd;
	u32 gpc_domain_id;
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

static struct irq_chip imx_gpc_psci_chip = {
	.name			= "GPC-PSCI",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= imx_gpc_psci_irq_mask,
	.irq_unmask		= imx_gpc_psci_irq_unmask,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_wake		= imx_gpc_psci_irq_set_wake,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
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

	spin_lock(&gpc_psci_lock);
	arm_smccc_smc(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, pd->gpc_domain_id,
		      GPC_PD_STATE_OFF, 0, 0, 0, 0, &res);
	spin_unlock(&gpc_psci_lock);

	return 0;
};

static int imx_gpc_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_gpc_pm_domain *imx_pm_domain;
	struct genpd_onecell_data *imx_pd_data;
	struct generic_pm_domain **domains;
	int ret, num_domains, i;

	pr_info("imx8mq pm domain init\n");
	if (!np) {
		dev_err(dev, "device tree node not found\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "num-domains", &num_domains);
	if (ret) {
		dev_err(dev, "number of domains not found\n");
		return -EINVAL;
	};

	imx_pm_domain = devm_kcalloc(dev, num_domains, sizeof(*imx_pm_domain), GFP_KERNEL);
	if (!imx_pm_domain)
		return -ENOMEM;

	imx_pd_data = devm_kzalloc(dev, sizeof(*imx_pd_data), GFP_KERNEL);
	if (!imx_pd_data)
		return -ENOMEM;

	domains = devm_kcalloc(dev, num_domains, sizeof(*domains), GFP_KERNEL);
	if (!domains)
		return -ENOMEM;

	for (i = 0; i < num_domains; i++, imx_pm_domain++) {
		domains[i] = &imx_pm_domain->pd;
		imx_pm_domain->gpc_domain_id = i;
		sprintf(imx_pm_domain->name, "%s.%d", np->name, i);
		imx_pm_domain->pd.name = imx_pm_domain->name;
		imx_pm_domain->pd.power_off = imx_gpc_pd_power_off;
		imx_pm_domain->pd.power_on = imx_gpc_pd_power_on;

		/* all power domains as off at boot */
		pm_genpd_init(&imx_pm_domain->pd, NULL, true);
	}

	imx_pd_data->domains = domains;
	imx_pd_data->num_domains = num_domains;

	of_genpd_add_provider_onecell(np, imx_pd_data);

	return 0;
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
