// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>

#include <soc/imx/imx_sip.h>

#define MAX_CLK_NUM	6
#define to_imx8m_pm_domain(_genpd) container_of(_genpd, struct imx8m_pm_domain, pd)


struct imx8m_pm_domain {
	struct device *dev;
	struct generic_pm_domain pd;
	u32 domain_index;
	struct clk *clk[MAX_CLK_NUM];
	unsigned int num_clks;
	struct regulator *reg;
};

enum imx8m_pm_domain_state {
	PD_STATE_OFF,
	PD_STATE_ON,
};

static DEFINE_MUTEX(gpc_pd_mutex);

static int imx8m_pd_power_on(struct generic_pm_domain *genpd)
{
	struct imx8m_pm_domain *domain = to_imx8m_pm_domain(genpd);
	struct arm_smccc_res res;
	int index, ret = 0;

	/* power on the external supply */
	if (!IS_ERR(domain->reg)) {
		ret = regulator_enable(domain->reg);
		if (ret) {
			dev_warn(domain->dev, "failed to power up the reg%d\n", ret);
			return ret;
		}
	}

	/* enable the necessary clks needed by the power domain */
	if (domain->num_clks) {
		for (index = 0; index < domain->num_clks; index++)
			clk_prepare_enable(domain->clk[index]);
	}

	mutex_lock(&gpc_pd_mutex);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, domain->domain_index,
		      PD_STATE_ON, 0, 0, 0, 0, &res);
	mutex_unlock(&gpc_pd_mutex);

	return 0;
}

static int imx8m_pd_power_off(struct generic_pm_domain *genpd)
{
	struct imx8m_pm_domain *domain = to_imx8m_pm_domain(genpd);
	struct arm_smccc_res res;
	int index, ret = 0;

	mutex_lock(&gpc_pd_mutex);
	arm_smccc_smc(IMX_SIP_GPC, IMX_SIP_CONFIG_GPC_PM_DOMAIN, domain->domain_index,
		      PD_STATE_OFF, 0, 0, 0, 0, &res);
	mutex_unlock(&gpc_pd_mutex);

	/* power off the external supply */
	if (!IS_ERR(domain->reg)) {
		ret = regulator_disable(domain->reg);
		if (ret) {
			dev_warn(domain->dev, "failed to power off the reg%d\n", ret);
			return ret;
		}
	}

	/* disable clks when power domain is off */
	if (domain->num_clks) {
		for (index = 0; index < domain->num_clks; index++)
			clk_disable_unprepare(domain->clk[index]);
	}

	return ret;
};

static int imx8m_pd_get_clocks(struct imx8m_pm_domain *domain)
{
	int i, ret;

	for (i = 0; ; i++) {
		struct clk *clk = of_clk_get(domain->dev->of_node, i);
		if (IS_ERR(clk))
			break;
		if (i >= MAX_CLK_NUM) {
			dev_err(domain->dev, "more than %d clocks\n",
				MAX_CLK_NUM);
			ret = -EINVAL;
			goto clk_err;
		}
		domain->clk[i] = clk;
	}
	domain->num_clks = i;

	return 0;

clk_err:
	while (i--)
		clk_put(domain->clk[i]);

	return ret;
}

static void imx8m_pd_put_clocks(struct imx8m_pm_domain *domain)
{
	int i;

	for (i = domain->num_clks - 1; i >= 0; i--)
		clk_put(domain->clk[i]);
}

static const struct of_device_id imx8m_pm_domain_ids[] = {
	{.compatible = "fsl,imx8m-pm-domain"},
	{},
};

static int imx8m_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx8m_pm_domain *domain;
	struct of_phandle_args parent, child;
	int ret;

	domain = devm_kzalloc(dev, sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return -ENOMEM;

	child.np = np;
	domain->dev = dev;

	ret = of_property_read_string(np, "domain-name", &domain->pd.name);
	if (ret) {
		dev_err(dev, "failed to get the domain name\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "domain-index", &domain->domain_index);
	if (ret) {
		dev_err(dev, "failed to get the domain index\n");
		return -EINVAL;
	}

	domain->reg = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(domain->reg)) {
		if (PTR_ERR(domain->reg) != -ENODEV) {
			if (PTR_ERR(domain->reg) != -EPROBE_DEFER)
				dev_err(dev, "failed to get domain's regulator\n");
			return PTR_ERR(domain->reg);
		}
	}

	ret = imx8m_pd_get_clocks(domain);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get domain's clocks\n");
		return ret;
	}

	domain->pd.power_off = imx8m_pd_power_off;
	domain->pd.power_on = imx8m_pd_power_on;

	pm_genpd_init(&domain->pd, NULL, true);

	ret = of_genpd_add_provider_simple(np, &domain->pd);
	if (ret) {
		dev_err(dev, "failed to add the domain provider\n");
		pm_genpd_remove(&domain->pd);
		imx8m_pd_put_clocks(domain);
		return ret;
	}

	/* add it as subdomain if necessary */
	if (!of_parse_phandle_with_args(np, "parent-domains",
			"#power-domain-cells", 0, &parent)) {
		ret = of_genpd_add_subdomain(&parent, &child);
		of_node_put(parent.np);

		if (ret < 0) {
			dev_dbg(dev, "failed to add the subdomain: %s: %d",
				domain->pd.name, ret);
			of_genpd_del_provider(np);
			pm_genpd_remove(&domain->pd);
			imx8m_pd_put_clocks(domain);
			return driver_deferred_probe_check_state(dev);
		}
	}

	return 0;
}

static struct platform_driver imx8m_pm_domain_driver = {
	.driver = {
		.name	= "imx8m_pm_domain",
		.owner	= THIS_MODULE,
		.of_match_table = imx8m_pm_domain_ids,
	},
	.probe = imx8m_pm_domain_probe,
};
module_platform_driver(imx8m_pm_domain_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP i.MX8M power domain driver");
MODULE_LICENSE("GPL v2");
