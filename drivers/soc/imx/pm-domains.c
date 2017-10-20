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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_clock.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>

#include <soc/imx8/sc/sci.h>

#include "pm-domain-imx8.h"

static sc_ipc_t pm_ipc_handle;
static sc_rsrc_t early_power_on_rsrc[] = {
	SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST,
	SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST,
};
static sc_rsrc_t rsrc_debug_console;

static int imx8_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct imx8_pm_domain *pd;
	sc_err_t sci_err = SC_ERR_NONE;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	if (pd->rsrc_id == SC_R_LAST)
		return 0;

	/* keep uart console power on for no_console_suspend */
	if (pd->rsrc_id == rsrc_debug_console &&
		!console_suspend_enabled && !power_on)
		return 0;

	sci_err = sc_pm_set_resource_power_mode(pm_ipc_handle, pd->rsrc_id,
		(power_on) ? SC_PM_PW_MODE_ON :
		(pd->runtime_idle_active) ? SC_PM_PW_MODE_LP : SC_PM_PW_MODE_OFF);
	if (sci_err)
		pr_err("Failed power operation on resource %d\n", pd->rsrc_id);

	return 0;
}

static int imx8_pd_power_on(struct generic_pm_domain *domain)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk;
	int ret = 0;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	ret = imx8_pd_power(domain, true);

	if (!list_empty(&pd->clks) && domain->status == GPD_STATE_POWER_OFF
		&& !pd->runtime_idle_active) {
		/*
		 * The SS is powered on restore the clock rates that
		 * may be lost.
		 */
		list_for_each_entry(imx8_rsrc_clk, &pd->clks, node) {

			if (imx8_rsrc_clk->parent)
				clk_set_parent(imx8_rsrc_clk->clk,
					imx8_rsrc_clk->parent);

			if (imx8_rsrc_clk->rate) {
				/*
				 * Need to read the clock so that rate in
				 * Linux is reset.
				 */
				clk_get_rate(imx8_rsrc_clk->clk);
				/* Restore the clock rate. */
				clk_set_rate(imx8_rsrc_clk->clk,
					imx8_rsrc_clk->rate);
			}
		}
	}
	pd->runtime_idle_active = false;

	return ret;
}

static int imx8_pd_power_off(struct generic_pm_domain *domain)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	if (!list_empty(&pd->clks) && (domain->status != GPD_STATE_POWER_OFF)
		&& (!pd->runtime_idle_active)) {
		/*
		 * The SS is going to be powered off, store the clock rates
		 * that may be lost.
		 */
		list_for_each_entry(imx8_rsrc_clk, &pd->clks, node) {
			imx8_rsrc_clk->parent = clk_get_parent(imx8_rsrc_clk->clk);
			imx8_rsrc_clk->rate = clk_hw_get_rate(__clk_get_hw(imx8_rsrc_clk->clk));
		}
	}
	return imx8_pd_power(domain, false);
}

static int imx8_pd_dev_start(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct imx8_pm_domain *pd;

	pd = container_of(genpd, struct imx8_pm_domain, pd);
	pd->runtime_idle_active = false;
	return 0;
}

static int imx8_pd_dev_stop(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct imx8_pm_domain *pd;

	pd = container_of(genpd, struct imx8_pm_domain, pd);
	if (pm_runtime_enabled(dev))
		pd->runtime_idle_active = true;
	return 0;
}

static int imx8_pm_runtime_idle(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct imx8_pm_domain *pd;

	pd = container_of(genpd, struct imx8_pm_domain, pd);
	if (pm_runtime_enabled(dev))
		pd->runtime_idle_active = true;
	return pm_runtime_autosuspend(dev);
}

static int imx8_attach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	struct imx8_pm_domain *pd;
	struct device_node *node = dev->of_node;
	struct of_phandle_args clkspec;
	int rc, index, num_clks;

	pd = container_of(genpd, struct imx8_pm_domain, pd);
	INIT_LIST_HEAD(&pd->clks);

	num_clks = of_count_phandle_with_args(node, "assigned-clocks",
						"#clock-cells");
	if (num_clks == -EINVAL)
		pr_err("%s: Invalid value of assigned-clocks property at %s\n",
			pd->name, node->full_name);

	for (index = 0; index < num_clks; index++) {
		struct imx8_pm_rsrc_clks *imx8_rsrc_clk;

		rc = of_parse_phandle_with_args(node, "assigned-clocks",
					"#clock-cells", index, &clkspec);
		if (rc < 0) {
			/* skip empty (null) phandles */
			if (rc == -ENOENT)
				continue;
			else
				return rc;
		}
		if (clkspec.np == node)
			return 0;

		imx8_rsrc_clk = devm_kzalloc(dev, sizeof(*imx8_rsrc_clk),
					GFP_KERNEL);
		if (!imx8_rsrc_clk)
			return -ENOMEM;

		imx8_rsrc_clk->clk = of_clk_get_from_provider(&clkspec);
		if (!IS_ERR(imx8_rsrc_clk->clk))
			list_add_tail(&imx8_rsrc_clk->node, &pd->clks);
	}
	return 0;
}

static void imx8_detach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk, *tmp;

	pd = container_of(genpd, struct imx8_pm_domain, pd);

	/* Free all the clock entry nodes. */
	if (list_empty(&pd->clks))
		return;

	list_for_each_entry_safe(imx8_rsrc_clk, tmp, &pd->clks, node) {
		list_del(&imx8_rsrc_clk->node);
		devm_kfree(dev, imx8_rsrc_clk);
	}
}

static void imx8_pm_domains_resume(void)
{
	sc_err_t sci_err = SC_ERR_NONE;
	int i;

	for (i = 0; i < (sizeof(early_power_on_rsrc) /
		sizeof(sc_rsrc_t)); i++) {
		if (early_power_on_rsrc[i] != SC_R_LAST) {
			sci_err = sc_pm_set_resource_power_mode(pm_ipc_handle,
				early_power_on_rsrc[i], SC_PM_PW_MODE_ON);
			if (sci_err != SC_ERR_NONE)
				pr_err("fail to power on resource %d\n",
					early_power_on_rsrc[i]);
		}
	}
}

struct syscore_ops imx8_pm_domains_syscore_ops = {
	.resume = imx8_pm_domains_resume,
};

static int __init imx8_add_pm_domains(struct device_node *parent,
					struct generic_pm_domain *genpd_parent)
{
	struct device_node *np;
	static int index;

	for_each_child_of_node(parent, np) {
		struct imx8_pm_domain *imx8_pd;
		sc_rsrc_t rsrc_id;

		if (!of_device_is_available(np))
			continue;

		imx8_pd = kzalloc(sizeof(*imx8_pd), GFP_KERNEL);
		if (!imx8_pd)
			return -ENOMEM;

		if (!of_property_read_string(np, "name", &imx8_pd->pd.name))
			imx8_pd->name = imx8_pd->pd.name;

		if (!of_property_read_u32(np, "reg", &rsrc_id))
			imx8_pd->rsrc_id = rsrc_id;

		if (imx8_pd->rsrc_id != SC_R_LAST) {
			imx8_pd->pd.power_off = imx8_pd_power_off;
			imx8_pd->pd.power_on = imx8_pd_power_on;
			imx8_pd->pd.dev_ops.start = imx8_pd_dev_start;
			imx8_pd->pd.dev_ops.stop = imx8_pd_dev_stop;
			imx8_pd->pd.attach_dev = imx8_attach_dev;
			imx8_pd->pd.detach_dev = imx8_detach_dev;

			if (of_property_read_bool(np, "early_power_on")
				&& index < (sizeof(early_power_on_rsrc) /
				sizeof(sc_rsrc_t))) {
				early_power_on_rsrc[index++] = imx8_pd->rsrc_id;
			}
			if (of_property_read_bool(np, "debug_console"))
				rsrc_debug_console = imx8_pd->rsrc_id;
		}
		INIT_LIST_HEAD(&imx8_pd->clks);
		pm_genpd_init(&imx8_pd->pd, NULL, true);

		imx8_pd->pd.domain.ops.runtime_idle = imx8_pm_runtime_idle;

		if (genpd_parent)
			pm_genpd_add_subdomain(genpd_parent, &imx8_pd->pd);

		of_genpd_add_provider_simple(np, &imx8_pd->pd);

		imx8_add_pm_domains(np, &imx8_pd->pd);
	}
	return 0;
}

static int __init imx8_init_pm_domains(void)
{
	struct device_node *np;
	sc_err_t sci_err;
	sc_rsrc_t rsrc_id;
	uint32_t mu_id;

	/* skip pm domains for non-SCFW system */
	if (!of_find_compatible_node(NULL, NULL, "nxp,imx8-pd"))
		return 0;

	pr_info("***** imx8_init_pm_domains *****\n");

	for_each_compatible_node(np, NULL, "nxp,imx8-pd") {
		struct imx8_pm_domain *imx8_pd;

		if (!of_device_is_available(np))
			continue;

		imx8_pd = kzalloc(sizeof(struct imx8_pm_domain), GFP_KERNEL);
		if (!imx8_pd) {
			pr_err("%s: failed to allocate memory for domain\n",
				__func__);
			return -ENOMEM;
		}
		if (!of_property_read_string(np, "name", &imx8_pd->pd.name))
			imx8_pd->name = imx8_pd->pd.name;

		if (!of_property_read_u32(np, "reg", &rsrc_id))
			imx8_pd->rsrc_id = rsrc_id;

		if (imx8_pd->rsrc_id != SC_R_LAST) {
			imx8_pd->pd.power_off = imx8_pd_power_off;
			imx8_pd->pd.power_on = imx8_pd_power_on;
		}
		INIT_LIST_HEAD(&imx8_pd->clks);

		pm_genpd_init(&imx8_pd->pd, NULL, true);
		of_genpd_add_provider_simple(np, &imx8_pd->pd);
		imx8_add_pm_domains(np, &imx8_pd->pd);
	}

	sci_err = sc_ipc_getMuID(&mu_id);
	if (sci_err != SC_ERR_NONE) {
		pr_info("Cannot obtain MU ID\n");
		return sci_err;
	}

	sci_err = sc_ipc_open(&pm_ipc_handle, mu_id);
	register_syscore_ops(&imx8_pm_domains_syscore_ops);

	return 0;
}

early_initcall(imx8_init_pm_domains);
