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

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_clock.h>
#include <linux/slab.h>

#include <soc/imx8/sc/sci.h>

#include "pm-domain-imx8.h"

static sc_ipc_t pm_ipc_handle;

static int imx8_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct imx8_pm_domain *pd;
	sc_err_t sci_err;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	if (pd->rsrc_id == SC_R_LAST)
		return 0;

	sci_err = sc_pm_set_resource_power_mode(pm_ipc_handle, pd->rsrc_id,
		(power_on) ? SC_PM_PW_MODE_ON : SC_PM_PW_MODE_OFF);

	if (sci_err)
		pr_err("Failed power operation on resource %d\n", pd->rsrc_id);

	return 0;
}

static int imx8_pd_power_on(struct generic_pm_domain *domain)
{
	return imx8_pd_power(domain, true);
}

static int imx8_pd_power_off(struct generic_pm_domain *domain)
{
	return imx8_pd_power(domain, false);
}

static int __init imx8_add_pm_domains(struct device_node *parent,
					struct generic_pm_domain *genpd_parent)
{
	struct device_node *np;

	for_each_child_of_node(parent, np) {
		struct imx8_pm_domain *imx8_pd;
		sc_rsrc_t rsrc_id;

		imx8_pd = kzalloc(sizeof(*imx8_pd), GFP_KERNEL);
		if (!imx8_pd)
			return -ENOMEM;

		if (!of_property_read_string(np, "name", &imx8_pd->pd.name))
			imx8_pd->name = imx8_pd->pd.name;

		if (!of_property_read_u32(np, "reg", &rsrc_id))
			imx8_pd->rsrc_id = rsrc_id;

		imx8_pd->pd.power_off = imx8_pd_power_off;
		imx8_pd->pd.power_on = imx8_pd_power_on;

		pm_genpd_init(&imx8_pd->pd, NULL, true);

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

	return 0;
}

early_initcall(imx8_init_pm_domains);
