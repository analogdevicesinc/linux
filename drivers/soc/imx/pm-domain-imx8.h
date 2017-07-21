/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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

#ifndef PM_DOMAIN_IMX8_H
#define PM_DOMAIN_IMX8_H

#include <linux/pm_domain.h>
#include <soc/imx8/sc/sci.h>

#define DEFAULT_DEV_LATENCY_NS	250000

struct platform_device;

struct imx8_pm_domain {
	const char *name;
	struct generic_pm_domain pd;
	struct dev_power_governor *gov;
	int (*suspend)(void);
	void (*resume)(void);
	sc_rsrc_t rsrc_id;
	bool runtime_idle_active;
};

static inline
struct imx8_pm_domain *to_imx8_pd(struct generic_pm_domain *d)
{
	return container_of(d, struct imx8_pm_domain, pd);
}

struct pm_domain_device {
	const char *domain_name;
	struct platform_device *pdev;
};

#endif /* PM_DOMAIN_IMX8_H */
