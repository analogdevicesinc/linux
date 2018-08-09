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

#include <linux/errno.h>
#include <soc/imx8/sc/sci.h>

#include "clk-imx8.h"

sc_ipc_t ccm_ipc_handle;

int imx8_clk_mu_init(void)
{
	uint32_t mu_id;
	sc_err_t sciErr;

	printk("MU and Power domains initialized\n");

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_info("Cannot obtain MU ID\n");
		return -EPROBE_DEFER;
	}

	sciErr = sc_ipc_open(&ccm_ipc_handle, mu_id);

	if (sciErr != SC_ERR_NONE) {
		pr_info("Cannot open MU channel to SCU\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

bool imx8_clk_is_resource_owned(sc_rsrc_t rsrc)
{
	/*
	 * A-core resources are special. SCFW reports they are not "owned" by
	 * current partition but linux can still adjust them for cpufreq.
	 *
	 * So force this to return false when running as a VM guest and always
	 * true otherwise.
	 */
	if (rsrc == SC_R_A53 || rsrc == SC_R_A72 || rsrc == SC_R_A35) {
		if (xen_domain() && !xen_initial_domain())
			return false;
		return true;
	}

	if (!ccm_ipc_handle) {
		pr_warn("%s: no ipc handle!\n", __func__);
		/* should have handled -EPROBE_DEFER from clk_mu_init earlier. */
		return false;
	}

	return sc_rm_is_resource_owned(ccm_ipc_handle, rsrc);
}
