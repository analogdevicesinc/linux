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

spinlock_t imx_ccm_lock;
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
