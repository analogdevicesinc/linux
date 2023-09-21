/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */

#include <linux/platform_device.h>

#define NETCCLKCR_FRAC		GENMASK(31, 0)
#define NETCCLKCR_PERIOD	GENMASK(25, 16)
#define NETCCLKCR_FREQ		GENMASK(10, 0)

#if IS_ENABLED(CONFIG_FSL_NETC_PRB_IERB)

int netc_ierb_get_init_status(void);
u64 netc_ierb_get_clk_config(void);

#else

static inline int netc_ierb_get_init_status(void)
{
	return -ENODEV;
}

static inline u64 netc_ierb_get_clk_config(void)
{
	return 0;
}

#endif
