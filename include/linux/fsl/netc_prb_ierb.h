/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */

#include <linux/platform_device.h>

#define NETCCLKCR_FRAC		GENMASK(31, 0)
#define NETCCLKCR_PERIOD	GENMASK(25, 16)
#define NETCCLKCR_FREQ		GENMASK(10, 0)

#if IS_ENABLED(CONFIG_FSL_NETC_PRB_IERB)

u64 netc_ierb_get_clk_config(void);
void netc_prb_ierb_register_emdio(struct device *emdio);
int netc_prb_ierb_check_emdio_state(void);
int netc_prb_ierb_add_emdio_consumer(struct device *consumer);

#else

static inline u64 netc_ierb_get_clk_config(void)
{
	return 0;
}

static inline void netc_prb_ierb_register_emdio(struct device *emdio)
{
}

static inline int netc_prb_ierb_check_emdio_state(void)
{
	return 0;
}

static inline int netc_prb_ierb_add_emdio_consumer(struct device *consumer)
{
	return 0;
}

#endif
