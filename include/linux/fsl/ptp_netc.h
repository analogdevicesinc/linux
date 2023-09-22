/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2023 NXP
 * Copyright (c) 2023 Wei Fang <wei.fang@nxp.com>
 */
#ifndef _PTP_NETC_H
#define _PTP_NETC_H

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK_NETC)
int netc_timer_get_phc_index(int domain, unsigned int bus, unsigned int devfn);
#else
static inline int netc_timer_get_phc_index(int domain, unsigned int bus,
					   unsigned int devfn)
{
	return -1;
}
#endif

#endif
