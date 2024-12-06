/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_PERFMON_H_
#define _PFE_PERFMON_H_

#include "pfe/pfe.h"

#define	CT_CPUMON_INTERVAL	(1 * TIMER_TICKS_PER_SEC)

struct pfe_cpumon {
	u32 cpu_usage_pct[MAX_PE];
	u32 class_usage_pct;
};

struct pfe_memmon {
	u32 kernel_memory_allocated;
};

int pfe_perfmon_init(struct pfe *pfe);
void pfe_perfmon_exit(struct pfe *pfe);

#endif /* _PFE_PERFMON_H_ */
