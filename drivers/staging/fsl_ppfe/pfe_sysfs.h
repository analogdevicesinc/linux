/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_SYSFS_H_
#define _PFE_SYSFS_H_

#include <linux/proc_fs.h>

u32 qm_read_drop_stat(u32 tmu, u32 queue, u32 *total_drops, int do_reset);

int pfe_sysfs_init(struct pfe *pfe);
void pfe_sysfs_exit(struct pfe *pfe);

#endif /* _PFE_SYSFS_H_ */
