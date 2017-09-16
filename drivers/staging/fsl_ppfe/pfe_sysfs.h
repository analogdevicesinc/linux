/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PFE_SYSFS_H_
#define _PFE_SYSFS_H_

#include <linux/proc_fs.h>

u32 qm_read_drop_stat(u32 tmu, u32 queue, u32 *total_drops, int do_reset);

int pfe_sysfs_init(struct pfe *pfe);
void pfe_sysfs_exit(struct pfe *pfe);

#endif /* _PFE_SYSFS_H_ */
