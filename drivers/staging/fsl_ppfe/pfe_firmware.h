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

#ifndef _PFE_FIRMWARE_H_
#define _PFE_FIRMWARE_H_

#define CLASS_FIRMWARE_FILENAME		"ppfe_class_ls1012a.elf"
#define TMU_FIRMWARE_FILENAME		"ppfe_tmu_ls1012a.elf"

#define PFE_FW_CHECK_PASS		0
#define PFE_FW_CHECK_FAIL		1
#define NUM_PFE_FW				3

int pfe_firmware_init(struct pfe *pfe);
void pfe_firmware_exit(struct pfe *pfe);

#endif /* _PFE_FIRMWARE_H_ */
