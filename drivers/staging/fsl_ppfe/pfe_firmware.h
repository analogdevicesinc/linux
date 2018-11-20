/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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
