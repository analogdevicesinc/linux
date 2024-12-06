/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 */

#ifndef _PFE_HW_H_
#define _PFE_HW_H_

#define PE_SYS_CLK_RATIO	1	/* SYS/AXI = 250MHz, HFE = 500MHz */

int pfe_hw_init(struct pfe *pfe, int resume);
void pfe_hw_exit(struct pfe *pfe);

#endif /* _PFE_HW_H_ */
