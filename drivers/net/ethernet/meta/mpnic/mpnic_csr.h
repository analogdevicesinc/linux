/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) Meta Platforms, Inc. and affiliates. */

#ifndef _MPNIC_CSR_H_
#define _MPNIC_CSR_H_

#include <linux/bits.h>

#define CSR_BIT(nr)		BIT_ULL(nr)
#define CSR_GENMASK(h, l)	GENMASK_ULL(h, l)

/* Register Definitions
 *
 * The register file is addressed as an array of le32, so the byte address of
 * a register is 4 times the index below. Each register is listed with its
 * name, index and byte address.
 *
 *	Name				Index			Address
 *****************************************************************************/

/* NIC_CORE_RBP_HP_GLBL */
#define MPNIC_BDQ_SPARE			0x42013e		/* 0x10804f8 */

#endif /* _MPNIC_CSR_H_ */
