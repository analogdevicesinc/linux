/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ADI System Protection Unit (SPU) consumer interface.
 *
 * TODO: replace with a proper drivers/bus/adi_spu.c driver.
 * On SC59x the SPU is inaccessible from EL1 and requires an SMC call (optee?).
 *
 * (C) Copyright 2026 - Analog Devices, Inc.
 */

#ifndef __LINUX_SOC_ADI_SPU_H
#define __LINUX_SOC_ADI_SPU_H

#include <linux/types.h>

/**
 * adi_spu_set_securep() - mark a peripheral as secure or non-secure
 * @peripheral_id: SPU peripheral index
 * @secure:        true to grant secure-master access, false to revoke
 *
 * Currently a no-op: the SPU driver does not exist yet.
 */
static inline int adi_spu_set_securep(u16 peripheral_id, bool secure)
{
	return 0;
}

#endif /* __LINUX_SOC_ADI_SPU_H */
