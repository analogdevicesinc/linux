/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __SMC_SIP_H__
#define __SMC_SIP_H__

#include <linux/arm-smccc.h>

/*
 * Macro definition building the OPTEE SMC Code function
 * for a Fast Call, SIP operation
 */
#define OPTEE_SMC_FAST_CALL_SIP_VAL(func_num) \
							ARM_SMCCC_CALL_VAL( \
							ARM_SMCCC_FAST_CALL, \
							ARM_SMCCC_SMC_32, \
							ARM_SMCCC_OWNER_SIP, \
							(func_num))


/*
 * Definition of the i.MX SMC SIP Operations
 * Operation value must be aligned with i.MX OPTEE
 * SIP definitions
 */
/* Busfreq operation */
#define IMX_SIP_BUSFREQ_CHANGE             6

#endif /* __SMC_SIP_H__ */

