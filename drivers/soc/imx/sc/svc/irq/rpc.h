/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the IRQ RPC implementation.
 *
 * @addtogroup IRQ_SVC
 * @{
 */

#ifndef SC_IRQ_RPC_H
#define SC_IRQ_RPC_H

/* Includes */

/* Defines */

/*!
 * @name Defines for RPC IRQ function calls
 */
/*@{*/
#define IRQ_FUNC_UNKNOWN 0	/* Unknown function */
#define IRQ_FUNC_ENABLE 1U	/* Index for irq_enable() RPC call */
#define IRQ_FUNC_STATUS 2U	/* Index for irq_status() RPC call */
/*@}*/

/* Types */

/* Functions */

/*!
 * This function dispatches an incoming IRQ RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void irq_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

#endif				/* SC_IRQ_RPC_H */

/**@}*/
