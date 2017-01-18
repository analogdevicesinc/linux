/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the IRQ RPC implementation.
 *
 * @addtogroup IRQ_SVC
 * @{
 */

#ifndef _SC_IRQ_RPC_H
#define _SC_IRQ_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC IRQ function calls.
 */
typedef enum irq_func_e {
	IRQ_FUNC_UNKNOWN = 0,	/* Unknown function */
	IRQ_FUNC_ENABLE = 1,	/* Index for irq_enable() RPC call */
	IRQ_FUNC_STATUS = 2,	/* Index for irq_status() RPC call */
} irq_func_t;

/* Functions */

/*!
 * This function dispatches an incoming IRQ RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void irq_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an IRQ RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void irq_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_IRQ_RPC_H */

/**@}*/
