/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the PM RPC implementation.
 *
 * @addtogroup PM_SVC
 * @{
 */

#ifndef _SC_PM_RPC_H
#define _SC_PM_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC PM function calls.
 */
typedef enum pm_func_e {
	PM_FUNC_UNKNOWN = 0,	/* Unknown function */
	PM_FUNC_SET_SYS_POWER_MODE = 19,	/* Index for pm_set_sys_power_mode() RPC call */
	PM_FUNC_SET_PARTITION_POWER_MODE = 1,	/* Index for pm_set_partition_power_mode() RPC call */
	PM_FUNC_GET_SYS_POWER_MODE = 2,	/* Index for pm_get_sys_power_mode() RPC call */
	PM_FUNC_SET_RESOURCE_POWER_MODE = 3,	/* Index for pm_set_resource_power_mode() RPC call */
	PM_FUNC_GET_RESOURCE_POWER_MODE = 4,	/* Index for pm_get_resource_power_mode() RPC call */
	PM_FUNC_REQ_LOW_POWER_MODE = 16,	/* Index for pm_req_low_power_mode() RPC call */
	PM_FUNC_SET_CPU_RESUME_ADDR = 17,	/* Index for pm_set_cpu_resume_addr() RPC call */
	PM_FUNC_REQ_SYS_IF_POWER_MODE = 18,	/* Index for pm_req_sys_if_power_mode() RPC call */
	PM_FUNC_SET_CLOCK_RATE = 5,	/* Index for pm_set_clock_rate() RPC call */
	PM_FUNC_GET_CLOCK_RATE = 6,	/* Index for pm_get_clock_rate() RPC call */
	PM_FUNC_CLOCK_ENABLE = 7,	/* Index for pm_clock_enable() RPC call */
	PM_FUNC_SET_CLOCK_PARENT = 14,	/* Index for pm_set_clock_parent() RPC call */
	PM_FUNC_GET_CLOCK_PARENT = 15,	/* Index for pm_get_clock_parent() RPC call */
	PM_FUNC_RESET = 13,	/* Index for pm_reset() RPC call */
	PM_FUNC_RESET_REASON = 10,	/* Index for pm_reset_reason() RPC call */
	PM_FUNC_BOOT = 8,	/* Index for pm_boot() RPC call */
	PM_FUNC_REBOOT = 9,	/* Index for pm_reboot() RPC call */
	PM_FUNC_REBOOT_PARTITION = 12,	/* Index for pm_reboot_partition() RPC call */
	PM_FUNC_CPU_START = 11,	/* Index for pm_cpu_start() RPC call */
} pm_func_t;

/* Functions */

/*!
 * This function dispatches an incoming PM RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void pm_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an PM RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void pm_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_PM_RPC_H */

/**@}*/
