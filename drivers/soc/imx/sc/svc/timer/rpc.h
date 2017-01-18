/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the TIMER RPC implementation.
 *
 * @addtogroup TIMER_SVC
 * @{
 */

#ifndef _SC_TIMER_RPC_H
#define _SC_TIMER_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC TIMER function calls.
 */
typedef enum timer_func_e {
	TIMER_FUNC_UNKNOWN = 0,	/* Unknown function */
	TIMER_FUNC_SET_WDOG_TIMEOUT = 1,	/* Index for timer_set_wdog_timeout() RPC call */
	TIMER_FUNC_SET_WDOG_PRE_TIMEOUT = 12,	/* Index for timer_set_wdog_pre_timeout() RPC call */
	TIMER_FUNC_START_WDOG = 2,	/* Index for timer_start_wdog() RPC call */
	TIMER_FUNC_STOP_WDOG = 3,	/* Index for timer_stop_wdog() RPC call */
	TIMER_FUNC_PING_WDOG = 4,	/* Index for timer_ping_wdog() RPC call */
	TIMER_FUNC_GET_WDOG_STATUS = 5,	/* Index for timer_get_wdog_status() RPC call */
	TIMER_FUNC_PT_GET_WDOG_STATUS = 13,	/* Index for timer_pt_get_wdog_status() RPC call */
	TIMER_FUNC_SET_WDOG_ACTION = 10,	/* Index for timer_set_wdog_action() RPC call */
	TIMER_FUNC_SET_RTC_TIME = 6,	/* Index for timer_set_rtc_time() RPC call */
	TIMER_FUNC_GET_RTC_TIME = 7,	/* Index for timer_get_rtc_time() RPC call */
	TIMER_FUNC_GET_RTC_SEC1970 = 9,	/* Index for timer_get_rtc_sec1970() RPC call */
	TIMER_FUNC_SET_RTC_ALARM = 8,	/* Index for timer_set_rtc_alarm() RPC call */
	TIMER_FUNC_SET_RTC_CALB = 11,	/* Index for timer_set_rtc_calb() RPC call */
} timer_func_t;

/* Functions */

/*!
 * This function dispatches an incoming TIMER RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void timer_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an TIMER RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void timer_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_TIMER_RPC_H */

/**@}*/
