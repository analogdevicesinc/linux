/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the TIMER service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup TIMER_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/timer/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_timer_set_wdog_timeout(sc_ipc_t ipc, sc_timer_wdog_time_t timeout)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_WDOG_TIMEOUT;
	RPC_U32(&msg, 0) = timeout;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_set_wdog_pre_timeout(sc_ipc_t ipc,
				       sc_timer_wdog_time_t pre_timeout)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_WDOG_PRE_TIMEOUT;
	RPC_U32(&msg, 0) = pre_timeout;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_start_wdog(sc_ipc_t ipc, bool lock)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_START_WDOG;
	RPC_U8(&msg, 0) = (uint8_t)lock;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_stop_wdog(sc_ipc_t ipc)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_STOP_WDOG;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_ping_wdog(sc_ipc_t ipc)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_PING_WDOG;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_get_wdog_status(sc_ipc_t ipc,
				  sc_timer_wdog_time_t *timeout,
				  sc_timer_wdog_time_t *max_timeout,
				  sc_timer_wdog_time_t *remaining_time)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_GET_WDOG_STATUS;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (timeout != NULL) {
		*timeout = RPC_U32(&msg, 0);
	}

	if (max_timeout != NULL) {
		*max_timeout = RPC_U32(&msg, 4);
	}

	if (remaining_time != NULL) {
		*remaining_time = RPC_U32(&msg, 8);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_pt_get_wdog_status(sc_ipc_t ipc, sc_rm_pt_t pt, bool *enb,
				     sc_timer_wdog_time_t *timeout,
				     sc_timer_wdog_time_t *remaining_time)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_PT_GET_WDOG_STATUS;
	RPC_U8(&msg, 0) = pt;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (timeout != NULL) {
		*timeout = RPC_U32(&msg, 0);
	}

	if (remaining_time != NULL) {
		*remaining_time = RPC_U32(&msg, 4);
	}

	result = RPC_R8(&msg);
	if (enb != NULL) {
		*enb = RPC_U8(&msg, 8);
	}

	return (sc_err_t)result;
}

sc_err_t sc_timer_set_wdog_action(sc_ipc_t ipc,
				  sc_rm_pt_t pt, sc_timer_wdog_action_t action)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_WDOG_ACTION;
	RPC_U8(&msg, 0) = pt;
	RPC_U8(&msg, 1) = action;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_set_rtc_time(sc_ipc_t ipc, uint16_t year, uint8_t mon,
			       uint8_t day, uint8_t hour, uint8_t min,
			       uint8_t sec)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_RTC_TIME;
	RPC_U16(&msg, 0) = year;
	RPC_U8(&msg, 2) = mon;
	RPC_U8(&msg, 3) = day;
	RPC_U8(&msg, 4) = hour;
	RPC_U8(&msg, 5) = min;
	RPC_U8(&msg, 6) = sec;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_get_rtc_time(sc_ipc_t ipc, uint16_t *year, uint8_t *mon,
			       uint8_t *day, uint8_t *hour, uint8_t *min,
			       uint8_t *sec)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_GET_RTC_TIME;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (year != NULL) {
		*year = RPC_U16(&msg, 0);
	}

	result = RPC_R8(&msg);
	if (mon != NULL) {
		*mon = RPC_U8(&msg, 2);
	}

	if (day != NULL) {
		*day = RPC_U8(&msg, 3);
	}

	if (hour != NULL) {
		*hour = RPC_U8(&msg, 4);
	}

	if (min != NULL) {
		*min = RPC_U8(&msg, 5);
	}

	if (sec != NULL) {
		*sec = RPC_U8(&msg, 6);
	}

	return (sc_err_t)result;
}

sc_err_t sc_timer_get_rtc_sec1970(sc_ipc_t ipc, uint32_t *sec)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_GET_RTC_SEC1970;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (sec != NULL) {
		*sec = RPC_U32(&msg, 0);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_set_rtc_alarm(sc_ipc_t ipc, uint16_t year, uint8_t mon,
				uint8_t day, uint8_t hour, uint8_t min,
				uint8_t sec)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_RTC_ALARM;
	RPC_U16(&msg, 0) = year;
	RPC_U8(&msg, 2) = mon;
	RPC_U8(&msg, 3) = day;
	RPC_U8(&msg, 4) = hour;
	RPC_U8(&msg, 5) = min;
	RPC_U8(&msg, 6) = sec;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_timer_set_rtc_calb(sc_ipc_t ipc, int8_t count)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_TIMER;
	RPC_FUNC(&msg) = (uint8_t)TIMER_FUNC_SET_RTC_CALB;
	RPC_I8(&msg, 0) = count;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

/**@}*/
