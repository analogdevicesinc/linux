/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the PM service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup PM_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/pm/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_pm_set_sys_power_mode(sc_ipc_t ipc, sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_SYS_POWER_MODE);
	RPC_U8(&msg, 0U) = U8(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_partition_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
					sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_PARTITION_POWER_MODE);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_sys_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
				  sc_pm_power_mode_t *mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_GET_SYS_POWER_MODE);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mode != NULL) {
		*mode = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pm_set_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_RESOURCE_POWER_MODE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_resource_power_mode_all(sc_ipc_t ipc,
					   sc_rm_pt_t pt,
					   sc_pm_power_mode_t mode,
					   sc_rsrc_t exclude)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_RESOURCE_POWER_MODE_ALL);
	RPC_U16(&msg, 0U) = U16(exclude);
	RPC_U8(&msg, 2U) = U8(pt);
	RPC_U8(&msg, 3U) = U8(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t *mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_GET_RESOURCE_POWER_MODE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mode != NULL) {
		*mode = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pm_req_low_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				  sc_pm_power_mode_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_REQ_LOW_POWER_MODE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_req_cpu_low_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				      sc_pm_power_mode_t mode,
				      sc_pm_wake_src_t wake_src)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_REQ_CPU_LOW_POWER_MODE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(mode);
	RPC_U8(&msg, 3U) = U8(wake_src);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_cpu_resume_addr(sc_ipc_t ipc, sc_rsrc_t resource,
				   sc_faddr_t address)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_CPU_RESUME_ADDR);
	RPC_U32(&msg, 0U) = U32(address >> 32ULL);
	RPC_U32(&msg, 4U) = U32(address);
	RPC_U16(&msg, 8U) = U16(resource);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_cpu_resume(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_bool_t isPrimary, sc_faddr_t address)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_CPU_RESUME);
	RPC_U32(&msg, 0U) = U32(address >> 32ULL);
	RPC_U32(&msg, 4U) = U32(address);
	RPC_U16(&msg, 8U) = U16(resource);
	RPC_U8(&msg, 10U) = B2U8(isPrimary);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_req_sys_if_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				     sc_pm_sys_if_t sys_if,
				     sc_pm_power_mode_t hpm,
				     sc_pm_power_mode_t lpm)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_REQ_SYS_IF_POWER_MODE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(sys_if);
	RPC_U8(&msg, 3U) = U8(hpm);
	RPC_U8(&msg, 4U) = U8(lpm);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_CLOCK_RATE);
	RPC_U32(&msg, 0U) = *PTR_U32(rate);
	RPC_U16(&msg, 4U) = U16(resource);
	RPC_U8(&msg, 6U) = U8(clk);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	*rate = RPC_U32(&msg, 0U);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_GET_CLOCK_RATE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(clk);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (rate != NULL) {
		*rate = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_clock_enable(sc_ipc_t ipc, sc_rsrc_t resource,
			    sc_pm_clk_t clk, sc_bool_t enable, sc_bool_t autog)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_CLOCK_ENABLE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(clk);
	RPC_U8(&msg, 3U) = B2U8(enable);
	RPC_U8(&msg, 4U) = B2U8(autog);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_set_clock_parent(sc_ipc_t ipc, sc_rsrc_t resource,
				sc_pm_clk_t clk, sc_pm_clk_parent_t parent)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_SET_CLOCK_PARENT);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(clk);
	RPC_U8(&msg, 3U) = U8(parent);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_get_clock_parent(sc_ipc_t ipc, sc_rsrc_t resource,
				sc_pm_clk_t clk, sc_pm_clk_parent_t * parent)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_GET_CLOCK_PARENT);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(clk);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (parent != NULL) {
		*parent = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pm_reset(sc_ipc_t ipc, sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_RESET);
	RPC_U8(&msg, 0U) = U8(type);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_reset_reason(sc_ipc_t ipc, sc_pm_reset_reason_t *reason)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_RESET_REASON);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (reason != NULL) {
		*reason = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pm_boot(sc_ipc_t ipc, sc_rm_pt_t pt,
		    sc_rsrc_t resource_cpu, sc_faddr_t boot_addr,
		    sc_rsrc_t resource_mu, sc_rsrc_t resource_dev)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_BOOT);
	RPC_U32(&msg, 0U) = U32(boot_addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(boot_addr);
	RPC_U16(&msg, 8U) = U16(resource_cpu);
	RPC_U16(&msg, 10U) = U16(resource_mu);
	RPC_U16(&msg, 12U) = U16(resource_dev);
	RPC_U8(&msg, 14U) = U8(pt);
	RPC_SIZE(&msg) = 5U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_pm_reboot(sc_ipc_t ipc, sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_REBOOT);
	RPC_U8(&msg, 0U) = U8(type);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_TRUE);

	return;
}

sc_err_t sc_pm_reboot_partition(sc_ipc_t ipc, sc_rm_pt_t pt,
				sc_pm_reset_type_t type)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_REBOOT_PARTITION);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(type);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pm_cpu_start(sc_ipc_t ipc, sc_rsrc_t resource, sc_bool_t enable,
			 sc_faddr_t address)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_CPU_START);
	RPC_U32(&msg, 0U) = U32(address >> 32ULL);
	RPC_U32(&msg, 4U) = U32(address);
	RPC_U16(&msg, 8U) = U16(resource);
	RPC_U8(&msg, 10U) = B2U8(enable);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_pm_cpu_reset(sc_ipc_t ipc, sc_rsrc_t resource, sc_faddr_t address)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PM);
	RPC_FUNC(&msg) = U8(PM_FUNC_CPU_RESET);
	RPC_U32(&msg, 0U) = U32(address >> 32ULL);
	RPC_U32(&msg, 4U) = U32(address);
	RPC_U16(&msg, 8U) = U16(resource);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_TRUE);

	return;
}

/**@}*/
