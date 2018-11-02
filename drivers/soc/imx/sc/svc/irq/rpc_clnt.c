/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the IRQ service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup IRQ_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_irq_enable(sc_ipc_t ipc, sc_rsrc_t resource,
		       sc_irq_group_t group, uint32_t mask, sc_bool_t enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_IRQ);
	RPC_FUNC(&msg) = U8(IRQ_FUNC_ENABLE);
	RPC_U32(&msg, 0U) = U32(mask);
	RPC_U16(&msg, 4U) = U16(resource);
	RPC_U8(&msg, 6U) = U8(group);
	RPC_U8(&msg, 7U) = B2U8(enable);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_irq_status(sc_ipc_t ipc, sc_rsrc_t resource,
		       sc_irq_group_t group, uint32_t *status)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_IRQ);
	RPC_FUNC(&msg) = U8(IRQ_FUNC_STATUS);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(group);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (status != NULL) {
		*status = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

/**@}*/
