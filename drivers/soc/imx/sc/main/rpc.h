/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the RPC implementation.
 */

#ifndef _SC_RPC_H
#define _SC_RPC_H

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/ipc.h>

/* Defines */

#define SC_RPC_VERSION          1

#define SC_RPC_MAX_MSG          8

#define RPC_VER(MSG)            ((MSG)->version)
#define RPC_SIZE(MSG)           ((MSG)->size)
#define RPC_SVC(MSG)            ((MSG)->svc)
#define RPC_FUNC(MSG)           ((MSG)->func)
#define RPC_R8(MSG)             ((MSG)->func)
#define RPC_I32(MSG, IDX)       ((MSG)->DATA.i32[(IDX) / 4])
#define RPC_I16(MSG, IDX)       ((MSG)->DATA.i16[(IDX) / 2])
#define RPC_I8(MSG, IDX)        ((MSG)->DATA.i8[(IDX)])
#define RPC_U32(MSG, IDX)       ((MSG)->DATA.u32[(IDX) / 4])
#define RPC_U16(MSG, IDX)       ((MSG)->DATA.u16[(IDX) / 2])
#define RPC_U8(MSG, IDX)        ((MSG)->DATA.u8[(IDX)])

/* Types */

typedef enum sc_rpc_svc_e {
	SC_RPC_SVC_UNKNOWN = 0,
	SC_RPC_SVC_RETURN = 1,
	SC_RPC_SVC_PM = 2,
	SC_RPC_SVC_RM = 3,
	SC_RPC_SVC_TIMER = 5,
	SC_RPC_SVC_PAD = 6,
	SC_RPC_SVC_MISC = 7,
	SC_RPC_SVC_IRQ = 8,
	SC_RPC_SVC_ABORT = 9
} sc_rpc_svc_t;

typedef struct sc_rpc_msg_s {
	uint8_t version;
	uint8_t size;
	uint8_t svc;
	uint8_t func;
	union {
		int32_t i32[(SC_RPC_MAX_MSG - 1)];
		int16_t i16[(SC_RPC_MAX_MSG - 1) * 2];
		int8_t i8[(SC_RPC_MAX_MSG - 1) * 4];
		uint32_t u32[(SC_RPC_MAX_MSG - 1)];
		uint16_t u16[(SC_RPC_MAX_MSG - 1) * 2];
		uint8_t u8[(SC_RPC_MAX_MSG - 1) * 4];
	} DATA;
} sc_rpc_msg_t;

typedef enum sc_rpc_async_state_e {
	SC_RPC_ASYNC_STATE_RD_START = 0,
	SC_RPC_ASYNC_STATE_RD_ACTIVE = 1,
	SC_RPC_ASYNC_STATE_RD_DONE = 2,
	SC_RPC_ASYNC_STATE_WR_START = 3,
	SC_RPC_ASYNC_STATE_WR_ACTIVE = 4,
	SC_RPC_ASYNC_STATE_WR_DONE = 5,
} sc_rpc_async_state_t;

typedef struct sc_rpc_async_msg_s {
	sc_rpc_async_state_t state;
	uint8_t wordIdx;
	sc_rpc_msg_t msg;
	uint32_t timeStamp;
} sc_rpc_async_msg_t;

/* Functions */

/*!
 * This is an internal function to send an RPC message over an IPC
 * channel. It is called by client-side SCFW API function shims.
 *
 * @param[in]     ipc         IPC handle
 * @param[in,out] msg         handle to a message
 * @param[in]     no_resp     response flag
 *
 * If \a no_resp is false then this function waits for a response
 * and returns the result in \a msg.
 */
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, bool no_resp);

/*!
 * This is an internal function to dispath an RPC call that has
 * arrived via IPC over an MU. It is called by server-side SCFW.
 *
 * @param[in]     mu          MU message arrived on
 * @param[in,out] msg         handle to a message
 *
 * The function result is returned in \a msg.
 */
void sc_rpc_dispatch(sc_rsrc_t mu, sc_rpc_msg_t *msg);

/*!
 * This function translates an RPC message and forwards on to the
 * normal RPC API.  It is used only by hypervisors.
 *
 * @param[in]     ipc         IPC handle
 * @param[in,out] msg         handle to a message
 *
 * This function decodes a message, calls macros to translate the
 * resources, pads, addresses, partitions, memory regions, etc. and
 * then forwards on to the hypervisors SCFW API.Return results are
 * translated back abd placed back into the message to be returned
 * to the original API.
 */
void sc_rpc_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_RPC_H */
