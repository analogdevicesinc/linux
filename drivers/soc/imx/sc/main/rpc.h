/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the RPC implementation.
 */

#ifndef SC_RPC_H
#define SC_RPC_H

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/ipc.h>

/* Defines */

#define SC_RPC_VERSION          1U

#define SC_RPC_MAX_MSG          8U

#define RPC_VER(MESG)           ((MESG)->version)
#define RPC_SIZE(MESG)          ((MESG)->size)
#define RPC_SVC(MESG)           ((MESG)->svc)
#define RPC_FUNC(MESG)          ((MESG)->func)
#define RPC_R8(MESG)            ((MESG)->func)
#define RPC_I32(MESG, IDX)      ((MESG)->DATA.i32[(IDX) / 4U])
#define RPC_I16(MESG, IDX)      ((MESG)->DATA.i16[(IDX) / 2U])
#define RPC_I8(MESG, IDX)       ((MESG)->DATA.i8[(IDX)])
#define RPC_U32(MESG, IDX)      ((MESG)->DATA.u32[(IDX) / 4U])
#define RPC_U16(MESG, IDX)      ((MESG)->DATA.u16[(IDX) / 2U])
#define RPC_U8(MESG, IDX)       ((MESG)->DATA.u8[(IDX)])

#define SC_RPC_SVC_UNKNOWN      0U
#define SC_RPC_SVC_RETURN       1U
#define SC_RPC_SVC_PM           2U
#define SC_RPC_SVC_RM           3U
#define SC_RPC_SVC_TIMER        5U
#define SC_RPC_SVC_PAD          6U
#define SC_RPC_SVC_MISC         7U
#define SC_RPC_SVC_IRQ          8U
#define SC_RPC_SVC_ABORT        9U

#define SC_RPC_ASYNC_STATE_RD_START      0U
#define SC_RPC_ASYNC_STATE_RD_ACTIVE     1U
#define SC_RPC_ASYNC_STATE_RD_DONE       2U
#define SC_RPC_ASYNC_STATE_WR_START      3U
#define SC_RPC_ASYNC_STATE_WR_ACTIVE     4U
#define SC_RPC_ASYNC_STATE_WR_DONE       5U

#define SC_RPC_MU_GIR_SVC       0x1U
#define SC_RPC_MU_GIR_DBG       0x8U

#define I8(X)       ((int8_t) (X))
#define I16(X)      ((int16_t) (X))
#define I32(X)      ((int32_t) (X))
#define I64(X)      ((int64_t) (X))
#define U8(X)       ((uint8_t) (X))
#define U16(X)      ((uint16_t) (X))
#define U32(X)      ((uint32_t) (X))
#define U64(X)      ((uint64_t) (X))

#define PTR_I8(X)   ((int8_t *) (X))
#define PTR_I16(X)  ((int16_t *) (X))
#define PTR_I32(X)  ((int32_t *) (X))
#define PTR_I64(X)  ((int64_t *) (X))
#define PTR_U8(X)   ((uint8_t *) (X))
#define PTR_U16(X)  ((uint16_t *) (X))
#define PTR_U32(X)  ((uint32_t *) (X))
#define PTR_U64(X)  ((uint64_t *) (X))

#define U2B(X)      (((X) != 0U) ? SC_TRUE : SC_FALSE)
#define U2B32(X)    (((X) != 0UL) ? SC_TRUE : SC_FALSE)
#define B2U8(X)     (((X) != SC_FALSE) ? U8(0x01U) : U8(0x00U))
#define B2U16(X)    (((X) != SC_FALSE) ? U16(0x01U) : U16(0x00U))
#define B2U32(X)    (((X) != SC_FALSE) ? U32(0x01U) : U32(0x00U))

/* Types */

typedef uint8_t sc_rpc_svc_t;

typedef struct sc_rpc_msg_s {
	uint8_t version;
	uint8_t size;
	uint8_t svc;
	uint8_t func;
	union {
		int32_t i32[(SC_RPC_MAX_MSG - 1U)];
		int16_t i16[(SC_RPC_MAX_MSG - 1U) * 2U];
		int8_t i8[(SC_RPC_MAX_MSG - 1U) * 4U];
		uint32_t u32[(SC_RPC_MAX_MSG - 1U)];
		uint16_t u16[(SC_RPC_MAX_MSG - 1U) * 2U];
		uint8_t u8[(SC_RPC_MAX_MSG - 1U) * 4U];
	} DATA;
} sc_rpc_msg_t;

typedef uint8_t sc_rpc_async_state_t;

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
 * If \a no_resp is SC_FALSE then this function waits for a response
 * and returns the result in \a msg.
 */
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp);

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

#endif				/* SC_RPC_H */
