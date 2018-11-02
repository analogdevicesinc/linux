/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the RM service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup RM_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_rm_partition_alloc(sc_ipc_t ipc, sc_rm_pt_t *pt, sc_bool_t secure,
			       sc_bool_t isolated, sc_bool_t restricted,
			       sc_bool_t grant, sc_bool_t coherent)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_PARTITION_ALLOC);
	RPC_U8(&msg, 0U) = B2U8(secure);
	RPC_U8(&msg, 1U) = B2U8(isolated);
	RPC_U8(&msg, 2U) = B2U8(restricted);
	RPC_U8(&msg, 3U) = B2U8(grant);
	RPC_U8(&msg, 4U) = B2U8(coherent);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (pt != NULL) {
		*pt = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_set_confidential(sc_ipc_t ipc, sc_rm_pt_t pt, sc_bool_t retro)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_CONFIDENTIAL);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = B2U8(retro);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_partition_free(sc_ipc_t ipc, sc_rm_pt_t pt)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_PARTITION_FREE);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_rm_did_t sc_rm_get_did(sc_ipc_t ipc)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_GET_DID);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_rm_did_t) result;
}

sc_err_t sc_rm_partition_static(sc_ipc_t ipc, sc_rm_pt_t pt, sc_rm_did_t did)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_PARTITION_STATIC);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(did);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_partition_lock(sc_ipc_t ipc, sc_rm_pt_t pt)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_PARTITION_LOCK);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_get_partition(sc_ipc_t ipc, sc_rm_pt_t *pt)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_GET_PARTITION);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (pt != NULL) {
		*pt = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_set_parent(sc_ipc_t ipc, sc_rm_pt_t pt, sc_rm_pt_t pt_parent)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_PARENT);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(pt_parent);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_move_all(sc_ipc_t ipc, sc_rm_pt_t pt_src, sc_rm_pt_t pt_dst,
			sc_bool_t move_rsrc, sc_bool_t move_pads)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_MOVE_ALL);
	RPC_U8(&msg, 0U) = U8(pt_src);
	RPC_U8(&msg, 1U) = U8(pt_dst);
	RPC_U8(&msg, 2U) = B2U8(move_rsrc);
	RPC_U8(&msg, 3U) = B2U8(move_pads);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_assign_resource(sc_ipc_t ipc, sc_rm_pt_t pt, sc_rsrc_t resource)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_ASSIGN_RESOURCE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(pt);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_resource_movable(sc_ipc_t ipc, sc_rsrc_t resource_fst,
				    sc_rsrc_t resource_lst, sc_bool_t movable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_RESOURCE_MOVABLE);
	RPC_U16(&msg, 0U) = U16(resource_fst);
	RPC_U16(&msg, 2U) = U16(resource_lst);
	RPC_U8(&msg, 4U) = B2U8(movable);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_subsys_rsrc_movable(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_bool_t movable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_SUBSYS_RSRC_MOVABLE);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = B2U8(movable);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_master_attributes(sc_ipc_t ipc, sc_rsrc_t resource,
				     sc_rm_spa_t sa, sc_rm_spa_t pa,
				     sc_bool_t smmu_bypass)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_MASTER_ATTRIBUTES);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(sa);
	RPC_U8(&msg, 3U) = U8(pa);
	RPC_U8(&msg, 4U) = B2U8(smmu_bypass);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_master_sid(sc_ipc_t ipc, sc_rsrc_t resource, sc_rm_sid_t sid)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_MASTER_SID);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U16(&msg, 2U) = U16(sid);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_peripheral_permissions(sc_ipc_t ipc, sc_rsrc_t resource,
					  sc_rm_pt_t pt, sc_rm_perm_t perm)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_PERIPHERAL_PERMISSIONS);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(pt);
	RPC_U8(&msg, 3U) = U8(perm);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_bool_t sc_rm_is_resource_owned(sc_ipc_t ipc, sc_rsrc_t resource)
{
	sc_rpc_msg_t msg;
	sc_bool_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_IS_RESOURCE_OWNED);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = U2B(RPC_R8(&msg));
	return result;
}

sc_bool_t sc_rm_is_resource_master(sc_ipc_t ipc, sc_rsrc_t resource)
{
	sc_rpc_msg_t msg;
	sc_bool_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_IS_RESOURCE_MASTER);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = U2B(RPC_R8(&msg));
	return result;
}

sc_bool_t sc_rm_is_resource_peripheral(sc_ipc_t ipc, sc_rsrc_t resource)
{
	sc_rpc_msg_t msg;
	sc_bool_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_IS_RESOURCE_PERIPHERAL);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = U2B(RPC_R8(&msg));
	return result;
}

sc_err_t sc_rm_get_resource_info(sc_ipc_t ipc, sc_rsrc_t resource,
				 sc_rm_sid_t *sid)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_GET_RESOURCE_INFO);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (sid != NULL) {
		*sid = RPC_U16(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_memreg_alloc(sc_ipc_t ipc, sc_rm_mr_t *mr,
			    sc_faddr_t addr_start, sc_faddr_t addr_end)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_MEMREG_ALLOC);
	RPC_U32(&msg, 0U) = U32(addr_start >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr_start);
	RPC_U32(&msg, 8U) = U32(addr_end >> 32ULL);
	RPC_U32(&msg, 12U) = U32(addr_end);
	RPC_SIZE(&msg) = 5U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mr != NULL) {
		*mr = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_memreg_split(sc_ipc_t ipc, sc_rm_mr_t mr,
			    sc_rm_mr_t *mr_ret, sc_faddr_t addr_start,
			    sc_faddr_t addr_end)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_MEMREG_SPLIT);
	RPC_U32(&msg, 0U) = U32(addr_start >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr_start);
	RPC_U32(&msg, 8U) = U32(addr_end >> 32ULL);
	RPC_U32(&msg, 12U) = U32(addr_end);
	RPC_U8(&msg, 16U) = U8(mr);
	RPC_SIZE(&msg) = 6U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mr_ret != NULL) {
		*mr_ret = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_memreg_frag(sc_ipc_t ipc, sc_rm_mr_t *mr_ret,
			   sc_faddr_t addr_start, sc_faddr_t addr_end)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_MEMREG_FRAG);
	RPC_U32(&msg, 0U) = U32(addr_start >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr_start);
	RPC_U32(&msg, 8U) = U32(addr_end >> 32ULL);
	RPC_U32(&msg, 12U) = U32(addr_end);
	RPC_SIZE(&msg) = 5U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mr_ret != NULL) {
		*mr_ret = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_memreg_free(sc_ipc_t ipc, sc_rm_mr_t mr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_MEMREG_FREE);
	RPC_U8(&msg, 0U) = U8(mr);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_find_memreg(sc_ipc_t ipc, sc_rm_mr_t *mr,
			   sc_faddr_t addr_start, sc_faddr_t addr_end)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_FIND_MEMREG);
	RPC_U32(&msg, 0U) = U32(addr_start >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr_start);
	RPC_U32(&msg, 8U) = U32(addr_end >> 32ULL);
	RPC_U32(&msg, 12U) = U32(addr_end);
	RPC_SIZE(&msg) = 5U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mr != NULL) {
		*mr = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_rm_assign_memreg(sc_ipc_t ipc, sc_rm_pt_t pt, sc_rm_mr_t mr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_ASSIGN_MEMREG);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(mr);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_memreg_permissions(sc_ipc_t ipc, sc_rm_mr_t mr,
				      sc_rm_pt_t pt, sc_rm_perm_t perm)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_MEMREG_PERMISSIONS);
	RPC_U8(&msg, 0U) = U8(mr);
	RPC_U8(&msg, 1U) = U8(pt);
	RPC_U8(&msg, 2U) = U8(perm);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_bool_t sc_rm_is_memreg_owned(sc_ipc_t ipc, sc_rm_mr_t mr)
{
	sc_rpc_msg_t msg;
	sc_bool_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_IS_MEMREG_OWNED);
	RPC_U8(&msg, 0U) = U8(mr);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = U2B(RPC_R8(&msg));
	return result;
}

sc_err_t sc_rm_get_memreg_info(sc_ipc_t ipc, sc_rm_mr_t mr,
			       sc_faddr_t *addr_start, sc_faddr_t *addr_end)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_GET_MEMREG_INFO);
	RPC_U8(&msg, 0U) = U8(mr);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (addr_start != NULL) {
		*addr_start =
		    ((uint64_t) RPC_U32(&msg, 0U) << 32U) | RPC_U32(&msg, 4U);
	}

	if (addr_end != NULL) {
		*addr_end =
		    ((uint64_t) RPC_U32(&msg, 8U) << 32U) | RPC_U32(&msg, 12U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_assign_pad(sc_ipc_t ipc, sc_rm_pt_t pt, sc_pad_t pad)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_ASSIGN_PAD);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(pt);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_rm_set_pad_movable(sc_ipc_t ipc, sc_pad_t pad_fst,
			       sc_pad_t pad_lst, sc_bool_t movable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_SET_PAD_MOVABLE);
	RPC_U16(&msg, 0U) = U16(pad_fst);
	RPC_U16(&msg, 2U) = U16(pad_lst);
	RPC_U8(&msg, 4U) = B2U8(movable);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_bool_t sc_rm_is_pad_owned(sc_ipc_t ipc, sc_pad_t pad)
{
	sc_rpc_msg_t msg;
	sc_bool_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_IS_PAD_OWNED);
	RPC_U8(&msg, 0U) = U8(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = U2B(RPC_R8(&msg));
	return result;
}

void sc_rm_dump(sc_ipc_t ipc)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_RM);
	RPC_FUNC(&msg) = U8(RM_FUNC_DUMP);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	return;
}

/**@}*/
