/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file for the RM RPC implementation.
 *
 * @addtogroup RM_SVC
 * @{
 */

#ifndef _SC_RM_RPC_H
#define _SC_RM_RPC_H

/* Includes */

/* Defines */

/* Types */

/*!
 * This type is used to indicate RPC RM function calls.
 */
typedef enum rm_func_e {
	RM_FUNC_UNKNOWN = 0,	/* Unknown function */
	RM_FUNC_PARTITION_ALLOC = 1,	/* Index for rm_partition_alloc() RPC call */
	RM_FUNC_SET_CONFIDENTIAL = 31,	/* Index for rm_set_confidential() RPC call */
	RM_FUNC_PARTITION_FREE = 2,	/* Index for rm_partition_free() RPC call */
	RM_FUNC_GET_DID = 26,	/* Index for rm_get_did() RPC call */
	RM_FUNC_PARTITION_STATIC = 3,	/* Index for rm_partition_static() RPC call */
	RM_FUNC_PARTITION_LOCK = 4,	/* Index for rm_partition_lock() RPC call */
	RM_FUNC_GET_PARTITION = 5,	/* Index for rm_get_partition() RPC call */
	RM_FUNC_SET_PARENT = 6,	/* Index for rm_set_parent() RPC call */
	RM_FUNC_MOVE_ALL = 7,	/* Index for rm_move_all() RPC call */
	RM_FUNC_ASSIGN_RESOURCE = 8,	/* Index for rm_assign_resource() RPC call */
	RM_FUNC_SET_RESOURCE_MOVABLE = 9,	/* Index for rm_set_resource_movable() RPC call */
	RM_FUNC_SET_SUBSYS_RSRC_MOVABLE = 28,	/* Index for rm_set_subsys_rsrc_movable() RPC call */
	RM_FUNC_SET_MASTER_ATTRIBUTES = 10,	/* Index for rm_set_master_attributes() RPC call */
	RM_FUNC_SET_MASTER_SID = 11,	/* Index for rm_set_master_sid() RPC call */
	RM_FUNC_SET_PERIPHERAL_PERMISSIONS = 12,	/* Index for rm_set_peripheral_permissions() RPC call */
	RM_FUNC_IS_RESOURCE_OWNED = 13,	/* Index for rm_is_resource_owned() RPC call */
	RM_FUNC_IS_RESOURCE_MASTER = 14,	/* Index for rm_is_resource_master() RPC call */
	RM_FUNC_IS_RESOURCE_PERIPHERAL = 15,	/* Index for rm_is_resource_peripheral() RPC call */
	RM_FUNC_GET_RESOURCE_INFO = 16,	/* Index for rm_get_resource_info() RPC call */
	RM_FUNC_MEMREG_ALLOC = 17,	/* Index for rm_memreg_alloc() RPC call */
	RM_FUNC_MEMREG_SPLIT = 29,	/* Index for rm_memreg_split() RPC call */
	RM_FUNC_MEMREG_FREE = 18,	/* Index for rm_memreg_free() RPC call */
	RM_FUNC_FIND_MEMREG = 30,	/* Index for rm_find_memreg() RPC call */
	RM_FUNC_ASSIGN_MEMREG = 19,	/* Index for rm_assign_memreg() RPC call */
	RM_FUNC_SET_MEMREG_PERMISSIONS = 20,	/* Index for rm_set_memreg_permissions() RPC call */
	RM_FUNC_IS_MEMREG_OWNED = 21,	/* Index for rm_is_memreg_owned() RPC call */
	RM_FUNC_GET_MEMREG_INFO = 22,	/* Index for rm_get_memreg_info() RPC call */
	RM_FUNC_ASSIGN_PAD = 23,	/* Index for rm_assign_pad() RPC call */
	RM_FUNC_SET_PAD_MOVABLE = 24,	/* Index for rm_set_pad_movable() RPC call */
	RM_FUNC_IS_PAD_OWNED = 25,	/* Index for rm_is_pad_owned() RPC call */
	RM_FUNC_DUMP = 27,	/* Index for rm_dump() RPC call */
} rm_func_t;

/* Functions */

/*!
 * This function dispatches an incoming RM RPC request.
 *
 * @param[in]     caller_pt   caller partition
 * @param[in]     msg         pointer to RPC message
 */
void rm_dispatch(sc_rm_pt_t caller_pt, sc_rpc_msg_t *msg);

/*!
 * This function translates and dispatches an RM RPC request.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     msg         pointer to RPC message
 */
void rm_xlate(sc_ipc_t ipc, sc_rpc_msg_t *msg);

#endif				/* _SC_RM_RPC_H */

/**@}*/
