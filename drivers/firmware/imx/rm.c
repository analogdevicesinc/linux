// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 *
 * File containing client-side RPC functions for the RM service. These
 * function are ported to clients that communicate to the SC.
 */

#include <linux/firmware/imx/svc/rm.h>

struct imx_sc_msg_rm_rsrc_owned {
	struct imx_sc_rpc_msg hdr;
	u16 resource;
} __packed __aligned(4);

struct imx_sc_msg_rm_pt {
	struct imx_sc_rpc_msg hdr;
	u8 val;
} __packed __aligned(4);

/*
 * This function check @resource is owned by current partition or not
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    resource the control is associated with
 *
 * @return Returns 0 for not owned and 1 for owned.
 */
bool imx_sc_rm_is_resource_owned(struct imx_sc_ipc *ipc, u16 resource)
{
	struct imx_sc_msg_rm_rsrc_owned msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_IS_RESOURCE_OWNED;
	hdr->size = 2;

	msg.resource = resource;

	/*
	 * SCU firmware only returns value 0 or 1
	 * for resource owned check which means not owned or owned.
	 * So it is always successful.
	 */
	imx_scu_call_rpc(ipc, &msg, true);

	return hdr->func;
}
EXPORT_SYMBOL(imx_sc_rm_is_resource_owned);

struct imx_sc_msg_rm_get_resource_owner {
	struct imx_sc_rpc_msg hdr;
	union {
		struct {
			u16 resource;
		} req;
		struct {
			u8 val;
		} resp;
	} data;
} __packed __aligned(4);

/*
 * This function get @resource partition number
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    resource the control is associated with
 * @param[out]    pt          pointer to return the partition number
 *
 * @return Returns 0 for success and < 0 for errors.
 */
int imx_sc_rm_get_resource_owner(struct imx_sc_ipc *ipc, u16 resource, u8 *pt)
{
	struct imx_sc_msg_rm_get_resource_owner msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_GET_RESOURCE_OWNER;
	hdr->size = 2;

	msg.data.req.resource = resource;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	if (pt)
		*pt = msg.data.resp.val;

	return 0;
}
EXPORT_SYMBOL(imx_sc_rm_get_resource_owner);

/*
 * This function returns the current partition number
 *
 * @param[in]     ipc         IPC handle
 * @param[out]    pt          holding the partition number
 *
 * @return Returns 0 for success and < 0 for errors.
 */
int imx_sc_rm_get_partition(struct imx_sc_ipc *ipc, u8 *pt)
{
	struct imx_sc_msg_rm_pt msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_GET_PARTITION;
	hdr->size = 1;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	if (pt != NULL)
		*pt = msg.val;

	return 0;
}
EXPORT_SYMBOL(imx_sc_rm_get_partition);

struct imx_sc_msg_rm_find_memreg {
	struct imx_sc_rpc_msg hdr;
	union {
		struct {
			u32 add_start_hi;
			u32 add_start_lo;
			u32 add_end_hi;
			u32 add_end_lo;
		} req;
		struct {
			u8 val;
		} resp;
	} data;
}  __packed __aligned(4);

int imx_sc_rm_find_memreg(struct imx_sc_ipc *ipc, u8 *mr, u64 addr_start,
			  u64 addr_end)
{
	struct imx_sc_msg_rm_find_memreg msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_FIND_MEMREG;
	hdr->size = 5;

	msg.data.req.add_start_hi = addr_start >> 32;
	msg.data.req.add_start_lo = addr_start;
	msg.data.req.add_end_hi = addr_end >> 32;
	msg.data.req.add_end_lo = addr_end;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret)
		return ret;

	if (mr)
		*mr = msg.data.resp.val;

	return 0;
}
EXPORT_SYMBOL(imx_sc_rm_find_memreg);

struct imx_sc_msg_set_memreg_permissions {
	struct imx_sc_rpc_msg hdr;
	u8 mr;
	u8 pt;
	u8 perm;
} __packed __aligned(4);

int imx_sc_rm_set_memreg_permissions(struct imx_sc_ipc *ipc, u8 mr,
				     u8 pt, u8 perm)
{
	struct imx_sc_msg_set_memreg_permissions msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_SET_MEMREG_PERMISSIONS;
	hdr->size = 2;

	msg.mr = mr;
	msg.pt = pt;
	msg.perm = perm;

	return imx_scu_call_rpc(ipc, &msg, true);
}
EXPORT_SYMBOL(imx_sc_rm_set_memreg_permissions);

int imx_sc_rm_get_did(struct imx_sc_ipc *ipc, u8 *did)
{
	struct imx_sc_rpc_msg msg;
	struct imx_sc_rpc_msg *hdr = &msg;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_RM;
	hdr->func = IMX_SC_RM_FUNC_GET_DID;
	hdr->size = 1;

	ret = imx_scu_call_rpc(ipc, &msg, true);
	if (ret < 0)
		return ret;

	if (did)
		*did = msg.func;

	return 0;
}
EXPORT_SYMBOL(imx_sc_rm_get_did);
