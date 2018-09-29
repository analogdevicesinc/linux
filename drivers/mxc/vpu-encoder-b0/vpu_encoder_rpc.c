/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/stddef.h>
#include "vpu_encoder_rpc.h"

void rpc_init_shared_memory_encoder(struct shared_addr *This,
		unsigned long long base_phy_addr,
		void *base_virt_addr,
		u_int32 total_size)
{
	pENC_RPC_HOST_IFACE pSharedInterface;
	unsigned int phy_addr;
	unsigned int i;
	unsigned int temp_addr;
	BUFFER_DESCRIPTOR_TYPE *pSharedCmdBufDescPtr;
	BUFFER_DESCRIPTOR_TYPE *pSharedMsgBufDescPtr;
	pMEDIA_ENC_API_CONTROL_INTERFACE pEncCtrlInterface;

	This->shared_mem_phy = base_phy_addr;
	This->shared_mem_vir = base_virt_addr;
	This->base_offset = (unsigned long long)(base_virt_addr - base_phy_addr);

	pSharedInterface = (pENC_RPC_HOST_IFACE)This->shared_mem_vir;
	This->pSharedInterface = pSharedInterface;

	pSharedInterface->FwExecBaseAddr = base_phy_addr;
	pSharedInterface->FwExecAreaSize = total_size;

	pSharedCmdBufDescPtr = (BUFFER_DESCRIPTOR_TYPE *)&pSharedInterface->StreamCmdBufferDesc;
	pSharedMsgBufDescPtr = (BUFFER_DESCRIPTOR_TYPE *)&pSharedInterface->StreamMsgBufferDesc;

	phy_addr = base_phy_addr + sizeof(ENC_RPC_HOST_IFACE);
	This->cmd_mem_phy = phy_addr;
	This->cmd_mem_vir = This->shared_mem_vir + sizeof(ENC_RPC_HOST_IFACE);

	pSharedCmdBufDescPtr->wptr = phy_addr;
	pSharedCmdBufDescPtr->rptr = pSharedCmdBufDescPtr->wptr;
	pSharedCmdBufDescPtr->start = pSharedCmdBufDescPtr->wptr;
	pSharedCmdBufDescPtr->end = pSharedCmdBufDescPtr->start + CMD_SIZE;

	phy_addr += CMD_SIZE;
	This->msg_mem_phy = phy_addr;
	This->msg_mem_vir = This->cmd_mem_vir + CMD_SIZE;

	pSharedMsgBufDescPtr->wptr = phy_addr;
	pSharedMsgBufDescPtr->rptr = pSharedMsgBufDescPtr->wptr;
	pSharedMsgBufDescPtr->start = pSharedMsgBufDescPtr->wptr;
	pSharedMsgBufDescPtr->end = pSharedMsgBufDescPtr->start + MSG_SIZE;

	phy_addr += MSG_SIZE;

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		pSharedInterface->pEncCtrlInterface[i] = phy_addr;
		phy_addr += sizeof(MEDIA_ENC_API_CONTROL_INTERFACE);
	}

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		temp_addr = pSharedInterface->pEncCtrlInterface[i];
		pEncCtrlInterface = (pMEDIA_ENC_API_CONTROL_INTERFACE)(temp_addr + This->base_offset);
		pEncCtrlInterface->pEncYUVBufferDesc = phy_addr;
		phy_addr += sizeof(MEDIAIP_ENC_YUV_BUFFER_DESC);
		pEncCtrlInterface->pEncStreamBufferDesc = phy_addr;
		phy_addr += sizeof(BUFFER_DESCRIPTOR_TYPE);
		pEncCtrlInterface->pEncExpertModeParam = phy_addr;
		phy_addr += sizeof(MEDIAIP_ENC_EXPERT_MODE_PARAM);
		pEncCtrlInterface->pEncParam = phy_addr;
		phy_addr += sizeof(MEDIAIP_ENC_PARAM);
		pEncCtrlInterface->pEncMemPool = phy_addr;
		phy_addr += sizeof(MEDIAIP_ENC_MEM_POOL);
		pEncCtrlInterface->pEncEncodingStatus = phy_addr;
		phy_addr += sizeof(ENC_ENCODING_STATUS);
		pEncCtrlInterface->pEncDSAStatus = phy_addr;
		phy_addr += sizeof(ENC_DSA_STATUS_t);
	}
}

void rpc_set_system_cfg_value_encoder(void *Interface, u_int32 regs_base, u_int32 core_id)
{
	pENC_RPC_HOST_IFACE pSharedInterface;
	MEDIAIP_FW_SYSTEM_CONFIG *pSystemCfg;

	pSharedInterface = (pENC_RPC_HOST_IFACE)Interface;
	pSystemCfg = &pSharedInterface->sSystemCfg;
	pSystemCfg->uNumWindsors = 1;
	pSystemCfg->uWindsorIrqPin[0x0][0x0] = 0x4; // PAL_IRQ_WINDSOR_LOW
	pSystemCfg->uWindsorIrqPin[0x0][0x1] = 0x5; // PAL_IRQ_WINDSOR_HI
	pSystemCfg->uMaloneBaseAddress[0] = (unsigned int)(regs_base + 0x180000);
	if (core_id == 0)
		pSystemCfg->uWindsorBaseAddress[0] = (unsigned int)(regs_base + 0x800000);
	else
		pSystemCfg->uWindsorBaseAddress[0] = (unsigned int)(regs_base + 0xa00000);
	pSystemCfg->uMaloneBaseAddress[0x1] = 0x0;
	pSystemCfg->uHifOffset[0x0] = 0x1C000;
	pSystemCfg->uHifOffset[0x1] = 0x0;

	pSystemCfg->uDPVBaseAddr = 0x0;
	pSystemCfg->uDPVIrqPin = 0x0;
	pSystemCfg->uPixIfBaseAddr = (unsigned int)(regs_base + 0x180000 + 0x20000);
	pSystemCfg->uFSLCacheBaseAddr[0] = (unsigned int)(regs_base + 0x60000);
	pSystemCfg->uFSLCacheBaseAddr[1] = (unsigned int)(regs_base + 0x68000);
}

u_int32 rpc_MediaIPFW_Video_buffer_space_check_encoder(BUFFER_DESCRIPTOR_TYPE *pBufDesc,
	BOOL bFull,
	u_int32 uSize,
	u_int32 *puUpdateAddress)
{
	u_int32 uPtr1;
	u_int32 uPtr2;
	u_int32 start;
	u_int32 end;
	u_int32 uTemp;

	/* bFull is FALSE when send message, write data   */
	/* bFull is TRUE when process commands, read data */
	uPtr1 = (bFull) ? pBufDesc->rptr : pBufDesc->wptr;
	uPtr2 = (bFull) ? pBufDesc->wptr : pBufDesc->rptr;

	if (uPtr1 == uPtr2) {
		if (bFull)
			/* No data at all to read */
			return 0;
		else {
			/* wrt pointer equal to read pointer thus the     */
			/* buffer is completely empty for further writes  */
			start = pBufDesc->start;
			end   = pBufDesc->end;
			/* The address to be returned in this case is for */
			/* the updated write pointer.                     */
			uTemp = uPtr1 + uSize;
			if (uTemp >= end)
				uTemp += (start - end);
			*puUpdateAddress = uTemp;
			return (end - start);
		}
	} else if (uPtr1 < uPtr2) {
		/* return updated rd pointer address                */
		/* In this case if size was too big - we expect the */
		/* external ftn to compare the size against the     */
		/* space returned.
		 */
		*puUpdateAddress = uPtr1 + uSize;
		return (uPtr2 - uPtr1);
	}
	/* We know the system has looped!! */
	start = pBufDesc->start;
	end   = pBufDesc->end;
	uTemp  = uPtr1 + uSize;
	if (uTemp >= end)
		uTemp += (start - end);
	*puUpdateAddress = uTemp;
	return ((end - uPtr1) + (uPtr2 - start));
}

static void rpc_update_cmd_buffer_ptr_encoder(BUFFER_DESCRIPTOR_TYPE *pCmdDesc)
{
	u_int32 uWritePtr;

	uWritePtr = pCmdDesc->wptr + 4;
	if (uWritePtr >= pCmdDesc->end)
		uWritePtr = pCmdDesc->start;
	pCmdDesc->wptr = uWritePtr;
}

void rpc_send_cmd_buf_encoder(struct shared_addr *This,
		u_int32 idx,
		u_int32 cmdid,
		u_int32 cmdnum,
		u_int32 *local_cmddata)
{
	pENC_RPC_HOST_IFACE pSharedInterface = (pENC_RPC_HOST_IFACE)This->shared_mem_vir;
	BUFFER_DESCRIPTOR_TYPE *pCmdDesc = &pSharedInterface->StreamCmdBufferDesc;
	u_int32 *cmddata;
	u_int32 i;
	u_int32 *cmdword = (u_int32 *)(This->cmd_mem_vir+pCmdDesc->wptr - pCmdDesc->start);

	*cmdword = 0;
	*cmdword |= ((idx & 0x000000ff) << 24);
	*cmdword |= ((cmdnum & 0x000000ff) << 16);
	*cmdword |= ((cmdid & 0x00003fff) << 0);
	rpc_update_cmd_buffer_ptr_encoder(pCmdDesc);

	for (i = 0; i < cmdnum; i++) {
		cmddata = (u_int32 *)(This->cmd_mem_vir+pCmdDesc->wptr - pCmdDesc->start);
		*cmddata = local_cmddata[i];
		rpc_update_cmd_buffer_ptr_encoder(pCmdDesc);
	}
}

u_int32 rpc_MediaIPFW_Video_message_check_encoder(struct shared_addr *This)
{
	u_int32 uSpace;
	u_int32 uIgnore;
	pENC_RPC_HOST_IFACE pSharedInterface = (pENC_RPC_HOST_IFACE)This->shared_mem_vir;
	BUFFER_DESCRIPTOR_TYPE *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	u_int32 msgword;
	u_int32 msgnum;

	uSpace = rpc_MediaIPFW_Video_buffer_space_check_encoder(pMsgDesc, TRUE, 0, &uIgnore);
	uSpace = (uSpace >> 2);
	if (uSpace) {
		/* get current msgword word */
		msgword      = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->rptr - pMsgDesc->start));
		/* Find the number of additional words */
		msgnum  = ((msgword & 0x00ff0000) >> 16);

		/*
		 * * Check the number of message words against
		 * * 1) a limit - some sort of maximum or at least
		 * * the size of the SW buffer the message is read into
		 * * 2) The space reported (where space is write ptr - read ptr in 32bit words)
		 * * It must be less than space (as opposed to <=) because
		 * * the message itself is not included in msgword
		 */
		if (msgnum < VID_API_MESSAGE_LIMIT) {
			if (msgnum < uSpace)
				return API_MSG_AVAILABLE;
			else
				return API_MSG_INCOMPLETE;
		} else
			return API_MSG_BUFFER_ERROR;
	}
	return API_MSG_UNAVAILABLE;
}

static void rpc_update_msg_buffer_ptr_encoder(BUFFER_DESCRIPTOR_TYPE *pMsgDesc)
{
	u_int32 uReadPtr;

	uReadPtr = pMsgDesc->rptr + 4;
	if (uReadPtr >= pMsgDesc->end)
		uReadPtr = pMsgDesc->start;
	pMsgDesc->rptr = uReadPtr;
}

void rpc_receive_msg_buf_encoder(struct shared_addr *This, struct event_msg *msg)
{
	unsigned int i;
	pENC_RPC_HOST_IFACE pSharedInterface = (pENC_RPC_HOST_IFACE)This->shared_mem_vir;
	BUFFER_DESCRIPTOR_TYPE *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	u_int32 msgword = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->rptr - pMsgDesc->start));

	msg->idx = ((msgword & 0xff000000) >> 24);
	msg->msgnum = ((msgword & 0x00ff0000) >> 16);
	msg->msgid = ((msgword & 0x00003fff) >> 0);
	rpc_update_msg_buffer_ptr_encoder(pMsgDesc);

	for (i = 0; i < msg->msgnum; i++) {
		msg->msgdata[i] = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->rptr - pMsgDesc->start));
		rpc_update_msg_buffer_ptr_encoder(pMsgDesc);
	}
}

static void *phy_to_virt(u_int32 src, unsigned long long offset)
{
	void *result;

	result = (void *)(src + offset);
	return result;
}

#define GET_CTRL_INTERFACE_MEMBER(shared_mem, index, name, member) \
	do {\
		pENC_RPC_HOST_IFACE iface = shared_mem->pSharedInterface; \
		pMEDIA_ENC_API_CONTROL_INTERFACE ctrl_interface =\
			phy_to_virt(iface->pEncCtrlInterface[index],\
					shared_mem->base_offset);\
		name = phy_to_virt(ctrl_interface->member,\
				shared_mem->base_offset);\
	} while (0)

pMEDIAIP_ENC_YUV_BUFFER_DESC rpc_get_yuv_buffer_desc(
		struct shared_addr *shared_mem, int index)
{
	pMEDIAIP_ENC_YUV_BUFFER_DESC desc = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index, desc, pEncYUVBufferDesc);

	return desc;
}

pBUFFER_DESCRIPTOR_TYPE rpc_get_stream_buffer_desc(
		struct shared_addr *shared_mem, int index)
{
	pBUFFER_DESCRIPTOR_TYPE desc = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index,
				desc, pEncStreamBufferDesc);

	return desc;
}

pMEDIAIP_ENC_EXPERT_MODE_PARAM rpc_get_expert_mode_param(
		struct shared_addr *shared_mem, int index)
{
	pMEDIAIP_ENC_EXPERT_MODE_PARAM param = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index,
				param, pEncExpertModeParam);

	return param;
}

pMEDIAIP_ENC_PARAM rpc_get_enc_param(
		struct shared_addr *shared_mem, int index)
{
	pMEDIAIP_ENC_PARAM param = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index, param, pEncParam);

	return param;
}

pMEDIAIP_ENC_MEM_POOL rpc_get_mem_pool(
		struct shared_addr *shared_mem, int index)
{
	pMEDIAIP_ENC_MEM_POOL pool = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index, pool, pEncMemPool);

	return pool;
}

pENC_ENCODING_STATUS rpc_get_encoding_status(
		struct shared_addr *shared_mem, int index)
{
	pENC_ENCODING_STATUS encoding_status = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index,
				encoding_status, pEncEncodingStatus);

	return encoding_status;
}

pENC_DSA_STATUS_t rpc_get_dsa_status(struct shared_addr *shared_mem, int index)
{
	pENC_DSA_STATUS_t dsa_status = NULL;

	GET_CTRL_INTERFACE_MEMBER(shared_mem, index, dsa_status, pEncDSAStatus);

	return dsa_status;
}
