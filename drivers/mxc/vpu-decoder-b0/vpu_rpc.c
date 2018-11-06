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

#include "vpu_rpc.h"

void rpc_init_shared_memory(struct shared_addr *This,
		unsigned long long base_phy_addr,
		void *base_virt_addr,
		u_int32 total_size)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	unsigned int phy_addr;
	unsigned int i;
	MediaIPFW_Video_BufDesc *pSharedCmdBufDescPtr;
	MediaIPFW_Video_BufDesc *pSharedMsgBufDescPtr;
	MediaIPFW_Video_BufDesc *pDebugBufferDesc;
	MediaIPFW_Video_BufDesc *pEngAccessBufferDesc;

	This->shared_mem_phy = base_phy_addr;
	This->shared_mem_vir = base_virt_addr;

	pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	This->pSharedInterface = pSharedInterface;

	pSharedInterface->FwExecBaseAddr = base_phy_addr;
	pSharedInterface->FwExecAreaSize = total_size;

	pSharedCmdBufDescPtr = (MediaIPFW_Video_BufDesc *)&pSharedInterface->StreamCmdBufferDesc;
	pSharedMsgBufDescPtr = (MediaIPFW_Video_BufDesc *)&pSharedInterface->StreamMsgBufferDesc;

	phy_addr = base_phy_addr + sizeof(DEC_RPC_HOST_IFACE);
	This->cmd_mem_phy = phy_addr;
	This->cmd_mem_vir = This->shared_mem_vir + sizeof(DEC_RPC_HOST_IFACE);

	pSharedCmdBufDescPtr->uWrPtr = phy_addr;
	pSharedCmdBufDescPtr->uRdPtr = pSharedCmdBufDescPtr->uWrPtr;
	pSharedCmdBufDescPtr->uStart = pSharedCmdBufDescPtr->uWrPtr;
	pSharedCmdBufDescPtr->uEnd = pSharedCmdBufDescPtr->uStart + CMD_SIZE;

	phy_addr += CMD_SIZE;
	This->msg_mem_phy = phy_addr;
	This->msg_mem_vir = This->cmd_mem_vir + CMD_SIZE;

	pSharedMsgBufDescPtr->uWrPtr = phy_addr;
	pSharedMsgBufDescPtr->uRdPtr = pSharedMsgBufDescPtr->uWrPtr;
	pSharedMsgBufDescPtr->uStart = pSharedMsgBufDescPtr->uWrPtr;
	pSharedMsgBufDescPtr->uEnd = pSharedMsgBufDescPtr->uStart + MSG_SIZE;

	phy_addr += MSG_SIZE;
	This->codec_mem_phy = phy_addr;
	This->codec_mem_vir = This->msg_mem_vir + MSG_SIZE;
	pSharedInterface->CodecParamTabDesc.pCodecParamArrayBase = This->codec_mem_phy;

	phy_addr += CODEC_SIZE;
	This->jpeg_mem_phy = phy_addr;
	This->jpeg_mem_vir = This->codec_mem_vir + CODEC_SIZE;
	pSharedInterface->JpegParamTabDesc.pJpegParamArrayBase = This->jpeg_mem_phy;

	phy_addr += JPEG_SIZE;
	This->seq_mem_phy = phy_addr;
	This->seq_mem_vir = This->jpeg_mem_vir + JPEG_SIZE;

	pSharedInterface->SeqInfoTabDesc.pSeqInfoArrayBase = This->seq_mem_phy;

	phy_addr += SEQ_SIZE;
	This->pic_mem_phy = phy_addr;
	This->pic_mem_vir = This->seq_mem_vir + SEQ_SIZE;

	pSharedInterface->PicInfoTabDesc.pPicInfoArrayBase = This->pic_mem_phy;

	phy_addr += PIC_SIZE;
	This->gop_mem_phy = phy_addr;
	This->gop_mem_vir = This->pic_mem_vir + PIC_SIZE;

	pSharedInterface->GopInfoTabDesc.pGopInfoArrayBase = This->gop_mem_phy;

	phy_addr += GOP_SIZE;
	This->qmeter_mem_phy = phy_addr;
	This->qmeter_mem_vir = This->gop_mem_vir + GOP_SIZE;

	pSharedInterface->QMeterInfoTabDesc.pQMeterInfoArrayBase = This->qmeter_mem_phy;

	phy_addr += QMETER_SIZE;
	pDebugBufferDesc = &pSharedInterface->DebugBufferDesc;
	pDebugBufferDesc->uWrPtr = base_phy_addr + M0_PRINT_OFFSET;
	pDebugBufferDesc->uRdPtr = pDebugBufferDesc->uWrPtr;
	pDebugBufferDesc->uStart = pDebugBufferDesc->uWrPtr;
	pDebugBufferDesc->uEnd = pDebugBufferDesc->uStart + DEBUG_SIZE;

	phy_addr += sizeof(MediaIPFW_Video_BufDesc);
	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		pEngAccessBufferDesc = &pSharedInterface->EngAccessBufferDesc[i];
		pEngAccessBufferDesc->uWrPtr = phy_addr;
		pEngAccessBufferDesc->uRdPtr = pEngAccessBufferDesc->uWrPtr;
		pEngAccessBufferDesc->uStart = pEngAccessBufferDesc->uWrPtr;
		pEngAccessBufferDesc->uEnd = pEngAccessBufferDesc->uStart + ENG_SIZE;
		phy_addr += ENG_SIZE;
	}

	for (i = 0; i < VPU_MAX_NUM_STREAMS; i++) {
		pSharedInterface->ptEncryptInfo[i] = phy_addr;
		phy_addr += sizeof(MediaIPFW_Video_Encrypt_Info);
	}
}

void rpc_set_stream_cfg_value(void *Interface, u_int32 str_idx)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	u_int32 *CurrStrfg;

	pSharedInterface = (pDEC_RPC_HOST_IFACE)Interface;
	CurrStrfg = &pSharedInterface->StreamConfig[str_idx];
	*CurrStrfg = 0;
	//the value should be passed from application
	VID_STREAM_CONFIG_STRBUFIDX_SET(0, CurrStrfg);
	VID_STREAM_CONFIG_NOSEQ_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_DEBLOCK_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_DERING_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_PLAY_MODE_SET(MEDIA_PLAYER_API_MODE_CONTINUOUS, CurrStrfg);
	VID_STREAM_CONFIG_FS_CTRL_MODE_SET(MEDIA_PLAYER_FS_CTRL_MODE_EXTERNAL, CurrStrfg);
	VID_STREAM_CONFIG_ENABLE_DCP_SET(TRUE, CurrStrfg);
	VID_STREAM_CONFIG_NUM_STR_BUF_SET(1, CurrStrfg);
	VID_STREAM_CONFIG_MALONE_USAGE_SET(1, CurrStrfg);
	VID_STREAM_CONFIG_MULTI_VID_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_OBFUSC_EN_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_RC4_EN_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_MCX_SET(TRUE, CurrStrfg);
	VID_STREAM_CONFIG_PES_SET(FALSE, CurrStrfg);
	VID_STREAM_CONFIG_NUM_DBE_SET(1, CurrStrfg);
}

void rpc_set_system_cfg_value(void *Interface, u_int32 regs_base)
{
	pDEC_RPC_HOST_IFACE pSharedInterface;
	MEDIAIP_FW_SYSTEM_CONFIG *pSystemCfg;

	pSharedInterface = (pDEC_RPC_HOST_IFACE)Interface;
	pSystemCfg = &pSharedInterface->sSystemCfg;
	pSystemCfg->uNumMalones = 1;
	pSystemCfg->uMaloneBaseAddress[0] = (unsigned int)(regs_base + 0x180000);

	pSystemCfg->uMaloneBaseAddress[0x1] = 0x0;
	pSystemCfg->uHifOffset[0x0] = 0x1C000;
	pSystemCfg->uHifOffset[0x1] = 0x0;

	pSystemCfg->uDPVBaseAddr = 0x0;
	pSystemCfg->uDPVIrqPin = 0x0;
	pSystemCfg->uPixIfBaseAddr = (unsigned int)(regs_base + 0x180000 + 0x20000);
	pSystemCfg->uFSLCacheBaseAddr[0] = (unsigned int)(regs_base + 0x60000);
	pSystemCfg->uFSLCacheBaseAddr[1] = (unsigned int)(regs_base + 0x68000);
}

u_int32 rpc_MediaIPFW_Video_buffer_space_check(MediaIPFW_Video_BufDesc *pBufDesc,
	BOOL bFull,
	u_int32 uSize,
	u_int32 *puUpdateAddress)
{
	u_int32 uPtr1;
	u_int32 uPtr2;
	u_int32 uStart;
	u_int32 uEnd;
	u_int32 uTemp;

	/* bFull is FALSE when send message, write data   */
	/* bFull is TRUE when process commands, read data */
	uPtr1 = (bFull) ? pBufDesc->uRdPtr : pBufDesc->uWrPtr;
	uPtr2 = (bFull) ? pBufDesc->uWrPtr : pBufDesc->uRdPtr;

	if (uPtr1 == uPtr2) {
		if (bFull)
			/* No data at all to read */
			return 0;
		else {
			/* wrt pointer equal to read pointer thus the     */
			/* buffer is completely empty for further writes  */
			uStart = pBufDesc->uStart;
			uEnd   = pBufDesc->uEnd;
			/* The address to be returned in this case is for */
			/* the updated write pointer.                     */
			uTemp = uPtr1 + uSize;
			if (uTemp >= uEnd)
				uTemp += (uStart - uEnd);
			*puUpdateAddress = uTemp;
			return (uEnd - uStart);
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
	uStart = pBufDesc->uStart;
	uEnd   = pBufDesc->uEnd;
	uTemp  = uPtr1 + uSize;
	if (uTemp >= uEnd)
		uTemp += (uStart - uEnd);
	*puUpdateAddress = uTemp;
	return ((uEnd - uPtr1) + (uPtr2 - uStart));
}

static void rpc_update_cmd_buffer_ptr(MediaIPFW_Video_BufDesc *pCmdDesc)
{
	u_int32 uWritePtr;

	uWritePtr = pCmdDesc->uWrPtr + 4;
	if (uWritePtr >= pCmdDesc->uEnd)
		uWritePtr = pCmdDesc->uStart;
	pCmdDesc->uWrPtr = uWritePtr;
}

void rpc_send_cmd_buf(struct shared_addr *This,
		u_int32 idx,
		u_int32 cmdid,
		u_int32 cmdnum,
		u_int32 *local_cmddata)
{
	pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	MediaIPFW_Video_BufDesc *pCmdDesc = &pSharedInterface->StreamCmdBufferDesc;
	u_int32 *cmddata;
	u_int32 i;
	u_int32 *cmdword = (u_int32 *)(This->cmd_mem_vir+pCmdDesc->uWrPtr - pCmdDesc->uStart);

	*cmdword = 0;
	*cmdword |= ((idx & 0x000000ff) << 24);
	*cmdword |= ((cmdnum & 0x000000ff) << 16);
	*cmdword |= ((cmdid & 0x00003fff) << 0);
	rpc_update_cmd_buffer_ptr(pCmdDesc);

	for (i = 0; i < cmdnum; i++) {
		cmddata = (u_int32 *)(This->cmd_mem_vir+pCmdDesc->uWrPtr - pCmdDesc->uStart);
		*cmddata = local_cmddata[i];
		rpc_update_cmd_buffer_ptr(pCmdDesc);
	}
}

u_int32 rpc_MediaIPFW_Video_message_check(struct shared_addr *This)
{
	u_int32 uSpace;
	u_int32 uIgnore;
	pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	MediaIPFW_Video_BufDesc *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	u_int32 msgword;
	u_int32 msgnum;

	uSpace = rpc_MediaIPFW_Video_buffer_space_check(pMsgDesc, TRUE, 0, &uIgnore);
	uSpace = (uSpace >> 2);
	if (uSpace) {
		/* get current msgword word */
		msgword      = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->uRdPtr - pMsgDesc->uStart));
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

static void rpc_update_msg_buffer_ptr(MediaIPFW_Video_BufDesc *pMsgDesc)
{
	u_int32 uReadPtr;

	uReadPtr = pMsgDesc->uRdPtr + 4;
	if (uReadPtr >= pMsgDesc->uEnd)
		uReadPtr = pMsgDesc->uStart;
	pMsgDesc->uRdPtr = uReadPtr;
}

void rpc_receive_msg_buf(struct shared_addr *This, struct event_msg *msg)
{
	unsigned int i;
	pDEC_RPC_HOST_IFACE pSharedInterface = (pDEC_RPC_HOST_IFACE)This->shared_mem_vir;
	MediaIPFW_Video_BufDesc *pMsgDesc = &pSharedInterface->StreamMsgBufferDesc;
	u_int32 msgword = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->uRdPtr - pMsgDesc->uStart));

	msg->idx = ((msgword & 0xff000000) >> 24);
	msg->msgnum = ((msgword & 0x00ff0000) >> 16);
	msg->msgid = ((msgword & 0x00003fff) >> 0);
	rpc_update_msg_buffer_ptr(pMsgDesc);

	for (i = 0; i < msg->msgnum; i++) {
		msg->msgdata[i] = *((u_int32 *)(This->msg_mem_vir+pMsgDesc->uRdPtr - pMsgDesc->uStart));
		rpc_update_msg_buffer_ptr(pMsgDesc);
	}
}
