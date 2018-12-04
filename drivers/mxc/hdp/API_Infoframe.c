/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017 NXP
 *
 ******************************************************************************
 *
 * API_Infoframe.c
 *
 ******************************************************************************
 */

#include "API_Infoframe.h"
#include "address.h"
#include "source_pif.h"
#include "util.h"

#define BANK_OFFSET 0x0

static CDN_API_STATUS infoframeSet(state_struct *state, u8 entry_id,
				   u8 packet_len,
				   u8 *packet, u8 packet_type, u8 active_idle)
{
	u32 idx;
	u32 *packet32, len;
	u32 activeIdleBit = (0 == active_idle) ? 0 : 0x20000;

	/* invalidate entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     activeIdleBit | F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	/* flush fifo 1 */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_FIFO1_FLUSH << 2),
	     F_FIFO1_FLUSH(1)))
		return CDN_ERR;

	/* write packet into memory */
	packet32 = (u32 *)packet;
	len = packet_len / 4;
	for (idx = 0; idx < len; idx++)
		if (cdn_apb_write
		    (state,
		     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_DATA_WR << 2),
		     F_DATA_WR(packet32[idx])))
			return CDN_ERR;

	/* write entry id */
	if (cdn_apb_write
	    (state, BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_WR_ADDR << 2),
	     F_WR_ADDR(entry_id)))
		return CDN_ERR;

	/* write request */
	if (cdn_apb_write
	    (state, BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_WR_REQ << 2),
	     F_HOST_WR(1)))
		return CDN_ERR;

	/* update entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     activeIdleBit | F_TYPE_VALID(1) | F_PACKET_TYPE(packet_type) |
	     F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_InfoframeSet(state_struct *state, u8 entry_id,
				    u8 packet_len, u8 *packet, u8 packet_type)
{
	return infoframeSet(state, entry_id, packet_len, packet, packet_type,
			    1);
}

CDN_API_STATUS CDN_API_InfoframeSetNoActiveIdle(state_struct *state,
						u8 entry_id, u8 packet_len,
						u8 *packet, u8 packet_type)
{
	return infoframeSet(state, entry_id, packet_len, packet, packet_type,
			    0);
}

CDN_API_STATUS CDN_API_InfoframeRemove(state_struct *state, u8 entry_id)
{
	/* invalidate entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     0x20000 | F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_InfoframeRemovePacket(state_struct *state, u8 entry_id, u8 packet_type)
{
	/* invalidate entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     0x20000 | F_PKT_ALLOC_ADDRESS(entry_id) |  F_PACKET_TYPE(packet_type)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	return CDN_OK;
}
