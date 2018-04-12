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
 * Copyright 2018 NXP
 *
 ******************************************************************************
 *
 * API_HDMIRX.c
 *
 ******************************************************************************
 */

#include "API_HDMIRX.h"
#include "util.h"
#include "opcodes.h"
#include "address.h"
#include "sink_vif.h"

CDN_API_STATUS CDN_API_HDMIRX_ReadEvent(state_struct *state, u8 *Events_5v)
{
	CDN_API_STATUS ret;
	u8 reserved1;
	u8 reserved2;
	u8 reserved3;
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_HDMI_RX, HDMI_RX_READ_EVENTS, 0);
		state->rxEnable = 1;
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	ret = internal_test_rx_head(state, MB_MODULE_ID_HDMI_RX, HDMI_RX_READ_EVENTS);
	if (ret != CDN_OK)
		return ret;
	internal_readmsg(state,
					 4,
					 1,
					 Events_5v,
					 1,
					 &reserved1,
					 1,
					 &reserved2,
					 1,
					 &reserved3);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMIRX_ReadEvent_blocking(
						state_struct *state,
						u8 *Events_5v)
{
	internal_block_function(&state->mutex, CDN_API_HDMIRX_ReadEvent(state, Events_5v));
}

CDN_API_STATUS CDN_API_HDMIRX_Init_blocking(state_struct *state)
{
	CDN_API_STATUS ret;

	ret =
	    CDN_API_General_Write_Register_blocking(state,
						    ADDR_SINK_VIDEO_HD + (VIDEO_UNPACK_CTRL << 2),
						    F_CD_ENABLE(1));
	ret =
	    CDN_API_General_Write_Register_blocking(state,
						    ADDR_SINK_VIDEO_HD + (VANLYZ_CTRL << 2),
						    F_VANLYZ_START(1) |
						    F_VANLYZ_FRAMES_CHECK_EN(1) |
						    F_VANLYZ_FORMAT_FINDER_EN(1));

	return ret;
}

CDN_API_STATUS CDN_API_HDMIRX_Stop_blocking(state_struct *state)
{
	CDN_API_STATUS ret;

	ret =
	    CDN_API_General_Write_Register_blocking(state,
						    ADDR_SINK_VIDEO_HD + (VIDEO_UNPACK_CTRL << 2),
						    F_CD_ENABLE(0));
	ret =
	    CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_VIDEO_HD + (VANLYZ_CTRL << 2), F_VANLYZ_RESET(1));

	return ret;
}

CDN_API_STATUS CDN_API_HDMIRX_SET_EDID(
						state_struct *state,
						u8 segment,
						u8 extension,
						u8 *edid_buff)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state,
						MB_MODULE_ID_HDMI_RX,
						HDMI_RX_SET_EDID,
						3,
						1,
						segment,
						1,
						extension,
						-128,
						edid_buff);
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMIRX_SET_EDID_blocking(
						state_struct *state,
						u8 segment,
						u8 extension,
						u8 *edid_buff)
{
	internal_block_function(&state->mutex,
				CDN_API_HDMIRX_SET_EDID(state, segment, extension, edid_buff));
}

CDN_API_STATUS CDN_API_HDMIRX_SET_SCDC_SLAVE(state_struct *state,
					     S_HDMI_SCDC_SET_MSG *scdcData)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state,
						MB_MODULE_ID_HDMI_RX,
						HDMI_RX_SCDC_SET,
						1,
						-sizeof(S_HDMI_SCDC_SET_MSG),
						scdcData);
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMIRX_SET_SCDC_SLAVE_blocking(
							state_struct *state,
							S_HDMI_SCDC_SET_MSG *scdcData)
{
	internal_block_function(&state->mutex,
				CDN_API_HDMIRX_SET_SCDC_SLAVE(state, scdcData));
}

CDN_API_STATUS CDN_API_HDMIRX_GET_SCDC_SLAVE(state_struct *state,
					     S_HDMI_SCDC_GET_MSG *scdcData)
{
	CDN_API_STATUS ret;
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_HDMI_RX, HDMI_RX_SCDC_GET, 0);
		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	ret = internal_test_rx_head(state, MB_MODULE_ID_HDMI_RX, HDMI_RX_SCDC_GET);
	if (ret != CDN_OK)
		return ret;
	internal_readmsg(state,
			 4,
			 1, &scdcData->source_ver,
			 1, &scdcData->TMDS_Config,
			 1, &scdcData->config_0,
			 -sizeof(scdcData->manufacturerSpecific),
			 &scdcData->manufacturerSpecific);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMIRX_GET_SCDC_SLAVE_blocking(state_struct *state,
						      S_HDMI_SCDC_GET_MSG *scdcData)
{
	internal_block_function(&state->mutex,
				CDN_API_HDMIRX_GET_SCDC_SLAVE(state, scdcData));
}

CDN_API_STATUS CDN_API_HDMIRX_SetHpd(state_struct *state, u8 hpd)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_HDMI_RX, HDMI_RX_SET_HPD, 1, 1, hpd);
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMIRX_SetHpd_blocking(state_struct *state, u8 hpd)
{
	internal_block_function(&state->mutex, CDN_API_HDMIRX_SetHpd(state, hpd));
}
