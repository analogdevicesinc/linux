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
 * Copyright 2017-2018 NXP
 *
 ******************************************************************************
 *
 * API_General.c
 *
 ******************************************************************************
 */

#include "API_General.h"
#include "address.h"
#include "apb_cfg.h"
#include "opcodes.h"
#include "general_handler.h"
#include "util.h"

CDN_API_STATUS CDN_API_LoadFirmware(state_struct *state, u8 *iMem,
				    int imemSize, u8 *dMem, int dmemSize)
{
	int i;
	for (i = 0; i < imemSize; i += 4)
		if (cdn_apb_write(state, ADDR_IMEM + i,
				  (u32) iMem[i] << 0 |
				  (u32) iMem[i + 1] << 8 |
				  (u32) iMem[i + 2] << 16 |
				  (u32) iMem[i + 3] << 24))
			return CDN_ERR;
	for (i = 0; i < dmemSize; i += 4)
		if (cdn_apb_write(state, ADDR_DMEM + i,
				  (u32) dMem[i] << 0 |
				  (u32) dMem[i + 1] << 8 |
				  (u32) dMem[i + 2] << 16 |
				  (u32) dMem[i + 3] << 24))
			return CDN_ERR;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo(state_struct *state, u32 val,
					 CDN_BUS_TYPE bus_type)
{
	CDN_API_STATUS ret;
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		state->bus_type = bus_type;
		state->rxEnable = 1;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_TEST_ECHO, 1, 4, val);
		return CDN_STARTED;
	}
	if (state->txEnable && !internal_mbox_tx_process(state).txend)
		return CDN_BSY;
	if (state->rxEnable && !internal_mbox_rx_process(state).rxend)
		return CDN_BSY;
	ret = internal_test_rx_head(state, MB_MODULE_ID_GENERAL,
				    GENERAL_TEST_ECHO);
	if (ret != CDN_OK) {
		state->running = 0;
		return ret;
	}
	state->running = 0;
	if (val != internal_betoi(state->rxBuffer + INTERNAL_CMD_HEAD_SIZE, 4))
		return CDN_ERR;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo_blocking(state_struct *state, u32 val,
						  CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex, CDN_API_General_Test_Echo
				(state, val, bus_type));
}

CDN_API_STATUS CDN_API_General_Test_Echo_Ext(state_struct *state,
					     u8 const *msg, u8 *resp,
					     u16 num_bytes,
					     CDN_BUS_TYPE bus_type)
{
	CDN_API_STATUS ret;

	if (!msg || !resp) {
		return CDN_ERR;
	}

	if ((num_bytes > GENERAL_TEST_ECHO_MAX_PAYLOAD)
	    || (num_bytes < GENERAL_TEST_ECHO_MIN_PAYLOAD)) {
		return CDN_ERR;
	}

	if (!state->running) {
		if (!internal_apb_available(state)) {
			return CDN_BSY;
		}

		state->bus_type = bus_type;
		state->rxEnable = 1;

		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_TEST_ECHO, 1, -num_bytes, msg);

		return CDN_STARTED;
	}

	if (state->txEnable && !internal_mbox_tx_process(state).txend) {
		return CDN_BSY;
	}

	if (state->rxEnable && !internal_mbox_rx_process(state).rxend) {
		return CDN_BSY;
	}

	ret =
	    internal_test_rx_head(state, MB_MODULE_ID_GENERAL,
				  GENERAL_TEST_ECHO);

	if (ret != CDN_OK) {
		state->running = 0;
		return ret;
	}

	state->running = 0;

	memcpy(resp, state->rxBuffer + INTERNAL_CMD_HEAD_SIZE, num_bytes);

	if (memcmp(msg, resp, num_bytes) != 0) {
		return CDN_ERR;
	}

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo_Ext_blocking(state_struct *state,
						      u8 const *msg, u8 *resp,
						      u16 num_bytes,
						      CDN_BUS_TYPE bus_type)
{
	internal_block_function(&state->mutex, CDN_API_General_Test_Echo_Ext
				       (state, msg, resp, num_bytes, bus_type));
}

CDN_API_STATUS CDN_API_General_getCurVersion(state_struct *state, u16 *ver,
					     u16 *verlib)
{
	u32 vh, vl, vlh, vll;
	if (cdn_apb_read(state, VER_L << 2, &vl))
		return CDN_ERR;
	if (cdn_apb_read(state, VER_H << 2, &vh))
		return CDN_ERR;
	if (cdn_apb_read(state, VER_LIB_L_ADDR << 2, &vll))
		return CDN_ERR;
	if (cdn_apb_read(state, VER_LIB_H_ADDR << 2, &vlh))
		return CDN_ERR;
	*ver = F_VER_MSB_RD(vh) << 8 | F_VER_LSB_RD(vl);
	*verlib = F_SW_LIB_VER_H_RD(vlh) << 8 | F_SW_LIB_VER_L_RD(vll);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_Get_Event(state_struct *state, u32 *events)
{
	u32 evt[4] = { 0 };

	if (!events) {
		return CDN_ERR;
	}

	if (cdn_apb_read(state, SW_EVENTS0 << 2, &evt[0])
	    || cdn_apb_read(state, SW_EVENTS1 << 2, &evt[1])
	    || cdn_apb_read(state, SW_EVENTS2 << 2, &evt[2])
	    || cdn_apb_read(state, SW_EVENTS3 << 2, &evt[3])) {
		printk("Failed to read events registers.\n");

		return CDN_ERR;
	}

	*events = (evt[0] & 0xFF)
	    | ((evt[1] & 0xFF) << 8)
	    | ((evt[2] & 0xFF) << 16)
	    | ((evt[3] & 0xFF) << 24);

	return CDN_OK;
}

CDN_API_STATUS CDN_API_Get_Debug_Reg_Val(state_struct *state, u16 *val)
{
	u32 dbg[2] = { 0 };

	if (!val) {
		printk("val pointer is NULL!\n");
		return CDN_ERR;
	}

	if (cdn_apb_read(state, SW_DEBUG_L << 2, &dbg[0])
	    || cdn_apb_read(state, SW_DEBUG_H << 2, &dbg[1])) {
		printk("Failed to read debug registers.\n");

		return CDN_ERR;
	}

	*val = (u16) ((dbg[0] & 0xFF) | ((dbg[1] & 0xFF) << 8));

	return CDN_OK;
}

CDN_API_STATUS CDN_API_CheckAlive(state_struct *state)
{
	u32  alive, newalive;
	u8 retries_left = 10;

	if (cdn_apb_read(state, KEEP_ALIVE << 2, &alive))
		return CDN_ERR;

	while (retries_left--) {
		udelay(1);

		if (cdn_apb_read(state, KEEP_ALIVE << 2, &newalive))
			return CDN_ERR;
		if (alive == newalive)
			continue;
		return CDN_OK;
	}
	return CDN_BSY;
}


CDN_API_STATUS CDN_API_CheckAlive_blocking(state_struct *state)
{
	internal_block_function(&state->mutex, CDN_API_CheckAlive(state));
}

CDN_API_STATUS CDN_API_MainControl(state_struct *state, u8 mode, u8 *resp)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_MAIN_CONTROL, 1, 1, mode);
		return CDN_STARTED;
	}
	internal_process_messages(state);
	internal_opcode_ok_or_return(state, MB_MODULE_ID_GENERAL,
				     GENERAL_MAIN_CONTROL_RESP);
	internal_readmsg(state, 1, 1, resp);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_MainControl_blocking(state_struct *state, u8 mode,
					    u8 *resp)
{
	internal_block_function(&state->mutex, CDN_API_MainControl(state, mode, resp));
}

CDN_API_STATUS CDN_API_ApbConf(state_struct *state, u8 dpcd_bus_sel,
			       u8 dpcd_bus_lock, u8 hdcp_bus_sel,
			       u8 hdcp_bus_lock, u8 capb_bus_sel,
			       u8 capb_bus_lock, u8 *dpcd_resp, u8 *hdcp_resp,
			       u8 *capb_resp)
{
	u8 resp;
	u8 set = 0;

	if (!state->running) {
		if (!internal_apb_available(state)) {
			return CDN_BSY;
		}

		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;

		set |= (dpcd_bus_sel)
		    ? (1 << GENERAL_BUS_SETTINGS_DPCD_BUS_BIT)
		    : 0;
		set |= (dpcd_bus_lock)
		    ? (1 << GENERAL_BUS_SETTINGS_DPCD_BUS_LOCK_BIT)
		    : 0;
		set |= (hdcp_bus_sel)
		    ? (1 << GENERAL_BUS_SETTINGS_HDCP_BUS_BIT)
		    : 0;
		set |= (hdcp_bus_lock)
		    ? (1 << GENERAL_BUS_SETTINGS_HDCP_BUS_LOCK_BIT)
		    : 0;
		set |= (capb_bus_sel)
		    ? (1 << GENERAL_BUS_SETTINGS_CAPB_OWNER_BIT)
		    : 0;
		set |= (capb_bus_lock)
		    ? (1 << GENERAL_BUS_SETTINGS_CAPB_OWNER_LOCK_BIT)
		    : 0;

		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_BUS_SETTINGS, 1, 1, set);

		return CDN_STARTED;
	}

	internal_process_messages(state);
	internal_opcode_ok_or_return(state, MB_MODULE_ID_GENERAL,
				     GENERAL_BUS_SETTINGS_RESP);

	/* Read one one-byte response */
	internal_readmsg(state, 1, 1, &resp);

	*dpcd_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_DPCD_BUS_BIT)) ? 1 : 0;
	*hdcp_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_HDCP_BUS_BIT)) ? 1 : 0;
	*capb_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_CAPB_OWNER_BIT)) ? 1 : 0;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_ApbConf_blocking(state_struct *state, u8 dpcd_bus_sel,
					u8 dpcd_bus_lock,
					u8 hdcp_bus_sel,
					u8 hdcp_bus_lock,
					u8 capb_bus_sel,
					u8 capb_bus_lock,
					u8 *dpcd_resp,
					u8 *hdcp_resp, u8 *capb_resp)
{
	internal_block_function(&state->mutex, CDN_API_ApbConf
				(state, dpcd_bus_sel, dpcd_bus_lock,
				 hdcp_bus_sel, hdcp_bus_lock, capb_bus_sel,
				 capb_bus_lock, dpcd_resp, hdcp_resp,
				 capb_resp));
}

CDN_API_STATUS CDN_API_SetClock(state_struct *state, u8 MHz)
{
	return cdn_apb_write(state, SW_CLK_H << 2, MHz);
}

CDN_API_STATUS CDN_API_GetClock(state_struct *state, u32 *MHz)
{
	return cdn_apb_read(state, SW_CLK_H << 2, MHz);
}

CDN_API_STATUS CDN_API_General_Read_Register(state_struct *state, u32 addr,
					     GENERAL_Read_Register_response *resp)
{
	CDN_API_STATUS ret;
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_READ_REGISTER, 1, 4, addr);
		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	ret = internal_test_rx_head(state, MB_MODULE_ID_GENERAL,
				    GENERAL_READ_REGISTER_RESP);
	if (ret != CDN_OK)
		return ret;
	internal_readmsg(state, 2, 4, &resp->addr, 4, &resp->val);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Read_Register_blocking(state_struct *state,
						      u32 addr, GENERAL_Read_Register_response *resp)
{
	internal_block_function(&state->mutex, CDN_API_General_Read_Register
				(state, addr, resp));
}

CDN_API_STATUS CDN_API_General_Write_Register(state_struct *state, u32 addr,
					      u32 val)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_WRITE_REGISTER, 2, 4, addr, 4,
				      val);
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Write_Register_blocking(state_struct *state,
						       u32 addr, u32 val)
{
	internal_block_function(&state->mutex, CDN_API_General_Write_Register
				(state, addr, val));
}

CDN_API_STATUS CDN_API_General_Write_Field(state_struct *state, u32 addr,
					   u8 startBit, u8 bitsNo, u32 val)
{
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_WRITE_FIELD, 4, 4, addr, 1,
				      startBit, 1, bitsNo, 4, val);
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Write_Field_blocking(state_struct *state,
						    u32 addr, u8 startBit,
						    u8 bitsNo, u32 val)
{
	internal_block_function(&state->mutex, CDN_API_General_Write_Field
				(state, addr, startBit, bitsNo, val));
}

CDN_API_STATUS CDN_API_General_Phy_Test_Access(state_struct *state, u8 *resp)
{
	CDN_API_STATUS ret;

	*resp = 0;

	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL,
				      GENERAL_TEST_ACCESS, 0);
		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;

		return CDN_STARTED;
	}

	internal_process_messages(state);

	ret =
	    internal_test_rx_head(state, MB_MODULE_ID_GENERAL,
				  GENERAL_TEST_ACCESS);

	if (ret != CDN_OK)
		return ret;

	internal_readmsg(state, 1, 1, resp);

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Phy_Test_Access_blocking(state_struct *state,
							u8 *resp)
{
	internal_block_function(&state->mutex, CDN_API_General_Phy_Test_Access(state, resp));
}

CDN_API_STATUS CDN_API_General_GetHpdState(state_struct *state, u8 *hpd_state)
{
	CDN_API_STATUS ret;
	*hpd_state = 0;

	if (!state->running) {
	    if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_GENERAL, GENERAL_GET_HPD_STATE, 0);
		state->bus_type = CDN_BUS_TYPE_APB;
		state->rxEnable = 1;
		return CDN_STARTED;
	}

	internal_process_messages(state);
	ret = internal_test_rx_head(state, MB_MODULE_ID_GENERAL, GENERAL_GET_HPD_STATE);
	if (ret != CDN_OK)
	    return ret;

	internal_readmsg(state, 1, 1, hpd_state);

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_GetHpdState_blocking(state_struct *state, u8 *hpd_state)
{
    internal_block_function(&state->mutex, CDN_API_General_GetHpdState(state, hpd_state));
}
