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
 * API_General.c
 *
 ******************************************************************************
 */

#include "API_General.h"
#include "util.h"
#include "address.h"
#include "apb_cfg.h"
#include "opcodes.h"
#include "general_handler.h"
#include "externs.h"

static unsigned int alive;

void CDN_API_Init(void)
{
	memset(&state, 0, sizeof(state_struct));
}

CDN_API_STATUS CDN_API_LoadFirmware(unsigned char *iMem, int imemSize,
				    unsigned char *dMem, int dmemSize)
{
	int i;
	for (i = 0; i < imemSize; i += 4)
		if (cdn_apb_write(ADDR_IMEM + i,
				  (unsigned int)iMem[i] << 0 |
				  (unsigned int)iMem[i + 1] << 8 |
				  (unsigned int)iMem[i + 2] << 16 |
				  (unsigned int)iMem[i + 3] << 24))
			return CDN_ERR;
	for (i = 0; i < dmemSize; i += 4)
		if (cdn_apb_write(ADDR_DMEM + i,
				  (unsigned int)dMem[i] << 0 |
				  (unsigned int)dMem[i + 1] << 8 |
				  (unsigned int)dMem[i + 2] << 16 |
				  (unsigned int)dMem[i + 3] << 24))
			return CDN_ERR;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo(unsigned int val,
					 CDN_BUS_TYPE bus_type)
{
	CDN_API_STATUS ret;
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL, GENERAL_TEST_ECHO,
				      1, 4, val);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	if (state.rxEnable && !internal_mbox_rx_process().rxend)
		return CDN_BSY;
	ret = internal_test_rx_head(MB_MODULE_ID_GENERAL,
						GENERAL_TEST_ECHO);
	if (ret != CDN_OK) {
		state.running = 0;
		return ret;
	}
	state.running = 0;
	if (val != internal_betoi(state.rxBuffer + INTERNAL_CMD_HEAD_SIZE, 4))
		return CDN_ERR;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo_blocking(unsigned int val,
						  CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_General_Test_Echo(val, bus_type));
}

CDN_API_STATUS CDN_API_General_Test_Echo_Ext(uint8_t const *msg, uint8_t *resp,
					     uint16_t num_bytes,
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

	if (!state.running) {
		if (!internal_apb_available()) {
			return CDN_BSY;
		}

		state.bus_type = bus_type;
		state.rxEnable = 1;

		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL, GENERAL_TEST_ECHO,
				      1, -num_bytes, msg);

		return CDN_STARTED;
	}

	if (state.txEnable && !internal_mbox_tx_process().txend) {
		return CDN_BSY;
	}

	if (state.rxEnable && !internal_mbox_rx_process().rxend) {
		return CDN_BSY;
	}

	ret = internal_test_rx_head(MB_MODULE_ID_GENERAL, GENERAL_TEST_ECHO);

	if (ret != CDN_OK) {
		state.running = 0;
		return ret;
	}

	state.running = 0;

	memcpy(resp, state.rxBuffer + INTERNAL_CMD_HEAD_SIZE, num_bytes);

	if (memcmp(msg, resp, num_bytes) != 0) {
		return CDN_ERR;
	}

	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Test_Echo_Ext_blocking(uint8_t const *msg,
						      uint8_t *resp,
						      uint16_t num_bytes,
						      CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_General_Test_Echo_Ext
				(msg, resp, num_bytes, bus_type)
	    );
}

CDN_API_STATUS CDN_API_General_getCurVersion(unsigned short *ver,
					     unsigned short *verlib)
{
	unsigned int vh, vl, vlh, vll;
	if (cdn_apb_read(VER_L << 2, &vl))
		return CDN_ERR;
	if (cdn_apb_read(VER_H << 2, &vh))
		return CDN_ERR;
	if (cdn_apb_read(VER_LIB_L_ADDR << 2, &vll))
		return CDN_ERR;
	if (cdn_apb_read(VER_LIB_H_ADDR << 2, &vlh))
		return CDN_ERR;
	*ver = F_VER_MSB_RD(vh) << 8 | F_VER_LSB_RD(vl);
	*verlib = F_SW_LIB_VER_H_RD(vlh) << 8 | F_SW_LIB_VER_L_RD(vll);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_Get_Event(uint32_t *events)
{
	uint32_t evt[4] = { 0 };

	if (!events) {
		printk("events pointer is NULL!\n");
		return CDN_ERR;
	}

	if (cdn_apb_read(SW_EVENTS0 << 2, &evt[0])
	    || cdn_apb_read(SW_EVENTS1 << 2, &evt[1])
	    || cdn_apb_read(SW_EVENTS2 << 2, &evt[2])
	    || cdn_apb_read(SW_EVENTS3 << 2, &evt[3])) {
		printk("Failed to read events registers.\n");

		return CDN_ERR;
	}

	*events = (evt[0] & 0xFF)
	    | ((evt[1] & 0xFF) << 8)
	    | ((evt[2] & 0xFF) << 16)
	    | ((evt[3] & 0xFF) << 24);

	return CDN_OK;
}

CDN_API_STATUS CDN_API_Get_Debug_Reg_Val(uint16_t *val)
{
	uint32_t dbg[2] = { 0 };

	if (!val) {
		printk("val pointer is NULL!\n");
		return CDN_ERR;
	}

	if (cdn_apb_read(SW_DEBUG_L << 2, &dbg[0])
	    || cdn_apb_read(SW_DEBUG_H << 2, &dbg[1])) {
		printk("Failed to read debug registers.\n");

		return CDN_ERR;
	}

	*val = (uint16_t) ((dbg[0] & 0xFF) | ((dbg[1] & 0xFF) << 8));

	return CDN_OK;
}

CDN_API_STATUS CDN_API_CheckAlive(void)
{
	unsigned int newalive;
	if (cdn_apb_read(KEEP_ALIVE << 2, &newalive))
		return CDN_ERR;
	if (alive == newalive)
		return CDN_BSY;
	alive = newalive;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_CheckAlive_blocking(void)
{
	internal_block_function(CDN_API_CheckAlive());
}

CDN_API_STATUS CDN_API_MainControl(unsigned char mode, unsigned char *resp)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = CDN_BUS_TYPE_APB;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL, GENERAL_MAIN_CONTROL, 1, 1, mode);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_ok_or_return(MB_MODULE_ID_GENERAL,
				     GENERAL_MAIN_CONTROL_RESP);
	internal_readmsg(1, 1, resp);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_MainControl_blocking(unsigned char mode,
					    unsigned char *resp)
{
	internal_block_function(CDN_API_MainControl(mode, resp));
}

CDN_API_STATUS CDN_API_ApbConf(uint8_t dpcd_bus_sel, uint8_t dpcd_bus_lock,
			       uint8_t hdcp_bus_sel, uint8_t hdcp_bus_lock,
			       uint8_t capb_bus_sel, uint8_t capb_bus_lock,
			       uint8_t *dpcd_resp, uint8_t *hdcp_resp,
			       uint8_t *capb_resp)
{
	uint8_t resp;
	uint8_t set = 0;

	if (!state.running) {
		if (!internal_apb_available()) {
			return CDN_BSY;
		}

		state.bus_type = CDN_BUS_TYPE_APB;
		state.rxEnable = 1;

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

		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL,
				      GENERAL_BUS_SETTINGS, 1, 1, set);

		return CDN_STARTED;
	}

	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_ok_or_return(MB_MODULE_ID_GENERAL,
				     GENERAL_BUS_SETTINGS_RESP);

	/* Read one one-byte response */
	internal_readmsg(1, 1, &resp);

	*dpcd_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_DPCD_BUS_BIT)) ? 1 : 0;
	*hdcp_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_HDCP_BUS_BIT)) ? 1 : 0;
	*capb_resp =
	    (resp & (1 << GENERAL_BUS_SETTINGS_RESP_CAPB_OWNER_BIT)) ? 1 : 0;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_ApbConf_blocking(uint8_t dpcd_bus_sel,
					uint8_t dpcd_bus_lock,
					uint8_t hdcp_bus_sel,
					uint8_t hdcp_bus_lock,
					uint8_t capb_bus_sel,
					uint8_t capb_bus_lock,
					uint8_t *dpcd_resp,
					uint8_t *hdcp_resp,
					uint8_t *capb_resp)
{
	internal_block_function(CDN_API_ApbConf(dpcd_bus_sel, dpcd_bus_lock,
						hdcp_bus_sel, hdcp_bus_lock,
						capb_bus_sel, capb_bus_lock,
						dpcd_resp, hdcp_resp,
						capb_resp));
}

CDN_API_STATUS CDN_API_SetClock(unsigned char MHz)
{
	return cdn_apb_write(SW_CLK_H << 2, MHz);
}

CDN_API_STATUS CDN_API_General_Read_Register(unsigned int addr,
					     GENERAL_Read_Register_response *resp)
{
	CDN_API_STATUS ret;
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL,
				      GENERAL_READ_REGISTER, 1, 4, addr);
		state.bus_type = CDN_BUS_TYPE_APB;
		state.rxEnable = 1;
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	ret = internal_test_rx_head(MB_MODULE_ID_GENERAL,
				   GENERAL_READ_REGISTER_RESP);
	if (ret != CDN_OK)
		return ret;
	internal_readmsg(2, 4, &resp->addr, 4, &resp->val);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Read_Register_blocking(unsigned int addr,
						      GENERAL_Read_Register_response *resp)
{
	internal_block_function(CDN_API_General_Read_Register(addr, resp));
}

CDN_API_STATUS CDN_API_General_Write_Register(unsigned int addr,
					      unsigned int val)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL,
				      GENERAL_WRITE_REGISTER, 2, 4, addr, 4,
				      val);
		state.bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Write_Register_blocking(unsigned int addr,
						       unsigned int val)
{
	internal_block_function(CDN_API_General_Write_Register(addr, val));
}

CDN_API_STATUS CDN_API_General_Write_Field(unsigned int addr,
					   unsigned char startBit,
					   unsigned char bitsNo,
					   unsigned int val)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		internal_tx_mkfullmsg(MB_MODULE_ID_GENERAL, GENERAL_WRITE_FIELD,
				      4, 4, addr, 1, startBit, 1, bitsNo, 4,
				      val);
		state.bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_General_Write_Field_blocking(unsigned int addr,
						    unsigned char startBit,
						    unsigned char bitsNo,
						    unsigned int val)
{
	internal_block_function(CDN_API_General_Write_Field
				(addr, startBit, bitsNo, val));
}
