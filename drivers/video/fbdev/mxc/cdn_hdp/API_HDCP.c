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
 * API_HDCP.c
 *
 ******************************************************************************
 */

#include "API_HDCP.h"
#include "util.h"
#include "address.h"
#include "opcodes.h"
#include "hdcp2.h"

CDN_API_STATUS CDN_API_HDCP_TX_CONFIGURATION(unsigned char val,
					     CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP_TX_CONFIGURATION, 1, 1, val);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP_TX_CONFIGURATION_blocking(unsigned char val,
						      CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP_TX_CONFIGURATION(val, bus_type));
}

CDN_API_STATUS
CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS(S_HDCP_TRANS_PUBLIC_KEY_PARAMS *val,
				       CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_SET_PUBLIC_KEY_PARAMS, 2,
				      -sizeof(val->N), &val->N, -sizeof(val->E),
				      &val->E);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS_blocking(S_HDCP_TRANS_PUBLIC_KEY_PARAMS *
						val, CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS
				(val, bus_type));
}

CDN_API_STATUS CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS(S_HDCP_TRANS_KM_KEY_PARAMS *
						  val, CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;

		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_SET_KM_KEY_PARAMS, 1,
				      -sizeof(val->KM_KEY), &val->KM_KEY);

		return CDN_STARTED;
	}

	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;

	state.running = 0;

	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS_blocking(S_HDCP_TRANS_KM_KEY_PARAMS *val,
					    CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS
				(val, bus_type));
}

CDN_API_STATUS
CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS(S_HDCP_TRANS_DEBUG_RANDOM_NUMBERS *
					  val, CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS, 5,
				      -sizeof(val->KM), &val->KM,
				      -sizeof(val->RN), &val->RN,
				      -sizeof(val->KS), &val->KS,
				      -sizeof(val->RIV), &val->RIV,
				      -sizeof(val->RTX), &val->RTX);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS
    CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS_blocking
    (S_HDCP_TRANS_DEBUG_RANDOM_NUMBERS *val, CDN_BUS_TYPE bus_type) {
	internal_block_function(CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS
				(val, bus_type));
}

CDN_API_STATUS CDN_API_HDCP2_TX_RESPOND_KM(S_HDCP_TRANS_PAIRING_DATA *val,
					   CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		/* pairing info found in storage */
		if (val != NULL)
			internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
					      HDCP2_TX_RESPOND_KM, 4,
					      -sizeof(val->Receiver_ID),
					      &val->Receiver_ID,
					      -sizeof(val->m), &val->m,
					      -sizeof(val->KM), &val->KM,
					      -sizeof(val->EKH), &val->EKH);
		else
			/* no pairing info found in storage */
			internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
					      HDCP2_TX_RESPOND_KM, 0);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP2_TX_RESPOND_KM_blocking(S_HDCP_TRANS_PAIRING_DATA *
						    val, CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP2_TX_RESPOND_KM(val, bus_type));
}

CDN_API_STATUS
CDN_API_HDCP1_TX_SEND_KEYS(S_HDCP_TX_MAIL_BOX_CMD_HDCP1_TX_SEND_KEYS *val,
			   CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX, HDCP1_TX_SEND_KEYS,
				      2, -sizeof(val->AKSV), &val->AKSV,
				      -sizeof(val->KSV), &val->KSV);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS
CDN_API_HDCP1_TX_SEND_KEYS_blocking(S_HDCP_TX_MAIL_BOX_CMD_HDCP1_TX_SEND_KEYS *
				    val, CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP1_TX_SEND_KEYS(val, bus_type));
}

CDN_API_STATUS CDN_API_HDCP1_TX_SEND_RANDOM_AN(unsigned char An[8],
					       CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP1_TX_SEND_RANDOM_AN, 1, -8, An);
		return CDN_STARTED;
	}
	if (state.txEnable && !internal_mbox_tx_process().txend)
		return CDN_BSY;
	state.running = 0;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP1_TX_SEND_RANDOM_AN_blocking(unsigned char An[8],
							CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP1_TX_SEND_RANDOM_AN(An, bus_type));
}

CDN_API_STATUS CDN_API_HDCP_TX_STATUS_REQ(unsigned char *resp,
					  CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP_TX_STATUS_CHANGE, 0);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_match_or_return();
	internal_readmsg(1, -5, resp);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP_TX_STATUS_REQ_blocking(unsigned char resp[5],
						   CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP_TX_STATUS_REQ(resp, bus_type));
}

CDN_API_STATUS CDN_API_HDCP2_TX_IS_KM_STORED_REQ(unsigned char resp[5],
						 CDN_BUS_TYPE bus_type)
{
	printk("_debug: 0\n");
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		printk("_debug: 1\n");
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_IS_KM_STORED, 0);
		printk("_debug: 2\n");
		return CDN_STARTED;
	}
	printk("_debug: 3\n");
	INTERNAL_PROCESS_MESSAGES;
	printk("_debug: 4\n");
	internal_opcode_match_or_return();
	printk("_debug: 5\n");
	internal_readmsg(1, -5, resp);
	printk("_debug: 6\n");
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP2_TX_IS_KM_STORED_REQ_blocking(unsigned char resp[5],
							  CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP2_TX_IS_KM_STORED_REQ
				(resp, bus_type));
}

CDN_API_STATUS CDN_API_HDCP2_TX_STORE_KM_REQ(S_HDCP_TRANS_PAIRING_DATA *resp,
					     CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX, HDCP2_TX_STORE_KM,
				      0);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_match_or_return();
	internal_readmsg(4,
			 -sizeof(resp->Receiver_ID), &resp->Receiver_ID,
			 -sizeof(resp->m), &resp->m,
			 -sizeof(resp->KM), &resp->KM,
			 -sizeof(resp->EKH), &resp->EKH);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP2_TX_STORE_KM_REQ_blocking(S_HDCP_TRANS_PAIRING_DATA
						      *resp,
						      CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP2_TX_STORE_KM_REQ(resp, bus_type));
}

CDN_API_STATUS CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ(unsigned char *num,
							unsigned char *id,
							CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP_TX_IS_RECEIVER_ID_VALID, 0);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_match_or_return();
	internal_readmsg(1, 1, num);
	internal_readmsg(2, 1, NULL, -5 * *num, id);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ_blocking(unsigned char
								 *num,
								 unsigned char
								 *id,
								 CDN_BUS_TYPE
								 bus_type)
{
	internal_block_function(CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ
				(num, id, bus_type));
}

CDN_API_STATUS CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID(unsigned char valid,
							 CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX,
				      HDCP_TX_RESPOND_RECEIVER_ID_VALID, 1, 1,
				      valid);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID_blocking(unsigned char
								  valid,
								  CDN_BUS_TYPE
								  bus_type)
{
	internal_block_function(CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID
				(valid, bus_type));
}

CDN_API_STATUS CDN_API_HDCP_GENERAL_2_SET_LC(unsigned char *lc,
					     CDN_BUS_TYPE bus_type)
{
	internal_macro_command_tx(MB_MODULE_ID_HDCP_GENERAL,
				  HDCP_GENERAL_SET_LC_128, bus_type, 1, -16,
				  lc);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDCP_GENERAL_2_SET_LC_blocking(unsigned char *lc,
						      CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_HDCP_GENERAL_2_SET_LC(lc, bus_type));
}

CDN_API_STATUS CDN_API_TEST_KEYS(unsigned char test_type, unsigned char resp[1],
				 CDN_BUS_TYPE bus_type)
{
	if (!state.running) {
		if (!internal_apb_available())
			return CDN_BSY;
		state.bus_type = bus_type;
		state.rxEnable = 1;
		internal_tx_mkfullmsg(MB_MODULE_ID_HDCP_TX, HDCP_TX_TEST_KEYS,
				      1, 1, test_type);
		return CDN_STARTED;
	}
	INTERNAL_PROCESS_MESSAGES;
	internal_opcode_match_or_return();
	internal_readmsg(1, -1, resp);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_TEST_KEYS_blocking(unsigned char test_type,
					  unsigned char resp[1],
					  CDN_BUS_TYPE bus_type)
{
	internal_block_function(CDN_API_TEST_KEYS(test_type, resp, bus_type));
}
