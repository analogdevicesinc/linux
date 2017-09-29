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
 * util.c
 *
 ******************************************************************************
 */
#include "util.h"
#include "API_General.h"
#include "apb_cfg.h"
#include "opcodes.h"

int cdn_apb_read(state_struct *state, u32 addr, u32 *value)
{
	struct hdp_mem *mem = &state->mem;
	state->rw->read_reg(mem, addr, value);
	return 0;
}

int cdn_apb_write(state_struct *state, u32 addr, u32 value)
{
	struct hdp_mem *mem = &state->mem;
	state->rw->write_reg(mem, addr, value);
	return 0;
}

int cdn_sapb_read(state_struct *state, u32 addr, u32 *value)
{
	struct hdp_mem *mem = &state->mem;
	state->rw->sread_reg(mem, addr, value);
	return 0;
}

int cdn_sapb_write(state_struct *state, u32 addr, u32 value)
{
	struct hdp_mem *mem = &state->mem;
	state->rw->swrite_reg(mem, addr, value);
	return 0;
}

void cdn_sleep(u32 ms)
{
	mdelay(ms);
}

void cdn_usleep(u32 us)
{
	udelay(us);
}

int cdn_bus_read(state_struct *state, u32 addr, u32 *value)
{
	return state->bus_type ?
	    cdn_sapb_read(state, addr, value) : cdn_apb_read(state, addr,
							     value);
}

int cdn_bus_write(state_struct *state, u32 addr, u32 value)
{
	return state->bus_type ?
	    cdn_sapb_write(state, addr, value) : cdn_apb_write(state, addr,
							       value);
}

void internal_itobe(int val, volatile u8 *dest, int bytes)
{
	int i;
	for (i = bytes - 1; i >= 0; --i) {
		dest[i] = (u8) val;
		val >>= 8;
	}
}

u32 internal_betoi(volatile u8 const *src, u8 bytes)
{
	u32 ret = 0;
	int i;

	if (bytes > sizeof(ret)) {
		printk
		    ("Warning. Read request for payload larger then supported.\n");
		bytes = sizeof(ret);

	}

	for (i = 0; i < bytes; ++i) {
		ret <<= 8;
		ret |= (u32) src[i];
	}

	return ret;
}

u32 internal_mkmsg(volatile u8 *dest, int valNo, ...)
{
	va_list vl;
	u32 len = 0;
	va_start(vl, valNo);
	len = internal_vmkmsg(dest, valNo, vl);
	va_end(vl);
	return len;
}

u32 internal_vmkmsg(volatile u8 *dest, int valNo, va_list vl)
{
	u32 len = 0;
	int i;
	for (i = 0; i < valNo; ++i) {
		int size = va_arg(vl, int);
		if (size > 0) {
			internal_itobe(va_arg(vl, int), dest, size);
			dest += size;
			len += size;;
		} else {
			memcpy((void *)dest, va_arg(vl, void *), -size);
			dest -= size;
			len -= size;
		}
	}
	return len;
}

void internal_tx_mkfullmsg(state_struct *state, u8 module, u8 opcode,
			   int valNo, ...)
{
	va_list vl;
	va_start(vl, valNo);
	internal_vtx_mkfullmsg(state, module, opcode, valNo, vl);
	va_end(vl);
}

void internal_vtx_mkfullmsg(state_struct *state, u8 module, u8 opcode,
			    int valNo, va_list vl)
{
	u32 len =
	    internal_vmkmsg(state->txBuffer + INTERNAL_CMD_HEAD_SIZE, valNo,
			    vl);
	internal_mbox_tx_enable(state, module, opcode, len);
	state->txEnable = 1;
	state->running = 1;
}

void internal_readmsg(state_struct *state, int valNo, ...)
{
	va_list vl;
	va_start(vl, valNo);
	internal_vreadmsg(state, valNo, vl);
	va_end(vl);
}

void internal_vreadmsg(state_struct *state, int valNo, va_list vl)
{
	u8 *src = state->rxBuffer + INTERNAL_CMD_HEAD_SIZE;
	size_t i;

	for (i = 0; i < (size_t) valNo; ++i) {
		int size = va_arg(vl, int);
		void *ptr = va_arg(vl, void *);

		if (!ptr) {
			src += size;
		} else if (!size) {
			*((u8 **) ptr) = src;
		} else if (size > 0) {
			switch ((size_t) size) {
			case sizeof(u8):
				*((u8 *) ptr) = internal_betoi(src, size);
				break;
			case sizeof(u16):
				*((u16 *) ptr) = internal_betoi(src, size);
				break;
			case 3:	// 3-byte value (e.g. DPCD address) can be safely converted from BE
			case sizeof(u32):
				*((u32 *) ptr) = internal_betoi(src, size);
				break;
			default:
				pr_warn("Warning. Unsupported variable size.\n");
				memcpy(ptr, src, size);
			};

			src += size;
		} else {
			memcpy(ptr, src, -size);
			src -= size;
		}
	}
}

INTERNAL_MBOX_STATUS mailbox_write(state_struct *state, u8 val)
{
	INTERNAL_MBOX_STATUS ret;
	u32 full;
	if (cdn_bus_read(state, MAILBOX_FULL_ADDR << 2, &full)) {
		ret.tx_status = CDN_TX_APB_ERROR;
		return ret;
	}
	if (full) {
		ret.tx_status = CDN_TX_FULL;
		return ret;
	}
	if (cdn_bus_write(state, MAILBOX0_WR_DATA << 2, val)) {
		ret.tx_status = CDN_TX_APB_ERROR;
		return ret;
	}
	ret.tx_status = CDN_TX_WRITE;
	return ret;
}

INTERNAL_MBOX_STATUS mailbox_read(state_struct *state, volatile u8 *val)
{
	INTERNAL_MBOX_STATUS ret;
	u32 empty;
	u32 rd;
	if (cdn_bus_read(state, MAILBOX_EMPTY_ADDR << 2, &empty)) {
		ret.rx_status = CDN_RX_APB_ERROR;
		return ret;
	}
	if (empty) {
		ret.rx_status = CDN_RX_EMPTY;
		return ret;
	}
	if (cdn_bus_read(state, MAILBOX0_RD_DATA << 2, &rd)) {
		ret.rx_status = CDN_RX_APB_ERROR;
		return ret;
	}
	*val = (u8) rd;
	ret.rx_status = CDN_RX_READ;
	return ret;
}

INTERNAL_MBOX_STATUS internal_mbox_tx_process(state_struct *state)
{
	u32 txCount = 0;
	u32 length = (u32) state->txBuffer[2] << 8 | (u32) state->txBuffer[3];
	INTERNAL_MBOX_STATUS ret = {.txend = 0 };
	INTERNAL_MBOX_STATUS tx_ret;

	ret.tx_status = CDN_TX_NOTHING;
	if (!state->txEnable)
		return ret;
	while ((tx_ret.tx_status =
		mailbox_write(state, state->txBuffer[state->txi]).tx_status) ==
	       CDN_TX_WRITE) {
		txCount++;
		if (++state->txi >= length + 4) {
			state->txEnable = 0;
			state->txi = 0;
			ret.txend = 1;
			break;
		}
	}
	if (txCount && tx_ret.tx_status == CDN_TX_FULL)
		ret.tx_status = CDN_TX_WRITE;
	else
		ret.tx_status = tx_ret.tx_status;
	return ret;
}

INTERNAL_MBOX_STATUS internal_mbox_rx_process(state_struct *state)
{
	u32 rxCount = 0;
	INTERNAL_MBOX_STATUS ret = { 0, 0, 0, 0 };
	INTERNAL_MBOX_STATUS rx_ret;
	while ((rx_ret.rx_status =
		mailbox_read(state, state->rxBuffer + state->rxi).rx_status) ==
	       CDN_RX_READ) {
		rxCount++;
		if (++state->rxi >=
		    4 +
		    ((u32) state->rxBuffer[2] << 8 | (u32) state->
		     rxBuffer[3])) {
			state->rxi = 0;
			ret.rxend = 1;
			state->rxEnable = 0;
			break;
		}
	}
	ret.rx_status = rxCount ? CDN_RX_READ : CDN_RX_EMPTY;
	return ret;
}

u32 internal_apb_available(state_struct *state)
{
	return !(state->rxEnable || state->txEnable);
}

void internal_mbox_tx_enable(state_struct *state, u8 module, u8 opcode,
			     u16 length)
{
	state->txBuffer[0] = opcode;
	state->txBuffer[1] = module;
	state->txBuffer[2] = (u8) (length >> 8);
	state->txBuffer[3] = (u8) length;
	state->txEnable = 1;
}

CDN_API_STATUS internal_test_rx_head(state_struct *state, u8 module, u8 opcode)
{
	if (opcode != state->rxBuffer[0])
		return CDN_BAD_OPCODE;
	if (module != state->rxBuffer[1])
		return CDN_BAD_MODULE;
	return CDN_OK;
}

CDN_API_STATUS internal_test_rx_head_match(state_struct *state)
{
	return internal_test_rx_head(state, state->txBuffer[1],
				     state->txBuffer[0]);
}

void print_fw_ver(state_struct *state)
{
	u16 ver, verlib;
	CDN_API_General_getCurVersion(state, &ver, &verlib);
	printk("FIRMWARE VERSION: %d, LIB VERSION: %d\n", ver, verlib);
}

u16 internal_get_msg_len(state_struct *state)
{
	return ((u16) state->rxBuffer[2] << 8) | (u16) state->rxBuffer[3];
}
