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
 * util.h
 *
 ******************************************************************************
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <linux/delay.h>
#include "API_General.h"
/**
 * \addtogroup UTILS
 * \{
 */
# define INTERNAL_CMD_HEAD_SIZE 4

/**
 * \brief expands to blocking function body
 * \param x - function call
 */
# define internal_block_function(x)               \
do {                                             \
    CDN_API_STATUS ret;                          \
    do ret = x;                                  \
    while (ret == CDN_BSY || ret == CDN_STARTED) \
		;                                        \
    return ret;                                  \
} while (0)

/**
 * \brief expands to blocking function body
 * \param x - function call
 * \param y - num of loop
 */
# define internal_block_function_udelay(x, y)    \
do {                                             \
	CDN_API_STATUS ret;                          \
	int i;                                       \
	for (i = 0; i < y; i++) {                    \
		ret = x;                                 \
		if (ret == CDN_OK)                       \
			break;                               \
		udelay(1);                               \
	}                                            \
	if (i == y)                                  \
		printk("timeout %s\n", __func__);        \
	return ret;                                  \
} while (0)

/**
 * \brief write message and write response (if any), non-blocking way. Also sets state.running = 0
 */
# define INTERNAL_PROCESS_MESSAGES                           \
do {                                                        \
	if (state.txEnable && !internal_mbox_tx_process().txend) \
		return CDN_BSY;                                     \
	if (state.rxEnable && !internal_mbox_rx_process().rxend) \
		return CDN_BSY;                                     \
	state.running = 0;                                      \
} while (0)

# define internal_opcode_ok_or_return(module, opcode) do {                         \
	CDN_API_STATUS ret;                                                           \
	ret = internal_test_rx_head(module, opcode);                                  \
	if (ret != CDN_OK)                                                            \
		return ret;                                                               \
} while (0)

# define internal_opcode_match_or_return() do {                                           \
	CDN_API_STATUS ret;                                                                  \
	ret = internal_test_rx_head_match();                                                 \
	if (ret != CDN_OK)                                                                   \
		return ret;                                                                      \
} while (0)

/* macro for simple tx only command, command format as in mkfullmsg (with count) */
# define internal_macro_command_tx(module, opcode, bustype, command...) \
do {                                                                    \
	if (!state.running) {                                                \
		internal_tx_mkfullmsg(module, opcode, command);                 \
		state.bus_type = bustype;                                      \
		return CDN_STARTED;                                             \
	}                                                                   \
	INTERNAL_PROCESS_MESSAGES;                                          \
} while (0)

/* macro for command with response with matching opcode, command format as in mkfullmsg (with count) */
# define internal_macro_command_txrx(module, opcode, bustype, command...) \
do {                                                                      \
	if (!state.running) {                                                 \
		internal_tx_mkfullmsg(module, opcode, command);                   \
		state.bus_type = bustype;                                        \
		state.rxEnable = 1;                                               \
		return CDN_STARTED;                                               \
	}                                                                     \
	INTERNAL_PROCESS_MESSAGES;                                            \
	internal_opcode_match_or_return();                                    \
} while (0)

typedef struct {
    /** apb write status */
	enum tx_status_enum {
	/** one or more bytes written */
		CDN_TX_WRITE = 0,
	/** nothing to write */
		CDN_TX_NOTHING = 1,
	/** mailbox full, 0 bytes written */
		CDN_TX_FULL = 2,
	/** APB error while writing */
		CDN_TX_APB_ERROR = 3
	} tx_status:3;
    /** apb read status */
	enum rx_status_enum {
	/** 1 or more bytes read */
		CDN_RX_READ = 0,
	/** mailbox empty, 0 bytes read */
		CDN_RX_EMPTY = 1,
	/** apb error while reading */
		CDN_RX_APB_ERROR = 2
	} rx_status:2;
    /** indicates end of currenly recived message */
	unsigned char rxend:1;
    /** end of tx message reached */
	unsigned char txend:1;
} INTERNAL_MBOX_STATUS;

/**
 * \brief put val into dest in big endian format
 * \param val - value to put
 * \param dest - place to put value
 * \param bytes - true size of val in bytes. for example if bytes = 2 val is treated as short int
 */
void internal_itobe(int val, volatile unsigned char *dest, int bytes);

/**
 * \brief read big endian value from src and return it
 * \param src - source to read from
 * \param bytes - size of read value
 * \return result
 */
uint32_t internal_betoi(volatile uint8_t const *src, uint8_t bytes);

/**
 * \brief create message from size and value pairs; also sets state.runnging and state.txEnable
 * \param dest - pointer to write message to
 * \param valNo - number of values to write
 * \param ... - pairs of size and value, each value is written after another. if size is positive value, value is written with #internal_itobe, if size is negative, value is treated as src pointer for memcpy
 *
 * example:
 *
 *  unsigned short x = 0xAABB;
 *
 *  internal_mkmsg(dest, 3, 1, 1, 2, 3, -2, &x);
 *
 *  will write 01 00 03 AA BB to dest
 */
unsigned int internal_mkmsg(volatile unsigned char *dest, int valNo, ...);
unsigned int internal_vmkmsg(volatile unsigned char *dest, int valNo,
			     va_list vl);

/**
 * \brief setup message header in txBuffer, set txEnable = 1
 */
void internal_mbox_tx_enable(unsigned char module, unsigned char opcode,
			     unsigned short length);

/**
 * \brief write from txBuffer to mailbox untill full or end of message.
 *
 * when txEnable == 0 writes nothing
 * when write reaches end of message set txEnable = 0
 */

/**
 * \brief combination of #internal_mkmsg and #internal_mbox_tx_enable
 *
 * #internal_mkmsg dest and #internal_mbox_tx_enable length are determined automaticly
 * this function also sets state.txEnable = 1 and state.running
 */
void internal_tx_mkfullmsg(unsigned char module, unsigned char opcode,
			   int valNo, ...);
void internal_vtx_mkfullmsg(unsigned char module, unsigned char opcode,
			    int valNo, va_list vl);

/**
 * \brief read from state.txBuffer and store results in specified pointers
 * \param valNo - numbero of values to read
 * \param ... - pairs of size and ptr
 *
 * this function is similar to #internal_mkmsg -
 *
 * when size is positive read value using #internal_betoi
 * when size is negative mempcy from txBuffer to ptr -size bytes
 * when size is 0 write to ptr addres of current position in rxbuffer
 * when ptr is NULL ignore size bytes (if size is negative this will rewind buffer)
 */
void internal_readmsg(int valNo, ...);
void internal_vreadmsg(int valNo, va_list vl);

INTERNAL_MBOX_STATUS internal_mbox_tx_process(void);
/**
 * \brief read to rxBuffer from mailbox untill empty or end of message
 *
 * when rxEnable == 0 reads nothing
 * when end of message reached sets rxEnable = 0
 */
INTERNAL_MBOX_STATUS internal_mbox_rx_process(void);

/**
 * \brief check if apb is available
 * \return !(rxEnable && txEable)
 */
unsigned int internal_apb_available(void);

/**
 * \brief test if parameters match module and opcode in rxBuffer
 * \return CDN_OK or CDN_BAD_OPCODE or CDN_BAD_MODULE
 */
CDN_API_STATUS internal_test_rx_head(unsigned char module,
				     unsigned char opcode);

CDN_API_STATUS internal_test_rx_head_match(void);

/**
 * \brief print current fw and lib version
 */
void print_fw_ver(void);

typedef struct {
	unsigned char txBuffer[1024];
	unsigned char rxBuffer[1024];
	unsigned int txi;	//iterators
	unsigned int rxi;
	unsigned char txEnable;	//data readt to send
	unsigned char rxEnable;
	unsigned char running;
	CDN_BUS_TYPE bus_type;
	unsigned int tmp;
} state_struct;

extern state_struct state;
extern int cdn_bus_read(unsigned int addr, unsigned int *value);
extern int cdn_bus_write(unsigned int addr, unsigned int value);
unsigned short internal_get_msg_len(void);

#endif
