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
 * util.h
 *
 ******************************************************************************
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <linux/delay.h>
#include <linux/mutex.h>
#include <drm/drm_modes.h>
#include <drm/drm_edid.h>

/**
 * \addtogroup GENERAL_API
 * \{
 */
/** status code returned by API calls */
typedef enum {
    /** operation succedded */
	CDN_OK = 0,
    /** CEC operation succedded */
	CDN_CEC_ERR_NONE = 0,
    /** mailbox is currently sending or receiving data */
	CDN_BSY,
    /** message set up and ready to be sent, no data sent yet */
	CDN_STARTED,
    /** error encountered while reading/writing APB */
	CDN_ERR,
    /** reply returned with bad opcode */
	CDN_BAD_OPCODE,
    /** reply returned with bad module */
	CDN_BAD_MODULE,
    /** reply not supported mode */
	CDN_ERROR_NOT_SUPPORTED,
    /** Invalid argument passed to CEC API function */
	CDN_CEC_ERR_INVALID_ARG,
    /**
     * TX Buffer for CEC Messages is full. This is applicable only
     * when TX Buffers for CEC Messages are implemented in the HW.
     */
	CDN_CEC_ERR_TX_BUFF_FULL,
    /** No Messages in the RX Buffers are present. */
	CDN_CEC_ERR_RX_BUFF_EMPTY,
    /** Timeout during TX operation */
	CDN_CEC_ERR_TX_TIMEOUT,
    /** Timeout during RX operation */
	CDN_CEC_ERR_RX_TIMEOUT,
    /** Data transmision fail. */
	CDN_CEC_ERR_TX_FAILED,
    /** Data reception fail. */
	CDN_CEC_ERR_RX_FAILED,
    /** Operation aborted. */
	CDN_CEC_ERR_ABORT,
    /** All Logical Addresses are in use. */
	CDN_CEC_ERR_ALL_LA_IN_USE,
} CDN_API_STATUS;

typedef enum {
	CDN_DPTX,
	CDN_HDMITX_TYPHOON,
	CDN_HDMITX_KIRAN,
} CDN_PROTOCOL_TYPE;

typedef enum {
	CDN_BUS_TYPE_APB = 0,
	CDN_BUS_TYPE_SAPB = 1
} CDN_BUS_TYPE;

typedef enum {
	NUM_OF_LANES_1 = 1,
	NUM_OF_LANES_2 = 2,
	NUM_OF_LANES_4 = 4,
} VIC_NUM_OF_LANES;

typedef enum {
	RATE_1_6 = 162,
	RATE_2_1 = 216,
	RATE_2_4 = 243,
	RATE_2_7 = 270,
	RATE_3_2 = 324,
	RATE_4_3 = 432,
	RATE_5_4 = 540,
	RATE_8_1 = 810,
} VIC_SYMBOL_RATE;

typedef enum {
	PXL_RGB = 0x1,
	YCBCR_4_4_4 = 0x2,
	YCBCR_4_2_2 = 0x4,
	YCBCR_4_2_0 = 0x8,
	Y_ONLY = 0x10,
} VIC_PXL_ENCODING_FORMAT;

typedef enum {
	BCS_6 = 0x1,
	BCS_8 = 0x2,
	BCS_10 = 0x4,
	BCS_12 = 0x8,
	BCS_16 = 0x10,
} VIC_COLOR_DEPTH;

typedef enum {
	STEREO_VIDEO_LEFT = 0x0,
	STEREO_VIDEO_RIGHT = 0x1,
} STEREO_VIDEO_ATTR;

typedef enum {
	BT_601 = 0x0,
	BT_709 = 0x1,
} BT_TYPE;

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
	u8 rxend:1;
    /** end of tx message reached */
	u8 txend:1;
} INTERNAL_MBOX_STATUS;

struct hdp_mem {
	void __iomem *regs_base; /* Controller regs base */
	void __iomem *ss_base; /* HDP Subsystem regs base */
	void __iomem *rst_base; /* HDP Subsystem reset base */
	struct mutex mutex;
};

struct hdp_rw_func {
	int (*read_reg) (struct hdp_mem *mem, u32 addr, u32 *value);
	int (*write_reg) (struct hdp_mem *mem, u32 addr, u32 value);
	int (*sread_reg) (struct hdp_mem *mem, u32 addr, u32 *value);
	int (*swrite_reg) (struct hdp_mem *mem, u32 addr, u32 value);
};

typedef struct {
	u8 txBuffer[1024];
	u8 rxBuffer[1024];
	u32 txi;		//iterators
	u32 rxi;
	u8 txEnable;		//data readt to send
	u8 rxEnable;
	u8 running;
	CDN_BUS_TYPE bus_type;
	u32 tmp;
	u32 edp; /* use eDP */

	struct mutex mutex;	//mutex may replace running
	struct hdp_mem *mem;
	struct hdp_rw_func *rw;
} state_struct;
/**
 * \addtogroup UTILS
 * \{
 */
#define INTERNAL_CMD_HEAD_SIZE 4

/**
 * \brief expands to blocking function body
 * \param x - function call
 */
#define MAILBOX_FILL_TIMEOUT	1500
#define internal_block_function(y, x)               \
do {                                                              \
	unsigned long end_jiffies = jiffies + 				\
			msecs_to_jiffies(MAILBOX_FILL_TIMEOUT);           \
    CDN_API_STATUS ret;                                        \
	mutex_lock(y);                                             \
	do {                                                       \
		ret = x;                                               \
		cpu_relax();                                           \
	} while (time_after(end_jiffies, jiffies) &&               \
			(ret == CDN_BSY || ret == CDN_STARTED));           \
	mutex_unlock(y);                                             \
	return ret;                                                \
} while (0)

/**
 * \brief write message and write response (if any), non-blocking way. Also sets state.running = 0
 */
#define internal_process_messages(state)                           \
do {                                                        \
	if (state->txEnable && !internal_mbox_tx_process(state).txend) \
		return CDN_BSY;                                     \
	if (state->rxEnable && !internal_mbox_rx_process(state).rxend) \
		return CDN_BSY;                                     \
	state->running = 0;                                      \
} while (0)

#define internal_opcode_ok_or_return(state, module, opcode) do {                         \
	CDN_API_STATUS ret;                                                           \
	ret = internal_test_rx_head(state, module, opcode);                                  \
	if (ret != CDN_OK)                                                            \
		return ret;                                                               \
} while (0)

#define internal_opcode_match_or_return(state) do {                                           \
	CDN_API_STATUS ret;                                                                  \
	ret = internal_test_rx_head_match(state);                                                 \
	if (ret != CDN_OK)                                                                   \
		return ret;                                                                      \
} while (0)

/* macro for simple tx only command, command format as in mkfullmsg (with count) */
#define internal_macro_command_tx(state, module, opcode, bustype, command...) \
do {                                                                    \
	if (!state->running) {                                                \
		internal_tx_mkfullmsg(state, module, opcode, command);                 \
		state->bus_type = bustype;                                      \
		return CDN_STARTED;                                             \
	}                                                                   \
	internal_process_messages(state);                                          \
} while (0)

/* macro for command with response with matching opcode, command format as in mkfullmsg (with count) */
#define internal_macro_command_txrx(state, module, opcode, bustype, command...) \
do {                                                                      \
	if (!state->running) {                                                 \
		internal_tx_mkfullmsg(state, module, opcode, command);                   \
		state->bus_type = bustype;                                        \
		state->rxEnable = 1;                                               \
		return CDN_STARTED;                                               \
	}                                                                     \
	internal_process_messages(state);                                            \
	internal_opcode_match_or_return(state);                                    \
} while (0)

/**
 * \brief put val into dest in big endian format
 * \param val - value to put
 * \param dest - place to put value
 * \param bytes - true size of val in bytes. for example if bytes = 2 val is treated as short int
 */
void internal_itobe(int val, volatile u8 *dest, int bytes);

/**
 * \brief read big endian value from src and return it
 * \param src - source to read from
 * \param bytes - size of read value
 * \return result
 */
u32 internal_betoi(volatile u8 const *src, u8 bytes);

/**
 * \brief create message from size and value pairs; also sets state.runnging and state.txEnable
 * \param dest - pointer to write message to
 * \param valNo - number of values to write
 * \param ... - pairs of size and value, each value is written after another. if size is positive value, value is written with #internal_itobe, if size is negative, value is treated as src pointer for memcpy
 *
 * example:
 *
 *  u16 x = 0xAABB;
 *
 *  internal_mkmsg(dest, 3, 1, 1, 2, 3, -2, &x);
 *
 *  will write 01 00 03 AA BB to dest
 */
u32 internal_mkmsg(volatile u8 *dest, int valNo, ...);
u32 internal_vmkmsg(volatile u8 *dest, int valNo, va_list vl);

/**
 * \brief setup message header in txBuffer, set txEnable = 1
 */
void internal_mbox_tx_enable(state_struct *state, u8 module, u8 opcode,
			     u16 length);

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
void internal_tx_mkfullmsg(state_struct *state, u8 module, u8 opcode,
			   int valNo, ...);
void internal_vtx_mkfullmsg(state_struct *state, u8 module, u8 opcode,
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
void internal_readmsg(state_struct *state, int valNo, ...);
void internal_vreadmsg(state_struct *state, int valNo, va_list vl);

INTERNAL_MBOX_STATUS internal_mbox_tx_process(state_struct *state);
/**
 * \brief read to rxBuffer from mailbox untill empty or end of message
 *
 * when rxEnable == 0 reads nothing
 * when end of message reached sets rxEnable = 0
 */
INTERNAL_MBOX_STATUS internal_mbox_rx_process(state_struct *state);

/**
 * \brief check if apb is available
 * \return !(rxEnable && txEable)
 */
u32 internal_apb_available(state_struct *state);

/**
 * \brief test if parameters match module and opcode in rxBuffer
 * \return CDN_OK or CDN_BAD_OPCODE or CDN_BAD_MODULE
 */
CDN_API_STATUS internal_test_rx_head(state_struct *state, u8 module,
				     u8 opcode);

CDN_API_STATUS internal_test_rx_head_match(state_struct *state);

/**
 * \brief print current fw and lib version
 */
void print_fw_ver(state_struct *state);

int cdn_apb_read(state_struct *state, u32 addr, u32 *value);
int cdn_sapb_read(state_struct *state, u32 addr, u32 *value);
int cdn_apb_write(state_struct *state, u32 addr, u32 value);
int cdn_sapb_write(state_struct *state, u32 addr, u32 value);
void cdn_sleep(u32 ms);
void cdn_usleep(u32 us);
u16 internal_get_msg_len(state_struct *state);
#endif
