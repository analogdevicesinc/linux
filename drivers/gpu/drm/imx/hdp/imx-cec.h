/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _IMX_CEC_H_
#define _IMX_CEC_H_

#include <linux/cec.h>

/* regsiter define */
/* register TX_MSG_HEADER */
#define TX_MSG_HEADER 0
#define F_TX_FOLLOWER_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_TX_FOLLOWER_ADDRESS_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_TX_INITIATOR_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_TX_INITIATOR_ADDRESS_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)

/* register TX_MSG_OPCODE */
#define TX_MSG_OPCODE 1
#define F_TX_MSG_OPCODE(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OPCODE_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP1 */
#define TX_MSG_OP1 2
#define F_TX_MSG_OP1(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP1_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP2 */
#define TX_MSG_OP2 3
#define F_TX_MSG_OP2(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP2_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP3 */
#define TX_MSG_OP3 4
#define F_TX_MSG_OP3(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP3_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP4 */
#define TX_MSG_OP4 5
#define F_TX_MSG_OP4(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP4_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP5 */
#define TX_MSG_OP5 6
#define F_TX_MSG_OP5(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP5_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP6 */
#define TX_MSG_OP6 7
#define F_TX_MSG_OP6(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP6_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP7 */
#define TX_MSG_OP7 8
#define F_TX_MSG_OP7(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP7_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP8 */
#define TX_MSG_OP8 9
#define F_TX_MSG_OP8(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP8_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP9 */
#define TX_MSG_OP9 10
#define F_TX_MSG_OP9(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP9_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP10 */
#define TX_MSG_OP10 11
#define F_TX_MSG_OP10(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP10_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP11 */
#define TX_MSG_OP11 12
#define F_TX_MSG_OP11(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP11_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP12 */
#define TX_MSG_OP12 13
#define F_TX_MSG_OP12(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP12_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP13 */
#define TX_MSG_OP13 14
#define F_TX_MSG_OP13(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP13_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_OP14 */
#define TX_MSG_OP14 15
#define F_TX_MSG_OP14(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_TX_MSG_OP14_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register TX_MSG_LENGTH */
#define TX_MSG_LENGTH 16
#define F_TX_MSG_LENGTH(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_TX_MSG_LENGTH_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register TX_MSG_CMD */
#define TX_MSG_CMD 17
#define F_TX_MSG_CMD(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_TX_MSG_CMD_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register TX_WRITE_BUF */
#define TX_WRITE_BUF 18
#define F_TX_WRITE_BUF(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_TX_WRITE_BUF_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register TX_CLEAR_BUF */
#define TX_CLEAR_BUF 19
#define F_TX_CLEAR_BUF(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_TX_CLEAR_BUF_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register RX_MSG_CMD */
#define RX_MSG_CMD 20
#define F_RX_MSG_CMD(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_RX_MSG_CMD_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register RX_CLEAR_BUF */
#define RX_CLEAR_BUF 21
#define F_RX_CLEAR_BUF(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_RX_CLEAR_BUF_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register LOGICAL_ADDRESS_LA0 */
#define LOGICAL_ADDRESS_LA0 22
#define F_MY_LOG_ADDR0(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MY_LOG_ADDR0_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_LOG_ADDR_VALID0(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_LOG_ADDR_VALID0_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register LOGICAL_ADDRESS_LA1 */
#define LOGICAL_ADDRESS_LA1 23
#define F_MY_LOG_ADDR1(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MY_LOG_ADDR1_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_LOG_ADDR_VALID1(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_LOG_ADDR_VALID1_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register LOGICAL_ADDRESS_LA2 */
#define LOGICAL_ADDRESS_LA2 24
#define F_MY_LOG_ADDR2(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MY_LOG_ADDR2_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_LOG_ADDR_VALID2(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_LOG_ADDR_VALID2_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register LOGICAL_ADDRESS_LA3 */
#define LOGICAL_ADDRESS_LA3 25
#define F_MY_LOG_ADDR3(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MY_LOG_ADDR3_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_LOG_ADDR_VALID3(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_LOG_ADDR_VALID3_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register LOGICAL_ADDRESS_LA4 */
#define LOGICAL_ADDRESS_LA4 26
#define F_MY_LOG_ADDR4(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_MY_LOG_ADDR4_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_LOG_ADDR_VALID4(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_LOG_ADDR_VALID4_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register CLK_DIV_MSB */
#define CLK_DIV_MSB 27
#define F_CLK_DIV_MSB(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_CLK_DIV_MSB_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register CLK_DIV_LSB */
#define CLK_DIV_LSB 28
#define F_CLK_DIV_LSB(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_CLK_DIV_LSB_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register CDC_MSG */
#define CDC_MSG 31
#define F_CDC_MSG(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CDC_MSG_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register RX_MSG_DATA1 */
#define RX_MSG_DATA1 64
#define F_RX_FOLLOWER_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_RX_FOLLOWER_ADDRESS_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_RX_INITIATOR_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 4)
#define F_RX_INITIATOR_ADDRESS_RD(x) (((x) & (((1 << 4) - 1) << 4)) >> 4)

/* register RX_MSG_DATA2 */
#define RX_MSG_DATA2 65
#define F_RX_MSG_OPCODE(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OPCODE_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA3 */
#define RX_MSG_DATA3 66
#define F_RX_MSG_OP1(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP1_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA4 */
#define RX_MSG_DATA4 67
#define F_RX_MSG_OP2(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP2_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA5 */
#define RX_MSG_DATA5 68
#define F_RX_MSG_OP3(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP3_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA6 */
#define RX_MSG_DATA6 69
#define F_RX_MSG_OP4(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP4_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA7 */
#define RX_MSG_DATA7 70
#define F_RX_MSG_OP5(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP5_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA8 */
#define RX_MSG_DATA8 71
#define F_RX_MSG_OP6(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP6_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA9 */
#define RX_MSG_DATA9 72
#define F_RX_MSG_OP7(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP7_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA10 */
#define RX_MSG_DATA10 73
#define F_RX_MSG_OP8(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP8_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA11 */
#define RX_MSG_DATA11 74
#define F_RX_MSG_OP9(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP9_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA12 */
#define RX_MSG_DATA12 75
#define F_RX_MSG_OP10(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP10_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA13 */
#define RX_MSG_DATA13 76
#define F_RX_MSG_OP11(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP11_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA14 */
#define RX_MSG_DATA14 77
#define F_RX_MSG_OP12(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP12_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA15 */
#define RX_MSG_DATA15 78
#define F_RX_MSG_OP13(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP13_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_DATA16 */
#define RX_MSG_DATA16 79
#define F_RX_MSG_OP14(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_RX_MSG_OP14_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)

/* register RX_MSG_LENGTH */
#define RX_MSG_LENGTH 80
#define F_RX_MSG_LENGTH(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_RX_MSG_LENGTH_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register RX_MSG_STATUS */
#define RX_MSG_STATUS 81
#define F_RX_MSG_STATUS(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_RX_MSG_STATUS_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register NUM_OF_MSG_RX_BUF */
#define NUM_OF_MSG_RX_BUF 82
#define F_RX_NUM_MSG(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_RX_NUM_MSG_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register TX_MSG_STATUS */
#define TX_MSG_STATUS 83
#define F_TX_MSG_STATUS(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_TX_MSG_STATUS_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)

/* register DB_L_TIMER */
#define DB_L_TIMER 96

/* register DB_M_TIMER */
#define DB_M_TIMER 97

/* register DB_H_TIMER */
#define DB_H_TIMER 98

#define RX_OP_TIMEOUT 10000
#define TX_OP_TIMEOUT 10000

/**
 * CEC Transceiver operation.
 */
enum {
	CEC_TX_STOP,
	CEC_TX_TRANSMIT,
	CEC_TX_ABORT,
	CEC_TX_ABORT_AND_TRANSMIT
};

/**
 * CEC Transceiver status.
 */
enum {
	CEC_STS_IDLE,
	CEC_STS_BUSY,
	CEC_STS_SUCCESS,
	CEC_STS_ERROR
};

/**
 * CEC Receiver operation.
 */
enum {
	CEC_RX_STOP,
	CEC_RX_READ,
	CEC_RX_DISABLE,
	CEC_RX_ABORT_AND_CLR_FIFO
};

struct imx_cec_dev {
	struct cec_adapter *adap;
	struct device *dev;
	struct mutex lock;
	void __iomem *reg_base;

	struct cec_msg msg;
	u32 clk_div;
	struct task_struct *cec_worker;
};

int imx_cec_register(struct imx_cec_dev *cec);
int imx_cec_unregister(struct imx_cec_dev *cec);
#endif
