/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mxc_sim_interface.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/types.h>

#define	DRIVER_NAME	"mxc_emvsim"

/* Definitions of the offset of the SIM hardware registers */
#define	EMV_SIM_VER_ID		0X00
#define	EMV_SIM_PARAM		0X04
#define	EMV_SIM_CLKCFG		0X08
#define	EMV_SIM_DIVISOR		0X0C
#define	EMV_SIM_CTRL		0X10
#define	EMV_SIM_INT_MASK	0X14
#define	EMV_SIM_RX_THD		0X18
#define	EMV_SIM_TX_THD		0X1C
#define	EMV_SIM_RX_STATUS	0X20
#define	EMV_SIM_TX_STATUS	0X24
#define	EMV_SIM_PCSR		0X28
#define	EMV_SIM_RX_BUF		0X2C
#define	EMV_SIM_TX_BUF		0X30
#define	EMV_SIM_TX_GETU		0X34
#define	EMV_SIM_CWT_VAL		0X38
#define	EMV_SIM_BWT_VAL		0X3C
#define	EMV_SIM_BGT_VAL		0X40
#define	EMV_SIM_GPCNT0_VAL	0X44
#define	EMV_SIM_GPCNT1_VAL	0X48

#define	SIM_XMT_BUFFER_SIZE	300
#define	SIM_RCV_BUFFER_SIZE	400

#define	SIM_TX_FIFO_DEPTH	16
#define	SIM_RX_FIFO_DEPTH	16
#define	TX_FIFO_THRESHOLD	4

#define	SIM_STATE_REMOVED		0
#define	SIM_STATE_DETECTED		1
#define	SIM_STATE_ATR_RECEIVING		2
#define	SIM_STATE_ATR_RECEIVED		3
#define	SIM_STATE_XMTING		4
#define	SIM_STATE_XMT_DONE		5
#define	SIM_STATE_XMT_ERROR		6
#define	SIM_STATE_RECEIVING		7
#define	SIM_STATE_RECEIVE_DONE		8
#define	SIM_STATE_RECEIVE_ERROR		9
#define	SIM_STATE_RESET_SEQUENCY	10

#define	SIM_CNTL_GPCNT_CARD_CLK		1
#define	SIM_CNTL_GPCNT_RCV_CLK		2
#define	SIM_CNTL_GPCNT_ETU_CLK		3
#define	SIM_EMV_NACK_THRESHOLD		5
#define	EMV_T0_BGT			16
#define	EMV_T1_BGT			22
#define	ATR_THRESHOLD_MAX		100
#define	ATR_MAX_CWT			10080
#define	ATR_MAX_DURATION		20160
#define	FCLK_FREQ			4000000

#define	ATR_TIMEOUT			5
#define	TX_TIMEOUT			10
#define	RX_TIMEOUT			100
#define	RESET_RETRY_TIMES		5
#define	EMV_RESET_LOW_CYCLES		40000
#define	ATR_MAX_DELAY_CLK		46400
#define	DIVISOR_VALUE			372

#define	SIM_CNTL_GPCNT0_CLK_SEL_MASK	(3 << 10)
#define	SIM_CNTL_GPCNT0_CLK_SEL(x)	((x & 3) << 10)
#define	SIM_CNTL_GPCNT1_CLK_SEL_MASK	(3 << 8)
#define	SIM_CNTL_GPCNT1_CLK_SEL(x)	((x & 3) << 8)

/* EMV_SIM_CTRL */
#define	IC		(1 << 0)
#define	ICM		(1 << 1)
#define	ANACK		(1 << 2)
#define	ONACK		(1 << 3)
#define	FLSH_RX		(1 << 8)
#define	FLSH_TX		(1 << 9)
#define	SW_RST		(1 << 10)
#define	KILL_CLOCKS	(1 << 11)
#define	RCV_EN		(1 << 16)
#define	XMT_EN		(1 << 17)
#define	RCVR_11		(1 << 18)
#define	CWT_EN		(1 << 27)
#define	BWT_EN		(1 << 31)

/* EMV_SIM_INT_MASK */
#define	RDT_IM		(1 << 0)
#define	TC_IM		(1 << 1)
#define	ETC_IM		(1 << 3)
#define	TNACK_IM	(1 << 5)
#define	TDT_IM		(1 << 7)
#define	GPCNT0_IM	(1 << 8)
#define	CWT_ERR_IM	(1 << 9)
#define	RNACK_IM	(1 << 10)
#define	BWT_ERR_IM	(1 << 11)
#define	GPCNT1_IM	(1 << 13)
#define	RX_DATA_IM	(1 << 14)

/* EMV_SIM_RX_THD */
#define	SIM_RCV_THRESHOLD_RDT_MASK	(0x0f << 0)
#define	SIM_RCV_THRESHOLD_RDT(x)	((x & 0x0f) << 0)
#define	SIM_RCV_THRESHOLD_RTH_MASK	(0x0f << 8)
#define	SIM_RCV_THRESHOLD_RTH(x)	((x & 0x0f) << 8)

/* EMV_SIM_TX_THD */
#define	SIM_XMT_THRESHOLD_TDT_MASK	(0x0f << 0)
#define	SIM_XMT_THRESHOLD_TDT(x)	((x & 0x0f) << 0)
#define	SIM_XMT_THRESHOLD_XTH_MASK	(0x0f << 8)
#define	SIM_XMT_THRESHOLD_XTH(x)	((x & 0x0f) << 8)

/* EMV_SIM_RX_STATUS */
#define	RDTF		(1 << 5)
#define	CWT_ERR		(1 << 8)
#define	RTE		(1 << 9)
#define	BWT_ERR		(1 << 10)
#define	BGT_ERR		(1 << 11)
#define	PEF		(1 << 12)
#define	FEF		(1 << 13)

/* EMV_SIM_TX_STATUS */
#define	TNTE		(1 << 0)
#define	ETCF		(1 << 4)
#define	TCF		(1 << 5)
#define	TDTF		(1 << 7)
#define	GPCNT0_TO	(1 << 8)
#define	GPCNT1_TO	(1 << 9)

/* EMV_SIM_PCSR */
#define	SAPD		(1 << 0)
#define	SVCC_EN		(1 << 1)
#define	VCCENP		(1 << 2)
#define	SRST		(1 << 3)
#define	SCEN		(1 << 4)
#define	SPD		(1 << 7)
#define	SPDIM		(1 << 24)
#define	SPDIF		(1 << 25)
#define	SPDP		(1 << 26)
#define	SPDES		(1 << 27)

struct emvsim_t {
	s32 present;
	u8 open_cnt;
	int state;
	struct clk *clk;
	struct clk *ipg;
	struct resource *res;
	void __iomem *ioaddr;
	int irq;

	int errval;
	int protocol_type;
	sim_timing_t timing_data;
	sim_baud_t baud_rate;
	int timeout;
	u8 nack_threshold;
	u8 nack_enable;
	u32 expected_rcv_cnt;
	u8 is_fixed_len_rec;
	u32 xmt_remaining;
	u32 xmt_pos;
	u32 rcv_count;
	u8 rcv_buffer[SIM_RCV_BUFFER_SIZE];
	u8 xmt_buffer[SIM_XMT_BUFFER_SIZE];
	struct completion xfer_done;
	u16 rcv_head;
	spinlock_t lock;
	u32 clk_rate;
	u8 checking_ts_timing;
};

static struct miscdevice emvsim_dev;

static void emvsim_data_reset(struct emvsim_t *emvsim)
{
	emvsim->errval = SIM_OK;
	emvsim->protocol_type = 0;
	emvsim->timeout = 0;
	emvsim->nack_threshold = SIM_EMV_NACK_THRESHOLD;
	emvsim->nack_enable = 0;
	memset(&emvsim->timing_data, 0, sizeof(emvsim->timing_data));
	memset(&emvsim->baud_rate, 0, sizeof(emvsim->baud_rate));

	emvsim->xmt_remaining = 0;
	emvsim->xmt_pos = 0;
	emvsim->rcv_count = 0;
	emvsim->rcv_head = 0;
	memset(emvsim->rcv_buffer, 0, SIM_RCV_BUFFER_SIZE);
	memset(emvsim->xmt_buffer, 0, SIM_XMT_BUFFER_SIZE);

	reinit_completion(&emvsim->xfer_done);
};

static void emvsim_set_nack(struct emvsim_t *emvsim, u8 enable)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	/*Disable overrun NACK setting for now*/
	reg_val &= ~ONACK;

	if (enable) {
		reg_val |= ANACK;
		__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);

		reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_THD);
		reg_val &= ~(SIM_XMT_THRESHOLD_XTH_MASK);
		reg_val |= SIM_XMT_THRESHOLD_XTH(emvsim->nack_threshold);
		__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_TX_THD);
	} else {
		reg_val &= ~ANACK;
		__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);
	}

	emvsim->nack_enable = enable;
}

static void emvsim_set_tx(struct emvsim_t *emvsim, u8 enable)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	if (enable) {
		reg_data |= XMT_EN;
		reg_data &= ~RCV_EN;
	} else {
		reg_data &= ~XMT_EN;
	}

	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_CTRL);
}

static void emvsim_set_rx(struct emvsim_t *emvsim, u8 enable)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	if (enable) {
		reg_data |= RCV_EN;
		reg_data &= ~XMT_EN;
	} else {
		reg_data &= ~RCV_EN;
	}

	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_CTRL);
}

static void emvsim_mask_timer0_int(struct emvsim_t *emvsim)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data |= GPCNT0_IM;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS);
	reg_data |= GPCNT0_TO;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_TX_STATUS);
}

static void emvsim_mask_timer1_int(struct emvsim_t *emvsim)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data |= GPCNT1_IM;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS);
	reg_data |= GPCNT1_TO;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_TX_STATUS);
}

static void emvsim_start_timer0(struct emvsim_t *emvsim, u8 clk_source)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CLKCFG);
	reg_data &= ~SIM_CNTL_GPCNT0_CLK_SEL_MASK;
	reg_data |= SIM_CNTL_GPCNT0_CLK_SEL(clk_source);
	writel(reg_data, emvsim->ioaddr + EMV_SIM_CLKCFG);
}

static void emvsim_start_timer1(struct emvsim_t *emvsim, u8 clk_source)
{
	u32 reg_data;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CLKCFG);
	reg_data &= ~SIM_CNTL_GPCNT1_CLK_SEL_MASK;
	reg_data |= SIM_CNTL_GPCNT1_CLK_SEL(clk_source);
	writel(reg_data, emvsim->ioaddr + EMV_SIM_CLKCFG);
}

static int emvsim_reset_low_timing(struct emvsim_t *emvsim, u32 clock_cycle)
{
	int errval = 0;
	int timeout = 0;
	u32 fclk_in_khz, delay_in_us, reg_data;

	fclk_in_khz = emvsim->clk_rate / MSEC_PER_SEC;
	delay_in_us = EMV_RESET_LOW_CYCLES * USEC_PER_MSEC / fclk_in_khz;

	emvsim_mask_timer0_int(emvsim);
	__raw_writel(clock_cycle, emvsim->ioaddr + EMV_SIM_GPCNT0_VAL);
	emvsim_start_timer0(emvsim, SIM_CNTL_GPCNT_CARD_CLK);
	emvsim_set_tx(emvsim, 1);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data &= ~GPCNT0_IM;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

	timeout  = wait_for_completion_timeout(
			&emvsim->xfer_done,
			msecs_to_jiffies(delay_in_us / 1000 * 2));
	if (timeout == 0) {
		dev_err(emvsim_dev.parent, "Reset low GPC timout\n");
		errval =  -SIM_E_TIMEOUT;
	}

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data |= GPCNT0_IM;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

	return errval;
}

static void emvsim_set_cwt(struct emvsim_t *emvsim, u8 enable)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	if (enable && emvsim->timing_data.cwt)
		reg_val |= CWT_EN;
	else
		reg_val &= ~CWT_EN;
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);
}

static void emvsim_set_bwt(struct emvsim_t *emvsim, u8 enable)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	if (enable && (emvsim->timing_data.bwt || emvsim->timing_data.bgt))
		reg_val |= BWT_EN;
	else
		reg_val &= ~BWT_EN;
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);
}

static int emvsim_reset_module(struct emvsim_t *emvsim)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
	reg_val |= SW_RST;
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);

	/* Software should allow a minimum of 4 Protocol clock cycles(4MHz)*/
	usleep_range(1, 3);

	return 0;
}

static void emvsim_receive_atr_set(struct emvsim_t *emvsim)
{
	u32 reg_data;

	emvsim_mask_timer0_int(emvsim);
	emvsim_mask_timer1_int(emvsim);
	__raw_writel(ATR_MAX_DELAY_CLK, emvsim->ioaddr + EMV_SIM_GPCNT0_VAL);
	__raw_writel(0xFFFF, emvsim->ioaddr + EMV_SIM_GPCNT1_VAL);
	emvsim_start_timer0(emvsim, SIM_CNTL_GPCNT_CARD_CLK);
	emvsim_start_timer1(emvsim, SIM_CNTL_GPCNT_ETU_CLK);
	emvsim_set_rx(emvsim, 1);

	emvsim_set_nack(emvsim, 0);
	emvsim->errval = 0;
	emvsim->rcv_count = 0;
	emvsim->checking_ts_timing = 1;
	emvsim->state = SIM_STATE_ATR_RECEIVING;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data &= ~(RDT_IM | GPCNT0_IM);
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);
}

static int32_t emvsim_check_rec_data(u32 *reg_data)
{
	s32 err = 0;

	if (*reg_data & CWT_ERR)
		err |= SIM_ERROR_CWT;

	if (*reg_data & FEF)
		err |= SIM_ERROR_FRAME;

	if (*reg_data & PEF)
		err |= SIM_ERROR_PARITY;

	return err;
}

static void emvsim_xmt_fill_fifo(struct emvsim_t *emvsim)
{
	u32 reg_data;
	u32 bytesleft, i;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS);
	bytesleft = SIM_TX_FIFO_DEPTH - ((reg_data >> 24) & 0x1F);

	if (bytesleft > emvsim->xmt_remaining)
		bytesleft = emvsim->xmt_remaining;

	for (i = 0; i < bytesleft; i++) {
		__raw_writel(emvsim->xmt_buffer[emvsim->xmt_pos],
			     emvsim->ioaddr + EMV_SIM_TX_BUF);
		emvsim->xmt_pos++;
	};
	emvsim->xmt_remaining -= bytesleft;
};

static void emvsim_rcv_read_fifo(struct emvsim_t *emvsim)
{
	u16 i, count;
	u32 reg_data;
	u8 data;

	count  = __raw_readl(emvsim->ioaddr + EMV_SIM_RX_STATUS)  >> 24;

	spin_lock(&emvsim->lock);
	for (i = 0; i < count; i++) {
		reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_RX_STATUS);
		emvsim->errval |= emvsim_check_rec_data(&reg_data);

		/* T1 mode and t0 mode no parity error, T1 mode SIM module
		 * will not produce NACK be NACK is disabled. T0 mode to
		 * ensure there is no parity error for the current byte
		 */
		if (!(emvsim->nack_enable && (reg_data & PEF))) {
			data = __raw_readb(emvsim->ioaddr + EMV_SIM_RX_BUF);
			emvsim->rcv_buffer[emvsim->rcv_head + emvsim->rcv_count] = data;
			emvsim->rcv_count++;
		}

		if (emvsim->rcv_head + emvsim->rcv_count >=
		    SIM_RCV_BUFFER_SIZE) {
			dev_err(emvsim_dev.parent,
				"The software fifo is full,head %d, cnt%d\n",
				emvsim->rcv_head, emvsim->rcv_count);
			break;
		}
	}
	spin_unlock(&emvsim->lock);
}

static void emvsim_tx_irq_enable(struct emvsim_t *emvsim)
{
	u32 reg_val;

	/*Clear the TX&RX status, W1C */
	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS);
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_TX_STATUS);
	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_RX_STATUS);
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_RX_STATUS);

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_val |= CWT_ERR_IM | BWT_ERR_IM | RX_DATA_IM | RDT_IM;

	if (emvsim->xmt_remaining != 0) {
		reg_val &= ~TDT_IM;
	} else {
		reg_val &= ~TC_IM;
		reg_val &= ~ETC_IM;
	}

	/* NACK interrupt is enabled only when T0 mode*/
	if (emvsim->protocol_type == SIM_PROTOCOL_T0 ||
	    emvsim->nack_enable != 0)
		reg_val &= ~TNACK_IM;
	else
		reg_val |= TNACK_IM;

	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_INT_MASK);
}

static void emvsim_tx_irq_disable(struct emvsim_t *emvsim)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_val |= (TDT_IM | TC_IM | TNACK_IM | ETC_IM);
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_INT_MASK);
}

static void emvsim_rx_irq_enable(struct emvsim_t *emvsim)
{
	u32 reg_data;

	 /* Ensure the CWT timer is enabled */
	emvsim_set_cwt(emvsim, 1);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data |= (TC_IM | TDT_IM | TNACK_IM);
	reg_data &= ~(RDT_IM | CWT_ERR_IM | BWT_ERR_IM);

	if (emvsim->protocol_type == SIM_PROTOCOL_T0 ||
	    emvsim->nack_enable != 0)
		reg_data &= ~RNACK_IM;
	else
		reg_data |= RNACK_IM;

	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);
}

static void emvsim_rx_irq_disable(struct emvsim_t *emvsim)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_val |= (RDT_IM | CWT_ERR_IM | BWT_ERR_IM | RNACK_IM);
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_INT_MASK);
}

static irqreturn_t emvsim_irq_handler(int irq, void *dev_id)
{
	u32 reg_data, tx_status, rx_status;
	struct emvsim_t *emvsim = (struct emvsim_t *)dev_id;

	/* clear TX/RX interrupt status, W1C*/
	tx_status  = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS);
	rx_status  = __raw_readl(emvsim->ioaddr + EMV_SIM_RX_STATUS);
	__raw_writel(tx_status, emvsim->ioaddr + EMV_SIM_TX_STATUS);
	__raw_writel(rx_status, emvsim->ioaddr + EMV_SIM_RX_STATUS);

	if (emvsim->state == SIM_STATE_ATR_RECEIVING &&
	    emvsim->checking_ts_timing == 1) {
		if ((tx_status & GPCNT0_TO) && !(rx_status & RDTF)) {
			reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
			reg_data &= ~CWT_EN;
			__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_CTRL);

			reg_data = __raw_readl(emvsim->ioaddr +
					       EMV_SIM_INT_MASK);
			reg_data |= (GPCNT0_IM | CWT_ERR_IM | RDT_IM);
			__raw_writel(reg_data,
				     emvsim->ioaddr + EMV_SIM_INT_MASK);

			emvsim->errval = SIM_ERROR_ATR_DELAY;
			complete(&emvsim->xfer_done);
			emvsim->checking_ts_timing = 0;
		} else if (rx_status & RDTF) {
			u8 rdt = SIM_RX_FIFO_DEPTH >> 1;

			emvsim_mask_timer0_int(emvsim);
			emvsim_rcv_read_fifo(emvsim);

			/* ATR each received byte will cost 12 ETU */
			reg_data = ATR_MAX_DURATION - emvsim->rcv_count * 12;
			__raw_writel(reg_data,
				     emvsim->ioaddr + EMV_SIM_GPCNT1_VAL);

			reg_data = __raw_readl(emvsim->ioaddr +
					       EMV_SIM_INT_MASK);
			reg_data &= ~(GPCNT1_IM | CWT_ERR_IM | RDT_IM);
			__raw_writel(reg_data,
				     emvsim->ioaddr + EMV_SIM_INT_MASK);

			reg_data = SIM_RCV_THRESHOLD_RTH(0) |
				   SIM_RCV_THRESHOLD_RDT(rdt);
			__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_RX_THD);

			/* ATR has arrived as EMV demands */
			emvsim->checking_ts_timing = 0;
		} else {
			dev_err(emvsim_dev.parent,
				"Unexpected irq when delay checking\n");
		}
	}

	else if (emvsim->state == SIM_STATE_ATR_RECEIVING) {
		/*CWT ERROR OR ATR_MAX_DURATION TIMEOUT */
		if ((rx_status & CWT_ERR) ||
		    ((tx_status & GPCNT1_TO) && (emvsim->rcv_count != 0))) {
			reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
			reg_data &= ~CWT_EN;
			__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_CTRL);

			reg_data = __raw_readl(emvsim->ioaddr +
					       EMV_SIM_INT_MASK);
			reg_data |= (GPCNT1_IM | CWT_ERR_IM | RDT_IM);
			__raw_writel(reg_data,
				     emvsim->ioaddr + EMV_SIM_INT_MASK);

			if (tx_status & GPCNT1_TO)
				emvsim->errval |= SIM_ERROR_ATR_TIMEROUT;

			if (rx_status & CWT_ERR)
				emvsim->errval |= SIM_ERROR_CWT;

			emvsim_rcv_read_fifo(emvsim);
			emvsim->state = SIM_STATE_ATR_RECEIVED;

			complete(&emvsim->xfer_done);
		} else if (rx_status & RDTF) {
			/* Receive Data Threshold Interrupt */
			emvsim_rcv_read_fifo(emvsim);
		}
	}

	else if (emvsim->state == SIM_STATE_XMTING) {
		/* need to enable CWT timer */
		if (tx_status & ETCF)
			emvsim_set_cwt(emvsim, 1);

		if (tx_status & TNTE) {
			emvsim_set_tx(emvsim, 0);

			/*Disalbe the timers*/
			emvsim_set_cwt(emvsim, 0);
			emvsim_set_bwt(emvsim, 0);

			/*Disable the NACK interruptand TX related interrupt*/
			emvsim_tx_irq_disable(emvsim);

			/*Update the state and status*/
			emvsim->errval |= SIM_ERROR_NACK_THRESHOLD;
			emvsim->state = SIM_STATE_XMT_ERROR;

			complete(&emvsim->xfer_done);
		} else if (tx_status & TDTF && emvsim->xmt_remaining != 0) {
			emvsim_xmt_fill_fifo(emvsim);
			if (emvsim->xmt_remaining == 0) {
				reg_data = __raw_readl(emvsim->ioaddr +
						       EMV_SIM_INT_MASK);
				reg_data |= TDT_IM;
				reg_data &= ~(TC_IM | ETC_IM);
				__raw_writel(reg_data, emvsim->ioaddr +
					     EMV_SIM_INT_MASK);
			}
		} else if ((tx_status & TCF) && !emvsim->xmt_remaining) {
			emvsim_tx_irq_disable(emvsim);
			emvsim_set_rx(emvsim, 1);
			emvsim->state = SIM_STATE_XMT_DONE;
			complete(&emvsim->xfer_done);
		}
	}

	/*
	 * It takes some time to change from SIM_STATE_XMT_DONE to
	 * SIM_STATE_RECEIVING RX would only be enabled after state
	 * becomes SIM_STATE_RECEIVING
	 */
	else if (emvsim->state == SIM_STATE_RECEIVING) {
		if (rx_status & RTE) {
			emvsim_set_rx(emvsim, 0);

			/* Disable the BWT timer and CWT timer right now */
			emvsim_set_cwt(emvsim, 0);
			emvsim_set_bwt(emvsim, 0);

			/* Disable the interrupt right now */
			emvsim_rx_irq_disable(emvsim);

			/* Should we read the fifo or just flush the fifo? */
			emvsim_rcv_read_fifo(emvsim);
			emvsim->errval = SIM_ERROR_NACK_THRESHOLD;
			emvsim->state = SIM_STATE_RECEIVE_ERROR;
			complete(&emvsim->xfer_done);
		}

		if (rx_status & RDTF) {
			emvsim_rcv_read_fifo(emvsim);
			if (emvsim->is_fixed_len_rec &&
			    emvsim->rcv_count >= emvsim->expected_rcv_cnt) {
				emvsim_rx_irq_disable(emvsim);

				if (emvsim->state == SIM_STATE_RECEIVING) {
					emvsim->state = SIM_STATE_RECEIVE_DONE;
					complete(&emvsim->xfer_done);
				}
			}
		}

		if (rx_status & (CWT_ERR | BWT_ERR | BGT_ERR)) {
			emvsim_set_cwt(emvsim, 0);
			emvsim_set_bwt(emvsim, 0);
			emvsim_rx_irq_disable(emvsim);

			if (rx_status & BWT_ERR)
				emvsim->errval |= SIM_ERROR_BWT;
			if (rx_status & CWT_ERR)
				emvsim->errval |= SIM_ERROR_CWT;
			if (rx_status & BGT_ERR)
				emvsim->errval |= SIM_ERROR_BGT;

			emvsim_rcv_read_fifo(emvsim);

			if (emvsim->state == SIM_STATE_RECEIVING) {
				emvsim->state = SIM_STATE_RECEIVE_DONE;
				complete(&emvsim->xfer_done);
			}
		}
	}

	else if ((emvsim->state == SIM_STATE_RESET_SEQUENCY) &&
		 (tx_status & GPCNT0_TO)) {
		complete(&emvsim->xfer_done);
		emvsim_mask_timer0_int(emvsim);
	} else if (rx_status & RDTF) {
		dev_err(emvsim_dev.parent,
			"unexpected  status %d\n", emvsim->state);
		emvsim_rcv_read_fifo(emvsim);
	}

	return IRQ_HANDLED;
};

static void emvsim_start(struct emvsim_t *emvsim)
{
	u32 reg_data, clk_rate, clk_div = 0;

	clk_rate = clk_get_rate(emvsim->clk);
	clk_div = (clk_rate + emvsim->clk_rate - 1) / emvsim->clk_rate;
	__raw_writel(clk_div, emvsim->ioaddr + EMV_SIM_CLKCFG);

	/* SPDP=0: SIM Presence Detect pin is low, default PRESENT status */
	if (__raw_readl(emvsim->ioaddr + EMV_SIM_PCSR) & SPDP) {
		emvsim->present = SIM_PRESENT_REMOVED;
		emvsim->state = SIM_STATE_REMOVED;
	} else {
		emvsim->present = SIM_PRESENT_DETECTED;
		emvsim->state = SIM_STATE_DETECTED;
	};

	/* disabled card interrupt. clear interrupt status*/
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data |= SPDIM | SPDIF;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);
};

static void emvsim_cold_reset_sequency(struct emvsim_t *emvsim)
{
	u32 reg_data;

	emvsim->state = SIM_STATE_RESET_SEQUENCY;

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data &= ~VCCENP;
	reg_data |= SVCC_EN;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);

	msleep(20);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data |= SCEN;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);

	emvsim_reset_low_timing(emvsim, EMV_RESET_LOW_CYCLES);

	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data |= SRST;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);
};

static void emvsim_deactivate(struct emvsim_t *emvsim)
{
	u32 reg_data;

	/* Auto powdown to implement the deactivate sequence */
	if (emvsim->present != SIM_PRESENT_REMOVED) {
		reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
		reg_data |= SAPD | SPD;
		writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);
	} else {
		dev_err(emvsim_dev.parent, ">>>No card%s\n", __func__);
	}
};

static void emvsim_cold_reset(struct emvsim_t *emvsim)
{
	if (emvsim->present != SIM_PRESENT_REMOVED) {
		emvsim->state = SIM_STATE_DETECTED;
		emvsim->present = SIM_PRESENT_DETECTED;
		emvsim_cold_reset_sequency(emvsim);
		emvsim_receive_atr_set(emvsim);
	} else {
		dev_err(emvsim_dev.parent, "No card%s\n", __func__);
	}
};

static void emvsim_warm_reset_sequency(struct emvsim_t *emvsim)
{
	u32 reg_data;

	/*enable power/clk, deassert rst*/
	emvsim->state = SIM_STATE_RESET_SEQUENCY;
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data |= (SRST | SCEN);
	reg_data &= ~VCCENP;
	reg_data |= SVCC_EN;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);

	usleep_range(20, 25);

	/* assert rst */
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data &= ~SRST;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);

	/* rst keep low */
	emvsim_reset_low_timing(emvsim, EMV_RESET_LOW_CYCLES);

	/* deassert rst */
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	reg_data |= SRST;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_PCSR);
}

static void emvsim_warm_reset(struct emvsim_t *emvsim)
{
	if (emvsim->present != SIM_PRESENT_REMOVED) {
		emvsim_data_reset(emvsim);
		emvsim_warm_reset_sequency(emvsim);
		emvsim_receive_atr_set(emvsim);
	} else {
		dev_err(emvsim_dev.parent, "No card%s\n", __func__);
	}
};

static int emvsim_card_lock(struct emvsim_t *emvsim)
{
	int errval;

	/* place holder for true physcial locking */
	if (emvsim->present != SIM_PRESENT_REMOVED)
		errval = SIM_OK;
	else
		errval = -SIM_E_NOCARD;

	return errval;
};

static int emvsim_card_eject(struct emvsim_t *emvsim)
{
	int errval;

	/* place holder for true physcial locking */
	if (emvsim->present != SIM_PRESENT_REMOVED)
		errval = SIM_OK;
	else
		errval = -SIM_E_NOCARD;

	return errval;
};

static int emvsim_check_baud_rate(sim_baud_t *baud_rate)
{
	/* The valid value is decribed in the 8.3.3.1 in EMV 4.3 */
	if (baud_rate->fi == 1 && (baud_rate->di == 1 ||
	    baud_rate->di == 2 || baud_rate->di == 3))
		return 0;

	return -EINVAL;
}

static int emvsim_set_baud_rate(struct emvsim_t *emvsim)
{
	u32 reg_data;

	switch (emvsim->baud_rate.di) {
	case 1:
		reg_data = 372;
		break;
	case 2:
		reg_data = 372 >> 1;
		break;
	case 3:
		reg_data = 372 >> 2;
		break;
	default:
		dev_err(emvsim_dev.parent,
			"Invalid baud Di, Using default 372 / 1\n");
		reg_data = 372;
		break;
	}

	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_DIVISOR);

	return 0;
}

static int emvsim_check_timing_data(sim_timing_t *timing_data)
{
	if (timing_data->wwt > 0xFFFF || timing_data->cwt > 0xFFFF ||
	    timing_data->bgt > 0xFFFF || timing_data->cgt > 0xFF) {
		dev_err(emvsim_dev.parent,
			"The timing value is out of scope of IP\n");
		return -EINVAL;
	}

	return 0;
}

static void emvsim_set_timer_counter(struct emvsim_t *emvsim)
{
	u32 reg;

	if (emvsim->timing_data.wwt != 0 &&
	    emvsim->protocol_type == SIM_PROTOCOL_T0) {
		emvsim->timing_data.cwt = emvsim->timing_data.wwt;
		emvsim->timing_data.bwt = emvsim->timing_data.wwt;
	}

	if (emvsim->timing_data.bgt != 0)
		__raw_writel(emvsim->timing_data.bgt,
			     emvsim->ioaddr + EMV_SIM_BGT_VAL);

	if (emvsim->timing_data.cwt != 0)
		__raw_writel(emvsim->timing_data.cwt,
			     emvsim->ioaddr + EMV_SIM_CWT_VAL);

	if (emvsim->timing_data.bwt != 0)
		__raw_writel(emvsim->timing_data.bwt,
			     emvsim->ioaddr + EMV_SIM_BWT_VAL);

	/* 11 etu and 12 etu, T0: 12ETU; T1: 11ETU */
	if (emvsim->protocol_type == SIM_PROTOCOL_T0) {
		/*
		 * From EMV4.3 , T0 mode means 12 ETU. TotalETU=12+CGT.
		 * If cgt equals 0xFF, TotalETU = 12
		 */
		reg = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
		reg &= ~RCVR_11;
		 __raw_writel(reg, emvsim->ioaddr + EMV_SIM_CTRL);

		/* set Transmitter Guard Time Value in ETU */
		if (emvsim->timing_data.cgt == 0xFF)
			__raw_writel(0, emvsim->ioaddr + EMV_SIM_TX_GETU);
		else
			__raw_writel(emvsim->timing_data.cgt,
				     emvsim->ioaddr + EMV_SIM_TX_GETU);
	} else if (emvsim->protocol_type == SIM_PROTOCOL_T1) {
		/* From EMV4.3 , T1 mode means 11 ETU. TotalETU=11+CGT */
		reg = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
		reg |= RCVR_11;
		__raw_writel(reg, emvsim->ioaddr + EMV_SIM_CTRL);
		__raw_writel(emvsim->timing_data.cgt,
			     emvsim->ioaddr + EMV_SIM_TX_GETU);
	}
}

static int emvsim_xmt_start(struct emvsim_t *emvsim)
{
	u32 reg_val;

	emvsim->state = SIM_STATE_XMTING;

	emvsim_set_baud_rate(emvsim);
	if (emvsim->protocol_type == SIM_PROTOCOL_T0) {
		emvsim_set_nack(emvsim, 1);
	} else if (emvsim->protocol_type == SIM_PROTOCOL_T1) {
		emvsim_set_nack(emvsim, 0);
	} else {
		dev_err(emvsim_dev.parent, "Invalid protocol not T0 or T1\n");
		return -EINVAL;
	}

	emvsim_set_timer_counter(emvsim);

	if (emvsim->xmt_remaining != 0) {
		reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_TX_THD);
		reg_val &= ~SIM_XMT_THRESHOLD_TDT_MASK;
		reg_val |= SIM_XMT_THRESHOLD_TDT(TX_FIFO_THRESHOLD);
		__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_TX_THD);
	}

	emvsim_set_bwt(emvsim, 1);
	emvsim_set_cwt(emvsim, 0);

	emvsim_set_tx(emvsim, 1);
	emvsim_xmt_fill_fifo(emvsim);
	emvsim_tx_irq_enable(emvsim);

	return 0;
}

static void emvsim_flush_fifo(struct emvsim_t *emvsim, u8 flush_tx, u8 flush_rx)
{
	u32 reg_val;

	reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);

	if (flush_tx)
		reg_val |= FLSH_TX;
	if (flush_rx)
		reg_val |= FLSH_RX;
	__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_CTRL);
}

static void emvsim_change_rcv_threshold(struct emvsim_t *emvsim)
{
	u32 rx_threshold = 0;
	u32 reg_val = 0;

	if (emvsim->is_fixed_len_rec) {
		rx_threshold = emvsim->expected_rcv_cnt - emvsim->rcv_count;
		if (rx_threshold > (SIM_RX_FIFO_DEPTH  >> 1))
			rx_threshold = (SIM_RX_FIFO_DEPTH  >> 1);
		reg_val = __raw_readl(emvsim->ioaddr + EMV_SIM_RX_THD);
		reg_val &= ~(SIM_RCV_THRESHOLD_RDT_MASK);
		reg_val |= SIM_RCV_THRESHOLD_RDT(rx_threshold);
		__raw_writel(reg_val, emvsim->ioaddr + EMV_SIM_RX_THD);
	}
}

static void emvsim_start_rcv(struct emvsim_t *emvsim)
{
	int rdt = 1;

	emvsim->state = SIM_STATE_RECEIVING;

	emvsim_set_rx(emvsim, 1);
	emvsim_set_baud_rate(emvsim);
	emvsim_set_timer_counter(emvsim);
	emvsim_set_cwt(emvsim, 1);
	emvsim_set_bwt(emvsim, 1);

	if (emvsim->protocol_type == SIM_PROTOCOL_T0)
		emvsim_set_nack(emvsim, 1);
	else if (emvsim->protocol_type == SIM_PROTOCOL_T1)
		emvsim_set_nack(emvsim, 0);

	/*Set RX threshold*/
	if (emvsim->protocol_type == SIM_PROTOCOL_T0)
		__raw_writel(SIM_RCV_THRESHOLD_RTH(emvsim->nack_threshold) |
			     SIM_RCV_THRESHOLD_RDT(rdt),
			     emvsim->ioaddr + EMV_SIM_RX_THD);
	else
		__raw_writel(SIM_RCV_THRESHOLD_RDT(rdt),
			     emvsim->ioaddr + EMV_SIM_RX_THD);

	/*Clear status and enable interrupt*/
	emvsim_rx_irq_enable(emvsim);
}

static void emvsim_polling_delay(struct emvsim_t *emvsim, u32 delay)
{
	u32 reg_data;
	unsigned long orig_jiffies = jiffies;

	emvsim_mask_timer1_int(emvsim);

	__raw_writel(delay, emvsim->ioaddr + EMV_SIM_GPCNT0_VAL);
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
	reg_data &= ~GPCNT1_IM;
	__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

	/* Loop for timeout, add timeout mechanism to avoid dead loop */
	while (!(__raw_readl(emvsim->ioaddr + EMV_SIM_TX_STATUS) & GPCNT0_TO)) {
		if (time_after(jiffies, orig_jiffies + msecs_to_jiffies(500))) {
			dev_err(emvsim_dev.parent, "polling delay timeout\n");
			break;
		}

		usleep_range(10, 20);
	}

	emvsim_mask_timer1_int(emvsim);
}

void emvsim_clear_rx_buf(struct emvsim_t *emvsim)
{
	unsigned int i;

	for (i = 0; i < SIM_RCV_BUFFER_SIZE; i++)
		emvsim->rcv_buffer[i] = 0;
	emvsim->rcv_count = 0;
	emvsim->rcv_head = 0;
}

static long emvsim_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int ret, errval = SIM_OK;
	unsigned long timeout;
	u32 reg_data;
	u32 delay;
	u32 copy_cnt, val;
	unsigned long flags;
	unsigned char __user *atr_buffer;
	unsigned char __user *xmt_buffer;
	unsigned char __user *rcv_buffer;

	struct emvsim_t *emvsim = (struct emvsim_t *)file->private_data;

	switch (cmd) {
	case SIM_IOCTL_GET_ATR:
		if (emvsim->present != SIM_PRESENT_DETECTED) {
			dev_err(emvsim_dev.parent, "NO card ...\n");
			errval = -SIM_E_NOCARD;
			break;
		}

		emvsim->timeout = ATR_TIMEOUT * HZ;
		val = 0;
		ret = copy_to_user(&(((sim_atr_t *)arg)->size), &val,
				   sizeof((((sim_atr_t *)arg)->size)));

		timeout = wait_for_completion_interruptible_timeout(
				&emvsim->xfer_done, emvsim->timeout);

		reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_CTRL);
		reg_data &= ~CWT_EN;
		__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_CTRL);

		reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_INT_MASK);
		reg_data |= (GPCNT0_IM | CWT_ERR_IM);
		__raw_writel(reg_data, emvsim->ioaddr + EMV_SIM_INT_MASK);

		if (timeout == 0) {
			dev_err(emvsim_dev.parent, "ATR timeout\n");
			errval = -SIM_E_TIMEOUT;
			break;
		}

		ret = copy_to_user(&(((sim_atr_t *)arg)->size),
				   &emvsim->rcv_count,
				   sizeof(emvsim->rcv_count));
		if (ret) {
			dev_err(emvsim_dev.parent,
				"ATR ACCESS rcv_count Error, %d\n", ret);
			errval = -SIM_E_ACCESS;
			break;
		}

		__get_user(atr_buffer, &((sim_atr_t __user *)arg)->atr_buffer);
		ret = copy_to_user(atr_buffer,
				   emvsim->rcv_buffer, emvsim->rcv_count);
		if (ret) {
			dev_err(emvsim_dev.parent,
				"ATR ACCESS buffer Error %d %d\n",
				emvsim->rcv_count, ret);
			errval = -SIM_E_ACCESS;
			break;
		}

		ret = copy_to_user(&(((sim_atr_t *)arg)->errval),
				   &emvsim->errval, sizeof(emvsim->errval));
		if (ret) {
			dev_err(emvsim_dev.parent, "ATR ACCESS Error\n");
			errval = -SIM_E_ACCESS;
			break;
		}
		emvsim->rcv_count = 0;
		emvsim->rcv_head = 0;
		emvsim->errval = 0;

		break;

	case SIM_IOCTL_DEACTIVATE:
		emvsim_deactivate(emvsim);
		break;

	case SIM_IOCTL_COLD_RESET:
		emvsim->present = SIM_PRESENT_REMOVED;
		emvsim->state = SIM_STATE_REMOVED;
		emvsim_reset_module(emvsim);
		emvsim_data_reset(emvsim);
		emvsim_start(emvsim);
		emvsim_cold_reset(emvsim);
		break;

	case SIM_IOCTL_WARM_RESET:
		emvsim_warm_reset(emvsim);
		break;

	case SIM_IOCTL_XMT:
		ret = copy_from_user(&emvsim->xmt_remaining,
				     &(((sim_xmt_t *)arg)->xmt_length),
				     sizeof(uint32_t));
		if (ret || emvsim->xmt_remaining > SIM_XMT_BUFFER_SIZE) {
			dev_err(emvsim_dev.parent,
				"copy error or to big buffer\n");
			errval = -EINVAL;
			break;
		}

		__get_user(xmt_buffer, &((sim_xmt_t *)arg)->xmt_buffer);
		ret = copy_from_user(emvsim->xmt_buffer, xmt_buffer,
				     emvsim->xmt_remaining);
		if (ret) {
			dev_err(emvsim_dev.parent, "Copy Error\n");
			errval = ret;
			break;
		}

		emvsim_clear_rx_buf(emvsim);
		emvsim_set_cwt(emvsim, 0);
		emvsim_set_bwt(emvsim, 0);
		/*Flush the tx rx fifo*/
		emvsim_flush_fifo(emvsim, 1, 1);
		emvsim->xmt_pos = 0;
		emvsim->errval = 0;

		errval = emvsim_xmt_start(emvsim);
		if (errval)
			break;

		emvsim->timeout = TX_TIMEOUT * HZ;
		timeout = wait_for_completion_interruptible_timeout(
				&emvsim->xfer_done, emvsim->timeout);
		if (timeout == 0) {
			/*Disable the NACK interruptand TX related interrupt*/
			emvsim_tx_irq_disable(emvsim);
			dev_err(emvsim_dev.parent, "tx timeout\n");
		}

		if (timeout == 0 || emvsim->state == SIM_STATE_XMT_ERROR) {
			dev_err(emvsim_dev.parent, "TX error\n");
			/*Disable timers*/
			emvsim_set_cwt(emvsim, 0);
			emvsim_set_bwt(emvsim, 0);
			/*Disable TX*/
			emvsim_set_tx(emvsim, 0);
			/*Flush the tx fifos*/
			emvsim_flush_fifo(emvsim, 1, 0);
			if (timeout == 0)
				errval = -SIM_E_TIMEOUT;
			else
				errval = -SIM_E_NACK;

			ret = copy_to_user(&(((sim_atr_t *)arg)->errval),
					   &emvsim->errval,
					   sizeof(emvsim->errval));
			emvsim->errval = 0;
			break;
		}

		/*Copy the error status to user space*/
		ret = copy_to_user(&(((sim_atr_t *)arg)->errval),
				   &emvsim->errval, sizeof(emvsim->errval));
		emvsim->errval = 0;

		emvsim_start_rcv(emvsim);

		break;

	case SIM_IOCTL_RCV:
		if (emvsim->present != SIM_PRESENT_DETECTED) {
			errval = -SIM_E_NOCARD;
			break;
		}

		val = 0;
		emvsim->is_fixed_len_rec = 0;
		ret = copy_from_user(&emvsim->expected_rcv_cnt,
				     &(((sim_rcv_t *)arg)->rcv_length),
				     sizeof(emvsim->expected_rcv_cnt));

		/*Set the length to be 0 at first*/
		ret = copy_to_user(&(((sim_rcv_t *)arg)->rcv_length), &val,
				   sizeof(val));

		/*Set error value to be 0 at first*/
		ret = copy_to_user(&(((sim_rcv_t *)arg)->errval), &val,
				   sizeof(val));

		if (emvsim->expected_rcv_cnt != 0)
			emvsim->is_fixed_len_rec = 1;

		if (emvsim->is_fixed_len_rec &&
		    emvsim->rcv_count >= emvsim->expected_rcv_cnt)
			goto copy_data;

		if (emvsim->state != SIM_STATE_RECEIVING)
			emvsim_start_rcv(emvsim);

		spin_lock_irqsave(&emvsim->lock, flags);
		if (emvsim->is_fixed_len_rec &&
		    emvsim->rcv_count < emvsim->expected_rcv_cnt)
			emvsim_change_rcv_threshold(emvsim);
		spin_unlock_irqrestore(&emvsim->lock, flags);
		emvsim->timeout = RX_TIMEOUT * HZ;
		timeout = wait_for_completion_interruptible_timeout(
				&emvsim->xfer_done, emvsim->timeout);
		if (timeout == 0) {
			dev_err(emvsim_dev.parent, "Receiving timeout\n");
			emvsim_set_cwt(emvsim, 0);
			emvsim_set_bwt(emvsim, 0);
			emvsim_rx_irq_disable(emvsim);
			errval = -SIM_E_TIMEOUT;
			break;
		}
copy_data:
		if (emvsim->is_fixed_len_rec)
			copy_cnt = emvsim->rcv_count > emvsim->expected_rcv_cnt
				 ? emvsim->expected_rcv_cnt
				 : emvsim->rcv_count;
		else
			copy_cnt = emvsim->rcv_count;

		ret = copy_to_user(&(((sim_rcv_t *)arg)->rcv_length),
				   &copy_cnt, sizeof(copy_cnt));
		if (ret) {
			dev_err(emvsim_dev.parent, "ATR ACCESS Error\n");
			errval = -SIM_E_ACCESS;
			break;
		}

		__get_user(rcv_buffer, &((sim_rcv_t *)arg)->rcv_buffer);
		ret = copy_to_user(rcv_buffer,
				   &emvsim->rcv_buffer[emvsim->rcv_head],
				   copy_cnt);
		if (ret) {
			dev_err(emvsim_dev.parent, "ATR ACCESS Error\n");
			errval = -SIM_E_ACCESS;
			break;
		}

		ret = copy_to_user(&(((sim_rcv_t *)arg)->errval),
				   &emvsim->errval, sizeof(emvsim->errval));
		if (ret) {
			dev_err(emvsim_dev.parent, "ATR ACCESS Error\n");
			errval = -SIM_E_ACCESS;
			break;
		}
		/*Reset the receiving count and errval*/
		spin_lock_irqsave(&emvsim->lock, flags);
		emvsim->rcv_head += copy_cnt;
		emvsim->rcv_count -= copy_cnt;
		emvsim->errval = 0;
		spin_unlock_irqrestore(&emvsim->lock, flags);

		break;

	case SIM_IOCTL_SET_PROTOCOL:
		ret = copy_from_user(&emvsim->protocol_type, (int *)arg,
				     sizeof(int));
		if (ret)
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_SET_TIMING:
		ret = copy_from_user(&emvsim->timing_data, (sim_timing_t *)arg,
				     sizeof(sim_timing_t));
		if (ret) {
			dev_err(emvsim_dev.parent, "Copy Error\n");
			errval = ret;
			break;
		}

		ret = emvsim_check_timing_data(&emvsim->timing_data);
		if (ret)
			errval = ret;

		break;

	case SIM_IOCTL_SET_BAUD:
		ret = copy_from_user(&emvsim->baud_rate, (sim_baud_t *)arg,
				     sizeof(sim_baud_t));
		if (ret) {
			dev_err(emvsim_dev.parent, "Copy Error\n");
			errval = ret;
			break;
		}

		emvsim_check_baud_rate(&emvsim->baud_rate);

		break;
	case SIM_IOCTL_WAIT:
		ret = copy_from_user(&delay, (unsigned int *)arg,
				     sizeof(unsigned int));
		if (ret) {
			dev_err(emvsim_dev.parent, "\nWait Copy Error\n");
			errval = ret;
			break;
		}

		emvsim_polling_delay(emvsim, delay);
		break;

	case SIM_IOCTL_GET_PRESENSE:
		if (put_user(emvsim->present, (int *)arg))
			errval = -SIM_E_ACCESS;
		break;

	case SIM_IOCTL_CARD_LOCK:
		errval = emvsim_card_lock(emvsim);
		break;

	case SIM_IOCTL_CARD_EJECT:
		errval = emvsim_card_eject(emvsim);
		break;
	};

	return errval;
};

static int emvsim_open(struct inode *inode, struct file *file)
{
	int errval = SIM_OK;
	struct emvsim_t *emvsim = dev_get_drvdata(emvsim_dev.parent);

	file->private_data = emvsim;
	spin_lock_init(&emvsim->lock);

	if (!emvsim->ioaddr) {
		errval = -ENOMEM;
		return errval;
	}

	if (!emvsim->open_cnt) {
		clk_prepare_enable(emvsim->ipg);
		clk_prepare_enable(emvsim->clk);
	}

	emvsim->open_cnt = 1;
	init_completion(&emvsim->xfer_done);
	errval = emvsim_reset_module(emvsim);
	emvsim_data_reset(emvsim);

	return errval;
};

static int emvsim_release(struct inode *inode, struct file *file)
{
	u32 reg_data;
	struct emvsim_t *emvsim = (struct emvsim_t *)file->private_data;

	/* disable presense detection interrupt */
	reg_data = __raw_readl(emvsim->ioaddr + EMV_SIM_PCSR);
	__raw_writel(reg_data | SPDIM, emvsim->ioaddr + EMV_SIM_PCSR);

	if (emvsim->present != SIM_PRESENT_REMOVED)
		emvsim_deactivate(emvsim);

	if (emvsim->open_cnt) {
		clk_disable_unprepare(emvsim->clk);
		clk_disable_unprepare(emvsim->ipg);
	}

	emvsim->open_cnt = 0;

	return 0;
};

static const struct file_operations emvsim_fops = {
	.owner = THIS_MODULE,
	.open = emvsim_open,
	.release = emvsim_release,
	.unlocked_ioctl = emvsim_ioctl,
};

static struct miscdevice emvsim_dev = {
	MISC_DYNAMIC_MINOR,
	"mxc_sim",
	&emvsim_fops
};

static const struct of_device_id emvsim_imx_dt_ids[] = {
	{ .compatible = "fsl,imx8-emvsim" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, emvsim_imx_dt_ids);

static int emvsim_probe(struct platform_device *pdev)
{
	int ret = 0;
	const struct of_device_id *of_id;
	struct emvsim_t *emvsim = NULL;

	emvsim = devm_kzalloc(&pdev->dev, sizeof(struct emvsim_t),
			      GFP_KERNEL);
	if (!emvsim)
		return -ENOMEM;

	of_id = of_match_device(emvsim_imx_dt_ids, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;
	else
		return -EINVAL;

	emvsim->clk_rate = FCLK_FREQ;
	emvsim->open_cnt = 0;

	emvsim->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!emvsim->res) {
		dev_err(emvsim_dev.parent, "Can't get the MEMORY\n");
		return -ENOMEM;
	}
	emvsim->ioaddr = devm_ioremap_resource(&pdev->dev, emvsim->res);
	if (IS_ERR(emvsim->ioaddr)) {
		dev_err(&pdev->dev,
			"failed to get ioremap base\n");
		ret = PTR_ERR(emvsim->ioaddr);
		return ret;
	}

	/* request the emvsim per clk and ipg clk */
	emvsim->clk = devm_clk_get(&pdev->dev, "sim");
	if (IS_ERR(emvsim->clk)) {
		ret = PTR_ERR(emvsim->clk);
		dev_err(emvsim_dev.parent, "Get PER CLK ERROR !\n");
		return ret;
	}

	emvsim->ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(emvsim->ipg)) {
		ret = PTR_ERR(emvsim->ipg);
		dev_err(emvsim_dev.parent, "Get IPG CLK ERROR !\n");
		return ret;
	}

	emvsim->irq = platform_get_irq(pdev, 0);
	if (emvsim->irq < 0) {
		dev_err(&pdev->dev, "No irq line provided\n");
		return -ENOENT;
	}

	if (devm_request_irq(&pdev->dev, emvsim->irq, emvsim_irq_handler,
			     0, "mxc_emvsim_irq", emvsim)) {
		dev_err(&pdev->dev, "can't claim irq %d\n", emvsim->irq);
		return -ENOENT;
	}

	platform_set_drvdata(pdev, emvsim);
	emvsim_dev.parent = &pdev->dev;

	ret = misc_register(&emvsim_dev);
	dev_info(&pdev->dev, "emvsim register %s\n", ret ? "fail" : "success");

	return ret;
}

static int emvsim_remove(struct platform_device *pdev)
{
	struct emvsim_t *emvsim = platform_get_drvdata(pdev);

	if (emvsim->open_cnt) {
		clk_disable_unprepare(emvsim->clk);
		clk_disable_unprepare(emvsim->ipg);
	}

	misc_deregister(&emvsim_dev);

	return 0;
}

#ifdef CONFIG_PM
static int emvsim_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct emvsim_t *emvsim = platform_get_drvdata(pdev);

	if (emvsim->open_cnt) {
		clk_disable_unprepare(emvsim->clk);
		clk_disable_unprepare(emvsim->ipg);
	}

	pinctrl_pm_select_sleep_state(&pdev->dev);

	return 0;
}

static int emvsim_resume(struct platform_device *pdev)
{
	struct emvsim_t *emvsim = platform_get_drvdata(pdev);

	if (!emvsim->open_cnt) {
		clk_prepare_enable(emvsim->ipg);
		clk_prepare_enable(emvsim->clk);
	}

	pinctrl_pm_select_default_state(&pdev->dev);

	return 0;
}

#else
#define emvsim_suspend NULL
#define emvsim_resume NULL
#endif

static struct platform_driver emvsim_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = emvsim_imx_dt_ids,
	},
	.probe = emvsim_probe,
	.remove = emvsim_remove,
	.suspend = emvsim_suspend,
	.resume = emvsim_resume,
};

static int __init emvsim_drv_init(void)
{
	return platform_driver_register(&emvsim_driver);
}

static void __exit emvsim_drv_exit(void)
{
	platform_driver_unregister(&emvsim_driver);
}

module_init(emvsim_drv_init);
module_exit(emvsim_drv_exit);

MODULE_AUTHOR("Gao Pan <pandy.gao@nxp.com>");
MODULE_DESCRIPTION("NXP EMVSIM Driver");
MODULE_LICENSE("GPL");
