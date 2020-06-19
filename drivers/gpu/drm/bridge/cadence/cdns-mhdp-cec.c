/*
 * Copyright 2019 NXP
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
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <drm/bridge/cdns-mhdp.h>

#define CEC_NAME	"cdns-mhdp-cec"

#define REG_ADDR_OFF 4

/* regsiter define */
#define TX_MSG_HEADER 0x33800
#define TX_MSG_LENGTH 0x33840
#define TX_MSG_CMD 0x33844
#define RX_MSG_CMD 0x33850
#define RX_CLEAR_BUF 0x33854
#define LOGICAL_ADDRESS_LA0 0x33858

#define CLK_DIV_MSB 0x3386c
#define CLK_DIV_LSB 0x33870
#define RX_MSG_DATA1 0x33900
#define RX_MSG_LENGTH 0x33940
#define RX_MSG_STATUS 0x33944
#define NUM_OF_MSG_RX_BUF 0x33948
#define TX_MSG_STATUS 0x3394c
#define DB_L_TIMER 0x33980

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
/**
 * Maximum number of Messages in the RX Buffers.
 */
#define CEC_MAX_RX_MSGS 2

static u32 mhdp_cec_read(struct cdns_mhdp_cec *cec, u32 offset)
{
	struct cdns_mhdp_device *mhdp =
			container_of(cec, struct cdns_mhdp_device, hdmi.cec);
	return cdns_mhdp_bus_read(mhdp, offset);
}

static void mhdp_cec_write(struct cdns_mhdp_cec *cec, u32 offset, u32 val)
{
	struct cdns_mhdp_device *mhdp =
			container_of(cec, struct cdns_mhdp_device, hdmi.cec);
	cdns_mhdp_bus_write(val, mhdp, offset);
}

static void mhdp_cec_clear_rx_buffer(struct cdns_mhdp_cec *cec)
{
	mhdp_cec_write(cec, RX_CLEAR_BUF, 1);
	mhdp_cec_write(cec, RX_CLEAR_BUF, 0);
}

static void mhdp_cec_set_divider(struct cdns_mhdp_cec *cec)
{
	struct cdns_mhdp_device *mhdp =
			container_of(cec, struct cdns_mhdp_device, hdmi.cec);
	u32 clk_div;

	/* Set clock divider */
	clk_div = cdns_mhdp_get_fw_clk(mhdp) * 10;

	mhdp_cec_write(cec, CLK_DIV_MSB,
			  (clk_div >> 8) & 0xFF);
	mhdp_cec_write(cec, CLK_DIV_LSB, clk_div & 0xFF);
}

static u32 mhdp_cec_read_message(struct cdns_mhdp_cec *cec)
{
	struct cec_msg *msg = &cec->msg;
	int len;
	int i;

	mhdp_cec_write(cec, RX_MSG_CMD, CEC_RX_READ);

	len = mhdp_cec_read(cec, RX_MSG_LENGTH);
	msg->len = len + 1;
	dev_dbg(cec->dev, "RX MSG len =%d\n", len);

	/* Read RX MSG bytes */
	for (i = 0; i < msg->len; ++i) {
		msg->msg[i] = (u8) mhdp_cec_read(cec, RX_MSG_DATA1 + (i * REG_ADDR_OFF));
		dev_dbg(cec->dev, "RX MSG[%d]=0x%x\n", i, msg->msg[i]);
	}

	mhdp_cec_write(cec, RX_MSG_CMD, CEC_RX_STOP);

	return true;
}

static u32 mhdp_cec_write_message(struct cdns_mhdp_cec *cec, struct cec_msg *msg)
{
	u8 i;

	mhdp_cec_write(cec, TX_MSG_CMD, CEC_TX_STOP);

	if (msg->len > CEC_MAX_MSG_SIZE) {
		dev_err(cec->dev, "Invalid MSG size!\n");
		return -EINVAL;
	}

	for (i = 0; i < msg->len; ++i)
		printk("msg[%d]=0x%x\n",i, msg->msg[i]);

	/* Write Message to register */
	for (i = 0; i < msg->len; ++i) {
		mhdp_cec_write(cec, TX_MSG_HEADER + (i * REG_ADDR_OFF),
			  msg->msg[i]);
	}
	/* Write Message Length (payload + opcode) */
	mhdp_cec_write(cec, TX_MSG_LENGTH, msg->len - 1);

	mhdp_cec_write(cec, TX_MSG_CMD, CEC_TX_TRANSMIT);

	return true;
}

static int mhdp_cec_set_logical_addr(struct cdns_mhdp_cec *cec, u32 la)
{
	u8 la_reg;
	u8 i;

	if (la == CEC_LOG_ADDR_INVALID)
		/* invalid all LA address */
		for (i = 0; i < CEC_MAX_LOG_ADDRS; i++) {
			mhdp_cec_write(cec, LOGICAL_ADDRESS_LA0 + (i * REG_ADDR_OFF), 0);
			return 0;
		}

	/* In fact cdns mhdp cec could support max 5 La address */
	for (i = 0; i < CEC_MAX_LOG_ADDRS; i++) {
		la_reg = mhdp_cec_read(cec, LOGICAL_ADDRESS_LA0 + (i * REG_ADDR_OFF));
		/* Check LA already used */
		if (la_reg & 0x10)
			continue;

		if ((la_reg & 0xF) == la) {
			dev_warn(cec->dev, "Warning. LA already in use.\n");
			return 0;
		}

		la = (la & 0xF) | (1 << 4);

		mhdp_cec_write(cec, LOGICAL_ADDRESS_LA0 + (i * REG_ADDR_OFF), la);
		return 0;
	}

	dev_warn(cec->dev, "All LA in use\n");

	return -ENXIO;
}

static int mhdp_cec_poll_worker(void *_cec)
{
	struct cdns_mhdp_cec *cec = (struct cdns_mhdp_cec *)_cec;
	int num_rx_msgs, i;
	int sts;

	set_freezable();

	for (;;) {
		if (kthread_freezable_should_stop(NULL))
			break;

		/* Check TX State */
		sts = mhdp_cec_read(cec, TX_MSG_STATUS);
		switch (sts) {
		case CEC_STS_SUCCESS:
			cec_transmit_done(cec->adap, CEC_TX_STATUS_OK, 0, 0, 0,
					  0);
			mhdp_cec_write(cec, TX_MSG_CMD, CEC_TX_STOP);
			break;
		case CEC_STS_ERROR:
			mhdp_cec_write(cec, TX_MSG_CMD, CEC_TX_STOP);
			cec_transmit_done(cec->adap,
					  CEC_TX_STATUS_MAX_RETRIES |
					  CEC_TX_STATUS_NACK, 0, 1, 0, 0);
			break;
		case CEC_STS_BUSY:
		default:
			break;
		}

		/* Check RX State */
		sts = mhdp_cec_read(cec, RX_MSG_STATUS);
		num_rx_msgs = mhdp_cec_read(cec, NUM_OF_MSG_RX_BUF);
		switch (sts) {
		case CEC_STS_SUCCESS:
			if (num_rx_msgs == 0xf)
				num_rx_msgs = CEC_MAX_RX_MSGS;

			if (num_rx_msgs > CEC_MAX_RX_MSGS) {
				dev_err(cec->dev, "Error rx msg num %d\n",
					num_rx_msgs);
				mhdp_cec_clear_rx_buffer(cec);
				break;
			}

			/* Rx FIFO Depth 2 RX MSG */
			for (i = 0; i < num_rx_msgs; i++) {
				mhdp_cec_read_message(cec);
				cec->msg.rx_status = CEC_RX_STATUS_OK;
				cec_received_msg(cec->adap, &cec->msg);
			}
			break;
		default:
			break;
		}

		if (!kthread_should_stop())
			schedule_timeout_idle(20);
	}

	return 0;
}

static int mhdp_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct cdns_mhdp_cec *cec = cec_get_drvdata(adap);

	if (enable) {
		mhdp_cec_write(cec, DB_L_TIMER, 0x10);
		mhdp_cec_set_divider(cec);
	} else
		mhdp_cec_set_divider(cec);

	return 0;
}

static int mhdp_cec_adap_log_addr(struct cec_adapter *adap, u8 addr)
{
	struct cdns_mhdp_cec *cec = cec_get_drvdata(adap);

	return mhdp_cec_set_logical_addr(cec, addr);
}

static int mhdp_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct cdns_mhdp_cec *cec = cec_get_drvdata(adap);

	mhdp_cec_write_message(cec, msg);

	return 0;
}

static const struct cec_adap_ops cdns_mhdp_cec_adap_ops = {
	.adap_enable = mhdp_cec_adap_enable,
	.adap_log_addr = mhdp_cec_adap_log_addr,
	.adap_transmit = mhdp_cec_adap_transmit,
};

int cdns_mhdp_register_cec_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct cdns_mhdp_cec *cec = &mhdp->hdmi.cec;
	int ret;

	cec->adap = cec_allocate_adapter(&cdns_mhdp_cec_adap_ops, cec,
					 CEC_NAME,
					 CEC_CAP_PHYS_ADDR | CEC_CAP_LOG_ADDRS |
					 CEC_CAP_TRANSMIT | CEC_CAP_PASSTHROUGH
					 | CEC_CAP_RC, CEC_MAX_LOG_ADDRS);
	ret = PTR_ERR_OR_ZERO(cec->adap);
	if (ret)
		return ret;
	ret = cec_register_adapter(cec->adap, dev);
	if (ret) {
		cec_delete_adapter(cec->adap);
		return ret;
	}

	cec->dev = dev;

	cec->cec_worker = kthread_create(mhdp_cec_poll_worker, cec, "cdns-mhdp-cec");
	if (IS_ERR(cec->cec_worker))
		dev_err(cec->dev, "failed  create hdp cec thread\n");

	wake_up_process(cec->cec_worker);

	dev_dbg(dev, "CEC successfuly probed\n");
	return 0;
}

int cdns_mhdp_unregister_cec_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct cdns_mhdp_cec *cec = &mhdp->hdmi.cec;

	if (cec->cec_worker) {
		kthread_stop(cec->cec_worker);
		cec->cec_worker = NULL;
	}
	cec_unregister_adapter(cec->adap);
	return 0;
}

MODULE_AUTHOR("Sandor.Yu@NXP.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NXP CDNS MHDP HDMI CEC driver");
