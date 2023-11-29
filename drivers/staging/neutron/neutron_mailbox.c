// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 *
 */

#include <linux/io.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "neutron_device.h"
#include "neutron_mailbox.h"

#define DRIVER_NAME	"imx-neutron-mailbox"

#define INFERENCE_DONE_IRQ_ENABLE	BIT(1)
#define MBOX_IRQ_ENABLE			BIT(2)
#define SHUTDOWN_IRQ_ENABLE		BIT(7)

#define MAX_SEND_MSG_ARGC		4
#define MAX_RECV_MSG_ARGC		2
#define SEND_MSG_ARG(n)			(MBOX3 + (((n) + 1) << 2))
#define RECV_MSG_ARG(n)			(MBOX0 + (((n) + 1) << 2))

/*
 * Generate doorbell interrupt to firmware.
 *
 */
static int mbox_doorbell(struct neutron_mbox *mbox)
{
	u32 appctrl;

	/* Bit5: RESDONE,  BIT2: SYS_SHUTDOWN */
	appctrl = readl(mbox->base + APPCTRL);
	appctrl |= 0x4;
	writel(appctrl, mbox->base + APPCTRL);

	return 0;
}

/**
 * mbox_read_ret - Read mailbox return code
 *
 * @mbox: neutron_mbox pointer
 *
 * Return: Code value.
 */
static unsigned int mbox_read_ret(struct neutron_mbox *mbox)
{
	return readl(mbox->base + MBOX0);
}

/**
 * mbox_tx_done - See if the last tx message is sent
 *
 * ACK will be set after message is received by firmware
 *
 * @mbox: neutron_mbox pointer
 *
 * Return: 'true' is no pending tx data, 'false' if there are any.
 */
static bool mbox_tx_done(struct neutron_mbox *mbox)
{
	u32 ack;

	ack = readl(mbox->base + MBOX0);

	/* Always true current if ack is not 0 */
	if (ack)
		return true;
	else
		return false;
}

/**
 * mbox_send_data - Send mailbox data.
 *
 * @mbox: neutron_mbox pointer
 * @data: data to be sent
 *
 * Return: 0 is success, else fail.
 */
static int mbox_send_data(struct neutron_mbox *mbox, void *data)
{
	struct neutron_mbox_tx_msg *msg = data;
	int i;

	if (!data)
		return -EINVAL;

	if (msg->argc > MAX_SEND_MSG_ARGC) {
		dev_warn(mbox->ndev->dev, "Using maximum %d\n", MAX_SEND_MSG_ARGC);
		msg->argc = MAX_SEND_MSG_ARGC;
	}

	for (i = 0; i < msg->argc; i++)
		writel(msg->args[i], mbox->base + SEND_MSG_ARG(i));

	writel(msg->command, mbox->base + MBOX3);

	neu_dbg("command 0x%x is sent\n", msg->command);

	/* Tell firmware that msg is sent */
	mbox_doorbell(mbox);

	/* Wait until ACK is received */
	udelay(2);
	for (i = 0; i < 20; i++) {
		/* return success if tx is done */
		if (mbox_tx_done(mbox))
			return 0;
		usleep_range(1, 10);
	}

	/* Timeout */
	dev_err(mbox->ndev->dev, "send timeout: 0x%x\n", msg->command);
	return -ETIME;
}

/**
 * mbox_send_reset - Send command to reset neutron state
 *
 * Return: 0 is successful, else fail.
 */

static int mbox_send_reset(struct neutron_mbox *mbox)
{
	u32 i, val;

	writel(RESET_VAL, mbox->base + MBOX4);
	writel(RESET_VAL, mbox->base + MBOX5);
	writel(RESET, mbox->base + MBOX3);

	usleep_range(2, 5);
	/* Wait for neutron to get into reset */
	for (i = 0; i < 50; i++) {
		val = readl(mbox->base + MBOX0);
		if (val != RESET_VAL)
			usleep_range(2, 10);
		else
			return 0;
	}

	return val;
}

/**
 * mbox_recv_data - Recevie mailbox data and store into data.
 *
 * @mbox: neutron_mbox pointer
 * @data: pointer to store received data
 *
 * Return: 0 is success, else fail.
 */
static int mbox_recv_data(struct neutron_mbox *mbox, void *data)
{
	struct neutron_mbox_rx_msg *rx_msg;
	int i;

	if (IS_ERR(data))
		return -1;

	rx_msg = data;

	rx_msg->retcode = readl(mbox->base + MBOX0);
	dev_dbg(mbox->ndev->dev,
		"mbox read 0x%x: 0x%x\n", MBOX0, rx_msg->retcode);

	for (i = 0; i < MAX_RECV_MSG_ARGC; i++)
		rx_msg->args[i] = readl(mbox->base + RECV_MSG_ARG(i));

	return 0;
}

static const struct neutron_mbox_ops neutron_mbox_ops = {
	.send_data	= mbox_send_data,
	.send_reset	= mbox_send_reset,
	.recv_data	= mbox_recv_data,
	.read_ret	= mbox_read_ret,
	.tx_done	= mbox_tx_done,
};

static void mbox_recv_callback(struct neutron_mbox *mbox)
{
	struct neutron_mbox_rx_msg rx_msg;

	mbox_recv_data(mbox, &rx_msg);

	if (mbox->callback)
		mbox->callback(mbox->ndev, &rx_msg);
}

static irqreturn_t mbox_irq_handler(int irq, void *data)
{
	struct neutron_mbox *mbox = data;
	u32 val, appstatus;

	val = readl(mbox->base + INTENA);
	appstatus = readl(mbox->base + APPSTATUS);

	neu_dbg("irq: INTENA: 0x%x NTCLR: 0x%x\n",
		readl(mbox->base + INTENA),
		readl(mbox->base + INTCLR));

	if (val & (MBOX_IRQ_ENABLE | INFERENCE_DONE_IRQ_ENABLE))
		mbox_recv_callback(mbox);

	/* Clear irq */
	writel(val, mbox->base + INTCLR);
	/* Enable irq again */
	writel(val, mbox->base + INTENA);

	neu_dbg("irq: IRQ_HANDLED\n");
	return IRQ_HANDLED;
}

static int mbox_request_irq(struct neutron_mbox *mbox,
			    irq_handler_t handler)
{
	u32 reg;
	int ret;

	ret = devm_request_irq(mbox->ndev->dev, mbox->irq, handler, 0,
			       DRIVER_NAME, mbox);
	if (ret < 0) {
		dev_err(mbox->ndev->dev, "Cannot request irq\n");
		return ret;
	}

	/* Enable irq, SHUTDOWN_IRQ works currently */
	reg = readl(mbox->base + INTENA);
	reg |= SHUTDOWN_IRQ_ENABLE;
	writel(reg, mbox->base + INTENA);

	return 0;
}

static void mbox_free_irq(struct neutron_mbox *mbox)
{
	u32 reg;

	/* disable irq  */
	reg = readl(mbox->base + INTENA);
	reg &= ~SHUTDOWN_IRQ_ENABLE;
	writel(reg, mbox->base + INTENA);

	devm_free_irq(mbox->ndev->dev, mbox->irq, mbox);
}

struct neutron_mbox *neutron_mbox_create(struct neutron_device *ndev, int irq,
					 mbox_rx_callback callback)
{
	struct neutron_mbox *mbox;
	int ret;

	if ((!ndev) || (irq < 0))
		return NULL;

	mbox = devm_kzalloc(ndev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return NULL;

	mbox->ndev = ndev;
	mbox->base = ndev->reg_base;
	mbox->irq = irq;

	mbox->callback = callback;
	mbox->ops = &neutron_mbox_ops;

	ret = mbox_request_irq(mbox, mbox_irq_handler);
	if (ret < 0) {
		dev_err(ndev->dev, "mailbox: Cannot request irq\n");
		goto free_mbox;
	}

	return mbox;

free_mbox:
	devm_kfree(ndev->dev, mbox);

	return NULL;
}

void neutron_mbox_destroy(struct neutron_mbox *mbox)
{
	/* Free irq */
	mbox_free_irq(mbox);

	mbox->callback = NULL;
	mbox->ops = NULL;

	/* Free neutron_mbox device */
	devm_kfree(mbox->ndev->dev, mbox);
}
