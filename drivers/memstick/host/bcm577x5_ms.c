// SPDX-License-Identifier: GPL-2.0-only
#include <linux/align.h>
#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/bug.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/memstick.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pm.h>
#include <linux/scatterlist.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "bcm577x5_ms.h"

#define DRV_NAME "bcm577x5_ms"

static const struct pci_device_id bcm577x5_pci_id_tbl[] = {
	{
		PCI_VDEVICE(BROADCOM, 0x16be),
	},
	{},
};
MODULE_DEVICE_TABLE(pci, bcm577x5_pci_id_tbl);

static inline u32 bcm577x5_reg_readl(struct bcm577x5_device *dev, int address)
{
	return readl(dev->mmio + address);
}

static inline u16 bcm577x5_reg_readw(struct bcm577x5_device *dev, int address)
{
	return readw(dev->mmio + address);
}

static inline u8 bcm577x5_reg_readb(struct bcm577x5_device *dev, int address)
{
	return readb(dev->mmio + address);
}

static inline void bcm577x5_reg_writel(struct bcm577x5_device *dev, int address,
				       u32 value)
{
	writel(value, dev->mmio + address);
}

static inline void bcm577x5_reg_writew(struct bcm577x5_device *dev, int address,
				       u16 value)
{
	writew(value, dev->mmio + address);
}

static inline void bcm577x5_reg_writeb(struct bcm577x5_device *dev, int address,
				       u8 value)
{
	writeb(value, dev->mmio + address);
}

static int bcm577x5_reg_waitb(struct bcm577x5_device *dev, int address, u8 mask,
			      u8 value, unsigned int timeout_ms)
{
	u8 reg;

	return readb_poll_timeout(dev->mmio + address, reg,
				  (reg & mask) == value,
				  BCM577x5_MS_POLL_DELAY_US,
				  timeout_ms * USEC_PER_MSEC);
}

static int bcm577x5_ms_reset(struct bcm577x5_device *dev, u8 reset)
{
	bcm577x5_reg_writeb(dev, BCM577x5_MS_SOFTWARE_RESET, reset);

	return bcm577x5_reg_waitb(dev, BCM577x5_MS_SOFTWARE_RESET, reset, 0,
				  1000);
}

static int bcm577x5_ms_power(struct bcm577x5_device *dev, int power)
{
	int err;

	switch (power) {
	case MEMSTICK_POWER_ON:
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL,
				    BCM577x5_MS_POWER_330);
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL,
				    BCM577x5_MS_POWER_330 |
					    BCM577x5_MS_POWER_ON);
		err = bcm577x5_reg_waitb(dev, BCM577x5_MS_POWER_CONTROL,
					 BCM577x5_MS_POWER_ON,
					 BCM577x5_MS_POWER_ON, 1000);
		if (err)
			return err;

		msleep(20);

		bcm577x5_reg_writel(dev, BCM577x5_MS_STATUS_MUX_CONTROL,
				    bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS_MUX_CONTROL) |
					    BCM577x5_MS_CLOCK_SOURCE_62_5MHZ);
		return 0;
	case MEMSTICK_POWER_OFF:
		bcm577x5_reg_writeb(dev, BCM577x5_MS_POWER_CONTROL, 0);
		bcm577x5_reg_writel(dev, BCM577x5_MS_STATUS_MUX_CONTROL,
				    bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS_MUX_CONTROL) &
					    ~BCM577x5_MS_CLOCK_SOURCE_62_5MHZ);
		return 0;
	default:
		return -EINVAL;
	}
}

static int bcm577x5_ms_clock(struct bcm577x5_device *dev, int clock_hz)
{
	int err;
	u16 divisor, divisor_lo, divisor_hi, clock_reg;

	divisor = DIV_ROUND_UP(dev->base_clock, 2 * clock_hz);
	divisor_lo = divisor & BCM577x5_MS_DIV_MASK;
	divisor_hi = (divisor & BCM577x5_MS_DIV_HI_MASK) >> 8;
	clock_reg = (divisor_lo << BCM577x5_MS_DIVIDER_SHIFT) |
		    (divisor_hi << BCM577x5_MS_DIVIDER_HI_SHIFT);

	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL, clock_reg);
	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL,
			    clock_reg | BCM577x5_MS_CLOCK_INT_EN);
	err = bcm577x5_reg_waitb(dev, BCM577x5_MS_CLOCK_CONTROL,
				 BCM577x5_MS_CLOCK_INT_STABLE,
				 BCM577x5_MS_CLOCK_INT_STABLE, 1000);
	if (err)
		return err;
	bcm577x5_reg_writew(dev, BCM577x5_MS_CLOCK_CONTROL,
			    clock_reg | BCM577x5_MS_CLOCK_INT_EN |
				    BCM577x5_MS_CLOCK_CARD_EN);
	return 0;
}

static int bcm577x5_ms_mode(struct bcm577x5_device *dev, int mode)
{
	u8 host_control = bcm577x5_reg_readb(dev, BCM577x5_MS_HOST_CONTROL);
	u32 bridge_mode = bcm577x5_reg_readl(dev, BCM577x5_MS_BRIDGE_MODE);

	switch (mode) {
	case MEMSTICK_SERIAL:
		bcm577x5_reg_writel(dev, BCM577x5_MS_BRIDGE_MODE,
				    bridge_mode & ~BCM577x5_MS_BRIDGE_MODE_PARALLEL);
		bcm577x5_reg_writeb(dev, BCM577x5_MS_HOST_CONTROL,
				    host_control & ~BCM577x5_MS_CTRL_4BITBUS);
		break;
	case MEMSTICK_PAR4:
		bcm577x5_reg_writel(dev, BCM577x5_MS_BRIDGE_MODE,
				    bridge_mode | BCM577x5_MS_BRIDGE_MODE_PARALLEL);
		bcm577x5_reg_writeb(dev, BCM577x5_MS_HOST_CONTROL,
				    host_control | BCM577x5_MS_CTRL_4BITBUS);
		break;
	default:
		return -EINVAL;
	}

	dev->mode = mode;
	return 0;
}

static int bcm577x5_ms_init(struct bcm577x5_device *dev)
{
	int error;

	dev->prev_tpc = 0;

	/* Reset the controller */
	error = bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_ALL);
	if (error)
		return error;

	/* Power cycle the controller */
	error = bcm577x5_ms_power(dev, MEMSTICK_POWER_OFF);
	if (error)
		return error;
	error = bcm577x5_ms_power(dev, MEMSTICK_POWER_ON);
	if (error)
		return error;

	/* Set clock divider to 2 */
	error = bcm577x5_ms_clock(dev, 20000000);
	if (error)
		return error;

	/* Set timeout */
	bcm577x5_reg_writeb(dev, BCM577x5_MS_TIMEOUT_CONTROL, 0xE);

	/* Set mode */
	error = bcm577x5_ms_mode(dev, dev->mode);
	if (error)
		return error;

	error = bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_CMD);
	if (error)
		return error;
	error = bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA);
	if (error)
		return error;

	/* Re-enable interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE,
			    BCM577x5_MS_INT_DEFAULT);

	/* Avoid retriggering a card insert interrupt */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_STATUS,
			    BCM577x5_MS_INT_CARD_INSERT);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE,
			    BCM577x5_MS_INT_DEFAULT);

	return 0;
}

static irqreturn_t bcm577x5_irq(int irq, void *data)
{
	struct bcm577x5_device *dev = data;
	irqreturn_t ret = IRQ_NONE;
	u32 val, status;
	int i;

	val = bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS);
	if (val & BCM577x5_MS_STATUS_INT_ASSERTED) {
		status = bcm577x5_reg_readl(dev, BCM577x5_MS_INT_STATUS);
		atomic_or(status, &dev->int_seen);

		if (status & 0x100) {
			bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE,
					    bcm577x5_reg_readl(dev,
							       BCM577x5_MS_INT_ENABLE) &
					~0x100);
		}

		/* Each individual bit needs to be acknowledged separately */
		for (i = 0; i < 32; i++) {
			if (status & BIT(i)) {
				bcm577x5_reg_writel(dev, BCM577x5_MS_INT_STATUS,
						    BIT(i));
			}
		}

		/* Signal the command or TPC is done */
		if (status & BCM577x5_MS_INT_RESPONSE)
			complete(&dev->cmd_done);

		/* Signal the FIFO is ready */
		if (status & (BCM577x5_MS_INT_DATA_AVAIL |
			      BCM577x5_MS_INT_SPACE_AVAIL))
			complete(&dev->fifo_done);

		if (status & BCM577x5_MS_INT_DATA_TIMEOUT)
			dev_warn(&dev->pci_dev->dev, "Timeout!\n");

		/* Signal data block transferred */
		if (status & BCM577x5_MS_INT_DMA_END)
			complete(&dev->dma_done);

		/* Signal end of the read/write phase */
		if (status & BCM577x5_MS_INT_DATA_END)
			complete(&dev->data_done);

		/* Immediately stop all ongoing requests */
		if (status & BCM577x5_MS_INT_CARD_REMOVE) {
			complete(&dev->cmd_done);
			complete(&dev->fifo_done);
			complete(&dev->dma_done);
			complete(&dev->data_done);
		}

		if (status & (BCM577x5_MS_INT_CARD_INSERT |
			      BCM577x5_MS_INT_CARD_REMOVE)) {
			if (status & BCM577x5_MS_INT_CARD_INSERT)
				dev_info(&dev->pci_dev->dev, "Card inserted\n");
			else
				dev_info(&dev->pci_dev->dev, "Card removed\n");
			memstick_detect_change(dev->host);
		}

		ret = IRQ_HANDLED;
	}
	return ret;
}

/* External interface: set settings */
static int bcm577x5_ms_set_param(struct memstick_host *host,
				 enum memstick_param param, int value)
{
	struct bcm577x5_device *dev = memstick_priv(host);

	switch (param) {
	case MEMSTICK_POWER:
		return bcm577x5_ms_power(dev, value);
	case MEMSTICK_INTERFACE:
		return bcm577x5_ms_mode(dev, value);
	default:
		return -EINVAL;
	}
}

static int bcm577x5_ms_prepare_dma(struct bcm577x5_device *dev)
{
	int len = dev->req->sg.length;
	void *dma_copy_buffer;
	dma_addr_t dma_address;

	if (len > 4096)
		return -EOPNOTSUPP;

	reinit_completion(&dev->dma_done);

	if (dev->req->data_dir == WRITE) {
		dma_copy_buffer = dev->dma_buffer + 4096 - len;
		sg_copy_to_buffer(&dev->req->sg,
				  sg_nents_for_len(&dev->req->sg, len),
				  dma_copy_buffer, len);
	}
	/*
	 * The last byte should cause an overflow interrupt on the 4096 byte
	 * boundary
	 */
	dma_address = dev->dma_handle + 4096 - len;

	bcm577x5_reg_writel(dev, BCM577x5_MS_DMA_ADDRESS, dma_address);
	return 0;
}

static int bcm577x5_ms_finish_dma(struct bcm577x5_device *dev)
{
	int len = dev->req->sg.length;
	void *dma_copy_buffer;

	if (!wait_for_completion_timeout(&dev->dma_done,
					 msecs_to_jiffies(BCM577x5_MS_REQ_TIMEOUT_MS)))
		return -ETIMEDOUT;

	if (dev->req->data_dir == READ) {
		dma_copy_buffer = dev->dma_buffer + 4096 - len;
		sg_copy_from_buffer(&dev->req->sg,
				    sg_nents_for_len(&dev->req->sg, len),
				    dma_copy_buffer, len);
	}

	return 0;
}

static int bcm577x5_ms_prepare_fifo(struct bcm577x5_device *dev)
{
	if (dev->prev_tpc != MS_TPC_READ_LONG_DATA &&
	    dev->prev_tpc != MS_TPC_WRITE_LONG_DATA)
		reinit_completion(&dev->fifo_done);

	return 0;
}

static int bcm577x5_ms_finish_fifo(struct bcm577x5_device *dev)
{
	struct sg_mapping_iter miter;

	if (!wait_for_completion_timeout(&dev->fifo_done,
					 msecs_to_jiffies(BCM577x5_MS_REQ_TIMEOUT_MS)))
		return -ETIMEDOUT;

	sg_miter_start(&miter, &dev->req->sg, 1,
		       dev->req->data_dir == READ ? SG_MITER_TO_SG :
						    SG_MITER_FROM_SG);
	while (sg_miter_next(&miter)) {
		if (WARN_ON_ONCE(!IS_ALIGNED(miter.length, 4))) {
			sg_miter_stop(&miter);
			return -EINVAL;
		}
		if (dev->req->data_dir == READ)
			ioread32_rep(dev->mmio + BCM577x5_MS_BUFFER,
				     miter.addr, miter.length / 4);
		else if (dev->req->data_dir == WRITE)
			iowrite32_rep(dev->mmio + BCM577x5_MS_BUFFER,
				      miter.addr, miter.length / 4);
	}
	sg_miter_stop(&miter);

	return 0;
}

static int bcm577x5_ms_execute_cmd_short(struct bcm577x5_device *dev, u32 cmd)
{
	unsigned int len = dev->req->data_len;
	u8 *data = dev->req->data;
	u32 response, val;
	int err;

	/* Write the data into the controller */
	if (dev->req->data_dir == WRITE) {
		bcm577x5_reg_writel(dev, BCM577x5_MS_ARGUMENT,
				    get_unaligned_le32(data));
		if (len > 4)
			bcm577x5_reg_writel(dev, BCM577x5_MS_ARGUMENT2,
					    get_unaligned_le32(data + 4));
	}

	reinit_completion(&dev->cmd_done);
	if (dev->req->tpc == MS_TPC_SET_CMD)
		reinit_completion(&dev->data_done);

	response = bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE);
	bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG, cmd);

	if (!wait_for_completion_timeout(&dev->cmd_done,
					 msecs_to_jiffies(100)))
		return -ETIMEDOUT;

	if (dev->req->tpc == MS_TPC_SET_CMD) {
		/*
		 * The controller will execute a GET_INT command, wait for it.
		 * Sometimes, the second interrupt will coalesce with the first,
		 * or arrive after the response changed, so wait for the response.
		 */
		readl_poll_timeout(dev->mmio + BCM577x5_MS_RESPONSE, val,
				   val != response, BCM577x5_MS_POLL_DELAY_US,
				   100 * USEC_PER_MSEC);
		/* A second interrupt should come within this time */
		usleep_range(20, 30);
		if (bcm577x5_reg_readl(dev, BCM577x5_MS_INT_STATUS) &
		    BCM577x5_MS_INT_RESPONSE)
			wait_for_completion_timeout(&dev->cmd_done,
						    msecs_to_jiffies(5));

		/* The int reg is placed in the response register */
		dev->auto_int =
			bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE) & 0xFF;
		dev->req->int_reg = dev->auto_int;

		/*
		 * On legacy MS cards, the controller doesn't automatically issue a
		 * GET_INT after the command is done, but it still requires one to
		 * keep working without getting stuck
		 */
		dev->get_int_after_done = dev->auto_int ==
				    (MEMSTICK_INT_CED | MEMSTICK_INT_BREQ);

		/*
		 * Reset is needed because the controller will expect certain commands
		 * to follow the SET_CMD command, which the block driver may not
		 * necessarily issue.
		 */
		if (!(dev->auto_int & MEMSTICK_INT_BREQ) || dev->get_int_after_done) {
			err = bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA);
			if (err)
				return err;
		}
	}

	/* Read the data back */
	if (dev->req->data_dir == READ) {
		put_unaligned_le32(bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE),
				   data);
		if (len > 4)
			put_unaligned_le32(bcm577x5_reg_readl(dev,
							      BCM577x5_MS_RESPONSE2),
					   data + 4);
	}

	return 0;
}

static int bcm577x5_ms_execute_cmd_long(struct bcm577x5_device *dev, u32 cmd, bool use_dma)
{
	int err;

	if (use_dma) {
		err = bcm577x5_ms_prepare_dma(dev);
		if (err)
			use_dma = false;
	}
	if (!use_dma)
		err = bcm577x5_ms_prepare_fifo(dev);

	if (use_dma)
		cmd |= BCM577x5_MS_TRNS_DMA;

	if (dev->req->data_dir == WRITE ||
	    (!use_dma && !dev->get_int_after_done)) {
		/* PIO or DMA write has to be multi-block */
		bcm577x5_reg_writew(dev, BCM577x5_MS_BLOCK_COUNT, 0xFFFF);
		cmd |= BCM577x5_MS_TRNS_MULTI | BCM577x5_MS_TRNS_BLK_CNT_EN;
	}

	/* Execute the TPC */
	if (dev->prev_tpc == MS_TPC_READ_LONG_DATA ||
	    dev->prev_tpc == MS_TPC_WRITE_LONG_DATA)
		/*
		 * The controller will hang if another TPC is written,
		 * this is a kind of continue with the last TPC command
		 */
		bcm577x5_reg_writew(dev, BCM577x5_MS_TPC_REG, cmd & 0xFFFF);
	else
		bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG, cmd);

	if (use_dma)
		err = bcm577x5_ms_finish_dma(dev);
	else
		err = bcm577x5_ms_finish_fifo(dev);

	if (!err && dev->get_int_after_done) {
		/*
		 * On legacy MS cards, we need to manually issue a GET_INT.
		 * This GET_INT is special because it interrupts with
		 * data_done, not cmd_done
		 */
		dev->get_int_after_done = false;
		reinit_completion(&dev->data_done);
		bcm577x5_reg_writew(dev, BCM577x5_MS_BLOCK_SIZE, 1);
		bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG,
				    BCM577x5_MS_MAKE_TPC(MS_TPC_GET_INT) |
					    BCM577x5_MS_TRNS_READ);
		if (!wait_for_completion_timeout(&dev->data_done,
						 msecs_to_jiffies(100)))
			err = -ETIMEDOUT;
	}

	dev->auto_int = bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE) & 0xFF;
	dev->req->int_reg = dev->auto_int;

	return err;
}

static int bcm577x5_ms_execute_cmd(struct bcm577x5_device *dev)
{
	int len =
		dev->req->long_data ? dev->req->sg.length : dev->req->data_len;
	bool use_dma = READ_ONCE(dev->enable_dma);
	unsigned long timeout;
	int err;
	u32 cmd;

	/* Immediately return if there is no card inserted */
	if (!(bcm577x5_reg_readl(dev, BCM577x5_MS_STATUS) &
	      BCM577x5_MS_STATUS_MS_DETECT))
		return -ENODEV;

	if (dev->req->tpc == MS_TPC_GET_INT && dev->auto_int &&
	    (dev->prev_tpc == MS_TPC_SET_CMD ||
	     dev->prev_tpc == MS_TPC_READ_LONG_DATA ||
	     dev->prev_tpc == MS_TPC_WRITE_LONG_DATA)) {
		dev->req->data[0] = dev->auto_int;
		dev->req->int_reg = dev->auto_int;
		dev->auto_int = 0;
		return 0;
	}

	if (dev->req->tpc == MS_TPC_GET_INT &&
	    dev->prev_tpc == MS_TPC_READ_LONG_DATA && use_dma) {
		/*
		 * The controller doesn't respond to any command until DATA_END
		 * is completed.
		 */
		if (!wait_for_completion_timeout(&dev->data_done,
						 msecs_to_jiffies(BCM577x5_MS_REQ_TIMEOUT_MS)))
			return -ETIMEDOUT;

		err = bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_DATA);
		if (err)
			return err;
	}

	/* PIO has no DATA_END but needs the same treatment */
	if (dev->req->tpc == MS_TPC_GET_INT &&
	    (dev->prev_tpc == MS_TPC_WRITE_LONG_DATA ||
	     (dev->prev_tpc == MS_TPC_READ_LONG_DATA && !use_dma))) {
		/*
		 * There seems to be no way to wait for the command to actually finish,
		 * since DMA_END only signals the data is in the controller, but may not
		 * be fully written into the card. Instead, the controller ignores any
		 * commands until it is done, so keep issuing GET_INT until it
		 * goes through
		 */
		bcm577x5_reg_writel(dev, BCM577x5_MS_BLOCK_SIZE, len);
		cmd = BCM577x5_MS_MAKE_TPC(MS_TPC_GET_INT) | BCM577x5_MS_TRNS_READ;
		timeout = jiffies + msecs_to_jiffies(BCM577x5_MS_REQ_TIMEOUT_MS);
		/* A small wait is needed for the write data to be fully written */
		if (dev->prev_tpc == MS_TPC_WRITE_LONG_DATA)
			usleep_range(150, 300);
		while (!time_after(jiffies, timeout)) {
			reinit_completion(&dev->cmd_done);
			bcm577x5_reg_writel(dev, BCM577x5_MS_TPC_REG, cmd);
			/* 1 jiffy is too coarse */
			usleep_range(200, 220);
			if (completion_done(&dev->cmd_done))
				break;
		}
		if (!completion_done(&dev->cmd_done))
			return -ETIMEDOUT;

		dev->req->data[0] =
			bcm577x5_reg_readl(dev, BCM577x5_MS_RESPONSE) & 0xFF;
		dev->req->int_reg = dev->req->data[0];
		if (!(dev->req->data[0] & MEMSTICK_INT_CED))
			return 0;

		dev->prev_tpc = dev->req->tpc;
		return bcm577x5_ms_reset(dev, BCM577x5_MS_RESET_CMD |
					      BCM577x5_MS_RESET_DATA);
	}

	/* Same for the next page of a write */
	if (dev->req->tpc == MS_TPC_WRITE_LONG_DATA &&
	    dev->prev_tpc == MS_TPC_WRITE_LONG_DATA)
		usleep_range(100, 120);

	/* Write the length of the transfer */
	bcm577x5_reg_writel(dev, BCM577x5_MS_BLOCK_SIZE, len);

	/* Create the execution command */
	cmd = BCM577x5_MS_MAKE_TPC(dev->req->tpc);
	if (dev->req->data_dir == READ)
		cmd |= BCM577x5_MS_TRNS_READ;

	/*
	 * Two types of transfers have very different flow,
	 * so split them into functions
	 */
	if (dev->req->long_data)
		err = bcm577x5_ms_execute_cmd_long(dev, cmd, use_dma);
	else
		err = bcm577x5_ms_execute_cmd_short(dev, cmd);

	dev->prev_tpc = dev->req->tpc;
	return err;
}

static void bcm577x5_ms_req_process(struct work_struct *data)
{
	struct bcm577x5_device *dev =
		container_of(data, struct bcm577x5_device, req_work);
	int err, cmderr = 0;

	/* Already processing requests */
	if (dev->req)
		return;

	mutex_lock(&dev->req_lock);
	do {
		err = memstick_next_req(dev->host, &dev->req);

		if (!err && completion_done(&dev->host->card->mrq_complete)) {
			dev->req = NULL;
			break;
		}

		if (!err) {
			atomic_set(&dev->int_seen, 0);
			cmderr = bcm577x5_ms_execute_cmd(dev);
			if (atomic_read(&dev->int_seen) &
			    BCM577x5_MS_INT_CARD_REMOVE)
				cmderr = -ENODEV;
			dev->req->error = cmderr;
		}
	} while (!err && !cmderr);

	if (cmderr) {
		/* Reset the controller so it accepts new commands */
		bcm577x5_ms_init(dev);

		/* Controller will not respond to any more commands anyway */
		while (!err && dev->req) {
			dev->req->error = cmderr;
			err = memstick_next_req(dev->host, &dev->req);
		}
	}
	mutex_unlock(&dev->req_lock);
}

static void bcm577x5_ms_submit_req(struct memstick_host *host)
{
	struct bcm577x5_device *dev = memstick_priv(host);

	schedule_work(&dev->req_work);
}

static int bcm577x5_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int error;
	struct memstick_host *host;
	struct bcm577x5_device *dev;
	u32 val;

	host = memstick_alloc_host(sizeof(struct bcm577x5_device), &pdev->dev);
	if (!host)
		return -ENOMEM;

	dev = memstick_priv(host);
	dev->host = host;
	dev->pci_dev = pdev;
	pci_set_drvdata(pdev, dev);

	error = pci_enable_device(pdev);
	if (error)
		goto err_free_host;

	pci_set_master(pdev);

	error = pci_request_regions(pdev, DRV_NAME);
	if (error)
		goto err_disable_device;

	dev->mmio = pci_ioremap_bar(pdev, 0);
	if (!dev->mmio) {
		error = -EIO;
		goto err_release_regions;
	}

	INIT_WORK(&dev->req_work, bcm577x5_ms_req_process);

	host->caps = MEMSTICK_CAP_PAR4 | MEMSTICK_CAP_AUTO_GET_INT;
	host->request = bcm577x5_ms_submit_req;
	host->set_param = bcm577x5_ms_set_param;

	val = bcm577x5_reg_readl(dev, BCM577x5_MS_CAPABILITY_SLOT2);

	/* An alternate clock source is found that gives higher write reliability */
	dev->base_clock = 62500000;

	if (val & BCM577x5_MS_CAPABILITY_SDMA) {
		dev->enable_dma = true;
		if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
			dev_warn(&pdev->dev, "No suitable DMA available\n");
			dev->enable_dma = false;
		}
	}

	if (dev->enable_dma) {
		/*
		 * This needs to be 4096 aligned since the hardware does
		 * not seem to issue a dma complete interrupt, so we have
		 * to rely on address line overflows to detect completion.
		 */
		dev->dma_buffer = dma_alloc_coherent(&pdev->dev, 4096,
						     &dev->dma_handle,
						     GFP_KERNEL);
		if (!dev->dma_buffer) {
			dev_warn(&pdev->dev,
				 "DMA memory alloc failed, disabling\n");
			dev->enable_dma = false;
		}
	}

	/* Disable interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE, 0);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE, 0);

	dev->irq = pdev->irq;
	dev_info(&pdev->dev, "Rev %c%c dma=%d clock=%dHz irq=%d\n",
		 (pdev->revision >> 3) + 'A', (pdev->revision & 0b111) + '0',
		 dev->enable_dma, dev->base_clock, dev->irq);

	mutex_init(&dev->req_lock);
	init_completion(&dev->cmd_done);
	init_completion(&dev->fifo_done);
	init_completion(&dev->dma_done);
	init_completion(&dev->data_done);
	error = request_irq(dev->irq, &bcm577x5_irq, IRQF_SHARED, DRV_NAME, dev);
	if (error)
		goto err_iounmap;

	error = bcm577x5_ms_init(dev);
	if (error)
		goto err_free_irq;

	error = memstick_add_host(host);
	if (error)
		goto err_free_irq;

	/* Switch between the DMA and PIO paths at runtime */
	if (dev->enable_dma) {
		dev->debugfs = debugfs_create_dir(dev_name(&pdev->dev), NULL);
		debugfs_create_bool("enable_dma", 0644, dev->debugfs,
				    &dev->enable_dma);
	}

	return 0;
err_free_irq:
	bcm577x5_ms_power(dev, MEMSTICK_POWER_OFF);
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE, 0);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE, 0);
	free_irq(dev->irq, dev);
err_iounmap:
	iounmap(dev->mmio);
	if (dev->dma_buffer) {
		dma_free_coherent(&dev->pci_dev->dev, 4096, dev->dma_buffer,
				  dev->dma_handle);
	}
err_release_regions:
	pci_release_regions(pdev);
err_disable_device:
	pci_disable_device(pdev);
err_free_host:
	memstick_free_host(host);
	return error;
}

static void bcm577x5_remove(struct pci_dev *pdev)
{
	struct bcm577x5_device *dev = pci_get_drvdata(pdev);

	debugfs_remove(dev->debugfs);

	dev->host->removing = true;
	memstick_remove_host(dev->host);

	/* Cancel the work before interrupts are disabled */
	cancel_work_sync(&dev->req_work);

	/* Clear interrupts */
	bcm577x5_reg_writel(dev, BCM577x5_MS_INT_ENABLE, 0);
	bcm577x5_reg_writel(dev, BCM577x5_MS_SIGNAL_ENABLE, 0);

	if (dev->dma_buffer) {
		dma_free_coherent(&dev->pci_dev->dev, 4096, dev->dma_buffer,
				  dev->dma_handle);
	}

	free_irq(dev->irq, dev);
	iounmap(dev->mmio);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	memstick_free_host(dev->host);
}

static int bcm577x5_ms_suspend(struct device *pdev)
{
	struct bcm577x5_device *dev = dev_get_drvdata(pdev);

	memstick_suspend_host(dev->host);
	return 0;
}

static int bcm577x5_ms_resume(struct device *pdev)
{
	struct bcm577x5_device *dev = dev_get_drvdata(pdev);
	int error;

	error = bcm577x5_ms_init(dev);
	if (error)
		return error;

	memstick_resume_host(dev->host);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bcm577x5_ms_pm_ops, bcm577x5_ms_suspend,
				bcm577x5_ms_resume);

static struct pci_driver bcm577x5_pci_driver = {
	.name = DRV_NAME,
	.id_table = bcm577x5_pci_id_tbl,
	.probe = bcm577x5_probe,
	.remove = bcm577x5_remove,
	.driver.pm = pm_sleep_ptr(&bcm577x5_ms_pm_ops),
};

module_pci_driver(bcm577x5_pci_driver);

MODULE_AUTHOR("Ho Jie Feng <hjf3108@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BCM577x5 MS/MSPro card reader driver");
