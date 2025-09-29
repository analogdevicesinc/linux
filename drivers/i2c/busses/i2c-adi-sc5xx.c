// SPDX-License-Identifier: GPL-2.0
/*
 * ADI On-Chip Two Wire Interface Driver
 *
 * Copyright 2022-2024 - Analog Devices Inc.
 */

#include <asm/irq.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

/* TWI_PRESCALE Masks */
#define	TWI_ENA		0x0080	/* TWI Enable */

/* TWI_MASTER_CTL Masks	*/
#define	MEN		0x0001	/* Master Mode Enable          */
#define	MDIR		0x0004	/* Master Transmit Direction (RX/TX*) */
#define	FAST		0x0008	/* Use Fast Mode Timing Specs  */
#define	STOP		0x0010	/* Issue Stop Condition        */
#define	RSTART		0x0020	/* Repeat Start or Stop* At End Of Transfer */
#define	SDAOVR		0x4000	/* Serial Data Override        */
#define	SCLOVR		0x8000	/* Serial Clock Override       */

/* TWI_MASTER_STAT Masks */
#define	LOSTARB		0x0002	/* Lost Arbitration Indicator (Xfer Aborted) */
#define	ANAK		0x0004	/* Address Not Acknowledged    */
#define	DNAK		0x0008	/* Data Not Acknowledged       */
#define	BUFRDERR	0x0010	/* Buffer Read Error           */
#define	BUFWRERR	0x0020	/* Buffer Write Error          */
#define	SDASEN		0x0040	/* Serial Data Sense           */
#define	BUSBUSY		0x0100	/* Bus Busy Indicator          */

/* TWI_INT_SRC and TWI_INT_ENABLE Masks	*/
#define	MCOMP		0x0010	/* Master Transfer Complete    */
#define	MERR		0x0020	/* Master Transfer Error       */
#define	XMTSERV		0x0040	/* Transmit FIFO Service       */
#define	RCVSERV		0x0080	/* Receive FIFO Service        */

/* TWI_FIFO_STAT Masks */
#define	XMTSTAT		0x0003	/* Transmit FIFO Status                  */
#define	XMT_FULL	0x0003	/* Transmit FIFO Full (2 Bytes To Write) */
#define	RCVSTAT		0x000C	/* Receive FIFO Status                   */

/* SMBus mode*/
#define TWI_I2C_MODE_STANDARD		1
#define TWI_I2C_MODE_STANDARDSUB	2
#define TWI_I2C_MODE_COMBINED		3
#define TWI_I2C_MODE_REPEAT		4

/*
 * ADI I2C/TWI registers layout
 */
#define ADI_I2C_CLKDIV_REG	0x0
#define ADI_I2C_CTL_REG		0x4
#define ADI_I2C_SLVCTTL_REG	0x8
#define ADI_I2C_SLVSTAT_REG	0xC
#define ADI_I2C_SLVADDR_REG	0x10
#define ADI_I2C_MSTRCTRL_REG	0x14
#define ADI_I2C_MSTRSTAT_REG	0x18
#define ADI_I2C_MSTRADDR_REG	0x1C
#define ADI_I2C_ISTAT_REG	0x20
#define ADI_I2C_IMSK_REG	0x24
#define ADI_I2C_FIFOCTL_REG	0x28
#define ADI_I2C_FIFOSTAT_REG	0x2C
#define ADI_I2C_TXDATA8_REG	0x80
#define ADI_I2C_TXDATA16_REG	0x84
#define ADI_I2C_RXDATA8_REG	0x88
#define ADI_I2C_RXDATA16_REG	0x8C

struct adi_twi_iface {
	int			irq;
	//spinlock_t		lock;
	char			read_write;
	u8			command;
	u8			*trans_ptr;
	int			read_num;
	int			write_num;
	int			cur_mode;
	int			manual_stop;
	int			result;
	unsigned int		twi_clk;
	struct i2c_adapter	adap;
	struct completion	complete;
	struct i2c_msg		*pmsg;
	int			msg_num;
	int			cur_msg;
	u16			saved_clkdiv;
	u16			saved_control;
	void __iomem		*reg_base;
	struct clk *sclk;
};

static void adi_twi_handle_interrupt(struct adi_twi_iface *i2c,
				     unsigned short twi_int_status,
				     bool polling)
{
	u16 write_value;
	unsigned short mast_stat = readw(i2c->reg_base + ADI_I2C_MSTRSTAT_REG);

	if (twi_int_status & XMTSERV) {
		if (i2c->write_num <= 0) {
			/* start receive immediately after complete sending in
			 * combine mode.
			 */
			if (i2c->cur_mode == TWI_I2C_MODE_COMBINED) {
				write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | MDIR;
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			} else if (i2c->manual_stop) {
				write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | STOP;
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			} else if (i2c->cur_mode == TWI_I2C_MODE_REPEAT &&
				   i2c->cur_msg + 1 < i2c->msg_num) {
				if (i2c->pmsg[i2c->cur_msg + 1].flags & I2C_M_RD) {
					write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG)
							      | MDIR;
					writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				} else {
					write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG)
							      & ~MDIR;
					writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				}
			}
		}
		/* Transmit next data */
		while (i2c->write_num > 0 &&
		       (readw(i2c->reg_base + ADI_I2C_FIFOSTAT_REG) & XMTSTAT) != XMT_FULL) {
			writew(*(i2c->trans_ptr++), i2c->reg_base + ADI_I2C_TXDATA8_REG);
			i2c->write_num--;
		}
	}
	if (twi_int_status & RCVSERV) {
		while (i2c->read_num > 0 &&
		       (readw(i2c->reg_base + ADI_I2C_FIFOSTAT_REG) & RCVSTAT)) {
			/* Receive next data */
			*i2c->trans_ptr = readw(i2c->reg_base + ADI_I2C_RXDATA8_REG);
			if (i2c->cur_mode == TWI_I2C_MODE_COMBINED) {
				/* Change combine mode into sub mode after
				 * read first data.
				 */
				i2c->cur_mode = TWI_I2C_MODE_STANDARDSUB;
				/* Get read number from first byte in block
				 * combine mode.
				 */
				if (i2c->read_num == 1 && i2c->manual_stop)
					i2c->read_num = *i2c->trans_ptr + 1;
			}
			i2c->trans_ptr++;
			i2c->read_num--;
		}

		if (i2c->read_num == 0) {
			if (i2c->manual_stop) {
				/* Temporary workaround to avoid possible bus stall -
				 * Flush FIFO before issuing the STOP condition
				 */
				readw(i2c->reg_base + ADI_I2C_RXDATA16_REG);
				write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | STOP;
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			} else if (i2c->cur_mode == TWI_I2C_MODE_REPEAT &&
					i2c->cur_msg + 1 < i2c->msg_num) {
				if (i2c->pmsg[i2c->cur_msg + 1].flags & I2C_M_RD) {
					write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) |
						     MDIR;
					writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				} else {
					write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) &
						     ~MDIR;
					writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				}
			}
		}
	}
	if (twi_int_status & MERR) {
		writew(0, i2c->reg_base + ADI_I2C_IMSK_REG);
		writew(0x3e, i2c->reg_base + ADI_I2C_MSTRSTAT_REG);
		writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		i2c->result = -EIO;

		if (mast_stat & LOSTARB)
			dev_dbg(&i2c->adap.dev, "Lost Arbitration\n");
		if (mast_stat & ANAK)
			dev_dbg(&i2c->adap.dev, "Address Not Acknowledged\n");
		if (mast_stat & DNAK)
			dev_dbg(&i2c->adap.dev, "Data Not Acknowledged\n");
		if (mast_stat & BUFRDERR)
			dev_dbg(&i2c->adap.dev, "Buffer Read Error\n");
		if (mast_stat & BUFWRERR)
			dev_dbg(&i2c->adap.dev, "Buffer Write Error\n");

		/* Faulty slave devices, may drive SDA low after a transfer
		 * finishes. To release the bus this code generates up to 9
		 * extra clocks until SDA is released.
		 */

		if (readw(i2c->reg_base + ADI_I2C_MSTRSTAT_REG) & SDASEN) {
			int cnt = 9;

			do {
				writew(SCLOVR, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				udelay(6);
				writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				udelay(6);
			} while ((readw(i2c->reg_base + ADI_I2C_MSTRSTAT_REG) & SDASEN) && cnt--);

			writew(SDAOVR | SCLOVR, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			udelay(6);
			writew(SDAOVR, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			udelay(6);
			writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		}

		/* If it is a quick transfer, only address without data,
		 * not an err, return 1.
		 */
		if (i2c->cur_mode == TWI_I2C_MODE_STANDARD && !i2c->trans_ptr &&
		    (twi_int_status & MCOMP) && (mast_stat & DNAK))
			i2c->result = 1;

		if (!polling)
			complete(&i2c->complete);
		return;
	}
	if (twi_int_status & MCOMP) {
		if (twi_int_status & (XMTSERV | RCVSERV) &&
		    (readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) & MEN) == 0 &&
		    (i2c->cur_mode == TWI_I2C_MODE_REPEAT ||
		    i2c->cur_mode == TWI_I2C_MODE_COMBINED)) {
			i2c->result = -1;
			writew(0, i2c->reg_base + ADI_I2C_IMSK_REG);
			writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		} else if (i2c->cur_mode == TWI_I2C_MODE_COMBINED) {
			if (i2c->read_num == 0) {
				/* set the read number to 1 and ask for manual
				 * stop in block combine mode
				 */
				i2c->read_num = 1;
				i2c->manual_stop = 1;
				write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | (0xff << 6);
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			} else {
				/* set the readd number in other
				 * combine mode.
				 */
				write_value = (readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG)
					     & (~(0xff << 6))) | (i2c->read_num << 6);
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			}
			/* remove restart bit and enable master receive */
			write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) & ~RSTART;
			writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		} else if (i2c->cur_mode == TWI_I2C_MODE_REPEAT &&
				i2c->cur_msg + 1 < i2c->msg_num) {
			i2c->cur_msg++;
			i2c->trans_ptr = i2c->pmsg[i2c->cur_msg].buf;
			i2c->write_num = i2c->pmsg[i2c->cur_msg].len;
			i2c->read_num = i2c->pmsg[i2c->cur_msg].len;
			/* Set Transmit device address */
			writew(i2c->pmsg[i2c->cur_msg].addr, i2c->reg_base + ADI_I2C_MSTRADDR_REG);
			if (i2c->pmsg[i2c->cur_msg].flags & I2C_M_RD) {
				i2c->read_write = I2C_SMBUS_READ;
			} else {
				i2c->read_write = I2C_SMBUS_WRITE;
				/* Transmit first data */
				if (i2c->write_num > 0) {
					writew(*(i2c->trans_ptr++),
					       i2c->reg_base + ADI_I2C_TXDATA8_REG);
					i2c->write_num--;
				}
			}

			if (i2c->pmsg[i2c->cur_msg].len <= 255) {
				write_value = (readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG)
					     & (~(0xff << 6)))
					     | (i2c->pmsg[i2c->cur_msg].len << 6);
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				i2c->manual_stop = 0;
			} else {
				write_value = (readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG)
					     | (0xff << 6));
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				i2c->manual_stop = 1;
			}

			/* remove restart bit before last message */
			if (i2c->cur_msg + 1 == i2c->msg_num) {
				write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) & ~RSTART;
				writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			}

		} else {
			i2c->result = 1;
			writew(0, i2c->reg_base + ADI_I2C_IMSK_REG);
			writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		}
		if (!polling)
			complete(&i2c->complete);
	}
}

/* Interrupt handler */
static irqreturn_t adi_twi_handle_all_interrupts(struct adi_twi_iface *i2c,
						 bool polling)
{
	unsigned short twi_int_status;

	twi_int_status = readw(i2c->reg_base + ADI_I2C_ISTAT_REG);
	if (!twi_int_status)
		return IRQ_NONE;
	writew(twi_int_status, i2c->reg_base + ADI_I2C_ISTAT_REG);
	adi_twi_handle_interrupt(i2c, twi_int_status, polling);

	return IRQ_HANDLED;
}

static irqreturn_t adi_twi_interrupt_entry(int irq, void *dev_id)
{
	struct adi_twi_iface *i2c = dev_id;

	return adi_twi_handle_all_interrupts(i2c, false);
}

/*
 * One i2c master transfer
 */
static int adi_twi_do_master_xfer(struct i2c_adapter *adap,
				  struct i2c_msg *msgs, int num, bool polling)
{
	struct adi_twi_iface *i2c = adap->algo_data;
	struct i2c_msg *pmsg;
	int ret = 0;
	u16 write_value;

	if (!(readw(i2c->reg_base + ADI_I2C_CTL_REG) & TWI_ENA))
		return -ENXIO;

	if (readw(i2c->reg_base + ADI_I2C_MSTRSTAT_REG) & BUSBUSY)
		return -EAGAIN;

	i2c->pmsg = msgs;
	i2c->msg_num = num;
	i2c->cur_msg = 0;

	pmsg = &msgs[0];
	if (pmsg->flags & I2C_M_TEN) {
		dev_err(&adap->dev, "10 bits addr not supported!\n");
		return -EINVAL;
	}

	if (i2c->msg_num > 1)
		i2c->cur_mode = TWI_I2C_MODE_REPEAT;
	i2c->manual_stop = 0;
	i2c->trans_ptr = pmsg->buf;
	i2c->write_num = pmsg->len;
	i2c->read_num = pmsg->len;
	i2c->result = 0;
	if (!polling)
		init_completion(&i2c->complete);
	/* Set Transmit device address */
	writew(pmsg->addr, i2c->reg_base + ADI_I2C_MSTRADDR_REG);

	/* FIFO Initiation. Data in FIFO should be
	 *  discarded before start a new operation.
	 */
	writew(0x3, i2c->reg_base + ADI_I2C_FIFOCTL_REG);
	writew(0, i2c->reg_base + ADI_I2C_FIFOCTL_REG);

	if (pmsg->flags & I2C_M_RD) {
		i2c->read_write = I2C_SMBUS_READ;
	} else {
		i2c->read_write = I2C_SMBUS_WRITE;
		/* Transmit first data */
		if (i2c->write_num > 0) {
			writew(*(i2c->trans_ptr++), i2c->reg_base + ADI_I2C_TXDATA8_REG);
			i2c->write_num--;
		}
	}

	/* clear int stat */
	writew(MERR | MCOMP | XMTSERV | RCVSERV, i2c->reg_base + ADI_I2C_ISTAT_REG);

	/* Interrupt mask . Enable XMT, RCV interrupt */
	writew(MCOMP | MERR | RCVSERV | XMTSERV, i2c->reg_base + ADI_I2C_IMSK_REG);

	if (pmsg->len <= 255) {
		writew(pmsg->len << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
	} else {
		writew(0xff << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		i2c->manual_stop = 1;
	}

	/* Master enable */
	write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) |
		     MEN |
		     ((i2c->msg_num > 1) ? RSTART : 0) |
		     ((i2c->read_write == I2C_SMBUS_READ) ? MDIR : 0) |
		     ((i2c->twi_clk > 100) ? FAST : 0);

	writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);

	if (polling) {
		int timeout = 50000;

		for (;;) {
			irqreturn_t handled = adi_twi_handle_all_interrupts(i2c, true);

			if (handled == IRQ_HANDLED && i2c->result)
				break;
			if (--timeout == 0) {
				i2c->result = -1;
				dev_err(&adap->dev, "master polling timeout");
				break;
			}
		}
	} else { /* interrupt driven */
		while (!i2c->result) {
			if (!wait_for_completion_timeout(&i2c->complete, adap->timeout)) {
				i2c->result = -1;
				dev_err(&adap->dev, "master transfer timeout");
			}
		}
	}

	if (i2c->result == 1)
		ret = i2c->cur_msg + 1;
	else
		ret = i2c->result;

	return ret;
}

/*
 * Generic i2c master transfer entrypoint
 */
static int adi_twi_master_xfer(struct i2c_adapter *adap,
			       struct i2c_msg *msgs, int num)
{
	return adi_twi_do_master_xfer(adap, msgs, num, false);
}

static int adi_twi_master_xfer_atomic(struct i2c_adapter *adap,
				      struct i2c_msg *msgs, int num)
{
	return adi_twi_do_master_xfer(adap, msgs, num, true);
}

/*
 * One I2C SMBus transfer
 */
static int adi_twi_do_smbus_xfer(struct i2c_adapter *adap, u16 addr,
				 unsigned short flags, char read_write,
				 u8 command, int size, union i2c_smbus_data *data,
				 bool polling)
{
	struct adi_twi_iface *i2c = adap->algo_data;
	int ret = 0;
	u16 write_value;

	if (!(readw(i2c->reg_base + ADI_I2C_CTL_REG) & TWI_ENA))
		return -ENXIO;

	if (readw(i2c->reg_base + ADI_I2C_MSTRSTAT_REG) & BUSBUSY)
		return -EAGAIN;

	i2c->write_num = 0;
	i2c->read_num = 0;

	/* Prepare datas & select mode */
	switch (size) {
	case I2C_SMBUS_QUICK:
		i2c->trans_ptr = NULL;
		i2c->cur_mode = TWI_I2C_MODE_STANDARD;
		break;
	case I2C_SMBUS_BYTE:
		if (!data) {
			i2c->trans_ptr = NULL;
		} else {
			if (read_write == I2C_SMBUS_READ)
				i2c->read_num = 1;
			else
				i2c->write_num = 1;
			i2c->trans_ptr = &data->byte;
		}
		i2c->cur_mode = TWI_I2C_MODE_STANDARD;
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ) {
			i2c->read_num = 1;
			i2c->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			i2c->write_num = 1;
			i2c->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		i2c->trans_ptr = &data->byte;
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ) {
			i2c->read_num = 2;
			i2c->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			i2c->write_num = 2;
			i2c->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		i2c->trans_ptr = (u8 *)&data->word;
		break;
	case I2C_SMBUS_PROC_CALL:
		i2c->write_num = 2;
		i2c->read_num = 2;
		i2c->cur_mode = TWI_I2C_MODE_COMBINED;
		i2c->trans_ptr = (u8 *)&data->word;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			i2c->read_num = 0;
			i2c->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			i2c->write_num = data->block[0] + 1;
			i2c->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		i2c->trans_ptr = data->block;
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			i2c->read_num = data->block[0];
			i2c->cur_mode = TWI_I2C_MODE_COMBINED;
		} else {
			i2c->write_num = data->block[0];
			i2c->cur_mode = TWI_I2C_MODE_STANDARDSUB;
		}
		i2c->trans_ptr = (u8 *)&data->block[1];
		break;
	default:
		return -1;
	}

	i2c->result = 0;
	i2c->manual_stop = 0;
	i2c->read_write = read_write;
	i2c->command = command;
	if (!polling)
		init_completion(&i2c->complete);

	/* FIFO Initiation. Data in FIFO should be discarded before
	 * start a new operation.
	 */
	writew(0x3, i2c->reg_base + ADI_I2C_FIFOCTL_REG);
	writew(0, i2c->reg_base + ADI_I2C_FIFOCTL_REG);

	/* clear int stat */
	writew(MERR | MCOMP | XMTSERV | RCVSERV, i2c->reg_base + ADI_I2C_ISTAT_REG);

	/* Set Transmit device address */
	writew(addr, i2c->reg_base + ADI_I2C_MSTRADDR_REG);

	switch (i2c->cur_mode) {
	case TWI_I2C_MODE_STANDARDSUB:
		writew(i2c->command, i2c->reg_base + ADI_I2C_TXDATA8_REG);

		write_value = MCOMP | MERR;
		if (i2c->read_write == I2C_SMBUS_READ)
			write_value |= RCVSERV;
		else
			write_value |= XMTSERV;

		writew(write_value, i2c->reg_base + ADI_I2C_IMSK_REG);

		if (i2c->write_num + 1 <= 255) {
			writew((i2c->write_num + 1) << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		} else {
			writew(0xff << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
			i2c->manual_stop = 1;
		}
		/* Master enable */
		write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | MEN;
		if (i2c->twi_clk > 100)
			write_value |= FAST;
		writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		break;
	case TWI_I2C_MODE_COMBINED:
		writew(i2c->command, i2c->reg_base + ADI_I2C_TXDATA8_REG);
		writew(MCOMP | MERR | RCVSERV | XMTSERV, i2c->reg_base + ADI_I2C_IMSK_REG);

		if (i2c->write_num > 0)
			writew((i2c->write_num + 1) << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		else
			writew(0x1 << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		/* Master enable */
		write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | MEN | RSTART;
		if (i2c->twi_clk > 100)
			write_value |= FAST;
		writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		break;
	default:
		writew(0, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		if (size != I2C_SMBUS_QUICK) {
			/* Don't access xmit data register when this is a
			 * read operation.
			 */
			if (i2c->read_write != I2C_SMBUS_READ) {
				if (i2c->write_num > 0) {
					writew(*(i2c->trans_ptr++),
					       i2c->reg_base + ADI_I2C_TXDATA8_REG);
					if (i2c->write_num <= 255) {
						writew(i2c->write_num << 6,
						       i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
					} else {
						writew(0xff << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
						i2c->manual_stop = 1;
					}
					i2c->write_num--;
				} else {
					writew(i2c->command, i2c->reg_base + ADI_I2C_TXDATA8_REG);
					writew(1 << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				}
			} else {
				if (i2c->read_num > 0 && i2c->read_num <= 255) {
					writew(i2c->read_num << 6,
					       i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
				} else if (i2c->read_num > 255) {
					writew(0xff << 6, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
					i2c->manual_stop = 1;
				} else {
					break;
				}
			}
		}
		write_value = MCOMP | MERR;
		if (i2c->read_write == I2C_SMBUS_READ)
			write_value |= RCVSERV;
		else
			write_value |= XMTSERV;
		writew(write_value, i2c->reg_base + ADI_I2C_IMSK_REG);

		/* Master enable */
		write_value = readw(i2c->reg_base + ADI_I2C_MSTRCTRL_REG) | MEN |
			     ((i2c->read_write == I2C_SMBUS_READ) ? MDIR : 0) |
			     ((i2c->twi_clk > 100) ? FAST : 0);
		writew(write_value, i2c->reg_base + ADI_I2C_MSTRCTRL_REG);
		break;
	}

	if (polling) {
		int timeout = 50000;

		for (;;) {
			irqreturn_t handled = adi_twi_handle_all_interrupts(i2c, true);

			if (handled == IRQ_HANDLED && i2c->result)
				break;
			if (--timeout == 0) {
				i2c->result = -1;
				dev_err(&adap->dev, "smbus polling timeout");
				break;
			}
		}
	} else { /* interrupt driven */
		while (!i2c->result) {
			if (!wait_for_completion_timeout(&i2c->complete, adap->timeout)) {
				i2c->result = -1;
				dev_err(&adap->dev, "smbus transfer timeout");
			}
		}
	}

	ret = (i2c->result >= 0) ? 0 : -1;

	return ret;
}

/*
 * Generic I2C SMBus transfer entrypoint
 */
static int adi_twi_smbus_xfer(struct i2c_adapter *adap, u16 addr, unsigned short flags,
			      char read_write, u8 command, int size, union i2c_smbus_data *data)
{
	return adi_twi_do_smbus_xfer(adap, addr, flags,
			read_write, command, size, data, false);
}

static int adi_twi_smbus_xfer_atomic(struct i2c_adapter *adap, u16 addr,
				     unsigned short flags, char read_write,
				     u8 command, int size, union i2c_smbus_data *data)
{
	return adi_twi_do_smbus_xfer(adap, addr, flags,
			read_write, command, size, data, true);
}

/*
 * Return what the adapter supports
 */
static u32 adi_twi_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
	       I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
	       I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SMBUS_PROC_CALL |
	       I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm adi_twi_algorithm = {
	.master_xfer	    = adi_twi_master_xfer,
	.master_xfer_atomic = adi_twi_master_xfer_atomic,
	.smbus_xfer	    = adi_twi_smbus_xfer,
	.smbus_xfer_atomic  = adi_twi_smbus_xfer_atomic,
	.functionality	    = adi_twi_functionality,
};

static int i2c_adi_twi_suspend(struct device *dev)
{
	struct adi_twi_iface *i2c = dev_get_drvdata(dev);

	i2c->saved_clkdiv = readw(i2c->reg_base + ADI_I2C_CLKDIV_REG);
	i2c->saved_control = readw(i2c->reg_base + ADI_I2C_CTL_REG);

	free_irq(i2c->irq, i2c);

	/* Disable TWI */
	writew(i2c->saved_control & ~TWI_ENA, i2c->reg_base + ADI_I2C_CTL_REG);

	return 0;
}

static int i2c_adi_twi_resume(struct device *dev)
{
	struct adi_twi_iface *i2c = dev_get_drvdata(dev);

	int ret = request_irq(i2c->irq, adi_twi_interrupt_entry,
		0, to_platform_device(dev)->name, i2c);
	if (ret) {
		dev_err(dev, "Can't get IRQ %d !\n", i2c->irq);
		return -ENODEV;
	}

	/* Resume TWI interface clock as specified */
	writew(i2c->saved_clkdiv, i2c->reg_base + ADI_I2C_CLKDIV_REG);

	/* Resume TWI */
	writew(i2c->saved_control, i2c->reg_base + ADI_I2C_CTL_REG);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(i2c_adi_twi_pm, i2c_adi_twi_suspend, i2c_adi_twi_resume);

static const struct of_device_id adi_twi_of_match[] = {
	{
		.compatible = "adi,sc5xx-i2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_twi_of_match);

static int i2c_adi_twi_probe(struct platform_device *pdev)
{
	struct adi_twi_iface *i2c;
	struct i2c_adapter *adap;
	struct device_node *node = pdev->dev.of_node;
	int ret;
	unsigned int clkhilow;
	u16 write_value;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->reg_base))
		return PTR_ERR(i2c->reg_base);

	i2c->sclk = devm_clk_get(&pdev->dev, "sclk0");
	if (IS_ERR(i2c->sclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(i2c->sclk), "cannot get sclk0");

	/* Set TWI internal clock as 10MHz */
	ret = clk_prepare_enable(i2c->sclk);
	if (ret)
		return ret;

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return i2c->irq;

	ret = devm_request_irq(&pdev->dev, i2c->irq, adi_twi_interrupt_entry,
			       0, pdev->name, i2c);
	if (ret)
		return ret;

	if (of_property_read_u32(node, "clock-khz", &i2c->twi_clk))
		i2c->twi_clk = 50;

	adap = &i2c->adap;
	adap->nr = pdev->id;
	strscpy(adap->name, pdev->name, sizeof(adap->name));
	adap->algo = &adi_twi_algorithm;
	adap->algo_data = i2c;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = node;
	adap->timeout = 5 * HZ;
	adap->retries = 3;

	write_value = ((clk_get_rate(i2c->sclk) / 1000 / 1000 + 5) / 10) & 0x7F;
	writew(write_value, i2c->reg_base + ADI_I2C_CTL_REG);

	/*
	 * We will not end up with a CLKDIV=0 because no one will specify
	 * 20kHz SCL or less in Kconfig now. (5 * 1000 / 20 = 250)
	 */
	clkhilow = ((10 * 1000 / i2c->twi_clk) + 1) / 2;

	/* Set Twi interface clock as specified */
	write_value = (clkhilow << 8) | clkhilow;
	writew(write_value, i2c->reg_base + ADI_I2C_CLKDIV_REG);

	/* Enable TWI */
	write_value = readw(i2c->reg_base + ADI_I2C_CTL_REG) | TWI_ENA;
	writew(write_value, i2c->reg_base + ADI_I2C_CTL_REG);

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, i2c);

	return 0;
}

static void i2c_adi_twi_remove(struct platform_device *pdev)
{
	struct adi_twi_iface *i2c = platform_get_drvdata(pdev);

	clk_disable_unprepare(i2c->sclk);
	i2c_del_adapter(&i2c->adap);
}

static struct platform_driver i2c_adi_twi_driver = {
	.probe		= i2c_adi_twi_probe,
	.remove		= i2c_adi_twi_remove,
	.driver		= {
		.name	= "sc5xx-i2c",
		.pm	= &i2c_adi_twi_pm,
		.of_match_table = of_match_ptr(adi_twi_of_match),
	},
};

module_platform_driver(i2c_adi_twi_driver);

MODULE_AUTHOR("Bryan Wu, Sonic Zhang");
MODULE_DESCRIPTION("ADI on-chip I2C TWI Controller Driver");
MODULE_LICENSE("GPL");
