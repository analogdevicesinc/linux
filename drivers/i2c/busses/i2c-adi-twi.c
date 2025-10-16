// SPDX-License-Identifier: GPL-2.0+
/*
 * ADI On-Chip Two Wire Interface Driver
 *
 * Copyright 2005-2024 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/bitfield.h>

#include <asm/irq.h>

#define ADI_TWI_BUSY_TIMEOUT          50

/*
 * ADI SPI registers layout
 */
#define ADI_TWI_CLKDIV              0x00

#define ADI_TWI_CTL                 0x04
#define   TWI_ENA                   BIT(7)              /* TWI Enable */

#define ADI_TWI_SLVCTL              0x08
#define ADI_TWI_SLVSTAT             0x0c
#define ADI_TWI_SLVADDR             0x10

#define ADI_TWI_MSTRCTL             0x14
#define   SCLOVR                    BIT(15)             /* Serial Clock Override */
#define   SDAOVR                    BIT(14)             /* Serial Data Override */
#define   RSTART                    BIT(5)              /* Repeat Start or Stop* At End Of Transfer */
#define   DCNT                      GENMASK(13, 6)      /* Data Transfer Count */
#define   STOP                      BIT(4)              /* Issue Stop Condition */
#define   FAST                      BIT(3)              /* Use Fast Mode Timing Specs */
#define   MDIR                      BIT(2)              /* Master Transmit Direction (RX/TX*) */
#define   MEN                       BIT(0)              /* Master Mode Enable */

#define ADI_TWI_MSTRSTAT            0x18
#define   BUSBUSY                   BIT(8)              /* Bus Busy Indicator */
#define   SDASEN                    BIT(6)              /* Serial Data Sense */
#define   BUFWRERR                  BIT(5)              /* Buffer Write Error */
#define   BUFRDERR                  BIT(4)              /* Buffer Read Error */
#define   DNAK                      BIT(3)              /* Data Not Acknowledged */
#define   ANAK                      BIT(2)              /* Address Not Acknowledged */
#define   LOSTARB                   BIT(1)              /* Lost Arbitration Indicator (Xfer Aborted) */

#define ADI_TWI_MSTRADDR            0x1c
#define ADI_TWI_ISTAT               0x20

#define ADI_TWI_IMSK                0x24
#define   RCVSERV                   BIT(7)              /* Receive FIFO Service */
#define   XMTSERV                   BIT(6)              /* Transmit FIFO Service */
#define   MERR                      BIT(5)              /* Master Transfer Error */
#define   MCOMP                     BIT(4)              /* Master Transfer Complete */

#define ADI_TWI_FIFOCTL             0x28

#define ADI_TWI_FIFOSTAT            0x2c
#define   XMTSTAT                   0x0003              /* Transmit FIFO Status                  */
#define   XMT_FULL                  0x0003              /* Transmit FIFO Full (2 Bytes To Write) */
#define   RCVSTAT                   0x000C              /* Receive FIFO Status                   */

#define ADI_TWI_TXDATA8             0x80
#define ADI_TWI_TXDATA16            0x84
#define ADI_TWI_RXDATA8             0x88
#define ADI_TWI_RXDATA16            0x8c

struct adi_twi_dev {
	struct device *dev;
	void __iomem *base;
	int irq;
	spinlock_t lock;
	u8 *msg_buf;
	int msg_buf_remaining;
	bool manual_stop;
	bool last_msg;
	int result;
	unsigned int twi_clk;
	struct i2c_adapter adap;
	struct completion msg_complete;
	u16 saved_clkdiv;
	u16 saved_control;
	struct clk *sclk;
};

static void i2c_adi_twi_master_init(struct adi_twi_dev *priv)
{
	iowrite16(0x3e, priv->base + ADI_TWI_MSTRSTAT);
	iowrite16(0, priv->base + ADI_TWI_IMSK);
	iowrite16(0, priv->base + ADI_TWI_MSTRCTL);
}

static void i2c_adi_twi_handle_interrupt(struct adi_twi_dev *priv,
					 unsigned short twi_int_status,
					 bool polling)
{
	u16 val;
	unsigned short mstat = ioread16(priv->base + ADI_TWI_MSTRSTAT);

	if (twi_int_status & XMTSERV) {
		/* Transmit next data */
		while (priv->msg_buf_remaining &&
		       (ioread16(priv->base + ADI_TWI_FIFOSTAT) & XMTSTAT) != XMT_FULL) {
			iowrite16(*priv->msg_buf++, priv->base + ADI_TWI_TXDATA8);
			priv->msg_buf_remaining--;
		}

		if (!priv->msg_buf_remaining && priv->manual_stop) {
			val = ioread16(priv->base + ADI_TWI_MSTRCTL) | STOP;
			iowrite16(val, priv->base + ADI_TWI_MSTRCTL);
		}
	}

	if (twi_int_status & RCVSERV) {
		while (priv->msg_buf_remaining &&
		       (ioread16(priv->base + ADI_TWI_FIFOSTAT) & RCVSTAT)) {
			/* Receive next data */
			*priv->msg_buf++ = ioread16(priv->base + ADI_TWI_RXDATA8);
			priv->msg_buf_remaining--;
		}

		if (!priv->msg_buf_remaining && priv->manual_stop) {
			val = ioread16(priv->base + ADI_TWI_MSTRCTL) | STOP;
			iowrite16(val, priv->base + ADI_TWI_MSTRCTL);
		}
	}

	if (twi_int_status & MERR) {
		i2c_adi_twi_master_init(priv);

		/* If it is a quick transfer, only address without data,
		 * not an err, return 1.
		 */
		if (!priv->last_msg &&
		    !priv->msg_buf &&
		    twi_int_status & MCOMP &&
		    mstat & DNAK) {
			priv->result = 1;
		} else {
			if (mstat & LOSTARB)
				dev_dbg(priv->dev, "lost arbitration");
			if (mstat & ANAK)
				dev_dbg(priv->dev, "address not acknowledged");
			if (mstat & DNAK)
				dev_dbg(priv->dev, "data not acknowledged");
			if (mstat & BUFRDERR)
				dev_dbg(priv->dev, "buffer read error");
			if (mstat & BUFWRERR)
				dev_dbg(priv->dev, "buffer write error");

			priv->result = -EIO;
		}

		/* Faulty slave devices, may drive SDA low after a transfer
		 * finishes. To release the bus this code generates up to 9
		 * extra clocks until SDA is released.
		 */
		if (ioread16(priv->base + ADI_TWI_MSTRSTAT) & SDASEN) {
			int cnt = 9;

			do {
				iowrite16(SCLOVR, priv->base + ADI_TWI_MSTRCTL);
				udelay(6);
				iowrite16(0, priv->base + ADI_TWI_MSTRCTL);
				udelay(6);
			} while ((ioread16(priv->base + ADI_TWI_MSTRSTAT) & SDASEN) && cnt--);
			iowrite16(SDAOVR | SCLOVR, priv->base + ADI_TWI_MSTRCTL);
			udelay(6);
			iowrite16(SDAOVR, priv->base + ADI_TWI_MSTRCTL);
			udelay(6);
			iowrite16(0, priv->base + ADI_TWI_MSTRCTL);
		}

		if (!polling)
			complete(&priv->msg_complete);
	} else if (twi_int_status & MCOMP) {
		priv->result = 1;
		if (!polling)
			complete(&priv->msg_complete);
	}
}

static irqreturn_t i2c_adi_twi_handle_all_interrupts(struct adi_twi_dev *priv,
						     bool polling)
{
	unsigned short twi_int_status;

	while (1) {
		twi_int_status = ioread16(priv->base + ADI_TWI_ISTAT);
		if (!twi_int_status)
			return IRQ_HANDLED;
		/* Clear interrupt status */
		iowrite16(twi_int_status, priv->base + ADI_TWI_ISTAT);
		i2c_adi_twi_handle_interrupt(priv, twi_int_status, polling);
	}
}

static irqreturn_t i2c_adi_twi_interrupt_entry(int irq, void *dev_id)
{
	struct adi_twi_dev *priv = dev_id;
	unsigned long flags;
	irqreturn_t handled;

	spin_lock_irqsave(&priv->lock, flags);
	handled = i2c_adi_twi_handle_all_interrupts(priv, false);
	spin_unlock_irqrestore(&priv->lock, flags);
	return handled;
}

static int i2c_adi_twi_check_bus_status(struct adi_twi_dev *priv)
{
	u32 busy_timeout = ADI_TWI_BUSY_TIMEOUT;
	u16 val;

	val = ioread16(priv->base + ADI_TWI_MSTRSTAT);
	if (val & BUSBUSY) {
		i2c_adi_twi_master_init(priv);

		while (busy_timeout) {
			val = ioread16(priv->base + ADI_TWI_MSTRSTAT);
			if (!(val & BUSBUSY))
				break;

			busy_timeout--;
			usleep_range(1000, 1100);
		}
	}

	return busy_timeout ? 0 : -EIO;
}

static int i2c_adi_twi_xfer_msg(struct adi_twi_dev *priv,
				struct i2c_msg *msg, bool last_msg, bool polling)
{
	u16 val;

	if (msg->flags & I2C_M_TEN) {
		dev_err(priv->dev, "10 bits addr not supported!");
		return -EINVAL;
	}

	priv->msg_buf = msg->buf;
	priv->msg_buf_remaining = msg->len;
	priv->result = 0;

	if (!polling)
		init_completion(&priv->msg_complete);

	/* Set Transmit device address */
	iowrite16(msg->addr, priv->base + ADI_TWI_MSTRADDR);

	/* FIFO Initiation. Data in FIFO should be
	 * discarded before start a new operation.
	 */
	iowrite16(0x3, priv->base + ADI_TWI_FIFOCTL);
	iowrite16(0, priv->base + ADI_TWI_FIFOCTL);

	if (!(msg->flags & I2C_M_RD)) {
		/* Transmit first data */
		if (priv->msg_buf_remaining) {
			iowrite16(*priv->msg_buf++, priv->base + ADI_TWI_TXDATA8);
			priv->msg_buf_remaining--;
		}
	}

	if (msg->len < 255) {
		val = FIELD_PREP(DCNT, msg->len);
		priv->manual_stop = false;
	} else {
		val = FIELD_PREP(DCNT, 0xff);
		priv->manual_stop = true;
	}
	priv->last_msg = last_msg;

	/* Initiate data transfer */
	val |= MEN | (last_msg ? 0 : RSTART) |
	       ((msg->flags & I2C_M_RD) ? MDIR : 0) |
	       ((priv->twi_clk > 100) ? FAST : 0);
	iowrite16(val, priv->base + ADI_TWI_MSTRCTL);

	if (polling) { /* Polling mode */
		unsigned long timeout = jiffies + priv->adap.timeout;

		while (!priv->result) {
			i2c_adi_twi_handle_all_interrupts(priv, true);

			if (time_is_before_jiffies(timeout)) {
				priv->result = -1;
				dev_err(priv->dev, "transfer timeout occurred (polling mode)");
				break;
			}
			udelay(1);
		}
	} else { /* Interrupt mode */
		while (!priv->result) {
			if (!wait_for_completion_timeout(&priv->msg_complete, priv->adap.timeout)) {
				priv->result = -1;
				dev_err(priv->dev, "transfer timeout occurred (irq mode)");
			}
		}
	}

	return priv->result;
}

static int i2c_adi_twi_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			    int num, bool polling)
{
	struct adi_twi_dev *priv = i2c_get_adapdata(adap);
	int i, ret;

	if (!(ioread16(priv->base + ADI_TWI_CTL) & TWI_ENA))
		return -ENXIO;

	ret = i2c_adi_twi_check_bus_status(priv);
	if (ret)
		return ret;

	/* Clear interrupt stat */
	iowrite16(MCOMP | MERR | XMTSERV | RCVSERV, priv->base + ADI_TWI_ISTAT);
	/* Set interrupt mask. Enable XMT, RCV interrupt source */
	if (!polling)
		iowrite16(MCOMP | MERR | XMTSERV | RCVSERV, priv->base + ADI_TWI_IMSK);

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD && msgs[i].len == 0)
			continue;
		ret = i2c_adi_twi_xfer_msg(priv, &msgs[i], i == num - 1, polling);
		if (ret < 0)
			return ret;
	}

	i2c_adi_twi_master_init(priv);

	return num;
}

static int i2c_adi_twi_master_xfer(struct i2c_adapter *adap,
				   struct i2c_msg *msgs, int num)
{
	return i2c_adi_twi_xfer(adap, msgs, num, false);
}

static int i2c_adi_twi_master_xfer_atomic(struct i2c_adapter *adap,
					  struct i2c_msg *msgs, int num)
{
	return i2c_adi_twi_xfer(adap, msgs, num, true);
}

static u32 i2c_adi_twi_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm adi_twi_algorithm = {
	.master_xfer		= i2c_adi_twi_master_xfer,
	.master_xfer_atomic	= i2c_adi_twi_master_xfer_atomic,
	.functionality		= i2c_adi_twi_functionality,
};

#ifdef CONFIG_PM_SLEEP
static int i2c_adi_twi_suspend(struct device *dev)
{
	struct adi_twi_dev *priv = dev_get_drvdata(dev);

	priv->saved_clkdiv = ioread16(priv->base + ADI_TWI_CLKDIV);
	priv->saved_control = ioread16(priv->base + ADI_TWI_CTL);

	free_irq(priv->irq, priv);

	/* Disable TWI */
	iowrite16(priv->saved_control & ~TWI_ENA, priv->base + ADI_TWI_CTL);

	return 0;
}

static int i2c_adi_twi_resume(struct device *dev)
{
	struct adi_twi_dev *priv = dev_get_drvdata(dev);

	int ret = request_irq(priv->irq, i2c_adi_twi_interrupt_entry,
			      0, to_platform_device(dev)->name, priv);

	if (ret) {
		dev_err(dev, "cannot claim irq: %d", priv->irq);
		return -ENODEV;
	}

	/* Resume TWI interface clock as specified */
	iowrite16(priv->saved_clkdiv, priv->base + ADI_TWI_CLKDIV);

	/* Resume TWI */
	iowrite16(priv->saved_control, priv->base + ADI_TWI_CTL);

	return 0;
}

static SIMPLE_DEV_PM_OPS(i2c_adi_twi_pm,
			 i2c_adi_twi_suspend, i2c_adi_twi_resume);
#define I2C_ADI_TWI_PM_OPS      (&i2c_adi_twi_pm)
#else
#define I2C_ADI_TWI_PM_OPS      NULL
#endif

#ifdef CONFIG_OF
static const struct of_device_id adi_twi_of_match[] = {
	{
		.compatible = "adi,twi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_twi_of_match);
#endif

static int i2c_adi_twi_probe(struct platform_device *pdev)
{
	struct adi_twi_dev *priv;
	struct i2c_adapter *adap;
	struct resource *res;
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	unsigned int clkhilow;
	int ret;
	u16 val;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;

	spin_lock_init(&priv->lock);

	match = of_match_device(of_match_ptr(adi_twi_of_match), &pdev->dev);
	if (match) {
		if (of_property_read_u32(node, "clock-khz", &priv->twi_clk))
			priv->twi_clk = 50;
	} else {
		priv->twi_clk = CONFIG_I2C_ADI_TWI_CLK_KHZ;
	}

	priv->sclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->sclk)) {
		if (PTR_ERR(priv->sclk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "missing i2c clock");
		return PTR_ERR(priv->sclk);
	}

	/* Find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resources");
		return -ENODEV;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		dev_err(&pdev->dev, "cannot map io");
		return PTR_ERR(priv->base);
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return -ENOENT;

	adap = &priv->adap;
	adap->nr = pdev->id;
	strscpy(adap->name, pdev->name, sizeof(adap->name));
	adap->algo = &adi_twi_algorithm;
	adap->algo_data = priv;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = node;
	adap->timeout = HZ;
	i2c_set_adapdata(adap, priv);

	ret = devm_request_irq(&pdev->dev, priv->irq, i2c_adi_twi_interrupt_entry,
			       0, pdev->name, priv);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim irq: %d", priv->irq);
		ret = -ENODEV;
		goto out_error;
	}

	/* Set TWI internal clock as 10MHz */
	clk_prepare_enable(priv->sclk);
	if (ret) {
		dev_err(&pdev->dev, "cannot enable sclk");
		goto out_error;
	}

	val = ((clk_get_rate(priv->sclk) / 1000 / 1000 + 5) / 10) & 0x7F;
	iowrite16(val, priv->base + ADI_TWI_CTL);

	/*
	 * We will not end up with a CLKDIV=0 because no one will specify
	 * 20kHz SCL or less in Kconfig now. (5 * 1000 / 20 = 250)
	 */
	clkhilow = ((10 * 1000 / priv->twi_clk) + 1) / 2;

	/* Set Twi interface clock as specified */
	val = (clkhilow << 8) | clkhilow;
	iowrite16(val, priv->base + ADI_TWI_CLKDIV);

	/* Enable TWI */
	val = ioread16(priv->base + ADI_TWI_CTL) | TWI_ENA;
	iowrite16(val, priv->base + ADI_TWI_CTL);

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
		goto disable_clk;

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "ADI on-chip I2C TWI Controller probed!");

	return 0;

disable_clk:
	clk_disable_unprepare(priv->sclk);

out_error:
	return ret;
}

static void i2c_adi_twi_remove(struct platform_device *pdev)
{
	struct adi_twi_dev *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->sclk);
	i2c_del_adapter(&priv->adap);
}

static struct platform_driver i2c_adi_twi_driver = {
	.probe			= i2c_adi_twi_probe,
	.remove			= i2c_adi_twi_remove,
	.driver			= {
		.name		= "i2c-adi-twi",
		.pm		= I2C_ADI_TWI_PM_OPS,
		.of_match_table = of_match_ptr(adi_twi_of_match),
	},
};

static int __init i2c_adi_twi_init(void)
{
	return platform_driver_register(&i2c_adi_twi_driver);
}

static void __exit i2c_adi_twi_exit(void)
{
	platform_driver_unregister(&i2c_adi_twi_driver);
}

subsys_initcall(i2c_adi_twi_init);
module_exit(i2c_adi_twi_exit);

MODULE_AUTHOR("Bryan Wu, Sonic Zhang");
MODULE_AUTHOR("Slawomir Kulig <slawomir.kulig@analog.com>");
MODULE_DESCRIPTION("ADI on-chip I2C TWI Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-adi-twi");
