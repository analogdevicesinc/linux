// SPDX-License-Identifier: GPL-2.0-only
/*
 * flexio i2c master driver
 *
 * Copyright 2021 NXP
 *
 * Author: Alice Guo <alice.guo@nxp.com>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/imx-flexio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/kernel.h>

#define TRANSMIT_STAT	0x1
#define RECEIVE_STAT	0x2
#define XFER_TIMEOUT	10000

enum transfer_state {
	START_ADDRESS_WRITE,
	WRITE_DATA,
	WRITE_END,
	START_ADDRESS_READ,
	READ_NOT_LAST_DATA,
	READ_LAST_DATA,
	READ_END,
};

enum stop_position {
	NOT_LAST_8_BIT,
	LAST_8_BIT,
};

struct imx_flexio_i2c_master_dev {
	void __iomem *base;

	struct device *dev;
	struct clk *clk;
	struct i2c_adapter adapter;
	struct completion complete;
	enum transfer_state state;
	enum stop_position stop;

	spinlock_t lock;

	unsigned int irq;
	unsigned int baudrate;
	unsigned int src_clock;
	u8 shifters[2];
	u8 timers[2];
	u8 sda_pin;
	u8 scl_pin;
	u8 slave_addr;
	u8 *data;
	u16 len;

	bool need_check_ack;
	bool nack;
	bool read;
	bool repeated_start;
};

enum shifter_flags {
	TX_EMPTY_F = BIT(0),
	RX_FULL_F = BIT(1),
	TX_ERR_F = BIT(2),
	RX_NAK_F = BIT(3),
};

static void i2c_master_enable_ack(struct imx_flexio_i2c_master_dev *i2c_dev,
				  bool enable)
{
	void __iomem *base = i2c_dev->base;
	u32 cfg;

	cfg = flexio_readl(base, SHIFTCFG_0);
	cfg &= ~SHIFTCFG_SSTOP_MASK;
	if (enable)
		cfg |= SHIFTCFG_SSTOP(SSTOP_BIT_LOW);
	else
		cfg |= SHIFTCFG_SSTOP(SSTOP_BIT_HIGH);

	flexio_writel(base, cfg, SHIFTCFG_0);
}

static irqreturn_t imx_flexio_i2c_isr(int irq, void *dev_id)
{
	struct imx_flexio_i2c_master_dev *i2c_dev = dev_id;
	void __iomem *base = i2c_dev->base;
	void __iomem *pinstat = i2c_dev->base + PINSTAT;
	u32 shiftstat, ack, val, ctl;

	shiftstat = flexio_readl(base, SHIFTSTAT);

	if (shiftstat & TRANSMIT_STAT) {
		switch (i2c_dev->state) {
		case START_ADDRESS_WRITE:
			flexio_writel(base, i2c_dev->slave_addr, SHIFTBUFBBS_0);

			if (i2c_dev->len > 0)
				i2c_dev->state = WRITE_DATA;
			else
				i2c_dev->state = WRITE_END;
			break;
		case WRITE_DATA:
			flexio_writel(base, *i2c_dev->data, SHIFTBUFBBS_0);
			i2c_dev->data++;
			i2c_dev->len--;

			if (!i2c_dev->len)
				i2c_dev->state = WRITE_END;
			break;
		case WRITE_END:
			if (i2c_dev->repeated_start)
				flexio_writel(base, 0xff, SHIFTBUFBBS_0);

			i2c_dev->stop = LAST_8_BIT;
			break;
		case START_ADDRESS_READ:
			flexio_writel(base, i2c_dev->slave_addr, SHIFTBUFBBS_0);

			if (i2c_dev->len > 1)
				i2c_dev->state = READ_NOT_LAST_DATA;
			else if (i2c_dev->len == 1)
				i2c_dev->state = READ_LAST_DATA;
			else if (i2c_dev->len == 0)
				i2c_dev->state = READ_END;
			break;
		case READ_NOT_LAST_DATA:
			flexio_writel(base, 0xff, SHIFTBUFBBS_0);
			break;
		case READ_LAST_DATA:
			i2c_master_enable_ack(i2c_dev, false);
			flexio_writel(base, 0xff, SHIFTBUFBBS_0);
			break;
		case READ_END:
			i2c_dev->stop = LAST_8_BIT;
			break;
		default:
			break;
		}
	}

	if (shiftstat & RECEIVE_STAT) {
		if (i2c_dev->need_check_ack) {
			ack = flexio_readl(base, SHIFTERR) & 2;
			if (!ack) {
				if (i2c_dev->stop == LAST_8_BIT) {
					flexio_writel(base, 0, SHIFTSIEN);
					complete(&i2c_dev->complete);
				}

				if (i2c_dev->read) {
					i2c_dev->need_check_ack = false;
					i2c_dev->read = false;
					flexio_readl(base, SHIFTBUFBIS_1);

					switch (i2c_dev->state) {
					case READ_NOT_LAST_DATA:
						i2c_master_enable_ack(i2c_dev, true);
						break;
					case READ_LAST_DATA:
						i2c_master_enable_ack(i2c_dev, false);
						i2c_dev->state = READ_END;
						break;
					case READ_END:
					default:
						flexio_writel(base, 0, SHIFTSIEN);
						complete(&i2c_dev->complete);
						break;
					}
				}
			} else {
				flexio_writel(base, 0, SHIFTSIEN);
				i2c_dev->nack = true;

				switch (i2c_dev->stop) {
				case NOT_LAST_8_BIT:
					goto stop;
				case LAST_8_BIT:
					flexio_writel(base, 0, SHIFTBUFBBS_0);
					complete(&i2c_dev->complete);
					break;
				default:
					complete(&i2c_dev->complete);
					break;
				}
			}

			flexio_readl(base, SHIFTBUFBIS_1);
		} else {
			*i2c_dev->data = flexio_readl(base, SHIFTBUFBIS_1);
			i2c_dev->data++;
			i2c_dev->len--;

			if (i2c_dev->len == 1)
				i2c_dev->state = READ_LAST_DATA;
			else if (i2c_dev->len == 0) {
				flexio_writel(base, 0, SHIFTBUFBBS_0);

				flexio_writel(base, 0, SHIFTSIEN);
				complete(&i2c_dev->complete);
			}
		}
	}

	return IRQ_HANDLED;

stop:
	/* generate STOP */
	flexio_writel(base, 0, SHIFTBUFBBS_0);

	/*
	 * Software should then wait for the next rising edge on SCL and then
	 * disable both timers.
	 */
	flexio_writel(base, 1 << i2c_dev->scl_pin, PINREN);
	readl_relaxed_poll_timeout(pinstat, val, val & (1 << i2c_dev->scl_pin), 0, 1000);

	ctl = flexio_readl(base, TIMCTL_0);
	ctl &= ~TIMCTL_TIMOD_MASK;
	ctl |= TIMCTL_TIMOD(TIMER_DISABLE);
	flexio_writel(base, ctl, TIMCTL_0);

	ctl = flexio_readl(base, TIMCTL_1);
	ctl &= ~TIMCTL_TIMOD_MASK;
	ctl |= TIMCTL_TIMOD(TIMER_DISABLE);
	flexio_writel(base, ctl, TIMCTL_1);

	/*
	 * The transmit shifter should then be disabled after waiting the setup
	 * delay for a repeated START or STOP condition.
	 */
	udelay(10);
	ctl = flexio_readl(base, SHIFTCTL_0);
	ctl &= ~SHIFTCTL_PINCFG_MASK;
	ctl |= SHIFTCTL_PINCFG(SHIFTER_PIN_OUTPUT_DISABLE);
	ctl &= ~SHIFTCTL_SMOD_MASK;
	ctl |= SHIFTCTL_SMOD(SHIFTER_DISABLE);
	flexio_writel(base, ctl, SHIFTCTL_0);

	flexio_writel(base, 0, PINREN);
	flexio_writel(base, 1 << i2c_dev->scl_pin, PINSTAT);

	complete(&i2c_dev->complete);

	return IRQ_HANDLED;
}

static void clear_shifter_flags(struct imx_flexio_i2c_master_dev *i2c_dev,
				enum shifter_flags flags)
{
	void __iomem *base = i2c_dev->base;
	u32 v = 1;
	u8 tx = i2c_dev->shifters[0], rx = i2c_dev->shifters[1];

	/*
	 * For SMOD = Transmit, the status flag is set when SHIFTBUF is empty or
	 * when initially configured for SMOD=Transmit.
	 * For SMOD = Receive, the status flag is set when SHIFTBUF is full, and
	 * the shifter error flag is set when the received start or stop bit
	 * does not match the expected value.
	 */
	if (flags & TX_EMPTY_F)
		flexio_writel(base, v << tx, SHIFTSTAT);
	if (flags & RX_FULL_F)
		flexio_writel(base, v << rx, SHIFTSTAT);
	if (flags & TX_ERR_F)
		flexio_writel(base, v << tx, SHIFTERR);
	if (flags & RX_NAK_F)
		flexio_writel(base, v << rx, SHIFTERR);
}

static void imx_flexio_init_hardware(struct imx_flexio_i2c_master_dev *i2c_dev)
{
	void __iomem *base = i2c_dev->base;
	struct flexio_control ctrl;
	struct flexio_shifter_control shiftctl;
	struct flexio_shifter_config shiftcfg;
	struct flexio_timer_control timerctl;
	struct flexio_timer_config timercfg;
	u32 timercmp;

	flexio_sw_reset(base, CTRL);

	/* configure the shifter 0 as transmitter */
	shiftcfg.insrc	= INPUT_SRC_PIN;
	shiftcfg.sstop	= SSTOP_BIT_HIGH;
	shiftcfg.sstart = SSTART_BIT_LOW;
	flexio_setup_shiftcfg(base, &shiftcfg, SHIFTCFG_0);

	shiftctl.timsel = i2c_dev->timers[1];
	shiftctl.timpol = SHIFT_ON_POSEDGE;
	shiftctl.pincfg = SHIFTER_PIN_OPEN_DRAIN_OUTPUT;
	shiftctl.pinsel = i2c_dev->sda_pin;
	shiftctl.pinpol = PIN_ACTIVE_LOW;
	shiftctl.smod	= SHIFTER_TRANSMIT;
	flexio_setup_shiftctl(base, &shiftctl, SHIFTCTL_0);

	/* configure the shifter 1 as receiver */
	shiftcfg.insrc	= INPUT_SRC_PIN;
	shiftcfg.sstop	= SSTOP_BIT_LOW;
	shiftcfg.sstart = SSTART_BIT_DISABLE;
	flexio_setup_shiftcfg(base, &shiftcfg, SHIFTCFG_1);

	shiftctl.timsel = i2c_dev->timers[1];
	shiftctl.timpol = SHIFT_ON_NEGEDGE;
	shiftctl.pincfg = SHIFTER_PIN_OUTPUT_DISABLE;
	shiftctl.pinsel = i2c_dev->sda_pin;
	shiftctl.pinpol = PIN_ACTIVE_HIGH;
	shiftctl.smod	= SHIFTER_RECEIVE;
	flexio_setup_shiftctl(base, &shiftctl, SHIFTCTL_1);

	/* configure the timer 0 for the SCL and to trigger the timer 1 */
	timercfg.timout	= TIMOUT_ZERO_NOTAFFECT_BY_RESET;
	timercfg.timdec	= TIMDEC_FLEXIO_CLK;
	timercfg.timrst	= TIMRST_TIMPIN_EQUAL_TIMOUTPUT;
	timercfg.timdis	= TIMDIS_TIMER_COMPARE;
	timercfg.timena	= TIMENA_TRG_HIGH;
	timercfg.tstop	= TSTOP_BIT_ENABLE_TIMDIS;
	timercfg.tstart	= TSTART_BIT_ENABLE;
	flexio_setup_timercfg(base, &timercfg, TIMCFG_0);

	/* the baud rate divider equal to (CMP[7:0] + 1) * 2 */
	timercmp = (u32)(i2c_dev->src_clock / i2c_dev->baudrate) / 2 - 1;
	flexio_writel(base, timercmp, TIMCMP_0);

	timerctl.trgsel = TIMER_TRGSEL_SHIFTER(i2c_dev->shifters[0]);
	timerctl.trgpol = TIMER_TRG_ACTIVE_LOW;
	timerctl.trgsrc = TIMER_TRGSRC_INTER;
	timerctl.pincfg = TIMPIN_OPEN_DRAIN_OUTPUT;
	timerctl.pinsel = i2c_dev->scl_pin;
	timerctl.pinpol = TIMPIN_ACTIVE_HIGH;
	timerctl.timod	= DUAL_8BIT_COUNTERS_BAUD;
	flexio_setup_timerctl(base, &timerctl, TIMCTL_0);

	/* configure the timer 1 to control shifters */
	timercfg.timout = TIMOUT_ONE_NOTAFFECT_BY_RESET;
	timercfg.timdec = TIMDEC_PIN_INPUT;
	timercfg.timrst = TIMRST_NEVER;
	timercfg.timdis = TIMDIS_TIMER_DISABLE;
	timercfg.timena = TIMENA_PREV_TIMENA;
	timercfg.tstop  = TSTOP_BIT_ENABLE_TIMCMP;
	timercfg.tstart = TSTART_BIT_ENABLE;
	flexio_setup_timercfg(base, &timercfg, TIMCFG_1);

	/* the number of bits in each word equal to (CMP[15:0] + 1) / 2 */
	timercmp = 8 * 2 - 1;
	flexio_writel(i2c_dev->base, timercmp, TIMCMP_1);

	timerctl.trgsel = TIMER_TRGSEL_SHIFTER(i2c_dev->shifters[0]);
	timerctl.trgpol = TIMER_TRG_ACTIVE_LOW;
	timerctl.trgsrc = TIMER_TRGSRC_INTER;
	timerctl.pincfg = TIMPIN_OUTPUT_DISABLE;
	timerctl.pinsel = i2c_dev->scl_pin;
	timerctl.pinpol = TIMPIN_ACTIVE_LOW;
	timerctl.timod	= SINGLE_16BIT_COUNTER;
	flexio_setup_timerctl(base, &timerctl, TIMCTL_1);

	/* disable the shifter status interrupt and shifter error interrupt */
	flexio_writel(base, 0, SHIFTSIEN);
	flexio_writel(base, 0, SHIFTEIEN);

	flexio_get_default_ctrl(&ctrl);
	flexio_setup_ctrl(base, &ctrl, CTRL);
}

static int i2c_bus_busy(struct imx_flexio_i2c_master_dev *i2c_dev)
{
	unsigned int i, mask;
	void __iomem *base = i2c_dev->base;

	/*
	 * If in certain loops the SDA/SCL is continuously pulled down, then
	 * return bus busy status.
	 */
	for (i = 0; i < 100; i++) {
		mask = 1 << i2c_dev->sda_pin | 1 << i2c_dev->scl_pin;

		if ((flexio_readl(base, PIN) & mask) == mask)
			return 0;
	}

	return -EBUSY;
}

static int setup_xfer_count(struct imx_flexio_i2c_master_dev *i2c_dev,
			    u32 xfer_len)
{
	void __iomem *base = i2c_dev->base;
	u32 cmp, cfg;

	if (xfer_len > ((0xff - 1) / (16 + 1 + 1))) {
		dev_err(i2c_dev->dev, "more than 14 bytes to be transferred\n");
		return -EINVAL;
	}

	/* configure the number of shift clock edges in the transfer */
	cmp = flexio_readl(base, TIMCMP_0);
	cmp &= 0x00ff;
	cmp |= (xfer_len * 18 + 1) << 8;
	flexio_writel(base, cmp, TIMCMP_0);

	cfg = flexio_readl(base, TIMCFG_0);
	cfg &= ~TIMCFG_TIMDIS_MASK;
	cfg |= TIMCFG_TIMDIS(TIMDIS_TIMER_COMPARE);
	flexio_writel(base, cfg, TIMCFG_0);

	return 0;
}

static int i2c_master_write(struct imx_flexio_i2c_master_dev *i2c_dev,
			    struct i2c_msg *msg)
{
	void __iomem *base = i2c_dev->base;
	void __iomem *timstat = base + TIMSTAT;
	int timeout, val;

	i2c_dev->state = START_ADDRESS_WRITE;
	if (msg->len)
		i2c_dev->stop = NOT_LAST_8_BIT;
	else
		i2c_dev->stop = LAST_8_BIT;
	i2c_dev->need_check_ack = true;

	i2c_dev->slave_addr = i2c_8bit_addr_from_msg(msg);
	i2c_dev->data = msg->buf;
	i2c_dev->len = msg->len;

	flexio_writel(base, 3, SHIFTSIEN);

	timeout = wait_for_completion_timeout(&i2c_dev->complete, XFER_TIMEOUT);
	if (timeout) {
		if (i2c_dev->nack)
			return -EIO;
	} else {
		return -EIO;
	}
	reinit_completion(&i2c_dev->complete);

	i2c_dev->nack = false;
	flexio_writel(base, 0, SHIFTSIEN);

	readl_relaxed_poll_timeout(timstat, val, val & 1, 0, 1000);
	flexio_writel(base, 1, TIMSTAT);

	return 0;
}

static int i2c_master_read(struct imx_flexio_i2c_master_dev *i2c_dev,
			   struct i2c_msg *msg)
{
	void __iomem *base = i2c_dev->base;
	void __iomem *timstat = base + TIMSTAT;
	int timeout, val;

	clear_shifter_flags(i2c_dev, RX_FULL_F);

	i2c_dev->state = START_ADDRESS_READ;
	if (msg->len)
		i2c_dev->stop = NOT_LAST_8_BIT;
	else
		i2c_dev->stop = LAST_8_BIT;
	i2c_dev->need_check_ack = true;
	i2c_dev->read = true;

	i2c_dev->slave_addr = i2c_8bit_addr_from_msg(msg);
	i2c_dev->data = msg->buf;
	i2c_dev->len = msg->len;

	flexio_writel(base, 3, SHIFTSIEN);

	timeout = wait_for_completion_timeout(&i2c_dev->complete, XFER_TIMEOUT);
	if (timeout) {
		if (i2c_dev->nack)
			return -EIO;
	} else {
		return -EIO;
	}
	reinit_completion(&i2c_dev->complete);

	i2c_dev->nack = false;
	flexio_writel(base, 0, SHIFTSIEN);

	readl_relaxed_poll_timeout(timstat, val, val & 1, 0, 1000);
	flexio_writel(base, 1, TIMSTAT);

	return 0;
}

static int xfer_msg(struct imx_flexio_i2c_master_dev *i2c_dev, struct i2c_msg *msg)
{
	void __iomem *base = i2c_dev->base;
	void __iomem *shiftstat = base + SHIFTSTAT;
	u32 xfer_len, val;
	bool msg_read = !!(msg->flags & I2C_M_RD);
	int err;

	xfer_len = msg->len + 1;

	/*
	 * Sets the number of bytes to be transferred from a start signal to a
	 * stop signal. Timer 0 is used to generate SCL output and to trigger
	 * timer 1.
	 */
	if (setup_xfer_count(i2c_dev, xfer_len))
		return -EINVAL;

	err = readl_relaxed_poll_timeout(shiftstat, val, val & 1, 0, 1000);
	if (err) {
		dev_err(i2c_dev->dev, "wait transmit SHIFTBUF empty timeout\n");
		return err;
	}

	if (!msg_read)
		err = i2c_master_write(i2c_dev, msg);
	else
		err = i2c_master_read(i2c_dev, msg);
	if (err)
		return err;

	return 0;
}

static int imx_flexio_i2c_master_xfer(struct i2c_adapter *adap,
				      struct i2c_msg msgs[], int num)
{
	struct imx_flexio_i2c_master_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	u32 ctl;
	int i, err;

	if (i2c_dev->nack) {
		ctl = flexio_readl(base, SHIFTCTL_0);
		ctl &= ~SHIFTCTL_PINCFG_MASK;
		ctl |= SHIFTCTL_PINCFG(SHIFTER_PIN_OPEN_DRAIN_OUTPUT);
		ctl &= ~SHIFTCTL_SMOD_MASK;
		ctl |= SHIFTCTL_SMOD(SHIFTER_TRANSMIT);
		flexio_writel(base, ctl, SHIFTCTL_0);

		ctl = flexio_readl(base, TIMCTL_0);
		ctl &= ~TIMCTL_TIMOD_MASK;
		ctl |= TIMCTL_TIMOD(DUAL_8BIT_COUNTERS_BAUD);
		flexio_writel(base, ctl, TIMCTL_0);

		ctl = flexio_readl(base, TIMCTL_1);
		ctl &= ~TIMCTL_TIMOD_MASK;
		ctl |= TIMCTL_TIMOD(SINGLE_16BIT_COUNTER);
		flexio_writel(base, ctl, TIMCTL_1);
	}
	i2c_dev->nack = false;

	i2c_dev->repeated_start = false;
	if (num > 1)
		i2c_dev->repeated_start = true;

	clear_shifter_flags(i2c_dev, RX_FULL_F | RX_NAK_F);

	err = i2c_bus_busy(i2c_dev);
	if (err) {
		dev_dbg(i2c_dev->dev, "SDA/SCL is continuously pulled down\n");

		imx_flexio_init_hardware(i2c_dev);
		clear_shifter_flags(i2c_dev, RX_FULL_F | RX_NAK_F);
	}

	for (i = 0; i < num; i++) {
		err = xfer_msg(i2c_dev, &msgs[i]);
		if (err)
			return err;
	}

	return (err < 0) ? err : num;
}

static int imx_flexio_i2c_master_xfer_atomic(struct i2c_adapter *adap,
					     struct i2c_msg msgs[], int num)
{
	return 0;
}

static u32 imx_flexio_i2c_master_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm imx_flexio_i2c_master_algo = {
	.master_xfer		= imx_flexio_i2c_master_xfer,
	.master_xfer_atomic	= imx_flexio_i2c_master_xfer_atomic,
	.functionality		= imx_flexio_i2c_master_func,
};

static int imx_flexio_i2c_issue_bus_clear(struct i2c_adapter *adap)
{
	return 0;
}

static struct i2c_bus_recovery_info imx_flexio_i2c_recovery_info = {
	.recover_bus = imx_flexio_i2c_issue_bus_clear,
};

static int imx_flexio_i2c_master_probe(struct platform_device *pdev)
{
	struct flexio_ddata *ddata = dev_get_drvdata(pdev->dev.parent);
	struct imx_flexio_i2c_master_dev *i2c_dev;
	int err;

	if (IS_ERR_OR_NULL(ddata))
		return -EINVAL;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2c_dev);

	i2c_dev->base = ddata->base;
	if (IS_ERR(i2c_dev->base))
		return PTR_ERR(i2c_dev->base);

	i2c_dev->dev = &pdev->dev;
	i2c_dev->irq = ddata->irq;

	err = devm_request_irq(i2c_dev->dev, i2c_dev->irq, imx_flexio_i2c_isr,
			       IRQF_NO_SUSPEND, dev_name(i2c_dev->dev),
			       i2c_dev);
	if (err)
		return err;

	i2c_dev->clk = ddata->ipg_clk;
	if (IS_ERR(i2c_dev->clk)) {
		dev_err(&pdev->dev, "missing imx flexio i2c master clock\n");
		return PTR_ERR(i2c_dev->clk);
	}

	err = clk_prepare_enable(i2c_dev->clk);
	if (err) {
		dev_err(i2c_dev->dev, "failed to enable imx flexio i2c master clock: %d\n", err);
		return err;
	}

	/* hardware configuration */
	err = of_property_read_u32(pdev->dev.of_node, "clock-frequency", &i2c_dev->baudrate);
	if (err < 0) {
		dev_err(i2c_dev->dev, "no clock-frequency found\n");
		return err;
	}
	i2c_dev->src_clock	= clk_get_rate(i2c_dev->clk);
	i2c_dev->shifters[0]	= 0;
	i2c_dev->shifters[1]	= 1;
	i2c_dev->timers[0]	= 0;
	i2c_dev->timers[1]	= 1;
	err = of_property_read_u8(pdev->dev.of_node, "sda", &i2c_dev->sda_pin);
	if (err < 0) {
		dev_err(i2c_dev->dev, "no sda property found\n");
		return err;
	}
	err = of_property_read_u8(pdev->dev.of_node, "scl", &i2c_dev->scl_pin);
	if (err < 0) {
		dev_err(i2c_dev->dev, "no scl property found\n");
		return err;
	}
	imx_flexio_init_hardware(i2c_dev);

	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	i2c_dev->adapter.algo = &imx_flexio_i2c_master_algo;
	i2c_dev->adapter.timeout = 600;
	i2c_dev->adapter.retries = 1;
	i2c_dev->adapter.dev.parent = i2c_dev->dev;
	i2c_dev->adapter.dev.of_node = i2c_dev->dev->of_node;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.bus_recovery_info = &imx_flexio_i2c_recovery_info;

	strscpy(i2c_dev->adapter.name, dev_name(i2c_dev->dev),
		sizeof(i2c_dev->adapter.name));

	init_completion(&i2c_dev->complete);
	spin_lock_init(&i2c_dev->lock);

	err = i2c_add_adapter(&i2c_dev->adapter);
	if (err)
		goto release_clock;

	return 0;

release_clock:
	clk_disable(i2c_dev->clk);

	return err;
}

static void imx_flexio_i2c_master_remove(struct platform_device *pdev)
{
	struct imx_flexio_i2c_master_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);
	clk_disable(i2c_dev->clk);
}

static const struct of_device_id imx_flexio_i2c_master_of_match[] = {
	{ .compatible = "nxp,imx-flexio-i2c-master", },
	{},
};
MODULE_DEVICE_TABLE(of, imx_flexio_i2c_master_of_match);

static struct platform_driver imx_flexio_i2c_master_driver = {
	.probe = imx_flexio_i2c_master_probe,
	.remove = imx_flexio_i2c_master_remove,
	.driver = {
		.name = "imx-flexio-i2c-master",
		.of_match_table = imx_flexio_i2c_master_of_match,
	},
};
module_platform_driver(imx_flexio_i2c_master_driver);

MODULE_DESCRIPTION("NXP I.MX FlexIO I2C Master driver");
MODULE_AUTHOR("Alice Guo <alice.guo@nxp.com>");
MODULE_LICENSE("GPL v2");
