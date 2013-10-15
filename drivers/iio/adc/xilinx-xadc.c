/*
 * Xilinx XADC driver
 *
 * Copyright 2013 Analog Devices Inc.
 *  Author: Lars-Peter Clauen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "xilinx-xadc.h"

/* PS7 register definitions */
#define XADCPS7_REG_CFG		0x00
#define XADCPS7_REG_INTSTS	0x04
#define XADCPS7_REG_INTMSK	0x08
#define XADCPS7_REG_STATUS	0x0c
#define XADCPS7_REG_CFIFO	0x10
#define XADCPS7_REG_DFIFO	0x14
#define XADCPS7_REG_CTL		0x18

#define XADCPS7_CFG_ENABLE		BIT(31)
#define XADCPS7_CFG_CFIFOTH_MASK	(0xf << 20)
#define XADCPS7_CFG_CFIFOTH_OFFSET	20
#define XADCPS7_CFG_DFIFOTH_MASK	(0xf << 16)
#define XADCPS7_CFG_DFIFOTH_OFFSET	16
#define XADCPS7_CFG_WEDGE		BIT(13)
#define XADCPS7_CFG_REDGE		BIT(12)
#define XADCPS7_CFG_TCKRATE_MASK	(0x3 << 8)
#define XADCPS7_CFG_TCKRATE_DIV2	(0x0 << 8)
#define XADCPS7_CFG_TCKRATE_DIV4	(0x1 << 8)
#define XADCPS7_CFG_TCKRATE_DIV8	(0x2 << 8)
#define XADCPS7_CFG_TCKRATE_DIV16	(0x3 << 8)
#define XADCPS7_CFG_IGAP_MASK		0x1f
#define XADCPS7_CFG_IGAP(x)		(x)

#define XADCPS7_INT_CFIFO_LTH		BIT(9)
#define XADCPS7_INT_DFIFO_GTH		BIT(8)
#define XADCPS7_INT_ALARM_MASK		0xff
#define XADCPS7_INT_ALARM_OFFSET	0

#define XADCPS7_STATUS_CFIFO_LVL_MASK	(0xf << 16)
#define XADCPS7_STATUS_CFIFO_LVL_OFFSET	16
#define XADCPS7_STATUS_DFIFO_LVL_MASK	(0xf << 12)
#define XADCPS7_STATUS_DFIFO_LVL_OFFSET	12
#define XADCPS7_STATUS_CFIFOF		BIT(11)
#define XADCPS7_STATUS_CFIFOE		BIT(10)
#define XADCPS7_STATUS_DFIFOF		BIT(9)
#define XADCPS7_STATUS_DFIFOE		BIT(8)
#define XADCPS7_STATUS_OT		BIT(7)
#define XADCPS7_STATUS_ALM(x)		BIT(x)

#define XADCPS7_CTL_RESET		BIT(4)

#define XADCPS7_CMD_NOP			0x00
#define XADCPS7_CMD_READ		0x01
#define XADCPS7_CMD_WRITE		0x02

#define XADCPS7_CMD(cmd, addr, data) (((cmd) << 26) | ((addr) << 16) | (data))

/* AXI register definitions */
#define XADC_AXI_REG_RESET		0x00
#define XADC_AXI_REG_STATUS		0x04
#define XADC_AXI_REG_ALARM_STATUS	0x08
#define XADC_AXI_REG_CONVST		0x0c
#define XADC_AXI_REG_XADC_RESET		0x10
#define XADC_AXI_REG_GIER		0x5c
#define XADC_AXI_REG_IPISR		0x60
#define XADC_AXI_REG_IPIER		0x68
#define XADC_AXI_ADC_REG_OFFSET		0x200

#define XADC_AXI_RESET_MAGIC		0xa
#define XADC_AXI_GIER_ENABLE		BIT(31)

#define XADC_AXI_INT_EOS		BIT(4)
#define XADC_AXI_INT_ALARM_MASK		0x3c0f

#define XADC_FLAGS_BUFFERED BIT(0)

static void xadc_ps7_write_reg(struct xadc *xadc, unsigned int reg,
	uint32_t val)
{
	writel(val, xadc->base + reg);
}

static void xadc_ps7_read_reg(struct xadc *xadc, unsigned int reg,
	uint32_t *val)
{
	*val = readl(xadc->base + reg);
}

static void xadc_ps7_write_fifo(struct xadc *xadc, uint32_t *cmd,
	unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		xadc_ps7_write_reg(xadc, XADCPS7_REG_CFIFO, cmd[i]);
}

static int xadc_ps7_write_adc_reg(struct xadc *xadc, unsigned int reg,
	uint16_t val)
{
	uint32_t cmd[1];
	uint32_t tmp;
	uint32_t mask;
	int ret;

	INIT_COMPLETION(xadc->completion);

	cmd[0] = XADCPS7_CMD(XADCPS7_CMD_WRITE, reg, val);
	xadc_ps7_write_fifo(xadc, cmd, ARRAY_SIZE(cmd));
	xadc_ps7_read_reg(xadc, XADCPS7_REG_CFG, &tmp);
	tmp &= ~XADCPS7_CFG_DFIFOTH_MASK;
	tmp |= 0 << XADCPS7_CFG_DFIFOTH_OFFSET;
	xadc_ps7_write_reg(xadc, XADCPS7_REG_CFG, tmp);

	spin_lock_irq(&xadc->lock);
	xadc_ps7_read_reg(xadc, XADCPS7_REG_INTMSK, &mask);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTMSK,
			mask & ~XADCPS7_INT_DFIFO_GTH);
	spin_unlock_irq(&xadc->lock);

	ret = wait_for_completion_interruptible_timeout(&xadc->completion, HZ);
	if (ret == 0)
		ret = -EIO;
	else
		ret = 0;

	xadc_ps7_read_reg(xadc, XADCPS7_REG_DFIFO, &tmp);

	return ret;
}

static int xadc_ps7_read_adc_reg(struct xadc *xadc, unsigned int reg,
	uint16_t *val)
{
	uint32_t cmd[2];
	uint32_t resp, tmp, mask;
	int ret;

	cmd[0] = XADCPS7_CMD(XADCPS7_CMD_READ, reg, 0);
	cmd[1] = XADCPS7_CMD(XADCPS7_CMD_NOP, 0, 0);

	INIT_COMPLETION(xadc->completion);

	xadc_ps7_write_fifo(xadc, cmd, ARRAY_SIZE(cmd));
	xadc_ps7_read_reg(xadc, XADCPS7_REG_CFG, &tmp);
	tmp &= ~XADCPS7_CFG_DFIFOTH_MASK;
	tmp |= 1 << XADCPS7_CFG_DFIFOTH_OFFSET;
	xadc_ps7_write_reg(xadc, XADCPS7_REG_CFG, tmp);

	spin_lock_irq(&xadc->lock);
	xadc_ps7_read_reg(xadc, XADCPS7_REG_INTMSK, &mask);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTMSK,
			mask & ~XADCPS7_INT_DFIFO_GTH);
	spin_unlock_irq(&xadc->lock);

	ret = wait_for_completion_interruptible_timeout(&xadc->completion, HZ);
	if (ret == 0)
		ret = -EIO;
	if (ret < 0)
		return ret;

	xadc_ps7_read_reg(xadc, XADCPS7_REG_DFIFO, &resp);
	xadc_ps7_read_reg(xadc, XADCPS7_REG_DFIFO, &resp);

	*val = resp & 0xffff;

	return 0;
}

static irqreturn_t xadc_ps7_threaded_interrupt_handler(int irq, void *devid)
{
	struct iio_dev *indio_dev = devid;
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned int alarm, status;

	spin_lock_irq(&xadc->lock);
	status = xadc->ps7_alarm;
	xadc->ps7_alarm = 0;
	spin_unlock_irq(&xadc->lock);

	alarm = ((status & 0x80) >> 4);
	alarm |= ((status & 0x78) << 1);
	alarm |= (status & 0x07);
	xadc_handle_events(indio_dev, alarm);

	return IRQ_HANDLED;
}

static irqreturn_t xadc_ps7_interrupt_handler(int irq, void *devid)
{
	struct iio_dev *indio_dev = devid;
	struct xadc *xadc = iio_priv(indio_dev);
	uint32_t status, mask;

	xadc_ps7_read_reg(xadc, XADCPS7_REG_INTSTS, &status);
	xadc_ps7_read_reg(xadc, XADCPS7_REG_INTMSK, &mask);

	status &= ~mask;

	if (!status)
		return IRQ_NONE;

	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTSTS, status);

	if (status & XADCPS7_INT_DFIFO_GTH) {
		xadc_ps7_write_reg(xadc, XADCPS7_REG_INTMSK, mask |
				XADCPS7_INT_DFIFO_GTH);
		complete(&xadc->completion);
	}

	if (status & XADCPS7_INT_ALARM_MASK) {
		spin_lock(&xadc->lock);
		xadc->ps7_alarm |= status & XADCPS7_INT_ALARM_MASK;
		spin_unlock(&xadc->lock);
	    return IRQ_WAKE_THREAD;
	}

	return IRQ_HANDLED;
}

#define XADCPS7_TCK_RATE_MAX 50000000
#define XADCPS7_IGAP_DEFAULT 20

static void xadc_ps7_parse_dt(struct xadc *xadc, struct device_node *np,
	unsigned int *tck_div, u32 *igap)
{
	u32 tck_rate = XADCPS7_TCK_RATE_MAX;
	unsigned long pcap_rate;
	unsigned int div;

	*igap = XADCPS7_IGAP_DEFAULT;

	/* We'll use the default if the property is not present */
	of_property_read_u32(np, "xlnx,igap", igap);
	of_property_read_u32(np, "xlnx,tck-rate", &tck_rate);

	pcap_rate = clk_get_rate(xadc->clk);

	if (tck_rate > XADCPS7_TCK_RATE_MAX)
		tck_rate = XADCPS7_TCK_RATE_MAX;
	if (tck_rate > pcap_rate / 2) {
		div = 2;
	} else {
		div = pcap_rate / tck_rate;
		if (pcap_rate / div > XADCPS7_TCK_RATE_MAX)
			div++;
	}

	if (div <= 3)
		*tck_div = XADCPS7_CFG_TCKRATE_DIV2;
	else if (div <= 7)
		*tck_div = XADCPS7_CFG_TCKRATE_DIV4;
	else if (div <= 15)
		*tck_div = XADCPS7_CFG_TCKRATE_DIV8;
	else
		*tck_div = XADCPS7_CFG_TCKRATE_DIV16;
}

static int xadc_ps7_setup(struct platform_device *pdev,
	struct iio_dev *indio_dev, int irq)
{
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned int tck_div;
	u32 igap;

	xadc_ps7_parse_dt(xadc, pdev->dev.of_node, &tck_div, &igap);

	xadc_ps7_write_reg(xadc, XADCPS7_REG_CTL, XADCPS7_CTL_RESET);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_CTL, 0);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTSTS, ~0);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTMSK, ~0);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_CFG, XADCPS7_CFG_ENABLE |
			XADCPS7_CFG_REDGE | XADCPS7_CFG_WEDGE |
			tck_div | XADCPS7_CFG_IGAP(igap));

	return 0;
}

static unsigned long xadc_ps7_get_dclk_rate(struct xadc *xadc)
{
	unsigned int div;
	uint32_t val;

	xadc_ps7_read_reg(xadc, XADCPS7_REG_CFG, &val);

	switch (val & XADCPS7_CFG_TCKRATE_MASK) {
	case XADCPS7_CFG_TCKRATE_DIV4:
		div = 4;
		break;
	case XADCPS7_CFG_TCKRATE_DIV8:
		div = 8;
		break;
	case XADCPS7_CFG_TCKRATE_DIV16:
		div = 16;
		break;
	default:
		div = 2;
		break;
	}

	return clk_get_rate(xadc->clk) / div;
}

static void xadc_ps7_update_alarm(struct xadc *xadc, unsigned int alarm)
{
	unsigned long flags;
	uint32_t val;

	/* Move OT to bit 7 */
	alarm = ((alarm & 0x08) << 4) | ((alarm & 0xf0) >> 1) | (alarm & 0x07);

	spin_lock_irqsave(&xadc->lock, flags);
	xadc_ps7_read_reg(xadc, XADCPS7_REG_INTMSK, &val);
	val |= XADCPS7_INT_ALARM_MASK;
	val &= ~(alarm << XADCPS7_INT_ALARM_OFFSET);
	xadc_ps7_write_reg(xadc, XADCPS7_REG_INTMSK, val);
	spin_unlock_irqrestore(&xadc->lock, flags);
}

static const struct xadc_ops xadc_ps7_ops = {
	.read = xadc_ps7_read_adc_reg,
	.write = xadc_ps7_write_adc_reg,
	.setup = xadc_ps7_setup,
	.get_dclk_rate = xadc_ps7_get_dclk_rate,
	.interrupt_handler = xadc_ps7_interrupt_handler,
	.threaded_interrupt_handler = xadc_ps7_threaded_interrupt_handler,
	.update_alarm = xadc_ps7_update_alarm,
};

static void xadc_axi_read_reg(struct xadc *xadc, unsigned int reg,
	uint32_t *val)
{
	*val = readl(xadc->base + reg);
}

static void xadc_axi_write_reg(struct xadc *xadc, unsigned int reg,
	uint32_t val)
{
	writel(val, xadc->base + reg);
}

static int xadc_axi_read_adc_reg(struct xadc *xadc, unsigned int reg,
	uint16_t *val)
{
	uint32_t val32;

	xadc_axi_read_reg(xadc, XADC_AXI_ADC_REG_OFFSET + reg * 4, &val32);
	*val = val32 & 0xffff;

	return 0;
}

static int xadc_axi_write_adc_reg(struct xadc *xadc, unsigned int reg,
	uint16_t val)
{
	xadc_axi_write_reg(xadc, XADC_AXI_ADC_REG_OFFSET + reg * 4, val);

	return 0;
}

static int xadc_axi_setup(struct platform_device *pdev,
	struct iio_dev *indio_dev, int irq)
{
	struct xadc *xadc = iio_priv(indio_dev);

	xadc_axi_write_reg(xadc, XADC_AXI_REG_RESET, XADC_AXI_RESET_MAGIC);
	xadc_axi_write_reg(xadc, XADC_AXI_REG_GIER, XADC_AXI_GIER_ENABLE);

	return 0;
}

static irqreturn_t xadc_axi_interrupt_handler(int irq, void *devid)
{
	struct iio_dev *indio_dev = devid;
	struct xadc *xadc = iio_priv(indio_dev);
	uint32_t status, mask;
	unsigned int events;

	xadc_axi_read_reg(xadc, XADC_AXI_REG_IPISR, &status);
	xadc_axi_read_reg(xadc, XADC_AXI_REG_IPIER, &mask);
	status &= mask;

	if (!status)
		return IRQ_NONE;

	if ((status & XADC_AXI_INT_EOS) && xadc->trigger)
		iio_trigger_poll(xadc->trigger, 0);

	if (status & XADC_AXI_INT_ALARM_MASK) {
		events = (status & 0x000e) >> 1;
		events |= (status & 0x0001) << 3;
		events |= (status & 0x3c00) >> 6;
		xadc_handle_events(indio_dev, events);
	}

	xadc_axi_write_reg(xadc, XADC_AXI_REG_IPISR, status);

	return IRQ_HANDLED;
}

static void xadc_axi_update_alarm(struct xadc *xadc, unsigned int alarm)
{
	uint32_t val;
	unsigned long flags;

	alarm = ((alarm & 0x07) << 1) | ((alarm & 0x08) >> 3) |
			((alarm & 0xf0) << 6);

	spin_lock_irqsave(&xadc->lock, flags);
	xadc_axi_read_reg(xadc, XADC_AXI_REG_IPIER, &val);
	val &= ~XADC_AXI_INT_ALARM_MASK;
	val |= alarm;
	xadc_axi_write_reg(xadc, XADC_AXI_REG_IPIER, val);
	spin_unlock_irqrestore(&xadc->lock, flags);
}

static unsigned long xadc_axi_get_dclk(struct xadc *xadc)
{
	return clk_get_rate(xadc->clk);
}

static const struct xadc_ops xadc_axi_ops = {
	.read = xadc_axi_read_adc_reg,
	.write = xadc_axi_write_adc_reg,
	.setup = xadc_axi_setup,
	.get_dclk_rate = xadc_axi_get_dclk,
	.update_alarm = xadc_axi_update_alarm,
	.interrupt_handler = xadc_axi_interrupt_handler,
	.flags = XADC_FLAGS_BUFFERED,
};

static int xadc_update_reg(struct xadc *xadc, unsigned int reg, uint16_t mask,
	uint16_t val)
{
	uint16_t tmp;
	int ret;

	mutex_lock(&xadc->mutex);

	ret = _xadc_read_reg(xadc, reg, &tmp);
	if (ret)
		goto err_unlock;

	ret = _xadc_write_reg(xadc, reg, (tmp & ~mask) | val);
err_unlock:
	mutex_unlock(&xadc->mutex);

	return ret;
}

static unsigned long xadc_get_dclk_rate(struct xadc *xadc)
{
	return xadc->ops->get_dclk_rate(xadc);
}

static int xadc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *mask)
{
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned int n;

	n = bitmap_weight(mask, indio_dev->masklength);

	kfree(xadc->data);
	xadc->data = kcalloc(n, sizeof(*xadc->data), GFP_KERNEL);
	if (!xadc->data)
		return -ENOMEM;

	return 0;
}

static unsigned int xadc_scan_index_to_channel(unsigned int scan_index)
{
	switch (scan_index) {
	case 5:
		return XADC_REG_VCCPINT;
	case 6:
		return XADC_REG_VCCPAUX;
	case 7:
		return XADC_REG_VCCO_DDR;
	case 8:
		return XADC_REG_TEMP;
	case 9:
		return XADC_REG_VCCINT;
	case 10:
		return XADC_REG_VCCAUX;
	case 11:
		return XADC_REG_VPVN;
	case 12:
		return XADC_REG_VREFP;
	case 13:
		return XADC_REG_VREFN;
	case 14:
		return XADC_REG_VCCBRAM;
	default:
		return XADC_REG_VAUX(scan_index - 16);
	}
}

static irqreturn_t xadc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned int chan;
	int i, j;

	if (!xadc->data)
		goto out;

	j = 0;
	for_each_set_bit(i, indio_dev->active_scan_mask,
		indio_dev->masklength) {
		chan = xadc_scan_index_to_channel(i);
		_xadc_read_reg(xadc, chan, &xadc->data[j]);
		j++;
	}

	iio_push_to_buffers(indio_dev, (uint8_t *)xadc->data);

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int xadc_trigger_set_state(struct iio_trigger *trigger, bool state)
{
	struct xadc *xadc = iio_trigger_get_drvdata(trigger);
	unsigned long flags;
	unsigned int convst;
	unsigned int val;
	int ret = 0;

	mutex_lock(&xadc->mutex);

	if (state) {
		/* Only one of the two triggers can be active at the a time. */
		if (xadc->trigger != NULL) {
			ret = -EBUSY;
			goto err_out;
		} else {
			xadc->trigger = trigger;
			if (trigger == xadc->convst_trigger)
				convst = XADC_CONF0_EC;
			else
				convst = 0;
		}
		ret = xadc_update_reg(xadc, XADC_REG_CONF1, XADC_CONF0_EC,
					convst);
		if (ret)
			goto err_out;
	} else {
		xadc->trigger = NULL;
	}

	spin_lock_irqsave(&xadc->lock, flags);
	xadc_axi_read_reg(xadc, XADC_AXI_REG_IPIER, &val);
	xadc_axi_write_reg(xadc, XADC_AXI_REG_IPISR, val & XADC_AXI_INT_EOS);
	if (state)
		val |= XADC_AXI_INT_EOS;
	else
		val &= ~XADC_AXI_INT_EOS;
	xadc_axi_write_reg(xadc, XADC_AXI_REG_IPIER, val);
	spin_unlock_irqrestore(&xadc->lock, flags);

err_out:
	mutex_unlock(&xadc->mutex);

	return ret;
}

static const struct iio_trigger_ops xadc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &xadc_trigger_set_state,
};

static struct iio_trigger *xadc_alloc_trigger(struct iio_dev *indio_dev,
	const char *name)
{
	struct iio_trigger *trig;
	int ret;

	trig = iio_trigger_alloc("%s-%s%d", indio_dev->name, name,
				indio_dev->id);
	if (trig == NULL)
		return ERR_PTR(-ENOMEM);

	trig->dev.parent = indio_dev->dev.parent;
	trig->ops = &xadc_trigger_ops;
	iio_trigger_set_drvdata(trig, iio_priv(indio_dev));

	ret = iio_trigger_register(trig);
	if (ret)
		goto error_free_trig;

	return trig;

error_free_trig:
	iio_trigger_free(trig);
	return ERR_PTR(ret);
}

static int xadc_power_adc_b(struct xadc *xadc, unsigned int seq_mode)
{
	uint16_t val;

	switch (seq_mode) {
	case XADC_CONF1_SEQ_SIMULTANEOUS:
	case XADC_CONF1_SEQ_INDEPENDENT:
		val = XADC_CONF2_PD_ADC_B;
		break;
	default:
		val = 0;
		break;
	}

	return xadc_update_reg(xadc, XADC_REG_CONF2, XADC_CONF2_PD_MASK, val);
}

static int xadc_get_seq_mode(struct xadc *xadc, unsigned long scan_mode)
{
	unsigned int aux_scan_mode = scan_mode >> 16;

	if (xadc->external_mux_mode == XADC_EXTERNAL_MUX_DUAL)
		return XADC_CONF1_SEQ_SIMULTANEOUS;

	if ((aux_scan_mode & 0xff00) == 0 ||
		(aux_scan_mode & 0x00ff) == 0)
		return XADC_CONF1_SEQ_CONTINUOUS;

	return XADC_CONF1_SEQ_SIMULTANEOUS;
}

static int xadc_postdisable(struct iio_dev *indio_dev)
{
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned long scan_mask;
	int ret;
	int i;

	scan_mask = 1; /* Run calibration as part of the sequence */
	for (i = 0; i < indio_dev->num_channels; i++)
		scan_mask |= BIT(indio_dev->channels[i].scan_index);

	/* Enable all channels and calibration */
	ret = xadc_write_reg(xadc, XADC_REG_SEQ(0), scan_mask & 0xffff);
	if (ret)
		return ret;

	ret = xadc_write_reg(xadc, XADC_REG_SEQ(1), scan_mask >> 16);
	if (ret)
		return ret;

	ret = xadc_update_reg(xadc, XADC_REG_CONF1, XADC_CONF1_SEQ_MASK,
		XADC_CONF1_SEQ_CONTINUOUS);
	if (ret)
		return ret;

	return xadc_power_adc_b(xadc, XADC_CONF1_SEQ_CONTINUOUS);
}

static int xadc_preenable(struct iio_dev *indio_dev)
{
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned long scan_mask;
	int seq_mode;
	int ret;

	ret = iio_sw_buffer_preenable(indio_dev);
	if (ret)
		goto err;

	ret = xadc_update_reg(xadc, XADC_REG_CONF1, XADC_CONF1_SEQ_MASK,
		XADC_CONF1_SEQ_DEFAULT);
	if (ret)
		goto err;

	scan_mask = *indio_dev->active_scan_mask;
	seq_mode = xadc_get_seq_mode(xadc, scan_mask);

	ret = xadc_write_reg(xadc, XADC_REG_SEQ(0), scan_mask & 0xffff);
	if (ret)
		goto err;

	ret = xadc_write_reg(xadc, XADC_REG_SEQ(1), scan_mask >> 16);
	if (ret)
		goto err;

	ret = xadc_power_adc_b(xadc, seq_mode);
	if (ret)
		goto err;

	ret = xadc_update_reg(xadc, XADC_REG_CONF1, XADC_CONF1_SEQ_MASK,
		seq_mode);
	if (ret)
		goto err;

	return 0;
err:
	xadc_postdisable(indio_dev);
	return ret;
}

static struct iio_buffer_setup_ops xadc_buffer_ops = {
	.preenable = &xadc_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &xadc_postdisable,
};

static int xadc_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct xadc *xadc = iio_priv(indio_dev);
	uint16_t val16;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (iio_buffer_enabled(indio_dev))
			return -EBUSY;
		ret = xadc_read_reg(xadc, chan->address, &val16);
		if (ret < 0)
			return ret;

		val16 >>= 4;
		if (chan->scan_type.sign == 'u')
			*val = val16;
		else
			*val = sign_extend32(val16, 11);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			/* V = (val * 3.0) / 4096 */
			switch (chan->address) {
			case XADC_REG_VCCINT:
			case XADC_REG_VCCAUX:
			case XADC_REG_VCCBRAM:
			case XADC_REG_VCCPINT:
			case XADC_REG_VCCPAUX:
			case XADC_REG_VCCO_DDR:
				*val = 3000;
				break;
			default:
				*val = 1000;
				break;
			}
			*val2 = 12;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_TEMP:
			/* Temp in C = (val * 503.975) / 4096 - 273.15 */
			*val = 503975;
			*val2 = 12;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		/* Only the temperature channel has an offset */
		*val = -((273150 << 12) / 503975);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static ssize_t xadc_read_frequency(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned long clk_rate = xadc_get_dclk_rate(xadc);
	uint16_t val;
	unsigned int div;
	int ret;

	ret = xadc_read_reg(xadc, XADC_REG_CONF2, &val);
	if (ret)
		return ret;

	div = (val & XADC_CONF2_DIV_MASK) >> XADC_CONF2_DIV_OFFSET;
	if (div < 2)
		div = 2;

	return sprintf(buf, "%lu\n", clk_rate / div / 26);
}

static ssize_t xadc_write_frequency(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned long clk_rate = xadc_get_dclk_rate(xadc);
	unsigned long lval;
	unsigned int div;
	int ret;

	ret = kstrtoul(buf, 10, &lval);
	if (ret)
		return ret;

	/* Max. 150 kSPS */
	if (lval > 150000)
		lval = 150000;

	lval *= 26;

	/* Min 1MHz */
	if (lval < 1000000)
		lval = 1000000;

	/*
	 * We want to round down, but only if we do not exceed the 150 kSPS
	 * limit.
	 */
	div = clk_rate / lval;
	if (clk_rate / div / 26 > 150000)
		div++;
	if (div < 2)
		div = 2;
	else if (div > 0xff)
		div = 0xff;

	ret = xadc_update_reg(xadc, XADC_REG_CONF2, XADC_CONF2_DIV_MASK,
		div << XADC_CONF2_DIV_OFFSET);
	if (ret)
		return ret;

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
	xadc_read_frequency, xadc_write_frequency);

static struct attribute *xadc_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	NULL
};

static const struct attribute_group xadc_attribute_group = {
	.attrs = xadc_attributes,
};

#define XADC_CHAN_TEMP(_chan, _scan_index, _addr) { \
	.type = IIO_TEMP, \
	.indexed = 1, \
	.channel = (_chan), \
	.address = (_addr), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
		BIT(IIO_CHAN_INFO_SCALE) | \
		BIT(IIO_CHAN_INFO_OFFSET), \
	.event_mask = IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING), \
	.scan_index = (_scan_index), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 4, \
		.endianness = IIO_CPU, \
	}, \
}

#define XADC_CHAN_VOLTAGE(_chan, _scan_index, _addr, _ext, _alarm) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan), \
	.address = (_addr), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
		BIT(IIO_CHAN_INFO_SCALE), \
	.event_mask = !(_alarm) ? 0 : \
		(IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING) | \
		IIO_EV_BIT(IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING)), \
	.scan_index = (_scan_index), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 4, \
		.endianness = IIO_CPU, \
	}, \
	.extend_name = _ext, \
}

static const struct iio_chan_spec xadc_channels[] = {
	XADC_CHAN_TEMP(0, 8, XADC_REG_TEMP),
	XADC_CHAN_VOLTAGE(0, 9, XADC_REG_VCCINT, "vccint", true),
	XADC_CHAN_VOLTAGE(1, 10, XADC_REG_VCCINT, "vccaux", true),
	XADC_CHAN_VOLTAGE(2, 14, XADC_REG_VCCBRAM, "vccbram", true),
	XADC_CHAN_VOLTAGE(3, 5, XADC_REG_VCCPINT, "vccpint", true),
	XADC_CHAN_VOLTAGE(4, 6, XADC_REG_VCCPAUX, "vccpaux", true),
	XADC_CHAN_VOLTAGE(5, 7, XADC_REG_VCCO_DDR, "vcco_ddr", true),
	XADC_CHAN_VOLTAGE(6, 12, XADC_REG_VREFP, "vrefp", false),
	XADC_CHAN_VOLTAGE(7, 13, XADC_REG_VREFN, "vrefn", false),
	XADC_CHAN_VOLTAGE(8, 11, XADC_REG_VPVN, NULL, false),
	XADC_CHAN_VOLTAGE(9, 16, XADC_REG_VAUX(0), NULL, false),
	XADC_CHAN_VOLTAGE(10, 17, XADC_REG_VAUX(1), NULL, false),
	XADC_CHAN_VOLTAGE(11, 18, XADC_REG_VAUX(2), NULL, false),
	XADC_CHAN_VOLTAGE(12, 19, XADC_REG_VAUX(3), NULL, false),
	XADC_CHAN_VOLTAGE(13, 20, XADC_REG_VAUX(4), NULL, false),
	XADC_CHAN_VOLTAGE(14, 21, XADC_REG_VAUX(5), NULL, false),
	XADC_CHAN_VOLTAGE(15, 22, XADC_REG_VAUX(6), NULL, false),
	XADC_CHAN_VOLTAGE(16, 23, XADC_REG_VAUX(7), NULL, false),
	XADC_CHAN_VOLTAGE(17, 24, XADC_REG_VAUX(8), NULL, false),
	XADC_CHAN_VOLTAGE(18, 25, XADC_REG_VAUX(9), NULL, false),
	XADC_CHAN_VOLTAGE(19, 26, XADC_REG_VAUX(10), NULL, false),
	XADC_CHAN_VOLTAGE(20, 27, XADC_REG_VAUX(11), NULL, false),
	XADC_CHAN_VOLTAGE(21, 28, XADC_REG_VAUX(12), NULL, false),
	XADC_CHAN_VOLTAGE(22, 29, XADC_REG_VAUX(13), NULL, false),
	XADC_CHAN_VOLTAGE(23, 30, XADC_REG_VAUX(14), NULL, false),
	XADC_CHAN_VOLTAGE(24, 31, XADC_REG_VAUX(15), NULL, false),
};

static const struct iio_info xadc_info = {
	.read_raw = &xadc_read_raw,
	.read_event_config = &xadc_read_event_config,
	.write_event_config = &xadc_write_event_config,
	.read_event_value = &xadc_read_event_value,
	.write_event_value = &xadc_write_event_value,
	.update_scan_mode = &xadc_update_scan_mode,
	.attrs = &xadc_attribute_group,
	.driver_module = THIS_MODULE,
};

static const struct of_device_id xadc_of_match_table[] = {
	{ .compatible = "xlnx,ps7-xadc-1.00.a", (void *)&xadc_ps7_ops },
	{ .compatible = "xlnx,axi-xadc-1.00.a", (void *)&xadc_axi_ops },
	{ },
};
MODULE_DEVICE_TABLE(of, xadc_of_match_table);

static int xadc_parse_dt(struct iio_dev *indio_dev, struct device_node *np,
	unsigned int *conf)
{
	struct xadc *xadc = iio_priv(indio_dev);
	struct iio_chan_spec *channels, *chan;
	struct device_node *chan_node, *child;
	unsigned int num_channels;
	const char *external_mux;
	u32 ext_mux_chan;
	int reg;
	int ret;

	*conf = 0;

	ret = of_property_read_string(np, "xlnx,external-mux", &external_mux);
	if (ret < 0 || strcasecmp(external_mux, "none") == 0)
		xadc->external_mux_mode = XADC_EXTERNAL_MUX_NONE;
	else if (strcasecmp(external_mux, "single") == 0)
		xadc->external_mux_mode = XADC_EXTERNAL_MUX_SINGLE;
	else if (strcasecmp(external_mux, "dual") == 0)
		xadc->external_mux_mode = XADC_EXTERNAL_MUX_DUAL;
	else
		return -EINVAL;

	if (xadc->external_mux_mode != XADC_EXTERNAL_MUX_NONE) {
		ret = of_property_read_u32(np, "xlnx,external-mux-channel",
					&ext_mux_chan);
		if (ret < 0)
			return ret;

		if (xadc->external_mux_mode == XADC_EXTERNAL_MUX_SINGLE) {
			if (ext_mux_chan == 0)
				ext_mux_chan = XADC_REG_VPVN;
			else if (ext_mux_chan <= 16)
				ext_mux_chan = XADC_REG_VAUX(ext_mux_chan - 1);
			else
				return -EINVAL;
		} else {
			if (ext_mux_chan > 0 && ext_mux_chan <= 8)
				ext_mux_chan = XADC_REG_VAUX(ext_mux_chan - 1);
			else
				return -EINVAL;
		}

		*conf |= XADC_CONF0_MUX | XADC_CONF0_CHAN(ext_mux_chan);
	}

	if (of_property_read_bool(np, "xlnx,extended-acquisition-time"))
		*conf |= XADC_CONF0_ACQ;

	channels = kmemdup(xadc_channels, sizeof(xadc_channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	num_channels = 9;
	chan = &channels[9];

	chan_node = of_find_node_by_name(np, "xlnx,channels");
	if (chan_node) {
		for_each_child_of_node(chan_node, child) {
			if (num_channels >= ARRAY_SIZE(xadc_channels))
				break;

			ret = of_property_read_u32(child, "reg", &reg);
			if (ret || reg > 16)
				continue;

			if (of_property_read_bool(child, "xlnx,bipolar"))
				chan->scan_type.sign = 's';

			if (reg == 0) {
				chan->scan_index = 11;
				chan->address = XADC_REG_VPVN;
			} else {
				chan->scan_index = 15 + reg;
				chan->scan_index = XADC_REG_VAUX(reg - 1);
			}
			num_channels++;
			chan++;
		}
	}

	indio_dev->num_channels = num_channels;
	indio_dev->channels = krealloc(channels, sizeof(*channels) *
					num_channels, GFP_KERNEL);
	/* If we can't resize the channels array, just use the original */
	if (!indio_dev->channels)
		indio_dev->channels = channels;

	return 0;
}

static int xadc_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	unsigned int bipolar_mask;
	struct resource *mem;
	unsigned int conf0;
	struct xadc *xadc;
	int ret;
	int irq;
	int i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	id = of_match_node(xadc_of_match_table, pdev->dev.of_node);
	if (!id)
		return -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENXIO;

	indio_dev = iio_device_alloc(sizeof(*xadc));
	if (!indio_dev)
		return -ENOMEM;

	xadc = iio_priv(indio_dev);
	xadc->ops = id->data;
	init_completion(&xadc->completion);
	mutex_init(&xadc->mutex);
	spin_lock_init(&xadc->lock);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	xadc->base = devm_request_and_ioremap(&pdev->dev, mem);
	if (!xadc->base) {
		ret = -ENXIO;
		goto err_device_free;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->name = "xadc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &xadc_info;

	ret = xadc_parse_dt(indio_dev, pdev->dev.of_node, &conf0);
	if (ret)
		goto err_device_free;

	if (xadc->ops->flags & XADC_FLAGS_BUFFERED) {
		ret = iio_triggered_buffer_setup(indio_dev,
			&iio_pollfunc_store_time, &xadc_trigger_handler,
			&xadc_buffer_ops, IIO_BUFFER_DIRECTION_IN);
		if (ret)
			goto err_device_free;

		xadc->convst_trigger = xadc_alloc_trigger(indio_dev, "convst");
		if (IS_ERR(xadc->convst_trigger))
			goto err_triggered_buffer_cleanup;
		xadc->samplerate_trigger = xadc_alloc_trigger(indio_dev,
			"samplerate");
		if (IS_ERR(xadc->samplerate_trigger))
			goto err_free_convst_trigger;
	}

	xadc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(xadc->clk)) {
		ret = PTR_ERR(xadc->clk);
		goto err_free_samplerate_trigger;
	}
	clk_prepare_enable(xadc->clk);

	ret = xadc->ops->setup(pdev, indio_dev, irq);
	if (ret)
		goto err_free_samplerate_trigger;

	ret = request_threaded_irq(irq, xadc->ops->interrupt_handler,
				xadc->ops->threaded_interrupt_handler,
				0, dev_name(&pdev->dev), indio_dev);
	if (ret)
		goto err_clk_disable_unprepare;

	for (i = 0; i < 16; i++)
		xadc_read_reg(xadc, XADC_REG_THRESHOLD(i), &xadc->threshold[i]);

	ret = xadc_write_reg(xadc, XADC_REG_CONF0, conf0);
	if (ret)
		goto err_free_irq;

	bipolar_mask = 0;
	for (i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].scan_type.sign == 's')
			bipolar_mask |= BIT(indio_dev->channels[i].scan_index);
	}

	ret = xadc_write_reg(xadc, XADC_REG_INPUT_MODE(0), bipolar_mask);
	if (ret)
		goto err_free_irq;
	ret = xadc_write_reg(xadc, XADC_REG_INPUT_MODE(1), bipolar_mask >> 16);
	if (ret)
		goto err_free_irq;

	/* Go to non-buffered mode */
	xadc_postdisable(indio_dev);

	/* Disable all alarms */
	xadc_update_reg(xadc, XADC_REG_CONF1, XADC_CONF1_ALARM_MASK,
		XADC_CONF1_ALARM_MASK);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_free_irq;

	platform_set_drvdata(pdev, indio_dev);

	return 0;

err_free_irq:
	free_irq(irq, indio_dev);
err_free_samplerate_trigger:
	if (xadc->ops->flags & XADC_FLAGS_BUFFERED)
		iio_trigger_free(xadc->samplerate_trigger);
err_free_convst_trigger:
	if (xadc->ops->flags & XADC_FLAGS_BUFFERED)
		iio_trigger_free(xadc->convst_trigger);
err_triggered_buffer_cleanup:
	if (xadc->ops->flags & XADC_FLAGS_BUFFERED)
		iio_triggered_buffer_cleanup(indio_dev);
err_clk_disable_unprepare:
	clk_disable_unprepare(xadc->clk);
err_device_free:
	kfree(indio_dev->channels);
	iio_device_free(indio_dev);

	return ret;
}

static int xadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct xadc *xadc = iio_priv(indio_dev);
	int irq = platform_get_irq(pdev, 0);

	iio_device_unregister(indio_dev);
	if (xadc->ops->flags & XADC_FLAGS_BUFFERED) {
		iio_trigger_free(xadc->convst_trigger);
		iio_trigger_free(xadc->samplerate_trigger);
		iio_triggered_buffer_cleanup(indio_dev);
	}
	free_irq(irq, indio_dev);
	clk_disable_unprepare(xadc->clk);
	kfree(xadc->data);
	kfree(indio_dev->channels);
	iio_device_free(indio_dev);

	return 0;
}

static struct platform_driver xadc_driver = {
	.probe = xadc_probe,
	.remove = xadc_remove,
	.driver = {
		.name = "xadc",
		.owner = THIS_MODULE,
		.of_match_table = xadc_of_match_table,
	},
};
module_platform_driver(xadc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Xilinx XADC IIO driver");
