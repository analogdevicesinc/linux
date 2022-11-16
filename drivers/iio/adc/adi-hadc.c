// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define ADI_HADC_CTL		0x00
#define ADI_HADC_CHAN_MSK	0x04
#define ADI_HADC_IMSK		0x08
#define ADI_HADC_STAT		0x0c

// There are 16 channels (0-15) supported by hardware
// but for now we only export the 8 that are "built in" and do
// not require the external multiplexer
#define ADI_HADC_DATA(n)	(0x10 + ((n)*4))

#define CTL_ENLS		BIT(13)
#define CTL_DOUTOREOCB	BIT(12)
#define CTL_FIXEDCNV	GENMASK(11, 8)
#define CTL_FIXED_SHIFT	8
#define CTL_CONT		BIT(7)
#define CTL_FDIV		GENMASK(6, 3)
#define CTL_DIV_SHIFT	3
#define CTL_STARTCNV	BIT(2)
#define CTL_PD			BIT(1)
#define CTL_NRST		BIT(0)

#define CHAN_MASK		GENMASK(15, 0)

#define	IMSK_RDY		BIT(17)
#define IMSK_SEQ		BIT(16)
#define IMSK_CHAN		GENMASK(15, 0)

#define STAT_TMU_BUSY	BIT(20)
#define STAT_CHAN_INT	GENMASK(19, 4)
#define STAT_CHAN_SHIFT	4
#define STAT_SEQ		BIT(3)
#define STAT_RDYW1C		BIT(2)
#define STAT_RDY		BIT(0)

#define DATA_MASK		GENMASK(11, 0)

#define ADI_HADC_ALL_CHANNELS_MASKED 0xffff

// in us
#define HADC_CONVERSION_TIMEOUT	100000

#define HADC_CHANNEL(_idx, _name) { \
	.type = IIO_VOLTAGE, \
	.channel = _idx, \
	.indexed = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.datasheet_name = _name, \
	.scan_index = _idx, \
	.scan_type = { \
		.sign = 'u', \
		.shift = 0, \
		.realbits = 12, \
		.storagebits = 16, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec adi_hadc_channels[] = {
	HADC_CHANNEL(0, "chan0"),
	HADC_CHANNEL(1, "chan1"),
	HADC_CHANNEL(2, "chan2"),
	HADC_CHANNEL(3, "chan3"),
	HADC_CHANNEL(4, "chan4"),
	HADC_CHANNEL(5, "chan5"),
	HADC_CHANNEL(6, "chan6"),
	HADC_CHANNEL(7, "chan7"),
};

struct adi_hadc {
	void __iomem *base;
	struct device *dev;
	int irq;
	spinlock_t lock;

	/* 1 = in continuous mode, 0 = in fixed mode */
	int continuous;

	/* One copy of data to help */
	u16 data[ARRAY_SIZE(adi_hadc_channels)];
};

static void hadc_writel(struct adi_hadc *hadc, u32 val, u32 offset)
{
	writel(val, hadc->base + offset);
}

static u32 hadc_readl(struct adi_hadc *hadc, u32 offset)
{
	return readl(hadc->base + offset);
}

/**
 * To set the conversion mode:
 * 1) Put HADC into reset
 * 2) Set or clear the continuous conversion bit (it will do fixed conversions if clear)
 * 3) Take HADC out of reset
 * All steps should be done separately for the sake of it, and a couple of cycles
 * of f_sample clk later it will resume sampling if appropriate
 */
static void hadc_set_conversion_mode(struct adi_hadc *hadc, int cont)
{
	u32 val;

	if (cont == hadc->continuous)
		return;

	val = hadc_readl(hadc, ADI_HADC_CTL);
	val &= ~CTL_NRST;
	hadc_writel(hadc, val, ADI_HADC_CTL);

	if (cont)
		val |= CTL_CONT | CTL_STARTCNV;
	else
		val &= ~CTL_CONT & ~CTL_STARTCNV;

	hadc_writel(hadc, val, ADI_HADC_CTL);

	val |= CTL_NRST;
	hadc_writel(hadc, val, ADI_HADC_CTL);

	hadc->continuous = cont;
}

static int adi_hadc_update_scan_mode(struct iio_dev *idev,
	const unsigned long *mask)
{
	struct adi_hadc *hadc = iio_priv(idev);
	int bit;
	int ch = ADI_HADC_ALL_CHANNELS_MASKED;

	// Unmask each channel in the scan mask
	for_each_set_bit(bit, mask, idev->masklength) {
		if (bit <= ARRAY_SIZE(adi_hadc_channels))
			ch &= ~BIT(bit);
	}

	hadc_writel(hadc, ch, ADI_HADC_CHAN_MSK);
	return 0;
}

static int adi_hadc_read_raw(struct iio_dev *idev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct adi_hadc *hadc = iio_priv(idev);
	int ch = BIT(chan->channel);
	u32 original_mask;
	u32 regval;

	// Make sure the channel we want is unmasked
	original_mask = hadc_readl(hadc, ADI_HADC_CHAN_MSK);
	regval = original_mask & ~ch;
	hadc_writel(hadc, regval, ADI_HADC_CHAN_MSK);

	// Clear completion bit for that channel
	regval = ch << STAT_CHAN_SHIFT;
	hadc_writel(hadc, regval, ADI_HADC_STAT);

	// If we're in fixed conversion mode, need to wait for ready and then
	// signal a start
	if (!hadc->continuous) {
		if (readl_poll_timeout(hadc->base + ADI_HADC_STAT, regval,
			regval & STAT_RDY, 0, HADC_CONVERSION_TIMEOUT)) {
			dev_err(hadc->dev, "timeout waiting for HADC to be ready\n");
			return -ETIMEDOUT;
		}

		regval = hadc_readl(hadc, ADI_HADC_CTL);
		regval |= CTL_STARTCNV;
		hadc_writel(hadc, regval, ADI_HADC_CTL);
	}

	// Wait until channel is complete
	if (readl_poll_timeout(hadc->base + ADI_HADC_STAT, regval,
		regval & (ch << STAT_CHAN_SHIFT), 0, HADC_CONVERSION_TIMEOUT)) {
		dev_err(hadc->dev, "timeout waiting for conversion to finish\n");
		return -ETIMEDOUT;
	}

	*val = hadc_readl(hadc, ADI_HADC_DATA(chan->channel)) & DATA_MASK;

	// Restore channel scan mask
	hadc_writel(hadc, original_mask, ADI_HADC_CHAN_MSK);

	return IIO_VAL_INT;
}

static int adi_hadc_xlate(struct iio_dev *idev, const struct of_phandle_args *spec)
{
	if (spec->args[0] < ARRAY_SIZE(adi_hadc_channels))
		return spec->args[0];
	return -EINVAL;
}

static int adi_hadc_buffered_start(struct iio_dev *idev)
{
	struct adi_hadc *hadc = iio_priv(idev);

	enable_irq(hadc->irq);
	hadc_set_conversion_mode(hadc, 1);
	return 0;
}

static int adi_hadc_buffered_stop(struct iio_dev *idev)
{
	struct adi_hadc *hadc = iio_priv(idev);

	hadc_set_conversion_mode(hadc, 0);
	disable_irq(hadc->irq);
	return 0;
}

static const struct iio_buffer_setup_ops adi_hadc_setup_ops = {
	.preenable = &adi_hadc_buffered_start,
	.predisable = &adi_hadc_buffered_stop,
};

static const struct iio_info adi_hadc_iio_info = {
	.read_raw = &adi_hadc_read_raw,
	.of_xlate = &adi_hadc_xlate,
	.update_scan_mode = adi_hadc_update_scan_mode,
};

static irqreturn_t adi_hadc_handler(int irq, void *id)
{
	struct iio_dev *idev = id;
	struct adi_hadc *hadc = iio_priv(idev);
	int bit;
	size_t i;
	u32 stat;

	spin_lock(&hadc->lock);

	i = 0;
	for_each_set_bit(bit, idev->active_scan_mask, idev->masklength) {
		if (bit <= ARRAY_SIZE(adi_hadc_channels))
			hadc->data[i++] = hadc_readl(hadc, ADI_HADC_DATA(bit)) & DATA_MASK;
	}

	iio_push_to_buffers(idev, hadc->data);

	spin_unlock(&hadc->lock);
	return IRQ_HANDLED;
}

static int adi_hadc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct iio_dev *indio_dev;
	struct adi_hadc *hadc;
	struct resource *res;
	struct iio_buffer *buffer;
	void __iomem *base;
	int irq = 0;
	u32 val;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*hadc));
	if (!indio_dev)
		return -ENOMEM;

	hadc = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Missing base address\n");
		return -ENODEV;
	}

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		dev_err(dev, "Cannot remap base address\n");
		return PTR_ERR(base);
	}

	irq = of_irq_get(node, 0);
	if (irq <= 0) {
		dev_err(dev, "Missing IRQ\n");
		return irq ? irq : -ENOENT;
	}

	ret = devm_request_threaded_irq(dev, irq, adi_hadc_handler,
		NULL, 0, "adi-hadc", indio_dev);
	if (ret) {
		dev_err(dev, "Could not create IRQ handler\n");
		return ret;
	}

	hadc->base = base;
	hadc->dev = dev;
	hadc->irq = irq;
	spin_lock_init(&hadc->lock);

	// IRQs start disabled until we have a buffer enabled and switch to
	// continuous conversion mode
	disable_irq(irq);
	hadc->continuous = 0;

	indio_dev->dev.parent = dev;
	indio_dev->dev.of_node = node;
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->info = &adi_hadc_iio_info;
	indio_dev->channels = adi_hadc_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_hadc_channels);
	indio_dev->setup_ops = &adi_hadc_setup_ops;

	// Ensure we're in reset mode to make changes
	val = hadc_readl(hadc, ADI_HADC_CTL);
	val &= ~CTL_NRST;
	hadc_writel(hadc, val, ADI_HADC_CTL);

	// Set a single non-continuous conversion with a 125 MHz/32 =  clock
	val &= ~CTL_CONT & ~CTL_FDIV & ~CTL_FIXEDCNV;
	val |= (15 << CTL_DIV_SHIFT) & CTL_FDIV;
	val |= (1 << CTL_FIXED_SHIFT) & CTL_FIXEDCNV;
	hadc_writel(hadc, val, ADI_HADC_CTL);

	// Mask all channels
	val = ADI_HADC_ALL_CHANNELS_MASKED;
	hadc_writel(hadc, val, ADI_HADC_CHAN_MSK);

	// Only unmask the sequence completion interrupt
	val = IMSK_CHAN | IMSK_RDY;
	hadc_writel(hadc, val, ADI_HADC_IMSK);

	// Clear PD, RST, and enable level shifters
	val = hadc_readl(hadc, ADI_HADC_CTL);
	val &= ~CTL_PD;
	val |= CTL_NRST | CTL_ENLS;
	hadc_writel(hadc, val, ADI_HADC_CTL);

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  INDIO_BUFFER_SOFTWARE,
					  &adi_hadc_setup_ops);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id adi_hadc_match_table[] = {
	{ .compatible = "adi,hadc" },
	{ }
};

MODULE_DEVICE_TABLE(of, adi_hadc_match_table);

static struct platform_driver adi_hadc_driver = {
	.driver = {
		.name = "adi-hadc",
		.of_match_table = adi_hadc_match_table,
	},
	.probe = adi_hadc_probe,
};
module_platform_driver(adi_hadc_driver);

MODULE_DESCRIPTION("Analog Devices Housekeeping ADC driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
