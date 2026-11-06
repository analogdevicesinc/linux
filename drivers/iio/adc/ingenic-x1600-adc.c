// SPDX-License-Identifier: GPL-2.0
/*
 * Ingenic X1600 ADC driver
 *
 * Copyright (c) 2026 Olivier Louvignes <olivier@mgcrea.io>
 *
 * The X1600's SAR ADC has four auxiliary channels, each with its own start
 * bit, interrupt and 12-bit result (X1600 programming manual, chapter 19).
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/units.h>

#define X1600_ADC_REG_ENABLE		0x00
#define X1600_ADC_REG_CTRL		0x08
#define X1600_ADC_REG_STATUS		0x0c
#define X1600_ADC_REG_DATA(bank)	(0x10 + (bank) * 4)
#define X1600_ADC_REG_CLKDIV		0x20
#define X1600_ADC_REG_STABLE		0x24

/* ENABLE: bits 3..0 start a channel's conversion; bit 15 powers down. */
#define X1600_ADC_ENABLE_POWER_DOWN	BIT(15)
/* CTRL: bits 3..0 mask the channels' interrupts. STATUS: write 1 to clear. */
#define X1600_ADC_CHANNELS_MASK		GENMASK(3, 0)
#define X1600_ADC_DATA_MASK(shift)	(GENMASK(11, 0) << ((shift) * 16))
/* CLKDIV: an even divider from the device clock; US and MS: for repeats. */
#define X1600_ADC_CLKDIV_CLKDIV		GENMASK(7, 0)
#define X1600_ADC_CLKDIV_US		GENMASK(15, 8)
#define X1600_ADC_CLKDIV_MS		GENMASK(31, 16)
#define X1600_ADC_STABLE_TIME		GENMASK(15, 0)

#define X1600_ADC_NUM_CHANNELS		4
/* SADC_VREFP when no vref-supply gives it: the datasheet's typical value. */
#define X1600_ADC_VREF_DEFAULT_mV	3300
#define X1600_ADC_BITS			12
/* The ADC clock the vendor kernel runs at. */
#define X1600_ADC_CLK_RATE_HZ		1166667
#define X1600_ADC_TIMEOUT		msecs_to_jiffies(20)

struct x1600_adc {
	void __iomem *base;
	struct clk *clk;
	struct completion done;
	int vref_mV;
	/* Serialises conversions: there is one completion for all channels. */
	struct mutex lock;
};

static u32 x1600_adc_read(struct x1600_adc *adc, unsigned int offset)
{
	return readl(adc->base + offset);
}

static void x1600_adc_write(struct x1600_adc *adc, unsigned int offset, u32 value)
{
	writel(value, adc->base + offset);
}

static irqreturn_t x1600_adc_irq(int irq, void *data)
{
	struct x1600_adc *adc = data;
	u32 status;

	status = x1600_adc_read(adc, X1600_ADC_REG_STATUS) & X1600_ADC_CHANNELS_MASK;
	if (!status)
		return IRQ_NONE;

	x1600_adc_write(adc, X1600_ADC_REG_STATUS, status);
	complete(&adc->done);
	return IRQ_HANDLED;
}

static int x1600_adc_read_raw_channel(struct x1600_adc *adc, unsigned int ch, int *val)
{
	unsigned int bank, shift;
	u32 enable, data;
	int ret;

	guard(mutex)(&adc->lock);

	reinit_completion(&adc->done);
	x1600_adc_write(adc, X1600_ADC_REG_STATUS, BIT(ch));
	enable = x1600_adc_read(adc, X1600_ADC_REG_ENABLE);
	x1600_adc_write(adc, X1600_ADC_REG_ENABLE, enable | BIT(ch));

	if (wait_for_completion_timeout(&adc->done, X1600_ADC_TIMEOUT))
		ret = 0;
	else
		ret = -ETIMEDOUT;

	enable = x1600_adc_read(adc, X1600_ADC_REG_ENABLE);
	x1600_adc_write(adc, X1600_ADC_REG_ENABLE, enable & ~BIT(ch));
	if (ret)
		return ret;

	/* Two results to a data register, the even channel's in the low half. */
	bank = ch / 2;
	shift = ch % 2;
	data = x1600_adc_read(adc, X1600_ADC_REG_DATA(bank));
	*val = field_get(X1600_ADC_DATA_MASK(shift), data);

	return 0;
}

static int x1600_adc_read_raw(struct iio_dev *iio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct x1600_adc *adc = iio_priv(iio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = x1600_adc_read_raw_channel(adc, chan->channel, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = adc->vref_mV;
		*val2 = X1600_ADC_BITS;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info x1600_adc_info = {
	.read_raw = x1600_adc_read_raw,
};

#define X1600_ADC_CHANNEL(_ch) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_ch),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
}

static const struct iio_chan_spec x1600_adc_channels[] = {
	X1600_ADC_CHANNEL(0),
	X1600_ADC_CHANNEL(1),
	X1600_ADC_CHANNEL(2),
	X1600_ADC_CHANNEL(3),
};

static void x1600_adc_power_down(void *data)
{
	struct x1600_adc *adc = data;

	x1600_adc_write(adc, X1600_ADC_REG_CTRL, X1600_ADC_CHANNELS_MASK);
	x1600_adc_write(adc, X1600_ADC_REG_ENABLE, X1600_ADC_ENABLE_POWER_DOWN);
}

static int x1600_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *iio_dev;
	struct x1600_adc *adc;
	unsigned long rate;
	u32 div, reg;
	int irq, ret;

	iio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!iio_dev)
		return -ENOMEM;

	adc = iio_priv(iio_dev);
	init_completion(&adc->done);
	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	adc->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk), "Unable to get clock\n");

	ret = devm_regulator_get_enable_read_voltage(dev, "vref");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "Unable to get the reference voltage\n");

	adc->vref_mV = ret == -ENODEV ? X1600_ADC_VREF_DEFAULT_mV : ret / (MICRO / MILLI);

	/* Powered down, interrupts masked and cleared, as the vendor does. */
	x1600_adc_write(adc, X1600_ADC_REG_ENABLE, X1600_ADC_ENABLE_POWER_DOWN);
	x1600_adc_write(adc, X1600_ADC_REG_CTRL, X1600_ADC_CHANNELS_MASK);
	x1600_adc_write(adc, X1600_ADC_REG_STATUS, X1600_ADC_CHANNELS_MASK);

	rate = clamp(clk_get_rate(adc->clk),
		     2UL * X1600_ADC_CLK_RATE_HZ, 254UL * X1600_ADC_CLK_RATE_HZ);
	div = round_up(DIV_ROUND_UP(rate, X1600_ADC_CLK_RATE_HZ), 2);
	/* US and MS at their reset values: single conversions don't use them. */
	x1600_adc_write(adc, X1600_ADC_REG_CLKDIV,
			FIELD_PREP(X1600_ADC_CLKDIV_CLKDIV, div) |
			FIELD_PREP(X1600_ADC_CLKDIV_US, 16) |
			FIELD_PREP(X1600_ADC_CLKDIV_MS, 4));
	reg = x1600_adc_read(adc, X1600_ADC_REG_STABLE) & ~X1600_ADC_STABLE_TIME;
	x1600_adc_write(adc, X1600_ADC_REG_STABLE,
			reg | FIELD_PREP(X1600_ADC_STABLE_TIME, 1));

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, x1600_adc_irq, 0, dev_name(dev), adc);
	if (ret)
		return ret;

	/*
	 * Left powered up: a typical use, such as buttons read through adc-keys,
	 * polls every few tens of milliseconds, and powering up takes 2 ms.
	 */
	x1600_adc_write(adc, X1600_ADC_REG_ENABLE, 0);
	fsleep(2 * USEC_PER_MSEC);
	x1600_adc_write(adc, X1600_ADC_REG_CTRL, 0);

	ret = devm_add_action_or_reset(dev, x1600_adc_power_down, adc);
	if (ret)
		return ret;

	iio_dev->name = "x1600-adc";
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->channels = x1600_adc_channels;
	iio_dev->num_channels = ARRAY_SIZE(x1600_adc_channels);
	iio_dev->info = &x1600_adc_info;

	return devm_iio_device_register(dev, iio_dev);
}

static const struct of_device_id x1600_adc_of_match[] = {
	{ .compatible = "ingenic,x1600-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, x1600_adc_of_match);

static struct platform_driver x1600_adc_driver = {
	.driver = {
		.name = "ingenic-x1600-adc",
		.of_match_table = x1600_adc_of_match,
	},
	.probe = x1600_adc_probe,
};
module_platform_driver(x1600_adc_driver);
MODULE_AUTHOR("Olivier Louvignes <olivier@mgcrea.io>");
MODULE_DESCRIPTION("Ingenic X1600 ADC driver");
MODULE_LICENSE("GPL");
