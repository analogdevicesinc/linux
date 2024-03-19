// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>


#define PWM_CONTROL_REG                 0x40
#define PWM_CLK_DIV_REG                 0x44
#define PWM_AVRG_READY_REG              0x48

#define ADI_PWM_DAC_NUM_CHANNELS        16

struct adi_pwm_dac {
	void __iomem *base;
	struct mutex lock;
	u32 iovdd_microvolt;
	u32 gpio_max_frequency;
};

#define ADI_PWM_DAC_CHANNEL(chan) {                  \
		.type = IIO_VOLTAGE,                    \
		.indexed = 1,                           \
		.output = 1,                            \
		.channel = chan,                        \
		.info_mask_separate =                   \
			BIT(IIO_CHAN_INFO_RAW) |        \
			BIT(IIO_CHAN_INFO_ENABLE),      \
		.info_mask_shared_by_all =              \
			BIT(IIO_CHAN_INFO_FREQUENCY),   \
}

static const struct iio_chan_spec adi_pwm_dac_channels[] = {
	ADI_PWM_DAC_CHANNEL(0),
	ADI_PWM_DAC_CHANNEL(1),
	ADI_PWM_DAC_CHANNEL(2),
	ADI_PWM_DAC_CHANNEL(3),
	ADI_PWM_DAC_CHANNEL(4),
	ADI_PWM_DAC_CHANNEL(5),
	ADI_PWM_DAC_CHANNEL(6),
	ADI_PWM_DAC_CHANNEL(7),
	ADI_PWM_DAC_CHANNEL(8),
	ADI_PWM_DAC_CHANNEL(9),
	ADI_PWM_DAC_CHANNEL(10),
	ADI_PWM_DAC_CHANNEL(11),
	ADI_PWM_DAC_CHANNEL(12),
	ADI_PWM_DAC_CHANNEL(13),
	ADI_PWM_DAC_CHANNEL(14),
	ADI_PWM_DAC_CHANNEL(15),
};

static int adi_pwm_dac_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct adi_pwm_dac *const dac = iio_priv(indio_dev);
	u32 reg;
	u64 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		reg = readl(dac->base + chan->channel * 4);
		tmp = dac->iovdd_microvolt;
		tmp = tmp * reg / 65521;
		*val = (int)tmp;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_ENABLE:
		reg = readl(dac->base + PWM_CONTROL_REG);
		*val = (reg >> chan->channel) & 0x1;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_FREQUENCY:
		reg = readl(dac->base + PWM_CLK_DIV_REG);
		*val = dac->gpio_max_frequency / (reg + 1);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int adi_pwm_dac_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct adi_pwm_dac *const dac = iio_priv(indio_dev);
	u32 reg;
	u64 tmp;
	unsigned long timenow;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/* The required analog voltage should not exceed IOVDD */
		if (val > dac->iovdd_microvolt)
			return -EINVAL;

		tmp = val;
		tmp = tmp * 65521 / dac->iovdd_microvolt;

		/*
		 * Wait until the AVRG register is ready to be written or
		 * timeout. One second should be good for timeout value.
		 */
		timenow = jiffies;
		do
			reg = readl(dac->base + PWM_AVRG_READY_REG);
		while ((1UL & (reg >> chan->channel)) == 0
		       && time_before(jiffies, timenow + HZ));

		if ((1UL & (reg >> chan->channel)) == 0)
			return -EBUSY;

		reg = (u32)tmp;
		writel(reg, dac->base + chan->channel * 4);
		return 0;

	case IIO_CHAN_INFO_ENABLE:
		if (val != 0 && val != 1)
			return -EINVAL;

		mutex_lock(&dac->lock);

		reg = readl(dac->base + PWM_CONTROL_REG);
		if (val == 0)
			reg &= ~((u32)1 << chan->channel);
		else /* val == 1 */
			reg |= (u32)1 << chan->channel;

		writel(reg, dac->base + PWM_CONTROL_REG);

		mutex_unlock(&dac->lock);

		return 0;

	case IIO_CHAN_INFO_FREQUENCY:
		if (val > dac->gpio_max_frequency)
			return -EINVAL;

		reg = dac->gpio_max_frequency / val - 1;
		writel(reg, dac->base + PWM_CLK_DIV_REG);
		return 0;

	default:
		return -EINVAL;
	}
}

static const struct iio_info adi_pwm_dac_info = {
	.read_raw	= adi_pwm_dac_read_raw,
	.write_raw	= adi_pwm_dac_write_raw
};


static int adi_pwm_dac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct adi_pwm_dac *dac;
	int i, ret;

	if (!np)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, indio_dev);
	dac = iio_priv(indio_dev);

	mutex_init(&dac->lock);

	indio_dev->info = &adi_pwm_dac_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adi_pwm_dac_channels;
	indio_dev->num_channels = ADI_PWM_DAC_NUM_CHANNELS;
	indio_dev->name = dev_name(dev);

	dac->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dac->base))
		return PTR_ERR(dac->base);

	ret = of_property_read_u32(dev->of_node, "adi,iovdd-microvolt",
				   &dac->iovdd_microvolt);
	if (ret != 0) {
		dev_err(dev, "Missing or bad adi,iovdd_microvolt property\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node, "adi,gpio-max-frequency",
				   &dac->gpio_max_frequency);
	if (ret != 0) {
		dev_err(dev,
			"Missing or bad adi,gpio_max_frequency property\n");
		return -EINVAL;
	}

	/* Disable all channels */
	writel(0, dac->base + PWM_CONTROL_REG);

	for (i = 0; i < ADI_PWM_DAC_NUM_CHANNELS; i++)
		writel(0, dac->base + i * 4);

	/* The reset value of PWM_CLK_DIV_REG is 0x7 */
	writel(7, dac->base + PWM_CLK_DIV_REG);

	return iio_device_register(indio_dev);
}

static int adi_pwm_dac_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct adi_pwm_dac *dac = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	/* Disable all channels */
	writel(0, dac->base + PWM_CONTROL_REG);

	return 0;
}

static const struct of_device_id adi_pwm_dac_of_match[] = {
	{ .compatible = "adi,pwm-dac", },
	{},
};
MODULE_DEVICE_TABLE(of, adi_pwm_dac_of_match);

static struct platform_driver adi_pwm_dac_driver = {
	.probe			= adi_pwm_dac_probe,
	.remove			= adi_pwm_dac_remove,
	.driver			= {
		.name		= "adi-pwm-dac",
		.of_match_table = adi_pwm_dac_of_match,
	},
};
module_platform_driver(adi_pwm_dac_driver);

MODULE_DESCRIPTION("Analog Devices PWM DAC driver");
MODULE_AUTHOR("Jie Zhang <jie.zhang@analog.com>");
MODULE_LICENSE("GPL v2");
