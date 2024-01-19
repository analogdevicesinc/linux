// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform driver for a PWM trigger and DMA buffer connected to a SPI
 * controller offload instance implementing the iio-hw-triggered-buffer
 * interface.
 *
 * Copyright (C) 2023 Analog Devices, Inc.
 * Copyright (C) 2023 BayLibre, SAS
 */

#include <linux/auxiliary_bus.h>
#include <linux/pwm.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw_triggered_buffer_impl.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/sysfs.h>

struct iio_pwm_triggered_dma_buffer {
	struct iio_hw_triggered_buffer_device hw;
	struct pwm_device *pwm;
};

static const struct iio_trigger_ops iio_pwm_triggered_dma_buffer_ops;

static int iio_pwm_triggered_dma_buffer_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_pwm_triggered_dma_buffer *st = iio_trigger_get_drvdata(trig);

	if (state)
		return pwm_enable(st->pwm);

	pwm_disable(st->pwm);

	return 0;
}

static int iio_pwm_triggered_dma_buffer_validate_device(struct iio_trigger *trig,
							struct iio_dev *indio_dev)
{
	/* Don't allow assigning trigger via sysfs. */
	return -EINVAL;
}

static const struct iio_trigger_ops iio_pwm_triggered_dma_buffer_ops = {
	.set_trigger_state = iio_pwm_triggered_dma_buffer_set_state,
	.validate_device = iio_pwm_triggered_dma_buffer_validate_device,
};

static u32 axi_spi_engine_offload_pwm_trigger_get_rate(struct iio_trigger *trig)
{
	struct iio_pwm_triggered_dma_buffer *st = iio_trigger_get_drvdata(trig);
	u64 period_ns = pwm_get_period(st->pwm);

	if (period_ns)
		return DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, period_ns);

	return 0;
}

static int
axi_spi_engine_offload_set_samp_freq(struct iio_pwm_triggered_dma_buffer *st,
				     u32 requested_hz)
{
	int period_ns;

	if (requested_hz == 0)
		return -EINVAL;

	period_ns = DIV_ROUND_UP(NSEC_PER_SEC, requested_hz);

	/*
	 * FIXME: We really just need a clock, not a PWM. The current duty cycle
	 * value is a hack to work around the edge vs. level offload trigger
	 * issue in the ADI AXI SPI Engine firmware.
	 */
	return pwm_config(st->pwm, period_ns / 2, period_ns);
}

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);

	return sysfs_emit(buf, "%u\n",
			  axi_spi_engine_offload_pwm_trigger_get_rate(trig));
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_pwm_triggered_dma_buffer *st = iio_trigger_get_drvdata(trig);
	int ret;
	u32 val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	ret = axi_spi_engine_offload_set_samp_freq(st, val);
	if (ret)
		return ret;

	return len;
}

static DEVICE_ATTR_RW(sampling_frequency);

static struct attribute *iio_pwm_triggered_dma_buffer_attrs[] = {
	&dev_attr_sampling_frequency.attr,
	NULL
};

ATTRIBUTE_GROUPS(iio_pwm_triggered_dma_buffer);

static void iio_pwm_triggered_dma_buffer_adev_release(struct device *dev)
{
}

static void iio_pwm_triggered_dma_buffer_unregister_adev(void *adev)
{
	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int iio_pwm_triggered_dma_buffer_probe(struct platform_device *pdev)
{
	struct iio_pwm_triggered_dma_buffer *st;
	struct auxiliary_device *adev;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(st->pwm))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->pwm),
				     "failed to get PWM\n");

	st->hw.buffer = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "rx");
	if (IS_ERR(st->hw.buffer))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->hw.buffer),
				     "failed to allocate buffer\n");

	st->hw.trig = devm_iio_trigger_alloc(&pdev->dev, "%s-%s-pwm-trigger",
					     dev_name(pdev->dev.parent),
					     dev_name(&pdev->dev));
	if (!st->hw.trig)
		return -ENOMEM;

	st->hw.trig->ops = &iio_pwm_triggered_dma_buffer_ops;
	st->hw.trig->dev.parent = &pdev->dev;
	st->hw.trig->dev.groups = iio_pwm_triggered_dma_buffer_groups;
	iio_trigger_set_drvdata(st->hw.trig, st);

	/* start with a reasonable default value */
	ret = axi_spi_engine_offload_set_samp_freq(st, 1000);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set sampling frequency\n");

	ret = devm_iio_trigger_register(&pdev->dev, st->hw.trig);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register trigger\n");

	adev = &st->hw.adev;
	adev->name = "triggered-buffer";
	adev->dev.parent = &pdev->dev;
	adev->dev.release = iio_pwm_triggered_dma_buffer_adev_release;
	adev->id = 0;

	ret = auxiliary_device_init(adev);
	if (ret)
		return ret;

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(&pdev->dev,
			iio_pwm_triggered_dma_buffer_unregister_adev, adev);
}

static const struct of_device_id iio_pwm_triggered_dma_buffer_match_table[] = {
	{ .compatible = "adi,spi-offload-pwm-trigger-dma-buffer" },
	{ }
};
MODULE_DEVICE_TABLE(of, iio_pwm_triggered_dma_buffer_match_table);

static struct platform_driver iio_pwm_triggered_dma_buffer_driver = {
	.probe = iio_pwm_triggered_dma_buffer_probe,
	.driver = {
		.name = "iio-pwm-triggered-dma-buffer",
		.of_match_table = iio_pwm_triggered_dma_buffer_match_table,
	},
};
module_platform_driver(iio_pwm_triggered_dma_buffer_driver);

MODULE_AUTHOR("David Lechner <dlechner@baylibre.com>");
MODULE_DESCRIPTION("AXI SPI Engine Offload PWM Trigger");
MODULE_LICENSE("GPL");
