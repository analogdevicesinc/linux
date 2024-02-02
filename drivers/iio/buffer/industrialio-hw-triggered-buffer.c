// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Analog Devices, Inc.
 * Copyright (c) 2024 BayLibre, SAS
 */

#include <linux/auxiliary_bus.h>
#include <linux/container_of.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/iio/hw_triggered_buffer_impl.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>

static int iio_hw_triggered_buffer_match(struct device *dev, const void *match)
{
	return dev->parent == match;
}

static struct iio_hw_triggered_buffer_device
*iio_hw_trigger_buffer_get(struct device *match)
{
	struct auxiliary_device *adev;

	adev = auxiliary_find_device(NULL, match, iio_hw_triggered_buffer_match);
	if (!adev)
		return ERR_PTR(-EPROBE_DEFER);

	return container_of(adev, struct iio_hw_triggered_buffer_device, adev);
}

static void iio_hw_trigger_buffer_put(void *dev)
{
	put_device(dev);
}

/**
 * devm_iio_hw_triggered_buffer_setup - Setup a hardware triggered buffer
 * @dev:	Device for devm management
 * @indio_dev:	An unconfigured/partially configured IIO device struct
 * @match:	Device for matching the auxiliary bus device that provides the
 *		interface to the hardware triggered buffer
 * @ops:	Buffer setup functions to use for this IIO device
 *
 * Return: 0 on success, negative error code on failure.
 *
 * This function will search all registered hardware triggered buffers for one
 * that matches the given indio_dev. If found, it will be used to setup both
 * the trigger and the buffer on the indio_dev.
 */
int devm_iio_hw_triggered_buffer_setup(struct device *dev,
				       struct iio_dev *indio_dev,
				       struct device *match,
				       const struct iio_buffer_setup_ops *ops)
{
	struct iio_hw_triggered_buffer_device *hw;
	int ret;

	hw = iio_hw_trigger_buffer_get(match);
	if (IS_ERR(hw))
		return PTR_ERR(hw);

	ret = devm_add_action_or_reset(dev, iio_hw_trigger_buffer_put, &hw->adev.dev);
	if (ret)
		return ret;

	indio_dev->modes |= INDIO_HW_BUFFER_TRIGGERED;
	indio_dev->trig = iio_trigger_get(hw->trig);
	indio_dev->setup_ops = ops;

	return iio_device_attach_buffer(indio_dev, hw->buffer);
}
EXPORT_SYMBOL_GPL(devm_iio_hw_triggered_buffer_setup);

static int iio_hw_trigger_buffer_probe(struct auxiliary_device *adev,
				       const struct auxiliary_device_id *id)
{
	struct iio_hw_triggered_buffer_device *hw =
		container_of(adev, struct iio_hw_triggered_buffer_device, adev);

	if (!hw->buffer || !hw->trig)
		return -EINVAL;

	return 0;
}

static const struct auxiliary_device_id iio_hw_trigger_buffer_id_table[] = {
	{ .name = "pwm-triggered-dma-buffer.triggered-buffer" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, iio_hw_trigger_buffer_id_table);

static struct auxiliary_driver iio_hw_trigger_buffer_driver = {
	.driver = {
		.name = "iio-hw-triggered-buffer",
	},
	.probe = iio_hw_trigger_buffer_probe,
	.id_table = iio_hw_trigger_buffer_id_table,
};
module_auxiliary_driver(iio_hw_trigger_buffer_driver);

MODULE_AUTHOR("David Lechner <dlechner@baylibre.com>");
MODULE_DESCRIPTION("IIO helper functions for setting up hardware triggered buffers");
MODULE_LICENSE("GPL");
