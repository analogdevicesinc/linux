/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AXI_JESD204_H_
#define _AXI_JESD204_H_

#include <linux/bitfield.h>

#define JESD204_REG_SYNTH_DATA_PATH_WIDTH	0x14
#define JESD204_SYNTH_DATA_PATH_WIDTH_MASK	GENMASK(7, 0)
#define JESD204_SYNTH_DATA_PATH_WIDTH_GET(x)	FIELD_GET(JESD204_SYNTH_DATA_PATH_WIDTH_MASK, x)
#define JESD204_TPL_DATA_PATH_WIDTH_MASK	GENMASK(15, 8)
#define JESD204_TPL_DATA_PATH_WIDTH_GET(x)	FIELD_GET(JESD204_TPL_DATA_PATH_WIDTH_MASK, x)

#define JESD204_REG_SYNTH_REG_1		0x18
#define JESD204_ENCODER_MASK		GENMASK(9, 8)
#define JESD204_ENCODER_GET(x)		FIELD_GET(JESD204_ENCODER_MASK, x)

/**
 * axi_jesd_ext_reset - Perform an optional external GT reset using GPIO signals
 * @dev: Pointer to the device structure
 * @name: Name of the reset signal
 * @reset: Pointer to the GPIO descriptor for the reset pin
 * @done: Pointer to the GPIO descriptor for the done pin
 *
 * This function performs an external reset by controlling the reset and done
 * GPIO pins. It sets the reset pin to high, waits for a short delay, and then
 * sets the reset pin to low. It then waits for the done pin to indicate that
 * the reset is complete. The function will retry up to 32 times, waiting for
 * 1 millisecond between each attempt. If the reset is successful, it returns 0.
 * If the reset times out, it returns -ETIMEDOUT.
 *
 * Return: 0 on success, -ETIMEDOUT on timeout, or an error code on failure
 */
static inline int axi_jesd_ext_reset(struct device *dev, const char *name,
				     struct gpio_desc *reset, struct gpio_desc *done)
{
	int ret, count = 0;
	unsigned int resetdone;

	if (!(reset && done))
		return 0;

	gpiod_set_value(reset, 1);
	fsleep(20);
	gpiod_set_value(reset, 0);

	while (count < 32) {
		fsleep(1000);
		ret = gpiod_get_value(done);
		if (ret < 0) {
			dev_err(dev, "Waiting for %s_reset_done failed (%d)\n", name, ret);
			return ret;
		}
		resetdone = ret;
		fsleep(1000);

		ret = gpiod_get_value(done);
		if (ret < 0) {
			dev_err(dev, "Waiting for %s_reset_done failed (%d)\n", name, ret);
			return ret;
		}

		resetdone &= ret;

		if (resetdone) {
			dev_dbg(dev, "Waiting for %s_reset_done: %d\n", name, count);
			return 0;
		}
		count++;
	}

	dev_err(dev, "Waiting for %s_reset_done timedout (%d)\n", name, ret);

	return -ETIMEDOUT;
}
#endif
