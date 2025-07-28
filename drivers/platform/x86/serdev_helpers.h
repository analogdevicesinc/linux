/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * In some cases UART attached devices which require an in kernel driver,
 * e.g. UART attached Bluetooth HCIs are described in the ACPI tables
 * by an ACPI device with a broken or missing UartSerialBusV2() resource.
 *
 * This causes the kernel to create a /dev/ttyS# char-device for the UART
 * instead of creating an in kernel serdev-controller + serdev-device pair
 * for the in kernel driver.
 *
 * The quirk handling in acpi_quirk_skip_serdev_enumeration() makes the kernel
 * create a serdev-controller device for these UARTs instead of a /dev/ttyS#.
 *
 * Instantiating the actual serdev-device to bind to is up to pdx86 code,
 * this header provides a helper for getting the serdev-controller device.
 */
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/printk.h>
#include <linux/sprintf.h>
#include <linux/string.h>

static inline struct device *
get_serdev_controller(const char *serial_ctrl_hid,
		      const char *serial_ctrl_uid,
		      int serial_ctrl_port,
		      const char *serdev_ctrl_name)
{
	struct device *ctrl_dev, *child;
	struct acpi_device *ctrl_adev;
	char name[32];
	int i;

	ctrl_adev = acpi_dev_get_first_match_dev(serial_ctrl_hid, serial_ctrl_uid, -1);
	if (!ctrl_adev) {
		pr_err("error could not get %s/%s serial-ctrl adev\n",
		       serial_ctrl_hid, serial_ctrl_uid);
		return ERR_PTR(-ENODEV);
	}

	/* get_first_physical_node() returns a weak ref */
	ctrl_dev = get_device(acpi_get_first_physical_node(ctrl_adev));
	if (!ctrl_dev) {
		pr_err("error could not get %s/%s serial-ctrl physical node\n",
		       serial_ctrl_hid, serial_ctrl_uid);
		ctrl_dev = ERR_PTR(-ENODEV);
		goto put_ctrl_adev;
	}

	/* Walk host -> uart-ctrl -> port -> serdev-ctrl */
	for (i = 0; i < 3; i++) {
		switch (i) {
		case 0:
			snprintf(name, sizeof(name), "%s:0", dev_name(ctrl_dev));
			break;
		case 1:
			snprintf(name, sizeof(name), "%s.%d",
				 dev_name(ctrl_dev), serial_ctrl_port);
			break;
		case 2:
			strscpy(name, serdev_ctrl_name, sizeof(name));
			break;
		}

		child = device_find_child_by_name(ctrl_dev, name);
		put_device(ctrl_dev);
		if (!child) {
			pr_err("error could not find '%s' device\n", name);
			ctrl_dev = ERR_PTR(-ENODEV);
			goto put_ctrl_adev;
		}

		ctrl_dev = child;
	}

put_ctrl_adev:
	acpi_dev_put(ctrl_adev);
	return ctrl_dev;
}
