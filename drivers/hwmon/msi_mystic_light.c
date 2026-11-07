// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * hwmon driver for the voltage sensors of the MSI Mystic Light USB
 * controller (0db0:0076) found on MSI 800-series motherboards
 *
 * The microcontroller that drives the RGB LED headers of these boards also
 * measures a few voltage rails that are not monitored by the Super I/O chip.
 * They are returned in HID feature report 0xf0. This driver only reads that
 * report and never sends anything else to the device. The hidraw interface
 * is kept, so user space LED control keeps working.
 *
 * The controller has the same USB ID on different boards, while the rails
 * and their position in the report depend on the board. The driver is
 * therefore only registered on supported boards, which are matched by DMI.
 *
 * Copyright (C) 2026 Tomasz Bojanowski <tomasz@bojanowski.me>
 */

#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/dmi.h>
#include <linux/err.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#define USB_VENDOR_ID_MSI_MB		0x0db0
#define USB_PRODUCT_ID_MYSTIC_LIGHT	0x0076

#define MSI_ML_SENSOR_REPORT_ID		0xf0
#define MSI_ML_SENSOR_REPORT_SIZE	64

/* Fixed bytes found in every sensor report: 0xa0 at [1], 01 fc at [9..10] */
#define MSI_ML_SENSOR_CMD_OFFSET	1
#define MSI_ML_SENSOR_CMD		0xa0
#define MSI_ML_SENSOR_PAGE_OFFSET	9
#define MSI_ML_SENSOR_PAGE_HI		0x01
#define MSI_ML_SENSOR_PAGE_LO		0xfc

#define MSI_ML_MAX_IN			3
#define MSI_ML_CACHE_TIME		HZ

struct msi_ml_board {
	int num_in;
	const char *const *in_label;
	/* little endian u16 in mV, offset at most MSI_ML_SENSOR_REPORT_SIZE - 2 */
	const u8 *in_offset;
};

/* MAG B850M MORTAR WIFI (MS-7E61) */
static const char *const msi_ml_7e61_in_label[] = {
	"CPU 1P8", "CHIP Core", "VDD MISC",
};

static const u8 msi_ml_7e61_in_offset[] = { 23, 17, 25 };

static_assert(ARRAY_SIZE(msi_ml_7e61_in_label) ==
	      ARRAY_SIZE(msi_ml_7e61_in_offset));
static_assert(ARRAY_SIZE(msi_ml_7e61_in_label) <= MSI_ML_MAX_IN);

static const struct msi_ml_board msi_ml_7e61 = {
	.num_in = ARRAY_SIZE(msi_ml_7e61_in_label),
	.in_label = msi_ml_7e61_in_label,
	.in_offset = msi_ml_7e61_in_offset,
};

static const struct dmi_system_id msi_ml_dmi_table[] __initconst = {
	{
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Micro-Star International"),
			DMI_MATCH(DMI_BOARD_NAME, "MS-7E61"),
		},
		.driver_data = (void *)&msi_ml_7e61,
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, msi_ml_dmi_table);

/* Board matched by DMI, set once before the HID driver is registered */
static const struct msi_ml_board *msi_ml_board;

struct msi_ml_data {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	const struct msi_ml_board *board;
	u8 *buf;			/* report buffer, must be DMA safe */
	u16 in[MSI_ML_MAX_IN];		/* mV */
	unsigned int in_mask;		/* rails with a reading in probe */
	u64 updated;			/* jiffies of the last successful read */
};

/*
 * Read the sensor report. Called from probe before the hwmon device is
 * registered, and afterwards only from the hwmon read callback, which the
 * hwmon core serializes. No additional locking is needed for the buffer
 * and the cached values.
 */
static int msi_ml_read_report(struct msi_ml_data *priv)
{
	const struct msi_ml_board *board = priv->board;
	u8 *buf = priv->buf;
	int i, ret;

	memset(buf, 0, MSI_ML_SENSOR_REPORT_SIZE);
	buf[0] = MSI_ML_SENSOR_REPORT_ID;

	/* Make sure the device is not autosuspended during the request */
	ret = hid_hw_power(priv->hdev, PM_HINT_FULLON);
	if (ret < 0)
		return ret;

	ret = hid_hw_raw_request(priv->hdev, MSI_ML_SENSOR_REPORT_ID, buf,
				 MSI_ML_SENSOR_REPORT_SIZE, HID_FEATURE_REPORT,
				 HID_REQ_GET_REPORT);

	hid_hw_power(priv->hdev, PM_HINT_NORMAL);

	if (ret < 0)
		return ret;
	if (ret != MSI_ML_SENSOR_REPORT_SIZE)
		return -EPROTO;

	if (buf[0] != MSI_ML_SENSOR_REPORT_ID ||
	    buf[MSI_ML_SENSOR_CMD_OFFSET] != MSI_ML_SENSOR_CMD ||
	    buf[MSI_ML_SENSOR_PAGE_OFFSET] != MSI_ML_SENSOR_PAGE_HI ||
	    buf[MSI_ML_SENSOR_PAGE_OFFSET + 1] != MSI_ML_SENSOR_PAGE_LO)
		return -EPROTO;

	for (i = 0; i < board->num_in; i++)
		priv->in[i] = get_unaligned_le16(buf + board->in_offset[i]);

	priv->updated = get_jiffies_64();

	return 0;
}

static umode_t msi_ml_is_visible(const void *data,
				 enum hwmon_sensor_types type, u32 attr,
				 int channel)
{
	const struct msi_ml_data *priv = data;

	/* Only the rails that returned a reading in probe */
	if (priv->in_mask & BIT(channel))
		return 0444;

	return 0;
}

static int msi_ml_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	struct msi_ml_data *priv = dev_get_drvdata(dev);
	int ret;

	if (time_after_eq64(get_jiffies_64(),
			    priv->updated + MSI_ML_CACHE_TIME)) {
		ret = msi_ml_read_report(priv);
		if (ret)
			return ret;
	}

	*val = priv->in[channel];

	return 0;
}

static int msi_ml_read_string(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, const char **str)
{
	struct msi_ml_data *priv = dev_get_drvdata(dev);

	*str = priv->board->in_label[channel];

	return 0;
}

static const struct hwmon_ops msi_ml_hwmon_ops = {
	.is_visible = msi_ml_is_visible,
	.read = msi_ml_read,
	.read_string = msi_ml_read_string,
};

/* One entry per channel, MSI_ML_MAX_IN in total */
static const struct hwmon_channel_info *const msi_ml_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info msi_ml_chip_info = {
	.ops = &msi_ml_hwmon_ops,
	.info = msi_ml_info,
};

static int msi_ml_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct msi_ml_data *priv;
	int i, ret;

	priv = devm_kzalloc(&hdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->buf = devm_kzalloc(&hdev->dev, MSI_ML_SENSOR_REPORT_SIZE, GFP_KERNEL);
	if (!priv->buf)
		return -ENOMEM;

	priv->hdev = hdev;
	priv->board = msi_ml_board;
	hid_set_drvdata(hdev, priv);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	/*
	 * Keep the hidraw interface for user space LED control. Also claim
	 * the device for this driver, so that probing does not fail when
	 * CONFIG_HIDRAW is disabled.
	 */
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW | HID_CONNECT_DRIVER);
	if (ret)
		return ret;

	/*
	 * Keep the device open while the driver is bound, so that its
	 * interrupt endpoint is polled as it is with the vendor software.
	 * Reading the sensor report has only been tested in this state.
	 */
	ret = hid_hw_open(hdev);
	if (ret)
		goto fail_stop;

	/* Check that the sensor report works, and which rails have a reading */
	ret = msi_ml_read_report(priv);
	if (ret) {
		hid_err(hdev, "failed to read the sensor report: %pe\n",
			ERR_PTR(ret));
		goto fail_close;
	}

	for (i = 0; i < priv->board->num_in; i++) {
		if (priv->in[i])
			priv->in_mask |= BIT(i);
	}

	if (!priv->in_mask) {
		hid_err(hdev, "no voltage readings in the sensor report\n");
		ret = -ENODEV;
		goto fail_close;
	}

	priv->hwmon_dev = hwmon_device_register_with_info(&hdev->dev,
							  "msi_mystic_light",
							  priv,
							  &msi_ml_chip_info,
							  NULL);
	if (IS_ERR(priv->hwmon_dev)) {
		ret = PTR_ERR(priv->hwmon_dev);
		goto fail_close;
	}

	return 0;

fail_close:
	hid_hw_close(hdev);
fail_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void msi_ml_remove(struct hid_device *hdev)
{
	struct msi_ml_data *priv = hid_get_drvdata(hdev);

	hwmon_device_unregister(priv->hwmon_dev);
	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id msi_ml_table[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_MSI_MB, USB_PRODUCT_ID_MYSTIC_LIGHT) },
	{ }
};

static struct hid_driver msi_ml_driver = {
	.name = "msi_mystic_light",
	.id_table = msi_ml_table,
	.probe = msi_ml_probe,
	.remove = msi_ml_remove,
};

static int __init msi_ml_init(void)
{
	const struct dmi_system_id *dmi_id;

	dmi_id = dmi_first_match(msi_ml_dmi_table);
	if (!dmi_id)
		return -ENODEV;

	msi_ml_board = dmi_id->driver_data;

	return hid_register_driver(&msi_ml_driver);
}

static void __exit msi_ml_exit(void)
{
	hid_unregister_driver(&msi_ml_driver);
}

/* When built into the kernel, register after the HID bus is initialized */
late_initcall(msi_ml_init);
module_exit(msi_ml_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tomasz Bojanowski <tomasz@bojanowski.me>");
MODULE_DESCRIPTION("Hwmon driver for MSI Mystic Light USB controller voltage sensors");
