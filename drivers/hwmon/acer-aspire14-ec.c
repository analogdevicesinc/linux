// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hwmon driver for the Acer Aspire 14 AI embedded controller.
 *
 * Author: Ethan Jimmy <duhxu0@gmail.com>
 *
 * On the Aspire A14-52M and A14-52MT the EC is an I2C HID device
 * (vendor 0x1025, product 0x174b) that also serves the keyboard. The same
 * product id is used by other Acer keyboards. The HID driver is registered
 * only when DMI matches these models, so those machines stay with hid-generic.
 *
 * Fan speed is read with one feature report: A0 00 A0 08 00 02 02. The
 * reply carries a little-endian status word, command id, and RPM.
 */

#include <linux/dmi.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#define ACER_ASPIRE14_VID 0x1025
#define ACER_ASPIRE14_PID 0x174b
#define ACER_ASPIRE14_REPORT_ID 0xa0
#define ACER_ASPIRE14_PREFIX 0xa0
#define ACER_ASPIRE14_CMD_MARK 0x02
#define ACER_ASPIRE14_RPM_OFFSET 8
#define ACER_ASPIRE14_MIN_REPLY (ACER_ASPIRE14_RPM_OFFSET + sizeof(u16))
#define ACER_ASPIRE14_STATUS_OK 0xe000
#define ACER_ASPIRE14_CMD_FAN_RPM 0x0008

struct acer_aspire14_data {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	u8 *buf;
	u32 report_len;
};

static int acer_aspire14_fan_rpm(struct acer_aspire14_data *data, long *val)
{
	u16 status, command_id;
	int ret;

	memset(data->buf, 0, data->report_len);
	data->buf[0] = ACER_ASPIRE14_REPORT_ID;
	data->buf[2] = ACER_ASPIRE14_PREFIX;
	put_unaligned_le16(ACER_ASPIRE14_CMD_FAN_RPM, &data->buf[3]);
	data->buf[5] = ACER_ASPIRE14_CMD_MARK;
	data->buf[6] = ACER_ASPIRE14_CMD_MARK;

	ret = hid_hw_raw_request(data->hdev, ACER_ASPIRE14_REPORT_ID, data->buf,
				 data->report_len, HID_FEATURE_REPORT,
				 HID_REQ_SET_REPORT);
	if (ret < 0)
		return ret;
	if (ret != data->report_len)
		return -EIO;

	memset(data->buf, 0, data->report_len);
	data->buf[0] = ACER_ASPIRE14_REPORT_ID;
	ret = hid_hw_raw_request(data->hdev, ACER_ASPIRE14_REPORT_ID, data->buf,
				 data->report_len, HID_FEATURE_REPORT,
				 HID_REQ_GET_REPORT);
	if (ret < 0)
		return ret;
	if (ret < ACER_ASPIRE14_MIN_REPLY)
		return -EIO;

	status = get_unaligned_le16(&data->buf[1]);
	command_id = get_unaligned_le16(&data->buf[3]);
	if (status != ACER_ASPIRE14_STATUS_OK ||
	    command_id != ACER_ASPIRE14_CMD_FAN_RPM)
		return -EIO;

	*val = get_unaligned_le16(&data->buf[ACER_ASPIRE14_RPM_OFFSET]);
	return 0;
}

static int acer_aspire14_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct acer_aspire14_data *data = dev_get_drvdata(dev);

	return acer_aspire14_fan_rpm(data, val);
}

static umode_t acer_aspire14_is_visible(const void *drvdata,
					enum hwmon_sensor_types type, u32 attr,
					int channel)
{
	return 0444;
}

static const struct hwmon_channel_info *const acer_aspire14_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT), NULL
};

static const struct hwmon_ops acer_aspire14_ops = {
	.is_visible = acer_aspire14_is_visible,
	.read = acer_aspire14_read,
};

static const struct hwmon_chip_info acer_aspire14_chip_info = {
	.ops = &acer_aspire14_ops,
	.info = acer_aspire14_info,
};

static const struct dmi_system_id acer_aspire14_dmi_table[] = {
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "Aspire A14-52M"),
		},
	},
	{
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "Acer"),
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "Aspire A14-52MT"),
		},
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, acer_aspire14_dmi_table);

static u32 acer_aspire14_fan_report_len(struct hid_device *hdev)
{
	struct hid_report *report;
	u32 len;

	report = hdev->report_enum[HID_FEATURE_REPORT]
			 .report_id_hash[ACER_ASPIRE14_REPORT_ID];
	if (!report)
		return 0;

	len = hid_report_len(report);
	if (len < ACER_ASPIRE14_MIN_REPLY || len > HID_MAX_BUFFER_SIZE)
		return 0;

	return len;
}

static int acer_aspire14_probe(struct hid_device *hdev,
			       const struct hid_device_id *id)
{
	struct acer_aspire14_data *data;
	struct device *hwmon_dev;
	long rpm;
	u32 report_len;
	int ret;

	hdev->quirks |= HID_QUIRK_INPUT_PER_APP;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT | HID_CONNECT_DRIVER);
	if (ret)
		return ret;

	report_len = acer_aspire14_fan_report_len(hdev);
	if (!report_len) {
		ret = -ENODEV;
		goto stop;
	}

	data = devm_kzalloc(&hdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto stop;
	}

	data->buf = devm_kzalloc(&hdev->dev, report_len, GFP_KERNEL);
	if (!data->buf) {
		ret = -ENOMEM;
		goto stop;
	}
	data->report_len = report_len;

	data->hdev = hdev;
	hid_set_drvdata(hdev, data);

	ret = acer_aspire14_fan_rpm(data, &rpm);
	if (ret)
		hid_dbg(hdev, "fan report failed: %d\n", ret);
	else
		hid_dbg(hdev, "initial fan speed %ld RPM\n", rpm);

	hwmon_dev =
		hwmon_device_register_with_info(&hdev->dev, "aspire14_ec", data,
						&acer_aspire14_chip_info, NULL);
	if (IS_ERR(hwmon_dev)) {
		ret = PTR_ERR(hwmon_dev);
		hid_warn(hdev, "failed to register hwmon: %pe\n", hwmon_dev);
		goto stop;
	}
	data->hwmon_dev = hwmon_dev;

	return 0;

stop:
	hid_hw_stop(hdev);
	return ret;
}

static void acer_aspire14_remove(struct hid_device *hdev)
{
	struct acer_aspire14_data *data = hid_get_drvdata(hdev);

	if (data && data->hwmon_dev)
		hwmon_device_unregister(data->hwmon_dev);
	hid_hw_stop(hdev);
}

static int acer_aspire14_reset_resume(struct hid_device *hdev)
{
	if (hdev->claimed & HID_CLAIMED_INPUT)
		hidinput_reset_resume(hdev);

	return 0;
}

static const struct hid_device_id acer_aspire14_id_table[] = {
	{ HID_I2C_DEVICE(ACER_ASPIRE14_VID, ACER_ASPIRE14_PID) },
	{}
};

static struct hid_driver acer_aspire14_driver = {
	.name = "acer-aspire14-ec",
	.id_table = acer_aspire14_id_table,
	.probe = acer_aspire14_probe,
	.remove = acer_aspire14_remove,
	.reset_resume = acer_aspire14_reset_resume,
};

static int __init acer_aspire14_init(void)
{
	if (!dmi_check_system(acer_aspire14_dmi_table))
		return -ENODEV;

	return hid_register_driver(&acer_aspire14_driver);
}

static void __exit acer_aspire14_exit(void)
{
	hid_unregister_driver(&acer_aspire14_driver);
}

late_initcall(acer_aspire14_init);
module_exit(acer_aspire14_exit);

MODULE_AUTHOR("Ethan Jimmy <duhxu0@gmail.com>");
MODULE_DESCRIPTION("Acer Aspire 14 AI EC fan driver");
MODULE_LICENSE("GPL");
