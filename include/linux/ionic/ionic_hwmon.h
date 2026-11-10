/* SPDX-License-Identifier: (GPL-2.0 OR Linux-OpenIB) OR BSD-2-Clause */
/* Copyright(c) 2026 Advanced Micro Devices, Inc */

#ifndef _LINUX_IONIC_HWMON_H_
#define _LINUX_IONIC_HWMON_H_

#include <linux/types.h>

#define IONIC_HWMON_CAPS_BUF_SIZE	4096
#define IONIC_HWMON_CAPS_VERSION_0	0

/**
 * enum ionic_hwmon_sensor_type - Hardware monitoring sensor types
 * @IONIC_HWMON_TYPE_TEMP:      Temperature sensors
 * @IONIC_HWMON_TYPE_VOLTAGE:   Voltage sensors
 * @IONIC_HWMON_TYPE_POWER:     Power sensors
 * @IONIC_HWMON_TYPE_CHIP:      Chip wide attributes
 * @IONIC_HWMON_TYPE_CURRENT:   Current sensors
 * @IONIC_HWMON_TYPE_ENERGY:    Energy sensors
 * @IONIC_HWMON_TYPE_HUMIDITY:  Humidity sensors
 * @IONIC_HWMON_TYPE_FAN:       Fan speed sensors
 * @IONIC_HWMON_TYPE_PWM:       PWM controllers
 * @IONIC_HWMON_TYPE_INTRUSION: Intrusion detection
 * @IONIC_HWMON_TYPE_MAX:       Number of sensor types defined by this interface
 *
 * These values do not necessarily match enum hwmon_sensor_types.
 */
enum ionic_hwmon_sensor_type {
	IONIC_HWMON_TYPE_TEMP		= 0,
	IONIC_HWMON_TYPE_VOLTAGE	= 1,
	IONIC_HWMON_TYPE_POWER		= 2,
	IONIC_HWMON_TYPE_CHIP		= 3,
	IONIC_HWMON_TYPE_CURRENT	= 4,
	IONIC_HWMON_TYPE_ENERGY		= 5,
	IONIC_HWMON_TYPE_HUMIDITY	= 6,
	IONIC_HWMON_TYPE_FAN		= 7,
	IONIC_HWMON_TYPE_PWM		= 8,
	IONIC_HWMON_TYPE_INTRUSION	= 9,
	IONIC_HWMON_TYPE_MAX		= 10,
};

/**
 * enum ionic_hwmon_temp_attr - Temperature sensor attributes
 * @IONIC_HWMON_TEMP_ATTR_INPUT:      Temperature input reading
 * @IONIC_HWMON_TEMP_ATTR_LABEL:      Temperature sensor label
 * @IONIC_HWMON_TEMP_ATTR_CRIT:       Critical temperature threshold
 * @IONIC_HWMON_TEMP_ATTR_CRIT_ALARM: Critical temperature alarm status
 */
enum ionic_hwmon_temp_attr {
	IONIC_HWMON_TEMP_ATTR_INPUT		= 0,
	IONIC_HWMON_TEMP_ATTR_LABEL		= 1,
	IONIC_HWMON_TEMP_ATTR_CRIT		= 2,
	IONIC_HWMON_TEMP_ATTR_CRIT_ALARM	= 3,
};

/**
 * enum ionic_hwmon_voltage_attr - Voltage sensor attributes
 * @IONIC_HWMON_VOLTAGE_ATTR_INPUT: Voltage input reading
 * @IONIC_HWMON_VOLTAGE_ATTR_LABEL: Voltage sensor label
 */
enum ionic_hwmon_voltage_attr {
	IONIC_HWMON_VOLTAGE_ATTR_INPUT	= 0,
	IONIC_HWMON_VOLTAGE_ATTR_LABEL	= 1,
};

/**
 * enum ionic_hwmon_power_attr - Power sensor attributes
 * @IONIC_HWMON_POWER_ATTR_INPUT: Power input reading
 * @IONIC_HWMON_POWER_ATTR_LABEL: Power sensor label
 * @IONIC_HWMON_POWER_ATTR_MAX:   Maximum power threshold
 */
enum ionic_hwmon_power_attr {
	IONIC_HWMON_POWER_ATTR_INPUT	= 0,
	IONIC_HWMON_POWER_ATTR_LABEL	= 1,
	IONIC_HWMON_POWER_ATTR_MAX	= 2,
};

/**
 * struct ionic_hwmon_chan_caps - Per-channel capability descriptor
 * @attr_caps:  Attribute bitmask where bit N indicates attribute N is supported
 *              (use sensor-type-specific attribute enums:
 *              ionic_hwmon_temp_attr, ionic_hwmon_voltage_attr,
 *              ionic_hwmon_power_attr)
 * @mode:       Permissions for attribute N, as a single octal rwx digit
 *              (4 = readable).  Zero hides the attribute.  Values above 7
 *              are not a mode and hide the attribute too.
 * @label:      NUL-terminated channel label string
 *
 * Every channel reports at least one attribute that its sensor type defines.
 * Firmware that adds attributes keeps reporting one that older drivers
 * recognise, and uses @mode to control what is visible.
 *
 * The structure needs to be packed since the size of n_channels in the
 * capability buffer layout documented below is variable, so the channels
 * array is not guaranteed to start on a 4 byte boundary.
 */
struct ionic_hwmon_chan_caps {
	__le32	attr_caps;
	__le32	mode[32];
	char	label[32];
} __packed;

#define IONIC_HWMON_MODE_MASK		0x7
#define IONIC_HWMON_MODE_READ		4

/*
 * Hardware monitoring capabilities buffer layout
 *
 * The buffer is a struct ionic_hwmon_caps_hdr followed by the channel
 * descriptors, so where the descriptors start depends on num_types:
 *
 *   struct ionic_hwmon_caps_buf {
 *       __le16  num_types;
 *       __le16  n_channels[num_types];
 *       struct ionic_hwmon_chan_caps channels[total_channels];
 *   };
 *
 * total_channels is the sum of n_channels, and the channels of each sensor
 * type are contiguous, in n_channels order.  A driver reads at most
 * IONIC_HWMON_TYPE_MAX entries of n_channels and ignores the channels of
 * any further types, which follow the ones it knows.
 *
 * num_types and n_channels come from the device, and the completion does not
 * report how much was written, so a parser checks every offset against the
 * resp_buf_len sent in IONIC_CMD_HWMON_GETCAPS.
 */

/**
 * struct ionic_hwmon_caps_hdr - Start of the capabilities buffer
 * @num_types:  Number of sensor types described by firmware
 * @n_channels: Channel count for each sensor type, num_types entries
 */
struct ionic_hwmon_caps_hdr {
	__le16	num_types;
	__le16	n_channels[];
};

struct auxiliary_device;
struct device;

/**
 * ionic_hwmon_get_caps - Read the sensor capabilities from the device
 * @adev:    ionic "hwmon" auxiliary device
 * @max_ver: highest capabilities buffer layout version the caller parses
 * @buf:     set to the capabilities buffer
 * @len:     set to the length of @buf
 * @ver:     set to the layout version of @buf, as reported by the device
 *
 * The buffer belongs to ionic and is overwritten by the next call.
 *
 * Return: zero or negative error status
 */
int ionic_hwmon_get_caps(struct auxiliary_device *adev, u8 max_ver,
			 const void **buf, size_t *len, u8 *ver);

/**
 * ionic_hwmon_get_attr - Read one sensor attribute from the device
 * @adev:    ionic "hwmon" auxiliary device
 * @type:    sensor type from enum ionic_hwmon_sensor_type
 * @channel: channel index within @type
 * @attr:    attribute ID from the attribute enum for @type
 * @val:     set to the attribute value, in the units documented for
 *           struct ionic_hwmon_getattr_comp
 *
 * Return: zero or negative error status
 */
int ionic_hwmon_get_attr(struct auxiliary_device *adev, u8 type, u8 channel,
			 u8 attr, u32 *val);

/**
 * ionic_hwmon_dev - Device to parent the hwmon device to
 * @adev: ionic "hwmon" auxiliary device
 *
 * Return: the PCI function's device
 */
struct device *ionic_hwmon_dev(struct auxiliary_device *adev);

#endif /* _LINUX_IONIC_HWMON_H_ */
