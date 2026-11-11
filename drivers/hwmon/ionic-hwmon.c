// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2026 Advanced Micro Devices, Inc */

#include <linux/auxiliary_bus.h>
#include <linux/ctype.h>
#include <linux/hwmon.h>
#include <linux/ionic/ionic_hwmon.h>
#include <linux/module.h>

struct ionic_hwmon {
	struct auxiliary_device *adev;
	struct device *dev;
	struct hwmon_chip_info chip_info;
	struct hwmon_channel_info chan_info[IONIC_HWMON_TYPE_MAX];
	const struct hwmon_channel_info *info[IONIC_HWMON_TYPE_MAX + 1];
	u16 n_channels[IONIC_HWMON_TYPE_MAX];
	u16 total_channels;
	struct ionic_hwmon_chan_caps channels[] __counted_by(total_channels);
};

static const enum hwmon_sensor_types
ionic_hwmon_linux_type[IONIC_HWMON_TYPE_MAX] = {
	[IONIC_HWMON_TYPE_TEMP]		= hwmon_temp,
	[IONIC_HWMON_TYPE_VOLTAGE]	= hwmon_in,
	[IONIC_HWMON_TYPE_POWER]	= hwmon_power,
	[IONIC_HWMON_TYPE_CHIP]		= hwmon_chip,
	[IONIC_HWMON_TYPE_CURRENT]	= hwmon_curr,
	[IONIC_HWMON_TYPE_ENERGY]	= hwmon_energy,
	[IONIC_HWMON_TYPE_HUMIDITY]	= hwmon_humidity,
	[IONIC_HWMON_TYPE_FAN]		= hwmon_fan,
	[IONIC_HWMON_TYPE_PWM]		= hwmon_pwm,
	[IONIC_HWMON_TYPE_INTRUSION]	= hwmon_intrusion,
};

static const char * const ionic_hwmon_type_name[IONIC_HWMON_TYPE_MAX] = {
	[IONIC_HWMON_TYPE_TEMP]		= "temp",
	[IONIC_HWMON_TYPE_VOLTAGE]	= "in",
	[IONIC_HWMON_TYPE_POWER]	= "power",
	[IONIC_HWMON_TYPE_CHIP]		= "chip",
	[IONIC_HWMON_TYPE_CURRENT]	= "curr",
	[IONIC_HWMON_TYPE_ENERGY]	= "energy",
	[IONIC_HWMON_TYPE_HUMIDITY]	= "humidity",
	[IONIC_HWMON_TYPE_FAN]		= "fan",
	[IONIC_HWMON_TYPE_PWM]		= "pwm",
	[IONIC_HWMON_TYPE_INTRUSION]	= "intrusion",
};

static unsigned int ionic_hwmon_sysfs_channel(unsigned int ionic_type,
					      unsigned int channel)
{
	enum hwmon_sensor_types type = ionic_hwmon_linux_type[ionic_type];

	if (type == hwmon_in || type == hwmon_intrusion)
		return channel;

	return channel + 1;
}

static unsigned int ionic_hwmon_chan_idx(const struct ionic_hwmon *hwmon,
					 enum hwmon_sensor_types type,
					 int channel)
{
	unsigned int i, start = 0;

	for (i = 0; i < IONIC_HWMON_TYPE_MAX; i++) {
		if (ionic_hwmon_linux_type[i] == type)
			break;
		start += hwmon->n_channels[i];
	}

	return start + channel;
}

struct ionic_hwmon_attr {
	u8 ionic;
	u8 hwmon;
};

struct ionic_hwmon_attr_map {
	const struct ionic_hwmon_attr *attrs;
	unsigned int n_attrs;
};

static const struct ionic_hwmon_attr ionic_hwmon_temp_attr_map[] = {
	{ IONIC_HWMON_TEMP_ATTR_INPUT,		hwmon_temp_input },
	{ IONIC_HWMON_TEMP_ATTR_LABEL,		hwmon_temp_label },
	{ IONIC_HWMON_TEMP_ATTR_CRIT,		hwmon_temp_crit },
	{ IONIC_HWMON_TEMP_ATTR_CRIT_ALARM,	hwmon_temp_crit_alarm },
};

static const struct ionic_hwmon_attr ionic_hwmon_voltage_attr_map[] = {
	{ IONIC_HWMON_VOLTAGE_ATTR_INPUT,	hwmon_in_input },
	{ IONIC_HWMON_VOLTAGE_ATTR_LABEL,	hwmon_in_label },
};

static const struct ionic_hwmon_attr ionic_hwmon_power_attr_map[] = {
	{ IONIC_HWMON_POWER_ATTR_INPUT,		hwmon_power_input },
	{ IONIC_HWMON_POWER_ATTR_LABEL,		hwmon_power_label },
	{ IONIC_HWMON_POWER_ATTR_MAX,		hwmon_power_max },
};

static const struct ionic_hwmon_attr_map ionic_hwmon_attr_maps[] = {
	[hwmon_temp]	= { ionic_hwmon_temp_attr_map,
			    ARRAY_SIZE(ionic_hwmon_temp_attr_map) },
	[hwmon_in]	= { ionic_hwmon_voltage_attr_map,
			    ARRAY_SIZE(ionic_hwmon_voltage_attr_map) },
	[hwmon_power]	= { ionic_hwmon_power_attr_map,
			    ARRAY_SIZE(ionic_hwmon_power_attr_map) },
};

static const struct ionic_hwmon_attr_map *
ionic_hwmon_attr_map(enum hwmon_sensor_types type)
{
	if (type >= ARRAY_SIZE(ionic_hwmon_attr_maps) ||
	    !ionic_hwmon_attr_maps[type].n_attrs)
		return NULL;

	return &ionic_hwmon_attr_maps[type];
}

static int ionic_hwmon_attr_to_ionic_attr(enum hwmon_sensor_types type,
					  u32 attr, unsigned int *ionic_attr)
{
	const struct ionic_hwmon_attr_map *map = ionic_hwmon_attr_map(type);
	unsigned int i;

	if (!map)
		return -EOPNOTSUPP;

	for (i = 0; i < map->n_attrs; i++) {
		if (map->attrs[i].hwmon == attr) {
			*ionic_attr = map->attrs[i].ionic;
			return 0;
		}
	}

	return -EOPNOTSUPP;
}

static void ionic_hwmon_check_labels(struct device *dev,
				     struct ionic_hwmon *hwmon)
{
	unsigned int type, j, idx = 0;
	char *label, *p;

	for (type = 0; type < IONIC_HWMON_TYPE_MAX; type++) {
		for (j = 0; j < hwmon->n_channels[type]; j++, idx++) {
			label = hwmon->channels[idx].label;
			label[sizeof(hwmon->channels[idx].label) - 1] = '\0';

			for (p = label; *p; p++) {
				if (isascii(*p) && isprint(*p))
					continue;
				dev_warn(dev,
					 "%s%u_label is not printable, ignoring it\n",
					 ionic_hwmon_type_name[type],
					 ionic_hwmon_sysfs_channel(type, j));
				label[0] = '\0';
				break;
			}
		}
	}
}

static struct ionic_hwmon *ionic_hwmon_create(struct device *dev,
					      struct auxiliary_device *adev,
					      const void *buf, size_t buf_len)
{
	unsigned int channels_offset, num_types, total_channels, i;
	const struct ionic_hwmon_caps_hdr *hdr = buf;
	u16 chan_counts[IONIC_HWMON_TYPE_MAX];
	struct ionic_hwmon *hwmon;
	u16 fw_num_types;

	if (buf_len < sizeof(*hdr))
		return ERR_PTR(-EINVAL);

	fw_num_types = le16_to_cpu(READ_ONCE(hdr->num_types));

	channels_offset = struct_size(hdr, n_channels, fw_num_types);
	if (channels_offset > buf_len)
		return ERR_PTR(-EINVAL);

	num_types = min_t(unsigned int, fw_num_types, IONIC_HWMON_TYPE_MAX);

	total_channels = 0;
	for (i = 0; i < num_types; i++) {
		chan_counts[i] = le16_to_cpu(READ_ONCE(hdr->n_channels[i]));
		total_channels += chan_counts[i];
	}

	if (total_channels >
	    (buf_len - channels_offset) / sizeof(struct ionic_hwmon_chan_caps))
		return ERR_PTR(-EINVAL);

	hwmon = devm_kzalloc(dev, struct_size(hwmon, channels, total_channels),
			     GFP_KERNEL);
	if (!hwmon)
		return ERR_PTR(-ENOMEM);

	hwmon->adev = adev;

	for (i = 0; i < num_types; i++)
		hwmon->n_channels[i] = chan_counts[i];

	hwmon->total_channels = total_channels;
	memcpy(hwmon->channels, buf + channels_offset,
	       flex_array_size(hwmon, channels, total_channels));

	ionic_hwmon_check_labels(dev, hwmon);

	return hwmon;
}

static umode_t ionic_hwmon_file_mode(u32 mode)
{
	if ((mode & ~IONIC_HWMON_MODE_MASK) || !(mode & IONIC_HWMON_MODE_READ))
		return 0;

	/* No attribute is writable: hwmon refuses to register the device if
	 * any attribute has write permission without a .write op.
	 */
	return 0444;
}

static bool ionic_hwmon_attr_is_label(enum hwmon_sensor_types type, u32 attr)
{
	return (type == hwmon_temp && attr == hwmon_temp_label) ||
	       (type == hwmon_in && attr == hwmon_in_label) ||
	       (type == hwmon_power && attr == hwmon_power_label);
}

static umode_t ionic_hwmon_is_visible(const void *data,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	const struct ionic_hwmon_chan_caps *chan_caps;
	const struct ionic_hwmon *hwmon = data;
	unsigned int idx, ionic_attr;

	if (ionic_hwmon_attr_to_ionic_attr(type, attr, &ionic_attr))
		return 0;

	idx = ionic_hwmon_chan_idx(hwmon, type, channel);
	chan_caps = &hwmon->channels[idx];

	if (ionic_hwmon_attr_is_label(type, attr) && !chan_caps->label[0])
		return 0;

	return ionic_hwmon_file_mode(le32_to_cpu(chan_caps->mode[ionic_attr]));
}

static int ionic_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct ionic_hwmon *hwmon = dev_get_drvdata(dev);
	enum ionic_hwmon_sensor_type sensor_type;
	unsigned int ionic_attr;
	u32 data;
	int err;

	err = ionic_hwmon_attr_to_ionic_attr(type, attr, &ionic_attr);
	if (err)
		return err;

	switch (type) {
	case hwmon_temp:
		sensor_type = IONIC_HWMON_TYPE_TEMP;
		break;
	case hwmon_in:
		sensor_type = IONIC_HWMON_TYPE_VOLTAGE;
		break;
	case hwmon_power:
		sensor_type = IONIC_HWMON_TYPE_POWER;
		break;
	default:
		return -EOPNOTSUPP;
	}

	err = ionic_hwmon_get_attr(hwmon->adev, sensor_type, channel,
				   ionic_attr, &data);
	if (err)
		return err;

	/* Temperatures are signed millidegrees Celsius and can read below
	 * zero, so convert them to s32 rather than doing a zero extension.
	 */
	if (type == hwmon_temp) {
		if (attr == hwmon_temp_crit_alarm)
			*val = !!data;
		else
			*val = (s32)data;
	} else {
		*val = data;
	}

	/* The device reports power in milliwatts, sysfs wants microwatts */
	if (type == hwmon_power)
		*val *= 1000;

	return 0;
}

static int ionic_hwmon_read_label(struct device *dev,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel, const char **str)
{
	struct ionic_hwmon *hwmon = dev_get_drvdata(dev);
	unsigned int idx;

	idx = ionic_hwmon_chan_idx(hwmon, type, channel);
	*str = hwmon->channels[idx].label;

	return 0;
}

static const struct hwmon_ops ionic_hwmon_ops = {
	.is_visible = ionic_hwmon_is_visible,
	.read = ionic_hwmon_read,
	.read_string = ionic_hwmon_read_label,
};

static u32
ionic_hwmon_attr_caps_to_config(u32 attr_caps,
				const struct ionic_hwmon_attr_map *map)
{
	unsigned int i;
	u32 config = 0;

	for (i = 0; i < map->n_attrs; i++) {
		if (attr_caps & BIT(map->attrs[i].ionic))
			config |= BIT(map->attrs[i].hwmon);
	}

	return config;
}

static u32 *
ionic_hwmon_type_config(struct device *dev, unsigned int ionic_type,
			const struct ionic_hwmon_attr_map *map,
			const struct ionic_hwmon_chan_caps *chan_caps,
			unsigned int n_channels)
{
	unsigned int j;
	u32 *config;

	config = devm_kcalloc(dev, n_channels + 1, sizeof(*config), GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	for (j = 0; j < n_channels; j++) {
		u32 attr_caps = le32_to_cpu(chan_caps[j].attr_caps);

		config[j] = ionic_hwmon_attr_caps_to_config(attr_caps, map);
		if (!config[j]) {
			dev_warn(dev,
				 "%s%u has no usable attributes, ignoring all %s channels\n",
				 ionic_hwmon_type_name[ionic_type],
				 ionic_hwmon_sysfs_channel(ionic_type, j),
				 ionic_hwmon_type_name[ionic_type]);
			devm_kfree(dev, config);
			return NULL;
		}
	}

	return config;
}

static int ionic_hwmon_build_channel_info(struct device *dev,
					  struct ionic_hwmon *hwmon)
{
	unsigned int i, n_types, chan_idx;

	chan_idx = 0;
	n_types = 0;

	for (i = 0; i < IONIC_HWMON_TYPE_MAX; i++) {
		enum hwmon_sensor_types type = ionic_hwmon_linux_type[i];
		unsigned int n_channels = hwmon->n_channels[i];
		const struct ionic_hwmon_chan_caps *chan_caps;
		const struct ionic_hwmon_attr_map *map;
		struct hwmon_channel_info *chan_info;
		u32 *config;

		if (!n_channels)
			continue;

		chan_caps = &hwmon->channels[chan_idx];
		chan_idx += n_channels;

		map = ionic_hwmon_attr_map(type);
		if (!map)
			continue;

		config = ionic_hwmon_type_config(dev, i, map, chan_caps,
						 n_channels);
		if (IS_ERR(config))
			return PTR_ERR(config);

		if (!config)
			continue;

		chan_info = &hwmon->chan_info[n_types];
		chan_info->type = type;
		chan_info->config = config;
		hwmon->info[n_types++] = chan_info;
	}

	return n_types ? 0 : -ENODEV;
}

static int ionic_hwmon_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct ionic_hwmon *hwmon;
	const void *buf;
	size_t len;
	int err;
	u8 ver;

	err = ionic_hwmon_get_caps(adev, IONIC_HWMON_CAPS_VERSION_0, &buf,
				   &len, &ver);
	if (err)
		return dev_err_probe(dev, err,
				     "cannot read sensor capabilities\n");

	if (ver != IONIC_HWMON_CAPS_VERSION_0)
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "unsupported capabilities version %u\n",
				     ver);

	hwmon = ionic_hwmon_create(dev, adev, buf, len);
	if (IS_ERR(hwmon))
		return dev_err_probe(dev, PTR_ERR(hwmon),
				     "invalid sensor capabilities\n");

	err = ionic_hwmon_build_channel_info(dev, hwmon);
	if (err == -ENODEV) {
		dev_info(dev, "no sensors this driver supports\n");
		return err;
	}
	if (err)
		return dev_err_probe(dev, err,
				     "cannot build sensor configuration\n");

	hwmon->chip_info.ops = &ionic_hwmon_ops;
	hwmon->chip_info.info = hwmon->info;

	/* The hwmon device is parented to the PCI function, not to adev, so
	 * userspace finds the sensors by PCI endpoint.  Its chip_info and
	 * config arrays are devres on adev, so it is unregistered in remove
	 * rather than through devm.
	 */
	hwmon->dev = hwmon_device_register_with_info(ionic_hwmon_dev(adev),
						     "ionic", hwmon,
						     &hwmon->chip_info, NULL);
	if (IS_ERR(hwmon->dev))
		return dev_err_probe(dev, PTR_ERR(hwmon->dev),
				     "cannot register hwmon device\n");

	auxiliary_set_drvdata(adev, hwmon);

	return 0;
}

static void ionic_hwmon_remove(struct auxiliary_device *adev)
{
	struct ionic_hwmon *hwmon = auxiliary_get_drvdata(adev);

	/* Unregister before the devres on adev frees chip_info and config */
	hwmon_device_unregister(hwmon->dev);
}

static const struct auxiliary_device_id ionic_hwmon_id_table[] = {
	{ .name = "ionic.hwmon", },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, ionic_hwmon_id_table);

static struct auxiliary_driver ionic_hwmon_driver = {
	.name = "hwmon",
	.probe = ionic_hwmon_probe,
	.remove = ionic_hwmon_remove,
	.id_table = ionic_hwmon_id_table,
};
module_auxiliary_driver(ionic_hwmon_driver);

MODULE_AUTHOR("Arun Kumar Prakash <ArunKumar.Prakash@amd.com>");
MODULE_DESCRIPTION("AMD Pensando ionic hardware monitoring driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("NET_IONIC");
