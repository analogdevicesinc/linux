// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2026 Nexthop Systems Inc.
 *
 * Driver for the TDK FS160x family of uPOL DC-DC converters with integrated
 * inductor. Supported models: FS1603, FS1604, FS1606, FS1006.
 *
 * The parts expose voltage, current and temperature telemetry over a
 * proprietary I2C register map. They are not PMBus devices, and their
 * register numbers actively collide with PMBus command codes -- 0x21 is a
 * status byte here where PMBus defines VOUT_COMMAND -- so a generic PMBus
 * driver must never be bound to them.
 *
 * Read-only. VOUT_SET (0x12/0x13) is an atomic write-protected pair: writing
 * the high byte arms an update that takes effect only once the low byte is
 * written. A partial write leaves the part armed, after which it acknowledges
 * but silently ignores every register write until its input power is cycled,
 * so this driver only reads it, to scale the over-voltage threshold.
 *
 * On a board that raises the rail with a feedback divider, VOUT still reports
 * the rail but VOUT_SET holds the feedback-pin target, so the over-voltage
 * threshold is scaled by the divider ratio, measured at probe from VOUT and
 * the Fb register (0x22). TDK notes that current reporting does not apply in
 * that configuration.
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define FS1603_REG_PVIN			0x0C
#define FS1603_REG_VOUT			0x0D
#define FS1603_REG_IOUT			0x0E
#define FS1603_REG_TEMP			0x0F
#define FS1603_REG_VOUT_SET_HI		0x12
#define FS1603_REG_VOUT_SET_LO		0x13
#define FS1603_REG_OV_THRESHOLD		0x17
#define FS1603_REG_OT_THRESHOLD		0x19
#define FS1603_REG_IOUT_GAIN		0x1A
#define FS1603_REG_STATUS		0x21
#define FS1603_REG_FB			0x22

#define FS1603_VOUT_SET_HI_MASK		BIT(0)
#define FS1603_OV_THRESHOLD_MASK	GENMASK(1, 0)
#define FS1603_OT_THRESHOLD_MASK	GENMASK(1, 0)
#define FS1603_IOUT_GAIN_MASK		GENMASK(7, 2)

/* Bits in FS1603_REG_STATUS. */
#define FS1603_STATUS_OVER_VOLTAGE	BIT(6)
#define FS1603_STATUS_OVER_CURRENT	BIT(5)
#define FS1603_STATUS_OVER_TEMP		BIT(4)

/* OV_Threshold[1:0] in percent of programmed VOUT; OT_Threshold[1:0] in degC. */
static const unsigned int fs1603_ov_threshold_pct[] = { 105, 110, 115, 120 };
static const unsigned int fs1603_ot_threshold_c[] = { 75, 85, 125, 145 };

/*
 * Voltage telemetry is a linear step and base, both fixed per variant.
 * Current is raw/32 A except on the FS1606-0600, which applies a calibration
 * correction derived from FS1603_REG_IOUT_GAIN and the output voltage.
 */
struct fs1603_info {
	unsigned int vout_step_mv;
	unsigned int vout_base_mv;
	bool iout_gain_corrected;
};

enum fs1603_model {
	fs1603,
	fs1606,
	fs1606_0600,
};

/*
 * The FS1603, FS1604, FS1006 and FS1606-2500 share the 20mV/600mV encoding.
 * The other FS1606 variants report at 10mV/300mV.
 */
static const struct fs1603_info fs1603_models[] = {
	[fs1603]      = { .vout_step_mv = 20, .vout_base_mv = 600 },
	[fs1606]      = { .vout_step_mv = 10, .vout_base_mv = 300 },
	[fs1606_0600] = { .vout_step_mv = 10, .vout_base_mv = 300,
			  .iout_gain_corrected = true },
};

/*
 * Every register is a byte, and every read is live telemetry, so nothing is
 * cached. regmap is used for the uniform accessors and the debugfs dump.
 */
static const struct regmap_config fs1603_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FS1603_REG_FB,
	.cache_type = REGCACHE_NONE,
};

struct fs1603_data {
	struct regmap *regmap;
	const struct fs1603_info *info;
	u32 fb_ratio_num;
	u32 fb_ratio_den;
};

static int fs1603_read_vout_mv(struct fs1603_data *data, long *val)
{
	unsigned int raw;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_VOUT, &raw);
	if (err)
		return err;

	*val = raw * data->info->vout_step_mv + data->info->vout_base_mv;
	return 0;
}

/*
 * Output current in mA.
 *
 * Every variant but the FS1606-0600 reports raw/32 A directly. The -0600
 * additionally applies, with G taken from FS1603_REG_IOUT_GAIN bits [7:2]:
 *
 *   I = raw/32 - (9.05 - 0.24 * G) * Vout - 0.356 * G + 13.1
 *
 * Scaled to milliamps and millivolts so it stays in integer arithmetic:
 *
 *   I_mA = raw * 125 / 4
 *          - ((9050 - 240 * G) * Vout_mV) / 1000
 *          - 356 * G
 *          + 13100
 */
static int fs1603_read_iout_ma(struct fs1603_data *data, long *val)
{
	unsigned int raw, gain;
	long vout_mv;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_IOUT, &raw);
	if (err)
		return err;

	*val = raw * 125 / 4;

	if (!data->info->iout_gain_corrected)
		return 0;

	err = regmap_read(data->regmap, FS1603_REG_IOUT_GAIN, &gain);
	if (err)
		return err;
	gain = FIELD_GET(FS1603_IOUT_GAIN_MASK, gain);

	err = fs1603_read_vout_mv(data, &vout_mv);
	if (err)
		return err;

	*val -= ((9050 - 240 * (long)gain) * vout_mv) / 1000;
	*val -= 356 * (long)gain;
	*val += 13100;

	if (*val < 0)
		*val = 0;

	return 0;
}

/*
 * The 9-bit VOUT_SET code steps at half the telemetry resolution from a base
 * of 80 steps.
 */
static int fs1603_read_vout_set_mv(struct fs1603_data *data, long *val)
{
	unsigned int hi, lo;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_VOUT_SET_HI, &hi);
	if (err)
		return err;
	err = regmap_read(data->regmap, FS1603_REG_VOUT_SET_LO, &lo);
	if (err)
		return err;

	*val = ((FIELD_GET(FS1603_VOUT_SET_HI_MASK, hi) << 8 | lo) + 80) *
	       data->info->vout_step_mv / 2;
	return 0;
}

/* Fb has no documented encoding; it decodes with the VOUT step and base. */
static int fs1603_read_fb_mv(struct fs1603_data *data, long *val)
{
	unsigned int raw;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_FB, &raw);
	if (err)
		return err;

	*val = raw * data->info->vout_step_mv + data->info->vout_base_mv;
	return 0;
}

/*
 * A board that raises the rail with a feedback divider leaves VOUT_SET at the
 * feedback target, and Fb then reads well below VOUT. The divider is fixed in
 * hardware, so measure its ratio once. Without a divider Fb reads the output
 * itself, and the ratio stays one so the limit does not follow ADC noise.
 */
static int fs1603_measure_fb_ratio(struct fs1603_data *data)
{
	long vout_mv, fb_mv;
	int err;

	data->fb_ratio_num = 1;
	data->fb_ratio_den = 1;

	err = fs1603_read_vout_mv(data, &vout_mv);
	if (err)
		return err;
	err = fs1603_read_fb_mv(data, &fb_mv);
	if (err)
		return err;

	if (fb_mv > 0 && vout_mv > fb_mv + 4 * data->info->vout_step_mv) {
		data->fb_ratio_num = vout_mv;
		data->fb_ratio_den = fb_mv;
	}
	return 0;
}

/*
 * OVP trips at OV_Threshold percent of the VOUT_SET target, measured at the
 * feedback pin. Scale it by the feedback divider ratio to report it in the
 * same rail domain as in1_input.
 */
static int fs1603_read_vout_crit_mv(struct fs1603_data *data, long *val)
{
	unsigned int raw;
	long vout_set_mv;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_OV_THRESHOLD, &raw);
	if (err)
		return err;
	err = fs1603_read_vout_set_mv(data, &vout_set_mv);
	if (err)
		return err;

	raw = FIELD_GET(FS1603_OV_THRESHOLD_MASK, raw);
	*val = DIV_ROUND_CLOSEST_ULL((u64)fs1603_ov_threshold_pct[raw] *
				     vout_set_mv * data->fb_ratio_num,
				     100 * data->fb_ratio_den);
	return 0;
}

static int fs1603_read_status_bit(struct fs1603_data *data, u8 mask,
				  long *val)
{
	unsigned int status;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_STATUS, &status);
	if (err)
		return err;

	*val = !!(status & mask);
	return 0;
}

static int fs1603_read_in(struct fs1603_data *data, u32 attr, int channel,
			  long *val)
{
	unsigned int raw;
	int err;

	switch (attr) {
	case hwmon_in_input:
		if (channel == 0) {
			err = regmap_read(data->regmap, FS1603_REG_PVIN, &raw);
			if (err)
				return err;
			/* PVIN reports in 1/16 V steps. */
			*val = raw * 1000 / 16;
			return 0;
		}
		return fs1603_read_vout_mv(data, val);
	case hwmon_in_crit:
		return fs1603_read_vout_crit_mv(data, val);
	case hwmon_in_alarm:
		return fs1603_read_status_bit(data, FS1603_STATUS_OVER_VOLTAGE,
					      val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fs1603_read_curr(struct fs1603_data *data, u32 attr, long *val)
{
	switch (attr) {
	case hwmon_curr_input:
		return fs1603_read_iout_ma(data, val);
	case hwmon_curr_alarm:
		return fs1603_read_status_bit(data, FS1603_STATUS_OVER_CURRENT,
					      val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fs1603_read_temp_crit_mc(struct fs1603_data *data, long *val)
{
	unsigned int raw;
	int err;

	err = regmap_read(data->regmap, FS1603_REG_OT_THRESHOLD, &raw);
	if (err)
		return err;

	*val = fs1603_ot_threshold_c[FIELD_GET(FS1603_OT_THRESHOLD_MASK, raw)] * 1000;
	return 0;
}

static int fs1603_read_temp(struct fs1603_data *data, u32 attr, long *val)
{
	unsigned int raw;
	int err;

	switch (attr) {
	case hwmon_temp_input:
		err = regmap_read(data->regmap, FS1603_REG_TEMP, &raw);
		if (err)
			return err;
		/* TEMP_REPORT is degC directly: 1 degC per LSB, no offset. */
		*val = raw * 1000;
		return 0;
	case hwmon_temp_crit:
		return fs1603_read_temp_crit_mc(data, val);
	case hwmon_temp_alarm:
		return fs1603_read_status_bit(data, FS1603_STATUS_OVER_TEMP,
					      val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fs1603_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	struct fs1603_data *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_in:
		return fs1603_read_in(data, attr, channel, val);
	case hwmon_curr:
		return fs1603_read_curr(data, attr, val);
	case hwmon_temp:
		return fs1603_read_temp(data, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int fs1603_read_string(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, const char **str)
{
	if (type != hwmon_in || attr != hwmon_in_label)
		return -EOPNOTSUPP;

	*str = channel == 0 ? "vin" : "vout";
	return 0;
}

static umode_t fs1603_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
			return 0444;
		case hwmon_in_crit:
		case hwmon_in_alarm:
			/* OVP and its alarm describe the regulated output only. */
			return channel == 1 ? 0444 : 0;
		default:
			return 0;
		}
	case hwmon_curr:
	case hwmon_temp:
		return 0444;
	default:
		return 0;
	}
}

static const struct hwmon_ops fs1603_ops = {
	.is_visible = fs1603_is_visible,
	.read = fs1603_read,
	.read_string = fs1603_read_string,
};

static const struct hwmon_channel_info *fs1603_channel_info[] = {
	HWMON_CHANNEL_INFO(in, HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_CRIT |
			   HWMON_I_ALARM),
	HWMON_CHANNEL_INFO(curr, HWMON_C_INPUT | HWMON_C_ALARM),
	HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT | HWMON_T_CRIT | HWMON_T_ALARM),
	NULL
};

static const struct hwmon_chip_info fs1603_chip_info = {
	.ops = &fs1603_ops,
	.info = fs1603_channel_info,
};

static int fs1603_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	char *name;
	struct fs1603_data *data;
	struct device *hwmon;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(dev, -ENODEV,
				     "SMBus byte data not supported\n");

	name = devm_hwmon_sanitize_name(dev, client->name);
	if (IS_ERR(name))
		return PTR_ERR(name);

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = devm_regmap_init_i2c(client, &fs1603_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap),
				     "failed to allocate regmap\n");

	data->info = i2c_get_match_data(client);
	if (!data->info)
		return dev_err_probe(dev, -ENODEV, "no match data for %s\n",
				     client->name);

	err = fs1603_measure_fb_ratio(data);
	if (err)
		return dev_err_probe(dev, err,
				     "failed to read the feedback ratio\n");

	hwmon = devm_hwmon_device_register_with_info(dev, name, data,
						     &fs1603_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon);
}

static const struct i2c_device_id fs1603_id[] = {
	{ .name = "fs1603",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1603] },
	{ .name = "fs1604",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1603] },
	{ .name = "fs1606",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1606] },
	{ .name = "fs1606_0600",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1606_0600] },
	{ .name = "fs1606_2500",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1603] },
	{ .name = "fs1006_5000",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1603] },
	{ .name = "fs1006_3300",
	  .driver_data = (kernel_ulong_t)&fs1603_models[fs1603] },
	{}
};
MODULE_DEVICE_TABLE(i2c, fs1603_id);

static const struct of_device_id fs1603_of_match[] = {
	{ .compatible = "tdk,fs1603", .data = &fs1603_models[fs1603] },
	{ .compatible = "tdk,fs1606", .data = &fs1603_models[fs1606] },
	{ .compatible = "tdk,fs1606-0600",
	  .data = &fs1603_models[fs1606_0600] },
	{}
};
MODULE_DEVICE_TABLE(of, fs1603_of_match);

static struct i2c_driver fs1603_driver = {
	.driver = {
		.name	= "fs1603",
		.of_match_table = fs1603_of_match,
	},
	.probe = fs1603_probe,
	.id_table = fs1603_id,
};
module_i2c_driver(fs1603_driver);

MODULE_AUTHOR("Kevin Jia <kjia@nexthop.ai>");
MODULE_DESCRIPTION("TDK FS160x uPOL DC-DC converter telemetry driver");
MODULE_LICENSE("GPL");
