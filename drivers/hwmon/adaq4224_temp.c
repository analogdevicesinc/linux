// SPDX-License-Identifier: GPL-2.0
/*
 * adaq4224_temp.c - Support for ADAQ4224 chipset thermal sensor
 *
 * Copyright (c) 2024 Radu Sabau <radu.sabau@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include <linux/of_device.h>

enum {
	ADAQ4224_TEMP_DEBUGFS_PEC_ENABLE = 0,
	ADAQ4224_TEMP_DEBUGFS_TIMEOUT,
	ADAQ4224_TEMP_DEBUGFS_RESOLUTION,
	ADAQ4224_TEMP_DEBUGFS_ALARM_POLARITY,
	ADAQ4224_TEMP_DEBUGFS_COMP_INT,
	ADAQ4224_TEMP_DEBUGFS_FAULT_QUEUE,
	ADAQ4224_TEMP_DEBUGFS_PEC_ERROR,
	ADAQ4224_TEMP_DEBUGFS_NUM_ENTRIES
};

#define ADAQ4224_TEMP_T_REG				0x0
#define ADAQ4224_TEMP_CONFIGURATION_REG			0x2
#define ADAQ4224_TEMP_TH_REG				0x4
#define ADAQ4224_TEMP_TL_REG				0x6
#define ADAQ4224_TEMP_TH_HYST_REG			0x8
#define ADAQ4224_TEMP_TL_HYST_REG			0xA

#define ADAQ4224_TEMP_CONFIGURATION_1SHOT_MASK		BIT(0)
#define ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK	GENMASK(3, 1)
#define ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK		BIT(4)
#define ADAQ4224_TEMP_CONFIGURATION_TIMEOUT_MASK	BIT(5)
#define ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK	GENMASK(7, 6)
#define ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK	BIT(8)
#define ADAQ4224_TEMP_CONFIGURATION_COMP_INT_MASK	BIT(9)
#define ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK		GENMASK(11, 10)
#define ADAQ4224_TEMP_CONFIGURATION_PEC_ERR_MASK	BIT(13)
#define ADAQ4224_TEMP_CONFIGURATION_U_TEMP_STAT_MASK	BIT(14)
#define ADAQ4224_TEMP_CONFIGURATION_O_TEMP_STAT_MASK	BIT(15)

#define ADAQ4224_TEMP_ALRM_POL_LOW			0x0
#define ADAQ4224_TEMP_ALRM_POL_HIGH			0x1
#define ADAQ4224_TEMP_FLT_Q_1				0x0
#define ADAQ4224_TEMP_FLT_Q_4				0x2

#define ADAQ4224_TEMP_8_BIT_CNV_TIME			9
#define ADAQ4224_TEMP_9_BIT_CNV_TIME			18
#define ADAQ4224_TEMP_10_BIT_CNV_TIME			35
#define ADAQ4224_TEMP_12_BIT_CNV_TIME			140

#define ADAQ4224_TEMP_16_BIT_TO_M_DGR(x)		(sign_extend32(x, 15) * 1000 / 16)
#define ADAQ4224_TEMP_M_DGR_TO_16_BIT(x)		(((x) << 4) / 1000)
#define ADAQ4224_TEMP_DEVICE_ENABLE(x)			((x) ? 0xA : 0x0)

#define DEBUG_FS_DATA_MAX				16

enum adaq4224_temp_cnv {
	ADAQ4224_TEMP_CNV_1_DIV_64_HZ = 1,
	ADAQ4224_TEMP_CNV_1_DIV_32_HZ,
	ADAQ4224_TEMP_CNV_1_DIV_16_HZ,
	ADAQ4224_TEMP_CNV_1_DIV_4_HZ,
	ADAQ4224_TEMP_CNV_1_HZ,
	ADAQ4224_TEMP_CNV_4_HZ,
	ADAQ4224_TEMP_CNV_8_HZ,
};

static const u16 adaq4224_temp_conversions[] = {
	[ADAQ4224_TEMP_CNV_1_DIV_64_HZ] = 64000,
	[ADAQ4224_TEMP_CNV_1_DIV_32_HZ] = 32000,
	[ADAQ4224_TEMP_CNV_1_DIV_16_HZ] = 16000,
	[ADAQ4224_TEMP_CNV_1_DIV_4_HZ] = 4000,
	[ADAQ4224_TEMP_CNV_1_HZ] = 1000,
	[ADAQ4224_TEMP_CNV_4_HZ] = 250,
	[ADAQ4224_TEMP_CNV_8_HZ] = 125,
};

enum adaq4224_temp_resolution {
	ADAQ4224_TEMP_RES_8_BIT = 0,
	ADAQ4224_TEMP_RES_9_BIT,
	ADAQ4224_TEMP_RES_10_BIT,
	ADAQ4224_TEMP_RES_12_BIT,
};

static const u16 adaq4224_temp_resolutions[] = {
	[ADAQ4224_TEMP_RES_8_BIT] = 1000,
	[ADAQ4224_TEMP_RES_9_BIT] = 500,
	[ADAQ4224_TEMP_RES_10_BIT] = 250,
	[ADAQ4224_TEMP_RES_12_BIT] = 62,
};

static const u16 adaq4224_temp_conv_times[] = {
	[ADAQ4224_TEMP_RES_8_BIT] = ADAQ4224_TEMP_8_BIT_CNV_TIME,
	[ADAQ4224_TEMP_RES_9_BIT] = ADAQ4224_TEMP_9_BIT_CNV_TIME,
	[ADAQ4224_TEMP_RES_10_BIT] = ADAQ4224_TEMP_10_BIT_CNV_TIME,
	[ADAQ4224_TEMP_RES_12_BIT] = ADAQ4224_TEMP_12_BIT_CNV_TIME,
};

struct adaq4224_temp_debugfs_data {
	int debugfs_entries[ADAQ4224_TEMP_DEBUGFS_NUM_ENTRIES];
};

struct adaq4224_temp_state {
	/*
	 * Prevent simultaneous access to the i2c client.
	 */
	struct adaq4224_temp_debugfs_data psu;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;
	unsigned int update_interval;
	unsigned int resolution;
	unsigned int test;
	bool enable;
};

static const struct regmap_config adaq4224_temp_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xA,
};

static int adaq4224_temp_reg_write(struct adaq4224_temp_state *st,
				   unsigned int reg, unsigned int val)
{
	unsigned int cfg;
	int ret;

	ret = regmap_write(st->regmap, reg, val);
	if (ret)
		return ret;

	if (st->client->flags & I2C_CLIENT_PEC) {
		ret = regmap_read(st->regmap, ADAQ4224_TEMP_CONFIGURATION_REG,
				  &cfg);
		if (ret)
			return ret;

		if (cfg & ADAQ4224_TEMP_CONFIGURATION_PEC_ERR_MASK)
			return -ECOMM;
	}

	return 0;
}

static int adaq4224_temp_update_bits(struct adaq4224_temp_state *st,
				     unsigned int reg, unsigned int mask,
				     unsigned int val)
{
	unsigned int tmp = 0;
	int ret;

	ret = regmap_read(st->regmap, reg, &tmp);
	if (ret)
		return ret;

	tmp = (tmp & ~mask) | (val & mask);
	ret = adaq4224_temp_reg_write(st, reg, tmp);

	return ret;
}

static int shutdown_write(struct adaq4224_temp_state *st,
			  unsigned int reg, unsigned int mask,
			  unsigned int val)
{
	unsigned int cfg;
	unsigned int cnv_rate;
	int ret;

	/*
	 * Before the Temperature Threshold Alarm, Alarm Hysteresis Threshold
	 * and Resolution bits from Configuration register are changed over I2C,
	 * the part must be in shutdown mode.
	 *
	 * Mutex is used to ensure, that some other process doesn't change the
	 * configuration register.
	 */
	mutex_lock(&st->lock);

	if (!st->enable) {
		if (!mask)
			ret = adaq4224_temp_reg_write(st, reg, val);
		else
			ret = adaq4224_temp_update_bits(st, reg, mask, val);
		goto unlock;
	}

	ret = regmap_read(st->regmap, ADAQ4224_TEMP_CONFIGURATION_REG, &cfg);
	if (ret)
		goto unlock;

	cnv_rate = ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK & cfg;
	cfg = cfg & ~(ADAQ4224_TEMP_CONFIGURATION_1SHOT_MASK |
		      ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK);
	ret = adaq4224_temp_reg_write(st, ADAQ4224_TEMP_CONFIGURATION_REG, cfg);
	if (ret)
		goto unlock;

	if (!mask)
		ret = adaq4224_temp_reg_write(st, reg, val);
	else
		ret = adaq4224_temp_update_bits(st, reg, mask, val);

	if (ret)
		goto unlock;

	ret = adaq4224_temp_update_bits(st, ADAQ4224_TEMP_CONFIGURATION_REG,
					ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
					cnv_rate);

unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int write_alarm_val(struct adaq4224_temp_state *st, unsigned int reg,
			   long val)
{
	val = ADAQ4224_TEMP_M_DGR_TO_16_BIT(val);

	return shutdown_write(st, reg, 0, val);
}

static umode_t adaq4224_temp_is_visible(const void *state,
					enum hwmon_sensor_types type, u32 attr,
					int channel)
{
	if (type == hwmon_temp) {
		switch (attr) {
		case hwmon_temp_enable:
		case hwmon_temp_max:
		case hwmon_temp_min:
		case hwmon_temp_max_hyst:
		case hwmon_temp_min_hyst:
			return 0644;
		case hwmon_temp_input:
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
			return 0444;
		default:
			return 0;
		}
	} else if (type == hwmon_chip) {
		if (attr == hwmon_chip_update_interval)
			return 0644;
	}

	return 0;
}

static int adaq4224_temp_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct adaq4224_temp_state *st = dev_get_drvdata(dev);
	unsigned int uval;
	int ret = 0;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			ret = regmap_read(st->regmap,
					  ADAQ4224_TEMP_CONFIGURATION_REG, &uval);
			if (ret)
				break;

			uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_1SHOT_MASK |
					 ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
					 uval);
			*val = !!uval;

			break;
		case hwmon_temp_input:
			mutex_lock(&st->lock);

			if (!st->enable) {
				/*
				 * This operation requires mutex protection,
				 * because the chip configuration should not
				 * be changed during the conversion process.
				 */

				ret = adaq4224_temp_update_bits(st,
								ADAQ4224_TEMP_CONFIGURATION_REG,
								ADAQ4224_TEMP_CONFIGURATION_1SHOT_MASK,
								1);
				if (ret) {
					mutex_unlock(&st->lock);
					return ret;
				}
				msleep(adaq4224_temp_conv_times[st->resolution]);
			}

			/*
			 * For 12-bit resolution the conversion time is 140 ms,
			 * thus an additional 15 ms is needed to complete the
			 * conversion: 125 ms + 15 ms = 140 ms
			 */
			if (adaq4224_temp_resolutions[st->resolution] == 12 &&
			    st->update_interval == 125)
				usleep_range(15000, 20000);

			ret = regmap_read(st->regmap, ADAQ4224_TEMP_T_REG, &uval);

			mutex_unlock(&st->lock);

			if (ret)
				break;

			*val = ADAQ4224_TEMP_16_BIT_TO_M_DGR(uval);

			break;
		case hwmon_temp_max:
			ret = regmap_read(st->regmap, ADAQ4224_TEMP_TH_REG, &uval);
			if (ret)
				break;

			*val = ADAQ4224_TEMP_16_BIT_TO_M_DGR(uval);
			break;
		case hwmon_temp_max_hyst:
			ret = regmap_read(st->regmap, ADAQ4224_TEMP_TH_HYST_REG,
					  &uval);
			if (ret)
				break;

			*val = ADAQ4224_TEMP_16_BIT_TO_M_DGR(uval);
			break;
		case hwmon_temp_max_alarm:
			ret = regmap_read(st->regmap,
					  ADAQ4224_TEMP_CONFIGURATION_REG, &uval);
			if (ret)
				break;

			*val = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_O_TEMP_STAT_MASK,
					 uval);
			break;
		case hwmon_temp_min:
			ret = regmap_read(st->regmap, ADAQ4224_TEMP_TL_REG, &uval);
			if (ret)
				break;

			*val = ADAQ4224_TEMP_16_BIT_TO_M_DGR(uval);
			break;
		case hwmon_temp_min_hyst:
			ret = regmap_read(st->regmap, ADAQ4224_TEMP_TL_HYST_REG,
					  &uval);
			if (ret)
				break;

			*val = ADAQ4224_TEMP_16_BIT_TO_M_DGR(uval);
			break;
		case hwmon_temp_min_alarm:
			ret = regmap_read(st->regmap,
					  ADAQ4224_TEMP_CONFIGURATION_REG, &uval);
			if (ret)
				break;

			*val = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_U_TEMP_STAT_MASK,
					 uval);
			break;
		default:
			ret = -EOPNOTSUPP;
			break;
		}

		break;

	case hwmon_chip:
		if (attr == hwmon_chip_update_interval) {
			ret = regmap_read(st->regmap,
					  ADAQ4224_TEMP_CONFIGURATION_REG, &uval);
			if (ret)
				break;

			uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
					 uval);
			*val = adaq4224_temp_conversions[uval];
		}
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static int adaq4224_temp_write(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	struct adaq4224_temp_state *st = dev_get_drvdata(dev);
	int res = 1;
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			if (val >> 1)
				return -EOPNOTSUPP;

			mutex_lock(&st->lock);
			/**
			 * The chip should not be enabled while a conversion is
			 * performed. Neither should the chip be enabled when
			 * the alarm values are changed.
			 */

			st->enable = val;

			ret = adaq4224_temp_update_bits(st,
							ADAQ4224_TEMP_CONFIGURATION_REG,
							ADAQ4224_TEMP_CONFIGURATION_1SHOT_MASK |
							ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
							ADAQ4224_TEMP_DEVICE_ENABLE(val));

			mutex_unlock(&st->lock);

			return ret;

		case hwmon_temp_max:
			return write_alarm_val(st, ADAQ4224_TEMP_TH_REG, val);

		case hwmon_temp_max_hyst:
			return write_alarm_val(st, ADAQ4224_TEMP_TH_HYST_REG, val);

		case hwmon_temp_min:
			return write_alarm_val(st, ADAQ4224_TEMP_TL_REG, val);

		case hwmon_temp_min_hyst:
			return write_alarm_val(st, ADAQ4224_TEMP_TL_HYST_REG, val);

		default:
			return -EOPNOTSUPP;
		}

	case hwmon_chip:
		if (attr == hwmon_chip_update_interval) {
			if (!st->enable)
				return -EOPNOTSUPP;

			/*
			 * Convert the desired conversion rate into register
			 * bits. res is already initialized with 1.
			 *
			 * This was inspired by lm73 driver.
			 */
			while (res < ARRAY_SIZE(adaq4224_temp_conversions) &&
			       val < adaq4224_temp_conversions[res])
			       res++;

			if (res == ARRAY_SIZE(adaq4224_temp_conversions))
				res = ARRAY_SIZE(adaq4224_temp_conversions) - 1;

			res = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
					 res);

			ret = adaq4224_temp_update_bits(st,
							ADAQ4224_TEMP_CONFIGURATION_REG,
							ADAQ4224_TEMP_CONFIGURATION_CNV_RATE_MASK,
							res);
			if (ret)
				return ret;

			st->update_interval = val;
		}
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static ssize_t temp1_resolution_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	struct adaq4224_temp_state *st = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADAQ4224_TEMP_CONFIGURATION_REG, &val);
	if (ret)
		return ret;

	val = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK, val);

	return scnprintf(buf, PAGE_SIZE, "%u\n", adaq4224_temp_resolutions[val]);
}

static ssize_t temp1_resolution_store(struct device *dev,
				      struct device_attribute *devattr,
				      const char *buf, size_t count)
{
	struct adaq4224_temp_state *st = dev_get_drvdata(dev);
	unsigned int idx = 0;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	/*
	 * Convert the desired resolution into register
	 * bits. idx is already initialized with 0.
	 *
	 * This was inspired by lm73 driver.
	 */
	while (idx < ARRAY_SIZE(adaq4224_temp_resolutions) &&
	       val < adaq4224_temp_resolutions[idx])
		idx++;

	if (idx == ARRAY_SIZE(adaq4224_temp_resolutions))
		idx = ARRAY_SIZE(adaq4224_temp_resolutions) - 1;

	st->resolution = idx;

	ret = shutdown_write(st, ADAQ4224_TEMP_CONFIGURATION_REG,
			     ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK,
			     FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK,
			     idx));

	return ret ? ret : count;
}

static ssize_t pec_show(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!(client->flags & I2C_CLIENT_PEC));
}

static ssize_t pec_store(struct device *dev, struct device_attribute *devattr,
			 const char *buf, size_t count)
{
	struct adaq4224_temp_state *st = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int val, val2;
	int err;

	err = kstrtouint(buf, 10, &val);
	if (err < 0)
		return err;

	val2 = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK, !!val);

	switch (val) {
	case 0:
		err = adaq4224_temp_update_bits(st, ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK,
						val2);
		if (err)
			return err;

		client->flags &= ~I2C_CLIENT_PEC;
		break;
	case 1:
		err = adaq4224_temp_update_bits(st, ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK,
						val2);
		if (err)
			return err;

		client->flags |= I2C_CLIENT_PEC;
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR_RW(temp1_resolution);
static DEVICE_ATTR_RW(pec);

static struct attribute *adaq4224_temp_attrs[] = {
	&dev_attr_temp1_resolution.attr,
	&dev_attr_pec.attr,
	NULL
};
ATTRIBUTE_GROUPS(adaq4224_temp);

static const struct i2c_device_id adaq4224_temp_i2c_id[] = {
	{ "adaq4224_temp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adaq4224_temp_i2c_id);

static int adaq4224_temp_init_client(struct adaq4224_temp_state *st,
				     struct device *dev)
{
	struct fwnode_handle *fwnode;
	unsigned int res = 0;
	u32 data, lsb_idx;
	bool prop;
	int ret;

	fwnode = dev_fwnode(dev);

	st->enable = true;
	res |= ADAQ4224_TEMP_DEVICE_ENABLE(1);

	res |= ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK;

	prop = fwnode_property_read_bool(fwnode, "adi,comp-int");
	res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_COMP_INT_MASK, prop);

	prop = fwnode_property_read_bool(fwnode, "adi,timeout-enable");
	res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_TIMEOUT_MASK, !prop);
	if (fwnode_property_present(fwnode, "adi,alarm-pol")) {
		ret = fwnode_property_read_u32(fwnode, "adi,alarm-pol", &data);
		if (ret)
			return ret;

		res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK,
				  !!data);
	} else 
		res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK,
				  ADAQ4224_TEMP_ALRM_POL_LOW);

	if (fwnode_property_present(fwnode, "adi,fault-q")) {
		ret = fwnode_property_read_u32(fwnode, "adi,fault-q", &data);
		if (ret)
			return ret;

		/*
		 * Convert the desired fault queue into register bits.
		 */
		if (data != 0)
			lsb_idx = __ffs(data);

		if (hweight32(data) != 1 || lsb_idx > 4) {
			dev_err(dev, "Invalid data in adi,fault-q\n");
			return -EINVAL;
		}

		res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK,
				  lsb_idx);
	} else 
		res |= FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK,
				  ADAQ4224_TEMP_FLT_Q_1);

	return adaq4224_temp_reg_write(st, ADAQ4224_TEMP_CONFIGURATION_REG, res);
}

static const struct hwmon_channel_info *adaq4224_temp_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_ENABLE | HWMON_T_INPUT | HWMON_T_MIN |
			   HWMON_T_MIN_HYST | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX | HWMON_T_MAX_HYST |
			   HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	NULL,
};

static const struct hwmon_ops adaq4224_temp_hwmon_ops = {
	.is_visible = adaq4224_temp_is_visible,
	.read = adaq4224_temp_read,
	.write = adaq4224_temp_write,
};

static const struct hwmon_chip_info adaq4224_temp_chip_info = {
	.ops = &adaq4224_temp_hwmon_ops,
	.info = adaq4224_temp_info,
};

#ifdef CONFIG_DEBUG_FS
static ssize_t adaq4224_temp_debugfs_read(struct file *file, char __user *buf,
					  size_t count, loff_t *ppos)
{
	char tbuf[DEBUG_FS_DATA_MAX] = { 0 };
	struct adaq4224_temp_debugfs_data *psu;
	struct adaq4224_temp_state *st;
	int *attrp = file_inode(file)->i_private;
	int attr = *attrp;
	unsigned int uval;
	int ret, len;

	psu = container_of(attrp, struct adaq4224_temp_debugfs_data, debugfs_entries[attr]);
	st = container_of(psu, struct adaq4224_temp_state, psu);

	ret = regmap_read(st->regmap, ADAQ4224_TEMP_CONFIGURATION_REG, &uval);
	if (ret)
		return ret;

	switch (attr) {
	case ADAQ4224_TEMP_DEBUGFS_PEC_ENABLE:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_TIMEOUT:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_TIMEOUT_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_RESOLUTION:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_ALARM_POLARITY:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_COMP_INT:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_COMP_INT_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_FAULT_QUEUE:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_PEC_ERROR:
		uval = FIELD_GET(ADAQ4224_TEMP_CONFIGURATION_PEC_ERR_MASK, uval);
		len = scnprintf(tbuf, DEBUG_FS_DATA_MAX, "%d\n", uval);
		break;
	default:
		len = strscpy(tbuf, "Invalid\n", DEBUG_FS_DATA_MAX);
	}

	return simple_read_from_buffer(buf, count, ppos, tbuf, len);
}

static ssize_t adaq4224_temp_debugfs_write(struct file *file, const char __user *buf,
					   size_t count, loff_t *ppos)
{
	char tbuf[DEBUG_FS_DATA_MAX] = { 0 };
	struct adaq4224_temp_debugfs_data *psu;
	struct adaq4224_temp_state *st;
	int *attrp = file_inode(file)->i_private;
	int attr = *attrp;
	u16 uval;
	int ret;

	pr_info("attr = %d\n", attr);
	psu = container_of(attrp, struct adaq4224_temp_debugfs_data, debugfs_entries[attr]);
	pr_info("First container ok.\n");
	st = container_of(psu, struct adaq4224_temp_state, psu);
	pr_info("st->test = %d\n", st->test);

	ret = kstrtou16_from_user(buf, count, 0, &uval);
	if (ret)
		return ret;

	pr_info("uval = %s\n", tbuf);

	switch (attr) {
	case ADAQ4224_TEMP_DEBUGFS_PEC_ENABLE:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_PEC_EN_MASK,
						uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_TIMEOUT:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_TIMEOUT_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_TIMEOUT_MASK,
						uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_RESOLUTION:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_RESOLUTION_MASK,
						uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_ALARM_POLARITY:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_ALRM_POL_MASK,
						uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_COMP_INT:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_COMP_INT_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_COMP_INT_MASK,
						uval);
		break;
	case ADAQ4224_TEMP_DEBUGFS_FAULT_QUEUE:
		uval = FIELD_PREP(ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK, uval);
		ret = adaq4224_temp_update_bits(st,
						ADAQ4224_TEMP_CONFIGURATION_REG,
						ADAQ4224_TEMP_CONFIGURATION_FLT_Q_MASK,
						uval);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	return count;
}

static const struct file_operations adaq4224_temp_fops = {
	.read = adaq4224_temp_debugfs_read,
	.write = adaq4224_temp_debugfs_write,
};

static int adaq4224_temp_init_debugfs(struct adaq4224_temp_state *st,
				      struct i2c_client *client)
{
	struct dentry *debugfs;
	int i;

	debugfs = debugfs_create_dir(client->name, NULL);
	if (!debugfs)
		return -ENOENT;

	for (i = 0; i < ADAQ4224_TEMP_DEBUGFS_NUM_ENTRIES; ++i)
		st->psu.debugfs_entries[i] = i;

	debugfs_create_file("pec_enable", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_PEC_ENABLE],
			    &adaq4224_temp_fops);
	debugfs_create_file("timeout", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_TIMEOUT],
			    &adaq4224_temp_fops);
	debugfs_create_file("resolution", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_RESOLUTION],
			    &adaq4224_temp_fops);
	debugfs_create_file("alarm_polarity", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_ALARM_POLARITY],
			    &adaq4224_temp_fops);
	debugfs_create_file("comp_int", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_COMP_INT],
			    &adaq4224_temp_fops);
	debugfs_create_file("fault_queue", 0644, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_FAULT_QUEUE],
			    &adaq4224_temp_fops);
	debugfs_create_file("pec_error", 0444, debugfs,
			    &st->psu.debugfs_entries[ADAQ4224_TEMP_DEBUGFS_PEC_ERROR],
			    &adaq4224_temp_fops);

	return 0;
}
#else
static int adaq4224_temp_init_debugfs(struct adaq4224_temp_state *st,
				      struct i2c_client *client)
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static int adaq4224_temp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct adaq4224_temp_state *st;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_i2c(client, &adaq4224_temp_regmap);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to allocate regmap.\n");

	err = devm_regulator_get_enable(dev, "vref");
	if (err)
		return dev_err_probe(dev, err, "failed to enable regulator\n");

	err = adaq4224_temp_init_client(st, dev);
	if (err)
		return err;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, st,
							 &adaq4224_temp_chip_info,
							 adaq4224_temp_groups);
	if (IS_ERR(hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(hwmon_dev),
				     "Failed to register device.\n");

	adaq4224_temp_init_debugfs(st, client);

	return 0;
}

static const struct of_device_id adaq4224_temp_of_match[] = {
	{ .compatible = "adi,adaq4224-temp" },
	{ }
};
MODULE_DEVICE_TABLE(of, adaq4224_temp_of_match);

static struct i2c_driver adaq4224_temp_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "adaq4224_temp",
		.of_match_table = adaq4224_temp_of_match,
	},
	.probe_new = adaq4224_temp_probe,
	.id_table = adaq4224_temp_i2c_id,
};
module_i2c_driver(adaq4224_temp_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAQ4224 chipset thermal sensor driver");
MODULE_LICENSE("GPL");
