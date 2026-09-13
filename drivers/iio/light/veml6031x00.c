// SPDX-License-Identifier: GPL-2.0+
/*
 * VEML6031X00 Ambient Light Sensor
 *
 * Copyright (c) 2026, Javier Carrasco <javier.carrasco.cruz@gmail.com>
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <asm/byteorder.h>

#include <linux/iio/iio.h>
#include <linux/iio/iio-gts-helper.h>

/* Device registers */
#define VEML6031X00_REG_CONF0       0x00
#define VEML6031X00_REG_CONF1       0x01
#define VEML6031X00_REG_ALS_L       0x10
#define VEML6031X00_REG_ALS_H       0x11
#define VEML6031X00_REG_IR_L        0x12
#define VEML6031X00_REG_IR_H        0x13
#define VEML6031X00_REG_ID_L        0x14
#define VEML6031X00_REG_ID_H        0x15

/* Bit masks for specific functionality */
#define VEML6031X00_CONF0_SD        BIT(0)
#define VEML6031X00_CONF1_IR_SD     BIT(7)

#define VEML6031X00_GAIN_SEL(pd_div4, gain) (((pd_div4) << 2) | (gain))

struct veml6031x00_rf {
	struct regmap_field *gain;
	struct regmap_field *it;
	struct regmap_field *pd_div4;
};

struct veml6031x00_chip {
	const char *name;
	const unsigned int part_id;
};

struct veml6031x00_data {
	struct iio_gts gts;
	struct regmap *regmap;
	struct veml6031x00_rf rf;
	const struct veml6031x00_chip *chip;
	/*
	 * Serialize access to scale register fields scattered across multiple
	 * registers (rf.gain, rf.pd_div4, rf.it) to read and write them as a
	 * consistent set.
	 */
	struct mutex scale_lock;
};

static const struct iio_itime_sel_mul veml6031x00_it_sel[] = {
	GAIN_SCALE_ITIME_US(3125, 0, 1),
	GAIN_SCALE_ITIME_US(6250, 1, 2),
	GAIN_SCALE_ITIME_US(12500, 2, 4),
	GAIN_SCALE_ITIME_US(25000, 3, 8),
	GAIN_SCALE_ITIME_US(50000, 4, 16),
	GAIN_SCALE_ITIME_US(100000, 5, 32),
	GAIN_SCALE_ITIME_US(200000, 6, 64),
	GAIN_SCALE_ITIME_US(400000, 7, 128),
};

/*
 * The gain selector encodes (PD_D4 << 2) | GAIN to identify each gain setting.
 * Gains are multiplied by 8 to work with integers. The values in the iio-gts
 * tables don't need corrections because the maximum value of the scale refers
 * to GAIN = x1, and the rest of the values are obtained from the resulting
 * linear function.
 * TODO: add support for GAIN_0_165 and GAIN_0_660
 */
#define VEML6031X00_SEL_GAIN_0_125 0x07
#define VEML6031X00_SEL_GAIN_0_250 0x04
#define VEML6031X00_SEL_GAIN_0_500 0x03
#define VEML6031X00_SEL_GAIN_1_000 0x00
#define VEML6031X00_SEL_GAIN_2_000 0x01
static const struct iio_gain_sel_pair veml6031x00_gain_sel[] = {
	GAIN_SCALE_GAIN(1, VEML6031X00_SEL_GAIN_0_125),
	GAIN_SCALE_GAIN(2, VEML6031X00_SEL_GAIN_0_250),
	GAIN_SCALE_GAIN(4, VEML6031X00_SEL_GAIN_0_500),
	GAIN_SCALE_GAIN(8, VEML6031X00_SEL_GAIN_1_000),
	GAIN_SCALE_GAIN(16, VEML6031X00_SEL_GAIN_2_000),
};

/*
 * The shutdown bits (SD and ALS_IR_SD) are in different registers, and both
 * must be updated when changing the device power state.
 */
static int veml6031x00_set_power(struct veml6031x00_data *data, bool state)
{
	int ret;

	ret = regmap_assign_bits(data->regmap, VEML6031X00_REG_CONF0,
				 VEML6031X00_CONF0_SD, !state);
	if (ret)
		return ret;

	return regmap_assign_bits(data->regmap, VEML6031X00_REG_CONF1,
				  VEML6031X00_CONF1_IR_SD, !state);
}

static void veml6031x00_als_shutdown_action(void *data)
{
	struct veml6031x00_data *priv = data;
	struct device *dev = regmap_get_device(priv->regmap);
	int ret;

	ret = veml6031x00_set_power(priv, false);
	if (ret)
		dev_warn(dev, "Failed to shut down device: %d\n", ret);
}

static const struct iio_chan_spec veml6031x00_channels[] = {
	{
		.type = IIO_LIGHT,
		.address = VEML6031X00_REG_ALS_L,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_INT_TIME) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_INT_TIME) |
						BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_INTENSITY,
		.address = VEML6031X00_REG_IR_L,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static const struct regmap_range veml6031x00_readable_ranges[] = {
	regmap_reg_range(VEML6031X00_REG_CONF0, VEML6031X00_REG_CONF1),
	regmap_reg_range(VEML6031X00_REG_ALS_L, VEML6031X00_REG_ID_H),
};

static const struct regmap_access_table veml6031x00_readable_table = {
	.yes_ranges = veml6031x00_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(veml6031x00_readable_ranges),
};

static const struct regmap_range veml6031x00_writable_ranges[] = {
	regmap_reg_range(VEML6031X00_REG_CONF0, VEML6031X00_REG_CONF1),
};

static const struct regmap_access_table veml6031x00_writable_table = {
	.yes_ranges = veml6031x00_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(veml6031x00_writable_ranges),
};

static const struct regmap_range veml6031x00_volatile_ranges[] = {
	/* AF_TRIG in CONF0 is volatile */
	regmap_reg_range(VEML6031X00_REG_CONF0, VEML6031X00_REG_CONF0),
	regmap_reg_range(VEML6031X00_REG_ALS_L, VEML6031X00_REG_IR_H),
};

static const struct regmap_access_table veml6031x00_volatile_table = {
	.yes_ranges = veml6031x00_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(veml6031x00_volatile_ranges),
};

static const struct regmap_config veml6031x00_regmap_config = {
	.name = "veml6031x00_regmap",
	.reg_bits = 8,
	.val_bits = 8,
	.rd_table = &veml6031x00_readable_table,
	.wr_table = &veml6031x00_writable_table,
	.volatile_table = &veml6031x00_volatile_table,
	.max_register = VEML6031X00_REG_ID_H,
	.cache_type = REGCACHE_MAPLE,
};

static const struct reg_field veml6031x00_rf_it =
	REG_FIELD(VEML6031X00_REG_CONF0, 4, 6);

static const struct reg_field veml6031x00_rf_gain =
	REG_FIELD(VEML6031X00_REG_CONF1, 3, 4);

static const struct reg_field veml6031x00_rf_pd_div4 =
	REG_FIELD(VEML6031X00_REG_CONF1, 6, 6);

static int veml6031x00_regfield_init(struct veml6031x00_data *data)
{
	struct regmap *map = data->regmap;
	struct device *dev = regmap_get_device(map);
	struct regmap_field *rm_field;
	struct veml6031x00_rf *rf = &data->rf;

	rm_field = devm_regmap_field_alloc(dev, map, veml6031x00_rf_gain);
	if (IS_ERR(rm_field))
		return PTR_ERR(rm_field);
	rf->gain = rm_field;

	rm_field = devm_regmap_field_alloc(dev, map, veml6031x00_rf_it);
	if (IS_ERR(rm_field))
		return PTR_ERR(rm_field);
	rf->it = rm_field;

	rm_field = devm_regmap_field_alloc(dev, map, veml6031x00_rf_pd_div4);
	if (IS_ERR(rm_field))
		return PTR_ERR(rm_field);
	rf->pd_div4 = rm_field;

	return 0;
}

static int __veml6031x00_get_it(struct veml6031x00_data *data, int *val2)
{
	unsigned int it_idx;
	int ret;

	ret = regmap_field_read(data->rf.it, &it_idx);
	if (ret)
		return ret;

	ret = iio_gts_find_int_time_by_sel(&data->gts, it_idx);
	if (ret < 0)
		return ret;

	*val2 = ret;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int veml6031x00_get_it(struct veml6031x00_data *data, int *val2)
{
	guard(mutex)(&data->scale_lock);

	return __veml6031x00_get_it(data, val2);
}

static int veml6031x00_write_gain(struct veml6031x00_data *data, unsigned int gain)
{
	int ret;

	ret = regmap_field_write(data->rf.pd_div4, gain >> 2);
	if (ret)
		return ret;

	return regmap_field_write(data->rf.gain, gain & 0x03);
}

static int veml6031x00_set_it(struct iio_dev *iio, int val, int val2)
{
	struct veml6031x00_data *data = iio_priv(iio);
	struct device *dev = regmap_get_device(data->regmap);
	int ret, gain_sel, new_gain, prev_gain, prev_it;
	unsigned int gain_reg, it_idx, pd_div4;
	struct iio_gts *gts = &data->gts;
	bool gain_in_range;

	if (val || !iio_gts_valid_time(gts, val2))
		return -EINVAL;

	guard(mutex)(&data->scale_lock);

	ret = regmap_field_read(data->rf.it, &it_idx);
	if (ret)
		return ret;

	ret = regmap_field_read(data->rf.gain, &gain_reg);
	if (ret)
		return ret;

	ret = regmap_field_read(data->rf.pd_div4, &pd_div4);
	if (ret)
		return ret;

	prev_it = iio_gts_find_int_time_by_sel(gts, it_idx);
	if (prev_it < 0)
		return prev_it;

	if (prev_it == val2)
		return 0;

	gain_sel = VEML6031X00_GAIN_SEL(pd_div4, gain_reg);
	prev_gain = iio_gts_find_gain_by_sel(gts, gain_sel);
	if (prev_gain < 0)
		return prev_gain;

	ret = iio_gts_find_new_gain_by_gain_time_min(gts, prev_gain, prev_it, val2,
						     &new_gain, &gain_in_range);
	if (ret)
		return ret;

	if (!gain_in_range)
		dev_dbg(dev, "Optimal gain out of range\n");

	ret = iio_gts_find_sel_by_int_time(gts, val2);
	if (ret < 0)
		return ret;

	ret = regmap_field_write(data->rf.it, ret);
	if (ret)
		return ret;

	gain_sel = iio_gts_find_sel_by_gain(gts, new_gain);
	if (gain_sel < 0)
		return gain_sel;

	return veml6031x00_write_gain(data, gain_sel);
}

static int veml6031x00_set_scale(struct iio_dev *iio, int val, int val2)
{
	struct veml6031x00_data *data = iio_priv(iio);
	struct iio_gts *gts = &data->gts;
	int gain_sel, it_sel, ret;

	ret = iio_gts_find_gain_time_sel_for_scale(gts, val, val2, &gain_sel, &it_sel);
	if (ret)
		return ret;

	guard(mutex)(&data->scale_lock);

	ret = regmap_field_write(data->rf.it, it_sel);
	if (ret)
		return ret;

	return veml6031x00_write_gain(data, gain_sel);
}

static int veml6031x00_get_scale(struct veml6031x00_data *data, int *val, int *val2)
{
	unsigned int gain_reg, pd_div4, it_reg;
	int gain, it, ret, sel;

	scoped_guard(mutex, &data->scale_lock) {
		ret = regmap_field_read(data->rf.gain, &gain_reg);
		if (ret)
			return ret;

		ret = regmap_field_read(data->rf.pd_div4, &pd_div4);
		if (ret)
			return ret;

		sel = VEML6031X00_GAIN_SEL(pd_div4, gain_reg);
		gain = iio_gts_find_gain_by_sel(&data->gts, sel);
		if (gain < 0)
			return gain;

		ret = regmap_field_read(data->rf.it, &it_reg);
		if (ret)
			return ret;
	}

	it = iio_gts_find_int_time_by_sel(&data->gts, it_reg);
	if (it < 0)
		return it;

	ret = iio_gts_get_scale(&data->gts, gain, it, val, val2);
	if (ret)
		return ret;

	return IIO_VAL_INT_PLUS_NANO;
}

static int veml6031x00_single_read(struct iio_dev *iio, enum iio_chan_type type,
				   int *val)
{
	struct veml6031x00_data *data = iio_priv(iio);
	unsigned int addr;
	int it_usec, ret;
	__le16 regval;

	switch (type) {
	case IIO_LIGHT:
		addr = VEML6031X00_REG_ALS_L;
		break;
	case IIO_INTENSITY:
		addr = VEML6031X00_REG_IR_L;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&data->scale_lock);

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(regmap_get_device(data->regmap), pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	ret = __veml6031x00_get_it(data, &it_usec);
	if (ret < 0)
		return ret;

	/* integration time + 10% to ensure completion */
	fsleep(it_usec + (it_usec / 10));

	ret = regmap_bulk_read(data->regmap, addr, &regval, sizeof(regval));
	if (ret)
		return ret;

	*val = le16_to_cpu(regval);

	return IIO_VAL_INT;
}

static int veml6031x00_read_raw(struct iio_dev *iio,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct veml6031x00_data *data = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return veml6031x00_single_read(iio, chan->type, val);
	case IIO_CHAN_INFO_INT_TIME:
		*val = 0;
		return veml6031x00_get_it(data, val2);
	case IIO_CHAN_INFO_SCALE:
		return veml6031x00_get_scale(data, val, val2);
	default:
		return -EINVAL;
	}
}

static int veml6031x00_read_avail(struct iio_dev *iio,
				  struct iio_chan_spec const *chan,
				  const int **vals, int *type, int *length,
				  long mask)
{
	struct veml6031x00_data *data = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return iio_gts_avail_times(&data->gts, vals, type, length);
	case IIO_CHAN_INFO_SCALE:
		return iio_gts_all_avail_scales(&data->gts, vals, type, length);
	default:
		return -EINVAL;
	}
}

static int veml6031x00_write_raw(struct iio_dev *iio,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return veml6031x00_set_it(iio, val, val2);
	case IIO_CHAN_INFO_SCALE:
		return veml6031x00_set_scale(iio, val, val2);
	default:
		return -EINVAL;
	}
}

static int veml6031x00_write_raw_get_fmt(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_INT_TIME:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info veml6031x00_info = {
	.read_raw = veml6031x00_read_raw,
	.read_avail = veml6031x00_read_avail,
	.write_raw = veml6031x00_write_raw,
	.write_raw_get_fmt = veml6031x00_write_raw_get_fmt,
};

static int veml6031x00_validate_part_id(struct veml6031x00_data *data)
{
	struct regmap *map = data->regmap;
	struct device *dev = regmap_get_device(map);
	unsigned int part_id;
	int ret;
	__le16 regval;

	ret = regmap_bulk_read(map, VEML6031X00_REG_ID_L, &regval, sizeof(regval));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read ID\n");

	part_id = le16_to_cpu(regval);
	if (part_id != data->chip->part_id)
		dev_info(dev, "Unknown or unexpected ID %04x\n", part_id);

	return 0;
}

static int veml6031x00_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct veml6031x00_data *data;
	struct iio_dev *iio;
	int ret;

	iio = devm_iio_device_alloc(dev, sizeof(*data));
	if (!iio)
		return -ENOMEM;

	data = iio_priv(iio);
	i2c_set_clientdata(i2c, iio);

	data->chip = i2c_get_match_data(i2c);
	if (!data->chip)
		return dev_err_probe(dev, -ENODATA, "Failed to get chip data\n");

	ret = devm_mutex_init(dev, &data->scale_lock);
	if (ret)
		return ret;

	data->regmap = devm_regmap_init_i2c(i2c, &veml6031x00_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap),
				     "Failed to set regmap\n");

	iio->name = data->chip->name;
	iio->channels = veml6031x00_channels;
	iio->num_channels = ARRAY_SIZE(veml6031x00_channels);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &veml6031x00_info;

	ret = veml6031x00_regfield_init(data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init regfield\n");

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulator\n");

	/* The device starts in power down mode by default */
	ret = veml6031x00_set_power(data, true);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to power on the device\n");

	ret = devm_add_action_or_reset(dev, veml6031x00_als_shutdown_action, data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add shutdown action\n");

	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable runtime PM\n");

	ret = veml6031x00_validate_part_id(data);
	if (ret)
		return ret;

	/* Max resolution = 6.9632 lx/cnt for gain = 0.125 and IT = 3.125ms */
	ret = devm_iio_init_iio_gts(dev, 6, 963200000,
				    veml6031x00_gain_sel,
				    ARRAY_SIZE(veml6031x00_gain_sel),
				    veml6031x00_it_sel,
				    ARRAY_SIZE(veml6031x00_it_sel),
				    &data->gts);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init IIO GTS\n");

	ret = devm_iio_device_register(dev, iio);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IIO device\n");

	return 0;
}

static int veml6031x00_runtime_suspend(struct device *dev)
{
	struct veml6031x00_data *data = iio_priv(dev_get_drvdata(dev));

	return veml6031x00_set_power(data, false);
}

static int veml6031x00_runtime_resume(struct device *dev)
{
	struct veml6031x00_data *data = iio_priv(dev_get_drvdata(dev));

	return veml6031x00_set_power(data, true);
}

static DEFINE_RUNTIME_DEV_PM_OPS(veml6031x00_pm_ops,
				 veml6031x00_runtime_suspend,
				 veml6031x00_runtime_resume,
				 NULL);

static const struct veml6031x00_chip veml6031x00_chip = {
	.name = "veml6031x00",
	.part_id = 0x0001,
};

static const struct veml6031x00_chip veml6031x01_chip = {
	.name = "veml6031x01",
	.part_id = 0x0001,
};

static const struct veml6031x00_chip veml60311x00_chip = {
	.name = "veml60311x00",
	.part_id = 0x1001,
};

static const struct veml6031x00_chip veml60311x01_chip = {
	.name = "veml60311x01",
	.part_id = 0x1001,
};

static const struct of_device_id veml6031x00_of_match[] = {
	{
		.compatible = "vishay,veml6031x00",
		.data = &veml6031x00_chip,
	},
	{
		.compatible = "vishay,veml6031x01",
		.data = &veml6031x01_chip,
	},
	{
		.compatible = "vishay,veml60311x00",
		.data = &veml60311x00_chip,
	},
	{
		.compatible = "vishay,veml60311x01",
		.data = &veml60311x01_chip,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, veml6031x00_of_match);

static const struct i2c_device_id veml6031x00_id[] = {
	{
		.name = "veml6031x00",
		.driver_data = (kernel_ulong_t)&veml6031x00_chip,
	},
	{
		.name = "veml6031x01",
		.driver_data = (kernel_ulong_t)&veml6031x01_chip,
	},
	{
		.name = "veml60311x00",
		.driver_data = (kernel_ulong_t)&veml60311x00_chip,
	},
	{
		.name = "veml60311x01",
		.driver_data = (kernel_ulong_t)&veml60311x01_chip,
	},
	{ }
};
MODULE_DEVICE_TABLE(i2c, veml6031x00_id);

static struct i2c_driver veml6031x00_driver = {
	.driver = {
		.name = "veml6031x00",
		.of_match_table = veml6031x00_of_match,
		.pm = pm_ptr(&veml6031x00_pm_ops),
	},
	.probe = veml6031x00_probe,
	.id_table = veml6031x00_id,
};
module_i2c_driver(veml6031x00_driver);

MODULE_AUTHOR("Javier Carrasco <javier.carrasco.cruz@gmail.com>");
MODULE_DESCRIPTION("VEML6031X00 Ambient Light Sensor");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_GTS_HELPER");
