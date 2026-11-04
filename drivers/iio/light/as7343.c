// SPDX-License-Identifier: GPL-2.0-only
/*
 * Support for AMS AS7343 14-channel multi-spectral sensor.
 * (7-bit I2C slave address 0x39)
 *
 * Based on the work of:
 *   Christian Eggers <ceggers@arri.de> (AS73211 driver)
 *
 * Copyright (c) 2026 Chang Yu <marcus.yu.56@gmail.com>
 *
 * Datasheet:
 *   https://look.ams-osram.com/m/5f2d27fff9a874d2/original/AS7343-14-Channel-Multi-Spectral-Sensor.pdf
 *
 * TODO:
 *   - Support for configurable gain and integration time
 *   - Interrupt support
 *   - Add support for reading the VIS channel
 *   - Flicker detection
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <asm/byteorder.h>

#include <linux/iio/iio.h>

#define AS7343_ID_REG				0x5a

#define AS7343_ENABLE_REG			0x80
#define AS7343_ENABLE_PON			BIT(0)
#define AS7343_ENABLE_SP_EN			BIT(1)

#define AS7343_ATIME_REG			0x81
#define AS7343_STATUS2_REG			0x90
#define AS7343_STATUS3_REG			0x91
#define AS7343_STATUS_REG			0x93

#define AS7343_ASTATUS_REG			0x94
#define AS7343_ASTATUS_ASAT_STATUS		BIT(7)

#define AS7343_DATA_FZ_REG			0x95
#define AS7343_DATA_FY_REG			0x97
#define AS7343_DATA_FXL_REG			0x99
#define AS7343_DATA_NIR_REG			0x9b
#define AS7343_DATA_F2_REG			0xa1
#define AS7343_DATA_F3_REG			0xa3
#define AS7343_DATA_F4_REG			0xa5
#define AS7343_DATA_F6_REG			0xa7
#define AS7343_DATA_F1_REG			0xad
#define AS7343_DATA_F7_REG			0xaf
#define AS7343_DATA_F8_REG			0xb1
#define AS7343_DATA_F5_REG			0xb3
#define AS7343_DATA_FD_L_REG			0xb7
#define AS7343_DATA_FD_H_REG			0xb8

#define AS7343_STATUS5_REG			0xbb
#define AS7343_STATUS4_REG			0xbc

#define AS7343_CFG0_REG				0xbf
#define AS7343_CFG0_REG_BANK			BIT(4)

#define AS7343_CFG1_REG				0xc6
#define AS7343_CFG1_AGAIN			GENMASK(4, 0)
#define AS7343_CFG1_AGAIN_X0_5			0
#define AS7343_CFG1_AGAIN_X1			1
#define AS7343_CFG1_AGAIN_X2			2
#define AS7343_CFG1_AGAIN_X4			3
#define AS7343_CFG1_AGAIN_X8			4
#define AS7343_CFG1_AGAIN_X16			5
#define AS7343_CFG1_AGAIN_X32			6
#define AS7343_CFG1_AGAIN_X64			7
#define AS7343_CFG1_AGAIN_X128			8
#define AS7343_CFG1_AGAIN_X256			9
#define AS7343_CFG1_AGAIN_X512			10
#define AS7343_CFG1_AGAIN_X1024			11
#define AS7343_CFG1_AGAIN_X2048			12

#define AS7343_ASTEP_REG			0xd4

#define AS7343_CFG20_REG			0xd6
#define AS7343_CFG20_AUTO_SMUX			GENMASK(6, 5)
#define AS7343_CFG20_AUTO_SMUX_READOUT_ALL	3 /* all-channel readout */

#define AS7343_FD_STATUS_REG			0xe3
#define AS7343_CONTROL_REG			0xfa
#define AS7343_FIFO_LVL_REG			0xfd
#define AS7343_FDATA_L_REG			0xfe
#define AS7343_FDATA_H_REG			0xff

/* AS7343 channel indices. Follow data register order for AUTO_SMUX = 3  */
#define AS7343_CHAN_IDX_FZ	0
#define AS7343_CHAN_IDX_FY	1
#define AS7343_CHAN_IDX_FXL	2
#define AS7343_CHAN_IDX_NIR	3
#define AS7343_CHAN_IDX_F2	4
#define AS7343_CHAN_IDX_F3	5
#define AS7343_CHAN_IDX_F4	6
#define AS7343_CHAN_IDX_F6	7
#define AS7343_CHAN_IDX_F1	8
#define AS7343_CHAN_IDX_F7	9
#define AS7343_CHAN_IDX_F8	10
#define AS7343_CHAN_IDX_F5	11

#define AS7343_CHAN(_chan)					\
	{							\
		.type = IIO_INTENSITY,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.address = AS7343_DATA_##_chan##_REG,		\
		.indexed = 1,					\
		.channel = AS7343_CHAN_IDX_##_chan,		\
	}

static const struct iio_chan_spec as7343_channels[] = {
	AS7343_CHAN(FZ), AS7343_CHAN(FY), AS7343_CHAN(FXL), AS7343_CHAN(NIR),
	AS7343_CHAN(F2), AS7343_CHAN(F3), AS7343_CHAN(F4),  AS7343_CHAN(F6),
	AS7343_CHAN(F1), AS7343_CHAN(F7), AS7343_CHAN(F8),  AS7343_CHAN(F5),
};

/*
 * Integration time is calculated as (ATIME + 1) * ((ASTEP + 1) * 2.78us).
 * ATIME must be between 0 and 255, inclusive
 * ASTEP must be between 0 and 65534, inclusive
 * Per the datasheet, do not use 65535 for ASTEP.
 */
static int as7343_atime_steps(unsigned int step, u8 *atime)
{
	if (step < 1 || step > 256)
		return -EINVAL;

	*atime = step - 1;
	return 0;
}

static int as7343_astep_x2780ns(unsigned int n, __le16 *astep)
{
	u16 val;

	if (n < 1 || n > 65535)
		return -EINVAL;

	val = n - 1;
	*astep = cpu_to_le16(val);
	return 0;
}

struct as7343_data {
	struct regmap *regmap;
	/* Ensures reads don't stomp on each other */
	struct mutex mutex;
};

static int as7343_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct as7343_data *data = iio_priv(indio_dev);
	struct regmap *map = data->regmap;
	struct device *dev = regmap_get_device(map);
	unsigned int status;
	__le16 result;
	int ret;

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		/*
		 * Wait until integration time passes for all 3 cycles, plus
		 * some margins.
		 */
		msleep(3 * 50 + 10);

		/* Reading ASTATUS latches all data registers to this read. */
		guard(mutex)(&data->mutex);

		ret = regmap_read(map, AS7343_ASTATUS_REG, &status);
		if (ret)
			return ret;

		/* Check for saturation. */
		if (status & AS7343_ASTATUS_ASAT_STATUS)
			return -EOVERFLOW;

		ret = regmap_bulk_read(map, chan->address,
				       &result, sizeof(result));
		if (ret)
			return ret;

		*val = le16_to_cpu(result);
		return IIO_VAL_INT;
	}

	default:
		return -EINVAL;
	}
}

/*
 * Channel names and wavelength ranges as defined in the datasheet
 * (Figure 7, "AS7343 Optical Channel Summary"). Values are the
 * minimum and maximum peak wavelength in nanometers.
 *
 * F1:  395-415 nm
 * F2:  415-435 nm
 * FZ:  440-460 nm
 * F3:  465-485 nm
 * F4:  505-525 nm
 * FY:  545-565 nm
 * F5:  540-560 nm
 * FXL: 590-610 nm
 * F6:  630-650 nm
 * F7:  680-700 nm
 * F8:  735-755 nm
 * NIR: 845-865 nm
 */
static const char * const as7343_channel_labels[] = {
	[AS7343_CHAN_IDX_F1] = "F1: 395-415 nm",
	[AS7343_CHAN_IDX_F2] = "F2: 415-435 nm",
	[AS7343_CHAN_IDX_FZ] = "FZ: 440-460 nm",
	[AS7343_CHAN_IDX_F3] = "F3: 465-485 nm",
	[AS7343_CHAN_IDX_F4] = "F4: 505-525 nm",
	[AS7343_CHAN_IDX_FY] = "FY: 545-565 nm",
	[AS7343_CHAN_IDX_F5] = "F5: 540-560 nm",
	[AS7343_CHAN_IDX_FXL] = "FXL: 590-610 nm",
	[AS7343_CHAN_IDX_F6] = "F6: 630-650 nm",
	[AS7343_CHAN_IDX_F7] = "F7: 680-700 nm",
	[AS7343_CHAN_IDX_F8] = "F8: 735-755 nm",
	[AS7343_CHAN_IDX_NIR] = "NIR: 845-865 nm",
};

static int as7343_read_label(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, char *label)
{
	unsigned int num_labels;
	int channel;

	channel = chan->channel;
	num_labels = ARRAY_SIZE(as7343_channel_labels);
	if (!in_range(channel, 0, num_labels))
		return -ECHRNG;

	return sysfs_emit(label, "%s\n", as7343_channel_labels[channel]);
}

static const struct iio_info as7343_info = {
	.read_raw = as7343_read_raw,
	.read_label = as7343_read_label,
};

static const struct regmap_range as7343_volatile_ranges[] = {
	regmap_reg_range(AS7343_ENABLE_REG, AS7343_ENABLE_REG),
	regmap_reg_range(AS7343_STATUS2_REG, AS7343_DATA_FD_H_REG),
	regmap_reg_range(AS7343_STATUS5_REG, AS7343_STATUS4_REG),
	regmap_reg_range(AS7343_FD_STATUS_REG, AS7343_FD_STATUS_REG),
	regmap_reg_range(AS7343_CONTROL_REG, AS7343_CONTROL_REG),
	regmap_reg_range(AS7343_FIFO_LVL_REG, AS7343_FDATA_H_REG),
};

static const struct regmap_access_table as7343_volatile_table = {
	.yes_ranges = as7343_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(as7343_volatile_ranges),
};

static const struct regmap_config as7343_regmap_config = {
	.name = "as7343",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AS7343_FDATA_H_REG,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.cache_type = REGCACHE_MAPLE,
	.volatile_table = &as7343_volatile_table,
};

static int as7343_setup_device(struct device *dev, struct as7343_data *data)
{
	struct regmap *map = data->regmap;
	unsigned int val;
	__le16 astep;
	u8 atime;
	int ret;

	/* Need to set REG_BANK to 0 before we can set PON */
	ret = regmap_update_bits(map, AS7343_CFG0_REG, AS7343_CFG0_REG_BANK,
				 FIELD_PREP(AS7343_CFG0_REG_BANK, 0));
	if (ret)
		return ret;

	/* Set PON and enter IDLE state */
	ret = regmap_set_bits(map, AS7343_ENABLE_REG, AS7343_ENABLE_PON);
	if (ret)
		return ret;

	/* Need to set REG_BANK to 1 before we can access ID */
	ret = regmap_update_bits(map, AS7343_CFG0_REG, AS7343_CFG0_REG_BANK,
				 FIELD_PREP(AS7343_CFG0_REG_BANK, 1));
	if (ret)
		return ret;

	ret = regmap_read(map, AS7343_ID_REG, &val);
	if (ret)
		return ret;

	if (val != 0x81)
		dev_info(dev, "Unknown device ID: %x\n", val);

	ret = regmap_update_bits(map, AS7343_CFG0_REG, AS7343_CFG0_REG_BANK,
				 FIELD_PREP(AS7343_CFG0_REG_BANK, 0));
	if (ret)
		return ret;

	/* Configure the SMUX to readout all channels */
	ret = regmap_update_bits(map, AS7343_CFG20_REG, AS7343_CFG20_AUTO_SMUX,
				 FIELD_PREP(AS7343_CFG20_AUTO_SMUX,
					    AS7343_CFG20_AUTO_SMUX_READOUT_ALL));
	if (ret)
		return ret;

	/*
	 * Setting a 600 * 2.78us * 30 = 50ms integration time as the default,
	 * as recommended by Figure 10 of the datasheet ("Typical integration
	 * time": ASTEP=599, ATIME=29). The helpers take the number of steps,
	 * so pass 600 and 30 respectively.
	 */
	ret = as7343_astep_x2780ns(600, &astep);
	if (ret)
		return ret;

	ret = regmap_bulk_write(map, AS7343_ASTEP_REG, &astep, sizeof(astep));
	if (ret)
		return ret;

	ret = as7343_atime_steps(30, &atime);
	if (ret)
		return ret;

	ret = regmap_write(map, AS7343_ATIME_REG, atime);
	if (ret)
		return ret;

	return regmap_update_bits(map, AS7343_CFG1_REG, AS7343_CFG1_AGAIN,
				  FIELD_PREP(AS7343_CFG1_AGAIN,
					     AS7343_CFG1_AGAIN_X64));
}

static int as7343_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct as7343_data *data = iio_priv(indio_dev);
	struct regmap *map = data->regmap;
	int ret;

	ret = regmap_clear_bits(map, AS7343_ENABLE_REG, AS7343_ENABLE_SP_EN);
	if (ret)
		return ret;

	return regmap_clear_bits(map, AS7343_ENABLE_REG, AS7343_ENABLE_PON);
}

static int as7343_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct as7343_data *data = iio_priv(indio_dev);
	struct regmap *map = data->regmap;
	int ret;

	ret = regmap_set_bits(map, AS7343_ENABLE_REG, AS7343_ENABLE_PON);
	if (ret)
		return ret;

	return regmap_set_bits(map, AS7343_ENABLE_REG, AS7343_ENABLE_SP_EN);
}

static void as7343_suspend_action(void *dev)
{
	as7343_suspend(dev);
}

static int as7343_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct as7343_data *data;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);

	regmap = devm_regmap_init_i2c(client, &as7343_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	data = iio_priv(indio_dev);
	data->regmap = regmap;

	ret = devm_mutex_init(dev, &data->mutex);
	if (ret)
		return ret;

	indio_dev->name = "as7343";
	indio_dev->info = &as7343_info;
	indio_dev->channels = as7343_channels;
	indio_dev->num_channels = ARRAY_SIZE(as7343_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return ret;

	ret = as7343_setup_device(dev, data);
	if (ret)
		return ret;

	/* Enter ACTIVE statet and start measurements */
	ret = regmap_set_bits(regmap, AS7343_ENABLE_REG, AS7343_ENABLE_SP_EN);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, as7343_suspend_action, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add suspend action\n");

	pm_runtime_set_autosuspend_delay(dev, 3000);
	pm_runtime_use_autosuspend(dev);

	ret = devm_pm_runtime_set_active_enabled(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to activate PM runtime\n");

	return devm_iio_device_register(dev, indio_dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(as7343_pm_ops,
				 as7343_suspend, as7343_resume, NULL);

static const struct of_device_id as7343_of_match[] = {
	{ .compatible = "ams,as7343" },
	{ }
};
MODULE_DEVICE_TABLE(of, as7343_of_match);

static const struct i2c_device_id as7343_id[] = {
	{ .name = "as7343" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, as7343_id);

static struct i2c_driver as7343_driver = {
	.driver = {
		.name = "as7343",
		.of_match_table = as7343_of_match,
		.pm = pm_ptr(&as7343_pm_ops),
	},
	.probe = as7343_probe,
	.id_table = as7343_id,
};
module_i2c_driver(as7343_driver);

MODULE_AUTHOR("Chang Yu <marcus.yu.56@gmail.com>");
MODULE_DESCRIPTION("AS7343 14 Channel Multi-Spectral Sensor driver");
MODULE_LICENSE("GPL");
