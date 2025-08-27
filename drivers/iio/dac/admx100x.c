// SPDX-License-Identifier: GPL-2.0
/*
 * ADMX1001/ADMX1002 Ultra-low Distortion Signal Generator IIO driver
 *
 * Copyright (C) 2024 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/bitfield.h>
#include <linux/kstrtox.h>
#include <linux/math64.h>

/* Register Definitions */
#define ADMX_REG_CONTROL			0x000 // Some of the control bits are auto-cleared.
#define ADMX_REG_STATUS				0x004 // this register provides info of the status of the module
#define ADMX_REG_VERSION_NUMBERS		0x008
#define ADMX_REG_MODULE_ID			0x00C
#define ADMX_REG_GEN_AMPLITUDE_PRIMARY		0x014
#define ADMX_REG_GEN_FREQUENCY_PRIMARY_L	0x018
#define ADMX_REG_GEN_FREQUENCY_PRIMARY_H	0x01C
#define ADMX_REG_GEN_CYCLE_COUNT		0x020
#define ADMX_REG_SET_AMPLITUDE_PRIMARY		0x040 //holds the desired amplitude programmed by the user.
#define ADMX_REG_SET_FREQUENCY_PRIMARY_L	0x044
#define ADMX_REG_SET_FREQUENCY_PRIMARY_H	0x048 //holds the upper 32-bit value of the desired frequency programmed by the user.
#define ADMX_REG_SET_CYCLE_COUNT		0x04C //holds the desired number of cycles programmed by the user.
#define ADMX_REG_SET_SIGNAL_TYPE		0x05C //holds the type of the set signal
#define ADMX_REG_SOFT_RESET			0x064
#define ADMX_REG_PROFILE_ID			0x068 //this register holds the ID for a specific profile

/* Control Register Bits */
#define ADMX_CONTROL_TASK_ID_MASK		GENMASK(15, 8)
#define ADMX_CONTROL_SKIP_PRE_CAL		BIT(4)
#define ADMX_CONTROL_PROFILE_LOAD_EN		BIT(3)
#define ADMX_CONTROL_VALIDATE			BIT(1)
#define ADMX_CONTROL_TASK_CONTROL		BIT(0)

/* Task IDs */
#define ADMX_TASK_IDLE				0x00
#define ADMX_TASK_LOAD_PROFILE			0x01
#define ADMX_TASK_SAVE_PROFILE			0x02
#define ADMX_TASK_CALIBRATE_SIGNAL		0x03
#define ADMX_TASK_GENERATE_SIGNAL		0x04
#define ADMX_TASK_RUN_AWG			0x0B

/* Status Register Bits */
#define ADMX_STATUS_SIGNAL_CYCLE_VALID		BIT(18)
#define ADMX_STATUS_FREQUENCY_VALID		BIT(17)
#define ADMX_STATUS_AMPLITUDE_VALID		BIT(16)
#define ADMX_STATUS_TASK_RESULT			BIT(3)
#define ADMX_STATUS_READY			BIT(2)
#define ADMX_STATUS_SIGNAL_OUTPUT		BIT(1)
#define ADMX_STATUS_CALIBRATION			BIT(0)

/* Signal Types */
#define ADMX_SIGNAL_TYPE_SINE			0x0
#define ADMX_SIGNAL_TYPE_DC			0x1
#define ADMX_SIGNAL_TYPE_IMD			0x5

/* Constants */
#define ADMX_AMPLITUDE_MIN_UV			100000    /* 0.1 Vrms */
#define ADMX_AMPLITUDE_MAX_UV			4200000   /* 4.2 Vrms */
#define ADMX_FREQUENCY_MIN_UHZ			10000000  /* 10 Hz */
#define ADMX_FREQUENCY_MAX_UHZ			40000000000ULL /* 40 kHz */

struct admx_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
	u32 amplitude_uvrms;
	u64 frequency_uhz;
	u8 signal_type;
	u8 read_flag_mask;
	u8 write_flag_mask;
	int pad_bits;
	bool output_enabled;
};

static const struct regmap_config admx_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.read_flag_mask = 0x03,
	.write_flag_mask = 0x02,
	.pad_bits = 40,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = 0x3F4,
};

static const char * const admx_signal_types[] = {
	[ADMX_SIGNAL_TYPE_SINE] = "sine",
	[ADMX_SIGNAL_TYPE_DC] = "dc",
	[ADMX_SIGNAL_TYPE_IMD] = "dual_tone",
};

static int admx_wait_ready(struct admx_state *st, unsigned int timeout_ms)
{
	unsigned int val;
	//int ret;
	return regmap_read_poll_timeout(st->regmap, ADMX_REG_STATUS, val,
					val & ADMX_STATUS_READY,
					1000, timeout_ms * 1000);
}

static int admx_validate_params(struct admx_state *st)
{
	unsigned int val;
	int ret;

	/* Set validate bit */
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				 ADMX_CONTROL_VALIDATE,
				 ADMX_CONTROL_VALIDATE);
	if (ret)
		return ret;

	/* Wait for validation to complete */
	msleep(10);

	/* Check validation results */
	ret = regmap_read(st->regmap, ADMX_REG_STATUS, &val);
	if (ret)
		return ret;

	if (!(val & ADMX_STATUS_AMPLITUDE_VALID))
		return -EINVAL;

	if (!(val & ADMX_STATUS_FREQUENCY_VALID))
		return -EINVAL;

	return 0;
}

static int admx_set_task(struct admx_state *st, u8 task_id)
{
	int ret;

	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				 ADMX_CONTROL_TASK_ID_MASK,
				 FIELD_PREP(ADMX_CONTROL_TASK_ID_MASK, task_id));
	if (ret)
		return ret;

	/* Start task */
	return regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				  ADMX_CONTROL_TASK_CONTROL,
				  ADMX_CONTROL_TASK_CONTROL);
}

static int admx_stop_output(struct admx_state *st)
{
	return regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				  ADMX_CONTROL_TASK_CONTROL, 0);
}

static int admx_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int *val, int *val2, long mask)
{
	struct admx_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		/* Convert from µHz to Hz.mHz */
		/**val = st->frequency_uhz / 1000000;
		*val2 = (st->frequency_uhz % 1000000) / 1000;*/
		u64 f_uhz = st->frequency_uhz;
   		u32 rem_uhz;
    	/* Împarte corect fără apel la __aeabi_uldivmod */
    	*val = div_u64_rem(f_uhz, 1000000, &rem_uhz); /* partea întreagă în Hz */
    	*val2 = rem_uhz / 1000;                       /* partea fracționară în mHz */

		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_SCALE:
		/* Convert from µVrms to Vrms */
		*val = st->amplitude_uvrms / 1000000;
		*val2 = st->amplitude_uvrms % 1000000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_ENABLE:
		*val = st->output_enabled;
		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);
	return ret;
}

static int admx_write_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int val, int val2, long mask)
{
	struct admx_state *st = iio_priv(indio_dev);
	u64 frequency;
	u32 amplitude;
	int ret;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		/* Convert from Hz.mHz to µHz */
		frequency = (u64)val * 1000000 + val2 * 1000;

		if (frequency < ADMX_FREQUENCY_MIN_UHZ ||
		    frequency > ADMX_FREQUENCY_MAX_UHZ) {
			ret = -EINVAL;
			break;
		}

		/* Write frequency registers (64-bit value) */
		ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L,
				   frequency & 0xFFFFFFFF);
		if (ret)
			break;

		ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H,
				   frequency >> 32);
		if (ret)
			break;

		st->frequency_uhz = frequency;
		break;

	case IIO_CHAN_INFO_SCALE:
		/* Convert from Vrms to µVrms */
		amplitude = val * 1000000 + val2;

		if (amplitude < ADMX_AMPLITUDE_MIN_UV ||
		    amplitude > ADMX_AMPLITUDE_MAX_UV) {
			ret = -EINVAL;
			break;
		}

		ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY,
				   amplitude);
		if (ret)
			break;

		st->amplitude_uvrms = amplitude;
		break;

	case IIO_CHAN_INFO_ENABLE:
		if (val) {
			/* Validate parameters first */
			ret = admx_validate_params(st);
			if (ret)
				break;

			/* Start signal generation */
			ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
			if (!ret)
				st->output_enabled = true;
		} else {
			ret = admx_stop_output(st);
			if (!ret)
				st->output_enabled = false;
		}
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t admx_signal_type_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admx_state *st = iio_priv(indio_dev);
	int type;

	mutex_lock(&st->lock);
	type = st->signal_type;
	mutex_unlock(&st->lock);

	if (type < ARRAY_SIZE(admx_signal_types) && admx_signal_types[type])
		return sprintf(buf, "%s\n", admx_signal_types[type]);

	return sprintf(buf, "unknown\n");
}

static ssize_t admx_signal_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admx_state *st = iio_priv(indio_dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(admx_signal_types); i++) {
		if (!admx_signal_types[i])
			continue;
		if (sysfs_streq(buf, admx_signal_types[i]))
			break;
	}

	if (i == ARRAY_SIZE(admx_signal_types))
		return -EINVAL;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, ADMX_REG_SET_SIGNAL_TYPE,
				 GENMASK(3, 0), i);
	if (!ret)
		st->signal_type = i;

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t admx_signal_types_available_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	int i, len = 0;

	for (i = 0; i < ARRAY_SIZE(admx_signal_types); i++) {
		if (admx_signal_types[i])
			len += sprintf(buf + len, "%s ", admx_signal_types[i]);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t admx_calibrate_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admx_state *st = iio_priv(indio_dev);
	bool val;
	int ret;

	//ret = strtobool(buf, &val);
	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (!val)
		return len;

	mutex_lock(&st->lock);

	/* Run calibration (DPD) */
	ret = admx_set_task(st, ADMX_TASK_CALIBRATE_SIGNAL);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	/* Wait for calibration to complete (up to 2 minutes) */
	ret = admx_wait_ready(st, 120000);

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(signal_type, 0644,
		       admx_signal_type_show,
		       admx_signal_type_store, 0);

static IIO_DEVICE_ATTR(signal_types_available, 0444,
		       admx_signal_types_available_show, NULL, 0);

static IIO_DEVICE_ATTR(calibrate, 0200, NULL, admx_calibrate_store, 0);

static struct attribute *admx_attributes[] = {
	&iio_dev_attr_signal_type.dev_attr.attr,
	&iio_dev_attr_signal_types_available.dev_attr.attr,
	&iio_dev_attr_calibrate.dev_attr.attr,
	NULL,
};

static const struct attribute_group admx_attribute_group = {
	.attrs = admx_attributes,
};

static const struct iio_info admx_info = {
	.read_raw = admx_read_raw,
	.write_raw = admx_write_raw,
	.attrs = &admx_attribute_group,
};

static const struct iio_chan_spec admx_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.channel = 0,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_ENABLE),
	}
};

static int admx_init(struct admx_state *st)
{
	unsigned int val;
	int ret;

	/* Wait for module to be ready */
	ret = admx_wait_ready(st, 5000);
	if (ret) {
		dev_err(&st->spi->dev, "Module not ready\n");
		return ret;
	}

	/* Read module ID */
	ret = regmap_read(st->regmap, ADMX_REG_MODULE_ID, &val);
	if (ret)
		return ret;

	dev_info(&st->spi->dev, "Module ID: 0x%08x\n", val);

	/* Set default values */
	st->amplitude_uvrms = 1000000; /* 1 Vrms */
	st->frequency_uhz = 1000000000; /* 1 kHz */
	st->signal_type = ADMX_SIGNAL_TYPE_SINE;
	st->output_enabled = false;

	/* Set continuous mode (cycles = 0xFFFFFFFF) */
	ret = regmap_write(st->regmap, ADMX_REG_SET_CYCLE_COUNT, 0xFFFFFFFF);
	if (ret)
		return ret;

	/* Set signal type */
	return regmap_write(st->regmap, ADMX_REG_SET_SIGNAL_TYPE,
			    st->signal_type);

	/*Write PROFILE_ID*/
	ret = regmap_write(st->regmap, ADMX_REG_PROFILE_ID, 0xE);
	if (ret)
		return ret;

	/*Write amplitude_primary*/
	ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY, 0x056511);
	if (ret)
		return ret;

	/*Write frequency_primary_l*/
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L, 0x3B9ACA00);
	if (ret)
		return ret;

	/*Write frequency_primary_h*/
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H, 0);
	if (ret)
		return ret;

	/*Write frequency_primary_h*/
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H, 0);
	if (ret)
		return ret;

	/*Write reg control*/
	ret = regmap_write(st->regmap, ADMX_REG_CONTROL, 0x3);
	if (ret)
		return ret;
}

static int admx_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admx_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_spi(spi, &admx_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = admx_channels;
	indio_dev->num_channels = ARRAY_SIZE(admx_channels);
	indio_dev->info = &admx_info;

	ret = admx_init(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id admx_id[] = {
	{ "admx1001", 0 },
	{ "admx1002", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, admx_id);

static const struct of_device_id admx_of_match[] = {
	{ .compatible = "adi,admx1001" },
	{ .compatible = "adi,admx1002" },
	{ }
};
MODULE_DEVICE_TABLE(of, admx_of_match);

static struct spi_driver admx_driver = {
	.driver = {
		.name = "admx100x",
		.of_match_table = admx_of_match,
	},
	.probe = admx_probe,
	.id_table = admx_id,
};
module_spi_driver(admx_driver);

MODULE_AUTHOR("Your Name <your.email@example.com>");
MODULE_DESCRIPTION("Analog Devices ADMX1001/ADMX1002 Signal Generator driver");
MODULE_LICENSE("GPL v2");
