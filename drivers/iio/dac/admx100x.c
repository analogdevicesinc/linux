// SPDX-License-Identifier: GPL-2.0
/*
 * ADMX1001/ADMX1002 Ultra-low Distortion Signal Generator IIO driver
 *
 * Copyright (C) 2025 Analog Devices Inc.
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
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>


/* Register Definitions */
#define ADMX_REG_CONTROL			0x000 // Some of the control bits are auto-cleared.
#define ADMX_STATUS_READY_STATUS         BIT(2)
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
#define ADMX_AMPLITUDE_SECONDARY  0x080
#define ADMX_FREQUENCY_SECONDARY_L  0x84
#define ADMX_FREQUENCY_SECONDARY_H  0x88

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
#define ADMX_SIGNAL_TYPE_DC			    0x1
#define ADMX_SIGNAL_TYPE_IMD			0x5

/* Constants */
#define ADMX_AMPLITUDE_MIN_UV			0    /* 0.1 Vrms */
#define ADMX_AMPLITUDE_MAX_UV			4200000   /* 4.2 Vrms */
#define ADMX_FREQUENCY_MIN_UHZ			10000000  /* 10 Hz */
#define ADMX_FREQUENCY_MAX_UHZ			40000000000ULL /* 40 kHz */

struct admx_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_ready;
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
	.pad_bits = 48,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
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
	int ret;

	ret = regmap_read(st->regmap, ADMX_REG_STATUS, &val);
	dev_info(&st->spi->dev, "ADMX_REG_STATUS = 0x%08x \n", val);
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

	dev_info(&st->spi->dev, "ADMX: set_task - setting task_id=0x%02x in CONTROL reg 0x%03x\n",
		task_id, ADMX_REG_CONTROL);
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				 ADMX_CONTROL_TASK_ID_MASK,
				 FIELD_PREP(ADMX_CONTROL_TASK_ID_MASK, task_id));
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: set_task - failed to set task_id (ret=%d)\n", ret);
		return ret;
	}

	/* Start task */
	dev_info(&st->spi->dev, "ADMX: set_task - starting task by setting TASK_CONTROL bit\n");
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				 ADMX_CONTROL_TASK_CONTROL,
				 ADMX_CONTROL_TASK_CONTROL);
	if (ret)
		dev_err(&st->spi->dev, "ADMX: set_task - failed to start task (ret=%d)\n", ret);

	return ret;
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
		u64 f_uhz = st->frequency_uhz;
		u32 rem_uhz;
		*val = div_u64_rem(f_uhz, 1000000, &rem_uhz);
		*val2 = rem_uhz / 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		dev_info(&st->spi->dev, "ADMX: READ_FREQUENCY =%llu uHz, returned=%d.%06d Hz\n",
		st->frequency_uhz, *val, *val2);
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_SCALE:
		/* Convert from µVrms to Vrms */
		*val = st->amplitude_uvrms / 1000000;
		*val2 = st->amplitude_uvrms % 1000000;
		dev_info(&st->spi->dev, "ADMX: READ SCALE=%u uVrms, returned=%d.%06d Vrms\n",
			st->amplitude_uvrms, *val, *val2);
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_ENABLE:
		*val = st->output_enabled;
		ret = IIO_VAL_INT;
		dev_info(&st->spi->dev, "ADMX: READ ENABLE =%d\n", *val);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);
	return ret;
}

static int admx_stop_output(struct admx_state *st)
{
	int ret;
	dev_info(&st->spi->dev, "ADMX: stop_output - clearing TASK_CONTROL bit in CONTROL reg 0x%03x\n",
		ADMX_REG_CONTROL);
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				  ADMX_CONTROL_TASK_CONTROL, 0);
	if (ret)
		dev_err(&st->spi->dev, "ADMX: stop_output - failed to stop (ret=%d)\n", ret);
	return ret;
}

static int admx_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int val, int val2, long mask)
{
	struct admx_state *st = iio_priv(indio_dev);
	u64 frequency;
	u32 amplitude;
	int ret=0;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
	/* Convert from Hz.mHz to µHz */
	frequency = (u64)val * 1000000 + val2 * 1000;
	dev_info(&st->spi->dev, "ADMX: WRITE_FREQUENCY - converted to %llu uHz (range: %u - %llu)\n",
	frequency, ADMX_FREQUENCY_MIN_UHZ, ADMX_FREQUENCY_MAX_UHZ);
	if (frequency < ADMX_FREQUENCY_MIN_UHZ ||
		frequency > ADMX_FREQUENCY_MAX_UHZ) {
		dev_err(&st->spi->dev, "ADMX: WRITE_FREQUENCY - INVALID range!\n");
		ret = -EINVAL;
		break;
	}
	/* Write frequency registers (64-bit value) */
	dev_info(&st->spi->dev, "ADMX: WRITE_FREQUENCY - writing L=0x%08x to reg 0x%03x\n",
	(u32)(frequency & 0xFFFFFFFF), ADMX_REG_SET_FREQUENCY_PRIMARY_L);
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L,
			frequency & 0xFFFFFFFF);
	if (ret) {
	dev_err(&st->spi->dev, "ADMX: WRITE_FREQUENCY - failed to write L register (ret=%d)\n", ret);
		break;
	}

	dev_info(&st->spi->dev, "ADMX: WRITE_FREQUENCY - writing H=0x%08x to reg 0x%03x\n",
	(u32)(frequency >> 32), ADMX_REG_SET_FREQUENCY_PRIMARY_H);
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H,
			frequency >> 32);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: WRITE_FREQUENCY - failed to write H register (ret=%d)\n", ret);
		break;
	}
	st->frequency_uhz = frequency;
	dev_info(&st->spi->dev, "ADMX: Frequency set to %llu uHz\n",st->frequency_uhz);
	break;

	case IIO_CHAN_INFO_SCALE:
	/* Convert from Vrms to µVrms */
	dev_info(&st->spi->dev, "ADMX: WRITE_AMPLITUDE - input val=%d val2=%d\n", val, val2);
	amplitude = val * 1000000 + val2;
	dev_info(&st->spi->dev, "ADMX: WRITE_AMPLITUDE - converted to %u uVrms (max: %u)\n",
	amplitude, ADMX_AMPLITUDE_MAX_UV);

	// if (amplitude < ADMX_AMPLITUDE_MIN_UV ||
	//     amplitude > ADMX_AMPLITUDE_MAX_UV) {
	// 	ret = -EINVAL;
	// 	break;
	// }
	if (amplitude > ADMX_AMPLITUDE_MAX_UV) {
		dev_err(&st->spi->dev, "ADMX: WRITE_AMPLITUDE - INVALID range!\n");
		ret = -EINVAL;
		break;
	}
	dev_info(&st->spi->dev, "ADMX: WRITE_AMPLITUDE - writing 0x%08x to reg 0x%03x\n",
	amplitude, ADMX_REG_SET_AMPLITUDE_PRIMARY);
	ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY,
			amplitude);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: WRITE_AMPLITUDE - failed to write register (ret=%d)\n", ret);
		break;
	}
	st->amplitude_uvrms = amplitude;
	dev_info(&st->spi->dev, "ADMX: Amplitude set to %u uVrms\n", st->amplitude_uvrms);
	break;

	case IIO_CHAN_INFO_ENABLE:
	dev_info(&st->spi->dev, "ADMX: WRITE_ENABLE - input val=%d (current state=%d)\n", val, st->output_enabled);
	if (val) {
		dev_info(&st->spi->dev, "ADMX: WRITE_ENABLE - enabling output.\n");
	ret = regmap_write(st->regmap, ADMX_REG_SET_SIGNAL_TYPE, 0);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: WRITE_ENABLE - failed to set SIGNAL_TYPE (ret=%d)\n", ret);
		break;
	}
	/* Validate parameters first */
	ret = admx_validate_params(st);
	if (ret)
		break;
	/* Start: task GENERATE_SIGNAL */
	ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Failed to enable output (ret=%d)\n", ret);
		break;
	}
	/* --- (D) Confirmare start: poll pe STATUS.SIGNAL_OUTPUT_STATUS --- */
	{
		unsigned int status;
		ret = regmap_read_poll_timeout(st->regmap, ADMX_REG_STATUS, status,
				!!(status & ADMX_STATUS_SIGNAL_OUTPUT),
				1000 /*us*/, 100000 /*us*/);
		if (ret) {
			/* --- (E) Retry scurt: VALIDATE + GENERATE din nou --- */
			dev_warn(&st->spi->dev,
					"ADMX: start not asserted (STATUS=0x%08x) – revalidate & retry\n", status);

			/* VALIDATE auto-clear (UG p.36–37) */
			regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
							ADMX_CONTROL_VALIDATE, ADMX_CONTROL_VALIDATE);

			ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
			if (ret) {
				dev_err(&st->spi->dev, "ADMX: retry start failed (ret=%d)\n", ret);
				break;
			}

			// ret = regmap_read_poll_timeout(st->regmap, ADMX_REG_STATUS, status,
			// 		!!(status & ADMX_STATUS_SIGNAL_OUTPUT),
			// 		1000 /*us*/, 100000 /*us*/);
			// if (ret) {
			// 	dev_err(&st->spi->dev,
			// 			"ADMX: start failed – STATUS=0x%08x\n", status);
			// 	st->output_enabled = false;
			// 	ret = -EIO;
			// 	break;
			// }
		}
	}

	st->output_enabled = true;
	dev_info(&st->spi->dev, "ADMX: Output ENABLED successfully\n");

	} else {
	dev_info(&st->spi->dev, "ADMX: WRITE_ENABLE - disabling output...\n");
	ret = admx_stop_output(st);
	if (!ret) {
		st->output_enabled = false;
		dev_info(&st->spi->dev, "ADMX: Output DISABLED successfully\n");
	} else {
		dev_err(&st->spi->dev, "ADMX: Failed to disable output (ret=%d)\n", ret);
	}
	}
	break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);
	return ret;
}
/* Start signal generation */
//dev_info(&st->spi->dev, "ADMX: WRITE_ENABLE - setting task GENERATE_SIGNAL (0x%02x)\n", ADMX_TASK_GENERATE_SIGNAL);
// 	ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
// 	if (!ret) {
// 		st->output_enabled = true;
// 		dev_info(&st->spi->dev, "ADMX: Output ENABLED successfully\n");
// 	} else {
// 		dev_err(&st->spi->dev, "ADMX: Failed to enable output (ret=%d)\n", ret);
// 	}
// 	} else {
// 	dev_info(&st->spi->dev, "ADMX: WRITE_ENABLE - disabling output...\n");
// 	ret = admx_stop_output(st);
// 	if (!ret) {
// 		st->output_enabled = false;
// 		dev_info(&st->spi->dev, "ADMX: Output DISABLED successfully\n");
// 	} else {
// 		dev_err(&st->spi->dev, "ADMX: Failed to disable output (ret=%d)\n", ret);
// 	}
// 	}
// 	break;

// 	default:
// 		ret = -EINVAL;
// 	}

// 	mutex_unlock(&st->lock);
// 	return ret;
// }

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
	/* Remove duplicate enable - use IIO channel enable only */
	NULL,
};

static const struct attribute_group admx_attribute_group = {
	.attrs = admx_attributes,
};

static int admx100x_debugfs_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int writeval, unsigned int *readval)
{
	struct admx_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info admx_info = {
	.read_raw = admx_read_raw,
	.write_raw = admx_write_raw,
	.attrs = &admx_attribute_group,
	.debugfs_reg_access = admx100x_debugfs_reg_access,
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
	int ret = 0;

	//Wait for module to be ready
	ret = admx_wait_ready(st, 120000); //2 min
	if (ret) {
		dev_err(&st->spi->dev, "Module not ready\n");
		return ret;
	}
	//Read module ID
	ret = regmap_read(st->regmap, ADMX_REG_MODULE_ID, &val);
	if (ret){
		dev_err(&st->spi->dev, "Failed to read Module ID\n");
        return ret;
	}
	dev_info(&st->spi->dev, "Module ID: 0x%08x\n", val);

	//Set default values
	st->amplitude_uvrms = 1000000; //1 Vrms
	st->frequency_uhz = 1000000000; //1 kHz
	st->signal_type = ADMX_SIGNAL_TYPE_SINE;
	st->output_enabled = false;

	// Program hardware registers with defaults
	ret = regmap_write(st->regmap, ADMX_REG_SET_SIGNAL_TYPE,
		st->signal_type);
	if (ret)
	return ret;

	ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY,
			st->amplitude_uvrms);
	if (ret)
	return ret;

	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L,
			st->frequency_uhz & 0xFFFFFFFF);
	if (ret)
	return ret;

	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H,
			st->frequency_uhz >> 32);
	if (ret)
	return ret;

	dev_info(&st->spi->dev, "ADMX: defaults set (amp=%u uVrms, freq=%llu uHz, type=%d)\n",
	st->amplitude_uvrms, st->frequency_uhz, st->signal_type);
	//return 0;

	/* Step 1 */
	ret = regmap_write(st->regmap, ADMX_REG_PROFILE_ID, 0x0000000E);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX_REG_PROFILE_ID failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_PROFILE_ID, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 1 ADMX_REG_PROFILE_ID failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 1 ADMX_REG_PROFILE_ID(0x068) = 0x%08x (expected 0x0000000E)\n", val);
	/* Step 2 */
	ret = regmap_write(st->regmap, 0x104, 0x3);
	if (ret) {
		dev_err(&st->spi->dev, "Reg 0x104 failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, 0x104, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 2 Reg 0x104 failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 2 Reg 0x104 = 0x%08x (expected 0x3)\n", val);
	/* Step 3 */
	ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY, 0xF4240);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_SET_AMPLITUDE_PRIMARY failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 3 ADMX_REG_SET_AMPLITUDE_PRIMARY failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 3 Reg ADMX_REG_SET_AMPLITUDE_PRIMARY = 0x%08x (expected 0xF4240)\n", val);
	/* Step 4 */
	ret = regmap_write(st->regmap, ADMX_AMPLITUDE_SECONDARY, 0x3D090);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_AMPLITUDE_SECONDARY failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_AMPLITUDE_SECONDARY, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 4 ADMX_AMPLITUDE_SECONDARY failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 4 Reg ADMX_AMPLITUDE_SECONDARY = 0x%08x (expected 0x3D090)\n", val);
	/* Step 5 */
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L, 0x3B9ACA00);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_SET_FREQUENCY_PRIMARY_L failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 5 Reg ADMX_REG_SET_FREQUENCY_PRIMARY_L failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 5 Reg ADMX_REG_SET_FREQUENCY_PRIMARY_L = 0x%08x (expected 0x3B9ACA00)\n", val);
	/* Step 6 */
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H, 0x0);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_SET_FREQUENCY_PRIMARY_H failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 6 Reg ADMX_REG_SET_FREQUENCY_PRIMARY_H failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 6 Reg ADMX_REG_SET_FREQUENCY_PRIMARY_H = 0x%08x (expected 0x0)\n", val);
	/* Step 7 */
	ret = regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_L, 0xA13B8600);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_FREQUENCY_SECONDARY_L failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_FREQUENCY_SECONDARY_L, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 7 Reg ADMX_FREQUENCY_SECONDARY_L failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 7 Reg ADMX_FREQUENCY_SECONDARY_L = 0x%08x (expected 0xA13B8600)\n", val);
	/* Step 8 */
	ret = regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_H, 0x01);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_FREQUENCY_SECONDARY_H failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_FREQUENCY_SECONDARY_H, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 8 Reg ADMX_FREQUENCY_SECONDARY_H failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 8 Reg ADMX_FREQUENCY_SECONDARY_H = 0x%08x (expected 0x01)\n", val);
	/* Step 9 */
	ret = regmap_write(st->regmap, ADMX_REG_SET_CYCLE_COUNT, 0xFFFFFFFF);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_SET_CYCLE_COUNT failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_SET_CYCLE_COUNT, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 9 Reg ADMX_REG_SET_CYCLE_COUNT failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 9 Reg ADMX_REG_SET_CYCLE_COUNT = 0x%08x (expected 0xFFFFFFFF)\n", val);
	/* Step 10 */
	ret = regmap_write(st->regmap, 0x50, 0x2710);
	if (ret) {
		dev_err(&st->spi->dev, "Reg 0x50 failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, 0x50, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 10 Reg 0x50 failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 10 Reg 0x50 = 0x%08x (expected 0x2710)\n", val);
	/* Step 11 */
	ret = regmap_write(st->regmap, 0x54, 0x40420F00);
	if (ret) {
		dev_err(&st->spi->dev, "Reg 0x54 failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, 0x54, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 11 Reg 0x54 failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 11 Reg 0x54 = 0x%08x (expected 0x40420F00)\n", val);
	/* Step 12 */
	ret = regmap_write(st->regmap, 0x58, 0x0);
	if (ret) {
		dev_err(&st->spi->dev, "Reg 0x58 failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, 0x58, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 12 Reg 0x58 failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 12 Reg 0x58 = 0x%08x (expected 0x0)\n", val);
	/* Step 13 */
	ret = regmap_write(st->regmap, ADMX_REG_SET_SIGNAL_TYPE, 0x0);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_SET_SIGNAL_TYPE failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, 0x104, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 13 Reg ADMX_REG_SET_SIGNAL_TYPE failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 13 Reg ADMX_REG_SET_SIGNAL_TYPE = 0x%08x (expected 0x0)\n", val);
	/* Step 14
	ret = regmap_write(st->regmap, ADMX_REG_STATUS, 0x03);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_STATUS failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_STATUS, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 14 Reg ADMX_REG_STATUS failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 14 Reg ADMX_REG_STATUS = 0x%08x (expected 0x03)\n", val);
	Step 15 */
	ret = regmap_write(st->regmap, ADMX_REG_CONTROL, 0x05);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_CONTROL, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 15 Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 15 Reg ADMX_REG_CONTROL = 0x%08x (expected 0x05)\n", val);
	/* Step 16 */
	ret = regmap_write(st->regmap, ADMX_REG_CONTROL, 0x400);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_CONTROL, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 16 Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 16 Reg ADMX_REG_CONTROL = 0x%08x (expected 0x400)\n", val);
	/* Step 17 */
	ret = regmap_write(st->regmap, ADMX_REG_CONTROL, 0x401);
	if (ret) {
		dev_err(&st->spi->dev, "Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	ret = regmap_read(st->regmap, ADMX_REG_CONTROL, &val);
	if (ret) {
		dev_err(&st->spi->dev, "Step 17 Reg ADMX_REG_CONTROL failed: %d\n", ret);
		return ret;
	}
	dev_info(&st->spi->dev, "Step 17 Reg ADMX_REG_CONTROL = 0x%08x (expected 0x401)\n", val);

    dev_info(&st->spi->dev, "ADMX init sequence completed.\n");
    return 0;
}

static int admx_read_status(struct regmap *map)
{
    unsigned int status;
    int ret = regmap_read(map, ADMX_REG_STATUS, &status);
    if (ret)
        return ret;
    return status;
}

// static int admx_post_reset_fixup(struct admx_state *st)
// {
//     int ret;
//     unsigned int status;

//     /* 1) Impune un SIGNAL_TYPE valid. Simplu: SINE=0 */
//     ret = regmap_write(st->regmap, ADMX_REG_SET_SIGNAL_TYPE, 0);
//     if (ret) {
//         dev_err(&st->spi->dev, "ADMX: fixup - write SIGNAL_TYPE failed (ret=%d)\n", ret);
//         return ret;
//     }

//     /* 2) Rulează VALIDATE ca în UG (actualizează Generated_* și validitățile) */
//     ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
//                              ADMX_CONTROL_VALIDATE, ADMX_CONTROL_VALIDATE);
//     if (ret) {
//         dev_err(&st->spi->dev, "ADMX: fixup - CONTROL.VALIDATE write failed (ret=%d)\n", ret);
//         return ret;
//     }

//     /* 3) Citește STATUS și verifică validitățile */
//     ret = regmap_read(st->regmap, ADMX_REG_STATUS, &status);
//     if (ret) {
//         dev_err(&st->spi->dev, "ADMX: fixup - STATUS read failed (ret=%d)\n", ret);
//         return ret;
//     }

//     if (!(status & ADMX_STATUS_AMPLITUDE_VALID) ||
//         !(status & ADMX_STATUS_FREQUENCY_VALID) ||
//         !(status & ADMX_STATUS_SIGNAL_CYCLE_VALID)) {
//         dev_warn(&st->spi->dev, "ADMX: fixup - params invalid after reset (STATUS=0x%08x)\n", status);
//         /* nu facem hard-fail; doar logăm */
//     } else {
//         dev_info(&st->spi->dev, "ADMX: fixup - params valid (STATUS=0x%08x)\n", status);
//     }

//     return 0;
// }

static int admx_hw_reset(struct admx_state *st)
{
    int ret, val;
    /* 1) Obține GPIO-uri (optional) din DT */
    if (!st->gpio_reset) {
        st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset", GPIOD_OUT_LOW);
        if (IS_ERR(st->gpio_reset))
            return dev_err_probe(&st->spi->dev, PTR_ERR(st->gpio_reset), "ADMX: reset gpio get failed");
    }
    if (!st->gpio_ready) {
        st->gpio_ready = devm_gpiod_get_optional(&st->spi->dev, "ready", GPIOD_IN);
        if (IS_ERR(st->gpio_ready))
            st->gpio_ready = NULL;
    }

    dev_info(&st->spi->dev, "ADMX: HW reset - toggling reset/en pin\n");

    /* 2) Toggling simplu: LOW -> 10..20us -> HIGH*/
    if (st->gpio_reset) {
        gpiod_set_value_cansleep(st->gpio_reset, 1);
        msleep(1000);
        gpiod_set_value_cansleep(st->gpio_reset, 0);
		msleep(8000);
    } else {
        dev_warn(&st->spi->dev, "ADMX: no reset gpio; please add 'reset-gpios' in DT");
        return -ENODEV;
    }

    /* 3) Poll READY (GPIO dacă există, altfel STATUS.READY_STATUS) */
    if (st->gpio_ready) {
        ret = readx_poll_timeout(gpiod_get_value_cansleep, st->gpio_ready, val,
                                 val == 1, 5000 /*us*/, 5000000 /*us*/);
        if (ret) {
            dev_err(&st->spi->dev, "ADMX: HW reset - READY GPIO timeout");
            return ret;
        }
    } else {
        struct regmap *map = st->regmap;
        ret = readx_poll_timeout(admx_read_status, map, val,
                                 !!(val & ADMX_STATUS_READY_STATUS),
                                 5000 /*us*/, 5000000 /*us*/);
        if (ret) {
            dev_err(&st->spi->dev, "ADMX: HW reset - STATUS READY timeout");
            return ret;
        }
    }
    /* 4) Curăta cache/flaguri locale */
    st->output_enabled = 0;
	//admx_post_reset_fixup(st);
	regmap_update_bits(st->regmap, ADMX_REG_SET_SIGNAL_TYPE, 0xF, 0x0);
	regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
					ADMX_CONTROL_VALIDATE, ADMX_CONTROL_VALIDATE);
    dev_info(&st->spi->dev, "ADMX: HW reset complete\n");
    return 0;
}

static int admx_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct admx_state *st;
	//struct gpio_desc *reset_gpio;
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
	// reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	// gpiod_set_value(reset_gpio, 0);
	// fsleep(1000);
	// gpiod_set_value(reset_gpio, 1);
	// fsleep(3000);
	// if (reset_gpio) {
	// 	/* ASSERT reset */
    //     gpiod_set_value_cansleep(reset_gpio, 1);
    //     usleep_range(2000, 4000);    /* ~2 ms */
    //     /* DEASSERT reset */
    //     gpiod_set_value_cansleep(reset_gpio, 0);
    //     usleep_range(5000, 8000);
    // }
	ret = admx_hw_reset(st);
	if (ret)
		return ret;

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

MODULE_AUTHOR("Capota Ramona-Bianca <Bianca-ramona.Capota@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADMX1001/ADMX1002 Signal Generator driver");
MODULE_LICENSE("GPL v2");
