// SPDX-License-Identifier: GPL-2.0
/*
 * ADMX1001/ADMX1002 Ultra-low Distortion Signal Generator IIO driver
 *
 * Copyright (C) 2025-2026 Analog Devices Inc.
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
 #define ADMX_REG_CONTROL					0x000
 #define ADMX_STATUS_READY_STATUS           BIT(2)
 #define ADMX_REG_STATUS					0x004
 #define ADMX_REG_VERSION_NUMBERS			0x008
 #define ADMX_REG_MODULE_ID					0x00C
 #define ADMX_REG_GEN_AMPLITUDE_PRIMARY		0x014
 #define ADMX_REG_GEN_FREQUENCY_PRIMARY_L	0x018
 #define ADMX_REG_GEN_FREQUENCY_PRIMARY_H	0x01C
 #define ADMX_REG_GEN_CYCLE_COUNT			0x020
 #define ADMX_REG_SET_AMPLITUDE_PRIMARY		0x040
 #define ADMX_REG_SET_FREQUENCY_PRIMARY_L	0x044
 #define ADMX_REG_SET_FREQUENCY_PRIMARY_H	0x048
 #define ADMX_REG_SET_CYCLE_COUNT			0x04C
 #define ADMX_REG_SET_SIGNAL_TYPE			0x05C
 #define ADMX_REG_PROFILE_ID				0x068
 #define ADMX_AMPLITUDE_SECONDARY  			0x080
 #define ADMX_FREQUENCY_SECONDARY_L  		0x84
 #define ADMX_FREQUENCY_SECONDARY_H  		0x88

 /* Control Register Bits */
 #define ADMX_CONTROL_TASK_ID_MASK			GENMASK(15, 8)
 #define ADMX_CONTROL_SKIP_PRE_CAL			BIT(4)
 #define ADMX_CONTROL_PROFILE_LOAD_EN		BIT(3)
 #define ADMX_CONTROL_VALIDATE				BIT(1)
 #define ADMX_CONTROL_TASK_CONTROL			BIT(0)

 /* Task IDs */
 #define ADMX_TASK_IDLE						0x00
 #define ADMX_TASK_LOAD_PROFILE				0x01
 #define ADMX_TASK_SAVE_PROFILE				0x02
 #define ADMX_TASK_CALIBRATE_SIGNAL			0x03
 #define ADMX_TASK_GENERATE_SIGNAL			0x04
 #define ADMX_TASK_RUN_AWG					0x0B

 /* Status Register Bits */
 #define ADMX_STATUS_SIGNAL_CYCLE_VALID		BIT(18)
 #define ADMX_STATUS_FREQUENCY_VALID		BIT(17)
 #define ADMX_STATUS_AMPLITUDE_VALID		BIT(16)
 #define ADMX_STATUS_TASK_RESULT			BIT(3)
 #define ADMX_STATUS_READY					BIT(2)
 #define ADMX_STATUS_SIGNAL_OUTPUT			BIT(1)
 #define ADMX_STATUS_CALIBRATION			BIT(0)

 /* Signal Types */
 #define ADMX_SIGNAL_TYPE_SINE				0x0
 #define ADMX_SIGNAL_TYPE_DC				0x1
 #define ADMX_SIGNAL_TYPE_IMD				0x5

 /* Constants */
 #define ADMX_AMPLITUDE_MIN_UV				0
 #define ADMX_AMPLITUDE_MAX_UV				4200000
 #define ADMX_FREQUENCY_MIN_UHZ				10000000
 #define ADMX_FREQUENCY_MAX_UHZ				40000000000ULL

struct admx_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_ready;
	u32 amplitude_uvrms;
	u32 amplitude2_uvrms;
	u64 frequency_uhz;
	u64 frequency2_uhz;
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
	"sine",
	"dc",
	"dual_tone",
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

	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				ADMX_CONTROL_VALIDATE,
				ADMX_CONTROL_VALIDATE);
	if (ret)
		return ret;

	msleep(10);

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

	dev_info(&st->spi->dev, "ADMX: set_task - starting task by setting TASK_CONTROL bit\n");
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				 ADMX_CONTROL_TASK_CONTROL,
				 ADMX_CONTROL_TASK_CONTROL);
	if (ret)
		dev_err(&st->spi->dev, "ADMX: set_task - failed to start task (ret=%d)\n", ret);

	return ret;
}

static int admx_get_signal_type(struct iio_dev *indio_dev,
								 const struct iio_chan_spec *chan)
{
	struct admx_state *st = iio_priv(indio_dev);
	unsigned int reg, hw;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, ADMX_REG_SET_SIGNAL_TYPE, &reg);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;

	hw = reg & GENMASK(3, 0);
	st->signal_type = hw;

	if (hw == ADMX_SIGNAL_TYPE_SINE)  return 0;
	if (hw == ADMX_SIGNAL_TYPE_DC)    return 1;
	if (hw == ADMX_SIGNAL_TYPE_IMD)   return 2;

	return -EINVAL;
}

static int admx_set_signal_type(struct iio_dev *indio_dev,
								 const struct iio_chan_spec *chan,
								 unsigned int type)
 {
	struct admx_state *st = iio_priv(indio_dev);
	unsigned int hw;
	int ret;

	if (type >= ARRAY_SIZE(admx_signal_types))
		return -EINVAL;

	if (type == 0)
		hw = ADMX_SIGNAL_TYPE_SINE;
	else if (type == 1)
		hw = ADMX_SIGNAL_TYPE_DC;
	else
		hw = ADMX_SIGNAL_TYPE_IMD;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->regmap, ADMX_REG_SET_SIGNAL_TYPE,
				GENMASK(3,0), hw);
	if (!ret)
		st->signal_type = hw;
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_enum admx_signal_type_enum = {
	.items = admx_signal_types,
	.num_items = ARRAY_SIZE(admx_signal_types),
	.get = admx_get_signal_type,
	.set = admx_set_signal_type,
};

static int admx_read_raw(struct iio_dev *indio_dev,
			 struct iio_chan_spec const *chan,
			 int *val, int *val2, long mask)
{
	 struct admx_state *st = iio_priv(indio_dev);
	 int ret;

	 mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		u64 f_uhz = st->frequency_uhz;
		u32 rem_uhz;
		*val = div_u64_rem(f_uhz, 1000000, &rem_uhz);
		*val2 = rem_uhz / 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		dev_info(&st->spi->dev, "ADMX: Read frequency =%llu uHz, returned=%d.%06d Hz\n",
		st->frequency_uhz, *val, *val2);
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_SCALE:
		*val = st->amplitude_uvrms / 1000000;
		*val2 = st->amplitude_uvrms % 1000000;
		dev_info(&st->spi->dev, "ADMX: Read scale =%u uVrms, returned=%d.%06d Vrms\n",
			st->amplitude_uvrms, *val, *val2);
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;

	case IIO_CHAN_INFO_ENABLE:
		*val = st->output_enabled;
		ret = IIO_VAL_INT;
		dev_info(&st->spi->dev, "ADMX: Read enable =%d\n", *val);
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
	dev_info(&st->spi->dev, "ADMX: Stop output - clearing TASK_CONTROL bit in CONTROL reg 0x%03x\n",
		ADMX_REG_CONTROL);
	ret = regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
				ADMX_CONTROL_TASK_CONTROL, 0);
	if (ret)
		dev_err(&st->spi->dev, "ADMX: Stop output - failed to stop (ret=%d)\n", ret);
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
	frequency = (u64)val * 1000000 + val2 * 1000;
	dev_info(&st->spi->dev, "ADMX: Write Frequency - converted to %llu uHz (range: %u - %llu)\n",
	frequency, ADMX_FREQUENCY_MIN_UHZ, ADMX_FREQUENCY_MAX_UHZ);
	if (frequency < ADMX_FREQUENCY_MIN_UHZ ||
		frequency > ADMX_FREQUENCY_MAX_UHZ) {
		dev_err(&st->spi->dev, "ADMX: Write Frequency - INVALID range!\n");
		ret = -EINVAL;
		break;
	}

	dev_info(&st->spi->dev, "ADMX: Write Frequency - writing L=0x%08x to reg 0x%03x\n",
	(u32)(frequency & 0xFFFFFFFF), ADMX_REG_SET_FREQUENCY_PRIMARY_L);
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_L,
			frequency & 0xFFFFFFFF);
	if (ret) {
	dev_err(&st->spi->dev, "ADMX: Write Frequency - failed to write L register (ret=%d)\n", ret);
		break;
	}

	dev_info(&st->spi->dev, "ADMX: Write Frequency - writing H=0x%08x to reg 0x%03x\n",
	(u32)(frequency >> 32), ADMX_REG_SET_FREQUENCY_PRIMARY_H);
	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H,
			frequency >> 32);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Write Frequency - failed to write H register (ret=%d)\n", ret);
		break;
	}
	st->frequency_uhz = frequency;
	dev_info(&st->spi->dev, "ADMX: Frequency set to %llu uHz\n",st->frequency_uhz);
	break;

	case IIO_CHAN_INFO_SCALE:
	dev_info(&st->spi->dev, "ADMX: Write Amplitude - input val=%d val2=%d\n", val, val2);
	amplitude = val * 1000000 + val2;
	dev_info(&st->spi->dev, "ADMX: Write Amplitude - converted to %u uVrms (max: %u)\n",
	amplitude, ADMX_AMPLITUDE_MAX_UV);

	if (amplitude > ADMX_AMPLITUDE_MAX_UV) {
		dev_err(&st->spi->dev, "ADMX: Write Amplitude - INVALID range!\n");
		ret = -EINVAL;
		break;
	}
	dev_info(&st->spi->dev, "ADMX: Write Amplitude - writing 0x%08x to reg 0x%03x\n",
	amplitude, ADMX_REG_SET_AMPLITUDE_PRIMARY);
	ret = regmap_write(st->regmap, ADMX_REG_SET_AMPLITUDE_PRIMARY,
			amplitude);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Write Amplitude - failed to write register (ret=%d)\n", ret);
		break;
	}
	st->amplitude_uvrms = amplitude;
	dev_info(&st->spi->dev, "ADMX: Amplitude set to %u uVrms\n", st->amplitude_uvrms);
	break;

	case IIO_CHAN_INFO_ENABLE:
	dev_info(&st->spi->dev, "ADMX: Write Enable - input val=%d (current state=%d)\n", val, st->output_enabled);
	if (val) {
		dev_info(&st->spi->dev, "ADMX: Write Enable - enabling output.\n");

	ret = regmap_update_bits(st->regmap, ADMX_REG_SET_SIGNAL_TYPE,
							GENMASK(3,0), st->signal_type);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Write Enable - failed to set SIGNAL_TYPE (ret=%d)\n", ret);
		break;
	}

	ret = admx_validate_params(st);
	if (ret)
		break;

	ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Failed to enable output (ret=%d)\n", ret);
		break;
	}
	{
		unsigned int status;
		ret = regmap_read_poll_timeout(st->regmap, ADMX_REG_STATUS, status,
				!!(status & ADMX_STATUS_SIGNAL_OUTPUT),
				1000 /*us*/, 100000 /*us*/);
		if (ret) {
			dev_warn(&st->spi->dev,
					"ADMX: start not asserted (STATUS=0x%08x) revalidate & retry\n", status);
			regmap_update_bits(st->regmap, ADMX_REG_CONTROL,
							ADMX_CONTROL_VALIDATE, ADMX_CONTROL_VALIDATE);

			ret = admx_set_task(st, ADMX_TASK_GENERATE_SIGNAL);
			if (ret) {
				dev_err(&st->spi->dev, "ADMX: retry start failed (ret=%d)\n", ret);
				break;
			}
		}
	}

	st->output_enabled = true;
	dev_info(&st->spi->dev, "ADMX: Output Enabled successfully\n");

	} else {
	dev_info(&st->spi->dev, "ADMX: Write Enable - disabling output.\n");
	ret = admx_stop_output(st);
	if (!ret) {
		st->output_enabled = false;
		dev_info(&st->spi->dev, "ADMX: Output Disabled successfully\n");
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

static ssize_t admx_read_frequency2(struct iio_dev *indio_dev,
									uintptr_t private,
									const struct iio_chan_spec *chan,
									char *buf)
{
	struct admx_state *st = iio_priv(indio_dev);
	u64 f = st->frequency2_uhz;
	u32 rem;
	int hz, micro;

	hz = div_u64_rem(f, 1000000, &rem);
	micro = rem / 1000;

	dev_info(&st->spi->dev, "ADMX: Read Frequency2 = %llu uHz -> %d.%06d Hz\n",
	st->frequency2_uhz, hz, micro);

	return sysfs_emit(buf, "%d.%06d\n", hz, micro);
}

static ssize_t admx_write_frequency2(struct iio_dev *indio_dev,
                                     uintptr_t private,
                                     const struct iio_chan_spec *chan,
                                     const char *buf, size_t len)
{
	struct admx_state *st = iio_priv(indio_dev);
	int hz, micro, ret;
	u64 frequency;

	ret = iio_str_to_fixpoint(buf, 6, &hz, &micro);
	if (ret)
		return ret;

	frequency = (u64)hz * 1000000ULL + (u64)micro * 1000ULL;

	dev_info(&st->spi->dev, "ADMX: Write Frequency - converted to %llu uHz (range: %u - %llu)\n",
			frequency, ADMX_FREQUENCY_MIN_UHZ, ADMX_FREQUENCY_MAX_UHZ);
	if (frequency < ADMX_FREQUENCY_MIN_UHZ || frequency > ADMX_FREQUENCY_MAX_UHZ) {
		dev_err(&st->spi->dev, "ADMX: Write Frequency - INVALID range!\n");
		return -EINVAL;
	}

	mutex_lock(&st->lock);

	dev_info(&st->spi->dev, "ADMX: Write Frequency - writing L=0x%08x to reg 0x%03x\n",
	(u32)(frequency & 0xFFFFFFFF), ADMX_FREQUENCY_SECONDARY_L);
	ret = regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_L,
			frequency & 0xFFFFFFFF);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Write Frequency - failed to write L register (ret=%d)\n", ret);
		goto out_unlock;
	}

	dev_info(&st->spi->dev, "ADMX: Write Frequency - writing H=0x%08x to reg 0x%03x\n",
	(u32)(frequency >> 32), ADMX_FREQUENCY_SECONDARY_H);
	ret = regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_H,
			frequency >> 32);
	if (ret) {
		dev_err(&st->spi->dev, "ADMX: Write Frequency - failed to write H register (ret=%d)\n", ret);
		goto out_unlock;
	}
	st->frequency2_uhz = frequency;
	dev_info(&st->spi->dev, "ADMX: Frequency2 set to %llu uHz\n",st->frequency2_uhz);

    ret = len;


out_unlock:
    mutex_unlock(&st->lock);
    return ret;
}

static ssize_t admx_read_scale2(struct iio_dev *indio_dev,
								uintptr_t private,
								const struct iio_chan_spec *chan,
								char *buf)
{
	struct admx_state *st = iio_priv(indio_dev);
	u32 uvrms;
	int v, micro;

	mutex_lock(&st->lock);

	uvrms = st->amplitude2_uvrms;
	v     = uvrms / 1000000U;
	micro = uvrms % 1000000U;
	dev_info(&st->spi->dev, "ADMX: Read Scale2 =%u uVrms -> %d.%06d Vrms\n", uvrms, v, micro);

	mutex_unlock(&st->lock);

	return sysfs_emit(buf, "%d.%06d\n", v, micro);
}

static ssize_t admx_write_scale2(struct iio_dev *indio_dev,
								uintptr_t private,
								const struct iio_chan_spec *chan,
								const char *buf, size_t len)
{
	struct admx_state *st = iio_priv(indio_dev);
	int v;
	int micro;
	u32 amplitude;
	int ret;

	ret = iio_str_to_fixpoint(buf, 6, &v, &micro);
	if (ret)
	return ret;

	amplitude = (u32)v * 1000000U + (u32)micro;

	dev_info(&st->spi->dev,
	"ADMX: Write Amplitude2 - converted to %u uVrms (max: %u)\n",
	amplitude, ADMX_AMPLITUDE_MAX_UV);

	if (amplitude > ADMX_AMPLITUDE_MAX_UV) {
	dev_err(&st->spi->dev,
	"ADMX: Write Amplitude2 - INVALID range!\n");
	return -EINVAL;
	}

	mutex_lock(&st->lock);

	dev_info(&st->spi->dev,
	"ADMX: Write Amplitude2 - writing 0x%08x to reg 0x%03x\n",
	amplitude, ADMX_AMPLITUDE_SECONDARY);

	ret = regmap_write(st->regmap, ADMX_AMPLITUDE_SECONDARY, amplitude);
	if (ret) {
	dev_err(&st->spi->dev,
	"ADMX: Write Amplitude2 - failed to write register (ret=%d)\n",
	ret);
	goto out_unlock;
	}

	st->amplitude2_uvrms = amplitude;

	dev_info(&st->spi->dev,
	"ADMX: Amplitude2 set to %u uVrms\n", st->amplitude2_uvrms);

	ret = len;

out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t admx_calibrate_store(struct device *dev,
									struct device_attribute *attr,
									const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admx_state *st = iio_priv(indio_dev);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
	 return ret;

	if (!val)
	 return len;

	mutex_lock(&st->lock);

	ret = admx_set_task(st, ADMX_TASK_CALIBRATE_SIGNAL);
	if (ret) {
	 mutex_unlock(&st->lock);
	 return ret;
	}
	ret = admx_wait_ready(st, 120000);

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(calibrate, 0200, NULL, admx_calibrate_store, 0);

static struct attribute *admx_attributes[] = {
	&iio_dev_attr_calibrate.dev_attr.attr,
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

static const struct iio_chan_spec_ext_info admx_ext_info[] = {
	IIO_ENUM("signal_type", IIO_SHARED_BY_ALL, &admx_signal_type_enum),
	IIO_ENUM_AVAILABLE("signal_type", IIO_SHARED_BY_ALL, &admx_signal_type_enum),
	{
		.name = "frequency2",
		.read = admx_read_frequency2,
		.write = admx_write_frequency2,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "scale2",
		.read = admx_read_scale2,
		.write = admx_write_scale2,
		.shared = IIO_SEPARATE,
	},
	{ },
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
		.ext_info = admx_ext_info,
	}
};

static int admx_init(struct admx_state *st)
{
	unsigned int val;
	int ret = 0;

	ret = admx_wait_ready(st, 10000);
	if (ret) {
		dev_err(&st->spi->dev, "Module not ready\n");
		return ret;
	}

	ret = regmap_read(st->regmap, ADMX_REG_MODULE_ID, &val);
	if (ret){
		dev_err(&st->spi->dev, "Failed to read Module ID\n");
		return ret;
	}
	dev_info(&st->spi->dev, "Module ID: 0x%08x\n", val);

	//Set default values
	st->amplitude_uvrms = 1000000; //1 Vrms
	st->frequency_uhz = 1000000000; //1 kHz
	st->amplitude2_uvrms = 2000000;
	st->frequency2_uhz   = 2000000000;
	st->signal_type = ADMX_SIGNAL_TYPE_SINE;
	st->output_enabled = false;
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

	regmap_write(st->regmap, ADMX_AMPLITUDE_SECONDARY,
				st->amplitude2_uvrms);

	regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_L,
				(u32)(st->frequency2_uhz & 0xFFFFFFFF));

	regmap_write(st->regmap, ADMX_FREQUENCY_SECONDARY_H,
				(u32)(st->frequency2_uhz >> 32));

	dev_info(&st->spi->dev,
			"ADMX: defaults secondary set (amp2=%u uVrms, freq2=%llu uHz)\n",
			st->amplitude2_uvrms, st->frequency2_uhz);
	if (ret)
	 	return ret;

	ret = regmap_write(st->regmap, ADMX_REG_SET_FREQUENCY_PRIMARY_H,
			st->frequency_uhz >> 32);
	if (ret)
		return ret;

	dev_info(&st->spi->dev, "ADMX: defaults set (amp=%u uVrms, freq=%llu uHz, type=%d)\n",
	st->amplitude_uvrms, st->frequency_uhz, st->signal_type);
	// Step 1
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
	// Step 2
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
	// Step 3
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
	// Step 4
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
	// Step 5
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
	// Step 6
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
	// Step 7
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
	// Step 8
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
	// Step 9
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
	// Step 10
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
	// Step 11
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
	// Step 12
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
	// Step 13
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
	// Step 14
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
	// Step 15
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
	// Step 16
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

 static int admx_hw_reset(struct admx_state *st)
 {
	int ret, val;

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

	if (st->gpio_reset) {
		gpiod_set_value_cansleep(st->gpio_reset, 1);
		msleep(10000);
		gpiod_set_value_cansleep(st->gpio_reset, 0);
		msleep(16000);
	} else {
		dev_warn(&st->spi->dev, "ADMX: no reset gpio; please add 'reset-gpios' in DT");
		return -ENODEV;
	}

	if (st->gpio_ready) {
		ret = readx_poll_timeout(gpiod_get_value_cansleep, st->gpio_ready, val,
								val == 1, 5000 /*us*/, 50000000 /*us*/);
		if (ret) {
			dev_err(&st->spi->dev, "ADMX: HW reset - READY GPIO timeout");
			return ret;
		}
	} else {
		struct regmap *map = st->regmap;
		ret = readx_poll_timeout(admx_read_status, map, val,
								!!(val & ADMX_STATUS_READY_STATUS),
								5000 /*us*/, 50000000 /*us*/);
		if (ret) {
			dev_err(&st->spi->dev, "ADMX: HW reset - STATUS READY timeout");
			return ret;
		}
	}
	st->output_enabled = 0;
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
