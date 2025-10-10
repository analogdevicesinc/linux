// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5413 Digital to analog converters driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * TODO: Currently CRC is not supported in this driver
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

/* AD5413 registers definition */
#define AD5413_REG_NOP 0x00
#define AD5413_REG_DAC_INPUT 0x01
#define AD5413_REG_DAC_OUTPUT 0x02
#define AD5413_REG_CLEAR_CODE 0x03
#define AD5413_REG_USER_GAIN 0x04
#define AD5413_REG_USER_OFFSET 0x05
#define AD5413_REG_DAC_CONFIG 0x06
#define AD5413_REG_DAC_CONFIG 0x06
#define AD5413_REG_DAC_CONFIG_SR_STEP_MSK GENMASK(15, 13)
#define AD5413_REG_DAC_CONFIG_SR_CLOCK_MSK GENMASK(12, 9)
#define AD5413_REG_DAC_CONFIG_SR_EN_MSK BIT(8)
#define AD5413_REG_DAC_CONFIG_RSET_EXT_EN_MSK BIT(7)
#define AD5413_REG_DAC_CONFIG_OUT_EN_MSK BIT(6)
#define AD5413_REG_DAC_CONFIG_INT_EN_MSK BIT(5)
#define AD5413_REG_DAC_CONFIG_OVRNG_EN_MSK BIT(4)
#define AD5413_REG_DAC_CONFIG_RANGE_MSK GENMASK(3, 0)
#define AD5413_REG_SW_LDAC 0x07
#define AD5413_REG_KEY 0x08
#define AD5413_REG_KEY_REG_ADDR_MASK GENMASK(20, 16)
#define AD5413_REG_KEY_CODE_MASK GENMASK(15, 0)
#define AD5413_REG_GP_CONFIG1 0x09
#define AD5413_REG_GP_CONFIG2 0x0A
#define AD5413_REG_DIGITAL_DIAG_CONFIG 0x10
#define AD5413_REG_FAULT_PIN_CONFIG 0x12
#define AD5413_REG_TWO_STAGE_READBACK_SELECT 0x13
#define AD5413_REG_DIGITAL_DIAG_RESULTS 0x14
#define AD5413_REG_DIGITAL_DIAG_CAL_MEM_UNREFRESHED_MSK BIT(15)
#define AD5413_REG_DIGITAL_DIAG_SLEW_BUSY_MSK BIT(14)
#define AD5413_REG_ANALOG_DIAG_RESULTS 0x15
#define AD5413_REG_STATUS 0x16
#define AD5413_REG_CHIP_ID 0x17
#define AD5413_REG_FREQ_MONITOR 0x18
#define AD5413_REG_DEVICE_ID_3 0x1C

/* Special Key Codes (bits[15:0]) */
#define AD5413_REG_KEY_CODE_RESET_1 0x15FA
#define AD5413_REG_KEY_CODE_RESET_2 0xAF51
#define AD5413_REG_KEY_CODE_CALIB_MEM_REFRESH 0xFCBA

/* DIGITAL_DIAG_RESULTS Register (Addr = 0x14) Busy Flags */

#define AD5413_WR_FLAG_MSK(x) (0x80 | ((x) & 0x1F))

#define AD5413_FULL_SCALE_MICRO (16383 * MICRO)

/* 0x5C3A: CRC disable code for DIGITAL_DIAG_CONFIG register (per datasheet) */
#define AD5413_DIGITAL_DIAG_CRC_DISABLE_VAL 0x5C3A
#define AD5413_CRC_DISABLE_CMD AD5413_DIGITAL_DIAG_CRC_DISABLE_VAL

struct ad5413_range {
	int reg;
	int min;
	int max;
};

struct ad5413_state {
	struct spi_device *spi;
	struct mutex lock; /* Protects all register access via SPI */
	struct gpio_desc *gpio_reset;
	const struct ad5413_range *out_range;
	unsigned int slew_time;
	bool pwr_down;
	__be32 d32[3] __aligned(IIO_DMA_MINALIGN);
};

/*
 * Output ranges corresponding to DAC_CONFIG[3:0] bits.
 * Defined by the following symbolic identifiers:
 *   AD5413_RANGE_PLUSMINUS_10_5V - ±10.5 V voltage range
 *   AD5413_RANGE_0MA_24MA        - 0 mA to 24 mA current range
 */
enum ad5413_output_range {
	AD5413_RANGE_PLUSMINUS_10_5V = 0x3,
	AD5413_RANGE_0MA_24MA = 0x9,
};

static const struct ad5413_range ad5413_voltage_range[] = { {
	.reg = AD5413_RANGE_PLUSMINUS_10_5V,
	.min = -10500000,
	.max = 10500000,
} };

static const struct ad5413_range ad5413_current_range[] = { {
	.reg = AD5413_RANGE_0MA_24MA,
	.min = 0,
	.max = 24000,
} };

static const int ad5413_sr_clk[16] = {
	240000, 200000, 150000, 128000, 64000, 32000, 16000, 8000,
	4000,	2000,	1000,	512,	256,   128,   64,    16,
};

static const int ad5413_sr_step[8] = {
	4, 12, 64, 120, 256, 500, 1820, 2048,
};

static int ad5413_spi_reg_read(struct ad5413_state *st, unsigned int addr)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d32[0],
			.len = 4,
			.cs_change = 1,
		},
		{
			.tx_buf = &st->d32[1],
			.rx_buf = &st->d32[2],
			.len = 4,
		},
	};
	int ret;

	st->d32[0] = cpu_to_be32(
		(AD5413_WR_FLAG_MSK(AD5413_REG_TWO_STAGE_READBACK_SELECT)
		 << 24) |
		(addr << 8));
	st->d32[1] = cpu_to_be32(AD5413_WR_FLAG_MSK(AD5413_REG_NOP) << 24);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	return (be32_to_cpu(st->d32[2]) >> 8) & 0xFFFF;
}

static int ad5413_spi_reg_write(struct ad5413_state *st, unsigned int addr,
				unsigned int val)
{
	st->d32[0] = cpu_to_be32((AD5413_WR_FLAG_MSK(addr) << 24) |
				 ((val & 0xFFFF) << 8));

	return spi_write(st->spi, &st->d32[0], sizeof(st->d32[0]));
}

static int ad5413_spi_write_mask(struct ad5413_state *st, unsigned int addr,
				 unsigned long mask, unsigned int val)
{
	int regval;

	regval = ad5413_spi_reg_read(st, addr);
	if (regval < 0)
		return regval;

	regval &= ~mask;
	regval |= val;

	return ad5413_spi_reg_write(st, addr, regval);
}

/**
 * ad5413_find_best_match - choose the smallest valid step >= target
 * @array: sorted array of valid values
 * @size:  number of elements in @array
 * @val:   target value to match
 *
 * Scans @array in ascending order and returns the index of the first
 * element ≥ @val; if none is found, returns size-1.
 */
static int ad5413_find_best_match(const int *array, unsigned int size, int val)
{
	int i;

	for (i = 0; i < size; i++) {
		if (val <= array[i])
			return i;
	}

	return size - 1;
}

static int ad5413_wait_for_task_complete(struct ad5413_state *st,
					 unsigned int reg, unsigned int mask)
{
	unsigned int timeout = 10;
	int ret;

	do {
		ret = ad5413_spi_reg_read(st, reg);
		if (ret < 0)
			return ret;

		if (!(ret & mask))
			return 0;

		fsleep(100);
	} while (--timeout);

	dev_err(&st->spi->dev, "Error reading bit 0x%x in 0x%x register\n",
		mask, reg);

	return -ETIMEDOUT;
}

static int ad5413_calib_mem_refresh(struct ad5413_state *st)
{
	int ret;

	ret = ad5413_spi_reg_write(st, AD5413_REG_KEY,
				   AD5413_REG_KEY_CODE_CALIB_MEM_REFRESH);
	if (ret < 0) {
		return dev_err_probe(
			&st->spi->dev, ret,
			"Failed to initiate a calibration memory refresh\n");
	}

	/* Wait to allow time for the internal calibrations to complete */
	return ad5413_wait_for_task_complete(
		st, AD5413_REG_DIGITAL_DIAG_RESULTS,
		AD5413_REG_DIGITAL_DIAG_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5413_slew_rate_set(struct ad5413_state *st,
				unsigned int sr_clk_idx,
				unsigned int sr_step_idx)
{
	unsigned int mode;
	unsigned long mask;
	int ret;

	mask = AD5413_REG_DAC_CONFIG_SR_EN_MSK |
	       AD5413_REG_DAC_CONFIG_SR_CLOCK_MSK |
	       AD5413_REG_DAC_CONFIG_SR_STEP_MSK;
	mode = FIELD_PREP(AD5413_REG_DAC_CONFIG_SR_EN_MSK, 1) |
	       FIELD_PREP(AD5413_REG_DAC_CONFIG_SR_STEP_MSK, sr_step_idx) |
	       FIELD_PREP(AD5413_REG_DAC_CONFIG_SR_CLOCK_MSK, sr_clk_idx);

	ret = ad5413_spi_write_mask(st, AD5413_REG_DAC_CONFIG, mask, mode);
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	return ad5413_wait_for_task_complete(
		st, AD5413_REG_DIGITAL_DIAG_RESULTS,
		AD5413_REG_DIGITAL_DIAG_SLEW_BUSY_MSK);
}

static int ad5413_slew_rate_config(struct ad5413_state *st)
{
	unsigned int sr_clk_idx, sr_step_idx;
	int i, res;
	s64 diff_new, diff_old;
	u64 sr_step, calc_slew_time;

	sr_clk_idx = 0;
	sr_step_idx = 0;
	diff_old = S64_MAX;
	/*
	 * The slew time can be determined by using the formula:
	 * Slew Time = (Full Scale Out / (Step Size x Update Clk Freq))
	 * where Slew time is expressed in microseconds
	 * Given the desired slew time, the following algorithm determines the
	 * best match for the step size and the update clock frequency.
	 */
	for (i = 0; i < ARRAY_SIZE(ad5413_sr_clk); i++) {
		/*
		 * Go through each valid update clock freq and determine a raw
		 * value for the step size by using the formula:
		 * Step Size = Full Scale Out / (Update Clk Freq * Slew Time)
		 */
		sr_step = AD5413_FULL_SCALE_MICRO;
		do_div(sr_step, ad5413_sr_clk[i]);
		do_div(sr_step, st->slew_time);
		/*
		 * After a raw value for step size was determined, find the
		 * closest valid match
		 */
		res = ad5413_find_best_match(
			ad5413_sr_step, ARRAY_SIZE(ad5413_sr_step), sr_step);
		/* Calculate the slew time */
		calc_slew_time = AD5413_FULL_SCALE_MICRO;
		do_div(calc_slew_time, ad5413_sr_step[res]);
		do_div(calc_slew_time, ad5413_sr_clk[i]);
		/*
		 * Determine with how many microseconds the calculated slew time
		 * is different from the desired slew time and store the diff
		 * for the next iteration
		 */
		diff_new = abs(st->slew_time - calc_slew_time);
		if (diff_new < diff_old) {
			diff_old = diff_new;
			sr_clk_idx = i;
			sr_step_idx = res;
		}
	}

	return ad5413_slew_rate_set(st, sr_clk_idx, sr_step_idx);
}

static int ad5413_set_out_range(struct ad5413_state *st, int range)
{
	int ret;

	ret = ad5413_spi_write_mask(
		st, AD5413_REG_DAC_CONFIG, AD5413_REG_DAC_CONFIG_RANGE_MSK,
		FIELD_PREP(AD5413_REG_DAC_CONFIG_RANGE_MSK, (range)));
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	return ad5413_wait_for_task_complete(
		st, AD5413_REG_DIGITAL_DIAG_RESULTS,
		AD5413_REG_DIGITAL_DIAG_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5413_internal_buffers_en(struct ad5413_state *st, bool enable)
{
	int ret;

	ret = ad5413_spi_write_mask(
		st, AD5413_REG_DAC_CONFIG, AD5413_REG_DAC_CONFIG_INT_EN_MSK,
		FIELD_PREP(AD5413_REG_DAC_CONFIG_INT_EN_MSK, (enable)));
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	return ad5413_wait_for_task_complete(
		st, AD5413_REG_DIGITAL_DIAG_RESULTS,
		AD5413_REG_DIGITAL_DIAG_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5413_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad5413_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);
	if (readval) {
		ret = ad5413_spi_reg_read(st, reg);
		if (ret < 0)
			return ret;

		*readval = ret;
		return 0;
	}

	return ad5413_spi_reg_write(st, reg, writeval);
}

static int ad5413_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad5413_state *st = iio_priv(indio_dev);
	int max, min, ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		guard(mutex)(&st->lock);
		ret = ad5413_spi_reg_read(st, AD5413_REG_DAC_INPUT);
		if (ret < 0)
			return ret;

		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		min = st->out_range->min;
		max = st->out_range->max;
		*val = (max - min) / 1000;
		*val2 = 14;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		min = st->out_range->min;
		max = st->out_range->max;
		*val = ((min * (1 << 14)) / (max - min)) / 1000;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad5413_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long info)
{
	struct ad5413_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		guard(mutex)(&st->lock);
		ret = ad5413_spi_reg_write(st, AD5413_REG_DAC_INPUT, val);
		return ret;
	default:
		return -EINVAL;
	}
}

static ssize_t ad5413_read_powerdown(struct iio_dev *indio_dev, uintptr_t priv,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad5413_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->pwr_down);
}

static ssize_t ad5413_write_powerdown(struct iio_dev *indio_dev, uintptr_t priv,
				      struct iio_chan_spec const *chan,
				      const char *buf, size_t len)
{
	struct ad5413_state *st = iio_priv(indio_dev);
	bool pwr_down;
	unsigned int dac_config_mode, val;
	unsigned long dac_config_msk;
	int ret;

	ret = kstrtobool(buf, &pwr_down);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	val = pwr_down ? 0 : 1;

	dac_config_mode = FIELD_PREP(AD5413_REG_DAC_CONFIG_OUT_EN_MSK, (val)) |
			  FIELD_PREP(AD5413_REG_DAC_CONFIG_INT_EN_MSK, (val));
	dac_config_msk = AD5413_REG_DAC_CONFIG_OUT_EN_MSK |
			 AD5413_REG_DAC_CONFIG_INT_EN_MSK;

	ret = ad5413_spi_write_mask(st, AD5413_REG_DAC_CONFIG, dac_config_msk,
				    dac_config_mode);
	if (ret < 0)
		return ret;

	st->pwr_down = pwr_down;

	return len;
}

static const struct iio_info ad5413_info = {
	.read_raw = ad5413_read_raw,
	.write_raw = ad5413_write_raw,
	.debugfs_reg_access = &ad5413_reg_access,
};

static const struct iio_chan_spec_ext_info ad5413_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad5413_read_powerdown,
		.write = ad5413_write_powerdown,
		.shared = IIO_SHARED_BY_TYPE,
	},
	{}
};

#define AD5413_DAC_CHAN(_chan_type)                                    \
	{                                                              \
		.type = (_chan_type),                                  \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | \
					    BIT(IIO_CHAN_INFO_OFFSET), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),          \
		.indexed = 1,                                          \
		.output = 1,                                           \
		.ext_info = ad5413_ext_info,                           \
	}

static const struct iio_chan_spec ad5413_voltage_ch[] = {
	AD5413_DAC_CHAN(IIO_VOLTAGE),
};

static const struct iio_chan_spec ad5413_current_ch[] = {
	AD5413_DAC_CHAN(IIO_CURRENT) };

static int ad5413_crc_disable(struct ad5413_state *st)
{
	return ad5413_spi_reg_write(st, AD5413_REG_DIGITAL_DIAG_CONFIG,
				    AD5413_CRC_DISABLE_CMD);
}

static int ad5413_find_out_range(struct ad5413_state *st,
				 const struct ad5413_range *range,
				 unsigned int size, int min, int max)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (min == range[i].min && max == range[i].max) {
			st->out_range = &range[i];

			return 0;
		}
	}

	return -EINVAL;
}

static int ad5413_parse_fw(struct ad5413_state *st)
{
	unsigned int slew_time_us, tmparray[2], size;
	const struct ad5413_range *ranges;
	int ret;

	ret = device_property_read_u32_array(&st->spi->dev,
		"adi,range-microvolt",
		tmparray, 2);
	if (ret == 0) {
		/* Validate voltage range: only ±10.5 V supported */
		if (tmparray[0] != -10500000 || tmparray[1] != 10500000)
			return dev_err_probe(
				&st->spi->dev, -EINVAL,
				"Invalid voltage range %u-%u µV; only ±10.5 V supported\n",
				tmparray[0], tmparray[1]);

		ranges = ad5413_voltage_range;
		size = ARRAY_SIZE(ad5413_voltage_range);
	} else {
		ret = device_property_read_u32_array(&st->spi->dev,
			"adi,range-microamp",
			tmparray, 2);
		if (ret)
			return dev_err_probe(
				&st->spi->dev, ret,
				"Missing \"adi,range-microvolt\" or \"adi,range-microamp\"\n");
		/* Validate current range: only 0–24000 µA supported */
		if (tmparray[0] != 0 || tmparray[1] != 24000)
			return dev_err_probe(
				&st->spi->dev, -EINVAL,
				"Invalid current range %u-%u µA; only 0–24000 supported\n",
				tmparray[0], tmparray[1]);
		ranges = ad5413_current_range;
		size = ARRAY_SIZE(ad5413_current_range);
	}

	/* 2) find_out_range() and st->out_range */
	ret = ad5413_find_out_range(st, ranges, size, tmparray[0], tmparray[1]);
	if (ret < 0)
		return dev_err_probe(&st->spi->dev, ret,
				     "Invalid range values (%u, %u)\n",
				     tmparray[0], tmparray[1]);

	/* 3) read slew-time (optional) */
	ret = device_property_read_u32(&st->spi->dev, "adi,slew-time-us",
				       &slew_time_us);
	if (ret == -ENODATA || ret == -EINVAL) {
		slew_time_us = 0;
	} else if (ret < 0) {
		return dev_err_probe(&st->spi->dev, ret,
				     "Failed to read adi,slew-time-us\n");
	}

	/* Store slew time for later use in DAC output ramping */
	st->slew_time = slew_time_us;

	return 0;
}

static int ad5413_init(struct ad5413_state *st)
{
	int regval, ret;

	/* reset-gpios is active low; idle state is high (released). */
	st->gpio_reset =
		devm_gpiod_get_optional(&st->spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	if (st->gpio_reset) {
		/* Hard reset via active-low reset GPIO */
		fsleep(100);
		gpiod_set_value(st->gpio_reset, 0);
		fsleep(100);
		gpiod_set_value(st->gpio_reset, 1);
	} else {
		/* No reset pin - fallback to soft reset */

		/*
		 * If chip was left with CRC enabled, the reset command
		 * may fail. Disable CRC first to ensure reset works.
		 */
		ret = ad5413_crc_disable(st);
		if (ret)
			return ret;

		/* Perform soft reset using key sequence */
		ret = ad5413_spi_reg_write(st, AD5413_REG_KEY,
					   AD5413_REG_KEY_CODE_RESET_1);
		if (ret)
			return ret;

		ret = ad5413_spi_reg_write(st, AD5413_REG_KEY,
					   AD5413_REG_KEY_CODE_RESET_2);
		if (ret)
			return ret;

		fsleep(100);
	}

	/*
	 * After any reset (HW or SW), CRC is enabled by default.
	 * Disable it here so that subsequent register accesses
	 * do not require CRC bytes.
	 */
	ret = ad5413_crc_disable(st);
	if (ret)
		return ret;

	/* Refresh calibration memory */
	ret = ad5413_calib_mem_refresh(st);
	if (ret < 0)
		return ret;

	/* Read and clear diagnostic flags */
	regval = ad5413_spi_reg_read(st, AD5413_REG_DIGITAL_DIAG_RESULTS);
	if (regval < 0)
		return regval;

	/* Clear all the error flags */
	ret = ad5413_spi_reg_write(st, AD5413_REG_DIGITAL_DIAG_RESULTS, regval);
	if (ret < 0)
		return ret;

	/* Configure the output range */
	ret = ad5413_set_out_range(st, st->out_range->reg);
	if (ret < 0)
		return ret;

	/* Enable Slew Rate Control, set the slew rate clock and step */
	if (st->slew_time) {
		ret = ad5413_slew_rate_config(st);
		if (ret < 0)
			return ret;
	}

	/* Power up the DAC and internal amplifiers */
	ret = ad5413_internal_buffers_en(st, 1);
	if (ret < 0)
		return ret;

	return ad5413_spi_write_mask(st, AD5413_REG_DAC_CONFIG,
		AD5413_REG_DAC_CONFIG_OUT_EN_MSK,
		FIELD_PREP(AD5413_REG_DAC_CONFIG_OUT_EN_MSK, 1));
}

static int ad5413_probe(struct spi_device *spi)
{
	struct ad5413_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5413_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 1;

	ret = ad5413_parse_fw(st);
	if (ret < 0)
		return ret;

	if (st->out_range->reg == AD5413_RANGE_PLUSMINUS_10_5V)
		indio_dev->channels = ad5413_voltage_ch;
	else
		indio_dev->channels = ad5413_current_ch;

	ret = ad5413_init(st);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "AD5413 init failed\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad5413_id[] = {
	{ "ad5413", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5413_id);

static const struct of_device_id ad5413_of_match[] = { 
	{ .compatible = "adi,ad5413" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5413_of_match);

static struct spi_driver ad5413_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ad5413_of_match,
	},
	.probe = ad5413_probe,
	.id_table = ad5413_id,
};
module_spi_driver(ad5413_driver);

MODULE_AUTHOR("Bruce Tsao <bruce.tsao@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5413 DAC");
MODULE_LICENSE("GPL v2");
