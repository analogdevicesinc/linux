// SPDX-License-Identifier: GPL-2.0+
/*
 * AD5758 Digital to analog converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>

/* AD5758 registers definition */
#define AD5758_NOP					0x00
#define AD5758_DAC_INPUT				0x01
#define AD5758_DAC_OUTPUT				0x02
#define AD5758_CLEAR_CODE				0x03
#define AD5758_USER_GAIN				0x04
#define AD5758_USER_OFFSET				0x05
#define AD5758_DAC_CONFIG				0x06
#define AD5758_SW_LDAC					0x07
#define AD5758_KEY					0x08
#define AD5758_GP_CONFIG1				0x09
#define AD5758_GP_CONFIG2				0x0A
#define AD5758_DCDC_CONFIG1				0x0B
#define AD5758_DCDC_CONFIG2				0x0C
#define AD5758_WDT_CONFIG				0x0F
#define AD5758_DIGITAL_DIAG_CONFIG			0x10
#define AD5758_ADC_CONFIG				0x11
#define AD5758_FAULT_PIN_CONFIG				0x12
#define AD5758_TWO_STAGE_READBACK_SELECT		0x13
#define AD5758_DIGITAL_DIAG_RESULTS			0x14
#define AD5758_ANALOG_DIAG_RESULTS			0x15
#define AD5758_STATUS					0x16
#define AD5758_CHIP_ID					0x17
#define AD5758_FREQ_MONITOR				0x18
#define AD5758_DEVICE_ID_0				0x19
#define AD5758_DEVICE_ID_1				0x1A
#define AD5758_DEVICE_ID_2				0x1B
#define AD5758_DEVICE_ID_3				0x1C

/* AD5758_DAC_CONFIG */
#define AD5758_DAC_CONFIG_RANGE_MSK			GENMASK(3, 0)
#define AD5758_DAC_CONFIG_RANGE_MODE(x)			(((x) & 0xF) << 0)
#define AD5758_DAC_CONFIG_OVRNG_EN_MSK			BIT(4)
#define AD5758_DAC_CONFIG_OVRNG_EN_MODE(x)		(((x) & 0x1) << 4)
#define AD5758_DAC_CONFIG_INT_EN_MSK			BIT(5)
#define AD5758_DAC_CONFIG_INT_EN_MODE(x)		(((x) & 0x1) << 5)
#define AD5758_DAC_CONFIG_OUT_EN_MSK			BIT(6)
#define AD5758_DAC_CONFIG_OUT_EN_MODE(x)		(((x) & 0x1) << 6)
#define AD5758_DAC_CONFIG_RSET_EXT_EN_MSK		BIT(7)
#define AD5758_DAC_CONFIG_RSET_EXT_EN_MODE(x)		(((x) & 0x1) << 7)
#define AD5758_DAC_CONFIG_SR_EN_MSK			BIT(8)
#define AD5758_DAC_CONFIG_SR_EN_MODE(x)			(((x) & 0x1) << 8)
#define AD5758_DAC_CONFIG_SR_CLOCK_MSK			GENMASK(12, 9)
#define AD5758_DAC_CONFIG_SR_CLOCK_MODE(x)		(((x) & 0xF) << 9)
#define AD5758_DAC_CONFIG_SR_STEP_MSK			GENMASK(15, 13)
#define AD5758_DAC_CONFIG_SR_STEP_MODE(x)		(((x) & 0x7) << 13)

/* AD5758_KEY */
#define AD5758_KEY_CODE_RESET_1				0x15FA
#define AD5758_KEY_CODE_RESET_2				0xAF51
#define AD5758_KEY_CODE_SINGLE_ADC_CONV			0x1ADC
#define AD5758_KEY_CODE_RESET_WDT			0x0D06
#define AD5758_KEY_CODE_CALIB_MEM_REFRESH		0xFCBA

/* AD5758_DCDC_CONFIG1 */
#define AD5758_DCDC_CONFIG1_DCDC_VPROG_MSK		GENMASK(4, 0)
#define AD5758_DCDC_CONFIG1_DCDC_VPROG_MODE(x)		(((x) & 0x1F) << 0)
#define AD5758_DCDC_CONFIG1_DCDC_MODE_MSK		GENMASK(6, 5)
#define AD5758_DCDC_CONFIG1_DCDC_MODE_MODE(x)		(((x) & 0x3) << 5)
#define AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MSK	BIT(7)
#define AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MODE(x)	(((x) & 0x1) << 7)

/* AD5758_DCDC_CONFIG2 */
#define AD5758_DCDC_CONFIG2_ILIMIT_MSK			GENMASK(3, 1)
#define AD5758_DCDC_CONFIG2_ILIMIT_MODE(x)		(((x) & 0x7) << 1)
#define AD5758_DCDC_CONFIG2_ADC_CONTROL_DIAG_MSK	GENMASK(5, 4)
#define AD5758_DCDC_CONFIG2_ADC_CONTROL_DIAG_MODE(x)	(((x) & 0x3) << 4)
#define AD5758_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MSK	BIT(6)
#define AD5758_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MODE(x)	(((x) & 0x1) << 6)
#define AD5758_DCDC_CONFIG2_SHORT_DEGLITCH_MSK		BIT(7)
#define AD5758_DCDC_CONFIG2_SHORT_DEGLITCH_MODE(x)	(((x) & 0x1) << 7)
#define AD5758_DCDC_CONFIG2_READ_COMP_DIS_MSK		BIT(10)
#define AD5758_DCDC_CONFIG2_READ_COMP_DIS_MODE(x)	(((x) & 0x1) << 10)
#define AD5758_DCDC_CONFIG2_INTR_SAT_3WI_MSK		BIT(11)
#define AD5758_DCDC_CONFIG2_BUSY_3WI_MSK		BIT(12)

/* AD5758_DIGITAL_DIAG_RESULTS */
#define AD5758_DIG_DIAG_RES_CAL_MEM_UNREFRESHED_MSK	BIT(15)

#define AD5758_REG_WRITE(x)	((0x80) | ((x) & 0x1F))

/* x^8 + x^2 + x^1 + x^0 */
#define AD5758_CRC8_POLY	0x07

/**
 * struct ad5758_state - driver instance specific data
 * @spi:	spi_device
 * @dc_dc_mode:	variable which stores the mode of operation
 * @out_range:	variable which stores the output range
 * @pwr_down:	variable which contains whether a channel is powered down or not
 * @data:	spi transfer buffers
 */

struct ad5758_state {
	struct spi_device *spi;
	struct mutex lock;
	struct gpio_desc *gpio_reset;
	unsigned int dc_dc_mode;
	unsigned int dc_dc_ilim;
	unsigned int sr_config[3];
	unsigned int out_range;
	unsigned int pwr_down;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	union {
		__be32 d32;
		u8 d8[4];
	} data[3] ____cacheline_aligned;
};

enum ad5758_output_range {
	AD5758_RANGE_0V_5V,
	AD5758_RANGE_0V_10V,
	AD5758_RANGE_PLUSMINUS_5V,
	AD5758_RANGE_PLUSMINUS_10V,
	AD5758_RANGE_0mA_20mA = 8,
	AD5758_RANGE_0mA_24mA,
	AD5758_RANGE_4mA_24mA,
	AD5758_RANGE_PLUSMINUS_20mA,
	AD5758_RANGE_PLUSMINUS_24mA,
	AD5758_RANGE_MINUS_1mA_PLUS_22mA,
};

enum ad5758_dc_dc_mode {
	AD5758_DCDC_MODE_POWER_OFF,
	AD5758_DCDC_MODE_DPC_CURRENT,
	AD5758_DCDC_MODE_DPC_VOLTAGE,
	AD5758_DCDC_MODE_PPC_CURRENT,
};

struct ad5758_range {
	int reg;
	int min;
	int max;
};

static const struct ad5758_range ad5758_min_max_table[] = {
	{ AD5758_RANGE_0V_5V, 0, 5000 },
	{ AD5758_RANGE_0V_10V, 0, 10000 },
	{ AD5758_RANGE_PLUSMINUS_5V, -5000, 5000 },
	{ AD5758_RANGE_PLUSMINUS_10V, -10000, 10000 },
	{ AD5758_RANGE_0mA_20mA, 0, 20},
	{ AD5758_RANGE_0mA_24mA, 0, 24 },
	{ AD5758_RANGE_4mA_24mA, 4, 24 },
	{ AD5758_RANGE_PLUSMINUS_20mA, -20, 20 },
	{ AD5758_RANGE_PLUSMINUS_24mA, -24, 24 },
	{ AD5758_RANGE_MINUS_1mA_PLUS_22mA, -1, 22 },
};

static const int ad5758_slew_rate_clk[16] = {
	240000, 200000, 150000, 128000, 64000, 32000, 16000, 8000, 4000, 2000,
	1000, 512, 256, 128, 64, 16
};

static const int ad5758_slew_rate_step[8] = {
	4, 12, 64, 120, 256, 500, 1820, 2048
};

static const int ad5758_dc_dc_ilimt[6] = {
	150, 200, 250, 300, 350, 400
};

static int ad5758_spi_reg_read(struct ad5758_state *st, unsigned int addr)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[0],
			.len = 4,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d8[0],
			.rx_buf = &st->data[2].d8[0],
			.len = 4,
		},
	};
	int ret;

	st->data[0].d32 = cpu_to_be32(
		(AD5758_REG_WRITE(AD5758_TWO_STAGE_READBACK_SELECT) << 24) |
		(addr << 8));
	st->data[1].d32 = cpu_to_be32(AD5758_REG_WRITE(AD5758_NOP) << 24);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	return (be32_to_cpu(st->data[2].d32) >> 8) & 0xFFFF;
}

static int ad5758_spi_reg_write(struct ad5758_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->data[0].d32 = cpu_to_be32((AD5758_REG_WRITE(addr) << 24) |
				      ((val & 0xFFFF) << 8));

	return spi_write(st->spi, &st->data[0].d8[0], 4);
}

static int ad5758_spi_write_mask(struct ad5758_state *st,
				 unsigned int addr,
				 unsigned long int mask,
				 unsigned int val)
{
	int regval;

	regval = ad5758_spi_reg_read(st, addr);
	if (regval < 0)
		return regval;

	regval &= ~mask;
	regval |= val;

	return ad5758_spi_reg_write(st, addr, regval);
}

static int ad5758_get_array_index(const int *array, unsigned int size, int val)
{
	int i;

	for (i = 0; i < size; i++) {
		if (val == array[i])
			return i;
	}

	return -EINVAL;
}

static int ad5758_wait_for_task_complete(struct ad5758_state *st,
					 unsigned int reg,
					 unsigned int mask)
{
	unsigned int timeout;
	int ret;

	timeout = 4;
	do {
		ret = ad5758_spi_reg_read(st, reg);
		if (ret < 0)
			return ret;

		if (!(ret & mask))
			return 0;

		mdelay(1);
	} while (--timeout);

	dev_err(&st->spi->dev,
		"Error reading bit 0x%x in 0x%x register\n", mask, reg);

	return -EIO;
}

static int ad5758_calib_mem_refresh(struct ad5758_state *st)
{
	int ret;

	ret = ad5758_spi_reg_write(st, AD5758_KEY,
				   AD5758_KEY_CODE_CALIB_MEM_REFRESH);

	if (ret < 0) {
		dev_err(&st->spi->dev,
			"Failed to initiate a calibration memory refresh\n");
		return ret;
	}

	/* Wait to allow time for the internal calibrations to complete */
	return ad5758_wait_for_task_complete(st, AD5758_DIGITAL_DIAG_RESULTS,
				AD5758_DIG_DIAG_RES_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5758_soft_reset(struct ad5758_state *st)
{
	int ret;

	ret = ad5758_spi_reg_write(st, AD5758_KEY, AD5758_KEY_CODE_RESET_1);
	if (ret < 0)
		return ret;

	ret = ad5758_spi_reg_write(st, AD5758_KEY, AD5758_KEY_CODE_RESET_2);

	mdelay(1);

	return ret;
}

static int ad5758_set_dc_dc_conv_mode(struct ad5758_state *st,
				      unsigned int mode)
{
	int ret;

	ret = ad5758_spi_write_mask(st, AD5758_DCDC_CONFIG1,
				    AD5758_DCDC_CONFIG1_DCDC_MODE_MSK,
				    AD5758_DCDC_CONFIG1_DCDC_MODE_MODE(mode));
	if (ret < 0)
		return ret;

	/*
	 * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
	 * This allows the 3-wire interface communication to complete.
	 */
	ret = ad5758_wait_for_task_complete(st, AD5758_DCDC_CONFIG2,
					    AD5758_DCDC_CONFIG2_BUSY_3WI_MSK);
	if (ret < 0)
		return ret;

	st->dc_dc_mode = mode;

	return ret;
}

static int ad5758_set_dc_dc_ilim(struct ad5758_state *st, unsigned int ilim)
{
	int ret;

	ret = ad5758_spi_write_mask(st, AD5758_DCDC_CONFIG2,
				    AD5758_DCDC_CONFIG2_ILIMIT_MSK,
				    AD5758_DCDC_CONFIG2_ILIMIT_MODE(ilim));
	if (ret < 0)
		return ret;
	/*
	 * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
	 * This allows the 3-wire interface communication to complete.
	 */
	return ad5758_wait_for_task_complete(st, AD5758_DCDC_CONFIG2,
					     AD5758_DCDC_CONFIG2_BUSY_3WI_MSK);
}

static int ad5758_slew_rate_config(struct ad5758_state *st,
				   unsigned int enable,
				   unsigned int sr_clk,
				   unsigned int sr_step)
{
	unsigned int mode;
	unsigned long int mask;
	int ret;

	mask = (AD5758_DAC_CONFIG_SR_EN_MSK |
	       AD5758_DAC_CONFIG_SR_CLOCK_MSK |
	       AD5758_DAC_CONFIG_SR_STEP_MSK);

	mode = (AD5758_DAC_CONFIG_SR_EN_MODE(enable) |
		AD5758_DAC_CONFIG_SR_CLOCK_MODE(sr_clk) |
		AD5758_DAC_CONFIG_SR_STEP_MODE(sr_step));

	ret = ad5758_spi_write_mask(st, AD5758_DAC_CONFIG, mask, mode);
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	return ad5758_wait_for_task_complete(st, AD5758_DIGITAL_DIAG_RESULTS,
				AD5758_DIG_DIAG_RES_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5758_set_out_range(struct ad5758_state *st, unsigned int range)
{
	int ret;

	ret = ad5758_spi_write_mask(st, AD5758_DAC_CONFIG,
				    AD5758_DAC_CONFIG_RANGE_MSK,
				    AD5758_DAC_CONFIG_RANGE_MODE(
					    ad5758_min_max_table[range].reg));
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	ret =  ad5758_wait_for_task_complete(st, AD5758_DIGITAL_DIAG_RESULTS,
				AD5758_DIG_DIAG_RES_CAL_MEM_UNREFRESHED_MSK);
	if (ret < 0)
		return ret;

	st->out_range = range;

	return ret;
}

static int ad5758_fault_prot_switch_en(struct ad5758_state *st,
				       unsigned char enable)
{
	int ret;

	ret = ad5758_spi_write_mask(st, AD5758_DCDC_CONFIG1,
			AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MSK,
			AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MODE(enable));
	if (ret < 0)
		return ret;
	/*
	 * Poll the BUSY_3WI bit in the DCDC_CONFIG2 register until it is 0.
	 * This allows the 3-wire interface communication to complete.
	 */
	return ad5758_wait_for_task_complete(st, AD5758_DCDC_CONFIG2,
					     AD5758_DCDC_CONFIG2_BUSY_3WI_MSK);
}

static int ad5758_internal_buffers_en(struct ad5758_state *st,
				      unsigned char enable)
{
	int ret;

	ret = ad5758_spi_write_mask(st, AD5758_DAC_CONFIG,
				    AD5758_DAC_CONFIG_INT_EN_MSK,
				    AD5758_DAC_CONFIG_INT_EN_MODE(enable));
	if (ret < 0)
		return ret;

	/* Wait to allow time for the internal calibrations to complete */
	return ad5758_wait_for_task_complete(st, AD5758_DIGITAL_DIAG_RESULTS,
				AD5758_DIG_DIAG_RES_CAL_MEM_UNREFRESHED_MSK);
}

static int ad5758_reset(struct ad5758_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 0);
		udelay(100);
		gpiod_set_value(st->gpio_reset, 1);
	} else {
		/* Perform a software reset */
		return ad5758_soft_reset(st);
	}

	mdelay(1);

	return 0;
}

static int ad5758_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad5758_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		ret =  ad5758_spi_reg_write(st, reg, writeval);
	} else {
		ret = ad5758_spi_reg_read(st, reg);
		if (ret < 0)
			return ret;

		*readval = ret;
		ret = 0;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5758_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad5758_state *st = iio_priv(indio_dev);
	int max, min, ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = ad5758_spi_reg_read(st, AD5758_DAC_INPUT);
		if (ret < 0) {
			mutex_unlock(&st->lock);
			return ret;
		}

		*val = ret;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		min = ad5758_min_max_table[st->out_range].min;
		max = ad5758_min_max_table[st->out_range].max;
		*val = max - min;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int ad5758_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad5758_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = ad5758_spi_reg_write(st, AD5758_DAC_INPUT, val);
		mutex_unlock(&st->lock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t ad5758_read_powerdown(struct iio_dev *indio_dev,
				     uintptr_t priv,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad5758_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->pwr_down);
}

static ssize_t ad5758_write_powerdown(struct iio_dev *indio_dev,
				      uintptr_t priv,
				      struct iio_chan_spec const *chan,
				      const char *buf, size_t len)
{
	struct ad5758_state *st = iio_priv(indio_dev);
	bool pwr_down;
	unsigned int dcdc_config1_mode, dc_dc_mode, dac_config_mode, val;
	unsigned long int dcdc_config1_msk, dac_config_msk;
	int ret;

	ret = strtobool(buf, &pwr_down);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	if (pwr_down) {
		dc_dc_mode = AD5758_DCDC_MODE_POWER_OFF;
		val = 0;
	} else {
		dc_dc_mode = st->dc_dc_mode;
		val = 1;
	}

	dcdc_config1_mode = (AD5758_DCDC_CONFIG1_DCDC_MODE_MODE(dc_dc_mode) |
			     AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MODE(val));
	dcdc_config1_msk = (AD5758_DCDC_CONFIG1_DCDC_MODE_MSK |
			    AD5758_DCDC_CONFIG1_FAULT_PROT_SW_EN_MSK);

	ret = ad5758_spi_write_mask(st, AD5758_DCDC_CONFIG1,
				    dcdc_config1_msk,
				    dcdc_config1_mode);
	if (ret < 0)
		goto err_unlock;

	dac_config_mode = (AD5758_DAC_CONFIG_OUT_EN_MODE(val) |
			   AD5758_DAC_CONFIG_INT_EN_MODE(val));
	dac_config_msk =  (AD5758_DAC_CONFIG_OUT_EN_MSK |
			   AD5758_DAC_CONFIG_INT_EN_MSK);

	ret = ad5758_spi_write_mask(st, AD5758_DAC_CONFIG,
				    dac_config_msk,
				    dac_config_mode);
	if (ret < 0)
		goto err_unlock;

	st->pwr_down = pwr_down;

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static const struct iio_info ad5758_info = {
	.read_raw = ad5758_read_raw,
	.write_raw = ad5758_write_raw,
	.debugfs_reg_access = &ad5758_reg_access,
};

static const struct iio_chan_spec_ext_info ad5758_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad5758_read_powerdown,
		.write = ad5758_write_powerdown,
		.shared = IIO_SEPARATE,
	},
	{ },
};

static const struct iio_chan_spec ad5758_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 32,
			.shift = 0,
			.endianness = IIO_BE,
		},
		.ext_info = ad5758_ext_info,
	},
};

static int ad5758_crc_disable(struct ad5758_state *st)
{
	st->data[0].d32 =
		cpu_to_be32(
		(AD5758_REG_WRITE(AD5758_DIGITAL_DIAG_CONFIG) << 24) | 0x5C3A);

	return spi_write(st->spi, &st->data[0].d8[0], 4);
}

static void ad5758_parse_dt(struct ad5758_state *st)
{
	unsigned int i, tmp, tmparray[3];
	int index;

	st->dc_dc_ilim = 150;
	if (!device_property_read_u32(&st->spi->dev,
				      "adi,dc-dc-ilim", &tmp)) {
		index = ad5758_get_array_index(ad5758_dc_dc_ilimt,
					ARRAY_SIZE(ad5758_dc_dc_ilimt), tmp);
		if (index < 0)
			dev_warn(&st->spi->dev,
				 "dc-dc-ilim out of range using default");
		else
			st->dc_dc_ilim = index;
	} else {
		dev_dbg(&st->spi->dev,
			 "Missing \"dc-dc-ilim\" property, using default\n");
	}

	st->dc_dc_mode = AD5758_DCDC_MODE_DPC_VOLTAGE;
	if (device_property_read_u32(&st->spi->dev, "adi,dc-dc-mode",
				     &st->dc_dc_mode))
		dev_dbg(&st->spi->dev,
			"Missing \"dc-dc-mode\" property, using DPC_VOLTAGE\n");

	/* 0 V to 5 V voltage range */
	st->out_range = 0;
	if (!device_property_read_u32(&st->spi->dev, "adi,range", &tmp)) {
		for (i = 0; i < ARRAY_SIZE(ad5758_min_max_table); i++) {
			if (tmp == ad5758_min_max_table[i].reg) {
				st->out_range = i;
				break;
			}
		}

		if (i == ARRAY_SIZE(ad5758_min_max_table))
			dev_warn(&st->spi->dev, "range invalid, using default");
	} else {
		dev_dbg(&st->spi->dev,
			"Missing \"range\" property, using default\n");
	}

	st->sr_config[0] = 1;
	st->sr_config[1] = 16000;
	st->sr_config[2] = 4;
	if (!device_property_read_u32_array(&st->spi->dev, "adi,slew",
					    tmparray, 3)) {
		st->sr_config[0] = tmparray[0];

		index = ad5758_get_array_index(ad5758_slew_rate_clk,
					       ARRAY_SIZE(ad5758_slew_rate_clk),
					       tmparray[1]);

		if (index < 0)
			dev_warn(&st->spi->dev,
				"slew rate clock out of range, using default");
		else
			st->sr_config[1] = index;

		index = ad5758_get_array_index(ad5758_slew_rate_step,
					ARRAY_SIZE(ad5758_slew_rate_step),
					tmparray[2]);

		if (index < 0)
			dev_warn(&st->spi->dev,
				"slew rate step out of range, using default");
		else
			st->sr_config[2] = index;
	} else {
		dev_dbg(&st->spi->dev,
			 "Missing \"slew\" property, using default\n");
	}
}

static int ad5758_init(struct ad5758_state *st)
{
	int regval, ret;

	ad5758_parse_dt(st);

	st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	/* Disable CRC checks */
	ret = ad5758_crc_disable(st);
	if (ret < 0)
		return ret;

	/* Perform a reset */
	ret = ad5758_reset(st);
	if (ret < 0)
		return ret;

	/* Disable CRC checks */
	ret = ad5758_crc_disable(st);
	if (ret < 0)
		return ret;

	/* Perform a calibration memory refresh */
	ret = ad5758_calib_mem_refresh(st);
	if (ret < 0)
		return ret;

	regval = ad5758_spi_reg_read(st, AD5758_DIGITAL_DIAG_RESULTS);
	if (regval < 0)
		return regval;

	/* Clear all the error flags */
	ret = ad5758_spi_reg_write(st, AD5758_DIGITAL_DIAG_RESULTS, regval);
	if (ret < 0)
		return ret;

	/* Set the dc-to-dc current limit */
	ret = ad5758_set_dc_dc_ilim(st, st->dc_dc_ilim);
	if (ret < 0)
		return ret;

	/* Configure the dc-to-dc controller mode */
	ret = ad5758_set_dc_dc_conv_mode(st, st->dc_dc_mode);
	if (ret < 0)
		return ret;

	/* Configure the output range */
	ret = ad5758_set_out_range(st, st->out_range);
	if (ret < 0)
		return ret;

	/* Enable Slew Rate Control, set the slew rate clock and step */
	ret = ad5758_slew_rate_config(st, st->sr_config[0],
				      st->sr_config[1], st->sr_config[2]);
	if (ret < 0)
		return ret;

	/* Enable the VIOUT fault protection switch (FPS is closed) */
	ret = ad5758_fault_prot_switch_en(st, 1);
	if (ret < 0)
		return ret;

	/* Power up the DAC and internal (INT) amplifiers */
	ret = ad5758_internal_buffers_en(st, 1);
	if (ret < 0)
		return ret;

	/* Enable VIOUT */
	ret = ad5758_spi_write_mask(st, AD5758_DAC_CONFIG,
				    AD5758_DAC_CONFIG_OUT_EN_MSK,
				    AD5758_DAC_CONFIG_OUT_EN_MODE(1));

	return ret;
}

static int ad5758_probe(struct spi_device *spi)
{
	struct ad5758_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5758_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5758_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5758_channels);

	mutex_init(&st->lock);

	ret = ad5758_init(st);
	if (ret < 0) {
		dev_err(&spi->dev, "AD5758 init failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device\n");
		return ret;
	}

	return 0;
}

static const struct spi_device_id ad5758_id[] = {
	{ "ad5758", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5758_id);

static struct spi_driver ad5758_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad5758_probe,
	.id_table = ad5758_id,
};

module_spi_driver(ad5758_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5758 DAC");
MODULE_LICENSE("GPL v2");
