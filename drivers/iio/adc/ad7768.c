/*
 * Analog Devices AD7768-1 SPI ADC driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <asm/div64.h>
#include <linux/log2.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/ad_sigma_delta.h>

/*
 * AD7768 registers definition
 */
#define	AD7768_REG_CHIP_TYPE 		0x3
#define	AD7768_REG_PROD_ID_L		0x4
#define	AD7768_REG_PROD_ID_H		0x5
#define	AD7768_REG_CHIP_GRADE		0x6
#define	AD7768_REG_SCRATCH_PAD		0x0A
#define	AD7768_REG_VENDOR_L		0x0C
#define	AD7768_REG_VENDOR_H		0x0D
#define	AD7768_REG_INTERFACE_FORMAT	0x14
#define AD7768_REG_POWER_CLOCK		0x15
#define AD7768_REG_ANALOG		0x16
#define AD7768_REG_ANALOG2		0x17
#define AD7768_REG_CONVERSION		0x18
#define AD7768_REG_DIGITAL_FILTER	0x19
#define AD7768_REG_SINC3_DEC_RATE_MSB	0x1A
#define AD7768_REG_SINC3_DEC_RATE_LSB	0x1B
#define AD7768_REG_DUTY_CYCLE_RATIO	0x1C
#define AD7768_REG_SYNC_RESET		0x1D
#define AD7768_REG_GPIO_CONTROL		0x1E
#define AD7768_REG_GPIO_WRITE		0x1F
#define AD7768_REG_GPIO_READ		0x20
#define AD7768_REG_OFFSET_HI		0x21
#define AD7768_REG_OFFSET_MID		0x22
#define AD7768_REG_OFFSET_LO		0x23
#define AD7768_REG_GAIN_HI		0x24
#define AD7768_REG_GAIN_MID		0x25
#define AD7768_REG_GAIN_LO		0x26
#define AD7768_REG_SPI_DIAG_ENABLE	0x28
#define AD7768_REG_ADC_DIAG_ENABLE	0x29
#define AD7768_REG_DIG_DIAG_ENABLE	0x2A
#define	AD7768_REG_ADC_DATA		0x2C
#define	AD7768_REG_MASTER_STATUS	0x2D
#define	AD7768_REG_SPI_DIAG_STATUS	0x2E
#define	AD7768_REG_ADC_DIAG_STATUS	0x2F
#define	AD7768_REG_DIG_DIAG_STATUS	0x30
#define	AD7768_REG_MCLK_COUNTER		0x31

/*
 * AD7768_REG_CONVERSION
 */
#define AD7768_CONVERSION_MODE_MSK		(0x7 << 0)
#define AD7768_CONVERSION_MODE(x) 		(((x) & 0x7) << 0)

/*
 * AD7768_REG_POWER_CLOCK
 */
#define AD7768_POWER_CLK_MCLK_DIV_MSK 		GENMASK(5, 4)
#define AD7768_POWER_CLK_MCLK_DIV(x) 		(((x) & 0x3) << 4)
#define AD7768_POWER_CLK_PWRMODE_MSK 		GENMASK(1, 0)
#define AD7768_POWER_CLK_PWRMODE(x) 		(((x) & 0x3) << 0)

/*
 * AD7768_REG_DIGITAL_FILTER
 */
#define AD7768_DIGITAL_FILTER_FILTER_MSK 	GENMASK(6, 4)
#define AD7768_DIGITAL_FILTER_FILTER(x)  	(((x) & 0x7) << 4)
#define AD7768_DIGITAL_FILTER_DEC_RATE_MSK	GENMASK(2, 0)
#define AD7768_DIGITAL_FILTER_DEC_RATE(x)	(((x) & 0x7) << 0)

/*
 * AD7768_REG_SINC3_DEC_RATE_MSB
 */
#define AD7768_SINC3_DEC_RATE_MSB_MSK		GENMASK(4, 0)
#define AD7768_SINC3_DEC_RATE_MSB(x)		(((x) & 0xF) << 0)

/*
 * AD7768_REG_SINC3_DEC_RATE_LSB
 */
#define AD7768_SINC3_DEC_RATE_LSB_MSK		GENMASK(7, 0)
#define AD7768_SINC3_DEC_RATE_LSB(x)		(((x) & 0xFF) << 0)

/* ID Register Bit Designations (AD7768_REG_PROD_ID_L) */
#define AD7768_1_ID			 	0x01

/* The maximum decimation rate */
#define MAX_DEC_RATE				262144
/* The minimum decimation rate */
#define MIN_DEC_RATE				8
/* The minimum decimation rate possible for SINC3 filter */
#define SINC3_MIN_DEC_RATE			33

/*
 * Constant factors used for BW calculation for 3 different
 * types of filter
 */
#define SINC5_FILTER_FACTOR			204
#define SINC3_FILTER_FACTOR			260
#define FIR_FILTER_FACTOR			430

enum ad7768_filter {
	SINC5,
	SINC5_X8,
	SINC5_X16,
	SINC3,
	FIR
};

enum ad7768_pwrmode {
	ECO_POWER_MODE,
	MEDIAN_POWER_MODE = 2,
	FAST_POWER_MODE
};

static const int ad7768_mclk_divs[4] = {
	16, 8, 4, 2
};

static const int ad7768_fir_dec[6] = {
	32, 64, 128, 256, 512, 1024
};

static const int ad7768_fixed_clk_div_tbl[11][3] = {
	{16,    2,  8},
	{32, 	2,  16},
	{64,    2,  32},
	{128,   2,  64},
	{256,   2,  128},
	{512,   4,  128},
	{1024,  4,  256},
	{2048,  4,  512},
	{4096,  4,  1024},
	{8192,  8,  1024},
	{16384, 16, 1024},
};

static const struct iio_chan_spec ad7768_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 24,
			.storagebits = 32,
			.shift = 0,
			.endianness = IIO_BE,
		},
	},
};

struct ad7768_chip_info {
	unsigned int id;
};

struct ad7768_state {
	const struct ad7768_chip_info	*chip_info;
	struct regulator		*vref;
	struct ad_sigma_delta 		sd;
	unsigned long 			mclk_freq;
};

enum ad7768_supported_device_ids {
	ID_AD7768_1,
};

static struct ad7768_chip_info ad7768_chip_info_tbl[] = {
	[ID_AD7768_1] = {
		.id = AD7768_1_ID,
	},
};

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad_sd_write_reg(&st->sd, reg, 1, writeval);
	} else {
		ad_sd_read_reg(&st->sd, reg, 1, readval);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int ad7768_get_mclk_div(struct ad7768_state *st, unsigned int *mclk_div)
{
	unsigned int regval;
	int i;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, &regval);
	if (ret)
		return ret;

	i = (regval & AD7768_POWER_CLK_MCLK_DIV_MSK) >> 4;
	*mclk_div = ad7768_mclk_divs[i];

	return 0;
}

static int ad7768_set_mclk_div(struct ad7768_state *st,
			       unsigned int mclk_div)
{
	unsigned int readval;
	unsigned int writeval;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ad7768_mclk_divs); i++) {
		if (mclk_div == ad7768_mclk_divs[i])
			break;
	}

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, &readval);
	if (ret)
		return ret;

	writeval = ((readval & ~AD7768_POWER_CLK_MCLK_DIV_MSK) |
		    AD7768_POWER_CLK_MCLK_DIV(i));
	ret = ad_sd_write_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, writeval);

	return ret;
}

static int ad7768_set_filter_reg_val(struct ad7768_state *st,
				     enum ad7768_filter filter)
{
	unsigned int readval;
	unsigned int writeval;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_DIGITAL_FILTER, 1, &readval);
	if (ret)
		return ret;

	writeval = ((readval & ~AD7768_DIGITAL_FILTER_FILTER_MSK) |
		    AD7768_DIGITAL_FILTER_FILTER(filter));
	ret = ad_sd_write_reg(&st->sd, AD7768_REG_DIGITAL_FILTER, 1, writeval);

	return ret;
}

static int ad7768_get_dec_rate(struct ad7768_state *st, unsigned int *dec_rate)
{
	unsigned int regval;
	unsigned int filter;
	unsigned int sinc3_dec;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_DIGITAL_FILTER, 1, &regval);
	if (ret)
		return ret;

	filter = (regval & AD7768_DIGITAL_FILTER_FILTER_MSK) >> 4;

	switch (filter) {
	case SINC5:
	case FIR:
		*dec_rate = ad7768_fir_dec[regval &
					   AD7768_DIGITAL_FILTER_DEC_RATE_MSK];
		break;
	case SINC5_X8:
		*dec_rate = 8;
		break;
	case SINC5_X16:
		*dec_rate = 16;
		break;
	case SINC3:
		ret = ad_sd_read_reg(&st->sd,
				     AD7768_REG_SINC3_DEC_RATE_MSB, 1, &regval);
		if (ret)
			return ret;

		sinc3_dec = regval & AD7768_SINC3_DEC_RATE_MSB_MSK;
		sinc3_dec <<= 8;
		ret = ad_sd_read_reg(&st->sd,
				     AD7768_REG_SINC3_DEC_RATE_LSB, 1, &regval);
		if (ret)
			return ret;

		sinc3_dec |= (regval & AD7768_SINC3_DEC_RATE_LSB_MSK);
		/*
		 * Register value is incremented by 1 and multiplied by 32
		 * to give the actual decimation rate.
		 */
		*dec_rate = ((sinc3_dec + 1) << 5);
		break;
	}

	return 0;
}

static int ad7768_get_freq(struct ad7768_state *st, unsigned int *freq)
{
	unsigned int mclk_div;
	unsigned int dec_rate;
	int ret;

	ret = ad7768_get_mclk_div(st, &mclk_div);
	if (ret)
		return ret;

	ret = ad7768_get_dec_rate(st, &dec_rate);
	if (ret)
		return ret;

	*freq = st->mclk_freq / mclk_div / dec_rate;

	return 0;
}

static int ad7768_set_dec_rate(struct ad7768_state *st,
			       unsigned int dec_rate)
{
	unsigned int readval;
	unsigned int writeval;
	unsigned int filter;
	int i;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_DIGITAL_FILTER, 1, &readval);
	if (ret)
		return ret;

	filter = (readval & AD7768_DIGITAL_FILTER_FILTER_MSK) >> 4;

	/* Fixed decimation rate */
	if (filter == FIR) {
		for (i = 0; i < ARRAY_SIZE(ad7768_fir_dec); i++) {
			if (dec_rate == ad7768_fir_dec[i])
				break;
		}

		writeval = ((readval & ~AD7768_DIGITAL_FILTER_DEC_RATE_MSK) |
			    AD7768_DIGITAL_FILTER_DEC_RATE(i));
		ret = ad_sd_write_reg(&st->sd,
				      AD7768_REG_DIGITAL_FILTER, 1, writeval);
	}
	/* Programmable decimation rate */
	else if (filter == SINC3) {
		/*
		 * Value entered is divided by 32 and decremented by 1
		 * to determine the value to be written in the register
		 */
		dec_rate = (dec_rate >> 5) - 1;
		writeval = (dec_rate & GENMASK(12, 8)) >> 8;
		ret = ad_sd_write_reg(&st->sd,
				      AD7768_REG_SINC3_DEC_RATE_MSB, 1, writeval);
		if (ret)
			return ret;

		writeval = (dec_rate & GENMASK(7, 0));
		ret = ad_sd_write_reg(&st->sd,
				      AD7768_REG_SINC3_DEC_RATE_LSB, 1, writeval);
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static int ad7768_set_power_mode(struct ad7768_state *st,
				 unsigned int mclk_div)
{
	unsigned int readval;
	unsigned int writeval;
	unsigned int pwr_mode;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, &readval);
	if (ret)
		return ret;

	if (mclk_div == 2)
		pwr_mode = FAST_POWER_MODE;
	else if (mclk_div == 16)
		pwr_mode = ECO_POWER_MODE;
	else
		pwr_mode = MEDIAN_POWER_MODE;

	writeval = ((readval & ~AD7768_POWER_CLK_PWRMODE_MSK) |
		    AD7768_POWER_CLK_PWRMODE(pwr_mode));
	ret = ad_sd_write_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, writeval);

	return ret;
}

static int ad7768_fixed_clk_div_search(struct ad7768_state *st,
				       unsigned int freq,
				       int *index)
{
	int temp_freq;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad7768_fixed_clk_div_tbl); i++) {
		temp_freq = freq * ad7768_fixed_clk_div_tbl[i][0];
		if (st->mclk_freq == temp_freq) {
			*index = i;
			return 1;
		}
	}

	return 0;
}

static int ad7768_calc_freq_settings(struct ad7768_state *st,
				     unsigned int freq,
				     unsigned int *dec_rate,
				     unsigned int *mclk_div,
				     unsigned int *filter)
{
	unsigned int temp;
	unsigned int tmp_dec_rate;
	int i;
	int index;

	/* Frequency can be obtained by using a programmable decimation rate */
	if (ad7768_fixed_clk_div_search(st, freq, &index) < 1) {
		temp = DIV_ROUND_CLOSEST(st->mclk_freq, freq);
		/* Go through each MCLK_DIV and calculate the decimation rate */
		for (i = 0; i < ARRAY_SIZE(ad7768_mclk_divs); i++) {
			tmp_dec_rate = DIV_ROUND_CLOSEST(temp, ad7768_mclk_divs[i]);
			if ((tmp_dec_rate >= SINC3_MIN_DEC_RATE) &&
			    (tmp_dec_rate <= MAX_DEC_RATE)) {
				*mclk_div = ad7768_mclk_divs[i];
				*dec_rate = tmp_dec_rate;
				*filter = SINC3;
			}
		}
	}
	/*
	 * Frequency can be obtained by using one of the fixed decimation rates:
	 * 8, 16, 32, 64, 128, 256, 512, 1024.
	 * By default, FIR filter is set.
	 */
	else {
		*filter = FIR;
		*mclk_div = ad7768_fixed_clk_div_tbl[index][1];
		*dec_rate = ad7768_fixed_clk_div_tbl[index][2];
	}

	return 0;
}

static int ad7768_set_freq(struct ad7768_state *st, unsigned int freq)
{
	unsigned int mclk_div;
	unsigned int dec_rate;
	unsigned int filter;
	int ret;

	if ((freq < (st->mclk_freq / MAX_DEC_RATE / ad7768_mclk_divs[0])) ||
	    (freq > (st->mclk_freq / MIN_DEC_RATE / ad7768_mclk_divs[3]))) {
		return -EINVAL;
	} else {
		ret = ad7768_calc_freq_settings(st, freq, &dec_rate,
						&mclk_div, &filter);
		if (ret)
			return ret;

		if (dec_rate == 8) {
			ret = ad7768_set_filter_reg_val(st, SINC5_X8);
		} else if (dec_rate == 16) {
			ret = ad7768_set_filter_reg_val(st, SINC5_X16);
		} else {
			ret = ad7768_set_filter_reg_val(st, filter);
			if (ret)
				return ret;

			ret = ad7768_set_dec_rate(st, dec_rate);
			if (ret)
				return ret;

			ret = ad7768_set_mclk_div(st, mclk_div);
			if (ret)
				return ret;
		}
		ret = ad7768_set_power_mode(st, mclk_div);
	}

	return ret;
}

static int ad7768_get_filter_reg_val(struct ad7768_state *st,
				     unsigned int *filter_reg_val)
{
	unsigned int regval;
	int ret;

	ret = ad_sd_read_reg(&st->sd, AD7768_REG_DIGITAL_FILTER, 1, &regval);
	if (ret)
		return ret;

	*filter_reg_val =  (regval & AD7768_DIGITAL_FILTER_FILTER_MSK) >> 4;

	return 0;
}

static int ad7768_set_bw(struct ad7768_state *st, unsigned int bw)
{
	unsigned int freq;
	int index;
	int ret;

	ret = ad7768_get_freq(st, &freq);
	if (ret)
		return ret;

	ret = ad7768_fixed_clk_div_search(st, freq, &index);
	if (ret < 1)
		return ret;

	else if ((bw * 1000) <= (freq * SINC5_FILTER_FACTOR))
		ret = ad7768_set_filter_reg_val(st, SINC5);
	else if ((bw * 1000) <= (freq * SINC3_FILTER_FACTOR))
		ret = ad7768_set_filter_reg_val(st, SINC3);
	else
		ret = ad7768_set_filter_reg_val(st, FIR);

	return ret;
}

static int ad7768_get_bw(struct ad7768_state *st, unsigned int *bw)
{
	unsigned int freq;
	unsigned int val;
	int ret;

	ret = ad7768_get_filter_reg_val(st, &val);
	if (ret)
		return ret;

	ret = ad7768_get_freq(st, &freq);
	if (ret)
		return ret;

	switch (val) {
	case SINC5:
	case SINC5_X8:
	case SINC5_X16:
		*bw = freq * SINC5_FILTER_FACTOR;
		break;
	case SINC3:
		*bw = freq * SINC3_FILTER_FACTOR;
		break;
	case FIR:
		*bw = freq * FIR_FILTER_FACTOR;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct ad_sigma_delta_info ad7768_sigma_delta_info = {
	.has_registers = true,
	.data_reg = AD7768_REG_ADC_DATA,
	.addr_shift = 0,
	.read_mask = BIT(6),
};

static ssize_t ad7768_show_bw_available(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int freq;
	int index;
	int ret;

	ret = ad7768_get_freq(st, &freq);
	if (ret)
		return ret;

	ret = ad7768_fixed_clk_div_search(st, freq, &index);
	if (ret < 1) {
		freq *= SINC3_FILTER_FACTOR;
		ret = sprintf(buf, "%d.%06u\n", freq / 1000,
			      (freq % 1000) * 1000);
	}
	/* Decimation rates of x8 and x16 are available only for Sinc5 filter */
	else if ((ad7768_fixed_clk_div_tbl[index][2] == 8) ||
		 (ad7768_fixed_clk_div_tbl[index][2] == 16)) {
		freq *= SINC5_FILTER_FACTOR;
		ret = sprintf(buf, "%d.%06u\n",
			      freq / 1000,
			      (freq % 1000) * 1000);
	} else {
		ret = sprintf(buf, "%d.%06u, %d.%06u, %d.%06u\n",
			      (freq * SINC5_FILTER_FACTOR / 1000),
			      (freq * SINC5_FILTER_FACTOR % 1000) * 1000,
			      (freq * SINC3_FILTER_FACTOR / 1000),
			      (freq * SINC3_FILTER_FACTOR % 1000) * 1000,
			      (freq * FIR_FILTER_FACTOR / 1000),
			      (freq * FIR_FILTER_FACTOR % 1000) * 1000);
	}

	return ret;
}

static IIO_DEVICE_ATTR(in_voltage_filter_low_pass_3db_frequency_available,
		       S_IRUGO , ad7768_show_bw_available, NULL, 0);

static struct attribute *ad7768_attributes[] = {
	&iio_dev_attr_in_voltage_filter_low_pass_3db_frequency_available.
	dev_attr.attr,
	NULL
};

static const struct attribute_group ad7768_attribute_group = {
	.attrs = ad7768_attributes,
};

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int scale_uv;
	int bw;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad_sigma_delta_single_conversion(indio_dev, chan, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		scale_uv = regulator_get_voltage(st->vref);
		if (scale_uv < 0)
			return scale_uv;

		*val = scale_uv * 2 / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ad7768_get_freq(st, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = ad7768_get_bw(st, &bw);
		if (ret)
			return ret;

		*val = bw / 1000;
		*val2 = (bw % 1000) * 1000;

		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}

	switch (info) {

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ad7768_set_freq(st, val);
		break;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = ad7768_set_bw(st, val);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static const struct iio_info ad7768_info = {
	.read_raw = &ad7768_read_raw,
	.write_raw = &ad7768_write_raw,
	.attrs = &ad7768_attribute_group,
	.debugfs_reg_access = &ad7768_reg_access,
	.driver_module = THIS_MODULE,
};

static int ad7768_probe(struct spi_device *spi)
{
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	struct clk *mclk;
	unsigned int id;
	int ret;

	if (!spi->irq) {
		dev_err(&spi->dev, "No IRQ specified\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info =
		&ad7768_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	ad_sd_init(&st->sd, indio_dev, spi, &ad7768_sigma_delta_info);

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(mclk)) {
		ret = PTR_ERR(mclk);
		if (ret != -ENOENT) {
			dev_err(&spi->dev, "Failed getting mclk clock (%d)\n", ret);
			return ret;
		}
	} else {
		st->mclk_freq = clk_get_rate(mclk);
		clk_prepare_enable(mclk);
	}

	spi_set_drvdata(spi, indio_dev);

	indio_dev->channels = ad7768_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7768_channels);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad7768_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad_sd_setup_buffer_and_trigger(indio_dev);
	if (ret)
		goto error_disable_vref;

	/* read test for device presence */
	ret = ad_sd_read_reg(&st->sd, AD7768_REG_PROD_ID_L, 1, &id);
	if (ret)
		goto error_remove_trigger;

	id &= AD7768_1_ID;

	if (id != st->chip_info->id) {
		dev_err(&st->sd.spi->dev, "device ID query failed\n");
		goto error_remove_trigger;
	}

	/* perform a soft reset at startup */
	ret = ad_sd_write_reg(&st->sd, AD7768_REG_SYNC_RESET, 1, 0x83);
	if (ret)
		goto error_remove_trigger;

	ret = ad_sd_write_reg(&st->sd, AD7768_REG_SYNC_RESET, 1, 0x82);
	if (ret)
		goto error_remove_trigger;

	/* set power mode to fast */
	ret = ad_sd_write_reg(&st->sd, AD7768_REG_POWER_CLOCK, 1, 0x3);
	if (ret)
		goto error_remove_trigger;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;

	return 0;

error_remove_trigger:
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
error_disable_vref:
	regulator_disable(st->vref);

	return ret;
}

static int ad7768_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7768_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
	regulator_disable(st->vref);

	return 0;
}

static const struct spi_device_id ad7768_id[] = {
	{ "ad7768-1", ID_AD7768_1 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad7768_id);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad7768_probe,
	.remove = ad7768_remove,
	.id_table = ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768-1 ADC driver");
MODULE_LICENSE("GPL v2");
