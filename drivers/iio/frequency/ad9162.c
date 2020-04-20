/*
 * AD9162 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2016-2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>

#include <linux/clk/clkscale.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "cf_axi_dds.h"

#include "ad916x/AD916x.h"

#define AD9162_REG_TEMP_SENS_LSB	0x132
#define AD9162_REG_TEMP_SENS_MSB	0x133
#define AD9162_REG_TEMP_UPDATE		0x134
#define AD9162_TEMP_UPDATE		0x01
#define AD9162_REG_TEMP_CTRL		0x135
#define AD9162_TEMP_ENABLE		0xA1

#define to_ad916x_state(__conv)	\
	container_of(__conv, struct ad9162_state, conv)

#define ad9162_temp_slope(tref, code) \
		DIV_ROUND_CLOSEST(((tref) + 190) * 1000, (code))

#define AD916X_TEST_WORD_MAX	0x7FFF

enum ad916x_variant {
	AD9162,
	AD9166,
};

enum {
	AD916x_NCO_FREQ,
	AD916x_SAMPLING_FREQUENCY,
	AD916x_TEMP_CALIB,
};

struct ad916x_chip_info {
	const struct iio_chan_spec *channels;
	const u32 num_channels;
};

struct ad9162_state {
	struct cf_axi_converter conv;
	struct ad916x_chip_info *ad916x_info;
	struct regmap *map;
	ad916x_handle_t dac_h;
	bool complex_mode;
	bool dc_test_mode;
	bool iq_swap;
	unsigned int interpolation;
	struct mutex lock;
};

struct ad9162_clk {
	const char *name;
	bool mandatory;
};

static const struct ad9162_clk ad9162_clks[] = {
	[CLK_DATA] = {
		.name = "jesd_dac_clk",
	},
	[CLK_DAC] = {
		.name = "dac_clk",
		.mandatory = true
	},
	[CLK_REF] = {
		.name = "dac_sysref",
	}
};

static int ad9162_read(struct spi_device *spi, unsigned int reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9162_state *st = to_ad916x_state(conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9162_write(struct spi_device *spi, unsigned int reg,
			unsigned int val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9162_state *st = to_ad916x_state(conv);

	return regmap_write(st->map, reg, val);
}

static int ad9162_get_temperature_code(struct ad9162_state *st, u16 *code)
{
	int ret;
	u8 _val_lsb = 0, _val_msb = 0;

	/* update the sensor with a new value */
	ret = ad916x_register_write(&st->dac_h, AD9162_REG_TEMP_UPDATE,
				    AD9162_TEMP_UPDATE);
	if (ret)
		return ret;

	ret = ad916x_register_read(&st->dac_h, AD9162_REG_TEMP_SENS_LSB,
				   &_val_lsb);
	if (ret)
		return ret;

	ret = ad916x_register_read(&st->dac_h, AD9162_REG_TEMP_SENS_MSB,
				   &_val_msb);
	if (ret)
		return ret;

	*code = (_val_msb << 8) | _val_lsb;

	return 0;
}

static unsigned long long ad9162_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9162_state *st = to_ad916x_state(conv);

	return div_u64(clk_get_rate_scaled(conv->clk[CLK_DAC],
					&conv->clkscale[CLK_DAC]),
					st->interpolation);
}

static int ad916x_set_data_clk(struct ad9162_state *st, const u64 rate)
{
	int ret;

	ret = clk_set_rate_scaled(st->conv.clk[CLK_DAC], rate,
				  &st->conv.clkscale[CLK_DAC]);
	if (ret)
		return ret;

	return ad916x_dac_set_clk_frequency(&st->dac_h, rate);
}

static int ad916x_setup_jesd(struct ad9162_state *st)
{
	struct device *dev = &st->conv.spi->dev;
	jesd_param_t appJesdConfig = {8, 1, 2, 4, 1, 32, 16, 16, 0, 0,
				0, 0, 0, 0};
	ad916x_jesd_link_stat_t link_status;
	uint8_t pll_lock_status = 0x0;
	u64 jesdLaneRate;
	unsigned long lane_rate_kHz;
	ad916x_handle_t *ad916x_h = &st->dac_h;
	int ret;

	/* check nco only mode */
	if (st->dc_test_mode) {
		dev_dbg(dev,
			"Device in nco only mode. No need to setup jesd\n");
		return 0;
	}

	st->complex_mode = true;
	st->interpolation = 2;
	st->iq_swap = true;

	appJesdConfig.jesd_L = 8;
	appJesdConfig.jesd_M = (st->complex_mode ? 2 : 1);
	appJesdConfig.jesd_F = 1;
	appJesdConfig.jesd_S = 2;
	appJesdConfig.jesd_HD = ((appJesdConfig.jesd_F == 1) ? 1 : 0);

	if (appJesdConfig.jesd_M == 2)
		st->conv.id = ID_AD9162_COMPLEX;

	ret = ad916x_jesd_config_datapath(ad916x_h, appJesdConfig,
				    st->interpolation, &jesdLaneRate);
	if (ret != 0)
		return ret;

	ret = ad916x_jesd_enable_datapath(ad916x_h, 0xFF, 0x1, 0x1);
	if (ret != 0)
		return ret;

	msleep(100);

	ret = ad916x_jesd_get_pll_status(ad916x_h, &pll_lock_status);
	if (ret != 0)
		return ret;

	dev_info(dev, "Serdes PLL %s (stat: %x)\n",
		 ((pll_lock_status & 0x39) == 0x9) ?
		 "Locked" : "Unlocked",  pll_lock_status);

	lane_rate_kHz = div_u64(ad916x_h->dac_freq_hz * 20 *
				appJesdConfig.jesd_M,
				appJesdConfig.jesd_L *
				st->interpolation * 1000);

	ret = clk_set_rate(st->conv.clk[CLK_DATA], lane_rate_kHz);
	if (ret < 0) {
		dev_err(dev, "Failed to set lane rate to %lu kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	ret = clk_prepare_enable(st->conv.clk[CLK_DATA]);
	if (ret) {
		dev_err(dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	/* Enable Link */
	ret = ad916x_jesd_enable_link(ad916x_h, 0x1);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Enable Link failed\n");
		return -EIO;
	}


	ret = ad916x_jesd_get_link_status(ad916x_h, &link_status);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: Get Link status failed \r\n");
		return -EIO;
	}

	dev_info(dev, "code_grp_sync: %x\n",
					link_status.code_grp_sync_stat);
	dev_info(dev, "frame_sync_stat: %x\n",
					link_status.frame_sync_stat);
	dev_info(dev, "good_checksum_stat: %x\n",
					link_status.good_checksum_stat);
	dev_info(dev, "init_lane_sync_stat: %x\n",
					link_status.init_lane_sync_stat);
	dev_info(dev, "%d lanes @ %llu GBps\n", appJesdConfig.jesd_L,
						jesdLaneRate);

	msleep(100);

	return 0;
}

static int ad9162_setup(struct ad9162_state *st)
{
	struct device *dev = &st->conv.spi->dev;
	uint8_t revision[3] = {0, 0, 0};
	ad916x_chip_id_t dac_chip_id;
	int ret = 0;
	u64 dac_rate_Hz;
	u32 fsc, sgl_pt[2];
	ad916x_handle_t *ad916x_h = &st->dac_h;

	/* Initialise DAC Module */
	ret = ad916x_init(ad916x_h);
	if (ret != 0)
		return ret;

	ret = ad916x_get_chip_id(ad916x_h, &dac_chip_id);
	if (ret != 0)
		return ret;

	ret = ad916x_get_revision(ad916x_h, &revision[0], &revision[1],
				  &revision[2]);
	if (ret != 0)
		return ret;

	dev_info(dev, "AD916x DAC Chip ID: %d\n", dac_chip_id.chip_type);
	dev_info(dev, "AD916x DAC Product ID: %x\n", dac_chip_id.prod_id);
	dev_info(dev, "AD916x DAC Product Grade: %d\n", dac_chip_id.prod_grade);
	dev_info(dev, "AD916x DAC Product Revision: %d\n",
						dac_chip_id.dev_revision);
	dev_info(dev, "AD916x Revision: %d.%d.%d\n", revision[0], revision[1],
						revision[2]);

	/*
	 * Check for the fsc property. If not present, leave the device
	 * with the default value...
	 */
	if (device_property_read_u32(dev, "adi,full-scale-current-microamp",
				     &fsc))
		fsc = 40000;

	ad916x_dac_set_full_scale_current(ad916x_h, fsc/1000);

	dac_rate_Hz = clk_get_rate_scaled(st->conv.clk[CLK_DAC],
					  &st->conv.clkscale[CLK_DAC]);

	ret = ad916x_dac_set_clk_frequency(ad916x_h, dac_rate_Hz);
	if (ret != 0)
		return ret;

	/* check for dc test mode */
	if (device_property_read_bool(dev, "adi,dc-test-en"))
		st->dc_test_mode = true;

	st->interpolation = 1;

	dev_dbg(dev, "DAC CLK rate: %llu\n", dac_rate_Hz);

	/* enable temperature sensor */
	ret = ad916x_register_write(ad916x_h, AD9162_REG_TEMP_CTRL,
				    AD9162_TEMP_ENABLE);
	if (ret) {
		dev_err(dev, "Failed to enable the temperature sensor\n");
		return ret;
	}
	/* check for sensor calibration point */
	ret = device_property_read_u32_array(dev,
					     "adi,temperature-single-point-calibration",
					     sgl_pt, ARRAY_SIZE(sgl_pt));
	if (!ret) {
		if (!sgl_pt[1]) {
			dev_err(dev,
				"adi,temperature-single-point-calibration: Raw code cannot be 0!\n");
			return -EINVAL;
		}
		st->conv.temp_calib = sgl_pt[0];
		st->conv.temp_calib_code = sgl_pt[1];
		st->conv.temp_slope = ad9162_temp_slope(sgl_pt[0], sgl_pt[1]);
	}

	return 0;
}

static int ad9162_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(ad9162_clks); i++) {
		clk = devm_clk_get(&conv->spi->dev, ad9162_clks[i].name);
		if (IS_ERR(clk) && PTR_ERR(clk) != -ENOENT)
			return PTR_ERR(clk);

		if (PTR_ERR(clk) == -ENOENT) {
			if (ad9162_clks[i].mandatory)
				return PTR_ERR(clk);

			conv->clk[i] = NULL;
			continue;
		}

		if (i > CLK_DATA) {
			ret = clk_prepare_enable(clk);
			if (ret < 0)
				return ret;
		}

		of_clk_get_scale(conv->spi->dev.of_node, ad9162_clks[i].name,
				 &conv->clkscale[i]);

		conv->clk[i] = clk;
	}

	return 0;
}

static int ad9162_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	unsigned int tmp;
	int ret;
	u16 amplitude_raw, code;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*val = ad9162_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = conv->temp_calib_code;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		if (!conv->temp_calib_code)
			return -ENOTSUPP;

		ret = ad9162_get_temperature_code(st, &code);
		if (ret < 0)
			return ret;

		/* value in milli degrees */
		*val = 1000 * conv->temp_calib + conv->temp_slope *
					(code - conv->temp_calib_code);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
			mutex_lock(&st->lock);
			ret = ad916x_dc_test_get_mode(&st->dac_h,
						      &amplitude_raw,
						      &tmp);
			mutex_unlock(&st->lock);

			if (ret)
				return ret;

			*val = amplitude_raw;
			return IIO_VAL_INT;
		case IIO_TEMP:
			mutex_lock(&st->lock);
			ret = ad9162_get_temperature_code(st, &code);
			mutex_unlock(&st->lock);

			if (ret)
				return ret;

			*val = code;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}
	return -EINVAL;
}

static int ad9162_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		conv->temp_calib_code = val;
		break;
	case IIO_CHAN_INFO_RAW:
		if (val > AD916X_TEST_WORD_MAX || val < 0)
			return -EINVAL;

		mutex_lock(&st->lock);
		ret = ad916x_dc_test_set_mode(&st->dac_h, val,
					      st->dc_test_mode);
		mutex_unlock(&st->lock);
		return ret;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9162_prepare(struct cf_axi_converter *conv)
{
	struct cf_axi_dds_state *st = iio_priv(conv->indio_dev);
	struct ad9162_state *ad9162 = to_ad916x_state(conv);

	/* FIXME This needs documenation */
	dds_write(st, 0x428, (ad9162->complex_mode ? 0x1 : 0x0) |
		  (ad9162->iq_swap ? 0x2 : 0x0));
	return 0;
}

static const struct regmap_config ad9162_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	/* TODO: Add volatile/writeable registers tables */
	.cache_type = REGCACHE_NONE,
};


static int delay_us(void *user_data, unsigned int time_us)
{
	usleep_range(time_us, time_us + (time_us >> 3));
	return 0;
}



static int spi_xfer_dummy(void *user_data, uint8_t *wbuf, uint8_t *rbuf,
			  int len)
{
	return 0;
}


static ssize_t ad9162_attr_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	ad916x_handle_t *ad916x_h = &st->dac_h;
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	switch ((u32)this_attr->address) {
	case 0:
		ret = ad916x_nco_set(ad916x_h, 0, readin, 0, 0);
		break;
	case 1:
		ret = ad916x_fir85_set_enable(ad916x_h, !!readin);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad9162_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	ad916x_handle_t *ad916x_h = &st->dac_h;
	int ret = 0;
	u64 freq;
	u16 ampl;

	mutex_lock(&st->lock);
	switch ((u32)this_attr->address) {
	case 0:
		ad916x_nco_get(ad916x_h, 0, &freq, &ampl, &ret);
		ret = sprintf(buf, "%llu\n", freq);
		break;
	case 1:
		ad916x_fir85_get_enable(ad916x_h, &ret);
		ret = sprintf(buf, "%d\n", !!ret);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}


static IIO_DEVICE_ATTR(out_altvoltage2_frequency_nco,
		       0644,
		       ad9162_attr_show,
		       ad9162_attr_store,
		       0);


static IIO_DEVICE_ATTR(out_voltage_fir85_enable,
		       0644,
		       ad9162_attr_show,
		       ad9162_attr_store,
		       1);

static struct attribute *ad9162_attributes[] = {
	&iio_dev_attr_out_altvoltage2_frequency_nco.dev_attr.attr,
	&iio_dev_attr_out_voltage_fir85_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9162_attribute_group = {
	.attrs = ad9162_attributes,
};

static ssize_t ad916x_write_ext(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	s64 freq_hz;
	int tref;
	u16 test_word, code;
	u64 samp_freq_hz;
	/* en just because we have to pass it to ad916x_dc_test_get_mod */
	int ret, en;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case AD916x_NCO_FREQ:
		ret = kstrtoll(buf, 10, &freq_hz);
		if (ret)
			break;

		/* just use the current test word */
		ret = ad916x_dc_test_get_mode(&st->dac_h, &test_word, &en);
		if (ret) {
			dev_warn(&conv->spi->dev, "Using max amplitude...\n");
			test_word = AD916X_TEST_WORD_MAX;
		}

		ret = ad916x_nco_set(&st->dac_h, 0, freq_hz, test_word,
				     st->dc_test_mode);
		break;
	case AD916x_SAMPLING_FREQUENCY:
		ret = kstrtoull(buf, 10, &samp_freq_hz);
		if (ret)
			break;

		ret = ad916x_set_data_clk(st, samp_freq_hz);
		break;
	case AD916x_TEMP_CALIB:
		/* value in milli degrees */
		ret = kstrtoint(buf, 10, &tref);
		if (ret)
			break;

		ret = ad9162_get_temperature_code(st, &code);
		if (ret < 0 || code == 0)
			break;

		conv->temp_calib = DIV_ROUND_CLOSEST(tref, 1000);
		conv->temp_slope = ad9162_temp_slope(conv->temp_calib, code);
		conv->temp_calib_code = code;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad916x_read_ext(struct iio_dev *indio_dev,
			       uintptr_t private,
			       const struct iio_chan_spec *chan,
			       char *buf)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = to_ad916x_state(conv);
	s64 freq;
	u16 test_word;
	int ret, dc_test_en;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case AD916x_NCO_FREQ:
		ret = ad916x_nco_get(&st->dac_h, 0, &freq, &test_word,
				     &dc_test_en);
		if (!ret)
			ret = sprintf(buf, "%lld\n", freq);
		break;
	case AD916x_SAMPLING_FREQUENCY:
		ret = sprintf(buf, "%llu\n", ad9162_get_data_clk(conv));
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ad9162_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	if (readval == NULL)
		ad9162_write(conv->spi, reg, writeval);
	else
		*readval = ad9162_read(conv->spi, reg);

	return 0;
}

#define _AD916x_CHAN_EXT_INFO(_name, _what, _shared) { \
	.name = _name, \
	.read = ad916x_read_ext, \
	.write = ad916x_write_ext, \
	.private = _what, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info ad916x_ext_info[] = {
	_AD916x_CHAN_EXT_INFO("nco_frequency", AD916x_NCO_FREQ, IIO_SEPARATE),
	_AD916x_CHAN_EXT_INFO("sampling_frequency", AD916x_SAMPLING_FREQUENCY,
			      IIO_SHARED_BY_ALL),
	{},
};

static const struct iio_chan_spec_ext_info ad916x_temp_ext_info[] = {
	_AD916x_CHAN_EXT_INFO("single_point_calib", AD916x_TEMP_CALIB,
			      IIO_SEPARATE),
	{},
};

#define AD916x_VOLTAGE_CHAN(index) { \
	.type = IIO_ALTVOLTAGE,	\
	.indexed = 1, \
	.channel = index, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.output = 1, \
	.ext_info = ad916x_ext_info, \
}

#define AD916x_TEMP_CHAN(index) { \
	.type = IIO_TEMP,	\
	.indexed = 1, \
	.channel = index, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_PROCESSED), \
	.ext_info = ad916x_temp_ext_info, \
}

static const struct iio_chan_spec ad916x_chann_spec[] = {
	AD916x_VOLTAGE_CHAN(0),
	AD916x_TEMP_CHAN(0),
};

/*
 * This info structure is only used when the device is used in standalone mode.
 * Note that ad9162 was not actually tested in this mode but it should be
 * fairly similar...
 * As for now, the only use case for standalone mode is to use this devices as
 * oscillators making use of the nco only mode.
 */
static struct ad916x_chip_info ad916x_info[] = {
	[AD9162] = {
		.num_channels = 1,
		.channels = ad916x_chann_spec,
	},
	[AD9166] = {
		.num_channels = 2,
		.channels = ad916x_chann_spec,
	}
};

static const struct iio_info ad916x_iio_info = {
	.read_raw = ad9162_read_raw,
	.write_raw = ad9162_write_raw,
	.debugfs_reg_access = ad9162_reg_access,
};

static void ad9162_clks_disable(void *data)
{
	struct cf_axi_converter *conv = data;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad9162_clks); i++)
		if (conv->clk[i])
			clk_disable_unprepare(conv->clk[i]);
}

static int ad916x_standalone_probe(struct ad9162_state *st)
{
	struct iio_dev *indio_dev = NULL;
	struct device *dev = &st->conv.spi->dev;
	struct device_node *np = dev->of_node;
	int ret;
	u8 _index = spi_get_device_id(st->conv.spi)->driver_data;

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	st->ad916x_info = &ad916x_info[_index];

	indio_dev->dev.parent = dev;
	indio_dev->name = np ? np->name :
		spi_get_device_id(st->conv.spi)->name;
	indio_dev->num_channels = st->ad916x_info->num_channels;
	indio_dev->channels = st->ad916x_info->channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad916x_iio_info;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register iio dev\n");
		return ret;
	}

	iio_device_set_drvdata(indio_dev, &st->conv);

	return 0;
}

static int ad9162_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
	struct ad9162_state *st;
	int ret;
	bool spi3wire = false;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	conv = &st->conv;

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	conv->txen_gpio[0] = devm_gpiod_get_optional(&spi->dev, "txen",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(conv->txen_gpio[0]))
		return PTR_ERR(conv->txen_gpio[0]);

	st->map = devm_regmap_init_spi(spi, &ad9162_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	conv->spi = spi;
	conv->id = ID_AD9162;

	ret = ad9162_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks, %d\n", ret);
		goto out;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad9162_clks_disable, conv);
	if (ret)
		return ret;

	if (device_property_read_bool(&spi->dev, "adi,spi-3wire-enable"))
		spi3wire = true;

	st->dac_h.user_data = st->map;
	st->dac_h.sdo = ((spi->mode & SPI_3WIRE) || spi3wire) ? SPI_SDIO :
			SPI_SDO;
	st->dac_h.dev_xfer = spi_xfer_dummy;
	st->dac_h.delay_us = delay_us;
	st->dac_h.event_handler = NULL;
	st->dac_h.tx_en_pin_ctrl = NULL;
	st->dac_h.reset_pin_ctrl = NULL;

	ret = ad9162_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	ret = ad916x_setup_jesd(st);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup JESD interface\n");
		return ret;
	}

	mutex_init(&st->lock);

	spi_set_drvdata(spi, conv);

	/* check for standalone probing... */
	if (device_property_read_bool(&spi->dev, "adi,standalone-probe"))
		return ad916x_standalone_probe(st);

	conv->write = ad9162_write;
	conv->read = ad9162_read;
	conv->setup = ad9162_prepare;

	conv->get_data_clk = ad9162_get_data_clk;
	conv->write_raw = ad9162_write_raw;
	conv->read_raw = ad9162_read_raw;
	conv->attrs = &ad9162_attribute_group;

	dev_info(&spi->dev, "Probed.\n");

	return 0;
out:
	return ret;
}


static const struct of_device_id ad916x_dt_id[] = {
	{ .compatible = "adi,ad9162" },
	{ .compatible = "adi,ad9166" },
	{},
};
MODULE_DEVICE_TABLE(of, ad916x_dt_id);

static const struct spi_device_id ad9162_id[] = {
	{ "ad9162", AD9162 },
	{ "ad9166", AD9166 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad9162_id);

static struct spi_driver ad9162_driver = {
	.driver = {
		   .name = "ad9162",
		   .of_match_table = ad916x_dt_id,
	},
	.probe = ad9162_probe,
	.id_table = ad9162_id,
};

module_spi_driver(ad9162_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9162 DAC");
MODULE_LICENSE("GPL v2");
