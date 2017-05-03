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


enum chip_id {
	CHIPID_AD9162 = 0x62,
};

struct ad9162_platform_data {
	u8 xbar_lane0_sel;
	u8 xbar_lane1_sel;
	u8 xbar_lane2_sel;
	u8 xbar_lane3_sel;
	bool lanes2_3_swap_data;
};

struct ad9162_state {
	struct cf_axi_converter conv;
	enum chip_id id;
	struct regmap *map;
	ad916x_handle_t dac_h;
	bool complex_mode;
	bool iq_swap;
	unsigned interpolation;

};

static const char *clk_names[3] = { "jesd_dac_clk", "dac_clk", "dac_sysref" };

static int ad9162_read(struct spi_device *spi, unsigned reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9162_state *st = container_of(conv, struct ad9162_state, conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9162_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9162_state *st = container_of(conv, struct ad9162_state, conv);

	return regmap_write(st->map, reg, val);
}

static int ad9162_get_temperature_code(struct cf_axi_converter *conv)
{
	return 0;
}

static unsigned long long ad9162_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9162_state *st = container_of(conv, struct ad9162_state, conv);

	return div_u64(clk_get_rate_scaled(conv->clk[CLK_DAC], &conv->clkscale[CLK_DAC]), st->interpolation);
}

static int ad9162_setup(struct ad9162_state *st,
		struct ad9162_platform_data *pdata)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	uint8_t revision[3] = {0,0,0};
	ad916x_chip_id_t dac_chip_id;

	jesd_param_t appJesdConfig = { 8, 1, 2, 4, 1,32,16,16,0,0, 0,0,0,0};
	uint8_t pll_lock_status = 0x0;
// 	ad916x_event_t jesd_event_list[EVENT_NOF_EVENTS] = {
// 		EVENT_SYSREF_JITTER,
// 		EVENT_DATA_RDY,
// 		EVENT_JESD_LANE_FIFO_ERR,
// 		EVENT_JESD_PRBS_IMG_ERR,
// 		EVENT_JESD_PRBS_REAL_ERR,
// 		EVENT_JESD_NOT_IN_TBL_ERR,
// 		EVENT_JESD_K_ERR,
// 		EVENT_JESD_ILD_ERR,
// 		EVENT_JESD_ILS_ERR,
// 		EVENT_JESD_CKSUM_ERR,
// 		EVENT_JESD_FS_ERR,
// 		EVENT_JESD_CGS_ERR,
// 		EVENT_JESD_ILAS_ERR};

	int ret;
	u64 jesdLaneRate;
	ad916x_jesd_link_stat_t link_status;

	ad916x_handle_t *ad916x_h = &st->dac_h;

	/*Initialise DAC Module*/
	ret = ad916x_init(ad916x_h);
	if (ret != 0)
		return ret;

	ret = ad916x_get_chip_id(ad916x_h, &dac_chip_id);
	if (ret != 0)
		return ret;

	ret = ad916x_get_revision(ad916x_h, &revision[0], &revision[1], &revision[2]);
	if (ret != 0)
		return ret;

	dev_info(dev, "AD916x DAC Chip ID: %d\n", dac_chip_id.chip_type);
	dev_info(dev, "AD916x DAC Product ID: %x\n", dac_chip_id.prod_id);
	dev_info(dev, "AD916x DAC Product Grade: %d\n", dac_chip_id.prod_grade);
	dev_info(dev, "AD916x DAC Product Revision: %d\n", dac_chip_id.dev_revision);
	dev_info(dev, "AD916x Revision: %d.%d.%d\n", revision[0], revision[1], revision[2]);

	ad916x_dac_set_full_scale_current(ad916x_h, 20);

	ad916x_dac_set_clk_frequency(ad916x_h,
				     clk_get_rate_scaled(st->conv.clk[CLK_DAC],
				     &st->conv.clkscale[CLK_DAC]));
	if (ret != 0)
		return ret;

	st->complex_mode = true;
	st->interpolation = 2;
	st->iq_swap = true;

	appJesdConfig.jesd_L = 8;
	appJesdConfig.jesd_M = (st->complex_mode ? 2 : 1);
	appJesdConfig.jesd_F = 1;
	appJesdConfig.jesd_S = 2;
	appJesdConfig.jesd_HD = ((appJesdConfig.jesd_F == 1) ? 1 : 0);

	ad916x_jesd_config_datapath(ad916x_h, appJesdConfig,
				    st->interpolation, &jesdLaneRate);
	if (ret != 0)
		return ret;

	ad916x_jesd_enable_datapath(ad916x_h, 0xFF, 0x1, 0x1);
	if (ret != 0)
		return ret;

	msleep(100);

	ret = ad916x_jesd_get_pll_status(ad916x_h, &pll_lock_status);
	dev_info(dev, "Serdes PLL %s (stat: %x)\n", ((pll_lock_status & 0x39) == 0x9) ? "Locked" : "Unlocked",  pll_lock_status);

	if (ret != 0)
		return ret;

	/*Enable Link*/
	ret = ad916x_jesd_enable_link(ad916x_h, 0x1);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Enable Link failed\n");
		return -EIO;
	}


	ret = ad916x_jesd_get_link_status(ad916x_h, &link_status);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Get Link status failed \r\n");
		return -EIO;
	}

	dev_info(dev, "code_grp_sync: %x \n", link_status.code_grp_sync_stat);
	dev_info(dev, "frame_sync_stat: %x \n", link_status.frame_sync_stat);
	dev_info(dev, "good_checksum_stat: %x \n", link_status.good_checksum_stat);
	dev_info(dev, "init_lane_sync_stat: %x \n", link_status.init_lane_sync_stat);
	dev_info(dev, "%d lanes @ %llu GBps\n", appJesdConfig.jesd_L, jesdLaneRate);

	msleep(100);

	return 0;
}

static int ad9162_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < 3; i++) {
		clk = clk_get(&conv->spi->dev, &clk_names[i][0]);
		if (IS_ERR(clk)) {
			return -EPROBE_DEFER;
		}

		if (i > 0) {
			ret = clk_prepare_enable(clk);
			if (ret < 0)
				return ret;
		}

		of_clk_get_scale(conv->spi->dev.of_node, &clk_names[i][0], &conv->clkscale[i]);

		conv->clk[i] = clk;
	}
	return 0;
}

static int ad9162_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned tmp;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*val = ad9162_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = conv->temp_calib_code;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		if (!conv->temp_calib_code)
			return -EINVAL;

		tmp = ad9162_get_temperature_code(conv);

		*val = ((tmp - conv->temp_calib_code) * 77
			+ conv->temp_calib * 10 + 10000) / 10;

		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9162_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		conv->temp_calib_code = val;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		/*
		 * Writing in_temp0_input with the device temperature in milli
		 * degrees Celsius triggers the calibration.
		 */
		conv->temp_calib_code = ad9162_get_temperature_code(conv);
		conv->temp_calib = val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9162_prepare(struct cf_axi_converter *conv)
{
	struct cf_axi_dds_state *st = iio_priv(conv->indio_dev);
	struct ad9162_state *ad9162 = container_of(conv, struct ad9162_state, conv);

	/* FIXME This needs documenation */
	dds_write(st, 0x428, (ad9162->complex_mode ? 0x1 : 0x0) |
		  (ad9162->iq_swap ? 0x2 : 0x0 ));
	return 0;
}

#ifdef CONFIG_OF
static struct ad9162_platform_data *ad9162_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9162_platform_data *pdata;
	unsigned int tmp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	tmp = 0;
	of_property_read_u32(np, "adi,jesd-xbar-lane0-sel", &tmp);
	pdata->xbar_lane0_sel = tmp;

	tmp = 1;
	of_property_read_u32(np, "adi,jesd-xbar-lane1-sel", &tmp);
	pdata->xbar_lane1_sel = tmp;

	tmp = 2;
	of_property_read_u32(np, "adi,jesd-xbar-lane2-sel", &tmp);
	pdata->xbar_lane2_sel = tmp;

	tmp = 3;
	of_property_read_u32(np, "adi,jesd-xbar-lane3-sel", &tmp);
	pdata->xbar_lane3_sel = tmp;

	pdata->lanes2_3_swap_data = of_property_read_bool(np,
			"adi,lanes2-3-swap-data");

	return pdata;
}
#else
static
struct ad9162_platform_data *ad9162_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

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



static int spi_xfer_dummy(void *user_data, uint8_t *wbuf, uint8_t *rbuf, int len)
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
	struct ad9162_state *st = container_of(conv, struct ad9162_state, conv);
	ad916x_handle_t *ad916x_h = &st->dac_h;
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

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

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9162_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9162_state *st = container_of(conv, struct ad9162_state, conv);
	ad916x_handle_t *ad916x_h = &st->dac_h;
	int ret = 0;
	u64 freq;
	u16 ampl;

	mutex_lock(&indio_dev->mlock);
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
	mutex_unlock(&indio_dev->mlock);

	return ret;
}


static IIO_DEVICE_ATTR(out_altvoltage2_frequency_nco, S_IRUGO | S_IWUSR,
		       ad9162_attr_show,
		       ad9162_attr_store,
		       0);


static IIO_DEVICE_ATTR(out_voltage_fir85_enable,
		       S_IRUGO | S_IWUSR,
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


static int ad9162_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9162_platform_data *pdata;
	struct ad9162_state *st;
	int ret;

	if (spi->dev.of_node)
		pdata = ad9162_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->id = (enum chip_id) dev_id->driver_data;
	conv = &st->conv;

	conv->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);

	conv->txen_gpio = devm_gpiod_get(&spi->dev, "txen", GPIOD_OUT_HIGH);

	st->map = devm_regmap_init_spi(spi, &ad9162_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	conv->write = ad9162_write;
	conv->read = ad9162_read;
	conv->setup = ad9162_prepare;

	conv->get_data_clk = ad9162_get_data_clk;
	conv->write_raw = ad9162_write_raw;
	conv->read_raw = ad9162_read_raw;
	conv->attrs = &ad9162_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9162;

	ret = ad9162_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	st->dac_h.user_data = st->map;
	st->dac_h.sdo = SPI_SDIO;
	st->dac_h.dev_xfer = spi_xfer_dummy;
	st->dac_h.delay_us = delay_us;
	st->dac_h.event_handler = NULL;
	st->dac_h.tx_en_pin_ctrl = NULL;
	st->dac_h.reset_pin_ctrl = NULL;

	ret = ad9162_setup(st, pdata);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	clk_prepare_enable(conv->clk[0]);
	spi_set_drvdata(spi, conv);

	dev_info(&spi->dev, "Probed.\n");
	return 0;
out:
	return ret;
}

static int ad9162_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct spi_device_id ad9162_id[] = {
	{ "ad9162", CHIPID_AD9162 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad9162_id);

static struct spi_driver ad9162_driver = {
	.driver = {
		   .name = "ad9162",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9162_probe,
	.remove = ad9162_remove,
	.id_table = ad9162_id,
};

module_spi_driver(ad9162_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9162 DAC");
MODULE_LICENSE("GPL v2");
