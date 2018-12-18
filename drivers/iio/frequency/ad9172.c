/*
 * AD9172 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2018 Analog Devices Inc.
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

#include "ad917x/AD917x.h"

#define AD9172_SAMPLE_RATE_KHZ 3000000UL /* 3 GSPS */


enum chip_id {
	CHIPID_AD9171 = 0x71,
	CHIPID_AD9172 = 0x72,
	CHIPID_AD9173 = 0x73,
	CHIPID_AD9174 = 0x74,
	CHIPID_AD9175 = 0x75,
	CHIPID_AD9176 = 0x76,
};

struct ad9172_state {
	struct cf_axi_converter conv;
	enum chip_id id;
	struct regmap *map;
	ad917x_handle_t dac_h;
	bool complex_mode;
	bool iq_swap;
	unsigned interpolation;

};

static const char * const clk_names[] = {
	[CLK_DATA] = "jesd_dac_clk",
	[CLK_DAC] = "dac_clk",
	[CLK_REF] = "dac_sysref"
};

static int ad9172_read(struct spi_device *spi, unsigned reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9172_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);

	return regmap_write(st->map, reg, val);
}

static int ad9172_get_temperature_code(struct cf_axi_converter *conv)
{
	return 0;
}

static unsigned long long ad9172_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	u64 dac_rate_Hz;

	ad917x_get_dac_clk_freq(&st->dac_h, &dac_rate_Hz);

	return div_u64(dac_rate_Hz, st->interpolation);
}

static int ad9172_setup(struct ad9172_state *st)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	uint8_t revision[3] = {0,0,0};
	adi_chip_id_t dac_chip_id;
	jesd_param_t appJesdConfig;
	uint8_t pll_lock_status = 0, dll_lock_stat = 0;
	int ret;
	u64 dac_rate_Hz;
	unsigned long dac_clkin_Hz, lane_rate_kHz;
	ad917x_jesd_link_stat_t link_status;
	ad917x_handle_t *ad917x_h = &st->dac_h;
	unsigned long pll_mult;

	/*Initialise DAC Module*/
	ret = ad917x_init(ad917x_h);
	if (ret != 0) {
		dev_err(dev, "ad917x_init failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_reset(ad917x_h, 0);
	if (ret != 0) {
		dev_err(dev, "ad917x_reset failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_get_chip_id(ad917x_h, &dac_chip_id);
	if (ret != 0) {
		dev_err(dev, "ad917x_get_chip_id failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_get_revision(ad917x_h, &revision[0], &revision[1], &revision[2]);
	if (ret != 0)
		return ret;

	dev_info(dev, "AD916x DAC Chip ID: %d\n", dac_chip_id.chip_type);
	dev_info(dev, "AD916x DAC Product ID: %x\n", dac_chip_id.prod_id);
	dev_info(dev, "AD916x DAC Product Grade: %d\n", dac_chip_id.prod_grade);
	dev_info(dev, "AD916x DAC Product Revision: %d\n", dac_chip_id.dev_revision);
	dev_info(dev, "AD916x Revision: %d.%d.%d\n", revision[0], revision[1], revision[2]);

	dac_clkin_Hz = clk_get_rate(st->conv.clk[CLK_DAC]);


	dev_info(dev, "PLL Input rate %lu\n", dac_clkin_Hz);

	pll_mult = DIV_ROUND_CLOSEST(AD9172_SAMPLE_RATE_KHZ, dac_clkin_Hz / 1000);

	ret = ad917x_set_dac_clk(ad917x_h, dac_clkin_Hz * pll_mult, 1, dac_clkin_Hz);
	if (ret != 0) {
		dev_err(dev, "ad917x_set_dac_clk failed (%d)\n", ret);
		return ret;
	}

	msleep(100); /* Wait 100 ms for PLL to lock */

	ret = ad917x_get_dac_clk_status(ad917x_h,
				  &pll_lock_status, &dll_lock_stat);
	if (ret != 0) {
		dev_err(dev, "ad917x_get_dac_clk_status failed (%d)\n", ret);
		return ret;
	}

	dev_info(dev, "PLL lock status %x,  DLL lock status: %x\n",
		 pll_lock_status, dll_lock_stat);


	/* DEBUG: route DAC clock to output, so we can meassure it */
	ret = ad917x_set_clkout_config(ad917x_h, 1);
	if (ret != 0) {
		dev_err(dev, "ad917x_set_clkout_config failed (%d)\n", ret);
		return ret;
	}

	st->interpolation = 1; /* JESD MODE 10, 11 only support INT 1 */
	st->conv.id = ID_AD9172;

	ret = ad917x_jesd_config_datapath(ad917x_h, 0, 10, 1, st->interpolation);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_config_datapath failed (%d)\n", ret);
		return ret;
	}
	ret = ad917x_jesd_get_cfg_param(ad917x_h, &appJesdConfig);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_get_cfg_param failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_jesd_set_scrambler_enable(ad917x_h, 1);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_set_scrambler_enable failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_jesd_enable_datapath(ad917x_h, 0xFF, 0x1, 0x1);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_enable_datapath failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_jesd_set_syncoutb_enable(ad917x_h, SYNCOUTB_0, 1);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_set_syncoutb_enable failed (%d)\n", ret);
		return ret;
	}

	msleep(100);

	ret = ad917x_jesd_get_pll_status(ad917x_h, &pll_lock_status);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_get_pll_status failed (%d)\n", ret);
		return ret;
	}

	dev_info(dev, "Serdes PLL %s (stat: %x)\n",
		 ((pll_lock_status & 0x1) == 0x1) ?
		 "Locked" : "Unlocked",  pll_lock_status);

	ad917x_get_dac_clk_freq(ad917x_h, &dac_rate_Hz);

	lane_rate_kHz = div_u64(dac_rate_Hz * 20 * appJesdConfig.jesd_M,
				appJesdConfig.jesd_L * st->interpolation * 1000);

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

	ad917x_jesd_set_sysref_enable(ad917x_h, 0); /* subclass 0 */

	/*Enable Link*/
	ret = ad917x_jesd_enable_link(ad917x_h, JESD_LINK_ALL, 0x1);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Enable Link failed\n");
		return -EIO;
	}

	msleep(100);

	ret = ad917x_jesd_get_link_status(ad917x_h, JESD_LINK_0, &link_status);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Get Link status failed \r\n");
		return -EIO;
	}

	dev_info(dev, "code_grp_sync: %x \n", link_status.code_grp_sync_stat);
	dev_info(dev, "frame_sync_stat: %x \n", link_status.frame_sync_stat);
	dev_info(dev, "good_checksum_stat: %x \n", link_status.good_checksum_stat);
	dev_info(dev, "init_lane_sync_stat: %x \n", link_status.init_lane_sync_stat);
	dev_info(dev, "%d lanes @ %lu kBps\n", appJesdConfig.jesd_L, lane_rate_kHz);

	regmap_write(st->map, 0x008, 0xc0);
	regmap_write(st->map, 0x596, 0x1c);

	return 0;
}

static int ad9172_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(clk_names); i++) {
		clk = devm_clk_get(&conv->spi->dev, clk_names[i]);
		if (IS_ERR(clk) && PTR_ERR(clk) != -ENOENT)
			return PTR_ERR(clk);

		if (PTR_ERR(clk) == -ENOENT) {
			/* sysref might be optional */
			conv->clk[i] = NULL;
			continue;
		}

		if (i > CLK_DATA) {
			ret = clk_prepare_enable(clk);
			if (ret < 0)
				return ret;
		}

		of_clk_get_scale(conv->spi->dev.of_node, clk_names[i], &conv->clkscale[i]);
		conv->clk[i] = clk;
	}

	return 0;
}

static int ad9172_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned tmp;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*val = ad9172_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = conv->temp_calib_code;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		if (!conv->temp_calib_code)
			return -EINVAL;

		tmp = ad9172_get_temperature_code(conv);

		*val = ((tmp - conv->temp_calib_code) * 77
			+ conv->temp_calib * 10 + 10000) / 10;

		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9172_write_raw(struct iio_dev *indio_dev,
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
		conv->temp_calib_code = ad9172_get_temperature_code(conv);
		conv->temp_calib = val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9172_prepare(struct cf_axi_converter *conv)
{
//	struct cf_axi_dds_state *st = iio_priv(conv->indio_dev);
//	struct ad9172_state *ad9172 = container_of(conv, struct ad9172_state, conv);

// 	/* FIXME This needs documenation */
// 	dds_write(st, 0x428, (ad9172->complex_mode ? 0x1 : 0x0) |
// 		  (ad9172->iq_swap ? 0x2 : 0x0 ));
	return 0;
}

static const struct regmap_config ad9172_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	/* TODO: Add volatile/writeable registers tables */
	.cache_type = REGCACHE_NONE,
};

static int delay_us(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int ad9172_spi_xfer(void *user_data, uint8_t *wbuf, uint8_t *rbuf, int len)
{
	struct spi_device *spi = user_data;

	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len,
	};

	return spi_sync_transfer(spi, &t, 1);
}

static ssize_t ad9172_attr_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
//	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
//	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
//	ad917x_handle_t *ad917x_h = &st->dac_h;
	unsigned long long readin;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
		case 0:
			//ret = ad917x_nco_set(ad917x_h, 0, readin, 0, 0);
			break;
		default:
			ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9172_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
//	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
//	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
//	ad917x_handle_t *ad917x_h = &st->dac_h;
	int ret = 0;
	u64 freq = 0;
//	u16 ampl;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
		case 0:
			//ad917x_nco_get(ad917x_h, 0, &freq, &ampl, &ret);
			ret = sprintf(buf, "%llu\n", freq);
			break;
		default:
			ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(out_altvoltage4_frequency_nco, S_IRUGO | S_IWUSR,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       0);

static struct attribute *ad9172_attributes[] = {
	&iio_dev_attr_out_altvoltage4_frequency_nco.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9172_attribute_group = {
	.attrs = ad9172_attributes,
};


static int ad9172_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9172_state *st;
	int ret;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->id = (enum chip_id) dev_id->driver_data;
	conv = &st->conv;

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	conv->txen_gpio[0] = devm_gpiod_get_optional(&spi->dev, "txen0", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->txen_gpio[0]))
		return PTR_ERR(conv->txen_gpio[0]);

	conv->txen_gpio[1] = devm_gpiod_get_optional(&spi->dev, "txen1", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->txen_gpio[1]))
		return PTR_ERR(conv->txen_gpio[1]);

	st->map = devm_regmap_init_spi(spi, &ad9172_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	conv->write = ad9172_write;
	conv->read = ad9172_read;
	conv->setup = ad9172_prepare;
	conv->get_data_clk = ad9172_get_data_clk;
	conv->write_raw = ad9172_write_raw;
	conv->read_raw = ad9172_read_raw;
	conv->attrs = &ad9172_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9172;

	ret = ad9172_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	st->dac_h.user_data = spi;
	st->dac_h.sdo = SPI_SDO;
	st->dac_h.dev_xfer = ad9172_spi_xfer;
	st->dac_h.delay_us = delay_us;
	st->dac_h.tx_en_pin_ctrl = NULL;
	st->dac_h.reset_pin_ctrl = NULL;
	st->dac_h.syncoutb = SIGNAL_LVDS;
	st->dac_h.sysref = COUPLING_AC;

	ret = ad9172_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	spi_set_drvdata(spi, conv);

	dev_info(&spi->dev, "Probed.\n");
	return 0;
out:
	return ret;
}

static int ad9172_remove(struct spi_device *spi)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	int i;

	for (i = 0; i < ARRAY_SIZE(clk_names); i++)
		if (conv->clk[i])
			clk_disable_unprepare(conv->clk[i]);

	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct spi_device_id ad9172_id[] = {
	{ "ad9171", CHIPID_AD9171 },
	{ "ad9172", CHIPID_AD9172 },
	{ "ad9173", CHIPID_AD9173 },
	{ "ad9174", CHIPID_AD9174 },
	{ "ad9175", CHIPID_AD9175 },
	{ "ad9176", CHIPID_AD9176 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad9172_id);

static struct spi_driver ad9172_driver = {
	.driver = {
		   .name = "ad9172",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9172_probe,
	.remove = ad9172_remove,
	.id_table = ad9172_id,
};

module_spi_driver(ad9172_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9172 DAC");
MODULE_LICENSE("GPL v2");
