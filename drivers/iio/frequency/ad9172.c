// SPDX-License-Identifier: GPL-2.0
/*
 * AD9172 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2018-2022 Analog Devices Inc.
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

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#include "cf_axi_dds.h"
#include "ad917x/AD917x.h"
#include "ad917x/ad917x_reg.h"

#define AD9172_SAMPLE_RATE_KHZ 3000000UL /* 3 GSPS */

#define AD9172_ATTR_CHAN_NCO(x) (2 << 8 | (x))
#define AD9172_ATTR_CHAN_PHASE(x) (3 << 8 | (x))
#define AD9172_ATTR_CHAN_TONE_AMP(x) (4 << 8 | (x))
#define AD9172_ATTR_CHAN_NCO_EN(x) (5 << 8 | (x))

#define AD9172_ATTR_MAIN_NCO(x) (6 << 8 | (x))
#define AD9172_ATTR_MAIN_PHASE(x) (7 << 8 | (x))
#define AD9172_ATTR_MAIN_NCO_EN(x) (8 << 8 | (x))

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
	struct jesd204_dev *jdev;
	struct jesd204_link jesd204_link;
	ad917x_handle_t dac_h;
	jesd_param_t appJesdConfig;
	u32 dac_rate_khz;
	u32 dac_interpolation;
	u32 channel_interpolation;
	u32 interpolation;
	u32 jesd_link_mode;
	u32 jesd_dual_link_mode;
	u32 jesd_subclass;
	u32 clock_output_config;
	u32 scrambling;
	u32 sysref_mode;
	bool pll_bypass;
	signal_type_t syncoutb_type;
	signal_coupling_t sysref_coupling;
	u8 nco_main_enable;
	u8 nco_channel_enable;
	u8 logic_lanes[8];
};

struct ad9172_jesd204_priv {
	struct ad9172_state *st;
};

static const char * const clk_names[] = {
	[CLK_DATA] = "jesd_dac_clk",	/* Not required with jesd204-fsm */
	[CLK_DAC] = "dac_clk",		/* Always needed */
	[CLK_REF] = "dac_sysref"	/* Not required with jesd204-fsm */
};

static int ad9172_read(struct spi_device *spi, u32 reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	u32 val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9172_write(struct spi_device *spi, u32 reg, u32 val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);

	return regmap_write(st->map, reg, val);
}

static unsigned long long ad9172_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	u64 dac_rate_Hz;

	ad917x_get_dac_clk_freq(&st->dac_h, &dac_rate_Hz);

	return div_u64(dac_rate_Hz, st->interpolation);
}

static int ad9172_link_status_get(struct ad9172_state *st, unsigned long lane_rate_kHz)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	ad917x_jesd_link_stat_t link_status;
	int ret, i;

	for (i = JESD_LINK_0; i <= JESD_LINK_1; i++) {
		ret = ad917x_jesd_get_link_status(&st->dac_h, i, &link_status);
		if (ret != 0) {
			dev_err(dev,
				"DAC:MODE:JESD: ERROR : Get Link%d status failed \r\n", i);
			return -EIO;
		}

		dev_info(dev, "Link%d code_grp_sync: %x\n", i, link_status.code_grp_sync_stat);
		dev_info(dev, "Link%d frame_sync_stat: %x\n", i, link_status.frame_sync_stat);
		dev_info(dev, "Link%d good_checksum_stat: %x\n",
			i, link_status.good_checksum_stat);
		dev_info(dev, "Link%d init_lane_sync_stat: %x\n",
			i, link_status.init_lane_sync_stat);
		dev_info(dev, "Link%d %d lanes @ %lu kBps\n",
			i, st->appJesdConfig.jesd_L, lane_rate_kHz);

		if (hweight8(link_status.code_grp_sync_stat) != st->appJesdConfig.jesd_L ||
			link_status.code_grp_sync_stat != link_status.frame_sync_stat ||
			link_status.code_grp_sync_stat != link_status.init_lane_sync_stat)
			return -EFAULT;

		if (!st->jesd_dual_link_mode)
			break;
	}

	return 0;
}

static int ad9172_finalize_setup(struct ad9172_state *st)
{
	ad917x_handle_t *ad917x_h = &st->dac_h;
	int ret;
	u8 dac_mask, chan_mask;

	if (st->jesd_dual_link_mode || st->interpolation == 1)
		dac_mask = AD917X_DAC0 | AD917X_DAC1;
	else
		dac_mask = AD917X_DAC0;

	if (st->interpolation > 1) {
		chan_mask = GENMASK(st->appJesdConfig.jesd_M / 2, 0);
		ret = ad917x_set_page_idx(ad917x_h, AD917X_DAC_NONE, chan_mask);
		if (ret)
			return ret;

		ret = ad917x_set_channel_gain(ad917x_h, 2048); /* GAIN = 1 */
		if (ret)
			return ret;

		st->nco_main_enable = dac_mask;

		ad917x_nco_enable(ad917x_h, st->nco_main_enable, 0);
	}

	ret = ad917x_set_page_idx(ad917x_h, dac_mask, AD917X_CH_NONE);
	if (ret != 0)
		return -EIO;

	return regmap_write(st->map, 0x596, 0x1c);
}

static int ad9172_setup(struct ad9172_state *st)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	uint8_t revision[3] = {0, 0, 0};
	adi_chip_id_t dac_chip_id;
	uint8_t pll_lock_status = 0, dll_lock_stat = 0;
	int ret, i;
	u64 dac_rate_Hz, dac_clkin_Hz;
	unsigned long lane_rate_kHz;
	ad917x_handle_t *ad917x_h = &st->dac_h;
	unsigned long pll_mult;

	st->interpolation = st->dac_interpolation *
			    st->channel_interpolation;

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

	ret = ad917x_get_revision(ad917x_h, &revision[0], &revision[1],
				  &revision[2]);
	if (ret != 0)
		return ret;

	dev_info(dev, "ad917x DAC Chip ID: %d\n", dac_chip_id.chip_type);
	dev_info(dev, "ad917x DAC Product ID: %x\n", dac_chip_id.prod_id);
	dev_info(dev, "ad917x DAC Product Grade: %d\n", dac_chip_id.prod_grade);
	dev_info(dev, "ad917x DAC Product Revision: %d\n",
		 dac_chip_id.dev_revision);
	dev_info(dev, "ad917x Revision: %d.%d.%d\n",
		 revision[0], revision[1], revision[2]);

	dac_clkin_Hz = clk_get_rate_scaled(st->conv.clk[CLK_DAC],
		&st->conv.clkscale[CLK_DAC]);

	dev_info(dev, "CLK Input rate %llu\n", dac_clkin_Hz);

	if (!st->pll_bypass) {
		u64 tmp = dac_clkin_Hz;

		do_div(tmp, 1000);

		pll_mult = DIV_ROUND_CLOSEST_ULL(st->dac_rate_khz, tmp);

		ret = ad917x_set_dac_clk(ad917x_h, dac_clkin_Hz * pll_mult,
			1, dac_clkin_Hz);
	} else {
		ret = ad917x_set_dac_clk(ad917x_h, dac_clkin_Hz, 0,
			dac_clkin_Hz);
	}

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

	if (st->clock_output_config) {
		/* DEBUG: route DAC clock to output, so we can meassure it */
		ret = ad917x_set_clkout_config(ad917x_h,
					       st->clock_output_config);
		if (ret != 0) {
			dev_err(dev, "ad917x_set_clkout_config failed (%d)\n",
				ret);
			return ret;
		}
	}

	ret = ad917x_jesd_config_datapath(ad917x_h, st->jesd_dual_link_mode,
					  st->jesd_link_mode,
					  st->channel_interpolation,
					  st->dac_interpolation);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_config_datapath failed (%d)\n", ret);
		return ret;
	}
	ret = ad917x_jesd_get_cfg_param(ad917x_h, &st->appJesdConfig);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_get_cfg_param failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_jesd_set_scrambler_enable(ad917x_h, st->scrambling);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_set_scrambler_enable failed (%d)\n",
			ret);
		return ret;
	}

	/*Configure xbar*/
	for (i = 0; i < 8; i++) {
		ret = ad917x_jesd_set_lane_xbar(ad917x_h, i, st->logic_lanes[i]);
		if (ret != 0)
			return ret;
	}

	ret = ad917x_jesd_enable_datapath(ad917x_h, 0xFF, 0x1, 0x1);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_enable_datapath failed (%d)\n", ret);
		return ret;
	}

	ret = ad917x_jesd_set_syncoutb_enable(ad917x_h, SYNCOUTB_0, 1);
	if (ret != 0) {
		dev_err(dev, "ad917x_jesd_set_syncoutb_enable failed (%d)\n",
			ret);
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

	/* No need to continue here when jesd204-fsm enabled */
	if (st->jdev)
		return 0;

	ad917x_get_dac_clk_freq(ad917x_h, &dac_rate_Hz);

	lane_rate_kHz = div_u64(dac_rate_Hz * 20 * st->appJesdConfig.jesd_M,
				st->appJesdConfig.jesd_L *
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

	devm_add_action_or_reset(dev,
				 (void(*)(void *))clk_disable_unprepare,
				 st->conv.clk[CLK_DATA]);

	ad917x_jesd_set_sysref_enable(ad917x_h, !!st->jesd_subclass);

	/*Enable Link*/
	ret = ad917x_jesd_enable_link(ad917x_h, JESD_LINK_ALL, 0x1);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Enable Link failed\n");
		return -EIO;
	}

	msleep(100);

	ret = ad9172_link_status_get(st, lane_rate_kHz);
	if (ret != 0) {
		dev_err(dev, "DAC:MODE:JESD: ERROR : Link status failed\n");
		return ret;
	}

	return ad9172_finalize_setup(st);
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

			devm_add_action_or_reset(&conv->spi->dev,
					(void(*)(void *))clk_disable_unprepare,
					clk);
		}

		of_clk_get_scale(conv->spi->dev.of_node,
				 clk_names[i], &conv->clkscale[i]);
		conv->clk[i] = clk;
	}

	return 0;
}

static int ad9172_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	ad917x_handle_t *ad917x_h = &st->dac_h;
	u16 val16 = 0;
	u8 cached_page_mask;
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad9172_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ad917x_register_read(&st->dac_h,
			AD917X_SPI_PAGEINDX_REG, &cached_page_mask);

		ret = ad917x_set_page_idx(ad917x_h,
					  AD917X_DAC_NONE, BIT(chan->channel));
		if (ret < 0)
			return ret;
		ret = ad917x_get_channel_gain(ad917x_h, &val16);
		if (ret < 0)
			return ret;

		ad917x_register_write(&st->dac_h,
			AD917X_SPI_PAGEINDX_REG, cached_page_mask);

		*val = val16;
		*val2 = 11;
		return IIO_VAL_FRACTIONAL_LOG2;
	}
	return -EINVAL;
}

static int ad9172_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	ad917x_handle_t *ad917x_h = &st->dac_h;
	u16 val16;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		val16 = clamp_t(u16, val * 2048 + (val2 * 2048 / 1000000),
				0, 4095);
		ret = ad917x_set_page_idx(ad917x_h,
					  AD917X_DAC_NONE, BIT(chan->channel));
		if (ret < 0)
			return ret;
		ret = ad917x_set_channel_gain(ad917x_h, val16);
		if (ret < 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9172_prepare(struct cf_axi_converter *conv)
{
	return 0;
}

static const struct regmap_config ad9172_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	.cache_type = REGCACHE_NONE,
};

static int delay_us(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int ad9172_spi_xfer(void *user_data, uint8_t *wbuf,
			   uint8_t *rbuf, int len)
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
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	ad917x_handle_t *ad917x_h = &st->dac_h;
	long long readin;
	u32 dest = (u32)this_attr->address & 0xFF;
	int ret;
	s16 val16;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&conv->lock);

	switch ((u32)this_attr->address & ~0xFF) {
	case AD9172_ATTR_CHAN_NCO(0):
		readin *= st->dac_interpolation;
		ret = ad917x_nco_set(ad917x_h, AD917X_DAC_NONE, BIT(dest),
				     readin, 0x1FFF, 0, 0);
		st->nco_channel_enable |= BIT(dest);
		break;
	case AD9172_ATTR_CHAN_PHASE(0):
		val16 = div_s64(readin * 32768, 180000LL);
		ret = ad917x_nco_set_phase_offset(ad917x_h, AD917X_DAC_NONE, 0,
						  BIT(dest), val16);
		break;
	case AD9172_ATTR_MAIN_NCO(0):
		ret = ad917x_nco_set(ad917x_h, BIT(dest), AD917X_CH_NONE,
				     readin, 0x1FFF, 0, 0);
		st->nco_main_enable |= BIT(dest);
		break;
	case AD9172_ATTR_MAIN_PHASE(0):
		val16 = div_s64(readin * 32768, 180000LL);
		ret = ad917x_nco_set_phase_offset(ad917x_h, BIT(dest), val16,
						  AD917X_CH_NONE, 0);
		break;
	case AD9172_ATTR_CHAN_NCO_EN(0):
		if (readin)
			st->nco_channel_enable |= BIT(dest);
		else
			st->nco_channel_enable &= ~BIT(dest);

		ret = ad917x_nco_enable(ad917x_h, st->nco_main_enable,
					st->nco_channel_enable);
		break;
	case AD9172_ATTR_MAIN_NCO_EN(0):
		if (readin)
			st->nco_main_enable |= BIT(dest);
		else
			st->nco_main_enable &= ~BIT(dest);

		ret = ad917x_nco_enable(ad917x_h, st->nco_main_enable,
					st->nco_channel_enable);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&conv->lock);

	return ret ? ret : len;
}

static ssize_t ad9172_attr_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	ad917x_handle_t *ad917x_h = &st->dac_h;
	u32 dest = (u32)this_attr->address & 0xFF;
	int ret = 0;
	s64 val64 = 0;
	s16 val16_2, val16 = 0;

	mutex_lock(&conv->lock);
	switch ((u32)this_attr->address & ~0xFF) {
	case AD9172_ATTR_CHAN_NCO(0):
		ret = ad917x_nco_channel_freq_get(ad917x_h, BIT(dest),
						  &val64);
		val64 = div_s64(val64, st->dac_interpolation);
		break;
	case AD9172_ATTR_CHAN_PHASE(0):
		ret = ad917x_nco_get_phase_offset(ad917x_h,
						  AD917X_DAC_NONE, &val16_2,
						  BIT(dest), &val16);
		val64 = div_s64(val16 * 180000LL, 32768);
		break;
	case AD9172_ATTR_MAIN_NCO(0):
		ret = ad917x_nco_main_freq_get(ad917x_h, BIT(dest),
						  &val64);
		break;
	case AD9172_ATTR_MAIN_PHASE(0):
		ret = ad917x_nco_get_phase_offset(ad917x_h, BIT(dest), &val16,
						  AD917X_CH_NONE, &val16_2);
		val64 = div_s64(val16 * 180000LL, 32768);
		break;
	case AD9172_ATTR_CHAN_NCO_EN(0):
		val64 = !!(st->nco_channel_enable & BIT(dest));
		break;
	case AD9172_ATTR_MAIN_NCO_EN(0):
		val64 = !!(st->nco_main_enable & BIT(dest));
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&conv->lock);

	if (ret >= 0)
		ret = sprintf(buf, "%lld\n", val64);

	return ret;
}

static IIO_DEVICE_ATTR(out_voltage0_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(0));

static IIO_DEVICE_ATTR(out_voltage1_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(1));

static IIO_DEVICE_ATTR(out_voltage2_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(2));

static IIO_DEVICE_ATTR(out_voltage3_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(3));

static IIO_DEVICE_ATTR(out_voltage4_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(4));

static IIO_DEVICE_ATTR(out_voltage5_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO(5));

static IIO_DEVICE_ATTR(out_voltage6_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_NCO(0));

static IIO_DEVICE_ATTR(out_voltage7_nco_frequency, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_NCO(1));

static IIO_DEVICE_ATTR(out_voltage0_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(0));

static IIO_DEVICE_ATTR(out_voltage1_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(1));

static IIO_DEVICE_ATTR(out_voltage2_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(2));

static IIO_DEVICE_ATTR(out_voltage3_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(3));

static IIO_DEVICE_ATTR(out_voltage4_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(4));

static IIO_DEVICE_ATTR(out_voltage5_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_NCO_EN(5));

static IIO_DEVICE_ATTR(out_voltage6_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_NCO_EN(0));

static IIO_DEVICE_ATTR(out_voltage7_nco_enable, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_NCO_EN(1));

static IIO_DEVICE_ATTR(out_voltage0_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(0));

static IIO_DEVICE_ATTR(out_voltage1_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(1));

static IIO_DEVICE_ATTR(out_voltage2_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(2));

static IIO_DEVICE_ATTR(out_voltage3_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(3));

static IIO_DEVICE_ATTR(out_voltage4_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(4));

static IIO_DEVICE_ATTR(out_voltage5_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_CHAN_PHASE(5));

static IIO_DEVICE_ATTR(out_voltage6_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_PHASE(0));

static IIO_DEVICE_ATTR(out_voltage7_nco_phase, 0644,
		       ad9172_attr_show,
		       ad9172_attr_store,
		       AD9172_ATTR_MAIN_PHASE(1));

static struct attribute *ad9172_attributes_dual_m2[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_enable.dev_attr.attr,
	NULL,
};

static struct attribute *ad9172_attributes_dual_m4[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_enable.dev_attr.attr,
	NULL,
};

static struct attribute *ad9172_attributes_dual_m6[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage3_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage4_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage5_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage5_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage5_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage7_nco_enable.dev_attr.attr,
	NULL,
};

static struct attribute *ad9172_attributes_m2[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	NULL,
};

static struct attribute *ad9172_attributes_m4[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	NULL,
};

static struct attribute *ad9172_attributes_m6[] = {
	&iio_dev_attr_out_voltage0_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage0_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage1_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage2_nco_enable.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_phase.dev_attr.attr,
	&iio_dev_attr_out_voltage6_nco_enable.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9172_attribute_group_dual_m6 = {
	.attrs = ad9172_attributes_dual_m6,
};

static const struct attribute_group ad9172_attribute_group_dual_m4 = {
	.attrs = ad9172_attributes_dual_m4,
};

static const struct attribute_group ad9172_attribute_group_dual_m2 = {
	.attrs = ad9172_attributes_dual_m2,
};

static const struct attribute_group ad9172_attribute_group_m6 = {
	.attrs = ad9172_attributes_m6,
};

static const struct attribute_group ad9172_attribute_group_m4 = {
	.attrs = ad9172_attributes_m4,
};

static const struct attribute_group ad9172_attribute_group_m2 = {
	.attrs = ad9172_attributes_m2,
};

static int ad9172_parse_dt(struct spi_device *spi, struct ad9172_state *st)
{
	struct device_node *np = spi->dev.of_node;
	int ret, i;

	st->dac_rate_khz = AD9172_SAMPLE_RATE_KHZ;
	of_property_read_u32(np, "adi,dac-rate-khz", &st->dac_rate_khz);

	st->jesd_link_mode = 10;
	of_property_read_u32(np, "adi,jesd-link-mode", &st->jesd_link_mode);

	st->jesd_dual_link_mode = 0;
	of_property_read_u32(np, "adi,dual-link", &st->jesd_dual_link_mode);

	st->jesd_subclass = 0;
	of_property_read_u32(np, "adi,jesd-subclass", &st->jesd_subclass);

	st->pll_bypass = of_property_read_bool(np, "adi,direct-clocking-enable");

	st->scrambling = 1;
	of_property_read_u32(np, "", &st->scrambling);

	st->dac_interpolation = 1;
	of_property_read_u32(np, "adi,dac-interpolation",
			     &st->dac_interpolation);

	st->channel_interpolation = 1;
	of_property_read_u32(np, "adi,channel-interpolation",
			     &st->channel_interpolation);

	st->clock_output_config = 0;
	of_property_read_u32(np, "adi,clock-output-divider",
			     &st->clock_output_config);

	if (of_property_read_bool(np, "adi,syncoutb-signal-type-lvds-enable"))
		st->syncoutb_type = SIGNAL_LVDS;
	else
		st->syncoutb_type = SIGNAL_CMOS;

	if (of_property_read_bool(np, "adi,sysref-coupling-dc-enable"))
		st->sysref_coupling = COUPLING_DC;
	else
		st->sysref_coupling = COUPLING_AC;

	if (of_property_read_u32(np, "adi,sysref-mode", &st->sysref_mode))
		st->sysref_mode = SYSREF_CONT;

	/*Logic lane configuration*/
	ret = of_property_read_u8_array(np,"adi,logic-lanes-mapping",
				      st->logic_lanes, sizeof(st->logic_lanes));
	if (ret)
		for(i = 0; i < sizeof(st->logic_lanes); i++)
			st->logic_lanes[i] = i;

	return 0;
}

static int ad9172_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9172_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9172_state *st = priv->st;
	u64 rate;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	lnk->num_converters = st->appJesdConfig.jesd_M;
	lnk->num_lanes = st->appJesdConfig.jesd_L;
	lnk->octets_per_frame = st->appJesdConfig.jesd_F;
	lnk->frames_per_multiframe = st->appJesdConfig.jesd_K;
	lnk->device_id = st->appJesdConfig.jesd_DID;
	lnk->bank_id = st->appJesdConfig.jesd_BID;
	lnk->scrambling = st->scrambling;
	lnk->bits_per_sample = st->appJesdConfig.jesd_NP;
	lnk->converter_resolution = st->appJesdConfig.jesd_N;
	lnk->ctrl_bits_per_sample = st->appJesdConfig.jesd_CS;
	lnk->jesd_version = JESD204_VERSION_B;
	lnk->subclass = st->jesd_subclass ?
		JESD204_SUBCLASS_1 : JESD204_SUBCLASS_0;
	lnk->high_density = st->appJesdConfig.jesd_HD;
	lnk->samples_per_conv_frame = st->appJesdConfig.jesd_S;

	ret = ad917x_get_dac_clk_freq(&st->dac_h, &rate);
	if (ret)
		return ret;

	lnk->sample_rate = rate;
	lnk->sample_rate_div = st->interpolation;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;
	lnk->is_transmit = true;

	if (st->sysref_mode == SYSREF_CONT)
		lnk->sysref.mode = JESD204_SYSREF_CONTINUOUS;
	else if (st->sysref_mode == SYSREF_ONESHOT)
		lnk->sysref.mode = JESD204_SYSREF_ONESHOT;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9172_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9172_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9172_state *st = priv->st;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	ad917x_jesd_set_sysref_enable(&st->dac_h, !!st->jesd_subclass);

	/*Enable Link*/
	ret = ad917x_jesd_enable_link(&st->dac_h, JESD_LINK_ALL,
		reason == JESD204_STATE_OP_REASON_INIT);
	if (ret != 0) {
		dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
		return -EFAULT;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9172_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9172_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9172_state *st = priv->st;
	unsigned long lane_rate_khz;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	ret = jesd204_link_get_rate_khz(lnk, &lane_rate_khz);
	if (ret) {
		dev_err(dev, "Failed JESD204 link get rate (%d)\n", ret);
		return ret;
	}

	ret = ad9172_link_status_get(st, lane_rate_khz);
	if (ret) {
		dev_err(dev, "Failed JESD204 link status (%d)\n", ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9172_jesd204_post_running_stage(struct jesd204_dev *jdev,
	enum jesd204_state_op_reason reason)
{
	struct ad9172_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9172_state *st = priv->st;
	int ret;

	ret = ad9172_finalize_setup(st);

	if (ret < 0)
		return ret;

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9172_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9172_jesd204_link_init,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9172_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9172_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = ad9172_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 1,
	.num_retries = 2,
	.sizeof_priv = sizeof(struct ad9172_jesd204_priv),
};

static int ad9172_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9172_state *st;
	int ret;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->id = (enum chip_id) dev_id->driver_data;
	conv = &st->conv;

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	conv->txen_gpio[0] = devm_gpiod_get_optional(&spi->dev, "txen0",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(conv->txen_gpio[0]))
		return PTR_ERR(conv->txen_gpio[0]);

	conv->txen_gpio[1] = devm_gpiod_get_optional(&spi->dev, "txen1",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(conv->txen_gpio[1]))
		return PTR_ERR(conv->txen_gpio[1]);

	st->map = devm_regmap_init_spi(spi, &ad9172_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	ret = ad9172_parse_dt(spi, st);
	if (ret < 0)
		return ret;

	st->jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9172_init);
	if (IS_ERR(st->jdev))
		return PTR_ERR(st->jdev);

	if (st->jdev) {
		struct ad9172_jesd204_priv *priv;

		priv = jesd204_dev_priv(st->jdev);
		priv->st = st;
	}

	conv->write = ad9172_write;
	conv->read = ad9172_read;
	conv->setup = ad9172_prepare;
	conv->get_data_clk = ad9172_get_data_clk;
	conv->write_raw = ad9172_write_raw;
	conv->read_raw = ad9172_read_raw;
	conv->spi = spi;
	conv->id = ID_AUTO_SYNTH_PARAM;

	ret = ad9172_get_clks(conv);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	st->dac_h.user_data = spi;
	st->dac_h.sdo = SPI_SDO;
	st->dac_h.dev_xfer = ad9172_spi_xfer;
	st->dac_h.delay_us = delay_us;
	st->dac_h.tx_en_pin_ctrl = NULL;
	st->dac_h.reset_pin_ctrl = NULL;
	st->dac_h.syncoutb = st->syncoutb_type;
	st->dac_h.sysref = st->sysref_coupling;

	ret = ad9172_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	if (st->interpolation == 1) {
		conv->attrs = NULL;
	} else {
		switch (st->appJesdConfig.jesd_M) {
		case 2:
			if (st->jesd_dual_link_mode)
				conv->attrs = &ad9172_attribute_group_dual_m2;
			else
				conv->attrs = &ad9172_attribute_group_m2;
			break;
		case 4:
			if (st->jesd_dual_link_mode)
				conv->attrs = &ad9172_attribute_group_dual_m4;
			else
				conv->attrs = &ad9172_attribute_group_m4;
			break;
		case 6:
			if (st->jesd_dual_link_mode)
				conv->attrs = &ad9172_attribute_group_dual_m6;
			else
				conv->attrs = &ad9172_attribute_group_m6;
			break;
		default:
			return -EINVAL;
		}
	}

	spi_set_drvdata(spi, conv);

	dev_info(&spi->dev, "Probed.\n");

	return jesd204_fsm_start(st->jdev, JESD204_LINKS_ALL);
out:
	return ret;
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

static const struct of_device_id ad9172_of_match[] = {
	{ .compatible = "adi,ad9171" },
	{ .compatible = "adi,ad9172" },
	{ .compatible = "adi,ad9173" },
	{ .compatible = "adi,ad9174" },
	{ .compatible = "adi,ad9175" },
	{ .compatible = "adi,ad9176" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad9172_of_match);

static struct spi_driver ad9172_driver = {
	.driver = {
		.name = "ad9172",
		.of_match_table = of_match_ptr(ad9172_of_match),
	},
	.probe = ad9172_probe,
	.id_table = ad9172_id,
};

module_spi_driver(ad9172_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9172 DAC");
MODULE_LICENSE("GPL");
