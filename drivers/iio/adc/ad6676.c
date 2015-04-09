/*
 * AD6676 SPI driver for AXIADC/FPGA Module
 *
 * Copyright 2014-2015 Analog Devices Inc.
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
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"
#include "ad6676.h"

struct ad6676_jesd_conf {
	bool scrambling_en;
	bool lvds_syncb;
	bool sysref_pd;
	u8 l; /* lanes */
	u8 f; /* Frames per multi frame */
};

struct ad6676_base_conf {
	u32 f_adc_hz;
	u32 f_if_hz;
	u32 f_if_min_hz; /* depend on Lext */
	u32 f_if_max_hz; /* depend on Lext */
	u32 bw_hz;
	u8 bw_margin_low_mhz;
	u8 bw_margin_high_mhz;
	s8 bw_margin_if_mhz;
	u8 decimation;
	u8 ext_l;
	u8 attenuation;
	u8 scale;
	bool use_extclk;
	bool fadc_fixed;
};

struct ad6676_agc_conf {
	s16 dec_peak_thresh_a;
	s16 dec_peak_thresh_b;
	s16 dec_low_thresh;
	u8 dwell_mantissa;
	u8 dwell_exp;
};

struct ad6676_shuffler_conf {
	u8 shuffle_ctrl;
	u8 shuffle_thresh;
};

struct ad6676_platform_data {
	struct ad6676_base_conf base;
	struct ad6676_jesd_conf jesd;
	struct ad6676_agc_conf agc;
	struct ad6676_shuffler_conf shuffler;
	bool spi3wire;
};

struct ad6676_phy {
	struct ad6676_platform_data *pdata;
	unsigned long ref_clk;
	u32 m;
};

enum ad6676_iio_dev_attr {
	AD6676_ATTR_FADC,
	AD6676_ATTR_FIF,
	AD6676_ATTR_BW,
	AD6676_ATTR_MRGN_L,
	AD6676_ATTR_MRGN_H,
	AD6676_ATTR_MRGN_IF,
	AD6676_ATTR_SHUF_TH,
};

static inline struct ad6676_phy *conv_to_phy(struct axiadc_converter *conv)
{
	return conv->phy;
}

static int ad6676_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;
	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, buf[2], ret);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	return buf[2];
}

static int ad6676_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

	return 0;
}

static int ad6676_set_splitreg(struct spi_device *spi, u32 reg, u32 val)
{
	int ret;

	ret = ad6676_spi_write(spi, reg, val & 0xFF);
	ret |= ad6676_spi_write(spi, reg + 1, val >> 8);

	return ret;
}

static inline int ad6676_get_splitreg(struct spi_device *spi, u32 reg, u32 *val)
{
	int ret;

	ret = ad6676_spi_read(spi, reg);
	if (ret < 0)
		return ret;

	*val = ret;

	ret = ad6676_spi_read(spi, reg + 1);
	if (ret < 0)
		return ret;

	*val |= ((u32)ret << 8);

	return ret;
}

static int ad6676_set_fadc(struct axiadc_converter *conv, u32 val)
{
	return ad6676_set_splitreg(conv->spi, AD6676_FADC_0,
		clamp_t(u32, val, MIN_FADC, MAX_FADC) / MHz);
}

static inline u32 ad6676_get_fadc(struct axiadc_converter *conv)
{
	u32 val;
	int ret = ad6676_get_splitreg(conv->spi, AD6676_FADC_0, &val);
	if (ret < 0)
		return 0;

	return val * MHz;
}

static int ad6676_set_fif(struct axiadc_converter *conv, u32 val)
{
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;

	return ad6676_set_splitreg(conv->spi, AD6676_FIF_0,
		clamp_t(u32, val, pdata->base.f_if_min_hz,
			pdata->base.f_if_max_hz) / MHz);
}

static u32 ad6676_get_fif(struct axiadc_converter *conv)
{
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;
	s64 mix1, mix2;

	mix1 = ad6676_spi_read(conv->spi, AD6676_MIX1_TUNING);
	mix2 = (s8)ad6676_spi_read(conv->spi, AD6676_MIX2_TUNING);

	mix1 = mix1 * pdata->base.f_adc_hz;
	mix2 = mix2 * pdata->base.f_adc_hz;

	do_div(mix1, 64);
	do_div(mix2, phy->m);

	return mix1 + mix2;
}

static int ad6676_set_bw(struct axiadc_converter *conv, u32 val)
{
	return ad6676_set_splitreg(conv->spi, AD6676_BW_0,
		clamp_t(u32, val, MIN_BW, MAX_BW) / MHz);
}

static inline u32 ad6676_get_bw(struct axiadc_converter *conv)
{
	u32 val;
	int ret = ad6676_get_splitreg(conv->spi, AD6676_BW_0, &val);
	if (ret < 0)
		return 0;

	return val * MHz;
}

static int ad6676_set_decimation(struct axiadc_converter *conv, u32 val)
{
	struct ad6676_phy *phy = conv_to_phy(conv);

	switch (val) {
	case 32:
		val = DEC_32;
		phy->m = 4096;
		break;
	case 24:
		val = DEC_24;
		phy->m = 3072;
		break;
	case 16:
		val = DEC_16;
		phy->m = 4096;
		break;
	case 12:
		val = DEC_12;
		phy->m = 3072;
		break;
	default:
		return -EINVAL;
	}

	return ad6676_spi_write(conv->spi, AD6676_DEC_MODE, val);
}

static int ad6676_set_clk_synth(struct axiadc_converter *conv, u32 refin_Hz, u32 freq)
{
	struct spi_device *spi = conv->spi;
	u32 f_pfd, reg_val, tout, div_val;
	u64 val64;
	int ret;

	dev_dbg(&spi->dev, "%s: REFin %u frequency %u\n",
		__func__, refin_Hz, freq);

	if (refin_Hz < 40000000UL) {
		f_pfd = 2 * refin_Hz;
		div_val = R_DIV(3);
	} else if (refin_Hz < 80000000UL) {
		f_pfd = refin_Hz;
		div_val = R_DIV(0);
	} else if (refin_Hz < 160000000UL) {
		f_pfd = refin_Hz / 2;
		div_val = R_DIV(1);
	} else if (refin_Hz < 320000000UL) {
		f_pfd = refin_Hz / 4;
		div_val = R_DIV(2);
	} else
		return -EINVAL;

	/* Compute N val */

	freq = clamp_t(u32, freq, MIN_FADC_INT_SYNTH, MAX_FADC);

	reg_val = freq / (f_pfd / 2);
	ret = ad6676_set_splitreg(spi, AD6676_CLKSYN_INT_N_LSB, reg_val); /* 2A0 */
	if (ret < 0)
		return ret;

	ret = ad6676_spi_write(spi, AD6676_CLKSYN_LOGEN, RESET_CAL);
	if (ret < 0)
		return ret;

	/* Compute I_CP val */

	reg_val = (f_pfd / MHz) * (freq / MHz) * (freq / MHz);
	val64 = 13300000000ULL + reg_val / 2;
	do_div(val64, reg_val);
	reg_val = min_t(u64, 64U, val64 - 1);

	ret = ad6676_spi_write(spi, AD6676_CLKSYN_I_CP, reg_val); /* I_CP 2AC */
	if (ret < 0)
		return ret;

// 	ret = ad6676_spi_write(spi, 0x2B2, 0x80); /* Reference Div */
// 	if (ret < 0)
// 		return ret;

	/* VCO Configuration Settings */
	if (freq <= 2950000000UL)
		reg_val = 0xF0;
	else if (freq < 3100000000UL)
		reg_val = 0xE0;
	else
		reg_val = 0xD0;

	ret = ad6676_spi_write(spi, AD6676_CLKSYN_VCO_BIAS, 0x37); /* 2AA */
	if (ret < 0)
		return ret;

	ret = ad6676_spi_write(spi, AD6676_CLKSYN_VCO_VAR, reg_val); /* 2B7 */
	if (ret < 0)
		return ret;

	ret = ad6676_spi_write(spi, AD6676_CLKSYN_R_DIV,
			div_val | CLKSYN_R_DIV_RESERVED); /* Reference Div 2BB */
	if (ret < 0)
		return ret;


	/* Enable CLKSYN and ADC clock */
	ret = ad6676_spi_write(spi, AD6676_CLKSYN_ENABLE, /* 2A0 */
		EN_OVER_IDE | EN_VCO | EN_VCO_ALC |
		EN_VCO_PTAT | EN_ADC_CK | EN_SYNTH);
	if (ret < 0)
		return ret;

	/* Start VCO calibration */
	ret = ad6676_spi_write(spi, AD6676_CLKSYN_VCO_CAL, /* 2AB */
		INIT_ALC_VALUE(0xC) | 0x5);
	if (ret < 0)
		return ret;

	tout = 4;
	do {
		mdelay(1);
		reg_val = ad6676_spi_read(spi, AD6676_CLKSYN_STATUS);
	} while (--tout && (reg_val & SYN_STAT_VCO_CAL_BUSY));

	if (!tout)
		dev_err(&spi->dev, "VCO CAL failed (0x%X)\n", reg_val);


	/* Start CP calibration */
	ret = ad6676_spi_write(spi, AD6676_CLKSYN_CP_CAL, CP_CAL_EN); /* 2AD */
	if (ret < 0)
		return ret;

	tout = 4;
	do {
		mdelay(1);
		reg_val = ad6676_spi_read(spi, AD6676_CLKSYN_STATUS);
		reg_val &= SYN_STAT_PLL_LCK | SYN_STAT_VCO_CAL_BUSY | SYN_STAT_CP_CAL_DONE;
	} while (--tout && (reg_val != (SYN_STAT_PLL_LCK | SYN_STAT_CP_CAL_DONE)));

	if (!tout) {
		dev_err(&spi->dev, "Synthesizer PLL unlocked (0x%X)\n", reg_val);
		return -EFAULT;
	}

	return 0;
}

static int ad6676_jesd_setup(struct axiadc_converter *conv, struct ad6676_jesd_conf *conf)
{
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad6676_spi_write(spi, AD6676_SYNCB_CTRL,
		(conf->sysref_pd ? PD_SYSREF_RX : 0) |
		(conf->lvds_syncb ? LVDS_SYNCB : 0)); // lvds sync_n
	ret |= ad6676_spi_write(spi, AD6676_DID, 0x01); // device id
	ret |= ad6676_spi_write(spi, AD6676_BID, 0x05); // bank id
	ret |= ad6676_spi_write(spi, AD6676_L, (conf->l - 1) |
		(conf->scrambling_en ? SCR : 0)); // scrambling, 2 lanes
	ret |= ad6676_spi_write(spi, AD6676_F, 0x01); // 2 bytes/frame
	ret |= ad6676_spi_write(spi, AD6676_K, conf->f - 1); // 16 frames/multi-frame
	ret |= ad6676_spi_write(spi, AD6676_M, 0x01); // 2 converters
	ret |= ad6676_spi_write(spi, AD6676_S, 0x00); // 1 samples per frame
	ret |= ad6676_spi_write(spi, AD6676_SER2, 0xBD);

	return ret;

}

static int ad6676_shuffle_setup(struct axiadc_converter *conv, struct ad6676_shuffler_conf *conf)
{
	u32 reg_val = 0, val, thresh;
	int i;

	thresh = clamp_t(u8, conf->shuffle_thresh, 0, 8U);

	for (i = 0; i < 4; i++) {
		if ((i + 1) == conf->shuffle_ctrl)
			val = thresh;
		else
			val = 0xF;

		reg_val |= (val << (i * 4));
	}

	return ad6676_set_splitreg(conv->spi, AD6676_SHUFFLE_THREG0, reg_val);
}

static int ad6676_calibrate(struct axiadc_converter *conv, unsigned cal)
{
	struct spi_device *spi = conv->spi;
	int tout_i = 2, tout_o = 2;
	u32 done;

	do {
		ad6676_spi_write(spi, AD6676_CAL_CMD, cal);

		do {
			mdelay(250);
			done = ad6676_spi_read(spi, AD6676_CAL_DONE) & CAL_DONE;
		} while (!done && tout_i--);

		if (!done) {
			dev_dbg(&spi->dev, "CAL timeout (0x%X)\n", cal);
			ad6676_spi_write(spi, AD6676_FORCE_END_CAL, FORCE_END_CAL);
			ad6676_spi_write(spi, AD6676_FORCE_END_CAL, 0);
		} else {
			return 0;
		}

	} while (tout_o--);

	dev_err(&spi->dev, "CAL failed (0x%X)\n", cal);

	return -EFAULT;
}

static int ad6676_reset(struct axiadc_converter *conv, bool spi3wire)
{
	int ret = ad6676_spi_write(conv->spi, AD6676_SPI_CONFIG,
				   SPI_CONF_SW_RESET |
				   (spi3wire ? 0 : SPI_CONF_SDIO_DIR));
	mdelay(2);

	return ret;
}

static int ad6676_setup(struct axiadc_converter *conv)
{
	struct spi_device *spi = conv->spi;
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;
	struct clk *clk;
	int ret;
	unsigned reg_val;

	clk = devm_clk_get(&spi->dev, "ref_clk");
	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret < 0)
			return ret;

		phy->ref_clk = clk_get_rate(clk);
	}

	ret = ad6676_reset(conv, (spi->mode & SPI_3WIRE) || pdata->spi3wire);
	if (ret < 0)
		return ret;

	ad6676_set_clk_synth(conv, phy->ref_clk, pdata->base.f_adc_hz);

	ad6676_jesd_setup(conv, &phy->pdata->jesd);

	ad6676_set_fadc(conv, pdata->base.f_adc_hz);
	ad6676_set_fif(conv, pdata->base.f_if_hz);
	ad6676_set_bw(conv, pdata->base.bw_hz);


	ret |= ad6676_spi_write(spi, AD6676_LEXT, pdata->base.ext_l);
	ret |= ad6676_spi_write(spi, AD6676_MRGN_L, pdata->base.bw_margin_low_mhz);
	ret |= ad6676_spi_write(spi, AD6676_MRGN_U, pdata->base.bw_margin_high_mhz);
	ret |= ad6676_spi_write(spi, AD6676_MRGN_IF, pdata->base.bw_margin_if_mhz);
	ret |= ad6676_spi_write(spi, AD6676_XSCALE_1, pdata->base.scale);

	ad6676_set_decimation(conv, 32);

	ret = ad6676_calibrate(conv, RESON1_CAL | INIT_ADC);

	ad6676_set_decimation(conv, pdata->base.decimation);

	ret = ad6676_calibrate(conv, XCMD0 | XCMD1 | INIT_ADC | TUNE_ADC | FLASH_CAL);

// 	ad6676_spi_write(spi, 0x118, 0x00);
// 	ad6676_spi_write(spi, 0x115, 0x00);
// 	ad6676_spi_write(spi, 0x181, 0x00);
// 	ad6676_spi_write(spi, 0x182, 0x00);
// 	ad6676_spi_write(spi, 0x19E, 0x02);

	ret = clk_prepare_enable(conv->clk);
	if (ret < 0)
		return ret;

	conv->clk = clk;

	reg_val = ad6676_spi_read(spi, AD6676_JESDSYN_STATUS);
	reg_val &= SYN_STAT_PLL_LCK;

	if (reg_val != SYN_STAT_PLL_LCK) {
		dev_err(&spi->dev, "JESD PLL unlocked (0x%X)\n", reg_val);
		return -EFAULT;
	}

	return 0;
}

static int ad6676_update(struct axiadc_converter *conv, struct ad6676_platform_data *pdata)
{
	struct spi_device *spi = conv->spi;
	int ret;

	pdata->base.bw_hz = clamp_t(u32, pdata->base.bw_hz,
				    pdata->base.f_adc_hz / 200,
			     pdata->base.f_adc_hz / 20);

	pdata->base.f_if_hz = clamp_t(u32, pdata->base.f_if_hz,
				pdata->base.f_if_min_hz,
				pdata->base.f_if_max_hz);

	ret = ad6676_set_fif(conv, pdata->base.f_if_hz);
	ret |= ad6676_set_bw(conv, pdata->base.bw_hz);

	ret |= ad6676_spi_write(spi, AD6676_MRGN_L, pdata->base.bw_margin_low_mhz);
	ret |= ad6676_spi_write(spi, AD6676_MRGN_U, pdata->base.bw_margin_high_mhz);
	ret |= ad6676_spi_write(spi, AD6676_MRGN_IF, pdata->base.bw_margin_if_mhz);
	ret |= ad6676_spi_write(spi, AD6676_XSCALE_1, pdata->base.scale);

	ret = ad6676_calibrate(conv, RESON1_CAL | INIT_ADC);
	ret = ad6676_calibrate(conv, XCMD0 | XCMD1 | INIT_ADC | TUNE_ADC | FLASH_CAL);

	pdata->base.f_if_hz = ad6676_get_fif(conv);

	return ret;
}

static int ad6676_outputmode_set(struct axiadc_converter *conv, unsigned mode)
{
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad6676_spi_write(spi, AD6676_DP_CTRL, mode);
	if (ret < 0)
		return ret;

	return ad6676_spi_write(spi, AD6676_TEST_GEN, TESTGENMODE_OFF);
}

static int ad6676_testmode_set(struct iio_dev *indio_dev,
			       unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad6676_spi_write(conv->spi, AD6676_TEST_GEN, mode);

	conv->testmode[chan] = mode;
	return 0;
}

static int ad6676_test_and_outputmode_set(struct iio_dev *indio_dev,
					  unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret;

	if (mode == TESTGENMODE_OFF)
		ret = ad6676_spi_write(conv->spi, AD6676_DP_CTRL,
				       conv->adc_output_mode);
	else
		ret = ad6676_spi_write(conv->spi, AD6676_DP_CTRL,
				       DP_CTRL_OFFSET_BINARY);

	if (ret < 0)
		return ret;

	return ad6676_testmode_set(indio_dev, chan, mode);
}

static const char *const testmodes[] = {
	[TESTGENMODE_OFF] = "off",
	[TESTGENMODE_ALT_CHECKERBOARD] = "checkerboard",
	[TESTGENMODE_PN23_SEQ] = "pn23",
	[TESTGENMODE_PN9_SEQ] = "pn9",
	[TESTGENMODE_REP_USER_PAT] = "rep_user_pat",
	[TESTGENMODE_SING_USER_PAT] = "sing_user_pat",
	[TESTGENMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[TESTGENMODE_RAMP] = "ramp",
	[TESTGENMODE_MOD_RPAT] = "mod_rpat",
	[TESTGENMODE_JSPAT] = "jspat",
	[TESTGENMODE_JTSPAT] = "jtspat",
};

static ssize_t ad6676_testmode_mode_available(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	size_t len = 0;
	int i;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (testmodes[i])
			len += sprintf(buf + len, "%s ", testmodes[i]);
	}
	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t axiadc_testmode_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return sprintf(buf, "%s\n", testmodes[conv->testmode[chan->channel]]);
}

static ssize_t axiadc_testmode_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode, i;
	int ret;

	mode = 0;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (testmodes[i] && sysfs_streq(buf, testmodes[i])) {
			mode = i;
			break;
		}
	}

	mutex_lock(&indio_dev->mlock);
	ret = ad6676_testmode_set(indio_dev, chan->channel, mode);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

#define update(old, new) {if (new != old) {old = new; update = 1;} else {update = 0;}}

static ssize_t ad6676_extinfo_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;
	bool update = false;
	s64 readin;
	int ret = 0;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)private) {
	case AD6676_ATTR_FADC:
		if (pdata->base.fadc_fixed) {
			ret = -EINVAL;
			break;
		}
		update(pdata->base.f_adc_hz, readin);
		break;
	case AD6676_ATTR_FIF:
		update(pdata->base.f_if_hz, readin);
		break;
	case AD6676_ATTR_BW:
		update(pdata->base.bw_hz, readin);
		break;
	case AD6676_ATTR_MRGN_L:
		update(pdata->base.bw_margin_low_mhz, readin);
		break;
	case AD6676_ATTR_MRGN_H:
		update(pdata->base.bw_margin_high_mhz, readin);
		break;
	case AD6676_ATTR_MRGN_IF:
		update(pdata->base.bw_margin_if_mhz, readin);
		break;
	case AD6676_ATTR_SHUF_TH:
		update(pdata->shuffler.shuffle_thresh, readin);
		if (update) {
			ad6676_shuffle_setup(conv, &pdata->shuffler);
			update = false;
		}
		break;
	default:
		ret = -EINVAL;
	}

	if (update)
		ret = ad6676_update(conv, pdata);

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad6676_extinfo_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;

	int val;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)private) {
	case AD6676_ATTR_FADC:
		val = pdata->base.f_adc_hz;
		break;
	case AD6676_ATTR_FIF:
		val = pdata->base.f_if_hz;
		break;
	case AD6676_ATTR_BW:
		val = pdata->base.bw_hz;
		break;
	case AD6676_ATTR_MRGN_L:
		val = pdata->base.bw_margin_low_mhz;
		break;
	case AD6676_ATTR_MRGN_H:
		val = pdata->base.bw_margin_high_mhz;
		break;
	case AD6676_ATTR_MRGN_IF:
		val = pdata->base.bw_margin_if_mhz;
		mutex_unlock(&indio_dev->mlock);
		return sprintf(buf, "%d\n", val);
	case AD6676_ATTR_SHUF_TH:
		val = pdata->shuffler.shuffle_thresh;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : sprintf(buf, "%u\n", val);
}

#define AD6676_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad6676_extinfo_read, \
	.write = ad6676_extinfo_write, \
	.private = _ident, \
	.shared = IIO_SHARED_BY_TYPE, \
	}

static int ad6676_set_shuffler_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, u32 mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;

	pdata->shuffler.shuffle_ctrl = mode;

	return ad6676_shuffle_setup(conv, &pdata->shuffler);;
}

static int ad6676_get_shuffler_mode(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);

	return phy->pdata->shuffler.shuffle_ctrl;
}

static const char * const shuffler_modes[] =
 	{"disable", "fadc", "fadc/2", "fadc/3", "fadc/4"};

static const struct iio_enum ad6676_shuffler_modes_available = {
	.items = shuffler_modes,
	.num_items = ARRAY_SIZE(shuffler_modes),
	.get = ad6676_get_shuffler_mode,
	.set = ad6676_set_shuffler_mode,

};

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	AD6676_EXT_INFO("adc_frequency", AD6676_ATTR_FADC),
	AD6676_EXT_INFO("intermediate_frequency", AD6676_ATTR_FIF),
	AD6676_EXT_INFO("bandwidth", AD6676_ATTR_BW),
	AD6676_EXT_INFO("bw_margin_low", AD6676_ATTR_MRGN_L),
	AD6676_EXT_INFO("bw_margin_high", AD6676_ATTR_MRGN_H),
	AD6676_EXT_INFO("bw_margin_if", AD6676_ATTR_MRGN_IF),
	AD6676_EXT_INFO("shuffler_thresh", AD6676_ATTR_SHUF_TH),
	IIO_ENUM_AVAILABLE("shuffler_control",
			&ad6676_shuffler_modes_available),
	IIO_ENUM("shuffler_control", IIO_SHARED_BY_TYPE,
			&ad6676_shuffler_modes_available),
	{
	 .name = "test_mode",
	 .read = axiadc_testmode_read,
	 .write = axiadc_testmode_write,
	 .shared = IIO_SHARED_BY_TYPE,
	 },
	{
	 .name = "test_mode_available",
	 .read = ad6676_testmode_mode_available,
	 .shared = IIO_SHARED_BY_TYPE,
	 },
	{},
};

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	BIT(IIO_CHAN_INFO_HARDWAREGAIN) |\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = 0,					\
	  },								\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD6676] = {
		       .name = "AD6676",
		       .max_rate = 250000000UL,
		       .max_testmode = TESTGENMODE_JTSPAT,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 'S'),
		       .channel[1] = AIM_CHAN_NOCALIB(1, 1, 16, 'S'),
		       },
};


static int ad6676_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		*val = pdata->base.scale / 64;
		*val2 = ((pdata->base.scale % 64) * 1000000) / 64;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = pdata->base.f_adc_hz / pdata->base.decimation;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
			*val = -1 * pdata->base.attenuation;
			*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO_DB;
	}
	return -EINVAL;
}

static int ad6676_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad6676_phy *phy = conv_to_phy(conv);
	struct ad6676_platform_data *pdata = phy->pdata;
	unsigned tmp;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		tmp = val * 1000000 + val2;
		pdata->base.scale = clamp_t(u8, DIV_ROUND_CLOSEST(tmp * 64, 1000000), 16, 64);
		return ad6676_update(conv, pdata);
	case IIO_CHAN_INFO_HARDWAREGAIN:
			pdata->base.attenuation = clamp(-1 * val, 0, 27);
			ad6676_spi_write(conv->spi, AD6676_ATTEN_VALUE_PIN0,
					 pdata->base.attenuation);
			ad6676_spi_write(conv->spi, AD6676_ATTEN_VALUE_PIN1,
					 pdata->base.attenuation);
			break;
	case IIO_CHAN_INFO_SAMP_FREQ:

		return -EINVAL;
	default:
		return -EINVAL;
	}

	return 0;
}

static IIO_CONST_ATTR(in_voltage_shuffler_thresh_available, "0 1 2 3 4 5 6 7 8");

static struct attribute *ad6676_attributes[] = {
	&iio_const_attr_in_voltage_shuffler_thresh_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad6676_attribute_group = {
	.attrs = ad6676_attributes,
};

static int ad6676_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_IQCOR_ENB | ADI_ENABLE);
	}

	return 0;
}

struct gpio_board_cfg {
	const char *gpio_name;
	unsigned value;
};

static struct gpio_board_cfg board_cfg[] = {
	{"oen", 0},
	{"sela", 0},
	{"selb", 1},
	{"s0", 0},
	{"s1", 1},
};

static int ad6676_gpio_config(struct spi_device *spi)
{
	struct gpio_desc	 *gpio;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(board_cfg); i++) {
		gpio = devm_gpiod_get(&spi->dev, board_cfg[i].gpio_name);
		if (!IS_ERR(gpio)) {
			ret = gpiod_direction_output(gpio, board_cfg[i].value);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static u32 ad6676_of_property_read_u32(struct device_node *np,
					const char *propname, u32 default_val)
{
	u32 tmp = default_val;
	of_property_read_u32(np, propname, &tmp);
	return tmp;
}

static struct ad6676_platform_data *ad6676_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad6676_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	pdata->spi3wire = of_property_read_bool(np, "adi,spi-3wire-enable");

	pdata->base.f_adc_hz = ad6676_of_property_read_u32(np, "adi,adc-frequency-hz", 320000000UL);
	pdata->base.fadc_fixed = of_property_read_bool(np, "adi,adc-frequency-fixed-enable");
	pdata->base.f_if_hz = ad6676_of_property_read_u32(np, "adi,intermediate-frequency-hz", 250000000UL);

	 /* min/max depend on Lext see Lext Selection in the datasheet
	  * Maximum External Inductor Value as a Function of IF Frequency
	  */
	pdata->base.f_if_min_hz = ad6676_of_property_read_u32(np, "adi,intermediate-frequency-min-hz", MIN_FIF);
	pdata->base.f_if_max_hz = ad6676_of_property_read_u32(np, "adi,intermediate-frequency-max-hz", MAX_FIF);

	pdata->base.bw_hz = ad6676_of_property_read_u32(np, "adi,bandwidth-hz", 75000000UL);
	pdata->base.bw_margin_low_mhz = ad6676_of_property_read_u32(np, "adi,bandwidth-margin-low-mhz", 5);
	pdata->base.bw_margin_high_mhz = ad6676_of_property_read_u32(np, "adi,bandwidth-margin-high-mhz", 5);
	pdata->base.bw_margin_if_mhz = ad6676_of_property_read_u32(np, "adi,bandwidth-margin-if-mhz", 0);
	pdata->base.decimation = ad6676_of_property_read_u32(np, "adi,decimation", 16);
	pdata->base.ext_l = ad6676_of_property_read_u32(np, "adi,external-inductance-l-nh", 19);
	pdata->base.use_extclk = of_property_read_bool(np, "adi,use-external-clk-enable");

	pdata->jesd.scrambling_en = of_property_read_bool(np, "adi,jesd-scrambling-enable");
	pdata->jesd.lvds_syncb = of_property_read_bool(np, "adi,jesd-use-lvds-syncb-enable");
	pdata->jesd.sysref_pd = of_property_read_bool(np, "adi,jesd-powerdown-sysref-enable");
	pdata->jesd.l = ad6676_of_property_read_u32(np, "adi,jesd-l-lanes", 2);
	pdata->jesd.f = ad6676_of_property_read_u32(np, "adi,jesd-f-frames-per-multiframe", 16);

	pdata->base.scale = ad6676_of_property_read_u32(np, "adi,idac1-fullscale-adjust", 64);

// 	pdata->agc.dec_peak_thresh_a = ad6676_of_property_read_u32(np, "adi,", 0);
// 	pdata->agc.dec_peak_thresh_b = ad6676_of_property_read_u32(np, "adi,", 0);
// 	pdata->agc.dec_low_thresh = ad6676_of_property_read_u32(np, "adi,", 0);
// 	pdata->agc.dwell_mantissa = ad6676_of_property_read_u32(np, "adi,", 0);
// 	pdata->agc.dwell_exp = ad6676_of_property_read_u32(np, "adi,", 0);

	pdata->shuffler.shuffle_ctrl = ad6676_of_property_read_u32(np, "adi,shuffler-control", 1);
	pdata->shuffler.shuffle_thresh = ad6676_of_property_read_u32(np, "adi,shuffler-thresh", 5);

	return pdata;
}
#else
static
struct ad6676_platform_data *ad6676_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ad6676_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad6676_phy *phy;
	struct clk *clk = NULL;
	int ret, clk_enabled = 0;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	spi_set_drvdata(spi, conv);
	conv->phy = phy;
	conv->clk = clk;
	conv->spi = spi;

	if (spi->dev.of_node)
		phy->pdata = ad6676_parse_dt(&spi->dev);
	else
		phy->pdata = spi->dev.platform_data;

	if (!phy->pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	ad6676_gpio_config(spi);

	/* RESET here */
	conv->pwrdown_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (!IS_ERR(conv->pwrdown_gpio)) {
		ret = gpiod_direction_output(conv->pwrdown_gpio, 1);
	}

	mdelay(100);

	conv->id = ad6676_spi_read(spi, AD6676_CHIP_ID0);

	if (conv->id != spi_get_device_id(spi)->driver_data) {

		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n",
			conv->id);
		ret = -ENODEV;
		goto out;
	}

	switch (conv->id) {
	case CHIPID0_AD6676:

		ret = ad6676_setup(conv);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize\n");
			ret = -EIO;
			goto out;
		}

		conv->chip_info = &axiadc_chip_info_tbl[ID_AD6676];
		conv->adc_output_mode = DP_CTRL_TWOS_COMPLEMENT;
		ret = ad6676_outputmode_set(conv, conv->adc_output_mode);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		ret = -ENODEV;
		goto out;
	}

	if (ret < 0)
		goto out;

	conv->write = ad6676_spi_write;
	conv->read = ad6676_spi_read;
	conv->write_raw = ad6676_write_raw;
	conv->read_raw = ad6676_read_raw;
	conv->post_setup = ad6676_post_setup;
	conv->testmode_set = ad6676_test_and_outputmode_set;
	conv->attrs = &ad6676_attribute_group;

	return 0;

out:
	if (clk_enabled)
		clk_disable_unprepare(clk);

	return ret;
}

static int ad6676_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	clk_disable_unprepare(conv->clk);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad6676_id[] = {
	{"ad6676", CHIPID0_AD6676},
	{}
};

MODULE_DEVICE_TABLE(spi, ad6676_id);

static struct spi_driver ad6676_driver = {
	.driver = {
		   .name = "ad6676",
		   .owner = THIS_MODULE,
		   },
	.probe = ad6676_probe,
	.remove = ad6676_remove,
	.id_table = ad6676_id,
};

module_spi_driver(ad6676_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD6676 ADC");
MODULE_LICENSE("GPL v2");
