// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9081 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2019-2020 Analog Devices Inc.
 */
//#define DEBUG
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "ad9081/adi_ad9081.h"
#include "ad9081/adi_ad9081_hal.h"
#include "cf_axi_adc.h"

//#include <dt-bindings/iio/adc/adi,ad9081.h>

#define CHIPID_AD9081 0x9081
#define CHIPID_MASK 0xFFFF
#define ID_DUAL BIT(31)

#define MAX_NUM_MAIN_DATAPATHS 4
#define MAX_NUM_CHANNELIZER 8

enum { ID_AD9081,
};

enum {	CDDC_NCO_FREQ,
	FDDC_NCO_FREQ,
	CDDC_NCO_PHASE,
	FDDC_NCO_PHASE,
	FDDC_NCO_GAIN,
	DAC_MAIN_TEST_TONE_EN,
	DAC_CHAN_TEST_TONE_EN,
	DAC_MAIN_TEST_TONE_OFFSET,
	DAC_CHAN_TEST_TONE_OFFSET,
	TRX_CONVERTER_RATE,
	TRX_ENABLE,
	CDDC_FFH_HOPF_SET,
};


enum {
	AD9081_LOOPBACK_MODE,
	AD9081_ADC_CLK_PWDN,
	AD9081_MCS,
	AD9081_DAC_FFH_FREQ_SET,
	AD9081_DAC_FFH_INDEX_SET,
	AD9081_DAC_FFH_MODE_SET,
};

struct ad9081_jesd_link {
	bool is_jrx;
	adi_cms_jesd_param_t jesd_param;
	u8 logiclane_mapping[8];
	u8 link_converter_select[16];
	u64 lane_rate;
};

enum ad9081_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	NUM_AD9081_CLKS,
};

struct ad9081_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct ad9081_phy *phy;
	unsigned long rate;
	enum ad9081_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct ad9081_clock, hw)

struct dac_settings_cache {
	u16 chan_gain[MAX_NUM_CHANNELIZER];
	u16 main_test_tone_offset[MAX_NUM_CHANNELIZER];
	u16 chan_test_tone_offset[MAX_NUM_CHANNELIZER];
	s32 main_phase[MAX_NUM_CHANNELIZER];
	s32 chan_phase[MAX_NUM_CHANNELIZER];
	u8 main_test_tone_en[MAX_NUM_CHANNELIZER];
	u8 chan_test_tone_en[MAX_NUM_CHANNELIZER];
	u8 enable[MAX_NUM_CHANNELIZER];
};

struct device_settings_cache {
	u8 loopback_mode;
	u8 adc_clk_pwdn;
};

struct ad9081_phy {
	struct spi_device *spi;
	adi_ad9081_device_t ad9081;
	struct axiadc_chip_info chip_info;
	struct clk *dev_clk;
	struct clk *fmc_clk;
	struct clk *sysref_dev_clk;
	struct clk *sysref_fmc_clk;
	struct clk *jesd_rx_clk;
	struct clk *jesd_tx_clk;

	struct gpio_desc *rx1_en_gpio;
	struct gpio_desc *rx2_en_gpio;
	struct gpio_desc *tx1_en_gpio;
	struct gpio_desc *tx2_en_gpio;

	struct clk *clks[NUM_AD9081_CLKS];
	struct ad9081_clock clk_priv[NUM_AD9081_CLKS];
	struct clk_onecell_data clk_data;

	struct delayed_work dwork;

	u32 mcs_cached_val;

	u32 multidevice_instance_count;
	u32 lmfc_delay;
	bool config_sync_01_swapped;

	struct device_settings_cache device_cache;

	u64 dac_frequency_hz;
	s64 tx_main_shift[MAX_NUM_MAIN_DATAPATHS];
	s64 tx_chan_shift[MAX_NUM_CHANNELIZER];
	u32 tx_main_interp;
	u32 tx_chan_interp;
	u8 tx_dac_chan_xbar[MAX_NUM_MAIN_DATAPATHS];
	u8 tx_main_ffh_select[MAX_NUM_MAIN_DATAPATHS];

	u8 ffh_hopf_index;
	u8 ffh_hopf_mode;
	u32 ffh_hopf_vals[32];


	struct dac_settings_cache dac_cache;

	struct ad9081_jesd_link jesd_tx_link;

	u32 adc_main_decimation[MAX_NUM_MAIN_DATAPATHS];
	u32 adc_chan_decimation[MAX_NUM_CHANNELIZER];
	u64 adc_frequency_hz;
	s64 rx_fddc_shift[MAX_NUM_CHANNELIZER];
	s64 rx_cddc_shift[MAX_NUM_MAIN_DATAPATHS];
	s32 rx_fddc_phase[MAX_NUM_MAIN_DATAPATHS];
	s32 rx_cddc_phase[MAX_NUM_CHANNELIZER];

	u32 rx_nyquist_zone;
	u8 rx_cddc_c2r[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_fddc_c2r[MAX_NUM_CHANNELIZER];
	u8 rx_fddc_dcm[MAX_NUM_CHANNELIZER];
	u8 rx_cddc_dcm[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_fddc_select;
	u8 rx_cddc_select;

	struct ad9081_jesd_link jesd_rx_link[2];
};

static int ad9081_nco_sync_master_slave(struct ad9081_phy *phy, bool master)
{
	int ret;

	ret = adi_ad9081_dac_nco_master_slave_gpio_set(&phy->ad9081, 0, master);
	if (ret < 0)
		return ret;
	/* source  0: sysref, 1: lmfc rising edge, 2: lmfc falling edge */
	ret = adi_ad9081_dac_nco_master_slave_trigger_source_set(
		&phy->ad9081, 1); /* REG 0xCC */
	if (ret < 0)
		return ret;

	ret = adi_ad9081_dac_nco_master_slave_mode_set(&phy->ad9081,
		master ? 1 : 2); /* REG 0xCC */

	adi_ad9081_dac_nco_sync_reset_via_sysref_set(&phy->ad9081, 1);

	if (master)
		return adi_ad9081_dac_nco_master_slave_trigger_set(
			&phy->ad9081); /* REG 0xBC */

	return ret;
}

unsigned long ad9081_calc_lanerate(struct ad9081_jesd_link *link,
				   u64 converter_rate, u32 intp_decim)
{
	u64 rate, encoding_n, encoding_d;

	if (!intp_decim || !link->jesd_param.jesd_l) {
		pr_err("%s: Invalid pramater", __func__);
		return 0;
	}

	switch (link->jesd_param.jesd_jesdv) {
	case 2:
		encoding_n = 66; /* JESD 204C */
		encoding_d = 64;
		break;
	default:
		encoding_n = 10; /* JESD 204AB */
		encoding_d = 8;
		break;
	}

	rate = link->jesd_param.jesd_m * link->jesd_param.jesd_np * encoding_n *
	converter_rate;
	do_div(rate, link->jesd_param.jesd_l * encoding_d * intp_decim * 1000);

	return rate;
}

int32_t ad9081_log_write(void *user_data, int32_t log_type, const char *message,
			 va_list argp)
{
	struct axiadc_converter *conv = user_data;
	char logMessage[160];

	vsnprintf(logMessage, sizeof(logMessage), message, argp);

	switch (log_type) {
	case ADI_CMS_LOG_NONE:
		break;
	case ADI_CMS_LOG_MSG:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_WARN:
		dev_warn(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ERR:
		dev_err(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_SPI:
		break;
	case ADI_CMS_LOG_API:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ALL:
		printk(logMessage);
		break;
	}

	return 0;
}

static int ad9081_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int ad9081_spi_xfer(void *user_data, uint8_t *wbuf, uint8_t *rbuf,
			   uint32_t len)
{
	struct axiadc_converter *conv = user_data;
	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len & 0xFFFF,
	};

	return spi_sync_transfer(conv->spi, &t, 1);
}

int32_t ad9081_reset_pin_ctrl(void *user_data, uint8_t enable)
{
	struct axiadc_converter *conv = user_data;

	if (conv->reset_gpio)
		return gpiod_direction_output(conv->reset_gpio, enable);

	return 0;
}

static int ad9081_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 val;
	int ret;

	if (readval == NULL)
		return adi_ad9081_hal_reg_set(&phy->ad9081, reg, writeval);

	ret = adi_ad9081_hal_reg_get(&phy->ad9081, reg, &val);
	if (ret < 0)
		return ret;
	*readval = val;

	return 0;
}

#define AD9081_MAX_CLK_NAME 79

static char *ad9081_clk_set_dev_name(struct ad9081_phy *phy, char *dest,
				     const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strlcpy(dest, dev_name(&phy->spi->dev),
			      AD9081_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, AD9081_MAX_CLK_NAME - len);
}

static unsigned long ad9081_bb_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int ad9081_bb_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long ad9081_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *prate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = ad9081_bb_round_rate,
	.set_rate = ad9081_bb_set_rate,
	.recalc_rate = ad9081_bb_recalc_rate,
};

static int ad9081_clk_register(struct ad9081_phy *phy, const char *name,
			       const char *parent_name,
			       const char *parent_name2, unsigned long flags,
			       u32 source)
{
	struct ad9081_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[AD9081_MAX_CLK_NAME + 1],
		p_name[2][AD9081_MAX_CLK_NAME + 1];
	const char *_parent_name[2];

	/* struct ad9081_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] = ad9081_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] = ad9081_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = ad9081_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
	case RX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		break;
	case TX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		break;
	default:
			return -EINVAL;
	}

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

#if 0
static unsigned int ad9081_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return AD9081_TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return AD9081_TESTMODE_PN23_SEQ;
	default:
		return AD9081_TESTMODE_OFF;
	}
}

static int ad9081_testmode_set(struct iio_dev *indio_dev, unsigned int chan,
	unsigned int mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret;

	ad9081_adc_set_channel_select(&phy->ad9081, BIT(chan & 1));
	/* FIXME: Add support for DDC testmodes */
	ret = ad9081_spi_write(conv->spi, AD9081_REG_TEST_MODE, mode);
	conv->testmode[chan] = mode;
	ad9081_adc_set_channel_select(&phy->ad9081, AD9081_ADC_CH_ALL);

	return ret;
}

static int ad9081_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9081_pnsel_to_testmode(sel);
	unsigned int output_mode;
	int ret;

	output_mode = conv->adc_output_mode;
	if (mode != AD9081_TESTMODE_OFF)
		output_mode &= ~AD9081_OUTPUT_MODE_TWOS_COMPLEMENT;

	ret = ad9081_spi_write(conv->spi, AD9081_REG_OUTPUT_MODE, output_mode);
	if (ret < 0)
		return ret;

	return ad9081_testmode_set(indio_dev, chan, mode);
}

static int ad9081_read_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	u16 low, high;

	mutex_lock(&indio_dev->mlock);
	low = (ad9081_spi_read(spi, AD9081_FD_LT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_LT_LSB_REG);
	high = (ad9081_spi_read(spi, AD9081_FD_UT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_UT_LSB_REG);
	mutex_unlock(&indio_dev->mlock);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		*val = high - low;
		break;
	case IIO_EV_INFO_VALUE:
		*val = high;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ad9081_read_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad9081_spi_read(spi, AD9081_CHIP_PIN_CTRL1_REG);
	if (ret < 0)
		return ret;
	else
		return !(ret & AD9081_CHIP_PIN_CTRL_MASK(chan->channel));
}

static int ad9081_write_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret = 0;
	int low, high;

	mutex_lock(&indio_dev->mlock);
	high = (ad9081_spi_read(spi, AD9081_FD_UT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_UT_LSB_REG);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		if (val < 0) {
			ret = -EINVAL;
			goto unlock;
		}

		low = high - val;
		break;

	case IIO_EV_INFO_VALUE:
		if (val > 0x7FF) {
			ret = -EINVAL;
			goto unlock;
		}

		ad9081_spi_write(spi, AD9081_FD_UT_MSB_REG, val >> 8);
		ad9081_spi_write(spi, AD9081_FD_UT_LSB_REG, val & 0xFF);

		/* Calculate the new lower threshold limit */
		low = (ad9081_spi_read(spi, AD9081_FD_LT_MSB_REG) << 8) |
			ad9081_spi_read(spi, AD9081_FD_LT_LSB_REG);
		low = val - high + low;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	if (low < 0)
		low = 0;

	ad9081_spi_write(spi, AD9081_FD_LT_MSB_REG, low >> 8);
	ad9081_spi_write(spi, AD9081_FD_LT_LSB_REG, low & 0xFF);

unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad9081_write_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	mutex_lock(&indio_dev->mlock);

	ret = ad9081_spi_read(spi, AD9081_CHIP_PIN_CTRL1_REG);
	if (ret < 0)
		goto err_unlock;

	if (state)
		ret &= ~AD9081_CHIP_PIN_CTRL_MASK(chan->channel);
	else
		ret |= AD9081_CHIP_PIN_CTRL_MASK(chan->channel);

	ret = ad9081_spi_write(spi, AD9081_CHIP_PIN_CTRL1_REG, ret);
err_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}
#endif

static irqreturn_t ad9081_event_handler(struct axiadc_converter *conv,
	unsigned int chn)
{
	u64 event = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, chn,
			IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
	s64 timestamp = iio_get_time_ns(conv->indio_dev);

	if (conv->indio_dev)
		iio_push_event(conv->indio_dev, event, timestamp);

	return IRQ_HANDLED;
}

static irqreturn_t ad9081_fdA_handler(int irq, void *private)
{
	return ad9081_event_handler(private, 0);
}

static irqreturn_t ad9081_fdB_handler(int irq, void *private)
{
	return ad9081_event_handler(private, 1);
}

static int ad9081_testmode_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->testmode[chan->channel];
}

static int ad9081_testmode_write(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = adi_ad9081_adc_test_mode_config_set(&phy->ad9081, item, item,
						  AD9081_LINK_ALL);
	if (!ret)
		conv->testmode[chan->channel] = item;
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const char *const ad9081_adc_testmodes[] = {
	[AD9081_TMODE_OFF] = "off",
	[AD9081_TMODE_MIDSCALE] = "midscale_short",
	[AD9081_TMODE_POS_FULL] = "pos_fullscale",
	[AD9081_TMODE_NEG_FULL] = "neg_fullscale",
	[AD9081_TMODE_ALT_CHECKER] = "checkerboard",
	[AD9081_TMODE_PN9] = "pn9",
	[AD9081_TMODE_PN23] = "pn32",
	[AD9081_TMODE_1_0_TOGG] = "one_zero_toggle",
	[AD9081_TMODE_USER_PAT] = "user",
	[AD9081_TMODE_PN7] = "pn7",
	[AD9081_TMODE_PN15] = "pn15",
	[AD9081_TMODE_PN31] = "pn31",
	[AD9081_TMODE_RAMP] = "ramp",
};

static const char *const ad9081_jesd_testmodes[] = {
	[AD9081_JESD_TX_TEST_MODE_DISABLED] = "off",
	[AD9081_JESD_TX_TEST_MODE_CHECKER_BOARD] = "checkerboard",
	[AD9081_JESD_TX_TEST_MODE_WORD_TOGGLE] = "word_toggle",
	[AD9081_JESD_TX_TEST_MODE_PN31] = "pn31",
	[AD9081_JESD_TX_TEST_MODE_PN15] = "pn15",
	[AD9081_JESD_TX_TEST_MODE_PN7] = "pn7",
	[AD9081_JESD_TX_TEST_MODE_RAMP] = "ramp",
	[AD9081_JESD_TX_TEST_MODE_USER_REPEAT] = "user_repeat",
	[AD9081_JESD_TX_TEST_MODE_USER_SINGLE] = "user_single",
};

static const struct iio_enum ad9081_testmode_enum = {
	.items = ad9081_adc_testmodes,
	.num_items = ARRAY_SIZE(ad9081_adc_testmodes),
	.set = ad9081_testmode_write,
	.get = ad9081_testmode_read,
};

int ad9081_iio_val_to_str(char *buf, u32 max, int val)
{
	int vals[2];

	vals[0] = val;
	vals[1] = max;

	return iio_format_value(buf, IIO_VAL_FRACTIONAL, 2, vals);
}

int ad9081_iio_str_to_val(const char *str, int min, int max, int *val)
{
	int ret, integer, fract;

	ret = iio_str_to_fixpoint(str, 100000, &integer, &fract);

	*val = DIV_ROUND_CLOSEST(
		max * (integer * 1000 + DIV_ROUND_CLOSEST(fract, 1000)), 1000);

	*val = clamp(*val, min, max);

	return ret;
}

static ssize_t ad9081_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	long long val;
	u8 cddc, fddc_num;
	int i, ret = -EINVAL;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case CDDC_NCO_FREQ:
		if (chan->output) {
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			     i++) {
				if (phy->tx_dac_chan_xbar[i] &
				    BIT(chan->channel)) {
					val = phy->tx_main_shift[i];
					ret = 0;
					break;
				}
			}
		} else {
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;
			adi_ad9081_adc_xbar_find_cddc(&phy->ad9081,
						      BIT(fddc_num), &cddc);
			val = phy->rx_cddc_shift[ilog2(cddc)];
			ret = 0;
		}
		break;
	case FDDC_NCO_FREQ:
		if (chan->output) {
			val = phy->tx_chan_shift[chan->channel];
			ret = 0;
		} else {
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;
			val = phy->rx_fddc_shift[fddc_num];
			ret = 0;
		}
		break;
	case CDDC_NCO_PHASE:
		if (chan->output) {
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			i++) {
				if (phy->tx_dac_chan_xbar[i] &
				BIT(chan->channel)) {
					val = phy->dac_cache.main_phase[i];
					ret = 0;
					break;
				}
			}
		} else {
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;
			adi_ad9081_adc_xbar_find_cddc(&phy->ad9081,
						BIT(fddc_num), &cddc);
			val = phy->rx_cddc_phase[ilog2(cddc)];
			ret = 0;
		}
		break;
	case FDDC_NCO_PHASE:
		if (chan->output) {
			val = phy->dac_cache.chan_phase[chan->channel];
			ret = 0;
		} else {
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;
			val = phy->rx_fddc_phase[fddc_num];
			ret = 0;
		}
		break;
	case FDDC_NCO_GAIN:
		val = phy->dac_cache.chan_gain[chan->channel];
		mutex_unlock(&indio_dev->mlock);
		return ad9081_iio_val_to_str(buf, 0xFFF, val);

	case DAC_MAIN_TEST_TONE_EN:
		val = phy->dac_cache.main_test_tone_en[chan->channel];
		ret = 0;
		break;
	case DAC_CHAN_TEST_TONE_EN:
		val = phy->dac_cache.chan_test_tone_en[chan->channel];
		ret = 0;
		break;
	case DAC_MAIN_TEST_TONE_OFFSET:
		val = phy->dac_cache.main_test_tone_offset[chan->channel];
		mutex_unlock(&indio_dev->mlock);
		return ad9081_iio_val_to_str(buf, 0x7FFF, val);
	case DAC_CHAN_TEST_TONE_OFFSET:
		val = phy->dac_cache.chan_test_tone_offset[chan->channel];
		mutex_unlock(&indio_dev->mlock);
		return ad9081_iio_val_to_str(buf, 0x7FFF, val);
	case TRX_CONVERTER_RATE:
		if (chan->output)
			val = phy->ad9081.dev_info.dac_freq_hz;
		else
			val = phy->ad9081.dev_info.adc_freq_hz;

		ret = 0;
		break;
	case CDDC_FFH_HOPF_SET:
		if (chan->output) {
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			     i++) {
				if (phy->tx_dac_chan_xbar[i] &
				    BIT(chan->channel)) {
					val = phy->tx_main_ffh_select[i];
					ret = 0;
					break;
				}
			}
		}
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	if (ret == 0)
		ret = sprintf(buf, "%lld\n", val);

	return ret;
}

u8 ad9081_get_main_dac_mask(struct ad9081_phy *phy, u32 channel)
{
	u8 mask = 0;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar); i++)
		if (phy->tx_dac_chan_xbar[i] & BIT(channel))
			mask |= BIT(i);

	return mask;
}

u8 ad9081_get_main_adc_mask(struct ad9081_phy *phy, int scan_index)
{
	u8 mask = 0;
	u32 fddc_num;

	if (scan_index < 0)
		return 0;

	fddc_num = phy->jesd_rx_link[0].link_converter_select[scan_index] / 2;
	adi_ad9081_adc_xbar_find_cddc(&phy->ad9081, BIT(fddc_num), &mask);

	return mask;
}

static ssize_t ad9081_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	long long readin;
	bool enable;
	int i, ret, readin_32;
	u8 cddc, fddc_num, mask;
	s16 val16;
	s64 val64;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case CDDC_NCO_FREQ:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			/* set main nco */
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			     i++) {
				if (phy->tx_dac_chan_xbar[i] &
				    BIT(chan->channel)) {
					ret = adi_ad9081_dac_duc_nco_set(
						&phy->ad9081, BIT(i),
						AD9081_DAC_CH_NONE, readin);
					if (!ret)
						phy->tx_main_shift[i] = readin;
					else
						ret = -EFAULT;
				}
			}

		} else {
			cddc = ad9081_get_main_adc_mask(phy, chan->address);

			ret = adi_ad9081_adc_ddc_coarse_nco_set(&phy->ad9081,
								cddc, readin);
			if (!ret)
				phy->rx_cddc_shift[ilog2(cddc)] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_FREQ:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			ret = adi_ad9081_dac_duc_nco_set(&phy->ad9081,
							 AD9081_DAC_NONE,
							 BIT(chan->channel),
							 readin);
			if (!ret)
				phy->tx_chan_shift[chan->channel] = readin;
			else
				ret = -EFAULT;

		} else {
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;
			ret = adi_ad9081_adc_ddc_fine_nco_set(
				&phy->ad9081, BIT(fddc_num), readin);
			if (!ret)
				phy->rx_fddc_shift[fddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case CDDC_NCO_PHASE:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		readin = clamp_t(long long, readin, -180000, 180000);

		if (chan->output) {
			val16 = div_s64(readin * 32768, 180000LL);

			/* set main nco */
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			i++) {
				if (phy->tx_dac_chan_xbar[i] &
				BIT(chan->channel)) {
					ret = adi_ad9081_dac_duc_nco_phase_offset_set(
						&phy->ad9081, BIT(i), val16,
						AD9081_DAC_CH_NONE, 0);
					if (!ret)
						phy->dac_cache.main_phase[i] =
							readin;
					else
						ret = -EFAULT;
				}
			}

		} else {
			val64 = div_s64(readin * 14073748835533, 18000LL);
			cddc = ad9081_get_main_adc_mask(phy, chan->address);

			ret =  adi_ad9081_adc_ddc_coarse_nco_phase_offset_set(
				&phy->ad9081, cddc, val64);
			if (!ret)
				phy->rx_cddc_phase[ilog2(cddc)] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_PHASE:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		readin = clamp_t(long long, readin, -180000, 180000);

		if (chan->output) {

			val16 = div_s64(readin * 32768, 180000LL);
			ret = adi_ad9081_dac_duc_nco_phase_offset_set(
					&phy->ad9081,
					AD9081_DAC_NONE, 0,
					BIT(chan->channel),
					val16);
			if (!ret)
				phy->dac_cache.chan_phase[chan->channel] =
					readin;
			else
				ret = -EFAULT;

		} else {
			val64 = div_s64(readin * 14073748835533, 18000LL);
			fddc_num =
				phy->jesd_rx_link[0].link_converter_select[
					chan->address] / 2;

			ret = adi_ad9081_adc_ddc_fine_nco_phase_offset_set(
				&phy->ad9081, BIT(fddc_num), val64);
			if (!ret)
				phy->rx_fddc_phase[fddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_GAIN:
		ret = ad9081_iio_str_to_val(buf, 0, 0xFFF, &readin_32);
		if (ret)
			goto out;
		ret = adi_ad9081_dac_duc_nco_gain_set(
			&phy->ad9081, BIT(chan->channel), readin_32);
		if (!ret)
			phy->dac_cache.chan_gain[chan->channel] = readin_32;
		break;
	case DAC_MAIN_TEST_TONE_EN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		mask = ad9081_get_main_dac_mask(phy, chan->channel);
		ret = adi_ad9081_dac_duc_main_dc_test_tone_en_set(&phy->ad9081,
								  mask, enable);
		if (!ret)
			phy->dac_cache.main_test_tone_en[chan->channel] =
				enable;
		break;
	case DAC_CHAN_TEST_TONE_EN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_dc_test_tone_en_set(
			&phy->ad9081, BIT(chan->channel), enable);
		if (!ret)
			phy->dac_cache.chan_test_tone_en[chan->channel] =
				enable;
		break;
	case DAC_MAIN_TEST_TONE_OFFSET:
		ret = ad9081_iio_str_to_val(buf, 0, 0x7FFF, &readin_32);
		if (ret)
			goto out;

		mask = ad9081_get_main_dac_mask(phy, chan->channel);
		ret = adi_ad9081_dac_duc_main_dc_test_tone_offset_set(
			&phy->ad9081, mask, readin_32);
		if (!ret)
			phy->dac_cache.main_test_tone_offset[chan->channel] =
				readin_32;

		break;
	case DAC_CHAN_TEST_TONE_OFFSET:
		ret = ad9081_iio_str_to_val(buf, 0, 0x7FFF, &readin_32);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_dc_test_tone_offset_set(
			&phy->ad9081, BIT(chan->channel), readin_32);
		if (!ret)
			phy->dac_cache.chan_test_tone_offset[chan->channel] =
				readin_32;
		break;

	case CDDC_FFH_HOPF_SET:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			/* set main nco */
			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
			     i++) {
				if (phy->tx_dac_chan_xbar[i] &
				    BIT(chan->channel)) {
					ret = adi_ad9081_dac_duc_main_nco_hopf_select_set(
							&phy->ad9081, BIT(i), readin);
					if (!ret)
						phy->tx_main_ffh_select[i] = readin;
					else
						ret = -EFAULT;

				}
			}

		}
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info rxadc_ext_info[] = {
	IIO_ENUM("test_mode", IIO_SHARED_BY_TYPE, &ad9081_testmode_enum),
	IIO_ENUM_AVAILABLE("test_mode", &ad9081_testmode_enum),
	{
		.name = "main_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_FREQ,
	},
	{
		.name = "channel_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_FREQ,
	},
	{
		.name = "main_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_PHASE,
	},
	{
		.name = "adc_frequency",
		.read = ad9081_ext_info_read,
		.shared = true,
		.private = TRX_CONVERTER_RATE,
	},
	{},
};

static struct iio_chan_spec_ext_info txdac_ext_info[] = {
	{
		.name = "main_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_FREQ,
	},
	{
		.name = "channel_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_FREQ,
	},
	{
		.name = "main_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_gain_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_GAIN,
	},
	{
		.name = "main_nco_test_tone_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_MAIN_TEST_TONE_EN,
	},
	{
		.name = "channel_nco_test_tone_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_CHAN_TEST_TONE_EN,
	},
	{
		.name = "main_nco_test_tone_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_MAIN_TEST_TONE_OFFSET,
	},
	{
		.name = "channel_nco_test_tone_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_CHAN_TEST_TONE_OFFSET,
	},
	{
		.name = "dac_frequency",
		.read = ad9081_ext_info_read,
		.shared = true,
		.private = TRX_CONVERTER_RATE,
	},
	{
		.name = "main_nco_ffh_select",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_FFH_HOPF_SET,
	},
	{},
};

static int ad9081_set_sample_rate(struct axiadc_converter *conv,
				  unsigned int sample_rate)
{
	return -ENOTSUPP;
}

static int ad9081_request_clks(struct axiadc_converter *conv)
{
	struct ad9081_phy *phy = conv->phy;
	int ret;

	phy->jesd_rx_clk = devm_clk_get(&conv->spi->dev, "jesd_rx_clk");
	phy->jesd_tx_clk = devm_clk_get(&conv->spi->dev, "jesd_tx_clk");

	phy->dev_clk = devm_clk_get(&conv->spi->dev, "dev_clk");
	if (IS_ERR(phy->dev_clk))
		return PTR_ERR(phy->dev_clk);

	phy->fmc_clk = devm_clk_get(&conv->spi->dev, "fmc_clk");
	if (IS_ERR(phy->fmc_clk))
		return PTR_ERR(phy->fmc_clk);

	phy->sysref_dev_clk = devm_clk_get(&conv->spi->dev, "sysref_dev_clk");
	phy->sysref_fmc_clk = devm_clk_get(&conv->spi->dev, "sysref_fmc_clk");

	ret = clk_prepare_enable(phy->fmc_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(phy->dev_clk);
	if (ret)
		return ret;

	return 0;
}

static int ad9081_main_decimation_to_val(u32 decim)
{
	switch (decim) {
	case 1:
		return AD9081_CDDC_DCM_1;
	case 2:
		return AD9081_CDDC_DCM_2;
	case 3:
		return AD9081_CDDC_DCM_3;
	case 4:
		return AD9081_CDDC_DCM_4;
	case 6:
		return AD9081_CDDC_DCM_6;
	case 8:
		return AD9081_CDDC_DCM_8;
	case 9:
		return AD9081_CDDC_DCM_9;
	case 12:
		return AD9081_CDDC_DCM_12;
	case 16:
		return AD9081_CDDC_DCM_16;
	case 18:
		return AD9081_CDDC_DCM_18;
	case 24:
		return AD9081_CDDC_DCM_24;
	case 36:
		return AD9081_CDDC_DCM_36;
	default:
		return -EINVAL;
	}
}

static int ad9081_chan_decimation_to_val(u32 decim)
{
	switch (decim) {
	case 1:
		return AD9081_FDDC_DCM_1;
	case 2:
		return AD9081_FDDC_DCM_2;
	case 3:
		return AD9081_FDDC_DCM_3;
	case 4:
		return AD9081_FDDC_DCM_4;
	case 6:
		return AD9081_FDDC_DCM_6;
	case 8:
		return AD9081_FDDC_DCM_8;
	case 12:
		return AD9081_FDDC_DCM_12;
	case 16:
		return AD9081_FDDC_DCM_16;
	case 24:
		return AD9081_FDDC_DCM_24;
	default:
		return -EINVAL;
	}
}

static const char *const ad9081_jtx_qbf_states[] = {
	"CGS", "ILA_M0R", "ILA_M0", "ILA_M1R", "ILA_M1C1", "ILA_M1C2",
	"ILA_M1C3", "ILA_M1", "ILA_M2R", "ILA_M2", "ILA_M3R", "ILA_M3",
	"ILA_BP", "DATA"
};

int ad9081_jesd_tx_link_status_print(struct ad9081_phy *phy)
{
	int ret, l;
	u16 stat;

	for (l = AD9081_LINK_0; l < AD9081_LINK_ALL; l++) {

		ret = adi_ad9081_jesd_tx_link_status_get(
			&phy->ad9081, l, &stat);

		if (ret)
			return -EFAULT;

		if (phy->jesd_rx_link[l - 1].jesd_param.jesd_jesdv == 2)
			dev_info(&phy->spi->dev,
				"JESD RX (JTX) Link%d PLL %s, PHASE %s, MODE %s\n",
				l,
				stat & BIT(5) ? "locked" : "unlocked",
				stat & BIT(6) ? "established" : "lost",
				stat & BIT(7) ? "invalid" : "valid");
		else
			dev_info(&phy->spi->dev,
				"JESD RX (JTX) Link%d in %s, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
				l, ad9081_jtx_qbf_states[stat & 0xF],
				stat & BIT(4) ? "deasserted" : "asserted",
				stat & BIT(5) ? "locked" : "unlocked",
				stat & BIT(6) ? "established" : "lost",
				stat & BIT(7) ? "invalid" : "valid");


		if (!phy->jesd_rx_link[l - 1].jesd_param.jesd_duallink)
			return 0;
	}

	return 0;
}

static const char *const ad9081_jrx_204c_states[] = {
	"Reset", "Undef", "Sync header alignment done",
	"Extended multiblock sync complete",
	"Extended multiblock alignment complete",
	"Undef", "Link is good", "Undef",
};

int ad9081_jesd_rx_link_status_print(struct ad9081_phy *phy)
{
	int ret, l;
	u16 stat;

	for (l = AD9081_LINK_0; l < AD9081_LINK_ALL; l++) {

		ret = adi_ad9081_jesd_rx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		if (phy->jesd_tx_link.jesd_param.jesd_jesdv == 2) {
			stat >>= 8;
			dev_info(&phy->spi->dev,
				"JESD TX (JRX) Link%d 204C status: %s (%d)\n",
				l, ad9081_jrx_204c_states[stat & 0x7], stat);

			if (stat == 6) /* FIXME DUAL Link */
				return 0xF;
			else
				return 0;
		} else {
			dev_info(&phy->spi->dev,
				"JESD TX (JRX) Link%d 0x%X lanes in DATA\n",
				l, stat & 0xF);
		}

		if (!phy->jesd_tx_link.jesd_param.jesd_duallink)
			return stat & 0xF;
	}

	return stat & 0xF;
}

static void ad9081_convert_link_converter_select(
	adi_ad9081_jtx_conv_sel_t *jesd_conv_sel, u8 *vals)
{
	jesd_conv_sel->virtual_converter0_index = *vals++;
	jesd_conv_sel->virtual_converter1_index = *vals++;
	jesd_conv_sel->virtual_converter2_index = *vals++;
	jesd_conv_sel->virtual_converter3_index = *vals++;
	jesd_conv_sel->virtual_converter4_index = *vals++;
	jesd_conv_sel->virtual_converter5_index = *vals++;
	jesd_conv_sel->virtual_converter6_index = *vals++;
	jesd_conv_sel->virtual_converter7_index = *vals++;
	jesd_conv_sel->virtual_converter8_index = *vals++;
	jesd_conv_sel->virtual_converter9_index = *vals++;
	jesd_conv_sel->virtual_convertera_index = *vals++;
	jesd_conv_sel->virtual_converterb_index = *vals++;
	jesd_conv_sel->virtual_converterc_index = *vals++;
	jesd_conv_sel->virtual_converterd_index = *vals++;
	jesd_conv_sel->virtual_convertere_index = *vals++;
	jesd_conv_sel->virtual_converterf_index = *vals++;
}

static int ad9081_setup(struct spi_device *spi, bool ad9234)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;
	struct clock_scale devclk_clkscale;
	u64 dev_frequency_hz, sample_rate, status64;
	unsigned long rx_lane_rate_kbps, tx_lane_rate_kbps;
	int ret, i, stat, retry = 5;
	adi_cms_jesd_param_t jesd_param[2];
	adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2];
	u8 txfe_pll_stat, dcm;

	of_clk_get_scale(spi->dev.of_node, "dev_clk", &devclk_clkscale);
	dev_frequency_hz = clk_get_rate_scaled(phy->dev_clk, &devclk_clkscale);

	tx_lane_rate_kbps = ad9081_calc_lanerate(&phy->jesd_tx_link,
				phy->dac_frequency_hz,
				phy->tx_main_interp * phy->tx_chan_interp);

	/* The 204c calibration routine requires the link to be up */
	if (!IS_ERR_OR_NULL(phy->jesd_tx_clk)) {
		ret = clk_set_rate(phy->jesd_tx_clk, tx_lane_rate_kbps);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
				tx_lane_rate_kbps, ret);
		}
		if (phy->jesd_tx_link.jesd_param.jesd_jesdv == 2) {
			ret = clk_prepare_enable(phy->jesd_tx_clk);
			if (ret < 0) {
				dev_err(&spi->dev,
					"Failed to enable JESD204 link: %d\n", ret);
				return ret;
			}
		}
	}

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_LMFC_DELAY_ADDR,
		BF_SYNC_LMFC_DELAY_SET_INFO,
		BF_SYNC_LMFC_DELAY_SET(phy->lmfc_delay));
	if (ret != 0)
		return ret;

	/* AC couple SYSREF */
	ret = adi_ad9081_jesd_rx_sysref_input_mode_set(&phy->ad9081, 0);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYSREF_AVERAGE_ADDR,
		BF_SYSREF_AVERAGE_INFO,
		BF_SYSREF_AVERAGE(7));
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_DEBUG0_ADDR,
		BF_AVRG_FLOW_EN_INFO, 1);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_device_clk_config_set(
		&phy->ad9081, phy->dac_frequency_hz, phy->adc_frequency_hz,
		dev_frequency_hz, dev_frequency_hz != phy->dac_frequency_hz);
	if (ret != 0)
		return ret;

	if (dev_frequency_hz != phy->dac_frequency_hz) {
		ret = adi_ad9081_device_clk_pll_lock_status_get(&phy->ad9081,
								&txfe_pll_stat);
		if (ret != 0)
			return ret;

		if (txfe_pll_stat != 3) {
			dev_err(&spi->dev, "CLK PLL Failed to Lock (Status: %d)",
				txfe_pll_stat);
			return -EFAULT;
		}
	}

	/* start txfe tx */
	ret = adi_ad9081_device_startup_tx(
		&phy->ad9081, phy->tx_main_interp, phy->tx_chan_interp,
		phy->tx_dac_chan_xbar, phy->tx_main_shift, phy->tx_chan_shift,
		&phy->jesd_tx_link.jesd_param);

	if (ret != 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(phy->adc_main_decimation); i++) {
		ret = ad9081_main_decimation_to_val(
			phy->adc_main_decimation[i]);
		if (ret >= 0)
			phy->rx_cddc_dcm[i] = ret;
	}

	for (i = 0; i < ARRAY_SIZE(phy->adc_chan_decimation); i++) {
		ret = ad9081_chan_decimation_to_val(
			phy->adc_chan_decimation[i]);
		if (ret >= 0)
			phy->rx_fddc_dcm[i] = ret;
	}

	/* FIXME - the API should change here */
	ad9081_convert_link_converter_select(&jesd_conv_sel[0],
		phy->jesd_rx_link[0].link_converter_select);
	ad9081_convert_link_converter_select(&jesd_conv_sel[1],
		phy->jesd_rx_link[1].link_converter_select);

	jesd_param[0] = phy->jesd_rx_link[0].jesd_param;
	jesd_param[1] = phy->jesd_rx_link[1].jesd_param;

	/* start txfe rx and set other settings for normal use cases */
	/* start txfe rx */
	ret = adi_ad9081_device_startup_rx(&phy->ad9081, phy->rx_cddc_select,
					   phy->rx_fddc_select,
					   phy->rx_cddc_shift,
					   phy->rx_fddc_shift, phy->rx_cddc_dcm,
					   phy->rx_fddc_dcm, phy->rx_cddc_c2r,
					   phy->rx_fddc_c2r, jesd_param,
					   jesd_conv_sel);
	if (ret != 0)
		return ret;

	/* setup txfe dac channel gain */
	ret = adi_ad9081_dac_duc_nco_gains_set(&phy->ad9081,
					       phy->dac_cache.chan_gain);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_jesd_rx_lanes_xbar_set(&phy->ad9081, AD9081_LINK_0,
			phy->jesd_tx_link.logiclane_mapping);
	if (ret != 0)
		return ret;

	if (phy->jesd_tx_link.jesd_param.jesd_duallink > 0) {
		ret = adi_ad9081_jesd_rx_lanes_xbar_set(
				&phy->ad9081, AD9081_LINK_1,
				phy->jesd_tx_link.logiclane_mapping);
		if (ret != 0)
			return ret;
	}

	ret = adi_ad9081_jesd_tx_lanes_xbar_set(&phy->ad9081, AD9081_LINK_0,
			phy->jesd_rx_link[0].logiclane_mapping);
	if (ret != 0)
		return ret;
	ret = adi_ad9081_jesd_tx_lids_cfg_set(&phy->ad9081, AD9081_LINK_0,
			phy->jesd_rx_link[0].logiclane_mapping);
	if (ret != 0)
		return ret;

	/* setup txfe jtx converter mapping */
	for (i = 0; i < ARRAY_SIZE(phy->jesd_rx_link[0].link_converter_select);
	     i++) {
		ret = adi_ad9081_jesd_tx_conv_sel_set(
			&phy->ad9081, AD9081_LINK_0, i,
			phy->jesd_rx_link[0].link_converter_select[i]);
		if (ret != 0)
			return ret;
	}
	if (phy->jesd_rx_link[0].jesd_param.jesd_duallink > 0) {
		for (i = 0;
		     i < ARRAY_SIZE(phy->jesd_rx_link[1].link_converter_select);
		     i++) {
			ret = adi_ad9081_jesd_tx_conv_sel_set(
				&phy->ad9081, AD9081_LINK_1, i,
				phy->jesd_rx_link[1].link_converter_select[i]);
			if (ret != 0)
				return ret;
		}
	}

	ret = adi_ad9081_adc_chip_dcm_ratio_get(&phy->ad9081,
						AD9081_LINK_0, &dcm);
	if (ret != 0 || !dcm)
		return ret;

	if (phy->config_sync_01_swapped) {
		adi_ad9081_jesd_rx_syncb_driver_powerdown_set(&phy->ad9081, 0);
		adi_ad9081_hal_reg_set(&phy->ad9081,
			REG_GENERAL_JRX_CTRL_ADDR, 0x80);
		adi_ad9081_jesd_tx_sync_mode_set(&phy->ad9081,
			AD9081_LINK_0, 1);
	}

	if (!IS_ERR_OR_NULL(phy->jesd_rx_clk)) {
		rx_lane_rate_kbps = ad9081_calc_lanerate(&phy->jesd_rx_link[0],
						phy->adc_frequency_hz,
						dcm);

		ret = clk_set_rate(phy->jesd_rx_clk, rx_lane_rate_kbps);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
			rx_lane_rate_kbps, ret);
		}
	}

	if ((phy->jesd_tx_link.jesd_param.jesd_jesdv == 2) &&
		(tx_lane_rate_kbps > 16230000UL)) {
		ret = adi_ad9081_jesd_rx_calibrate_204c(&phy->ad9081);
		if (ret < 0)
			return ret;
	}

	ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
		(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
		AD9081_LINK_ALL : AD9081_LINK_0, 1);
	if (ret != 0)
		return ret;

	if (!IS_ERR_OR_NULL(phy->jesd_rx_clk)) {
		msleep(10);
		ret = clk_prepare_enable(phy->jesd_rx_clk);
		if (ret < 0) {
			dev_err(&spi->dev,
				"Failed to enable JESD204 link: %d\n", ret);
			return ret;
		}
	}

	ret = adi_ad9081_jesd_tx_link_enable_set(&phy->ad9081,
		(phy->jesd_rx_link[0].jesd_param.jesd_duallink > 0) ?
		AD9081_LINK_ALL : AD9081_LINK_0, 1);
	if (ret != 0)
		return ret;

	if (!IS_ERR_OR_NULL(phy->jesd_tx_clk) &&
		(phy->jesd_tx_link.jesd_param.jesd_jesdv == 1)) {
		ret = clk_prepare_enable(phy->jesd_tx_clk);
		if (ret < 0) {
			dev_err(&spi->dev,
				"Failed to enable JESD204 link: %d\n", ret);
			return ret;
		}
	}

	/*
	 * 204c doesn't have a SYNC, so the link should come up.
	 * This needs to be revisited once we move this driver to the
	 * new JESD framework ...
	 */

	if (phy->jesd_tx_link.jesd_param.jesd_jesdv == 2 ||
		!IS_ERR_OR_NULL(phy->jesd_tx_clk)) {
		do {	/* temp workaround until API is fixed */
			mdelay(10);
			stat = ad9081_jesd_rx_link_status_print(phy);
			if (stat <= 0) {
				ret = adi_ad9081_jesd_rx_link_enable_set(
					&phy->ad9081,
					(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
					AD9081_LINK_ALL : AD9081_LINK_0, 0);
				if (ret != 0)
					return ret;

				if (!IS_ERR_OR_NULL(phy->jesd_tx_clk)) {
					clk_disable_unprepare(phy->jesd_tx_clk);

					mdelay(100);

					ret = clk_prepare_enable(phy->jesd_tx_clk);
					if (ret < 0) {
						dev_err(&spi->dev,
							"Failed to enable JESD204 link: %d\n",
							ret);
						return ret;
					}
				} else {
					mdelay(100);
				}

				ret = adi_ad9081_jesd_rx_link_enable_set(
					&phy->ad9081,
					(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
					AD9081_LINK_ALL : AD9081_LINK_0, 1);
				if (ret != 0)
					return ret;

				mdelay(100);
			}
		} while (stat <= 0 && retry--);
	}

	ad9081_jesd_tx_link_status_print(phy);

	adi_ad9081_dac_irqs_status_get(&phy->ad9081, &status64);
	dev_dbg(&spi->dev, "DAC IRQ status 0x%llX\n", status64);

	sample_rate = phy->adc_frequency_hz;
	do_div(sample_rate, dcm);

	clk_set_rate(phy->clks[RX_SAMPL_CLK], sample_rate);

	sample_rate = phy->dac_frequency_hz;
	do_div(sample_rate, phy->tx_main_interp * phy->tx_chan_interp);

	clk_set_rate(phy->clks[TX_SAMPL_CLK], sample_rate);

	ret = adi_ad9081_adc_nyquist_zone_set(&phy->ad9081,
		phy->rx_nyquist_zone);
	if (ret != 0)
		return ret;

	ret = ad9081_nco_sync_master_slave(phy,
		!IS_ERR_OR_NULL(phy->jesd_rx_clk));
	if (ret != 0)
		return ret;

	schedule_delayed_work(&phy->dwork, msecs_to_jiffies(1000));

	return 0;
}

static int ad9081_multichip_sync(struct ad9081_phy *phy, int step)
{
	int ret;

	dev_dbg(&phy->spi->dev, "%s:%d\n", __func__, step);

	switch (step & 0xFF) {
	case 0:
		/* enable txfe link */
		ret = adi_ad9081_jesd_tx_link_enable_set(&phy->ad9081,
			(phy->jesd_rx_link[0].jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 0);
		if (ret != 0)
			return ret;

		if (!IS_ERR_OR_NULL(phy->jesd_rx_clk))
			clk_disable_unprepare(phy->jesd_rx_clk);

		break;
	case 1:
		/* enable txfe link */
		ret = adi_ad9081_jesd_tx_link_enable_set(&phy->ad9081,
			(phy->jesd_rx_link[0].jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 1);
		if (ret != 0)
			return ret;

		if (!IS_ERR_OR_NULL(phy->jesd_rx_clk)) {
			ret = clk_prepare_enable(phy->jesd_rx_clk);
			if (ret < 0) {
				dev_err(&phy->spi->dev,
					"Failed to enable JESD204 link: %d\n",
					ret);
				return ret;
			}
		}
		ad9081_jesd_tx_link_status_print(phy);
		break;
	case 2:

		ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
			(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 0);
		if (ret != 0)
			return ret;

		if (!IS_ERR_OR_NULL(phy->jesd_tx_clk))
			clk_disable_unprepare(phy->jesd_tx_clk);

		break;
	case 3:

		if (!IS_ERR_OR_NULL(phy->jesd_tx_clk)) {
			ret = clk_prepare_enable(phy->jesd_tx_clk);
			if (ret < 0) {
				dev_err(&phy->spi->dev,
					"Failed to enable JESD204 link: %d\n",
					ret);
				return ret;
			}
		}

		ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
			(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 1);
		if (ret != 0)
			return ret;

		ad9081_jesd_rx_link_status_print(phy);
		break;
	case 4:
		ret = adi_ad9081_jesd_oneshot_sync(&phy->ad9081);
		break;
	case 5:
		ret = ad9081_nco_sync_master_slave(phy,
			!IS_ERR_OR_NULL(phy->jesd_rx_clk));
		if (ret != 0)
			return ret;
		break;
	case 6:
		ret = adi_ad9081_dac_nco_sync_set(&phy->ad9081, step >> 8);
		break;
	case 7:
		ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
			(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 0);
		if (ret != 0)
			return ret;
		mdelay(1);
		ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
			(phy->jesd_tx_link.jesd_param.jesd_duallink > 0) ?
			AD9081_LINK_ALL : AD9081_LINK_0, 1);
		ad9081_jesd_rx_link_status_print(phy);
		if (ret != 0)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9081_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan, int *val,
			   int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk =
			clk_get_rate_scaled(conv->clk, &conv->adc_clkscale);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output) {
			*val = phy->dac_cache.enable[chan->channel];
			return IIO_VAL_INT;
		}
	}
	return -EINVAL;
}

static int ad9081_write_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan, int val, int val2,
			    long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	unsigned long r_clk;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		if (conv->id == CHIPID_AD9081)
			return ad9081_set_sample_rate(conv, val);

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;
		break;
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output) {
			u8 mask;

			mask = ad9081_get_main_dac_mask(phy, chan->channel);

			if (mask) {
				ret = adi_ad9081_dac_tx_enable_set(&phy->ad9081,
								   mask, !!val);

				if (!ret)
					phy->dac_cache.enable[chan->channel] =
					!!val;
			}
		} else {
			/* FIXME: NO API? */

		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t ad9081_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	unsigned long res;
	uint64_t ftw;
	bool bres;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address & 0xFF) {
	case AD9081_LOOPBACK_MODE:
		ret = kstrtoul(buf, 0, &res);
		if (ret || res > 2) {
			ret = -EINVAL;
			break;
		}
		/* setup txfe loopback mode */
		adi_ad9081_jesd_loopback_mode_set(&phy->ad9081, res);
		phy->device_cache.loopback_mode = res;
		break;
	case AD9081_ADC_CLK_PWDN:
		ret = strtobool(buf, &bres);
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}

		adi_ad9081_adc_clk_enable_set(&phy->ad9081, !bres);
		phy->device_cache.adc_clk_pwdn = bres;
		break;
	case AD9081_MCS:
		ret = kstrtoul(buf, 0, &res);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		/* setup txfe loopback mode */
		ret = ad9081_multichip_sync(phy, res);
		if (!ret)
			phy->mcs_cached_val = res;

		break;
	case AD9081_DAC_FFH_INDEX_SET:
		ret = kstrtoul(buf, 0, &res);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		phy->ffh_hopf_index = res;
		break;
	case AD9081_DAC_FFH_FREQ_SET:
		ret = kstrtoul(buf, 0, &res);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		ret = adi_ad9081_hal_calc_tx_nco_ftw32(&phy->ad9081,
				phy->ad9081.dev_info.dac_freq_hz, res,
				&ftw);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		ret = adi_ad9081_dac_duc_main_nco_hopf_ftw_set(&phy->ad9081,
						 AD9081_DAC_ALL,
						 phy->ffh_hopf_index,
						 ftw);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		phy->ffh_hopf_vals[phy->ffh_hopf_index] = res;
		break;
	case AD9081_DAC_FFH_MODE_SET:
		ret = kstrtoul(buf, 0, &res);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		ret = adi_ad9081_dac_duc_main_nco_hopf_mode_set(&phy->ad9081,
						  AD9081_DAC_ALL, res);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		phy->ffh_hopf_mode = res;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9081_phy_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address & 0xFF) {
	case AD9081_LOOPBACK_MODE:
		ret = sprintf(buf, "%u\n", phy->device_cache.loopback_mode);
		break;
	case AD9081_ADC_CLK_PWDN:
		ret = sprintf(buf, "%u\n", phy->device_cache.adc_clk_pwdn);
		break;
	case AD9081_DAC_FFH_INDEX_SET:
		ret = sprintf(buf, "%u\n", phy->ffh_hopf_index);
		break;
	case AD9081_DAC_FFH_FREQ_SET:
		ret = sprintf(buf, "%u\n", phy->ffh_hopf_vals[phy->ffh_hopf_index]);
		break;
	case AD9081_DAC_FFH_MODE_SET:
		ret = sprintf(buf, "%u\n", phy->ffh_hopf_mode);
		break;
	case AD9081_MCS:
		ret = sprintf(buf, "%u\n", phy->mcs_cached_val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(loopback_mode, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_LOOPBACK_MODE);

static IIO_DEVICE_ATTR(adc_clk_powerdown, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_ADC_CLK_PWDN);

static IIO_DEVICE_ATTR(multichip_sync, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_MCS);

static IIO_DEVICE_ATTR(out_voltage_main_ffh_frequency, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_DAC_FFH_FREQ_SET);

static IIO_DEVICE_ATTR(out_voltage_main_ffh_index, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_DAC_FFH_INDEX_SET);

static IIO_DEVICE_ATTR(out_voltage_main_ffh_mode, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_DAC_FFH_MODE_SET);

static struct attribute *ad9081_phy_attributes[] = {
	&iio_dev_attr_loopback_mode.dev_attr.attr,
	&iio_dev_attr_adc_clk_powerdown.dev_attr.attr,
	&iio_dev_attr_multichip_sync.dev_attr.attr,
	&iio_dev_attr_out_voltage_main_ffh_frequency.dev_attr.attr,
	&iio_dev_attr_out_voltage_main_ffh_index.dev_attr.attr,
	&iio_dev_attr_out_voltage_main_ffh_mode.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9081_phy_attribute_group = {
	.attrs = ad9081_phy_attributes,
};

static int ad9081_request_fd_irqs(struct axiadc_converter *conv)
{
	struct device *dev = &conv->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9081_fdA_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-a", conv);
		if (ret < 0)
			return ret;
	}

	gpio = devm_gpiod_get(dev, "fastdetect-b", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9081_fdB_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-b", conv);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9081_post_setup(struct iio_dev *indio_dev)
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
static int ad9081_status_show(struct seq_file *file, void *offset)
{
	struct axiadc_converter *conv = spi_get_drvdata(file->private);
	struct ad9081_phy *phy = conv->phy;
	int ret, l;
	u16 stat;

	for (l = AD9081_LINK_0; l < AD9081_LINK_ALL; l++) {

		ret = adi_ad9081_jesd_rx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		seq_printf(file,
			"JESD TX (JRX) Link%d 0x%X lanes in DATA\n",
			l, stat & 0xF);

		if (phy->jesd_rx_link[l - 1].jesd_param.jesd_jesdv == 2)
			seq_printf(file,
				"JESD TX Link%d 204C status %d\n",
				l, stat >> 8);

		ret = adi_ad9081_jesd_tx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		seq_printf(file,
			"JESD RX (JTX) Link%d in %s, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
			l, ad9081_jtx_qbf_states[stat & 0xF],
			stat & BIT(4) ? "deasserted" : "asserted",
			stat & BIT(5) ? "locked" : "unlocked",
			stat & BIT(6) ? "established" : "lost",
			stat & BIT(7) ? "invalid" : "valid");

		if (!phy->jesd_rx_link[l - 1].jesd_param.jesd_duallink)
			return 0;

		if (!phy->jesd_tx_link.jesd_param.jesd_duallink)
			return 0;
	}

	return 0;
}

static void ad9081_work_func(struct work_struct *work)
{
	u8 status;
	int ret;
	struct ad9081_phy *phy =
		container_of(work, struct ad9081_phy, dwork.work);

	ret = adi_ad9081_hal_reg_get(&phy->ad9081, REG_IRQ_STATUS0_ADDR,
				     &status);

	if (!(status & BIT(6))) {
		dev_err(&phy->spi->dev, "IRQ_STATUS0: 0x%X\n", status);
		if (phy->jesd_tx_link.jesd_param.jesd_jesdv == 2) {
			ad9081_multichip_sync(phy, 7);
		} else {
			ad9081_multichip_sync(phy, 2);
			mdelay(20);
			ad9081_multichip_sync(phy, 3);
		}
	}
	schedule_delayed_work(&phy->dwork, msecs_to_jiffies(1000));
}

static int ad9081_post_iio_register(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	if (iio_get_debugfs_dentry(indio_dev)) {
		struct dentry *stats;

		stats = debugfs_create_devm_seqfile(&conv->spi->dev, "status",
					iio_get_debugfs_dentry(indio_dev),
					ad9081_status_show);
		if (PTR_ERR_OR_ZERO(stats))
			dev_err(&conv->spi->dev,
				"Failed to create debugfs entry");
	}

	return 0;
}

static int ad9081_get_jesd_converter_selection(struct ad9081_phy *phy,
					       const struct device_node *np,
					       const char *phandle_name,
					       struct ad9081_jesd_link *link)
{
	struct of_phandle_args args;
	int ret, i;
	u32 reg;

	for (i = 0; i < ARRAY_SIZE(link->link_converter_select); i++) {
		ret = of_parse_phandle_with_fixed_args(np, phandle_name, 1, i,
						       &args);
		if (ret)
			break;

		ret = of_property_read_u32(args.np, "reg", &reg);
		if (ret || reg > 7 || args.args[0] > 1) {
			dev_err(&phy->spi->dev,
				"Invalid converter selection (%d, %d, %d)\n",
				ret, reg, args.args[0]);
			return -EINVAL;
		}

		link->link_converter_select[i] = 2 * reg + args.args[0];

		dev_dbg(&phy->spi->dev, "%s: %d = 0x%X\n", __func__, i,
		       link->link_converter_select[i]);

		of_node_put(args.np);
	}

	return i;
}

static int ad9081_parse_jesd_link_dt(struct ad9081_phy *phy,
				     struct device_node *np,
				     struct ad9081_jesd_link *link, bool jtx)
{
	u32 tmp;
	int ret;

	link->jesd_param.jesd_scr = 1; /* Force scambling on */

	tmp = 0;
	of_property_read_u32(np, "adi,device-id", &tmp);
	link->jesd_param.jesd_did = tmp;

	ret = of_property_read_u32(np, "adi,octets-per-frame", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_f = tmp;

	ret = of_property_read_u32(np, "adi,frames-per-multiframe", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_k = tmp;

	ret = of_property_read_u32(np, "adi,samples-per-converter-per-frame",
				   &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_s = tmp;

	ret = of_property_read_u32(np, "adi,high-density", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_hd = tmp;

	ret = of_property_read_u32(np, "adi,converter-resolution", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_n = tmp;

	ret = of_property_read_u32(np, "adi,bits-per-sample", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_np = tmp;

	ret = of_property_read_u32(np, "adi,converters-per-device", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_m = tmp;

	ret = of_property_read_u32(np, "adi,control-bits-per-sample", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_cs = tmp;

	ret = of_property_read_u32(np, "adi,lanes-per-device", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_l = tmp;

	ret = of_property_read_u32(np, "adi,subclass", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_subclass = tmp;

	ret = of_property_read_u32(np, "adi,link-mode", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_mode_id = tmp;

	ret = of_property_read_u32(np, "adi,dual-link", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_duallink = tmp;

	ret = of_property_read_u32(np, "adi,version", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_jesdv = tmp;

	ret = of_property_read_variable_u8_array(
		np, "adi,logical-lane-mapping", link->logiclane_mapping, 1,
		ARRAY_SIZE(link->logiclane_mapping));

	if (ret < 0) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}

	if (jtx) { /* JTX - for RX ADC path */
		ret = ad9081_get_jesd_converter_selection(
			phy, np, "adi,converter-select", link);

		if (ret < 0 || ret != link->jesd_param.jesd_m) {
			dev_err(&phy->spi->dev,
				"converter number mismatch (%d != %d)\n", ret,
				link->jesd_param.jesd_m);
			return -EINVAL;
		}
	}

	return 0;
}

static int ad9081_reg_from_phandle(struct ad9081_phy *phy,
				   const struct device_node *np,
				   const char *phandle_name, int index,
				   u32 *reg)
{
	int ret;
	struct device_node *ph = of_parse_phandle(np, phandle_name, index);

	if (np == NULL)
		return -EINVAL;

	ret = of_property_read_u32(ph, "reg", reg);

	of_node_put(ph);

	return ret;
}

static int ad9081_parse_dt_tx(struct ad9081_phy *phy, struct device_node *np)
{
	struct device_node *of_channels, *of_chan;
	struct device_node *of_trx_path;
	u32 reg, index;
	int i, ret;

	of_trx_path = of_get_child_by_name(np, "adi,tx-dacs");
	if (of_trx_path == NULL)
		return -ENODEV;

	of_property_read_u64(of_trx_path, "adi,dac-frequency-hz",
			     &phy->dac_frequency_hz);

	/* The 4 DAC Main Datapaths */

	of_channels = of_get_child_by_name(of_trx_path, "adi,main-data-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	of_property_read_u32(of_channels, "adi,interpolation",
			     &phy->tx_main_interp);

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_main_shift))) {
			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->tx_main_shift[reg]);

			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
				i++) {
				ret = ad9081_reg_from_phandle(phy, of_chan,
					"adi,crossbar-select", i, &index);
				if (ret)
					break;

				if (index >= MAX_NUM_CHANNELIZER) {
					dev_err(&phy->spi->dev,
						"invalid device tree configuration: index (%d > %d)\n",
						index, MAX_NUM_CHANNELIZER);
					return -EINVAL;
				}

				phy->tx_dac_chan_xbar[reg] |= BIT(index);
			}
		}
	}

	of_node_put(of_channels);

	/* The 8 DAC Channelizers */

	of_channels =
		of_get_child_by_name(of_trx_path, "adi,channelizer-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	of_property_read_u32(of_channels, "adi,interpolation",
			     &phy->tx_chan_interp);

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_chan_shift))) {
			u32 val;

			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->tx_chan_shift[reg]);
			ret = of_property_read_u32(of_chan, "adi,gain", &val);
			if (!ret)
				phy->dac_cache.chan_gain[reg] = val;

			phy->dac_cache.enable[reg] = 1; /* powers up enabled */
		}
	}

	of_node_put(of_channels);

	of_channels = of_get_child_by_name(of_trx_path, "adi,jesd-links");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ad9081_parse_jesd_link_dt(phy, of_chan, &phy->jesd_tx_link,
					  false);
	}

	of_node_put(of_channels);
	of_node_put(of_trx_path);

	return 0;
}

static int ad9081_parse_dt_rx(struct ad9081_phy *phy, struct device_node *np)
{
	struct device_node *of_channels, *of_chan;
	struct device_node *of_trx_path;
	u32 reg;
	int i = 0, ret;

	/* The 4 ADC Main Datapaths */

	of_trx_path = of_get_child_by_name(np, "adi,rx-adcs");
	if (of_trx_path == NULL)
		return -ENODEV;

	of_property_read_u64(of_trx_path, "adi,adc-frequency-hz",
			     &phy->adc_frequency_hz);

	of_property_read_u32(of_trx_path, "adi,nyquist-zone",
			     &phy->rx_nyquist_zone);

	/* The 4 DAC Main Datapaths */

	of_channels = of_get_child_by_name(of_trx_path, "adi,main-data-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_main_shift))) {
			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->rx_cddc_shift[reg]);
			of_property_read_u32(of_chan, "adi,decimation",
					     &phy->adc_main_decimation[reg]);
			phy->rx_cddc_c2r[reg] = of_property_read_bool(
				of_chan, "adi,complex-to-real-enable");
			phy->rx_cddc_select |= BIT(reg);
		}
	}

	of_node_put(of_channels);

	/* The 8 ADC Channelizers */

	of_channels =
		of_get_child_by_name(of_trx_path, "adi,channelizer-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->rx_fddc_shift))) {
			of_property_read_u32(of_chan, "adi,decimation",
					     &phy->adc_chan_decimation[reg]);
			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->rx_fddc_shift[reg]);
			phy->rx_fddc_c2r[reg] = of_property_read_bool(
				of_chan, "adi,complex-to-real-enable");
			phy->rx_fddc_select |= BIT(reg);
		}
	}

	of_node_put(of_channels);

	of_channels = of_get_child_by_name(of_trx_path, "adi,jesd-links");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ad9081_parse_jesd_link_dt(phy, of_chan, &phy->jesd_rx_link[i++],
					  true);
	}

	of_node_put(of_channels);
	of_node_put(of_trx_path);

	return 0;
}

static int ad9081_parse_dt(struct ad9081_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	phy->multidevice_instance_count = 1;
	of_property_read_u32(np, "adi,multidevice-instance-count",
			     &phy->multidevice_instance_count);

	phy->config_sync_01_swapped =
		of_property_read_bool(np, "adi,jesd-sync-pins-01-swap-enable");

	phy->lmfc_delay = 0;
	of_property_read_u32(np, "adi,lmfc-delay-dac-clk-cycles",
			&phy->lmfc_delay);

	ret = ad9081_parse_dt_tx(phy, np);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "failed to parse devicetree");
		return ret;
	}

	ret = ad9081_parse_dt_rx(phy, np);
	if (ret < 0) {
		dev_err(&phy->spi->dev, "failed to parse devicetree");
		return ret;
	}

	/* More here */

	return 0;
}

static int ad9081_setup_chip_info_tbl(struct ad9081_phy *phy,
				      bool complex, bool buffer_capable)
{
	int i, c, m;

	m = phy->jesd_rx_link[0].jesd_param.jesd_m +
	    phy->jesd_rx_link[1].jesd_param.jesd_m;

	switch (m) {
	case 2:
	case 4:
	case 6:
	case 8:
	case 12:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	for (c = 0, i = 0; i < (m * phy->multidevice_instance_count);
		 i++, c++) {
		phy->chip_info.channel[c].type = IIO_VOLTAGE;
		phy->chip_info.channel[c].output = 0;
		phy->chip_info.channel[c].indexed = 1;
		phy->chip_info.channel[c].modified = complex ? 1 : 0;
		phy->chip_info.channel[c].channel = complex ? i / 2 : i;
		phy->chip_info.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		phy->chip_info.channel[c].scan_index = buffer_capable ? i : -1;
		phy->chip_info.channel[c].address = i,
		phy->chip_info.channel[c].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);

		if (i < m)
			phy->chip_info.channel[c].ext_info = rxadc_ext_info;

		phy->chip_info.channel[c].scan_type.realbits =
			phy->jesd_rx_link[0].jesd_param.jesd_n;
		phy->chip_info.channel[c].scan_type.storagebits =
			phy->jesd_rx_link[0].jesd_param.jesd_np;
		phy->chip_info.channel[c].scan_type.sign = 's';
	}

	m = phy->jesd_tx_link.jesd_param.jesd_m *
	    (phy->jesd_tx_link.jesd_param.jesd_duallink > 0 ? 2 : 1);

	for (i = 0; i < m; i++, c++) {
		phy->chip_info.channel[c].type = IIO_VOLTAGE;
		phy->chip_info.channel[c].output = 1;
		phy->chip_info.channel[c].indexed = 1;
		phy->chip_info.channel[c].modified = complex ? 1 : 0;
		phy->chip_info.channel[c].channel = complex ? i / 2 : i;
		phy->chip_info.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		phy->chip_info.channel[c].scan_index = -1;
		phy->chip_info.channel[c].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);

		phy->chip_info.channel[c].info_mask_separate =
			BIT(IIO_CHAN_INFO_ENABLE);

		phy->chip_info.channel[c].ext_info = txdac_ext_info;
	}

	phy->chip_info.num_channels = c;
	phy->chip_info.name = "AD9081";
	phy->chip_info.max_rate = 3000000000UL;

	return 0;
}

static const struct iio_info ad9081_iio_info = {
	.read_raw = &ad9081_read_raw,
	.write_raw = &ad9081_write_raw,
	.debugfs_reg_access = &ad9081_reg_access,
	.attrs = &ad9081_phy_attribute_group,
};

static int ad9081_register_iiodev(struct axiadc_converter *conv)
{
	struct iio_dev *indio_dev;
	struct spi_device *spi = conv->spi;
	struct ad9081_phy *phy = conv->phy;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, conv);

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	indio_dev->info = &ad9081_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = phy->chip_info.channel;
	indio_dev->num_channels = phy->chip_info.num_channels;

	ret = iio_device_register(indio_dev);
	ad9081_post_iio_register(indio_dev);

	conv->indio_dev = indio_dev;

	return ret;
}

static int ad9081_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9081_phy *phy;
	adi_cms_chip_id_t chip_id;
	u8 api_rev[3];
	u32 spi_id, fw_rev[2];
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (phy == NULL)
		return -ENOMEM;

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	spi_set_drvdata(spi, conv);
	conv->spi = spi;
	conv->phy = phy;
	phy->spi = spi;

	ret = ad9081_request_clks(conv);
	if (ret)
		return ret;

	phy->ad9081.hal_info.sdo = SPI_SDO;
	phy->ad9081.hal_info.msb = SPI_MSB_FIRST;
	phy->ad9081.hal_info.addr_inc = SPI_ADDR_INC_AUTO;
	phy->ad9081.hal_info.delay_us = ad9081_udelay;
	phy->ad9081.hal_info.spi_xfer = ad9081_spi_xfer;
	phy->ad9081.hal_info.reset_pin_ctrl = ad9081_reset_pin_ctrl;
	phy->ad9081.hal_info.user_data = conv;
	phy->ad9081.hal_info.log_write = ad9081_log_write;

	conv->reset_gpio =
		devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	phy->rx1_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "rx1-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->rx1_en_gpio))
		return PTR_ERR(phy->rx1_en_gpio);

	phy->rx2_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "rx2-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->rx2_en_gpio))
		return PTR_ERR(phy->rx2_en_gpio);

	phy->tx1_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "tx1-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->tx1_en_gpio))
		return PTR_ERR(phy->tx1_en_gpio);

	phy->tx2_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "tx2-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->tx2_en_gpio))
		return PTR_ERR(phy->tx2_en_gpio);

	ret = ad9081_parse_dt(phy, &spi->dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Parsing devicetree failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = adi_ad9081_device_reset(&phy->ad9081, AD9081_HARD_RESET_AND_INIT);
	if (ret < 0) {
		dev_err(&spi->dev, "reset/init failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = adi_ad9081_device_chip_id_get(&phy->ad9081, &chip_id);
	if (ret < 0) {
		dev_err(&spi->dev, "chip_id failed (%d)\n", ret);
		return -ENODEV;
	}

	spi_id = spi_get_device_id(spi)->driver_data;
	conv->id = chip_id.prod_id;
	if (conv->id != (spi_id & CHIPID_MASK)) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	ad9081_clk_register(phy, "-rx_sampl_clk",
			    __clk_get_name(phy->dev_clk), NULL,
			    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			    RX_SAMPL_CLK);

	ad9081_clk_register(phy, "-tx_sampl_clk",
			    __clk_get_name(phy->dev_clk), NULL,
			    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			    TX_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_AD9081_CLKS;

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_onecell_get,
				  &phy->clk_data);

	INIT_DELAYED_WORK(&phy->dwork, ad9081_work_func);


	switch (conv->id) {
	case CHIPID_AD9081:
		ret = ad9081_setup_chip_info_tbl(phy, true,
			!IS_ERR_OR_NULL(phy->jesd_rx_clk));
		if (ret)
			break;
		conv->chip_info = &phy->chip_info;
		ret = ad9081_setup(spi, false);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
		goto out_clk_del_provider;
	}

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->reg_access = ad9081_reg_access;
	conv->write_raw = ad9081_write_raw;
	conv->read_raw = ad9081_read_raw;
#if 0
	conv->read_event_value = ad9081_read_thresh,
	conv->write_event_value = ad9081_write_thresh,
	conv->read_event_config = ad9081_read_thresh_en,
	conv->write_event_config = ad9081_write_thresh_en,
	conv->set_pnsel = ad9081_set_pnsel;
#endif
	conv->post_setup = ad9081_post_setup;
	conv->post_iio_register = ad9081_post_iio_register;

	conv->attrs = &ad9081_phy_attribute_group;

	if (IS_ERR_OR_NULL(phy->jesd_rx_clk)) {
		ret = ad9081_register_iiodev(conv);
		if (ret)
			goto out_clk_del_provider;
	}

	if (conv->id == CHIPID_AD9081) {
		ret = ad9081_request_fd_irqs(conv);
		if (ret < 0)
			dev_warn(&spi->dev,
				 "Failed to request FastDetect IRQs (%d)", ret);
	}

	adi_ad9081_device_api_revision_get(&phy->ad9081, &api_rev[0],
					   &api_rev[1], &api_rev[2]);

	adi_ad9081_device_firmware_revision_get(&phy->ad9081, &fw_rev[0]);
	adi_ad9081_device_firmware_patch_revision_get(&phy->ad9081, &fw_rev[1]);

	dev_info(&spi->dev, "%s Rev. %u Grade %u Firmware %u.%u (API %u.%u.%u) probed\n",
		 conv->chip_info->name, chip_id.dev_revision,
		 chip_id.prod_grade, fw_rev[0], fw_rev[1],
		 api_rev[0], api_rev[1], api_rev[2]);

	return 0;

out_clk_del_provider:
	of_clk_del_provider(spi->dev.of_node);

	return ret;
}

static int ad9081_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;

	cancel_delayed_work_sync(&phy->dwork);

	if (!IS_ERR_OR_NULL(phy->jesd_tx_clk))
		clk_disable_unprepare(phy->jesd_tx_clk);

	if (!IS_ERR_OR_NULL(phy->jesd_rx_clk))
		clk_disable_unprepare(phy->jesd_rx_clk);
	else
		iio_device_unregister(conv->indio_dev);

	clk_disable_unprepare(phy->fmc_clk);
	clk_disable_unprepare(phy->dev_clk);
	of_clk_del_provider(spi->dev.of_node);
	adi_ad9081_device_deinit(&phy->ad9081);

	return 0;
}

static const struct spi_device_id ad9081_id[] = {
	{ "ad9081", CHIPID_AD9081 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9081_id);

static const struct of_device_id ad9081_of_match[] = {
	{ .compatible = "adi,ad9081" },
	{},
};
MODULE_DEVICE_TABLE(of, ad9081_of_match);

static struct spi_driver ad9081_driver = {
	.driver = {
			.name = "ad9081",
			.of_match_table = of_match_ptr(ad9081_of_match),
		},
	.probe = ad9081_probe,
	.remove = ad9081_remove,
	.id_table = ad9081_id,
};
module_spi_driver(ad9081_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9081 ADC");
MODULE_LICENSE("GPL v2");
