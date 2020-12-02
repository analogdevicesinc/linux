// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9208 and similar high-speed Analog-to-Digital converters
 *
 * Copyright 2019-2020 Analog Devices Inc.
 */

#include <linux/clk.h>
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
#include <linux/debugfs.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"
#include "ad9208/AD9208.h"
#include "ad9208/ad9208_reg.h"

#include <dt-bindings/iio/adc/adi,ad9208.h>

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#define CHIPID_AD9208			0xDF
#define CHIPID_AD6684			0xDC
#define CHIPID_AD6688			0xE2
#define CHIPID_AD9689			0xE2
#define CHIPID_AD9694			0xDB
#define CHIPID_AD9695			0xDE
#define CHIPID_AD9697			0xDE

#define CHIPID_MASK			0xFF
#define ID_DUAL				BIT(31)

enum {
	ID_AD9208,
	ID_AD9208_X2,
};

enum {
	DC_FILT,
};

struct ad9208_ddc {
	u32 decimation;
	u32 nco_mode;
	u64 carrier_freq_hz;
	u64 po;
	bool gain_db;
};

struct ad9208_jesd204_priv {
	struct ad9208_phy *phy;
};

struct ad9208_phy {
	ad9208_handle_t ad9208;
	struct axiadc_chip_info chip_info;
	struct jesd204_dev *jdev;
	struct jesd204_link jesd204_link;
	jesd_param_t jesd_param;
	u8 current_scale;
	bool dc_filter_enable;
	u32 ddc_cnt;
	u32 dcm;

	bool powerdown_pin_en;
	u32 powerdown_mode;

	u64 sampling_frequency_hz;
	u32 input_div;
	bool duty_cycle_stabilizer_en;

	bool analog_input_mode;
	bool ext_vref_en;
	u32 buff_curr_n;
	u32 buff_curr_p;

	u32 fc_ch;
	bool ddc_output_format_real_en;
	bool ddc_input_format_real_en;

	u32 jesd_subclass;
	u32 sysref_lmfc_offset;
	bool sysref_edge_sel;
	bool sysref_clk_edge_sel;
	u32 sysref_neg_window_skew;
	u32 sysref_pos_window_skew;
	u32 sysref_mode;
	u32 sysref_count;
	struct ad9208_ddc ddc[4];
};

static int ad9208_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int ad9208_spi_xfer(void *user_data, uint8_t *wbuf,
			   uint8_t *rbuf, int len)
{
	struct axiadc_converter *conv = user_data;
	int ret;

	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len,
	};

	ret = spi_sync_transfer(conv->spi, &t, 1);

	dev_dbg(&conv->spi->dev,"%s: reg=0x%X, val=0x%X",
		(wbuf[0] & 0x80) ? "rd" : "wr",
		(wbuf[0] & 0x7F) << 8 | wbuf[1],
		(wbuf[0] & 0x80) ? rbuf[2] : wbuf[2]);

	return ret;
}

static int ad9208_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int ret;
	u8 val;

	if (readval == NULL)
		return ad9208_register_write(&phy->ad9208, reg, writeval);

	ret = ad9208_register_read(&phy->ad9208, reg, &val);
	if (ret < 0)
		return ret;
	*readval = val;

	return 0;
}

static unsigned int ad9208_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return AD9208_TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return AD9208_TESTMODE_PN23_SEQ;
	default:
		return AD9208_TESTMODE_OFF;
	}
}

static int ad9208_testmode_set(struct iio_dev *indio_dev, unsigned int chan,
	unsigned int mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int ret;

	ad9208_adc_set_channel_select(&phy->ad9208, BIT(chan & 1));
	/* FIXME: Add support for DDC testmodes */
	ret = ad9208_register_write(&phy->ad9208, AD9208_REG_TEST_MODE, mode);
	conv->testmode[chan] = mode;
	ad9208_adc_set_channel_select(&phy->ad9208, AD9208_ADC_CH_ALL);

	return ret;
}

static int ad9208_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9208_pnsel_to_testmode(sel);
	struct ad9208_phy *phy = conv->phy;
	unsigned int output_mode;
	int ret;

	output_mode = conv->adc_output_mode;
	if (mode != AD9208_TESTMODE_OFF)
		output_mode &= ~AD9208_OUTPUT_MODE_TWOS_COMPLEMENT;

	ret = ad9208_register_write(&phy->ad9208, AD9208_REG_OUTPUT_MODE, output_mode);
	if (ret < 0)
		return ret;

	return ad9208_testmode_set(indio_dev, chan, mode);
}

static irqreturn_t ad9208_event_handler(struct axiadc_converter *conv,
	unsigned int chn)
{
	u64 event = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, chn,
			IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
	s64 timestamp = iio_get_time_ns(conv->indio_dev);

	if (conv->indio_dev)
		iio_push_event(conv->indio_dev, event, timestamp);

	return IRQ_HANDLED;
}

static irqreturn_t ad9208_fdA_handler(int irq, void *private)
{
	return ad9208_event_handler(private, 0);
}

static irqreturn_t ad9208_fdB_handler(int irq, void *private)
{
	return ad9208_event_handler(private, 1);
}

static int ad9208_read_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	u16 low, high;
	u8 val_h, val_l;

	mutex_lock(&indio_dev->mlock);
	ad9208_register_read(&phy->ad9208, AD9208_FD_LT_MSB_REG, &val_h);
	ad9208_register_read(&phy->ad9208, AD9208_FD_LT_LSB_REG, &val_l);
	low = (val_h << 8) | val_l;
	ad9208_register_read(&phy->ad9208, AD9208_FD_UT_MSB_REG, &val_h);
	ad9208_register_read(&phy->ad9208, AD9208_FD_UT_LSB_REG, &val_l);
	high = (val_h << 8) | val_l;
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

static int ad9208_read_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int ret;
	u8 val;

	ret = ad9208_register_read(&phy->ad9208, AD9208_CHIP_PIN_CTRL1_REG, &val);
	if (ret < 0)
		return ret;

	return !(val & AD9208_CHIP_PIN_CTRL_MASK(chan->channel));
}

static int ad9208_write_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int ret = 0;
	int low, high;
	u8 val_h, val_l;

	mutex_lock(&indio_dev->mlock);
	ad9208_register_read(&phy->ad9208, AD9208_FD_UT_MSB_REG, &val_h);
	ad9208_register_read(&phy->ad9208, AD9208_FD_UT_LSB_REG, &val_l);
	high = (val_h << 8) | val_l;

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

		ad9208_register_write(&phy->ad9208, AD9208_FD_UT_MSB_REG, val >> 8);
		ad9208_register_write(&phy->ad9208, AD9208_FD_UT_LSB_REG, val & 0xFF);

		/* Calculate the new lower threshold limit */
		ad9208_register_read(&phy->ad9208, AD9208_FD_LT_MSB_REG, &val_h);
		ad9208_register_read(&phy->ad9208, AD9208_FD_LT_LSB_REG, &val_l);

		low = (val_h << 8) | val_l;
		low = val - high + low;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	if (low < 0)
		low = 0;
	ad9208_register_write(&phy->ad9208, AD9208_FD_LT_MSB_REG, low >> 8);
	ad9208_register_write(&phy->ad9208, AD9208_FD_LT_LSB_REG, low & 0xFF);

unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad9208_write_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int ret;
	u8 val;

	mutex_lock(&indio_dev->mlock);

	ret = ad9208_register_read(&phy->ad9208, AD9208_CHIP_PIN_CTRL1_REG, &val);
	if (ret < 0)
		goto err_unlock;

	if (state)
		val &= ~AD9208_CHIP_PIN_CTRL_MASK(chan->channel);
	else
		val |= AD9208_CHIP_PIN_CTRL_MASK(chan->channel);

	ret = ad9208_register_write(&phy->ad9208, AD9208_CHIP_PIN_CTRL1_REG, val);
err_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static const int ad9208_scale_table[][2] = {
	{1130, AD9208_ADC_SCALE_1P13_VPP}, {1250, AD9208_ADC_SCALE_1P25_VPP},
	{1700, AD9208_ADC_SCALE_1P7_VPP}, {1810, AD9208_ADC_SCALE_1P81_VPP},
	{1930, AD9208_ADC_SCALE_1P93_VPP}, {2040, AD9208_ADC_SCALE_2P04_VPP},
};

static void ad9208_scale(struct axiadc_converter *conv, int index,
	unsigned int *val, unsigned int *val2)
{
	unsigned int tmp;

	if (index > conv->chip_info->num_scales) {
		*val = 0;
		*val2 = 0;
		return;
	}

	tmp = (conv->chip_info->scale_table[index][0] * 1000000ULL) >>
		    conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;
}

static ssize_t ad9208_show_scale_available(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int scale[2];
	int i, len = 0;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9208_scale(conv, i, &scale[0], &scale[1]);
		len += sprintf(buf + len, "%u.%06u ", scale[0], scale[1]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static int ad9208_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	struct ad9208_phy *phy = conv->phy;
	unsigned int i;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		if (phy->current_scale == conv->chip_info->scale_table[i][1])
			break;
	}

	ad9208_scale(conv, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ad9208_set_scale(struct axiadc_converter *conv, int val, int val2)
{
	struct ad9208_phy *phy = conv->phy;
	unsigned int scale_val[2];
	unsigned int i;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9208_scale(conv, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		ad9208_adc_set_input_scale(&phy->ad9208,
					   conv->chip_info->scale_table[i][1]);

		phy->current_scale = conv->chip_info->scale_table[i][1];
		return 0;
	}

	return -EINVAL;
}

static int ad9208_testmode_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->testmode[chan->channel];
}

static int ad9208_testmode_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = ad9208_testmode_set(indio_dev, chan->channel, item);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const char * const ad9208_testmodes[] = {
	[AD9208_TESTMODE_OFF] = "off",
	[AD9208_TESTMODE_MIDSCALE_SHORT] = "midscale_short",
	[AD9208_TESTMODE_POS_FULLSCALE] = "pos_fullscale",
	[AD9208_TESTMODE_NEG_FULLSCALE] = "neg_fullscale",
	[AD9208_TESTMODE_ALT_CHECKERBOARD] = "checkerboard",
	[AD9208_TESTMODE_PN23_SEQ] = "pn_long",
	[AD9208_TESTMODE_PN9_SEQ] = "pn_short",
	[AD9208_TESTMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[AD9208_TESTMODE_USER] = "user",
	[AD9208_TESTMODE_RAMP] = "ramp",
};

static const struct iio_enum ad9208_testmode_enum = {
	.items = ad9208_testmodes,
	.num_items = ARRAY_SIZE(ad9208_testmodes),
	.set = ad9208_testmode_write,
	.get = ad9208_testmode_read,
};

static ssize_t ad9208_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	int val, ret;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case DC_FILT:
		val = phy->dc_filter_enable;
		ret = 0;
		break;
	default:
		ret = -EINVAL;

	}

	mutex_unlock(&indio_dev->mlock);

	if (ret == 0)
		ret = sprintf(buf, "%d\n", val);

	return ret;
}

static ssize_t ad9208_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9208_phy *phy = conv->phy;
	bool enable;
	int ret;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case DC_FILT:
		ret = ad9208_adc_set_dc_offset_filt_en(&phy->ad9208, enable);
		if (ret == 0)
			phy->dc_filter_enable = enable;
		break;
	default:
		ret = -EINVAL;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	IIO_ENUM("test_mode", IIO_SEPARATE, &ad9208_testmode_enum),
	IIO_ENUM_AVAILABLE("test_mode", &ad9208_testmode_enum),
	{
		.name = "scale_available",
		.read = ad9208_show_scale_available,
		.shared = true,
	},
	{
		.name = "dc_filter_enable",
		.read = ad9208_ext_info_read,
		.write = ad9208_ext_info_write,
		.shared = true,
		.private = DC_FILT,
	},
	{},
};

static const struct iio_event_spec ad9208_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_type = BIT(IIO_EV_INFO_VALUE) |
				       BIT(IIO_EV_INFO_HYSTERESIS),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define AD9208_CHAN(_chan, _si, _bits, _sign, _shift, _ev, _nb_ev)	\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,					\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	  .event_spec = _ev,						\
	  .num_event_specs = _nb_ev,					\
	}

#define AD9208_MC_CHAN(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
		.indexed = 1,						\
		.channel = _chan,					\
		.scan_index = _si,					\
		.scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
		},							\
	}

static struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9208] = {
		.name = "AD9208",
		.max_rate = 3000000000UL,
		.scale_table = ad9208_scale_table,
		.num_scales = ARRAY_SIZE(ad9208_scale_table),
		.num_channels = 2,
		.channel[0] = AD9208_CHAN(0, 0, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[1] = AD9208_CHAN(1, 1, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[2] = AD9208_CHAN(2, 2, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[3] = AD9208_CHAN(3, 3, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[4] = AD9208_CHAN(4, 4, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[5] = AD9208_CHAN(5, 5, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[6] = AD9208_CHAN(6, 6, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[7] = AD9208_CHAN(7, 7, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
	},
	[ID_AD9208_X2] = {
		.name = "AD9208 DUAL (MASTER with DMA)",
		.max_rate = 3000000000UL,
		.scale_table = ad9208_scale_table,
		.num_scales = ARRAY_SIZE(ad9208_scale_table),
		.num_channels = 2,
		.num_shadow_slave_channels = 2,
		.channel[0] = AD9208_CHAN(0, 0, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[1] = AD9208_CHAN(1, 1, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[2] = AD9208_CHAN(2, 2, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[3] = AD9208_CHAN(3, 3, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[4] = AD9208_CHAN(4, 4, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[5] = AD9208_CHAN(5, 5, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[6] = AD9208_CHAN(6, 6, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),
		.channel[7] = AD9208_CHAN(7, 7, 14, 'S', 0,
			ad9208_events, ARRAY_SIZE(ad9208_events)),

		.channel[8] = AD9208_MC_CHAN(8, 8, 14, 'S', 0),
		.channel[9] = AD9208_MC_CHAN(9, 9, 14, 'S', 0),
		.channel[10] = AD9208_MC_CHAN(10, 10, 14, 'S', 0),
		.channel[11] = AD9208_MC_CHAN(11, 11, 14, 'S', 0),
		.channel[12] = AD9208_MC_CHAN(12, 12, 14, 'S', 0),
		.channel[13] = AD9208_MC_CHAN(13, 13, 14, 'S', 0),
		.channel[14] = AD9208_MC_CHAN(14, 14, 14, 'S', 0),
		.channel[15] = AD9208_MC_CHAN(15, 15, 14, 'S', 0),
	},
};

static int ad9208_set_sample_rate(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	/* TODO: Not yet supported */
	return -ENOTSUPP;
}

static int ad9208_request_clks(struct axiadc_converter *conv)
{
	struct ad9208_phy *phy = conv->phy;
	int ret;

	conv->clk = devm_clk_get(&conv->spi->dev, "adc_clk");
	if (IS_ERR(conv->clk) && PTR_ERR(conv->clk) != -ENOENT)
		return PTR_ERR(conv->clk);

	if (phy->jdev)
		return 0;

	conv->lane_clk = devm_clk_get(&conv->spi->dev, "jesd_adc_clk");
	if (IS_ERR(conv->lane_clk) && PTR_ERR(conv->lane_clk) != -ENOENT)
		return PTR_ERR(conv->lane_clk);

	conv->sysref_clk = devm_clk_get(&conv->spi->dev, "adc_sysref");
	if (IS_ERR(conv->sysref_clk)) {
		if (PTR_ERR(conv->sysref_clk) != -ENOENT)
			return PTR_ERR(conv->sysref_clk);
		conv->sysref_clk = NULL;
	} else {
		ret = clk_prepare_enable(conv->sysref_clk);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9208_setup(struct spi_device *spi, bool ad9234)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9208_phy *phy = conv->phy;

	uint64_t lane_rate_kbps;
	u8 pll_stat, dcm;
	int ret, timeout, i;
	u64 sample_rate;
	ad9208_adc_data_frmt_t input_fmt, output_fmt;

	ret = ad9208_request_clks(conv);
	if (ret)
		return ret;

	ad9208_adc_set_channel_select(&phy->ad9208, AD9208_ADC_CH_ALL);

	ret = ad9208_set_pdn_pin_mode(&phy->ad9208, phy->powerdown_pin_en,
				      phy->powerdown_mode);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to set PWDN pin mode (%d)\n", ret);
		return ret;
	}

	ret = ad9208_set_input_clk_duty_cycle_stabilizer(&phy->ad9208,
						phy->duty_cycle_stabilizer_en);
	if (ret < 0) {
		dev_err(&spi->dev,
			"Failed to set clk duty cycle stabilizer (%d)\n", ret);
		return ret;
	}

	sample_rate = phy->sampling_frequency_hz * phy->input_div;

	ret = ad9208_set_input_clk_cfg(&phy->ad9208, sample_rate,
				       phy->input_div);
	if (ret < 0) {
		dev_err(&spi->dev,
			"Failed to set input clk config (%d)\n", ret);
		return ret;
	}

	ret = clk_set_rate(conv->clk, sample_rate);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to set converter clock rate to %llu kHz: %d\n",
			sample_rate, ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->clk);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to enable converter clock: %d\n", ret);
		return ret;
	}

	phy->current_scale = AD9208_ADC_SCALE_1P7_VPP;

	ret = ad9208_adc_set_input_cfg(&phy->ad9208,
			phy->analog_input_mode ? COUPLING_DC : COUPLING_AC,
			phy->ext_vref_en, phy->current_scale);
	if (ret) {
		dev_err(&spi->dev, "Failed to set adc input config: %d\n", ret);
		return ret;
	}

	ret = ad9208_adc_set_input_buffer_cfg(&phy->ad9208, phy->buff_curr_n,
			phy->buff_curr_p, AD9208_BUFF_CURR_600_UA);
	if (ret) {
		dev_err(&spi->dev,
			"Failed to set input buffer config: %d\n", ret);
		return ret;
	}


	ret = ad9208_adc_set_fc_ch_mode(&phy->ad9208, phy->fc_ch);
	if (ret) {
		dev_err(&spi->dev, "Failed to set channel mode: %d\n", ret);
		return ret;
	}

	if (phy->fc_ch == AD9208_FULL_BANDWIDTH_MODE) {
		dcm = 1; /* Full bandwidth */
	} else {
		dcm = phy->ddc[0].decimation;
		for (i = 1; i < phy->ddc_cnt; i++)
			dcm = min_t(u8, dcm, phy->ddc[i].decimation);
	}

	dev_dbg(&spi->dev, "using chip decimation %u\n", ret);

	ret = ad9208_adc_set_dcm_mode(&phy->ad9208, dcm);
	if (ret) {
		dev_err(&spi->dev, "Failed to set decimation mode: %d\n", ret);
		return ret;
	}

	phy->dcm = dcm;

	/* DDC Setup */

	if (phy->ddc_input_format_real_en)
		input_fmt = AD9208_DATA_FRMT_REAL;
	else
		input_fmt = AD9208_DATA_FRMT_COMPLEX;

	if (phy->ddc_output_format_real_en)
		output_fmt = AD9208_DATA_FRMT_REAL;
	else
		output_fmt = AD9208_DATA_FRMT_COMPLEX;

	ad9208_adc_set_data_format(&phy->ad9208, input_fmt, output_fmt);

	for (i = 0; i < phy->ddc_cnt; i++) {
		ret = ad9208_adc_set_ddc_gain(&phy->ad9208, i,
					      phy->ddc[i].gain_db ? 6 : 0);
		if (ret) {
			dev_err(&spi->dev, "Failed to set ddc gain: %d\n", ret);
			return ret;
		}
		ret = ad9208_adc_set_ddc_dcm(&phy->ad9208, i,
					     phy->ddc[i].decimation);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to set ddc decimation mode: %d\n", ret);
			return ret;
		}
		ret = ad9208_adc_set_ddc_nco_mode(&phy->ad9208, i,
						  phy->ddc[i].nco_mode);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to set ddc nco mode: %d\n", ret);
			return ret;
		}
		ret = ad9208_adc_set_ddc_nco(&phy->ad9208, i,
					     phy->ddc[i].carrier_freq_hz);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to set ddc nco frequency: %d\n", ret);
			return ret;
		}
		ret = ad9208_adc_set_ddc_nco_phase(&phy->ad9208, i,
						   phy->ddc[i].po);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to set ddc nco phase offset: %d\n",
				ret);
			return ret;
		}
	}

	ret = ad9208_jesd_syref_lmfc_offset_set(&phy->ad9208,
						phy->sysref_lmfc_offset);

	ret = ad9208_jesd_syref_config_set(&phy->ad9208, phy->sysref_edge_sel,
				     phy->sysref_clk_edge_sel,
				     phy->sysref_neg_window_skew,
				     phy->sysref_pos_window_skew);

	ret = ad9208_jesd_syref_mode_set(&phy->ad9208, phy->sysref_mode,
					 phy->sysref_count);

	ret = ad9208_jesd_set_if_config(&phy->ad9208, phy->jesd_param,
					&lane_rate_kbps);
	if (ret < 0) {
		dev_err(&spi->dev,
			"Failed to set JESD204 interface config (%d)\n", ret);
		return ret;
	}

	ret = ad9208_jesd_subclass_set(&phy->ad9208, phy->jesd_subclass);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to set subclass (%d)\n", ret);
		return ret;
	}

	ret = ad9208_jesd_enable_scrambler(&phy->ad9208, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable scrambler (%d)\n", ret);
		return ret;
	}

	timeout = 10;

	do {
		mdelay(10);

		ret = ad9208_jesd_get_pll_status(&phy->ad9208, &pll_stat);
		if (ret < 0) {
			dev_err(&spi->dev,
				"Failed to get pll status (%d)\n", ret);
			return ret;
		}
	} while (!(pll_stat & AD9208_JESD_PLL_LOCK_STAT) && timeout--);

	dev_info(&conv->spi->dev, "AD9208 PLL %s\n",
		 pll_stat & AD9208_JESD_PLL_LOCK_STAT ? "LOCKED" : "UNLOCKED");

	if (!phy->jdev) {
		ret = clk_set_rate(conv->lane_clk, lane_rate_kbps);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to set lane rate to %llu kHz: %d\n",
				lane_rate_kbps, ret);
		}

		ret = ad9208_jesd_enable_link(&phy->ad9208, 1);
		if (ret < 0) {
			dev_err(&spi->dev,
				"Failed to enabled JESD204 link (%d)\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(conv->lane_clk);
		if (ret < 0) {
			dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int ad9208_status_show(struct seq_file *file, void *offset)
{
	struct axiadc_converter *conv = spi_get_drvdata(file->private);
	struct ad9208_phy *phy = conv->phy;
	const char *hold_setup_desc;
	u8 hold, setup, phase, stat;

	ad9208_register_read(&phy->ad9208, AD9208_IP_CLK_STAT_REG, &stat);
	seq_printf(file, "Input clock %sdetected\n",
		   (stat & 0x01) ? "" : "not ");

	ad9208_jesd_get_pll_status(&phy->ad9208, &stat);
	seq_printf(file, "JESD204 PLL is %slocked\n",
		   (stat & AD9208_JESD_PLL_LOCK_STAT) ? "" : "not ");

	ad9208_register_read(&phy->ad9208, AD9208_SYSREF_STAT_2_REG, &stat);
	seq_printf(file, "SYSREF counter: %d\n", stat);

	ad9208_jesd_syref_status_get(&phy->ad9208, &hold, &setup, &phase);
	if (hold == 0x0 && setup <= 0x7)
		hold_setup_desc = "Possible setup error";
	else if (hold <= 0x8 && setup == 0x8)
		hold_setup_desc = "No setup or hold error (best hold margin)";
	else if (hold == 0x8 && setup >= 0x9)
		hold_setup_desc =
			"No setup or hold error (best setup and hold margin)";
	else if (hold == 0x8 && setup == 0x0)
		hold_setup_desc = "No setup or hold error (best setup margin)";
	else if (hold >= 0x9 && setup == 0x0)
		hold_setup_desc = "Possible hold error";
	else
		hold_setup_desc = "Possible setup or hold error";

	seq_printf(file, "SYSREF hold/setup status: %s (%x/%x)\n"
		   "SYSREF divider phase %d * 1/2 cycles delayed\n",
		   hold_setup_desc, hold, setup, phase);

	return 0;
}

static int ad9208_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad9208_get_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate_scaled(conv->clk,
							   &conv->adc_clkscale);
		return IIO_VAL_INT;

	}
	return -EINVAL;
}

static int ad9208_write_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val, int val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad9208_set_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		if (conv->id == CHIPID_AD9208)
			return ad9208_set_sample_rate(conv, val);

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
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9208_request_fd_irqs(struct axiadc_converter *conv)
{
	struct device *dev = &conv->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9208_fdA_handler,
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
				irq, NULL, ad9208_fdB_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-b", conv);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9208_post_setup(struct iio_dev *indio_dev)
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

static int ad9208_post_iio_register(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	if (iio_get_debugfs_dentry(indio_dev)) {
		struct dentry *stats;

		stats = debugfs_create_devm_seqfile(&conv->spi->dev, "status",
					iio_get_debugfs_dentry(indio_dev),
					ad9208_status_show);
		if (PTR_ERR_OR_ZERO(stats))
			dev_err(&conv->spi->dev,
				"Failed to create debugfs entry");
	}

	return 0;
}

static int ad9208_parse_dt(struct ad9208_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *chan_np;
	u32 tmp, reg;
	int ret;

	/* Pin Config */

	phy->powerdown_pin_en = of_property_read_bool(np,
					"adi,powerdown-pin-enable");

	tmp = AD9208_POWERUP;
	of_property_read_u32(np, "adi,powerdown-mode", &tmp);
	phy->powerdown_mode = tmp;

	/* Clock Config */

	of_property_read_u64(np, "adi,sampling-frequency",
			     &phy->sampling_frequency_hz);

	tmp = 1;
	of_property_read_u32(np, "adi,input-clock-divider-ratio", &tmp);
	phy->input_div = tmp;

	phy->duty_cycle_stabilizer_en = of_property_read_bool(np,
					"adi,duty-cycle-stabilizer-enable");

	/* Analog Conifg */

	phy->analog_input_mode = of_property_read_bool(np,
					"adi,analog-input-dc-coupling-enable");

	phy->ext_vref_en  = of_property_read_bool(np,
					"adi,external-vref-enable");

	tmp = AD9208_ADC_BUFF_CURR_500_UA;
	of_property_read_u32(np, "adi,analog-input-neg-buffer-current", &tmp);
	phy->buff_curr_n = tmp;

	tmp = AD9208_ADC_BUFF_CURR_500_UA;
	of_property_read_u32(np, "adi,analog-input-pos-buffer-current", &tmp);
	phy->buff_curr_p = tmp;

	/* SYSREF Config */

	tmp = 0;
	of_property_read_u32(np, "adi,sysref-lmfc-offset", &tmp);
	phy->sysref_lmfc_offset = tmp;

	phy->sysref_edge_sel = of_property_read_bool(np,
					"adi,sysref-edge-high-low-enable");
	phy->sysref_clk_edge_sel  = of_property_read_bool(np,
					"adi,sysref-clk-edge-falling-enable");

	tmp = 0;
	of_property_read_u32(np, "adi,sysref-neg-window-skew", &tmp);
	phy->sysref_neg_window_skew = tmp;

	tmp = 0;
	of_property_read_u32(np, "adi,sysref-pos-window-skew", &tmp);
	phy->sysref_pos_window_skew = tmp;

	tmp = AD9208_SYSREF_CONT;
	of_property_read_u32(np, "adi,sysref-mode", &tmp);
	phy->sysref_mode = tmp;

	tmp = 0;
	of_property_read_u32(np,  "adi,sysref-nshot-ignore-count", &tmp);
	phy->sysref_count = tmp;

	/* DDC Config */

	tmp = AD9208_FULL_BANDWIDTH_MODE;
	of_property_read_u32(np, "adi,ddc-channel-number", &tmp);
	phy->fc_ch = tmp;

	phy->ddc_output_format_real_en = of_property_read_bool(np,
					"adi,ddc-complex-to-real-enable");
	phy->ddc_input_format_real_en = of_property_read_bool(np,
					"adi,ddc-mixer-real-enable");

	for_each_child_of_node(np, chan_np) {
		ret = of_property_read_u32(chan_np, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->ddc))) {
			ret = of_property_read_u32(chan_np, "adi,decimation",
						   &phy->ddc[reg].decimation);
			if (ret)
				return ret;
			ret = of_property_read_u32(chan_np,
						"adi,nco-mode-select",
						&phy->ddc[reg].nco_mode);
			if (ret)
				return ret;

			of_property_read_u64(chan_np,
				"adi,nco-channel-carrier-frequency-hz",
				&phy->ddc[reg].carrier_freq_hz);
			of_property_read_u64(chan_np,
				"adi,nco-channel-phase-offset",
				&phy->ddc[reg].po);
			phy->ddc[reg].gain_db = of_property_read_bool(chan_np,
				"adi,ddc-gain-6dB-enable");
			phy->ddc_cnt++;
		}
	}

	/* JESD Link Config */

	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, &phy->jesd204_link,
					  &phy->jesd_param.jesd_F, 1);

	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, &phy->jesd204_link,
					       &phy->jesd_param.jesd_K , 32);

	JESD204_LNK_READ_HIGH_DENSITY(dev, np, &phy->jesd204_link,
				      &phy->jesd_param.jesd_HD, 0);

	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, &phy->jesd204_link,
					      &phy->jesd_param.jesd_N, 16);

	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
					 &phy->jesd_param.jesd_NP, 16);

	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, &phy->jesd204_link,
					&phy->jesd_param.jesd_M, 2);

	JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
					      &phy->jesd_param.jesd_CS, 0);

	JESD204_LNK_READ_NUM_LANES(dev, np, &phy->jesd204_link,
				   &phy->jesd_param.jesd_L , 8);

	JESD204_LNK_READ_SUBCLASS(dev, np, &phy->jesd204_link,
				  &phy->jesd_subclass, JESD_SUBCLASS_0);

	return 0;
}

static int ad9208_setup_chip_info_tbl(struct ad9208_phy *phy, u32 id)
{
	int i, m = phy->jesd_param.jesd_M;

	memcpy(&phy->chip_info, &axiadc_chip_info_tbl[id],
	       sizeof(phy->chip_info));

	switch (m) {
	case 1:
	case 2:
	case 4:
	case 8:
		break;
	default:
		return -EINVAL;
	}

	phy->chip_info.num_channels = m;

	if (axiadc_chip_info_tbl[id].num_shadow_slave_channels)
		phy->chip_info.num_channels *= 2;

	for (i = 0; i < phy->chip_info.num_channels; i++) {
		phy->chip_info.channel[i].scan_type.realbits =
			phy->jesd_param.jesd_N;
		phy->chip_info.channel[i].scan_type.storagebits =
			phy->jesd_param.jesd_NP;

		if (i >= m) { /* Shadow channels */
			phy->chip_info.channel[i].info_mask_shared_by_type = 0;
			phy->chip_info.channel[i].ext_info = NULL;
			phy->chip_info.channel[i].event_spec = NULL;
			phy->chip_info.channel[i].num_event_specs = 0;
		}
	}

	return 0;
}

static int ad9208_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9208_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9208_phy *phy = priv->phy;
	struct jesd204_link *link;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &phy->jesd204_link;

	jesd204_copy_link_params(lnk, link);

	lnk->sample_rate = phy->sampling_frequency_hz;
	lnk->sample_rate_div = phy->dcm;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	if (phy->sysref_mode == AD9208_SYSREF_CONT)
		lnk->sysref.mode = JESD204_SYSREF_CONTINUOUS;
	else if (phy->sysref_mode == AD9208_SYSREF_ONESHOT)
		lnk->sysref.mode = JESD204_SYSREF_ONESHOT;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9208_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9208_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9208_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	ret = ad9208_jesd_enable_link(&phy->ad9208,
		reason == JESD204_STATE_OP_REASON_INIT);
	if (ret < 0) {
		dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9208_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9208_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9208_jesd204_link_init,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9208_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9208_jesd204_link_enable,
			.post_state_sysref = true,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9208_jesd204_priv),
};

static int ad9208_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9208_phy *phy;
	struct jesd204_dev *jdev;
	struct ad9208_jesd204_priv *priv;
	adi_chip_id_t chip_id;
	u8 api_rev[3];
	u32 spi_id;
	int ret;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9208_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

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

	if (jdev) {
		phy->jdev = jdev;
		priv = jesd204_dev_priv(jdev);
		priv->phy = phy;
	}

	phy->ad9208.user_data = conv;
	phy->ad9208.dev_xfer = ad9208_spi_xfer;
	phy->ad9208.delay_us = ad9208_udelay;

	conv->pwrdown_gpio = devm_gpiod_get_optional(&spi->dev, "powerdown",
		GPIOD_OUT_LOW);
	if (IS_ERR(conv->pwrdown_gpio))
		return PTR_ERR(conv->pwrdown_gpio);


	ret = ad9208_parse_dt(phy, &spi->dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Parsing devicetree failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = ad9208_init(&phy->ad9208);
	if (ret < 0) {
		dev_err(&spi->dev, "init failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = ad9208_reset(&phy->ad9208, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "reset failed (%d)\n", ret);
		return -ENODEV;
	}

	ad9208_get_chip_id(&phy->ad9208, &chip_id);
	if (ret < 0) {
		dev_err(&spi->dev, "reset failed (%d)\n", ret);
		return -ENODEV;
	}

	spi_id = spi_get_device_id(spi)->driver_data;
	conv->id = chip_id.prod_id;
	if (conv->id != (spi_id & CHIPID_MASK)) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	switch (conv->id) {
	case CHIPID_AD9208:
	case CHIPID_AD6684:
	case CHIPID_AD9689:
	case CHIPID_AD9694:
	case CHIPID_AD9695:
		ret = ad9208_setup_chip_info_tbl(phy, (spi_id & ID_DUAL) ?
						 ID_AD9208_X2 : ID_AD9208);
		if (ret)
			break;
		conv->chip_info = &phy->chip_info;
		ret = ad9208_setup(spi, false);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
		return ret;
	}

	conv->reg_access = ad9208_reg_access;
	conv->write_raw = ad9208_write_raw;
	conv->read_raw = ad9208_read_raw;
	conv->read_event_value = ad9208_read_thresh,
	conv->write_event_value = ad9208_write_thresh,
	conv->read_event_config = ad9208_read_thresh_en,
	conv->write_event_config = ad9208_write_thresh_en,
	conv->post_setup = ad9208_post_setup;
	conv->post_iio_register = ad9208_post_iio_register;
	conv->set_pnsel = ad9208_set_pnsel;

	if (conv->id == CHIPID_AD9208) {
		ret = ad9208_request_fd_irqs(conv);
		if (ret < 0)
			dev_warn(&spi->dev,
				 "Failed to request FastDetect IRQs (%d)", ret);
	}

	ad9208_get_revision(&phy->ad9208, &api_rev[0],
			    &api_rev[1], &api_rev[2]);

	dev_info(&spi->dev, "%s Rev. %u Grade %u (API %u.%u.%u) probed\n",
		 conv->chip_info->name, chip_id.dev_revision,
		 chip_id.prod_grade, api_rev[0], api_rev[1], api_rev[2]);

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static int ad9208_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9208_phy *phy = conv->phy;

	clk_disable_unprepare(conv->clk);

	if (!IS_ERR_OR_NULL(conv->sysref_clk))
		clk_disable_unprepare(conv->sysref_clk);

	clk_disable_unprepare(conv->lane_clk);


	ad9208_deinit(&phy->ad9208);

	return 0;
}

static const struct spi_device_id ad9208_id[] = {
	{ "ad9208", CHIPID_AD9208 },
	{ "ad9208x2", CHIPID_AD9208 | ID_DUAL},
	{ "ad6684", CHIPID_AD6684 },
	{ "ad6688", CHIPID_AD6688 },
	{ "ad9689", CHIPID_AD9689 },
	{ "ad9694", CHIPID_AD9694 },
	{ "ad9695", CHIPID_AD9695 },
	{ "ad9697", CHIPID_AD9697 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9208_id);

static const struct of_device_id ad9208_of_match[] = {
	{ .compatible = "adi,ad9208" },
	{ .compatible = "adi,ad9208x2" },
	{ .compatible = "adi,ad6684" },
	{ .compatible = "adi,ad6688" },
	{ .compatible = "adi,ad9689" },
	{ .compatible = "adi,ad9694" },
	{ .compatible = "adi,ad9695" },
	{ .compatible = "adi,ad9697" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad9208_of_match);

static struct spi_driver ad9208_driver = {
	.driver = {
		   .name = "ad9208",
		   .of_match_table = of_match_ptr(ad9208_of_match),
	},
	.probe = ad9208_probe,
	.remove = ad9208_remove,
	.id_table = ad9208_id,
};
module_spi_driver(ad9208_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9208 ADC");
MODULE_LICENSE("GPL v2");
