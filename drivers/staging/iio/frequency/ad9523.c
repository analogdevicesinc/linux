/*
 * AD9523 SPI Low Jitter Clock Generator
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "../iio.h"
#include "../sysfs.h"

#include "ad9523.h"

enum {
	AD9523_STATUS0,
	AD9523_VCO1_FREQ,
	AD9523_VCO2_FREQ,
	AD9523_VCO_FREQ_AVAIL,
	AD9523_SYNC,
	AD9523_EEPROM,
	AD9523_ALT_SRC_CH0,
	AD9523_ALT_SRC_CH1,
	AD9523_ALT_SRC_CH2,
	AD9523_ALT_SRC_CH3,
	AD9523_ALT_SRC_CH4,
	AD9523_ALT_SRC_CH5,
	AD9523_ALT_SRC_CH6,
	AD9523_ALT_SRC_CH7,
	AD9523_ALT_SRC_CH8,
	AD9523_ALT_SRC_CH9,
};

#define AD_IF(_pde, _a) ((pdata->_pde) ? _a : 0)
#define AD_IFE(_pde, _a, _b) ((pdata->_pde) ? _a : _b)

struct ad9523_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	struct ad9523_platform_data	*pdata;
	struct iio_chan_spec		ad9523_channels[14];

	unsigned long			vcxo_freq;
	unsigned long			vco_freq;
	unsigned long			vco_out_freq[3];
	unsigned char			vco_out_map[14];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[4];
	} data[2] ____cacheline_aligned;
};

static struct ad9523_platform_data default_pdata = {

};

static int ad9523_read(struct iio_dev *indio_dev, unsigned addr)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	struct spi_message m;
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
			.cs_change = 0,
		}, {
			.rx_buf = &st->data[1].d8[4 - AD9523_TRANSFER_LEN(addr)],
			.len = AD9523_TRANSFER_LEN(addr),
		},
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	st->data[0].d32 = cpu_to_be32(AD9523_READ |
				      AD9523_CNT(AD9523_TRANSFER_LEN(addr)) |
				      AD9523_ADDR(addr));

	ret = spi_sync(st->spi, &m);
	if (ret >= 0)
		ret = be32_to_cpu(st->data[1].d32) & (0xFFFFFF >>
				  (8 * (3 - AD9523_TRANSFER_LEN(addr))));
	else
		dev_err(&indio_dev->dev, "read failed (%d)", ret);

	return ret;
};

static int ad9523_write(struct iio_dev *indio_dev, unsigned addr, unsigned val)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	struct spi_message m;
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
			.cs_change = 0,
		}, {
			.tx_buf = &st->data[1].d8[4 - AD9523_TRANSFER_LEN(addr)],
			.len = AD9523_TRANSFER_LEN(addr),
		},
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);

	st->data[0].d32 = cpu_to_be32(AD9523_WRITE |
				      AD9523_CNT(AD9523_TRANSFER_LEN(addr)) |
				      AD9523_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(val);

	ret = spi_sync(st->spi, &m);


	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int ad9523_io_update(struct iio_dev *indio_dev)
{
	return ad9523_write(indio_dev, AD9523_IO_UPDATE, AD9523_IO_UPDATE_EN);
}

static int ad9523_vco_out_map(struct iio_dev *indio_dev,
			      unsigned ch, unsigned out)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	int ret;
	unsigned mask;

	switch (ch) {
	case 0 ... 3:
		ret = ad9523_read(indio_dev, AD9523_PLL1_OUTPUT_CHANNEL_CTRL);
		if (ret < 0)
			break;
		mask = AD9523_PLL1_OUTP_CH_CTRL_VCXO_SRC_SEL_CH0 << ch;
		if (out) {
			ret |= mask;
			out = 2;
		} else {
			ret &= ~mask;
		}
		ret = ad9523_write(indio_dev, AD9523_PLL1_OUTPUT_CHANNEL_CTRL, ret);
		break;
	case 4 ... 6:
		ret = ad9523_read(indio_dev, AD9523_PLL1_OUTPUT_CTRL);
		if (ret < 0)
			break;
		mask = AD9523_PLL1_OUTP_CTRL_VCO_DIV_SEL_CH4_M2 << (ch - 4);
		if (out)
			ret |= mask;
		else
			ret &= ~mask;
		ret = ad9523_write(indio_dev, AD9523_PLL1_OUTPUT_CTRL, ret);
		break;
	case 7 ... 9:
		ret = ad9523_read(indio_dev, AD9523_PLL1_OUTPUT_CHANNEL_CTRL);
		if (ret < 0)
			break;
		mask = AD9523_PLL1_OUTP_CH_CTRL_VCO_DIV_SEL_CH7_M2 << (ch - 7);
		if (out)
			ret |= mask;
		else
			ret &= ~mask;
		ret = ad9523_write(indio_dev, AD9523_PLL1_OUTPUT_CHANNEL_CTRL, ret);
		break;
	default:
		return 0;
	}

	st->vco_out_map[ch] = out;
	return ret;
}


static ssize_t ad9523_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad9523_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	unsigned long readin, tmp;
	int ret;

	ret = kstrtoul(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case AD9523_VCO1_FREQ:
		ret = ad9523_read(indio_dev, AD9523_PLL2_VCO_DIVIDER);
		if (ret < 0)
			goto out;

		tmp = st->vco_freq / readin;
		if (tmp < 3 || tmp > 5) {
			ret = -EINVAL;
			goto out;
		}

		ret &= 0xFC;
		ret |= AD9523_PLL2_VCO_DIV_M1(tmp);
		ret = ad9523_write(indio_dev, AD9523_PLL2_VCO_DIVIDER, ret);
		if (ret < 0)
			goto out;
		st->vco_out_freq[0] = st->vco_freq / tmp;
		break;
	case AD9523_VCO2_FREQ:
		ret = ad9523_read(indio_dev, AD9523_PLL2_VCO_DIVIDER);
		if (ret < 0)
			goto out;

		tmp = st->vco_freq / readin;
		if (tmp < 3 || tmp > 5) {
			ret = -EINVAL;
			goto out;
		}

		ret &= 0xCF;
		ret |= AD9523_PLL2_VCO_DIV_M2(tmp);
		ret = ad9523_write(indio_dev, AD9523_PLL2_VCO_DIVIDER, ret);
		if (ret < 0)
			goto out;
		st->vco_out_freq[1] = st->vco_freq / tmp;
		break;
	case AD9523_ALT_SRC_CH0 ... AD9523_ALT_SRC_CH9:
		ret = ad9523_vco_out_map(indio_dev, (u32)this_attr->address -
				   AD9523_ALT_SRC_CH0, !!readin);
		if (ret < 0)
			goto out;
		break;
	case AD9523_SYNC:
		ret = ad9523_read(indio_dev, AD9523_STATUS_SIGNALS);
		if (ret < 0)
			goto out;
		tmp = ret;
		tmp |= AD9523_STATUS_SIGNALS_SYNC_MAN_CTRL;
		ret = ad9523_write(indio_dev, AD9523_STATUS_SIGNALS, tmp);
		if (ret < 0)
			goto out;
		ad9523_io_update(indio_dev);
		tmp &= ~AD9523_STATUS_SIGNALS_SYNC_MAN_CTRL;
		ret = ad9523_write(indio_dev, AD9523_STATUS_SIGNALS, tmp);
		if (ret < 0)
			goto out;
		break;
	case AD9523_EEPROM:
		ret = ad9523_write(indio_dev, AD9523_EEPROM_CTRL1,
				   AD9523_EEPROM_CTRL1_EEPROM_WRITE_PROT_DIS);
		if (ret < 0)
			goto out;
		ret = ad9523_write(indio_dev, AD9523_EEPROM_CTRL2,
				   AD9523_EEPROM_CTRL2_REG2EEPROM);
		if (ret < 0)
			goto out;

		tmp = 4;
		do {
			msleep(16);
			ret = ad9523_read(indio_dev,
					  AD9523_EEPROM_DATA_XFER_STATUS);
			if (ret < 0)
				goto out;
		} while ((ret & AD9523_EEPROM_DATA_XFER_IN_PROGRESS) && tmp--);

		ret = ad9523_write(indio_dev, AD9523_EEPROM_CTRL1, 0);
		if (ret < 0)
			goto out;

		ret = ad9523_read(indio_dev, AD9523_EEPROM_ERROR_READBACK);
		if (ret < 0)
			goto out;

		if (ret & AD9523_EEPROM_ERROR_READBACK_FAIL) {
			dev_err(&indio_dev->dev, "Verify EEPROM failed");
			ret = -EIO;
		}
		break;
	default:
		ret = -ENODEV;
		goto out;
	}

	ret = ad9523_io_update(indio_dev);
out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9523_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad9523_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
 	switch ((u32)this_attr->address) {
	case AD9523_STATUS0:
		ret = ad9523_read(indio_dev, AD9523_READBACK_0);
		if (ret >= 0) {
			ret = sprintf(buf, "PLL2 reference clock:\t%s\nPLL2 feedback clock:\t%s\nVCXO:\t%s\nREF_TEST:\t%s\nREFB:\t%s\nREFA:\t%s\nPLL2 lock detect:\t%s\nPLL1 lock detect:\t%s\n",
				((ret & AD9523_READBACK_0_STAT_PLL2_REF_CLK) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_PLL2_FB_CLK) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_VCXO) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_REF_TEST) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_REFB) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_REFA) ? "OK" : "MISSING"),
				((ret & AD9523_READBACK_0_STAT_PLL2_LD) ? "OK" : "FAIL"),
				((ret & AD9523_READBACK_0_STAT_PLL1_LD) ? "OK" : "FAIL"));
		}

		break;
	case AD9523_VCO1_FREQ:
		ret = ad9523_read(indio_dev, AD9523_PLL2_VCO_DIVIDER);
		if (ret < 0)
			break;
		ret = sprintf(buf, "%lu\n", st->vco_freq / ((ret & 0x3) + 3));
		break;
	case AD9523_VCO2_FREQ:
		ret = ad9523_read(indio_dev, AD9523_PLL2_VCO_DIVIDER);
		if (ret < 0)
			break;
		ret = sprintf(buf, "%lu\n", st->vco_freq /
				(((ret >> 4) & 0x3) + 3));
		break;
	case AD9523_VCO_FREQ_AVAIL:
		ret = sprintf(buf, "%lu %lu %lu\n", st->vco_freq / 3,
			      st->vco_freq / 4, st->vco_freq / 5);
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(vco1_frequency, S_IRUGO | S_IWUSR,
 			ad9523_show,
 			ad9523_store,
 			AD9523_VCO1_FREQ);

static IIO_DEVICE_ATTR(vco2_frequency, S_IRUGO | S_IWUSR,
 			ad9523_show,
 			ad9523_store,
 			AD9523_VCO2_FREQ);

static IIO_DEVICE_ATTR(vco_frequency_available, S_IRUGO,
 			ad9523_show,
 			NULL,
 			AD9523_VCO_FREQ_AVAIL);

static IIO_DEVICE_ATTR(status, S_IRUGO,
			ad9523_show,
			NULL,
			AD9523_STATUS0);

static IIO_DEVICE_ATTR(sync, S_IWUSR,
			NULL,
			ad9523_store,
			AD9523_SYNC);

static IIO_DEVICE_ATTR(store_eeprom, S_IWUSR,
			NULL,
			ad9523_store,
			AD9523_EEPROM);

#define IIO_DEV_ATTR_CLK_SRC(_channel, _name)				\
	IIO_DEVICE_ATTR(out_altvoltage##_channel##_clk_src_##_name##_en,\
			S_IRUGO | S_IWUSR, ad9523_show, ad9523_store,	\
			AD9523_ALT_SRC_CH##_channel)

static IIO_DEV_ATTR_CLK_SRC(0, vcxo);
static IIO_DEV_ATTR_CLK_SRC(1, vcxo);
static IIO_DEV_ATTR_CLK_SRC(2, vcxo);
static IIO_DEV_ATTR_CLK_SRC(3, vcxo);
static IIO_DEV_ATTR_CLK_SRC(4, vco2);
static IIO_DEV_ATTR_CLK_SRC(5, vco2);
static IIO_DEV_ATTR_CLK_SRC(6, vco2);
static IIO_DEV_ATTR_CLK_SRC(7, vco2);
static IIO_DEV_ATTR_CLK_SRC(8, vco2);
static IIO_DEV_ATTR_CLK_SRC(9, vco2);

static struct attribute *ad9523_attributes[] = {
	&iio_dev_attr_status.dev_attr.attr,
	&iio_dev_attr_sync.dev_attr.attr,
	&iio_dev_attr_store_eeprom.dev_attr.attr,
	&iio_dev_attr_vco_frequency_available.dev_attr.attr,
	&iio_dev_attr_vco1_frequency.dev_attr.attr,
	&iio_dev_attr_vco2_frequency.dev_attr.attr,
	&iio_dev_attr_out_altvoltage0_clk_src_vcxo_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage1_clk_src_vcxo_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage2_clk_src_vcxo_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage3_clk_src_vcxo_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage4_clk_src_vco2_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage5_clk_src_vco2_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage6_clk_src_vco2_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage7_clk_src_vco2_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage8_clk_src_vco2_en.dev_attr.attr,
	&iio_dev_attr_out_altvoltage9_clk_src_vco2_en.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9523_attribute_group = {
	.attrs = ad9523_attributes,
};

static int ad9523_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = ad9523_read(indio_dev, AD9523_CHANNEL_CLOCK_DIST(chan->channel));
	mutex_unlock(&indio_dev->mlock);

	if (ret < 0)
		return ret;

	switch (m) {
	case 0:
		*val = !(ret & AD9523_CLOCK_DIST_PWR_DOWN_EN);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = st->vco_out_freq[st->vco_out_map[chan->channel]] /
			(((ret >> 8) & 0x3FF) + 1);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		*val = (ret >> 18) & 0x3F;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static int ad9523_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	unsigned reg;
	int ret, tmp;

	mutex_lock(&indio_dev->mlock);
	ret = ad9523_read(indio_dev, AD9523_CHANNEL_CLOCK_DIST(chan->channel));
	if (ret < 0)
		goto out;

	reg = ret;

	switch (mask) {
	case 0:
		if (val)
			reg &= ~AD9523_CLOCK_DIST_PWR_DOWN_EN;
		else
			reg |= AD9523_CLOCK_DIST_PWR_DOWN_EN;
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		tmp = st->vco_out_freq[st->vco_out_map[chan->channel]] / val;
		tmp = clamp(tmp, 1, 1024);
		reg &= ~(0x3FF << 8);
		reg |= AD9523_CLOCK_DIST_DIV(tmp);
		break;
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 63) {
			ret = -EINVAL;
			goto out;
		}
		reg &= ~(0x3F << 18);
		reg |= AD9523_CLOCK_DIST_DIV_PHASE(val);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	ret = ad9523_write(indio_dev, AD9523_CHANNEL_CLOCK_DIST(chan->channel),
			   reg);
	if (ret < 0)
		goto out;

	ad9523_io_update(indio_dev);
out:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad9523_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad9523_write(indio_dev, reg | AD9523_R1B, writeval);
		ad9523_io_update(indio_dev);
	} else {
		ret = ad9523_read(indio_dev, reg | AD9523_R1B);
		if (ret < 0)
			return ret;
		*readval = ret;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info ad9523_info = {
	.read_raw = &ad9523_read_raw,
	.write_raw = &ad9523_write_raw,
	.debugfs_reg_access = &ad9523_reg_access,
	.attrs = &ad9523_attribute_group,
	.driver_module = THIS_MODULE,
};

static int ad9523_setup(struct iio_dev *indio_dev)
{
	struct ad9523_state *st = iio_priv(indio_dev);
	struct ad9523_platform_data *pdata = st->pdata;
	struct ad9523_channel_spec	*chan;
	unsigned active_mask = 0;
	int ret, i;

	ret = ad9523_write(indio_dev, AD9523_SERIAL_PORT_CONFIG,
			   AD9523_SER_CONF_SOFT_RESET |
			  (st->spi->mode & SPI_3WIRE ? 0 : AD9523_SER_CONF_SDO_ACTIVE));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_READBACK_CTRL,
			  AD9523_READBACK_CTRL_READ_BUFFERED);
	if (ret < 0)
		return ret;

	ret = ad9523_io_update(indio_dev);
	if (ret < 0)
		return ret;

	/*
	 * PLL1 Setup
	 */
	ret = ad9523_write(indio_dev, AD9523_PLL1_REF_A_DIVIDER,
		pdata->refa_r_div);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_REF_B_DIVIDER,
		pdata->refb_r_div);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_FEEDBACK_DIVIDER,
		pdata->pll1_feedback_div);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_CHARGE_PUMP_CTRL,
		AD9523_PLL1_CHARGE_PUMP_CURRENT_nA(pdata->pll1_charge_pump_current_nA) |
		AD9523_PLL1_CHARGE_PUMP_MODE_NORMAL |
		AD9523_PLL1_BACKLASH_PW_MIN);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_INPUT_RECEIVERS_CTRL,
		AD_IF(refa_diff_rcv_en, AD9523_PLL1_REFA_RCV_EN) |
		AD_IF(refb_diff_rcv_en, AD9523_PLL1_REFB_RCV_EN) |
		AD_IF(osc_in_diff_en, AD9523_PLL1_OSC_IN_DIFF_EN) |
		AD_IF(osc_in_cmos_neg_inp_en, AD9523_PLL1_OSC_IN_CMOS_NEG_INP_EN) |
		AD_IF(refa_diff_rcv_en, AD9523_PLL1_REFA_DIFF_RCV_EN) |
		AD_IF(refb_diff_rcv_en, AD9523_PLL1_REFB_DIFF_RCV_EN));
	if (ret < 0)
		return ret;


	ret = ad9523_write(indio_dev, AD9523_PLL1_REF_CTRL,
		AD_IF(zd_in_diff_en, AD9523_PLL1_ZD_IN_DIFF_EN) |
		AD_IF(zd_in_cmos_neg_inp_en, AD9523_PLL1_ZD_IN_CMOS_NEG_INP_EN) |
		AD_IF(zero_delay_mode_internal_en, AD9523_PLL1_ZERO_DELAY_MODE_INT) |
		AD_IF(osc_in_feedback_en, AD9523_PLL1_OSC_IN_PLL_FEEDBACK_EN) |
		AD_IF(refa_cmos_neg_inp_en, AD9523_PLL1_REFA_CMOS_NEG_INP_EN) |
		AD_IF(refb_cmos_neg_inp_en, AD9523_PLL1_REFB_CMOS_NEG_INP_EN));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_MISC_CTRL,
		AD9523_PLL1_REFB_INDEP_DIV_CTRL_EN |
		/*AD9523_PLL1_OSC_CTRL_FAIL_VCC_BY2_EN |*/
		AD9523_PLL1_REF_MODE(pdata->ref_mode));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL1_LOOP_FILTER_CTRL,
		AD9523_PLL1_LOOP_FILTER_RZERO(pdata->pll1_loop_filter_rzero));
	if (ret < 0)
		return ret;
 	/*
	 * PLL2 Setup
	 */

	ret = ad9523_write(indio_dev, AD9523_PLL2_CHARGE_PUMP,
		AD9523_PLL2_CHARGE_PUMP_CURRENT_nA(pdata->pll2_charge_pump_current_nA));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL2_FEEDBACK_DIVIDER_AB,
		AD9523_PLL2_FB_NDIV_A_CNT(pdata->pll2_ndiv_a_cnt) |
		AD9523_PLL2_FB_NDIV_B_CNT(pdata->pll2_ndiv_b_cnt));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL2_CTRL,
		AD9523_PLL2_CHARGE_PUMP_MODE_NORMAL |
		AD9523_PLL2_BACKLASH_CTRL_EN |
		AD_IF(pll2_freq_doubler_en, AD9523_PLL2_FREQ_DOUBLER_EN));
	if (ret < 0)
		return ret;

	st->vco_freq = (pdata->vcxo_freq * (pdata->pll2_freq_doubler_en ? 2 : 1)
			/ pdata->pll2_r2_div) *
			AD9523_PLL2_FB_NDIV(pdata->pll2_ndiv_a_cnt, pdata->pll2_ndiv_b_cnt);

	ret = ad9523_write(indio_dev, AD9523_PLL2_VCO_CTRL,
		AD9523_PLL2_VCO_CALIBRATE);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL2_VCO_DIVIDER,
		AD9523_PLL2_VCO_DIV_M1(pdata->pll2_vco_diff_m1) |
		AD9523_PLL2_VCO_DIV_M2(pdata->pll2_vco_diff_m2) |
		AD_IFE(pll2_vco_diff_m1, 0, AD9523_PLL2_VCO_DIV_M1_PWR_DOWN_EN) |
		AD_IFE(pll2_vco_diff_m2, 0, AD9523_PLL2_VCO_DIV_M2_PWR_DOWN_EN));
	if (ret < 0)
		return ret;

	if (pdata->pll2_vco_diff_m1)
		st->vco_out_freq[0] = st->vco_freq / pdata->pll2_vco_diff_m1;

	if (pdata->pll2_vco_diff_m2)
		st->vco_out_freq[1] = st->vco_freq / pdata->pll2_vco_diff_m2;

	st->vco_out_freq[2] = pdata->vcxo_freq;

	ret = ad9523_write(indio_dev, AD9523_PLL2_R2_DIVIDER,
		AD9523_PLL2_R2_DIVIDER_VAL(pdata->pll2_r2_div));
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_PLL2_LOOP_FILTER_CTRL,
		AD9523_PLL2_LOOP_FILTER_CPOLE1(pdata->cpole1) |
		AD9523_PLL2_LOOP_FILTER_RZERO(pdata->rzero) |
		AD9523_PLL2_LOOP_FILTER_RPOLE2(pdata->rpole2) |
		AD_IF(rzero_bypass_en, AD9523_PLL2_LOOP_FILTER_RZERO_BYPASS_EN));
	if (ret < 0)
		return ret;

	for (i = 0; i < pdata->num_channels; i++) {
		chan = &pdata->channels[i];
		if (chan->channel_num <= AD9523_NUM_CHAN) {
			active_mask |= (1 << chan->channel_num);
			ret = ad9523_write(indio_dev,
				AD9523_CHANNEL_CLOCK_DIST(chan->channel_num),
				AD9523_CLOCK_DIST_DRIVER_MODE(chan->driver_mode) |
				AD9523_CLOCK_DIST_DIV(chan->channel_divider) |
				AD9523_CLOCK_DIST_DIV_PHASE(chan->divider_phase) |
				(chan->sync_ignore_en ? AD9523_CLOCK_DIST_IGNORE_SYNC_EN : 0) |
				(chan->divider_output_invert_en ? AD9523_CLOCK_DIST_INV_DIV_OUTPUT_EN : 0) |
				(chan->low_power_mode_en ? AD9523_CLOCK_DIST_LOW_PWR_MODE_EN : 0) |
				(chan->output_dis ? AD9523_CLOCK_DIST_PWR_DOWN_EN : 0));
			if (ret < 0)
				return ret;

			ret = ad9523_vco_out_map(indio_dev, chan->channel_num,
					   chan->use_alt_clock_src);
			if (ret < 0)
				return ret;

			st->ad9523_channels[i].type = IIO_ALTVOLTAGE;
			st->ad9523_channels[i].output = 1;
			st->ad9523_channels[i].indexed = 1;
			st->ad9523_channels[i].channel = chan->channel_num;
			st->ad9523_channels[i].extend_name = chan->extended_name;
			st->ad9523_channels[i].info_mask = IIO_CHAN_INFO_PHASE_SEPARATE_BIT |
				IIO_CHAN_INFO_FREQUENCY_SEPARATE_BIT;

		}
	}

	for (i = 0; i <= AD9523_NUM_CHAN; i++)
		if (!(active_mask & (1 << i)))
			ad9523_write(indio_dev,
				     AD9523_CHANNEL_CLOCK_DIST(i),
				     AD9523_CLOCK_DIST_DRIVER_MODE(TRISTATE) |
				     AD9523_CLOCK_DIST_PWR_DOWN_EN);

	ret = ad9523_write(indio_dev, AD9523_POWER_DOWN_CTRL, 0);
	if (ret < 0)
		return ret;

	ret = ad9523_write(indio_dev, AD9523_STATUS_SIGNALS, 0x000302);
	if (ret < 0)
		return ret;

	ret = ad9523_io_update(indio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int __devinit ad9523_probe(struct spi_device *spi)
{
	struct ad9523_platform_data *pdata = spi->dev.platform_data;
	struct iio_dev *indio_dev;
	struct ad9523_state *st;
	int ret;

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		pdata = &default_pdata;
	}

	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->reg = regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_put_reg;
	}

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (pdata->name[0] != 0) ? pdata->name :
			  spi_get_device_id(spi)->name;
	indio_dev->info = &ad9523_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->ad9523_channels;
	indio_dev->num_channels = pdata->num_channels;

	ret = ad9523_setup(indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	dev_info(&spi->dev, "probed %s\n", indio_dev->name);

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
error_put_reg:
	if (!IS_ERR(st->reg))
		regulator_put(st->reg);

	iio_free_device(indio_dev);

	return ret;
}

static int __devexit ad9523_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9523_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg)) {
		regulator_disable(reg);
		regulator_put(reg);
	}

	iio_free_device(indio_dev);

	return 0;
}

static const struct spi_device_id ad9523_id[] = {
	{"ad9523", 9523},
	{}
};

static struct spi_driver ad9523_driver = {
	.driver = {
		.name	= "ad9523",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9523_probe,
	.remove		= __devexit_p(ad9523_remove),
	.id_table	= ad9523_id,
};

static int __init ad9523_init(void)
{
	return spi_register_driver(&ad9523_driver);
}
module_init(ad9523_init);

static void __exit ad9523_exit(void)
{
	spi_unregister_driver(&ad9523_driver);
}
module_exit(ad9523_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD9523 CLOCKDIST/PLL");
MODULE_LICENSE("GPL v2");
