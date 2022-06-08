// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9467 SPI ADC driver
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */
#define DEBUG

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>


#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>

#define DCO_DEBUG

/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877:
 *   https://www.analog.com/media/en/technical-documentation/application-notes/AN-877.pdf
 */

#define AN877_ADC_REG_CHIP_PORT_CONF		0x00
#define AN877_ADC_REG_CHIP_ID			0x03
#define AN877_ADC_REG_CHIP_GRADE		0x02
#define AN877_ADC_REG_CHAN_INDEX		0x05
#define AN877_ADC_REG_TRANSFER			0xFF
#define AN877_ADC_REG_MODES			0x08
#define AN877_ADC_REG_TEST_IO			0x0D
#define AN877_ADC_REG_ADC_INPUT			0x0F
#define AN877_ADC_REG_OFFSET			0x10
#define AN877_ADC_REG_OUTPUT_MODE		0x14
#define AN877_ADC_REG_OUTPUT_ADJUST		0x15
#define AN877_ADC_REG_OUTPUT_PHASE		0x16
#define AN877_ADC_REG_OUTPUT_DELAY		0x17
#define AN877_ADC_REG_VREF			0x18
#define AN877_ADC_REG_ANALOG_INPUT		0x2C

/* AN877_ADC_REG_TEST_IO */
#define AN877_ADC_TESTMODE_OFF			0x0
#define AN877_ADC_TESTMODE_MIDSCALE_SHORT	0x1
#define AN877_ADC_TESTMODE_POS_FULLSCALE	0x2
#define AN877_ADC_TESTMODE_NEG_FULLSCALE	0x3
#define AN877_ADC_TESTMODE_ALT_CHECKERBOARD	0x4
#define AN877_ADC_TESTMODE_PN23_SEQ		0x5
#define AN877_ADC_TESTMODE_PN9_SEQ		0x6
#define AN877_ADC_TESTMODE_ONE_ZERO_TOGGLE	0x7
#define AN877_ADC_TESTMODE_USER			0x8
#define AN877_ADC_TESTMODE_BIT_TOGGLE		0x9
#define AN877_ADC_TESTMODE_SYNC			0xA
#define AN877_ADC_TESTMODE_ONE_BIT_HIGH		0xB
#define AN877_ADC_TESTMODE_MIXED_BIT_FREQUENCY	0xC
#define AN877_ADC_TESTMODE_RAMP			0xF

/* AN877_ADC_REG_TRANSFER */
#define AN877_ADC_TRANSFER_SYNC			0x1

/* AN877_ADC_REG_OUTPUT_MODE */
#define AN877_ADC_OUTPUT_MODE_OFFSET_BINARY	0x0
#define AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define AN877_ADC_OUTPUT_MODE_GRAY_CODE		0x2

/* AN877_ADC_REG_OUTPUT_PHASE */
#define AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN	0x20
#define AN877_ADC_INVERT_DCO_CLK		0x80

/* AN877_ADC_REG_OUTPUT_DELAY */
#define AN877_ADC_DCO_DELAY_ENABLE		0x80

/*
 * Analog Devices AD9467 16-Bit, 200/250 MSPS ADC
 */

#define CHIPID_AD9467			0x50
#define AD9467_DEF_OUTPUT_MODE		0x08
#define AD9467_REG_VREF_MASK		0x0F

/*
 * Analog Devices AD9643 Dual 14-Bit, 170/210/250 MSPS ADC
 */

#define CHIPID_AD9643			0x82
#define AD9643_REG_VREF_MASK		0x1F
#define AD9643_DEF_OUTPUT_MODE		0x00

/*
 * Analog Devices AD9250 Dual 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9250			0xB9
#define AD9250_REG_VREF_MASK		0x1F
#define AD9250_DEF_OUTPUT_MODE		0x00

/*
 * Analog Devices AD9683 14-Bit, 170/250 MSPS ADC, JESD204B
 */

#define CHIPID_AD9683			0xC3
#define AD9683_DEF_OUTPUT_MODE		0x00
#define AD9683_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9625 12-Bit, 2500 MSPS ADC, JESD204B
 */

#define CHIPID_AD9625			0x41
#define AD9625_DEF_OUTPUT_MODE		0x00
#define AD9625_SIGNEXTEND		(1 << 0)

/*
 * Analog Devices AD9265 16-Bit, 125/105/80 MSPS ADC
 */

#define CHIPID_AD9265			0x64
#define AD9265_DEF_OUTPUT_MODE		0x40
#define AD9265_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9434 12-Bit, 370/500 MSPS ADC
 */

#define CHIPID_AD9434			0x6A
#define AD9434_DEF_OUTPUT_MODE		0x00
#define AD9434_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9652
 */

#define CHIPID_AD9652			0xC1
#define AD9652_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9649
 */

#define CHIPID_AD9649			0x6F

/*
 * Analog Devices MACH1
 */

#define CHIPID_MACH1			0x3

enum {
	ID_AD9467,
	ID_AD9643,
	ID_AD9250,
	ID_AD9265,
	ID_AD9683,
	ID_AD9625,
	ID_AD9434,
	ID_AD9652,
	ID_AD9649,
	ID_MACH1,
};

static int ad9467_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = 0x80 | (reg >> 8);
		buf[1] = reg & 0xFF;

		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, buf[2], ret);

		if (ret < 0)
			return ret;

		return buf[2];
	}
	return -ENODEV;
}

static int ad9467_spi_write(struct spi_device *spi, unsigned int reg,
			    unsigned int val)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;
	ret = spi_write_then_read(spi, buf, 3, NULL, 0);

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

	if (ret < 0)
		return ret;

	if ((reg == AN877_ADC_REG_TRANSFER) && (val == AN877_ADC_TRANSFER_SYNC) &&
	    (conv->chip_info->id == CHIPID_AD9265))
		return ad9467_spi_write(spi, AN877_ADC_REG_TRANSFER, 0);

	return 0;
}

static int ad9467_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	if (readval == NULL) {
		ret = ad9467_spi_write(spi, reg, writeval);
		if (conv->chip_info->id != CHIPID_MACH1)
		  ad9467_spi_write(spi, AN877_ADC_REG_TRANSFER, AN877_ADC_TRANSFER_SYNC);
		return ret;
	} else {
		ret = ad9467_spi_read(spi, reg);
		if (ret < 0)
			return ret;
		*readval = ret;
	}

	return 0;
}

static int ad9467_outputmode_set(struct spi_device *spi, unsigned mode)
{
	int ret;

	ret = ad9467_spi_write(spi, AN877_ADC_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;
	ret = ad9467_spi_write(spi, AN877_ADC_REG_TEST_IO, AN877_ADC_TESTMODE_OFF);
	if (ret < 0)
		return ret;

	return ad9467_spi_write(spi, AN877_ADC_REG_TRANSFER, AN877_ADC_TRANSFER_SYNC);
}

static int ad9467_testmode_set(struct iio_dev *indio_dev,
			       unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad9467_spi_write(conv->spi, AN877_ADC_REG_CHAN_INDEX, 1 << chan);
	ad9467_spi_write(conv->spi, AN877_ADC_REG_TEST_IO, mode);
	ad9467_spi_write(conv->spi, AN877_ADC_REG_CHAN_INDEX, 0x3);
	ad9467_spi_write(conv->spi, AN877_ADC_REG_TRANSFER, AN877_ADC_TRANSFER_SYNC);
	conv->testmode[chan] = mode;
	return 0;
}

static unsigned int ad9467_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return AN877_ADC_TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return AN877_ADC_TESTMODE_PN23_SEQ;
	default:
		return AN877_ADC_TESTMODE_OFF;
	}
}

static int ad9467_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9467_pnsel_to_testmode(sel);
	int ret;

	if (mode == AN877_ADC_TESTMODE_OFF)
		ret = ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_MODE,
				       conv->adc_output_mode);
	else
		ret = ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_MODE,
				       conv->
				       adc_output_mode &
				       ~AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT);

	if (ret < 0)
		return ret;

	return ad9467_testmode_set(indio_dev, chan, mode);
}

static int ad9467_calibrate(struct iio_dev *indio_dev, unsigned chan,
		bool dco, unsigned dco_en, unsigned nb_lanes)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, val, cnt, start, max_start, max_cnt;
	unsigned stat, inv_range = 0, do_inv, lane,
		 chan_ctrl0, chan_ctrl1, max_val = dco ? 32 : 31;
	unsigned char err_field[66];

	ret = ad9467_outputmode_set(conv->spi,
			conv->adc_output_mode & ~AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT);
	if (ret < 0)
		return ret;

	chan_ctrl0 = axiadc_read(st, ADI_REG_CHAN_CNTRL(0));
	chan_ctrl1 = axiadc_read(st, ADI_REG_CHAN_CNTRL(1));

	do {
		if (dco && conv->chip_info->id != CHIPID_AD9652) {
			ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_PHASE,
					AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN | (inv_range ?
						AN877_ADC_INVERT_DCO_CLK : 0));
		} else if (!dco) {
			unsigned reg_cntrl = axiadc_read(st, ADI_REG_CNTRL);

			if (inv_range)
				reg_cntrl |= ADI_DDR_EDGESEL;
			else
				reg_cntrl &= ~ADI_DDR_EDGESEL;
			axiadc_write(st, ADI_REG_CNTRL, reg_cntrl);
		}

		if (chan == 2) {
			ad9467_testmode_set(indio_dev, 1, AN877_ADC_TESTMODE_PN23_SEQ);
			axiadc_write(st, ADI_REG_CHAN_CNTRL(1), ADI_ENABLE);
			axiadc_set_pnsel(st, 1, ADC_PN23A);
			axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);
		}

		ad9467_testmode_set(indio_dev, 0, AN877_ADC_TESTMODE_PN9_SEQ);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(0), ADI_ENABLE);
		axiadc_set_pnsel(st, 0, ADC_PN9);
		axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);

		for (val = 0; val <= max_val; val++) {
			if (dco) {
				ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_DELAY,
						val > 0 ? ((val - 1) | dco_en) : 0);
				ad9467_spi_write(conv->spi, AN877_ADC_REG_TRANSFER,
						AN877_ADC_TRANSFER_SYNC);
				ad9467_spi_read(conv->spi, AN877_ADC_REG_OUTPUT_DELAY);
			} else {
				for (lane = 0; lane < nb_lanes; lane++) {
					axiadc_idelay_set(st, lane, val);
				}
			}

			axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);
			if (chan == 2)
				axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);

			mdelay(1);

			stat = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
			if (chan == 2)
				stat |= axiadc_read(st, ADI_REG_CHAN_STATUS(1));

			err_field[val + (inv_range * (max_val + 1))] =
			    ! !(stat & (ADI_PN_ERR | ADI_PN_OOS));
		}

		for (val = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		     val <= (max_val + (inv_range * (max_val + 1))); val++) {
			if (err_field[val] == 0) {
				if (start == -1)
					start = val;
				cnt++;
			} else {
				if (cnt > max_cnt) {
					max_cnt = cnt;
					max_start = start;
				}
				start = -1;
				cnt = 0;
			}
		}

		if (cnt > max_cnt) {
			max_cnt = cnt;
			max_start = start;
		}

		if ((inv_range == 0) && ((max_cnt < 3) || (err_field[max_val] == 0))) {
			do_inv = 1;
			inv_range = 1;
		} else {
			do_inv = 0;
		}

	} while (do_inv);

	val = max_start + (max_cnt / 2);

#ifdef DCO_DEBUG
	for (cnt = 0; cnt <= (max_val + (inv_range * (max_val + 1))); cnt++) {
		if (cnt == val)
			printk("|");
		else
			printk("%c", err_field[cnt] ? '-' : 'o');
		if (cnt == max_val)
			printk("\n");
	}
#endif
	if (val > max_val) {
		val -= max_val + 1;
		if (dco && conv->chip_info->id != CHIPID_AD9652) {
			ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_PHASE,
				 AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN | AN877_ADC_INVERT_DCO_CLK);
		} else if (!dco) {
			unsigned reg_cntrl = axiadc_read(st, ADI_REG_CNTRL);
			reg_cntrl |= ADI_DDR_EDGESEL;
			axiadc_write(st, ADI_REG_CNTRL, reg_cntrl);
		}
		cnt = 1;
	} else {
		if (dco && conv->chip_info->id != CHIPID_AD9652) {
			ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_PHASE,
				 AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN);
		} else if (!dco) {
			unsigned reg_cntrl = axiadc_read(st, ADI_REG_CNTRL);
			reg_cntrl &= ~ADI_DDR_EDGESEL;
			axiadc_write(st, ADI_REG_CNTRL, reg_cntrl);
		}
		cnt = 0;
	}

#ifdef DCO_DEBUG
	if (dco)
		printk(" %s DCO 0x%X CLK %lu Hz\n", cnt ? "INVERT" : "",
				val > 0 ? ((val - 1) | dco_en) : 0,
				conv->adc_clk);
	else
		printk(" %s IDELAY 0x%x\n", cnt ? "INVERT" : "", val);
#endif

	ad9467_testmode_set(indio_dev, 0, AN877_ADC_TESTMODE_OFF);
	ad9467_testmode_set(indio_dev, 1, AN877_ADC_TESTMODE_OFF);
	if (dco) {
		ad9467_spi_write(conv->spi, AN877_ADC_REG_OUTPUT_DELAY,
				val > 0 ? ((val - 1) | dco_en) : 0);
		ad9467_spi_write(conv->spi, AN877_ADC_REG_TRANSFER, AN877_ADC_TRANSFER_SYNC);
	} else {
		for (lane = 0; lane < nb_lanes; lane++) {
			axiadc_idelay_set(st, lane, val);
		}
	}

	axiadc_write(st, ADI_REG_CHAN_CNTRL(0), chan_ctrl0);
	axiadc_write(st, ADI_REG_CHAN_CNTRL(1), chan_ctrl1);

	ret = ad9467_outputmode_set(conv->spi, conv->adc_output_mode);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9467_idelay_calibrate(struct iio_dev *indio_dev, unsigned chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned nb_lanes;

	switch (conv->chip_info->id) {
	case CHIPID_AD9467:
		nb_lanes = 8;
		break;
	case CHIPID_AD9434:
		nb_lanes = 6;
		break;
	default:
		return 0;
	}

	return ad9467_calibrate(indio_dev, chan, false, false, nb_lanes);
}

static int ad9467_dco_calibrate(struct iio_dev *indio_dev, unsigned chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned dco_en;

	switch (conv->chip_info->id) {
	case CHIPID_AD9467:
	case CHIPID_AD9250:
	case CHIPID_AD9683:
	case CHIPID_AD9625:
	case CHIPID_AD9434:
	case CHIPID_AD9649:
		return 0;
	case CHIPID_AD9265:
	case CHIPID_AD9652:
		dco_en = 0;
		break;
	default:
		dco_en = AN877_ADC_DCO_DELAY_ENABLE;
	}

	return ad9467_calibrate(indio_dev, chan, true, dco_en, 0);
}

static const int ad9265_scale_table[][2] = {
	{1250, 0x00}, {1500, 0x40}, {1750, 0x80}, {2000, 0xC0},
};

static const int ad9467_scale_table[][2] = {
	{2000, 0}, {2100, 6}, {2200, 7},
	{2300, 8}, {2400, 9}, {2500, 10},
};

static const int ad9643_scale_table[][2] = {
	{2087, 0x0F}, {2065, 0x0E}, {2042, 0x0D}, {2020, 0x0C}, {1997, 0x0B},
	{1975, 0x0A}, {1952, 0x09}, {1930, 0x08}, {1907, 0x07}, {1885, 0x06},
	{1862, 0x05}, {1840, 0x04}, {1817, 0x03}, {1795, 0x02}, {1772, 0x01},
	{1750, 0x00}, {1727, 0x1F}, {1704, 0x1E}, {1681, 0x1D}, {1658, 0x1C},
	{1635, 0x1B}, {1612, 0x1A}, {1589, 0x19}, {1567, 0x18}, {1544, 0x17},
	{1521, 0x16}, {1498, 0x15}, {1475, 0x14}, {1452, 0x13}, {1429, 0x12},
	{1406, 0x11}, {1383, 0x10},
};

static const int ad9434_scale_table[][2] = {
	{1600, 0x1C}, {1580, 0x1D}, {1550, 0x1E}, {1520, 0x1F}, {1500, 0x00},
	{1470, 0x01}, {1440, 0x02}, {1420, 0x03}, {1390, 0x04}, {1360, 0x05},
	{1340, 0x06}, {1310, 0x07}, {1280, 0x08}, {1260, 0x09}, {1230, 0x0A},
	{1200, 0x0B}, {1180, 0x0C},
};

static const int ad9652_scale_table[][2] = {
	{1250, 0}, {1125, 1}, {1200, 2}, {1250, 3}, {1000, 5},
};

static const int ad9649_scale_table[][2] = {
	{2000, 0},
};

static const int mach1_scale_table[][2] = {
	{6000, 0},
};

static void ad9467_scale(struct axiadc_converter *conv, int index,
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

static const char *const testmodes[] = {
	[AN877_ADC_TESTMODE_OFF] = "off",
	[AN877_ADC_TESTMODE_MIDSCALE_SHORT] = "midscale_short",
	[AN877_ADC_TESTMODE_POS_FULLSCALE] = "pos_fullscale",
	[AN877_ADC_TESTMODE_NEG_FULLSCALE] = "neg_fullscale",
	[AN877_ADC_TESTMODE_ALT_CHECKERBOARD] = "checkerboard",
	[AN877_ADC_TESTMODE_PN23_SEQ] = "pn_long",
	[AN877_ADC_TESTMODE_PN9_SEQ] = "pn_short",
	[AN877_ADC_TESTMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[AN877_ADC_TESTMODE_USER] = "user",
	[AN877_ADC_TESTMODE_BIT_TOGGLE] = "bit_toggle",
	[AN877_ADC_TESTMODE_SYNC] = "sync",
	[AN877_ADC_TESTMODE_ONE_BIT_HIGH] = "one_bit_high",
	[AN877_ADC_TESTMODE_MIXED_BIT_FREQUENCY] = "mixed_bit_frequency",
	[AN877_ADC_TESTMODE_RAMP] = "ramp",
};

static bool ad9467_valid_test_mode(struct axiadc_converter *conv,
	unsigned int mode)
{
	if (!testmodes[mode])
		return false;

	/*
	 * All converters that support the ramp testmode have a gap between USER and
	 * RAMP.
	 */
	if (conv->chip_info->max_testmode == AN877_ADC_TESTMODE_RAMP &&
	    mode > AN877_ADC_TESTMODE_USER && mode < AN877_ADC_TESTMODE_RAMP)
		return false;

	return true;
}

static ssize_t ad9467_show_scale_available(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int scale[2];
	int i, len = 0;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9467_scale(conv, i, &scale[0], &scale[1]);
		len += sprintf(buf + len, "%u.%06u ", scale[0], scale[1]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}

static ssize_t ad9467_testmode_mode_available(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	size_t len = 0;
	int i;

	for (i = 0; i <= conv->chip_info->max_testmode; ++i) {
		if (ad9467_valid_test_mode(conv, i))
			len += sprintf(buf + len, "%s ", testmodes[i]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

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
		if (ad9467_valid_test_mode(conv, i) &&
		    sysfs_streq(buf, testmodes[i])) {
			mode = i;
			break;
		}
	}

	mutex_lock(&indio_dev->mlock);
	ret = ad9467_testmode_set(indio_dev, chan->channel, mode);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	{
	 .name = "test_mode",
	 .read = axiadc_testmode_read,
	 .write = axiadc_testmode_write,
	 },
	{
	 .name = "test_mode_available",
	 .read = ad9467_testmode_mode_available,
	 .shared = true,
	 },
	{
	 .name = "scale_available",
	 .read = ad9467_show_scale_available,
	 .shared = true,
	 },
	{},
};

#define AIM_CHAN(_chan, _si, _bits, _sign)				\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
			BIT(IIO_CHAN_INFO_CALIBPHASE) |			\
			BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY), \
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,					\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = 0,					\
	  },								\
	}

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = _shift,				\
	  },								\
	}

#define AIM_CHAN_NOCALIB_32(_chan, _si, _bits, _sign, _shift)		\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | 	\
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 32,				\
			.shift = _shift,				\
	  },								\
	}
static const struct axiadc_chip_info ad9467_chip_tbl[] = {
	[ID_AD9467] = {
		.name = "AD9467",
		.id = CHIPID_AD9467,
		.max_rate = 250000000UL,
		.scale_table = ad9467_scale_table,
		.num_scales = ARRAY_SIZE(ad9467_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 1,
		.channel[0] = AIM_CHAN(0, 0, 16, 'S'),
	},
	[ID_AD9643] = {
		.name = "AD9643",
		.id = CHIPID_AD9643,
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_RAMP,
		.num_channels = 2,
		.channel[0] = AIM_CHAN(0, 0, 14, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 14, 'S'),
	},
	[ID_AD9250] = {
		.name = "AD9250",
		.id = CHIPID_AD9250,
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_RAMP,
		.num_channels = 2,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
		.channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'S', 0),
	},
	[ID_AD9683] = {
		.name = "AD9683",
		.id = CHIPID_AD9683,
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_RAMP,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
	},
	[ID_AD9625] = {
		.name = "AD9625",
		.id = CHIPID_AD9625,
		.max_rate = 2500000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_RAMP,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 'S', 0),
	},
	[ID_AD9265] = {
		.name = "AD9265",
		.id = CHIPID_AD9265,
		.max_rate = 125000000UL,
		.scale_table = ad9265_scale_table,
		.num_scales = ARRAY_SIZE(ad9265_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 'S', 0),
	},
	[ID_AD9434] = {
		.name = "AD9434",
		.id = CHIPID_AD9434,
		.max_rate = 500000000UL,
		.scale_table = ad9434_scale_table,
		.num_scales = ARRAY_SIZE(ad9434_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_USER,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 'S', 0),
	},
	[ID_AD9652] = {
		.name = "AD9652",
		.id = CHIPID_AD9652,
		.max_rate = 310000000UL,
		.scale_table = ad9652_scale_table,
		.num_scales = ARRAY_SIZE(ad9652_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_ONE_ZERO_TOGGLE,
		.num_channels = 2,
		.channel[0] = AIM_CHAN(0, 0, 16, 'S'),
		.channel[1] = AIM_CHAN(1, 1, 16, 'S'),
	},
	[ID_AD9649] = {
		.name = "AD9649",
		.id = CHIPID_AD9649,
		.max_rate = 80000000UL,
		.scale_table = ad9649_scale_table,
		.num_scales = ARRAY_SIZE(ad9649_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_MIXED_BIT_FREQUENCY,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
	},
	[ID_MACH1] = {
		.name = "MACH1",
		.id = CHIPID_MACH1,
		.max_rate = 40000000UL,
		.scale_table = mach1_scale_table,
		.num_scales = ARRAY_SIZE(mach1_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_MIXED_BIT_FREQUENCY,
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB_32(0, 0, 20, 'S', 0),
	},
};

static int ad9250_setup(struct spi_device *spi, unsigned m, unsigned l)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct clk *clk;
	int ret;
	unsigned pll_stat;
	static int sel = 0;

	clk = devm_clk_get(&spi->dev, "adc_clk");
	if (!IS_ERR(clk)) {
		ret = clk_prepare_enable(clk);
		if (ret < 0)
			return ret;
		conv->adc_clk = clk_get_rate_scaled(clk, &conv->adc_clkscale);
	}

	ret = ad9467_spi_write(spi, 0x5f, (0x16 | 0x1));	// trail bits, ilas normal & pd
	ret |= ad9467_spi_write(spi, 0x5e, m << 4 | l);	// m=2, l=2
	ret |= ad9467_spi_write(spi, 0x66, sel++);	// lane id
	ret |= ad9467_spi_write(spi, 0x67, sel++);	// lane id
	ret |= ad9467_spi_write(spi, 0x6e, 0x80 | (l - 1));	// scr, 2-lane
	ret |= ad9467_spi_write(spi, 0x70, 0x1f);	// no. of frames per multi frame
	ret |= ad9467_spi_write(spi, 0x3a, 0x01);	// sysref enabled
	ret |= ad9467_spi_write(spi, 0x3a, 0x13);	// sysref enabled
	ret |= ad9467_spi_write(spi, 0x5f, (0x16 | 0x0));	// enable
	ret |= ad9467_spi_write(spi, 0x14, 0x00);	// offset binary
	ret |= ad9467_spi_write(spi, 0x0d, 0x00);	// test patterns

	ret |= ad9467_spi_write(spi, 0xff, 0x01);
	ret |= ad9467_spi_write(spi, 0xff, 0x00);

	ret = clk_prepare_enable(conv->clk);
	if (ret < 0)
		return ret;

	conv->clk = clk;

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "AD9250 PLL %s, JESD204B Link %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED",
		 pll_stat & 0x01 ? "Ready" : "Fail");

	conv->sample_rate_read_only = true;

	return ret;
}

static int ad9625_setup(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	unsigned long lane_rate_kHz;
	unsigned pll_stat;
	int ret;

	conv->sysref_clk = devm_clk_get(&spi->dev, "adc_sysref");
	if (IS_ERR(conv->sysref_clk) && PTR_ERR(conv->sysref_clk) != -ENOENT)
		return PTR_ERR(conv->sysref_clk);

	if (!IS_ERR(conv->sysref_clk)) {
		ret = clk_prepare_enable(conv->sysref_clk);
		if (ret < 0)
			return ret;
	}

	/* for JESD converters conv->clk is JESD/Data clock */
	conv->lane_clk = conv->clk;

	conv->clk = devm_clk_get(&spi->dev, "adc_clk");
	if (IS_ERR(conv->clk) && PTR_ERR(conv->clk) != -ENOENT)
		return PTR_ERR(conv->clk);

	if (!IS_ERR(conv->clk)) {
		ret = clk_prepare_enable(conv->clk);
		if (ret < 0)
			return ret;
		of_clk_get_scale(spi->dev.of_node, "adc_clk", &conv->adc_clkscale);
		conv->adc_clk = clk_get_rate_scaled(conv->clk, &conv->adc_clkscale);
	}

	ret = ad9467_spi_write(spi, 0x000, 0x24);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	mdelay(10);
	ret |= ad9467_spi_write(spi, 0x008, 0x00);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);
	ret |= ad9467_spi_write(spi, 0x05f, 0x15);
	ret |= ad9467_spi_write(spi, 0x080, 0x00);
	ret |= ad9467_spi_write(spi, 0x120, 0x11);
	ret |= ad9467_spi_write(spi, 0x00d, 0x00);
	ret |= ad9467_spi_write(spi, 0x014, 0x00);
	ret |= ad9467_spi_write(spi, 0x015, 0x10);
	ret |= ad9467_spi_write(spi, 0x05f, 0x14);
	ret |= ad9467_spi_write(spi, 0x0ff, 0x01);

	mdelay(10);

	/* 16bits * 10bits / 8bits / 8lanes / 1000Hz = 1 / 400 */
	lane_rate_kHz = DIV_ROUND_CLOSEST(conv->adc_clk, 400);

	ret = clk_set_rate(conv->lane_clk, lane_rate_kHz);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret == -EIO) /* Sync issue on the dual FMCADC5 */
		ret = 0;

	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	mdelay(10);

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "AD9625 PLL %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED");

	conv->sample_rate_read_only = true;

	return ret;
}

static int mach1_write_sram_loc(struct spi_device *spi,
		unsigned char b0, unsigned char b1, unsigned char b2,
		unsigned char b3, unsigned char b4)
{
	int ret = 0;
	ret += ad9467_spi_write(spi, 0x45, b0);
	ret += ad9467_spi_write(spi, 0x46, b1);
	ret += ad9467_spi_write(spi, 0x47, b2);
	ret += ad9467_spi_write(spi, 0x48, b3);
	ret += ad9467_spi_write(spi, 0x49, b4);
	return ret;
}

static int mach1_init_sram(struct spi_device *spi)
{

	int ret = 0;
	ret += mach1_write_sram_loc(spi, 0x0  ,0x0  ,0xf0 ,0xb0 ,0x22);
	ret += mach1_write_sram_loc(spi, 0x1  ,0x0  ,0x0  ,0xf  ,0xc );
	ret += mach1_write_sram_loc(spi, 0x2  ,0x1  ,0xf0 ,0xe0 ,0x1e);
	ret += mach1_write_sram_loc(spi, 0x3  ,0x1  ,0x0  ,0xff ,0x3b);
	ret += mach1_write_sram_loc(spi, 0x4  ,0x82 ,0xf0 ,0xa0 ,0x3d);
	ret += mach1_write_sram_loc(spi, 0x5  ,0x82 ,0x0  ,0xaf ,0x3 );
	ret += mach1_write_sram_loc(spi, 0x6  ,0x83 ,0xf0 ,0x50 ,0x19);
	ret += mach1_write_sram_loc(spi, 0x7  ,0x83 ,0x0  ,0x3f ,0x30);
	ret += mach1_write_sram_loc(spi, 0x8  ,0x4  ,0xf1 ,0x60 ,0x26);
	ret += mach1_write_sram_loc(spi, 0x9  ,0x4  ,0x1  ,0x6f ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xa  ,0x5  ,0xf1 ,0x50 ,0x36);
	ret += mach1_write_sram_loc(spi, 0xb  ,0x5  ,0x1  ,0x3f ,0x7 );
	ret += mach1_write_sram_loc(spi, 0xc  ,0x86 ,0xf1 ,0x80 ,0x13);
	ret += mach1_write_sram_loc(spi, 0xd  ,0x86 ,0x1  ,0x2f ,0x29);
	ret += mach1_write_sram_loc(spi, 0xe  ,0x87 ,0xf1 ,0x70 ,0x15);
	ret += mach1_write_sram_loc(spi, 0xf  ,0x87 ,0x1  ,0xf  ,0xa );
	ret += mach1_write_sram_loc(spi, 0x10 ,0x8  ,0xf2 ,0xb0 ,0x3d);
	ret += mach1_write_sram_loc(spi, 0x11 ,0x8  ,0x2  ,0xcf ,0x3a);
	ret += mach1_write_sram_loc(spi, 0x12 ,0x9  ,0xf2 ,0xe0 ,0x1b);
	ret += mach1_write_sram_loc(spi, 0x13 ,0x9  ,0x2  ,0x1f ,0xe );
	ret += mach1_write_sram_loc(spi, 0x14 ,0x8a ,0xf2 ,0x20 ,0x2a);
	ret += mach1_write_sram_loc(spi, 0x15 ,0x8a ,0x2  ,0x1f ,0x31);
	ret += mach1_write_sram_loc(spi, 0x16 ,0x8b ,0xf2 ,0xd0 ,0xa );
	ret += mach1_write_sram_loc(spi, 0x17 ,0x8b ,0x2  ,0xdf ,0x17);
	ret += mach1_write_sram_loc(spi, 0x18 ,0xc  ,0xf3 ,0x90 ,0x27);
	ret += mach1_write_sram_loc(spi, 0x19 ,0xc  ,0x3  ,0x6f ,0x1c);
	ret += mach1_write_sram_loc(spi, 0x1a ,0xd  ,0xf3 ,0x50 ,0x23);
	ret += mach1_write_sram_loc(spi, 0x1b ,0xd  ,0x3  ,0x9f ,0x35);
	ret += mach1_write_sram_loc(spi, 0x1c ,0x8e ,0xf3 ,0x70 ,0x2c);
	ret += mach1_write_sram_loc(spi, 0x1d ,0x8e ,0x3  ,0x1f ,0x4 );
	ret += mach1_write_sram_loc(spi, 0x1e ,0x8f ,0xf3 ,0x90 ,0x12);
	ret += mach1_write_sram_loc(spi, 0x1f ,0x8f ,0x3  ,0xbf ,0x0 );
	ret += mach1_write_sram_loc(spi, 0x20 ,0x10 ,0xf4 ,0x30 ,0x11);
	ret += mach1_write_sram_loc(spi, 0x21 ,0x10 ,0x4  ,0x8f ,0x2d);
	ret += mach1_write_sram_loc(spi, 0x22 ,0x11 ,0xf4 ,0x90 ,0x4 );
	ret += mach1_write_sram_loc(spi, 0x23 ,0x11 ,0x4  ,0xf  ,0x15);
	ret += mach1_write_sram_loc(spi, 0x24 ,0x92 ,0xf4 ,0x70 ,0x3a);
	ret += mach1_write_sram_loc(spi, 0x25 ,0x92 ,0x4  ,0xff ,0x0 );
	ret += mach1_write_sram_loc(spi, 0x26 ,0x93 ,0xf4 ,0xb0 ,0x37);
	ret += mach1_write_sram_loc(spi, 0x27 ,0x93 ,0x4  ,0x4f ,0x27);
	ret += mach1_write_sram_loc(spi, 0x28 ,0x14 ,0xf5 ,0x70 ,0xf );
	ret += mach1_write_sram_loc(spi, 0x29 ,0x14 ,0x5  ,0x1f ,0xb );
	ret += mach1_write_sram_loc(spi, 0x2a ,0x15 ,0xf5 ,0x80 ,0x28);
	ret += mach1_write_sram_loc(spi, 0x2b ,0x15 ,0x5  ,0x2f ,0x30);
	ret += mach1_write_sram_loc(spi, 0x2c ,0x96 ,0xf5 ,0x0  ,0x23);
	ret += mach1_write_sram_loc(spi, 0x2d ,0x96 ,0x5  ,0xef ,0x3d);
	ret += mach1_write_sram_loc(spi, 0x2e ,0x97 ,0xf5 ,0x20 ,0x1b);
	ret += mach1_write_sram_loc(spi, 0x2f ,0x97 ,0x5  ,0x4f ,0x1c);
	ret += mach1_write_sram_loc(spi, 0x30 ,0x18 ,0xf6 ,0x40 ,0x3e);
	ret += mach1_write_sram_loc(spi, 0x31 ,0x18 ,0x6  ,0xaf ,0x28);
	ret += mach1_write_sram_loc(spi, 0x32 ,0x19 ,0xf6 ,0x40 ,0x39);
	ret += mach1_write_sram_loc(spi, 0x33 ,0x19 ,0x6  ,0xff ,0x22);
	ret += mach1_write_sram_loc(spi, 0x34 ,0x9a ,0xf6 ,0xc0 ,0x14);
	ret += mach1_write_sram_loc(spi, 0x35 ,0x9a ,0x6  ,0x3f ,0x26);
	ret += mach1_write_sram_loc(spi, 0x36 ,0x9b ,0xf6 ,0xc0 ,0x2f);
	ret += mach1_write_sram_loc(spi, 0x37 ,0x9b ,0x6  ,0x6f ,0x7 );
	ret += mach1_write_sram_loc(spi, 0x38 ,0x1c ,0xf7 ,0xd0 ,0x34);
	ret += mach1_write_sram_loc(spi, 0x39 ,0x1c ,0x7  ,0xdf ,0x31);
	ret += mach1_write_sram_loc(spi, 0x3a ,0x1d ,0xf7 ,0xf0 ,0x11);
	ret += mach1_write_sram_loc(spi, 0x3b ,0x1d ,0x7  ,0x5f ,0x18);
	ret += mach1_write_sram_loc(spi, 0x3c ,0x9e ,0xf7 ,0xa0 ,0xe );
	ret += mach1_write_sram_loc(spi, 0x3d ,0x9e ,0x7  ,0xef ,0x1c);
	ret += mach1_write_sram_loc(spi, 0x3e ,0x9f ,0xf7 ,0x80 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0x3f ,0x9f ,0x7  ,0xcf ,0x9 );
	ret += mach1_write_sram_loc(spi, 0x40 ,0x20 ,0xf0 ,0x20 ,0x25);
	ret += mach1_write_sram_loc(spi, 0x41 ,0x20 ,0x0  ,0xef ,0xb );
	ret += mach1_write_sram_loc(spi, 0x42 ,0x21 ,0xf0 ,0xc0 ,0xc );
	ret += mach1_write_sram_loc(spi, 0x43 ,0x21 ,0x0  ,0x2f ,0x14);
	ret += mach1_write_sram_loc(spi, 0x44 ,0xa2 ,0xf0 ,0xc0 ,0x1 );
	ret += mach1_write_sram_loc(spi, 0x45 ,0xa2 ,0x0  ,0xdf ,0x1e);
	ret += mach1_write_sram_loc(spi, 0x46 ,0xa3 ,0xf0 ,0xa0 ,0x2c);
	ret += mach1_write_sram_loc(spi, 0x47 ,0xa3 ,0x0  ,0xaf ,0x22);
	ret += mach1_write_sram_loc(spi, 0x48 ,0x24 ,0xf1 ,0xb0 ,0x11);
	ret += mach1_write_sram_loc(spi, 0x49 ,0x24 ,0x1  ,0xbf ,0x3c);
	ret += mach1_write_sram_loc(spi, 0x4a ,0x25 ,0xf1 ,0x70 ,0x18);
	ret += mach1_write_sram_loc(spi, 0x4b ,0x25 ,0x1  ,0x2f ,0x6 );
	ret += mach1_write_sram_loc(spi, 0x4c ,0xa6 ,0xf1 ,0xb0 ,0x2b);
	ret += mach1_write_sram_loc(spi, 0x4d ,0xa6 ,0x1  ,0xef ,0x3a);
	ret += mach1_write_sram_loc(spi, 0x4e ,0xa7 ,0xf1 ,0x10 ,0x37);
	ret += mach1_write_sram_loc(spi, 0x4f ,0xa7 ,0x1  ,0xff ,0x33);
	ret += mach1_write_sram_loc(spi, 0x50 ,0x28 ,0xf2 ,0x60 ,0x1 );
	ret += mach1_write_sram_loc(spi, 0x51 ,0x28 ,0x2  ,0xf  ,0x3f);
	ret += mach1_write_sram_loc(spi, 0x52 ,0x29 ,0xf2 ,0x50 ,0x26);
	ret += mach1_write_sram_loc(spi, 0x53 ,0x29 ,0x2  ,0xff ,0x19);
	ret += mach1_write_sram_loc(spi, 0x54 ,0xaa ,0xf2 ,0x20 ,0xf );
	ret += mach1_write_sram_loc(spi, 0x55 ,0xaa ,0x2  ,0x5f ,0x2d);
	ret += mach1_write_sram_loc(spi, 0x56 ,0xab ,0xf2 ,0x80 ,0x34);
	ret += mach1_write_sram_loc(spi, 0x57 ,0xab ,0x2  ,0xcf ,0x1f);
	ret += mach1_write_sram_loc(spi, 0x58 ,0x2c ,0xf3 ,0x10 ,0x21);
	ret += mach1_write_sram_loc(spi, 0x59 ,0x2c ,0x3  ,0x7f ,0x11);
	ret += mach1_write_sram_loc(spi, 0x5a ,0x2d ,0xf3 ,0x90 ,0x14);
	ret += mach1_write_sram_loc(spi, 0x5b ,0x2d ,0x3  ,0xf  ,0x39);
	ret += mach1_write_sram_loc(spi, 0x5c ,0xae ,0xf3 ,0x10 ,0x4 );
	ret += mach1_write_sram_loc(spi, 0x5d ,0xae ,0x3  ,0x8f ,0xa );
	ret += mach1_write_sram_loc(spi, 0x5e ,0xaf ,0xf3 ,0x30 ,0x2b);
	ret += mach1_write_sram_loc(spi, 0x5f ,0xaf ,0x3  ,0xdf ,0x30);
	ret += mach1_write_sram_loc(spi, 0x60 ,0x30 ,0xf4 ,0x50 ,0x3b);
	ret += mach1_write_sram_loc(spi, 0x61 ,0x30 ,0x4  ,0xaf ,0x31);
	ret += mach1_write_sram_loc(spi, 0x62 ,0x31 ,0xf4 ,0x70 ,0xa );
	ret += mach1_write_sram_loc(spi, 0x63 ,0x31 ,0x4  ,0x5f ,0x10);
	ret += mach1_write_sram_loc(spi, 0x64 ,0xb2 ,0xf4 ,0xd0 ,0x15);
	ret += mach1_write_sram_loc(spi, 0x65 ,0xb2 ,0x4  ,0xf  ,0x3e);
	ret += mach1_write_sram_loc(spi, 0x66 ,0xb3 ,0xf4 ,0xb0 ,0x36);
	ret += mach1_write_sram_loc(spi, 0x67 ,0xb3 ,0x4  ,0x9f ,0x25);
	ret += mach1_write_sram_loc(spi, 0x68 ,0x34 ,0xf5 ,0xc0 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0x69 ,0x34 ,0x5  ,0x4f ,0x6 );
	ret += mach1_write_sram_loc(spi, 0x6a ,0x35 ,0xf5 ,0x60 ,0x23);
	ret += mach1_write_sram_loc(spi, 0x6b ,0x35 ,0x5  ,0xff ,0x1c);
	ret += mach1_write_sram_loc(spi, 0x6c ,0xb6 ,0xf5 ,0x30 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0x6d ,0xb6 ,0x5  ,0x8f ,0x1b);
	ret += mach1_write_sram_loc(spi, 0x6e ,0xb7 ,0xf5 ,0x60 ,0x28);
	ret += mach1_write_sram_loc(spi, 0x6f ,0xb7 ,0x5  ,0x4f ,0xc );
	ret += mach1_write_sram_loc(spi, 0x70 ,0x38 ,0xf6 ,0xe0 ,0x20);
	ret += mach1_write_sram_loc(spi, 0x71 ,0x38 ,0x6  ,0x1f ,0xe );
	ret += mach1_write_sram_loc(spi, 0x72 ,0x39 ,0xf6 ,0x30 ,0x6 );
	ret += mach1_write_sram_loc(spi, 0x73 ,0x39 ,0x6  ,0x8f ,0x15);
	ret += mach1_write_sram_loc(spi, 0x74 ,0xba ,0xf6 ,0x0  ,0x30);
	ret += mach1_write_sram_loc(spi, 0x75 ,0xba ,0x6  ,0x9f ,0x3a);
	ret += mach1_write_sram_loc(spi, 0x76 ,0xbb ,0xf6 ,0x90 ,0x3 );
	ret += mach1_write_sram_loc(spi, 0x77 ,0xbb ,0x6  ,0x4f ,0x27);
	ret += mach1_write_sram_loc(spi, 0x78 ,0x3c ,0xf7 ,0x30 ,0x2c);
	ret += mach1_write_sram_loc(spi, 0x79 ,0x3c ,0x7  ,0xaf ,0xb );
	ret += mach1_write_sram_loc(spi, 0x7a ,0x3d ,0xf7 ,0x40 ,0x3d);
	ret += mach1_write_sram_loc(spi, 0x7b ,0x3d ,0x7  ,0xdf ,0x1f);
	ret += mach1_write_sram_loc(spi, 0x7c ,0xbe ,0xf7 ,0xe0 ,0x19);
	ret += mach1_write_sram_loc(spi, 0x7d ,0xbe ,0x7  ,0x7f ,0x2b);
	ret += mach1_write_sram_loc(spi, 0x7e ,0xbf ,0xf7 ,0x60 ,0x12);
	ret += mach1_write_sram_loc(spi, 0x7f ,0xbf ,0x7  ,0xff ,0x36);
	ret += mach1_write_sram_loc(spi, 0x80 ,0x40 ,0xf0 ,0xe0 ,0x6 );
	ret += mach1_write_sram_loc(spi, 0x81 ,0x40 ,0x0  ,0xef ,0xd );
	ret += mach1_write_sram_loc(spi, 0x82 ,0x41 ,0xf0 ,0x40 ,0x38);
	ret += mach1_write_sram_loc(spi, 0x83 ,0x41 ,0x0  ,0x5f ,0xb );
	ret += mach1_write_sram_loc(spi, 0x84 ,0xc2 ,0xf0 ,0xc0 ,0x2f);
	ret += mach1_write_sram_loc(spi, 0x85 ,0xc2 ,0x0  ,0xdf ,0x20);
	ret += mach1_write_sram_loc(spi, 0x86 ,0xc3 ,0xf0 ,0x30 ,0x11);
	ret += mach1_write_sram_loc(spi, 0x87 ,0xc3 ,0x0  ,0xaf ,0x30);
	ret += mach1_write_sram_loc(spi, 0x88 ,0x44 ,0xf1 ,0x10 ,0x26);
	ret += mach1_write_sram_loc(spi, 0x89 ,0x44 ,0x1  ,0x5f ,0x34);
	ret += mach1_write_sram_loc(spi, 0x8a ,0x45 ,0xf1 ,0x60 ,0x14);
	ret += mach1_write_sram_loc(spi, 0x8b ,0x45 ,0x1  ,0x6f ,0x1b);
	ret += mach1_write_sram_loc(spi, 0x8c ,0xc6 ,0xf1 ,0xb0 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0x8d ,0xc6 ,0x1  ,0xff ,0x1f);
	ret += mach1_write_sram_loc(spi, 0x8e ,0xc7 ,0xf1 ,0xf0 ,0x28);
	ret += mach1_write_sram_loc(spi, 0x8f ,0xc7 ,0x1  ,0xf  ,0x3f);
	ret += mach1_write_sram_loc(spi, 0x90 ,0x48 ,0xf2 ,0xe0 ,0x8 );
	ret += mach1_write_sram_loc(spi, 0x91 ,0x48 ,0x2  ,0xcf ,0x21);
	ret += mach1_write_sram_loc(spi, 0x92 ,0x49 ,0xf2 ,0x90 ,0x3c);
	ret += mach1_write_sram_loc(spi, 0x93 ,0x49 ,0x2  ,0xcf ,0x6 );
	ret += mach1_write_sram_loc(spi, 0x94 ,0xca ,0xf2 ,0x10 ,0xc );
	ret += mach1_write_sram_loc(spi, 0x95 ,0xca ,0x2  ,0x9f ,0x36);
	ret += mach1_write_sram_loc(spi, 0x96 ,0xcb ,0xf2 ,0x80 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0x97 ,0xcb ,0x2  ,0xaf ,0x15);
	ret += mach1_write_sram_loc(spi, 0x98 ,0x4c ,0xf3 ,0xd0 ,0x2e);
	ret += mach1_write_sram_loc(spi, 0x99 ,0x4c ,0x3  ,0xdf ,0x1f);
	ret += mach1_write_sram_loc(spi, 0x9a ,0x4d ,0xf3 ,0x10 ,0x39);
	ret += mach1_write_sram_loc(spi, 0x9b ,0x4d ,0x3  ,0xff ,0x32);
	ret += mach1_write_sram_loc(spi, 0x9c ,0xce ,0xf3 ,0x90 ,0x27);
	ret += mach1_write_sram_loc(spi, 0x9d ,0xce ,0x3  ,0xf  ,0x19);
	ret += mach1_write_sram_loc(spi, 0x9e ,0xcf ,0xf3 ,0x80 ,0x10);
	ret += mach1_write_sram_loc(spi, 0x9f ,0xcf ,0x3  ,0xbf ,0x29);
	ret += mach1_write_sram_loc(spi, 0xa0 ,0x50 ,0xf4 ,0x40 ,0x16);
	ret += mach1_write_sram_loc(spi, 0xa1 ,0x50 ,0x4  ,0x5f ,0x1a);
	ret += mach1_write_sram_loc(spi, 0xa2 ,0x51 ,0xf4 ,0xc0 ,0x38);
	ret += mach1_write_sram_loc(spi, 0xa3 ,0x51 ,0x4  ,0xbf ,0x23);
	ret += mach1_write_sram_loc(spi, 0xa4 ,0xd2 ,0xf4 ,0xd0 ,0x35);
	ret += mach1_write_sram_loc(spi, 0xa5 ,0xd2 ,0x4  ,0x2f ,0xf );
	ret += mach1_write_sram_loc(spi, 0xa6 ,0xd3 ,0xf4 ,0x80 ,0x3f);
	ret += mach1_write_sram_loc(spi, 0xa7 ,0xd3 ,0x4  ,0x2f ,0x8 );
	ret += mach1_write_sram_loc(spi, 0xa8 ,0x54 ,0xf5 ,0x60 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xa9 ,0x54 ,0x5  ,0x1f ,0x2b);
	ret += mach1_write_sram_loc(spi, 0xaa ,0x55 ,0xf5 ,0xf0 ,0x11);
	ret += mach1_write_sram_loc(spi, 0xab ,0x55 ,0x5  ,0xf  ,0x4 );
	ret += mach1_write_sram_loc(spi, 0xac ,0xd6 ,0xf5 ,0xe0 ,0x33);
	ret += mach1_write_sram_loc(spi, 0xad ,0xd6 ,0x5  ,0x5f ,0x25);
	ret += mach1_write_sram_loc(spi, 0xae ,0xd7 ,0xf5 ,0xa0 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0xaf ,0xd7 ,0x5  ,0x3f ,0x1c);
	ret += mach1_write_sram_loc(spi, 0xb0 ,0x58 ,0xf6 ,0x70 ,0x7 );
	ret += mach1_write_sram_loc(spi, 0xb1 ,0x58 ,0x6  ,0x3f ,0x23);
	ret += mach1_write_sram_loc(spi, 0xb2 ,0x59 ,0xf6 ,0x70 ,0x31);
	ret += mach1_write_sram_loc(spi, 0xb3 ,0x59 ,0x6  ,0x7f ,0x10);
	ret += mach1_write_sram_loc(spi, 0xb4 ,0xda ,0xf6 ,0xb0 ,0xc );
	ret += mach1_write_sram_loc(spi, 0xb5 ,0xda ,0x6  ,0xf  ,0x1a);
	ret += mach1_write_sram_loc(spi, 0xb6 ,0xdb ,0xf6 ,0x30 ,0xa );
	ret += mach1_write_sram_loc(spi, 0xb7 ,0xdb ,0x6  ,0x7f ,0x26);
	ret += mach1_write_sram_loc(spi, 0xb8 ,0x5c ,0xf7 ,0x20 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xb9 ,0x5c ,0x7  ,0x9f ,0x29);
	ret += mach1_write_sram_loc(spi, 0xba ,0x5d ,0xf7 ,0x80 ,0x35);
	ret += mach1_write_sram_loc(spi, 0xbb ,0x5d ,0x7  ,0x4f ,0x1f);
	ret += mach1_write_sram_loc(spi, 0xbc ,0xde ,0xf7 ,0xa0 ,0x3b);
	ret += mach1_write_sram_loc(spi, 0xbd ,0xde ,0x7  ,0x6f ,0x2 );
	ret += mach1_write_sram_loc(spi, 0xbe ,0xdf ,0xf7 ,0x20 ,0x3e);
	ret += mach1_write_sram_loc(spi, 0xbf ,0xdf ,0x7  ,0x4f ,0x15);
	ret += mach1_write_sram_loc(spi, 0xc0 ,0x60 ,0xf0 ,0x10 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xc1 ,0x60 ,0x0  ,0xf  ,0x34);
	ret += mach1_write_sram_loc(spi, 0xc2 ,0x61 ,0xf0 ,0x70 ,0x31);
	ret += mach1_write_sram_loc(spi, 0xc3 ,0x61 ,0x0  ,0x5f ,0x4 );
	ret += mach1_write_sram_loc(spi, 0xc4 ,0xe2 ,0xf0 ,0x40 ,0x1c);
	ret += mach1_write_sram_loc(spi, 0xc5 ,0xe2 ,0x0  ,0xef ,0x25);
	ret += mach1_write_sram_loc(spi, 0xc6 ,0xe3 ,0xf0 ,0xa0 ,0x2 );
	ret += mach1_write_sram_loc(spi, 0xc7 ,0xe3 ,0x0  ,0x8f ,0x10);
	ret += mach1_write_sram_loc(spi, 0xc8 ,0x64 ,0xf1 ,0x10 ,0x3a);
	ret += mach1_write_sram_loc(spi, 0xc9 ,0x64 ,0x1  ,0xbf ,0x28);
	ret += mach1_write_sram_loc(spi, 0xca ,0x65 ,0xf1 ,0xf0 ,0xc );
	ret += mach1_write_sram_loc(spi, 0xcb ,0x65 ,0x1  ,0x5f ,0xa );
	ret += mach1_write_sram_loc(spi, 0xcc ,0xe6 ,0xf1 ,0xf0 ,0x1a);
	ret += mach1_write_sram_loc(spi, 0xcd ,0xe6 ,0x1  ,0x1f ,0x3f);
	ret += mach1_write_sram_loc(spi, 0xce ,0xe7 ,0xf1 ,0x60 ,0x23);
	ret += mach1_write_sram_loc(spi, 0xcf ,0xe7 ,0x1  ,0xdf ,0x16);
	ret += mach1_write_sram_loc(spi, 0xd0 ,0x68 ,0xf2 ,0xe0 ,0x2a);
	ret += mach1_write_sram_loc(spi, 0xd1 ,0x68 ,0x2  ,0x4f ,0xd );
	ret += mach1_write_sram_loc(spi, 0xd2 ,0x69 ,0xf2 ,0x30 ,0x35);
	ret += mach1_write_sram_loc(spi, 0xd3 ,0x69 ,0x2  ,0xdf ,0x38);
	ret += mach1_write_sram_loc(spi, 0xd4 ,0xea ,0xf2 ,0xa0 ,0x11);
	ret += mach1_write_sram_loc(spi, 0xd5 ,0xea ,0x2  ,0xcf ,0x26);
	ret += mach1_write_sram_loc(spi, 0xd6 ,0xeb ,0xf2 ,0xb0 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xd7 ,0xeb ,0x2  ,0x6f ,0x21);
	ret += mach1_write_sram_loc(spi, 0xd8 ,0x6c ,0xf3 ,0x50 ,0x17);
	ret += mach1_write_sram_loc(spi, 0xd9 ,0x6c ,0x3  ,0x8f ,0x19);
	ret += mach1_write_sram_loc(spi, 0xda ,0x6d ,0xf3 ,0x70 ,0x3c);
	ret += mach1_write_sram_loc(spi, 0xdb ,0x6d ,0x3  ,0x2f ,0x2 );
	ret += mach1_write_sram_loc(spi, 0xdc ,0xee ,0xf3 ,0xc0 ,0x31);
	ret += mach1_write_sram_loc(spi, 0xdd ,0xee ,0x3  ,0xef ,0x7 );
	ret += mach1_write_sram_loc(spi, 0xde ,0xef ,0xf3 ,0xe0 ,0x1c);
	ret += mach1_write_sram_loc(spi, 0xdf ,0xef ,0x3  ,0x3f ,0xb );
	ret += mach1_write_sram_loc(spi, 0xe0 ,0x70 ,0xf4 ,0x0  ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xe1 ,0x70 ,0x4  ,0xdf ,0x39);
	ret += mach1_write_sram_loc(spi, 0xe2 ,0x71 ,0xf4 ,0x20 ,0x3f);
	ret += mach1_write_sram_loc(spi, 0xe3 ,0x71 ,0x4  ,0x1f ,0x28);
	ret += mach1_write_sram_loc(spi, 0xe4 ,0xf2 ,0xf4 ,0xb0 ,0x26);
	ret += mach1_write_sram_loc(spi, 0xe5 ,0xf2 ,0x4  ,0x9f ,0x1f);
	ret += mach1_write_sram_loc(spi, 0xe6 ,0xf3 ,0xf4 ,0xf0 ,0xb );
	ret += mach1_write_sram_loc(spi, 0xe7 ,0xf3 ,0x4  ,0xf  ,0x1a);
	ret += mach1_write_sram_loc(spi, 0xe8 ,0x74 ,0xf5 ,0xc0 ,0x13);
	ret += mach1_write_sram_loc(spi, 0xe9 ,0x74 ,0x5  ,0x2f ,0x15);
	ret += mach1_write_sram_loc(spi, 0xea ,0x75 ,0xf5 ,0xc0 ,0x30);
	ret += mach1_write_sram_loc(spi, 0xeb ,0x75 ,0x5  ,0x3f ,0x36);
	ret += mach1_write_sram_loc(spi, 0xec ,0xf6 ,0xf5 ,0xa0 ,0x7 );
	ret += mach1_write_sram_loc(spi, 0xed ,0xf6 ,0x5  ,0xbf ,0x3 );
	ret += mach1_write_sram_loc(spi, 0xee ,0xf7 ,0xf5 ,0xd0 ,0x23);
	ret += mach1_write_sram_loc(spi, 0xef ,0xf7 ,0x5  ,0x4f ,0xf );
	ret += mach1_write_sram_loc(spi, 0xf0 ,0x78 ,0xf6 ,0x60 ,0x32);
	ret += mach1_write_sram_loc(spi, 0xf1 ,0x78 ,0x6  ,0x9f ,0x6 );
	ret += mach1_write_sram_loc(spi, 0xf2 ,0x79 ,0xf6 ,0x80 ,0x13);
	ret += mach1_write_sram_loc(spi, 0xf3 ,0x79 ,0x6  ,0x5f ,0x1 );
	ret += mach1_write_sram_loc(spi, 0xf4 ,0xfa ,0xf6 ,0x90 ,0x18);
	ret += mach1_write_sram_loc(spi, 0xf5 ,0xfa ,0x6  ,0x7f ,0xf );
	ret += mach1_write_sram_loc(spi, 0xf6 ,0xfb ,0xf6 ,0x40 ,0x3e);
	ret += mach1_write_sram_loc(spi, 0xf7 ,0xfb ,0x6  ,0x6f ,0x28);
	ret += mach1_write_sram_loc(spi, 0xf8 ,0x7c ,0xf7 ,0x0  ,0x23);
	ret += mach1_write_sram_loc(spi, 0xf9 ,0x7c ,0x7  ,0xff ,0x35);
	ret += mach1_write_sram_loc(spi, 0xfa ,0x7d ,0xf7 ,0x90 ,0x2d);
	ret += mach1_write_sram_loc(spi, 0xfb ,0x7d ,0x7  ,0x3f ,0x14);
	ret += mach1_write_sram_loc(spi, 0xfc ,0xfe ,0xf7 ,0x70 ,0x26);
	ret += mach1_write_sram_loc(spi, 0xfd ,0xfe ,0x7  ,0xaf ,0x38);
	ret += mach1_write_sram_loc(spi, 0xfe ,0xff ,0xf7 ,0x80 ,0xa );
	ret += mach1_write_sram_loc(spi, 0xff ,0xff ,0x7  ,0x2f ,0x1c);
	return ret;
}


static int mach1_setup(struct spi_device *spi, int num_lanes)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	unsigned long lane_rate_kHz;
	unsigned pll_stat;
	unsigned test_mode_status;
	int ret;

//  ret = gpiod_get_value(conv->pgf_gpio);
//  if (ret == 0) {
//		dev_err(&conv->spi->dev, "Power good not set !!!");
//		return -1;
//	}
//
//	// Issue a software reset
//	ret = ad9467_spi_write(spi, 0x00, 0x91);
//
//  // WRITE INTERFACE_CONFIG_A
//  // Set Address Ascension
//	ret = ad9467_spi_write(spi, 0x00, 0x30);
//
//  // WRITE INTERFACE_CONFIG_B
//  // Set Single Instruction Mode
//	ret = ad9467_spi_write(spi, 0x01, 0x80);
//
//	// Clear POR
//	ret = ad9467_spi_write(spi, 0x20, 0x81);
//
//	// FUSE CLOCK CONFIGURATON
//	//
//	// Configure GPIO_1 as input
//	ret = ad9467_spi_write(spi, 0x24, 0x0D);
//
//	// Set gpio[40] to 1
//	ret = gpiod_direction_output(conv->fuse_clk_en_gpio, 1);
//	gpiod_set_value(conv->fuse_clk_en_gpio, 1);
//
//  //	Set Fuse Clock Enable
//	ret = ad9467_spi_write(spi, 0x42, 0x10);
//
//	// TEST MODE ENTRY SEQUENCE
//	ret = ad9467_spi_write(spi, 0x52, 0xDA); // Passing the first test mode entry key.
//	ret = ad9467_spi_write(spi, 0x52, 0x39); // Passing the second test mode entry key
//
//	// READ register 0x52h and confirm the value to be 0x01h indicating test mode is unlocked.
//	test_mode_status = ad9467_spi_read(spi, 0x52);
//	if (test_mode_status != 1) {
//		ret = ad9467_spi_write(spi, 0x52, 0xDA); // Passing the first test mode entry key.
//		ret = ad9467_spi_write(spi, 0x52, 0x39); // Passing the second test mode entry key
//	}
//	test_mode_status = ad9467_spi_read(spi, 0x52);
//	if (test_mode_status != 1) {
//		dev_err(&conv->spi->dev, "Failed to set test mode. Test mode status: %d\n", test_mode_status);
//		return -1;
//	}
//
//	// FUSE REFRESH
//	// A fuse refresh is initiated by executing the following indirect write sequence to the fuse control register
////	ret = ad9467_spi_write(spi, 0x104, 0x8);  // ECC/CTRL Register
//	//
//	//	Write (shadow address – 0x104) with 0x8
//	//	//
//	ret = ad9467_spi_write(spi, 0x56, 0x01);
//	ret = ad9467_spi_write(spi, 0x57, 0x04);
//	ret = ad9467_spi_write(spi, 0x58, 0x00);
//	ret = ad9467_spi_write(spi, 0x59, 0x80);
//	ret = ad9467_spi_write(spi, 0x54, 0x1);
//	ret = ad9467_spi_write(spi, 0x54, 0x0);
//
//
//
//	// PROGRAM SRAM
//	ret = ad9467_spi_write(spi, 0x40, 0x00); // Sets shuffler control to internal (Default).
//	ret = ad9467_spi_write(spi, 0x43, 0x02); // This enables the shuffler SRAM Memory.
//	ret = ad9467_spi_write(spi, 0x44, 0x10); // This sequence enables the SRAM Write Assist Function.
//
//  ret = mach1_init_sram(spi);
//
//	// This selects the programmed SRAM contents for shuffler and dither control.
//	ret = ad9467_spi_write(spi, 0x40, 0x08);
//
//  //  
//	//  Tentative to write shadow registers
//	//
//	// Write shadow register address 0xE5 with 0x2BA7 
//	// 
//	// Unlock the shadow registers
//	ret = ad9467_spi_write(spi, 0xFC, 0x08);
//	ret = ad9467_spi_write(spi, 0xF8, 0xE4);
//	ret = ad9467_spi_read(spi, 0xFA);
//	if (ret != 1) {
//		dev_err(&conv->spi->dev, "Failed to unlock shadow reg. 0xFA : %d\n", ret);
//	//	return -1;
//	}
//	
//	//
//	//	Write (shadow address – 0xE5) to address 0x56
//	//
//	ret = ad9467_spi_write(spi, 0x56, 0x00);
//	ret = ad9467_spi_write(spi, 0x57, 0xE5);
//	ret = ad9467_spi_write(spi, 0x58, 0x2B);
//	ret = ad9467_spi_write(spi, 0x59, 0xA7);
//	ret = ad9467_spi_write(spi, 0x54, 0x1);
//	ret = ad9467_spi_write(spi, 0x54, 0x0);
//
//#define FUSE_RDREG_MSB_ADDR 0x5A
//#define FUSE_RDREG_LSB_ADDR 0x5B
//#define FUSE_DATAR_MSB_ADDR 0x5C
//#define FUSE_DATAR_LSB_ADDR 0x5D
//
//	ret = ad9467_spi_write(spi, FUSE_RDREG_MSB_ADDR, 0x0);
//	ret = ad9467_spi_write(spi, FUSE_RDREG_LSB_ADDR, 0xE5);
//	ret = ad9467_spi_read(spi, FUSE_DATAR_MSB_ADDR);
//	dev_err(&conv->spi->dev, "0xE5 [15:8]: %x\n", ret);
//	ret = ad9467_spi_read(spi, FUSE_DATAR_LSB_ADDR);
//	dev_err(&conv->spi->dev, "0xE5 [7:0]: %x\n", ret);
//
//
//	// DISABLE FUSE CLOCK
//	// Disable FUSE CLOCK by clearing the FUSE_CLK_EN bit
//	ret = ad9467_spi_write(spi, 0x42, 0x00);
//
//  if (num_lanes == 1) {
//	  //set single lane mode
//	  ret = ad9467_spi_write(spi, 0x21, 0x00);
//	  ret = ad9467_spi_write(spi, 0x22, 5 << 2);
//	} else if (num_lanes == 2) {
//		// Set dual lane mode
//		ret = ad9467_spi_write(spi, 0x21, 0x40);
//		ret = ad9467_spi_write(spi, 0x22, 4 << 2);
//	} else {
//		dev_err(&conv->spi->dev, "Invalid number of lanes: %d\n", num_lanes);
//		return -1;
//	}
//
//  // Enable CNV and clock input
//	ret = ad9467_spi_write(spi, 0x4F, 0x2);
//
//	//disable (Power down) the internal reference buffer
//	ret = ad9467_spi_write(spi, 0x50, 0x1);
//	//enable the ADC2 Conversion Start to be tied to an internal timer.
//	//This is required for READ DURING CONVERSION selection to be made.
//	ret = ad9467_spi_write(spi, 0x41, 0x72);
//
//

	return ret;
}
static int ad9467_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned vref_val, vref_mask;
	unsigned int i;

	switch (conv->chip_info->id) {
	case CHIPID_AD9467:
		vref_mask = AD9467_REG_VREF_MASK;
		break;
	case CHIPID_AD9643:
		vref_mask = AD9643_REG_VREF_MASK;
		break;
	case CHIPID_AD9250:
	case CHIPID_AD9683:
	case CHIPID_AD9625:
		vref_mask = AD9250_REG_VREF_MASK;
		break;
	case CHIPID_AD9265:
		vref_mask = AD9265_REG_VREF_MASK;
		break;
	case CHIPID_AD9652:
		vref_mask = AD9652_REG_VREF_MASK;
		break;
	case CHIPID_MACH1:
		i = 0;
		goto skip_reg_read;
	default:
		vref_mask = 0xFFFF;
		break;
	}

	vref_val = ad9467_spi_read(conv->spi, AN877_ADC_REG_VREF);

	vref_val &= vref_mask;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		if (vref_val == conv->chip_info->scale_table[i][1])
			break;
	}

skip_reg_read:
	ad9467_scale(conv, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ad9467_set_scale(struct axiadc_converter *conv, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int i;

	switch (conv->chip_info->id) {
	case CHIPID_MACH1:
		return -EINVAL;
	default:
		break;
	}

	if (val != 0)
		return -EINVAL;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9467_scale(conv, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		ad9467_spi_write(conv->spi, AN877_ADC_REG_VREF,
				 conv->chip_info->scale_table[i][1]);
		ad9467_spi_write(conv->spi, AN877_ADC_REG_TRANSFER,
				 AN877_ADC_TRANSFER_SYNC);
		return 0;
	}

	return -EINVAL;
}

static int ad9467_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ad9467_get_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = conv->adc_clk = clk_get_rate_scaled(conv->clk, &conv->adc_clkscale);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9467_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long r_clk;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return ad9467_set_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		if (conv->adc_clk != r_clk) {
			conv->adc_clk = r_clk;
			ret = ad9467_dco_calibrate(indio_dev,
						   conv->chip_info->
						   num_channels);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9467_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
  struct spi_device *spi = conv->spi;

	int ret, i;

	unsigned reg_cntrl = axiadc_read(st, ADI_REG_CNTRL);
	//  Set EDGESEL to 0 - Sample with posedge	
	reg_cntrl &= ~ADI_DDR_EDGESEL;
//   // Set SDR mode,
// 	reg_cntrl |= ADI_SDR_DDR_N;
	// Set number of lanes
	reg_cntrl |= ADI_NUM_LANES(conv->num_lanes);
	
	axiadc_write(st, ADI_REG_CNTRL, reg_cntrl);

	////  !!!!
	////  Place this after the CNV and clock input of the converter is enabled.
	////  SYNC generation depends on falling edge of fuse clk enable
	////  !!!!
	//// Disable fuse clock generation
	//// set gpio[40] to 0
	//gpiod_set_value(conv->fuse_clk_en_gpio, 0);
	//ret = gpiod_direction_output(conv->fuse_clk_en_gpio, 0);

	//// Set GPIO_1 as output
	//ret = ad9467_spi_write(spi, 0x24, 0xF);

/*
	ret = ad9467_idelay_calibrate(indio_dev, conv->chip_info->num_channels);
	if (ret < 0)
		return ret;

	ret = ad9467_dco_calibrate(indio_dev, conv->chip_info->num_channels);
	if (ret < 0)
		return ret;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
			     ADI_IQCOR_ENB | ADI_ENABLE);
	}
*/
	return 0;
}

static int ad9467_setup(struct axiadc_converter *st, unsigned int chip_id)
{
	struct spi_device *spi = st->spi;
	int ret;

	st->adc_output_mode = AN877_ADC_OUTPUT_MODE_TWOS_COMPLEMENT;

	switch (chip_id) {
	case CHIPID_AD9467:
		st->adc_output_mode |= AD9467_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9643:
		st->adc_output_mode |= AD9643_DEF_OUTPUT_MODE;
		return ad9467_spi_write(spi, AN877_ADC_REG_OUTPUT_PHASE,
					AN877_ADC_OUTPUT_EVEN_ODD_MODE_EN);
	case CHIPID_AD9250:
		ret = ad9250_setup(spi, 2, 2);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			return ret;
		}

		st->adc_output_mode |= AD9250_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9683:
		ret = ad9250_setup(spi, 1, 1);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			return ret;
		}
		st->adc_output_mode |= AD9683_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9625:
		ret = ad9625_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			return ret;
		}

		st->adc_output_mode |= AD9625_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9265:
		st->adc_output_mode |= AD9265_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9434:
		st->adc_output_mode |= AD9434_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9652:
		st->adc_output_mode |= AD9643_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_AD9649:
		st->adc_output_mode |= AD9643_DEF_OUTPUT_MODE;
		return 0;
	case CHIPID_MACH1:
		ret = mach1_setup(spi, st->num_lanes);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			return ret;
		}

		st->adc_output_mode |= AD9434_DEF_OUTPUT_MODE;
		return 0;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", chip_id);
		return -ENODEV;;
	}
}

static void ad9467_clk_disable(void *data)
{
	struct axiadc_converter *st = data;

	clk_disable_unprepare(st->clk);
}

static int ad9467_probe(struct spi_device *spi)
{
	const struct axiadc_chip_info *info;
	struct axiadc_converter *conv, *st;
	unsigned int id;
	int ret;
	u32 val;

	info = of_device_get_match_data(&spi->dev);
	if (!info)
		return -ENODEV;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;
	/* FIXME: remove this asap; it's to make things easier to diff with upstream version */
	st = conv;

	st->spi = spi;

	st->clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	if (info->id != CHIPID_AD9625) {
		ret = clk_prepare_enable(st->clk);
		if (ret < 0)
			return ret;

		ret = devm_add_action_or_reset(&spi->dev, ad9467_clk_disable, st);
		if (ret)
			return ret;
	}

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	st->pwrdown_gpio = devm_gpiod_get_optional(&spi->dev, "powerdown",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->pwrdown_gpio))
		return PTR_ERR(st->pwrdown_gpio);

	st->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	st->fuse_clk_en_gpio = devm_gpiod_get_optional(&spi->dev, "fuse_clk_en",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->fuse_clk_en_gpio))
		return PTR_ERR(st->fuse_clk_en_gpio);

	st->pgf_gpio = devm_gpiod_get(&spi->dev, "pgf",
						 GPIOD_IN);
	if (IS_ERR(st->pgf_gpio))
		return PTR_ERR(st->pgf_gpio);

	st->num_lanes = 1;

	ret = of_property_read_u32(spi->dev.of_node, "num_lanes", &val);
	if (ret == 0)
		st->num_lanes = val;


	if (st->reset_gpio) {
		udelay(1);
		ret = gpiod_direction_output(st->reset_gpio, 1);
		if (ret)
			return ret;
		mdelay(10);
	}

	spi_set_drvdata(spi, st);

	conv->chip_info = info;

	id = 0x03; //ad9467_spi_read(spi, AN877_ADC_REG_CHIP_ID);
//	if (id != conv->chip_info->id) {
//		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
//		return -ENODEV;
//	}

	conv->reg_access = ad9467_reg_access;
	conv->write_raw = ad9467_write_raw;
	conv->read_raw = ad9467_read_raw;
	conv->post_setup = ad9467_post_setup;
	conv->set_pnsel = ad9467_set_pnsel;

	ret = ad9467_setup(conv, id);
	if (ret)
		return ret;

	//return ad9467_outputmode_set(spi, conv->adc_output_mode);
	return 0;
}

static const struct of_device_id ad9467_of_match[] = {
	{ .compatible = "adi,ad9467", .data = &ad9467_chip_tbl[ID_AD9467], },
	{ .compatible = "adi,ad9643", .data = &ad9467_chip_tbl[ID_AD9643], },
	{ .compatible = "adi,ad9250", .data = &ad9467_chip_tbl[ID_AD9250], },
	{ .compatible = "adi,ad9265", .data = &ad9467_chip_tbl[ID_AD9265], },
	{ .compatible = "adi,ad9683", .data = &ad9467_chip_tbl[ID_AD9683], },
	{ .compatible = "adi,ad9434", .data = &ad9467_chip_tbl[ID_AD9434], },
	{ .compatible = "adi,ad9625", .data = &ad9467_chip_tbl[ID_AD9625], },
	{ .compatible = "adi,ad9652", .data = &ad9467_chip_tbl[ID_AD9652], },
	{ .compatible = "adi,ad9649", .data = &ad9467_chip_tbl[ID_AD9649], },
	{ .compatible = "adi,mach1", .data = &ad9467_chip_tbl[ID_MACH1], },
	{}
};
MODULE_DEVICE_TABLE(of, ad9467_of_match);

static struct spi_driver ad9467_driver = {
	.driver = {
		.name = "ad9467",
		.of_match_table = ad9467_of_match,
	},
	.probe = ad9467_probe,
};
module_spi_driver(ad9467_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC driver");
MODULE_LICENSE("GPL v2");
