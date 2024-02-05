// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9467 SPI ADC driver
 *
 * Copyright 2012-2020 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>


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

/*
 * There's an upstreamed version of this driver that was used as first user of
 * the high speed adc core. However, that version is still too limited and lacks
 * support for a lot of things. Hence, we will still take the old driver until the
 * the upstreamed version is ready to be used...
 */

#define AN877_ADC_REG_CHIP_PORT_CONF		0x00
#define AN877_ADC_REG_CHIP_ID			0x01
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

#define CHIPID_MACH1			0x7
#define MACH1_ADC_REG_CHIP_ID		0x3

enum {
	ID_AD9467,
	ID_AD9643,
	ID_AD9250,
	ID_AD9250_2,
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
	if (ret < 0)
		return ret;

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

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

static const int ad9625_scale_table[][2] = {
	{1000, 0},
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
	*val = tmp / 1000000000;
	*val2 = tmp % 1000000000;
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

static ssize_t ad9467_lvds_sync_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "enable\n");
}

static ssize_t ad9467_lvds_sync_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned timeout = 100;
	int ret;

	mutex_lock(&indio_dev->mlock);

	ret = ad9467_spi_write(conv->spi, 0x15, 0x50);
	if (ret)
		return ret;

	axiadc_write(st, ADI_REG_CNTRL, 0x108);

	do {
		if (axiadc_read(st, ADI_REG_SYNC_STATUS) == 0)
			dev_info(&conv->spi->dev, "Not Locked: Running Bit Slip\n");
		else
			break;
	} while (--timeout);

	if (timeout) {
		dev_info(&conv->spi->dev, "Success: Pattern correct and Locked!\n");
		ret = ad9467_spi_write(conv->spi, 0x15, 0x40);
	} else {
		dev_info(&conv->spi->dev, "LVDS Sync Timeout.\n");
		ret = -ETIME;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9467_lvds_cnv_en_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret;

	ret = ad9467_spi_read(conv->spi, 0x15);
	if (ret < 0)
		return ret;

	ret &= BIT(0);

	return ret < 0 ? ret : sysfs_emit(buf, "%u\n", ret);
}

static ssize_t ad9467_lvds_cnv_en_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	ret = ad9467_spi_read(conv->spi, 0x15);
	if (ret < 0)
		goto mutex_unlock;

	ret &= ~BIT(0);
	ret |= en;

	ret = ad9467_spi_write(conv->spi, 0x15, ret);

mutex_unlock:
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
	 },
	{
	 .name = "lvds_sync",
	 .write = ad9467_lvds_sync_write,
	 .read = ad9467_lvds_sync_read,
	 },
	{
	 .name = "lvds_cnv",
	 .read = ad9467_lvds_cnv_en_read,
	 .write = ad9467_lvds_cnv_en_write,
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
	  .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),		\
	  .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
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
	[ID_AD9250_2] = {
		.name = "AD9250_2",
		.id = CHIPID_AD9250,
		.max_rate = 250000000UL,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.max_testmode = AN877_ADC_TESTMODE_RAMP,
		.num_channels = 4,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
		.channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'S', 0),
		.channel[2] = AIM_CHAN_NOCALIB(2, 2, 14, 'S', 0),
		.channel[3] = AIM_CHAN_NOCALIB(3, 3, 14, 'S', 0),
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
		.scale_table = ad9625_scale_table,
		.num_scales = ARRAY_SIZE(ad9625_scale_table),
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

static void ad9625_clk_del_provider(void *data)
{
	struct axiadc_converter *conv = data;

	of_clk_del_provider(conv->spi->dev.of_node);
	clk_unregister_fixed_factor(conv->out_clk);
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

	conv->out_clk = clk_register_fixed_factor(&spi->dev, "div_clk",
		__clk_get_name(conv->clk), 0, 1, 4 / conv->adc_clkscale.div);
	if (IS_ERR(conv->out_clk)) {
		dev_warn(&spi->dev, "Failed to register dic_clk\n");
	} else {
		ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, conv->out_clk);
		if (ret)
			clk_unregister_fixed_factor(conv->out_clk);

		devm_add_action_or_reset(&spi->dev, ad9625_clk_del_provider, conv);
	}

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
		break;
	case CHIPID_AD9265:
		vref_mask = AD9265_REG_VREF_MASK;
		break;
	case CHIPID_AD9652:
		vref_mask = AD9652_REG_VREF_MASK;
		break;
	case CHIPID_AD9649:
	case CHIPID_AD9625:
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

	return IIO_VAL_INT_PLUS_NANO;
}

static int ad9467_set_scale(struct axiadc_converter *conv, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int i;

	switch (conv->chip_info->id) {
	case CHIPID_AD9625:
	case CHIPID_AD9649:
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
		if (conv->chip_info->id == CHIPID_MACH1)
			return -EINVAL;
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

static int mach1_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	unsigned int reg_cntrl = axiadc_read(st, ADI_REG_CNTRL);
	//  Set EDGESEL to 0 - Sample with posedge
	reg_cntrl &= ~ADI_DDR_EDGESEL;
	// Set SDR mode,
	// reg_cntrl |= ADI_SDR_DDR_N;
	// Set number of lanes
	reg_cntrl |= ADI_NUM_LANES(conv->num_lanes);

	axiadc_write(st, ADI_REG_CNTRL, reg_cntrl);

	return 0;
}

static int ad9467_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, i;

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
		ad9467_spi_write(spi, 0x16, 0x61);
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
	unsigned int id, id_reg;
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

	if (info->id != CHIPID_AD9625 && info->id != CHIPID_AD9250) {
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

	if(conv->chip_info->id == CHIPID_MACH1)
		id_reg = MACH1_ADC_REG_CHIP_ID;
	else
		id_reg = AN877_ADC_REG_CHIP_ID;

	id = ad9467_spi_read(spi, id_reg);
	if (id != conv->chip_info->id) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		return -ENODEV;
	}

	conv->reg_access = ad9467_reg_access;
	conv->write_raw = ad9467_write_raw;
	conv->read_raw = ad9467_read_raw;
	if (conv->chip_info->id != CHIPID_MACH1)
		conv->post_setup = ad9467_post_setup;
	else
		conv->post_setup = mach1_post_setup;
	conv->set_pnsel = ad9467_set_pnsel;

	ret = ad9467_setup(conv, id);
	if (ret)
		return ret;

	if (conv->chip_info->id != CHIPID_MACH1)
		return ad9467_outputmode_set(spi, conv->adc_output_mode);

	return 0;
}

static const struct of_device_id ad9467_of_match[] = {
	{ .compatible = "adi,ad9467", .data = &ad9467_chip_tbl[ID_AD9467], },
	{ .compatible = "adi,ad9643", .data = &ad9467_chip_tbl[ID_AD9643], },
	{ .compatible = "adi,ad9250", .data = &ad9467_chip_tbl[ID_AD9250], },
	{ .compatible = "adi,ad9250_2", .data = &ad9467_chip_tbl[ID_AD9250_2], },
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
