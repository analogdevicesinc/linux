/*
 * AD9467 SPI ADC driver for DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#include <linux/clk.h>
#include <linux/clkdev.h>

#define DCO_DEBUG

/*
 * ADI High-Speed ADC common spi interface registers
 * See Application-Note AN-877
 */

#define ADC_REG_CHIP_PORT_CONF		0x00
#define ADC_REG_CHIP_ID			0x01
#define ADC_REG_CHIP_GRADE		0x02
#define ADC_REG_CHAN_INDEX		0x05
#define ADC_REG_TRANSFER		0xFF
#define ADC_REG_MODES			0x08
#define ADC_REG_TEST_IO			0x0D
#define ADC_REG_ADC_INPUT		0x0F
#define ADC_REG_OFFSET			0x10
#define ADC_REG_OUTPUT_MODE		0x14
#define ADC_REG_OUTPUT_ADJUST		0x15
#define ADC_REG_OUTPUT_PHASE		0x16
#define ADC_REG_OUTPUT_DELAY		0x17
#define ADC_REG_VREF			0x18
#define ADC_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ		0x6
#define TESTMODE_ONE_ZERO_TOGGLE	0x7
#define TESTMODE_USER			0x8
#define TESTMODE_BIT_TOGGLE		0x9
#define TESTMODE_SYNC			0xA
#define TESTMODE_ONE_BIT_HIGH		0xB
#define TESTMODE_MIXED_BIT_FREQUENCY	0xC
#define TESTMODE_RAMP			0xF

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

/* ADC_REG_OUTPUT_DELAY */
#define DCO_DELAY_ENABLE 		0x80

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
 * Analog Devices AD9680
 */

#define AD9680_REG_CHIP_ID_LOW		0x004
#define AD9680_REG_CHIP_ID_HIGH		0x005
#define AD9680_REG_DEVICE_INDEX		0x008
#define AD9680_REG_INPUT_FS_RANGE	0x025
#define AD9680_REG_CHIP_PIN_CTRL	0x040

#define AD9680_REG_OUTPUT_MODE		0x561
#define AD9680_REG_TEST_MODE		0x550

#define AD9680_REG_THRESH_CTRL		0x245
#define AD9680_REG_THRESH_HI_LSB	0x247
#define AD9680_REG_THRESH_HI_MSB	0x248
#define AD9680_REG_THRESH_LOW_LSB	0x249
#define AD9680_REG_THRESH_LOW_MSB	0x24A

#define CHIPID_AD9680			0xC5
#define AD9680_DEF_OUTPUT_MODE		0x00
#define AD9680_REG_VREF_MASK		0x0F

#define AD9680_REG_CHIP_PIN_CTRL_MASK(chn)	(0x07 << (3 * (chn)))

/*
 * Analog Devices AD9652
 */

#define CHIPID_AD9652			0xC1
#define AD9652_REG_VREF_MASK		0xC0

/*
 * Analog Devices AD9234
 */

#define CHIPID_AD9234			0xCE

/*
 * Analog Devices AD9649
 */

#define CHIPID_AD9649			0x6F

/*
 * Analog Devices AD9684
 */

#define CHIPID_AD9684			0xD2

enum {
	ID_AD9467,
	ID_AD9643,
	ID_AD9250,
	ID_AD9265,
	ID_AD9680,
	ID_AD9683,
	ID_AD9625,
	ID_AD9434,
	ID_AD9652,
	ID_AD9234,
	ID_AD9680_x2,
	ID_AD9649,
	ID_AD9684,
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

static int ad9467_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg >> 8;
		buf[1] = reg & 0xFF;
		buf[2] = val;
		ret = spi_write_then_read(spi, buf, 3, NULL, 0);
		if (ret < 0)
			return ret;

		dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
			__func__, reg, val, ret);

		if ((reg == ADC_REG_TRANSFER) && (val == TRANSFER_SYNC) &&
		    (spi_get_device_id(spi)->driver_data == CHIPID_AD9265))
			ad9467_spi_write(spi, ADC_REG_TRANSFER, 0);

		return 0;
	}

	return -ENODEV;
}

static int ad9467_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	if (readval == NULL) {
		ret = ad9467_spi_write(spi, reg, writeval);
		switch (conv->id) {
		case CHIPID_AD9234:
		case CHIPID_AD9680:
		case CHIPID_AD9684:
			break;
		default:
			ad9467_spi_write(spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
			break;
		}
		return ret;
	} else {
		ret = ad9467_spi_read(spi, reg);
		if (ret < 0)
			return ret;
		*readval = ret;
	}

	return 0;
}

static int ad9680_select_channel(struct axiadc_converter *conv,
	int chan)
{
	unsigned int val;

	if (chan >= 0)
		val = BIT(chan & 0x1);
	else
		val = 0x3;

	return ad9467_spi_write(conv->spi, AD9680_REG_DEVICE_INDEX, val);
}

static int ad9680_channel_write(struct axiadc_converter *conv,
	unsigned int chan, unsigned int reg, unsigned int val)
{
	int ret;

	ret = ad9680_select_channel(conv, chan);
	ret |= ad9467_spi_write(conv->spi, reg, val);
	ret |= ad9680_select_channel(conv, -1);

	return ret;
}

static int ad9467_outputmode_set(struct spi_device *spi, unsigned mode)
{
	int ret;

	ret = ad9467_spi_write(spi, ADC_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;
	ret = ad9467_spi_write(spi, ADC_REG_TEST_IO, TESTMODE_OFF);
	if (ret < 0)
		return ret;

	return ad9467_spi_write(spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
}

static int ad9467_testmode_set(struct iio_dev *indio_dev,
			       unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad9467_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 1 << chan);
	ad9467_spi_write(conv->spi, ADC_REG_TEST_IO, mode);
	ad9467_spi_write(conv->spi, ADC_REG_CHAN_INDEX, 0x3);
	ad9467_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);
	conv->testmode[chan] = mode;
	return 0;
}

static unsigned int ad9467_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return TESTMODE_PN23_SEQ;
	default:
		return TESTMODE_OFF;
	}
}

static int ad9467_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9467_pnsel_to_testmode(sel);
	int ret;

	if (mode == TESTMODE_OFF)
		ret = ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
				       conv->adc_output_mode);
	else
		ret = ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_MODE,
				       conv->
				       adc_output_mode &
				       ~OUTPUT_MODE_TWOS_COMPLEMENT);

	if (ret < 0)
		return ret;

	return ad9467_testmode_set(indio_dev, chan, mode);
}

static int ad9680_outputmode_set(struct spi_device *spi, unsigned mode)
{
	int ret;

	ret = ad9467_spi_write(spi, AD9680_REG_OUTPUT_MODE, mode);
	if (ret < 0)
		return ret;

	return ad9467_spi_write(spi, AD9680_REG_TEST_MODE, TESTMODE_OFF);
}

static int ad9680_testmode_set(struct iio_dev *indio_dev,
			       unsigned chan, unsigned mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	ad9680_channel_write(conv, chan, AD9680_REG_TEST_MODE, mode);
	conv->testmode[chan] = mode;

	return 0;
}

static int ad9680_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9467_pnsel_to_testmode(sel);
	int ret;

	if (mode == TESTMODE_OFF)
		ret = ad9467_spi_write(conv->spi, AD9680_REG_OUTPUT_MODE,
				       conv->adc_output_mode);
	else
		ret = ad9467_spi_write(conv->spi, AD9680_REG_OUTPUT_MODE,
				       conv->
				       adc_output_mode &
				       ~OUTPUT_MODE_TWOS_COMPLEMENT);

	if (ret < 0)
		return ret;

	return ad9680_testmode_set(indio_dev, chan, mode);
}

static int ad9467_dco_calibrate(struct iio_dev *indio_dev, unsigned chan)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, dco, cnt, start, max_start, max_cnt;
	unsigned stat, inv_range = 0, dco_en = 0, do_inv;
	unsigned char err_field[66];
	unsigned chan_ctrl0, chan_ctrl1;

	switch (conv->id) {
	case CHIPID_AD9250:
	case CHIPID_AD9683:
	case CHIPID_AD9680:
	case CHIPID_AD9625:
	case CHIPID_AD9434:	/* TODO */
		return 0;
	case CHIPID_AD9265:
	case CHIPID_AD9652:
		dco_en = 0;
		break;
	default:
		dco_en = DCO_DELAY_ENABLE;
	}

	ret = ad9467_outputmode_set(conv->spi,
				    conv->
				    adc_output_mode &
				    ~OUTPUT_MODE_TWOS_COMPLEMENT);
	if (ret < 0)
		return ret;

	chan_ctrl0 = axiadc_read(st, ADI_REG_CHAN_CNTRL(0));
	chan_ctrl1 = axiadc_read(st, ADI_REG_CHAN_CNTRL(1));

	do {
		if (conv->id !=	CHIPID_AD9652)
			ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE,
					OUTPUT_EVEN_ODD_MODE_EN | (inv_range ?
						INVERT_DCO_CLK : 0));

		if (chan == 2) {
			ad9467_testmode_set(indio_dev, 1, TESTMODE_PN23_SEQ);
			axiadc_write(st, ADI_REG_CHAN_CNTRL(1), ADI_ENABLE);
			axiadc_set_pnsel(st, 1, ADC_PN23A);
			axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);
		}

		ad9467_testmode_set(indio_dev, 0, TESTMODE_PN9_SEQ);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(0), ADI_ENABLE);
		axiadc_set_pnsel(st, 0, ADC_PN9);
		axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);

		for (dco = 0; dco <= 32; dco++) {
			ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_DELAY,
					 dco > 0 ? ((dco - 1) | dco_en) : 0);
			ad9467_spi_write(conv->spi, ADC_REG_TRANSFER,
					 TRANSFER_SYNC);
			ad9467_spi_read(conv->spi, ADC_REG_OUTPUT_DELAY);

			axiadc_write(st, ADI_REG_CHAN_STATUS(0), ~0);
			if (chan == 2)
				axiadc_write(st, ADI_REG_CHAN_STATUS(1), ~0);

			mdelay(1);

			stat = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
			if (chan == 2)
				stat |= axiadc_read(st, ADI_REG_CHAN_STATUS(1));

			err_field[dco + (inv_range * 33)] =
			    ! !(stat & (ADI_PN_ERR | ADI_PN_OOS));
		}

		for (dco = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		     dco <= (32 + (inv_range * 33)); dco++) {
			if (err_field[dco] == 0) {
				if (start == -1)
					start = dco;
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

		if ((inv_range == 0) && ((max_cnt < 3) || (err_field[32] == 0))) {
			do_inv = 1;
			inv_range = 1;
		} else {
			do_inv = 0;
		}

	} while (do_inv);

	dco = max_start + (max_cnt / 2);

#ifdef DCO_DEBUG
	for (cnt = 0; cnt <= (32 + (inv_range * 33)); cnt++)
		if (cnt == dco)
			printk("|");
		else
			printk("%c", err_field[cnt] ? '-' : 'o');
#endif
	if (dco > 32) {
		dco -= 33;
		if (conv->id !=	CHIPID_AD9652)
			ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN | INVERT_DCO_CLK);
		cnt = 1;
	} else {
		if (conv->id !=	CHIPID_AD9652)
			ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN);
		cnt = 0;
	}

#ifdef DCO_DEBUG
	printk(" %s DCO 0x%X CLK %lu Hz\n", cnt ? "INVERT" : "",
	       dco > 0 ? ((dco - 1) | dco_en) : 0, conv->adc_clk);
#endif

	ad9467_testmode_set(indio_dev, 0, TESTMODE_OFF);
	ad9467_testmode_set(indio_dev, 1, TESTMODE_OFF);
	ad9467_spi_write(conv->spi, ADC_REG_OUTPUT_DELAY,
			 dco > 0 ? ((dco - 1) | dco_en) : 0);
	ad9467_spi_write(conv->spi, ADC_REG_TRANSFER, TRANSFER_SYNC);

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

	switch (conv->id) {
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

	switch (conv->id) {
	case CHIPID_AD9234:
	case CHIPID_AD9467:
	case CHIPID_AD9250:
	case CHIPID_AD9683:
	case CHIPID_AD9680:
	case CHIPID_AD9625:
	case CHIPID_AD9434:
	case CHIPID_AD9649:
	case CHIPID_AD9684:
		return 0;
	case CHIPID_AD9265:
	case CHIPID_AD9652:
		dco_en = 0;
		break;
	default:
		dco_en = DCO_DELAY_ENABLE;
	}

	return ad9467_calibrate(indio_dev, chan, true, dco_en, 0);
}

static const int ad9265_scale_table[][2] = {
	{1250, 0x00}, {1500, 0x40}, {1750, 0x80}, {2000, 0xC0},
};

static const int ad9680_scale_table[][2] = {
	{1460, 0x08}, {1580, 0x09}, {1700, 0x0A}, {1820, 0x0B},
	{1940, 0x00}, {2060, 0x0C},
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

static irqreturn_t ad9680_event_handler(
		struct axiadc_converter *conv, unsigned chn)
{
	u64 event = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, chn,
			IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
	s64 timestamp = iio_get_time_ns();

	if (conv->indio_dev)
		iio_push_event(conv->indio_dev, event, timestamp);

	return IRQ_HANDLED;
}

static irqreturn_t ad9680_fdA_handler(int irq, void *private)
{
	return ad9680_event_handler(private, 0);
}

static irqreturn_t ad9680_fdB_handler(int irq, void *private)
{
	return ad9680_event_handler(private, 1);
}

static int ad9680_read_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	u16 low, high;

	mutex_lock(&indio_dev->mlock);
	low = (ad9467_spi_read(spi, AD9680_REG_THRESH_LOW_MSB) << 8) |
		ad9467_spi_read(spi, AD9680_REG_THRESH_LOW_LSB);
	high = (ad9467_spi_read(spi, AD9680_REG_THRESH_HI_MSB) << 8) |
		ad9467_spi_read(spi, AD9680_REG_THRESH_HI_LSB);
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

static int ad9680_read_thresh_en(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad9467_spi_read(spi, AD9680_REG_CHIP_PIN_CTRL);
	if (ret < 0)
		return ret;
	else
		return !(ret & AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel));
}

static int ad9680_write_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret = 0;
	int low, high;

	mutex_lock(&indio_dev->mlock);
	high = (ad9467_spi_read(spi, AD9680_REG_THRESH_HI_MSB) << 8) |
		ad9467_spi_read(spi, AD9680_REG_THRESH_HI_LSB);

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

		ad9467_spi_write(spi, AD9680_REG_THRESH_HI_MSB, val >> 8);
		ad9467_spi_write(spi, AD9680_REG_THRESH_HI_LSB, val & 0xFF);

		/* Calculate the new lower threshold limit */
		low = (ad9467_spi_read(spi, AD9680_REG_THRESH_LOW_MSB) << 8) |
			ad9467_spi_read(spi, AD9680_REG_THRESH_LOW_LSB);
		low = val - high + low;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	if (low < 0)
		low = 0;
	ad9467_spi_write(spi, AD9680_REG_THRESH_LOW_MSB, low >> 8);
	ad9467_spi_write(spi, AD9680_REG_THRESH_LOW_LSB, low & 0xFF);

unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad9680_write_thresh_en(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	mutex_lock(&indio_dev->mlock);

	ret = ad9467_spi_read(spi, AD9680_REG_CHIP_PIN_CTRL);
	if (ret < 0)
		goto err_unlock;

	if (state)
		ret &= ~AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel);
	else
		ret |= AD9680_REG_CHIP_PIN_CTRL_MASK(chan->channel);

	ret = ad9467_spi_write(spi, AD9680_REG_CHIP_PIN_CTRL, ret);
err_unlock:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static const char *const testmodes[] = {
	[TESTMODE_OFF] = "off",
	[TESTMODE_MIDSCALE_SHORT] = "midscale_short",
	[TESTMODE_POS_FULLSCALE] = "pos_fullscale",
	[TESTMODE_NEG_FULLSCALE] = "neg_fullscale",
	[TESTMODE_ALT_CHECKERBOARD] = "checkerboard",
	[TESTMODE_PN23_SEQ] = "pn_long",
	[TESTMODE_PN9_SEQ] = "pn_short",
	[TESTMODE_ONE_ZERO_TOGGLE] = "one_zero_toggle",
	[TESTMODE_USER] = "user",
	[TESTMODE_BIT_TOGGLE] = "bit_toggle",
	[TESTMODE_SYNC] = "sync",
	[TESTMODE_ONE_BIT_HIGH] = "one_bit_high",
	[TESTMODE_MIXED_BIT_FREQUENCY] = "mixed_bit_frequency",
	[TESTMODE_RAMP] = "ramp",
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
	if (conv->chip_info->max_testmode == TESTMODE_RAMP &&
	    mode > TESTMODE_USER && mode < TESTMODE_RAMP)
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

	switch (conv->id) {
	case CHIPID_AD9680:
	case CHIPID_AD9234:
	case CHIPID_AD9684:
		ret = ad9680_testmode_set(indio_dev, chan->channel, mode);
		break;
	default:
		ret = ad9467_testmode_set(indio_dev, chan->channel, mode);
		break;
	}

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

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9467] = {
		       .name = "AD9467",
		       .max_rate = 250000000UL,
		       .scale_table = ad9467_scale_table,
		       .num_scales = ARRAY_SIZE(ad9467_scale_table),
		       .max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN(0, 0, 16, 'S'),
		       },
	[ID_AD9643] = {
		       .name = "AD9643",
		       .max_rate = 250000000UL,
		       .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN(0, 0, 14, 'S'),
		       .channel[1] = AIM_CHAN(1, 1, 14, 'S'),
		       },
	[ID_AD9250] = {
		       .name = "AD9250",
		       .max_rate = 250000000UL,
		       .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
		       .channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'S', 0),
		       },
	[ID_AD9683] = {
		       .name = "AD9683",
		       .max_rate = 250000000UL,
		       .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
		       },
	[ID_AD9625] = {
		       .name = "AD9625",
		       .max_rate = 2500000000UL,
		       .scale_table = ad9643_scale_table,
		       .num_scales = ARRAY_SIZE(ad9643_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 'S', 0),
		       },
	[ID_AD9265] = {
		       .name = "AD9265",
		       .max_rate = 125000000UL,
		       .scale_table = ad9265_scale_table,
		       .num_scales = ARRAY_SIZE(ad9265_scale_table),
		       .max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 'S', 0),
		       },
	[ID_AD9434] = {
		       .name = "AD9434",
		       .max_rate = 500000000UL,
		       .scale_table = ad9434_scale_table,
		       .num_scales = ARRAY_SIZE(ad9434_scale_table),
		       .max_testmode = TESTMODE_USER,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 12, 'S', 0),
		       },
	[ID_AD9680_x2] = {
		       .name = "AD9680",
		       .max_rate = 1250000000UL,
		       .scale_table = ad9680_scale_table,
		       .num_scales = ARRAY_SIZE(ad9680_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 4,
		       .num_shadow_slave_channels = 2,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0,
				       ad9680_events, ARRAY_SIZE(ad9680_events)),
		       .channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'S', 0,
				       ad9680_events, ARRAY_SIZE(ad9680_events)),
		       .channel[2] = AIM_CHAN_NOCALIB(2, 2, 14, 'S', 0,
				       ad9680_events, ARRAY_SIZE(ad9680_events)),
		       .channel[3] = AIM_CHAN_NOCALIB(3, 3, 14, 'S', 0,
				       ad9680_events, ARRAY_SIZE(ad9680_events)),
	},
	[ID_AD9652] = {
		       .name = "AD9652",
		       .max_rate = 310000000UL,
		       .scale_table = ad9652_scale_table,
		       .num_scales = ARRAY_SIZE(ad9652_scale_table),
		       .max_testmode = TESTMODE_ONE_ZERO_TOGGLE,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN(0, 0, 16, 'S'),
		       .channel[1] = AIM_CHAN(1, 1, 16, 'S'),
		       },
	[ID_AD9649] = {
		       .name = "AD9649",
		       .max_rate = 80000000UL,
		       .scale_table = ad9649_scale_table,
		       .num_scales = ARRAY_SIZE(ad9649_scale_table),
		       .max_testmode = TESTMODE_MIXED_BIT_FREQUENCY,
		       .num_channels = 1,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0),
		       },
	[ID_AD9684] = {
		       .name = "AD9684",
		       .max_rate = 1250000000UL,
		       .scale_table = ad9680_scale_table,
		       .num_scales = ARRAY_SIZE(ad9680_scale_table),
		       .max_testmode = TESTMODE_RAMP,
		       .num_channels = 2,
		       .channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'S', 0, NULL, 0),
		       .channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'S', 0, NULL, 0),
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

	ret = clk_prepare_enable(conv->clk);
	if (ret == -EIO) /* Sync issue on the dual FMCADC5 */
		ret = 0;

	if (ret < 0)
		return ret;

	mdelay(10);

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "AD9625 PLL %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED");

	conv->sample_rate_read_only = true;

	return ret;
}

static int ad9680_setup_jesd204_link(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	unsigned long lane_rate_kHz;
	unsigned long sysref_rate;
	int ret;

	sysref_rate = DIV_ROUND_CLOSEST(sample_rate, 32);
	lane_rate_kHz = DIV_ROUND_CLOSEST(sample_rate, 100);

	if (lane_rate_kHz < 3125000 || lane_rate_kHz > 12500000) {
		dev_err(&conv->spi->dev, "Lane rate %lu Mbps out of bounds. Must be between 3125 and 12500 Mbps",
			lane_rate_kHz / 1000);
		return -EINVAL;;
	}

	if (lane_rate_kHz < 6250000)
		ad9467_spi_write(conv->spi, 0x56e, 0x10);	// low line rate mode must be enabled
	else
		ad9467_spi_write(conv->spi, 0x56e, 0x00);	// low line rate mode must be disabled

	ret = clk_set_rate(conv->sysref_clk, sysref_rate);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set SYSREF clock to %lu kHz: %d\n",
			sysref_rate / 1000, ret);
		return ret;
	}

	ret = clk_set_rate(conv->lane_clk, lane_rate_kHz);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to set lane rate to %lu kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	return 0;
}

static int ad9680_set_sample_rate(struct axiadc_converter *conv,
	unsigned int sample_rate)
{
	unsigned int pll_stat;
	int ret = 0;

	/*
	 * Minimum ADC samplerate is 300 MSPS. But the minimum lane rate is
	 * 3.125 Gbps, which results in a minumum ADC samplerate of 312.5 Msps when
	 * using 4 lanes. Lower the minimum here once support for dynamic lane
	 * enable/disable has been implemented.
	 */
	sample_rate = clamp(sample_rate, 312500000U, 1000000000U);
	sample_rate = clk_round_rate(conv->clk, sample_rate);

	/* Disable link */
	ad9467_spi_write(conv->spi, 0x571, 0x15);

	clk_disable_unprepare(conv->lane_clk);
	clk_disable_unprepare(conv->sysref_clk);
	clk_disable_unprepare(conv->clk);

	ret = clk_set_rate(conv->clk, sample_rate);
	if (ret) {
		dev_err(&conv->spi->dev, "Failed to set converter clock rate to %u kHz: %d\n",
			sample_rate / 1000, ret);
		return ret;
	}

	ret = ad9680_setup_jesd204_link(conv, sample_rate);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(conv->clk);
	if (ret) {
		dev_err(&conv->spi->dev, "Failed to enable converter clock: %d\n", ret);
		return ret;
	}
	ret = clk_prepare_enable(conv->sysref_clk);
	if (ret) {
		dev_err(&conv->spi->dev, "Failed to enable SYSREF clock: %d\n", ret);
		return ret;
	}

	// Enable link
	ad9467_spi_write(conv->spi, 0x571, 0x14);

	mdelay(20);
	pll_stat = ad9467_spi_read(conv->spi, 0x56f);

	dev_info(&conv->spi->dev, "PLL %s\n",
		 (pll_stat & 0x80) ? "LOCKED" : "UNLOCKED");

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret < 0) {
		dev_err(&conv->spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	conv->adc_clk = sample_rate;

	return 0;
}

static int ad9680_request_clks(struct axiadc_converter *conv)
{
	int ret;

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

	conv->clk = devm_clk_get(&conv->spi->dev, "adc_clk");
	if (IS_ERR(conv->clk) && PTR_ERR(conv->clk) != -ENOENT)
		return PTR_ERR(conv->clk);

	if (!IS_ERR(conv->clk)) {
		ret = clk_prepare_enable(conv->clk);
		if (ret < 0)
			return ret;

		conv->adc_clk = clk_get_rate(conv->clk);
	}

	conv->lane_clk = devm_clk_get(&conv->spi->dev, "jesd_adc_clk");
	if (IS_ERR(conv->lane_clk) && PTR_ERR(conv->lane_clk) != -ENOENT)
		return PTR_ERR(conv->lane_clk);

	return 0;
}

enum ad9680_sysref_mode {
	AD9680_SYSREF_DISABLED,
	AD9680_SYSREF_CONTINUOUS,
	AD9680_SYSREF_ONESHOT
};

struct ad9680_sysref_config {
	enum ad9680_sysref_mode mode;
	bool capture_falling_edge;
	bool valid_falling_edge;
};

struct ad9680_jesd204_link_config {
	uint8_t did;
	uint8_t bid;

	uint8_t num_lanes;
	uint8_t num_converters;
	uint8_t octets_per_frame;
	uint8_t frames_per_multiframe;;

	uint8_t bits_per_sample;
	uint8_t converter_resolution;

	uint8_t lid[4];
	uint8_t lane_mux[4];

	bool scrambling;
	uint8_t subclass;

	struct ad9680_sysref_config sysref;
};

static int ad9680_setup_link(struct spi_device *spi,
	const struct ad9680_jesd204_link_config *config)
{
	unsigned int val;
	unsigned int i;
	int ret;

	val = ilog2(config->octets_per_frame);
	val |= ilog2(config->num_converters) << 3;
	val |= ilog2(config->num_lanes) << 6;

	ret |= ad9467_spi_write(spi, 0x580, config->did);
	ret |= ad9467_spi_write(spi, 0x581, config->bid);

	ret = ad9467_spi_write(spi, 0x570, val); // Quick config

	for (i = 0; i < config->num_lanes; i++) {
		ret |= ad9467_spi_write(spi, 0x583 + i, config->lid[i]);

		val = config->lane_mux[i];
		val |= val << 4;
		ret |= ad9467_spi_write(spi, 0x5b2 + i + (i / 2), val);
	}

	val = config->num_lanes - 1;
	val |= config->scrambling ? 0x80 : 0x00;
	ret |= ad9467_spi_write(spi, 0x58b, val);

	ret |= ad9467_spi_write(spi, 0x58d, config->frames_per_multiframe - 1);
	ret |= ad9467_spi_write(spi, 0x58f, config->converter_resolution - 1);

	val = config->bits_per_sample - 1;
	val |= config->subclass ? 0x20 : 0x00;
	ret |= ad9467_spi_write(spi, 0x590, val);

	/* Disable SYSREF */
	ret |= ad9467_spi_write(spi, 0x120, 0x00);

	switch (config->sysref.mode) {
	case AD9680_SYSREF_CONTINUOUS:
		val = 0x02;
		break;
	case AD9680_SYSREF_ONESHOT:
		val = 0x04;
		break;
	default:
		val = 0x00;
		break;
	}

	if (config->sysref.capture_falling_edge)
		val |= 0x08;

	if (config->sysref.valid_falling_edge)
		val |= 0x10;
	ret |= ad9467_spi_write(spi, 0x120, val);

	return ret;
}

static int ad9680_setup(struct spi_device *spi, bool ad9234)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9680_jesd204_link_config link_config;
	unsigned int i;
	int ret, tmp = 1;
	unsigned pll_stat;
	static const u32 sfdr_optim_regs[8] =
		{0x16, 0x18, 0x19, 0x1A, 0x30, 0x11A, 0x934, 0x935};
	u32 sfdr_optim_vals[ARRAY_SIZE(sfdr_optim_regs)];

	ret = ad9680_request_clks(conv);
	if (ret)
		return ret;

#ifdef CONFIG_OF
	if (spi->dev.of_node)
		tmp = of_property_read_u32_array(
			spi->dev.of_node, "adi,sfdr-optimization-config",
			sfdr_optim_vals, ARRAY_SIZE(sfdr_optim_regs));
#endif

	ad9467_spi_write(spi, 0x000, 0x81);	// RESET
	mdelay(5);
	ad9467_spi_write(spi, 0x001, 0x01);	// RESET
	mdelay(1);

	ret = ad9467_spi_write(spi, 0x008, 0x03);	// select both channels
	ret |= ad9467_spi_write(spi, 0x201, 0x00);	// full sample rate (decimation = 1)

	if (tmp == 0) {
		for (; tmp < ARRAY_SIZE(sfdr_optim_regs); tmp++)
			ret |= ad9467_spi_write(spi, sfdr_optim_regs[tmp],
						sfdr_optim_vals[tmp]);
	}

	memset(&link_config, sizeof(link_config), 0x00);
	link_config.did = 0;
	link_config.bid = 1;
	link_config.num_lanes = 4;
	for (i = 0; i < link_config.num_lanes; i++) {
		link_config.lid[i] = i;
		link_config.lane_mux[i] = i;
	}
	link_config.num_converters = 2;
	link_config.octets_per_frame = 1;
	link_config.frames_per_multiframe = 32;
	link_config.converter_resolution = ad9234 ? 12 : 14;
	link_config.bits_per_sample = 16;
	link_config.scrambling = true;

	if (conv->sysref_clk) {
		link_config.subclass = 1;
		link_config.sysref.mode = AD9680_SYSREF_CONTINUOUS;
	} else {
		link_config.subclass = 0;
		link_config.sysref.mode = AD9680_SYSREF_DISABLED;
	}

	link_config.sysref.capture_falling_edge = true;
	link_config.sysref.valid_falling_edge = false;

	ret = ad9680_setup_link(spi, &link_config);
	if (ret < 0)
		return ret;

	ret = ad9680_setup_jesd204_link(conv, conv->adc_clk);
	if (ret < 0)
		return ret;
	mdelay(20);
	pll_stat = ad9467_spi_read(conv->spi, 0x56f);

	dev_info(&conv->spi->dev, "AD9680 PLL %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED");

	ret = clk_prepare_enable(conv->lane_clk);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	return ret;
}

static int ad9684_setup(struct spi_device *spi)
{
	unsigned clk_stat;
	int ret;

	ret = ad9467_spi_write(spi, 0x000, 0x81);
	mdelay(10);
	ret |= ad9467_spi_write(spi, 0x001, 0x02);
	mdelay(10);

	ret |= ad9467_spi_write(spi, 0x03f, 0x80);
	ret |= ad9467_spi_write(spi, 0x040, 0xbf);
	ret |= ad9467_spi_write(spi, 0x568, 0x01);

	clk_stat = ad9467_spi_read(spi, 0x11c);
	dev_info(&spi->dev, "AD9684 input clock %s\n",
		 clk_stat & 0x01 ? "DETECTED" : "NOT DETECTED");

	return ret;
}

static struct attribute *ad9467_attributes[] = {
	NULL,
};

static const struct attribute_group ad9467_attribute_group = {
	.attrs = ad9467_attributes,
};

static int ad9467_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned vref_val, vref_mask;
	unsigned int i;

	switch (conv->id) {
	case CHIPID_AD9234:
	case CHIPID_AD9680:
	case CHIPID_AD9684:
		vref_val = ad9467_spi_read(conv->spi, AD9680_REG_INPUT_FS_RANGE);
		break;
	default:
		vref_val = ad9467_spi_read(conv->spi, ADC_REG_VREF);

		switch (conv->id) {
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
		default:
			vref_mask = 0xFFFF;
			break;
		}

		vref_val &= vref_mask;
		break;
	}

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		if (vref_val == conv->chip_info->scale_table[i][1])
			break;
	}

	ad9467_scale(conv, i, val, val2);

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ad9467_set_scale(struct axiadc_converter *conv, int val, int val2)
{
	unsigned int scale_val[2];
	unsigned int i;

	if (val != 0)
		return -EINVAL;

	for (i = 0; i < conv->chip_info->num_scales; i++) {
		ad9467_scale(conv, i, &scale_val[0], &scale_val[1]);
		if (scale_val[0] != val || scale_val[1] != val2)
			continue;

		switch (conv->id) {
		case CHIPID_AD9234:
		case CHIPID_AD9680:
		case CHIPID_AD9684:
			ad9467_spi_write(conv->spi, AD9680_REG_INPUT_FS_RANGE,
					 conv->chip_info->scale_table[i][1]);
			break;
		default:
			ad9467_spi_write(conv->spi, ADC_REG_VREF,
					 conv->chip_info->scale_table[i][1]);
			ad9467_spi_write(conv->spi, ADC_REG_TRANSFER,
					 TRANSFER_SYNC);
			break;
		}
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

		*val = conv->adc_clk = clk_get_rate(conv->clk);

		return IIO_VAL_INT;

	}
	return -EINVAL;
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

		if (conv->id == CHIPID_AD9680)
			return ad9680_set_sample_rate(conv, val);

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

		return 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9680_request_fd_irqs(struct axiadc_converter *conv)
{
	struct device *dev = &conv->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);
		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9680_fdA_handler,
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
				irq, NULL, ad9680_fdB_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-b", conv);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9467_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret, i;

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

static int ad9467_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct clk *clk = NULL;
	bool master_slave_2x_quirk = false;
	int ret, clk_enabled = 0;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	if (!(spi_get_device_id(spi)->driver_data == CHIPID_AD9680
	     || spi_get_device_id(spi)->driver_data == CHIPID_AD9625)) {
		ret = clk_prepare_enable(clk);
		if (ret < 0)
			return ret;
		clk_enabled = 1;
		conv->adc_clk = clk_get_rate(clk);
	}

	spi_set_drvdata(spi, conv);
	conv->spi = spi;
	conv->clk = clk;

	conv->pwrdown_gpio = devm_gpiod_get_optional(&spi->dev, "powerdown",
		GPIOD_OUT_LOW);
	if (IS_ERR(conv->pwrdown_gpio))
		return PTR_ERR(conv->pwrdown_gpio);

	conv->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	if (conv->reset_gpio) {
		udelay(1);
		ret = gpiod_direction_output(conv->reset_gpio, 1);
	}

	mdelay(10);

	switch (spi_get_device_id(spi)->driver_data) {
	case CHIPID_AD9680:
	case CHIPID_AD9234:
	case CHIPID_AD9684:
		conv->id = ad9467_spi_read(spi, AD9680_REG_CHIP_ID_LOW);
		break;
	default:
		conv->id = ad9467_spi_read(spi, ADC_REG_CHIP_ID);
		break;
	}

	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n",
			conv->id);
		ret = -ENODEV;
		goto out;
	}

	switch (conv->id) {
	case CHIPID_AD9467:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9467];
		conv->adc_output_mode =
		    AD9467_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9643:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9643];
		conv->adc_output_mode =
		    AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ad9467_spi_write(spi, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN);
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9250:
		ret = ad9250_setup(spi, 2, 2);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}

		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9250];
		conv->adc_output_mode =
		    AD9250_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9683:
		ret = ad9250_setup(spi, 1, 1);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9683];
		conv->adc_output_mode =
		    AD9683_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9234:
		ret = ad9680_setup(spi, true);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9234];
		conv->adc_output_mode =
		    AD9680_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9680_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9680:
#ifdef CONFIG_OF
		if (spi->dev.of_node)
			master_slave_2x_quirk = of_property_read_bool(
				spi->dev.of_node, "adi,master-slave-2x-quirk");
#endif
		ret = ad9680_setup(spi, false);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}
		conv->chip_info = &axiadc_chip_info_tbl[master_slave_2x_quirk ?
			ID_AD9680_x2 : ID_AD9680];
		conv->adc_output_mode =
		    AD9680_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9680_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9625:
		ret = ad9625_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}

		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9625];
		conv->adc_output_mode =
		    AD9625_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9265:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9265];
		conv->adc_output_mode =
		    AD9265_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9434:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9434];
		conv->adc_output_mode =
		    AD9434_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9652:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9652];
		conv->adc_output_mode =
		    AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9649:
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9649];
		conv->adc_output_mode =
		    AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9467_outputmode_set(spi, conv->adc_output_mode);
		break;
	case CHIPID_AD9684:
		ret = ad9684_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
			goto out;
		}
		conv->chip_info = &axiadc_chip_info_tbl[ID_AD9684];
		conv->adc_output_mode =
		    AD9680_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		ret = ad9680_outputmode_set(spi, conv->adc_output_mode);
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		ret = -ENODEV;
		goto out;
	}

	if (ret < 0)
		goto out;

	conv->reg_access = ad9467_reg_access;
	conv->write_raw = ad9467_write_raw;
	conv->read_raw = ad9467_read_raw;
	conv->read_event_value = ad9680_read_thresh,
	conv->write_event_value = ad9680_write_thresh,
	conv->read_event_config = ad9680_read_thresh_en,
	conv->write_event_config = ad9680_write_thresh_en,
	conv->post_setup = ad9467_post_setup;

	switch (conv->id) {
	case CHIPID_AD9680:
	case CHIPID_AD9234:
	case CHIPID_AD9684:
		conv->set_pnsel = ad9680_set_pnsel;
		break;
	default:
		conv->set_pnsel = ad9467_set_pnsel;
		break;
	}

	conv->attrs = &ad9467_attribute_group;

	if (conv->id == CHIPID_AD9680) {
		ret = ad9680_request_fd_irqs(conv);
		if (ret < 0)
			goto out;

		/* Enable Fast Detect output */
		ret = ad9467_spi_write(spi, AD9680_REG_THRESH_CTRL, 0x1);
		if (ret < 0)
			goto out;
	}

	return 0;

out:
	if (clk_enabled)
		clk_disable_unprepare(clk);

	return ret;
}

static int ad9467_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	clk_disable_unprepare(conv->clk);

	return 0;
}

static const struct spi_device_id ad9467_id[] = {
	{"ad9467", CHIPID_AD9467},
	{"ad9643", CHIPID_AD9643},
	{"ad9250", CHIPID_AD9250},
	{"ad9265", CHIPID_AD9265},
	{"ad9683", CHIPID_AD9683},
	{"ad9434", CHIPID_AD9434},
	{"ad9625", CHIPID_AD9625},
	{"ad9680", CHIPID_AD9680},
	{"ad9652", CHIPID_AD9652},
	{"ad9234", CHIPID_AD9234},
	{"ad9649", CHIPID_AD9649},
	{"ad9684", CHIPID_AD9684},
	{}
};

MODULE_DEVICE_TABLE(spi, ad9467_id);

static struct spi_driver ad9467_driver = {
	.driver = {
		   .name = "ad9467",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9467_probe,
	.remove = ad9467_remove,
	.id_table = ad9467_id,
};

module_spi_driver(ad9467_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC");
MODULE_LICENSE("GPL v2");
