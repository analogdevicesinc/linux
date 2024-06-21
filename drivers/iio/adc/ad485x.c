// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AD485x DAS driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "cf_axi_adc.h"

#define AD485x_REG_INTERFACE_CONFIG_A	0x00
#define AD485x_REG_INTERFACE_CONFIG_B	0x01
#define AD485x_REG_DEVICE_CONFIG	0x02
#define AD485x_REG_CHIP_TYPE		0x03
#define AD485x_REG_PRODUCT_ID_L		0x04
#define AD485x_REG_PRODUCT_ID_H		0x05
#define AD485x_REG_CHIP_GRADE		0x06
#define AD485x_REG_SCRATCHPAD		0x0A
#define AD485x_REG_SPI_REVISION		0x0B
#define AD485x_REG_VENDORL_L		0x0C
#define AD485x_REG_VENDOR_H		0x0D
#define AD485x_REG_STREAM_MODE		0x0E
#define AD485x_REG_TRANSFER_CONFIG	0x0F
#define AD485x_REG_INTERFACE_CONFIG_C	0x10
#define AD485x_REG_SPI_STATUS		0x11
#define AD485x_REG_SPI_CONFIG_D		0x14
#define AD485x_REG_DEVICE_STATUS	0x20
#define AD485x_REG_CH_OR_STATUS		0x21
#define AD485x_REG_CH_UR_STATUS		0x22
#define AD485x_REG_REGMAP_CRC		0x23
#define AD485x_REG_DEVICE_CTRL		0x25
#define AD485x_REG_PACKET		0x26
#define AD485x_REG_OVERSAMPLE		0x27
#define AD485x_REG_SEAMLESS_HDR		0x28
#define AD485x_REG_CH_SLEEP		0x29
#define AD485x_REG_CH_CONFIG_BASE	0x2A
#define AD485x_REG_CHX_SOFTSPAN(ch)	(0x12 * ch) + AD485x_REG_CH_CONFIG_BASE
#define AD485x_REG_CHX_OFFSET(ch)	AD485x_REG_CHX_SOFTSPAN(ch) + 0x01
#define AD485x_REG_CHX_OFFSET_LSB(ch)	AD485x_REG_CHX_OFFSET(ch)
#define AD485x_REG_CHX_OFFSET_MID(ch)	AD485x_REG_CHX_OFFSET_LSB(ch) + 0x01
#define AD485x_REG_CHX_OFFSET_MSB(ch)	AD485x_REG_CHX_OFFSET_MID(ch) + 0x01
#define AD485x_REG_CHX_GAIN(ch)		AD485x_REG_CHX_OFFSET(ch) + 0x03
#define AD485x_REG_CHX_GAIN_LSB(ch)	AD485x_REG_CHX_GAIN(ch)
#define AD485x_REG_CHX_GAIN_MSB(ch)	AD485x_REG_CHX_GAIN(ch) + 0x01
#define AD485x_REG_CHX_PHASE(ch)	AD485x_REG_CHX_GAIN(ch) + 0x02
#define AD485x_REG_CHX_PHASE_LSB(ch)	AD485x_REG_CHX_PHASE(ch)
#define AD485x_REG_CHX_PHASE_MSB(ch)	AD485x_REG_CHX_PHASE_LSB(ch) + 0x01
#define AD485x_REG_CHX_OR(ch)		AD485x_REG_CHX_PHASE(ch) + 0x02
#define AD485x_REG_CHX_UR(ch)		AD485x_REG_CHX_OR(ch) + 0x04
#define AD485x_REG_CHX_TESTPAT(ch)	AD485x_REG_CHX_UR(ch) + 0x03

#define AD485x_MSK_OS_RATIO		GENMASK(0, 3)

#define AD485x_SW_RESET			BIT(7) | BIT(0)
#define AD485x_SDO_ENABLE		BIT(4)
#define AD485x_SINGLE_INSTRUCTION	BIT(7)
#define AD485x_ECHO_CLOCK_MODE		BIT(0)

#define AD485x_PACKET_FORMAT_0		0
#define AD485x_PACKET_FORMAT_1		1
#define AD485x_PACKET_FORMAT		GENMASK(1, 0)
#define AD485x_OS_EN			BIT(7)

#define AD485x_INT_REF_VOLTAGE_MV	4096
#define AD485x_T_CNVH_NS		40

#define AD485x_STATUS_ADDRESS_INVALID	BIT(0)
#define AD485x_STATUS_WR_TO_RD_ONLY	BIT(2)
#define AD485x_STATUS_CRC_ERR		BIT(3)
#define AD485x_STATUS_CLK_COUNT_ERR	BIT(4)
#define AD485x_STATUS_NOT_READY		BIT(7)

#define AD485x_STATUS_SLEEP		BIT(0)
#define AD485x_STATUS_POWERDOWN		BIT(1)
#define AD485x_STATUS_CH_OR_UR		BIT(2)
#define AD485x_STATUS_SPI		BIT(3)
#define AD485x_STATUS_REGMAP_CRC	BIT(4)
#define AD485x_STATUS_FUSE_CRC		BIT(5)
#define AD485x_STATUS_RESET		BIT(6)
#define AD485x_STATUS_READY		BIT(7)

#define AD485x_TEST_PAT			BIT(2)

#define AD4858_PACKET_SIZE_20		0
#define AD4858_PACKET_SIZE_24		1
#define AD4858_PACKET_SIZE_32		2

#define AD4857_PACKET_SIZE_16		0
#define AD4857_PACKET_SIZE_24		1

#define AD485x_AXI_REG_CNTRL_3		0x4C
#define AD485x_AXI_PACKET_CONFIG	GENMASK(1, 0)
#define AD485x_AXI_OVERSAMPLE_EN	BIT(2)
#define AD485x_AXI_CRC_EN		BIT(8)

#define AD485x_AXI_CRC_ERR		BIT(12)

#define AD485x_AXI_ADC_TWOS_COMPLEMENT	0x01

#define AD4858_PRODUCT_ID		0x60
#define AD4857_PRODUCT_ID		0x61
#define AD4856_PRODUCT_ID		0x62
#define AD4855_PRODUCT_ID		0x63
#define AD4854_PRODUCT_ID		0x64
#define AD4853_PRODUCT_ID		0x65
#define AD4852_PRODUCT_ID		0x66
#define AD4851_PRODUCT_ID		0x67
#define AD4858I_PRODUCT_ID		0x6F

static const unsigned long ad485x_throughput[] = {
	1000000, 1000000, 250000, 250000,
	1000000, 1000000, 250000, 250000,
	800000
};

enum ad485x_type {
	ID_AD4858,
	ID_AD4857,
	ID_AD4856,
	ID_AD4855,
	ID_AD4854,
	ID_AD4853,
	ID_AD4852,
	ID_AD4851,
	ID_AD4858I,
};

struct ad485x_dev {
	enum ad485x_type	type;
	unsigned int		sampling_freq;
	struct regulator	*refin;
	struct spi_device	*spi;
	struct pwm_device	*cnv;
	struct clk		*sampl_clk;
	unsigned int		vref_mv;
	unsigned int		softspan[8];
};

static const char *const ad485x_os_ratios[] = {
	"DISABLED", "2", "4", "8", "16", "32", "64", "128", "256", "512",
	"1024", "2048", "4096", "8192", "16384", "32786", "65536"
};

static const char *const ad4858_packet_fmts[] = {
	"20-bit", "24-bit", "32-bit"
};

static const char *const ad4857_packet_fmts[] = {
	"16-bit", "24-bit"
};

static int ad485x_spi_reg_write(struct ad485x_dev *adc, unsigned int addr,
				unsigned int val)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
	};

	tx_data[0] = ((addr >> 8) & 0xFF);
	tx_data[1] = addr & 0xFF;
	tx_data[2] = val;

	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static int ad485x_spi_reg_read(struct ad485x_dev *adc, unsigned int addr,
			       unsigned int *val)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
	};

	tx_data[0] = ((addr >> 8) & 0xFF) | BIT(7);
	tx_data[1] = addr & 0xFF;
	tx_data[2] = 0;

	if (spi_sync_transfer(adc->spi, &xfer, 1) < 0)
		return -EIO;

	*val = rx_data[2];

	return 0;
}

static int ad485x_set_sampling_freq(struct ad485x_dev *adc, unsigned int freq)
{
	struct pwm_state cnv_state = {
		.duty_cycle = AD485x_T_CNVH_NS,
		.time_unit = PWM_UNIT_NSEC,
		.enabled = true,
	};
	int ret;

	if (freq > ad485x_throughput[adc->type])
		freq = ad485x_throughput[adc->type];

	cnv_state.period = DIV_ROUND_CLOSEST_ULL(1000000000, freq),

	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	adc->sampling_freq = freq;

	return 0;
}

static int ad485x_set_oversampling_ratio(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 unsigned int ratio)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int spi_reg = 0;
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, AD485x_AXI_REG_CNTRL_3);
	axi_reg &= ~AD485x_AXI_OVERSAMPLE_EN;

	if (ratio) {
		axi_reg |= AD485x_AXI_OVERSAMPLE_EN;
		spi_reg = AD485x_OS_EN | (ratio - 1);
	}

	axiadc_write(st, AD485x_AXI_REG_CNTRL_3, axi_reg);

	return ad485x_spi_reg_write(adc, AD485x_REG_OVERSAMPLE, spi_reg);
}

static int ad485x_get_oversampling_ratio(struct iio_dev *indio_dev,
				    	 const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int val;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_OVERSAMPLE, &val);
	if (val & AD485x_OS_EN)
		return (val & 0xF) + 1;

	return 0;
}

static int ad485x_set_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int format)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int axi_reg;
	unsigned int spi_reg;
	int ret;

	axi_reg = axiadc_read(st, AD485x_AXI_REG_CNTRL_3);
	axi_reg &= ~AD485x_AXI_PACKET_CONFIG;
	axi_reg |= FIELD_PREP(AD485x_AXI_PACKET_CONFIG, format);

	ret = ad485x_spi_reg_read(adc, AD485x_REG_PACKET, &spi_reg);
	if (ret < 0)
		return ret;
	spi_reg &= ~AD485x_PACKET_FORMAT;
	spi_reg |= FIELD_PREP(AD485x_PACKET_FORMAT, format);

	axiadc_write(st, AD485x_AXI_REG_CNTRL_3, axi_reg);

	return ad485x_spi_reg_write(adc, AD485x_REG_PACKET, spi_reg);
}

static int ad485x_get_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int format;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_PACKET, &format);
	if (ret < 0)
		return ret;

	format &= AD485x_PACKET_FORMAT;

	return format;
}

static const char * const ad485x_softspan[] = {
	[1] = "M2.5-2.5",
	[2] = "0-5",
	[3] = "M5-5",
	[4] = "0-6.25",
	[5] = "M6.25-6.25",
	[6] = "0-10",
	[7] = "M10-10",
	[8] = "0-12.5",
	[9] = "M12.5-12.5",
	[10] = "0-20",
	[11] = "M20-20",
	[12] = "0-25",
	[13] = "M25-25",
	[14] = "0-40",
	[15] = "M40-40",
};

static int ad485x_softspan_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	int ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CHX_SOFTSPAN(chan->channel), item);
	if (ret)
		return ret;

	return 0;
}

static int ad485x_softspan_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int softspan;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CHX_SOFTSPAN(chan->channel),
		&softspan);
	if (ret)
		return ret;

	return softspan;
}

static const char * const axi_crc_control[] = {
	[0] = "disable",
	[1] = "enable",
	[2] = "clear_status",
};

static int axi_crc_control_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	unsigned int axi_reg;

	if (item == 2) {
		axi_reg = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
		axiadc_write(st, ADI_REG_CHAN_STATUS(0), axi_reg);
	} else {
		axi_reg = axiadc_read(st, AD485x_AXI_REG_CNTRL_3);
		axi_reg &= ~AD485x_AXI_CRC_EN;
		axi_reg |= FIELD_PREP(AD485x_AXI_CRC_EN, item);
		axiadc_write(st, AD485x_AXI_REG_CNTRL_3, axi_reg);
	}

	return 0;
}

static int axi_crc_control_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, AD485x_AXI_REG_CNTRL_3);
	axi_reg &= AD485x_AXI_CRC_EN;

	return axi_reg ? 1 : 0;
}

static ssize_t ad485x_get_axi_crc_status(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
	axi_reg &= AD485x_AXI_CRC_ERR;

	return sprintf(buf, "%s\n", axi_reg ? "error" : "ok");
}

static const char * const seamless_high_dynamic_range[] = {
	[0] = "disable",
	[1] = "enable",
};

static int seamless_high_dynamic_range_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int reg;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_SEAMLESS_HDR, &reg);
	if (ret)
		return ret;

	if (item)
		reg |= BIT(chan->address);
	else
		reg &= ~BIT(chan->address);

	ret = ad485x_spi_reg_write(adc, AD485x_REG_SEAMLESS_HDR, reg);
	if (ret)
		return ret;

	return 0;
}

static int seamless_high_dynamic_range_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int reg;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_SEAMLESS_HDR, &reg);
	if (ret)
		return ret;

	reg &= BIT(chan->address);

	return reg ? 1 : 0;
}

static const struct iio_enum seamless_high_dynamic_range_enum = {
	.items = seamless_high_dynamic_range,
	.num_items = ARRAY_SIZE(seamless_high_dynamic_range),
	.set = seamless_high_dynamic_range_set,
	.get = seamless_high_dynamic_range_get,
};

static const char * const channel_sleep[] = {
	[0] = "disable",
	[1] = "enable",
};

static int channel_sleep_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int reg;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CH_SLEEP, &reg);
	if (ret)
		return ret;

	if (item)
		reg |= BIT(chan->address);
	else
		reg &= ~BIT(chan->address);

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CH_SLEEP, reg);
	if (ret)
		return ret;

	return 0;
}

static int channel_sleep_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int reg;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CH_SLEEP, &reg);
	if (ret)
		return ret;

	reg &= BIT(chan->address);

	return reg ? 1 : 0;
}

static const struct iio_enum channel_sleep_enum = {
	.items = channel_sleep,
	.num_items = ARRAY_SIZE(channel_sleep),
	.set = channel_sleep_set,
	.get = channel_sleep_get,
};

static const struct iio_enum ad485x_os_ratio = {
	.items = ad485x_os_ratios,
	.num_items = ARRAY_SIZE(ad485x_os_ratios),
	.set = ad485x_set_oversampling_ratio,
	.get = ad485x_get_oversampling_ratio,
};

static const struct iio_enum ad4858_packet_fmt = {
	.items = ad4858_packet_fmts,
	.num_items = ARRAY_SIZE(ad4858_packet_fmts),
	.set = ad485x_set_packet_format,
	.get = ad485x_get_packet_format,
};

static const struct iio_enum ad4857_packet_fmt = {
	.items = ad4857_packet_fmts,
	.num_items = ARRAY_SIZE(ad4857_packet_fmts),
	.set = ad485x_set_packet_format,
	.get = ad485x_get_packet_format,
};

static const struct iio_enum ad485x_softspan_enum = {
	.items = ad485x_softspan,
	.num_items = ARRAY_SIZE(ad485x_softspan),
	.set = ad485x_softspan_write,
	.get = ad485x_softspan_read,
};

static const struct iio_enum axi_crc_control_enum = {
	.items = axi_crc_control,
	.num_items = ARRAY_SIZE(axi_crc_control),
	.set = axi_crc_control_set,
	.get = axi_crc_control_get,
};

static int ad485x_set_calibbias(struct ad485x_dev *adc,
	int ch, int val, int val2)
{
	unsigned int lsb, mid, msb;
	int ret;

	if (adc->type % 2) {
		lsb = 0;
		mid = val & 0xFF;
		msb = (val >> 8) & 0xFF;
	} else {
		lsb = (val << 4) & 0xFF;
		mid = (val >> 4) & 0xFF;
		msb = (val >> 12) & 0xFF;
	}

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CHX_OFFSET_LSB(ch), lsb);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CHX_OFFSET_MID(ch), mid);
	if (ret < 0)
		return ret;

	return ad485x_spi_reg_write(adc, AD485x_REG_CHX_OFFSET_MSB(ch), msb);
}

static int ad485x_get_calibbias(struct ad485x_dev *adc,
	int ch, int *val, int *val2)
{
	unsigned int lsb, mid, msb;
	int ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CHX_OFFSET_MSB(ch),
				  &msb);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CHX_OFFSET_MID(ch),
				  &mid);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_CHX_OFFSET_LSB(ch),
				  &lsb);
	if (ret < 0)
		return ret;

	if (adc->type % 2) {
		*val = msb << 8;
		*val |= mid;
		if (*val & 0x8000)
			*val |= ~((1 << 16) - 1);
	} else {
		*val = msb << 12;
		*val |= mid << 4;
		*val |= lsb >> 4;
		if (*val & 0x80000)
			*val |= ~((1 << 20) - 1);
	}

	return IIO_VAL_INT;
}

static int ad485x_set_calibscale(struct ad485x_dev *adc,
	int ch, int val, int val2)
{
	unsigned long long gain;
	unsigned int reg_val;
	int ret;

	gain = (val * 1000000 + val2);
	gain = gain * 32768;
	do_div(gain, 1000000);

	reg_val = gain;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CHX_GAIN_MSB(ch),
				   reg_val >> 8);
	if (ret < 0)
		return ret;

	return ad485x_spi_reg_write(adc, AD485x_REG_CHX_GAIN_LSB(ch),
				    reg_val & 0xFF);
}

static int ad485x_get_calibscale(struct ad485x_dev *adc,
	int ch, int *val, int *val2)
{
	unsigned int reg_val;
	int gain;
	int ret;

	ret = ad485x_spi_reg_read(adc,
		AD485x_REG_CHX_GAIN_MSB(ch), &reg_val);
	if (ret < 0)
		return ret;

	gain = (reg_val & 0xFF) << 8;
	ret = ad485x_spi_reg_read(adc,
		AD485x_REG_CHX_GAIN_LSB(ch), &reg_val);
	if (ret < 0)
		return ret;

	gain |= reg_val & 0xFF;

	*val = gain;
	*val2 = 32768;

	return IIO_VAL_FRACTIONAL;
}

static int ad485x_set_phase(struct ad485x_dev *adc, int addr, int val)
{
	int ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_CHX_PHASE_LSB(addr),
				   val & 0xFF);
	if (ret < 0)
		return ret;

	return ad485x_spi_reg_write(adc, AD485x_REG_CHX_PHASE_MSB(addr),
				    (val >> 8) & 0xFF);
}

static int ad485x_get_phase(struct ad485x_dev *adc, int addr, int *val)
{
	int readval;
	int ret;

	ret = ad485x_spi_reg_read(adc,
		AD485x_REG_CHX_PHASE_MSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val = readval << 8;
	ret = ad485x_spi_reg_read(adc,
		AD485x_REG_CHX_PHASE_LSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val |= readval;

	return 0;
}

static ssize_t ad485x_spi_status_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned long val;
	unsigned int reg;
	int ret, len = 0;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_SPI_STATUS, &reg);
	if (ret < 0)
		return ret;

	val = reg;
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"SPI status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tADDRESS_INVALID_ERR: %d\n",
		test_bit(AD485x_STATUS_ADDRESS_INVALID, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tWR_TO_RD_ONLY_REG_ERR: %d\n",
		test_bit(AD485x_STATUS_WR_TO_RD_ONLY, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCRC_ERR: %d\n",
		test_bit(AD485x_STATUS_CRC_ERR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCLOCK_COUNT_ERR: %d\n",
		test_bit(AD485x_STATUS_CLK_COUNT_ERR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tNOT_READY_ERR: %d\n",
		test_bit(AD485x_STATUS_NOT_READY, &val));

	buf[len - 1] = '\n';

	return len;
}

static DEVICE_ATTR(spi_status, 0444, ad485x_spi_status_read, NULL);

static ssize_t ad485x_device_status_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned long val;
	unsigned int reg;
	int ret, len = 0;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_DEVICE_STATUS, &reg);
	if (ret < 0)
		return ret;

	val = reg;
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "Device status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSLEEP: %d\n",
			 test_bit(AD485x_STATUS_SLEEP, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tPOWERDOWN: %d\n",
			 test_bit(AD485x_STATUS_POWERDOWN, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tCH_OR_UR: %d\n",
			 test_bit(AD485x_STATUS_CH_OR_UR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSPI: %d\n",
			 test_bit(AD485x_STATUS_SPI, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREGMAP_CRC: %d\n",
			 test_bit(AD485x_STATUS_REGMAP_CRC, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tFUSE_CRC: %d\n",
			 test_bit(AD485x_STATUS_FUSE_CRC, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tRESET: %d\n",
			 test_bit(AD485x_STATUS_RESET, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREADY: %d\n",
			 test_bit(AD485x_STATUS_READY, &val));

	buf[len - 1] = '\n';

	return len;
}

static DEVICE_ATTR(device_status, 0444, ad485x_device_status_read, NULL);

static int ad485x_setup(struct ad485x_dev *adc)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
		.cs_change = 1,
	};
	unsigned int product_id;
	int ret;

	ret = ad485x_set_sampling_freq(adc, 1000000);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_INTERFACE_CONFIG_A,
				   AD485x_SW_RESET);
	if (ret < 0)
		return ret;

	usleep_range(5000, 10000);
	ret = ad485x_spi_reg_write(adc, AD485x_REG_INTERFACE_CONFIG_B,
				   AD485x_SINGLE_INSTRUCTION);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_INTERFACE_CONFIG_A,
				   AD485x_SDO_ENABLE);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_read(adc, AD485x_REG_PRODUCT_ID_L, &product_id);
	if (ret < 0)
		return ret;

	switch (product_id) {
	case AD4858_PRODUCT_ID:
		adc->type = ID_AD4858;
		break;
	case AD4857_PRODUCT_ID:
		adc->type = ID_AD4857;
		break;
	case AD4856_PRODUCT_ID:
		adc->type = ID_AD4856;
		break;
	case AD4855_PRODUCT_ID:
		adc->type = ID_AD4855;
		break;
	case AD4854_PRODUCT_ID:
		adc->type = ID_AD4854;
		break;
	case AD4853_PRODUCT_ID:
		adc->type = ID_AD4853;
		break;
	case AD4852_PRODUCT_ID:
		adc->type = ID_AD4852;
		break;
	case AD4851_PRODUCT_ID:
		adc->type = ID_AD4851;
		break;
	case AD4858I_PRODUCT_ID:
		adc->type = ID_AD4858I;
		break;
	default:
		dev_err(&adc->spi->dev, "Unknown product ID: 0x%02X\n",
			product_id);
		return -EIO;
	}

	ret = ad485x_spi_reg_write(adc, AD485x_REG_DEVICE_CTRL,
				   AD485x_ECHO_CLOCK_MODE);
	if (ret < 0)
		return ret;

	ret = ad485x_spi_reg_write(adc, AD485x_REG_PACKET, 0);
	if (ret < 0)
		return ret;

	tx_data[0] = 0x0;
	tx_data[1] = 0xA;
	tx_data[2] = 0x0;

	/* Do a dummy spi transfer and change the cs low after */
	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static const long ad485x_softspan_range_mv[] = {
	[0] = 2500,   // "0-2.5",
	[1] = 5000,   // "M2.5-2.5",
	[2] = 5000,   // "0-5",
	[3] = 10000,  // "M5-5",
	[4] = 6250,   // "0-6.25",
	[5] = 12500,  // "M6.25-6.25",
	[6] = 10000,  // "0-10",
	[7] = 20000,  // "M10-10",
	[8] = 12500,  // "0-12.5",
	[9] = 25000,  // "M12.5-12.5",
	[10] = 20000, // "0-20",
	[11] = 40000, // "M20-20",
	[12] = 25000, // "0-25",
	[13] = 50000, // "M25-25",
	[14] = 40000, // "0-40",
	[15] = 80000, // "M40-40",
};

static int ad485x_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad485x_dev *adc = conv->phy;
	unsigned int softspan;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		axiadc_write(st, ADI_REG_CHAN_CNTRL(chan->channel), ADI_ENABLE);
		//*val = axiadc_read(st, ADI_REG_CHAN_RAW_DATA(chan->channel));
		*val = axiadc_read(st, (0x0408 + (chan->channel) * 0x40));
		axiadc_write(st, ADI_REG_CHAN_CNTRL(chan->channel), 0);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad485x_get_calibscale(adc, chan->channel, val, val2);
	case IIO_CHAN_INFO_SCALE:
		ret = ad485x_spi_reg_read(adc,
			AD485x_REG_CHX_SOFTSPAN(chan->channel), &softspan);
		if (ret)
			return ret;
		*val = ad485x_softspan_range_mv[softspan];
		*val2 = (1 << chan->scan_type.realbits) * 1000;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad485x_get_calibbias(adc, chan->channel, val, val2);
	case IIO_CHAN_INFO_CALIBPHASE:
		ad485x_get_phase(adc, chan->channel, val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad485x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad485x_set_sampling_freq(adc, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad485x_set_calibscale(adc, chan->channel, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad485x_set_calibbias(adc, chan->channel, val, val2);
	case IIO_CHAN_INFO_CALIBPHASE:
		return ad485x_set_phase(adc, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad485x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad485x_dev *adc = conv->phy;

	if (readval)
		return ad485x_spi_reg_read(adc, reg, readval);

	return ad485x_spi_reg_write(adc, reg, writeval);
}

static void ad485x_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad485x_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void ad485x_pwm_diasble(void *data)
{
	pwm_disable(data);
}

int find_opt(u8 *field, u32 size, u32 *ret_start)
{
       int i, cnt = 0, max_cnt = 0, start, max_start = 0;

       for(i = 0, start = -1; i < size; i++) {
               if (field[i] == 0) {
                       if (start == -1)
                               start = i;
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

       *ret_start = max_start;

       return max_cnt;
}

static int ad485x_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct ad485x_dev *adc = conv->phy;
	u8 pn_status[8][32];
	int opt_delay, s, c;
	unsigned int lane_num;
	unsigned int delay;
	unsigned int cmos;
	unsigned int val;
	unsigned int i;
	unsigned int j;
	int ret;

	cmos = (axiadc_read(st, ADI_REG_CONFIG) & ADI_CMOS_OR_LVDS_N) ? 1 : 0;
	if (cmos)
		lane_num = 8;
	else
		lane_num = 1;

	if (adc->type % 2) {
		axiadc_write(st, AD485x_AXI_REG_CNTRL_3,
			AD4857_PACKET_SIZE_24);
		ret = ad485x_spi_reg_write(adc, AD485x_REG_PACKET,
			AD485x_TEST_PAT | AD4857_PACKET_SIZE_24);
	} else {
		axiadc_write(st, AD485x_AXI_REG_CNTRL_3,
			AD4858_PACKET_SIZE_32);
		ret = ad485x_spi_reg_write(adc, AD485x_REG_PACKET,
			AD485x_TEST_PAT | AD4858_PACKET_SIZE_32);
	}
	if (ret < 0)
		return ret;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		ad485x_spi_reg_write(adc, 0x38 + i * 0x12, 0x2A);
		ad485x_spi_reg_write(adc, 0x39 + i * 0x12, 0x3C);
		ad485x_spi_reg_write(adc, 0x3a + i * 0x12, 0xCE);
		ad485x_spi_reg_write(adc, 0x3b + i * 0x12, 0x0A + (0x10 * i));
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE);
	}

	for (i = 0; i < lane_num; i++) {
		for (delay = 0; delay < 32; delay++) {
			val = axiadc_read(st, ADI_REG_CHAN_STATUS(i));
			axiadc_write(st, ADI_REG_CHAN_STATUS(i), val);
			axiadc_write(st, 0x800 + (i * 4), delay);
			mdelay(1);
			if (axiadc_read(st, ADI_REG_CHAN_STATUS(i)) & ADI_PN_ERR)
				pn_status[i][delay] = 1;
			else
				pn_status[i][delay] = 0;
		}
	}

	dev_info(&conv->spi->dev, "digital interface tuning:\n");

	pr_cont("  ");
	for (i = 0; i < 31; i++)
		pr_cont("%02d:", i);
	pr_cont("31\n");

	for (i = 0; i < lane_num; i++) {
		pr_info("%x:", i);
		for (j = 0; j < 32; j++) {
			if (pn_status[i][j])
			    pr_cont(" # ");
			else
				pr_cont(" o ");
		}
		pr_cont("\n");
	}

	for (i = 0; i < lane_num; i++) {
		c = find_opt(&pn_status[i][0], 32, &s);
		opt_delay = s + c / 2;
		axiadc_write(st, 0x800 + (i * 4), opt_delay);
		dev_info(&conv->spi->dev, "lane %d: selected delay: %d\n",
			i, opt_delay);
	}

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), 0);

	axiadc_write(st, AD485x_AXI_REG_CNTRL_3, 0);
	ret = ad485x_spi_reg_write(adc, AD485x_REG_PACKET, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad485x_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan, char *label)
{
	return sprintf(label, "%d\n", chan->channel);
}

#define AD485x_EXT_INFO							\
	IIO_ENUM("oversampling_ratio",					\
		IIO_SHARED_BY_ALL, &ad485x_os_ratio),			\
	IIO_ENUM_AVAILABLE_SHARED("oversampling_ratio",			\
		IIO_SHARED_BY_ALL, &ad485x_os_ratio),			\
	IIO_ENUM("softspan",						\
		IIO_SEPARATE, &ad485x_softspan_enum),			\
	IIO_ENUM_AVAILABLE_SHARED("softspan",				\
		IIO_SHARED_BY_TYPE, &ad485x_softspan_enum),		\
	IIO_ENUM("axi_crc_control",					\
		IIO_SHARED_BY_ALL, &axi_crc_control_enum),		\
	IIO_ENUM_AVAILABLE_SHARED("axi_crc_control",			\
		IIO_SHARED_BY_ALL, &axi_crc_control_enum),		\
	IIO_ENUM("seamless_high_dynamic_range",				\
		IIO_SEPARATE, &seamless_high_dynamic_range_enum),	\
	IIO_ENUM_AVAILABLE_SHARED("seamless_high_dynamic_range",	\
		IIO_SHARED_BY_TYPE, &seamless_high_dynamic_range_enum),	\
	IIO_ENUM("sleep",						\
		IIO_SEPARATE, &channel_sleep_enum),			\
	IIO_ENUM_AVAILABLE_SHARED("sleep",				\
		IIO_SHARED_BY_TYPE, &channel_sleep_enum),		\
	{								\
		.name = "axi_crc_status",				\
		.read = ad485x_get_axi_crc_status,			\
		.shared = IIO_SHARED_BY_ALL,				\
	}

static struct iio_chan_spec_ext_info ad4858_ext_info[] = {
	AD485x_EXT_INFO,
	IIO_ENUM("packet_format",			\
		IIO_SHARED_BY_ALL, &ad4858_packet_fmt),	\
	IIO_ENUM_AVAILABLE_SHARED("packet_format",	\
		IIO_SHARED_BY_ALL, &ad4858_packet_fmt),	\
	{},
};

static struct iio_chan_spec_ext_info ad4857_ext_info[] = {
	AD485x_EXT_INFO,
	IIO_ENUM("packet_format", IIO_SHARED_BY_ALL,	\
		 &ad4857_packet_fmt),			\
	IIO_ENUM_AVAILABLE_SHARED("packet_format",	\
		IIO_SHARED_BY_ALL, &ad4857_packet_fmt),	\
	{},
};

#define AD485x_IIO_CHANNEL(index, real, storage, info)		\
{								\
	.type = IIO_VOLTAGE,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
		BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
		BIT(IIO_CHAN_INFO_CALIBPHASE) |			\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.ext_info = info,				\
	.address = index,					\
	.indexed = 1,						\
	.channel = index,					\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = real,				\
		.storagebits = storage,				\
	},							\
}

static const struct axiadc_chip_info adc_chip_info[] = {
	[ID_AD4858] = {
		.name = "ad4858",
		.max_rate = ad485x_throughput[ID_AD4858],
		.num_channels = 8,
		.channel[0] = AD485x_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
		.channel[4] = AD485x_IIO_CHANNEL(4, 20, 32, ad4858_ext_info),
		.channel[5] = AD485x_IIO_CHANNEL(5, 20, 32, ad4858_ext_info),
		.channel[6] = AD485x_IIO_CHANNEL(6, 20, 32, ad4858_ext_info),
		.channel[7] = AD485x_IIO_CHANNEL(7, 20, 32, ad4858_ext_info),
	},
	[ID_AD4857] = {
		.name = "ad4857",
		.max_rate = ad485x_throughput[ID_AD4857],
		.num_channels = 8,
		.channel[0] = AD485x_IIO_CHANNEL(0, 16, 16, ad4857_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 16, 16, ad4857_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 16, 16, ad4857_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 16, 16, ad4857_ext_info),
		.channel[4] = AD485x_IIO_CHANNEL(4, 16, 16, ad4857_ext_info),
		.channel[5] = AD485x_IIO_CHANNEL(5, 16, 16, ad4857_ext_info),
		.channel[6] = AD485x_IIO_CHANNEL(6, 16, 16, ad4857_ext_info),
		.channel[7] = AD485x_IIO_CHANNEL(7, 16, 16, ad4857_ext_info),
	},
	[ID_AD4856] = {
		.name = "ad4856",
		.max_rate = ad485x_throughput[ID_AD4856],
		.num_channels = 8,
		.channel[0] = AD485x_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
		.channel[4] = AD485x_IIO_CHANNEL(4, 20, 32, ad4858_ext_info),
		.channel[5] = AD485x_IIO_CHANNEL(5, 20, 32, ad4858_ext_info),
		.channel[6] = AD485x_IIO_CHANNEL(6, 20, 32, ad4858_ext_info),
		.channel[7] = AD485x_IIO_CHANNEL(7, 20, 32, ad4858_ext_info),
	},
	[ID_AD4855] = {
		.name = "ad4855",
		.max_rate = ad485x_throughput[ID_AD4855],
		.num_channels = 8,
		.channel[0] = AD485x_IIO_CHANNEL(0, 16, 16, ad4857_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 16, 16, ad4857_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 16, 16, ad4857_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 16, 16, ad4857_ext_info),
		.channel[4] = AD485x_IIO_CHANNEL(4, 16, 16, ad4857_ext_info),
		.channel[5] = AD485x_IIO_CHANNEL(5, 16, 16, ad4857_ext_info),
		.channel[6] = AD485x_IIO_CHANNEL(6, 16, 16, ad4857_ext_info),
		.channel[7] = AD485x_IIO_CHANNEL(7, 16, 16, ad4857_ext_info),
	},
	[ID_AD4854] = {
		.name = "ad4854",
		.max_rate = ad485x_throughput[ID_AD4854],
		.num_channels = 4,
		.channel[0] = AD485x_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
	},
	[ID_AD4853] = {
		.name = "ad4853",
		.max_rate = ad485x_throughput[ID_AD4853],
		.num_channels = 4,
		.channel[0] = AD485x_IIO_CHANNEL(0, 16, 16, ad4857_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 16, 16, ad4857_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 16, 16, ad4857_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 16, 16, ad4857_ext_info),
	},
	[ID_AD4852] = {
		.name = "ad4852",
		.max_rate = ad485x_throughput[ID_AD4852],
		.num_channels = 4,
		.channel[0] = AD485x_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
	},
	[ID_AD4851] = {
		.name = "ad4851",
		.max_rate = ad485x_throughput[ID_AD4851],
		.num_channels = 4,
		.channel[0] = AD485x_IIO_CHANNEL(0, 16, 16, ad4857_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 16, 16, ad4857_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 16, 16, ad4857_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 16, 16, ad4857_ext_info),
	},
	[ID_AD4858I] = {
		.name = "ad4858i",
		.max_rate = ad485x_throughput[ID_AD4858I],
		.num_channels = 8,
		.channel[0] = AD485x_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
		.channel[1] = AD485x_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
		.channel[2] = AD485x_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
		.channel[3] = AD485x_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
		.channel[4] = AD485x_IIO_CHANNEL(4, 20, 32, ad4858_ext_info),
		.channel[5] = AD485x_IIO_CHANNEL(5, 20, 32, ad4858_ext_info),
		.channel[6] = AD485x_IIO_CHANNEL(6, 20, 32, ad4858_ext_info),
		.channel[7] = AD485x_IIO_CHANNEL(7, 20, 32, ad4858_ext_info),
	},
};

static int ad485x_probe(struct spi_device *spi)
{
	struct axiadc_converter	*conv;
	struct iio_dev *indio_dev;
	struct ad485x_dev *adc;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	spi_set_drvdata(spi, conv);
	adc->spi = spi;

	adc->refin = devm_regulator_get(&spi->dev, "refin");
	if (!IS_ERR(adc->refin)) {
		ret = regulator_enable(adc->refin);
		if (ret) {
			dev_err(&spi->dev, "Can't enable refin\n");
			return ret;
		}
		ret = regulator_get_voltage(adc->refin);
		if (ret < 0)
			return ret;

		adc->vref_mv = ret / 1000;
		ret = devm_add_action_or_reset(&spi->dev,
					       ad485x_regulator_disable,
					       adc->refin);
		if (ret)
			return ret;
	} else {
		if (PTR_ERR(adc->refin) != -ENODEV)
			return PTR_ERR(adc->refin);

		/* Use internal 4.096V reference */
		adc->vref_mv = AD485x_INT_REF_VOLTAGE_MV;
	}

	adc->sampl_clk = devm_clk_get(&spi->dev, "scki");
	if (IS_ERR(adc->sampl_clk))
		return PTR_ERR(adc->sampl_clk);

	ret = clk_prepare_enable(adc->sampl_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad485x_clk_disable,
				       adc->sampl_clk);
	if (ret)
		return ret;

	adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);

	ret = devm_add_action_or_reset(&spi->dev, ad485x_pwm_diasble,
				       adc->cnv);
	if (ret)
		return ret;

	ret = ad485x_setup(adc);
	if (ret < 0)
		return ret;

	conv->spi = spi;
	conv->clk = adc->sampl_clk;
	conv->chip_info = &adc_chip_info[adc->type];
	conv->adc_output_mode = AD485x_AXI_ADC_TWOS_COMPLEMENT;
	conv->reg_access = &ad485x_reg_access;
	conv->write_raw = &ad485x_write_raw;
	conv->post_setup = &ad485x_post_setup;
	conv->read_raw = &ad485x_read_raw;
	conv->read_label = &ad485x_read_label;
	conv->phy = adc;

	device_create_file(&spi->dev, &dev_attr_spi_status);
	device_create_file(&spi->dev, &dev_attr_device_status);

	return 0;
}

static const struct of_device_id ad485x_of_match[] = {
	{ .compatible = "adi,ad4858" },
	{ .compatible = "adi,ad4857" },
	{ .compatible = "adi,ad4856" },
	{ .compatible = "adi,ad4855" },
	{ .compatible = "adi,ad4854" },
	{ .compatible = "adi,ad4853" },
	{ .compatible = "adi,ad4852" },
	{ .compatible = "adi,ad4851" },
	{ .compatible = "adi,ad4858i" },
	{}
};

static struct spi_driver ad485x_driver = {
	.probe = ad485x_probe,
	.driver = {
		.name   = "ad485x",
		.of_match_table = ad485x_of_match,
	},
};
module_spi_driver(ad485x_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD485X DAS driver");
MODULE_LICENSE("Dual BSD/GPL");
