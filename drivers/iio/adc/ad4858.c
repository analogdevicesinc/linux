// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AD4858 DAQ driver
 *
 * Copyright 2022 Analog Devices Inc.
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

#define AD4858_REG_INTERFACE_CONFIG_A	0x00
#define AD4858_REG_INTERFACE_CONFIG_B	0x01
#define AD4858_REG_DEVICE_CONFIG	0x02
#define AD4858_REG_CHIP_TYPE		0x03
#define AD4858_REG_CHIP_GRADE		0x06
#define AD4858_REG_SCRATCHPAD		0x0A
#define AD4858_REG_SPI_REVISION		0x0B
#define AD4858_REG_VENDORL_L		0x0C
#define AD4858_REG_VENDOR_H		0x0D
#define AD4858_REG_STREAM_MODE		0x0E
#define AD4858_REG_TRANSFER_CONFIG	0x0F
#define AD4858_REG_INTERFACE_CONFIG_C	0x10
#define AD4858_REG_SPI_STATUS		0x11
#define AD4858_REG_SPI_CONFIG_D		0x14
#define AD4858_REG_DEVICE_STATUS	0x20
#define AD4858_REG_CH_OR_STATUS		0x21
#define AD4858_REG_CH_UR_STATUS		0x22
#define AD4858_REG_REGMAP_CRC		0x23
#define AD4858_REG_DEVICE_CTRL		0x25
#define AD4858_REG_PACKET		0x26
#define AD4858_REG_OVERSAMPLE		0x27
#define AD4858_REG_SEAMLESS_HDR		0x28
#define AD4858_REG_CH_SLEEP		0x29
#define AD4858_REG_CH_CONFIG_BASE	0x2A
#define AD4858_REG_CHX_SOFTSPAN(ch)	(0x12 * ch) + AD4858_REG_CH_CONFIG_BASE
#define AD4858_REG_CHX_OFFSET(ch)	AD4858_REG_CHX_SOFTSPAN(ch) + 0x01
#define AD4858_REG_CHX_OFFSET_LSB(ch)	AD4858_REG_CHX_OFFSET(ch)
#define AD4858_REG_CHX_OFFSET_MID(ch)	AD4858_REG_CHX_OFFSET_LSB(ch) + 0x01
#define AD4858_REG_CHX_OFFSET_MSB(ch)	AD4858_REG_CHX_OFFSET_MID(ch) + 0x01
#define AD4858_REG_CHX_GAIN(ch)		AD4858_REG_CHX_OFFSET(ch) + 0x03
#define AD4858_REG_CHX_GAIN_LSB(ch)	AD4858_REG_CHX_GAIN(ch)
#define AD4858_REG_CHX_GAIN_MSB(ch)	AD4858_REG_CHX_GAIN(ch) + 0x01
#define AD4858_REG_CHX_PHASE(ch)	AD4858_REG_CHX_GAIN(ch) + 0x02
#define AD4858_REG_CHX_PHASE_LSB(ch)	AD4858_REG_CHX_PHASE(ch)
#define AD4858_REG_CHX_PHASE_MSB(ch)	AD4858_REG_CHX_PHASE_LSB(ch) + 0x01
#define AD4858_REG_CHX_OR(ch)		AD4858_REG_CHX_PHASE(ch) + 0x02
#define AD4858_REG_CHX_UR(ch)		AD4858_REG_CHX_OR(ch) + 0x04
#define AD4858_REG_CHX_TESTPAT(ch)	AD4858_REG_CHX_UR(ch) + 0x03

#define AD4858_MSK_OS_RATIO		GENMASK(0, 3)

#define AD4858_SW_RESET			BIT(7) | BIT(0)
#define AD4858_SDO_ENABLE		BIT(4)
#define AD4858_SINGLE_INSTRUCTION	BIT(7)
#define AD4858_ECHO_CLOCK_MODE		BIT(0)

#define AD4858_PACKET_FORMAT_0		0
#define AD4858_PACKET_FORMAT_1		1
#define AD4858_PACKET_FORMAT		GENMASK(1, 0)
#define AD4858_OS_EN			BIT(7)

#define AD4858_INT_REF_VOLTAGE_MV	4096
#define AD4858_T_CNVH_NS		40

#define AD4858_STATUS_ADDRESS_INVALID	BIT(0)
#define AD4858_STATUS_WR_TO_RD_ONLY	BIT(2)
#define AD4858_STATUS_CRC_ERR		BIT(3)
#define AD4858_STATUS_CLK_COUNT_ERR	BIT(4)
#define AD4858_STATUS_NOT_READY		BIT(7)

#define AD4858_STATUS_SLEEP		BIT(0)
#define AD4858_STATUS_POWERDOWN		BIT(1)
#define AD4858_STATUS_CH_OR_UR		BIT(2)
#define AD4858_STATUS_SPI		BIT(3)
#define AD4858_STATUS_REGMAP_CRC	BIT(4)
#define AD4858_STATUS_FUSE_CRC		BIT(5)
#define AD4858_STATUS_RESET		BIT(6)
#define AD4858_STATUS_READY		BIT(7)

#define AD4858_AXI_REG_CNTRL_3		0x4C
#define AD4858_AXI_PACKET_CONFIG	GENMASK(1, 0)
#define AD4858_AXI_OVERSAMPLE_EN	BIT(2)
#define AD4858_AXI_CRC_EN		BIT(8)

#define AD4858_AXI_CRC_ERR		BIT(12)

#define AD4858_AXI_ADC_TWOS_COMPLEMENT	0x01

#define AD4858_IIO_CHANNEL(index)				\
{								\
	.type = IIO_VOLTAGE,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN) |		\
		BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
		BIT(IIO_CHAN_INFO_CALIBPHASE) |			\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.ext_info = ad4858_ext_info,				\
	.address = index,					\
	.indexed = 1,						\
	.channel = index,					\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 20,					\
		.storagebits = 32,				\
	},							\
}

struct ad4858_dev {
	unsigned int		sampling_freq;
	struct regulator	*refin;
	struct spi_device	*spi;
	struct pwm_device	*cnv;
	struct clk		*sampl_clk;
	unsigned int		vref_mv;
	unsigned int		softspan[8];
};

static const char *const ad4858_os_ratios[] = {
	"DISABLED", "2", "4", "8", "16", "32", "64", "128", "256", "512",
	"1024", "2048", "4096", "8192", "16384", "32786", "65536"
};

static const char *const ad4858_packet_fmts[] = {
	"20-bit", "24-bit", "32-bit"
};

static int ad4858_spi_reg_write(struct ad4858_dev *adc, unsigned int addr,
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

static int ad4858_spi_reg_read(struct ad4858_dev *adc, unsigned int addr,
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

static int ad4858_set_sampling_freq(struct ad4858_dev *adc, unsigned int freq)
{
	struct pwm_state cnv_state = {
		.period = DIV_ROUND_CLOSEST_ULL(1000000000, freq),
		.duty_cycle = AD4858_T_CNVH_NS,
		.time_unit = PWM_UNIT_NSEC,
		.enabled = true,
	};
	int ret;

	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	adc->sampling_freq = freq;

	return 0;
}

static int ad4858_set_oversampling_ratio(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 unsigned int ratio)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int spi_reg = 0;
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, AD4858_AXI_REG_CNTRL_3);
	axi_reg &= ~AD4858_AXI_OVERSAMPLE_EN;

	if (ratio) {
		axi_reg |= AD4858_AXI_OVERSAMPLE_EN;
		spi_reg = AD4858_OS_EN | (ratio - 1);
	}

	axiadc_write(st, AD4858_AXI_REG_CNTRL_3, axi_reg);

	return ad4858_spi_reg_write(adc, AD4858_REG_OVERSAMPLE, spi_reg);
}

static int ad4858_get_oversampling_ratio(struct iio_dev *indio_dev,
				    	 const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int val;
	int ret;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_OVERSAMPLE, &val);
	if (val & AD4858_OS_EN)
		return (val & 0xF) + 1;

	return 0;
}

static int ad4858_set_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int format)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int axi_reg;
	unsigned int spi_reg;
	int ret;

	axi_reg = axiadc_read(st, AD4858_AXI_REG_CNTRL_3);
	axi_reg &= ~AD4858_AXI_PACKET_CONFIG;
	axi_reg |= FIELD_PREP(AD4858_AXI_PACKET_CONFIG, format);

	ret = ad4858_spi_reg_read(adc, AD4858_REG_PACKET, &spi_reg);
	if (ret < 0)
		return ret;
	spi_reg &= ~AD4858_PACKET_FORMAT;
	spi_reg |= FIELD_PREP(AD4858_PACKET_FORMAT, format);

	axiadc_write(st, AD4858_AXI_REG_CNTRL_3, axi_reg);

	return ad4858_spi_reg_write(adc, AD4858_REG_PACKET, spi_reg);
}

static int ad4858_get_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int format;
	int ret;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_PACKET, &format);
	if (ret < 0)
		return ret;

	format &= AD4858_PACKET_FORMAT;

	return format;
}

static const char * const ad4858_softspan[] = {
	[0] = "0-2.5",
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

static int ad4858_softspan_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	int ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_CHX_SOFTSPAN(chan->channel), item);
	if (ret)
		return ret;

	return 0;
}

static int ad4858_softspan_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int softspan;
	int ret;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_CHX_SOFTSPAN(chan->channel),
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
		axi_reg = axiadc_read(st, AD4858_AXI_REG_CNTRL_3);
		axi_reg &= ~AD4858_AXI_CRC_EN;
		axi_reg |= FIELD_PREP(AD4858_AXI_CRC_EN, item);
		axiadc_write(st, AD4858_AXI_REG_CNTRL_3, axi_reg);
	}

	return 0;
}

static int axi_crc_control_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, AD4858_AXI_REG_CNTRL_3);
	axi_reg &= AD4858_AXI_CRC_EN;

	return axi_reg ? 1 : 0;
}

static ssize_t ad4858_get_axi_crc_status(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	unsigned int axi_reg;

	axi_reg = axiadc_read(st, ADI_REG_CHAN_STATUS(0));
	axi_reg &= AD4858_AXI_CRC_ERR;

	return sprintf(buf, "%s\n", axi_reg ? "error" : "ok");
}

static const struct iio_enum ad4858_os_ratio = {
	.items = ad4858_os_ratios,
	.num_items = ARRAY_SIZE(ad4858_os_ratios),
	.set = ad4858_set_oversampling_ratio,
	.get = ad4858_get_oversampling_ratio,
};

static const struct iio_enum ad4858_packet_fmt = {
	.items = ad4858_packet_fmts,
	.num_items = ARRAY_SIZE(ad4858_packet_fmts),
	.set = ad4858_set_packet_format,
	.get = ad4858_get_packet_format,
};

static const struct iio_enum ad4858_softspan_enum = {
	.items = ad4858_softspan,
	.num_items = ARRAY_SIZE(ad4858_softspan),
	.set = ad4858_softspan_write,
	.get = ad4858_softspan_read,
};

static const struct iio_enum axi_crc_control_enum = {
	.items = axi_crc_control,
	.num_items = ARRAY_SIZE(axi_crc_control),
	.set = axi_crc_control_set,
	.get = axi_crc_control_get,
};

static struct iio_chan_spec_ext_info ad4858_ext_info[] = {
	IIO_ENUM("oversampling_ratio", IIO_SHARED_BY_ALL,
		 &ad4858_os_ratio),
	IIO_ENUM_AVAILABLE_SHARED("oversampling_ratio", IIO_SHARED_BY_ALL,
				  &ad4858_os_ratio),
	IIO_ENUM("packet_format", IIO_SHARED_BY_ALL,
		 &ad4858_packet_fmt),
	IIO_ENUM_AVAILABLE_SHARED("packet_format", IIO_SHARED_BY_ALL,
				  &ad4858_packet_fmt),
	IIO_ENUM("softspan", IIO_SEPARATE, &ad4858_softspan_enum),
	IIO_ENUM_AVAILABLE_SHARED("softspan", IIO_SHARED_BY_TYPE,
				  &ad4858_softspan_enum),

	IIO_ENUM("axi_crc_control", IIO_SHARED_BY_ALL,
		 &axi_crc_control_enum),
	IIO_ENUM_AVAILABLE_SHARED("axi_crc_control", IIO_SHARED_BY_ALL,
				  &axi_crc_control_enum),

	{
		.name = "axi_crc_status",
		.read = ad4858_get_axi_crc_status,
		.shared = IIO_SHARED_BY_ALL,
	},
	{},
};

static int ad4858_set_offset(struct ad4858_dev *adc, int addr, int val)
{
	int ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_CHX_OFFSET_LSB(addr),
				   (val << 3) & 0xFF);
	if (ret < 0)
		return ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_CHX_OFFSET_MID(addr),
				   (val >> 5) & 0xFF);
	if (ret < 0)
		return ret;

	return ad4858_spi_reg_write(adc, AD4858_REG_CHX_OFFSET_MSB(addr),
				    (val >> 13) & 0xFF);
}

static int ad4858_get_offset(struct ad4858_dev *adc, int addr, int *val)
{
	unsigned int readval;
	int ret;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_CHX_OFFSET_MSB(addr),
				  &readval);
	if (ret < 0)
		return ret;

	*val = readval << 16;
	ret = ad4858_spi_reg_read(adc, AD4858_REG_CHX_OFFSET_MID(addr),
				  &readval);
	if (ret < 0)
		return ret;

	*val |= readval << 8;
	ret = ad4858_spi_reg_read(adc, AD4858_REG_CHX_OFFSET_LSB(addr),
				  &readval);
	if (ret < 0)
		return ret;

	*val |= readval;
	*val = *val >> 3;

	return 0;
}

static int ad4858_set_gain(struct ad4858_dev *adc, int addr, int val)
{
	int ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_CHX_GAIN_MSB(addr),
				   val >> 8);
	if (ret < 0)
		return ret;

	return ad4858_spi_reg_write(adc, AD4858_REG_CHX_GAIN_LSB(addr),
				    val & 0xFF);
}

static int ad4858_get_gain(struct ad4858_dev *adc, int addr, int *val)
{
	int readval;
	int ret;

	ret = ad4858_spi_reg_read(adc,
		AD4858_REG_CHX_GAIN_MSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val = readval << 8;
	ret = ad4858_spi_reg_read(adc,
		AD4858_REG_CHX_GAIN_LSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val |= readval & 0xFF;

	return 0;
}

static int ad4858_set_phase(struct ad4858_dev *adc, int addr, int val)
{
	int ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_CHX_PHASE_LSB(addr),
				   val & 0xFF);
	if (ret < 0)
		return ret;

	return ad4858_spi_reg_write(adc, AD4858_REG_CHX_PHASE_MSB(addr),
				    (val >> 8) & 0xFF);
}

static int ad4858_get_phase(struct ad4858_dev *adc, int addr, int *val)
{
	int readval;
	int ret;

	ret = ad4858_spi_reg_read(adc,
		AD4858_REG_CHX_PHASE_MSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val = readval << 8;
	ret = ad4858_spi_reg_read(adc,
		AD4858_REG_CHX_PHASE_LSB(addr), &readval);
	if (ret < 0)
		return ret;

	*val |= readval;

	return 0;
}

static ssize_t ad4858_spi_status_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned long val;
	unsigned int reg;
	int ret, len = 0;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_SPI_STATUS, &reg);
	if (ret < 0)
		return ret;

	val = reg;
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"SPI status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tADDRESS_INVALID_ERR: %d\n",
		test_bit(AD4858_STATUS_ADDRESS_INVALID, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tWR_TO_RD_ONLY_REG_ERR: %d\n",
		test_bit(AD4858_STATUS_WR_TO_RD_ONLY, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCRC_ERR: %d\n",
		test_bit(AD4858_STATUS_CRC_ERR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCLOCK_COUNT_ERR: %d\n",
		test_bit(AD4858_STATUS_CLK_COUNT_ERR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tNOT_READY_ERR: %d\n",
		test_bit(AD4858_STATUS_NOT_READY, &val));

	buf[len - 1] = '\n';

	return len;
}

static DEVICE_ATTR(spi_status, 0444, ad4858_spi_status_read, NULL);

static ssize_t ad4858_device_status_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned long val;
	unsigned int reg;
	int ret, len = 0;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_DEVICE_STATUS, &reg);
	if (ret < 0)
		return ret;

	val = reg;
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "Device status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSLEEP: %d\n",
			 test_bit(AD4858_STATUS_SLEEP, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tPOWERDOWN: %d\n",
			 test_bit(AD4858_STATUS_POWERDOWN, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tCH_OR_UR: %d\n",
			 test_bit(AD4858_STATUS_CH_OR_UR, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSPI: %d\n",
			 test_bit(AD4858_STATUS_SPI, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREGMAP_CRC: %d\n",
			 test_bit(AD4858_STATUS_REGMAP_CRC, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tFUSE_CRC: %d\n",
			 test_bit(AD4858_STATUS_FUSE_CRC, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tRESET: %d\n",
			 test_bit(AD4858_STATUS_RESET, &val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREADY: %d\n",
			 test_bit(AD4858_STATUS_READY, &val));

	buf[len - 1] = '\n';

	return len;
}

static DEVICE_ATTR(device_status, 0444, ad4858_device_status_read, NULL);

static int ad4858_setup(struct ad4858_dev *adc)
{
	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
		.cs_change = 1,
	};
	int ret;

	ret = ad4858_set_sampling_freq(adc, 500000);
	if (ret < 0)
		return ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_INTERFACE_CONFIG_A,
				   AD4858_SW_RESET);
	if (ret < 0)
		return ret;

	usleep_range(5000, 10000);
	ret = ad4858_spi_reg_write(adc, AD4858_REG_INTERFACE_CONFIG_B,
				   AD4858_SINGLE_INSTRUCTION);
	if (ret < 0)
		return ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_INTERFACE_CONFIG_A,
				   AD4858_SDO_ENABLE);
	if (ret < 0)
		return ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_DEVICE_CTRL,
				   AD4858_ECHO_CLOCK_MODE);
	if (ret < 0)
		return ret;

	ret = ad4858_spi_reg_write(adc, AD4858_REG_PACKET, 0);
	if (ret < 0)
		return ret;

	tx_data[0] = 0x0;
	tx_data[1] = 0xA;
	tx_data[2] = 0x0;

	/* Do a dummy spi transfer and change the cs low after */
	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static const long ad4858_softspan_range_mv[] = {
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

static int ad4858_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(conv->indio_dev);
	struct ad4858_dev *adc = conv->phy;
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
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ad4858_get_gain(adc, chan->channel, val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = ad4858_spi_reg_read(adc,
			AD4858_REG_CHX_SOFTSPAN(chan->channel), &softspan);
		if (ret)
			return ret;
		*val = ad4858_softspan_range_mv[softspan];
		*val2 = (1 << chan->scan_type.realbits) * 1000;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		ad4858_get_offset(adc, chan->channel, val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBPHASE:
		ad4858_get_phase(adc, chan->channel, val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4858_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4858_set_sampling_freq(adc, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return ad4858_set_gain(adc, chan->channel, val);
	case IIO_CHAN_INFO_OFFSET:
		return 0;
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad4858_set_offset(adc, chan->channel, val);
	case IIO_CHAN_INFO_CALIBPHASE:
		return ad4858_set_phase(adc, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad4858_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;

	if (readval)
		return ad4858_spi_reg_read(adc, reg, readval);

	return ad4858_spi_reg_write(adc, reg, writeval);
}

static const struct iio_info ad4858_info = {
	.read_raw = ad4858_read_raw,
	.write_raw = ad4858_write_raw,
	.debugfs_reg_access = &ad4858_reg_access,
};

static void ad4858_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad4858_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void ad4858_pwm_diasble(void *data)
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

static int ad4858_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct ad4858_dev *adc = conv->phy;
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

	axiadc_write(st, AD4858_AXI_REG_CNTRL_3, 2);
	ret = ad4858_spi_reg_write(adc, AD4858_REG_PACKET, 6);
	if (ret < 0)
		return ret;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		ad4858_spi_reg_write(adc, 0x38 + i * 0x12, 0x2A);
		ad4858_spi_reg_write(adc, 0x39 + i * 0x12, 0x3C);
		ad4858_spi_reg_write(adc, 0x3a + i * 0x12, 0xCE);
		ad4858_spi_reg_write(adc, 0x3b + i * 0x12, 0x0A + (0x10 * i));
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

	dev_info(&conv->spi->dev, "ad4858 digital interface tuning:\n");

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

	axiadc_write(st, AD4858_AXI_REG_CNTRL_3, 0);
	ret = ad4858_spi_reg_write(adc, AD4858_REG_PACKET, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad4858_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan, char *label)
{
	return sprintf(label, "%d\n", chan->channel);
}

static const struct axiadc_chip_info adc_chip_info = {
	.name = "ad4858",
	.max_rate = 1000000UL,
	.num_channels = 8,
	.channel[0] = AD4858_IIO_CHANNEL(0),
	.channel[1] = AD4858_IIO_CHANNEL(1),
	.channel[2] = AD4858_IIO_CHANNEL(2),
	.channel[3] = AD4858_IIO_CHANNEL(3),
	.channel[4] = AD4858_IIO_CHANNEL(4),
	.channel[5] = AD4858_IIO_CHANNEL(5),
	.channel[6] = AD4858_IIO_CHANNEL(6),
	.channel[7] = AD4858_IIO_CHANNEL(7),
};

static int ad4858_probe(struct spi_device *spi)
{
	struct axiadc_converter	*conv;
	struct iio_dev *indio_dev;
	struct ad4858_dev *adc;
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
					       ad4858_regulator_disable,
					       adc->refin);
		if (ret)
			return ret;
	} else {
		if (PTR_ERR(adc->refin) != -ENODEV)
			return PTR_ERR(adc->refin);

		/* Use internal 4.096V reference */
		adc->vref_mv = AD4858_INT_REF_VOLTAGE_MV;
	}

	adc->sampl_clk = devm_clk_get(&spi->dev, "scki");
	if (IS_ERR(adc->sampl_clk))
		return PTR_ERR(adc->sampl_clk);

	ret = clk_prepare_enable(adc->sampl_clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad4858_clk_disable,
				       adc->sampl_clk);
	if (ret)
		return ret;

	adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);

	ret = devm_add_action_or_reset(&spi->dev, ad4858_pwm_diasble,
				       adc->cnv);

	if (ret)
		return ret;

	ret = ad4858_setup(adc);
	if (ret < 0)
		return ret;

	conv->spi = spi;
	conv->clk = adc->sampl_clk;
	conv->chip_info = &adc_chip_info;
	conv->adc_output_mode = AD4858_AXI_ADC_TWOS_COMPLEMENT;
	conv->reg_access = &ad4858_reg_access;
	conv->write_raw = &ad4858_write_raw;
	conv->post_setup = &ad4858_post_setup;
	conv->read_raw = &ad4858_read_raw;
	conv->read_label = &ad4858_read_label;
	conv->phy = adc;

	device_create_file(&spi->dev, &dev_attr_spi_status);
	device_create_file(&spi->dev, &dev_attr_device_status);

	return 0;
}

static const struct of_device_id ad4858_of_match[] = {
	{ .compatible = "adi,ad4858" },
	{}
};

static struct spi_driver ad4858_driver = {
	.probe = ad4858_probe,
	.driver = {
		.name   = "ad4858",
		.of_match_table = ad4858_of_match,
	},
};
module_spi_driver(ad4858_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4858 DAQ");
MODULE_LICENSE("Dual BSD/GPL");
