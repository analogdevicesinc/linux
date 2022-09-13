// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices AD4858 DAQ driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/device.h>
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
#include <linux/regmap.h>
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
#define AD4858_REG_CHX_GAIN(ch)		AD4858_REG_CHX_SOFTSPAN(ch) + 0x03
#define AD4858_REG_CHX_PHASE(ch)	AD4858_REG_CHX_SOFTSPAN(ch) + 0x02
#define AD4858_REG_CHX_OR(ch)		AD4858_REG_CHX_SOFTSPAN(ch) + 0x02
#define AD4858_REG_CHX_UR(ch)		AD4858_REG_CHX_SOFTSPAN(ch) + 0x04
#define AD4858_REG_CHX_TESTPAT(ch)	AD4858_REG_CHX_SOFTSPAN(ch) + 0x03

#define AD4858_MSK_OS_RATIO		GENMASK(0, 4)

#define AD4858_SW_RESET			BIT(7) | BIT(0)
#define AD4858_SDO_ENABLE		BIT(4)
#define AD4858_SINGLE_INSTRUCTION	BIT(7)
#define AD4858_ECHO_CLOCK_MODE		BIT(0)
#define AD4858_PACKET_FORMAT_0		0
#define AD4858_OS_EN			BIT(7)

#define AD4858_INT_REF_VOLTAGE_MV	4096
#define AD4858_T_CNVH_NS		40

#define AD4858_AXI_ADC_TWOS_COMPLEMENT	0x01

#define AD4858_IIO_CHANNEL(index)                                       \
{									\
        .type = IIO_VOLTAGE,                                            \
	.info_mask_separate = BIT(IIO_CHAN_INFO_OFFSET)		|	\
			BIT(IIO_CHAN_INFO_HARDWAREGAIN)		|	\
			BIT(IIO_CHAN_INFO_PHASE),			\
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),           \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ)	|	\
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.address = index,						\
	.indexed = 1,							\
	.channel = index,						\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 20,						\
		.storagebits = 32,					\
	},				                        	\
}

struct ad4858_dev {
        struct regulator *refin;
	struct spi_device *spi;
        struct pwm_device *cnv;
	struct regmap *regmap;
	struct clk *sampl_clk;
        unsigned int vref_mv;
};

static const struct regmap_config ad4858_regmap_config = {
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(15),
	.max_register = 0xB6
};

static const char * const ad4858_os_ratios[] = {
	"DISABLED", "2", "4", "8", "16", "32", "64", "128", "256", "512",
	"1024", "2048", "4096", "8192", "16384", "32786", "65536"
};

static const struct iio_chan_spec ad4858_iio_channels[] = {
	AD4858_IIO_CHANNEL(0),
	AD4858_IIO_CHANNEL(1),
	AD4858_IIO_CHANNEL(2),
	AD4858_IIO_CHANNEL(3),
	AD4858_IIO_CHANNEL(4),
	AD4858_IIO_CHANNEL(5),
	AD4858_IIO_CHANNEL(6),
	AD4858_IIO_CHANNEL(7),
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
		.period = DIV_ROUND_CLOSEST_ULL(1000000000ULL, freq),
		.duty_cycle = AD4858_T_CNVH_NS,
		.time_unit = PWM_UNIT_NSEC,
		.enabled = true,
	};

	return pwm_apply_state(adc->cnv, &cnv_state);
}

static int ad4858_set_oversampling_ratio(struct iio_dev *indio_dev,
					 unsigned int ratio)
{
	unsigned int reg = 0;
	int ret;

	if (ratio > 10)
		return -EINVAL;

	// if (ratio) {
	// 	reg = AD4858_OS_EN;
	// 	ret = find_first_bit(&ratio, 32);
	// 	pr_err("\n[adc] first bit %d", ret);
	// }

	// ret = ad4858_spi_reg_write(adc, ,);
	// if (ret < 0)
	// 	return ret;
	return 0;
}

static int ad4858_get_oversampling_ratio(struct iio_dev *indio_dev,
					 unsigned int *ratio)
{
	return 0;
}

#define AD4858_STATUS_ADDRESS_INVALID 		BIT(0)
#define AD4858_STATUS_WR_TO_RD_ONLY		BIT(2)
#define AD4858_STATUS_CRC_ERR			BIT(3)
#define AD4858_STATUS_CLK_COUNT_ERR		BIT(4)
#define AD4858_STATUS_NOT_READY			BIT(7)

#define AD4858_STATUS_SLEEP			BIT(0)
#define AD4858_STATUS_POWERDOWN			BIT(1)
#define AD4858_STATUS_CH_OR_UR			BIT(2)
#define AD4858_STATUS_SPI			BIT(3)
#define AD4858_STATUS_REGMAP_CRC		BIT(4)
#define AD4858_STATUS_FUSE_CRC			BIT(5)
#define AD4858_STATUS_RESET			BIT(6)
#define AD4858_STATUS_READY			BIT(7)


static ssize_t ad4858_spi_status_read(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct axiadc_converter *conv = dev_get_drvdata(dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned int val;
	int ret, len = 0;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_SPI_STATUS, &val);
	if (ret < 0)
		return ret;
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"SPI status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tADDRESS_INVALID_ERR: %d\n",
		test_bit(AD4858_STATUS_ADDRESS_INVALID, val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tWR_TO_RD_ONLY_REG_ERR: %d\n",
		test_bit(AD4858_STATUS_WR_TO_RD_ONLY, val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCRC_ERR: %d\n",
		test_bit(AD4858_STATUS_CRC_ERR, val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tCLOCK_COUNT_ERR: %d\n",
		test_bit(AD4858_STATUS_CLK_COUNT_ERR, val));
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"\tNOT_READY_ERR: %d\n",
		test_bit(AD4858_STATUS_NOT_READY, val));

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
	unsigned int val;
	int ret, len = 0;

	ret = ad4858_spi_reg_read(adc, AD4858_REG_DEVICE_STATUS, &val);
	if (ret < 0)
		return ret;
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "Device status:\n");
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSLEEP: %d\n",
			 test_bit(AD4858_STATUS_SLEEP, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tPOWERDOWN: %d\n",
			 test_bit(AD4858_STATUS_POWERDOWN, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tCH_OR_UR: %d\n",
			 test_bit(AD4858_STATUS_CH_OR_UR, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tSPI: %d\n",
			 test_bit(AD4858_STATUS_SPI, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREGMAP_CRC: %d\n",
			 test_bit(AD4858_STATUS_REGMAP_CRC, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tFUSE_CRC: %d\n",
			 test_bit(AD4858_STATUS_FUSE_CRC, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tRESET: %d\n",
			 test_bit(AD4858_STATUS_RESET, val));
	len += scnprintf(buf + len, PAGE_SIZE - len, "\tREADY: %d\n",
			 test_bit(AD4858_STATUS_READY, val));

	buf[len - 1] = '\n';

	return len;
}

static DEVICE_ATTR(device_status, 0444, ad4858_spi_status_read, NULL);

static int ad4858_setup(struct ad4858_dev *adc)
{
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

	// ad4858_spi_reg_write(adc, AD4858_REG_PACKET,
	// 		     AD4858_PACKET_FORMAT_0);

	ad4858_spi_reg_write(adc, AD4858_REG_PACKET,
			     1);

	unsigned char tx_data[3], rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
		.cs_change = 1,
	};

	tx_data[0] = 0x0;
	tx_data[1] = 0xA;
	tx_data[2] = 0x0;

	/* Do a dummy spi transfer and change the cs low after */
	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static int ad4858_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	struct pwm_capture cnv_state;
	unsigned long long temp;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		// ret = pwm_capture(adc->cnv, &cnv_state, 0);
		// if (ret < 0)
		// 	return ret;
		// temp = DIV_ROUND_CLOSEST_ULL(1000000000ULL, cnv_state.period);
		// *val = temp;
		*val = 500000;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4858_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad4858_dev *adc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4858_set_sampling_freq(adc, val);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4858_set_oversampling_ratio(indio_dev, val);
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
		// return regmap_read(adc->regmap, reg, readval);

	return ad4858_spi_reg_write(adc, reg, writeval);
	// return regmap_write(adc->regmap, reg, writeval);
}

static const struct iio_info ad4858_info = {
	.read_raw = ad4858_read_raw,
	.write_raw = ad4858_write_raw,
	.debugfs_reg_access = &ad4858_reg_access,
};

static int ad7768_buffer_preenable(struct iio_dev *indio_dev)
{

	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct ad4858_dev *adc = conv->phy;
	unsigned char tx_data[3] = {BIT(7), 0, 0} , rx_data[3];
	struct spi_transfer xfer = {
		.rx_buf = rx_data,
		.tx_buf = tx_data,
		.len = 3,
		.bits_per_word = 8,
		.cs_change = 1,
	};

	axiadc_write(st, 0x40, 0x2);
	usleep_range(5000,6000);
	axiadc_write(st, 0x40, 0x3);

	/* Do a dummy spi transfer and change the cs low after */
	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static const struct iio_buffer_setup_ops ad4858_buffer_ops = {
	.preenable = &ad7768_buffer_preenable,
};

static int ad4858_axi_adc_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct ad4858_dev *adc = conv->phy;

	device_create_file(&adc->spi->dev, &dev_attr_spi_status);
	device_create_file(&adc->spi->dev, &dev_attr_device_status);
	indio_dev->setup_ops = &ad4858_buffer_ops;

	return 0;
}

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
	adc->spi = spi;

        // adc->refin = devm_regulator_get(&spi->dev, "refin");
        // if (!IS_ERR(adc->refin)) {
        //         ret = regulator_enable(adc->refin);
        //         if (ret) {
        //                 dev_err(&spi->dev, "Can't enable refin\n");
        //                 return ret;
        //         }
        //         ret = regulator_get_voltage(adc->refin);
	// 	if (ret < 0)
	// 		return ret;

	// 	adc->vref_mv = ret / 1000;
	// 	ret = devm_add_action_or_reset(&spi->dev,
	// 				       ad4858_regulator_disable,
	// 				       adc->refin);
	// 	if (ret)
	// 		return ret;
        // } else {
	// 	if (PTR_ERR(adc->refin) != -ENODEV)
	// 		return PTR_ERR(adc->refin);

        //         /* Use internal 4.096V reference */
	// 	adc->vref_mv = AD4858_INT_REF_VOLTAGE_MV;
        // }

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

	adc->regmap = devm_regmap_init_spi(spi, &ad4858_regmap_config);
	if (IS_ERR(adc->regmap))
		return PTR_ERR(adc->regmap);

	ret = ad4858_setup(adc);
	if (ret < 0)
		return ret;

	conv->spi = spi;
	conv->clk = adc->sampl_clk;
	conv->chip_info = &adc_chip_info;
	conv->adc_output_mode = AD4858_AXI_ADC_TWOS_COMPLEMENT;
	conv->reg_access = &ad4858_reg_access;
	conv->write_raw = &ad4858_write_raw;
	conv->read_raw = &ad4858_read_raw;
	// conv->attrs = &ad4858_group;
	conv->phy = adc;
	conv->post_setup = ad4858_axi_adc_post_setup;
	spi_set_drvdata(spi, conv);

	return 0;
}

static const struct of_device_id ad4858_of_match[] = {
        { .compatible = "adi,ad4858" },
        {}
};

static struct spi_driver ad4858_driver = {
	.probe          = ad4858_probe,
	.driver         = {
		.name   = "ad4858",
		.of_match_table = ad4858_of_match,
	},
};
module_spi_driver(ad4858_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4858 DAQ");
MODULE_LICENSE("Dual BSD/GPL");
