// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices QUAD AD7768-1/ADAQ7768-1 ADC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include "linux/util_macros.h"
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/units.h>
#include <linux/rational.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>


/* AD7768 registers definition */
#define AD7768_REG_CHIP_TYPE		0x3
#define AD7768_REG_PROD_ID_L		0x4
#define AD7768_REG_PROD_ID_H		0x5
#define AD7768_REG_CHIP_GRADE		0x6
#define AD7768_REG_SCRATCH_PAD		0x0A
#define AD7768_REG_VENDOR_L		0x0C
#define AD7768_REG_VENDOR_H		0x0D
#define AD7768_REG_INTERFACE_FORMAT	0x14
#define AD7768_REG_POWER_CLOCK		0x15
#define AD7768_REG_ANALOG		0x16
#define AD7768_REG_ANALOG2		0x17
#define AD7768_REG_CONVERSION		0x18
#define AD7768_REG_DIGITAL_FILTER	0x19
#define AD7768_REG_SINC3_DEC_RATE_MSB	0x1A
#define AD7768_REG_SINC3_DEC_RATE_LSB	0x1B
#define AD7768_REG_DUTY_CYCLE_RATIO	0x1C
#define AD7768_REG_SYNC_RESET		0x1D
#define AD7768_REG_GPIO_CONTROL		0x1E
#define AD7768_REG_GPIO_WRITE		0x1F
#define AD7768_REG_GPIO_READ		0x20
#define AD7768_REG_OFFSET_HI		0x21
#define AD7768_REG_OFFSET_MID		0x22
#define AD7768_REG_OFFSET_LO		0x23
#define AD7768_REG_GAIN_HI		0x24
#define AD7768_REG_GAIN_MID		0x25
#define AD7768_REG_GAIN_LO		0x26
#define AD7768_REG_SPI_DIAG_ENABLE	0x28
#define AD7768_REG_ADC_DIAG_ENABLE	0x29
#define AD7768_REG_DIG_DIAG_ENABLE	0x2A
#define AD7768_REG_ADC_DATA		0x2C
#define AD7768_REG_MASTER_STATUS	0x2D
#define AD7768_REG_SPI_DIAG_STATUS	0x2E
#define AD7768_REG_ADC_DIAG_STATUS	0x2F
#define AD7768_REG_DIG_DIAG_STATUS	0x30
#define AD7768_REG_MCLK_COUNTER		0x31

/* AD7768_REG_POWER_CLOCK */
#define AD7768_PWR_MCLK_DIV_MSK		GENMASK(5, 4)
#define AD7768_PWR_MCLK_DIV(x)		FIELD_PREP(AD7768_PWR_MCLK_DIV_MSK, x)
#define AD7768_PWR_PWRMODE_MSK		GENMASK(1, 0)
#define AD7768_PWR_PWRMODE(x)		FIELD_PREP(AD7768_PWR_PWRMODE_MSK, x)

/* AD7768_REG_DIGITAL_FILTER */
#define AD7768_DIG_FIL_FIL_MSK		GENMASK(6, 4)
#define AD7768_DIG_FIL_FIL(x)		FIELD_PREP(AD7768_DIG_FIL_FIL_MSK, x)
#define AD7768_DIG_FIL_DEC_MSK		GENMASK(2, 0)
#define AD7768_DIG_FIL_DEC_RATE(x)	FIELD_PREP(AD7768_DIG_FIL_DEC_MSK, x)

/* AD7768_SINC3_DEC_RATE */
#define AD7768_SINC3_DEC_RATE_MSB_MSK	GENMASK(12, 8)
#define AD7768_SINC3_DEC_RATE_LSB_MSK	GENMASK(7, 0)

/* AD7768_REG_CONVERSION */
#define AD7768_CONV_MODE_MSK		GENMASK(2, 0)
#define AD7768_CONV_MODE(x)		FIELD_PREP(AD7768_CONV_MODE_MSK, x)

/* AD7768_REG_GPIO_CONTROL */
#define AD7768_GPIO_CONTROL_MSK		GENMASK(3, 0)
#define AD7768_GPIO_UNIVERSAL_EN	BIT(7)
#define AD7768_GPIO_PGIA_EN		(AD7768_GPIO_UNIVERSAL_EN | GENMASK(2, 0))

/* AD7768_REG_GPIO_WRITE */
#define AD7768_GPIO_WRITE_MSK		GENMASK(3, 0)
#define AD7768_GPIO_WRITE(x)		FIELD_PREP(AD7768_GPIO_WRITE_MSK, x)

/* AD7768_REG_GPIO_READ */
#define AD7768_GPIO_READ_MSK		GENMASK(3, 0)
#define AD7768_GPIO_READ(x)		FIELD_PREP(AD7768_GPIO_READ_MSK, x)

/* AD7768_REG_CONVLEN */
#define AD7768_REG_CONVLEN_MSK		GENMASK(3, 3)
#define AD7768_REG_CONVLEN(x)		FIELD_PREP(AD7768_REG_CONVLEN_MSK, x)

#define AD7768_GPIO_INPUT(x)		0x00
#define AD7768_GPIO_OUTPUT(x)		BIT(x)

#define AD7768_RD_FLAG_MSK(x)		(BIT(6) | ((x) & 0x3F))
#define AD7768_WR_FLAG_MSK(x)		((x) & 0x3F)

#define AD7768_CHAN_INFO_NONE		0

#define ADAQ776X_GAIN_MAX_NANO		(128 * NANO)
#define ADAQ776X_MAX_GAIN_MODES		8

enum ad7768_conv_mode {
	AD7768_CONTINUOUS,
	AD7768_ONE_SHOT,
	AD7768_SINGLE,
	AD7768_PERIODIC,
	AD7768_STANDBY
};

enum ad7768_pwrmode {
	AD7768_ECO_MODE = 0,
	AD7768_MED_MODE = 2,
	AD7768_FAST_MODE = 3
};

enum ad7768_mclk_div {
	AD7768_MCLK_DIV_16,
	AD7768_MCLK_DIV_8,
	AD7768_MCLK_DIV_4,
	AD7768_MCLK_DIV_2
};

enum ad7768_flt_mode {
	SINC5,
	SINC5_DEC_X8,
	SINC5_DEC_X16,
	SINC3,
	WIDEBAND
};

enum ad7768_dec_rate {
	AD7768_DEC_RATE_32 = 0,
	AD7768_DEC_RATE_64 = 1,
	AD7768_DEC_RATE_128 = 2,
	AD7768_DEC_RATE_256 = 3,
	AD7768_DEC_RATE_512 = 4,
	AD7768_DEC_RATE_1024 = 5
};

enum ad7768_scan_type {
	AD7768_SCAN_TYPE_NORMAL,
	AD7768_SCAN_TYPE_HIGH_SPEED,
};

struct ad7768_clk_configuration {
	enum ad7768_mclk_div mclk_div;
	enum ad7768_dec_rate dec_rate;
	unsigned int clk_div;
};

enum {
	AD7768_PGA_GAIN_0,
	AD7768_PGA_GAIN_1,
	AD7768_PGA_GAIN_2,
	AD7768_PGA_GAIN_3,
	AD7768_PGA_GAIN_4,
	AD7768_PGA_GAIN_5,
	AD7768_PGA_GAIN_6,
	AD7768_PGA_GAIN_7,
	AD7768_MAX_PGA_GAIN,
};

enum {
	AD7768_AAF_IN1,
	AD7768_AAF_IN2,
	AD7768_AAF_IN3,
};

/*
 * Gains computed as fractions of 1000 so they can be expressed by integers.
 */
static const int adaq7768_gains[7] = {
	[AD7768_PGA_GAIN_0] = 325,
	[AD7768_PGA_GAIN_1] = 650,
	[AD7768_PGA_GAIN_2] = 1300,
	[AD7768_PGA_GAIN_3] = 2600,
	[AD7768_PGA_GAIN_4] = 5200,
	[AD7768_PGA_GAIN_5] = 10400,
	[AD7768_PGA_GAIN_6] = 20800
};

static const int adaq7769_gains[8] = {
	[AD7768_PGA_GAIN_0] = 1000,
	[AD7768_PGA_GAIN_1] = 2000,
	[AD7768_PGA_GAIN_2] = 4000,
	[AD7768_PGA_GAIN_3] = 8000,
	[AD7768_PGA_GAIN_4] = 16000,
	[AD7768_PGA_GAIN_5] = 32000,
	[AD7768_PGA_GAIN_6] = 64000,
	[AD7768_PGA_GAIN_7] = 128000
};

static const int ad7768_aaf_gains[3] = {
	[AD7768_AAF_IN1] = 1000,
	[AD7768_AAF_IN2] = 364,
	[AD7768_AAF_IN3] = 143
};

static const char * const ad7768_vcm_modes[] = {
	"(AVDD1-AVSS)/2",
	"2V5",
	"2V05",
	"1V9",
	"1V65",
	"1V1",
	"0V9",
	"OFF",
};

struct ad7768_clk_div_range {
	unsigned int clk_div_min;
	unsigned int clk_div_max;
};

static const struct ad7768_clk_div_range ad7768_clk_div_ranges[] = {
	[SINC5] = {.clk_div_min = 64, .clk_div_max = 16384},
	[SINC5_DEC_X8] = {.clk_div_min = 16, .clk_div_max = 128},
	[SINC5_DEC_X16] = {.clk_div_min = 32, .clk_div_max = 256},
	[SINC3] = {.clk_div_min = 64, .clk_div_max = 327680},
	[WIDEBAND] = {.clk_div_min = 64, .clk_div_max = 16384},
};

static const struct ad7768_clk_configuration ad7768_sinc5_wideband_clk_conf[] = {
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_32, 64},
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_64, 128 },
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_128, 256 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_128, 512 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_256, 1024 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_512, 2048 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_1024, 4096 },
	{ AD7768_MCLK_DIV_8, AD7768_DEC_RATE_1024, 8192 },
	{ AD7768_MCLK_DIV_16, AD7768_DEC_RATE_1024, 16384 },
};

static int ad7768_freq_available_range[5][3];

static const int ad7768_mclk_div_rates[4] = {
	16, 8, 4, 2
};

static const int dec_rate_values[6] = {
	32, 64, 128, 256, 512, 1024,
};

static const  int sinc3_dec_rate_max_values[4] = {
	20480, 40960, 81920, 163840,
};

static const struct iio_scan_type ad7768_scan_type[] = {
	[AD7768_SCAN_TYPE_NORMAL] = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
	},
	[AD7768_SCAN_TYPE_HIGH_SPEED] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
	},
};

static const char * const ad7768_filter_enum[] = {
	[SINC5] = "sinc5",
	[SINC5_DEC_X8] = "sinc5-dec8",
	[SINC5_DEC_X16] = "sinc5-dec16",
	[SINC3] = "sinc3",
	[WIDEBAND] = "wideband",
};

static const char * const ad7768_pwr_mode_enum[] = {
	[AD7768_ECO_MODE]   = "eco_mode",
	[AD7768_MED_MODE]   = "med_mode",
	[AD7768_FAST_MODE]  = "fast_mode",
};

static int ad7768_get_vcm(struct iio_dev *dev, const struct iio_chan_spec *chan);
static int ad7768_set_vcm(struct iio_dev *dev, const struct iio_chan_spec *chan,
			  unsigned int mode);
static int ad7768_get_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan);
static int ad7768_set_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan, unsigned int filter);
static int ad7768_get_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan);
static int ad7768_set_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan, unsigned int pwr_mode);

static const struct iio_enum ad7768_pwr_mode_iio_enum = {
	.items = ad7768_pwr_mode_enum,
	.num_items = ARRAY_SIZE(ad7768_pwr_mode_enum),
	.set = ad7768_set_pwr_mode,
	.get = ad7768_get_pwr_mode,
};

static const struct iio_enum ad7768_flt_type_iio_enum = {
	.items = ad7768_filter_enum,
	.num_items = ARRAY_SIZE(ad7768_filter_enum),
	.set = ad7768_set_dig_fil_attr,
	.get = ad7768_get_dig_fil_attr,
};

static const struct iio_enum ad7768_vcm_mode_enum = {
	.items = ad7768_vcm_modes,
	.num_items = ARRAY_SIZE(ad7768_vcm_modes),
	.set = ad7768_set_vcm,
	.get = ad7768_get_vcm,
};

//static struct iio_chan_spec_ext_info ad7768_ext_info[] = {
//	IIO_ENUM("common_mode_voltage",
//		 IIO_SHARED_BY_ALL,
//		 &ad7768_vcm_mode_enum),
//	IIO_ENUM_AVAILABLE("common_mode_voltage",
//			   IIO_SHARED_BY_ALL,
//			   &ad7768_vcm_mode_enum),
//	IIO_ENUM("power_mode", IIO_SHARED_BY_ALL, &ad7768_pwr_mode_iio_enum),
//	IIO_ENUM_AVAILABLE("power_mode", IIO_SHARED_BY_ALL, &ad7768_pwr_mode_iio_enum),
//	IIO_ENUM("filter_mode", IIO_SHARED_BY_ALL, &ad7768_flt_type_iio_enum),
//	IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_ALL, &ad7768_flt_type_iio_enum),
//	{ },
//};

#define AD7768_CHAN(_idx, _msk_avail) {	\
		.type = IIO_VOLTAGE,\
		.info_mask_separate_available = _msk_avail,\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,\
		.channel = _idx,\
		.scan_index = 0,\
		.has_ext_scan_type = 1,\
		.ext_scan_type = ad7768_scan_type,\
		.num_ext_scan_type = ARRAY_SIZE(ad7768_scan_type),\
}

static const struct iio_chan_spec ad7768_channels[] = {
	AD7768_CHAN(0, AD7768_CHAN_INFO_NONE),
};

static const struct iio_chan_spec adaq776x_channels[] = {
	AD7768_CHAN(0, BIT(IIO_CHAN_INFO_SCALE)),
};

struct ad7768_chip_info {
	const char *name;
	bool has_variable_aaf;
	bool has_pga;
	int num_pga_modes;
	int default_pga_mode;
	int pgia_mode2pin_offset;
	const int *pga_gains;
	const struct iio_chan_spec *channel_spec;
	const unsigned long *available_masks;
	int num_channels;
};




static const unsigned long ad7768_channel_masks[] = {
	BIT(0),
	0,
};

static const struct ad7768_chip_info ad7768_chip_info = {
	.name = "ad7768-1",
	.channel_spec = ad7768_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
};

static const struct ad7768_chip_info adaq7767_chip_info = {
	.name = "adaq7767-1",
	.channel_spec = ad7768_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.has_pga = false,
	.has_variable_aaf = true
};

static const struct ad7768_chip_info adaq7768_chip_info = {
	.name = "adaq7768-1",
	.channel_spec = adaq776x_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.pga_gains = adaq7768_gains,
	.default_pga_mode = AD7768_PGA_GAIN_2,
	.num_pga_modes = ARRAY_SIZE(adaq7768_gains),
	.pgia_mode2pin_offset = 6,
	.has_pga = true,
	.has_variable_aaf = false
};

static const struct ad7768_chip_info adaq7769_chip_info = {
	.name = "adaq7769-1",
	.channel_spec = adaq776x_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.pga_gains = adaq7769_gains,
	.default_pga_mode = AD7768_PGA_GAIN_0,
	.num_pga_modes = ARRAY_SIZE(adaq7769_gains),
	.pgia_mode2pin_offset = 0,
	.has_pga = true,
	.has_variable_aaf = true
};






//static int quad_ad7768_probe(struct platform_device *pdev)
static int quad_ad7768_probe(struct spi_device *spi)
{
	//struct device *dev = &pdev->dev;
	struct device *dev = &spi->dev;
	//struct device_node *np = dev->of_node;
	//struct component_match *match = NULL;
	int charger_status;
	int i, irq;
	int ret;


	dev_info(dev, "PROBED!!!!!!!!!!!!!!!\n");
	return 0;
}

static const struct spi_device_id quad_ad7768_id_table[] = {
	{ "quad-ad7768-1", (kernel_ulong_t)&ad7768_chip_info },
	{ "quad-adaq7767-1", (kernel_ulong_t)&adaq7767_chip_info },
	{ "quad-adaq7768-1", (kernel_ulong_t)&adaq7768_chip_info },
	{ "quad-adaq7769-1", (kernel_ulong_t)&adaq7769_chip_info },
	{}
};
MODULE_DEVICE_TABLE(spi, quad_ad7768_id_table);
static const struct of_device_id quad_ad7768_of_match[] = {
	{ .compatible = "adi,quad-ad7768-1", .data = &ad7768_chip_info },
	{ .compatible = "adi,quad-adaq7767-1", .data = &adaq7767_chip_info },
	{ .compatible = "adi,quad-adaq7768-1", .data = &adaq7768_chip_info },
	{ .compatible = "adi,quad-adaq7769-1", .data = &adaq7769_chip_info },
	{ },
};
MODULE_DEVICE_TABLE(of, quad_ad7768_of_match);

static struct spi_driver quad_ad7768_driver = {
	.driver = {
		.name = "quad-ad7768-1",
		.of_match_table = quad_ad7768_of_match,
	},
	.probe = quad_ad7768_probe,
	.id_table = quad_ad7768_id_table,
};
module_spi_driver(quad_ad7768_driver);


MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices Quad AD7768-1 ADC driver");
MODULE_LICENSE("GPL");
