// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices, Inc.
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define AD74416H_CRC_POLYNOMIAL	0x7
DECLARE_CRC8_TABLE(ad74416h_crc8_table);

#define AD74416H_REG_CH_FUNC_SETUP_X(x)		(0x01 + ((x) * 12))
#define AD74416H_CH_FUNC_SETUP_MASK		GENMASK(3, 0)

#define AD74416H_REG_ADC_CONFIG_X(x)		(0x02 + ((x) * 12))
#define AD74416H_ADC_CONFIG_RATE_MASK		GENMASK(10, 8)
#define AD74416H_ADC_CONFIG_RANGE_MASK		GENMASK(6, 4)
#define AD74416H_ADC_CONFIG_MUX_MASK		GENMASK(2, 0)
#define AD74416H_ADC_RATE_10SPS_50_60_HART	0b000
#define AD74416H_ADC_RATE_20SPS_50_60		0b001
#define AD74416H_ADC_RATE_1K2SPS		0b010
#define AD74416H_ADC_RATE_4K8SPS		0b011
#define AD74416H_ADC_RATE_9K6SPS		0b100
#define AD74416H_ADC_RATE_19K2SPS		0b101
#define AD74416H_ADC_RATE_200SPS_HART		0b110
#define AD74416H_ADC_RANGE_0_12_V		0b000
#define AD74416H_ADC_RANGE_NEG12_12_V		0b001
#define AD74416H_ADC_RANGE_NEG0P3125_0P3125V	0b010
#define AD74416H_ADC_RANGE_NEG0P3125_0V		0b011
#define AD74416H_ADC_RANGE_0_0P3125V		0b100
#define AD74416H_ADC_RANGE_0_0P625V		0b101
#define AD74416H_ADC_RANGE_NEG104_104MV		0b110
#define AD74416H_ADC_RANGE_NEG2P5_2P5V		0b111
#define	AD74416H_MUX_LF_TO_AGND			0b000
#define	AD74416H_MUX_HF_TO_LF			0b001
#define	AD74416H_MUX_VSENSEN_TO_AGND		0b010
#define	AD74416H_MUX_LF_TO_VSENSEN		0b011
#define	AD74416H_MUX_AGND_TO_AGND		0b100

#define AD74416H_REG_OUTPUT_CONFIG_X(x)		(0x05 + ((x) * 12))
#define AD74416H_VOUT_RANGE_MSK			BIT(7)
#define AD74416H_VOUT_RANGE_0_12V		0b000
#define AD74416H_VOUT_RANGE_NEG12_12V		0b001

#define AD74416H_REG_DAC_CODE_X(x)		(0x0A + ((x) * 12))
#define AD74416H_DAC_CODE_MAX			GENMASK(15, 0)
#define AD74416H_DAC_VOLTAGE_MAX		12000

#define AD74416H_REG_ADC_CONV_CTRL		0x39
#define AD74416H_CONV_SEQ_MASK			GENMASK(9, 8)
#define AD74416H_CONV_SEQ_ON			0b000
#define AD74416H_CONV_SEQ_SINGLE		0b001
#define AD74416H_CONV_SEQ_CONTINUOUS		0b010
#define AD74416H_CONV_SEQ_OFF			0b011
#define AD74416H_CH_EN_MASK(x)			BIT(x)

#define AD74416H_REG_ADC_RESULT_UPR_X(x)	(0x41 + ((x) * 2))
#define AD74416H_REG_ADC_RESULT_X(x)		(0x42 + ((x) * 2))
#define AD74416H_ADC_RESULT_MAX			GENMASK(23, 0)
#define AD74416H_CONV_RES_UPR_MSK		GENMASK(7, 0)
#define AD74416H_CONV_RESULT_MSK		GENMASK(15, 0)

#define AD74416H_REG_GPIO_CONFIG_X(x)		(0x32 + (x))
#define AD74416H_GPIO_CONFIG_DATA_MASK		BIT(4)
#define AD74416H_GPIO_CONFIG_SELECT_MASK	GENMASK(2, 0)
#define AD74416H_GPIO_CONFIG_HIGH_Z		0b000
#define	AD74416H_GPIO_CONFIG_DATA		0b001
#define	AD74416H_GPIO_CONFIG_IN			0b010
#define	AD74416H_GPIO_CONFIG_COMP		0b011
#define	AD74416H_GPIO_CONFIG_DO			0b100

#define AD74416H_REG_DIN_CONFIG0_X(x)		(0x03 + ((x) * 12))
#define AD74416H_DIN_DEBOUNCE_MASK		GENMASK(4, 0)
#define AD74416H_DIN_DEBOUNCE_LEN		BIT(5)
#define AD74416H_DIN_SINK_MASK			GENMASK(11, 7)

#define AD74416H_REG_DIN_COMP_OUT		0x3E

#define CH_FUNC_HIGH_IMPEDANCE			0x0
#define CH_FUNC_VOLTAGE_OUTPUT			0x1
#define CH_FUNC_CURRENT_OUTPUT			0x2
#define CH_FUNC_VOLTAGE_INPUT			0x3
#define CH_FUNC_CURRENT_INPUT_EXT_POWER		0x4
#define CH_FUNC_CURRENT_INPUT_LOOP_POWER	0x5
#define CH_FUNC_RESISTANCE_INPUT		0x6
#define CH_FUNC_DIGITAL_INPUT_LOGIC		0x7
#define CH_FUNC_DIGITAL_INPUT_LOOP_POWER	0x8
#define CH_FUNC_CURRENT_INPUT_EXT_POWER_HART	0x9
#define CH_FUNC_CURRENT_INPUT_LOOP_POWER_HART	0xA

#define CH_FUNC_MIN	CH_FUNC_HIGH_IMPEDANCE
#define CH_FUNC_MAX	CH_FUNC_CURRENT_INPUT_LOOP_POWER_HART

#define AD74416H_REG_CMD_KEY			0x74
#define AD74416H_CMD_KEY_RESET1			0x15FA
#define AD74416H_CMD_KEY_RESET2			0xAF51

#define AD74416H_CHANNEL_MAX			4
#define AD74416H_DIN_DEBOUNCE_LEN		BIT(5)
#define AD77416H_DEV_ADDRESS_MSK		GENMASK(5, 4)
#define AD74416H_FRAME_SIZE			5
#define AD74416H_REG_READ_SELECT		0x6E

static const unsigned int ad74416h_debounce_map[AD74416H_DIN_DEBOUNCE_LEN] = {
	0,     13,    18,    24,    32,    42,    56,    75,
	100,   130,   180,   240,   320,   420,   560,   750,
	1000,  1300,  1800,  2400,  3200,  4200,  5600,  7500,
	10000, 13000, 18000, 24000, 32000, 42000, 56000, 75000,
};

static const int ad74416h_adc_sampling_rates[] = {
	10, 20, 1200, 4800, 9600, 19200, 200
};

static int ad74416h_crc(u8 *buf)
{
	return crc8(ad74416h_crc8_table, buf, 4, 0);
}

struct ad74416h_channel_config {
	u32		func;
	u32		drive_strength;
	bool		gpio_comparator;
	bool		initialized;
};

struct ad74416h_channels {
	struct iio_chan_spec	*channels;
	unsigned int		num_channels;
};

struct ad74416h_state {
	u32				dev_addr;
	struct ad74416h_channel_config	channel_configs[AD74416H_CHANNEL_MAX];
	unsigned int			gpo_gpio_offsets[AD74416H_CHANNEL_MAX];
	unsigned int			comp_gpio_offsets[AD74416H_CHANNEL_MAX];
	struct gpio_chip		gpo_gpiochip;
	struct gpio_chip		comp_gpiochip;
	struct completion		adc_data_completion;
	unsigned int			num_gpios;
	unsigned int			num_comparator_gpios;
	u32				sense_resistor_ohms;

	/*
	 * Synchronize consecutive operations when doing a one-shot
	 * conversion and when updating the ADC samples SPI message.
	 */
	struct mutex			lock;

	const struct ad74416h_chip_info	*chip_info;
	struct spi_device		*spi;
	struct regulator		*refin_reg;
	struct regmap			*regmap;
	struct device			*dev;
	struct iio_trigger		*trig;
	struct gpio_desc		*reset_gpio;

	size_t			adc_active_channels;
	struct spi_message	adc_samples_msg;
	struct spi_transfer	adc_samples_xfer[AD74416H_CHANNEL_MAX + 1];

	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */
	struct {
		u8 rx_buf[AD74416H_FRAME_SIZE * AD74416H_CHANNEL_MAX];
		s64 timestamp;
	} adc_samples_buf __aligned(IIO_DMA_MINALIGN);

	u8	adc_samples_tx_buf[AD74416H_FRAME_SIZE * AD74416H_CHANNEL_MAX];
	u8	reg_tx_buf[AD74416H_FRAME_SIZE];
	u8	reg_rx_buf[AD74416H_FRAME_SIZE];
};

#define AD74416H_DAC_CHANNEL(_type, extra_mask_separate)		\
	{								\
		.type = (_type),					\
		.indexed = 1,						\
		.output = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)		\
				      | (extra_mask_separate),		\
	}

#define AD74416H_ADC_CHANNEL(_type, extra_mask_separate)		\
	{								\
		.type = (_type),					\
		.indexed = 1,						\
		.output = 0,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)		\
				      | BIT(IIO_CHAN_INFO_SAMP_FREQ)	\
				      | (extra_mask_separate),		\
		.info_mask_separate_available =				\
					BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = 16,					\
			.storagebits = 32,				\
			.shift = 8,					\
			.endianness = IIO_BE,				\
		},							\
	}

#define AD74416H_ADC_VOLTAGE_CHANNEL					\
	AD74416H_ADC_CHANNEL(IIO_VOLTAGE, BIT(IIO_CHAN_INFO_SCALE)	\
			     | BIT(IIO_CHAN_INFO_OFFSET))

#define AD74416H_ADC_CURRENT_CHANNEL					\
	AD74416H_ADC_CHANNEL(IIO_CURRENT,  BIT(IIO_CHAN_INFO_SCALE)	\
			     | BIT(IIO_CHAN_INFO_OFFSET))

static struct iio_chan_spec ad74416h_voltage_output_channels[] = {
	AD74416H_DAC_CHANNEL(IIO_VOLTAGE, BIT(IIO_CHAN_INFO_SCALE)),
	AD74416H_ADC_CURRENT_CHANNEL,
};

static struct iio_chan_spec ad74416h_current_output_channels[] = {
	AD74416H_DAC_CHANNEL(IIO_CURRENT, BIT(IIO_CHAN_INFO_SCALE)),
	AD74416H_ADC_VOLTAGE_CHANNEL,
};

static struct iio_chan_spec ad74416h_voltage_input_channels[] = {
	AD74416H_ADC_VOLTAGE_CHANNEL,
};

static struct iio_chan_spec ad74416h_current_input_channels[] = {
	AD74416H_ADC_CURRENT_CHANNEL,
};

static struct iio_chan_spec ad74416h_current_input_loop_channels[] = {
	AD74416H_DAC_CHANNEL(IIO_CURRENT, BIT(IIO_CHAN_INFO_SCALE)),
	AD74416H_ADC_CURRENT_CHANNEL,
};

static struct iio_chan_spec ad74416h_resistance_input_channels[] = {
	AD74416H_ADC_CHANNEL(IIO_RESISTANCE, BIT(IIO_CHAN_INFO_PROCESSED)),
};

static struct iio_chan_spec ad74416h_digital_input_channels[] = {
	AD74416H_ADC_VOLTAGE_CHANNEL,
};

#define _AD74416H_CHANNELS(_channels)			\
	{						\
		.channels = _channels,			\
		.num_channels = ARRAY_SIZE(_channels),	\
	}

#define AD74416H_CHANNELS(name) \
	_AD74416H_CHANNELS(ad74416h_ ## name ## _channels)

static const struct ad74416h_channels ad74416h_channels_map[] = {
	[CH_FUNC_HIGH_IMPEDANCE] = AD74416H_CHANNELS(voltage_input),
	[CH_FUNC_VOLTAGE_OUTPUT] = AD74416H_CHANNELS(voltage_output),
	[CH_FUNC_CURRENT_OUTPUT] = AD74416H_CHANNELS(current_output),
	[CH_FUNC_VOLTAGE_INPUT] = AD74416H_CHANNELS(voltage_input),
	[CH_FUNC_CURRENT_INPUT_EXT_POWER] = AD74416H_CHANNELS(current_input),
	[CH_FUNC_CURRENT_INPUT_LOOP_POWER] = AD74416H_CHANNELS(current_input_loop),
	[CH_FUNC_RESISTANCE_INPUT] = AD74416H_CHANNELS(resistance_input),
	[CH_FUNC_DIGITAL_INPUT_LOGIC] = AD74416H_CHANNELS(digital_input),
	[CH_FUNC_DIGITAL_INPUT_LOOP_POWER] = AD74416H_CHANNELS(digital_input),
	[CH_FUNC_CURRENT_INPUT_EXT_POWER_HART] = AD74416H_CHANNELS(current_input),
	[CH_FUNC_CURRENT_INPUT_LOOP_POWER_HART] = AD74416H_CHANNELS(current_input),
};

static void ad74416h_format_reg_write(u8 addr, u8 reg, u16 val, u8 *buf)
{
	buf[0] = FIELD_PREP(AD77416H_DEV_ADDRESS_MSK, addr);
	buf[1] = reg;
	put_unaligned_be16(val, &buf[2]);
	buf[3] = ad74416h_crc(buf);
}

static int ad74416h_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad74416h_state *st = context;

	ad74416h_format_reg_write(st->dev_addr, reg, val, st->reg_tx_buf);

	return spi_write(st->spi, st->reg_tx_buf, AD74416H_FRAME_SIZE);
}

static int ad74416h_crc_check(struct ad74416h_state *st, u8 *buf)
{
	u8 expected_crc = ad74416h_crc(buf);

	if (buf[4] != expected_crc) {
		dev_err(st->dev, "Bad CRC %02x for %02x%02x%02x%02x\n",
			buf[4], buf[0], buf[1], buf[2], buf[3]);
		return -EINVAL;
	}

	return 0;
}

static int ad74416h_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad74416h_state *st = context;
	struct spi_transfer reg_read_xfer[] = {
		{
			.tx_buf = st->reg_tx_buf,
			.len = AD74416H_FRAME_SIZE,
			.cs_change = 1,
		},
		{
			.rx_buf = st->reg_rx_buf,
			.len = AD74416H_FRAME_SIZE,
		},
	};
	int ret;

	ad74416h_format_reg_write(st->dev_addr, AD74416H_REG_READ_SELECT, reg,
				  st->reg_tx_buf);

	ret = spi_sync_transfer(st->spi, reg_read_xfer,
				ARRAY_SIZE(reg_read_xfer));
	if (ret)
		return ret;

	ret = ad74416h_crc_check(st, st->reg_rx_buf);
	if (ret)
		return ret;

	*val = get_unaligned_be16(&st->reg_rx_buf[1]);

	return 0;
}

static int ad74416h_rate_to_reg_val(struct ad74416h_state *st,
				    int rate, unsigned int *val)
{
	switch (rate) {
	case 20:
		*val = AD74416H_ADC_RATE_20SPS_50_60;
		return 0;
	case 1200:
		*val = AD74416H_ADC_RATE_1K2SPS;
		return 0;
	case 4800:
		*val = AD74416H_ADC_RATE_4K8SPS;
		return 0;
	case 9600:
		*val = AD74416H_ADC_RATE_9K6SPS;
		return 0;
	case 19200:
		*val = AD74416H_ADC_RATE_19K2SPS;
		return 0;
	default:
		dev_err(st->dev, "ADC rate invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_reg_val_to_rate(struct ad74416h_state *st,
				    int reg_val, unsigned int *val)
{
	switch (reg_val) {
	case AD74416H_ADC_RATE_20SPS_50_60:
		*val = 20;
		return 0;
	case AD74416H_ADC_RATE_1K2SPS:
		*val = 1200;
		return 0;
	case AD74416H_ADC_RATE_4K8SPS:
		*val = 4800;
		return 0;
	case AD74416H_ADC_RATE_9K6SPS:
		*val = 9600;
		return 0;
	case AD74416H_ADC_RATE_19K2SPS:
		*val = 19200;
		return 0;
	default:
		dev_err(st->dev, "ADC rate reg val invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_set_adc_rate(struct ad74416h_state *st,
				 unsigned int channel, int val)
{
	unsigned int reg_val;
	int ret;

	ret = ad74416h_rate_to_reg_val(st, val, &reg_val);
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap,
				  AD74416H_REG_ADC_CONFIG_X(channel),
				  AD74416H_ADC_CONFIG_RATE_MASK,
				  FIELD_PREP(AD74416H_ADC_CONFIG_RATE_MASK,
					     reg_val));
}

static int ad74416h_get_adc_rate(struct ad74416h_state *st,
				 unsigned int channel, int *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_ADC_CONFIG_X(channel), &reg_val);
	if (ret)
		return ret;

	reg_val = FIELD_GET(AD74416H_ADC_CONFIG_RATE_MASK, reg_val);

	return ad74416h_reg_val_to_rate(st, reg_val, val);
}

static int ad74416h_set_channel_dac_code(struct ad74416h_state *st,
					 unsigned int channel, int dac_code)
{
	return regmap_write(st->regmap, AD74416H_REG_DAC_CODE_X(channel),
			    dac_code);
}

static int ad74416h_get_dac_range(struct ad74416h_state *st,
				  unsigned int channel,
				  unsigned int *val)
{
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_OUTPUT_CONFIG_X(channel), val);
	if (ret)
		return ret;

	*val = FIELD_GET(AD74416H_VOUT_RANGE_MSK, *val);

	return 0;
}

static int ad74416h_dac_range_to_voltage_range(struct ad74416h_state *st,
					       unsigned int range, int *val)
{
	switch (range) {
	case AD74416H_VOUT_RANGE_0_12V:
		*val = AD74416H_DAC_VOLTAGE_MAX;
		return 0;
	case AD74416H_VOUT_RANGE_NEG12_12V:
		*val = AD74416H_DAC_VOLTAGE_MAX * 2;
		return 0;
	default:
		dev_err(st->dev, "DAC range invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_get_output_voltage_scale(struct ad74416h_state *st,
					     unsigned int channel,
					     int *val, int *val2)
{
	unsigned int range;
	int ret;

	ret = ad74416h_get_dac_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74416h_dac_range_to_voltage_range(st, range, val);
	if (ret)
		return ret;

	*val2 = AD74416H_DAC_CODE_MAX;

	return IIO_VAL_FRACTIONAL;
}

static int ad74416h_get_adc_range(struct ad74416h_state *st,
				  unsigned int channel,
				  unsigned int *val)
{
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_ADC_CONFIG_X(channel), val);
	if (ret)
		return ret;

	*val = FIELD_GET(AD74416H_ADC_CONFIG_RANGE_MASK, *val);

	return 0;
}

static int ad74416h_adc_range_to_voltage_range(struct ad74416h_state *st,
					       unsigned int range, int *val)
{
	switch (range) {
	case AD74416H_ADC_RANGE_0_12_V:
		*val = 12000;
		return 0;
	case AD74416H_ADC_RANGE_NEG12_12_V:
		*val = 24000;
		return 0;
	case AD74416H_ADC_RANGE_NEG0P3125_0P3125V:
	case AD74416H_ADC_RANGE_0_0P625V:
		*val = 625;
		return 0;
	case AD74416H_ADC_RANGE_NEG0P3125_0V:
	case AD74416H_ADC_RANGE_0_0P3125V:
		*val = 312;
		return 0;
	case AD74416H_ADC_RANGE_NEG104_104MV:
		*val = 208;
		return 0;
	case AD74416H_ADC_RANGE_NEG2P5_2P5V:
		*val = 5000;
		return 0;
	default:
		dev_err(st->dev, "ADC range invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_get_input_voltage_scale(struct ad74416h_state *st,
					    unsigned int channel,
					    int *val, int *val2)
{
	unsigned int range;
	int ret;

	ret = ad74416h_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74416h_adc_range_to_voltage_range(st, range, val);
	if (ret)
		return ret;

	*val2 = AD74416H_ADC_RESULT_MAX;

	return IIO_VAL_FRACTIONAL;
}

static int ad74416h_get_output_current_scale(struct ad74416h_state *st,
					     int *val, int *val2)
{
	*val = regulator_get_voltage(st->refin_reg);
	*val2 = st->sense_resistor_ohms * AD74416H_DAC_CODE_MAX * 1000;

	return IIO_VAL_FRACTIONAL;
}

static int ad74416h_get_input_current_scale(struct ad74416h_state *st,
					    unsigned int channel, int *val,
					    int *val2)
{
	unsigned int range;
	int ret;

	ret = ad74416h_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74416h_adc_range_to_voltage_range(st, range, val);
	if (ret)
		return ret;

	*val2 = AD74416H_ADC_RESULT_MAX * st->sense_resistor_ohms;

	return IIO_VAL_FRACTIONAL;
}

static int ad74416h_adc_range_to_voltage_offset_raw(struct ad74416h_state *st,
						    unsigned int range, int *val)
{
	switch (range) {
	case AD74416H_ADC_RANGE_0_12_V:
	case AD74416H_ADC_RANGE_0_0P625V:
	case AD74416H_ADC_RANGE_0_0P3125V:
		*val = 0;
		return 0;
	case AD74416H_ADC_RANGE_NEG12_12_V:
	case AD74416H_ADC_RANGE_NEG0P3125_0P3125V:
	case AD74416H_ADC_RANGE_NEG0P3125_0V:
	case AD74416H_ADC_RANGE_NEG104_104MV:
	case AD74416H_ADC_RANGE_NEG2P5_2P5V:
		*val = -((int)AD74416H_ADC_RESULT_MAX / 2);
		return 0;
	default:
		dev_err(st->dev, "ADC range invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_range_to_voltage_offset(struct ad74416h_state *st,
					    unsigned int range, int *val)
{
	switch (range) {
	case AD74416H_ADC_RANGE_0_12_V:
	case AD74416H_ADC_RANGE_0_0P625V:
	case AD74416H_ADC_RANGE_0_0P3125V:
		*val = 0;
		return 0;
	case AD74416H_ADC_RANGE_NEG12_12_V:
		*val = -6000;
		return 0;
	case AD74416H_ADC_RANGE_NEG0P3125_0P3125V:
		*val = -156;
		return 0;
	case AD74416H_ADC_RANGE_NEG0P3125_0V:
		*val = -234;
		return 0;
	case AD74416H_ADC_RANGE_NEG104_104MV:
		*val = -52;
		return 0;
	case AD74416H_ADC_RANGE_NEG2P5_2P5V:
		*val = -1250;
		return 0;
	default:
		dev_err(st->dev, "ADC range invalid\n");
		return -EINVAL;
	}
}

static int ad74416h_get_input_voltage_offset(struct ad74416h_state *st,
					     unsigned int channel, int *val)
{
	unsigned int range;
	int ret;

	ret = ad74416h_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74416h_adc_range_to_voltage_offset_raw(st, range, val);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static int ad74416h_get_input_current_offset(struct ad74416h_state *st,
					     unsigned int channel, int *val)
{
	unsigned int range;
	int voltage_range;
	int voltage_offset;
	int ret;

	ret = ad74416h_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74416h_adc_range_to_voltage_range(st, range, &voltage_range);
	if (ret)
		return ret;

	ret = ad74416h_range_to_voltage_offset(st, range, &voltage_offset);
	if (ret)
		return ret;

	*val = voltage_offset * (int)AD74416H_ADC_RESULT_MAX / voltage_range;

	return IIO_VAL_INT;
}

static int ad74416h_get_raw_adc_result(struct ad74416h_state *st,
				       unsigned int channel,
				       unsigned int *val)
{
	unsigned int val_msb, val_lsb;
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_ADC_RESULT_UPR_X(channel), &val_msb);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD74416H_REG_ADC_RESULT_X(channel), &val_lsb);
	if (ret)
		return ret;

	*val = (FIELD_GET(AD74416H_CONV_RES_UPR_MSK, val_msb) << 16) |
	       FIELD_GET(AD74416H_CONV_RESULT_MSK, val_lsb);

	return 0;
}

static int ad74416h_set_adc_conv_seq(struct ad74416h_state *st,
				     unsigned int status)
{
	int ret;

	/*
	 * These bits do not clear when a conversion completes.
	 * To enable a subsequent conversion, repeat the write.
	 */
	ret = regmap_write_bits(st->regmap, AD74416H_REG_ADC_CONV_CTRL,
				AD74416H_CONV_SEQ_MASK,
				FIELD_PREP(AD74416H_CONV_SEQ_MASK, status));
	if (ret)
		return ret;

	/*
	 * Wait 100us before starting conversions.
	 */
	usleep_range(100, 120);

	return 0;
}

static int ad74416h_set_adc_channel_enable(struct ad74416h_state *st,
					   unsigned int channel,
					   bool status)
{
	return regmap_update_bits(st->regmap, AD74416H_REG_ADC_CONV_CTRL,
				  AD74416H_CH_EN_MASK(channel),
				  status ? AD74416H_CH_EN_MASK(channel) : 0);
}

static int _ad74416h_get_single_adc_result(struct ad74416h_state *st,
					   unsigned int channel, int *val)
{
	unsigned int uval;
	int ret;

	reinit_completion(&st->adc_data_completion);

	ret = ad74416h_set_adc_channel_enable(st, channel, true);
	if (ret)
		return ret;

	ret = ad74416h_set_adc_conv_seq(st, AD74416H_CONV_SEQ_SINGLE);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&st->adc_data_completion,
					  msecs_to_jiffies(1000));
	if (!ret) {
		ret = -ETIMEDOUT;
		return ret;
	}

	ret = ad74416h_get_raw_adc_result(st, channel, &uval);

	if (ret)
		return ret;

	ret = ad74416h_set_adc_conv_seq(st, AD74416H_CONV_SEQ_OFF);
	if (ret)
		return ret;

	ret = ad74416h_set_adc_channel_enable(st, channel, false);
	if (ret)
		return ret;

	*val = uval;

	return IIO_VAL_INT;
}

static int ad74416h_get_single_adc_result(struct iio_dev *indio_dev,
					  unsigned int channel, int *val)
{
	struct ad74416h_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = _ad74416h_get_single_adc_result(st, channel, val);
	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static void ad74416h_adc_to_resistance_result(int adc_result, int *val)
{
	if (adc_result == AD74416H_ADC_RESULT_MAX)
		adc_result = AD74416H_ADC_RESULT_MAX - 1;

	*val = DIV_ROUND_CLOSEST(adc_result * 2012,
				 AD74416H_ADC_RESULT_MAX - adc_result);
}

static int ad74416h_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long info)
{
	struct ad74416h_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			if (chan->output)
				return ad74416h_get_output_voltage_scale(st,
					chan->channel, val, val2);
			else
				return ad74416h_get_input_voltage_scale(st,
					chan->channel, val, val2);
		case IIO_CURRENT:
			if (chan->output)
				return ad74416h_get_output_current_scale(st,
					val, val2);
			else
				return ad74416h_get_input_current_scale(st,
					chan->channel, val, val2);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_VOLTAGE:
			return ad74416h_get_input_voltage_offset(st,
				chan->channel, val);
		case IIO_CURRENT:
			return ad74416h_get_input_current_offset(st,
				chan->channel, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_RAW:
		if (chan->output)
			return -EINVAL;

		return ad74416h_get_single_adc_result(indio_dev, chan->channel,
						      val);
	case IIO_CHAN_INFO_PROCESSED: {
		int ret;

		ret = ad74416h_get_single_adc_result(indio_dev, chan->channel,
						     val);
		if (ret < 0)
			return ret;

		ad74416h_adc_to_resistance_result(*val, val);

		return ret;
	}
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad74416h_get_adc_rate(st, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad74416h_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long info)
{
	struct ad74416h_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (!chan->output)
			return -EINVAL;

		if (val < 0 || val > AD74416H_DAC_CODE_MAX) {
			dev_err(st->dev, "Invalid DAC code\n");
			return -EINVAL;
		}

		return ad74416h_set_channel_dac_code(st, chan->channel, val);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad74416h_set_adc_rate(st, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad74416h_read_avail(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       const int **vals, int *type, int *length,
			       long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ad74416h_adc_sampling_rates;
		*length = ARRAY_SIZE(ad74416h_adc_sampling_rates);
		*type = IIO_VAL_INT;

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad74416h_set_gpio_config(struct ad74416h_state *st,
				    unsigned int offset, u8 mode)
{
	return regmap_update_bits(st->regmap, AD74416H_REG_GPIO_CONFIG_X(offset),
				  AD74416H_GPIO_CONFIG_SELECT_MASK, mode);
}

static void ad74416h_gpio_set(struct gpio_chip *chip,
			      unsigned int offset, int val)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int real_offset = st->gpo_gpio_offsets[offset];
	int ret;

	ret = ad74416h_set_gpio_config(st, real_offset,
				       AD74416H_GPIO_CONFIG_DATA);
	if (ret)
		return;

	regmap_update_bits(st->regmap, AD74416H_REG_GPIO_CONFIG_X(real_offset),
			   AD74416H_GPIO_CONFIG_DATA_MASK,
			   val ? AD74416H_GPIO_CONFIG_DATA_MASK : 0);
}

static void ad74416h_gpio_set_multiple(struct gpio_chip *chip,
				       unsigned long *mask,
				       unsigned long *bits)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int offset, real_offset, real_bits;

	for_each_set_bit(offset, mask, chip->ngpio) {
		real_offset = st->gpo_gpio_offsets[offset];
		real_bits = (*bits & offset);

		ad74416h_gpio_set(chip, real_offset, real_bits);
	}
}

static int ad74416h_gpio_set_gpio_config(struct gpio_chip *chip,
					 unsigned int offset,
					 unsigned long config)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int real_offset = st->gpo_gpio_offsets[offset];

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		return ad74416h_set_gpio_config(st, real_offset,
			AD74416H_GPIO_CONFIG_HIGH_Z);
	default:
		return -EOPNOTSUPP;
	}
}

static int ad74416h_set_comp_debounce(struct ad74416h_state *st,
				      unsigned int offset,
				      unsigned int debounce)
{
	unsigned int val = AD74416H_DIN_DEBOUNCE_LEN - 1;
	unsigned int i;

	for (i = 0; i < AD74416H_DIN_DEBOUNCE_LEN; i++)
		if (debounce <= ad74416h_debounce_map[i]) {
			val = i;
			break;
		}

	return regmap_update_bits(st->regmap,
				  AD74416H_REG_DIN_CONFIG0_X(offset),
				  AD74416H_DIN_DEBOUNCE_MASK,
				  val);
}

static int ad74416h_gpio_get_gpio_direction(struct gpio_chip *chip,
					    unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int ad74416h_gpio_set_comp_config(struct gpio_chip *chip,
					 unsigned int offset,
					 unsigned long config)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int real_offset = st->comp_gpio_offsets[offset];

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_INPUT_DEBOUNCE:
		return ad74416h_set_comp_debounce(st, real_offset,
			pinconf_to_config_argument(config));
	default:
		return -EOPNOTSUPP;
	}
}

static int ad74416h_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int real_offset = st->comp_gpio_offsets[offset];
	unsigned int status;
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_DIN_COMP_OUT, &status);
	if (ret)
		return ret;

	status &= BIT(real_offset);

	return status ? 1 : 0;
}

static int ad74416h_gpio_get_multiple(struct gpio_chip *chip,
				      unsigned long *mask,
				      unsigned long *bits)
{
	struct ad74416h_state *st = gpiochip_get_data(chip);
	unsigned int offset, real_offset, val;
	int ret;

	ret = regmap_read(st->regmap, AD74416H_REG_DIN_COMP_OUT, &val);
	if (ret)
		return ret;

	for_each_set_bit(offset, mask, chip->ngpio) {
		real_offset = st->comp_gpio_offsets[offset];

		__assign_bit(offset, bits, val & BIT(real_offset));
	}

	return ret;
}

static int ad74416h_update_scan_mode(struct iio_dev *indio_dev,
				     const unsigned long *active_scan_mask)
{
	struct ad74416h_state *st = iio_priv(indio_dev);
	struct spi_transfer *xfer = st->adc_samples_xfer;
	u8 *rx_buf = st->adc_samples_buf.rx_buf;
	u8 *tx_buf = st->adc_samples_tx_buf;
	unsigned int channel;
	int ret = -EINVAL;

	mutex_lock(&st->lock);

	spi_message_init(&st->adc_samples_msg);
	st->adc_active_channels = 0;

	for_each_clear_bit(channel, active_scan_mask, AD74416H_CHANNEL_MAX) {
		ret = ad74416h_set_adc_channel_enable(st, channel, false);
		if (ret)
			goto out;
	}

	if (*active_scan_mask == 0)
		goto out;

	/*
	 * The read select register is used to select which register's value
	 * will be sent by the slave on the next SPI frame.
	 *
	 * Create an SPI message that, on each step, writes to the read select
	 * register to select the ADC result of the next enabled channel, and
	 * reads the ADC result of the previous enabled channel.
	 *
	 * Example:
	 * W: [WCH1] [WCH2] [WCH2] [WCH3] [    ]
	 * R: [    ] [RCH1] [RCH2] [RCH3] [RCH4]
	 */

	for_each_set_bit(channel, active_scan_mask, AD74416H_CHANNEL_MAX) {
		ret = ad74416h_set_adc_channel_enable(st, channel, true);
		if (ret)
			goto out;

		st->adc_active_channels++;

		if (xfer == st->adc_samples_xfer)
			xfer->rx_buf = NULL;
		else
			xfer->rx_buf = rx_buf;

		xfer->tx_buf = tx_buf;
		xfer->len = AD74416H_FRAME_SIZE * 2;
		xfer->cs_change = 1;

		ad74416h_format_reg_write(st->dev_addr,
					  AD74416H_REG_READ_SELECT,
					  AD74416H_REG_ADC_RESULT_UPR_X(channel),
					  tx_buf);

		ad74416h_format_reg_write(st->dev_addr,
					  AD74416H_REG_READ_SELECT,
					  AD74416H_REG_ADC_RESULT_X(channel),
					  tx_buf + AD74416H_FRAME_SIZE);

		spi_message_add_tail(xfer, &st->adc_samples_msg);

		tx_buf += AD74416H_FRAME_SIZE * 2;
		if (xfer != st->adc_samples_xfer)
			rx_buf += AD74416H_FRAME_SIZE * 2;
		xfer++;
	}

	xfer->rx_buf = rx_buf;
	xfer->tx_buf = NULL;
	xfer->len = AD74416H_FRAME_SIZE * 2;
	xfer->cs_change = 0;

	spi_message_add_tail(xfer, &st->adc_samples_msg);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad74416h_gpio_get_comp_direction(struct gpio_chip *chip,
					    unsigned int offset)
{
	return GPIO_LINE_DIRECTION_IN;
}

static const struct iio_trigger_ops ad74416h_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static const struct iio_info ad74416h_info = {
	.read_raw = &ad74416h_read_raw,
	.write_raw = &ad74416h_write_raw,
	.read_avail = &ad74416h_read_avail,
	.update_scan_mode = &ad74416h_update_scan_mode,
};

static const struct regmap_config ad74416h_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.reg_read = ad74416h_reg_read,
	.reg_write = ad74416h_reg_write,
};

static irqreturn_t ad74416h_adc_data_interrupt(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ad74416h_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->adc_data_completion);

	return IRQ_HANDLED;
}

static irqreturn_t ad74416h_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad74416h_state *st = iio_priv(indio_dev);
	u8 *rx_buf = st->adc_samples_buf.rx_buf;
	unsigned int i;
	int ret;

	ret = spi_sync(st->spi, &st->adc_samples_msg);
	if (ret)
		goto out;

	for (i = 0; i < st->adc_active_channels; i++)
		ad74416h_crc_check(st, &rx_buf[i * AD74416H_FRAME_SIZE]);

	iio_push_to_buffers_with_timestamp(indio_dev, &st->adc_samples_buf,
					   iio_get_time_ns(indio_dev));

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ad74416h_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad74416h_state *st = iio_priv(indio_dev);

	return ad74416h_set_adc_conv_seq(st, AD74416H_CONV_SEQ_CONTINUOUS);
}

static int ad74416h_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad74416h_state *st = iio_priv(indio_dev);

	return ad74416h_set_adc_conv_seq(st, AD74416H_CONV_SEQ_OFF);
}

static const struct iio_buffer_setup_ops ad74416h_buffer_ops = {
	.postenable = &ad74416h_buffer_postenable,
	.predisable = &ad74416h_buffer_predisable,
};

static int ad74416h_parse_channel_config(struct iio_dev *indio_dev,
					 struct fwnode_handle *channel_node)
{
	struct ad74416h_state *st = iio_priv(indio_dev);
	struct ad74416h_channel_config *config;
	u32 index;
	int ret;

	ret = fwnode_property_read_u32(channel_node, "reg", &index);
	if (ret) {
		dev_err(st->dev, "Failed to read channel reg: %d\n", ret);
		return ret;
	}

	if (index >= AD74416H_CHANNEL_MAX) {
		dev_err(st->dev, "Channel index %u is too large\n", index);
		return -EINVAL;
	}

	config = &st->channel_configs[index];
	if (config->initialized) {
		dev_err(st->dev, "Channel %u already initialized\n", index);
		return -EINVAL;
	}

	config->func = CH_FUNC_HIGH_IMPEDANCE;
	fwnode_property_read_u32(channel_node, "adi,ch-func", &config->func);

	if (config->func < CH_FUNC_MIN || config->func > CH_FUNC_MAX) {
		dev_err(st->dev, "Invalid channel function %u\n", config->func);
		return -EINVAL;
	}

	if (config->func == CH_FUNC_DIGITAL_INPUT_LOGIC ||
	    config->func == CH_FUNC_DIGITAL_INPUT_LOOP_POWER)
		st->num_comparator_gpios++;

	config->gpio_comparator = fwnode_property_read_bool(channel_node,
							    "adi,gpo-comparator");

	fwnode_property_read_u32(channel_node, "drive-strength-microamp",
				 &config->drive_strength);

	if (!config->gpio_comparator)
		st->num_gpios++;

	indio_dev->num_channels += ad74416h_channels_map[config->func].num_channels;

	config->initialized = true;

	return 0;
}

static int ad74416h_parse_channel_configs(struct iio_dev *indio_dev)
{
	struct ad74416h_state *st = iio_priv(indio_dev);
	struct fwnode_handle *channel_node = NULL;
	int ret;

	fwnode_for_each_available_child_node(dev_fwnode(st->dev), channel_node) {
		ret = ad74416h_parse_channel_config(indio_dev, channel_node);
		if (ret)
			goto put_channel_node;
	}

	return 0;

put_channel_node:
	fwnode_handle_put(channel_node);

	return ret;
}

static int ad74416h_set_channel_function(struct ad74416h_state *st,
					 unsigned int channel, u8 func)
{
	int ret;

	ret = regmap_update_bits(st->regmap,
				 AD74416H_REG_CH_FUNC_SETUP_X(channel),
				 AD74416H_CH_FUNC_SETUP_MASK,
				 CH_FUNC_HIGH_IMPEDANCE);
	if (ret)
		return ret;

	ret = ad74416h_set_channel_dac_code(st, channel, 0);
	if (ret)
		return ret;

	/* Datasheet delay required before transition to new desired mode */
	usleep_range(200, 210);

	ret = regmap_update_bits(st->regmap,
				 AD74416H_REG_CH_FUNC_SETUP_X(channel),
				 AD74416H_CH_FUNC_SETUP_MASK, func);
	if (ret)
		return ret;

	/* Datasheet delay required before updating the new DAC code */
	usleep_range(200, 210);

	return ret;
}

static int ad74416h_setup_channels(struct iio_dev *indio_dev)
{
	struct ad74416h_state *st = iio_priv(indio_dev);
	struct ad74416h_channel_config *config;
	struct iio_chan_spec *channels, *chans;
	unsigned int i, num_chans, chan_i;
	int ret;

	channels = devm_kcalloc(st->dev, sizeof(*channels),
				indio_dev->num_channels, GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	indio_dev->channels = channels;

	for (i = 0; i < AD74416H_CHANNEL_MAX; i++) {
		config = &st->channel_configs[i];
		chans = ad74416h_channels_map[config->func].channels;
		num_chans = ad74416h_channels_map[config->func].num_channels;

		memcpy(channels, chans, num_chans * sizeof(*chans));

		for (chan_i = 0; chan_i < num_chans; chan_i++) {
			struct iio_chan_spec *chan = &channels[chan_i];

			chan->channel = i;
			if (chan->output)
				chan->scan_index = -1;
			else
				chan->scan_index = i;
		}

		ret = ad74416h_set_channel_function(st, i, config->func);
		if (ret)
			return ret;

		channels += num_chans;
	}

	return 0;
}

static int ad74416h_set_comp_drive_strength(struct ad74416h_state *st,
					    unsigned int offset,
					    unsigned int strength)
{
	strength = min(strength, 1800U);

	return regmap_update_bits(st->regmap, AD74416H_REG_DIN_CONFIG0_X(offset),
				  AD74416H_DIN_SINK_MASK,
				  FIELD_PREP(AD74416H_DIN_SINK_MASK, strength / 120));
}

static int ad74416h_setup_gpios(struct ad74416h_state *st)
{
	struct ad74416h_channel_config *config;
	unsigned int comp_gpio_i = 0;
	unsigned int gpo_gpio_i = 0;
	unsigned int i;
	u8 gpo_config;
	u32 strength;
	int ret;

	for (i = 0; i < AD74416H_CHANNEL_MAX; i++) {
		config = &st->channel_configs[i];

		if (config->gpio_comparator) {
			gpo_config = AD74416H_GPIO_CONFIG_COMP;
		} else {
			gpo_config = AD74416H_GPIO_CONFIG_DATA;
			st->gpo_gpio_offsets[gpo_gpio_i++] = i;
		}

		if (config->func == CH_FUNC_DIGITAL_INPUT_LOGIC ||
		    config->func == CH_FUNC_DIGITAL_INPUT_LOOP_POWER) {
			st->comp_gpio_offsets[comp_gpio_i++] = i;

			strength = config->drive_strength;
			ret = ad74416h_set_comp_drive_strength(st, i, strength);
			if (ret)
				return ret;
		}

		ret = ad74416h_set_gpio_config(st, i, gpo_config);
		if (ret)
			return ret;
	}

	return 0;
}

static int ad74416h_reset(struct ad74416h_state *st)
{
	int ret;

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		fsleep(50);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
		return 0;
	}

	ret = regmap_write(st->regmap, AD74416H_REG_CMD_KEY,
			   AD74416H_CMD_KEY_RESET1);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD74416H_REG_CMD_KEY,
			    AD74416H_CMD_KEY_RESET2);
}

static void ad74416h_regulator_disable(void *regulator)
{
	regulator_disable(regulator);
}

static int ad74416h_probe(struct spi_device *spi)
{
	struct ad74416h_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->dev = &spi->dev;

	mutex_init(&st->lock);
	init_completion(&st->adc_data_completion);

	st->regmap = devm_regmap_init(st->dev, NULL, st,
				      &ad74416h_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->reset_gpio = devm_gpiod_get_optional(st->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	st->dev_addr = 0;
	device_property_read_u32(st->dev, "device-address",
				 &st->dev_addr);
	if (st->dev_addr > 3)
		dev_err_probe(st->dev, -EINVAL, "invalid SPI device address.\n");

	st->refin_reg = devm_regulator_get(st->dev, "refin");
	if (IS_ERR(st->refin_reg))
		return dev_err_probe(st->dev, PTR_ERR(st->refin_reg),
				     "Failed to get refin regulator\n");

	ret = regulator_enable(st->refin_reg);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(st->dev, ad74416h_regulator_disable,
				       st->refin_reg);
	if (ret)
		return ret;

	st->sense_resistor_ohms = 120000000;
	device_property_read_u32(st->dev, "shunt-resistor-micro-ohms",
				 &st->sense_resistor_ohms);
	st->sense_resistor_ohms /= 1000000;

	st->trig = devm_iio_trigger_alloc(st->dev, "%s-dev%d",
					  "ad74416h", iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad74416h_trigger_ops;
	iio_trigger_set_drvdata(st->trig, st);

	ret = devm_iio_trigger_register(st->dev, st->trig);
	if (ret)
		return ret;

	indio_dev->name = "ad74416h";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad74416h_info;
	indio_dev->trig = iio_trigger_get(st->trig);

	ret = ad74416h_reset(st);
	if (ret)
		return ret;

	ret = ad74416h_parse_channel_configs(indio_dev);
	if (ret)
		return ret;

	ret = ad74416h_setup_channels(indio_dev);
	if (ret)
		return ret;

	ret = ad74416h_setup_gpios(st);
	if (ret)
		return ret;

	if (st->num_gpios) {
		st->gpo_gpiochip.owner = THIS_MODULE;
		st->gpo_gpiochip.label = "ad74416h";
		st->gpo_gpiochip.base = -1;
		st->gpo_gpiochip.ngpio = st->num_gpios;
		st->gpo_gpiochip.parent = st->dev;
		st->gpo_gpiochip.can_sleep = true;
		st->gpo_gpiochip.set = ad74416h_gpio_set;
		st->gpo_gpiochip.set_multiple = ad74416h_gpio_set_multiple;
		st->gpo_gpiochip.set_config = ad74416h_gpio_set_gpio_config;
		st->gpo_gpiochip.get_direction =
			ad74416h_gpio_get_gpio_direction;

		ret = devm_gpiochip_add_data(st->dev, &st->gpo_gpiochip, st);
		if (ret)
			return ret;
	}

	if (st->num_comparator_gpios) {
		st->comp_gpiochip.owner = THIS_MODULE;
		st->comp_gpiochip.label = "ad74416h";
		st->comp_gpiochip.base = -1;
		st->comp_gpiochip.ngpio = st->num_comparator_gpios;
		st->comp_gpiochip.parent = st->dev;
		st->comp_gpiochip.can_sleep = true;
		st->comp_gpiochip.get = ad74416h_gpio_get;
		st->comp_gpiochip.get_multiple = ad74416h_gpio_get_multiple;
		st->comp_gpiochip.set_config = ad74416h_gpio_set_comp_config;
		st->comp_gpiochip.get_direction =
			ad74416h_gpio_get_comp_direction;

		ret = devm_gpiochip_add_data(st->dev, &st->comp_gpiochip, st);
		if (ret)
			return ret;
	}

	ret = ad74416h_set_adc_conv_seq(st, AD74416H_CONV_SEQ_OFF);
	if (ret)
		return ret;

	ret = devm_request_irq(st->dev, spi->irq, ad74416h_adc_data_interrupt,
			       0, "ad74416h", indio_dev);
	if (ret)
		return dev_err_probe(st->dev, ret, "Failed to request irq\n");

	ret = devm_iio_triggered_buffer_setup(st->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad74416h_trigger_handler,
					      &ad74416h_buffer_ops);
	if (ret)
		return ret;

	return devm_iio_device_register(st->dev, indio_dev);
}

static int ad74416h_unregister_driver(struct spi_driver *spi)
{
	spi_unregister_driver(spi);

	return 0;
}

static int __init ad74416h_register_driver(struct spi_driver *spi)
{
	crc8_populate_msb(ad74416h_crc8_table, AD74416H_CRC_POLYNOMIAL);

	return spi_register_driver(spi);
}

static const struct spi_device_id ad74416h_id[] = {
	{ "ad74416h", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad74416h_id);

static const struct of_device_id ad74416h_of_match[] = {
	{ .compatible = "adi,ad74416h" },
	{}
};

static struct spi_driver ad74416h_driver = {
	.driver = {
		.name = "ad74416h",
		.of_match_table = ad74416h_of_match,
	},
	.probe = ad74416h_probe,
	.id_table = ad74416h_id,
};

module_driver(ad74416h_driver,
	      ad74416h_register_driver,
	      ad74416h_unregister_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD74416H ADDAC");
MODULE_LICENSE("GPL");
