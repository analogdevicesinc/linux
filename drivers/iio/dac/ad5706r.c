// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5706R 16-bit Current Output Digital to Analog Converter
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

/* SPI frame layout */
#define AD5706R_RD_MASK			BIT(15)
#define AD5706R_ADDR_PIN_MASK		GENMASK(14, 13)
#define AD5706R_ADDR_MASK		GENMASK(11, 0)

/* Registers */
#define AD5706R_REG_INTERFACE_CONFIG_A		0x00
#define AD5706R_MASK_ADDR_ASCENSION		BIT(5)
#define AD5706R_REG_INTERFACE_CONFIG_B		0x01
#define AD5706R_MASK_SINGLE_INSTR		BIT(7)
#define AD5706R_REG_MUX_OUT_SEL			0x54
#define AD5706R_REG_MUX_OUT_CONTROL		0x56
#define AD5706R_REG_MULTI_DAC_SW_LDAC		0x5A
#define AD5706R_REG_MULTI_DAC_INPUT_A		0x5C
#define AD5706R_MASK_MULTI_DAC_INPUT_A		GENMASK(15, 0)
#define AD5706R_REG_DAC_INPUT_A_CH(x)		(0x60 + ((x) * 2))
#define AD5706R_REG_DAC_DATA_READBACK_CH(x)	(0x68 + ((x) * 2))
#define AD5706R_REG_BANDGAP_CONTROL		0x73

#define AD5706R_DAC_RESOLUTION		16
#define AD5706R_DAC_MAX_CODE		GENMASK(15, 0)
#define AD5706R_MULTIBYTE_REG_START	0x14
#define AD5706R_MULTIBYTE_REG_END	0x71
#define AD5706R_MAX_REG			0x77
#define AD5706R_DEV_ADDR_MAX		3

#define AD5706R_SAMPLING_FREQ_MIN_HZ	1
#define AD5706R_SAMPLING_FREQ_MAX_HZ	(10 * HZ_PER_MHZ)

/* Device attribute enums */
enum addr_ascension_iio_dev_attr {
	ADDR_ASCENSION_DECREMENT,
	ADDR_ASCENSION_INCREMENT,
};

static const char * const addr_ascension_iio_dev_attr_vals[] = {
	[ADDR_ASCENSION_DECREMENT] = "decrement",
	[ADDR_ASCENSION_INCREMENT] = "increment",
};

enum single_instr_iio_dev_attr {
	SINGLE_INSTR_STREAMING,
	SINGLE_INSTR_SINGLE_INSTRUCTION,
};

static const char * const single_instr_iio_dev_attr_vals[] = {
	[SINGLE_INSTR_STREAMING] = "streaming",
	[SINGLE_INSTR_SINGLE_INSTRUCTION] = "single_instruction",
};

enum hw_ldac_tg_state_iio_dev_attr {
	HW_LDAC_TG_STATE_LOW,
	HW_LDAC_TG_STATE_HIGH,
};

static const char * const hw_ldac_tg_state_iio_dev_attr_vals[] = {
	[HW_LDAC_TG_STATE_LOW] = "low",
	[HW_LDAC_TG_STATE_HIGH] = "high",
};

enum hw_ldac_tg_pwm_iio_dev_attr {
	HW_LDAC_TG_PWM_DISABLED,
	HW_LDAC_TG_PWM_ENABLED,
};

static const char * const hw_ldac_tg_pwm_iio_dev_attr_vals[] = {
	[HW_LDAC_TG_PWM_DISABLED] = "disable",
	[HW_LDAC_TG_PWM_ENABLED] = "enable",
};

enum mux_out_sel_iio_dev_attr {
	MUX_OUT_SEL_DISABLED,
	MUX_OUT_SEL_AGND,
	MUX_OUT_SEL_AVDD,
	MUX_OUT_SEL_VREF,
	MUX_OUT_SEL_IOUT0_VMON,
	MUX_OUT_SEL_IOUT1_VMON,
	MUX_OUT_SEL_IOUT2_VMON,
	MUX_OUT_SEL_IOUT3_VMON,
	MUX_OUT_SEL_IOUT0_IMON,
	MUX_OUT_SEL_IOUT1_IMON,
	MUX_OUT_SEL_IOUT2_IMON,
	MUX_OUT_SEL_IOUT3_IMON,
	MUX_OUT_SEL_PVDD0,
	MUX_OUT_SEL_PVDD1,
	MUX_OUT_SEL_PVDD2,
	MUX_OUT_SEL_PVDD3,
	MUX_OUT_SEL_TEMP_SENSOR0,
	MUX_OUT_SEL_TEMP_SENSOR1,
	MUX_OUT_SEL_TEMP_SENSOR2,
	MUX_OUT_SEL_TEMP_SENSOR3,
	MUX_OUT_SEL_MUX_IN0,
	MUX_OUT_SEL_MUX_IN1,
	MUX_OUT_SEL_MUX_IN2,
	MUX_OUT_SEL_MUX_IN3,
};

static const char * const mux_out_sel_iio_dev_attr_vals[] = {
	[MUX_OUT_SEL_DISABLED]		= "disabled",
	[MUX_OUT_SEL_AGND]		= "agnd",
	[MUX_OUT_SEL_AVDD]		= "avdd",
	[MUX_OUT_SEL_VREF]		= "vref",
	[MUX_OUT_SEL_IOUT0_VMON]	= "iout0_vmon",
	[MUX_OUT_SEL_IOUT1_VMON]	= "iout1_vmon",
	[MUX_OUT_SEL_IOUT2_VMON]	= "iout2_vmon",
	[MUX_OUT_SEL_IOUT3_VMON]	= "iout3_vmon",
	[MUX_OUT_SEL_IOUT0_IMON]	= "iout0_imon",
	[MUX_OUT_SEL_IOUT1_IMON]	= "iout1_imon",
	[MUX_OUT_SEL_IOUT2_IMON]	= "iout2_imon",
	[MUX_OUT_SEL_IOUT3_IMON]	= "iout3_imon",
	[MUX_OUT_SEL_PVDD0]		= "pvdd0",
	[MUX_OUT_SEL_PVDD1]		= "pvdd1",
	[MUX_OUT_SEL_PVDD2]		= "pvdd2",
	[MUX_OUT_SEL_PVDD3]		= "pvdd3",
	[MUX_OUT_SEL_TEMP_SENSOR0]	= "tdiode_ch0",
	[MUX_OUT_SEL_TEMP_SENSOR1]	= "tdiode_ch1",
	[MUX_OUT_SEL_TEMP_SENSOR2]	= "tdiode_ch2",
	[MUX_OUT_SEL_TEMP_SENSOR3]	= "tdiode_ch3",
	[MUX_OUT_SEL_MUX_IN0]		= "mux_in0",
	[MUX_OUT_SEL_MUX_IN1]		= "mux_in1",
	[MUX_OUT_SEL_MUX_IN2]		= "mux_in2",
	[MUX_OUT_SEL_MUX_IN3]		= "mux_in3",
};

static const u8 mux_out_sel_reg_vals[] = {
	[MUX_OUT_SEL_DISABLED]		= 0x00,
	[MUX_OUT_SEL_AGND]		= 0x80,
	[MUX_OUT_SEL_AVDD]		= 0x81,
	[MUX_OUT_SEL_VREF]		= 0x82,
	[MUX_OUT_SEL_IOUT0_VMON]	= 0x84,
	[MUX_OUT_SEL_IOUT1_VMON]	= 0x85,
	[MUX_OUT_SEL_IOUT2_VMON]	= 0x86,
	[MUX_OUT_SEL_IOUT3_VMON]	= 0x87,
	[MUX_OUT_SEL_IOUT0_IMON]	= 0x88,
	[MUX_OUT_SEL_IOUT1_IMON]	= 0x89,
	[MUX_OUT_SEL_IOUT2_IMON]	= 0x8A,
	[MUX_OUT_SEL_IOUT3_IMON]	= 0x8B,
	[MUX_OUT_SEL_PVDD0]		= 0x8C,
	[MUX_OUT_SEL_PVDD1]		= 0x8D,
	[MUX_OUT_SEL_PVDD2]		= 0x8E,
	[MUX_OUT_SEL_PVDD3]		= 0x8F,
	[MUX_OUT_SEL_TEMP_SENSOR0]	= 0x90,
	[MUX_OUT_SEL_TEMP_SENSOR1]	= 0x91,
	[MUX_OUT_SEL_TEMP_SENSOR2]	= 0x92,
	[MUX_OUT_SEL_TEMP_SENSOR3]	= 0x93,
	[MUX_OUT_SEL_MUX_IN0]		= 0x94,
	[MUX_OUT_SEL_MUX_IN1]		= 0x95,
	[MUX_OUT_SEL_MUX_IN2]		= 0x96,
	[MUX_OUT_SEL_MUX_IN3]		= 0x97,
};

enum multi_dac_sw_ldac_trigger_iio_dev_attr {
	MULTI_DAC_SW_LDAC_TRIGGER_LOW = 0,
	MULTI_DAC_SW_LDAC_TRIGGER_TRIGGER,
};

static const char * const multi_dac_sw_ldac_trigger_iio_dev_attr_vals[] = {
	[MULTI_DAC_SW_LDAC_TRIGGER_LOW] = "low",
	[MULTI_DAC_SW_LDAC_TRIGGER_TRIGGER] = "trigger",
};

enum hw_shutdown_state_iio_dev_attr {
	HW_SHUTDOWN_STATE_LOW,
	HW_SHUTDOWN_STATE_HIGH,
};

static const char * const hw_shutdown_state_iio_dev_attr_vals[] = {
	[HW_SHUTDOWN_STATE_LOW] = "low",
	[HW_SHUTDOWN_STATE_HIGH] = "high",
};

struct ad5706r_state {
	struct spi_device *spi;
	struct regmap *regmap;

	struct pwm_device *ldac_pwm;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *shdn_gpio;
	struct regulator *vref;

	u8 shift_val;

	u8 dev_addr;
	enum addr_ascension_iio_dev_attr addr_ascension;
	enum single_instr_iio_dev_attr single_instr;
	enum hw_ldac_tg_state_iio_dev_attr hw_ldac_tg_state;
	u64 sampling_frequency;
	enum mux_out_sel_iio_dev_attr mux_out_sel;

	u8 tx_buf[4] __aligned(IIO_DMA_MINALIGN);
	u8 rx_buf[4];
};

static int ad5706r_reg_len(unsigned int reg)
{
	if (reg >= AD5706R_MULTIBYTE_REG_START && reg <= AD5706R_MULTIBYTE_REG_END)
		return 2;

	return 1;
}

static int ad5706r_regmap_write(void *context, const void *data, size_t count)
{
	struct ad5706r_state *st = context;
	unsigned int num_bytes;
	u16 reg, val, cmd;

	if (count != 4)
		return -EINVAL;

	reg = get_unaligned_be16(data);
	val = get_unaligned_be16(data + 2);
	num_bytes = ad5706r_reg_len(reg);

	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.len = num_bytes + 2,
	};

	if (num_bytes == 1) {
		st->tx_buf[2] = (u8)val;
	} else if (num_bytes == 2) {
		put_unaligned_be16(val, &st->tx_buf[2]);
		
		/*
		 * In addr ascension DECREMENT, write starts on next register for
		 * multibyte registers. For INCREMENT, write values are swapped
		 * as per device expectation
		 */
		if (st->addr_ascension == ADDR_ASCENSION_DECREMENT)
			reg = reg + 1;
		else
			swap(st->tx_buf[2], st->tx_buf[3]);
	} else {
		return -EINVAL;
	}

	/* Construct command word: [dev_addr(14:13)] [reg_addr(11:0)] */
	cmd = ((st->dev_addr << 13) & AD5706R_ADDR_PIN_MASK) | (reg & AD5706R_ADDR_MASK);
	put_unaligned_be16(cmd, &st->tx_buf[0]);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad5706r_regmap_read(void *context, const void *reg_buf,
			       size_t reg_size, void *val_buf, size_t val_size)
{
	struct ad5706r_state *st = context;
	unsigned int num_bytes;
	u16 reg, cmd, val;
	int ret;

	if (reg_size != 2 || val_size != 2)
		return -EINVAL;

	reg = get_unaligned_be16(reg_buf);
	num_bytes = ad5706r_reg_len(reg);

	/* Full duplex, device responds immediately after command */
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = 2 + num_bytes,
	};

	/*
	 * In addr ascension DECREMENT, read starts on next register for
	 * multibyte registers
	 */
	if (num_bytes == 2 && st->addr_ascension == ADDR_ASCENSION_DECREMENT)
		reg = reg + 1;

	/* Construct command word: [RD(15)][dev_addr(14:13)] [reg_addr(11:0)] */
	cmd = AD5706R_RD_MASK | ((st->dev_addr << 13) & AD5706R_ADDR_PIN_MASK) |
	      (reg & AD5706R_ADDR_MASK);
	put_unaligned_be16(cmd, &st->tx_buf[0]);
	put_unaligned_be16(0, &st->tx_buf[2]);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	/*
	 * Extract value from response (skip 2-byte command echo)
	 * In addr ascension INCREMENT, multibyte values are swapped
	 * as per device expectation
	 */
	if (num_bytes == 1) {
		val = st->rx_buf[2];
	} else if (num_bytes == 2) {
		val = get_unaligned_be16(&st->rx_buf[2]);

		if (st->addr_ascension == ADDR_ASCENSION_INCREMENT)
			val = swab16(val);
	} else {
		return -EINVAL;
	}

	put_unaligned_be16(val, val_buf);

	return 0;
}

static int ad5706r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg, reg_val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		reg = AD5706R_REG_DAC_DATA_READBACK_CH(chan->channel);
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;

		*val = reg_val;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 50;
		*val2 = AD5706R_DAC_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_frequency;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad5706r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldac_pwm_state;
	unsigned int reg;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > AD5706R_DAC_MAX_CODE)
			return -EINVAL;

		reg = AD5706R_REG_DAC_INPUT_A_CH(chan->channel);
		return regmap_write(st->regmap, reg, val);
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Sets minimum and maximum frequency */
		val = clamp_t(int, val, AD5706R_SAMPLING_FREQ_MIN_HZ, AD5706R_SAMPLING_FREQ_MAX_HZ);

		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		ldac_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, val);
		ldac_pwm_state.duty_cycle = DIV_ROUND_CLOSEST_ULL(ldac_pwm_state.period, 2);
		ldac_pwm_state.enabled = true;

		ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
		if (ret)
			return ret;

		st->sampling_frequency = val;

		return 0;
	default:
		return -EINVAL;
	}
}

static int _set_pwm_duty_cycle(struct ad5706r_state *st, int duty_cycle)
{
	struct pwm_state ldac_pwm_state;
	int ret, val;

	if (!st->ldac_pwm)
		return -ENODEV;

	pwm_get_state(st->ldac_pwm, &ldac_pwm_state);

	val = DIV_ROUND_CLOSEST_ULL(ldac_pwm_state.period * duty_cycle, 100);
	ldac_pwm_state.duty_cycle = val;

	ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
	if (ret)
		return ret;

	return 0;
}

static ssize_t ad5706r_dev_addr_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	u8 reg_val;
	int ret;

	ret = kstrtou8(buf, 10, &reg_val);
	if (ret)
		return ret;

	if (reg_val > AD5706R_DEV_ADDR_MAX)
		return -EINVAL;

	st->dev_addr = reg_val;

	return len;
}

static ssize_t ad5706r_dev_addr_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%u\n", st->dev_addr);
}

static int ad5706r_addr_ascension_write(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret, val;

	if (item >= ARRAY_SIZE(addr_ascension_iio_dev_attr_vals))
		return -EINVAL;

	if (item == ADDR_ASCENSION_DECREMENT)
		val = 0;
	else
		val = 1;

	val = FIELD_PREP(AD5706R_MASK_ADDR_ASCENSION, val);

	ret = regmap_update_bits(st->regmap, AD5706R_REG_INTERFACE_CONFIG_A,
				 AD5706R_MASK_ADDR_ASCENSION, val);
	if (ret)
		return ret;

	st->addr_ascension = item;

	return 0;
}

static int ad5706r_addr_ascension_read(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, AD5706R_REG_INTERFACE_CONFIG_A, &reg_val);
	if (ret)
		return ret;

	if (reg_val & AD5706R_MASK_ADDR_ASCENSION)
		st->addr_ascension = ADDR_ASCENSION_INCREMENT;
	else
		st->addr_ascension = ADDR_ASCENSION_DECREMENT;

	return st->addr_ascension;
}

static const struct iio_enum ad5706r_addr_ascension_enum = {
	.items = addr_ascension_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(addr_ascension_iio_dev_attr_vals),
	.set = ad5706r_addr_ascension_write,
	.get = ad5706r_addr_ascension_read,
};

static int ad5706r_single_instr_write(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;
	int val;

	if (item >= ARRAY_SIZE(single_instr_iio_dev_attr_vals))
		return -EINVAL;

	if (item == SINGLE_INSTR_STREAMING)
		val = 0;
	else
		val = 1;

	val = FIELD_PREP(AD5706R_MASK_SINGLE_INSTR, val);

	ret = regmap_update_bits(st->regmap, AD5706R_REG_INTERFACE_CONFIG_B,
				 AD5706R_MASK_SINGLE_INSTR, val);
	if (ret)
		return ret;

	st->single_instr = item;

	return 0;
}

static int ad5706r_single_instr_read(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, AD5706R_REG_INTERFACE_CONFIG_B, &reg_val);
	if (ret)
		return ret;

	if (reg_val & AD5706R_MASK_SINGLE_INSTR)
		st->single_instr = SINGLE_INSTR_SINGLE_INSTRUCTION;
	else
		st->single_instr = SINGLE_INSTR_STREAMING;

	return st->single_instr;
}

static const struct iio_enum ad5706r_single_instr_enum = {
	.items = single_instr_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(single_instr_iio_dev_attr_vals),
	.set = ad5706r_single_instr_write,
	.get = ad5706r_single_instr_read,
};

static int ad5706r_hw_ldac_tg_state_write(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	if (item >= ARRAY_SIZE(hw_ldac_tg_state_iio_dev_attr_vals))
		return -EINVAL;

	/* Setting either high or low can override hw_ldac_tg_pwm */
	if (item == HW_LDAC_TG_STATE_HIGH)
		ret = _set_pwm_duty_cycle(st, 100);
	else
		ret = _set_pwm_duty_cycle(st, 0);

	if (ret)
		return ret;

	st->hw_ldac_tg_state = item;

	return 0;
}

static int ad5706r_hw_ldac_tg_state_read(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	/* Simply return previously set state, even if pwm is running */
	return st->hw_ldac_tg_state;
}

static const struct iio_enum ad5706r_hw_ldac_tg_state_enum = {
	.items = hw_ldac_tg_state_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_ldac_tg_state_iio_dev_attr_vals),
	.set = ad5706r_hw_ldac_tg_state_write,
	.get = ad5706r_hw_ldac_tg_state_read,
};

static int ad5706r_hw_ldac_tg_pwm_write(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	if (item >= ARRAY_SIZE(hw_ldac_tg_pwm_iio_dev_attr_vals))
		return -EINVAL;

	/* Enable = 50% duty, Disable notes present value of hw_ldac_tg_state */
	if (item == HW_LDAC_TG_PWM_DISABLED)
		if (st->hw_ldac_tg_state == HW_LDAC_TG_STATE_LOW)
			ret = _set_pwm_duty_cycle(st, 0);
		else
			ret = _set_pwm_duty_cycle(st, 100);
	else
		ret = _set_pwm_duty_cycle(st, 50);

	return ret;
}

static int ad5706r_hw_ldac_tg_pwm_read(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldac_pwm_state;

	pwm_get_state(st->ldac_pwm, &ldac_pwm_state);

	if (ldac_pwm_state.duty_cycle == 0 ||
	    ldac_pwm_state.duty_cycle == ldac_pwm_state.period)
		return HW_LDAC_TG_PWM_DISABLED;
	else
		return HW_LDAC_TG_PWM_ENABLED;
}

static const struct iio_enum ad5706r_hw_ldac_tg_pwm_enum = {
	.items = hw_ldac_tg_pwm_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_ldac_tg_pwm_iio_dev_attr_vals),
	.set = ad5706r_hw_ldac_tg_pwm_write,
	.get = ad5706r_hw_ldac_tg_pwm_read,
};

static int ad5706r_mux_out_sel_write(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int ret;

	if (item >= ARRAY_SIZE(mux_out_sel_iio_dev_attr_vals))
		return -EINVAL;

	ret = regmap_write(st->regmap, AD5706R_REG_MUX_OUT_SEL,
			   mux_out_sel_reg_vals[item]);
	if (ret)
		return ret;

	st->mux_out_sel = item;

	return 0;
}

static int ad5706r_mux_out_sel_read(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	return st->mux_out_sel;
}

static const struct iio_enum ad5706r_mux_out_sel_enum = {
	.items = mux_out_sel_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(mux_out_sel_iio_dev_attr_vals),
	.set = ad5706r_mux_out_sel_write,
	.get = ad5706r_mux_out_sel_read,
};

static ssize_t ad5706r_multi_dac_input_a_write(struct iio_dev *indio_dev,
					       uintptr_t private,
					       const struct iio_chan_spec *chan,
					       const char *buf, size_t len)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = kstrtou32(buf, 16, &val);
	if (ret)
		return ret;

	if (val > 0xFFFF)
		return -EINVAL;

	ret = regmap_write(st->regmap, AD5706R_REG_MULTI_DAC_INPUT_A, val);
	if (ret)
		return ret;

	return len;
}

static ssize_t ad5706r_multi_dac_input_a_read(struct iio_dev *indio_dev,
					      uintptr_t private,
					      const struct iio_chan_spec *chan,
					      char *buf)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, AD5706R_REG_MULTI_DAC_INPUT_A, &reg_val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%04x\n", reg_val);
}

static int ad5706r_multi_dac_sw_ldac_trigger_write(struct iio_dev *indio_dev,
						   const struct iio_chan_spec *chan,
						   unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	if (item >= ARRAY_SIZE(multi_dac_sw_ldac_trigger_iio_dev_attr_vals))
		return -EINVAL;

	return regmap_write(st->regmap, AD5706R_REG_MULTI_DAC_SW_LDAC, item);
}

static int ad5706r_multi_dac_sw_ldac_trigger_read(struct iio_dev *indio_dev,
						  const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, AD5706R_REG_MULTI_DAC_SW_LDAC, &reg_val);
	if (ret)
		return ret;

	return reg_val;
}

static const struct iio_enum ad5706r_multi_dac_sw_ldac_trigger_enum = {
	.items = multi_dac_sw_ldac_trigger_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(multi_dac_sw_ldac_trigger_iio_dev_attr_vals),
	.set = ad5706r_multi_dac_sw_ldac_trigger_write,
	.get = ad5706r_multi_dac_sw_ldac_trigger_read,
};

static int ad5706r_hw_shutdown_state_write(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   unsigned int item)
{
	struct ad5706r_state *st = iio_priv(indio_dev);

	if (item >= ARRAY_SIZE(hw_shutdown_state_iio_dev_attr_vals))
		return -EINVAL;

	if (item == HW_SHUTDOWN_STATE_LOW)
		gpiod_set_value_cansleep(st->shdn_gpio, 0);
	else
		gpiod_set_value_cansleep(st->shdn_gpio, 1);

	return 0;
}

static int ad5706r_hw_shutdown_state_read(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	int val;

	val = gpiod_get_value_cansleep(st->shdn_gpio);

	if (val == 0)
		return HW_SHUTDOWN_STATE_LOW;
	else
		return HW_SHUTDOWN_STATE_HIGH;
}

static const struct iio_enum ad5706r_hw_shutdown_state_enum = {
	.items = hw_shutdown_state_iio_dev_attr_vals,
	.num_items = ARRAY_SIZE(hw_shutdown_state_iio_dev_attr_vals),
	.set = ad5706r_hw_shutdown_state_write,
	.get = ad5706r_hw_shutdown_state_read,
};

static const struct regmap_bus ad5706r_regmap_bus = {
	.write = ad5706r_regmap_write,
	.read = ad5706r_regmap_read,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad5706r_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = AD5706R_MAX_REG,
	.cache_type = REGCACHE_NONE,
};

static const struct iio_info ad5706r_info = {
	.read_raw = ad5706r_read_raw,
	.write_raw = ad5706r_write_raw,
};

#define AD5706R_CHAN_EXT_INFO(_name, _what, _shared, _read, _write) {	\
	.name = _name,							\
	.read = (_read),						\
	.write = (_write),						\
	.private = (_what),						\
	.shared = (_shared),						\
}

static struct iio_chan_spec_ext_info ad5706r_ext_info[] = {
	AD5706R_CHAN_EXT_INFO("dev_addr", 0, IIO_SHARED_BY_ALL,
			      ad5706r_dev_addr_read, ad5706r_dev_addr_write),
	IIO_ENUM("addr_ascension", IIO_SHARED_BY_ALL, &ad5706r_addr_ascension_enum),
	IIO_ENUM_AVAILABLE("addr_ascension", IIO_SHARED_BY_ALL, &ad5706r_addr_ascension_enum),
	IIO_ENUM("single_instr", IIO_SHARED_BY_ALL, &ad5706r_single_instr_enum),
	IIO_ENUM_AVAILABLE("single_instr", IIO_SHARED_BY_ALL, &ad5706r_single_instr_enum),
	IIO_ENUM("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),
	IIO_ENUM("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),
	IIO_ENUM("mux_out_sel", IIO_SHARED_BY_ALL, &ad5706r_mux_out_sel_enum),
	IIO_ENUM_AVAILABLE("mux_out_sel", IIO_SHARED_BY_ALL, &ad5706r_mux_out_sel_enum),
	AD5706R_CHAN_EXT_INFO("multi_dac_input_a", 0, IIO_SHARED_BY_ALL,
			      ad5706r_multi_dac_input_a_read, ad5706r_multi_dac_input_a_write),
	IIO_ENUM("multi_dac_sw_ldac_trigger", IIO_SHARED_BY_ALL,
		 &ad5706r_multi_dac_sw_ldac_trigger_enum),
	IIO_ENUM_AVAILABLE("multi_dac_sw_ldac_trigger", IIO_SHARED_BY_ALL,
			   &ad5706r_multi_dac_sw_ldac_trigger_enum),
	IIO_ENUM("hw_shutdown_state", IIO_SHARED_BY_ALL, &ad5706r_hw_shutdown_state_enum),
	IIO_ENUM_AVAILABLE("hw_shutdown_state", IIO_SHARED_BY_ALL, &ad5706r_hw_shutdown_state_enum),
	{ }
};

#define AD5706R_CHAN(_channel) {				\
	.type = IIO_CURRENT,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.output = 1,						\
	.indexed = 1,						\
	.channel = _channel,					\
	.ext_info = ad5706r_ext_info,				\
}

static const struct iio_chan_spec ad5706r_channels[] = {
	AD5706R_CHAN(0),
	AD5706R_CHAN(1),
	AD5706R_CHAN(2),
	AD5706R_CHAN(3),
};

static int ad5706r_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5706r_state *st;
	struct pwm_state ldac_pwm_state;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	/* Initialize default values for device attributes */
	st->sampling_frequency = HZ_PER_MHZ;

	/* Get optional PWM for LDAC triggering */
	st->ldac_pwm = devm_pwm_get(dev, "ldac");
	if (IS_ERR(st->ldac_pwm))
		return dev_err_probe(dev, PTR_ERR(st->ldac_pwm),
				     "Failed to get LDAC PWM\n");
	pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
	ldac_pwm_state.duty_cycle = 0;
	ldac_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, st->sampling_frequency);
	ldac_pwm_state.enabled = true;
	ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to apply PWM state\n");

	/* Get optional GPIOs */
	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	st->shdn_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(st->shdn_gpio))
		return dev_err_probe(dev, PTR_ERR(st->shdn_gpio),
				     "Failed to get enable GPIO\n");

	/* Initialize regmap with custom read/write functions */
	st->regmap = devm_regmap_init(dev, &ad5706r_regmap_bus,
				      st, &ad5706r_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap\n");

	/* Voltage reference - external or internal */
	st->vref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(st->vref)) {
		ret = PTR_ERR(st->vref);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret,
					     "Failed to get vref regulator\n");
		st->vref = NULL;
	}

	if (st->vref) {
		/* External reference provided */
		ret = regulator_enable(st->vref);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable vref regulator\n");
	} else {
		/* Use internal reference */
		ret = regmap_write(st->regmap, AD5706R_REG_BANDGAP_CONTROL, 1);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable internal reference\n");
	}

	indio_dev->name = "ad5706r";
	indio_dev->info = &ad5706r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5706r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5706r_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad5706r_of_match[] = {
	{ .compatible = "adi,ad5706r" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5706r_of_match);

static const struct spi_device_id ad5706r_id[] = {
	{ "ad5706r" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5706r_id);

static struct spi_driver ad5706r_driver = {
	.driver = {
		.name = "ad5706r",
		.of_match_table = ad5706r_of_match,
	},
	.probe = ad5706r_probe,
	.id_table = ad5706r_id,
};
module_spi_driver(ad5706r_driver);

MODULE_AUTHOR("Alexis Czezar Torreno <alexisczezar.torreno@analog.com>");
MODULE_DESCRIPTION("AD5706R 16-bit Current Output DAC driver");
MODULE_LICENSE("GPL");
