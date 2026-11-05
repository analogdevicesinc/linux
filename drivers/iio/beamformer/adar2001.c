// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR2001 10 GHz to 40 GHz, 1:4 Channel, 4x Frequency Multiplier/Filter
 * Transmitter
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADAR2001_REG_INTERFACE_CONFIG_A		0x000
#define   ADAR2001_SOFTRESET			(BIT(7) | BIT(0))
#define   ADAR2001_SDOACTIVE			(BIT(4) | BIT(3))
#define ADAR2001_REG_SCRATCH_PAD		0x00A
#define ADAR2001_REG_PWRON			0x010
#define   ADAR2001_PWRON_MSK			BIT(0)
#define ADAR2001_REG_BIAS_CURRENT_MULT1		0x011
#define   ADAR2001_MULT_MID_BIAS_MSK		GENMASK(7, 4)
#define   ADAR2001_MULT_LOW_BIAS_MSK		GENMASK(3, 0)
#define ADAR2001_REG_BIAS_CURRENT_MULT2		0x012
#define   ADAR2001_MULT_HIGH_BIAS_MSK		GENMASK(3, 0)
#define ADAR2001_REG_BIAS_CURRENT_RF_AMP	0x013
#define   ADAR2001_RF_AMP2_BIAS_MSK		GENMASK(7, 4)
#define   ADAR2001_RF_AMP1_BIAS_MSK		GENMASK(3, 0)
#define ADAR2001_REG_BIAS_CURRENT_SPLT		0x014
#define   ADAR2001_SPLT2_BIAS_MSK		GENMASK(7, 4)
#define   ADAR2001_SPLT1_BIAS_MSK		GENMASK(3, 0)
#define ADAR2001_REG_BIAS_CURRENT_PA		0x015
#define   ADAR2001_PA_BIAS_MSK			GENMASK(3, 0)
#define ADAR2001_REG_TX_SEQUENCER_SETUP		0x016
#define ADAR2001_REG_TX_SEQUENCER_SETUP2	0x017
#define   ADAR2001_TX_STATES_MSK		GENMASK(6, 0)
#define ADAR2001_REG_MULT_SEQUENCER_SETUP	0x018
#define   ADAR2001_MULT_STATES_MSK		GENMASK(3, 0)
#define   ADAR2001_SEQ_EN_MSK			BIT(7)
#define   ADAR2001_SLP_CTRL_MSK			BIT(6)
#define   ADAR2001_SLP_HOLD_MSK			BIT(5)
#define   ADAR2001_CTL_LATCH_BYP_MSK		BIT(4)
#define ADAR2001_REG_TX_STATES			0x019
#define ADAR2001_REG_MULT_STATES		0x03C
#define ADAR2001_REG_SEQUENCER_CTRL_SPI		0x044
#define   ADAR2001_MULT_RST_SPI_MSK		BIT(3)
#define   ADAR2001_MULT_ADV_SPI_MSK		BIT(2)
#define   ADAR2001_TX_RST_SPI_MSK		BIT(1)
#define   ADAR2001_TX_ADV_SPI_MSK		BIT(0)
#define ADAR2001_REG_TX_EN1_SPI			0x045
/* CHx_RDY/CHx_ACT, two bits per channel, channel 0 in bits 7:6. */
#define   ADAR2001_PA_STATE_MSK(_ch)		GENMASK(7 - 2 * (_ch), 6 - 2 * (_ch))
#define ADAR2001_REG_TX_EN2_SPI			0x046
#define   ADAR2001_SPLT1_EN_MSK			BIT(2)
#define   ADAR2001_SPLT12_EN_MSK		BIT(1)
#define   ADAR2001_SPLT34_EN_MSK		BIT(0)
#define ADAR2001_REG_MULT_EN_SPI		0x047
#define   ADAR2001_RF_AMP_EN_MSK		BIT(6)
#define   ADAR2001_MULT_LOW_MSK			GENMASK(5, 4)
#define   ADAR2001_MULT_MID_MSK			GENMASK(3, 2)
#define   ADAR2001_MULT_HIGH_MSK		GENMASK(1, 0)
#define ADAR2001_REG_MULT_PASS_SPI		0x048
#define   ADAR2001_BPF_MSK			BIT(7)
#define   ADAR2001_PA_NOTCH_MSK			BIT(6)
#define   ADAR2001_ATTN_MSK			GENMASK(4, 0)
#define ADAR2001_REG_DET_ENABLES		0x049
#define ADAR2001_REG_ADC_CTRL			0x04A
#define   ADAR2001_ADC_CLKFREQ_SEL_MSK		BIT(7)
#define   ADAR2001_ADC_EN_MSK			BIT(6)
#define   ADAR2001_ADC_CLK_EN_MSK		BIT(5)
#define   ADAR2001_ADC_ST_CONV_MSK		BIT(4)
#define   ADAR2001_ADC_MUX_SEL_MSK		GENMASK(3, 1)
#define   ADAR2001_ADC_MUX_TEMP			0
#define   ADAR2001_ADC_MUX_DETECTOR(_ch)	((_ch) + 1)
#define ADAR2001_REG_ADC_OUTPUT			0x04B
#define ADAR2001_REG_TX_CURR_MODE		0x04C
#define   ADAR2001_TX_CURR_MODE_MSK		GENMASK(3, 0)
#define ADAR2001_REG_TX_CURR_STATE		0x04D
#define   ADAR2001_TX_CURR_STATE_MSK		GENMASK(6, 0)
#define ADAR2001_REG_MULT_STATUS		0x04E
#define   ADAR2001_MULT_CURR_STATE_MSK		GENMASK(7, 4)
#define   ADAR2001_MULT_CURR_MODE_MSK		GENMASK(3, 0)
#define ADAR2001_REG_TX_MODE			0x050
#define ADAR2001_REG_MULT_MODE			0x070
/* Even mode bytes pack one ready/active pair per block, low bit first. */
#define   ADAR2001_PAIR_LSB_MSK			0x55
#define ADAR2001_REG_SCAN_MODE_EN		0x100

#define ADAR2001_NUM_CHANNELS			4
#define ADAR2001_NUM_MULT_BLOCKS		3
#define ADAR2001_NUM_MODES			16
#define ADAR2001_TX_STATES_MAX			70
#define ADAR2001_MULT_STATES_MAX		16
#define ADAR2001_MODE_MAX			15

#define ADAR2001_MODE_TABLE_SIZE		(ADAR2001_NUM_MODES * 2)

#define ADAR2001_SCRATCH_PAD_TEST		0xAD
#define ADAR2001_RESET_DELAY_US			1000
#define ADAR2001_ADC_CONV_US			64

/* Datasheet minimum is 3ns high and 10ns between consecutive pulse starts. */
#define ADAR2001_SEQ_PULSE_NS			10

#define ADAR2001_ADC_CLK_250KHZ			250000
#define ADAR2001_ADC_CLK_2MHZ			2000000

/*
 * Datasheet gives TA = 1.16 * ADC_OUTPUT - 77 degrees C, but IIO computes
 * (raw + offset) * scale, so the offset is -77000/1160 = -1925/29.
 */
#define ADAR2001_TEMP_SCALE			1160
#define ADAR2001_TEMP_OFFSET_NUM		(-1925)
#define ADAR2001_TEMP_OFFSET_DEN		29

/*
 * iio_dev_attr.address packs [7:0] field mask, [23:8] register, [31:24] flags,
 * [39:32] largest accepted user value (0 = derive it from the field mask).
 */
#define ADAR2001_ATTR(_reg, _msk)	((((u64)(_reg)) << 8) | (_msk))
#define ADAR2001_ATTR_MAX(_max)		(((u64)(_max)) << 32)
#define ADAR2001_ATTR_GET_MSK(_a)	((u8)((_a) & GENMASK(7, 0)))
#define ADAR2001_ATTR_GET_REG(_a)	((unsigned int)(((_a) >> 8) & GENMASK(15, 0)))
#define ADAR2001_ATTR_GET_MAX(_a)	((unsigned int)(((_a) >> 32) & GENMASK(7, 0)))

/* The user value is the logical inverse of the field. */
#define ADAR2001_ATTR_INVERT		BIT_ULL(24)
/* The user value is the field plus one (zero-indexed hardware counters). */
#define ADAR2001_ATTR_PLUS1		BIT_ULL(25)

/* Non-constant mask variants of FIELD_PREP() and FIELD_GET() */
#define field_prep(_mask, _val)	(((_val) << (ffs(_mask) - 1)) & (_mask))
#define field_get(_mask, _val)	(((_val) & (_mask)) >> (ffs(_mask) - 1))

enum adar2001_block_state {
	ADAR2001_STATE_SLEEP,
	ADAR2001_STATE_READY,
	ADAR2001_STATE_ACTIVE,
};

static const u8 adar2001_block_state_field[] = {
	[ADAR2001_STATE_SLEEP]	= 0x0,
	[ADAR2001_STATE_READY]	= 0x2,
	[ADAR2001_STATE_ACTIVE]	= 0x3,
};

static const char * const adar2001_block_state_items[] = {
	[ADAR2001_STATE_SLEEP]	= "sleep",
	[ADAR2001_STATE_READY]	= "ready",
	[ADAR2001_STATE_ACTIVE]	= "active",
};

static const char * const adar2001_bpf_items[] = {
	"high",
	"low",
};

struct adar2001_state {
	struct regmap		*regmap;
	struct gpio_desc	*tx_adv_gpio;
	struct gpio_desc	*tx_rst_gpio;
	struct gpio_desc	*mult_adv_gpio;
	struct gpio_desc	*mult_rst_gpio;
	/* Serialises read-modify-write and ADC conversion sequences. */
	struct mutex		lock;
};

static int adar2001_read_field(struct adar2001_state *st, u64 address,
			       unsigned int *val)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, ADAR2001_ATTR_GET_REG(address), &regval);
	if (ret)
		return ret;

	*val = field_get(ADAR2001_ATTR_GET_MSK(address), regval);

	return 0;
}

static int adar2001_write_field(struct adar2001_state *st, u64 address,
				unsigned int val)
{
	u8 mask;

	mask = ADAR2001_ATTR_GET_MSK(address);

	return regmap_update_bits(st->regmap, ADAR2001_ATTR_GET_REG(address),
				  mask, field_prep(mask, val));
}

static ssize_t adar2001_show_field(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int val;
	int ret;

	scoped_guard(mutex, &st->lock) {
		ret = adar2001_read_field(st, address, &val);
		if (ret)
			return ret;
	}

	if (address & ADAR2001_ATTR_INVERT)
		val = !val;

	if (address & ADAR2001_ATTR_PLUS1)
		val += 1;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t adar2001_store_field(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int val, offset, max;
	u8 mask;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	mask = ADAR2001_ATTR_GET_MSK(address);

	max = ADAR2001_ATTR_GET_MAX(address);
	if (!max)
		max = field_get(mask, mask);

	offset = (address & ADAR2001_ATTR_PLUS1) ? 1 : 0;
	if (val < offset || val > max)
		return -EINVAL;

	val -= offset;
	if (address & ADAR2001_ATTR_INVERT)
		val = !val;

	guard(mutex)(&st->lock);

	ret = adar2001_write_field(st, address, val);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_show_block_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int field;
	int i, ret;

	scoped_guard(mutex, &st->lock) {
		ret = adar2001_read_field(st, address, &field);
		if (ret)
			return ret;
	}

	for (i = 0; i < ARRAY_SIZE(adar2001_block_state_field); i++) {
		if (adar2001_block_state_field[i] == field)
			return sysfs_emit(buf, "%s\n",
					  adar2001_block_state_items[i]);
	}

	return -EIO;
}

static ssize_t adar2001_store_block_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	int ret;

	ret = sysfs_match_string(adar2001_block_state_items, buf);
	if (ret < 0)
		return ret;

	guard(mutex)(&st->lock);

	ret = adar2001_write_field(st, address,
				   adar2001_block_state_field[ret]);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_show_bpf(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int val;
	int ret;

	guard(mutex)(&st->lock);

	ret = adar2001_read_field(st, address, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n", adar2001_bpf_items[val]);
}

static ssize_t adar2001_store_bpf(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	int ret;

	ret = sysfs_match_string(adar2001_bpf_items, buf);
	if (ret < 0)
		return ret;

	guard(mutex)(&st->lock);

	ret = adar2001_write_field(st, address, ret);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_store_pulse(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	struct gpio_desc *gpio;
	unsigned int reg;
	bool val;
	u8 mask;
	int ret;

	mask = ADAR2001_ATTR_GET_MSK(address);
	reg = ADAR2001_ATTR_GET_REG(address);

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	if (!val)
		return -EINVAL;

	switch (mask) {
	case ADAR2001_TX_ADV_SPI_MSK:
		gpio = st->tx_adv_gpio;
		break;
	case ADAR2001_TX_RST_SPI_MSK:
		gpio = st->tx_rst_gpio;
		break;
	case ADAR2001_MULT_ADV_SPI_MSK:
		gpio = st->mult_adv_gpio;
		break;
	case ADAR2001_MULT_RST_SPI_MSK:
		gpio = st->mult_rst_gpio;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&st->lock);

	/* The datasheet presents the pins as the primary control path. */
	if (gpio) {
		gpiod_set_value_cansleep(gpio, 1);
		ndelay(ADAR2001_SEQ_PULSE_NS);
		gpiod_set_value_cansleep(gpio, 0);

		return len;
	}

	/*
	 * Documented R/W without stating whether they self-clear, so drive the
	 * edge explicitly.
	 */
	ret = regmap_update_bits(st->regmap, reg, mask, mask);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, reg, mask, 0);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_store_seq_en(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int reg, val;
	bool enable;
	int ret;

	reg = ADAR2001_ATTR_GET_REG(address);

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	/* The sequencer needs the control latch, SPI writes need it bypassed. */
	val = enable ? ADAR2001_SEQ_EN_MSK : ADAR2001_CTL_LATCH_BYP_MSK;

	guard(mutex)(&st->lock);

	ret = regmap_update_bits(st->regmap, reg,
				 ADAR2001_SEQ_EN_MSK |
				 ADAR2001_CTL_LATCH_BYP_MSK, val);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_store_sleep_control(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	u64 address = to_iio_dev_attr(attr)->address;
	unsigned int reg, other, val;
	bool enable;
	int ret;

	reg = ADAR2001_ATTR_GET_REG(address);

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	other = reg == ADAR2001_REG_TX_SEQUENCER_SETUP ?
		ADAR2001_REG_MULT_SEQUENCER_SETUP :
		ADAR2001_REG_TX_SEQUENCER_SETUP;

	guard(mutex)(&st->lock);

	/*
	 * Only one of the two sleep control bits may be set. Reject the write
	 * rather than silently clearing the other one.
	 */
	if (enable) {
		ret = regmap_read(st->regmap, other, &val);
		if (ret)
			return ret;

		if (val & ADAR2001_SLP_CTRL_MSK)
			return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, reg, ADAR2001_SLP_CTRL_MSK,
				 enable ? ADAR2001_SLP_CTRL_MSK : 0);
	if (ret)
		return ret;

	return len;
}

static ssize_t adar2001_show_adc_clock(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, ADAR2001_REG_ADC_CTRL, &val);
	if (ret)
		return ret;

	/* A set bit selects 250 kHz, a clear bit selects 2 MHz. */
	return sysfs_emit(buf, "%u\n", val & ADAR2001_ADC_CLKFREQ_SEL_MSK ?
			  ADAR2001_ADC_CLK_250KHZ : ADAR2001_ADC_CLK_2MHZ);
}

static ssize_t adar2001_store_adc_clock(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar2001_state *st = iio_priv(indio_dev);
	unsigned int val, freq;
	int ret;

	ret = kstrtouint(buf, 0, &freq);
	if (ret)
		return ret;

	switch (freq) {
	case ADAR2001_ADC_CLK_250KHZ:
		val = ADAR2001_ADC_CLKFREQ_SEL_MSK;
		break;
	case ADAR2001_ADC_CLK_2MHZ:
		val = 0;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&st->lock);

	ret = regmap_update_bits(st->regmap, ADAR2001_REG_ADC_CTRL,
				 ADAR2001_ADC_CLKFREQ_SEL_MSK, val);
	if (ret)
		return ret;

	return len;
}

/*
 * One 4-bit mode index per sequencer state, offset 0 being state 1 (state 0 is
 * hardwired to mode 0). Even offsets are the high nibble, odd ones the low.
 */
static ssize_t adar2001_table_read(struct adar2001_state *st, unsigned int base,
				   char *buf, loff_t pos, size_t count)
{
	unsigned int regval;
	size_t i;
	int ret;

	guard(mutex)(&st->lock);

	for (i = 0; i < count; i++) {
		size_t off = pos + i;

		ret = regmap_read(st->regmap, base + off / 2, &regval);
		if (ret)
			return ret;

		buf[i] = off % 2 ? regval & GENMASK(3, 0) : regval >> 4;
	}

	return count;
}

static ssize_t adar2001_table_write(struct adar2001_state *st,
				    unsigned int base, const char *buf,
				    loff_t pos, size_t count)
{
	const u8 *in = buf;
	size_t i;
	int ret;

	/* Validate the whole buffer before any register is touched. */
	for (i = 0; i < count; i++) {
		if (in[i] > ADAR2001_MODE_MAX)
			return -EINVAL;
	}

	guard(mutex)(&st->lock);

	for (i = 0; i < count; i++) {
		size_t off = pos + i;
		u8 mask;

		mask = off % 2 ? GENMASK(3, 0) : GENMASK(7, 4);

		ret = regmap_update_bits(st->regmap, base + off / 2, mask,
					 field_prep(mask, in[i]));
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t tx_sequencer_table_read(struct file *filp,
				       struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_table_read(iio_priv(indio_dev), ADAR2001_REG_TX_STATES,
				   buf, pos, count);
}

static ssize_t tx_sequencer_table_write(struct file *filp,
					struct kobject *kobj,
					struct bin_attribute *bin_attr,
					char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_table_write(iio_priv(indio_dev), ADAR2001_REG_TX_STATES,
				    buf, pos, count);
}

static ssize_t multiplier_sequencer_table_read(struct file *filp,
					       struct kobject *kobj,
					       struct bin_attribute *bin_attr,
					       char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_table_read(iio_priv(indio_dev),
				   ADAR2001_REG_MULT_STATES, buf, pos, count);
}

static ssize_t multiplier_sequencer_table_write(struct file *filp,
						struct kobject *kobj,
						struct bin_attribute *bin_attr,
						char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_table_write(iio_priv(indio_dev),
				    ADAR2001_REG_MULT_STATES, buf, pos, count);
}

/* Plain register image, two bytes per mode. */
static ssize_t adar2001_mode_table_read(struct adar2001_state *st,
					unsigned int base, char *buf,
					loff_t pos, size_t count)
{
	unsigned int regval;
	size_t i;
	int ret;

	guard(mutex)(&st->lock);

	for (i = 0; i < count; i++) {
		ret = regmap_read(st->regmap, base + pos + i, &regval);
		if (ret)
			return ret;

		buf[i] = regval;
	}

	return count;
}

static ssize_t adar2001_mode_table_write(struct adar2001_state *st,
					 unsigned int base, unsigned int pairs,
					 const char *buf, loff_t pos,
					 size_t count)
{
	const u8 *in = buf;
	u8 lsb_msk;
	size_t i;
	int ret;

	lsb_msk = ADAR2001_PAIR_LSB_MSK & GENMASK(2 * pairs - 1, 0);

	/*
	 * Only even offsets hold state pairs, and 0b01 is undocumented in any
	 * of them. Reject it before touching a register.
	 */
	for (i = pos % 2; i < count; i += 2) {
		if (in[i] & ~(in[i] >> 1) & lsb_msk)
			return -EINVAL;
	}

	guard(mutex)(&st->lock);

	for (i = 0; i < count; i++) {
		ret = regmap_write(st->regmap, base + pos + i, in[i]);
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t tx_mode_table_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr, char *buf,
				  loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_mode_table_read(iio_priv(indio_dev),
					ADAR2001_REG_TX_MODE, buf, pos, count);
}

static ssize_t tx_mode_table_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr, char *buf,
				   loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_mode_table_write(iio_priv(indio_dev),
					 ADAR2001_REG_TX_MODE,
					 ADAR2001_NUM_CHANNELS, buf, pos,
					 count);
}

static ssize_t multiplier_mode_table_read(struct file *filp,
					  struct kobject *kobj,
					  struct bin_attribute *bin_attr,
					  char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_mode_table_read(iio_priv(indio_dev),
					ADAR2001_REG_MULT_MODE, buf, pos,
					count);
}

static ssize_t multiplier_mode_table_write(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *bin_attr,
					   char *buf, loff_t pos, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));

	return adar2001_mode_table_write(iio_priv(indio_dev),
					 ADAR2001_REG_MULT_MODE,
					 ADAR2001_NUM_MULT_BLOCKS, buf, pos,
					 count);
}

static BIN_ATTR_RW(tx_sequencer_table, ADAR2001_TX_STATES_MAX);
static BIN_ATTR_RW(multiplier_sequencer_table, ADAR2001_MULT_STATES_MAX);
static BIN_ATTR_RW(tx_mode_table, ADAR2001_MODE_TABLE_SIZE);
static BIN_ATTR_RW(multiplier_mode_table, ADAR2001_MODE_TABLE_SIZE);

static struct bin_attribute *adar2001_bin_attributes[] = {
	&bin_attr_tx_sequencer_table,
	&bin_attr_multiplier_sequencer_table,
	&bin_attr_tx_mode_table,
	&bin_attr_multiplier_mode_table,
	NULL
};

static int adar2001_pa_state_get(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adar2001_state *st = iio_priv(indio_dev);
	unsigned int field;
	int i, ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, ADAR2001_REG_TX_EN1_SPI, &field);
	if (ret)
		return ret;

	field = field_get(ADAR2001_PA_STATE_MSK(chan->channel), field);

	for (i = 0; i < ARRAY_SIZE(adar2001_block_state_field); i++) {
		if (adar2001_block_state_field[i] == field)
			return i;
	}

	return -EIO;
}

static int adar2001_pa_state_set(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct adar2001_state *st = iio_priv(indio_dev);
	u8 mask, val;

	mask = ADAR2001_PA_STATE_MSK(chan->channel);
	val = adar2001_block_state_field[mode];

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, ADAR2001_REG_TX_EN1_SPI, mask,
				  field_prep(mask, val));
}

static const struct iio_enum adar2001_pa_state_enum = {
	.items = adar2001_block_state_items,
	.num_items = ARRAY_SIZE(adar2001_block_state_items),
	.get = adar2001_pa_state_get,
	.set = adar2001_pa_state_set,
};

static const struct iio_chan_spec_ext_info adar2001_tx_ext_info[] = {
	IIO_ENUM("pa_state", IIO_SEPARATE, &adar2001_pa_state_enum),
	IIO_ENUM_AVAILABLE("pa_state", IIO_SEPARATE, &adar2001_pa_state_enum),
	{ }
};

static int adar2001_adc_read(struct adar2001_state *st, unsigned int mux,
			     int *val)
{
	unsigned int regval;
	int ret;

	guard(mutex)(&st->lock);

	/*
	 * ST_CONV is not documented as self-clearing, so clear it first to
	 * guarantee the second write produces a rising edge.
	 */
	ret = regmap_update_bits(st->regmap, ADAR2001_REG_ADC_CTRL,
				 ADAR2001_ADC_EN_MSK |
				 ADAR2001_ADC_CLK_EN_MSK |
				 ADAR2001_ADC_ST_CONV_MSK |
				 ADAR2001_ADC_MUX_SEL_MSK,
				 ADAR2001_ADC_EN_MSK |
				 ADAR2001_ADC_CLK_EN_MSK |
				 FIELD_PREP(ADAR2001_ADC_MUX_SEL_MSK, mux));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, ADAR2001_REG_ADC_CTRL,
				 ADAR2001_ADC_ST_CONV_MSK,
				 ADAR2001_ADC_ST_CONV_MSK);
	if (ret)
		return ret;

	fsleep(ADAR2001_ADC_CONV_US);

	ret = regmap_read(st->regmap, ADAR2001_REG_ADC_OUTPUT, &regval);
	if (ret)
		return ret;

	*val = regval;

	return 0;
}

static int adar2001_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct adar2001_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_TEMP:
			ret = adar2001_adc_read(st, ADAR2001_ADC_MUX_TEMP, val);
			break;
		case IIO_POWER:
			ret = adar2001_adc_read(st,
						ADAR2001_ADC_MUX_DETECTOR(chan->channel),
						val);
			break;
		default:
			return -EINVAL;
		}
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = ADAR2001_TEMP_SCALE;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = ADAR2001_TEMP_OFFSET_NUM;
		*val2 = ADAR2001_TEMP_OFFSET_DEN;

		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_ENABLE:
		scoped_guard(mutex, &st->lock) {
			ret = regmap_read(st->regmap, ADAR2001_REG_DET_ENABLES,
					  &regval);
			if (ret)
				return ret;
		}

		*val = !!(regval & BIT(chan->channel));

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adar2001_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct adar2001_state *st = iio_priv(indio_dev);

	if (mask != IIO_CHAN_INFO_ENABLE)
		return -EINVAL;

	if (val < 0 || val > 1)
		return -EINVAL;

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, ADAR2001_REG_DET_ENABLES,
				  BIT(chan->channel),
				  val ? BIT(chan->channel) : 0);
}

static int adar2001_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			       unsigned int writeval, unsigned int *readval)
{
	struct adar2001_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

#define ADAR2001_POWER_CHANNEL(_ch) {					\
	.type = IIO_POWER,						\
	.indexed = 1,							\
	.channel = (_ch),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_ENABLE),		\
}

#define ADAR2001_TX_CHANNEL(_ch) {					\
	.type = IIO_ALTVOLTAGE,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (_ch),						\
	.ext_info = adar2001_tx_ext_info,				\
}

static const struct iio_chan_spec adar2001_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
	},
	ADAR2001_POWER_CHANNEL(0),
	ADAR2001_POWER_CHANNEL(1),
	ADAR2001_POWER_CHANNEL(2),
	ADAR2001_POWER_CHANNEL(3),
	ADAR2001_TX_CHANNEL(0),
	ADAR2001_TX_CHANNEL(1),
	ADAR2001_TX_CHANNEL(2),
	ADAR2001_TX_CHANNEL(3),
};

/* Power and bias */
static IIO_DEVICE_ATTR(powerdown, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_PWRON, ADAR2001_PWRON_MSK) |
		       ADAR2001_ATTR_INVERT);
static IIO_DEVICE_ATTR(multiplier_low_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_MULT1,
				     ADAR2001_MULT_LOW_BIAS_MSK));
static IIO_DEVICE_ATTR(multiplier_mid_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_MULT1,
				     ADAR2001_MULT_MID_BIAS_MSK));
static IIO_DEVICE_ATTR(multiplier_high_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_MULT2,
				     ADAR2001_MULT_HIGH_BIAS_MSK));
static IIO_DEVICE_ATTR(rf_amp1_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_RF_AMP,
				     ADAR2001_RF_AMP1_BIAS_MSK));
static IIO_DEVICE_ATTR(rf_amp2_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_RF_AMP,
				     ADAR2001_RF_AMP2_BIAS_MSK));
static IIO_DEVICE_ATTR(splitter1_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_SPLT,
				     ADAR2001_SPLT1_BIAS_MSK));
static IIO_DEVICE_ATTR(splitter2_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_SPLT,
				     ADAR2001_SPLT2_BIAS_MSK));
static IIO_DEVICE_ATTR(pa_bias, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_BIAS_CURRENT_PA,
				     ADAR2001_PA_BIAS_MSK));

/* Manual transmit path control */
static IIO_DEVICE_ATTR(splitter1_en, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_TX_EN2_SPI,
				     ADAR2001_SPLT1_EN_MSK));
static IIO_DEVICE_ATTR(splitter12_en, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_TX_EN2_SPI,
				     ADAR2001_SPLT12_EN_MSK));
static IIO_DEVICE_ATTR(splitter34_en, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_TX_EN2_SPI,
				     ADAR2001_SPLT34_EN_MSK));

/* Manual multiplier and filter path control */
static IIO_DEVICE_ATTR(rf_amp_en, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_EN_SPI,
				     ADAR2001_RF_AMP_EN_MSK));
static IIO_DEVICE_ATTR(multiplier_low_state, 0644, adar2001_show_block_state,
		       adar2001_store_block_state,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_EN_SPI,
				     ADAR2001_MULT_LOW_MSK));
static IIO_CONST_ATTR(multiplier_low_state_available, "sleep ready active");
static IIO_DEVICE_ATTR(multiplier_mid_state, 0644, adar2001_show_block_state,
		       adar2001_store_block_state,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_EN_SPI,
				     ADAR2001_MULT_MID_MSK));
static IIO_CONST_ATTR(multiplier_mid_state_available, "sleep ready active");
static IIO_DEVICE_ATTR(multiplier_high_state, 0644, adar2001_show_block_state,
		       adar2001_store_block_state,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_EN_SPI,
				     ADAR2001_MULT_HIGH_MSK));
static IIO_CONST_ATTR(multiplier_high_state_available, "sleep ready active");
static IIO_DEVICE_ATTR(bandpass_filter, 0644, adar2001_show_bpf,
		       adar2001_store_bpf,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_PASS_SPI,
				     ADAR2001_BPF_MSK));
static IIO_CONST_ATTR(bandpass_filter_available, "high low");
static IIO_DEVICE_ATTR(pa_notch_en, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_PASS_SPI,
				     ADAR2001_PA_NOTCH_MSK));
static IIO_DEVICE_ATTR(attenuation, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_PASS_SPI,
				     ADAR2001_ATTN_MSK));

/* ADC */
static IIO_DEVICE_ATTR(adc_clock_frequency, 0644, adar2001_show_adc_clock,
		       adar2001_store_adc_clock, 0);
static IIO_CONST_ATTR(adc_clock_frequency_available, "250000 2000000");

/* Sequencer control */
static IIO_DEVICE_ATTR(tx_sequencer_en, 0644, adar2001_show_field,
		       adar2001_store_seq_en,
		       ADAR2001_ATTR(ADAR2001_REG_TX_SEQUENCER_SETUP,
				     ADAR2001_SEQ_EN_MSK));
static IIO_DEVICE_ATTR(multiplier_sequencer_en, 0644, adar2001_show_field,
		       adar2001_store_seq_en,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_SEQUENCER_SETUP,
				     ADAR2001_SEQ_EN_MSK));
static IIO_DEVICE_ATTR(tx_sequencer_depth, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_TX_SEQUENCER_SETUP2,
				     ADAR2001_TX_STATES_MSK) |
		       ADAR2001_ATTR_PLUS1 |
		       ADAR2001_ATTR_MAX(ADAR2001_TX_STATES_MAX));
static IIO_DEVICE_ATTR(multiplier_sequencer_depth, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_SEQUENCER_SETUP,
				     ADAR2001_MULT_STATES_MSK) |
		       ADAR2001_ATTR_PLUS1 |
		       ADAR2001_ATTR_MAX(ADAR2001_MULT_STATES_MAX));
static IIO_DEVICE_ATTR(tx_sleep_control, 0644, adar2001_show_field,
		       adar2001_store_sleep_control,
		       ADAR2001_ATTR(ADAR2001_REG_TX_SEQUENCER_SETUP,
				     ADAR2001_SLP_CTRL_MSK));
static IIO_DEVICE_ATTR(multiplier_sleep_control, 0644, adar2001_show_field,
		       adar2001_store_sleep_control,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_SEQUENCER_SETUP,
				     ADAR2001_SLP_CTRL_MSK));
static IIO_DEVICE_ATTR(tx_sleep_hold, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_TX_SEQUENCER_SETUP,
				     ADAR2001_SLP_HOLD_MSK));
static IIO_DEVICE_ATTR(multiplier_sleep_hold, 0644, adar2001_show_field,
		       adar2001_store_field,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_SEQUENCER_SETUP,
				     ADAR2001_SLP_HOLD_MSK));
static IIO_DEVICE_ATTR(tx_sequencer_advance, 0200, NULL, adar2001_store_pulse,
		       ADAR2001_ATTR(ADAR2001_REG_SEQUENCER_CTRL_SPI,
				     ADAR2001_TX_ADV_SPI_MSK));
static IIO_DEVICE_ATTR(tx_sequencer_reset, 0200, NULL, adar2001_store_pulse,
		       ADAR2001_ATTR(ADAR2001_REG_SEQUENCER_CTRL_SPI,
				     ADAR2001_TX_RST_SPI_MSK));
static IIO_DEVICE_ATTR(multiplier_sequencer_advance, 0200, NULL,
		       adar2001_store_pulse,
		       ADAR2001_ATTR(ADAR2001_REG_SEQUENCER_CTRL_SPI,
				     ADAR2001_MULT_ADV_SPI_MSK));
static IIO_DEVICE_ATTR(multiplier_sequencer_reset, 0200, NULL,
		       adar2001_store_pulse,
		       ADAR2001_ATTR(ADAR2001_REG_SEQUENCER_CTRL_SPI,
				     ADAR2001_MULT_RST_SPI_MSK));

/* Sequencer status */
static IIO_DEVICE_ATTR(tx_current_mode, 0444, adar2001_show_field, NULL,
		       ADAR2001_ATTR(ADAR2001_REG_TX_CURR_MODE,
				     ADAR2001_TX_CURR_MODE_MSK));
static IIO_DEVICE_ATTR(tx_current_state, 0444, adar2001_show_field, NULL,
		       ADAR2001_ATTR(ADAR2001_REG_TX_CURR_STATE,
				     ADAR2001_TX_CURR_STATE_MSK));
static IIO_DEVICE_ATTR(multiplier_current_mode, 0444, adar2001_show_field,
		       NULL,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_STATUS,
				     ADAR2001_MULT_CURR_MODE_MSK));
static IIO_DEVICE_ATTR(multiplier_current_state, 0444, adar2001_show_field,
		       NULL,
		       ADAR2001_ATTR(ADAR2001_REG_MULT_STATUS,
				     ADAR2001_MULT_CURR_STATE_MSK));

static struct attribute *adar2001_attributes[] = {
	&iio_dev_attr_powerdown.dev_attr.attr,
	&iio_dev_attr_multiplier_low_bias.dev_attr.attr,
	&iio_dev_attr_multiplier_mid_bias.dev_attr.attr,
	&iio_dev_attr_multiplier_high_bias.dev_attr.attr,
	&iio_dev_attr_rf_amp1_bias.dev_attr.attr,
	&iio_dev_attr_rf_amp2_bias.dev_attr.attr,
	&iio_dev_attr_splitter1_bias.dev_attr.attr,
	&iio_dev_attr_splitter2_bias.dev_attr.attr,
	&iio_dev_attr_pa_bias.dev_attr.attr,
	&iio_dev_attr_splitter1_en.dev_attr.attr,
	&iio_dev_attr_splitter12_en.dev_attr.attr,
	&iio_dev_attr_splitter34_en.dev_attr.attr,
	&iio_dev_attr_rf_amp_en.dev_attr.attr,
	&iio_dev_attr_multiplier_low_state.dev_attr.attr,
	&iio_const_attr_multiplier_low_state_available.dev_attr.attr,
	&iio_dev_attr_multiplier_mid_state.dev_attr.attr,
	&iio_const_attr_multiplier_mid_state_available.dev_attr.attr,
	&iio_dev_attr_multiplier_high_state.dev_attr.attr,
	&iio_const_attr_multiplier_high_state_available.dev_attr.attr,
	&iio_dev_attr_bandpass_filter.dev_attr.attr,
	&iio_const_attr_bandpass_filter_available.dev_attr.attr,
	&iio_dev_attr_pa_notch_en.dev_attr.attr,
	&iio_dev_attr_attenuation.dev_attr.attr,
	&iio_dev_attr_adc_clock_frequency.dev_attr.attr,
	&iio_const_attr_adc_clock_frequency_available.dev_attr.attr,
	&iio_dev_attr_tx_sequencer_en.dev_attr.attr,
	&iio_dev_attr_multiplier_sequencer_en.dev_attr.attr,
	&iio_dev_attr_tx_sequencer_depth.dev_attr.attr,
	&iio_dev_attr_multiplier_sequencer_depth.dev_attr.attr,
	&iio_dev_attr_tx_sleep_control.dev_attr.attr,
	&iio_dev_attr_multiplier_sleep_control.dev_attr.attr,
	&iio_dev_attr_tx_sleep_hold.dev_attr.attr,
	&iio_dev_attr_multiplier_sleep_hold.dev_attr.attr,
	&iio_dev_attr_tx_sequencer_advance.dev_attr.attr,
	&iio_dev_attr_tx_sequencer_reset.dev_attr.attr,
	&iio_dev_attr_multiplier_sequencer_advance.dev_attr.attr,
	&iio_dev_attr_multiplier_sequencer_reset.dev_attr.attr,
	&iio_dev_attr_tx_current_mode.dev_attr.attr,
	&iio_dev_attr_tx_current_state.dev_attr.attr,
	&iio_dev_attr_multiplier_current_mode.dev_attr.attr,
	&iio_dev_attr_multiplier_current_state.dev_attr.attr,
	NULL
};

static const struct attribute_group adar2001_attribute_group = {
	.attrs = adar2001_attributes,
	.bin_attrs = adar2001_bin_attributes,
};

static const struct iio_info adar2001_info = {
	.read_raw = adar2001_read_raw,
	.write_raw = adar2001_write_raw,
	.debugfs_reg_access = adar2001_reg_access,
	.attrs = &adar2001_attribute_group,
};

static const struct regmap_config adar2001_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = ADAR2001_REG_SCAN_MODE_EN,
};

static int adar2001_request_seq_gpios(struct device *dev,
				      struct adar2001_state *st)
{
	st->tx_adv_gpio = devm_gpiod_get_optional(dev, "tx-advance",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->tx_adv_gpio))
		return dev_err_probe(dev, PTR_ERR(st->tx_adv_gpio),
				     "failed to get tx-advance GPIO\n");

	st->tx_rst_gpio = devm_gpiod_get_optional(dev, "tx-reset",
						  GPIOD_OUT_LOW);
	if (IS_ERR(st->tx_rst_gpio))
		return dev_err_probe(dev, PTR_ERR(st->tx_rst_gpio),
				     "failed to get tx-reset GPIO\n");

	st->mult_adv_gpio = devm_gpiod_get_optional(dev, "multiplier-advance",
						    GPIOD_OUT_LOW);
	if (IS_ERR(st->mult_adv_gpio))
		return dev_err_probe(dev, PTR_ERR(st->mult_adv_gpio),
				     "failed to get multiplier-advance GPIO\n");

	st->mult_rst_gpio = devm_gpiod_get_optional(dev, "multiplier-reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(st->mult_rst_gpio))
		return dev_err_probe(dev, PTR_ERR(st->mult_rst_gpio),
				     "failed to get multiplier-reset GPIO\n");

	return 0;
}

static int adar2001_setup(struct iio_dev *indio_dev)
{
	struct adar2001_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	unsigned int val;
	int ret;

	/*
	 * The part has no reset pin, so reset over SPI. This writes the whole
	 * register, so SDOACTIVE has to be set here or readback stops working.
	 */
	ret = regmap_write(st->regmap, ADAR2001_REG_INTERFACE_CONFIG_A,
			   ADAR2001_SOFTRESET | ADAR2001_SDOACTIVE);
	if (ret)
		return ret;

	fsleep(ADAR2001_RESET_DELAY_US);

	ret = regmap_write(st->regmap, ADAR2001_REG_SCRATCH_PAD,
			   ADAR2001_SCRATCH_PAD_TEST);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADAR2001_REG_SCRATCH_PAD, &val);
	if (ret)
		return ret;

	if (val != ADAR2001_SCRATCH_PAD_TEST)
		return dev_err_probe(dev, -ENODEV,
				     "scratch pad readback failed, wrote 0x%02x read 0x%02x\n",
				     ADAR2001_SCRATCH_PAD_TEST, val);

	return 0;
}

static int adar2001_probe(struct spi_device *spi)
{
	struct adar2001_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init_spi(spi, &adar2001_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "failed to initialise regmap\n");

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable_optional(&spi->dev, "vpos");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(&spi->dev, ret,
				     "failed to enable vpos supply\n");

	/* Optional: the reference wiring straps VPOS2 to the on-chip VREG. */
	ret = devm_regulator_get_enable_optional(&spi->dev, "vpos2");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(&spi->dev, ret,
				     "failed to enable vpos2 supply\n");

	ret = adar2001_request_seq_gpios(&spi->dev, st);
	if (ret)
		return ret;

	indio_dev->name = "adar2001";
	indio_dev->info = &adar2001_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adar2001_channels;
	indio_dev->num_channels = ARRAY_SIZE(adar2001_channels);

	ret = adar2001_setup(indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adar2001_id[] = {
	{ .name = "adar2001" },
	{ }
};
MODULE_DEVICE_TABLE(spi, adar2001_id);

static const struct of_device_id adar2001_of_match[] = {
	{ .compatible = "adi,adar2001" },
	{ }
};
MODULE_DEVICE_TABLE(of, adar2001_of_match);

static struct spi_driver adar2001_driver = {
	.driver = {
		.name = "adar2001",
		.of_match_table = adar2001_of_match,
	},
	.probe = adar2001_probe,
	.id_table = adar2001_id,
};
module_spi_driver(adar2001_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR2001 frequency multiplier/filter");
MODULE_LICENSE("GPL");
