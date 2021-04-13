/* SPDX-License-Identifier: GPL-2.0+
 *
 * ADPD188 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/timekeeping.h>

#include "adpd188.h"

#define ADPD188_REG_STATUS			0x00
#define ADPD188_REG_INT_MASK			0x01
#define ADPD188_REG_GPIO_DRV			0x02
#define ADPD188_REG_FIFO_THRESH			0x06
#define ADPD188_REG_DEVID			0x08
#define ADPD188_REG_GPIO_CTL			0x0B
#define ADPD188_REG_SW_RESET			0x0F
#define ADPD188_REG_MODE			0x10
#define ADPD188_REG_SLOT_EN			0x11
#define ADPD188_REG_FSAMPLE			0x12
#define ADPD188_REG_PD_LED_SELECT		0x14
#define ADPD188_REG_NUM_AVG			0x15
#define ADPD188_REG_INT_SEQ_A			0x17
#define ADPD188_REG_SLOTA_CH1_OFFSET		0x18
#define ADPD188_REG_SLOTA_CH2_OFFSET		0x19
#define ADPD188_REG_SLOTA_CH3_OFFSET		0x1A
#define ADPD188_REG_SLOTA_CH4_OFFSET		0x1B
#define ADPD188_REG_INT_SEQ_B			0x1D
#define ADPD188_REG_SLOTB_CH1_OFFSET		0x1E
#define ADPD188_REG_SLOTB_CH2_OFFSET		0x1F
#define ADPD188_REG_SLOTB_CH3_OFFSET		0x20
#define ADPD188_REG_SLOTB_CH4_OFFSET		0x21

enum adpd188_iled_coarse_index {
	LED3,
	LED1,
	LED2
};
#define ADPD188_REG_ILEDx_COARSE(x)		(0x22 + (x))

enum adpd188_slots {
	ADPD188_SLOTA,
	ADPD188_SLOTB
};
#define ADPD188_REG_SLOTx_NUMPULSES(x)		(0x31 + (x) * 5)
#define ADPD188_REG_SLOTx_AFE_WINDOW(x)		(0x39 + (x) * 2)

#define ADPD188_REG_MATH			0x58
#define ADPD188_REG_AFE_PWR_CFG1		0x3C
#define ADPD188_REG_SAMPLE_CLK			0x4B
#define ADPD188_REG_FIFO_DATA			0x60
#define ADPD188_REG_16BIT_DATA_BASE		0x64
#define ADPD188_REG_32BIT_DATA_LOW_BASE		0x70
#define ADPD188_REG_32BIT_DATA_HIGH_BASE	0x74
#define ADPD188_REG_16BIT_DATA(_slot, _ch)	\
	(ADPD188_REG_16BIT_DATA_BASE + (_ch) + (_slot) * 4)
#define ADPD188_REG_32BIT_DATA_LOW(_slot, _ch)	\
	(ADPD188_REG_32BIT_DATA_LOW_BASE + (_ch) + (_slot) * 8)
#define ADPD188_REG_32BIT_DATA_HIGH(_slot, _ch)	\
	(ADPD188_REG_32BIT_DATA_HIGH_BASE + (_ch) + (_slot) * 8)

/** ADPD188_REG_STATUS */
#define ADPD188_FIFO_CLEAR			BIT(15)
#define ADPD188_FIFO_SAMPLES			GENMASK(15, 8)
#define ADPD188_SLOTB_INT			BIT(6)
#define ADPD188_SLOTA_INT			BIT(5)

/** ADPD188_REG_INT_MASK */
#define ADPD188_FIFO_INT_MASK			BIT(8)
#define ADPD188_SLOTB_INT_MASK			BIT(6)
#define ADPD188_SLOTA_INT_MASK			BIT(5)

/** ADPD188_REG_GPIO_DRV */
#define ADPD188_GPIO1_DRV			BIT(9)
#define ADPD188_GPIO1_POL			BIT(8)
#define ADPD188_GPIO0_ENA			BIT(2)
#define ADPD188_GPIO0_DRV			BIT(1)
#define ADPD188_GPIO0_POL			BIT(0)

/** ADPD188_REG_FIFO_THRESH */
#define ADPD188_FIFO_THRESH			GENMASK(13, 8)

/** ADPD188_REG_GPIO_CTL */
#define ADPD188_GPIO1_ALT_CONFIG		GENMASK(12, 8)
#define ADPD188_GPIO0_ALT_CONFIG		GENMASK(4, 0)

/** ADPD188_REG_DEVID */
#define ADPD188_DEVID_MASK				GENMASK(7, 0)

/** ADPD188_REG_MODE */
#define ADPD188_MODE_MASK			GENMASK(1, 0)

/** ADPD188_REG_SLOT_EN */
#define ADPD188_RDOUT_MODE_MASK			BIT(13)
#define ADPD188_FIFO_OVRN_PREVENT_MASK		BIT(12)
#define ADPD188_SLOTB_FIFO_MODE_MASK		GENMASK(8, 6)
#define ADPD188_SLOTB_EN_MASK			BIT(5)
#define ADPD188_SLOTA_FIFO_MODE_MASK		GENMASK(4, 2)
#define ADPD188_SLOTA_EN_MASK			BIT(0)

/** ADPD188_REG_SAMPLE_CLK */
#define ADPD188_CLK32K_BYP			BIT(8)
#define ADPD188_CLK32K_EN			BIT(7)
#define ADPD188_CLK32K_ADJUST			GENMASK(5, 0)

/** ADPD188_REG_PD_LED_SELECT */
#define ADPD188_SLOTB_PD_SEL_MASK		GENMASK(11, 8)
#define ADPD188_SLOTA_PD_SEL_MASK		GENMASK(7, 4)
#define ADPD188_SLOTB_LED_SEL_MASK		GENMASK(3, 2)
#define ADPD188_SLOTA_LED_SEL_MASK		GENMASK(1, 0)

/** ADPD188_REG_NUM_AVG */
#define ADPD188_SLOTB_NUM_AVG_MASK		GENMASK(10, 8)
#define ADPD188_SLOTA_NUM_AVG_MASK		GENMASK(6, 4)

/** ADPD188_REG_INT_SEQ_A */
#define ADPD188_INTEG_ORDER_A_MASK		GENMASK(3, 0)

/* ADPD188_REG_INT_SEQ_B */
#define ADPD188_INTEG_ORDER_B_MASK		GENMASK(3, 0)

/** ADPD188_REG_ILEDx_COARSE */
#define ADPD188_ILEDx_SCALE_MASK		BIT(13)
#define ADPD188_ILEDx_SLEW_MASK			GENMASK(6, 4)
#define ADPD188_ILEDx_COARSE_MASK		GENMASK(3, 0)

/** ADPD188_REG_SLOTx_NUMPULSES */
#define ADPD188_SLOTx_PULSES_MASK		GENMASK(15, 8)
#define ADPD188_SLOTx_PERIOD_MASK		GENMASK(7, 0)

/** ADPD188_REG_SLOTx_AFE_WINDOW */
#define ADPD188_SLOTx_AFE_WIDTH_MASK		GENMASK(15, 11)
#define ADPD188_SLOTx_AFE_OFFSET_MASK		GENMASK(10, 0)

/** ADPD188_REG_AFE_PWR_CFG1 */
#define ADPD188_V_CATHODE_MASK			BIT(9)
#define ADPD188_AFE_POWERDOWN_MASK		GENMASK(8, 3)

/** ADPD188_REG_MATH */
#define ADPD188_FLT_MATH34_B_MASK		GENMASK(11, 10)
#define ADPD188_FLT_MATH34_A_MASK		GENMASK(9, 8)
#define ADPD188_ENA_INT_AS_BUF_MASK		BIT(7)
#define ADPD188_FLT_MATH12_B_MASK		GENMASK(6, 5)
#define ADPD188_FLT_MATH12_A_MASK		GENMASK(2, 1)

#define ADPD188_DEVID		0x16
#define ADPD188_MAX_FSAMPLE	2000

enum adpd188_modes {
	STANDBY,
	PROGRAM,
	NORMAL
};

static char *adpd188_ascii_modes[] = { "STANDBY", "PROGRAM", "NORMAL" };

#define SAMPLE_WORDS	4
struct adpd188 {
	struct regmap *regmap;
	volatile int trigno;
	u64 start_ts, end_ts;
	union {
		u32 word[SAMPLE_WORDS];
		u16 reg_data[SAMPLE_WORDS * 2];
	} data ____cacheline_aligned;
	struct completion value_ok;
};

enum adpd188_slot_fifo_mode {
	ADPD188_NO_FIFO,
	ADPD188_16BIT_SUM,
	ADPD188_32BIT_SUM,
	ADPD188_16BIT_4CHAN = 0x4,
	ADPD188_32BIT_4CHAN = 0x6
};

struct adpd188_slot_config {
	enum adpd188_slots slot_id;
	bool slot_en;
	enum adpd188_slot_fifo_mode slot_fifo_mode;
};

static int adpd188_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int tx_val,
			      unsigned int *rx_val)
{
	struct adpd188 *dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(dev->regmap, reg, rx_val);
	else
		return regmap_write(dev->regmap, reg, tx_val);
}

static int adpd188_reg_write_mask(struct regmap *regmap, int addr,
				  int value, int mask)
{
	int temp_val;
	int ret;

	ret = regmap_read(regmap, addr, &temp_val);
	if (ret < 0)
		return ret;
	temp_val &= ~mask;
	temp_val |= (value << (ffs(mask) - 1)) & mask;

	return regmap_write(regmap, addr, temp_val);
}

static int adpd188_set_device_mode(struct regmap *regmap, int new_mode)
{
	int regval;

	regval = new_mode & ADPD188_MODE_MASK;

	return regmap_write(regmap, ADPD188_REG_MODE, regval);
}

static int adpd188_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adpd188 *data = iio_priv(indio_dev);
	int ret, i;
	int regval, regaddr;
	int data_buf[SAMPLE_WORDS];

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = regmap_read(data->regmap, ADPD188_REG_FSAMPLE, &regval);
		if (ret < 0)
			return ret;
		*val = 32000 / (regval * 4);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->channel == 0)
			regaddr = ADPD188_REG_SLOTA_CH1_OFFSET;
		else if (chan->channel == 1)
			regaddr = ADPD188_REG_SLOTB_CH1_OFFSET;
		else
			return -EINVAL;
		ret = regmap_read(data->regmap, regaddr, &regval);
		if (ret < 0)
			return ret;
		*val = regval;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = adpd188_set_device_mode(data->regmap, PROGRAM);
		if (ret < 0)
			return ret;

		ret = adpd188_reg_write_mask(data->regmap, ADPD188_REG_STATUS,
					1, ADPD188_FIFO_CLEAR);
		if (ret < 0)
			return ret;

		ret = adpd188_set_device_mode(data->regmap, NORMAL);
		if (ret < 0)
			return ret;

		while (true) {
			ret = regmap_read(data->regmap, ADPD188_REG_STATUS, &regval);
			if (ret < 0)
				return ret;
			regval &= ADPD188_FIFO_SAMPLES;
			regval >>= ffs(ADPD188_FIFO_SAMPLES) - 1;
			if (regval >= (SAMPLE_WORDS * 2))
				break;
		}

		for (i = 0; i < SAMPLE_WORDS; i++) {
			ret = regmap_read(data->regmap, ADPD188_REG_FIFO_DATA, data_buf + i);
			if (ret < 0)
				return ret;
		}

		if (chan->channel == 0) {
			*val = data_buf[0];
			*val |= data_buf[1] << 16;
		} else if (chan->channel == 1) {
			*val = data_buf[2];
			*val |= data_buf[3] << 16;
		}

		ret = adpd188_set_device_mode(data->regmap, PROGRAM);
		if (ret < 0)
			return ret;

		ret = adpd188_set_device_mode(data->regmap, STANDBY);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adpd188_set_sample_freq(struct regmap *regmap, int new_freq)
{
	int regval;

	regval = 32000 / (new_freq * 4);

	return regmap_write(regmap, ADPD188_REG_FSAMPLE, regval);
}

static int adpd188_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adpd188 *data = iio_priv(indio_dev);
	int regaddr;
	int ret;

	ret = adpd188_set_device_mode(data->regmap, PROGRAM);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if ((val > ADPD188_MAX_FSAMPLE) || (val < 0)) {
			ret = -EINVAL;
			break;
		}

		ret = adpd188_set_sample_freq(data->regmap, val);
		break;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->channel == 0) {
			regaddr = ADPD188_REG_SLOTA_CH1_OFFSET;
		} else if (chan->channel == 1) {
			regaddr = ADPD188_REG_SLOTB_CH1_OFFSET;
		} else {
			ret = -EINVAL;
			break;
		}

		ret = regmap_write(data->regmap, regaddr, val);
		break;
	}

	adpd188_set_device_mode(data->regmap, STANDBY);

	return ret;
}


ssize_t adpd188_mode_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adpd188 *data = iio_priv(indio_dev);
	int regval;
	int ret;

	ret = regmap_read(data->regmap, ADPD188_REG_MODE, &regval);
	if (ret < 0)
		return ret;
	regval &= ADPD188_MODE_MASK;

	return sprintf(buf, "%s\n", adpd188_ascii_modes[regval]);
}

ssize_t adpd188_mode_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adpd188 *data = iio_priv(indio_dev);
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(adpd188_ascii_modes); i++)
		if (!strncmp(buf, adpd188_ascii_modes[i], strlen(adpd188_ascii_modes[i])))
			break;
	if (i == ARRAY_SIZE(adpd188_ascii_modes))
		return -EINVAL;

	ret = adpd188_set_device_mode(data->regmap, i);
	if (ret < 0)
		return ret;

	/* Always return full write size even if we didn't consume all */
	return size;
}

IIO_CONST_ATTR(mode_available, "STANDBY PROGRAM NORMAL");
IIO_DEVICE_ATTR(mode, 0644, adpd188_mode_show, adpd188_mode_store, ADPD188_REG_MODE);

struct attribute *adpd188_attrs[] = {
	&iio_const_attr_mode_available.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	NULL
};

#define ADPD188_32BIT_CHANNEL(index, color, label) {                                  \
	.type = IIO_LIGHT,                                              \
	.modified = 1,                                                  \
	.channel2 = IIO_MOD_LIGHT_##color,                                     \
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 32,						\
		.storagebits = 32,					\
		.endianness = IIO_LE,					\
	},								\
	.channel = index,                                               \
	.scan_index = index,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_OFFSET),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),                           \
	.label_name = label						\
}

static const struct iio_chan_spec adpd188_channels[] = {
	ADPD188_32BIT_CHANNEL(0, BLUE, "BLUE_32_BIT"),
	ADPD188_32BIT_CHANNEL(1, IR, "IR_32_BIT"),
	IIO_CHAN_SOFT_TIMESTAMP(2)
};

static const struct attribute_group adpd188_attrs_group = {
	.attrs = adpd188_attrs,
};

static const struct iio_info adpd188_info = {
	.attrs		= &adpd188_attrs_group,
	.read_raw       = adpd188_read_raw,
	.write_raw      = adpd188_write_raw,
	.debugfs_reg_access = &adpd188_reg_access,
};

static int adpd188_slot_setup(struct regmap *regmap, struct adpd188_slot_config config)
{
	int ret;

	if(config.slot_id == ADPD188_SLOTA) {
		ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN,
					     config.slot_en ? 1 : 0,
					     ADPD188_SLOTA_EN_MASK);
		if (ret < 0)
			return ret;
		ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN,
					     config.slot_fifo_mode,
					     ADPD188_SLOTA_FIFO_MODE_MASK);
	} else if (config.slot_id == ADPD188_SLOTB) {
		ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN,
					     config.slot_en ? 1 : 0,
					     ADPD188_SLOTB_EN_MASK);
		if (ret < 0)
			return ret;
		ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN,
					     config.slot_fifo_mode,
					     ADPD188_SLOTB_FIFO_MODE_MASK);
	}

	return ret;
}

#define DEFAULT_SAMPLE_FREQUENCY	16
static int adpd188_core_smoke_setup(struct regmap *regmap)
{
	int ret;
	struct adpd188_slot_config ts_config;

	ret = adpd188_set_device_mode(regmap, PROGRAM);
	if (ret < 0)
		return ret;

	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN, 1,
				     ADPD188_RDOUT_MODE_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOT_EN, 1,
				     ADPD188_FIFO_OVRN_PREVENT_MASK);
	if (ret < 0)
		return ret;
	ts_config.slot_id = ADPD188_SLOTA;
	ts_config.slot_en = true;
	ts_config.slot_fifo_mode = ADPD188_32BIT_SUM;
	ret = adpd188_slot_setup(regmap, ts_config);
	if (ret < 0)
		return ret;
	ts_config.slot_id = ADPD188_SLOTB;
	ts_config.slot_en = true;
	ts_config.slot_fifo_mode = ADPD188_32BIT_SUM;
	ret = adpd188_slot_setup(regmap, ts_config);
	if (ret < 0)
		return ret;

	ret = adpd188_set_sample_freq(regmap, DEFAULT_SAMPLE_FREQUENCY);
	if (ret < 0)
		return ret;

	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_PD_LED_SELECT,
				     1, ADPD188_SLOTA_LED_SEL_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_PD_LED_SELECT,
				     3, ADPD188_SLOTB_LED_SEL_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_PD_LED_SELECT,
				     1, ADPD188_SLOTA_PD_SEL_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_PD_LED_SELECT,
				     1, ADPD188_SLOTB_PD_SEL_MASK);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, ADPD188_REG_NUM_AVG, 0);
	if (ret < 0)
		return ret;

	/* Slot A chan chop mode and chan offset */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_INT_SEQ_A,
				     9, ADPD188_INTEG_ORDER_A_MASK);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTA_CH1_OFFSET, 0);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTA_CH2_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTA_CH3_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTA_CH4_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;

	/* Slot B chan chop mode and chan offset */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_INT_SEQ_B,
				     9, ADPD188_INTEG_ORDER_B_MASK);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTB_CH1_OFFSET, 0);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTB_CH2_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTB_CH3_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;
	ret = regmap_write(regmap, ADPD188_REG_SLOTB_CH4_OFFSET, 0x3FFF);
	if (ret < 0)
		return ret;

	/** Set IR LED 3 power */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED3),
				     9, ADPD188_ILEDx_COARSE_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED3),
				     3, ADPD188_ILEDx_SLEW_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED3),
				     1, ADPD188_ILEDx_SCALE_MASK);
	if (ret < 0)
		return ret;
	/** Set blue LED 1 power */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED1),
				     6, ADPD188_ILEDx_COARSE_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED1),
				     3, ADPD188_ILEDx_SLEW_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_ILEDx_COARSE(LED1),
				     1, ADPD188_ILEDx_SCALE_MASK);
	if (ret < 0)
		return ret;

	/* Slot A 4 LED pulses with 15us period */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_NUMPULSES(ADPD188_SLOTA),
				     4, ADPD188_SLOTx_PULSES_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_NUMPULSES(ADPD188_SLOTA),
				     0xE, ADPD188_SLOTx_PERIOD_MASK);
	if (ret < 0)
		return ret;
	/* Slot B 4 LED pulses with 15us period */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_NUMPULSES(ADPD188_SLOTB),
				     4, ADPD188_SLOTx_PULSES_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_NUMPULSES(ADPD188_SLOTB),
				     0xE, ADPD188_SLOTx_PERIOD_MASK);
	if (ret < 0)
		return ret;

	/* Slot A integrator window */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_AFE_WINDOW(ADPD188_SLOTA),
				     4, ADPD188_SLOTx_AFE_WIDTH_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_AFE_WINDOW(ADPD188_SLOTA),
				     0x2F0, ADPD188_SLOTx_AFE_OFFSET_MASK);
	if (ret < 0)
		return ret;
	/* Slot B integrator window */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_AFE_WINDOW(ADPD188_SLOTB),
				     4, ADPD188_SLOTx_AFE_WIDTH_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_SLOTx_AFE_WINDOW(ADPD188_SLOTB),
				     0x2F0, ADPD188_SLOTx_AFE_OFFSET_MASK);
	if (ret < 0)
		return ret;

	/* Power down channels 2, 3 and 4 */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_AFE_PWR_CFG1,
				     0x38, ADPD188_AFE_POWERDOWN_MASK);
	if (ret < 0)
		return ret;

	/* Math for chop mode is inverted, non-inverted, non-inverted, inverted */
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_MATH,
				     1, ADPD188_FLT_MATH34_B_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_MATH,
				     1, ADPD188_FLT_MATH34_A_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_MATH,
				     2, ADPD188_FLT_MATH12_B_MASK);
	if (ret < 0)
		return ret;
	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_MATH,
				     2, ADPD188_FLT_MATH12_A_MASK);
	if (ret < 0)
		return ret;

	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_FIFO_THRESH,
					SAMPLE_WORDS, ADPD188_FIFO_THRESH);
	if (ret < 0)
		return ret;

	return adpd188_set_device_mode(regmap, STANDBY);
}

static int adpd188_data_rdy_trigger_set_state(struct iio_trigger *trig,
					      bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct adpd188 *data = iio_priv(indio_dev);
	int reg, ret;

	ret = adpd188_set_device_mode(data->regmap, PROGRAM);
	if (ret < 0)
		return ret;
	reg = state ? 0 : 1;

	ret = adpd188_reg_write_mask(data->regmap, ADPD188_REG_INT_MASK,
				      reg, ADPD188_FIFO_INT_MASK);
	if (ret < 0)
		return ret;

	reg = state ? NORMAL : STANDBY;

	return adpd188_set_device_mode(data->regmap, reg);
}

static const struct iio_trigger_ops adpd188_trigger_ops = {
	.set_trigger_state = adpd188_data_rdy_trigger_set_state,
	.validate_device = iio_trigger_validate_own_device,
};

static irqreturn_t adpd188_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;

	iio_trigger_poll(indio_dev->trig);

	return IRQ_WAKE_THREAD;
};

static irqreturn_t adpd188_oneshot_apply(int irq, void *dev_id)
{
	int ret;
	struct iio_dev *indio_dev = dev_id;
	struct adpd188 *dev_data = iio_priv(indio_dev);

	ret = wait_for_completion_interruptible_timeout(&dev_data->value_ok,
			msecs_to_jiffies(5000));
	if (ret == 0)
		ret = -ETIMEDOUT;
	reinit_completion(&dev_data->value_ok);

	return IRQ_HANDLED;
}

static irqreturn_t adpd188_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adpd188 *dev_data = iio_priv(indio_dev);
	int i;

	regmap_noinc_read(dev_data->regmap, ADPD188_REG_FIFO_DATA,
		      dev_data->data.reg_data, 2 * SAMPLE_WORDS);

	for (i = 0; i < 8; i++)
		dev_data->data.reg_data[i] = (dev_data->data.reg_data[i] >> 8) |
					     (dev_data->data.reg_data[i] << 8);
	iio_push_to_buffers_with_timestamp(indio_dev, dev_data->data.word,
		iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);
	complete(&dev_data->value_ok);

	return IRQ_HANDLED;
}

static int adpd188_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct adpd188 *dev_data = iio_priv(indio_dev);

	return adpd188_reg_write_mask(dev_data->regmap, ADPD188_REG_STATUS,
				      1, ADPD188_FIFO_CLEAR);
}

static const struct iio_buffer_setup_ops adpd188_buffer_setup_ops = {
	.postdisable = &adpd188_buffer_postdisable,
};

static int adpd188_core_setup(struct iio_dev *indio_dev, struct device *dev,
			      struct regmap *regmap, int irq)
{
	int ret;
	int regval;
	struct adpd188 *dev_data = iio_priv(indio_dev);
	struct iio_trigger *trig;

	ret = regmap_read(dev_data->regmap, ADPD188_REG_DEVID, &regval);
	if (ret < 0) {
		dev_err(dev, "Error reading device ID: %d\n", ret);
		return ret;
	}
	if ((regval & ADPD188_DEVID_MASK) != ADPD188_DEVID) {
		dev_err(dev, "Error wrong device ID: %d\n", ret);
		return -ENODEV;
	}

	ret = adpd188_reg_write_mask(dev_data->regmap, ADPD188_REG_SAMPLE_CLK, 1, ADPD188_CLK32K_EN);
	if (ret < 0) {
		dev_err(dev, "Error activating device clock: %d\n", ret);
		return ret;
	}

	ret = adpd188_core_smoke_setup(dev_data->regmap);
	if (ret < 0) {
		dev_err(dev, "Error smoke setup: %d\n", ret);
		return ret;
	}

	ret = adpd188_reg_write_mask(regmap, ADPD188_REG_GPIO_DRV, 1, ADPD188_GPIO0_ENA);
	if (ret < 0)
		return ret;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	trig = devm_iio_trigger_alloc(indio_dev->dev.parent, "%s-dev%d",
				      indio_dev->name,
				      indio_dev->id);
	if (!trig)
		return -ENOMEM;

	trig->dev.parent = indio_dev->dev.parent;
	trig->ops = &adpd188_trigger_ops;
	iio_trigger_set_drvdata(trig, indio_dev);

	ret = devm_iio_trigger_register(dev, trig);
	if (ret)
		return ret;

	indio_dev->trig = trig;

	init_completion(&dev_data->value_ok);

	ret = devm_request_threaded_irq(indio_dev->dev.parent, irq,
			       &adpd188_interrupt, &adpd188_oneshot_apply,
			       IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			       indio_dev->name, indio_dev);
	if (ret)
		return ret;

	return devm_iio_triggered_buffer_setup(indio_dev->dev.parent, indio_dev,
					       &iio_pollfunc_store_time,
					       &adpd188_trigger_handler,
					       NULL); //TODO: postdisable clear FIFO
}

int adpd188_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name, int irq)
{
	struct iio_dev *indio_dev;
	struct adpd188 *dev_data;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*dev_data));
	if (!indio_dev) {
		dev_err(dev, "Error allocating IIO device memory: %ld\n", PTR_ERR(indio_dev));
		return -ENOMEM;
	}

	dev_set_drvdata(dev, indio_dev);
	dev_data = iio_priv(indio_dev);
	dev_data->regmap = regmap;
	dev_data->trigno = 0;

	indio_dev->dev.parent = dev;
	indio_dev->info = &adpd188_info;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adpd188_channels;
	indio_dev->num_channels = ARRAY_SIZE(adpd188_channels);

	ret = adpd188_core_setup(indio_dev, dev, regmap, irq);
	if (ret < 0) {
		dev_err(dev, "Error setting up the device: %d\n", ret);
		return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0)
		dev_err(dev, "iio_device_register failed: %d\n", ret);
	else
		dev_info(dev, "%s probed\n", indio_dev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(adpd188_core_probe);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188core driver");
MODULE_LICENSE("GPL v2");

