// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Analog Devices, Inc.
 * Author: Ana-Maria Cusco <ana-maria.cusco@analog.com>
 */

/* In development */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#include <asm/div64.h>
#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include "ad4170.h"



struct ad4170_setup_info {
	bool ref_bufp;
	bool ref_bufm;
	unsigned int burnout;
	unsigned int ref_sel;
	unsigned int pga_bits;
	//unsigned int odr;
	//unsigned int odr_sel_bits;
	enum ad4170_filter_type filter_type;
	//bool live;
	//unsigned int cfg_slot;
};

struct ad4170_chan_info{
	struct ad4170_current_source current_source[AD4170_NUM_CURRENT_SOURCE];
	unsigned int nr;
	struct ad4170_setup_info setup;
	unsigned int ain;
	int slot;
	bool enabled;
	bool initialized;
	u32  iout0;
	u32  iout1;
};


struct ad4170_state {
	struct regmap			*regmap; 
	struct spi_device		*spi; 
	struct clk			*mclk; //common
	struct regulator_bulk_data	regulators[4]; //struct regulator*
	struct mutex 			 lock; //common
	struct ad4170_chan_info   	*chan_info;
	unsigned int num_channels;
	u32				fclk;
	u8			reg_read_tx_buf[2];
	u8			reg_read_rx_buf[4];
	u8			reg_write_tx_buf[6];
	u32			vbias_pins[AD4170_MAX_ANALOG_PINS];
	u32			num_vbias_pins;
	bool bipolar;
	bool			int_ref_en;
	u32			int_ref_uv;
	struct ad4170_pin_muxing pin_muxing;
	struct ad4170_clock_ctrl clock_ctrl;
	struct gpio_desc *dig_aux1_gpio;
	struct gpio_desc *dig_aux2_gpio;
	struct gpio_desc *sync_gpio;
	struct completion		completion;
	struct ad4170_config config;
	struct iio_chan_spec chan;
};

static const char * const ad4170_dig_aux_1_pin_names[] = {
	[AD4170_DIG_AUX1_DISABLED] = "disabled",
	[AD4170_DIG_AUX1_RDY] = "rdy",
	[AD4170_DIG_AUX1_SYNC] = "sync",
};

static const char * const ad4170_dig_aux_2_pin_names[] = {
	[AD4170_DIG_AUX2_DISABLED] = "disabled",
	[AD4170_DIG_AUX2_LDAC] = "ldac",
	[AD4170_DIG_AUX2_SYNC] = "sync",
};

static const char * const ad4170_sync_pin_names[] = {
	[AD4170_SYNC_DISABLED] = "disabled",
	[AD4170_SYNC_STANDARD] = "std",
	[AD4170_SYNC_ALTERNATE] = "alt",
};

static const unsigned int ad4170_iout_current_ua_tbl[AD4170_I_OUT_MAX] = {
	[AD4170_I_OUT_0UA] = 0,
	[AD4170_I_OUT_10UA] = 10,
	[AD4170_I_OUT_50UA] = 50,
	[AD4170_I_OUT_100UA] = 100,
	[AD4170_I_OUT_250UA] = 250,
	[AD4170_I_OUT_500UA] = 500,
	[AD4170_I_OUT_1000UA] = 1000,
	[AD4170_I_OUT_1500UA] = 1500,

};

static const unsigned int ad4170_burnout_current_na_tbl[AD4170_BURNOUT_MAX] = {
	[AD4170_BURNOUT_OFF] = 0,
	[AD4170_BURNOUT_100NA] = 500,
	[AD4170_BURNOUT_2000NA] = 2000,
	[AD4170_BURNOUT_10000NA] = 10000,
};

static int ad4170_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4170_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}



static const struct iio_chan_spec ad4170_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.differential = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_OFFSET) |
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE) |
					BIT(IIO_CHAN_INFO_SAMP_FREQ),
	//.ext_info = ad4170_filter_mode_ext_info,
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};



static int _ad4170_find_table_index(const unsigned int *tbl, size_t len,
				    unsigned int val)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		if (tbl[i] == val)
			return i;

	return -EINVAL;
}

#define ad4170_find_table_index(table, val) \
	_ad4170_find_table_index(table, ARRAY_SIZE(table), val)


static int ad4170_set_mode(struct ad4170_state *st, enum ad4170_mode mode)
{
	return regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				  AD4170_REG_CTRL_MODE_MSK,
				  FIELD_PREP(AD4170_REG_CTRL_MODE_MSK, mode));
}
/* 8 possible setups (slots) (0-7)*/
static int ad4170_write_slot_setup(struct ad4170_state *st,
				   unsigned int slot,
				   struct ad4170_setup *setup)
{
	unsigned int val;
	int ret;
	val = FIELD_PREP(AD4170_ADC_SETUPS_MISC_CHOP_IEXC_MSK, setup->misc.chop_iexc) |
	      FIELD_PREP(AD4170_ADC_SETUPS_MISC_CHOP_ADC_MSK, setup->misc.chop_adc);


	ret = regmap_write(st->regmap, AD4170_MISC_X_REG(slot), val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_BUF_M_MSK, setup->afe.ref_buf_m) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_BUF_P_MSK, setup->afe.ref_buf_p) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_SELECT_MSK, setup->afe.ref_select) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_BIPOLAR_MSK, setup->afe.bipolar) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_PGA_GAIN_MSK, setup->afe.pga_gain);

	ret = regmap_write(st->regmap, AD4170_AFE_X_REG(slot), val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_ADC_SETUPS_POST_FILTER_SEL_MSK, setup->filter.post_filter_sel) |
	      FIELD_PREP(AD4170_ADC_SETUPS_FILTER_TYPE_MSK, setup->filter.filter_type);

	ret = regmap_write(st->regmap, AD4170_FILTER_X_REG(slot), val);
	if (ret)
		return ret;


	ret = regmap_write(st->regmap, AD4170_FILTER_FS_X_REG(slot), setup->filter_fs);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_OFFSET_X_REG(slot), setup->offset);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_GAIN_X_REG(slot), setup->gain);
	if (ret)
		return ret;

	//memcpy(&st->slots_info[slot].setup, setup_info, sizeof(*setup_info));

	return 0;
}
static int ad4170_write_channel_setup(struct ad4170_state *st,
				      unsigned int channel, bool on_enable)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup setup;
	int slot = 0;
	int ret;

	/* Write a hard-coded setup */
	slot = 0;
	setup.afe.bipolar = false;
	setup.afe.pga_gain = AD4170_PGA_GAIN_1;
	setup.afe.ref_buf_m = AD4170_REF_BUF_PRE;
	setup.afe.ref_buf_p = AD4170_REF_BUF_PRE;
	setup.afe.ref_select = AD4170_REFIN_REFOUT;
	setup.filter.filter_type = AD4170_FILT_SINC5_AVG;
	setup.filter.post_filter_sel = AD4170_POST_FILTER_NONE;
	setup.filter_fs = 0x4;
	setup.offset = 0x0; 
	setup.gain = 0x555555;
	setup.misc.chop_iexc = AD4170_CHOP_IEXC_OFF;
	setup.misc.chop_adc = AD4170_CHOP_OFF;
	setup.misc.burnout = AD4170_BURNOUT_OFF;


	// ret = ad4170_write_slot_setup(st, slot, &setup);
	// if (ret)
	// 	return ret;


	/* Hardcode setup 0 for channel x and write it*/
	ret = regmap_update_bits(st->regmap, AD4170_CHANNEL_SETUP_X_REG(channel),
				 AD4170_CHANNEL_SETUPN_SETUP_N_MSK,
				 FIELD_PREP(AD4170_CHANNEL_SETUPN_SETUP_N_MSK, slot));
	if (ret)
		return ret;

	return 0;
}

static int ad4170_set_channel_enable(struct ad4170_state *st,
				     unsigned int channel, bool status)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	//struct ad4170_slot_info *slot_info;
	struct ad4170_setup setup;
	int ret, slot;

	if (chan_info->enabled == status)
		return 0;

	if (status) {
		ret = ad4170_write_channel_setup(st, channel, true);
		if (ret)
			return ret;
	}

	//slot_info = &st->slots_info[chan_info->slot];

	/* Enable the channel*/
	ret = regmap_update_bits(st->regmap, AD4170_CHANNEL_EN_REG,
                         AD4170_CHANNEL_EN(channel),
                         status ? AD4170_CHANNEL_EN(channel) : 0);
	if (ret)
		return ret;

	//slot_info->enabled_channels += status ? 1 : -1;
	chan_info->enabled = status;

	return 0;
}


static int _ad4170_read_sample(struct iio_dev *indio_dev, unsigned int channel,
			       int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad4170_set_channel_enable(st, channel, true);
	if (ret)
		return ret;

	reinit_completion(&st->completion);

	ret = ad4170_set_mode(st, AD4170_MODE_SINGLE);
	if (ret)
		return ret;

	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4170_DATA_24b_REG, val);
	if (ret)
		return ret;

	ret = ad4170_set_channel_enable(st, channel, false);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static int ad4170_read_sample(struct iio_dev *indio_dev, unsigned int channel,
			      int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = _ad4170_read_sample(indio_dev, channel, val);
	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}


static int _ad4170_read_raw(struct iio_dev *indio_dev, unsigned int channel,
			    int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;
	uint8_t rdyb;
	int timeout = 1000;

	/* Single conversion Mode*/
	ret = regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				 AD4170_REG_CTRL_MODE_MSK, AD4170_MODE_SINGLE);
	if (ret)
		return ret;

	rdyb = 0;
	while(rdyb == 0 && timeout--){
		rdyb = gpiod_get_value(st->dig_aux1_gpio);
		pr_err("%s: %d rdyb=%d\n", __FUNCTION__, __LINE__, rdyb);
		
	}
	if (!timeout)
		pr_err("%s: %d TIMEOUT\n", __FUNCTION__, __LINE__);

	ret = regmap_read(st->regmap, AD4170_DATA_PER_CHANNEL_X_REG(0), val);
	if (ret)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	return IIO_VAL_INT;
}
static int ad4170_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;

	switch (info) {
	case IIO_CHAN_INFO_RAW:

		//Check if buffer enabled
		//LOck indio_dev->mlock
		//preoare_chhannel
		//set_channel
		//set active slots
		
		return _ad4170_read_raw(indio_dev, channel, val);
		//return ad4170_read_sample(indio_dev, channel, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad4170_info = {
	.read_raw = ad4170_read_raw,
	// .read_avail = ad4170_read_avail,
	// .write_raw = ad4170_write_raw,
	// .update_scan_mode = ad4170_update_scan_mode,
	 .debugfs_reg_access = ad4170_reg_access,
};


static int ad4170_soft_reset(struct ad4170_state *st)
{
	int ret;
	unsigned int reg = AD4170_INTERFACE_CONFIG_A_REG;

	ret = regmap_write(st->regmap, reg, AD4170_SW_RESET_MSK);
	if (ret)
		return ret;

	/* AD4170-4 requires a minimum of 1 ms between any reset event and 
	a register read/write transaction */
	fsleep(AD4170_RESET_SLEEP_US);

	return 0;
}

static int ad4170_get_reg_size(struct ad4170_state *st, unsigned int reg,
			       unsigned int *size)
{
	if (reg >= ARRAY_SIZE(ad4170_reg_size))
		return -EINVAL;
	
	if (!ad4170_reg_size[reg])
		return -EINVAL;

	*size = ad4170_reg_size[reg];

	return 0;
}

static int ad4170_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4170_state *st = context;
	unsigned int size, addr;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	if (ret)
		return ret;

	addr = reg + size - 1;
	// st->reg_write_tx_buf[0] = (reg >> 8) & 0x3F; 
	// st->reg_write_tx_buf[1] = reg & 0xFF; 
	put_unaligned_be16(addr, &st->reg_write_tx_buf[0]);

	switch (size) {
	case 4:
		put_unaligned_be32(val, &st->reg_write_tx_buf[2]);
		break;
	case 3:
		put_unaligned_be24(val, &st->reg_write_tx_buf[2]);
		break;
	case 2:
		put_unaligned_be16(val, &st->reg_write_tx_buf[2]);
		break;
	case 1:
		st->reg_write_tx_buf[2] = val;
		break;
	default:
		return -EINVAL;
	}

	print_hex_dump(KERN_ERR, "ad4170", DUMP_PREFIX_OFFSET, 8, 1,
                       st->reg_write_tx_buf, size + 2, true);

	return spi_write(st->spi, st->reg_write_tx_buf, size + 2);
}

static int ad4170_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4170_state *st = context;
	struct spi_transfer t[] = {
		{
			.tx_buf = st->reg_read_tx_buf,
			.len = ARRAY_SIZE(st->reg_read_tx_buf),
		},
		{
			.rx_buf = st->reg_read_rx_buf,
		},
	};
	unsigned int size, addr;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	if (ret) 
		return ret;
	
	addr = reg + size - 1;
	put_unaligned_be16(AD4170_READ_MASK | addr, &st->reg_read_tx_buf[0]);
	/* check what happens if reg exceeds max value, as well ad wtiting to non existent addr*/
	// st->reg_read_tx_buf[0] = AD4170_READ_MASK | ((reg >> 8) & 0x3F);
	// st->reg_read_tx_buf[1] = reg & 0xFF;
	t[1].len = size;

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;
	

	switch (size) {
	case 4:
		*val = get_unaligned_be32(st->reg_read_rx_buf);
		break;
	case 3:
		*val = get_unaligned_be24(st->reg_read_rx_buf);
		break;
	case 2:
		*val = get_unaligned_be16(st->reg_read_rx_buf);
		break;
	case 1:
		*val = st->reg_read_rx_buf[0];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config ad4170_regmap_config = {
	.reg_bits = 14,
	.val_bits = 32,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
    	.val_format_endian = REGMAP_ENDIAN_BIG,
	.reg_read = ad4170_reg_read,
	.reg_write = ad4170_reg_write,
};


static void ad4170_clk_disable_unprepare(void *clk)
{
	clk_disable_unprepare(clk);
}

static inline bool ad4170_valid_external_frequency(u32 freq)
{
	return (freq >= AD4170_EXT_FREQ_MHZ_MIN &&
		freq <= AD4170_EXT_FREQ_MHZ_MAX);
}
static int ad4170_of_clock_select(struct ad4170_state *st)
{
	struct device_node *np = st->spi->dev.of_node;
	unsigned int rate;
	int ret;

	st->clock_ctrl.clocksel = AD4170_INTERNAL_OSC;
	rate = AD4170_INT_FREQ_16MHZ;

	/* use internal clock */
	if (PTR_ERR(st->mclk) == -ENOENT) {
		pr_err("%s: %d\n", __FUNCTION__, __LINE__);
		if (of_property_read_bool(np, "adi,int-clock-output-enable"))
			st->clock_ctrl.clocksel = AD4170_INTERNAL_OSC_OUTPUT;
	} else {
		if (of_property_read_bool(np, "adi,clock-xtal"))
			st->clock_ctrl.clocksel = AD4170_EXTERNAL_XTAL;
		else {
			pr_err("%s: %d\n", __FUNCTION__, __LINE__);
			st->clock_ctrl.clocksel = AD4170_EXTERNAL_OSC;
		}

		rate = clk_get_rate(st->mclk);
	}

	if (!ad4170_valid_external_frequency(rate))
		return -EINVAL;

	pr_err("%s: %d rate=%d\n", __FUNCTION__, __LINE__,rate);
	ret  = clk_set_rate(st->mclk, rate);
	if (ret < 0)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);

	return 0;
}


static void ad4170_parse_digif_fw(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	const char *function;
	int ret, i;

	st->pin_muxing.dig_aux1_ctrl = AD4170_DIG_AUX1_DISABLED;
	st->dig_aux1_gpio = devm_gpiod_get_optional(&st->spi->dev, "dig-aux1", GPIOD_OUT_LOW);
	if (IS_ERR(st->dig_aux2_gpio)) 
		dev_warn(&st->spi->dev, "Could not get dig-aux1-gpios\n");


	ret = of_property_read_string(st->spi->dev.of_node, 
					     "adi,dig-aux1-function", &function);
	if (ret < 0) 
    		dev_warn(&st->spi->dev, "Could not read dig-aux1-function\n");
	
	for (i = 0; !ret && i < ARRAY_SIZE(ad4170_dig_aux_1_pin_names); i++) {
		if (!strcmp(function, ad4170_dig_aux_1_pin_names[i])) {
			st->pin_muxing.dig_aux1_ctrl = i;
			break;
		}
	}

	st->pin_muxing.dig_aux2_ctrl = AD4170_DIG_AUX2_DISABLED;
	st->dig_aux2_gpio = devm_gpiod_get_optional(&st->spi->dev, "dig-aux2", GPIOD_IN);
	if (IS_ERR(st->dig_aux2_gpio))
		dev_warn(&st->spi->dev, "Could not get dig-aux2-gpios\n");


	ret = of_property_read_string(st->spi->dev.of_node, 
					     "adi,dig-aux2-function", &function);
	if (ret < 0) 
    		dev_warn(&st->spi->dev, "Could not read dig-aux2-function\n");
	
	for (i = 0; !ret && i < ARRAY_SIZE(ad4170_dig_aux_2_pin_names); i++) {
		if (!strcmp(function, ad4170_dig_aux_2_pin_names[i])) {
			st->pin_muxing.dig_aux2_ctrl = i;
			break;
		}
	}

	st->pin_muxing.sync_ctrl = AD4170_SYNC_STANDARD;
	st->sync_gpio = devm_gpiod_get_optional(&st->spi->dev, "sync", GPIOD_OUT_LOW);
	if (IS_ERR(st->sync_gpio)) 
		dev_warn(&st->spi->dev, "Could not get dig-aux1-gpios\n");


	ret = of_property_read_string(st->spi->dev.of_node, 
					     "adi,dig-aux1-function", &function);
	if (ret < 0) 
    		dev_warn(&st->spi->dev, "Could not read dig-aux1-function\n");
	
	for (i = 0; !ret && i < ARRAY_SIZE(ad4170_sync_pin_names); i++) {
		if (!strcmp(function, ad4170_sync_pin_names[i])) {
			st->pin_muxing.sync_ctrl = i;
			break;
		}
	}
}
static int ad4170_get_ref_voltage(struct ad4170_state *st,
				  enum ad4170_ref_select ref_sel)
{
	switch (ref_sel) {
	case AD4170_REFIN_REFIN1:
		return regulator_get_voltage(st->regulators[2].consumer);
	case AD4170_REFIN_REFIN2:
		return regulator_get_voltage(st->regulators[3].consumer);
	case AD4170_REFIN_AVDD:
		return regulator_get_voltage(st->regulators[0].consumer);
	case AD4170_REFIN_REFOUT:
		return st->int_ref_uv;
	default:
		return -EINVAL;
	}
}

static int ad4170_parse_fw_setup(struct ad4170_state *st,
				 struct fwnode_handle *child,
				 struct ad4170_chan_info *chan_info)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,excitation-current-0-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	chan_info->current_source[0].i_out_val = ret;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,excitation-current-1-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	chan_info->current_source[1].i_out_val = ret;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,excitation-current-2-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	chan_info->current_source[2].i_out_val = ret;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,excitation-current-3-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	chan_info->current_source[3].i_out_val = ret;


	tmp = 0;
	fwnode_property_read_u32(child, "adi,burnout-current-nanoamp", &tmp);
	ret = ad4170_find_table_index(ad4170_burnout_current_na_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid burnout current %unA\n", tmp);
	chan_info->setup.burnout = ret;

	chan_info->setup.ref_bufp = fwnode_property_read_bool(child, "adi,buffered-positive");
	chan_info->setup.ref_bufm= fwnode_property_read_bool(child, "adi,buffered-negative");

	chan_info->setup.ref_sel = AD4170_REFIN_REFOUT;
	fwnode_property_read_u32(child, "adi,reference-select",
				 &chan_info->setup.ref_sel);
	if (chan_info->setup.ref_sel >= AD4170_REFIN_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid reference selected %u\n",
				     chan_info->setup.ref_sel);

	if (chan_info->setup.ref_sel == AD4170_REFIN_REFOUT)
		st->int_ref_en = true;

	ret = ad4170_get_ref_voltage(st, chan_info->setup.ref_sel);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Cannot use reference %u\n",
				    chan_info->setup.ref_sel);

	return 0;
}

static int ad4170_parse_fw_channel(struct iio_dev *indio_dev,
				   struct fwnode_handle *child)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	//unsigned int resolution = ad4130_resolution(st);
	unsigned int index = 0;
	struct device *dev = &st->spi->dev;
	struct ad4170_chan_info *chan_info;
	struct iio_chan_spec *chan;
	u32 pins[2];
	int ret;

	
	ret = fwnode_property_read_u32(child, "reg", &index);
	if (ret)
		return ret;

	if (index >= indio_dev->num_channels)
		return dev_err_probe(dev, -EINVAL, "Channel idx greater than no of channels\n");
	
	chan = &indio_dev->channels[index];
	chan_info = &st->chan_info[index];

	*chan = ad4170_channel_template;
	//chan->scan_type.realbits = resolution;
	//chan->scan_type.storagebits = resolution;
	chan->address = index;
	chan->scan_index = index;
	

	chan_info->slot = AD4170_INVALID_SLOT;
	//chan_info->setup.fs = AD4170_FILTER_SELECT_MIN;
	chan_info->initialized = true;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	ret = fwnode_property_read_u32_array(child, "diff-channels", pins,
					     ARRAY_SIZE(pins));
	if (ret)
		return ret;

	// ret = ad4170_validate_diff_channels(st, pins, ARRAY_SIZE(pins));
	// if (ret)
	// 	return ret;

	chan->channel = pins[0];
	chan->channel2 = pins[1];

	ret = ad4170_parse_fw_setup(st, child, chan_info);
	if (ret)
		return ret;

	fwnode_property_read_u32(child, "adi,excitation-pin-0",
				 &chan_info->current_source[0].i_out_pin);
	// if (chan_info->setup.iout0_val != AD4170_I_OUT_0UA) {
	// 	ret = ad4730_validate_excitation_pin(st, chan_info->iout0);
	// 	if (ret)
	// 		return ret;
	// }

	fwnode_property_read_u32(child, "adi,excitation-pin-1",
				 &chan_info->current_source[1].i_out_pin);
	// if (chan_info->setup.iout1_val != AD4170_I_OUT_0UA) {
	// 	ret = ad4170_validate_excitation_pin(st, chan_info->iout1);
	// 	if (ret)
	// 		return ret;
	// }

	fwnode_property_read_u32(child, "adi,excitation-pin-1",
				 &chan_info->current_source[2].i_out_pin);
	// if (chan_info->setup.iout1_val != AD4170_I_OUT_0UA) {
	// 	ret = ad4170_validate_excitation_pin(st, chan_info->iout1);
	// 	if (ret)
	// 		return ret;
	// }

	fwnode_property_read_u32(child, "adi,excitation-pin-1",
				 &chan_info->current_source[3].i_out_pin);
	// if (chan_info->setup.iout1_val != AD4170_I_OUT_0UA) {
	// 	ret = ad4170_validate_excitation_pin(st, chan_info->iout1);
	// 	if (ret)
	// 		return ret;
	// }


	return 0;
}

static int ad4170_parse_fw_children(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *child;
	struct ad4170_chan_info *chan_info;
	struct iio_chan_spec *chan;
	unsigned int num_channels;
	int ret;

	num_channels = device_get_child_node_count(dev);
	if (!num_channels)
		return dev_err_probe(&indio_dev->dev, -ENODEV,
				     "no channels defined\n");
	
	chan = devm_kcalloc(dev, num_channels, sizeof(chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan_info = devm_kcalloc(dev, num_channels, 
				     sizeof(chan_info), GFP_KERNEL);
	if (!chan_info)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = num_channels;
	st->chan_info = chan_info;

	device_for_each_child_node(dev, child) {
		ret = ad4170_parse_fw_channel(indio_dev, child);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	return 0;
}


static int ad4170_parse_fw(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *chan;
	struct fwnode_handle *child;
	unsigned int num_channels;
	int ret;

	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	st->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(st->mclk))
		return dev_err_probe(dev, PTR_ERR(st->mclk),
				     "Failed to get mclk\n");

	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	ret = ad4170_of_clock_select(st);
	if (ret)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	ad4170_parse_digif_fw(indio_dev);
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);

	st->bipolar = device_property_read_bool(dev, "adi,bipolar");
	
	ret = device_property_count_u32(dev, "adi,vbias-pins");
	if (ret > 0) {
		if (ret > AD4170_MAX_ANALOG_PINS)
			return dev_err_probe(dev, -EINVAL,
					     "Too many vbias pins %u\n", ret);

		st->num_vbias_pins = ret;

		ret = device_property_read_u32_array(dev, "adi,vbias-pins",
						     st->vbias_pins,
						     st->num_vbias_pins);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to read vbias pins\n");

		// ret = ad4170_validate_vbias_pins(st, st->vbias_pins,
		// 				 st->num_vbias_pins);
		// if (ret)
		// 	return ret;
	}


	ret = ad4170_parse_fw_children(indio_dev);
	if (ret)
		return ret;

	return 0;
}

static void ad4170_disable_regulators(void *data)
{
	struct ad4170_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static int 
ad4170_setup(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	unsigned int int_ref_val;
	unsigned int val;
	unsigned int i;
	int ret;

	ret = clk_prepare_enable(st->mclk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ad4170_clk_disable_unprepare,
				       st->mclk);
	if (ret)
		return ret;

	st->int_ref_uv = AD4170_INT_REF_2_5V;


	ret = regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				 AD4170_REG_CTRL_MODE_MSK, AD4170_MODE_IDLE);
	if (ret)
		return ret;


	/* Setup channels. */
	// for (i = 0; i < indio_dev->num_channels; i++) {
	// 	struct ad4170_chan_info *chan_info = &st->chan_info[i];
	// 	struct iio_chan_spec *chan = &st->chans[i];
	// 	unsigned int val;

	// 	val = FIELD_PREP(AD4170_CHANNEL_MAPN_AINP_MSK, chan->channel) |
	// 	      FIELD_PREP(AD4170_CHANNEL_MAPN_AINM_MSK, chan->channel2);

	// 	ret = regmap_write(st->regmap, AD4130_CHANNEL_X_REG(i), val);
	// 	if (ret)
	// 		return ret;
	// }

	return 0;
}


static int ad4170_hardcoded_read_raw_setup(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret, val;
	st->chan = ad4170_channel_template;
	st->chan.address = 0;
	st->chan.scan_index = 0;
	st->chan.channel = 0x1D;
	st->chan.channel2= 0x18;

	indio_dev->num_channels = 1;
	indio_dev->channels = &st->chan;


	st->dig_aux1_gpio = devm_gpiod_get(dev, "dig-aux1", GPIOD_OUT_LOW);
	if (IS_ERR(st->dig_aux1_gpio)) 
		dev_warn(dev, "Could not get dig-aux1-gpios\n");

	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	/* DIg aux 1 is used as a data ready signal*/
	ret = regmap_update_bits(st->regmap, AD4170_PIN_MUXING_REG,
				AD4170_PIN_MUXING_DIG_AUX1_CTRL_MSK,
				FIELD_PREP(AD4170_PIN_MUXING_DIG_AUX1_CTRL_MSK,
					   AD4170_DIG_AUX1_RDY));
	if(ret)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);

	/* Single conversion Mode*/
	ret = regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				 AD4170_REG_CTRL_MODE_MSK, AD4170_MODE_SINGLE);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4170_ADC_CTRL_REG, &val);
	if (ret)
		return ret;
	pr_err("%s: %d  AD4170_ADC_CTRL: 0x%x\n", __FUNCTION__, __LINE__, val);
	/* Enable channel 0*/	
	ret = regmap_update_bits(st->regmap, AD4170_CHANNEL_EN_REG,
                         AD4170_CHANNEL_EN(0),
                         1);
	if (ret)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	val = FIELD_PREP(AD4170_CHANNEL_MAPN_AINP_MSK, 0x1D) |
	      FIELD_PREP(AD4170_CHANNEL_MAPN_AINM_MSK, 0x18);

	ret = regmap_write(st->regmap, AD4170_CHANNEL_MAP_X_REG(0), val);
	if (ret)
		return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static int ad4170_probe(struct spi_device *spi)
{

	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4170_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	init_completion(&st->completion);
	mutex_init(&st->lock);
	st->spi = spi;

	indio_dev->name = AD4170_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4170_info;

	st->regmap = devm_regmap_init(dev, NULL, st, &ad4170_regmap_config);

	st->regulators[0].supply = "avdd";
	st->regulators[1].supply = "iovdd";
	st->regulators[2].supply = "refin1";
	st->regulators[3].supply = "refin2";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4170_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	ret = ad4170_soft_reset(st);
	if (ret) 
		return ret;

	ret = ad4170_hardcoded_read_raw_setup(indio_dev);
	if (ret)
		return ret;
	// ret = ad4170_parse_fw(indio_dev);
	// if (ret)
	// 	return ret;

	// ret = ad4170_setup(indio_dev);
	// if (ret)
	// 	return ret;
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);
	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4170_of_match[] = {
	{
		.compatible = "adi,ad4170",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4170_of_match);

static struct spi_driver ad4170_driver = {
	.driver = {
		.name = AD4170_NAME,
		.of_match_table = ad4170_of_match,
	},
	.probe = ad4170_probe,
};
module_spi_driver(ad4170_driver);

MODULE_AUTHOR("Ana-Maria Cusco <ana-maria.cuso@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4170 SPI driver");
MODULE_LICENSE("GPL");