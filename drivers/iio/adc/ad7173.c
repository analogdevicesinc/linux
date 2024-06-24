// SPDX-License-Identifier: GPL-2.0+
/*
 * AD717x family SPI ADC driver
 *
 * Supported devices:
 *  AD7172-2/AD7172-4/AD7173-8/AD7175-2
 *  AD7175-8/AD7176-2/AD7177-2
 *
 * Copyright (C) 2015, 2024 Analog Devices, Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/container_of.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/regmap.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/units.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/iio/adc/ad_sigma_delta.h>

#define AD7173_REG_COMMS		0x00
#define AD7173_REG_ADC_MODE		0x01
#define AD7173_REG_INTERFACE_MODE	0x02
#define AD7173_REG_CRC			0x03
#define AD7173_REG_DATA			0x04
#define AD7173_REG_GPIO			0x06
#define AD7173_REG_ID			0x07
#define AD7173_REG_CH(x)		(0x10 + (x))
#define AD7173_REG_SETUP(x)		(0x20 + (x))
#define AD7173_REG_FILTER(x)		(0x28 + (x))
#define AD7173_REG_OFFSET(x)		(0x30 + (x))
#define AD7173_REG_GAIN(x)		(0x38 + (x))

#define AD7173_RESET_LENGTH		BITS_TO_BYTES(64)

#define AD7173_CH_ENABLE		BIT(15)
#define AD7173_CH_SETUP_SEL_MASK	GENMASK(14, 12)
#define AD7173_CH_SETUP_AINPOS_MASK	GENMASK(9, 5)
#define AD7173_CH_SETUP_AINNEG_MASK	GENMASK(4, 0)

#define AD7173_CH_ADDRESS(pos, neg) \
	(FIELD_PREP(AD7173_CH_SETUP_AINPOS_MASK, pos) | \
	 FIELD_PREP(AD7173_CH_SETUP_AINNEG_MASK, neg))
#define AD7173_AIN_TEMP_POS	17
#define AD7173_AIN_TEMP_NEG	18

#define AD7172_2_ID			0x00d0
#define AD7175_ID			0x0cd0
#define AD7176_ID			0x0c90
#define AD7175_2_ID			0x0cd0
#define AD7172_4_ID			0x2050
#define AD7173_ID			0x30d0
#define AD7175_8_ID			0x3cd0
#define AD7177_ID			0x4fd0
#define AD7173_ID_MASK			GENMASK(15, 4)

#define AD7173_ADC_MODE_REF_EN		BIT(15)
#define AD7173_ADC_MODE_SING_CYC	BIT(13)
#define AD7173_ADC_MODE_MODE_MASK	GENMASK(6, 4)
#define AD7173_ADC_MODE_CLOCKSEL_MASK	GENMASK(3, 2)
#define AD7173_ADC_MODE_CLOCKSEL_INT		0x0
#define AD7173_ADC_MODE_CLOCKSEL_INT_OUTPUT	0x1
#define AD7173_ADC_MODE_CLOCKSEL_EXT		0x2
#define AD7173_ADC_MODE_CLOCKSEL_XTAL		0x3

#define AD7173_GPIO_PDSW	BIT(14)
#define AD7173_GPIO_OP_EN2_3	BIT(13)
#define AD7173_GPIO_MUX_IO	BIT(12)
#define AD7173_GPIO_SYNC_EN	BIT(11)
#define AD7173_GPIO_ERR_EN	BIT(10)
#define AD7173_GPIO_ERR_DAT	BIT(9)
#define AD7173_GPIO_GP_DATA3	BIT(7)
#define AD7173_GPIO_GP_DATA2	BIT(6)
#define AD7173_GPIO_IP_EN1	BIT(5)
#define AD7173_GPIO_IP_EN0	BIT(4)
#define AD7173_GPIO_OP_EN1	BIT(3)
#define AD7173_GPIO_OP_EN0	BIT(2)
#define AD7173_GPIO_GP_DATA1	BIT(1)
#define AD7173_GPIO_GP_DATA0	BIT(0)

#define AD7173_GPO12_DATA(x)	BIT((x) + 0)
#define AD7173_GPO23_DATA(x)	BIT((x) + 4)
#define AD7173_GPO_DATA(x)	((x) < 2 ? AD7173_GPO12_DATA(x) : AD7173_GPO23_DATA(x))

#define AD7173_INTERFACE_DATA_STAT	BIT(6)
#define AD7173_INTERFACE_DATA_STAT_EN(x) \
	FIELD_PREP(AD7173_INTERFACE_DATA_STAT, x)

#define AD7173_SETUP_BIPOLAR		BIT(12)
#define AD7173_SETUP_AREF_BUF_MASK	GENMASK(11, 10)
#define AD7173_SETUP_AIN_BUF_MASK	GENMASK(9, 8)

#define AD7173_SETUP_REF_SEL_MASK	GENMASK(5, 4)
#define AD7173_SETUP_REF_SEL_AVDD1_AVSS	0x3
#define AD7173_SETUP_REF_SEL_INT_REF	0x2
#define AD7173_SETUP_REF_SEL_EXT_REF2	0x1
#define AD7173_SETUP_REF_SEL_EXT_REF	0x0
#define AD7173_VOLTAGE_INT_REF_uV	2500000
#define AD7173_TEMP_SENSIIVITY_uV_per_C	477
#define AD7177_ODR_START_VALUE		0x07

#define AD7173_FILTER_ODR0_MASK		GENMASK(5, 0)
#define AD7173_MAX_CONFIGS		8

enum ad7173_ids {
	ID_AD7172_2,
	ID_AD7172_4,
	ID_AD7173_8,
	ID_AD7175_2,
	ID_AD7175_8,
	ID_AD7176_2,
	ID_AD7177_2,
};

struct ad7173_device_info {
	const unsigned int *sinc5_data_rates;
	unsigned int num_sinc5_data_rates;
	unsigned int odr_start_value;
	unsigned int num_channels;
	unsigned int num_configs;
	unsigned int num_inputs;
	unsigned int clock;
	unsigned int id;
	char *name;
	bool has_temp;
	bool has_input_buf;
	bool has_int_ref;
	bool has_ref2;
	u8 num_gpios;
};

struct ad7173_channel_config {
	u8 cfg_slot;
	bool live;

	/* Following fields are used to compare equality. */
	struct_group(config_props,
		bool bipolar;
		bool input_buf;
		u8 odr;
		u8 ref_sel;
	);
};

struct ad7173_channel {
	unsigned int chan_reg;
	unsigned int ain;
	struct ad7173_channel_config cfg;
};

struct ad7173_state {
	struct ad_sigma_delta sd;
	const struct ad7173_device_info *info;
	struct ad7173_channel *channels;
	struct regulator_bulk_data regulators[3];
	unsigned int adc_mode;
	unsigned int interface_mode;
	unsigned int num_channels;
	struct ida cfg_slots_status;
	unsigned long long config_usage_counter;
	unsigned long long *config_cnts;
	struct clk *ext_clk;
	struct clk_hw int_clk_hw;
#if IS_ENABLED(CONFIG_GPIOLIB)
	struct regmap *reg_gpiocon_regmap;
	struct gpio_regmap *gpio_regmap;
#endif
};

static const unsigned int ad7173_sinc5_data_rates[] = {
	6211000, 6211000, 6211000, 6211000, 6211000, 6211000, 5181000, 4444000,	/*  0-7  */
	3115000, 2597000, 1007000, 503800,  381000,  200300,  100500,  59520,	/*  8-15 */
	49680,	 20010,	  16333,   10000,   5000,    2500,    1250,		/* 16-22 */
};

static const unsigned int ad7175_sinc5_data_rates[] = {
	50000000, 41667000, 31250000, 27778000,	/*  0-3  */
	20833000, 17857000, 12500000, 10000000,	/*  4-7  */
	5000000,  2500000,  1000000,  500000,	/*  8-11 */
	397500,   200000,   100000,   59920,	/* 12-15 */
	49960,    20000,    16666,    10000,	/* 16-19 */
	5000,					/* 20    */
};

static const struct ad7173_device_info ad7173_device_info[] = {
	[ID_AD7172_2] = {
		.name = "ad7172-2",
		.id = AD7172_2_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.num_gpios = 2,
		.has_temp = true,
		.has_input_buf = true,
		.has_int_ref = true,
		.clock = 2 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7173_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7173_sinc5_data_rates),
	},
	[ID_AD7172_4] = {
		.name = "ad7172-4",
		.id = AD7172_4_ID,
		.num_inputs = 9,
		.num_channels = 8,
		.num_configs = 8,
		.num_gpios = 4,
		.has_temp = false,
		.has_input_buf = true,
		.has_ref2 = true,
		.clock = 2 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7173_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7173_sinc5_data_rates),
	},
	[ID_AD7173_8] = {
		.name = "ad7173-8",
		.id = AD7173_ID,
		.num_inputs = 17,
		.num_channels = 16,
		.num_configs = 8,
		.num_gpios = 4,
		.has_temp = true,
		.has_input_buf = true,
		.has_int_ref = true,
		.has_ref2 = true,
		.clock = 2 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7173_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7173_sinc5_data_rates),
	},
	[ID_AD7175_2] = {
		.name = "ad7175-2",
		.id = AD7175_2_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.num_gpios = 2,
		.has_temp = true,
		.has_input_buf = true,
		.has_int_ref = true,
		.clock = 16 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
	[ID_AD7175_8] = {
		.name = "ad7175-8",
		.id = AD7175_8_ID,
		.num_inputs = 17,
		.num_channels = 16,
		.num_configs = 8,
		.num_gpios = 4,
		.has_temp = true,
		.has_input_buf = true,
		.has_int_ref = true,
		.has_ref2 = true,
		.clock = 16 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
	[ID_AD7176_2] = {
		.name = "ad7176-2",
		.id = AD7176_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.num_gpios = 2,
		.has_temp = false,
		.has_input_buf = false,
		.has_int_ref = true,
		.clock = 16 * HZ_PER_MHZ,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
	[ID_AD7177_2] = {
		.name = "ad7177-2",
		.id = AD7177_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.num_gpios = 2,
		.has_temp = true,
		.has_input_buf = true,
		.has_int_ref = true,
		.clock = 16 * HZ_PER_MHZ,
		.odr_start_value = AD7177_ODR_START_VALUE,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
};

static const char *const ad7173_ref_sel_str[] = {
	[AD7173_SETUP_REF_SEL_EXT_REF]    = "vref",
	[AD7173_SETUP_REF_SEL_EXT_REF2]   = "vref2",
	[AD7173_SETUP_REF_SEL_INT_REF]    = "refout-avss",
	[AD7173_SETUP_REF_SEL_AVDD1_AVSS] = "avdd",
};

static const char *const ad7173_clk_sel[] = {
	"ext-clk", "xtal"
};

#if IS_ENABLED(CONFIG_GPIOLIB)

static const struct regmap_range ad7173_range_gpio[] = {
	regmap_reg_range(AD7173_REG_GPIO, AD7173_REG_GPIO),
};

static const struct regmap_access_table ad7173_access_table = {
	.yes_ranges = ad7173_range_gpio,
	.n_yes_ranges = ARRAY_SIZE(ad7173_range_gpio),
};

static const struct regmap_config ad7173_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.rd_table = &ad7173_access_table,
	.wr_table = &ad7173_access_table,
	.read_flag_mask = BIT(6),
};

static int ad7173_mask_xlate(struct gpio_regmap *gpio, unsigned int base,
			     unsigned int offset, unsigned int *reg,
			     unsigned int *mask)
{
	*mask = AD7173_GPO_DATA(offset);
	*reg = base;
	return 0;
}

static void ad7173_gpio_disable(void *data)
{
	struct ad7173_state *st = data;
	unsigned int mask;

	mask = AD7173_GPIO_OP_EN0 | AD7173_GPIO_OP_EN1 | AD7173_GPIO_OP_EN2_3;
	regmap_update_bits(st->reg_gpiocon_regmap, AD7173_REG_GPIO, mask, ~mask);
}

static int ad7173_gpio_init(struct ad7173_state *st)
{
	struct gpio_regmap_config gpio_regmap = {};
	struct device *dev = &st->sd.spi->dev;
	unsigned int mask;
	int ret;

	st->reg_gpiocon_regmap = devm_regmap_init_spi(st->sd.spi, &ad7173_regmap_config);
	ret = PTR_ERR_OR_ZERO(st->reg_gpiocon_regmap);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to init regmap\n");

	mask = AD7173_GPIO_OP_EN0 | AD7173_GPIO_OP_EN1 | AD7173_GPIO_OP_EN2_3;
	regmap_update_bits(st->reg_gpiocon_regmap, AD7173_REG_GPIO, mask, mask);

	ret = devm_add_action_or_reset(dev, ad7173_gpio_disable, st);
	if (ret)
		return ret;

	gpio_regmap.parent = dev;
	gpio_regmap.regmap = st->reg_gpiocon_regmap;
	gpio_regmap.ngpio = st->info->num_gpios;
	gpio_regmap.reg_set_base = AD7173_REG_GPIO;
	gpio_regmap.reg_mask_xlate = ad7173_mask_xlate;

	st->gpio_regmap = devm_gpio_regmap_register(dev, &gpio_regmap);
	ret = PTR_ERR_OR_ZERO(st->gpio_regmap);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to init gpio-regmap\n");

	return 0;
}
#else
static int ad7173_gpio_init(struct ad7173_state *st)
{
	return 0;
}
#endif /* CONFIG_GPIOLIB */

static struct ad7173_state *ad_sigma_delta_to_ad7173(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct ad7173_state, sd);
}

static struct ad7173_state *clk_hw_to_ad7173(struct clk_hw *hw)
{
	return container_of(hw, struct ad7173_state, int_clk_hw);
}

static void ad7173_ida_destroy(void *data)
{
	struct ad7173_state *st = data;

	ida_destroy(&st->cfg_slots_status);
}

static void ad7173_reset_usage_cnts(struct ad7173_state *st)
{
	memset64(st->config_cnts, 0, st->info->num_configs);
	st->config_usage_counter = 0;
}

static struct ad7173_channel_config *
ad7173_find_live_config(struct ad7173_state *st, struct ad7173_channel_config *cfg)
{
	struct ad7173_channel_config *cfg_aux;
	ptrdiff_t cmp_size;
	int i;

	cmp_size = sizeof_field(struct ad7173_channel_config, config_props);
	for (i = 0; i < st->num_channels; i++) {
		cfg_aux = &st->channels[i].cfg;

		if (cfg_aux->live &&
		    !memcmp(&cfg->config_props, &cfg_aux->config_props, cmp_size))
			return cfg_aux;
	}
	return NULL;
}

/* Could be replaced with a generic LRU implementation */
static int ad7173_free_config_slot_lru(struct ad7173_state *st)
{
	int i, lru_position = 0;

	for (i = 1; i < st->info->num_configs; i++)
		if (st->config_cnts[i] < st->config_cnts[lru_position])
			lru_position = i;

	for (i = 0; i < st->num_channels; i++)
		if (st->channels[i].cfg.cfg_slot == lru_position)
			st->channels[i].cfg.live = false;

	ida_free(&st->cfg_slots_status, lru_position);
	return ida_alloc(&st->cfg_slots_status, GFP_KERNEL);
}

/* Could be replaced with a generic LRU implementation */
static int ad7173_load_config(struct ad7173_state *st,
			      struct ad7173_channel_config *cfg)
{
	unsigned int config;
	int free_cfg_slot, ret;

	free_cfg_slot = ida_alloc_range(&st->cfg_slots_status, 0,
					st->info->num_configs - 1, GFP_KERNEL);
	if (free_cfg_slot < 0)
		free_cfg_slot = ad7173_free_config_slot_lru(st);

	cfg->cfg_slot = free_cfg_slot;
	config = FIELD_PREP(AD7173_SETUP_REF_SEL_MASK, cfg->ref_sel);

	if (cfg->bipolar)
		config |= AD7173_SETUP_BIPOLAR;

	if (cfg->input_buf)
		config |= AD7173_SETUP_AIN_BUF_MASK;

	ret = ad_sd_write_reg(&st->sd, AD7173_REG_SETUP(free_cfg_slot), 2, config);
	if (ret)
		return ret;

	return ad_sd_write_reg(&st->sd, AD7173_REG_FILTER(free_cfg_slot), 2,
			       AD7173_FILTER_ODR0_MASK & cfg->odr);
}

static int ad7173_config_channel(struct ad7173_state *st, int addr)
{
	struct ad7173_channel_config *cfg = &st->channels[addr].cfg;
	struct ad7173_channel_config *live_cfg;
	int ret;

	if (!cfg->live) {
		live_cfg = ad7173_find_live_config(st, cfg);
		if (live_cfg) {
			cfg->cfg_slot = live_cfg->cfg_slot;
		} else {
			ret = ad7173_load_config(st, cfg);
			if (ret)
				return ret;
			cfg->live = true;
		}
	}

	if (st->config_usage_counter == U64_MAX)
		ad7173_reset_usage_cnts(st);

	st->config_usage_counter++;
	st->config_cnts[cfg->cfg_slot] = st->config_usage_counter;

	return 0;
}

static int ad7173_set_channel(struct ad_sigma_delta *sd, unsigned int channel)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);
	unsigned int val;
	int ret;

	ret = ad7173_config_channel(st, channel);
	if (ret)
		return ret;

	val = AD7173_CH_ENABLE |
	      FIELD_PREP(AD7173_CH_SETUP_SEL_MASK, st->channels[channel].cfg.cfg_slot) |
	      st->channels[channel].ain;

	return ad_sd_write_reg(&st->sd, AD7173_REG_CH(channel), 2, val);
}

static int ad7173_set_mode(struct ad_sigma_delta *sd,
			   enum ad_sigma_delta_mode mode)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);

	st->adc_mode &= ~AD7173_ADC_MODE_MODE_MASK;
	st->adc_mode |= FIELD_PREP(AD7173_ADC_MODE_MODE_MASK, mode);

	return ad_sd_write_reg(&st->sd, AD7173_REG_ADC_MODE, 2, st->adc_mode);
}

static int ad7173_append_status(struct ad_sigma_delta *sd, bool append)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);
	unsigned int interface_mode = st->interface_mode;
	int ret;

	interface_mode &= ~AD7173_INTERFACE_DATA_STAT;
	interface_mode |= AD7173_INTERFACE_DATA_STAT_EN(append);
	ret = ad_sd_write_reg(&st->sd, AD7173_REG_INTERFACE_MODE, 2, interface_mode);
	if (ret)
		return ret;

	st->interface_mode = interface_mode;

	return 0;
}

static int ad7173_disable_all(struct ad_sigma_delta *sd)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);
	int ret;
	int i;

	for (i = 0; i < st->num_channels; i++) {
		ret = ad_sd_write_reg(sd, AD7173_REG_CH(i), 2, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static struct ad_sigma_delta_info ad7173_sigma_delta_info = {
	.set_channel = ad7173_set_channel,
	.append_status = ad7173_append_status,
	.disable_all = ad7173_disable_all,
	.set_mode = ad7173_set_mode,
	.has_registers = true,
	.addr_shift = 0,
	.read_mask = BIT(6),
	.status_ch_mask = GENMASK(3, 0),
	.data_reg = AD7173_REG_DATA,
};

static int ad7173_setup(struct iio_dev *indio_dev)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct device *dev = &st->sd.spi->dev;
	u8 buf[AD7173_RESET_LENGTH];
	unsigned int id;
	int ret;

	/* reset the serial interface */
	memset(buf, 0xff, AD7173_RESET_LENGTH);
	ret = spi_write_then_read(st->sd.spi, buf, sizeof(buf), NULL, 0);
	if (ret < 0)
		return ret;

	/* datasheet recommends a delay of at least 500us after reset */
	fsleep(500);

	ret = ad_sd_read_reg(&st->sd, AD7173_REG_ID, 2, &id);
	if (ret)
		return ret;

	id &= AD7173_ID_MASK;
	if (id != st->info->id)
		dev_warn(dev, "Unexpected device id: 0x%04X, expected: 0x%04X\n",
			 id, st->info->id);

	st->adc_mode |= AD7173_ADC_MODE_SING_CYC;
	st->interface_mode = 0x0;

	st->config_usage_counter = 0;
	st->config_cnts = devm_kcalloc(dev, st->info->num_configs,
				       sizeof(*st->config_cnts), GFP_KERNEL);
	if (!st->config_cnts)
		return -ENOMEM;

	/* All channels are enabled by default after a reset */
	return ad7173_disable_all(&st->sd);
}

static unsigned int ad7173_get_ref_voltage_milli(struct ad7173_state *st,
						 u8 reference_select)
{
	int vref;

	switch (reference_select) {
	case AD7173_SETUP_REF_SEL_EXT_REF:
		vref = regulator_get_voltage(st->regulators[0].consumer);
		break;

	case AD7173_SETUP_REF_SEL_EXT_REF2:
		vref = regulator_get_voltage(st->regulators[1].consumer);
		break;

	case AD7173_SETUP_REF_SEL_INT_REF:
		vref = AD7173_VOLTAGE_INT_REF_uV;
		break;

	case AD7173_SETUP_REF_SEL_AVDD1_AVSS:
		vref = regulator_get_voltage(st->regulators[2].consumer);
		break;

	default:
		return -EINVAL;
	}

	if (vref < 0)
		return vref;

	return vref / (MICRO / MILLI);
}

static int ad7173_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct ad7173_channel *ch = &st->channels[chan->address];
	unsigned int reg;
	u64 temp;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad_sigma_delta_single_conversion(indio_dev, chan, val);
		if (ret < 0)
			return ret;

		/* disable channel after single conversion */
		ret = ad_sd_write_reg(&st->sd, AD7173_REG_CH(chan->address), 2, 0);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			temp = AD7173_VOLTAGE_INT_REF_uV * MILLI;
			temp /= AD7173_TEMP_SENSIIVITY_uV_per_C;
			*val = temp;
			*val2 = chan->scan_type.realbits;
		} else {
			*val = ad7173_get_ref_voltage_milli(st, ch->cfg.ref_sel);
			*val2 = chan->scan_type.realbits - !!(ch->cfg.bipolar);
		}
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			/* 0 Kelvin -> raw sample */
			temp   = -ABSOLUTE_ZERO_MILLICELSIUS;
			temp  *= AD7173_TEMP_SENSIIVITY_uV_per_C;
			temp <<= chan->scan_type.realbits;
			temp   = DIV_U64_ROUND_CLOSEST(temp,
						       AD7173_VOLTAGE_INT_REF_uV *
						       MILLI);
			*val   = -temp;
		} else {
			*val = -BIT(chan->scan_type.realbits - 1);
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		reg = st->channels[chan->address].cfg.odr;

		*val = st->info->sinc5_data_rates[reg] / MILLI;
		*val2 = (st->info->sinc5_data_rates[reg] % MILLI) * (MICRO / MILLI);

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad7173_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct ad7173_channel_config *cfg;
	unsigned int freq, i;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		freq = val * MILLI + val2 / MILLI;
		for (i = st->info->odr_start_value; i < st->info->num_sinc5_data_rates - 1; i++)
			if (freq >= st->info->sinc5_data_rates[i])
				break;

		cfg = &st->channels[chan->address].cfg;
		cfg->odr = i;
		cfg->live = false;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static int ad7173_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	int i, ret;

	for (i = 0; i < indio_dev->num_channels; i++) {
		if (test_bit(i, scan_mask))
			ret = ad7173_set_channel(&st->sd, i);
		else
			ret = ad_sd_write_reg(&st->sd, AD7173_REG_CH(i), 2, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad7173_debug_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				   unsigned int writeval, unsigned int *readval)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	u8 reg_size;

	if (reg == AD7173_REG_COMMS)
		reg_size = 1;
	else if (reg == AD7173_REG_CRC || reg == AD7173_REG_DATA ||
		 reg >= AD7173_REG_OFFSET(0))
		reg_size = 3;
	else
		reg_size = 2;

	if (readval)
		return ad_sd_read_reg(&st->sd, reg, reg_size, readval);

	return ad_sd_write_reg(&st->sd, reg, reg_size, writeval);
}

static const struct iio_info ad7173_info = {
	.read_raw = &ad7173_read_raw,
	.write_raw = &ad7173_write_raw,
	.debugfs_reg_access = &ad7173_debug_reg_access,
	.validate_trigger = ad_sd_validate_trigger,
	.update_scan_mode = ad7173_update_scan_mode,
};

static const struct iio_chan_spec ad7173_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};

static const struct iio_chan_spec ad7173_temp_iio_channel_template = {
	.type = IIO_TEMP,
	.channel = AD7173_AIN_TEMP_POS,
	.channel2 = AD7173_AIN_TEMP_NEG,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET) |
		BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};

static void ad7173_disable_regulators(void *data)
{
	struct ad7173_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static void ad7173_clk_disable_unprepare(void *clk)
{
	clk_disable_unprepare(clk);
}

static unsigned long ad7173_sel_clk(struct ad7173_state *st,
				    unsigned int clk_sel)
{
	int ret;

	st->adc_mode &= ~AD7173_ADC_MODE_CLOCKSEL_MASK;
	st->adc_mode |= FIELD_PREP(AD7173_ADC_MODE_CLOCKSEL_MASK, clk_sel);
	ret = ad_sd_write_reg(&st->sd, AD7173_REG_ADC_MODE, 0x2, st->adc_mode);

	return ret;
}

static unsigned long ad7173_clk_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct ad7173_state *st = clk_hw_to_ad7173(hw);

	return st->info->clock / HZ_PER_KHZ;
}

static int ad7173_clk_output_is_enabled(struct clk_hw *hw)
{
	struct ad7173_state *st = clk_hw_to_ad7173(hw);
	u32 clk_sel;

	clk_sel = FIELD_GET(AD7173_ADC_MODE_CLOCKSEL_MASK, st->adc_mode);
	return clk_sel == AD7173_ADC_MODE_CLOCKSEL_INT_OUTPUT;
}

static int ad7173_clk_output_prepare(struct clk_hw *hw)
{
	struct ad7173_state *st = clk_hw_to_ad7173(hw);

	return ad7173_sel_clk(st, AD7173_ADC_MODE_CLOCKSEL_INT_OUTPUT);
}

static void ad7173_clk_output_unprepare(struct clk_hw *hw)
{
	struct ad7173_state *st = clk_hw_to_ad7173(hw);

	ad7173_sel_clk(st, AD7173_ADC_MODE_CLOCKSEL_INT);
}

static const struct clk_ops ad7173_int_clk_ops = {
	.recalc_rate = ad7173_clk_recalc_rate,
	.is_enabled = ad7173_clk_output_is_enabled,
	.prepare = ad7173_clk_output_prepare,
	.unprepare = ad7173_clk_output_unprepare,
};

static int ad7173_register_clk_provider(struct iio_dev *indio_dev)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct clk_init_data init = {};
	int ret;

	if (!IS_ENABLED(CONFIG_COMMON_CLK))
		return 0;

	init.name = fwnode_get_name(fwnode);
	init.ops = &ad7173_int_clk_ops;

	st->int_clk_hw.init = &init;
	ret = devm_clk_hw_register(dev, &st->int_clk_hw);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					   &st->int_clk_hw);
}

static int ad7173_fw_parse_channel_config(struct iio_dev *indio_dev)
{
	struct ad7173_channel *chans_st_arr, *chan_st_priv;
	struct ad7173_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct iio_chan_spec *chan_arr, *chan;
	unsigned int ain[2], chan_index = 0;
	int ref_sel, ret;

	chan_arr = devm_kcalloc(dev, sizeof(*indio_dev->channels),
				st->num_channels, GFP_KERNEL);
	if (!chan_arr)
		return -ENOMEM;

	chans_st_arr = devm_kcalloc(dev, st->num_channels, sizeof(*st->channels),
				    GFP_KERNEL);
	if (!chans_st_arr)
		return -ENOMEM;

	indio_dev->channels = chan_arr;
	st->channels = chans_st_arr;

	if (st->info->has_temp) {
		chan_arr[chan_index] = ad7173_temp_iio_channel_template;
		chan_st_priv = &chans_st_arr[chan_index];
		chan_st_priv->ain =
			AD7173_CH_ADDRESS(chan_arr[chan_index].channel,
					  chan_arr[chan_index].channel2);
		chan_st_priv->cfg.bipolar = false;
		chan_st_priv->cfg.input_buf = st->info->has_input_buf;
		chan_st_priv->cfg.ref_sel = AD7173_SETUP_REF_SEL_INT_REF;
		st->adc_mode |= AD7173_ADC_MODE_REF_EN;

		chan_index++;
	}

	device_for_each_child_node_scoped(dev, child) {
		chan = &chan_arr[chan_index];
		chan_st_priv = &chans_st_arr[chan_index];
		ret = fwnode_property_read_u32_array(child, "diff-channels",
						     ain, ARRAY_SIZE(ain));
		if (ret)
			return ret;

		if (ain[0] >= st->info->num_inputs ||
		    ain[1] >= st->info->num_inputs)
			return dev_err_probe(dev, -EINVAL,
				"Input pin number out of range for pair (%d %d).\n",
				ain[0], ain[1]);

		ret = fwnode_property_match_property_string(child,
							    "adi,reference-select",
							    ad7173_ref_sel_str,
							    ARRAY_SIZE(ad7173_ref_sel_str));
		if (ret < 0)
			ref_sel = AD7173_SETUP_REF_SEL_INT_REF;
		else
			ref_sel = ret;

		if (ref_sel == AD7173_SETUP_REF_SEL_INT_REF &&
		    !st->info->has_int_ref)
			return dev_err_probe(dev, -EINVAL,
				"Internal reference is not available on current model.\n");

		if (ref_sel == AD7173_SETUP_REF_SEL_EXT_REF2 && !st->info->has_ref2)
			return dev_err_probe(dev, -EINVAL,
				"External reference 2 is not available on current model.\n");

		ret = ad7173_get_ref_voltage_milli(st, ref_sel);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "Cannot use reference %u\n", ref_sel);

		if (ref_sel == AD7173_SETUP_REF_SEL_INT_REF)
			st->adc_mode |= AD7173_ADC_MODE_REF_EN;
		chan_st_priv->cfg.ref_sel = ref_sel;

		*chan = ad7173_channel_template;
		chan->address = chan_index;
		chan->scan_index = chan_index;
		chan->channel = ain[0];
		chan->channel2 = ain[1];
		chan->differential = true;

		chan_st_priv->ain = AD7173_CH_ADDRESS(ain[0], ain[1]);
		chan_st_priv->chan_reg = chan_index;
		chan_st_priv->cfg.input_buf = st->info->has_input_buf;
		chan_st_priv->cfg.odr = 0;

		chan_st_priv->cfg.bipolar = fwnode_property_read_bool(child, "bipolar");
		if (chan_st_priv->cfg.bipolar)
			chan->info_mask_separate |= BIT(IIO_CHAN_INFO_OFFSET);

		chan_index++;
	}
	return 0;
}

static int ad7173_fw_parse_device_config(struct iio_dev *indio_dev)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	unsigned int num_channels;
	int ret;

	st->regulators[0].supply = ad7173_ref_sel_str[AD7173_SETUP_REF_SEL_EXT_REF];
	st->regulators[1].supply = ad7173_ref_sel_str[AD7173_SETUP_REF_SEL_EXT_REF2];
	st->regulators[2].supply = ad7173_ref_sel_str[AD7173_SETUP_REF_SEL_AVDD1_AVSS];

	/*
	 * If a regulator is not available, it will be set to a dummy regulator.
	 * Each channel reference is checked with regulator_get_voltage() before
	 * setting attributes so if any channel uses a dummy supply the driver
	 * probe will fail.
	 */
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad7173_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	ret = device_property_match_property_string(dev, "clock-names",
						    ad7173_clk_sel,
						    ARRAY_SIZE(ad7173_clk_sel));
	if (ret < 0) {
		st->adc_mode |= FIELD_PREP(AD7173_ADC_MODE_CLOCKSEL_MASK,
					   AD7173_ADC_MODE_CLOCKSEL_INT);
		ad7173_register_clk_provider(indio_dev);
	} else {
		st->adc_mode |= FIELD_PREP(AD7173_ADC_MODE_CLOCKSEL_MASK,
					   AD7173_ADC_MODE_CLOCKSEL_EXT + ret);
		st->ext_clk = devm_clk_get(dev, ad7173_clk_sel[ret]);
		if (IS_ERR(st->ext_clk))
			return dev_err_probe(dev, PTR_ERR(st->ext_clk),
					     "Failed to get external clock\n");

		ret = clk_prepare_enable(st->ext_clk);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable external clock\n");

		ret = devm_add_action_or_reset(dev, ad7173_clk_disable_unprepare,
					       st->ext_clk);
		if (ret)
			return ret;
	}

	ret = fwnode_irq_get_byname(dev_fwnode(dev), "rdy");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Interrupt 'rdy' is required\n");

	ad7173_sigma_delta_info.irq_line = ret;

	num_channels = device_get_child_node_count(dev);

	if (st->info->has_temp)
		num_channels++;

	if (num_channels == 0)
		return dev_err_probe(dev, -ENODATA, "No channels specified\n");
	indio_dev->num_channels = num_channels;
	st->num_channels = num_channels;

	return ad7173_fw_parse_channel_config(indio_dev);
}

static int ad7173_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ad7173_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	ida_init(&st->cfg_slots_status);
	ret = devm_add_action_or_reset(dev, ad7173_ida_destroy, st);
	if (ret)
		return ret;

	indio_dev->name = st->info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad7173_info;

	spi->mode = SPI_MODE_3;
	spi_setup(spi);

	ad7173_sigma_delta_info.num_slots = st->info->num_configs;
	ret = ad_sd_init(&st->sd, indio_dev, spi, &ad7173_sigma_delta_info);
	if (ret)
		return ret;

	ret = ad7173_fw_parse_device_config(indio_dev);
	if (ret)
		return ret;

	ret = devm_ad_sd_setup_buffer_and_trigger(dev, indio_dev);
	if (ret)
		return ret;

	ret = ad7173_setup(indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_GPIOLIB))
		return ad7173_gpio_init(st);

	return 0;
}

static const struct of_device_id ad7173_of_match[] = {
	{ .compatible = "adi,ad7172-2",
	  .data = &ad7173_device_info[ID_AD7172_2]},
	{ .compatible = "adi,ad7172-4",
	  .data = &ad7173_device_info[ID_AD7172_4]},
	{ .compatible = "adi,ad7173-8",
	  .data = &ad7173_device_info[ID_AD7173_8]},
	{ .compatible = "adi,ad7175-2",
	  .data = &ad7173_device_info[ID_AD7175_2]},
	{ .compatible = "adi,ad7175-8",
	  .data = &ad7173_device_info[ID_AD7175_8]},
	{ .compatible = "adi,ad7176-2",
	  .data = &ad7173_device_info[ID_AD7176_2]},
	{ .compatible = "adi,ad7177-2",
	  .data = &ad7173_device_info[ID_AD7177_2]},
	{ }
};
MODULE_DEVICE_TABLE(of, ad7173_of_match);

static const struct spi_device_id ad7173_id_table[] = {
	{ "ad7172-2", (kernel_ulong_t)&ad7173_device_info[ID_AD7172_2]},
	{ "ad7172-4", (kernel_ulong_t)&ad7173_device_info[ID_AD7172_4]},
	{ "ad7173-8", (kernel_ulong_t)&ad7173_device_info[ID_AD7173_8]},
	{ "ad7175-2", (kernel_ulong_t)&ad7173_device_info[ID_AD7175_2]},
	{ "ad7175-8", (kernel_ulong_t)&ad7173_device_info[ID_AD7175_8]},
	{ "ad7176-2", (kernel_ulong_t)&ad7173_device_info[ID_AD7176_2]},
	{ "ad7177-2", (kernel_ulong_t)&ad7173_device_info[ID_AD7177_2]},
	{ }
};
MODULE_DEVICE_TABLE(spi, ad7173_id_table);

static struct spi_driver ad7173_driver = {
	.driver = {
		.name	= "ad7173",
		.of_match_table = ad7173_of_match,
	},
	.probe		= ad7173_probe,
	.id_table	= ad7173_id_table,
};
module_spi_driver(ad7173_driver);

MODULE_IMPORT_NS(IIO_AD_SIGMA_DELTA);
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafo.de>");
MODULE_AUTHOR("Dumitru Ceclan <dumitru.ceclan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7172/AD7173/AD7175/AD7176 ADC driver");
MODULE_LICENSE("GPL");
