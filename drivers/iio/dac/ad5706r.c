// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5706R 16-bit Current Output Digital to Analog Converter
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

/* SPI frame layout */
#define AD5706R_RD_MASK			BIT(15)
#define AD5706R_ADDR_PIN_MASK		GENMASK(13, 12)
#define AD5706R_ADDR_MASK		GENMASK(11, 0)

/* Registers */
#define AD5706R_REG_OUT_OP_MODE			0x22
#define AD5706R_REG_OUT_EN			0x26
#define AD5706R_REG_DAC_INPUT_A_CH(x)		(0x60 + ((x) * 2))
#define AD5706R_REG_DAC_DATA_READBACK_CH(x)	(0x68 + ((x) * 2))
#define AD5706R_REG_BANDGAP_CONTROL		0x73

#define AD5706R_DAC_RESOLUTION		16
#define AD5706R_DAC_MAX_CODE		GENMASK(15, 0)
#define AD5706R_MULTIBYTE_REG_START	0x14
#define AD5706R_MULTIBYTE_REG_END	0x71
#define AD5706R_MAX_REG			0x77
#define AD5706R_DEV_ADDR_MAX		3

#define AD5706R_SAMPLING_FREQ_MIN_HZ	(1 * HZ_PER_KHZ)
#define AD5706R_SAMPLING_FREQ_MAX_HZ	(10 * HZ_PER_MHZ)

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

struct ad5706r_state {
	struct spi_device *spi;
	struct regmap *regmap;

	struct pwm_device *ldac_pwm;
	struct gpio_desc *out_en_gpio;

	u32 dev_addr;
	enum hw_ldac_tg_state_iio_dev_attr hw_ldac_tg_state;
	u64 sampling_frequency;

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
		/*
		 * DECREMENT mode (default): For 2-byte registers, add 1 to address
		 * so device decrements from MSB address to LSB address.
		 */
		reg++;
		put_unaligned_be16(val, &st->tx_buf[2]);
	} else {
		return -EINVAL;
	}

	/* Construct command word: [dev_addr(13:12)] [reg_addr(11:0)] */
	cmd = FIELD_PREP(AD5706R_ADDR_PIN_MASK, st->dev_addr);
	cmd |= FIELD_PREP(AD5706R_ADDR_MASK, reg);
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
	 * DECREMENT mode (default): For 2-byte registers, add 1 to address
	 * so device decrements from MSB address to LSB address.
	 */
	if (num_bytes == 2)
		reg++;

	/* Construct command word: [RD(15)][dev_addr(13:12)] [reg_addr(11:0)] */
	cmd = AD5706R_RD_MASK;
	cmd |= FIELD_PREP(AD5706R_ADDR_PIN_MASK, st->dev_addr);
	cmd |= FIELD_PREP(AD5706R_ADDR_MASK, reg);

	put_unaligned_be16(cmd, &st->tx_buf[0]);
	put_unaligned_be16(0, &st->tx_buf[2]);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	/* Extract value from response (skip 2-byte command echo) */
	if (num_bytes == 1)
		val = st->rx_buf[2];
	else if (num_bytes == 2)
		val = get_unaligned_be16(&st->rx_buf[2]);
	else
		return -EINVAL;

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
	case IIO_CHAN_INFO_ENABLE:
		if (!st->out_en_gpio)
			return -ENODEV;
		*val = gpiod_get_value_cansleep(st->out_en_gpio);
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
	u64 new_period;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > AD5706R_DAC_MAX_CODE)
			return -EINVAL;

		reg = AD5706R_REG_DAC_INPUT_A_CH(chan->channel);
		return regmap_write(st->regmap, reg, val);
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->ldac_pwm)
			return -ENODEV;

		/* Clamp to valid frequency range */
		val = clamp_t(int, val, AD5706R_SAMPLING_FREQ_MIN_HZ, AD5706R_SAMPLING_FREQ_MAX_HZ);

		new_period = DIV_ROUND_CLOSEST_ULL(NANO, val);
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);

		/* Scale duty cycle proportionally to avoid duty_cycle > period */
		if (ldac_pwm_state.period)
			ldac_pwm_state.duty_cycle =
				DIV_ROUND_CLOSEST_ULL(ldac_pwm_state.duty_cycle * new_period,
						      ldac_pwm_state.period);
		ldac_pwm_state.period = new_period;
		ldac_pwm_state.enabled = true;

		ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
		if (ret)
			return ret;

		st->sampling_frequency = val;

		return 0;
	case IIO_CHAN_INFO_ENABLE:
		if (!st->out_en_gpio)
			return -ENODEV;
		if (val != 0 && val != 1)
			return -EINVAL;
		gpiod_set_value_cansleep(st->out_en_gpio, val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int ad5706r_set_pwm_duty_cycle(struct ad5706r_state *st, int duty_cycle)
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
		ret = ad5706r_set_pwm_duty_cycle(st, 100);
	else
		ret = ad5706r_set_pwm_duty_cycle(st, 0);

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

	/* Enable = 50% duty, Disable restores present value of hw_ldac_tg_state */
	if (item == HW_LDAC_TG_PWM_DISABLED) {
		if (st->hw_ldac_tg_state == HW_LDAC_TG_STATE_LOW)
			ret = ad5706r_set_pwm_duty_cycle(st, 0);
		else
			ret = ad5706r_set_pwm_duty_cycle(st, 100);
	} else {
		ret = ad5706r_set_pwm_duty_cycle(st, 50);
	}

	return ret;
}

static int ad5706r_hw_ldac_tg_pwm_read(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldac_pwm_state;

	if (!st->ldac_pwm)
		return -ENODEV;

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

static struct iio_chan_spec_ext_info ad5706r_ext_info[] = {
	IIO_ENUM("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_state", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_state_enum),
	IIO_ENUM("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),
	IIO_ENUM_AVAILABLE("hw_ldac_tg_pwm", IIO_SHARED_BY_ALL, &ad5706r_hw_ldac_tg_pwm_enum),
	{ }
};

#define AD5706R_CHAN(_channel) {				\
	.type = IIO_CURRENT,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) | \
				   BIT(IIO_CHAN_INFO_ENABLE),	\
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

static const char * const ad5706r_supply_names[] = {
	"iovdd", "avdd", "pvdd0", "pvdd1", "pvdd2", "pvdd3",
};

static int ad5706r_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5706r_state *st;
	struct reset_control *rst;
	struct pwm_state ldac_pwm_state;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	/* Initialize default values for device attributes */
	st->sampling_frequency = HZ_PER_MHZ;

	/* Get device address from device tree */
	if (device_property_present(dev, "spi-device-addr")) {
		ret = device_property_read_u32(dev, "spi-device-addr", &st->dev_addr);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to read spi-device-addr\n");

		if (st->dev_addr > AD5706R_DEV_ADDR_MAX)
			return dev_err_probe(dev, -EADDRNOTAVAIL,
					     "spi-device-addr %u out of range [0, %u]\n",
					     st->dev_addr, AD5706R_DEV_ADDR_MAX);
	}

	/* Initialize regmap with custom read/write functions */
	st->regmap = devm_regmap_init(dev, &ad5706r_regmap_bus,
				      st, &ad5706r_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap\n");

	/* Get and enable power supplies */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad5706r_supply_names),
					     ad5706r_supply_names);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get and enable supplies\n");

	/* Voltage reference - external or internal */
	ret = devm_regulator_get_enable_optional(dev, "vref");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "Failed to get and enable vref\n");

	/* Wait t_RESET_DELAY = 200 us after power-on before first register access */
	fsleep(200);

	if (ret == -ENODEV) {
		/* No external reference — enable internal bandgap */
		ret = regmap_write(st->regmap, AD5706R_REG_BANDGAP_CONTROL, 1);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable internal reference\n");
	}

	/* Put all channels in normal (non-SW-shutdown) operating mode */
	ret = regmap_write(st->regmap, AD5706R_REG_OUT_OP_MODE, GENMASK(3, 0));
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set output operating mode\n");

	/* Enable OUT_EN pin control for all channels */
	ret = regmap_write(st->regmap, AD5706R_REG_OUT_EN, GENMASK(3, 0));
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable OUT_EN pin control\n");

	/* Get optional PWM for LDAC/TGP/DCK triggering */
	st->ldac_pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->ldac_pwm)) {
		ret = PTR_ERR(st->ldac_pwm);
		if (ret != -ENOENT && ret != -ENODEV)
			return dev_err_probe(dev, ret, "Failed to get LDAC PWM\n");
		st->ldac_pwm = NULL;
	}

	if (st->ldac_pwm) {
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		ldac_pwm_state.duty_cycle = 0;
		ldac_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, st->sampling_frequency);
		ldac_pwm_state.enabled = true;
		ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to apply PWM state\n");
	}

	/* Optional hardware reset */
	rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "Failed to get reset\n");

	if (rst) {
		ret = reset_control_assert(rst);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to assert reset\n");

		/* Minimum reset low width (t_reset) is 20 ns per datasheet. */
		ndelay(20);

		ret = reset_control_deassert(rst);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to deassert reset\n");
	}

	st->out_en_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(st->out_en_gpio))
		return dev_err_probe(dev, PTR_ERR(st->out_en_gpio),
				     "Failed to get enable GPIO\n");

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
