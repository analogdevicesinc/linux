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
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

/* SPI frame layout */
#define AD5706R_RD_MASK			BIT(15)
#define AD5706R_ADDR_PIN_MASK		GENMASK(13, 12)
#define AD5706R_ADDR_MASK		GENMASK(11, 0)

/* Registers */
#define AD5706R_REG_LDAC_SYNC_ASYNC		0x16
#define AD5706R_REG_OUT_OP_MODE			0x22
#define AD5706R_REG_OUT_EN			0x26
#define AD5706R_REG_FUNC_EN			0x30
#define AD5706R_REG_FUNC_MODE_SEL_CH(x)		(0x32 + ((x) * 2))
#define AD5706R_REG_FUNC_DAC_INPUT_B_CH(x)	(0x3A + ((x) * 2))
#define AD5706R_REG_FUNC_DITHER_PERIOD_CH(x)	(0x42 + ((x) * 2))
#define AD5706R_REG_FUNC_DITHER_PHASE_CH(x)	(0x4A + ((x) * 2))
#define AD5706R_REG_DAC_INPUT_A_CH(x)		(0x60 + ((x) * 2))
#define AD5706R_REG_DAC_DATA_READBACK_CH(x)	(0x68 + ((x) * 2))
#define AD5706R_REG_BANDGAP_CONTROL		0x73

#define AD5706R_DAC_RESOLUTION		16
#define AD5706R_DAC_MAX_CODE		GENMASK(15, 0)
#define AD5706R_DITHER_PERIOD_MAX	GENMASK(3, 0)
#define AD5706R_MULTIBYTE_REG_START	0x14
#define AD5706R_MULTIBYTE_REG_END	0x71
#define AD5706R_MAX_REG			0x77
#define AD5706R_DEV_ADDR_MAX		3

#define AD5706R_FUNC_MODE_TOGGLE	0
#define AD5706R_FUNC_MODE_DITHER	1
#define AD5706R_CH_EN_BIT(ch)		BIT(ch)

#define AD5706R_SAMPLING_FREQ_MIN_HZ	(1 * HZ_PER_KHZ)
#define AD5706R_SAMPLING_FREQ_MAX_HZ	(25 * HZ_PER_MHZ)

enum ad5706r_chan_idx {
	AD5706R_CHAN_IDX_PHY_0,
	AD5706R_CHAN_IDX_PHY_1,
	AD5706R_CHAN_IDX_PHY_2,
	AD5706R_CHAN_IDX_PHY_3,
	AD5706R_CHAN_IDX_TOGGLE_0,
	AD5706R_CHAN_IDX_TOGGLE_1,
	AD5706R_CHAN_IDX_TOGGLE_2,
	AD5706R_CHAN_IDX_TOGGLE_3,
	AD5706R_CHAN_IDX_DITHER_0,
	AD5706R_CHAN_IDX_DITHER_1,
	AD5706R_CHAN_IDX_DITHER_2,
	AD5706R_CHAN_IDX_DITHER_3,
};

struct ad5706r_state {
	struct spi_device *spi;
	struct regmap *regmap;

	struct pwm_device *ldac_pwm;
	struct gpio_desc *out_en_gpio;

	u32 dev_addr;

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

static unsigned int ad5706r_phys_ch(const struct iio_chan_spec *chan)
{
	return chan->address & 3;
}

static int ad5706r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ad5706r_state *st = iio_priv(indio_dev);
	struct pwm_state ldac_pwm_state;
	unsigned int reg, reg_val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->address) {
		case AD5706R_CHAN_IDX_PHY_0 ... AD5706R_CHAN_IDX_PHY_3:
			reg = AD5706R_REG_DAC_DATA_READBACK_CH(ad5706r_phys_ch(chan));
			break;
		case AD5706R_CHAN_IDX_TOGGLE_0 ... AD5706R_CHAN_IDX_TOGGLE_3:
		case AD5706R_CHAN_IDX_DITHER_0 ... AD5706R_CHAN_IDX_DITHER_3:
			reg = AD5706R_REG_FUNC_DAC_INPUT_B_CH(ad5706r_phys_ch(chan));
			break;
		default:
			return -EINVAL;
		}
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;
		*val = reg_val;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 50;
		*val2 = AD5706R_DAC_RESOLUTION;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->address) {
		case AD5706R_CHAN_IDX_PHY_0 ... AD5706R_CHAN_IDX_PHY_3:
			ret = regmap_read(st->regmap, AD5706R_REG_OUT_EN, &reg_val);
			if (ret)
				return ret;
			*val = !!(reg_val & AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)));
			return IIO_VAL_INT;
		case AD5706R_CHAN_IDX_TOGGLE_0 ... AD5706R_CHAN_IDX_TOGGLE_3:
		case AD5706R_CHAN_IDX_DITHER_0 ... AD5706R_CHAN_IDX_DITHER_3:
			ret = regmap_read(st->regmap, AD5706R_REG_FUNC_EN, &reg_val);
			if (ret)
				return ret;
			if (!(reg_val & AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)))) {
				*val = 0;
				return IIO_VAL_INT;
			}
			ret = regmap_read(st->regmap,
					  AD5706R_REG_FUNC_MODE_SEL_CH(ad5706r_phys_ch(chan)),
					  &reg_val);
			if (ret)
				return ret;
			if (chan->address >= AD5706R_CHAN_IDX_DITHER_0)
				*val = !!(reg_val & BIT(0));
			else
				*val = !(reg_val & BIT(0));
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->ldac_pwm)
			return -ENODEV;
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		if (!ldac_pwm_state.period)
			return -EINVAL;
		*val = DIV_ROUND_CLOSEST_ULL(NANO, ldac_pwm_state.period);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		if (!st->ldac_pwm)
			return -ENODEV;
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		if (!ldac_pwm_state.period)
			return -EINVAL;
		ret = regmap_read(st->regmap,
				  AD5706R_REG_FUNC_DITHER_PERIOD_CH(ad5706r_phys_ch(chan)),
				  &reg_val);
		if (ret)
			return ret;
		if (!reg_val)
			reg_val = 1;
		*val = DIV_ROUND_CLOSEST_ULL(NANO, ldac_pwm_state.period * reg_val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		reg = AD5706R_REG_FUNC_DITHER_PHASE_CH(ad5706r_phys_ch(chan));
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;
		*val = reg_val;
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
		switch (chan->address) {
		case AD5706R_CHAN_IDX_PHY_0 ... AD5706R_CHAN_IDX_PHY_3:
			reg = AD5706R_REG_DAC_INPUT_A_CH(ad5706r_phys_ch(chan));
			break;
		case AD5706R_CHAN_IDX_TOGGLE_0 ... AD5706R_CHAN_IDX_TOGGLE_3:
		case AD5706R_CHAN_IDX_DITHER_0 ... AD5706R_CHAN_IDX_DITHER_3:
			reg = AD5706R_REG_FUNC_DAC_INPUT_B_CH(ad5706r_phys_ch(chan));
			break;
		default:
			return -EINVAL;
		}
		return regmap_write(st->regmap, reg, val);
	case IIO_CHAN_INFO_ENABLE:
		if (val != 0 && val != 1)
			return -EINVAL;
		switch (chan->address) {
		case AD5706R_CHAN_IDX_PHY_0 ... AD5706R_CHAN_IDX_PHY_3:
			if (!st->out_en_gpio)
				return -ENODEV;
			return regmap_update_bits(st->regmap, AD5706R_REG_OUT_EN,
						  AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)),
						  val ? AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)) : 0);
		case AD5706R_CHAN_IDX_TOGGLE_0 ... AD5706R_CHAN_IDX_TOGGLE_3:
			if (!st->ldac_pwm)
				return -ENODEV;
			if (val) {
				/* Clear FUNC_EN before changing mode */
				ret = regmap_update_bits(st->regmap, AD5706R_REG_FUNC_EN,
							 AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)), 0);
				if (ret)
					return ret;
				ret = regmap_write(st->regmap,
						   AD5706R_REG_FUNC_MODE_SEL_CH(ad5706r_phys_ch(chan)),
						   AD5706R_FUNC_MODE_TOGGLE);
				if (ret)
					return ret;
			}
			ret = regmap_update_bits(st->regmap, AD5706R_REG_FUNC_EN,
						 AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)),
						 val ? AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)) : 0);
			if (ret)
				return ret;
			/* 50% duty cycle for even toggle, 0% when disabled */
			pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
			ldac_pwm_state.duty_cycle = val ?
				DIV_ROUND_CLOSEST_ULL(ldac_pwm_state.period, 2) : 0;
			return pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
		case AD5706R_CHAN_IDX_DITHER_0 ... AD5706R_CHAN_IDX_DITHER_3:
			if (val) {
				/* Clear FUNC_EN before changing mode */
				ret = regmap_update_bits(st->regmap, AD5706R_REG_FUNC_EN,
							 AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)), 0);
				if (ret)
					return ret;
				ret = regmap_write(st->regmap,
						   AD5706R_REG_FUNC_MODE_SEL_CH(ad5706r_phys_ch(chan)),
						   AD5706R_FUNC_MODE_DITHER);
				if (ret)
					return ret;
			}
			return regmap_update_bits(st->regmap, AD5706R_REG_FUNC_EN,
						  AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)),
						  val ? AD5706R_CH_EN_BIT(ad5706r_phys_ch(chan)) : 0);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!st->ldac_pwm)
			return -ENODEV;
		val = clamp_t(int, val, AD5706R_SAMPLING_FREQ_MIN_HZ,
			      AD5706R_SAMPLING_FREQ_MAX_HZ);
		new_period = DIV_ROUND_CLOSEST_ULL(NANO, val);
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		if (ldac_pwm_state.period)
			ldac_pwm_state.duty_cycle =
				DIV_ROUND_CLOSEST_ULL(ldac_pwm_state.duty_cycle * new_period,
						      ldac_pwm_state.period);
		ldac_pwm_state.period = new_period;
		ldac_pwm_state.enabled = true;
		return pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
	case IIO_CHAN_INFO_FREQUENCY: {
		unsigned int period_val;

		if (!st->ldac_pwm)
			return -ENODEV;
		if (val <= 0)
			return -EINVAL;
		pwm_get_state(st->ldac_pwm, &ldac_pwm_state);
		if (!ldac_pwm_state.period)
			return -EINVAL;
		period_val = DIV_ROUND_CLOSEST_ULL(NANO, ldac_pwm_state.period * val);
		if (period_val < 1 || period_val > AD5706R_DITHER_PERIOD_MAX)
			return -EINVAL;
		return regmap_write(st->regmap,
				    AD5706R_REG_FUNC_DITHER_PERIOD_CH(ad5706r_phys_ch(chan)),
				    period_val);
	}
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > AD5706R_DAC_MAX_CODE)
			return -EINVAL;
		reg = AD5706R_REG_FUNC_DITHER_PHASE_CH(ad5706r_phys_ch(chan));
		return regmap_write(st->regmap, reg, val);
	default:
		return -EINVAL;
	}
}

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

static const char * const ad5706r_channel_str[] = {
	[AD5706R_CHAN_IDX_PHY_0]    = "phy0",
	[AD5706R_CHAN_IDX_PHY_1]    = "phy1",
	[AD5706R_CHAN_IDX_PHY_2]    = "phy2",
	[AD5706R_CHAN_IDX_PHY_3]    = "phy3",
	[AD5706R_CHAN_IDX_TOGGLE_0] = "toggle0",
	[AD5706R_CHAN_IDX_TOGGLE_1] = "toggle1",
	[AD5706R_CHAN_IDX_TOGGLE_2] = "toggle2",
	[AD5706R_CHAN_IDX_TOGGLE_3] = "toggle3",
	[AD5706R_CHAN_IDX_DITHER_0] = "dither0",
	[AD5706R_CHAN_IDX_DITHER_1] = "dither1",
	[AD5706R_CHAN_IDX_DITHER_2] = "dither2",
	[AD5706R_CHAN_IDX_DITHER_3] = "dither3",
};

static int ad5706r_read_label(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      char *label)
{
	return sysfs_emit(label, "%s\n", ad5706r_channel_str[chan->address]);
}

static const struct iio_info ad5706r_info = {
	.read_raw = ad5706r_read_raw,
	.write_raw = ad5706r_write_raw,
	.read_label = ad5706r_read_label,
};

#define AD5706R_PHY_CHAN(ch) {						\
	.type = IIO_CURRENT,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (ch),						\
	.address = AD5706R_CHAN_IDX_PHY_ ## ch,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE) |			\
			      BIT(IIO_CHAN_INFO_ENABLE),			\
}

#define AD5706R_TOGGLE_CHAN(ch) {					\
	.type = IIO_CURRENT,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (ch) + 100,						\
	.address = AD5706R_CHAN_IDX_TOGGLE_ ## ch,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_ENABLE) |		\
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.parent = &ad5706r_channels[AD5706R_CHAN_IDX_PHY_ ## ch],	\
}

#define AD5706R_DITHER_CHAN(ch) {					\
	.type = IIO_ALTCURRENT,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (ch),						\
	.address = AD5706R_CHAN_IDX_DITHER_ ## ch,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |		\
			      BIT(IIO_CHAN_INFO_PHASE) |			\
			      BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_FREQUENCY) |		\
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.parent = &ad5706r_channels[AD5706R_CHAN_IDX_PHY_ ## ch],	\
}

static const struct iio_chan_spec ad5706r_channels[] = {
	[AD5706R_CHAN_IDX_PHY_0] = AD5706R_PHY_CHAN(0),
	[AD5706R_CHAN_IDX_PHY_1] = AD5706R_PHY_CHAN(1),
	[AD5706R_CHAN_IDX_PHY_2] = AD5706R_PHY_CHAN(2),
	[AD5706R_CHAN_IDX_PHY_3] = AD5706R_PHY_CHAN(3),
	[AD5706R_CHAN_IDX_TOGGLE_0] = AD5706R_TOGGLE_CHAN(0),
	[AD5706R_CHAN_IDX_TOGGLE_1] = AD5706R_TOGGLE_CHAN(1),
	[AD5706R_CHAN_IDX_TOGGLE_2] = AD5706R_TOGGLE_CHAN(2),
	[AD5706R_CHAN_IDX_TOGGLE_3] = AD5706R_TOGGLE_CHAN(3),
	[AD5706R_CHAN_IDX_DITHER_0] = AD5706R_DITHER_CHAN(0),
	[AD5706R_CHAN_IDX_DITHER_1] = AD5706R_DITHER_CHAN(1),
	[AD5706R_CHAN_IDX_DITHER_2] = AD5706R_DITHER_CHAN(2),
	[AD5706R_CHAN_IDX_DITHER_3] = AD5706R_DITHER_CHAN(3),
};

static const char * const ad5706r_supply_names[] = {
	"iovdd", "avdd", "pvdd0", "pvdd1", "pvdd2", "pvdd3",
};

static int ad5706r_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5706r_state *st;
	struct gpio_desc *rst_gpio;
	struct pwm_state ldac_pwm_state;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

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

	/*
	 * Voltage reference: use external vref-supply if present, otherwise
	 * enable the internal bandgap after the mandatory power-on delay below.
	 */
	ret = devm_regulator_get_enable_optional(dev, "vref");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "Failed to get and enable vref\n");

	/* Optional hardware reset via reset-gpios: assert, hold t_RESET_PW = 20 ns, deassert */
	rst_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(rst_gpio))
		return dev_err_probe(dev, PTR_ERR(rst_gpio),
				     "Failed to get reset GPIO\n");

	if (rst_gpio) {
		gpiod_set_value_cansleep(rst_gpio, 1);
		ndelay(20);
		gpiod_set_value_cansleep(rst_gpio, 0);
	}

	/* Wait t_RESET_DELAY = 200 us after power-on before first register access */
	fsleep(200);

	if (ret == -ENODEV) {
		/* No external reference — enable internal bandgap */
		ret = regmap_write(st->regmap, AD5706R_REG_BANDGAP_CONTROL, 1);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable internal reference\n");
	}

	/* Enable async load mode: DAC output updates immediately on INPUT A write, no LDAC pulse needed */
	ret = regmap_write(st->regmap, AD5706R_REG_LDAC_SYNC_ASYNC, GENMASK(3, 0));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set async load mode\n");

	/* Put all channels in normal (non-SW-shutdown) operating mode */
	ret = regmap_write(st->regmap, AD5706R_REG_OUT_OP_MODE, GENMASK(3, 0));
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to set output operating mode\n");

	st->out_en_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(st->out_en_gpio))
		return dev_err_probe(dev, PTR_ERR(st->out_en_gpio),
				     "Failed to get enable GPIO\n");

	/* Enable OUT_EN pin control only when the pin is driven by enable-gpios */
	if (st->out_en_gpio) {
		ret = regmap_write(st->regmap, AD5706R_REG_OUT_EN, GENMASK(3, 0));
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable OUT_EN pin control\n");
	}

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
		ldac_pwm_state.period = DIV_ROUND_CLOSEST_ULL(NANO, HZ_PER_MHZ);
		ldac_pwm_state.enabled = true;
		ret = pwm_apply_might_sleep(st->ldac_pwm, &ldac_pwm_state);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to apply PWM state\n");
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
