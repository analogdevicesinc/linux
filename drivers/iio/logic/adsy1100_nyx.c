// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADSY1100 Apollo SOM "Nyx" RF front-end controller.
 *
 * GPIO-driven driver that provides a high-level IIO interface for the
 * 4 RX / 4 TX RF channels on the Nyx daughter board:
 *   - per-channel digital step attenuator (0 .. -31.5 dB, 0.5 dB step)
 *   - per-channel filter band selection (low / thru / X / Ku)
 *   - per-channel powerdown
 *   - per-RX-channel low-noise amplifier bypass
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <dt-bindings/iio/logic/adi,adsy1100-nyx.h>

#define ADSY1100_NYX_NUM_RX		4
#define ADSY1100_NYX_NUM_TX		4

#define ADSY1100_NYX_DSA_BITS		6
#define ADSY1100_NYX_DSA_MAX_X2		63	/* 31.5 dB / 0.5 dB step */

/*
 * GPIO layout in the "out" gpio array, matching the order the DT lists in
 * the out-gpios property of the Nyx node:
 *
 *   [ 0.. 7]  TX3..TX0 filter_ctrl_{0,1}        (TX descending)
 *   [ 8..15]  RX3..RX0 filter_ctrl_{0,1}        (RX descending)
 *   [16..21]  TX0 dsa[0..5]
 *   [22..27]  TX1 dsa[0..5]
 *   [28..33]  TX2 dsa[0..5]
 *   [34..39]  TX3 dsa[0..5]
 *   [40..45]  RX0 dsa[0..5]
 *   [46..51]  RX1 dsa[0..5]
 *   [52..57]  RX2 dsa[0..5]
 *   [58..63]  RX3 dsa[0..5]
 *   [64..67]  TX0..TX3 enable
 *   [68..71]  RX0..RX3 enable
 *   [72..75]  RX0..RX3 amp_bypass
 */
#define ADSY1100_NYX_GPIO_TX_FILT_BASE	0
#define ADSY1100_NYX_GPIO_RX_FILT_BASE	8
#define ADSY1100_NYX_GPIO_TX_DSA_BASE	16
#define ADSY1100_NYX_GPIO_RX_DSA_BASE	40
#define ADSY1100_NYX_GPIO_TX_EN_BASE	64
#define ADSY1100_NYX_GPIO_RX_EN_BASE	68
#define ADSY1100_NYX_GPIO_RX_AMP_BASE	72
#define ADSY1100_NYX_GPIO_COUNT		76

#define ADSY1100_NYX_FILTER_NUM_MODES	4

struct adsy1100_nyx {
	struct mutex lock;	/* serialise GPIO updates and shadow state */
	struct gpio_descs *gpios;

	u8 rx_dsa_x2[ADSY1100_NYX_NUM_RX];
	u8 tx_dsa_x2[ADSY1100_NYX_NUM_TX];
	u8 rx_filter[ADSY1100_NYX_NUM_RX];
	u8 tx_filter[ADSY1100_NYX_NUM_TX];
	bool rx_powerdown[ADSY1100_NYX_NUM_RX];
	bool tx_powerdown[ADSY1100_NYX_NUM_TX];
	bool rx_amp_bypass[ADSY1100_NYX_NUM_RX];
};

static unsigned int adsy1100_nyx_filt_base(bool tx, unsigned int ch)
{
	/* DT lists TX3..TX0 / RX3..RX0, so the per-channel base is reversed. */
	unsigned int base = tx ? ADSY1100_NYX_GPIO_TX_FILT_BASE
			       : ADSY1100_NYX_GPIO_RX_FILT_BASE;
	unsigned int n = tx ? ADSY1100_NYX_NUM_TX : ADSY1100_NYX_NUM_RX;

	return base + (n - 1 - ch) * 2;
}

static unsigned int adsy1100_nyx_dsa_base(bool tx, unsigned int ch)
{
	unsigned int base = tx ? ADSY1100_NYX_GPIO_TX_DSA_BASE
			       : ADSY1100_NYX_GPIO_RX_DSA_BASE;

	return base + ch * ADSY1100_NYX_DSA_BITS;
}

static void adsy1100_nyx_apply_dsa(struct adsy1100_nyx *st, bool tx,
				   unsigned int ch, u8 val_x2)
{
	unsigned int base = adsy1100_nyx_dsa_base(tx, ch);
	unsigned int i;

	for (i = 0; i < ADSY1100_NYX_DSA_BITS; i++)
		gpiod_set_value_cansleep(st->gpios->desc[base + i],
					 (val_x2 >> i) & 1);

	if (tx)
		st->tx_dsa_x2[ch] = val_x2;
	else
		st->rx_dsa_x2[ch] = val_x2;
}

static void adsy1100_nyx_apply_filter(struct adsy1100_nyx *st, bool tx,
				      unsigned int ch, u8 mode)
{
	unsigned int base = adsy1100_nyx_filt_base(tx, ch);

	gpiod_set_value_cansleep(st->gpios->desc[base + 0], mode & 0x1);
	gpiod_set_value_cansleep(st->gpios->desc[base + 1], (mode >> 1) & 0x1);

	if (tx)
		st->tx_filter[ch] = mode;
	else
		st->rx_filter[ch] = mode;
}

static void adsy1100_nyx_apply_enable(struct adsy1100_nyx *st, bool tx,
				      unsigned int ch, bool powerdown)
{
	unsigned int idx = (tx ? ADSY1100_NYX_GPIO_TX_EN_BASE
			       : ADSY1100_NYX_GPIO_RX_EN_BASE) + ch;

	gpiod_set_value_cansleep(st->gpios->desc[idx], powerdown ? 1 : 0);

	if (tx)
		st->tx_powerdown[ch] = powerdown;
	else
		st->rx_powerdown[ch] = powerdown;
}

static void adsy1100_nyx_apply_amp_bypass(struct adsy1100_nyx *st,
					  unsigned int ch, bool bypass)
{
	gpiod_set_value_cansleep(st->gpios->desc[ADSY1100_NYX_GPIO_RX_AMP_BASE + ch],
				 bypass ? 1 : 0);
	st->rx_amp_bypass[ch] = bypass;
}

/*
 * hardwaregain encodes the DSA setting as a (negative) gain in dB. The DSA
 * resolution is 0.5 dB, so val2 is always 0 or 500000.
 */
static int adsy1100_nyx_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);
	u8 dsa_x2;

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		dsa_x2 = chan->output ? st->tx_dsa_x2[chan->channel]
				      : st->rx_dsa_x2[chan->channel];
		*val = -(dsa_x2 / 2);
		*val2 = (dsa_x2 & 1) ? 500000 : 0;
		if (*val == 0 && (dsa_x2 & 1))
			*val2 = -500000;
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
}

static int adsy1100_nyx_write_raw(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  int val, int val2, long mask)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);
	int millidB;
	u8 val_x2;

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		/* Attenuator: only non-positive gains are accepted. */
		if (val > 0 || (val == 0 && val2 > 0))
			return -EINVAL;

		/*
		 * IIO encodes "-N.5 dB" as val=-N, val2=+500000 (the
		 * fractional part always carries the magnitude of the
		 * sub-integer portion, not its sign). "-0.5 dB" is the
		 * special case where val==0 and val2==-500000.
		 */
		if (val < 0)
			millidB = (-val) * 1000 + val2 / 1000;
		else
			millidB = (-val2) / 1000;
		if (millidB % 500)
			return -EINVAL;

		val_x2 = millidB / 500;
		if (val_x2 > ADSY1100_NYX_DSA_MAX_X2)
			return -EINVAL;

		adsy1100_nyx_apply_dsa(st, chan->output, chan->channel, val_x2);
		return 0;
	default:
		return -EINVAL;
	}
}

static int adsy1100_nyx_write_raw_get_fmt(struct iio_dev *indio_dev,
					  struct iio_chan_spec const *chan,
					  long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
}

static const char * const adsy1100_nyx_filter_modes[] = {
	[ADSY1100_NYX_FILTER_LOW_BAND] = "low_band",
	[ADSY1100_NYX_FILTER_THRU]     = "thru",
	[ADSY1100_NYX_FILTER_X_BAND]   = "x_band",
	[ADSY1100_NYX_FILTER_KU_BAND]  = "ku_band",
};

static_assert(ARRAY_SIZE(adsy1100_nyx_filter_modes) == ADSY1100_NYX_FILTER_NUM_MODES);

static int adsy1100_nyx_set_filter_mode(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					unsigned int mode)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	adsy1100_nyx_apply_filter(st, chan->output, chan->channel, mode);
	return 0;
}

static int adsy1100_nyx_get_filter_mode(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);

	return chan->output ? st->tx_filter[chan->channel]
			    : st->rx_filter[chan->channel];
}

static const struct iio_enum adsy1100_nyx_filter_enum = {
	.items = adsy1100_nyx_filter_modes,
	.num_items = ARRAY_SIZE(adsy1100_nyx_filter_modes),
	.set = adsy1100_nyx_set_filter_mode,
	.get = adsy1100_nyx_get_filter_mode,
};

static ssize_t adsy1100_nyx_powerdown_read(struct iio_dev *indio_dev,
					   uintptr_t priv,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);
	bool state = chan->output ? st->tx_powerdown[chan->channel]
				  : st->rx_powerdown[chan->channel];

	return sysfs_emit(buf, "%d\n", state);
}

static ssize_t adsy1100_nyx_powerdown_write(struct iio_dev *indio_dev,
					    uintptr_t priv,
					    const struct iio_chan_spec *chan,
					    const char *buf, size_t len)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	adsy1100_nyx_apply_enable(st, chan->output, chan->channel, state);
	return len;
}

static ssize_t adsy1100_nyx_amp_bypass_read(struct iio_dev *indio_dev,
					    uintptr_t priv,
					    const struct iio_chan_spec *chan,
					    char *buf)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", st->rx_amp_bypass[chan->channel]);
}

static ssize_t adsy1100_nyx_amp_bypass_write(struct iio_dev *indio_dev,
					     uintptr_t priv,
					     const struct iio_chan_spec *chan,
					     const char *buf, size_t len)
{
	struct adsy1100_nyx *st = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = kstrtobool(buf, &state);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	adsy1100_nyx_apply_amp_bypass(st, chan->channel, state);
	return len;
}

static const struct iio_chan_spec_ext_info adsy1100_nyx_rx_ext_info[] = {
	IIO_ENUM("filter_mode", IIO_SEPARATE, &adsy1100_nyx_filter_enum),
	IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_TYPE,
			   &adsy1100_nyx_filter_enum),
	{
		.name = "powerdown",
		.read = adsy1100_nyx_powerdown_read,
		.write = adsy1100_nyx_powerdown_write,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "bypass_amplifier_en",
		.read = adsy1100_nyx_amp_bypass_read,
		.write = adsy1100_nyx_amp_bypass_write,
		.shared = IIO_SEPARATE,
	},
	{ }
};

static const struct iio_chan_spec_ext_info adsy1100_nyx_tx_ext_info[] = {
	IIO_ENUM("filter_mode", IIO_SEPARATE, &adsy1100_nyx_filter_enum),
	IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_TYPE,
			   &adsy1100_nyx_filter_enum),
	{
		.name = "powerdown",
		.read = adsy1100_nyx_powerdown_read,
		.write = adsy1100_nyx_powerdown_write,
		.shared = IIO_SEPARATE,
	},
	{ }
};

#define ADSY1100_NYX_RX_CHAN(idx) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = (idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.ext_info = adsy1100_nyx_rx_ext_info,				\
}

#define ADSY1100_NYX_TX_CHAN(idx) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.output = 1,							\
	.channel = (idx),						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),		\
	.ext_info = adsy1100_nyx_tx_ext_info,				\
}

static const struct iio_chan_spec adsy1100_nyx_channels[] = {
	ADSY1100_NYX_RX_CHAN(0),
	ADSY1100_NYX_RX_CHAN(1),
	ADSY1100_NYX_RX_CHAN(2),
	ADSY1100_NYX_RX_CHAN(3),
	ADSY1100_NYX_TX_CHAN(0),
	ADSY1100_NYX_TX_CHAN(1),
	ADSY1100_NYX_TX_CHAN(2),
	ADSY1100_NYX_TX_CHAN(3),
};

static const struct iio_info adsy1100_nyx_iio_info = {
	.read_raw = adsy1100_nyx_read_raw,
	.write_raw = adsy1100_nyx_write_raw,
	.write_raw_get_fmt = adsy1100_nyx_write_raw_get_fmt,
};

/*
 * adi,attenuation-millidb is encoded as a positive value [0..31500] in 500
 * mdB steps. The IIO sysfs "hardwaregain" attribute exposes the same value
 * with the opposite sign.
 */
#define ADSY1100_NYX_DSA_MAX_MILLIDB	31500
#define ADSY1100_NYX_DSA_STEP_MILLIDB	500

static int adsy1100_nyx_parse_dt_channel(struct device *dev,
					 struct fwnode_handle *node,
					 struct adsy1100_nyx *st,
					 bool tx, unsigned int ch)
{
	u8 dsa_x2 = 0;
	u8 filter = ADSY1100_NYX_FILTER_THRU;
	bool powerdown;
	bool amp_enable = false;
	u32 val;

	if (!fwnode_property_read_u32(node, "adi,attenuation-millidb", &val)) {
		if (val > ADSY1100_NYX_DSA_MAX_MILLIDB ||
		    val % ADSY1100_NYX_DSA_STEP_MILLIDB)
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: adi,attenuation-millidb %u invalid (0..%u in %u mdB steps)\n",
					     node, val,
					     ADSY1100_NYX_DSA_MAX_MILLIDB,
					     ADSY1100_NYX_DSA_STEP_MILLIDB);
		dsa_x2 = val / ADSY1100_NYX_DSA_STEP_MILLIDB;
	}

	if (!fwnode_property_read_u32(node, "adi,filter-mode", &val)) {
		if (val >= ADSY1100_NYX_FILTER_NUM_MODES)
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: adi,filter-mode %u invalid\n",
					     node, val);
		filter = val;
	}

	powerdown = fwnode_property_read_bool(node, "adi,powerdown");

	if (!tx)
		amp_enable = fwnode_property_read_bool(node,
						       "adi,amplifier-enable");

	adsy1100_nyx_apply_dsa(st, tx, ch, dsa_x2);
	adsy1100_nyx_apply_filter(st, tx, ch, filter);
	adsy1100_nyx_apply_enable(st, tx, ch, powerdown);
	if (!tx)
		adsy1100_nyx_apply_amp_bypass(st, ch, !amp_enable);

	return 0;
}

static int adsy1100_nyx_parse_dt_group(struct device *dev,
				       struct adsy1100_nyx *st,
				       const char *group_name,
				       bool tx, unsigned int max_ch)
{
	int ret;

	struct fwnode_handle *group __free(fwnode_handle) =
		device_get_named_child_node(dev, group_name);
	if (!group)
		return 0;

	fwnode_for_each_child_node_scoped(group, child) {
		u32 reg;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "%pfw: missing reg property\n",
					     child);

		if (reg >= max_ch)
			return dev_err_probe(dev, -EINVAL,
					     "%pfw: reg %u out of range (max %u)\n",
					     child, reg, max_ch - 1);

		ret = adsy1100_nyx_parse_dt_channel(dev, child, st, tx, reg);
		if (ret)
			return ret;
	}

	return 0;
}

static int adsy1100_nyx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct adsy1100_nyx *st;
	unsigned int i;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->gpios = devm_gpiod_get_array(dev, "out", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpios))
		return dev_err_probe(dev, PTR_ERR(st->gpios),
				     "failed to acquire control GPIOs\n");

	if (st->gpios->ndescs != ADSY1100_NYX_GPIO_COUNT)
		return dev_err_probe(dev, -EINVAL,
				     "expected %d out-gpios, got %u\n",
				     ADSY1100_NYX_GPIO_COUNT,
				     st->gpios->ndescs);

	/*
	 * Bring the front-end into a defined state: 0 dB attenuation, THRU
	 * filter band, channels enabled (powerdown=0), RX LNAs bypassed
	 * (bypass=1). Per-channel defaults provided through the
	 * output-channels / input-channels subnodes override the built-in
	 * defaults below.
	 */
	for (i = 0; i < ADSY1100_NYX_NUM_RX; i++) {
		adsy1100_nyx_apply_dsa(st, false, i, 0);
		adsy1100_nyx_apply_filter(st, false, i,
					  ADSY1100_NYX_FILTER_THRU);
		adsy1100_nyx_apply_enable(st, false, i, false);
		adsy1100_nyx_apply_amp_bypass(st, i, true);
	}
	for (i = 0; i < ADSY1100_NYX_NUM_TX; i++) {
		adsy1100_nyx_apply_dsa(st, true, i, 0);
		adsy1100_nyx_apply_filter(st, true, i,
					  ADSY1100_NYX_FILTER_THRU);
		adsy1100_nyx_apply_enable(st, true, i, false);
	}

	ret = adsy1100_nyx_parse_dt_group(dev, st, "output-channels", true,
					  ADSY1100_NYX_NUM_TX);
	if (ret)
		return ret;

	ret = adsy1100_nyx_parse_dt_group(dev, st, "input-channels", false,
					  ADSY1100_NYX_NUM_RX);
	if (ret)
		return ret;

	indio_dev->name = "adsy1100-nyx";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adsy1100_nyx_iio_info;
	indio_dev->channels = adsy1100_nyx_channels;
	indio_dev->num_channels = ARRAY_SIZE(adsy1100_nyx_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id adsy1100_nyx_of_match[] = {
	{ .compatible = "adi,adsy1100-nyx" },
	{ }
};
MODULE_DEVICE_TABLE(of, adsy1100_nyx_of_match);

static struct platform_driver adsy1100_nyx_driver = {
	.driver = {
		.name = "adsy1100-nyx",
		.of_match_table = adsy1100_nyx_of_match,
	},
	.probe = adsy1100_nyx_probe,
};
module_platform_driver(adsy1100_nyx_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADSY1100 Nyx RF front-end controller");
MODULE_LICENSE("GPL");
