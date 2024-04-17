// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/units.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>

#define AD4134_NAME				"ad4134"

#define AD4134_IF_CONFIG_B_REG					0x01
#define AD4134_IF_CONFIG_B_SINGLE_INSTR			BIT(7)
#define AD4134_IF_CONFIG_B_MASTER_SLAVE_RD_CTRL	BIT(5)
#define AD4134_IF_CONFIG_B_DIG_IF_RESET			BIT(1)

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0b1

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_FRAME_24BIT_CRC		0b11
#define AD4134_DATA_FRAME_24BIT		    0b10
#define AD4134_DATA_FRAME_16BIT_CRC		0b01
#define AD4134_DATA_FRAME_16BIT			0b00

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_QUAD_CH_PARALLEL	0b10

#define AD4134_CHAN_DIG_FILTER_SEL_REG			0x1E
#define AD4134_CHAN_DIG_FILTER_SEL_MASK	GENMASK(7,0)
#define AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH0 GENMASK(1,0)
#define AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH1 GENMASK(3,2)
#define AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH2 GENMASK(5,4)
#define AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH3 GENMASK(7,6)

#define AD4134_SINC6_FILTER		0b01010101

#define AD4134_ODR_MIN				10
#define AD4134_ODR_MAX				1496000
#define AD4134_ODR_DEFAULT			1496000

#define AD4134_NUM_CHANNELS			8
#define AD4134_REAL_BITS			16
#define AD4134_WORD_BITS			16

#define AD4134_RESET_TIME_US			10000000

enum {
	ODR_SET_FREQ,
};

enum ad4134_regulators {
	AD4134_AVDD5_REGULATOR,
	AD4134_AVDD1V8_REGULATOR,
	AD4134_IOVDD_REGULATOR,
	AD4134_REFIN_REGULATOR,
	AD4134_NUM_REGULATORS
};

enum ad7134_flt_type {
	WIDEBAND,
	SINC6,
	SINC3,
	SINC3_REJECTION
};
static const char * const ad7134_filter_enum[] = {
	[WIDEBAND] = "WIDEBAND",
	[SINC6] = "SINC6",
	[SINC3] = "SINC3",
	[SINC3_REJECTION] = "SINC3_REJECTION",
};

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev,uintptr_t private,const struct iio_chan_spec *chan,const char *buf, size_t len);
static ssize_t ad7134_get_sync(struct iio_dev *indio_dev,uintptr_t private,const struct iio_chan_spec *chan, char *buf);
static int ad7134_set_dig_fil(struct iio_dev *dev, const struct iio_chan_spec *chan, unsigned int filter);
static int ad7134_get_dig_fil(struct iio_dev *dev, const struct iio_chan_spec *chan);
static ssize_t ad7134_ext_info_write(struct iio_dev *indio_dev,uintptr_t private, const struct iio_chan_spec *chan, const char *buf, size_t len);
static ssize_t ad7134_ext_info_read(struct iio_dev *indio_dev, uintptr_t private, const struct iio_chan_spec *chan, char *buf);

static const struct iio_enum ad7134_flt_type_iio_enum = {
	.items = ad7134_filter_enum,
	.num_items = ARRAY_SIZE(ad7134_filter_enum),
	.set = ad7134_set_dig_fil,
	.get = ad7134_get_dig_fil,
};

static struct iio_chan_spec_ext_info ad7134_ext_info[] = {

	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),

	{
	 .name = "ad7134_sync",
	 .write = ad7134_set_sync,
	 .read = ad7134_get_sync,
	 .shared = IIO_SHARED_BY_ALL,
	 },

	 {
	 .name = "odr_set_freq",
	 .read = ad7134_ext_info_read,
	 .write = ad7134_ext_info_write,
	 .shared =  IIO_SHARED_BY_ALL,
	 .private = ODR_SET_FREQ,
	},
	{ },
};

struct ad4134_state {
	struct fwnode_handle		*spi_engine_fwnode;
	struct regmap			*regmap;
	struct spi_device		*spi;
	struct spi_device		*spi_engine;
	struct pwm_device		*odr_pwm;
	struct regulator_bulk_data	regulators[AD4134_NUM_REGULATORS];

	/*
	 * Synchronize access to members the of driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex			lock;

	struct spi_message		buf_read_msg;
	struct spi_transfer		buf_read_xfer;

	unsigned int			odr;
	unsigned int            filter_type;

	unsigned long			sys_clk_rate;
	int				refin_mv;
	struct gpio_desc *cs_gpio;
};

static ssize_t ad7134_get_sync(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "enable\n");
}

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

    gpiod_set_value_cansleep(st->cs_gpio, 1);
	ret = regmap_update_bits(st->regmap, AD4134_IF_CONFIG_B_REG,
				 (AD4134_IF_CONFIG_B_DIG_IF_RESET |  AD4134_IF_CONFIG_B_SINGLE_INSTR),
				 (AD4134_IF_CONFIG_B_DIG_IF_RESET |  AD4134_IF_CONFIG_B_SINGLE_INSTR));
	if (ret)
		return ret;
	gpiod_set_value_cansleep(st->cs_gpio, 0);

	return ret ? ret : len;
}

static int ad7134_set_dig_fil(struct iio_dev *dev,
 						      const struct iio_chan_spec *chan,
		                      unsigned int filter)
{   struct ad4134_state *st = iio_priv(dev);
	int ret;
    st->filter_type = filter;
	printk("ad4134_channel: %d", chan->channel);
	printk("ad4134_filter: %d", filter);
	gpiod_set_value_cansleep(st->cs_gpio, 1);

	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
					 AD4134_CHAN_DIG_FILTER_SEL_MASK,
					 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH0, filter) |
					 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH1, filter) |
					 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH2, filter) |
					 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH3, filter));

	gpiod_set_value_cansleep(st->cs_gpio, 0);

	if (ret)
		return ret;
	return 0;
}

static int ad7134_get_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan)
{
	struct ad4134_state *st = iio_priv(dev);
	int ret;
	unsigned int readval;
    ret = regmap_read(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG, &readval);
	if (ret)
		return ret;

    printk("ad4134_channel: %d", chan->channel);
	printk("ad4134_readval: %d", readval);

		return FIELD_GET(AD4134_CHAN_DIG_FILTER_SEL_CONFIG_FRAME_MASK_CH0, readval);
}

static ssize_t ad7134_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{

	int ret = -EINVAL;
	long long val;
    struct pwm_state state;
	struct ad4134_state *st = iio_priv(indio_dev);

    mutex_lock(&st->lock);

	switch (private) {
		case ODR_SET_FREQ:
		   printk("ad4134_in_case");
			val = st->odr;
	    break;

	    default:
			ret = -EINVAL;
		}

	mutex_unlock(&st->lock);

	return sprintf(buf, "%lld\n", val);
}

static int ad4134_samp_freq_avail[] = { AD4134_ODR_MIN, 1, AD4134_ODR_MAX };

static int _ad4134_set_odr(struct ad4134_state *st, unsigned int odr)
{
	struct pwm_state state;
	int ret;

	if (!st->odr_pwm)
		return 0;

	if (odr < AD4134_ODR_MIN || odr > AD4134_ODR_MAX)
		return -EINVAL;
    
	pwm_get_state(st->odr_pwm, &state);
	printk("ad4134__ad4134_set_odr");

	/*
	 * fDIGCLK = fSYSCLK / 2
	 * tDIGCLK = 1s / fDIGCLK
	 * tODR_HIGH_TIME = 3 * tDIGCLK
	 * See datasheet page 10, Table 3. Data Interface Timing with Gated DCLK.
	 */
	state.duty_cycle = DIV_ROUND_CLOSEST_ULL(PICO * 6, st->sys_clk_rate) - 1;
	printk("ad4134__ad4134_set_odr_duty %lu ", state.duty_cycle);
	state.period = DIV_ROUND_CLOSEST_ULL(PICO, odr);
	printk("ad4134__ad4134_set_odr_period %lu ", state.period);
	state.time_unit = PWM_UNIT_PSEC;
	printk("ad4134__ad4134_set_odr_time %lu ", state.time_unit);

	ret = pwm_apply_state(st->odr_pwm, &state);
	if (ret)
		return ret;

	st->odr = odr;

	return 0;
}

static int ad4134_set_odr(struct iio_dev *indio_dev, unsigned int odr)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	if (IS_ERR(st->odr_pwm))
		return 0;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = _ad4134_set_odr(st, odr);

	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);
	printk("ad4134__ad4134_set_odr");

	return ret;
}


static ssize_t ad7134_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	int ret = -EINVAL;
	long long readin;
	struct ad4134_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);

	switch (private) {
		case ODR_SET_FREQ:
			ret = kstrtoll(buf, 10, &readin);
			if (ret)
				goto out;
			readin = clamp_t(long long, readin, 0, 1500000);
			ret = _ad4134_set_odr(st, readin);
	    break;

	    default:
			ret = -EINVAL;
		}
out:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = chan->scan_type.realbits - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = st->odr;
		mutex_unlock(&st->lock);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4134_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ad4134_samp_freq_avail;
		*type = IIO_VAL_INT;

		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int ad4134_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4134_set_odr(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad4134_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.read_avail = ad4134_read_avail,
	.write_raw = ad4134_write_raw,
	.debugfs_reg_access = ad4134_reg_access,
};

/*#define AD4134_CHANNEL(index) {						\
	.type = IIO_VOLTAGE,							 \
	.ext_info = ad7134_ext_info,						\
	.indexed = 1,							\
	.channel = (index),						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
				    BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_type_available =				\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (index),						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = AD4134_REAL_BITS,				\
		.storagebits = 32,					\
		.shift = AD4134_WORD_BITS - AD4134_REAL_BITS		\
	},								\
}
*/
#define AD4134_CHANNEL(index) {						\
	.type = IIO_VOLTAGE,								\
	.ext_info = ad7134_ext_info,						\
	.indexed = 1,							\
	.channel = (index),						\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
				    BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_type_available =				\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (index),						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,				\
		.storagebits = 32,					\
		.shift = 0		\
	},								\
}

static const struct iio_chan_spec ad4134_channels[] = {
	AD4134_CHANNEL(0),
	AD4134_CHANNEL(1),
	AD4134_CHANNEL(2),
	AD4134_CHANNEL(3),
	AD4134_CHANNEL(4),
	AD4134_CHANNEL(5),
	AD4134_CHANNEL(6),
	AD4134_CHANNEL(7),
};

static const unsigned long ad4134_channel_masks[] = {
	GENMASK(AD4134_NUM_CHANNELS - 1, 0),
	0,
};

static int ad4134_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ret = spi_engine_offload_load_msg(st->spi_engine, &st->buf_read_msg);
	if (ret)
		return ret;

	spi_engine_offload_enable(st->spi_engine, true);

	return 0;
}

static int ad4134_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi_engine, false);

	return 0;
}

static const struct iio_buffer_setup_ops ad4134_buffer_ops = {
	.postenable = ad4134_buffer_postenable,
	.predisable = ad4134_buffer_predisable,
};

static void ad4134_disable_regulators(void *data)
{
	struct ad4134_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static void ad4134_disable_clk(void *data)
{
	clk_disable_unprepare(data);
}

static void ad4134_disable_pwm(void *data)
{
	pwm_disable(data);
}

static int ad4134_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset_gpio;
	//struct gpio_desc *cs_gpio;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "sys_clk");
	printk("ad4134_clk %s", clk);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to find SYS clock\n");

	ret = clk_prepare_enable(clk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable SYS clock\n");

	ret = devm_add_action_or_reset(dev, ad4134_disable_clk, clk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add SYS clock disable action\n");

	st->sys_clk_rate = clk_get_rate(clk);
	printk("ad4134_sys_clk %lu", st->sys_clk_rate);
	if (!st->sys_clk_rate)
		return dev_err_probe(dev, -EINVAL, "Failed to get SYS clock rate\n");

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = regulator_get_voltage(st->regulators[AD4134_REFIN_REGULATOR].consumer);\
	printk("ad4134_regulator %lu", ret);
	if (ret < 0)
		return ret;

	st->refin_mv = ret / 1000;

	ret = devm_add_action_or_reset(dev, ad4134_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	printk("ad4134_reset_gpio %lu", devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH));
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to find reset GPIO\n");

	st->cs_gpio = devm_gpiod_get_optional(dev, "gpio-cs", GPIOD_OUT_LOW);
	printk("ad4134_cs_gpio %lu", st->cs_gpio);
	if (IS_ERR(st->cs_gpio))
		return dev_err_probe(dev, PTR_ERR(st->cs_gpio),
				     "Failed to find cs-gpio \n");

	fsleep(AD4134_RESET_TIME_US);

	printk("ad4134_Before_reset_gpio_0");
	gpiod_set_value_cansleep(reset_gpio, 0);
    printk("ad4134_After_reset_gpio_0");
	st->odr_pwm = devm_pwm_get(dev, "odr_pwm");
	if (IS_ERR(st->odr_pwm))
	//	return dev_err_probe(dev, PTR_ERR(st->odr_pwm),
	//			     "Failed to find ODR PWM\n");
		pr_err("%s: %d: Failed to find ODR PWM\n", __FUNCTION__, __LINE__);

	printk("ad4134_Before_initialize_ODR");
	if (!IS_ERR(st->odr_pwm)) {
		ret = _ad4134_set_odr(st, AD4134_ODR_DEFAULT);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to initialize ODR\n");

		ret = pwm_enable(st->odr_pwm);
		if (ret)
			return ret;
	printk("ad4134_After_initialize_ODR");
		ret = devm_add_action_or_reset(dev, ad4134_disable_pwm, st->odr_pwm);
		if (ret)
			return dev_err_probe(dev, ret,
					"Failed to add ODR PWM disable action\n");
	}
printk("ad4134_Before_reg_0x11");
	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    AD4134_DATA_FRAME_16BIT	));

	if (ret)
		return ret;

printk("ad4134_Before_reg_0x12");
	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_QUAD_CH_PARALLEL));
	if (ret)
		return ret;

printk("ad4134_Before_reg_0x02");
	 ret = regmap_update_bits(st->regmap, AD4134_DEVICE_CONFIG_REG,
				  AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
				  FIELD_PREP(AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
					     AD4134_POWER_MODE_HIGH_PERF));
	if (ret)
		return ret;

	printk("ad4134_Before_reg_0x1E");
	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				 AD4134_CHAN_DIG_FILTER_SEL_MASK,
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_MASK,
					    AD4134_SINC6_FILTER));

	if (ret)
		return ret;

	return 0;
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static inline int ad4134_spi_engine_compare_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	return device_match_fwnode(dev, fwnode);
}

static inline void ad4134_spi_engine_release_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	fwnode_handle_put(fwnode);
}

static int ad4134_bind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ret = component_bind_all(dev, st);
	if (ret)
		return ret;

	return iio_device_register(indio_dev);
}

static void ad4134_unbind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	iio_device_unregister(indio_dev);

	component_unbind_all(dev, NULL);
}

static const struct component_master_ops ad4134_comp_ops = {
	.bind = ad4134_bind,
	.unbind = ad4134_unbind,
};

static int ad4134_probe(struct spi_device *spi)
{
	struct component_match *match = NULL;
	struct device *dev = &spi->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct iio_dev *indio_dev;
	struct ad4134_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	printk("ad4134_probe_indio_dev %lu", indio_dev);
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);
	st->spi = spi;

	dev_set_drvdata(dev, indio_dev);

	st->regulators[AD4134_AVDD5_REGULATOR].supply = "avdd5";
	st->regulators[AD4134_AVDD1V8_REGULATOR].supply = "avdd1v8";
	st->regulators[AD4134_IOVDD_REGULATOR].supply = "iovdd";
	st->regulators[AD4134_REFIN_REGULATOR].supply = "refin";

	/*
	 * Receive buffer needs to be non-zero for the SPI engine master
	 * to mark the transfer as a read.
	 */
	st->buf_read_xfer.rx_buf = (void *)-1;
	st->buf_read_xfer.len = 1;
	st->buf_read_xfer.bits_per_word = AD4134_WORD_BITS;
	//st->buf_read_xfer.bits_per_word = 24;
	spi_message_init_with_transfers(&st->buf_read_msg,
					&st->buf_read_xfer, 1);

	st->regmap = devm_regmap_init_spi(spi, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	indio_dev->channels = ad4134_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad4134_channels);
	indio_dev->available_scan_masks = ad4134_channel_masks;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad4134_buffer_ops;
	indio_dev->info = &ad4134_info;

	ret = ad4134_setup(st);
	if (ret)
		return ret;

	ret = devm_iio_dmaengine_buffer_setup(dev, indio_dev, "rx",
					      IIO_BUFFER_DIRECTION_IN);
	if (ret) {
		indio_dev->channels = 0;
		indio_dev->num_channels = 0;
		indio_dev->available_scan_masks = 0;
		indio_dev->name = spi->dev.of_node->name;
		indio_dev->modes = 0;
		indio_dev->setup_ops = 0;
		printk("Device_name: %s",indio_dev->name);
		return devm_iio_device_register(dev, indio_dev);
	}
printk("Device_name: %s",indio_dev->name);
//		return dev_err_probe(dev, ret,
				 //    "Failed to allocate IIO DMA buffer\n");

	st->spi_engine_fwnode = fwnode_find_reference(fwnode, "adi,spi-engine", 0);
	//if (IS_ERR(st->spi_engine_fwnode))
	//	return dev_err_probe(dev, PTR_ERR(st->spi_engine_fwnode),
	//			     "Failed to find SPI engine node\n");

	component_match_add_release(dev, &match, ad4134_spi_engine_release_fwnode,
				    ad4134_spi_engine_compare_fwnode,
				    st->spi_engine_fwnode);

	return component_master_add_with_match(dev, &ad4134_comp_ops, match);
}

static void ad4134_remove(struct spi_device *spi)
{
	component_master_del(&spi->dev, &ad4134_comp_ops);
}

static const struct spi_device_id ad4134_id[] = {
	{ "ad4134", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{
		.compatible = "adi,ad4134",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_of_match);

static struct spi_driver ad4134_driver = {
	.driver = {
		.name = AD4134_NAME,
		.of_match_table = ad4134_of_match,
	},
	.probe = ad4134_probe,
	.remove = ad4134_remove,
	.id_table = ad4134_id,
};

static int ad4134_spi_engine_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct ad4134_state *st = data;

	st->spi_engine = to_spi_device(dev);

	return 0;
}

static const struct component_ops ad4134_spi_engine_ops = {
	.bind   = ad4134_spi_engine_bind,
};

static int ad4134_spi_engine_probe(struct spi_device *spi)
{
	return component_add(&spi->dev, &ad4134_spi_engine_ops);
}

static void ad4134_spi_engine_remove(struct spi_device *spi)
{
	component_del(&spi->dev, &ad4134_spi_engine_ops);
}

static const struct spi_device_id ad4134_spi_engine_id[] = {
	{ "ad4134-spi-engine", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_spi_engine_id);

static const struct of_device_id ad4134_spi_engine_of_match[] = {
	{
		.compatible = "adi,ad4134-spi-engine",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_spi_engine_of_match);

static struct spi_driver ad4134_spi_engine_driver = {
	.driver = {
		.name = "ad4134-spi-engine",
		.of_match_table = ad4134_spi_engine_of_match,
	},
	.probe = ad4134_spi_engine_probe,
	.remove = ad4134_spi_engine_remove,
	.id_table = ad4134_spi_engine_id,
};

static int __init ad4134_init(void)
{
	int ret;

	ret = spi_register_driver(&ad4134_driver);
	if (ret)
		return ret;

	ret = spi_register_driver(&ad4134_spi_engine_driver);
	if (ret) {
		spi_unregister_driver(&ad4134_driver);
		return ret;
	}

	return 0;
}
module_init(ad4134_init);

static void __exit ad4134_exit(void)
{
	spi_unregister_driver(&ad4134_spi_engine_driver);
	spi_unregister_driver(&ad4134_driver);
}
module_exit(ad4134_exit);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
