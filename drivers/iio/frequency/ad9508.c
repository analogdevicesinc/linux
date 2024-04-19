/*
 * AD9508 SPI Clock Fanout Buffer with Output Dividers and Delay Adjust
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/frequency/ad9508.h>
#include <dt-bindings/iio/frequency/ad9508.h>

#define AD9508_READ	(1 << 15)
#define AD9508_WRITE	(0 << 15)
#define AD9508_CNT(x)	(((x) - 1) << 13)
#define AD9508_ADDR(x)	((x) & 0xFFF)

#define AD9508_1B(x)	((1 << 16) | ((x) + 0))
#define AD9508_2B(x)	((2 << 16) | ((x) + 1))
#define AD9508_3B(x)	((3 << 16) | ((x) + 2))
#define AD9508_4B(x)	((4 << 16) | ((x) + 3))
#define AD9508_TRANSF_LEN(x)			((x) >> 16)

#define AD9508_SERIAL_PORT_CONFIG		AD9508_1B(0x0)
#define AD9508_SILICON_REV			AD9508_1B(0xA)
#define AD9508_PART_ID				AD9508_2B(0xB)

#define AD9508_SLEEP				AD9508_1B(0x13)
#define AD9508_SYNC_BAR				AD9508_1B(0x14)

#define AD9508_CHANNEL_OUT_DIV(ch)		AD9508_2B(0x15 + 6 * ch)
#define AD9508_CHANNEL_OUT_PHASE(ch)		AD9508_2B(0x17 + 6 * ch)
#define AD9508_CHANNEL_OUT_DRIVER(ch)		AD9508_1B(0x19 + 6 * ch)
#define AD9508_CHANNEL_OUT_CMOS(ch)		AD9508_1B(0x1A + 6 * ch)

#define AD9508_CHANNEL_OUT_DRIVER_ALL(ch)	AD9508_2B(0x19 + 6 * ch)

/* AD9508_SERIAL_PORT_CONFIG */
#define AD9508_SER_CONF_SOFT_RESET		(BIT(5) | BIT(2))
#define AD9508_SER_CONF_LSB_FIRST		(BIT(1) | BIT(6))
#define AD9508_SER_CONF_ADDR_INCREMENT		(BIT(2) | BIT(5))
#define AD9508_SER_CONF_SDO_ACTIVE		(BIT(7) | BIT(0))

/* AD9508_SYNC_BAR */
#define SYNC_BAR				BIT(0)

#define DIVIDER_SYNC_MASK	BIT(6)
#define DIVIDER_PD		BIT(7)


#define AD9508_NUM_CHAN		4

struct ad9508_outputs {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
	unsigned num;
	bool is_enabled;
};

#define to_ad9508_clk_output(_hw) container_of(_hw, struct ad9508_outputs, hw)

struct ad9508_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	struct ad9508_platform_data	*pdata;
	struct ad9508_outputs		output[AD9508_NUM_CHAN];
	struct iio_chan_spec		ad9508_channels[AD9508_NUM_CHAN];
	struct clk			*clkin;
	struct clk_onecell_data		clk_data;
	struct clk			*clks[AD9508_NUM_CHAN];

	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*sync_gpio;

	struct mutex		lock;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[4];
	} data[2] ____cacheline_aligned;
};

static int ad9508_read(struct iio_dev *indio_dev, unsigned addr)
{
	return 0;
	struct ad9508_state *st = iio_priv(indio_dev);
	int ret;
	u32 mask = ~0U >> (32 - 8 * AD9508_TRANSF_LEN(addr));

	/* We encode the register size 1..3 bytes into the register address.
	 * On transfer we get the size from the register datum, and make sure
	 * the result is properly aligned.
	 */

	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
		}, {
			.rx_buf = &st->data[1].d8[4 - AD9508_TRANSF_LEN(addr)],
			.len = AD9508_TRANSF_LEN(addr),
		},
	};

	st->data[0].d32 = cpu_to_be32(AD9508_READ |
			AD9508_CNT(AD9508_TRANSF_LEN(addr)) |
			AD9508_ADDR(addr));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		dev_err(&indio_dev->dev, "read failed (%d)", ret);
	else
		ret = be32_to_cpu(st->data[1].d32) & mask;

	return ret;
};

static int ad9508_write(struct iio_dev *indio_dev, unsigned addr, unsigned val)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
		}, {
			.tx_buf = &st->data[1].d8[4 - AD9508_TRANSF_LEN(addr)],
			.len = AD9508_TRANSF_LEN(addr),
		},
	};

	st->data[0].d32 = cpu_to_be32(AD9508_WRITE |
			AD9508_CNT(AD9508_TRANSF_LEN(addr)) |
			AD9508_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(val);

	dev_info(&indio_dev->dev, "Write 0x%x: 0x%x\n",
			AD9508_ADDR(addr) - AD9508_TRANSF_LEN(addr) + 1, val);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int ad9508_set_channel_div_enable(struct iio_dev *indio_dev, unsigned chan, bool enable)
{
	int ret = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_DRIVER(chan));
	if (ret < 0)
		return ret;

	if (enable)
		ret &= ~DIVIDER_PD;
	else
		ret |= DIVIDER_PD;

	return ad9508_write(indio_dev,
			    AD9508_CHANNEL_OUT_DRIVER(chan), ret);
}

static int ad9508_get_channel_div_enable(struct iio_dev *indio_dev, unsigned chan)
{
	int ret = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_DRIVER(chan));
	if (ret < 0)
		return ret;

	return !(ret & DIVIDER_PD);
}

static int ad9508_sync(struct iio_dev *indio_dev)
{
	int ret = ad9508_write(indio_dev,
			AD9508_SYNC_BAR, 0);
	if (ret < 0)
		return ret;

	return ad9508_write(indio_dev,
			     AD9508_SYNC_BAR, 1);
}

static ssize_t ad9508_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9508_state *st = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;

	if (!state)
		return 0;

	mutex_lock(&st->lock);
	switch ((u32)this_attr->address) {
	case 0:
		ret = ad9508_sync(indio_dev);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}



static IIO_DEVICE_ATTR(sync_dividers, S_IWUSR,
			NULL,
			ad9508_store,
			0);

static struct attribute *ad9508_attributes[] = {
	&iio_dev_attr_sync_dividers.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9508_attribute_group = {
	.attrs = ad9508_attributes,
};

static int ad9508_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	unsigned code;
	int ret, div;

	mutex_lock(&st->lock);

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = ad9508_get_channel_div_enable(indio_dev, chan->channel);
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = clk_get_rate(st->clks[chan->channel]);
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		ret = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_PHASE(chan->channel));
		div = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_DIV(chan->channel));
		div += 1;
		code = DIV_ROUND_CLOSEST(ret * 3141592, div);
		*val = code / 1000000;
		*val2 = (code % 1000000) * 10;
		mutex_unlock(&st->lock);
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
};

static int ad9508_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	int ret, tmp, code;

	mutex_lock(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad9508_set_channel_div_enable(indio_dev, chan->channel, val);
		if (ret >= 0)
			st->output[chan->channel].is_enabled = !!val;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (val <= 0) {
			ret = -EINVAL;
			goto out;
		}

		ret = clk_set_rate(st->clks[chan->channel], val);
		break;
	case IIO_CHAN_INFO_PHASE:
		ret = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_DIV(chan->channel));
		ret += 1;
		code = val * 1000000 + val2 % 1000000;
		tmp = DIV_ROUND_CLOSEST(code * ret, 3141592);
		tmp = clamp(tmp, 0, 2047);
		ret = ad9508_write(indio_dev, AD9508_CHANNEL_OUT_PHASE(chan->channel), tmp);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad9508_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		ret = ad9508_write(indio_dev, AD9508_1B(reg), writeval);
	} else {
		ret = ad9508_read(indio_dev, AD9508_1B(reg));
		if (ret < 0)
			goto out_unlock;
		*readval = ret;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info ad9508_info = {
	.read_raw = &ad9508_read_raw,
	.write_raw = &ad9508_write_raw,
	.debugfs_reg_access = &ad9508_reg_access,
	.attrs = &ad9508_attribute_group,
};

static unsigned long ad9508_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct iio_dev *indio_dev = to_ad9508_clk_output(hw)->indio_dev;
	int ret = ad9508_read(indio_dev, AD9508_CHANNEL_OUT_DIV(to_ad9508_clk_output(hw)->num));

	return parent_rate / (ret + 1);
}

static int ad9508_clk_is_enabled(struct clk_hw *hw)
{
	return to_ad9508_clk_output(hw)->is_enabled;
}

static int ad9508_clk_prepare(struct clk_hw *hw)
{
	struct iio_dev *indio_dev = to_ad9508_clk_output(hw)->indio_dev;

	return ad9508_set_channel_div_enable(indio_dev, to_ad9508_clk_output(hw)->num, 1);
}

static void ad9508_clk_unprepare(struct clk_hw *hw)
{
	struct iio_dev *indio_dev = to_ad9508_clk_output(hw)->indio_dev;

	ad9508_set_channel_div_enable(indio_dev, to_ad9508_clk_output(hw)->num, 0);
}

static long ad9508_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	unsigned long tmp;

	if (!rate)
		return 0;

	tmp = DIV_ROUND_CLOSEST(*prate, rate);
	tmp = clamp(tmp, 1UL, 1024UL);

	return *prate / tmp;
}

static int ad9508_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long prate)
{
	struct iio_dev *indio_dev = to_ad9508_clk_output(hw)->indio_dev;
	unsigned long tmp;

	tmp = DIV_ROUND_CLOSEST(prate, rate);
	tmp = clamp(tmp, 1UL, 1024UL) - 1;

	return ad9508_write(indio_dev, AD9508_CHANNEL_OUT_DIV(to_ad9508_clk_output(hw)->num), tmp);
}

static const struct clk_ops ad9508_clk_ops = {
	.recalc_rate = ad9508_clk_recalc_rate,
	.is_enabled = ad9508_clk_is_enabled,
	.prepare = ad9508_clk_prepare,
	.unprepare = ad9508_clk_unprepare,
	.set_rate = ad9508_clk_set_rate,
	.round_rate = ad9508_clk_round_rate,
};

static struct clk *ad9508_clk_register(struct iio_dev *indio_dev, unsigned num,
				bool is_enabled)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	struct device_node *of_node = st->spi->dev.of_node;
	struct clk_init_data init;
	struct ad9508_outputs *output = &st->output[num];
	struct clk *clk;
	const char *parent_name;
	char name[SPI_NAME_SIZE + 8];
	int ret;

	ret = of_property_read_string_index(of_node, "clock-output-names",
		num, &init.name);
	if (ret < 0) {
		sprintf(name, "%s_out%d", indio_dev->name, num);
		init.name = name;
	}

	init.ops = &ad9508_clk_ops;

	parent_name = of_clk_get_parent_name(of_node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	output->hw.init = &init;
	output->indio_dev = indio_dev;
	output->num = num;
	output->is_enabled = is_enabled;

	/* register the clock */
	clk = devm_clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

static int ad9508_setup(struct iio_dev *indio_dev)
{
	struct ad9508_state *st = iio_priv(indio_dev);
	struct ad9508_platform_data *pdata = st->pdata;
	struct ad9508_channel_spec *chan;
	int ret, i;

	dev_info(&indio_dev->dev, "ad9508 setup\n");

	ret = ad9508_write(indio_dev, AD9508_SERIAL_PORT_CONFIG,
			AD9508_SER_CONF_SOFT_RESET |
			((st->spi->mode & SPI_3WIRE || pdata->spi3wire) ? 0 :
			 AD9508_SER_CONF_SDO_ACTIVE));
	if (ret < 0)
		return ret;


	ret = ad9508_read(indio_dev, AD9508_PART_ID);
	if (ret < 0)
		return ret;

	if (ret != 0x0500) {
		dev_err(&st->spi->dev, "Unexpected device ID (0x%.4x)\n", ret);
		// return -ENODEV;
	}

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = AD9508_NUM_CHAN;

	for (i = 0; i < pdata->num_channels; i++) {
		chan = &pdata->channels[i];
		if (chan->channel_num < AD9508_NUM_CHAN) {
			struct clk *clk;
			unsigned mode = 0;
			dev_err(&st->spi->dev, "chan %d divider: %d\n", i, chan->channel_divider);
			dev_err(&st->spi->dev, "output_dis: %d\n", chan->output_dis);
			if (chan->output_dis)
				continue;

			// if (chan->sync_ignore_en)
				mode |= DIVIDER_SYNC_MASK;

			ret = ad9508_write(indio_dev,
					   AD9508_CHANNEL_OUT_DRIVER_ALL(chan->channel_num),
					   mode | chan->driver_mode);
			if (ret < 0)
				return ret;
			ret = ad9508_write(indio_dev,
					   AD9508_CHANNEL_OUT_DIV(chan->channel_num),
					   chan->channel_divider);
			if (ret < 0)
				return ret;
			ret = ad9508_write(indio_dev,
					   AD9508_CHANNEL_OUT_PHASE(chan->channel_num),
					   chan->divider_phase);
			if (ret < 0)
				return ret;

			st->ad9508_channels[i].type = IIO_ALTVOLTAGE;
			st->ad9508_channels[i].output = 1;
			st->ad9508_channels[i].indexed = 1;
			st->ad9508_channels[i].channel = chan->channel_num;
			st->ad9508_channels[i].extend_name =
				chan->extended_name;
			st->ad9508_channels[i].info_mask_separate =
				BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_PHASE) |
				BIT(IIO_CHAN_INFO_FREQUENCY);

			clk = ad9508_clk_register(indio_dev, chan->channel_num,
						  !chan->output_dis);
			if (IS_ERR(clk))
				return PTR_ERR(clk);
		}
	}

	of_clk_add_provider(st->spi->dev.of_node,
			    of_clk_src_onecell_get, &st->clk_data);

	return ad9508_sync(indio_dev);
}

#ifdef CONFIG_OF
static struct ad9508_platform_data *ad9508_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node, *chan_np;
	struct ad9508_platform_data *pdata;
	struct ad9508_channel_spec *chan;
	unsigned int tmp, cnt = 0;
	const char *str;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->spi3wire = of_property_read_bool(np, "adi,spi-3wire-enable");
	dev_info(dev, "ad9508 setup, spi-3wire-enable: %d", pdata->spi3wire);

	strncpy(&pdata->name[0], np->name, SPI_NAME_SIZE - 1);

	for_each_child_of_node(np, chan_np)
		cnt++;

	pdata->num_channels = cnt;
	pdata->channels = devm_kzalloc(dev, sizeof(*chan) * cnt, GFP_KERNEL);
	if (!pdata->channels)
		return NULL;

	cnt = 0;
	for_each_child_of_node(np, chan_np) {
		of_property_read_u32(chan_np, "reg",
				     &pdata->channels[cnt].channel_num);
		pdata->channels[cnt].sync_ignore_en = of_property_read_bool(
				chan_np, "adi,sync-ignore-enable");
		pdata->channels[cnt].output_dis =
			of_property_read_bool(chan_np, "adi,output-dis");

		of_property_read_u32(chan_np, "adi,driver-mode", &tmp);
		pdata->channels[cnt].driver_mode = tmp;
		of_property_read_u32(chan_np, "adi,divider-phase", &tmp);
		pdata->channels[cnt].divider_phase = tmp;
		of_property_read_u32(chan_np, "adi,channel-divider", &tmp);
		pdata->channels[cnt].channel_divider = tmp - 1;

		ret = of_property_read_string(
				chan_np, "adi,extended-name", &str);
		if (ret >= 0)
			strlcpy(pdata->channels[cnt].extended_name, str,
					sizeof(pdata->channels[cnt].extended_name));

		cnt++;
	}

	return pdata;
}
#else
static
struct ad9508_platform_data *ad9508_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ad9508_probe(struct spi_device *spi)
{
	struct ad9508_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct ad9508_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin)) {
		ret = PTR_ERR(st->clkin);
		if (ret != -ENOENT) {
			dev_err(&spi->dev, "Failed getting REFIN clock (%d)\n", ret);
			return ret;
		}
	} else {
		clk_prepare_enable(st->clkin);
	}

	if (spi->dev.of_node)
		pdata = ad9508_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	mutex_init(&st->lock);

	st->reg = devm_regulator_get_optional(&spi->dev, "vcc");
	if (IS_ERR(st->reg)) {
		if ((PTR_ERR(st->reg) != -ENODEV) && spi->dev.of_node)
			return PTR_ERR(st->reg);

		st->reg = NULL;
	} else {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	st->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (!IS_ERR(st->reset_gpio)) {
		udelay(1);
		ret = gpiod_direction_output(st->reset_gpio, 1);
	}

	mdelay(10);

	st->sync_gpio = devm_gpiod_get_optional(&spi->dev, "sync", GPIOD_OUT_HIGH);

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (pdata->name[0] != 0) ? pdata->name :
			  spi_get_device_id(spi)->name;
	indio_dev->info = &ad9508_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->ad9508_channels;
	indio_dev->num_channels = pdata->num_channels;

	ret = ad9508_setup(indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	dev_info(&spi->dev, "probed %s\n", indio_dev->name);

	return 0;

error_disable_reg:
	if (!IS_ERR_OR_NULL(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static void ad9508_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9508_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (!IS_ERR_OR_NULL(st->reg))
		regulator_disable(st->reg);
}

static const struct spi_device_id ad9508_id[] = {
	{"ad9508", 9508},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9508_id);

static struct spi_driver ad9508_driver = {
	.driver = {
		.name	= "ad9508",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9508_probe,
	.remove		= ad9508_remove,
	.id_table	= ad9508_id,
};
module_spi_driver(ad9508_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9508 CLOCKDIST");
MODULE_LICENSE("GPL v2");
