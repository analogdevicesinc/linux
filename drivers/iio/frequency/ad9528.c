/*
 * AD9528 SPI Low Jitter Clock Generator
 *
 * Copyright 2014 Analog Devices Inc.
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
#include <linux/rational.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/frequency/ad9528.h>
#include <dt-bindings/iio/frequency/ad9528.h>

#define AD9528_READ	(1 << 15)
#define AD9528_WRITE	(0 << 15)
#define AD9528_CNT(x)	(((x) - 1) << 13)
#define AD9528_ADDR(x)	((x) & 0xFFF)

#define AD9528_1B(x)	((1 << 16) | ((x) + 0))
#define AD9528_2B(x)	((2 << 16) | ((x) + 1))
#define AD9528_3B(x)	((3 << 16) | ((x) + 2))
#define AD9528_4B(x)	((4 << 16) | ((x) + 3))
#define AD9528_TRANSF_LEN(x)			((x) >> 16)

#define AD9528_SERIAL_PORT_CONFIG		AD9528_1B(0x0)
#define AD9528_SERIAL_PORT_CONFIG_B		AD9528_1B(0x1)
#define AD9528_CHIP_ID				AD9528_3B(0x3)
#define AD9528_CHIP_REVISION			AD9528_1B(0x6)
#define AD9528_IO_UPDATE			AD9528_1B(0xF)

#define AD9528_PLL1_REF_A_DIVIDER		AD9528_2B(0x100)
#define AD9528_PLL1_REF_B_DIVIDER		AD9528_2B(0x102)
#define AD9528_PLL1_FEEDBACK_DIVIDER		AD9528_2B(0x104)
#define AD9528_PLL1_CHARGE_PUMP_CTRL		AD9528_2B(0x106)
#define AD9528_PLL1_CTRL			AD9528_3B(0x108)

#define AD9528_PLL2_CHARGE_PUMP			AD9528_1B(0x200)
#define AD9528_PLL2_FEEDBACK_DIVIDER_AB		AD9528_1B(0x201)
#define AD9528_PLL2_CTRL			AD9528_1B(0x202)
#define AD9528_PLL2_VCO_CTRL			AD9528_1B(0x203)
#define AD9528_PLL2_VCO_DIVIDER			AD9528_1B(0x204)
#define AD9528_PLL2_LOOP_FILTER_CTRL		AD9528_2B(0x205)
#define AD9528_PLL2_R1_DIVIDER			AD9528_1B(0x207)
#define AD9528_PLL2_N2_DIVIDER			AD9528_1B(0x208)

#define AD9528_CHANNEL_OUTPUT(ch)		AD9528_3B(0x300 + 3 * ch)
#define AD9528_CHANNEL_SYNC			AD9528_1B(0x32A)
#define AD9528_CHANNEL_SYNC_IGNORE		AD9528_2B(0x32B)

#define AD9528_SYSREF_RESAMPLE_CTRL		AD9528_2B(0x32D)

#define AD9528_SYSREF_K_DIVIDER			AD9528_2B(0x400)
#define AD9528_SYSREF_CTRL			AD9528_2B(0x402)

#define AD9528_PD_EN				AD9528_1B(0x500)
#define AD9528_CHANNEL_PD_EN			AD9528_2B(0x501)

#define AD9528_STAT_MON0			AD9528_1B(0x505)
#define AD9528_STAT_MON1			AD9528_1B(0x506)
#define AD9528_STAT_PIN_EN			AD9528_1B(0x507)
#define AD9528_READBACK				AD9528_2B(0x508)

/* AD9528_SERIAL_PORT_CONFIG */
#define AD9528_SER_CONF_SOFT_RESET		(BIT(0) | BIT(7))
#define AD9528_SER_CONF_LSB_FIRST		(BIT(1) | BIT(6))
#define AD9528_SER_CONF_ADDR_INCREMENT		(BIT(2) | BIT(5))
#define AD9528_SER_CONF_SDO_ACTIVE		(BIT(3) | BIT(4))

/* AD9528_SERIAL_PORT_CONFIG_B */
#define AD9528_SER_CONF_READ_BUFFERED		BIT(5)
#define AD9528_SER_CONF_RESET_SANS_REGMAP	BIT(2)

/* AD9528_IO_UPDATE */
#define AD9528_IO_UPDATE_EN			BIT(0)

/* AD9528_PLL1_CHARGE_PUMP_CTRL */
#define AD9528_PLL1_CHARGE_PUMP_AUTO_TRISTATE_DIS	BIT(12)
#define AD9528_PLL1_CHARGE_PUMP_MODE_NORMAL	(3 << 8)
#define AD9528_PLL1_CHARGE_PUMP_MODE_PUMP_DOWN	(2 << 8)
#define AD9528_PLL1_CHARGE_PUMP_MODE_PUMP_UP	(1 << 8)
#define AD9528_PLL1_CHARGE_PUMP_MODE_TRISTATE	(0 << 8)
#define AD9528_PLL1_CHARGE_PUMP_TRISTATE	BIT(7)
#define AD9528_PLL1_CHARGE_PUMP_CURRENT_nA(x)	(((x) / 500) & 0x7F)

/* AD9528_PLL1_CTRL */
#define AD9528_PLL1_OSC_CTRL_FAIL_VCC_BY2_EN	BIT(19)
#define AD9528_PLL1_REF_MODE(x)			((x) << 16)
#define AD9528_PLL1_FEEDBACK_BYPASS_EN		BIT(13)
#define AD9528_PLL1_REFB_BYPASS_EN		BIT(12)
#define AD9528_PLL1_REFA_BYPASS_EN		BIT(11)
#define AD9528_PLL1_SOURCE_VCXO			BIT(10)
#define AD9528_PLL1_REFB_CMOS_NEG_INP_EN	BIT(9)
#define AD9528_PLL1_REFA_CMOS_NEG_INP_EN	BIT(8)
#define AD9528_PLL1_FREQ_DETECTOR_PD_EN		BIT(7)
#define AD9528_PLL1_REFB_DIFF_RCV_EN		BIT(6)
#define AD9528_PLL1_REFA_DIFF_RCV_EN		BIT(5)
#define AD9528_PLL1_REFB_RCV_EN			BIT(4)
#define AD9528_PLL1_REFA_RCV_EN			BIT(3)
#define AD9528_PLL1_VCXO_RCV_PD_EN		BIT(2)
#define AD9528_PLL1_OSC_IN_CMOS_NEG_INP_EN	BIT(1)
#define AD9528_PLL1_OSC_IN_DIFF_EN		BIT(0)

/* AD9528_PLL2_CHARGE_PUMP */
#define AD9528_PLL2_CHARGE_PUMP_CURRENT_nA(x)	((x) / 3500)

/* AD9528_PLL2_FEEDBACK_DIVIDER_AB */
#define AD9528_PLL2_FB_NDIV_A_CNT(x)		(((x) & 0x3) << 6)
#define AD9528_PLL2_FB_NDIV_B_CNT(x)		(((x) & 0x3F) << 0)

/* AD9528_PLL2_CTRL */
#define AD9528_PLL2_LOCK_DETECT_PWR_DOWN_EN	BIT(7)
#define AD9528_PLL2_FREQ_DOUBLER_EN		BIT(5)
#define AD9528_PLL2_CHARGE_PUMP_MODE_NORMAL	(3 << 0)
#define AD9528_PLL2_CHARGE_PUMP_MODE_PUMP_DOWN	(2 << 0)
#define AD9528_PLL2_CHARGE_PUMP_MODE_PUMP_UP	(1 << 0)
#define AD9528_PLL2_CHARGE_PUMP_MODE_TRISTATE	(0 << 0)

/* AD9528_PLL2_VCO_CTRL */
#define	AD9528_PLL2_DOUBLER_R1_EN		BIT(4)
#define AD9528_PLL2_FORCE_REFERENCE_VALID	BIT(2)
#define AD9528_PLL2_FORCE_VCO_MIDSCALE		BIT(1)
#define AD9528_PLL2_VCO_CALIBRATE		BIT(0)

/* AD9528_PLL2_VCO_DIVIDER */
#define AD9528_PLL2_VCO_DIV_M1_PWR_DOWN_EN	BIT(3)
#define AD9528_PLL2_VCO_DIV_M1(x)		(((x) & 0x7) << 0)

/* AD9528_PLL2_LOOP_FILTER_CTRL */
#define AD9528_PLL2_LOOP_FILTER_RZERO_BYPASS_EN	(1 << 8)
#define AD9528_PLL2_LOOP_FILTER_RPOLE2(x)	(((x) & 0x3) << 6)
#define AD9528_PLL2_LOOP_FILTER_RZERO(x)	(((x) & 0x7) << 3)
#define AD9528_PLL2_LOOP_FILTER_CPOLE1(x)	(((x) & 0x7) << 0)

/* AD9528_PLL2_R1_DIVIDER */
#define AD9528_PLL2_R1_DIV(x)			(((x) & 0x1F) << 0)

/* AD9528_PLL2_N2_DIVIDER */
#define AD9528_PLL2_N2_DIV(x)			((((x) - 1) & 0xFF) << 0)

/* AD9528_CHANNEL_OUTPUT */
#define AD9528_CLK_DIST_DIV(x)			((((x) - 1) & 0xFF) << 16)
#define AD9528_CLK_DIST_DIV_MASK		(0xFF << 16)
#define AD9528_CLK_DIST_DIV_REV(x)		((((x) >> 16) & 0xFF) + 1)
#define AD9528_CLK_DIST_DRIVER_MODE(x)		(((x) & 0x3) << 13)
#define AD9528_CLK_DIST_DRIVER_MODE_REV(x)	(((x) >> 13) & 0x3)
#define AD9528_CLK_DIST_DIV_PHASE(x)		(((x) & 0x3F) << 8)
#define AD9528_CLK_DIST_DIV_PHASE_MASK		(0x3F << 8)
#define AD9528_CLK_DIST_DIV_PHASE_REV(x)	(((x) >> 8) & 0x3F)
#define AD9528_CLK_DIST_CTRL(x)			(((x) & 0x7) << 5)
#define AD9528_CLK_DIST_CTRL_MASK		(0x7 << 5)
#define AD9528_CLK_DIST_CTRL_REV(x)		(((x) >> 5) & 0x7)

#if 0
/* Leftovers */
#define AD9528_CLK_DIST_INV_DIV_OUTPUT_EN	(1 << 7)
#endif

/* AD9528_CHANNEL_SYNC */
#define AD9528_CHANNEL_SYNC_SET			BIT(0)

/* AD9528_CHANNEL_SYNC_IGNORE */
#define AD9528_CHANNEL_IGNORE_MASK(x)		(((x) & 0x3FFF) << 0)
#define AD9528_CHANNEL_IGNORE_MASK_REV(x)	(((x) >> 0) & 0x3FFF)

/* AD9528_SYSREF_K_DIVIDER */
#define AD9528_SYSREF_K_DIV(x)			(((x) & 0xFFFF) << 0)

/* AD9528_SYSREF_CTRL */
#define AD9528_SYSREF_SOURCE(x)			(((x) & 0x3) << 14)
#define AD9528_SYSREF_PATTERN_MODE(x)		(((x) & 0x3) << 12)
#define AD9528_SYSREF_NSHOT_MODE(x)		(((x) & 0x7) << 9)
#define AD9528_SYSREF_PATTERN_REQ		BIT(8)
#define AD9528_SYSREF_REQUEST_BY_PIN		BIT(7)
#define AD9528_SYSREF_PATTERN_TRIGGER_CTRL(x)	(((x) & 0x3) << 5)
#define AD9528_SYSREF_RESAMPLER_CLK_SRC_PLL1	BIT(4)
#define AD9528_SYSREF_PATTERN_CLK_SRC_PLL1	BIT(3)
#define AD9528_SYSREF_TEST_MODE(x)		(((x) & 0x3) << 1)
#define AD9528_SYSREF_RESET			BIT(0)

/* AD9528_PD_EN */
#define AD9528_PD_BIAS				BIT(4)
#define AD9528_PD_PLL2				BIT(3)
#define AD9528_PD_PLL1				BIT(2)
#define AD9528_PD_OUT_CLOCKS			BIT(1)
#define AD9528_PD_CHIP				BIT(0)

/* AD9528_CHANNEL_PD_EN */
#define AD9528_CHANNEL_PD_MASK(x)		(((x) & 0x3FFF) << 0)
#define AD9528_CHANNEL_PD_MASK_REV(x)		(((x) >> 0) & 0x3FFF)


/* AD9528_READBACK */
#define AD9528_IS_CALIBRATING			BIT(8)
#define	AD9528_PLL2_OK				BIT(7)
#define	AD9528_PLL1_OK				BIT(6)
#define AD9528_VCXO_OK				BIT(5)
#define AD9528_REFA_REFB_NOK			BIT(4)
#define AD9528_REFB_OK				BIT(3)
#define AD9528_REFA_OK				BIT(2)
#define AD9528_PLL2_LOCKED			BIT(1)
#define AD9528_PLL1_LOCKED			BIT(0)

/* AD9528_STAT_PIN_EN */
#define AD9528_STAT0_PIN_EN			BIT(2)
#define AD9528_STAT1_PIN_EN			BIT(3)
#define AD9528_STAT0_DIV_EN			BIT(1)
#define AD9528_STAT1_DIV_EN			BIT(0)

#define AD9528_NUM_CHAN					14

#define AD9528_SPI_MAGIC				0x00FF05

/* Helpers to avoid excess line breaks */
#define AD_IFE(_pde, _a, _b) ((pdata->_pde) ? _a : _b)
#define AD_IF(_pde, _a) AD_IFE(_pde, _a, 0)

/* In kHz */
#define AD9528_VCO_FREQ_MIN 3450000
#define AD9528_VCO_FREQ_MAX 4025000

enum {
	AD9528_STAT_PLL1_LD,
	AD9528_STAT_PLL2_LD,
	AD9528_STAT_REFA,
	AD9528_STAT_REFB,
	AD9528_STAT_REFAB_MISSING,
	AD9528_STAT_VCXO,
	AD9528_STAT_PLL1_FB_CLK,
	AD9528_STAT_PLL2_FB_CLK,
	AD9528_SYNC,
};

enum ad9528_output_source {
	AD9528_VCO,
	AD9528_VCXO,
	AD9528_SYSREF,
	AD9528_NUM_CLK_SRC,
};

struct ad9528_outputs {
	struct clk_hw hw;
	struct iio_dev *indio_dev;
	unsigned num;
	enum ad9528_output_source source;
	bool is_enabled;
};

#define to_ad9528_clk_output(_hw) container_of(_hw, struct ad9528_outputs, hw)

struct ad9528_state {
	struct spi_device		*spi;
	struct regulator		*reg;
	struct ad9528_platform_data	*pdata;
	struct ad9528_outputs		output[AD9528_NUM_CHAN];
	struct iio_chan_spec		ad9528_channels[AD9528_NUM_CHAN];
	struct clk_onecell_data		clk_data;
	struct clk			*clks[AD9528_NUM_CHAN];
	struct gpio_desc			*reset_gpio;

	unsigned long		vco_out_freq[AD9528_NUM_CLK_SRC];
	unsigned long		sysref_src_pll2;

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

static int ad9528_read(struct iio_dev *indio_dev, unsigned addr)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	int ret;
	u32 mask = ~0U >> (32 - 8 * AD9528_TRANSF_LEN(addr));

	/* We encode the register size 1..3 bytes into the register address.
	 * On transfer we get the size from the register datum, and make sure
	 * the result is properly aligned.
	 */

	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
		}, {
			.rx_buf = &st->data[1].d8[4 - AD9528_TRANSF_LEN(addr)],
			.len = AD9528_TRANSF_LEN(addr),
		},
	};

	st->data[0].d32 = cpu_to_be32(AD9528_READ |
			AD9528_CNT(AD9528_TRANSF_LEN(addr)) |
			AD9528_ADDR(addr));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		dev_err(&indio_dev->dev, "read failed (%d)", ret);
	else
		ret = be32_to_cpu(st->data[1].d32) & mask;

	return ret;
};

static int ad9528_write(struct iio_dev *indio_dev, unsigned addr, unsigned val)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[2],
			.len = 2,
		}, {
			.tx_buf = &st->data[1].d8[4 - AD9528_TRANSF_LEN(addr)],
			.len = AD9528_TRANSF_LEN(addr),
		},
	};

	st->data[0].d32 = cpu_to_be32(AD9528_WRITE |
			AD9528_CNT(AD9528_TRANSF_LEN(addr)) |
			AD9528_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(val);

	dev_dbg(&indio_dev->dev, "Write 0x%x: 0x%x\n",
			AD9528_ADDR(addr) - AD9528_TRANSF_LEN(addr) + 1, val);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

	if (ret < 0)
		dev_err(&indio_dev->dev, "write failed (%d)", ret);

	return ret;
}

static int ad9528_poll(struct iio_dev *indio_dev,
		unsigned addr, unsigned mask, unsigned data)
{
	unsigned timeout = 100;

	while (((ad9528_read(indio_dev, addr) & mask) != data)
			&& --timeout)
		msleep(1);

	return timeout ? 0 : -ETIMEDOUT;
}

static int ad9528_io_update(struct iio_dev *indio_dev)
{
	return ad9528_write(indio_dev, AD9528_IO_UPDATE, AD9528_IO_UPDATE_EN);
}

static int ad9528_sync(struct iio_dev *indio_dev)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct ad9528_platform_data *pdata = st->pdata;
	int ret = ad9528_write(indio_dev,
			AD9528_CHANNEL_SYNC, AD9528_CHANNEL_SYNC_SET);
	if (ret < 0)
		return ret;

	ret = ad9528_io_update(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_CHANNEL_SYNC, 0);
	if (ret < 0)
		return ret;

	ret = ad9528_io_update(indio_dev);
	if (ret < 0)
		return ret;

	return ad9528_poll(indio_dev, AD9528_READBACK,
			AD9528_VCXO_OK |
			   AD_IFE(pll2_bypass_en, 0, AD9528_PLL2_LOCKED),
			AD9528_VCXO_OK |
			   AD_IFE(pll2_bypass_en, 0, AD9528_PLL2_LOCKED));
}

static ssize_t ad9528_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9528_state *st = iio_priv(indio_dev);
	bool state;
	int ret;

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;

	if (!state)
		return len;

	mutex_lock(&st->lock);
	switch ((u32)this_attr->address) {
	case AD9528_SYNC:
		ret = ad9528_sync(indio_dev);
		break;
	default:
		ret = -ENODEV;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t ad9528_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9528_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = ad9528_read(indio_dev, AD9528_READBACK);
	if (ret >= 0) {
		ret = sprintf(buf, "%d\n", !!(ret & (1 <<
			(u32)this_attr->address)));
	}
	mutex_unlock(&st->lock);

	return ret;
}

static IIO_DEVICE_ATTR(pll1_locked, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_PLL1_LD);

static IIO_DEVICE_ATTR(pll2_locked, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_PLL2_LD);

static IIO_DEVICE_ATTR(pll1_reference_clk_a_present, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_REFA);

static IIO_DEVICE_ATTR(pll1_reference_clk_b_present, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_REFB);

static IIO_DEVICE_ATTR(pll1_reference_clk_ab_missing, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_REFAB_MISSING);

static IIO_DEVICE_ATTR(vcxo_clk_present, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_VCXO);

static IIO_DEVICE_ATTR(pll1_feedback_clk_present, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_PLL1_FB_CLK);

static IIO_DEVICE_ATTR(pll2_feedback_clk_present, S_IRUGO,
			ad9528_show,
			NULL,
			AD9528_STAT_PLL2_FB_CLK);

static IIO_DEVICE_ATTR(sync_dividers, S_IWUSR,
			NULL,
			ad9528_store,
			AD9528_SYNC);

static struct attribute *ad9528_attributes[] = {
	&iio_dev_attr_sync_dividers.dev_attr.attr,
	&iio_dev_attr_pll2_feedback_clk_present.dev_attr.attr,
	&iio_dev_attr_pll1_feedback_clk_present.dev_attr.attr,
	&iio_dev_attr_pll1_reference_clk_a_present.dev_attr.attr,
	&iio_dev_attr_pll1_reference_clk_b_present.dev_attr.attr,
	&iio_dev_attr_pll1_reference_clk_ab_missing.dev_attr.attr,
	&iio_dev_attr_vcxo_clk_present.dev_attr.attr,
	&iio_dev_attr_pll1_locked.dev_attr.attr,
	&iio_dev_attr_pll2_locked.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9528_attribute_group = {
	.attrs = ad9528_attributes,
};

static unsigned int ad9528_attr_to_addr(const struct iio_chan_spec *chan,
	unsigned int attr)
{
	if (attr == IIO_CHAN_INFO_RAW)
		return AD9528_CHANNEL_PD_EN;
	else
		return AD9528_CHANNEL_OUTPUT(chan->channel);
}

static int ad9528_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct ad9528_outputs *output = &st->output[chan->channel];
	unsigned code;
	int ret;

	mutex_lock(&st->lock);
	ret = ad9528_read(indio_dev, ad9528_attr_to_addr(chan, m));
	mutex_unlock(&st->lock);

	if (ret < 0)
		return ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = !(AD9528_CHANNEL_PD_MASK_REV(ret) & BIT(chan->channel));
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		*val = st->vco_out_freq[output->source];
		if (output->source != AD9528_SYSREF)
			*val /= AD9528_CLK_DIST_DIV_REV(ret);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		code = (AD9528_CLK_DIST_DIV_PHASE_REV(ret) * 3141592) /
			AD9528_CLK_DIST_DIV_REV(ret);
		*val = code / 1000000;
		*val2 = code % 1000000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
};

static int ad9528_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct ad9528_outputs *output = &st->output[chan->channel];
	unsigned int addr = ad9528_attr_to_addr(chan, mask);
	int ret, tmp, code;
	int reg_val;

	mutex_lock(&st->lock);

	reg_val = ad9528_read(indio_dev, addr);
	if (reg_val < 0) {
		ret = reg_val;
		goto out;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val)
			reg_val &= ~BIT(chan->channel);
		else
			reg_val |= BIT(chan->channel);

		output->is_enabled = !!val;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (val <= 0) {
			ret = -EINVAL;
			goto out;
		}

		if (output->source == AD9528_SYSREF) {
			tmp = DIV_ROUND_CLOSEST(st->sysref_src_pll2, val);
			tmp = clamp_t(unsigned long, tmp, 1UL, 65535UL);

			ret = ad9528_write(indio_dev, AD9528_SYSREF_K_DIVIDER,
					   AD9528_SYSREF_K_DIV(tmp));

			if (!ret)
				st->vco_out_freq[AD9528_SYSREF] =
					DIV_ROUND_CLOSEST(st->sysref_src_pll2,
							  tmp);

			ad9528_io_update(indio_dev);
			goto out;
		}

		tmp = DIV_ROUND_CLOSEST(st->vco_out_freq[output->source], val);
		tmp = clamp(tmp, 1, 256);

		reg_val &= ~(AD9528_CLK_DIST_CTRL_MASK | AD9528_CLK_DIST_DIV_MASK);
		reg_val |= AD9528_CLK_DIST_DIV(tmp);
		reg_val |= AD9528_CLK_DIST_CTRL(output->source);
		break;
	case IIO_CHAN_INFO_PHASE:
		code = val * 1000000 + val2 % 1000000;
		tmp = (code * AD9528_CLK_DIST_DIV_REV(reg_val)) / 3141592;
		tmp = clamp(tmp, 0, 63);
		reg_val &= ~AD9528_CLK_DIST_DIV_PHASE_MASK;
		reg_val |= AD9528_CLK_DIST_DIV_PHASE(tmp);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	ret = ad9528_write(indio_dev, addr, reg_val);
	if (ret < 0)
		goto out;

	ad9528_io_update(indio_dev);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad9528_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		ret = ad9528_write(indio_dev, AD9528_1B(reg), writeval);
		ad9528_io_update(indio_dev);
	} else {
		ret = ad9528_read(indio_dev, AD9528_1B(reg));
		if (ret < 0)
			goto out_unlock;
		*readval = ret;
		ret = 0;
	}

out_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info ad9528_info = {
	.read_raw = &ad9528_read_raw,
	.write_raw = &ad9528_write_raw,
	.debugfs_reg_access = &ad9528_reg_access,
	.attrs = &ad9528_attribute_group,
};

static bool ad9528_pll2_valid_calib_div(unsigned int div)
{
	if (div < 16 || div > 255)
		return false;

	switch (div) {
	case 18:
	case 19:
	case 23:
	case 27:
		return false;
	}

	return true;
}

static int ad9528_calc_dividers(unsigned int vcxo_freq, unsigned int m1_freq,
	struct ad9528_platform_data *pdata)
{
	unsigned long n2[2], r1[2];
	unsigned int fpfd;
	unsigned int m1;

	m1_freq /= 1000;
	vcxo_freq /= 1000;

	for (m1 = 3; m1 <= 5; m1++) {
		if (AD9528_VCO_FREQ_MIN <= m1_freq * m1 &&
		    AD9528_VCO_FREQ_MAX >= m1_freq * m1)
			break;
	}

	if (m1 == 6)
		return -EINVAL;

	rational_best_approximation(m1_freq, vcxo_freq,
		255 / m1, 31, &n2[0], &r1[0]);

	pdata->pll2_freq_doubler_en = false;

	if (m1_freq != vcxo_freq * n2[0] / r1[0]) {
		rational_best_approximation(m1_freq, vcxo_freq*2,
			255 / m1, 31, &n2[1], &r1[1]);

		if (abs((int)m1_freq - (int)(vcxo_freq * n2[0] / r1[0])) >
			abs((int)m1_freq - (int)(vcxo_freq * 2 * n2[1] / r1[1]))) {
			n2[0] = n2[1];
			r1[0] = r1[1];
			pdata->pll2_freq_doubler_en = true;
		}
	}

	fpfd = vcxo_freq * (pdata->pll2_freq_doubler_en ? 2 : 1) / r1[0];

	while (fpfd > 275000 || !ad9528_pll2_valid_calib_div(n2[0] * m1)) {
		fpfd /= 2;
		n2[0] *= 2;
		r1[0] *= 2;
	}

	pdata->pll2_r1_div = r1[0];
	pdata->pll2_n2_div = n2[0];
	pdata->pll2_vco_div_m1 = m1;

	return 0;
}

static long ad9528_get_clk_attr(struct clk_hw *hw, long mask)
{
	struct iio_dev *indio_dev = to_ad9528_clk_output(hw)->indio_dev;
	int val, ret;
	struct iio_chan_spec chan;

	chan.channel = to_ad9528_clk_output(hw)->num;

	ret = ad9528_read_raw(indio_dev, &chan, &val, NULL, mask);

	if (ret == IIO_VAL_INT)
		return val;

	return ret;
}

static unsigned long ad9528_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return ad9528_get_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY);
}

static int ad9528_clk_is_enabled(struct clk_hw *hw)
{
	return to_ad9528_clk_output(hw)->is_enabled;
}

static long ad9528_set_clk_attr(struct clk_hw *hw, long mask, unsigned long val)
{
	struct iio_dev *indio_dev = to_ad9528_clk_output(hw)->indio_dev;
	struct iio_chan_spec chan;

	chan.channel = to_ad9528_clk_output(hw)->num;

	return ad9528_write_raw(indio_dev, &chan, val, 0, mask);
}

static int ad9528_clk_prepare(struct clk_hw *hw)
{
	return ad9528_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 1);
}

static void ad9528_clk_unprepare(struct clk_hw *hw)
{
	ad9528_set_clk_attr(hw, IIO_CHAN_INFO_RAW, 0);
}

static long ad9528_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct ad9528_outputs *output = to_ad9528_clk_output(hw);
	struct iio_dev *indio_dev = output->indio_dev;
	struct ad9528_state *st = iio_priv(indio_dev);
	unsigned long freq, div;

	if (!rate)
		return 0;

	if (output->source == AD9528_SYSREF) {
		freq = st->sysref_src_pll2;
		div = DIV_ROUND_CLOSEST(freq, rate);
		div = clamp_t(unsigned long, div, 1UL, 65535UL);
	} else {
		freq = st->vco_out_freq[output->source];
		div = DIV_ROUND_CLOSEST(freq, rate);
		div = clamp(div, 1UL, 256UL);
	}

	return DIV_ROUND_CLOSEST(freq, div);
}

static int ad9528_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long prate)
{
	return ad9528_set_clk_attr(hw, IIO_CHAN_INFO_FREQUENCY, rate);
}

static const struct clk_ops ad9528_clk_ops = {
	.recalc_rate = ad9528_clk_recalc_rate,
	.is_enabled = ad9528_clk_is_enabled,
	.prepare = ad9528_clk_prepare,
	.unprepare = ad9528_clk_unprepare,
	.set_rate = ad9528_clk_set_rate,
	.round_rate = ad9528_clk_round_rate,
};

static struct clk *ad9528_clk_register(struct iio_dev *indio_dev, unsigned num,
				bool is_enabled, bool have_clk_out_name)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	struct ad9528_outputs *output = &st->output[num];
	struct clk *clk;
	const char *name = NULL;
	char namebuf[SPI_NAME_SIZE + 8];

	if (have_clk_out_name) {
		struct device_node *np = st->spi->dev.of_node;

		of_property_read_string_index(np, "clock-output-names",
					      num, &name);
		if (!name || !strlen(name))
			have_clk_out_name = false;
	}

	if (!have_clk_out_name) {
		snprintf(namebuf, sizeof(namebuf), "%s_out%d",
			 indio_dev->name, num);
		name = namebuf;
	}

	init.name = name;
	init.ops = &ad9528_clk_ops;

	init.num_parents = 0;
	init.flags = CLK_GET_RATE_NOCACHE;
	output->hw.init = &init;
	output->indio_dev = indio_dev;
	output->num = num;
	output->is_enabled = is_enabled;

	/* register the clock */
	clk = clk_register(&st->spi->dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

static int ad9528_clks_register(struct iio_dev *indio_dev)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct ad9528_platform_data *pdata = st->pdata;
	struct device_node *np = st->spi->dev.of_node;
	struct ad9528_channel_spec *chan;
	unsigned int i, count;
	int ret;

	count = of_property_count_strings(np, "clock-output-names");

	for (i = 0; i < pdata->num_channels; i++) {
		struct clk *clk;

		chan = &pdata->channels[i];
		if (chan->channel_num >= AD9528_NUM_CHAN || chan->output_dis)
			continue;

		clk = ad9528_clk_register(indio_dev, chan->channel_num,
					  !chan->output_dis,
					  count == AD9528_NUM_CHAN);
		if (IS_ERR(clk))
			return PTR_ERR(clk);
	}

	ret = of_clk_add_provider(np, of_clk_src_onecell_get, &st->clk_data);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9528_setup(struct iio_dev *indio_dev)
{
	struct ad9528_state *st = iio_priv(indio_dev);
	struct ad9528_platform_data *pdata = st->pdata;
	struct ad9528_channel_spec *chan;
	unsigned long active_mask = 0, ignoresync_mask = 0;
	unsigned vco_freq, vco_ctrl = 0, sysref_ctrl, stat_en_mask = 0;
	unsigned int pll2_ndiv, pll2_ndiv_a_cnt, pll2_ndiv_b_cnt;
	int ret, i;

	ret = ad9528_write(indio_dev, AD9528_SERIAL_PORT_CONFIG,
			AD9528_SER_CONF_SOFT_RESET |
			((st->spi->mode & SPI_3WIRE || pdata->spi3wire) ? 0 :
			 AD9528_SER_CONF_SDO_ACTIVE));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_SERIAL_PORT_CONFIG_B,
			AD9528_SER_CONF_READ_BUFFERED);
	if (ret < 0)
		return ret;

	ret = ad9528_io_update(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad9528_read(indio_dev, AD9528_CHIP_ID);
	if (ret < 0)
		return ret;

	if ((ret & 0xFFFFFF) != AD9528_SPI_MAGIC) {
		dev_err(&indio_dev->dev,
				"SPI Read Verify failed (0x%X)\n", ret);
		return -EIO;
	}

	/*
	 * PLL1 Setup
	 */
	ret = ad9528_write(indio_dev, AD9528_PLL1_REF_A_DIVIDER,
		pdata->refa_r_div);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL1_REF_B_DIVIDER,
		pdata->refb_r_div);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL1_FEEDBACK_DIVIDER,
		pdata->pll1_feedback_div);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL1_CHARGE_PUMP_CTRL,
		AD_IFE(pll1_bypass_en, AD9528_PLL1_CHARGE_PUMP_TRISTATE,
		AD9528_PLL1_CHARGE_PUMP_CURRENT_nA(pdata->
			pll1_charge_pump_current_nA) |
		AD9528_PLL1_CHARGE_PUMP_MODE_NORMAL |
		AD9528_PLL1_CHARGE_PUMP_AUTO_TRISTATE_DIS));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL1_CTRL,
		AD_IFE(pll1_bypass_en,
		AD_IF(osc_in_diff_en, AD9528_PLL1_OSC_IN_DIFF_EN) |
		AD_IF(osc_in_cmos_neg_inp_en,
		      AD9528_PLL1_OSC_IN_CMOS_NEG_INP_EN) |
		AD9528_PLL1_REFB_BYPASS_EN | AD9528_PLL1_REFA_BYPASS_EN |
		AD9528_PLL1_FEEDBACK_BYPASS_EN,
		AD_IF(refa_en, AD9528_PLL1_REFA_RCV_EN) |
		AD_IF(refb_en, AD9528_PLL1_REFB_RCV_EN) |
		AD_IF(osc_in_diff_en, AD9528_PLL1_OSC_IN_DIFF_EN) |
		AD_IF(osc_in_cmos_neg_inp_en,
		      AD9528_PLL1_OSC_IN_CMOS_NEG_INP_EN) |
		AD_IF(refa_diff_rcv_en, AD9528_PLL1_REFA_DIFF_RCV_EN) |
		AD_IF(refb_diff_rcv_en, AD9528_PLL1_REFB_DIFF_RCV_EN)) |
		AD_IF(refa_cmos_neg_inp_en, AD9528_PLL1_REFA_CMOS_NEG_INP_EN) |
		AD_IF(refb_cmos_neg_inp_en, AD9528_PLL1_REFB_CMOS_NEG_INP_EN) |
		AD_IF(pll1_feedback_src_vcxo, AD9528_PLL1_SOURCE_VCXO) |
		AD9528_PLL1_REF_MODE(pdata->ref_mode) |
		AD9528_PLL1_OSC_CTRL_FAIL_VCC_BY2_EN);
	if (ret < 0)
		return ret;

	/*
	 * PLL2 Setup
	 */

	if (pdata->pll2_bypass_en) {
		ret = ad9528_write(indio_dev, AD9528_PLL2_CTRL,
				   AD9528_PLL2_CHARGE_PUMP_MODE_TRISTATE);
		if (ret < 0)
			return ret;

		ret = ad9528_write(indio_dev, AD9528_SYSREF_RESAMPLE_CTRL, BIT(0));
		if (ret < 0)
			return ret;

		pdata->sysref_src = SYSREF_SRC_EXTERNAL;

		st->vco_out_freq[AD9528_VCO] = pdata->vcxo_freq;
		st->vco_out_freq[AD9528_VCXO] = pdata->vcxo_freq;
		st->vco_out_freq[AD9528_SYSREF] = pdata->vcxo_freq;

		goto pll2_bypassed;
	}


	pll2_ndiv = pdata->pll2_vco_div_m1 * pdata->pll2_n2_div;
	if (!ad9528_pll2_valid_calib_div(pll2_ndiv)) {
		dev_err(&st->spi->dev,
			"Feedback calibration divider value (%d) out of range\n",
			pll2_ndiv);
		return -EINVAL;
	}

	pll2_ndiv_a_cnt = pll2_ndiv % 4;
	pll2_ndiv_b_cnt = pll2_ndiv / 4;

	ret = ad9528_write(indio_dev, AD9528_PLL2_CHARGE_PUMP,
		AD9528_PLL2_CHARGE_PUMP_CURRENT_nA(pdata->
			pll2_charge_pump_current_nA));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL2_FEEDBACK_DIVIDER_AB,
		AD9528_PLL2_FB_NDIV_A_CNT(pll2_ndiv_a_cnt) |
		AD9528_PLL2_FB_NDIV_B_CNT(pll2_ndiv_b_cnt));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL2_CTRL,
		AD9528_PLL2_CHARGE_PUMP_MODE_NORMAL |
		AD_IF(pll2_freq_doubler_en, AD9528_PLL2_FREQ_DOUBLER_EN));
	if (ret < 0)
		return ret;


	vco_freq = div_u64((unsigned long long)pdata->vcxo_freq *
			   (pdata->pll2_freq_doubler_en ? 2 : 1) * pll2_ndiv,
			    pdata->pll2_r1_div);

	vco_ctrl = AD_IF(pll2_freq_doubler_en || pdata->pll2_r1_div != 1,
				AD9528_PLL2_DOUBLER_R1_EN);
	ret = ad9528_write(indio_dev, AD9528_PLL2_VCO_CTRL, vco_ctrl);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL2_VCO_DIVIDER,
		AD9528_PLL2_VCO_DIV_M1(pdata->pll2_vco_div_m1) |
		AD_IFE(pll2_vco_div_m1, 0,
		       AD9528_PLL2_VCO_DIV_M1_PWR_DOWN_EN));
	if (ret < 0)
		return ret;

	if (pdata->pll2_vco_div_m1)
		st->vco_out_freq[AD9528_VCO] =
			vco_freq / pdata->pll2_vco_div_m1;
	else
		st->vco_out_freq[AD9528_VCO] = vco_freq;

	st->vco_out_freq[AD9528_VCXO] = pdata->vcxo_freq;

	st->sysref_src_pll2 = vco_freq / (pll2_ndiv * 2);

	st->vco_out_freq[AD9528_SYSREF] = st->sysref_src_pll2 /
					  pdata->sysref_k_div;

	ret = ad9528_write(indio_dev, AD9528_PLL2_R1_DIVIDER,
		AD9528_PLL2_R1_DIV(pdata->pll2_r1_div));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL2_N2_DIVIDER,
		AD9528_PLL2_N2_DIV(pdata->pll2_n2_div));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PLL2_LOOP_FILTER_CTRL,
		AD9528_PLL2_LOOP_FILTER_CPOLE1(pdata->cpole1) |
		AD9528_PLL2_LOOP_FILTER_RZERO(pdata->rzero) |
		AD9528_PLL2_LOOP_FILTER_RPOLE2(pdata->rpole2) |
		AD_IF(rzero_bypass_en,
		      AD9528_PLL2_LOOP_FILTER_RZERO_BYPASS_EN));
	if (ret < 0)
		return ret;

pll2_bypassed:
	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = AD9528_NUM_CHAN;

	for (i = 0; i < pdata->num_channels; i++) {
		chan = &pdata->channels[i];
		if (chan->channel_num >= AD9528_NUM_CHAN)
			continue;
		if (chan->output_dis)
			continue;

		__set_bit(chan->channel_num, &active_mask);
		if (chan->sync_ignore_en)
			__set_bit(chan->channel_num, &ignoresync_mask);
		ret = ad9528_write(indio_dev,
			AD9528_CHANNEL_OUTPUT(chan->channel_num),
			AD9528_CLK_DIST_DRIVER_MODE(chan->driver_mode) |
			AD9528_CLK_DIST_DIV(chan->channel_divider) |
			AD9528_CLK_DIST_DIV_PHASE(chan->divider_phase) |
			AD9528_CLK_DIST_CTRL(chan->signal_source));
		if (ret < 0)
			return ret;

		st->ad9528_channels[i].type = IIO_ALTVOLTAGE;
		st->ad9528_channels[i].output = 1;
		st->ad9528_channels[i].indexed = 1;
		st->ad9528_channels[i].channel = chan->channel_num;
		st->ad9528_channels[i].extend_name =
			chan->extended_name;
		st->ad9528_channels[i].info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_PHASE) |
			BIT(IIO_CHAN_INFO_FREQUENCY);

		switch (chan->signal_source) {
		case SOURCE_VCXO:
		case SOURCE_VCXO_INV:
			st->output[chan->channel_num].source = AD9528_VCXO;
			break;
		case SOURCE_SYSREF_VCO:
		case SOURCE_SYSREF_VCXO:
		case SOURCE_SYSREF_VCXO_INV:
			st->output[chan->channel_num].source = AD9528_SYSREF;
			break;
		default:
			st->output[chan->channel_num].source = AD9528_VCO;
			break;
		}
	}

	ret = ad9528_write(indio_dev, AD9528_CHANNEL_PD_EN,
			AD9528_CHANNEL_PD_MASK(~active_mask));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_CHANNEL_SYNC_IGNORE,
			AD9528_CHANNEL_IGNORE_MASK(ignoresync_mask));
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_SYSREF_K_DIVIDER,
			AD9528_SYSREF_K_DIV(pdata->sysref_k_div));
	if (ret < 0)
		return ret;

	sysref_ctrl = AD9528_SYSREF_PATTERN_MODE(pdata->sysref_pattern_mode) |
		AD9528_SYSREF_SOURCE(pdata->sysref_src) |
		AD9528_SYSREF_NSHOT_MODE(pdata->sysref_nshot_mode) |
		AD9528_SYSREF_PATTERN_TRIGGER_CTRL(pdata->sysref_req_trigger_mode) |
		(pdata->sysref_req_en ? AD9528_SYSREF_REQUEST_BY_PIN : 0);
	ret = ad9528_write(indio_dev, AD9528_SYSREF_CTRL, sysref_ctrl);
	if (ret < 0)
		return ret;

	ret = ad9528_write(indio_dev, AD9528_PD_EN, AD9528_PD_BIAS |
			   AD_IF(pll1_bypass_en, AD9528_PD_PLL1) |
			   AD_IF(pll2_bypass_en, AD9528_PD_PLL2));
	if (ret < 0)
		return ret;

	ret = ad9528_io_update(indio_dev);
	if (ret < 0)
		return ret;

	if (!pdata->pll2_bypass_en) {
		ret = ad9528_write(indio_dev, AD9528_PLL2_VCO_CTRL,
				vco_ctrl | AD9528_PLL2_VCO_CALIBRATE);
		if (ret < 0)
			return ret;

		ret = ad9528_io_update(indio_dev);
		if (ret < 0)
			return ret;

		ret = ad9528_poll(indio_dev, AD9528_READBACK,
				AD9528_IS_CALIBRATING, 0);
		if (ret < 0)
			return ret;
	}

	sysref_ctrl |= AD9528_SYSREF_PATTERN_REQ;
	ret = ad9528_write(indio_dev, AD9528_SYSREF_CTRL, sysref_ctrl);
	if (ret < 0)
		return ret;

	if (pdata->stat0_pin_func_sel != 0xFF) {
		ret = ad9528_write(indio_dev, AD9528_STAT_MON0,
				   pdata->stat0_pin_func_sel);
		if (ret < 0)
			return ret;

		stat_en_mask |= AD9528_STAT0_PIN_EN;
	}

	if (pdata->stat1_pin_func_sel != 0xFF) {
		ret = ad9528_write(indio_dev, AD9528_STAT_MON1,
				   pdata->stat1_pin_func_sel);
		if (ret < 0)
			return ret;

		stat_en_mask |= AD9528_STAT1_PIN_EN;
	}

	if (stat_en_mask) {
		ret = ad9528_write(indio_dev, AD9528_STAT_PIN_EN,
				   stat_en_mask);
		if (ret < 0)
			return ret;
	}

	ret = ad9528_io_update(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad9528_sync(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad9528_clks_register(indio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

#ifdef CONFIG_OF
static struct ad9528_platform_data *ad9528_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node, *chan_np;
	struct ad9528_platform_data *pdata;
	struct ad9528_channel_spec *chan;
	unsigned int tmp, cnt = 0;
	const char *str;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->spi3wire = of_property_read_bool(np, "adi,spi-3wire-enable");

	tmp = 0;
	of_property_read_u32(np, "adi,vcxo-freq", &tmp);
	pdata->vcxo_freq = tmp;

	pdata->refa_en = of_property_read_bool(np, "adi,refa-enable");
	pdata->refb_en = of_property_read_bool(np, "adi,refb-enable");

	/* Differential/ Single-Ended Input Configuration */
	pdata->refa_diff_rcv_en = of_property_read_bool(np, "adi,refa-diff-rcv-enable");
	pdata->refb_diff_rcv_en = of_property_read_bool(np, "adi,refb-diff-rcv-enable");
	pdata->osc_in_diff_en = of_property_read_bool(np, "adi,osc-in-diff-enable");

	/*
	 * Valid if differential input disabled
	 * if false defaults to pos input
	 */
	pdata->refa_cmos_neg_inp_en =
		of_property_read_bool(np, "adi,refa-cmos-neg-inp-enable");
	pdata->refb_cmos_neg_inp_en =
		of_property_read_bool(np, "adi,refb-cmos-neg-inp-enable");
	pdata->osc_in_cmos_neg_inp_en =
		of_property_read_bool(np, "adi,osc-in-cmos-neg-inp-enable");

	/* PLL1 Setting */
	tmp = 1;
	of_property_read_u32(np, "adi,refa-r-div", &tmp);
	pdata->refa_r_div = tmp;
	tmp = 1;
	of_property_read_u32(np, "adi,refb-r-div", &tmp);
	pdata->refb_r_div = tmp;

	tmp = 1;
	of_property_read_u32(np, "adi,pll1-feedback-div", &tmp);
	pdata->pll1_feedback_div = tmp;

	tmp = 1;
	of_property_read_u32(np, "adi,pll1-feedback-src-vcxo", &tmp);
	pdata->pll1_feedback_src_vcxo = tmp;

	tmp = 5000;
	of_property_read_u32(np, "adi,pll1-charge-pump-current-nA", &tmp);
	pdata->pll1_charge_pump_current_nA = tmp;

	pdata->pll1_bypass_en = of_property_read_bool(np, "adi,pll1-bypass-enable");

	/* Reference */
	tmp = REF_MODE_EXT_REF;
	of_property_read_u32(np, "adi,ref-mode", &tmp);
	pdata->ref_mode = tmp;

	tmp = SYSREF_SRC_INTERNAL;
	of_property_read_u32(np, "adi,sysref-src", &tmp);
	pdata->sysref_src = tmp;

	tmp = SYSREF_PATTERN_CONTINUOUS;
	of_property_read_u32(np, "adi,sysref-pattern-mode", &tmp);
	pdata->sysref_pattern_mode = tmp;

	tmp = 512;
	of_property_read_u32(np, "adi,sysref-k-div", &tmp);
	pdata->sysref_k_div = tmp;

	tmp = 0;
	of_property_read_u32(np, "adi,sysref-nshot-mode", &tmp);
	pdata->sysref_nshot_mode = tmp;

	pdata->sysref_req_en = of_property_read_bool(np, "adi,sysref-request-enable");

	tmp = 0;
	of_property_read_u32(np, "adi,sysref-request-trigger-mode", &tmp);
	pdata->sysref_req_trigger_mode = tmp;

	/* PLL2 Setting */
	of_property_read_u32(np, "adi,pll2-charge-pump-current-nA",
			     &pdata->pll2_charge_pump_current_nA);

	ret = of_property_read_u32(np, "adi,pll2-m1-frequency", &tmp);
	if (ret == 0) {
		ret = ad9528_calc_dividers(pdata->vcxo_freq, tmp, pdata);
		if (ret)
			return NULL;
	} else {
		pdata->pll2_freq_doubler_en =
			of_property_read_bool(np, "adi,pll2-freq-doubler-enable");

		tmp = 0;
		of_property_read_u32(np, "adi,pll2-r1-div", &tmp);
		pdata->pll2_r1_div = tmp;
		tmp = 0;
		of_property_read_u32(np, "adi,pll2-n2-div", &tmp);
		pdata->pll2_n2_div = tmp;
		tmp = 0;
		of_property_read_u32(np, "adi,pll2-vco-diff-m1", &tmp);
		of_property_read_u32(np, "adi,pll2-vco-div-m1", &tmp);
		pdata->pll2_vco_div_m1 = tmp;
	}
	pdata->pll2_bypass_en = of_property_read_bool(np, "adi,pll2-bypass-enable");

	/* Loop Filter PLL2 */

	tmp = RPOLE2_900_OHM;
	of_property_read_u32(np, "adi,rpole2", &tmp);
	pdata->rpole2 = tmp;

	tmp = RZERO_1850_OHM;
	of_property_read_u32(np, "adi,rzero", &tmp);
	pdata->rzero = tmp;

	tmp = CPOLE1_16_PF;
	of_property_read_u32(np, "adi,cpole1", &tmp);
	pdata->cpole1 = tmp;

	pdata->rzero_bypass_en = of_property_read_bool(np, "adi,rzero-bypass-enable");

	/* Status Monitor Pins */

	tmp = 0xFF;
	of_property_read_u32(np, "adi,status-mon-pin0-function-select", &tmp);
	pdata->stat0_pin_func_sel = tmp;

	tmp = 0xFF;
	of_property_read_u32(np, "adi,status-mon-pin1-function-select", &tmp);
	pdata->stat1_pin_func_sel = tmp;

	/* Output Channel Configuration */

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
		pdata->channels[cnt].channel_divider = tmp;
		of_property_read_u32(chan_np, "adi,signal-source", &tmp);
		pdata->channels[cnt].signal_source = tmp;
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
struct ad9528_platform_data *ad9528_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ad9528_probe(struct spi_device *spi)
{
	struct ad9528_platform_data *pdata;
	struct iio_dev *indio_dev;
	struct ad9528_state *st;
	struct gpio_desc *status0_gpio;
	struct gpio_desc *status1_gpio;
	struct clk *clk;
	int ret;

	clk = devm_clk_get(&spi->dev, NULL);
	if (PTR_ERR(clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (spi->dev.of_node)
		pdata = ad9528_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);

	st->reg = devm_regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	status0_gpio = devm_gpiod_get_optional(&spi->dev,
					"status0", GPIOD_OUT_LOW);
	status1_gpio = devm_gpiod_get_optional(&spi->dev,
					"status1", GPIOD_OUT_LOW);

	st->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (!IS_ERR(st->reset_gpio)) {
		udelay(1);
		ret = gpiod_direction_output(st->reset_gpio, 1);
	}

	mdelay(10);

	if (!PTR_ERR_OR_ZERO(status0_gpio))
		gpiod_direction_input(status0_gpio);
	if (!PTR_ERR_OR_ZERO(status1_gpio))
		gpiod_direction_input(status1_gpio);

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;
	st->pdata = pdata;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = (pdata->name[0] != 0) ? pdata->name :
			  spi_get_device_id(spi)->name;
	indio_dev->info = &ad9528_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->ad9528_channels;
	indio_dev->num_channels = pdata->num_channels;

	ret = ad9528_setup(indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int ad9528_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9528_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return 0;
}

static const struct spi_device_id ad9528_id[] = {
	{"ad9528", 9528},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9528_id);

static struct spi_driver ad9528_driver = {
	.driver = {
		.name	= "ad9528",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9528_probe,
	.remove		= ad9528_remove,
	.id_table	= ad9528_id,
};
module_spi_driver(ad9528_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD9528 CLOCKDIST/PLL");
MODULE_LICENSE("GPL v2");
