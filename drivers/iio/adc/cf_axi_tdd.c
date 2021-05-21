// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * TDD HDL CORE driver
 *
 * Copyright 2016 Analog Devices Inc.
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>

/* Transceiver TDD Control (axi_ad*) */

#define ADI_REG_TDD_CONTROL_0		0x0040
#define ADI_TDD_DMA_GATE_MASK		GENMASK(5, 4)
#define ADI_TDD_DMA_GATE_GET(x)		FIELD_GET(ADI_TDD_DMA_GATE_MASK, x)
#define ADI_TDD_DMA_GATE(x)		FIELD_PREP(ADI_TDD_DMA_GATE_MASK, x)
#define ADI_TDD_EN_MODE_MASK		GENMASK(3, 2)
#define ADI_TDD_EN_MODE_GET(x)		FIELD_GET(ADI_TDD_EN_MODE_MASK, x)
#define ADI_TDD_EN_MODE(x)		FIELD_PREP(ADI_TDD_EN_MODE_MASK, x)
#define ADI_TDD_SECONDARY_MASK		BIT(1)
#define ADI_TDD_SECONDARY_GET(x)	FIELD_GET(ADI_TDD_SECONDARY_MASK, x)
#define ADI_TDD_SECONDARY(x)		FIELD_PREP(ADI_TDD_SECONDARY_MASK, x)
#define ADI_TDD_ENABLE			BIT(0)

#define ADI_TDD_RX_TX_MASK		GENMASK(23, 0)
#define ADI_TDD_RX_TX(x)		FIELD_PREP(ADI_TDD_RX_TX_MASK, x)

#define ADI_REG_TDD_CONTROL_1		0x0044
#define ADI_TDD_BURST_COUNT_MASK	GENMASK(7, 0)
#define ADI_TDD_BURST_COUNT(x)		FIELD_PREP(ADI_TDD_BURST_COUNT_MASK, x)

#define ADI_REG_TDD_COUNTER_2		0x0048
#define ADI_TDD_COUNTER_INIT(x)		ADI_TDD_RX_TX(x)

#define ADI_REG_TDD_FRAME_LENGTH	0x004c

#define ADI_REG_TDD_SYNC_TERM_TYPE	0x0050
#define ADI_TDD_SYNC_PULSE_ENABLE	BIT(0)

#define ADI_REG_TDD_VCO_RX_TX_ON(o, c)		(0x0080 + 8 * (o) + 0x40 * (c))
#define ADI_REG_TDD_VCO_RX_TX_OFF(o, c)		(0x0084 + 8 * (o) + 0x40 * (c))
#define ADI_REG_TDD_RX_TX_ON(o, c)		(0x0090 + 8 * (o) + 0x40 * (c))
#define ADI_REG_TDD_RX_TX_OFF(o, c)		(0x0094 + 8 * (o) + 0x40 * (c))
#define ADI_REG_TDD_RX_TX_DP_ON(o, c)		(0x00A0 + 8 * (o) + 0x40 * (c))
#define ADI_REG_TDD_RX_TX_DP_OFF(o, c)		(0x00A4 + 8 * (o) + 0x40 * (c))

struct cf_axi_tdd_clk {
	struct notifier_block nb;
	struct clk *clk;
	unsigned long rate;
};

struct cf_axi_tdd_state {
	struct cf_axi_tdd_clk	clk;
	void __iomem		*regs;
	/* TDD core access locking */
	struct mutex		lock;
};

enum {
	CF_AXI_TDD_ENABLE,
	CF_AXI_TDD_ENABLE_MODE,
	CF_AXI_TDD_DMA_GATEING_MODE,
	CF_AXI_TDD_BURST_COUNT,
	CF_AXI_TDD_SECONDARY,
	CF_AXI_TDD_COUNTER_INT,
	CF_AXI_TDD_FRAME_LEN,
	CF_AXI_TDD_SYNC_TERMINAL_TYPE,
	CF_AXI_TDD_CHAN_ON,
	CF_AXI_TDD_CHAN_OFF,
	CF_AXI_TDD_CHAN_DP_ON,
	CF_AXI_TDD_CHAN_DP_OFF,
	CF_AXI_TDD_CHAN_VCO_ON,
	CF_AXI_TDD_CHAN_VCO_OFF,
};

static inline void tdd_write(struct cf_axi_tdd_state *st, const u32 reg, const u32 val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int tdd_read(struct cf_axi_tdd_state *st, const u32 reg)
{
	return ioread32(st->regs + reg);
}

static void tdd_update_bits(struct cf_axi_tdd_state *st, const u32 reg, const u32 mask,
			    const u32 val)
{
	u32 __val;

	mutex_lock(&st->lock);
	__val = tdd_read(st, reg);
	__val = (__val & ~mask) | (val & mask);
	tdd_write(st, reg, __val);
	mutex_unlock(&st->lock);
}

enum {
	CF_AXI_TDD_RX_TX,
	CF_AXI_TDD_RXONLY,
	CF_AXI_TDD_TXONLY
};

static const char * const cf_axi_tdd_en_modes[] = {
	"rx_tx", "rx_only", "tx_only"
};

static int cf_axi_tdd_set_en_modes(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				   u32 mode)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);

	if (mode > CF_AXI_TDD_TXONLY)
		return -EINVAL;

	tdd_update_bits(st, ADI_REG_TDD_CONTROL_0, ADI_TDD_EN_MODE_MASK, ADI_TDD_EN_MODE(mode));

	return 0;
}

static int cf_axi_tdd_get_en_modes(struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	const u32 reg = tdd_read(st, ADI_REG_TDD_CONTROL_0) & ADI_TDD_EN_MODE_MASK;

	return ADI_TDD_EN_MODE_GET(reg);
}

static const struct iio_enum cf_axi_tdd_en_available = {
	.items = cf_axi_tdd_en_modes,
	.num_items = ARRAY_SIZE(cf_axi_tdd_en_modes),
	.get = cf_axi_tdd_get_en_modes,
	.set = cf_axi_tdd_set_en_modes,
};

enum {
	CF_AXI_TDD_DMA_GATE_NONE,
	CF_AXI_TDD_DMA_GATE_RX,
	CF_AXI_TDD_DMA_GATE_TX,
	CF_AXI_TDD_DMA_GATE_RX_TX
};

static const char * const cf_axi_tdd_dma_gateing_mode[] = {
	"rx_tx", "rx_only", "tx_only", "none"
};

static int cf_axi_tdd_get_dma_gateing_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	const u32 reg = tdd_read(st, ADI_REG_TDD_CONTROL_0);

	return ADI_TDD_DMA_GATE_GET(reg);
}

static int cf_axi_tdd_set_dma_gateing_mode(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan, u32 mode)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);

	tdd_update_bits(st, ADI_REG_TDD_CONTROL_0, ADI_TDD_DMA_GATE_MASK, ADI_TDD_DMA_GATE(mode));

	return 0;
}

static const struct iio_enum cf_axi_tdd_dma_gateing_mode_available = {
	.items = cf_axi_tdd_dma_gateing_mode,
	.num_items = ARRAY_SIZE(cf_axi_tdd_dma_gateing_mode),
	.get = cf_axi_tdd_get_dma_gateing_mode,
	.set = cf_axi_tdd_set_dma_gateing_mode,
};

static ssize_t cf_axi_tdd_read(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	u32 val;

	switch (private) {
	case CF_AXI_TDD_BURST_COUNT:
		val = tdd_read(st, ADI_REG_TDD_CONTROL_1);
		return sprintf(buf, "%d\n", val);
	case CF_AXI_TDD_SECONDARY:
		val = tdd_read(st, ADI_REG_TDD_CONTROL_0);
		return sprintf(buf, "%lu\n", ADI_TDD_SECONDARY_GET(val));
	case CF_AXI_TDD_COUNTER_INT:
		val = tdd_read(st, ADI_REG_TDD_COUNTER_2);
		return sprintf(buf, "%d\n", val);
	case CF_AXI_TDD_SYNC_TERMINAL_TYPE:
		val = tdd_read(st, ADI_REG_TDD_SYNC_TERM_TYPE);
		return sprintf(buf, "%d\n", val);
	case CF_AXI_TDD_FRAME_LEN:
		val = tdd_read(st, ADI_REG_TDD_FRAME_LENGTH);
		break;
	case CF_AXI_TDD_CHAN_ON:
		val = tdd_read(st, ADI_REG_TDD_RX_TX_ON(chan->output, chan->channel));
		break;
	case CF_AXI_TDD_CHAN_OFF:
		val = tdd_read(st, ADI_REG_TDD_RX_TX_OFF(chan->output, chan->channel));
		break;
	case CF_AXI_TDD_CHAN_DP_ON:
		val = tdd_read(st, ADI_REG_TDD_RX_TX_DP_ON(chan->output, chan->channel));
		break;
	case CF_AXI_TDD_CHAN_DP_OFF:
		val = tdd_read(st, ADI_REG_TDD_RX_TX_DP_OFF(chan->output, chan->channel));
		break;
	case CF_AXI_TDD_CHAN_VCO_ON:
		val = tdd_read(st, ADI_REG_TDD_VCO_RX_TX_ON(chan->output, chan->channel));
		break;
	case CF_AXI_TDD_CHAN_VCO_OFF:
		val = tdd_read(st, ADI_REG_TDD_VCO_RX_TX_OFF(chan->output, chan->channel));
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%llu\n", DIV_ROUND_CLOSEST_ULL((u64)val * 1000,
							    READ_ONCE(st->clk.rate)));
}

static ssize_t cf_axi_tdd_write(struct iio_dev *indio_dev, uintptr_t private,
				const struct iio_chan_spec *chan, const char *buf, size_t len)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	u32 val, reg;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	switch (private) {
	case CF_AXI_TDD_BURST_COUNT:
		tdd_write(st, ADI_REG_TDD_CONTROL_1, ADI_TDD_BURST_COUNT(val));
		return len;
	case CF_AXI_TDD_SECONDARY:
		tdd_update_bits(st, ADI_REG_TDD_CONTROL_0, ADI_TDD_SECONDARY_MASK,
				ADI_TDD_SECONDARY(val));
		return len;
	case CF_AXI_TDD_COUNTER_INT:
		tdd_write(st, ADI_REG_TDD_COUNTER_2, ADI_TDD_COUNTER_INIT(val));
		return len;
	case CF_AXI_TDD_SYNC_TERMINAL_TYPE:
		tdd_write(st, ADI_REG_TDD_SYNC_TERM_TYPE, val & ADI_TDD_SYNC_PULSE_ENABLE);
		return len;
	case CF_AXI_TDD_FRAME_LEN:
		reg = ADI_REG_TDD_FRAME_LENGTH;
		break;
	case CF_AXI_TDD_CHAN_ON:
		reg = ADI_REG_TDD_RX_TX_ON(chan->output, chan->channel);
		break;
	case CF_AXI_TDD_CHAN_OFF:
		reg = ADI_REG_TDD_RX_TX_OFF(chan->output, chan->channel);
		break;
	case CF_AXI_TDD_CHAN_DP_ON:
		reg = ADI_REG_TDD_RX_TX_DP_ON(chan->output, chan->channel);
		break;
	case CF_AXI_TDD_CHAN_DP_OFF:
		reg = ADI_REG_TDD_RX_TX_DP_OFF(chan->output, chan->channel);
		break;
	case CF_AXI_TDD_CHAN_VCO_ON:
		reg = ADI_REG_TDD_VCO_RX_TX_ON(chan->output, chan->channel);
		break;
	case CF_AXI_TDD_CHAN_VCO_OFF:
		reg = ADI_REG_TDD_VCO_RX_TX_OFF(chan->output, chan->channel);
		break;
	default:
		return -EINVAL;
	}

	val =  DIV_ROUND_CLOSEST_ULL((u64)val * READ_ONCE(st->clk.rate), 1000);
	tdd_write(st, reg, ADI_TDD_RX_TX(val));

	return len;
}

#define CF_AXI_TDD_IIO_EXT_INFO(_name, _priv, _shared) { \
	.name = _name,	\
	.read = cf_axi_tdd_read, \
	.write = cf_axi_tdd_write, \
	.private = _priv, \
	.shared = _shared \
}

static const struct iio_chan_spec_ext_info cf_axi_tdd_ext_info[] = {
	IIO_ENUM("en_mode", IIO_SHARED_BY_ALL, &cf_axi_tdd_en_available),
	IIO_ENUM_AVAILABLE_SHARED("en_mode", IIO_SHARED_BY_ALL, &cf_axi_tdd_en_available),
	IIO_ENUM("dma_gateing_mode", IIO_SHARED_BY_ALL, &cf_axi_tdd_dma_gateing_mode_available),
	IIO_ENUM_AVAILABLE_SHARED("dma_gateing_mode", IIO_SHARED_BY_ALL,
				  &cf_axi_tdd_dma_gateing_mode_available),
	CF_AXI_TDD_IIO_EXT_INFO("burst_count", CF_AXI_TDD_BURST_COUNT, IIO_SHARED_BY_ALL),
	CF_AXI_TDD_IIO_EXT_INFO("secondary", CF_AXI_TDD_SECONDARY, IIO_SHARED_BY_ALL),
	CF_AXI_TDD_IIO_EXT_INFO("counter_int", CF_AXI_TDD_COUNTER_INT, IIO_SHARED_BY_ALL),
	CF_AXI_TDD_IIO_EXT_INFO("frame_length_ms", CF_AXI_TDD_FRAME_LEN, IIO_SHARED_BY_ALL),
	CF_AXI_TDD_IIO_EXT_INFO("sync_terminal_type", CF_AXI_TDD_SYNC_TERMINAL_TYPE,
				IIO_SHARED_BY_ALL),
	{}
};

#define CF_AXI_TDD_IIO_SHARED() { \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE), \
	.ext_info = cf_axi_tdd_ext_info, \
}

static const struct iio_chan_spec_ext_info cf_axi_tdd_chan_ext_info[] = {
	CF_AXI_TDD_IIO_EXT_INFO("on_ms", CF_AXI_TDD_CHAN_ON, IIO_SEPARATE),
	CF_AXI_TDD_IIO_EXT_INFO("off_ms", CF_AXI_TDD_CHAN_OFF, IIO_SEPARATE),
	CF_AXI_TDD_IIO_EXT_INFO("dp_on_ms", CF_AXI_TDD_CHAN_DP_ON, IIO_SEPARATE),
	CF_AXI_TDD_IIO_EXT_INFO("dp_off_ms", CF_AXI_TDD_CHAN_DP_OFF, IIO_SEPARATE),
	CF_AXI_TDD_IIO_EXT_INFO("vco_on_ms", CF_AXI_TDD_CHAN_VCO_ON, IIO_SEPARATE),
	CF_AXI_TDD_IIO_EXT_INFO("vco_off_ms", CF_AXI_TDD_CHAN_VCO_OFF, IIO_SEPARATE),
	{}
};

#define CF_AXI_TDD_IIO_CHAN(_idx, _dir) { \
	.type = IIO_GENERIC_DATA, \
	.indexed = 1, \
	.channel = _idx, \
	.output = _dir, \
	.ext_info = cf_axi_tdd_chan_ext_info, \
}

struct iio_chan_spec cf_axi_tdd_channels[] = {
	CF_AXI_TDD_IIO_SHARED(),
	/* RX1 path registers */
	CF_AXI_TDD_IIO_CHAN(0, 0),
	/* RX2 path registers */
	CF_AXI_TDD_IIO_CHAN(1, 0),
	/* TX1 path registers */
	CF_AXI_TDD_IIO_CHAN(0, 1),
	/* TX2 path registers */
	CF_AXI_TDD_IIO_CHAN(1, 1),
};

static int cf_axi_tdd_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = tdd_read(st, ADI_REG_TDD_CONTROL_0) & ADI_TDD_ENABLE;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int cf_axi_tdd_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		tdd_update_bits(st, ADI_REG_TDD_CONTROL_0, ADI_TDD_ENABLE, val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int cf_axi_tdd_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct cf_axi_tdd_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (!readval) {
		tdd_write(st, reg & 0xFFFF, writeval);
		ret = 0;
	} else {
		ret = tdd_read(st, reg & 0xFFFF);
		*readval = ret;
		ret = 0;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info cf_axi_tdd_info = {
	.read_raw = &cf_axi_tdd_read_raw,
	.write_raw = &cf_axi_tdd_write_raw,
	.debugfs_reg_access = &cf_axi_tdd_reg_access,
};

static int cf_axi_tdd_rate_change(struct notifier_block *nb, unsigned long flags, void *data)
{
	struct cf_axi_tdd_clk *clk = container_of(nb, struct cf_axi_tdd_clk, nb);
	struct clk_notifier_data *cnd = data;

	/* cache the new rate */
	WRITE_ONCE(clk->rate, cnd->new_rate);

	return NOTIFY_OK;
}

static void cf_axi_tdd_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static void cf_axi_tdd_clk_notifier_unreg(void *data)
{
	struct cf_axi_tdd_clk *clk = data;

	clk_notifier_unregister(clk->clk, &clk->nb);
}

static int cf_axi_tdd_clk_setup(struct platform_device *pdev, struct cf_axi_tdd_state *st)
{
	int ret;
	struct cf_axi_tdd_clk *clk = &st->clk;
	struct clk *aclk;

	aclk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(aclk))
		return PTR_ERR(aclk);

	ret = clk_prepare_enable(aclk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, cf_axi_tdd_clk_disable, aclk);
	if (ret)
		return ret;

	clk->clk = devm_clk_get(&pdev->dev, "intf_clk");
	if (IS_ERR(clk->clk))
		return PTR_ERR(clk->clk);

	ret = clk_prepare_enable(clk->clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, cf_axi_tdd_clk_disable, clk->clk);
	if (ret)
		return ret;

	clk->rate = clk_get_rate(clk->clk);
	clk->nb.notifier_call = cf_axi_tdd_rate_change;
	ret = clk_notifier_register(clk->clk, &clk->nb);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&pdev->dev, cf_axi_tdd_clk_notifier_unreg, clk);
}

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_tdd_of_match[] = {
	{ .compatible = "adi,axi-tdd-1.00", .data = 0},
	{ }
};
MODULE_DEVICE_TABLE(of, cf_axi_tdd_of_match);

static int cf_axi_tdd_probe(struct platform_device *pdev)
{
	unsigned int expected_version, version;
	struct cf_axi_tdd_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = cf_axi_tdd_clk_setup(pdev, st);
	if (ret)
		return ret;

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	version = tdd_read(st, ADI_AXI_REG_VERSION);
	expected_version = ADI_AXI_PCORE_VER(1, 0, 'a');

	if (ADI_AXI_PCORE_VER_MAJOR(version) !=
		ADI_AXI_PCORE_VER_MAJOR(expected_version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(expected_version),
			ADI_AXI_PCORE_VER_MINOR(expected_version),
			ADI_AXI_PCORE_VER_PATCH(expected_version),
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));
		return -ENODEV;
	}

	mutex_init(&st->lock);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "axi-core-tdd";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = cf_axi_tdd_channels;
	indio_dev->num_channels = ARRAY_SIZE(cf_axi_tdd_channels);
	indio_dev->info = &cf_axi_tdd_info;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0)
		return ret;

	dev_info(&pdev->dev, "Analog Devices CF_AXI_TDD %s (%d.%.2d.%c)",
		 tdd_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER",
		 ADI_AXI_PCORE_VER_MAJOR(version),
		 ADI_AXI_PCORE_VER_MINOR(version),
		 ADI_AXI_PCORE_VER_PATCH(version));

	return 0;
}

static struct platform_driver cf_axi_tdd_driver = {
	.driver = {
		.name = "cf_axi_tdd",
		.of_match_table = cf_axi_tdd_of_match,
	},
	.probe		= cf_axi_tdd_probe,
};
module_platform_driver(cf_axi_tdd_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices TDD HDL CORE driver");
MODULE_LICENSE("Dual BSD/GPL");
