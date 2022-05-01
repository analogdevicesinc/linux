// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * TDD NG HDL CORE driver
 *
 * Copyright 2022 Analog Devices Inc.
 *
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

/* Register Map */

#define ADI_REG_TDD_MAGIC 0x000c
#define ADI_TDD_MAGIC 0x5444444E // TDDN

#define ADI_REG_TDD_INTERFACE_DESCRIPTION 0x0010
#define ADI_TDD_SYNC_COUNT_WIDTH GENMASK(30, 24)
#define ADI_TDD_SYNC_COUNT_WIDTH_GET(x) FIELD_GET(ADI_TDD_SYNC_COUNT_WIDTH, x)
#define ADI_TDD_BURST_COUNT_WIDTH GENMASK(21, 16)
#define ADI_TDD_BURST_COUNT_WIDTH_GET(x) FIELD_GET(ADI_TDD_BURST_COUNT_WIDTH, x)
#define ADI_TDD_REG_WIDTH GENMASK(13, 8)
#define ADI_TDD_REG_WIDTH_GET(x) FIELD_GET(ADI_TDD_REG_WIDTH, x)
#define ADI_TDD_SYNC_EXTERNAL_CDC BIT(7)
#define ADI_TDD_SYNC_EXTERNAL BIT(6)
#define ADI_TDD_SYNC_INTERNAL BIT(5)
#define ADI_TDD_CHANNEL_COUNT GENMASK(4, 0)
#define ADI_TDD_CHANNEL_COUNT_GET(x) FIELD_GET(ADI_TDD_CHANNEL_COUNT, x)

#define ADI_REG_TDD_CONTROL 0x0040
#define ADI_TDD_SYNC_SOFT BIT(4)
#define ADI_TDD_SYNC_EXT BIT(3)
#define ADI_TDD_SYNC_INT BIT(2)
#define ADI_TDD_SYNC_RST BIT(1)
#define ADI_TDD_ENABLE BIT(0)

#define ADI_REG_TDD_CHANNEL_ENABLE 0x0044
#define ADI_REG_TDD_CHANNEL_POLARITY 0x0048
#define ADI_REG_TDD_BURST_COUNT 0x004c
#define ADI_REG_TDD_STARTUP_DELAY 0x0050
#define ADI_REG_TDD_FRAME_LENGTH 0x0054
#define ADI_REG_TDD_SYNC_COUNTER_LOW 0x0058
#define ADI_REG_TDD_SYNC_COUNTER_HIGH 0x005c

#define ADI_REG_TDD_STATUS 0x0060

#define ADI_REG_TDD_CHANNEL_BASE 0x0080
#define ADI_TDD_CHANNEL_OFFSET 0x0008
#define ADI_TDD_CHANNEL_ON 0x0000
#define ADI_TDD_CHANNEL_OFF 0x0004

#define ADI_REG_TDD_CHANNEL(c, o)					\
	(ADI_REG_TDD_CHANNEL_BASE + ADI_TDD_CHANNEL_OFFSET * (c) + o)

enum adi_axi_tdd_attribute_id {
	ADI_TDD_ATTR_VERSION,
	ADI_TDD_ATTR_CORE_ID,
	ADI_TDD_ATTR_SCRATCH,
	ADI_TDD_ATTR_MAGIC,

	ADI_TDD_ATTR_SYNC_SOFT,
	ADI_TDD_ATTR_SYNC_EXT,
	ADI_TDD_ATTR_SYNC_INT,
	ADI_TDD_ATTR_SYNC_RST,
	ADI_TDD_ATTR_ENABLE,

	ADI_TDD_ATTR_BURST_COUNT,
	ADI_TDD_ATTR_STARTUP_DELAY_RAW,
	ADI_TDD_ATTR_STARTUP_DELAY_MS,
	ADI_TDD_ATTR_FRAME_LENGTH_RAW,
	ADI_TDD_ATTR_FRAME_LENGTH_MS,
	ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_RAW,
	ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_MS,

	ADI_TDD_ATTR_STATE,

	ADI_TDD_ATTR_CHANNEL_ENABLE,
	ADI_TDD_ATTR_CHANNEL_POLARITY,
	ADI_TDD_ATTR_CHANNEL_ON_RAW,
	ADI_TDD_ATTR_CHANNEL_OFF_RAW,
	ADI_TDD_ATTR_CHANNEL_ON_MS,
	ADI_TDD_ATTR_CHANNEL_OFF_MS,
};

struct adi_axi_tdd_ng_clk {
	struct notifier_block nb;
	struct clk *clk;
	unsigned long rate;
};

struct adi_axi_tdd_attribute {
	struct device_attribute attr;
	enum adi_axi_tdd_attribute_id id;
	u32 channel;
	u8 name[32];
};

#define to_tdd_attribute(x) container_of(x, struct adi_axi_tdd_attribute, attr)

/**
 * struct adi_axi_tdd_ng_state - Driver state information for the TDD NG CORE
 * @clk: Interface clock definition. Used to translate ms into cycle counts
 * @regs: Device register base address in memory
 * @sync_count_width: Bit width of the internal synchronization counter, <= 64
 * @sync_count_mask: Bit mask of sync counter
 * @burst_count_width: Bit width of the burst counter, <= 32
 * @burst_count_mask: Bit mask of the burst counter
 * @reg_width: Timing register bit width, <= 32
 * @reg_mask: Timing register bit mask
 * @sync_external_cdc: Whether the external sync input is synchronized into the main clock domain
 * @sync_external: Whether external synchronization support was enabled at synthesis time
 * @sync_internal: Whether internal synchronization support was enabled at synthesis time
 * @channel_count: Available channel count
 * @enabled: Whether the TDD engine is currently enabled.
 *	     Note: Most configuration registers cannot be changed while the TDD core is enabled.
 * @lock: Protects state fields and protects against unordered accesses to the registers
 */
struct adi_axi_tdd_ng_state {
	struct adi_axi_tdd_ng_clk clk;
	void __iomem *regs;
	u32 sync_count_width;
	u64 sync_count_mask;
	u32 burst_count_width;
	u32 burst_count_mask;
	u32 reg_width;
	u32 reg_mask;
	bool sync_external_cdc;
	bool sync_external;
	bool sync_internal;
	u32 channel_count;
	bool enabled;

	/* TDD core access locking */
	struct mutex lock;
};

static inline void _tdd_write(struct adi_axi_tdd_ng_state *st, const u32 reg, const u32 val)
{
	iowrite32(val, st->regs + reg);
}

static inline u32 _tdd_read(struct adi_axi_tdd_ng_state *st, const u32 reg)
{
	return ioread32(st->regs + reg);
}

static inline void tdd_write(struct adi_axi_tdd_ng_state *st, const u32 reg, const u32 val)
{
	mutex_lock(&st->lock);
	iowrite32(val, st->regs + reg);
	mutex_unlock(&st->lock);
}

static inline void tdd_write64(struct adi_axi_tdd_ng_state *st, const u32 lreg, const u32 hreg,
			     const u64 val)
{
	mutex_lock(&st->lock);

	_tdd_write(st, lreg, (u32) (val & 0xFFFFFFFFU));
	_tdd_write(st, hreg, (u32) ((val >> 32) & 0xFFFFFFFFU));

	mutex_unlock(&st->lock);
}

static inline u32 tdd_read(struct adi_axi_tdd_ng_state *st, const u32 reg)
{
	u32 val;

	mutex_lock(&st->lock);
	val = ioread32(st->regs + reg);
	mutex_unlock(&st->lock);

	return val;
}

static inline u64 tdd_read64(struct adi_axi_tdd_ng_state *st, const u32 lreg, const u32 hreg)
{
	u64 data;

	mutex_lock(&st->lock);

	data = ((u64) _tdd_read(st, hreg)) << 32;
	data |= _tdd_read(st, lreg);

	mutex_unlock(&st->lock);

	return data;
}

static void _tdd_update_bits(struct adi_axi_tdd_ng_state *st, const u32 reg,
			     const u32 mask, const u32 val)
{
	u32 __val;

	__val = _tdd_read(st, reg);
	__val = (__val & ~mask) | (val & mask);
	_tdd_write(st, reg, __val);
}

static void tdd_update_bits(struct adi_axi_tdd_ng_state *st, const u32 reg,
			    const u32 mask, const u32 val)
{
	mutex_lock(&st->lock);
	_tdd_update_bits(st, reg, mask, val);
	mutex_unlock(&st->lock);
}

static int adi_axi_tdd_ng_rate_change(struct notifier_block *nb,
				      unsigned long flags, void *data)
{
	struct adi_axi_tdd_ng_clk *clk =
		container_of(nb, struct adi_axi_tdd_ng_clk, nb);
	struct clk_notifier_data *cnd = data;

	/* cache the new rate */
	WRITE_ONCE(clk->rate, cnd->new_rate);

	return NOTIFY_OK;
}

static void adi_axi_tdd_ng_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static void adi_axi_tdd_ng_clk_notifier_unreg(void *data)
{
	struct adi_axi_tdd_ng_clk *clk = data;

	clk_notifier_unregister(clk->clk, &clk->nb);
}

static int adi_axi_tdd_ng_clk_setup(struct platform_device *pdev, struct adi_axi_tdd_ng_state *st)
{
	int ret;
	struct adi_axi_tdd_ng_clk *clk = &st->clk;
	struct clk *aclk;

	aclk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(aclk))
		return PTR_ERR(aclk);

	ret = clk_prepare_enable(aclk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, adi_axi_tdd_ng_clk_disable, aclk);
	if (ret)
		return ret;

	clk->clk = devm_clk_get(&pdev->dev, "intf_clk");
	if (IS_ERR(clk->clk))
		return PTR_ERR(clk->clk);

	ret = clk_prepare_enable(clk->clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, adi_axi_tdd_ng_clk_disable, clk->clk);
	if (ret)
		return ret;

	clk->rate = clk_get_rate(clk->clk);
	clk->nb.notifier_call = adi_axi_tdd_ng_rate_change;
	ret = clk_notifier_register(clk->clk, &clk->nb);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&pdev->dev, adi_axi_tdd_ng_clk_notifier_unreg, clk);
}

static int adi_axi_tdd_format_ms(struct adi_axi_tdd_ng_state *st, u64 x, char *buf)
{
	u64 t_ns;
	u32 vals[2];

	t_ns = div_u64(x * 1000000000ULL, READ_ONCE(st->clk.rate));
	vals[0] = div_u64_rem(t_ns, 1000000, &vals[1]);
	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, 2, vals);
}

static ssize_t adi_axi_tdd_show(struct device *dev,
				struct device_attribute *dev_attr, char *buf)
{
	struct adi_axi_tdd_ng_state *st = dev_get_drvdata(dev);
	const struct adi_axi_tdd_attribute *attr = to_tdd_attribute(dev_attr);
	u32 data;
	int ret = -ENODEV;
	u32 channel = attr->channel;
	bool ms = false;

	switch (attr->id) {
	case ADI_TDD_ATTR_VERSION:
		data = _tdd_read(st, ADI_AXI_REG_VERSION);
		ret = sysfs_emit(buf, "%d.%.2d.%c\n",
				 ADI_AXI_PCORE_VER_MAJOR(data),
				 ADI_AXI_PCORE_VER_MINOR(data),
				 ADI_AXI_PCORE_VER_PATCH(data));
		break;

	case ADI_TDD_ATTR_CORE_ID:
		data = _tdd_read(st, ADI_AXI_REG_ID);
		ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_SCRATCH:
		data = tdd_read(st, ADI_AXI_REG_SCRATCH);
		ret = sysfs_emit(buf, "0x%08x\n", data);
		break;

	case ADI_TDD_ATTR_MAGIC:
		data = _tdd_read(st, ADI_AXI_REG_CONFIG);
		ret = sysfs_emit(buf, "0x%08x\n", data);
		break;

	case ADI_TDD_ATTR_SYNC_EXT:
		data = tdd_read(st, ADI_REG_TDD_CONTROL);
		ret = sysfs_emit(buf, "%d\n", !!(data & ADI_TDD_SYNC_EXT));
		break;

	case ADI_TDD_ATTR_SYNC_INT:
		data = tdd_read(st, ADI_REG_TDD_CONTROL);
		ret = sysfs_emit(buf, "%d\n", !!(data & ADI_TDD_SYNC_INT));
		break;

	case ADI_TDD_ATTR_SYNC_RST:
		data = tdd_read(st, ADI_REG_TDD_CONTROL);
		ret = sysfs_emit(buf, "%d\n", !!(data & ADI_TDD_SYNC_RST));
		break;

	case ADI_TDD_ATTR_ENABLE:
		data = tdd_read(st, ADI_REG_TDD_CONTROL);
		st->enabled = !!(data & ADI_TDD_ENABLE);
		ret = sysfs_emit(buf, "%d\n", st->enabled);
		break;

	case ADI_TDD_ATTR_BURST_COUNT:
		data = tdd_read(st, ADI_REG_TDD_BURST_COUNT);
		ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_STARTUP_DELAY_RAW:
		data = tdd_read(st, ADI_REG_TDD_STARTUP_DELAY);
		ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_STARTUP_DELAY_MS:
		ret = adi_axi_tdd_format_ms(st, tdd_read(st, ADI_REG_TDD_STARTUP_DELAY), buf);
		break;

	case ADI_TDD_ATTR_FRAME_LENGTH_RAW:
		data = tdd_read(st, ADI_REG_TDD_FRAME_LENGTH);
		ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_FRAME_LENGTH_MS:
		ret = adi_axi_tdd_format_ms(st, tdd_read(st, ADI_REG_TDD_FRAME_LENGTH), buf);
		break;

	case ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_RAW:
		ret = sysfs_emit(buf, "%llu\n", tdd_read64(st, ADI_REG_TDD_SYNC_COUNTER_LOW,
							   ADI_REG_TDD_SYNC_COUNTER_HIGH));
		break;

	case ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_MS:
		ret = adi_axi_tdd_format_ms(st, tdd_read64(st, ADI_REG_TDD_SYNC_COUNTER_LOW,
							   ADI_REG_TDD_SYNC_COUNTER_HIGH), buf);
		break;

	case ADI_TDD_ATTR_STATE:
		data = _tdd_read(st, ADI_REG_TDD_STATUS);
		ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_CHANNEL_ENABLE:
		data = _tdd_read(st, ADI_REG_TDD_CHANNEL_ENABLE);
		ret = sysfs_emit(buf, "%d\n", !!((1U << channel) & data));
		break;

	case ADI_TDD_ATTR_CHANNEL_POLARITY:
		data = _tdd_read(st, ADI_REG_TDD_CHANNEL_POLARITY);
		ret = sysfs_emit(buf, "%d\n", !!((1U << channel) & data));
		break;

	case ADI_TDD_ATTR_CHANNEL_ON_MS:
		ms = true;
		fallthrough;
	case ADI_TDD_ATTR_CHANNEL_ON_RAW:
		data = _tdd_read(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_ON));
		if (ms)
			ret = adi_axi_tdd_format_ms(st, data, buf);
		else
			ret = sysfs_emit(buf, "%u\n", data);
		break;

	case ADI_TDD_ATTR_CHANNEL_OFF_MS:
		ms = true;
		fallthrough;
	case ADI_TDD_ATTR_CHANNEL_OFF_RAW:
		data = _tdd_read(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_OFF));
		if (ms)
			ret = adi_axi_tdd_format_ms(st, data, buf);
		else
			ret = sysfs_emit(buf, "%u\n", data);
		break;

	default:
		// Must not happen
		dev_err(dev, "Failed to handle unknown attribute id: %d\n", attr->id);
		break;
	}

	return ret;
}

static u64 adi_axi_tdd_parse_ms(struct adi_axi_tdd_ng_state *st, const char *buf)
{
	u64 clk_rate = READ_ONCE(st->clk.rate);
	int ret;
	int ival, frac;

	ret = iio_str_to_fixpoint(buf, 100000, &ival, &frac);
	return DIV_ROUND_CLOSEST_ULL((u64)ival * clk_rate, 1000)
		+ DIV_ROUND_CLOSEST_ULL((u64)frac * clk_rate, 1000000000);
}

static ssize_t adi_axi_tdd_store(struct device *dev,
				 struct device_attribute *dev_attr,
				 const char *buf, size_t count)
{
	struct adi_axi_tdd_ng_state *st = dev_get_drvdata(dev);
	const struct adi_axi_tdd_attribute *attr = to_tdd_attribute(dev_attr);
	int ret = count;
	u32 data;
	u64 data64;
	u32 channel = attr->channel;

#define PARSE_AS_U32() {			\
		ret = kstrtou32(buf, 0, &data);	\
		if (ret) {			\
			ret = -EINVAL;		\
			break;			\
		}				\
		ret = count;			\
}

#define PARSE_AS_U64() {				\
		ret = kstrtou64(buf, 0, &data64);	\
		if (ret) {				\
			ret = -EINVAL;			\
			break;				\
		}					\
		ret = count;				\
}

	switch (attr->id) {
	case ADI_TDD_ATTR_SCRATCH:
		PARSE_AS_U32();
		_tdd_write(st, ADI_AXI_REG_SCRATCH, data);
		break;

	case ADI_TDD_ATTR_SYNC_SOFT:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CONTROL, ADI_TDD_SYNC_SOFT,
				ADI_TDD_SYNC_SOFT * !!data);
		break;

	case ADI_TDD_ATTR_SYNC_EXT:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CONTROL, ADI_TDD_SYNC_EXT,
				ADI_TDD_SYNC_EXT * !!data);
		break;

	case ADI_TDD_ATTR_SYNC_INT:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CONTROL, ADI_TDD_SYNC_INT,
				ADI_TDD_SYNC_INT * !!data);
		break;

	case ADI_TDD_ATTR_SYNC_RST:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CONTROL, ADI_TDD_SYNC_RST,
				ADI_TDD_SYNC_RST * !!data);
		break;

	case ADI_TDD_ATTR_ENABLE:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CONTROL, ADI_TDD_ENABLE,
				ADI_TDD_ENABLE * !!data);
		break;

	case ADI_TDD_ATTR_BURST_COUNT:
		PARSE_AS_U32();
		_tdd_write(st, ADI_REG_TDD_BURST_COUNT, data);
		break;

	case ADI_TDD_ATTR_STARTUP_DELAY_RAW:
		PARSE_AS_U32();
		_tdd_write(st, ADI_REG_TDD_STARTUP_DELAY, data);
		break;

	case ADI_TDD_ATTR_STARTUP_DELAY_MS:
		data64 = adi_axi_tdd_parse_ms(st, buf);
		if (data64 >> 32) {
			ret = -EINVAL;
			break;
		}

		_tdd_write(st, ADI_REG_TDD_STARTUP_DELAY, (u32) data64);
		break;

	case ADI_TDD_ATTR_FRAME_LENGTH_RAW:
		PARSE_AS_U32();
		_tdd_write(st, ADI_REG_TDD_FRAME_LENGTH, data);
		break;

	case ADI_TDD_ATTR_FRAME_LENGTH_MS:
		data64 = adi_axi_tdd_parse_ms(st, buf);
		if (data64 >> 32) {
			ret = -EINVAL;
			break;
		}

		_tdd_write(st, ADI_REG_TDD_FRAME_LENGTH, (u32) data64);
		break;

	case ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_RAW:
		PARSE_AS_U64();
		tdd_write64(st, ADI_REG_TDD_SYNC_COUNTER_LOW,
			    ADI_REG_TDD_SYNC_COUNTER_HIGH, data64);
		break;

	case ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_MS:
		data64 = adi_axi_tdd_parse_ms(st, buf);
		tdd_write64(st, ADI_REG_TDD_SYNC_COUNTER_LOW,
			    ADI_REG_TDD_SYNC_COUNTER_HIGH, data64);
		break;

	case ADI_TDD_ATTR_CHANNEL_ENABLE:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CHANNEL_ENABLE, (1U << channel),
				(!!data) << channel);
		break;

	case ADI_TDD_ATTR_CHANNEL_POLARITY:
		PARSE_AS_U32();
		tdd_update_bits(st, ADI_REG_TDD_CHANNEL_POLARITY, (1U << channel),
				(!!data) << channel);
		break;

	case ADI_TDD_ATTR_CHANNEL_ON_RAW:
		PARSE_AS_U32();
		tdd_write(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_ON), data);
		break;

	case ADI_TDD_ATTR_CHANNEL_ON_MS:
		data64 = adi_axi_tdd_parse_ms(st, buf);
		if (data64 >> 32) {
			ret = -EINVAL;
			break;
		}

		tdd_write(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_ON), (u32) data64);
		break;

	case ADI_TDD_ATTR_CHANNEL_OFF_RAW:
		PARSE_AS_U32();
		tdd_write(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_OFF), data);
		break;

	case ADI_TDD_ATTR_CHANNEL_OFF_MS:
		data64 = adi_axi_tdd_parse_ms(st, buf);
		if (data64 >> 32) {
			ret = -EINVAL;
			break;
		}

		tdd_write(st, ADI_REG_TDD_CHANNEL(channel, ADI_TDD_CHANNEL_OFF), (u32) data64);
		break;

	default:
		// Must not happen
		dev_err(dev, "Failed to handle unknown attribute id: %d\n", attr->id);
		break;
	}

#undef PARSE_AS_U32
#undef PARSE_AS_U64

	return ret;
}

static int
adi_axi_tdd_init_synthesis_parameters(struct adi_axi_tdd_ng_state *st)
{
	u32 interface_config = _tdd_read(st, ADI_REG_TDD_INTERFACE_DESCRIPTION);

	st->sync_count_width = ADI_TDD_SYNC_COUNT_WIDTH_GET(interface_config);
	st->burst_count_width = ADI_TDD_BURST_COUNT_WIDTH_GET(interface_config);
	st->reg_width = ADI_TDD_REG_WIDTH_GET(interface_config);
	st->sync_external_cdc = ADI_TDD_SYNC_EXTERNAL_CDC & interface_config;
	st->sync_external = ADI_TDD_SYNC_EXTERNAL & interface_config;
	st->sync_internal = ADI_TDD_SYNC_INTERNAL & interface_config;
	st->channel_count = ADI_TDD_CHANNEL_COUNT_GET(interface_config) + 1;

	if (!st->sync_count_width || !st->burst_count_width || !st->reg_width)
		return -EINVAL;

	st->sync_count_mask = (u64)((1ULL << st->sync_count_width) - 1ULL);
	st->burst_count_mask = (u32)((1ULL << st->burst_count_width) - 1ULL);
	st->reg_mask = (u32)((1ULL << st->reg_width) - 1ULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id adi_axi_tdd_ng_of_match[] = {
	{ .compatible = "adi,axi-tdd-ng-1.00.a" },
	{}
};
MODULE_DEVICE_TABLE(of, adi_axi_tdd_ng_of_match);

#define __TDD_ATTR(_name, _id, _channel, _mode)					\
	{									\
		.attr = __ATTR(_name, _mode, adi_axi_tdd_show,			\
			       adi_axi_tdd_store),				\
		.id = _id,							\
		.channel = _channel						\
	}

#define TDD_ATTR(_name, _id, _mode)						\
	struct adi_axi_tdd_attribute dev_attr_##_name =				\
		__TDD_ATTR(_name, _id, 0, _mode)

static TDD_ATTR(version, ADI_TDD_ATTR_VERSION, 0444);
static TDD_ATTR(core_id, ADI_TDD_ATTR_CORE_ID, 0444);
static TDD_ATTR(scratch, ADI_TDD_ATTR_SCRATCH, 0644);
static TDD_ATTR(magic, ADI_TDD_ATTR_MAGIC, 0444);

static TDD_ATTR(sync_soft, ADI_TDD_ATTR_SYNC_SOFT, 0200);
static TDD_ATTR(sync_external, ADI_TDD_ATTR_SYNC_EXT, 0644);
static TDD_ATTR(sync_internal, ADI_TDD_ATTR_SYNC_INT, 0644);
static TDD_ATTR(sync_reset, ADI_TDD_ATTR_SYNC_RST, 0644);
static TDD_ATTR(enable, ADI_TDD_ATTR_ENABLE, 0644);

static TDD_ATTR(burst_count, ADI_TDD_ATTR_BURST_COUNT, 0644);
static TDD_ATTR(startup_delay_raw, ADI_TDD_ATTR_STARTUP_DELAY_RAW, 0644);
static TDD_ATTR(startup_delay_ms, ADI_TDD_ATTR_STARTUP_DELAY_MS, 0644);
static TDD_ATTR(frame_length_raw, ADI_TDD_ATTR_FRAME_LENGTH_RAW,
		0644);
static TDD_ATTR(frame_length_ms, ADI_TDD_ATTR_FRAME_LENGTH_MS,
		0644);
static TDD_ATTR(internal_sync_period_raw, ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_RAW,
		0644);
static TDD_ATTR(internal_sync_period_ms, ADI_TDD_ATTR_INTERNAL_SYNC_PERIOD_MS,
		0644);

static TDD_ATTR(state, ADI_TDD_ATTR_STATE, 0444);

static struct attribute *adi_axi_tdd_base_attributes[] = {
	&dev_attr_version.attr.attr,
	&dev_attr_core_id.attr.attr,
	&dev_attr_scratch.attr.attr,
	&dev_attr_magic.attr.attr,
	&dev_attr_sync_soft.attr.attr,
	&dev_attr_sync_external.attr.attr,
	&dev_attr_sync_internal.attr.attr,
	&dev_attr_sync_reset.attr.attr,
	&dev_attr_enable.attr.attr,
	&dev_attr_burst_count.attr.attr,
	&dev_attr_startup_delay_raw.attr.attr,
	&dev_attr_startup_delay_ms.attr.attr,
	&dev_attr_frame_length_raw.attr.attr,
	&dev_attr_frame_length_ms.attr.attr,
	&dev_attr_internal_sync_period_raw.attr.attr,
	&dev_attr_internal_sync_period_ms.attr.attr,
	&dev_attr_state.attr.attr,
	/* NOT TERMINATED */
};

static int adi_axi_tdd_init_sysfs(struct platform_device *pdev, struct adi_axi_tdd_ng_state *st)
{
	struct attribute **tdd_attrs;
	struct adi_axi_tdd_attribute *channel_attributes;
	struct adi_axi_tdd_attribute *channel_iter;
	struct attribute_group *attr_group;
	int ret;
	u32 i, j;

	size_t base_attr_count = ARRAY_SIZE(adi_axi_tdd_base_attributes);
	size_t attribute_count = base_attr_count + 6 * st->channel_count + 1;

	static const char * const channel_names[] = {
		"out_channel%u_enable", "out_channel%u_polarity",
		"out_channel%u_on_raw", "out_channel%u_off_raw",
		"out_channel%u_on_ms",	"out_channel%u_off_ms"
	};

	static const enum adi_axi_tdd_attribute_id channel_ids[] = {
		ADI_TDD_ATTR_CHANNEL_ENABLE, ADI_TDD_ATTR_CHANNEL_POLARITY,
		ADI_TDD_ATTR_CHANNEL_ON_RAW, ADI_TDD_ATTR_CHANNEL_OFF_RAW,
		ADI_TDD_ATTR_CHANNEL_ON_MS,  ADI_TDD_ATTR_CHANNEL_OFF_MS
	};

	channel_attributes = devm_kcalloc(&pdev->dev, 6 * st->channel_count,
					  sizeof(*channel_attributes), GFP_KERNEL);

	tdd_attrs = devm_kcalloc(&pdev->dev, attribute_count,
				 sizeof(*tdd_attrs), GFP_KERNEL);
	memcpy(tdd_attrs, adi_axi_tdd_base_attributes, sizeof(adi_axi_tdd_base_attributes));

	channel_iter = channel_attributes;

	for (i = 0; i < st->channel_count; i++) {
		for (j = 0; j < ARRAY_SIZE(channel_names); j++) {
			snprintf(channel_iter->name,
				 sizeof(channel_iter->name), channel_names[j],
				 i);
			channel_iter->id = channel_ids[j];
			channel_iter->channel = i;
			channel_iter->attr.attr.name = channel_iter->name;
			channel_iter->attr.attr.mode = 0644;
			channel_iter->attr.show = adi_axi_tdd_show;
			channel_iter->attr.store = adi_axi_tdd_store;

			tdd_attrs[base_attr_count + 6 * i + j] = &channel_iter->attr.attr;
			channel_iter++;
		}
	}

	// Terminate list, technically not necessary because kcalloc was used
	tdd_attrs[base_attr_count + 6 * st->channel_count] = NULL;

	attr_group = devm_kcalloc(&pdev->dev, 1, sizeof(attr_group), GFP_KERNEL);
	if (!attr_group)
		return -ENOMEM;

	attr_group->attrs = tdd_attrs;

	ret = devm_device_add_group(&pdev->dev, attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register attribute group: %d\n", ret);
		return ret;
	}

	return 0;
}

static int adi_axi_tdd_ng_probe(struct platform_device *pdev)
{
	unsigned int expected_version, version;
	struct adi_axi_tdd_ng_state *st;
	int ret;

	st = devm_kcalloc(&pdev->dev, 1, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mutex_init(&st->lock);

	platform_set_drvdata(pdev, st);

	ret = adi_axi_tdd_ng_clk_setup(pdev, st);
	if (ret)
		return ret;

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	version = _tdd_read(st, ADI_AXI_REG_VERSION);
	expected_version = ADI_AXI_PCORE_VER(1, 0, 'a');

	if (ADI_AXI_PCORE_VER_MAJOR(version) !=
	    ADI_AXI_PCORE_VER_MAJOR(expected_version)) {
		dev_err(&pdev->dev,
			"Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(expected_version),
			ADI_AXI_PCORE_VER_MINOR(expected_version),
			ADI_AXI_PCORE_VER_PATCH(expected_version),
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));
		return -ENODEV;
	}

	ret = adi_axi_tdd_init_synthesis_parameters(st);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to load synthesis parameters, make sure the device is configured correctly.\n");
		return ret;
	}

	st->enabled = _tdd_read(st, ADI_REG_TDD_CONTROL) & ADI_TDD_ENABLE;

	ret = adi_axi_tdd_init_sysfs(pdev, st);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init sysfs, aborting ...\n");
		return ret;
	}

	dev_dbg(&pdev->dev, "Probed Analog Devices AXI TDD (NG) (%d.%.2d.%c)",
		ADI_AXI_PCORE_VER_MAJOR(version),
		ADI_AXI_PCORE_VER_MINOR(version),
		ADI_AXI_PCORE_VER_PATCH(version));

	return 0;
}

static struct platform_driver adi_axi_tdd_ng_driver = {
	.driver = { .name = "adi_axi_tdd_ng",
		    .of_match_table = adi_axi_tdd_ng_of_match },
	.probe = adi_axi_tdd_ng_probe,
};
module_platform_driver(adi_axi_tdd_ng_driver);

MODULE_AUTHOR("David Winter <david.winter@analog.com>");
MODULE_DESCRIPTION("Analog Devices TDD NG HDL CORE driver");
MODULE_LICENSE("Dual BSD/GPL");
