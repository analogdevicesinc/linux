// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AXI-AION-TRIG IP core.
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regmap.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/jesd204/jesd204.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>

#include <linux/fpga/adi-axi-common.h>

#define AXI_AION_IIO_DRIVER_NAME "axi_aion_trig"

/* Register addresses */

#define AION_IDENTIFICATION_REG 0x000C
#define AION_CONTROL_REG 0x0010
#define AION_DEBUG_REG 0x0014
#define AION_MANUAL_TRIGGER 0x0018
#define AION_TRIG_CH0_PHASE_REG 0x001C
#define AION_TRIG_CH1_PHASE_REG 0x0020
#define AION_TRIG_CH2_PHASE_REG 0x0024
#define AION_TRIG_CH3_PHASE_REG 0x0028
#define AION_TRIG_CH4_PHASE_REG 0x002C
#define AION_TRIG_CH5_PHASE_REG 0x0030
#define AION_TRIG_CH6_PHASE_REG 0x0034
#define AION_TRIG_CH7_PHASE_REG 0x0038

/* Register bit masks */
#define AION_CONTROL_ENABLE_MISALIGN_CHECK BIT(14)
#define AION_CONTROL_DEBUG_TRIG BIT(13)
#define AION_CONTROL_ENABLE_DEBUG_TRIG  BIT(12)
#define AION_CONTROL_TRIG_SELECT        BIT(11)
#define AION_CONTROL_SW_RESET   BIT(10)
#define AION_CONTROL_TRIG_CHANNEL_ENABLE_MASK GENMASK(9, 2)
#define AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK BIT(1)
#define AION_CONTROL_BSYNC_DIRECTION_MASK BIT(0)

#define AION_DEBUG_BSYNC_STATE_MASK GENMASK(26, 24)
#define AION_DEBUG_BSYNC_CAPTURED_MASK BIT(23)
#define AION_DEBUG_BSYNC_ALIGNMENT_ERROR_MASK BIT(22)
#define AION_DEBUG_BSYNC_RATIO_MASK GENMASK(21, 6)
#define AION_DEBUG_BSYNC_DELAY_MASK GENMASK(5, 1)
#define AION_DEBUG_BSYNC_READY_MASK BIT(0)

#define AION_TRIG_CH_STATE_MASK GENMASK(18, 16)
#define AION_TRIG_CH_PHASE_MASK GENMASK(15, 0)

static const char *const axi_aion_trig_states[] = {
	"idle", "trig_edge", "phase_read", "trig_adjust",
	"UNDEF", "UNDEF", "UNDEF", "UNDEF",
};

static const char *const axi_aion_bsync_states[] = {
	"idle",  "bsync_edge", "calib", "bsync_gen",
	"bsync_alignment_error", "UNDEF", "UNDEF", "UNDEF",
};

struct axi_aion_state {
	struct regmap *regmap;
	/*
	 * lock to protect against concurrent accesses of the HW and device
	 * global variables.
	 */
	struct mutex lock;
	struct clk *dev_clk;
	struct jesd204_dev *jdev;
	struct iio_channel *iio_adf4030;
};

struct axi_aion_state *st_global;

enum {
	BSYNC_OUT_EN,
	BSYNC_INT_EN,
	STATUS,
	FREQUENCY,
	TRIGGER_SELECT_GPIO_EN,
	MANUAL_TRIGGER,
};

static ssize_t axi_aion_ext_info_read(struct iio_dev *indio_dev, uintptr_t private,
				      const struct iio_chan_spec *chan, char *buf)
{
	struct axi_aion_state *st = iio_priv(indio_dev);
	unsigned int regval, regval2;
	unsigned long dev_clk;
	int ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case BSYNC_OUT_EN:
		ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", !(regval & AION_CONTROL_BSYNC_DIRECTION_MASK));
	case BSYNC_INT_EN:
		ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n",
				  !(regval & AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK));
	case STATUS:
		ret = regmap_read(st->regmap, AION_DEBUG_REG, &regval);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, chan->address, &regval2);
		if (ret)
			return ret;

		return sysfs_emit(
			       buf,
			       "trig_state=%s bsync_state=%s bsync_captured=%u bsync_alignment_error=%u bsync_ratio=%u bsync_delay=%u bsync_ready=%u\n",
			       axi_aion_trig_states[(regval2 & AION_TRIG_CH_STATE_MASK) >> 16],
			       axi_aion_bsync_states[(regval & AION_DEBUG_BSYNC_STATE_MASK) >> 24],
			       !!(regval & AION_DEBUG_BSYNC_CAPTURED_MASK),
			       !!(regval & AION_DEBUG_BSYNC_ALIGNMENT_ERROR_MASK),
			       (u32)((regval & AION_DEBUG_BSYNC_RATIO_MASK) >> 6),
			       (u32)((regval & AION_DEBUG_BSYNC_DELAY_MASK) >> 1),
			       !!(regval & AION_DEBUG_BSYNC_READY_MASK));
	case FREQUENCY:
		ret = regmap_read(st->regmap, AION_DEBUG_REG, &regval);
		if (ret)
			return ret;

		dev_clk = clk_get_rate(st->dev_clk);
		if (!dev_clk)
			return -EINVAL;

		return sysfs_emit(buf, "%lu\n", dev_clk / ((u8)((regval & AION_DEBUG_BSYNC_RATIO_MASK) >> 6) * 2));

	case TRIGGER_SELECT_GPIO_EN:
		ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", !!(regval & AION_CONTROL_TRIG_SELECT));
	case MANUAL_TRIGGER:
		ret = regmap_read(st->regmap, AION_MANUAL_TRIGGER, &regval);
		if (ret)
			return ret;

		return sysfs_emit(buf, "%u\n", regval); /* Self clears should always be 0 */
	default:
		return -EINVAL;
	};
}

static ssize_t axi_aion_ext_info_write(struct iio_dev *indio_dev, uintptr_t private,
				       const struct iio_chan_spec *chan, const char *buf,
				       size_t len)
{
	struct axi_aion_state *st = iio_priv(indio_dev);
	bool readin;
	int ret = 0;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case BSYNC_OUT_EN:
		if (readin)
			ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG,
						AION_CONTROL_BSYNC_DIRECTION_MASK);
		else
			ret = regmap_set_bits(st->regmap, AION_CONTROL_REG,
					      AION_CONTROL_BSYNC_DIRECTION_MASK);

		if (ret)
			return ret;

		break;
	case BSYNC_INT_EN:
		if (readin)
			ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG,
						AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);
		else
			ret = regmap_set_bits(st->regmap, AION_CONTROL_REG,
					      AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);

		if (ret)
			return ret;
		break;
	case MANUAL_TRIGGER:
		if (readin)
			ret = regmap_write(st->regmap, AION_MANUAL_TRIGGER, 1);

		break;
	case TRIGGER_SELECT_GPIO_EN:
		if (readin)
			ret = regmap_set_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_TRIG_SELECT);
		else
			ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_TRIG_SELECT);

		if (ret)
			return ret;
		break;
	default:
		return -ENOTSUPP;
	}

	return len;
}

static struct iio_chan_spec_ext_info axi_aion_ext_info[] = {
	{
		.name = "output_enable",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_OUT_EN,
	},
	{
		.name = "internal_bsync_enable",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = BSYNC_INT_EN,
	},
	{
		.name = "status",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = STATUS,
	},
	{
		.name = "frequency",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = FREQUENCY,
	},
	{
		.name = "trigger_select_gpio_enable",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = TRIGGER_SELECT_GPIO_EN,
	},
	{
		.name = "trigger_now",
		.read = axi_aion_ext_info_read,
		.write = axi_aion_ext_info_write,
		.shared = IIO_SHARED_BY_TYPE,
		.private = MANUAL_TRIGGER,
	},
	{}
};

static int axi_aion_iio_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct axi_aion_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PHASE:
		ret = regmap_read(st->regmap, chan->address, &regval);
		if (ret)
			return ret;
		*val = regval & 0xFFFF;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);
		if (ret)
			return ret;
		*val = !!(regval & BIT(chan->channel + 2));
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int axi_aion_iio_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
				  int val, int val2, long mask)
{
	struct axi_aion_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_PHASE:
		if (val > 0xFFFF || val < 0)
			return -EINVAL;
		return regmap_write(st->regmap, chan->address, val);
	case IIO_CHAN_INFO_ENABLE:
		return regmap_update_bits(st->regmap, AION_CONTROL_REG, BIT(chan->channel + 2),
					  val ? BIT(chan->channel + 2) : 0);
	default:
		return -EINVAL;
	}
}

static int axi_aion_reg_access(struct iio_dev *indio_dev, unsigned int reg, unsigned int writeval,
			       unsigned int *readval)
{
	struct axi_aion_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info axi_aion_iio_info = {
	.read_raw = axi_aion_iio_read_raw,
	.write_raw = axi_aion_iio_write_raw,
	.debugfs_reg_access = &axi_aion_reg_access,
};

static const struct iio_chan_spec axi_aion_iio_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.address = AION_TRIG_CH0_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 1,
		.address = AION_TRIG_CH1_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 2,
		.address = AION_TRIG_CH2_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 3,
		.address = AION_TRIG_CH3_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 4,
		.address = AION_TRIG_CH4_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 5,
		.address = AION_TRIG_CH5_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 6,
		.address = AION_TRIG_CH6_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 7,
		.address = AION_TRIG_CH7_PHASE_REG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PHASE) | BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = axi_aion_ext_info,
	},
};

void axi_aion_trig_manual_trigger(void)
{
	regmap_write(st_global->regmap, AION_MANUAL_TRIGGER, 1);
}
EXPORT_SYMBOL_GPL(axi_aion_trig_manual_trigger);

static int axi_aion_enable_debug_trig_set(void *arg, const u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct axi_aion_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (val)
		return regmap_set_bits(st->regmap, AION_CONTROL_REG,
				       AION_CONTROL_ENABLE_DEBUG_TRIG);
	else
		return regmap_clear_bits(st->regmap, AION_CONTROL_REG,
					 AION_CONTROL_ENABLE_DEBUG_TRIG);
}

static int axi_aion_enable_debug_trig_get(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct axi_aion_state *st = iio_priv(indio_dev);
	int ret;
	u32 regval;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);

	*val = !!(regval & AION_CONTROL_ENABLE_DEBUG_TRIG);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(axi_aion_enable_debug_trig_fops,
			 axi_aion_enable_debug_trig_get,
			 axi_aion_enable_debug_trig_set, "%llu\n");

static int axi_aion_debug_trig_set(void *arg, const u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct axi_aion_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (val)
		return regmap_set_bits(st->regmap, AION_CONTROL_REG,
				       AION_CONTROL_DEBUG_TRIG);
	else
		return regmap_clear_bits(st->regmap, AION_CONTROL_REG,
					 AION_CONTROL_DEBUG_TRIG);
}

static int axi_aion_debug_trig_get(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct axi_aion_state *st = iio_priv(indio_dev);
	int ret;
	u32 regval;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AION_CONTROL_REG, &regval);

	*val = !!(regval & AION_CONTROL_DEBUG_TRIG);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(axi_aion_debug_trig_fops,
			 axi_aion_debug_trig_get,
			 axi_aion_debug_trig_set, "%llu\n");

static int axi_aion_iio_write_channel_ext_info(struct iio_channel *chan,
					       const char *ext_name, long long val)
{
	ssize_t size;
	char str[16];

	snprintf(str, sizeof(str), "%lld\n", val);

	size = iio_write_channel_ext_info(chan, ext_name, str, sizeof(str));
	if (size != sizeof(str))
		return -EINVAL;

	return 0;
}

static int axi_aion_jesd204_link_supported(struct jesd204_dev *jdev,
					   enum jesd204_state_op_reason reason,
					   struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axi_aion_state *st = iio_priv(indio_dev);
	unsigned long rate;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int axi_aion_jesd204_opt_setup_stage1(struct jesd204_dev *jdev,
					     enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axi_aion_state *st = iio_priv(indio_dev);
	struct axi_aion_chan_spec *chan;
	int ret, i;
	u32 regval;
	s64 adf4030_phase;
	int val, val2;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (IS_ERR_OR_NULL(st->iio_adf4030))
		return JESD204_STATE_CHANGE_DONE;

	ret = regmap_write(st->regmap, AION_CONTROL_REG, AION_CONTROL_SW_RESET | AION_CONTROL_BSYNC_DIRECTION_MASK);
	if (ret) {
		dev_err(dev, "Failed to reset BSYNC\n");
		goto out;
	}

	ret = axi_aion_iio_write_channel_ext_info(st->iio_adf4030, "output_enable", 1);
	if (ret) {
		dev_err(dev, "Failed to enable BSYNC output\n");
		goto out;
	}
	ret = iio_write_channel_attribute(st->iio_adf4030, 0, 0, IIO_CHAN_INFO_PHASE);
	if (ret) {
		dev_err(dev, "Failed to set phase\n");
		goto out;
	}

	ret = regmap_write(st->regmap, AION_CONTROL_REG, AION_CONTROL_SW_RESET | AION_CONTROL_BSYNC_DIRECTION_MASK);
	if (ret) {
		dev_err(dev, "Failed to reset BSYNC\n");
		goto out;
	}

	ret = regmap_read_poll_timeout(st->regmap, AION_DEBUG_REG,
				       regval, (regval & (AION_DEBUG_BSYNC_READY_MASK | AION_DEBUG_BSYNC_CAPTURED_MASK)) ==
				       (AION_DEBUG_BSYNC_READY_MASK | AION_DEBUG_BSYNC_CAPTURED_MASK),
				       100, 10000);
	if (ret) {
		dev_err(dev, "BSYNC not ready %u\n", __LINE__);
		goto out;
	}

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);
	if (ret) {
		dev_err(dev, "Failed to disable internal BSYNC\n");
		goto out;
	}

	ret = axi_aion_iio_write_channel_ext_info(st->iio_adf4030, "output_enable", 0);
	if (ret) {
		dev_err(dev, "Failed to enable BSYNC output\n");
		goto out;
	}

	ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG,
				AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);
	if (ret) {
		dev_err(dev, "Failed to enable internal BSYNC\n");
		goto out;
	}

	ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG,
				AION_CONTROL_BSYNC_DIRECTION_MASK);
	if (ret) {
		dev_err(dev, "Failed to set BSYNC direction\n");
		goto out;
	}

	ret = regmap_read_poll_timeout(st->regmap, AION_DEBUG_REG,
				       regval, (regval & AION_DEBUG_BSYNC_READY_MASK),
				       100, 10000);
	if (ret) {
		dev_err(dev, "BSYNC not ready %u\n", __LINE__);
		goto out;
	}

	ret = iio_read_channel_attribute(st->iio_adf4030, &val, &val2, IIO_CHAN_INFO_PHASE);
	if (ret != IIO_VAL_INT_64) {
		dev_err(dev, "Failed to read phase (ret=%d)\n", ret);
		goto out;
	}
	adf4030_phase = (s64)((((u64)val2) << 32) | (u32)val);

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG,
			      AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);
	if (ret) {
		dev_err(dev, "Failed to disable internal BSYNC\n");
		goto out;
	}

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG,
			      AION_CONTROL_BSYNC_DIRECTION_MASK);
	if (ret) {
		dev_err(dev, "Failed to set BSYNC direction\n");
		goto out;
	}

	ret = axi_aion_iio_write_channel_ext_info(st->iio_adf4030, "output_enable", 1);
	if (ret) {
		dev_err(dev, "Failed to enable BSYNC output\n");
		goto out;
	}

	ret = regmap_read_poll_timeout(st->regmap, AION_DEBUG_REG,
				       regval, (regval & (AION_DEBUG_BSYNC_READY_MASK | AION_DEBUG_BSYNC_CAPTURED_MASK)) ==
				       (AION_DEBUG_BSYNC_READY_MASK | AION_DEBUG_BSYNC_CAPTURED_MASK),
				       10, 1000);
	if (ret) {
		dev_err(dev, "BSYNC not ready\n");
		goto out;
	}

	val = lower_32_bits(-1 * adf4030_phase);
	val2 = upper_32_bits(-1 * adf4030_phase);

	ret = iio_write_channel_attribute(st->iio_adf4030, val, val2, IIO_CHAN_INFO_PHASE);
	if (ret) {
		dev_err(dev, "Failed to set phase (ret=%d)\n", ret);
		goto out;
	}

	ret = regmap_clear_bits(st->regmap, AION_CONTROL_REG,
				AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);
	if (ret) {
		dev_err(dev, "Failed to disable internal BSYNC\n");
		goto out;
	}

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_TRIG_CHANNEL_ENABLE_MASK);
	if (ret) {
		dev_err(dev, "Failed to enable trigger channels\n");
		return ret;
	}

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_TRIG_SELECT);
	if (ret) {
		dev_err(dev, "Failed to select trigger channel\n");
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
out:
	regmap_set_bits(st->regmap, AION_CONTROL_REG,
			AION_CONTROL_BSYNC_DIRECTION_MASK);

	regmap_clear_bits(st->regmap, AION_CONTROL_REG,
			  AION_CONTROL_DISABLE_INTERNAL_BSYNC_MASK);

	axi_aion_iio_write_channel_ext_info(st->iio_adf4030, "output_enable", 1);

	return ret;
}

static const struct jesd204_dev_data axi_aion_jesd204_data = {
	.state_ops = {
		[JESD204_OP_LINK_SUPPORTED] = {
			.per_link = axi_aion_jesd204_link_supported,
		},

		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = axi_aion_jesd204_opt_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},
};

static const struct regmap_config axi_aion_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1000,
};

static int axi_aion_iio_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axi_aion_state *st;
	struct resource *res;
	void __iomem *base;
	const unsigned int *expected_ver;
	u32 ver;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->regmap = devm_regmap_init_mmio(&pdev->dev, base, &axi_aion_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->dev_clk = devm_clk_get_enabled(&pdev->dev, "device_clk");
	if (IS_ERR(st->dev_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->dev_clk), "failed to get clock\n");

	st->jdev = devm_jesd204_dev_register(&pdev->dev, &axi_aion_jesd204_data);
	if (IS_ERR(st->jdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->jdev),
				     "failed to register JESD204 device\n");

	st->iio_adf4030 = devm_fwnode_iio_channel_get_by_name(&pdev->dev, of_fwnode_handle(pdev->dev.of_node), "bsync");
	if (IS_ERR(st->iio_adf4030) && PTR_ERR(st->iio_adf4030) != -ENOENT)
		return dev_err_probe(&pdev->dev, PTR_ERR(st->iio_adf4030), "%s: error getting channel\n",
				     "bsync");

	dev_set_drvdata(&pdev->dev, indio_dev);

	mutex_init(&st->lock);

	indio_dev->name = AXI_AION_IIO_DRIVER_NAME;
	indio_dev->info = &axi_aion_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = axi_aion_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(axi_aion_iio_channels);

	expected_ver = device_get_match_data(&pdev->dev);
	if (!expected_ver)
		return -ENODEV;

	ret = regmap_set_bits(st->regmap, AION_CONTROL_REG, AION_CONTROL_SW_RESET);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADI_AXI_REG_VERSION, &ver);
	if (ret)
		return ret;

	if (*expected_ver > ver) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(*expected_ver),
			ADI_AXI_PCORE_VER_MINOR(*expected_ver),
			ADI_AXI_PCORE_VER_PATCH(*expected_ver), ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver), ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	st_global = st;

	debugfs_create_file_unsafe("enable_debug_trig", 0600,
				   iio_get_debugfs_dentry(indio_dev), indio_dev,
				   &axi_aion_enable_debug_trig_fops);

	debugfs_create_file_unsafe("debug_trig", 0600,
				   iio_get_debugfs_dentry(indio_dev), indio_dev,
				   &axi_aion_debug_trig_fops);

	ret = devm_jesd204_fsm_start(&pdev->dev, st->jdev, JESD204_LINKS_ALL);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to start JESD204 FSM\n");

	dev_info(&pdev->dev, "AXI AION IP core (%d.%.2d.%c) probed\n", ADI_AXI_PCORE_VER_MAJOR(ver),
		 ADI_AXI_PCORE_VER_MINOR(ver), ADI_AXI_PCORE_VER_PATCH(ver));

	return 0;
}

static unsigned int axi_aion_1_0_a_info = ADI_AXI_PCORE_VER(1, 0, 'a');

static const struct of_device_id axi_aion_iio_of_match[] = {
	{ .compatible = "adi,axi-aion-trig-1.0.a", .data = &axi_aion_1_0_a_info },
	{}
};
MODULE_DEVICE_TABLE(of, axi_aion_iio_of_match);

static struct platform_driver axi_aion_iio_driver = {
	.driver = {
		.name = AXI_AION_IIO_DRIVER_NAME,
		.of_match_table = axi_aion_iio_of_match,
	},
	.probe = axi_aion_iio_probe,
};
module_platform_driver(axi_aion_iio_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("AXI AION IIO Platform Driver");
MODULE_LICENSE("GPL");
