// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9508 1.65 GHz Clock Fanout Buffer with Output Dividers and Delay Adjust
 * Core driver, shared by the bus drivers.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/time64.h>
#include <linux/units.h>

#include "clk-ad9508.h"

/*
 * Registers wider than 8 bits span two consecutive addresses, the LSB at the
 * lower one. The regmap is indexed by the lower address and holds the whole
 * 16-bit value, so that masks cover the full register. The bus drivers access
 * such a register as a single transfer when the bus allows it.
 */
#define AD9508_REG_SPI_CONFIG		0x00
#define AD9508_REG_SILICON_REV		0x0A
#define AD9508_REG_PART_ID		0x0C
#define AD9508_REG_SLEEP		0x13
#define AD9508_REG_SYNC			0x14
#define AD9508_REG_OUT_BASE		0x15
#define AD9508_REG_OUT_STRIDE		6
#define AD9508_REG_DIV(ch)		(0x15 + AD9508_REG_OUT_STRIDE * (ch))
#define AD9508_REG_PHASE(ch)		(0x17 + AD9508_REG_OUT_STRIDE * (ch))
#define AD9508_REG_OUT(ch)		(0x19 + AD9508_REG_OUT_STRIDE * (ch))
#define AD9508_REG_MAX			AD9508_REG_OUT(AD9508_NUM_OUTPUTS - 1)

/* AD9508_REG_SPI_CONFIG: each bit is mirrored, both copies must match */
#define AD9508_SPI_CONFIG_SDO_EN	(BIT(7) | BIT(0))
#define AD9508_SPI_CONFIG_SOFT_RESET	(BIT(5) | BIT(2))

/* AD9508_REG_SYNC */
#define AD9508_SYNC_BAR			BIT(0)

/* AD9508_REG_DIV: division is the field value + 1 */
#define AD9508_DIV_MSK			GENMASK(9, 0)

/* AD9508_REG_PHASE: phase offset in steps of half an input clock period */
#define AD9508_PHASE_MSK		GENMASK(10, 0)

/* AD9508_REG_OUT: output driver, power down and sync */
#define AD9508_OUT_CMOS_P_EN		BIT(15)
#define AD9508_OUT_CMOS_P_PHASE_MSK	GENMASK(14, 13)
#define AD9508_OUT_CMOS_N_EN		BIT(12)
#define AD9508_OUT_CMOS_N_PHASE_MSK	GENMASK(11, 10)
#define AD9508_OUT_PD			BIT(7)
#define AD9508_OUT_SYNC_MASK		BIT(6)
#define AD9508_OUT_PHASE_MSK		GENMASK(5, 4)
#define AD9508_OUT_MODE_MSK		GENMASK(3, 1)

/* Driver phase field values, for both differential and CMOS drivers */
#define AD9508_PHASE_FORCE_HIGH		0
#define AD9508_PHASE_NONINVERTING	1
#define AD9508_PHASE_INVERTING		2
#define AD9508_PHASE_FORCE_LOW		3
/* Not a register value: CMOS driver disabled */
#define AD9508_CMOS_DISABLED		4

/* Output driver mode field values */
#define AD9508_MODE_LVDS_0_5X		0
#define AD9508_MODE_LVDS_0_75X		1
#define AD9508_MODE_LVDS		2
#define AD9508_MODE_LVDS_1_25X		3
#define AD9508_MODE_HSTL		4
#define AD9508_MODE_HSTL_BOOST		5
#define AD9508_MODE_CMOS		6

#define AD9508_NUM_OUTPUTS		4
#define AD9508_DIV_WIDTH		10
#define AD9508_PART_ID			0x0005

#define AD9508_RESET_DELAY_US		(20 * USEC_PER_MSEC)

/* Indexed by the AD9508_OUT_MODE_MSK field value */
static const char * const ad9508_output_modes[] = {
	[AD9508_MODE_LVDS_0_5X] = "lvds-0.5x",
	[AD9508_MODE_LVDS_0_75X] = "lvds-0.75x",
	[AD9508_MODE_LVDS] = "lvds",
	[AD9508_MODE_LVDS_1_25X] = "lvds-1.25x",
	[AD9508_MODE_HSTL] = "hstl",
	[AD9508_MODE_HSTL_BOOST] = "hstl-boost",
	[AD9508_MODE_CMOS] = "cmos",
};

/* Indexed by the differential driver phase field value */
static const char * const ad9508_polarities[] = {
	[AD9508_PHASE_FORCE_HIGH] = "force-high",
	[AD9508_PHASE_NONINVERTING] = "normal",
	[AD9508_PHASE_INVERTING] = "inverted",
	[AD9508_PHASE_FORCE_LOW] = "force-low",
};

/* Indexed by the CMOS driver phase field value */
static const char * const ad9508_cmos_pin_modes[] = {
	[AD9508_PHASE_FORCE_HIGH] = "force-high",
	[AD9508_PHASE_NONINVERTING] = "normal",
	[AD9508_PHASE_INVERTING] = "inverted",
	[AD9508_PHASE_FORCE_LOW] = "force-low",
	[AD9508_CMOS_DISABLED] = "disabled",
};

struct ad9508;

struct ad9508_output {
	struct clk_hw hw;
	struct ad9508 *st;
	unsigned int idx;
	bool sync_ignore;
};

struct ad9508 {
	struct device *dev;
	const struct ad9508_bus_ops *bus_ops;
	void *bus_context;
	struct regmap *regmap;
	struct gpio_desc *sync_gpio;
	struct clk *ref_clk;
	struct ad9508_output outputs[AD9508_NUM_OUTPUTS];
	struct clk_hw_onecell_data *clk_data;
};

static inline struct ad9508_output *to_ad9508_output(struct clk_hw *hw)
{
	return container_of(hw, struct ad9508_output, hw);
}

static int ad9508_get_div(struct ad9508_output *out, unsigned int *div)
{
	unsigned int val;
	int ret;

	ret = regmap_read(out->st->regmap, AD9508_REG_DIV(out->idx), &val);
	if (ret)
		return ret;

	*div = FIELD_GET(AD9508_DIV_MSK, val) + 1;

	return 0;
}

static int ad9508_get_phase_code(struct ad9508_output *out, unsigned int *code)
{
	unsigned int val;
	int ret;

	ret = regmap_read(out->st->regmap, AD9508_REG_PHASE(out->idx), &val);
	if (ret)
		return ret;

	*code = FIELD_GET(AD9508_PHASE_MSK, val);

	return 0;
}

static int ad9508_set_phase_code(struct ad9508_output *out, unsigned int code)
{
	return regmap_update_bits(out->st->regmap, AD9508_REG_PHASE(out->idx),
				  AD9508_PHASE_MSK,
				  FIELD_PREP(AD9508_PHASE_MSK, code));
}

static int ad9508_set_sync_masks(struct ad9508 *st, unsigned long masked)
{
	unsigned int i;
	int ret;

	for (i = 0; i < AD9508_NUM_OUTPUTS; i++) {
		ret = regmap_update_bits(st->regmap, AD9508_REG_OUT(i),
					 AD9508_OUT_SYNC_MASK,
					 FIELD_PREP(AD9508_OUT_SYNC_MASK,
						    test_bit(i, &masked)));
		if (ret)
			return ret;
	}

	return 0;
}

static unsigned long ad9508_default_sync_masks(struct ad9508 *st)
{
	unsigned long masked = 0;
	unsigned int i;

	for (i = 0; i < AD9508_NUM_OUTPUTS; i++)
		if (st->outputs[i].sync_ignore)
			__set_bit(i, &masked);

	return masked;
}

/*
 * Divider and phase offset changes only take effect deterministically on
 * the rising edge of SYNC, and the datasheet requires SYNC to be asserted
 * before the phase offset is changed. Every reconfiguration is therefore
 * bracketed by ad9508_sync_begin()/ad9508_sync_end(): outputs taking part
 * in the synchronization are held low while the registers are written and
 * restart edge aligned when SYNC is released.
 *
 * Outputs flagged with adi,sync-ignore form no group: when one of them is
 * reconfigured, all the other outputs are temporarily masked so that only
 * the output being changed restarts. Otherwise, the whole group of
 * synchronized outputs restarts together.
 *
 * All callers are clk_ops that run under the clk prepare lock, which
 * serializes the SYNC sequence and the sync mask updates, or the probe
 * before the clocks are registered.
 */
static int ad9508_sync_begin(struct ad9508_output *out)
{
	struct ad9508 *st = out->st;
	int ret;

	if (out->sync_ignore) {
		ret = ad9508_set_sync_masks(st, GENMASK(AD9508_NUM_OUTPUTS - 1, 0) &
					    ~BIT(out->idx));
		if (ret)
			return ret;
	}

	if (st->sync_gpio) {
		gpiod_set_value_cansleep(st->sync_gpio, 1);
		return 0;
	}

	return regmap_clear_bits(st->regmap, AD9508_REG_SYNC, AD9508_SYNC_BAR);
}

static int ad9508_sync_end(struct ad9508_output *out)
{
	struct ad9508 *st = out->st;
	int ret;

	if (st->sync_gpio) {
		gpiod_set_value_cansleep(st->sync_gpio, 0);
	} else {
		ret = regmap_set_bits(st->regmap, AD9508_REG_SYNC,
				      AD9508_SYNC_BAR);
		if (ret)
			return ret;
	}

	if (out->sync_ignore)
		return ad9508_set_sync_masks(st, ad9508_default_sync_masks(st));

	return 0;
}

static int ad9508_clk_prepare(struct clk_hw *hw)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	int ret, ret2;

	/*
	 * A divider that is powered up while other outputs are running is not
	 * aligned with them, so power it up within a SYNC sequence.
	 */
	ret = ad9508_sync_begin(out);
	if (ret)
		return ret;

	ret = regmap_update_bits(out->st->regmap, AD9508_REG_OUT(out->idx),
				 AD9508_OUT_PD, 0);
	ret2 = ad9508_sync_end(out);

	return ret ?: ret2;
}

static void ad9508_clk_unprepare(struct clk_hw *hw)
{
	struct ad9508_output *out = to_ad9508_output(hw);

	regmap_update_bits(out->st->regmap, AD9508_REG_OUT(out->idx), AD9508_OUT_PD,
			   AD9508_OUT_PD);
}

static int ad9508_clk_is_prepared(struct clk_hw *hw)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	unsigned int val;
	int ret;

	ret = regmap_read(out->st->regmap, AD9508_REG_OUT(out->idx), &val);
	if (ret)
		return ret;

	return !FIELD_GET(AD9508_OUT_PD, val);
}

static unsigned long ad9508_clk_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	unsigned int div;
	int ret;

	ret = ad9508_get_div(out, &div);
	if (ret)
		return 0;

	return DIV_ROUND_UP(parent_rate, div);
}

static int ad9508_clk_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	return divider_determine_rate(hw, req, NULL, AD9508_DIV_WIDTH,
				      CLK_DIVIDER_ROUND_CLOSEST);
}

static int ad9508_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	int val, ret, ret2;

	val = divider_get_val(rate, parent_rate, NULL, AD9508_DIV_WIDTH,
			      CLK_DIVIDER_ROUND_CLOSEST);
	if (val < 0)
		return val;

	/*
	 * The phase offset is a delay in half input clock periods, which is
	 * kept across divider changes, such as the board delay compensation
	 * set by adi,output-delay-ps.
	 */
	ret = ad9508_sync_begin(out);
	if (ret)
		return ret;

	ret = regmap_update_bits(out->st->regmap, AD9508_REG_DIV(out->idx),
				 AD9508_DIV_MSK, FIELD_PREP(AD9508_DIV_MSK, val));
	ret2 = ad9508_sync_end(out);

	return ret ?: ret2;
}

static int ad9508_clk_get_phase(struct clk_hw *hw)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	unsigned int div, code;
	int ret;

	ret = ad9508_get_div(out, &div);
	if (ret)
		return ret;

	ret = ad9508_get_phase_code(out, &code);
	if (ret)
		return ret;

	/* One code is half an input period, that is 180 / div degrees */
	return DIV_ROUND_CLOSEST(code * 180, div) % 360;
}

static int ad9508_clk_set_phase(struct clk_hw *hw, int degrees)
{
	struct ad9508_output *out = to_ad9508_output(hw);
	unsigned int div, code;
	int ret, ret2;

	/* No other output is aligned with this one, phase is meaningless */
	if (out->sync_ignore)
		return -EOPNOTSUPP;

	ret = ad9508_get_div(out, &div);
	if (ret)
		return ret;

	/* degrees is in [0, 360), so code is below 2048 */
	code = DIV_ROUND_CLOSEST(degrees * div, 180) % (2 * div);

	ret = ad9508_sync_begin(out);
	if (ret)
		return ret;

	ret = ad9508_set_phase_code(out, code);
	ret2 = ad9508_sync_end(out);

	return ret ?: ret2;
}

static const struct clk_ops ad9508_clk_ops = {
	.prepare = ad9508_clk_prepare,
	.unprepare = ad9508_clk_unprepare,
	.is_prepared = ad9508_clk_is_prepared,
	.recalc_rate = ad9508_clk_recalc_rate,
	.determine_rate = ad9508_clk_determine_rate,
	.set_rate = ad9508_clk_set_rate,
	.get_phase = ad9508_clk_get_phase,
	.set_phase = ad9508_clk_set_phase,
};

static unsigned int ad9508_reg_size(unsigned int reg)
{
	if (reg == AD9508_REG_PART_ID)
		return 2;

	/* Divide ratio, phase offset and output registers, 2 bytes each */
	if (reg >= AD9508_REG_OUT_BASE)
		return 2;

	return 1;
}

static bool ad9508_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD9508_REG_SPI_CONFIG:
	case AD9508_REG_SILICON_REV:
	case AD9508_REG_PART_ID:
	case AD9508_REG_SLEEP:
	case AD9508_REG_SYNC:
		return true;
	case AD9508_REG_OUT_BASE ... AD9508_REG_MAX:
		return !((reg - AD9508_REG_OUT_BASE) % 2);
	default:
		return false;
	}
}

static bool ad9508_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD9508_REG_SILICON_REV:
	case AD9508_REG_PART_ID:
		return false;
	default:
		return ad9508_readable_reg(dev, reg);
	}
}

static bool ad9508_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == AD9508_REG_SPI_CONFIG;
}

static int ad9508_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad9508 *st = context;

	return st->bus_ops->read(st->bus_context, reg, ad9508_reg_size(reg),
				 val);
}

static int ad9508_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad9508 *st = context;

	return st->bus_ops->write(st->bus_context, reg, ad9508_reg_size(reg),
				  val);
}

static const struct regmap_config ad9508_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = AD9508_REG_MAX,
	.reg_read = ad9508_reg_read,
	.reg_write = ad9508_reg_write,
	.readable_reg = ad9508_readable_reg,
	.writeable_reg = ad9508_writeable_reg,
	.volatile_reg = ad9508_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

static int ad9508_reset(struct ad9508 *st, bool sdo_enable)
{
	struct gpio_desc *reset_gpio;
	unsigned int config, id;
	int ret;

	/* The SDO enable bits have no effect in I2C mode */
	config = sdo_enable ? AD9508_SPI_CONFIG_SDO_EN : 0;

	reset_gpio = devm_gpiod_get_optional(st->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(st->dev, PTR_ERR(reset_gpio),
				     "failed to get reset GPIO\n");

	if (reset_gpio) {
		fsleep(1);
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		/*
		 * The soft reset bits are self clearing, but the device stays
		 * in reset until a further serial clock cycle, provided by the
		 * write below.
		 */
		ret = regmap_write(st->regmap, AD9508_REG_SPI_CONFIG,
				   config | AD9508_SPI_CONFIG_SOFT_RESET);
		if (ret)
			return ret;
	}

	/* The SDO pin is disabled after reset, set the SPI port mode */
	ret = regmap_write(st->regmap, AD9508_REG_SPI_CONFIG, config);
	if (ret)
		return ret;

	/* Outputs take about 20 ms to start toggling after a reset */
	fsleep(AD9508_RESET_DELAY_US);

	ret = regmap_read(st->regmap, AD9508_REG_PART_ID, &id);
	if (ret)
		return ret;

	if (id != AD9508_PART_ID)
		return dev_err_probe(st->dev, -ENODEV,
				     "unexpected part ID 0x%04x\n", id);

	return 0;
}

static int ad9508_match_string(struct device *dev,
			       struct fwnode_handle *child,
			       const char *propname, const char * const *array,
			       size_t n, int def)
{
	int ret;

	ret = fwnode_property_match_property_string(child, propname, array, n);
	if (ret == -EINVAL)
		return def;
	if (ret < 0)
		return dev_err_probe(dev, ret, "%pfwP: invalid %s\n", child,
				     propname);

	return ret;
}

static int ad9508_parse_delay(struct ad9508 *st, struct fwnode_handle *child,
			      struct ad9508_output *out, unsigned int *code)
{
	struct device *dev = st->dev;
	unsigned long ref_rate;
	u32 delay_ps;
	u64 val;
	int ret;

	ret = fwnode_property_read_u32(child, "adi,output-delay-ps", &delay_ps);
	if (ret)
		return ret == -EINVAL ? 0 : ret;

	if (out->sync_ignore)
		return dev_err_probe(dev, -EINVAL,
				     "%pfwP: adi,output-delay-ps needs synchronization\n",
				     child);

	ref_rate = clk_get_rate(st->ref_clk);
	if (!ref_rate)
		return dev_err_probe(dev, -EINVAL, "unknown input clock rate\n");

	/* One phase offset step is half an input clock period */
	val = DIV64_U64_ROUND_CLOSEST((u64)delay_ps * 2 * ref_rate,
				      PSEC_PER_SEC);
	if (val > FIELD_MAX(AD9508_PHASE_MSK))
		return dev_err_probe(dev, -ERANGE,
				     "%pfwP: adi,output-delay-ps out of range\n",
				     child);

	*code = val;

	return 1;
}

static int ad9508_set_delays(struct ad9508 *st, const unsigned int *codes,
			     unsigned long delayed)
{
	struct ad9508_output *out;
	unsigned int i;
	int ret, ret2;

	if (!delayed)
		return 0;

	/* Delayed outputs are synchronized together, one sequence is enough */
	out = &st->outputs[__ffs(delayed)];
	ret = ad9508_sync_begin(out);
	if (ret)
		return ret;

	for_each_set_bit(i, &delayed, AD9508_NUM_OUTPUTS) {
		ret = ad9508_set_phase_code(&st->outputs[i], codes[i]);
		if (ret)
			break;
	}

	ret2 = ad9508_sync_end(out);

	return ret ?: ret2;
}

static int ad9508_parse_outputs(struct ad9508 *st)
{
	unsigned int codes[AD9508_NUM_OUTPUTS] = { };
	struct device *dev = st->dev;
	int mode, polarity, cmos_p, cmos_n, ret;
	unsigned long delayed = 0;
	unsigned int cfg;
	u32 reg;

	device_for_each_child_node_scoped(dev, child) {
		struct ad9508_output *out;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret, "%pfwP: missing reg\n",
					     child);

		if (reg >= AD9508_NUM_OUTPUTS)
			return dev_err_probe(dev, -EINVAL,
					     "%pfwP: invalid output %u\n",
					     child, reg);

		out = &st->outputs[reg];
		out->sync_ignore = fwnode_property_read_bool(child,
							     "adi,sync-ignore");

		ret = ad9508_parse_delay(st, child, out, &codes[reg]);
		if (ret < 0)
			return ret;
		if (ret)
			__set_bit(reg, &delayed);

		mode = ad9508_match_string(dev, child, "adi,output-mode",
					   ad9508_output_modes,
					   ARRAY_SIZE(ad9508_output_modes),
					   AD9508_MODE_LVDS);
		if (mode < 0)
			return mode;

		/* Differential driver settings are kept at default in CMOS mode */
		polarity = AD9508_PHASE_NONINVERTING;
		cmos_p = AD9508_CMOS_DISABLED;
		cmos_n = AD9508_CMOS_DISABLED;
		if (mode == AD9508_MODE_CMOS) {
			cmos_p = ad9508_match_string(dev, child, "adi,cmos-p-mode",
						     ad9508_cmos_pin_modes,
						     ARRAY_SIZE(ad9508_cmos_pin_modes),
						     AD9508_PHASE_NONINVERTING);
			if (cmos_p < 0)
				return cmos_p;

			cmos_n = ad9508_match_string(dev, child, "adi,cmos-n-mode",
						     ad9508_cmos_pin_modes,
						     ARRAY_SIZE(ad9508_cmos_pin_modes),
						     AD9508_PHASE_INVERTING);
			if (cmos_n < 0)
				return cmos_n;
		} else {
			polarity = ad9508_match_string(dev, child,
						       "adi,output-polarity",
						       ad9508_polarities,
						       ARRAY_SIZE(ad9508_polarities),
						       AD9508_PHASE_NONINVERTING);
			if (polarity < 0)
				return polarity;
		}

		ret = regmap_read(st->regmap, AD9508_REG_OUT(reg), &cfg);
		if (ret)
			return ret;

		/* A disabled CMOS driver keeps its default phase */
		FIELD_MODIFY(AD9508_OUT_MODE_MSK, &cfg, mode);
		FIELD_MODIFY(AD9508_OUT_PHASE_MSK, &cfg, polarity);
		FIELD_MODIFY(AD9508_OUT_CMOS_P_EN, &cfg,
			     cmos_p != AD9508_CMOS_DISABLED);
		FIELD_MODIFY(AD9508_OUT_CMOS_P_PHASE_MSK, &cfg,
			     cmos_p != AD9508_CMOS_DISABLED ? cmos_p : 0);
		FIELD_MODIFY(AD9508_OUT_CMOS_N_EN, &cfg,
			     cmos_n != AD9508_CMOS_DISABLED);
		FIELD_MODIFY(AD9508_OUT_CMOS_N_PHASE_MSK, &cfg,
			     cmos_n != AD9508_CMOS_DISABLED ? cmos_n : 0);

		ret = regmap_write(st->regmap, AD9508_REG_OUT(reg), cfg);
		if (ret)
			return ret;
	}

	ret = ad9508_set_sync_masks(st, ad9508_default_sync_masks(st));
	if (ret)
		return ret;

	return ad9508_set_delays(st, codes, delayed);
}

static int ad9508_register_clocks(struct ad9508 *st)
{
	const char *names[AD9508_NUM_OUTPUTS] = { };
	const struct clk_parent_data parent_data = { .index = 0 };
	struct device *dev = st->dev;
	struct clk_init_data init = { };
	unsigned int i;
	int ret;

	st->clk_data = devm_kzalloc(dev, struct_size(st->clk_data, hws,
						     AD9508_NUM_OUTPUTS),
				    GFP_KERNEL);
	if (!st->clk_data)
		return -ENOMEM;

	st->clk_data->num = AD9508_NUM_OUTPUTS;

	ret = device_property_read_string_array(dev, "clock-output-names",
						names, ARRAY_SIZE(names));
	if (ret < 0 && ret != -EINVAL)
		return ret;

	init.ops = &ad9508_clk_ops;
	init.parent_data = &parent_data;
	init.num_parents = 1;

	for (i = 0; i < AD9508_NUM_OUTPUTS; i++) {
		struct ad9508_output *out = &st->outputs[i];

		init.name = names[i];
		if (!init.name) {
			init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-out%u",
						   dev_name(dev), i);
			if (!init.name)
				return -ENOMEM;
		}

		out->hw.init = &init;
		ret = devm_clk_hw_register(dev, &out->hw);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to register output %u\n", i);

		st->clk_data->hws[i] = &out->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   st->clk_data);
}

int ad9508_probe(struct device *dev, const struct ad9508_bus_ops *bus_ops,
		 void *bus_context, bool sdo_enable)
{
	struct ad9508 *st;
	unsigned int i;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = dev;
	st->bus_ops = bus_ops;
	st->bus_context = bus_context;

	st->regmap = devm_regmap_init(dev, NULL, st, &ad9508_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to init regmap\n");

	for (i = 0; i < AD9508_NUM_OUTPUTS; i++) {
		st->outputs[i].st = st;
		st->outputs[i].idx = i;
	}

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd supply\n");

	st->ref_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(st->ref_clk))
		return dev_err_probe(dev, PTR_ERR(st->ref_clk),
				     "failed to get input clock\n");

	/* SYNC is released: outputs keep toggling while being configured */
	st->sync_gpio = devm_gpiod_get_optional(dev, "sync", GPIOD_OUT_LOW);
	if (IS_ERR(st->sync_gpio))
		return dev_err_probe(dev, PTR_ERR(st->sync_gpio),
				     "failed to get sync GPIO\n");

	ret = ad9508_reset(st, sdo_enable);
	if (ret)
		return ret;

	ret = ad9508_parse_outputs(st);
	if (ret)
		return ret;

	return ad9508_register_clocks(st);
}
EXPORT_SYMBOL_NS_GPL(ad9508_probe, "CLK_AD9508");

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9508 clock fanout buffer core driver");
MODULE_LICENSE("GPL");
