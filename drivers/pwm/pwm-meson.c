// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * PWM controller driver for Amlogic Meson SoCs.
 *
 * This PWM is only a set of Gates, Dividers and Counters:
 * PWM output is achieved by calculating a clock that permits calculating
 * two periods (low and high). The counter then has to be set to switch after
 * N cycles for the first half period.
 * Some of the IP block revisions have an invert bit that swaps the high and
 * low parts of the output. This is used to implement waveforms with
 * duty_offset_ns + duty_length_ns == period_length_ns, i.e. what the legacy
 * API calls inverted polarity. On the other revisions a nonzero duty_offset_ns
 * is rounded down to zero.
 * Disabling the PWM stops the output immediately (without waiting for the
 * current period to complete first).
 *
 * The public S912 (GXM) datasheet contains some documentation for this PWM
 * controller starting on page 543:
 * https://dl.khadas.com/Hardware/VIM2/Datasheet/S912_Datasheet_V0.220170314publicversion-Wesion.pdf
 * An updated version of this IP block is found in S922X (G12B) SoCs. The
 * datasheet contains the description for this IP block revision starting at
 * page 1084:
 * https://dn.odroid.com/S922X/ODROID-N2/Datasheet/S922X_Public_Datasheet_V0.2.pdf
 *
 * Copyright (c) 2016 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2014 Amlogic, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define REG_PWM_A		0x0
#define REG_PWM_B		0x4
#define PWM_LOW_MASK		GENMASK(15, 0)
#define PWM_HIGH_MASK		GENMASK(31, 16)

#define REG_MISC_AB		0x8
#define MISC_B_CLK_EN_SHIFT	23
#define MISC_A_CLK_EN_SHIFT	15
#define MISC_CLK_DIV_WIDTH	7
#define MISC_B_CLK_DIV_SHIFT	16
#define MISC_A_CLK_DIV_SHIFT	8
#define MISC_B_CLK_SEL_SHIFT	6
#define MISC_A_CLK_SEL_SHIFT	4
#define MISC_CLK_SEL_MASK	0x3
#define MISC_B_CONSTANT_EN	BIT(29)
#define MISC_A_CONSTANT_EN	BIT(28)
#define MISC_B_INVERT_EN	BIT(27)
#define MISC_A_INVERT_EN	BIT(26)
#define MISC_B_EN		BIT(1)
#define MISC_A_EN		BIT(0)

#define MESON_NUM_PWMS		2
#define MESON_NUM_MUX_PARENTS	4

static struct meson_pwm_channel_data {
	u8		reg_offset;
	u8		clk_sel_shift;
	u8		clk_div_shift;
	u8		clk_en_shift;
	u32		pwm_en_mask;
	u32		const_en_mask;
	u32		inv_en_mask;
} meson_pwm_per_channel_data[MESON_NUM_PWMS] = {
	{
		.reg_offset	= REG_PWM_A,
		.clk_sel_shift	= MISC_A_CLK_SEL_SHIFT,
		.clk_div_shift	= MISC_A_CLK_DIV_SHIFT,
		.clk_en_shift	= MISC_A_CLK_EN_SHIFT,
		.pwm_en_mask	= MISC_A_EN,
		.const_en_mask	= MISC_A_CONSTANT_EN,
		.inv_en_mask	= MISC_A_INVERT_EN,
	},
	{
		.reg_offset	= REG_PWM_B,
		.clk_sel_shift	= MISC_B_CLK_SEL_SHIFT,
		.clk_div_shift	= MISC_B_CLK_DIV_SHIFT,
		.clk_en_shift	= MISC_B_CLK_EN_SHIFT,
		.pwm_en_mask	= MISC_B_EN,
		.const_en_mask	= MISC_B_CONSTANT_EN,
		.inv_en_mask	= MISC_B_INVERT_EN,
	}
};

struct meson_pwm_channel {
	struct clk_mux mux;
	struct clk_divider div;
	struct clk_gate gate;
	struct clk *clk;
};

struct meson_pwm_data {
	const char *const parent_names[MESON_NUM_MUX_PARENTS];
	int (*channels_init)(struct pwm_chip *chip);
	bool has_constant;
	bool has_polarity;
};

struct meson_pwm {
	const struct meson_pwm_data *data;
	struct meson_pwm_channel channels[MESON_NUM_PWMS];
	void __iomem *base;
	/*
	 * Protects register (write) access to the REG_MISC_AB register
	 * that is shared between the two PWMs.
	 */
	spinlock_t lock;
};

static inline struct meson_pwm *to_meson_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int meson_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct meson_pwm_channel *channel = &meson->channels[pwm->hwpwm];
	struct device *dev = pwmchip_parent(chip);
	int err;

	err = clk_prepare_enable(channel->clk);
	if (err < 0) {
		dev_err(dev, "failed to enable clock %s: %d\n",
			__clk_get_name(channel->clk), err);
		return err;
	}

	return 0;
}

static void meson_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct meson_pwm_channel *channel = &meson->channels[pwm->hwpwm];

	clk_disable_unprepare(channel->clk);
}

struct meson_pwm_waveform {
	unsigned long rate;
	u16 hi;
	u16 lo;
	bool enabled;
	bool inverted;
};

static int meson_pwm_round_waveform_tohw(struct pwm_chip *chip,
					 struct pwm_device *pwm,
					 const struct pwm_waveform *wf,
					 void *_wfhw)
{
	struct meson_pwm_waveform *wfhw = _wfhw;
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct meson_pwm_channel *channel = &meson->channels[pwm->hwpwm];
	u64 cnt, duty_cnt, freq;
	long fin_freq;
	int ret = 0;

	if (wf->period_length_ns == 0) {
		*wfhw = (struct meson_pwm_waveform){
			.enabled = false,
		};

		return 0;
	}

	/*
	 * Get the highest input clock rate that makes the requested period
	 * representable with the 16 bit wide counters, but at most 1 GHz so
	 * one counter cycle stays at least a nanosecond long, which
	 * .round_waveform_fromhw() relies on. clk_round_rate() may round up
	 * past the requested rate, so reject a too-high result below.
	 */
	freq = div64_u64((u64)NSEC_PER_SEC * FIELD_MAX(PWM_LOW_MASK),
			 wf->period_length_ns);
	freq = min_t(u64, freq, NSEC_PER_SEC);

	fin_freq = clk_round_rate(channel->clk, freq);
	if (fin_freq <= 0) {
		dev_err(pwmchip_parent(chip),
			"invalid source clock frequency %llu\n", freq);
		return fin_freq ? fin_freq : -EINVAL;
	}

	if (fin_freq > NSEC_PER_SEC) {
		dev_err(pwmchip_parent(chip),
			"source clock frequency %ld too high\n", fin_freq);
		return -EINVAL;
	}

	/*
	 * If the requested period is too long to be counted even at the
	 * lowest possible input clock rate, implement the longest possible
	 * period instead.
	 */
	cnt = mul_u64_u64_div_u64(fin_freq, wf->period_length_ns, NSEC_PER_SEC);
	cnt = min_t(u64, cnt, FIELD_MAX(PWM_LOW_MASK));

	if (cnt == 0) {
		/*
		 * The requested period is shorter than one cycle of the
		 * fastest available input clock, implement the minimal
		 * period with zero duty cycle (the general calculation below
		 * yields that as duty_length_ns and duty_offset_ns cannot
		 * exceed the requested period) and signal the rounding up
		 * in the return value.
		 */
		cnt = 1;
		ret = 1;
	}

	duty_cnt = mul_u64_u64_div_u64(fin_freq, wf->duty_length_ns, NSEC_PER_SEC);
	duty_cnt = min(duty_cnt, cnt);

	*wfhw = (struct meson_pwm_waveform){
		.rate = fin_freq,
		.hi = duty_cnt,
		.lo = cnt - duty_cnt,
		.enabled = true,
	};

	if (meson->data->has_polarity && duty_cnt) {
		u64 offset_cnt = mul_u64_u64_div_u64(fin_freq, wf->duty_offset_ns,
						     NSEC_PER_SEC);

		/*
		 * Waveforms with duty_offset_ns + duty_length_ns ==
		 * period_length_ns are implemented by inverting the output:
		 * The signal then is low for hi counts (the offset) followed
		 * by high for lo counts (the duty cycle). Other nonzero
		 * duty_offset_ns values are rounded down to 0.
		 */
		if (offset_cnt && duty_cnt + offset_cnt >= cnt) {
			swap(wfhw->hi, wfhw->lo);
			wfhw->inverted = true;
		}
	}

	dev_dbg(pwmchip_parent(chip),
		"pwm#%u: %llu/%llu [+%llu] @%lu -> hi: %u, lo: %u, inverted: %d\n",
		pwm->hwpwm, wf->duty_length_ns, wf->period_length_ns,
		wf->duty_offset_ns, wfhw->rate, wfhw->hi, wfhw->lo,
		wfhw->inverted);

	return ret;
}

static u64 meson_pwm_cnt_to_ns(unsigned long fin_freq, u32 cnt)
{
	return DIV64_U64_ROUND_UP(NSEC_PER_SEC * (u64)cnt, fin_freq);
}

static int meson_pwm_round_waveform_fromhw(struct pwm_chip *chip,
					   struct pwm_device *pwm,
					   const void *_wfhw,
					   struct pwm_waveform *wf)
{
	const struct meson_pwm_waveform *wfhw = _wfhw;

	if (!wfhw->enabled || !wfhw->rate) {
		*wf = (struct pwm_waveform){
			.period_length_ns = 0,
		};

		return 0;
	}

	/*
	 * wfhw->hi and wfhw->lo are u16, so their sum doesn't overflow and
	 * NSEC_PER_SEC * (hi + lo) fits into the u64 dividend in
	 * meson_pwm_cnt_to_ns().
	 */
	wf->period_length_ns = meson_pwm_cnt_to_ns(wfhw->rate, wfhw->hi + wfhw->lo);

	if (wfhw->inverted && wfhw->lo == 0) {
		/*
		 * The output is constant low. Report a normal constant-low
		 * waveform instead of duty_offset_ns == period_length_ns
		 * which wouldn't be a valid waveform description. This
		 * setting is only reachable when the hardware was programmed
		 * by the bootloader as .round_waveform_tohw() never produces
		 * it.
		 */
		wf->duty_length_ns = 0;
		wf->duty_offset_ns = 0;
	} else if (wfhw->inverted) {
		wf->duty_length_ns = meson_pwm_cnt_to_ns(wfhw->rate, wfhw->lo);
		wf->duty_offset_ns = meson_pwm_cnt_to_ns(wfhw->rate, wfhw->hi);
	} else {
		wf->duty_length_ns = meson_pwm_cnt_to_ns(wfhw->rate, wfhw->hi);
		wf->duty_offset_ns = 0;
	}

	return 0;
}

static int meson_pwm_read_waveform(struct pwm_chip *chip,
				   struct pwm_device *pwm,
				   void *_wfhw)
{
	struct meson_pwm_waveform *wfhw = _wfhw;
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct meson_pwm_channel *channel = &meson->channels[pwm->hwpwm];
	struct meson_pwm_channel_data *channel_data;
	u32 value, pwm_reg;

	channel_data = &meson_pwm_per_channel_data[pwm->hwpwm];

	value = readl(meson->base + REG_MISC_AB);
	if (!(value & channel_data->pwm_en_mask)) {
		*wfhw = (struct meson_pwm_waveform){
			.enabled = false,
		};

		return 0;
	}

	pwm_reg = readl(meson->base + channel_data->reg_offset);

	*wfhw = (struct meson_pwm_waveform){
		.rate = clk_get_rate(channel->clk),
		.hi = FIELD_GET(PWM_HIGH_MASK, pwm_reg),
		.lo = FIELD_GET(PWM_LOW_MASK, pwm_reg),
		.enabled = true,
		.inverted = meson->data->has_polarity &&
			    (value & channel_data->inv_en_mask),
	};

	return 0;
}

static int meson_pwm_write_waveform(struct pwm_chip *chip,
				    struct pwm_device *pwm,
				    const void *_wfhw)
{
	const struct meson_pwm_waveform *wfhw = _wfhw;
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct meson_pwm_channel *channel = &meson->channels[pwm->hwpwm];
	struct meson_pwm_channel_data *channel_data;
	unsigned long flags;
	u32 value;
	int err;

	channel_data = &meson_pwm_per_channel_data[pwm->hwpwm];

	/*
	 * Channel clock operations also modify REG_MISC_AB under meson->lock,
	 * including from .request()/.free() outside the pwmchip lock. Set the
	 * rate before taking meson->lock to avoid recursive locking. Skip
	 * clk_set_rate() when unchanged because even a no-op re-evaluates the
	 * mux parent and reparses the device tree.
	 */
	if (wfhw->enabled && clk_get_rate(channel->clk) != wfhw->rate) {
		err = clk_set_rate(channel->clk, wfhw->rate);
		if (err) {
			dev_err(pwmchip_parent(chip),
				"setting clock rate failed: %d\n", err);
			return err;
		}

		/*
		 * The rate computed by .round_waveform_tohw() might not be
		 * hit if an input clock changed its rate in between; the
		 * counter values only implement the promised waveform at
		 * exactly wfhw->rate.
		 */
		if (clk_get_rate(channel->clk) != wfhw->rate) {
			dev_err(pwmchip_parent(chip),
				"clock rate changed since rounding\n");
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&meson->lock, flags);

	if (wfhw->enabled) {
		value = FIELD_PREP(PWM_HIGH_MASK, wfhw->hi) |
			FIELD_PREP(PWM_LOW_MASK, wfhw->lo);
		writel(value, meson->base + channel_data->reg_offset);

		value = readl(meson->base + REG_MISC_AB);
		value |= channel_data->pwm_en_mask;

		if (meson->data->has_constant) {
			value &= ~channel_data->const_en_mask;
			/* The signal is constant when one phase is empty */
			if (!wfhw->hi || !wfhw->lo)
				value |= channel_data->const_en_mask;
		}

		if (meson->data->has_polarity) {
			value &= ~channel_data->inv_en_mask;
			if (wfhw->inverted)
				value |= channel_data->inv_en_mask;
		}

		writel(value, meson->base + REG_MISC_AB);
	} else {
		value = readl(meson->base + REG_MISC_AB);
		value &= ~channel_data->pwm_en_mask;
		writel(value, meson->base + REG_MISC_AB);
	}

	spin_unlock_irqrestore(&meson->lock, flags);

	return 0;
}

static const struct pwm_ops meson_pwm_ops = {
	.request = meson_pwm_request,
	.free = meson_pwm_free,
	.sizeof_wfhw = sizeof(struct meson_pwm_waveform),
	.round_waveform_tohw = meson_pwm_round_waveform_tohw,
	.round_waveform_fromhw = meson_pwm_round_waveform_fromhw,
	.read_waveform = meson_pwm_read_waveform,
	.write_waveform = meson_pwm_write_waveform,
};

static int meson_pwm_init_clocks_meson8b(struct pwm_chip *chip,
					 struct clk_parent_data *mux_parent_data)
{
	struct meson_pwm *meson = to_meson_pwm(chip);
	struct device *dev = pwmchip_parent(chip);
	unsigned int i;
	char name[255];
	int err;

	for (i = 0; i < MESON_NUM_PWMS; i++) {
		struct meson_pwm_channel *channel = &meson->channels[i];
		struct clk_parent_data div_parent = {}, gate_parent = {};
		struct clk_init_data init = {};

		snprintf(name, sizeof(name), "%s#mux%u", dev_name(dev), i);

		init.name = name;
		init.ops = &clk_mux_ops;
		init.flags = 0;
		init.parent_data = mux_parent_data;
		init.num_parents = MESON_NUM_MUX_PARENTS;

		channel->mux.reg = meson->base + REG_MISC_AB;
		channel->mux.shift =
				meson_pwm_per_channel_data[i].clk_sel_shift;
		channel->mux.mask = MISC_CLK_SEL_MASK;
		channel->mux.flags = 0;
		channel->mux.lock = &meson->lock;
		channel->mux.table = NULL;
		channel->mux.hw.init = &init;

		err = devm_clk_hw_register(dev, &channel->mux.hw);
		if (err)
			return dev_err_probe(dev, err,
					     "failed to register %s\n", name);

		snprintf(name, sizeof(name), "%s#div%u", dev_name(dev), i);

		init.name = name;
		init.ops = &clk_divider_ops;
		init.flags = CLK_SET_RATE_PARENT;
		div_parent.index = -1;
		div_parent.hw = &channel->mux.hw;
		init.parent_data = &div_parent;
		init.num_parents = 1;

		channel->div.reg = meson->base + REG_MISC_AB;
		channel->div.shift = meson_pwm_per_channel_data[i].clk_div_shift;
		channel->div.width = MISC_CLK_DIV_WIDTH;
		channel->div.hw.init = &init;
		channel->div.flags = 0;
		channel->div.lock = &meson->lock;

		err = devm_clk_hw_register(dev, &channel->div.hw);
		if (err)
			return dev_err_probe(dev, err,
					     "failed to register %s\n", name);

		snprintf(name, sizeof(name), "%s#gate%u", dev_name(dev), i);

		init.name = name;
		init.ops = &clk_gate_ops;
		init.flags = CLK_SET_RATE_PARENT | CLK_IGNORE_UNUSED;
		gate_parent.index = -1;
		gate_parent.hw = &channel->div.hw;
		init.parent_data = &gate_parent;
		init.num_parents = 1;

		channel->gate.reg = meson->base + REG_MISC_AB;
		channel->gate.bit_idx = meson_pwm_per_channel_data[i].clk_en_shift;
		channel->gate.hw.init = &init;
		channel->gate.flags = 0;
		channel->gate.lock = &meson->lock;

		err = devm_clk_hw_register(dev, &channel->gate.hw);
		if (err)
			return dev_err_probe(dev, err, "failed to register %s\n", name);

		channel->clk = devm_clk_hw_get_clk(dev, &channel->gate.hw, NULL);
		if (IS_ERR(channel->clk))
			return dev_err_probe(dev, PTR_ERR(channel->clk),
					     "failed to register %s\n", name);
	}

	return 0;
}

static int meson_pwm_init_channels_meson8b_legacy(struct pwm_chip *chip)
{
	struct clk_parent_data mux_parent_data[MESON_NUM_MUX_PARENTS] = {};
	struct meson_pwm *meson = to_meson_pwm(chip);
	int i;

	dev_warn_once(pwmchip_parent(chip),
		      "using obsolete compatible, please consider updating dt\n");

	for (i = 0; i < MESON_NUM_MUX_PARENTS; i++) {
		mux_parent_data[i].index = -1;
		mux_parent_data[i].name = meson->data->parent_names[i];
	}

	return meson_pwm_init_clocks_meson8b(chip, mux_parent_data);
}

static int meson_pwm_init_channels_meson8b_v2(struct pwm_chip *chip)
{
	struct clk_parent_data mux_parent_data[MESON_NUM_MUX_PARENTS] = {};
	int i;

	/*
	 * NOTE: Instead of relying on the hard coded names in the driver
	 * as the legacy version, this relies on DT to provide the list of
	 * clocks.
	 * For once, using input numbers actually makes more sense than names.
	 * Also DT requires clock-names to be explicitly ordered, so there is
	 * no point bothering with clock names in this case.
	 */
	for (i = 0; i < MESON_NUM_MUX_PARENTS; i++)
		mux_parent_data[i].index = i;

	return meson_pwm_init_clocks_meson8b(chip, mux_parent_data);
}

static void meson_pwm_s4_put_clk(void *data)
{
	struct clk *clk = data;

	clk_put(clk);
}

static int meson_pwm_init_channels_s4(struct pwm_chip *chip)
{
	struct device *dev = pwmchip_parent(chip);
	struct device_node *np = dev->of_node;
	struct meson_pwm *meson = to_meson_pwm(chip);
	int i, ret;

	for (i = 0; i < MESON_NUM_PWMS; i++) {
		meson->channels[i].clk = of_clk_get(np, i);
		if (IS_ERR(meson->channels[i].clk))
			return dev_err_probe(dev,
					     PTR_ERR(meson->channels[i].clk),
					     "Failed to get clk\n");

		ret = devm_add_action_or_reset(dev, meson_pwm_s4_put_clk,
					       meson->channels[i].clk);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to add clk_put action\n");
	}

	return 0;
}

static const struct meson_pwm_data pwm_meson8b_data = {
	.parent_names = { "xtal", NULL, "fclk_div4", "fclk_div3" },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
};

/*
 * Only the 2 first inputs of the GXBB AO PWMs are valid
 * The last 2 are grounded
 */
static const struct meson_pwm_data pwm_gxbb_ao_data = {
	.parent_names = { "xtal", "clk81", NULL, NULL },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
};

static const struct meson_pwm_data pwm_axg_ee_data = {
	.parent_names = { "xtal", "fclk_div5", "fclk_div4", "fclk_div3" },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_axg_ao_data = {
	.parent_names = { "xtal", "axg_ao_clk81", "fclk_div4", "fclk_div5" },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_g12a_ee_data = {
	.parent_names = { "xtal", NULL, "fclk_div4", "fclk_div3" },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_g12a_ao_ab_data = {
	.parent_names = { "xtal", "g12a_ao_clk81", "fclk_div4", "fclk_div5" },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_g12a_ao_cd_data = {
	.parent_names = { "xtal", "g12a_ao_clk81", NULL, NULL },
	.channels_init = meson_pwm_init_channels_meson8b_legacy,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_meson8_v2_data = {
	.channels_init = meson_pwm_init_channels_meson8b_v2,
};

static const struct meson_pwm_data pwm_meson_axg_v2_data = {
	.channels_init = meson_pwm_init_channels_meson8b_v2,
	.has_constant = true,
	.has_polarity = true,
};

static const struct meson_pwm_data pwm_s4_data = {
	.channels_init = meson_pwm_init_channels_s4,
	.has_constant = true,
	.has_polarity = true,
};

static const struct of_device_id meson_pwm_matches[] = {
	{
		.compatible = "amlogic,meson8-pwm-v2",
		.data = &pwm_meson8_v2_data
	}, {
		.compatible = "amlogic,meson-axg-pwm-v2",
		.data = &pwm_meson_axg_v2_data
	}, {
		.compatible = "amlogic,meson-g12-pwm-v2",
		.data = &pwm_meson_axg_v2_data
	},

	/* The following compatibles are obsolete */
	{
		.compatible = "amlogic,meson8b-pwm",
		.data = &pwm_meson8b_data
	},
	{
		.compatible = "amlogic,meson-gxbb-pwm",
		.data = &pwm_meson8b_data
	},
	{
		.compatible = "amlogic,meson-gxbb-ao-pwm",
		.data = &pwm_gxbb_ao_data
	},
	{
		.compatible = "amlogic,meson-axg-ee-pwm",
		.data = &pwm_axg_ee_data
	},
	{
		.compatible = "amlogic,meson-axg-ao-pwm",
		.data = &pwm_axg_ao_data
	},
	{
		.compatible = "amlogic,meson-g12a-ee-pwm",
		.data = &pwm_g12a_ee_data
	},
	{
		.compatible = "amlogic,meson-g12a-ao-pwm-ab",
		.data = &pwm_g12a_ao_ab_data
	},
	{
		.compatible = "amlogic,meson-g12a-ao-pwm-cd",
		.data = &pwm_g12a_ao_cd_data
	},
	{
		.compatible = "amlogic,meson-s4-pwm",
		.data = &pwm_s4_data
	},
	{ }
};
MODULE_DEVICE_TABLE(of, meson_pwm_matches);

static int meson_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct meson_pwm *meson;
	int err;

	chip = devm_pwmchip_alloc(&pdev->dev, MESON_NUM_PWMS, sizeof(*meson));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	meson = to_meson_pwm(chip);

	meson->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(meson->base))
		return PTR_ERR(meson->base);

	spin_lock_init(&meson->lock);
	chip->ops = &meson_pwm_ops;

	meson->data = of_device_get_match_data(&pdev->dev);

	err = meson->data->channels_init(chip);
	if (err < 0)
		return err;

	err = devm_pwmchip_add(&pdev->dev, chip);
	if (err < 0)
		return dev_err_probe(&pdev->dev, err,
				     "failed to register PWM chip\n");

	return 0;
}

static struct platform_driver meson_pwm_driver = {
	.driver = {
		.name = "meson-pwm",
		.of_match_table = meson_pwm_matches,
	},
	.probe = meson_pwm_probe,
};
module_platform_driver(meson_pwm_driver);

MODULE_DESCRIPTION("Amlogic Meson PWM Generator driver");
MODULE_AUTHOR("Neil Armstrong <narmstrong@baylibre.com>");
MODULE_LICENSE("Dual BSD/GPL");
