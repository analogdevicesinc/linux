// SPDX-License-Identifier: GPL-2.0
/*
 * PWM fan controller driver for StarFive JHB100
 *
 * Copyright (C) 2018-2026 StarFive Technology Co., Ltd.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pwm.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#define STARFIVE_FAN_TACH_CH			16
#define STARFIVE_FAN_MAX			8
#define STARFIVE_FAN_TACH_PER_FAN		2
#define STARFIVE_FAN_PWM_VAL_MAX		255

/* Fan-tach register offset */
#define STARFIVE_FAN_TACH_STATUS		0x0c

#define STARFIVE_FAN_TACH_SPEED(ch)		(((ch) * 0x04) + 0x10)
#define STARFIVE_FAN_TACH_SPEED_VALID		BIT(31)
#define STARFIVE_FAN_TACH_VALUE_MASK		GENMASK(30, 0)

#define STARFIVE_FAN_TACH_THRESHOLD(ch)		(((ch) * 0x04) + 0x50)

#define STARFIVE_FAN_TACH_INT_EN		0x90
#define STARFIVE_FAN_TACH_STALL_INT_MASK	GENMASK(15, 0)
#define STARFIVE_FAN_TACH_SLOW_INT_MASK		GENMASK(31, 16)
#define STARFIVE_FAN_TACH_STALL_INT(ch)		BIT(ch)
#define STARFIVE_FAN_TACH_SLOW_INT(ch)		BIT((ch) + 16)

#define STARFIVE_FAN_TACH_MEASURE_TIME		0x94

#define STARFIVE_FAN_TACH_CH_EN			0x98
#define STARFIVE_FAN_TACH_EN(ch)		BIT(ch)

#define STARFIVE_FAN_DEFAULT_PULSE_PR		2
#define STARFIVE_FAN_DEFAULT_MEASURE_RATIO	10
#define STARFIVE_FAN_DEFAULT_RPM_PAUSE_TIME	(60 * STARFIVE_FAN_DEFAULT_MEASURE_RATIO)

#define STARFIVE_FAN_TACH_TIMEOUT \
	(USEC_PER_SEC / STARFIVE_FAN_DEFAULT_MEASURE_RATIO)

#define STARFIVE_FAN_TACH_TIMEOUT_JIFFIES \
	(msecs_to_jiffies(1000) / STARFIVE_FAN_DEFAULT_MEASURE_RATIO)

#define FAN_ATTRIBUTE_SET \
	(HWMON_F_INPUT | HWMON_F_MIN | HWMON_F_ENABLE | \
	 HWMON_F_FAULT | HWMON_F_MIN_ALARM)

struct starfive_pwm_fan {
	struct pwm_device *pwm;
	struct pwm_state pwm_state;
	unsigned int pwm_value;
};

struct starfive_fan_tach_ctx {
	u32 ch_en;
	u32 measure_time;
	u32 threshold[STARFIVE_FAN_TACH_CH];
};

struct starfive_pwm_fan_data {
	struct device *dev;
	void __iomem *regs;
	struct reset_control *rst;
	struct clk *clk;
	/* Serializes PWM updates with PM operations */
	struct mutex pwm_lock;
	struct starfive_pwm_fan fans[STARFIVE_FAN_MAX];
	bool tach_present[STARFIVE_FAN_TACH_CH];
	u8 pulses_per_rev[STARFIVE_FAN_TACH_CH];
	struct completion comp_stall[STARFIVE_FAN_TACH_CH];
	struct completion comp_slow[STARFIVE_FAN_TACH_CH];
	u8 fan_stall[STARFIVE_FAN_TACH_CH];
	u8 fan_slow[STARFIVE_FAN_TACH_CH];
	u32 armed_stall;
	u32 armed_slow;
	/* tach registers saved across system sleep */
	struct starfive_fan_tach_ctx tach_ctx;
	/* protects fan_stall[]/fan_slow[]/armed_* and the STATUS register */
	spinlock_t lock;
};

static void starfive_fan_tach_ch_enable(struct starfive_pwm_fan_data *priv, u8 tach_ch,
					bool enable)
{
	if (enable) {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_CH_EN) |
		       STARFIVE_FAN_TACH_EN(tach_ch),
		       priv->regs + STARFIVE_FAN_TACH_CH_EN);
	} else {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_CH_EN) &
		       ~(STARFIVE_FAN_TACH_EN(tach_ch)),
		       priv->regs + STARFIVE_FAN_TACH_CH_EN);
	}
}

static void starfive_fan_tach_ch_stall_unmask(struct starfive_pwm_fan_data *priv, u8 tach_ch,
					      bool unmask)
{
	if (unmask) {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_INT_EN) |
		       STARFIVE_FAN_TACH_STALL_INT(tach_ch),
		       priv->regs + STARFIVE_FAN_TACH_INT_EN);
	} else {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_INT_EN) &
		       ~STARFIVE_FAN_TACH_STALL_INT(tach_ch),
		       priv->regs + STARFIVE_FAN_TACH_INT_EN);
	}
}

static void starfive_fan_tach_ch_slow_unmask(struct starfive_pwm_fan_data *priv, u8 tach_ch,
					     bool unmask)
{
	if (unmask) {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_INT_EN) |
		       STARFIVE_FAN_TACH_SLOW_INT(tach_ch),
		       priv->regs + STARFIVE_FAN_TACH_INT_EN);
	} else {
		writel(readl(priv->regs + STARFIVE_FAN_TACH_INT_EN) &
		       ~STARFIVE_FAN_TACH_SLOW_INT(tach_ch),
		       priv->regs + STARFIVE_FAN_TACH_INT_EN);
	}
}

static u32 starfive_fan_tach_rpm_to_val(struct starfive_pwm_fan_data *priv, u8 tach_ch, u32 rpm)
{
	u64 tach_val;
	u8 ppr;

	ppr = priv->pulses_per_rev[tach_ch];

	tach_val = (u64)rpm * ppr;
	tach_val = div_u64(tach_val, STARFIVE_FAN_DEFAULT_RPM_PAUSE_TIME);

	return min_t(u64, tach_val, STARFIVE_FAN_TACH_VALUE_MASK);
}

static long starfive_fan_tach_val_to_rpm(struct starfive_pwm_fan_data *priv, u8 tach_ch,
					 u32 tach_val)
{
	u64 rpm;
	u8 ppr;

	ppr = priv->pulses_per_rev[tach_ch];

	rpm = (u64)tach_val * STARFIVE_FAN_DEFAULT_RPM_PAUSE_TIME;

	return div_u64(rpm, ppr);
}

static int starfive_fan_tach_get_rpm(struct starfive_pwm_fan_data *priv,
				     u8 tach_ch, long *rpm)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(priv->regs + STARFIVE_FAN_TACH_SPEED(tach_ch),
				 val, val & STARFIVE_FAN_TACH_SPEED_VALID,
				 100, STARFIVE_FAN_TACH_TIMEOUT);
	if (ret)
		return -ENODATA;

	val = FIELD_GET(STARFIVE_FAN_TACH_VALUE_MASK, val);
	if (!val) {
		/* Sampling = 0 may mean no valid capture; wait 1.5 more cycles for valid read. */
		fsleep(STARFIVE_FAN_TACH_TIMEOUT + STARFIVE_FAN_TACH_TIMEOUT / 2);
		ret = readl_poll_timeout(priv->regs + STARFIVE_FAN_TACH_SPEED(tach_ch),
					 val, val & STARFIVE_FAN_TACH_SPEED_VALID,
					 100, STARFIVE_FAN_TACH_TIMEOUT);
		if (ret)
			return -ENODATA;

		val = FIELD_GET(STARFIVE_FAN_TACH_VALUE_MASK, val);
	}

	*rpm = starfive_fan_tach_val_to_rpm(priv, tach_ch, val);

	return 0;
}

static long starfive_fan_tach_get_rpm_threshold(struct starfive_pwm_fan_data *priv,
						u8 tach_ch)
{
	u32 val;

	val = readl(priv->regs + STARFIVE_FAN_TACH_THRESHOLD(tach_ch));

	val = FIELD_GET(STARFIVE_FAN_TACH_VALUE_MASK, val);

	return starfive_fan_tach_val_to_rpm(priv, tach_ch, val);
}

static void starfive_set_tach_rpm_threshold(struct starfive_pwm_fan_data *priv,
					    u8 tach_ch, u32 val)
{
	u32 reg_val;

	reg_val = starfive_fan_tach_rpm_to_val(priv, tach_ch, val);
	reg_val = clamp_val(reg_val, 1, STARFIVE_FAN_TACH_VALUE_MASK);

	writel(reg_val, priv->regs + STARFIVE_FAN_TACH_THRESHOLD(tach_ch));
}

static bool starfive_fan_tach_get_ch_en(struct starfive_pwm_fan_data *priv,
					u8 tach_ch)
{
	u32 enable;

	enable = readl(priv->regs + STARFIVE_FAN_TACH_CH_EN);

	return !!(enable & STARFIVE_FAN_TACH_EN(tach_ch));
}

static int starfive_pwm_fan_set_pwm(struct starfive_pwm_fan_data *priv,
				    int channel, unsigned int pwm)
{
	struct starfive_pwm_fan *fan = &priv->fans[channel];
	struct pwm_state state;
	int ret;

	guard(mutex)(&priv->pwm_lock);

	state = fan->pwm_state;
	state.duty_cycle = DIV_ROUND_UP_ULL((u64)pwm * state.period,
					    STARFIVE_FAN_PWM_VAL_MAX);
	/*
	 * Keep the PWM enabled even at zero duty cycle so the output stays
	 * driven low instead of being left floating by the PWM hardware.
	 */
	state.enabled = true;

	ret = pwm_apply_might_sleep(fan->pwm, &state);
	if (ret)
		return ret;

	fan->pwm_state = state;
	fan->pwm_value = pwm;

	return 0;
}

static int starfive_pwm_fan_pwm_read(struct starfive_pwm_fan_data *priv,
				     u32 attr, int channel, long *val)
{
	switch (attr) {
	case hwmon_pwm_input:
		scoped_guard(mutex, &priv->pwm_lock) {
			*val = priv->fans[channel].pwm_value;
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int starfive_pwm_fan_fan_read(struct starfive_pwm_fan_data *priv,
				     u32 attr, int channel, long *val)
{
	int ret;

	switch (attr) {
	case hwmon_fan_fault:
		scoped_guard(spinlock_irqsave, &priv->lock) {
			writel(STARFIVE_FAN_TACH_STALL_INT(channel),
			       priv->regs + STARFIVE_FAN_TACH_STATUS);
			/* clear fan_stall first */
			priv->fan_stall[channel] = 0;
			reinit_completion(&priv->comp_stall[channel]);
			priv->armed_stall |= BIT(channel);
		}

		starfive_fan_tach_ch_stall_unmask(priv, channel, true);

		/* Waiting for hardware to measure */
		wait_for_completion_timeout(&priv->comp_stall[channel],
					    2 * STARFIVE_FAN_TACH_TIMEOUT_JIFFIES);

		starfive_fan_tach_ch_stall_unmask(priv, channel, false);

		scoped_guard(spinlock_irqsave, &priv->lock) {
			priv->armed_stall &= ~BIT(channel);
			*val = priv->fan_stall[channel];
		}

		break;
	case hwmon_fan_input:
		if (!starfive_fan_tach_get_ch_en(priv, channel))
			return -ENODATA;

		ret = starfive_fan_tach_get_rpm(priv, channel, val);
		if (ret < 0)
			return ret;

		break;
	case hwmon_fan_min:
		*val = starfive_fan_tach_get_rpm_threshold(priv, channel);

		break;
	case hwmon_fan_min_alarm:
		scoped_guard(spinlock_irqsave, &priv->lock) {
			writel(STARFIVE_FAN_TACH_SLOW_INT(channel),
			       priv->regs + STARFIVE_FAN_TACH_STATUS);
			/* clear fan_slow first */
			priv->fan_slow[channel] = 0;
			reinit_completion(&priv->comp_slow[channel]);
			priv->armed_slow |= BIT(channel);
		}

		starfive_fan_tach_ch_slow_unmask(priv, channel, true);

		/* Waiting for hardware to measure */
		wait_for_completion_timeout(&priv->comp_slow[channel],
					    2 * STARFIVE_FAN_TACH_TIMEOUT_JIFFIES);

		starfive_fan_tach_ch_slow_unmask(priv, channel, false);

		scoped_guard(spinlock_irqsave, &priv->lock) {
			priv->armed_slow &= ~BIT(channel);
			*val = priv->fan_slow[channel];
		}

		break;
	case hwmon_fan_enable:
		*val = starfive_fan_tach_get_ch_en(priv, channel);

		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int starfive_pwm_fan_read(struct device *dev,
				 enum hwmon_sensor_types type, u32 attr,
				 int channel, long *val)
{
	struct starfive_pwm_fan_data *priv = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		return starfive_pwm_fan_pwm_read(priv, attr, channel, val);
	case hwmon_fan:
		return starfive_pwm_fan_fan_read(priv, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int starfive_pwm_fan_write(struct device *dev,
				  enum hwmon_sensor_types type, u32 attr,
				  int channel, long val)
{
	struct starfive_pwm_fan_data *priv = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_pwm:
		if (attr != hwmon_pwm_input)
			return -EOPNOTSUPP;

		if (val < 0 || val > STARFIVE_FAN_PWM_VAL_MAX)
			return -EINVAL;

		return starfive_pwm_fan_set_pwm(priv, channel, val);
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_min:
			if (val < 0 || val > U32_MAX)
				return -EINVAL;

			starfive_set_tach_rpm_threshold(priv, channel, val);

			return 0;
		case hwmon_fan_enable:
			if (val != 0 && val != 1)
				return -EINVAL;

			starfive_fan_tach_ch_enable(priv, channel, val);

			return 0;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t starfive_pwm_fan_is_visible(const void *drvdata,
					   enum hwmon_sensor_types type,
					   u32 attr, int channel)
{
	const struct starfive_pwm_fan_data *priv = drvdata;

	switch (type) {
	case hwmon_pwm:
		if (!priv->fans[channel].pwm)
			return 0;

		if (attr == hwmon_pwm_input)
			return 0644;

		return 0;
	case hwmon_fan:
		if (!priv->tach_present[channel])
			return 0;

		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_fault:
		case hwmon_fan_min_alarm:
			return 0444;
		case hwmon_fan_min:
		case hwmon_fan_enable:
			return 0644;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static const struct hwmon_ops starfive_pwm_fan_ops = {
	.is_visible = starfive_pwm_fan_is_visible,
	.read = starfive_pwm_fan_read,
	.write = starfive_pwm_fan_write,
};

static const struct hwmon_channel_info * const starfive_pwm_fan_info[] = {
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT, HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT, HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT, HWMON_PWM_INPUT,
			   HWMON_PWM_INPUT, HWMON_PWM_INPUT),
	HWMON_CHANNEL_INFO(fan,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET,
			   FAN_ATTRIBUTE_SET, FAN_ATTRIBUTE_SET),
	NULL
};

static const struct hwmon_chip_info starfive_pwm_fan_chip_info = {
	.ops = &starfive_pwm_fan_ops,
	.info = starfive_pwm_fan_info,
};

static int starfive_pwm_fan_create_fan(struct device *dev, struct device_node *child,
				       struct starfive_pwm_fan_data *priv,
				       unsigned int fan_idx)
{
	struct starfive_pwm_fan *fan = &priv->fans[fan_idx];
	u8 tach_ch[STARFIVE_FAN_TACH_PER_FAN];
	struct pwm_device *pwm;
	int ret, count;
	u32 ppr, index;

	count = of_property_count_u8_elems(child, "tach-ch");
	if (count < 1 || count > STARFIVE_FAN_TACH_PER_FAN)
		return dev_err_probe(dev, -EINVAL, "%pOF: invalid tach-ch\n", child);

	ret = of_property_read_u8_array(child, "tach-ch", tach_ch, count);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: failed to read tach-ch\n", child);

	/* Parse pulses-per-revolution, default to 2 if not specified */
	ppr = STARFIVE_FAN_DEFAULT_PULSE_PR;
	of_property_read_u32(child, "pulses-per-revolution", &ppr);

	if (!ppr || ppr > 4)
		return dev_err_probe(dev, -EINVAL,
				     "%pOF: invalid pulses-per-revolution %u, must be 1-4\n",
				     child, ppr);

	pwm = devm_fwnode_pwm_get(dev, of_fwnode_handle(child), NULL);
	if (IS_ERR(pwm))
		return dev_err_probe(dev, PTR_ERR(pwm), "%pOF: could not get PWM\n", child);

	pwm_init_state(pwm, &fan->pwm_state);
	if (!fan->pwm_state.period)
		return dev_err_probe(dev, -EINVAL, "%pOF: PWM period is zero\n", child);

	fan->pwm = pwm;

	for (index = 0; index < count; index++) {
		u8 ch = tach_ch[index];

		if (ch >= STARFIVE_FAN_TACH_CH)
			return dev_err_probe(dev, -EINVAL, "%pOF: invalid tach-ch %u\n",
					     child, ch);

		if (priv->tach_present[ch])
			return dev_err_probe(dev, -EINVAL, "%pOF: duplicate tach-ch %u\n",
					     child, ch);

		priv->pulses_per_rev[ch] = ppr;
		priv->tach_present[ch] = true;
		starfive_fan_tach_ch_enable(priv, ch, true);
	}

	return 0;
}

static irqreturn_t starfive_pwm_fan_irq(int irq, void *dev_id)
{
	struct starfive_pwm_fan_data *priv = dev_id;
	u32 fan_status;
	int i;

	guard(spinlock)(&priv->lock);

	fan_status = readl(priv->regs + STARFIVE_FAN_TACH_STATUS);
	if (!fan_status)
		return IRQ_NONE;

	writel(fan_status, priv->regs + STARFIVE_FAN_TACH_STATUS);

	for (i = 0; i < STARFIVE_FAN_TACH_CH; i++) {
		if (!priv->tach_present[i])
			continue;

		if ((fan_status & STARFIVE_FAN_TACH_STALL_INT(i)) &&
		    (priv->armed_stall & BIT(i))) {
			priv->fan_stall[i] = 1;
			complete(&priv->comp_stall[i]);
		}

		if ((fan_status & STARFIVE_FAN_TACH_SLOW_INT(i)) &&
		    (priv->armed_slow & BIT(i))) {
			priv->fan_slow[i] = 1;
			complete(&priv->comp_slow[i]);
		}
	}

	return IRQ_HANDLED;
}

static int starfive_pwm_fan_pwm_disable_all(struct starfive_pwm_fan_data *priv)
{
	struct pwm_state state;
	int i, ret, err = 0;

	guard(mutex)(&priv->pwm_lock);

	for (i = 0; i < STARFIVE_FAN_MAX; i++) {
		struct starfive_pwm_fan *fan = &priv->fans[i];

		if (!fan->pwm || !fan->pwm_state.enabled)
			continue;

		state = fan->pwm_state;
		state.duty_cycle = 0;
		state.enabled = false;

		/* Keep going so one broken channel does not leave others running */
		ret = pwm_apply_might_sleep(fan->pwm, &state);
		if (ret) {
			dev_err(priv->dev, "Failed to disable PWM of fan %d: %d\n",
				i, ret);
			if (!err)
				err = ret;
		}
	}

	return err;
}

static int starfive_pwm_fan_pwm_restore_all(struct starfive_pwm_fan_data *priv)
{
	int i, ret, err = 0;

	guard(mutex)(&priv->pwm_lock);

	for (i = 0; i < STARFIVE_FAN_MAX; i++) {
		struct starfive_pwm_fan *fan = &priv->fans[i];

		if (!fan->pwm)
			continue;

		ret = pwm_apply_might_sleep(fan->pwm, &fan->pwm_state);
		if (ret) {
			dev_err(priv->dev, "Failed to restore PWM of fan %d: %d\n",
				i, ret);
			if (!err)
				err = ret;
		}
	}

	return err;
}

static void starfive_pwm_fan_pwm_disable(void *data)
{
	struct starfive_pwm_fan_data *priv = data;

	starfive_pwm_fan_pwm_disable_all(priv);
}

static void starfive_fan_tach_save(struct starfive_pwm_fan_data *priv)
{
	struct starfive_fan_tach_ctx *ctx = &priv->tach_ctx;
	int i;

	ctx->ch_en = readl(priv->regs + STARFIVE_FAN_TACH_CH_EN);
	ctx->measure_time = readl(priv->regs + STARFIVE_FAN_TACH_MEASURE_TIME);

	for (i = 0; i < STARFIVE_FAN_TACH_CH; i++) {
		if (priv->tach_present[i])
			ctx->threshold[i] =
				readl(priv->regs + STARFIVE_FAN_TACH_THRESHOLD(i));
	}
}

static void starfive_fan_tach_restore(struct starfive_pwm_fan_data *priv)
{
	struct starfive_fan_tach_ctx *ctx = &priv->tach_ctx;
	int i;

	writel(0, priv->regs + STARFIVE_FAN_TACH_INT_EN);
	writel(STARFIVE_FAN_TACH_STALL_INT_MASK | STARFIVE_FAN_TACH_SLOW_INT_MASK,
	       priv->regs + STARFIVE_FAN_TACH_STATUS);
	writel(ctx->measure_time, priv->regs + STARFIVE_FAN_TACH_MEASURE_TIME);

	for (i = 0; i < STARFIVE_FAN_TACH_CH; i++) {
		if (priv->tach_present[i])
			writel(ctx->threshold[i],
			       priv->regs + STARFIVE_FAN_TACH_THRESHOLD(i));
	}

	/* Enable channels last so measurement starts with the restored config */
	writel(ctx->ch_en, priv->regs + STARFIVE_FAN_TACH_CH_EN);
}

static void starfive_fan_tach_disable(void *data)
{
	struct starfive_pwm_fan_data *priv = data;

	clk_disable_unprepare(priv->clk);
	reset_control_assert(priv->rst);
}

static int starfive_pwm_fan_probe(struct platform_device *pdev)
{
	struct device *hwmon_dev, *dev = &pdev->dev;
	struct starfive_pwm_fan_data *priv;
	unsigned int fan_idx;
	unsigned long clk_rate;
	int irq;
	int ret;
	u32 i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->lock);

	ret = devm_mutex_init(dev, &priv->pwm_lock);
	if (ret)
		return ret;

	for (i = 0; i < STARFIVE_FAN_TACH_CH; i++) {
		init_completion(&priv->comp_stall[i]);
		init_completion(&priv->comp_slow[i]);
	}

	platform_set_drvdata(pdev, priv);

	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs))
		return dev_err_probe(dev, PTR_ERR(priv->regs),
				     "Unable to map IO resources\n");

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "Unable to get fan tach's clock\n");

	priv->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->rst))
		return dev_err_probe(dev, PTR_ERR(priv->rst),
				     "Unable to get fan tach's reset\n");

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clock\n");

	clk_rate = clk_get_rate(priv->clk);
	if (!clk_rate || clk_rate > U32_MAX) {
		clk_disable_unprepare(priv->clk);
		return dev_err_probe(dev, -EINVAL, "Invalid clock rate: %lu\n",
				     clk_rate);
	}

	ret = reset_control_deassert(priv->rst);
	if (ret) {
		clk_disable_unprepare(priv->clk);
		return dev_err_probe(dev, ret, "Failed to deassert reset\n");
	}

	ret = devm_add_action_or_reset(dev, starfive_fan_tach_disable, priv);
	if (ret)
		return ret;

	/* Mask all sources before touching status / requesting the IRQ. */
	writel(0, priv->regs + STARFIVE_FAN_TACH_INT_EN);
	writel(STARFIVE_FAN_TACH_STALL_INT_MASK | STARFIVE_FAN_TACH_SLOW_INT_MASK,
	       priv->regs + STARFIVE_FAN_TACH_STATUS);
	writel(clk_rate / STARFIVE_FAN_DEFAULT_MEASURE_RATIO,
	       priv->regs + STARFIVE_FAN_TACH_MEASURE_TIME);

	fan_idx = 0;
	for_each_available_child_of_node_scoped(dev->of_node, child) {
		if (fan_idx >= STARFIVE_FAN_MAX)
			return dev_err_probe(dev, -EINVAL,
					     "Too many fan nodes, at most %d\n",
					     STARFIVE_FAN_MAX);

		ret = starfive_pwm_fan_create_fan(dev, child, priv, fan_idx++);
		if (ret)
			return ret;
	}

	/* Must be registered after the PWMs are requested */
	ret = devm_add_action_or_reset(dev, starfive_pwm_fan_pwm_disable, priv);
	if (ret)
		return ret;

	for (i = 0; i < STARFIVE_FAN_MAX; i++) {
		if (!priv->fans[i].pwm)
			continue;

		ret = starfive_pwm_fan_set_pwm(priv, i, STARFIVE_FAN_PWM_VAL_MAX);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to configure PWM of fan %u\n", i);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Failed to get IRQ\n");

	ret = devm_request_irq(dev, irq, starfive_pwm_fan_irq, 0, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "starfive_pwm_fan",
							 priv, &starfive_pwm_fan_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(hwmon_dev),
				     "Failed to register hwmon device\n");

	return 0;
}

static int starfive_pwm_fan_suspend(struct device *dev)
{
	struct starfive_pwm_fan_data *priv = dev_get_drvdata(dev);
	int ret;

	ret = starfive_pwm_fan_pwm_disable_all(priv);
	if (ret) {
		/* ->resume() is not called for us on failure, undo here */
		starfive_pwm_fan_pwm_restore_all(priv);
		return ret;
	}

	starfive_fan_tach_save(priv);

	return 0;
}

static int starfive_pwm_fan_resume(struct device *dev)
{
	struct starfive_pwm_fan_data *priv = dev_get_drvdata(dev);

	starfive_fan_tach_restore(priv);

	return starfive_pwm_fan_pwm_restore_all(priv);
}

static DEFINE_SIMPLE_DEV_PM_OPS(starfive_pwm_fan_pm,
				starfive_pwm_fan_suspend,
				starfive_pwm_fan_resume);

static const struct of_device_id starfive_pwm_fan_of_match[] = {
	{ .compatible = "starfive,jhb100-pwm-fan", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, starfive_pwm_fan_of_match);

static struct platform_driver starfive_pwm_fan_driver = {
	.probe = starfive_pwm_fan_probe,
	.driver	= {
		.name = "starfive-pwm-fan",
		.pm = pm_sleep_ptr(&starfive_pwm_fan_pm),
		.of_match_table = starfive_pwm_fan_of_match,
	},
};

module_platform_driver(starfive_pwm_fan_driver);

MODULE_AUTHOR("William Qiu <william.qiu@starfivetech.com>");
MODULE_AUTHOR("Changhuang Liang <changhuang.liang@starfivetech.com>");
MODULE_DESCRIPTION("StarFive JHB100 PWM fan controller driver");
MODULE_LICENSE("GPL");
