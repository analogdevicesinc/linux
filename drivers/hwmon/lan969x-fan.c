// SPDX-License-Identifier: GPL-2.0-only
/* Microchip LAN969x fan controller driver */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/thermal.h>

#include <dt-bindings/pwm/pwm.h>

#define FAN_CFG			0
#define  FAN_CFG_DUTY_CYCLE	GENMASK(23, 16)
#define  FAN_CFG_INV_POL		BIT(3)
#define  FAN_CFG_GATE_ENA	BIT(2)
#define  FAN_CFG_STAT_MODE	BIT(0)
#define FAN_PWM_FREQ		4
#define  FAN_PWM_FREQ_CYCLES	GENMASK(27, 16)
#define  FAN_PWM_FREQ_DIV	GENMASK(15, 0)
#define FAN_CNT			8
#define  FAN_CNT_DATA		GENMASK(15, 0)

#define FAN_TACH_INTERVAL_HZ	100000
#define FAN_PWM_FREQ_SCALE	256

struct lan969x_fan {
	void __iomem *base;
	struct clk *clk;
	struct device *hwmon_dev;
	unsigned long clk_rate;
	u32 pulses_per_revolution;
	u32 pwm_period;
	bool pwm_inverted;
	u32 *cooling_levels;
	unsigned long cooling_state;
	unsigned long max_cooling_state;
};

static void lan969x_fan_enable(struct lan969x_fan *fan)
{
	u32 cycles, val;

	cycles = DIV_ROUND_CLOSEST(fan->clk_rate, FAN_TACH_INTERVAL_HZ);
	cycles = clamp_val(cycles, 1, FIELD_MAX(FAN_PWM_FREQ_CYCLES));

	val = readl(fan->base + FAN_PWM_FREQ);
	val &= ~FAN_PWM_FREQ_CYCLES;
	val |= FIELD_PREP(FAN_PWM_FREQ_CYCLES, cycles);
	writel(val, fan->base + FAN_PWM_FREQ);

	/* Count all tachometer pulses over one-second intervals. */
	val = readl(fan->base + FAN_CFG);
	val &= ~(FAN_CFG_STAT_MODE | FAN_CFG_GATE_ENA);
	if (fan->pwm_period) {
		val &= ~FAN_CFG_INV_POL;
		if (fan->pwm_inverted)
			val |= FAN_CFG_INV_POL;
	}
	writel(val, fan->base + FAN_CFG);
}

static void lan969x_fan_update_cooling_state(struct lan969x_fan *fan, u32 pwm)
{
	unsigned long state;

	if (!fan->cooling_levels)
		return;

	for (state = 0; state < fan->max_cooling_state; state++)
		if (pwm < fan->cooling_levels[state + 1])
			break;

	fan->cooling_state = state;
}

static int lan969x_fan_read(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, long *val)
{
	struct lan969x_fan *fan = dev_get_drvdata(dev);
	unsigned long rate;
	u32 data;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input)
			break;
		data = readl_relaxed(fan->base + FAN_CNT);
		data = FIELD_GET(FAN_CNT_DATA, data);
		*val = data * 60 / fan->pulses_per_revolution;
		return 0;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			data = readl_relaxed(fan->base + FAN_CFG);
			*val = FIELD_GET(FAN_CFG_DUTY_CYCLE, data);
			return 0;
		case hwmon_pwm_freq:
			data = readl_relaxed(fan->base + FAN_PWM_FREQ);
			data = FIELD_GET(FAN_PWM_FREQ_DIV, data);
			if (!data)
				return -ENODATA;

			rate = DIV_ROUND_CLOSEST(fan->clk_rate,
						 FAN_PWM_FREQ_SCALE);
			*val = DIV_ROUND_CLOSEST(rate, data);
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static void lan969x_fan_set_pwm(struct lan969x_fan *fan, u32 pwm)
{
	u32 data;

	data = readl_relaxed(fan->base + FAN_CFG);
	data &= ~FAN_CFG_DUTY_CYCLE;
	data |= FIELD_PREP(FAN_CFG_DUTY_CYCLE, pwm);
	writel_relaxed(data, fan->base + FAN_CFG);
}

static int lan969x_fan_write_pwm(struct lan969x_fan *fan, long val)
{
	if (val < 0 || val > 255)
		return -EINVAL;

	lan969x_fan_set_pwm(fan, val);
	lan969x_fan_update_cooling_state(fan, val);

	return 0;
}

static void lan969x_fan_write_pwm_divider(struct lan969x_fan *fan, u32 divider)
{
	u32 data;

	data = readl_relaxed(fan->base + FAN_PWM_FREQ);
	data &= ~FAN_PWM_FREQ_DIV;
	data |= FIELD_PREP(FAN_PWM_FREQ_DIV, divider);
	writel_relaxed(data, fan->base + FAN_PWM_FREQ);
}

static int lan969x_fan_write_pwm_period(struct lan969x_fan *fan)
{
	unsigned long rate;
	u64 divider;

	rate = DIV_ROUND_CLOSEST(fan->clk_rate, FAN_PWM_FREQ_SCALE);
	divider = DIV_ROUND_CLOSEST_ULL((u64)rate * fan->pwm_period,
					NSEC_PER_SEC);
	if (!divider || divider > FIELD_MAX(FAN_PWM_FREQ_DIV))
		return -ERANGE;

	lan969x_fan_write_pwm_divider(fan, divider);

	return 0;
}

static int lan969x_fan_write_pwm_freq(struct lan969x_fan *fan, long val)
{
	unsigned long rate, divider;

	if (val <= 0)
		return -EINVAL;

	rate = DIV_ROUND_CLOSEST(fan->clk_rate, FAN_PWM_FREQ_SCALE);
	if (val < DIV_ROUND_CLOSEST(rate, FIELD_MAX(FAN_PWM_FREQ_DIV)) ||
	    val > rate)
		return -ERANGE;

	divider = DIV_ROUND_CLOSEST(rate, val);
	divider = min_t(unsigned long, divider, FIELD_MAX(FAN_PWM_FREQ_DIV));

	lan969x_fan_write_pwm_divider(fan, divider);

	return 0;
}

static int lan969x_fan_write(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long val)
{
	struct lan969x_fan *fan = dev_get_drvdata(dev);

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_pwm_input:
		return lan969x_fan_write_pwm(fan, val);
	case hwmon_pwm_freq:
		return lan969x_fan_write_pwm_freq(fan, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t lan969x_fan_is_visible(const void *_data,
				      enum hwmon_sensor_types type,
				      u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return attr == hwmon_fan_input ? 0444 : 0;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input || attr == hwmon_pwm_freq)
			return 0644;
		return 0;
	default:
		return 0;
	}
}

static const struct hwmon_channel_info * const lan969x_fan_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_FREQ),
	NULL
};

static const struct hwmon_ops lan969x_fan_hwmon_ops = {
	.is_visible = lan969x_fan_is_visible,
	.read = lan969x_fan_read,
	.write = lan969x_fan_write,
};

static const struct hwmon_chip_info lan969x_fan_chip_info = {
	.ops = &lan969x_fan_hwmon_ops,
	.info = lan969x_fan_info,
};

static int lan969x_fan_cooling_get_max_state(struct thermal_cooling_device *cdev,
					     unsigned long *state)
{
	struct lan969x_fan *fan = cdev->devdata;

	*state = fan->max_cooling_state;

	return 0;
}

static int lan969x_fan_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					     unsigned long *state)
{
	struct lan969x_fan *fan = cdev->devdata;

	hwmon_lock(fan->hwmon_dev);
	*state = fan->cooling_state;
	hwmon_unlock(fan->hwmon_dev);

	return 0;
}

static int lan969x_fan_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					      unsigned long state)
{
	struct lan969x_fan *fan = cdev->devdata;

	if (state > fan->max_cooling_state)
		return -EINVAL;

	hwmon_lock(fan->hwmon_dev);
	lan969x_fan_set_pwm(fan, fan->cooling_levels[state]);
	fan->cooling_state = state;
	hwmon_unlock(fan->hwmon_dev);

	return 0;
}

static const struct thermal_cooling_device_ops lan969x_fan_cooling_ops = {
	.get_max_state = lan969x_fan_cooling_get_max_state,
	.get_cur_state = lan969x_fan_cooling_get_cur_state,
	.set_cur_state = lan969x_fan_cooling_set_cur_state,
};

static int lan969x_fan_get_cooling_data(struct device *dev,
					struct device_node *fan_np,
					struct lan969x_fan *fan)
{
	u32 *levels;
	int count, i, ret;

	if (!of_property_present(fan_np, "cooling-levels"))
		return 0;

	count = of_property_count_u32_elems(fan_np, "cooling-levels");
	if (count <= 0)
		return dev_err_probe(dev, count ? : -EINVAL,
				     "invalid cooling-levels property\n");

	levels = devm_kcalloc(dev, count, sizeof(*levels), GFP_KERNEL);
	if (!levels)
		return -ENOMEM;
	fan->cooling_levels = levels;

	ret = of_property_read_u32_array(fan_np, "cooling-levels",
					 fan->cooling_levels, count);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read cooling-levels\n");

	for (i = 0; i < count; i++) {
		if (fan->cooling_levels[i] > 255)
			return dev_err_probe(dev, -EINVAL,
					     "cooling level %d exceeds 255\n", i);
		if (i && fan->cooling_levels[i] < fan->cooling_levels[i - 1])
			return dev_err_probe(dev, -EINVAL,
					     "cooling levels must be ascending\n");
	}

	fan->max_cooling_state = count - 1;

	return 0;
}

static int lan969x_fan_get_fan_data(struct device *dev,
				    struct device_node *fan_np,
				    struct lan969x_fan *fan)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_args(fan_np, "pwms", "#pwm-cells", 0,
					 &args);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse fan PWM\n");

	if (args.np != dev->of_node || args.args_count != 2 || !args.args[0] ||
	    args.args[1] & ~PWM_POLARITY_INVERTED) {
		of_node_put(args.np);
		return dev_err_probe(dev, -EINVAL, "invalid fan PWM specifier\n");
	}

	fan->pwm_period = args.args[0];
	fan->pwm_inverted = args.args[1] & PWM_POLARITY_INVERTED;
	of_node_put(args.np);

	if (of_property_present(fan_np, "pulses-per-revolution")) {
		ret = of_property_read_u32(fan_np, "pulses-per-revolution",
					   &fan->pulses_per_revolution);
		if (ret)
			return dev_err_probe(dev, ret,
					     "invalid pulses-per-revolution\n");
	}
	if (!fan->pulses_per_revolution || fan->pulses_per_revolution > 4)
		return dev_err_probe(dev, -EINVAL,
				     "invalid pulses-per-revolution\n");

	return lan969x_fan_get_cooling_data(dev, fan_np, fan);
}

static int lan969x_fan_probe(struct platform_device *pdev)
{
	struct thermal_cooling_device *cdev;
	struct device *dev = &pdev->dev;
	struct lan969x_fan *fan;
	struct device_node *fan_np __free(device_node) = NULL;
	u32 initial_pwm;
	int ret;

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;

	fan->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fan->base))
		return PTR_ERR(fan->base);

	fan->pulses_per_revolution = 2;
	fan_np = of_get_available_child_by_name(dev->of_node, "fan");
	if (fan_np) {
		ret = lan969x_fan_get_fan_data(dev, fan_np, fan);
		if (ret)
			return ret;
	}

	fan->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(fan->clk))
		return PTR_ERR(fan->clk);
	fan->clk_rate = clk_get_rate(fan->clk);
	if (!fan->clk_rate)
		return dev_err_probe(dev, -EINVAL, "invalid fan clock rate\n");

	if (fan->pwm_period) {
		ret = lan969x_fan_write_pwm_period(fan);
		if (ret)
			return dev_err_probe(dev, ret,
					     "invalid fan PWM period\n");
	}
	lan969x_fan_enable(fan);

	if (fan->cooling_levels) {
		initial_pwm = fan->cooling_levels[fan->max_cooling_state];
		ret = lan969x_fan_write_pwm(fan, initial_pwm);
		if (ret)
			return ret;
	}

	fan->hwmon_dev = devm_hwmon_device_register_with_info(dev, "lan969x_fan",
					fan, &lan969x_fan_chip_info, NULL);
	if (IS_ERR(fan->hwmon_dev))
		return PTR_ERR(fan->hwmon_dev);

	if (IS_ENABLED(CONFIG_THERMAL_OF) && fan->cooling_levels) {
		cdev = devm_thermal_of_child_cooling_device_register(dev, fan_np,
			"lan969x-fan", fan, &lan969x_fan_cooling_ops);
		if (IS_ERR(cdev))
			return dev_err_probe(dev, PTR_ERR(cdev),
					     "failed to register cooling device\n");
	}

	return 0;
}

static const struct of_device_id lan969x_fan_match[] = {
	{ .compatible = "microchip,lan9691-fan" },
	{},
};
MODULE_DEVICE_TABLE(of, lan969x_fan_match);

static struct platform_driver lan969x_fan_driver = {
	.probe = lan969x_fan_probe,
	.driver = {
		.name = "lan969x-fan",
		.of_match_table = lan969x_fan_match,
	},
};

module_platform_driver(lan969x_fan_driver);

MODULE_DESCRIPTION("Microchip LAN969x fan controller driver");
MODULE_AUTHOR("Robert Marko <robert.marko@sartura.hr>");
MODULE_LICENSE("GPL");
