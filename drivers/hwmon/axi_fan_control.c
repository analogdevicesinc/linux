// SPDX-License-Identifier: GPL-2.0
/*
 * Fan Control HDL CORE driver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/* register map */
#define ADI_REG_RSTN		0x0080
#define ADI_REG_PWM_WIDTH	0x0084
#define ADI_REG_TACH_PERIOD	0x0088
#define ADI_REG_TACH_TOLERANCE	0x008c
#define ADI_REG_PWM_PERIOD	0x00c0
#define ADI_REG_TACH_MEASUR	0x00c4
#define ADI_REG_TEMPERATURE	0x00c8

#define ADI_REG_IRQ_MASK	0x0040
#define ADI_REG_IRQ_PENDING	0x0044
#define ADI_REG_IRQ_SRC		0x0048

/* IRQ sources */
#define ADI_IRQ_SRC_PWM_CHANGED		(1 << 0)
#define ADI_IRQ_SRC_TACH_ERR		(1 << 1)
#define ADI_IRQ_SRC_TEMP_INCREASE	(1 << 2)
#define ADI_IRQ_SRC_NEW_MEASUR		(1 << 3)
#define ADI_IRQ_SRC_MASK		GENMASK(3, 0)
#define ADI_IRQ_MASK_OUT_ALL		0xFFFFFFFFU

#define SYSFS_PWM_MAX			255

struct axi_fan_control_data {
	struct clk *clk;
	void __iomem *base;
	atomic_t tach;	/* tacho period */
	int irq;
	u32 ppr;	/* pulses per revolution */
	bool hw_pwm_req;
	bool update_tacho_params;
	u8 fan_fault;
};

static inline void axi_fan_control_iowrite(const u32 val, const u32 reg,
				const struct axi_fan_control_data *ctl)
{
	iowrite32(val, ctl->base + reg);
}

static inline u32 axi_fan_control_ioread(const u32 reg,
				const struct axi_fan_control_data *ctl)
{
	return ioread32(ctl->base + reg);
}

static long axi_fan_control_get_pwm_duty(const struct axi_fan_control_data *ctl)
{
	u32 pwm_width = axi_fan_control_ioread(ADI_REG_PWM_WIDTH, ctl);
	u32 pwm_period = axi_fan_control_ioread(ADI_REG_PWM_PERIOD, ctl);

	return DIV_ROUND_CLOSEST(pwm_width * SYSFS_PWM_MAX, pwm_period);
}

static int axi_fan_control_set_pwm_duty(const long val,
					struct axi_fan_control_data *ctl)
{
	u32 pwm_period = axi_fan_control_ioread(ADI_REG_PWM_PERIOD, ctl);
	u32 new_width;

	/*
	 * As stated in the hwmon sysfs-interface, the pwm value should be
	 * in the range of [0-255].
	 */
	if (val < 0 || val > SYSFS_PWM_MAX)
		return -EINVAL;

	new_width = DIV_ROUND_CLOSEST(val * pwm_period, SYSFS_PWM_MAX);

	axi_fan_control_iowrite(new_width, ADI_REG_PWM_WIDTH, ctl);

	return 0;
}

static long axi_fan_control_get_fan_rpm(const struct axi_fan_control_data *ctl)
{
	unsigned long clk_rate = clk_get_rate(ctl->clk);

	if (!clk_rate)
		return -EINVAL;

	/*
	 * The tacho period should be:
	 *      TACH = 60/(ppr * rpm), where rpm is revolutions per second
	 *      and ppr is pulses per revolution.
	 * Given the tacho period, we can multiply it by the input clock
	 * so that we know how many clocks we need to have this period.
	 * From this, we can derive the RPM value.
	 */
	return DIV_ROUND_CLOSEST(60 * clk_rate,
				 ctl->ppr * atomic_read(&ctl->tach));
}

static int axi_fan_control_read_temp(struct device *dev, u32 attr, long *val)
{
	struct axi_fan_control_data *ctl = dev_get_drvdata(dev);
	long raw_temp;

	switch (attr) {
	case hwmon_temp_input:
		raw_temp = axi_fan_control_ioread(ADI_REG_TEMPERATURE, ctl);
		/*
		 * The formula for the temperature is:
		 *      T = (ADC * 501.3743 / 2^bits) - 273.6777
		 * It's multiplied by 1000 to have milidegrees as
		 * specified by the hwmon sysfs interface.
		 */
		*val = ((raw_temp * 501374) >> 16) - 273677;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_read_fan(struct device *dev, u32 attr, long *val)
{
	struct axi_fan_control_data *ctl = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_fan_fault:
		*val = ctl->fan_fault;
		return 0;
	case hwmon_fan_input:
		*val = axi_fan_control_get_fan_rpm(ctl);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_read_pwm(struct device *dev, u32 attr, long *val)
{
	struct axi_fan_control_data *ctl = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_pwm_input:
		*val = axi_fan_control_get_pwm_duty(ctl);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_write_pwm(struct device *dev, u32 attr, long val)
{
	struct axi_fan_control_data *ctl = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_pwm_input:
		return axi_fan_control_set_pwm_duty(val, ctl);
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_read_labels(struct device *dev,
				       enum hwmon_sensor_types type,
				       u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_fan:
		*str = "SOM FAN";
		return 0;
	case hwmon_temp:
		*str = "SYSMON4";
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_read(struct device *dev,
				enum hwmon_sensor_types type,
				u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return axi_fan_control_read_fan(dev, attr, val);
	case hwmon_pwm:
		return axi_fan_control_read_pwm(dev, attr, val);
	case hwmon_temp:
		return axi_fan_control_read_temp(dev, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_fan_control_write(struct device *dev,
				 enum hwmon_sensor_types type,
				 u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_pwm:
		return axi_fan_control_write_pwm(dev, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t axi_fan_control_fan_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_fan_input:
	case hwmon_fan_fault:
	case hwmon_fan_label:
		return 0444;
	default:
		return 0;
	}
}

static umode_t axi_fan_control_pwm_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_pwm_input:
		return 0644;
	default:
		return 0;
	}
}

static umode_t axi_fan_control_temp_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_label:
		return 0444;
	default:
		return 0;
	}
}

static umode_t axi_fan_control_is_visible(const void *data,
					  enum hwmon_sensor_types type,
					  u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return axi_fan_control_fan_is_visible(attr);
	case hwmon_pwm:
		return axi_fan_control_pwm_is_visible(attr);
	case hwmon_temp:
		return axi_fan_control_temp_is_visible(attr);
	default:
		return 0;
	}
}

static irqreturn_t axi_fan_control_irq_handler(int irq, void *data)
{
	struct axi_fan_control_data *ctl = (struct axi_fan_control_data *)data;
	u32 irq_pending = axi_fan_control_ioread(ADI_REG_IRQ_PENDING, ctl);
	u32 clear_mask;

	if (irq_pending & ADI_IRQ_SRC_NEW_MEASUR) {
		u32 new_tach = axi_fan_control_ioread(ADI_REG_TACH_MEASUR,
						      ctl);

		if (ctl->update_tacho_params == true) {
			/* get 25% tolerance */
			u32 tach_tol = DIV_ROUND_CLOSEST(new_tach * 25, 100);
			/* set new tacho parameters */
			axi_fan_control_iowrite(new_tach,
					ADI_REG_TACH_PERIOD, ctl);
			axi_fan_control_iowrite(tach_tol,
					ADI_REG_TACH_TOLERANCE, ctl);
			ctl->update_tacho_params = false;
		}

		atomic_set(&ctl->tach, new_tach);
	}

	if (irq_pending & ADI_IRQ_SRC_PWM_CHANGED) {
		/*
		 * if the pwm changes on behalf of software,
		 * we need to provide new tacho parameters to the core.
		 * Wait for the next measurement for that...
		 */
		if (!ctl->hw_pwm_req)
			ctl->update_tacho_params = true;

		/* just set this to false even if it is already... */
		ctl->hw_pwm_req = false;
	}

	if (irq_pending & ADI_IRQ_SRC_TEMP_INCREASE)
		/* hardware requested a new pwm */
		ctl->hw_pwm_req = true;

	if (irq_pending & ADI_IRQ_SRC_TACH_ERR)
		ctl->fan_fault = 1;

	/* clear all interrupts */
	clear_mask = irq_pending & ADI_IRQ_SRC_MASK;
	axi_fan_control_iowrite(clear_mask, ADI_REG_IRQ_PENDING, ctl);

	return IRQ_HANDLED;
}

static int axi_fan_control_init(struct axi_fan_control_data *ctl,
				const struct device_node *np)
{
	int ret;

	/* get fan pulses per revolution */
	ret = of_property_read_u32(np, "adi,pulses-per-revolution", &ctl->ppr);
	if (ret)
		return ret;
	/*
	 * Enable all IRQs
	 */
	axi_fan_control_iowrite((ADI_IRQ_MASK_OUT_ALL &
			~(ADI_IRQ_SRC_NEW_MEASUR | ADI_IRQ_SRC_TACH_ERR |
			ADI_IRQ_SRC_PWM_CHANGED | ADI_IRQ_SRC_TEMP_INCREASE)),
			ADI_REG_IRQ_MASK, ctl);

	/* bring the device out of reset */
	axi_fan_control_iowrite(0x01, ADI_REG_RSTN, ctl);

	return ret;
}

static const u32 axi_fan_control_temp_config[] = {
	HWMON_T_INPUT | HWMON_T_LABEL,
	0
};

static const struct hwmon_channel_info axi_fan_control_temp = {
	.type = hwmon_temp,
	.config = axi_fan_control_temp_config,
};

static const u32 axi_fan_control_fan_config[] = {
	HWMON_F_INPUT | HWMON_F_FAULT | HWMON_F_LABEL,
	0
};

static const struct hwmon_channel_info axi_fan_control_fan = {
	.type = hwmon_fan,
	.config = axi_fan_control_fan_config,
};

static const u32 axi_fan_control_pwm_config[] = {
	HWMON_PWM_INPUT,
	0
};

static const struct hwmon_channel_info axi_fan_control_pwm = {
	.type = hwmon_pwm,
	.config = axi_fan_control_pwm_config,
};

static const struct hwmon_channel_info *axi_fan_control_info[] = {
	&axi_fan_control_pwm,
	&axi_fan_control_fan,
	&axi_fan_control_temp,
	NULL
};

static const struct hwmon_ops axi_fan_control_hwmon_ops = {
	.is_visible = axi_fan_control_is_visible,
	.read = axi_fan_control_read,
	.write = axi_fan_control_write,
	.read_string = axi_fan_control_read_labels,
};

static const struct hwmon_chip_info axi_fan_control_chip_info = {
	.ops = &axi_fan_control_hwmon_ops,
	.info = axi_fan_control_info,
};

static const u32 version_1_0_0 = ADI_AXI_PCORE_VER(1, 0, 'a');

static const struct of_device_id axi_fan_control_of_match[] = {
	{ .compatible = "adi,axi-fan-control-1.00.a",
		.data = (void *)&version_1_0_0},
	{},
};
MODULE_DEVICE_TABLE(of, axi_fan_control_of_match);

static int axi_fan_control_probe(struct platform_device *pdev)
{
	struct device *hwmon_dev;
	struct axi_fan_control_data *ctl;
	const struct of_device_id *id;
	struct resource *res;
	u32 version;
	int ret;

	id = of_match_node(axi_fan_control_of_match, pdev->dev.of_node);
	if (!id)
		return -EINVAL;

	ctl = devm_kzalloc(&pdev->dev, sizeof(*ctl), GFP_KERNEL);
	if (!ctl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctl->base)) {
		dev_err(&pdev->dev, "ioremap failed with %ld\n",
			PTR_ERR(ctl->base));
		return PTR_ERR(ctl->base);
	}

	ctl->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ctl->clk)) {
		dev_err(&pdev->dev, "clk_get failed with %ld\n",
			PTR_ERR(ctl->clk));
		return PTR_ERR(ctl->clk);
	}

	dev_info(&pdev->dev, "Re-mapped from 0x%08llX to %p\n",
			(unsigned long long)res->start, ctl->base);

	version = axi_fan_control_ioread(ADI_AXI_REG_VERSION, ctl);
	if (ADI_AXI_PCORE_VER_MAJOR(version) !=
	    ADI_AXI_PCORE_VER_MAJOR((*(u32 *)id->data))) {
		dev_err(&pdev->dev, "Major version mismatch. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_MINOR((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_PATCH((*(u32 *)id->data)),
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));
		return -ENODEV;
	}

	ctl->irq = platform_get_irq(pdev, 0);
	if (ctl->irq < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed with %d\n",
				ctl->irq);
		return ctl->irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, ctl->irq, NULL,
				axi_fan_control_irq_handler, IRQF_ONESHOT |
				IRQF_TRIGGER_HIGH, pdev->driver_override, ctl);
	if (ret) {
		dev_err(&pdev->dev, "failed to request an irq, %d", ret);
		return ret;
	}

	ret = axi_fan_control_init(ctl, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize device\n");
		return ret;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
						"axi_fan_control",
						ctl,
						&axi_fan_control_chip_info,
						NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static struct platform_driver axi_fan_control_driver = {
	.driver = {
		.name = "axi_fan_control_driver",
		.of_match_table = axi_fan_control_of_match,
	},
	.probe = axi_fan_control_probe,
};
module_platform_driver(axi_fan_control_driver);


MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices Fan Control HDL CORE driver");
MODULE_LICENSE("GPL");
