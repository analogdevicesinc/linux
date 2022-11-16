// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Rotary counter driver for Analog Devices SC5XX Processors
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/platform_data/adi_rotary.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/portmux.h>
#include <mach/gpio.h>
#else
#include <asm/portmux.h>
#endif

#define CNT_CONFIG_OFF		0	/* CNT Config Offset */
#define CNT_IMASK_OFF		4	/* CNT Interrupt Mask Offset */
#define CNT_STATUS_OFF		8	/* CNT Status Offset */
#define CNT_COMMAND_OFF		12	/* CNT Command Offset */
#define CNT_DEBOUNCE_OFF	16	/* CNT Debounce Offset */
#define CNT_COUNTER_OFF		20	/* CNT Counter Offset */
#define CNT_MAX_OFF		24	/* CNT Maximum Count Offset */
#define CNT_MIN_OFF		28	/* CNT Minimum Count Offset */

enum button_state {IDLE, ACTIVE};
enum boundary_mode {
	BND_COMP,			/* Boundary Compare Mode */
	BND_ZERO,			/* Boundary Zero mode */
	BND_CAPT,			/* Boundary Capture Mode */
	BND_AEXT,			/* Boundary Auto-extend Mode */
};

struct adi_rot {
	struct input_dev *input;
	void __iomem *base;
	int irq;
	unsigned int up_key;
	unsigned int down_key;
	unsigned int button_key;
	unsigned int abs_code;

	unsigned short mode;
	unsigned short debounce;
	int thwl_en_gpio, gpio_act_flg;

	unsigned short cnt_config;
	unsigned short cnt_imask;
	unsigned short cnt_debounce;
	enum button_state but_state;
	int cnt_range_state;
	int cnt_upper_range;
	int cnt_lower_range;
};

static void rotary_wheel_count_event(struct adi_rot *rotary, int delta)
{
	input_report_abs(rotary->input, rotary->abs_code, delta);
	input_sync(rotary->input);
}

static void rotary_wheel_key_event(struct input_dev *input, int keycode)
{
	/* Simulate a press-n-release */
	input_report_key(input, keycode, 1);
	input_sync(input);
	input_report_key(input, keycode, 0);
	input_sync(input);
}

static void report_rotary_wheel_event(struct adi_rot *rotary)
{
	int delta = readl(rotary->base + CNT_COUNTER_OFF);

	if (rotary->up_key)
		rotary_wheel_key_event(rotary->input,
				 delta > 0 ? rotary->up_key : rotary->down_key);
	else
		rotary_wheel_count_event(rotary, delta);
}

static void report_rotary_button_event(struct adi_rot *rotary)
{
	switch (rotary->but_state) {
	case IDLE:
		/* Button pressed event report*/
		input_report_key(rotary->input, rotary->button_key, 1);
		input_sync(rotary->input);
		rotary->but_state = ACTIVE;
		rotary->cnt_config = readw(rotary->base + CNT_CONFIG_OFF);
		writew(rotary->cnt_config & ~CNTE, rotary->base + CNT_CONFIG_OFF);
		/* CZM pin polarity invert */
		rotary->cnt_config ^= (1 << 6);
		writew(rotary->cnt_config | CNTE, rotary->base + CNT_CONFIG_OFF);
		break;

	case ACTIVE:
		/* Button released event report */
		input_report_key(rotary->input, rotary->button_key, 0);
		input_sync(rotary->input);
		rotary->but_state = IDLE;
		rotary->cnt_config = readw(rotary->base + CNT_CONFIG_OFF);
		writew(rotary->cnt_config & ~CNTE, rotary->base + CNT_CONFIG_OFF);
		/* CZM pin polarity invert */
		rotary->cnt_config ^= (1 << 6);
		writew(rotary->cnt_config | CNTE, rotary->base + CNT_CONFIG_OFF);
		break;

	default:
		break;
	}
}

static int adi_rotary_boundary_mask(struct adi_rot *rotary, int cnt_status)
{
	enum boundary_mode bnd_mode;

	bnd_mode = (rotary->mode & BNDMODE) >> BNDMODE_SHIFT;
	switch (bnd_mode) {
	case BND_COMP:
		{
			int max_value = readl(rotary->base + CNT_MAX_OFF);
			int min_value = readl(rotary->base + CNT_MIN_OFF);

			if (max_value <= min_value)
				return -EINVAL;

			switch (cnt_status) {
			case UCII | MAXCII:
			case UCII | MAXCII | CZEROII:
				writew(W1LCNT_MAX, rotary->base + CNT_COMMAND_OFF);
				rotary->cnt_range_state &= ~MINCII;
				break;

			case DCII | MINCII:
			case DCII | MINCII | CZEROII:
				writew(W1LCNT_MIN, rotary->base + CNT_COMMAND_OFF);
				rotary->cnt_range_state &= ~MAXCII;
				break;

			case DCII | MAXCII:
			case DCII | MAXCII | CZEROII:
				writew(MAXCII, rotary->base + CNT_STATUS_OFF);
				rotary->cnt_range_state &= ~MAXCII;
				break;

			case UCII | MINCII:
			case UCII | MINCII | CZEROII:
				writew(MINCII, rotary->base + CNT_STATUS_OFF);
				rotary->cnt_range_state &= ~MINCII;
				break;

			case UCII:
			case DCII:
				writew(MAXCII | MINCII, rotary->base + CNT_STATUS_OFF);
				rotary->cnt_range_state &= ~(MINCII | MAXCII);
				break;

			case CZMZII:
			case CZMZII | CZMII:
			default:
				if (cnt_status & CZMZII) {
					if (rotary->cnt_upper_range < 0) {
						rotary->cnt_range_state |= MAXCII;
						rotary->cnt_range_state &= ~MINCII;
					} else if (rotary->cnt_lower_range > 0) {
						rotary->cnt_range_state |= MINCII;
						rotary->cnt_range_state &= ~MAXCII;
					}
				}
				break;
			}
		}
		break;

	case BND_ZERO:
		{
			int max_value = readl(rotary->base + CNT_MAX_OFF);
			int min_value = readl(rotary->base + CNT_MIN_OFF);

			if (max_value < 0 || min_value > 0 || max_value == min_value)
				return -EINVAL;
			writew(MAXCII | MINCII, rotary->base + CNT_STATUS_OFF);
		}
		break;

	default:
		writew(MAXCII | MINCII, rotary->base + CNT_STATUS_OFF);
		break;
	}
	return 0;
}

static irqreturn_t adi_rotary_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct adi_rot *rotary = platform_get_drvdata(pdev);
	unsigned int cnt_status;

	cnt_status = readw(rotary->base + CNT_STATUS_OFF);
	cnt_status |= rotary->cnt_range_state;

	if (adi_rotary_boundary_mask(rotary, cnt_status)) {
		/* IRQ handle error and clear STATUS */
		writew(0xffff, rotary->base + CNT_STATUS_OFF);
		return IRQ_NONE;
	}

	if (cnt_status & (UCII | DCII))
		report_rotary_wheel_event(rotary);
	else if (cnt_status & CZMII)
		report_rotary_button_event(rotary);

	/* Clear STATUS and COUNTER after every IRQ
	 * NOTE:
	 * 1. To report the delta of the counts, clean the count register
	 *	  after each counting.
	 * 2. To report the sum of the counts, keep it changed by events.
	 */
	if (rotary->up_key) {
		writew(W1LCNT_ZERO, rotary->base + CNT_COMMAND_OFF);
		writew(0xffff, rotary->base + CNT_STATUS_OFF);
	} else
		writew(0xffff & ~(MAXCII | MINCII), rotary->base + CNT_STATUS_OFF);

	return IRQ_HANDLED;
}

static int adi_rotary_open(struct input_dev *input)
{
	struct adi_rot *rotary = input_get_drvdata(input);
	enum boundary_mode bnd_mode;
	unsigned short val;

	/* Debounce enable/disable */
	if (rotary->mode & DEBE)
		writew(rotary->debounce & DPRESCALE, rotary->base + CNT_DEBOUNCE_OFF);

	writew(rotary->mode & ~CNTE, rotary->base + CNT_CONFIG_OFF);

	/* Configure boundary mode */
	bnd_mode = (rotary->mode & BNDMODE) >> BNDMODE_SHIFT;
	switch (bnd_mode) {
	case BND_COMP:
	case BND_ZERO:
		writel(rotary->cnt_upper_range, rotary->base + CNT_MAX_OFF);
		writel(rotary->cnt_lower_range, rotary->base + CNT_MIN_OFF);
		if (rotary->cnt_upper_range < 0) {
			rotary->cnt_range_state |= MAXCII;
			rotary->cnt_range_state &= ~MINCII;
		} else if (rotary->cnt_lower_range > 0) {
			rotary->cnt_range_state |= MINCII;
			rotary->cnt_range_state &= ~MAXCII;
		}
		break;

	default:
		break;
	}

	/* configure and enable Counter */
	val = UCIE | DCIE;
	if (rotary->button_key)
		val |= CZMIE;
	writew(val, rotary->base + CNT_IMASK_OFF);
	writew(rotary->mode | CNTE, rotary->base + CNT_CONFIG_OFF);

	return 0;
}

static void adi_rotary_close(struct input_dev *input)
{
	struct adi_rot *rotary = input_get_drvdata(input);
	unsigned int val;

	/* Clear Counter/Maximum/Minimum count register */
	val = W1LCNT_ZERO | W1LMIN_ZERO | W1LMAX_ZERO;
	writew(val, rotary->base + CNT_COMMAND_OFF);
	writew(0xffff, rotary->base + CNT_STATUS_OFF);
	writew(0, rotary->base + CNT_CONFIG_OFF);
	writew(0, rotary->base + CNT_IMASK_OFF);
	rotary->cnt_range_state &= ~(MAXCII | MINCII);
	rotary->but_state = IDLE;
}

static void adi_rotary_free_action(void *data)
{
	unsigned short *pin_list = (unsigned short *)data;

	peripheral_free_list(pin_list);
}

#ifdef CONFIG_OF
static const struct of_device_id adi_rotary_of_match[] = {
	{ .compatible = "adi,gp_counter" },
	{},
};
MODULE_DEVICE_TABLE(of, adi_rotary_of_match);

static int adi_rotary_parse_dt(struct device *dev,
							   struct adi_rotary_platform_data *pdata)
{
	struct device_node *node;
	int error;
	u16 val;

	/* Platform data configuration from device tree node */
	node = dev->of_node;

	if (!node)
		return -EINVAL;

	/* Rotary features supported and configuration */
	of_property_read_u32(node, "rotary_up_key", &pdata->rotary_up_key);
	of_property_read_u32(node, "rotary_down_key", &pdata->rotary_down_key);
	of_property_read_u32(node, "rotary_abs_code", &pdata->rotary_abs_code);
	error = of_property_read_u32(node, "rotary_button_key",
				&pdata->rotary_button_key);
	if (error)
		pdata->rotary_button_key = KEY_ENTER;

	/* Parse rotary debounce mode configuration field */
	of_property_read_u16(node, "debounce_en", &val);
	pdata->mode |= (val << 1) & DEBE;
	if (pdata->mode & DEBE)
		of_property_read_u16(node, "debounce", &pdata->debounce);

	/* Parse CZM/CUD/CDG Pin polarity invert value field */
	of_property_read_u16(node, "invert_cdg", &val);
	pdata->mode |= (val << 4) & CDGINV;
	of_property_read_u16(node, "invert_cud", &val);
	pdata->mode |= (val << 5) & CUDINV;
	of_property_read_u16(node, "invert_czm", &val);
	pdata->mode |= (val << 6) & CZMINV;

	/* Parse rotary counter operating mode field */
	of_property_read_u16(node, "cnt_mode", &val);
	pdata->mode |= (val << CNTMODE_SHIFT) & CNTMODE;

	of_property_read_u16(node, "czm_zero_cnt_en", &val);
	pdata->mode |= (val << 11) & ZMZC;

	/* Parse boundary register mode field */
	of_property_read_u16(node, "boundary_mode", &val);
	pdata->mode |= (val << BNDMODE_SHIFT) & BNDMODE;
	if (((pdata->mode & BNDMODE) == BNDMODE_COMP) ||
				((pdata->mode & BNDMODE) == BNDMODE_ZERO)) {
		error = of_property_read_u32_array(node, "boundary_ranges",
					pdata->cnt_bound_ranges,
					ARRAY_SIZE(pdata->cnt_bound_ranges));
		if (!error) {
			if (pdata->cnt_bound_ranges[0] < CNT_MINIMUM_LOWER_RANGE)
				pdata->cnt_bound_ranges[0] = CNT_MINIMUM_LOWER_RANGE;

			if (pdata->cnt_bound_ranges[1] > CNT_MAXIMUM_UPPER_RANGE)
				pdata->cnt_bound_ranges[1] = CNT_MAXIMUM_UPPER_RANGE;

			if (pdata->cnt_bound_ranges[0] >= pdata->cnt_bound_ranges[1]) {
				pdata->cnt_bound_ranges[0] = CNT_MINIMUM_LOWER_RANGE;
				pdata->cnt_bound_ranges[1] = CNT_MAXIMUM_UPPER_RANGE;
			}
		} else {
			pdata->cnt_bound_ranges[0] = CNT_MINIMUM_LOWER_RANGE;
			pdata->cnt_bound_ranges[1] = CNT_MAXIMUM_UPPER_RANGE;
		}
	} else {
		pdata->cnt_bound_ranges[0] = CNT_MINIMUM_LOWER_RANGE;
		pdata->cnt_bound_ranges[1] = CNT_MAXIMUM_UPPER_RANGE;
	}

	/* Power management configuration for rotary device */
	of_property_read_u16(node, "pm_wakeup", &pdata->pm_wakeup);
	/* Peripheral devices configuration */
	of_property_read_u16(node, "pin_list", pdata->pin_list);

	return 0;
}
#endif

static int adi_rotary_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_rotary_platform_data *pdata;
	struct adi_rot *rotary;
	struct resource *res;
	const struct of_device_id *match;
	struct input_dev *input;
	int error;

	match = of_match_device(of_match_ptr(adi_rotary_of_match), dev);
	if (match) {
		pdata = devm_kzalloc(dev,
							sizeof(struct adi_rotary_platform_data),
							GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		error = adi_rotary_parse_dt(dev, pdata);
		if (error) {
			dev_err(dev, "failed to parse platform data from device tree\n");
			return error;
		}
	} else
		pdata  = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "fail to get platform data\n");
		return -EINVAL;
	}

	/*
	 *  Rotary basic validation
	 */

	rotary = devm_kzalloc(dev, sizeof(struct adi_rot), GFP_KERNEL);
	if (!rotary)
		return -ENOMEM;

	/* Get resources from platform data */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rotary->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(rotary->base))
		return PTR_ERR(rotary->base);

	/* Up and down key should be configured both simultaneously */
	if ((pdata->rotary_up_key && !pdata->rotary_down_key) ||
		(!pdata->rotary_up_key && pdata->rotary_down_key)) {
		return -EINVAL;
	}

	/* Initialize field of Peripheral devices */
	if (pdata->pin_list) {
		error = peripheral_request_list(pdata->pin_list,
						dev_name(dev));
		if (error) {
			dev_err(dev, "requesting peripherals failed: %d\n", error);
			return error;
		}

		error = devm_add_action_or_reset(dev, adi_rotary_free_action,
						pdata->pin_list);
		if (error) {
			dev_err(dev, "setting cleanup action failed: %d\n", error);
			return error;
		}
	}

	/* Enable rotary hardware device via active GPIO */
	if (likely(of_count_phandle_with_args(dev->of_node,
						"enable-pin", NULL) > 0)) {
		error = softconfig_of_set_active_pin_output(dev, dev->of_node,
					"enable-pin", 0, &pdata->thwl_en_gpio,
					&pdata->gpio_active_flags, true);
		if (error)
			return error;
	}

	/* Allocate rotary managed input device */
	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "unable to allocate input device\n");
		return -ENOMEM;
	}

	/* Initialize field of rotary device */
	rotary->input			= input;
	rotary->up_key			= pdata->rotary_up_key;
	rotary->down_key		= pdata->rotary_down_key;
	rotary->button_key		= pdata->rotary_button_key;
	rotary->abs_code		= pdata->rotary_abs_code;
	rotary->mode			= pdata->mode;
	rotary->debounce		= pdata->debounce;
	rotary->thwl_en_gpio	= pdata->thwl_en_gpio;
	rotary->gpio_act_flg	= pdata->gpio_active_flags;
	rotary->cnt_lower_range	= pdata->cnt_bound_ranges[0];
	rotary->cnt_upper_range	= pdata->cnt_bound_ranges[1];

	/* Initialize field of input device */
	input->name				= pdev->name;
	input->phys				= "adi-rotary/input0";
	input->dev.parent		= dev;

	input->id.bustype		= BUS_HOST;
	input->id.vendor		= 0x0001;
	input->id.product		= 0x0001;
	input->id.version		= 0x0100;

	input->open				= adi_rotary_open;
	input->close			= adi_rotary_close;

	input_set_drvdata(input, rotary);

	if (rotary->up_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(rotary->up_key, input->keybit);
		__set_bit(rotary->down_key, input->keybit);
	} else {
		__set_bit(EV_ABS, input->evbit);
		input_set_abs_params(rotary->input, rotary->abs_code,
					rotary->cnt_lower_range, rotary->cnt_upper_range, 0, 0);
	}

	if (rotary->button_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(rotary->button_key, input->keybit);
	}

	/* Quiesce the device before requesting irq */
	adi_rotary_close(input);

	rotary->irq = platform_get_irq(pdev, 0);
	if (rotary->irq < 0) {
		dev_err(dev, "No rotary IRQ specified\n");
		return -ENOENT;
	}

	error = devm_request_irq(dev, rotary->irq, adi_rotary_isr, 0, dev_name(dev), pdev);
	if (error) {
		dev_err(dev, "unable to claim irq %d; error %d\n", rotary->irq, error);
		return error;
	}

	/* Register input device */
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "unable to register input device (%d)\n", error);
		return error;
	}

	platform_set_drvdata(pdev, rotary);
	device_init_wakeup(dev, pdata->pm_wakeup);
	devm_kfree(dev, pdata);
	return 0;
}

static int adi_rotary_remove(struct platform_device *pdev)
{
	struct adi_rot *rotary = platform_get_drvdata(pdev);

	writew(0, rotary->base + CNT_CONFIG_OFF);
	writew(0, rotary->base + CNT_IMASK_OFF);

	if (rotary->thwl_en_gpio && gpio_is_valid(rotary->thwl_en_gpio))
		gpio_direction_output(rotary->thwl_en_gpio,
					rotary->gpio_act_flg ? 1 : 0);

	return 0;
}

static int __maybe_unused adi_rotary_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct adi_rot *rotary = platform_get_drvdata(pdev);

	rotary->cnt_config = readw(rotary->base + CNT_CONFIG_OFF);
	rotary->cnt_imask = readw(rotary->base + CNT_IMASK_OFF);
	rotary->cnt_debounce = readw(rotary->base + CNT_DEBOUNCE_OFF);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(rotary->irq);

	return 0;
}

static int __maybe_unused adi_rotary_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct adi_rot *rotary = platform_get_drvdata(pdev);

	writew(rotary->cnt_debounce, rotary->base + CNT_DEBOUNCE_OFF);
	writew(rotary->cnt_imask, rotary->base + CNT_IMASK_OFF);
	writew(rotary->cnt_config & ~CNTE, rotary->base + CNT_CONFIG_OFF);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(rotary->irq);

	if (rotary->cnt_config & CNTE)
		writew(rotary->cnt_config, rotary->base + CNT_CONFIG_OFF);

	return 0;
}

static const struct dev_pm_ops adi_rotary_pm_ops = {
	.suspend	= adi_rotary_suspend,
	.resume		= adi_rotary_resume,
};

static struct platform_driver adi_rotary_device_driver = {
	.probe		= adi_rotary_probe,
	.remove		= adi_rotary_remove,
	.driver		= {
		.name	= "adi-rotary",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(adi_rotary_of_match),
#ifdef CONFIG_PM
		.pm	= &adi_rotary_pm_ops,
#endif
	},
};
module_platform_driver(adi_rotary_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Rotary Counter driver for ADI Processors");
MODULE_ALIAS("platform:adi-rotary");
