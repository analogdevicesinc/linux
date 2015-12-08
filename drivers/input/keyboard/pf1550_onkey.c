/*
 * Driver for the PF1550 ON_KEY
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/pf1550.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct onkey_drv_data {
	struct device *dev;
	struct pf1550_dev *pf1550;
	int irq;
	int keycode;
	int wakeup;
	struct input_dev *input;
};

static struct pf1550_irq_info pf1550_onkey_irqs[] = {
	{ PF1550_ONKEY_IRQ_PUSHI,	"release" },
	{ PF1550_ONKEY_IRQ_1SI,		"1S" },
	{ PF1550_ONKEY_IRQ_2SI,		"2S" },
	{ PF1550_ONKEY_IRQ_3SI,		"3S" },
	{ PF1550_ONKEY_IRQ_4SI,		"4S" },
	{ PF1550_ONKEY_IRQ_8SI,		"8S" },
};

static irqreturn_t pf1550_onkey_irq_handler(int irq, void *data)
{
	struct onkey_drv_data *onkey = data;
	int i, state, irq_type = -1;

	onkey->irq = irq;

	for (i = 0; i < ARRAY_SIZE(pf1550_onkey_irqs); i++)
		if (onkey->irq == pf1550_onkey_irqs[i].virq)
			irq_type = pf1550_onkey_irqs[i].irq;
	switch (irq_type) {
	case PF1550_ONKEY_IRQ_PUSHI:
		state = 0;
		break;
	case PF1550_ONKEY_IRQ_1SI:
	case PF1550_ONKEY_IRQ_2SI:
	case PF1550_ONKEY_IRQ_3SI:
	case PF1550_ONKEY_IRQ_4SI:
	case PF1550_ONKEY_IRQ_8SI:
		state = 1;
		break;
	default:
		dev_err(onkey->dev, "onkey interrupt: irq %d occurred\n",
			irq_type);
		return IRQ_HANDLED;
	}

	input_event(onkey->input, EV_KEY, onkey->keycode, state);
	input_sync(onkey->input);

	return IRQ_HANDLED;
}

static int pf1550_onkey_probe(struct platform_device *pdev)
{
	struct onkey_drv_data *onkey;
	struct input_dev *input = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct pf1550_dev *pf1550 = dev_get_drvdata(pdev->dev.parent);
	int i, error;

	if (!np)
		return -ENODEV;

	onkey = devm_kzalloc(&pdev->dev, sizeof(*onkey), GFP_KERNEL);
	if (!onkey)
		return -ENOMEM;

	if (of_property_read_u32(np, "linux,keycode", &onkey->keycode)) {
		onkey->keycode = KEY_POWER;
		dev_warn(&pdev->dev, "KEY_POWER without setting in dts\n");
	}

	onkey->wakeup = of_property_read_bool(np, "wakeup");

	input = devm_input_allocate_device(&pdev->dev);
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate the input device\n");
		return -ENOMEM;
	}

	input->name = pdev->name;
	input->phys = "pf1550-onkey/input0";
	input->id.bustype = BUS_HOST;

	input_set_capability(input, EV_KEY, onkey->keycode);

	for (i = 0; i < ARRAY_SIZE(pf1550_onkey_irqs); i++) {
		struct pf1550_irq_info *onkey_irq =
						&pf1550_onkey_irqs[i];
		unsigned int virq = 0;

		virq = regmap_irq_get_virq(pf1550->irq_data_onkey,
					onkey_irq->irq);
		if (!virq)
			return -EINVAL;

		onkey_irq->virq = virq;

		error = devm_request_threaded_irq(&pdev->dev, virq, NULL,
					pf1550_onkey_irq_handler,
					IRQF_NO_SUSPEND,
					onkey_irq->name, onkey);
		if (error) {
			dev_err(&pdev->dev,
				"failed: irq request (IRQ: %d, error :%d)\n",
				onkey_irq->irq, error);
			return error;
		}
	}

	error = input_register_device(input);
	if (error < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		input_free_device(input);
		return error;
	}

	onkey->input = input;
	platform_set_drvdata(pdev, onkey);

	device_init_wakeup(&pdev->dev, onkey->wakeup);

	return 0;
}

static int pf1550_onkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct onkey_drv_data *onkey = platform_get_drvdata(pdev);

	if (!device_may_wakeup(&pdev->dev))
		regmap_write(onkey->pf1550->regmap,
			     PF1550_PMIC_REG_ONKEY_INT_MASK0,
			     ONKEY_IRQ_PUSHI | ONKEY_IRQ_1SI | ONKEY_IRQ_2SI |
			     ONKEY_IRQ_3SI | ONKEY_IRQ_4SI | ONKEY_IRQ_8SI);

	return 0;
}

static int pf1550_onkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct onkey_drv_data *onkey = platform_get_drvdata(pdev);

	if (!device_may_wakeup(&pdev->dev))
		regmap_write(onkey->pf1550->regmap,
			     PF1550_PMIC_REG_ONKEY_INT_MASK0,
			     ~(ONKEY_IRQ_PUSHI | ONKEY_IRQ_1SI | ONKEY_IRQ_2SI |
			     ONKEY_IRQ_3SI | ONKEY_IRQ_4SI | ONKEY_IRQ_8SI));

	return 0;
}

static const struct of_device_id pf1550_onkey_ids[] = {
	{ .compatible = "fsl,pf1550-onkey" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pf1550_onkey_ids);

static SIMPLE_DEV_PM_OPS(pf1550_onkey_pm_ops, pf1550_onkey_suspend,
				pf1550_onkey_resume);

static struct platform_driver pf1550_onkey_driver = {
	.driver = {
		.name = "pf1550-onkey",
		.pm     = &pf1550_onkey_pm_ops,
		.of_match_table = pf1550_onkey_ids,
	},
	.probe = pf1550_onkey_probe,
};
module_platform_driver(pf1550_onkey_driver);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION(" PF1550 onkey Driver");
MODULE_LICENSE("GPL");
