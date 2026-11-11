// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ON pin driver for Dialog DA9055 PMICs
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * Author: David Dajun Chen <dchen@diasemi.com>
 */

#include <linux/devm-helpers.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/mfd/da9055/core.h>
#include <linux/mfd/da9055/reg.h>

struct da9055_onkey {
	struct da9055 *da9055;
	struct input_dev *input;
	struct delayed_work work;
};

static void da9055_onkey_query(struct da9055_onkey *onkey)
{
	int key_stat;

	key_stat = da9055_reg_read(onkey->da9055, DA9055_REG_STATUS_A);
	if (key_stat < 0) {
		dev_err(onkey->da9055->dev,
			"Failed to read onkey event %d\n", key_stat);
	} else {
		key_stat &= DA9055_NOKEY_STS;
		/*
		 * Onkey status bit is cleared when onkey button is released.
		 */
		if (!key_stat) {
			input_report_key(onkey->input, KEY_POWER, 0);
			input_sync(onkey->input);
		}
	}

	/*
	 * Interrupt is generated only when the ONKEY pin is asserted.
	 * Hence the deassertion of the pin is simulated through work queue.
	 */
	if (key_stat)
		schedule_delayed_work(&onkey->work, msecs_to_jiffies(10));
}

static void da9055_onkey_work(struct work_struct *work)
{
	struct da9055_onkey *onkey = container_of(work, struct da9055_onkey,
						  work.work);

	da9055_onkey_query(onkey);
}

static irqreturn_t da9055_onkey_irq(int irq, void *data)
{
	struct da9055_onkey *onkey = data;

	input_report_key(onkey->input, KEY_POWER, 1);
	input_sync(onkey->input);

	da9055_onkey_query(onkey);

	return IRQ_HANDLED;
}

static int da9055_onkey_probe(struct platform_device *pdev)
{
	struct da9055 *da9055 = dev_get_drvdata(pdev->dev.parent);
	struct da9055_onkey *onkey;
	struct input_dev *input_dev;
	int irq, err;

	irq = platform_get_irq_byname(pdev, "ONKEY");
	if (irq < 0)
		return -EINVAL;

	onkey = devm_kzalloc_obj(&pdev->dev, *onkey);
	if (!onkey)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&pdev->dev);
	if (!input_dev)
		return -ENOMEM;

	onkey->input = input_dev;
	onkey->da9055 = da9055;
	input_dev->name = "da9055-onkey";
	input_dev->phys = "da9055-onkey/input0";

	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	err = devm_delayed_work_autocancel(&pdev->dev, &onkey->work,
					   da9055_onkey_work);
	if (err)
		return err;

	err = devm_request_threaded_irq(&pdev->dev, irq, NULL, da9055_onkey_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"ONKEY", onkey);
	if (err)
		return err;

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&pdev->dev, "Unable to register input device, %d\n",
			err);
		return err;
	}

	return 0;
}

static struct platform_driver da9055_onkey_driver = {
	.probe	= da9055_onkey_probe,
	.driver	= {
		.name	= "da9055-onkey",
	},
};

module_platform_driver(da9055_onkey_driver);

MODULE_AUTHOR("David Dajun Chen <dchen@diasemi.com>");
MODULE_DESCRIPTION("Onkey driver for DA9055");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:da9055-onkey");
