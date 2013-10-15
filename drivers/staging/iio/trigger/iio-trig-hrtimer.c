/**
 * The industrial I/O periodic hrtimer trigger driver
 *
 * Copyright (C) Intuitive Aerial AB
 * Written by Marten Svanfeldt, marten@intuitiveaerial.com
 * Copyright (C) 2012, Analog Device Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>

struct iio_hrtimer_trig_info {
	struct iio_trigger *trigger;
	unsigned int frequency;
	struct hrtimer timer;
	ktime_t period;
};

static enum hrtimer_restart iio_trig_hrtimer_trig(struct hrtimer *timer)
{
	struct iio_hrtimer_trig_info *trig_info;
	trig_info = container_of(timer, struct iio_hrtimer_trig_info, timer);

	hrtimer_forward_now(timer, trig_info->period);
	iio_trigger_poll(trig_info->trigger, 0);

	return HRTIMER_RESTART;
}

static int iio_trig_hrtimer_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_hrtimer_trig_info *trig_info = iio_trigger_get_drvdata(trig);

	if (trig_info->frequency == 0)
		return -EINVAL;

	if (state) {
		hrtimer_start(&trig_info->timer, trig_info->period,
			HRTIMER_MODE_REL);
	} else {
		hrtimer_cancel(&trig_info->timer);
	}

	return 0;
}

static ssize_t iio_trig_hrtimer_read_freq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_hrtimer_trig_info *trig_info = iio_trigger_get_drvdata(trig);

	return sprintf(buf, "%u\n", trig_info->frequency);
}

static ssize_t iio_trig_hrtimer_write_freq(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_hrtimer_trig_info *trig_info = iio_trigger_get_drvdata(trig);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	if (val > NSEC_PER_SEC)
		return -EINVAL;

	trig_info->frequency = val;
	if (trig_info->frequency != 0)
		trig_info->period = ktime_set(0, NSEC_PER_SEC / trig_info->frequency);

	return len;
}

static DEVICE_ATTR(frequency, S_IRUGO | S_IWUSR,
		iio_trig_hrtimer_read_freq,
		iio_trig_hrtimer_write_freq);

static struct attribute *iio_trig_hrtimer_attrs[] = {
	&dev_attr_frequency.attr,
	NULL,
};

static const struct attribute_group iio_trig_hrtimer_attr_group = {
	.attrs = iio_trig_hrtimer_attrs,
};

static const struct attribute_group *iio_trig_hrtimer_attr_groups[] = {
	&iio_trig_hrtimer_attr_group,
	NULL
};

static const struct iio_trigger_ops iio_hrtimer_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = iio_trig_hrtimer_set_state,
};

static int iio_trig_hrtimer_probe(struct platform_device *pdev)
{
	struct iio_hrtimer_trig_info *trig_info;
	struct iio_trigger *trig;
	char *name;
	int ret;

	trig_info = devm_kzalloc(&pdev->dev, sizeof(*trig_info), GFP_KERNEL);
	if (!trig_info)
		return -ENOMEM;

	name = pdev->dev.platform_data;

	if (name != NULL)
		trig = iio_trigger_alloc("hrtimer%s", name);
	else
		trig = iio_trigger_alloc("hrtimer%d", pdev->id);

	if (!trig)
		return -ENOMEM;

	trig_info->trigger = trig;
	iio_trigger_set_drvdata(trig, trig_info);
	trig->ops = &iio_hrtimer_trigger_ops;
	trig->dev.groups = iio_trig_hrtimer_attr_groups;
	trig->dev.parent = &pdev->dev;

	hrtimer_init(&trig_info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	trig_info->timer.function = iio_trig_hrtimer_trig;

	ret = iio_trigger_register(trig);
	if (ret)
		goto err_free_trigger;

	platform_set_drvdata(pdev, trig_info);

	return 0;
err_free_trigger:
	iio_trigger_free(trig);

	return ret;
}

static int iio_trig_hrtimer_remove(struct platform_device *pdev)
{
	struct iio_hrtimer_trig_info *trig_info = platform_get_drvdata(pdev);
	struct iio_trigger *trig = trig_info->trigger;

	iio_trigger_unregister(trig);
	hrtimer_cancel(&trig_info->timer);
	iio_trigger_free(trig);

	return 0;
}

static const struct of_device_id trigger_of_match[] = {
	{ .compatible = "iio-trigger-hrtimer", },
	{}
};
MODULE_DEVICE_TABLE(of, trigger_of_match);

static struct platform_driver iio_trig_hrtimer_driver = {
	.driver = {
		.name = "iio-trigger-hrtimer",
		.owner = THIS_MODULE,
		.of_match_table = trigger_of_match
	},
	.probe = iio_trig_hrtimer_probe,
	.remove = iio_trig_hrtimer_remove,
};
module_platform_driver(iio_trig_hrtimer_driver);

MODULE_AUTHOR("Marten Svanfeldt <marten@intuitiveaerial.com>");
MODULE_DESCRIPTION("Periodic hrtimer trigger for the IIO subsystem");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:iio-trigger-hrtimer");
