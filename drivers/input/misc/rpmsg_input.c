/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/imx_rpmsg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>

#define RPMSG_INPUT_DEV_STANDBY	 	0x00
#define RPMSG_INPUT_DEV_ACTIVED 	0x01
#define DETECTOR_DEVICE			"step_detector"
#define COUNTER_DEVICE			"step_counter"

#define PEDOMETER_TYPE			0x0
#define PEDOMETER_IDX			0x0

#define RPMSG_TIMEOUT 			1000

enum rpmsg_input_header_type {
	RPMSG_INPUT_SETUP,
	RPMSG_INPUT_REPLY,
	RPMSG_INPUT_NOTIFY,
};

enum rpmsg_input_header_cmd {
	RPMSG_INPUT_DETECTOR_CMD,
	RPMSG_INPUT_COUNTER_CMD,
	RPMSG_INPUT_POLL_DELAY_CMD,
};

struct rpmsg_input_msg {
	struct imx_rpmsg_head header;
	u8 sensor_type;
	u8 sensor_index;
	union {
		union {
			u8 enable;
			u8 retcode;
		} inout;

		u32 val;
	} instruct;
} __packed __aligned(8);

struct rpmsg_input_data {
	struct device *dev;
	struct miscdevice *detector_miscdev;
	struct miscdevice *counter_miscdev;
	struct input_dev *detector_input;
	struct input_dev *counter_input;
	atomic_t detector_active;
	atomic_t counter_active;
	atomic_t counter_delay;

	struct rpmsg_device *rpdev;
	struct rpmsg_input_msg *reply_msg;
	struct rpmsg_input_msg *notify_msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct mutex lock;
};

static struct rpmsg_input_data *input_rpmsg;

static int input_send_message(struct rpmsg_input_msg *msg,
			      struct rpmsg_input_data *info)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	pm_qos_add_request(&info->pm_qos_req,
			   PM_QOS_CPU_DMA_LATENCY, 0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			 sizeof(struct rpmsg_input_msg));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	err = wait_for_completion_timeout(&info->cmd_complete,
				msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
		err = -ETIMEDOUT;
		goto err_out;
	}

	err = info->reply_msg->instruct.inout.retcode;
	if (err != 0) {
		dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n", err);
		err = -EINVAL;
		goto err_out;
	}

	err = 0;

err_out:
	pm_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	return err;
}

static inline void rpmsg_input_evt_report(struct rpmsg_input_msg *msg)
{
	int val = 0x1;

	if (msg->header.cmd == RPMSG_INPUT_DETECTOR_CMD) {
		input_report_rel(input_rpmsg->detector_input, REL_MISC, val);
		input_sync(input_rpmsg->detector_input);
	} else if (msg->header.cmd ==  RPMSG_INPUT_COUNTER_CMD) {
		val = msg->instruct.val;
		input_report_abs(input_rpmsg->counter_input, ABS_MISC, val);
		input_sync(input_rpmsg->counter_input);
	}
}

static int rpmsg_input_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct rpmsg_input_msg *msg = (struct rpmsg_input_msg *)data;

	if (msg->header.type == RPMSG_INPUT_REPLY) {
		input_rpmsg->reply_msg = msg;
		complete(&input_rpmsg->cmd_complete);
		return 0;
	} else if (msg->header.type == RPMSG_INPUT_NOTIFY) {
		input_rpmsg->notify_msg = msg;
		rpmsg_input_evt_report(msg);
	} else
		dev_err(&input_rpmsg->rpdev->dev, "wrong command type!\n");

	return 0;
}

static int rpmsg_input_probe(struct rpmsg_device *rpdev)
{
	input_rpmsg->rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&input_rpmsg->cmd_complete);
	mutex_init(&input_rpmsg->lock);

	return 0;
}

static struct rpmsg_device_id rpmsg_input_id_table[] = {
	{ .name	= "rpmsg-sensor-channel" },
	{ },
};

static struct rpmsg_driver rpmsg_input_driver = {
	.drv.name	= "input_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_input_id_table,
	.probe		= rpmsg_input_probe,
	.callback	= rpmsg_input_cb,
};

static int rpmsg_input_open(struct inode *inode, struct file *file)
{
	file->private_data = input_rpmsg;
	return nonseekable_open(inode, file);
}

static int rpmsg_input_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long
rpmsg_input_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* TBD: for future sensor interfaces support */
	return 0;
}

static const struct file_operations rpmsg_input_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_input_open,
	.release = rpmsg_input_release,
	.unlocked_ioctl = rpmsg_input_ioctl,
};

static struct miscdevice step_detector_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "step_detector",
	.fops = &rpmsg_input_fops,
};

static struct miscdevice step_counter_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "step_counter",
	.fops = &rpmsg_input_fops,
};

static int rpmsg_input_change_mode(int cmd, int enable)
{

	struct rpmsg_input_msg msg;

	memset(&msg, 0, sizeof(struct rpmsg_input_msg));
	msg.header.cate = IMX_RPMSG_SENSOR;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RPMSG_INPUT_SETUP;
	msg.header.cmd = cmd;
	msg.sensor_type = PEDOMETER_TYPE;
	msg.sensor_index = PEDOMETER_IDX;
	msg.instruct.inout.enable = enable;

	return input_send_message(&msg, input_rpmsg);
}

static int rpmsg_input_set_poll_delay(int cmd, int delay)
{
	struct rpmsg_input_msg msg;

	memset(&msg, 0, sizeof(struct rpmsg_input_msg));
	msg.header.cate = IMX_RPMSG_SENSOR;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RPMSG_INPUT_SETUP;
	msg.header.cmd = cmd;
	msg.sensor_type = PEDOMETER_TYPE;
	msg.sensor_index = PEDOMETER_IDX;
	msg.instruct.val = delay;

	return input_send_message(&msg, input_rpmsg);
}

static ssize_t rpmsg_input_enable_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct rpmsg_input_data *pdata = input_rpmsg;
	int enable = 0;

	if (pdata->detector_miscdev == misc_dev)
		enable = atomic_read(&pdata->detector_active);
	if (pdata->counter_miscdev == misc_dev)
		enable = atomic_read(&pdata->counter_active);

	return sprintf(buf, "%d\n", enable);
}

static ssize_t rpmsg_input_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct rpmsg_input_data *pdata = input_rpmsg;
	unsigned long enable;
	int cmd;
	int ret;

	if (kstrtoul(buf, 10, &enable) < 0)
		return -EINVAL;

	if (misc_dev == pdata->detector_miscdev)
		cmd = RPMSG_INPUT_DETECTOR_CMD;
	if (misc_dev == pdata->counter_miscdev)
		cmd = RPMSG_INPUT_COUNTER_CMD;

	enable = enable > 0 ? RPMSG_INPUT_DEV_ACTIVED :
			      RPMSG_INPUT_DEV_STANDBY;
	ret = rpmsg_input_change_mode(cmd, enable);
	if (!ret) {
		if (cmd == RPMSG_INPUT_DETECTOR_CMD)
			atomic_set(&pdata->detector_active, enable);
		else if (cmd == RPMSG_INPUT_COUNTER_CMD)
			atomic_set(&pdata->counter_active, enable);
	}

	return count;
}

static ssize_t rpmsg_input_poll_delay_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct rpmsg_input_data *pdata = input_rpmsg;
	int poll_delay = 0;

	if (pdata->detector_miscdev == misc_dev)
		return -ENOTSUPP;

	if (pdata->counter_miscdev == misc_dev)
		poll_delay = atomic_read(&pdata->counter_delay);

	return sprintf(buf, "%d\n", poll_delay);
}


static ssize_t rpmsg_input_poll_delay_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct miscdevice *misc_dev = dev_get_drvdata(dev);
	struct rpmsg_input_data *pdata = input_rpmsg;
	unsigned long delay;
	int cmd;
	int ret;

	if (kstrtoul(buf, 10, &delay) < 0)
		return -EINVAL;

	if (pdata->detector_miscdev == misc_dev)
		return -ENOTSUPP;

	if (pdata->counter_miscdev == misc_dev)
		cmd = RPMSG_INPUT_POLL_DELAY_CMD;

	ret = rpmsg_input_set_poll_delay(cmd, delay);
	if (!ret) {
		if (cmd == RPMSG_INPUT_POLL_DELAY_CMD)
			atomic_set(&pdata->counter_delay, delay);
	}

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   rpmsg_input_enable_show,
		   rpmsg_input_enable_store);
static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
		   rpmsg_input_poll_delay_show,
		   rpmsg_input_poll_delay_store);

static struct attribute *rpmsg_input_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group rpmsg_input_attr_group = {
	.attrs = rpmsg_input_attributes,
};

static int rpmsg_input_register_sysfs_device(struct rpmsg_input_data *pdata)
{
	struct miscdevice *misc_dev = NULL;
	int err = 0;

	/* register sysfs for detector */
	misc_dev = pdata->detector_miscdev;
	err = sysfs_create_group(&misc_dev->this_device->kobj,
				 &rpmsg_input_attr_group);
	if (err)
		goto out;

	/* register sysfs for counter */
	misc_dev = pdata->counter_miscdev;
	err = sysfs_create_group(&misc_dev->this_device->kobj,
				 &rpmsg_input_attr_group);
	if (err)
		goto err_register_sysfs;

	return 0;

err_register_sysfs:
	misc_dev = pdata->detector_miscdev;
	sysfs_remove_group(&misc_dev->this_device->kobj, &rpmsg_input_attr_group);
out:
	return err;
}

static int rpmsg_input_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmsg_input_data *pdata;
	struct input_dev *detector_input;
	struct input_dev *counter_input;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	detector_input = devm_input_allocate_device(dev);
	if (!detector_input) {
		dev_err(dev, "failed to allocate detector device\n");
		return -ENOMEM;
	}

	counter_input = devm_input_allocate_device(dev);
	if (!counter_input) {
		dev_err(dev, "failed to allocate counter device\n");
		return -ENOMEM;
	}

	detector_input->name = DETECTOR_DEVICE;
	detector_input->phys = "rpmsg-input/input0";
	detector_input->dev.parent = &pdev->dev;
	detector_input->id.bustype = BUS_HOST;
	detector_input->evbit[0] = BIT_MASK(EV_REL);
	detector_input->relbit[0] = BIT_MASK(REL_MISC);
	pdata->detector_input = detector_input;

	counter_input->name = COUNTER_DEVICE;
	counter_input->phys = "rpmsg-input/input1";
	counter_input->dev.parent = &pdev->dev;
	counter_input->id.bustype = BUS_HOST;
	counter_input->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(counter_input, ABS_MISC, 0, 0xFFFFFFFF, 0, 0);
	pdata->counter_input = counter_input;

	input_rpmsg = pdata;
	platform_set_drvdata(pdev, pdata);

	ret = misc_register(&step_detector_device);
	if (ret != 0) {
		dev_err(dev, "register acc miscdevice error");
		goto err_register_detector_misc;
	}
	pdata->detector_miscdev = &step_detector_device;

	ret = misc_register(&step_counter_device);
	if (ret != 0) {
		dev_err(dev, "register acc miscdevice error");
		goto err_register_counter_misc;
	}
	pdata->counter_miscdev = &step_counter_device;

	ret = rpmsg_input_register_sysfs_device(pdata);
	if (ret) {
		dev_err(dev, "Unable to register input sysfs, error: %d\n",
			ret);
		goto err_register_sysfs;
	}

	ret = input_register_device(detector_input);
	if (ret) {
		dev_err(dev, "Unable to register detector device, error: %d\n",
			ret);
		goto err_register_detector_input;
	}

	ret = input_register_device(counter_input);
	if (ret) {
		dev_err(dev, "Unable to register counter device, error: %d\n",
			ret);
		goto err_register_counter_input;
	}

	ret =  register_rpmsg_driver(&rpmsg_input_driver);
	if (ret) {
		dev_err(dev, "Unable to register rpmsg driver, error: %d\n",
			ret);
		goto err_register_rmpsg_driver;
	}

	return ret;

err_register_rmpsg_driver:
	input_unregister_device(counter_input);
err_register_counter_input:
	input_unregister_device(detector_input);
err_register_detector_input:
err_register_sysfs:
	misc_deregister(&step_counter_device);
	pdata->counter_miscdev = NULL;
err_register_counter_misc:
	misc_deregister(&step_detector_device);
	pdata->detector_miscdev = NULL;
err_register_detector_misc:
	pdata->detector_input = NULL;
	pdata->counter_input = NULL;
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static const struct of_device_id rpmsg_input_of_match[] = {
	{ .compatible = "fsl,rpmsg-input", },
	{ },
};

MODULE_DEVICE_TABLE(of, rpmsg_input_of_match);

static struct platform_driver rpmsg_input_device_driver = {
	.probe		= rpmsg_input_device_probe,
	.driver		=  {
		.name	= "rpmsg-input",
		.of_match_table = of_match_ptr(rpmsg_input_of_match)
	}
};

module_platform_driver(rpmsg_input_device_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX7ULP rpmsg sensor input driver");
MODULE_LICENSE("GPL v2");
