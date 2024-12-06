// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 */

#include <linux/err.h>
#include <linux/imx_rpmsg.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/pm_qos.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/slab.h>
#include <linux/module.h>


#define RPMSG_IIO_DEV_STANDBY		0x00
#define RPMSG_IIO_DEV_ACTIVED		0x01
#define PEDOMETER_TYPE			0x00
#define PEDOMETER_IDX			0x00

#define RPMSG_TIMEOUT_MS		1000


/*
 * Default use 1000ms poll delay, the supported range
 * is 500~3600000, which means from half second to one
 * hour.
 */
#define RPMSG_IIO_POLL_DELAY_DEFAULT	1000
#define RPMSG_IIO_POLL_DELAY_MIN	500
#define RPMSG_IIO_POLL_DELAY_MAX	3600000

enum rpmsg_iio_header_type {
	RPMSG_IIO_SETUP,
	RPMSG_IIO_REPLY,
	RPMSG_IIO_NOTIFY,
};

enum rpmsg_iio_header_cmd {
	RPMSG_IIO_DETECTOR_CMD,
	RPMSG_IIO_COUNTER_CMD,
	RPMSG_IIO_POLL_DELAY_CMD,
};

struct rpmsg_iio_msg {
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

struct rpmsg_iio_data {
	struct device *dev;
	struct iio_dev *indio_dev;
	bool counter_enable;
	bool detector_enable;
	u32 steps;
	u32 step_poll_delay;	// in ms

	struct rpmsg_device *rpdev;
	struct rpmsg_iio_msg *reply_msg;
	struct rpmsg_iio_msg *notify_msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct mutex lock;
};

/* This driver can only support one sensor instance. */
static struct rpmsg_iio_data *iio_rpmsg;

static int iio_send_message(struct rpmsg_iio_msg *msg,
			    struct rpmsg_iio_data *info)
{
	int err;

	if (!info->rpdev) {
		dev_err(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	cpu_latency_qos_add_request(&info->pm_qos_req, 0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg, sizeof(struct rpmsg_iio_msg));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	err = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(RPMSG_TIMEOUT_MS));
	if (!err) {
		dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
		goto err_out;
	}

	err = info->reply_msg->instruct.inout.retcode;
	if (err != 0) {
		dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n", err);
		err = -EINVAL;
		goto err_out;
	}

err_out:
	cpu_latency_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	return err;
}

static inline void rpmsg_iio_event_report(struct rpmsg_iio_msg *msg)
{
	struct iio_dev *indio_dev;
	s64 ts;

	indio_dev = iio_rpmsg->indio_dev;
	ts = iio_get_time_ns(indio_dev);
	if (msg->header.cmd == RPMSG_IIO_DETECTOR_CMD) {
		iio_push_event(indio_dev,
			       IIO_EVENT_CODE(IIO_ACTIVITY, 0, IIO_NO_MOD,
					      IIO_EV_DIR_RISING,
					      IIO_EV_TYPE_THRESH, 0, 0, 0),
			       ts);
	} else if (msg->header.cmd == RPMSG_IIO_COUNTER_CMD) {
		iio_rpmsg->steps = msg->instruct.val;
		iio_push_event(indio_dev,
			       IIO_EVENT_CODE(IIO_STEPS, 0, IIO_NO_MOD,
					      IIO_EV_DIR_NONE,
					      IIO_EV_TYPE_CHANGE, 0, 0, 0),
			       ts);
	}
}

static int rpmsg_iio_cb(struct rpmsg_device *rpdev, void *data, int len,
				void *priv, u32 src)
{
	struct rpmsg_iio_msg *msg = (struct rpmsg_iio_msg *)data;

	if (msg->header.type == RPMSG_IIO_REPLY) {
		iio_rpmsg->reply_msg = msg;
		complete(&iio_rpmsg->cmd_complete);
	} else if (msg->header.type == RPMSG_IIO_NOTIFY) {
		iio_rpmsg->notify_msg = msg;
		rpmsg_iio_event_report(msg);
	} else {
		dev_err(&iio_rpmsg->rpdev->dev, "wrong command type %d!\n",
			msg->header.type);
		return -EINVAL;
	}

	return 0;
}

static int rpmsg_iio_probe(struct rpmsg_device *rpdev)
{
	iio_rpmsg->rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x\n",
			rpdev->src, rpdev->dst);

	init_completion(&iio_rpmsg->cmd_complete);
	mutex_init(&iio_rpmsg->lock);

	return 0;
}

static struct rpmsg_device_id rpmsg_iio_id_table[] = {
	{ .name = "rpmsg-sensor-channel" },
	{ },
};

static struct rpmsg_driver rpmsg_iio_driver = {
	.drv.name	= "iio_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_iio_id_table,
	.probe		= rpmsg_iio_probe,
	.callback	= rpmsg_iio_cb,
};

static int rpmsg_iio_change_mode(int cmd, int enable)
{
	struct rpmsg_iio_msg msg;

	memset(&msg, 0, sizeof(struct rpmsg_iio_msg));
	msg.header.cate = IMX_RPMSG_SENSOR;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RPMSG_IIO_SETUP;
	msg.header.cmd = cmd;
	msg.sensor_type = PEDOMETER_TYPE;
	msg.sensor_index = PEDOMETER_IDX;
	msg.instruct.inout.enable = enable;

	return iio_send_message(&msg, iio_rpmsg);
}

static int rpmsg_iio_set_step_poll_delay(int cmd, int poll_delay)
{
	struct rpmsg_iio_msg msg;

	memset(&msg, 0, sizeof(struct rpmsg_iio_msg));
	msg.header.cate = IMX_RPMSG_SENSOR;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RPMSG_IIO_SETUP;
	msg.header.cmd = cmd;
	msg.sensor_type = PEDOMETER_TYPE;
	msg.sensor_index = PEDOMETER_IDX;
	msg.instruct.val = poll_delay;

	return iio_send_message(&msg, iio_rpmsg);
}

static int rpmsg_iio_read_event_value(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					enum iio_event_info info,
					int *val, int *val2)
{
	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = iio_rpmsg->steps;
		return IIO_VAL_INT;
	case IIO_EV_INFO_PERIOD:
		*val = iio_rpmsg->step_poll_delay;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int rpmsg_iio_write_event_value(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir,
					enum iio_event_info info,
					int val, int val2)
{
	int ret;
	switch (info) {
	case IIO_EV_INFO_PERIOD:
		if (val < RPMSG_IIO_POLL_DELAY_MIN ||
		    val > RPMSG_IIO_POLL_DELAY_MAX) {
			dev_err(&indio_dev->dev,
				"Incorrect value, the val must in 500~3600000\n");
			return -EINVAL;
		}
		ret = rpmsg_iio_set_step_poll_delay(RPMSG_IIO_POLL_DELAY_CMD, val);
		if (!ret)
			iio_rpmsg->step_poll_delay = val;
		return ret;
	default:
		return -EINVAL;
	}
}

static int rpmsg_iio_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct rpmsg_iio_data *pdata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->type) {
		case IIO_STEPS:
			*val = pdata->counter_enable;
			return IIO_VAL_INT;
		case IIO_ACTIVITY:
			*val = pdata->detector_enable;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}

}

static int rpmsg_iio_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct rpmsg_iio_data *pdata = iio_priv(indio_dev);
	unsigned long mode;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->type) {
		case IIO_STEPS:
			mode = val ? RPMSG_IIO_DEV_ACTIVED:
				     RPMSG_IIO_DEV_STANDBY;
			ret = rpmsg_iio_change_mode(RPMSG_IIO_COUNTER_CMD, mode);
			if (!ret)
				pdata->counter_enable = val;
			return ret;
		case IIO_ACTIVITY:
			mode = val ? RPMSG_IIO_DEV_ACTIVED:
				     RPMSG_IIO_DEV_STANDBY;
			ret = rpmsg_iio_change_mode(RPMSG_IIO_DETECTOR_CMD, mode);
			if (!ret)
				pdata->detector_enable = val;
			return ret;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_event_spec rpmsg_iio_step_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_NONE,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD),
};

static const struct iio_chan_spec rpmsg_iio_channels[] = {
	{
		.type = IIO_STEPS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.event_spec = &rpmsg_iio_step_event,
		.num_event_specs = 1,
	},

	{
		.type = IIO_ACTIVITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
	},
};

static const struct iio_info rpmsg_iio_info = {
	.read_raw = rpmsg_iio_read_raw,
	.write_raw = rpmsg_iio_write_raw,
	.read_event_value = rpmsg_iio_read_event_value,
	.write_event_value = rpmsg_iio_write_event_value,
};

static int rpmsg_iio_pedometer_device_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct rpmsg_iio_data));
	if (!indio_dev)
		return -ENOMEM;

	iio_rpmsg = iio_priv(indio_dev);
	iio_rpmsg->indio_dev = indio_dev;
	platform_set_drvdata(pdev, iio_rpmsg);

	indio_dev->channels = rpmsg_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(rpmsg_iio_channels);
	indio_dev->name = "rpmsg-iio-pedometer";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &rpmsg_iio_info;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "unable to register iio pedometer device\n");
		goto err_register_iio;
	}

	ret = register_rpmsg_driver(&rpmsg_iio_driver);
	if (ret) {
		dev_err(dev, "unable to register rpmsg iio driver\n");
		goto err_register_rpmsg;
	}

	/*
	 * Default use 1000ms poll delay, the supported range
	 * is 500~3600000, which means from half second to one
	 * hour.
	 */
	iio_rpmsg->step_poll_delay = 1000;

	return ret;

err_register_rpmsg:
	iio_device_unregister(indio_dev);
err_register_iio:
	platform_set_drvdata(pdev, NULL);

	return ret;

}

static const struct of_device_id rpmsg_iio_pedometer_of_match[] = {
	{ .compatible = "nxp,rpmsg-iio-pedometer", },
	{ },
};

MODULE_DEVICE_TABLE(of, rpmsg_iio_pedometer_of_match);

static struct platform_driver rpmsg_iio_pedometer_device_driver = {
	.probe		= rpmsg_iio_pedometer_device_probe,
	.driver		= {
		.name	= "rpmsg-iio-pedometer",
		.of_match_table = of_match_ptr(rpmsg_iio_pedometer_of_match)
	}
};

module_platform_driver(rpmsg_iio_pedometer_device_driver);

MODULE_AUTHOR("Haibo Chen <haibo.chen@nxp.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NXP rpmsg iio pedometer driver");
