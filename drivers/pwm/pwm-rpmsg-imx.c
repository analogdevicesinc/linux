// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP.
 */

/*
 * The pwm-rpmsg transfer protocol:
 *   +---------------+-------------------------------+
 *   |  Byte Offset  |            Content            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Reserved0           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |          Request ID           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11      |         Return Value          |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       12      |            PWM ID             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       13      |          Channel ID           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    14 ~ 21    |            Period             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    22 ~ 29    |          Duty Cycle           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       30      |           Polarity            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       31      |            Enable             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 * The definition of Return Value:
 * 0x00 = Success
 * 0x01 = Failed
 * 0x02 = Invalid parameter
 * 0x03 = Chip Busy
 * 0x04 = Operate in invalid state
 * 0x05 = Memory allocation failed
 * 0x06 = Timeout when waiting for an event
 * 0x07 = Cannot add to list as node already in another list
 * 0x08 = Cannot remove from list as node not in list
 * 0x09 = Transfer timeout
 * 0x0A = Transfer failed due to peer core not ready
 * 0x0B = Transfer failed due to communication failure
 * 0x0C = Cannot find service for a request/notification
 * 0x0D = Service version cannot support the request/notification
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>

#define PWM_RPMSG_TIMEOUT_MS			500

#define PWM_RPMSG_CATEGORY			0x0a
#define PWM_RPMSG_VERSION			0x0001
#define PWM_RPMSG_TYPE_REQUEST			0x00
#define PWM_RPMSG_TYPE_RESPONSE			0x01
#define PWM_RPMSG_COMMAND_GET			0x00
#define PWM_RPMSG_COMMAND_SET			0x01

struct pwm_rpmsg_msg {
	struct imx_rpmsg_head header;

	/* Payload Start*/
	u8 request_id;
	u8 ret_val;
	u8 chip_id;
	u8 channel_id;
	u64 period;
	u64 duty_cycle;
	u8 polarity;
	u8 enabled;
} __packed __aligned(1);

struct pwm_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct pwm_rpmsg_msg *msg;
	struct completion cmd_complete;
	struct mutex lock;

	u8 chip_id;
	u8 channel_id;
	u8 request_id;
};

static struct pwm_rpmsg_info pwm_rpmsg;

struct imx_rpmsg_pwm_data {
	u8 chip_id;
};

static inline struct imx_rpmsg_pwm_data *
to_imx_rpmsg_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int pwm_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	struct pwm_rpmsg_msg *msg = (struct pwm_rpmsg_msg *)data;

	if (msg->header.type != PWM_RPMSG_TYPE_RESPONSE)
		return -EINVAL;

	if (msg->request_id != pwm_rpmsg.request_id) {
		dev_err(&rpdev->dev,
		"expected request_id:%d, received request_id:%d, drop this recv\n",
		pwm_rpmsg.request_id, msg->request_id);

		/*
		 * This response does not match the request id.
		 * Drop it and wait for the right response.
		 */
		return 0;
	}

	if (msg->chip_id != pwm_rpmsg.chip_id ||
	    msg->channel_id != pwm_rpmsg.channel_id) {
		dev_err(&rpdev->dev,
		"expected chip_id:%d, channel_id:%2x, received chip_id:%d, channel_id:%2x\n",
		pwm_rpmsg.chip_id, pwm_rpmsg.channel_id,
		msg->chip_id, msg->channel_id);

		/*
		 * The chip_id or channel_id of this response does not match the
		 * request, but the request_id match. So return error.
		 */
		return -EINVAL;
	}

	/* Receive Success */
	pwm_rpmsg.msg = msg;

	complete(&pwm_rpmsg.cmd_complete);

	return 0;
}

static int rpmsg_xfer(struct pwm_rpmsg_msg *rmsg, struct pwm_rpmsg_info *info)
{
	int ret = 0;

	ret = rpmsg_send(info->rpdev->ept, (void *)rmsg,
						sizeof(struct pwm_rpmsg_msg));
	if (ret < 0) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(PWM_RPMSG_TIMEOUT_MS));
	if (!ret) {
		dev_err(&info->rpdev->dev,
		"%s failed: timeout, target chip_id=%-2d, channel_id=0x%02X, %s state\n",
				__func__, rmsg->chip_id, rmsg->channel_id,
				(rmsg->header.cmd == 1) ? "SET" : "GET");
		return -ETIMEDOUT;
	}

	if (info->msg->ret_val) {
		dev_dbg(&info->rpdev->dev,
			"%s failed: %d\n", __func__, info->msg->ret_val);
		return -(info->msg->ret_val);
	}

	return 0;
}

static int pwm_rpsmg_get(struct pwm_rpmsg_info *info, struct pwm_state *state)
{
	int ret;
	struct pwm_rpmsg_msg rmsg;

	if (!info || !info->rpdev)
		return -EINVAL;

	memset(&rmsg, 0, sizeof(struct pwm_rpmsg_msg));
	rmsg.header.cate = PWM_RPMSG_CATEGORY;
	rmsg.header.major = PWM_RPMSG_VERSION;
	rmsg.header.minor = PWM_RPMSG_VERSION >> 8;
	rmsg.header.type = PWM_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd = PWM_RPMSG_COMMAND_GET;
	rmsg.request_id = info->request_id;
	rmsg.chip_id = info->chip_id;
	rmsg.channel_id = info->channel_id;
	rmsg.period = state->period;
	rmsg.duty_cycle = state->duty_cycle;
	rmsg.polarity = state->polarity;
	rmsg.enabled = state->enabled;

	reinit_completion(&info->cmd_complete);

	ret = rpmsg_xfer(&rmsg, info);
	if (ret)
		return ret;

	state->period = info->msg->period;
	state->duty_cycle = info->msg->duty_cycle;
	state->polarity = info->msg->polarity;
	state->enabled = info->msg->enabled;

	return ret;
}

static int pwm_rpsmg_set(struct pwm_rpmsg_info *info, const struct pwm_state *state)
{
	int ret;
	struct pwm_rpmsg_msg rmsg;

	if (!info || !info->rpdev)
		return -EINVAL;

	memset(&rmsg, 0, sizeof(struct pwm_rpmsg_msg));
	rmsg.header.cate = PWM_RPMSG_CATEGORY;
	rmsg.header.major = PWM_RPMSG_VERSION;
	rmsg.header.minor = PWM_RPMSG_VERSION >> 8;
	rmsg.header.type = PWM_RPMSG_TYPE_REQUEST;
	rmsg.header.cmd = PWM_RPMSG_COMMAND_SET;
	rmsg.request_id = info->request_id;
	rmsg.chip_id = info->chip_id;
	rmsg.channel_id = info->channel_id;
	rmsg.period = state->period;
	rmsg.duty_cycle = state->duty_cycle;
	rmsg.polarity = state->polarity;
	rmsg.enabled = state->enabled;

	reinit_completion(&info->cmd_complete);

	ret = rpmsg_xfer(&rmsg, info);

	return ret;
}

static int pwm_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret = 0;

	if (!rpdev) {
		dev_info(&rpdev->dev, "%s failed, rpdev=NULL\n", __func__);
		return -EINVAL;
	}

	pwm_rpmsg.rpdev = rpdev;

	mutex_init(&pwm_rpmsg.lock);
	init_completion(&pwm_rpmsg.cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
						rpdev->src, rpdev->dst);

	return ret;
}

static void pwm_rpmsg_remove(struct rpmsg_device *rpdev)
{
	pwm_rpmsg.rpdev = NULL;
	dev_info(&rpdev->dev, "pwm rpmsg driver is removed\n");
}

static struct rpmsg_device_id pwm_rpmsg_id_table[] = {
	{ .name	= "rpmsg-pwm-channel" },
	{ },
};

static struct rpmsg_driver pwm_rpmsg_driver = {
	.drv.name	= "pwm-rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= pwm_rpmsg_id_table,
	.probe		= pwm_rpmsg_probe,
	.remove		= pwm_rpmsg_remove,
	.callback	= pwm_rpmsg_cb,
};

static int pwm_rpchip_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct imx_rpmsg_pwm_data *rdata = to_imx_rpmsg_pwm_chip(chip);
	int ret;

	mutex_lock(&pwm_rpmsg.lock);

	if (pwm_rpmsg.request_id >= 0xFF)
		pwm_rpmsg.request_id = 0;
	pwm_rpmsg.request_id++;

	pwm_rpmsg.chip_id = rdata->chip_id;
	pwm_rpmsg.channel_id = pwm->hwpwm;
	ret = pwm_rpsmg_get(&pwm_rpmsg, state);

	mutex_unlock(&pwm_rpmsg.lock);

	return 0;
}

static int pwm_rpchip_apply(struct pwm_chip *chip,
			     struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct imx_rpmsg_pwm_data *rdata = to_imx_rpmsg_pwm_chip(chip);
	int ret;

	mutex_lock(&pwm_rpmsg.lock);

	if (pwm_rpmsg.request_id >= 0xFF)
		pwm_rpmsg.request_id = 0;
	pwm_rpmsg.request_id++;

	pwm_rpmsg.chip_id = rdata->chip_id;
	pwm_rpmsg.channel_id = pwm->hwpwm;
	ret = pwm_rpsmg_set(&pwm_rpmsg, state);

	mutex_unlock(&pwm_rpmsg.lock);

	return ret;
}

static const struct pwm_ops pwm_rpchip_ops = {
	.get_state = pwm_rpchip_get_state,
	.apply = pwm_rpchip_apply,
};

static int pwm_rpchip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_pwm_data *rdata;
	struct pwm_chip *chip;
	unsigned int npwm;
	int ret;

	if (!pwm_rpmsg.rpdev)
		return -EPROBE_DEFER;

	ret = of_property_read_u32(np, "fsl,pwm-channel-number", &npwm);
	if (ret < 0) {
		dev_err(dev, "failed to read pwm channel number from dts: %d\n",
			     ret);
		return -EINVAL;
	}

	chip = devm_pwmchip_alloc(&pdev->dev, npwm, sizeof(*rdata));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	rdata = to_imx_rpmsg_pwm_chip(chip);

	/* setup pwm chip description */
	chip->ops = &pwm_rpchip_ops;
	chip->of_xlate = of_pwm_xlate_with_flags;

	rdata->chip_id = of_alias_get_id(np, "pwm");
	if (rdata->chip_id < 0) {
		dev_err(dev, "failed to get pwm alias number: %d\n",
			     rdata->chip_id);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, rdata);

	ret = devm_pwmchip_add(&pdev->dev, chip);
	if (ret) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	dev_info(dev, "add PWM chip %d successfully\n", rdata->chip_id);

	return ret;
}

static const struct of_device_id imx_rpmsg_pwm_dt_ids[] = {
	{ .compatible = "fsl,pwm-rpchip", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_pwm_dt_ids);

static struct platform_driver imx_rpmsg_pwm_driver = {
	.driver = {
		.name	= "imx_rpmsg_pwm",
		.of_match_table = imx_rpmsg_pwm_dt_ids,
	},
	.probe		= pwm_rpchip_probe,
};

static int __init imx_rpmsg_pwm_driver_init(void)
{
	int ret = 0;

	ret = register_rpmsg_driver(&pwm_rpmsg_driver);
	if (ret < 0)
		return ret;

	return platform_driver_register(&(imx_rpmsg_pwm_driver));
}
device_initcall(imx_rpmsg_pwm_driver_init);

MODULE_AUTHOR("Clark Wang<xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("Driver for pwm over rpmsg");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pwm-rpchip");
