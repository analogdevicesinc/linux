// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Intel Corporation
 */

/*
 * This driver exposes some optional features of the Intel Stratix 10 SoC FPGA.
 * The SysFS interfaces exposed here are FPGA Remote System Update (RSU)
 * related.  They allow user space software to query the configuration system
 * status and to request optional reboot behavior specific to Intel FPGAs.
 */

#include <linux/arm-smccc.h>
#include <linux/completion.h>
#include <linux/intel-service-client.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#define MAX_U64_STR_LEN 22

/*
 * Private data structure
 */
struct intel_rsu_priv {
	struct intel_svc_chan *chan;
	struct intel_svc_client client;
	struct completion svc_completion;
	struct {
		unsigned long current_image;
		unsigned long fail_image;
		unsigned int version;
		unsigned int state;
		unsigned int error_details;
		unsigned int error_location;
	} status;
};

/*
 * status_svc_callback() - Callback from intel-service layer that returns SMC
 *                         response with RSU status data. Parses up data and
 *                         update driver private data structure.
 * client - returned context from intel-service layer
 * data - SMC response data
 */
static void status_svc_callback(struct intel_svc_client *client,
				struct intel_svc_c_data *data)
{
	struct intel_rsu_priv *priv = client->priv;
	struct arm_smccc_res *res = (struct arm_smccc_res *)data->kaddr1;

	if (data->status == BIT(SVC_STATUS_RSU_OK)) {
		priv->status.version =
		    (unsigned int)(res->a2 >> 32) & 0xFFFFFFFF;
		priv->status.state = (unsigned int)res->a2 & 0xFFFFFFFF;
		priv->status.fail_image = res->a1;
		priv->status.current_image = res->a0;
		priv->status.error_location =
		    (unsigned int)res->a3 & 0xFFFFFFFF;
		priv->status.error_details =
		    (unsigned int)(res->a3 >> 32) & 0xFFFFFFFF;
	} else {
		dev_err(client->dev, "COMMAND_RSU_STATUS returned 0x%lX\n",
			res->a0);
		priv->status.version = 0;
		priv->status.state = 0;
		priv->status.fail_image = 0;
		priv->status.current_image = 0;
		priv->status.error_location = 0;
		priv->status.error_details = 0;
	}

	complete(&priv->svc_completion);
}

/*
 * get_status() - Start an intel-service layer transaction to perform the SMC
 *                that is necessary to get RSU status information. Wait for
 *                completion and timeout if needed.
 * priv - driver private data
 *
 * Returns 0 on success
 */
static int get_status(struct intel_rsu_priv *priv)
{
	struct intel_svc_client_msg msg;
	int ret;
	unsigned long timeout;

	reinit_completion(&priv->svc_completion);
	priv->client.receive_cb = status_svc_callback;

	msg.command = COMMAND_RSU_STATUS;
	ret = intel_svc_send(priv->chan, &msg);
	if (ret < 0)
		return ret;

	timeout = msecs_to_jiffies(SVC_RSU_REQUEST_TIMEOUT_MS);
	ret =
	    wait_for_completion_interruptible_timeout(&priv->svc_completion,
						      timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for COMMAND_RSU_STATUS\n");
		return -ETIMEDOUT;
	}
	if (ret < 0) {
		dev_err(priv->client.dev,
			"error (%d) waiting for COMMAND_RSU_STATUS\n", ret);
		return ret;
	}

	return 0;
}

/* current_image_show() - DEVICE_ATTR callback to show current_image status */
static ssize_t current_image_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%ld", priv->status.current_image);
}

/* fail_image_show() - DEVICE_ATTR callback to show fail_image status */
static ssize_t fail_image_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%ld", priv->status.fail_image);
}

/* version_show() - DEVICE_ATTR callback to show version status */
static ssize_t version_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%d", priv->status.version);
}

/* state_show() - DEVICE_ATTR callback to show state status */
static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%d", priv->status.state);
}

/* error_location_show() - DEVICE_ATTR callback to show error_location status */
static ssize_t error_location_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%d", priv->status.error_location);
}

/* error_details_show() - DEVICE_ATTR callback to show error_details status */
static ssize_t error_details_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);

	if (!priv)
		return -ENODEV;

	return sprintf(buf, "%d", priv->status.error_details);
}

/*
 * update_svc_callback() - Callback from intel-service layer that returns SMC
 *                         response from RSU update. Checks for success/fail.
 * client - returned context from intel-service layer
 * data - SMC repsonse data
 */
static void update_svc_callback(struct intel_svc_client *client,
				struct intel_svc_c_data *data)
{
	struct intel_rsu_priv *priv = client->priv;

	if (data->status != BIT(SVC_STATUS_RSU_OK))
		dev_err(client->dev, "COMMAND_RSU_UPDATE returned %i\n",
			data->status);

	complete(&priv->svc_completion);
}

/*
 * send_update() - Start an intel-service layer transaction to perform the SMC
 *                 that is necessary to send an RSU update request. Wait for
 *                 completion and timeout if needed.
 * priv - driver private data
 *
 * Returns 0 on success
 */
static int send_update(struct intel_rsu_priv *priv,
		       unsigned long address)
{
	struct intel_svc_client_msg msg;
	int ret;
	unsigned long timeout;

	reinit_completion(&priv->svc_completion);
	priv->client.receive_cb = update_svc_callback;

	msg.command = COMMAND_RSU_UPDATE;
	msg.arg[0] = address;

	ret = intel_svc_send(priv->chan, &msg);
	if (ret < 0)
		return ret;

	timeout = msecs_to_jiffies(SVC_RSU_REQUEST_TIMEOUT_MS);
	ret = wait_for_completion_interruptible_timeout(&priv->svc_completion,
							timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for COMMAND_RSU_UPDATE\n");
		return -ETIMEDOUT;
	}
	if (ret < 0) {
		dev_err(priv->client.dev,
			"error (%d) waiting for COMMAND_RSU_UPDATE\n", ret);
		return ret;
	}

	return 0;
}

/* reboot_image_store() - DEVICE_ATTR callback to store reboot_image request */
static ssize_t reboot_image_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct intel_rsu_priv *priv = dev_get_drvdata(dev);
	unsigned long address;
	int ret;

	if (priv == 0)
		return -ENODEV;

	/* Ensure the input buffer is null terminated and not too long */
	if (strnlen(buf, MAX_U64_STR_LEN) == MAX_U64_STR_LEN)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &address);
	if (ret)
		return ret;

	send_update(priv, address);

	return count;
}

/*
 * Attribute structures
 */

static DEVICE_ATTR_RO(current_image);
static DEVICE_ATTR_RO(fail_image);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(error_location);
static DEVICE_ATTR_RO(error_details);
static DEVICE_ATTR_WO(reboot_image);

static struct attribute *attrs[] = {
	&dev_attr_current_image.attr,
	&dev_attr_fail_image.attr,
	&dev_attr_state.attr,
	&dev_attr_version.attr,
	&dev_attr_error_location.attr,
	&dev_attr_error_details.attr,
	&dev_attr_reboot_image.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = attrs
};

static int intel_rsu_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct intel_rsu_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = update_svc_callback;
	priv->client.priv = priv;

	priv->status.current_image = 0;
	priv->status.fail_image = 0;
	priv->status.error_location = 0;
	priv->status.error_details = 0;
	priv->status.version = 0;
	priv->status.state = 0;

	priv->chan = request_svc_channel_byname(&priv->client, SVC_CLIENT_RSU);
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "couldn't get service channel (%s)\n",
			SVC_CLIENT_RSU);
		return PTR_ERR(priv->chan);
	}

	init_completion(&priv->svc_completion);

	platform_set_drvdata(pdev, priv);

	ret = get_status(priv);
	if (ret) {
		dev_err(dev, "Error getting RSU status (%i)\n", ret);
		free_svc_channel(priv->chan);
		return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &attr_group);
	if (ret)
		free_svc_channel(priv->chan);

	return ret;
}

static int intel_rsu_remove(struct platform_device *pdev)
{
	struct intel_rsu_priv *priv = platform_get_drvdata(pdev);

	free_svc_channel(priv->chan);

	return 0;
}

static const struct of_device_id intel_rsu_of_match[] = {
	{.compatible = "intel,stratix10-rsu",},
	{},
};
MODULE_DEVICE_TABLE(of, intel_rsu_of_match);

static struct platform_driver intel_rsu_driver = {
	.probe = intel_rsu_probe,
	.remove = intel_rsu_remove,
	.driver = {
		   .name = "intel-rsu",
		   .of_match_table = intel_rsu_of_match,
		   },
};

module_platform_driver(intel_rsu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel Remote System Update SysFS Driver");
MODULE_AUTHOR("David Koltak <david.koltak@linux.intel.com>");
