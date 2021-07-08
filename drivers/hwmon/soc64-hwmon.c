// SPDX-License-Identifier: GPL-2.0
/*
 * 64-bit SoC FPGA hardware monitoring features
 *
 * Copyright (c) 2021 Intel Corporation. All rights reserved
 *
 * Author: Kris Chaplin <kris.chaplin@intel.com>
 */

#include <linux/arm-smccc.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define HWMON_TIMEOUT     (msecs_to_jiffies(SVC_HWMON_REQUEST_TIMEOUT_MS))

#define ETEMP_INACTIVE         0x80000000
#define ETEMP_TOO_OLD          0x80000001
#define ETEMP_NOT_PRESENT      0x80000002
#define ETEMP_TIMEOUT          0x80000003
#define ETEMP_CORRUPT          0x80000004
#define ETEMP_BUSY             0x80000005
#define ETEMP_NOT_INITIALIZED  0x800000FF

#define SOC64_HWMON_MAXSENSORS 16
#define SOC64_HWMON_TEMPERATURE "temperature"
#define SOC64_HWMON_VOLTAGE "voltage"


struct soc64_hwmon_priv {
	struct stratix10_svc_chan *chan;
	struct stratix10_svc_client client;
	struct completion completion;
	struct mutex lock;
	int temperature;
	int voltage;
	int temperature_channels;
	int voltage_channels;
	const char *soc64_volt_chan_names[SOC64_HWMON_MAXSENSORS];
	const char *soc64_temp_chan_names[SOC64_HWMON_MAXSENSORS];
	u32 soc64_volt_chan[SOC64_HWMON_MAXSENSORS];
	u32 soc64_temp_chan[SOC64_HWMON_MAXSENSORS];
};


static umode_t soc64_is_visible(const void *dev,
				enum hwmon_sensor_types type,
				u32 attr, int chan)
{
	const struct soc64_hwmon_priv *priv = dev;

	switch (type) {
	case hwmon_temp:
		if (chan < priv->temperature_channels)
			return 0444;

		return 0;
	case hwmon_in:
		if (chan < priv->voltage_channels)
			return 0444;

		return 0;

	default:
		return 0;
	}
}


static void soc64_readtemp_smc_callback(struct stratix10_svc_client *client,
					struct stratix10_svc_cb_data *data)
{
	struct soc64_hwmon_priv *priv = client->priv;
	struct arm_smccc_res *res = (struct arm_smccc_res *)data->kaddr1;

	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->temperature = res->a0;
	} else {
		dev_err(client->dev, "%s returned 0x%lX\n",
		__func__, res->a0);
	}

	complete(&priv->completion);
}

static void soc64_readvolt_smc_callback(struct stratix10_svc_client *client,
					struct stratix10_svc_cb_data *data)
{
	struct soc64_hwmon_priv *priv = client->priv;
	struct arm_smccc_res *res = (struct arm_smccc_res *)data->kaddr1;

	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->voltage = res->a0;
	} else {
		dev_err(client->dev, "%s returned 0x%lX\n",
				__func__, res->a0);
	}

	complete(&priv->completion);
}


static int soc64_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int chan, long *val)
{
	struct soc64_hwmon_priv *priv = dev_get_drvdata(dev);
	struct stratix10_svc_client_msg msg;
	int ret;

	mutex_lock(&priv->lock);

	reinit_completion(&priv->completion);

	switch (type) {
	case hwmon_temp:
		if (chan > 15)
			return -EOPNOTSUPP;

		msg.arg[0] = (u64) (1 << priv->soc64_temp_chan[chan]);
		priv->client.receive_cb = soc64_readtemp_smc_callback;
		msg.command = COMMAND_HWMON_READTEMP;

		ret = stratix10_svc_send(priv->chan, &msg);
		if (ret < 0)
			goto status_done;

		ret = wait_for_completion_interruptible_timeout(
			&priv->completion, HWMON_TIMEOUT);

		if (!ret) {
			dev_err(priv->client.dev,
				"timeout waiting for SMC call\n");
			ret = -ETIMEDOUT;
			goto status_done;
		} else if (ret < 0) {
			dev_err(priv->client.dev,
				"error %d waiting for SMC call\n", ret);
			goto status_done;
		} else {
			ret = 0;
		}

		*val = ((long)(priv->temperature)) * 1000 /  256;

		switch (priv->temperature) {
		case ETEMP_INACTIVE:
		case ETEMP_NOT_PRESENT:
		case ETEMP_CORRUPT:
		case ETEMP_NOT_INITIALIZED:
			ret = -EOPNOTSUPP;
			break;

		case ETEMP_TIMEOUT:
		case ETEMP_BUSY:
		case ETEMP_TOO_OLD:
			ret = -EAGAIN;
			break;
		default:
			ret = 0;
			break;
		}

		break;

	case hwmon_in: // Read voltage
		if (chan > 15)
			return -EOPNOTSUPP; // Channel outside of range

		msg.arg[0] = (u64) (1 << priv->soc64_volt_chan[chan]);
		priv->client.receive_cb = soc64_readvolt_smc_callback;
		msg.command = COMMAND_HWMON_READVOLT;

		ret = stratix10_svc_send(priv->chan, &msg);
		if (ret < 0)
			goto status_done;

		ret = wait_for_completion_interruptible_timeout(
			&priv->completion, HWMON_TIMEOUT);

		if (!ret) {
			dev_err(priv->client.dev,
				"timeout waiting for SMC call\n");
			ret = -ETIMEDOUT;
			goto status_done;
		}  else if (ret < 0) {
			dev_err(priv->client.dev,
				"error %d waiting for SMC call\n", ret);
			goto status_done;
		} else {
			ret = 0;
		}

		*val = ((long)(priv->voltage)) * 1000 /  65536;
		ret = 0;
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

status_done:
	stratix10_svc_done(priv->chan);
	mutex_unlock(&priv->lock);
	return ret;
}

static int soc64_read_string(struct device *dev,
				enum hwmon_sensor_types type, u32 attr,
				int chan, const char **str)
{
	struct soc64_hwmon_priv *priv = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_in:
		*str = priv->soc64_volt_chan_names[chan];
		return 0;
	case hwmon_temp:
		*str = priv->soc64_temp_chan_names[chan];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}


static const struct hwmon_ops soc64_ops = {
	.is_visible = soc64_is_visible,
	.read = soc64_read,
	.read_string = soc64_read_string,
};

static const struct hwmon_channel_info *soc64_info[] = {
	HWMON_CHANNEL_INFO(temp,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL,
		HWMON_T_INPUT | HWMON_T_LABEL, HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(in,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL,
		HWMON_I_INPUT | HWMON_I_LABEL, HWMON_I_INPUT | HWMON_I_LABEL),
	NULL
};

static const struct hwmon_chip_info soc64_chip_info = {
	.ops = &soc64_ops,
	.info = soc64_info,
};

static int soc64_add_channel(struct device *dev,  const char *type,
				u32 val, const char *label,
				struct soc64_hwmon_priv *priv)
{
	if (!strcmp(type, SOC64_HWMON_TEMPERATURE)) {
		if (priv->temperature_channels >= SOC64_HWMON_MAXSENSORS) {
			dev_warn(dev,
				"Cant add temp node %s, too many channels",
				label);
		return 0;
		}

		priv->soc64_temp_chan_names[priv->temperature_channels] = label;
		priv->soc64_temp_chan[priv->temperature_channels] = val;
		priv->temperature_channels++;
		return 0;
	}

	if (!strcmp(type, SOC64_HWMON_VOLTAGE)) {
		if (priv->voltage_channels >= SOC64_HWMON_MAXSENSORS) {
			dev_warn(dev,
				"Cant add voltage node %s, too many channels",
				label);
			return 0;
		}

		priv->soc64_volt_chan_names[priv->voltage_channels] = label;
		priv->soc64_volt_chan[priv->voltage_channels] = val;
		priv->voltage_channels++;
		return 0;
	}

	dev_warn(dev, "unsupported sensor type %s", type);
	return 0;
}

static int soc64_probe_child_from_dt(struct device *dev,
					struct device_node *child,
					struct soc64_hwmon_priv *priv)
{
	u32 val;
	int ret;
	struct device_node *grandchild;
	const char *label;
	const char *type;

	of_property_read_string(child, "name", &type);
	for_each_child_of_node(child, grandchild) {

		ret = of_property_read_u32(grandchild, "reg", &val);
		if (ret) {
			dev_err(dev, "missing reg property of %pOFn\n",
				grandchild);
			return ret;
		}
		ret = of_property_read_string(grandchild, "label",
				&label);
		if (ret) {
			dev_err(dev, "missing label propoerty of %pOFn\n",
				grandchild);
			return ret;
		}

		soc64_add_channel(dev, type, val, label, priv);
	}

	return 0;
}

static int soc64_probe_from_dt(struct device *dev,
				struct soc64_hwmon_priv *priv)
{
	const struct device_node *np = dev->of_node;
	struct device_node *child;
	int ret;

	/* Compatible with non-DT platforms */
	if (!np)
		return 0;

	for_each_child_of_node(np, child) {
		ret = soc64_probe_child_from_dt(dev, child, priv);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	return 0;
}

static int soc64_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *hwmon_dev;
	struct soc64_hwmon_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = NULL;
	priv->client.priv = priv;
	priv->temperature_channels = 0;
	priv->voltage_channels = 0;

	ret = soc64_probe_from_dt(dev, priv);
	if (ret) {
		dev_err(dev, "Unable to probe from device tree\n");
		return ret;
	}

	dev_info(dev, "Initialized %d temperature and %d voltage channels",
		priv->temperature_channels, priv->voltage_channels);

	mutex_init(&priv->lock);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "soc64hwmon",
							 priv,
							 &soc64_chip_info,
							 NULL);

	priv->chan = stratix10_svc_request_channel_byname(&priv->client,
					SVC_CLIENT_HWMON);
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "couldn't get service channel %s\n",
			SVC_CLIENT_RSU);
	return PTR_ERR(priv->chan);
	}

	init_completion(&priv->completion);
	platform_set_drvdata(pdev, priv);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int soc64_hwmon_remove(struct platform_device *pdev)
{
	struct soc64_hwmon_priv *priv = platform_get_drvdata(pdev);

	stratix10_svc_free_channel(priv->chan);
	return 0;
}

static const struct of_device_id soc64_of_match[] = {
	{ .compatible = "intel,soc64-hwmon" },
	{},
};
MODULE_DEVICE_TABLE(of, soc64_of_match);

static struct platform_driver soc64_hwmon_driver = {
	.driver = {
		.name = "soc64-hwmon",
		.of_match_table = soc64_of_match,
	},
	.probe = soc64_hwmon_probe,
	.remove = soc64_hwmon_remove,
};
module_platform_driver(soc64_hwmon_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("64-bit SoC FPGA hardware monitoring features");
MODULE_LICENSE("GPL v2");
