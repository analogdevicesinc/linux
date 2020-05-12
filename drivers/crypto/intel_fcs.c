// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020, Intel Corporation
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/firmware/intel/stratix10-svc-client.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include "intel_fcs.h"
#include <uapi/linux/intel_fcs-ioctl.h>

#define RANDOM_NUMBER_SIZE	32
#define FILE_NAME_SIZE		32
#define INVALID_STATUS		0xff

#define FCS_REQUEST_TIMEOUT (msecs_to_jiffies(SVC_FCS_REQUEST_TIMEOUT_MS))
#define FCS_COMPLETED_TIMEOUT (msecs_to_jiffies(SVC_COMPLETED_TIMEOUT_MS))

typedef void (*fcs_callback)(struct stratix10_svc_client *client,
			     struct stratix10_svc_cb_data *data);

struct intel_fcs_priv {
	struct stratix10_svc_chan *chan;
	struct stratix10_svc_client client;
	struct completion completion;
	struct mutex lock;
	struct miscdevice miscdev;
	unsigned int status;
	void *kbuf;
	unsigned int size;
};

static void fcs_data_callback(struct stratix10_svc_client *client,
			     struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;
	unsigned int *status = (unsigned int *)data->kaddr1;

	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	} else {
		dev_err(client->dev, "failed, mbox_error=0x%x\n", *status);
		priv->status = *status;
		priv->kbuf = NULL;
		priv->size = 0;
	}

	complete(&priv->completion);
}

static void fcs_vab_callback(struct stratix10_svc_client *client,
			     struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;
	unsigned int *status = (unsigned int *)data->kaddr1;

	priv->status = 0;

	if (data->status == BIT(SVC_STATUS_INVALID_PARAM)) {
		priv->status = -EINVAL;
		dev_warn(client->dev, "rejected, invalid param\n");
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *status;
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	} else if (data->status == BIT(SVC_STATUS_BUSY)) {
		priv->status = -ETIMEDOUT;
		dev_err(client->dev, "timeout to get completed status\n");
	}

	complete(&priv->completion);
}

static int fcs_request_service(struct intel_fcs_priv *priv,
			       void *msg, unsigned long timeout)
{
	struct stratix10_svc_client_msg *p_msg =
			(struct stratix10_svc_client_msg *)msg;
	int ret;

	mutex_lock(&priv->lock);
	reinit_completion(&priv->completion);

	ret = stratix10_svc_send(priv->chan, p_msg);
	if (ret)
		return -EINVAL;

	ret = wait_for_completion_interruptible_timeout(&priv->completion,
							timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for SMC call\n");
		ret = -ETIMEDOUT;
	} else if (ret < 0)
		dev_err(priv->client.dev,
			"interrupted while waiting for SMC call\n");
	else
		ret = 0;

	mutex_unlock(&priv->lock);
	return ret;
}

static void fcs_close_services(struct intel_fcs_priv *priv,
			       void *sbuf, void *dbuf)
{
	if (sbuf)
		stratix10_svc_free_memory(priv->chan, sbuf);

	if (dbuf)
		stratix10_svc_free_memory(priv->chan, dbuf);

	stratix10_svc_done(priv->chan);
}

static long fcs_ioctl(struct file *file, unsigned int cmd,
		      unsigned long arg)
{
	struct intel_fcs_dev_ioctl *data;
	struct intel_fcs_priv *priv;
	struct device *dev;
	struct stratix10_svc_client_msg *msg;
	const struct firmware *fw;
	char filename[FILE_NAME_SIZE];
	size_t tsz, datasz;
	void *s_buf;
	void *d_buf;
	int ret = 0;
	int i;

	priv = container_of(file->private_data, struct intel_fcs_priv, miscdev);
	dev = priv->client.dev;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	msg = devm_kzalloc(dev, sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	switch (cmd) {
	case INTEL_FCS_DEV_VALIDATION_REQUEST:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			return -EFAULT;
		}

		/* for bitstream */
		dev_dbg(dev, "file_name=%s, status=%d\n",
			 (char *)data->com_paras.s_request.src, data->status);
		scnprintf(filename, FILE_NAME_SIZE, "%s",
				(char *)data->com_paras.s_request.src);
		ret = request_firmware(&fw, filename, priv->client.dev);
		if (ret) {
			dev_err(dev, "error requesting firmware %s\n",
				(char *)data->com_paras.s_request.src);
			return -EFAULT;
		}

		dev_dbg(dev, "FW size=%ld\n", fw->size);
		s_buf = stratix10_svc_allocate_memory(priv->chan, fw->size);
		if (!s_buf) {
			dev_err(dev, "failed to allocate VAB buffer\n");
			release_firmware(fw);
			return -ENOMEM;
		}

		memcpy(s_buf, fw->data, fw->size);

		msg->payload_length = fw->size;
		release_firmware(fw);

		msg->command = COMMAND_FCS_REQUEST_SERVICE;
		msg->payload = s_buf;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		dev_dbg(dev, "fcs_request_service ret=%d\n", ret);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_vab_callback;
			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			dev_dbg(dev, "fcs_request_service ret=%d\n", ret);
			if (!ret && !priv->status)
				data->status = 0;
			else
				data->status = priv->status;
		} else
			data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;

	case INTEL_FCS_DEV_SEND_CERTIFICATE:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			return -EFAULT;
		}

		dev_dbg(dev, "Test=%d, Size=%ld; Address=0x%p\n",
			data->com_paras.c_request.test.test_bit,
			data->com_paras.c_request.size,
			data->com_paras.c_request.addr);

		/* Allocate memory for certificate + test word */
		tsz = sizeof(struct intel_fcs_cert_test_word);
		datasz = data->com_paras.s_request.size + tsz;

		s_buf = stratix10_svc_allocate_memory(priv->chan, datasz);
		if (!s_buf) {
			dev_err(dev, "failed to allocate VAB buffer\n");
			return -ENOMEM;
		}

		/* Copy the test word */
		memcpy(s_buf, &data->com_paras.c_request.test, tsz);

		/* Copy in the certificate data (skipping over the test word) */
		ret = copy_from_user(s_buf + tsz,
				     data->com_paras.c_request.addr,
				     data->com_paras.s_request.size);
		if (ret) {
			dev_err(dev, "failed copy buf ret=%d\n", ret);
			stratix10_svc_free_memory(priv->chan, s_buf);
			return -EFAULT;
		}

		msg->payload_length = datasz;
		msg->command = COMMAND_FCS_SEND_CERTIFICATE;
		msg->payload = s_buf;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		dev_dbg(dev, "fcs_request_service ret=%d\n", ret);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_vab_callback;
			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			dev_dbg(dev, "request service ret=%d\n", ret);
			if (!ret && !priv->status)
				data->status = 0;
			else
				data->status = priv->status;
		} else
			data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;

	case INTEL_FCS_DEV_RANDOM_NUMBER_GEN:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "faiure on copy_from_user\n");
			return -EFAULT;
		}

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      RANDOM_NUMBER_SIZE);
		if (!s_buf) {
			dev_err(dev, "failed to allocate RNG buffer\n");
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_RANDOM_NUMBER_GEN;
		msg->payload = s_buf;
		msg->payload_length = RANDOM_NUMBER_SIZE;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);

		if (!ret && !priv->status) {
			/* for debug only, will be removed for upstream */
			for (i = 0; i < 8; i++)
				dev_info(dev, "output_data[%d]=%d\n", i,
					 *((int *)priv->kbuf + i));

			for (i = 0; i < 8; i++)
				data->com_paras.rn_gen.rndm[i] =
					*((int *)priv->kbuf + i);
			data->status = priv->status;

		} else {
			/* failed to get RNG */
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "faiure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;
	case INTEL_FCS_DEV_GET_PROVISION_DATA:
		if (copy_from_user(data, (void __user *)arg,
				   sizeof(*data))) {
			dev_err(dev, "faiure on copy_from_user\n");
			return -EFAULT;
		}

		s_buf = stratix10_svc_allocate_memory(priv->chan,
					data->com_paras.gp_data.size);
		if (!s_buf) {
			dev_err(dev, "failed allocate provision buffer\n");
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_GET_PROVISION_DATA;
		msg->payload = s_buf;
		msg->payload_length = data->com_paras.gp_data.size;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			data->com_paras.gp_data.size = priv->size;
			ret = copy_to_user(data->com_paras.gp_data.addr,
					   priv->kbuf, priv->size);
			if (ret) {
				dev_err(dev, "faiure on copy_to_user\n");
				fcs_close_services(priv, s_buf, NULL);
				return -EFAULT;
			}
			data->status = 0;
		} else {
			data->com_paras.gp_data.addr = NULL;
			data->com_paras.gp_data.size = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "faiure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			return -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;
	case INTEL_FCS_DEV_DATA_ENCRYPTION:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "faiure on copy_from_user\n");
			return -EFAULT;
		}

		/* allocate buffer for both source and destination */
		s_buf = stratix10_svc_allocate_memory(priv->chan,
				data->com_paras.d_encryption.src_size);
		if (!s_buf) {
			dev_err(dev, "failed allocate encrypt src buf\n");
			return -ENOMEM;
		}
		d_buf = stratix10_svc_allocate_memory(priv->chan,
				data->com_paras.d_encryption.dst_size);
		if (!d_buf) {
			dev_err(dev, "failed allocate encrypt dst buf\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			return -ENOMEM;
		}
		ret = copy_from_user(s_buf,
				     data->com_paras.d_encryption.src,
				     data->com_paras.d_encryption.src_size);
		if (ret) {
			dev_err(dev, "faiure on copy_from_user\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			stratix10_svc_free_memory(priv->chan, d_buf);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_DATA_ENCRYPTION;
		msg->payload = s_buf;
		msg->payload_length =
			data->com_paras.d_encryption.src_size;
		msg->payload_output = d_buf;
		msg->payload_length_output =
			data->com_paras.d_encryption.dst_size;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			data->com_paras.d_encryption.dst_size = priv->size;
			ret = copy_to_user(data->com_paras.d_encryption.dst,
					   priv->kbuf, priv->size);
			if (ret) {
				dev_err(dev, "faiure on copy_to_user\n");
				fcs_close_services(priv, s_buf, d_buf);
				ret = -EFAULT;
			}
			data->status = 0;
		} else {
			data->com_paras.d_encryption.dst = NULL;
			data->com_paras.d_encryption.dst_size = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "faiure on copy_to_user\n");
			fcs_close_services(priv, s_buf, d_buf);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;
	case INTEL_FCS_DEV_DATA_DECRYPTION:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "faiure on copy_from_user\n");
			return -EFAULT;
		}

		/* allocate buffer for both source and destination */
		s_buf = stratix10_svc_allocate_memory(priv->chan,
				data->com_paras.d_decryption.src_size);
		if (!s_buf) {
			dev_err(dev, "failed allocate decrypt src buf\n");
			return -ENOMEM;
		}
		d_buf = stratix10_svc_allocate_memory(priv->chan,
				data->com_paras.d_decryption.dst_size);
		if (!d_buf) {
			dev_err(dev, "failed allocate decrypt dst buf\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			return -ENOMEM;
		}

		ret = copy_from_user(s_buf,
				     data->com_paras.d_decryption.src,
				     data->com_paras.d_decryption.src_size);
		if (ret) {
			dev_err(dev, "faiure on copy_from_user\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			stratix10_svc_free_memory(priv->chan, d_buf);
		}

		msg->command = COMMAND_FCS_DATA_DECRYPTION;
		msg->payload = s_buf;
		msg->payload_length =
				data->com_paras.d_decryption.src_size;
		msg->payload_output = d_buf;
		msg->payload_length_output =
				data->com_paras.d_decryption.dst_size;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			ret = copy_to_user(data->com_paras.d_decryption.dst,
					   priv->kbuf, priv->size);
			if (ret) {
				dev_err(dev, "faiure on copy_to_user\n");
				fcs_close_services(priv, s_buf, d_buf);
				ret = -EFAULT;
			}

			data->com_paras.d_decryption.dst_size = priv->size;
			data->status = 0;
		} else {
			data->com_paras.d_decryption.dst = NULL;
			data->com_paras.d_decryption.dst_size = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "faiure on copy_to_user\n");
			fcs_close_services(priv, s_buf, d_buf);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;
	default:
		dev_warn(dev, "shouldn't be here [0x%x]\n", cmd);
		break;
	}

	return ret;
}

static int fcs_open(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);

	return 0;
}

static int fcs_close(struct inode *inode, struct file *file)
{

	pr_debug("%s\n", __func__);

	return 0;
}

static const struct file_operations fcs_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fcs_ioctl,
	.open = fcs_open,
	.release = fcs_close,
};

static int fcs_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_fcs_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = NULL;
	priv->client.priv = priv;
	priv->status = INVALID_STATUS;

	mutex_init(&priv->lock);
	priv->chan = stratix10_svc_request_channel_byname(&priv->client,
							  SVC_CLIENT_FCS);
	if (IS_ERR(priv->chan)) {
		dev_err(dev, "couldn't get service channel %s\n",
			SVC_CLIENT_FCS);
		return PTR_ERR(priv->chan);
	}

	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = "fcs";
	priv->miscdev.fops = &fcs_fops;

	init_completion(&priv->completion);

	platform_set_drvdata(pdev, priv);

	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(dev, "can't register onn minor=%d\n",
			MISC_DYNAMIC_MINOR);
		return ret;
	}

	return 0;
}

static int fcs_driver_remove(struct platform_device *pdev)
{
	struct intel_fcs_priv *priv = platform_get_drvdata(pdev);

	misc_deregister(&priv->miscdev);
	stratix10_svc_free_channel(priv->chan);

	return 0;
}

static struct platform_driver fcs_driver = {
	.probe = fcs_driver_probe,
	.remove = fcs_driver_remove,
	.driver = {
		.name = "intel-fcs",
	},
};

module_platform_driver(fcs_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel FGPA Crypto Services Driver");
MODULE_AUTHOR("Richard Gong <richard.gong@intel.com>");

