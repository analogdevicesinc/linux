// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020, Intel Corporation
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hw_random.h>
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

#include <uapi/linux/intel_fcs-ioctl.h>
#include "intel_fcs_smmu.h"

#define RANDOM_NUMBER_SIZE	32
#define RANDOM_NUMBER_EXT_SIZE	4080
#define RANDOM_NUMBER_EXT_OFFSET 12
#define FILE_NAME_SIZE		32
#define PS_BUF_SIZE		64
#define SMMU_BUF_SIZE		128
#define SHA384_SIZE		48
#define INVALID_STATUS		0xFFFFFFFF
#define INVALID_ID		0xFFFFFFFF
#define ASYNC_POLL_SERVICE	0x00004F4E

#define MIN_SDOS_BUF_SZ		16
#define MAX_SDOS_BUF_SZ		32768
#define DEC_MIN_SZ		72
#define DEC_MAX_SZ		32712
#define ENC_MIN_SZ		120
#define ENC_MAX_SZ		32760

#define SUBKEY_CMD_MAX_SZ	4092
#define SUBKEY_RSP_MAX_SZ	820
#define MEASUREMENT_CMD_MAX_SZ	4092
#define MEASUREMENT_RSP_MAX_SZ	4092
#define CERTIFICATE_RSP_MAX_SZ	4096

#define CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ 364
#define CRYPTO_GET_KEY_INFO_MAX_SZ 144

#define CRYPTO_ECC_PARAM_SZ	4
#define CRYPTO_ECC_DIGEST_SZ_OFFSET 4

#define AES_CRYPT_CMD_MAX_SZ	SZ_4M /* set 4 Mb for now */
#define AES_BUFFER_CMD_MAX_SZ	0xE600000 /* set 230 Mb */
#define HMAC_CMD_MAX_SZ			0x1D600000 /* set 470 Mb */
#define ECDSA_CMD_MAX_SZ		0x1D600000 /* set 470 Mb */
#define SMMU_MAX_ALLOC_SZ		0x1E000000 /* set 480 Mb */
#define AES_CRYPT_MODE_ECB	0
#define AES_CRYPT_MODE_CBC	1
#define AES_CRYPT_MODE_CTR	2
#define AES_CRYPT_PARAM_SIZE_ECB	12
#define AES_CRYPT_PARAM_SIZE_CBC_CTR	28

#define FCS_REQUEST_TIMEOUT (msecs_to_jiffies(SVC_FCS_REQUEST_TIMEOUT_MS))
#define FCS_COMPLETED_TIMEOUT (msecs_to_jiffies(SVC_COMPLETED_TIMEOUT_MS))
#define SIGMA_SESSION_ID_ONE	0x1
#define SIGMA_UNKNOWN_SESSION	0xffffffff

#define SRC_BUFFER_STARTING_L2_IDX	17
#define get_buffer_addr(a)(a*2*1024*1024)

#define SDM_SMMU_FW_MIN_VER		0x2722C
#define ATF_SMMU_FW_MAJOR_VER		0x2
#define ATF_SMMU_FW_MIN_VER		0x1
#define AGILEX_PLATFORM			"agilex"
#define AGILEX_PLATFORM_STR_LEN		6

#define SDOS_DECRYPTION_ERROR_102	0x102
#define SDOS_DECRYPTION_ERROR_103	0x103

/*SDM required minimun 8 bytes of data for crypto service*/
#define CRYPTO_SERVICE_MIN_DATA_SIZE	8

/**
 * struct socfpga_fcs_data - FCS platform data structure.
 * @hwrng		Flag to indicate support for HW random number generator.
 */
struct socfpga_fcs_data {
	bool have_hwrng;
};

static char *source_ptr;

typedef void (*fcs_callback)(struct stratix10_svc_client *client,
			     struct stratix10_svc_cb_data *data);

static void fcs_atf_version_smmu_check_callback(struct stratix10_svc_client *client,
			      struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		if ((*((unsigned int *)data->kaddr1) >= ATF_SMMU_FW_MAJOR_VER) &&
			(*((unsigned int *)data->kaddr2) >= ATF_SMMU_FW_MIN_VER))
			priv->status = 0;
		else
			priv->status = -1;
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

static void fcs_fw_version_callback(struct stratix10_svc_client *client,
				    struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = -1;
	if (data->status == BIT(SVC_STATUS_OK)) {
		if (*((unsigned int *)data->kaddr1) > SDM_SMMU_FW_MIN_VER)
			priv->status = 0;
	} else {
		dev_err(client->dev, "Failed to get FW version %lu\n",
			BIT(data->status));
	}

	complete(&priv->completion);
}

static void fcs_data_callback(struct stratix10_svc_client *client,
			      struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	if ((data->status == BIT(SVC_STATUS_OK)) ||
	    (data->status == BIT(SVC_STATUS_COMPLETED))) {
		priv->status = 0;
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "error, mbox_error=0x%x\n", priv->status);
		priv->kbuf = data->kaddr2;
		priv->size = (data->kaddr3) ?
			*((unsigned int *)data->kaddr3) : 0;
	} else if ((data->status == BIT(SVC_STATUS_BUSY)) ||
		   (data->status == BIT(SVC_STATUS_NO_RESPONSE))) {
		priv->status = 0;
		priv->kbuf = NULL;
		priv->size = 0;
	} else {
		dev_err(client->dev, "rejected, invalid param\n");
		priv->status = -EINVAL;
		priv->kbuf = NULL;
		priv->size = 0;
	}

	complete(&priv->completion);
}

static void fcs_vab_callback(struct stratix10_svc_client *client,
			     struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	} else if (data->status == BIT(SVC_STATUS_BUSY)) {
		priv->status = -ETIMEDOUT;
		dev_err(client->dev, "timeout to get completed status\n");
	} else if (data->status == BIT(SVC_STATUS_INVALID_PARAM)) {
		priv->status = -EINVAL;
		dev_err(client->dev, "request rejected\n");
	} else if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
	} else if (data->status == BIT(SVC_STATUS_NO_SUPPORT)) {
		priv->status = -EINVAL;
		dev_err(client->dev, "firmware doesn't support...\n");
	} else {
		priv->status = -EINVAL;
		dev_err(client->dev, "rejected, invalid param\n");
	}

	complete(&priv->completion);
}

static void fcs_chipid_callback(struct stratix10_svc_client *client,
				struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
		priv->cid_low = *((unsigned int *)data->kaddr2);
		priv->cid_high = *((unsigned int *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

static void fcs_attestation_callback(struct stratix10_svc_client *client,
				     struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

static void fcs_crypto_sessionid_callback(struct stratix10_svc_client *client,
					 struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = 0;
		priv->sid = *((unsigned int *)data->kaddr2);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

static void fcs_hwrng_callback(struct stratix10_svc_client *client,
			       struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = 0;
	priv->kbuf = NULL;
	priv->size = 0;

	if ((data->status == BIT(SVC_STATUS_OK)) ||
	    (data->status == BIT(SVC_STATUS_COMPLETED))) {
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	}

	complete(&priv->completion);
}

static void fcs_mbox_send_cmd_callback(struct stratix10_svc_client *client,
				     struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status =  0;
		priv->size = *((unsigned int *)data->kaddr2);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	} else if (data->status == BIT(SVC_STATUS_INVALID_PARAM)) {
		priv->status = -EINVAL;
		dev_err(client->dev, "request rejected\n");
	} else {
		priv->status = -EINVAL;
		dev_err(client->dev, "rejected, invalid param\n");
	}

	complete(&priv->completion);
}

static void fcs_sdos_data_poll_callback(struct stratix10_svc_client *client,
			      struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	if ((data->status == BIT(SVC_STATUS_OK)) ||
	    (data->status == BIT(SVC_STATUS_COMPLETED))) {
		priv->status = (data->kaddr1) ?
			*((unsigned int *)data->kaddr1) : 0;
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "error, mbox_error=0x%x\n", priv->status);
		priv->kbuf = data->kaddr2;
		priv->size = (data->kaddr3) ?
			*((unsigned int *)data->kaddr3) : 0;
	} else if ((data->status == BIT(SVC_STATUS_BUSY)) ||
		   (data->status == BIT(SVC_STATUS_NO_RESPONSE))) {
		priv->status = 0;
		priv->kbuf = NULL;
		priv->size = 0;
	} else {
		dev_err(client->dev, "rejected, invalid param\n");
		priv->status = -EINVAL;
		priv->kbuf = NULL;
		priv->size = 0;
	}

	complete(&priv->completion);
}

static void fcs_sdos_data_callback(struct stratix10_svc_client *client,
				      struct stratix10_svc_cb_data *data)
{
	struct intel_fcs_priv *priv = client->priv;

	priv->status = data->status;
	if (data->status == BIT(SVC_STATUS_OK)) {
		priv->status = *((unsigned int *)data->kaddr1);
		priv->kbuf = data->kaddr2;
		priv->size = *((unsigned int *)data->kaddr3);
	} else if (data->status == BIT(SVC_STATUS_ERROR)) {
		priv->status = *((unsigned int *)data->kaddr1);
		dev_err(client->dev, "mbox_error=0x%x\n", priv->status);
	}

	complete(&priv->completion);
}

static int fcs_request_service(struct intel_fcs_priv *priv,
			       void *msg, unsigned long timeout)
{
	struct stratix10_svc_client_msg *p_msg =
			(struct stratix10_svc_client_msg *)msg;
	int ret;

	reinit_completion(&priv->completion);

	ret = stratix10_svc_send(priv->chan, p_msg);
	if (ret)
		return -EINVAL;

	ret = wait_for_completion_timeout(&priv->completion,
							timeout);
	if (!ret) {
		dev_err(priv->client.dev,
			"timeout waiting for SMC call\n");
		ret = -ETIMEDOUT;
	} else
		ret = 0;

	return ret;
}

static void fcs_free_memory(struct intel_fcs_priv *priv,
				void *buf1, void *buf2, void *buf3)
{
	if (buf1)
		stratix10_svc_free_memory(priv->chan, buf1);

	if (buf2)
		stratix10_svc_free_memory(priv->chan, buf2);

	if (buf3)
		stratix10_svc_free_memory(priv->chan, buf3);
}

static void fcs_close_services(struct intel_fcs_priv *priv,
			       void *sbuf, void *dbuf)
{
	fcs_free_memory(priv, sbuf, dbuf, NULL);
	stratix10_svc_done(priv->chan);
	mutex_unlock(&priv->lock);
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
	size_t tsz, rsz, datasz, ud_sz;
	uint32_t sid;
	uint32_t kuid;
	uint32_t cid;
	void *s_buf;
	void *d_buf;
	void *ps_buf;
	void *iv_field_buf;
	void *input_file_pointer;
	void *output_file_pointer;
	unsigned int buf_sz, in_sz, out_sz;
	uint32_t remaining_size, data_size, total_out_size;
	uint32_t sign_size;
	int ret = 0;
	int i;
	int timeout;
	phys_addr_t src_addr;
	phys_addr_t dst_addr;

	priv = container_of(file->private_data, struct intel_fcs_priv, miscdev);
	dev = priv->client.dev;
	mutex_lock(&priv->lock);
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		mutex_unlock(&priv->lock);
		return -ENOMEM;
	}

	msg = devm_kzalloc(dev, sizeof(*msg), GFP_KERNEL);
	if (!msg) {
		mutex_unlock(&priv->lock);
		return -ENOMEM;
	}

	switch (cmd) {
	case INTEL_FCS_DEV_VALIDATION_REQUEST:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
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
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		dev_dbg(dev, "FW size=%ld\n", fw->size);
		s_buf = stratix10_svc_allocate_memory(priv->chan, fw->size);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed to allocate VAB buffer\n");
			release_firmware(fw);
			mutex_unlock(&priv->lock);
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
			priv->client.receive_cb = fcs_data_callback;
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
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.c_request.size == 0 ||
		    data->com_paras.c_request.addr == NULL) {
			dev_err(dev, "Invalid VAB request param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		dev_dbg(dev, "Test=%d, Size=%d; Address=0x%p\n",
			data->com_paras.c_request.test.test_word,
			data->com_paras.c_request.size,
			data->com_paras.c_request.addr);

		/* Allocate memory for certificate + test word */
		tsz = sizeof(struct intel_fcs_cert_test_word);
		datasz = data->com_paras.c_request.size + tsz;

		s_buf = stratix10_svc_allocate_memory(priv->chan, datasz);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed to allocate VAB buffer\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		/* Copy the test word */
		memcpy(s_buf, &data->com_paras.c_request.test, tsz);

		/* Copy in the certificate data (skipping over the test word) */
		ret = copy_from_user(s_buf + tsz,
				     data->com_paras.c_request.addr,
				     data->com_paras.c_request.size);
		if (ret) {
			dev_err(dev, "failed copy buf ret=%d\n", ret);
			fcs_free_memory(priv, s_buf, ps_buf, NULL);
			mutex_unlock(&priv->lock);
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
			msg->payload = ps_buf;
			msg->payload_length = PS_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;
			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			dev_dbg(dev, "request service ret=%d\n", ret);
			if (!ret && !priv->status) {
				data->status = 0;
				data->mbox_status = 0;
			} else {
				if (priv->kbuf) {
					data->com_paras.c_request.c_status =
						(*(u32 *)priv->kbuf);
					data->mbox_status = priv->status;
					pr_info("data->mbox_status:0x%x\n", data->mbox_status);
				} else
					data->com_paras.c_request.c_status =
						INVALID_STATUS;
			}
		} else
			data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, ps_buf);
		break;

	case INTEL_FCS_DEV_COUNTER_SET_PREAUTHORIZED:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_COUNTER_SET_PREAUTHORIZED;
		msg->arg[0] = data->com_paras.i_request.counter_type;
		msg->arg[1] = data->com_paras.i_request.counter_value;
		msg->arg[2] = data->com_paras.i_request.test.test_word;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the request,ret=%d\n",
				ret);
			fcs_close_services(priv, NULL, NULL);
			return -EFAULT;
		}

		data->status = priv->status;
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_RANDOM_NUMBER_GEN:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      RANDOM_NUMBER_SIZE);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed to allocate RNG buffer\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_RANDOM_NUMBER_GEN;
		msg->payload = s_buf;
		msg->payload_length = RANDOM_NUMBER_SIZE;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);

		if (!ret && !priv->status) {
			if (!priv->kbuf) {
				dev_err(dev, "failure on kbuf\n");
				fcs_close_services(priv, s_buf, NULL);
				return -EFAULT;
			}

			for (i = 0; i < 8; i++)
				dev_dbg(dev, "output_data[%d]=%d\n", i,
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
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;

	case INTEL_FCS_DEV_GET_PROVISION_DATA:
		if (copy_from_user(data, (void __user *)arg,
				   sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.gp_data.size == 0 ||
		    data->com_paras.gp_data.addr == NULL) {
			dev_err(dev, "Invalid provision request param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		s_buf = stratix10_svc_allocate_memory(priv->chan,
					data->com_paras.gp_data.size);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate provision buffer\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_GET_PROVISION_DATA;
		msg->payload = NULL;
		msg->payload_length = 0;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->arg[0] = ASYNC_POLL_SERVICE;
			msg->payload = s_buf;
			msg->payload_length = data->com_paras.gp_data.size;
			msg->command = COMMAND_POLL_SERVICE_STATUS_ASYNC;
			priv->client.receive_cb = fcs_data_callback;

			timeout = 100;
			while (timeout != 0) {
				ret = fcs_request_service(priv, (void *)msg,
							  FCS_REQUEST_TIMEOUT);
				dev_dbg(dev, "request service ret=%d\n", ret);

				if (!ret && !priv->status) {
					if (priv->size) {
						if (!priv->kbuf) {
							dev_err(dev, "failure on kbuf\n");
							fcs_close_services(priv, s_buf, NULL);
							return -EFAULT;
						}

						data->com_paras.gp_data.size = priv->size;
						ret = copy_to_user(data->com_paras.gp_data.addr,
								   priv->kbuf, priv->size);
						if (ret) {
							dev_err(dev, "failure on copy_to_user\n");
							fcs_close_services(priv, s_buf, NULL);
							return -EFAULT;
						}
						break;
					}
				} else {
					data->com_paras.gp_data.addr = NULL;
					data->com_paras.gp_data.size = 0;
					break;
				}
				timeout--;
				mdelay(500);
			}
		} else {
			data->com_paras.gp_data.addr = NULL;
			data->com_paras.gp_data.size = 0;
		}

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			return -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;

	case INTEL_FCS_DEV_DATA_ENCRYPTION:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.src_size < DEC_MIN_SZ ||
		    data->com_paras.d_encryption.src_size > DEC_MAX_SZ) {
			dev_err(dev, "Invalid SDOS Buffer src size:%d\n",
				data->com_paras.d_encryption.src_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.dst_size < ENC_MIN_SZ ||
		    data->com_paras.d_encryption.dst_size > ENC_MAX_SZ) {
			dev_err(dev, "Invalid SDOS Buffer dst size:%d\n",
				data->com_paras.d_encryption.dst_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.src == NULL ||
		    data->com_paras.d_encryption.dst == NULL) {
			dev_err(dev, "Invalid SDOS Buffer pointer\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		/* allocate buffer for both source and destination */
		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      MAX_SDOS_BUF_SZ);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate encrypt src buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}
		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      MAX_SDOS_BUF_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate encrypt dst buf\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}
		ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed allocate p-status buffer\n");
			fcs_free_memory(priv, s_buf, d_buf, NULL);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}
		ret = copy_from_user(s_buf,
				     data->com_paras.d_encryption.src,
				     data->com_paras.d_encryption.src_size);
		if (ret) {
			dev_err(dev, "failure on copy_from_user\n");
			fcs_free_memory(priv, ps_buf, s_buf, d_buf);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_DATA_ENCRYPTION;
		msg->payload = s_buf;
		msg->payload_length =
			data->com_paras.d_encryption.src_size;
		msg->payload_output = d_buf;
		msg->payload_length_output =
			data->com_paras.d_encryption.dst_size;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			msg->payload = ps_buf;
			msg->payload_length = PS_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;

			priv->client.receive_cb = fcs_data_callback;
			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			dev_dbg(dev, "request service ret=%d\n", ret);

			if (!ret && !priv->status) {
				if (!priv->kbuf) {
					dev_err(dev, "failure on kbuf\n");
					fcs_free_memory(priv, ps_buf, s_buf, d_buf);
					fcs_close_services(priv, NULL, NULL);
					return -EFAULT;
				}
				buf_sz = *(unsigned int *)priv->kbuf;
				data->com_paras.d_encryption.dst_size = buf_sz;
				data->status = 0;
				ret = copy_to_user(data->com_paras.d_encryption.dst,
						   d_buf, buf_sz);
				if (ret) {
					dev_err(dev, "failure on copy_to_user\n");
					fcs_free_memory(priv, ps_buf, s_buf, d_buf);
					fcs_close_services(priv, NULL, NULL);
					return -EFAULT;
				}
			} else {
				data->com_paras.d_encryption.dst = NULL;
				data->com_paras.d_encryption.dst_size = 0;
				data->status = priv->status;
			}
		} else {
			data->com_paras.d_encryption.dst = NULL;
			data->com_paras.d_encryption.dst_size = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_free_memory(priv, ps_buf, s_buf, d_buf);
			fcs_close_services(priv, NULL, NULL);
			ret = -EFAULT;
		}

		fcs_free_memory(priv, ps_buf, s_buf, d_buf);
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_DATA_DECRYPTION:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.src_size < ENC_MIN_SZ ||
		    data->com_paras.d_encryption.src_size > ENC_MAX_SZ) {
			dev_err(dev, "Invalid SDOS Buffer src size:%d\n",
				data->com_paras.d_encryption.src_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.dst_size < DEC_MIN_SZ ||
		    data->com_paras.d_encryption.dst_size > DEC_MAX_SZ) {
			dev_err(dev, "Invalid SDOS Buffer dst size:%d\n",
				data->com_paras.d_encryption.dst_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.d_encryption.src == NULL ||
		    data->com_paras.d_encryption.dst == NULL) {
			dev_err(dev, "Invalid SDOS Buffer pointer\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		/* allocate buffer for both source and destination */
		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      MAX_SDOS_BUF_SZ);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate decrypt src buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}
		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      MAX_SDOS_BUF_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate decrypt dst buf\n");
			stratix10_svc_free_memory(priv->chan, s_buf);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan,
						       PS_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed allocate p-status buffer\n");
			fcs_free_memory(priv, s_buf, d_buf, NULL);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		ret = copy_from_user(s_buf,
				     data->com_paras.d_decryption.src,
				     data->com_paras.d_decryption.src_size);
		if (ret) {
			dev_err(dev, "failure on copy_from_user\n");
			fcs_free_memory(priv, ps_buf, s_buf, d_buf);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_DATA_DECRYPTION;
		msg->payload = s_buf;
		msg->payload_length =
				data->com_paras.d_decryption.src_size;
		msg->payload_output = d_buf;
		msg->payload_length_output =
				data->com_paras.d_decryption.dst_size;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			msg->payload = ps_buf;
			msg->payload_length = PS_BUF_SIZE;
			priv->client.receive_cb = fcs_sdos_data_poll_callback;
			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			dev_dbg(dev, "request service ret=%d\n", ret);
			if (!ret &&
			    (!priv->status ||
			    priv->status == SDOS_DECRYPTION_ERROR_102 ||
			    priv->status == SDOS_DECRYPTION_ERROR_103)) {
				if (!priv->kbuf) {
					dev_err(dev, "failure on kbuf\n");
					fcs_free_memory(priv, ps_buf, s_buf, d_buf);
					fcs_close_services(priv, NULL, NULL);
					return -EFAULT;
				}
				buf_sz = *((unsigned int *)priv->kbuf);
				data->com_paras.d_decryption.dst_size = buf_sz;
				data->status = priv->status;
				ret = copy_to_user(data->com_paras.d_decryption.dst,
						   d_buf, buf_sz);
				if (ret) {
					dev_err(dev, "failure on copy_to_user\n");
					fcs_free_memory(priv, ps_buf, s_buf, d_buf);
					fcs_close_services(priv, NULL, NULL);
					return -EFAULT;
				}
			} else {
				data->com_paras.d_decryption.dst = NULL;
				data->com_paras.d_decryption.dst_size = 0;
				data->status = priv->status;
			}
		} else {
			data->com_paras.d_decryption.dst = NULL;
			data->com_paras.d_decryption.dst_size = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_free_memory(priv, ps_buf, s_buf, d_buf);
			fcs_close_services(priv, NULL, NULL);
			ret = -EFAULT;
		}

		fcs_free_memory(priv, ps_buf, s_buf, d_buf);
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_PSGSIGMA_TEARDOWN:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.tdown.sid;
		if ((sid != SIGMA_SESSION_ID_ONE) &&
			(sid != SIGMA_UNKNOWN_SESSION)) {
			dev_err(dev, "Invalid session ID:%d\n", sid);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_PSGSIGMA_TEARDOWN;
		msg->arg[0] = sid;
		priv->client.receive_cb = fcs_vab_callback;
		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the request,ret=%d\n",
				ret);
			fcs_close_services(priv, NULL, NULL);
			return -EFAULT;
		}

		data->status = priv->status;
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_CHIP_ID:
		msg->command = COMMAND_FCS_GET_CHIP_ID;
		priv->client.receive_cb = fcs_chipid_callback;
		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the request,ret=%d\n",
				ret);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		data->status = priv->status;
		data->com_paras.c_id.chip_id_low = priv->cid_low;
		data->com_paras.c_id.chip_id_high = priv->cid_high;
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_ATTESTATION_SUBKEY:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.subkey.cmd_data_sz > SUBKEY_CMD_MAX_SZ) {
			dev_err(dev, "Invalid subkey CMD size %d\n",
				data->com_paras.subkey.cmd_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.subkey.rsp_data_sz > SUBKEY_RSP_MAX_SZ) {
			dev_err(dev, "Invalid subkey RSP size %d\n",
				data->com_paras.subkey.rsp_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.subkey.cmd_data == NULL ||
		    data->com_paras.subkey.rsp_data == NULL) {
			dev_err(dev, "Invalid subkey data pointer\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		/* allocate buffer for both soruce and destination */
		rsz = sizeof(struct intel_fcs_attestation_resv_word);
		datasz = data->com_paras.subkey.cmd_data_sz + rsz;

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      SUBKEY_CMD_MAX_SZ +
						      rsz);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate subkey CMD buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      SUBKEY_RSP_MAX_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate subkey RSP buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		/* copy the reserve word first then command payload */
		memcpy(s_buf, &data->com_paras.subkey.resv.resv_word, rsz);

		/* Copy user data from user space to kernel space */
		ret = copy_from_user(s_buf + rsz,
				     data->com_paras.subkey.cmd_data,
				     data->com_paras.subkey.cmd_data_sz);
		if (ret) {
			dev_err(dev, "failure on copy_from_user\n");
			fcs_free_memory(priv, s_buf, d_buf, NULL);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_ATTESTATION_SUBKEY;
		msg->payload = s_buf;
		msg->payload_length = datasz;
		msg->payload_output = d_buf;
		msg->payload_length_output = SUBKEY_RSP_MAX_SZ;
		priv->client.receive_cb = fcs_attestation_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			if (priv->size > SUBKEY_RSP_MAX_SZ) {
				dev_err(dev,
					"returned size is incorrect\n");
				fcs_close_services(priv, s_buf, d_buf);
				return -EFAULT;
			}

			memcpy(data->com_paras.subkey.rsp_data,
			       priv->kbuf, priv->size);
			data->com_paras.subkey.rsp_data_sz = priv->size;
			data->status = priv->status;

		} else {
			data->com_paras.subkey.rsp_data = NULL;
			data->com_paras.subkey.rsp_data_sz = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;

	case INTEL_FCS_DEV_ATTESTATION_MEASUREMENT:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.measurement.cmd_data_sz > MEASUREMENT_CMD_MAX_SZ) {
			dev_err(dev, "Invalid measurement CMD size %d\n",
				data->com_paras.measurement.cmd_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.measurement.rsp_data_sz > MEASUREMENT_RSP_MAX_SZ) {
			dev_err(dev, "Invalid measurement RSP size %d\n",
				data->com_paras.measurement.rsp_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.measurement.cmd_data == NULL ||
		    data->com_paras.measurement.rsp_data == NULL) {
			dev_err(dev, "Invalid measurement data pointer\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		/* allocate buffer for both soruce and destination */
		rsz = sizeof(struct intel_fcs_attestation_resv_word);
		datasz = data->com_paras.measurement.cmd_data_sz + rsz;

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      MEASUREMENT_CMD_MAX_SZ +
						      rsz);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate measurement CMD buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      MEASUREMENT_RSP_MAX_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate measurement RSP buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		/* copy the reserve word first then command payload */
		memcpy(s_buf, &data->com_paras.measurement.resv.resv_word, rsz);

		/* Copy user data from user space to kernel space */
		ret = copy_from_user(s_buf + rsz,
				     data->com_paras.measurement.cmd_data,
				     data->com_paras.measurement.cmd_data_sz);
		if (ret) {
			dev_err(dev, "failure on copy_from_user\n");
			fcs_free_memory(priv, s_buf, d_buf, NULL);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_ATTESTATION_MEASUREMENTS;
		msg->payload = s_buf;
		msg->payload_length = datasz;
		msg->payload_output = d_buf;
		msg->payload_length_output = MEASUREMENT_RSP_MAX_SZ;
		priv->client.receive_cb = fcs_attestation_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			if (priv->size > MEASUREMENT_RSP_MAX_SZ) {
				dev_err(dev,
					"returned size is incorrect\n");
				fcs_close_services(priv, s_buf, d_buf);
				return -EFAULT;
			}

			memcpy(data->com_paras.measurement.rsp_data,
			       priv->kbuf, priv->size);
			data->com_paras.measurement.rsp_data_sz = priv->size;
			data->status = priv->status;
		} else {
			data->com_paras.measurement.rsp_data = NULL;
			data->com_paras.measurement.rsp_data_sz = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;

	case INTEL_FCS_DEV_ATTESTATION_GET_CERTIFICATE:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.certificate.rsp_data_sz > CERTIFICATE_RSP_MAX_SZ) {
			dev_err(dev, "Invalid certificate RSP size %d\n",
				data->com_paras.certificate.rsp_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      CERTIFICATE_RSP_MAX_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate certificate RSP buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_ATTESTATION_CERTIFICATE;
		msg->payload = NULL;
		msg->payload_length = 0;
		msg->payload_output = d_buf;
		msg->payload_length_output = CERTIFICATE_RSP_MAX_SZ;
		msg->arg[0] = data->com_paras.certificate.c_request & 0x00ff;
		priv->client.receive_cb = fcs_attestation_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			if (priv->size > CERTIFICATE_RSP_MAX_SZ) {
				dev_err(dev,
					"returned size is incorrect\n");
				fcs_close_services(priv, NULL, d_buf);
				return -EFAULT;
			}

			memcpy(data->com_paras.certificate.rsp_data,
			       priv->kbuf, priv->size);
			data->com_paras.certificate.rsp_data_sz = priv->size;
			data->status = priv->status;
		} else {
			data->com_paras.certificate.rsp_data = NULL;
			data->com_paras.certificate.rsp_data_sz = 0;
			data->status = priv->status;
		}

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, NULL, d_buf);
			ret = -EFAULT;
		}

		fcs_close_services(priv, NULL, d_buf);
		break;

	case INTEL_FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_ATTESTATION_CERTIFICATE_RELOAD;
		msg->arg[0] = data->com_paras.c_reload.c_request & 0x00ff;
		priv->client.receive_cb = fcs_vab_callback;
		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the request,ret=%d\n",
				ret);
			fcs_close_services(priv, NULL, NULL);
			return -EFAULT;
		}

		data->status = priv->status;
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_GET_ROM_PATCH_SHA384:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      SHA384_SIZE);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed to allocate RNG buffer\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_GET_ROM_PATCH_SHA384;
		msg->payload = s_buf;
		msg->payload_length = SHA384_SIZE;
		priv->client.receive_cb = fcs_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);

		if (!ret && !priv->status) {
			if (!priv->kbuf) {
				dev_err(dev, "failure on kbuf\n");
				fcs_close_services(priv, s_buf, NULL);
				return -EFAULT;
			}

			if (priv->size > SHA384_SIZE) {
				dev_err(dev, "returned size is incorrect\n");
				fcs_close_services(priv, s_buf, NULL);
				ret = -EFAULT;
			}

			for (i = 0; i < 12; i++)
				dev_dbg(dev, "output_data[%d]=%d\n", i,
					 *((int *)priv->kbuf + i));
			for (i = 0; i < 12; i++)
				data->com_paras.sha384.checksum[i] =
					*((int *)priv->kbuf + i);
			data->status = priv->status;

		} else {
			/* failed to get SHA */
			data->status = priv->status;
		}


		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, NULL);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, NULL);
		break;

	case INTEL_FCS_DEV_CRYPTO_OPEN_SESSION:
		msg->command = COMMAND_FCS_CRYPTO_OPEN_SESSION;
		priv->client.receive_cb = fcs_crypto_sessionid_callback;
		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the cmd=%d,ret=%d\n",
				COMMAND_FCS_CRYPTO_OPEN_SESSION, ret);
			fcs_close_services(priv, NULL, NULL);
			return -EFAULT;
		}

		data->status = priv->status;
		data->com_paras.s_session.sid = priv->sid;
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}
		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_CRYPTO_CLOSE_SESSION:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_CRYPTO_CLOSE_SESSION;
		msg->arg[0] = data->com_paras.s_session.sid;
		priv->client.receive_cb = fcs_vab_callback;
		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret) {
			dev_err(dev, "failed to send the request,ret=%d\n",
				 ret);
			fcs_close_services(priv, NULL, NULL);
			return -EFAULT;
		 }

		 data->status = priv->status;
		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 ret = -EFAULT;
		 }
		 fcs_close_services(priv, NULL, NULL);
		 break;

	case INTEL_FCS_DEV_CRYPTO_IMPORT_KEY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		if (data->com_paras.k_import.obj_data_sz == 0 ||
		    data->com_paras.k_import.obj_data == NULL) {
			dev_err(dev, "Invalid key import request param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		 /* Allocate memory for header + key object */
		 tsz = sizeof(struct fcs_crypto_key_header);
		 datasz = data->com_paras.k_import.obj_data_sz + tsz;

		 s_buf = stratix10_svc_allocate_memory(priv->chan, datasz);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed to allocate key import buffer\n");
			 mutex_unlock(&priv->lock);
			 return -ENOMEM;
		 }

		 ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		 if (IS_ERR(ps_buf)) {
			 dev_err(dev, "failed allocate p-status buffer\n");
			 fcs_free_memory(priv, s_buf, NULL, NULL);
			 mutex_unlock(&priv->lock);
			 return -ENOMEM;
		 }

		 /* copy session ID from the header */
		 memcpy(s_buf, &data->com_paras.k_import.hd.sid, sizeof(uint32_t));
		 ret = copy_from_user(s_buf + tsz,
				      data->com_paras.k_import.obj_data,
				      data->com_paras.k_import.obj_data_sz);
		 if (ret) {
			 dev_err(dev, "failed copy buf ret=%d\n", ret);
			 fcs_free_memory(priv, ps_buf, s_buf, d_buf);
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 msg->payload = s_buf;
		 msg->payload_length = datasz;
		 msg->command = COMMAND_FCS_CRYPTO_IMPORT_KEY;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 dev_dbg(dev, "request service ret=%d\n", ret);

		 if (!ret && !priv->status) {
			 /* to query the complete status */
			 msg->payload = ps_buf;
			 msg->payload_length = PS_BUF_SIZE;
			 msg->command = COMMAND_POLL_SERVICE_STATUS;
			 priv->client.receive_cb = fcs_data_callback;
			 ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			 dev_dbg(dev, "request service ret=%d\n", ret);
			 if (!ret && !priv->status)
				data->status = 0;
			 else {
				data->status = priv->status;
				if (priv->kbuf)
					data->status |= ((*(u32 *)priv->kbuf) & 0xFF)
							<< 16;
			 }
		 } else {
			 data->status = priv->status;
		 }

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, NULL, NULL);
			 fcs_free_memory(priv, ps_buf, s_buf, NULL);
			 return -EFAULT;
		 }

		 fcs_close_services(priv, NULL, NULL);
		 fcs_free_memory(priv, ps_buf, s_buf, NULL);
		 break;

	case INTEL_FCS_DEV_CRYPTO_EXPORT_KEY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 if (data->com_paras.k_object.obj_data_sz >
		     CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ) {
			 dev_err(dev, "Invalid key object size %d\n",
				 data->com_paras.k_object.obj_data_sz);
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan,
				 CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate key object buf\n");
			 mutex_unlock(&priv->lock);
			 return -ENOMEM;
		 }

		 msg->command = COMMAND_FCS_CRYPTO_EXPORT_KEY;
		 msg->payload = NULL;
		 msg->payload_length = 0;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ;
		 msg->arg[0] = data->com_paras.k_object.sid;
		 msg->arg[1] = data->com_paras.k_object.kid;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (!ret && !priv->status) {
			 if (priv->size > CRYPTO_EXPORTED_KEY_OBJECT_MAX_SZ) {
				 dev_err(dev, "returned size %d is incorrect\n",
					 priv->size);
				 fcs_close_services(priv, NULL, d_buf);
				 return -EFAULT;
			 }

			 memcpy(data->com_paras.k_object.obj_data,
				priv->kbuf, priv->size);
			 data->com_paras.k_object.obj_data_sz = priv->size;
		 } else {
			 data->com_paras.k_object.obj_data = NULL;
			 data->com_paras.k_object.obj_data_sz = 0;
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, NULL, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, NULL, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_REMOVE_KEY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 msg->command = COMMAND_FCS_CRYPTO_REMOVE_KEY;
		 msg->arg[0] = data->com_paras.k_object.sid;
		 msg->arg[1] = data->com_paras.k_object.kid;
		 priv->client.receive_cb = fcs_vab_callback;
		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret) {
			 dev_err(dev, "failed to send the request,ret=%d\n",
				 ret);
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 data->status = priv->status;
		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 ret = -EFAULT;
		 }
		 fcs_close_services(priv, NULL, NULL);
		 break;

	case INTEL_FCS_DEV_CRYPTO_GET_KEY_INFO:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 if (data->com_paras.k_object.obj_data_sz > CRYPTO_GET_KEY_INFO_MAX_SZ) {
			 dev_err(dev, "Invalid key object size %d\n",
				 data->com_paras.k_object.obj_data_sz);
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan,
				 CRYPTO_GET_KEY_INFO_MAX_SZ);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate key object buf\n");
			 mutex_unlock(&priv->lock);
			 return -ENOMEM;
		 }

		 msg->command = COMMAND_FCS_CRYPTO_GET_KEY_INFO;
		 msg->payload = NULL;
		 msg->payload_length = 0;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = CRYPTO_GET_KEY_INFO_MAX_SZ;
		 msg->arg[0] = data->com_paras.k_object.sid;
		 msg->arg[1] = data->com_paras.k_object.kid;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (!ret && !priv->status) {
			 if (priv->size > CRYPTO_GET_KEY_INFO_MAX_SZ) {
				 dev_err(dev, "returned size %d is incorrect\n",
					 priv->size);
				 fcs_close_services(priv, NULL, d_buf);
				 return -EFAULT;
			 }

			 memcpy(data->com_paras.k_object.obj_data,
				priv->kbuf, priv->size);
			 data->com_paras.k_object.obj_data_sz = priv->size;
		 } else {
			 data->com_paras.k_object.obj_data = NULL;
			 data->com_paras.k_object.obj_data_sz = 0;
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, NULL, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, NULL, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_AES_CRYPT:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user data\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		if ((data->com_paras.a_crypt.cpara.bmode == AES_CRYPT_MODE_ECB) &&
		   (data->com_paras.a_crypt.cpara_size != AES_CRYPT_PARAM_SIZE_ECB)) {
			dev_err(dev, "AES param size incorrect. Block mode=%d, size=%d\n",
				data->com_paras.a_crypt.cpara.bmode,
				data->com_paras.a_crypt.cpara_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		} else if (((data->com_paras.a_crypt.cpara.bmode == AES_CRYPT_MODE_CBC) ||
			  (data->com_paras.a_crypt.cpara.bmode == AES_CRYPT_MODE_CTR)) &&
			  (data->com_paras.a_crypt.cpara_size != AES_CRYPT_PARAM_SIZE_CBC_CTR)) {
			dev_err(dev, "AES param size incorrect. Block mode=%d, size=%d\n",
				data->com_paras.a_crypt.cpara.bmode,
				data->com_paras.a_crypt.cpara_size);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		} else if (data->com_paras.a_crypt.cpara.bmode > AES_CRYPT_MODE_CTR) {
			dev_err(dev, "Unknown AES block mode. Block mode=%d\n",
				data->com_paras.a_crypt.cpara.bmode);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		iv_field_buf = stratix10_svc_allocate_memory(priv->chan, 28);
		if (IS_ERR(iv_field_buf)) {
			dev_err(dev, "failed allocate iv_field buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		sid = data->com_paras.a_crypt.sid;
		cid = data->com_paras.a_crypt.cid;
		kuid = data->com_paras.a_crypt.kuid;

		memcpy(iv_field_buf, &data->com_paras.a_crypt.cpara.bmode, 1);
		memcpy(iv_field_buf + 1, &data->com_paras.a_crypt.cpara.aes_mode, 1);
		memcpy(iv_field_buf + 12, data->com_paras.a_crypt.cpara.iv_field, 16);

		msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_INIT;
		msg->payload = iv_field_buf;
		msg->payload_length = data->com_paras.a_crypt.cpara_size;
		msg->payload_output = NULL;
		msg->payload_length_output = 0;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;

		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		if (ret || priv->status) {
			dev_err(dev, "failed to send the cmd=%d,ret=%d\n",
				COMMAND_FCS_CRYPTO_AES_CRYPT_INIT,
				ret);
			fcs_close_services(priv, iv_field_buf, NULL);
			return -EFAULT;
		}

		fcs_free_memory(priv, iv_field_buf, NULL, NULL);

		s_buf = stratix10_svc_allocate_memory(priv->chan,
						      data->com_paras.a_crypt.src_size);;
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate source buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan,
						      data->com_paras.a_crypt.src_size);
		if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		}

		ret = copy_from_user(s_buf, data->com_paras.a_crypt.src,
				      data->com_paras.a_crypt.src_size);
		if (ret) {
			dev_err(dev, "failure on copy_from_user\n");
			fcs_close_services(priv, s_buf, d_buf);
			return -EFAULT;
		}

		 ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		 if (IS_ERR(ps_buf)) {
			 dev_err(dev, "failed to allocate p-status buf\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 return -ENOMEM;
		 }

		 msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->payload = s_buf;
		 msg->payload_length = data->com_paras.a_crypt.src_size;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = data->com_paras.a_crypt.dst_size;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		 if (!ps_buf) {
			 dev_err(dev, "failed to allocate p-status buf\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 return -ENOMEM;
		 }

		 while (remaining_size > 0) {
			if (remaining_size > AES_CRYPT_CMD_MAX_SZ) {
				msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE;
				data_size = AES_CRYPT_CMD_MAX_SZ;
				dev_dbg(dev, "AES crypt update. data_size=%d\n", data_size);
			} else {
				msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE;
				data_size = remaining_size;
				dev_dbg(dev, "AES crypt finalize. data_size=%d\n", data_size);
			}

			ret = copy_from_user(s_buf, input_file_pointer, data_size);

			if (ret) {
				dev_err(dev, "failure on copy_from_user s_buf\n");
				fcs_free_memory(priv, s_buf, d_buf, ps_buf);
				fcs_close_services(priv, NULL, NULL);
				return -EFAULT;
			}

			msg->arg[0] = sid;
			msg->arg[1] = cid;
			msg->payload = s_buf;
			msg->payload_length = data_size;
			msg->payload_output = d_buf;
			msg->payload_length_output = data_size;
			priv->client.receive_cb = fcs_attestation_callback;

			ret = fcs_request_service(priv, (void *)msg,
						   FCS_REQUEST_TIMEOUT);
			if (!ret && !priv->status) {
				/* to query the complete status */
				msg->payload = ps_buf;
				msg->payload_length = PS_BUF_SIZE;
				msg->command = COMMAND_POLL_SERVICE_STATUS;
				priv->client.receive_cb = fcs_data_callback;

				ret = fcs_request_service(priv, (void *)msg,
							  FCS_COMPLETED_TIMEOUT);
				if (!ret && !priv->status) {
					if (!priv->kbuf || priv->size != 16) {
						dev_err(dev, "unregconize response\n");
						fcs_free_memory(priv, s_buf, d_buf, ps_buf);
						fcs_close_services(priv, NULL, NULL);
						return -EFAULT;
					}

					buf_sz = ((u32 *)priv->kbuf)[3];

					ret = copy_to_user(output_file_pointer, d_buf, buf_sz);

					total_out_size += buf_sz;

					if (ret) {
						dev_err(dev, "failure on copy_to_user\n");
						fcs_free_memory(priv, s_buf, d_buf, ps_buf);
						fcs_close_services(priv, NULL, NULL);
						return -EFAULT;
					}
				}
			} else {
				data->com_paras.a_crypt.dst = NULL;
				data->com_paras.a_crypt.dst_size = 0;
				dev_err(dev, "unregconize response. ret=%d. status=%d\n",
						ret, priv->status);
				break;
			}

			remaining_size -= data_size;
			if (remaining_size == 0) {
				dev_dbg(dev, "AES crypt finish sending\n");
				data->com_paras.a_crypt.dst_size = total_out_size;
				break;
			} else {
				input_file_pointer += data_size;
				output_file_pointer += data_size;
				dev_dbg(dev, "Complete one update. Remaining size = %d\n",
						remaining_size);
			}
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		 }
		 fcs_free_memory(priv, s_buf, d_buf, ps_buf);
		 fcs_close_services(priv, NULL, NULL);
		 break;

	case INTEL_FCS_DEV_CRYPTO_GET_DIGEST:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.s_mac_data.sid;
		 cid = data->com_paras.s_mac_data.cid;
		 kuid = data->com_paras.s_mac_data.kuid;

		 msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.s_mac_data.sha_op_mode |
			       (data->com_paras.s_mac_data.sha_digest_sz <<
			       CRYPTO_ECC_DIGEST_SZ_OFFSET);

		 priv->client.receive_cb = fcs_vab_callback;
		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_GET_DIGEST_INIT, ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 input_file_pointer = data->com_paras.s_mac_data.src;
		 remaining_size = data->com_paras.s_mac_data.src_size;

		 s_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 while (remaining_size > 0) {
			if (remaining_size > AES_CRYPT_CMD_MAX_SZ) {
				msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE;
				data_size = AES_CRYPT_CMD_MAX_SZ;
				dev_dbg(dev, "Crypto get digest update. data_size=%d\n",
						data_size);
			} else {
				msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE;
				data_size = remaining_size;
				dev_dbg(dev, "Crypto get digest finalize. data_size=%d\n",
						data_size);
			}

			memcpy(s_buf, input_file_pointer, data_size);

			msg->arg[0] = sid;
			msg->arg[1] = cid;
			msg->payload = s_buf;
			msg->payload_length = data_size;
			msg->payload_output = d_buf;
			msg->payload_length_output = AES_CRYPT_CMD_MAX_SZ;
			priv->client.receive_cb = fcs_attestation_callback;

			ret = fcs_request_service(priv, (void *)msg,
						10 * FCS_REQUEST_TIMEOUT);
			if (!ret && !priv->status) {
				if (priv->size > AES_CRYPT_CMD_MAX_SZ) {
					dev_err(dev, "returned size %d is incorrect\n",
						priv->size);
					fcs_close_services(priv, s_buf, d_buf);
					return -EFAULT;
				}
			} else {
				data->com_paras.s_mac_data.dst = NULL;
				data->com_paras.s_mac_data.dst_size = 0;
				dev_err(dev, "unregconize response. ret=%d. status=%d\n",
						ret, priv->status);
				break;
			}

			remaining_size -= data_size;
			if (remaining_size == 0) {
				dev_dbg(dev, "Crypto get digest finish sending\n");
				memcpy(data->com_paras.s_mac_data.dst, priv->kbuf, priv->size);
				data->com_paras.s_mac_data.dst_size = priv->size;
				break;
			} else {
				input_file_pointer += data_size;
				dev_dbg(dev, "Complete update. Remaining size = %d\n",
						remaining_size);
			}
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, s_buf, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_MAC_VERIFY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.s_mac_data.sid;
		 cid = data->com_paras.s_mac_data.cid;
		 kuid = data->com_paras.s_mac_data.kuid;
		 out_sz = data->com_paras.s_mac_data.dst_size;
		 ud_sz = data->com_paras.s_mac_data.userdata_sz;

		 msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.s_mac_data.sha_op_mode |
			       (data->com_paras.s_mac_data.sha_digest_sz <<
			       CRYPTO_ECC_DIGEST_SZ_OFFSET);
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT, ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 input_file_pointer = data->com_paras.s_mac_data.src;
		 remaining_size = data->com_paras.s_mac_data.src_size;
		 sign_size = data->com_paras.s_mac_data.src_size
						- data->com_paras.s_mac_data.userdata_sz;

		 s_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (!d_buf) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 while (remaining_size > 0) {
			if (remaining_size > AES_CRYPT_CMD_MAX_SZ) {
				/* Finalize stage require minimun 8bytes data size */
				if ((remaining_size - AES_CRYPT_CMD_MAX_SZ) >=
					(CRYPTO_SERVICE_MIN_DATA_SIZE + sign_size)) {
					data_size = AES_CRYPT_CMD_MAX_SZ;
					ud_sz = AES_CRYPT_CMD_MAX_SZ;
					dev_dbg(dev, "Update full. data_size=%d, ud_sz=%ld\n",
							data_size, ud_sz);
				} else {
					data_size = (remaining_size - CRYPTO_SERVICE_MIN_DATA_SIZE -
								sign_size);
					ud_sz = (remaining_size - CRYPTO_SERVICE_MIN_DATA_SIZE -
							sign_size);
					dev_dbg(dev, "Update partial. data_size=%d, ud_sz=%ld\n",
							data_size, ud_sz);
				}
				msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE;
			} else {
				data_size = remaining_size;
				ud_sz = remaining_size - sign_size;
				msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE;
				dev_dbg(dev, "Finalize. data_size=%d, ud_sz=%ld\n", data_size,
						ud_sz);
			}

			memcpy(s_buf, input_file_pointer, data_size);

			msg->arg[0] = sid;
			msg->arg[1] = cid;
			msg->arg[2] = ud_sz;
			msg->payload = s_buf;
			msg->payload_length = data_size;
			msg->payload_output = d_buf;
			msg->payload_length_output = out_sz;
			priv->client.receive_cb = fcs_attestation_callback;

			ret = fcs_request_service(priv, (void *)msg,
						10 * FCS_REQUEST_TIMEOUT);
			if (!ret && !priv->status) {
				if (priv->size > out_sz) {
					dev_err(dev, "returned size %d is incorrect\n",
						priv->size);
					fcs_close_services(priv, s_buf, d_buf);
					return -EFAULT;
				}
			} else {
				data->com_paras.s_mac_data.dst = NULL;
				data->com_paras.s_mac_data.dst_size = 0;
				dev_err(dev, "unregconize response. ret=%d. status=%d\n",
						ret, priv->status);
				break;
			}

			remaining_size -= data_size;
			if (remaining_size == 0) {
				dev_dbg(dev, "Crypto get verify finish sending\n");
				memcpy(data->com_paras.s_mac_data.dst, priv->kbuf, priv->size);
				data->com_paras.s_mac_data.dst_size = priv->size;
				break;
			} else {
				input_file_pointer += data_size;
				dev_dbg(dev, "Complete one update. Remaining size = %d\n",
						remaining_size);
			}
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, s_buf, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.ecdsa_data.sid;
		 cid = data->com_paras.ecdsa_data.cid;
		 kuid = data->com_paras.ecdsa_data.kuid;
		 in_sz = data->com_paras.ecdsa_data.src_size;
		 out_sz = data->com_paras.ecdsa_data.dst_size;

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT,
				 ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 s_buf = stratix10_svc_allocate_memory(priv->chan, in_sz);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 memcpy(s_buf, data->com_paras.ecdsa_data.src,
			data->com_paras.ecdsa_data.src_size);

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_FINALIZE;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->payload = s_buf;
		 msg->payload_length = in_sz;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = out_sz;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   10 * FCS_REQUEST_TIMEOUT);
		 if (!ret && !priv->status) {
			 if (priv->size > out_sz) {
				 dev_err(dev, "returned size %d is incorrect\n",
					 priv->size);
				 fcs_close_services(priv, s_buf, d_buf);
				 return -EFAULT;
			 }

			 memcpy(data->com_paras.ecdsa_data.dst,
				priv->kbuf, priv->size);
			 data->com_paras.ecdsa_data.dst_size = priv->size;
		 } else {
			 data->com_paras.ecdsa_data.dst = NULL;
			 data->com_paras.ecdsa_data.dst_size = 0;
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, s_buf, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.ecdsa_data.sid;
		 cid = data->com_paras.ecdsa_data.cid;
		 kuid = data->com_paras.ecdsa_data.kuid;
		 in_sz = data->com_paras.ecdsa_data.src_size;
		 out_sz = data->com_paras.ecdsa_data.dst_size;

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT,
				 ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 input_file_pointer = data->com_paras.ecdsa_data.src;

		 remaining_size = data->com_paras.ecdsa_data.src_size;

		 s_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 while (remaining_size > 0) {
			if (remaining_size > AES_CRYPT_CMD_MAX_SZ) {
				msg->command =
					COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE;
				data_size = AES_CRYPT_CMD_MAX_SZ;
				dev_dbg(dev, "ECDSA data sign update stage. data_size=%d\n",
						data_size);
			} else {
				msg->command =
					COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE;
				data_size = remaining_size;
				dev_dbg(dev, "ECDSA data sign finalize stage. data_size=%d\n",
						data_size);
			}

			memcpy(s_buf, input_file_pointer, data_size);

			msg->arg[0] = sid;
			msg->arg[1] = cid;
			msg->payload = s_buf;
			msg->payload_length = data_size;
			msg->payload_output = d_buf;
			msg->payload_length_output = out_sz;
			priv->client.receive_cb = fcs_attestation_callback;

			ret = fcs_request_service(priv, (void *)msg,
						10 * FCS_REQUEST_TIMEOUT);
			if (!ret && !priv->status) {
				if (priv->size > out_sz) {
					dev_err(dev, "returned size %d is incorrect\n",
						priv->size);
					fcs_close_services(priv, s_buf, d_buf);
					return -EFAULT;
				}
			} else {
				data->com_paras.ecdsa_data.dst = NULL;
				data->com_paras.ecdsa_data.dst_size = 0;
				dev_err(dev, "unregconize response. ret=%d. status=%d\n",
				ret, priv->status);
				break;
			}

			remaining_size -= data_size;
			if (remaining_size == 0) {
				dev_dbg(dev, "ECDSA data sign finish sending\n");
				memcpy(data->com_paras.ecdsa_data.dst, priv->kbuf, priv->size);
				data->com_paras.ecdsa_data.dst_size = priv->size;
				break;
			} else {
				input_file_pointer += data_size;
				dev_dbg(dev, "Complete update. Remaining size = %d\n",
						remaining_size);
			}
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		 fcs_close_services(priv, s_buf, d_buf);
		 break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.ecdsa_data.sid;
		 cid = data->com_paras.ecdsa_data.cid;
		 kuid = data->com_paras.ecdsa_data.kuid;
		 in_sz = data->com_paras.ecdsa_data.src_size;
		 out_sz = data->com_paras.ecdsa_data.dst_size;

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_INIT,
				 ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 s_buf = stratix10_svc_allocate_memory(priv->chan, in_sz);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 memcpy(s_buf, data->com_paras.ecdsa_data.src,
			data->com_paras.ecdsa_data.src_size);

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_HASH_VERIFY_FINALIZE;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->payload = s_buf;
		 msg->payload_length = in_sz;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = out_sz;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   10 * FCS_REQUEST_TIMEOUT);
		 if (!ret && !priv->status) {
			 if (priv->size > out_sz) {
				 dev_err(dev, "returned size %d is incorrect\n",
					 priv->size);
				 fcs_close_services(priv, s_buf, d_buf);
				 return -EFAULT;
			 }

			 memcpy(data->com_paras.ecdsa_data.dst,
				priv->kbuf, priv->size);
			 data->com_paras.ecdsa_data.dst_size = priv->size;
		 } else {
			 data->com_paras.ecdsa_data.dst = NULL;
			 data->com_paras.ecdsa_data.dst_size = 0;
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		fcs_close_services(priv, s_buf, d_buf);

		break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.ecdsa_sha2_data.sid;
		 cid = data->com_paras.ecdsa_sha2_data.cid;
		 kuid = data->com_paras.ecdsa_sha2_data.kuid;
		 in_sz = data->com_paras.ecdsa_sha2_data.src_size;
		 out_sz = data->com_paras.ecdsa_sha2_data.dst_size;
		 ud_sz = data->com_paras.ecdsa_sha2_data.userdata_sz;

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.ecdsa_sha2_data.ecc_algorithm & 0xF;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT,
				 ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 input_file_pointer = data->com_paras.ecdsa_sha2_data.src;
		 remaining_size = data->com_paras.ecdsa_sha2_data.src_size;
		 sign_size = data->com_paras.ecdsa_sha2_data.src_size -
						data->com_paras.ecdsa_sha2_data.userdata_sz;

		 s_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		 if (IS_ERR(s_buf)) {
			 dev_err(dev, "failed allocate source buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, s_buf, NULL);
			 return -ENOMEM;
		 }

		 while (remaining_size > 0) {
			if (remaining_size > AES_CRYPT_CMD_MAX_SZ) {
				/* Finalize stage require minimun 8bytes data size */
				if ((remaining_size - AES_CRYPT_CMD_MAX_SZ) >=
					(CRYPTO_SERVICE_MIN_DATA_SIZE + sign_size)) {
					data_size = AES_CRYPT_CMD_MAX_SZ;
					ud_sz = AES_CRYPT_CMD_MAX_SZ;
					dev_dbg(dev, "Update full. data_size=%d, ud_sz=%ld\n",
							data_size, ud_sz);
				} else {
					data_size = (remaining_size - CRYPTO_SERVICE_MIN_DATA_SIZE -
								sign_size);
					ud_sz = (remaining_size - CRYPTO_SERVICE_MIN_DATA_SIZE -
							sign_size);
					dev_dbg(dev, "Update partial. data_size=%d, ud_sz=%ld\n",
							data_size, ud_sz);
				}
				msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE;
			} else {
				data_size = remaining_size;
				ud_sz = remaining_size - sign_size;
				msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE;
				dev_dbg(dev, "Finalize. data_size=%d, ud_sz=%ld\n", data_size,
						ud_sz);
			}

			memcpy(s_buf, input_file_pointer, data_size);

			msg->arg[0] = sid;
			msg->arg[1] = cid;
			msg->arg[2] = ud_sz;
			msg->payload = s_buf;
			msg->payload_length = data_size;
			msg->payload_output = d_buf;
			msg->payload_length_output = out_sz;
			priv->client.receive_cb = fcs_attestation_callback;

			ret = fcs_request_service(priv, (void *)msg,
						10 * FCS_REQUEST_TIMEOUT);
			if (!ret && !priv->status) {
				if (priv->size > out_sz) {
					dev_err(dev, "returned size %d is incorrect\n",
						priv->size);
					fcs_close_services(priv, s_buf, d_buf);
					return -EFAULT;
				}
			} else {
				data->com_paras.ecdsa_sha2_data.dst = NULL;
				data->com_paras.ecdsa_sha2_data.dst_size = 0;
				dev_err(dev, "unregconize response. ret=%d. status=%d\n",
						ret, priv->status);
				break;
			}

			remaining_size -= data_size;
			if (remaining_size == 0) {
				dev_dbg(dev, "ECDSA data verify finish sending\n");
				memcpy(data->com_paras.ecdsa_sha2_data.dst, priv->kbuf,
						priv->size);
				data->com_paras.ecdsa_sha2_data.dst_size = priv->size;
				break;
			} else {
				input_file_pointer += data_size;
				dev_dbg(dev, "Complete one update. Remaining size = %d\n",
						remaining_size);
			}
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, s_buf, d_buf);
			 ret = -EFAULT;
		 }

		fcs_close_services(priv, s_buf, d_buf);
		break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY:
		 if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			 dev_err(dev, "failure on copy_from_user\n");
			 mutex_unlock(&priv->lock);
			 return -EFAULT;
		 }

		 sid = data->com_paras.ecdsa_data.sid;
		 cid = data->com_paras.ecdsa_data.cid;
		 kuid = data->com_paras.ecdsa_data.kuid;
		 out_sz = data->com_paras.ecdsa_data.dst_size;

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->arg[2] = kuid;
		 msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		 msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		 priv->client.receive_cb = fcs_vab_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		 if (ret || priv->status) {
			 dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				 COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_INIT,
				 ret, priv->status);
			 fcs_close_services(priv, NULL, NULL);
			 return -EFAULT;
		 }

		 d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		 if (IS_ERR(d_buf)) {
			 dev_err(dev, "failed allocate destation buf\n");
			 fcs_close_services(priv, NULL, NULL);
			 return -ENOMEM;
		 }

		 msg->command = COMMAND_FCS_CRYPTO_ECDSA_GET_PUBLIC_KEY_FINALIZE;
		 msg->arg[0] = sid;
		 msg->arg[1] = cid;
		 msg->payload = NULL;
		 msg->payload_length = 0;
		 msg->payload_output = d_buf;
		 msg->payload_length_output = out_sz;
		 priv->client.receive_cb = fcs_attestation_callback;

		 ret = fcs_request_service(priv, (void *)msg,
					   10 * FCS_REQUEST_TIMEOUT);
		 if (!ret && !priv->status) {
			 if (priv->size > out_sz) {
				dev_err(dev, "returned size %d is incorrect\n",
					 priv->size);
				fcs_close_services(priv, NULL, d_buf);
				return -EFAULT;
			 }

			 memcpy(data->com_paras.ecdsa_data.dst,
				priv->kbuf, priv->size);
			 data->com_paras.ecdsa_data.dst_size = priv->size;
		 } else {
			 data->com_paras.ecdsa_data.dst = NULL;
			 data->com_paras.ecdsa_data.dst_size = 0;
		 }

		 data->status = priv->status;

		 if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			 dev_err(dev, "failure on copy_to_user\n");
			 fcs_close_services(priv, NULL, d_buf);
			 ret = -EFAULT;
		 }

		fcs_close_services(priv, NULL, d_buf);
		break;

	case INTEL_FCS_DEV_CRYPTO_ECDH_REQUEST:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.ecdsa_data.src_size == 0 ||
		    data->com_paras.ecdsa_data.src == NULL) {
			dev_err(dev, "Invalid ECDH request src param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.ecdsa_data.dst_size == 0 ||
		    data->com_paras.ecdsa_data.dst == NULL) {
			dev_err(dev, "Invalid ECDH request dst param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.ecdsa_data.sid;
		cid = data->com_paras.ecdsa_data.cid;
		kuid = data->com_paras.ecdsa_data.kuid;
		in_sz = data->com_paras.ecdsa_data.src_size;
		out_sz = data->com_paras.ecdsa_data.dst_size;

		msg->command = COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		priv->client.receive_cb = fcs_vab_callback;

		ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		if (ret || priv->status) {
			dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				COMMAND_FCS_CRYPTO_ECDH_REQUEST_INIT,
				ret, priv->status);
		};

		s_buf = stratix10_svc_allocate_memory(priv->chan, in_sz);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate source buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_close_services(priv, s_buf, NULL);
			return -ENOMEM;
		}

		/* Copy user data from user space to kernel space */
		ret = copy_from_user(s_buf,
				     data->com_paras.ecdsa_data.src,
				     data->com_paras.ecdsa_data.src_size);
		if (ret) {
			dev_err(dev, "failed copy buf ret=%d\n", ret);
			fcs_close_services(priv, s_buf, d_buf);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_CRYPTO_ECDH_REQUEST_FINALIZE;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->payload = s_buf;
		msg->payload_length = in_sz;
		msg->payload_output = d_buf;
		msg->payload_length_output = out_sz;
		priv->client.receive_cb = fcs_attestation_callback;

		ret = fcs_request_service(priv, (void *)msg,
					   10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			if (priv->size > out_sz) {
				dev_err(dev, "returned size %d is incorrect\n",
					priv->size);
				fcs_close_services(priv, s_buf, d_buf);
				return -EFAULT;
			}

			memcpy(data->com_paras.ecdsa_data.dst,
			       priv->kbuf, priv->size);
			data->com_paras.ecdsa_data.dst_size = priv->size;
		} else {
			data->com_paras.ecdsa_data.dst = NULL;
			data->com_paras.ecdsa_data.dst_size = 0;
		}

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, d_buf);
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;

	case INTEL_FCS_DEV_RANDOM_NUMBER_GEN_EXT:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.rn_gen_ext.sid;
		cid = data->com_paras.rn_gen_ext.cid;
		out_sz = data->com_paras.rn_gen_ext.rng_sz;
		buf_sz = RANDOM_NUMBER_EXT_SIZE + RANDOM_NUMBER_EXT_OFFSET;

		d_buf = stratix10_svc_allocate_memory(priv->chan, buf_sz);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed to allocate RNG_EXT output buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		msg->command = COMMAND_FCS_RANDOM_NUMBER_GEN_EXT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = out_sz;
		priv->client.receive_cb = fcs_attestation_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);
		dev_dbg(dev, "request service ret=%d\n", ret);

		timeout = 100;
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->arg[0] = ASYNC_POLL_SERVICE;
			msg->payload = d_buf;
			msg->payload_length = buf_sz;
			msg->command = COMMAND_POLL_SERVICE_STATUS_ASYNC;
			priv->client.receive_cb = fcs_data_callback;

			while (timeout != 0) {
				ret = fcs_request_service(priv, (void *)msg,
							  FCS_REQUEST_TIMEOUT);
				dev_dbg(dev, "request service ret=%d\n", ret);

				if (!ret && !priv->status) {
					if (priv->size == out_sz + RANDOM_NUMBER_EXT_OFFSET) {
						memcpy(data->com_paras.rn_gen_ext.rng_data,
						       priv->kbuf + RANDOM_NUMBER_EXT_OFFSET,
						       out_sz);
						data->com_paras.rn_gen_ext.rng_sz = out_sz;
						break;
					}
				} else {
					data->com_paras.rn_gen_ext.rng_data = NULL;
					data->com_paras.rn_gen_ext.rng_sz = 0;
					break;
				}
				timeout--;
				mdelay(500);
			}
		}

		if (priv->status == 0 && timeout == 0)
			data->status = -ETIMEDOUT;
		else
			data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, NULL, d_buf);
			return -EFAULT;
		}

		fcs_close_services(priv, NULL, d_buf);
		break;

	case INTEL_FCS_DEV_SDOS_DATA_EXT:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.data_sdos_ext.src_size == 0 ||
		    data->com_paras.data_sdos_ext.src == NULL) {
			dev_err(dev, "Invalid SDOS request src param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.data_sdos_ext.dst_size == 0 ||
		    data->com_paras.data_sdos_ext.dst == NULL) {
			dev_err(dev, "Invalid SDOS request dst param\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.data_sdos_ext.sid;
		cid = data->com_paras.data_sdos_ext.cid;
		in_sz = data->com_paras.data_sdos_ext.src_size;

		s_buf = stratix10_svc_allocate_memory(priv->chan, in_sz);
		if (IS_ERR(s_buf)) {
			dev_err(dev, "failed allocate source buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		d_buf = stratix10_svc_allocate_memory(priv->chan, AES_CRYPT_CMD_MAX_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_free_memory(priv, s_buf, NULL, NULL);
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		/* Copy user data from user space to kernel space */
		ret = copy_from_user(s_buf,
				     data->com_paras.data_sdos_ext.src,
				     data->com_paras.data_sdos_ext.src_size);
		if (ret) {
			dev_err(dev, "failed copy buf ret=%d\n", ret);
			fcs_close_services(priv, s_buf, d_buf);
			return -EFAULT;
		}

		msg->command = COMMAND_FCS_SDOS_DATA_EXT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = data->com_paras.data_sdos_ext.op_mode;
		msg->payload = s_buf;
		msg->payload_length = in_sz;
		msg->payload_output = d_buf;
		msg->payload_length_output = AES_CRYPT_CMD_MAX_SZ;
		priv->client.receive_cb = fcs_sdos_data_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);
		if (!ret &&
		    (!priv->status ||
		    priv->status == SDOS_DECRYPTION_ERROR_102 ||
		    priv->status == SDOS_DECRYPTION_ERROR_103)) {
			if (priv->size > AES_CRYPT_CMD_MAX_SZ) {
				dev_err(dev, "returned size %d is incorrect\n",
					priv->size);
				fcs_close_services(priv, s_buf, d_buf);
				return -EFAULT;
			}

			memcpy(data->com_paras.data_sdos_ext.dst,
			       priv->kbuf, priv->size);
			data->com_paras.data_sdos_ext.dst_size = priv->size;
		} else {
			data->com_paras.data_sdos_ext.dst = NULL;
			data->com_paras.data_sdos_ext.dst_size = 0;
		}

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			fcs_close_services(priv, s_buf, d_buf);
			return -EFAULT;
		}
		fcs_close_services(priv, s_buf, d_buf);

		break;

	case INTEL_FCS_DEV_MBOX_SEND:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.mbox_send_cmd.cmd_data_sz % 4) {
			dev_err(dev, "Command data size (%d) is not 4 byte align\n",
			data->com_paras.mbox_send_cmd.cmd_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.mbox_send_cmd.rsp_data_sz % 4) {
			dev_err(dev, "Respond data size (%d) is not 4 byte align\n",
			data->com_paras.mbox_send_cmd.rsp_data_sz);
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		if (data->com_paras.mbox_send_cmd.cmd_data_sz) {
			s_buf = stratix10_svc_allocate_memory(priv->chan,
					data->com_paras.mbox_send_cmd.cmd_data_sz);
			if (IS_ERR(s_buf)) {
				dev_err(dev, "failed allocate source CMD buf\n");
				mutex_unlock(&priv->lock);
				return -ENOMEM;
			}
		} else {
			s_buf = NULL;
		}

		if (data->com_paras.mbox_send_cmd.rsp_data_sz) {
			d_buf = stratix10_svc_allocate_memory(priv->chan,
					data->com_paras.mbox_send_cmd.rsp_data_sz);
			if (IS_ERR(d_buf)) {
				dev_err(dev, "failed allocate destination RSP buf\n");
				fcs_free_memory(priv, s_buf, NULL, NULL);
				mutex_unlock(&priv->lock);
				return -ENOMEM;
			}
		} else {
			d_buf = NULL;
		}

		if (s_buf != NULL) {
			/* Copy user data from user space to kernel space */
			ret = copy_from_user(s_buf,
						data->com_paras.mbox_send_cmd.cmd_data,
						data->com_paras.mbox_send_cmd.cmd_data_sz);
			if (ret) {
				dev_err(dev, "failed copy buf ret=%d\n", ret);
				fcs_free_memory(priv, s_buf, d_buf, NULL);
				mutex_unlock(&priv->lock);
				return -EFAULT;
			}
		}

		msg->command = COMMAND_MBOX_SEND_CMD;
		msg->arg[0] = data->com_paras.mbox_send_cmd.mbox_cmd;
		msg->arg[1] = data->com_paras.mbox_send_cmd.urgent;
		msg->payload = s_buf;
		msg->payload_length = data->com_paras.mbox_send_cmd.cmd_data_sz;
		msg->payload_output = d_buf;
		msg->payload_length_output = data->com_paras.mbox_send_cmd.rsp_data_sz;
		priv->client.receive_cb = fcs_mbox_send_cmd_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  10 * FCS_REQUEST_TIMEOUT);

		if (!ret && !priv->status) {
			if (priv->size > data->com_paras.mbox_send_cmd.rsp_data_sz) {
				dev_err(dev, "Resp data size (%d) bigger than dest size\n",
						priv->size);
				fcs_close_services(priv, s_buf, d_buf);
				return -EFAULT;
			}

			data->com_paras.mbox_send_cmd.rsp_data_sz = priv->size;

			if (data->com_paras.mbox_send_cmd.rsp_data_sz) {
				ret = copy_to_user(data->com_paras.mbox_send_cmd.rsp_data, d_buf,
									data->com_paras.mbox_send_cmd.rsp_data_sz);

				if (ret) {
					dev_err(dev, "failure on copy_to_user\n");
					fcs_close_services(priv, s_buf, d_buf);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.mbox_send_cmd.rsp_data = NULL;
			data->com_paras.mbox_send_cmd.rsp_data_sz = 0;
		}
		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, s_buf, d_buf);
		break;

	case INTEL_FCS_DEV_CHECK_SMMU_ENABLED:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		msg->command = COMMAND_SMC_SVC_VERSION;
		priv->client.receive_cb = fcs_atf_version_smmu_check_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);

		if (!ret && !priv->status)
			data->status = -1;
		else
			return -EFAULT;

		data->status = priv->status;

		msg->command = COMMAND_FIRMWARE_VERSION;
		priv->client.receive_cb = fcs_fw_version_callback;

		ret = fcs_request_service(priv, (void *)msg,
					  FCS_REQUEST_TIMEOUT);

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, NULL, NULL);
		break;

	case INTEL_FCS_DEV_CRYPTO_AES_CRYPT_SMMU:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user data\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		iv_field_buf = stratix10_svc_allocate_memory(priv->chan, 28);
		if (IS_ERR(iv_field_buf)) {
			dev_err(dev, "failed allocate iv_field buf\n");
			mutex_unlock(&priv->lock);
			return -ENOMEM;
		}

		sid = data->com_paras.a_crypt.sid;
		cid = data->com_paras.a_crypt.cid;
		kuid = data->com_paras.a_crypt.kuid;

		memcpy(iv_field_buf, &data->com_paras.a_crypt.cpara.bmode, 1);
		memcpy(iv_field_buf + 1, &data->com_paras.a_crypt.cpara.aes_mode, 1);
		memcpy(iv_field_buf + 12, data->com_paras.a_crypt.cpara.iv_field, 16);

		msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_INIT;
		msg->payload = iv_field_buf;
		msg->payload_length = data->com_paras.a_crypt.cpara_size;
		msg->payload_output = NULL;
		msg->payload_length_output = 0;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;

		priv->client.receive_cb = fcs_vab_callback;

		if (data->com_paras.a_crypt.init == true) {
			ret = fcs_request_service(priv, (void *)msg,
						FCS_REQUEST_TIMEOUT);
			if (ret || priv->status) {
				dev_err(dev, "failed to send the cmd=%d,ret=%d\n",
					COMMAND_FCS_CRYPTO_AES_CRYPT_INIT,
					ret);
				fcs_close_services(priv, iv_field_buf, NULL);
				return -EFAULT;
			}
		}
		fcs_free_memory(priv, iv_field_buf, NULL, NULL);

		remaining_size = data->com_paras.a_crypt.src_size;

		ps_buf = stratix10_svc_allocate_memory(priv->chan, PS_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		if (remaining_size > AES_BUFFER_CMD_MAX_SZ) {
			msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_UPDATE_SMMU;
			data_size = AES_BUFFER_CMD_MAX_SZ;
			dev_dbg(dev, "AES crypt update. data_size=%d\n", data_size);
		} else {
			msg->command = COMMAND_FCS_CRYPTO_AES_CRYPT_FINALIZE_SMMU;
			data_size = remaining_size;
			dev_dbg(dev, "AES crypt finalize. data_size=%d\n", data_size);
		}

		src_addr = get_buffer_addr(SRC_BUFFER_STARTING_L2_IDX);
		dst_addr = get_buffer_addr((SRC_BUFFER_STARTING_L2_IDX +
					data->com_paras.a_crypt.buffer_offset));

		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->payload = &src_addr;
		msg->payload_length = data_size;
		msg->payload_output = &dst_addr;
		msg->payload_length_output = data_size;
		priv->client.receive_cb = fcs_attestation_callback;

		context_bank_enable(priv);

		ret = fcs_request_service(priv, (void *)msg,
					   FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->payload = ps_buf;
			msg->payload_length = PS_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;

			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			if (!ret && !priv->status) {
				if (!priv->kbuf || priv->size != 16) {
					dev_err(dev, "unregconize response\n");
					context_bank_disable(priv);
					fcs_close_services(priv, ps_buf, NULL);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.a_crypt.dst = NULL;
			data->com_paras.a_crypt.dst_size = 0;
			dev_err(dev, "unregconize response. ret=%d. status=%d\n",
					ret, priv->status);
			context_bank_disable(priv);
			fcs_close_services(priv, ps_buf, NULL);
			return -EFAULT;
		}

		context_bank_disable(priv);
		invalidate_smmu_tlb_entries(priv);

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, ps_buf, NULL);
		break;

	case INTEL_FCS_DEV_CRYPTO_GET_DIGEST_SMMU:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.s_mac_data.sid;
		cid = data->com_paras.s_mac_data.cid;
		kuid = data->com_paras.s_mac_data.kuid;

		msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_INIT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = data->com_paras.s_mac_data.sha_op_mode |
			       (data->com_paras.s_mac_data.sha_digest_sz <<
			       CRYPTO_ECC_DIGEST_SZ_OFFSET);

		priv->client.receive_cb = fcs_vab_callback;
		if (data->com_paras.s_mac_data.init == true) {
			ret = fcs_request_service(priv, (void *)msg,
				   FCS_REQUEST_TIMEOUT);
			if (ret || priv->status) {
				dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
				COMMAND_FCS_CRYPTO_GET_DIGEST_INIT, ret, priv->status);
				fcs_close_services(priv, NULL, NULL);
				return -EFAULT;
			}
		}

		remaining_size = data->com_paras.s_mac_data.src_size;

		d_buf = stratix10_svc_allocate_memory(priv->chan,
						       AES_CRYPT_CMD_MAX_SZ);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan, SMMU_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			fcs_close_services(priv, d_buf, NULL);
			return -ENOMEM;
		}

		if (remaining_size > HMAC_CMD_MAX_SZ) {
			msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_UPDATE_SMMU;
			data_size = HMAC_CMD_MAX_SZ;
			dev_dbg(dev, "Crypto get digest update. data_size=%d\n",
					data_size);
		} else {
			msg->command = COMMAND_FCS_CRYPTO_GET_DIGEST_FINALIZE_SMMU;
			data_size = remaining_size;
			dev_dbg(dev, "Crypto get digest finalize. data_size=%d\n",
					data_size);
		}

		src_addr = get_buffer_addr(SRC_BUFFER_STARTING_L2_IDX);

		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->payload = &src_addr;
		msg->payload_length = data_size;
		msg->payload_output = d_buf;
		msg->payload_length_output = AES_CRYPT_CMD_MAX_SZ;
		priv->client.receive_cb = fcs_attestation_callback;

		context_bank_enable(priv);

		ret = fcs_request_service(priv, (void *)msg,
					10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->payload = ps_buf;
			msg->payload_length = SMMU_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;

			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			if (!ret && !priv->status) {
				if (!priv->kbuf) {
					dev_err(dev, "unregconize response\n");
					context_bank_disable(priv);
					fcs_close_services(priv, d_buf, ps_buf);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.s_mac_data.dst = NULL;
			data->com_paras.s_mac_data.dst_size = 0;
			dev_err(dev, "unregconize response. ret=%d. status=%d\n",
					ret, priv->status);
			context_bank_disable(priv);
			fcs_close_services(priv, d_buf, ps_buf);
			return -EFAULT;
		}

		remaining_size -= data_size;
		if (remaining_size == 0) {
			dev_dbg(dev, "Crypto get digest finish sending\n");
			memcpy(data->com_paras.s_mac_data.dst, priv->kbuf, priv->size);
			data->com_paras.s_mac_data.dst_size = priv->size;
		}

		context_bank_disable(priv);
		invalidate_smmu_tlb_entries(priv);

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, ps_buf, d_buf);
		break;

	case INTEL_FCS_DEV_CRYPTO_MAC_VERIFY_SMMU:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.s_mac_data.sid;
		cid = data->com_paras.s_mac_data.cid;
		kuid = data->com_paras.s_mac_data.kuid;
		out_sz = data->com_paras.s_mac_data.dst_size;
		ud_sz = data->com_paras.s_mac_data.userdata_sz;

		msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = data->com_paras.s_mac_data.sha_op_mode |
			       (data->com_paras.s_mac_data.sha_digest_sz <<
			       CRYPTO_ECC_DIGEST_SZ_OFFSET);
		priv->client.receive_cb = fcs_vab_callback;

		if (data->com_paras.s_mac_data.init == true) {
			ret = fcs_request_service(priv, (void *)msg,
						FCS_REQUEST_TIMEOUT);
			if (ret || priv->status) {
				dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
					COMMAND_FCS_CRYPTO_MAC_VERIFY_INIT, ret, priv->status);
				fcs_close_services(priv, NULL, NULL);
				return -EFAULT;
			}
		}

		remaining_size = data->com_paras.s_mac_data.src_size;

		d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan, SMMU_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			fcs_close_services(priv, d_buf, NULL);
			return -ENOMEM;
		}

		if (remaining_size > HMAC_CMD_MAX_SZ) {
			if (data->com_paras.s_mac_data.userdata_sz >= HMAC_CMD_MAX_SZ) {
				data_size = HMAC_CMD_MAX_SZ;
				ud_sz = HMAC_CMD_MAX_SZ;
			} else {
				data_size = data->com_paras.s_mac_data.userdata_sz;
				ud_sz = data->com_paras.s_mac_data.userdata_sz;
			}
			msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_UPDATE_SMMU;
		} else {
			data_size = remaining_size;
			ud_sz = data->com_paras.s_mac_data.userdata_sz;
			memcpy(d_buf, (source_ptr+ud_sz), (remaining_size-ud_sz));
			msg->command = COMMAND_FCS_CRYPTO_MAC_VERIFY_FINALIZE_SMMU;
			dev_dbg(dev, "Finalize. data_size=%d, ud_sz=%ld\n", data_size,
					ud_sz);

		}

		src_addr = get_buffer_addr(SRC_BUFFER_STARTING_L2_IDX);

		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = ud_sz;
		msg->payload = &src_addr;
		msg->payload_length = data_size;
		msg->payload_output = d_buf;
		msg->payload_length_output = out_sz;
		priv->client.receive_cb = fcs_attestation_callback;

		context_bank_enable(priv);

		ret = fcs_request_service(priv, (void *)msg,
					10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->payload = ps_buf;
			msg->payload_length = SMMU_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;

			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			if (!ret && !priv->status) {
				if (!priv->kbuf) {
					dev_err(dev, "unregconize response\n");
					context_bank_disable(priv);
					fcs_close_services(priv, d_buf, ps_buf);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.s_mac_data.dst = NULL;
			data->com_paras.s_mac_data.dst_size = 0;
			dev_err(dev, "unregconize response. ret=%d. status=%d\n",
					ret, priv->status);
			context_bank_disable(priv);
			fcs_close_services(priv, ps_buf, d_buf);
			return -EFAULT;
		}

		remaining_size -= data_size;
		if (remaining_size == 0) {
			dev_dbg(dev, "Crypto get verify finish sending\n");
			memcpy(data->com_paras.s_mac_data.dst, priv->kbuf, priv->size);
			data->com_paras.s_mac_data.dst_size = priv->size;
		}

		context_bank_disable(priv);
		invalidate_smmu_tlb_entries(priv);
		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, ps_buf, d_buf);
		break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_SMMU:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.ecdsa_data.sid;
		cid = data->com_paras.ecdsa_data.cid;
		kuid = data->com_paras.ecdsa_data.kuid;
		in_sz = data->com_paras.ecdsa_data.src_size;
		out_sz = data->com_paras.ecdsa_data.dst_size;

		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_INIT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = data->com_paras.ecdsa_data.ecc_algorithm & 0xF;
		priv->client.receive_cb = fcs_vab_callback;

		if (data->com_paras.ecdsa_data.init == true) {
			ret = fcs_request_service(priv, (void *)msg,
				   FCS_REQUEST_TIMEOUT);
			if (ret || priv->status) {
				dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
					COMMAND_FCS_CRYPTO_ECDSA_HASH_SIGNING_INIT,
					ret, priv->status);
				fcs_close_services(priv, NULL, NULL);
				return -EFAULT;
			}
		}

		remaining_size = data->com_paras.ecdsa_data.src_size;

		d_buf = stratix10_svc_allocate_memory(priv->chan, out_sz);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan, SMMU_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			fcs_close_services(priv, d_buf, NULL);
			return -ENOMEM;
		}

		if (remaining_size > ECDSA_CMD_MAX_SZ) {
			msg->command =
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_UPDATE_SMMU;
			data_size = ECDSA_CMD_MAX_SZ;
			dev_dbg(dev, "ECDSA data sign update stage. data_size=%d\n",
					data_size);
		} else {
			msg->command =
				COMMAND_FCS_CRYPTO_ECDSA_SHA2_DATA_SIGNING_FINALIZE_SMMU;
			data_size = remaining_size;
			dev_dbg(dev, "ECDSA data sign finalize stage. data_size=%d\n",
					data_size);
		}

		src_addr = get_buffer_addr(SRC_BUFFER_STARTING_L2_IDX);

		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->payload = &src_addr;
		msg->payload_length = data_size;
		msg->payload_output = d_buf;
		msg->payload_length_output = out_sz;
		priv->client.receive_cb = fcs_attestation_callback;

		context_bank_enable(priv);

		ret = fcs_request_service(priv, (void *)msg,
					10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			msg->payload = ps_buf;
			msg->payload_length = SMMU_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;

			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			if (!ret && !priv->status) {
				if (!priv->kbuf) {
					dev_err(dev, "unregconize response\n");
					context_bank_disable(priv);
					fcs_close_services(priv, d_buf, ps_buf);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.ecdsa_data.dst = NULL;
			data->com_paras.ecdsa_data.dst_size = 0;
			dev_err(dev, "unregconize response. ret=%d. status=%d\n",
				ret, priv->status);
			context_bank_disable(priv);
			fcs_close_services(priv, d_buf, ps_buf);
			return -EFAULT;
		}

		remaining_size -= data_size;
		if (remaining_size == 0) {
			dev_dbg(dev, "ECDSA data sign finish sending\n");
			memcpy(data->com_paras.ecdsa_data.dst, priv->kbuf, priv->size);
			data->com_paras.ecdsa_data.dst_size = priv->size;
		}

		context_bank_disable(priv);
		invalidate_smmu_tlb_entries(priv);

		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, ps_buf, d_buf);
		break;

	case INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_SMMU:
		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			dev_err(dev, "failure on copy_from_user\n");
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}

		sid = data->com_paras.ecdsa_sha2_data.sid;
		cid = data->com_paras.ecdsa_sha2_data.cid;
		kuid = data->com_paras.ecdsa_sha2_data.kuid;
		in_sz = data->com_paras.ecdsa_sha2_data.src_size;
		out_sz = data->com_paras.ecdsa_sha2_data.dst_size;
		ud_sz = data->com_paras.ecdsa_sha2_data.userdata_sz;

		msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT;
		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = kuid;
		msg->arg[3] = CRYPTO_ECC_PARAM_SZ;
		msg->arg[4] = data->com_paras.ecdsa_sha2_data.ecc_algorithm & 0xF;
		priv->client.receive_cb = fcs_vab_callback;

		if (data->com_paras.ecdsa_sha2_data.init == true) {
			ret = fcs_request_service(priv, (void *)msg,
				   FCS_REQUEST_TIMEOUT);
			if (ret || priv->status) {
				dev_err(dev, "failed to send the cmd=%d,ret=%d, status=%d\n",
					COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_INIT,
					ret, priv->status);
				fcs_close_services(priv, NULL, NULL);
				return -EFAULT;
			}
		}

		remaining_size = data->com_paras.ecdsa_sha2_data.src_size;

		d_buf = stratix10_svc_allocate_memory(priv->chan, SMMU_BUF_SIZE);
		if (IS_ERR(d_buf)) {
			dev_err(dev, "failed allocate destation buf\n");
			fcs_close_services(priv, NULL, NULL);
			return -ENOMEM;
		}

		ps_buf = stratix10_svc_allocate_memory(priv->chan, SMMU_BUF_SIZE);
		if (IS_ERR(ps_buf)) {
			dev_err(dev, "failed to allocate p-status buf\n");
			fcs_close_services(priv, d_buf, NULL);
			return -ENOMEM;
		}

		if (remaining_size > ECDSA_CMD_MAX_SZ) {
			if (data->com_paras.s_mac_data.userdata_sz >= ECDSA_CMD_MAX_SZ) {
				data_size = ECDSA_CMD_MAX_SZ;
				ud_sz = ECDSA_CMD_MAX_SZ;
			} else {
				data_size = data->com_paras.ecdsa_sha2_data.userdata_sz;
				ud_sz = data->com_paras.ecdsa_sha2_data.userdata_sz;
			}
			msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_UPDATE_SMMU;
		} else {
			data_size = remaining_size;
			ud_sz = data->com_paras.ecdsa_sha2_data.userdata_sz;
			memcpy(d_buf, (source_ptr+ud_sz), (remaining_size-ud_sz));
			msg->command = COMMAND_FCS_CRYPTO_ECDSA_SHA2_VERIFY_FINALIZE_SMMU;
			dev_dbg(dev, "Finalize. data_size=%d, ud_sz=%ld\n", data_size,
					ud_sz);
		}

		src_addr = get_buffer_addr(SRC_BUFFER_STARTING_L2_IDX);

		msg->arg[0] = sid;
		msg->arg[1] = cid;
		msg->arg[2] = ud_sz;
		msg->payload = &src_addr;
		msg->payload_length = data_size;
		msg->payload_output = d_buf;
		msg->payload_length_output = out_sz;
		priv->client.receive_cb = fcs_attestation_callback;

		context_bank_enable(priv);

		ret = fcs_request_service(priv, (void *)msg,
					10 * FCS_REQUEST_TIMEOUT);
		if (!ret && !priv->status) {
			/* to query the complete status */
			msg->payload = ps_buf;
			msg->payload_length = SMMU_BUF_SIZE;
			msg->command = COMMAND_POLL_SERVICE_STATUS;
			priv->client.receive_cb = fcs_data_callback;

			ret = fcs_request_service(priv, (void *)msg,
						  FCS_COMPLETED_TIMEOUT);
			if (!ret && !priv->status) {
				if (!priv->kbuf) {
					dev_err(dev, "unregconize response\n");
					context_bank_disable(priv);
					fcs_close_services(priv, d_buf, ps_buf);
					return -EFAULT;
				}
			}
		} else {
			data->com_paras.ecdsa_sha2_data.dst = NULL;
			data->com_paras.ecdsa_sha2_data.dst_size = 0;
			dev_err(dev, "unregconize response. ret=%d. status=%d\n",
					ret, priv->status);
			context_bank_disable(priv);
			fcs_close_services(priv, d_buf, ps_buf);
			return -EFAULT;
		}

		remaining_size -= data_size;
		if (remaining_size == 0) {
			dev_dbg(dev, "ECDSA data verify finish sending\n");
			memcpy(data->com_paras.ecdsa_sha2_data.dst, priv->kbuf,
					priv->size);
			data->com_paras.ecdsa_sha2_data.dst_size = priv->size;
		}

		context_bank_disable(priv);
		invalidate_smmu_tlb_entries(priv);
		data->status = priv->status;

		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			dev_err(dev, "failure on copy_to_user\n");
			ret = -EFAULT;
		}

		fcs_close_services(priv, d_buf, ps_buf);
		break;

	default:
		mutex_unlock(&priv->lock);
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

static int fcs_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct stratix10_svc_client_msg *msg;
	struct intel_fcs_priv *priv;
	struct device *dev;
	void *s_buf;
	int ret = 0;
	size_t size = 0;

	priv = (struct intel_fcs_priv *)rng->priv;
	dev = priv->client.dev;
	mutex_lock(&priv->lock);
	msg = devm_kzalloc(dev, sizeof(*msg), GFP_KERNEL);
	if (!msg) {
		dev_err(dev, "failed to allocate msg buffer\n");
		mutex_unlock(&priv->lock);
		return -ENOMEM;
	}

	s_buf = stratix10_svc_allocate_memory(priv->chan,
					      RANDOM_NUMBER_SIZE);
	if (IS_ERR(s_buf)) {
		dev_err(dev, "failed to allocate random number buffer\n");
		mutex_unlock(&priv->lock);
		return -ENOMEM;
	}

	msg->command = COMMAND_FCS_RANDOM_NUMBER_GEN;
	msg->payload = s_buf;
	msg->payload_length = RANDOM_NUMBER_SIZE;
	priv->client.receive_cb = fcs_hwrng_callback;

	ret = fcs_request_service(priv, (void *)msg,
				  FCS_REQUEST_TIMEOUT);
	if (!ret && !priv->status) {
		if (priv->size && priv->kbuf) {
			if (max > priv->size)
				size = priv->size;
			else
				size = max;

			memcpy((uint8_t *)buf, (uint8_t *)priv->kbuf, size);
		}
	}

	fcs_close_services(priv, s_buf, NULL);

	if (size == 0)
		return -ENOTSUPP;

	return size;
}

static int fcs_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size, off;
	struct page *page;

	if (!source_ptr) {
		pr_err("vmalloc failed mmap %s", __func__);
		return -ENOMEM;
	}

	size = vma->vm_end - vma->vm_start;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_DONTEXPAND;
	for (off = 0; off < size; off += PAGE_SIZE) {
		page = vmalloc_to_page(source_ptr + off);
		if (vm_insert_page(vma, vma->vm_start + off, page))
			pr_err("vm_insert_page() failed");

	}

	return 0;
}

static const struct file_operations fcs_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = fcs_ioctl,
	.open = fcs_open,
	.release = fcs_close,
	.mmap = fcs_mmap
};

static int fcs_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_fcs_priv *priv;
	int ret, i;
	const char *platform;
	struct stratix10_svc_client_msg msg;
	unsigned long off;
	int l2_idx = SRC_BUFFER_STARTING_L2_IDX;
	int l3_idx = 0;
	uint64_t phys;
	unsigned long pfn;
	struct page *page;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client.dev = dev;
	priv->client.receive_cb = NULL;
	priv->client.priv = priv;
	priv->kbuf = NULL;
	priv->size = 0;
	priv->status = INVALID_STATUS;
	priv->cid_low = INVALID_ID;
	priv->cid_high = INVALID_ID;
	priv->sid = INVALID_ID;

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

	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(dev, "can't register on minor=%d\n",
			MISC_DYNAMIC_MINOR);
		goto release_channel;
	}

	priv->p_data = of_device_get_match_data(dev);
	if (!priv->p_data)
		goto cleanup;
	
	ret = of_property_read_string(dev->of_node, "platform", &platform);
	if (ret) {
		dev_err(dev, "can't find platform");
		goto cleanup;
	}

	/* Proceed only if platform is agilex as
	 * register addresses are platform specific
	 */
	if (!strncmp(platform, AGILEX_PLATFORM, AGILEX_PLATFORM_STR_LEN)) {

		msg.command = COMMAND_SMC_SVC_VERSION;
		priv->client.receive_cb = fcs_atf_version_smmu_check_callback;

		ret = stratix10_svc_send(priv->chan, &msg);
		if (ret)
			return -EINVAL;

		ret = wait_for_completion_timeout(&priv->completion,
							FCS_REQUEST_TIMEOUT);
		if (!ret) {
			dev_err(priv->client.dev, "timeout waiting for SMC call\n");
			ret = -ETIMEDOUT;
			return ret;
		}

		/* Program registers only if ATF support programming
		 * SMMU secure register addresses
		 */
		if (priv->status == 0) {
			l1_table = kmalloc((sizeof(uint64_t)*512), GFP_KERNEL);
			if (!l1_table)
				return -ENOMEM;
			l2_table = kmalloc((sizeof(uint64_t)*512), GFP_KERNEL);
			if (!l2_table)
				return -ENOMEM;

			memcpy(l1_table, smmu_sdm_el3_l1_table, (sizeof(uint64_t)*512));
			memcpy(l2_table, smmu_sdm_el3_l2_table, (sizeof(uint64_t)*512));

			for (i = 0; i < 512; i++) {
				l3_tables[i] = kmalloc((sizeof(uint64_t)*512), GFP_KERNEL);
				if (!l3_tables[i])
					return -ENOMEM;
				memcpy(l3_tables[i], smmu_sdm_l3_def_table, (sizeof(uint64_t)*512));
			}

			if (source_ptr)
				vfree(source_ptr);

			source_ptr = vmalloc_user(SMMU_MAX_ALLOC_SZ);
			if (!source_ptr) {
				pr_err("vmalloc failed probe %s", __func__);
				return -ENOMEM;
			}

			for (off = 0; off < SMMU_MAX_ALLOC_SZ; off += PAGE_SIZE) {
				page = vmalloc_to_page(source_ptr + off);
				pfn = page_to_pfn(page);
				phys = __pa(pfn_to_kaddr(pfn)) + offset_in_page(source_ptr + off);

				if (l3_idx >= 512) {
					l2_idx++;
					l3_idx = 0;
				}
				fill_l3_table(phys, l2_idx, l3_idx);
				l3_idx++;
			}

			intel_fcs_smmu_init(priv);
		}
	}

	/* only register the HW RNG if the platform supports it! */
	if (priv->p_data->have_hwrng) {
		/* register hwrng device */
		priv->rng.name = "intel-rng";
		priv->rng.read = fcs_rng_read;
		priv->rng.priv = (unsigned long)priv;

		ret = hwrng_register(&priv->rng);
		if (ret) {
			dev_err(dev, "can't register RNG device (%d)\n", ret);
			return ret;
		}
	} else {
		/* Notes of registering /dev/hwrng:
		 * 1 For now, /dev/hwrng is not supported on Agilex devices
		 *   due to hardware implementation.
		 * 2 It means On Agilex devices, /dev/hwrng is a dummy node
		 *   without HW backend. You can get the HW RNG function by
		 *   IOCTL command provided from this driver on Agilex devices.
		 * 3 In the future, it may be implemented in a different way.
		 */
		dev_notice(dev, "/dev/hwrng is not supported on Agilex devices.\n");
	}

	platform_set_drvdata(pdev, priv);

	return 0;

cleanup:
	misc_deregister(&priv->miscdev);
release_channel:
	stratix10_svc_free_channel(priv->chan);
	return -ENODEV;
}

static int fcs_driver_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct intel_fcs_priv *priv = platform_get_drvdata(pdev);
	struct stratix10_svc_client_msg msg;
	int i, ret;
	const char *platform;

	ret = of_property_read_string(dev->of_node, "platform", &platform);
	if (ret)
		goto no_platform;

	if (!strncmp(platform, AGILEX_PLATFORM, AGILEX_PLATFORM_STR_LEN)) {
		msg.command = COMMAND_SMC_SVC_VERSION;
		priv->client.receive_cb = fcs_atf_version_smmu_check_callback;

		ret = stratix10_svc_send(priv->chan, &msg);

		ret = wait_for_completion_timeout(&priv->completion,
							FCS_REQUEST_TIMEOUT);

		/* Program registers only if ATF support programming
		 * SMMU secure register addresses
		 */
		if (priv->status == 0) {
			kfree(l1_table);
			kfree(l2_table);

			for (i = 0; i < 512; i++)
				kfree(l3_tables[i]);

			if (source_ptr)
				vfree(source_ptr);

			context_bank_disable(priv);
		}
	}

no_platform:
	if (priv->p_data->have_hwrng)
		hwrng_unregister(&priv->rng);
	misc_deregister(&priv->miscdev);
	stratix10_svc_free_channel(priv->chan);

	return 0;
}

/* Note: /dev/hwrng is not supported on Agilex devices now! */
static const struct socfpga_fcs_data agilex_fcs_data = {
	.have_hwrng	= false,
};

static const struct socfpga_fcs_data n5x_fcs_data = {
	.have_hwrng	= true,
};

static const struct socfpga_fcs_data s10_fcs_data = {
	.have_hwrng	= true,
};

static const struct of_device_id fcs_of_match[] = {
	{.compatible = "intel,stratix10-soc-fcs",
	 .data = &s10_fcs_data
	},
	{.compatible = "intel,agilex-soc-fcs",
	 .data = &agilex_fcs_data
	},
	{.compatible = "intel,n5x-soc-fcs",
	 .data = &n5x_fcs_data
	},
	{},
};

static struct platform_driver fcs_driver = {
	.probe = fcs_driver_probe,
	.remove = fcs_driver_remove,
	.driver = {
		.name = "intel-fcs",
		.of_match_table = of_match_ptr(fcs_of_match),
	},
};

MODULE_DEVICE_TABLE(of, fcs_of_match);

static int __init fcs_init(void)
{
	struct device_node *fw_np;
	struct device_node *np;
	int ret;

	fw_np = of_find_node_by_name(NULL, "svc");
	if (!fw_np)
		return -ENODEV;

	of_node_get(fw_np);
	np = of_find_matching_node(fw_np, fcs_of_match);
	if (!np) {
		of_node_put(fw_np);
		return -ENODEV;
	}

	of_node_put(np);
	ret = of_platform_populate(fw_np, fcs_of_match, NULL, NULL);
	of_node_put(fw_np);
	if (ret)
		return ret;

	return platform_driver_register(&fcs_driver);
}

static void __exit fcs_exit(void)
{
	return platform_driver_unregister(&fcs_driver);
}

module_init(fcs_init);
module_exit(fcs_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel FGPA Crypto Services Driver");
MODULE_AUTHOR("Richard Gong <richard.gong@intel.com>");
