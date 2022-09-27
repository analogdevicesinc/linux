// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Arm Limited.
 */

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_capabilities.h"

#include "ethosu_device.h"

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>

/****************************************************************************
 * Defines
 ****************************************************************************/

#define CAPABILITIES_RESP_TIMEOUT_MS 2000

/****************************************************************************
 * Functions
 ****************************************************************************/

static inline int ethosu_capabilities_send(struct ethosu_capabilities *cap)
{
	return ethosu_rpmsg_capabilities_request(&cap->edev->erp,
						 &cap->msg);
}

static void ethosu_capabilities_fail(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_capabilities *cap =
		container_of(msg, typeof(*cap), msg);

	if (completion_done(&cap->done))
		return;

	cap->errno = -EFAULT;
	complete(&cap->done);
}

static int ethosu_capabilities_resend(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_capabilities *cap =
		container_of(msg, typeof(*cap), msg);

	/* Don't resend request if response has already been received */
	if (completion_done(&cap->done))
		return 0;

	/* Resend request */
	return ethosu_capabilities_send(cap);
}

void ethosu_capability_rsp(struct ethosu_device *edev,
			   struct ethosu_core_msg_capabilities_rsp *rsp)
{
	int id = (int)rsp->user_arg;
	struct ethosu_rpmsg_msg *msg;
	struct ethosu_capabilities *cap;

	msg = ethosu_rpmsg_find(&edev->erp, id);
	if (IS_ERR(msg)) {
		dev_warn(edev->dev,
			 "Id for capabilities msg not found. id=%d\n",
			 id);

		return;
	}

	cap = container_of(msg, typeof(*cap), msg);

	if (completion_done(&cap->done))
		return;

	cap->uapi->hw_id.version_status = rsp->version_status;
	cap->uapi->hw_id.version_minor = rsp->version_minor;
	cap->uapi->hw_id.version_major = rsp->version_major;
	cap->uapi->hw_id.product_major = rsp->product_major;
	cap->uapi->hw_id.arch_patch_rev = rsp->arch_patch_rev;
	cap->uapi->hw_id.arch_minor_rev = rsp->arch_minor_rev;
	cap->uapi->hw_id.arch_major_rev = rsp->arch_major_rev;
	cap->uapi->driver_patch_rev = rsp->driver_patch_rev;
	cap->uapi->driver_minor_rev = rsp->driver_minor_rev;
	cap->uapi->driver_major_rev = rsp->driver_major_rev;
	cap->uapi->hw_cfg.macs_per_cc = rsp->macs_per_cc;
	cap->uapi->hw_cfg.cmd_stream_version = rsp->cmd_stream_version;
	cap->uapi->hw_cfg.custom_dma = rsp->custom_dma;

	cap->errno = 0;
	complete(&cap->done);
}

int ethosu_capabilities_request(struct ethosu_device *edev,
				struct ethosu_uapi_device_capabilities *uapi)
{
	struct ethosu_capabilities *cap;
	int ret;
	int timeout;

	cap = devm_kzalloc(edev->dev, sizeof(struct ethosu_capabilities),
			   GFP_KERNEL);
	if (!cap)
		return -ENOMEM;

	cap->edev = edev;
	cap->uapi = uapi;
	init_completion(&cap->done);
	cap->msg.fail = ethosu_capabilities_fail;
	cap->msg.resend = ethosu_capabilities_resend;

	ret = ethosu_rpmsg_register(&cap->edev->erp, &cap->msg);
	if (ret < 0)
		goto kfree;

	dev_dbg(edev->dev, "Capabilities create. Id=%d, handle=0x%p\n",
		cap->msg.id, cap);

	ret = ethosu_capabilities_send(cap);
	if (ret != 0)
		goto deregister;

	/* Unlock the mutex before going to block on the condition */
	mutex_unlock(&edev->mutex);

	/* wait for response to arrive back */
	timeout = wait_for_completion_timeout(&cap->done,
					      msecs_to_jiffies(CAPABILITIES_RESP_TIMEOUT_MS));

	/* take back the mutex before resuming to do anything */
	mutex_lock(&edev->mutex);

	if (timeout == 0) {
		dev_warn(edev->dev, "Capabilities response timeout");
		ret = -ETIME;
		goto deregister;
	}

	if (cap->errno) {
		ret = cap->errno;
		goto deregister;
	}

deregister:
	ethosu_rpmsg_deregister(&cap->edev->erp, &cap->msg);

kfree:
	dev_dbg(cap->edev->dev, "Capabilities destroy. Id=%d, handle=0x%p\n",
		cap->msg.id, cap);
	devm_kfree(cap->edev->dev, cap);

	return ret;
}
