// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Arm Limited.
 */

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_cancel_inference.h"

#include "ethosu_core_interface.h"
#include "ethosu_device.h"
#include "ethosu_inference.h"

#include <linux/wait.h>

/****************************************************************************
 * Defines
 ****************************************************************************/

#define CANCEL_INFERENCE_RESP_TIMEOUT_MS 2000

/****************************************************************************
 * Functions
 ****************************************************************************/

static int ethosu_cancel_inference_send(struct ethosu_cancel_inference *cancellation)
{
	return ethosu_rpmsg_cancel_inference(&cancellation->edev->erp,
					     &cancellation->msg,
					     cancellation->inf->msg.id);
}

static void ethosu_cancel_inference_fail(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_cancel_inference *cancellation =
		container_of(msg, typeof(*cancellation), msg);

	if (completion_done(&cancellation->done))
		return;

	cancellation->errno = -EFAULT;
	cancellation->uapi->status = ETHOSU_UAPI_STATUS_ERROR;
	complete(&cancellation->done);
}

static int ethosu_cancel_inference_complete(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_cancel_inference *cancellation =
		container_of(msg, typeof(*cancellation), msg);

	if (completion_done(&cancellation->done))
		return 0;

	cancellation->errno = 0;
	cancellation->uapi->status =
		cancellation->inf->done &&
		cancellation->inf->status != ETHOSU_UAPI_STATUS_OK ?
		ETHOSU_UAPI_STATUS_OK :
		ETHOSU_UAPI_STATUS_ERROR;
	complete(&cancellation->done);

	return 0;
}

int ethosu_cancel_inference_request(struct ethosu_inference *inf,
				    struct ethosu_uapi_cancel_inference_status *uapi)
{
	struct ethosu_cancel_inference *cancellation;
	int ret;
	int timeout;

	if (inf->done) {
		uapi->status = ETHOSU_UAPI_STATUS_ERROR;

		return 0;
	}

	cancellation =
		devm_kzalloc(inf->edev->dev,
			     sizeof(struct ethosu_cancel_inference),
			     GFP_KERNEL);
	if (!cancellation)
		return -ENOMEM;

	/* increase ref count on the inference we are referring to */
	ethosu_inference_get(inf);
	/* mark inference ABORTING to avoid resending the inference message */
	inf->status = ETHOSU_UAPI_STATUS_ABORTING;

	cancellation->edev = inf->edev;
	cancellation->inf = inf;
	cancellation->uapi = uapi;
	init_completion(&cancellation->done);
	cancellation->msg.fail = ethosu_cancel_inference_fail;

	/* Never resend messages but always complete, since we have restart the
	 * whole firmware and marked the inference as aborted
	 */
	cancellation->msg.resend = ethosu_cancel_inference_complete;

	ret = ethosu_rpmsg_register(&cancellation->edev->erp,
				    &cancellation->msg);
	if (ret < 0)
		goto kfree;

	dev_dbg(cancellation->edev->dev,
		"Inference cancellation create. cancel=0x%pK, msg.id=%d\n",
		cancellation, cancellation->msg.id);

	ret = ethosu_cancel_inference_send(cancellation);
	if (ret != 0)
		goto deregister;

	/* Unlock the mutex before going to block on the condition */
	mutex_unlock(&cancellation->edev->mutex);
	/* wait for response to arrive back */
	timeout = wait_for_completion_timeout(&cancellation->done,
					      msecs_to_jiffies(CANCEL_INFERENCE_RESP_TIMEOUT_MS));

	/* take back the mutex before resuming to do anything */
	ret = mutex_lock_interruptible(&cancellation->edev->mutex);
	if (ret != 0)
		goto deregister;

	if (timeout == 0) {
		dev_warn(inf->edev->dev,
			 "Msg: Cancel Inference response lost - timeout\n");
		ret = -EIO;
		goto deregister;
	}

	if (cancellation->errno) {
		ret = cancellation->errno;
		goto deregister;
	}

deregister:
	ethosu_rpmsg_deregister(&cancellation->edev->erp,
				&cancellation->msg);

kfree:
	dev_dbg(cancellation->edev->dev,
		"Cancel inference destroy. cancel=0x%pK\n", cancellation);
	/* decrease the reference on the inference we are referring to */
	ethosu_inference_put(cancellation->inf);
	devm_kfree(cancellation->edev->dev, cancellation);

	return ret;
}

void ethosu_cancel_inference_rsp(struct ethosu_device *edev,
				 struct ethosu_core_cancel_inference_rsp *rsp)
{
	int id = (int)rsp->user_arg;
	struct ethosu_rpmsg_msg *msg;
	struct ethosu_cancel_inference *cancellation;

	msg = ethosu_rpmsg_find(&edev->erp, id);
	if (IS_ERR(msg)) {
		dev_warn(edev->dev,
			 "Handle not found in cancel inference list. handle=0x%p\n",
			 rsp);

		return;
	}

	cancellation = container_of(msg, typeof(*cancellation), msg);

	if (completion_done(&cancellation->done))
		return;

	cancellation->errno = 0;
	switch (rsp->status) {
	case ETHOSU_CORE_STATUS_OK:
		cancellation->uapi->status = ETHOSU_UAPI_STATUS_OK;
		break;
	case ETHOSU_CORE_STATUS_ERROR:
		cancellation->uapi->status = ETHOSU_UAPI_STATUS_ERROR;
		break;
	}

	complete(&cancellation->done);
}
