// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Arm Limited.
 */

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_network_info.h"

#include "ethosu_device.h"
#include "ethosu_network.h"
#include "ethosu_rpmsg.h"
#include "uapi/ethosu.h"

#define NETWORK_INFO_RESP_TIMEOUT_MS 30000

static inline int ethosu_network_info_send(struct ethosu_network_info *info)
{
	/* Send network info request to firmware */
	return ethosu_rpmsg_network_info_request(&info->edev->erp,
						 &info->msg,
						 info->net->buf,
						 info->net->index);
}

static void ethosu_network_info_fail(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_network_info *info =
		container_of(msg, typeof(*info), msg);

	if (completion_done(&info->done))
		return;

	info->errno = -EFAULT;
	complete(&info->done);
}

static int ethosu_network_info_resend(struct ethosu_rpmsg_msg *msg)
{
	struct ethosu_network_info *info =
		container_of(msg, typeof(*info), msg);
	int ret;

	/* Don't resend request if response has already been received */
	if (completion_done(&info->done))
		return 0;

	/* Resend request */
	ret = ethosu_network_info_send(info);
	if (ret)
		return ret;

	return 0;
}

int ethosu_network_info_request(struct ethosu_network *net,
				struct ethosu_uapi_network_info *uapi)
{
	struct ethosu_network_info *info;
	int ret;
	int timeout;

	info = devm_kzalloc(net->edev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->edev = net->edev;
	info->net = net;
	info->uapi = uapi;
	init_completion(&info->done);
	info->msg.fail = ethosu_network_info_fail;
	info->msg.resend = ethosu_network_info_resend;

	ret = ethosu_rpmsg_register(&info->edev->erp, &info->msg);
	if (ret < 0)
		goto kfree;

	/* Get reference to network */
	ethosu_network_get(info->net);

	ret = ethosu_network_info_send(info);
	if (ret)
		goto deregister;

	dev_dbg(info->edev->dev,
		"Network info create. info=0x%pK, net=0x%pK, msg.id=0x%x\n",
		info, info->net, info->msg.id);

	/* Unlock the device mutex and wait for completion */
	mutex_unlock(&info->edev->mutex);
	timeout = wait_for_completion_timeout(&info->done,
					      msecs_to_jiffies(NETWORK_INFO_RESP_TIMEOUT_MS));
	mutex_lock(&info->edev->mutex);

	if (timeout == 0) {
		dev_warn(info->edev->dev, "Network info timed out. info=0x%pK",
			 info);

		ret = -ETIME;
		goto deregister;
	}

	ret = info->errno;

deregister:
	ethosu_rpmsg_deregister(&info->edev->erp, &info->msg);
	ethosu_network_put(info->net);

kfree:
	dev_dbg(info->edev->dev,
		"Network info destroy. info=0x%pK, msg.id=0x%x\n",
		info, info->msg.id);
	devm_kfree(info->edev->dev, info);

	return ret;
}

void ethosu_network_info_rsp(struct ethosu_device *edev,
			     struct ethosu_core_network_info_rsp *rsp)
{
	int ret;
	int id = (int)rsp->user_arg;
	struct ethosu_rpmsg_msg *msg;
	struct ethosu_network_info *info;
	u32 i, j;

	msg = ethosu_rpmsg_find(&edev->erp, id);
	if (IS_ERR(msg)) {
		dev_warn(edev->dev,
			 "Id for network info msg not found. msg.id=0x%x\n",
			 id);

		return;
	}

	info = container_of(msg, typeof(*info), msg);

	if (completion_done(&info->done))
		return;

	info->errno = 0;

	if (rsp->status != ETHOSU_CORE_STATUS_OK) {
		info->errno = -EBADF;
		goto signal_complete;
	}

	if (rsp->ifm_count > ETHOSU_CORE_BUFFER_MAX || rsp->ofm_count > ETHOSU_CORE_BUFFER_MAX) {
		info->errno = -ENFILE;
		goto signal_complete;
	}

	ret = strscpy(info->uapi->desc, rsp->desc, sizeof(info->uapi->desc));
	if (ret < 0) {
		info->errno = ret;
		goto signal_complete;
	}

	info->uapi->is_vela = rsp->is_vela;
	info->uapi->ifm_count = rsp->ifm_count;
	for (i = 0; i < rsp->ifm_count; i++) {
		info->uapi->ifm_size[i] = rsp->ifm_size[i];
		info->uapi->ifm_types[i] = rsp->ifm_types[i];
		info->uapi->ifm_offset[i] = rsp->ifm_offset[i];
		info->uapi->ifm_dims[i] = rsp->ifm_dims[i];
		for (j = 0; j < rsp->ifm_dims[i]; j++)
			info->uapi->ifm_shapes[i][j] = rsp->ifm_shapes[i][j];
        }

	info->uapi->ofm_count = rsp->ofm_count;
	for (i = 0; i < rsp->ofm_count; i++) {
		info->uapi->ofm_size[i] = rsp->ofm_size[i];
		info->uapi->ofm_types[i] = rsp->ofm_types[i];
		info->uapi->ofm_offset[i] = rsp->ofm_offset[i];
		info->uapi->ofm_dims[i] = rsp->ofm_dims[i];
		for (j = 0; j < rsp->ofm_dims[i]; j++)
			info->uapi->ofm_shapes[i][j] = rsp->ofm_shapes[i][j];
	}

signal_complete:
	complete(&info->done);
}
