/*
 * Copyright (c) 2020-2022 Arm Limited.
 * Copyright 2020-2022 NXP
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_device.h"

#include "ethosu_buffer.h"
#include "ethosu_core_interface.h"
#include "ethosu_capabilities.h"
#include "ethosu_inference.h"
#include "ethosu_network.h"
#include "ethosu_network_info.h"
#include "uapi/ethosu.h"

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_reserved_mem.h>
#include <linux/remoteproc.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

/****************************************************************************
 * Defines
 ****************************************************************************/

#define DMA_ADDR_BITS 32 /* Number of address bits */

#define ETHOSU_FIRMWARE_NAME "ethosu_firmware"

/****************************************************************************
 * Types
 ****************************************************************************/

/****************************************************************************
 * Functions
 ****************************************************************************/

/* Incoming messages */
static int ethosu_handle_msg(struct ethosu_device *edev, void *data)
{
	int ret = 0;
	struct ethosu_core_msg *header = (struct ethosu_core_msg *)data;
	struct ethosu_core_msg_err *error =
			(struct ethosu_core_msg_err *)
			((char *)data + sizeof(struct ethosu_core_msg));
	struct ethosu_core_inference_rsp *rsp =
			(struct ethosu_core_inference_rsp *)
			((char *)data + sizeof(struct ethosu_core_msg));
	struct ethosu_core_msg_version *version =
			(struct ethosu_core_msg_version *)
			((char *)data + sizeof(struct ethosu_core_msg));
	struct ethosu_core_msg_capabilities_rsp *capabilities =
			(struct ethosu_core_msg_capabilities_rsp *)
			((char *)data + sizeof(struct ethosu_core_msg));
	struct ethosu_core_network_info_rsp *network_info =
			(struct ethosu_core_network_info_rsp *)
			((char *)data + sizeof(struct ethosu_core_msg));

	switch (header->type) {
	case ETHOSU_CORE_MSG_ERR:
		if (header->length != sizeof(struct ethosu_core_msg_err)) {
			dev_warn(edev->dev,
				 "Msg: Error message of incorrect size. size=%u, expected=%zu\n", header->length,
				 sizeof(struct ethosu_core_msg_err));
			ret = -EBADMSG;
			break;
		}

		error->msg[sizeof(error->msg) - 1] = '\0';
		dev_warn(edev->dev, "Msg: Error. type=%u, msg=\"%s\"\n",
			 error->type, error->msg);
		ret = -EBADMSG;
		break;
	case ETHOSU_CORE_MSG_PING:
		dev_dbg(edev->dev, "Msg: Ping\n");
		ret = ethosu_rpmsg_pong(&edev->erp);
		break;
	case ETHOSU_CORE_MSG_PONG:
		dev_dbg(edev->dev, "Msg: Pong\n");
		break;
	case ETHOSU_CORE_MSG_POWER_RSP:
		dev_dbg(edev->dev, "Msg: Power response\n");
		break;
	case ETHOSU_CORE_MSG_INFERENCE_RSP:
		if (header->length != sizeof(struct ethosu_core_inference_rsp)) {
			dev_warn(edev->dev,
				 "Msg: Inference response of incorrect size. size=%u, expected=%zu\n", header->length,
				 sizeof(struct ethosu_core_inference_rsp));
			ret = -EBADMSG;
			break;
		}

		dev_dbg(edev->dev,
			"Msg: Inference response. user_arg=0x%llx, ofm_count=%u, status=%u\n",
			rsp->user_arg, rsp->ofm_count,
			rsp->status);
		ethosu_inference_rsp(edev, rsp);
		break;
	case ETHOSU_CORE_MSG_VERSION_RSP:
		if (header->length != sizeof(struct ethosu_core_msg_version)) {
			dev_warn(edev->dev,
				 "Msg: Version response of incorrect size. size=%u, expected=%zu\n", header->length,
				 sizeof(struct ethosu_core_msg_version));
			ret = -EBADMSG;
			break;
		}

		dev_dbg(edev->dev, "Msg: Version response v%u.%u.%u\n",
			version->major, version->minor,
			version->patch);

		/* Check major and minor version match, else return error */
		if (version->major != ETHOSU_CORE_MSG_VERSION_MAJOR ||
		    version->minor != ETHOSU_CORE_MSG_VERSION_MINOR) {
			dev_warn(edev->dev, "Msg: Version mismatch detected! ");
			dev_warn(edev->dev, "Local version: v%u.%u.%u\n",
				 ETHOSU_CORE_MSG_VERSION_MAJOR,
				 ETHOSU_CORE_MSG_VERSION_MINOR,
				 ETHOSU_CORE_MSG_VERSION_PATCH);
		}

		break;
	case ETHOSU_CORE_MSG_CAPABILITIES_RSP:
		if (header->length != sizeof(struct ethosu_core_msg_capabilities_rsp)) {
			dev_warn(edev->dev,
				 "Msg: Capabilities response of incorrect size. size=%u, expected=%zu\n", header->length,
				 sizeof(struct ethosu_core_msg_capabilities_rsp));
			ret = -EBADMSG;
			break;
		}

		dev_dbg(edev->dev,
			"Msg: Capabilities response ua%llx vs%hhu v%hhu.%hhu p%hhu av%hhu.%hhu.%hhu dv%hhu.%hhu.%hhu mcc%hhu csv%hhu cd%hhu\n",
			capabilities->user_arg,
			capabilities->version_status,
			capabilities->version_major,
			capabilities->version_minor,
			capabilities->product_major,
			capabilities->arch_major_rev,
			capabilities->arch_minor_rev,
			capabilities->arch_patch_rev,
			capabilities->driver_major_rev,
			capabilities->driver_minor_rev,
			capabilities->driver_patch_rev,
			capabilities->macs_per_cc,
			capabilities->cmd_stream_version,
			capabilities->custom_dma);

		ethosu_capability_rsp(edev, capabilities);
		break;
	case ETHOSU_CORE_MSG_NETWORK_INFO_RSP:
		if (header->length != sizeof(struct ethosu_core_network_info_rsp)) {
			dev_warn(edev->dev,
				 "Msg: Network info response of incorrect size. size=%u, expected=%zu\n",
				 header->length, sizeof(struct ethosu_core_network_info_rsp));
			ret = -EBADMSG;
			break;
		}

		dev_dbg(edev->dev,
			"Msg: Network info response. user_arg=0x%llx, status=%u",
			network_info->user_arg,
			network_info->status);

		ethosu_network_info_rsp(edev, network_info);
		break;
	default:
		/* This should not happen due to version checks */
		dev_warn(edev->dev, "Msg: Protocol error\n");
		ret = -EPROTO;
		break;
	}

	return ret;
}

static int ethosu_open(struct inode *inode,
		       struct file *file)
{
	struct ethosu_device *edev =
		container_of(inode->i_cdev, struct ethosu_device, cdev);
	phandle rproc_phandle;
	struct rproc *rproc;
	int ret = 0;

	file->private_data = edev;

	dev_dbg(edev->dev, "Device open. file=0x%pK\n", file);

	if (of_property_read_u32(edev->dev->of_node, "fsl,cm33-proc",
				 &rproc_phandle)) {
		dev_err(edev->dev, "could not get rproc phandle\n");
		return -ENODEV;
	}

	rproc = rproc_get_by_phandle(rproc_phandle);
	if (!rproc) {
		dev_err(edev->dev, "could not get rproc handle\n");
		return -EINVAL;
	}

	ret = rproc_set_firmware(rproc, ETHOSU_FIRMWARE_NAME);

	if (!ret && atomic_read(&rproc->power) == 0) {
		init_completion(&edev->erp.rpmsg_ready);
		ret = rproc_boot(rproc);
		if (ret || wait_for_completion_interruptible(&edev->erp.rpmsg_ready))
			dev_err(edev->dev, "could not boot a remote processor\n");
	} else {
		dev_err(edev->dev, "can't change firmware or remote processor is running\n");
	}

	edev->open = true;

	return nonseekable_open(inode, file);
}

static long ethosu_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct ethosu_device *edev = file->private_data;
	void __user *udata = (void __user *)arg;
	int ret = -EINVAL;

	ret = mutex_lock_interruptible(&edev->mutex);
	if (ret)
		return ret;

	dev_dbg(edev->dev, "Device ioctl. file=0x%pK, cmd=0x%x, arg=0x%lx\n",
		file, cmd, arg);

	switch (cmd) {
	case ETHOSU_IOCTL_VERSION_REQ:
		dev_dbg(edev->dev, "Device ioctl: Send version request\n");
		ret = ethosu_rpmsg_version_request(&edev->erp);
		break;
	case ETHOSU_IOCTL_CAPABILITIES_REQ: {
		struct ethosu_uapi_device_capabilities uapi;

		dev_dbg(edev->dev,
			"Device ioctl: Send capabilities request\n");

		ret = ethosu_capabilities_request(edev, &uapi);
		if (ret)
			break;

		ret = copy_to_user(udata, &uapi, sizeof(uapi)) ? -EFAULT : 0;
		break;
	}
	case ETHOSU_IOCTL_PING: {
		dev_dbg(edev->dev, "Device ioctl: Send ping\n");
		ret = ethosu_rpmsg_ping(&edev->erp);
		break;
	}
	case ETHOSU_IOCTL_BUFFER_CREATE: {
		struct ethosu_uapi_buffer_create uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(edev->dev,
			"Device ioctl: Buffer create. capacity=%u\n",
			uapi.capacity);

		ret = ethosu_buffer_create(edev, uapi.capacity);
		break;
	}
	case ETHOSU_IOCTL_NETWORK_CREATE: {
		struct ethosu_uapi_network_create uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(edev->dev,
			"Device ioctl: Network create. type=%u, fd/index=%u\n",
			uapi.type, uapi.fd);

		ret = ethosu_network_create(edev, &uapi);
		break;
	}
	default: {
		dev_err(edev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	}

	mutex_unlock(&edev->mutex);

	return ret;
}

static void ethosu_rpmsg_rx(void *user_arg, void *data)
{
	struct ethosu_device *edev = user_arg;
	int ret;

	mutex_lock(&edev->mutex);

	do {
		ret = ethosu_handle_msg(edev, data);
	} while (ret != 0);

	mutex_unlock(&edev->mutex);
}

int ethosu_dev_init(struct ethosu_device *edev,
		    struct device *dev,
		    struct class *class,
		    dev_t devt)
{
	static const struct file_operations fops = {
		.owner          = THIS_MODULE,
		.open           = &ethosu_open,
		.unlocked_ioctl = &ethosu_ioctl,
#ifdef CONFIG_COMPAT
		.compat_ioctl   = &ethosu_ioctl,
#endif
	};
	struct device *sysdev;
	int ret;

	edev->dev = dev;
	edev->class = class;
	edev->devt = devt;
	mutex_init(&edev->mutex);

	ret = of_reserved_mem_device_init(edev->dev);
	if (ret)
		return ret;

	dma_set_mask_and_coherent(edev->dev, DMA_BIT_MASK(DMA_ADDR_BITS));

	ret = ethosu_rpmsg_init(&edev->erp, ethosu_rpmsg_rx, edev);
	if (ret)
		goto release_reserved_mem;

	cdev_init(&edev->cdev, &fops);
	edev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&edev->cdev, edev->devt, 1);
	if (ret) {
		dev_err(edev->dev, "Failed to add character device.\n");
		goto deinit_rpmsg;
	}

	sysdev = device_create(edev->class, NULL, edev->devt, edev,
			       "ethosu%d", MINOR(edev->devt));
	if (IS_ERR(sysdev)) {
		dev_err(edev->dev, "Failed to create device.\n");
		ret = PTR_ERR(sysdev);
		goto del_cdev;
	}

	dev_dbg(edev->dev,
		"Created Arm Ethos-U device. name=%s, major=%d, minor=%d\n",
		dev_name(sysdev), MAJOR(edev->devt), MINOR(edev->devt));

	return 0;

del_cdev:
	cdev_del(&edev->cdev);

deinit_rpmsg:
	ethosu_rpmsg_deinit(&edev->erp);

release_reserved_mem:
	of_reserved_mem_device_release(edev->dev);

	return ret;
}

void ethosu_dev_deinit(struct ethosu_device *edev)
{
	ethosu_rpmsg_deinit(&edev->erp);
	device_destroy(edev->class, edev->cdev.dev);
	cdev_del(&edev->cdev);
	of_reserved_mem_device_release(edev->dev);
}
