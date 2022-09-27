/*
 * Copyright (c) 2020,2022 Arm Limited.
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

#include "ethosu_network.h"

#include "ethosu_buffer.h"
#include "ethosu_device.h"
#include "ethosu_inference.h"
#include "ethosu_network_info.h"
#include "uapi/ethosu.h"

#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/****************************************************************************
 * Variables
 ****************************************************************************/

static int ethosu_network_release(struct inode *inode,
				  struct file *file);

static long ethosu_network_ioctl(struct file *file,
				 unsigned int cmd,
				 unsigned long arg);

static const struct file_operations ethosu_network_fops = {
	.release        = &ethosu_network_release,
	.unlocked_ioctl = &ethosu_network_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = &ethosu_network_ioctl,
#endif
};

/****************************************************************************
 * Functions
 ****************************************************************************/

static bool ethosu_network_verify(struct file *file)
{
	return file->f_op == &ethosu_network_fops;
}

static void ethosu_network_destroy(struct kref *kref)
{
	struct ethosu_network *net =
		container_of(kref, struct ethosu_network, kref);

	dev_dbg(net->edev->dev, "Network destroy. net=0x%pK\n", net);

	if (net->buf)
		ethosu_buffer_put(net->buf);

	devm_kfree(net->edev->dev, net);
}

static int ethosu_network_release(struct inode *inode,
				  struct file *file)
{
	struct ethosu_network *net = file->private_data;

	dev_dbg(net->edev->dev, "Network release. file=0x%pK, net=0x%pK\n",
		file, net);

	ethosu_network_put(net);

	return 0;
}

static long ethosu_network_ioctl(struct file *file,
				 unsigned int cmd,
				 unsigned long arg)
{
	struct ethosu_network *net = file->private_data;
	void __user *udata = (void __user *)arg;
	int ret = -EINVAL;

	ret = mutex_lock_interruptible(&net->edev->mutex);
	if (ret)
		return ret;

	dev_dbg(net->edev->dev,
		"Network ioctl: file=0x%pK, net=0x%pK, cmd=0x%x, arg=0x%lx\n",
		file, net, cmd, arg);

	switch (cmd) {
	case ETHOSU_IOCTL_NETWORK_INFO: {
		struct ethosu_uapi_network_info uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(net->edev->dev,
			 "Network ioctl: Network info. net=0x%pK\n",
			 net);

		ret = ethosu_network_info_request(net, &uapi);
		if (ret)
			break;

		ret = copy_to_user(udata, &uapi, sizeof(uapi)) ? -EFAULT : 0;
		break;
	}
	case ETHOSU_IOCTL_INFERENCE_CREATE: {
		struct ethosu_uapi_inference_create uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(net->edev->dev,
			"Network ioctl: Inference. ifm_fd=%u, ofm_fd=%u\n",
			uapi.ifm_fd[0], uapi.ofm_fd[0]);

		ret = ethosu_inference_create(net->edev, net, &uapi);
		break;
	}
	default: {
		dev_err(net->edev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	}

	mutex_unlock(&net->edev->mutex);

	return ret;
}

int ethosu_network_create(struct ethosu_device *edev,
			  struct ethosu_uapi_network_create *uapi)
{
	struct ethosu_network *net;
	int ret = -ENOMEM;

	net = devm_kzalloc(edev->dev, sizeof(*net), GFP_KERNEL);
	if (!net)
		return -ENOMEM;

	net->edev = edev;
	net->buf = NULL;
	kref_init(&net->kref);

	if (uapi->type == ETHOSU_UAPI_NETWORK_BUFFER) {
		net->buf = ethosu_buffer_get_from_fd(uapi->fd);
		if (IS_ERR(net->buf)) {
			ret = PTR_ERR(net->buf);
			goto free_net;
		}
	} else {
		net->index = uapi->index;
	}

	ret = anon_inode_getfd("ethosu-network", &ethosu_network_fops, net,
			       O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto put_buf;

	net->file = fget(ret);
	fput(net->file);

	dev_dbg(edev->dev,
		"Network create. file=0x%pK, fd=%d, net=0x%pK, buf=0x%pK, index=%u",
		net->file, ret, net, net->buf, net->index);

	return ret;

put_buf:
	if (net->buf)
		ethosu_buffer_put(net->buf);

free_net:
	devm_kfree(edev->dev, net);

	return ret;
}

struct ethosu_network *ethosu_network_get_from_fd(int fd)
{
	struct ethosu_network *net;
	struct file *file;

	file = fget(fd);
	if (!file)
		return ERR_PTR(-EINVAL);

	if (!ethosu_network_verify(file)) {
		fput(file);

		return ERR_PTR(-EINVAL);
	}

	net = file->private_data;
	ethosu_network_get(net);
	fput(file);

	return net;
}

void ethosu_network_get(struct ethosu_network *net)
{
	kref_get(&net->kref);
}

int ethosu_network_put(struct ethosu_network *net)
{
	return kref_put(&net->kref, ethosu_network_destroy);
}
