/*
 * (C) COPYRIGHT 2020 ARM Limited. All rights reserved.
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

	dev_info(net->edev->dev, "Network destroy. handle=0x%pK\n", net);

	ethosu_buffer_put(net->buf);
	devm_kfree(net->edev->dev, net);
}

static int ethosu_network_release(struct inode *inode,
				  struct file *file)
{
	struct ethosu_network *net = file->private_data;

	dev_info(net->edev->dev, "Network release. handle=0x%pK\n", net);

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

	dev_info(net->edev->dev, "Ioctl: cmd=%u, arg=%lu\n", cmd, arg);

	switch (cmd) {
	case ETHOSU_IOCTL_INFERENCE_CREATE: {
		struct ethosu_uapi_inference_create uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_info(net->edev->dev,
			 "Ioctl: Inference. ifm_fd=%u, ofm_fd=%u\n",
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
	struct ethosu_buffer *buf;
	struct ethosu_network *net;
	int ret = -ENOMEM;

	buf = ethosu_buffer_get_from_fd(uapi->fd);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	net = devm_kzalloc(edev->dev, sizeof(*net), GFP_KERNEL);
	if (!net) {
		ret = -ENOMEM;
		goto put_buf;
	}

	net->edev = edev;
	net->buf = buf;
	kref_init(&net->kref);

	ret = anon_inode_getfd("ethosu-network", &ethosu_network_fops, net,
			       O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto free_net;

	net->file = fget(ret);
	fput(net->file);

	dev_info(edev->dev, "Network create. handle=0x%pK",
		 net);

	return ret;

free_net:
	devm_kfree(edev->dev, net);

put_buf:
	ethosu_buffer_put(buf);

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

void ethosu_network_put(struct ethosu_network *net)
{
	kref_put(&net->kref, ethosu_network_destroy);
}
