/*
 * Copyright (c) 2020-2022 Arm Limited.
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

#include "ethosu_buffer.h"

#include "ethosu_device.h"
#include "uapi/ethosu.h"

#include <linux/anon_inodes.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

/****************************************************************************
 * Variables
 ****************************************************************************/

static int ethosu_buffer_release(struct inode *inode,
				 struct file *file);

static int ethosu_buffer_mmap(struct file *file,
			      struct vm_area_struct *vma);

static long ethosu_buffer_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg);

static const struct file_operations ethosu_buffer_fops = {
	.release        = &ethosu_buffer_release,
	.mmap           = &ethosu_buffer_mmap,
	.unlocked_ioctl = &ethosu_buffer_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = &ethosu_buffer_ioctl,
#endif
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/*
 * The 'dma-ranges' device tree property for shared dma memory does not seem
 * to be fully supported for coherent memory. Therefor we apply the DMA range
 * offset ourselves.
 */
static dma_addr_t ethosu_buffer_dma_ranges(struct device *dev,
					   dma_addr_t dma_addr,
					   size_t dma_buf_size)
{
	struct device_node *node = dev->of_node;
	const __be32 *ranges;
	int len;
	int naddr;
	int nsize;
	int inc;
	int i;

	if (!node)
		return dma_addr;

	/* Get the #address-cells and #size-cells properties */
	naddr = of_n_addr_cells(node);
	nsize = of_n_size_cells(node);

	/* Read the 'dma-ranges' property */
	ranges = of_get_property(node, "dma-ranges", &len);
	if (!ranges || len <= 0)
		return dma_addr;

	dev_dbg(dev, "ranges=%p, len=%d, naddr=%d, nsize=%d\n",
		ranges, len, naddr, nsize);

	len /= sizeof(*ranges);
	inc = naddr + naddr + nsize;

	for (i = 0; (i + inc) <= len; i += inc) {
		dma_addr_t daddr;
		dma_addr_t paddr;
		dma_addr_t size;

		daddr = of_read_number(&ranges[i], naddr);
		paddr = of_read_number(&ranges[i + naddr], naddr);
		size = of_read_number(&ranges[i + naddr + naddr], nsize);

		dev_dbg(dev, "daddr=0x%llx, paddr=0x%llx, size=0x%llx\n",
			daddr, paddr, size);

		if (dma_addr >= paddr &&
		    (dma_addr + dma_buf_size) < (paddr + size))
			return dma_addr + daddr - paddr;
	}

	return dma_addr;
}

static bool ethosu_buffer_verify(struct file *file)
{
	return file->f_op == &ethosu_buffer_fops;
}

static void ethosu_buffer_destroy(struct kref *kref)
{
	struct ethosu_buffer *buf =
		container_of(kref, struct ethosu_buffer, kref);

	dev_dbg(buf->edev->dev, "Buffer destroy. buf=0x%pK\n", buf);

	dma_free_coherent(buf->edev->dev, buf->capacity, buf->cpu_addr,
			  buf->dma_addr_orig);
	devm_kfree(buf->edev->dev, buf);
}

static int ethosu_buffer_release(struct inode *inode,
				 struct file *file)
{
	struct ethosu_buffer *buf = file->private_data;

	dev_dbg(buf->edev->dev, "Buffer release. file=0x%pK, buf=0x%pK\n",
		file, buf);

	ethosu_buffer_put(buf);

	return 0;
}

static int ethosu_buffer_mmap(struct file *file,
			      struct vm_area_struct *vma)
{
	struct ethosu_buffer *buf = file->private_data;
	int ret;

	dev_dbg(buf->edev->dev, "Buffer mmap. file=0x%pK, buf=0x%pK\n",
		file, buf);

	ret = dma_mmap_coherent(buf->edev->dev, vma, buf->cpu_addr,
				buf->dma_addr_orig,
				buf->capacity);

	return ret;
}

static long ethosu_buffer_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg)
{
	struct ethosu_buffer *buf = file->private_data;
	void __user *udata = (void __user *)arg;
	int ret = -EINVAL;

	ret = mutex_lock_interruptible(&buf->edev->mutex);
	if (ret)
		return ret;

	dev_dbg(buf->edev->dev,
		"Buffer ioctl. file=0x%pK, buf=0x%pK, cmd=0x%x, arg=%lu\n",
		file, buf, cmd, arg);

	switch (cmd) {
	case ETHOSU_IOCTL_BUFFER_SET: {
		struct ethosu_uapi_buffer uapi;

		if (copy_from_user(&uapi, udata, sizeof(uapi)))
			break;

		dev_dbg(buf->edev->dev,
			"Buffer ioctl: Buffer set. size=%u, offset=%u\n",
			uapi.size, uapi.offset);

		ret = ethosu_buffer_resize(buf, uapi.size, uapi.offset);
		break;
	}
	case ETHOSU_IOCTL_BUFFER_GET: {
		struct ethosu_uapi_buffer uapi;

		uapi.size = buf->size;
		uapi.offset = buf->offset;

		dev_dbg(buf->edev->dev,
			"Buffer ioctl: Buffer get. size=%u, offset=%u\n",
			uapi.size, uapi.offset);

		if (copy_to_user(udata, &uapi, sizeof(uapi)))
			break;

		ret = 0;
		break;
	}
	default: {
		dev_err(buf->edev->dev, "Invalid ioctl. cmd=%u, arg=%lu",
			cmd, arg);
		break;
	}
	}

	mutex_unlock(&buf->edev->mutex);

	return ret;
}

int ethosu_buffer_create(struct ethosu_device *edev,
			 size_t capacity)
{
	struct ethosu_buffer *buf;
	int ret = -ENOMEM;

	if (!capacity)
		return -EINVAL;

	buf = devm_kzalloc(edev->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->edev = edev;
	buf->capacity = capacity;
	buf->offset = 0;
	buf->size = 0;
	kref_init(&buf->kref);

	buf->cpu_addr = dma_alloc_coherent(buf->edev->dev, capacity,
					   &buf->dma_addr_orig, GFP_KERNEL);
	if (!buf->cpu_addr)
		goto free_buf;

	buf->dma_addr = ethosu_buffer_dma_ranges(buf->edev->dev,
						 buf->dma_addr_orig,
						 buf->capacity);

	ret = anon_inode_getfd("ethosu-buffer", &ethosu_buffer_fops, buf,
			       O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto free_dma;

	buf->file = fget(ret);
	fput(buf->file);

	dev_dbg(buf->edev->dev,
		"Buffer create. file=0x%pK, fd=%d, buf=0x%pK, capacity=%zu, cpu_addr=0x%pK, dma_addr=0x%llx, dma_addr_orig=0x%llx, phys_addr=0x%llx\n",
		buf->file, ret, buf, capacity, buf->cpu_addr, buf->dma_addr,
		buf->dma_addr_orig, virt_to_phys(buf->cpu_addr));

	return ret;

free_dma:
	dma_free_coherent(buf->edev->dev, buf->capacity, buf->cpu_addr,
			  buf->dma_addr_orig);

free_buf:
	devm_kfree(buf->edev->dev, buf);

	return ret;
}

struct ethosu_buffer *ethosu_buffer_get_from_fd(int fd)
{
	struct ethosu_buffer *buf;
	struct file *file;

	file = fget(fd);
	if (!file)
		return ERR_PTR(-EINVAL);

	if (!ethosu_buffer_verify(file)) {
		fput(file);

		return ERR_PTR(-EINVAL);
	}

	buf = file->private_data;
	ethosu_buffer_get(buf);
	fput(file);

	return buf;
}

void ethosu_buffer_get(struct ethosu_buffer *buf)
{
	kref_get(&buf->kref);
}

void ethosu_buffer_put(struct ethosu_buffer *buf)
{
	kref_put(&buf->kref, ethosu_buffer_destroy);
}

int ethosu_buffer_resize(struct ethosu_buffer *buf,
			 size_t size,
			 size_t offset)
{
	if ((size + offset) > buf->capacity)
		return -EINVAL;

	buf->size = size;
	buf->offset = offset;

	return 0;
}
