// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

/****************************************************************************/

#include <linux/anon_inodes.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "neutron_buffer.h"
#include "neutron_device.h"
#include "uapi/neutron.h"

/****************************************************************************/

static int neutron_buffer_release(struct inode *inode,
				  struct file *file);

static int neutron_buffer_mmap(struct file *file,
			       struct vm_area_struct *vma);

static const struct file_operations neutron_buffer_fops = {
	.release        = &neutron_buffer_release,
	.mmap           = &neutron_buffer_mmap,
};

/****************************************************************************/

static void neutron_buffer_destroy(struct kref *kref)
{
	struct neutron_buffer *buf =
		container_of(kref, struct neutron_buffer, kref);

	dev_dbg(buf->ndev->dev, "Buffer destroy. buf=0x%pS\n", buf);

	dma_free_coherent(buf->ndev->dev, buf->size, buf->cpu_addr,
			  buf->dma_addr);
	devm_kfree(buf->ndev->dev, buf);
}

static int neutron_buffer_release(struct inode *inode,
				  struct file *file)
{
	struct neutron_buffer *buf = file->private_data;

	dev_dbg(buf->ndev->dev, "Buffer release. file=0x%pS, buf=0x%pS\n",
		file, buf);

	neutron_buffer_put(buf);

	return 0;
}

static int neutron_buffer_mmap(struct file *file,
			       struct vm_area_struct *vma)
{
	struct neutron_buffer *buf = file->private_data;
	int ret;

	dev_dbg(buf->ndev->dev, "Buffer mmap. file=0x%pS, buf=0x%pS\n",
		file, buf);

	ret = dma_mmap_coherent(buf->ndev->dev, vma, buf->cpu_addr,
				buf->dma_addr, buf->size);

	return ret;
}

int neutron_buffer_create(struct neutron_device *ndev,
			  size_t size, __u64 *addr_out)
{
	struct neutron_buffer *buf;
	int ret = -ENOMEM;

	if (!size)
		return -EINVAL;

	buf = devm_kzalloc(ndev->dev, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->ndev = ndev;
	buf->size = size;
	kref_init(&buf->kref);

	buf->cpu_addr = dma_alloc_coherent(buf->ndev->dev, size,
					   &buf->dma_addr, GFP_KERNEL);
	if (!buf->cpu_addr)
		goto free_buf;

	ret = anon_inode_getfd("neutron-buffer", &neutron_buffer_fops, buf,
			       O_RDWR | O_CLOEXEC);
	if (ret < 0)
		goto free_dma;

	buf->file = fget(ret);
	fput(buf->file);
	*addr_out = buf->dma_addr;

	dev_dbg(buf->ndev->dev,
		"Buffer create. fd=%d, size=%zu, cpu_addr=0x%pK, dma_addr=0x%llx\n",
		ret, size, buf->cpu_addr, buf->dma_addr);

	return ret;

free_dma:
	dma_free_coherent(buf->ndev->dev, buf->size, buf->cpu_addr,
			  buf->dma_addr);

free_buf:
	devm_kfree(buf->ndev->dev, buf);

	return ret;
}

void neutron_buffer_get(struct neutron_buffer *buf)
{
	kref_get(&buf->kref);
}

void neutron_buffer_put(struct neutron_buffer *buf)
{
	kref_put(&buf->kref, neutron_buffer_destroy);
}
