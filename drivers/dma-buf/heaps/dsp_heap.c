// SPDX-License-Identifier: GPL-2.0
/*
 * DMABUF dsp heap exporter
 *
 * Copyright 2021 NXP.
 *
 */

#include <linux/genalloc.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of.h>
#include <linux/of_fdt.h>

struct dsp_heap_buffer {
	struct dma_heap *heap;
	struct list_head attachments;
	struct mutex lock;  /* mutex lock */
};

struct dsp_heap {
	struct dma_heap *heap;
	phys_addr_t base;
	phys_addr_t size;
};

static int dsp_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct dsp_heap_buffer *buffer = dmabuf->priv;
	struct dsp_heap *dsp_heap = dma_heap_get_drvdata(buffer->heap);
	unsigned long pfn;
	size_t size;
	int ret;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	size = dsp_heap->size;
	pfn =  dsp_heap->base >> PAGE_SHIFT;

	ret = remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot);
	if (ret < 0)
		return ret;

	return 0;
}

static void dsp_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct dsp_heap_buffer *buffer = dmabuf->priv;

	kfree(buffer);
}

static struct sg_table *dsp_heap_map_dma_buf(struct dma_buf_attachment *attachment,
					     enum dma_data_direction direction)
{
	return NULL;
}

static void dsp_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
				   struct sg_table *table,
				   enum dma_data_direction direction)
{
}

static const struct dma_buf_ops dsp_heap_buf_ops = {
	.mmap = dsp_heap_mmap,
	.map_dma_buf = dsp_heap_map_dma_buf,
	.unmap_dma_buf = dsp_heap_unmap_dma_buf,
	.release = dsp_heap_dma_buf_release,
};

static struct dma_buf * dsp_heap_allocate(struct dma_heap *heap,
					  unsigned long len,
					  u32 fd_flags,
					  u64 heap_flags)
{
	struct dsp_heap *dsp_heap = dma_heap_get_drvdata(heap);
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dsp_heap_buffer *buffer;
	struct dma_buf *dmabuf;

	if (len > dsp_heap->size)
		return ERR_PTR(-ENOMEM);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->heap = heap;

	/* create the dmabuf */
	exp_info.ops = &dsp_heap_buf_ops;
	exp_info.size = len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		kfree(buffer);
		return dmabuf;
	}

	return dmabuf;
}

static const struct dma_heap_ops dsp_heap_ops = {
	.allocate = dsp_heap_allocate,
};

static int dsp_heap_create(void)
{
	struct dma_heap_export_info exp_info;
	struct dsp_heap *dsp_heap;
	struct reserved_mem *rmem;
	struct device_node *np;

	np = of_find_node_by_path("/reserved-memory");
	if (!np)
		return 0;

	np = of_find_node_by_name(np, "dsp_reserved_heap");
	if (!np)
		return 0;

	rmem = of_reserved_mem_lookup(np);
	if (!rmem) {
		pr_err("of_reserved_mem_lookup() returned NULL\n");
		return 0;
	}

	if (rmem->base == 0 || rmem->size == 0) {
		pr_err("dsp_data base or size is not correct\n");
		return -EINVAL;
	}

	dsp_heap = kzalloc(sizeof(*dsp_heap), GFP_KERNEL);
	if (!dsp_heap)
		return -ENOMEM;

	dsp_heap->base = rmem->base;
	dsp_heap->size = rmem->size;

	exp_info.name = "dsp";
	exp_info.ops = &dsp_heap_ops;
	exp_info.priv = dsp_heap;
	dsp_heap->heap = dma_heap_add(&exp_info);
	if (IS_ERR(dsp_heap->heap)) {
		int ret = PTR_ERR(dsp_heap->heap);

		kfree(dsp_heap);
		return ret;
	}

	return 0;
}
module_init(dsp_heap_create);
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(DMA_BUF);
