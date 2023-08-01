// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SRAM mmap misc driver for ADI processor on-chip memory
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_reserved_mem.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/genalloc.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#define SRAM_MMAP_DRV_NAME		"sram_mmap"

#define MAX_SRAM_REGIONS 4
struct sram_region_info {
	struct reserved_mem *rmem;
	struct device *dev;
};

static struct sram_region_info sram_regions[MAX_SRAM_REGIONS];
static int next_region = 0;

static struct reserved_mem *find_sram_mem(struct device *dev, int *id) {
	int i;

	if (!dev) {
		pr_err("NULL device to find_sram_mem\n");
		return NULL;
	}

	for (i = 0; i < MAX_SRAM_REGIONS; ++i) {
		if (sram_regions[i].dev == dev) {
			*id = i;
			return sram_regions[i].rmem;
		}
	}

	dev_err(dev, "Unable to find SRAM reservation!\n");
	return NULL;
}

struct adi_sram_mmap {
	struct miscdevice miscdev;
	struct device *dev;
	struct page *start;
	struct reserved_mem *rmem;
};

int sram_set_page_dirty(struct page *page) {
	/* do nothing but avoid using __set_page_dirty_buffers which would
	 * actually mark the page as dirty and cause a warning later */
	return 0;
}

struct address_space_operations sram_aops = {
	.set_page_dirty = sram_set_page_dirty,
};

/**
 * For now ignore pgoff supplied by the user and start mapping at
 * the start of SRAM
 */
static int sram_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct adi_sram_mmap *sram = container_of(fp->private_data,
				struct adi_sram_mmap, miscdev);
	size_t sram_size = vma->vm_end - vma->vm_start;
	struct page *start_page;
	int pages, i;
	int ret = 0;

	if ((vma->vm_pgoff * PAGE_SIZE) + sram_size > sram->rmem->size) {
		dev_err(sram->dev, "Tried to map 0x%zx@0x%zx, only 0x%zx available\n",
			sram_size, vma->vm_pgoff * PAGE_SIZE, (size_t)sram->rmem->size);
		return -ENOMEM;
	}

	if (sram_size % PAGE_SIZE) {
		dev_err(sram->dev, "Requested mapping is not a multiple of page size, requested 0x%lx bytes\n", sram_size);
		return -EINVAL;
	}

	fp->f_mapping->a_ops = &sram_aops;
	vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot, PTE_ATTRINDX_MASK,
		PTE_ATTRINDX(MT_NORMAL) | PTE_PXN | PTE_UXN);
	vma->vm_private_data = sram;
	vma->vm_ops = NULL;

	start_page = sram->start;
	pages = sram_size / PAGE_SIZE;
	for (i = 0; i < pages; ++i) {
		struct page *page = start_page + vma->vm_pgoff + i;

		if (!page->mapping)
			page->mapping = fp->f_mapping;

		ret = vm_insert_page(vma, vma->vm_start + (i*PAGE_SIZE), page);
		if (ret) {
			dev_err(sram->dev, "Failed to map page to userspace\n");
			return ret;
		}
	}

	dev_info(sram->dev, "Mapped 0x%zx : 0x%zx successfully\n",
		(size_t) (sram->rmem->base + vma->vm_pgoff * PAGE_SIZE),
		(size_t) (sram->rmem->base + vma->vm_pgoff * PAGE_SIZE + sram_size));
	return 0;
}

static const struct file_operations sram_fops = {
	.mmap		= sram_mmap,
};

static const struct of_device_id adi_sram_mmap_of_match[] = {
	{ .compatible = "adi,sram-mmap" },
	{ },
};
MODULE_DEVICE_TABLE(of,adi_sram_mmap_of_match);

static int adi_sram_mmap_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct adi_sram_mmap *sram;
	struct device *dev;
	struct page *page;
	struct reserved_mem *mem;
	int pages, i, id;

	dev = &pdev->dev;

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "Unable to configure SRAM reserved memory\n");
		return ret;
	}

	mem = find_sram_mem(dev, &id);
	if (!mem) {
		dev_err(dev, "SRAM MMAP requires adi,sram-access reserved memory, please check your device tree\n");
		return -ENOENT;
	}

	page = pfn_to_page(PFN_DOWN(mem->base));
	pages = mem->size / PAGE_SIZE;

	// Grab reference to page so they're never freed back into the allocator
	for (i = 0; i < pages; ++i) {
		set_page_count(page+i, 1);
	}

	sram = devm_kzalloc(dev, sizeof(*sram), GFP_KERNEL);
	if (!sram) {
		dev_err(dev, "Unable to allocate sram device data\n");
		return -ENOMEM;
	}

	sram->dev = dev;
	sram->start = page;
	sram->rmem = mem;
	dev_set_drvdata(&pdev->dev, sram);

	sram->miscdev.minor = MISC_DYNAMIC_MINOR;
	sram->miscdev.name = kasprintf(GFP_KERNEL, "%s%d", SRAM_MMAP_DRV_NAME, id);
	sram->miscdev.fops = &sram_fops;
	sram->miscdev.parent = dev;

	ret = misc_register(&sram->miscdev);
	if (ret < 0)
		dev_err(dev, "Faied to register sram mmap misc device\n");

	return ret;
}

static int adi_sram_mmap_remove(struct platform_device *pdev)
{
	struct adi_sram_mmap *sram = dev_get_drvdata(&pdev->dev);
	misc_deregister(&sram->miscdev);

	return 0;
}

static struct platform_driver adi_sram_mmap_driver = {
	.probe = adi_sram_mmap_probe,
	.remove = adi_sram_mmap_remove,
	.driver = {
		.name = SRAM_MMAP_DRV_NAME,
		.of_match_table = of_match_ptr(adi_sram_mmap_of_match),
	},
};

static int rmem_sram_init(struct reserved_mem *rmem, struct device *dev) {
	struct sram_region_info *info = rmem->priv;
	info->dev = dev;
	return 0;
}

static void rmem_sram_release(struct reserved_mem *rmem, struct device *dev) {
	struct sram_region_info *info = rmem->priv;
	info->dev = NULL;
}

static const struct reserved_mem_ops rmem_sram_ops = {
	.device_init = rmem_sram_init,
	.device_release = rmem_sram_release,
};

static int __init rmem_sram_setup(struct reserved_mem *rmem) {
	if (next_region >= MAX_SRAM_REGIONS) {
		pr_err("Cannot allocate more SRAM regions--increase MAX_SRAM_REGIONS\n");
		return -EINVAL;
	}

	if (rmem->base & (PAGE_SIZE-1)) {
		pr_err("sram region starting at 0x%px is not page aligned!\n", (void *)rmem->base);
		return -EINVAL;
	}

	if (rmem->size & (PAGE_SIZE-1)) {
		pr_err("sram region starting at 0x%px is not a multiple of the page size (requested 0x%llx bytes)\n",
			(void *)rmem->base, rmem->size);
		return -EINVAL;
	}

	rmem->ops = &rmem_sram_ops;
	rmem->priv = &sram_regions[next_region];
	sram_regions[next_region].rmem = rmem;
	next_region += 1;

	pr_info("Reserved memory: SRAM at %pa, size %ld KiB\n",
		&rmem->base, (unsigned long) (rmem->size / SZ_1K));
	return 0;
}
RESERVEDMEM_OF_DECLARE(adi_sram, "adi,sram-access", rmem_sram_setup);

module_platform_driver(adi_sram_mmap_driver);
MODULE_DESCRIPTION("SRAM mmap misc driver for ADI processor on-chip memory");
MODULE_LICENSE("GPL");
