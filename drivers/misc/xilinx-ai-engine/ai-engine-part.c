// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx AI Engine partition driver
 *
 * Copyright (C) 2020 Xilinx, Inc.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmu_context.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <uapi/linux/xlnx-ai-engine.h>
#include <linux/xlnx-ai-engine.h>

#include "ai-engine-internal.h"
#include "ai-engine-trace.h"

/**
 * aie_cal_loc() - calculate tile location from register offset to the AI
 *		   engine device
 * @adev: AI engine device
 * @loc: memory pointer to restore returning location information
 * @regoff: tile internal register offset
 *
 * This function returns the tile location.
 */
static void aie_cal_loc(struct aie_device *adev,
			struct aie_location *loc, u64 regoff)
{
	loc->col = (u32)aie_tile_reg_field_get(aie_col_mask(adev),
					       adev->col_shift, regoff);
	loc->row = (u32)aie_tile_reg_field_get(aie_row_mask(adev),
					       adev->row_shift, regoff);
}

/**
 * aie_part_reg_validation() - validate AI engine partition register access
 * @apart: AI engine partition
 * @offset: AI engine register offset relative in partition.
 * @len: len of data to write/read
 * @is_write: is the access to write to register
 * @return: 0 for success, or negative value for failure.
 *
 * This function validate if the register to access is within the AI engine
 * partition. If it is write access, if the register is writable by user.
 */
static int aie_part_reg_validation(struct aie_partition *apart, size_t offset,
				   size_t len, u8 is_write)
{
	struct aie_device *adev;
	u32 regend32, ttype;
	u64 regoff, regend64;
	struct aie_location loc, aloc;
	unsigned int i, num_mems;
	struct aie_part_mem *pmem = apart->pmems;

	adev = apart->adev;
	if (offset % sizeof(u32)) {
		dev_err(&apart->dev,
			"Invalid reg off(0x%zx), not 32bit aligned.\n",
			offset);
		return -EINVAL;
	}

	if (len % sizeof(u32)) {
		dev_err(&apart->dev, "Invalid reg operation len %zu.\n", len);
		return -EINVAL;
	}

	regoff = aie_cal_tile_reg(adev, offset);
	regend64 = regoff + len - 1;
	if (regend64 >= BIT_ULL(adev->row_shift)) {
		dev_err(&apart->dev,
			"Invalid reg operation len %zu.\n", len);
		return -EINVAL;
	}

	aie_cal_loc(adev, &loc, offset);
	if (aie_validate_location(apart, loc)) {
		dev_err(&apart->dev,
			"Invalid (%d,%d) out of part(%d,%d)\n",
			loc.col, loc.row,
			apart->range.size.col, apart->range.size.row);
		return -EINVAL;
	}

	/*
	 * We check if a tile is gated before trying to access the tile.
	 * As we mmap() the registers as read only to enable faster status
	 * enquiry, and mmap() memories as write/read to faster memory access,
	 * user can still access the clock gated tiles from userspace by
	 * accessing the mmapped space.
	 * Accessing the gated tiles can cause decode error. With PDI flow,
	 * the PDI sets up the SHIM NOC AXI MM to only generate AI engine error
	 * even instead of generating the NSU error. but for non PDI flow, as
	 * the AXI MM register are protected register, until we have EEMI API
	 * to update the AXI MM register, access the gated tiles can cause NSU
	 * errors.
	 * TODO: To solve this, we need to either request EEMI to configure
	 * AXI MM or split the mmapped space into tiles based lists.
	 */
	aloc.col = loc.col + apart->range.start.col;
	aloc.row = loc.row;
	if (!aie_part_check_clk_enable_loc(apart, &aloc)) {
		dev_err(&apart->dev,
			"Tile(%u,%d) is gated.\n", loc.col, loc.row);
		return -EINVAL;
	}

	num_mems = apart->adev->ops->get_mem_info(apart->adev,
						  &apart->range, NULL);
	for (i = 0; i < num_mems; i++) {
		if (i == AIE_PM_MEM_OFFSET_IDX)
			continue;
		if (pmem[i].mem.range.start.row <= aloc.row &&
		    (pmem[i].mem.range.start.row +
		     pmem[i].mem.range.size.row) > aloc.row) {
			if (pmem[i].mem.offset <= regoff &&
			    ((pmem[i].mem.offset + pmem[i].mem.size)
			      >= regoff)) {
				if ((pmem[i].mem.offset + pmem[i].mem.size)
				     < regend64) {
					dev_err(&apart->dev,
						"address 0x%zx, 0x%zx not accessible.\n",
						offset, len);
					return -EINVAL;
				}
			} else if (pmem[i].mem.offset > regoff &&
				   (pmem[i].mem.offset <= regend64 &&
				    ((pmem[i].mem.offset + pmem[i].mem.size)
				     >= regend64))) {
				dev_err(&apart->dev,
					"address 0x%zx, 0x%zx not accessible.\n",
					offset, len);
				return -EINVAL;
			}
		}
	}

	if (!is_write)
		return 0;

	regend32 = lower_32_bits(regend64);
	ttype = adev->ops->get_tile_type(adev, &loc);
	for (i = 0; i < adev->num_kernel_regs; i++) {
		const struct aie_tile_regs *regs;
		u32 rttype, writable;

		regs = &adev->kernel_regs[i];
		rttype = (regs->attribute & AIE_REGS_ATTR_TILE_TYPE_MASK) >>
			 AIE_REGS_ATTR_TILE_TYPE_SHIFT;
		writable = (regs->attribute & AIE_REGS_ATTR_PERM_MASK) >>
			   AIE_REGS_ATTR_PERM_SHIFT;
		if (!(BIT(ttype) & rttype))
			continue;
		if ((regoff >= regs->soff && regoff <= regs->eoff) ||
		    (regend32 >= regs->soff && regend32 <= regs->eoff)) {
			if (!writable) {
				dev_err(&apart->dev,
					"reg 0x%zx,0x%zx not writable.\n",
					offset, len);
				return -EINVAL;
			}
		}
	}

	return 0;
}

/**
 * aie_part_write_register() - AI engine partition write register
 * @apart: AI engine partition
 * @offset: AI engine register offset
 * @len: len of data to write
 * @data: data to write
 * @mask: mask, if it is non 0, it is mask write.
 * @return: number of bytes write for success, or negative value for failure.
 *
 * This function writes data to the specified registers.
 * If the mask is non 0, it is mask write.
 */
static int aie_part_write_register(struct aie_partition *apart, size_t offset,
				   size_t len, void *data, u32 mask)
{
	struct aie_aperture *aperture = apart->aperture;
	u32 i;
	int ret;
	void __iomem *va;

	trace_aie_part_write_register(apart, offset, len, data, mask);
	if (mask && len > sizeof(u32)) {
		/* For mask write, only allow 32bit. */
		dev_err(&apart->dev,
			"failed mask write, len is more that 32bit.\n");
		return -EINVAL;
	}

	/* offset is expected to be relative to the start of the partition */
	ret = aie_part_reg_validation(apart, offset, len, 1);
	if (ret < 0) {
		dev_err(&apart->dev, "failed to write to 0x%zx,0x%zx.\n",
			offset, len);
		return ret;
	}

	offset += aie_aperture_cal_regoff(aperture, apart->range.start, 0);
	va = aperture->base + offset;
	if (!mask) {
		/*
		 * TODO: use the burst mode to improve performance when len
		 * is more than 4. Additional checks have to be made to ensure
		 * the destination address is 128 bit aligned when burst mode
		 * is used.
		 */
		for (i = 0; i < len; i = i + 4) {
			u32 val = *((u32 *)(data + i));

			trace_aie_part_write_register_data(apart, i, val, offset + i);
			iowrite32(val, va + i);
		}
	} else {
		u32 val = ioread32(va);

		val &= ~mask;
		val |= *((u32 *)data) & mask;
		trace_aie_part_write_register_data(apart, 0, val, offset);
		iowrite32(val, va);
	}

	return (int)len;
}

/**
 * aie_partition_write() - AI engine partition write
 * @dev: AI engine tile device
 * @loc: AI engine tile location
 * @offset: AI engine register offset
 * @len: len of data to write
 * @data: data to write
 * @mask: mask, if it is non 0, it is mask write.
 * @return: number of bytes write for success, or negative value for failure.
 *
 * This function writes data to the specified registers.
 * If the mask is non 0, it is mask write.
 */
int aie_partition_write(struct device *dev, struct aie_location loc,
			size_t offset, size_t len, void *data, u32 mask)
{
	struct aie_partition *apart;
	int ret;

	if (!dev || !data)
		return -EINVAL;

	apart = dev_to_aiepart(dev);
	if (IS_ERR(apart))
		return -EINVAL;

	offset = aie_cal_regoff(apart->adev, loc, offset);
	ret = aie_part_write_register(apart, offset, len, data, mask);
	if (ret < 0)
		dev_err(&apart->dev, "failed to write to 0x%zx,0x%zx.\n",
			offset, len);

	return ret;
}
EXPORT_SYMBOL_GPL(aie_partition_write);

/**
 * aie_part_read_register() - AI engine partition read register
 * @apart: AI engine partition
 * @offset: AI engine register offset
 * @len: len of data to read
 * @data: pointer to the memory to store the read data
 * @return: number of bytes read for success, or negative value for failure.
 *
 * This function reads data from the specified registers.
 */
static int aie_part_read_register(struct aie_partition *apart, size_t offset,
				  size_t len, void *data)
{
	struct aie_aperture *aperture = apart->aperture;
	void __iomem *va;
	int ret;

	/* offset is expected to be relative to the start of the partition */
	ret = aie_part_reg_validation(apart, offset, len, 0);
	if (ret) {
		dev_err(&apart->dev, "Invalid read request 0x%zx,0x%zx.\n",
			offset, len);
		return -EINVAL;
	}

	offset += aie_aperture_cal_regoff(aperture, apart->range.start, 0);
	va = aperture->base + offset;
	if (len == 4)
		*((u32 *)data) = ioread32(va);
	else
		memcpy_fromio(data, va, len);

	return (int)len;
}

/**
 * aie_partition_read() - AI engine partition read register
 * @dev: AI engine device
 * @loc: AI engine tile location
 * @offset: AI engine register offset
 * @len: len of data to read
 * @data: pointer to the memory to store the read data
 * @return: number of bytes read for success, or negative value for failure.
 *
 * This function reads data from the specified registers.
 */
int aie_partition_read(struct device *dev, struct aie_location loc,
		       size_t offset, size_t len, void *data)
{
	struct aie_partition *apart;
	int ret;

	if (!dev || !data)
		return -EINVAL;

	apart = dev_to_aiepart(dev);
	if (IS_ERR(apart))
		return -EINVAL;

	offset = aie_cal_regoff(apart->adev, loc, offset);
	ret = aie_part_read_register(apart, offset, len, data);
	if (ret < 0)
		dev_err(&apart->dev, "failed to write to 0x%zx,0x%zx.\n",
			offset, len);

	return ret;
}
EXPORT_SYMBOL_GPL(aie_partition_read);

/**
 * aie_part_block_set() - AI Engine partition block set registers
 * @apart: AI engine partition
 * @args: regestier access arguments
 * @return: 0 for success, and negative value for failure
 */
static int aie_part_block_set(struct aie_partition *apart,
			      struct aie_reg_args *args)
{
	u32 i;
	int ret;

	for (i = 0; i < args->len; i++) {
		size_t offset = (size_t)args->offset;

		ret = aie_part_write_register(apart, offset + i * 4,
					      sizeof(args->val), &args->val,
					      args->mask);
		if (ret < 0)
			return ret;
	}

	return 0;
}

#ifndef CONFIG_ARM64_SW_TTBR0_PAN
/**
 * aie_part_pin_user_region() - pin user pages for access
 * @apart: AI engine partition
 * @region: user space region to pin. Includes virtual address and size of the
 *	    user space buffer.
 * @return: 0 for success, and negative value for failure.
 *
 * This function pins all the pages of a user space buffer.
 */
static int aie_part_pin_user_region(struct aie_partition *apart,
				    struct aie_part_pinned_region *region)
{
	int ret, npages;
	unsigned long first, last;
	struct page **pages;

	first = (region->user_addr & PAGE_MASK) >> PAGE_SHIFT;
	last = ((region->user_addr + region->len - 1) & PAGE_MASK) >>
		PAGE_SHIFT;
	npages = last - first + 1;

	pages = kcalloc(npages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	ret = pin_user_pages_fast(region->user_addr, npages, 0, pages);
	if (ret < 0) {
		kfree(pages);
		dev_err(&apart->dev, "Unable to pin user pages\n");
		return ret;
	} else if (ret != npages) {
		unpin_user_pages(pages, ret);
		kfree(pages);
		dev_err(&apart->dev, "Unable to pin all user pages\n");
		return -EFAULT;
	}

	region->pages = pages;
	region->npages = npages;

	return 0;
}
#endif

/**
 * aie_part_unpin_user_region() - unpin user pages
 * @region: user space region to unpin.
 *
 * This function unpins all the pages of a user space buffer. User region passed
 * to this api must be pinned using aie_part_pin_user_region()
 */
static void aie_part_unpin_user_region(struct aie_part_pinned_region *region)
{
	unpin_user_pages(region->pages, region->npages);
	kfree(region->pages);
}

/**
 * aie_part_copy_user_region() - copy user space data to kernel space
 * @apart: AI engine partition
 * @region: User space region to copy data from
 * @data: User data pointer
 *
 * This function replaces the previous method of pinning user pages
 * directly and instead copies user space data to kernel space using
 * copy_from_user(). It ensures that user space data is safely and securely
 * copied to the kernel without directly accessing user pages.
 *
 * @return: 0 on success, negative error code on failure.
 */
static int aie_part_copy_user_region(struct aie_partition *apart,
				     struct aie_part_pinned_region *region,
				     void *data)
{
#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	if (region->len == 0)
		return 0;

	region->user_addr = (__u64)dma_alloc_coherent(&apart->dev, region->len,
			&region->aie_dma_handle,
			GFP_KERNEL | GFP_DMA);
	if (!region->user_addr)
		return -ENOMEM;

	if (copy_from_user((void *)region->user_addr, (const void __user *)data,
			   region->len)) {
		dma_free_coherent(&apart->dev, region->len,
				  (void *)region->user_addr,
				  region->aie_dma_handle);
		return -EFAULT;
	}
#else
	int ret;

	region->user_addr = (__u64)data;
	ret = aie_part_pin_user_region(apart, region);
	if (ret)
		return ret;
#endif
	return 0;
}

/**
 * aie_part_free_region() - free allocated memory
 * @apart: AI engine partition
 * @region: User space region structure to free
 *
 * This function simplifies the freeing of allocated kernel memory.
 * It only frees the allocated memory associated with the user space region.
 */
static void aie_part_free_region(struct aie_partition *apart,
				 struct aie_part_pinned_region *region)
{
#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	dma_free_coherent(&apart->dev, region->len, (void *)region->user_addr,
			  region->aie_dma_handle);
#else
	aie_part_unpin_user_region(region);
#endif
}

/**
 * aie_part_access_regs() - AI engine partition registers access
 * @apart: AI engine partition
 * @num_reqs: number of access requests
 * @reqs: array of registers access
 * @return: 0 for success, and negative value for failure.
 *
 * This function executes AI engine partition register access requests.
 */
static int aie_part_access_regs(struct aie_partition *apart, u32 num_reqs,
				struct aie_reg_args *reqs)
{
	u32 i;

	for (i = 0; i < num_reqs; i++) {
		struct aie_reg_args *args = &reqs[i];
		int ret;

		trace_aie_part_access_reg(apart, args->op);
		switch (args->op) {
		case AIE_REG_WRITE:
		{
			ret = aie_part_write_register(apart,
						      (size_t)args->offset,
						      sizeof(args->val),
						      &args->val, args->mask);
			break;
		}
		case AIE_REG_BLOCKWRITE:
		{
			struct aie_part_pinned_region region;

			region.len = args->len * sizeof(u32);
			ret = aie_part_copy_user_region(apart, &region, (void *)args->dataptr);
			if (ret)
				break;

			ret = aie_part_write_register(apart,
						      (size_t)args->offset,
						      sizeof(u32) * args->len,
						      (void *)region.user_addr,
						      args->mask);
			aie_part_free_region(apart, &region);
			break;
		}
		case AIE_REG_BLOCKSET:
		{
			ret = aie_part_block_set(apart, args);
			break;
		}
		case AIE_CONFIG_SHIMDMA_BD:
		{
			struct aie_part_pinned_region data_region;

			data_region.len = sizeof(struct aie_dma_bd_args);
			ret = aie_part_copy_user_region(apart, &data_region,
							(void *)args->dataptr);
			if (ret)
				break;

			ret =  aie_part_set_bd(apart,
				(struct aie_dma_bd_args *)data_region.user_addr);
			aie_part_free_region(apart, &data_region);
			break;
		}
		case AIE_CONFIG_SHIMDMA_DMABUF_BD:
		{
			struct aie_part_pinned_region data_region;

			data_region.len = sizeof(struct aie_dmabuf_bd_args);
			ret = aie_part_copy_user_region(apart, &data_region,
							(void *)args->dataptr);
			if (ret)
				break;

			ret =  aie_part_set_dmabuf_bd(apart,
				(struct aie_dmabuf_bd_args *)data_region.user_addr);
			aie_part_unpin_user_region(&data_region);
			break;
		}
		default:
			dev_err(&apart->dev,
				"Invalid register command type: %u.\n",
				args->op);
			return -EINVAL;
		}

		if (ret < 0) {
			dev_err(&apart->dev, "reg op %u failed: 0x%llx.\n",
				args->op, args->offset);
			return ret;
		}
	}

	return 0;
}

/**
 * aie_part_execute_transaction_from_user() - AI engine configure registers
 * @apart: AI engine partition
 * @user_args: arguments passed by user.
 * @return: 0 for success, and negative value for failure.
 *
 * This function executes AI engine register access requests that are part of a
 * buffer that is populated and passed by user.
 */
static int aie_part_execute_transaction_from_user(struct aie_partition *apart,
						  void __user *user_args)
{
	long ret;
	struct aie_txn_inst txn_inst;
	struct aie_part_pinned_region region;

	if (copy_from_user(&txn_inst, user_args, sizeof(txn_inst)))
		return -EFAULT;

	if (txn_inst.num_cmds == 0)
		return 0;

	region.len = txn_inst.num_cmds * sizeof(struct aie_reg_args);
	ret = aie_part_copy_user_region(apart, &region, (void *)txn_inst.cmdsptr);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&apart->mlock);
	if (ret) {
		aie_part_free_region(apart, &region);
		return ret;
	}

	ret = aie_part_access_regs(apart, txn_inst.num_cmds,
				   (struct aie_reg_args *)region.user_addr);

	mutex_unlock(&apart->mlock);

	aie_part_free_region(apart, &region);
	return ret;
}

/**
 * aie_part_create_event_bitmap() - create event bitmap for all modules in a
 *				    given partition.
 * @apart: AI engine partition
 * @return: 0 for success, and negative value for failure.
 */
static int aie_part_create_event_bitmap(struct aie_partition *apart)
{
	struct aie_range range = apart->range;
	u32 bitmap_sz;
	u32 num_aie_module = range.size.col * (range.size.row - 1);
	int ret;

	/*
	 * TODO: resource manager, events is not supported for AIEML device and
	 * the users are not expected to call any function as of now.
	 */
	if (apart->adev->dev_gen == AIE_DEVICE_GEN_AIEML) {
		dev_dbg(&apart->dev, "Skipping event bitmap allocation.\n");
		return 0;
	}
	bitmap_sz = num_aie_module * apart->adev->core_events->num_events;
	ret = aie_resource_initialize(&apart->core_event_status, bitmap_sz);
	if (ret) {
		dev_err(&apart->dev,
			"failed to initialize event status resource.\n");
		return -ENOMEM;
	}

	bitmap_sz = num_aie_module * apart->adev->mem_events->num_events;
	ret = aie_resource_initialize(&apart->mem_event_status, bitmap_sz);
	if (ret) {
		dev_err(&apart->dev,
			"failed to initialize event status resource.\n");
		return -ENOMEM;
	}

	bitmap_sz = range.size.col * apart->adev->pl_events->num_events;
	ret = aie_resource_initialize(&apart->pl_event_status, bitmap_sz);
	if (ret) {
		dev_err(&apart->dev,
			"failed to initialize event status resource.\n");
		return -ENOMEM;
	}
	return 0;
}

/**
 * aie_part_release_event_bitmap() - Deallocates event bitmap for all modules
 *				     in a given partition.
 * @apart: AI engine partition
 * @return: 0 for success, and negative value for failure.
 */
static void aie_part_release_event_bitmap(struct aie_partition *apart)
{
	/* TODO: remove check once resource manager is enabled for AIEML. */
	if (apart->adev->dev_gen == AIE_DEVICE_GEN_AIEML)
		return;

	aie_resource_uninitialize(&apart->core_event_status);
	aie_resource_uninitialize(&apart->mem_event_status);
	aie_resource_uninitialize(&apart->pl_event_status);
}

static int aie_part_release(struct inode *inode, struct file *filp)
{
	struct aie_partition *apart = filp->private_data;
	int ret;

	/* some reset bits in NPI are global, we need to lock adev */
	ret = mutex_lock_interruptible(&apart->adev->mlock);
	if (ret)
		return ret;

	ret = mutex_lock_interruptible(&apart->mlock);
	if (ret)
		return ret;

	aie_part_release_dmabufs(apart);
	/* aie_part_clean() will do hardware reset */
	if (apart->adev->ops->part_clean)
		apart->adev->ops->part_clean(apart);
	mutex_unlock(&apart->adev->mlock);

	apart->error_cb.cb = NULL;
	apart->error_cb.priv = NULL;
	apart->status = 0;
	apart->error_to_report = 0;

	aie_part_clear_cached_events(apart);

	aie_part_rscmgr_reset(apart);

	mutex_unlock(&apart->mlock);
	aie_part_remove(apart);

	return 0;
}

static ssize_t aie_part_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct file *filp = iocb->ki_filp;
	struct aie_partition *apart = filp->private_data;
	size_t len = iov_iter_count(from);
	loff_t offset = iocb->ki_pos;
	void *buf;
	int ret;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (!copy_from_iter_full(buf, len, from)) {
		kfree(buf);
		return -EFAULT;
	}

	ret = mutex_lock_interruptible(&apart->mlock);
	if (ret) {
		kfree(buf);
		return ret;
	}

	ret = aie_part_write_register(apart, (size_t)offset, len, buf, 0);
	mutex_unlock(&apart->mlock);
	kfree(buf);

	return ret;
}

static ssize_t aie_part_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct file *filp = iocb->ki_filp;
	struct aie_partition *apart = filp->private_data;
	size_t len = iov_iter_count(to);
	loff_t offset = iocb->ki_pos;
	void *buf;
	int ret;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = mutex_lock_interruptible(&apart->mlock);
	if (ret) {
		kfree(buf);
		return ret;
	}

	ret = aie_part_read_register(apart, (size_t)offset, len, buf);
	mutex_unlock(&apart->mlock);
	if (ret > 0) {
		if (copy_to_iter(buf, ret, to) != len) {
			dev_err(&apart->dev, "Failed to copy to read iter.\n");
			ret = -EFAULT;
		}
	}
	kfree(buf);

	return ret;
}

static const struct vm_operations_struct aie_part_physical_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys,
#endif
};

static int aie_part_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct aie_partition *apart = fp->private_data;
	struct aie_device *adev = apart->adev;
	unsigned long offset = vma->vm_pgoff * PAGE_SIZE;
	phys_addr_t addr;
	size_t size;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	/* Only allow userspace directly read registers */
	if (vma->vm_flags & VM_WRITE) {
		dev_err(&apart->dev, "%s: do not support writable mmap.\n",
			__func__);
		return -EINVAL;
	}
	vma->vm_private_data = apart;
	vma->vm_ops = &aie_part_physical_vm_ops;
	size = apart->range.size.col << adev->col_shift;
	if ((vma->vm_end - vma->vm_start) > (size - offset)) {
		dev_err(&apart->dev,
			"%s: size exceed.\n", __func__);
		return -EINVAL;
	}
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	/* Calculate the partition address */
	addr = apart->aperture->res.start;
	addr += (phys_addr_t)apart->range.start.col << adev->col_shift;
	addr += (phys_addr_t)apart->range.start.row << adev->row_shift;
	addr += offset;
	return remap_pfn_range(vma,
			       vma->vm_start,
			       addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

static long aie_part_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct aie_partition *apart = fp->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	trace_aie_part_ioctl(apart, _IOC_NR(cmd));
	switch (cmd) {
	case AIE_PARTITION_INIT_IOCTL:
		if (apart->adev->ops->part_init)
			return apart->adev->ops->part_init(apart, argp);
		else
			return -EINVAL;
	case AIE_PARTITION_TEAR_IOCTL:
		if (apart->adev->ops->part_teardown)
			return apart->adev->ops->part_teardown(apart);
		else
			return -EINVAL;
	case AIE_PARTITION_CLR_CONTEXT_IOCTL:
		if (apart->adev->ops->part_clear_context)
			return apart->adev->ops->part_clear_context(apart);
		else
			return -EINVAL;
	case AIE_REG_IOCTL:
	{
		struct aie_reg_args raccess;

		if (copy_from_user(&raccess, argp, sizeof(raccess)))
			return -EFAULT;

		ret = mutex_lock_interruptible(&apart->mlock);
		if (ret)
			return ret;

		ret = aie_part_access_regs(apart, 1, &raccess);
		mutex_unlock(&apart->mlock);
		break;
	}
	case AIE_GET_MEM_IOCTL:
		return aie_mem_get_info(apart, arg);
	case AIE_DMA_MEM_ALLOCATE_IOCTL:
	{
		__kernel_size_t size;

		if (get_user(size, (__kernel_size_t *)argp))
			return -EFAULT;

		size = PAGE_ALIGN(size);
		ret = aie_dma_mem_alloc(apart, size);
		if (put_user(size, (__kernel_size_t *)argp)) {
			aie_dma_mem_free(ret);
			return -EFAULT;
		}
		return ret;
	}
	case AIE_DMA_MEM_FREE_IOCTL:
	{
		int fd;

		if (get_user(fd, (int *)argp))
			return -EFAULT;
		return aie_dma_mem_free(fd);
	}
	case AIE_ATTACH_DMABUF_IOCTL:
		return aie_part_attach_dmabuf_req(apart, argp);
	case AIE_DETACH_DMABUF_IOCTL:
		return aie_part_detach_dmabuf_req(apart, argp);
	case AIE_UPDATE_SHIMDMA_DMABUF_BD_ADDR_IOCTL:
		return aie_part_update_dmabuf_bd_from_user(apart, argp);
	case AIE_SET_SHIMDMA_BD_IOCTL:
		return aie_part_set_bd_from_user(apart, argp);
	case AIE_SET_SHIMDMA_DMABUF_BD_IOCTL:
		return aie_part_set_dmabuf_bd_from_user(apart, argp);
	case AIE_REQUEST_TILES_IOCTL:
		return aie_part_request_tiles_from_user(apart, argp);
	case AIE_RELEASE_TILES_IOCTL:
		return aie_part_release_tiles_from_user(apart, argp);
	case AIE_TRANSACTION_IOCTL:
		return aie_part_execute_transaction_from_user(apart, argp);
	case AIE_RSC_REQ_IOCTL:
		return aie_part_rscmgr_rsc_req(apart, argp);
	case AIE_RSC_REQ_SPECIFIC_IOCTL:
		return aie_part_rscmgr_rsc_req_specific(apart, argp);
	case AIE_RSC_RELEASE_IOCTL:
		return aie_part_rscmgr_rsc_release(apart, argp);
	case AIE_RSC_FREE_IOCTL:
		return aie_part_rscmgr_rsc_free(apart, argp);
	case AIE_RSC_CHECK_AVAIL_IOCTL:
		return aie_part_rscmgr_rsc_check_avail(apart, argp);
	case AIE_RSC_GET_COMMON_BROADCAST_IOCTL:
		return aie_part_rscmgr_get_broadcast(apart, argp);
	case AIE_RSC_GET_STAT_IOCTL:
		return aie_part_rscmgr_get_statistics(apart, argp);
	case AIE_SET_COLUMN_CLOCK_IOCTL:
		return aie_part_set_column_clock_from_user(apart, argp);
	default:
		dev_err(&apart->dev, "Invalid/Unsupported ioctl command %u.\n",
			cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

const struct file_operations aie_part_fops = {
	.owner		= THIS_MODULE,
	.release	= aie_part_release,
	.read_iter	= aie_part_read_iter,
	.write_iter	= aie_part_write_iter,
	.mmap		= aie_part_mmap,
	.unlocked_ioctl	= aie_part_ioctl,
};

/**
 * aie_part_open() - open the AI engine partition instance to get it ready to
 *		     be used.
 * @apart: AI engine partition instance pointer
 * @rsc_metadata: pointer to static resource metadata
 * @return: 0 for success, negative value for failure
 *
 * This function will make the AI engine partition instance ready to use. It
 * should be called when the partition is requested.
 */
int aie_part_open(struct aie_partition *apart, void *rsc_metadata)
{
	int ret;

	/* scan to setup the initial clock state for tiles */
	ret = aie_part_scan_clk_state(apart);
	if (ret)
		return ret;

	/* Sets bitmaps of statically allocated resources */
	if (rsc_metadata) {
		ret = aie_part_rscmgr_set_static(apart,
						 rsc_metadata);
		if (ret)
			return ret;
	}

	/* preallocate memory pool for storing dmabuf descriptors */
	ret =  aie_part_prealloc_dbufs_cache(apart);
	if (ret)
		return ret;

	/* check if there is any errors reported for the partition */
	if (aie_part_has_error(apart))
		schedule_work(&apart->aperture->backtrack);

	apart->status = XAIE_PART_STATUS_INUSE;

	return 0;
}

/**
 * aie_tile_release_device() - release an AI engine tile instance
 * @dev: AI engine tile device
 *
 * It will be called by device driver core when no one holds a valid
 * pointer to @dev anymore.
 */
static void aie_tile_release_device(struct device *dev)
{
	(void)dev;
}

/**
 * aie_part_release_device() - release an AI engine partition instance
 * @dev: AI engine partition device
 *
 * It will be called by device driver core when no one holds a valid
 * pointer to @dev anymore.
 */
static void aie_part_release_device(struct device *dev)
{
	struct aie_partition *apart = dev_to_aiepart(dev);
	struct aie_aperture *aperture = apart->aperture;
	int ret;

	ret = mutex_lock_interruptible(&aperture->mlock);
	if (ret) {
		dev_warn(&apart->dev,
			 "getting adev->mlock is interrupted by signal\n");
	}

	aie_resource_put_region(&aperture->cols_res, apart->range.start.col,
				apart->range.size.col);
	aie_part_release_event_bitmap(apart);
	list_del(&apart->node);
	mutex_unlock(&aperture->mlock);
	aie_resource_uninitialize(&apart->cores_clk_state);
	aie_resource_uninitialize(&apart->tiles_inuse);
	aie_part_rscmgr_finish(apart);
	/* Check and set frequency requirement for aperture */
	aie_part_set_freq(apart, 0);
}

/**
 * aie_part_create_mems_info() - creates array to store the AI engine partition
 *				 different memories types information
 * @apart: AI engine partition
 * @return: 0 for success, negative value for failure
 *
 * This function will create array to store the information of different
 * memories types in the partition. This array is stored in @apart->pmems.
 */
static int aie_part_create_mems_info(struct aie_partition *apart)
{
	unsigned int i, num_mems;

	num_mems = apart->adev->ops->get_mem_info(apart->adev, &apart->range,
						  NULL);
	if (!num_mems)
		return 0;

	apart->pmems = devm_kcalloc(&apart->dev, num_mems,
				    sizeof(struct aie_part_mem),
				    GFP_KERNEL);
	if (!apart->pmems)
		return -ENOMEM;

	apart->adev->ops->get_mem_info(apart->adev, &apart->range,
				       apart->pmems);
	for (i = 0; i < num_mems; i++) {
		struct aie_mem *mem = &apart->pmems[i].mem;

		apart->pmems[i].apart = apart;
		apart->pmems[i].size = mem->size *
				       mem->range.size.col *
				       mem->range.size.row;
	}
	return 0;
}

/**
 * aie_create_tiles() - create AI engine tile devices
 * @apart: AI engine partition
 * @return: 0 for success, error code on failure
 *
 * This function creates AI engine child tile devices for a given partition.
 */
static int aie_create_tiles(struct aie_partition *apart)
{
	struct aie_tile *atile;
	u32 row, col, numtiles;
	int ret = 0;

	numtiles = apart->range.size.col * apart->range.size.row;
	atile = devm_kzalloc(&apart->dev, numtiles * sizeof(struct aie_tile),
			     GFP_KERNEL);
	if (!atile)
		return -ENOMEM;

	apart->atiles = atile;
	for (col = 0; col < apart->range.size.col; col++) {
		for (row = 0; row < apart->range.size.row; row++) {
			struct device *tdev = &atile->dev;
			char tdevname[10];

			atile->apart = apart;
			atile->loc.col = apart->range.start.col + col;
			atile->loc.row = apart->range.start.row + row;
			device_initialize(tdev);
			tdev->parent = &apart->dev;
			dev_set_drvdata(tdev, atile);
			snprintf(tdevname, sizeof(tdevname) - 1, "%d_%d",
				 apart->range.start.col + col,
				 apart->range.start.row + row);
			dev_set_name(tdev, tdevname);
			tdev->release = aie_tile_release_device;
			ret = device_add(tdev);
			if (ret) {
				dev_err(tdev, "tile device_add failed: %d\n",
					ret);
				put_device(tdev);
				return ret;
			}
			ret = aie_tile_sysfs_create_entries(atile);
			if (ret) {
				dev_err(tdev, "failed to create tile sysfs: %d\n",
					ret);
				device_del(tdev);
				put_device(tdev);
				return ret;
			}
			atile++;
		}
	}
	return ret;
}

/**
 * aie_create_partition() - create AI engine partition instance
 * @aperture: AI engine aperture
 * @partition_id: AI engine partition ID which contains partition range
 *		  information such as start column and number of columns
 * @return: created AI engine partition pointer for success, and PTR_ERR
 *	    for failure.
 *
 * This function creates an AI engine partition instance.
 * It creates AI engine partition, the AI engine partition device and
 * the AI engine partition character device.
 */
struct aie_partition *aie_create_partition(struct aie_aperture *aperture,
					   u32 partition_id)
{
	struct aie_partition *apart;
	struct device *dev;
	int ret;

	apart = devm_kzalloc(&aperture->dev, sizeof(*apart), GFP_KERNEL);
	if (!apart)
		return ERR_PTR(-ENOMEM);

	apart->aperture = aperture;
	apart->adev = aperture->adev;
	apart->partition_id = partition_id;
	INIT_LIST_HEAD(&apart->dbufs);
	INIT_LIST_HEAD(&apart->dma_mem);
	mutex_init(&apart->mlock);
	apart->range.start.col = aie_part_id_get_start_col(partition_id);
	apart->range.size.col = aie_part_id_get_num_cols(partition_id);
	apart->range.start.row = aperture->range.start.row;
	apart->range.size.row = aperture->range.size.row;

	/* Create AI engine partition device */
	dev = &apart->dev;
	dev->parent = &aperture->dev;
	dev->class = aie_class;
	dev_set_drvdata(dev, apart);
	dev_set_name(dev, "aiepart_%d_%d", apart->range.start.col,
		     apart->range.size.col);
	/* We can now rely on the release function for cleanup */
	dev->release = aie_part_release_device;
	ret = device_register(dev);
	if (ret) {
		dev_err(dev, "device_add failed: %d\n", ret);
		put_device(dev);
		return ERR_PTR(ret);
	}

	/* Set up the DMA mask */
	set_dma_ops(dev, get_dma_ops(&aperture->dev));
	ret = dma_coerce_mask_and_coherent(dev, dma_get_mask(&aperture->dev));
	if (ret) {
		dev_warn(dev,
			 "Failed to set DMA mask %llx. Trying to continue... %x\n",
			 dma_get_mask(&aperture->dev), ret);
	}

	/* Create AI Engine tile devices */
	ret = aie_create_tiles(apart);
	if (ret) {
		dev_err(dev, "Failed to create tile devices.\n");
		put_device(dev);
		return ERR_PTR(ret);
	}

	/*
	 * Create array to keep the information of the different types of tile
	 * memories information of the AI engine partition.
	 */
	ret = aie_part_create_mems_info(apart);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = apart->adev->ops->init_part_clk_state(apart);
	if (ret) {
		put_device(dev);
		return ERR_PTR(ret);
	}

	/*
	 * Create bitmap to record event status for each module in a
	 * partition
	 */
	ret = aie_part_create_event_bitmap(apart);
	if (ret < 0) {
		dev_err(&apart->dev, "Failed to allocate event bitmap.\n");
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = aie_part_rscmgr_init(apart);
	if (ret < 0) {
		dev_err(&apart->dev,
			"Failed to initialize resources bitmaps.\n");
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = aie_part_sysfs_create_entries(apart);
	if (ret) {
		dev_err(&apart->dev, "Failed to create partition sysfs.\n");
		put_device(dev);
		return ERR_PTR(ret);
	}

	ret = aie_part_pm_ops_create(apart);
	if (ret) {
		dev_err(&apart->dev, "Failed to create pm ops pkt.");
		put_device(dev);
		return ERR_PTR(ret);
	}

	dev_dbg(dev, "created AIE partition device.\n");

	return apart;
}

/**
 * aie_tile_remove() - remove AI engine tile device.
 * @atile: AI engine tile.
 *
 * This function will remove AI engine tile device.
 */
static void aie_tile_remove(struct aie_tile *atile)
{
	aie_tile_sysfs_remove_entries(atile);
	device_del(&atile->dev);
	put_device(&atile->dev);
}

/**
 * aie_part_remove() - destroy AI engine partition
 * @apart: AI engine partition
 *
 * This function will remove AI engine partition.
 */
void aie_part_remove(struct aie_partition *apart)
{
	struct aie_aperture *aperture = apart->aperture;
	struct aie_tile *atile = apart->atiles;
	u32 index;

	for (index = 0; index < apart->range.size.col * apart->range.size.row;
	     index++, atile++)
		aie_tile_remove(atile);

	aie_part_sysfs_remove_entries(apart);

	device_del(&apart->dev);
	put_device(&apart->dev);
	aie_part_pm_ops_free(apart);
	devm_kfree(&aperture->dev, apart);
}

/**
 * aie_part_has_regs_mmapped() - check if registers in the partition are mapped.
 * @apart: AI engine partition
 * @return: return true if there are registers mmaped, false otherwise.
 *
 * This function checks if there are registerss in the partition mmapped in the
 * partition.
 */
bool aie_part_has_regs_mmapped(struct aie_partition *apart)
{
	struct address_space *mapping;

	mapping = apart->filep->f_inode->i_mapping;
	return mapping_mapped(mapping);
}

/**
 * aie_part_get_tile_rows - helper function to get the number of rows of a
 *			    tile type.
 *
 * @apart: AI engine partition
 * @ttype: tile type
 * @return: number of rows of a tile type
 */
int aie_part_get_tile_rows(struct aie_partition *apart,
			   enum aie_tile_type ttype)
{
	struct aie_tile_attr *tattr = &apart->adev->ttype_attr[ttype];

	/*
	 * TODO: number of rows information of the AI engine device
	 * should get from device tree.
	 */
	if (tattr->num_rows != 0xFF)
		return tattr->num_rows;
	else
		return (apart->range.size.row - tattr->start_row);
}
