// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2022-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
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
 */

/*
 * Debugfs interface to dump information about GPU_VA memory zones
 */

#include "mali_kbase_debug_mem_zones.h"
#include "mali_kbase.h"

#include <linux/list.h>
#include <linux/file.h>

#if IS_ENABLED(CONFIG_DEBUG_FS)

/**
 * debug_mem_zones_show - Show information about GPU_VA memory zones
 * @sfile: The debugfs entry
 * @data: Data associated with the entry
 *
 * This function is called to get the contents of the @c mem_zones debugfs file.
 * This lists the start address and size (in pages) of each initialized memory
 * zone within GPU_VA memory.
 *
 * Return:
 * 0 if successfully prints data in debugfs entry file
 * -1 if it encountered an error
 */
static int debug_mem_zones_show(struct seq_file *sfile, void *data)
{
	struct kbase_context *const kctx = sfile->private;
	struct kbase_reg_zone *reg_zone;
	enum kbase_memory_zone zone_idx;

	CSTD_UNUSED(data);

	kbase_gpu_vm_lock(kctx);

	for (zone_idx = 0; zone_idx < CONTEXT_ZONE_MAX; zone_idx++) {
		reg_zone = &kctx->reg_zone[zone_idx];

		if (reg_zone->base_pfn) {
			seq_printf(sfile, "%15s %u 0x%.16llx 0x%.16llx\n",
				   kbase_reg_zone_get_name(zone_idx), zone_idx, reg_zone->base_pfn,
				   reg_zone->va_size_pages);
		}
	}
#if MALI_USE_CSF
	reg_zone = &kctx->kbdev->csf.mcu_shared_zone;

	if (reg_zone && reg_zone->base_pfn) {
		seq_printf(sfile, "%15s %u 0x%.16llx 0x%.16llx\n",
			   kbase_reg_zone_get_name(MCU_SHARED_ZONE), MCU_SHARED_ZONE,
			   reg_zone->base_pfn, reg_zone->va_size_pages);
	}
#endif

	kbase_gpu_vm_unlock(kctx);
	return 0;
}

/*
 *  File operations related to debugfs entry for mem_zones
 */
static int debug_mem_zones_open(struct inode *in, struct file *file)
{
	return single_open(file, debug_mem_zones_show, in->i_private);
}

static const struct file_operations kbase_debug_mem_zones_fops = {
	.owner = THIS_MODULE,
	.open = debug_mem_zones_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 *  Initialize debugfs entry for mem_zones
 */
void kbase_debug_mem_zones_init(struct kbase_context *const kctx)
{
	/* Caller already ensures this, but we keep the pattern for
	 * maintenance safety.
	 */
	if (WARN_ON(!kctx) || WARN_ON(IS_ERR_OR_NULL(kctx->kctx_dentry)))
		return;

	debugfs_create_file("mem_zones", 0400, kctx->kctx_dentry, kctx,
			    &kbase_debug_mem_zones_fops);
}
#else
/*
 * Stub functions for when debugfs is disabled
 */
void kbase_debug_mem_zones_init(struct kbase_context *const kctx)
{
}
#endif
