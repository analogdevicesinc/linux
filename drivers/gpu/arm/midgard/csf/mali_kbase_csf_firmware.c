// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2024 ARM Limited. All rights reserved.
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

#include "mali_kbase.h"
#include "mali_kbase_csf_firmware_cfg.h"
#include "mali_kbase_csf_firmware_log.h"
#include "mali_kbase_csf_firmware_core_dump.h"
#include "mali_kbase_csf_trace_buffer.h"
#include "mali_kbase_csf_timeout.h"
#include "mali_kbase_mem.h"
#include "mali_kbase_reg_track.h"
#include "mali_kbase_mem_pool_group.h"
#include "mali_kbase_reset_gpu.h"
#include "mali_kbase_csf_scheduler.h"
#include <mali_kbase_hwaccess_time.h>
#include "device/mali_kbase_device.h"
#include "backend/gpu/mali_kbase_pm_internal.h"
#include "tl/mali_kbase_timeline_priv.h"
#include "tl/mali_kbase_tracepoints.h"
#include "mali_kbase_csf_tl_reader.h"
#include "backend/gpu/mali_kbase_clk_rate_trace_mgr.h"
#include <csf/ipa_control/mali_kbase_csf_ipa_control.h>
#include <csf/mali_kbase_csf_registers.h>
#include <csf/mali_kbase_csf_fw_io.h>
#include <mali_kbase_io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/mman.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#if (KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE)
#include <linux/set_memory.h>
#endif
#include <mmu/mali_kbase_mmu.h>
#include <asm/arch_timer.h>
#include <linux/delay.h>
#include <linux/version_compat_defs.h>
#include <mali_kbase_config_defaults.h>
#define MALI_MAX_DEFAULT_FIRMWARE_NAME_LEN ((size_t)20)

static char default_fw_name[MALI_MAX_DEFAULT_FIRMWARE_NAME_LEN] = "mali_csffw.bin";
module_param_string(fw_name, default_fw_name, sizeof(default_fw_name), 0644);
MODULE_PARM_DESC(fw_name, "firmware image");

/* The waiting time for firmware to boot */
static unsigned int csf_firmware_boot_timeout_ms;
module_param(csf_firmware_boot_timeout_ms, uint, 0444);
MODULE_PARM_DESC(csf_firmware_boot_timeout_ms, "Maximum time to wait for firmware to boot.");

static bool kbase_iter_trace_enable;

#ifdef CONFIG_MALI_DEBUG
/* Makes Driver wait indefinitely for an acknowledgment for the different
 * requests it sends to firmware. Otherwise the timeouts interfere with the
 * use of debugger for source-level debugging of firmware as Driver initiates
 * a GPU reset when a request times out, which always happen when a debugger
 * is connected.
 */
bool fw_debug; /* Default value of 0/false */
module_param(fw_debug, bool, 0444);
MODULE_PARM_DESC(fw_debug, "Enables effective use of a debugger for debugging firmware code.");
#endif

#define FIRMWARE_HEADER_MAGIC (0xC3F13A6Eul)
#define FIRMWARE_HEADER_VERSION_MAJOR (0ul)
#define FIRMWARE_HEADER_VERSION_MINOR (3ul)
#define FIRMWARE_HEADER_LENGTH (0x14ul)

#define CSF_FIRMWARE_ENTRY_SUPPORTED_FLAGS                                                    \
	(CSF_FIRMWARE_ENTRY_READ | CSF_FIRMWARE_ENTRY_WRITE | CSF_FIRMWARE_ENTRY_EXECUTE |    \
	 CSF_FIRMWARE_ENTRY_PROTECTED | CSF_FIRMWARE_ENTRY_SHARED | CSF_FIRMWARE_ENTRY_ZERO | \
	 CSF_FIRMWARE_ENTRY_CACHE_MODE)

#define CSF_FIRMWARE_ENTRY_TYPE_INTERFACE (0)
#define CSF_FIRMWARE_ENTRY_TYPE_CONFIGURATION (1)
#define CSF_FIRMWARE_ENTRY_TYPE_TRACE_BUFFER (3)
#define CSF_FIRMWARE_ENTRY_TYPE_TIMELINE_METADATA (4)
#define CSF_FIRMWARE_ENTRY_TYPE_BUILD_INFO_METADATA (6)
#define CSF_FIRMWARE_ENTRY_TYPE_FUNC_CALL_LIST (7)
#define CSF_FIRMWARE_ENTRY_TYPE_PAGE_FAULT_CNT (8)
#define CSF_FIRMWARE_ENTRY_TYPE_CORE_DUMP (9)

#define CSF_FIRMWARE_CACHE_MODE_NONE (0ul << 3)
#define CSF_FIRMWARE_CACHE_MODE_CACHED (1ul << 3)
#define CSF_FIRMWARE_CACHE_MODE_UNCACHED_COHERENT (2ul << 3)
#define CSF_FIRMWARE_CACHE_MODE_CACHED_COHERENT (3ul << 3)

#define INTERFACE_ENTRY_NAME_OFFSET (0x14)

#define TL_METADATA_ENTRY_NAME_OFFSET (0x8)

#define BUILD_INFO_METADATA_SIZE_OFFSET (0x4)
#define BUILD_INFO_GIT_SHA_LEN (40U)
#define BUILD_INFO_GIT_DIRTY_LEN (1U)
#define BUILD_INFO_GIT_SHA_PATTERN "git_sha: "

#define CSF_GLB_REQ_CFG_MASK                                           \
	(GLB_REQ_CFG_ALLOC_EN_MASK | GLB_REQ_CFG_PROGRESS_TIMER_MASK | \
	 GLB_REQ_CFG_PWROFF_TIMER_MASK | GLB_REQ_IDLE_ENABLE_MASK |    \
	 GLB_REQ_CFG_EVICTION_TIMER_MASK | GLB_REQ_ITER_TRACE_ENABLE_MASK)

static unsigned int entry_type(u32 header)
{
	return header & 0xFF;
}
static unsigned int entry_size(u32 header)
{
	return (header >> 8) & 0xFF;
}
static bool entry_update(u32 header)
{
	return (header >> 30) & 0x1;
}
static bool entry_optional(u32 header)
{
	return (header >> 31) & 0x1;
}

/**
 * struct firmware_timeline_metadata - Timeline metadata item within the MCU firmware
 *
 * @node: List head linking all timeline metadata to
 *        kbase_device:csf.firmware_timeline_metadata.
 * @name: NUL-terminated string naming the metadata.
 * @data: Metadata content.
 * @size: Metadata size.
 */
struct firmware_timeline_metadata {
	struct list_head node;
	char *name;
	char *data;
	size_t size;
};

static void reinit_page_fault_cnt_firmware_memory(struct kbase_device *kbdev)
{
	if (!kbdev->csf.page_fault_cnt_ptr)
		return;

	/* Store the GPU address of shared memory location, where the page fault counter
	 * value will be written, inside the FW data memory.
	 */
	kbase_csf_update_firmware_memory(
		kbdev, kbdev->csf.page_fault_cnt_ptr_address,
		(u32)((kbdev->csf.firmware_trace_buffers.mcu_rw.va_reg->start_pfn << PAGE_SHIFT) +
		      PAGE_SIZE - sizeof(u32)));

	*kbdev->csf.page_fault_cnt_ptr = kbdev->csf.page_fault_cnt = 0;
}

static void init_page_fault_cnt_firmware_memory(struct kbase_device *kbdev)
{
	if (!kbdev->csf.page_fault_cnt_ptr_address)
		return;

	if (WARN_ON_ONCE(!kbdev->csf.firmware_trace_buffers.mcu_rw.va_reg))
		return;

	/* Save the CPU address of shared memory location where the page fault counter
	 * value will be written.
	 * The shared memory location comes from the last 4 bytes of the page that
	 * is allocated to maintain the extract offset value for different trace
	 * buffers. Only the first 4 bytes of every cacheline is used for the extract offset
	 * value.
	 */
	kbdev->csf.page_fault_cnt_ptr =
		(u32 *)((u8 *)kbdev->csf.firmware_trace_buffers.mcu_rw.cpu_addr + PAGE_SIZE -
			sizeof(u32));
	reinit_page_fault_cnt_firmware_memory(kbdev);
}

/**
 * set_iterator_trace_enable - Set the value for 'kbase_iter_trace_enable' global variable
 *                             according to the value of GLB_FEATURES.ITER_TRACE_SUPPORTED bit,
 *                             and the corresponding device tree entry.
 * @kbdev: Kernel base device pointer
 */
static void set_iterator_trace_enable(struct kbase_device *kbdev)
{
	const struct kbase_csf_global_iface *iface = &kbdev->csf.global_iface;
	bool dev_support_iter_trace = iface->features & GLB_FEATURES_ITER_TRACE_SUPPORTED_MASK;
	const void *dt_iter_trace_param;
	unsigned int val;

	if (!dev_support_iter_trace) {
		kbase_iter_trace_enable = false;
		return;
	}


	/* check device tree for iterator trace enable property and
	 * fallback to "iter_trace_enable" if not found and try again
	 */
	dt_iter_trace_param = of_get_property(kbdev->dev->of_node, "iter-trace-enable", NULL);

	if (!dt_iter_trace_param)
		dt_iter_trace_param =
			of_get_property(kbdev->dev->of_node, "iter_trace_enable", NULL);

	val = (dt_iter_trace_param) ? be32_to_cpup(dt_iter_trace_param) : 0;
	dev_dbg(kbdev->dev, "Iterator trace enable device-tree config value: %u", val);

	kbase_iter_trace_enable = val ? true : false;
}

static void iterator_trace_reinit(struct kbase_device *kbdev)
{
	if (kbase_iter_trace_enable) {
		unsigned long fw_io_flags;

		/* This can be called during a GPU resume procedure, so we need to
		 * write to FW pages regardless of the FW responsiveness.
		 */
		kbase_csf_fw_io_open_force(&kbdev->csf.fw_io, &fw_io_flags);
		kbase_csf_fw_io_global_write_mask(&kbdev->csf.fw_io, GLB_REQ,
						  GLB_REQ_ITER_TRACE_ENABLE_MASK,
						  GLB_REQ_ITER_TRACE_ENABLE_MASK);
		kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
	}
}

static void iterator_trace_init(struct kbase_device *kbdev)
{
	set_iterator_trace_enable(kbdev);
	iterator_trace_reinit(kbdev);
}

/* The shared interface area, used for communicating with firmware, is managed
 * like a virtual memory zone. Reserve the virtual space from that zone
 * corresponding to shared interface entry parsed from the firmware image.
 * The MCU_SHARED_ZONE should have been initialized before calling this
 * function.
 */
static int setup_shared_iface_static_region(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_interface *interface = kbdev->csf.shared_interface;
	struct kbase_va_region *reg;
	int ret = -ENOMEM;

	if (!interface)
		return -EINVAL;

	reg = kbase_alloc_free_region(&kbdev->csf.mcu_shared_zone, 0, interface->num_pages_aligned);
	if (reg) {
		mutex_lock(&kbdev->csf.reg_lock);
		ret = kbase_add_va_region_rbtree(kbdev, reg, interface->virtual,
						 interface->num_pages_aligned, 1);
		mutex_unlock(&kbdev->csf.reg_lock);
		if (ret)
			kfree(reg);
		else
			reg->flags &= ~KBASE_REG_FREE;
	}

	return ret;
}

static u32 get_global_ack_state(struct kbase_csf_fw_io *fw_io)
{
	u32 const glb_ack = kbase_csf_fw_io_global_read(fw_io, GLB_ACK);

	return GLB_ACK_STATE_GET(glb_ack);
}

#if IS_ENABLED(CONFIG_MALI_DEBUG)
static u32 get_global_req_state(struct kbase_csf_fw_io *fw_io)
{
	u32 const glb_req = kbase_csf_fw_io_global_input_read(fw_io, GLB_REQ);

	return GLB_REQ_STATE_GET(glb_req);
}
#endif

static void check_active_state(struct kbase_csf_fw_io *fw_io, u32 new_glb_req_state)
{
#if IS_ENABLED(CONFIG_MALI_DEBUG)
	struct kbase_device *const kbdev = fw_io->kbdev;
	u32 const glb_req_state = get_global_req_state(fw_io);
	u32 const glb_ack_state = get_global_ack_state(fw_io);

	if (glb_req_state != GLB_REQ_STATE_ACTIVE)
		dev_warn(kbdev->dev, "GLB_REQ_STATE (%u) not as active when changing state to %u",
			 glb_req_state, new_glb_req_state);

	if (glb_ack_state != GLB_ACK_STATE_ACTIVE)
		dev_warn(kbdev->dev, "GLB_ACK_STATE (%u) not as active when changing state to %u",
			 glb_ack_state, new_glb_req_state);
#endif
}

static void set_global_req_state(struct kbase_csf_fw_io *fw_io, u32 const glb_req_state)
{
	struct kbase_device *const kbdev = fw_io->kbdev;
	u32 glb_req;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_fw_io_assert_opened(fw_io);

	glb_req = kbase_csf_fw_io_global_input_read(fw_io, GLB_REQ);
	glb_req = GLB_REQ_STATE_SET(glb_req, glb_req_state);
	kbase_csf_fw_io_global_write(fw_io, GLB_REQ, glb_req);
}

static void set_global_req_state_as_sleep(struct kbase_csf_fw_io *fw_io)
{
	struct kbase_device *const kbdev = fw_io->kbdev;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_fw_io_assert_opened(fw_io);

	check_active_state(fw_io, GLB_REQ_STATE_SLEEP);
	set_global_req_state(fw_io, GLB_REQ_STATE_SLEEP);
}

static void set_global_req_state_as_halt(struct kbase_csf_fw_io *fw_io)
{
	struct kbase_device *const kbdev = fw_io->kbdev;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_fw_io_assert_opened(fw_io);

	check_active_state(fw_io, GLB_REQ_STATE_HALT);
	set_global_req_state(fw_io, GLB_REQ_STATE_HALT);
}


void kbase_csf_firmware_disable_mcu(struct kbase_device *kbdev)
{
	KBASE_TLSTREAM_TL_KBASE_CSFFW_FW_DISABLING(kbdev, kbase_backend_get_cycle_cnt(kbdev));

	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(MCU_CONTROL), MCU_CONTROL_REQ_DISABLE);
}

void kbase_csf_firmware_disable_mcu_wait(struct kbase_device *kbdev)
{
	u32 val;
	const u32 timeout_us =
		kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_STOP_TIMEOUT) * USEC_PER_MSEC;
	const int err = kbase_reg_poll32_timeout(kbdev, GPU_CONTROL_ENUM(MCU_STATUS), val,
						 val == MCU_CONTROL_REQ_DISABLE, 0, timeout_us,
						 false);

	if (err)
		dev_err(kbdev->dev, "Firmware failed to stop, error no: %d", err);

	KBASE_TLSTREAM_TL_KBASE_CSFFW_FW_OFF(kbdev, kbase_backend_get_cycle_cnt(kbdev));
}

void kbase_csf_stop_firmware_and_wait(struct kbase_device *kbdev)
{
	/* Stop the MCU firmware */
	kbase_csf_firmware_disable_mcu(kbdev);

	kbase_csf_firmware_disable_mcu_wait(kbdev);
}

static void wait_for_firmware_boot(struct kbase_device *kbdev)
{
	long wait_timeout;
	long remaining;

	if (!csf_firmware_boot_timeout_ms)
		csf_firmware_boot_timeout_ms =
			kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_BOOT_TIMEOUT);

	wait_timeout = kbase_csf_timeout_in_jiffies(csf_firmware_boot_timeout_ms);

	/* Firmware will generate a global interface interrupt once booting
	 * is complete.
	 * Ignore FW I/O status by using wait_event_timeout() directly,
	 * instead of kbase_csf_fw_io_wait_event_timeout().
	 *
	 * Once FW boot is started, FW notifies host when successful
	 * before any servicing of YIELD_NOW.
	 */
	remaining = wait_event_timeout(kbdev->csf.event_wait, kbdev->csf.interrupt_received == true,
				       wait_timeout);
	if (!remaining)
		dev_err(kbdev->dev, "Timed out waiting for fw boot completion");
	kbdev->csf.interrupt_received = false;
}

static void enable_mcu(struct kbase_device *kbdev)
{
	/* Trigger the boot of MCU firmware, Use the AUTO mode as
	 * otherwise on fast reset, to exit protected mode, MCU will
	 * not reboot by itself to enter normal mode.
	 */
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(MCU_CONTROL), MCU_CONTROL_REQ_AUTO);
}

static void boot_csf_firmware(struct kbase_device *kbdev)
{
	enable_mcu(kbdev);

	wait_for_firmware_boot(kbdev);
}

/**
 * wait_ready() - Wait for previously issued MMU command to complete.
 *
 * @kbdev:        Kbase device to wait for a MMU command to complete.
 *
 * Reset GPU if the wait for previously issued command times out.
 *
 * Return:  0 on success, error code otherwise.
 */
static int wait_ready(struct kbase_device *kbdev)
{
	u32 val;
	const u32 timeout_us =
		kbase_get_timeout_ms(kbdev, MMU_AS_INACTIVE_WAIT_TIMEOUT) * USEC_PER_MSEC;
	const int err = kbase_reg_poll32_timeout(kbdev, MMU_AS_OFFSET(MCU_AS_NR, STATUS), val,
						 !(val & AS_STATUS_AS_ACTIVE_EXT_MASK), 0,
						 timeout_us, false);

	if (!err)
		return 0;

	dev_err(kbdev->dev,
		"AS_ACTIVE bit stuck for MCU AS. Might be caused by unstable GPU clk/pwr or faulty system");

	if (kbase_prepare_to_reset_gpu_locked(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
		kbase_reset_gpu_locked(kbdev);

	return -ETIMEDOUT;
}

static void unload_mmu_tables(struct kbase_device *kbdev)
{
	unsigned long irq_flags;

	mutex_lock(&kbdev->mmu_hw_mutex);
	spin_lock_irqsave(&kbdev->hwaccess_lock, irq_flags);
	if (kbase_io_is_gpu_powered(kbdev))
		kbase_mmu_disable_as(kbdev, MCU_AS_NR);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, irq_flags);
	mutex_unlock(&kbdev->mmu_hw_mutex);
}

static int load_mmu_tables(struct kbase_device *kbdev)
{
	unsigned long irq_flags;

	mutex_lock(&kbdev->mmu_hw_mutex);
	spin_lock_irqsave(&kbdev->hwaccess_lock, irq_flags);
	kbase_mmu_update(kbdev, &kbdev->csf.mcu_mmu, MCU_AS_NR);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, irq_flags);
	mutex_unlock(&kbdev->mmu_hw_mutex);

	/* Wait for a while for the update command to take effect */
	return wait_ready(kbdev);
}

/**
 * convert_mem_flags() - Convert firmware memory flags to GPU region flags
 *
 * Return: GPU memory region flags
 *
 * @kbdev: Instance of GPU platform device (used to determine system coherency)
 * @flags: Flags of an "interface memory setup" section in a firmware image
 * @cm:    appropriate cache mode chosen for the "interface memory setup"
 *         section, which could be different from the cache mode requested by
 *         firmware.
 */
static unsigned long convert_mem_flags(const struct kbase_device *const kbdev, const u32 flags,
				       u32 *cm)
{
	unsigned long mem_flags = 0;
	u32 cache_mode = flags & CSF_FIRMWARE_ENTRY_CACHE_MODE;
	bool is_shared = (flags & CSF_FIRMWARE_ENTRY_SHARED) ? true : false;

	/* The memory flags control the access permissions for the MCU, the
	 * shader cores/tiler are not expected to access this memory
	 */
	if (flags & CSF_FIRMWARE_ENTRY_READ)
		mem_flags |= KBASE_REG_GPU_RD;

	if (flags & CSF_FIRMWARE_ENTRY_WRITE)
		mem_flags |= KBASE_REG_GPU_WR;

	if ((flags & CSF_FIRMWARE_ENTRY_EXECUTE) == 0)
		mem_flags |= KBASE_REG_GPU_NX;

	if (flags & CSF_FIRMWARE_ENTRY_PROTECTED)
		mem_flags |= KBASE_REG_PROTECTED;

	/* Substitute uncached coherent memory for cached coherent memory if
	 * the system does not support ACE coherency.
	 */
	if ((cache_mode == CSF_FIRMWARE_CACHE_MODE_CACHED_COHERENT) &&
	    (kbdev->system_coherency != COHERENCY_ACE))
		cache_mode = CSF_FIRMWARE_CACHE_MODE_UNCACHED_COHERENT;

	/* Substitute uncached incoherent memory for uncached coherent memory
	 * if the system does not support ACE-Lite coherency.
	 */
	if ((cache_mode == CSF_FIRMWARE_CACHE_MODE_UNCACHED_COHERENT) &&
	    (kbdev->system_coherency == COHERENCY_NONE))
		cache_mode = CSF_FIRMWARE_CACHE_MODE_NONE;

	*cm = cache_mode;

	switch (cache_mode) {
	case CSF_FIRMWARE_CACHE_MODE_NONE:
		mem_flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_NON_CACHEABLE);
		break;
	case CSF_FIRMWARE_CACHE_MODE_CACHED:
		mem_flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_IMPL_DEF_CACHE_POLICY);
		break;
	case CSF_FIRMWARE_CACHE_MODE_UNCACHED_COHERENT:
	case CSF_FIRMWARE_CACHE_MODE_CACHED_COHERENT:
		WARN_ON(!is_shared);
		mem_flags |= KBASE_REG_SHARE_BOTH |
			     KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_SHARED);
		break;
	default:
		dev_err(kbdev->dev, "Firmware contains interface with unsupported cache mode\n");
		break;
	}
	return mem_flags;
}

static void load_fw_image_section(struct kbase_device *kbdev, const u8 *data,
				  struct tagged_addr *phys, u32 num_pages, u32 flags,
				  u32 data_start, u32 data_end)
{
	u32 data_pos = data_start;
	u32 data_len = data_end - data_start;
	u32 page_num;
	u32 page_limit;

	if (flags & CSF_FIRMWARE_ENTRY_ZERO)
		page_limit = num_pages;
	else
		page_limit = (data_len + PAGE_SIZE - 1) / PAGE_SIZE;

	for (page_num = 0; page_num < page_limit; ++page_num) {
		struct page *const page = as_page(phys[page_num]);
		char *const p = kbase_kmap_atomic(page);
		u32 const copy_len = min_t(u32, PAGE_SIZE, data_len);

		if (copy_len > 0) {
			memcpy(p, data + data_pos, copy_len);
			data_pos += copy_len;
			data_len -= copy_len;
		}

		if (flags & CSF_FIRMWARE_ENTRY_ZERO) {
			u32 const zi_len = PAGE_SIZE - copy_len;

			memset(p + copy_len, 0, zi_len);
		}

		kbase_sync_single_for_device(kbdev, kbase_dma_addr_from_tagged(phys[page_num]),
					     PAGE_SIZE, DMA_TO_DEVICE);
		kbase_kunmap_atomic(p);
	}
}

static int reload_fw_image(struct kbase_device *kbdev)
{
	const u32 magic = FIRMWARE_HEADER_MAGIC;
	struct kbase_csf_firmware_interface *interface;
	struct kbase_csf_mcu_fw *const mcu_fw = &kbdev->csf.fw;
	int ret = 0;

	if (WARN_ON(mcu_fw->data == NULL)) {
		dev_err(kbdev->dev, "Firmware image copy not loaded\n");
		ret = -EINVAL;
		goto out;
	}

	/* Do a basic sanity check on MAGIC signature */
	if (memcmp(mcu_fw->data, &magic, sizeof(magic)) != 0) {
		dev_err(kbdev->dev,
			"Incorrect magic value, firmware image could have been corrupted\n");
		ret = -EINVAL;
		goto out;
	}

	list_for_each_entry(interface, &kbdev->csf.firmware_interfaces, node) {
		/* Dont skip re-loading any section if full reload was requested */
		if (!kbdev->csf.firmware_full_reload_needed) {
			/* Skip reload of text & read only data sections */
			if ((interface->flags & CSF_FIRMWARE_ENTRY_EXECUTE) ||
			    !(interface->flags & CSF_FIRMWARE_ENTRY_WRITE))
				continue;
		}

		load_fw_image_section(kbdev, mcu_fw->data, interface->phys, interface->num_pages,
				      interface->flags, interface->data_start, interface->data_end);
	}

	kbdev->csf.firmware_full_reload_needed = false;

	kbase_csf_firmware_reload_trace_buffers_data(kbdev);
	reinit_page_fault_cnt_firmware_memory(kbdev);
	iterator_trace_reinit(kbdev);
out:
	return ret;
}

/**
 * entry_find_large_page_to_reuse() - Find if the large page of previously parsed
 *                                    FW interface entry can be reused to store
 *                                    the contents of new FW interface entry.
 *
 * @kbdev: Kbase device structure
 * @virtual_start: Start of the virtual address range required for an entry allocation
 * @virtual_end: End of the virtual address range required for an entry allocation
 * @flags: Firmware entry flags for comparison with the reusable pages found
 * @phys: Pointer to the array of physical (tagged) addresses making up the new
 *        FW interface entry. It is an output parameter which would be made to
 *        point to an already existing array allocated for the previously parsed
 *        FW interface entry using large page(s). If no appropriate entry is
 *        found it is set to NULL.
 * @pma:  Pointer to a protected memory allocation. It is an output parameter
 *        which would be made to the protected memory allocation of a previously
 *        parsed FW interface entry using large page(s) from protected memory.
 *        If no appropriate entry is found it is set to NULL.
 * @num_pages: Number of pages requested.
 * @num_pages_aligned: This is an output parameter used to carry the number of small pages
 *                     within the 2MB pages aligned allocation.
 * @is_small_page: This is an output flag used to select between the small and large page
 *                 to be used for the FW entry allocation.
 * @force_small_page: Use small pages to allocate memory needed for FW loading
 *
 * Go through all the already initialized interfaces and find if a previously
 * allocated large page can be used to store contents of new FW interface entry.
 *
 * Return: true if a large page can be reused, false otherwise.
 */
static inline bool entry_find_large_page_to_reuse(struct kbase_device *kbdev,
						  const u32 virtual_start, const u32 virtual_end,
						  const u32 flags, struct tagged_addr **phys,
						  struct protected_memory_allocation ***pma,
						  u32 num_pages, u32 *num_pages_aligned,
						  bool *is_small_page, bool force_small_page)
{
	struct kbase_csf_firmware_interface *interface = NULL;
	struct kbase_csf_firmware_interface *target_interface = NULL;
	u32 virtual_diff_min = U32_MAX;
	bool reuse_large_page = false;

	CSTD_UNUSED(interface);
	CSTD_UNUSED(target_interface);
	CSTD_UNUSED(virtual_diff_min);

	*num_pages_aligned = num_pages;
	*is_small_page = true;
	*phys = NULL;
	*pma = NULL;

	if (force_small_page)
		goto out;

	/* If the section starts at 2MB aligned boundary,
	 * then use 2MB page(s) for it.
	 */
	if (!(virtual_start & (SZ_2M - 1))) {
		*num_pages_aligned = round_up(*num_pages_aligned, NUM_PAGES_IN_2MB_LARGE_PAGE);
		*is_small_page = false;
		goto out;
	}

	/* If the section doesn't lie within the same 2MB aligned boundary,
	 * then use small pages as it would be complicated to use a 2MB page
	 * for such section.
	 */
	if ((virtual_start & ~(SZ_2M - 1UL)) != (virtual_end & ~(SZ_2M - 1UL)))
		goto out;

	/* Find the nearest 2MB aligned section which comes before the current
	 * section.
	 */
	list_for_each_entry(interface, &kbdev->csf.firmware_interfaces, node) {
		const u32 virtual_diff = virtual_start - interface->virtual;

		if (interface->virtual > virtual_end)
			continue;

		if (interface->virtual & (SZ_2M - 1))
			continue;

		if ((virtual_diff < virtual_diff_min) && (interface->flags == flags)) {
			target_interface = interface;
			virtual_diff_min = virtual_diff;
		}
	}

	if (target_interface) {
		const u32 page_index = virtual_diff_min >> PAGE_SHIFT;

		if (page_index >= target_interface->num_pages_aligned)
			goto out;

		if (target_interface->phys)
			*phys = &target_interface->phys[page_index];

		if (target_interface->pma)
			*pma = &target_interface->pma[page_index / NUM_PAGES_IN_2MB_LARGE_PAGE];

		*is_small_page = false;
		reuse_large_page = true;
	}

out:
	return reuse_large_page;
}

/**
 * parse_memory_setup_entry() - Process an "interface memory setup" section
 *
 * @kbdev: Kbase device structure
 * @fw: The firmware image containing the section
 * @entry: Pointer to the start of the section
 * @size: Size (in bytes) of the section
 *
 * Read an "interface memory setup" section from the firmware image and create
 * the necessary memory region including the MMU page tables. If successful
 * the interface will be added to the kbase_device:csf.firmware_interfaces list.
 *
 * Return: 0 if successful, negative error code on failure
 */
static int parse_memory_setup_entry(struct kbase_device *kbdev,
				    const struct kbase_csf_mcu_fw *const fw, const u32 *entry,
				    unsigned int size)
{
	int ret = 0;
	const u32 flags = entry[0];
	const u32 virtual_start = entry[1];
	const u32 virtual_end = entry[2];
	const u32 data_start = entry[3];
	const u32 data_end = entry[4];
	u32 num_pages;
	u32 num_pages_aligned;
	char *name;
	void *name_entry;
	unsigned int name_len;
	struct tagged_addr *phys = NULL;
	struct kbase_csf_firmware_interface *interface = NULL;
	bool protected_mode = false;
	unsigned long mem_flags = 0;
	u32 cache_mode = 0;
	struct protected_memory_allocation **pma = NULL;
	bool reuse_pages = false;
	bool is_small_page = true;
	bool force_small_page = false;

	if (data_end < data_start) {
		dev_err(kbdev->dev, "Firmware corrupt, data_end < data_start (0x%x<0x%x)\n",
			data_end, data_start);
		return -EINVAL;
	}
	if (virtual_end < virtual_start) {
		dev_err(kbdev->dev, "Firmware corrupt, virtual_end < virtual_start (0x%x<0x%x)\n",
			virtual_end, virtual_start);
		return -EINVAL;
	}
	if (data_end > fw->size) {
		dev_err(kbdev->dev,
			"Firmware corrupt, file truncated? data_end=0x%x > fw->size=0x%zx\n",
			data_end, fw->size);
		return -EINVAL;
	}

	if ((virtual_start & ~PAGE_MASK) != 0 || (virtual_end & ~PAGE_MASK) != 0) {
		dev_err(kbdev->dev,
			"Firmware corrupt: virtual addresses not page aligned: 0x%x-0x%x\n",
			virtual_start, virtual_end);
		return -EINVAL;
	}

	if ((flags & CSF_FIRMWARE_ENTRY_SUPPORTED_FLAGS) != flags) {
		dev_err(kbdev->dev, "Firmware contains interface with unsupported flags (0x%x)\n",
			flags);
		return -EINVAL;
	}

	if (flags & CSF_FIRMWARE_ENTRY_PROTECTED)
		protected_mode = true;

	if (protected_mode && kbdev->csf.pma_dev == NULL) {
		dev_err(kbdev->dev,
			"Protected memory allocator not found, Firmware protected mode entry will not be supported");
		return 0;
	}

	num_pages = (virtual_end - virtual_start) >> PAGE_SHIFT;

retry_alloc:
	ret = 0;
	reuse_pages = entry_find_large_page_to_reuse(kbdev, virtual_start, virtual_end, flags,
						     &phys, &pma, num_pages, &num_pages_aligned,
						     &is_small_page, force_small_page);
	if (!reuse_pages)
		phys = kmalloc_array(num_pages_aligned, sizeof(*phys), GFP_KERNEL);

	if (!phys)
		return -ENOMEM;

	if (protected_mode) {
		if (!reuse_pages) {
			pma = kbase_csf_protected_memory_alloc(kbdev, phys, num_pages_aligned,
							       is_small_page);
			if (!pma)
				ret = -ENOMEM;
		} else if (WARN_ON(!pma)) {
			ret = -EINVAL;
			goto out;
		}
	} else {
		if (!reuse_pages) {
			ret = kbase_mem_pool_alloc_pages(
				kbase_mem_pool_group_select(kbdev, KBASE_MEM_GROUP_CSF_FW,
							    is_small_page),
				num_pages_aligned, phys, false, NULL);
		}
	}

	if (ret < 0) {
		dev_warn(
			kbdev->dev,
			"Failed to allocate %u physical pages for the firmware interface entry at VA 0x%x using %s ",
			num_pages_aligned, virtual_start,
			is_small_page ? "small pages" : "large page");
		WARN_ON(reuse_pages);
		if (!is_small_page) {
			dev_warn(kbdev->dev, "Retrying by using small pages");
			force_small_page = true;
			kfree(phys);
			goto retry_alloc;
		}
		goto out;
	}

	load_fw_image_section(kbdev, fw->data, phys, num_pages, flags, data_start, data_end);

	/* Allocate enough memory for the struct kbase_csf_firmware_interface and
	 * the name of the interface.
	 */
	name_entry = (void *)entry + INTERFACE_ENTRY_NAME_OFFSET;
	name_len = strnlen(name_entry, size - INTERFACE_ENTRY_NAME_OFFSET);
	if (size < (INTERFACE_ENTRY_NAME_OFFSET + name_len + 1 + sizeof(u32))) {
		dev_err(kbdev->dev, "Memory setup entry too short to contain virtual_exe_start");
		ret = -EINVAL;
		goto out;
	}

	interface = kmalloc(sizeof(*interface) + name_len + 1, GFP_KERNEL);
	if (!interface) {
		ret = -ENOMEM;
		goto out;
	}
	name = (void *)(interface + 1);
	memcpy(name, name_entry, name_len);
	name[name_len] = 0;

	interface->name = name;
	interface->phys = phys;
	interface->reuse_pages = reuse_pages;
	interface->is_small_page = is_small_page;
	interface->num_pages = num_pages;
	interface->num_pages_aligned = num_pages_aligned;
	interface->virtual = virtual_start;
	interface->kernel_map = NULL;
	interface->flags = flags;
	interface->data_start = data_start;
	interface->data_end = data_end;
	interface->pma = pma;

	/* Discover the virtual execution address field after the end of the name
	 * field taking into account the NULL-termination character.
	 */
	interface->virtual_exe_start = *((u32 *)(name_entry + name_len + 1));

	mem_flags = convert_mem_flags(kbdev, flags, &cache_mode);

	if (flags & CSF_FIRMWARE_ENTRY_SHARED) {
		struct page **page_list;
		u32 i;
		pgprot_t cpu_map_prot;
		u32 mem_attr_index = KBASE_REG_MEMATTR_VALUE(mem_flags);

		/* Since SHARED memory type was used for mapping shared memory
		 * on GPU side, it can be mapped as cached on CPU side on both
		 * types of coherent platforms.
		 */
		if ((cache_mode == CSF_FIRMWARE_CACHE_MODE_CACHED_COHERENT) ||
		    (cache_mode == CSF_FIRMWARE_CACHE_MODE_UNCACHED_COHERENT)) {
			WARN_ON(mem_attr_index != KBASE_MEMATTR_INDEX_SHARED);
			cpu_map_prot = PAGE_KERNEL;
		} else {
			WARN_ON(mem_attr_index != KBASE_MEMATTR_INDEX_NON_CACHEABLE);
			cpu_map_prot = pgprot_writecombine(PAGE_KERNEL);
		}

		page_list = kmalloc_array(num_pages, sizeof(*page_list), GFP_KERNEL);
		if (!page_list) {
			ret = -ENOMEM;
			goto out;
		}

		for (i = 0; i < num_pages; i++)
			page_list[i] = as_page(phys[i]);

		interface->kernel_map = vmap(page_list, num_pages, VM_MAP, cpu_map_prot);

		kfree(page_list);

		if (!interface->kernel_map) {
			ret = -ENOMEM;
			goto out;
		}
	}

	/* Start location of the shared interface area is fixed and is
	 * specified in firmware spec, and so there shall only be a
	 * single entry with that start address.
	 */
	if (virtual_start == (KBASE_REG_ZONE_MCU_SHARED_BASE << PAGE_SHIFT))
		kbdev->csf.shared_interface = interface;

	list_add(&interface->node, &kbdev->csf.firmware_interfaces);

	if (!reuse_pages) {
		ret = kbase_mmu_insert_pages_no_flush(kbdev, &kbdev->csf.mcu_mmu,
						      virtual_start >> PAGE_SHIFT, phys,
						      num_pages_aligned, mem_flags,
						      KBASE_MEM_GROUP_CSF_FW, NULL, NULL);

		if (ret != 0) {
			dev_err(kbdev->dev, "Failed to insert firmware pages\n");
			/* The interface has been added to the list, so cleanup will
			 * be handled by firmware unloading
			 */
		}
	}

	dev_dbg(kbdev->dev, "Processed section '%s'", name);

	return ret;

out:
	if (!reuse_pages && phys) {
		if (pma) {
			kbase_csf_protected_memory_free(kbdev, pma, num_pages_aligned,
							is_small_page);
		} else {
			kbase_mem_pool_free_pages(
				kbase_mem_pool_group_select(kbdev, KBASE_MEM_GROUP_CSF_FW,
							    is_small_page),
				num_pages_aligned, phys, false, false);
		}
		kfree(phys);
	}

	kfree(interface);
	return ret;
}

/**
 * parse_timeline_metadata_entry() - Process a "timeline metadata" section
 *
 * Return: 0 if successful, negative error code on failure
 *
 * @kbdev: Kbase device structure
 * @fw:    Firmware image containing the section
 * @entry: Pointer to the section
 * @size:  Size (in bytes) of the section
 */
static int parse_timeline_metadata_entry(struct kbase_device *kbdev,
					 const struct kbase_csf_mcu_fw *const fw, const u32 *entry,
					 unsigned int size)
{
	const u32 data_start = entry[0];
	const u32 data_size = entry[1];
	const u32 data_end = data_start + data_size;
	const char *name = (char *)&entry[2];
	struct firmware_timeline_metadata *metadata;
	const unsigned int name_len = size - TL_METADATA_ENTRY_NAME_OFFSET;
	size_t allocation_size = sizeof(*metadata) + name_len + 1 + data_size;

	if (data_end > fw->size) {
		dev_err(kbdev->dev,
			"Firmware corrupt, file truncated? data_end=0x%x > fw->size=0x%zx",
			data_end, fw->size);
		return -EINVAL;
	}

	/* Allocate enough space for firmware_timeline_metadata,
	 * its name and the content.
	 */
	metadata = kmalloc(allocation_size, GFP_KERNEL);
	if (!metadata)
		return -ENOMEM;

	metadata->name = (char *)(metadata + 1);
	metadata->data = (char *)(metadata + 1) + name_len + 1;
	metadata->size = data_size;

	memcpy(metadata->name, name, name_len);
	metadata->name[name_len] = 0;

	/* Copy metadata's content. */
	memcpy(metadata->data, fw->data + data_start, data_size);

	list_add(&metadata->node, &kbdev->csf.firmware_timeline_metadata);

	dev_dbg(kbdev->dev, "Timeline metadata '%s'", metadata->name);

	return 0;
}

/**
 * parse_build_info_metadata_entry() - Process a "build info metadata" section
 * @kbdev: Kbase device structure
 * @fw:    Firmware image containing the section
 * @entry: Pointer to the section
 * @size:  Size (in bytes) of the section
 *
 * This prints the git SHA of the firmware on frimware load.
 *
 * Return: 0 if successful, negative error code on failure
 */
static int parse_build_info_metadata_entry(struct kbase_device *kbdev,
					   const struct kbase_csf_mcu_fw *const fw,
					   const u32 *entry, unsigned int size)
{
	const u32 meta_start_addr = entry[0];
	char *ptr = NULL;
	size_t sha_pattern_len = strlen(BUILD_INFO_GIT_SHA_PATTERN);
	CSTD_UNUSED(size);

	/* Only print git SHA to avoid releasing sensitive information */
	ptr = strstr(fw->data + meta_start_addr, BUILD_INFO_GIT_SHA_PATTERN);
	/* Check that we won't overrun the found string  */
	if (ptr &&
	    strlen(ptr) >= BUILD_INFO_GIT_SHA_LEN + BUILD_INFO_GIT_DIRTY_LEN + sha_pattern_len) {
		char git_sha[BUILD_INFO_GIT_SHA_LEN + BUILD_INFO_GIT_DIRTY_LEN + 1];
		unsigned int i = 0;

		/* Move ptr to start of SHA */
		ptr += sha_pattern_len;
		for (i = 0; i < BUILD_INFO_GIT_SHA_LEN; i++) {
			/* Ensure that the SHA is made up of hex digits */
			if (!isxdigit(ptr[i]))
				break;

			git_sha[i] = ptr[i];
		}

		/* Check if the next char indicates git SHA is dirty */
		if (ptr[i] == ' ' || ptr[i] == '+') {
			git_sha[i] = ptr[i];
			i++;
		}
		git_sha[i] = '\0';

		dev_info(kbdev->dev, "Mali firmware git_sha: %s\n", git_sha);
	} else
		dev_info(kbdev->dev, "Mali firmware git_sha not found or invalid\n");

	return 0;
}

/**
 * load_firmware_entry() - Process an entry from a firmware image
 *
 * @kbdev:  Kbase device
 * @fw:     Firmware image containing the entry
 * @offset: Byte offset within the image of the entry to load
 * @header: Header word of the entry
 *
 * Read an entry from a firmware image and do any necessary work (e.g. loading
 * the data into page accessible to the MCU).
 *
 * Unknown entries are ignored if the 'optional' flag is set within the entry,
 * otherwise the function will fail with -EINVAL
 *
 * Return: 0 if successful, negative error code on failure
 */
static int load_firmware_entry(struct kbase_device *kbdev, const struct kbase_csf_mcu_fw *const fw,
			       u32 offset, u32 header)
{
	const unsigned int type = entry_type(header);
	unsigned int size = entry_size(header);
	const bool optional = entry_optional(header);
	/* Update is used with configuration and tracebuffer entries to
	 * initiate a FIRMWARE_CONFIG_UPDATE, instead of triggering a
	 * silent reset.
	 */
	const bool updatable = entry_update(header);
	const u32 *entry = (void *)(fw->data + offset);

	if ((offset % sizeof(*entry)) || (size % sizeof(*entry))) {
		dev_err(kbdev->dev, "Firmware entry isn't 32 bit aligned, offset=0x%x size=0x%x\n",
			offset, size);
		return -EINVAL;
	}

	if (size < sizeof(*entry)) {
		dev_err(kbdev->dev, "Size field too small: %u\n", size);
		return -EINVAL;
	}

	/* Remove the header */
	entry++;
	size -= sizeof(*entry);

	switch (type) {
	case CSF_FIRMWARE_ENTRY_TYPE_INTERFACE:
		/* Interface memory setup */
		if (size < INTERFACE_ENTRY_NAME_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Interface memory setup entry too short (size=%u)\n",
				size);
			return -EINVAL;
		}
		return parse_memory_setup_entry(kbdev, fw, entry, size);
	case CSF_FIRMWARE_ENTRY_TYPE_CONFIGURATION:
		/* Configuration option */
		if (size < CONFIGURATION_ENTRY_NAME_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Configuration option entry too short (size=%u)\n",
				size);
			return -EINVAL;
		}
		return kbase_csf_firmware_cfg_option_entry_parse(kbdev, fw, entry, size, updatable);
	case CSF_FIRMWARE_ENTRY_TYPE_TRACE_BUFFER:
		/* Trace buffer */
		if (size < TRACE_BUFFER_ENTRY_NAME_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Trace Buffer entry too short (size=%u)\n", size);
			return -EINVAL;
		}
		return kbase_csf_firmware_parse_trace_buffer_entry(kbdev, entry, size, updatable);
	case CSF_FIRMWARE_ENTRY_TYPE_TIMELINE_METADATA:
		/* Meta data section */
		if (size < TL_METADATA_ENTRY_NAME_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Timeline metadata entry too short (size=%u)\n", size);
			return -EINVAL;
		}
		return parse_timeline_metadata_entry(kbdev, fw, entry, size);
	case CSF_FIRMWARE_ENTRY_TYPE_BUILD_INFO_METADATA:
		if (size < BUILD_INFO_METADATA_SIZE_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Build info metadata entry too short (size=%u)\n",
				size);
			return -EINVAL;
		}
		return parse_build_info_metadata_entry(kbdev, fw, entry, size);
	case CSF_FIRMWARE_ENTRY_TYPE_FUNC_CALL_LIST:
		/* Function call list section */
		if (size < FUNC_CALL_LIST_ENTRY_NAME_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "Function call list entry too short (size=%u)\n", size);
			return -EINVAL;
		}
		kbase_csf_firmware_log_parse_logging_call_list_entry(kbdev, entry);
		return 0;
	case CSF_FIRMWARE_ENTRY_TYPE_PAGE_FAULT_CNT:
		/* Entry about the location of page fault counter */
		if (size < sizeof(*entry)) {
			dev_err(kbdev->dev, "Page fault counter entry too short (size=%u)", size);
			return -EINVAL;
		}
		kbdev->csf.page_fault_cnt_ptr_address = *entry;
		return 0;
	case CSF_FIRMWARE_ENTRY_TYPE_CORE_DUMP:
		/* Core Dump section */
		if (size < CORE_DUMP_ENTRY_START_ADDR_OFFSET + sizeof(*entry)) {
			dev_err(kbdev->dev, "FW Core dump entry too short (size=%u)\n", size);
			return -EINVAL;
		}
		return kbase_csf_firmware_core_dump_entry_parse(kbdev, entry);
	default:
		if (!optional) {
			dev_err(kbdev->dev, "Unsupported non-optional entry type %u in firmware\n",
				type);
			return -EINVAL;
		}
	}

	return 0;
}

static void free_global_iface(struct kbase_device *kbdev)
{
	struct kbase_csf_global_iface *iface = &kbdev->csf.global_iface;

	if (iface->groups) {
		unsigned int gid;

		for (gid = 0; gid < iface->group_num; ++gid)
			kfree(iface->groups[gid].streams);

		kfree(iface->groups);
		iface->groups = NULL;
	}

	kbase_csf_fw_io_pages_term(&kbdev->csf.fw_io, iface->group_num);
}

/**
 * iface_gpu_va_to_cpu - Convert a GPU VA address within the shared interface
 *                       region to a CPU address, using the existing mapping.
 * @kbdev: Device pointer
 * @gpu_va: GPU VA to convert
 *
 * Return: A CPU pointer to the location within the shared interface region, or
 *         NULL on failure.
 */
static inline void *iface_gpu_va_to_cpu(struct kbase_device *kbdev, u32 gpu_va)
{
	struct kbase_csf_firmware_interface *interface = kbdev->csf.shared_interface;
	u8 *kernel_base = interface->kernel_map;

	if (gpu_va < interface->virtual ||
	    gpu_va >= interface->virtual + interface->num_pages * PAGE_SIZE) {
		dev_err(kbdev->dev, "Interface address 0x%x not within %u-page region at 0x%x",
			gpu_va, interface->num_pages, interface->virtual);
		return NULL;
	}

	return (void *)(kernel_base + (gpu_va - interface->virtual));
}

static int parse_cmd_stream_info(struct kbase_device *kbdev,
				 struct kbase_csf_cmd_stream_info *sinfo, u32 *stream_base)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	void *stream_input, *stream_output;

	sinfo->kbdev = kbdev;
	sinfo->features = stream_base[STREAM_FEATURES / 4];

	stream_input = iface_gpu_va_to_cpu(kbdev, stream_base[STREAM_INPUT_VA / 4]);
	stream_output = iface_gpu_va_to_cpu(kbdev, stream_base[STREAM_OUTPUT_VA / 4]);
	if (!stream_input || !stream_output)
		return -ENOMEM;

	kbase_csf_fw_io_set_stream_pages(fw_io, sinfo->gid, sinfo->sid, stream_input,
					 stream_output);

	return 0;
}

static int parse_cmd_stream_group_info(struct kbase_device *kbdev,
				       struct kbase_csf_cmd_stream_group_info *ginfo,
				       u32 *group_base, u32 group_stride)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	u32 sid;
	void *group_input, *group_output;
	int err;

	ginfo->kbdev = kbdev;
	ginfo->features = group_base[GROUP_FEATURES / 4];

	group_input = iface_gpu_va_to_cpu(kbdev, group_base[GROUP_INPUT_VA / 4]);
	group_output = iface_gpu_va_to_cpu(kbdev, group_base[GROUP_OUTPUT_VA / 4]);
	if (!group_input || !group_output)
		return -ENOMEM;

	kbase_csf_fw_io_set_group_pages(fw_io, ginfo->gid, group_input, group_output);

	ginfo->suspend_size = group_base[GROUP_SUSPEND_SIZE / 4];
	ginfo->protm_suspend_size = group_base[GROUP_PROTM_SUSPEND_SIZE / 4];
	ginfo->stream_num = group_base[GROUP_STREAM_NUM / 4];

	if (ginfo->stream_num < BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MIN ||
	    ginfo->stream_num > BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MAX) {
		dev_err(kbdev->dev, "CSG with %u CSs out of range %u-%u", ginfo->stream_num,
			BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MIN, BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MAX);
		return -EINVAL;
	}

	ginfo->stream_stride = group_base[GROUP_STREAM_STRIDE / 4];

	if (ginfo->stream_num * ginfo->stream_stride > group_stride) {
		dev_err(kbdev->dev, "group stride of 0x%x exceeded by %u CSs with stride 0x%x",
			group_stride, ginfo->stream_num, ginfo->stream_stride);
		return -EINVAL;
	}

	ginfo->streams = kmalloc_array(ginfo->stream_num, sizeof(*ginfo->streams), GFP_KERNEL);

	if (!ginfo->streams)
		return -ENOMEM;

	err = kbase_csf_fw_io_streams_pages_init(fw_io, ginfo->gid, ginfo->stream_num);
	if (err)
		return err;

	for (sid = 0; sid < ginfo->stream_num; sid++) {
		u32 *stream_base = group_base + (STREAM_CONTROL_0 + ginfo->stream_stride * sid) / 4;

		ginfo->streams[sid].gid = ginfo->gid;
		ginfo->streams[sid].sid = sid;
		err = parse_cmd_stream_info(kbdev, &ginfo->streams[sid], stream_base);
		if (err < 0) {
			/* caller will free the memory for CSs array */
			return err;
		}
	}

	return 0;
}

static u32 get_firmware_version(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_interface *interface = kbdev->csf.shared_interface;
	u32 *shared_info = interface->kernel_map;

	return shared_info[GLB_VERSION / 4];
}

static int parse_capabilities(struct kbase_device *kbdev)
{
	struct kbase_csf_firmware_interface *interface = kbdev->csf.shared_interface;
	u32 *shared_info = interface->kernel_map;
	struct kbase_csf_global_iface *iface = &kbdev->csf.global_iface;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	void *global_input, *global_output;
	u32 gid;
	int err;

	/* All offsets are in bytes, so divide by 4 for access via a u32 pointer
	 */

	/* The version number of the global interface is expected to be a
	 * non-zero value. If it's not, the firmware may not have booted.
	 */
	iface->version = get_firmware_version(kbdev);
	if (!iface->version) {
		dev_err(kbdev->dev, "Version check failed. Firmware may have failed to boot.");
		return -EINVAL;
	}

	iface->kbdev = kbdev;
	iface->features = shared_info[GLB_FEATURES / 4];

	global_input = iface_gpu_va_to_cpu(kbdev, shared_info[GLB_INPUT_VA / 4]);
	global_output = iface_gpu_va_to_cpu(kbdev, shared_info[GLB_OUTPUT_VA / 4]);

	if (!global_input || !global_output)
		return -ENOMEM;

	kbase_csf_fw_io_set_global_pages(fw_io, global_input, global_output);

	iface->group_num = shared_info[GLB_GROUP_NUM / 4];

	if ((iface->group_num < BASEP_QUEUE_GROUP_MIN) ||
	    (iface->group_num > BASEP_QUEUE_GROUP_MAX)) {
		dev_err(kbdev->dev,
			"Invalid number of CSGs (%u), BASEP_QUEUE_GROUP_MIN = %u, BASEP_QUEUE_GROUP_MAX = %u",
			iface->group_num, BASEP_QUEUE_GROUP_MIN, BASEP_QUEUE_GROUP_MAX);
		return -EINVAL;
	}

	iface->group_stride = shared_info[GLB_GROUP_STRIDE / 4];
	iface->prfcnt_size = shared_info[GLB_PRFCNT_SIZE / 4];

	if (iface->version >= kbase_csf_interface_version(1, 1, 0))
		iface->instr_features = shared_info[GLB_INSTR_FEATURES / 4];
	else
		iface->instr_features = 0;

	if ((GROUP_CONTROL_0 + (unsigned long)iface->group_num * iface->group_stride) >
	    (interface->num_pages * PAGE_SIZE)) {
		dev_err(kbdev->dev,
			"interface size of %u pages exceeded by %u CSGs with stride 0x%x",
			interface->num_pages, iface->group_num, iface->group_stride);
		return -EINVAL;
	}

	WARN_ON(iface->groups);

	iface->groups = kcalloc(iface->group_num, sizeof(*iface->groups), GFP_KERNEL);
	if (!iface->groups)
		return -ENOMEM;

	err = kbase_csf_fw_io_groups_pages_init(fw_io, iface->group_num);
	if (err) {
		free_global_iface(kbdev);
		return err;
	}

	for (gid = 0; gid < iface->group_num; gid++) {
		u32 *group_base = shared_info + (GROUP_CONTROL_0 + iface->group_stride * gid) / 4;

		iface->groups[gid].gid = gid;
		err = parse_cmd_stream_group_info(kbdev, &iface->groups[gid], group_base,
						  iface->group_stride);
		if (err < 0) {
			free_global_iface(kbdev);
			return err;
		}
	}

	return 0;
}

static inline void access_firmware_memory_common(struct kbase_device *kbdev,
						 struct kbase_csf_firmware_interface *interface,
						 u32 offset_bytes, u32 *value, const bool read)
{
	u32 page_num = offset_bytes >> PAGE_SHIFT;
	u32 offset_in_page = offset_bytes & ~PAGE_MASK;
	struct page *target_page = as_page(interface->phys[page_num]);
	uintptr_t cpu_addr = (uintptr_t)kbase_kmap_atomic(target_page);
	u32 *addr = (u32 *)(cpu_addr + offset_in_page);

	if (read) {
		kbase_sync_single_for_device(kbdev,
					     kbase_dma_addr_from_tagged(interface->phys[page_num]) +
						     offset_in_page,
					     sizeof(u32), DMA_BIDIRECTIONAL);
		*value = *addr;
	} else {
		*addr = *value;
		kbase_sync_single_for_device(kbdev,
					     kbase_dma_addr_from_tagged(interface->phys[page_num]) +
						     offset_in_page,
					     sizeof(u32), DMA_BIDIRECTIONAL);
	}

	kbase_kunmap_atomic((u32 *)cpu_addr);
}

static inline void access_firmware_memory(struct kbase_device *kbdev, u32 gpu_addr, u32 *value,
					  const bool read)
{
	struct kbase_csf_firmware_interface *interface, *access_interface = NULL;
	u32 offset_bytes = 0;

	list_for_each_entry(interface, &kbdev->csf.firmware_interfaces, node) {
		if ((gpu_addr >= interface->virtual) &&
		    (gpu_addr < interface->virtual + (interface->num_pages << PAGE_SHIFT))) {
			offset_bytes = gpu_addr - interface->virtual;
			access_interface = interface;
			break;
		}
	}

	if (access_interface)
		access_firmware_memory_common(kbdev, access_interface, offset_bytes, value, read);
	else
		dev_warn(kbdev->dev, "Invalid GPU VA %x passed", gpu_addr);
}

static inline void access_firmware_memory_exe(struct kbase_device *kbdev, u32 gpu_addr, u32 *value,
					      const bool read)
{
	struct kbase_csf_firmware_interface *interface, *access_interface = NULL;
	u32 offset_bytes = 0;

	list_for_each_entry(interface, &kbdev->csf.firmware_interfaces, node) {
		if ((gpu_addr >= interface->virtual_exe_start) &&
		    (gpu_addr <
		     interface->virtual_exe_start + (interface->num_pages << PAGE_SHIFT))) {
			offset_bytes = gpu_addr - interface->virtual_exe_start;
			access_interface = interface;

			/* If there's an overlap in execution address range between a moved and a
			 * non-moved areas, always prefer the moved one. The idea is that FW may
			 * move sections around during init time, but after the layout is settled,
			 * any moved sections are going to override non-moved areas at the same
			 * location.
			 */
			if (interface->virtual_exe_start != interface->virtual)
				break;
		}
	}

	if (access_interface)
		access_firmware_memory_common(kbdev, access_interface, offset_bytes, value, read);
	else
		dev_warn(kbdev->dev, "Invalid GPU VA %x passed", gpu_addr);
}

void kbase_csf_read_firmware_memory(struct kbase_device *kbdev, u32 gpu_addr, u32 *value)
{
	access_firmware_memory(kbdev, gpu_addr, value, true);
}

void kbase_csf_update_firmware_memory(struct kbase_device *kbdev, u32 gpu_addr, u32 value)
{
	access_firmware_memory(kbdev, gpu_addr, &value, false);
}

void kbase_csf_read_firmware_memory_exe(struct kbase_device *kbdev, u32 gpu_addr, u32 *value)
{
	access_firmware_memory_exe(kbdev, gpu_addr, value, true);
}

void kbase_csf_update_firmware_memory_exe(struct kbase_device *kbdev, u32 gpu_addr, u32 value)
{
	access_firmware_memory_exe(kbdev, gpu_addr, &value, false);
}

/**
 * csf_doorbell_offset() - Calculate the offset to the CSF host doorbell
 * @kbdev: Device pointer
 * @doorbell_nr: Doorbell number
 *
 * Return: CSF host register offset for the specified doorbell number.
 */
static u32 csf_doorbell_offset(struct kbase_device *kbdev, int doorbell_nr)
{
	WARN_ON(doorbell_nr < 0);
	WARN_ON(doorbell_nr >= kbdev->csf.num_doorbells);

	return DOORBELL_BLOCK_OFFSET(doorbell_nr, DOORBELL);
}

void kbase_csf_ring_doorbell(struct kbase_device *kbdev, int doorbell_nr)
{
	kbase_reg_write32(kbdev, csf_doorbell_offset(kbdev, doorbell_nr), (u32)1);
}
EXPORT_SYMBOL(kbase_csf_ring_doorbell);

static bool global_request_complete(struct kbase_csf_fw_io *fw_io,

				    u32 const req_mask)
{
	struct kbase_device *const kbdev = fw_io->kbdev;

	bool complete = false;
	unsigned long flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);

	if ((kbase_csf_fw_io_global_read(fw_io, GLB_ACK) & req_mask) ==
	    (kbase_csf_fw_io_global_input_read(fw_io, GLB_REQ) & req_mask))
		complete = true;

	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	return complete;
}

static int wait_for_global_request_with_timeout(struct kbase_csf_fw_io *fw_io, u32 const req_mask,
						unsigned int timeout_ms)
{
	struct kbase_device *const kbdev = fw_io->kbdev;
	const long wait_timeout = kbase_csf_timeout_in_jiffies(timeout_ms);
	long remaining;

	remaining = kbase_csf_fw_io_wait_event_timeout(fw_io, kbdev->csf.event_wait,
						       global_request_complete(fw_io, req_mask),
						       wait_timeout);

	if (!remaining) {
		dev_warn(kbdev->dev,
			 "[%llu] Timeout (%d ms) waiting for global request %x to complete",
			 kbase_backend_get_cycle_cnt(kbdev), timeout_ms, req_mask);

		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_global_request(struct kbase_csf_fw_io *fw_io, u32 const req_mask)
{
	return wait_for_global_request_with_timeout(
		fw_io, req_mask, kbase_get_timeout_ms(fw_io->kbdev, CSF_FIRMWARE_TIMEOUT));
}

static void set_global_request(struct kbase_csf_fw_io *fw_io, u32 const req_mask)
{
	u32 glb_req;

	kbase_csf_scheduler_spin_lock_assert_held(fw_io->kbdev);
	kbase_csf_fw_io_assert_opened(fw_io);

	glb_req = kbase_csf_fw_io_global_read(fw_io, GLB_ACK);
	glb_req ^= req_mask;
	kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, glb_req, req_mask);
}

static void enable_endpoints_global(struct kbase_csf_fw_io *fw_io,

				    u64 const shader_core_mask)
{
	kbase_csf_fw_io_assert_opened(fw_io);

	kbase_csf_fw_io_global_write(fw_io, GLB_ALLOC_EN_LO, shader_core_mask & U32_MAX);
	kbase_csf_fw_io_global_write(fw_io, GLB_ALLOC_EN_HI, shader_core_mask >> 32);

	set_global_request(fw_io, GLB_REQ_CFG_ALLOC_EN_MASK);
}

static void set_shader_poweroff_timer(struct kbase_csf_fw_io *fw_io)
{
	u32 pwroff_reg;
	struct kbase_device *const kbdev = fw_io->kbdev;

	kbase_csf_fw_io_assert_opened(fw_io);

	if (kbdev->csf.firmware_hctl_core_pwr)
		pwroff_reg = GLB_PWROFF_TIMER_TIMER_SOURCE_SET(
			DISABLE_GLB_PWROFF_TIMER, GLB_PWROFF_TIMER_TIMER_SOURCE_SYSTEM_TIMESTAMP);
	else
		pwroff_reg = kbdev->csf.mcu_core_pwroff_dur_count;

	kbase_csf_fw_io_global_write(fw_io, GLB_PWROFF_TIMER, pwroff_reg);

	kbase_csf_fw_io_global_write_mask(fw_io, GLB_PWROFF_TIMER_CONFIG,
					  kbdev->csf.mcu_core_pwroff_dur_count_no_modifier,
					  GLB_PWROFF_TIMER_CONFIG_NO_MODIFIER_MASK);

	set_global_request(fw_io, GLB_REQ_CFG_PWROFF_TIMER_MASK);

	/* Save the programed reg value in its shadow field */
	kbdev->csf.mcu_core_pwroff_reg_shadow = pwroff_reg;

	dev_dbg(kbdev->dev, "GLB_PWROFF_TIMER set to 0x%.8x\n", pwroff_reg);
}

static void set_timeout_global(struct kbase_csf_fw_io *fw_io,

			       u64 const timeout)
{
	kbase_csf_fw_io_assert_opened(fw_io);

	kbase_csf_fw_io_global_write(fw_io, GLB_PROGRESS_TIMER,
				     timeout / GLB_PROGRESS_TIMER_TIMEOUT_SCALE);

	set_global_request(fw_io, GLB_REQ_CFG_PROGRESS_TIMER_MASK);
}

static inline void set_gpu_idle_timer_glb_req(struct kbase_csf_fw_io *fw_io, bool set)
{
	struct kbase_device *const kbdev = fw_io->kbdev;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(fw_io);

	if (set) {
		kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, GLB_REQ_REQ_IDLE_ENABLE,
						  GLB_REQ_IDLE_ENABLE_MASK);
	} else {
		kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, GLB_REQ_REQ_IDLE_DISABLE,
						  GLB_REQ_IDLE_DISABLE_MASK);
	}

	atomic_set(&kbdev->csf.scheduler.gpu_idle_timer_enabled, set);
	KBASE_KTRACE_ADD(kbdev, CSF_FIRMWARE_GLB_IDLE_TIMER_CHANGED, NULL, set);
}

static void enable_gpu_idle_timer(struct kbase_csf_fw_io *fw_io)
{
	struct kbase_device *const kbdev = fw_io->kbdev;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(fw_io);

	kbase_csf_fw_io_global_write(fw_io, GLB_IDLE_TIMER, kbdev->csf.gpu_idle_dur_count);

	kbase_csf_fw_io_global_write_mask(fw_io, GLB_IDLE_TIMER_CONFIG,
					  kbdev->csf.gpu_idle_dur_count_no_modifier
						  << GLB_IDLE_TIMER_CONFIG_NO_MODIFIER_SHIFT,
					  GLB_IDLE_TIMER_CONFIG_NO_MODIFIER_MASK);

	set_gpu_idle_timer_glb_req(fw_io, true);
	dev_dbg(kbdev->dev, "Enabling GPU idle timer with count-value: 0x%.8x",
		kbdev->csf.gpu_idle_dur_count);
}

/**
 * convert_dur_to_suspend_count() - Convert CSG suspend timeout from ms to cycle count
 * @kbdev:        Instance of a GPU platform device that implements a CSF interface
 * @dur_ms:       Timeout value in ms
 * @no_modifier:  Indicate whether bit-shift is applied, 0 when applied, 1 otherwise
 *
 * Convert CSG suspend timeout from ms to cycle count, then generate a register value
 * combining cycle count and timer source
 *
 * Return:  Register value which will be stored into register GLB_EVICTION_TIMER.
 */
static u32 convert_dur_to_suspend_count(struct kbase_device *kbdev, const u64 dur_ms,
					u32 *no_modifier)
{
	/* Get the cntfreq_el0 value, which drives the SYSTEM_TIMESTAMP */
	u64 freq = kbase_arch_timer_get_cntfrq(kbdev);
	u64 dur_val = dur_ms;
	u32 cnt_val_u32, reg_val_u32;
	const bool src_system_timestamp = freq > 0;
	const u8 SUSPEND_VAL_UNIT_SHIFT = 10;

	if (!src_system_timestamp) {
		/* Get the cycle_counter source alternative */
		spin_lock(&kbdev->pm.clk_rtm.lock);
		if (kbdev->pm.clk_rtm.clks[0])
			freq = kbdev->pm.clk_rtm.clks[0]->clock_val;
		else
			dev_err(kbdev->dev, "No GPU clock, unexpected intregration issue!");
		spin_unlock(&kbdev->pm.clk_rtm.lock);

		dev_info(kbdev->dev,
			 "No timestamp frequency, use cycle counter for csg suspend timeout!");
	}

	/* Formula for dur_val = (dur/1e3) * freq_HZ) */
	dur_val = dur_val * freq;
	dur_val = div_u64(dur_val, MSEC_PER_SEC);
	if (dur_val < S32_MAX) {
		*no_modifier = 1;
	} else {
		dur_val = dur_val >> SUSPEND_VAL_UNIT_SHIFT;
		*no_modifier = 0;
	}

	/* Interface limits the value field to S32_MAX */
	cnt_val_u32 = (dur_val > S32_MAX) ? S32_MAX : (u32)dur_val;

	reg_val_u32 = GLB_EVICTION_TIMER_TIMEOUT_SET(0, cnt_val_u32);
	/* add the source flag */
	reg_val_u32 = GLB_EVICTION_TIMER_TIMER_SOURCE_SET(
		reg_val_u32,
		(src_system_timestamp ? GLB_EVICTION_TIMER_TIMER_SOURCE_SYSTEM_TIMESTAMP :
					      GLB_EVICTION_TIMER_TIMER_SOURCE_GPU_COUNTER));

	return reg_val_u32;
}

/**
 * set_csg_suspend_timeout() - Update CSG suspend timeout setting on FW side
 *
 * @kbdev:        Instance of a GPU platform device that implements a CSF interface
 */
static void set_csg_suspend_timeout(struct kbase_device *const kbdev)
{
	u32 dur_ms, dur_val;
	u32 no_modifier = 0;
	const u32 timeout_ms =
		(kbdev->gpu_props.impl_tech == THREAD_FEATURES_IMPLEMENTATION_TECHNOLOGY_FPGA) ||
				(kbdev->gpu_props.impl_tech ==
				 THREAD_FEATURES_IMPLEMENTATION_TECHNOLOGY_SOFTWARE) ?
			      CSG_SUSPEND_TIMEOUT_FPGA_MS :
			      CSG_SUSPEND_TIMEOUT_MS;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(fw_io);

	dur_ms = kbdev->csf.csg_suspend_timeout_ms;
	if (unlikely(dur_ms < CSG_SUSPEND_TIMEOUT_FIRMWARE_MS_MIN +
				      CSG_SUSPEND_TIMEOUT_HOST_ADDED_MS ||
		     dur_ms > CSG_SUSPEND_TIMEOUT_FIRMWARE_MS_MAX +
				      CSG_SUSPEND_TIMEOUT_HOST_ADDED_MS)) {
		dev_err(kbdev->dev, "Unexpected CSG suspend timeout: %ums, default to: %ums",
			dur_ms, timeout_ms);
		kbdev->csf.csg_suspend_timeout_ms = timeout_ms;
		dur_ms = timeout_ms;
	}
	dur_ms = dur_ms - CSG_SUSPEND_TIMEOUT_HOST_ADDED_MS;

	dur_val = convert_dur_to_suspend_count(kbdev, dur_ms, &no_modifier);

	kbase_csf_fw_io_global_write(fw_io, GLB_EVICTION_TIMER, dur_val);

	kbase_csf_fw_io_global_write_mask(fw_io, GLB_EVICTION_TIMER_CONFIG, no_modifier,
					  GLB_EVICTION_TIMER_CONFIG_NO_MODIFIER_MASK);

	set_global_request(fw_io, GLB_REQ_CFG_EVICTION_TIMER_MASK);

	dev_dbg(kbdev->dev, "Updating CSG suspend timeout with count-value: 0x%.8x", dur_val);
}

static bool global_debug_request_complete(struct kbase_csf_fw_io *fw_io, u32 const req_mask)
{
	struct kbase_device *const kbdev = fw_io->kbdev;
	bool complete = false;
	unsigned long flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);

	if ((kbase_csf_fw_io_global_read(fw_io, GLB_DEBUG_ACK) & req_mask) ==
	    (kbase_csf_fw_io_global_input_read(fw_io, GLB_DEBUG_REQ) & req_mask))
		complete = true;

	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	return complete;
}

static void set_global_debug_request(struct kbase_csf_fw_io *fw_io, u32 const req_mask)
{
	u32 glb_debug_req;

	kbase_csf_scheduler_spin_lock_assert_held(fw_io->kbdev);
	kbase_csf_fw_io_assert_opened(fw_io);

	glb_debug_req = kbase_csf_fw_io_global_read(fw_io, GLB_DEBUG_ACK);
	glb_debug_req ^= req_mask;

	kbase_csf_fw_io_global_write_mask(fw_io, GLB_DEBUG_REQ, glb_debug_req, req_mask);
}

static void request_fw_core_dump(struct kbase_csf_fw_io *fw_io)
{
	uint32_t run_mode = GLB_DEBUG_REQ_RUN_MODE_SET(0, GLB_DEBUG_RUN_MODE_TYPE_CORE_DUMP);

	kbase_csf_fw_io_assert_opened(fw_io);

	set_global_debug_request(fw_io, GLB_DEBUG_REQ_DEBUG_RUN_MASK | run_mode);

	set_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);
}

int kbase_csf_firmware_req_core_dump(struct kbase_device *const kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;
	int ret;

	/* Serialize CORE_DUMP requests. */
	mutex_lock(&kbdev->csf.reg_lock);

	/* Update GLB_REQ with CORE_DUMP request and make firmware act on it. */
	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		ret = -ENODEV;
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		goto exit;
	}

	request_fw_core_dump(fw_io);
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	/* Wait for firmware to acknowledge completion of the CORE_DUMP request. */
	ret = wait_for_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);
	if (!ret)
		WARN_ON(!global_debug_request_complete(fw_io, GLB_DEBUG_REQ_DEBUG_RUN_MASK));

exit:
	mutex_unlock(&kbdev->csf.reg_lock);

	return ret;
}

/**
 * kbasep_enable_rtu - Enable Ray Tracing Unit on powering up shader core
 *
 * @kbdev:     The kbase device structure of the device
 *
 * This function needs to be called to enable the Ray Tracing Unit
 * by writing SHADER_PWRFEATURES only when host controls shader cores power.
 */
static void kbasep_enable_rtu(struct kbase_device *kbdev)
{
	if (!kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(SHADER_PWRFEATURES)))
		return;

	if (kbdev->csf.firmware_hctl_core_pwr)
		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(SHADER_PWRFEATURES), 1);
}


static void global_init(struct kbase_device *const kbdev, u64 core_mask)
{
	u32 ack_irq_mask =
		GLB_ACK_IRQ_MASK_CFG_ALLOC_EN_MASK | GLB_ACK_IRQ_MASK_PING_MASK |
		GLB_ACK_IRQ_MASK_CFG_PROGRESS_TIMER_MASK | GLB_ACK_IRQ_MASK_PROTM_ENTER_MASK |
		GLB_ACK_IRQ_MASK_PROTM_EXIT_MASK | GLB_ACK_IRQ_MASK_FIRMWARE_CONFIG_UPDATE_MASK |
		GLB_ACK_IRQ_MASK_CFG_PWROFF_TIMER_MASK | GLB_ACK_IRQ_MASK_IDLE_EVENT_MASK |
		GLB_REQ_DEBUG_CSF_REQ_MASK | GLB_ACK_IRQ_MASK_IDLE_ENABLE_MASK |
		GLB_ACK_IRQ_MASK_CFG_EVICTION_TIMER_MASK | GLB_ACK_IRQ_MASK_ITER_TRACE_ENABLE_MASK |
		GLB_ACK_FATAL_MASK;
	bool const fw_soi_allowed = kbase_pm_fw_sleep_on_idle_allowed(kbdev);
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		dev_warn(kbdev->dev, "MCU unresponsive during global init");
		goto exit;
	}

	kbasep_enable_rtu(kbdev);

	/* Update shader core allocation enable mask */
	enable_endpoints_global(fw_io, core_mask);
	set_shader_poweroff_timer(fw_io);

	set_timeout_global(fw_io, kbase_csf_timeout_get(kbdev));

	/* Update the FW Sleep-on-Idle config. This will take effect when the
	 * GLB_IDLE timer is enabled at a later point.
	 * Whilst the GLB_IDLE timer could be disabled/enabled multiple times
	 * during the normal course of operation, this is the only place where
	 * SoI config is updated for simplicity. This is to ensure that SoI
	 * config cannot change (and so will FW's expected behaviour) so that
	 * the top-half, bottom-half of GPU idle event handling & the MCU SM
	 * gets the same view of the config.
	 */
	kbase_csf_fw_io_global_write_mask(fw_io, GLB_IDLE_TIMER_CONFIG,
					  fw_soi_allowed
						  << GLB_IDLE_TIMER_CONFIG_SLEEP_ON_IDLE_SHIFT,
					  GLB_IDLE_TIMER_CONFIG_SLEEP_ON_IDLE_MASK);
	atomic_set(&kbdev->csf.scheduler.fw_soi_enabled, fw_soi_allowed);
	dev_dbg(kbdev->dev, "FW Sleep-on-Idle was %s", fw_soi_allowed ? "enabled" : "disabled");
	KBASE_KTRACE_ADD(kbdev, CSF_FIRMWARE_SLEEP_ON_IDLE_CHANGED, NULL, fw_soi_allowed);


	/* The csg suspend timeout is always enabled so customer has the flexibility to update it
	 * at any time.
	 */
	set_csg_suspend_timeout(kbdev);

	if (kbdev->pm.backend.has_host_pwr_iface)
		ack_irq_mask |= GLB_ACK_IRQ_MASK_STATE_MASK;
	/* Unmask the interrupts */
	kbase_csf_fw_io_global_write(fw_io, GLB_ACK_IRQ_MASK, ack_irq_mask);

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
	/* Enable FW MCU read/write debug interfaces */
	kbase_csf_fw_io_global_write_mask(
		fw_io, GLB_DEBUG_ACK_IRQ_MASK,
		GLB_DEBUG_REQ_FW_AS_READ_MASK | GLB_DEBUG_REQ_FW_AS_WRITE_MASK,
		GLB_DEBUG_REQ_FW_AS_READ_MASK | GLB_DEBUG_REQ_FW_AS_WRITE_MASK);
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */

	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);
exit:
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}

/**
 * global_init_on_boot - Sends a global request to control various features.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface
 *
 * Currently only the request to enable endpoints and timeout for GPU progress
 * timer is sent.
 *
 * Return: 0 on success, or negative on failure.
 */
static int global_init_on_boot(struct kbase_device *const kbdev)
{
	unsigned long flags;
	u64 core_mask;
	int ret = 0;
	u32 request_mask = CSF_GLB_REQ_CFG_MASK;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	core_mask = kbase_pm_ca_get_core_mask(kbdev);
	kbdev->csf.firmware_hctl_core_pwr = kbase_pm_no_mcu_core_pwroff(kbdev);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	global_init(kbdev, core_mask);

	ret = wait_for_global_request(&kbdev->csf.fw_io, request_mask);

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
	if (!ret) {
		kbase_debug_coresight_csf_state_request(kbdev, KBASE_DEBUG_CORESIGHT_CSF_ENABLED);

		if (!kbase_debug_coresight_csf_state_wait(kbdev,
							  KBASE_DEBUG_CORESIGHT_CSF_ENABLED)) {
			dev_err(kbdev->dev, "Timeout waiting for CoreSight to be enabled");
			ret = -ETIME;
		}
	}
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */

	return ret;
}

void kbase_csf_firmware_global_reinit(struct kbase_device *kbdev, u64 core_mask)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	kbdev->csf.glb_init_request_pending = true;
	kbdev->csf.firmware_hctl_core_pwr = kbase_pm_no_mcu_core_pwroff(kbdev);
	global_init(kbdev, core_mask);
}

bool kbase_csf_firmware_global_reinit_complete(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);
	WARN_ON(!kbdev->csf.glb_init_request_pending);

	if (global_request_complete(&kbdev->csf.fw_io, CSF_GLB_REQ_CFG_MASK))
		kbdev->csf.glb_init_request_pending = false;

	return !kbdev->csf.glb_init_request_pending;
}

void kbase_csf_firmware_update_core_attr(struct kbase_device *kbdev, bool update_core_pwroff_timer,
					 bool update_core_mask, u64 core_mask)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;

	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_GOV_CORE_MASK_SUPPORT))
		core_mask = U64_MAX;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		dev_err(kbdev->dev, "Failed to update core attributes due to unresponsive MCU.");
		goto unlock;
	}
	if (update_core_mask)
		enable_endpoints_global(fw_io, core_mask);
	if (update_core_pwroff_timer)
		set_shader_poweroff_timer(fw_io);

	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
unlock:
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}

bool kbase_csf_firmware_core_attr_updated(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	return global_request_complete(&kbdev->csf.fw_io,
				       GLB_REQ_CFG_ALLOC_EN_MASK | GLB_REQ_CFG_PWROFF_TIMER_MASK);
}

/**
 * kbase_csf_firmware_reload_worker() - reload the fw image and re-enable the MCU
 * @work: CSF Work item for reloading the firmware.
 *
 * This helper function will reload the firmware image and re-enable the MCU.
 * It is supposed to be called after MCU(GPU) has been reset.
 * Unlike the initial boot the firmware binary image is not parsed completely.
 * Only the data sections, which were loaded in memory during the initial boot,
 * are re-initialized either by zeroing them or copying their data from the
 * firmware binary image. The memory allocation for the firmware pages and
 * MMU programming is not needed for the reboot, presuming the firmware binary
 * file on the filesystem would not change.
 */
static void kbase_csf_firmware_reload_worker(struct work_struct *work)
{
	struct kbase_device *kbdev =
		container_of(work, struct kbase_device, csf.firmware_reload_work);
	unsigned long flags;
	int err;

	dev_info(kbdev->dev, "reloading firmware");

	/* Reload just the data sections from firmware binary image */
	err = reload_fw_image(kbdev);
	if (err)
		return;

	kbase_csf_tl_reader_reset(&kbdev->timeline->csf_tl_reader);

	err = kbase_csf_firmware_cfg_fw_wa_enable(kbdev);
	if (WARN_ON(err))
		return;

	/* Reboot the firmware */
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbase_csf_firmware_enable_mcu(kbdev);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
}

void kbase_csf_firmware_trigger_reload(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	kbdev->csf.firmware_reloaded = false;

	if (kbdev->csf.firmware_reload_needed) {
		kbdev->csf.firmware_reload_needed = false;
		queue_work(system_wq, &kbdev->csf.firmware_reload_work);
	} else {
		kbase_csf_firmware_enable_mcu(kbdev);
	}
}

void kbase_csf_firmware_reload_completed(struct kbase_device *kbdev)
{
	u32 version;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (unlikely(!kbdev->csf.firmware_inited))
		return;

	if (IS_ENABLED(CONFIG_MALI_DEBUG) && kbdev->pm.backend.has_host_pwr_iface) {
		const u32 glb_ack_state = get_global_ack_state(&kbdev->csf.fw_io);

		if (glb_ack_state != GLB_ACK_STATE_ACTIVE)
			dev_warn(kbdev->dev,
				 "Active state check failed on firmware reboot, state set as %u",
				 glb_ack_state);
	}

	/* Check firmware rebooted properly: we do not expect
	 * the version number to change with a running reboot.
	 */
	version = get_firmware_version(kbdev);

	if (version != kbdev->csf.global_iface.version)
		dev_err(kbdev->dev, "Version check failed in firmware reboot.");

	KBASE_KTRACE_ADD(kbdev, CSF_FIRMWARE_REBOOT, NULL, 0u);

	/* Tell MCU state machine to transit to next state */
	kbdev->csf.firmware_reloaded = true;
	kbase_pm_update_state(kbdev);
}
KBASE_EXPORT_TEST_API(kbase_csf_firmware_reload_completed);

static u32 convert_dur_to_idle_count(struct kbase_device *kbdev, const u64 dur_ns, u32 *no_modifier)
{
#define HYSTERESIS_VAL_UNIT_SHIFT (10)
	/* Get the cntfreq_el0 value, which drives the SYSTEM_TIMESTAMP */
	u64 freq = kbase_arch_timer_get_cntfrq(kbdev);
	u64 dur_val = dur_ns;
	u32 cnt_val_u32, reg_val_u32;
	bool src_system_timestamp = freq > 0;

	if (!src_system_timestamp) {
		/* Get the cycle_counter source alternative */
		spin_lock(&kbdev->pm.clk_rtm.lock);
		if (kbdev->pm.clk_rtm.clks[0])
			freq = kbdev->pm.clk_rtm.clks[0]->clock_val;
		else
			dev_warn(kbdev->dev, "No GPU clock, unexpected intregration issue!");
		spin_unlock(&kbdev->pm.clk_rtm.lock);

		dev_info(
			kbdev->dev,
			"Can't get the timestamp frequency, use cycle counter format with firmware idle hysteresis!");
	}

	/* Formula for dur_val = (dur/1e9) * freq_HZ) */
	dur_val = dur_val * freq;
	dur_val = div_u64(dur_val, NSEC_PER_SEC);
	if (dur_val < S32_MAX) {
		*no_modifier = 1;
	} else {
		dur_val = dur_val >> HYSTERESIS_VAL_UNIT_SHIFT;
		*no_modifier = 0;
	}

	/* Interface limits the value field to S32_MAX */
	cnt_val_u32 = (dur_val > S32_MAX) ? S32_MAX : (u32)dur_val;

	reg_val_u32 = GLB_IDLE_TIMER_TIMEOUT_SET(0, cnt_val_u32);
	/* add the source flag */
	reg_val_u32 = GLB_IDLE_TIMER_TIMER_SOURCE_SET(
		reg_val_u32, (src_system_timestamp ? GLB_IDLE_TIMER_TIMER_SOURCE_SYSTEM_TIMESTAMP :
							   GLB_IDLE_TIMER_TIMER_SOURCE_GPU_COUNTER));

	return reg_val_u32;
}

u64 kbase_csf_firmware_get_gpu_idle_hysteresis_time(struct kbase_device *kbdev)
{
	unsigned long flags;
	u64 dur_ns;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	dur_ns = kbdev->csf.gpu_idle_hysteresis_ns;
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	return dur_ns;
}

u32 kbase_csf_firmware_set_gpu_idle_hysteresis_time(struct kbase_device *kbdev, u64 dur_ns)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags;
	u32 no_modifier = 0;

	const u32 hysteresis_val = convert_dur_to_idle_count(kbdev, dur_ns, &no_modifier);

	/* The 'fw_load_lock' is taken to synchronize against the deferred
	 * loading of FW, where the idle timer will be enabled.
	 */
	mutex_lock(&kbdev->fw_load_lock);
	if (unlikely(!kbdev->csf.firmware_inited)) {
		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		kbdev->csf.gpu_idle_hysteresis_ns = dur_ns;
		kbdev->csf.gpu_idle_dur_count = hysteresis_val;
		kbdev->csf.gpu_idle_dur_count_no_modifier = no_modifier;
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		mutex_unlock(&kbdev->fw_load_lock);
		goto end;
	}
	mutex_unlock(&kbdev->fw_load_lock);

	if (kbase_reset_gpu_prevent_and_wait(kbdev)) {
		dev_warn(kbdev->dev,
			 "Failed to prevent GPU reset when updating idle_hysteresis_time");
		return kbdev->csf.gpu_idle_dur_count;
	}

	kbase_csf_scheduler_pm_active(kbdev);
	if (kbase_csf_scheduler_killable_wait_mcu_active(kbdev)) {
		dev_err(kbdev->dev,
			"Unable to activate the MCU, the idle hysteresis value shall remain unchanged");
		kbase_csf_scheduler_pm_idle(kbdev);
		kbase_reset_gpu_allow(kbdev);

		return kbdev->csf.gpu_idle_dur_count;
	}

	/* The scheduler lock is also taken and is held till the update is
	 * complete, to ensure the update of idle timer value by multiple Users
	 * gets serialized.
	 */
	kbase_csf_scheduler_lock(kbdev);
	kbase_csf_scheduler_spin_lock(kbdev, &flags);

	kbdev->csf.gpu_idle_hysteresis_ns = dur_ns;
	kbdev->csf.gpu_idle_dur_count = hysteresis_val;
	kbdev->csf.gpu_idle_dur_count_no_modifier = no_modifier;

	if (atomic_read(&kbdev->csf.scheduler.gpu_idle_timer_enabled)) {
		/* Timer is already enabled. Disable the timer as FW only reads
		 * the new idle timer value when timer is re-enabled.
		 */
		if (kbase_csf_firmware_disable_gpu_idle_timer(kbdev)) {
			dev_err(kbdev->dev,
				"MCU is unresponsive, GPU idle timer failed to disable.");
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
			goto unlock;
		}
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		/* Ensure that the request has taken effect */
		wait_for_global_request(fw_io, GLB_REQ_IDLE_DISABLE_MASK);
		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		if (kbase_csf_firmware_enable_gpu_idle_timer(kbdev)) {
			dev_err(kbdev->dev, "MCU is unresponsive, GPU idle timer is disabled.");
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
			goto unlock;
		}
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		wait_for_global_request(fw_io, GLB_REQ_IDLE_ENABLE_MASK);
	} else {
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
	}
unlock:
	kbase_csf_scheduler_unlock(kbdev);
	kbase_csf_scheduler_pm_idle(kbdev);
	kbase_reset_gpu_allow(kbdev);
end:
	dev_dbg(kbdev->dev, "CSF set firmware idle hysteresis count-value: 0x%.8x", hysteresis_val);

	return hysteresis_val;
}
KBASE_EXPORT_TEST_API(kbase_csf_firmware_set_gpu_idle_hysteresis_time);

static u32 convert_dur_to_core_pwroff_count(struct kbase_device *kbdev, const u64 dur_ns,
					    u32 *no_modifier)
{
	/* Get the cntfreq_el0 value, which drives the SYSTEM_TIMESTAMP */
	u64 freq = kbase_arch_timer_get_cntfrq(kbdev);
	u64 dur_val = dur_ns;
	u32 cnt_val_u32, reg_val_u32;
	bool src_system_timestamp = freq > 0;

	const struct kbase_pm_policy *current_policy = kbase_pm_get_policy(kbdev);
	bool always_on = current_policy == &kbase_pm_always_on_policy_ops;

	if (!src_system_timestamp) {
		/* Get the cycle_counter source alternative */
		spin_lock(&kbdev->pm.clk_rtm.lock);
		if (kbdev->pm.clk_rtm.clks[0])
			freq = kbdev->pm.clk_rtm.clks[0]->clock_val;
		else
			dev_warn(kbdev->dev, "No GPU clock, unexpected integration issue!");
		spin_unlock(&kbdev->pm.clk_rtm.lock);

		dev_info(
			kbdev->dev,
			"Can't get the timestamp frequency, use cycle counter with MCU shader Core Poweroff timer!");
	}

	/* Formula for dur_val = (dur/1e9) * freq_HZ) */
	dur_val = dur_val * freq;
	dur_val = div_u64(dur_val, NSEC_PER_SEC);
	if (dur_val < S32_MAX) {
		*no_modifier = 1;
	} else {
		dur_val = dur_val >> HYSTERESIS_VAL_UNIT_SHIFT;
		*no_modifier = 0;
	}

	if (dur_val == 0 && !always_on) {
		/* Lower Bound - as 0 disables timeout and host controls shader-core power management. */
		cnt_val_u32 = 1;
	} else if (dur_val > S32_MAX) {
		/* Upper Bound - as interface limits the field to S32_MAX */
		cnt_val_u32 = S32_MAX;
	} else {
		cnt_val_u32 = (u32)dur_val;
	}

	reg_val_u32 = GLB_PWROFF_TIMER_TIMEOUT_SET(0, cnt_val_u32);
	/* add the source flag */
	reg_val_u32 = GLB_PWROFF_TIMER_TIMER_SOURCE_SET(
		reg_val_u32,
		(src_system_timestamp ? GLB_PWROFF_TIMER_TIMER_SOURCE_SYSTEM_TIMESTAMP :
					      GLB_PWROFF_TIMER_TIMER_SOURCE_GPU_COUNTER));

	return reg_val_u32;
}

u64 kbase_csf_firmware_get_mcu_core_pwroff_time(struct kbase_device *kbdev)
{
	u64 pwroff_ns;
	unsigned long flags;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	pwroff_ns = kbdev->csf.mcu_core_pwroff_dur_ns;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	return pwroff_ns;
}
KBASE_EXPORT_TEST_API(kbase_csf_firmware_get_mcu_core_pwroff_time);

u32 kbase_csf_firmware_set_mcu_core_pwroff_time(struct kbase_device *kbdev, u64 dur_ns)
{
	unsigned long flags;
	u32 no_modifier = 0;

	const u32 pwroff = convert_dur_to_core_pwroff_count(kbdev, dur_ns, &no_modifier);

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->csf.mcu_core_pwroff_dur_ns = dur_ns;
	kbdev->csf.mcu_core_pwroff_dur_count = pwroff;
	kbdev->csf.mcu_core_pwroff_dur_count_no_modifier = no_modifier;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	dev_dbg(kbdev->dev, "MCU shader Core Poweroff input update: 0x%.8x", pwroff);

	return pwroff;
}

u32 kbase_csf_firmware_reset_mcu_core_pwroff_time(struct kbase_device *kbdev)
{
	return kbase_csf_firmware_set_mcu_core_pwroff_time(kbdev, DEFAULT_GLB_PWROFF_TIMEOUT_NS);
}

int kbase_csf_firmware_early_init(struct kbase_device *kbdev)
{
	kbdev->csf.num_doorbells =
		(kbdev->reg_size - CSF_HW_DOORBELL_PAGE_OFFSET) / CSF_HW_DOORBELL_PAGE_SIZE;

	if (!kbdev->csf.num_doorbells || (kbdev->csf.num_doorbells > CSF_NUM_DOORBELL_MAX)) {
		dev_err(kbdev->dev, "Invalid number of doorbell pages: %u",
			kbdev->csf.num_doorbells);
		return -EINVAL;
	}

	init_waitqueue_head(&kbdev->csf.event_wait);

	kbase_csf_firmware_reset_mcu_core_pwroff_time(kbdev);
	INIT_LIST_HEAD(&kbdev->csf.firmware_interfaces);
	INIT_LIST_HEAD(&kbdev->csf.firmware_config);
	INIT_LIST_HEAD(&kbdev->csf.firmware_timeline_metadata);
	INIT_LIST_HEAD(&kbdev->csf.firmware_trace_buffers.list);
	INIT_LIST_HEAD(&kbdev->csf.user_reg.list);
	INIT_WORK(&kbdev->csf.firmware_reload_work, kbase_csf_firmware_reload_worker);
	INIT_WORK(&kbdev->csf.glb_fatal_work, kbase_csf_glb_fatal_worker);

	init_rwsem(&kbdev->csf.mmu_sync_sem);
	mutex_init(&kbdev->csf.reg_lock);
	kbase_csf_pending_gpuq_kick_queues_init(kbdev);

	kbdev->csf.fw = (struct kbase_csf_mcu_fw){ .data = NULL };

	return 0;
}

void kbase_csf_firmware_early_term(struct kbase_device *kbdev)
{
	kbase_csf_pending_gpuq_kick_queues_term(kbdev);
	mutex_destroy(&kbdev->csf.reg_lock);
}

int kbase_csf_firmware_late_init(struct kbase_device *kbdev)
{
	u32 no_modifier = 0;
	const u32 timeout_ms =
		(kbdev->gpu_props.impl_tech == THREAD_FEATURES_IMPLEMENTATION_TECHNOLOGY_FPGA) ||
				(kbdev->gpu_props.impl_tech ==
				 THREAD_FEATURES_IMPLEMENTATION_TECHNOLOGY_SOFTWARE) ?
			      CSG_SUSPEND_TIMEOUT_FPGA_MS :
			      CSG_SUSPEND_TIMEOUT_MS;

	kbdev->csf.gpu_idle_hysteresis_ns = FIRMWARE_IDLE_HYSTERESIS_TIME_NS;

#ifdef KBASE_PM_RUNTIME
	if (kbase_pm_gpu_sleep_allowed(kbdev))
		kbdev->csf.gpu_idle_hysteresis_ns /= FIRMWARE_IDLE_HYSTERESIS_GPU_SLEEP_SCALER;
#endif
	WARN_ON(!kbdev->csf.gpu_idle_hysteresis_ns);
	kbdev->csf.gpu_idle_dur_count =
		convert_dur_to_idle_count(kbdev, kbdev->csf.gpu_idle_hysteresis_ns, &no_modifier);
	kbdev->csf.gpu_idle_dur_count_no_modifier = no_modifier;

	kbdev->csf.csg_suspend_timeout_ms = timeout_ms;

	return 0;
}

int kbase_csf_firmware_load_init(struct kbase_device *kbdev)
{
	const struct firmware *firmware = NULL;
	struct kbase_csf_mcu_fw *const mcu_fw = &kbdev->csf.fw;
	const u32 magic = FIRMWARE_HEADER_MAGIC;
	u8 version_major, version_minor;
	u32 version_hash;
	u32 entry_end_offset;
	u32 entry_offset;
	int ret;
	const char *fw_name = default_fw_name;

	lockdep_assert_held(&kbdev->fw_load_lock);

	if (WARN_ON((kbdev->as_free & MCU_AS_BITMASK) == 0))
		return -EINVAL;
	kbdev->as_free &= ~MCU_AS_BITMASK;

	ret = kbase_mmu_init(kbdev, &kbdev->csf.mcu_mmu, NULL, BASE_MEM_GROUP_DEFAULT);

	if (ret != 0) {
		/* Release the address space */
		kbdev->as_free |= MCU_AS_BITMASK;
		return ret;
	}

	ret = kbase_mcu_shared_interface_region_tracker_init(kbdev);
	if (ret != 0) {
		dev_err(kbdev->dev,
			"Failed to setup the rb tree for managing shared interface segment\n");
		goto err_out;
	}

#if IS_ENABLED(CONFIG_OF)
	/* If we can't read CSF firmware name from DTB,
	 * fw_name is not modified and remains the default.
	 */
	ret = of_property_read_string(kbdev->dev->of_node, "firmware-name", &fw_name);
	if (ret == -EINVAL) {
		/* Property doesn't exist in DTB, and fw_name already points to default FW name
		 * so just reset return value and continue.
		 */
		ret = 0;
	} else if (ret == -ENODATA) {
		dev_warn(kbdev->dev,
			 "\"firmware-name\" DTB property contains no data, using default FW name");
		/* Reset return value so FW does not fail to load */
		ret = 0;
	} else if (ret == -EILSEQ) {
		/* This is reached when the size of the fw_name buffer is too small for the string
		 * stored in the DTB and the null terminator.
		 */
		dev_warn(kbdev->dev,
			 "\"firmware-name\" DTB property value too long, using default FW name.");
		/* Reset return value so FW does not fail to load */
		ret = 0;
	}

#endif /* IS_ENABLED(CONFIG_OF) */

	if (request_firmware(&firmware, fw_name, kbdev->dev) != 0) {
		dev_err(kbdev->dev, "Failed to load firmware image '%s'\n", fw_name);
		ret = -ENOENT;
	} else {
		/* Try to save a copy and then release the loaded firmware image */
		mcu_fw->size = firmware->size;
		mcu_fw->data = vmalloc((unsigned long)mcu_fw->size);

		if (mcu_fw->data == NULL) {
			ret = -ENOMEM;
		} else {
			memcpy(mcu_fw->data, firmware->data, mcu_fw->size);
			dev_dbg(kbdev->dev, "Firmware image (%zu-bytes) retained in csf.fw\n",
				mcu_fw->size);
		}

		release_firmware(firmware);
	}

	/* If error in loading or saving the image, branches to error out */
	if (ret)
		goto err_out;

	if (mcu_fw->size < FIRMWARE_HEADER_LENGTH) {
		dev_err(kbdev->dev, "Firmware too small\n");
		ret = -EINVAL;
		goto err_out;
	}

	if (memcmp(mcu_fw->data, &magic, sizeof(magic)) != 0) {
		dev_err(kbdev->dev, "Incorrect firmware magic\n");
		ret = -EINVAL;
		goto err_out;
	}

	version_minor = mcu_fw->data[4];
	version_major = mcu_fw->data[5];

	if (version_major != FIRMWARE_HEADER_VERSION_MAJOR ||
	    version_minor != FIRMWARE_HEADER_VERSION_MINOR) {
		dev_err(kbdev->dev, "Firmware header version %d.%d not understood\n", version_major,
			version_minor);
		ret = -EINVAL;
		goto err_out;
	}

	memcpy(&version_hash, &mcu_fw->data[8], sizeof(version_hash));

	dev_notice(kbdev->dev, "Loading Mali firmware 0x%x", version_hash);

	memcpy(&entry_end_offset, &mcu_fw->data[0x10], sizeof(entry_end_offset));

	if (entry_end_offset > mcu_fw->size) {
		dev_err(kbdev->dev, "Firmware image is truncated\n");
		ret = -EINVAL;
		goto err_out;
	}

	entry_offset = FIRMWARE_HEADER_LENGTH;
	while (entry_offset < entry_end_offset) {
		u32 header;
		unsigned int size;

		memcpy(&header, &mcu_fw->data[entry_offset], sizeof(header));

		size = entry_size(header);

		ret = load_firmware_entry(kbdev, mcu_fw, entry_offset, header);
		if (ret != 0) {
			dev_err(kbdev->dev, "Failed to load firmware image\n");
			goto err_out;
		}
		entry_offset += size;
	}

	if (!kbdev->csf.shared_interface) {
		dev_err(kbdev->dev, "Shared interface region not found\n");
		ret = -EINVAL;
		goto err_out;
	} else {
		ret = setup_shared_iface_static_region(kbdev);
		if (ret != 0) {
			dev_err(kbdev->dev,
				"Failed to insert a region for shared iface entry parsed from fw image\n");
			goto err_out;
		}
	}

	ret = kbase_csf_firmware_trace_buffers_init(kbdev);
	if (ret != 0) {
		dev_err(kbdev->dev, "Failed to initialize trace buffers\n");
		goto err_out;
	}

	init_page_fault_cnt_firmware_memory(kbdev);

	ret = kbase_csf_firmware_cfg_fw_wa_init(kbdev);
	if (ret != 0) {
		dev_err(kbdev->dev, "Failed to initialize firmware workarounds");
		goto err_out;
	}

	/* Make sure L2 cache is powered up */
	kbase_pm_wait_for_l2_powered(kbdev);

	kbase_csf_fw_io_init(&kbdev->csf.fw_io, kbdev);

	/* Load the MMU tables into the selected address space */
	ret = load_mmu_tables(kbdev);
	if (ret != 0)
		goto err_out;

	boot_csf_firmware(kbdev);

	ret = parse_capabilities(kbdev);
	if (ret != 0)
		goto err_out;

	iterator_trace_init(kbdev);

	ret = kbase_csf_doorbell_mapping_init(kbdev);
	if (ret != 0)
		goto err_out;

	ret = kbase_csf_scheduler_init(kbdev);
	if (ret != 0)
		goto err_out;

	ret = kbase_csf_setup_dummy_user_reg_page(kbdev);
	if (ret != 0)
		goto err_out;

	ret = kbase_csf_timeout_init(kbdev);
	if (ret != 0)
		goto err_out;

	ret = global_init_on_boot(kbdev);
	if (ret != 0)
		goto err_out;

	ret = kbase_csf_firmware_log_init(kbdev);
	if (ret != 0) {
		dev_err(kbdev->dev, "Failed to initialize FW trace (err %d)", ret);
		goto err_out;
	}

	ret = kbase_csf_firmware_cfg_init(kbdev);
	if (ret != 0)
		goto err_out;

	if (kbdev->csf.fw_core_dump.available)
		kbase_csf_firmware_core_dump_init(kbdev);

	/* Firmware loaded successfully, ret = 0 */
	KBASE_KTRACE_ADD(kbdev, CSF_FIRMWARE_BOOT, NULL,
			 (((u64)version_hash) << 32) | (((u64)version_major) << 8) | version_minor);
	return 0;

err_out:
	kbase_csf_firmware_unload_term(kbdev);
	return ret;
}

void kbase_csf_firmware_unload_term(struct kbase_device *kbdev)
{
	unsigned long flags;
	int ret = 0;

	cancel_work_sync(&kbdev->csf.glb_fatal_work);

	ret = kbase_reset_gpu_wait(kbdev);

	WARN(ret, "failed to wait for GPU reset");

	kbase_csf_firmware_cfg_term(kbdev);

	kbase_csf_firmware_log_term(kbdev);

	kbase_csf_timeout_term(kbdev);

	kbase_csf_free_dummy_user_reg_page(kbdev);

	kbase_csf_scheduler_term(kbdev);

	kbase_csf_doorbell_mapping_term(kbdev);

	/* Explicitly trigger the disabling of MCU through the state machine and
	 * wait for its completion. It may not have been disabled yet due to the
	 * power policy.
	 */
	kbdev->pm.backend.mcu_desired = false;
	kbdev->pm.backend.l2_desired = false;
	kbase_pm_wait_for_desired_state(kbdev);

	free_global_iface(kbdev);

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->csf.firmware_inited = false;
	if (WARN_ON(kbdev->pm.backend.mcu_state != KBASE_MCU_OFF)) {
		kbdev->pm.backend.mcu_state = KBASE_MCU_OFF;
		kbase_csf_stop_firmware_and_wait(kbdev);
	}
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	unload_mmu_tables(kbdev);

	kbase_csf_firmware_cfg_fw_wa_term(kbdev);

	kbase_csf_fw_io_term(&kbdev->csf.fw_io);

	kbase_csf_firmware_trace_buffers_term(kbdev);

	while (!list_empty(&kbdev->csf.firmware_interfaces)) {
		struct kbase_csf_firmware_interface *interface;

		interface = list_first_entry(&kbdev->csf.firmware_interfaces,
					     struct kbase_csf_firmware_interface, node);
		list_del(&interface->node);

		vunmap(interface->kernel_map);

		if (!interface->reuse_pages) {
			if (interface->flags & CSF_FIRMWARE_ENTRY_PROTECTED) {
				kbase_csf_protected_memory_free(kbdev, interface->pma,
								interface->num_pages_aligned,
								interface->is_small_page);
			} else {
				kbase_mem_pool_free_pages(
					kbase_mem_pool_group_select(kbdev, KBASE_MEM_GROUP_CSF_FW,
								    interface->is_small_page),
					interface->num_pages_aligned, interface->phys, true, false);
			}

			kfree(interface->phys);
		}

		kfree(interface);
	}

	while (!list_empty(&kbdev->csf.firmware_timeline_metadata)) {
		struct firmware_timeline_metadata *metadata;

		metadata = list_first_entry(&kbdev->csf.firmware_timeline_metadata,
					    struct firmware_timeline_metadata, node);
		list_del(&metadata->node);

		kfree(metadata);
	}

	if (kbdev->csf.fw.data) {
		/* Free the copy of the firmware image */
		vfree(kbdev->csf.fw.data);
		kbdev->csf.fw.data = NULL;
		dev_dbg(kbdev->dev, "Free retained image csf.fw (%zu-bytes)\n", kbdev->csf.fw.size);
	}

	/* This will also free up the region allocated for the shared interface
	 * entry parsed from the firmware image.
	 */
	kbase_mcu_shared_interface_region_tracker_term(kbdev);

	kbase_mmu_term(kbdev, &kbdev->csf.mcu_mmu);

	/* Release the address space */
	kbdev->as_free |= MCU_AS_BITMASK;
}

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
int kbase_csf_firmware_mcu_register_write(struct kbase_device *const kbdev, u32 const reg_addr,
					  u32 const reg_val)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;
	int err;
	u32 glb_req;

	mutex_lock(&kbdev->csf.reg_lock);
	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	/* Force a FW transaction in case of coresight-specific code. */
	kbase_csf_fw_io_open_force(fw_io, &fw_io_flags);

	/* Set the address and value to write */
	kbase_csf_fw_io_global_write(fw_io, GLB_DEBUG_ARG_IN0, reg_addr);
	kbase_csf_fw_io_global_write(fw_io, GLB_DEBUG_ARG_IN1, reg_val);

	/* Set the Global Debug request for FW MCU write */
	glb_req = kbase_csf_fw_io_global_read(fw_io, GLB_DEBUG_ACK);
	glb_req ^= GLB_DEBUG_REQ_FW_AS_WRITE_MASK;
	kbase_csf_fw_io_global_write_mask(fw_io, GLB_DEBUG_REQ, glb_req,
					  GLB_DEBUG_REQ_FW_AS_WRITE_MASK);

	set_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);

	/* Notify FW about the Global Debug request */
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	err = wait_for_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);

	mutex_unlock(&kbdev->csf.reg_lock);

	dev_dbg(kbdev->dev, "w: reg %08x val %08x", reg_addr, reg_val);

	return err;
}

int kbase_csf_firmware_mcu_register_read(struct kbase_device *const kbdev, u32 const reg_addr,
					 u32 *reg_val)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;
	int err;
	u32 glb_req;

	if (WARN_ON(reg_val == NULL))
		return -EINVAL;

	mutex_lock(&kbdev->csf.reg_lock);
	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	/* Force a FW transaction in case of coresight-specific code. */
	kbase_csf_fw_io_open_force(fw_io, &fw_io_flags);

	/* Set the address to read */
	kbase_csf_fw_io_global_write(fw_io, GLB_DEBUG_ARG_IN0, reg_addr);

	/* Set the Global Debug request for FW MCU read */
	glb_req = kbase_csf_fw_io_global_read(fw_io, GLB_DEBUG_ACK);
	glb_req ^= GLB_DEBUG_REQ_FW_AS_READ_MASK;
	kbase_csf_fw_io_global_write_mask(fw_io, GLB_DEBUG_REQ, glb_req,
					  GLB_DEBUG_REQ_FW_AS_READ_MASK);

	set_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);

	/* Notify FW about the Global Debug request */
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	err = wait_for_global_request(fw_io, GLB_REQ_DEBUG_CSF_REQ_MASK);

	if (!err) {
		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		*reg_val = kbase_csf_fw_io_global_read(fw_io, GLB_DEBUG_ARG_OUT0);
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
	}

	mutex_unlock(&kbdev->csf.reg_lock);

	dev_dbg(kbdev->dev, "r: reg %08x val %08x", reg_addr, *reg_val);

	return err;
}

int kbase_csf_firmware_mcu_register_poll(struct kbase_device *const kbdev, u32 const reg_addr,
					 u32 const val_mask, u32 const reg_val)
{
	unsigned long remaining =
		kbase_csf_timeout_in_jiffies(kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT)) +
		jiffies;
	u32 read_val = 0;

	dev_dbg(kbdev->dev, "p: reg %08x val %08x mask %08x", reg_addr, reg_val, val_mask);

	while (time_before(jiffies, remaining)) {
		int err = kbase_csf_firmware_mcu_register_read(kbdev, reg_addr, &read_val);

		if (err) {
			dev_err(kbdev->dev,
				"Error reading MCU register value (read_val = %u, expect = %u)\n",
				read_val, reg_val);
			return err;
		}

		if ((read_val & val_mask) == reg_val)
			return 0;
	}

	dev_err(kbdev->dev,
		"Timeout waiting for MCU register value to be set (read_val = %u, expect = %u)\n",
		read_val, reg_val);

	return -ETIMEDOUT;
}
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */

int kbase_csf_firmware_enable_gpu_idle_timer(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;
	u32 glb_req;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		return -ENODEV;

	/* The scheduler is assumed to only call the enable when its internal
	 * state indicates that the idle timer has previously been disabled. So
	 * on entry the expected field values are:
	 *   1. GLOBAL_INPUT_BLOCK.GLB_REQ.IDLE_ENABLE: 0
	 *   2. GLOBAL_OUTPUT_BLOCK.GLB_ACK.IDLE_ENABLE: 0, or, on 1 -> 0
	 */
	glb_req = kbase_csf_fw_io_global_input_read(fw_io, GLB_REQ);
	if (glb_req & GLB_REQ_IDLE_ENABLE_MASK)
		dev_err(kbdev->dev, "Incoherent scheduler state on REQ_IDLE_ENABLE!");

	enable_gpu_idle_timer(fw_io);
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);

	return 0;
}

int kbase_csf_firmware_disable_gpu_idle_timer(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		return -ENODEV;

	set_gpu_idle_timer_glb_req(fw_io, false);
	dev_dbg(kbdev->dev, "Sending request to disable gpu idle timer");

	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);

	return 0;
}

void kbase_csf_firmware_ping(struct kbase_device *const kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		goto unlock;
	set_global_request(fw_io, GLB_REQ_PING_MASK);
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
unlock:
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}

int kbase_csf_firmware_ping_wait(struct kbase_device *const kbdev, unsigned int wait_timeout_ms)
{
	kbase_csf_firmware_ping(kbdev);

	return wait_for_global_request_with_timeout(&kbdev->csf.fw_io, GLB_REQ_PING_MASK,
						    wait_timeout_ms);
}

int kbase_csf_firmware_set_timeout(struct kbase_device *const kbdev, u64 const timeout)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;
	int err;

	/* The 'reg_lock' is also taken and is held till the update is not
	 * complete, to ensure the update of timeout value by multiple Users
	 * gets serialized.
	 */
	mutex_lock(&kbdev->csf.reg_lock);
	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		err = -ENODEV;
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		goto exit;
	}
	set_timeout_global(fw_io, timeout);
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	err = wait_for_global_request(fw_io, GLB_REQ_CFG_PROGRESS_TIMER_MASK);
exit:
	mutex_unlock(&kbdev->csf.reg_lock);

	return err;
}

int kbase_csf_enter_protected_mode(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		return -ENODEV;

	KBASE_TLSTREAM_AUX_PROTECTED_ENTER_START(kbdev, kbdev);

	set_global_request(fw_io, GLB_REQ_PROTM_ENTER_MASK);
	dev_dbg(kbdev->dev, "Sending request to enter protected mode");
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);

	return 0;
}

int kbase_csf_wait_protected_mode_enter(struct kbase_device *kbdev)
{
	int err;

	err = wait_for_global_request(&kbdev->csf.fw_io, GLB_REQ_PROTM_ENTER_MASK);

	if (!err) {
#define WAIT_TIMEOUT 5000 /* 50ms timeout */
#define DELAY_TIME_IN_US 10
		const int max_iterations = WAIT_TIMEOUT;
		int loop;

		/* Wait for the GPU to actually enter protected mode */
		for (loop = 0; loop < max_iterations; loop++) {
			unsigned long flags;
			bool pmode_exited;

			if (kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_STATUS)) &
			    GPU_STATUS_PROTECTED_MODE_ACTIVE)
				break;

			/* Check if GPU already exited the protected mode */
			kbase_csf_scheduler_spin_lock(kbdev, &flags);
			pmode_exited = !kbase_csf_scheduler_protected_mode_in_use(kbdev);
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
			if (pmode_exited)
				break;

			udelay(DELAY_TIME_IN_US);
		}

		if (loop == max_iterations) {
			dev_err(kbdev->dev, "Timeout for actual pmode entry after PROTM_ENTER ack");
			err = -ETIMEDOUT;
		}
	}

	if (unlikely(err)) {
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
			kbase_reset_gpu(kbdev);
	}

	KBASE_TLSTREAM_AUX_PROTECTED_ENTER_END(kbdev, kbdev);

	return err;
}

void kbase_csf_firmware_trigger_mcu_halt(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	/* Validate there are no on-slot groups when sending the
	 * halt request to firmware.
	 */
	WARN_ON(kbase_csf_scheduler_get_nr_active_csgs_locked(kbdev));

	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		goto unlock;

	if (kbdev->pm.backend.has_host_pwr_iface)
		set_global_req_state_as_halt(fw_io);
	else
		set_global_request(fw_io, GLB_REQ_HALT_MASK);
	dev_dbg(kbdev->dev, "Sending request to HALT MCU");
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);
unlock:
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}

void kbase_csf_firmware_enable_mcu(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	if (kbdev->pm.backend.has_host_pwr_iface)
		/* Set the state to ACTIVE before triggering the boot of MCU firmware */
		kbase_csf_firmware_set_glb_state_active(kbdev);
	else {
		/* Clear the HALT bit before triggering the boot of MCU firmware,
		 * regardless of FW I/O status.
		 */
		kbase_csf_fw_io_open_force(fw_io, &fw_io_flags);
		kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, 0, GLB_REQ_HALT_MASK);
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
	}

	enable_mcu(kbdev);
}

#ifdef KBASE_PM_RUNTIME
void kbase_csf_firmware_trigger_mcu_sleep(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		goto unlock;

	if (kbdev->pm.backend.has_host_pwr_iface)
		set_global_req_state_as_sleep(fw_io);
	else
		set_global_request(fw_io, GLB_REQ_SLEEP_MASK);
	dev_dbg(kbdev->dev, "Sending sleep request to MCU");
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
unlock:
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}
KBASE_EXPORT_TEST_API(kbase_csf_firmware_trigger_mcu_sleep);

bool kbase_csf_firmware_is_mcu_in_sleep(struct kbase_device *kbdev)
{
	bool db_notif_disabled;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	db_notif_disabled = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(MCU_CONTROL)) &
			    MCU_CNTRL_DOORBELL_DISABLE_MASK;
	if (!db_notif_disabled || !kbase_csf_firmware_mcu_halted(kbdev))
		return false;

	if (kbdev->pm.backend.has_host_pwr_iface)
		return (get_global_ack_state(fw_io) == GLB_ACK_STATE_SLEEP);

	return global_request_complete(fw_io, GLB_REQ_SLEEP_MASK);
}
KBASE_EXPORT_TEST_API(kbase_csf_firmware_is_mcu_in_sleep);

#endif /* KBASE_PM_RUNTIME */

bool kbase_csf_firmware_mcu_halt_req_complete(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (kbdev->pm.backend.has_host_pwr_iface)
		return (get_global_ack_state(&kbdev->csf.fw_io) == GLB_ACK_STATE_HALT) &&
		       kbase_csf_firmware_mcu_halted(kbdev);

	return kbase_csf_firmware_mcu_halted(kbdev);
}

void kbase_csf_firmware_set_glb_state_active(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Set Active state regardless of FW I/O status */
	kbase_csf_fw_io_open_force(fw_io, &fw_io_flags);
	set_global_req_state(fw_io, GLB_REQ_STATE_ACTIVE);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
}

int kbase_csf_trigger_firmware_config_update(struct kbase_device *kbdev)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags, fw_io_flags;
	int err = 0;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err) {
		dev_warn(kbdev->dev, "Unsuccessful GPU reset detected when updating FW config");
		return err;
	}

	/* Ensure GPU is powered-up until we complete config update.*/
	kbase_csf_scheduler_pm_active(kbdev);
	err = kbase_csf_scheduler_killable_wait_mcu_active(kbdev);
	if (err)
		goto exit;

	/* The 'reg_lock' is also taken and is held till the update is
	 * complete, to ensure the config update gets serialized.
	 */
	mutex_lock(&kbdev->csf.reg_lock);
	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	kbase_csf_fw_io_open_force(fw_io, &fw_io_flags);

	set_global_request(fw_io, GLB_REQ_FIRMWARE_CONFIG_UPDATE_MASK);
	dev_dbg(kbdev->dev, "Sending request for FIRMWARE_CONFIG_UPDATE");
	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);

	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	err = wait_for_global_request(fw_io, GLB_REQ_FIRMWARE_CONFIG_UPDATE_MASK);

	mutex_unlock(&kbdev->csf.reg_lock);

exit:
	kbase_csf_scheduler_pm_idle(kbdev);
	kbase_reset_gpu_allow(kbdev);
	return err;
}

/**
 * copy_grp_and_stm - Copy CS and/or group data
 *
 * @iface:                Global CSF interface provided by the firmware.
 * @group_data:           Pointer where to store all the group data
 *                        (sequentially).
 * @max_group_num:        The maximum number of groups to be read. Can be 0, in
 *                        which case group_data is unused.
 * @stream_data:          Pointer where to store all the CS data
 *                        (sequentially).
 * @max_total_stream_num: The maximum number of CSs to be read.
 *                        Can be 0, in which case stream_data is unused.
 *
 * Return: Total number of CSs, summed across all groups.
 */
static u32 copy_grp_and_stm(const struct kbase_csf_global_iface *const iface,
			    struct basep_cs_group_control *const group_data, u32 max_group_num,
			    struct basep_cs_stream_control *const stream_data,
			    u32 max_total_stream_num)
{
	u32 i, total_stream_num = 0;

	if (WARN_ON((max_group_num > 0) && !group_data))
		max_group_num = 0;

	if (WARN_ON((max_total_stream_num > 0) && !stream_data))
		max_total_stream_num = 0;

	for (i = 0; i < iface->group_num; i++) {
		u32 j;

		if (i < max_group_num) {
			group_data[i].features = iface->groups[i].features;
			group_data[i].stream_num = iface->groups[i].stream_num;
			group_data[i].suspend_size = iface->groups[i].suspend_size;
		}
		for (j = 0; j < iface->groups[i].stream_num; j++) {
			if (total_stream_num < max_total_stream_num)
				stream_data[total_stream_num].features =
					iface->groups[i].streams[j].features;
			total_stream_num++;
		}
	}

	return total_stream_num;
}

u32 kbase_csf_firmware_get_glb_iface(struct kbase_device *kbdev,
				     struct basep_cs_group_control *const group_data,
				     u32 const max_group_num,
				     struct basep_cs_stream_control *const stream_data,
				     u32 const max_total_stream_num, u32 *const glb_version,
				     u32 *const features, u32 *const group_num,
				     u32 *const prfcnt_size, u32 *instr_features)
{
	const struct kbase_csf_global_iface *const iface = &kbdev->csf.global_iface;

	if (WARN_ON(!glb_version) || WARN_ON(!features) || WARN_ON(!group_num) ||
	    WARN_ON(!prfcnt_size) || WARN_ON(!instr_features))
		return 0;

	*glb_version = iface->version;
	*features = iface->features;
	*group_num = iface->group_num;
	*prfcnt_size = iface->prfcnt_size;
	*instr_features = iface->instr_features;

	return copy_grp_and_stm(iface, group_data, max_group_num, stream_data,
				max_total_stream_num);
}

const char *kbase_csf_firmware_get_timeline_metadata(struct kbase_device *kbdev, const char *name,
						     size_t *size)
{
	struct firmware_timeline_metadata *metadata;

	list_for_each_entry(metadata, &kbdev->csf.firmware_timeline_metadata, node) {
		if (!strcmp(metadata->name, name)) {
			*size = metadata->size;
			return metadata->data;
		}
	}

	*size = 0;
	return NULL;
}

int kbase_csf_firmware_mcu_shared_mapping_init(struct kbase_device *kbdev, unsigned int num_pages,
					       unsigned long cpu_map_properties,
					       unsigned long gpu_map_properties,
					       struct kbase_csf_mapping *csf_mapping)
{
	struct tagged_addr *phys;
	struct kbase_va_region *va_reg;
	struct page **page_list;
	void *cpu_addr;
	unsigned int i;
	int ret = 0;
	pgprot_t cpu_map_prot = PAGE_KERNEL;
	unsigned long gpu_map_prot;

	if (cpu_map_properties & PROT_READ)
		cpu_map_prot = PAGE_KERNEL_RO;

	if (kbdev->system_coherency == COHERENCY_ACE) {
		gpu_map_prot = KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_DEFAULT_ACE);
	} else if (kbdev->system_coherency == COHERENCY_ACE_LITE) {
		gpu_map_prot = KBASE_REG_SHARE_BOTH |
			       KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_SHARED);
	} else {
		gpu_map_prot = KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_NON_CACHEABLE);
		cpu_map_prot = pgprot_writecombine(cpu_map_prot);
	}

	phys = kmalloc_array(num_pages, sizeof(*phys), GFP_KERNEL);
	if (!phys)
		goto out;

	page_list = kmalloc_array(num_pages, sizeof(*page_list), GFP_KERNEL);
	if (!page_list)
		goto page_list_alloc_error;

	ret = kbase_mem_pool_alloc_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], num_pages,
					 phys, false, NULL);
	if (ret <= 0)
		goto phys_mem_pool_alloc_error;

	for (i = 0; i < num_pages; i++)
		page_list[i] = as_page(phys[i]);

	cpu_addr = vmap(page_list, num_pages, VM_MAP, cpu_map_prot);
	if (!cpu_addr)
		goto vmap_error;

	va_reg = kbase_alloc_free_region(&kbdev->csf.mcu_shared_zone, 0, num_pages);
	if (!va_reg)
		goto va_region_alloc_error;

	mutex_lock(&kbdev->csf.reg_lock);
	ret = kbase_add_va_region_rbtree(kbdev, va_reg, 0, num_pages, 1);
	va_reg->flags &= ~KBASE_REG_FREE;
	if (ret)
		goto va_region_add_error;
	mutex_unlock(&kbdev->csf.reg_lock);

	gpu_map_properties &= (KBASE_REG_GPU_RD | KBASE_REG_GPU_WR);
	gpu_map_properties |= gpu_map_prot;

	ret = kbase_mmu_insert_pages_no_flush(kbdev, &kbdev->csf.mcu_mmu, va_reg->start_pfn,
					      &phys[0], num_pages, gpu_map_properties,
					      KBASE_MEM_GROUP_CSF_FW, NULL, NULL);
	if (ret)
		goto mmu_insert_pages_error;

	kfree(page_list);
	csf_mapping->phys = phys;
	csf_mapping->cpu_addr = cpu_addr;
	csf_mapping->va_reg = va_reg;
	csf_mapping->num_pages = num_pages;

	return 0;

mmu_insert_pages_error:
	mutex_lock(&kbdev->csf.reg_lock);
	kbase_remove_va_region(kbdev, va_reg);
va_region_add_error:
	kbase_free_alloced_region(va_reg);
	mutex_unlock(&kbdev->csf.reg_lock);
va_region_alloc_error:
	vunmap(cpu_addr);
vmap_error:
	kbase_mem_pool_free_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], num_pages, phys,
				  false, false);

phys_mem_pool_alloc_error:
	kfree(page_list);
page_list_alloc_error:
	kfree(phys);
out:
	/* Zero-initialize the mapping to make sure that the termination
	 * function doesn't try to unmap or free random addresses.
	 */
	csf_mapping->phys = NULL;
	csf_mapping->cpu_addr = NULL;
	csf_mapping->va_reg = NULL;
	csf_mapping->num_pages = 0;

	return -ENOMEM;
}

void kbase_csf_firmware_mcu_shared_mapping_term(struct kbase_device *kbdev,
						struct kbase_csf_mapping *csf_mapping)
{
	if (csf_mapping->va_reg) {
		mutex_lock(&kbdev->csf.reg_lock);
		kbase_remove_va_region(kbdev, csf_mapping->va_reg);
		kbase_free_alloced_region(csf_mapping->va_reg);
		mutex_unlock(&kbdev->csf.reg_lock);
	}

	if (csf_mapping->phys) {
		/* This is on module unload path, so the pages can be left uncleared before
		 * returning them back to kbdev memory pool.
		 */
		kbase_mem_pool_free_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW],
					  csf_mapping->num_pages, csf_mapping->phys, false, false);
	}

	vunmap(csf_mapping->cpu_addr);
	kfree(csf_mapping->phys);
}

#ifdef KBASE_PM_RUNTIME

void kbase_csf_firmware_soi_update(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	bool const soi_allowed = kbase_pm_fw_sleep_on_idle_allowed(kbdev);

	if (likely(!(soi_allowed ^ atomic_read(&kbdev->csf.scheduler.fw_soi_enabled))))
		return;

	if (likely(!soi_allowed && !atomic_read(&kbdev->csf.scheduler.gpu_idle_timer_enabled)))
		return;

	kbase_csf_scheduler_lock(kbdev);

	mutex_lock(&kbdev->pm.lock);
	if (unlikely(kbdev->pm.active_count > 1)) {
		mutex_unlock(&kbdev->pm.lock);
		goto out_unlock_scheduler_lock;
	}
	mutex_unlock(&kbdev->pm.lock);

	if ((scheduler->state == SCHED_SUSPENDED) || (scheduler->state == SCHED_SLEEPING))
		goto out_unlock_scheduler_lock;

	if (kbase_csf_scheduler_protected_mode_in_use(kbdev))
		goto out_unlock_scheduler_lock;

	/* Force the scheduler to suspend to allow the SoI config to be
	 * updated safely without racing with any ongoing or future GPU idle
	 * events.
	 */
	if (kbase_csf_scheduler_pm_suspend_no_lock(kbdev)) {
		dev_warn(
			kbdev->dev,
			"Failed to suspend the scheduler, FW Sleep-on-Idle config remains unchanged");
		goto out_unlock_scheduler_lock;
	}
	if (kbase_pm_wait_for_desired_state(kbdev))
		dev_err(kbdev->dev,
			"Scheduler suspension timed out, going to try resumption regardless");
	kbase_csf_scheduler_pm_resume_no_lock(kbdev);

out_unlock_scheduler_lock:
	kbase_csf_scheduler_unlock(kbdev);
}

void kbase_csf_firmware_glb_idle_timer_update(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags;
	enum kbase_mcu_state mcu_state;
	bool glb_request = true;

	if (likely(atomic_read(&scheduler->non_idle_offslot_grps) ^
		   atomic_read(&scheduler->gpu_idle_timer_enabled)))
		return;

	if (kbase_reset_gpu_try_prevent(kbdev))
		return;

	kbase_csf_scheduler_lock(kbdev);

	if ((scheduler->state == SCHED_SUSPENDED) || (scheduler->state == SCHED_SLEEPING))
		goto out_unlock_scheduler_lock;

	if (kbase_csf_scheduler_protected_mode_in_use(kbdev))
		goto out_unlock_scheduler_lock;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	mcu_state = kbdev->pm.backend.mcu_state;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	if (mcu_state != KBASE_MCU_ON)
		goto out_unlock_scheduler_lock;

	/* Ensure that an existing DISABLE request is completed before
	 * proceeding. They are made without waiting for them to complete such
	 * as when enabling the MCU.
	 */
	if (wait_for_global_request(fw_io, GLB_REQ_IDLE_DISABLE_MASK)) {
		dev_warn(
			kbdev->dev,
			"Existing GLB_IDLE timer config change failed to complete in time (gpu_sleep_allowed:%lx)",
			kbdev->pm.backend.gpu_sleep_allowed);
		goto out_unlock_scheduler_lock;
	}

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (atomic_read(&scheduler->non_idle_offslot_grps)) {
		if (atomic_read(&scheduler->gpu_idle_timer_enabled))
			if (kbase_csf_firmware_disable_gpu_idle_timer(kbdev)) {
				dev_err(kbdev->dev, "Failed to disable GPU idle timer.");
				glb_request = false;
			}
	} else if (!atomic_read(&scheduler->gpu_idle_timer_enabled)) {
		if (kbase_csf_firmware_enable_gpu_idle_timer(kbdev)) {
			dev_err(kbdev->dev, "Failed to enable GPU idle timer.");
			glb_request = false;
		}
	}
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
	if (likely(glb_request) && wait_for_global_request(fw_io, GLB_REQ_IDLE_DISABLE_MASK))
		dev_warn(kbdev->dev, "Failed to %s GLB_IDLE timer",
			 atomic_read(&scheduler->non_idle_offslot_grps) ? "disable" : "enable");

out_unlock_scheduler_lock:
	kbase_csf_scheduler_unlock(kbdev);
	kbase_reset_gpu_allow(kbdev);
}

int kbase_csf_firmware_soi_disable_on_scheduler_suspend(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long flags;

	lockdep_assert_held(&scheduler->lock);

	if (WARN_ON_ONCE(scheduler->state != SCHED_INACTIVE))
		return 0;

	if (!atomic_read(&kbdev->csf.scheduler.fw_soi_enabled))
		return 0;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	if (atomic_read(&scheduler->fw_soi_enabled)) {
		if (kbase_csf_firmware_disable_gpu_idle_timer(kbdev)) {
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
			dev_err(kbdev->dev,
				"Failed to disable Sleep-on-Idle config due to unresponsive MCU.");
			return -ENODEV;
		}
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		if (wait_for_global_request(&kbdev->csf.fw_io, GLB_REQ_IDLE_DISABLE_MASK)) {
			dev_err(kbdev->dev, "Failed to disable Sleep-on-Idle config");
			return -ETIMEDOUT;
		}
	} else {
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
	}

	return 0;
}

#endif /* KBASE_PM_RUNTIME */
