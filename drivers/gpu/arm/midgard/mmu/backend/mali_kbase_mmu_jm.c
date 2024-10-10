// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2024 ARM Limited. All rights reserved.
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

/**
 * DOC: Base kernel MMU management specific for Job Manager GPU.
 */

#include <mali_kbase.h>
#include <gpu/mali_kbase_gpu_fault.h>
#include <mali_kbase_hwaccess_jm.h>
#include <device/mali_kbase_device.h>
#include <mali_kbase_as_fault_debugfs.h>
#include <mmu/mali_kbase_mmu_internal.h>
#include <mmu/mali_kbase_mmu_faults_decoder.h>

void kbase_mmu_get_as_setup(struct kbase_mmu_table *mmut, struct kbase_mmu_setup *const setup)
{
	/* Set up the required caching policies at the correct indices
	 * in the memattr register.
	 */
	setup->memattr =
		(KBASE_MEMATTR_IMPL_DEF_CACHE_POLICY
		 << (KBASE_MEMATTR_INDEX_IMPL_DEF_CACHE_POLICY * 8)) |
		(KBASE_MEMATTR_FORCE_TO_CACHE_ALL << (KBASE_MEMATTR_INDEX_FORCE_TO_CACHE_ALL * 8)) |
		(KBASE_MEMATTR_WRITE_ALLOC << (KBASE_MEMATTR_INDEX_WRITE_ALLOC * 8)) |
		(KBASE_MEMATTR_AARCH64_OUTER_IMPL_DEF << (KBASE_MEMATTR_INDEX_OUTER_IMPL_DEF * 8)) |
		(KBASE_MEMATTR_AARCH64_OUTER_WA << (KBASE_MEMATTR_INDEX_OUTER_WA * 8)) |
		(KBASE_MEMATTR_AARCH64_NON_CACHEABLE << (KBASE_MEMATTR_INDEX_NON_CACHEABLE * 8));

	setup->transtab = (u64)mmut->pgd & AS_TRANSTAB_BASE_MASK;
	setup->transcfg = AS_TRANSCFG_MODE_SET(0ULL, AS_TRANSCFG_MODE_AARCH64_4K);
}

void kbase_gpu_report_bus_fault_and_kill(struct kbase_context *kctx, struct kbase_as *as,
					 struct kbase_fault *fault)
{
	struct kbase_device *const kbdev = kctx->kbdev;
	const u32 status = fault->status;
	const u32 exception_type = AS_FAULTSTATUS_EXCEPTION_TYPE_GET(status);
	const u32 access_type = AS_FAULTSTATUS_ACCESS_TYPE_GET(status);
	const u32 source_id = AS_FAULTSTATUS_SOURCE_ID_GET(status);
	unsigned int const as_no = as->number;
	unsigned long flags;
	const uintptr_t fault_addr = fault->addr;

	/* terminal fault, print info about the fault */
	dev_err(kbdev->dev,
		"GPU bus fault in AS%u at PA %pK\n"
		"raw fault status: 0x%X\n"
		"exception type 0x%X: %s\n"
		"access type 0x%X: %s\n"
		"source id 0x%X (core_id:utlb:IR 0x%X:0x%X:0x%X): %s, %s\n"
		"pid: %d\n",
		as_no, (void *)fault_addr, status, exception_type,
		kbase_gpu_exception_name(exception_type), access_type,
		kbase_gpu_access_type_name(kbdev, access_type), source_id,
		FAULT_SOURCE_ID_CORE_ID_GET(source_id), FAULT_SOURCE_ID_UTLB_ID_GET(source_id),
		fault_source_id_internal_requester_get(kbdev, source_id),
		fault_source_id_core_type_description_get(kbdev, source_id),
		fault_source_id_internal_requester_get_str(kbdev, source_id, access_type),
		kctx->pid);

	/* switch to UNMAPPED mode, will abort all jobs and stop any hw counter
	 * dumping AS transaction begin
	 */
	mutex_lock(&kbdev->mmu_hw_mutex);

	/* Set the MMU into unmapped mode */
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbase_mmu_disable(kctx);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	mutex_unlock(&kbdev->mmu_hw_mutex);
	/* AS transaction end */

	kbase_mmu_hw_clear_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_BUS_UNEXPECTED);
	kbase_mmu_hw_enable_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_BUS_UNEXPECTED);

}

/*
 * The caller must ensure it's retained the ctx to prevent it from being
 * scheduled out whilst it's being worked on.
 */
void kbase_mmu_report_fault_and_kill(struct kbase_context *kctx, struct kbase_as *as,
				     const char *reason_str, struct kbase_fault *fault)
{
	unsigned long flags;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbasep_js_device_data *js_devdata = &kbdev->js_data;
	unsigned int as_no = as->number;

	/* Make sure the context was active */
	if (WARN_ON(atomic_read(&kctx->refcount) <= 0))
		return;

	if (!kbase_ctx_flag(kctx, KCTX_PAGE_FAULT_REPORT_SKIP)) {
		/* decode the fault status */
		const u32 status = fault->status;
		const u32 exception_type = AS_FAULTSTATUS_EXCEPTION_TYPE_GET(status);
		const u32 access_type = AS_FAULTSTATUS_ACCESS_TYPE_GET(status);
		const u32 source_id = AS_FAULTSTATUS_SOURCE_ID_GET(status);
		/* terminal fault, print info about the fault */
		if (kbdev->gpu_props.gpu_id.product_model < GPU_ID_MODEL_MAKE(9, 0)) {
			dev_err(kbdev->dev,
				"Unhandled Page fault in AS%u at VA 0x%016llX\n"
				"Reason: %s\n"
				"raw fault status: 0x%X\n"
				"exception type 0x%X: %s\n"
				"access type 0x%X: %s\n"
				"pid: %d\n",
				as_no, fault->addr, reason_str, status, exception_type,
				kbase_gpu_exception_name(exception_type), access_type,
				kbase_gpu_access_type_name(kbdev, status), kctx->pid);
		} else {
			dev_err(kbdev->dev,
				"Unhandled Page fault in AS%u at VA 0x%016llX\n"
				"Reason: %s\n"
				"raw fault status: 0x%X\n"
				"exception type 0x%X: %s\n"
				"access type 0x%X: %s\n"
				"source id 0x%X (core_id:utlb:IR 0x%X:0x%X:0x%X): %s, %s\n"
				"pid: %d\n",
				as_no, fault->addr, reason_str, status, exception_type,
				kbase_gpu_exception_name(exception_type), access_type,
				kbase_gpu_access_type_name(kbdev, status), source_id,
				FAULT_SOURCE_ID_CORE_ID_GET(source_id),
				FAULT_SOURCE_ID_UTLB_ID_GET(source_id),
				fault_source_id_internal_requester_get(kbdev, source_id),
				fault_source_id_core_type_description_get(kbdev, source_id),
				fault_source_id_internal_requester_get_str(kbdev, source_id,
									   access_type),
				kctx->pid);
		}
	}

	/* hardware counters dump fault handling */
	spin_lock_irqsave(&kbdev->hwcnt.lock, flags);
	if ((kbdev->hwcnt.kctx) && (kbdev->hwcnt.kctx->as_nr == as_no) &&
	    (kbdev->hwcnt.backend.state == KBASE_INSTR_STATE_DUMPING)) {
		if ((fault->addr >= kbdev->hwcnt.addr) &&
		    (fault->addr < (kbdev->hwcnt.addr + kbdev->hwcnt.addr_bytes)))
			kbdev->hwcnt.backend.state = KBASE_INSTR_STATE_FAULT;
	}
	spin_unlock_irqrestore(&kbdev->hwcnt.lock, flags);

	/* Stop the kctx from submitting more jobs and cause it to be scheduled
	 * out/rescheduled - this will occur on releasing the context's refcount
	 */
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbasep_js_clear_submit_allowed(js_devdata, kctx);

	/* Kill any running jobs from the context. Submit is disallowed, so no
	 * more jobs from this context can appear in the job slots from this
	 * point on
	 */
	kbase_backend_jm_kill_running_jobs_from_kctx(kctx);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	/* AS transaction begin */
	mutex_lock(&kbdev->mmu_hw_mutex);

	/* switch to UNMAPPED mode, will abort all jobs and stop
	 * any hw counter dumping
	 */
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbase_mmu_disable(kctx);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	mutex_unlock(&kbdev->mmu_hw_mutex);

	/* AS transaction end */
	/* Clear down the fault */
	kbase_mmu_hw_clear_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_PAGE_UNEXPECTED);
	kbase_mmu_hw_enable_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_PAGE_UNEXPECTED);

}

/**
 * kbase_mmu_interrupt_process() - Process a bus or page fault.
 * @kbdev:	The kbase_device the fault happened on
 * @kctx:	The kbase_context for the faulting address space if one was
 *		found.
 * @as:		The address space that has the fault
 * @fault:	Data relating to the fault
 *
 * This function will process a fault on a specific address space
 */
static void kbase_mmu_interrupt_process(struct kbase_device *kbdev, struct kbase_context *kctx,
					struct kbase_as *as, struct kbase_fault *fault)
{
	unsigned long flags;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	dev_dbg(kbdev->dev, "Entering %s kctx %pK, as %pK\n", __func__, (void *)kctx, (void *)as);

	if (!kctx) {
		if (kbase_as_has_bus_fault(as, fault)) {
			dev_warn(
				kbdev->dev,
				"Bus error in AS%u at PA 0x%pK with no context present! Spurious IRQ or SW Design Error?\n",
				as->number, (void *)(uintptr_t)fault->addr);
		} else {
			dev_warn(
				kbdev->dev,
				"Page fault in AS%u at VA 0x%016llx with no context present! Spurious IRQ or SW Design Error?\n",
				as->number, fault->addr);
		}
		/* Since no ctx was found, the MMU must be disabled. */
		WARN_ON(as->current_setup.transtab);

		if (kbase_as_has_bus_fault(as, fault)) {
			kbase_mmu_hw_clear_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_BUS_UNEXPECTED);
			kbase_mmu_hw_enable_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_BUS_UNEXPECTED);
		} else if (kbase_as_has_page_fault(as, fault)) {
			kbase_mmu_hw_clear_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_PAGE_UNEXPECTED);
			kbase_mmu_hw_enable_fault(kbdev, as, KBASE_MMU_FAULT_TYPE_PAGE_UNEXPECTED);
		}

		return;
	}

	if (kbase_as_has_bus_fault(as, fault)) {
		struct kbasep_js_device_data *js_devdata = &kbdev->js_data;

		/*
		 * hw counters dumping in progress, signal the
		 * other thread that it failed
		 */
		spin_lock_irqsave(&kbdev->hwcnt.lock, flags);
		if ((kbdev->hwcnt.kctx == kctx) &&
		    (kbdev->hwcnt.backend.state == KBASE_INSTR_STATE_DUMPING))
			kbdev->hwcnt.backend.state = KBASE_INSTR_STATE_FAULT;

		spin_unlock_irqrestore(&kbdev->hwcnt.lock, flags);

		/*
		 * Stop the kctx from submitting more jobs and cause it
		 * to be scheduled out/rescheduled when all references
		 * to it are released
		 */
		kbasep_js_clear_submit_allowed(js_devdata, kctx);

		dev_warn(kbdev->dev, "Bus error in AS%u at PA=0x%pK, IPA=0x%pK\n", as->number,
			 (void *)(uintptr_t)fault->addr, (void *)(uintptr_t)fault->extra_addr);

		/*
		 * We need to switch to UNMAPPED mode - but we do this in a
		 * worker so that we can sleep
		 */
		WARN_ON(!queue_work(as->pf_wq, &as->work_busfault));
		atomic_inc(&kbdev->faults_pending);
	} else {
		WARN_ON(!queue_work(as->pf_wq, &as->work_pagefault));
		atomic_inc(&kbdev->faults_pending);
	}

	dev_dbg(kbdev->dev, "Leaving %s kctx %pK, as %pK\n", __func__, (void *)kctx, (void *)as);
}

static void validate_protected_page_fault(struct kbase_device *kbdev)
{
	/* GPUs which support (native) protected mode shall not report page
	 * fault addresses unless it has protected debug mode and protected
	 * debug mode is turned on
	 */
	u32 protected_debug_mode = 0;

	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_PROTECTED_DEBUG_MODE)) {
		protected_debug_mode = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_STATUS)) &
				       GPU_STATUS_GPU_DBG_ENABLED;
	}

	if (!protected_debug_mode) {
		/* fault_addr should never be reported in protected mode.
		 * However, we just continue by printing an error message
		 */
		dev_err(kbdev->dev, "Fault address reported in protected mode\n");
	}
}

void kbase_mmu_interrupt(struct kbase_device *kbdev, u32 irq_stat)
{
	const int num_as = 16;
	const int busfault_shift = MMU_PAGE_FAULT_FLAGS;
	const int pf_shift = 0;
	const unsigned long as_bit_mask = (1UL << num_as) - 1;
	unsigned long flags;
	u32 new_mask;
	u32 tmp, bf_bits, pf_bits;

	dev_dbg(kbdev->dev, "Entering %s irq_stat %u\n", __func__, irq_stat);
	/* bus faults */
	bf_bits = (irq_stat >> busfault_shift) & as_bit_mask;
	/* page faults (note: Ignore ASes with both pf and bf) */
	pf_bits = ((irq_stat >> pf_shift) & as_bit_mask) & ~bf_bits;

	if (WARN_ON(kbdev == NULL))
		return;

	/* remember current mask */
	spin_lock_irqsave(&kbdev->mmu_mask_change, flags);
	new_mask = kbase_reg_read32(kbdev, MMU_CONTROL_ENUM(IRQ_MASK));
	/* mask interrupts for now */
	kbase_reg_write32(kbdev, MMU_CONTROL_ENUM(IRQ_MASK), 0);
	spin_unlock_irqrestore(&kbdev->mmu_mask_change, flags);

	while (bf_bits | pf_bits) {
		struct kbase_as *as;
		unsigned int as_no;
		struct kbase_context *kctx;
		struct kbase_fault *fault;

		/*
		 * the while logic ensures we have a bit set, no need to check
		 * for not-found here
		 */
		as_no = (unsigned int)ffs((int)(bf_bits | pf_bits)) - 1;
		as = &kbdev->as[as_no];

		/* find the fault type */
		if (bf_bits & (1UL << as_no))
			fault = &as->bf_data;
		else
			fault = &as->pf_data;

		/*
		 * Refcount the kctx ASAP - it shouldn't disappear anyway, since
		 * Bus/Page faults _should_ only occur whilst jobs are running,
		 * and a job causing the Bus/Page fault shouldn't complete until
		 * the MMU is updated
		 */
		kctx = kbase_ctx_sched_as_to_ctx_refcount(kbdev, as_no);

		/* find faulting address */
		fault->addr = kbase_reg_read64(kbdev, MMU_AS_OFFSET(as_no, FAULTADDRESS));
		/* Mark the fault protected or not */
		fault->protected_mode = kbdev->protected_mode;

		if (kbdev->protected_mode && fault->addr) {
			/* check if address reporting is allowed */
			validate_protected_page_fault(kbdev);
		}

		/* report the fault to debugfs */
		kbase_as_fault_debugfs_new(kbdev, as_no);

		/* record the fault status */
		fault->status = kbase_reg_read32(kbdev, MMU_AS_OFFSET(as_no, FAULTSTATUS));
		fault->extra_addr = kbase_reg_read64(kbdev, MMU_AS_OFFSET(as_no, FAULTEXTRA));

		if (kbase_as_has_bus_fault(as, fault)) {
			/* Mark bus fault as handled.
			 * Note that a bus fault is processed first in case
			 * where both a bus fault and page fault occur.
			 */
			bf_bits &= ~(1UL << as_no);

			/* remove the queued BF (and PF) from the mask */
			new_mask &= ~(MMU_BUS_ERROR(as_no) | MMU_PAGE_FAULT(as_no));
		} else {
			/* Mark page fault as handled */
			pf_bits &= ~(1UL << as_no);

			/* remove the queued PF from the mask */
			new_mask &= ~MMU_PAGE_FAULT(as_no);
		}

		/* Process the interrupt for this address space */
		spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
		kbase_mmu_interrupt_process(kbdev, kctx, as, fault);
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	}

	/* reenable interrupts */
	spin_lock_irqsave(&kbdev->mmu_mask_change, flags);
	tmp = kbase_reg_read32(kbdev, MMU_CONTROL_ENUM(IRQ_MASK));
	new_mask |= tmp;
	kbase_reg_write32(kbdev, MMU_CONTROL_ENUM(IRQ_MASK), new_mask);
	spin_unlock_irqrestore(&kbdev->mmu_mask_change, flags);

	dev_dbg(kbdev->dev, "Leaving %s irq_stat %u\n", __func__, irq_stat);
}

int kbase_mmu_as_init(struct kbase_device *kbdev, unsigned int i)
{
	kbdev->as[i].number = i;
	kbdev->as[i].bf_data.addr = 0ULL;
	kbdev->as[i].pf_data.addr = 0ULL;

	kbdev->as[i].pf_wq = alloc_workqueue("mali_mmu%u", 0, 0, i);
	if (!kbdev->as[i].pf_wq)
		return -ENOMEM;

	INIT_WORK(&kbdev->as[i].work_pagefault, kbase_mmu_page_fault_worker);
	INIT_WORK(&kbdev->as[i].work_busfault, kbase_mmu_bus_fault_worker);

	return 0;
}
