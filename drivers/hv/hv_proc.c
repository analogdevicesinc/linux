// SPDX-License-Identifier: GPL-2.0
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/clockchips.h>
#include <linux/slab.h>
#include <linux/cpuhotplug.h>
#include <linux/minmax.h>
#include <linux/export.h>
#include <linux/set_memory.h>
#include <asm/mshyperv.h>
#include <asm/tlbflush.h>

#define HV_DEPOSIT_MAX 512
#define HV_DEPOSIT_INP_MAX ((HV_HYP_PAGE_SIZE -  \
	offsetof(struct hv_deposit_memory, gpa_page_list)) / sizeof(u64))

/*
 * Add or remove a set of physically contiguous page runs from the kernel
 * direct map. Once a page has been deposited the hypervisor owns it and
 * revokes root partition access to it.
 */
static int hv_deposit_update_direct_map(const u64 *pfns, int count, bool valid)
{
	int i, end, err, ret = 0;

	for (i = 0; i < count; i = end) {
		for (end = i + 1; end < count; ++end) {
			if (pfns[end] != pfns[end - 1] + 1)
				break;
		}
		err = set_direct_map_valid_noflush(pfn_to_page(pfns[i]),
						   end - i, valid);
		if (err && !ret)
			ret = err;
	}

	if (valid)
		return ret;

	for (i = 0; i < count; i = end) {
		unsigned long addr = (unsigned long)page_address(pfn_to_page(pfns[i]));
		unsigned long size;

		for (end = i + 1; end < count; ++end) {
			if (pfns[end] != pfns[end - 1] + 1)
				break;
		}
		size = (unsigned long)(end - i) << PAGE_SHIFT;
		flush_tlb_kernel_range(addr, addr + size);
	}

	return ret;
}

/*
 * Allocate free pages for deposit to hypervisor. pfna[] must be large enough
 * to hold HV_DEPOSIT_INP_MAX (511) pages. If num_pages is 512, return last
 * pfn in lastpfn. If @single, then it must be a single allocation (not split
 * over multiple contiguous ranges).
 *
 * Returns: number of pages allocated or -ENOMEM
 */
static int hv_alloc_dep_pages(int node, u64 *pfna, u64 *lastpfnp, int num_pages,
			      bool single)
{
	struct page *page;
	int num_allocd, count = 0;

	/* Published ABI, enforce its immutability. */
	BUILD_BUG_ON(HV_DEPOSIT_INP_MAX != 511);

	if (num_pages > HV_DEPOSIT_MAX ||
	    (num_pages == HV_DEPOSIT_MAX && lastpfnp == NULL))
		return -EINVAL;

	while (num_pages) {
		/* Find highest order we can actually allocate */
		int order = 31 - __builtin_clz(num_pages);
		gfp_t gfp_flags = GFP_KERNEL;

		if (!single)
			gfp_flags |= __GFP_NOWARN;

		while (1) {
			page = alloc_pages_node(node, gfp_flags, order);
			if (page || order == 0 || single)
				break;

			order--;
		}

		if (page == NULL)
			break;

		split_page(page, order);
		num_allocd = 1 << order;
		num_pages -= num_allocd;

		while (num_allocd && count < HV_DEPOSIT_INP_MAX) {
			pfna[count++] = page_to_pfn(page++);
			num_allocd--;
		}

		if (num_allocd-- && count == HV_DEPOSIT_INP_MAX) {
			*lastpfnp = page_to_pfn(page);
			count++;
			break;
		}
	}

	return count ? count : -ENOMEM;
}

/*
 * Deposit memory in the hypervisor. Even if @contiguous is false, a contiguous
 * 2M worth of pfns is utmost desired for performance reasons. But short of
 * that, we deposit whatever contiguous chunks we can get. If @contiguous is
 * true, then the entire range has to be physically contiguous. Note, in that
 * case, upon withdrawl, hypervisor could return any page in between the range,
 * so we must split that also. Lastly, HV_MAX_CONTIGUOUS_ALLOCATION_PAGES is
 * not guaranteed to always be power of 2.
 */
static int hv_call_deposit_pages(int node, u64 partition_id, bool contiguous)
{
	struct hv_deposit_memory *hc_input;
	int i, rc, num_pages, completed = 0;
	u64 status, *pfna, lastpfn = 0;
	bool trunc_extra = false;

	BUILD_BUG_ON(HV_MAX_CONTIGUOUS_ALLOCATION_PAGES > HV_DEPOSIT_MAX);

	if (contiguous) {
		num_pages = roundup_pow_of_two(
					  HV_MAX_CONTIGUOUS_ALLOCATION_PAGES);
		trunc_extra = num_pages != HV_MAX_CONTIGUOUS_ALLOCATION_PAGES;
	} else {
		num_pages = HV_DEPOSIT_MAX;
	}

	hc_input = (struct hv_deposit_memory *)get_zeroed_page(GFP_KERNEL);
	if (hc_input == NULL)
		return -ENOMEM;

	hc_input->partition_id = partition_id;
	pfna = hc_input->gpa_page_list;

	rc = hv_alloc_dep_pages(node, pfna, &lastpfn, num_pages, contiguous);
	if (rc < 0)
		goto out_free;

	num_pages = rc;
	if (num_pages > HV_DEPOSIT_INP_MAX)
		num_pages = HV_DEPOSIT_INP_MAX;

	if (contiguous && trunc_extra) {
		for (i = HV_MAX_CONTIGUOUS_ALLOCATION_PAGES; i < num_pages; i++)
			__free_page(pfn_to_page(pfna[i]));

		if (lastpfn) {
			__free_page(pfn_to_page(lastpfn));
			lastpfn = 0;
		}

		num_pages = HV_MAX_CONTIGUOUS_ALLOCATION_PAGES;
	}

	rc = hv_deposit_update_direct_map(pfna, num_pages, false);
	if (rc)
		goto err_restore_direct_map;

	if (lastpfn) {
		rc = hv_deposit_update_direct_map(&lastpfn, 1, false);
		if (rc)
			goto err_restore_direct_map;
	}

	/* Not using hyperv_pcpu_input_arg, so no need to disable interrupts */

	status = hv_do_rep_hypercall(HVCALL_DEPOSIT_MEMORY, num_pages, 0,
				     hc_input, NULL);
	if (!hv_result_success(status))
		goto err_free_dep_pages;

	if (lastpfn) {
		hc_input->gpa_page_list[0] = lastpfn;
		status = hv_do_rep_hypercall(HVCALL_DEPOSIT_MEMORY, 1, 0,
					     hc_input, NULL);

		if (!hv_result_success(status) && hv_repcomp(status) == 0)
			/* We deposited lot earlier, so give it a go */
			hv_restore_withdrawn_pages(&lastpfn, 1);
	}

	free_page((unsigned long)hc_input);
	return 0;

err_free_dep_pages:
	hv_status_err(status, "\n");
	rc = hv_result_to_errno(status);
	completed = hv_repcomp(status);

err_restore_direct_map:
	/*
	 * We don't want to return pages to the allocator if weren't able to
	 * mark them valid in the direct map.
	 */
	if (hv_deposit_update_direct_map(pfna + completed,
					 num_pages - completed, true) ||
	    (lastpfn && hv_deposit_update_direct_map(&lastpfn, 1, true))) {
		WARN(1, "leaking %d page(s) that could not be set to valid\n",
		     num_pages - completed + !!lastpfn);
		goto out_free;
	}

	for (i = completed; i < num_pages; i++)
		__free_page(pfn_to_page(pfna[i]));
	if (lastpfn)
		__free_page(pfn_to_page(lastpfn));

out_free:
	free_page((unsigned long)hc_input);
	return rc;
}

/*
 * Put withdrawn pages back in the direct map. Counterpart to the direct map
 * removal done by hv_call_deposit_pages().
 */
void hv_restore_withdrawn_pages(const u64 *pfns, int count)
{
	int i, j, end, n, ret;
	struct page *page;

	for (i = 0; i < count; i = end) {
		page = pfn_to_page(pfns[i]);
		/* find consecutive runs of pfns */
		for (end = i + 1; end < count; ++end) {
			if (pfns[end] != pfns[end - 1] + 1)
				break;
		}
		n = end - i;
		ret = set_direct_map_valid_noflush(page, n, true);

		for (j = i; j < end; ++j) {
			page = pfn_to_page(pfns[j]);
			/*
			 * Retry individually on failed batch. We don't want
			 * to return pages to the allocator that we couldn't mark
			 * valid in the direct map.
			 */
			if (ret &&
			    WARN_ON_ONCE(set_direct_map_valid_noflush(page, 1, true)))
				continue;
			__free_page(page);
		}
	}
}
EXPORT_SYMBOL_GPL(hv_restore_withdrawn_pages);

int hv_deposit_memory_node(int node, u64 pt_id, u64 hv_status)
{
	int result = hv_result(hv_status);
	bool contiguous = false;

	if (result == HV_STATUS_INSUFFICIENT_ROOT_MEMORY ||
	    result == HV_STATUS_INSUFFICIENT_CONTIGUOUS_ROOT_MEMORY) {
		if (!hv_root_partition()) {
			hv_status_err(hv_status,
				      "Unexpected root memory deposit\n");
			return -EINVAL;
		}

		pt_id = HV_PARTITION_ID_SELF;
	}

	if (result == HV_STATUS_INSUFFICIENT_CONTIGUOUS_MEMORY ||
	    result == HV_STATUS_INSUFFICIENT_CONTIGUOUS_ROOT_MEMORY)
		contiguous = true;

	return hv_call_deposit_pages(node, pt_id, contiguous);
}
EXPORT_SYMBOL_GPL(hv_deposit_memory_node);

bool hv_result_needs_memory(u64 status)
{
	switch (hv_result(status)) {
	case HV_STATUS_INSUFFICIENT_MEMORY:
	case HV_STATUS_INSUFFICIENT_CONTIGUOUS_MEMORY:
	case HV_STATUS_INSUFFICIENT_ROOT_MEMORY:
	case HV_STATUS_INSUFFICIENT_CONTIGUOUS_ROOT_MEMORY:
		return true;
	}
	return false;
}
EXPORT_SYMBOL_GPL(hv_result_needs_memory);

int hv_call_add_logical_proc(int node, u32 lp_index, u32 apic_id)
{
	struct hv_input_add_logical_processor *input;
	struct hv_output_add_logical_processor *output;
	u64 status;
	unsigned long flags;
	int ret = 0;

	/*
	 * When adding a logical processor, the hypervisor may return
	 * HV_STATUS_INSUFFICIENT_MEMORY. When that happens, we deposit more
	 * pages and retry.
	 */
	do {
		local_irq_save(flags);

		input = *this_cpu_ptr(hyperv_pcpu_input_arg);
		/* We don't do anything with the output right now */
		output = *this_cpu_ptr(hyperv_pcpu_output_arg);

		input->lp_index = lp_index;
		input->apic_id = apic_id;
		input->proximity_domain_info = hv_numa_node_to_pxm_info(node);
		status = hv_do_hypercall(HVCALL_ADD_LOGICAL_PROCESSOR,
					 input, output);
		local_irq_restore(flags);

		if (!hv_result_needs_memory(status)) {
			if (!hv_result_success(status)) {
				hv_status_err(status, "cpu %u apic ID: %u\n",
					      lp_index, apic_id);
				ret = hv_result_to_errno(status);
			}
			break;
		}
		ret = hv_deposit_memory_node(node, hv_current_partition_id,
					     status);
	} while (!ret);

	return ret;
}

int hv_call_create_vp(int node, u64 partition_id, u32 vp_index, u32 flags)
{
	struct hv_create_vp *input;
	u64 status;
	unsigned long irq_flags;
	int ret = 0;

	do {
		local_irq_save(irq_flags);

		input = *this_cpu_ptr(hyperv_pcpu_input_arg);

		input->partition_id = partition_id;
		input->vp_index = vp_index;
		input->flags = flags;
		input->subnode_type = HV_SUBNODE_ANY;
		input->proximity_domain_info = hv_numa_node_to_pxm_info(node);
		status = hv_do_hypercall(HVCALL_CREATE_VP, input, NULL);
		local_irq_restore(irq_flags);

		if (!hv_result_needs_memory(status)) {
			if (!hv_result_success(status)) {
				hv_status_err(status, "vcpu: %u, lp: %u\n",
					      vp_index, flags);
				ret = hv_result_to_errno(status);
			}
			break;
		}
		ret = hv_deposit_memory_node(node, partition_id, status);

	} while (!ret);

	return ret;
}
EXPORT_SYMBOL_GPL(hv_call_create_vp);

int hv_call_notify_all_processors_started(void)
{
	struct hv_input_notify_partition_event *input;
	u64 status;
	unsigned long irq_flags;
	int ret = 0;

	local_irq_save(irq_flags);
	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	memset(input, 0, sizeof(*input));
	input->event = HV_PARTITION_ALL_LOGICAL_PROCESSORS_STARTED;
	status = hv_do_hypercall(HVCALL_NOTIFY_PARTITION_EVENT,
				 input, NULL);
	local_irq_restore(irq_flags);

	if (!hv_result_success(status)) {
		hv_status_err(status, "\n");
		ret = hv_result_to_errno(status);
	}
	return ret;
}

bool hv_lp_exists(u32 lp_index)
{
	struct hv_input_get_logical_processor_run_time *input;
	struct hv_output_get_logical_processor_run_time *output;
	unsigned long flags;
	u64 status;

	local_irq_save(flags);
	input = *this_cpu_ptr(hyperv_pcpu_input_arg);
	output = *this_cpu_ptr(hyperv_pcpu_output_arg);

	input->lp_index = lp_index;
	status = hv_do_hypercall(HVCALL_GET_LOGICAL_PROCESSOR_RUN_TIME,
				 input, output);
	local_irq_restore(flags);

	if (!hv_result_success(status) &&
	    hv_result(status) != HV_STATUS_INVALID_LP_INDEX) {
		hv_status_err(status, "\n");
		BUG();
	}

	return hv_result_success(status);
}
