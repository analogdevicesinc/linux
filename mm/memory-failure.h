/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * mm/ internal memory failure declarations
 */
#ifndef __MM_MEMORY_FAILURE_H
#define __MM_MEMORY_FAILURE_H

#include <linux/mm.h>

#ifdef CONFIG_MEMORY_FAILURE
int unmap_poisoned_folio(struct folio *folio, unsigned long pfn, bool must_kill);
void shake_folio(struct folio *folio);
typedef int hwpoison_filter_func_t(struct page *p);
void hwpoison_filter_register(hwpoison_filter_func_t *filter);
void hwpoison_filter_unregister(void);

#define MAGIC_HWPOISON	0x48575053U	/* HWPS */
void SetPageHWPoisonTakenOff(struct page *page);
void ClearPageHWPoisonTakenOff(struct page *page);
bool take_page_off_buddy(struct page *page);
bool put_page_back_buddy(struct page *page);
struct task_struct *task_early_kill(struct task_struct *tsk, int force_early);
void add_to_kill_ksm(struct task_struct *tsk, const struct page *p,
		     struct vm_area_struct *vma, struct list_head *to_kill,
		     unsigned long ksm_addr);
unsigned long page_mapped_in_vma(const struct page *page,
		struct vm_area_struct *vma);

#else
static inline int unmap_poisoned_folio(struct folio *folio, unsigned long pfn, bool must_kill)
{
	return -EBUSY;
}
#endif

#endif /* __MM_MEMORY_FAILURE_H */
