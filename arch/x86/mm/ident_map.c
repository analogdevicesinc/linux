// SPDX-License-Identifier: GPL-2.0
/*
 * Helper routines for building identity mapping page tables. This is
 * included by both the compressed kernel and the regular kernel.
 */

static void free_pte(struct x86_mapping_info *info, pmd_t *pmd)
{
	pte_t *pte = pte_offset_kernel(pmd, 0);

	info->free_pgt_page(pte, info->context);
}

static void free_pmd(struct x86_mapping_info *info, pud_t *pud)
{
	pmd_t *pmd = pmd_offset(pud, 0);
	int i;

	for (i = 0; i < PTRS_PER_PMD; i++) {
		if (!pmd_present(pmd[i]))
			continue;

		if (pmd_leaf(pmd[i]))
			continue;

		free_pte(info, &pmd[i]);
	}

	info->free_pgt_page(pmd, info->context);
}

static void free_pud(struct x86_mapping_info *info, p4d_t *p4d)
{
	pud_t *pud = pud_offset(p4d, 0);
	int i;

	for (i = 0; i < PTRS_PER_PUD; i++) {
		if (!pud_present(pud[i]))
			continue;

		if (pud_leaf(pud[i]))
			continue;

		free_pmd(info, &pud[i]);
	}

	info->free_pgt_page(pud, info->context);
}

static void free_p4d(struct x86_mapping_info *info, pgd_t *pgd)
{
	p4d_t *p4d = p4d_offset(pgd, 0);
	int i;

	for (i = 0; i < PTRS_PER_P4D; i++) {
		if (!p4d_present(p4d[i]))
			continue;

		free_pud(info, &p4d[i]);
	}

	if (pgtable_l5_enabled())
		info->free_pgt_page(p4d, info->context);
}

void kernel_ident_mapping_free(struct x86_mapping_info *info, pgd_t *pgd)
{
	int i;

	for (i = 0; i < PTRS_PER_PGD; i++) {
		if (!pgd_present(pgd[i]))
			continue;

		free_p4d(info, &pgd[i]);
	}

	info->free_pgt_page(pgd, info->context);
}

static void ident_pmd_init(struct x86_mapping_info *info, pmd_t *pmd_page,
			   unsigned long addr, unsigned long end)
{
	addr &= PMD_MASK;
	for (; addr < end; addr += PMD_SIZE) {
		pmd_t *pmd = pmd_page + pmd_index(addr);

		if (pmd_present(*pmd))
			continue;

		set_pmd(pmd, __pmd((addr - info->offset) | info->page_flag));
	}
}

static int ident_pud_init(struct x86_mapping_info *info, pud_t *pud_page,
			  unsigned long addr, unsigned long end)
{
	unsigned long next;

	for (; addr < end; addr = next) {
		pud_t *pud = pud_page + pud_index(addr);
		pmd_t *pmd;
		bool use_gbpage;

		next = (addr & PUD_MASK) + PUD_SIZE;
		if (next > end)
			next = end;

		/* if this is already a gbpage, this portion is already mapped */
		if (pud_leaf(*pud))
			continue;

		/* Is using a gbpage allowed? */
		use_gbpage = info->direct_gbpages;

		/* Don't use gbpage if it maps more than the requested region. */
		/* at the begining: */
		use_gbpage &= ((addr & ~PUD_MASK) == 0);
		/* ... or at the end: */
		use_gbpage &= ((next & ~PUD_MASK) == 0);

		/* Never overwrite existing mappings */
		use_gbpage &= !pud_present(*pud);

		if (use_gbpage) {
			pud_t pudval;

			pudval = __pud((addr - info->offset) | info->page_flag);
			set_pud(pud, pudval);
			continue;
		}

		if (pud_present(*pud)) {
			pmd = pmd_offset(pud, 0);
			ident_pmd_init(info, pmd, addr, next);
			continue;
		}
		pmd = (pmd_t *)info->alloc_pgt_page(info->context);
		if (!pmd)
			return -ENOMEM;
		ident_pmd_init(info, pmd, addr, next);
		set_pud(pud, __pud(__pa(pmd) | info->kernpg_flag));
	}

	return 0;
}

static int ident_p4d_init(struct x86_mapping_info *info, p4d_t *p4d_page,
			  unsigned long addr, unsigned long end)
{
	unsigned long next;
	int result;

	for (; addr < end; addr = next) {
		p4d_t *p4d = p4d_page + p4d_index(addr);
		pud_t *pud;

		next = (addr & P4D_MASK) + P4D_SIZE;
		if (next > end)
			next = end;

		if (p4d_present(*p4d)) {
			pud = pud_offset(p4d, 0);
			result = ident_pud_init(info, pud, addr, next);
			if (result)
				return result;

			continue;
		}
		pud = (pud_t *)info->alloc_pgt_page(info->context);
		if (!pud)
			return -ENOMEM;

		result = ident_pud_init(info, pud, addr, next);
		if (result)
			return result;

		set_p4d(p4d, __p4d(__pa(pud) | info->kernpg_flag));
	}

	return 0;
}

int kernel_ident_mapping_init(struct x86_mapping_info *info, pgd_t *pgd_page,
			      unsigned long pstart, unsigned long pend)
{
	unsigned long addr = pstart + info->offset;
	unsigned long end = pend + info->offset;
	unsigned long next;
	int result;

	/* Set the default pagetable flags if not supplied */
	if (!info->kernpg_flag)
		info->kernpg_flag = _KERNPG_TABLE;

	/* Filter out unsupported __PAGE_KERNEL_* bits: */
	info->kernpg_flag &= __default_kernel_pte_mask;

	for (; addr < end; addr = next) {
		pgd_t *pgd = pgd_page + pgd_index(addr);
		p4d_t *p4d;

		next = (addr & PGDIR_MASK) + PGDIR_SIZE;
		if (next > end)
			next = end;

		if (pgd_present(*pgd)) {
			p4d = p4d_offset(pgd, 0);
			result = ident_p4d_init(info, p4d, addr, next);
			if (result)
				return result;
			continue;
		}

		p4d = (p4d_t *)info->alloc_pgt_page(info->context);
		if (!p4d)
			return -ENOMEM;
		result = ident_p4d_init(info, p4d, addr, next);
		if (result)
			return result;
		if (pgtable_l5_enabled()) {
			set_pgd(pgd, __pgd(__pa(p4d) | info->kernpg_flag));
		} else {
			/*
			 * With p4d folded, pgd is equal to p4d.
			 * The pgd entry has to point to the pud page table in this case.
			 */
			pud_t *pud = pud_offset(p4d, 0);
			set_pgd(pgd, __pgd(__pa(pud) | info->kernpg_flag));
		}
	}

	return 0;
}
