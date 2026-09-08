// SPDX-License-Identifier: GPL-2.0-or-later

static bool test_mmap_region_basic(void)
{
	const vma_flags_t vma_flags = mk_vma_flags(VMA_READ_BIT, VMA_WRITE_BIT,
			VMA_MAYREAD_BIT, VMA_MAYWRITE_BIT);
	struct mm_struct mm = {};
	unsigned long addr;
	struct vm_area_struct *vma;
	VMA_ITERATOR(vmi, &mm, 0);

	current->mm = &mm;

	/* Map at 0x300000, length 0x3000. */
	addr = __mmap_region(NULL, 0x300000, 0x3000, vma_flags, 0x300, NULL);
	ASSERT_EQ(addr, 0x300000);

	/* Map at 0x250000, length 0x3000. */
	addr = __mmap_region(NULL, 0x250000, 0x3000, vma_flags, 0x250, NULL);
	ASSERT_EQ(addr, 0x250000);

	/* Map at 0x303000, merging to 0x300000 of length 0x6000. */
	addr = __mmap_region(NULL, 0x303000, 0x3000, vma_flags, 0x303, NULL);
	ASSERT_EQ(addr, 0x303000);

	/* Map at 0x24d000, merging to 0x250000 of length 0x6000. */
	addr = __mmap_region(NULL, 0x24d000, 0x3000, vma_flags, 0x24d, NULL);
	ASSERT_EQ(addr, 0x24d000);

	ASSERT_EQ(mm.map_count, 2);

	for_each_vma(vmi, vma) {
		if (vma->vm_start == 0x300000) {
			ASSERT_EQ(vma->vm_end, 0x306000);
			ASSERT_EQ(vma->vm_pgoff, 0x300);
		} else if (vma->vm_start == 0x24d000) {
			ASSERT_EQ(vma->vm_end, 0x253000);
			ASSERT_EQ(vma->vm_pgoff, 0x24d);
		} else {
			ASSERT_FALSE(true);
		}
	}

	cleanup_mm(&mm, &vmi);
	return true;
}

static bool test_pure_anon_dev_zero(void)
{
	const vma_flags_t vma_flags = mk_vma_flags(VMA_READ_BIT, VMA_WRITE_BIT,
			VMA_MAYREAD_BIT, VMA_MAYWRITE_BIT);
	struct file file = {
		.f_op = &zero_fops,
	};
	struct mm_struct mm = {};
	struct vm_area_struct *vma;
	unsigned long addr;
	VMA_ITERATOR(vmi, &mm, 0);

	current->mm = &mm;

	/*
	 * Map a MAP_PRIVATE-/dev/zero mapping at address 0x300000 with a page
	 * offset of 0x10, which we expect to be reset to the anonymous page
	 * offset.
	 */
	addr = __mmap_region(&file, 0x300000, 0x3000, vma_flags, 0x10, NULL);
	ASSERT_EQ(addr, 0x300000);

	/* Assert that it truly is an anonymous mapping. */
	vma = vma_lookup(&mm, addr);
	ASSERT_NE(vma, NULL);
	ASSERT_TRUE(vma_is_anonymous(vma));
	ASSERT_EQ(vma->vm_file, NULL);
	ASSERT_EQ(vma->vm_private_data, NULL);
	/* Expect anonymous page offsets. */
	ASSERT_EQ(vma->vm_pgoff, 0x300);
	ASSERT_EQ(vma_start_anon_pgoff(vma), 0x300);

	cleanup_mm(&mm, &vmi);
	return true;
}

static void run_mmap_tests(int *num_tests, int *num_fail)
{
	TEST(mmap_region_basic);
	TEST(pure_anon_dev_zero);
}
