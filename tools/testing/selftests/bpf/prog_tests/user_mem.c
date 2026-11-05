// SPDX-License-Identifier: GPL-2.0

#include <test_progs.h>
#include <sys/mman.h>
#include <unistd.h>

#include "user_mem.skel.h"
#include "trace_helpers.h"

enum {
	USER_MEM_MMAP = 1,
	USER_MEM_UNMAP,
	USER_MEM_COPY_INSNS,
	USER_MEM_COPY,
};

static const char data[] = "fault-enabled BPF write";

static noinline void user_mem_trigger(void)
{
	asm volatile ("" ::: "memory");
}

static bool check_user_mem_vma(void *addr, unsigned long size, const char *perms)
{
	size_t start, end;
	char vma_perms[5];
	int err;

	err = get_procmap_info(addr, &start, &end, vma_perms);
	if (!ASSERT_OK(err, "user_mem_vma"))
		return false;

	/* VMA can be merged, so ensure it covers at least the requested range. */
	if (!ASSERT_GE(end - (size_t)addr, size, "vma_size"))
		return false;

	return ASSERT_STREQ(vma_perms, perms, "vma_perms");
}

static bool check_user_mem_unmapped(void *addr)
{
	size_t start, end;
	int err;

	err = get_procmap_info(addr, &start, &end, NULL);
	return ASSERT_EQ(err, -ESRCH, "user_mem_unmapped");
}

static bool user_mem_mmap(struct user_mem *skel, unsigned long len,
			   unsigned long prot, unsigned long *addr)
{
	skel->bss->map_len = len;
	skel->bss->map_prot = prot;
	skel->bss->action = USER_MEM_MMAP;
	user_mem_trigger();

	/* vm_mmap encodes errors in the top MAX_ERRNO unsigned-long values. */
	if (!ASSERT_LT(skel->bss->user_addr, -4095UL, "user_addr"))
		return false;

	*addr = skel->bss->user_addr;
	return true;
}

static bool user_mem_munmap(struct user_mem *skel, unsigned long addr,
			   unsigned long len)
{
	skel->bss->user_addr = addr;
	skel->bss->map_len = len;
	skel->bss->action = USER_MEM_UNMAP;
	user_mem_trigger();

	return ASSERT_OK(skel->bss->ret, "user_munmap");
}

static void test_mapping(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	const unsigned long len = page_size * 3;
	unsigned long addr = ~0UL;

	if (!user_mem_mmap(skel, len, PROT_READ | PROT_WRITE, &addr))
		goto out;
	if (!check_user_mem_vma((void *)addr, len, "rw-p"))
		goto out;
	if (!user_mem_munmap(skel, addr, len))
		goto out;
	if (!check_user_mem_unmapped((void *)addr))
		goto out;
	addr = ~0UL;

out:
	if (addr != ~0UL)
		munmap((void *)addr, len);
}

static bool user_mem_copy_insns(struct user_mem *skel, unsigned long addr)
{
	skel->bss->user_addr = addr;
	skel->bss->action = USER_MEM_COPY_INSNS;
	user_mem_trigger();

	return ASSERT_OK(skel->bss->ret, "user_write");
}

static int user_mem_copy(struct user_mem *skel, unsigned long addr,
			u32 size, u64 flags)
{
	skel->bss->user_addr = addr;
	skel->bss->copy_size = size;
	skel->bss->copy_flags = flags;
	skel->bss->action = USER_MEM_COPY;
	user_mem_trigger();

	return skel->bss->ret;
}

static void test_execute_user_page(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	unsigned long addr = ~0UL;

#if !defined(__x86_64__) && !defined(__aarch64__)
	test__skip();
	return;
#endif

	if (!user_mem_mmap(skel, page_size, PROT_READ | PROT_EXEC, &addr))
		return;
	if (!check_user_mem_vma((void *)addr, page_size, "r-xp"))
		goto out;
	if (!user_mem_copy_insns(skel, addr))
		goto out;

	/* Execute user page. */
	ASSERT_EQ(((int (*)(void))addr)(), 42, "execute_user_page");

out:
	if (addr != ~0UL)
		munmap((void *)addr, page_size);
}

static void test_partial_exec(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	unsigned long addr = ~0UL;

#if !defined(__x86_64__) && !defined(__aarch64__)
	test__skip();
	return;
#endif

	if (!user_mem_mmap(skel, page_size * 3, PROT_READ | PROT_WRITE, &addr))
		return;

	/* Copy instructions to middle page. */
	if (!user_mem_copy_insns(skel, addr + page_size))
		goto out;

	/* Set +x to middle page. */
	if (!ASSERT_OK(mprotect((void *)(addr + page_size), page_size,
				PROT_READ | PROT_EXEC), "middle_page_mprotect"))
		goto out;

	if (!check_user_mem_vma((void *)addr, page_size, "rw-p") ||
	    !check_user_mem_vma((void *)(addr + page_size), page_size, "r-xp") ||
	    !check_user_mem_vma((void *)(addr + 2 * page_size), page_size, "rw-p"))
		goto out;

	/* Execute middle page. */
	if (!ASSERT_EQ(((int (*)(void))(addr + page_size))(), 42, "execute_middle_page"))
		goto out;

out:
	if (addr != ~0UL)
		munmap((void *)addr, page_size * 3);
}

static void test_faultable_write(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	unsigned long addr = ~0UL;
	unsigned char vec;
	int i;

	if (!user_mem_mmap(skel, page_size, PROT_READ | PROT_WRITE, &addr))
		return;

	/* Make sure there is a private resident page to page out. */
	memset((void *)addr, 0, page_size);

	/* Retry pageout until mincore confirms that the page is no longer resident. */
	for (i = 0; i < 500; i++) {
		if (madvise((void *)addr, page_size, MADV_PAGEOUT)) {
			test__skip();
			goto out;
		}
		if (!ASSERT_OK(mincore((void *)addr, page_size, &vec), "mincore"))
			goto out;
		if (!(vec & 1))
			break;
		usleep(100);
	}
	if (i == 500) {
		test__skip();
		goto out;
	}

	/* ..and do the copy to it. */
	if (!ASSERT_OK(user_mem_copy(skel, addr, sizeof(data), 0), "user_write_data"))
		goto out;

	ASSERT_EQ(memcmp((void *)addr, data, sizeof(data)), 0, "written_data");

out:
	if (addr != ~0UL)
		munmap((void *)addr, page_size);
}

static void test_copy_to_readonly_mapping(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	unsigned long addr = ~0UL;

	if (!user_mem_mmap(skel, page_size, PROT_READ, &addr))
		return;

	/* Assert we can copy to read-only mapping. */
	if (!ASSERT_OK(user_mem_copy(skel, addr, sizeof(data), 0), "readonly_copy"))
		goto out;

	ASSERT_EQ(memcmp((void *)addr, data, sizeof(data)), 0, "readonly_copy_data");

out:
	if (addr != ~0UL)
		munmap((void *)addr, page_size);
}

static void test_partial_copy_fault(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	bool second_page_unmapped = false;
	unsigned long addr = ~0UL;

	if (!user_mem_mmap(skel, page_size * 2, PROT_READ | PROT_WRITE, &addr))
		return;

	if (!user_mem_munmap(skel, addr + page_size, page_size))
		goto out;

	second_page_unmapped = true;

	ASSERT_EQ(user_mem_copy(skel, addr + page_size - 1, sizeof(data), 0),
		  -EFAULT, "partial_copy");

	/* The byte before the fault stays written. */
	ASSERT_EQ(((char *)addr)[page_size - 1], 'f', "partial_copy_data");

out:
	if (addr != ~0UL)
		munmap((void *)addr, second_page_unmapped ? page_size : page_size * 2);
}

static void test_zero_length_mapping(struct user_mem *skel)
{
	skel->bss->map_len = 0;
	skel->bss->map_prot = PROT_READ | PROT_WRITE;
	skel->bss->action = USER_MEM_MMAP;
	user_mem_trigger();

	ASSERT_EQ(skel->bss->user_addr, (unsigned long)-EINVAL, "zero_length_mapping");
}

static void test_failed_unmap(struct user_mem *skel)
{
	long page_size = sysconf(_SC_PAGESIZE);
	unsigned long addr = ~0UL;

	if (!user_mem_mmap(skel, page_size, PROT_READ | PROT_WRITE, &addr))
		return;

	skel->bss->user_addr = addr;
	skel->bss->map_len = 0;
	skel->bss->action = USER_MEM_UNMAP;
	user_mem_trigger();

	if (ASSERT_EQ(skel->bss->ret, -EINVAL, "zero_length_unmap"))
		check_user_mem_vma((void *)addr, page_size, "rw-p");

	munmap((void *)addr, page_size);
}

static void test_invalid_copy(struct user_mem *skel)
{
	ASSERT_EQ(user_mem_copy(skel, 1, sizeof(data), 0), -EFAULT, "bad_copy");
	ASSERT_EQ(user_mem_copy(skel, 1, sizeof(data), 1), -EINVAL, "bad_copy_flags");
	ASSERT_EQ(user_mem_copy(skel, 1, 0, 0), 0, "zero_length_copy");
	ASSERT_EQ(user_mem_copy(skel, 1, 0, 1), -EINVAL, "zero_length_copy_bad_flags");
}

static void test_invalid_prot(struct user_mem *skel)
{
	skel->bss->map_len = sysconf(_SC_PAGESIZE);
	skel->bss->map_prot = 0x80000000;
	skel->bss->action = USER_MEM_MMAP;
	user_mem_trigger();

	ASSERT_EQ(skel->bss->user_addr, -EINVAL, "invalid_prot");
}

void test_user_mem(void)
{
	struct user_mem *skel;

	RUN_TESTS(user_mem);

	skel = user_mem__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;

	skel->bss->monitored_pid = getpid();
	if (!ASSERT_OK(user_mem__attach(skel), "attach"))
		goto out;

	if (test__start_subtest("mapping"))
		test_mapping(skel);
	if (test__start_subtest("execute_user_page"))
		test_execute_user_page(skel);
	if (test__start_subtest("partial_exec"))
		test_partial_exec(skel);
	if (test__start_subtest("faultable_write"))
		test_faultable_write(skel);
	if (test__start_subtest("copy_to_readonly_mapping"))
		test_copy_to_readonly_mapping(skel);
	if (test__start_subtest("partial_copy_fault"))
		test_partial_copy_fault(skel);
	if (test__start_subtest("failed_unmap"))
		test_failed_unmap(skel);
	if (test__start_subtest("invalid_copy"))
		test_invalid_copy(skel);
	if (test__start_subtest("invalid_prot"))
		test_invalid_prot(skel);
	if (test__start_subtest("zero_length_mapping"))
		test_zero_length_mapping(skel);

out:
	user_mem__destroy(skel);
}
