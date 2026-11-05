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
};

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

void test_user_mem(void)
{
	struct user_mem *skel;

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

out:
	user_mem__destroy(skel);
}
