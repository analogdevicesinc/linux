// SPDX-License-Identifier: GPL-2.0

#include <test_progs.h>

struct elem {
	struct elem *next;
	int val;
};

#include "arena_data_ptrs.skel.h"

static int run_prog(struct bpf_program *prog)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts);

	if (!ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(prog), &opts), "run"))
		return -1;
	return opts.retval;
}

static void test_ptrs_and_bss(void)
{
	struct arena_data_ptrs *skel;
	size_t elf_sz;

	skel = arena_data_ptrs__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;

	ASSERT_OK(run_prog(skel->progs.use_ptrs), "use_ptrs");
	if (skel->bss->skip) {
		printf("%s:SKIP:compiler doesn't support arena_cast\n", __func__);
		test__skip();
		goto out;
	}
	ASSERT_OK(run_prog(skel->progs.use_bss), "use_bss");

	/* addresses are the same in user space */
	ASSERT_EQ((void *)skel->arena->px, (void *)&skel->arena->x, "px");
	ASSERT_EQ((void *)skel->arena->parr, (void *)&skel->arena->arr[2], "parr");
	ASSERT_EQ((void *)skel->arena->head, (void *)&skel->arena->n1, "head");
	ASSERT_EQ((void *)skel->arena->n1.next, (void *)&skel->arena->n2, "n1.next");
	ASSERT_EQ(*(long *)skel->arena->pbig, 7, "*pbig");
	/* static variables are not in the skeleton */
	ASSERT_EQ(*(int *)skel->arena->psx, 45, "*psx");
	ASSERT_EQ(*(long *)skel->arena->psbig, 9, "*psbig");

	/* 'big' is not in the file */
	arena_data_ptrs__elf_bytes(&elf_sz);
	ASSERT_LT(elf_sz, 8192 * sizeof(long), "elf_sz");
out:
	arena_data_ptrs__destroy(skel);
}

static void test_ptr_to_not_arena(void)
{
	struct bpf_object *obj;

	obj = bpf_object__open_file("./arena_data_ptrs_fail.bpf.o", NULL);
	if (!ASSERT_ERR_PTR(obj, "open"))
		bpf_object__close(obj);
}

void serial_test_arena_data_ptrs(void)
{
	if (test__start_subtest("ptrs_and_bss"))
		test_ptrs_and_bss();
	if (test__start_subtest("ptr_to_not_arena"))
		test_ptr_to_not_arena();
}
