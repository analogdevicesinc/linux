// SPDX-License-Identifier: GPL-2.0

#include <test_progs.h>
#include <linux/sizes.h>
#include "arena_default_size.skel.h"

void serial_test_arena_default_size(void)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts);
	struct arena_default_size *skel;

	skel = arena_default_size__open();
	if (!ASSERT_OK_PTR(skel, "open"))
		return;
	ASSERT_EQ(bpf_map__max_entries(skel->maps.arena), SZ_4G / getpagesize(), "max_entries");
	if (!ASSERT_OK(arena_default_size__load(skel), "load"))
		goto out;
	ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(skel->progs.alloc), &opts), "run");
	ASSERT_OK(opts.retval, "retval");
out:
	arena_default_size__destroy(skel);
}
