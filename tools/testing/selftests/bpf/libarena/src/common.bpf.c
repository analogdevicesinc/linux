// SPDX-License-Identifier: LGPL-2.1 OR BSD-2-Clause
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */
#include <libarena/common.h>

const volatile u32 zero = 0;

int arena_fls(__u64 word)
{
	if (!word)
		return 0;

	return 64 - __builtin_clzll(word);
}

SEC("syscall")
__weak int arena_get_info(struct arena_get_info_args *args)
{
	args->arena_base = arena_base(&arena);

	return 0;
}

SEC("syscall")
__weak int arena_alloc_reserve(struct arena_alloc_reserve_args *args)
{
	return bpf_arena_reserve_pages(&arena, NULL, args->nr_pages);
}

char _license[] SEC("license") = "GPL";
