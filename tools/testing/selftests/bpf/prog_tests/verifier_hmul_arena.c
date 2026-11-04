// SPDX-License-Identifier: GPL-2.0

#include <test_progs.h>
#include "verifier_hmul_arena.skel.h"

void test_verifier_hmul_arena(void)
{
	int ret;

	/* Arena support can depend on CPU features, such as Zacas on RV64. */
	ret = libbpf_probe_bpf_map_type(BPF_MAP_TYPE_ARENA, NULL);
	if (!ASSERT_GE(ret, 0, "probe_arena"))
		return;
	if (!ret) {
		test__skip();
		return;
	}
	RUN_TESTS(verifier_hmul_arena);
}
