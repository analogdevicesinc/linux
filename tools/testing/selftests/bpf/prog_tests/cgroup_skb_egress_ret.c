// SPDX-License-Identifier: GPL-2.0
#include <test_progs.h>
#include "cgroup_helpers.h"
#include "cgroup_skb_egress_ret.skel.h"

void test_cgroup_skb_egress_ret(void)
{
	struct cgroup_skb_egress_ret *skel;
	int cg, fd;

	cg = test__join_cgroup("/cgroup_skb_egress_ret");
	if (!ASSERT_OK_FD(cg, "join_cgroup"))
		return;
	skel = cgroup_skb_egress_ret__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		goto out;
	fd = bpf_program__fd(skel->progs.one_or_two);
	ASSERT_EQ(bpf_prog_attach(fd, cg, BPF_CGROUP_INET_INGRESS, 0), -EINVAL, "ingress");
	ASSERT_OK(bpf_prog_attach(fd, cg, BPF_CGROUP_INET_EGRESS, 0), "egress");
	cgroup_skb_egress_ret__destroy(skel);
out:
	close(cg);
}
