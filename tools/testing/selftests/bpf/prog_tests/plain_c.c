// SPDX-License-Identifier: GPL-2.0

/*
 * progs/plain_c_X.c has the progs of progs/X.c that needed bpf_for(), bpf_repeat(),
 * 'i = zero', barrier_var(), can_loop or cond_break to get past the verifier, without these.
 */

#include <test_progs.h>

/* types of globals in the skeletons */
#include "bpf_arena_list.h"

#include "plain_c_arena_htab.skel.h"
#include "plain_c_arena_htab_asm.skel.h"
#include "plain_c_arena_list.skel.h"
#include "plain_c_arena_race.skel.h"
#include "plain_c_arena_strsearch.skel.h"
#include "plain_c_bpf_ma_ttrace.skel.h"
#include "plain_c_bpf_nop_bench.skel.h"
#include "plain_c_bpf_qdisc_fifo.skel.h"
#include "plain_c_bpf_qdisc_fq.skel.h"
#include "plain_c_cpumask_success.skel.h"
#include "plain_c_dynptr_success.skel.h"
#include "plain_c_iters.skel.h"
#include "plain_c_mem_rdonly_untrusted.skel.h"
#include "plain_c_mptcp_subflow.skel.h"
#include "plain_c_rbtree_search.skel.h"
#include "plain_c_rbtree_search_kptr.skel.h"
#include "plain_c_strobemeta_bpf_loop.skel.h"
#include "plain_c_tailcall_bpf2bpf2.skel.h"
#include "plain_c_tailcall_bpf2bpf_hierarchy1.skel.h"
#include "plain_c_tailcall_bpf2bpf_hierarchy2.skel.h"
#include "plain_c_tailcall_bpf2bpf_hierarchy3.skel.h"
#include "plain_c_tailcall_callback.skel.h"
#include "plain_c_test_wakeup_source.skel.h"
#include "plain_c_tracing_stack_arg_tailcall_tgt.skel.h"
#include "plain_c_verifier_arena_large.skel.h"
#include "plain_c_verifier_iterating_callbacks.skel.h"
#include "plain_c_verifier_loop_as_is.skel.h"
#include "plain_c_xdp_lb_bench.skel.h"

#define RUN(name) RUN_TESTS(plain_c_##name)

/* prog_tests/arena_list.c: add 1000 elems, delete them, compare the sums */
static void arena_list_add_del(void)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts);
	struct plain_c_arena_list *skel;
	int cnt = 1000, sum = cnt * (cnt - 1) / 2;

	skel = plain_c_arena_list__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;
	skel->bss->cnt = cnt;
	ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(skel->progs.arena_list_add), &opts),
		  "add");
	if (skel->bss->skip) {
		test__skip();
		goto out;
	}
	ASSERT_EQ(skel->arena->arena_sum, sum, "sum of elems");
	ASSERT_EQ(skel->arena->test_val, cnt + 1, "num of elems");
	ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(skel->progs.arena_list_del), &opts),
		  "del");
	ASSERT_EQ(skel->bss->list_sum, sum, "sum of deleted elems");
out:
	plain_c_arena_list__destroy(skel);
}

/* prog_tests/bpf_ma_ttrace.c: the loop finds one cache per cpu */
static void ma_ttrace_run(void)
{
	LIBBPF_OPTS(bpf_test_run_opts, opts);
	struct plain_c_bpf_ma_ttrace *skel;

	skel = plain_c_bpf_ma_ttrace__open_and_load();
	if (!ASSERT_OK_PTR(skel, "open_and_load"))
		return;
	skel->bss->nr_cpus = libbpf_num_possible_cpus();
	ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(skel->progs.check_ttrace), &opts),
		  "test_run");
	ASSERT_OK(opts.retval, "retval");
	ASSERT_EQ(skel->bss->nr_caches, skel->bss->nr_cpus, "nr_caches");
	plain_c_bpf_ma_ttrace__destroy(skel);
}

/* prog_tests/dynptr.c: run each prog, it leaves 0 in 'err' */
static void dynptr_run(void)
{
	char data[90000];
	LIBBPF_OPTS(bpf_test_run_opts, opts, .data_in = data, .repeat = 1,
		    .data_size_in = getpagesize() == 65536 ? sizeof(data) : 5000);
	struct plain_c_dynptr_success *skel;
	struct bpf_program *prog;

	skel = plain_c_dynptr_success__open();
	if (!ASSERT_OK_PTR(skel, "open"))
		return;
	bpf_object__for_each_program(prog, skel->obj)
		bpf_program__set_autoload(prog, true);
	if (!ASSERT_OK(plain_c_dynptr_success__load(skel), "load"))
		goto out;
	bpf_object__for_each_program(prog, skel->obj) {
		skel->bss->err = 0;
		ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(prog), &opts), "test_run");
		ASSERT_EQ(skel->bss->err, 0, bpf_program__name(prog));
	}
out:
	plain_c_dynptr_success__destroy(skel);
}

/* Files with an arena map are serial, as prog_tests/arena_*.c: 512 MB of KASAN shadow per arena */
void serial_test_plain_c_arena_htab(void) { RUN(arena_htab); }
void serial_test_plain_c_arena_htab_asm(void) { RUN(arena_htab_asm); }

void serial_test_plain_c_arena_list(void)
{
	RUN(arena_list);
	if (test__start_subtest("run"))
		arena_list_add_del();
}

void test_plain_c_arena_race(void)
{
	/* prog_tests/arena_race.c: the target of fentry */
	if (libbpf_find_vmlinux_btf_id("zap_vma_range", BPF_TRACE_FENTRY) <= 0) {
		test__skip();
		return;
	}
	RUN(arena_race);
}

void serial_test_plain_c_arena_strsearch(void) { RUN(arena_strsearch); }

void test_plain_c_bpf_ma_ttrace(void)
{
	RUN(bpf_ma_ttrace);
	if (test__start_subtest("run"))
		ma_ttrace_run();
}

void test_plain_c_bpf_nop_bench(void) { RUN(bpf_nop_bench); }
void test_plain_c_bpf_qdisc_fifo(void) { RUN(bpf_qdisc_fifo); }
void test_plain_c_bpf_qdisc_fq(void) { RUN(bpf_qdisc_fq); }
void test_plain_c_cpumask_success(void) { RUN(cpumask_success); }

void test_plain_c_dynptr_success(void)
{
	RUN(dynptr_success);
	if (test__start_subtest("run"))
		dynptr_run();
}

void test_plain_c_iters(void) { RUN(iters); }
void test_plain_c_mem_rdonly_untrusted(void) { RUN(mem_rdonly_untrusted); }
void test_plain_c_mptcp_subflow(void) { RUN(mptcp_subflow); }
void test_plain_c_rbtree_search(void) { RUN(rbtree_search); }
void test_plain_c_rbtree_search_kptr(void) { RUN(rbtree_search_kptr); }
void test_plain_c_strobemeta_bpf_loop(void) { RUN(strobemeta_bpf_loop); }
void test_plain_c_tailcall_bpf2bpf2(void) { RUN(tailcall_bpf2bpf2); }
void test_plain_c_tailcall_bpf2bpf_hierarchy1(void) { RUN(tailcall_bpf2bpf_hierarchy1); }
void test_plain_c_tailcall_bpf2bpf_hierarchy2(void) { RUN(tailcall_bpf2bpf_hierarchy2); }
void test_plain_c_tailcall_bpf2bpf_hierarchy3(void) { RUN(tailcall_bpf2bpf_hierarchy3); }
void test_plain_c_tailcall_callback(void) { RUN(tailcall_callback); }

void test_plain_c_test_wakeup_source(void)
{
	/* prog_tests/wakeup_source.c: the kfuncs are there with CONFIG_PM_SLEEP */
	if (libbpf_find_vmlinux_btf_id("bpf_wakeup_sources_get_head", BPF_TRACE_FENTRY) <= 0) {
		test__skip();
		return;
	}
	RUN(test_wakeup_source);
}

void test_plain_c_tracing_stack_arg_tailcall_tgt(void) { RUN(tracing_stack_arg_tailcall_tgt); }
void serial_test_plain_c_verifier_arena_large(void) { RUN(verifier_arena_large); }
void test_plain_c_verifier_iterating_callbacks(void) { RUN(verifier_iterating_callbacks); }
void test_plain_c_verifier_loop_as_is(void) { RUN(verifier_loop_as_is); }
void test_plain_c_xdp_lb_bench(void) { RUN(xdp_lb_bench); }
