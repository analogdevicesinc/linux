// SPDX-License-Identifier: GPL-2.0
/* The tables that follow the insns which the verifier patches and removes */

#include <test_progs.h>
#include <bpf/btf.h>
#include "sysctl_helpers.h"
#include "patch_list.skel.h"

/* load one program of the object */
static struct patch_list *open_and_load(const char *name, int jmp_table_fd)
{
	struct bpf_program *prog;
	struct patch_list *skel;

	skel = patch_list__open();
	if (!ASSERT_OK_PTR(skel, "open"))
		return NULL;

	bpf_object__for_each_program(prog, skel->obj)
		bpf_program__set_autoload(prog, !strcmp(bpf_program__name(prog), name));
	if (jmp_table_fd >= 0 &&
	    !ASSERT_OK(bpf_map__reuse_fd(skel->maps.jmp_table, jmp_table_fd), "reuse_fd"))
		goto err;
	if (!ASSERT_OK(patch_list__load(skel), "load"))
		goto err;
	return skel;
err:
	patch_list__destroy(skel);
	return NULL;
}

#if defined(__x86_64__)
static int run(struct bpf_program *prog)
{
	char data[64] = {};
	LIBBPF_OPTS(bpf_test_run_opts, opts,
		.data_in = data,
		.data_size_in = sizeof(data),
		.repeat = 1,
	);

	if (!ASSERT_OK(bpf_prog_test_run_opts(bpf_program__fd(prog), &opts), "test_run"))
		return -1;
	return opts.retval;
}
#endif

/* One of the insns that set the may_goto count, in the stack or in per-cpu memory. */
static bool is_may_goto_setup(const struct bpf_insn *insn)
{
	switch (insn->code) {
	case BPF_ST | BPF_MEM | BPF_DW:
	case BPF_LD | BPF_IMM | BPF_DW:
	case 0: /* the second half of ld_imm64 */
	case BPF_ALU64 | BPF_MOV | BPF_X:
		return true;
	}
	return false;
}

/* Whether 'r0 = mark' is at 'off', or behind the insns inserted in front of a function. */
static bool mark_is_at(const struct bpf_insn *insns, __u32 cnt, __u32 off, bool func_start,
		       int mark)
{
	while (func_start && off < cnt && is_may_goto_setup(&insns[off]))
		off++;
	return off < cnt && insns[off].code == (BPF_ALU64 | BPF_MOV | BPF_K) &&
	       insns[off].dst_reg == BPF_REG_0 && insns[off].imm == mark;
}

static void test_line_and_func_info(void)
{
	/* what is left of the marks and functions of fn_1001(), in the order of the insns */
	static const int marks[] = { 1001, 1004, 1005, 1006, 1007, 1008, 3001, 3002,
				     4001, 4002, 4003 };
	static const int funcs[] = { 1001, 3001, 4001 };
	__u64 jited_linfo[64] = {}, ksyms[8] = {};
	struct bpf_line_info linfo[64] = {};
	struct bpf_func_info finfo[8] = {};
	struct bpf_insn insns[128] = {};
	struct bpf_prog_info info = {};
	__u32 i, j, k, cnt, starts = 0, len = sizeof(info);
	struct patch_list *skel;
	struct btf *btf = NULL;
	const char *str;
	int mark;

	skel = open_and_load("fn_1001", -1);
	if (!skel)
		return;

	info.xlated_prog_insns = ptr_to_u64(insns);
	info.xlated_prog_len = sizeof(insns);
	info.line_info = ptr_to_u64(linfo);
	info.nr_line_info = ARRAY_SIZE(linfo);
	info.line_info_rec_size = sizeof(*linfo);
	info.jited_line_info = ptr_to_u64(jited_linfo);
	info.nr_jited_line_info = ARRAY_SIZE(jited_linfo);
	info.jited_line_info_rec_size = sizeof(*jited_linfo);
	info.func_info = ptr_to_u64(finfo);
	info.nr_func_info = ARRAY_SIZE(finfo);
	info.func_info_rec_size = sizeof(*finfo);
	info.jited_ksyms = ptr_to_u64(ksyms);
	info.nr_jited_ksyms = ARRAY_SIZE(ksyms);
	if (!ASSERT_OK(bpf_prog_get_info_by_fd(bpf_program__fd(skel->progs.fn_1001), &info, &len),
		       "get_info"))
		goto out;
	cnt = info.xlated_prog_len / sizeof(*insns);
	if (!ASSERT_LE(cnt, ARRAY_SIZE(insns), "xlated insns") ||
	    !ASSERT_LE(info.nr_line_info, ARRAY_SIZE(linfo), "nr_line_info"))
		goto out;

	btf = btf__load_from_kernel_by_id(info.btf_id);
	if (!ASSERT_OK_PTR(btf, "btf"))
		goto out;

	if (!ASSERT_EQ(info.nr_func_info, ARRAY_SIZE(funcs), "nr_func_info"))
		goto out;
	for (i = 0; i < info.nr_func_info; i++) {
		str = btf__name_by_offset(btf, btf__type_by_id(btf, finfo[i].type_id)->name_off);
		mark = 0;
		sscanf(str, "fn_%d", &mark);
		ASSERT_EQ(mark, funcs[i], "func");
		ASSERT_TRUE(mark_is_at(insns, cnt, finfo[i].insn_off, true, mark), str);
	}

	for (i = 0, k = 0; i < info.nr_line_info; i++) {
		for (j = 0; j < info.nr_func_info; j++)
			if (finfo[j].insn_off == linfo[i].insn_off)
				break;
		/* a function begins with a line, also in the JITed code */
		if (j < info.nr_func_info)
			starts++;
		if (j < info.nr_func_info && info.nr_jited_line_info && info.nr_jited_ksyms)
			ASSERT_EQ(jited_linfo[i], ksyms[j], "jited_line_info");
		/* the line of a removed insn doesn't stay beside the line of the next insn */
		if (i)
			ASSERT_GT(linfo[i].insn_off, linfo[i - 1].insn_off, "insn_off");

		str = strstr(btf__name_by_offset(btf, linfo[i].line_off), "MARK(");
		if (!str)
			continue;
		mark = atoi(str + strlen("MARK("));
		if (k < ARRAY_SIZE(marks))
			ASSERT_EQ(mark, marks[k], "mark");
		k++;
		ASSERT_TRUE(mark_is_at(insns, cnt, linfo[i].insn_off, j < info.nr_func_info, mark),
			    str);
	}
	if (skel->rodata->has_line_text)
		ASSERT_EQ(k, ARRAY_SIZE(marks), "marks");
	ASSERT_EQ(starts, info.nr_func_info, "functions that begin with a line");
out:
	btf__free(btf);
	patch_list__destroy(skel);
}

/*
 * The x86-64 JIT finds the function of a tail call by the index in the poke
 * descriptor, and connects it at once when the program array has the target.
 * No other JIT uses the poke descriptors.
 */
static void test_poke_descriptor(void)
{
#if defined(__x86_64__)
	struct patch_list *target, *entry = NULL;
	int key = 0, fd;

	if (!is_jit_enabled()) {
		printf("%s:SKIP:JIT is disabled\n", __func__);
		test__skip();
		return;
	}

	target = open_and_load("poke_target", -1);
	if (!target)
		return;
	fd = bpf_program__fd(target->progs.poke_target);
	if (!ASSERT_OK(bpf_map__update_elem(target->maps.jmp_table, &key, sizeof(key),
					    &fd, sizeof(fd), 0), "map_update"))
		goto out;

	entry = open_and_load("poke_entry", bpf_map__fd(target->maps.jmp_table));
	if (!entry)
		goto out;
	ASSERT_EQ(run(entry->progs.poke_entry), 7, "retval");
out:
	patch_list__destroy(entry);
	patch_list__destroy(target);
#else
	printf("%s:SKIP:the JIT does not use poke descriptors\n", __func__);
	test__skip();
#endif
}

/*
 * A classic filter has no verifier env, so bpf_patch_list_apply() puts the
 * blinded constants in. Each insn with a constant grows, and the jumps of the
 * filter go over such insns. The filter returns how much of the packet stays.
 */
static void test_classic_filter(void)
{
	const char *harden = "/proc/sys/net/core/bpf_jit_harden";
	struct sock_filter code[] = {
		/* A = first byte of the packet */
		BPF_STMT(BPF_LD | BPF_B | BPF_ABS, 0),
		BPF_JUMP(BPF_JMP | BPF_JGT | BPF_K, 40, 4, 0),
		BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, 7, 2, 0),
		BPF_STMT(BPF_ALU | BPF_ADD | BPF_K, 10),
		BPF_STMT(BPF_RET | BPF_A, 0),
		/* A == 7 */
		BPF_STMT(BPF_RET | BPF_K, 0),
		/* A > 40 */
		BPF_STMT(BPF_ALU | BPF_AND | BPF_K, 15),
		BPF_JUMP(BPF_JMP | BPF_JSET | BPF_K, 1, 2, 0),
		BPF_STMT(BPF_ALU | BPF_MUL | BPF_K, 3),
		BPF_STMT(BPF_RET | BPF_A, 0),
		/* A is odd */
		BPF_STMT(BPF_RET | BPF_K, 33),
	};
	struct sock_fprog prog = { .len = ARRAY_SIZE(code), .filter = code };
	static const struct {
		char byte;
		int len;
	} tests[] = { { 1, 11 }, { 7, 0 }, { 40, 50 }, { 41, 33 }, { 44, 36 } };
	char old[16] = {}, buf[64] = {};
	int sv[2], i, err, len;

	if (!is_jit_enabled()) {
		printf("%s:SKIP:JIT is disabled\n", __func__);
		test__skip();
		return;
	}

	if (!ASSERT_OK(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), "socketpair"))
		return;
	if (sysctl_set_or_fail(harden, old, "2"))
		goto out;
	err = setsockopt(sv[1], SOL_SOCKET, SO_ATTACH_FILTER, &prog, sizeof(prog));
	sysctl_set_or_fail(harden, NULL, old);
	if (!ASSERT_OK(err, "SO_ATTACH_FILTER"))
		goto out;

	for (i = 0; i < ARRAY_SIZE(tests); i++) {
		buf[0] = tests[i].byte;
		if (!ASSERT_EQ(send(sv[0], buf, sizeof(buf), 0), sizeof(buf), "send"))
			break;
		/* nothing arrives when the filter returns 0 */
		len = recv(sv[1], buf, sizeof(buf), MSG_DONTWAIT);
		ASSERT_EQ(len < 0 ? 0 : len, tests[i].len, "len");
	}
out:
	close(sv[0]);
	close(sv[1]);
}

/*
 * Serial: the xlated code is another one and there are no poke descriptors
 * while another test sets bpf_jit_harden, and classic_filter sets it.
 */
void serial_test_patch_list(void)
{
	if (test__start_subtest("line_and_func_info"))
		test_line_and_func_info();
	if (test__start_subtest("poke_descriptor"))
		test_poke_descriptor();
	if (test__start_subtest("classic_filter"))
		test_classic_filter();
}
