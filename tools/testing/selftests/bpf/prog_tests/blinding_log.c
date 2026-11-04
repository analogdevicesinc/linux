// SPDX-License-Identifier: GPL-2.0
/* The verifier log of a prog whose constants are blinded */

#include <test_progs.h>
#include <linux/filter.h>
#include "sysctl_helpers.h"

#define NR_CONSTS 11000
#define LOG_SIZE 65536

/*
 * A prog of one function gets its constants blinded when its runtime is
 * selected. Each 'r0 += 1' becomes 3 insns, which pushes the jump over them
 * out of its 16-bit range. Serial, because it sets bpf_jit_harden.
 */
void serial_test_blinding_log(void)
{
	const char *harden = "/proc/sys/net/core/bpf_jit_harden";
	LIBBPF_OPTS(bpf_prog_load_opts, opts);
	struct bpf_insn *insns;
	char old[16] = {}, *log;
	int i, cnt = 0, fd;

	if (!is_jit_enabled()) {
		printf("%s:SKIP:JIT is disabled\n", __func__);
		test__skip();
		return;
	}

	insns = calloc(NR_CONSTS + 3, sizeof(*insns));
	log = calloc(1, LOG_SIZE);
	if (!ASSERT_OK_PTR(insns, "insns") || !ASSERT_OK_PTR(log, "log"))
		goto out;

	insns[cnt++] = BPF_EMIT_CALL(BPF_FUNC_get_prandom_u32);
	insns[cnt++] = BPF_JMP_IMM(BPF_JEQ, BPF_REG_0, 0, NR_CONSTS);
	for (i = 0; i < NR_CONSTS; i++)
		insns[cnt++] = BPF_ALU64_IMM(BPF_ADD, BPF_REG_0, 1);
	insns[cnt++] = BPF_EXIT_INSN();

	opts.log_buf = log;
	opts.log_size = LOG_SIZE;
	/* log stats only: level 1 logs each insn, which wraps the buffer */
	opts.log_level = 4;
	if (sysctl_set_or_fail(harden, old, "2"))
		goto out;
	fd = bpf_prog_load(BPF_PROG_TYPE_SOCKET_FILTER, NULL, "GPL", insns, cnt, &opts);
	sysctl_set_or_fail(harden, NULL, old);
	/* don't mind whether the interpreter takes the prog or the load fails */
	if (fd >= 0)
		close(fd);

	/* expect the message about the jump in the log and in log_true_size */
	ASSERT_HAS_SUBSTR(log, "cannot be patched due to 16-bit range", "log");
	ASSERT_EQ(opts.log_true_size, strlen(log) + 1, "log_true_size");
out:
	free(insns);
	free(log);
}
