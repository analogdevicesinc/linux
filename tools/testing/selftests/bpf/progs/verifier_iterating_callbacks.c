// SPDX-License-Identifier: GPL-2.0
#include "bpf_misc.h"
#include "bpf_experimental.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 8);
	__type(key, __u32);
	__type(value, __u64);
} map SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_USER_RINGBUF);
	__uint(max_entries, 8);
} ringbuf SEC(".maps");

struct vm_area_struct;
struct bpf_map;

struct buf_context {
	char *buf;
};

struct num_context {
	__u64 i;
	__u64 j;
};

__u8 choice_arr[2] = { 0, 1 };

static int unsafe_on_2nd_iter_cb(__u32 idx, struct buf_context *ctx)
{
	if (idx == 0) {
		ctx->buf = (char *)(0xDEAD);
		return 0;
	}

	if (bpf_probe_read_user(ctx->buf, 8, (void *)(0xBADC0FFEE)))
		return 1;

	return 0;
}

SEC("?raw_tp")
__failure __msg("R1 type=scalar expected=fp")
int unsafe_on_2nd_iter(void *unused)
{
	char buf[4];
	struct buf_context loop_ctx = { .buf = buf };

	bpf_loop(100, unsafe_on_2nd_iter_cb, &loop_ctx, 0);
	return 0;
}

static int unsafe_on_zero_iter_cb(__u32 idx, struct num_context *ctx)
{
	ctx->i = 0;
	return 0;
}

SEC("?raw_tp")
__failure __msg("invalid access to map value, value_size=2 off=32 size=1")
int unsafe_on_zero_iter(void *unused)
{
	struct num_context loop_ctx = { .i = 32 };

	bpf_loop(100, unsafe_on_zero_iter_cb, &loop_ctx, 0);
	return choice_arr[loop_ctx.i];
}

static int widening_cb(__u32 idx, struct num_context *ctx)
{
	++ctx->i;
	return 0;
}

SEC("?raw_tp")
__success
int widening(void *unused)
{
	struct num_context loop_ctx = { .i = 0, .j = 1 };

	bpf_loop(100, widening_cb, &loop_ctx, 0);
	/* loop_ctx.j is not changed during callback iteration,
	 * verifier should not apply widening to it.
	 */
	return choice_arr[loop_ctx.j];
}

static int loop_detection_cb(__u32 idx, struct num_context *ctx)
{
	for (;;) {}
	return 0;
}

SEC("?raw_tp")
__failure __msg("infinite loop detected")
int loop_detection(void *unused)
{
	struct num_context loop_ctx = { .i = 0 };

	bpf_loop(100, loop_detection_cb, &loop_ctx, 0);
	return 0;
}

static __always_inline __u64 oob_state_machine(struct num_context *ctx)
{
	switch (ctx->i) {
	case 0:
		ctx->i = 1;
		break;
	case 1:
		ctx->i = 32;
		break;
	}
	return 0;
}

static __u64 for_each_map_elem_cb(struct bpf_map *map, __u32 *key, __u64 *val, void *data)
{
	return oob_state_machine(data);
}

SEC("?raw_tp")
__failure __msg("invalid access to map value, value_size=2 off=32 size=1")
int unsafe_for_each_map_elem(void *unused)
{
	struct num_context loop_ctx = { .i = 0 };

	bpf_for_each_map_elem(&map, for_each_map_elem_cb, &loop_ctx, 0);
	return choice_arr[loop_ctx.i];
}

static __u64 ringbuf_drain_cb(struct bpf_dynptr *dynptr, void *data)
{
	return oob_state_machine(data);
}

SEC("?raw_tp")
__failure __msg("invalid access to map value, value_size=2 off=32 size=1")
int unsafe_ringbuf_drain(void *unused)
{
	struct num_context loop_ctx = { .i = 0 };

	bpf_user_ringbuf_drain(&ringbuf, ringbuf_drain_cb, &loop_ctx, 0);
	return choice_arr[loop_ctx.i];
}

static __u64 find_vma_cb(struct task_struct *task, struct vm_area_struct *vma, void *data)
{
	return oob_state_machine(data);
}

SEC("?raw_tp")
__failure __msg("invalid access to map value, value_size=2 off=32 size=1")
int unsafe_find_vma(void *unused)
{
	struct task_struct *task = bpf_get_current_task_btf();
	struct num_context loop_ctx = { .i = 0 };

	bpf_find_vma(task, 0, find_vma_cb, &loop_ctx, 0);
	return choice_arr[loop_ctx.i];
}

static int iter_limit_cb(__u32 idx, struct num_context *ctx)
{
	ctx->i++;
	return 0;
}

SEC("?raw_tp")
__success
int bpf_loop_iter_limit_ok(void *unused)
{
	struct num_context ctx = { .i = 0 };

	bpf_loop(1, iter_limit_cb, &ctx, 0);
	return choice_arr[ctx.i];
}

SEC("?raw_tp")
__failure __msg("invalid access to map value, value_size=2 off=2 size=1")
int bpf_loop_iter_limit_overflow(void *unused)
{
	struct num_context ctx = { .i = 0 };

	bpf_loop(2, iter_limit_cb, &ctx, 0);
	return choice_arr[ctx.i];
}

static int iter_limit_level2a_cb(__u32 idx, struct num_context *ctx)
{
	ctx->i += 100;
	return 0;
}

static int iter_limit_level2b_cb(__u32 idx, struct num_context *ctx)
{
	ctx->i += 10;
	return 0;
}

static int iter_limit_level1_cb(__u32 idx, struct num_context *ctx)
{
	ctx->i += 1;
	bpf_loop(1, iter_limit_level2a_cb, ctx, 0);
	bpf_loop(1, iter_limit_level2b_cb, ctx, 0);
	return 0;
}

/* Check that path visiting every callback function once had been
 * reached by verifier. Variables 'ctx{1,2}i' below serve as flags,
 * with each decimal digit corresponding to a callback visit marker.
 */
SEC("socket")
__success __retval(111111)
int bpf_loop_iter_limit_nested(void *unused)
{
	struct num_context ctx1 = { .i = 0 };
	struct num_context ctx2 = { .i = 0 };
	__u64 a, b, c;

	bpf_loop(1, iter_limit_level1_cb, &ctx1, 0);
	bpf_loop(1, iter_limit_level1_cb, &ctx2, 0);
	a = ctx1.i;
	b = ctx2.i;
	/* Force 'ctx1.i' and 'ctx2.i' precise. */
	c = choice_arr[(a + b) % 2];
	/* This makes 'c' zero, but neither clang nor verifier know it. */
	c /= 10;
	/* Make sure that verifier does not visit 'impossible' states:
	 * enumerate all possible callback visit masks.
	 */
	if (a != 0 && a != 1 && a != 11 && a != 101 && a != 111 &&
	    b != 0 && b != 1 && b != 11 && b != 101 && b != 111)
		asm volatile ("r0 /= 0;" ::: "r0");
	return 1000 * a + b + c;
}

struct iter_limit_bug_ctx {
	__u64 a;
	__u64 b;
	__u64 c;
};

static __naked void iter_limit_bug_cb(void)
{
	/* This is the same as C code below, but written
	 * in assembly to control which branches are fall-through.
	 *
	 *   switch (bpf_get_prandom_u32()) {
	 *   case 1:  ctx->a = 42; break;
	 *   case 2:  ctx->b = 42; break;
	 *   default: ctx->c = 42; break;
	 *   }
	 */
	asm volatile (
	"r9 = r2;"
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"r2 = 42;"
	"r0 = 0;"
	"if r1 == 0x1 goto 1f;"
	"if r1 == 0x2 goto 2f;"
	"*(u64 *)(r9 + 16) = r2;"
	"exit;"
	"1: *(u64 *)(r9 + 0) = r2;"
	"exit;"
	"2: *(u64 *)(r9 + 8) = r2;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all
	);
}

int tmp_var;
SEC("socket")
__failure __msg("infinite loop detected at insn 2")
__naked void jgt_imm64_and_may_goto(void)
{
	asm volatile ("			\
	r0 = %[tmp_var] ll;		\
l0_%=:	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short -3; /* off -3 */		\
	.long 0; /* imm */		\
	if r0 > 10 goto l0_%=;		\
	r0 = 0;				\
	exit;				\
"	:: __imm_addr(tmp_var)
	: __clobber_all);
}

SEC("socket")
__failure __msg("infinite loop detected at insn 1")
__naked void may_goto_self(void)
{
	asm volatile ("			\
	r0 = *(u32 *)(r10 - 4);		\
l0_%=:	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short -1; /* off -1 */		\
	.long 0; /* imm */		\
	if r0 > 10 goto l0_%=;		\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

SEC("socket")
__success __retval(0)
__naked void may_goto_neg_off(void)
{
	asm volatile ("			\
	r0 = *(u32 *)(r10 - 4);		\
	goto l0_%=;			\
	goto l1_%=;			\
l0_%=:	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short -2; /* off -2 */		\
	.long 0; /* imm */		\
	if r0 > 10 goto l0_%=;		\
l1_%=:	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

SEC("tc")
__failure
__flag(BPF_F_TEST_STATE_FREQ)
int iter_limit_bug(struct __sk_buff *skb)
{
	struct iter_limit_bug_ctx ctx = { 7, 7, 7 };

	bpf_loop(2, iter_limit_bug_cb, &ctx, 0);

	/* This is the same as C code below,
	 * written in assembly to guarantee checks order.
	 *
	 *   if (ctx.a == 42 && ctx.b == 42 && ctx.c == 7)
	 *     asm volatile("r1 /= 0;":::"r1");
	 */
	asm volatile (
	"r1 = *(u64 *)%[ctx_a];"
	"if r1 != 42 goto 1f;"
	"r1 = *(u64 *)%[ctx_b];"
	"if r1 != 42 goto 1f;"
	"r1 = *(u64 *)%[ctx_c];"
	"if r1 != 7 goto 1f;"
	"r1 /= 0;"
	"1:"
	:
	: [ctx_a]"m"(ctx.a),
	  [ctx_b]"m"(ctx.b),
	  [ctx_c]"m"(ctx.c)
	: "r1"
	);
	return 0;
}

SEC("socket")
__success __retval(0)
__naked void ja_and_may_goto(void)
{
	asm volatile ("			\
l0_%=:	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 1; /* off 1 */		\
	.long 0; /* imm */		\
	goto l0_%=;			\
	r0 = 0;				\
	exit;				\
"	::: __clobber_common);
}

SEC("socket")
__success __retval(0)
__naked void ja_and_may_goto2(void)
{
	asm volatile ("			\
l0_%=:	r0 = 0;				\
	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 1; /* off 1 */		\
	.long 0; /* imm */		\
	goto l0_%=;			\
	r0 = 0;				\
	exit;				\
"	::: __clobber_common);
}

SEC("socket")
__success __retval(0)
__naked void jlt_and_may_goto(void)
{
	asm volatile ("			\
l0_%=:	call %[bpf_jiffies64];		\
	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 1; /* off 1 */		\
	.long 0; /* imm */		\
	if r0 < 10 goto l0_%=;		\
	r0 = 0;				\
	exit;				\
"	:: __imm(bpf_jiffies64)
	: __clobber_all);
}

#if (defined(__TARGET_ARCH_arm64) || defined(__TARGET_ARCH_x86) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64) || \
	defined(__TARGET_ARCH_arm) || defined(__TARGET_ARCH_s390) || \
	defined(__TARGET_ARCH_loongarch)) && \
	__clang_major__ >= 18
SEC("socket")
__success __retval(0)
__naked void gotol_and_may_goto(void)
{
	asm volatile ("			\
l0_%=:	r0 = 0;				\
	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 1; /* off 1 */		\
	.long 0; /* imm */		\
	gotol l0_%=;			\
	r0 = 0;				\
	exit;				\
"	::: __clobber_common);
}
#endif

SEC("socket")
__success __retval(0)
__naked void ja_and_may_goto_subprog(void)
{
	asm volatile ("			\
	call subprog_with_may_goto;	\
	exit;				\
"	::: __clobber_all);
}

static __naked __noinline __used
void subprog_with_may_goto(void)
{
	asm volatile ("			\
l0_%=:	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 1; /* off 1 */		\
	.long 0; /* imm */		\
	goto l0_%=;			\
	r0 = 0;				\
	exit;				\
"	::: __clobber_all);
}

#define ARR_SZ 1000000
int zero;
char arr[ARR_SZ];

SEC("socket")
__success __retval(0xd495cdc0)
int cond_break1(const void *ctx)
{
	unsigned long i;
	unsigned int sum = 0;

	for (i = zero; i < ARR_SZ && can_loop; i++)
		sum += i;
	for (i = zero; i < ARR_SZ; i++) {
		barrier_var(i);
		sum += i + arr[i];
		cond_break;
	}

	return sum;
}

SEC("socket")
__success __retval(999000000)
int cond_break2(const void *ctx)
{
	int i, j;
	int sum = 0;

	for (i = zero; i < 1000 && can_loop; i++)
		for (j = zero; j < 1000; j++) {
			sum += i + j;
			cond_break;
	}
	return sum;
}

static __noinline int loop(void)
{
	int i, sum = 0;

	for (i = zero; i <= 1000000 && can_loop; i++)
		sum += i;

	return sum;
}

SEC("socket")
__success __retval(0x6a5a2920)
int cond_break3(const void *ctx)
{
	return loop();
}

SEC("socket")
__success __retval(1)
int cond_break4(const void *ctx)
{
	int cnt = zero;

	for (;;) {
		/* should eventually break out of the loop */
		cond_break;
		cnt++;
	}
	/* if we looped a bit, it's a success */
	return cnt > 1 ? 1 : 0;
}

static __noinline int static_subprog(void)
{
	int cnt = zero;

	for (;;) {
		cond_break;
		cnt++;
	}

	return cnt;
}

SEC("socket")
__success __retval(1)
int cond_break5(const void *ctx)
{
	int cnt1 = zero, cnt2;

	for (;;) {
		cond_break;
		cnt1++;
	}

	cnt2 = static_subprog();

	/* main and subprog have to loop a bit */
	return cnt1 > 1 && cnt2 > 1 ? 1 : 0;
}

#define ARR2_SZ 1000
SEC(".data.arr2")
char arr2[ARR2_SZ];

SEC("socket")
__success __flag(BPF_F_TEST_STATE_FREQ)
int loop_inside_iter(const void *ctx)
{
	struct bpf_iter_num it;
	int *v, sum = 0;
	__u64 i = 0;

	bpf_iter_num_new(&it, 0, ARR2_SZ);
	while ((v = bpf_iter_num_next(&it))) {
		if (i < ARR2_SZ)
			sum += arr2[i++];
	}
	bpf_iter_num_destroy(&it);
	return sum;
}

SEC("socket")
__success __flag(BPF_F_TEST_STATE_FREQ)
int loop_inside_iter_signed(const void *ctx)
{
	struct bpf_iter_num it;
	int *v, sum = 0;
	long i = 0;

	bpf_iter_num_new(&it, 0, ARR2_SZ);
	while ((v = bpf_iter_num_next(&it))) {
		if (i < ARR2_SZ && i >= 0)
			sum += arr2[i++];
	}
	bpf_iter_num_destroy(&it);
	return sum;
}

volatile const int limit = ARR2_SZ;

SEC("socket")
__success __flag(BPF_F_TEST_STATE_FREQ)
int loop_inside_iter_volatile_limit(const void *ctx)
{
	struct bpf_iter_num it;
	int *v, sum = 0;
	__u64 i = 0;

	bpf_iter_num_new(&it, 0, ARR2_SZ);
	while ((v = bpf_iter_num_next(&it))) {
		if (i < limit)
			sum += arr2[i++];
	}
	bpf_iter_num_destroy(&it);
	return sum;
}

#define ARR_LONG_SZ 1000

SEC(".data.arr_long")
long arr_long[ARR_LONG_SZ];

SEC("socket")
__success
int test1(const void *ctx)
{
	long i;

	for (i = 0; i < ARR_LONG_SZ && can_loop; i++)
		arr_long[i] = i;
	return 0;
}

SEC("socket")
__success
int test2(const void *ctx)
{
	__u64 i;

	for (i = zero; i < ARR_LONG_SZ && can_loop; i++) {
		barrier_var(i);
		arr_long[i] = i;
	}
	return 0;
}

SEC(".data.arr_foo")
struct {
	int a;
	int b;
} arr_foo[ARR_LONG_SZ];

SEC("socket")
__success
int test3(const void *ctx)
{
	__u64 i;

	for (i = zero; i < ARR_LONG_SZ && can_loop; i++) {
		barrier_var(i);
		arr_foo[i].a = i;
		arr_foo[i].b = i;
	}
	return 0;
}

SEC("socket")
__success
int test4(const void *ctx)
{
	long i;

	for (i = zero + ARR_LONG_SZ - 1; i < ARR_LONG_SZ && i >= 0 && can_loop; i--) {
		barrier_var(i);
		arr_foo[i].a = i;
		arr_foo[i].b = i;
	}
	return 0;
}

char buf[10] SEC(".data.buf");

SEC("socket")
__description("check add const")
__success
__naked void check_add_const(void)
{
	/* typical LLVM generated loop with may_goto */
	asm volatile ("			\
	call %[bpf_ktime_get_ns];	\
	if r0 > 9 goto l1_%=;		\
l0_%=:	r1 = %[buf];			\
	r2 = r0;			\
	r1 += r2;			\
	r3 = *(u8 *)(r1 +0);		\
	.byte 0xe5; /* may_goto */	\
	.byte 0; /* regs */		\
	.short 4; /* off of l1_%=: */	\
	.long 0; /* imm */		\
	r0 = r2;			\
	r0 += 1;			\
	if r2 < 9 goto l0_%=;		\
	exit;				\
l1_%=:	r0 = 0;				\
	exit;				\
"	:
	: __imm(bpf_ktime_get_ns),
	  __imm_ptr(buf)
	: __clobber_common);
}

SEC("socket")
__failure
__msg("*(u8 *)(r7 +0) = r0")
__msg("invalid access to map value, value_size=10 off=10 size=1")
__naked void check_add_const_3regs(void)
{
	asm volatile (
	"r6 = %[buf];"
	"r7 = %[buf];"
	"call %[bpf_ktime_get_ns];"
	"r1 = r0;"              /* link r0.id == r1.id == r2.id */
	"r2 = r0;"
	"r1 += 1;"              /* r1 == r0+1 */
	"r2 += 2;"              /* r2 == r0+2 */
	"if r0 > 8 goto 1f;"    /* r0 range [0, 8]  */
	"r6 += r1;"             /* r1 range [1, 9]  */
	"r7 += r2;"             /* r2 range [2, 10] */
	"*(u8 *)(r6 +0) = r0;"  /* safe, within bounds   */
	"*(u8 *)(r7 +0) = r0;"  /* unsafe, out of bounds */
	"1: exit;"
	:
	: __imm(bpf_ktime_get_ns),
	  __imm_ptr(buf)
	: __clobber_common);
}

SEC("socket")
__failure
__msg("*(u8 *)(r8 -1) = r0")
__msg("invalid access to map value, value_size=10 off=10 size=1")
__naked void check_add_const_3regs_2if(void)
{
	asm volatile (
	"r6 = %[buf];"
	"r7 = %[buf];"
	"r8 = %[buf];"
	"call %[bpf_ktime_get_ns];"
	"if r0 < 2 goto 1f;"
	"r1 = r0;"              /* link r0.id == r1.id == r2.id */
	"r2 = r0;"
	"r1 += 1;"              /* r1 == r0+1 */
	"r2 += 2;"              /* r2 == r0+2 */
	"if r2 > 11 goto 1f;"   /* r2 range [0, 11] -> r0 range [-2, 9]; r1 range [-1, 10] */
	"if r0 s< 0 goto 1f;"   /* r0 range [0, 9] -> r1 range [1, 10]; r2 range [2, 11]; */
	"r6 += r0;"             /* r0 range [0, 9]  */
	"r7 += r1;"             /* r1 range [1, 10] */
	"r8 += r2;"             /* r2 range [2, 11] */
	"*(u8 *)(r6 +0) = r0;"  /* safe, within bounds   */
	"*(u8 *)(r7 -1) = r0;"  /* safe */
	"*(u8 *)(r8 -1) = r0;"  /* unsafe */
	"1: exit;"
	:
	: __imm(bpf_ktime_get_ns),
	  __imm_ptr(buf)
	: __clobber_common);
}

SEC("socket")
__failure
__flag(BPF_F_TEST_STATE_FREQ)
__naked void check_add_const_regsafe_off(void)
{
	asm volatile (
	"r8 = %[buf];"
	"call %[bpf_ktime_get_ns];"
	"r6 = r0;"
	"call %[bpf_ktime_get_ns];"
	"r7 = r0;"
	"call %[bpf_ktime_get_ns];"
	"r1 = r0;"              /* same ids for r1 and r0 */
	"if r6 > r7 goto 1f;"   /* this jump can't be predicted */
	"r1 += 1;"              /* r1.off == +1 */
	"goto 2f;"
	"1: r1 += 100;"         /* r1.off == +100 */
	"goto +0;"              /* verify r1.off in regsafe() after this insn */
	"2: if r0 > 8 goto 3f;" /* r0 range [0,8], r1 range either [1,9] or [100,108]*/
	"r8 += r1;"
	"*(u8 *)(r8 +0) = r0;"  /* potentially unsafe, buf size is 10 */
	"3: exit;"
	:
	: __imm(bpf_ktime_get_ns),
	  __imm_ptr(buf)
	: __clobber_common);
}

char _license[] SEC("license") = "GPL";
