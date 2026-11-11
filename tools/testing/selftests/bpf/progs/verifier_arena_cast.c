// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

struct task_struct {
	struct task_struct *group_leader;
	void *stack;
} __attribute__((preserve_access_index));
struct task_struct *bpf_task_acquire(struct task_struct *p) __ksym;
void bpf_task_release(struct task_struct *p) __ksym;

#ifdef __TARGET_ARCH_arm64
#define ARENA_VM_START (1ull << 32)
#else
#define ARENA_VM_START (1ull << 44)
#endif

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 4);
	__ulong(map_extra, ARENA_VM_START);
} arena SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, long long);
} hash SEC(".maps");

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_task_release(bpf_task_acquire(0));
}

/* dst = addr_space_cast(src, 0, 1) */
#define CAST_KERN(dst, src) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MOV | BPF_X, dst, src, BPF_ADDR_SPACE_CAST, 1)
/* dst = addr_space_cast(src, 1, 0) */
#define CAST_USER(dst, src) \
	BPF_RAW_INSN(BPF_ALU64 | BPF_MOV | BPF_X, dst, src, BPF_ADDR_SPACE_CAST, 1 << 16)

/* r0 = the element of the hash map or NULL. There is one after the update. */
#define UPDATE_AND_LOOKUP				\
	"r1 = 0;"					\
	"*(u32 *)(r10 - 4) = r1;"			\
	"r1 = 7;"					\
	"*(u64 *)(r10 - 16) = r1;"			\
	"r1 = %[hash] ll;"				\
	"r2 = r10;"					\
	"r2 += -4;"					\
	"r3 = r10;"					\
	"r3 += -16;"					\
	"r4 = 0;"					\
	"call %[bpf_map_update_elem];"			\
	"r1 = %[hash] ll;"				\
	"r2 = r10;"					\
	"r2 += -4;"					\
	"call %[bpf_map_lookup_elem];"

SEC("syscall")
__description("arena cast: a pointer to map value, it's a move")
__success __retval(7)
__xlated("r6 = r0")
__xlated("r0 = *(u64 *)(r6 +0)")
__naked void map_value(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 1f;"
	".8byte %[cast_r6_r0];"
	"r0 = *(u64 *)(r6 + 0);"
"1:"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r6_r0, CAST_KERN(BPF_REG_6, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a pointer to map value into the same register, it's removed")
__success __retval(7)
__xlated("r7 = r0")
__xlated("r0 = *(u64 *)(r0 +0)")
__xlated("exit")
__naked void map_value_same_reg(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 1f;"
	"r7 = r0;"
	".8byte %[cast_r0_r0];"
	"r0 = *(u64 *)(r0 + 0);"
"1:"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r0_r0, CAST_KERN(BPF_REG_0, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: bounds of map value are checked after the cast")
__failure __msg("invalid access to map value, value_size=8 off=8 size=8")
__naked void map_value_bounds(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 1f;"
	".8byte %[cast_r6_r0];"
	"r0 = *(u64 *)(r6 + 8);"
"1:"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r6_r0, CAST_KERN(BPF_REG_6, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a pointer that may be NULL has to be checked after the cast")
__failure __msg("R6 invalid mem access 'map_value_or_null'")
__naked void map_value_or_null(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	".8byte %[cast_r6_r0];"
	"r0 = *(u64 *)(r6 + 0);"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r6_r0, CAST_KERN(BPF_REG_6, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: NULL check of the pointer is for its copy too")
__success __retval(7)
__naked void map_value_null_check_of_src(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	".8byte %[cast_r6_r0];"
	"if r0 == 0 goto 1f;"
	"r0 = *(u64 *)(r6 + 0);"
"1:"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r6_r0, CAST_KERN(BPF_REG_6, BPF_REG_0))
	: __clobber_all);
}

SEC("tc")
__description("arena cast: a pointer to ctx")
__success __log_level(2)
__msg("R6=ctx()")
__naked void ctx(void)
{
	asm volatile (
	"r2 = %[arena] ll;"
	".8byte %[cast_r6_r1];"
	"r0 = *(u32 *)(r6 + %[len]);"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_const(len, offsetof(struct __sk_buff, len)),
	  __imm_insn(cast_r6_r1, CAST_KERN(BPF_REG_6, BPF_REG_1))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a pointer to task_struct")
__success __log_level(2)
__msg("R6=trusted_ptr_task_struct()")
__naked void btf_id(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"call %[bpf_get_current_task_btf];"
	".8byte %[cast_r6_r0];"
	"r0 = *(u32 *)(r6 + 0);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm(bpf_get_current_task_btf),
	  __imm_insn(cast_r6_r0, CAST_KERN(BPF_REG_6, BPF_REG_0))
	: __clobber_all);
}

/* r6 = addr_space_cast(p, 0, 1) */
#define CAST_R6(p)							\
	asm volatile (							\
	"r1 = %[ptr];"							\
	".8byte %[cast_r6_r1];"						\
	"r1 = %[arena] ll;"						\
	:								\
	: [ptr]"r"(p),							\
	  __imm_addr(arena),						\
	  __imm_insn(cast_r6_r1, CAST_KERN(BPF_REG_6, BPF_REG_1))	\
	: "r1", "r6")

/* the prog can sleep: what it reads from task_struct outside of an RCU section is untrusted */
SEC("syscall")
__description("arena cast: an untrusted pointer to task_struct, it's a move")
__success __log_level(2)
__msg("R6=untrusted_ptr_task_struct()")
__xlated("r6 = r1")
int untrusted_btf_id(void *ctx)
{
	CAST_R6(bpf_get_current_task_btf()->group_leader);
	return 0;
}

SEC("syscall")
__description("arena cast: an untrusted pointer to kernel memory, it's a move")
__success __log_level(2)
__msg("R6=rdonly_untrusted_mem(sz=0)")
__xlated("r6 = r1")
int untrusted_mem(void *ctx)
{
	CAST_R6(bpf_get_current_task_btf()->stack);
	return 0;
}

SEC("syscall")
__description("arena cast: a reference is released through the copy")
__success __retval(0)
__naked void ref_released(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"call %[bpf_get_current_task_btf];"
	"r1 = r0;"
	"call %[bpf_task_acquire];"
	"if r0 == 0 goto 1f;"
	".8byte %[cast_r1_r0];"
	"call %[bpf_task_release];"
"1:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm(bpf_get_current_task_btf),
	  __imm(bpf_task_acquire),
	  __imm(bpf_task_release),
	  __imm_insn(cast_r1_r0, CAST_KERN(BPF_REG_1, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a reference doesn't go away")
__failure __msg("Unreleased reference")
__naked void ref_leaked(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"call %[bpf_get_current_task_btf];"
	"r1 = r0;"
	"call %[bpf_task_acquire];"
	"if r0 == 0 goto 1f;"
	".8byte %[cast_r0_r0];"
"1:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm(bpf_get_current_task_btf),
	  __imm(bpf_task_acquire),
	  __imm_insn(cast_r0_r0, CAST_KERN(BPF_REG_0, BPF_REG_0))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a number")
__success __log_level(2)
__msg("R6=arena")
__xlated("w6 = w6")
__naked void number(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r6 = %[addr] ll;"
	".8byte %[cast_r6_r6];"
	"r0 = *(u64 *)(r6 + 0);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_const(addr, ARENA_VM_START),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

/* static analysis of the stack follows the pointer through the cast */
SEC("syscall")
__description("arena cast: a pointer to stack")
__success __retval(7) __log_level(2)
__msg("R6=fp-8")
__naked void stack(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r6 = r10;"
	"r6 += -8;"
	".8byte %[cast_r6_r6];"
	"r0 = 7;"
	"*(u64 *)(r6 + 0) = r0;"
	"r0 = *(u64 *)(r10 - 8);"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a pointer is spilled through the cast of a pointer to stack")
__success
__naked void spill_through_cast(void)
{
	asm volatile (
	"r2 = %[arena] ll;"
	"r6 = r10;"
	"r6 += -8;"
	".8byte %[cast_r6_r6];"
	"*(u64 *)(r6 + 0) = r1;"
	"r2 = *(u64 *)(r10 - 8);"
	/* it's ctx */
	"r0 = *(u32 *)(r2 + 0);"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

static __naked __noinline __used long writes_through_cast(long p)
{
	asm volatile (
	".8byte %[cast_r1_r1];"
	"r0 = 7;"
	"*(u64 *)(r1 + 0) = r0;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(cast_r1_r1, CAST_KERN(BPF_REG_1, BPF_REG_1))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: the callee writes to the stack of the caller")
__success __retval(7)
__naked void callee_writes(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r0 = 0;"
	"*(u64 *)(r10 - 8) = r0;"
	"r1 = r10;"
	"r1 += -8;"
	"call writes_through_cast;"
	"r0 = *(u64 *)(r10 - 8);"
	"exit;"
	:
	: __imm_addr(arena)
	: __clobber_all);
}

static __naked __noinline __used long reads_through_cast(long p)
{
	asm volatile (
	"r6 = r1;"
	/* a place where what is not live is forgotten */
	"call %[bpf_get_prandom_u32];"
	"if r0 == 0 goto +0;"
	".8byte %[cast_r6_r6];"
	"r0 = *(u64 *)(r6 + 0);"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: the callee reads the stack of the caller")
__success __retval(5)
__naked void callee_reads(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r0 = 5;"
	"*(u64 *)(r10 - 8) = r0;"
	"r1 = r10;"
	"r1 += -8;"
	"call reads_through_cast;"
	"exit;"
	:
	: __imm_addr(arena)
	: __clobber_all);
}

/*
 * r7 = a number that static analysis of the stack takes for fp + off: fp-16 holds fp + off,
 * a helper overwrites fp-16, and the analysis doesn't follow what helpers write.
 */
#define STALE_SPILL_R7(off)				\
	"r1 = r10;"					\
	"r1 += " #off ";"				\
	"*(u64 *)(r10 - 16) = r1;"			\
	"r1 = r10;"					\
	"r1 += -16;"					\
	"r2 = 8;"					\
	"call %[bpf_get_current_comm];"			\
	"r7 = *(u64 *)(r10 - 16);"

#define SPILL_FP24_TO_FP8				\
	"r1 = r10;"					\
	"r1 += -24;"					\
	"*(u64 *)(r10 - 8) = r1;"

/*
 * fp-24 counts 0, 1, ... 7 and it's the offset in the map value in r9. fp-8 holds &fp[-24],
 * and the store through r8 changes the offset. When the analysis loses the pointer in fp-8,
 * it doesn't see that store, and the verifier takes 0 ... 7 for all values of fp-24.
 */
#define LOOP_WRITES_FP24_THROUGH_FP8			\
	"r8 = *(u64 *)(r10 - 8);"			\
	"r1 = 0;"					\
	"*(u64 *)(r10 - 24) = r1;"			\
	"r6 = 0;"					\
"1:"							\
	"r1 = *(u64 *)(r10 - 24);"			\
	"r2 = r9;"					\
	"r2 += r1;"					\
	"r0 = *(u8 *)(r2 + 0);"				\
	"r1 += 1;"					\
	"*(u64 *)(r10 - 24) = r1;"			\
	"r3 = 4096;"					\
	"*(u64 *)(r8 + 0) = r3;"			\
	"r6 += 1;"					\
	"if r6 < 8 goto 1b;"

SEC("syscall")
__description("arena cast: a store to arena doesn't replace what is spilled to the stack")
__failure __msg("invalid access to map value, value_size=8 off=4096 size=1")
__naked void stale_spill_store(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 2f;"
	"r9 = r0;"
	STALE_SPILL_R7(-8)
	".8byte %[cast_r7_r7];"
	SPILL_FP24_TO_FP8
	/* to arena, not to fp-8 */
	"r0 = 0;"
	"*(u64 *)(r7 + 0) = r0;"
	LOOP_WRITES_FP24_THROUGH_FP8
"2:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm(bpf_get_current_comm),
	  __imm_insn(cast_r7_r7, CAST_KERN(BPF_REG_7, BPF_REG_7))
	: __clobber_all);
}

/* fp+0 is what the analysis adds to fp-N to say "or not a pointer to stack" */
SEC("syscall")
__description("arena cast: a store to arena doesn't replace what is spilled, stale r10")
__failure __msg("invalid access to map value, value_size=8 off=4096 size=1")
__naked void stale_spill_store_r10(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 2f;"
	"r9 = r0;"
	STALE_SPILL_R7(0)
	".8byte %[cast_r7_r7];"
	SPILL_FP24_TO_FP8
	"r0 = 0;"
	"*(u64 *)(r7 - 8) = r0;"
	LOOP_WRITES_FP24_THROUGH_FP8
"2:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm(bpf_get_current_comm),
	  __imm_insn(cast_r7_r7, CAST_KERN(BPF_REG_7, BPF_REG_7))
	: __clobber_all);
}

/* compilers use a slot for two variables: a pointer to a local, then the buffer of a helper */
SEC("syscall")
__description("arena cast: a stack slot is live over a store to arena")
__success
__naked void stale_spill_live_slot(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 3f;"
	"r9 = r0;"
	"call %[bpf_get_prandom_u32];"
	"r6 = r0;"
	STALE_SPILL_R7(-8)
	".8byte %[cast_r7_r7];"
	"r1 = 0;"
	"if r6 == 0 goto 1f;"
	"r1 = 7;"
"1:"
	"*(u64 *)(r10 - 8) = r1;"
	/* a place where what is not live is forgotten */
	"if r6 == 1 goto +0;"
	"r1 = 0;"
	"*(u64 *)(r7 + 0) = r1;"
	"r1 = *(u64 *)(r10 - 8);"
	"r9 += r1;"
	"r0 = *(u8 *)(r9 + 0);"
"3:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm(bpf_get_current_comm),
	  __imm(bpf_get_prandom_u32),
	  __imm_insn(cast_r7_r7, CAST_KERN(BPF_REG_7, BPF_REG_7))
	: __clobber_all);
}

/* r1 is a number, r2 is &fp[-8] of the caller */
static __naked __noinline __used long writes_arena_reads_stack(long a, long *p)
{
	asm volatile (
	".8byte %[cast_r1_r1];"
	"r0 = 0;"
	"*(u64 *)(r1 + 0) = r0;"
	"r0 = *(u64 *)(r2 + 0);"
	"exit;"
	:
	: __imm_insn(cast_r1_r1, CAST_KERN(BPF_REG_1, BPF_REG_1))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: a stack slot of the caller is live over a store to arena in the callee")
__success
__naked void stale_spill_live_slot_of_caller(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 3f;"
	"r9 = r0;"
	"call %[bpf_get_prandom_u32];"
	"r6 = r0;"
	STALE_SPILL_R7(-8)
	"r1 = 0;"
	"if r6 == 0 goto 1f;"
	"r1 = 7;"
"1:"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = r7;"
	"r2 = r10;"
	"r2 += -8;"
	"call writes_arena_reads_stack;"
	"r9 += r0;"
	"r0 = *(u8 *)(r9 + 0);"
"3:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm(bpf_get_current_comm),
	  __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

/*
 * A store through the result of the cast can miss the stack: the slot keeps what it had, or
 * it has the new value. Static analysis must not make "it's fp+0" from
 * "it's fp+0, or it's not a pointer to stack".
 */
SEC("syscall")
__description("arena cast: a pointer to map value is stored over r10 through the cast")
__failure __msg("invalid access to map value, value_size=8 off=4096 size=1")
__naked void store_over_spilled_r10(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 2f;"
	"r9 = r0;"
	"*(u64 *)(r10 - 16) = r10;"
	"r7 = r10;"
	"r7 += -16;"
	".8byte %[cast_r7_r7];"
	"r1 = r9;"
	"r1 += 8;"
	"*(u64 *)(r7 + 0) = r1;"
	/* it's the map value, not r10 */
	"r7 = *(u64 *)(r10 - 16);"
	SPILL_FP24_TO_FP8
	"r0 = 0;"
	"*(u64 *)(r7 - 8) = r0;"
	LOOP_WRITES_FP24_THROUGH_FP8
"2:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cast_r7_r7, CAST_KERN(BPF_REG_7, BPF_REG_7))
	: __clobber_all);
}

SEC("syscall")
__description("arena cast: r10 is stored to arena, a pointer to map value is in the stack")
__failure __msg("invalid access to map value, value_size=8 off=4096 size=1")
__naked void r10_stored_to_arena(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	UPDATE_AND_LOOKUP
	"if r0 == 0 goto 2f;"
	"r9 = r0;"
	STALE_SPILL_R7(-8)
	".8byte %[cast_r7_r7];"
	"r1 = r9;"
	"r1 += 8;"
	"*(u64 *)(r10 - 32) = r1;"
	/* to arena, not to fp-32 */
	"*(u64 *)(r7 + 0) = r10;"
	"r7 = *(u64 *)(r10 - 32);"
	SPILL_FP24_TO_FP8
	"r0 = 0;"
	"*(u64 *)(r7 - 8) = r0;"
	LOOP_WRITES_FP24_THROUGH_FP8
"2:"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_addr(hash),
	  __imm(bpf_map_update_elem),
	  __imm(bpf_map_lookup_elem),
	  __imm(bpf_get_current_comm),
	  __imm_insn(cast_r7_r7, CAST_KERN(BPF_REG_7, BPF_REG_7))
	: __clobber_all);
}

static __naked __noinline __used long fills_through_cast(long *p, long v)
{
	asm volatile (
	".8byte %[cast_r1_r1];"
	"*(u64 *)(r1 + 0) = r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(cast_r1_r1, CAST_KERN(BPF_REG_1, BPF_REG_1))
	: __clobber_all);
}

/* the callee writes to fp-16 only, the verifier still can count the loop */
SEC("syscall")
__description("arena cast: the callee writes to a local, the counter of the loop is in the stack")
__success
__naked void callee_writes_in_loop(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
"1:"
	"r1 = r10;"
	"r1 += -16;"
	"r2 = *(u64 *)(r10 - 8);"
	"call fills_through_cast;"
	"r1 = *(u64 *)(r10 - 8);"
	"r1 += 1;"
	"*(u64 *)(r10 - 8) = r1;"
	"if r1 < 1000000 goto 1b;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena)
	: __clobber_all);
}

SEC("tc")
__description("arena cast: a number and a pointer at the same insn")
__failure __msg("same addr_space_cast insn cannot be used with a number and with a pointer")
__naked void number_and_pointer(void)
{
	asm volatile (
	"r2 = %[arena] ll;"
	"r6 = r1;"
	"call %[bpf_get_prandom_u32];"
	"if r0 == 0 goto 1f;"
	"r6 = 4096;"
"1:"
	".8byte %[cast_r6_r6];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm(bpf_get_prandom_u32),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

SEC("tc")
__description("arena cast: a pointer and a number at the same insn")
__failure __msg("same addr_space_cast insn cannot be used with a number and with a pointer")
__naked void pointer_and_number(void)
{
	asm volatile (
	"r2 = %[arena] ll;"
	"r6 = 4096;"
	"r7 = r1;"
	"call %[bpf_get_prandom_u32];"
	"if r0 == 0 goto 1f;"
	"r6 = r7;"
"1:"
	".8byte %[cast_r6_r6];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm(bpf_get_prandom_u32),
	  __imm_insn(cast_r6_r6, CAST_KERN(BPF_REG_6, BPF_REG_6))
	: __clobber_all);
}

SEC("tc")
__description("arena cast: the cast to as(1) of a pointer is a number")
__success __log_level(2)
__msg("R6=scalar()")
__naked void cast_user(void)
{
	asm volatile (
	"r2 = %[arena] ll;"
	".8byte %[cast_r6_r1];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_addr(arena),
	  __imm_insn(cast_r6_r1, CAST_USER(BPF_REG_6, BPF_REG_1))
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
