// SPDX-License-Identifier: GPL-2.0-only
/*
 * The main pass of the verifier as a fixpoint over the control flow graph.
 *
 * Every insn that starts a basic block has a set of states. The state that
 * arrives there is
 * - dropped, if it is within a state of the set,
 * - joined with a state of the set, if there is a join, see bpf_states_join(),
 *   that doesn't make up what neither of them has, see flow_join_makes_up(),
 * - added to the set otherwise.
 * What is new in the set is pending. Pending states are processed in the order
 * of the program in which a loop is done with before what follows it, see
 * flow_compute_order(). The pass ends when nothing is pending.
 *
 * Loops. The state that enters a loop is made to stand for all iterations with
 * what scalar evolution knows about the loop, see bpf_flow_loop_enter().
 * The state that comes back to the loop head is expected to be within it.
 * If it is not, the two are joined, what grew is forgotten, and the loop is
 * done again.
 *
 * Functions. A static function that gets nothing but numbers and pointers to
 * what is not the stack is verified on its own, for the states at its entry
 * and not for every chain of calls. The call waits. What comes out of the
 * function goes to all calls that wait for the states it comes from.
 */
#include <linux/bpf.h>
#include <linux/bpf_verifier.h>
#include <linux/filter.h>
#include <linux/jhash.h>
#include <linux/rbtree.h>
#include <linux/slab.h>

#define verbose(env, fmt, args...) bpf_verifier_log_write(env, fmt, ##args)

/*
 * States at an insn that have no join: kinds of values, references, chains of calls.
 * A state that arrives is compared with those that it may be related to, see flow_may_relate().
 * At worst that is all of them, and the work is the square of it.
 */
#define BPF_FLOW_MAX_STATES	1024
/*
 * All states in the sets at a time: that many for every insn of the prog, and some. Not counted:
 * calls that wait, what came out of a function, the states that loops were entered with.
 */
#define BPF_FLOW_LIVE_PER_INSN	16
#define BPF_FLOW_LIVE_BASE	4096
/* All states that were ever there. It bounds the work by the size of the prog. */
#define BPF_FLOW_TOTAL_PER_INSN	256
#define BPF_FLOW_TOTAL_BASE	65536
/*
 * What comes out of a function is joined that many times. Then it is widened, like what
 * comes back to a loop head: the exit states have to stop changing.
 */
#define BPF_FLOW_MAX_JOINS	16
/* With fewer states at an insn it's faster to compare them than to look at their shapes. */
#define BPF_FLOW_MIN_SHAPES	8
/* bpf_loop() with that many iterations at most is walked. */
#define BPF_FLOW_MAX_CALLBACKS	8
/* What a callback returns to the helper with is joined that many times before it is widened. */
#define BPF_FLOW_CALLBACK_JOINS	4

enum flow_cycle {
	FLOW_NO_CYCLE,
	/* what grew is forgotten */
	FLOW_CYCLE,
	/* after a while. A flag that a callback sets doesn't grow any further. */
	FLOW_CALLBACK_CYCLE,
};

/* See flow_may_relate() */
enum flow_word {
	FLOW_CHAIN,
	FLOW_CTX,
	FLOW_LAYOUT,
	FLOW_PTRS,
	FLOW_OFFS,
	FLOW_SHAPE_WORDS,
};

struct flow_shape {
	u32 word[FLOW_SHAPE_WORDS];
	/* how much of the packet is checked, the sum over the pointers to the packet */
	u32 pkt_range;
	/* what comes after FLOW_CTX is computed */
	bool places;
};

struct flow_state {
	struct bpf_verifier_state st;
	/* states at the insn, see struct flow_set, calls of struct flow_entry */
	struct list_head node;
	struct rb_node rb;		/* pending states */
	int prev_insn_idx;
	u32 seq;
	struct flow_shape shape;
	u16 joins;
	bool pending;
	/* the order of pending states, see flow_set_pending() */
	u8 key_len;
	u32 key[MAX_CALL_FRAMES + 1];
};

/* what came out of a function that is verified on its own */
struct flow_exit {
	struct list_head node;
	struct bpf_verifier_state st;
	u16 joins;
};

/*
 * Every call that waits makes an entry of the function: the state at its first insn and what
 * becomes of it. Entries whose states met are one set: what comes out of the function for one of
 * them is for the calls of all of them. One entry stands for the set and has its lists.
 */
struct flow_entry {
	struct flow_entry *up;		/* towards the one that stands for the set: up == itself */
	struct list_head calls;		/* struct flow_state: the calls that wait */
	struct list_head exits;		/* struct flow_exit: what came out */
};

struct flow_func {
	int exit_idx;			/* all exits are compared as if they were here */
};

/* States at an insn, in the order in which they came. */
struct flow_set {
	struct list_head list;
};

struct flow_united {
	u32 a, b;
};

struct flow_kept {
	struct list_head node;
	struct bpf_verifier_state st;
};

/* What the checks ahead of an insn look at, see enum flow_need_kind */
struct flow_need {
	u64 slots;			/* of the stack, fp-8 is bit 0 */
	u16 regs;
};

/* see flow_compute_need_const() */
enum flow_need_kind {
	FLOW_NEED_ARG,
	FLOW_NEED_STACK_OFF,
	FLOW_NEED_PTR_OFF,
	FLOW_NEED_JUMP,
	FLOW_NEED_VALUE_OFF,
	FLOW_NEED_BASE,
	FLOW_NEED_JUMP_ALU,
	FLOW_NEED_KINDS,
};

struct bpf_flow {
	/* computed once */
	u32 len;			/* of the prog during the pass */
	u32 nr_funcs;
	u32 *pos;			/* place of the insn in the order */
	u32 *rank;			/* callers go first */
	struct flow_need *need[FLOW_NEED_KINDS];
	unsigned long *point;		/* insn starts a basic block */
	unsigned long *keep;		/* states that were processed are compared with */

	/* of the pass */
	struct flow_set *states;	/* per insn */
	struct flow_set *enter;		/* per insn: the states that enter the loop at its head */
	u32 *nr_states;			/* per insn: the two sets are counted together */
	struct rb_root_cached pending;
	struct flow_func *funcs;
	struct flow_entry **entries;	/* bpf_verifier_state.flow_entry is the index, 0 is none */
	u32 entry_cnt, entry_cap;
	struct flow_united *united;
	u32 united_cnt, united_cap;
	struct list_head kept;
	struct bpf_verifier_state tmp;
	u32 seq;
	u32 live_states;
};
