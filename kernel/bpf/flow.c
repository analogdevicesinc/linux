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
	u32 from;			/* seq of the last state on the way to it that is kept */
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
	FLOW_NEED_FLAG,
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
	u32 from;			/* flow_state.from of what comes from the current state */
	u32 live_states;
};

/*
 * The order. Reverse post order is a topological order of the graph without
 * back edges, but it may put what follows a loop between the insns of
 * the loop. Keep the insns of a loop together, the head first: in the list of
 * its parent the loop takes the place of the insn of it that comes first.
 *
 * Every cycle has a back edge of the depth first search in it and
 * bpf_compute_loops() makes the target of such edge a loop head. So every edge
 * that goes backwards in the order is in a loop whose head is at or before its
 * target, loops that are not reducible included.
 */
static int flow_compute_order(struct bpf_verifier_env *env, struct bpf_flow *flow)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int len = env->prog->len, i, s, n, h, top, depth;
	int *next, *head, *tail, *stack;
	unsigned long *linked;
	u32 pos = 0;
	int err = -ENOMEM;

	/* lists of members are indexed by loop head, len is the function */
	next = kvmalloc_array(len, sizeof(int), GFP_KERNEL_ACCOUNT);
	head = kvmalloc_array(len + 1, sizeof(int), GFP_KERNEL_ACCOUNT);
	tail = kvmalloc_array(len + 1, sizeof(int), GFP_KERNEL_ACCOUNT);
	stack = kvmalloc_array(len + 1, sizeof(int), GFP_KERNEL_ACCOUNT);
	linked = kvcalloc(BITS_TO_LONGS(len), sizeof(long), GFP_KERNEL_ACCOUNT);
	if (!next || !head || !tail || !stack || !linked)
		goto out;

	for (i = 0; i <= len; i++)
		head[i] = tail[i] = -1;

	for (s = 0; s < env->subprog_cnt; s++) {
		int first = env->subprog_info[s].postorder_start;
		int last = env->subprog_info[s + 1].postorder_start - 1;

		head[len] = tail[len] = -1;
		for (i = last; i >= first; i--) {
			n = env->cfg.insn_postorder[i];
			/* n, then the heads of the loops around it that are not in a list yet */
			depth = 0;
			for (h = n; h >= 0 && !test_bit(h, linked); h = aux[h].loop_header)
				stack[depth++] = h;
			while (depth) {
				h = stack[--depth];
				top = aux[h].loop_header >= 0 ? aux[h].loop_header : len;
				__set_bit(h, linked);
				next[h] = -1;
				if (tail[top] < 0)
					head[top] = h;
				else
					next[tail[top]] = h;
				tail[top] = h;
			}
		}

		depth = 0;
		n = head[len];
		while (n >= 0 || depth) {
			if (n < 0) {
				n = next[stack[--depth]];
				continue;
			}
			flow->pos[n] = pos++;
			if (aux[n].loop && head[n] >= 0) {
				stack[depth++] = n;
				n = head[n];
			} else {
				n = next[n];
			}
		}
	}
	err = 0;
out:
	kvfree(next);
	kvfree(head);
	kvfree(tail);
	kvfree(stack);
	kvfree(linked);
	return err;
}

/* Does the insn at @idx stop the loop @h? */
static bool flow_is_bound(struct bpf_verifier_env *env, int h, int idx)
{
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	int t;

	/* Once it is out of budget it's a jump. It has to leave the loop. */
	if (bpf_is_may_goto_insn(insn)) {
		t = bpf_loop_at_index(env, idx + insn->off + 1);
		return t != h && !bpf_is_nested_loop(env, t, h);
	}
	if (!bpf_insn_is_iter_next(env, insn))
		return false;
	/* unless the main pass finds that it doesn't, see iter_stops_loops() */
	t = env->insn_aux_data[idx].not_stopped;
	return t < 0 || bpf_is_nested_loop(env, h, t);
}

/*
 * What stops a loop at run time is may_goto that leaves it, or bpf_iter_*_next() of
 * an iterator that is older than the loop, that every trip goes through. A loop without
 * it gets may_goto unless the number of its trips turns out to be known.
 *
 * The place for it is in front of a conditional jump that leaves the loop, with
 * the same target. The registers and the stack are what they are when
 * the program leaves the loop there. One such jump that every trip goes
 * through is enough. Otherwise every back edge needs one.
 */
static void flow_set_guard(struct bpf_verifier_env *env, int h, int n)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int t = n + env->prog->insnsi[n].off + 1;
	int i = bpf_loop_at_index(env, t);

	if (i == h || bpf_is_nested_loop(env, i, h))
		t = n + 1;
	aux[n].guard_place = true;
	aux[n].guard_to = t;
	aux[h].loop->has_guards = true;
}

static void flow_place_guards(struct bpf_verifier_env *env, int h, u32 *seen)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_loop *loop = aux[h].loop;
	int i, n, o, t, found = -1;

	if (loop->backedges_overflow)
		goto header;

	/*
	 * The head of a loop with more than one entry doesn't dominate what is in the loop.
	 * Then an insn that dominates a back edge can be passed once, on the way in, and
	 * never again. Every trip goes through the jump that is the back edge, and
	 * through the head.
	 */
	if (loop->irreducible) {
		for (i = 0; i < loop->backedges_cnt; i++)
			if (!loop->backedges[i].latches_cnt ||
			    loop->backedges[i].latches[0] != loop->backedges[i].from)
				goto header;
		for (i = 0; i < loop->backedges_cnt; i++)
			flow_set_guard(env, h, loop->backedges[i].from);
		return;
	}

	/* seen[n]: the number of back edges that n dominates */
	for (i = 0; i < loop->backedges_cnt; i++)
		for (n = loop->backedges[i].from; n >= 0; n = n == h ? -1 : env->idoms[n])
			seen[n]++;
	for (n = loop->backedges[0].from; n >= 0 && found < 0; n = n == h ? -1 : env->idoms[n])
		if (seen[n] == loop->backedges_cnt && bpf_insn_leaves_loop(env, h, n))
			found = n;
	for (i = 0; i < loop->backedges_cnt; i++)
		for (n = loop->backedges[i].from; n >= 0; n = n == h ? -1 : env->idoms[n])
			seen[n] = 0;
	if (found >= 0) {
		flow_set_guard(env, h, found);
		return;
	}

	for (i = 0; i < loop->backedges_cnt; i++)
		if (!loop->backedges[i].latches_cnt)
			goto header;
	for (i = 0; i < loop->backedges_cnt; i++)
		flow_set_guard(env, h, loop->backedges[i].latches[0]);
	return;

header:
	/*
	 * In front of a jump may_goto goes where the jump goes. From the head to where the loop
	 * is left is not an edge of the program: it skips what is in the loop. If that is what
	 * stops a loop around this one, bpf_iter_*_next() or i++, and the exit leads back to
	 * the head of that loop, the program goes round and round from may_goto to may_goto.
	 * So leave all loops.
	 *
	 * What is derived from the graph in the direction of the edges has to know about this
	 * one, see bpf_head_guard_target().
	 */
	for (o = h; aux[o].loop_header >= 0; o = aux[o].loop_header)
		;
	/* a loop that has no exit can only be stopped together with the program */
	if (!aux[o].loop->exits_cnt)
		return;
	t = aux[o].loop->exits[0].to;
	aux[h].guard_place = true;
	aux[h].guard_at_header = true;
	aux[h].guard_to = t;
	loop->has_guards = true;
}

/* Does every trip of the loop @h go through an insn that stops it? */
static bool flow_loop_is_bounded(struct bpf_verifier_env *env, int h)
{
	struct bpf_loop *loop = env->insn_aux_data[h].loop;
	int i, n;

	/* see flow_place_guards() about loops with more than one entry */
	if (loop->backedges_overflow || loop->irreducible)
		return false;
	for (i = 0; i < loop->backedges_cnt; i++) {
		for (n = loop->backedges[i].from; n >= 0 && n != h; n = env->idoms[n])
			if (flow_is_bound(env, h, n))
				break;
		if (n < 0 || !flow_is_bound(env, h, n))
			return false;
	}
	return true;
}

/*
 * The main pass got to bpf_iter_*_next() at @insn_idx in a state in which it doesn't stop
 * the loop @h around it. What is assumed about the loops stays as it is until the pass
 * is over, so that all states are treated alike.
 */
void bpf_flow_next_is_no_bound(struct bpf_verifier_env *env, int insn_idx, int h)
{
	struct bpf_insn_aux_data *aux = &env->insn_aux_data[insn_idx];

	if (aux->not_stopped >= 0 && !bpf_is_nested_loop(env, h, aux->not_stopped))
		return;
	aux->not_stopped = h;
	env->restart = true;
}

/*
 * True if a loop was taken for bounded in the pass and is not. The pass is done again then.
 * A loop that has something else that stops it, like the outer one of two nested
 * bpf_for(), is not affected. Every time at least one loop is found out for good.
 */
bool bpf_flow_loops_lost_bound(struct bpf_verifier_env *env)
{
	struct bpf_loop *loop;
	int h;

	for (h = 0; h < env->prog->len; h++) {
		loop = env->insn_aux_data[h].loop;
		if (loop && loop->rt_bounded && !flow_loop_is_bounded(env, h))
			return true;
	}
	return false;
}

/*
 * A state is in the loop @h without a count, so the loop gets may_goto. At run time may_goto is
 * there for everybody, also for those who know the number of trips or got to the end of the loop
 * trip by trip. It can be out of budget in the first trip, because of a loop before this one.
 *
 * So a loop has may_goto or it has not, and a state takes the edges of may_goto if the loop had
 * it when the state entered. If a state entered without it, with a count or on a walk that may
 * give up, all is done again, and the loop has may_goto from the start.
 */
void bpf_flow_loop_gets_guards(struct bpf_verifier_env *env, int h)
{
	struct bpf_loop *loop = env->insn_aux_data[h].loop;

	if (loop->guarded)
		return;
	loop->guarded = true;
	if (loop->no_guard_edges)
		env->guards_late = true;
}

/* The main prog and the global functions are about to be verified, maybe not for the first time */
void bpf_flow_new_round(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int i;

	env->restart = false;
	env->guards_late = false;
	for (i = 0; i < env->prog->len; i++) {
		aux[i].guard_edge_dropped = false;
		if (!aux[i].loop)
			continue;
		aux[i].loop->no_guard_edges = false;
		/* what the last round found out */
		if (aux[i].loop->rt_bounded)
			aux[i].loop->rt_bounded = flow_loop_is_bounded(env, i);
	}
}

/*
 * Which loops have something that stops them at run time, and where may_goto goes in the others.
 * Returns 1 if a place is at the head of a loop.
 */
int bpf_flow_place_guards(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int len = env->prog->len, h, i, depth, ret = 0;
	struct bpf_loop *loop;
	u32 *seen;

	seen = kvcalloc(len, sizeof(*seen), GFP_KERNEL_ACCOUNT);
	if (!seen)
		return -ENOMEM;

	for (i = 0; i < len; i++) {
		aux[i].not_stopped = -1;
		depth = 0;
		for (h = bpf_loop_at_index(env, i); h >= 0; h = aux[h].loop_header) {
			/* the main pass doesn't get that deep */
			if (++depth > LOOP_STACK_SIZE) {
				verbose(env, "Too many nested loops at %d\n", i);
				kvfree(seen);
				return -E2BIG;
			}
			aux[h].loop->nr_insns++;
			if (bpf_pseudo_call(&env->prog->insnsi[i]) ||
			    bpf_is_callx(&env->prog->insnsi[i]))
				aux[h].loop->has_calls = true;
		}
	}

	for (h = 0; h < len; h++) {
		loop = aux[h].loop;
		if (!loop)
			continue;
		/* a deep nest of loops takes long */
		if (signal_pending(current)) {
			ret = -EAGAIN;
			break;
		}
		cond_resched();
		loop->rt_bounded = flow_loop_is_bounded(env, h);
		/* also when it's bounded: the main pass may find that it's not */
		flow_place_guards(env, h, seen);
		if (aux[h].guard_at_header)
			ret = 1;
	}
	kvfree(seen);
	return ret;
}

/* The slot that @insn spills to or fills from, if it's one that struct flow_need has a bit for */
static int flow_fp_slot(const struct bpf_insn *insn, u8 base)
{
	u8 mode = BPF_MODE(insn->code);

	if (base != BPF_REG_FP || (mode != BPF_MEM && mode != BPF_MEMSX) ||
	    insn->off >= 0 || insn->off < -64 * BPF_REG_SIZE || insn->off % BPF_REG_SIZE)
		return -1;
	return -insn->off / BPF_REG_SIZE - 1;
}

/*
 * A number is added to a pointer to the stack. A store at an offset that is not known
 * forgets what the slots it may hit hold, pointers above all.
 */
static u16 flow_stack_off_regs(struct bpf_verifier_env *env, int idx)
{
	u16 ptrs = env->insn_aux_data[idx].stack_ptrs;
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	u8 op = BPF_OP(insn->code);

	if (BPF_CLASS(insn->code) != BPF_ALU64 || BPF_SRC(insn->code) != BPF_X ||
	    (op != BPF_ADD && op != BPF_SUB))
		return 0;
	if ((ptrs & BIT(insn->dst_reg)) && !(ptrs & BIT(insn->src_reg)))
		return BIT(insn->src_reg);
	if (op == BPF_ADD && (ptrs & BIT(insn->src_reg)) && !(ptrs & BIT(insn->dst_reg)))
		return BIT(insn->dst_reg);
	return 0;
}

/*
 * A jump that compares a register with a constant, which may be in the other register.
 * A call does the same: the verifier compares a size, or what bpf_set_retval() or bpf_throw()
 * takes, with what is allowed. So does the exit of the prog, see flow_compute_need_const().
 */
static u16 flow_tested_regs(struct bpf_verifier_env *env, int idx)
{
	struct bpf_insn_aux_data *aux = &env->insn_aux_data[idx];
	u16 consts = aux->const_reg_mask | aux->const_reg_wide_mask;
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code);

	if (bpf_helper_call(insn) || bpf_pseudo_kfunc_call(insn))
		return bpf_call_checked_arg_regs(env, insn);
	if ((class != BPF_JMP && class != BPF_JMP32) ||
	    op == BPF_JA || op == BPF_CALL || op == BPF_EXIT || op == BPF_JCOND)
		return 0;
	if (BPF_SRC(insn->code) == BPF_K || (consts & BIT(insn->src_reg)))
		return BIT(insn->dst_reg);
	return consts & BIT(insn->dst_reg) ? BIT(insn->src_reg) : 0;
}

/* A jump that compares a register with 0 or 1 */
u16 bpf_flow_flag_regs(struct bpf_verifier_env *env, int idx)
{
	struct bpf_insn *insn = &env->prog->insnsi[idx];

	if (BPF_SRC(insn->code) != BPF_K || !bpf_insn_is_cond_jump(insn->code) ||
	    BPF_OP(insn->code) == BPF_JCOND)
		return 0;
	if (BPF_OP(insn->code) == BPF_JSET ? !(insn->imm & 1) : (u32)insn->imm > 1)
		return 0;
	return BIT(insn->dst_reg);
}

/* With or without sign extension */
static bool flow_is_copy(const struct bpf_insn *insn)
{
	return BPF_OP(insn->code) == BPF_MOV && BPF_SRC(insn->code) == BPF_X && !insn->imm &&
	       (!insn->off || insn->off == 8 || insn->off == 16 || insn->off == 32);
}

static bool flow_is_shift(const struct bpf_insn *insn)
{
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code);

	return (class == BPF_ALU || class == BPF_ALU64) &&
	       (op == BPF_LSH || op == BPF_RSH || op == BPF_ARSH);
}

/*
 * The places of a function that hold a value: the registers, the arguments on the stack that it
 * got and those that it hands over, the slots of the stack that struct flow_need has a bit for.
 */
enum {
	FLOW_ARG_IN	= 16,
	FLOW_ARG_OUT	= FLOW_ARG_IN + MAX_STACK_ARG_SLOTS,
	FLOW_SLOT	= 64,
	FLOW_PLACES	= 128,
};
static_assert(FLOW_ARG_OUT + MAX_STACK_ARG_SLOTS <= FLOW_SLOT);

enum flow_kind {
	FLOW_IS_PLAIN,	/* a number or a pointer to arena: nothing checks where it points to */
	FLOW_IS_VALUE,	/* that, or a pointer to a map value that holds nothing but numbers */
	FLOW_IS_CTX,	/* the context, as the prog or a global function got it */
	FLOW_KINDS,
	FLOW_IS_OTHER = FLOW_KINDS,
};

/* What the places hold for sure in front of an insn */
struct flow_kinds {
	unsigned long is[FLOW_KINDS][BITS_TO_LONGS(FLOW_PLACES)];
};

static void flow_kind_set(struct flow_kinds *k, u32 place, enum flow_kind kind)
{
	int i;

	for (i = 0; i < FLOW_KINDS; i++)
		__assign_bit(place, k->is[i], i == kind);
	if (kind == FLOW_IS_PLAIN)
		__set_bit(place, k->is[FLOW_IS_VALUE]);
}

static void flow_kind_copy(struct flow_kinds *to, u32 place, const struct flow_kinds *from,
			   u32 from_place)
{
	int i;

	for (i = 0; i < FLOW_KINDS; i++)
		__assign_bit(place, to->is[i], test_bit(from_place, from->is[i]));
}

/* What holds on both ways to an insn. Returns true if that is less than before. */
static bool flow_kinds_meet(struct flow_kinds *to, const struct flow_kinds *from)
{
	unsigned long lost = 0;
	int i, j;

	for (i = 0; i < FLOW_KINDS; i++) {
		for (j = 0; j < ARRAY_SIZE(to->is[i]); j++) {
			lost |= to->is[i][j] & ~from->is[i][j];
			to->is[i][j] &= from->is[i][j];
		}
	}
	return lost;
}

/* The first insn of a function that is called from @k */
static bool flow_kinds_enter(struct flow_kinds *to, const struct flow_kinds *k)
{
	struct flow_kinds e = {};
	int i;

	/* r12 points to arena. The rest is not set. */
	bitmap_fill(e.is[FLOW_IS_PLAIN], FLOW_PLACES);
	bitmap_fill(e.is[FLOW_IS_VALUE], FLOW_PLACES);
	flow_kind_set(&e, BPF_REG_FP, FLOW_IS_OTHER);
	for (i = BPF_REG_1; i <= BPF_REG_5; i++)
		flow_kind_copy(&e, i, k, i);
	for (i = 0; i < MAX_STACK_ARG_SLOTS; i++)
		flow_kind_copy(&e, FLOW_ARG_IN + i, k, FLOW_ARG_OUT + i);
	return flow_kinds_meet(to, &e);
}

/* The place that @insn loads from or stores to through r10 or r11, -1 if it's something else */
static int flow_mem_place(const struct bpf_insn *insn, u8 base)
{
	int n = (abs(insn->off) - 1) / BPF_REG_SIZE;

	if (!insn->off)
		return -1;
	if (base == BPF_REG_FP && insn->off < 0 && n < 64)
		return FLOW_SLOT + n;
	if (base == BPF_REG_PARAMS && n < MAX_STACK_ARG_SLOTS)
		return (insn->off < 0 ? FLOW_ARG_OUT : FLOW_ARG_IN) + n;
	return -1;
}

static enum flow_kind flow_map_value_kind(const struct bpf_map *map)
{
	if (map->map_type == BPF_MAP_TYPE_ARENA)
		return FLOW_IS_PLAIN;
	/* what is at a known offset of a read-only map is known: a number, a string, a function */
	if (map->map_flags & BPF_F_RDONLY_PROG)
		return FLOW_IS_OTHER;
	return IS_ERR_OR_NULL(map->record) && !map->inner_map_meta ? FLOW_IS_VALUE : FLOW_IS_OTHER;
}

/* What @insn loads through the register @base */
static enum flow_kind flow_loaded_kind(struct bpf_verifier_env *env, const struct flow_kinds *k,
				       const struct bpf_insn *insn, u8 base)
{
	struct bpf_insn_access_aux info = { .reg_type = SCALAR_VALUE };
	int size = bpf_size_to_bytes(BPF_SIZE(insn->code));
	bool tstamp = env->prog->tstamp_type_access, valid;

	/* a number that is good for a load is an address in arena */
	if (test_bit(base, k->is[FLOW_IS_VALUE]))
		return FLOW_IS_PLAIN;
	if (size == BPF_REG_SIZE && !test_bit(base, k->is[FLOW_IS_CTX]))
		return FLOW_IS_OTHER;
	valid = env->ops->is_valid_access &&
		env->ops->is_valid_access(insn->off, size, BPF_READ, env->prog, &info);
	/* it notes what the prog reads, and this is a question */
	env->prog->tstamp_type_access = tstamp;
	if (valid && info.reg_type == PTR_TO_PACKET_END)
		bpf_find_containing_subprog(env, insn - env->prog->insnsi)->loads_pkt_end = true;
	/*
	 * Only the context has pointers that are shorter than a register. A base that nothing is
	 * known about can be the context: it came out of a struct, or only some paths have it.
	 */
	if (size < BPF_REG_SIZE)
		return valid && info.reg_type != SCALAR_VALUE ? FLOW_IS_OTHER : FLOW_IS_PLAIN;
	return valid && info.reg_type == SCALAR_VALUE ? FLOW_IS_PLAIN : FLOW_IS_OTHER;
}

/* What a helper or a kfunc returns */
static enum flow_kind flow_returned_kind(struct bpf_verifier_env *env, int idx)
{
	struct bpf_insn_aux_data *aux = &env->insn_aux_data[idx];
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	const struct bpf_func_proto *fn;
	struct bpf_call_arg_meta meta;
	const struct btf_type *t;

	if (bpf_pseudo_kfunc_call(insn)) {
		if (bpf_fetch_kfunc_arg_meta(env, insn->imm, insn->off, &meta) < 0)
			return FLOW_IS_OTHER;
		t = btf_type_skip_modifiers(meta.btf, meta.func_proto->type, NULL);
		if (!btf_type_is_ptr(t))
			return FLOW_IS_PLAIN;
		/* void * is a number, unless the verifier knows the kfunc: it may fail, or casts */
		t = btf_type_skip_modifiers(meta.btf, t->type, NULL);
		return btf_type_is_void(t) && !(meta.kfunc_flags & KF_RET_NULL) &&
		       !bpf_is_cast_kfunc(&meta) ? FLOW_IS_PLAIN : FLOW_IS_OTHER;
	}
	if (bpf_get_helper_proto(env, insn->imm, &fn) < 0)
		return FLOW_IS_OTHER;
	if (fn->ret_type == RET_INTEGER || fn->ret_type == RET_VOID)
		return FLOW_IS_PLAIN;
	if (base_type(fn->ret_type) == RET_PTR_TO_MAP_VALUE &&
	    (aux->const_reg_map_mask & BIT(BPF_REG_1)))
		return flow_map_value_kind(env->used_maps[aux->const_reg_vals[BPF_REG_1]]);
	return FLOW_IS_OTHER;
}

/*
 * @k is what holds in front of the insn at @idx. Make it what holds behind it.
 * @ret: the registers that are FLOW_IS_PLAIN at all exits of a subprog.
 * Returns true if less is known than before at the entry or about the exits of a subprog.
 */
static bool flow_kinds_step(struct bpf_verifier_env *env, struct flow_kinds *kinds, u16 *ret,
			    struct flow_kinds *k, int idx)
{
	struct bpf_subprog_info *sub = env->subprog_info;
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code);
	u8 dst = insn->dst_reg, src = insn->src_reg;
	bool wide = BPF_SIZE(insn->code) == BPF_DW && BPF_MODE(insn->code) == BPF_MEM &&
		    !(insn->off % BPF_REG_SIZE);
	bool lost = false;
	u16 regs = U16_MAX;
	int i, s;

	if (class == BPF_ALU64 && (op == BPF_ADD || op == BPF_SUB)) {
		__clear_bit(dst, k->is[FLOW_IS_CTX]);
		for (i = 0; BPF_SRC(insn->code) == BPF_X && i < FLOW_IS_CTX; i++)
			if (!test_bit(src, k->is[i]))
				__clear_bit(dst, k->is[i]);
	} else if (insn->code == (BPF_ALU64 | BPF_MOV | BPF_X) && !insn->off) {
		flow_kind_copy(k, dst, k, src);
	} else if (class == BPF_ALU || class == BPF_ALU64) {
		/*
		 * addr_space_cast too. It leaves a pointer that is not in arena as it is, and
		 * a compiler casts what it cannot know. Nearly all of that is in arena.
		 */
		flow_kind_set(k, dst, FLOW_IS_PLAIN);
	} else if (class == BPF_LDX) {
		s = flow_mem_place(insn, src);
		if (wide && s >= 0)
			flow_kind_copy(k, dst, k, s);
		else
			flow_kind_set(k, dst, flow_loaded_kind(env, k, insn, src));
	} else if (class == BPF_STX || class == BPF_ST) {
		i = bpf_atomic_load_reg(insn);
		if (i >= 0)
			flow_kind_set(k, i, flow_loaded_kind(env, k, insn,
						insn->imm == BPF_LOAD_ACQ ? src : dst));
		s = flow_mem_place(insn, dst);
		if (class == BPF_STX && wide && s >= 0)
			flow_kind_copy(k, s, k, src);
		else if (s >= 0)
			flow_kind_set(k, s, FLOW_IS_PLAIN);
	} else if (bpf_is_ldimm64(insn)) {
		if (src == BPF_PSEUDO_MAP_VALUE || src == BPF_PSEUDO_MAP_IDX_VALUE)
			flow_kind_set(k, dst, flow_map_value_kind(
					env->used_maps[env->insn_aux_data[idx].map_index]));
		else
			flow_kind_set(k, dst, src ? FLOW_IS_OTHER : FLOW_IS_PLAIN);
	} else if (insn->code == (BPF_JMP | BPF_EXIT) && !bpf_is_unwind_exit(insn)) {
		s = bpf_find_containing_subprog(env, idx) - sub;
		lost = ret[s] & ~k->is[FLOW_IS_PLAIN][0];
		ret[s] &= k->is[FLOW_IS_PLAIN][0];
	} else if (class == BPF_LD || (class == BPF_JMP && op == BPF_CALL)) {
		s = bpf_pseudo_call(insn) ? bpf_find_subprog(env, idx + insn->imm + 1) : -1;
		if (s >= 0 && !bpf_subprog_is_global(env, s)) {
			lost = flow_kinds_enter(&kinds[sub[s].start], k);
			regs = ret[s];
		} else if (bpf_is_callx(insn)) {
			regs = 0;
		}
		/* a subprog may return a pair in R0 and R2, the rest is not set */
		regs |= ~(BIT(BPF_REG_0) | BIT(BPF_REG_2));
		for (i = BPF_REG_0; i <= BPF_REG_5; i++)
			flow_kind_set(k, i, regs & BIT(i) ? FLOW_IS_PLAIN : FLOW_IS_OTHER);
		for (i = 0; i < MAX_STACK_ARG_SLOTS; i++)
			flow_kind_set(k, FLOW_ARG_OUT + i, FLOW_IS_PLAIN);
		if (bpf_helper_call(insn) || bpf_pseudo_kfunc_call(insn))
			flow_kind_set(k, BPF_REG_0, flow_returned_kind(env, idx));
	}
	return lost;
}

/*
 * Find the registers that hold a number or a pointer to arena, and those that point to the value
 * of a map that holds nothing but numbers at worst, whichever way the prog gets to an insn:
 * plain_regs and value_regs of struct bpf_insn_aux_data. The others may hold any pointer.
 *
 * Forwards, from "all is known" down, until nothing changes. Arguments go from the callers to
 * the callees and results the other way, so the subprogs are looked at in both orders in turn.
 *
 * Nothing depends on it but which states are kept apart. A pointer that is taken for
 * a number costs a rule. A number that is taken for a pointer costs states. So it's fine to
 * miss a store through a pointer to the stack, and to take what callx calls for unknown.
 *
 * The edges that the verifier adds, see bpf_head_guard_target(): may_goto at the head of a loop
 * takes what holds there to its target. A subprog that is out of budget goes to an exit with 0
 * in place of the result, a number.
 */
int bpf_flow_compute_kinds(struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *sub = env->subprog_info;
	int len = env->prog->len, cnt = env->subprog_cnt, i, j, n, s, t, idx, err = 0;
	struct flow_kinds *kinds, k = {};
	struct bpf_iarray *succ;
	bool lost = true;
	u16 *ret;

	kinds = kvmalloc_array(len, sizeof(*kinds), GFP_KERNEL_ACCOUNT);
	ret = kvmalloc_array(env->subprog_cnt, sizeof(*ret), GFP_KERNEL_ACCOUNT);
	if (!kinds || !ret) {
		kvfree(kinds);
		kvfree(ret);
		return -ENOMEM;
	}
	memset(kinds, 0xff, len * sizeof(*kinds));
	memset(ret, 0xff, env->subprog_cnt * sizeof(*ret));

	/* what the kernel, a helper, callx or another prog calls: only the context is known */
	for (s = 0; s < env->subprog_cnt; s++) {
		if (s && !bpf_subprog_is_global(env, s) && !sub[s].addr_taken)
			continue;
		flow_kinds_enter(&kinds[sub[s].start], &k);
		for (i = 0; i < MAX_BPF_FUNC_REG_ARGS; i++)
			if (sub[s].args[i].arg_type == ARG_PTR_TO_CTX ||
			    (!s && !i && env->prog->type != BPF_PROG_TYPE_EXT))
				flow_kind_set(&kinds[sub[s].start], BPF_REG_1 + i, FLOW_IS_CTX);
	}

	for (n = 0; lost; n++) {
		lost = false;
		/* a chain of calls takes a round for each link */
		if (signal_pending(current)) {
			err = -EAGAIN;
			break;
		}
		cond_resched();
		for (j = 0; j < cnt; j++) {
			/* it has callees first */
			s = env->subprog_topo_order[n & 1 ? j : cnt - 1 - j];
			for (i = sub[s + 1].postorder_start; i-- > sub[s].postorder_start;) {
				idx = env->cfg.insn_postorder[i];
				k = kinds[idx];
				lost |= flow_kinds_step(env, kinds, ret, &k, idx);
				succ = bpf_insn_successors(env, idx);
				iarray_for_each(t, succ)
					lost |= flow_kinds_meet(&kinds[t], &k);
				t = bpf_head_guard_target(env, idx);
				if (t >= 0)
					lost |= flow_kinds_meet(&kinds[t], &kinds[idx]);
			}
		}
	}
	for (i = 0; i < len; i++) {
		env->insn_aux_data[i].plain_regs = kinds[i].is[FLOW_IS_PLAIN][0];
		env->insn_aux_data[i].value_regs = kinds[i].is[FLOW_IS_VALUE][0];
	}
	kvfree(kinds);
	kvfree(ret);
	return err;
}

/*
 * A number is added to what may be a pointer whose offset the verifier looks at:
 * - An object of the kernel, the context, a socket take a known offset only.
 * - The packet: the offset was compared with the end of the packet before the states met.
 * - The value of a map that has a timer, a lock, a pointer in the middle.
 * - The value of a read-only map: what is loaded from a known offset is known.
 * Not a value that holds nothing but numbers. Only the lowest and the highest offset
 * count there, and the join of two constants has both. @any_value: that one too.
 */
static u16 flow_ptr_off_regs(struct bpf_verifier_env *env, int idx, bool any_value)
{
	struct bpf_insn_aux_data *aux = &env->insn_aux_data[idx];
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	u16 both = BIT(insn->dst_reg) | BIT(insn->src_reg), ptrs;
	u8 op = BPF_OP(insn->code);

	if (BPF_CLASS(insn->code) != BPF_ALU64 || BPF_SRC(insn->code) != BPF_X ||
	    (op != BPF_ADD && op != BPF_SUB) || (aux->stack_ptrs & both))
		return 0;
	ptrs = both & ~(any_value ? aux->plain_regs : aux->value_regs);
	if (op == BPF_SUB)
		ptrs &= BIT(insn->dst_reg);
	/* the one that may be the pointer makes the other one the number */
	return ptrs && ptrs != both ? both ^ ptrs : ptrs;
}

/* The base of a load or a store, unless it's a number or a pointer to arena for sure */
static u16 flow_base_regs(struct bpf_verifier_env *env, int idx)
{
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	u8 class = BPF_CLASS(insn->code);
	u16 regs = 0;

	if (class == BPF_LDX || bpf_atomic_is_load_acq(insn))
		regs = BIT(insn->src_reg);
	else if (class == BPF_ST || class == BPF_STX)
		regs = BIT(insn->dst_reg);
	return regs & ~env->insn_aux_data[idx].plain_regs & GENMASK(BPF_REG_9, BPF_REG_0);
}

/*
 * Some arguments of kfuncs and helpers have to be known constants: the id of a type,
 * the size of what is reserved in a ring buffer, the size of a slice of a dynptr.
 * A compiler makes one call of
 *	if (x) p = bpf_rdonly_cast(q, ID); else p = bpf_rdonly_cast(q, 0);
 * and one store of
 *	if (x) a[n++] = p; else b[m++] = p;
 * where a and b are on the stack.
 * and one load of
 *	x = ipv6 ? *(u16 *)(pkt + 54) : *(u16 *)(pkt + 34);	y = z ? task->pid : task->tgid;
 * The join of two constants is not a constant. Find the registers and the slots of the stack
 * that end up in such arguments, or in offsets in the stack or from a pointer,
 * like live registers are found: backwards, until nothing changes.
 * States in which they differ are not joined.
 * Nothing depends on it but what is known: it's fine to miss a store through a pointer.
 *
 * FLOW_NEED_JUMP: those that a jump ahead compares with a constant, see flow_join_makes_up().
 * As they are, or extended: without 32-bit insns a compiler tests
 *	r7 = r8; r7 <<= 32; r7 >>= 32;
 * and goes on with r8.
 * A slot that is filled through another pointer to the stack than r10 is not one of them, and
 * flow_null_differs() counts on that.
 *
 * FLOW_NEED_FLAG: those of which a jump tells 0 from 1, see flow_join_makes_up(). As they are,
 * extended, or masked: a bool is tested as "w1 &= 1; if w1 != 0". Not through a call.
 * The jump ends the basic block of the insn, or a block that this block leads to. Not further:
 * every state that is kept apart goes through all that is on the way to the jump once more.
 * So there are two rounds. In the first one nothing comes from the blocks ahead. What it finds
 * at the first insn of a block is what the second one takes at the end of the blocks that lead
 * there. In both a block depends on nothing that the round itself computes for another block,
 * so the order in which the insns are looked at doesn't matter.
 *
 * The last three kinds keep no states apart. They are for bpf_flow_is_needed(), which asks
 * whether anything ahead looks at a number that a loop changes from iteration to iteration.
 * FLOW_NEED_VALUE_OFF: FLOW_NEED_PTR_OFF, and the offset in a map value that holds nothing but
 * numbers. It has to be in the value. It can be an argument, or what a function returns:
 *	return v->hist[count_bits(x)];
 * So can an offset in the stack. FLOW_NEED_STACK_OFF doesn't follow it through a return, so this
 * table has it too.
 * FLOW_NEED_BASE: the base of a load or a store, as it is, behind a cast, moved by a constant:
 *	p = NULL; for (i = 0; i < 3; i++) { if (p) n += p->len; p = next(); }
 * FLOW_NEED_JUMP_ALU: FLOW_NEED_JUMP, and what the tested number is made of step by step:
 *	for (i = 0; i < 3; i++) x /= 10; if (x < 8) ...
 * In x op= y that is x. y is not followed.
 * What is stored through a pointer to the stack other than r10 counts as tested. It's a result
 * for the caller, and no table follows it from the store to the fill:
 *	count_bits(x, &n); return v->hist[n];
 * These two go through callx, to every function whose address is taken.
 */
static int flow_compute_need_const(struct bpf_verifier_env *env, struct flow_need **pneed,
				   enum flow_need_kind kind)
{
	struct bpf_insn *insns = env->prog->insnsi, *insn;
	struct flow_need in, out, *need, *from, *ahead = NULL;
	struct bpf_iarray *succ;
	bool jump = kind == FLOW_NEED_JUMP || kind == FLOW_NEED_JUMP_ALU;
	bool callx = kind == FLOW_NEED_BASE || kind == FLOW_NEED_JUMP_ALU;
	bool flag = kind == FLOW_NEED_FLAG, second = false;
	bool changed = true, any = false, keep, from_src, reg;
	int i, s, idx, slot, err = 0;
	u16 *gen, callx_regs = 0, pair = BIT(BPF_REG_0) | BIT(BPF_REG_2);
	u8 *ret, class, op;

	need = kvcalloc(env->prog->len, sizeof(*need), GFP_KERNEL_ACCOUNT);
	gen = kvcalloc(env->prog->len, sizeof(u16), GFP_KERNEL_ACCOUNT);
	/* the callers need what the subprog returns: r0, and r2 of a pair */
	ret = kvcalloc(env->subprog_cnt, sizeof(u8), GFP_KERNEL_ACCOUNT);
	if (flag)
		ahead = kvcalloc(env->prog->len, sizeof(*need), GFP_KERNEL_ACCOUNT);
	*pneed = need;
	if (!need || !gen || !ret || (flag && !ahead)) {
		kvfree(ahead);
		kvfree(gen);
		kvfree(ret);
		return -ENOMEM;
	}
	for (i = 0; i < env->prog->len; i++) {
		if (jump)
			gen[i] = flow_tested_regs(env, i);
		else if (flag)
			gen[i] = bpf_flow_flag_regs(env, i);
		else if (kind == FLOW_NEED_STACK_OFF)
			gen[i] = flow_stack_off_regs(env, i);
		else if (kind == FLOW_NEED_PTR_OFF || kind == FLOW_NEED_VALUE_OFF)
			gen[i] = flow_ptr_off_regs(env, i, kind == FLOW_NEED_VALUE_OFF);
		else if (kind == FLOW_NEED_BASE)
			gen[i] = flow_base_regs(env, i);
		else if (bpf_helper_call(&insns[i]) || bpf_pseudo_kfunc_call(&insns[i]))
			gen[i] = bpf_call_const_arg_regs(env, &insns[i]);
		if (kind == FLOW_NEED_JUMP_ALU && BPF_CLASS(insns[i].code) == BPF_STX &&
		    insns[i].dst_reg != BPF_REG_FP &&
		    (env->insn_aux_data[i].stack_ptrs & BIT(insns[i].dst_reg)))
			gen[i] = BIT(insns[i].src_reg);
		if (kind == FLOW_NEED_VALUE_OFF)
			gen[i] |= flow_stack_off_regs(env, i);
		any |= gen[i];
	}
	if (jump) {
		/* the verifier compares what the prog and its callbacks return with a range */
		ret[0] = BIT(BPF_REG_0);
		for (i = 0; i < env->prog->len; i++) {
			if (!bpf_pseudo_func(&insns[i]))
				continue;
			s = bpf_find_subprog(env, i + insns[i].imm + 1);
			if (s >= 0)
				ret[s] = BIT(BPF_REG_0);
		}
		any = true;
	}
again:
	while (any && changed) {
		changed = false;
		if (signal_pending(current)) {
			err = -EAGAIN;
			break;
		}
		cond_resched();
		for (s = 0; callx && s < env->subprog_cnt; s++)
			if (env->subprog_info[s].addr_taken)
				callx_regs |= need[env->subprog_info[s].start].regs;
		for (i = 0; i < env->cfg.cur_postorder; i++) {
			idx = env->cfg.insn_postorder[i];
			insn = &insns[idx];
			class = BPF_CLASS(insn->code);
			succ = bpf_insn_successors(env, idx);
			out.regs = 0;
			out.slots = 0;
			from = flag && env->insn_aux_data[idx].bb_end ? ahead : need;
			for (s = 0; s < succ->cnt; s++) {
				out.regs |= from[succ->items[s]].regs;
				out.slots |= from[succ->items[s]].slots;
			}
			in = out;
			if (class == BPF_JMP && BPF_OP(insn->code) == BPF_CALL) {
				in.regs &= ~GENMASK(BPF_REG_5, BPF_REG_0);
				if (bpf_is_callx(insn))
					in.regs |= callx_regs & GENMASK(BPF_REG_5, BPF_REG_1);
				s = idx + insn->imm + 1;
				if (bpf_pseudo_call(insn) && !flag &&
				    !bpf_subprog_is_global(env, bpf_find_subprog(env, s))) {
					if (kind != FLOW_NEED_PTR_OFF)
						in.regs |= need[s].regs &
							   GENMASK(BPF_REG_5, BPF_REG_1);
					s = bpf_find_subprog(env, s);
					/*
					 * Not for an index where states are kept apart: that is
					 * one of many numbers.
					 */
					if ((kind == FLOW_NEED_ARG || kind == FLOW_NEED_VALUE_OFF ||
					     jump) &&
					    (out.regs & pair & ~ret[s])) {
						ret[s] |= out.regs & pair;
						changed = true;
					}
				}
			} else if (insn->code == (BPF_JMP | BPF_EXIT)) {
				s = bpf_find_containing_subprog(env, idx) - env->subprog_info;
				in.regs |= ret[s];
			} else if (class == BPF_LDX || class == BPF_LD) {
				slot = class == BPF_LDX ? flow_fp_slot(insn, insn->src_reg) : -1;
				if (slot >= 0 && (out.regs & BIT(insn->dst_reg)))
					in.slots |= BIT_ULL(slot);
				in.regs &= ~BIT(insn->dst_reg);
			} else if (class == BPF_STX || class == BPF_ST) {
				slot = flow_fp_slot(insn, insn->dst_reg);
				if (slot >= 0 && (out.slots & BIT_ULL(slot))) {
					in.slots &= ~BIT_ULL(slot);
					if (class == BPF_STX)
						in.regs |= BIT(insn->src_reg);
				}
			} else if ((class == BPF_ALU || class == BPF_ALU64) &&
				   (out.regs & BIT(insn->dst_reg))) {
				op = BPF_OP(insn->code);
				reg = BPF_SRC(insn->code) == BPF_X;
				/* the result is known when what it is made of is */
				keep = op != BPF_MOV;
				from_src = reg;
				if (jump || flag)
					from_src = flow_is_copy(insn);
				if (kind == FLOW_NEED_JUMP || flag)
					keep = flow_is_shift(insn) && !reg;
				if (flag && op == BPF_AND && !reg)
					keep = insn->imm & 1;
				if (kind == FLOW_NEED_BASE) {
					keep = !reg && (op == BPF_ADD || op == BPF_SUB);
					from_src = reg && op == BPF_MOV;
				}
				if (!keep)
					in.regs &= ~BIT(insn->dst_reg);
				if (from_src)
					in.regs |= BIT(insn->src_reg);
			}
			in.regs |= gen[idx];
			if (in.regs != need[idx].regs || in.slots != need[idx].slots) {
				need[idx] = in;
				changed = true;
			}
		}
	}
	if (flag && !err && !second) {
		memcpy(ahead, need, env->prog->len * sizeof(*need));
		second = changed = true;
		goto again;
	}
	/*
	 * An offset that is an argument: not through the call, see above, and not where the calls
	 * meet. A function would be looked at once for each offset that it's called with.
	 */
	for (s = 0; kind == FLOW_NEED_PTR_OFF && s < env->subprog_cnt; s++)
		need[env->subprog_info[s].start].regs = 0;
	kvfree(ahead);
	kvfree(gen);
	kvfree(ret);
	if (!any) {
		kvfree(need);
		*pneed = NULL;
	}
	return err;
}

/*
 * Does anything ahead of the insn at @idx look at what @r holds there? @r is a register or a stack
 * slot of the function as scev.c counts them. See flow_compute_need_const(): it's a guess.
 */
bool bpf_flow_is_needed(struct bpf_verifier_env *env, int idx, u32 r)
{
	const struct flow_need *need;
	int i;

	/* no table has a bit for it */
	if (r >= MAX_BPF_REG + 64)
		return true;
	for (i = 0; i < FLOW_NEED_KINDS; i++) {
		need = env->flow->need[i];
		if (need && (r < MAX_BPF_REG ? need[idx].regs & BIT(r) :
					       need[idx].slots & BIT_ULL(r - MAX_BPF_REG)))
			return true;
	}
	return false;
}

/*
 * A number that grows from one iteration of a loop to the next is not followed step by step. The
 * bound that moved is put where it may come to rest, see bpf_reg_widen_range(). The jumps of
 * the loop tell where that is:
 *	do { a[i] = 0; i += 3; } while (i < 30);		29 at most comes back to the head
 *	while (i < 30) { a[i] = 0; i += 3; }			32
 *	loop { if (...) { if (n >= 64) break; a[n++] = x; } }	64
 *
 * One iteration is looked at, from the head to the jumps back. What a register or a stack slot
 * holds is written as root + constant. A root is what a place held at the head, or what was made
 * in a place since. A copy, an extension, a spill and a fill keep the root. += and -= of a constant
 * change the constant. Everything else makes a new root. Where paths meet two constants become
 * a range of them, and two roots a new one.
 * A jump that compares root + c with a constant bounds the root on each of its edges that stays in
 * the loop. What comes back to the head in a place is root + c again: that, with the bounds of
 * the root, is where the place may come to rest.
 *
 * It's a guess. The state that comes back to the head is compared with it like with any other
 * state. So it's fine to take a 32-bit add for an add, to miss a store through a pointer, to mix up
 * two numbers that are made in one place, and to know nothing of the edges that the verifier adds.
 */
struct flow_sym {
	s32 lo, hi;
	u8 root;
};

#define FLOW_NO_PLACE	U8_MAX
/* How many of them the look at one loop may take. A loop that needs more has no table. */
#define BPF_FLOW_MAX_SYMS	(4 << 20)

static_assert(FLOW_SLOT == BPF_LOOP_SLOT && FLOW_PLACES == BPF_LOOP_PLACES);

/* @insn makes the place *@to a copy of the place *@from, extended or cut to its low bits */
static bool flow_copies_place(const struct bpf_insn *insn, u8 *to, u8 *from)
{
	u8 class = BPF_CLASS(insn->code);
	int slot;

	if (class == BPF_ALU || class == BPF_ALU64) {
		*to = insn->dst_reg;
		*from = insn->src_reg;
		return flow_is_copy(insn) && *to < BPF_REG_FP && *from < BPF_REG_FP;
	}
	if (class != BPF_LDX && class != BPF_STX)
		return false;
	slot = flow_fp_slot(insn, class == BPF_LDX ? insn->src_reg : insn->dst_reg);
	if (slot < 0)
		return false;
	*to = class == BPF_LDX ? insn->dst_reg : FLOW_SLOT + slot;
	*from = class == BPF_LDX ? FLOW_SLOT + slot : insn->src_reg;
	return *to != BPF_REG_FP && *from != BPF_REG_FP;
}

/* An ALU insn that leaves a small number as it is: an extension, or half of one */
static bool flow_extends(const struct bpf_insn *insn)
{
	if (BPF_SRC(insn->code) != BPF_K)
		return false;
	if (flow_is_shift(insn))
		return insn->imm == 32 || insn->imm == 48 || insn->imm == 56;
	return BPF_OP(insn->code) == BPF_AND &&
	       (insn->imm == 0xff || insn->imm == 0xffff || insn->imm == -1);
}

static void flow_written_places(const struct bpf_insn *insn, unsigned long *places)
{
	u8 class = BPF_CLASS(insn->code);
	int p;

	bitmap_zero(places, FLOW_PLACES);
	if (class == BPF_ALU || class == BPF_ALU64 || class == BPF_LDX || bpf_is_ldimm64(insn)) {
		__set_bit(insn->dst_reg, places);
	} else if (class == BPF_LD || (class == BPF_JMP && BPF_OP(insn->code) == BPF_CALL)) {
		bitmap_set(places, BPF_REG_0, BPF_REG_5 + 1);
	} else if (class == BPF_ST || class == BPF_STX) {
		p = bpf_atomic_load_reg(insn);
		if (p >= 0)
			__set_bit(p, places);
		p = flow_mem_place(insn, insn->dst_reg);
		if (p >= FLOW_SLOT)
			__set_bit(p, places);
	}
}

/*
 * The jump at @idx compares the register *@reg with a constant: it's taken if "reg op k".
 * The constant has 32 bits, so does what is added to it below: nothing wraps.
 */
static bool flow_jump_cmp(struct bpf_verifier_env *env, int idx, u8 *reg, u8 *op, s64 *k)
{
	const struct bpf_insn_aux_data *aux = &env->insn_aux_data[idx];
	const struct bpf_insn *insn = &env->prog->insnsi[idx];
	u8 class = BPF_CLASS(insn->code);

	*op = BPF_OP(insn->code);
	if ((class != BPF_JMP && class != BPF_JMP32) || *op == BPF_JA || *op == BPF_CALL ||
	    *op == BPF_EXIT || *op == BPF_JCOND || *op == BPF_JSET)
		return false;
	*reg = insn->dst_reg;
	if (BPF_SRC(insn->code) == BPF_K) {
		*k = insn->imm;
	} else if (aux->const_reg_mask & BIT(insn->src_reg)) {
		*k = aux->const_reg_vals[insn->src_reg];
	} else if (aux->const_reg_mask & BIT(insn->dst_reg)) {
		*k = aux->const_reg_vals[insn->dst_reg];
		*reg = insn->src_reg;
		*op = bpf_flip_opcode(*op);
	} else {
		return false;
	}
	if (class == BPF_JMP32)
		*k = *op == BPF_JSGT || *op == BPF_JSGE || *op == BPF_JSLT || *op == BPF_JSLE ?
		     (s64)(s32)*k : (s64)(u32)*k;
	return *reg < BPF_REG_FP;
}

static void flow_add_max(struct bpf_loop_rungs *r, s64 max)
{
	r->max[0] = r->has_max ? min(r->max[0], max) : max;
	r->max[1] = r->has_max ? max(r->max[1], max) : max;
	r->has_max = true;
}

static void flow_add_min(struct bpf_loop_rungs *r, s64 min)
{
	r->min[0] = r->has_min ? max(r->min[0], min) : min;
	r->min[1] = r->has_min ? min(r->min[1], min) : min;
	r->has_min = true;
}

/* "x op k" holds. What does that say about the root of @x? */
static void flow_note_cmp(struct bpf_loop_rungs *root, const struct flow_sym *x, u8 op, s64 k)
{
	bool has_max = true, has_min = true;
	s64 max = k, min = k;

	switch (op) {
	case BPF_JEQ:
		break;
	case BPF_JNE:
		/* it cuts off the end of a range */
		max = k - 1;
		min = k + 1;
		break;
	case BPF_JLT:
		max = k - 1;
		fallthrough;
	case BPF_JLE:
		/* the ranges are signed: nothing is said by a constant above S64_MAX */
		min = 0;
		has_max = max >= 0;
		has_min = max >= 0;
		break;
	case BPF_JGT:
		min = k + 1;
		fallthrough;
	case BPF_JGE:
		has_max = false;
		has_min = k >= 0;
		break;
	case BPF_JSLT:
		max = k - 1;
		fallthrough;
	case BPF_JSLE:
		has_min = false;
		break;
	case BPF_JSGT:
		min = k + 1;
		fallthrough;
	case BPF_JSGE:
		has_max = false;
		break;
	default:
		return;
	}
	if (has_max)
		flow_add_max(root, max - x->lo);
	if (has_min)
		flow_add_min(root, min - x->hi);
}

/* @cur is what the @n places hold in front of @insn. Make it what they hold behind it. */
static void flow_sym_step(const struct bpf_insn *insn, struct flow_sym *cur, const u8 *idx, u32 n)
{
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code), to, from;
	DECLARE_BITMAP(written, FLOW_PLACES);
	struct flow_sym *x;
	s64 c = insn->imm;
	int p;

	/* both are among the places or none is, see flow_compute_loop_rungs() */
	if (flow_copies_place(insn, &to, &from)) {
		if (idx[to] != FLOW_NO_PLACE)
			cur[idx[to]] = cur[idx[from]];
		return;
	}
	if ((class == BPF_ALU || class == BPF_ALU64) && idx[insn->dst_reg] != FLOW_NO_PLACE) {
		x = &cur[idx[insn->dst_reg]];
		if (flow_extends(insn))
			return;
		if (op == BPF_SUB)
			c = -c;
		if ((op == BPF_ADD || op == BPF_SUB) && BPF_SRC(insn->code) == BPF_K &&
		    x->lo + c >= S32_MIN && x->hi + c <= S32_MAX) {
			x->lo += c;
			x->hi += c;
			return;
		}
	}
	flow_written_places(insn, written);
	for_each_set_bit(p, written, FLOW_PLACES)
		if (idx[p] != FLOW_NO_PLACE)
			cur[idx[p]] = (struct flow_sym){ .root = n + idx[p] };
}

/* One more path leads to where @to holds */
static void flow_sym_meet(struct flow_sym *to, const struct flow_sym *from, u32 n)
{
	u32 j;

	if (to[0].root == FLOW_NO_PLACE) {
		memcpy(to, from, n * sizeof(*to));
		return;
	}
	for (j = 0; j < n; j++) {
		if (to[j].root != from[j].root) {
			to[j] = (struct flow_sym){ .root = n + j };
			continue;
		}
		to[j].lo = min(to[j].lo, from[j].lo);
		to[j].hi = max(to[j].hi, from[j].hi);
	}
}

static u8 flow_place_set(u8 *set, u8 p)
{
	while (set[p] != p) {
		set[p] = set[set[p]];
		p = set[p];
	}
	return p;
}

/* @at: the insns in the order. The insns of a loop are next to each other, the head first. */
static int flow_compute_loop_rungs(struct bpf_verifier_env *env, const u32 *at, u32 total, int h)
{
	const struct bpf_insn *insns = env->prog->insnsi, *insn;
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_loop_rungs *roots = NULL, *rungs, *r;
	u8 set[FLOW_PLACES], idx[FLOW_PLACES], to, from, reg, op;
	u32 first = env->flow->pos[h], nr, n = 0, back_cnt = 0, cnt = 0, j, k;
	DECLARE_BITMAP(tested, FLOW_PLACES) = {};
	DECLARE_BITMAP(moves, FLOW_PLACES) = {};
	struct flow_sym *syms, *cur, *back, *x;
	struct bpf_loop *loop = aux[h].loop;
	const u32 *pos = env->flow->pos;
	struct bpf_iarray *succ;
	int i, p, t, err = -ENOMEM;
	bool carry = false;
	s64 c;

	nr = min(loop->nr_insns, total - first);

	/*
	 * The places to look at: what a jump compares with a constant, and what that is copied
	 * from and to. One of them has to change in the loop.
	 */
	for (p = 0; p < FLOW_PLACES; p++)
		set[p] = p;
	for (k = 0; k < nr; k++) {
		i = at[first + k];
		if (flow_copies_place(&insns[i], &to, &from))
			set[flow_place_set(set, to)] = flow_place_set(set, from);
		else if (flow_jump_cmp(env, i, &reg, &op, &c))
			__set_bit(reg, tested);
	}
	for_each_set_bit(p, tested, FLOW_PLACES)
		__set_bit(flow_place_set(set, p), moves);
	bitmap_copy(tested, moves, FLOW_PLACES);
	bitmap_zero(moves, FLOW_PLACES);
	for_each_set_bit(p, loop->writes, FLOW_PLACES)
		__set_bit(flow_place_set(set, p), moves);
	for (p = 0; p < FLOW_PLACES; p++) {
		i = flow_place_set(set, p);
		idx[p] = test_bit(i, tested) && test_bit(i, moves) ? n++ : FLOW_NO_PLACE;
	}
	if (!n || (u64)n * nr > BPF_FLOW_MAX_SYMS)
		return 0;

	/* for each insn, for the current insn, for each jump back */
	syms = kvmalloc_array((size_t)nr + 1 + loop->backedges_cnt, n * sizeof(*syms),
			      GFP_KERNEL_ACCOUNT);
	/* for each root, and the result for each place */
	roots = kvcalloc(3 * n, sizeof(*roots), GFP_KERNEL_ACCOUNT);
	if (!syms || !roots)
		goto out;
	memset(syms, 0xff, (size_t)nr * n * sizeof(*syms));
	cur = syms + (size_t)nr * n;
	back = cur + n;
	rungs = roots + 2 * n;
	for (j = 0; j < n; j++)
		syms[j] = (struct flow_sym){ .root = j };

	for (k = 0; k < nr; k++) {
		i = at[first + k];
		insn = &insns[i];
		if (!carry)
			memcpy(cur, syms + (size_t)k * n, n * sizeof(*cur));
		/* nothing but a jump back leads here */
		for (j = 0; !carry && j < n; j++)
			if (cur[j].root == FLOW_NO_PLACE)
				cur[j] = (struct flow_sym){ .root = n + j };
		/* a loop in the loop: nobody knows how many times it does what it does */
		if (k && aux[i].loop)
			for_each_set_bit(p, aux[i].loop->writes, FLOW_PLACES)
				if (idx[p] != FLOW_NO_PLACE)
					cur[idx[p]] = (struct flow_sym){ .root = n + idx[p] };

		if (flow_jump_cmp(env, i, &reg, &op, &c) && idx[reg] != FLOW_NO_PLACE) {
			x = &cur[idx[reg]];
			if (pos[i + insn->off + 1] - first < nr)
				flow_note_cmp(&roots[x->root], x, op, c);
			if (pos[i + 1] - first < nr)
				flow_note_cmp(&roots[x->root], x, bpf_rev_opcode(op), c);
		}
		flow_sym_step(insn, cur, idx, n);

		succ = bpf_insn_successors(env, i);
		carry = succ->cnt == 1 && !test_bit(succ->items[0], env->flow->point) &&
			pos[succ->items[0]] == first + k + 1;
		if (carry)
			continue;
		iarray_for_each(t, succ) {
			if (t == h && back_cnt < loop->backedges_cnt)
				memcpy(back + (size_t)back_cnt++ * n, cur, n * sizeof(*cur));
			else if (pos[t] > pos[i] && pos[t] - first < nr)
				flow_sym_meet(syms + (size_t)(pos[t] - first) * n, cur, n);
			if (t == h || pos[t] > pos[i] || pos[t] - first >= nr)
				continue;
			/* jump back of an inner loop: what it adds moves the bounds of the root */
			for (j = 0; j < n; j++) {
				r = &roots[cur[j].root];
				if (r->has_max && cur[j].hi > 0)
					flow_add_max(r, r->max[1] + cur[j].hi);
				if (r->has_min && cur[j].lo < 0)
					flow_add_min(r, r->min[1] + cur[j].lo);
			}
		}
	}

	for (k = 0; k < back_cnt; k++) {
		for (j = 0; j < n; j++) {
			x = &back[(size_t)k * n + j];
			r = &roots[x->root];
			/* it is what it was */
			if (x->root == j && !x->lo && !x->hi)
				continue;
			if (r->has_max) {
				flow_add_max(&rungs[j], r->max[0] + x->hi);
				flow_add_max(&rungs[j], r->max[1] + x->hi);
			}
			if (r->has_min) {
				flow_add_min(&rungs[j], r->min[0] + x->lo);
				flow_add_min(&rungs[j], r->min[1] + x->lo);
			}
		}
	}
	for (j = 0; j < n; j++)
		cnt += rungs[j].has_max || rungs[j].has_min;
	err = 0;
	if (!cnt)
		goto out;
	loop->rungs = kvcalloc(cnt, sizeof(*loop->rungs), GFP_KERNEL_ACCOUNT);
	if (!loop->rungs) {
		err = -ENOMEM;
		goto out;
	}
	for (p = 0; p < FLOW_PLACES; p++) {
		if (idx[p] == FLOW_NO_PLACE || (!rungs[idx[p]].has_max && !rungs[idx[p]].has_min))
			continue;
		loop->rungs[loop->rungs_cnt] = rungs[idx[p]];
		loop->rungs[loop->rungs_cnt++].place = p;
	}
out:
	kvfree(syms);
	kvfree(roots);
	return err;
}

static int flow_compute_rungs(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	u32 total = env->cfg.cur_postorder, depth, k, *at;
	DECLARE_BITMAP(written, FLOW_PLACES);
	struct bpf_loop *loop;
	int h, i, err = 0;

	for (h = 0; h < env->prog->len && !aux[h].loop; h++)
		;
	if (h == env->prog->len)
		return 0;
	at = kvmalloc_array(total, sizeof(*at), GFP_KERNEL_ACCOUNT);
	if (!at)
		return -ENOMEM;
	for (k = 0; k < total; k++) {
		i = env->cfg.insn_postorder[k];
		at[env->flow->pos[i]] = i;
		flow_written_places(&env->prog->insnsi[i], written);
		for (h = bpf_loop_at_index(env, i); h >= 0; h = aux[h].loop_header)
			bitmap_or(aux[h].loop->writes, aux[h].loop->writes, written, FLOW_PLACES);
	}
	for (h = 0; h < env->prog->len && !err; h++) {
		loop = aux[h].loop;
		/* the order says little about a loop with more than one entry */
		if (!loop || loop->irreducible || loop->backedges_overflow)
			continue;
		/* the main pass doesn't get that deep */
		depth = 0;
		for (i = aux[h].loop_header; i >= 0; i = aux[i].loop_header)
			depth++;
		if (depth >= LOOP_STACK_SIZE)
			continue;
		if (signal_pending(current)) {
			err = -EAGAIN;
			break;
		}
		cond_resched();
		err = flow_compute_loop_rungs(env, at, total, h);
	}
	kvfree(at);
	return err;
}

int bpf_flow_prepare(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_insn *insns = env->prog->insnsi;
	/* the rest is for loops */
	int len = env->prog->len, kinds = FLOW_NEED_VALUE_OFF, i, s, err;
	struct bpf_iarray *succ;
	struct bpf_flow *flow;
	u8 *preds;

	flow = kzalloc_obj(*flow, GFP_KERNEL_ACCOUNT);
	if (!flow)
		return -ENOMEM;
	env->flow = flow;
	INIT_LIST_HEAD(&flow->kept);
	flow->pending = RB_ROOT_CACHED;

	flow->pos = kvcalloc(len, sizeof(u32), GFP_KERNEL_ACCOUNT);
	flow->rank = kvcalloc(env->subprog_cnt + 1, sizeof(u32), GFP_KERNEL_ACCOUNT);
	flow->point = kvcalloc(BITS_TO_LONGS(len), sizeof(long), GFP_KERNEL_ACCOUNT);
	flow->keep = kvcalloc(BITS_TO_LONGS(len), sizeof(long), GFP_KERNEL_ACCOUNT);
	flow->states = kvmalloc_array(len, sizeof(*flow->states), GFP_KERNEL_ACCOUNT);
	flow->enter = kvmalloc_array(len, sizeof(*flow->enter), GFP_KERNEL_ACCOUNT);
	flow->nr_states = kvcalloc(len, sizeof(u32), GFP_KERNEL_ACCOUNT);
	flow->funcs = kvcalloc(env->subprog_cnt + 1, sizeof(*flow->funcs), GFP_KERNEL_ACCOUNT);
	preds = kvcalloc(len, 1, GFP_KERNEL_ACCOUNT);
	if (!flow->pos || !flow->rank || !flow->point || !flow->keep || !flow->states ||
	    !flow->enter ||
	    !flow->nr_states || !flow->funcs || !preds) {
		kvfree(preds);
		return -ENOMEM;
	}
	for (i = 0; i < len; i++) {
		INIT_LIST_HEAD(&flow->states[i].list);
		INIT_LIST_HEAD(&flow->enter[i].list);
	}
	for (i = 0; i <= env->subprog_cnt; i++)
		flow->funcs[i].exit_idx = -1;
	/* from here on there is something for bpf_flow_reset() to look at */
	flow->len = len;
	flow->nr_funcs = env->subprog_cnt + 1;

	/* subprog_topo_order[] has callees first */
	for (i = 0; i < env->subprog_cnt; i++)
		flow->rank[env->subprog_topo_order[i]] = env->subprog_cnt - i;

	for (i = 0; i < len; i++) {
		succ = bpf_insn_successors(env, i);
		iarray_for_each(s, succ) {
			if (preds[s] < 2)
				preds[s]++;
			if (aux[i].bb_end)
				__set_bit(s, flow->point);
		}
		/* where the callees come back to */
		if (bpf_pseudo_call(&insns[i]) || bpf_is_callx(&insns[i])) {
			if (i + 1 < len) {
				__set_bit(i + 1, flow->point);
				__set_bit(i + 1, flow->keep);
			}
		}
		if (aux[i].calls_callback) {
			__set_bit(i, flow->point);
			__set_bit(i, flow->keep);
		}
		if (aux[i].loop) {
			__set_bit(i, flow->point);
			__set_bit(i, flow->keep);
			kinds = FLOW_NEED_KINDS;
		}
		if (bpf_is_ldimm64(&insns[i]))
			i++;
	}
	for (i = 0; i < len; i++)
		if (preds[i] > 1)
			__set_bit(i, flow->keep);
	for (i = 0; i < env->subprog_cnt; i++) {
		__set_bit(env->subprog_info[i].start, flow->point);
		__set_bit(env->subprog_info[i].start, flow->keep);
	}
	kvfree(preds);

	err = flow_compute_order(env, flow);
	if (err)
		return err;
	for (i = 0; !err && i < kinds; i++)
		err = flow_compute_need_const(env, &flow->need[i], i);
	err = err ?: flow_compute_rungs(env);
	if (err)
		return err;
	return 0;
}

static void flow_free_state(struct bpf_verifier_env *env, struct flow_state *s)
{
	bpf_free_verifier_state(&s->st, false);
	kfree(s);
	env->flow->live_states--;
}

/* What a pass leaves behind. */
void bpf_flow_reset(struct bpf_verifier_env *env)
{
	struct bpf_flow *flow = env->flow;
	struct flow_state *s, *sn;
	struct flow_kept *k, *kn;
	struct flow_exit *x, *xn;
	struct flow_entry *e;
	int i;

	if (!flow || !flow->states || !flow->enter)
		return;
	for (i = 0; i < flow->len; i++) {
		list_for_each_entry_safe(s, sn, &flow->states[i].list, node) {
			list_del(&s->node);
			flow_free_state(env, s);
		}
		list_for_each_entry_safe(s, sn, &flow->enter[i].list, node) {
			list_del(&s->node);
			flow_free_state(env, s);
		}
		flow->nr_states[i] = 0;
	}
	flow->pending = RB_ROOT_CACHED;
	for (i = 0; i < flow->nr_funcs && flow->funcs; i++)
		flow->funcs[i].exit_idx = -1;
	for (i = 1; i <= flow->entry_cnt; i++) {
		e = flow->entries[i];
		list_for_each_entry_safe(s, sn, &e->calls, node) {
			bpf_free_verifier_state(&s->st, false);
			kfree(s);
		}
		list_for_each_entry_safe(x, xn, &e->exits, node) {
			bpf_free_verifier_state(&x->st, false);
			kfree(x);
		}
		kfree(e);
	}
	list_for_each_entry_safe(k, kn, &flow->kept, node) {
		bpf_free_verifier_state(&k->st, false);
		kfree(k);
	}
	INIT_LIST_HEAD(&flow->kept);
	bpf_free_verifier_state(&flow->tmp, false);
	memset(&flow->tmp, 0, sizeof(flow->tmp));
	flow->entry_cnt = 0;
	flow->united_cnt = 0;
}

void bpf_flow_free(struct bpf_verifier_env *env)
{
	struct bpf_flow *flow = env->flow;
	int i;

	if (!flow)
		return;
	bpf_flow_reset(env);
	kvfree(flow->pos);
	kvfree(flow->rank);
	for (i = 0; i < FLOW_NEED_KINDS; i++)
		kvfree(flow->need[i]);
	kvfree(flow->point);
	kvfree(flow->keep);
	kvfree(flow->states);
	kvfree(flow->enter);
	kvfree(flow->nr_states);
	kvfree(flow->funcs);
	kvfree(flow->entries);
	kvfree(flow->united);
	kfree(flow);
	env->flow = NULL;
}

bool bpf_flow_is_point(struct bpf_verifier_env *env, int insn_idx)
{
	return test_bit(insn_idx, env->flow->point);
}

/* A copy of @st that lives as long as the pass. */
struct bpf_verifier_state *bpf_flow_keep_state(struct bpf_verifier_env *env,
					       const struct bpf_verifier_state *st)
{
	struct flow_kept *k = kzalloc_obj(*k, GFP_KERNEL_ACCOUNT);

	if (!k)
		return NULL;
	list_add(&k->node, &env->flow->kept);
	if (bpf_copy_verifier_state(&k->st, st))
		return NULL;
	return &k->st;
}

/*
 * Both are in a loop whose iterations are kept apart: an iteration is done with before
 * the next one begins. A back edge in the middle of the loop would run ahead otherwise and
 * leave the rest of every iteration behind, see flow_earlier_trip(). The state that stands
 * for all iterations comes last.
 */
static int flow_cmp_trips(const struct bpf_func_state *fa, const struct bpf_func_state *fb)
{
	const struct loop_stack_entry *ea, *eb;
	u32 i, ta, tb;

	for (i = 0; i < fa->loop_stack_cnt && i < fb->loop_stack_cnt; i++) {
		ea = &fa->loop_stack[i];
		eb = &fb->loop_stack[i];
		if (ea->loop_id != eb->loop_id)
			break;
		ta = ea->apart ? ea->trip : U32_MAX;
		tb = eb->apart ? eb->trip : U32_MAX;
		if (ta != tb)
			return ta < tb ? -1 : 1;
	}
	return 0;
}

static bool flow_less(struct rb_node *na, const struct rb_node *nb)
{
	const struct flow_state *a = rb_entry(na, struct flow_state, rb);
	const struct flow_state *b = rb_entry(nb, struct flow_state, rb);
	int cmp;
	u32 i;

	for (i = 0; i < a->key_len && i < b->key_len; i++) {
		/* key[i] is the place in frame[i - 1] */
		if (i) {
			cmp = flow_cmp_trips(a->st.frame[i - 1], b->st.frame[i - 1]);
			if (cmp)
				return cmp < 0;
		}
		if (a->key[i] != b->key[i])
			return a->key[i] < b->key[i];
	}
	if (a->key_len != b->key_len)
		return a->key_len < b->key_len;
	return a->seq < b->seq;
}

/*
 * Callers first, so that the calls of a function get to it before it is looked
 * at. Then by the place of the call in every frame: what is called in the frames of
 * its callers is done with before what follows the call. What follows the call of
 * a function that is verified on its own goes on with every state that comes out.
 */
static void flow_set_pending(struct bpf_flow *flow, struct flow_state *s)
{
	const struct bpf_verifier_state *st = &s->st;
	u32 i;

	s->key[0] = flow->rank[st->frame[0]->subprogno];
	/* the place in the function makes the rank of the function redundant below */
	for (i = 0; i < st->curframe && i + 2 < ARRAY_SIZE(s->key); i++)
		s->key[i + 1] = flow->pos[st->frame[i + 1]->callsite];
	s->key[i + 1] = flow->pos[st->insn_idx];
	s->key_len = i + 2;
	s->pending = true;
	rb_add_cached(&s->rb, &flow->pending, flow_less);
}

static void flow_unlink(struct bpf_verifier_env *env, struct flow_state *s)
{
	struct bpf_flow *flow = env->flow;

	if (s->pending)
		rb_erase_cached(&s->rb, &flow->pending);
	s->pending = false;
	list_del(&s->node);
	flow->nr_states[s->st.insn_idx]--;
}

/* The entry that stands for the set that entry @id is in */
static struct flow_entry *flow_entry_find(struct bpf_flow *flow, u32 id)
{
	struct flow_entry *e = flow->entries[id];

	while (e->up != e) {
		e->up = e->up->up;
		e = e->up;
	}
	return e;
}

static int flow_entry_new(struct bpf_flow *flow)
{
	struct flow_entry **entries, *e;
	u32 cap;

	if (flow->entry_cnt + 1 >= flow->entry_cap) {
		cap = max(64U, flow->entry_cap * 2);
		entries = kvrealloc(flow->entries, cap * sizeof(*entries), GFP_KERNEL_ACCOUNT);
		if (!entries)
			return -ENOMEM;
		flow->entries = entries;
		flow->entry_cap = cap;
	}
	e = kmalloc_obj(*e, GFP_KERNEL_ACCOUNT);
	if (!e)
		return -ENOMEM;
	e->up = e;
	INIT_LIST_HEAD(&e->calls);
	INIT_LIST_HEAD(&e->exits);
	flow->entries[++flow->entry_cnt] = e;
	return flow->entry_cnt;
}

/* It delivers states, so it is done when nothing is in the middle of that. */
static int flow_unite(struct bpf_flow *flow, u32 a, u32 b)
{
	struct flow_united *u;
	u32 cap;

	if (!a || !b || flow_entry_find(flow, a) == flow_entry_find(flow, b))
		return 0;
	if (flow->united_cnt == flow->united_cap) {
		cap = max(16U, flow->united_cap * 2);
		u = kvrealloc(flow->united, cap * sizeof(*u), GFP_KERNEL_ACCOUNT);
		if (!u)
			return -ENOMEM;
		flow->united = u;
		flow->united_cap = cap;
	}
	u = &flow->united[flow->united_cnt++];
	u->a = a;
	u->b = b;
	return 0;
}

/*
 * What is known about a counted loop comes from the state it was entered
 * with. @t stands for @s only if that holds for these states too.
 */
static bool flow_loops_within(struct bpf_verifier_env *env, const struct bpf_verifier_state *t,
			      const struct bpf_verifier_state *s)
{
	const struct loop_stack_entry *lt, *ls;
	u32 fr, i;

	for (fr = 0; fr <= t->curframe; fr++) {
		if (t->frame[fr]->loop_stack_cnt != s->frame[fr]->loop_stack_cnt)
			return false;
		for (i = 0; i < t->frame[fr]->loop_stack_cnt; i++) {
			lt = &t->frame[fr]->loop_stack[i];
			ls = &s->frame[fr]->loop_stack[i];
			if (!lt->terminates || lt->entry_state == ls->entry_state)
				continue;
			if (!ls->terminates ||
			    !bpf_state_subsumes(env, lt->entry_state, ls->entry_state))
				return false;
		}
	}
	return true;
}

static bool flow_within(struct bpf_verifier_env *env, struct bpf_verifier_state *t,
			struct bpf_verifier_state *s)
{
	return t->curframe == s->curframe &&
	       t->frame[0]->subprogno == s->frame[0]->subprogno &&
	       /* an async callback returns to the kernel, a call of the function to its caller */
	       !t->flow_entry == !s->flow_entry &&
	       bpf_state_subsumes(env, t, s) && flow_loops_within(env, t, s);
}

/*
 * How many iterations is @t behind @s, in the loop in which that is the most? Only the loops
 * whose iterations are kept apart are looked at.
 * 0 if both are in the same iteration of every loop, U32_MAX if @t is ahead of @s in a loop.
 */
static u32 flow_trips_behind(const struct bpf_verifier_state *t, const struct bpf_verifier_state *s)
{
	const struct loop_stack_entry *lt, *ls;
	u32 fr, i, behind = 0;

	if (t->curframe != s->curframe)
		return U32_MAX;
	for (fr = 0; fr <= t->curframe; fr++) {
		if (t->frame[fr]->loop_stack_cnt != s->frame[fr]->loop_stack_cnt)
			return U32_MAX;
		for (i = 0; i < t->frame[fr]->loop_stack_cnt; i++) {
			lt = &t->frame[fr]->loop_stack[i];
			ls = &s->frame[fr]->loop_stack[i];
			if (!lt->apart && !ls->apart)
				continue;
			if (lt->trip > ls->trip)
				return U32_MAX;
			behind = max(behind, ls->trip - lt->trip);
		}
	}
	return behind;
}

/*
 * A state is compared with the states of its own iteration and of the two before it. The older ones
 * are gone, see flow_earlier_trip(), but for what is pending or waits: every iteration of a loop
 * calls a recursive function, and what comes out of it goes to all the calls at once.
 */
static bool flow_trips_near(const struct bpf_verifier_state *t, const struct bpf_verifier_state *s)
{
	return flow_trips_behind(t, s) <= 2;
}

/*
 * @s is within @t, and from now on @t stands for it.
 *
 * A walk on patience gets to the end of a loop when no state comes back to the head any more.
 * Then the loop ends at run time too: the number of the iteration grows at the head, nothing makes
 * it smaller, and there is a largest one.
 * If @t is of an earlier iteration than @s that is not true any more. What becomes of @t got to @s
 * and is @t again: as far as the verifier can see the program goes around for ever. A spin lock
 * is such a loop. So the loop gets may_goto.
 *
 * It's asked for every loop that @t is behind in, not only for the outermost one. This is the only
 * way for the number of a loop to get smaller. In a walk it doesn't change in another way but at
 * the head: a state never stands for a state of an earlier iteration, see loop_stack_safe(), and it
 * takes the place of, or is joined with, states of its own iterations only, see flow_merge().
 * Where the iterations are not kept apart a join has the larger number, and a state may stand
 * for one with a smaller number. Such loop has a count or may_goto, or something stops it, since
 * the state entered it.
 */
static int flow_cycle_closed(struct bpf_verifier_env *env, const struct bpf_verifier_state *t,
			     const struct bpf_verifier_state *s)
{
	const struct loop_stack_entry *lt, *ls;
	struct bpf_loop *loop;
	u32 fr, i;

	for (fr = 0; fr <= t->curframe; fr++) {
		for (i = 0; i < t->frame[fr]->loop_stack_cnt; i++) {
			lt = &t->frame[fr]->loop_stack[i];
			ls = &s->frame[fr]->loop_stack[i];
			if (lt->trip >= ls->trip)
				continue;
			/* counted: what comes back is expected to be within what is at the head */
			if (lt->terminates && !bpf_loop_walk_may_give_up(lt))
				continue;
			loop = env->insn_aux_data[lt->loop_id].loop;
			/* it goes around for as long as the iterator has elements or may_goto budget */
			if (loop->rt_bounded)
				continue;
			if (!loop->has_guards) {
				verbose(env, "loop at insn %d never exits\n", lt->loop_id);
				return -EINVAL;
			}
			bpf_flow_loop_gets_guards(env, lt->loop_id);
		}
	}
	return 0;
}

/*
 * @t is of an earlier iteration of a loop whose iterations are kept apart than @s.
 * What comes after @s is of later iterations too, so @t is not going to be
 * of use. If something of its iteration does come it is looked at again.
 *
 * Not so where a state stands for many iterations. States of different kinds are at the head of
 * such loop. One of them comes to rest early, and what comes around is within it for as long as
 * the others take.
 */
static bool flow_earlier_trip(const struct bpf_verifier_state *t,
			      const struct bpf_verifier_state *s)
{
	const struct bpf_func_state *ft, *fs;
	u32 fr, i;

	if (t->curframe != s->curframe)
		return false;
	for (fr = 0; fr <= t->curframe; fr++) {
		ft = t->frame[fr];
		fs = s->frame[fr];
		if (ft->subprogno != fs->subprogno || ft->callsite != fs->callsite ||
		    ft->loop_stack_cnt != fs->loop_stack_cnt)
			return false;
		for (i = 0; i < ft->loop_stack_cnt; i++) {
			if (ft->loop_stack[i].loop_id != fs->loop_stack[i].loop_id ||
			    ft->loop_stack[i].apart != fs->loop_stack[i].apart)
				return false;
			if (ft->loop_stack[i].apart &&
			    ft->loop_stack[i].trip != fs->loop_stack[i].trip)
				return ft->loop_stack[i].trip + 1 < fs->loop_stack[i].trip;
		}
	}
	return false;
}

/* Can flow_earlier_trip() be true for a state and @s? */
static bool flow_has_earlier_trips(const struct bpf_verifier_state *s)
{
	const struct loop_stack_entry *entry;
	u32 fr, i;

	for (fr = 0; fr <= s->curframe; fr++) {
		for (i = 0; i < s->frame[fr]->loop_stack_cnt; i++) {
			entry = &s->frame[fr]->loop_stack[i];
			if (entry->apart && entry->trip > 1)
				return true;
		}
	}
	return false;
}

/* @flag: 0 and 1 */
static bool flow_two_consts(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb,
			    bool flag)
{
	return ra->type == SCALAR_VALUE && rb->type == SCALAR_VALUE &&
	       tnum_is_const(ra->var_off) && tnum_is_const(rb->var_off) &&
	       ra->var_off.value != rb->var_off.value &&
	       (!flag || (ra->var_off.value | rb->var_off.value) == 1);
}

/* Constants that have to stay constants, see flow_compute_need_const() */
static bool flow_needed_consts_differ(const struct bpf_flow *flow, enum flow_need_kind kind,
				      const struct bpf_verifier_state *a,
				      const struct bpf_verifier_state *b)
{
	const struct flow_need *needs = flow->need[kind], *need;
	const struct bpf_stack_state *sa, *sb;
	const struct bpf_func_state *fa, *fb;
	bool flag = kind == FLOW_NEED_FLAG;
	u32 fr, i, nr;

	if (!needs || a->curframe != b->curframe)
		return false;
	for (fr = 0; fr <= a->curframe; fr++) {
		/* the callers need what they need when the call returns */
		need = &needs[fr == a->curframe ? a->insn_idx : a->frame[fr + 1]->callsite + 1];
		fa = a->frame[fr];
		fb = b->frame[fr];
		for (i = BPF_REG_0; need->regs && i < BPF_REG_FP; i++)
			if ((need->regs & BIT(i)) &&
			    flow_two_consts(&fa->regs[i], &fb->regs[i], flag))
				return true;
		nr = min(bpf_stack_nr_slots(fa), bpf_stack_nr_slots(fb));
		for (i = 0; i < nr && i < 64 && need->slots >> i; i++) {
			sa = bpf_stack_slot(fa, i);
			sb = bpf_stack_slot(fb, i);
			if ((need->slots & BIT_ULL(i)) &&
			    bpf_is_spilled_scalar_reg(sa) && bpf_is_spilled_scalar_reg(sb) &&
			    flow_two_consts(&sa->spilled_ptr, &sb->spilled_ptr, flag))
				return true;
		}
	}
	return false;
}

/* regsafe() and join_reg() say what each type needs */
static void flow_shape_ptr(struct flow_shape *sh, u32 place, const struct bpf_reg_state *reg)
{
	u32 ids = (reg->id != 0) | (reg->parent_id != 0) << 1 | (reg->map_uid != 0) << 2;
	u32 len = offsetof(struct bpf_reg_state, var_off);
	bool has_off = false, var_off = false;
	u64 off = reg->var_off.value;

	switch (base_type(reg->type)) {
	case NOT_INIT:
	case SCALAR_VALUE:
		return;
	case PTR_TO_ARENA:
		/* the offset is not tracked */
		len = sizeof(reg->type);
		ids = 0;
		break;
	case PTR_TO_PACKET_META:
	case PTR_TO_PACKET:
		len = sizeof(reg->type);
		ids &= 1;
		/* behind the end of the packet, or at the end: the same in both states */
		if (reg->range < 0)
			ids |= -reg->range << 1;
		else
			sh->pkt_range += reg->range;
		if (reg->id) {
			/* displacements between pointers that share an id are kept */
			has_off = true;
			off = reg->r64.base;
		} else if (reg->range >= 0) {
			/* join_reg() takes two such constants for two places in the packet */
			has_off = true;
			var_off = !tnum_is_const(reg->var_off) || off > MAX_PACKET_OFF;
		}
		break;
	case PTR_TO_INSN:
		/* a range of offsets is a set of targets */
		ids = 0;
		break;
	case PTR_TO_MAP_KEY:
	case PTR_TO_MAP_VALUE:
	case PTR_TO_MEM:
	case PTR_TO_BUF:
	case PTR_TO_TP_BUFFER:
	case PTR_TO_STACK:
	case PTR_TO_BTF_ID:
		has_off = true;
		var_off = !tnum_is_const(reg->var_off);
		break;
	default:
		len = offsetof(struct bpf_reg_state, id);
	}
	sh->word[FLOW_LAYOUT] = jhash_1word(place << 1 | var_off, sh->word[FLOW_LAYOUT]);
	sh->word[FLOW_PTRS] = jhash2((const u32 *)reg, len / sizeof(u32),
				     sh->word[FLOW_PTRS] + ids);
	if (has_off && !var_off)
		sh->word[FLOW_OFFS] = jhash_2words(lower_32_bits(off), upper_32_bits(off),
						   sh->word[FLOW_OFFS]);
}

/* stacksafe() and join_special_slot() say what each kind needs */
static void flow_shape_special(struct flow_shape *sh, u32 place, const struct bpf_stack_state *slot)
{
	const struct bpf_reg_state *reg = &slot->spilled_ptr;
	u32 kind = slot->slot_type[BPF_REG_SIZE - 1] << 2 | (reg->id != 0);
	u32 *ptrs = &sh->word[FLOW_PTRS];

	switch (slot->slot_type[BPF_REG_SIZE - 1]) {
	case STACK_DYNPTR:
		kind |= (reg->parent_id != 0) << 1;
		*ptrs = jhash_3words(kind, reg->dynptr.type, reg->dynptr.first_slot, *ptrs);
		break;
	case STACK_ITER:
		*ptrs = jhash_3words(lower_32_bits((unsigned long)reg->iter.btf),
				     upper_32_bits((unsigned long)reg->iter.btf),
				     reg->iter.btf_id, *ptrs + kind);
		*ptrs = jhash_3words(reg->type, reg->iter.state, reg->iter.loops, *ptrs);
		break;
	case STACK_IRQ_FLAG:
		*ptrs = jhash_2words(kind, reg->irq.kfunc_class, *ptrs);
		break;
	}
	sh->word[FLOW_LAYOUT] = jhash_1word(place << 1, sh->word[FLOW_LAYOUT]);
}

/* The part of the shape that is made of the registers and the stack, when it is asked for */
static void flow_shape_places(struct bpf_verifier_env *env, struct flow_state *s)
{
	struct bpf_verifier_state *st = &s->st;
	const struct bpf_stack_state *slot;
	const struct bpf_func_state *frame;
	struct flow_shape *sh = &s->shape;
	u32 fr, i, args;
	u16 live;

	sh->places = true;
	for (fr = 0; fr <= st->curframe; fr++) {
		frame = st->frame[fr];
		/* a state that does a call again is not cleaned, dead registers are not compared */
		live = env->insn_aux_data[bpf_frame_insn_idx(st, fr)].live_regs_before;
		for (i = BPF_REG_0; i < BPF_REG_FP; i++)
			if (live & BIT(i))
				flow_shape_ptr(sh, fr << 10 | i, &frame->regs[i]);
		for (i = 0; i < bpf_stack_nr_slots(frame); i++) {
			slot = bpf_stack_slot(frame, i);
			switch (slot->slot_type[BPF_REG_SIZE - 1]) {
			case STACK_SPILL:
				flow_shape_ptr(sh, fr << 10 | (i + 16), &slot->spilled_ptr);
				break;
			case STACK_DYNPTR:
			case STACK_ITER:
			case STACK_IRQ_FLAG:
				flow_shape_special(sh, fr << 10 | (i + 16), slot);
				break;
			}
		}
		/* incoming arguments are not compared with outgoing ones: other places */
		args = 16 + MAX_BPF_STACK_SLOTS;
		if (bpf_stack_args_are_incoming(frame))
			args += MAX_STACK_ARG_SLOTS;
		for (i = 0; i < frame->out_stack_arg_cnt; i++)
			flow_shape_ptr(sh, fr << 10 | (i + args), &frame->stack_arg_regs[i]);
	}
}

/* The part of the shape that is not */
static void flow_shape_ctx(const struct bpf_verifier_state *st, struct flow_shape *sh)
{
	const struct bpf_reference_state *ref;
	const struct loop_stack_entry *entry;
	const struct bpf_func_state *frame;
	u32 chain = st->curframe, ctx, fr, i;

	ctx = jhash_3words(st->in_sleepable | st->callers_unknown << 1 |
			   st->unwound << 2 | st->changed_pkt_data << 3 |
			   (st->active_irq_id != 0) << 4 | (st->active_lock_id != 0) << 5,
			   st->acquired_refs, st->active_locks, 0);
	ctx = jhash_2words(st->active_preempt_locks, st->active_rcu_locks, ctx);
	ctx = jhash_2words(lower_32_bits((unsigned long)st->active_lock_ptr),
			   upper_32_bits((unsigned long)st->active_lock_ptr), ctx);
	for (i = 0; i < st->acquired_refs; i++) {
		ref = &st->refs[i];
		ctx = jhash_1word(ref->type, ctx);
		if (ref->type != REF_TYPE_PTR && ref->type != REF_TYPE_IRQ)
			ctx = jhash_2words(lower_32_bits((unsigned long)ref->ptr),
					   upper_32_bits((unsigned long)ref->ptr), ctx);
	}
	for (fr = 0; fr <= st->curframe; fr++) {
		frame = st->frame[fr];
		chain = jhash_3words(frame->subprogno, frame->callsite, frame->loop_stack_cnt,
				     chain);
		ctx = jhash_2words(frame->entry_refs, frame->in_rcu_callback_fn, ctx);
		ctx = jhash_2words(frame->entry_rcu_locks, frame->entry_preempt_locks, ctx);
		for (i = 0; i < frame->loop_stack_cnt; i++) {
			entry = &frame->loop_stack[i];
			chain = jhash_1word(entry->loop_id << 1 | entry->apart, chain);
			ctx = jhash_3words(entry->counted << 3 | entry->guarded << 2 |
					   entry->first << 1 | entry->terminates,
					   entry->max_header_count,
					   entry->counted ? entry->trip : 0, ctx);
		}
	}
	memset(sh, 0, sizeof(*sh));
	sh->word[FLOW_CHAIN] = chain;
	sh->word[FLOW_CTX] = ctx;
}

/* Are the words up to @last the same? */
static __always_inline bool flow_same_shape(const struct flow_state *a, const struct flow_state *b,
					    enum flow_word last)
{
	u32 i;

	for (i = 0; i <= last; i++)
		if (a->shape.word[i] != b->shape.word[i])
			return false;
	return true;
}

/*
 * A state that arrives is compared with the states at the insn: is it within one of them, see
 * bpf_state_subsumes(), does it have a join with one of them, see bpf_states_join()? Most of
 * them have nothing to do with it. The shape of a state tells that before the registers are
 * compared or, for a join, copied. It has what must be the same in two states that are related:
 *
 * FLOW_CHAIN	the chain of calls and the loops that every frame is in
 * FLOW_CTX	what is known about every loop, the iteration of a counted one,
 *		the flags of the state and of its frames, locks and references
 * FLOW_LAYOUT	which registers, stack slots and stack arguments hold a pointer, a dynptr,
 *		an iterator or an irq flag, which of the pointers are at a constant offset
 * FLOW_PTRS	what the pointers point to and which of their ids are set, the kind of the others
 * FLOW_OFFS	the constant offsets
 *
 * Nothing about numbers is in it.
 *
 * States that differ in one of the first two words are not related. Neither are states of
 * the same layout that differ in one of the last two: a pointer is related to a pointer to the same
 * thing only, and two constant offsets are two places in it.
 * States of different layout may be related, whatever the last two words are: nothing in
 * a register or in a slot stands for a pointer there, and a range of offsets stands for a constant,
 * or is joined with it.
 *
 * A word is a hash. It is a filter and proves nothing: the states that get through it go
 * through all the comparisons. So it's fine to look at the first two words only: @places.
 */
static bool flow_may_relate(struct bpf_verifier_env *env, struct flow_state *t,
			    struct flow_state *s, bool places)
{
	if (!flow_same_shape(t, s, FLOW_CTX))
		return false;
	if (!places)
		return true;
	if (!t->shape.places)
		flow_shape_places(env, t);
	if (!s->shape.places)
		flow_shape_places(env, s);
	return !flow_same_shape(t, s, FLOW_LAYOUT) || flow_same_shape(t, s, FLOW_OFFS);
}

/*
 * A pointer to the packet stands for one with at least as many checked bytes, and without
 * widening it's joined with one that has as many. In states of the same layout that holds for
 * the sums too. Returns 0 if the sums are the same, or say nothing.
 */
static int flow_cmp_pkt_range(const struct flow_state *t, const struct flow_state *s)
{
	if (!t->shape.places || !s->shape.places || !flow_same_shape(t, s, FLOW_LAYOUT) ||
	    t->shape.pkt_range == s->shape.pkt_range)
		return 0;
	return t->shape.pkt_range < s->shape.pkt_range ? -1 : 1;
}

/*
 * bpf_loop(N, ...) with a small N: the calls of the callback are looked at one by one, like
 * the iterations of a loop that is walked. The call stops to enter the callback when
 * callback_depth gets to N.
 */
static bool flow_callback_depth_differs(struct bpf_verifier_env *env,
					const struct bpf_verifier_state *a,
					const struct bpf_verifier_state *b)
{
	u32 max = BPF_FLOW_MAX_CALLBACKS, fr;
	const struct bpf_func_state *fa, *fb;
	struct bpf_insn *insn;

	if (a->curframe != b->curframe)
		return false;
	/* at the call, and in the callback and what it calls */
	for (fr = 0; fr <= a->curframe; fr++) {
		insn = &env->prog->insnsi[fr == a->curframe ? a->insn_idx :
					  a->frame[fr + 1]->callsite];
		if (!bpf_helper_call(insn) || insn->imm != BPF_FUNC_loop)
			continue;
		fa = a->frame[fr];
		fb = b->frame[fr];
		if (fa->callback_depth != fb->callback_depth &&
		    fa->regs[BPF_REG_1].type == SCALAR_VALUE &&
		    reg_umax(&fa->regs[BPF_REG_1]) <= max &&
		    fb->regs[BPF_REG_1].type == SCALAR_VALUE &&
		    reg_umax(&fb->regs[BPF_REG_1]) <= max)
			return true;
	}
	return false;
}

static const struct bpf_reg_state *flow_slot_number(const struct bpf_func_state *frame, u32 i)
{
	const struct bpf_stack_state *slot = bpf_stack_slot(frame, i);

	if (slot->slot_type[0] != STACK_SPILL || slot->slot_type[BPF_REG_SIZE - 1] != STACK_SPILL ||
	    slot->spilled_ptr.type != SCALAR_VALUE)
		return NULL;
	return &slot->spilled_ptr;
}

/* Returns true when the answer is known */
static bool flow_null_pair(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb,
			   bool *a_null, bool *b_null)
{
	if (tnum_equals_const(ra->var_off, 0) && !tnum_is_const(rb->var_off)) {
		*a_null = true;
		if (reg_umin(rb) == 1 && reg_umax(rb) == U64_MAX)
			return true;
	}
	if (tnum_equals_const(rb->var_off, 0) && !tnum_is_const(ra->var_off)) {
		*b_null = true;
		if (reg_umin(ra) == 1 && reg_umax(ra) == U64_MAX)
			return true;
	}
	return *a_null && *b_null;
}

/*
 * Two things of which one is there: "if (hdr->ipv4) ... else if (hdr->ipv6)". The one that
 * is not there is 0. What the other one is may be unknown, e.g. a pointer to the packet
 * after the packet was changed. (0, x) is not within (y, 0) and the join (?, ?) forgets that
 * it's one or the other.
 * The same for a number that was compared with 0: 0 on one path, not 0 on the other.
 *
 * Only where a jump ahead can see it: FLOW_NEED_JUMP, see flow_join_makes_up(). What no jump
 * looks at again sends no state another way, and every such number doubles the states.
 *
 *	for (i = 0; i < n; i++) { p = a[i]; if (!p) continue; put(p); }
 * a[] is on the stack. The jump finds out about the slot that p was filled from too. Nothing
 * looks at a[i] again, but the loop reads through a pointer that moves, so all of a[] is live
 * until the loop is done: 2^n states if they stay apart.
 * They don't, because FLOW_NEED_JUMP has only what is filled through r10, see flow_fp_slot().
 * It cannot tell a[i] from a[i + 1]. With what is filled through other pointers to the stack in
 * it, all of a[] is tested in all of the loop.
 */
static bool flow_null_differs(const struct bpf_flow *flow, const struct bpf_verifier_state *a,
			      const struct bpf_verifier_state *b)
{
	const struct bpf_func_state *fa, *fb;
	const struct bpf_reg_state *ra, *rb;
	const struct flow_need *need = flow->need[FLOW_NEED_JUMP];
	bool a_null = false, b_null = false;
	u32 i;

	if (!need || a->curframe != b->curframe)
		return false;
	need += a->insn_idx;
	fa = a->frame[a->curframe];
	fb = b->frame[b->curframe];
	for (i = BPF_REG_0; i < BPF_REG_FP; i++) {
		ra = &fa->regs[i];
		rb = &fb->regs[i];
		if ((need->regs & BIT(i)) &&
		    ra->type == SCALAR_VALUE && rb->type == SCALAR_VALUE &&
		    flow_null_pair(ra, rb, &a_null, &b_null))
			return true;
	}
	for (i = 0; i < 64 && i < bpf_stack_nr_slots(fa) && i < bpf_stack_nr_slots(fb); i++) {
		if (!(need->slots & BIT_ULL(i)))
			continue;
		ra = flow_slot_number(fa, i);
		rb = flow_slot_number(fb, i);
		if (ra && rb && flow_null_pair(ra, rb, &a_null, &b_null))
			return true;
	}
	return false;
}

static const struct bpf_reg_state *flow_number_at(const struct bpf_func_state *frame, u8 loc)
{
	const struct bpf_reg_state *reg;

	if (loc <= BPF_REG_FP)
		reg = &frame->regs[loc];
	else if (loc - BPF_REG_FP - 1 < bpf_stack_nr_slots(frame) &&
		 bpf_is_spilled_scalar_reg(bpf_stack_slot(frame, loc - BPF_REG_FP - 1)))
		reg = &bpf_stack_slot(frame, loc - BPF_REG_FP - 1)->spilled_ptr;
	else
		return NULL;
	return reg->type == SCALAR_VALUE ? reg : NULL;
}

/*
 * A register is set in one of the states and not in the other. Only live registers are in
 * a state, so something ahead may read it. The state in which it's not set doesn't get there:
 * a jump sends it another way. The join would not know which way to go, and would get
 * there without the register. There is nothing to gain from such a join but a state less.
 */
static bool flow_unset_differs(const struct bpf_verifier_state *a,
			       const struct bpf_verifier_state *b)
{
	u32 fr, i;

	for (fr = 0; fr <= a->curframe; fr++)
		for (i = BPF_REG_0; i < BPF_REG_FP; i++)
			if ((a->frame[fr]->regs[i].type == NOT_INIT) !=
			    (b->frame[fr]->regs[i].type == NOT_INIT))
				return true;
	return false;
}

static bool flow_disjoint(const struct bpf_reg_state *a, const struct bpf_reg_state *b)
{
	return cnum64_is_empty(cnum64_intersect(a->r64, b->r64)) ||
	       cnum32_is_empty(cnum32_intersect(a->r32, b->r32)) ||
	       ((a->var_off.value ^ b->var_off.value) & ~(a->var_off.mask | b->var_off.mask));
}

/* The smallest range that has both has numbers that are in neither */
static bool flow_gap(const struct bpf_reg_state *a, const struct bpf_reg_state *b)
{
	if (cnum64_is_empty(cnum64_intersect(a->r64, b->r64)) &&
	    a->r64.size + b->r64.size + 1 > a->r64.size &&
	    cnum64_union(a->r64, b->r64).size > a->r64.size + b->r64.size + 1)
		return true;
	return cnum32_is_empty(cnum32_intersect(a->r32, b->r32)) &&
	       cnum32_union(a->r32, b->r32).size > (u64)a->r32.size + b->r32.size + 1;
}

/*
 * Numbers always have a join. It must not make up what neither state has where a jump ahead
 * can see it. FLOW_NEED_JUMP has the registers and the stack slots that a jump ahead compares
 * with a constant, or the verifier does at a call or at the exit.
 *
 *	if (ret < 0) goto err; if (ret > 0) goto skip; use(p);
 * ret = [-2, -1] and ret = [1, 2], both without p, meet in front of that. [-2, 2] has 0, gets
 * through both jumps, and there is no p. So two ranges that leave a gap are not joined. Ranges
 * that touch, 0 and 1, -1 and 0, or have something in common make up no number.
 *
 *	err = update(); ret = err < 0 ? DROP : 0; ... if (err < 0) goto out; ... if (ret < 0) ...
 * (err >= 0, ret = 0) and (err < 0, ret = DROP) meet. The join has err < 0 with ret = 0. So
 * two numbers, each with nothing in common between the states, are not joined either.
 *
 *	fatal = err == -ENOMEM; ... if (fatal) bpf_set_retval(err);
 * (fatal = 0, err = 1) and (fatal = 1, err = -ENOMEM) meet. The same: the join has
 * fatal = 1 with err = 1, which bpf_set_retval() doesn't take.
 *
 *	ok = i < 64 && in[i]; if (x) n++; if (ok) a[i] = 1;
 * (ok = 0, i >= 64), (ok = 0, i < 64) and (ok = 1, i < 64) meet. 0 and 1 touch, and i < 64 comes
 * with either, so none of the above keeps them apart. The join has ok = 1 with i >= 64. A flag is
 * made to send states different ways. So 0 and 1 are not joined where a jump that is not far
 * tells one from the other: FLOW_NEED_FLAG.
 *	while (more) { ...; more = next(&x); }
 * That jump can be the one that leaves a loop. It is decided in both states, one stays and one
 * leaves. It's the jump that set the flag, which went both ways, once more: no sign that the
 * loop gets anywhere. A walk on patience has to know, see mark_loop_exit_decided().
 *
 * The states go on to the jumps on their own. Behind the last one the numbers are dead, and what
 * is left is joined.
 */
static bool flow_join_makes_up(const struct bpf_flow *flow, struct bpf_verifier_state *a,
			       struct bpf_verifier_state *b)
{
	struct bpf_func_state *fa = a->frame[a->curframe], *fb = b->frame[b->curframe];
	const struct bpf_reg_state *ra, *rb;
	const struct flow_need *tested = flow->need[FLOW_NEED_JUMP], *need;
	u32 fr, i, id = 0;
	bool one = false;

	if (!tested)
		return false;
	for (fr = 0; fr <= a->curframe; fr++) {
		/* the callers go on when the call returns */
		need = &tested[fr == a->curframe ? a->insn_idx : a->frame[fr + 1]->callsite + 1];
		for (i = BPF_REG_0; i < BPF_REG_FP + 1 + 64; i++) {
			if (i <= BPF_REG_FP ? !(need->regs & BIT(i)) :
					      !(need->slots & BIT_ULL(i - BPF_REG_FP - 1)))
				continue;
			ra = flow_number_at(a->frame[fr], i);
			rb = flow_number_at(b->frame[fr], i);
			if (!ra || !rb || !flow_disjoint(ra, rb))
				continue;
			if (flow_gap(ra, rb))
				return true;
			/* copies of a number are one number */
			if (one && (!id || id != (ra->id & ~BPF_ADD_CONST)))
				return true;
			one = true;
			id = ra->id & ~BPF_ADD_CONST;
		}
	}
	if (!flow_needed_consts_differ(flow, FLOW_NEED_FLAG, a, b))
		return false;
	for (i = 0; i < fa->loop_stack_cnt; i++)
		fa->loop_stack[i].flag_apart = fa->loop_stack[i].probe;
	for (i = 0; i < fb->loop_stack_cnt; i++)
		fb->loop_stack[i].flag_apart = fb->loop_stack[i].probe;
	return true;
}

/*
 * @s came back to the head of a loop, or from a callback to the helper that calls it. @t is there.
 * Do they come from two states that entered the cycle?
 */
static bool flow_entries_differ(const struct bpf_verifier_state *t,
				const struct bpf_verifier_state *s, enum flow_cycle cycle)
{
	const struct bpf_func_state *ft = t->frame[t->curframe], *fs = s->frame[s->curframe];
	u32 cnt = fs->loop_stack_cnt;

	if (cycle == FLOW_CALLBACK_CYCLE)
		return ft->callback_entry_id != fs->callback_entry_id;
	return cnt && ft->loop_stack_cnt == cnt &&
	       ft->loop_stack[cnt - 1].entry_id != fs->loop_stack[cnt - 1].entry_id;
}

/*
 * States that have a join are kept apart while there is room: half of every limit.
 * After that they are joined, which may lose what the prog needs, and does not run into the limit.
 */
static bool flow_has_room(const struct bpf_verifier_env *env, u32 nr_states)
{
	u64 len = env->prog->len;

	return nr_states < BPF_FLOW_MAX_STATES / 2 &&
	       env->flow->live_states < (BPF_FLOW_LIVE_PER_INSN * len + BPF_FLOW_LIVE_BASE) / 2 &&
	       env->total_states < (BPF_FLOW_TOTAL_PER_INSN * len + BPF_FLOW_TOTAL_BASE) / 2;
}

/* The state of @set behind @t, the first one if @t is NULL, that @s may be related to */
static struct flow_state *flow_next_related(struct bpf_verifier_env *env,
					    const struct flow_set *set, struct flow_state *s,
					    struct flow_state *t, bool places)
{
	t = t ? list_next_entry(t, node) : list_first_entry(&set->list, struct flow_state, node);
	list_for_each_entry_from(t, &set->list, node)
		if (flow_may_relate(env, t, s, places))
			return t;
	return NULL;
}

static void flow_link(struct bpf_verifier_env *env, struct flow_set *set, struct flow_state *s)
{
	list_add_tail(&s->node, &set->list);
	env->flow->nr_states[s->st.insn_idx]++;
}

static void flow_sweep_trip(struct bpf_verifier_env *env, struct flow_state *t,
			    const struct flow_state *s)
{
	if (t->pending || !flow_earlier_trip(&t->st, &s->st))
		return;
	flow_unlink(env, t);
	flow_free_state(env, t);
}

/* States of earlier iterations than @s go away, see flow_earlier_trip() */
static void flow_sweep_trips(struct bpf_verifier_env *env, struct flow_set *set,
			     const struct flow_state *s)
{
	struct flow_state *t, *tn;

	if (!flow_has_earlier_trips(&s->st))
		return;
	list_for_each_entry_safe(t, tn, &set->list, node)
		if (flow_same_shape(t, s, FLOW_CHAIN))
			flow_sweep_trip(env, t, s);
}

/*
 * Put *@ps into @set, one of the sets of states at its insn. Returns
 * 1 if it is within a state of the set. It is gone then.
 * 0 if it is in the set now, as it was or joined with states of the set.
 * @cycle: it came around a cycle, what grew is forgotten.
 */
static int flow_merge(struct bpf_verifier_env *env, struct flow_set *set,
		      struct flow_state **ps, enum flow_cycle cycle)
{
	struct bpf_flow *flow = env->flow;
	struct flow_state *s = *ps, *t, *tn;
	int insn_idx = s->st.insn_idx, err;
	u32 *nr_states = &flow->nr_states[insn_idx];
	bool places, widen_all, widen;
	/* the set of the states that enter a loop is a stop on the way, see bpf_flow_deliver() */
	bool log = (env->log.level & BPF_LOG_LEVEL2) && set == &flow->states[insn_idx];
	const char *what = NULL;
	int prev = s->prev_insn_idx;

again:
	if (signal_pending(current)) {
		err = -EAGAIN;
		goto err_free;
	}
	if (need_resched())
		cond_resched();

	/* again after a join: an offset may not be a constant any more, a pointer may be gone */
	flow_shape_ctx(&s->st, &s->shape);
	places = *nr_states >= BPF_FLOW_MIN_SHAPES;
	for (t = flow_next_related(env, set, s, NULL, places); t;
	     t = flow_next_related(env, set, s, t, places)) {
		if (flow_cmp_pkt_range(t, s) > 0 || !flow_trips_near(&t->st, &s->st) ||
		    !flow_within(env, &t->st, &s->st))
			continue;
		if (env->log.level & BPF_LOG_LEVEL2)
			verbose(env, "from %d to %d: within\n", prev, insn_idx);
		err = flow_cycle_closed(env, &t->st, &s->st) ?:
		      flow_unite(flow, s->st.flow_entry, t->st.flow_entry);
		flow_free_state(env, s);
		return err ?: 1;
	}

	flow_sweep_trips(env, set, s);

	widen_all = cycle == FLOW_CYCLE;
	for (t = flow_next_related(env, set, s, NULL, places); t; t = tn) {
		tn = flow_next_related(env, set, s, t, places);
		/*
		 * Iterations that are kept apart have no join. Neither does a state take the place
		 * of a state of another iteration that it covers: that one is done with, or has its
		 * own way to go, see flow_cycle_closed().
		 * The shape is a hash, and what follows goes through the frames of both.
		 */
		if (flow_trips_behind(&t->st, &s->st))
			continue;
		widen = widen_all || (cycle == FLOW_CALLBACK_CYCLE &&
				      max(s->joins, t->joins) >= BPF_FLOW_CALLBACK_JOINS);
		/*
		 *	ret = 0; if (...) { denied++; ret = -EPERM; }
		 *	bpf_for(i, 0, n) { ... } return ret;
		 * Two states enter the loop. What comes around it with 0 is for the state with 0.
		 * Next to -EPERM it looks like a number that grew, and would be forgotten. The same
		 * for the header of a packet at pkt + 34 and at pkt + 54. No rule has to tell which
		 * numbers that holds for: a state knows where it comes from.
		 * If it has no join with that state it is at the head on its own from now on.
		 * The same for bpf_loop() in place of the loop.
		 */
		if (widen && flow_has_room(env, *nr_states) &&
		    flow_entries_differ(&t->st, &s->st, cycle))
			continue;
		if (!widen && flow_has_room(env, *nr_states) &&
		    flow_null_differs(flow, &t->st, &s->st))
			continue;
		if (!widen && flow_needed_consts_differ(flow, FLOW_NEED_ARG, &t->st, &s->st))
			continue;
		if (flow_callback_depth_differs(env, &t->st, &s->st))
			continue;
		/* what grows in a loop is not replaced with what is larger, it is forgotten */
		if (!widen && flow_cmp_pkt_range(s, t) <= 0 && flow_within(env, &s->st, &t->st)) {
			err = flow_unite(flow, s->st.flow_entry, t->st.flow_entry);
			if (err)
				goto err_free;
			/*
			 * What a callback comes back with and covers what is there grew, like
			 * what is joined with it. It counts, or nothing ends "*ctx += 1".
			 */
			s->joins = max(s->joins, t->joins) + (cycle == FLOW_CALLBACK_CYCLE);
			flow_unlink(env, t);
			flow_free_state(env, t);
			what = what ?: "joined";
			continue;
		}
		if (!widen && flow_cmp_pkt_range(s, t))
			continue;
		if (flow_has_room(env, *nr_states) &&
		    (flow_unset_differs(&t->st, &s->st) ||
		     (!widen && flow_join_makes_up(flow, &t->st, &s->st))))
			continue;
		/*
		 * Around a loop too:
		 *	u8 stack[10], depth = 0;
		 *	loop {
		 *		if (...) { if (depth >= 10) break; stack[depth++] = x; }
		 *		else { depth--; if (depth >= 10) break; x = stack[depth]; }
		 *	}
		 * depth is in a 32-bit register. A compiler tests 'depth & 255' and computes it
		 * anew for the index. What is found out about a copy says nothing about
		 * a range, and everything about a constant.
		 */
		if (flow_has_room(env, *nr_states) &&
		    flow_needed_consts_differ(flow, FLOW_NEED_STACK_OFF, &t->st, &s->st))
			continue;
		if (!widen && flow_has_room(env, *nr_states) &&
		    flow_needed_consts_differ(flow, FLOW_NEED_PTR_OFF, &t->st, &s->st))
			continue;
		err = bpf_states_join(env, &flow->tmp, &t->st, &s->st, widen);
		if (err < 0)
			goto err_free;
		if (err)
			continue;
		what = widen ? "widened" : "joined";
		err = flow_unite(flow, s->st.flow_entry, t->st.flow_entry);
		if (err)
			goto err_free;
		flow->tmp.flow_entry = t->st.flow_entry;
		flow->tmp.last_insn_idx = s->prev_insn_idx;
		s->joins = max(s->joins, t->joins) + 1;
		bpf_free_verifier_state(&s->st, false);
		s->st = flow->tmp;
		memset(&flow->tmp, 0, sizeof(flow->tmp));
		flow_unlink(env, t);
		flow_free_state(env, t);
		/* the join may be within, or have a join with, what is left in the set */
		goto again;
	}

	if (*nr_states >= BPF_FLOW_MAX_STATES) {
		verbose(env, "more than %d states that cannot be joined at insn %d\n",
			BPF_FLOW_MAX_STATES, insn_idx);
		err = -E2BIG;
		goto err_free;
	}
	if (flow->live_states > BPF_FLOW_LIVE_PER_INSN * env->prog->len + BPF_FLOW_LIVE_BASE ||
	    env->total_states >
	    BPF_FLOW_TOTAL_PER_INSN * (u64)env->prog->len + BPF_FLOW_TOTAL_BASE) {
		verbose(env, "too many states for %d insns: %d, %d at a time\n",
			env->prog->len, env->total_states, flow->live_states);
		err = -E2BIG;
		goto err_free;
	}
	/*
	 * The state is printed when its turn comes. That says enough about a state that
	 * is new, and about a state that goes on to the next insn.
	 */
	if (prev >= 0 && insn_idx == prev + 1 + bpf_is_ldimm64(&env->prog->insnsi[prev]) &&
	    what && !strcmp(what, "joined"))
		what = NULL;
	if (log && what)
		verbose(env, "from %d to %d: %s\n", prev, insn_idx, what);
	flow_link(env, set, s);
	env->max_states_per_insn = max(env->max_states_per_insn, *nr_states);
	return 0;

err_free:
	flow_free_state(env, s);
	return err;
}

static struct flow_state *flow_new_state(struct bpf_verifier_env *env, int prev_insn_idx)
{
	struct flow_state *s = kzalloc_obj(*s, GFP_KERNEL_ACCOUNT);
	struct bpf_flow *flow = env->flow;

	if (!s)
		return NULL;
	s->prev_insn_idx = prev_insn_idx;
	s->seq = ++flow->seq;
	s->from = flow->from;
	env->total_states++;
	flow->live_states++;
	env->peak_states = max(env->peak_states, flow->live_states);
	return s;
}

/* What @st holds is taken and @st is left empty. What doesn't matter at @insn_idx goes. */
static struct flow_state *flow_arrive(struct bpf_verifier_env *env, struct bpf_verifier_state *st,
				      int insn_idx, int prev_insn_idx, bool *enters)
{
	struct bpf_verifier_state *cur = env->cur_state;
	int cur_idx = env->insn_idx, err;
	struct flow_state *s;

	if (insn_idx < 0 || insn_idx >= env->prog->len) {
		verbose(env, "invalid insn idx %d insn_cnt %d\n", insn_idx, env->prog->len);
		return ERR_PTR(-EFAULT);
	}

	s = flow_new_state(env, prev_insn_idx);
	if (!s)
		return ERR_PTR(-ENOMEM);
	s->st = *st;
	memset(st, 0, sizeof(*st));
	s->st.insn_idx = insn_idx;
	s->st.last_insn_idx = prev_insn_idx;

	env->cur_state = &s->st;
	env->insn_idx = insn_idx;
	err = bpf_flow_loop_leave(env, enters);
	if (!err)
		err = bpf_clean_verifier_state(env, &s->st);
	env->cur_state = cur;
	env->insn_idx = cur_idx;
	if (err) {
		flow_free_state(env, s);
		return ERR_PTR(err);
	}
	/* when the registers that it was shared with are dead, or are those of the caller */
	bpf_clear_singular_ids(env, &s->st);
	return s;
}

/* Is that where may_goto in front of @insn_idx goes, as far as it's known? */
static bool flow_guard_edge_is_real(struct bpf_verifier_env *env, int insn_idx, u8 edge)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;

	return aux[bpf_loop_at_index(env, insn_idx)].loop->guarded &&
	       aux[insn_idx].guard_no_unwind == (edge == BPF_GUARD_LEAVES_LOOP);
}

/*
 * @st gets to @insn_idx if the loop has may_goto in front of @prev_insn_idx and that is where it
 * goes, see check_loop_guard(). If that is not so, or not known yet, @st is dropped. If it turns
 * out to be so later all is done again.
 */
int bpf_flow_guard_edge(struct bpf_verifier_env *env, struct bpf_verifier_state *st,
			int insn_idx, int prev_insn_idx, enum bpf_guard_edge edge)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;

	if (flow_guard_edge_is_real(env, prev_insn_idx, edge)) {
		aux[prev_insn_idx].loop_guard = true;
		return bpf_flow_deliver(env, st, insn_idx, prev_insn_idx);
	}
	/* it's not going to leave the subprog any more */
	if (edge == BPF_GUARD_LEAVES_SUBPROG && aux[prev_insn_idx].guard_no_unwind)
		return 0;
	aux[prev_insn_idx].guard_edge_dropped = true;
	aux[bpf_loop_at_index(env, prev_insn_idx)].loop->no_guard_edges = true;
	return 0;
}

/*
 * @s came back to the head of a loop, and the walk through the iterations ends, see
 * bpf_flow_loop_enter(). From now on a state stands for many iterations. The first such state is
 * the join of @s with the state that it comes from, one iteration ago. Both were seen, and what
 * differs between them is what the loop changes. If they have no join @s goes on as it is.
 * A state of another walk, which began with another state, has nothing to do with @s.
 */
static int flow_end_walk(struct bpf_verifier_env *env, struct flow_set *set, struct flow_state *s)
{
	struct bpf_func_state *frame = s->st.frame[s->st.curframe];
	struct loop_stack_entry *entry = &frame->loop_stack[frame->loop_stack_cnt - 1];
	u32 top = frame->loop_stack_cnt - 1, walk = entry->walk_id;
	struct bpf_flow *flow = env->flow;
	int insn_idx = s->st.insn_idx, err;
	struct flow_state *t;

	list_for_each_entry_reverse(t, &set->list, node) {
		/*
		 * Some rules of flow_merge() hold here too. One of them: the iteration has set
		 * a register, see flow_unset_differs(). The join would be without it.
		 */
		if (flow_trips_behind(&t->st, &s->st) ||
		    t->st.frame[t->st.curframe]->loop_stack[top].walk_id != walk ||
		    (flow_has_room(env, flow->nr_states[insn_idx]) &&
		     (flow_unset_differs(&t->st, &s->st) ||
		      flow_needed_consts_differ(flow, FLOW_NEED_STACK_OFF, &t->st, &s->st) ||
		      flow_needed_consts_differ(flow, FLOW_NEED_PTR_OFF, &t->st, &s->st))))
			continue;
		err = bpf_states_join(env, &flow->tmp, &t->st, &s->st, true);
		if (err < 0)
			return err;
		if (err)
			continue;
		err = flow_unite(flow, s->st.flow_entry, t->st.flow_entry);
		if (err)
			return err;
		flow->tmp.flow_entry = t->st.flow_entry;
		flow->tmp.last_insn_idx = s->prev_insn_idx;
		bpf_free_verifier_state(&s->st, false);
		s->st = flow->tmp;
		memset(&flow->tmp, 0, sizeof(flow->tmp));
		if (env->log.level & BPF_LOG_LEVEL2)
			verbose(env, "from %d to %d: widened\n", s->prev_insn_idx, insn_idx);
		break;
	}

	frame = s->st.frame[s->st.curframe];
	entry = &frame->loop_stack[frame->loop_stack_cnt - 1];
	entry->guarded = env->insn_aux_data[insn_idx].loop->guarded;
	entry->apart = false;
	entry->probe = false;
	entry->decided = false;
	entry->terminates = false;
	entry->trip++;
	entry->max_header_count = 0;
	return 0;
}

/*
 * @s goes into the set of the states at its insn. It is taken.
 * @enters: it enters a loop.
 */
static int flow_add(struct bpf_verifier_env *env, struct flow_state *s, enum flow_cycle cycle,
		    bool enters)
{
	struct bpf_verifier_state *cur = env->cur_state;
	int cur_idx = env->insn_idx, insn_idx = s->st.insn_idx, err;
	const struct loop_stack_entry *entry;
	const struct bpf_func_state *frame;
	bool backedge = false, drop = false, ends_walk = false;
	struct bpf_flow *flow = env->flow;

	env->cur_state = &s->st;
	env->insn_idx = insn_idx;
	err = bpf_flow_loop_enter(env, &backedge, &drop, &ends_walk);
	env->cur_state = cur;
	env->insn_idx = cur_idx;
	if (!err && ends_walk)
		err = flow_end_walk(env, &flow->states[insn_idx], s);
	if (!err && enters) {
		frame = s->st.frame[s->st.curframe];
		entry = &frame->loop_stack[frame->loop_stack_cnt - 1];
		/* it has a count, see bpf_flow_loop_gets_guards() */
		if (entry->terminates && !entry->guarded && !bpf_loop_walk_may_give_up(entry))
			env->insn_aux_data[insn_idx].loop->no_guard_edges = true;
	}
	if (err || drop) {
		flow_free_state(env, s);
		return err;
	}

	err = flow_merge(env, &flow->states[insn_idx], &s, backedge ? FLOW_CYCLE : cycle);
	if (err)
		return err < 0 ? err : 0;
	flow_set_pending(flow, s);
	return 0;
}

/*
 * @st gets to @insn_idx. What it holds is taken and @st is left empty.
 */
int bpf_flow_deliver(struct bpf_verifier_env *env, struct bpf_verifier_state *st,
		     int insn_idx, int prev_insn_idx)
{
	struct bpf_flow *flow = env->flow;
	struct bpf_subprog_info *sub;
	struct flow_state *s, *e;
	enum flow_cycle cycle;
	bool enters = false;
	int err;

	s = flow_arrive(env, st, insn_idx, prev_insn_idx, &enters);
	if (IS_ERR(s))
		return PTR_ERR(s);

	/*
	 * Cycles that have no loop head: the callback that came back to the helper
	 * that calls it, and calls that go through the entry of a function.
	 */
	sub = bpf_find_containing_subprog(env, insn_idx);
	cycle = FLOW_NO_CYCLE;
	if (sub->recursive && sub->start == insn_idx)
		cycle = FLOW_CYCLE;
	else if (env->insn_aux_data[insn_idx].calls_callback && prev_insn_idx >= 0 &&
		 env->prog->insnsi[prev_insn_idx].code == (BPF_JMP | BPF_EXIT))
		cycle = FLOW_CALLBACK_CYCLE;
	else if (env->insn_aux_data[insn_idx].calls_callback)
		s->st.frame[s->st.curframe]->callback_entry_id = ++env->loop_entry_cnt;

	/*
	 * What is known about a loop is derived from the state that enters it. So
	 * the states that enter are a set of their own, and what gets to
	 * the set at the head of the loop is derived from what is new in it.
	 */
	if (enters) {
		err = flow_merge(env, &flow->enter[insn_idx], &s, cycle);
		if (err)
			return err < 0 ? err : 0;
		cycle = FLOW_NO_CYCLE;
		e = s;
		s = flow_new_state(env, prev_insn_idx);
		if (!s)
			return -ENOMEM;
		err = bpf_copy_verifier_state(&s->st, &e->st);
		if (err) {
			flow_free_state(env, s);
			return err;
		}
	}
	return flow_add(env, s, cycle, enters);
}

static void flow_free_call(struct flow_state *w)
{
	list_del(&w->node);
	bpf_free_verifier_state(&w->st, false);
	kfree(w);
}

/* @x came out of the function that the call @w waits for. @w may be gone afterwards. */
static int flow_apply_exit(struct bpf_verifier_env *env, struct flow_state *w,
			   struct flow_exit *x)
{
	int call_idx = w->st.insn_idx, err;
	struct bpf_verifier_state st = {};
	struct flow_state *s;

	err = bpf_copy_verifier_state(&st, &w->st);
	if (!err)
		err = bpf_flow_return(env, &st, &x->st, call_idx);
	if (!err)
		err = bpf_flow_deliver(env, &st, call_idx + 1, call_idx);
	bpf_free_verifier_state(&st, false);
	if (err != 1)
		return err;

	/*
	 * The call is verified in the context of the caller: it doesn't wait any more, it is
	 * done again. Not with bpf_flow_deliver(): the state was at the insn before, it may
	 * be there still.
	 */
	s = flow_new_state(env, w->st.last_insn_idx);
	if (!s)
		return -ENOMEM;
	s->st = w->st;
	memset(&w->st, 0, sizeof(w->st));
	flow_free_call(w);
	flow_shape_ctx(&s->st, &s->shape);
	flow_link(env, &env->flow->states[call_idx], s);
	flow_set_pending(env->flow, s);
	return 0;
}

/* What came out for the set of @from goes to the calls that wait for the set of @to */
static int flow_apply_exits(struct bpf_verifier_env *env, struct flow_entry *from,
			    struct flow_entry *to)
{
	struct flow_state *w, *wn;
	struct flow_exit *x;
	int err;

	list_for_each_entry(x, &from->exits, node) {
		list_for_each_entry_safe(w, wn, &to->calls, node) {
			err = flow_apply_exit(env, w, x);
			if (err)
				return err;
		}
	}
	return 0;
}

static int flow_do_united(struct bpf_verifier_env *env)
{
	struct bpf_flow *flow = env->flow;
	struct flow_entry *a, *b;
	struct flow_united u;
	int err;

	while (flow->united_cnt) {
		u = flow->united[--flow->united_cnt];
		a = flow_entry_find(flow, u.a);
		b = flow_entry_find(flow, u.b);
		if (a == b)
			continue;
		err = flow_apply_exits(env, a, b) ?: flow_apply_exits(env, b, a);
		if (err)
			return err;
		list_splice_tail_init(&a->calls, &b->calls);
		list_splice_tail_init(&a->exits, &b->exits);
		a->up = b;
	}
	return 0;
}

/*
 * The call at @call_idx in @caller of a function that is verified on its own,
 * @entry is the state at its first insn. Both are taken.
 */
int bpf_flow_call(struct bpf_verifier_env *env, struct bpf_verifier_state *caller,
		  struct bpf_verifier_state *entry, int subprog, int call_idx)
{
	struct bpf_flow *flow = env->flow;
	struct flow_state *w;
	int id;

	w = kzalloc_obj(*w, GFP_KERNEL_ACCOUNT);
	if (!w)
		return -ENOMEM;
	w->st = *caller;
	memset(caller, 0, sizeof(*caller));
	w->st.insn_idx = call_idx;

	id = flow_entry_new(flow);
	if (id < 0) {
		bpf_free_verifier_state(&w->st, false);
		kfree(w);
		return id;
	}
	list_add_tail(&w->node, &flow->entries[id]->calls);

	entry->flow_entry = id;
	if (env->log.level & BPF_LOG_LEVEL2)
		verbose(env, "call at %d waits for func#%d\n", call_idx, subprog);
	return bpf_flow_deliver(env, entry, env->subprog_info[subprog].start, call_idx);
}

/* @st came to exit of the function that is verified on its own. It is taken. */
int bpf_flow_exit(struct bpf_verifier_env *env, struct bpf_verifier_state *st, int exit_idx)
{
	struct bpf_flow *flow = env->flow;
	struct flow_func *f = &flow->funcs[st->frame[0]->subprogno];
	u32 id = st->flow_entry, cnt = 0;
	struct flow_exit *x, *o, *on;
	struct flow_state *w, *wn;
	struct flow_entry *entry;
	u16 joins = 0;
	int err;

	st->insn_idx = exit_idx;
	err = bpf_clean_verifier_state(env, st);
	if (err)
		return err;
	if (f->exit_idx < 0)
		f->exit_idx = exit_idx;
	st->insn_idx = f->exit_idx;
	entry = flow_entry_find(flow, id);

	list_for_each_entry_safe(o, on, &entry->exits, node) {
		if (bpf_state_subsumes(env, &o->st, st))
			return 0;
		if (flow_has_room(env, ++cnt) && flow_join_makes_up(flow, &o->st, st))
			continue;
		/* what comes out of a recursive function gets into it again */
		err = bpf_states_join(env, &flow->tmp, &o->st, st,
				      env->subprog_info[st->frame[0]->subprogno].recursive ||
				      o->joins > BPF_FLOW_MAX_JOINS);
		if (err < 0)
			return err;
		if (err)
			continue;
		joins = max(joins, o->joins) + 1;
		bpf_free_verifier_state(st, false);
		*st = flow->tmp;
		memset(&flow->tmp, 0, sizeof(flow->tmp));
		st->flow_entry = id;
		list_del(&o->node);
		bpf_free_verifier_state(&o->st, false);
		kfree(o);
	}

	x = kzalloc_obj(*x, GFP_KERNEL_ACCOUNT);
	if (!x)
		return -ENOMEM;
	x->st = *st;
	memset(st, 0, sizeof(*st));
	x->joins = joins;
	list_add_tail(&x->node, &entry->exits);

	list_for_each_entry_safe(w, wn, &entry->calls, node) {
		err = flow_apply_exit(env, w, x);
		if (err)
			return err;
	}
	return 0;
}

/*
 * Make the next pending state the current one. env->cur_state has to be empty.
 * Returns -ENOENT when nothing is pending.
 */
int bpf_flow_next(struct bpf_verifier_env *env, int *prev_insn_idx)
{
	struct bpf_flow *flow = env->flow;
	struct flow_state *s;
	struct rb_node *n;
	int err;

	do {
		err = flow_do_united(env);
		if (err)
			return err;
	} while (flow->united_cnt);
	n = rb_first_cached(&flow->pending);
	if (!n)
		return -ENOENT;
	s = rb_entry(n, struct flow_state, rb);
	rb_erase_cached(&s->rb, &flow->pending);
	s->pending = false;

	env->insn_idx = s->st.insn_idx;
	*prev_insn_idx = s->prev_insn_idx;
	if (test_bit(s->st.insn_idx, flow->keep)) {
		flow->from = s->seq;
		return bpf_copy_verifier_state(env->cur_state, &s->st);
	}
	flow->from = s->from;
	*env->cur_state = s->st;
	memset(&s->st, 0, sizeof(s->st));
	s->st.insn_idx = env->insn_idx;
	flow_unlink(env, s);
	flow_free_state(env, s);
	return 0;
}

/* A basic block on the way to the block that is rejected, up to the insn that leads on */
struct flow_step {
	const struct flow_state *s;	/* the state at its head, if one is kept */
	int head, last;
};

/*
 * The state at @head with that @seq. If it is gone a state that came later took its place:
 * the last one then.
 */
static const struct flow_state *flow_find_state(const struct bpf_flow *flow, int head, u32 seq)
{
	const struct list_head *list = &flow->states[head].list;
	const struct flow_state *s;

	if (!test_bit(head, flow->keep))
		return NULL;
	list_for_each_entry(s, list, node)
		if (s->seq == seq)
			return s;
	return list_empty(list) ? NULL : list_last_entry(list, struct flow_state, node);
}

/*
 * The basic block at @head, which the current state got to from insn @i, is rejected. The block
 * and the message are what was printed since @log_pos. Print the way to the block in front of it.
 *
 * The way is put together backwards from what is there. A state that is kept knows the edge that
 * it came by, after a join the edge that changed it last, and the kept state that it comes from.
 * A block whose state is not kept has one predecessor, which is its dominator.
 * A block is on the way once. Where it would come again, after a round of a loop, the way goes on
 * from its dominator: all that gets into the loop goes through it.
 */
void bpf_flow_print_way(struct bpf_verifier_env *env, u64 log_pos, int head, int i)
{
	struct bpf_insn *insns = env->prog->insnsi;
	const struct bpf_flow *flow = env->flow;
	const struct flow_state *s;
	struct bpf_vlog_stash stash;
	struct flow_step *way, *w;
	u32 from = flow->from, cnt = 0;
	unsigned long *seen;

	seen = bitmap_zalloc(flow->len, GFP_KERNEL_ACCOUNT);
	way = kvmalloc_array(bitmap_weight(flow->point, flow->len), sizeof(*way),
			     GFP_KERNEL_ACCOUNT);
	if (!seen || !way)
		goto out;

	s = flow_find_state(flow, head, from);
	from = s ? s->from : from;
	while (i >= 0) {
		head = find_last_bit(flow->point, i + 1);
		s = NULL;
		if (!__test_and_set_bit(head, seen)) {
			s = flow_find_state(flow, head, from);
			way[cnt++] = (struct flow_step){ s, head, i };
		}
		from = s ? s->from : from;
		i = s ? s->prev_insn_idx : env->idoms[head];
		cond_resched();
	}

	if (!cnt || bpf_vlog_stash(&env->log, log_pos, &stash))
		goto out;
	env->prev_linfo = NULL;
	while (cnt-- && !signal_pending(current)) {
		w = &way[cnt];
		if (w->s) {
			verbose(env, "from %d to %d:", w->s->prev_insn_idx, w->head);
			print_verifier_state(env, &w->s->st, w->s->st.curframe, true);
		}
		for (i = w->head; i <= w->last; i += 1 + bpf_is_ldimm64(&insns[i])) {
			verbose_linfo(env, i, "; ");
			verbose(env, "%d: ", i);
			bpf_verbose_insn(env, &insns[i]);
			verbose(env, "\n");
		}
		cond_resched();
	}
	bpf_vlog_unstash(&env->log, &stash);
out:
	bitmap_free(seen);
	kvfree(way);
}
