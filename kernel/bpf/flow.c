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
	bool lost = false;
	int h;

	for (h = 0; h < env->prog->len; h++) {
		loop = env->insn_aux_data[h].loop;
		if (!loop || !loop->rt_bounded || flow_loop_is_bounded(env, h))
			continue;
		loop->rt_bounded = false;
		lost = true;
	}
	return lost;
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
		if (aux[i].loop)
			aux[i].loop->no_guard_edges = false;
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
