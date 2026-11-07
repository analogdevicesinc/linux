// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <linux/bpf_verifier.h>

/*
 * Forward dataflow analysis to determine constant register values at every
 * instruction. Tracks 64-bit constant values in R0-R9 through the program,
 * using a fixed-point iteration in reverse postorder. Records which registers
 * hold known constants and their values in
 * env->insn_aux_data[].{const_reg_mask, const_reg_wide_mask, const_reg_vals}.
 */

enum const_arg_state {
	CONST_ARG_UNVISITED,	/* instruction not yet reached */
	CONST_ARG_UNKNOWN,	/* register value not a known constant */
	CONST_ARG_CONST,	/* register holds a known 64-bit constant */
	CONST_ARG_MAP_PTR,	/* register holds a map pointer, map_index is set */
	CONST_ARG_MAP_VALUE,	/* register points to map value data, val is offset */
	CONST_ARG_SUBPROG,	/* register holds a subprog pointer, val is subprog number */
};

struct const_arg_info {
	enum const_arg_state state;
	u32 map_index;
	u64 val;
};

static bool ci_is_unvisited(const struct const_arg_info *ci)
{
	return ci->state == CONST_ARG_UNVISITED;
}

static bool ci_is_unknown(const struct const_arg_info *ci)
{
	return ci->state == CONST_ARG_UNKNOWN;
}

static bool ci_is_const(const struct const_arg_info *ci)
{
	return ci->state == CONST_ARG_CONST;
}

static bool ci_is_map_value(const struct const_arg_info *ci)
{
	return ci->state == CONST_ARG_MAP_VALUE;
}

/* Transfer function: compute output register state from instruction. */
static void const_reg_xfer(struct bpf_verifier_env *env, struct const_arg_info *ci_out,
			   struct bpf_insn *insn, struct bpf_insn *insns, int idx)
{
	struct const_arg_info unknown = { .state = CONST_ARG_UNKNOWN, .val = 0 };
	struct const_arg_info *dst = &ci_out[insn->dst_reg];
	struct const_arg_info *src = &ci_out[insn->src_reg];
	u8 class = BPF_CLASS(insn->code);
	u8 mode = BPF_MODE(insn->code);
	u8 opcode = BPF_OP(insn->code) | BPF_SRC(insn->code);
	int r;

	/* Stack arg stores (r11-based) are outside the tracked register set. */
	if (is_stack_arg_st(insn) || is_stack_arg_stx(insn))
		return;
	if (is_stack_arg_ldx(insn)) {
		ci_out[insn->dst_reg] = unknown;
		return;
	}
	/* r12 is outside of it too */
	if (is_arena_sp_adj(insn) || is_arena_sp_st(insn))
		return;
	if (is_arena_sp_mov(insn) || is_arena_sp_ldx(insn)) {
		ci_out[insn->dst_reg] = unknown;
		return;
	}

	switch (class) {
	case BPF_ALU:
	case BPF_ALU64:
		switch (opcode) {
		case BPF_MOV | BPF_K:
			dst->state = CONST_ARG_CONST;
			dst->val = (s64)insn->imm;
			break;
		case BPF_MOV | BPF_X:
			*dst = *src;
			if (!insn->off)
				break;
			if (!ci_is_const(dst)) {
				*dst = unknown;
				break;
			}
			switch (insn->off) {
			case 8:  dst->val = (s8)dst->val; break;
			case 16: dst->val = (s16)dst->val; break;
			case 32: dst->val = (s32)dst->val; break;
			default: *dst = unknown; break;
			}
			break;
		case BPF_ADD | BPF_K:
			if (!ci_is_const(dst) && !ci_is_map_value(dst)) {
				*dst = unknown;
				break;
			}
			dst->val += insn->imm;
			break;
		case BPF_SUB | BPF_K:
			if (!ci_is_const(dst) && !ci_is_map_value(dst)) {
				*dst = unknown;
				break;
			}
			dst->val -= insn->imm;
			break;
		case BPF_AND | BPF_K:
			if (!ci_is_const(dst)) {
				if (!insn->imm) {
					dst->state = CONST_ARG_CONST;
					dst->val = 0;
				} else {
					*dst = unknown;
				}
				break;
			}
			dst->val &= (s64)insn->imm;
			break;
		case BPF_AND | BPF_X:
			if (ci_is_const(dst) && dst->val == 0)
				break; /* 0 & x == 0 */
			if (ci_is_const(src) && src->val == 0) {
				dst->state = CONST_ARG_CONST;
				dst->val = 0;
				break;
			}
			if (!ci_is_const(dst) || !ci_is_const(src)) {
				*dst = unknown;
				break;
			}
			dst->val &= src->val;
			break;
		default:
			*dst = unknown;
			break;
		}
		if (class == BPF_ALU) {
			if (ci_is_const(dst))
				dst->val = (u32)dst->val;
			else if (!ci_is_unknown(dst))
				*dst = unknown;
		}
		break;
	case BPF_LD:
		if (mode == BPF_ABS || mode == BPF_IND)
			goto process_call;
		if (mode != BPF_IMM || BPF_SIZE(insn->code) != BPF_DW)
			break;
		if (insn->src_reg == BPF_PSEUDO_FUNC) {
			int subprog = bpf_find_subprog(env, idx + insn->imm + 1);

			if (subprog >= 0) {
				dst->state = CONST_ARG_SUBPROG;
				dst->val = subprog;
			} else {
				*dst = unknown;
			}
		} else if (insn->src_reg == BPF_PSEUDO_MAP_VALUE ||
			   insn->src_reg == BPF_PSEUDO_MAP_IDX_VALUE) {
			dst->state = CONST_ARG_MAP_VALUE;
			dst->map_index = env->insn_aux_data[idx].map_index;
			dst->val = env->insn_aux_data[idx].map_off;
		} else if (insn->src_reg == BPF_PSEUDO_MAP_FD ||
			   insn->src_reg == BPF_PSEUDO_MAP_IDX) {
			dst->state = CONST_ARG_MAP_PTR;
			dst->map_index = env->insn_aux_data[idx].map_index;
		} else if (insn->src_reg == 0) {
			dst->state = CONST_ARG_CONST;
			dst->val = (u64)(u32)insn->imm | ((u64)(u32)insns[idx + 1].imm << 32);
		} else {
			*dst = unknown;
		}
		break;
	case BPF_LDX:
		if (!ci_is_map_value(src)) {
			*dst = unknown;
			break;
		}
		struct bpf_map *map = env->used_maps[src->map_index];
		int size = bpf_size_to_bytes(BPF_SIZE(insn->code));
		bool is_ldsx = mode == BPF_MEMSX;
		int off = src->val + insn->off;
		u64 val = 0;
		u32 cnt;

		if (!bpf_map_is_rdonly(map) || !map->ops->map_direct_value_addr ||
		    off < 0 || off + size > map->value_size ||
		    /* so are the addresses of functions that the map points to */
		    bpf_map_range_func_ptrs(env, map, off, size, &cnt) ||
		    bpf_map_direct_read(map, off, size, &val, is_ldsx)) {
			*dst = unknown;
			break;
		}
		dst->state = CONST_ARG_CONST;
		dst->val = val;
		break;
	case BPF_JMP:
		/* both 'call imm' and 'callx reg' clobber caller saved registers */
		if (BPF_OP(insn->code) != BPF_CALL)
			break;
process_call:
		for (r = BPF_REG_0; r <= BPF_REG_5; r++)
			ci_out[r] = unknown;
		break;
	case BPF_STX:
		r = bpf_atomic_load_reg(insn);
		if (r >= 0)
			ci_out[r] = unknown;
		break;
	}
}

/* Join function: merge output state into a successor's input state. */
static bool const_reg_join(struct const_arg_info *ci_target,
			   struct const_arg_info *ci_out)
{
	bool changed = false;
	int r;

	for (r = 0; r < MAX_BPF_REG; r++) {
		struct const_arg_info *old = &ci_target[r];
		struct const_arg_info *new = &ci_out[r];

		if (ci_is_unvisited(old) && !ci_is_unvisited(new)) {
			ci_target[r] = *new;
			changed = true;
		} else if (!ci_is_unknown(old) && !ci_is_unvisited(old) &&
			   (new->state != old->state || new->val != old->val ||
			    new->map_index != old->map_index)) {
			old->state = CONST_ARG_UNKNOWN;
			changed = true;
		}
	}
	return changed;
}

int bpf_compute_const_regs(struct bpf_verifier_env *env)
{
	struct const_arg_info unknown = { .state = CONST_ARG_UNKNOWN, .val = 0 };
	struct bpf_insn_aux_data *insn_aux = env->insn_aux_data;
	struct bpf_insn *insns = env->prog->insnsi;
	int insn_cnt = env->prog->len;
	struct const_arg_info (*ci_in)[MAX_BPF_REG];
	struct const_arg_info ci_out[MAX_BPF_REG];
	struct bpf_iarray *succ;
	bool changed;
	int i, r, t;

	/* kvzalloc zeroes memory, so all entries start as CONST_ARG_UNVISITED (0) */
	ci_in = kvzalloc_objs(*ci_in, insn_cnt, GFP_KERNEL_ACCOUNT);
	if (!ci_in)
		return -ENOMEM;

	/* Subprogram entries (including main at subprog 0): all registers unknown */
	for (i = 0; i < env->subprog_cnt; i++) {
		int start = env->subprog_info[i].start;

		for (r = 0; r < MAX_BPF_REG; r++)
			ci_in[start][r] = unknown;
		/*
		 * The subprog gets there from where it is out of budget too, see
		 * push_budget_exit(). That is not an edge of the graph.
		 */
		for (r = 0; r < MAX_BPF_REG && env->subprog_info[i].bail_insn >= 0; r++)
			ci_in[env->subprog_info[i].bail_insn][r] = unknown;
	}
	/* The same from the head of a loop to where the loop is left, once the loops are known. */
	for (i = 0; i < insn_cnt; i++) {
		t = bpf_head_guard_target(env, i);
		for (r = 0; r < MAX_BPF_REG && t >= 0; r++)
			ci_in[t][r] = unknown;
	}

redo:
	changed = false;
	for (i = env->cfg.cur_postorder - 1; i >= 0; i--) {
		int idx = env->cfg.insn_postorder[i];
		struct bpf_insn *insn = &insns[idx];
		struct const_arg_info *ci = ci_in[idx];

		memcpy(ci_out, ci, sizeof(ci_out));

		const_reg_xfer(env, ci_out, insn, insns, idx);

		succ = bpf_insn_successors(env, idx);
		for (int s = 0; s < succ->cnt; s++)
			changed |= const_reg_join(ci_in[succ->items[s]], ci_out);
	}
	if (changed)
		goto redo;

	/* Save computed constants into insn_aux[]: the value if it fits into 32-bit */
	for (i = 0; i < insn_cnt; i++) {
		u16 mask = 0, wide_mask = 0, map_mask = 0, subprog_mask = 0;
		struct bpf_insn_aux_data *aux = &insn_aux[i];
		struct const_arg_info *ci = ci_in[i];

		for (r = BPF_REG_0; r < ARRAY_SIZE(aux->const_reg_vals); r++) {
			struct const_arg_info *c = &ci[r];

			switch (c->state) {
			case CONST_ARG_CONST: {
				u64 val = c->val;

				if (val != (u32)val) {
					wide_mask |= BIT(r);
					break;
				}
				mask |= BIT(r);
				aux->const_reg_vals[r] = val;
				break;
			}
			case CONST_ARG_MAP_PTR:
				map_mask |= BIT(r);
				aux->const_reg_vals[r] = c->map_index;
				break;
			case CONST_ARG_SUBPROG:
				subprog_mask |= BIT(r);
				aux->const_reg_vals[r] = c->val;
				break;
			default:
				break;
			}
		}
		aux->const_reg_mask = mask;
		aux->const_reg_wide_mask = wide_mask;
		aux->const_reg_map_mask = map_mask;
		aux->const_reg_subprog_mask = subprog_mask;
	}

	kvfree(ci_in);
	return 0;
}

static int eval_const_branch(u8 opcode, u64 dst_val, u64 src_val)
{
	switch (BPF_OP(opcode)) {
	case BPF_JEQ:	return dst_val == src_val;
	case BPF_JNE:	return dst_val != src_val;
	case BPF_JGT:	return dst_val > src_val;
	case BPF_JGE:	return dst_val >= src_val;
	case BPF_JLT:	return dst_val < src_val;
	case BPF_JLE:	return dst_val <= src_val;
	case BPF_JSGT:	return (s64)dst_val > (s64)src_val;
	case BPF_JSGE:	return (s64)dst_val >= (s64)src_val;
	case BPF_JSLT:	return (s64)dst_val < (s64)src_val;
	case BPF_JSLE:	return (s64)dst_val <= (s64)src_val;
	case BPF_JSET:	return (bool)(dst_val & src_val);
	default:	return -1;
	}
}

/* 1: the registers @ci take the conditional jump @insn, 0: they don't, -1: not known */
static int const_branch_taken(const struct bpf_insn *insn, const struct const_arg_info *ci)
{
	u64 dst = ci[insn->dst_reg].val, src = insn->imm;

	if (!ci_is_const(&ci[insn->dst_reg]))
		return -1;
	if (BPF_SRC(insn->code) == BPF_X) {
		if (!ci_is_const(&ci[insn->src_reg]))
			return -1;
		src = ci[insn->src_reg].val;
	}
	/* see bpf_prune_dead_branches() */
	if (BPF_CLASS(insn->code) == BPF_JMP32) {
		dst = (s32)dst;
		src = (s32)src;
	}
	return eval_const_branch(insn->code, dst, src);
}

/*
 * When is the program that stands in front of an insn of a loop sure to leave the loop?
 * Always, or when one register holds a number of a set, whatever the other registers hold.
 * A set has one bit per range of numbers of 32 bits. A flag is a small number: each of 0 - 31
 * is a range. An error code or a size is not: 32 - 63, 64 - 127, ... are the other ranges.
 */
struct bpf_leave_cond {
	u64 vals[MAX_BPF_REG];
	bool always;
};

#define LEAVE_SMALL	32
#define LEAVE_RANGES	(LEAVE_SMALL + 32 - ilog2(LEAVE_SMALL))

static const struct bpf_leave_cond leave_never;
static const struct bpf_leave_cond leave_always = {
	.vals = { [0 ... MAX_BPF_REG - 1] = U64_MAX },
	.always = true,
};

static u64 leave_val_bit(u64 val)
{
	if (val > U32_MAX)
		return 0;
	return BIT_ULL(val < LEAVE_SMALL ? val : ilog2(val) + LEAVE_SMALL - ilog2(LEAVE_SMALL));
}

/* The condition for the loop at @h in front of @t, where an insn of that loop goes on */
static const struct bpf_leave_cond *leave_cond_at(struct bpf_verifier_env *env, int h, int t)
{
	int l = bpf_loop_at_index(env, t);

	if (l != h)
		return bpf_is_nested_loop(env, l, h) ? &leave_never : &leave_always;
	return t == h || !env->leave_conds ? &leave_never : &env->leave_conds[t];
}

/* The same when the jump at @n goes on at @t: always, if the constants at @n meet it */
static const struct bpf_leave_cond *leave_cond_edge(struct bpf_verifier_env *env, int h, int n,
						    int t)
{
	const struct bpf_insn_aux_data *aux = &env->insn_aux_data[n];
	const struct bpf_leave_cond *p = leave_cond_at(env, h, t);
	int r;

	for (r = 0; r < ARRAY_SIZE(aux->const_reg_vals); r++)
		if ((aux->const_reg_mask & BIT(r)) &&
		    (p->vals[r] & leave_val_bit(aux->const_reg_vals[r])))
			return &leave_always;
	return p;
}

/* Is what the insn at @idx makes of @reg == @val in the set @vals? @ci: nothing is known. */
static bool insn_makes(struct bpf_verifier_env *env, struct const_arg_info *ci, int idx,
		       int reg, u64 val, u64 vals)
{
	struct const_arg_info unknown = { .state = CONST_ARG_UNKNOWN, .val = 0 };
	struct bpf_insn *insns = env->prog->insnsi;
	struct const_arg_info *dst = &ci[insns[idx].dst_reg];
	bool ret;

	if (reg >= 0) {
		ci[reg].state = CONST_ARG_CONST;
		ci[reg].val = val;
	}
	const_reg_xfer(env, ci, &insns[idx], insns, idx);
	ret = ci_is_const(dst) && (vals & leave_val_bit(dst->val));
	*dst = unknown;
	if (reg >= 0)
		ci[reg] = unknown;
	return ret;
}

/* @p holds behind the insn at @idx. Make it what holds in front of it. */
static void leave_cond_insn(struct bpf_verifier_env *env, int idx, struct bpf_leave_cond *p)
{
	struct bpf_insn *insns = env->prog->insnsi, *insn = &insns[idx];
	int r, v, dst = insn->dst_reg, src = insn->src_reg;
	struct const_arg_info ci[MAX_BPF_REG] = {};
	u8 class = BPF_CLASS(insn->code);
	u64 vals;

	/* Nothing is known about a register that is no constant although all were constants */
	for (r = 0; r < MAX_BPF_REG; r++)
		ci[r].state = CONST_ARG_CONST;
	const_reg_xfer(env, ci, insn, insns, idx);
	for (r = 0; r < MAX_BPF_REG; r++) {
		if (!ci_is_const(&ci[r]))
			p->vals[r] = 0;
		ci[r].state = CONST_ARG_UNKNOWN;
	}
	if ((class != BPF_ALU && class != BPF_ALU64 && class != BPF_LD) ||
	    dst >= MAX_BPF_REG || !p->vals[dst])
		return;
	vals = p->vals[dst];
	p->vals[dst] = 0;
	if (insn_makes(env, ci, idx, -1, 0, vals)) {
		*p = leave_always;
		return;
	}
	for (v = 0; v < LEAVE_SMALL && class != BPF_LD; v++) {
		if (insn_makes(env, ci, idx, dst, v, vals))
			p->vals[dst] |= BIT_ULL(v);
		if (BPF_SRC(insn->code) == BPF_X && src != dst && src < MAX_BPF_REG &&
		    insn_makes(env, ci, idx, src, v, vals))
			p->vals[src] |= BIT_ULL(v);
	}
}

/*
 * const_branch_taken() for all of @lo - @hi in the register that @insn compares with a number.
 * The range has numbers of 32 bits of one sign. So '<' and the like treat all of them alike
 * if they treat both ends alike. '==' and '!=' single out the number that they compare with.
 * '&': @lo is the bit that all of them have, @hi has the bits that any of them has.
 */
static int range_branch_taken(const struct bpf_insn *insn, struct const_arg_info *ci, u64 lo,
			      u64 hi)
{
	u64 vals[] = { lo, hi, clamp_t(u64, (u32)insn->imm, lo, hi) };
	int i, taken[ARRAY_SIZE(vals)];

	for (i = 0; i < ARRAY_SIZE(vals); i++) {
		ci[insn->dst_reg].val = vals[i];
		taken[i] = const_branch_taken(insn, ci);
	}
	return taken[0] == taken[1] && taken[0] == taken[2] ? taken[0] : -1;
}

/*
 * @p is what is enough on both edges of the jump at @idx. If the jump compares a register
 * with a number then a number in the register goes one way: enough on that edge is enough.
 */
static void leave_cond_jump(struct bpf_verifier_env *env, int h, int idx,
			    struct bpf_leave_cond *p)
{
	struct bpf_insn *insn = &env->prog->insnsi[idx];
	struct const_arg_info ci[MAX_BPF_REG] = {};
	u64 t, f, lo, go = 0, stay = 0;
	int i, taken;

	if (BPF_SRC(insn->code) != BPF_K)
		return;
	t = leave_cond_edge(env, h, idx, idx + insn->off + 1)->vals[insn->dst_reg];
	f = leave_cond_edge(env, h, idx, idx + 1)->vals[insn->dst_reg];
	if (t == f)
		return;
	ci[insn->dst_reg].state = CONST_ARG_CONST;
	for (i = 0; i < LEAVE_RANGES; i++) {
		lo = i < LEAVE_SMALL ? i : BIT_ULL(i - LEAVE_SMALL + ilog2(LEAVE_SMALL));
		taken = range_branch_taken(insn, ci, lo, i < LEAVE_SMALL ? lo : 2 * lo - 1);
		if (taken > 0)
			go |= BIT_ULL(i);
		else if (!taken)
			stay |= BIT_ULL(i);
	}
	p->vals[insn->dst_reg] |= (t & go) | (f & stay);
}

/*
 * A compiler makes
 *	w1 = 0; if w7 > 255 goto l; ...; w1 = 1; l: ...; w1 &= 1; if w1 != 0 goto head;
 * of 'do { ...; more = ++i < 256 && ...; ...; } while (more);'. The jump that leaves the loop
 * looks at 1 or 0. The jump that counts stays in the loop on both edges.
 *
 * So find for every insn of a loop when the loop is left from there on. Go against the edges:
 *	if w1 != 0 goto head	w1 is 0		the numbers that don't go to the head
 *	w1 &= 1			w1 is even	the numbers that the insn turns into 0
 *	l: ...			w1 is even	nothing writes w1
 *	w1 = 1			never		1 is not even
 * w1 is 0 at the jump that counts and 0 is even: the edge to l leaves the loop.
 *
 * An insn that is not in the loop: always. The head: never. More than one successor: what is
 * enough for each of them. Every insn has one turn, in post order. A cycle without the head is
 * a nested loop, so the successors in the loop itself had their turn. One that didn't: never.
 * The jumps on the way know constants too: 'r7 = 0; if w1 != 0 goto a; ...; a: if r7 == 2 goto
 * head' leaves the loop when w1 is not 0.
 *
 * Every edge that the program has at run time counts, the edges of may_goto that the verifier
 * adds too. May_goto of the loop and of the loops around it leaves the loop. A nested loop can
 * get one in front of a jump that the numbers decide the other way. So a nested loop: never.
 *
 * What is kept is the answer for the two edges of each jump: insn_aux_data->edge_leaves.
 */
int bpf_compute_leave_conds(struct bpf_verifier_env *env)
{
	const struct bpf_leave_cond *q;
	struct bpf_leave_cond *p;
	struct bpf_iarray *succ;
	int i, r, s, h, idx;
	bool jump;
	u64 any;

	env->leave_conds = kvcalloc(env->prog->len, sizeof(*p), GFP_KERNEL_ACCOUNT);
	if (!env->leave_conds)
		return -ENOMEM;
	for (i = 0; i < env->cfg.cur_postorder; i++) {
		idx = env->cfg.insn_postorder[i];
		h = bpf_loop_at_index(env, idx);
		if (h < 0)
			continue;
		p = &env->leave_conds[idx];
		*p = leave_always;
		jump = bpf_insn_is_cond_jump(env->prog->insnsi[idx].code);
		succ = bpf_insn_successors(env, idx);
		iarray_for_each(s, succ) {
			q = jump ? leave_cond_edge(env, h, idx, s) : leave_cond_at(env, h, s);
			for (r = 0; r < MAX_BPF_REG; r++)
				p->vals[r] &= q->vals[r];
			p->always &= q->always;
			if (jump && q->always)
				env->insn_aux_data[idx].edge_leaves |= BIT(s != idx + 1);
		}
		if (jump) {
			leave_cond_jump(env, h, idx, p);
			continue;
		}
		for (r = 0, any = 0; r < MAX_BPF_REG; r++)
			any |= p->vals[r];
		if (any && !p->always)
			leave_cond_insn(env, idx, p);
	}
	kvfree(env->leave_conds);
	env->leave_conds = NULL;
	return 0;
}

/*
 * Does the program leave the loop at @h when the conditional jump at @n goes on at @t?
 * @h is the innermost loop around @n.
 */
bool bpf_edge_leaves_loop(struct bpf_verifier_env *env, int h, int n, int t)
{
	return leave_cond_at(env, h, t)->always ||
	       (env->insn_aux_data[n].edge_leaves & BIT(t != n + 1));
}

/*
 * Rewrite conditional branches with constant outcomes into unconditional
 * jumps using register values resolved by bpf_compute_const_regs() pass.
 * This eliminates dead edges from the CFG so that compute_live_registers()
 * doesn't propagate liveness through dead code.
 */
int bpf_prune_dead_branches(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *insn_aux = env->insn_aux_data;
	struct bpf_insn *insns = env->prog->insnsi;
	int insn_cnt = env->prog->len;
	bool changed = false;
	int i;

	for (i = 0; i < insn_cnt; i++) {
		struct bpf_insn_aux_data *aux = &insn_aux[i];
		struct bpf_insn *insn = &insns[i];
		u8 class = BPF_CLASS(insn->code);
		u64 dst_val, src_val;
		int taken;

		if (!bpf_insn_is_cond_jump(insn->code))
			continue;
		if (BPF_OP(insn->code) == BPF_JCOND)
			continue;

		if (!(aux->const_reg_mask & BIT(insn->dst_reg)))
			continue;
		dst_val = aux->const_reg_vals[insn->dst_reg];

		if (BPF_SRC(insn->code) == BPF_K) {
			src_val = insn->imm;
		} else {
			if (!(aux->const_reg_mask & BIT(insn->src_reg)))
				continue;
			src_val = aux->const_reg_vals[insn->src_reg];
		}

		if (class == BPF_JMP32) {
			/*
			 * The (s32) cast maps the 32-bit range into two u64 sub-ranges:
			 * [0x00000000, 0x7FFFFFFF] -> [0x0000000000000000, 0x000000007FFFFFFF]
			 * [0x80000000, 0xFFFFFFFF] -> [0xFFFFFFFF80000000, 0xFFFFFFFFFFFFFFFF]
			 * The ordering is preserved within each sub-range, and
			 * the second sub-range is above the first as u64.
			 */
			dst_val = (s32)dst_val;
			src_val = (s32)src_val;
		}

		taken = eval_const_branch(insn->code, dst_val, src_val);
		if (taken < 0) {
			bpf_log(&env->log, "Unknown conditional jump %x\n", insn->code);
			return -EFAULT;
		}
		*insn = BPF_JMP_A(taken ? insn->off : 0);
		changed = true;
	}

	if (!changed)
		return 0;
	/* recompute postorder, since CFG has changed */
	kvfree(env->cfg.insn_postorder);
	env->cfg.insn_postorder = NULL;
	kvfree(env->cfg.postorder_nums);
	env->cfg.postorder_nums = NULL;
	return bpf_compute_postorder(env);
}
