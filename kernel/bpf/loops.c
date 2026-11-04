// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */

#include <linux/slab.h>
#include <linux/sched/signal.h>
#include <linux/bpf_verifier.h>

static bool is_cfg_jump(struct bpf_insn *insn)
{
	u8 class = BPF_CLASS(insn->code);
	u8 opcode = BPF_OP(insn->code);

	switch (class) {
	case BPF_JMP:
	case BPF_JMP32:
		return opcode != BPF_CALL;
	default:
		return false;
	}
}

static struct bpf_iarray **compute_predecessors(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_iarray *succ, *preds, **result;
	struct bpf_prog *prog = env->prog;
	u32 *num_preds, i, s, len = prog->len;
	struct bpf_insn *insn;
	size_t sz;
	void *tmp;

	num_preds = kvcalloc(prog->len, sizeof(u32), GFP_KERNEL_ACCOUNT);
	if (!num_preds)
		return NULL;

	/*
	 * 'result' layout:
	 *  - array of pointers (struct bpf_iarray *)[len]
	 *  - struct bpf_iarray one after another
	 */
	sz = size_mul(len, sizeof(*result) + sizeof(**result));
	for (i = 0; i < len; i++) {
		insn = env->prog->insnsi + i;
		succ = bpf_insn_successors(env, i);
		sz = size_add(sz, array_size(succ->cnt, sizeof(*succ->items)));
		iarray_for_each(s, succ) {
			num_preds[s]++;
		}
		if (bpf_is_ldimm64(insn))
			i++;
	}

	result = kvzalloc(sz, GFP_KERNEL_ACCOUNT);
	if (!result) {
		kvfree(num_preds);
		return NULL;
	}

	tmp = (void *)&result[len];
	for (i = 0; i < len; i++) {
		result[i] = tmp;
		tmp += struct_size(result[i], items, num_preds[i]);
	}

	for (i = 0; i < len; i++) {
		insn = env->prog->insnsi + i;
		succ = bpf_insn_successors(env, i);
		iarray_for_each(s, succ) {
			preds = result[s];
			preds->items[preds->cnt++] = i;
		}
		if (succ->cnt > 1 ||
		    (succ->cnt == 1 && num_preds[succ->items[0]] > 1) ||
		    is_cfg_jump(insn))
			aux[i].bb_end = true;
		if (bpf_is_ldimm64(insn))
			i++;
	}

	kvfree(num_preds);
	return result;
}

static int idoms_intersect(struct bpf_verifier_env *env, int a, int b)
{
	int *postorder_nums = env->cfg.postorder_nums;
	int *idoms = env->idoms;

	while (a != b) {
		while (postorder_nums[a] < postorder_nums[b]) {
			a = idoms[a];
		}
		while (postorder_nums[b] < postorder_nums[a]) {
			b = idoms[b];
		}
	}
	return a;
}

/* See "A Simple, Fast Dominance Algorithm" by Cooper et al. for details. */
static int compute_subprog_idoms(struct bpf_verifier_env *env, struct bpf_iarray **preds,
				int subprog_idx)
{
	struct bpf_subprog_info *subprog = &env->subprog_info[subprog_idx];
	int start = subprog->start;
	int po_first = subprog->postorder_start;
	int po_last = (subprog + 1)->postorder_start - 1;
	int *idoms = env->idoms;
	int po_num, pred;
	u32 work = 0;
	bool changed;

	idoms[start] = start;
	changed = true;
	do {
		changed = false;
		/*
		 * Iterate in reverse postorder, skipping the subprog entry:
		 * insn_postorder[po_last] is the subprogram entry.
		 */
		for (po_num = po_last - 1; po_num >= po_first; po_num--) {
			int idx = env->cfg.insn_postorder[po_num];
			int new_idom = -1;

			iarray_for_each(pred, preds[idx]) {
				if (++work % 1024 == 0) {
					if (signal_pending(current))
						return -EAGAIN;
					cond_resched();
				}

				if (idoms[pred] == -1)
					continue;
				if (new_idom == -1)
					new_idom = pred;
				else
					new_idom = idoms_intersect(env, pred, new_idom);
			}
			if (new_idom != -1 && idoms[idx] != new_idom) {
				idoms[idx] = new_idom;
				changed = true;
			}
		}
	} while (changed);
	idoms[start] = -1;
	return 0;
}

int bpf_compute_idoms(struct bpf_verifier_env *env)
{
	struct bpf_iarray **preds;
	u32 len = env->prog->len;
	int *idoms, i, err = 0;

	preds = compute_predecessors(env);
	if (!preds)
		return -ENOMEM;

	idoms = kvcalloc(len, sizeof(*idoms), GFP_KERNEL_ACCOUNT);
	if (!idoms) {
		kvfree(preds);
		return -ENOMEM;
	}

	env->idoms = idoms;
	for (i = 0; i < len; i++)
		idoms[i] = -1;

	for (i = 0; i < env->subprog_cnt; i++) {
		err = compute_subprog_idoms(env, preds, i);
		if (err)
			break;
	}

	kvfree(preds);
	return err;
}

struct dfs_state {
	u32 traversed:1;
	u32 next_succ:31;
};

struct loops_dfs {
	struct dfs_state *state;
	int *dfs_pos;
	int *stack;
};

static void mark_irreducible(struct bpf_verifier_env *env, int h)
{
	env->insn_aux_data[h].loop->irreducible = true;
}

static void add_backedge(struct bpf_verifier_env *env, int from, int h)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_loop *loop = aux[h].loop;
	int cnt = loop->backedges_cnt;

	if (cnt == MAX_BACKEDGES) {
		loop->backedges_overflow = true;
		return;
	}
	loop->backedges[cnt].from = from;
	loop->backedges[cnt].latches_cnt = 0;
	loop->backedges_cnt++;
}

static int add_exit(struct bpf_loop *loop, int from, int to)
{
	if (loop->exits_overflow)
		return 0;
	if (loop->exits_cnt == MAX_LOOP_EXITS) {
		loop->exits_overflow = true;
		return 0;
	}
	if (!loop->exits) {
		loop->exits = kvcalloc(MAX_LOOP_EXITS, sizeof(*loop->exits), GFP_KERNEL_ACCOUNT);
		if (!loop->exits)
			return -ENOMEM;
	}
	loop->exits[loop->exits_cnt] = (struct bpf_loop_exit) {
		.from = from,
		.to = to,
	};
	loop->exits_cnt++;
	return 0;
}

static int mark_as_header(struct bpf_verifier_env *env, int h)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;

	if (!aux[h].loop) {
		aux[h].loop = kvzalloc_obj(struct bpf_loop, GFP_KERNEL_ACCOUNT);
		if (!aux[h].loop)
			return -ENOMEM;
	}
	return 0;
}

static int assign_header(struct bpf_verifier_env *env, struct loops_dfs *dfs, int n, int h)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int *dfs_pos = dfs->dfs_pos;
	int err, nh;

	err = mark_as_header(env, h);
	if (err)
		return err;

	/* Don't encode self-loops, otherwise can't reflect loops nesting structure. */
	if (n == h)
		return 0;

	/* Make sure that loop headers up the chain are sorted by dfs_pos. */
	while (aux[n].loop_header != -1) {
		nh = aux[n].loop_header;
		if (nh == h)
			return 0;
		if (dfs_pos[nh] < dfs_pos[h]) {
			aux[n].loop_header = h;
			n = h;
			h = nh;
		} else {
			n = nh;
		}
	}
	aux[n].loop_header = h;
	return 0;
}

static bool is_cond_jmp_insn(struct bpf_insn *insn)
{
	u8 class = BPF_CLASS(insn->code);
	u8 opcode = BPF_OP(insn->code);

	if (class != BPF_JMP && class != BPF_JMP32)
		return false;

	switch (opcode) {
	case BPF_JEQ:
	case BPF_JGE:
	case BPF_JGT:
	case BPF_JLE:
	case BPF_JLT:
	case BPF_JNE:
	case BPF_JSET:
	case BPF_JSGE:
	case BPF_JSGT:
	case BPF_JSLE:
	case BPF_JSLT:
		return true;
	default:
		return false;
	}
}

int bpf_loop_at_index(struct bpf_verifier_env *env, u32 idx)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;

	return aux[idx].loop ? idx : aux[idx].loop_header;
}

/* @known: the registers that hold the constant @val behind @insn. False: a jump or the like. */
static bool track_consts(const struct bpf_insn *insn, u16 *known, s32 *val)
{
	u8 class = BPF_CLASS(insn->code);

	if (class == BPF_ST || (class == BPF_STX && BPF_MODE(insn->code) == BPF_MEM))
		return true;
	/* r6 - r9 are what they were when the call returns */
	if (insn->code == (BPF_JMP | BPF_CALL)) {
		*known &= ~GENMASK(BPF_REG_5, BPF_REG_0);
		return true;
	}
	if (class != BPF_ALU && class != BPF_ALU64 && class != BPF_LDX)
		return false;
	*known &= ~BIT(insn->dst_reg);
	if (class != BPF_LDX && BPF_OP(insn->code) == BPF_MOV && BPF_SRC(insn->code) == BPF_K) {
		*known |= BIT(insn->dst_reg);
		val[insn->dst_reg] = insn->imm;
	}
	return true;
}

/*
 * Does the program leave the loop at @h when the conditional jump at @n goes on at @t?
 * It does when @t is not in the loop. A compiler also makes
 *	w2 = 1; if w1 == 0 goto l; w2 = 0; l: ...; if w2 != 1 goto head;
 * of 'while (n--)'. So follow the constants that are set next to the jump for as long as there
 * is one way to go.
 */
bool bpf_edge_leaves_loop(struct bpf_verifier_env *env, int h, int n, int t)
{
	struct bpf_insn *insns = env->prog->insnsi, *insn;
	s32 val[MAX_BPF_REG];
	u16 known = 0;
	int i, l;
	u8 op;

	/* the insns in front of the jump, for as long as nothing else leads to the next one */
	for (i = n; i && i > n - 4 && !env->insn_aux_data[i].loop && env->idoms[i] == i - 1; i--)
		;
	for (; i < n; i++)
		if (!track_consts(&insns[i], &known, val))
			known = 0;
	for (i = 0; i < 16; i++, t++) {
		l = bpf_loop_at_index(env, t);
		if (l != h && !bpf_is_nested_loop(env, l, h))
			return true;
		insn = &insns[t];
		op = BPF_OP(insn->code);
		if (t == h) {
			return false;
		} else if (insn->code == (BPF_JMP | BPF_JA)) {
			t += insn->off;
		} else if (is_cond_jmp_insn(insn)) {
			/* a 32-bit and a 64-bit jump agree on a number that is not negative */
			if (!(known & BIT(insn->dst_reg)) || val[insn->dst_reg] < 0 ||
			    BPF_SRC(insn->code) != BPF_K || (op != BPF_JEQ && op != BPF_JNE))
				return false;
			if ((val[insn->dst_reg] == insn->imm) == (op == BPF_JEQ))
				t += insn->off;
		} else if (!track_consts(insn, &known, val)) {
			return false;
		}
	}
	return false;
}

/*
 * Collect conditional jumps within n's loop that dominate 'n' and leave
 * the loop on one edge, see bpf_edge_leaves_loop(), nearest first.
 */
static void find_dominating_conditions(struct bpf_verifier_env *env, int n,
				       struct bpf_backedge *backedge)
{
	struct bpf_insn *insns = env->prog->insnsi;
	bool t_exits_loop, f_exits_loop;
	int *idoms = env->idoms;
	int n_loop;

	n_loop = bpf_loop_at_index(env, n);
	if (idoms_intersect(env, n, n_loop) != n_loop)
		return;
	while (n >= 0) {
		if (is_cond_jmp_insn(&insns[n]) && bpf_loop_at_index(env, n) == n_loop) {
			t_exits_loop = bpf_edge_leaves_loop(env, n_loop, n, n + insns[n].off + 1);
			f_exits_loop = bpf_edge_leaves_loop(env, n_loop, n, n + 1);
			/* latches[0] is the latch: it has to leave the loop itself */
			if ((t_exits_loop || f_exits_loop) &&
			    (backedge->latches_cnt || bpf_jump_leaves_loop(env, n_loop, n))) {
				if (backedge->latches_cnt == MAX_LATCHES)
					return;
				backedge->latches[backedge->latches_cnt++] = n;
			}
		}
		if (n == n_loop)
			break;
		n = idoms[n];
	}
}

/*
 * As described in "A New Algorithm for Identifying Loops in Decompilation" by Wei et al,
 * adapted to be non-recursive.
 */
static int compute_loops_in_subprog(struct bpf_verifier_env *env, struct loops_dfs *dfs,
				    int subprog_idx)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct dfs_state *state = dfs->state;
	int start = env->subprog_info[subprog_idx].start;
	int *dfs_pos = dfs->dfs_pos;
	int *stack = dfs->stack;
	int s, h, err, cur, stack_sz;
	struct bpf_iarray *succ;
	u32 i;

	stack[0] = start;
	state[start].traversed = true;
	state[start].next_succ = 0;
	dfs_pos[start] = 1;
	stack_sz = 1;
	i = 0;
	do {
		/*
		 * The algorithm should be very fast in practice,
		 * guard against pathological inputs, just in case.
		 */
		if ((++i % 1024) == 0) {
			if (signal_pending(current))
				return -EAGAIN;
			cond_resched();
		}

		cur = stack[stack_sz - 1];
		succ = bpf_insn_successors(env, cur);
		if (state[cur].next_succ == succ->cnt) {
			dfs_pos[cur] = 0;
			stack_sz--;
			continue;
		}
		s = succ->items[state[cur].next_succ];
		if (!state[s].traversed) {
			/* Case A:  start -> ... -> cur -> s [unexplored] */
			state[s].traversed = true;
			state[s].next_succ = 0;
			stack[stack_sz] = s;
			dfs_pos[s] = stack_sz + 1;
			stack_sz++;
			continue;
		}
		/* 's' is fully explored at this point */
		if (dfs_pos[s]) {
			/*
			 * start -> ... -> s -> cur --.
			 *                 ^          |
			 *                 '----------'
			 * Case B: 's' is in the current DFS path.
			 */
			err = assign_header(env, dfs, cur, s);
			if (err)
				return err;
			add_backedge(env, cur, s);
		} else if (aux[s].loop_header == -1) {
			/*
			 * start -> ... -> ... -> s -> ... -> end
			 *           |            ^
			 *           '---> cur ---'
			 * Case C: 's' is explored, not in the current DFS path,
			 * and not a part of any loop.
			 */
		} else if (dfs_pos[aux[s].loop_header]) {
			/*
			 *                 .----------------------.
			 *                 v                      |
			 * start -> ... -> h -> ... -> ... -> s --'
			 *                       |            ^
			 *	                 '---> cur ---'
			 * Case D: 's' is explored, not in current DFS path,
			 * but its innermost loop header is.
			 */
			err = assign_header(env, dfs, cur, aux[s].loop_header);
			if (err)
				return err;
		} else {
			/*
			 * case E: 's' is explored, not in current DFS path,
			 * its innermost loop header is not in current DFS path,
			 * hence 's' is another entry into the same loop.
			 */
			h = aux[s].loop_header;
			mark_irreducible(env, h);
			while (aux[h].loop_header != -1) {
				h = aux[h].loop_header;
				if (dfs_pos[h]) {
					err = assign_header(env, dfs, cur, h);
					if (err)
						return err;
					break;
				}
				mark_irreducible(env, h);
			}
		}
		state[cur].next_succ++;
	} while (stack_sz);

	return 0;
}

bool bpf_is_nested_loop(struct bpf_verifier_env *env, int inner_header, int outer_header)
{
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int idx;

	for (idx = inner_header; idx >= 0; idx = aux[idx].loop_header)
		if (aux[idx].loop_header == outer_header)
			return true;

	return false;
}

/* Whether the conditional jump at @n, in the loop @h or in a loop nested in it, leaves @h */
bool bpf_jump_leaves_loop(struct bpf_verifier_env *env, int h, int n)
{
	struct bpf_insn *insn = &env->prog->insnsi[n];
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code);
	int t, f;

	if ((class != BPF_JMP && class != BPF_JMP32) || op == BPF_JA || op == BPF_CALL ||
	    op == BPF_EXIT || op == BPF_JCOND)
		return false;
	t = bpf_loop_at_index(env, n + insn->off + 1);
	f = bpf_loop_at_index(env, n + 1);
	return (t != h && !bpf_is_nested_loop(env, t, h)) ||
	       (f != h && !bpf_is_nested_loop(env, f, h));
}

int bpf_compute_loops(struct bpf_verifier_env *env)
{
	int i, j, k, s, t, iloop, sloop, err = 0, len = env->prog->len;
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_verifier_log *log = &env->log;
	struct bpf_backedge *backedge;
	struct loops_dfs dfs = {};
	struct bpf_iarray *succ;
	struct bpf_loop *loop;

	dfs.dfs_pos = kvcalloc(len, sizeof(int), GFP_KERNEL_ACCOUNT);
	dfs.state = kvcalloc(len, sizeof(struct dfs_state), GFP_KERNEL_ACCOUNT);
	dfs.stack = kvcalloc(len, sizeof(int), GFP_KERNEL_ACCOUNT);
	if (!dfs.dfs_pos || !dfs.state || !dfs.stack) {
		err = -ENOMEM;
		goto out;
	}
	for (i = 0; i < len; i++)
		aux[i].loop_header = -1;
	for (i = 0; i < env->subprog_cnt; i++) {
		err = compute_loops_in_subprog(env, &dfs, i);
		if (err)
			goto out;
	}
	/* find latches */
	for (i = 0; i < len; i++) {
		loop = aux[i].loop;
		if (!loop)
			continue;
		for (j = 0; j < loop->backedges_cnt; j++) {
			backedge = &loop->backedges[j];
			/*
			 * In theory, the backedge->from and it's latch can reside in
			 * an inner loop of `i`, e.g.:
			 *
			 *  1:  r7 += 1;
			 *  2:  r6 += 1;
			 *      if r7 == 2 goto 1b;
			 *      if r6 < 2 goto 2b;
			 *
			 * For now, let's assume that latches are unknown for such cases.
			 * (GCC/LLVM handle this by inserting artificial cfg nodes).
			 */
			if (bpf_loop_at_index(env, backedge->from) != i)
				continue;
			find_dominating_conditions(env, backedge->from, backedge);
		}
	}
	/* find exits */
	for (i = 0; i < len; i++) {
		iloop = aux[i].loop ? i : aux[i].loop_header;
		if (iloop < 0)
			continue;
		succ = bpf_insn_successors(env, i);
		iarray_for_each(s, succ) {
			/*
			 * Nothing left to record once the innermost loop of 'i'
			 * overflowed: the walk below would stop at it right away.
			 */
			if (aux[iloop].loop->exits_overflow)
				break;
			sloop = aux[s].loop ? s : aux[s].loop_header;
			if (iloop == sloop)
				continue;
			if (bpf_is_nested_loop(env, sloop, iloop))
				continue;
			/*
			 * At this point 'sloop' is either -1, an outer loop,
			 * or a loop in another branch of the loop hierarchy.
			 */
			for (t = iloop; t >= 0 && t != sloop; t = aux[t].loop_header) {
				/*
				 * Account for the following configuration:
				 *
				 *   tloop {
				 *     iloop {
				 *       ... i: goto s;
				 *     }
				 *     sloop {
				 *   s:
				 *       ...
				 *     }
				 *   }
				 */
				if (bpf_is_nested_loop(env, sloop, t))
					break;
				/*
				 * Record edges i -> s as exits from tloop when:
				 *
				 *   sloop {
				 *     tloop {
				 *       iloop {
				 *         ... i: goto s;
				 *       }
				 *     }
				 *  s: ...
				 *   }
				 */
				/* Enclosing loops get nested_exits_overflow, see below */
				if (aux[t].loop->exits_overflow)
					break;
				err = add_exit(aux[t].loop, i, s);
				if (err)
					goto out;
			}
		}
		if (bpf_is_ldimm64(env->prog->insnsi + i))
			i++;
	}
	/* A nested loop with a truncated exit list can't be abstracted by SCEV */
	for (i = 0; i < len; i++) {
		loop = aux[i].loop;
		if (!loop || !loop->exits_overflow)
			continue;
		for (t = aux[i].loop_header; t >= 0; t = aux[t].loop_header)
			aux[t].loop->nested_exits_overflow = true;
	}

	if (env->log.level & BPF_LOG_LEVEL2) {
		for (i = 0; i < len; i++) {
			loop = aux[i].loop;
			if (!loop)
				continue;
			bpf_log(log, "loop at %d", i);
			if (aux[i].loop_header >= 0)
				bpf_log(log, ", nested in %d", aux[i].loop_header);
			if (loop->irreducible)
				bpf_log(log, ", irreducible");
			if (loop->exits_overflow)
				bpf_log(log, ", too many exits");
			if (loop->nested_exits_overflow)
				bpf_log(log, ", too many exits of a nested loop");
			bpf_log(log, "\n");
			for (j = 0; j < loop->backedges_cnt; j++) {
				backedge = &loop->backedges[j];
				bpf_log(log, "  backedge from %d", backedge->from);
				for (k = 0; k < backedge->latches_cnt; k++) {
					if (k == 0)
						bpf_log(log, ", latch at ");
					if (k == 1)
						bpf_log(log, ", dominating exits at ");
					if (k > 1)
						bpf_log(log, ", ");
					bpf_log(log, "%d", backedge->latches[k]);
				}
				bpf_log(log, "\n");
			}
			for (j = 0; j < loop->exits_cnt; j++)
				bpf_log(log, "  exit from %d to %d\n",
					loop->exits[j].from, loop->exits[j].to);
		}
	}

out:
	kvfree(dfs.dfs_pos);
	kvfree(dfs.stack);
	kvfree(dfs.state);
	return err;
}
