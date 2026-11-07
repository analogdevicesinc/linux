// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */
#include <linux/bpf.h>
#include <linux/bpf_verifier.h>
#include <linux/filter.h>
#include <linux/sort.h>

#include "diagnostics.h"

#define verbose(env, fmt, args...) bpf_verifier_log_write(env, fmt, ##args)

#define BPF_MAX_GOTOX_EDGES	BPF_COMPLEXITY_LIMIT_INSNS

/* non-recursive DFS pseudo code
 * 1  procedure DFS-iterative(G,v):
 * 2      label v as discovered
 * 3      let S be a stack
 * 4      S.push(v)
 * 5      while S is not empty
 * 6            t <- S.peek()
 * 7            if t is what we're looking for:
 * 8                return t
 * 9            for all edges e in G.adjacentEdges(t) do
 * 10               if edge e is already labelled
 * 11                   continue with the next edge
 * 12               w <- G.adjacentVertex(t,e)
 * 13               if vertex w is not discovered and not explored
 * 14                   label e as tree-edge
 * 15                   label w as discovered
 * 16                   S.push(w)
 * 17                   continue at 5
 * 18               else if vertex w is discovered
 * 19                   label e as back-edge
 * 20               else
 * 21                   // vertex w is explored
 * 22                   label e as forward- or cross-edge
 * 23           label t as explored
 * 24           S.pop()
 *
 * convention:
 * 0x10 - discovered
 * 0x11 - discovered and fall-through edge labelled
 * 0x12 - discovered and fall-through and branch edges labelled
 * 0x20 - explored
 */

enum {
	DISCOVERED = 0x10,
	EXPLORED = 0x20,
	FALLTHROUGH = 1,
	BRANCH = 2,
};

static void mark_subprog_changes_pkt_data(struct bpf_verifier_env *env, int off)
{
	struct bpf_subprog_info *subprog;

	subprog = bpf_find_containing_subprog(env, off);
	subprog->changes_pkt_data = true;
}

static void mark_subprog_might_sleep(struct bpf_verifier_env *env, int off)
{
	struct bpf_subprog_info *subprog;

	subprog = bpf_find_containing_subprog(env, off);
	subprog->might_sleep = true;
}

static void mark_subprog_might_throw(struct bpf_verifier_env *env, int off)
{
	struct bpf_subprog_info *subprog;

	subprog = bpf_find_containing_subprog(env, off);
	subprog->might_throw = true;
}

/*
 * 't' is an index of a call-site.
 * 'w' is a callee entry point.
 * Eventually this function would be called when env->cfg.insn_state[w] == EXPLORED.
 * DFS traversal order guarantees that callee's effect marks are correct at that
 * moment, unless the callee calls one of its callers: see close_callee_effects().
 * Returns true if the marks of the caller changed.
 */
static bool merge_callee_effects(struct bpf_verifier_env *env, int t, int w)
{
	struct bpf_subprog_info *caller, *callee;

	caller = bpf_find_containing_subprog(env, t);
	callee = bpf_find_containing_subprog(env, w);
	if (caller->changes_pkt_data >= callee->changes_pkt_data &&
	    caller->might_sleep >= callee->might_sleep &&
	    caller->might_throw >= callee->might_throw)
		return false;
	caller->changes_pkt_data |= callee->changes_pkt_data;
	caller->might_sleep |= callee->might_sleep;
	caller->might_throw |= callee->might_throw;
	return true;
}

enum {
	DONE_EXPLORING = 0,
	KEEP_EXPLORING = 1,
};

/* t, w, e - match pseudo-code above:
 * t - index of current instruction
 * w - next instruction
 * e - edge
 */
static int push_insn(int t, int w, int e, struct bpf_verifier_env *env)
{
	int *insn_stack = env->cfg.insn_stack;
	int *insn_state = env->cfg.insn_state;

	if (e == FALLTHROUGH && insn_state[t] >= (DISCOVERED | FALLTHROUGH))
		return DONE_EXPLORING;

	if (e == BRANCH && insn_state[t] >= (DISCOVERED | BRANCH))
		return DONE_EXPLORING;

	if (w < 0 || w >= env->prog->len) {
		verbose_linfo(env, t, "%d: ", t);
		verbose(env, "jump out of range from insn %d to %d\n", t, w);
		bpf_diag_program_structure(
			env, t, "jump out of range", "Keep branch targets inside the program.",
			"Instruction %d jumps to instruction %d, but the program only contains instructions 0 through %d.",
			t, w, env->prog->len - 1);
		return -EINVAL;
	}

	if (e == BRANCH)
		mark_jump_target(env, w);

	if (insn_state[w] == 0) {
		/* tree-edge */
		insn_state[t] = DISCOVERED | e;
		insn_state[w] = DISCOVERED;
		if (env->cfg.cur_stack >= env->prog->len)
			return -E2BIG;
		insn_stack[env->cfg.cur_stack++] = w;
		return KEEP_EXPLORING;
	} else if ((insn_state[w] & 0xF0) == DISCOVERED) {
		if (env->bpf_capable)
			return DONE_EXPLORING;
		verbose_linfo(env, t, "%d: ", t);
		verbose_linfo(env, w, "%d: ", w);
		verbose(env, "back-edge from insn %d to %d\n", t, w);
		bpf_diag_program_structure(
			env, t, "back-edge is not allowed",
			"Load with privileges that allow this back-edge, or rewrite the control flow so it does not branch backward.",
			"Instruction %d branches back to instruction %d. This program is being rejected without the privilege needed for this back-edge.",
			t, w);
		return -EINVAL;
	} else if (insn_state[w] == EXPLORED) {
		/* forward- or cross-edge */
		insn_state[t] = DISCOVERED | e;
	} else {
		verifier_bug(env, "insn state internal bug");
		return -EFAULT;
	}
	return DONE_EXPLORING;
}

static int visit_func_call_insn(int t, struct bpf_insn *insns,
				struct bpf_verifier_env *env,
				bool visit_callee)
{
	int ret, insn_sz;
	int w;

	insn_sz = bpf_is_ldimm64(&insns[t]) ? 2 : 1;
	ret = push_insn(t, t + insn_sz, FALLTHROUGH, env);
	if (ret)
		return ret;

	if (visit_callee) {
		w = t + insns[t].imm + 1;
		merge_callee_effects(env, t, w);
		ret = push_insn(t, w, BRANCH, env);
	}
	return ret;
}

struct bpf_iarray *bpf_iarray_realloc(struct bpf_iarray *old, size_t n_elem)
{
	size_t new_size = sizeof(struct bpf_iarray) + n_elem * sizeof(old->items[0]);
	struct bpf_iarray *new;

	new = kvrealloc(old, new_size, GFP_KERNEL_ACCOUNT);
	if (!new) {
		/* this is what callers always want, so simplify the call site */
		kvfree(old);
		return NULL;
	}

	new->cnt = n_elem;
	return new;
}

static int copy_insn_array(struct bpf_map *map, u32 start, u32 end, u32 *items)
{
	struct bpf_insn_array_value *value;
	u32 i;

	for (i = start; i <= end; i++) {
		value = map->ops->map_lookup_elem(map, &i);
		/*
		 * map_lookup_elem of an array map will never return an error,
		 * but not checking it makes some static analysers to worry
		 */
		if (IS_ERR(value))
			return PTR_ERR(value);
		else if (!value)
			return -EINVAL;
		items[i - start] = value->xlated_off;
	}
	return 0;
}

static int cmp_ptr_to_u32(const void *a, const void *b)
{
	return *(u32 *)a - *(u32 *)b;
}

static int sort_insn_array_uniq(u32 *items, int cnt)
{
	int unique = 1;
	int i;

	sort(items, cnt, sizeof(items[0]), cmp_ptr_to_u32, NULL);

	for (i = 1; i < cnt; i++)
		if (items[i] != items[unique - 1])
			items[unique++] = items[i];

	return unique;
}

/*
 * sort_unique({map[start], ..., map[end]}) into off
 */
int bpf_copy_insn_array_uniq(struct bpf_map *map, u32 start, u32 end, u32 *off)
{
	u32 n = end - start + 1;
	int err;

	err = copy_insn_array(map, start, end, off);
	if (err)
		return err;

	return sort_insn_array_uniq(off, n);
}

/*
 * Copy all unique offsets from the map
 */
static struct bpf_iarray *jt_from_map(struct bpf_map *map)
{
	struct bpf_iarray *jt;
	int err;
	int n;

	jt = bpf_iarray_realloc(NULL, map->max_entries);
	if (!jt)
		return ERR_PTR(-ENOMEM);

	n = bpf_copy_insn_array_uniq(map, 0, map->max_entries - 1, jt->items);
	if (n < 0) {
		err = n;
		goto err_free;
	}
	if (n == 0) {
		err = -EINVAL;
		goto err_free;
	}
	jt->cnt = n;
	return jt;

err_free:
	kvfree(jt);
	return ERR_PTR(err);
}

/*
 * Collect the jump table of every subprogram that has one, as the combined
 * table of all maps whose targets land inside that subprogram. All gotox
 * instructions of a subprogram share the same table, so this is done in a
 * single pass over the maps rather than once per gotox.
 */
static int __compute_subprog_jts(struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *subprog;
	struct bpf_iarray *jt, *jt_cur;
	struct bpf_map *map;
	u32 old_cnt;
	int i;

	for (i = 0; i < env->insn_array_map_cnt; i++) {
		/*
		 * TODO (when needed): collect only jump tables, not static keys
		 * or maps for indirect calls
		 */
		map = env->insn_array_maps[i];

		jt_cur = jt_from_map(map);
		if (IS_ERR(jt_cur))
			return PTR_ERR(jt_cur);

		subprog = bpf_find_containing_subprog(env, jt_cur->items[0]);
		if (verifier_bug_if(!subprog, env, "no subprog contains insn %u of an insn array map",
				    jt_cur->items[0])) {
			kvfree(jt_cur);
			return -EFAULT;
		}
		if (jt_cur->items[jt_cur->cnt - 1] >= (subprog + 1)->start) {
			verbose(env, "jump table of subprog starting at %u spans multiple subprogs, map id=%d\n",
				subprog->start, map->id);
			bpf_diag_program_structure(
				env, subprog->start, "jump table spans subprograms",
				"Keep every entry of a jump table inside one subprogram.",
				"The jump table in map id %d for the subprogram that starts at instruction %u reaches past its end at instruction %u.",
				map->id, subprog->start, (subprog + 1)->start);
			kvfree(jt_cur);
			return -EINVAL;
		}

		old_cnt = subprog->jt ? subprog->jt->cnt : 0;
		jt = bpf_iarray_realloc(subprog->jt, old_cnt + jt_cur->cnt);
		if (!jt) {
			subprog->jt = NULL;
			kvfree(jt_cur);
			return -ENOMEM;
		}
		memcpy(jt->items + old_cnt, jt_cur->items, jt_cur->cnt << 2);
		subprog->jt = jt;

		kvfree(jt_cur);
	}

	for (i = 0; i < env->subprog_cnt; i++) {
		jt = env->subprog_info[i].jt;
		if (jt)
			jt->cnt = sort_insn_array_uniq(jt->items, jt->cnt);
	}

	env->cfg.subprog_jts_ready = true;
	return 0;
}

/* Leave nothing behind on failure. */
static int compute_subprog_jts(struct bpf_verifier_env *env)
{
	int err;

	err = __compute_subprog_jts(env);
	if (err)
		bpf_free_subprog_jts(env);
	return err;
}

void bpf_free_subprog_jts(struct bpf_verifier_env *env)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(env->subprog_info); i++) {
		kvfree(env->subprog_info[i].jt);
		env->subprog_info[i].jt = NULL;
	}
	env->cfg.subprog_jts_ready = false;
}

static struct bpf_iarray *
subprog_jt(int t, struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *subprog;
	int subprog_start, err;

	if (!env->cfg.subprog_jts_ready) {
		err = compute_subprog_jts(env);
		if (err)
			return ERR_PTR(err);
	}

	subprog = bpf_find_containing_subprog(env, t);
	if (verifier_bug_if(!subprog, env, "no subprog contains insn %d", t))
		return ERR_PTR(-EFAULT);
	subprog_start = subprog->start;

	if (!subprog->jt) {
		verbose(env, "no jump tables found for subprog starting at %u\n", subprog_start);
		bpf_diag_program_structure(
			env, t, "missing jump table",
			"Make sure subprograms containing gotox instructions are accompanied by jump tables referencing these subprograms.",
			"No jump table was found for the subprogram that starts at instruction %u.",
			subprog_start);
		return ERR_PTR(-EINVAL);
	}

	return subprog->jt;
}

/* "conditional jump with N edges" */
static int visit_gotox_insn(int t, struct bpf_verifier_env *env)
{
	int *insn_stack = env->cfg.insn_stack;
	int *insn_state = env->cfg.insn_state;
	bool keep_exploring = false;
	struct bpf_iarray *jt;
	int i, w;

	/*
	 * The first visit pushes every target that still needs exploring, and
	 * those are all done by the time this insn is revisited, so there is
	 * nothing left to walk the jump table for.
	 */
	if (insn_state[t] & BRANCH)
		return DONE_EXPLORING;

	jt = subprog_jt(t, env);
	if (IS_ERR(jt))
		return PTR_ERR(jt);

	insn_state[t] |= BRANCH;

	if (check_add_overflow(env->cfg.gotox_edges, jt->cnt,
			       &env->cfg.gotox_edges) ||
	    env->cfg.gotox_edges > BPF_MAX_GOTOX_EDGES) {
		verbose(env, "number of indirect jump edges in the program exceeds %u\n",
			BPF_MAX_GOTOX_EDGES);
		bpf_diag_program_structure(
			env, t, "too many indirect jump edges",
			"Reduce the number of indirect jumps, or the number of distinct targets they can reach.",
			"The program has more than %u indirect jump edges in total, counted over every gotox instruction.",
			BPF_MAX_GOTOX_EDGES);
		return -E2BIG;
	}

	for (i = 0; i < jt->cnt; i++) {
		w = jt->items[i];
		if (verifier_bug_if(w < 0 || w >= env->prog->len, env,
				    "indirect jump from insn %d to %d out of range", t, w))
			return -EFAULT;

		mark_jump_target(env, w);

		/* EXPLORED || DISCOVERED */
		if (insn_state[w])
			continue;

		if (env->cfg.cur_stack >= env->prog->len)
			return -E2BIG;

		insn_stack[env->cfg.cur_stack++] = w;
		insn_state[w] |= DISCOVERED;
		keep_exploring = true;
	}

	return keep_exploring ? KEEP_EXPLORING : DONE_EXPLORING;
}

/*
 * Return pointers to functions in the read-only map that ld_imm64 instruction
 * 't' loads the address of, or of its value, if there are any.
 */
static struct bpf_func_ptr *insn_func_ptrs(struct bpf_verifier_env *env, int t, u32 *cnt)
{
	struct bpf_insn *insn = &env->prog->insnsi[t];

	*cnt = 0;
	if (!env->func_ptr_cnt || !bpf_is_ldimm64(insn))
		return NULL;
	if (insn->src_reg != BPF_PSEUDO_MAP_VALUE &&
	    insn->src_reg != BPF_PSEUDO_MAP_IDX_VALUE &&
	    insn->src_reg != BPF_PSEUDO_MAP_FD &&
	    insn->src_reg != BPF_PSEUDO_MAP_IDX)
		return NULL;

	return bpf_map_func_ptrs(env, env->used_maps[env->insn_aux_data[t].map_index], cnt);
}

/*
 * ld_imm64 that loads the address of a map that has pointers to functions
 * is similar to ld_imm64 with BPF_PSEUDO_FUNC that loads the address of one
 * function: any of them may be read from the map and called via callx later.
 * Treat it as a call of all of them.
 */
static int visit_func_ptrs_insn(int t, struct bpf_verifier_env *env,
				struct bpf_func_ptr *ptrs, u32 cnt)
{
	int *insn_stack = env->cfg.insn_stack;
	int *insn_state = env->cfg.insn_state;
	int ret, w;
	u32 i;

	ret = push_insn(t, t + 2, FALLTHROUGH, env);
	if (ret)
		return ret;

	/*
	 * Explore one function at a time like push_insn() does, so that
	 * DISCOVERED function is a caller of 't' and not the one that waits in
	 * insn_stack[], which merge_callee_effects() relies on.
	 */
	for (i = 0; i < cnt; i++) {
		w = ptrs[i].xlated_off;
		if (insn_state[w])
			continue;

		if (env->cfg.cur_stack >= env->prog->len)
			return -E2BIG;

		insn_stack[env->cfg.cur_stack++] = w;
		insn_state[w] = DISCOVERED;
		return KEEP_EXPLORING;
	}

	/* all functions are explored, their effects are known */
	for (i = 0; i < cnt; i++) {
		w = ptrs[i].xlated_off;
		merge_callee_effects(env, t, w);

		/* the same mark as push_insn() leaves on a branch target */
		mark_jump_target(env, w);
	}
	return DONE_EXPLORING;
}

/*
 * A subprog that calls one of its callers is EXPLORED before that caller is: it
 * took the marks that the caller had so far, and so did everybody who called it
 * since. Merge again at every call until nothing changes.
 */
static void close_callee_effects(struct bpf_verifier_env *env)
{
	struct bpf_insn *insn = env->prog->insnsi;
	struct bpf_func_ptr *ptrs;
	bool changed;
	u32 cnt, j;
	int t;

	do {
		changed = false;
		for (t = 0; t < env->prog->len; t++) {
			if (bpf_pseudo_call(&insn[t]) || bpf_pseudo_func(&insn[t]))
				changed |= merge_callee_effects(env, t, t + insn[t].imm + 1);
			ptrs = insn_func_ptrs(env, t, &cnt);
			for (j = 0; j < cnt; j++)
				changed |= merge_callee_effects(env, t, ptrs[j].xlated_off);
			if (bpf_is_ldimm64(&insn[t]))
				t++;
		}
	} while (changed);
}

/* Visits the instruction at index t and returns one of the following:
 *  < 0 - an error occurred
 *  DONE_EXPLORING - the instruction was fully explored
 *  KEEP_EXPLORING - there is still work to be done before it is fully explored
 */
static int visit_insn(int t, struct bpf_verifier_env *env)
{
	struct bpf_insn *insns = env->prog->insnsi, *insn = &insns[t];
	struct bpf_func_ptr *ptrs;
	int ret, off, insn_sz;
	u32 cnt;

	if (bpf_pseudo_func(insn))
		return visit_func_call_insn(t, insns, env, true);

	ptrs = insn_func_ptrs(env, t, &cnt);
	if (ptrs)
		return visit_func_ptrs_insn(t, env, ptrs, cnt);

	/* All non-branch instructions have a single fall-through edge. */
	if (BPF_CLASS(insn->code) != BPF_JMP &&
	    BPF_CLASS(insn->code) != BPF_JMP32) {
		insn_sz = bpf_is_ldimm64(insn) ? 2 : 1;
		return push_insn(t, t + insn_sz, FALLTHROUGH, env);
	}

	switch (BPF_OP(insn->code)) {
	case BPF_EXIT:
		return DONE_EXPLORING;

	case BPF_CALL:
		/* For functions that invoke callbacks it is not known how many times
		 * callback would be called. Verifier models callback calling functions
		 * by repeatedly visiting callback bodies and returning to origin call
		 * instruction.
		 */
		if (bpf_is_sync_callback_calling_insn(insn))
			mark_calls_callback(env, t);
		if (bpf_helper_call(insn)) {
			const struct bpf_func_proto *fp;

			ret = bpf_get_helper_proto(env, insn->imm, &fp);
			/* If called in a non-sleepable context program will be
			 * rejected anyway, so we should end up with precise
			 * sleepable marks on subprogs, except for dead code
			 * elimination.
			 */
			if (ret == 0 && fp->might_sleep)
				mark_subprog_might_sleep(env, t);
			/* nobody knows yet whether the dynptr is that of an skb */
			if (bpf_helper_changes_pkt_data(insn->imm) ||
			    insn->imm == BPF_FUNC_dynptr_write)
				mark_subprog_changes_pkt_data(env, t);
		} else if (insn->src_reg == BPF_PSEUDO_KFUNC_CALL) {
			struct bpf_call_arg_meta meta;

			ret = bpf_fetch_kfunc_arg_meta(env, insn->imm, insn->off, &meta);
			/* Same as helpers, if called in a non-sleepable context
			 * program will be rejected anyway, so we should end up
			 * with precise sleepable marks on subprogs, except for
			 * dead code elimination.
			 */
			if (ret == 0 && bpf_is_kfunc_sleepable(&meta))
				mark_subprog_might_sleep(env, t);
			if (ret == 0 && bpf_is_kfunc_pkt_changing(&meta))
				mark_subprog_changes_pkt_data(env, t);
			if (ret == 0 && bpf_is_throw_kfunc(insn))
				mark_subprog_might_throw(env, t);
		}
		return visit_func_call_insn(t, insns, env, insn->src_reg == BPF_PSEUDO_CALL);

	case BPF_JA:
		if (BPF_SRC(insn->code) == BPF_X)
			return visit_gotox_insn(t, env);

		if (BPF_CLASS(insn->code) == BPF_JMP)
			off = insn->off;
		else
			off = insn->imm;

		/* unconditional jump with single edge */
		ret = push_insn(t, t + off + 1, FALLTHROUGH, env);
		if (ret)
			return ret;

		mark_jump_target(env, t + off + 1);

		return ret;

	default:
		/* conditional jump with two edges */
		ret = push_insn(t, t + 1, FALLTHROUGH, env);
		if (ret)
			return ret;

		return push_insn(t, t + insn->off + 1, BRANCH, env);
	}
}

/* non-recursive depth-first-search to detect loops in BPF program
 * loop == back-edge in directed graph
 */
int bpf_check_cfg(struct bpf_verifier_env *env)
{
	int insn_cnt = env->prog->len;
	int *insn_stack, *insn_state;
	int ex_insn_beg, i, ret = 0;

	insn_state = env->cfg.insn_state = kvzalloc_objs(int, insn_cnt,
							 GFP_KERNEL_ACCOUNT);
	if (!insn_state)
		return -ENOMEM;

	insn_stack = env->cfg.insn_stack = kvzalloc_objs(int, insn_cnt,
							 GFP_KERNEL_ACCOUNT);
	if (!insn_stack) {
		kvfree(insn_state);
		return -ENOMEM;
	}

	ex_insn_beg = env->exception_callback_subprog
		      ? env->subprog_info[env->exception_callback_subprog].start
		      : 0;

	insn_state[0] = DISCOVERED; /* mark 1st insn as discovered */
	insn_stack[0] = 0; /* 0 is the first instruction */
	env->cfg.cur_stack = 1;

walk_cfg:
	while (env->cfg.cur_stack > 0) {
		int t = insn_stack[env->cfg.cur_stack - 1];

		ret = visit_insn(t, env);
		switch (ret) {
		case DONE_EXPLORING:
			insn_state[t] = EXPLORED;
			env->cfg.cur_stack--;
			break;
		case KEEP_EXPLORING:
			break;
		default:
			if (ret > 0) {
				verifier_bug(env, "visit_insn internal bug");
				ret = -EFAULT;
			}
			goto err_free;
		}
	}

	if (env->cfg.cur_stack < 0) {
		verifier_bug(env, "pop stack internal bug");
		ret = -EFAULT;
		goto err_free;
	}

	if (ex_insn_beg && insn_state[ex_insn_beg] != EXPLORED) {
		insn_state[ex_insn_beg] = DISCOVERED;
		insn_stack[0] = ex_insn_beg;
		env->cfg.cur_stack = 1;
		goto walk_cfg;
	}

	for (i = 0; i < insn_cnt; i++) {
		struct bpf_insn *insn = &env->prog->insnsi[i];

		if (insn_state[i] != EXPLORED) {
			verbose(env, "unreachable insn %d\n", i);
			bpf_diag_program_structure(
				env, i, "unreachable instruction",
				"Remove the unreachable instruction or add valid control flow that reaches it.",
				"Instruction %d is not reachable from the program entry point.", i);
			ret = -EINVAL;
			goto err_free;
		}
		if (bpf_is_ldimm64(insn)) {
			if (insn_state[i + 1] != 0) {
				verbose(env, "jump into the middle of ldimm64 insn %d\n", i);
				bpf_diag_program_structure(
					env, i, "jump into ldimm64 immediate",
					"Target the first instruction of the ldimm64 pair, or restructure the jump target.",
					"Control flow reaches the second half of the ldimm64 instruction pair that starts at instruction %d.",
					i);
				ret = -EINVAL;
				goto err_free;
			}
			i++; /* skip second half of ldimm64 */
		}
	}
	ret = 0; /* cfg looks good */
	close_callee_effects(env);
	env->prog->aux->changes_pkt_data = env->subprog_info[0].changes_pkt_data;
	env->prog->aux->might_sleep = env->subprog_info[0].might_sleep;

err_free:
	kvfree(insn_state);
	kvfree(insn_stack);
	env->cfg.insn_state = env->cfg.insn_stack = NULL;
	return ret;
}

struct dfs_state {
	u32 traversed:1;
	u32 next_succ:31;
};

/*
 * For each subprogram 'i' fill array env->cfg.insn_subprogram sub-range
 * [env->subprog_info[i].postorder_start, env->subprog_info[i+1].postorder_start)
 * with indices of 'i' instructions in postorder.
 */
int bpf_compute_postorder(struct bpf_verifier_env *env)
{
	int *stack = NULL, *postorder = NULL, *postorder_nums = NULL;
	int subprog_idx, stack_sz, cur, s, cur_postorder, start;
	struct dfs_state *state = NULL;
	struct bpf_iarray *succ = NULL;

	postorder_nums = kvzalloc_objs(int, env->prog->len, GFP_KERNEL_ACCOUNT);
	postorder = kvzalloc_objs(int, env->prog->len, GFP_KERNEL_ACCOUNT);
	stack = kvzalloc_objs(int, env->prog->len, GFP_KERNEL_ACCOUNT);
	state = kvzalloc_objs(struct dfs_state, env->prog->len, GFP_KERNEL_ACCOUNT);
	if (!postorder_nums || !postorder || !stack || !state) {
		kvfree(postorder_nums);
		kvfree(postorder);
		kvfree(stack);
		kvfree(state);
		return -ENOMEM;
	}
	cur_postorder = 0;
	for (subprog_idx = 0; subprog_idx < env->subprog_cnt; subprog_idx++) {
		start = env->subprog_info[subprog_idx].start;
		env->subprog_info[subprog_idx].postorder_start = cur_postorder;
		stack[0] = start;
		stack_sz = 1;
		state[start].traversed = true;
		do {
			cur = stack[stack_sz - 1];
			succ = bpf_insn_successors(env, cur);
			if (state[cur].next_succ == succ->cnt) {
				postorder_nums[cur] = cur_postorder;
				postorder[cur_postorder] = cur;
				cur_postorder++;
				stack_sz--;
				continue;
			}
			s = succ->items[state[cur].next_succ];
			if (!state[s].traversed) {
				state[s].traversed = true;
				state[s].next_succ = 0;
				stack[stack_sz] = s;
				stack_sz++;
				continue;
			}
			state[cur].next_succ++;
		} while (stack_sz);
	}
	env->subprog_info[subprog_idx].postorder_start = cur_postorder;
	env->cfg.postorder_nums = postorder_nums;
	env->cfg.insn_postorder = postorder;
	env->cfg.cur_postorder = cur_postorder;
	kvfree(stack);
	kvfree(state);
	return 0;
}
