// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */
#include <linux/bpf.h>
#include <linux/btf.h>
#include <linux/bpf_verifier.h>
#include <linux/filter.h>
#include <linux/vmalloc.h>
#include <linux/perf_event.h>
#include <linux/sched/signal.h>
#include <net/xdp.h>
#include "disasm.h"

#define verbose(env, fmt, args...) bpf_verifier_log_write(env, fmt, ##args)

/*
 * Matches BPF_PROBE_ATOMIC too: bpf_convert_ctx_accesses() rewrites arena
 * atomics before bpf_opt_subreg_zext_lo32_rnd_hi32() runs.
 */
static bool is_cmpxchg_insn(const struct bpf_insn *insn)
{
	return BPF_CLASS(insn->code) == BPF_STX &&
	       (BPF_MODE(insn->code) == BPF_ATOMIC ||
		BPF_MODE(insn->code) == BPF_PROBE_ATOMIC) &&
	       insn->imm == BPF_CMPXCHG;
}

/* Returns true if 'insn' is an address space cast instruction translated as BPF_ALU op */
static bool is_addr_space_cast32(struct bpf_prog *prog, const struct bpf_insn *insn)
{
	struct bpf_map *arena = (struct bpf_map *)prog->aux->arena;

	if (insn->code != (BPF_ALU64 | BPF_MOV | BPF_X) || insn->off != BPF_ADDR_SPACE_CAST)
		return false;

	/* cast from as(1) to as(0) */
	if (insn->imm == 1)
		return true;

	/* cast from as(0) to as(1) */
	if (insn->imm == 1 << 16)
		return arena && arena->map_flags & BPF_F_NO_USER_CONV;

	/* non-BPF_F_NO_USER_CONV cast from as(0) to as(1) should be handled by JIT */
	return false;
}

/* Return the regno defined by the insn, or -1. */
static int insn_def_regno(const struct bpf_insn *insn)
{
	switch (BPF_CLASS(insn->code)) {
	case BPF_JMP:
	case BPF_JMP32:
	case BPF_ST:
		return -1;
	case BPF_STX:
		return bpf_atomic_load_reg(insn);
	default:
		return insn->dst_reg;
	}
}

/*
 * For use only in combination with insn_def_regno() >= 0.
 * Returns TRUE if the destination register operates on 64-bit,
 * otherwise return FALSE.
 */
static bool bpf_is_reg64(struct bpf_prog *prog, struct bpf_insn *insn)
{
	u8 class = BPF_CLASS(insn->code);
	u8 mode = BPF_MODE(insn->code);
	u8 size = BPF_SIZE(insn->code);
	u8 op = BPF_OP(insn->code);
	bool mode_mem;

	/* subregister endiness swap */
	if ((class == BPF_ALU || class == BPF_ALU64) && op == BPF_END && insn->imm != 64)
		return false;

	/* w0 += 1 */
	if (class == BPF_ALU && op != BPF_END)
		return false;

	/* address space casts converted to BPF_ALU, see bpf_do_misc_fixups() */
	if (is_addr_space_cast32(prog, insn))
		return false;

	/* non 64-bit, non signed extended loads */
	mode_mem = mode == BPF_MEM || mode == BPF_PROBE_MEM || mode == BPF_PROBE_MEM32;
	if (class == BPF_LDX && mode_mem && size != BPF_DW)
		return false;

	/* atomics, see insn_def_regno() */
	if (class == BPF_STX && size != BPF_DW)
		return false;

	/* both LD_IND and LD_ABS return 32-bit data. */
	if (class == BPF_LD && (mode == BPF_IND || mode == BPF_ABS))
		return false;

	/* Conservatively return true at default. */
	return true;
}

/*
 * Return the 32-bit subregister defined by INSN, or -1 if INSN does not
 * explicitly define a 32-bit value.
 */
int bpf_insn_def32(struct bpf_prog *prog, struct bpf_insn *insn)
{
	int dst_reg = insn_def_regno(insn);

	if (dst_reg < 0 || bpf_is_reg64(prog, insn))
		return -1;

	return dst_reg;
}

const struct btf_func_model *
bpf_jit_find_kfunc_model(const struct bpf_prog *prog,
			 const struct bpf_insn *insn)
{
	const struct bpf_kfunc_desc *res;
	struct bpf_kfunc_desc_tab *tab;

	tab = prog->aux->kfunc_tab;
	if (insn->off < 0 || insn->off >= tab->nr_descs)
		return NULL;

	res = &tab->descs[insn->off];
	if (bpf_jit_supports_far_kfunc_call()) {
		if (res->func_id != insn->imm)
			return NULL;
	} else if ((s32)BPF_CALL_IMM(res->addr) != insn->imm) {
		return NULL;
	}

	return &res->func_model;
}

static int add_kfunc_in_insns(struct bpf_verifier_env *env,
			      struct bpf_insn *insn, int cnt)
{
	int i, ret;

	for (i = 0; i < cnt; i++, insn++) {
		if (bpf_pseudo_kfunc_call(insn)) {
			ret = bpf_add_kfunc_call(env, insn->imm, insn->off);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

#ifndef CONFIG_BPF_JIT_ALWAYS_ON
static int get_callee_stack_depth(struct bpf_verifier_env *env,
				  const struct bpf_insn *insn, int idx)
{
	int start = idx + insn->imm + 1, subprog;

	subprog = bpf_find_subprog(env, start);
	if (verifier_bug_if(subprog < 0, env, "get stack depth: no program at insn %d", start))
		return -EFAULT;
	return env->subprog_info[subprog].stack_depth;
}
#endif

/*
 * A post-verification rewriting pass can queue a patch for each instruction
 * of a large program. Keep the primitives that queue them killable and
 * preemptible.
 */
static bool bpf_rewrite_must_abort(void)
{
	if (fatal_signal_pending(current))
		return true;
	cond_resched();
	return false;
}

/*
 * Queue the replacement of insn 'off' by 'patch[0..len)' until
 * bpf_patch_list_commit(), see __bpf_patch_list_add(). Replacing one insn by
 * one moves nothing and is done right away in the program, unless 'off' has a
 * queued patch: the commit takes the insn from there.
 */
struct bpf_insn *bpf_patch_list_add(struct bpf_verifier_env *env, u32 off,
				    const struct bpf_insn *patch, u32 len)
{
	struct bpf_patch_list *pl = &env->patch_list;
	struct bpf_insn_patch *p = pl->cnt ? &pl->patches[pl->cnt - 1] : NULL;

	if (bpf_rewrite_must_abort())
		return ERR_PTR(-EINTR);

	if (verifier_bug_if(!len || off >= env->prog->len ||
			    (p && (off < p->off || (off == p->off && !p->len))),
			    env, "insn %u patched out of order", off))
		return ERR_PTR(-EFAULT);

	if (len == 1 && (!p || off != p->off)) {
		struct bpf_insn *insn = &env->prog->insnsi[off];

		*insn = *patch;
		env->insn_aux_data[off].zext_dst = bpf_insn_def32(env->prog, insn) >= 0;
		/* a new insn may access memory, systematically mark it non-stack */
		env->insn_aux_data[off].non_stack_access = bpf_is_mem_insn(insn);
		return insn;
	}

	/* -ERANGE takes a second patch with a 16-bit jump back out of it: no pass has one */
	return __bpf_patch_list_add(pl, env->prog, off, patch, len);
}

/* Queue the removal of insn 'off' until bpf_patch_list_commit(). */
int bpf_patch_list_del(struct bpf_verifier_env *env, u32 off)
{
	struct bpf_patch_list *pl = &env->patch_list;

	if (bpf_rewrite_must_abort())
		return -EINTR;

	if (verifier_bug_if(off >= env->prog->len ||
			    (pl->cnt && off <= pl->patches[pl->cnt - 1].off),
			    env, "insn %u removed out of order", off))
		return -EFAULT;

	return __bpf_patch_list_del(pl, env->prog, off);
}

/* Queue insns to go in front of insn 'off', see __bpf_patch_list_insert(). */
int bpf_patch_list_insert(struct bpf_verifier_env *env, u32 off,
			  const struct bpf_insn *insns, u32 cnt)
{
	struct bpf_patch_list *pl = &env->patch_list;
	struct bpf_insn_patch *p = pl->cnt ? &pl->patches[pl->cnt - 1] : NULL;
	int err;

	if (bpf_rewrite_must_abort())
		return -EINTR;

	if (verifier_bug_if(!cnt || off >= env->prog->len ||
			    (p && (off < p->off || (off == p->off && !p->len))),
			    env, "insns inserted at %u out of order", off))
		return -EFAULT;

	err = __bpf_patch_list_insert(pl, env->prog, off, insns, cnt);
	if (err == -ERANGE)
		verbose(env, "insn %d cannot be patched due to 16-bit range\n",
			env->insn_aux_data[off].orig_idx);
	return err;
}

void bpf_patch_list_free(struct bpf_verifier_env *env)
{
	bpf_patch_list_release(&env->patch_list);
}

/*
 * Fill the aux data of the 'p->len' insns at 'slot' from the aux data 'old'
 * of the insn they replace. The insns in front of p->target were inserted.
 */
static void patch_list_adj_aux(struct bpf_prog *prog, struct bpf_insn_patch *p,
			       struct bpf_insn *patch,
			       const struct bpf_insn_aux_data *old,
			       struct bpf_insn_aux_data *slot)
{
	struct bpf_insn_aux_data *last = slot + p->len - 1;
	struct bpf_insn *orig = &p->orig;
	u32 i;

	*last = *old;
	last->zext_dst = bpf_insn_def32(prog, &patch[p->len - 1]) >= 0;
	memset(slot, 0, sizeof(*slot) * (p->len - 1));

	for (i = 0; i < p->len - 1; i++) {
		slot[i].seen = last->seen;
		/* messages name the insn of the original program */
		slot[i].orig_idx = last->orig_idx;
		slot[i].zext_dst = bpf_insn_def32(prog, &patch[i]) >= 0;
		if (i >= p->target && !memcmp(&patch[i], orig, sizeof(*orig))) {
			slot[i].non_stack_access = last->non_stack_access;
			last->non_stack_access = false;
		} else if (bpf_is_mem_insn(&patch[i])) {
			slot[i].non_stack_access = true;
		}
	}

	if (bpf_is_mem_insn(&patch[p->len - 1]) &&
	    memcmp(&patch[p->len - 1], orig, sizeof(*orig)))
		last->non_stack_access = true;

	/* indirect jumps to the replaced insn land on p->target */
	if (last->indirect_target && p->target != p->len - 1) {
		slot[p->target].indirect_target = 1;
		last->indirect_target = 0;
	}
}

/*
 * Let go of each range of removed insns: tell the device, whose driver finds
 * them through the aux data of the old image, and free what their aux data
 * points to. The commit can't fail anymore, so there is no check for a fatal
 * signal between the calls of the driver.
 */
static void patch_list_drop_removed(struct bpf_verifier_env *env)
{
	bool offloaded = bpf_prog_is_offloaded(env->prog->aux);
	struct bpf_patch_list *pl = &env->patch_list;
	u32 k, n;

	for (k = 0; k < pl->cnt; k = n) {
		for (n = k; n < pl->cnt && !pl->patches[n].len &&
		     pl->patches[n].off == pl->patches[k].off + n - k; n++)
			;
		if (n == k) {
			n++;
			continue;
		}
		if (offloaded)
			bpf_prog_offload_remove_insns(env, pl->patches[k].off, n - k);
		bpf_clear_insn_aux_data(env, pl->patches[k].off, n - k);
	}
}

/* Fill 'data', the aux data of the new image, from the aux data of the old one. */
static void patch_list_remap_aux(struct bpf_verifier_env *env,
				 const struct bpf_patch_image *img,
				 struct bpf_insn_aux_data *data)
{
	struct bpf_patch_list *pl = &env->patch_list;
	struct bpf_insn_patch *p;
	u32 i, k, end;

	for (i = 0, k = 0; i < img->cnt; i = end) {
		if (k < pl->cnt && pl->patches[k].off == i) {
			p = &pl->patches[k++];
			if (p->len)
				patch_list_adj_aux(env->prog, p, &pl->insns[p->start],
						   &env->insn_aux_data[i],
						   &data[img->new_off[i]]);
			end = i + 1;
			continue;
		}
		end = k < pl->cnt ? pl->patches[k].off : img->cnt;
		memcpy(data + img->new_off[i], env->insn_aux_data + i,
		       sizeof(*data) * (end - i));
	}
	vfree(env->insn_aux_data);
	env->insn_aux_data = data;
	env->insn_aux_data_len = img->new_cnt;
}

/*
 * Whether all insns from 'off' to the end of its subprog are removed. 'info'
 * has the 'cnt' subprogs of the old image and the fake 'exit' subprog.
 */
static bool patch_list_subprog_gone(const struct bpf_subprog_info *info, u32 cnt,
				    const u32 *new_off, u32 off)
{
	u32 l = 0, r = cnt, m;

	/* find the next subprog */
	while (l < r) {
		m = (l + r) / 2;
		if (info[m].start <= off)
			l = m + 1;
		else
			r = m;
	}
	return new_off[off] == new_off[info[l].start];
}

/*
 * Remove the subprogs whose insns are all removed, with their func_info. The
 * passes that remove insns run before the hidden subprog, which has no
 * func_info, is added.
 */
static void patch_list_del_subprogs(struct bpf_verifier_env *env,
				    const u32 *new_off)
{
	struct bpf_subprog_info *info = env->subprog_info;
	struct bpf_prog_aux *aux = env->prog->aux;
	int i, j, exc_cb = env->exception_callback_subprog;

	for (i = 0, j = 0; i < env->subprog_cnt; i++) {
		if (new_off[info[i].start] == new_off[info[i + 1].start]) {
			/* The exception callback is live, so it isn't removed. */
			if (env->exception_callback_subprog > i)
				exc_cb--;
			continue;
		}
		if (i != j) {
			info[j] = info[i];
			if (aux->func_info)
				aux->func_info[j] = aux->func_info[i];
			if (aux->func_info_aux)
				aux->func_info_aux[j] = aux->func_info_aux[i];
		}
		j++;
	}
	if (i == j)
		return;

	/* move fake 'exit' subprog as well */
	info[j] = info[i];
	/* func_info->insn_off is set after all code rewrites, in adjust_btf_func() */
	if (aux->func_info)
		aux->func_info_cnt -= i - j;
	env->subprog_cnt = j;
	env->exception_callback_subprog = exc_cb;
}

/*
 * Subprogs, and the function pointers that name them, follow their first insn.
 * A subprog whose insns are all removed goes away, and so do the pointers to
 * it. Only the subprogs tell where a function ends, so move the pointers while
 * the subprogs still have the starts in the old image.
 */
static void patch_list_remap_subprogs(struct bpf_verifier_env *env,
				      const struct bpf_patch_image *img)
{
	struct bpf_subprog_info *info = env->subprog_info;
	u32 i;

	for (i = 0; i < env->func_ptr_cnt; i++) {
		struct bpf_func_ptr *fp = &env->func_ptrs[i];

		if (fp->xlated_off == BPF_FUNC_PTR_DELETED)
			continue;
		if (bpf_patch_image_deleted(img, fp->xlated_off) &&
		    patch_list_subprog_gone(info, env->subprog_cnt, img->new_off,
					    fp->xlated_off))
			fp->xlated_off = BPF_FUNC_PTR_DELETED;
		else
			fp->xlated_off = bpf_patch_image_remap(img, img->new_off,
							       fp->xlated_off);
	}

	if (img->del)
		patch_list_del_subprogs(env, img->new_off);

	/* NOTE: fake 'exit' subprog should be updated as well. */
	for (i = 0; i <= env->subprog_cnt; i++) {
		info[i].start = bpf_patch_image_remap(img, img->new_off, info[i].start);
		if (img->new_linfo)
			info[i].linfo_idx =
				img->new_linfo[min(info[i].linfo_idx, img->nr_linfo)];
	}
}

/*
 * Apply all queued patches and removals in one go. Everything that names an
 * insn of the old image follows it to the new one. A patched insn is named by
 * the first insn of its patch. If insns were inserted in front of an insn,
 * jumps, instruction arrays and poke descriptors name the insn itself, while
 * subprogs, line info, function pointers and calls name the first inserted
 * insn.
 *
 * An entry that names a removed insn:
 *   jump, call, poke descriptor  names the next insn that stays
 *   subprog                      too, or goes away when none of its insns stays
 *   function pointer             follows its subprog or is BPF_FUNC_PTR_DELETED
 *   line info                    goes away, but the next insn that stays takes
 *                                the last record when it has none
 *   aux data                     goes away
 *   instruction array            is INSN_DELETED
 *
 * The order of the steps is a rule:
 * 1. All that can fail: build the new image aside and allocate memory. An
 *    error leaves the program and the env as they were.
 * 2. Let go of the removed insns while all of the env describes the old image:
 *    the helpers for that look at it.
 * 3. Move each table that names insns. The new value of an entry depends on
 *    its old value and on the maps of the image only. Function pointers need
 *    the old subprogs too, see patch_list_remap_subprogs(). Between the first
 *    and the last table one part of the env describes the old image and
 *    another part the new one, so don't call a helper that reads prog->len,
 *    the insns or one of these tables from the env or the program.
 * 4. Install the new image.
 */
int bpf_patch_list_commit(struct bpf_verifier_env *env)
{
	struct bpf_patch_list *pl = &env->patch_list;
	struct bpf_insn_aux_data *data = NULL;
	struct bpf_prog *prog = env->prog;
	struct bpf_patch_image img = {};
	int err = 0;
	u32 i;

	if (!pl->cnt)
		goto out;

	err = bpf_patch_list_build(pl, prog, &img);
	if (err == -ERANGE) {
		verbose(env, "insn %d cannot be patched due to 16-bit range\n",
			env->insn_aux_data[img.bad].orig_idx);
		goto out;
	}
	if (verifier_bug_if(err == -EFAULT, env, "insn %u jumps out of the program",
			    img.bad))
		goto out;
	if (err)
		goto out;

	err = -ENOMEM;
	data = __vmalloc(array_size(img.new_cnt, sizeof(*data)),
			 GFP_KERNEL_ACCOUNT | __GFP_ZERO);
	if (!data)
		goto out;

	prog = bpf_prog_realloc(prog, bpf_prog_size(img.new_cnt), GFP_USER);
	if (!prog)
		goto out;
	env->prog = prog;
	err = 0;

	if (img.del)
		patch_list_drop_removed(env);

	patch_list_remap_aux(env, &img, data);
	data = NULL;
	patch_list_remap_subprogs(env, &img);
	for (i = 0; i < env->insn_array_map_cnt; i++)
		bpf_insn_array_remap(env->insn_array_maps[i], img.new_off,
				     img.new_tgt, img.cnt, img.grow);

	bpf_patch_image_install(prog, &img);
out:
	vfree(data);
	bpf_patch_image_free(&img);
	bpf_patch_list_release(pl);
	return err;
}

/*
 * insn was moved down by delta insns inside its own patch. Operands relative
 * to the pc that point in front of the old position did not move with it.
 */
static int bpf_adj_moved_insn(struct bpf_insn *insn, u32 delta)
{
	u8 class = BPF_CLASS(insn->code), op = BPF_OP(insn->code);
	s64 off = insn->imm, off_min = S32_MIN;
	bool is_imm = true;

	if (bpf_pseudo_func(insn) || bpf_pseudo_call(insn)) {
		/* subprog that started at the old position starts with the patch */
		if (off >= 0)
			return 0;
	} else if ((class == BPF_JMP || class == BPF_JMP32) &&
		   op != BPF_CALL && op != BPF_EXIT) {
		if (insn->code != (BPF_JMP32 | BPF_JA)) {
			off = insn->off;
			off_min = S16_MIN;
			is_imm = false;
		}
		/* jump to itself stays */
		if (off >= -1)
			return 0;
	} else {
		return 0;
	}

	off -= delta;
	if (off < off_min)
		return -ERANGE;
	if (is_imm)
		insn->imm = off;
	else
		insn->off = off;
	return 0;
}

/* Clean up dynamically allocated fields of aux data for instructions [start, ...] */
void bpf_clear_insn_aux_data(struct bpf_verifier_env *env, int start, int len)
{
	struct bpf_insn_aux_data *aux_data = env->insn_aux_data;
	int end = start + len;
	int i;

	for (i = start; i < end; i++) {
		if (aux_data[i].loop) {
			kvfree(aux_data[i].loop->exits);
			kvfree(aux_data[i].loop);
			aux_data[i].loop = NULL;
		}
	}
}

static const struct bpf_insn NOP = BPF_JMP_IMM(BPF_JA, 0, 0, 0);
static const struct bpf_insn MAY_GOTO_0 = BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 0, 0);

bool bpf_insn_is_cond_jump(u8 code)
{
	u8 op;

	op = BPF_OP(code);
	if (BPF_CLASS(code) == BPF_JMP32)
		return op != BPF_JA;

	if (BPF_CLASS(code) != BPF_JMP)
		return false;

	return op != BPF_JA && op != BPF_EXIT && op != BPF_CALL;
}

void bpf_opt_hard_wire_dead_code_branches(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux_data = env->insn_aux_data;
	struct bpf_insn ja = BPF_JMP_IMM(BPF_JA, 0, 0, 0);
	struct bpf_insn trap = BPF_JMP_IMM(BPF_JA, 0, 0, -1);
	struct bpf_insn *insn = env->prog->insnsi;
	const int insn_cnt = env->prog->len;
	int i;

	for (i = 0; i < insn_cnt; i++, insn++) {
		if (!bpf_insn_is_cond_jump(insn->code))
			continue;

		if (bpf_is_unwind_goto(insn) && aux_data[i + 1 + insn->off].seen) {
			/*
			 * The callee never exits. Its unwind_exit needs
			 * unwind_goto all the same, and the cpu predicts that it
			 * returns to what follows. Don't let the removal of dead
			 * code put something else there.
			 */
			if (!aux_data[i + 1].seen) {
				memcpy(insn + 1, &trap, sizeof(trap));
				aux_data[i + 1].seen = env->pass_cnt;
			}
			continue;
		}

		if (!aux_data[i + 1].seen)
			ja.off = insn->off;
		else if (!aux_data[i + 1 + insn->off].seen)
			ja.off = 0;
		else
			continue;

		if (bpf_prog_is_offloaded(env->prog->aux))
			bpf_prog_offload_replace_insn(env, i, &ja);

		memcpy(insn, &ja, sizeof(ja));
	}
}

/*
 * The address of a function might be taken by the code that is not dead,
 * while the function itself is dead code, since nothing calls it. Keep the first
 * instruction of such function, so that the address remains valid and differs
 * from the addresses of other functions. Make it a trap like
 * sanitize_dead_code() does.
 */
static void keep_funcs_with_addr_taken(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux_data = env->insn_aux_data;
	struct bpf_insn trap = BPF_JMP_IMM(BPF_JA, 0, 0, -1);
	struct bpf_insn *insn = env->prog->insnsi;
	int insn_cnt = env->prog->len;
	int i, t;

	for (i = 0; i < insn_cnt; i++) {
		if (!aux_data[i].seen || !bpf_pseudo_func(insn + i))
			continue;
		t = i + insn[i].imm + 1;
		if (aux_data[t].seen)
			continue;
		memcpy(insn + t, &trap, sizeof(trap));
		aux_data[t].zext_dst = false;
		aux_data[t].seen = env->pass_cnt;
	}
}

int bpf_opt_remove_dead_code(struct bpf_verifier_env *env)
{
	struct bpf_insn_aux_data *aux_data = env->insn_aux_data;
	int insn_cnt = env->prog->len;
	int i, err;

	keep_funcs_with_addr_taken(env);

	for (i = 0; i < insn_cnt; i++) {
		if (aux_data[i].seen)
			continue;
		err = bpf_patch_list_del(env, i);
		if (err)
			return err;
	}

	return bpf_patch_list_commit(env);
}

/*
 * Whether insn i is a nop once the insns between it and 'next' are gone: "ja"
 * or may_goto that goes to 'next' at most.
 */
static bool insn_is_nop(struct bpf_verifier_env *env, int i, int next)
{
	struct bpf_insn insn = env->prog->insnsi[i];

	if (env->insn_aux_data[i].indirect_target)
		return false;
	if (insn.off < 0 || i + 1 + insn.off > next)
		return false;
	insn.off = 0;

	return !memcmp(&insn, &NOP, sizeof(NOP)) ||
	       !memcmp(&insn, &MAY_GOTO_0, sizeof(MAY_GOTO_0));
}

int bpf_opt_remove_nops(struct bpf_verifier_env *env)
{
	int insn_cnt = env->prog->len;
	unsigned long *removed;
	int i, next, err = 0;

	removed = kvcalloc(BITS_TO_LONGS(insn_cnt), sizeof(long), GFP_KERNEL_ACCOUNT);
	if (!removed)
		return -ENOMEM;

	/*
	 * Removing may_goto +0 turns the may_goto +1 in front of it into
	 * another one. Only a forward jump can turn into a nop, so walk from
	 * the end: all that insn i jumps over is decided when the walk gets to
	 * it. 'next' is the first insn behind i that stays. For a forward jump
	 * it is in the subprog of i, because a subprog doesn't end with one.
	 */
	for (i = insn_cnt - 1, next = insn_cnt; i >= 0; i--) {
		if (insn_is_nop(env, i, next))
			__set_bit(i, removed);
		else
			next = i;
	}

	for_each_set_bit(i, removed, insn_cnt) {
		err = bpf_patch_list_del(env, i);
		if (err)
			break;
	}
	kvfree(removed);

	return err ?: bpf_patch_list_commit(env);
}

int bpf_opt_subreg_zext_lo32_rnd_hi32(struct bpf_verifier_env *env,
					 const union bpf_attr *attr)
{
	struct bpf_insn *patch;
	/* use env->insn_buf as two independent buffers */
	struct bpf_insn *zext_patch = env->insn_buf;
	struct bpf_insn *rnd_hi32_patch = &env->insn_buf[2];
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	int i, patch_len, len = env->prog->len;
	struct bpf_insn *insns = env->prog->insnsi;
	bool rnd_hi32;

	rnd_hi32 = attr->prog_flags & BPF_F_TEST_RND_HI32;
	zext_patch[1] = BPF_ZEXT_REG(0);
	rnd_hi32_patch[1] = BPF_ALU64_IMM(BPF_MOV, BPF_REG_AX, 0);
	rnd_hi32_patch[2] = BPF_ALU64_IMM(BPF_LSH, BPF_REG_AX, 32);
	rnd_hi32_patch[3] = BPF_ALU64_REG(BPF_OR, 0, BPF_REG_AX);
	for (i = 0; i < len; i++) {
		struct bpf_insn insn;
		int load_reg;

		insn = insns[i];
		load_reg = insn_def_regno(&insn);
		if (!aux[i].zext_dst) {
			u8 code, class;
			u32 imm_rnd;

			if (!rnd_hi32)
				continue;

			code = insn.code;
			class = BPF_CLASS(code);
			if (load_reg == -1)
				continue;

			if (bpf_is_reg64(env->prog, &insn)) {
				if (class == BPF_LD &&
				    BPF_MODE(code) == BPF_IMM)
					i++;
				continue;
			}

			/* ctx load could be transformed into wider load. */
			if (class == BPF_LDX &&
			    aux[i].ptr_type == PTR_TO_CTX)
				continue;

			imm_rnd = get_random_u32();
			rnd_hi32_patch[0] = insn;
			rnd_hi32_patch[1].imm = imm_rnd;
			rnd_hi32_patch[3].dst_reg = load_reg;
			patch = rnd_hi32_patch;
			patch_len = 4;
			goto apply_patch_buffer;
		}

		/* Add in an zero-extend instruction if a) the JIT has requested
		 * it or b) it's a CMPXCHG.
		 *
		 * The latter is because: BPF_CMPXCHG always loads a value into
		 * R0, therefore always zero-extends. However some archs'
		 * equivalent instruction only does this load when the
		 * comparison is successful. This detail of CMPXCHG is
		 * orthogonal to the general zero-extension behaviour of the
		 * CPU, so it's treated independently of bpf_jit_needs_zext.
		 */
		if (!bpf_jit_needs_zext() && !is_cmpxchg_insn(&insn))
			continue;

		/* Zero-extension is done by the caller. */
		if (bpf_pseudo_kfunc_call(&insn))
			continue;

		if (verifier_bug_if(load_reg == -1, env,
				    "zext_dst is set, but no reg is defined"))
			return -EFAULT;

		zext_patch[0] = insn;
		zext_patch[1].dst_reg = load_reg;
		zext_patch[1].src_reg = load_reg;
		patch = zext_patch;
		patch_len = 2;
apply_patch_buffer:
		patch = bpf_patch_list_add(env, i, patch, patch_len);
		if (IS_ERR(patch))
			return PTR_ERR(patch);
	}

	return bpf_patch_list_commit(env);
}

/*
 * The prologue takes the context from R1, and it can clobber R2-R5. An
 * extension gets the arguments of the function that it replaces.
 */
static bool ext_prog_takes_prologue(const struct bpf_verifier_env *env)
{
	const struct bpf_subprog_info *sub = &env->subprog_info[0];
	int i, cnt = min_t(int, sub->arg_slot_cnt, MAX_BPF_FUNC_REG_ARGS);

	if (!cnt || sub->args[0].arg_type != ARG_PTR_TO_CTX)
		return false;

	for (i = 1; i < cnt; i++)
		if (sub->args[i].arg_type != ARG_SCALAR)
			return false;

	return true;
}

/* convert load instructions that access fields of a context type into a
 * sequence of instructions that access fields of the underlying structure:
 *     struct __sk_buff    -> struct sk_buff
 *     struct bpf_sock_ops -> struct sock
 */
/*
 * A load through an __arena pointer member of a kernel struct reads the kernel
 * address of arena memory. Append the conversion to the arena pointer form,
 * (u32)(kaddr - kern_vm_start), that the struct_ops trampoline applies to
 * __arena arguments. A __nullable member keeps NULL as NULL, tested on the
 * full kernel address, and so does a probed load, which reads as 0 when it
 * faults. Returns the number of insns in @buf, 0 when the insn loads no such
 * member, or an error when it does so on some paths only.
 */
static int arena_member_fixup(struct bpf_verifier_env *env, int idx, struct bpf_insn *insn,
			      struct bpf_insn *buf)
{
	struct bpf_insn_aux_data *aux = &env->insn_aux_data[idx];
	int cnt = 0;
	u32 base;

	if (!aux->arena_member)
		return 0;
	if (aux->non_arena_load) {
		verbose(env, "an insn loads an arena pointer member on one path and something else on another\n");
		return -EINVAL;
	}

	base = bpf_arena_get_kern_vm_start(env->prog->aux->arena);
	buf[cnt++] = *insn;
	if (aux->arena_member_nullable || BPF_MODE(insn->code) == BPF_PROBE_MEM)
		buf[cnt++] = BPF_JMP_IMM(BPF_JEQ, insn->dst_reg, 0, 1);
	buf[cnt++] = BPF_ALU32_IMM(BPF_SUB, insn->dst_reg, (s32)base);
	return cnt;
}

int bpf_convert_ctx_accesses(struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *subprogs = env->subprog_info;
	const struct bpf_verifier_ops *ops = env->ops;
	int i, cnt, size, ctx_field_size, ret, epilogue_cnt = 0, first = 0;
	struct bpf_insn *epilogue_buf = env->epilogue_buf;
	struct bpf_insn *insn_buf = env->insn_buf;
	struct bpf_insn *insn;
	u32 target_size, size_default, off;
	enum bpf_access_type type;
	bool is_narrower_load;
	int epilogue_idx = 0;

	if (ops->gen_epilogue) {
		epilogue_cnt = ops->gen_epilogue(epilogue_buf, env->prog,
						 -(subprogs[0].stack_depth + 8));
		if (epilogue_cnt >= INSN_BUF_SIZE) {
			verifier_bug(env, "epilogue is too long");
			return -EFAULT;
		} else if (epilogue_cnt) {
			/* Save the ARG_PTR_TO_CTX for the epilogue to use */
			subprogs[0].stack_depth += 8;
			insn_buf[0] = BPF_STX_MEM(BPF_DW, BPF_REG_FP, BPF_REG_1,
						  -subprogs[0].stack_depth);
			ret = bpf_patch_list_insert(env, 0, insn_buf, 1);
			if (ret)
				return ret;
			first++;

			ret = add_kfunc_in_insns(env, epilogue_buf, epilogue_cnt - 1);
			if (ret < 0)
				return ret;
		}
	}

	if (ops->gen_prologue || env->seen_direct_write) {
		if (!ops->gen_prologue) {
			verifier_bug(env, "gen_prologue is null");
			return -EFAULT;
		}
		cnt = ops->gen_prologue(insn_buf, env->seen_direct_write,
					env->prog);
		if (cnt >= INSN_BUF_SIZE) {
			verifier_bug(env, "prologue is too long");
			return -EFAULT;
		} else if (cnt) {
			if (env->prog->type == BPF_PROG_TYPE_EXT &&
			    !ext_prog_takes_prologue(env)) {
				verbose(env,
					"Extension writing packet data must take ctx and scalars only\n");
				return -EINVAL;
			}
			if (env->prog->type == BPF_PROG_TYPE_EXT &&
			    bpf_check_ext_prologue(&env->log,
						   env->prog->aux->dst_prog,
						   env->prog->aux->attach_btf_id))
				return -EINVAL;
			env->prog->aux->has_prologue = true;
			/* the prologue ends with the first insn of the program */
			if (verifier_bug_if(memcmp(&insn_buf[cnt - 1], env->prog->insnsi,
						   sizeof(*insn_buf)), env,
					    "prologue doesn't end with the first insn"))
				return -EFAULT;
			ret = bpf_patch_list_insert(env, 0, insn_buf, cnt - 1);
			if (ret)
				return ret;
			first += cnt - 1;

			ret = add_kfunc_in_insns(env, insn_buf, cnt - 1);
			if (ret < 0)
				return ret;
		}
	}

	/*
	 * The loop below rewrites some insns in place. No insn may have a queued
	 * patch when the loop gets to it: the list holds a copy of the insn.
	 */
	ret = bpf_patch_list_commit(env);
	if (ret)
		return ret;

	if (bpf_prog_is_offloaded(env->prog->aux))
		return 0;

	for (i = first; i < env->prog->len; i++) {
		bpf_convert_ctx_access_t convert_ctx_access;
		enum bpf_reg_type ptr_type;
		u8 mode;

		insn = &env->prog->insnsi[i];

		if (env->insn_aux_data[i].nospec) {
			WARN_ON_ONCE(env->insn_aux_data[i].alu_state);
			struct bpf_insn *patch = insn_buf;

			*patch++ = BPF_ST_NOSPEC();
			*patch = *insn;
			ret = bpf_adj_moved_insn(patch++, 1);
			if (ret) {
				verbose(env, "insn %d cannot be patched due to 16-bit range\n",
					env->insn_aux_data[i].orig_idx);
				return ret;
			}
			cnt = patch - insn_buf;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			/* This can not be easily merged with the
			 * nospec_result-case, because an insn may require a
			 * nospec before and after itself. Therefore also do not
			 * 'continue' here but potentially apply further
			 * patching to insn. *insn should equal patch[1] now.
			 */
		}

		if (insn->code == (BPF_LDX | BPF_MEM | BPF_B) ||
		    insn->code == (BPF_LDX | BPF_MEM | BPF_H) ||
		    insn->code == (BPF_LDX | BPF_MEM | BPF_W) ||
		    insn->code == (BPF_LDX | BPF_MEM | BPF_DW) ||
		    insn->code == (BPF_LDX | BPF_MEMSX | BPF_B) ||
		    insn->code == (BPF_LDX | BPF_MEMSX | BPF_H) ||
		    insn->code == (BPF_LDX | BPF_MEMSX | BPF_W)) {
			type = BPF_READ;
		} else if (insn->code == (BPF_STX | BPF_MEM | BPF_B) ||
			   insn->code == (BPF_STX | BPF_MEM | BPF_H) ||
			   insn->code == (BPF_STX | BPF_MEM | BPF_W) ||
			   insn->code == (BPF_STX | BPF_MEM | BPF_DW) ||
			   insn->code == (BPF_ST | BPF_MEM | BPF_B) ||
			   insn->code == (BPF_ST | BPF_MEM | BPF_H) ||
			   insn->code == (BPF_ST | BPF_MEM | BPF_W) ||
			   insn->code == (BPF_ST | BPF_MEM | BPF_DW)) {
			type = BPF_WRITE;
		} else if ((insn->code == (BPF_STX | BPF_ATOMIC | BPF_B) ||
			    insn->code == (BPF_STX | BPF_ATOMIC | BPF_H) ||
			    insn->code == (BPF_STX | BPF_ATOMIC | BPF_W) ||
			    insn->code == (BPF_STX | BPF_ATOMIC | BPF_DW)) &&
			   env->insn_aux_data[i].ptr_type == PTR_TO_ARENA) {
			insn->code = BPF_STX | BPF_PROBE_ATOMIC | BPF_SIZE(insn->code);
			env->prog->aux->num_exentries++;
			continue;
		} else if (insn->code == (BPF_STX | BPF_ATOMIC | BPF_DW) &&
			   insn->imm == BPF_LOAD_ACQ &&
			   env->insn_aux_data[i].arena_member) {
			/* an acquire load of an __arena member is rewritten like a plain load */
			cnt = arena_member_fixup(env, i, insn, insn_buf);
			if (cnt < 0)
				return cnt;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			continue;
		} else if (insn->code == (BPF_JMP | BPF_EXIT) &&
			   epilogue_cnt &&
			   i < subprogs[1].start) {
			/* Generate epilogue for the main prog */
			if (epilogue_idx) {
				/* jump back to the earlier generated epilogue */
				insn_buf[0] = BPF_JMP32_A(epilogue_idx - i - 1);
				cnt = 1;
			} else {
				memcpy(insn_buf, epilogue_buf,
				       epilogue_cnt * sizeof(*epilogue_buf));
				cnt = epilogue_cnt;
				/* epilogue_idx cannot be 0. It must have at
				 * least one ctx ptr saving insn before the
				 * epilogue.
				 */
				epilogue_idx = i;
			}
			goto patch_insn_buf;
		} else {
			continue;
		}

		if (type == BPF_WRITE &&
		    env->insn_aux_data[i].nospec_result) {
			/* nospec_result is only used to mitigate Spectre v4 and
			 * to limit verification-time for Spectre v1.
			 */
			struct bpf_insn *patch = insn_buf;

			*patch++ = *insn;
			*patch++ = BPF_ST_NOSPEC();
			cnt = patch - insn_buf;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			continue;
		}

		ptr_type = env->insn_aux_data[i].ptr_type;
		switch ((int)ptr_type) {
		case PTR_TO_CTX:
			if (!ops->convert_ctx_access)
				continue;
			convert_ctx_access = ops->convert_ctx_access;
			break;
		case PTR_TO_SOCKET:
		case PTR_TO_SOCK_COMMON:
			convert_ctx_access = bpf_sock_convert_ctx_access;
			break;
		case PTR_TO_TCP_SOCK:
			convert_ctx_access = bpf_tcp_sock_convert_ctx_access;
			break;
		case PTR_TO_XDP_SOCK:
			convert_ctx_access = bpf_xdp_sock_convert_ctx_access;
			break;
		case PTR_TO_ARENA:
			if (BPF_MODE(insn->code) == BPF_MEMSX) {
				if (!bpf_jit_supports_insn(insn, true)) {
					verbose(env, "sign extending loads from arena are not supported yet\n");
					return -EOPNOTSUPP;
				}
				insn->code = BPF_CLASS(insn->code) | BPF_PROBE_MEM32SX | BPF_SIZE(insn->code);
			} else {
				insn->code = BPF_CLASS(insn->code) | BPF_PROBE_MEM32 | BPF_SIZE(insn->code);
			}
			env->prog->aux->num_exentries++;
			continue;
		default:
			/*
			 * A pointer which may fault on a dereference must not
			 * be loaded from without fault protection, hence turn
			 * the BPF_LDX into a BPF_PROBE_MEM one so that a bad
			 * address is handled rather than panicking the kernel.
			 * A store through one is rejected earlier, there is no
			 * probed counterpart to rewrite it into.
			 */
			if (bpf_is_ptr_to_mem_or_btf_id(ptr_type) &&
			    bpf_may_fault_on_deref(ptr_type) &&
			    type == BPF_READ) {
				if (BPF_MODE(insn->code) == BPF_MEM)
					insn->code = BPF_LDX | BPF_PROBE_MEM |
						     BPF_SIZE(insn->code);
				else
					insn->code = BPF_LDX | BPF_PROBE_MEMSX |
						     BPF_SIZE(insn->code);
				env->prog->aux->num_exentries++;
			} else if (verifier_bug_if(bpf_may_fault_on_deref(ptr_type), env,
						   "access to a fault prone pointer is not rewritten as a probed one")) {
				return -EFAULT;
			}

			cnt = arena_member_fixup(env, i, insn, insn_buf);
			if (cnt < 0)
				return cnt;
			if (!cnt)
				continue;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			continue;
		}

		ctx_field_size = env->insn_aux_data[i].ctx_field_size;
		size = BPF_LDST_BYTES(insn);
		mode = BPF_MODE(insn->code);

		/* If the read access is a narrower load of the field,
		 * convert to a 4/8-byte load, to minimum program type specific
		 * convert_ctx_access changes. If conversion is successful,
		 * we will apply proper mask to the result.
		 */
		is_narrower_load = size < ctx_field_size;
		size_default = bpf_ctx_off_adjust_machine(ctx_field_size);
		off = insn->off;
		if (is_narrower_load) {
			u8 size_code;

			if (type == BPF_WRITE) {
				verifier_bug(env, "narrow ctx access misconfigured");
				return -EFAULT;
			}

			size_code = BPF_H;
			if (ctx_field_size == 4)
				size_code = BPF_W;
			else if (ctx_field_size == 8)
				size_code = BPF_DW;

			insn->off = off & ~(size_default - 1);
			insn->code = BPF_LDX | BPF_MEM | size_code;
		}

		target_size = 0;
		cnt = convert_ctx_access(type, insn, insn_buf, env->prog,
					 &target_size);
		if (cnt == 0 || cnt >= INSN_BUF_SIZE ||
		    (ctx_field_size && !target_size)) {
			verifier_bug(env, "error during ctx access conversion (%d)", cnt);
			return -EFAULT;
		}

		if (is_narrower_load && size < target_size) {
			u8 shift = bpf_ctx_narrow_access_offset(
				off, size, size_default) * 8;
			if (shift && cnt + 1 >= INSN_BUF_SIZE) {
				verifier_bug(env, "narrow ctx load misconfigured");
				return -EFAULT;
			}
			if (ctx_field_size <= 4) {
				if (shift)
					insn_buf[cnt++] = BPF_ALU32_IMM(BPF_RSH,
									insn->dst_reg,
									shift);
				insn_buf[cnt++] = BPF_ALU32_IMM(BPF_AND, insn->dst_reg,
								(1 << size * 8) - 1);
			} else {
				if (shift)
					insn_buf[cnt++] = BPF_ALU64_IMM(BPF_RSH,
									insn->dst_reg,
									shift);
				insn_buf[cnt++] = BPF_ALU32_IMM(BPF_AND, insn->dst_reg,
								(1ULL << size * 8) - 1);
			}
		}
		if (mode == BPF_MEMSX)
			insn_buf[cnt++] = BPF_RAW_INSN(BPF_ALU64 | BPF_MOV | BPF_X,
						       insn->dst_reg, insn->dst_reg,
						       size * 8, 0);

patch_insn_buf:
		insn = bpf_patch_list_add(env, i, insn_buf, cnt);
		if (IS_ERR(insn))
			return PTR_ERR(insn);
	}

	return bpf_patch_list_commit(env);
}

static u32 *bpf_dup_subprog_starts(struct bpf_verifier_env *env)
{
	u32 *starts = NULL;

	starts = kvmalloc_objs(u32, env->subprog_cnt, GFP_KERNEL_ACCOUNT);
	if (starts) {
		for (int i = 0; i < env->subprog_cnt; i++)
			starts[i] = env->subprog_info[i].start;
	}
	return starts;
}

static void bpf_restore_subprog_starts(struct bpf_verifier_env *env, u32 *orig_starts)
{
	for (int i = 0; i < env->subprog_cnt; i++)
		env->subprog_info[i].start = orig_starts[i];
	/* restore the start of fake 'exit' subprog as well */
	env->subprog_info[env->subprog_cnt].start = env->prog->len;
}

/*
 * Replace the offsets of functions with their addresses in the maps of
 * the program, see resolve_func_ptrs() in verifier.c. The program must be
 * the only user of such map. From now on no other program can use it, see
 * bpf_map_claim().
 */
static int resolve_func_ptrs(struct bpf_verifier_env *env, struct bpf_prog *prog,
			     struct bpf_prog **func)
{
	unsigned long me = (unsigned long)prog->aux;
	struct bpf_func_ptr *ptr;
	int i, err, subprog;
	u64 addr, new, *slot;

	for (i = 0; i < env->func_ptr_cnt; i++) {
		ptr = &env->func_ptrs[i];

		/* pointers are sorted by map */
		if ((!i || ptr->map != ptr[-1].map) &&
		    cmpxchg(&ptr->map->user, me, me | BPF_MAP_USER_PATCHED) != me) {
			verbose(env, "map '%s' is used by another program\n", ptr->map->name);
			return -EBUSY;
		}

		/* it's the address of the value of the map whatever the offset is */
		err = ptr->map->ops->map_direct_value_addr(ptr->map, &addr, 0);
		if (verifier_bug_if(err, env, "no value of map '%s'", ptr->map->name))
			return -EFAULT;
		slot = (u64 *)(unsigned long)(addr + ptr->map_off);

		/*
		 * The function that is dead code is removed and the pointer is
		 * NULL. The program may read it, but callx of it is dead code
		 * too, see check_func_callx().
		 */
		new = 0;
		if (ptr->xlated_off != BPF_FUNC_PTR_DELETED) {
			subprog = bpf_find_subprog(env, ptr->xlated_off);
			if (verifier_bug_if(!func || subprog <= 0, env, "no function at insn %u",
					    ptr->xlated_off))
				return -EFAULT;
			new = (unsigned long)func[subprog]->bpf_func;
		}

		/* the map is frozen and nothing else uses it */
		if (verifier_bug_if(*slot != (u64)ptr->orig_off * sizeof(struct bpf_insn), env,
				    "map '%s' offset %u changed", ptr->map->name, ptr->map_off))
			return -EFAULT;
		WRITE_ONCE(*slot, new);
	}
	return 0;
}

static int jit_subprogs(struct bpf_verifier_env *env)
{
	struct bpf_prog *prog = env->prog, **func, *tmp;
	int i, j, subprog_start, subprog_end = 0, len, subprog;
	struct bpf_map *map_ptr;
	struct bpf_insn *insn;
	void *old_bpf_func;
	int err, num_exentries;

	for (i = 0, insn = prog->insnsi; i < prog->len; i++, insn++) {
		if (!bpf_pseudo_func(insn) && !bpf_pseudo_call(insn))
			continue;

		/* Upon error here we cannot fall back to interpreter but
		 * need a hard reject of the program. Thus -EFAULT is
		 * propagated in any case.
		 */
		subprog = bpf_find_subprog(env, i + insn->imm + 1);
		if (verifier_bug_if(subprog < 0, env, "No program to jit at insn %d",
				    i + insn->imm + 1))
			return -EFAULT;
		/* temporarily remember subprog id inside insn instead of
		 * aux_data, since next loop will split up all insns into funcs
		 */
		insn->off = subprog;
		/* remember original imm in case JIT fails and fallback
		 * to interpreter will be needed
		 */
		env->insn_aux_data[i].call_imm = insn->imm;
		/* point imm to __bpf_call_base+1 from JITs point of view */
		insn->imm = 1;
		if (bpf_pseudo_func(insn)) {
#if defined(MODULES_VADDR)
			u64 addr = MODULES_VADDR;
#else
			u64 addr = VMALLOC_START;
#endif
			/* jit (e.g. x86_64) may emit fewer instructions
			 * if it learns a u32 imm is the same as a u64 imm.
			 * Set close enough to possible prog address.
			 */
			insn[0].imm = (u32)addr;
			insn[1].imm = addr >> 32;
		}
	}

	err = bpf_prog_alloc_jited_linfo(prog);
	if (err)
		goto out_undo_insn;

	err = -ENOMEM;
	func = kzalloc_objs(prog, env->subprog_cnt);
	if (!func)
		goto out_undo_insn;

	for (i = 0; i < env->subprog_cnt; i++) {
		subprog_start = subprog_end;
		subprog_end = env->subprog_info[i + 1].start;

		len = subprog_end - subprog_start;
		/* bpf_prog_run() doesn't call subprogs directly,
		 * hence main prog stats include the runtime of subprogs.
		 * subprogs don't have IDs and not reachable via prog_get_next_id
		 * func[i]->stats will never be accessed and stays NULL
		 */
		func[i] = bpf_prog_alloc_no_stats(bpf_prog_size(len), GFP_USER);
		if (!func[i])
			goto out_free;
		memcpy(func[i]->insnsi, &prog->insnsi[subprog_start],
		       len * sizeof(struct bpf_insn));
		func[i]->type = prog->type;
		func[i]->len = len;
		if (bpf_prog_calc_tag(func[i]))
			goto out_free;
		func[i]->is_func = 1;
		func[i]->sleepable = prog->sleepable;
		func[i]->blinded = prog->blinded;
		func[i]->aux->func_idx = i;
		/* Below members will be freed only at prog->aux */
		func[i]->aux->btf = prog->aux->btf;
		func[i]->aux->subprog_start = subprog_start;
		func[i]->aux->func_info = prog->aux->func_info;
		func[i]->aux->func_info_cnt = prog->aux->func_info_cnt;
		func[i]->aux->poke_tab = prog->aux->poke_tab;
		func[i]->aux->size_poke_tab = prog->aux->size_poke_tab;
		func[i]->aux->main_prog_aux = prog->aux;

		for (j = 0; j < prog->aux->size_poke_tab; j++) {
			struct bpf_jit_poke_descriptor *poke;

			poke = &prog->aux->poke_tab[j];
			if (poke->insn_idx < subprog_end &&
			    poke->insn_idx >= subprog_start)
				poke->aux = func[i]->aux;
		}

		func[i]->aux->name[0] = 'F';
		func[i]->aux->stack_depth = env->subprog_info[i].stack_depth;
		if (env->subprog_info[i].priv_stack_mode == PRIV_STACK_ADAPTIVE)
			func[i]->aux->jits_use_priv_stack = true;

		func[i]->jit_requested = 1;
		func[i]->blinding_requested = prog->blinding_requested;
		func[i]->aux->kfunc_tab = prog->aux->kfunc_tab;
		func[i]->aux->kfunc_btf_tab = prog->aux->kfunc_btf_tab;
		func[i]->aux->linfo = prog->aux->linfo;
		func[i]->aux->nr_linfo = prog->aux->nr_linfo;
		func[i]->aux->jited_linfo = prog->aux->jited_linfo;
		func[i]->aux->linfo_idx = env->subprog_info[i].linfo_idx;
		func[i]->aux->arena = prog->aux->arena;
		func[i]->aux->uses_arena_insns = bpf_prog_has_arena_insns(func[i]);
		func[i]->aux->used_maps = env->used_maps;
		func[i]->aux->used_map_cnt = env->used_map_cnt;
		num_exentries = 0;
		insn = func[i]->insnsi;
		for (j = 0; j < func[i]->len; j++, insn++) {
			if (BPF_CLASS(insn->code) == BPF_LDX &&
			    (BPF_MODE(insn->code) == BPF_PROBE_MEM ||
			     BPF_MODE(insn->code) == BPF_PROBE_MEM32 ||
			     BPF_MODE(insn->code) == BPF_PROBE_MEM32SX ||
			     BPF_MODE(insn->code) == BPF_PROBE_MEMSX))
				num_exentries++;
			if ((BPF_CLASS(insn->code) == BPF_STX ||
			     BPF_CLASS(insn->code) == BPF_ST) &&
			     BPF_MODE(insn->code) == BPF_PROBE_MEM32)
				num_exentries++;
			if (BPF_CLASS(insn->code) == BPF_STX &&
			     BPF_MODE(insn->code) == BPF_PROBE_ATOMIC)
				num_exentries++;
			if (bpf_is_unwind_goto(insn))
				num_exentries++;
		}
		func[i]->aux->num_exentries = num_exentries;
		func[i]->aux->tail_call_reachable = env->subprog_info[i].tail_call_reachable;
		func[i]->aux->exception_cb = env->subprog_info[i].is_exception_cb;
		func[i]->aux->changes_pkt_data = env->subprog_info[i].changes_pkt_data;
		func[i]->aux->might_sleep = env->subprog_info[i].might_sleep;
		func[i]->aux->can_unwind = env->subprog_info[i].can_unwind;
		func[i]->aux->token = prog->aux->token;
		if (!i)
			func[i]->aux->exception_boundary = env->seen_exception;
		func[i] = bpf_int_jit_compile(env, func[i]);
		if (!func[i]->jited) {
			err = -ENOTSUPP;
			goto out_free;
		}
		cond_resched();
	}

	/* at this point all bpf functions were successfully JITed
	 * now populate all bpf_calls with correct addresses and
	 * run last pass of JIT
	 */
	for (i = 0; i < env->subprog_cnt; i++) {
		insn = func[i]->insnsi;
		for (j = 0; j < func[i]->len; j++, insn++) {
			if (bpf_pseudo_func(insn)) {
				subprog = insn->off;
				insn[0].imm = (u32)(long)func[subprog]->bpf_func;
				insn[1].imm = ((u64)(long)func[subprog]->bpf_func) >> 32;
				continue;
			}
			if (!bpf_pseudo_call(insn))
				continue;
			subprog = insn->off;
			insn->imm = BPF_CALL_IMM(func[subprog]->bpf_func);
		}

		/* we use the aux data to keep a list of the start addresses
		 * of the JITed images for each function in the program
		 *
		 * for some architectures, such as powerpc64, the imm field
		 * might not be large enough to hold the offset of the start
		 * address of the callee's JITed image from __bpf_call_base
		 *
		 * in such cases, we can lookup the start address of a callee
		 * by using its subprog id, available from the off field of
		 * the call instruction, as an index for this list
		 */
		func[i]->aux->func = func;
		func[i]->aux->func_cnt = env->subprog_cnt - env->hidden_subprog_cnt;
		func[i]->aux->real_func_cnt = env->subprog_cnt;
	}
	for (i = 0; i < env->subprog_cnt; i++) {
		old_bpf_func = func[i]->bpf_func;
		tmp = bpf_int_jit_compile(env, func[i]);
		if (tmp != func[i] || func[i]->bpf_func != old_bpf_func) {
			verbose(env, "JIT doesn't support bpf-to-bpf calls\n");
			err = -ENOTSUPP;
			goto out_free;
		}
		cond_resched();
	}

	/* the addresses of all functions are final */
	err = resolve_func_ptrs(env, prog, func);
	if (err)
		goto out_free;

	/*
	 * Cleanup func[i]->aux fields which aren't required
	 * or can become invalid in future
	 */
	for (i = 0; i < env->subprog_cnt; i++) {
		func[i]->aux->used_maps = NULL;
		func[i]->aux->used_map_cnt = 0;
	}

	/* finally lock prog and jit images for all functions and
	 * populate kallsysm. Begin at the first subprogram, since
	 * bpf_prog_load will add the kallsyms for the main program.
	 */
	for (i = 1; i < env->subprog_cnt; i++) {
		err = bpf_prog_lock_ro(func[i]);
		if (err)
			goto out_free;
	}

	for (i = 1; i < env->subprog_cnt; i++)
		bpf_prog_kallsyms_add(func[i]);

	/* Last step: make now unused interpreter insns from main
	 * prog consistent for later dump requests, so they can
	 * later look the same as if they were interpreted only.
	 */
	for (i = 0, insn = prog->insnsi; i < prog->len; i++, insn++) {
		if (bpf_pseudo_func(insn)) {
			insn[0].imm = env->insn_aux_data[i].call_imm;
			insn[1].imm = insn->off;
			insn->off = 0;
			continue;
		}
		if (!bpf_pseudo_call(insn))
			continue;
		insn->imm = env->insn_aux_data[i].call_imm;
		subprog = bpf_find_subprog(env, i + insn->imm + 1);
		insn->off = subprog;
	}

	prog->jited = 1;
	prog->bpf_func = func[0]->bpf_func;
	prog->jited_len = func[0]->jited_len;
	prog->aux->extable = func[0]->aux->extable;
	prog->aux->num_exentries = func[0]->aux->num_exentries;
	prog->aux->func = func;
	prog->aux->func_cnt = env->subprog_cnt - env->hidden_subprog_cnt;
	prog->aux->real_func_cnt = env->subprog_cnt;
	prog->aux->bpf_exception_cb = (void *)func[env->exception_callback_subprog]->bpf_func;
	prog->aux->exception_boundary = func[0]->aux->exception_boundary;
	prog->aux->stack_arg_sp_adjust = func[0]->aux->stack_arg_sp_adjust;
	bpf_prog_jit_attempt_done(prog);
	return 0;
out_free:
	/* We failed JIT'ing, so at this point we need to unregister poke
	 * descriptors from subprogs, so that kernel is not attempting to
	 * patch it anymore as we're freeing the subprog JIT memory.
	 */
	for (i = 0; i < prog->aux->size_poke_tab; i++) {
		map_ptr = prog->aux->poke_tab[i].tail_call.map;
		map_ptr->ops->map_poke_untrack(map_ptr, prog->aux);
	}
	/* At this point we're guaranteed that poke descriptors are not
	 * live anymore. We can just unlink its descriptor table as it's
	 * released with the main prog.
	 */
	for (i = 0; i < env->subprog_cnt; i++) {
		if (!func[i])
			continue;
		func[i]->aux->poke_tab = NULL;
		bpf_jit_free(func[i]);
	}
	kfree(func);
out_undo_insn:
	bpf_prog_jit_attempt_done(prog);
	return err;
}

int bpf_jit_subprogs(struct bpf_verifier_env *env)
{
	int err, i;
	bool blinded = false;
	struct bpf_insn *insn;
	struct bpf_prog *prog, *orig_prog;
	u32 *orig_subprog_starts;

	/* all functions that the maps point to are removed as dead code */
	if (env->subprog_cnt <= 1)
		return resolve_func_ptrs(env, env->prog, NULL);

	prog = orig_prog = env->prog;
	if (bpf_prog_need_blind(prog)) {
		orig_subprog_starts = bpf_dup_subprog_starts(env);
		if (!orig_subprog_starts) {
			err = -ENOMEM;
			goto out_cleanup;
		}
		prog = bpf_jit_blind_constants(env, prog);
		if (IS_ERR(prog)) {
			err = PTR_ERR(prog);
			prog = orig_prog;
			goto out_restore;
		}
		blinded = true;
	}

	err = jit_subprogs(env);
	if (err)
		goto out_jit_err;

	if (blinded) {
		bpf_jit_prog_release_other(prog, orig_prog);
		kvfree(orig_subprog_starts);
	}

	return 0;

out_jit_err:
	if (blinded) {
		bpf_jit_prog_release_other(orig_prog, prog);
		/* roll back to the clean original prog */
		prog = env->prog = orig_prog;
		goto out_restore;
	} else {
		if (err != -EFAULT) {
			/*
			 * We will fall back to interpreter mode when err is not -EFAULT, before
			 * that, insn->off and insn->imm should be restored to their original
			 * values since they were modified by jit_subprogs.
			 */
			for (i = 0, insn = prog->insnsi; i < prog->len; i++, insn++) {
				if (!bpf_pseudo_call(insn))
					continue;
				insn->off = 0;
				insn->imm = env->insn_aux_data[i].call_imm;
			}
		}
		goto out_cleanup;
	}

out_restore:
	bpf_restore_subprog_starts(env, orig_subprog_starts);
	kvfree(orig_subprog_starts);
out_cleanup:
	/* cleanup main prog to be interpreted */
	prog->jit_requested = 0;
	prog->blinding_requested = 0;
	return err;
}

int bpf_fixup_call_args(struct bpf_verifier_env *env)
{
#ifndef CONFIG_BPF_JIT_ALWAYS_ON
	struct bpf_prog *prog = env->prog;
	struct bpf_insn *insn = prog->insnsi;
	int depth;
#endif
	int i, err = 0;

	for (i = 0; i < env->subprog_cnt; i++) {
		struct bpf_subprog_info *subprog = &env->subprog_info[i];
		u16 outgoing = subprog->stack_arg_cnt - bpf_in_stack_arg_cnt(subprog);

		if (subprog->max_out_stack_arg_cnt > outgoing) {
			verbose(env,
				"func#%d writes %u stack arg slots, but calls only require %u\n",
				i, subprog->max_out_stack_arg_cnt, outgoing);
			return -EINVAL;
		}
	}

	if (env->prog->jit_requested &&
	    !bpf_prog_is_offloaded(env->prog->aux)) {
		err = bpf_jit_subprogs(env);
		if (err == 0)
			return 0;
		if (err == -EFAULT || err == -EINTR)
			return err;
	}
#ifndef CONFIG_BPF_JIT_ALWAYS_ON
	if (prog->jit_required) {
		verbose(env, "program requires BPF JIT compiler but it is not available\n");
		return -EINVAL;
	}
	for (i = 0; i < env->subprog_cnt; i++) {
		if (bpf_in_stack_arg_cnt(&env->subprog_info[i])) {
			verbose(env, "stack args are not supported in non-JITed programs\n");
			return -EINVAL;
		}
	}
	if (env->subprog_cnt > 1 && env->prog->aux->tail_call_reachable) {
		/* When JIT fails the progs with bpf2bpf calls and tail_calls
		 * have to be rejected, since interpreter doesn't support them yet.
		 */
		verbose(env, "tail_calls are not allowed in non-JITed programs with bpf-to-bpf calls\n");
		return -EINVAL;
	}
	for (i = 0; i < prog->len; i++, insn++) {
		if (bpf_pseudo_func(insn)) {
			/* When JIT fails the progs with callback calls
			 * have to be rejected, since interpreter doesn't support them yet.
			 */
			verbose(env, "callbacks are not allowed in non-JITed programs\n");
			return -EINVAL;
		}

		if (!bpf_pseudo_call(insn))
			continue;
		depth = get_callee_stack_depth(env, insn, i);
		if (depth < 0)
			return depth;
		err = bpf_patch_call_args(insn, depth);
		if (err) {
			verbose(env, "stack depth %d exceeds interpreter stack depth limit\n",
				depth);
			return err;
		}
	}
	err = 0;
#endif
	return err;
}

/* The function requires that first instruction in 'patch' is insnsi[prog->len - 1] */
static int add_hidden_subprog(struct bpf_verifier_env *env, struct bpf_insn *patch, int len)
{
	struct bpf_subprog_info *info = env->subprog_info;
	int err, cnt = env->subprog_cnt;
	struct bpf_insn *last;
	struct bpf_prog *prog;

	/* We only reserve one slot for hidden subprogs in subprog_info. */
	if (env->hidden_subprog_cnt) {
		verifier_bug(env, "only one hidden subprog supported");
		return -EFAULT;
	}
	/*
	 * We're not patching any existing instruction, just appending the new
	 * ones for the hidden subprog.
	 */
	last = bpf_patch_list_add(env, env->prog->len - 1, patch, len);
	if (IS_ERR(last))
		return PTR_ERR(last);
	err = bpf_patch_list_commit(env);
	if (err)
		return err;
	prog = env->prog;
	info[cnt + 1].start = info[cnt].start;
	info[cnt].start = prog->len - len + 1;
	/* The patch ends with the exit. */
	info[cnt].exit_idx = prog->len - 1;
	env->subprog_cnt++;
	env->hidden_subprog_cnt++;
	return 0;
}

/*
 * Expand may_goto: load the count from the stack, jump to the target of
 * may_goto when it is zero, then the tail that updates the count.
 * 'off' is the offset of may_goto, 'reach' how far it jumps once the patches
 * queued so far are applied, see may_goto_off().
 * Use gotol when 'reach' is too far for 16-bit offset of a conditional jump.
 */
static int may_goto_expand(struct bpf_insn *insn_buf, int off, int reach,
			   int stack_off, const struct bpf_insn *tail, int tail_cnt)
{
	int cnt = 0;

	/* Forward jump has to step over the tail */
	off = off >= 0 ? off + tail_cnt : off - 1;
	reach = reach >= 0 ? reach + tail_cnt : reach - 1;

	insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_AX, BPF_REG_10, stack_off);
	if (reach == (s16)reach) {
		insn_buf[cnt++] = BPF_JMP_IMM(BPF_JEQ, BPF_REG_AX, 0, off);
	} else {
		insn_buf[cnt++] = BPF_JMP_IMM(BPF_JNE, BPF_REG_AX, 0, 1);
		insn_buf[cnt++] = BPF_JMP32_A(off >= 0 ? off : off - 1);
	}
	memcpy(insn_buf + cnt, tail, tail_cnt * sizeof(*tail));
	return cnt + tail_cnt;
}

/*
 * Timed may_goto. All may_goto of a prog use one count and one timestamp: per-cpu memory of the
 * prog, set in the first insns of the main prog, and by the callers of async callbacks.
 * @reg holds the address. It is a register that is dead at the may_goto, or R0 that is saved at
 * @save_off in the stack. See may_goto_expand() for @off and @reach.
 */
static int may_goto_expand_timed(struct bpf_insn *insn_buf, int off, int reach,
				 struct bpf_timed_may_goto __percpu *tmg, int reg, int save_off)
{
	struct bpf_insn ld_tmg[] = { BPF_LD_IMM64(reg, (__force unsigned long)tmg) };
	int cnt = 0, jmp, tail_cnt = save_off ? 6 : 5;

	if (save_off)
		insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, BPF_REG_10, reg, save_off);
	insn_buf[cnt++] = ld_tmg[0];
	insn_buf[cnt++] = ld_tmg[1];
	insn_buf[cnt++] = BPF_MOV64_PERCPU_REG(reg, reg);
	insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_AX, reg, 0);
	if (save_off) {
		insn_buf[cnt++] = BPF_JMP_IMM(BPF_JNE, BPF_REG_AX, 0, 2);
		insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, reg, BPF_REG_10, save_off);
		jmp = cnt++;
		insn_buf[jmp] = BPF_JMP32_A(off >= 0 ? off + tail_cnt : off - jmp);
	} else {
		jmp = cnt;
		off = off >= 0 ? off + tail_cnt : off - jmp;
		reach = reach >= 0 ? reach + tail_cnt : reach - jmp;
		if (reach == (s16)reach) {
			insn_buf[cnt++] = BPF_JMP_IMM(BPF_JEQ, BPF_REG_AX, 0, off);
		} else {
			insn_buf[cnt++] = BPF_JMP_IMM(BPF_JNE, BPF_REG_AX, 0, 1);
			insn_buf[cnt++] = BPF_JMP32_A(off >= 0 ? off : off - 1);
		}
	}
	insn_buf[cnt++] = BPF_ALU64_IMM(BPF_SUB, BPF_REG_AX, 1);
	insn_buf[cnt++] = BPF_JMP_IMM(BPF_JNE, BPF_REG_AX, 0, 2);
	/* AX is the argument and the return value */
	insn_buf[cnt++] = BPF_MOV64_REG(BPF_REG_AX, reg);
	insn_buf[cnt++] = BPF_EMIT_CALL(arch_bpf_timed_may_goto);
	insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, reg, BPF_REG_AX, 0);
	if (save_off)
		insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, reg, BPF_REG_10, save_off);
	return cnt;
}

/*
 * The offset of may_goto at 'idx' once the patches queued in front are applied.
 * The patches that a forward may_goto jumps over aren't queued yet, so its
 * offset stays as it is.
 */
static int may_goto_off(struct bpf_verifier_env *env, const struct bpf_insn *insn,
			u32 idx)
{
	struct bpf_patch_list *pl = &env->patch_list;
	u32 tgt = idx + insn->off + 1;
	int off = insn->off;
	u32 k = pl->cnt;

	if (off >= 0)
		return off;
	/* all queued patches are in front of idx */
	while (k > 0 && pl->patches[k - 1].off >= tgt)
		off -= pl->patches[--k].len - 1;
	return off;
}

/* Do various post-verification rewrites in a single program pass.
 * These rewrites simplify JIT and interpreter implementations.
 */
int bpf_do_misc_fixups(struct bpf_verifier_env *env)
{
	struct bpf_prog *prog = env->prog;
	enum bpf_attach_type eatype = prog->expected_attach_type;
	enum bpf_prog_type prog_type = resolve_prog_type(prog);
	struct bpf_insn *insn = prog->insnsi;
	const struct bpf_func_proto *fn;
	const int insn_cnt = prog->len;
	const struct bpf_map_ops *ops;
	struct bpf_insn_aux_data *aux;
	struct bpf_insn *insn_buf = env->insn_buf;
	struct bpf_map *map_ptr;
	int i, ret, cnt, cur_subprog = 0;
	struct bpf_subprog_info *subprogs = env->subprog_info;
	u16 stack_depth = subprogs[cur_subprog].stack_depth;
	u16 stack_depth_extra = 0;
	/* arch_bpf_timed_may_goto() takes and returns AX in a cpu register: for JITs only */
	bool timed_may_goto = prog->jit_requested && bpf_jit_supports_timed_may_goto();
	struct bpf_timed_may_goto __percpu *tmg = NULL;

	if (env->seen_exception && !env->exception_callback_subprog) {
		struct bpf_insn *patch = insn_buf;

		*patch++ = env->prog->insnsi[insn_cnt - 1];
		*patch++ = BPF_MOV64_REG(BPF_REG_0, BPF_REG_1);
		*patch++ = BPF_EXIT_INSN();
		ret = add_hidden_subprog(env, insn_buf, patch - insn_buf);
		if (ret < 0)
			return ret;
		prog = env->prog;
		insn = prog->insnsi;

		env->exception_callback_subprog = env->subprog_cnt - 1;
		/* Don't update insn_cnt, as add_hidden_subprog always appends insns */
		bpf_mark_subprog_exc_cb(env, env->exception_callback_subprog);
	}

	for (i = 0; i < insn_cnt;) {
		if (is_addr_space_cast32(env->prog, insn)) {
			/* convert to 32-bit mov that clears upper 32-bit */
			insn->code = BPF_ALU | BPF_MOV | BPF_X;
			/* clear off and imm, so it's a normal 'wX = wY' from JIT pov */
			insn->off = 0;
			insn->imm = 0;
			goto next_insn;
		}

		if (env->insn_aux_data[i].needs_zext)
			/* Convert BPF_CLASS(insn->code) == BPF_ALU64 to 32-bit ALU */
			insn->code = BPF_ALU | BPF_OP(insn->code) | BPF_SRC(insn->code);

		/* Make sdiv/smod divide-by-minus-one exceptions impossible. */
		if ((insn->code == (BPF_ALU64 | BPF_MOD | BPF_K) ||
		     insn->code == (BPF_ALU64 | BPF_DIV | BPF_K) ||
		     insn->code == (BPF_ALU | BPF_MOD | BPF_K) ||
		     insn->code == (BPF_ALU | BPF_DIV | BPF_K)) &&
		    insn->off == 1 && insn->imm == -1) {
			bool is64 = BPF_CLASS(insn->code) == BPF_ALU64;
			bool isdiv = BPF_OP(insn->code) == BPF_DIV;
			struct bpf_insn *patch = insn_buf;

			if (isdiv)
				*patch++ = BPF_RAW_INSN((is64 ? BPF_ALU64 : BPF_ALU) |
							BPF_NEG | BPF_K, insn->dst_reg,
							0, 0, 0);
			else
				*patch++ = BPF_MOV32_IMM(insn->dst_reg, 0);

			cnt = patch - insn_buf;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Make divide-by-zero and divide-by-minus-one exceptions impossible. */
		if (insn->code == (BPF_ALU64 | BPF_MOD | BPF_X) ||
		    insn->code == (BPF_ALU64 | BPF_DIV | BPF_X) ||
		    insn->code == (BPF_ALU | BPF_MOD | BPF_X) ||
		    insn->code == (BPF_ALU | BPF_DIV | BPF_X)) {
			bool is64 = BPF_CLASS(insn->code) == BPF_ALU64;
			bool isdiv = BPF_OP(insn->code) == BPF_DIV;
			bool is_sdiv = isdiv && insn->off == 1;
			bool is_smod = !isdiv && insn->off == 1;
			struct bpf_insn *patch = insn_buf;

			if (is_sdiv) {
				/* [R,W]x sdiv 0 -> 0
				 * LLONG_MIN sdiv -1 -> LLONG_MIN
				 * INT_MIN sdiv -1 -> INT_MIN
				 */
				*patch++ = BPF_MOV64_REG(BPF_REG_AX, insn->src_reg);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_ALU64 : BPF_ALU) |
							BPF_ADD | BPF_K, BPF_REG_AX,
							0, 0, 1);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JGT | BPF_K, BPF_REG_AX,
							0, 4, 1);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JEQ | BPF_K, BPF_REG_AX,
							0, 1, 0);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_ALU64 : BPF_ALU) |
							BPF_MOV | BPF_K, insn->dst_reg,
							0, 0, 0);
				/* BPF_NEG(LLONG_MIN) == -LLONG_MIN == LLONG_MIN */
				*patch++ = BPF_RAW_INSN((is64 ? BPF_ALU64 : BPF_ALU) |
							BPF_NEG | BPF_K, insn->dst_reg,
							0, 0, 0);
				*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
				*patch++ = *insn;
				cnt = patch - insn_buf;
			} else if (is_smod) {
				/* [R,W]x mod 0 -> [R,W]x */
				/* [R,W]x mod -1 -> 0 */
				*patch++ = BPF_MOV64_REG(BPF_REG_AX, insn->src_reg);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_ALU64 : BPF_ALU) |
							BPF_ADD | BPF_K, BPF_REG_AX,
							0, 0, 1);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JGT | BPF_K, BPF_REG_AX,
							0, 3, 1);
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JEQ | BPF_K, BPF_REG_AX,
							0, 3 + (is64 ? 0 : 1), 1);
				*patch++ = BPF_MOV32_IMM(insn->dst_reg, 0);
				*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
				*patch++ = *insn;

				if (!is64) {
					*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
					*patch++ = BPF_MOV32_REG(insn->dst_reg, insn->dst_reg);
				}
				cnt = patch - insn_buf;
			} else if (isdiv) {
				/* [R,W]x div 0 -> 0 */
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JNE | BPF_K, insn->src_reg,
							0, 2, 0);
				*patch++ = BPF_ALU32_REG(BPF_XOR, insn->dst_reg, insn->dst_reg);
				*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
				*patch++ = *insn;
				cnt = patch - insn_buf;
			} else {
				/* [R,W]x mod 0 -> [R,W]x */
				*patch++ = BPF_RAW_INSN((is64 ? BPF_JMP : BPF_JMP32) |
							BPF_JEQ | BPF_K, insn->src_reg,
							0, 1 + (is64 ? 0 : 1), 0);
				*patch++ = *insn;

				if (!is64) {
					*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
					*patch++ = BPF_MOV32_REG(insn->dst_reg, insn->dst_reg);
				}
				cnt = patch - insn_buf;
			}

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Make it impossible to de-reference a userspace address */
		if (BPF_CLASS(insn->code) == BPF_LDX &&
		    (BPF_MODE(insn->code) == BPF_PROBE_MEM ||
		     BPF_MODE(insn->code) == BPF_PROBE_MEMSX)) {
			struct bpf_insn *patch = insn_buf;
			u64 uaddress_limit = bpf_arch_uaddress_limit();

			if (!uaddress_limit)
				goto next_insn;

			*patch++ = BPF_MOV64_REG(BPF_REG_AX, insn->src_reg);
			if (insn->off)
				*patch++ = BPF_ALU64_IMM(BPF_ADD, BPF_REG_AX, insn->off);
			*patch++ = BPF_ALU64_IMM(BPF_RSH, BPF_REG_AX, 32);
			*patch++ = BPF_JMP_IMM(BPF_JLE, BPF_REG_AX, uaddress_limit >> 32, 2);
			*patch++ = *insn;
			*patch++ = BPF_JMP_IMM(BPF_JA, 0, 0, 1);
			*patch++ = BPF_MOV64_IMM(insn->dst_reg, 0);

			cnt = patch - insn_buf;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement LD_ABS and LD_IND with a rewrite, if supported by the program type. */
		if (BPF_CLASS(insn->code) == BPF_LD &&
		    (BPF_MODE(insn->code) == BPF_ABS ||
		     BPF_MODE(insn->code) == BPF_IND)) {
			cnt = env->ops->gen_ld_abs(insn, insn_buf);
			if (cnt == 0 || cnt >= INSN_BUF_SIZE) {
				verifier_bug(env, "%d insns generated for ld_abs", cnt);
				return -EFAULT;
			}

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Rewrite pointer arithmetic to mitigate speculation attacks. */
		if (insn->code == (BPF_ALU64 | BPF_ADD | BPF_X) ||
		    insn->code == (BPF_ALU64 | BPF_SUB | BPF_X)) {
			const u8 code_add = BPF_ALU64 | BPF_ADD | BPF_X;
			const u8 code_sub = BPF_ALU64 | BPF_SUB | BPF_X;
			struct bpf_insn *patch = insn_buf;
			bool issrc, isneg, isimm;
			u32 off_reg;

			aux = &env->insn_aux_data[i];
			if (!aux->alu_state ||
			    aux->alu_state == BPF_ALU_NON_POINTER)
				goto next_insn;

			isneg = aux->alu_state & BPF_ALU_NEG_VALUE;
			issrc = (aux->alu_state & BPF_ALU_SANITIZE) ==
				BPF_ALU_SANITIZE_SRC;
			isimm = aux->alu_state & BPF_ALU_IMMEDIATE;

			off_reg = issrc ? insn->src_reg : insn->dst_reg;
			if (isimm) {
				*patch++ = BPF_MOV32_IMM(BPF_REG_AX, aux->alu_limit);
			} else {
				if (isneg)
					*patch++ = BPF_ALU64_IMM(BPF_MUL, off_reg, -1);
				*patch++ = BPF_MOV32_IMM(BPF_REG_AX, aux->alu_limit);
				*patch++ = BPF_ALU64_REG(BPF_SUB, BPF_REG_AX, off_reg);
				*patch++ = BPF_ALU64_REG(BPF_OR, BPF_REG_AX, off_reg);
				*patch++ = BPF_ALU64_IMM(BPF_NEG, BPF_REG_AX, 0);
				*patch++ = BPF_ALU64_IMM(BPF_ARSH, BPF_REG_AX, 63);
				*patch++ = BPF_ALU64_REG(BPF_AND, BPF_REG_AX, off_reg);
			}
			if (!issrc)
				*patch++ = BPF_MOV64_REG(insn->dst_reg, insn->src_reg);
			insn->src_reg = BPF_REG_AX;
			if (isneg)
				insn->code = insn->code == code_add ?
					     code_sub : code_add;
			*patch++ = *insn;
			if (issrc && isneg && !isimm)
				*patch++ = BPF_ALU64_IMM(BPF_MUL, off_reg, -1);
			cnt = patch - insn_buf;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		if (bpf_is_may_goto_insn(insn) && timed_may_goto) {
			u16 live = env->insn_aux_data[i].live_regs_before;
			int reg, save_off = 0;

			/* a register that is dead here holds the address of the count */
			for (reg = BPF_REG_0; reg <= BPF_REG_9; reg++)
				if (!(live & BIT(reg)))
					break;
			if (reg > BPF_REG_9) {
				reg = BPF_REG_0;
				save_off = -stack_depth - 8;
				stack_depth_extra = 8;
			}
			if (!tmg) {
				tmg = alloc_percpu_gfp(struct bpf_timed_may_goto,
						       GFP_KERNEL_ACCOUNT);
				if (!tmg)
					return -ENOMEM;
				prog->aux->timed_may_goto = tmg;
			}
			prog->jit_required = 1;
			cnt = may_goto_expand_timed(insn_buf, insn->off, may_goto_off(env, insn, i),
						    tmg, reg, save_off);

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		} else if (bpf_is_may_goto_insn(insn)) {
			int stack_off = -stack_depth - 8;
			struct bpf_insn tail[] = {
				BPF_ALU64_IMM(BPF_SUB, BPF_REG_AX, 1),
				BPF_STX_MEM(BPF_DW, BPF_REG_10, BPF_REG_AX, stack_off),
			};

			stack_depth_extra = 8;
			cnt = may_goto_expand(insn_buf, insn->off,
					      may_goto_off(env, insn, i),
					      stack_off, tail, ARRAY_SIZE(tail));

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		if (bpf_jit_supports_percpu_insn() &&
		    insn->code == (BPF_LD | BPF_IMM | BPF_DW) &&
		    (insn->src_reg == BPF_PSEUDO_MAP_VALUE ||
		     insn->src_reg == BPF_PSEUDO_MAP_IDX_VALUE)) {
			struct bpf_map *map;

			aux = &env->insn_aux_data[i];
			map = env->used_maps[aux->map_index];
			if (map->map_type != BPF_MAP_TYPE_PERCPU_ARRAY)
				goto next_insn;

			prog->jit_required = true;

			/*
			 * We are *skipping* first half of ld_imm64 insn
			 * with 'i++;', patching over second half of it
			 * with that same half + mov64_percpu_reg insn.
			 * All because a patch can only
			 * replace one 8-byte insn, which does not work
			 * well for ld_imm64 insn.
			 */

			insn_buf[0] = insn[1];
			insn_buf[1] = BPF_MOV64_PERCPU_REG(insn->dst_reg, insn->dst_reg);
			cnt = 2;

			i++;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		if (insn->code != (BPF_JMP | BPF_CALL))
			goto next_insn;
		if (insn->src_reg == BPF_PSEUDO_CALL)
			goto next_insn;
		if (insn->src_reg == BPF_PSEUDO_KFUNC_CALL) {
			ret = bpf_fixup_kfunc_call(env, insn, insn_buf, i, &cnt);
			if (ret)
				return ret;
			if (cnt == 0)
				goto next_insn;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Skip inlining the helper call if the JIT does it. */
		if (bpf_jit_inlines_helper_call(insn->imm)) {
			prog->jit_required = 1;
			goto next_insn;
		}

		if (insn->imm == BPF_FUNC_get_route_realm)
			prog->dst_needed = 1;
		if (insn->imm == BPF_FUNC_get_prandom_u32)
			bpf_user_rnd_init_once();
		if (insn->imm == BPF_FUNC_override_return)
			prog->kprobe_override = 1;
		if (insn->imm == BPF_FUNC_tail_call) {
			/* If we tail call into other programs, we
			 * cannot make any assumptions since they can
			 * be replaced dynamically during runtime in
			 * the program array.
			 */
			prog->cb_access = 1;
			if (!bpf_allow_tail_call_in_subprogs(env))
				prog->aux->stack_depth = MAX_BPF_STACK;
			prog->aux->max_pkt_offset = MAX_PACKET_OFF;
			prog->aux->max_rdonly_access = U32_MAX;
			prog->aux->max_rdwr_access = U32_MAX;
			if (prog->type == BPF_PROG_TYPE_RAW_TRACEPOINT_WRITABLE)
				prog->aux->max_tp_access = U32_MAX;

			/* mark bpf_tail_call as different opcode to avoid
			 * conditional branch in the interpreter for every normal
			 * call and to prevent accidental JITing by JIT compiler
			 * that doesn't support bpf_tail_call yet
			 */
			insn->imm = 0;
			insn->code = BPF_JMP | BPF_TAIL_CALL;

			aux = &env->insn_aux_data[i];
			if (env->bpf_capable && !prog->blinding_requested &&
			    prog->jit_requested &&
			    !bpf_map_key_poisoned(aux) &&
			    !bpf_map_ptr_poisoned(aux) &&
			    !bpf_map_ptr_unpriv(aux)) {
				struct bpf_jit_poke_descriptor desc = {
					.reason = BPF_POKE_REASON_TAIL_CALL,
					.tail_call.map = aux->map_ptr_state.map_ptr,
					.tail_call.key = bpf_map_key_immediate(aux),
					.insn_idx = i,
				};

				ret = bpf_jit_add_poke_descriptor(prog, &desc);
				if (ret < 0) {
					verbose(env, "adding tail call poke descriptor failed\n");
					return ret;
				}

				insn->imm = ret + 1;
				goto next_insn;
			}

			if (!bpf_map_ptr_unpriv(aux))
				goto next_insn;

			/* instead of changing every JIT dealing with tail_call
			 * emit two extra insns:
			 * if (index >= max_entries) goto out;
			 * index &= array->index_mask;
			 * to avoid out-of-bounds cpu speculation
			 */
			if (bpf_map_ptr_poisoned(aux)) {
				verbose(env, "tail_call abusing map_ptr\n");
				return -EINVAL;
			}

			map_ptr = aux->map_ptr_state.map_ptr;
			insn_buf[0] = BPF_JMP_IMM(BPF_JGE, BPF_REG_3,
						  map_ptr->max_entries, 2);
			insn_buf[1] = BPF_ALU32_IMM(BPF_AND, BPF_REG_3,
						    container_of(map_ptr,
								 struct bpf_array,
								 map)->index_mask);
			insn_buf[2] = *insn;
			cnt = 3;
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		aux = &env->insn_aux_data[i];
		if (aux->arg_prog) {
			/* The verifier will process callback_fn as many times as necessary
			 * with different maps and the register states prepared by
			 * set_timer_callback_state will be accurate.
			 *
			 * The following use case is valid:
			 *   map1 is shared by prog1, prog2, prog3.
			 *   prog1 calls bpf_timer_init for some map1 elements
			 *   prog2 calls bpf_timer_set_callback for some map1 elements.
			 *     Those that were not bpf_timer_init-ed will return -EINVAL.
			 *   prog3 calls bpf_timer_start for some map1 elements.
			 *     Those that were not both bpf_timer_init-ed and
			 *     bpf_timer_set_callback-ed will return -EINVAL.
			 */
			struct bpf_insn ld_addrs[2] = {
				BPF_LD_IMM64(aux->arg_prog, (long)prog->aux),
			};

			insn_buf[0] = ld_addrs[0];
			insn_buf[1] = ld_addrs[1];
			insn_buf[2] = *insn;
			cnt = 3;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto patch_call_imm;
		}

		/* bpf_per_cpu_ptr() and bpf_this_cpu_ptr() */
		if (env->insn_aux_data[i].call_with_percpu_alloc_ptr) {
			/* patch with 'r1 = *(u64 *)(r1 + 0)' since for percpu data,
			 * bpf_mem_alloc() returns a ptr to the percpu data ptr.
			 */
			insn_buf[0] = BPF_LDX_MEM(BPF_DW, BPF_REG_1, BPF_REG_1, 0);
			insn_buf[1] = *insn;
			cnt = 2;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto patch_call_imm;
		}

		/* BPF_EMIT_CALL() assumptions in some of the map_gen_lookup
		 * and other inlining handlers are currently limited to 64 bit
		 * only.
		 */
		if (prog->jit_requested && BITS_PER_LONG == 64 &&
		    (insn->imm == BPF_FUNC_map_lookup_elem ||
		     insn->imm == BPF_FUNC_map_update_elem ||
		     insn->imm == BPF_FUNC_map_delete_elem ||
		     insn->imm == BPF_FUNC_map_push_elem   ||
		     insn->imm == BPF_FUNC_map_pop_elem    ||
		     insn->imm == BPF_FUNC_map_peek_elem   ||
		     insn->imm == BPF_FUNC_redirect_map    ||
		     insn->imm == BPF_FUNC_for_each_map_elem ||
		     insn->imm == BPF_FUNC_map_lookup_percpu_elem)) {
			aux = &env->insn_aux_data[i];
			if (bpf_map_ptr_poisoned(aux))
				goto patch_call_imm;

			map_ptr = aux->map_ptr_state.map_ptr;
			ops = map_ptr->ops;
			if (insn->imm == BPF_FUNC_map_lookup_elem &&
			    ops->map_gen_lookup) {
				cnt = ops->map_gen_lookup(map_ptr, insn_buf);
				if (cnt == -EOPNOTSUPP)
					goto patch_map_ops_generic;
				if (cnt <= 0 || cnt >= INSN_BUF_SIZE) {
					verifier_bug(env, "%d insns generated for map lookup", cnt);
					return -EFAULT;
				}

				if (bpf_map_is_percpu_map(map_ptr->map_type))
					prog->jit_required = true;

				insn = bpf_patch_list_add(env, i, insn_buf, cnt);
				if (IS_ERR(insn))
					return PTR_ERR(insn);
				goto next_insn;
			}

			BUILD_BUG_ON(!__same_type(ops->map_lookup_elem,
				     (void *(*)(struct bpf_map *map, void *key))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_delete_elem,
				     (long (*)(struct bpf_map *map, void *key))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_update_elem,
				     (long (*)(struct bpf_map *map, void *key, void *value,
					      u64 flags))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_push_elem,
				     (long (*)(struct bpf_map *map, void *value,
					      u64 flags))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_pop_elem,
				     (long (*)(struct bpf_map *map, void *value))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_peek_elem,
				     (long (*)(struct bpf_map *map, void *value))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_redirect,
				     (long (*)(struct bpf_map *map, u64 index, u64 flags))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_for_each_callback,
				     (long (*)(struct bpf_map *map,
					      bpf_callback_t callback_fn,
					      void *callback_ctx,
					      u64 flags))NULL));
			BUILD_BUG_ON(!__same_type(ops->map_lookup_percpu_elem,
				     (void *(*)(struct bpf_map *map, void *key, u32 cpu))NULL));

patch_map_ops_generic:
			switch (insn->imm) {
			case BPF_FUNC_map_lookup_elem:
				insn->imm = BPF_CALL_IMM(ops->map_lookup_elem);
				goto next_insn;
			case BPF_FUNC_map_update_elem:
				insn->imm = BPF_CALL_IMM(ops->map_update_elem);
				goto next_insn;
			case BPF_FUNC_map_delete_elem:
				insn->imm = BPF_CALL_IMM(ops->map_delete_elem);
				goto next_insn;
			case BPF_FUNC_map_push_elem:
				insn->imm = BPF_CALL_IMM(ops->map_push_elem);
				goto next_insn;
			case BPF_FUNC_map_pop_elem:
				insn->imm = BPF_CALL_IMM(ops->map_pop_elem);
				goto next_insn;
			case BPF_FUNC_map_peek_elem:
				insn->imm = BPF_CALL_IMM(ops->map_peek_elem);
				goto next_insn;
			case BPF_FUNC_redirect_map:
				insn->imm = BPF_CALL_IMM(ops->map_redirect);
				goto next_insn;
			case BPF_FUNC_for_each_map_elem:
				insn->imm = BPF_CALL_IMM(ops->map_for_each_callback);
				goto next_insn;
			case BPF_FUNC_map_lookup_percpu_elem:
				insn->imm = BPF_CALL_IMM(ops->map_lookup_percpu_elem);
				goto next_insn;
			}

			goto patch_call_imm;
		}

		/* Implement bpf_jiffies64 inline. */
		if (prog->jit_requested && BITS_PER_LONG == 64 &&
		    insn->imm == BPF_FUNC_jiffies64) {
			struct bpf_insn ld_jiffies_addr[2] = {
				BPF_LD_IMM64(BPF_REG_0,
					     (unsigned long)&jiffies),
			};

			insn_buf[0] = ld_jiffies_addr[0];
			insn_buf[1] = ld_jiffies_addr[1];
			insn_buf[2] = BPF_LDX_MEM(BPF_DW, BPF_REG_0,
						  BPF_REG_0, 0);
			cnt = 3;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

#if defined(CONFIG_X86_64) && !defined(CONFIG_UML)
		/* Implement bpf_get_smp_processor_id() inline. */
		if (insn->imm == BPF_FUNC_get_smp_processor_id &&
		    bpf_verifier_inlines_helper_call(env, insn->imm)) {
			/* BPF_FUNC_get_smp_processor_id inlining is an
			 * optimization, so if cpu_number is ever
			 * changed in some incompatible and hard to support
			 * way, it's fine to back out this inlining logic
			 */
#ifdef CONFIG_SMP
			prog->jit_required = true;
			insn_buf[0] = BPF_MOV64_IMM(BPF_REG_0, (u32)(unsigned long)&cpu_number);
			insn_buf[1] = BPF_MOV64_PERCPU_REG(BPF_REG_0, BPF_REG_0);
			insn_buf[2] = BPF_LDX_MEM(BPF_W, BPF_REG_0, BPF_REG_0, 0);
			cnt = 3;
#else
			insn_buf[0] = BPF_ALU32_REG(BPF_XOR, BPF_REG_0, BPF_REG_0);
			cnt = 1;
#endif
			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement bpf_get_current_task() and bpf_get_current_task_btf() inline. */
		if ((insn->imm == BPF_FUNC_get_current_task || insn->imm == BPF_FUNC_get_current_task_btf) &&
		    bpf_verifier_inlines_helper_call(env, insn->imm)) {
			prog->jit_required = true;
			insn_buf[0] = BPF_MOV64_IMM(BPF_REG_0, (u32)(unsigned long)&current_task);
			insn_buf[1] = BPF_MOV64_PERCPU_REG(BPF_REG_0, BPF_REG_0);
			insn_buf[2] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_0, 0);
			cnt = 3;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}
#endif
		/* Implement bpf_get_func_arg inline. */
		if (prog_type == BPF_PROG_TYPE_TRACING &&
		    insn->imm == BPF_FUNC_get_func_arg) {
			if (eatype == BPF_TRACE_RAW_TP) {
				int nr_args = btf_type_vlen(prog->aux->attach_func_proto);

				/* skip 'void *__data' in btf_trace_##name() and save to reg0 */
				insn_buf[0] = BPF_MOV64_IMM(BPF_REG_0, nr_args - 1);
				cnt = 1;
			} else {
				/* Load nr_args from ctx - 8 */
				insn_buf[0] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_1, -8);
				insn_buf[1] = BPF_ALU64_IMM(BPF_AND, BPF_REG_0, 0xFF);
				cnt = 2;
			}
			insn_buf[cnt++] = BPF_JMP32_REG(BPF_JGE, BPF_REG_2, BPF_REG_0, 6);
			insn_buf[cnt++] = BPF_ALU32_IMM(BPF_LSH, BPF_REG_2, 3);
			insn_buf[cnt++] = BPF_ALU64_REG(BPF_ADD, BPF_REG_2, BPF_REG_1);
			insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_2, 0);
			insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, BPF_REG_3, BPF_REG_0, 0);
			insn_buf[cnt++] = BPF_MOV64_IMM(BPF_REG_0, 0);
			insn_buf[cnt++] = BPF_JMP_A(1);
			insn_buf[cnt++] = BPF_MOV64_IMM(BPF_REG_0, -EINVAL);

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement bpf_get_func_ret inline. */
		if (prog_type == BPF_PROG_TYPE_TRACING &&
		    insn->imm == BPF_FUNC_get_func_ret) {
			if (eatype == BPF_TRACE_FEXIT ||
			    eatype == BPF_TRACE_FSESSION ||
			    eatype == BPF_TRACE_FEXIT_MULTI ||
			    eatype == BPF_TRACE_FSESSION_MULTI ||
			    eatype == BPF_MODIFY_RETURN) {
				/* Load nr_args from ctx - 8 */
				insn_buf[0] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_1, -8);
				insn_buf[1] = BPF_ALU64_IMM(BPF_AND, BPF_REG_0, 0xFF);
				insn_buf[2] = BPF_ALU64_IMM(BPF_LSH, BPF_REG_0, 3);
				insn_buf[3] = BPF_ALU64_REG(BPF_ADD, BPF_REG_0, BPF_REG_1);
				insn_buf[4] = BPF_LDX_MEM(BPF_DW, BPF_REG_3, BPF_REG_0, 0);
				insn_buf[5] = BPF_STX_MEM(BPF_DW, BPF_REG_2, BPF_REG_3, 0);
				insn_buf[6] = BPF_MOV64_IMM(BPF_REG_0, 0);
				cnt = 7;
			} else {
				insn_buf[0] = BPF_MOV64_IMM(BPF_REG_0, -EOPNOTSUPP);
				cnt = 1;
			}

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement get_func_arg_cnt inline. */
		if (prog_type == BPF_PROG_TYPE_TRACING &&
		    insn->imm == BPF_FUNC_get_func_arg_cnt) {
			if (eatype == BPF_TRACE_RAW_TP) {
				int nr_args = btf_type_vlen(prog->aux->attach_func_proto);

				/* skip 'void *__data' in btf_trace_##name() and save to reg0 */
				insn_buf[0] = BPF_MOV64_IMM(BPF_REG_0, nr_args - 1);
				cnt = 1;
			} else {
				/* Load nr_args from ctx - 8 */
				insn_buf[0] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_1, -8);
				insn_buf[1] = BPF_ALU64_IMM(BPF_AND, BPF_REG_0, 0xFF);
				cnt = 2;
			}

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement bpf_get_func_ip inline. */
		if (prog_type == BPF_PROG_TYPE_TRACING &&
		    insn->imm == BPF_FUNC_get_func_ip) {
			/* Load IP address from ctx - 16 */
			insn_buf[0] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_1, -16);

			insn = bpf_patch_list_add(env, i, insn_buf, 1);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement bpf_get_branch_snapshot inline. */
		if (IS_ENABLED(CONFIG_PERF_EVENTS) &&
		    prog->jit_requested && BITS_PER_LONG == 64 &&
		    insn->imm == BPF_FUNC_get_branch_snapshot) {
			/* We are dealing with the following func protos:
			 * u64 bpf_get_branch_snapshot(void *buf, u32 size, u64 flags);
			 * int perf_snapshot_branch_stack(struct perf_branch_entry *entries, u32 cnt);
			 */
			const u32 br_entry_size = sizeof(struct perf_branch_entry);

			/* struct perf_branch_entry is part of UAPI and is
			 * used as an array element, so extremely unlikely to
			 * ever grow or shrink
			 */
			BUILD_BUG_ON(br_entry_size != 24);

			/* if (unlikely(flags)) return -EINVAL */
			insn_buf[0] = BPF_JMP_IMM(BPF_JNE, BPF_REG_3, 0, 7);

			/* Transform size (bytes) into number of entries (cnt = size / 24).
			 * But to avoid expensive division instruction, we implement
			 * divide-by-3 through multiplication, followed by further
			 * division by 8 through 3-bit right shift.
			 * Refer to book "Hacker's Delight, 2nd ed." by Henry S. Warren, Jr.,
			 * p. 227, chapter "Unsigned Division by 3" for details and proofs.
			 *
			 * N / 3 <=> M * N / 2^33, where M = (2^33 + 1) / 3 = 0xaaaaaaab.
			 */
			insn_buf[1] = BPF_MOV32_IMM(BPF_REG_0, 0xaaaaaaab);
			insn_buf[2] = BPF_ALU64_REG(BPF_MUL, BPF_REG_2, BPF_REG_0);
			insn_buf[3] = BPF_ALU64_IMM(BPF_RSH, BPF_REG_2, 36);

			/* call perf_snapshot_branch_stack implementation */
			insn_buf[4] = BPF_EMIT_CALL(static_call_query(perf_snapshot_branch_stack));
			/* if (entry_cnt == 0) return -ENOENT */
			insn_buf[5] = BPF_JMP_IMM(BPF_JEQ, BPF_REG_0, 0, 4);
			/* return entry_cnt * sizeof(struct perf_branch_entry) */
			insn_buf[6] = BPF_ALU32_IMM(BPF_MUL, BPF_REG_0, br_entry_size);
			insn_buf[7] = BPF_JMP_A(3);
			/* return -EINVAL; */
			insn_buf[8] = BPF_MOV64_IMM(BPF_REG_0, -EINVAL);
			insn_buf[9] = BPF_JMP_A(1);
			/* return -ENOENT; */
			insn_buf[10] = BPF_MOV64_IMM(BPF_REG_0, -ENOENT);
			cnt = 11;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}

		/* Implement bpf_kptr_xchg inline */
		if (prog->jit_requested && BITS_PER_LONG == 64 &&
		    insn->imm == BPF_FUNC_kptr_xchg &&
		    bpf_jit_supports_ptr_xchg()) {
			insn_buf[0] = BPF_MOV64_REG(BPF_REG_0, BPF_REG_2);
			insn_buf[1] = BPF_ATOMIC_OP(BPF_DW, BPF_XCHG, BPF_REG_1, BPF_REG_0, 0);
			cnt = 2;

			insn = bpf_patch_list_add(env, i, insn_buf, cnt);
			if (IS_ERR(insn))
				return PTR_ERR(insn);
			goto next_insn;
		}
patch_call_imm:
		fn = env->ops->get_func_proto(insn->imm, env->prog);
		/* all functions that have prototype and verifier allowed
		 * programs to call them, must be real in-kernel functions
		 */
		if (!fn->func) {
			verifier_bug(env,
				     "not inlined functions %s#%d is missing func",
				     func_id_name(insn->imm), insn->imm);
			return -EFAULT;
		}
		insn->imm = BPF_CALL_IMM(fn->func);
next_insn:
		if (subprogs[cur_subprog + 1].start == i + 1) {
			subprogs[cur_subprog].stack_depth += stack_depth_extra;
			subprogs[cur_subprog].stack_extra = stack_depth_extra;

			stack_depth = subprogs[cur_subprog].stack_depth;
			if (stack_depth > MAX_BPF_STACK && !prog->jit_requested) {
				verbose(env, "stack size %d(extra %d) is too large\n",
					stack_depth, stack_depth_extra);
				return -EINVAL;
			}
			cur_subprog++;
			stack_depth = subprogs[cur_subprog].stack_depth;
			stack_depth_extra = 0;
		}
		i++;
		insn = &prog->insnsi[i];
	}

	ret = bpf_patch_list_commit(env);
	if (ret)
		return ret;
	prog = env->prog;

	env->prog->aux->stack_depth = subprogs[0].stack_depth;
	for (i = 0; i < env->subprog_cnt; i++) {
		int stack_slots = subprogs[i].stack_extra / 8;
		/* an invocation begins, see bpf_timed_may_goto_save() for async callbacks */
		bool main_prog = tmg && !i;
		/* only a helper calls it, in a loop that ends when it returns 1 */
		bool loop_cb = tmg && i && subprogs[i].is_loop_cb && !subprogs[i].is_called &&
			       !subprogs[i].is_async_cb;

		if (timed_may_goto ? !main_prog && !loop_cb : !stack_slots)
			continue;
		if (stack_slots > 1) {
			verifier_bug(env, "stack_slots supports may_goto only");
			return -EFAULT;
		}

		cnt = 0;
		stack_depth = subprogs[i].stack_depth;
		if (timed_may_goto) {
			struct bpf_insn ld_tmg[] = {
				BPF_LD_IMM64(BPF_REG_0, (__force unsigned long)tmg)
			};
			int count_off = offsetof(struct bpf_timed_may_goto, count);
			int timestamp_off = offsetof(struct bpf_timed_may_goto, timestamp);
			int owner_off = offsetof(struct bpf_timed_may_goto, owner);

			/* R0 has nothing at the entry */
			insn_buf[cnt++] = ld_tmg[0];
			insn_buf[cnt++] = ld_tmg[1];
			insn_buf[cnt++] = BPF_MOV64_PERCPU_REG(BPF_REG_0, BPF_REG_0);
			if (main_prog) {
				insn_buf[cnt++] = BPF_ST_MEM(BPF_DW, BPF_REG_0, count_off,
							     BPF_MAX_TIMED_LOOPS);
				insn_buf[cnt++] = BPF_ST_MEM(BPF_DW, BPF_REG_0, timestamp_off, 0);
				insn_buf[cnt++] = BPF_ST_MEM(BPF_DW, BPF_REG_0, owner_off, 0);
			} else {
				/*
				 * The time is up, but the helper would call the function again and
				 * again. End its loop. The verifier has seen that: the loop can end
				 * after each call.
				 */
				insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_0, BPF_REG_0,
							      count_off);
				insn_buf[cnt++] = BPF_JMP_IMM(BPF_JNE, BPF_REG_0, 0, 2);
				insn_buf[cnt++] = BPF_MOV64_IMM(BPF_REG_0, 1);
				insn_buf[cnt++] = BPF_EXIT_INSN();
			}
		} else {
			/* Add ST insn to subprog prologue to init extra stack */
			insn_buf[cnt++] = BPF_ST_MEM(BPF_DW, BPF_REG_FP, -stack_depth,
						     BPF_MAX_LOOPS);
		}
		/* jumps to the first insn of the subprog skip the setup */
		ret = bpf_patch_list_insert(env, subprogs[i].start, insn_buf, cnt);
		if (ret)
			return ret;
	}

	ret = bpf_patch_list_commit(env);
	if (ret)
		return ret;
	prog = env->prog;

	/* Since poke tab is now finalized, publish aux to tracker. */
	for (i = 0; i < prog->aux->size_poke_tab; i++) {
		map_ptr = prog->aux->poke_tab[i].tail_call.map;
		if (!map_ptr->ops->map_poke_track ||
		    !map_ptr->ops->map_poke_untrack ||
		    !map_ptr->ops->map_poke_run) {
			verifier_bug(env, "poke tab is misconfigured");
			return -EFAULT;
		}

		ret = map_ptr->ops->map_poke_track(map_ptr, prog->aux);
		if (ret < 0) {
			verbose(env, "tracking tail call prog failed\n");
			return ret;
		}
	}

	return 0;
}

static int inline_bpf_loop(struct bpf_verifier_env *env, int position,
			   s32 stack_base, u32 callback_subprogno)
{
	s32 r6_offset = stack_base + 0 * BPF_REG_SIZE;
	s32 r7_offset = stack_base + 1 * BPF_REG_SIZE;
	s32 r8_offset = stack_base + 2 * BPF_REG_SIZE;
	int reg_loop_max = BPF_REG_6;
	int reg_loop_cnt = BPF_REG_7;
	int reg_loop_ctx = BPF_REG_8;

	struct bpf_insn *insn_buf = env->insn_buf;
	u32 callback_start;
	struct bpf_insn *last;
	s32 callback_offset;
	u32 cnt = 0;

	/* This represents an inlined version of bpf_iter.c:bpf_loop,
	 * be careful to modify this code in sync.
	 */

	/* Return error and jump to the end of the patch if
	 * expected number of iterations is too big.
	 */
	insn_buf[cnt++] = BPF_JMP_IMM(BPF_JLE, BPF_REG_1, BPF_MAX_LOOPS, 2);
	insn_buf[cnt++] = BPF_MOV32_IMM(BPF_REG_0, -E2BIG);
	insn_buf[cnt++] = BPF_JMP_IMM(BPF_JA, 0, 0, 16);
	/* spill R6, R7, R8 to use these as loop vars */
	insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, BPF_REG_10, BPF_REG_6, r6_offset);
	insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, BPF_REG_10, BPF_REG_7, r7_offset);
	insn_buf[cnt++] = BPF_STX_MEM(BPF_DW, BPF_REG_10, BPF_REG_8, r8_offset);
	/* initialize loop vars */
	insn_buf[cnt++] = BPF_MOV64_REG(reg_loop_max, BPF_REG_1);
	insn_buf[cnt++] = BPF_MOV32_IMM(reg_loop_cnt, 0);
	insn_buf[cnt++] = BPF_MOV64_REG(reg_loop_ctx, BPF_REG_3);
	/* loop header,
	 * if reg_loop_cnt >= reg_loop_max skip the loop body
	 */
	insn_buf[cnt++] = BPF_JMP_REG(BPF_JGE, reg_loop_cnt, reg_loop_max, 5);
	/* callback call, the offset is set below */
	insn_buf[cnt++] = BPF_MOV64_REG(BPF_REG_1, reg_loop_cnt);
	insn_buf[cnt++] = BPF_MOV64_REG(BPF_REG_2, reg_loop_ctx);
	insn_buf[cnt++] = BPF_CALL_REL(0);
	/* increment loop counter */
	insn_buf[cnt++] = BPF_ALU64_IMM(BPF_ADD, reg_loop_cnt, 1);
	/* jump to loop header if callback returned 0 */
	insn_buf[cnt++] = BPF_JMP_IMM(BPF_JEQ, BPF_REG_0, 0, -6);
	/* return value of bpf_loop,
	 * set R0 to the number of iterations
	 */
	insn_buf[cnt++] = BPF_MOV64_REG(BPF_REG_0, reg_loop_cnt);
	/* restore original values of R6, R7, R8 */
	insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_6, BPF_REG_10, r6_offset);
	insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_7, BPF_REG_10, r7_offset);
	insn_buf[cnt++] = BPF_LDX_MEM(BPF_DW, BPF_REG_8, BPF_REG_10, r8_offset);

	/*
	 * Note: insn_buf[12] is the BPF_CALL_REL instruction. Jumps and calls
	 * out of a patch are relative to the patch placed at 'position': its
	 * 'cnt' insns push what is behind 'position' down by cnt - 1.
	 */
	callback_start = env->subprog_info[callback_subprogno].start;
	callback_offset = callback_start - position;
	if (callback_start > position)
		callback_offset += cnt - 1;
	insn_buf[12].imm = callback_offset - 12 - 1;

	last = bpf_patch_list_add(env, position, insn_buf, cnt);
	return PTR_ERR_OR_ZERO(last);
}

static bool is_bpf_loop_call(struct bpf_insn *insn)
{
	return insn->code == (BPF_JMP | BPF_CALL) &&
		insn->src_reg == 0 &&
		insn->imm == BPF_FUNC_loop;
}

/* For all sub-programs in the program (including main) check
 * insn_aux_data to see if there are bpf_loop calls that require
 * inlining. If such calls are found the calls are replaced with a
 * sequence of instructions produced by `inline_bpf_loop` function and
 * subprog stack_depth is increased by the size of 3 registers.
 * This stack space is used to spill values of the R6, R7, R8.  These
 * registers are used to store the loop bound, counter and context
 * variables.
 */
int bpf_optimize_bpf_loop(struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *subprogs = env->subprog_info;
	int i, cur_subprog = 0, err;
	struct bpf_insn *insn = env->prog->insnsi;
	int insn_cnt = env->prog->len;
	u16 stack_depth = subprogs[cur_subprog].stack_depth;
	u16 stack_depth_roundup = round_up(stack_depth, 8) - stack_depth;
	u16 stack_depth_extra = 0;

	for (i = 0; i < insn_cnt; i++, insn++) {
		struct bpf_loop_inline_state *inline_state =
			&env->insn_aux_data[i].loop_inline_state;

		if (is_bpf_loop_call(insn) && inline_state->fit_for_inline) {
			stack_depth_extra = BPF_REG_SIZE * 3 + stack_depth_roundup;
			err = inline_bpf_loop(env, i,
					      -(stack_depth + stack_depth_extra),
					      inline_state->callback_subprogno);
			if (err)
				return err;
		}

		if (subprogs[cur_subprog + 1].start == i + 1) {
			subprogs[cur_subprog].stack_depth += stack_depth_extra;
			cur_subprog++;
			stack_depth = subprogs[cur_subprog].stack_depth;
			stack_depth_roundup = round_up(stack_depth, 8) - stack_depth;
			stack_depth_extra = 0;
		}
	}

	env->prog->aux->stack_depth = env->subprog_info[0].stack_depth;

	return bpf_patch_list_commit(env);
}

/* Remove unnecessary spill/fill pairs, members of fastcall pattern,
 * adjust subprograms stack depth when possible.
 */
int bpf_remove_fastcall_spills_fills(struct bpf_verifier_env *env)
{
	struct bpf_subprog_info *subprog = env->subprog_info;
	struct bpf_insn_aux_data *aux = env->insn_aux_data;
	struct bpf_insn *insn = env->prog->insnsi;
	int insn_cnt = env->prog->len;
	u32 spills_num;
	bool modified = false;
	int i, j;

	for (i = 0; i < insn_cnt; i++, insn++) {
		if (aux[i].fastcall_spills_num > 0) {
			spills_num = aux[i].fastcall_spills_num;
			/* NOPs would be removed by opt_remove_nops() */
			for (j = 1; j <= spills_num; ++j) {
				*(insn - j) = NOP;
				*(insn + j) = NOP;
			}
			modified = true;
		}
		if ((subprog + 1)->start == i + 1) {
			if (modified && !subprog->keep_fastcall_stack)
				subprog->stack_depth = -subprog->fastcall_stack_off;
			subprog++;
			modified = false;
		}
	}

	return 0;
}
