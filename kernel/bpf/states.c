// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */
#include <linux/bpf.h>
#include <linux/bpf_verifier.h>
#include <linux/cnum.h>
#include <linux/filter.h>

/* check %cur's range satisfies %old's */
static bool range_within(const struct bpf_reg_state *old,
			 const struct bpf_reg_state *cur)
{
	if (!cnum64_is_subset(old->r64, cur->r64) ||
	    !cnum32_is_subset(old->r32, cur->r32))
		return false;

	if (old->step <= 1)
		return true;

	if (cnum64_is_const(cur->r64))
		return imod((s64)cnum64_smin(cur->r64), old->step) == old->base;

	/*
	 * Both `old` and `cur` define some sets of points.
	 * Return true, if points defined by `cur` are a subset of points defined by `old`:
	 * - bounds for `cur` should be within bounds for `old`;
	 * - cur->step should be dividable by old->step;
	 * - cur->base should start at integer number of old->step
	 *   steps from old->base.
	 *
	 * E.g. the following ranges are compatible:
	 * - old [0, 10] step 2
	 * - new [2, 10] step 4
	 */
	return cur->step % old->step == 0 &&
	       (cur->base - old->base) % old->step == 0;
}

/* If in the old state two registers had the same id, then they need to have
 * the same id in the new state as well.  But that id could be different from
 * the old state, so we need to track the mapping from old to new ids.
 * Once we have seen that, say, a reg with old id 5 had new id 9, any subsequent
 * regs with old id 5 must also have new id 9 for the new state to be safe.  But
 * regs with a different old id could still have new id 9, we don't care about
 * that.
 * So we look through our idmap to see if this old id has been seen before.  If
 * so, we require the new id to match; otherwise, we add the id pair to the map.
 */
static bool check_ids(u32 old_id, u32 cur_id, struct bpf_idmap *idmap)
{
	struct bpf_id_pair *map = idmap->map;
	unsigned int i;

	/* either both IDs should be set or both should be zero */
	if (!!old_id != !!cur_id)
		return false;

	if (old_id == 0) /* cur_id == 0 as well */
		return true;

	/* a name of what the caller has, see BPF_INHERITED_ID */
	if (((old_id | cur_id) & ~BPF_ADD_CONST) >= BPF_INHERITED_ID)
		return old_id == cur_id;

	for (i = 0; i < idmap->cnt; i++) {
		if (map[i].old == old_id)
			return map[i].cur == cur_id;
		if (map[i].cur == cur_id)
			return false;
	}

	/* Reached the end of known mappings; haven't seen this id before */
	if (idmap->cnt < BPF_ID_MAP_SIZE) {
		map[idmap->cnt].old = old_id;
		map[idmap->cnt].cur = cur_id;
		idmap->cnt++;
		return true;
	}

	/*
	 * idmap slots are bounded by the number of registers and stack slots.
	 * Since referenced dynptrs acquire intermediate references that do
	 * not live in either, so the map can be exhausted. Since it is unlikely,
	 * fail the verification by treating the states as not equivalent.
	 */
	return false;
}

/*
 * Compare scalar register IDs for state equivalence.
 *
 * When old_id == 0, the old register is independent - not linked to any
 * other register. Any linking in the current state only adds constraints,
 * making it more restrictive. Since the old state didn't rely on any ID
 * relationships for this register, it's always safe to accept cur regardless
 * of its ID. Hence, return true immediately.
 *
 * When old_id != 0 but cur_id == 0, we need to ensure that different
 * independent registers in cur don't incorrectly satisfy the ID matching
 * requirements of linked registers in old.
 *
 * Example: if old has r6.id=X and r7.id=X (linked), but cur has r6.id=0
 * and r7.id=0 (both independent), without temp IDs both would map old_id=X
 * to cur_id=0 and pass. With temp IDs: r6 maps X->temp1, r7 tries to map
 * X->temp2, but X is already mapped to temp1, so the check fails correctly.
 *
 * When old_id has BPF_ADD_CONST set, the compound id (base | flag) and the
 * base id (flag stripped) must both map consistently. Example: old has
 * r2.id=A, r3.id=A|flag (r3 = r2 + delta), cur has r2.id=B, r3.id=C|flag
 * (r3 derived from unrelated r4). Without the base check, idmap gets two
 * independent entries A->B and A|flag->C|flag, missing that A->C conflicts
 * with A->B. The base ID cross-check catches this.
 */
static bool check_scalar_ids(u32 old_id, u32 cur_id, struct bpf_idmap *idmap)
{
	if (!old_id)
		return true;

	cur_id = cur_id ? cur_id : ++idmap->tmp_id_gen;

	if (!check_ids(old_id, cur_id, idmap))
		return false;
	if (old_id & BPF_ADD_CONST) {
		old_id &= ~BPF_ADD_CONST;
		cur_id &= ~BPF_ADD_CONST;
		if (!check_ids(old_id, cur_id, idmap))
			return false;
	}
	return true;
}

static void __clean_func_state(struct bpf_verifier_env *env,
			       struct bpf_func_state *st,
			       u16 live_regs, int frame)
{
	int i, j;

	for (i = 0; i < BPF_REG_FP; i++) {
		/* liveness must not touch this register anymore */
		if (!(live_regs & BIT(i)))
			/* since the register is unused, clear its state
			 * to make further comparison simpler
			 */
			bpf_mark_reg_not_init(env, &st->regs[i]);
	}

	/*
	 * Clean dead 4-byte halves within each SPI independently.
	 * half_spi 2*i   → lower half: slot_type[0..3] (closer to FP)
	 * half_spi 2*i+1 → upper half: slot_type[4..7] (farther from FP)
	 */
	for (i = 0; i < bpf_stack_nr_slots(st); i++) {
		bool lo_live = bpf_stack_slot_alive(env, frame, i * 2);
		bool hi_live = bpf_stack_slot_alive(env, frame, i * 2 + 1);

		if (!hi_live || !lo_live) {
			int start = !lo_live ? 0 : BPF_REG_SIZE / 2;
			int end = !hi_live ? BPF_REG_SIZE : BPF_REG_SIZE / 2;
			u8 stype = bpf_stack_slot(st, i)->slot_type[7];

			/*
			 * Don't clear special slots.
			 * destroy_if_dynptr_stack_slot() needs STACK_DYNPTR to
			 * detect overwrites and invalidate associated data slices.
			 * is_iter_reg_valid_uninit() and is_irq_flag_reg_valid_uninit()
			 * check for their respective slot types to detect double-create.
			 */
			if (stype == STACK_DYNPTR || stype == STACK_ITER ||
			    stype == STACK_IRQ_FLAG)
				continue;

			/*
			 * Only scalar spills can be degraded to raw stack bytes
			 * when their high half is dead. Pointer spills need the
			 * saved spilled_ptr metadata so partial fills keep
			 * rejecting as non-scalar register fills.
			 */
			if (!hi_live) {
				struct bpf_reg_state *spill = &bpf_stack_slot(st, i)->spilled_ptr;

				if (lo_live && stype == STACK_SPILL) {
					if (spill->type != SCALAR_VALUE)
						continue;
					/* keep what is known about the low half */
					if (bpf_register_is_null(spill))
						continue;
					for (j = 0; j < 4; j++) {
						u8 *t = &bpf_stack_slot(st, i)->slot_type[j];

						if (*t == STACK_SPILL)
							*t = STACK_MISC;
					}
				}
				bpf_mark_reg_not_init(env, spill);
			}
			for (j = start; j < end; j++)
				bpf_stack_slot(st, i)->slot_type[j] = STACK_POISON;
		}
	}
}

static int clean_verifier_state(struct bpf_verifier_env *env,
				 struct bpf_verifier_state *st)
{
	int i, err;

	err = bpf_live_stack_query_init(env, st);
	if (err)
		return err;
	for (i = 0; i <= st->curframe; i++) {
		u32 ip = bpf_frame_insn_idx(st, i);
		u16 live_regs = env->insn_aux_data[ip].live_regs_before;

		__clean_func_state(env, st->frame[i], live_regs, i);
	}
	return 0;
}

static bool regs_exact(const struct bpf_reg_state *rold,
		       const struct bpf_reg_state *rcur,
		       struct bpf_idmap *idmap)
{
	return memcmp(rold, rcur, offsetof(struct bpf_reg_state, id)) == 0 &&
	       check_ids(rold->id, rcur->id, idmap) &&
	       check_ids(rold->parent_id, rcur->parent_id, idmap) &&
	       check_ids(rold->map_uid, rcur->map_uid, idmap);
}

enum exact_level {
	NOT_EXACT,
	EXACT,
	RANGE_WITHIN
};

/* Returns true if (rold safe implies rcur safe) */
static bool regsafe(struct bpf_verifier_env *env, struct bpf_reg_state *rold,
		    struct bpf_reg_state *rcur, struct bpf_idmap *idmap,
		    enum exact_level exact)
{
	if (exact == EXACT)
		return regs_exact(rold, rcur, idmap);

	if (rold->type == NOT_INIT)
		/* explored state can't have used this */
		return true;

	/* Enforce that register types have to match exactly, including their
	 * modifiers (like PTR_MAYBE_NULL, MEM_RDONLY, etc), as a general
	 * rule.
	 *
	 * One can make a point that using a pointer register as unbounded
	 * SCALAR would be technically acceptable, but this could lead to
	 * pointer leaks because scalars are allowed to leak while pointers
	 * are not. We could make this safe in special cases if root is
	 * calling us, but it's probably not worth the hassle.
	 *
	 * Also, register types that are *not* MAYBE_NULL could technically be
	 * safe to use as their MAYBE_NULL variants (e.g., PTR_TO_MAP_VALUE
	 * is safe to be used as PTR_TO_MAP_VALUE_OR_NULL, provided both point
	 * to the same map).
	 * However, if the old MAYBE_NULL register then got NULL checked,
	 * doing so could have affected others with the same id, and we can't
	 * check for that because we lost the id when we converted to
	 * a non-MAYBE_NULL variant.
	 * So, as a general rule we don't allow mixing MAYBE_NULL and
	 * non-MAYBE_NULL registers as well.
	 */
	if (rold->type != rcur->type)
		return false;

	switch (base_type(rold->type)) {
	case SCALAR_VALUE:
		if (env->explore_alu_limits) {
			/* explore_alu_limits disables tnum_in() and range_within()
			 * logic and requires everything to be strict
			 */
			return memcmp(rold, rcur, offsetof(struct bpf_reg_state, id)) == 0 &&
			       check_scalar_ids(rold->id, rcur->id, idmap);
		}
		/*
		 * Linked register tracking uses rold->id to detect relationships.
		 * When rold->id == 0, the register is independent and any linking
		 * in rcur only adds constraints. When rold->id != 0, we must verify
		 * id mapping and (for BPF_ADD_CONST) offset consistency.
		 *
		 * +------------------+-----------+------------------+---------------+
		 * |                  | rold->id  | rold + ADD_CONST | rold->id == 0 |
		 * |------------------+-----------+------------------+---------------|
		 * | rcur->id         | range,ids | false            | range         |
		 * | rcur + ADD_CONST | false     | range,ids,off    | range         |
		 * | rcur->id == 0    | range,ids | false            | range         |
		 * +------------------+-----------+------------------+---------------+
		 *
		 * Why check_ids() for scalar registers?
		 *
		 * Consider the following BPF code:
		 *   1: r6 = ... unbound scalar, ID=a ...
		 *   2: r7 = ... unbound scalar, ID=b ...
		 *   3: if (r6 > r7) goto +1
		 *   4: r6 = r7
		 *   5: if (r6 > X) goto ...
		 *   6: ... memory operation using r7 ...
		 *
		 * First verification path is [1-6]:
		 * - at (4) same bpf_reg_state::id (b) would be assigned to r6 and r7;
		 * - at (5) r6 would be marked <= X, sync_linked_regs() would also mark
		 *   r7 <= X, because r6 and r7 share same id.
		 * Next verification path is [1-4, 6].
		 *
		 * Instruction (6) would be reached in two states:
		 *   I.  r6{.id=b}, r7{.id=b} via path 1-6;
		 *   II. r6{.id=a}, r7{.id=b} via path 1-4, 6.
		 *
		 * Use check_ids() to distinguish these states.
		 * ---
		 * Also verify that new value satisfies old value range knowledge.
		 */

		/*
		 * ADD_CONST flags must match exactly: BPF_ADD_CONST32 and
		 * BPF_ADD_CONST64 have different linking semantics in
		 * sync_linked_regs() (alu32 zero-extends, alu64 does not),
		 * so pruning across different flag types is unsafe.
		 */
		if (rold->id &&
		    (rold->id & BPF_ADD_CONST) != (rcur->id & BPF_ADD_CONST))
			return false;

		/* Both have offset linkage: offsets must match */
		if ((rold->id & BPF_ADD_CONST) && rold->delta != rcur->delta)
			return false;

		if (!check_scalar_ids(rold->id, rcur->id, idmap))
			return false;

		return range_within(rold, rcur) && tnum_in(rold->var_off, rcur->var_off);
	case PTR_TO_MAP_KEY:
	case PTR_TO_MAP_VALUE:
	case PTR_TO_MEM:
	case PTR_TO_BUF:
	case PTR_TO_TP_BUFFER:
	case PTR_TO_STACK:
	case PTR_TO_BTF_ID:
		/* If the new min/max/var_off satisfy the old ones and
		 * everything else matches, we are OK.
		 */
		return memcmp(rold, rcur, offsetof(struct bpf_reg_state, var_off)) == 0 &&
		       range_within(rold, rcur) &&
		       tnum_in(rold->var_off, rcur->var_off) &&
		       check_ids(rold->id, rcur->id, idmap) &&
		       check_ids(rold->parent_id, rcur->parent_id, idmap) &&
		       check_ids(rold->map_uid, rcur->map_uid, idmap);
	case PTR_TO_PACKET_META:
	case PTR_TO_PACKET:
		/* We must have at least as much range as the old ptr
		 * did, so that any accesses which were safe before are
		 * still safe.  This is true even if old range < old off,
		 * since someone could have accessed through (ptr - k), or
		 * even done ptr -= k in a register, to get a safe access.
		 */
		if (rold->range < 0 || rcur->range < 0) {
			/* special case for [BEYOND|AT]_PKT_END */
			if (rold->range != rcur->range)
				return false;
		} else if (rold->range > rcur->range) {
			return false;
		}
		/* id relations must be preserved */
		if (!check_ids(rold->id, rcur->id, idmap))
			return false;
		/* Preserve displacements between pointers sharing an ID. */
		if (rold->id && rold->r64.base != rcur->r64.base)
			return false;
		/* new val must satisfy old val knowledge */
		return range_within(rold, rcur) &&
		       tnum_in(rold->var_off, rcur->var_off);
	case PTR_TO_ARENA:
		return true;
	case PTR_TO_INSN:
		return memcmp(rold, rcur, offsetof(struct bpf_reg_state, var_off)) == 0 &&
		       range_within(rold, rcur) && tnum_in(rold->var_off, rcur->var_off);
	default:
		return regs_exact(rold, rcur, idmap);
	}
}

static struct bpf_reg_state unbound_reg;

static __init int unbound_reg_init(void)
{
	bpf_mark_reg_unknown(&unbound_reg);
	return 0;
}
late_initcall(unbound_reg_init);

static bool is_spilled_scalar_after(const struct bpf_stack_state *stack, int im)
{
	return stack->slot_type[im] == STACK_SPILL &&
	       stack->spilled_ptr.type == SCALAR_VALUE;
}

static bool is_stack_misc_after(struct bpf_verifier_env *env,
				struct bpf_stack_state *stack, int im)
{
	u32 i;

	for (i = im; i < ARRAY_SIZE(stack->slot_type); ++i) {
		if ((stack->slot_type[i] == STACK_MISC) ||
		    ((stack->slot_type[i] == STACK_INVALID || stack->slot_type[i] == STACK_POISON) &&
		     env->allow_uninit_stack))
			continue;
		return false;
	}

	return true;
}

static struct bpf_reg_state *scalar_reg_for_stack(struct bpf_verifier_env *env,
						  struct bpf_stack_state *stack, int im)
{
	if (is_spilled_scalar_after(stack, im))
		return &stack->spilled_ptr;

	if (is_stack_misc_after(env, stack, im))
		return &unbound_reg;

	return NULL;
}

static bool stacksafe(struct bpf_verifier_env *env, struct bpf_func_state *old,
		      struct bpf_func_state *cur, struct bpf_idmap *idmap,
		      enum exact_level exact)
{
	int i, spi;

	/* walk slots of the explored stack and ignore any additional
	 * slots in the current stack, since explored(safe) state
	 * didn't use them
	 */
	for (i = 0; i < old->allocated_stack; i++) {
		struct bpf_stack_state *old_slot, *cur_slot;
		struct bpf_reg_state *old_reg, *cur_reg;
		int im = i % BPF_REG_SIZE;
		u8 old_type;

		spi = i / BPF_REG_SIZE;
		old_slot = bpf_stack_slot(old, spi);
		old_type = old_slot->slot_type[im];
		cur_slot = i < cur->allocated_stack ? bpf_stack_slot(cur, spi) : NULL;

		if (exact == EXACT) {
			u8 cur_type = cur_slot ? cur_slot->slot_type[im] : STACK_INVALID;

			/* STACK_INVALID and STACK_POISON are equivalent for pruning */
			if (old_type == STACK_POISON)
				old_type = STACK_INVALID;
			if (cur_type == STACK_POISON)
				cur_type = STACK_INVALID;
			if (!cur_slot || old_type != cur_type)
				return false;
		}

		if (old_type == STACK_INVALID || old_type == STACK_POISON)
			continue;

		if (env->allow_uninit_stack && old_type == STACK_MISC)
			continue;

		/* explored stack has more populated slots than current stack
		 * and these slots were used
		 */
		if (!cur_slot)
			return false;

		/*
		 * 64 and 32-bit scalar spills vs MISC/INVALID slots and vice versa.
		 * Load from MISC/INVALID slots produces unbound scalar.
		 * Construct a fake register for such stack and call
		 * regsafe() to ensure scalar ids are compared.
		 */
		if (im == 0 || im == 4) {
			old_reg = scalar_reg_for_stack(env, old_slot, im);
			cur_reg = scalar_reg_for_stack(env, cur_slot, im);
			if (old_reg && cur_reg) {
				if (!regsafe(env, old_reg, cur_reg, idmap, exact))
					return false;
				i += (im == 0 ? BPF_REG_SIZE - 1 : 3);
				continue;
			}
		}

		/* if old state was safe with misc data in the stack
		 * it will be safe with zero-initialized stack.
		 * The opposite is not true
		 */
		if (old_type == STACK_MISC && cur_slot->slot_type[im] == STACK_ZERO)
			continue;
		if (old_type != cur_slot->slot_type[im])
			/* Ex: old explored (safe) state has STACK_SPILL in
			 * this stack slot, but current has STACK_MISC ->
			 * this verifier states are not equivalent,
			 * return false to continue verification of this path
			 */
			return false;
		if (im != BPF_REG_SIZE - 1)
			continue;
		/* Both old and cur are having same slot_type */
		switch (old_type) {
		case STACK_SPILL:
			/* when explored and current stack slot are both storing
			 * spilled registers, check that stored pointers types
			 * are the same as well.
			 * Ex: explored safe path could have stored
			 * (bpf_reg_state) {.type = PTR_TO_STACK, .off = -8}
			 * but current path has stored:
			 * (bpf_reg_state) {.type = PTR_TO_STACK, .off = -16}
			 * such verifier states are not equivalent.
			 * return false to continue verification of this path
			 */
			if (!regsafe(env, &old_slot->spilled_ptr, &cur_slot->spilled_ptr,
				     idmap, exact))
				return false;
			break;
		case STACK_DYNPTR:
			old_reg = &old_slot->spilled_ptr;
			cur_reg = &cur_slot->spilled_ptr;
			if (old_reg->dynptr.type != cur_reg->dynptr.type ||
			    old_reg->dynptr.first_slot != cur_reg->dynptr.first_slot ||
			    !check_ids(old_reg->id, cur_reg->id, idmap) ||
			    !check_ids(old_reg->parent_id, cur_reg->parent_id, idmap))
				return false;
			break;
		case STACK_ITER:
			old_reg = &old_slot->spilled_ptr;
			cur_reg = &cur_slot->spilled_ptr;
			if (old_reg->type != cur_reg->type ||
			    old_reg->iter.btf != cur_reg->iter.btf ||
			    old_reg->iter.btf_id != cur_reg->iter.btf_id ||
			    old_reg->iter.state != cur_reg->iter.state ||
			    old_reg->iter.loops != cur_reg->iter.loops ||
			    !check_ids(old_reg->id, cur_reg->id, idmap))
				return false;
			break;
		case STACK_IRQ_FLAG:
			old_reg = &old_slot->spilled_ptr;
			cur_reg = &cur_slot->spilled_ptr;
			if (!check_ids(old_reg->id, cur_reg->id, idmap) ||
			    old_reg->irq.kfunc_class != cur_reg->irq.kfunc_class)
				return false;
			break;
		case STACK_MISC:
		case STACK_ZERO:
		case STACK_INVALID:
		case STACK_POISON:
			continue;
		/* Ensure that new unhandled slot types return false by default */
		default:
			return false;
		}
	}
	return true;
}

/*
 * Compare stack arg slots between old and current states.
 * Outgoing stack args are path-local state and must agree for pruning.
 */
static bool stack_arg_safe(struct bpf_verifier_env *env, struct bpf_func_state *old,
			   struct bpf_func_state *cur, struct bpf_idmap *idmap,
			   enum exact_level exact)
{
	int i, nslots, cur_cnt = cur->out_stack_arg_cnt;

	/* incoming arguments are not compared with outgoing ones, the other way is ruled out */
	if (bpf_stack_args_are_incoming(cur) && !bpf_stack_args_are_incoming(old))
		cur_cnt = 0;
	nslots = max(old->out_stack_arg_cnt, cur_cnt);
	for (i = 0; i < nslots; i++) {
		struct bpf_reg_state *old_arg, *cur_arg;
		struct bpf_reg_state not_init = { .type = NOT_INIT };

		old_arg = i < old->out_stack_arg_cnt ?
			  &old->stack_arg_regs[i] : &not_init;
		cur_arg = i < cur_cnt ?
			  &cur->stack_arg_regs[i] : &not_init;
		if (!regsafe(env, old_arg, cur_arg, idmap, exact))
			return false;
	}

	return true;
}

static bool refsafe(struct bpf_verifier_state *old, struct bpf_verifier_state *cur,
		    struct bpf_idmap *idmap)
{
	int i;

	if (old->acquired_refs != cur->acquired_refs)
		return false;

	if (old->active_locks != cur->active_locks)
		return false;

	if (old->active_preempt_locks != cur->active_preempt_locks)
		return false;

	if (old->active_rcu_locks != cur->active_rcu_locks)
		return false;

	if (!check_ids(old->active_irq_id, cur->active_irq_id, idmap))
		return false;

	if (!check_ids(old->active_lock_id, cur->active_lock_id, idmap) ||
	    old->active_lock_ptr != cur->active_lock_ptr)
		return false;

	for (i = 0; i < old->acquired_refs; i++) {
		if (!check_ids(old->refs[i].id, cur->refs[i].id, idmap) ||
		    old->refs[i].type != cur->refs[i].type)
			return false;
		switch (old->refs[i].type) {
		case REF_TYPE_PTR:
			if (!check_ids(old->refs[i].parent_id, cur->refs[i].parent_id, idmap))
				return false;
			break;
		case REF_TYPE_IRQ:
			break;
		case REF_TYPE_LOCK:
		case REF_TYPE_RES_LOCK:
		case REF_TYPE_RES_LOCK_IRQ:
			if (old->refs[i].ptr != cur->refs[i].ptr)
				return false;
			break;
		default:
			WARN_ONCE(1, "Unhandled enum type for reference state: %d\n", old->refs[i].type);
			return false;
		}
	}

	return true;
}

/*
 * In a loop that is walked without a count a state of an earlier iteration stands for a state of
 * a later one, see flow_cycle_closed(). Never the other way round.
 *
 * In a loop that is counted and walked the state that gets to iteration max_header_count is
 * dropped: there is no such iteration at run time. The number in a state is the number of
 * iterations that the program has done, and only states with the same number are compared. They
 * differ in what is counted anyway.
 *
 * The walk of a loop with two entries is meant to get to the end of the loop, like today's walk of
 * all paths. A state that is within the state of the iteration before it is on its way there:
 *	for (i = 7; i >= 0; i--) { if (i >= n) continue; ... }
 * i is in 0..6, then in 0..5. So here too only states with the same number are compared.
 *
 * Where the iterations are not kept apart a state stands for many of them, and the number says
 * nothing about what it describes.
 */
static bool loop_stack_safe(struct bpf_verifier_env *env, struct bpf_func_state *old,
			    struct bpf_func_state *cur)
{
	struct loop_stack_entry *old_stack = old->loop_stack;
	struct loop_stack_entry *cur_stack = cur->loop_stack;
	u32 i;

	if (old->loop_stack_cnt != cur->loop_stack_cnt)
		return false;

	for (i = 0; i < old->loop_stack_cnt; i++) {
		if (old_stack[i].loop_id == cur_stack[i].loop_id &&
		    old_stack[i].max_header_count == cur_stack[i].max_header_count &&
		    old_stack[i].terminates == cur_stack[i].terminates &&
		    old_stack[i].guarded == cur_stack[i].guarded &&
		    old_stack[i].counted == cur_stack[i].counted &&
		    old_stack[i].apart == cur_stack[i].apart &&
		    (!old_stack[i].apart ||
		     (old_stack[i].probe ? old_stack[i].trip <= cur_stack[i].trip :
					   old_stack[i].trip == cur_stack[i].trip)))
			continue;
		return false;
	}

	return true;
}

/* compare two verifier states
 *
 * all states stored in state_list are known to be valid, since
 * verifier reached 'bpf_exit' instruction through them
 *
 * this function is called when verifier exploring different branches of
 * execution popped from the state stack. If it sees an old state that has
 * more strict register state and more strict stack state then this execution
 * branch doesn't need to be explored further, since verifier already
 * concluded that more strict state leads to valid finish.
 *
 * Therefore two states are equivalent if register state is more conservative
 * and explored stack state is more conservative than the current one.
 * Example:
 *       explored                   current
 * (slot1=INV slot2=MISC) == (slot1=MISC slot2=MISC)
 * (slot1=MISC slot2=MISC) != (slot1=INV slot2=MISC)
 *
 * In other words if current stack state (one being explored) has more
 * valid slots than old one that already passed validation, it means
 * the verifier can stop exploring and conclude that current state is valid too
 *
 * Similarly with registers. If explored state has register type as invalid
 * whereas register type in current state is meaningful, it means that
 * the current state will reach 'bpf_exit' instruction safely
 */
static bool func_states_equal(struct bpf_verifier_env *env, struct bpf_func_state *old,
			      struct bpf_func_state *cur, u32 insn_idx, enum exact_level exact)
{
	u16 live_regs = env->insn_aux_data[insn_idx].live_regs_before;
	u16 i;

	if (old->callback_depth > cur->callback_depth)
		return false;

	if (!old->no_stack_arg_load && cur->no_stack_arg_load)
		return false;

	if (memcmp(&old->callback_ret_range, &cur->callback_ret_range,
		   sizeof(old->callback_ret_range)))
		return false;

	if (old->in_rcu_callback_fn != cur->in_rcu_callback_fn)
		return false;

	/* what the frame did to what its callers hold is a part of what it tells them */
	if (old->entry_refs != cur->entry_refs ||
	    old->entry_rcu_locks != cur->entry_rcu_locks ||
	    old->entry_preempt_locks != cur->entry_preempt_locks)
		return false;

	for (i = 0; i < MAX_BPF_REG; i++)
		if (((1 << i) & live_regs) &&
		    !regsafe(env, &old->regs[i], &cur->regs[i],
			     &env->idmap_scratch, exact))
			return false;

	if (!stacksafe(env, old, cur, &env->idmap_scratch, exact))
		return false;

	if (!stack_arg_safe(env, old, cur, &env->idmap_scratch, exact))
		return false;

	if (!loop_stack_safe(env, old, cur))
		return false;

	return true;
}

static void reset_idmap_scratch(struct bpf_verifier_env *env)
{
	struct bpf_idmap *idmap = &env->idmap_scratch;

	idmap->tmp_id_gen = env->id_gen;
	idmap->cnt = 0;
}

static bool states_equal(struct bpf_verifier_env *env,
			 struct bpf_verifier_state *old,
			 struct bpf_verifier_state *cur,
			 enum exact_level exact)
{
	u32 insn_idx;
	int i;

	if (old->curframe != cur->curframe)
		return false;

	reset_idmap_scratch(env);

	/* Verification state from speculative execution simulation
	 * must never prune a non-speculative execution one.
	 */
	if (old->speculative && !cur->speculative)
		return false;

	if (old->callers_unknown != cur->callers_unknown)
		return false;

	if (old->in_sleepable != cur->in_sleepable)
		return false;

	if (old->unwound != cur->unwound || old->changed_pkt_data != cur->changed_pkt_data)
		return false;

	if (!refsafe(old, cur, &env->idmap_scratch))
		return false;

	/* for states to be equal callsites have to be the same
	 * and all frame states need to be equivalent
	 */
	for (i = 0; i <= old->curframe; i++) {
		insn_idx = bpf_frame_insn_idx(old, i);
		if (old->frame[i]->callsite != cur->frame[i]->callsite)
			return false;
		if (!func_states_equal(env, old->frame[i], cur->frame[i], insn_idx, exact))
			return false;
	}

	return true;
}

int bpf_clean_verifier_state(struct bpf_verifier_env *env, struct bpf_verifier_state *st)
{
	return clean_verifier_state(env, st);
}

/* Everything that @cur describes is described by @old. */
bool bpf_state_subsumes(struct bpf_verifier_env *env, struct bpf_verifier_state *old,
			struct bpf_verifier_state *cur)
{
	return states_equal(env, old, cur, RANGE_WITHIN);
}
