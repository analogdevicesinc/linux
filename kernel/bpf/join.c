// SPDX-License-Identifier: GPL-2.0-only
/*
 * Join of verifier states.
 *
 * bpf_states_join(dst, a, b) computes the state that describes every concrete
 * state described by 'a' or by 'b'. With @widen a number that is larger in the
 * result than in 'a' gets the bound that moved put further out, so that a sequence
 * of states at a loop head ends. A pointer whose offset is larger has no join.
 *
 * Numbers and offsets of pointers are joined. A pointer and a number, pointers
 * to different objects, different references and locks are not: there is no
 * join and the states stay apart.
 *
 * The contract is checked on every call: both inputs have to be within the
 * result, see bpf_state_subsumes(). A violation is a verifier bug.
 *
 * Inputs have to be at the same insn and cleaned with
 * bpf_clean_verifier_state(): dead registers and stack slots are the same on
 * both sides then, but for dynptrs, iterators and irq flags: they stay.
 */
#include <linux/bpf.h>
#include <linux/bpf_verifier.h>
#include <linux/cnum.h>
#include <linux/tnum.h>
#include <linux/filter.h>
#include <linux/slab.h>

struct join_idpair {
	u32 a;
	u32 b;
	bool bad;
};

struct bpf_join_state {
	struct bpf_verifier_env *env;
	struct join_idpair pairs[BPF_ID_MAP_SIZE];
	u32 npairs;
	bool pairs_overflow;
	bool widen;
	/* the states came around this loop in the frame that is joined, NULL if not */
	const struct bpf_loop *loop;
	/* the register or stack slot that is joined, as in struct bpf_loop_rungs, or -1 */
	int place;
};

static struct bpf_join_state *join_state(struct bpf_verifier_env *env)
{
	if (!env->join) {
		env->join = kvzalloc_obj(*env->join, GFP_KERNEL_ACCOUNT);
		if (env->join)
			env->join->env = env;
	}
	return env->join;
}

void bpf_join_free(struct bpf_verifier_env *env)
{
	kvfree(env->join);
	env->join = NULL;
}

/*
 * An id in the result stands for the pair (id in 'a', id in 'b'). For the
 * result to be within both inputs the pairs have to form a bijection,
 * see check_ids(). The first pass over all locations records the pairs. An id
 * seen with two partners makes all pairs it takes part in bad.
 * The second pass keeps the id of 'a' for good pairs.
 */
static void jid_mark_bad(struct bpf_join_state *js, u32 a, u32 b)
{
	u32 i;

	for (i = 0; i < js->npairs; i++)
		if ((a && js->pairs[i].a == a) || (b && js->pairs[i].b == b))
			js->pairs[i].bad = true;
}

static void jid_note(struct bpf_join_state *js, u32 ida, u32 idb, bool consistent)
{
	u32 a = ida & ~BPF_ADD_CONST, b = idb & ~BPF_ADD_CONST;
	struct join_idpair *p;
	bool other = false;
	u32 i;

	if (!a || !b)
		return;
	/* a name of what the caller has, see BPF_INHERITED_ID */
	if (a != b && (a | b) >= BPF_INHERITED_ID)
		consistent = false;
	for (i = 0; i < js->npairs; i++) {
		p = &js->pairs[i];
		if (p->a == a && p->b == b) {
			if (!consistent)
				jid_mark_bad(js, a, b);
			return;
		}
		if (p->a == a || p->b == b)
			other = true;
	}
	if (other) {
		jid_mark_bad(js, a, b);
		consistent = false;
	}
	if (js->npairs == ARRAY_SIZE(js->pairs)) {
		js->pairs_overflow = true;
		return;
	}
	p = &js->pairs[js->npairs++];
	p->a = a;
	p->b = b;
	p->bad = !consistent;
}

static bool jid_good(struct bpf_join_state *js, u32 ida, u32 idb)
{
	u32 a = ida & ~BPF_ADD_CONST, b = idb & ~BPF_ADD_CONST;
	u32 i;

	if (!a || !b || js->pairs_overflow)
		return false;
	if ((ida ^ idb) & BPF_ADD_CONST)
		return false;
	for (i = 0; i < js->npairs; i++)
		if (js->pairs[i].a == a && js->pairs[i].b == b)
			return !js->pairs[i].bad;
	return false;
}

/* Ids that have to match: both zero, or a good pair */
static bool jid_strict_ok(struct bpf_join_state *js, u32 ida, u32 idb)
{
	if (!ida && !idb)
		return true;
	return jid_good(js, ida, idb);
}

static bool range_equal(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb)
{
	return ra->r64.base == rb->r64.base && ra->r64.size == rb->r64.size &&
	       ra->r32.base == rb->r32.base && ra->r32.size == rb->r32.size &&
	       ra->var_off.value == rb->var_off.value && ra->var_off.mask == rb->var_off.mask &&
	       ra->base == rb->base && ra->step == rb->step;
}

static const struct bpf_loop_rungs *join_rungs(const struct bpf_join_state *js)
{
	u32 i;

	for (i = 0; js->loop && i < js->loop->rungs_cnt; i++)
		if (js->loop->rungs[i].place == js->place)
			return &js->loop->rungs[i];
	return NULL;
}

/* Does every iteration add a constant to what is in the place? In 64 or in 32 bits, 0 if not */
static int join_place_is_linear(const struct bpf_join_state *js)
{
	/* the place as scalar evolution numbers it */
	u32 r = js->place < BPF_LOOP_SLOT ? js->place : js->place - BPF_LOOP_SLOT + MAX_BPF_REG;

	if (!js->loop || js->place < 0 || r >= BPF_SCEV_REGS_NUM)
		return 0;
	return test_bit(r, js->loop->linear_regs) ? 64 :
	       test_bit(r, js->loop->linear32_regs) ? 32 : 0;
}

/*
 * Join of the range of two registers of the same type, numbers or pointers
 * with variable offset. @dst is a copy of @ra. Returns false if there is no join.
 */
static bool join_range(struct bpf_join_state *js, struct bpf_reg_state *dst,
		       const struct bpf_reg_state *ra, const struct bpf_reg_state *rb)
{
	if (range_equal(ra, rb))
		return true;
	bpf_reg_union_range(dst, rb);
	if (!js->widen || range_equal(dst, ra))
		return true;
	/*
	 * The bound of a number that grows around a cycle is put where it may come to rest. If
	 * that is far out the number gets a bound again from the next comparison. The offset of
	 * a pointer doesn't: there is no access through a pointer at an offset that is not
	 * known. So there is nothing to gain from that join. If the offset stops to grow
	 * the next state is within the last one.
	 */
	if (dst->type != SCALAR_VALUE)
		return false;
	bpf_reg_widen_range(dst, ra, join_rungs(js), join_place_is_linear(js));
	return true;
}

static bool reg_fixed_part_equal(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb)
{
	return !memcmp(ra, rb, offsetof(struct bpf_reg_state, var_off));
}

static bool reg_value_equal(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb)
{
	return !memcmp(ra, rb, offsetof(struct bpf_reg_state, id));
}

static bool scalar_link_consistent(const struct bpf_reg_state *ra, const struct bpf_reg_state *rb)
{
	if ((ra->id ^ rb->id) & BPF_ADD_CONST)
		return false;
	if ((ra->id & BPF_ADD_CONST) && ra->delta != rb->delta)
		return false;
	return true;
}

/*
 * Join two registers, regsafe() documents what each type needs.
 * Pass 1 records id pairs, pass 2 computes @dst, which is a copy of @ra on entry.
 * Returns false if there is no join.
 */
static bool join_reg(struct bpf_join_state *js, struct bpf_reg_state *dst,
		     const struct bpf_reg_state *ra, const struct bpf_reg_state *rb, int pass)
{
	if (ra->type == NOT_INIT || rb->type == NOT_INIT) {
		if (pass == 2 && ra->type != NOT_INIT)
			*dst = *rb;
		return true;
	}

	if (ra->type != rb->type)
		return false;

	switch (base_type(ra->type)) {
	case SCALAR_VALUE:
		if (js->env->explore_alu_limits && !reg_value_equal(ra, rb))
			return false;
		if (pass == 1) {
			jid_note(js, ra->id, rb->id, scalar_link_consistent(ra, rb));
			return true;
		}
		join_range(js, dst, ra, rb);
		/* a link that exists on one side only is dropped */
		if (ra->id && !jid_good(js, ra->id, rb->id)) {
			dst->id = 0;
			dst->delta = 0;
		}
		return true;
	case PTR_TO_MAP_KEY:
	case PTR_TO_MAP_VALUE:
	case PTR_TO_MEM:
	case PTR_TO_BUF:
	case PTR_TO_TP_BUFFER:
		if (!reg_fixed_part_equal(ra, rb))
			return false;
		/*
		 * Two places in an object are two pointers: what is at a place
		 * and how much is left behind it is lost in a range of them.
		 */
		if (!js->widen && tnum_is_const(ra->var_off) && tnum_is_const(rb->var_off) &&
		    ra->var_off.value != rb->var_off.value)
			return false;
		break;
	case PTR_TO_PACKET_META:
	case PTR_TO_PACKET:
		if ((ra->range < 0 || rb->range < 0) && ra->range != rb->range)
			return false;
		/* how much of the packet was checked goes with numbers elsewhere in the state */
		if (!js->widen && ra->range != rb->range)
			return false;
		/*
		 * Displacements between pointers that share an id are kept. No id is an id too:
		 * the start of the packet plus a constant, see find_good_pkt_pointers().
		 */
		if (ra->r64.base != rb->r64.base)
			return false;
		if (pass == 1) {
			jid_note(js, ra->id, rb->id, true);
			return true;
		}
		if (!jid_strict_ok(js, ra->id, rb->id))
			return false;
		if (rb->range != ra->range)
			dst->range = min(ra->range, rb->range);
		return join_range(js, dst, ra, rb);
	case PTR_TO_ARENA:
		/* The offset is not tracked. */
		return true;
	case PTR_TO_INSN:
		if (!reg_fixed_part_equal(ra, rb))
			return false;
		return pass == 1 || join_range(js, dst, ra, rb);
	default:
		if (!reg_value_equal(ra, rb))
			return false;
		break;
	}

	if (pass == 1) {
		jid_note(js, ra->id, rb->id, true);
		jid_note(js, ra->parent_id, rb->parent_id, true);
		jid_note(js, ra->map_uid, rb->map_uid, true);
		return true;
	}
	if (!jid_strict_ok(js, ra->id, rb->id) ||
	    !jid_strict_ok(js, ra->parent_id, rb->parent_id) ||
	    !jid_strict_ok(js, ra->map_uid, rb->map_uid))
		return false;
	return join_range(js, dst, ra, rb);
}

static bool slot_is_special(const struct bpf_stack_state *s)
{
	u8 t = s->slot_type[BPF_REG_SIZE - 1];

	return t == STACK_DYNPTR || t == STACK_ITER || t == STACK_IRQ_FLAG;
}

static bool stype_is_invalid(u8 t)
{
	return t == STACK_INVALID || t == STACK_POISON;
}

static bool slot_has_spill(const struct bpf_stack_state *s)
{
	int i;

	for (i = 0; i < BPF_REG_SIZE; i++)
		if (s->slot_type[i] == STACK_SPILL)
			return true;
	return false;
}

static bool join_special_slot(struct bpf_join_state *js, const struct bpf_stack_state *sa,
			      const struct bpf_stack_state *sb, int pass)
{
	const struct bpf_reg_state *ra = &sa->spilled_ptr, *rb = &sb->spilled_ptr;

	if (memcmp(sa->slot_type, sb->slot_type, sizeof(sa->slot_type)))
		return false;
	switch (sa->slot_type[BPF_REG_SIZE - 1]) {
	case STACK_DYNPTR:
		if (ra->dynptr.type != rb->dynptr.type ||
		    ra->dynptr.first_slot != rb->dynptr.first_slot)
			return false;
		break;
	case STACK_ITER:
		if (ra->type != rb->type || ra->iter.btf != rb->iter.btf ||
		    ra->iter.btf_id != rb->iter.btf_id || ra->iter.state != rb->iter.state ||
		    ra->iter.loops != rb->iter.loops)
			return false;
		break;
	case STACK_IRQ_FLAG:
		if (ra->irq.kfunc_class != rb->irq.kfunc_class)
			return false;
		break;
	}
	if (pass == 1) {
		jid_note(js, ra->id, rb->id, true);
		jid_note(js, ra->parent_id, rb->parent_id, true);
		return true;
	}
	return jid_strict_ok(js, ra->id, rb->id) &&
	       jid_strict_ok(js, ra->parent_id, rb->parent_id);
}

/*
 * Join of one 8 byte stack slot, @ds is a copy of @sa on entry.
 * stacksafe() documents which slot is within which.
 */
static bool join_slot(struct bpf_join_state *js, struct bpf_stack_state *ds,
		      const struct bpf_stack_state *sa, const struct bpf_stack_state *sb, int pass)
{
	static const struct bpf_stack_state invalid_slot;
	bool a_ptr, b_ptr;
	u8 res[BPF_REG_SIZE];
	int i;

	if (!sb)
		sb = &invalid_slot;

	if (slot_is_special(sa) || slot_is_special(sb))
		return join_special_slot(js, sa, sb, pass);

	if (!memcmp(sa->slot_type, sb->slot_type, sizeof(sa->slot_type)) &&
	    sa->slot_type[BPF_REG_SIZE - 1] == STACK_SPILL)
		return join_reg(js, &ds->spilled_ptr, &sa->spilled_ptr, &sb->spilled_ptr, pass);

	a_ptr = slot_has_spill(sa) && sa->spilled_ptr.type != SCALAR_VALUE;
	b_ptr = slot_has_spill(sb) && sb->spilled_ptr.type != SCALAR_VALUE;
	/* a pointer and something else */
	if (a_ptr || b_ptr)
		return false;

	/* the prog can't read uninitialized stack: slots of different kinds have no join */
	if (!js->env->allow_uninit_stack)
		return !memcmp(sa->slot_type, sb->slot_type, sizeof(sa->slot_type));

	/* scalar spills, MISC, ZERO, INVALID, POISON: no ids are kept */
	if (pass == 1)
		return true;

	for (i = 0; i < BPF_REG_SIZE; i++) {
		u8 x = sa->slot_type[i], y = sb->slot_type[i];

		if (x == STACK_SPILL)
			x = STACK_MISC;
		if (y == STACK_SPILL)
			y = STACK_MISC;
		if (stype_is_invalid(x) && stype_is_invalid(y))
			res[i] = sa->slot_type[i];
		else if (stype_is_invalid(x) || stype_is_invalid(y))
			res[i] = STACK_INVALID;
		else if (x == y)
			res[i] = x;
		else
			res[i] = STACK_MISC;
	}
	memcpy(ds->slot_type, res, sizeof(res));
	bpf_mark_reg_not_init(js->env, &ds->spilled_ptr);
	return true;
}

static bool join_loop_stack(const struct bpf_func_state *fa, const struct bpf_func_state *fb)
{
	u32 i;

	if (fa->loop_stack_cnt != fb->loop_stack_cnt)
		return false;
	/* what is known about a counted loop comes from the state it was entered with */
	for (i = 0; i < fa->loop_stack_cnt; i++)
		if (fa->loop_stack[i].loop_id != fb->loop_stack[i].loop_id ||
		    fa->loop_stack[i].terminates != fb->loop_stack[i].terminates ||
		    fa->loop_stack[i].guarded != fb->loop_stack[i].guarded ||
		    fa->loop_stack[i].counted != fb->loop_stack[i].counted ||
		    fa->loop_stack[i].apart != fb->loop_stack[i].apart ||
		    fa->loop_stack[i].first != fb->loop_stack[i].first ||
		    (fa->loop_stack[i].apart && fa->loop_stack[i].trip != fb->loop_stack[i].trip) ||
		    fa->loop_stack[i].refs != fb->loop_stack[i].refs ||
		    fa->loop_stack[i].max_header_count != fb->loop_stack[i].max_header_count ||
		    fa->loop_stack[i].entry_state != fb->loop_stack[i].entry_state)
			return false;
	return true;
}

static bool join_func(struct bpf_join_state *js, struct bpf_func_state *fd,
		      const struct bpf_func_state *fa, const struct bpf_func_state *fb,
		      u16 live_regs, int pass)
{
	static const struct bpf_reg_state not_init = { .type = NOT_INIT };
	struct bpf_verifier_env *env = js->env;
	u32 i, nslots_b;
	bool in_a, in_b;

	if (pass == 1) {
		if (fa->callsite != fb->callsite || fa->frameno != fb->frameno ||
		    fa->subprogno != fb->subprogno)
			return false;
		if (fa->in_callback_fn != fb->in_callback_fn ||
		    fa->in_async_callback_fn != fb->in_async_callback_fn ||
		    fa->in_exception_callback_fn != fb->in_exception_callback_fn ||
		    fa->in_rcu_callback_fn != fb->in_rcu_callback_fn ||
		    fa->async_entry_cnt != fb->async_entry_cnt ||
		    memcmp(&fa->callback_ret_range, &fb->callback_ret_range,
			   sizeof(fa->callback_ret_range)))
			return false;
		if (fa->entry_refs != fb->entry_refs ||
		    fa->entry_rcu_locks != fb->entry_rcu_locks ||
		    fa->entry_preempt_locks != fb->entry_preempt_locks)
			return false;
		if (!join_loop_stack(fa, fb))
			return false;
	}

	for (i = 0; i < MAX_BPF_REG; i++) {
		if (!(live_regs & BIT(i))) {
			if (pass == 2 && i < BPF_REG_FP)
				bpf_mark_reg_not_init(env, &fd->regs[i]);
			continue;
		}
		js->place = i;
		if (!join_reg(js, &fd->regs[i], &fa->regs[i], &fb->regs[i], pass))
			return false;
	}

	nslots_b = bpf_stack_nr_slots(fb);
	for (i = 0; i < bpf_stack_nr_slots(fa); i++) {
		const struct bpf_stack_state *sb = i < nslots_b ? bpf_stack_slot(fb, i) : NULL;

		js->place = BPF_LOOP_SLOT + i;
		if (!join_slot(js, bpf_stack_slot(fd, i), bpf_stack_slot(fa, i), sb, pass))
			return false;
	}
	js->place = -1;
	/* Slots that exist only in 'b' are STACK_INVALID in the result. */
	for (i = bpf_stack_nr_slots(fa); pass == 1 && i < nslots_b; i++) {
		const struct bpf_stack_state *sb = bpf_stack_slot(fb, i);

		if (slot_is_special(sb) ||
		    (slot_has_spill(sb) && sb->spilled_ptr.type != SCALAR_VALUE))
			return false;
	}

	if (fa->out_stack_arg_cnt < fb->out_stack_arg_cnt)
		return false;
	/* incoming arguments are not joined with outgoing ones: that side has no outgoing ones */
	in_a = bpf_stack_args_are_incoming(fa) && !bpf_stack_args_are_incoming(fb);
	in_b = bpf_stack_args_are_incoming(fb) && !bpf_stack_args_are_incoming(fa);
	for (i = 0; i < fa->out_stack_arg_cnt; i++) {
		const struct bpf_reg_state *rb;

		if (in_a) {
			if (pass == 2)
				bpf_mark_reg_not_init(env, &fd->stack_arg_regs[i]);
			continue;
		}
		rb = i < fb->out_stack_arg_cnt && !in_b ? &fb->stack_arg_regs[i] : &not_init;
		if (!join_reg(js, &fd->stack_arg_regs[i], &fa->stack_arg_regs[i], rb, pass))
			return false;
	}

	if (pass == 2) {
		fd->no_stack_arg_load = fa->no_stack_arg_load || fb->no_stack_arg_load;
		fd->callback_depth = min(fa->callback_depth, fb->callback_depth);
		for (i = 0; i < fd->loop_stack_cnt; i++)
			fd->loop_stack[i].trip = max(fd->loop_stack[i].trip, fb->loop_stack[i].trip);
	}
	return true;
}

/* Things that can't be different in two states that are joined */
static bool join_check_state(const struct bpf_verifier_state *a,
			     const struct bpf_verifier_state *b)
{
	int i;

	if (a->curframe != b->curframe || a->insn_idx != b->insn_idx)
		return false;
	if (a->speculative != b->speculative || a->in_sleepable != b->in_sleepable ||
	    a->callers_unknown != b->callers_unknown ||
	    a->unwound != b->unwound || a->changed_pkt_data != b->changed_pkt_data)
		return false;
	if (a->active_locks != b->active_locks ||
	    a->active_preempt_locks != b->active_preempt_locks ||
	    a->active_rcu_locks != b->active_rcu_locks ||
	    a->active_lock_ptr != b->active_lock_ptr ||
	    !a->active_irq_id != !b->active_irq_id ||
	    !a->active_lock_id != !b->active_lock_id)
		return false;
	if (a->acquired_refs != b->acquired_refs)
		return false;
	for (i = 0; i < a->acquired_refs; i++) {
		if (a->refs[i].type != b->refs[i].type)
			return false;
		switch (a->refs[i].type) {
		case REF_TYPE_PTR:
		case REF_TYPE_IRQ:
			break;
		default:
			if (a->refs[i].ptr != b->refs[i].ptr)
				return false;
		}
	}
	return true;
}

static void join_refs_note(struct bpf_join_state *js, const struct bpf_verifier_state *a,
			   const struct bpf_verifier_state *b)
{
	int i;

	jid_note(js, a->active_irq_id, b->active_irq_id, true);
	jid_note(js, a->active_lock_id, b->active_lock_id, true);
	for (i = 0; i < a->acquired_refs; i++) {
		jid_note(js, a->refs[i].id, b->refs[i].id, true);
		if (a->refs[i].type == REF_TYPE_PTR)
			jid_note(js, a->refs[i].parent_id, b->refs[i].parent_id, true);
	}
}

static bool join_refs_check(struct bpf_join_state *js, const struct bpf_verifier_state *a,
			    const struct bpf_verifier_state *b)
{
	int i;

	if (!jid_strict_ok(js, a->active_irq_id, b->active_irq_id) ||
	    !jid_strict_ok(js, a->active_lock_id, b->active_lock_id))
		return false;
	for (i = 0; i < a->acquired_refs; i++) {
		if (!jid_strict_ok(js, a->refs[i].id, b->refs[i].id))
			return false;
		if (a->refs[i].type == REF_TYPE_PTR &&
		    !jid_strict_ok(js, a->refs[i].parent_id, b->refs[i].parent_id))
			return false;
	}
	return true;
}

/* The result has to describe everything that an input describes */
static int join_check(struct bpf_join_state *js, struct bpf_verifier_state *dst,
		      const struct bpf_verifier_state *in, const char *which)
{
	struct bpf_verifier_env *env = js->env;

	if (bpf_state_subsumes(env, dst, (struct bpf_verifier_state *)in))
		return 0;
	/* check_ids() has no room left: that says nothing about the join */
	if (env->idmap_scratch.cnt == BPF_ID_MAP_SIZE)
		return 1;
	bpf_log(&env->log, "join:");
	print_verifier_state(env, dst, dst->curframe, true);
	bpf_log(&env->log, "input %s:", which);
	print_verifier_state(env, in, in->curframe, true);
	verifier_bug(env, "join of states at insn %u does not subsume input '%s'",
		     dst->insn_idx, which);
	return -EFAULT;
}

/*
 * Compute in @dst the state that describes everything that @a or @b describe.
 * @dst has to be a valid state (zeroed is fine) different from @a and @b, its
 * previous content is released. Bookkeeping fields are copied from @a.
 *
 * Returns 0 on success, 1 if there is no join, negative error otherwise.
 * When 1 is returned the content of @dst is undefined, but it's a valid state.
 */
int bpf_states_join(struct bpf_verifier_env *env, struct bpf_verifier_state *dst,
		    const struct bpf_verifier_state *a, const struct bpf_verifier_state *b,
		    bool widen)
{
	struct bpf_join_state *js = join_state(env);
	const struct bpf_func_state *frame;
	int pass, fr, err;

	if (!js)
		return -ENOMEM;
	js->widen = widen;
	js->npairs = 0;
	js->pairs_overflow = false;

	if (!join_check_state(a, b))
		return 1;
	err = bpf_copy_verifier_state(dst, a);
	if (err)
		return err;

	for (pass = 1; pass <= 2; pass++) {
		if (pass == 1)
			join_refs_note(js, a, b);
		else if (!join_refs_check(js, a, b))
			return 1;
		for (fr = 0; fr <= a->curframe; fr++) {
			u32 insn_idx = bpf_frame_insn_idx((struct bpf_verifier_state *)a, fr);
			u16 live_regs = env->insn_aux_data[insn_idx].live_regs_before;
			u32 cnt;

			/* not the states that get to the head through a call of the function */
			frame = a->frame[fr];
			cnt = frame->loop_stack_cnt;
			js->loop = widen && fr == a->curframe && cnt &&
				   frame->loop_stack[cnt - 1].loop_id == insn_idx ?
				   env->insn_aux_data[insn_idx].loop : NULL;
			if (!join_func(js, dst->frame[fr], a->frame[fr], b->frame[fr],
				       live_regs, pass))
				return 1;
		}
	}

	return join_check(js, dst, a, "a") ?: join_check(js, dst, b, "b");
}
