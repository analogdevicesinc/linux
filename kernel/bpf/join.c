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
	/* the register or stack slot that is joined, as scalar evolution numbers it, or -1 */
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

/* Does every iteration add a constant to what is in the place? In 64 or in 32 bits, 0 if not */
static int join_place_is_linear(const struct bpf_join_state *js)
{
	if (!js->loop || js->place < 0 || js->place >= BPF_SCEV_REGS_NUM)
		return 0;
	return test_bit(js->place, js->loop->linear_regs) ? 64 :
	       test_bit(js->place, js->loop->linear32_regs) ? 32 : 0;
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
	bpf_reg_widen_range(dst, ra, join_place_is_linear(js));
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
