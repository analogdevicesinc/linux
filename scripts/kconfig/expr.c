// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2002 Roman Zippel <zippel@linux-m68k.org>
 */

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hash.h>
#include <xalloc.h>
#include "internal.h"
#include "lkc.h"

#define DEBUG_EXPR	0

HASHTABLE_DEFINE(expr_hashtable, EXPR_HASHSIZE);

static struct expr *expr_eliminate_yn(struct expr *e);

/**
 * expr_lookup - return the expression with the given type and sub-nodes
 * This looks up an expression with the specified type and sub-nodes. If such
 * an expression is found in the hash table, it is returned. Otherwise, a new
 * expression node is allocated and added to the hash table.
 * @type: expression type
 * @l: left node
 * @r: right node
 * return: expression
 */
static struct expr *expr_lookup(enum expr_type type, void *l, void *r)
{
	struct expr *e;
	int hash;

	hash = hash_32((unsigned int)type ^ hash_ptr(l) ^ hash_ptr(r));

	hash_for_each_possible(expr_hashtable, e, node, hash) {
		if (e->type == type && e->left._initdata == l &&
		    e->right._initdata == r)
			return e;
	}

	e = xmalloc(sizeof(*e));
	e->type = type;
	e->left._initdata = l;
	e->right._initdata = r;
	e->val_is_valid = false;

	hash_add(expr_hashtable, &e->node, hash);

	return e;
}

struct expr *expr_alloc_symbol(struct symbol *sym)
{
	return expr_lookup(E_SYMBOL, sym, NULL);
}

struct expr *expr_alloc_one(enum expr_type type, struct expr *ce)
{
	return expr_lookup(type, ce, NULL);
}

struct expr *expr_alloc_two(enum expr_type type, struct expr *e1, struct expr *e2)
{
	return expr_lookup(type, e1, e2);
}

struct expr *expr_alloc_comp(enum expr_type type, struct symbol *s1, struct symbol *s2)
{
	return expr_lookup(type, s1, s2);
}

struct expr *expr_alloc_and(struct expr *e1, struct expr *e2)
{
	if (!e1)
		return e2;
	return e2 ? expr_alloc_two(E_AND, e1, e2) : e1;
}

struct expr *expr_alloc_or(struct expr *e1, struct expr *e2)
{
	if (!e1)
		return e2;
	return e2 ? expr_alloc_two(E_OR, e1, e2) : e1;
}

static int trans_count;

/*
 * expr_eliminate_eq() helper.
 *
 * Walks the two expression trees given in 'ep1' and 'ep2'. Any node that does
 * not have type 'type' (E_OR/E_AND) is considered a leaf, and is compared
 * against all other leaves. Two equal leaves are both replaced with either 'y'
 * or 'n' as appropriate for 'type', to be eliminated later.
 */
static void __expr_eliminate_eq(enum expr_type type, struct expr **ep1, struct expr **ep2)
{
	struct expr *l, *r;

	/* Recurse down to leaves */

	if ((*ep1)->type == type) {
		l = (*ep1)->left.expr;
		r = (*ep1)->right.expr;
		__expr_eliminate_eq(type, &l, ep2);
		__expr_eliminate_eq(type, &r, ep2);
		*ep1 = expr_alloc_two(type, l, r);
		return;
	}
	if ((*ep2)->type == type) {
		l = (*ep2)->left.expr;
		r = (*ep2)->right.expr;
		__expr_eliminate_eq(type, ep1, &l);
		__expr_eliminate_eq(type, ep1, &r);
		*ep2 = expr_alloc_two(type, l, r);
		return;
	}

	/* *ep1 and *ep2 are leaves. Compare them. */

	if ((*ep1)->type == E_SYMBOL && (*ep2)->type == E_SYMBOL &&
	    (*ep1)->left.sym == (*ep2)->left.sym &&
	    ((*ep1)->left.sym == &symbol_yes || (*ep1)->left.sym == &symbol_no))
		return;
	if (!expr_eq(*ep1, *ep2))
		return;

	/* *ep1 and *ep2 are equal leaves. Prepare them for elimination. */

	trans_count++;
	switch (type) {
	case E_OR:
		*ep1 = expr_alloc_symbol(&symbol_no);
		*ep2 = expr_alloc_symbol(&symbol_no);
		break;
	case E_AND:
		*ep1 = expr_alloc_symbol(&symbol_yes);
		*ep2 = expr_alloc_symbol(&symbol_yes);
		break;
	default:
		;
	}
}

/*
 * Rewrites the expressions 'ep1' and 'ep2' to remove operands common to both.
 * Example reductions:
 *
 *	ep1: A && B           ->  ep1: y
 *	ep2: A && B && C      ->  ep2: C
 *
 *	ep1: A || B           ->  ep1: n
 *	ep2: A || B || C      ->  ep2: C
 *
 *	ep1: A && (B && FOO)  ->  ep1: FOO
 *	ep2: (BAR && B) && A  ->  ep2: BAR
 *
 *	ep1: A && (B || C)    ->  ep1: y
 *	ep2: (C || B) && A    ->  ep2: y
 *
 * Comparisons are done between all operands at the same "level" of && or ||.
 * For example, in the expression 'e1 && (e2 || e3) && (e4 || e5)', the
 * following operands will be compared:
 *
 *	- 'e1', 'e2 || e3', and 'e4 || e5', against each other
 *	- e2 against e3
 *	- e4 against e5
 *
 * Parentheses are irrelevant within a single level. 'e1 && (e2 && e3)' and
 * '(e1 && e2) && e3' are both a single level.
 *
 * See __expr_eliminate_eq() as well.
 */
void expr_eliminate_eq(struct expr **ep1, struct expr **ep2)
{
	if (!*ep1 || !*ep2)
		return;
	switch ((*ep1)->type) {
	case E_OR:
	case E_AND:
		__expr_eliminate_eq((*ep1)->type, ep1, ep2);
	default:
		;
	}
	if ((*ep1)->type != (*ep2)->type) switch ((*ep2)->type) {
	case E_OR:
	case E_AND:
		__expr_eliminate_eq((*ep2)->type, ep1, ep2);
	default:
		;
	}
	*ep1 = expr_eliminate_yn(*ep1);
	*ep2 = expr_eliminate_yn(*ep2);
}

/*
 * Returns true if 'e1' and 'e2' are equal, after minor simplification. Two
 * &&/|| expressions are considered equal if every operand in one expression
 * equals some operand in the other (operands do not need to appear in the same
 * order), recursively.
 */
bool expr_eq(struct expr *e1, struct expr *e2)
{
	int old_count;
	bool res;

	/*
	 * A NULL expr is taken to be yes, but there's also a different way to
	 * represent yes. expr_is_yes() checks for either representation.
	 */
	if (!e1 || !e2)
		return expr_is_yes(e1) && expr_is_yes(e2);

	if (e1->type != e2->type)
		return false;
	switch (e1->type) {
	case E_EQUAL:
	case E_GEQ:
	case E_GTH:
	case E_LEQ:
	case E_LTH:
	case E_UNEQUAL:
		return e1->left.sym == e2->left.sym && e1->right.sym == e2->right.sym;
	case E_SYMBOL:
		return e1->left.sym == e2->left.sym;
	case E_NOT:
		return expr_eq(e1->left.expr, e2->left.expr);
	case E_AND:
	case E_OR:
		old_count = trans_count;
		expr_eliminate_eq(&e1, &e2);
		res = (e1->type == E_SYMBOL && e2->type == E_SYMBOL &&
		       e1->left.sym == e2->left.sym);
		trans_count = old_count;
		return res;
	case E_RANGE:
	case E_NONE:
		/* panic */;
	}

	if (DEBUG_EXPR) {
		expr_fprint(e1, stdout);
		printf(" = ");
		expr_fprint(e2, stdout);
		printf(" ?\n");
	}

	return false;
}

/*
 * Recursively performs the following simplifications (as well as the
 * corresponding simplifications with swapped operands):
 *
 *	expr && n  ->  n
 *	expr && y  ->  expr
 *	expr || n  ->  expr
 *	expr || y  ->  y
 *
 * Returns the optimized expression.
 */
static struct expr *expr_eliminate_yn(struct expr *e)
{
	struct expr *l, *r;

	if (e) switch (e->type) {
	case E_AND:
		l = expr_eliminate_yn(e->left.expr);
		r = expr_eliminate_yn(e->right.expr);
		if (l->type == E_SYMBOL) {
			if (l->left.sym == &symbol_no)
				return l;
			else if (l->left.sym == &symbol_yes)
				return r;
		}
		if (r->type == E_SYMBOL) {
			if (r->left.sym == &symbol_no)
				return r;
			else if (r->left.sym == &symbol_yes)
				return l;
		}
		break;
	case E_OR:
		l = expr_eliminate_yn(e->left.expr);
		r = expr_eliminate_yn(e->right.expr);
		if (l->type == E_SYMBOL) {
			if (l->left.sym == &symbol_no)
				return r;
			else if (l->left.sym == &symbol_yes)
				return l;
		}
		if (r->type == E_SYMBOL) {
			if (r->left.sym == &symbol_no)
				return l;
			else if (r->left.sym == &symbol_yes)
				return r;
		}
		break;
	default:
		;
	}
	return e;
}

/*
 * e1 || e2 -> ?
 */
static struct expr *expr_join_or(struct expr *e1, struct expr *e2)
{
	struct expr *tmp;
	struct symbol *sym1, *sym2;

	if (expr_eq(e1, e2))
		return e1;
	if (e1->type != E_EQUAL && e1->type != E_UNEQUAL && e1->type != E_SYMBOL && e1->type != E_NOT)
		return NULL;
	if (e2->type != E_EQUAL && e2->type != E_UNEQUAL && e2->type != E_SYMBOL && e2->type != E_NOT)
		return NULL;
	if (e1->type == E_NOT) {
		tmp = e1->left.expr;
		if (tmp->type != E_EQUAL && tmp->type != E_UNEQUAL && tmp->type != E_SYMBOL)
			return NULL;
		sym1 = tmp->left.sym;
	} else
		sym1 = e1->left.sym;
	if (e2->type == E_NOT) {
		if (e2->left.expr->type != E_SYMBOL)
			return NULL;
		sym2 = e2->left.expr->left.sym;
	} else
		sym2 = e2->left.sym;
	if (sym1 != sym2)
		return NULL;
	if (sym1->type != S_BOOLEAN && sym1->type != S_TRISTATE)
		return NULL;
	if (sym1->type == S_TRISTATE) {
		if (e1->type == E_EQUAL && e2->type == E_EQUAL &&
		    ((e1->right.sym == &symbol_yes && e2->right.sym == &symbol_mod) ||
		     (e1->right.sym == &symbol_mod && e2->right.sym == &symbol_yes))) {
			// (a='y') || (a='m') -> (a!='n')
			return expr_alloc_comp(E_UNEQUAL, sym1, &symbol_no);
		}
		if (e1->type == E_EQUAL && e2->type == E_EQUAL &&
		    ((e1->right.sym == &symbol_yes && e2->right.sym == &symbol_no) ||
		     (e1->right.sym == &symbol_no && e2->right.sym == &symbol_yes))) {
			// (a='y') || (a='n') -> (a!='m')
			return expr_alloc_comp(E_UNEQUAL, sym1, &symbol_mod);
		}
		if (e1->type == E_EQUAL && e2->type == E_EQUAL &&
		    ((e1->right.sym == &symbol_mod && e2->right.sym == &symbol_no) ||
		     (e1->right.sym == &symbol_no && e2->right.sym == &symbol_mod))) {
			// (a='m') || (a='n') -> (a!='y')
			return expr_alloc_comp(E_UNEQUAL, sym1, &symbol_yes);
		}
	}
	if (sym1->type == S_BOOLEAN) {
		// a || !a -> y
		if ((e1->type == E_NOT && e1->left.expr->type == E_SYMBOL && e2->type == E_SYMBOL) ||
		    (e2->type == E_NOT && e2->left.expr->type == E_SYMBOL && e1->type == E_SYMBOL))
			return expr_alloc_symbol(&symbol_yes);
	}

	if (DEBUG_EXPR) {
		printf("optimize (");
		expr_fprint(e1, stdout);
		printf(") || (");
		expr_fprint(e2, stdout);
		printf(")?\n");
	}
	return NULL;
}

static struct expr *expr_join_and(struct expr *e1, struct expr *e2)
{
	struct expr *tmp;
	struct symbol *sym1, *sym2;

	if (expr_eq(e1, e2))
		return e1;
	if (e1->type != E_EQUAL && e1->type != E_UNEQUAL && e1->type != E_SYMBOL && e1->type != E_NOT)
		return NULL;
	if (e2->type != E_EQUAL && e2->type != E_UNEQUAL && e2->type != E_SYMBOL && e2->type != E_NOT)
		return NULL;
	if (e1->type == E_NOT) {
		tmp = e1->left.expr;
		if (tmp->type != E_EQUAL && tmp->type != E_UNEQUAL && tmp->type != E_SYMBOL)
			return NULL;
		sym1 = tmp->left.sym;
	} else
		sym1 = e1->left.sym;
	if (e2->type == E_NOT) {
		if (e2->left.expr->type != E_SYMBOL)
			return NULL;
		sym2 = e2->left.expr->left.sym;
	} else
		sym2 = e2->left.sym;
	if (sym1 != sym2)
		return NULL;
	if (sym1->type != S_BOOLEAN && sym1->type != S_TRISTATE)
		return NULL;

	if ((e1->type == E_SYMBOL && e2->type == E_EQUAL && e2->right.sym == &symbol_yes) ||
	    (e2->type == E_SYMBOL && e1->type == E_EQUAL && e1->right.sym == &symbol_yes))
		// (a) && (a='y') -> (a='y')
		return expr_alloc_comp(E_EQUAL, sym1, &symbol_yes);

	if ((e1->type == E_SYMBOL && e2->type == E_UNEQUAL && e2->right.sym == &symbol_no) ||
	    (e2->type == E_SYMBOL && e1->type == E_UNEQUAL && e1->right.sym == &symbol_no))
		// (a) && (a!='n') -> (a)
		return expr_alloc_symbol(sym1);

	if ((e1->type == E_SYMBOL && e2->type == E_UNEQUAL && e2->right.sym == &symbol_mod) ||
	    (e2->type == E_SYMBOL && e1->type == E_UNEQUAL && e1->right.sym == &symbol_mod))
		// (a) && (a!='m') -> (a='y')
		return expr_alloc_comp(E_EQUAL, sym1, &symbol_yes);

	if (sym1->type == S_TRISTATE) {
		if (e1->type == E_EQUAL && e2->type == E_UNEQUAL) {
			// (a='b') && (a!='c') -> 'b'='c' ? 'n' : a='b'
			sym2 = e1->right.sym;
			if ((e2->right.sym->flags & SYMBOL_CONST) && (sym2->flags & SYMBOL_CONST))
				return sym2 != e2->right.sym ? expr_alloc_comp(E_EQUAL, sym1, sym2)
							     : expr_alloc_symbol(&symbol_no);
		}
		if (e1->type == E_UNEQUAL && e2->type == E_EQUAL) {
			// (a='b') && (a!='c') -> 'b'='c' ? 'n' : a='b'
			sym2 = e2->right.sym;
			if ((e1->right.sym->flags & SYMBOL_CONST) && (sym2->flags & SYMBOL_CONST))
				return sym2 != e1->right.sym ? expr_alloc_comp(E_EQUAL, sym1, sym2)
							     : expr_alloc_symbol(&symbol_no);
		}
		if (e1->type == E_UNEQUAL && e2->type == E_UNEQUAL &&
			   ((e1->right.sym == &symbol_yes && e2->right.sym == &symbol_no) ||
			    (e1->right.sym == &symbol_no && e2->right.sym == &symbol_yes)))
			// (a!='y') && (a!='n') -> (a='m')
			return expr_alloc_comp(E_EQUAL, sym1, &symbol_mod);

		if (e1->type == E_UNEQUAL && e2->type == E_UNEQUAL &&
			   ((e1->right.sym == &symbol_yes && e2->right.sym == &symbol_mod) ||
			    (e1->right.sym == &symbol_mod && e2->right.sym == &symbol_yes)))
			// (a!='y') && (a!='m') -> (a='n')
			return expr_alloc_comp(E_EQUAL, sym1, &symbol_no);

		if (e1->type == E_UNEQUAL && e2->type == E_UNEQUAL &&
			   ((e1->right.sym == &symbol_mod && e2->right.sym == &symbol_no) ||
			    (e1->right.sym == &symbol_no && e2->right.sym == &symbol_mod)))
			// (a!='m') && (a!='n') -> (a='m')
			return expr_alloc_comp(E_EQUAL, sym1, &symbol_yes);

		if ((e1->type == E_SYMBOL && e2->type == E_EQUAL && e2->right.sym == &symbol_mod) ||
		    (e2->type == E_SYMBOL && e1->type == E_EQUAL && e1->right.sym == &symbol_mod) ||
		    (e1->type == E_SYMBOL && e2->type == E_UNEQUAL && e2->right.sym == &symbol_yes) ||
		    (e2->type == E_SYMBOL && e1->type == E_UNEQUAL && e1->right.sym == &symbol_yes))
			return NULL;
	}

	if (DEBUG_EXPR) {
		printf("optimize (");
		expr_fprint(e1, stdout);
		printf(") && (");
		expr_fprint(e2, stdout);
		printf(")?\n");
	}
	return NULL;
}

/*
 * expr_eliminate_dups() helper.
 *
 * Walks the two expression trees given in 'ep1' and 'ep2'. Any node that does
 * not have type 'type' (E_OR/E_AND) is considered a leaf, and is compared
 * against all other leaves to look for simplifications.
 */
static void expr_eliminate_dups1(enum expr_type type, struct expr **ep1, struct expr **ep2)
{
	struct expr *tmp, *l, *r;

	/* Recurse down to leaves */

	if ((*ep1)->type == type) {
		l = (*ep1)->left.expr;
		r = (*ep1)->right.expr;
		expr_eliminate_dups1(type, &l, ep2);
		expr_eliminate_dups1(type, &r, ep2);
		*ep1 = expr_alloc_two(type, l, r);
		return;
	}
	if ((*ep2)->type == type) {
		l = (*ep2)->left.expr;
		r = (*ep2)->right.expr;
		expr_eliminate_dups1(type, ep1, &l);
		expr_eliminate_dups1(type, ep1, &r);
		*ep2 = expr_alloc_two(type, l, r);
		return;
	}

	/* *ep1 and *ep2 are leaves. Compare and process them. */

	switch (type) {
	case E_OR:
		tmp = expr_join_or(*ep1, *ep2);
		if (tmp) {
			*ep1 = expr_alloc_symbol(&symbol_no);
			*ep2 = tmp;
			trans_count++;
		}
		break;
	case E_AND:
		tmp = expr_join_and(*ep1, *ep2);
		if (tmp) {
			*ep1 = expr_alloc_symbol(&symbol_yes);
			*ep2 = tmp;
			trans_count++;
		}
		break;
	default:
		;
	}
}

/*
 * Rewrites 'e' in-place to remove ("join") duplicate and other redundant
 * operands.
 *
 * Example simplifications:
 *
 *	A || B || A    ->  A || B
 *	A && B && A=y  ->  A=y && B
 *
 * Returns the deduplicated expression.
 */
struct expr *expr_eliminate_dups(struct expr *e)
{
	int oldcount;
	if (!e)
		return e;

	oldcount = trans_count;
	do {
		struct expr *l, *r;

		trans_count = 0;
		switch (e->type) {
		case E_OR: case E_AND:
			l = expr_eliminate_dups(e->left.expr);
			r = expr_eliminate_dups(e->right.expr);
			expr_eliminate_dups1(e->type, &l, &r);
			e = expr_alloc_two(e->type, l, r);
		default:
			;
		}
		e = expr_eliminate_yn(e);
	} while (trans_count); /* repeat until we get no more simplifications */
	trans_count = oldcount;
	return e;
}

/*
 * Performs various simplifications involving logical operators and
 * comparisons.
 *
 *   For bool type:
 *     A=n        ->  !A
 *     A=m        ->  n
 *     A=y        ->  A
 *     A!=n       ->  A
 *     A!=m       ->  y
 *     A!=y       ->  !A
 *
 *   For any type:
 *     !!A        ->  A
 *     !(A=B)     ->  A!=B
 *     !(A!=B)    ->  A=B
 *     !(A<=B)    ->  A>B
 *     !(A>=B)    ->  A<B
 *     !(A<B)     ->  A>=B
 *     !(A>B)     ->  A<=B
 *     !(A || B)  ->  !A && !B
 *     !(A && B)  ->  !A || !B
 *
 *   For constant:
 *     !y         ->  n
 *     !m         ->  m
 *     !n         ->  y
 *
 * Allocates and returns a new expression.
 */
struct expr *expr_transform(struct expr *e)
{
	if (!e)
		return NULL;
	switch (e->type) {
	case E_EQUAL:
	case E_GEQ:
	case E_GTH:
	case E_LEQ:
	case E_LTH:
	case E_UNEQUAL:
	case E_SYMBOL:
		break;
	default:
		e = expr_alloc_two(e->type,
				   expr_transform(e->left.expr),
				   expr_transform(e->right.expr));
	}

	switch (e->type) {
	case E_EQUAL:
		if (e->left.sym->type != S_BOOLEAN)
			break;
		if (e->right.sym == &symbol_no) {
			// A=n -> !A
			e = expr_alloc_one(E_NOT, expr_alloc_symbol(e->left.sym));
			break;
		}
		if (e->right.sym == &symbol_mod) {
			// A=m -> n
			printf("boolean symbol %s tested for 'm'? test forced to 'n'\n", e->left.sym->name);
			e = expr_alloc_symbol(&symbol_no);
			break;
		}
		if (e->right.sym == &symbol_yes) {
			// A=y -> A
			e = expr_alloc_symbol(e->left.sym);
			break;
		}
		break;
	case E_UNEQUAL:
		if (e->left.sym->type != S_BOOLEAN)
			break;
		if (e->right.sym == &symbol_no) {
			// A!=n -> A
			e = expr_alloc_symbol(e->left.sym);
			break;
		}
		if (e->right.sym == &symbol_mod) {
			// A!=m -> y
			printf("boolean symbol %s tested for 'm'? test forced to 'y'\n", e->left.sym->name);
			e = expr_alloc_symbol(&symbol_yes);
			break;
		}
		if (e->right.sym == &symbol_yes) {
			// A!=y -> !A
			e = expr_alloc_one(E_NOT, e->left.expr);
			break;
		}
		break;
	case E_NOT:
		switch (e->left.expr->type) {
		case E_NOT:
			// !!A -> A
			e = e->left.expr->left.expr;
			break;
		case E_EQUAL:
		case E_UNEQUAL:
			// !(A=B) -> A!=B
			e = expr_alloc_comp(e->left.expr->type == E_EQUAL ? E_UNEQUAL : E_EQUAL,
					    e->left.expr->left.sym,
					    e->left.expr->right.sym);
			break;
		case E_LEQ:
		case E_GEQ:
			// !(A<=B) -> A>B
			e = expr_alloc_comp(e->left.expr->type == E_LEQ ? E_GTH : E_LTH,
					    e->left.expr->left.sym,
					    e->left.expr->right.sym);
			break;
		case E_LTH:
		case E_GTH:
			// !(A<B) -> A>=B
			e = expr_alloc_comp(e->left.expr->type == E_LTH ? E_GEQ : E_LEQ,
					    e->left.expr->left.sym,
					    e->left.expr->right.sym);
			break;
		case E_OR:
			// !(A || B) -> !A && !B
			e = expr_alloc_and(expr_alloc_one(E_NOT, e->left.expr->left.expr),
					   expr_alloc_one(E_NOT, e->left.expr->right.expr));
			e = expr_transform(e);
			break;
		case E_AND:
			// !(A && B) -> !A || !B
			e = expr_alloc_or(expr_alloc_one(E_NOT, e->left.expr->left.expr),
					  expr_alloc_one(E_NOT, e->left.expr->right.expr));
			e = expr_transform(e);
			break;
		case E_SYMBOL:
			if (e->left.expr->left.sym == &symbol_yes)
				// !'y' -> 'n'
				e = expr_alloc_symbol(&symbol_no);
			else if (e->left.expr->left.sym == &symbol_mod)
				// !'m' -> 'm'
				e = expr_alloc_symbol(&symbol_mod);
			else if (e->left.expr->left.sym == &symbol_no)
				// !'n' -> 'y'
				e = expr_alloc_symbol(&symbol_yes);
			break;
		default:
			;
		}
		break;
	default:
		;
	}
	return e;
}

bool expr_contains_symbol(struct expr *dep, struct symbol *sym)
{
	if (!dep)
		return false;

	switch (dep->type) {
	case E_AND:
	case E_OR:
		return expr_contains_symbol(dep->left.expr, sym) ||
		       expr_contains_symbol(dep->right.expr, sym);
	case E_SYMBOL:
		return dep->left.sym == sym;
	case E_EQUAL:
	case E_GEQ:
	case E_GTH:
	case E_LEQ:
	case E_LTH:
	case E_UNEQUAL:
		return dep->left.sym == sym ||
		       dep->right.sym == sym;
	case E_NOT:
		return expr_contains_symbol(dep->left.expr, sym);
	default:
		;
	}
	return false;
}

bool expr_depends_symbol(struct expr *dep, struct symbol *sym)
{
	if (!dep)
		return false;

	switch (dep->type) {
	case E_AND:
		return expr_depends_symbol(dep->left.expr, sym) ||
		       expr_depends_symbol(dep->right.expr, sym);
	case E_SYMBOL:
		return dep->left.sym == sym;
	case E_EQUAL:
		if (dep->left.sym == sym) {
			if (dep->right.sym == &symbol_yes || dep->right.sym == &symbol_mod)
				return true;
		}
		break;
	case E_UNEQUAL:
		if (dep->left.sym == sym) {
			if (dep->right.sym == &symbol_no)
				return true;
		}
		break;
	default:
		;
	}
 	return false;
}

/*
 * Inserts explicit comparisons of type 'type' to symbol 'sym' into the
 * expression 'e'.
 *
 * Examples transformations for type == E_UNEQUAL, sym == &symbol_no:
 *
 *	A              ->  A!=n
 *	!A             ->  A=n
 *	A && B         ->  !(A=n || B=n)
 *	A || B         ->  !(A=n && B=n)
 *	A && (B || C)  ->  !(A=n || (B=n && C=n))
 *
 * Allocates and returns a new expression.
 */
struct expr *expr_trans_compare(struct expr *e, enum expr_type type, struct symbol *sym)
{
	struct expr *e1, *e2;

	if (!e) {
		e = expr_alloc_symbol(sym);
		if (type == E_UNEQUAL)
			e = expr_alloc_one(E_NOT, e);
		return e;
	}
	switch (e->type) {
	case E_AND:
		e1 = expr_trans_compare(e->left.expr, E_EQUAL, sym);
		e2 = expr_trans_compare(e->right.expr, E_EQUAL, sym);
		if (sym == &symbol_yes)
			e = expr_alloc_two(E_AND, e1, e2);
		if (sym == &symbol_no)
			e = expr_alloc_two(E_OR, e1, e2);
		if (type == E_UNEQUAL)
			e = expr_alloc_one(E_NOT, e);
		return e;
	case E_OR:
		e1 = expr_trans_compare(e->left.expr, E_EQUAL, sym);
		e2 = expr_trans_compare(e->right.expr, E_EQUAL, sym);
		if (sym == &symbol_yes)
			e = expr_alloc_two(E_OR, e1, e2);
		if (sym == &symbol_no)
			e = expr_alloc_two(E_AND, e1, e2);
		if (type == E_UNEQUAL)
			e = expr_alloc_one(E_NOT, e);
		return e;
	case E_NOT:
		return expr_trans_compare(e->left.expr, type == E_EQUAL ? E_UNEQUAL : E_EQUAL, sym);
	case E_UNEQUAL:
	case E_LTH:
	case E_LEQ:
	case E_GTH:
	case E_GEQ:
	case E_EQUAL:
		if (type == E_EQUAL) {
			if (sym == &symbol_yes)
				return e;
			if (sym == &symbol_mod)
				return expr_alloc_symbol(&symbol_no);
			if (sym == &symbol_no)
				return expr_alloc_one(E_NOT, e);
		} else {
			if (sym == &symbol_yes)
				return expr_alloc_one(E_NOT, e);
			if (sym == &symbol_mod)
				return expr_alloc_symbol(&symbol_yes);
			if (sym == &symbol_no)
				return e;
		}
		break;
	case E_SYMBOL:
		return expr_alloc_comp(type, e->left.sym, sym);
	case E_RANGE:
	case E_NONE:
		/* panic */;
	}
	return NULL;
}

enum string_value_kind {
	k_string,
	k_signed,
	k_unsigned,
};

union string_value {
	unsigned long long u;
	signed long long s;
};

static enum string_value_kind expr_parse_string(const char *str,
						enum symbol_type type,
						union string_value *val)
{
	char *tail;
	enum string_value_kind kind;

	errno = 0;
	switch (type) {
	case S_BOOLEAN:
	case S_TRISTATE:
		val->s = !strcmp(str, "n") ? 0 :
			 !strcmp(str, "m") ? 1 :
			 !strcmp(str, "y") ? 2 : -1;
		return k_signed;
	case S_INT:
		val->s = strtoll(str, &tail, 10);
		kind = k_signed;
		break;
	case S_HEX:
		val->u = strtoull(str, &tail, 16);
		kind = k_unsigned;
		break;
	default:
		val->s = strtoll(str, &tail, 0);
		kind = k_signed;
		break;
	}
	return !errno && !*tail && tail > str && isxdigit(tail[-1])
	       ? kind : k_string;
}

static tristate __expr_calc_value(struct expr *e)
{
	tristate val1, val2;
	const char *str1, *str2;
	enum string_value_kind k1 = k_string, k2 = k_string;
	union string_value lval = {}, rval = {};
	int res;

	switch (e->type) {
	case E_SYMBOL:
		sym_calc_value(e->left.sym);
		return e->left.sym->curr.tri;
	case E_AND:
		val1 = expr_calc_value(e->left.expr);
		val2 = expr_calc_value(e->right.expr);
		return EXPR_AND(val1, val2);
	case E_OR:
		val1 = expr_calc_value(e->left.expr);
		val2 = expr_calc_value(e->right.expr);
		return EXPR_OR(val1, val2);
	case E_NOT:
		val1 = expr_calc_value(e->left.expr);
		return EXPR_NOT(val1);
	case E_EQUAL:
	case E_GEQ:
	case E_GTH:
	case E_LEQ:
	case E_LTH:
	case E_UNEQUAL:
		break;
	default:
		printf("expr_calc_value: %d?\n", e->type);
		return no;
	}

	sym_calc_value(e->left.sym);
	sym_calc_value(e->right.sym);
	str1 = sym_get_string_value(e->left.sym);
	str2 = sym_get_string_value(e->right.sym);

	if (e->left.sym->type != S_STRING || e->right.sym->type != S_STRING) {
		k1 = expr_parse_string(str1, e->left.sym->type, &lval);
		k2 = expr_parse_string(str2, e->right.sym->type, &rval);
	}

	if (k1 == k_string || k2 == k_string)
		res = strcmp(str1, str2);
	else if (k1 == k_unsigned || k2 == k_unsigned)
		res = (lval.u > rval.u) - (lval.u < rval.u);
	else /* if (k1 == k_signed && k2 == k_signed) */
		res = (lval.s > rval.s) - (lval.s < rval.s);

	switch(e->type) {
	case E_EQUAL:
		return res ? no : yes;
	case E_GEQ:
		return res >= 0 ? yes : no;
	case E_GTH:
		return res > 0 ? yes : no;
	case E_LEQ:
		return res <= 0 ? yes : no;
	case E_LTH:
		return res < 0 ? yes : no;
	case E_UNEQUAL:
		return res ? yes : no;
	default:
		printf("expr_calc_value: relation %d?\n", e->type);
		return no;
	}
}

/**
 * expr_calc_value - return the tristate value of the given expression
 * @e: expression
 * return: tristate value of the expression
 */
tristate expr_calc_value(struct expr *e)
{
	if (!e)
		return yes;

	if (!e->val_is_valid) {
		e->val = __expr_calc_value(e);
		e->val_is_valid = true;
	}

	return e->val;
}

/**
 * expr_invalidate_all - invalidate all cached expression values
 */
void expr_invalidate_all(void)
{
	struct expr *e;

	hash_for_each(expr_hashtable, e, node)
		e->val_is_valid = false;
}

static int expr_compare_type(enum expr_type t1, enum expr_type t2)
{
	if (t1 == t2)
		return 0;
	switch (t1) {
	case E_LEQ:
	case E_LTH:
	case E_GEQ:
	case E_GTH:
		if (t2 == E_EQUAL || t2 == E_UNEQUAL)
			return 1;
		/* fallthrough */
	case E_EQUAL:
	case E_UNEQUAL:
		if (t2 == E_NOT)
			return 1;
		/* fallthrough */
	case E_NOT:
		if (t2 == E_AND)
			return 1;
		/* fallthrough */
	case E_AND:
		if (t2 == E_OR)
			return 1;
		/* fallthrough */
	default:
		break;
	}
	return 0;
}

void expr_print(const struct expr *e,
		void (*fn)(void *, struct symbol *, const char *),
		void *data, int prevtoken)
{
	if (!e) {
		fn(data, NULL, "y");
		return;
	}

	if (expr_compare_type(prevtoken, e->type) > 0)
		fn(data, NULL, "(");
	switch (e->type) {
	case E_SYMBOL:
		if (e->left.sym->name)
			fn(data, e->left.sym, e->left.sym->name);
		else
			fn(data, NULL, "<choice>");
		break;
	case E_NOT:
		fn(data, NULL, "!");
		expr_print(e->left.expr, fn, data, E_NOT);
		break;
	case E_EQUAL:
		if (e->left.sym->name)
			fn(data, e->left.sym, e->left.sym->name);
		else
			fn(data, NULL, "<choice>");
		fn(data, NULL, "=");
		fn(data, e->right.sym, e->right.sym->name);
		break;
	case E_LEQ:
	case E_LTH:
		if (e->left.sym->name)
			fn(data, e->left.sym, e->left.sym->name);
		else
			fn(data, NULL, "<choice>");
		fn(data, NULL, e->type == E_LEQ ? "<=" : "<");
		fn(data, e->right.sym, e->right.sym->name);
		break;
	case E_GEQ:
	case E_GTH:
		if (e->left.sym->name)
			fn(data, e->left.sym, e->left.sym->name);
		else
			fn(data, NULL, "<choice>");
		fn(data, NULL, e->type == E_GEQ ? ">=" : ">");
		fn(data, e->right.sym, e->right.sym->name);
		break;
	case E_UNEQUAL:
		if (e->left.sym->name)
			fn(data, e->left.sym, e->left.sym->name);
		else
			fn(data, NULL, "<choice>");
		fn(data, NULL, "!=");
		fn(data, e->right.sym, e->right.sym->name);
		break;
	case E_OR:
		expr_print(e->left.expr, fn, data, E_OR);
		fn(data, NULL, " || ");
		expr_print(e->right.expr, fn, data, E_OR);
		break;
	case E_AND:
		expr_print(e->left.expr, fn, data, E_AND);
		fn(data, NULL, " && ");
		expr_print(e->right.expr, fn, data, E_AND);
		break;
	case E_RANGE:
		fn(data, NULL, "[");
		fn(data, e->left.sym, e->left.sym->name);
		fn(data, NULL, " ");
		fn(data, e->right.sym, e->right.sym->name);
		fn(data, NULL, "]");
		break;
	default:
	  {
		char buf[32];
		sprintf(buf, "<unknown type %d>", e->type);
		fn(data, NULL, buf);
		break;
	  }
	}
	if (expr_compare_type(prevtoken, e->type) > 0)
		fn(data, NULL, ")");
}

static void expr_print_file_helper(void *data, struct symbol *sym, const char *str)
{
	xfwrite(str, strlen(str), 1, data);
}

void expr_fprint(struct expr *e, FILE *out)
{
	expr_print(e, expr_print_file_helper, out, E_NONE);
}

static void expr_print_gstr_helper(void *data, struct symbol *sym, const char *str)
{
	struct gstr *gs = (struct gstr*)data;
	const char *sym_str = NULL;

	if (sym)
		sym_str = sym_get_string_value(sym);

	if (gs->max_width) {
		unsigned extra_length = strlen(str);
		const char *last_cr = strrchr(gs->s, '\n');
		unsigned last_line_length;

		if (sym_str)
			extra_length += 4 + strlen(sym_str);

		if (!last_cr)
			last_cr = gs->s;

		last_line_length = strlen(gs->s) - (last_cr - gs->s);

		if ((last_line_length + extra_length) > gs->max_width)
			str_append(gs, "\\\n");
	}

	str_append(gs, str);
	if (sym && sym->type != S_UNKNOWN)
		str_printf(gs, " [=%s]", sym_str);
}

void expr_gstr_print(const struct expr *e, struct gstr *gs)
{
	expr_print(e, expr_print_gstr_helper, gs, E_NONE);
}

/*
 * Transform the top level "||" tokens into newlines and prepend each
 * line with a minus. This makes expressions much easier to read.
 * Suitable for reverse dependency expressions.
 */
static void expr_print_revdep(struct expr *e,
			      void (*fn)(void *, struct symbol *, const char *),
			      void *data, tristate pr_type, const char **title)
{
	if (e->type == E_OR) {
		expr_print_revdep(e->left.expr, fn, data, pr_type, title);
		expr_print_revdep(e->right.expr, fn, data, pr_type, title);
	} else if (expr_calc_value(e) == pr_type) {
		if (*title) {
			fn(data, NULL, *title);
			*title = NULL;
		}

		fn(data, NULL, "  - ");
		expr_print(e, fn, data, E_NONE);
		fn(data, NULL, "\n");
	}
}

void expr_gstr_print_revdep(struct expr *e, struct gstr *gs,
			    tristate pr_type, const char *title)
{
	expr_print_revdep(e, expr_print_gstr_helper, gs, pr_type, &title);
}
