// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * dwarf-aux.c : libdw auxiliary interfaces
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "debug.h"
#include "dwarf-aux.h"
#include "dwarf-regs.h"
#include "strbuf.h"
#include "string2.h"

/**
 * cu_find_realpath - Find the realpath of the target file
 * @cu_die: A DIE(dwarf information entry) of CU(compilation Unit)
 * @fname:  The tail filename of the target file
 *
 * Find the real(long) path of @fname in @cu_die.
 */
const char *cu_find_realpath(Dwarf_Die *cu_die, const char *fname)
{
	Dwarf_Files *files;
	size_t nfiles, i;
	const char *src = NULL;
	int ret;

	if (!fname)
		return NULL;

	ret = dwarf_getsrcfiles(cu_die, &files, &nfiles);
	if (ret != 0)
		return NULL;

	for (i = 0; i < nfiles; i++) {
		src = dwarf_filesrc(files, i, NULL, NULL);
		if (strtailcmp(src, fname) == 0)
			break;
	}
	if (i == nfiles)
		return NULL;
	return src;
}

/**
 * cu_get_comp_dir - Get the path of compilation directory
 * @cu_die: a CU DIE
 *
 * Get the path of compilation directory of given @cu_die.
 * Since this depends on DW_AT_comp_dir, older gcc will not
 * embedded it. In that case, this returns NULL.
 */
const char *cu_get_comp_dir(Dwarf_Die *cu_die)
{
	Dwarf_Attribute attr;
	if (dwarf_attr(cu_die, DW_AT_comp_dir, &attr) == NULL)
		return NULL;
	return dwarf_formstring(&attr);
}

/* Unlike dwarf_getsrc_die(), cu_getsrc_die() only returns statement line */
static Dwarf_Line *cu_getsrc_die(Dwarf_Die *cu_die, Dwarf_Addr addr)
{
	Dwarf_Addr laddr;
	Dwarf_Lines *lines;
	Dwarf_Line *line;
	size_t nlines, l, u, n;
	bool flag;

	if (dwarf_getsrclines(cu_die, &lines, &nlines) != 0 ||
	    nlines == 0)
		return NULL;

	/* Lines are sorted by address, use binary search */
	l = 0; u = nlines - 1;
	while (l < u) {
		n = u - (u - l) / 2;
		line = dwarf_onesrcline(lines, n);
		if (!line || dwarf_lineaddr(line, &laddr) != 0)
			return NULL;
		if (addr < laddr)
			u = n - 1;
		else
			l = n;
	}
	/* Going backward to find the lowest line */
	do {
		line = dwarf_onesrcline(lines, --l);
		if (!line || dwarf_lineaddr(line, &laddr) != 0)
			return NULL;
	} while (laddr == addr);
	l++;
	/* Going forward to find the statement line */
	do {
		line = dwarf_onesrcline(lines, l++);
		if (!line || dwarf_lineaddr(line, &laddr) != 0 ||
		    dwarf_linebeginstatement(line, &flag) != 0)
			return NULL;
		if (laddr > addr)
			return NULL;
	} while (!flag);

	return line;
}

/**
 * cu_find_lineinfo - Get a line number and file name for given address
 * @cu_die: a CU DIE
 * @addr: An address
 * @fname: a pointer which returns the file name string
 * @lineno: a pointer which returns the line number
 *
 * Find a line number and file name for @addr in @cu_die.
 */
int cu_find_lineinfo(Dwarf_Die *cu_die, Dwarf_Addr addr,
		     const char **fname, int *lineno)
{
	Dwarf_Line *line;
	Dwarf_Die die_mem;
	Dwarf_Addr faddr;

	if (die_find_realfunc(cu_die, addr, &die_mem)
	    && die_entrypc(&die_mem, &faddr) == 0 &&
	    faddr == addr) {
		*fname = die_get_decl_file(&die_mem);
		dwarf_decl_line(&die_mem, lineno);
		goto out;
	}

	line = cu_getsrc_die(cu_die, addr);
	if (line && dwarf_lineno(line, lineno) == 0) {
		*fname = dwarf_linesrc(line, NULL, NULL);
		if (!*fname)
			/* line number is useless without filename */
			*lineno = 0;
	}

out:
	return (*lineno && *fname) ? *lineno : -ENOENT;
}

static int __die_find_inline_cb(Dwarf_Die *die_mem, void *data);

/**
 * cu_walk_functions_at - Walk on function DIEs at given address
 * @cu_die: A CU DIE
 * @addr: An address
 * @callback: A callback which called with found DIEs
 * @data: A user data
 *
 * Walk on function DIEs at given @addr in @cu_die. Passed DIEs
 * should be subprogram or inlined-subroutines.
 */
int cu_walk_functions_at(Dwarf_Die *cu_die, Dwarf_Addr addr,
		    int (*callback)(Dwarf_Die *, void *), void *data)
{
	Dwarf_Die die_mem;
	Dwarf_Die *sc_die;
	int ret = -ENOENT;

	/* Inlined function could be recursive. Trace it until fail */
	for (sc_die = die_find_realfunc(cu_die, addr, &die_mem);
	     sc_die != NULL;
	     sc_die = die_find_child(sc_die, __die_find_inline_cb, &addr,
				     &die_mem)) {
		ret = callback(sc_die, data);
		if (ret)
			break;
	}

	return ret;

}

/**
 * die_get_linkage_name - Get the linkage name of the object
 * @dw_die: A DIE of the object
 *
 * Get the linkage name attribute of given @dw_die.
 * For C++ binary, the linkage name will be the mangled symbol.
 */
const char *die_get_linkage_name(Dwarf_Die *dw_die)
{
	Dwarf_Attribute attr;

	if (dwarf_attr_integrate(dw_die, DW_AT_linkage_name, &attr) == NULL)
		return NULL;
	return dwarf_formstring(&attr);
}

/**
 * die_compare_name - Compare diename and tname
 * @dw_die: a DIE
 * @tname: a string of target name
 *
 * Compare the name of @dw_die and @tname. Return false if @dw_die has no name.
 */
bool die_compare_name(Dwarf_Die *dw_die, const char *tname)
{
	const char *name;

	name = dwarf_diename(dw_die);
	return name ? (strcmp(tname, name) == 0) : false;
}

/**
 * die_match_name - Match diename/linkage name and glob
 * @dw_die: a DIE
 * @glob: a string of target glob pattern
 *
 * Glob matching the name of @dw_die and @glob. Return false if matching fail.
 * This also match linkage name.
 */
bool die_match_name(Dwarf_Die *dw_die, const char *glob)
{
	const char *name;

	name = dwarf_diename(dw_die);
	if (name && strglobmatch(name, glob))
		return true;
	/* fall back to check linkage name */
	name = die_get_linkage_name(dw_die);
	if (name && strglobmatch(name, glob))
		return true;

	return false;
}

/**
 * die_get_call_lineno - Get callsite line number of inline-function instance
 * @in_die: a DIE of an inlined function instance
 *
 * Get call-site line number of @in_die. This means from where the inline
 * function is called.
 */
int die_get_call_lineno(Dwarf_Die *in_die)
{
	Dwarf_Attribute attr;
	Dwarf_Word ret;

	if (!dwarf_attr(in_die, DW_AT_call_line, &attr))
		return -ENOENT;

	dwarf_formudata(&attr, &ret);
	return (int)ret;
}

/**
 * die_get_type - Get type DIE
 * @vr_die: a DIE of a variable
 * @die_mem: where to store a type DIE
 *
 * Get a DIE of the type of given variable (@vr_die), and store
 * it to die_mem. Return NULL if fails to get a type DIE.
 */
Dwarf_Die *die_get_type(Dwarf_Die *vr_die, Dwarf_Die *die_mem)
{
	Dwarf_Attribute attr;

	if (dwarf_attr_integrate(vr_die, DW_AT_type, &attr) &&
	    dwarf_formref_die(&attr, die_mem))
		return die_mem;
	else
		return NULL;
}

/* Get a type die, but skip qualifiers */
static Dwarf_Die *__die_get_real_type(Dwarf_Die *vr_die, Dwarf_Die *die_mem)
{
	int tag;

	do {
		vr_die = die_get_type(vr_die, die_mem);
		if (!vr_die)
			break;
		tag = dwarf_tag(vr_die);
	} while (tag == DW_TAG_const_type ||
		 tag == DW_TAG_restrict_type ||
		 tag == DW_TAG_volatile_type ||
		 tag == DW_TAG_shared_type);

	return vr_die;
}

/**
 * die_get_real_type - Get a type die, but skip qualifiers and typedef
 * @vr_die: a DIE of a variable
 * @die_mem: where to store a type DIE
 *
 * Get a DIE of the type of given variable (@vr_die), and store
 * it to die_mem. Return NULL if fails to get a type DIE.
 * If the type is qualifiers (e.g. const) or typedef, this skips it
 * and tries to find real type (structure or basic types, e.g. int).
 */
Dwarf_Die *die_get_real_type(Dwarf_Die *vr_die, Dwarf_Die *die_mem)
{
	do {
		vr_die = __die_get_real_type(vr_die, die_mem);
	} while (vr_die && dwarf_tag(vr_die) == DW_TAG_typedef);

	return vr_die;
}

/* Get attribute and translate it as a udata */
static int die_get_attr_udata(Dwarf_Die *tp_die, unsigned int attr_name,
			      Dwarf_Word *result)
{
	Dwarf_Attribute attr;

	if (dwarf_attr_integrate(tp_die, attr_name, &attr) == NULL ||
	    dwarf_formudata(&attr, result) != 0)
		return -ENOENT;

	return 0;
}

/**
 * die_is_signed_type - Check whether a type DIE is signed or not
 * @tp_die: a DIE of a type
 *
 * Get the encoding of @tp_die and return true if the encoding
 * is signed.
 */
bool die_is_signed_type(Dwarf_Die *tp_die)
{
	Dwarf_Word ret;

	if (die_get_attr_udata(tp_die, DW_AT_encoding, &ret))
		return false;

	return (ret == DW_ATE_signed_char || ret == DW_ATE_signed ||
		ret == DW_ATE_signed_fixed);
}

/**
 * die_is_func_def - Ensure that this DIE is a subprogram and definition
 * @dw_die: a DIE
 *
 * Ensure that this DIE is a subprogram and NOT a declaration. This
 * returns true if @dw_die is a function definition.
 **/
bool die_is_func_def(Dwarf_Die *dw_die)
{
	Dwarf_Attribute attr;
	Dwarf_Addr addr = 0;

	if (dwarf_tag(dw_die) != DW_TAG_subprogram)
		return false;

	if (dwarf_attr(dw_die, DW_AT_declaration, &attr))
		return false;

	/*
	 * DW_AT_declaration can be lost from function declaration
	 * by gcc's bug #97060.
	 * So we need to check this subprogram DIE has DW_AT_inline
	 * or an entry address.
	 */
	if (!dwarf_attr(dw_die, DW_AT_inline, &attr) &&
	    die_entrypc(dw_die, &addr) < 0)
		return false;

	return true;
}

/**
 * die_entrypc - Returns entry PC (the lowest address) of a DIE
 * @dw_die: a DIE
 * @addr: where to store entry PC
 *
 * Since dwarf_entrypc() does not return entry PC if the DIE has only address
 * range, we have to use this to retrieve the lowest address from the address
 * range attribute.
 */
int die_entrypc(Dwarf_Die *dw_die, Dwarf_Addr *addr)
{
	Dwarf_Addr base, end;
	Dwarf_Attribute attr;

	if (!addr)
		return -EINVAL;

	if (dwarf_entrypc(dw_die, addr) == 0)
		return 0;

	/*
	 *  Since the dwarf_ranges() will return 0 if there is no
	 * DW_AT_ranges attribute, we should check it first.
	 */
	if (!dwarf_attr(dw_die, DW_AT_ranges, &attr))
		return -ENOENT;

	return dwarf_ranges(dw_die, 0, &base, addr, &end) < 0 ? -ENOENT : 0;
}

/**
 * die_is_func_instance - Ensure that this DIE is an instance of a subprogram
 * @dw_die: a DIE
 *
 * Ensure that this DIE is an instance (which has an entry address).
 * This returns true if @dw_die is a function instance. If not, the @dw_die
 * must be a prototype. You can use die_walk_instances() to find actual
 * instances.
 **/
bool die_is_func_instance(Dwarf_Die *dw_die)
{
	Dwarf_Addr tmp;
	Dwarf_Attribute attr_mem;
	int tag = dwarf_tag(dw_die);

	if (tag != DW_TAG_subprogram &&
	    tag != DW_TAG_inlined_subroutine)
		return false;

	return dwarf_entrypc(dw_die, &tmp) == 0 ||
		dwarf_attr(dw_die, DW_AT_ranges, &attr_mem) != NULL;
}

/**
 * die_get_data_member_location - Get the data-member offset
 * @mb_die: a DIE of a member of a data structure
 * @offs: The offset of the member in the data structure
 *
 * Get the offset of @mb_die in the data structure including @mb_die, and
 * stores result offset to @offs. If any error occurs this returns errno.
 */
int die_get_data_member_location(Dwarf_Die *mb_die, Dwarf_Word *offs)
{
	Dwarf_Attribute attr;
	Dwarf_Op *expr;
	size_t nexpr;
	int ret;

	if (dwarf_attr(mb_die, DW_AT_data_member_location, &attr) == NULL)
		return -ENOENT;

	if (dwarf_formudata(&attr, offs) != 0) {
		/* DW_AT_data_member_location should be DW_OP_plus_uconst */
		ret = dwarf_getlocation(&attr, &expr, &nexpr);
		if (ret < 0 || nexpr == 0)
			return -ENOENT;

		if (expr[0].atom != DW_OP_plus_uconst || nexpr != 1) {
			pr_debug("Unable to get offset:Unexpected OP %x (%zd)\n",
				 expr[0].atom, nexpr);
			return -ENOTSUP;
		}
		*offs = (Dwarf_Word)expr[0].number;
	}
	return 0;
}

/* Get the call file index number in CU DIE */
static int die_get_call_fileno(Dwarf_Die *in_die)
{
	Dwarf_Word idx;

	if (die_get_attr_udata(in_die, DW_AT_call_file, &idx) == 0)
		return (int)idx;
	else
		return -ENOENT;
}

/* Get the declared file index number in CU DIE */
static int die_get_decl_fileno(Dwarf_Die *pdie)
{
	Dwarf_Word idx;

	if (die_get_attr_udata(pdie, DW_AT_decl_file, &idx) == 0)
		return (int)idx;
	else
		return -ENOENT;
}

/* Return the file name by index */
static const char *die_get_file_name(Dwarf_Die *dw_die, int idx)
{
	Dwarf_Die cu_die;
	Dwarf_Files *files;
	Dwarf_Attribute attr_mem;

	if (idx < 0 || !dwarf_attr_integrate(dw_die, DW_AT_decl_file, &attr_mem) ||
	    !dwarf_cu_die(attr_mem.cu, &cu_die, NULL, NULL, NULL, NULL, NULL, NULL) ||
	    dwarf_getsrcfiles(&cu_die, &files, NULL) != 0)
		return NULL;

	return dwarf_filesrc(files, idx, NULL, NULL);
}

/**
 * die_get_call_file - Get callsite file name of inlined function instance
 * @in_die: a DIE of an inlined function instance
 *
 * Get call-site file name of @in_die. This means from which file the inline
 * function is called.
 */
const char *die_get_call_file(Dwarf_Die *in_die)
{
	return die_get_file_name(in_die, die_get_call_fileno(in_die));
}

/**
 * die_get_decl_file - Find the declared file name of this DIE
 * @dw_die: a DIE for something declared.
 *
 * Get declared file name of @dw_die.
 * NOTE: Since some version of clang DWARF5 implementation incorrectly uses
 * file index 0 for DW_AT_decl_file, die_get_decl_file() will return NULL for
 * such cases. Use this function instead.
 */
const char *die_get_decl_file(Dwarf_Die *dw_die)
{
	return die_get_file_name(dw_die, die_get_decl_fileno(dw_die));
}

/**
 * die_find_child - Generic DIE search function in DIE tree
 * @rt_die: a root DIE
 * @callback: a callback function
 * @data: a user data passed to the callback function
 * @die_mem: a buffer for result DIE
 *
 * Trace DIE tree from @rt_die and call @callback for each child DIE.
 * If @callback returns DIE_FIND_CB_END, this stores the DIE into
 * @die_mem and returns it. If @callback returns DIE_FIND_CB_CONTINUE,
 * this continues to trace the tree. Optionally, @callback can return
 * DIE_FIND_CB_CHILD and DIE_FIND_CB_SIBLING, those means trace only
 * the children and trace only the siblings respectively.
 * Returns NULL if @callback can't find any appropriate DIE.
 */
Dwarf_Die *die_find_child(Dwarf_Die *rt_die,
			  int (*callback)(Dwarf_Die *, void *),
			  void *data, Dwarf_Die *die_mem)
{
	Dwarf_Die child_die;
	int ret;

	ret = dwarf_child(rt_die, die_mem);
	if (ret != 0)
		return NULL;

	do {
		ret = callback(die_mem, data);
		if (ret == DIE_FIND_CB_END)
			return die_mem;

		if ((ret & DIE_FIND_CB_CHILD) &&
		    die_find_child(die_mem, callback, data, &child_die)) {
			memcpy(die_mem, &child_die, sizeof(Dwarf_Die));
			return die_mem;
		}
	} while ((ret & DIE_FIND_CB_SIBLING) &&
		 dwarf_siblingof(die_mem, die_mem) == 0);

	return NULL;
}

struct __addr_die_search_param {
	Dwarf_Addr	addr;
	Dwarf_Die	*die_mem;
};

static int __die_search_func_tail_cb(Dwarf_Die *fn_die, void *data)
{
	struct __addr_die_search_param *ad = data;
	Dwarf_Addr addr = 0;

	if (dwarf_tag(fn_die) == DW_TAG_subprogram &&
	    !dwarf_highpc(fn_die, &addr) &&
	    addr == ad->addr) {
		memcpy(ad->die_mem, fn_die, sizeof(Dwarf_Die));
		return DWARF_CB_ABORT;
	}
	return DWARF_CB_OK;
}

/**
 * die_find_tailfunc - Search for a non-inlined function with tail call at
 * given address
 * @cu_die: a CU DIE which including @addr
 * @addr: target address
 * @die_mem: a buffer for result DIE
 *
 * Search for a non-inlined function DIE with tail call at @addr. Stores the
 * DIE to @die_mem and returns it if found. Returns NULL if failed.
 */
Dwarf_Die *die_find_tailfunc(Dwarf_Die *cu_die, Dwarf_Addr addr,
				    Dwarf_Die *die_mem)
{
	struct __addr_die_search_param ad;
	ad.addr = addr;
	ad.die_mem = die_mem;
	/* dwarf_getscopes can't find subprogram. */
	if (!dwarf_getfuncs(cu_die, __die_search_func_tail_cb, &ad, 0))
		return NULL;
	else
		return die_mem;
}

/* die_find callback for non-inlined function search */
static int __die_search_func_cb(Dwarf_Die *fn_die, void *data)
{
	struct __addr_die_search_param *ad = data;

	/*
	 * Since a declaration entry doesn't has given pc, this always returns
	 * function definition entry.
	 */
	if (dwarf_tag(fn_die) == DW_TAG_subprogram &&
	    dwarf_haspc(fn_die, ad->addr)) {
		memcpy(ad->die_mem, fn_die, sizeof(Dwarf_Die));
		return DWARF_CB_ABORT;
	}
	return DWARF_CB_OK;
}

/**
 * die_find_realfunc - Search a non-inlined function at given address
 * @cu_die: a CU DIE which including @addr
 * @addr: target address
 * @die_mem: a buffer for result DIE
 *
 * Search a non-inlined function DIE which includes @addr. Stores the
 * DIE to @die_mem and returns it if found. Returns NULL if failed.
 */
Dwarf_Die *die_find_realfunc(Dwarf_Die *cu_die, Dwarf_Addr addr,
				    Dwarf_Die *die_mem)
{
	struct __addr_die_search_param ad;
	ad.addr = addr;
	ad.die_mem = die_mem;
	/* dwarf_getscopes can't find subprogram. */
	if (!dwarf_getfuncs(cu_die, __die_search_func_cb, &ad, 0))
		return NULL;
	else
		return die_mem;
}

/* die_find callback for inline function search */
static int __die_find_inline_cb(Dwarf_Die *die_mem, void *data)
{
	Dwarf_Addr *addr = data;

	if (dwarf_tag(die_mem) == DW_TAG_inlined_subroutine &&
	    dwarf_haspc(die_mem, *addr))
		return DIE_FIND_CB_END;

	return DIE_FIND_CB_CONTINUE;
}

/**
 * die_find_top_inlinefunc - Search the top inlined function at given address
 * @sp_die: a subprogram DIE which including @addr
 * @addr: target address
 * @die_mem: a buffer for result DIE
 *
 * Search an inlined function DIE which includes @addr. Stores the
 * DIE to @die_mem and returns it if found. Returns NULL if failed.
 * Even if several inlined functions are expanded recursively, this
 * doesn't trace it down, and returns the topmost one.
 */
Dwarf_Die *die_find_top_inlinefunc(Dwarf_Die *sp_die, Dwarf_Addr addr,
				   Dwarf_Die *die_mem)
{
	return die_find_child(sp_die, __die_find_inline_cb, &addr, die_mem);
}

/**
 * die_find_inlinefunc - Search an inlined function at given address
 * @sp_die: a subprogram DIE which including @addr
 * @addr: target address
 * @die_mem: a buffer for result DIE
 *
 * Search an inlined function DIE which includes @addr. Stores the
 * DIE to @die_mem and returns it if found. Returns NULL if failed.
 * If several inlined functions are expanded recursively, this trace
 * it down and returns deepest one.
 */
Dwarf_Die *die_find_inlinefunc(Dwarf_Die *sp_die, Dwarf_Addr addr,
			       Dwarf_Die *die_mem)
{
	Dwarf_Die tmp_die;

	sp_die = die_find_child(sp_die, __die_find_inline_cb, &addr, &tmp_die);
	if (!sp_die)
		return NULL;

	/* Inlined function could be recursive. Trace it until fail */
	while (sp_die) {
		memcpy(die_mem, sp_die, sizeof(Dwarf_Die));
		sp_die = die_find_child(sp_die, __die_find_inline_cb, &addr,
					&tmp_die);
	}

	return die_mem;
}

static int __die_find_func_rettype_cb(Dwarf_Die *die_mem, void *data)
{
	const char *func_name;

	if (dwarf_tag(die_mem) != DW_TAG_subprogram)
		return DIE_FIND_CB_SIBLING;

	func_name = dwarf_diename(die_mem);
	if (func_name && !strcmp(func_name, data))
		return DIE_FIND_CB_END;

	return DIE_FIND_CB_SIBLING;
}

/**
 * die_find_func_rettype - Search a return type of function
 * @cu_die: a CU DIE
 * @name: target function name
 * @die_mem: a buffer for result DIE
 *
 * Search a non-inlined function which matches to @name and stores the
 * return type of the function to @die_mem and returns it if found.
 * Returns NULL if failed.  Note that it doesn't needs to find a
 * definition of the function, so it doesn't match with address.
 * Most likely, it can find a declaration at the top level.  Thus the
 * callback function continues to sibling entries only.
 */
Dwarf_Die *die_find_func_rettype(Dwarf_Die *cu_die, const char *name,
				 Dwarf_Die *die_mem)
{
	Dwarf_Die tmp_die;

	cu_die = die_find_child(cu_die, __die_find_func_rettype_cb,
				(void *)name, &tmp_die);
	if (!cu_die)
		return NULL;

	if (die_get_real_type(&tmp_die, die_mem) == NULL)
		return NULL;

	return die_mem;
}

struct __instance_walk_param {
	void    *addr;
	int	(*callback)(Dwarf_Die *, void *);
	void    *data;
	int	retval;
};

static int __die_walk_instances_cb(Dwarf_Die *inst, void *data)
{
	struct __instance_walk_param *iwp = data;
	Dwarf_Attribute attr_mem;
	Dwarf_Die origin_mem;
	Dwarf_Attribute *attr;
	Dwarf_Die *origin;
	int tmp;

	if (!die_is_func_instance(inst))
		return DIE_FIND_CB_CONTINUE;

	attr = dwarf_attr(inst, DW_AT_abstract_origin, &attr_mem);
	if (attr == NULL)
		return DIE_FIND_CB_CONTINUE;

	origin = dwarf_formref_die(attr, &origin_mem);
	if (origin == NULL || origin->addr != iwp->addr)
		return DIE_FIND_CB_CONTINUE;

	/* Ignore redundant instances */
	if (dwarf_tag(inst) == DW_TAG_inlined_subroutine) {
		dwarf_decl_line(origin, &tmp);
		if (die_get_call_lineno(inst) == tmp) {
			tmp = die_get_decl_fileno(origin);
			if (die_get_call_fileno(inst) == tmp)
				return DIE_FIND_CB_CONTINUE;
		}
	}

	iwp->retval = iwp->callback(inst, iwp->data);

	return (iwp->retval) ? DIE_FIND_CB_END : DIE_FIND_CB_CONTINUE;
}

/**
 * die_walk_instances - Walk on instances of given DIE
 * @or_die: an abstract original DIE
 * @callback: a callback function which is called with instance DIE
 * @data: user data
 *
 * Walk on the instances of give @in_die. @in_die must be an inlined function
 * declaration. This returns the return value of @callback if it returns
 * non-zero value, or -ENOENT if there is no instance.
 */
int die_walk_instances(Dwarf_Die *or_die, int (*callback)(Dwarf_Die *, void *),
		       void *data)
{
	Dwarf_Die cu_die;
	Dwarf_Die die_mem;
	struct __instance_walk_param iwp = {
		.addr = or_die->addr,
		.callback = callback,
		.data = data,
		.retval = -ENOENT,
	};

	if (dwarf_diecu(or_die, &cu_die, NULL, NULL) == NULL)
		return -ENOENT;

	die_find_child(&cu_die, __die_walk_instances_cb, &iwp, &die_mem);

	return iwp.retval;
}

/* Line walker internal parameters */
struct __line_walk_param {
	bool recursive;
	line_walk_callback_t callback;
	void *data;
	int retval;
};

static int __die_walk_funclines_cb(Dwarf_Die *in_die, void *data)
{
	struct __line_walk_param *lw = data;
	Dwarf_Addr addr = 0;
	const char *fname;
	int lineno;

	if (dwarf_tag(in_die) == DW_TAG_inlined_subroutine) {
		fname = die_get_call_file(in_die);
		lineno = die_get_call_lineno(in_die);
		if (fname && lineno > 0 && die_entrypc(in_die, &addr) == 0) {
			lw->retval = lw->callback(fname, lineno, addr, lw->data);
			if (lw->retval != 0)
				return DIE_FIND_CB_END;
		}
		if (!lw->recursive)
			return DIE_FIND_CB_SIBLING;
	}

	if (addr) {
		fname = die_get_decl_file(in_die);
		if (fname && dwarf_decl_line(in_die, &lineno) == 0) {
			lw->retval = lw->callback(fname, lineno, addr, lw->data);
			if (lw->retval != 0)
				return DIE_FIND_CB_END;
		}
	}

	/* Continue to search nested inlined function call-sites */
	return DIE_FIND_CB_CONTINUE;
}

/* Walk on lines of blocks included in given DIE */
static int __die_walk_funclines(Dwarf_Die *sp_die, bool recursive,
				line_walk_callback_t callback, void *data)
{
	struct __line_walk_param lw = {
		.recursive = recursive,
		.callback = callback,
		.data = data,
		.retval = 0,
	};
	Dwarf_Die die_mem;
	Dwarf_Addr addr;
	const char *fname;
	int lineno;

	/* Handle function declaration line */
	fname = die_get_decl_file(sp_die);
	if (fname && dwarf_decl_line(sp_die, &lineno) == 0 &&
	    die_entrypc(sp_die, &addr) == 0) {
		lw.retval = callback(fname, lineno, addr, data);
		if (lw.retval != 0)
			goto done;
	}
	die_find_child(sp_die, __die_walk_funclines_cb, &lw, &die_mem);
done:
	return lw.retval;
}

static int __die_walk_culines_cb(Dwarf_Die *sp_die, void *data)
{
	struct __line_walk_param *lw = data;

	/*
	 * Since inlined function can include another inlined function in
	 * the same file, we need to walk in it recursively.
	 */
	lw->retval = __die_walk_funclines(sp_die, true, lw->callback, lw->data);
	if (lw->retval != 0)
		return DWARF_CB_ABORT;

	return DWARF_CB_OK;
}

/**
 * die_walk_lines - Walk on lines inside given DIE
 * @rt_die: a root DIE (CU, subprogram or inlined_subroutine)
 * @callback: callback routine
 * @data: user data
 *
 * Walk on all lines inside given @rt_die and call @callback on each line.
 * If the @rt_die is a function, walk only on the lines inside the function,
 * otherwise @rt_die must be a CU DIE.
 * Note that this walks not only dwarf line list, but also function entries
 * and inline call-site.
 */
int die_walk_lines(Dwarf_Die *rt_die, line_walk_callback_t callback, void *data)
{
	Dwarf_Lines *lines;
	Dwarf_Line *line;
	Dwarf_Addr addr;
	const char *fname, *decf = NULL, *inf = NULL;
	int lineno, ret = 0;
	int decl = 0, inl;
	Dwarf_Die die_mem, *cu_die;
	size_t nlines, i;
	bool flag;

	/* Get the CU die */
	if (dwarf_tag(rt_die) != DW_TAG_compile_unit) {
		cu_die = dwarf_diecu(rt_die, &die_mem, NULL, NULL);
		dwarf_decl_line(rt_die, &decl);
		decf = die_get_decl_file(rt_die);
		if (!decf) {
			pr_debug2("Failed to get the declared file name of %s\n",
				  dwarf_diename(rt_die));
			return -EINVAL;
		}
	} else
		cu_die = rt_die;
	if (!cu_die) {
		pr_debug2("Failed to get CU from given DIE.\n");
		return -EINVAL;
	}

	/* Get lines list in the CU */
	if (dwarf_getsrclines(cu_die, &lines, &nlines) != 0) {
		pr_debug2("Failed to get source lines on this CU.\n");
		return -ENOENT;
	}
	pr_debug2("Get %zd lines from this CU\n", nlines);

	/* Walk on the lines on lines list */
	for (i = 0; i < nlines; i++) {
		line = dwarf_onesrcline(lines, i);
		if (line == NULL ||
		    dwarf_lineno(line, &lineno) != 0 ||
		    dwarf_lineaddr(line, &addr) != 0) {
			pr_debug2("Failed to get line info. "
				  "Possible error in debuginfo.\n");
			continue;
		}
		/* Skip end-of-sequence */
		if (dwarf_lineendsequence(line, &flag) != 0 || flag)
			continue;
		/* Skip Non statement line-info */
		if (dwarf_linebeginstatement(line, &flag) != 0 || !flag)
			continue;
		/* Filter lines based on address */
		if (rt_die != cu_die) {
			/*
			 * Address filtering
			 * The line is included in given function, and
			 * no inline block includes it.
			 */
			if (!dwarf_haspc(rt_die, addr))
				continue;

			if (die_find_inlinefunc(rt_die, addr, &die_mem)) {
				/* Call-site check */
				inf = die_get_call_file(&die_mem);
				if ((inf && !strcmp(inf, decf)) &&
				    die_get_call_lineno(&die_mem) == lineno)
					goto found;

				dwarf_decl_line(&die_mem, &inl);
				if (inl != decl ||
				    decf != die_get_decl_file(&die_mem))
					continue;
			}
		}
found:
		/* Get source line */
		fname = dwarf_linesrc(line, NULL, NULL);

		ret = callback(fname, lineno, addr, data);
		if (ret != 0)
			return ret;
	}

	/*
	 * Dwarf lines doesn't include function declarations and inlined
	 * subroutines. We have to check functions list or given function.
	 */
	if (rt_die != cu_die)
		/*
		 * Don't need walk inlined functions recursively, because
		 * inner inlined functions don't have the lines of the
		 * specified function.
		 */
		ret = __die_walk_funclines(rt_die, false, callback, data);
	else {
		struct __line_walk_param param = {
			.callback = callback,
			.data = data,
			.retval = 0,
		};
		dwarf_getfuncs(cu_die, __die_walk_culines_cb, &param, 0);
		ret = param.retval;
	}

	return ret;
}

struct __find_variable_param {
	const char *name;
	Dwarf_Addr addr;
};

static int __die_find_variable_cb(Dwarf_Die *die_mem, void *data)
{
	struct __find_variable_param *fvp = data;
	Dwarf_Attribute attr;
	int tag;

	tag = dwarf_tag(die_mem);
	if ((tag == DW_TAG_formal_parameter ||
	     tag == DW_TAG_variable) &&
	    die_compare_name(die_mem, fvp->name) &&
	/*
	 * Does the DIE have location information or const value
	 * or external instance?
	 */
	    (dwarf_attr(die_mem, DW_AT_external, &attr) ||
	     dwarf_attr(die_mem, DW_AT_location, &attr) ||
	     dwarf_attr(die_mem, DW_AT_const_value, &attr)))
		return DIE_FIND_CB_END;
	if (dwarf_haspc(die_mem, fvp->addr))
		return DIE_FIND_CB_CONTINUE;
	else
		return DIE_FIND_CB_SIBLING;
}

/**
 * die_find_variable_at - Find a given name variable at given address
 * @sp_die: a function DIE
 * @name: variable name
 * @addr: address
 * @die_mem: a buffer for result DIE
 *
 * Find a variable DIE called @name at @addr in @sp_die.
 */
Dwarf_Die *die_find_variable_at(Dwarf_Die *sp_die, const char *name,
				Dwarf_Addr addr, Dwarf_Die *die_mem)
{
	struct __find_variable_param fvp = { .name = name, .addr = addr};

	return die_find_child(sp_die, __die_find_variable_cb, (void *)&fvp,
			      die_mem);
}

static int __die_find_member_cb(Dwarf_Die *die_mem, void *data)
{
	const char *name = data;

	if (dwarf_tag(die_mem) == DW_TAG_member) {
		if (die_compare_name(die_mem, name))
			return DIE_FIND_CB_END;
		else if (!dwarf_diename(die_mem)) {	/* Unnamed structure */
			Dwarf_Die type_die, tmp_die;
			if (die_get_type(die_mem, &type_die) &&
			    die_find_member(&type_die, name, &tmp_die))
				return DIE_FIND_CB_END;
		}
	}
	return DIE_FIND_CB_SIBLING;
}

/**
 * die_find_member - Find a given name member in a data structure
 * @st_die: a data structure type DIE
 * @name: member name
 * @die_mem: a buffer for result DIE
 *
 * Find a member DIE called @name in @st_die.
 */
Dwarf_Die *die_find_member(Dwarf_Die *st_die, const char *name,
			   Dwarf_Die *die_mem)
{
	return die_find_child(st_die, __die_find_member_cb, (void *)name,
			      die_mem);
}

/**
 * die_get_typename_from_type - Get the name of given type DIE
 * @type_die: a type DIE
 * @buf: a strbuf for result type name
 *
 * Get the name of @type_die and stores it to @buf. Return 0 if succeeded.
 * and Return -ENOENT if failed to find type name.
 * Note that the result will stores typedef name if possible, and stores
 * "*(function_type)" if the type is a function pointer.
 */
int die_get_typename_from_type(Dwarf_Die *type_die, struct strbuf *buf)
{
	int tag, ret;
	const char *tmp = "";

	tag = dwarf_tag(type_die);
	if (tag == DW_TAG_pointer_type)
		tmp = "*";
	else if (tag == DW_TAG_array_type)
		tmp = "[]";
	else if (tag == DW_TAG_subroutine_type) {
		/* Function pointer */
		return strbuf_add(buf, "(function_type)", 15);
	} else {
		const char *name = dwarf_diename(type_die);

		if (tag == DW_TAG_union_type)
			tmp = "union ";
		else if (tag == DW_TAG_structure_type)
			tmp = "struct ";
		else if (tag == DW_TAG_enumeration_type)
			tmp = "enum ";
		else if (name == NULL)
			return -ENOENT;
		/* Write a base name */
		return strbuf_addf(buf, "%s%s", tmp, name ?: "");
	}
	ret = die_get_typename(type_die, buf);
	if (ret < 0) {
		/* void pointer has no type attribute */
		if (tag == DW_TAG_pointer_type && ret == -ENOENT)
			return strbuf_addf(buf, "void*");

		return ret;
	}
	return strbuf_addstr(buf, tmp);
}

/**
 * die_get_typename - Get the name of given variable DIE
 * @vr_die: a variable DIE
 * @buf: a strbuf for result type name
 *
 * Get the name of @vr_die and stores it to @buf. Return 0 if succeeded.
 * and Return -ENOENT if failed to find type name.
 * Note that the result will stores typedef name if possible, and stores
 * "*(function_type)" if the type is a function pointer.
 */
int die_get_typename(Dwarf_Die *vr_die, struct strbuf *buf)
{
	Dwarf_Die type;

	if (__die_get_real_type(vr_die, &type) == NULL)
		return -ENOENT;

	return die_get_typename_from_type(&type, buf);
}

/**
 * die_get_varname - Get the name and type of given variable DIE
 * @vr_die: a variable DIE
 * @buf: a strbuf for type and variable name
 *
 * Get the name and type of @vr_die and stores it in @buf as "type\tname".
 */
int die_get_varname(Dwarf_Die *vr_die, struct strbuf *buf)
{
	int ret;

	ret = die_get_typename(vr_die, buf);
	if (ret < 0) {
		pr_debug("Failed to get type, make it unknown.\n");
		ret = strbuf_add(buf, "(unknown_type)", 14);
	}

	return ret < 0 ? ret : strbuf_addf(buf, "\t%s", dwarf_diename(vr_die));
}

#if defined(HAVE_DWARF_GETLOCATIONS_SUPPORT) || defined(HAVE_DWARF_CFI_SUPPORT)
static int reg_from_dwarf_op(Dwarf_Op *op)
{
	switch (op->atom) {
	case DW_OP_reg0 ... DW_OP_reg31:
		return op->atom - DW_OP_reg0;
	case DW_OP_breg0 ... DW_OP_breg31:
		return op->atom - DW_OP_breg0;
	case DW_OP_regx:
	case DW_OP_bregx:
		return op->number;
	case DW_OP_fbreg:
		return DWARF_REG_FB;
	default:
		break;
	}
	return -1;
}

static int offset_from_dwarf_op(Dwarf_Op *op)
{
	switch (op->atom) {
	case DW_OP_reg0 ... DW_OP_reg31:
	case DW_OP_regx:
		return 0;
	case DW_OP_breg0 ... DW_OP_breg31:
	case DW_OP_fbreg:
		return op->number;
	case DW_OP_bregx:
		return op->number2;
	default:
		break;
	}
	return -1;
}

static bool check_allowed_ops(Dwarf_Op *ops, size_t nops)
{
	/* The first op is checked separately */
	ops++;
	nops--;

	/*
	 * It needs to make sure if the location expression matches to the given
	 * register and offset exactly.  Thus it rejects any complex expressions
	 * and only allows a few of selected operators that doesn't change the
	 * location.
	 */
	while (nops) {
		switch (ops->atom) {
		case DW_OP_stack_value:
		case DW_OP_deref_size:
		case DW_OP_deref:
		case DW_OP_piece:
			break;
		default:
			return false;
		}
		ops++;
		nops--;
	}
	return true;
}
#endif /* HAVE_DWARF_GETLOCATIONS_SUPPORT || HAVE_DWARF_CFI_SUPPORT */

#ifdef HAVE_DWARF_GETLOCATIONS_SUPPORT
/**
 * die_get_var_innermost_scope - Get innermost scope range of given variable DIE
 * @sp_die: a subprogram DIE
 * @vr_die: a variable DIE
 * @buf: a strbuf for variable byte offset range
 *
 * Get the innermost scope range of @vr_die and stores it in @buf as
 * "@<function_name+[NN-NN,NN-NN]>".
 */
static int die_get_var_innermost_scope(Dwarf_Die *sp_die, Dwarf_Die *vr_die,
				struct strbuf *buf)
{
	Dwarf_Die *scopes;
	int count;
	size_t offset = 0;
	Dwarf_Addr base;
	Dwarf_Addr start, end;
	Dwarf_Addr entry;
	int ret;
	bool first = true;
	const char *name;

	ret = die_entrypc(sp_die, &entry);
	if (ret)
		return ret;

	name = dwarf_diename(sp_die);
	if (!name)
		return -ENOENT;

	count = dwarf_getscopes_die(vr_die, &scopes);

	/* (*SCOPES)[1] is the DIE for the scope containing that scope */
	if (count <= 1) {
		ret = -EINVAL;
		goto out;
	}

	while ((offset = dwarf_ranges(&scopes[1], offset, &base,
					&start, &end)) > 0) {
		start -= entry;
		end -= entry;

		if (first) {
			ret = strbuf_addf(buf, "@<%s+[%" PRIu64 "-%" PRIu64,
					  name, start, end);
			first = false;
		} else {
			ret = strbuf_addf(buf, ",%" PRIu64 "-%" PRIu64,
					  start, end);
		}
		if (ret < 0)
			goto out;
	}

	if (!first)
		ret = strbuf_add(buf, "]>", 2);

out:
	free(scopes);
	return ret;
}

/**
 * die_get_var_range - Get byte offset range of given variable DIE
 * @sp_die: a subprogram DIE
 * @vr_die: a variable DIE
 * @buf: a strbuf for type and variable name and byte offset range
 *
 * Get the byte offset range of @vr_die and stores it in @buf as
 * "@<function_name+[NN-NN,NN-NN]>".
 */
int die_get_var_range(Dwarf_Die *sp_die, Dwarf_Die *vr_die, struct strbuf *buf)
{
	int ret = 0;
	Dwarf_Addr base;
	Dwarf_Addr start, end;
	Dwarf_Addr entry;
	Dwarf_Op *op;
	size_t nops;
	size_t offset = 0;
	Dwarf_Attribute attr;
	bool first = true;
	const char *name;

	ret = die_entrypc(sp_die, &entry);
	if (ret)
		return ret;

	name = dwarf_diename(sp_die);
	if (!name)
		return -ENOENT;

	if (dwarf_attr(vr_die, DW_AT_location, &attr) == NULL)
		return -EINVAL;

	while ((offset = dwarf_getlocations(&attr, offset, &base,
					&start, &end, &op, &nops)) > 0) {
		if (start == 0) {
			/* Single Location Descriptions */
			ret = die_get_var_innermost_scope(sp_die, vr_die, buf);
			goto out;
		}

		/* Location Lists */
		start -= entry;
		end -= entry;
		if (first) {
			ret = strbuf_addf(buf, "@<%s+[%" PRIu64 "-%" PRIu64,
					  name, start, end);
			first = false;
		} else {
			ret = strbuf_addf(buf, ",%" PRIu64 "-%" PRIu64,
					  start, end);
		}
		if (ret < 0)
			goto out;
	}

	if (!first)
		ret = strbuf_add(buf, "]>", 2);
out:
	return ret;
}

/* Interval parameters for __die_find_var_reg_cb() */
struct find_var_data {
	/* Target instruction address */
	Dwarf_Addr pc;
	/* Target memory address (for global data) */
	Dwarf_Addr addr;
	/* Target register */
	unsigned reg;
	/* Access offset, set for global data */
	int offset;
	/* True if the current register is the frame base */
	bool is_fbreg;
};

/* Max number of registers DW_OP_regN supports */
#define DWARF_OP_DIRECT_REGS  32

static bool match_var_offset(Dwarf_Die *die_mem, struct find_var_data *data,
			     u64 addr_offset, u64 addr_type, bool is_pointer)
{
	Dwarf_Die type_die;
	Dwarf_Word size;

	if (addr_offset == addr_type) {
		/* Update offset relative to the start of the variable */
		data->offset = 0;
		return true;
	}

	if (addr_offset < addr_type)
		return false;

	if (die_get_real_type(die_mem, &type_die) == NULL)
		return false;

	if (is_pointer && dwarf_tag(&type_die) == DW_TAG_pointer_type) {
		/* Get the target type of the pointer */
		if (die_get_real_type(&type_die, &type_die) == NULL)
			return false;
	}

	if (dwarf_aggregate_size(&type_die, &size) < 0)
		return false;

	if (addr_offset >= addr_type + size)
		return false;

	/* Update offset relative to the start of the variable */
	data->offset = addr_offset - addr_type;
	return true;
}

/* Only checks direct child DIEs in the given scope. */
static int __die_find_var_reg_cb(Dwarf_Die *die_mem, void *arg)
{
	struct find_var_data *data = arg;
	int tag = dwarf_tag(die_mem);
	ptrdiff_t off = 0;
	Dwarf_Attribute attr;
	Dwarf_Addr base, start, end;
	Dwarf_Op *ops;
	size_t nops;

	if (tag != DW_TAG_variable && tag != DW_TAG_formal_parameter)
		return DIE_FIND_CB_SIBLING;

	if (dwarf_attr(die_mem, DW_AT_location, &attr) == NULL)
		return DIE_FIND_CB_SIBLING;

	while ((off = dwarf_getlocations(&attr, off, &base, &start, &end, &ops, &nops)) > 0) {
		/* Assuming the location list is sorted by address */
		if (end < data->pc)
			continue;
		if (start > data->pc)
			break;

		/* Local variables accessed using frame base register */
		if (data->is_fbreg && ops->atom == DW_OP_fbreg &&
		    check_allowed_ops(ops, nops) &&
		    match_var_offset(die_mem, data, data->offset, ops->number,
				     /*is_pointer=*/false))
			return DIE_FIND_CB_END;

		/* Only match with a simple case */
		if (data->reg < DWARF_OP_DIRECT_REGS) {
			/* pointer variables saved in a register 0 to 31 */
			if (ops->atom == (DW_OP_reg0 + data->reg) &&
			    check_allowed_ops(ops, nops) &&
			    match_var_offset(die_mem, data, data->offset, 0,
					     /*is_pointer=*/true))
				return DIE_FIND_CB_END;

			/* Local variables accessed by a register + offset */
			if (ops->atom == (DW_OP_breg0 + data->reg) &&
			    check_allowed_ops(ops, nops) &&
			    match_var_offset(die_mem, data, data->offset, ops->number,
					     /*is_pointer=*/false))
				return DIE_FIND_CB_END;
		} else {
			/* pointer variables saved in a register 32 or above */
			if (ops->atom == DW_OP_regx && ops->number == data->reg &&
			    check_allowed_ops(ops, nops) &&
			    match_var_offset(die_mem, data, data->offset, 0,
					     /*is_pointer=*/true))
				return DIE_FIND_CB_END;

			/* Local variables accessed by a register + offset */
			if (ops->atom == DW_OP_bregx && data->reg == ops->number &&
			    check_allowed_ops(ops, nops) &&
			    match_var_offset(die_mem, data, data->offset, ops->number2,
					     /*is_poitner=*/false))
				return DIE_FIND_CB_END;
		}
	}
	return DIE_FIND_CB_SIBLING;
}

/**
 * die_find_variable_by_reg - Find a variable saved in a register
 * @sc_die: a scope DIE
 * @pc: the program address to find
 * @reg: the register number to find
 * @poffset: pointer to offset, will be updated for fbreg case
 * @is_fbreg: boolean value if the current register is the frame base
 * @die_mem: a buffer to save the resulting DIE
 *
 * Find the variable DIE accessed by the given register.  It'll update the @offset
 * when the variable is in the stack.
 */
Dwarf_Die *die_find_variable_by_reg(Dwarf_Die *sc_die, Dwarf_Addr pc, int reg,
				    int *poffset, bool is_fbreg,
				    Dwarf_Die *die_mem)
{
	struct find_var_data data = {
		.pc = pc,
		.reg = reg,
		.offset = *poffset,
		.is_fbreg = is_fbreg,
	};
	Dwarf_Die *result;

	result = die_find_child(sc_die, __die_find_var_reg_cb, &data, die_mem);
	if (result)
		*poffset = data.offset;
	return result;
}

/* Only checks direct child DIEs in the given scope */
static int __die_find_var_addr_cb(Dwarf_Die *die_mem, void *arg)
{
	struct find_var_data *data = arg;
	int tag = dwarf_tag(die_mem);
	ptrdiff_t off = 0;
	Dwarf_Attribute attr;
	Dwarf_Addr base, start, end;
	Dwarf_Op *ops;
	size_t nops;

	if (tag != DW_TAG_variable)
		return DIE_FIND_CB_SIBLING;

	if (dwarf_attr(die_mem, DW_AT_location, &attr) == NULL)
		return DIE_FIND_CB_SIBLING;

	while ((off = dwarf_getlocations(&attr, off, &base, &start, &end, &ops, &nops)) > 0) {
		if (ops->atom != DW_OP_addr)
			continue;

		if (check_allowed_ops(ops, nops) &&
		    match_var_offset(die_mem, data, data->addr, ops->number,
				     /*is_pointer=*/false))
			return DIE_FIND_CB_END;
	}
	return DIE_FIND_CB_SIBLING;
}

/**
 * die_find_variable_by_addr - Find variable located at given address
 * @sc_die: a scope DIE
 * @addr: the data address to find
 * @die_mem: a buffer to save the resulting DIE
 * @offset: the offset in the resulting type
 *
 * Find the variable DIE located at the given address (in PC-relative mode).
 * This is usually for global variables.
 */
Dwarf_Die *die_find_variable_by_addr(Dwarf_Die *sc_die, Dwarf_Addr addr,
				     Dwarf_Die *die_mem, int *offset)
{
	struct find_var_data data = {
		.addr = addr,
	};
	Dwarf_Die *result;

	result = die_find_child(sc_die, __die_find_var_addr_cb, &data, die_mem);
	if (result)
		*offset = data.offset;
	return result;
}

static int __die_collect_vars_cb(Dwarf_Die *die_mem, void *arg)
{
	struct die_var_type **var_types = arg;
	Dwarf_Die type_die;
	int tag = dwarf_tag(die_mem);
	Dwarf_Attribute attr;
	Dwarf_Addr base, start, end;
	Dwarf_Op *ops;
	size_t nops;
	struct die_var_type *vt;

	if (tag != DW_TAG_variable && tag != DW_TAG_formal_parameter)
		return DIE_FIND_CB_SIBLING;

	if (dwarf_attr(die_mem, DW_AT_location, &attr) == NULL)
		return DIE_FIND_CB_SIBLING;

	/*
	 * Only collect the first location as it can reconstruct the
	 * remaining state by following the instructions.
	 * start = 0 means it covers the whole range.
	 */
	if (dwarf_getlocations(&attr, 0, &base, &start, &end, &ops, &nops) <= 0)
		return DIE_FIND_CB_SIBLING;

	if (die_get_real_type(die_mem, &type_die) == NULL)
		return DIE_FIND_CB_SIBLING;

	vt = malloc(sizeof(*vt));
	if (vt == NULL)
		return DIE_FIND_CB_END;

	vt->die_off = dwarf_dieoffset(&type_die);
	vt->addr = start;
	vt->reg = reg_from_dwarf_op(ops);
	vt->offset = offset_from_dwarf_op(ops);
	vt->next = *var_types;
	*var_types = vt;

	return DIE_FIND_CB_SIBLING;
}

/**
 * die_collect_vars - Save all variables and parameters
 * @sc_die: a scope DIE
 * @var_types: a pointer to save the resulting list
 *
 * Save all variables and parameters in the @sc_die and save them to @var_types.
 * The @var_types is a singly-linked list containing type and location info.
 * Actual type can be retrieved using dwarf_offdie() with 'die_off' later.
 *
 * Callers should free @var_types.
 */
void die_collect_vars(Dwarf_Die *sc_die, struct die_var_type **var_types)
{
	Dwarf_Die die_mem;

	die_find_child(sc_die, __die_collect_vars_cb, (void *)var_types, &die_mem);
}

static int __die_collect_global_vars_cb(Dwarf_Die *die_mem, void *arg)
{
	struct die_var_type **var_types = arg;
	Dwarf_Die type_die;
	int tag = dwarf_tag(die_mem);
	Dwarf_Attribute attr;
	Dwarf_Addr base, start, end;
	Dwarf_Op *ops;
	size_t nops;
	struct die_var_type *vt;

	if (tag != DW_TAG_variable)
		return DIE_FIND_CB_SIBLING;

	if (dwarf_attr(die_mem, DW_AT_location, &attr) == NULL)
		return DIE_FIND_CB_SIBLING;

	/* Only collect the location with an absolute address. */
	if (dwarf_getlocations(&attr, 0, &base, &start, &end, &ops, &nops) <= 0)
		return DIE_FIND_CB_SIBLING;

	if (ops->atom != DW_OP_addr)
		return DIE_FIND_CB_SIBLING;

	if (!check_allowed_ops(ops, nops))
		return DIE_FIND_CB_SIBLING;

	if (die_get_real_type(die_mem, &type_die) == NULL)
		return DIE_FIND_CB_SIBLING;

	vt = malloc(sizeof(*vt));
	if (vt == NULL)
		return DIE_FIND_CB_END;

	vt->die_off = dwarf_dieoffset(&type_die);
	vt->addr = ops->number;
	vt->reg = -1;
	vt->offset = 0;
	vt->next = *var_types;
	*var_types = vt;

	return DIE_FIND_CB_SIBLING;
}

/**
 * die_collect_global_vars - Save all global variables
 * @cu_die: a CU DIE
 * @var_types: a pointer to save the resulting list
 *
 * Save all global variables in the @cu_die and save them to @var_types.
 * The @var_types is a singly-linked list containing type and location info.
 * Actual type can be retrieved using dwarf_offdie() with 'die_off' later.
 *
 * Callers should free @var_types.
 */
void die_collect_global_vars(Dwarf_Die *cu_die, struct die_var_type **var_types)
{
	Dwarf_Die die_mem;

	die_find_child(cu_die, __die_collect_global_vars_cb, (void *)var_types, &die_mem);
}
#endif /* HAVE_DWARF_GETLOCATIONS_SUPPORT */

#ifdef HAVE_DWARF_CFI_SUPPORT
/**
 * die_get_cfa - Get frame base information
 * @dwarf: a Dwarf info
 * @pc: program address
 * @preg: pointer for saved register
 * @poffset: pointer for saved offset
 *
 * This function gets register and offset for CFA (Canonical Frame Address)
 * by searching the CIE/FDE info.  The CFA usually points to the start address
 * of the current stack frame and local variables can be located using an offset
 * from the CFA.  The @preg and @poffset will be updated if it returns 0.
 */
int die_get_cfa(Dwarf *dwarf, u64 pc, int *preg, int *poffset)
{
	Dwarf_CFI *cfi;
	Dwarf_Frame *frame = NULL;
	Dwarf_Op *ops = NULL;
	size_t nops;

	cfi = dwarf_getcfi(dwarf);
	if (cfi == NULL)
		return -1;

	if (!dwarf_cfi_addrframe(cfi, pc, &frame) &&
	    !dwarf_frame_cfa(frame, &ops, &nops) &&
	    check_allowed_ops(ops, nops)) {
		*preg = reg_from_dwarf_op(ops);
		*poffset = offset_from_dwarf_op(ops);
		return 0;
	}
	return -1;
}
#endif /* HAVE_DWARF_CFI_SUPPORT */

/*
 * die_has_loclist - Check if DW_AT_location of @vr_die is a location list
 * @vr_die: a variable DIE
 */
static bool die_has_loclist(Dwarf_Die *vr_die)
{
	Dwarf_Attribute loc;
	int tag = dwarf_tag(vr_die);

	if (tag != DW_TAG_formal_parameter &&
	    tag != DW_TAG_variable)
		return false;

	return (dwarf_attr_integrate(vr_die, DW_AT_location, &loc) &&
		dwarf_whatform(&loc) == DW_FORM_sec_offset);
}

/*
 * die_is_optimized_target - Check if target program is compiled with
 * optimization
 * @cu_die: a CU DIE
 *
 * For any object in given CU whose DW_AT_location is a location list,
 * target program is compiled with optimization. This is applicable to
 * clang as well.
 */
bool die_is_optimized_target(Dwarf_Die *cu_die)
{
	Dwarf_Die tmp_die;

	if (die_has_loclist(cu_die))
		return true;

	if (!dwarf_child(cu_die, &tmp_die) &&
	    die_is_optimized_target(&tmp_die))
		return true;

	if (!dwarf_siblingof(cu_die, &tmp_die) &&
	    die_is_optimized_target(&tmp_die))
		return true;

	return false;
}

/*
 * die_search_idx - Search index of given line address
 * @lines: Line records of single CU
 * @nr_lines: Number of @lines
 * @addr: address we are looking for
 * @idx: index to be set by this function (return value)
 *
 * Search for @addr by looping over every lines of CU. If address
 * matches, set index of that line in @idx. Note that single source
 * line can have multiple line records. i.e. single source line can
 * have multiple index.
 */
static bool die_search_idx(Dwarf_Lines *lines, unsigned long nr_lines,
			   Dwarf_Addr addr, unsigned long *idx)
{
	unsigned long i;
	Dwarf_Addr tmp;

	for (i = 0; i < nr_lines; i++) {
		if (dwarf_lineaddr(dwarf_onesrcline(lines, i), &tmp))
			return false;

		if (tmp == addr) {
			*idx = i;
			return true;
		}
	}
	return false;
}

/*
 * die_get_postprologue_addr - Search next address after function prologue
 * @entrypc_idx: entrypc index
 * @lines: Line records of single CU
 * @nr_lines: Number of @lines
 * @hignpc: high PC address of function
 * @postprologue_addr: Next address after function prologue (return value)
 *
 * Look for prologue-end marker. If there is no explicit marker, return
 * address of next line record or next source line.
 */
static bool die_get_postprologue_addr(unsigned long entrypc_idx,
				      Dwarf_Lines *lines,
				      unsigned long nr_lines,
				      Dwarf_Addr highpc,
				      Dwarf_Addr *postprologue_addr)
{
	unsigned long i;
	int entrypc_lno, lno;
	Dwarf_Line *line;
	Dwarf_Addr addr;
	bool p_end;

	/* entrypc_lno is actual source line number */
	line = dwarf_onesrcline(lines, entrypc_idx);
	if (dwarf_lineno(line, &entrypc_lno))
		return false;

	for (i = entrypc_idx; i < nr_lines; i++) {
		line = dwarf_onesrcline(lines, i);

		if (dwarf_lineaddr(line, &addr) ||
		    dwarf_lineno(line, &lno)    ||
		    dwarf_lineprologueend(line, &p_end))
			return false;

		/* highpc is exclusive. [entrypc,highpc) */
		if (addr >= highpc)
			break;

		/* clang supports prologue-end marker */
		if (p_end)
			break;

		/* Actual next line in source */
		if (lno != entrypc_lno)
			break;

		/*
		 * Single source line can have multiple line records.
		 * For Example,
		 *     void foo() { printf("hello\n"); }
		 * contains two line records. One points to declaration and
		 * other points to printf() line. Variable 'lno' won't get
		 * incremented in this case but 'i' will.
		 */
		if (i != entrypc_idx)
			break;
	}

	dwarf_lineaddr(line, postprologue_addr);
	if (*postprologue_addr >= highpc)
		dwarf_lineaddr(dwarf_onesrcline(lines, i - 1),
			       postprologue_addr);

	return true;
}

/*
 * die_skip_prologue - Use next address after prologue as probe location
 * @sp_die: a subprogram DIE
 * @cu_die: a CU DIE
 * @entrypc: entrypc of the function
 *
 * Function prologue prepares stack and registers before executing function
 * logic. When target program is compiled without optimization, function
 * parameter information is only valid after prologue. When we probe entrypc
 * of the function, and try to record function parameter, it contains
 * garbage value.
 */
void die_skip_prologue(Dwarf_Die *sp_die, Dwarf_Die *cu_die,
		       Dwarf_Addr *entrypc)
{
	size_t nr_lines = 0;
	unsigned long entrypc_idx = 0;
	Dwarf_Lines *lines = NULL;
	Dwarf_Addr postprologue_addr;
	Dwarf_Addr highpc;

	if (dwarf_highpc(sp_die, &highpc))
		return;

	if (dwarf_getsrclines(cu_die, &lines, &nr_lines))
		return;

	if (!die_search_idx(lines, nr_lines, *entrypc, &entrypc_idx))
		return;

	if (!die_get_postprologue_addr(entrypc_idx, lines, nr_lines,
				       highpc, &postprologue_addr))
		return;

	*entrypc = postprologue_addr;
}

/* Internal parameters for __die_find_scope_cb() */
struct find_scope_data {
	/* Target instruction address */
	Dwarf_Addr pc;
	/* Number of scopes found [output] */
	int nr;
	/* Array of scopes found, 0 for the outermost one. [output] */
	Dwarf_Die *scopes;
};

static int __die_find_scope_cb(Dwarf_Die *die_mem, void *arg)
{
	struct find_scope_data *data = arg;

	if (dwarf_haspc(die_mem, data->pc)) {
		Dwarf_Die *tmp;

		tmp = realloc(data->scopes, (data->nr + 1) * sizeof(*tmp));
		if (tmp == NULL)
			return DIE_FIND_CB_END;

		memcpy(tmp + data->nr, die_mem, sizeof(*die_mem));
		data->scopes = tmp;
		data->nr++;
		return DIE_FIND_CB_CHILD;
	}
	return DIE_FIND_CB_SIBLING;
}

/**
 * die_get_scopes - Return a list of scopes including the address
 * @cu_die: a compile unit DIE
 * @pc: the address to find
 * @scopes: the array of DIEs for scopes (result)
 *
 * This function does the same as the dwarf_getscopes() but doesn't follow
 * the origins of inlined functions.  It returns the number of scopes saved
 * in the @scopes argument.  The outer scope will be saved first (index 0) and
 * the last one is the innermost scope at the @pc.
 */
int die_get_scopes(Dwarf_Die *cu_die, Dwarf_Addr pc, Dwarf_Die **scopes)
{
	struct find_scope_data data = {
		.pc = pc,
	};
	Dwarf_Die die_mem;

	die_find_child(cu_die, __die_find_scope_cb, &data, &die_mem);

	*scopes = data.scopes;
	return data.nr;
}

static int __die_find_member_offset_cb(Dwarf_Die *die_mem, void *arg)
{
	Dwarf_Die type_die;
	Dwarf_Word size, loc;
	Dwarf_Word offset = (long)arg;
	int tag = dwarf_tag(die_mem);

	if (tag != DW_TAG_member)
		return DIE_FIND_CB_SIBLING;

	/* Unions might not have location */
	if (die_get_data_member_location(die_mem, &loc) < 0)
		loc = 0;

	if (offset == loc)
		return DIE_FIND_CB_END;

	if (die_get_real_type(die_mem, &type_die) == NULL) {
		// TODO: add a pr_debug_dtp() later for this unlikely failure
		return DIE_FIND_CB_SIBLING;
	}

	if (dwarf_aggregate_size(&type_die, &size) < 0)
		size = 0;

	if (loc < offset && offset < (loc + size))
		return DIE_FIND_CB_END;

	return DIE_FIND_CB_SIBLING;
}

/**
 * die_get_member_type - Return type info of struct member
 * @type_die: a type DIE
 * @offset: offset in the type
 * @die_mem: a buffer to save the resulting DIE
 *
 * This function returns a type of a member in @type_die where it's located at
 * @offset if it's a struct.  For now, it just returns the first matching
 * member in a union.  For other types, it'd return the given type directly
 * if it's within the size of the type or NULL otherwise.
 */
Dwarf_Die *die_get_member_type(Dwarf_Die *type_die, int offset,
			       Dwarf_Die *die_mem)
{
	Dwarf_Die *member;
	Dwarf_Die mb_type;
	int tag;

	tag = dwarf_tag(type_die);
	/* If it's not a compound type, return the type directly */
	if (tag != DW_TAG_structure_type && tag != DW_TAG_union_type) {
		Dwarf_Word size;

		if (dwarf_aggregate_size(type_die, &size) < 0)
			size = 0;

		if ((unsigned)offset >= size)
			return NULL;

		*die_mem = *type_die;
		return die_mem;
	}

	mb_type = *type_die;
	/* TODO: Handle union types better? */
	while (tag == DW_TAG_structure_type || tag == DW_TAG_union_type) {
		member = die_find_child(&mb_type, __die_find_member_offset_cb,
					(void *)(long)offset, die_mem);
		if (member == NULL)
			return NULL;

		if (die_get_real_type(member, &mb_type) == NULL)
			return NULL;

		tag = dwarf_tag(&mb_type);

		if (tag == DW_TAG_structure_type || tag == DW_TAG_union_type) {
			Dwarf_Word loc;

			/* Update offset for the start of the member struct */
			if (die_get_data_member_location(member, &loc) == 0)
				offset -= loc;
		}
	}
	*die_mem = mb_type;
	return die_mem;
}

/**
 * die_deref_ptr_type - Return type info for pointer access
 * @ptr_die: a pointer type DIE
 * @offset: access offset for the pointer
 * @die_mem: a buffer to save the resulting DIE
 *
 * This function follows the pointer in @ptr_die with given @offset
 * and saves the resulting type in @die_mem.  If the pointer points
 * a struct type, actual member at the offset would be returned.
 */
Dwarf_Die *die_deref_ptr_type(Dwarf_Die *ptr_die, int offset,
			      Dwarf_Die *die_mem)
{
	Dwarf_Die type_die;

	if (dwarf_tag(ptr_die) != DW_TAG_pointer_type)
		return NULL;

	if (die_get_real_type(ptr_die, &type_die) == NULL)
		return NULL;

	return die_get_member_type(&type_die, offset, die_mem);
}
