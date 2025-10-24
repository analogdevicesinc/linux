// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * elf.c - ELF access library
 *
 * Adapted from kpatch (https://github.com/dynup/kpatch):
 * Copyright (C) 2013-2015 Josh Poimboeuf <jpoimboe@redhat.com>
 * Copyright (C) 2014 Seth Jennings <sjenning@redhat.com>
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <libgen.h>
#include <ctype.h>
#include <linux/interval_tree_generic.h>
#include <objtool/builtin.h>
#include <objtool/elf.h>
#include <objtool/warn.h>

#define ALIGN_UP(x, align_to) (((x) + ((align_to)-1)) & ~((align_to)-1))
#define ALIGN_UP_POW2(x) (1U << ((8 * sizeof(x)) - __builtin_clz((x) - 1U)))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static inline u32 str_hash(const char *str)
{
	return jhash(str, strlen(str), 0);
}

#define __elf_table(name)	(elf->name##_hash)
#define __elf_bits(name)	(elf->name##_bits)

#define __elf_table_entry(name, key) \
	__elf_table(name)[hash_min(key, __elf_bits(name))]

#define elf_hash_add(name, node, key)					\
({									\
	struct elf_hash_node *__node = node;				\
	__node->next = __elf_table_entry(name, key);			\
	__elf_table_entry(name, key) = __node;				\
})

static inline void __elf_hash_del(struct elf_hash_node *node,
				  struct elf_hash_node **head)
{
	struct elf_hash_node *cur, *prev;

	if (node == *head) {
		*head = node->next;
		return;
	}

	for (prev = NULL, cur = *head; cur; prev = cur, cur = cur->next) {
		if (cur == node) {
			prev->next = cur->next;
			break;
		}
	}
}

#define elf_hash_del(name, node, key) \
	__elf_hash_del(node, &__elf_table_entry(name, key))

#define elf_list_entry(ptr, type, member)				\
({									\
	typeof(ptr) __ptr = (ptr);					\
	__ptr ? container_of(__ptr, type, member) : NULL;		\
})

#define elf_hash_for_each_possible(name, obj, member, key)		\
	for (obj = elf_list_entry(__elf_table_entry(name, key), typeof(*obj), member); \
	     obj;							\
	     obj = elf_list_entry(obj->member.next, typeof(*(obj)), member))

#define elf_alloc_hash(name, size)					\
({									\
	__elf_bits(name) = max(10, ilog2(size));			\
	__elf_table(name) = mmap(NULL, sizeof(struct elf_hash_node *) << __elf_bits(name), \
				 PROT_READ|PROT_WRITE,			\
				 MAP_PRIVATE|MAP_ANON, -1, 0);		\
	if (__elf_table(name) == (void *)-1L) {				\
		ERROR_GLIBC("mmap fail " #name);			\
		__elf_table(name) = NULL;				\
	}								\
	__elf_table(name);						\
})

static inline unsigned long __sym_start(struct symbol *s)
{
	return s->offset;
}

static inline unsigned long __sym_last(struct symbol *s)
{
	return s->offset + (s->len ? s->len - 1 : 0);
}

INTERVAL_TREE_DEFINE(struct symbol, node, unsigned long, __subtree_last,
		     __sym_start, __sym_last, static inline __maybe_unused,
		     __sym)

#define __sym_for_each(_iter, _tree, _start, _end)			\
	for (_iter = __sym_iter_first((_tree), (_start), (_end));	\
	     _iter; _iter = __sym_iter_next(_iter, (_start), (_end)))

struct symbol_hole {
	unsigned long key;
	const struct symbol *sym;
};

/*
 * Find the last symbol before @offset.
 */
static int symbol_hole_by_offset(const void *key, const struct rb_node *node)
{
	const struct symbol *s = rb_entry(node, struct symbol, node);
	struct symbol_hole *sh = (void *)key;

	if (sh->key < s->offset)
		return -1;

	if (sh->key >= s->offset + s->len) {
		sh->sym = s;
		return 1;
	}

	return 0;
}

struct section *find_section_by_name(const struct elf *elf, const char *name)
{
	struct section *sec;

	elf_hash_for_each_possible(section_name, sec, name_hash, str_hash(name)) {
		if (!strcmp(sec->name, name))
			return sec;
	}

	return NULL;
}

static struct section *find_section_by_index(struct elf *elf,
					     unsigned int idx)
{
	struct section *sec;

	elf_hash_for_each_possible(section, sec, hash, idx) {
		if (sec->idx == idx)
			return sec;
	}

	return NULL;
}

static struct symbol *find_symbol_by_index(struct elf *elf, unsigned int idx)
{
	struct symbol *sym;

	elf_hash_for_each_possible(symbol, sym, hash, idx) {
		if (sym->idx == idx)
			return sym;
	}

	return NULL;
}

struct symbol *find_symbol_by_offset(struct section *sec, unsigned long offset)
{
	struct rb_root_cached *tree = (struct rb_root_cached *)&sec->symbol_tree;
	struct symbol *iter;

	__sym_for_each(iter, tree, offset, offset) {
		if (iter->offset == offset && !is_sec_sym(iter))
			return iter;
	}

	return NULL;
}

struct symbol *find_func_by_offset(struct section *sec, unsigned long offset)
{
	struct rb_root_cached *tree = (struct rb_root_cached *)&sec->symbol_tree;
	struct symbol *iter;

	__sym_for_each(iter, tree, offset, offset) {
		if (iter->offset == offset && is_func_sym(iter))
			return iter;
	}

	return NULL;
}

struct symbol *find_symbol_containing(const struct section *sec, unsigned long offset)
{
	struct rb_root_cached *tree = (struct rb_root_cached *)&sec->symbol_tree;
	struct symbol *sym = NULL, *tmp;

	__sym_for_each(tmp, tree, offset, offset) {
		if (tmp->len) {
			if (!sym) {
				sym = tmp;
				continue;
			}

			if (sym->offset != tmp->offset || sym->len != tmp->len) {
				/*
				 * In the rare case of overlapping symbols,
				 * pick the smaller one.
				 *
				 * TODO: outlaw overlapping symbols
				 */
				if (tmp->len < sym->len)
					sym = tmp;
			}
		}
	}

	return sym;
}

/*
 * Returns size of hole starting at @offset.
 */
int find_symbol_hole_containing(const struct section *sec, unsigned long offset)
{
	struct symbol_hole hole = {
		.key = offset,
		.sym = NULL,
	};
	struct rb_node *n;
	struct symbol *s;

	/*
	 * Find the rightmost symbol for which @offset is after it.
	 */
	n = rb_find(&hole, &sec->symbol_tree.rb_root, symbol_hole_by_offset);

	/* found a symbol that contains @offset */
	if (n)
		return 0; /* not a hole */

	/*
	 * @offset >= sym->offset + sym->len, find symbol after it.
	 * When hole.sym is empty, use the first node to compute the hole.
	 * If there is no symbol in the section, the first node will be NULL,
	 * in which case, -1 is returned to skip the whole section.
	 */
	if (hole.sym)
		n = rb_next(&hole.sym->node);
	else
		n = rb_first_cached(&sec->symbol_tree);

	if (!n)
		return -1; /* until end of address space */

	/* hole until start of next symbol */
	s = rb_entry(n, struct symbol, node);
	return s->offset - offset;
}

struct symbol *find_func_containing(struct section *sec, unsigned long offset)
{
	struct rb_root_cached *tree = (struct rb_root_cached *)&sec->symbol_tree;
	struct symbol *iter;

	__sym_for_each(iter, tree, offset, offset) {
		if (is_func_sym(iter))
			return iter;
	}

	return NULL;
}

struct symbol *find_symbol_by_name(const struct elf *elf, const char *name)
{
	struct symbol *sym;

	elf_hash_for_each_possible(symbol_name, sym, name_hash, str_hash(name)) {
		if (!strcmp(sym->name, name))
			return sym;
	}

	return NULL;
}

struct symbol *find_global_symbol_by_name(const struct elf *elf, const char *name)
{
	struct symbol *sym;

	elf_hash_for_each_possible(symbol_name, sym, name_hash, str_hash(name)) {
		if (!strcmp(sym->name, name) && !is_local_sym(sym))
			return sym;
	}

	return NULL;
}

struct reloc *find_reloc_by_dest_range(const struct elf *elf, struct section *sec,
				     unsigned long offset, unsigned int len)
{
	struct reloc *reloc, *r = NULL;
	struct section *rsec;
	unsigned long o;

	rsec = sec->rsec;
	if (!rsec)
		return NULL;

	for_offset_range(o, offset, offset + len) {
		elf_hash_for_each_possible(reloc, reloc, hash,
					   sec_offset_hash(rsec, o)) {
			if (reloc->sec != rsec)
				continue;

			if (reloc_offset(reloc) >= offset &&
			    reloc_offset(reloc) < offset + len) {
				if (!r || reloc_offset(reloc) < reloc_offset(r))
					r = reloc;
			}
		}
		if (r)
			return r;
	}

	return NULL;
}

struct reloc *find_reloc_by_dest(const struct elf *elf, struct section *sec, unsigned long offset)
{
	return find_reloc_by_dest_range(elf, sec, offset, 1);
}

static bool is_dwarf_section(struct section *sec)
{
	return !strncmp(sec->name, ".debug_", 7);
}

static int read_sections(struct elf *elf)
{
	Elf_Scn *s = NULL;
	struct section *sec;
	size_t shstrndx, sections_nr;
	int i;

	if (elf_getshdrnum(elf->elf, &sections_nr)) {
		ERROR_ELF("elf_getshdrnum");
		return -1;
	}

	if (elf_getshdrstrndx(elf->elf, &shstrndx)) {
		ERROR_ELF("elf_getshdrstrndx");
		return -1;
	}

	if (!elf_alloc_hash(section, sections_nr) ||
	    !elf_alloc_hash(section_name, sections_nr))
		return -1;

	elf->section_data = calloc(sections_nr, sizeof(*sec));
	if (!elf->section_data) {
		ERROR_GLIBC("calloc");
		return -1;
	}
	for (i = 0; i < sections_nr; i++) {
		sec = &elf->section_data[i];

		INIT_LIST_HEAD(&sec->symbol_list);

		s = elf_getscn(elf->elf, i);
		if (!s) {
			ERROR_ELF("elf_getscn");
			return -1;
		}

		sec->idx = elf_ndxscn(s);

		if (!gelf_getshdr(s, &sec->sh)) {
			ERROR_ELF("gelf_getshdr");
			return -1;
		}

		sec->name = elf_strptr(elf->elf, shstrndx, sec->sh.sh_name);
		if (!sec->name) {
			ERROR_ELF("elf_strptr");
			return -1;
		}

		if (sec_size(sec) != 0 && !is_dwarf_section(sec)) {
			sec->data = elf_getdata(s, NULL);
			if (!sec->data) {
				ERROR_ELF("elf_getdata");
				return -1;
			}
			if (sec->data->d_off != 0 ||
			    sec->data->d_size != sec_size(sec)) {
				ERROR("unexpected data attributes for %s", sec->name);
				return -1;
			}
		}

		list_add_tail(&sec->list, &elf->sections);
		elf_hash_add(section, &sec->hash, sec->idx);
		elf_hash_add(section_name, &sec->name_hash, str_hash(sec->name));

		if (is_reloc_sec(sec))
			elf->num_relocs += sec_num_entries(sec);
	}

	if (opts.stats) {
		printf("nr_sections: %lu\n", (unsigned long)sections_nr);
		printf("section_bits: %d\n", elf->section_bits);
	}

	/* sanity check, one more call to elf_nextscn() should return NULL */
	if (elf_nextscn(elf->elf, s)) {
		ERROR("section entry mismatch");
		return -1;
	}

	return 0;
}

static const char *demangle_name(struct symbol *sym)
{
	char *str;

	if (!is_local_sym(sym))
		return sym->name;

	if (!is_func_sym(sym) && !is_object_sym(sym))
		return sym->name;

	if (!strstarts(sym->name, "__UNIQUE_ID_") && !strchr(sym->name, '.'))
		return sym->name;

	str = strdup(sym->name);
	if (!str) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	for (int i = strlen(str) - 1; i >= 0; i--) {
		char c = str[i];

		if (!isdigit(c) && c != '.') {
			str[i + 1] = '\0';
			break;
		}
	};

	return str;
}

static int elf_add_symbol(struct elf *elf, struct symbol *sym)
{
	struct list_head *entry;
	struct rb_node *pnode;
	struct symbol *iter;

	INIT_LIST_HEAD(&sym->pv_target);
	sym->alias = sym;

	sym->type = GELF_ST_TYPE(sym->sym.st_info);
	sym->bind = GELF_ST_BIND(sym->sym.st_info);

	if (is_file_sym(sym))
		elf->num_files++;

	sym->offset = sym->sym.st_value;
	sym->len = sym->sym.st_size;

	__sym_for_each(iter, &sym->sec->symbol_tree, sym->offset, sym->offset) {
		if (iter->offset == sym->offset && iter->type == sym->type &&
		    iter->len == sym->len)
			iter->alias = sym;
	}

	__sym_insert(sym, &sym->sec->symbol_tree);
	pnode = rb_prev(&sym->node);
	if (pnode)
		entry = &rb_entry(pnode, struct symbol, node)->list;
	else
		entry = &sym->sec->symbol_list;
	list_add(&sym->list, entry);

	list_add_tail(&sym->global_list, &elf->symbols);
	elf_hash_add(symbol, &sym->hash, sym->idx);
	elf_hash_add(symbol_name, &sym->name_hash, str_hash(sym->name));

	if (is_func_sym(sym) &&
	    (strstarts(sym->name, "__pfx_") ||
	     strstarts(sym->name, "__cfi_") ||
	     strstarts(sym->name, "__pi___pfx_") ||
	     strstarts(sym->name, "__pi___cfi_")))
		sym->prefix = 1;

	if (strstarts(sym->name, ".klp.sym"))
		sym->klp = 1;

	if (!sym->klp && is_func_sym(sym) && strstr(sym->name, ".cold"))
		sym->cold = 1;
	sym->pfunc = sym->cfunc = sym;

	sym->demangled_name = demangle_name(sym);
	if (!sym->demangled_name)
		return -1;

	return 0;
}

static int read_symbols(struct elf *elf)
{
	struct section *symtab, *symtab_shndx, *sec;
	struct symbol *sym, *pfunc;
	int symbols_nr, i;
	char *coldstr;
	Elf_Data *shndx_data = NULL;
	Elf32_Word shndx;

	symtab = find_section_by_name(elf, ".symtab");
	if (symtab) {
		symtab_shndx = find_section_by_name(elf, ".symtab_shndx");
		if (symtab_shndx)
			shndx_data = symtab_shndx->data;

		symbols_nr = sec_num_entries(symtab);
	} else {
		/*
		 * A missing symbol table is actually possible if it's an empty
		 * .o file. This can happen for thunk_64.o. Make sure to at
		 * least allocate the symbol hash tables so we can do symbol
		 * lookups without crashing.
		 */
		symbols_nr = 0;
	}

	if (!elf_alloc_hash(symbol, symbols_nr) ||
	    !elf_alloc_hash(symbol_name, symbols_nr))
		return -1;

	elf->symbol_data = calloc(symbols_nr, sizeof(*sym));
	if (!elf->symbol_data) {
		ERROR_GLIBC("calloc");
		return -1;
	}

	INIT_LIST_HEAD(&elf->symbols);

	for (i = 0; i < symbols_nr; i++) {
		sym = &elf->symbol_data[i];

		sym->idx = i;

		if (!gelf_getsymshndx(symtab->data, shndx_data, i, &sym->sym,
				      &shndx)) {
			ERROR_ELF("gelf_getsymshndx");
			return -1;
		}

		sym->name = elf_strptr(elf->elf, symtab->sh.sh_link,
				       sym->sym.st_name);
		if (!sym->name) {
			ERROR_ELF("elf_strptr");
			return -1;
		}

		if ((sym->sym.st_shndx > SHN_UNDEF &&
		     sym->sym.st_shndx < SHN_LORESERVE) ||
		    (shndx_data && sym->sym.st_shndx == SHN_XINDEX)) {
			if (sym->sym.st_shndx != SHN_XINDEX)
				shndx = sym->sym.st_shndx;

			sym->sec = find_section_by_index(elf, shndx);
			if (!sym->sec) {
				ERROR("couldn't find section for symbol %s", sym->name);
				return -1;
			}
			if (GELF_ST_TYPE(sym->sym.st_info) == STT_SECTION) {
				sym->name = sym->sec->name;
				sym->sec->sym = sym;
			}
		} else
			sym->sec = find_section_by_index(elf, 0);

		if (elf_add_symbol(elf, sym))
			return -1;
	}

	if (opts.stats) {
		printf("nr_symbols: %lu\n", (unsigned long)symbols_nr);
		printf("symbol_bits: %d\n", elf->symbol_bits);
	}

	/* Create parent/child links for any cold subfunctions */
	list_for_each_entry(sec, &elf->sections, list) {
		sec_for_each_sym(sec, sym) {
			char *pname;
			size_t pnamelen;

			if (!sym->cold)
				continue;

			coldstr = strstr(sym->name, ".cold");
			if (!coldstr) {
				ERROR("%s(): cold subfunction without \".cold\"?", sym->name);
				return -1;
			}

			pnamelen = coldstr - sym->name;
			pname = strndup(sym->name, pnamelen);
			if (!pname) {
				ERROR("%s(): failed to allocate memory", sym->name);
				return -1;
			}

			pfunc = find_symbol_by_name(elf, pname);
			free(pname);

			if (!pfunc) {
				ERROR("%s(): can't find parent function", sym->name);
				return -1;
			}

			sym->pfunc = pfunc;
			pfunc->cfunc = sym;

			/*
			 * Unfortunately, -fnoreorder-functions puts the child
			 * inside the parent.  Remove the overlap so we can
			 * have sane assumptions.
			 *
			 * Note that pfunc->len now no longer matches
			 * pfunc->sym.st_size.
			 */
			if (sym->sec == pfunc->sec &&
			    sym->offset >= pfunc->offset &&
			    sym->offset + sym->len == pfunc->offset + pfunc->len) {
				pfunc->len -= sym->len;
			}
		}
	}

	return 0;
}

static int mark_group_syms(struct elf *elf)
{
	struct section *symtab, *sec;
	struct symbol *sym;

	symtab = find_section_by_name(elf, ".symtab");
	if (!symtab) {
		ERROR("no .symtab");
		return -1;
	}

	for_each_sec(elf, sec) {
		if (sec->sh.sh_type == SHT_GROUP &&
		    sec->sh.sh_link == symtab->idx) {
			sym = find_symbol_by_index(elf, sec->sh.sh_info);
			if (!sym) {
				ERROR("%s: can't find SHT_GROUP signature symbol",
				      sec->name);
				return -1;
			}

			sym->group_sec = sec;
		}
	}

	return 0;
}

/*
 * @sym's idx has changed.  Update the relocs which reference it.
 */
static int elf_update_sym_relocs(struct elf *elf, struct symbol *sym)
{
	struct reloc *reloc;

	for (reloc = sym->relocs; reloc; reloc = sym_next_reloc(reloc))
		set_reloc_sym(elf, reloc, reloc->sym->idx);

	return 0;
}

/*
 * The libelf API is terrible; gelf_update_sym*() takes a data block relative
 * index value, *NOT* the symbol index. As such, iterate the data blocks and
 * adjust index until it fits.
 *
 * If no data block is found, allow adding a new data block provided the index
 * is only one past the end.
 */
static int elf_update_symbol(struct elf *elf, struct section *symtab,
			     struct section *symtab_shndx, struct symbol *sym)
{
	Elf32_Word shndx;
	Elf_Data *symtab_data = NULL, *shndx_data = NULL;
	Elf64_Xword entsize = symtab->sh.sh_entsize;
	int max_idx, idx = sym->idx;
	Elf_Scn *s, *t = NULL;
	bool is_special_shndx = sym->sym.st_shndx >= SHN_LORESERVE &&
				sym->sym.st_shndx != SHN_XINDEX;

	shndx = is_special_shndx ? sym->sym.st_shndx : sym->sec->idx;

	s = elf_getscn(elf->elf, symtab->idx);
	if (!s) {
		ERROR_ELF("elf_getscn");
		return -1;
	}

	if (symtab_shndx) {
		t = elf_getscn(elf->elf, symtab_shndx->idx);
		if (!t) {
			ERROR_ELF("elf_getscn");
			return -1;
		}
	}

	for (;;) {
		/* get next data descriptor for the relevant sections */
		symtab_data = elf_getdata(s, symtab_data);
		if (t)
			shndx_data = elf_getdata(t, shndx_data);

		/* end-of-list */
		if (!symtab_data) {
			/*
			 * Over-allocate to avoid O(n^2) symbol creation
			 * behaviour.  The down side is that libelf doesn't
			 * like this; see elf_truncate_section() for the fixup.
			 */
			int num = max(1U, sym->idx/3);
			void *buf;

			if (idx) {
				/* we don't do holes in symbol tables */
				ERROR("index out of range");
				return -1;
			}

			/* if @idx == 0, it's the next contiguous entry, create it */
			symtab_data = elf_newdata(s);
			if (t)
				shndx_data = elf_newdata(t);

			buf = calloc(num, entsize);
			if (!buf) {
				ERROR_GLIBC("calloc");
				return -1;
			}

			symtab_data->d_buf = buf;
			symtab_data->d_size = num * entsize;
			symtab_data->d_align = 1;
			symtab_data->d_type = ELF_T_SYM;

			mark_sec_changed(elf, symtab, true);
			symtab->truncate = true;

			if (t) {
				buf = calloc(num, sizeof(Elf32_Word));
				if (!buf) {
					ERROR_GLIBC("calloc");
					return -1;
				}

				shndx_data->d_buf = buf;
				shndx_data->d_size = num * sizeof(Elf32_Word);
				shndx_data->d_align = sizeof(Elf32_Word);
				shndx_data->d_type = ELF_T_WORD;

				mark_sec_changed(elf, symtab_shndx, true);
				symtab_shndx->truncate = true;
			}

			break;
		}

		/* empty blocks should not happen */
		if (!symtab_data->d_size) {
			ERROR("zero size data");
			return -1;
		}

		/* is this the right block? */
		max_idx = symtab_data->d_size / entsize;
		if (idx < max_idx)
			break;

		/* adjust index and try again */
		idx -= max_idx;
	}

	/* something went side-ways */
	if (idx < 0) {
		ERROR("negative index");
		return -1;
	}

	/* setup extended section index magic and write the symbol */
	if (shndx < SHN_LORESERVE || is_special_shndx) {
		sym->sym.st_shndx = shndx;
		if (!shndx_data)
			shndx = 0;
	} else {
		sym->sym.st_shndx = SHN_XINDEX;
		if (!shndx_data) {
			ERROR("no .symtab_shndx");
			return -1;
		}
	}

	if (!gelf_update_symshndx(symtab_data, shndx_data, idx, &sym->sym, shndx)) {
		ERROR_ELF("gelf_update_symshndx");
		return -1;
	}

	return 0;
}

struct symbol *elf_create_symbol(struct elf *elf, const char *name,
				 struct section *sec, unsigned int bind,
				 unsigned int type, unsigned long offset,
				 size_t size)
{
	struct section *symtab, *symtab_shndx;
	Elf32_Word first_non_local, new_idx;
	struct symbol *old, *sym;

	sym = calloc(1, sizeof(*sym));
	if (!sym) {
		ERROR_GLIBC("calloc");
		return NULL;
	}

	sym->name = strdup(name);
	if (!sym->name) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	if (type != STT_SECTION) {
		sym->sym.st_name = elf_add_string(elf, NULL, sym->name);
		if (sym->sym.st_name == -1)
			return NULL;
	}

	if (sec) {
		sym->sec = sec;
	} else {
		sym->sec = find_section_by_index(elf, 0);
		if (!sym->sec) {
			ERROR("no NULL section");
			return NULL;
		}
	}

	sym->sym.st_info  = GELF_ST_INFO(bind, type);
	sym->sym.st_value = offset;
	sym->sym.st_size  = size;

	symtab = find_section_by_name(elf, ".symtab");
	if (!symtab) {
		ERROR("no .symtab");
		return NULL;
	}

	symtab_shndx = find_section_by_name(elf, ".symtab_shndx");

	new_idx = sec_num_entries(symtab);

	if (bind != STB_LOCAL)
		goto non_local;

	/*
	 * Move the first global symbol, as per sh_info, into a new, higher
	 * symbol index. This frees up a spot for a new local symbol.
	 */
	first_non_local = symtab->sh.sh_info;
	old = find_symbol_by_index(elf, first_non_local);
	if (old) {

		elf_hash_del(symbol, &old->hash, old->idx);
		elf_hash_add(symbol, &old->hash, new_idx);
		old->idx = new_idx;

		if (elf_update_symbol(elf, symtab, symtab_shndx, old)) {
			ERROR("elf_update_symbol move");
			return NULL;
		}

		if (elf_update_sym_relocs(elf, old))
			return NULL;

		if (old->group_sec) {
			old->group_sec->sh.sh_info = new_idx;
			mark_sec_changed(elf, old->group_sec, true);
		}

		new_idx = first_non_local;
	}

	/*
	 * Either way, we will add a LOCAL symbol.
	 */
	symtab->sh.sh_info += 1;

non_local:
	sym->idx = new_idx;
	if (sym->idx && elf_update_symbol(elf, symtab, symtab_shndx, sym))
		return NULL;

	symtab->sh.sh_size += symtab->sh.sh_entsize;
	mark_sec_changed(elf, symtab, true);

	if (symtab_shndx) {
		symtab_shndx->sh.sh_size += sizeof(Elf32_Word);
		mark_sec_changed(elf, symtab_shndx, true);
	}

	if (elf_add_symbol(elf, sym))
		return NULL;

	return sym;
}

struct symbol *elf_create_section_symbol(struct elf *elf, struct section *sec)
{
	struct symbol *sym = calloc(1, sizeof(*sym));

	sym = elf_create_symbol(elf, sec->name, sec, STB_LOCAL, STT_SECTION, 0, 0);
	if (!sym)
		return NULL;

	sec->sym = sym;

	return sym;
}

struct reloc *elf_init_reloc(struct elf *elf, struct section *rsec,
			     unsigned int reloc_idx, unsigned long offset,
			     struct symbol *sym, s64 addend, unsigned int type)
{
	struct reloc *reloc, empty = { 0 };

	if (reloc_idx >= sec_num_entries(rsec)) {
		ERROR("%s: bad reloc_idx %u for %s with %d relocs",
		      __func__, reloc_idx, rsec->name, sec_num_entries(rsec));
		return NULL;
	}

	reloc = &rsec->relocs[reloc_idx];

	if (memcmp(reloc, &empty, sizeof(empty))) {
		ERROR("%s: %s: reloc %d already initialized!",
		      __func__, rsec->name, reloc_idx);
		return NULL;
	}

	reloc->sec = rsec;
	reloc->sym = sym;

	set_reloc_offset(elf, reloc, offset);
	set_reloc_sym(elf, reloc, sym->idx);
	set_reloc_type(elf, reloc, type);
	set_reloc_addend(elf, reloc, addend);

	elf_hash_add(reloc, &reloc->hash, reloc_hash(reloc));
	set_sym_next_reloc(reloc, sym->relocs);
	sym->relocs = reloc;

	return reloc;
}

struct reloc *elf_init_reloc_text_sym(struct elf *elf, struct section *sec,
				      unsigned long offset,
				      unsigned int reloc_idx,
				      struct section *insn_sec,
				      unsigned long insn_off)
{
	struct symbol *sym = insn_sec->sym;
	s64 addend = insn_off;

	if (!is_text_sec(insn_sec)) {
		ERROR("bad call to %s() for data symbol %s", __func__, sym->name);
		return NULL;
	}

	if (!sym) {
		/*
		 * Due to how weak functions work, we must use section based
		 * relocations. Symbol based relocations would result in the
		 * weak and non-weak function annotations being overlaid on the
		 * non-weak function after linking.
		 */
		sym = elf_create_section_symbol(elf, insn_sec);
		if (!sym)
			return NULL;
	}

	return elf_init_reloc(elf, sec->rsec, reloc_idx, offset, sym, addend,
			      elf_text_rela_type(elf));
}

struct reloc *elf_init_reloc_data_sym(struct elf *elf, struct section *sec,
				      unsigned long offset,
				      unsigned int reloc_idx,
				      struct symbol *sym,
				      s64 addend)
{
	if (is_text_sec(sec)) {
		ERROR("bad call to %s() for text symbol %s", __func__, sym->name);
		return NULL;
	}

	return elf_init_reloc(elf, sec->rsec, reloc_idx, offset, sym, addend,
			      elf_data_rela_type(elf));
}

static int read_relocs(struct elf *elf)
{
	unsigned long nr_reloc, max_reloc = 0;
	struct section *rsec;
	struct reloc *reloc;
	unsigned int symndx;
	struct symbol *sym;
	int i;

	if (!elf_alloc_hash(reloc, elf->num_relocs))
		return -1;

	list_for_each_entry(rsec, &elf->sections, list) {
		if (!is_reloc_sec(rsec))
			continue;

		rsec->base = find_section_by_index(elf, rsec->sh.sh_info);
		if (!rsec->base) {
			ERROR("can't find base section for reloc section %s", rsec->name);
			return -1;
		}

		rsec->base->rsec = rsec;

		/* nr_alloc_relocs=0: libelf owns d_buf */
		rsec->nr_alloc_relocs = 0;

		rsec->relocs = calloc(sec_num_entries(rsec), sizeof(*reloc));
		if (!rsec->relocs) {
			ERROR_GLIBC("calloc");
			return -1;
		}

		nr_reloc = 0;
		for (i = 0; i < sec_num_entries(rsec); i++) {
			reloc = &rsec->relocs[i];

			reloc->sec = rsec;
			symndx = reloc_sym(reloc);
			reloc->sym = sym = find_symbol_by_index(elf, symndx);
			if (!reloc->sym) {
				ERROR("can't find reloc entry symbol %d for %s", symndx, rsec->name);
				return -1;
			}

			elf_hash_add(reloc, &reloc->hash, reloc_hash(reloc));
			set_sym_next_reloc(reloc, sym->relocs);
			sym->relocs = reloc;

			nr_reloc++;
		}
		max_reloc = max(max_reloc, nr_reloc);
	}

	if (opts.stats) {
		printf("max_reloc: %lu\n", max_reloc);
		printf("num_relocs: %lu\n", elf->num_relocs);
		printf("reloc_bits: %d\n", elf->reloc_bits);
	}

	return 0;
}

struct elf *elf_open_read(const char *name, int flags)
{
	struct elf *elf;
	Elf_Cmd cmd;

	elf_version(EV_CURRENT);

	elf = malloc(sizeof(*elf));
	if (!elf) {
		ERROR_GLIBC("malloc");
		return NULL;
	}
	memset(elf, 0, sizeof(*elf));

	INIT_LIST_HEAD(&elf->sections);

	elf->fd = open(name, flags);
	if (elf->fd == -1) {
		fprintf(stderr, "objtool: Can't open '%s': %s\n",
			name, strerror(errno));
		goto err;
	}

	elf->name = strdup(name);
	if (!elf->name) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	if ((flags & O_ACCMODE) == O_RDONLY)
		cmd = ELF_C_READ_MMAP;
	else if ((flags & O_ACCMODE) == O_RDWR)
		cmd = ELF_C_RDWR;
	else /* O_WRONLY */
		cmd = ELF_C_WRITE;

	elf->elf = elf_begin(elf->fd, cmd, NULL);
	if (!elf->elf) {
		ERROR_ELF("elf_begin");
		goto err;
	}

	if (!gelf_getehdr(elf->elf, &elf->ehdr)) {
		ERROR_ELF("gelf_getehdr");
		goto err;
	}

	if (read_sections(elf))
		goto err;

	if (read_symbols(elf))
		goto err;

	if (mark_group_syms(elf))
		goto err;

	if (read_relocs(elf))
		goto err;

	return elf;

err:
	elf_close(elf);
	return NULL;
}

struct elf *elf_create_file(GElf_Ehdr *ehdr, const char *name)
{
	struct section *null, *symtab, *strtab, *shstrtab;
	char *dir, *base, *tmp_name;
	struct symbol *sym;
	struct elf *elf;

	elf_version(EV_CURRENT);

	elf = calloc(1, sizeof(*elf));
	if (!elf) {
		ERROR_GLIBC("calloc");
		return NULL;
	}

	INIT_LIST_HEAD(&elf->sections);

	dir = strdup(name);
	if (!dir) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	dir = dirname(dir);

	base = strdup(name);
	if (!base) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	base = basename(base);

	tmp_name = malloc(256);
	if (!tmp_name) {
		ERROR_GLIBC("malloc");
		return NULL;
	}

	snprintf(tmp_name, 256, "%s/%s.XXXXXX", dir, base);

	elf->fd = mkstemp(tmp_name);
	if (elf->fd == -1) {
		ERROR_GLIBC("can't create tmp file");
		exit(1);
	}

	elf->tmp_name = tmp_name;

	elf->name = strdup(name);
	if (!elf->name) {
		ERROR_GLIBC("strdup");
		return NULL;
	}

	elf->elf = elf_begin(elf->fd, ELF_C_WRITE, NULL);
	if (!elf->elf) {
		ERROR_ELF("elf_begin");
		return NULL;
	}

	if (!gelf_newehdr(elf->elf, ELFCLASS64)) {
		ERROR_ELF("gelf_newehdr");
		return NULL;
	}

	memcpy(&elf->ehdr, ehdr, sizeof(elf->ehdr));

	if (!gelf_update_ehdr(elf->elf, &elf->ehdr)) {
		ERROR_ELF("gelf_update_ehdr");
		return NULL;
	}

	INIT_LIST_HEAD(&elf->symbols);

	if (!elf_alloc_hash(section,		1000) ||
	    !elf_alloc_hash(section_name,	1000) ||
	    !elf_alloc_hash(symbol,		10000) ||
	    !elf_alloc_hash(symbol_name,	10000) ||
	    !elf_alloc_hash(reloc,		100000))
		return NULL;

	null		= elf_create_section(elf, NULL, 0, 0, SHT_NULL, 0, 0);
	shstrtab	= elf_create_section(elf, NULL, 0, 0, SHT_STRTAB, 1, 0);
	strtab		= elf_create_section(elf, NULL, 0, 0, SHT_STRTAB, 1, 0);

	if (!null || !shstrtab || !strtab)
		return NULL;

	null->name	= "";
	shstrtab->name	= ".shstrtab";
	strtab->name	= ".strtab";

	null->sh.sh_name	= elf_add_string(elf, shstrtab, null->name);
	shstrtab->sh.sh_name	= elf_add_string(elf, shstrtab, shstrtab->name);
	strtab->sh.sh_name	= elf_add_string(elf, shstrtab, strtab->name);

	if (null->sh.sh_name == -1 || shstrtab->sh.sh_name == -1 || strtab->sh.sh_name == -1)
		return NULL;

	elf_hash_add(section_name, &null->name_hash,		str_hash(null->name));
	elf_hash_add(section_name, &strtab->name_hash,		str_hash(strtab->name));
	elf_hash_add(section_name, &shstrtab->name_hash,	str_hash(shstrtab->name));

	if (elf_add_string(elf, strtab, "") == -1)
		return NULL;

	symtab = elf_create_section(elf, ".symtab", 0x18, 0x18, SHT_SYMTAB, 0x8, 0);
	if (!symtab)
		return NULL;

	symtab->sh.sh_link = strtab->idx;
	symtab->sh.sh_info = 1;

	elf->ehdr.e_shstrndx = shstrtab->idx;
	if (!gelf_update_ehdr(elf->elf, &elf->ehdr)) {
		ERROR_ELF("gelf_update_ehdr");
		return NULL;
	}

	sym = calloc(1, sizeof(*sym));
	if (!sym) {
		ERROR_GLIBC("calloc");
		return NULL;
	}

	sym->name = "";
	sym->sec = null;
	elf_add_symbol(elf, sym);

	return elf;
}

unsigned int elf_add_string(struct elf *elf, struct section *strtab, const char *str)
{
	unsigned int offset;

	if (!strtab)
		strtab = find_section_by_name(elf, ".strtab");
	if (!strtab) {
		ERROR("can't find .strtab section");
		return -1;
	}

	if (!strtab->sh.sh_addralign) {
		ERROR("'%s': invalid sh_addralign", strtab->name);
		return -1;
	}

	offset = ALIGN_UP(strtab->sh.sh_size, strtab->sh.sh_addralign);

	if (!elf_add_data(elf, strtab, str, strlen(str) + 1))
		return -1;

	return offset;
}

void *elf_add_data(struct elf *elf, struct section *sec, const void *data, size_t size)
{
	unsigned long offset;
	Elf_Scn *s;

	if (!sec->sh.sh_addralign) {
		ERROR("'%s': invalid sh_addralign", sec->name);
		return NULL;
	}

	s = elf_getscn(elf->elf, sec->idx);
	if (!s) {
		ERROR_ELF("elf_getscn");
		return NULL;
	}

	sec->data = elf_newdata(s);
	if (!sec->data) {
		ERROR_ELF("elf_newdata");
		return NULL;
	}

	sec->data->d_buf = calloc(1, size);
	if (!sec->data->d_buf) {
		ERROR_GLIBC("calloc");
		return NULL;
	}

	if (data)
		memcpy(sec->data->d_buf, data, size);

	sec->data->d_size = size;
	sec->data->d_align = 1;

	offset = ALIGN_UP(sec->sh.sh_size, sec->sh.sh_addralign);
	sec->sh.sh_size = offset + size;

	mark_sec_changed(elf, sec, true);

	return sec->data->d_buf;
}

struct section *elf_create_section(struct elf *elf, const char *name,
				   size_t size, size_t entsize,
				   unsigned int type, unsigned int align,
				   unsigned int flags)
{
	struct section *sec, *shstrtab;
	Elf_Scn *s;

	if (name && find_section_by_name(elf, name)) {
		ERROR("section '%s' already exists", name);
		return NULL;
	}

	sec = calloc(1, sizeof(*sec));
	if (!sec) {
		ERROR_GLIBC("calloc");
		return NULL;
	}

	INIT_LIST_HEAD(&sec->symbol_list);

	/* don't actually create the section, just the data structures */
	if (type == SHT_NULL)
		goto add;

	s = elf_newscn(elf->elf);
	if (!s) {
		ERROR_ELF("elf_newscn");
		return NULL;
	}

	sec->idx = elf_ndxscn(s);

	if (size) {
		sec->data = elf_newdata(s);
		if (!sec->data) {
			ERROR_ELF("elf_newdata");
			return NULL;
		}

		sec->data->d_size = size;
		sec->data->d_align = 1;

		sec->data->d_buf = calloc(1, size);
		if (!sec->data->d_buf) {
			ERROR_GLIBC("calloc");
			return NULL;
		}
	}

	if (!gelf_getshdr(s, &sec->sh)) {
		ERROR_ELF("gelf_getshdr");
		return NULL;
	}

	sec->sh.sh_size = size;
	sec->sh.sh_entsize = entsize;
	sec->sh.sh_type = type;
	sec->sh.sh_addralign = align;
	sec->sh.sh_flags = flags;

	if (name) {
		sec->name = strdup(name);
		if (!sec->name) {
			ERROR("strdup");
			return NULL;
		}

		/* Add section name to .shstrtab (or .strtab for Clang) */
		shstrtab = find_section_by_name(elf, ".shstrtab");
		if (!shstrtab) {
			shstrtab = find_section_by_name(elf, ".strtab");
			if (!shstrtab) {
				ERROR("can't find .shstrtab or .strtab");
				return NULL;
			}
		}
		sec->sh.sh_name = elf_add_string(elf, shstrtab, sec->name);
		if (sec->sh.sh_name == -1)
			return NULL;

		elf_hash_add(section_name, &sec->name_hash, str_hash(sec->name));
	}

add:
	list_add_tail(&sec->list, &elf->sections);
	elf_hash_add(section, &sec->hash, sec->idx);

	mark_sec_changed(elf, sec, true);

	return sec;
}

static int elf_alloc_reloc(struct elf *elf, struct section *rsec)
{
	struct reloc *old_relocs, *old_relocs_end, *new_relocs;
	unsigned int nr_relocs_old = sec_num_entries(rsec);
	unsigned int nr_relocs_new = nr_relocs_old + 1;
	unsigned long nr_alloc;
	struct symbol *sym;

	if (!rsec->data) {
		rsec->data = elf_newdata(elf_getscn(elf->elf, rsec->idx));
		if (!rsec->data) {
			ERROR_ELF("elf_newdata");
			return -1;
		}

		rsec->data->d_align = 1;
		rsec->data->d_type = ELF_T_RELA;
		rsec->data->d_buf = NULL;
	}

	rsec->data->d_size = nr_relocs_new * elf_rela_size(elf);
	rsec->sh.sh_size   = rsec->data->d_size;

	nr_alloc = MAX(64, ALIGN_UP_POW2(nr_relocs_new));
	if (nr_alloc <= rsec->nr_alloc_relocs)
		return 0;

	if (rsec->data->d_buf && !rsec->nr_alloc_relocs) {
		void *orig_buf = rsec->data->d_buf;

		/*
		 * The original d_buf is owned by libelf so it can't be
		 * realloced.
		 */
		rsec->data->d_buf = malloc(nr_alloc * elf_rela_size(elf));
		if (!rsec->data->d_buf) {
			ERROR_GLIBC("malloc");
			return -1;
		}
		memcpy(rsec->data->d_buf, orig_buf,
		       nr_relocs_old * elf_rela_size(elf));
	} else {
		rsec->data->d_buf = realloc(rsec->data->d_buf,
					    nr_alloc * elf_rela_size(elf));
		if (!rsec->data->d_buf) {
			ERROR_GLIBC("realloc");
			return -1;
		}
	}

	rsec->nr_alloc_relocs = nr_alloc;

	old_relocs = rsec->relocs;
	new_relocs = calloc(nr_alloc, sizeof(struct reloc));
	if (!new_relocs) {
		ERROR_GLIBC("calloc");
		return -1;
	}

	if (!old_relocs)
		goto done;

	/*
	 * The struct reloc's address has changed.  Update all the symbols and
	 * relocs which reference it.
	 */

	old_relocs_end = &old_relocs[nr_relocs_old];
	for_each_sym(elf, sym) {
		struct reloc *reloc;

		reloc = sym->relocs;
		if (!reloc)
			continue;

		if (reloc >= old_relocs && reloc < old_relocs_end)
			sym->relocs = &new_relocs[reloc - old_relocs];

		while (1) {
			struct reloc *next_reloc = sym_next_reloc(reloc);

			if (!next_reloc)
				break;

			if (next_reloc >= old_relocs && next_reloc < old_relocs_end)
				set_sym_next_reloc(reloc, &new_relocs[next_reloc - old_relocs]);

			reloc = next_reloc;
		}
	}

	memcpy(new_relocs, old_relocs, nr_relocs_old * sizeof(struct reloc));

	for (int i = 0; i < nr_relocs_old; i++) {
		struct reloc *old = &old_relocs[i];
		struct reloc *new = &new_relocs[i];
		u32 key = reloc_hash(old);

		elf_hash_del(reloc, &old->hash, key);
		elf_hash_add(reloc, &new->hash, key);
	}

	free(old_relocs);
done:
	rsec->relocs = new_relocs;
	return 0;
}

struct section *elf_create_rela_section(struct elf *elf, struct section *sec,
					unsigned int nr_relocs)
{
	struct section *rsec;
	char *rsec_name;

	rsec_name = malloc(strlen(sec->name) + strlen(".rela") + 1);
	if (!rsec_name) {
		ERROR_GLIBC("malloc");
		return NULL;
	}
	strcpy(rsec_name, ".rela");
	strcat(rsec_name, sec->name);

	rsec = elf_create_section(elf, rsec_name, nr_relocs * elf_rela_size(elf),
				  elf_rela_size(elf), SHT_RELA, elf_addr_size(elf),
				  SHF_INFO_LINK);
	free(rsec_name);
	if (!rsec)
		return NULL;

	if (nr_relocs) {
		rsec->data->d_type = ELF_T_RELA;

		rsec->nr_alloc_relocs = nr_relocs;
		rsec->relocs = calloc(nr_relocs, sizeof(struct reloc));
		if (!rsec->relocs) {
			ERROR_GLIBC("calloc");
			return NULL;
		}
	}

	rsec->sh.sh_link = find_section_by_name(elf, ".symtab")->idx;
	rsec->sh.sh_info = sec->idx;

	sec->rsec = rsec;
	rsec->base = sec;

	return rsec;
}

struct reloc *elf_create_reloc(struct elf *elf, struct section *sec,
			       unsigned long offset,
			       struct symbol *sym, s64 addend,
			       unsigned int type)
{
	struct section *rsec = sec->rsec;

	if (!rsec) {
		rsec = elf_create_rela_section(elf, sec, 0);
		if (!rsec)
			return NULL;
	}

	if (find_reloc_by_dest(elf, sec, offset)) {
		ERROR_FUNC(sec, offset, "duplicate reloc");
		return NULL;
	}

	if (elf_alloc_reloc(elf, rsec))
		return NULL;

	mark_sec_changed(elf, rsec, true);

	return elf_init_reloc(elf, rsec, sec_num_entries(rsec) - 1, offset, sym,
			      addend, type);
}

struct section *elf_create_section_pair(struct elf *elf, const char *name,
					size_t entsize, unsigned int nr,
					unsigned int nr_relocs)
{
	struct section *sec;

	sec = elf_create_section(elf, name, nr * entsize, entsize,
				 SHT_PROGBITS, 1, SHF_ALLOC);
	if (!sec)
		return NULL;

	if (!elf_create_rela_section(elf, sec, nr_relocs))
		return NULL;

	return sec;
}

int elf_write_insn(struct elf *elf, struct section *sec,
		   unsigned long offset, unsigned int len,
		   const char *insn)
{
	Elf_Data *data = sec->data;

	if (data->d_type != ELF_T_BYTE || data->d_off) {
		ERROR("write to unexpected data for section: %s", sec->name);
		return -1;
	}

	memcpy(data->d_buf + offset, insn, len);

	mark_sec_changed(elf, sec, true);

	return 0;
}

/*
 * When Elf_Scn::sh_size is smaller than the combined Elf_Data::d_size
 * do you:
 *
 *   A) adhere to the section header and truncate the data, or
 *   B) ignore the section header and write out all the data you've got?
 *
 * Yes, libelf sucks and we need to manually truncate if we over-allocate data.
 */
static int elf_truncate_section(struct elf *elf, struct section *sec)
{
	u64 size = sec_size(sec);
	bool truncated = false;
	Elf_Data *data = NULL;
	Elf_Scn *s;

	s = elf_getscn(elf->elf, sec->idx);
	if (!s) {
		ERROR_ELF("elf_getscn");
		return -1;
	}

	for (;;) {
		/* get next data descriptor for the relevant section */
		data = elf_getdata(s, data);
		if (!data) {
			if (size) {
				ERROR("end of section data but non-zero size left\n");
				return -1;
			}
			return 0;
		}

		if (truncated) {
			/* when we remove symbols */
			ERROR("truncated; but more data\n");
			return -1;
		}

		if (!data->d_size) {
			ERROR("zero size data");
			return -1;
		}

		if (data->d_size > size) {
			truncated = true;
			data->d_size = size;
		}

		size -= data->d_size;
	}
}

int elf_write(struct elf *elf)
{
	struct section *sec;
	Elf_Scn *s;

	/* Update changed relocation sections and section headers: */
	list_for_each_entry(sec, &elf->sections, list) {
		if (sec->truncate && elf_truncate_section(elf, sec))
			return -1;

		if (sec_changed(sec)) {
			s = elf_getscn(elf->elf, sec->idx);
			if (!s) {
				ERROR_ELF("elf_getscn");
				return -1;
			}

			/* Note this also flags the section dirty */
			if (!gelf_update_shdr(s, &sec->sh)) {
				ERROR_ELF("gelf_update_shdr");
				return -1;
			}

			mark_sec_changed(elf, sec, false);
		}
	}

	/* Make sure the new section header entries get updated properly. */
	elf_flagelf(elf->elf, ELF_C_SET, ELF_F_DIRTY);

	/* Write all changes to the file. */
	if (elf_update(elf->elf, ELF_C_WRITE) < 0) {
		ERROR_ELF("elf_update");
		return -1;
	}

	elf->changed = false;

	return 0;
}

int elf_close(struct elf *elf)
{
	if (elf->elf)
		elf_end(elf->elf);

	if (elf->fd > 0)
		close(elf->fd);

	if (elf->tmp_name && rename(elf->tmp_name, elf->name))
		return -1;

	/*
	 * NOTE: All remaining allocations are leaked on purpose.  Objtool is
	 * about to exit anyway.
	 */
	return 0;
}
