/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
 */

#ifndef _OBJTOOL_ELF_H
#define _OBJTOOL_ELF_H

#include <stdio.h>
#include <gelf.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/hashtable.h>
#include <linux/rbtree.h>
#include <linux/jhash.h>

#include <objtool/endianness.h>
#include <objtool/checksum_types.h>
#include <arch/elf.h>

#define SEC_NAME_LEN		1024
#define SYM_NAME_LEN		512

#define bswap_if_needed(elf, val) __bswap_if_needed(&elf->ehdr, val)

#ifdef LIBELF_USE_DEPRECATED
# define elf_getshdrnum    elf_getshnum
# define elf_getshdrstrndx elf_getshstrndx
#endif

/*
 * Fallback for systems without this "read, mmaping if possible" cmd.
 */
#ifndef ELF_C_READ_MMAP
#define ELF_C_READ_MMAP ELF_C_READ
#endif

struct elf_hash_node {
	struct elf_hash_node *next;
};

struct section {
	struct list_head list;
	struct elf_hash_node hash;
	struct elf_hash_node name_hash;
	GElf_Shdr sh;
	struct rb_root_cached symbol_tree;
	struct list_head symbol_list;
	struct section *base, *rsec;
	struct symbol *sym;
	Elf_Data *data;
	const char *name;
	int idx;
	bool _changed, text, rodata, noinstr, init, truncate;
	struct reloc *relocs;
	unsigned long nr_alloc_relocs;
	struct section *twin;
};

struct symbol {
	struct list_head list;
	struct list_head global_list;
	struct rb_node node;
	struct elf_hash_node hash;
	struct elf_hash_node name_hash;
	GElf_Sym sym;
	struct section *sec;
	const char *name, *demangled_name;
	unsigned int idx, len;
	unsigned long offset;
	unsigned long __subtree_last;
	struct symbol *pfunc, *cfunc, *alias;
	unsigned char bind, type;
	u8 uaccess_safe      : 1;
	u8 static_call_tramp : 1;
	u8 retpoline_thunk   : 1;
	u8 return_thunk      : 1;
	u8 fentry            : 1;
	u8 profiling_func    : 1;
	u8 warned	     : 1;
	u8 embedded_insn     : 1;
	u8 local_label       : 1;
	u8 frame_pointer     : 1;
	u8 ignore	     : 1;
	u8 nocfi             : 1;
	u8 cold		     : 1;
	u8 prefix	     : 1;
	u8 debug_checksum    : 1;
	u8 changed	     : 1;
	u8 included	     : 1;
	u8 klp		     : 1;
	struct list_head pv_target;
	struct reloc *relocs;
	struct section *group_sec;
	struct checksum csum;
	struct symbol *twin, *clone;
};

struct reloc {
	struct elf_hash_node hash;
	struct section *sec;
	struct symbol *sym;
	unsigned long _sym_next_reloc;
};

struct elf {
	Elf *elf;
	GElf_Ehdr ehdr;
	int fd;
	bool changed;
	const char *name, *tmp_name;
	unsigned int num_files;
	struct list_head sections;
	struct list_head symbols;
	unsigned long num_relocs;

	int symbol_bits;
	int symbol_name_bits;
	int section_bits;
	int section_name_bits;
	int reloc_bits;

	struct elf_hash_node **symbol_hash;
	struct elf_hash_node **symbol_name_hash;
	struct elf_hash_node **section_hash;
	struct elf_hash_node **section_name_hash;
	struct elf_hash_node **reloc_hash;

	struct section *section_data;
	struct symbol *symbol_data;
};

struct elf *elf_open_read(const char *name, int flags);
struct elf *elf_create_file(GElf_Ehdr *ehdr, const char *name);

struct section *elf_create_section(struct elf *elf, const char *name,
				   size_t size, size_t entsize,
				   unsigned int type, unsigned int align,
				   unsigned int flags);
struct section *elf_create_section_pair(struct elf *elf, const char *name,
					size_t entsize, unsigned int nr,
					unsigned int reloc_nr);

struct section *elf_create_rela_section(struct elf *elf, struct section *sec,
					unsigned int reloc_nr);

struct symbol *elf_create_symbol(struct elf *elf, const char *name,
				 struct section *sec, unsigned int bind,
				 unsigned int type, unsigned long offset,
				 size_t size);
struct symbol *elf_create_section_symbol(struct elf *elf, struct section *sec);

void *elf_add_data(struct elf *elf, struct section *sec, const void *data,
		   size_t size);

unsigned int elf_add_string(struct elf *elf, struct section *strtab, const char *str);

struct reloc *elf_create_reloc(struct elf *elf, struct section *sec,
			       unsigned long offset, struct symbol *sym,
			       s64 addend, unsigned int type);

struct reloc *elf_init_reloc(struct elf *elf, struct section *rsec,
			     unsigned int reloc_idx, unsigned long offset,
			     struct symbol *sym, s64 addend, unsigned int type);

struct reloc *elf_init_reloc_text_sym(struct elf *elf, struct section *sec,
				      unsigned long offset,
				      unsigned int reloc_idx,
				      struct section *insn_sec,
				      unsigned long insn_off);

struct reloc *elf_init_reloc_data_sym(struct elf *elf, struct section *sec,
				      unsigned long offset,
				      unsigned int reloc_idx,
				      struct symbol *sym,
				      s64 addend);

int elf_write_insn(struct elf *elf, struct section *sec, unsigned long offset,
		   unsigned int len, const char *insn);

int elf_write(struct elf *elf);
int elf_close(struct elf *elf);

struct section *find_section_by_name(const struct elf *elf, const char *name);
struct symbol *find_func_by_offset(struct section *sec, unsigned long offset);
struct symbol *find_symbol_by_offset(struct section *sec, unsigned long offset);
struct symbol *find_symbol_by_name(const struct elf *elf, const char *name);
struct symbol *find_global_symbol_by_name(const struct elf *elf, const char *name);
struct symbol *find_symbol_containing(const struct section *sec, unsigned long offset);
int find_symbol_hole_containing(const struct section *sec, unsigned long offset);
struct reloc *find_reloc_by_dest(const struct elf *elf, struct section *sec, unsigned long offset);
struct reloc *find_reloc_by_dest_range(const struct elf *elf, struct section *sec,
				     unsigned long offset, unsigned int len);
struct symbol *find_func_containing(struct section *sec, unsigned long offset);

/*
 * Try to see if it's a whole archive (vmlinux.o or module).
 *
 * Note this will miss the case where a module only has one source file.
 */
static inline bool has_multiple_files(struct elf *elf)
{
	return elf->num_files > 1;
}

static inline size_t elf_addr_size(struct elf *elf)
{
	return elf->ehdr.e_ident[EI_CLASS] == ELFCLASS32 ? 4 : 8;
}

static inline size_t elf_rela_size(struct elf *elf)
{
	return elf_addr_size(elf) == 4 ? sizeof(Elf32_Rela) : sizeof(Elf64_Rela);
}

static inline unsigned int elf_data_rela_type(struct elf *elf)
{
	return elf_addr_size(elf) == 4 ? R_DATA32 : R_DATA64;
}

static inline unsigned int elf_text_rela_type(struct elf *elf)
{
	return elf_addr_size(elf) == 4 ? R_TEXT32 : R_TEXT64;
}

static inline bool is_undef_sym(struct symbol *sym)
{
	return !sym->sec->idx;
}

static inline bool is_null_sym(struct symbol *sym)
{
	return !sym->idx;
}

static inline bool is_sec_sym(struct symbol *sym)
{
	return sym->type == STT_SECTION;
}

static inline bool is_object_sym(struct symbol *sym)
{
	return sym->type == STT_OBJECT;
}

static inline bool is_func_sym(struct symbol *sym)
{
	return sym->type == STT_FUNC;
}

static inline bool is_file_sym(struct symbol *sym)
{
	return sym->type == STT_FILE;
}

static inline bool is_notype_sym(struct symbol *sym)
{
	return sym->type == STT_NOTYPE;
}

static inline bool is_global_sym(struct symbol *sym)
{
	return sym->bind == STB_GLOBAL;
}

static inline bool is_weak_sym(struct symbol *sym)
{
	return sym->bind == STB_WEAK;
}

static inline bool is_local_sym(struct symbol *sym)
{
	return sym->bind == STB_LOCAL;
}

static inline bool is_prefix_func(struct symbol *sym)
{
	return sym->prefix;
}

static inline bool is_reloc_sec(struct section *sec)
{
	return sec->sh.sh_type == SHT_RELA || sec->sh.sh_type == SHT_REL;
}

static inline bool is_string_sec(struct section *sec)
{
	return sec->sh.sh_flags & SHF_STRINGS;
}

static inline bool is_text_sec(struct section *sec)
{
	return sec->sh.sh_flags & SHF_EXECINSTR;
}

static inline bool sec_changed(struct section *sec)
{
	return sec->_changed;
}

static inline void mark_sec_changed(struct elf *elf, struct section *sec,
				    bool changed)
{
	sec->_changed = changed;
	elf->changed |= changed;
}

static inline unsigned int sec_num_entries(struct section *sec)
{
	return sec->sh.sh_size / sec->sh.sh_entsize;
}

static inline unsigned int reloc_idx(struct reloc *reloc)
{
	return reloc - reloc->sec->relocs;
}

static inline void *reloc_rel(struct reloc *reloc)
{
	struct section *rsec = reloc->sec;

	return rsec->data->d_buf + (reloc_idx(reloc) * rsec->sh.sh_entsize);
}

static inline bool is_32bit_reloc(struct reloc *reloc)
{
	/*
	 * Elf32_Rel:   8 bytes
	 * Elf32_Rela: 12 bytes
	 * Elf64_Rel:  16 bytes
	 * Elf64_Rela: 24 bytes
	 */
	return reloc->sec->sh.sh_entsize < 16;
}

static inline unsigned long sec_size(struct section *sec)
{
	return sec->sh.sh_size;
}

#define __get_reloc_field(reloc, field)					\
({									\
	is_32bit_reloc(reloc) ?						\
		((Elf32_Rela *)reloc_rel(reloc))->field :		\
		((Elf64_Rela *)reloc_rel(reloc))->field;		\
})

#define __set_reloc_field(reloc, field, val)				\
({									\
	if (is_32bit_reloc(reloc))					\
		((Elf32_Rela *)reloc_rel(reloc))->field = val;		\
	else								\
		((Elf64_Rela *)reloc_rel(reloc))->field = val;		\
})

static inline u64 reloc_offset(struct reloc *reloc)
{
	return __get_reloc_field(reloc, r_offset);
}

static inline void set_reloc_offset(struct elf *elf, struct reloc *reloc, u64 offset)
{
	__set_reloc_field(reloc, r_offset, offset);
	mark_sec_changed(elf, reloc->sec, true);
}

static inline s64 reloc_addend(struct reloc *reloc)
{
	return __get_reloc_field(reloc, r_addend);
}

static inline void set_reloc_addend(struct elf *elf, struct reloc *reloc, s64 addend)
{
	__set_reloc_field(reloc, r_addend, addend);
	mark_sec_changed(elf, reloc->sec, true);
}


static inline unsigned int reloc_sym(struct reloc *reloc)
{
	u64 info = __get_reloc_field(reloc, r_info);

	return is_32bit_reloc(reloc) ?
		ELF32_R_SYM(info) :
		ELF64_R_SYM(info);
}

static inline unsigned int reloc_type(struct reloc *reloc)
{
	u64 info = __get_reloc_field(reloc, r_info);

	return is_32bit_reloc(reloc) ?
		ELF32_R_TYPE(info) :
		ELF64_R_TYPE(info);
}

static inline void set_reloc_sym(struct elf *elf, struct reloc *reloc, unsigned int sym)
{
	u64 info = is_32bit_reloc(reloc) ?
		ELF32_R_INFO(sym, reloc_type(reloc)) :
		ELF64_R_INFO(sym, reloc_type(reloc));

	__set_reloc_field(reloc, r_info, info);

	mark_sec_changed(elf, reloc->sec, true);
}
static inline void set_reloc_type(struct elf *elf, struct reloc *reloc, unsigned int type)
{
	u64 info = is_32bit_reloc(reloc) ?
		ELF32_R_INFO(reloc_sym(reloc), type) :
		ELF64_R_INFO(reloc_sym(reloc), type);

	__set_reloc_field(reloc, r_info, info);

	mark_sec_changed(elf, reloc->sec, true);
}

static inline unsigned int annotype(struct elf *elf, struct section *sec,
				    struct reloc *reloc)
{
	unsigned int type;

	type = *(u32 *)(sec->data->d_buf + (reloc_idx(reloc) * 8) + 4);
	return bswap_if_needed(elf, type);
}

#define RELOC_JUMP_TABLE_BIT 1UL

/* Does reloc mark the beginning of a jump table? */
static inline bool is_jump_table(struct reloc *reloc)
{
	return reloc->_sym_next_reloc & RELOC_JUMP_TABLE_BIT;
}

static inline void set_jump_table(struct reloc *reloc)
{
	reloc->_sym_next_reloc |= RELOC_JUMP_TABLE_BIT;
}

static inline struct reloc *sym_next_reloc(struct reloc *reloc)
{
	return (struct reloc *)(reloc->_sym_next_reloc & ~RELOC_JUMP_TABLE_BIT);
}

static inline void set_sym_next_reloc(struct reloc *reloc, struct reloc *next)
{
	unsigned long bit = reloc->_sym_next_reloc & RELOC_JUMP_TABLE_BIT;

	reloc->_sym_next_reloc = (unsigned long)next | bit;
}

#define for_each_sec(elf, sec)						\
	list_for_each_entry(sec, &elf->sections, list)

#define sec_for_each_sym(sec, sym)					\
	list_for_each_entry(sym, &sec->symbol_list, list)

#define sec_prev_sym(sym)						\
	sym->sec && sym->list.prev != &sym->sec->symbol_list ?		\
	list_prev_entry(sym, list) : NULL

#define for_each_sym(elf, sym)						\
	list_for_each_entry(sym, &elf->symbols, global_list)

#define for_each_sym_continue(elf, sym)					\
	list_for_each_entry_continue(sym, &elf->symbols, global_list)

#define rsec_next_reloc(rsec, reloc)					\
	reloc_idx(reloc) < sec_num_entries(rsec) - 1 ? reloc + 1 : NULL

#define for_each_reloc(rsec, reloc)					\
	for (reloc = rsec->relocs; reloc; reloc = rsec_next_reloc(rsec, reloc))

#define for_each_reloc_from(rsec, reloc)				\
	for (; reloc; reloc = rsec_next_reloc(rsec, reloc))

#define for_each_reloc_continue(rsec, reloc)				\
	for (reloc = rsec_next_reloc(rsec, reloc); reloc;		\
	     reloc = rsec_next_reloc(rsec, reloc))

#define sym_for_each_reloc(elf, sym, reloc)				\
	for (reloc = find_reloc_by_dest_range(elf, sym->sec,		\
					      sym->offset, sym->len);	\
	     reloc && reloc_offset(reloc) <  sym->offset + sym->len;	\
	     reloc = rsec_next_reloc(sym->sec->rsec, reloc))

static inline struct symbol *get_func_prefix(struct symbol *func)
{
	struct symbol *prev;

	if (!is_func_sym(func))
		return NULL;

	prev = sec_prev_sym(func);
	if (prev && is_prefix_func(prev))
		return prev;

	return NULL;
}

#define OFFSET_STRIDE_BITS	4
#define OFFSET_STRIDE		(1UL << OFFSET_STRIDE_BITS)
#define OFFSET_STRIDE_MASK	(~(OFFSET_STRIDE - 1))

#define for_offset_range(_offset, _start, _end)			\
	for (_offset = ((_start) & OFFSET_STRIDE_MASK);		\
	     _offset >= ((_start) & OFFSET_STRIDE_MASK) &&	\
	     _offset <= ((_end) & OFFSET_STRIDE_MASK);		\
	     _offset += OFFSET_STRIDE)

static inline u32 sec_offset_hash(struct section *sec, unsigned long offset)
{
	u32 ol, oh, idx = sec->idx;

	offset &= OFFSET_STRIDE_MASK;

	ol = offset;
	oh = (offset >> 16) >> 16;

	__jhash_mix(ol, oh, idx);

	return ol;
}

static inline u32 reloc_hash(struct reloc *reloc)
{
	return sec_offset_hash(reloc->sec, reloc_offset(reloc));
}

#endif /* _OBJTOOL_ELF_H */
