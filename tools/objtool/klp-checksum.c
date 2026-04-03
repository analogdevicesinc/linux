// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>
#include <subcmd/parse-options.h>

#include <objtool/arch.h>
#include <objtool/builtin.h>
#include <objtool/check.h>
#include <objtool/elf.h>
#include <objtool/klp.h>
#include <objtool/objtool.h>
#include <objtool/warn.h>
#include <objtool/checksum.h>

static int checksum_debug_init(struct objtool_file *file)
{
	char *dup, *s;

	if (!opts.debug_checksum)
		return 0;

	dup = strdup(opts.debug_checksum);
	if (!dup) {
		ERROR_GLIBC("strdup");
		return -1;
	}

	s = dup;
	while (*s) {
		bool found = false;
		struct symbol *sym;
		char *comma;

		comma = strchr(s, ',');
		if (comma)
			*comma = '\0';

		for_each_sym_by_name(file->elf, s, sym) {
			if (!is_func_sym(sym))
				continue;
			sym->debug_checksum = 1;
			found = true;
		}

		if (!found)
			WARN("--debug-checksum: can't find '%s'", s);

		if (!comma)
			break;

		s = comma + 1;
	}

	free(dup);
	return 0;
}

static void checksum_update_insn(struct objtool_file *file, struct symbol *func,
				 struct instruction *insn)
{
	struct reloc *reloc = insn_reloc(file, insn);
	struct alternative *alt;
	unsigned long offset;
	struct symbol *sym;
	static bool in_alt;

	if (insn->fake)
		return;

	checksum_update(func, insn, insn->sec->data->d_buf + insn->offset, insn->len);

	if (!reloc) {
		struct symbol *call_dest = insn_call_dest(insn);

		if (call_dest)
			checksum_update(func, insn, call_dest->demangled_name,
					strlen(call_dest->demangled_name));
		goto alts;
	}

	sym = reloc->sym;
	offset = arch_insn_adjusted_addend(insn, reloc);

	if (is_string_sec(sym->sec)) {
		char *str;

		str = sym->sec->data->d_buf + sym->offset + offset;
		checksum_update(func, insn, str, strlen(str));
		goto alts;
	}

	if (is_sec_sym(sym)) {
		sym = find_symbol_containing(reloc->sym->sec, offset);
		if (!sym)
			goto alts;

		offset -= sym->offset;
	}

	checksum_update(func, insn, sym->demangled_name, strlen(sym->demangled_name));
	checksum_update(func, insn, &offset, sizeof(offset));

alts:
	for (alt = insn->alts; alt; alt = alt->next) {
		struct alt_group *alt_group = alt->insn->alt_group;

		/* Prevent __ex_table recursion, e.g. LOAD_SEGMENT() */
		if (in_alt)
			break;
		in_alt = true;

		checksum_update(func, insn, &alt->type, sizeof(alt->type));

		if (alt_group && alt_group->orig_group) {
			struct instruction *alt_insn;

			checksum_update(func, insn, &alt_group->feature, sizeof(alt_group->feature));

			for (alt_insn = alt->insn; alt_insn; alt_insn = next_insn_same_sec(file, alt_insn)) {
				checksum_update_insn(file, func, alt_insn);
				if (!alt_group->last_insn || alt_insn == alt_group->last_insn)
					break;
			}
		} else {
			checksum_update_insn(file, func, alt->insn);
		}

		in_alt = false;
	}
}

int calculate_checksums(struct objtool_file *file)
{
	struct instruction *insn;
	struct symbol *func;

	if (checksum_debug_init(file))
		return -1;

	for_each_sym(file->elf, func) {
		/*
		 * Skip cold subfunctions and aliases: they share the
		 * parent's checksum via func_for_each_insn() which
		 * follows func->cfunc into the cold subfunction.
		 */
		if (!is_func_sym(func) || is_cold_func(func) ||
		    is_alias_sym(func) || !func->len)
			continue;

		checksum_init(func);

		func_for_each_insn(file, func, insn)
			checksum_update_insn(file, func, insn);

		checksum_finish(func);
	}
	return 0;
}

int create_sym_checksum_section(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *sym;
	unsigned int idx = 0;
	struct sym_checksum *checksum;
	size_t entsize = sizeof(struct sym_checksum);

	sec = find_section_by_name(file->elf, ".discard.sym_checksum");
	if (sec) {
		if (!opts.dryrun)
			WARN("file already has .discard.sym_checksum section, skipping");

		return 0;
	}

	for_each_sym(file->elf, sym)
		if (sym->csum.checksum)
			idx++;

	sec = elf_create_section_pair(file->elf, ".discard.sym_checksum", entsize,
				      idx, idx);
	if (!sec)
		return -1;

	idx = 0;
	for_each_sym(file->elf, sym) {
		if (!sym->csum.checksum)
			continue;

		if (!elf_init_reloc(file->elf, sec->rsec, idx, idx * entsize,
				    sym, 0, R_TEXT64))
			return -1;

		checksum = (struct sym_checksum *)sec->data->d_buf + idx;
		checksum->addr = 0; /* reloc */
		checksum->checksum = sym->csum.checksum;

		mark_sec_changed(file->elf, sec, true);

		idx++;
	}

	return 0;
}

static const char * const klp_checksum_usage[] = {
	"objtool klp checksum [<options>] file.o",
	NULL,
};

int cmd_klp_checksum(int argc, const char **argv)
{
	struct objtool_file *file;
	int ret;

	const struct option options[] = {
		OPT_STRING(0,	"debug-checksum", &opts.debug_checksum,	"funcs", "enable checksum debug output"),
		OPT_BOOLEAN(0,	"dry-run", &opts.dryrun, "don't write modifications"),
		OPT_END(),
	};

	argc = parse_options(argc, argv, options, klp_checksum_usage, 0);
	if (argc != 1)
		usage_with_options(klp_checksum_usage, options);

	opts.checksum = true;

	objname = argv[0];

	file = objtool_open_read(objname);
	if (!file)
		return 1;

	ret = decode_file(file);
	if (ret)
		goto out;

	ret = calculate_checksums(file);
	if (ret)
		goto out;

	ret = create_sym_checksum_section(file);

out:
	free_insns(file);

	if (ret)
		return ret;

	if (!opts.dryrun && file->elf->changed && elf_write(file->elf))
		return 1;

	return elf_close(file->elf);
}
