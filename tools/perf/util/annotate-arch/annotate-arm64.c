// SPDX-License-Identifier: GPL-2.0
#include <linux/compiler.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/zalloc.h>
#include <regex.h>
#include "../annotate.h"
#include "../disasm.h"

struct arch_arm64 {
	struct arch arch;
	regex_t call_insn;
	regex_t jump_insn;
};

static bool arm64__is_reg(const char *op)
{
	if (!op || !*op)
		return false;

	/*
	 * General-purpose registers: x0-x30, w0-w30.
	 * Check for 'x' or 'w' prefix followed by a numeric index.
	 */
	if ((op[0] == 'x' || op[0] == 'w') && isdigit(op[1]))
		return true;

	/* Special-purpose registers: sp, wzr, xzr. */
	if (!strncmp(op, "sp", 2) || !strncmp(op, "xzr", 3) ||
	    !strncmp(op, "wzr", 3))
		return true;

	/* TODO: Support more registers. */
	return false;
}

static bool arm64__check_multi_regs(const struct arch *arch, const char *op)
{
	const char *p = op;
	int reg_count = 0;

	while (p && *p) {
		p = skip_spaces(p);
		if (*p == arch->objdump.memory_ref_char)
			p++;

		if (arm64__is_reg(p))
			reg_count++;

		if (reg_count >= 2)
			return true;

		/* Move to next operand after comma */
		p = strchr(p, ',');
		if (p)
			p++;
	}

	return false;
}

/*
 * Duplicate @insn, stripping the comment and trailing whitespace.
 * Returns a newly allocated string which the caller must free(),
 * or NULL on allocation failure or if @insn is NULL.
 */
static char *rstrip_space_and_comment(const char *insn, char comment_char)
{
	const char *end, *comment;
	size_t len;
	char *result;

	if (insn == NULL)
		return NULL;

	comment = strchr(insn, comment_char);
	if (comment != NULL)
		end = comment;
	else
		end = insn + strlen(insn);

	while (end > insn && isspace(end[-1]))
		--end;

	len = end - insn;
	result = malloc(len + 1);
	if (result == NULL)
		return NULL;

	memcpy(result, insn, len);
	result[len] = '\0';

	return result;
}

static int arm64_mov__parse(const struct arch *arch,
			    struct ins_operands *ops,
			    struct map_symbol *ms __maybe_unused,
			    struct disasm_line *dl __maybe_unused)
{
	char *s = strchr(ops->raw, ','), *source, *endptr;

	if (s == NULL)
		return -1;

	/* Parse target */
	*s = '\0';
	ops->target.raw = strdup(ops->raw);
	*s = ',';

	if (ops->target.raw == NULL)
		return -1;

	ops->target.multi_regs = arm64__check_multi_regs(arch, ops->target.raw);

	/* Parse source, stripping comment if present */
	source = skip_spaces(++s);
	ops->source.raw = rstrip_space_and_comment(source, arch->objdump.comment_char);
	if (ops->source.raw == NULL) {
		zfree(&ops->target.raw);
		return -1;
	}

	ops->source.multi_regs = arm64__check_multi_regs(arch, ops->source.raw);

	/*
	 * Parse 'addr <symbol>' from source (if any).
	 *
	 * A raw hex string may be a scalar immediate, not an address.
	 * Only validate 'source.addr' if accompanied by a '<symbol>' tag,
	 * otherwise reset it to 0 to avoid false positive address tracking.
	 */
	ops->source.addr = strtoull(ops->source.raw, &endptr, 16);
	if (endptr != ops->source.raw) {
		s = strchr(endptr, '<');
		if (s == NULL) {
			ops->source.addr = 0;
			return 0;
		}
		endptr = strrchr(s + 1, '>');
		if (endptr == NULL) {
			ops->source.addr = 0;
			return 0;
		}

		*endptr = '\0';
		ops->source.name = strdup(s + 1);
		*endptr = '>';
		if (ops->source.name == NULL) {
			ops->source.addr = 0;
			return 0;
		}
	}

	return 0;
}

static int arm64_mov__scnprintf(const struct ins *ins, char *bf, size_t size,
				struct ins_operands *ops, int max_ins_name)
{
	return scnprintf(bf, size, "%-*s %s, %s", max_ins_name, ins->name,
			 ops->target.raw, ops->source.name ?: ops->source.raw);
}

static const struct ins_ops arm64_mov_ops = {
	.parse	   = arm64_mov__parse,
	.scnprintf = arm64_mov__scnprintf,
};

static const struct ins_ops *arm64__associate_instruction_ops(struct arch *arch, const char *name)
{
	struct arch_arm64 *arm = container_of(arch, struct arch_arm64, arch);
	const struct ins_ops *ops;
	regmatch_t match[2];

	if (!regexec(&arm->jump_insn, name, 2, match, 0))
		ops = &jump_ops;
	else if (!regexec(&arm->call_insn, name, 2, match, 0))
		ops = &call_ops;
	else if (!strcmp(name, "ret"))
		ops = &ret_ops;
	else
		ops = &arm64_mov_ops;

	arch__associate_ins_ops(arch, name, ops);
	return ops;
}

const struct arch *arch__new_arm64(const struct e_machine_and_e_flags *id,
				   const char *cpuid __maybe_unused)
{
	int err;
	struct arch_arm64 *arm = zalloc(sizeof(*arm));
	struct arch *arch;

	if (!arm)
		return NULL;

	arch = &arm->arch;
	arch->name = "arm64";
	arch->id = *id;
	arch->objdump.comment_char	  = '/';
	arch->objdump.skip_functions_char = '+';
	arch->objdump.memory_ref_char	  = '[';
	arch->associate_instruction_ops   = arm64__associate_instruction_ops;

	/* bl, blr */
	err = regcomp(&arm->call_insn, "^blr?$", REG_EXTENDED);
	if (err)
		goto out_free_arm;

	/* b, b.cond, br, cbz/cbnz, tbz/tbnz */
	err = regcomp(&arm->jump_insn, "^[ct]?br?\\.?(cc|cs|eq|ge|gt|hi|hs|le|lo|ls|lt|mi|ne|pl|vc|vs)?n?z?$",
		      REG_EXTENDED);
	if (err)
		goto out_free_call;

	return arch;

out_free_call:
	regfree(&arm->call_insn);
out_free_arm:
	free(arm);
	errno = SYMBOL_ANNOTATE_ERRNO__ARCH_INIT_REGEXP;
	return NULL;
}
