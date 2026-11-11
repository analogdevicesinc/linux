// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * Kernel API specification debugfs interface
 *
 * This provides a debugfs interface to expose kernel API specifications
 * at runtime, allowing tools and users to query the complete API specs.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/kernel_api_spec.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "internal.h"

/* Helper to print parameter flags */
static void print_param_flags(struct seq_file *m, u32 flags)
{
	seq_puts(m, "    flags: ");
	if ((flags & KAPI_PARAM_INOUT) == KAPI_PARAM_INOUT)
		seq_puts(m, "INOUT ");
	else if (flags & KAPI_PARAM_IN)
		seq_puts(m, "IN ");
	else if (flags & KAPI_PARAM_OUT)
		seq_puts(m, "OUT ");
	if (flags & KAPI_PARAM_OPTIONAL)
		seq_puts(m, "OPTIONAL ");
	if (flags & KAPI_PARAM_CONST)
		seq_puts(m, "CONST ");
	if (flags & KAPI_PARAM_USER)
		seq_puts(m, "USER ");
	if (flags & KAPI_PARAM_VOLATILE)
		seq_puts(m, "VOLATILE ");
	if (flags & KAPI_PARAM_DMA)
		seq_puts(m, "DMA ");
	if (flags & KAPI_PARAM_ALIGNED)
		seq_puts(m, "ALIGNED ");
	seq_puts(m, "\n");
}

/* Helper to print context flags */
static void print_context_flags(struct seq_file *m, u32 flags)
{
	seq_puts(m, "Context flags: ");
	if (flags & KAPI_CTX_PROCESS)
		seq_puts(m, "PROCESS ");
	if (flags & KAPI_CTX_HARDIRQ)
		seq_puts(m, "HARDIRQ ");
	if (flags & KAPI_CTX_SOFTIRQ)
		seq_puts(m, "SOFTIRQ ");
	if (flags & KAPI_CTX_NMI)
		seq_puts(m, "NMI ");
	if (flags & KAPI_CTX_SLEEPABLE)
		seq_puts(m, "SLEEPABLE ");
	if (flags & KAPI_CTX_ATOMIC)
		seq_puts(m, "ATOMIC ");
	if (flags & KAPI_CTX_PREEMPT_DISABLED)
		seq_puts(m, "PREEMPT_DISABLED ");
	if (flags & KAPI_CTX_IRQ_DISABLED)
		seq_puts(m, "IRQ_DISABLED ");
	seq_puts(m, "\n");
}

/*
 * Print a multi-line value with every line indented, so that no line of it
 * can be mistaken for a field of its own.
 */
static void print_block(struct seq_file *m, const char *text)
{
	while (*text) {
		size_t len = strcspn(text, "\n");

		if (len)
			seq_printf(m, "  %.*s", (int)len, text);
		seq_putc(m, '\n');
		text += len;
		if (*text == '\n')
			text++;
	}
}

/* Helper to print a list of signed values */
static void print_s64_list(struct seq_file *m, const char *label,
			   const s64 *vals, u32 count)
{
	u32 i;

	if (!vals || !count)
		return;

	seq_printf(m, "%s:", label);
	for (i = 0; i < count; i++)
		seq_printf(m, " %lld", vals[i]);
	seq_puts(m, "\n");
}

/* Helper to print which parameter carries a buffer's element count */
static void print_size_param(struct seq_file *m,
			     const struct kernel_api_spec *spec,
			     const struct kapi_param_spec *param)
{
	int idx = param->size_param_idx - 1;

	if (idx < 0)
		return;

	seq_printf(m, "      size_param: %s (index %d)\n",
		   idx < spec->param_count && idx < KAPI_MAX_PARAMS ?
		   spec->params[idx].name : "?", idx);
	if (param->size_multiplier)
		seq_printf(m, "      size_multiplier: %zu\n",
			   param->size_multiplier);
}

/* Show function for individual API spec */
static int kapi_spec_show(struct seq_file *m, void *v)
{
	struct kernel_api_spec *spec = m->private;
	int i;

	seq_puts(m, "Kernel API Specification\n");
	seq_puts(m, "========================\n\n");

	/* Basic info */
	seq_printf(m, "Name: %s\n", spec->name);
	seq_printf(m, "Version: %u\n", spec->version);
	seq_printf(m, "Description: %s\n", spec->description);
	if (spec->long_description && *spec->long_description) {
		seq_puts(m, "Long description:\n");
		print_block(m, spec->long_description);
	}

	/* Context */
	print_context_flags(m, spec->context_flags);
	seq_puts(m, "\n");

	/* Parameters */
	if (spec->param_count > 0) {
		seq_printf(m, "Parameters (%u):\n", spec->param_count);
		for (i = 0; i < spec->param_count && i < KAPI_MAX_PARAMS; i++) {
			struct kapi_param_spec *param = &spec->params[i];

			seq_printf(m, "  [%d] %s:\n", i, param->name);
			seq_printf(m, "    type: %s (%s)\n",
				   kapi_param_type_to_string(param->type), param->type_name);
			print_param_flags(m, param->flags);
			if (param->description && *param->description)
				seq_printf(m, "    description: %s\n", param->description);
			if (param->size > 0)
				seq_printf(m, "    size: %zu\n", param->size);
			if (param->alignment > 0)
				seq_printf(m, "    alignment: %zu\n", param->alignment);

			/* Print constraints if any */
			if (param->constraint_type != KAPI_CONSTRAINT_NONE ||
			    (param->constraints && *param->constraints)) {
				seq_puts(m, "    constraints:\n");
				switch (param->constraint_type) {
				case KAPI_CONSTRAINT_NONE:
					break;
				case KAPI_CONSTRAINT_RANGE:
					seq_puts(m, "      type: range\n");
					seq_printf(m, "      min: %lld\n", param->min_value);
					seq_printf(m, "      max: %lld\n", param->max_value);
					break;
				case KAPI_CONSTRAINT_MASK:
					seq_puts(m, "      type: mask\n");
					seq_printf(m, "      valid_bits: 0x%llx\n",
						   param->valid_mask);
					break;
				case KAPI_CONSTRAINT_ENUM:
					seq_puts(m, "      type: enum\n");
					seq_printf(m, "      count: %u\n", param->enum_count);
					print_s64_list(m, "      values",
						       param->enum_values,
						       param->enum_count);
					break;
				case KAPI_CONSTRAINT_USER_STRING:
					seq_puts(m, "      type: user_string\n");
					seq_printf(m, "      min_len: %lld\n", param->min_value);
					seq_printf(m, "      max_len: %lld\n", param->max_value);
					break;
				case KAPI_CONSTRAINT_USER_PATH:
					seq_puts(m, "      type: user_path\n");
					seq_puts(m, "      max_len: PATH_MAX (4096)\n");
					break;
				case KAPI_CONSTRAINT_USER_PTR:
					seq_puts(m, "      type: user_ptr\n");
					seq_printf(m, "      size: %zu bytes\n", param->size);
					print_size_param(m, spec, param);
					break;
				case KAPI_CONSTRAINT_BUFFER:
					seq_puts(m, "      type: buffer\n");
					print_size_param(m, spec, param);
					break;
				case KAPI_CONSTRAINT_ALIGNMENT:
					seq_puts(m, "      type: alignment\n");
					seq_printf(m, "      alignment: %zu\n", param->alignment);
					break;
				case KAPI_CONSTRAINT_POWER_OF_TWO:
					seq_puts(m, "      type: power_of_two\n");
					break;
				case KAPI_CONSTRAINT_PAGE_ALIGNED:
					seq_puts(m, "      type: page_aligned\n");
					break;
				case KAPI_CONSTRAINT_NONZERO:
					seq_puts(m, "      type: nonzero\n");
					break;
				case KAPI_CONSTRAINT_CUSTOM:
					seq_puts(m, "      type: custom\n");
					break;
				default:
					seq_printf(m, "      type: unknown (%d)\n",
						   param->constraint_type);
					break;
				}
				if (param->constraints && *param->constraints)
					seq_printf(m, "      description: %s\n",
						   param->constraints);
			}
			seq_puts(m, "\n");
		}
	}

	/* Return value */
	seq_puts(m, "Return value:\n");
	seq_printf(m, "  type: %s\n", spec->return_spec.type_name);
	if (spec->return_spec.description && *spec->return_spec.description)
		seq_printf(m, "  description: %s\n", spec->return_spec.description);

	switch (spec->return_spec.check_type) {
	case KAPI_RETURN_EXACT:
		seq_printf(m, "  success: == %lld\n", spec->return_spec.success_value);
		break;
	case KAPI_RETURN_RANGE:
		seq_printf(m, "  success: [%lld, %lld]\n",
			   spec->return_spec.success_min,
			   spec->return_spec.success_max);
		break;
	case KAPI_RETURN_FD:
		seq_puts(m, "  success: valid file descriptor (>= 0)\n");
		break;
	case KAPI_RETURN_ERROR_CHECK:
		seq_puts(m, "  success: error check\n");
		print_s64_list(m, "  error values",
			       spec->return_spec.error_values,
			       spec->return_spec.error_count);
		break;
	case KAPI_RETURN_CUSTOM:
		seq_puts(m, "  success: custom check\n");
		break;
	case KAPI_RETURN_NO_RETURN:
		seq_puts(m, "  success: does not return\n");
		break;
	default:
		break;
	}
	seq_puts(m, "\n");

	/* Errors */
	if (spec->error_count > 0) {
		seq_printf(m, "Errors (%u):\n", spec->error_count);
		for (i = 0; i < spec->error_count && i < KAPI_MAX_ERRORS; i++) {
			struct kapi_error_spec *err = &spec->errors[i];

			seq_printf(m, "  %s (%d): %s\n",
				   err->name, err->error_code, err->description);
			if (err->condition && *err->condition)
				seq_printf(m, "    condition: %s\n", err->condition);
		}
		seq_puts(m, "\n");
	}

	/* Locks */
	if (spec->lock_count > 0) {
		seq_printf(m, "Locks (%u):\n", spec->lock_count);
		for (i = 0; i < spec->lock_count && i < KAPI_MAX_LOCKS; i++) {
			struct kapi_lock_spec *lock = &spec->locks[i];

			seq_printf(m, "  %s (%s): %s\n", lock->lock_name,
				   kapi_lock_type_to_string(lock->lock_type),
				   lock->description);
			seq_printf(m, "    scope: %s\n",
				   kapi_lock_scope_to_string(lock->scope));
		}
		seq_puts(m, "\n");
	}

	/* Constraints */
	if (spec->constraint_count > 0) {
		seq_printf(m, "Additional constraints (%u):\n", spec->constraint_count);
		for (i = 0; i < spec->constraint_count && i < KAPI_MAX_CONSTRAINTS; i++) {
			struct kapi_constraint_spec *cons = &spec->constraints[i];

			seq_printf(m, "  - %s", cons->name);
			if (cons->description && *cons->description)
				seq_printf(m, ": %s", cons->description);
			seq_puts(m, "\n");
			if (cons->expression && *cons->expression)
				seq_printf(m, "    expression: %s\n", cons->expression);
		}
		seq_puts(m, "\n");
	}

	/* Signals */
	if (spec->signal_count > 0) {
		seq_printf(m, "Signal handling (%u):\n", spec->signal_count);
		for (i = 0; i < spec->signal_count && i < KAPI_MAX_SIGNALS; i++) {
			struct kapi_signal_spec *sig = &spec->signals[i];

			seq_printf(m, "  %s (%d):\n", sig->signal_name, sig->signal_num);
			seq_puts(m, "    direction: ");
			if (sig->direction & KAPI_SIGNAL_SEND)
				seq_puts(m, "send ");
			if (sig->direction & KAPI_SIGNAL_RECEIVE)
				seq_puts(m, "receive ");
			if (sig->direction & KAPI_SIGNAL_HANDLE)
				seq_puts(m, "handle ");
			if (sig->direction & KAPI_SIGNAL_BLOCK)
				seq_puts(m, "block ");
			if (sig->direction & KAPI_SIGNAL_IGNORE)
				seq_puts(m, "ignore ");
			seq_puts(m, "\n");
			seq_puts(m, "    action: ");
			switch (sig->action) {
			case KAPI_SIGNAL_ACTION_DEFAULT:
				seq_puts(m, "default");
				break;
			case KAPI_SIGNAL_ACTION_TERMINATE:
				seq_puts(m, "terminate");
				break;
			case KAPI_SIGNAL_ACTION_COREDUMP:
				seq_puts(m, "coredump");
				break;
			case KAPI_SIGNAL_ACTION_STOP:
				seq_puts(m, "stop");
				break;
			case KAPI_SIGNAL_ACTION_CONTINUE:
				seq_puts(m, "continue");
				break;
			case KAPI_SIGNAL_ACTION_CUSTOM:
				seq_puts(m, "custom");
				break;
			case KAPI_SIGNAL_ACTION_RETURN:
				seq_puts(m, "return");
				break;
			case KAPI_SIGNAL_ACTION_RESTART:
				seq_puts(m, "restart");
				break;
			case KAPI_SIGNAL_ACTION_QUEUE:
				seq_puts(m, "queue");
				break;
			case KAPI_SIGNAL_ACTION_DISCARD:
				seq_puts(m, "discard");
				break;
			case KAPI_SIGNAL_ACTION_TRANSFORM:
				seq_puts(m, "transform");
				break;
			default:
				seq_puts(m, "unknown");
				break;
			}
			seq_puts(m, "\n");
			if (sig->description && *sig->description)
				seq_printf(m, "    description: %s\n", sig->description);
		}
		seq_puts(m, "\n");
	}

	/* Side effects */
	if (spec->side_effect_count > 0) {
		seq_printf(m, "Side effects (%u):\n", spec->side_effect_count);
		for (i = 0; i < spec->side_effect_count && i < KAPI_MAX_SIDE_EFFECTS; i++) {
			const struct kapi_side_effect *eff = &spec->side_effects[i];

			seq_printf(m, "  - %s", eff->target);
			if (eff->description && *eff->description)
				seq_printf(m, ": %s", eff->description);
			if (eff->reversible)
				seq_puts(m, " (reversible)");
			seq_puts(m, "\n");
		}
		seq_puts(m, "\n");
	}

	/* State transitions */
	if (spec->state_trans_count > 0) {
		seq_printf(m, "State transitions (%u):\n", spec->state_trans_count);
		for (i = 0; i < spec->state_trans_count && i < KAPI_MAX_STATE_TRANS; i++) {
			const struct kapi_state_transition *trans = &spec->state_transitions[i];

			seq_printf(m, "  %s: %s -> %s\n", trans->object,
				   trans->from_state, trans->to_state);
			if (trans->description && *trans->description)
				seq_printf(m, "    %s\n", trans->description);
		}
		seq_puts(m, "\n");
	}

	/* Capabilities */
	if (spec->capability_count > 0) {
		seq_printf(m, "Capabilities (%u):\n", spec->capability_count);
		for (i = 0; i < spec->capability_count && i < KAPI_MAX_CAPABILITIES; i++) {
			const struct kapi_capability_spec *cap = &spec->capabilities[i];

			seq_printf(m, "  %s (%d):\n", cap->cap_name,
				   cap->capability);
			if (cap->allows && *cap->allows)
				seq_printf(m, "    allows: %s\n", cap->allows);
			if (cap->without_cap && *cap->without_cap)
				seq_printf(m, "    without: %s\n", cap->without_cap);
		}
		seq_puts(m, "\n");
	}

	/* Additional info */
	if (spec->examples && *spec->examples) {
		seq_puts(m, "Examples:\n");
		print_block(m, spec->examples);
		seq_putc(m, '\n');
	}
	if (spec->notes && *spec->notes) {
		seq_puts(m, "Notes:\n");
		print_block(m, spec->notes);
		seq_putc(m, '\n');
	}

	return 0;
}

static int kapi_spec_open(struct inode *inode, struct file *file)
{
	return single_open(file, kapi_spec_show, inode->i_private);
}

static const struct file_operations kapi_spec_fops = {
	.open = kapi_spec_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * JSON view of a single spec.  kapi_export_json() writes straight into the
 * seq_file buffer; when the output does not fit, flagging an overflow makes
 * seq_file retry with a larger buffer, so no fixed size limit applies.
 */
static int kapi_spec_json_show(struct seq_file *m, void *v)
{
	const struct kernel_api_spec *spec = m->private;
	size_t size;
	char *buf;
	int ret;

	if (!spec)
		return -EINVAL;

	size = seq_get_buf(m, &buf);
	ret = size ? kapi_export_json(spec, buf, size) : -E2BIG;
	if (ret == -E2BIG) {
		seq_commit(m, -1);
		return 0;
	}
	if (ret < 0)
		return ret;

	seq_commit(m, ret);
	return 0;
}

static int kapi_spec_json_open(struct inode *inode, struct file *file)
{
	return single_open(file, kapi_spec_json_show, inode->i_private);
}

static const struct file_operations kapi_spec_json_fops = {
	.open = kapi_spec_json_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * Show all available API specs.
 *
 * Note: This only iterates the static .kapi_specs section. Specs registered
 * dynamically via kapi_register_spec() are not included in this listing
 * or in the per-spec debugfs files.
 */
static int kapi_list_show(struct seq_file *m, void *v)
{
	const struct kernel_api_spec * const *pp;
	int count = 0;

	seq_puts(m, "Available Kernel API Specifications\n");
	seq_puts(m, "===================================\n\n");

	for (pp = __start_kapi_specs; pp < __stop_kapi_specs; pp++) {
		const struct kernel_api_spec *spec = *pp;

		if (!spec)
			continue;
		seq_printf(m, "%s - %s\n", spec->name, spec->description);
		count++;
	}

	seq_printf(m, "\nTotal: %d specifications\n", count);
	return 0;
}

static int kapi_list_open(struct inode *inode, struct file *file)
{
	return single_open(file, kapi_list_show, NULL);
}

static const struct file_operations kapi_list_fops = {
	.open = kapi_list_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init kapi_debugfs_init(void)
{
	const struct kernel_api_spec * const *pp;
	struct dentry *root, *spec_dir, *json_dir;

	root = debugfs_create_dir("kapi", NULL);
	debugfs_create_file("list", 0444, root, NULL, &kapi_list_fops);
	spec_dir = debugfs_create_dir("specs", root);
	json_dir = debugfs_create_dir("specs-json", root);

	for (pp = __start_kapi_specs; pp < __stop_kapi_specs; pp++) {
		const struct kernel_api_spec *spec = *pp;

		if (!spec || !spec->name)
			continue;
		debugfs_create_file(spec->name, 0444, spec_dir,
				    (void *)spec, &kapi_spec_fops);
		debugfs_create_file(spec->name, 0444, json_dir,
				    (void *)spec, &kapi_spec_json_fops);
	}

	return 0;
}

/* Initialize as part of kernel, not as a module */
fs_initcall(kapi_debugfs_init);
