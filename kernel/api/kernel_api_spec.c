// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * kernel_api_spec.c - Kernel API Specification Framework Implementation
 *
 * Provides runtime support for kernel API specifications including validation,
 * export to various formats, and querying capabilities.
 */

#define pr_fmt(fmt) "kapi: " fmt

#include <linux/kernel.h>
#include <linux/kernel_api_spec.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/export.h>
#include <linux/preempt.h>
#include <linux/hardirq.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/uaccess.h>
#include <linux/limits.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/ratelimit.h>

#include "internal.h"

/* Dynamic API registration */
static LIST_HEAD(dynamic_api_specs);
static DEFINE_MUTEX(api_spec_mutex);

struct dynamic_api_spec {
	struct list_head list;
	const struct kernel_api_spec *spec;
};

/*
 * __kapi_find_spec_locked - Internal lookup, caller must hold api_spec_mutex
 */
static const struct kernel_api_spec *__kapi_find_spec_locked(const char *name)
{
	const struct kernel_api_spec * const *pp;
	struct dynamic_api_spec *dyn_spec;

	for (pp = __start_kapi_specs; pp < __stop_kapi_specs; pp++) {
		const struct kernel_api_spec *spec = *pp;

		if (spec && spec->name && strcmp(spec->name, name) == 0)
			return spec;
	}

	list_for_each_entry(dyn_spec, &dynamic_api_specs, list) {
		if (dyn_spec->spec->name &&
		    strcmp(dyn_spec->spec->name, name) == 0)
			return dyn_spec->spec;
	}

	return NULL;
}

/**
 * kapi_get_spec - Get API specification by name
 * @name: Function name to look up
 *
 * Return: Pointer to the API specification, or NULL if not found. The
 * pointer stays valid for specifications in the ``.kapi_specs`` ELF section
 * (built-in, statically defined). A dynamically registered spec stays valid
 * only until kapi_unregister_spec() is called for it, so the caller must
 * serialize against unregistration.
 *
 * Context: May sleep. Do not call under spinlock or in IRQ context.
 */
const struct kernel_api_spec *kapi_get_spec(const char *name)
{
	const struct kernel_api_spec *spec;

	if (!name)
		return NULL;

	mutex_lock(&api_spec_mutex);
	spec = __kapi_find_spec_locked(name);
	mutex_unlock(&api_spec_mutex);

	return spec;
}
EXPORT_SYMBOL_GPL(kapi_get_spec);

/**
 * kapi_register_spec - Register a dynamic API specification
 * @spec: API specification to register
 *
 * Return: 0 on success, negative error code on failure
 */
int kapi_register_spec(const struct kernel_api_spec *spec)
{
	struct dynamic_api_spec *dyn_spec;
	int ret = 0;

	if (!spec || !spec->name || !spec->name[0])
		return -EINVAL;

	dyn_spec = kzalloc_obj(*dyn_spec, GFP_KERNEL);
	if (!dyn_spec)
		return -ENOMEM;

	dyn_spec->spec = spec;

	mutex_lock(&api_spec_mutex);

	/* Check if already exists while holding lock to prevent races */
	if (__kapi_find_spec_locked(spec->name)) {
		ret = -EEXIST;
		kfree(dyn_spec);
	} else {
		list_add_tail(&dyn_spec->list, &dynamic_api_specs);
	}

	mutex_unlock(&api_spec_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(kapi_register_spec);

/**
 * kapi_unregister_spec - Unregister a dynamic API specification
 * @name: Name of API to unregister
 */
void kapi_unregister_spec(const char *name)
{
	struct dynamic_api_spec *dyn_spec, *tmp;

	if (!name)
		return;

	mutex_lock(&api_spec_mutex);
	list_for_each_entry_safe(dyn_spec, tmp, &dynamic_api_specs, list) {
		if (dyn_spec->spec->name &&
		    strcmp(dyn_spec->spec->name, name) == 0) {
			list_del(&dyn_spec->list);
			kfree(dyn_spec);
			break;
		}
	}
	mutex_unlock(&api_spec_mutex);
}
EXPORT_SYMBOL_GPL(kapi_unregister_spec);

/**
 * kapi_param_type_to_string - Convert parameter type to string
 * @type: Parameter type
 *
 * Return: String representation of type
 */
const char *kapi_param_type_to_string(enum kapi_param_type type)
{
	static const char * const type_names[] = {
		[KAPI_TYPE_VOID] = "void",
		[KAPI_TYPE_INT] = "int",
		[KAPI_TYPE_UINT] = "uint",
		[KAPI_TYPE_PTR] = "pointer",
		[KAPI_TYPE_STRUCT] = "struct",
		[KAPI_TYPE_UNION] = "union",
		[KAPI_TYPE_ENUM] = "enum",
		[KAPI_TYPE_FUNC_PTR] = "function_pointer",
		[KAPI_TYPE_ARRAY] = "array",
		[KAPI_TYPE_FD] = "file_descriptor",
		[KAPI_TYPE_USER_PTR] = "user_pointer",
		[KAPI_TYPE_PATH] = "pathname",
		[KAPI_TYPE_CUSTOM] = "custom",
	};

	if (type >= ARRAY_SIZE(type_names))
		return "unknown";

	return type_names[type];
}

/**
 * kapi_lock_type_to_string - Convert lock type to string
 * @type: Lock type
 *
 * Return: String representation of lock type
 */
const char *kapi_lock_type_to_string(enum kapi_lock_type type)
{
	static const char * const lock_names[] = {
		[KAPI_LOCK_NONE] = "none",
		[KAPI_LOCK_MUTEX] = "mutex",
		[KAPI_LOCK_SPINLOCK] = "spinlock",
		[KAPI_LOCK_RWLOCK] = "rwlock",
		[KAPI_LOCK_SEQLOCK] = "seqlock",
		[KAPI_LOCK_RCU] = "rcu",
		[KAPI_LOCK_SEMAPHORE] = "semaphore",
		[KAPI_LOCK_CUSTOM] = "custom",
	};

	if (type >= ARRAY_SIZE(lock_names))
		return "unknown";

	return lock_names[type];
}

/**
 * kapi_lock_scope_to_string - Convert lock scope to string
 * @scope: Lock scope
 *
 * Return: String representation of lock scope
 */
const char *kapi_lock_scope_to_string(enum kapi_lock_scope scope)
{
	static const char * const scope_names[] = {
		[KAPI_LOCK_INTERNAL] = "internal",
		[KAPI_LOCK_ACQUIRES] = "acquires",
		[KAPI_LOCK_RELEASES] = "releases",
		[KAPI_LOCK_CALLER_HELD] = "caller_held",
	};

	if (scope >= ARRAY_SIZE(scope_names))
		return "unknown";

	return scope_names[scope];
}

/**
 * return_check_type_to_string - Convert return check type to string
 * @type: Return check type
 *
 * Return: String representation of return check type
 */
static const char *return_check_type_to_string(enum kapi_return_check_type type)
{
	static const char * const check_names[] = {
		[KAPI_RETURN_EXACT] = "exact",
		[KAPI_RETURN_RANGE] = "range",
		[KAPI_RETURN_ERROR_CHECK] = "error_check",
		[KAPI_RETURN_FD] = "file_descriptor",
		[KAPI_RETURN_CUSTOM] = "custom",
		[KAPI_RETURN_NO_RETURN] = "no_return",
	};

	if (type >= ARRAY_SIZE(check_names))
		return "unknown";

	return check_names[type];
}

/**
 * capability_action_to_string - Convert capability action to string
 * @action: Capability action
 *
 * Return: String representation of capability action
 */
static const char *capability_action_to_string(enum kapi_capability_action action)
{
	static const char * const action_names[] = {
		[KAPI_CAP_BYPASS_CHECK] = "bypass_check",
		[KAPI_CAP_INCREASE_LIMIT] = "increase_limit",
		[KAPI_CAP_OVERRIDE_RESTRICTION] = "override_restriction",
		[KAPI_CAP_GRANT_PERMISSION] = "grant_permission",
		[KAPI_CAP_MODIFY_BEHAVIOR] = "modify_behavior",
		[KAPI_CAP_ACCESS_RESOURCE] = "access_resource",
		[KAPI_CAP_PERFORM_OPERATION] = "perform_operation",
	};

	if (action >= ARRAY_SIZE(action_names))
		return "unknown";

	return action_names[action];
}

/**
 * constraint_type_to_string - Convert constraint type to string
 * @type: Constraint type
 *
 * Return: String representation of constraint type
 */
static const char *constraint_type_to_string(enum kapi_constraint_type type)
{
	static const char * const constraint_names[] = {
		[KAPI_CONSTRAINT_NONE] = "none",
		[KAPI_CONSTRAINT_RANGE] = "range",
		[KAPI_CONSTRAINT_MASK] = "mask",
		[KAPI_CONSTRAINT_ENUM] = "enum",
		[KAPI_CONSTRAINT_ALIGNMENT] = "alignment",
		[KAPI_CONSTRAINT_POWER_OF_TWO] = "power_of_two",
		[KAPI_CONSTRAINT_PAGE_ALIGNED] = "page_aligned",
		[KAPI_CONSTRAINT_NONZERO] = "nonzero",
		[KAPI_CONSTRAINT_USER_STRING] = "user_string",
		[KAPI_CONSTRAINT_USER_PATH] = "user_path",
		[KAPI_CONSTRAINT_USER_PTR] = "user_ptr",
		[KAPI_CONSTRAINT_BUFFER] = "buffer",
		[KAPI_CONSTRAINT_CUSTOM] = "custom",
	};

	if (type >= ARRAY_SIZE(constraint_names))
		return "unknown";

	return constraint_names[type];
}

/*
 * kapi_json_escape - Write a JSON-escaped string into a buffer
 * @buf: Output buffer
 * @size: Remaining space in buffer
 * @str: Input string to escape
 *
 * Escapes backslash, double-quote, and control characters for JSON output.
 * Return: Number of bytes written (via scnprintf semantics)
 */
static int kapi_json_escape(char *buf, size_t size, const char *str)
{
	int ret = 0;
	const char *p;

	if (!str || size == 0)
		return 0;

	for (p = str; *p && ret < size - 1; p++) {
		switch (*p) {
		case '\\':
			ret += scnprintf(buf + ret, size - ret, "\\\\");
			break;
		case '"':
			ret += scnprintf(buf + ret, size - ret, "\\\"");
			break;
		case '\n':
			ret += scnprintf(buf + ret, size - ret, "\\n");
			break;
		case '\r':
			ret += scnprintf(buf + ret, size - ret, "\\r");
			break;
		case '\t':
			ret += scnprintf(buf + ret, size - ret, "\\t");
			break;
		default:
			if ((unsigned char)*p < 0x20) {
				ret += scnprintf(buf + ret, size - ret,
						 "\\u%04x", (unsigned char)*p);
			} else {
				ret += scnprintf(buf + ret, size - ret,
						 "%c", *p);
			}
			break;
		}
	}

	if (ret < size)
		buf[ret] = '\0';

	return ret;
}

static int kapi_json_str(char *buf, size_t size, const char *str)
{
	int ret = 0;

	ret += scnprintf(buf, size, "\"");
	ret += kapi_json_escape(buf + ret, size - ret, str);
	ret += scnprintf(buf + ret, size - ret, "\"");
	return ret;
}

static int kapi_json_s64_list(char *buf, size_t size, const s64 *vals, u32 count)
{
	int ret = scnprintf(buf, size, "[");
	u32 i;

	for (i = 0; vals && i < count; i++)
		ret += scnprintf(buf + ret, size - ret, "%s%lld",
				 i ? ", " : "", vals[i]);

	ret += scnprintf(buf + ret, size - ret, "]");
	return ret;
}

static int kapi_json_struct_spec(char *buf, size_t size,
				 const struct kapi_struct_spec *st)
{
	int ret;
	u32 i;

	ret = scnprintf(buf, size, "    {\n      \"name\": ");
	ret += kapi_json_str(buf + ret, size - ret, st->name);
	ret += scnprintf(buf + ret, size - ret,
		",\n      \"size\": %zu,\n      \"alignment\": %zu,\n      \"description\": ",
		st->size, st->alignment);
	ret += kapi_json_str(buf + ret, size - ret, st->description);
	ret += scnprintf(buf + ret, size - ret, ",\n      \"fields\": [\n");

	for (i = 0; i < st->field_count && i < KAPI_MAX_PARAMS; i++) {
		const struct kapi_struct_field *field = &st->fields[i];

		ret += scnprintf(buf + ret, size - ret,
			"        {\n          \"name\": ");
		ret += kapi_json_str(buf + ret, size - ret, field->name);
		ret += scnprintf(buf + ret, size - ret, ",\n          \"type\": ");
		ret += kapi_json_str(buf + ret, size - ret, field->type_name);
		ret += scnprintf(buf + ret, size - ret,
			",\n"
			"          \"type_class\": \"%s\",\n"
			"          \"offset\": %zu,\n"
			"          \"size\": %zu,\n"
			"          \"flags\": \"0x%x\",\n"
			"          \"constraint_type\": \"%s\",\n"
			"          \"min_value\": %lld,\n"
			"          \"max_value\": %lld,\n"
			"          \"valid_mask\": \"0x%llx\",\n"
			"          \"enum_values\": ",
			kapi_param_type_to_string(field->type),
			field->offset, field->size, field->flags,
			constraint_type_to_string(field->constraint_type),
			field->min_value, field->max_value, field->valid_mask);
		ret += kapi_json_str(buf + ret, size - ret, field->enum_values);
		ret += scnprintf(buf + ret, size - ret,
			",\n          \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, field->description);
		ret += scnprintf(buf + ret, size - ret,
			"\n        }%s\n",
			(i < st->field_count - 1) ? "," : "");
	}

	ret += scnprintf(buf + ret, size - ret, "      ]\n    }");
	return ret;
}

/**
 * kapi_export_json - Export API specification to JSON format
 * @spec: API specification to export
 * @buf: Buffer to write JSON to
 * @size: Size of buffer
 *
 * Return: Number of bytes written, -EINVAL on bad arguments, or -E2BIG
 * if the output did not fit in @buf (the contents are then incomplete)
 */
int kapi_export_json(const struct kernel_api_spec *spec, char *buf, size_t size)
{
	int ret = 0;
	int i, j;

	if (!spec || !buf || size == 0)
		return -EINVAL;

	ret = scnprintf(buf, size, "{\n  \"name\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->name);
	ret += scnprintf(buf + ret, size - ret,
			 ",\n  \"version\": %u,\n  \"description\": ",
			 spec->version);
	ret += kapi_json_str(buf + ret, size - ret, spec->description);
	ret += scnprintf(buf + ret, size - ret, ",\n  \"long_description\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->long_description);
	ret += scnprintf(buf + ret, size - ret,
			 ",\n  \"context_flags\": \"0x%x\",\n",
			 spec->context_flags);

	/* Parameters */
	ret += scnprintf(buf + ret, size - ret, "  \"parameters\": [\n");

	for (i = 0; i < spec->param_count && i < KAPI_MAX_PARAMS; i++) {
		const struct kapi_param_spec *param = &spec->params[i];

		ret += scnprintf(buf + ret, size - ret, "    {\n      \"name\": ");
		ret += kapi_json_str(buf + ret, size - ret, param->name);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"type\": ");
		ret += kapi_json_str(buf + ret, size - ret, param->type_name);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"type_class\": \"%s\",\n      \"flags\": \"0x%x\",\n      \"description\": ",
			kapi_param_type_to_string(param->type),
			param->flags);
		ret += kapi_json_str(buf + ret, size - ret, param->description);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"constraint_type\": \"%s\",\n      \"constraint_desc\": ",
			constraint_type_to_string(param->constraint_type));
		ret += kapi_json_str(buf + ret, size - ret, param->constraints);
		ret += scnprintf(buf + ret, size - ret,
			",\n"
			"      \"min_value\": %lld,\n"
			"      \"max_value\": %lld,\n"
			"      \"valid_mask\": \"0x%llx\",\n"
			"      \"enum_values\": ",
			param->min_value, param->max_value, param->valid_mask);
		ret += kapi_json_s64_list(buf + ret, size - ret,
					  param->enum_values, param->enum_count);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"size\": %zu,\n      \"alignment\": %zu,\n      \"size_param_idx\": ",
			param->size, param->alignment);
		if (param->size_param_idx > 0)
			ret += scnprintf(buf + ret, size - ret, "%d",
					 param->size_param_idx - 1);
		else
			ret += scnprintf(buf + ret, size - ret, "null");
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"size_multiplier\": %zu\n    }%s\n",
			param->size_multiplier,
			(i < spec->param_count - 1) ? "," : "");
	}

	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Return value */
	ret += scnprintf(buf + ret, size - ret, "  \"return\": {\n    \"type\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->return_spec.type_name);
	ret += scnprintf(buf + ret, size - ret,
		",\n"
		"    \"type_class\": \"%s\",\n"
		"    \"check_type\": \"%s\",\n"
		"    \"success_value\": %lld,\n"
		"    \"success_min\": %lld,\n"
		"    \"success_max\": %lld,\n"
		"    \"error_values\": ",
		kapi_param_type_to_string(spec->return_spec.type),
		return_check_type_to_string(spec->return_spec.check_type),
		spec->return_spec.success_value,
		spec->return_spec.success_min,
		spec->return_spec.success_max);
	ret += kapi_json_s64_list(buf + ret, size - ret,
				  spec->return_spec.error_values,
				  spec->return_spec.error_count);

	ret += scnprintf(buf + ret, size - ret, ",\n    \"description\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->return_spec.description);
	ret += scnprintf(buf + ret, size - ret, "\n  },\n");

	/* Errors */
	ret += scnprintf(buf + ret, size - ret, "  \"errors\": [\n");

	for (i = 0; i < spec->error_count && i < KAPI_MAX_ERRORS; i++) {
		const struct kapi_error_spec *error = &spec->errors[i];

		ret += scnprintf(buf + ret, size - ret,
			"    {\n      \"code\": %d,\n      \"name\": ",
			error->error_code);
		ret += kapi_json_str(buf + ret, size - ret, error->name);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"condition\": ");
		ret += kapi_json_str(buf + ret, size - ret, error->condition);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, error->description);
		ret += scnprintf(buf + ret, size - ret,
			"\n    }%s\n",
			(i < spec->error_count - 1) ? "," : "");
	}

	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Locks */
	ret += scnprintf(buf + ret, size - ret, "  \"locks\": [\n");

	for (i = 0; i < spec->lock_count && i < KAPI_MAX_LOCKS; i++) {
		const struct kapi_lock_spec *lock = &spec->locks[i];

		ret += scnprintf(buf + ret, size - ret, "    {\n      \"name\": ");
		ret += kapi_json_str(buf + ret, size - ret, lock->lock_name);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"type\": \"%s\",\n      \"scope\": \"%s\",\n      \"description\": ",
			kapi_lock_type_to_string(lock->lock_type),
			kapi_lock_scope_to_string(lock->scope));
		ret += kapi_json_str(buf + ret, size - ret, lock->description);
		ret += scnprintf(buf + ret, size - ret,
			"\n    }%s\n",
			(i < spec->lock_count - 1) ? "," : "");
	}

	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Capabilities */
	ret += scnprintf(buf + ret, size - ret, "  \"capabilities\": [\n");

	for (i = 0; i < spec->capability_count && i < KAPI_MAX_CAPABILITIES; i++) {
		const struct kapi_capability_spec *cap = &spec->capabilities[i];

		ret += scnprintf(buf + ret, size - ret,
			"    {\n      \"capability\": %d,\n      \"name\": ",
			cap->capability);
		ret += kapi_json_str(buf + ret, size - ret, cap->cap_name);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"action\": \"%s\",\n      \"allows\": ",
			capability_action_to_string(cap->action));
		ret += kapi_json_str(buf + ret, size - ret, cap->allows);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"without_cap\": ");
		ret += kapi_json_str(buf + ret, size - ret, cap->without_cap);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"check_condition\": ");
		ret += kapi_json_str(buf + ret, size - ret, cap->check_condition);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"priority\": %u", cap->priority);

		if (cap->alternative_count > 0) {
			ret += scnprintf(buf + ret, size - ret,
				",\n      \"alternatives\": [");
			for (j = 0; j < cap->alternative_count && j < KAPI_MAX_CAPABILITIES; j++) {
				ret += scnprintf(buf + ret, size - ret,
					"%s%d", j ? ", " : "",
					cap->alternative[j]);
			}
			ret += scnprintf(buf + ret, size - ret, "]");
		}

		ret += scnprintf(buf + ret, size - ret,
			"\n    }%s\n",
			(i < spec->capability_count - 1) ? "," : "");
	}

	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Constraints */
	ret += scnprintf(buf + ret, size - ret, "  \"constraints\": [\n");
	for (i = 0; i < spec->constraint_count && i < KAPI_MAX_CONSTRAINTS; i++) {
		const struct kapi_constraint_spec *con = &spec->constraints[i];

		ret += scnprintf(buf + ret, size - ret, "    {\n      \"name\": ");
		ret += kapi_json_str(buf + ret, size - ret, con->name);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, con->description);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"expression\": ");
		ret += kapi_json_str(buf + ret, size - ret, con->expression);
		ret += scnprintf(buf + ret, size - ret,
			"\n    }%s\n",
			(i < spec->constraint_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Signals */
	ret += scnprintf(buf + ret, size - ret, "  \"signals\": [\n");
	for (i = 0; i < spec->signal_count && i < KAPI_MAX_SIGNALS; i++) {
		const struct kapi_signal_spec *sig = &spec->signals[i];

		ret += scnprintf(buf + ret, size - ret,
			"    {\n      \"signal_num\": %d,\n      \"signal_name\": ",
			sig->signal_num);
		ret += kapi_json_str(buf + ret, size - ret, sig->signal_name);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"direction\": \"0x%x\",\n      \"action\": %u,\n      \"target\": ",
			sig->direction, sig->action);
		ret += kapi_json_str(buf + ret, size - ret, sig->target);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"condition\": ");
		ret += kapi_json_str(buf + ret, size - ret, sig->condition);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, sig->description);
		ret += scnprintf(buf + ret, size - ret,
			",\n"
			"      \"restartable\": %s,\n"
			"      \"sa_flags_required\": \"0x%x\",\n"
			"      \"sa_flags_forbidden\": \"0x%x\",\n"
			"      \"error_on_signal\": %d,\n"
			"      \"transform_to\": %d,\n"
			"      \"timing\": ",
			sig->restartable ? "true" : "false",
			sig->sa_flags_required,
			sig->sa_flags_forbidden,
			sig->error_on_signal,
			sig->transform_to);
		ret += kapi_json_str(buf + ret, size - ret, sig->timing);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"priority\": %u,\n      \"interruptible\": %s,\n      \"queue_behavior\": ",
			sig->priority,
			sig->interruptible ? "true" : "false");
		ret += kapi_json_str(buf + ret, size - ret, sig->queue_behavior);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"state_required\": \"0x%x\",\n      \"state_forbidden\": \"0x%x\"\n    }%s\n",
			sig->state_required,
			sig->state_forbidden,
			(i < spec->signal_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Side effects */
	ret += scnprintf(buf + ret, size - ret, "  \"side_effects\": [\n");
	for (i = 0; i < spec->side_effect_count && i < KAPI_MAX_SIDE_EFFECTS; i++) {
		const struct kapi_side_effect *eff = &spec->side_effects[i];

		ret += scnprintf(buf + ret, size - ret,
			"    {\n      \"type\": \"0x%x\",\n      \"target\": ",
			eff->type);
		ret += kapi_json_str(buf + ret, size - ret, eff->target);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"condition\": ");
		ret += kapi_json_str(buf + ret, size - ret, eff->condition);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, eff->description);
		ret += scnprintf(buf + ret, size - ret,
			",\n      \"reversible\": %s\n    }%s\n",
			eff->reversible ? "true" : "false",
			(i < spec->side_effect_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* State transitions */
	ret += scnprintf(buf + ret, size - ret, "  \"state_transitions\": [\n");
	for (i = 0; i < spec->state_trans_count && i < KAPI_MAX_STATE_TRANS; i++) {
		const struct kapi_state_transition *trans = &spec->state_transitions[i];

		ret += scnprintf(buf + ret, size - ret, "    {\n      \"object\": ");
		ret += kapi_json_str(buf + ret, size - ret, trans->object);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"from_state\": ");
		ret += kapi_json_str(buf + ret, size - ret, trans->from_state);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"to_state\": ");
		ret += kapi_json_str(buf + ret, size - ret, trans->to_state);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"condition\": ");
		ret += kapi_json_str(buf + ret, size - ret, trans->condition);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, trans->description);
		ret += scnprintf(buf + ret, size - ret,
			"\n    }%s\n",
			(i < spec->state_trans_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Signal masks */
	ret += scnprintf(buf + ret, size - ret, "  \"signal_masks\": [\n");
	for (i = 0; i < spec->signal_mask_count && i < KAPI_MAX_SIGNALS; i++) {
		const struct kapi_signal_mask_spec *mask = &spec->signal_masks[i];

		ret += scnprintf(buf + ret, size - ret, "    {\n      \"name\": ");
		ret += kapi_json_str(buf + ret, size - ret, mask->mask_name);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"description\": ");
		ret += kapi_json_str(buf + ret, size - ret, mask->description);
		ret += scnprintf(buf + ret, size - ret, ",\n      \"signals\": [");
		for (j = 0; j < mask->signal_count && j < KAPI_MAX_SIGNALS; j++)
			ret += scnprintf(buf + ret, size - ret, "%s%d",
					 j ? ", " : "", mask->signals[j]);
		ret += scnprintf(buf + ret, size - ret,
			"]\n    }%s\n",
			(i < spec->signal_mask_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Structure specifications */
	ret += scnprintf(buf + ret, size - ret, "  \"struct_specs\": [\n");
	for (i = 0; i < spec->struct_spec_count && i < KAPI_MAX_STRUCT_SPECS; i++) {
		ret += kapi_json_struct_spec(buf + ret, size - ret,
					     &spec->struct_specs[i]);
		ret += scnprintf(buf + ret, size - ret, "%s\n",
				 (i < spec->struct_spec_count - 1) ? "," : "");
	}
	ret += scnprintf(buf + ret, size - ret, "  ],\n");

	/* Additional info */
	ret += scnprintf(buf + ret, size - ret, "  \"examples\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->examples);
	ret += scnprintf(buf + ret, size - ret, ",\n  \"notes\": ");
	ret += kapi_json_str(buf + ret, size - ret, spec->notes);
	ret += scnprintf(buf + ret, size - ret, "\n}\n");

	/* scnprintf() never writes past size - 1, so a full buffer means truncation */
	if (ret >= size - 1)
		return -E2BIG;

	return ret;
}
EXPORT_SYMBOL_GPL(kapi_export_json);

#ifdef CONFIG_KAPI_RUNTIME_CHECKS

#define CREATE_TRACE_POINTS
#include <trace/events/kapi.h>

/*
 * Render a syscall's parameters as a "name=value, ..." string for the
 * kapi_syscall_enter tracepoint.  Names come from the spec; pointer-like
 * values are shown in hex, integers and file descriptors in decimal.
 */
static void kapi_trace_format_params(const struct kernel_api_spec *spec,
				     const s64 *args, int nargs,
				     char *buf, size_t size)
{
	int i, used = 0;

	buf[0] = '\0';
	/* Bound by the caller-supplied arg count; the spec arity may differ. */
	for (i = 0; args && i < nargs && i < 6; i++) {
		const char *name = "arg";
		bool dec = false;

		if (i < spec->param_count) {
			const struct kapi_param_spec *ps = &spec->params[i];

			if (ps->name)
				name = ps->name;
			dec = ps->type == KAPI_TYPE_INT || ps->type == KAPI_TYPE_FD;
		}

		used += scnprintf(buf + used, size - used, "%s%s=",
				  i ? ", " : "", name);
		if (dec)
			used += scnprintf(buf + used, size - used, "%lld",
					  (long long)args[i]);
		else
			used += scnprintf(buf + used, size - used, "0x%llx",
					  (unsigned long long)args[i]);
	}
}

/**
 * kapi_validate_fd - Validate that a file descriptor value is in valid range
 * @fd: File descriptor to validate
 *
 * Only the numeric range is checked: AT_FDCWD or a non-negative value.
 * Openness is left to the syscall, since the fd can be closed between check
 * and use. Other negative values are rejected here, so a syscall such as
 * close() fails with EINVAL for them rather than EBADF.
 *
 * Return: true if fd is in valid range, false otherwise
 */
static bool kapi_validate_fd(int fd)
{
	return fd == AT_FDCWD || fd >= 0;
}

/**
 * kapi_validate_user_ptr - Validate that a user pointer is accessible
 * @ptr: User pointer to validate
 * @size: Size in bytes to validate
 *
 * Return: true if user memory is accessible, false otherwise
 */
static bool kapi_validate_user_ptr(const void __user *ptr, size_t size)
{
	/* NULL pointers are not valid; caller handles optional case */
	if (!ptr)
		return false;

	return access_ok(ptr, size);
}

/**
 * kapi_validate_user_ptr_with_params - Validate user pointer with dynamic size
 * @param_spec: Parameter specification
 * @ptr: User pointer to validate
 * @all_params: Array of all parameter values
 * @param_count: Number of parameters
 *
 * Return: true if user memory is accessible, false otherwise
 */
static bool kapi_validate_user_ptr_with_params(const struct kapi_param_spec *param_spec,
						const void __user *ptr,
						const s64 *all_params,
						int param_count)
{
	size_t actual_size;

	/* NULL is allowed for optional parameters */
	if (!ptr && (param_spec->flags & KAPI_PARAM_OPTIONAL))
		return true;

	/*
	 * size_param_idx is stored 1-based (0 means "no dynamic sizing").
	 * Convert to a real index before looking into all_params.
	 */
	if (param_spec->size_param_idx > 0 &&
	    param_spec->size_param_idx - 1 < param_count) {
		s64 count = all_params[param_spec->size_param_idx - 1];
		size_t unit = param_spec->size_multiplier ?: 1;

		if (count < 0) {
			pr_warn_ratelimited("Parameter %s: size determinant is negative (%lld)\n",
				param_spec->name, count);
			return false;
		}

		/* A zero-length access never dereferences the pointer */
		if (count == 0)
			return true;

		if (count > SIZE_MAX / unit) {
			pr_warn_ratelimited("Parameter %s: size calculation overflow\n",
				param_spec->name);
			return false;
		}

		actual_size = (size_t)count * unit;
	} else {
		actual_size = param_spec->size;
	}

	return kapi_validate_user_ptr(ptr, actual_size);
}

/**
 * kapi_validate_path - Validate that a pathname is accessible and within limits
 * @path: User pointer to pathname
 * @param_spec: Parameter specification
 *
 * Return: true if path is valid, false otherwise
 */
static bool kapi_validate_path(const char __user *path,
				const struct kapi_param_spec *param_spec)
{
	size_t len;

	/* NULL is allowed for optional parameters */
	if (!path && (param_spec->flags & KAPI_PARAM_OPTIONAL))
		return true;

	if (!path) {
		pr_warn_ratelimited("Parameter %s: NULL path not allowed\n", param_spec->name);
		return false;
	}

	if (!access_ok(path, 1)) {
		pr_warn_ratelimited("Parameter %s: path pointer %p not accessible\n",
			param_spec->name, path);
		return false;
	}

	/*
	 * Use strnlen_user to check the path length and accessibility.
	 * Note: strnlen_user() is subject to TOCTOU -- the measured length
	 * may change if another thread modifies the user memory. This is
	 * acceptable since the kernel re-copies and re-validates the path
	 * later in the syscall path. This check is best-effort.
	 */
	len = strnlen_user(path, PATH_MAX + 1);
	if (len == 0) {
		pr_warn_ratelimited("Parameter %s: invalid path pointer %p\n",
			param_spec->name, path);
		return false;
	}

	if (len > PATH_MAX) {
		pr_warn_ratelimited("Parameter %s: path too long (exceeds PATH_MAX)\n",
			param_spec->name);
		return false;
	}

	return true;
}

/**
 * kapi_validate_user_string - Validate a userspace null-terminated string
 * @str: User pointer to string
 * @param_spec: Parameter specification containing length constraints
 *
 * Validates that the userspace string pointer is accessible and that the
 * string length (excluding null terminator) is within the range specified
 * by min_value and max_value in the parameter specification.
 *
 * Return: true if string is valid, false otherwise
 */
static bool kapi_validate_user_string(const char __user *str,
				       const struct kapi_param_spec *param_spec)
{
	size_t len;
	size_t max_check_len;

	/* NULL is allowed for optional parameters */
	if (!str && (param_spec->flags & KAPI_PARAM_OPTIONAL))
		return true;

	if (!str) {
		pr_warn_ratelimited("Parameter %s: NULL string not allowed\n", param_spec->name);
		return false;
	}

	if (!access_ok(str, 1)) {
		pr_warn_ratelimited("Parameter %s: string pointer %p not accessible\n",
			param_spec->name, str);
		return false;
	}

	/*
	 * Use strnlen_user to check the string length and validate accessibility.
	 * Check up to max_value + 1 to detect strings that are too long.
	 * If max_value is 0 or unset, use PATH_MAX as a reasonable default.
	 *
	 * Note: strnlen_user() is subject to TOCTOU -- see comment in
	 * kapi_validate_path() above. This check is best-effort.
	 */
	max_check_len = param_spec->max_value > 0 ?
			(size_t)param_spec->max_value + 1 : PATH_MAX + 1;
	len = strnlen_user(str, max_check_len);

	if (len == 0) {
		pr_warn_ratelimited("Parameter %s: invalid string pointer %p\n",
			param_spec->name, str);
		return false;
	}

	/*
	 * strnlen_user returns the length including the null terminator.
	 * Convert to string length (excluding terminator) for range check.
	 */
	len--;

	if (param_spec->min_value > 0 && len < (size_t)param_spec->min_value) {
		pr_warn_ratelimited("Parameter %s: string too short (%zu < %lld)\n",
			param_spec->name, len, param_spec->min_value);
		return false;
	}

	if (param_spec->max_value > 0 && len > (size_t)param_spec->max_value) {
		pr_warn_ratelimited("Parameter %s: string too long (%zu > %lld)\n",
			param_spec->name, len, param_spec->max_value);
		return false;
	}

	return true;
}

/**
 * kapi_validate_user_ptr_constraint - Validate a userspace pointer with size
 * @ptr: User pointer to validate
 * @param_spec: Parameter specification containing size
 *
 * Validates that the userspace pointer is accessible and that the memory
 * region of the specified size can be accessed. The size is taken from
 * the param_spec->size field.
 *
 * Return: true if pointer is valid, false otherwise
 */
static bool kapi_validate_user_ptr_constraint(const void __user *ptr,
					       const struct kapi_param_spec *param_spec)
{
	/* NULL is allowed for optional parameters */
	if (!ptr && (param_spec->flags & KAPI_PARAM_OPTIONAL))
		return true;

	if (!ptr) {
		pr_warn_ratelimited("Parameter %s: NULL pointer not allowed\n", param_spec->name);
		return false;
	}

	if (param_spec->size == 0) {
		pr_warn_ratelimited("Parameter %s: size not specified for user pointer validation\n",
			param_spec->name);
		return false;
	}

	if (!access_ok(ptr, param_spec->size)) {
		pr_warn_ratelimited("Parameter %s: user pointer %p not accessible for %zu bytes\n",
			param_spec->name, ptr, param_spec->size);
		return false;
	}

	return true;
}

/*
 * check_user_ptr is false once the pointer was validated against its dynamic
 * size; the fixed-size check would reject NULL even when that size is 0.
 */
static bool kapi_validate_param_checks(const struct kapi_param_spec *param_spec,
				       s64 value, bool check_user_ptr)
{
	int i;

	/* Special handling for file descriptor type */
	if (param_spec->type == KAPI_TYPE_FD &&
	    !(param_spec->flags & KAPI_PARAM_OPTIONAL)) {
		if (value < INT_MIN || value > INT_MAX) {
			pr_warn_ratelimited("Parameter %s: file descriptor %lld out of int range\n",
				param_spec->name, value);
			return false;
		}
		if (!kapi_validate_fd((int)value)) {
			pr_warn_ratelimited("Parameter %s: invalid file descriptor %lld\n",
				param_spec->name, value);
			return false;
		}
	}

	/* Special handling for user pointer type */
	if (check_user_ptr && param_spec->type == KAPI_TYPE_USER_PTR) {
		const void __user *ptr = (const void __user *)(unsigned long)value;

		/* NULL is allowed for optional parameters */
		if (!ptr && (param_spec->flags & KAPI_PARAM_OPTIONAL))
			return true;

		if (!kapi_validate_user_ptr(ptr, param_spec->size)) {
			pr_warn_ratelimited("Parameter %s: invalid user pointer %p (size: %zu)\n",
				param_spec->name, ptr, param_spec->size);
			return false;
		}
	}

	/* Special handling for path type */
	if (param_spec->type == KAPI_TYPE_PATH) {
		const char __user *path = (const char __user *)(unsigned long)value;

		if (!kapi_validate_path(path, param_spec))
			return false;
	}

	switch (param_spec->constraint_type) {
	case KAPI_CONSTRAINT_NONE:
	case KAPI_CONSTRAINT_BUFFER:
		return true;

	case KAPI_CONSTRAINT_RANGE:
		/*
		 * If max_value is below min_value, it was likely set from an
		 * unsigned constant (e.g. SIZE_MAX) that overflowed s64.  Treat
		 * as no upper bound; only check the minimum.
		 */
		if (param_spec->max_value >= param_spec->min_value) {
			if (value < param_spec->min_value ||
			    value > param_spec->max_value) {
				pr_warn_ratelimited("Parameter %s value %lld out of range [%lld, %lld]\n",
					param_spec->name, value,
					param_spec->min_value,
					param_spec->max_value);
				return false;
			}
		} else {
			if (value < param_spec->min_value) {
				pr_warn_ratelimited("Parameter %s value %lld below minimum %lld\n",
					param_spec->name, value,
					param_spec->min_value);
				return false;
			}
		}
		return true;

	case KAPI_CONSTRAINT_MASK:
		if (value & ~param_spec->valid_mask) {
			pr_warn_ratelimited("Parameter %s value 0x%llx contains invalid bits (valid mask: 0x%llx)\n",
				param_spec->name, value, param_spec->valid_mask);
			return false;
		}
		return true;

	case KAPI_CONSTRAINT_ENUM:
		if (!param_spec->enum_values || param_spec->enum_count == 0)
			return true;

		for (i = 0; i < param_spec->enum_count; i++) {
			if (value == param_spec->enum_values[i])
				return true;
		}
		pr_warn_ratelimited("Parameter %s value %lld not in valid enumeration\n",
			param_spec->name, value);
		return false;

	case KAPI_CONSTRAINT_ALIGNMENT:
		if (param_spec->alignment == 0) {
			pr_warn_ratelimited("Parameter %s: alignment constraint specified but alignment is 0\n",
				param_spec->name);
			return false;
		}
		if (param_spec->alignment & (param_spec->alignment - 1)) {
			pr_warn_ratelimited("Parameter %s: alignment %zu is not a power of two\n",
				param_spec->name, param_spec->alignment);
			return false;
		}
		if (value & (param_spec->alignment - 1)) {
			pr_warn_ratelimited("Parameter %s value 0x%llx not aligned to %zu boundary\n",
				param_spec->name, value, param_spec->alignment);
			return false;
		}
		return true;

	case KAPI_CONSTRAINT_POWER_OF_TWO:
		if (value == 0 || (value & (value - 1))) {
			pr_warn_ratelimited("Parameter %s value %lld is not a power of two\n",
				param_spec->name, value);
			return false;
		}
		return true;

	case KAPI_CONSTRAINT_PAGE_ALIGNED:
		if (value & (PAGE_SIZE - 1)) {
			pr_warn_ratelimited("Parameter %s value 0x%llx not page-aligned (PAGE_SIZE=%ld)\n",
				param_spec->name, value, PAGE_SIZE);
			return false;
		}
		return true;

	case KAPI_CONSTRAINT_NONZERO:
		if (value == 0) {
			pr_warn_ratelimited("Parameter %s must be non-zero\n", param_spec->name);
			return false;
		}
		return true;

	case KAPI_CONSTRAINT_USER_STRING:
		return kapi_validate_user_string((const char __user *)(unsigned long)value,
						 param_spec);

	case KAPI_CONSTRAINT_USER_PATH:
		return kapi_validate_path((const char __user *)(unsigned long)value, param_spec);

	case KAPI_CONSTRAINT_USER_PTR:
		return kapi_validate_user_ptr_constraint((const void __user *)(unsigned long)value,
							 param_spec);

	case KAPI_CONSTRAINT_CUSTOM:
		if (param_spec->validate)
			return param_spec->validate(value);
		return true;

	default:
		return true;
	}
}

/**
 * kapi_validate_param - Validate a parameter against its specification
 * @param_spec: Parameter specification
 * @value: Parameter value to validate
 *
 * Return: true if valid, false otherwise
 */
bool kapi_validate_param(const struct kapi_param_spec *param_spec, s64 value)
{
	return kapi_validate_param_checks(param_spec, value, true);
}
EXPORT_SYMBOL_GPL(kapi_validate_param);

/**
 * kapi_validate_param_with_context - Validate parameter with access to all params
 * @param_spec: Parameter specification
 * @value: Parameter value to validate
 * @all_params: Array of all parameter values
 * @param_count: Number of parameters
 *
 * Return: true if valid, false otherwise
 */
bool kapi_validate_param_with_context(const struct kapi_param_spec *param_spec,
				       s64 value, const s64 *all_params, int param_count)
{
	/* Special handling for user pointer type with dynamic sizing */
	if (param_spec->type == KAPI_TYPE_USER_PTR) {
		const void __user *ptr = (const void __user *)(unsigned long)value;

		/* NULL is allowed for optional parameters */
		if (!ptr && (param_spec->flags & KAPI_PARAM_OPTIONAL))
			return true;

		if (!kapi_validate_user_ptr_with_params(param_spec, ptr, all_params, param_count)) {
			pr_warn_ratelimited("Parameter %s: invalid user pointer %p\n",
				param_spec->name, ptr);
			return false;
		}
		return kapi_validate_param_checks(param_spec, value, false);
	}

	/* For other types, fall back to regular validation */
	return kapi_validate_param(param_spec, value);
}
EXPORT_SYMBOL_GPL(kapi_validate_param_with_context);

/**
 * kapi_validate_syscall_params - Validate all syscall parameters together
 * @spec: API specification
 * @params: Array of parameter values
 * @param_count: Number of parameters
 *
 * Return: -EINVAL if any parameter is invalid, 0 if all valid
 */
int kapi_validate_syscall_params(const struct kernel_api_spec *spec,
				 const s64 *params, int param_count)
{
	int i, ret = 0;

	if (!spec || !params)
		return 0;

	if (trace_kapi_syscall_enter_enabled()) {
		char pbuf[KAPI_TP_PARAMS_LEN];

		kapi_trace_format_params(spec, params, param_count, pbuf, sizeof(pbuf));
		trace_kapi_syscall_enter(spec->name, param_count, params, pbuf);
	}

	/* Validate that we have the expected number of parameters */
	if (param_count != spec->param_count) {
		pr_warn_ratelimited("API %s: parameter count mismatch (expected %u, got %d)\n",
			spec->name, spec->param_count, param_count);
		ret = -EINVAL;
		goto out;
	}

	/* Validate each parameter with context */
	for (i = 0; i < spec->param_count && i < KAPI_MAX_PARAMS; i++) {
		const struct kapi_param_spec *param_spec = &spec->params[i];

		if (!kapi_validate_param_with_context(param_spec, params[i], params, param_count)) {
			if (strncmp(spec->name, "sys_", 4) == 0) {
				/* For syscalls, we can return EINVAL to userspace */
				ret = -EINVAL;
				goto out;
			}
		}
	}

out:
	/*
	 * Emit the exit event on the rejection path too (the wrapper
	 * short-circuits the handler on a non-zero return), so every
	 * kapi_syscall_enter has a matching kapi_syscall_exit.
	 */
	if (ret)
		trace_kapi_syscall_exit(spec->name, ret, false);

	return ret;
}
EXPORT_SYMBOL_GPL(kapi_validate_syscall_params);

/**
 * kapi_check_return_success - Check if return value indicates success
 * @return_spec: Return specification
 * @retval: Return value to check
 *
 * Return: true if the return value indicates success according to the spec.
 */
bool kapi_check_return_success(const struct kapi_return_spec *return_spec, s64 retval)
{
	u32 i;

	if (!return_spec)
		return true;

	switch (return_spec->check_type) {
	case KAPI_RETURN_EXACT:
		return retval == return_spec->success_value;

	case KAPI_RETURN_RANGE:
		return retval >= return_spec->success_min &&
		       retval <= return_spec->success_max;

	case KAPI_RETURN_ERROR_CHECK:
		/* Success if NOT in error list */
		if (return_spec->error_values) {
			for (i = 0; i < return_spec->error_count; i++) {
				if (retval == return_spec->error_values[i])
					return false;
			}
		}
		return true;

	case KAPI_RETURN_FD:
		/* File descriptors: >= 0 is success, < 0 is error */
		return retval >= 0;

	case KAPI_RETURN_CUSTOM:
		if (return_spec->is_success)
			return return_spec->is_success(retval);
		fallthrough;

	default:
		return true;
	}
}
EXPORT_SYMBOL_GPL(kapi_check_return_success);

/**
 * kapi_validate_return_value - Validate that return value matches spec
 * @spec: API specification
 * @retval: Return value to validate
 *
 * Return: false if the spec is a KAPI_RETURN_FD check and a successful @retval
 * is not a valid file descriptor, true otherwise.
 *
 * kapi_check_return_success() runs first. A value that does not satisfy it is
 * treated as an error and is accepted. An error code that is not listed in the
 * spec is only logged with pr_debug().
 */
bool kapi_validate_return_value(const struct kernel_api_spec *spec, s64 retval)
{
	int i;
	bool is_success;

	if (!spec)
		return true; /* No spec means we can't validate */

	/* First check if this is a success return */
	is_success = kapi_check_return_success(&spec->return_spec, retval);

	if (is_success) {
		/* Special validation for file descriptor returns */
		if (spec->return_spec.check_type == KAPI_RETURN_FD) {
			if (retval > INT_MAX || !kapi_validate_fd((int)retval)) {
				pr_warn_ratelimited("API %s returned invalid file descriptor %lld\n",
					spec->name, retval);
				return false;
			}
		}
		return true;
	}

	if (spec->error_count == 0) {
		pr_debug("API %s returned unspecified error %lld\n",
			 spec->name, retval);
		return true;
	}

	for (i = 0; i < spec->error_count && i < KAPI_MAX_ERRORS; i++) {
		if (retval == spec->errors[i].error_code)
			return true;
	}

	/*
	 * Error not in spec - log at debug level since filesystem-specific and
	 * device-specific error codes may not be exhaustively listed.
	 */
	pr_debug("API %s returned error code %lld not listed in spec\n",
		 spec->name, retval);

	return true;
}
EXPORT_SYMBOL_GPL(kapi_validate_return_value);

/**
 * kapi_validate_syscall_return - Validate syscall return value
 * @spec: API specification
 * @retval: Return value
 *
 * Return: always 0. A return value that does not match the spec is only
 * logged and the syscall result is left unchanged.
 */
int kapi_validate_syscall_return(const struct kernel_api_spec *spec, s64 retval)
{
	bool valid = true;

	if (!spec)
		return 0;

	/* Validate against the return spec when one was defined */
	if (spec->return_magic == KAPI_MAGIC_RETURN)
		valid = kapi_validate_return_value(spec, retval);

	trace_kapi_syscall_exit(spec->name, retval, valid);

	return 0;
}
EXPORT_SYMBOL_GPL(kapi_validate_syscall_return);

/**
 * kapi_check_context - Check if current context matches API requirements
 * @spec: API specification to check against
 */
void kapi_check_context(const struct kernel_api_spec *spec)
{
	bool valid = false;
	u32 ctx;

	if (!spec)
		return;

	ctx = spec->context_flags;

	if (!ctx)
		return;

	/* Check if we're in an allowed context */
	if ((ctx & KAPI_CTX_PROCESS) && !in_interrupt())
		valid = true;

	if ((ctx & KAPI_CTX_SOFTIRQ) && in_softirq())
		valid = true;

	if ((ctx & KAPI_CTX_HARDIRQ) && in_hardirq())
		valid = true;

	if ((ctx & KAPI_CTX_NMI) && in_nmi())
		valid = true;

	if (!valid)
		WARN_ONCE(1, "API %s called from invalid context\n", spec->name);

	/* Check specific requirements */
	if ((ctx & KAPI_CTX_ATOMIC) && preemptible())
		WARN_ONCE(1, "API %s requires atomic context\n", spec->name);

	if ((ctx & KAPI_CTX_SLEEPABLE) && !preemptible())
		WARN_ONCE(1, "API %s requires sleepable context\n", spec->name);
}
EXPORT_SYMBOL_GPL(kapi_check_context);

#endif /* CONFIG_KAPI_RUNTIME_CHECKS */
