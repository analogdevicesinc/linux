/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM kapi

#if !defined(_TRACE_KAPI_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_KAPI_H

#include <linux/tracepoint.h>

/* Max length of the rendered "name=value, ..." parameter list. */
#define KAPI_TP_PARAMS_LEN 256

/*
 * Emitted from the CONFIG_KAPI_RUNTIME_CHECKS syscall validation path for
 * syscalls that have a KAPI specification: kapi_syscall_enter fires before
 * parameter validation, kapi_syscall_exit after the handler returns.
 * @name is the spec name, e.g. "sys_open".
 *
 * kapi_syscall_enter carries both the raw argument values (args[]) and, when
 * the spec provides parameter metadata, a rendered "name=value" list (params,
 * built by the caller): pointer-like values in hex, integers and fds in decimal.
 */
TRACE_EVENT(kapi_syscall_enter,

	TP_PROTO(const char *name, int nargs, const s64 *args, const char *params),

	TP_ARGS(name, nargs, args, params),

	TP_STRUCT__entry(
		__string(	name,	name	)
		__field(	int,	nargs	)
		__array(	u64,	args,	6	)
		__string(	params,	params	)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->nargs = nargs;
		memset(__entry->args, 0, sizeof(__entry->args));
		if (args && nargs > 0)
			memcpy(__entry->args, args,
			       min_t(int, nargs, 6) * sizeof(__entry->args[0]));
		__assign_str(params);
	),

	TP_printk("%s(%s)", __get_str(name), __get_str(params))
);

TRACE_EVENT(kapi_syscall_exit,

	TP_PROTO(const char *name, long ret, bool spec_match),

	TP_ARGS(name, ret, spec_match),

	TP_STRUCT__entry(
		__string(	name,		name		)
		__field(	long,		ret		)
		__field(	bool,		spec_match	)
	),

	TP_fast_assign(
		__assign_str(name);
		__entry->ret = ret;
		__entry->spec_match = spec_match;
	),

	TP_printk("%s = %ld spec_match=%d",
		  __get_str(name), __entry->ret, __entry->spec_match)
);

#endif /* _TRACE_KAPI_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
