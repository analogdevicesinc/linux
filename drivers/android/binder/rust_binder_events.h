/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 Google, Inc.
 */

#undef TRACE_SYSTEM
#undef TRACE_INCLUDE_FILE
#undef TRACE_INCLUDE_PATH
#define TRACE_SYSTEM rust_binder
#define TRACE_INCLUDE_FILE rust_binder_events
#define TRACE_INCLUDE_PATH ../drivers/android/binder

#if !defined(_RUST_BINDER_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _RUST_BINDER_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(binder_ioctl,
	TP_PROTO(unsigned int cmd, unsigned long arg),
	TP_ARGS(cmd, arg),

	TP_STRUCT__entry(
		__field(unsigned int, cmd)
		__field(unsigned long, arg)
	),
	TP_fast_assign(
		__entry->cmd = cmd;
		__entry->arg = arg;
	),
	TP_printk("cmd=0x%x arg=0x%lx", __entry->cmd, __entry->arg)
);

DECLARE_EVENT_CLASS(binder_function_return_class,
	TP_PROTO(int ret),
	TP_ARGS(ret),
	TP_STRUCT__entry(
		__field(int, ret)
	),
	TP_fast_assign(
		__entry->ret = ret;
	),
	TP_printk("ret=%d", __entry->ret)
);

#define DEFINE_RBINDER_FUNCTION_RETURN_EVENT(name)	\
DEFINE_EVENT(binder_function_return_class, name,	\
	TP_PROTO(int ret), \
	TP_ARGS(ret))

DEFINE_RBINDER_FUNCTION_RETURN_EVENT(binder_ioctl_done);
DEFINE_RBINDER_FUNCTION_RETURN_EVENT(binder_read_done);
DEFINE_RBINDER_FUNCTION_RETURN_EVENT(binder_write_done);

TRACE_EVENT(binder_wait_for_work,
	TP_PROTO(bool proc_work, bool transaction_stack, bool thread_todo),
	TP_ARGS(proc_work, transaction_stack, thread_todo),
	TP_STRUCT__entry(
		__field(bool, proc_work)
		__field(bool, transaction_stack)
		__field(bool, thread_todo)
	),
	TP_fast_assign(
		__entry->proc_work = proc_work;
		__entry->transaction_stack = transaction_stack;
		__entry->thread_todo = thread_todo;
	),
	TP_printk("proc_work=%d transaction_stack=%d thread_todo=%d",
		  __entry->proc_work, __entry->transaction_stack,
		  __entry->thread_todo)
);

TRACE_EVENT(binder_transaction,
	TP_PROTO(bool reply, rust_binder_transaction t, struct task_struct *thread),
	TP_ARGS(reply, t, thread),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, target_node)
		__field(int, to_proc)
		__field(int, to_thread)
		__field(int, reply)
		__field(unsigned int, code)
		__field(unsigned int, flags)
	),
	TP_fast_assign(
		rust_binder_process to = rust_binder_transaction_to_proc(t);
		rust_binder_node target_node = rust_binder_transaction_target_node(t);

		__entry->debug_id = rust_binder_transaction_debug_id(t);
		__entry->target_node = target_node ? rust_binder_node_debug_id(target_node) : 0;
		__entry->to_proc = rust_binder_process_task(to)->pid;
		__entry->to_thread = thread ? thread->pid : 0;
		__entry->reply = reply;
		__entry->code = rust_binder_transaction_code(t);
		__entry->flags = rust_binder_transaction_flags(t);
	),
	TP_printk("transaction=%d dest_node=%d dest_proc=%d dest_thread=%d reply=%d flags=0x%x code=0x%x",
		  __entry->debug_id, __entry->target_node,
		  __entry->to_proc, __entry->to_thread,
		  __entry->reply, __entry->flags, __entry->code)
);

TRACE_EVENT(binder_transaction_received,
	TP_PROTO(rust_binder_transaction t),
	TP_ARGS(t),
	TP_STRUCT__entry(
		__field(int, debug_id)
	),
	TP_fast_assign(
		__entry->debug_id = rust_binder_transaction_debug_id(t);
	),
	TP_printk("transaction=%d", __entry->debug_id)
);

TRACE_EVENT(binder_transaction_fd_send,
	TP_PROTO(int t_debug_id, int fd, size_t offset),
	TP_ARGS(t_debug_id, fd, offset),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, fd)
		__field(size_t, offset)
	),
	TP_fast_assign(
		__entry->debug_id = t_debug_id;
		__entry->fd = fd;
		__entry->offset = offset;
	),
	TP_printk("transaction=%d src_fd=%d offset=%zu",
		  __entry->debug_id, __entry->fd, __entry->offset)
);

TRACE_EVENT(binder_transaction_fd_recv,
	TP_PROTO(int t_debug_id, int fd, size_t offset),
	TP_ARGS(t_debug_id, fd, offset),
	TP_STRUCT__entry(
		__field(int, debug_id)
		__field(int, fd)
		__field(size_t, offset)
	),
	TP_fast_assign(
		__entry->debug_id = t_debug_id;
		__entry->fd = fd;
		__entry->offset = offset;
	),
	TP_printk("transaction=%d dest_fd=%d offset=%zu",
		  __entry->debug_id, __entry->fd, __entry->offset)
);

#endif /* _RUST_BINDER_TRACE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
