/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM workqueue

#if !defined(_TRACE_WORKQUEUE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_WORKQUEUE_H

#include <linux/tracepoint.h>
#include <linux/workqueue.h>

struct pool_workqueue;

/**
 * workqueue_queue_work - called when a work gets queued
 * @req_cpu:	the requested cpu
 * @pwq:	pointer to struct pool_workqueue
 * @work:	pointer to struct work_struct
 *
 * This event occurs when a work is queued immediately or once a
 * delayed work is actually queued on a workqueue (ie: once the delay
 * has been reached).
 */
TRACE_EVENT(workqueue_queue_work,

	TP_PROTO(int req_cpu, struct pool_workqueue *pwq,
		 struct work_struct *work),

	TP_ARGS(req_cpu, pwq, work),

	TP_STRUCT__entry(
		__field( void *,	work	)
		__field( void *,	function)
		__string( workqueue,	pwq->wq->name)
		__field( int,	req_cpu	)
		__field( int,	cpu	)
	),

	TP_fast_assign(
		__entry->work		= work;
		__entry->function	= work->func;
		__assign_str(workqueue);
		__entry->req_cpu	= req_cpu;
		__entry->cpu		= pwq->pool->cpu;
	),

	TP_printk("work struct=%p function=%ps workqueue=%s req_cpu=%d cpu=%d",
		  __entry->work, __entry->function, __get_str(workqueue),
		  __entry->req_cpu, __entry->cpu)
);

/**
 * workqueue_activate_work - called when a work gets activated
 * @work:	pointer to struct work_struct
 *
 * This event occurs when a queued work is put on the active queue,
 * which happens immediately after queueing unless @max_active limit
 * is reached.
 */
TRACE_EVENT(workqueue_activate_work,

	TP_PROTO(struct work_struct *work),

	TP_ARGS(work),

	TP_STRUCT__entry(
		__field( void *,	work	)
		__field( void *,	function)
	),

	TP_fast_assign(
		__entry->work		= work;
		__entry->function	= work->func;
	),

	TP_printk("work struct %p function=%ps ", __entry->work, __entry->function)
);

/**
 * workqueue_execute_start - called immediately before the workqueue callback
 * @work:	pointer to struct work_struct
 *
 * Allows to track workqueue execution.
 */
TRACE_EVENT(workqueue_execute_start,

	TP_PROTO(struct work_struct *work),

	TP_ARGS(work),

	TP_STRUCT__entry(
		__field( void *,	work	)
		__field( void *,	function)
	),

	TP_fast_assign(
		__entry->work		= work;
		__entry->function	= work->func;
	),

	TP_printk("work struct %p: function %ps", __entry->work, __entry->function)
);

/**
 * workqueue_execute_end - called immediately after the workqueue callback
 * @work:	pointer to struct work_struct
 * @function:   pointer to worker function
 *
 * Allows to track workqueue execution.
 */
TRACE_EVENT(workqueue_execute_end,

	TP_PROTO(struct work_struct *work, work_func_t function),

	TP_ARGS(work, function),

	TP_STRUCT__entry(
		__field( void *,	work	)
		__field( void *,	function)
	),

	TP_fast_assign(
		__entry->work		= work;
		__entry->function	= function;
	),

	TP_printk("work struct %p: function %ps", __entry->work, __entry->function)
);

/**
 * workqueue_cpu_intensive - called when a work item exceeds cpu_intensive threshold
 * @pwq:	pointer to struct pool_workqueue
 * @work:	pointer to struct work_struct
 * @function:	pointer to worker function
 * @duration_us: CPU time consumed in microseconds
 *
 * This event occurs when a concurrency-managed work item runs for longer
 * than wq_cpu_intensive_thresh_us without sleeping and is excluded from
 * concurrency management to prevent stalling other work items.
 */
TRACE_EVENT(workqueue_cpu_intensive,

	TP_PROTO(struct pool_workqueue *pwq, struct work_struct *work,
		 work_func_t function, u64 duration_us),

	TP_ARGS(pwq, work, function, duration_us),

	TP_STRUCT__entry(
		__field( void *,	work		)
		__field( void *,	function	)
		__string( workqueue,	pwq->wq->name	)
		__field( int,		cpu		)
		__field( u64,		duration_us	)
	),

	TP_fast_assign(
		__entry->work		= work;
		__entry->function	= function;
		__assign_str(workqueue);
		__entry->cpu		= pwq->pool->cpu;
		__entry->duration_us	= duration_us;
	),

	TP_printk("work struct=%p function=%ps workqueue=%s cpu=%d duration_us=%llu",
		  __entry->work, __entry->function, __get_str(workqueue),
		  __entry->cpu, __entry->duration_us)
);

#endif /*  _TRACE_WORKQUEUE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>

