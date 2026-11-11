/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM alloc_tag

#if !defined(_TRACE_ALLOC_TAG_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ALLOC_TAG_H

#include <linux/tracepoint.h>

/*
 * alloc_tag_hit is generated at the exact allocation call site and can be
 * used to capture a stack trace.
 *
 * To link this stack trace to the actual allocated memory chunk, tools must
 * correlate this event with the resulting alloc_tag_mem_alloced event. Since
 * multiple threads can hit the same tag simultaneously, tools must match BOTH
 * the `tag` field and the implicitly recorded PID provided by the core
 * tracing subsystem.
 */
TRACE_EVENT(alloc_tag_hit,
	TP_PROTO(struct alloc_tag *tag),

	TP_ARGS(tag),

	TP_STRUCT__entry(__field(struct alloc_tag *, tag)
			 __string(modname, tag->ct.modname ? tag->ct.modname : "NONE")
			 __string(filename, tag->ct.filename)
			 __string(function, tag->ct.function)
			 __field(unsigned int, lineno)
	),

	TP_fast_assign(__entry->tag = tag;
		       __assign_str(modname);
		       __assign_str(filename);
		       __assign_str(function);
		       __entry->lineno = tag->ct.lineno;
	),

	TP_printk("tag %p, module: %s, filename: %s, function %s, lineno %u",
		  __entry->tag,
		  __get_str(modname),
		  __get_str(filename),
		  __get_str(function),
		  __entry->lineno
	)
);

/*
 * alloc_tag_mem_alloced is generated after memory is successfully allocated.
 * It captures the exact byte size.
 *
 * The `ptr` value identifies the memory chunk for tracking its lifecycle
 * (e.g., matching it with alloc_tag_mem_freed).
 * - slab and percpu allocators: address of the object's codetag_ref
 * - page allocator: the head struct page of the allocation
 *
 * Because the kernel isolates active allocations within the task struct
 * (current->alloc_tag), this event will always share the same implicit PID as
 * its corresponding alloc_tag_hit event. Tools should use the combination
 * PID + `tag` to correlate them.
 */
TRACE_EVENT(alloc_tag_mem_alloced,
	TP_PROTO(const void *ptr, struct alloc_tag *tag, size_t bytes),

	TP_ARGS(ptr, tag, bytes),

	TP_STRUCT__entry(__field(const void *, ptr)
			 __field(struct alloc_tag *, tag)
			 __field(size_t, bytes)
	),

	TP_fast_assign(__entry->ptr = ptr;
		       __entry->tag = tag;
		       __entry->bytes = bytes;
	),

	TP_printk("ptr %p, tag %p, bytes %zu",
		  __entry->ptr,
		  __entry->tag,
		  __entry->bytes
	)
);

/*
 * alloc_tag_mem_freed event is generated immediately before memory is
 * freed. The `ptr` value matches the one emitted during allocation,
 * allowing tools to match it to its corresponding allocation and
 * call stack.
 *
 * Page allocator caveat: Pages can be split or moved to another folio
 * after the alloc event with no new events generated for that. As a
 * result, a free event may carry a `ptr` that never appeared in an
 * alloc event, and the `bytes` may be smaller than in the matching
 * alloc event. Tools must not assume alloc bytes == free bytes, nor
 * that every free has a matching alloc.
 */
TRACE_EVENT(alloc_tag_mem_freed,
	TP_PROTO(const void *ptr, struct alloc_tag *tag, size_t bytes),

	TP_ARGS(ptr, tag, bytes),

	TP_STRUCT__entry(__field(const void *, ptr)
			 __field(struct alloc_tag *, tag)
			 __field(size_t, bytes)
	),

	TP_fast_assign(__entry->ptr = ptr;
		       __entry->tag = tag;
		       __entry->bytes = bytes;
	),

	TP_printk("ptr %p, tag %p, bytes %zu",
		  __entry->ptr,
		  __entry->tag,
		  __entry->bytes
	)
);

#endif /* _TRACE_ALLOC_TAG_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
