/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2026, Microsoft Corporation.
 *
 * Tracepoint declarations for mshv driver.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mshv

#if !defined(__MSHV_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _MSHV_TRACE_H_

#include <linux/tracepoint.h>

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/hv

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE mshv_trace

TRACE_EVENT(mshv_create_partition,
	    TP_PROTO(u64 partition_id, int vm_fd),
	    TP_ARGS(partition_id, vm_fd),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(int, vm_fd)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vm_fd = vm_fd;
	    ),
	    TP_printk("partition_id=%llu vm_fd=%d",
		    __entry->partition_id,
		    __entry->vm_fd
	    )
);

TRACE_EVENT(mshv_hvcall_create_partition,
	    TP_PROTO(u64 flags, s64 partition_id),
	    TP_ARGS(flags, partition_id),
	    TP_STRUCT__entry(
		    __field(u64, flags)
		    __field(s64, partition_id)
	    ),
	    TP_fast_assign(
		    __entry->flags = flags;
		    __entry->partition_id = partition_id;
	    ),
	    TP_printk("flags=%#llx partition_id=%lld",
		    __entry->flags,
		    __entry->partition_id
	    )
);

TRACE_EVENT(mshv_hvcall_initialize_partition,
	    TP_PROTO(u64 partition_id, u64 status),
	    TP_ARGS(partition_id, status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu status=%#llx",
		    __entry->partition_id,
		    __entry->status
	    )
);

TRACE_EVENT(mshv_partition_release,
	    TP_PROTO(u64 partition_id),
	    TP_ARGS(partition_id),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
	    ),
	    TP_printk("partition_id=%llu",
		    __entry->partition_id
	    )
);

TRACE_EVENT(mshv_destroy_partition,
	    TP_PROTO(u64 partition_id),
	    TP_ARGS(partition_id),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
	    ),
	    TP_printk("partition_id=%llu",
		    __entry->partition_id
	    )
);

TRACE_EVENT(mshv_hvcall_finalize_partition,
	    TP_PROTO(u64 partition_id, u64 status),
	    TP_ARGS(partition_id, status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu status=%#llx ",
		    __entry->partition_id,
		    __entry->status
	    )
);

TRACE_EVENT(mshv_hvcall_withdraw_memory,
	    TP_PROTO(u64 partition_id, u64 withdrawn, u64 status),
	    TP_ARGS(partition_id, withdrawn, status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, withdrawn)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->withdrawn = withdrawn;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu withdrawn=%llu status=%#llx",
		    __entry->partition_id,
		    __entry->withdrawn,
		    __entry->status
	    )
);

TRACE_EVENT(mshv_hvcall_delete_partition,
	    TP_PROTO(u64 partition_id, u64 status),
	    TP_ARGS(partition_id, status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu status=%#llx",
		    __entry->partition_id,
		    __entry->status
	    )
);

TRACE_EVENT(mshv_create_vp,
	    TP_PROTO(u64 partition_id, u32 vp_index, long vp_fd),
	    TP_ARGS(partition_id, vp_index, vp_fd),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(long, vp_fd)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->vp_fd = vp_fd;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u vp_fd=%ld",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->vp_fd
	    )
);

TRACE_EVENT(mshv_hvcall_map_vp_state_page,
	    TP_PROTO(u64 partition_id, u32 vp_index, u32 page_type, u64 status),
	    TP_ARGS(partition_id, vp_index, page_type, status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(u32, page_type)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->page_type = page_type;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u page_type=%u status=%#llx",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->page_type,
		    __entry->status
	    )
);

TRACE_EVENT(mshv_drain_vp_signals,
	    TP_PROTO(u64 partition_id, u32 vp_index),
	    TP_ARGS(partition_id, vp_index),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u",
		    __entry->partition_id,
		    __entry->vp_index
	    )
);

TRACE_EVENT(mshv_disable_vp_dispatch,
	    TP_PROTO(u64 partition_id, u32 vp_index, int ret),
	    TP_ARGS(partition_id, vp_index, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(int, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u ret=%d",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->ret
	    )
);

TRACE_EVENT(mshv_vp_release,
	    TP_PROTO(u64 partition_id, u32 vp_index),
	    TP_ARGS(partition_id, vp_index),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u",
		    __entry->partition_id,
		    __entry->vp_index
	    )
);

TRACE_EVENT(mshv_run_vp_entry,
	    TP_PROTO(u64 partition_id, u32 vp_index),
	    TP_ARGS(partition_id, vp_index),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u",
		    __entry->partition_id,
		    __entry->vp_index
	    )
);

TRACE_EVENT(mshv_run_vp_exit,
	    TP_PROTO(u64 partition_id, u32 vp_index, u64 hv_message_type, long ret),
	    TP_ARGS(partition_id, vp_index, hv_message_type, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(u64, hv_message_type)
		    __field(long, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->hv_message_type = hv_message_type;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u hv_message_type=%#llx ret=%ld",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->hv_message_type,
		    __entry->ret
	    )
);

TRACE_EVENT(mshv_vp_clear_explicit_suspend,
	    TP_PROTO(u64 partition_id, u32 vp_index, int ret),
	    TP_ARGS(partition_id, vp_index, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(int, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u ret=%d",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->ret
	    )
);

TRACE_EVENT(mshv_xfer_to_guest_mode_work,
	    TP_PROTO(u64 partition_id, u32 vp_index, unsigned long thread_info_flag, long ret),
	    TP_ARGS(partition_id, vp_index, thread_info_flag, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(unsigned long, thread_info_flag)
		    __field(long, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->thread_info_flag = thread_info_flag;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u thread_info_flag=%#lx ret=%ld",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->thread_info_flag,
		    __entry->ret
	    )
);

TRACE_EVENT(mshv_hvcall_dispatch_vp,
	    TP_PROTO(u64 partition_id, u32 vp_index, u32 flags,
		     u32 dispatch_state, u32 dispatch_event, u64 irq_vectors, u64 status),
	    TP_ARGS(partition_id, vp_index, flags, dispatch_state, dispatch_event, irq_vectors,
		    status),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(u32, flags)
		    __field(u32, dispatch_state)
		    __field(u32, dispatch_event)
		    __field(u64, irq_vectors)
		    __field(u64, status)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->flags = flags;
		    __entry->dispatch_state = dispatch_state;
		    __entry->dispatch_event = dispatch_event;
		    __entry->irq_vectors = irq_vectors;
		    __entry->status = status;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u flags=%#x dispatch_state=%#x dispatch_event=%#x irq_vectors=%#016llx status=%#llx",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->flags,
		    __entry->dispatch_state,
		    __entry->dispatch_event,
		    __entry->irq_vectors,
		    __entry->status
	     )
);

TRACE_EVENT(mshv_update_routing_table,
	    TP_PROTO(u64 partition_id, void *old, void *new, u32 numents),
	    TP_ARGS(partition_id, old, new, numents),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(struct mshv_girq_routing_table *, old)
		    __field(struct mshv_girq_routing_table *, new)
		    __field(u32, numents)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->old = old;
		    __entry->new = new;
		    __entry->numents = numents;
	    ),
	    TP_printk("partition_id=%llu old=%p new=%p numents=%u",
		    __entry->partition_id,
		    __entry->old,
		    __entry->new,
		    __entry->numents
	    )
);

TRACE_EVENT(mshv_map_user_memory,
	    TP_PROTO(u64 partition_id, u64 start_uaddr, u64 start_gfn, u64 nr_pages, u32 map_flags,
		     long ret),
	    TP_ARGS(partition_id, start_uaddr, start_gfn, nr_pages, map_flags, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, start_uaddr)
		    __field(u64, start_gfn)
		    __field(u64, nr_pages)
		    __field(u32, map_flags)
		    __field(long, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->start_uaddr = start_uaddr;
		    __entry->start_gfn = start_gfn;
		    __entry->nr_pages = nr_pages;
		    __entry->map_flags = map_flags;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu start_uaddr=%#llx start_gfn=%#llx nr_pages=%llu map_flags=%#x ret=%ld",
		    __entry->partition_id,
		    __entry->start_uaddr,
		    __entry->start_gfn,
		    __entry->nr_pages,
		    __entry->map_flags,
		    __entry->ret
	     )
);

TRACE_EVENT(mshv_assign_ioeventfd,
	    TP_PROTO(u64 partition_id, u64 addr, u64 length, u64 datamatch, bool wildcard,
		     void *eventfd, int ret),
	    TP_ARGS(partition_id, addr, length, datamatch, wildcard, eventfd, ret),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, addr)
		    __field(u64, length)
		    __field(u64, datamatch)
		    __field(bool, wildcard)
		    __field(struct eventfd_ctx *, eventfd)
		    __field(int, ret)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->addr = addr;
		    __entry->length = length;
		    __entry->datamatch = datamatch;
		    __entry->wildcard = wildcard;
		    __entry->eventfd = eventfd;
		    __entry->ret = ret;
	    ),
	    TP_printk("partition_id=%llu addr=%#016llx length=%#llx datamatch=%#llx wildcard=%d eventfd=%p ret=%d",
		    __entry->partition_id,
		    __entry->addr,
		    __entry->length,
		    __entry->datamatch,
		    __entry->wildcard,
		    __entry->eventfd,
		    __entry->ret
	     )
);

TRACE_EVENT(mshv_deassign_ioeventfd,
	    TP_PROTO(u64 partition_id, u64 addr, u64 length, u64 datamatch, bool wildcard,
		     void *eventfd),
	    TP_ARGS(partition_id, addr, length, datamatch, wildcard, eventfd),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u64, addr)
		    __field(u64, length)
		    __field(u64, datamatch)
		    __field(bool, wildcard)
		    __field(struct eventfd_ctx *, eventfd)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->addr = addr;
		    __entry->length = length;
		    __entry->datamatch = datamatch;
		    __entry->wildcard = wildcard;
		    __entry->eventfd = eventfd;
	    ),
	    TP_printk("partition_id=%llu addr=%#016llx length=%#llx datamatch=%#llx wildcard=%d eventfd=%p",
		    __entry->partition_id,
		    __entry->addr,
		    __entry->length,
		    __entry->datamatch,
		    __entry->wildcard,
		    __entry->eventfd
	     )
);

TRACE_EVENT(mshv_vp_wait_for_hv_kick,
	    TP_PROTO(u64 partition_id, u32 vp_index, bool kicked_by_hv, bool blocked,
		     bool irq_pending),
	    TP_ARGS(partition_id, vp_index, kicked_by_hv, blocked, irq_pending),
	    TP_STRUCT__entry(
		    __field(u64, partition_id)
		    __field(u32, vp_index)
		    __field(bool, kicked_by_hv)
		    __field(bool, blocked)
		    __field(bool, irq_pending)
	    ),
	    TP_fast_assign(
		    __entry->partition_id = partition_id;
		    __entry->vp_index = vp_index;
		    __entry->kicked_by_hv = kicked_by_hv;
		    __entry->blocked = blocked;
		    __entry->irq_pending = irq_pending;
	    ),
	    TP_printk("partition_id=%llu vp_index=%u kicked_by_hv=%d blocked=%d irq_pending=%d",
		    __entry->partition_id,
		    __entry->vp_index,
		    __entry->kicked_by_hv,
		    __entry->blocked,
		    __entry->irq_pending
	    )
);

#endif /* _MSHV_TRACE_H_ */

/* This part must be outside protection */
#include <trace/define_trace.h>
