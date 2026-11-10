/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright © 2026 Intel Corporation
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM xe

#if !defined(_XE_TRACE_CPER_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _XE_TRACE_CPER_H_

#include <linux/tracepoint.h>
#include <linux/types.h>

#include "xe_cper_types.h"
#include "xe_device_types.h"

#define __dev_name_xe(xe)	dev_name((xe)->drm.dev)

TRACE_EVENT(xe_error_cper,
	    TP_PROTO(struct xe_device *xe,
		     const guid_t *platform_id, const guid_t *fru_id,
		     const u8 severity,
		     const struct xe_cper_intel_hdr *ihdr,
		     u32 cper_len, const u8 *cper),
	    TP_ARGS(xe, platform_id, fru_id, severity, ihdr, cper_len, cper),

	    TP_STRUCT__entry(
		__string(dev, __dev_name_xe(xe))
		__array(char, platform_id, UUID_SIZE)
		__array(char, fru_id, UUID_SIZE)
		__field(u8, sev)
		__array(u8, ihdr_raw, sizeof(struct xe_cper_intel_hdr))
		__field(u32, cper_len)
		__dynamic_array(u8, cper, cper_len)
	    ),

	    TP_fast_assign(
		__assign_str(dev);
		__entry->sev = severity;
		memcpy(__entry->platform_id, platform_id, UUID_SIZE);
		memcpy(__entry->fru_id, fru_id, UUID_SIZE);
		memcpy(__entry->ihdr_raw, ihdr, sizeof(struct xe_cper_intel_hdr));
		__entry->cper_len = cper_len;
		memcpy(__get_dynamic_array(cper), cper, cper_len);
	    ),

	    TP_printk("dev=%s severity=%d platform_id=%pUl fru_id=%pUl intel_hdr_raw=%s cper_len=%u cper_raw=%s",
		      __get_str(dev), __entry->sev,
		      __entry->platform_id, __entry->fru_id,
		      __print_hex(__entry->ihdr_raw,
				  sizeof(struct xe_cper_intel_hdr)),
		      __entry->cper_len,
		      __print_hex(__get_dynamic_array(cper),
				  __entry->cper_len))
);

#endif

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../drivers/gpu/drm/xe
#define TRACE_INCLUDE_FILE xe_trace_cper
#include <trace/define_trace.h>
