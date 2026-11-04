/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_CPER_H_
#define _XE_CPER_H_

#include <linux/types.h>

#include "abi/xe_sigid_abi.h"

struct va_format;
struct xe_device;
struct xe_ras_error_class;
struct xe_ras_get_counter_response;

#if IS_ENABLED(CONFIG_UEFI_CPER)
void xe_cper_emit(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
		  u32 component, u32 location, const void *data, size_t len,
		  struct va_format *vaf);
void xe_cper_emit_counter(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
			  const struct xe_ras_error_class *counter,
			  const struct xe_ras_get_counter_response *response);
int xe_cper_sysfs_init(struct xe_device *xe);
#else
static inline void xe_cper_emit(struct xe_device *xe, int cper_sev, enum xe_sigid sigid,
				u32 component, u32 location, const void *data, size_t len,
				struct va_format *vaf) {}
static inline void xe_cper_emit_counter(struct xe_device *xe, int cper_sev,
					enum xe_sigid sigid,
					const struct xe_ras_error_class *counter,
					const struct xe_ras_get_counter_response *response) {}
static inline int xe_cper_sysfs_init(struct xe_device *xe) { return 0; }
#endif

#endif /* _XE_CPER_H_ */
