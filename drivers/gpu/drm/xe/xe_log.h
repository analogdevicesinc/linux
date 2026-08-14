/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_LOG_H_
#define _XE_LOG_H_

#include <linux/cper.h>

#include "abi/xe_log_abi.h"
#include "abi/xe_sigid_abi.h"
#include "xe_any.h"

struct pci_dev;

__printf(8, 9)
void xe_log_emit(struct pci_dev *pdev, int cper_sev, enum xe_sigid sigid,
		 u32 component, u32 location, const void *data, size_t len,
		 const char *fmt, ...);

#define xe_log_emit_fatal(pdev, sig, comp, loc, data, len, fmt, args...) \
	xe_log_emit((pdev), CPER_SEV_FATAL, (sig), (comp), (loc), \
		    (data), (len), fmt, ##args)

#define xe_log_emit_recoverable(pdev, sig, comp, loc, data, len, fmt, args...) \
	xe_log_emit((pdev), CPER_SEV_RECOVERABLE, (sig), (comp), (loc), \
		    (data), (len), fmt, ##args)

#define xe_log_emit_corrected(pdev, sig, comp, loc, data, len, fmt, args...) \
	xe_log_emit((pdev), CPER_SEV_CORRECTED, (sig), (comp), (loc), \
		    (data), (len), fmt, ##args)

#define xe_log_emit_info(pdev, sig, comp, loc, data, len, fmt, args...) \
	xe_log_emit((pdev), CPER_SEV_INFORMATIONAL, (sig), (comp), (loc), \
		    (data), (len), fmt, ##args)

#define xe_log_location_type(any)							\
	_Generic((any),									\
		 struct xe_gt * : XE_LOG_LOCATION_TYPE_GT,				\
		 const struct xe_gt * : XE_LOG_LOCATION_TYPE_GT,			\
		 struct xe_tile * : XE_LOG_LOCATION_TYPE_TILE,				\
		 const struct xe_tile * : XE_LOG_LOCATION_TYPE_TILE,			\
		 struct xe_device * : XE_LOG_LOCATION_TYPE_DEVICE,			\
		 const struct xe_device * : XE_LOG_LOCATION_TYPE_DEVICE,		\
		 struct pci_dev * : XE_LOG_LOCATION_TYPE_DEVICE,			\
		 struct device * : XE_LOG_LOCATION_TYPE_DEVICE)

#define xe_log_location(any) \
	PREP_XE_LOG_LOCATION(xe_log_location_type(any), xe_any_id(any))

/**
 * xe_log_from() - Emit a structured SIGID log entry using @any pointer as location.
 * @any: the &xe_device or &xe_tile or &xe_gt pointer this report relates to
 * @cper_sev: CPER severity (CPER_SEV_FATAL, CPER_SEV_RECOVERABLE, ...)
 * @sigid: signature identifier, see &enum xe_sigid
 * @component: component identifer
 * @data: pointer to the additional details, or ERR_PTR, or NULL if not applicable
 * @len: length of the @data in bytes, or 0 if not applicable
 * @fmt: printf-style format string
 * @args: arguments for the @fmt format string
 *
 * The location used to emit SIGID entry will be based on the @any pointer type.
 * See xe_log_emit() for more details.
 */
#define xe_log_from(any, cper_sev, sigid, component, data, len, fmt, args...) do {	\
	typeof(any) ___any = (any);							\
	xe_log_emit(xe_any_to_pdev(___any), (cper_sev), (sigid), (component),		\
		    xe_log_location(___any), (data), (len), fmt, ##args);		\
} while (0)

#define xe_log_from_fatal(any, sig, comp, data, len, fmt, args...) \
	xe_log_from((any), CPER_SEV_FATAL, (sig), (comp), \
		    (data), (len), fmt, ##args)

#define xe_log_from_recoverable(any, sig, comp, data, len, fmt, args...) \
	xe_log_from((any), CPER_SEV_RECOVERABLE, (sig), (comp), \
		    (data), (len), fmt, ##args)

#define xe_log_from_corrected(any, sig, comp, data, len, fmt, args...) \
	xe_log_from((any), CPER_SEV_CORRECTED, (sig), (comp), \
		    (data), (len), fmt, ##args)

#define xe_log_from_info(any, sig, comp, data, len, fmt, args...) \
	xe_log_from((any), CPER_SEV_INFORMATIONAL, (sig), (comp), \
		    (data), (len), fmt, ##args)

/**
 * xe_log_comp() - Emit a structured SIGID log entry on the component behalf.
 * @any: the &xe_device or &xe_tile or &xe_gt pointer this report relates to
 * @cper_sev: CPER severity (CPER_SEV_FATAL, CPER_SEV_RECOVERABLE, ...)
 * @TAG: the component tag to use
 * @data: pointer to the additional details, or ERR_PTR, or NULL if not applicable
 * @len: length of the @data in bytes, or 0 if not applicable
 * @fmt: printf-style free text format string (not a stable interface)
 * @args: arguments for the @fmt format string
 *
 * The SIGID will be determined from the component's @TAG.
 * The component identifier will be determined from the component's @TAG.
 * The location used to emit SIGID entry will be based on the @any pointer type.
 */
#define xe_log_comp(any, cper_sev, TAG, data, len, fmt, args...) \
	xe_log_from((any), (cper_sev), (int)XE_LOG_COMPONENT_##TAG##_SIGID, \
		    XE_LOG_COMPONENT_##TAG, (data), (len), fmt, ##args)

#define xe_log_comp_fatal(any, TAG, data, len, fmt, args...) \
	xe_log_comp((any), CPER_SEV_FATAL, TAG, (data), (len), fmt, ##args)

#define xe_log_comp_recoverable(any, TAG, data, len, fmt, args...) \
	xe_log_comp((any), CPER_SEV_RECOVERABLE, TAG, (data), (len), fmt, ##args)

#define xe_log_comp_corrected(any, TAG, data, len, fmt, args...) \
	xe_log_comp((any), CPER_SEV_CORRECTED, TAG, (data), (len), fmt, ##args)

#define xe_log_comp_info(any, TAG, data, len, fmt, args...) \
	xe_log_comp((any), CPER_SEV_INFORMATIONAL, TAG, (data), (len), fmt, ##args)

#endif
