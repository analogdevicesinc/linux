/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_LOG_H_
#define _XE_LOG_H_

#include <linux/cper.h>

#include "abi/xe_sigid_abi.h"

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

#endif
