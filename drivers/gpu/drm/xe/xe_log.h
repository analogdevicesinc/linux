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

#endif
