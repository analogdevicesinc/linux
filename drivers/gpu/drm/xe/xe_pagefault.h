/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2025 Intel Corporation
 */

#ifndef _XE_PAGEFAULT_H_
#define _XE_PAGEFAULT_H_

#include "xe_pagefault_types.h"

struct xe_device;
struct xe_gt;
struct xe_pagefault;

int xe_pagefault_init(struct xe_device *xe);

void xe_pagefault_reset(struct xe_device *xe, struct xe_gt *gt);

int xe_pagefault_handler(struct xe_device *xe, struct xe_pagefault *pf);

#define XE_PAGEFAULT_END_ADDR_MASK	(~0xfffull)

/**
 * xe_pagefault_set_end_addr() - store serviced range end for a pagefault
 * @pf: Pagefault entry
 * @end_addr: Inclusive end address of the serviced fault range
 *
 * The pagefault consumer stores the resolved fault range so subsequent faults
 * hitting the same range can be immediately acknowledged without re-running
 * the full fault handling path.
 *
 * The end address shares storage with other consumer metadata and therefore
 * must be masked with %XE_PAGEFAULT_END_ADDR_MASK before storing. Bits outside
 * the mask are reserved for internal state tracking and must be preserved.
 */
static inline void
xe_pagefault_set_end_addr(struct xe_pagefault *pf, u64 end_addr)
{
	pf->consumer.end_addr &= ~XE_PAGEFAULT_END_ADDR_MASK;
	pf->consumer.end_addr |= end_addr;
}

/**
 * xe_pagefault_end_addr() - read serviced range end for a pagefault
 * @pf: Pagefault entry
 *
 * Returns the inclusive end address of the range previously recorded by
 * xe_pagefault_set_end_addr(). Only the bits covered by
 * %XE_PAGEFAULT_END_ADDR_MASK are returned; other bits in the storage are
 * reserved for internal state.
 *
 * Return: End address of the serviced fault range.
 */
static inline u64 xe_pagefault_end_addr(struct xe_pagefault *pf)
{
	return pf->consumer.end_addr & XE_PAGEFAULT_END_ADDR_MASK;
}

#undef XE_PAGEFAULT_END_ADDR_MASK

/**
 * xe_pagefault_set_start_addr() - store serviced range start for a pagefault
 * @pf: Pagefault entry
 * @start_addr: Start address of the serviced fault range
 *
 * The pagefault consumer stores the resolved fault range so subsequent faults
 * hitting the same range can be immediately acknowledged without re-running
 * the full fault handling path.
 */
static inline void
xe_pagefault_set_start_addr(struct xe_pagefault *pf, u64 start_addr)
{
	pf->consumer.page_addr = start_addr;
}

/**
 * xe_pagefault_start_addr() - read serviced range start for a pagefault
 * @pf: Pagefault entry
 *
 * Returns the inclusive start address of the range previously recorded by
 * xe_pagefault_set_start_addr().
 *
 * Return: Start address of the serviced fault range.
 */
static inline u64 xe_pagefault_start_addr(struct xe_pagefault *pf)
{
	return pf->consumer.page_addr;
}

#endif
