/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef __INTEL_DIP_REGS_H__
#define __INTEL_DIP_REGS_H__

#include "intel_display_reg_defs.h"

/* EMP (Extended Metadata Packet) AS (Adaptive Sync) SDP Transmission Line */
#define _EMP_AS_SDP_TL_A			0x60204
#define EMP_AS_SDP_TL(display, trans)		_MMIO_TRANS2((display), (trans), _EMP_AS_SDP_TL_A)
#define   EMP_AS_SDP_DB_TL_MASK			REG_GENMASK(12, 0)
#define   EMP_AS_SDP_DB_TL(db_transmit_line)	REG_FIELD_PREP(EMP_AS_SDP_DB_TL_MASK, \
							       (db_transmit_line))

#endif /* __INTEL_DIP_REGS_H__ */
