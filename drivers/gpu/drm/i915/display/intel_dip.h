/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef __INTEL_DIP_H__
#define __INTEL_DIP_H__

#include "intel_display_device.h"

/*
 * Video DIP (Data Island Packet) helpers.
 *
 * This file contains helpers for programming video DIP related hardware.
 *
 * TODO: Currently, this is only used for programming EMP_AS_SDP_TL i.e. to
 * program Transmission Line for HDMI 2.1 Extended Metadata Packet (EMP) and
 * DP Adaptive Sync (AS) Secondary Data Packet (SDP). However, all low level
 * DIP buffer read/write and related helpers should be extracted here later.
 */

struct intel_crtc_state;

/*
 * EMP AS SDP TL: Extended Metadata Packet (EMP) Adaptive Sync (AS)
 * Secondary Data Packet (SDP) Transmission Line (TL).
 *
 * Starting with BMG (display ver 14.01) and LNL+ (display ver 20+),
 * the AS SDP transmission line is programmable via the EMP AS SDP TL
 * register.
 */
#define HAS_EMP_AS_SDP_TL(__display)	(DISPLAY_VERx100(__display) == 1401 || \
					 DISPLAY_VER(__display) >= 20)

u16 intel_dip_read_emp_as_sdp_tl(const struct intel_crtc_state *crtc_state);
void intel_dip_write_emp_as_sdp_tl(const struct intel_crtc_state *crtc_state);

#endif /* __INTEL_DIP_H__ */
