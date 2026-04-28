// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Intel Corporation
 */

#include <drm/intel/intel_gmd_misc_regs.h>

#include "intel_de.h"
#include "intel_display_clock_gating.h"
#include "intel_display_regs.h"

void intel_display_skl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaFbcTurnOffFbcWatermark:skl
	 * Display WA #0562: skl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}

void intel_display_kbl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaFbcTurnOffFbcWatermark:kbl
	 * Display WA #0562: kbl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}
