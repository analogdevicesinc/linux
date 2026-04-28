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

void intel_display_cfl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaFbcTurnOffFbcWatermark:cfl
	 * Display WA #0562: cfl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}

void intel_display_bxt_init_clock_gating(struct intel_display *display)
{
	/*
	 * Wa: Backlight PWM may stop in the asserted state, causing backlight
	 * to stay fully on.
	 */
	intel_de_write(display, GEN9_CLKGATE_DIS_0,
		       intel_de_read(display, GEN9_CLKGATE_DIS_0) |
		       PWM1_GATING_DIS | PWM2_GATING_DIS);

	/*
	 * Lower the display internal timeout.
	 * This is needed to avoid any hard hangs when DSI port PLL
	 * is off and a MMIO access is attempted by any privilege
	 * application, using batch buffers or any other means.
	 */
	intel_de_write(display, RM_TIMEOUT, MMIO_TIMEOUT_US(950));

	/*
	 * WaFbcTurnOffFbcWatermark:bxt
	 * Display WA #0562: bxt
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}
