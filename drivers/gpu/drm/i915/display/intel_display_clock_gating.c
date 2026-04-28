// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Intel Corporation
 */

#include <drm/intel/intel_gmd_misc_regs.h>

#include "intel_de.h"
#include "intel_display.h"
#include "intel_display_clock_gating.h"
#include "intel_display_core.h"
#include "intel_display_regs.h"

static void intel_display_gen9_init_clock_gating(struct intel_display *display)
{
	/* See Bspec note for PSR2_CTL bit 31, Wa#828:skl,bxt,kbl,cfl */
	intel_de_rmw(display, CHICKEN_PAR1_1, 0, SKL_EDP_PSR_FIX_RDWRAP);

	/* WaEnableChickenDCPR:skl,bxt,kbl,glk,cfl */
	intel_de_rmw(display, GEN8_CHICKEN_DCPR_1, 0, MASK_WAKEMEM);

	/*
	 * WaFbcWakeMemOn:skl,bxt,kbl,glk,cfl
	 * Display WA #0859: skl,bxt,kbl,glk,cfl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_MEMORY_WAKE);
}

void intel_display_skl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaCompressedResourceDisplayNewHashMode:skl,kbl
	 * Display WA #0390: skl,kbl
	 *
	 * Must match Sampler, Pixel Back End, and Media. See
	 * WaCompressedResourceSamplerPbeMediaNewHashMode.
	 */
	intel_de_rmw(display, CHICKEN_PAR1_1, 0, SKL_DE_COMPRESSED_HASH_MODE);

	intel_display_gen9_init_clock_gating(display);

	/*
	 * WaFbcTurnOffFbcWatermark:skl
	 * Display WA #0562: skl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}

void intel_display_kbl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaCompressedResourceDisplayNewHashMode:skl,kbl
	 * Display WA #0390: skl,kbl
	 *
	 * Must match Sampler, Pixel Back End, and Media. See
	 * WaCompressedResourceSamplerPbeMediaNewHashMode.
	 */
	intel_de_rmw(display, CHICKEN_PAR1_1, 0, SKL_DE_COMPRESSED_HASH_MODE);

	intel_display_gen9_init_clock_gating(display);

	/*
	 * WaFbcTurnOffFbcWatermark:kbl
	 * Display WA #0562: kbl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}

void intel_display_cfl_init_clock_gating(struct intel_display *display)
{
	/*
	 * WaCompressedResourceDisplayNewHashMode:skl,kbl (and cfl, cml)
	 * Display WA #0390: skl,kbl (and cfl, cml)
	 *
	 * Must match Sampler, Pixel Back End, and Media. See
	 * WaCompressedResourceSamplerPbeMediaNewHashMode.
	 *
	 * NOTE: this is the same workaround used for skl and kbl,
	 * because the original implementation was checking HAS_LLC(),
	 * which cfl/cml have, even though the comment for the
	 * workaround doesn't mention it.
	 *
	 */
	intel_de_rmw(display, CHICKEN_PAR1_1, 0, SKL_DE_COMPRESSED_HASH_MODE);

	intel_display_gen9_init_clock_gating(display);

	/*
	 * WaFbcTurnOffFbcWatermark:cfl
	 * Display WA #0562: cfl
	 */
	intel_de_rmw(display, DISP_ARB_CTL, 0, DISP_FBC_WM_DIS);
}

void intel_display_bxt_init_clock_gating(struct intel_display *display)
{
	intel_display_gen9_init_clock_gating(display);

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

void intel_display_glk_init_clock_gating(struct intel_display *display)
{
	intel_display_gen9_init_clock_gating(display);

	/*
	 * WaDisablePWMClockGating:glk
	 * Backlight PWM may stop in the asserted state, causing backlight
	 * to stay fully on.
	 */
	intel_de_write(display, GEN9_CLKGATE_DIS_0,
		       intel_de_read(display, GEN9_CLKGATE_DIS_0) |
		       PWM1_GATING_DIS | PWM2_GATING_DIS);
}
