// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 *
 */

#include "intel_de.h"
#include "intel_dip.h"
#include "intel_dip_regs.h"
#include "intel_display_types.h"

static int intel_dip_get_as_sdp_transmission_line(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);

	if (!HAS_EMP_AS_SDP_TL(display))
		return 0;

	/*
	 * EMP_AS_SDP_TL defines the T1 position as the default AS SDP
	 * Transmission Line, which corresponds to the start of the
	 * VSYNC pulse.
	 *
	 * Use the T1 position for now.
	 */
	return crtc_state->vrr.vsync_start;
}

u16 intel_dip_read_emp_as_sdp_tl(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	u32 val;

	if (!HAS_EMP_AS_SDP_TL(display))
		return 0;

	val = intel_de_read(display, EMP_AS_SDP_TL(display, cpu_transcoder));
	return REG_FIELD_GET(EMP_AS_SDP_DB_TL_MASK, val);
}

void intel_dip_write_emp_as_sdp_tl(const struct intel_crtc_state *crtc_state)
{
	struct intel_display *display = to_intel_display(crtc_state);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;

	if (!HAS_EMP_AS_SDP_TL(display))
		return;

	intel_de_write(display,
		       EMP_AS_SDP_TL(display, cpu_transcoder),
		       EMP_AS_SDP_DB_TL(crtc_state->dip.emp_as_sdp_tl));
}

void intel_dip_sdp_tl_compute_config_late(struct intel_crtc_state *crtc_state)
{
	crtc_state->dip.emp_as_sdp_tl = intel_dip_get_as_sdp_transmission_line(crtc_state);
}

void intel_dip_sdp_transmission_line_get_config(struct intel_crtc_state *crtc_state)
{
	crtc_state->dip.emp_as_sdp_tl = intel_dip_read_emp_as_sdp_tl(crtc_state);
}
