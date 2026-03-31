// SPDX-License-Identifier: MIT
/* Copyright © 2025 Intel Corporation */

#include <drm/drm_print.h>

#include "intel_display_core.h"
#include "intel_display_types.h"
#include "intel_dpio_phy.h"
#include "vlv_sideband.h"

void vlv_bunit_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_BUNIT));
}

u32 vlv_bunit_read(struct intel_display *display, u32 reg)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_BUNIT, reg);
}

void vlv_bunit_write(struct intel_display *display, u32 reg, u32 val)
{
	vlv_iosf_sb_write(display->drm, VLV_IOSF_SB_BUNIT, reg, val);
}

void vlv_bunit_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_BUNIT));
}

void vlv_cck_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_CCK));
}

u32 vlv_cck_read(struct intel_display *display, u32 reg)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_CCK, reg);
}

void vlv_cck_write(struct intel_display *display, u32 reg, u32 val)
{
	vlv_iosf_sb_write(display->drm, VLV_IOSF_SB_CCK, reg, val);
}

void vlv_cck_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_CCK));
}

void vlv_ccu_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_CCU));
}

u32 vlv_ccu_read(struct intel_display *display, u32 reg)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_CCU, reg);
}

void vlv_ccu_write(struct intel_display *display, u32 reg, u32 val)
{
	vlv_iosf_sb_write(display->drm, VLV_IOSF_SB_CCU, reg, val);
}

void vlv_ccu_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_CCU));
}

void vlv_dpio_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_DPIO) | BIT(VLV_IOSF_SB_DPIO_2));
}

static enum vlv_iosf_sb_unit vlv_dpio_phy_to_unit(struct intel_display *display,
						  enum dpio_phy phy)
{
	/*
	 * IOSF_PORT_DPIO: VLV x2 PHY (DP/HDMI B and C), CHV x1 PHY (DP/HDMI D)
	 * IOSF_PORT_DPIO_2: CHV x2 PHY (DP/HDMI B and C)
	 */
	if (display->platform.cherryview)
		return phy == DPIO_PHY0 ? VLV_IOSF_SB_DPIO_2 : VLV_IOSF_SB_DPIO;
	else
		return VLV_IOSF_SB_DPIO;
}

u32 vlv_dpio_read(struct intel_display *display, enum dpio_phy phy, int reg)
{
	enum vlv_iosf_sb_unit unit = vlv_dpio_phy_to_unit(display, phy);
	u32 val;

	val = vlv_iosf_sb_read(display->drm, unit, reg);

	/*
	 * FIXME: There might be some registers where all 1's is a valid value,
	 * so ideally we should check the register offset instead...
	 */
	drm_WARN(display->drm, val == 0xffffffff,
		 "DPIO PHY%d read reg 0x%x == 0x%x\n",
		 phy, reg, val);

	return val;
}

void vlv_dpio_write(struct intel_display *display,
		    enum dpio_phy phy, int reg, u32 val)
{
	enum vlv_iosf_sb_unit unit = vlv_dpio_phy_to_unit(display, phy);

	vlv_iosf_sb_write(display->drm, unit, reg, val);
}

void vlv_dpio_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_DPIO) | BIT(VLV_IOSF_SB_DPIO_2));
}

void vlv_flisdsi_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_FLISDSI));
}

u32 vlv_flisdsi_read(struct intel_display *display, u32 reg)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_FLISDSI, reg);
}

void vlv_flisdsi_write(struct intel_display *display, u32 reg, u32 val)
{
	vlv_iosf_sb_write(display->drm, VLV_IOSF_SB_FLISDSI, reg, val);
}

void vlv_flisdsi_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_FLISDSI));
}

void vlv_nc_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_NC));
}

u32 vlv_nc_read(struct intel_display *display, u8 addr)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_NC, addr);
}

void vlv_nc_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_NC));
}

void vlv_punit_get(struct intel_display *display)
{
	vlv_iosf_sb_get(display->drm, BIT(VLV_IOSF_SB_PUNIT));
}

u32 vlv_punit_read(struct intel_display *display, u32 addr)
{
	return vlv_iosf_sb_read(display->drm, VLV_IOSF_SB_PUNIT, addr);
}

int vlv_punit_write(struct intel_display *display, u32 addr, u32 val)
{
	return vlv_iosf_sb_write(display->drm, VLV_IOSF_SB_PUNIT, addr, val);
}

void vlv_punit_put(struct intel_display *display)
{
	vlv_iosf_sb_put(display->drm, BIT(VLV_IOSF_SB_PUNIT));
}
