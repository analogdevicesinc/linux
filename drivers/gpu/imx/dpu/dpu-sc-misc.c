/*
 * Copyright 2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include "dpu-prv.h"

static inline int
dpu_sc_misc_set_ctrl(struct dpu_soc *dpu, u32 rsc, u8 ctrl, u32 val)
{
	return imx_sc_misc_set_control(dpu->dpu_ipc_handle, rsc, ctrl, val);
}

int dpu_sc_misc_get_handle(struct dpu_soc *dpu)
{
	return imx_scu_get_handle(&dpu->dpu_ipc_handle);
}

int dpu_pxlink_set_mst_addr(struct dpu_soc *dpu, int disp_id, u32 val)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;
	u8 ctrl = disp_id ?
		IMX_SC_C_PXL_LINK_MST2_ADDR : IMX_SC_C_PXL_LINK_MST1_ADDR;

	return dpu_sc_misc_set_ctrl(dpu, rsc, ctrl, val);
}

int dpu_pxlink_set_mst_enable(struct dpu_soc *dpu, int disp_id, bool enable)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;
	u8 ctrl = disp_id ?
		IMX_SC_C_PXL_LINK_MST2_ENB: IMX_SC_C_PXL_LINK_MST1_ENB;

	return dpu_sc_misc_set_ctrl(dpu, rsc, ctrl, enable);
}

int dpu_pxlink_set_mst_valid(struct dpu_soc *dpu, int disp_id, bool enable)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;
	u8 ctrl = disp_id ?
		IMX_SC_C_PXL_LINK_MST2_VLD : IMX_SC_C_PXL_LINK_MST1_VLD;

	return dpu_sc_misc_set_ctrl(dpu, rsc, ctrl, enable);
}

int dpu_pxlink_set_sync_ctrl(struct dpu_soc *dpu, int disp_id, bool enable)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;
	u8 ctrl = disp_id ? IMX_SC_C_SYNC_CTRL1 : IMX_SC_C_SYNC_CTRL0;

	return dpu_sc_misc_set_ctrl(dpu, rsc, ctrl, enable);
}

int dpu_pxlink_set_dc_sync_mode(struct dpu_soc *dpu, bool enable)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;

	return dpu_sc_misc_set_ctrl(dpu, rsc, IMX_SC_C_MODE, enable);
}

/* KACHUNK_CNT is needed for blit engine */
static int dpu_sc_misc_set_kachunk_cnt(struct dpu_soc *dpu, u32 cnt)
{
	u32 rsc = dpu->id ? IMX_SC_R_DC_1 : IMX_SC_R_DC_0;

	return dpu_sc_misc_set_ctrl(dpu, rsc, IMX_SC_C_KACHUNK_CNT, cnt);
}

int dpu_sc_misc_init(struct dpu_soc *dpu)
{
	int disp_id, ret = 0;

	for (disp_id = 0; disp_id < 2; disp_id++) {
		ret |= dpu_pxlink_set_mst_addr(dpu, disp_id, 0);
		ret |= dpu_pxlink_set_mst_enable(dpu, disp_id, false);
		ret |= dpu_pxlink_set_mst_valid(dpu, disp_id, false);
		ret |= dpu_pxlink_set_sync_ctrl(dpu, disp_id, false);
	}

	ret |= dpu_sc_misc_set_kachunk_cnt(dpu, 32);

	return ret;
}
