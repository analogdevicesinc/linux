/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "imx-hdp.h"

void imx_arc_power_up(state_struct *state)
{
	DRM_DEBUG("arc_power_up()\n");

	/* register CMN_RXCAL_OVRD */
	Afe_write(state, 0x5025, 0x0001);
}

void imx_arc_calibrate(state_struct *state)
{
	u16 txpu_calib_code;
	u16 txpd_calib_code;
	u16 txpu_adj_calib_code;
	u16 txpd_adj_calib_code;
	u16 prev_calib_code;
	u16 new_calib_code;
	u16 rdata;

	DRM_DEBUG("aux_cal_cfg() ARC programming\n");
	/* register TX_DIG_CTRL_REG_2 */
	prev_calib_code = Afe_read(state, 0x5024);
	/* register CMN_TXPUCAL_CTRL */
	txpu_calib_code = Afe_read(state, 0x00E0);
	/* register CMN_TXPDCAL_CTRL */
	txpd_calib_code = Afe_read(state, 0x00F0);
	/* register CMN_TXPU_ADJ_CTRL */
	txpu_adj_calib_code = Afe_read(state, 0x0108);
	/* register CMN_TXPD_ADJ_CTRL */
	txpd_adj_calib_code = Afe_read(state, 0x010c);

	new_calib_code = ((txpu_calib_code + txpd_calib_code) / 2)
		+ txpu_adj_calib_code + txpd_adj_calib_code;

	if (new_calib_code != prev_calib_code) {
		/* register TX_ANA_CTRL_REG_1 */
		rdata = Afe_read(state, 0x5020);
		rdata &= 0xDFFF;
		/* register TX_ANA_CTRL_REG_1 */
		Afe_write(state, 0x5020, rdata);
		/* register TX_DIG_CTRL_REG_2 */
		Afe_write(state, 0x5024, new_calib_code);
		mdelay(10);
		rdata |= 0x2000;
		/* register TX_ANA_CTRL_REG_1 */
		Afe_write(state, 0x5020, rdata);
		udelay(150);
	}
}

void imx_arc_config(state_struct *state)
{
	DRM_DEBUG("arc_config() ARC programming\n");

	/* register TX_ANA_CTRL_REG_2 */
	Afe_write(state, 0x5021, 0x0100);
	udelay(100);
	/* register TX_ANA_CTRL_REG_2 */
	Afe_write(state, 0x5021, 0x0300);
	udelay(100);
	/* register TX_ANA_CTRL_REG_3 */
	Afe_write(state, 0x5026, 0x0000);
	udelay(100);
	/* register TX_ANA_CTRL_REG_1 */
	Afe_write(state, 0x5020, 0x2008);
	udelay(100);
	/* register TX_ANA_CTRL_REG_1 */
	Afe_write(state, 0x5020, 0x2018);
	udelay(100);
	/* register TX_ANA_CTRL_REG_1 */
	Afe_write(state, 0x5020, 0x2098);
	/* register TX_ANA_CTRL_REG_2 */
	Afe_write(state, 0x5021, 0x030C);
	/* register TX_ANA_CTRL_REG_5 */
	Afe_write(state, 0x5029, 0x0010);
	udelay(100);
	/* register TX_ANA_CTRL_REG_4 */
	Afe_write(state, 0x5027, 0x4001);
	mdelay(5);
	/* register TX_ANA_CTRL_REG_1 */
	Afe_write(state, 0x5020, 0x2198);
	mdelay(5);
	/* register TX_ANA_CTRL_REG_2 */
	Afe_write(state, 0x5021, 0x030D);
	udelay(100);
	Afe_write(state, 0x5021, 0x030F);
}

