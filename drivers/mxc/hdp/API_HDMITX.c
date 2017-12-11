/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017 NXP
 *
 ******************************************************************************
 *
 * API_HDMITX.c
 *
 ******************************************************************************
 */

#include "API_HDMITX.h"
#include "util.h"
#include "opcodes.h"
#include "mhl_hdtx_top.h"
#include "source_phy.h"
#include "address.h"
#include "source_car.h"
#include "source_vif.h"
#include <soc/imx8/soc.h>

CDN_API_STATUS CDN_API_HDMITX_DDC_READ(state_struct *state,
				       HDMITX_TRANS_DATA *data_in,
				       HDMITX_TRANS_DATA *data_out)
{
	internal_macro_command_txrx(state, MB_MODULE_ID_HDMI_TX, HDMI_TX_READ,
				    CDN_BUS_TYPE_APB, 3, 1, data_in->slave, 1,
				    data_in->offset, 2, data_in->len);
	internal_readmsg(state, 5, 1, &data_out->status, 1, &data_out->slave, 1,
			 &data_out->offset, 2, &data_out->len, 0,
			 &data_out->buff);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_DDC_READ_blocking(state_struct *state,
						HDMITX_TRANS_DATA *data_in,
						HDMITX_TRANS_DATA *data_out)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_DDC_READ
				(state, data_in, data_out));
}

CDN_API_STATUS CDN_API_HDMITX_DDC_WRITE(state_struct *state,
					HDMITX_TRANS_DATA *data_in,
					HDMITX_TRANS_DATA *data_out)
{
	internal_macro_command_txrx(state, MB_MODULE_ID_HDMI_TX, HDMI_TX_WRITE,
				    CDN_BUS_TYPE_APB, 4, 1, data_in->slave, 1,
				    data_in->offset, 2, data_in->len,
				    -data_in->len, data_in->buff);
	internal_readmsg(state, 4, 1, &data_out->status, 1, &data_out->slave, 1,
			 &data_out->offset, 2, &data_out->len);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_DDC_WRITE_blocking(state_struct *state,
						 HDMITX_TRANS_DATA *data_in,
						 HDMITX_TRANS_DATA *data_out)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_DDC_WRITE
				(state, data_in, data_out));
}

CDN_API_STATUS CDN_API_HDMITX_DDC_UPDATE_READ(state_struct *state,
					      HDMITX_TRANS_DATA *data_out)
{
	internal_macro_command_txrx(state, MB_MODULE_ID_HDMI_TX,
				    HDMI_TX_UPDATE_READ, CDN_BUS_TYPE_APB, 0);
	internal_readmsg(state, 2, 1, &data_out->status, 0, &data_out->buff);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_DDC_UPDATE_READ_blocking(state_struct *state,
						       HDMITX_TRANS_DATA *
						       data_out)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_DDC_UPDATE_READ
				(state, data_out));
}

CDN_API_STATUS CDN_API_HDMITX_READ_EDID(state_struct *state, u8 block,
					u8 segment,
					HDMITX_TRANS_DATA *data_out)
{
	internal_macro_command_txrx(state, MB_MODULE_ID_HDMI_TX, HDMI_TX_EDID,
				    CDN_BUS_TYPE_APB, 2, 1, block, 1, segment);
	internal_readmsg(state, 5, 1, &data_out->status, 1, &data_out->slave, 1,
			 &data_out->offset, 2, &data_out->len, 0,
			 &data_out->buff);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_READ_EDID_blocking(state_struct *state, u8 block,
						 u8 segment,
						 HDMITX_TRANS_DATA *data_out)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_READ_EDID
				(state, block, segment, data_out));
}

CDN_API_STATUS
CDN_API_HDMITX_Set_Mode_blocking(state_struct *state,
				 HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE protocol,
				 u32 character_rate)
{
	CDN_API_STATUS ret;
	GENERAL_Read_Register_response resp;
	HDMITX_TRANS_DATA data_in;
	HDMITX_TRANS_DATA data_out;
	u32 clk_reg_0, clk_reg_1;
	u8 buff = 1;

	/* enable/disable  scrambler; */
	if (protocol == HDMI_TX_MODE_HDMI_2_0) {
		if (character_rate >= 340000) {
			buff = 3;	/* enable scrambling + TMDS_Bit_Clock_Ratio */
		} else {
			buff = 1;	/* enable scrambling */
		}
	} else {
		buff = 0;	/* disable scrambling */
	}

	data_in.buff = &buff;
	data_in.len = 1;
	data_in.slave = 0x54;
	data_in.offset = 0x20;	/* TMDS config */
	/* Workaround for imx8qm DDC R/W failed issue */
	if (!cpu_is_imx8qm()) {
		ret = CDN_API_HDMITX_DDC_WRITE_blocking(state, &data_in, &data_out);
		pr_info("CDN_API_HDMITX_DDC_WRITE_blocking ret = %d\n", ret);
		if (ret != CDN_OK)
			return ret;
	}

	ret = CDN_API_General_Read_Register_blocking(
				state, ADDR_SOURCE_MHL_HD + (HDTX_CONTROLLER << 2), &resp);
	if (ret != CDN_OK)
		return ret;

	/* remove data enable */
	resp.val = resp.val & (~(F_DATA_EN(1)));
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	if (ret != CDN_OK)
		return ret;
	clk_reg_0 = 0x7c1f;
	clk_reg_1 = 0x7c1f;
	if (protocol == HDMI_TX_MODE_HDMI_2_0) {
		if (character_rate >= 340000) {
			clk_reg_0 = 0;
			clk_reg_1 = 0xFFFFF;
		}
	}
	ret = CDN_API_General_Write_Register_blocking(
			state, ADDR_SOURCE_MHL_HD + (HDTX_CLOCK_REG_0 << 2),
			     F_DATA_REGISTER_VAL_0(clk_reg_0));
	if (ret != CDN_OK)
		return ret;
	ret = CDN_API_General_Write_Register_blocking(
			state, ADDR_SOURCE_MHL_HD + (HDTX_CLOCK_REG_1 << 2),
				F_DATA_REGISTER_VAL_1(clk_reg_1));
	if (ret != CDN_OK)
		return ret;

	/* set hdmi mode and preemble mode */
	resp.val = resp.val & (~(F_HDMI_MODE(3)));
	resp.val = resp.val & (~(F_HDMI2_PREAMBLE_EN(1)));

	resp.val =
	    (resp.val) | (F_HDMI_MODE(protocol)) | (F_HDMI2_PREAMBLE_EN(1));
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	if (ret != CDN_OK)
		return ret;

	/* data enable */
	resp.val |= F_DATA_EN(1);
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	return ret;
}

CDN_API_STATUS CDN_API_HDMITX_Init_blocking(state_struct *state)
{
	CDN_API_STATUS ret;

	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY +
						    (PHY_DATA_SEL << 2),
						    F_SOURCE_PHY_MHDP_SEL(1));
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_HPD << 2),
						    F_HPD_VALID_WIDTH(4) |
						    F_HPD_GLITCH_WIDTH(0));
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    F_HDMI_MODE(1) |
						    F_AUTO_MODE(0) | F_GCP_EN(1)
						    | F_DATA_EN(1) |
						    F_CLEAR_AVMUTE(1) |
						    F_HDMI2_PREAMBLE_EN(1) |
						    F_HDMI2_CTRL_IL_MODE(1) |
						    F_PIC_3D(0XF) |
						    F_BCH_EN(1));
	if (ret != CDN_OK)
		return ret;
	/* open CARS */
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_PHY_CAR << 2), 0xF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_HDTX_CAR << 2),
						    0xFF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_PKT_CAR << 2), 0xF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_AIF_CAR << 2), 0xF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_CIPHER_CAR << 2),
						    0xF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_CRYPTO_CAR << 2),
						    0xF);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						    (SOURCE_CEC_CAR << 2), 3);
	if (ret != CDN_OK)
		return ret;

	/* init vif */
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_VIF +
						    (HSYNC2VSYNC_POL_CTRL << 2),
						    F_HPOL(0) | F_VPOL(0));
	return ret;
}

CDN_API_STATUS CDN_API_HDMITX_SetVic_blocking(state_struct *state,
					      VIC_MODES vicMode, int bpp,
					      VIC_PXL_ENCODING_FORMAT format)
{
	CDN_API_STATUS ret;
	GENERAL_Read_Register_response resp;
	u32 vsync_lines = vic_table[vicMode][VSYNC];
	u32 eof_lines = vic_table[vicMode][TYPE_EOF];
	u32 sof_lines = vic_table[vicMode][SOF];
	u32 hblank = vic_table[vicMode][H_BLANK];
	u32 hactive = vic_table[vicMode][H_TOTAL] - hblank;
	u32 vblank = vsync_lines + eof_lines + sof_lines;
	u32 vactive = vic_table[vicMode][V_TOTAL] - vblank;
	u32 hfront = vic_table[vicMode][FRONT_PORCH];
	u32 hback = vic_table[vicMode][BACK_PORCH];
	u32 vfront = eof_lines;
	u32 hsync = hblank - hfront - hback;
	u32 vsync = vsync_lines;
	u32 vback = sof_lines;
	u32 v_h_polarity = ((vic_table[vicMode][HSYNC_POL] == ACTIVE_LOW) ? 0 : 1) + ((vic_table[vicMode][VSYNC_POL] == ACTIVE_LOW) ? 0 : 2);

	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (SCHEDULER_H_SIZE << 2),
						    (hactive << 16) + hblank);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (SCHEDULER_V_SIZE << 2),
						    (vactive << 16) + vblank);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_SIGNAL_FRONT_WIDTH <<
						     2),
						    (vfront << 16) + hfront);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_SIGNAL_SYNC_WIDTH <<
						     2), (vsync << 16) + hsync);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_SIGNAL_BACK_WIDTH <<
						     2), (vback << 16) + hback);
	if (ret != CDN_OK)
		return ret;
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_VIF +
						    (HSYNC2VSYNC_POL_CTRL << 2),
						    v_h_polarity);
	if (ret != CDN_OK)
		return ret;

	/* Reset Data Enable */
	CDN_API_General_Read_Register_blocking(state, ADDR_SOURCE_MHL_HD +
					       (HDTX_CONTROLLER << 2), &resp);
	if (ret != CDN_OK)
		return ret;

	/* reset data enable */
	resp.val = resp.val & (~(F_DATA_EN(1)));
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	if (ret != CDN_OK)
		return ret;

	/* set bpp */
	resp.val = resp.val & (~(F_VIF_DATA_WIDTH(3)));
	switch (bpp) {
	case 8:
		resp.val = resp.val | (F_VIF_DATA_WIDTH(0));
		break;

	case 10:
		resp.val = resp.val | (F_VIF_DATA_WIDTH(1));
		break;

	case 12:
		resp.val = resp.val | (F_VIF_DATA_WIDTH(2));
		break;

	case 16:
		resp.val = resp.val | (F_VIF_DATA_WIDTH(3));
		break;
	}

	/* select color encoding */
	resp.val = resp.val & (~(F_HDMI_ENCODING(3)));
	switch (format) {
	case PXL_RGB:

		resp.val = resp.val | (F_HDMI_ENCODING(0));
		break;

	case YCBCR_4_4_4:
		resp.val = resp.val | (F_HDMI_ENCODING(2));
		break;

	case YCBCR_4_2_2:
		resp.val = resp.val | (F_HDMI_ENCODING(1));
		break;

	case YCBCR_4_2_0:
		resp.val = resp.val | (F_HDMI_ENCODING(3));
		break;
	case Y_ONLY:
		/* not exist in hdmi */
		break;
	}

	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	if (ret != CDN_OK)
		return ret;

	/* set data enable */
	resp.val = resp.val | (F_DATA_EN(1));
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_MHL_HD +
						    (HDTX_CONTROLLER << 2),
						    resp.val);
	if (ret != CDN_OK)
		return ret;
	return ret;
}

CDN_API_STATUS CDN_API_HDMITX_ForceColorDepth_blocking(state_struct *state,
						       u8 force, u8 val)
{
	u32 valToWrite = F_COLOR_DEPTH_VAL(val) | F_COLOR_DEPTH_FORCE(force);
	return CDN_API_General_Write_Register_blocking(state,
						       ADDR_SOURCE_MHL_HD +
						       (GCP_FORCE_COLOR_DEPTH_CODING
							<< 2), valToWrite);

}

CDN_API_STATUS CDN_API_HDMITX_ReadEvents(state_struct *state,
					 uint32_t *events)
{
	CDN_API_STATUS ret;

	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;

		internal_tx_mkfullmsg(state, MB_MODULE_ID_HDMI_TX,
				      HDMI_TX_EVENTS, 0);
		state->rxEnable = 1;
		state->bus_type = CDN_BUS_TYPE_APB;

		return CDN_STARTED;
	}

	internal_process_messages(state);

	ret =
	    internal_test_rx_head(state, MB_MODULE_ID_HDMI_TX, HDMI_TX_EVENTS);

	if (ret != CDN_OK)
		return ret;

	internal_readmsg(state, 1, 4, events);

	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_ReadEvents_blocking(state_struct *state,
						  uint32_t *events)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_ReadEvents(state, events));
}

CDN_API_STATUS CDN_API_HDMITX_GetHpdStatus(state_struct *state, u8 *hpd_sts)
{
	CDN_API_STATUS ret;
	if (!state->running) {
		if (!internal_apb_available(state))
			return CDN_BSY;
		internal_tx_mkfullmsg(state, MB_MODULE_ID_HDMI_TX,
				      HDMI_TX_HPD_STATUS, 0);
		state->rxEnable = 1;
		state->bus_type = CDN_BUS_TYPE_APB;
		return CDN_STARTED;
	}
	internal_process_messages(state);
	ret =
	    internal_test_rx_head(state, MB_MODULE_ID_HDMI_TX,
				  HDMI_TX_HPD_STATUS);
	if (ret != CDN_OK)
		return ret;
	internal_readmsg(state, 1, 1, hpd_sts);
	return CDN_OK;
}

CDN_API_STATUS CDN_API_HDMITX_GetHpdStatus_blocking(state_struct *state,
						    u8 *hpd_sts)
{
	internal_block_function(&state->mutex, CDN_API_HDMITX_GetHpdStatus(state, hpd_sts));
}
