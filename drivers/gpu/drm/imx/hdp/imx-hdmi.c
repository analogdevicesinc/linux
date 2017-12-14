/*
 * Copyright 2017 NXP
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

#include <linux/clk.h>
#ifdef DEBUG_FW_LOAD
#include "hdmitx_firmware.h"
#endif
#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "API_AFE_ss28fdsoi_kiran_hdmitx.h"
#include "API_AFE_t28hpc_hdmitx.h"

static int character_freq_khz;
#ifdef DEBUG_FW_LOAD
void hdmi_fw_load(state_struct *state)
{
	pr_info("loading hdmi firmware\n");
	CDN_API_LoadFirmware(state,
		(u8 *)hdmitx_iram0_get_ptr(),
		hdmitx_iram0_get_size(),
		(u8 *)hdmitx_dram0_get_ptr(),
		hdmitx_dram0_get_size());
}
#endif
int hdmi_fw_init(state_struct *state)
{
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	u32 core_rate;
	int ret;
	u8 sts;

	core_rate = clk_get_rate(hdp->clks.clk_core);

	/* configure the clock */
	CDN_API_SetClock(state, core_rate/1000000);
	pr_info("CDN_API_SetClock completed\n");

	/* moved from CDN_API_LoadFirmware */
	cdn_apb_write(state, APB_CTRL << 2, 0);
	pr_info("Started firmware!\n");

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		pr_err("CDN_API_CheckAlive failed - check firmware!\n");
		return -ENXIO;
	} else
		pr_info("CDN_API_CheckAlive returned ret = %d\n", ret);

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &sts);
	pr_info("CDN_API_MainControl_blocking ret = %d sts = %u\n", ret, sts);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		 sizeof(echo_msg), CDN_BUS_TYPE_APB);

	if (0 != strncmp(echo_msg, echo_resp, sizeof(echo_msg))) {
		pr_err("CDN_API_General_Test_Echo_Ext_blocking - echo test failed, check firmware!");
		return -ENXIO;
	}
	pr_info("CDN_API_General_Test_Echo_Ext_blocking - APB(ret = %d echo_resp = %s)\n",
		  ret, echo_resp);

	return 0;
}

int hdmi_phy_init(state_struct *state, int vic, int format, int color_depth)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int ret;

	/* reset phy */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, 0);

	/* Configure PHY */
	character_freq_khz = phy_cfg_hdp_ss28fdsoi(state, 4, vic, color_depth, format);

	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, 1);

	hdmi_tx_kiran_power_configuration_seq(state, 4);

	/* Set the lane swapping */
	ret = CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY + (LANES_CONFIG << 2),
		F_SOURCE_PHY_LANE0_SWAP(3) | F_SOURCE_PHY_LANE1_SWAP(0) |
		F_SOURCE_PHY_LANE2_SWAP(1) | F_SOURCE_PHY_LANE3_SWAP(2) |
		F_SOURCE_PHY_COMB_BYPASS(0) | F_SOURCE_PHY_20_10(1));
	pr_info("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n", ret);

	return true;
}

void hdmi_mode_set(state_struct *state, int vic, int format, int color_depth, int temp)
{
	int ret;

	/* B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bw_type = 0;
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (vic == VIC_MODE_97_60Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);
	if (ret != CDN_OK) {
		pr_info("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);
		return;
	}

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);
	if (ret != CDN_OK) {
		pr_info("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);
		return;
	}

	ret = CDN_API_Set_AVI(state, vic, format, bw_type);
	if (ret != CDN_OK) {
		pr_info("CDN_API_Set_AVI  ret = %d\n", ret);
		return;
	}

	ret =  CDN_API_HDMITX_SetVic_blocking(state, vic, color_depth, format);
	if (ret != CDN_OK) {
		pr_info("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}

	msleep(50);
}

int hdmi_phy_init_t28hpc(state_struct *state, int vic, int format, int color_depth)
{
	int ret;
	/* 0- pixel clock from phy */
	u32	pixel_clk_from_phy = 1;
	char echo_msg[] = "echo test";
	char echo_resp[sizeof(echo_msg) + 1];

	/* Parameterization done */

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		pr_err("NO HDMI FW running\n");
		return -ENXIO;
	}

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
						     sizeof(echo_msg),
						     CDN_BUS_TYPE_APB);
	if (ret != 0) {
		pr_err("HDMI mailbox access failed\n");
		return -ENXIO;
	}

	/* Configure PHY */
	character_freq_khz =
	    phy_cfg_t28hpc(state, 4, vic, color_depth, format, pixel_clk_from_phy);

	hdmi_tx_t28hpc_power_config_seq(state, 4);

	/* Set the lane swapping */
	ret =
	    CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY +
						    (LANES_CONFIG << 2),
						    F_SOURCE_PHY_LANE0_SWAP(0) |
						    F_SOURCE_PHY_LANE1_SWAP(1) |
						    F_SOURCE_PHY_LANE2_SWAP(2) |
						    F_SOURCE_PHY_LANE3_SWAP(3) |
						    F_SOURCE_PHY_COMB_BYPASS(0)
						    | F_SOURCE_PHY_20_10(1));
	pr_info
	    ("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n",
	     ret);

	return true;
}

void hdmi_mode_set_t28hpc(state_struct *state, int vic, int format, int color_depth, int temp)
{
	int ret;

	/*  B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bw_type = 0;

	/* Set HDMI TX Mode */
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (vic == VIC_MODE_97_60Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);
	if (ret != CDN_OK) {
		pr_info("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);
		return;
	}

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);
	if (ret != CDN_OK) {
		pr_info("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);
		return;
	}

	ret = CDN_API_Set_AVI(state, vic, format, bw_type);
	if (ret != CDN_OK) {
		pr_info("CDN_API_Set_AVI  ret = %d\n", ret);
		return;
	}

	ret = CDN_API_HDMITX_SetVic_blocking(state, vic, color_depth, format);
	if (ret != CDN_OK) {
		pr_info("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}

	msleep(200);
}

int hdmi_get_edid_block(void *data, u8 *buf, u32 block, size_t len)
{
	HDMITX_TRANS_DATA edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = 0;

	printk("%s\n", __func__);
	memset(&edidResp, 0, sizeof(edidResp));
	switch (block) {
	case 0:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 0, 0, &edidResp);
		break;
	case 1:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 0, 1, &edidResp);
		break;
	case 2:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 1, 0, &edidResp);
		break;
	case 3:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 1, 1, &edidResp);
		break;
	default:
		pr_warn("EDID block %x read not support\n", block);
	}

	if (ret == CDN_OK)
		memcpy(buf, edidResp.buff, 128);

	return ret;
}

int hdmi_get_hpd_state(state_struct *state, u8 *hpd)
{
	int ret;

	ret = CDN_API_HDMITX_GetHpdStatus_blocking(state, hpd);
	return ret;
}
