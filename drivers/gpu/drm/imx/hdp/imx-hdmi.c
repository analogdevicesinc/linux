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
#include "hdmitx_firmware.h"
#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "API_AFE_ss28fdsoi_kiran_hdmitx.h"

static int character_freq_khz;

void hdmi_fw_load(state_struct *state)
{
	printk("loading hdmi firmware\n");
	CDN_API_LoadFirmware(state,
		(u8 *)hdmitx_iram0_get_ptr(),
		hdmitx_iram0_get_size(),
		(u8 *)hdmitx_dram0_get_ptr(),
		hdmitx_dram0_get_size());
}

void hdmi_fw_init(state_struct *state, u32 core_rate)
{
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	int ret;
	u8 sts;

	/* configure the clock */
	CDN_API_SetClock(state, core_rate/1000000);
	printk("CDN_API_SetClock completed\n");

	/* moved from CDN_API_LoadFirmware */
	cdn_apb_write(state, APB_CTRL << 2, 0);
	printk("Started firmware!\n");

	ret = CDN_API_CheckAlive_blocking(state);
	printk("CDN_API_CheckAlive returned ret = %d\n", ret);

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &sts);
	printk("CDN_API_MainControl_blocking ret = %d sts = %u\n", ret, sts);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		 sizeof(echo_msg), CDN_BUS_TYPE_APB);
	 printk("CDN_API_General_Test_Echo_Ext_blocking - APB(ret = %d echo_resp = %s)\n",
		 ret, echo_resp);
}

void hdmi_phy_init(state_struct *state, int vic, int format, int color_depth)
{
	int ret;

	/* Configure PHY */
	character_freq_khz = phy_cfg_hdp_ss28fdsoi(state, 4, vic, color_depth, format);

	hdp_phy_reset(1);

	hdmi_tx_kiran_power_configuration_seq(state, 4);

	/* Set the lane swapping */
	ret = CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY + (LANES_CONFIG << 2),
		F_SOURCE_PHY_LANE0_SWAP(3) | F_SOURCE_PHY_LANE1_SWAP(0) |
		F_SOURCE_PHY_LANE2_SWAP(1) | F_SOURCE_PHY_LANE3_SWAP(2) |
		F_SOURCE_PHY_COMB_BYPASS(0) | F_SOURCE_PHY_20_10(1));
	printk("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n", ret);
}

void hdmi_mode_set(state_struct *state, int vic, int format, int color_depth, int temp)
{
	int ret;
	GENERAL_Read_Register_response regresp;

	/* B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bw_type = 0;
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (vic == VIC_MODE_97_60Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);
	printk("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);
	printk("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);

	ret = CDN_API_Set_AVI(state, vic, format, bw_type);
	printk("CDN_API_Set_AVI  ret = %d\n", ret);

	ret =  CDN_API_HDMITX_SetVic_blocking(state, vic, color_depth, format);
	printk("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);

	/* adjust the vsync/hsync polarity */
	CDN_API_General_Read_Register_blocking(
					state, ADDR_SOURCE_VIF + (HSYNC2VSYNC_POL_CTRL << 2), &regresp);
	printk("Initial HSYNC2VSYNC_POL_CTRL: 0x%x\n", regresp.val);
	if ((regresp.val & 0x6) != 0) {
		__raw_writel(0x4, state->mem.ss_base);
	}
	msleep(50);
}

int hdmi_get_edid_block(void *data, u8 *buf, u32 block, size_t len)
{
	HDMITX_TRANS_DATA edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = 0;

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
		printk("EDID block %x read not support\n", block);
	}

	memcpy(buf, edidResp.buff, 128);

	return ret;
}

void hdmi_get_hpd_state(state_struct *state, u8 *hpd)
{
	CDN_API_HDMITX_GetHpdStatus_blocking(state, hpd);
}
