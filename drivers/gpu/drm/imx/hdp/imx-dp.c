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
#include "mhdp_firmware.h"
#endif
#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "imx-dp.h"

#ifdef DEBUG_FW_LOAD
void dp_fw_load(state_struct *state)
{
	DRM_INFO("loading hdmi firmware\n");
	CDN_API_LoadFirmware(state,
		(u8 *)mhdp_iram0_get_ptr(),
		mhdp_iram0_get_size(),
		(u8 *)mhdp_dram0_get_ptr(),
		mhdp_dram0_get_size());
}
#endif
int dp_fw_init(state_struct *state)
{
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	u32 core_rate;
	int ret;
	u8 resp;

	core_rate = clk_get_rate(hdp->clks.clk_core);

	/* configure the clock */
	CDN_API_SetClock(state, core_rate/1000000);
	pr_info("CDN_API_SetClock completed\n");

	cdn_apb_write(state, APB_CTRL << 2, 0);
	DRM_INFO("Started firmware!\n");

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		DRM_ERROR("CDN_API_CheckAlive failed - check firmware!\n");
		return -ENXIO;
	} else
		DRM_INFO("CDN_API_CheckAlive returned ret = %d\n", ret);

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &resp);
	DRM_INFO("CDN_API_MainControl_blocking (ret = %d resp = %u)\n",
		ret, resp);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		sizeof(echo_msg), CDN_BUS_TYPE_APB);
	if (0 != strncmp(echo_msg, echo_resp, sizeof(echo_msg))) {
		DRM_ERROR("CDN_API_General_Test_Echo_Ext_blocking - echo test failed, check firmware!");
		return -ENXIO;
	}
	DRM_INFO("CDN_API_General_Test_Echo_Ext_blocking (ret = %d echo_resp = %s)\n",
		ret, echo_resp);

	/* Line swaping */
	CDN_API_General_Write_Register_blocking(state,
		ADDR_SOURCD_PHY + (LANES_CONFIG << 2), 0x0040001b);
	DRM_INFO("CDN_API_General_Write_Register_blockin ... setting LANES_CONFIG\n");

	return 0;
}

int dp_phy_init(state_struct *state, int vic, int format, int color_depth)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int max_link_rate = hdp->link_rate;
	int num_lanes = 4;
	int ret;

	/* reset phy */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, 0);

	/* PHY initialization while phy reset pin is active */
	AFE_init(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_init\n");

	/* In this point the phy reset should be deactivated */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, 1);
	DRM_INFO("deasserted reset\n");

	/* PHY power set */
	AFE_power(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_power exit\n");

	/* Video off */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 0);
	DRM_INFO("CDN_API_DPTX_SetVideo_blocking (ret = %d)\n", ret);

	return true;
}

/* Max Link Rate: 06h (1.62Gbps), 0Ah (2.7Gbps), 14h (5.4Gbps), 1Eh (8.1Gbps)--N/A */
void dp_mode_set(state_struct *state, int vic, int format, int color_depth, int max_link_rate)
{
	int ret;

	/* Set Host capabilities */
	/* Number of lanes and SSC */
	u8 num_lanes = 4;
	u8 ssc = 0;
	u8 scrambler = 0;
	/* Max voltage swing */
	u8 max_vswing = 3;
	u8 force_max_vswing = 0;
	/* Max pre-emphasis */
	u8 max_preemph = 2;
	u8 force_max_preemph = 0;
	/* Supported test patterns mask */
	u8 supp_test_patterns = 0x0F;
	/* AUX training? */
	u8 no_aux_training = 0;
	/* Lane mapping */
	u8 lane_mapping = 0x1B; /*  we have 4 lane, so it's OK */
	/* Extended Host capabilities */
	u8 ext_host_cap = 1;
	/* Bits per sub-pixel */
	u8 bits_per_subpixel = 8;
	/* Stereoscopic video */
	STEREO_VIDEO_ATTR stereo = 0;
	/* B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bt_type = 0;
	/* Transfer Unit */
	u8 transfer_unit = 64;
	VIC_SYMBOL_RATE sym_rate;

	ret = CDN_API_DPTX_SetHostCap_blocking(state,
		max_link_rate,
		(num_lanes & 0x7) | ((ssc & 1) << 3) | ((scrambler & 1) << 4),
		(max_vswing & 0x3) | ((force_max_vswing & 1) << 4),
		(max_preemph & 0x3) | ((force_max_preemph & 1) << 4),
		supp_test_patterns,
		no_aux_training, //fast link training
		lane_mapping,
		ext_host_cap
		);
	DRM_INFO("CDN_API_DPTX_SetHostCap_blocking (ret = %d)\n", ret);

	switch (max_link_rate) {
	case 0x0a:
		sym_rate = RATE_2_7;
		break;
	case 0x14:
		sym_rate = RATE_5_4;
		break;
	default:
		sym_rate = RATE_1_6;
	}

	ret = CDN_API_DPTX_Set_VIC_blocking(state,
		vic,
		bits_per_subpixel,
		num_lanes,
		sym_rate,
		format,
		stereo,
		bt_type,
		transfer_unit
		);
	DRM_INFO("CDN_API_DPTX_Set_VIC_blocking (ret = %d)\n", ret);

	ret = CDN_API_DPTX_TrainingControl_blocking(state, 1);
	DRM_INFO("CDN_API_DPTX_TrainingControl_blocking (ret = %d)\n", ret);

	/* Set video on */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 1);
	DRM_INFO("CDN_API_DPTX_SetVideo_blocking (ret = %d)\n", ret);

	udelay(1000);
}

int dp_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
    DPTX_Read_EDID_response edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = 0;

	memset(&edidResp, 0, sizeof(edidResp));
	switch (block) {
	case 0:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 0, 0, &edidResp);
		break;
	case 1:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 0, 1, &edidResp);
		break;
	case 2:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 1, 0, &edidResp);
		break;
	case 3:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 1, 1, &edidResp);
		break;
	default:
		DRM_WARN("EDID block %x read not support\n", block);
	}

	memcpy(buf, edidResp.buff, 128);

	return ret;
}

int dp_get_hpd_state(state_struct *state, u8 *hpd)
{
	int ret;

	ret = CDN_API_DPTX_GetHpdStatus_blocking(state, hpd);
	return ret;
}
