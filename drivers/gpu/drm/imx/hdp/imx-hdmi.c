/*
 * Copyright 2017-2018 NXP
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

#define RGB_ALLOWED_COLORIMETRY (BIT(HDMI_EXTENDED_COLORIMETRY_BT2020) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_ADOBE_RGB))
#define YCC_ALLOWED_COLORIMETRY (BIT(HDMI_EXTENDED_COLORIMETRY_BT2020) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_ADOBE_YCC_601) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_S_YCC_601) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_709) |\
				 BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_601))

static int hdmi_avi_info_set(struct imx_hdp *hdp,
				struct drm_display_mode *mode,
				int format)
{
	struct hdmi_avi_infoframe frame;
	struct drm_display_info *di = &hdp->connector.display_info;
	enum hdmi_extended_colorimetry ext_colorimetry;
	u32 sink_colorimetry;
	u32 allowed_colorimetry;
	u8 buf[32];
	int ret;

	/* Initialise info frame from DRM mode */
	drm_hdmi_avi_infoframe_from_display_mode(&frame, mode, true);

	/* Set up colorimetry */
	allowed_colorimetry = format == PXL_RGB ? RGB_ALLOWED_COLORIMETRY :
						  YCC_ALLOWED_COLORIMETRY;

	sink_colorimetry = di->hdmi.colorimetry & allowed_colorimetry;

	if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_BT2020;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_ADOBE_RGB))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_ADOBE_RGB;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_709))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_XV_YCC_709;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_ADOBE_YCC_601))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_ADOBE_YCC_601;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_S_YCC_601))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_S_YCC_601;
	else if (sink_colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_XV_YCC_601))
		ext_colorimetry = HDMI_EXTENDED_COLORIMETRY_XV_YCC_601;
	else
		ext_colorimetry = 0;

	frame.colorimetry = sink_colorimetry ? HDMI_COLORIMETRY_EXTENDED :
					  HDMI_COLORIMETRY_NONE;
	frame.extended_colorimetry = ext_colorimetry;

	switch (format) {
	case YCBCR_4_4_4:
		frame.colorspace = HDMI_COLORSPACE_YUV444;
		break;
	case YCBCR_4_2_2:
		frame.colorspace = HDMI_COLORSPACE_YUV422;
		break;
	case YCBCR_4_2_0:
		frame.colorspace = HDMI_COLORSPACE_YUV420;
		break;
	default:
		frame.colorspace = HDMI_COLORSPACE_RGB;
		break;
	}

	ret = hdmi_avi_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_ERROR("failed to pack AVI infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	return CDN_API_InfoframeSet(&hdp->state, 0, sizeof(buf),
				    buf, HDMI_INFOFRAME_TYPE_AVI);

}

static int hdmi_vendor_info_set(struct imx_hdp *hdp,
				struct drm_display_mode *mode,
				int format)
{
	struct hdmi_vendor_infoframe frame;
	u8 buf[32];
	int ret;

	/* Initialise vendor frame from DRM mode */
	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame, mode);
	if (ret < 0) {
		DRM_DEBUG("Unable to init vendor infoframe: %d\n", ret);
		return -1;
	}

	ret = hdmi_vendor_infoframe_pack(&frame, buf + 1, sizeof(buf) - 1);
	if (ret < 0) {
		DRM_DEBUG("Unable to pack vendor infoframe: %d\n", ret);
		return -1;
	}

	buf[0] = 0;
	return CDN_API_InfoframeSet(&hdp->state, 0, sizeof(buf),
				    buf, HDMI_INFOFRAME_TYPE_VENDOR);

}

static void hdmi_mode_set_vswing(state_struct *state)
{
	GENERAL_Read_Register_response regresp[12];

	Afe_write(state, 0x41e1, 0x7c0);
	Afe_write(state, 0x43e1, 0x7c0);
	Afe_write(state, 0x45e1, 0x7c0);
	Afe_write(state, 0x47e1, 0x7c0);

	Afe_write(state, 0x404C, 0x0);
	Afe_write(state, 0x424C, 0x0);
	Afe_write(state, 0x444C, 0x0);
	Afe_write(state, 0x464C, 0x0);

	Afe_write(state, 0x4047, 0x120);
	Afe_write(state, 0x4247, 0x120);
	Afe_write(state, 0x4447, 0x120);
	Afe_write(state, 0x4647, 0x120);

	regresp[0].val = Afe_read(state, 0x41e1);
	regresp[1].val = Afe_read(state, 0x43e1);
	regresp[2].val = Afe_read(state, 0x45e1);
	regresp[3].val = Afe_read(state, 0x47e1);

	regresp[4].val = Afe_read(state, 0x404C);
	regresp[5].val = Afe_read(state, 0x424C);
	regresp[6].val = Afe_read(state, 0x444C);
	regresp[7].val = Afe_read(state, 0x464C);

	regresp[8].val = Afe_read(state, 0x4047);
	regresp[9].val = Afe_read(state, 0x4247);
	regresp[10].val = Afe_read(state, 0x4447);
	regresp[11].val = Afe_read(state, 0x4647);

	DRM_DEBUG("LANE0_TX_DIAG_TX_DRV 0x%x \n"
		  "LANE1_TX_DIAG_TX_DRV 0x%x \n"
		  "LANE2_TX_DIAG_TX_DRV 0x%x \n"
		  "LANE3_TX_DIAG_TX_DRV 0x%x \n"
		  "Lane0_TX_TXCC_CPOST_MULT_00 0x%x \n"
		  "Lane1_TX_TXCC_CPOST_MULT_00 0x%x \n"
		  "Lane2_TX_TXCC_CPOST_MULT_00 0x%x \n"
		  "Lane3_TX_TXCC_CPOST_MULT_00 0x%x \n"
		  "Lane0_TX_TXCC_CAL_SCLR_MULT 0x%x \n"
		  "Lane1_TX_TXCC_CAL_SCLR_MULT 0x%x \n"
		  "Lane2_TX_TXCC_CAL_SCLR_MULT 0x%x \n"
		  "Lane3_TX_TXCC_CAL_SCLR_MULT 0x%x \n",
		  regresp[0].val,
		  regresp[1].val,
		  regresp[2].val,
		  regresp[3].val,
		  regresp[4].val,
		  regresp[5].val,
		  regresp[6].val,
		  regresp[7].val,
		  regresp[8].val,
		  regresp[9].val,
		  regresp[10].val,
		  regresp[11].val
		  );
}

int hdmi_phy_init_ss28fdsoi(state_struct *state, struct drm_display_mode *mode, int format, int color_depth)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int ret;

	/* reset phy */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, NULL, 0);

	/* Configure PHY */
	character_freq_khz = phy_cfg_hdp_ss28fdsoi(state, 4, mode, color_depth, format);
	if (character_freq_khz == 0) {
		DRM_ERROR("failed to set phy pclock\n");
		return -EINVAL;
	}

	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, NULL, 1);

	hdmi_tx_kiran_power_configuration_seq(state, 4);

	/* Set the lane swapping */
	ret = CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY + (LANES_CONFIG << 2),
		F_SOURCE_PHY_LANE0_SWAP(3) | F_SOURCE_PHY_LANE1_SWAP(0) |
		F_SOURCE_PHY_LANE2_SWAP(1) | F_SOURCE_PHY_LANE3_SWAP(2) |
		F_SOURCE_PHY_COMB_BYPASS(0) | F_SOURCE_PHY_20_10(1));
	DRM_INFO("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n", ret);

	return true;
}

void hdmi_mode_set_ss28fdsoi(state_struct *state, struct drm_display_mode *mode, int format, int color_depth, int temp)
{
	struct imx_hdp *hdp = container_of(state, struct imx_hdp, state);
	int ret;

	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (drm_match_cea_mode(mode) == VIC_MODE_97_60Hz ||
	    drm_match_cea_mode(mode) == VIC_MODE_96_50Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);
	if (ret != CDN_OK) {
		DRM_INFO("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);
		return;
	}

	/* force GCP CD to 0 when bpp=24 for pass CTS 7-19 */
	if (color_depth == 8)
		CDN_API_HDMITX_Disable_GCP(state);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);
	if (ret != CDN_OK) {
		DRM_INFO("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);
		return;
	}

	ret = hdmi_avi_info_set(hdp, mode, format);
	if (ret < 0) {
		DRM_ERROR("hdmi avi info set ret = %d\n", ret);
		return;
	}

	hdmi_vendor_info_set(hdp, mode, format);

	ret =  CDN_API_HDMITX_SetVic_blocking(state, mode, color_depth, format);
	if (ret != CDN_OK) {
		DRM_INFO("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}

	hdmi_mode_set_vswing(state);
}

int hdmi_phy_init_t28hpc(state_struct *state, struct drm_display_mode *mode, int format, int color_depth)
{
	int ret;
	/* 0- pixel clock from phy */
	u32	pixel_clk_from_phy = 1;
	char echo_msg[] = "echo test";
	char echo_resp[sizeof(echo_msg) + 1];

	/* Parameterization done */

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		DRM_ERROR("NO HDMI FW running\n");
		return -ENXIO;
	}

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
						     sizeof(echo_msg),
						     CDN_BUS_TYPE_APB);
	if (ret != 0) {
		DRM_ERROR("HDMI mailbox access failed\n");
		return -ENXIO;
	}

	/* Configure PHY */
	character_freq_khz =
	    phy_cfg_hdp_t28hpc(state, 4, mode, color_depth, format, pixel_clk_from_phy);
	if (character_freq_khz == 0) {
		DRM_ERROR("failed to set phy pclock\n");
		return -EINVAL;
	}

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
	DRM_INFO
	    ("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n",
	     ret);

	return true;
}

void hdmi_phy_pix_engine_reset_t28hpc(state_struct *state)
{
	GENERAL_Read_Register_response regresp;

	CDN_API_General_Read_Register_blocking(state, ADDR_SOURCE_CAR +
					       (SOURCE_HDTX_CAR << 2),
					       &regresp);
	CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						(SOURCE_HDTX_CAR << 2),
						regresp.val & 0xFD);
	CDN_API_General_Write_Register_blocking(state, ADDR_SOURCE_CAR +
						(SOURCE_HDTX_CAR << 2),
						regresp.val);
}

void hdmi_mode_set_t28hpc(state_struct *state, struct drm_display_mode *mode, int format, int color_depth, int temp)
{
	struct imx_hdp *hdp = container_of(state, struct imx_hdp, state);
	int ret;

	/* Set HDMI TX Mode */
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (drm_match_cea_mode(mode) == VIC_MODE_97_60Hz ||
	    drm_match_cea_mode(mode) == VIC_MODE_96_50Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);
	if (ret != CDN_OK) {
		DRM_ERROR("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);
		return;
	}

	/* force GCP CD to 0 when bpp=24 for pass CTS 7-19 */
	if (color_depth == 8)
		CDN_API_HDMITX_Disable_GCP(state);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);
	if (ret != CDN_OK) {
		DRM_ERROR("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);
		return;
	}

	ret = hdmi_avi_info_set(hdp, mode, format);
	if (ret < 0) {
		DRM_ERROR("hdmi avi info set ret = %d\n", ret);
		return;
	}
	ret = hdmi_avi_info_set(hdp, mode, format);
	if (ret < 0) {
		DRM_ERROR("hdmi avi info set ret = %d\n", ret);
		return;
	}

	ret = hdmi_vendor_info_set(hdp, mode, format);
	if (ret < 0)
		DRM_WARN("Unable to configure VS infoframe\n");

	ret = CDN_API_HDMITX_SetVic_blocking(state, mode, color_depth, format);
	if (ret != CDN_OK) {
		DRM_ERROR("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);
		return;
	}

	hdmi_mode_set_vswing(state);

	msleep(50);
}

#define YUV_MODE		BIT(0)

bool hdmi_mode_fixup_t28hpc(state_struct *state,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	struct imx_hdp *hdp = container_of(state, struct imx_hdp, state);
	int vic = drm_match_cea_mode(mode);
	struct drm_display_info *di = &hdp->connector.display_info;

	hdp->bpc = 8;
	hdp->format = PXL_RGB;

	if ((vic == VIC_MODE_97_60Hz || vic == VIC_MODE_96_50Hz)) {
		if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_36)
			hdp->bpc = 12;
		else if (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_30)
			hdp->bpc = 10;

		if (drm_mode_is_420_only(di, mode) ||
		    (drm_mode_is_420_also(di, mode) && hdp->bpc > 8)) {
			hdp->format = YCBCR_4_2_0;

			adjusted_mode->private_flags = YUV_MODE;
		} else {
			hdp->bpc = 8;
		}

		return true;
	}

	if (di->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_36)
		hdp->bpc = 12;
	else if (di->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_30)
		hdp->bpc = 10;

	/* 10-bit color depth for the following modes is not supported */
	if ((vic == VIC_MODE_95_30Hz || vic == VIC_MODE_94_25Hz ||
	     vic == VIC_MODE_93_24Hz) && hdp->bpc == 10)
		hdp->bpc = 8;

	return true;
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

int hdmi_write_hdr_metadata(state_struct *state,
			    union hdmi_infoframe *hdr_infoframe)
{
	struct imx_hdp *hdp = container_of(state, struct imx_hdp, state);
	u8 buffer[40];
	int infoframe_size;

	infoframe_size = hdmi_infoframe_pack(hdr_infoframe,
					     buffer + 1, sizeof(buffer) - 1);
	if (infoframe_size < 0) {
		dev_err(hdp->dev, "Wrong metadata infoframe: %d\n",
			infoframe_size);
		return infoframe_size;
	}

	buffer[0] = 0;
	infoframe_size++;

	return CDN_API_InfoframeSet(state, 2, infoframe_size,
				    buffer, HDMI_INFOFRAME_TYPE_DRM);
}
