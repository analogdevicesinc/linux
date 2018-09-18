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

#include "API_AFE_ss28fdsoi_hdmirx.h"
#include "mxc-hdmi-rx.h"

u8 block0[128] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x3B, 0x10, 0xFD, 0x5A, 0x9B, 0x5F, 0x02, 0x00,
	0x19, 0x1C, 0x01, 0x04, 0xB3, 0x3C, 0x22, 0x78,
	0x9F, 0x30, 0x35, 0xA7, 0x55, 0x4E, 0xA3, 0x26,
	0x0F, 0x50, 0x54, 0x20, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0xFC, 0x00, 0x69, 0x4D, 0x58, 0x38, 0x51,
	0x4D, 0x20, 0x48, 0x44, 0x4D, 0x49, 0x52, 0x58,
	0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x28,
	0x3D, 0x87, 0x87, 0x1E, 0x01, 0x0A, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x64
};

u8 block1[128] = {
	0x02, 0x03, 0x13, 0x71, 0x46, 0x90, 0x04, 0x03,
	0x5D, 0x1F, 0x22, 0x23, 0x09, 0x07, 0x07, 0x83,
	0x01, 0x00, 0x00, 0x04, 0x74, 0x00, 0x30, 0xF2,
	0x70, 0x5A, 0x80, 0xB0, 0x58, 0x8A, 0x00, 0x20,
	0xC2, 0x31, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97
};

S_HDMI_SCDC_SET_MSG scdcExampleData = {
	.sink_ver = 6,
	.manufacturer_oui_1 = 0x1,
	.manufacturer_oui_2 = 0x2,
	.manufacturer_oui_3 = 0x3,
	.devId = {1, 2, 3, 4, 5, 6, 7, 8},
	.hardware_major_rev = 12,
	.hardware_minor_rev = 13,
	.software_major_rev = 0xAB,
	.software_minor_rev = 0XBC,
	.manufacturerSpecific = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	     20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34}
};

int hdmi_infoframe_poll(state_struct *state)
{
	GENERAL_Read_Register_response regresp;
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	u32 regread;
	u32 i;

	/* Unmask "AVI InfoFrame received" interrupt */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_MHL_HD_INT_MASK << 2),
						F_TMDS_MHL_HD_MASK(0xFD));

	/* Unmask "tmds_mhl_hd_int" interrupt */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (MHL_HD_INT_MASK << 2),
						F_MHL_HD_INT_MASK(0xE));

	/* MHL_HD_INT_STAT */
	CDN_API_General_Read_Register_blocking(state,
				       ADDR_SINK_MHL_HD + (MHL_HD_INT_STAT << 2), &regresp);
	for (i = 0; i < 5; i++) {
		if (regresp.val & (1 << 0))
			break;
		else
			CDN_API_General_Read_Register_blocking(state,
			      ADDR_SINK_MHL_HD + (MHL_HD_INT_STAT << 2), &regresp);
		msleep(20);
	}
	if (i == 5) {
		dev_err(&hdmi_rx->pdev->dev, "Waiting for tmds_mhl_hd_int...failed!\n");
		return -1;
	} else
		dev_dbg(&hdmi_rx->pdev->dev, "Waiting for tmds_mhl_hd_int...DONE\n");

	/* Mask "AVI InfoFrame received" interrupt */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_MHL_HD_INT_MASK << 2),
						F_TMDS_MHL_HD_MASK(0xFF));

	/* Mask "tmds_mhl_hd_int" interrupt */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (MHL_HD_INT_MASK << 2),
						F_MHL_HD_INT_MASK(0xF));

	/* Read back TMDS_MHL_HD_INT_STAT to clear the interrupt */
	CDN_API_General_Read_Register_blocking(state,
					       ADDR_SINK_MHL_HD + (TMDS_MHL_HD_INT_STAT << 2),
					       &regresp);

	/* Packet 0 read request */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_CTRL << 2),
		      F_PACKET_RDN_WR(0x0) | F_PACKET_NUM(0x0));

	/* Wait for pkt_host_rdy_status */
	cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &regread);
	for (i = 0; i < 5; i++) {
		if (regread & (1 << 16))
			break;
		else
			cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INT_STATUS << 2), &regread);
		msleep(20);
	}
	if (i == 5) {
		dev_err(&hdmi_rx->pdev->dev, "Waiting for packet data available for reading...FAILED!\n");
		return -1;
	} else
		dev_dbg(&hdmi_rx->pdev->dev, "Waiting for packet data available for reading...DONE\n");

	return 0;
}

/* Exemplary code to configure the controller */
/* for the AVI_InfoFrame reception */
static int get_avi_infoframe(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	GENERAL_Read_Register_response regresp;
	int ret;

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);
	/* Get VIC code and pixel encoding */
	ret = hdmi_infoframe_poll(state);
	if (ret < 0)
		return -1;

	CDN_API_General_Read_Register_blocking(state, ADDR_SINK_MHL_HD + (PKT_AVI_DATA_LOW << 2), &regresp);
	hdmi_rx->vic_code = (regresp.val & 0xFF000000) >> 24;
	hdmi_rx->pixel_encoding = (regresp.val & 0x00000060) >> 5;
	return 0;
}

static int get_vendor_infoframe(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	u32 regread;
	int ret;

	dev_dbg(&hdmi_rx->pdev->dev, "%s\n", __func__);
	/* Set info_type1 to vendor Info Frame */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2),
		      F_INFO_TYPE1(0x81));

	ret = hdmi_infoframe_poll(state);
	if (ret < 0)
		return -1;

	/* Get IEEE OUI */
	cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INFO_DATA1 << 2), &regread);
	if (regread >> 8 == 0x000c03)
		dev_info(&hdmi_rx->pdev->dev, "HDMI 1.4b Vendor Specific Infoframe\n");
	else if (regread >> 8 == 0xC45DD8)
		dev_info(&hdmi_rx->pdev->dev, "HDMI 2.0 Vendor Specific Infoframe\n");
	else
		dev_err(&hdmi_rx->pdev->dev,
					"Error Vendro Infoframe IEEE OUI=0x%6X\n", regread >> 8);

	/* Get HDMI Video format */
	cdn_apb_read(state, ADDR_SINK_PIF + (PKT_INFO_DATA2 << 2), &regread);

	/* Extened resoluction format */
	if (((regread >> 5) & 0x0000007) == 1) {
		hdmi_rx->hdmi_vic = (regread >> 8) & 0xff;
		dev_info(&hdmi_rx->pdev->dev, "hdmi_vic=%d\n", hdmi_rx->hdmi_vic);
	}

	/* Clear info_type1 */
	cdn_apb_write(state, ADDR_SINK_PIF + (PKT_INFO_TYPE_CFG1 << 2),
		      F_INFO_TYPE1(0x00));

	return 0;
}

static void get_color_depth(struct mxc_hdmi_rx_dev *hdmi_rx,
			    clk_ratio_t clk_ratio)
{
	u8 pixel_encoding = hdmi_rx->pixel_encoding;

	hdmi_rx->color_depth = 8;

	switch (pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_4:
		case CLK_RATIO_3_2:
		case CLK_RATIO_2_1:
		case CLK_RATIO_1_2:
		case CLK_RATIO_5_8:
		case CLK_RATIO_3_4:
		default:
			pr_err("YUV422 Not supported clk ration\n");
		}
		break;
	case PIXEL_ENCODING_YUV420:
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_8:
			hdmi_rx->color_depth = 10;
			break;
		case CLK_RATIO_3_4:
			hdmi_rx->color_depth = 12;
			break;
		case CLK_RATIO_5_4:
		case CLK_RATIO_3_2:
		case CLK_RATIO_2_1:
		default:
			pr_err("YUV420 Not supported clk ration\n");
		}
		break;
	default:		/* RGB/YUV444 */
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmi_rx->color_depth = 8;
			break;
		case CLK_RATIO_5_4:
			hdmi_rx->color_depth = 10;
			break;
		case CLK_RATIO_3_2:
			hdmi_rx->color_depth = 12;
			break;
		case CLK_RATIO_2_1:
			hdmi_rx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
		case CLK_RATIO_5_8:
		case CLK_RATIO_3_4:
		default:
			pr_err("RGB/YUV444 Not supported clk ration\n");
		}
	}
}

/* Set edid data sample */
void hdmirx_edid_set(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	GENERAL_Read_Register_response regresp;

	/* Set EDID - block 0 */
	CDN_API_HDMIRX_SET_EDID_blocking(state, 0, 0, &block0[0]);
	/* Set EDID - block 1 */
	CDN_API_HDMIRX_SET_EDID_blocking(state, 0, 1, &block1[0]);
	dev_dbg(&hdmi_rx->pdev->dev, "EDID block 0/1 set complete.\n");

	/* Read the current value of the CTRL register */
	CDN_API_General_Read_Register_blocking(state, 0xb000, &regresp);
	/* Clear the 'divisor_a' and 'divisor_b' values - bits 15:8 */
	regresp.val &= ~0xFF00;
	/* Set the 'divisor_a' value */
	regresp.val |= 0x4 << 14;
	/* Set the 'divisor_b' value */
	regresp.val |= 0x32 << 8;
	/* Write the CTRL register back */
	CDN_API_General_Write_Register_blocking(state, 0xb000, regresp.val);
}

/* Set SCDC data sample */
void hdmirx_scdc_set(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);

	CDN_API_HDMIRX_SET_SCDC_SLAVE_blocking(state, &scdcExampleData);
	dev_dbg(&hdmi_rx->pdev->dev, "SCDC set complete.\n");
}

int hdmirx_init(state_struct *state)
{
	u8 sts;
	u32 ret = 0;

	/* Set uCPU Clock frequency for FW's use [MHz]; */
	CDN_API_SetClock(state, 200);

	/* Relase uCPU */
	cdn_apb_write(state, ADDR_APB_CFG, 0);

	/* Check if the firmware is running */
	ret = CDN_API_CheckAlive_blocking(state);
	if (ret < 0)
		return ret;

	/* Set driver and firmware active */
	CDN_API_MainControl_blocking(state, 1, &sts);
	/* set sample edid and scdc */
	hdmirx_edid_set(state);
	hdmirx_scdc_set(state);

	return ret;
}

int hdmirx_check5v(state_struct *state)
{
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	u8 event5V = 0;
	u32 i;

	/* check for 5v to get hdmi cable state */
	CDN_API_HDMIRX_ReadEvent(state, &event5V);
	for (i = 0; i < 5; i++) {
		if (event5V & (1 << HDMI_RX_EVENT_5V_VAL)) {
			dev_dbg(&hdmi_rx->pdev->dev, "HDMI 5V present\n");
			break;
		}
		msleep(20);
		CDN_API_HDMIRX_ReadEvent(state, &event5V);
	}
	if (i == 5) {
		dev_dbg(&hdmi_rx->pdev->dev, "No HDMI 5V present!!!\n");
		return -1;
	}

	return 0;
}

void hdmirx_hotplug_trigger(state_struct *state)
{
	/* Clear HPD */
	CDN_API_HDMIRX_SetHpd_blocking(state, 0);
	/* provide minimum low pulse length (100ms) */
	msleep(110);
	CDN_API_HDMIRX_SetHpd_blocking(state, 1);
}

/* Bring-up sequence for the HDMI-RX */
int hdmirx_startup(state_struct *state)
{
	u8 sts;
	u32 rx_clk_freq;
	u8 data_rate_change = 0;
	u8 scrambling_en;
	clk_ratio_t clk_ratio, clk_ratio_detected;
	tmds_bit_clock_ratio_t tmds_bit_clock_ratio;
	struct mxc_hdmi_rx_dev *hdmi_rx = state_to_mxc_hdmirx(state);
	S_HDMI_SCDC_GET_MSG *scdcData = &hdmi_rx->scdcData;
	int ret = 0;

	/* Start from TMDS/pixel clock ratio of 1:1.
	 * It affects only pixel clock frequency as the character/data clocks are generated based on
	 * a measured TMDS clock.
	 * This guarantees that the TMDS characters are correctly decoded in the controller regardless
	 * of the pixel clock ratio being programmed. */
	clk_ratio = CLK_RATIO_1_1;

	/* Get TMDS_Bit_Clock_Ratio and Scrambling setting */
	CDN_API_HDMIRX_GET_SCDC_SLAVE_blocking(state, scdcData);
	tmds_bit_clock_ratio =
	    ((scdcData->TMDS_Config & (1 << 1)) >> 1) ?
				TMDS_BIT_CLOCK_RATIO_1_40 : TMDS_BIT_CLOCK_RATIO_1_10;
	scrambling_en = scdcData->TMDS_Config & (1 << 0);

	dev_dbg(&hdmi_rx->pdev->dev,
			"TMDS ratio: 1/%0d, Scrambling %0d).\n", tmds_bit_clock_ratio, scrambling_en);

	/* Configure the PHY */
	pma_config(state);

	/* Reset HDMI RX PHY */
	imx8qm_hdmi_phy_reset(state, 1);

	ret = pma_cmn_ready(state);
	if (ret < 0) {
		pr_err("pma_cmn_ready failed\n");
		return -1;
	}

	msleep(500);

	arc_config(state);

	ret = pma_rx_clk_signal_detect(state);
	if (ret < 0) {
		pr_err("pma_rx_clk_signal_detect failed\n");
		return -1;
	}
	/* Get TMDS clock frequency */
	rx_clk_freq = pma_rx_clk_freq_detect(state);

	ret = pma_pll_config(state, rx_clk_freq, clk_ratio, tmds_bit_clock_ratio,
		       data_rate_change);
	if (ret < 0) {
		pr_err("pma_pll_config failed\n");
		return -1;
	}
	msleep(500);

	/* Setup the scrambling mode */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_CTRL << 2),
						F_SCRAMBLER_MODE(scrambling_en));
	dev_info(&hdmi_rx->pdev->dev,
				"Scrambling %s.\n", (scrambling_en) ? "enabled" : "disabled");
	/*Just to initiate the counters: */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_CNT_INT_CTRL << 2),
						F_SCRAMBLER_SSCP_LINE_DET_THR(0) |
						F_SCRAMBLER_CTRL_LINE_DET_THR(0));
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SINK_MHL_HD + (TMDS_SCR_VALID_CTRL << 2),
						F_SCRAMBLER_SSCP_LINE_VALID_THR(1) |
						F_SCRAMBLER_CTRL_LINE_VALID_THR(0));

	/* The PHY got programmed with the assumed TMDS/pixel clock ratio of 1:1.
	 * Implement the link training procedure to find out the real clock ratio:
	 * 1. Wait for AVI InfoFrame packet
	 * 2. Get the VIC code and pixel encoding from the packet
	 * 3. Evaluate the TMDS/pixel clock ratio based on the vic_table.c
	 * 4. Compare the programmed clock ratio with evaluated one
	 * 5. If mismatch found - reprogram the PHY
	 * 6. Enable the video data path in the controller */

	ret = get_avi_infoframe(state);
	if (ret < 0) {
		pr_err("Get AVI info frame failed\n");
		return -1;
	}
	ret = get_vendor_infoframe(state);
	if (ret < 0)
		pr_info("No Vendor info frame\n");

	dev_dbg(&hdmi_rx->pdev->dev,
			"get_avi_infoframe() vic_code: %0d, pixel_encoding: %0d.\n",
			hdmi_rx->vic_code, hdmi_rx->pixel_encoding);
	ret = mxc_hdmi_frame_timing(hdmi_rx);
	if (ret < 0) {
		pr_err("Get frame timing failed\n\n");
		return -1;
	}

	clk_ratio_detected = clk_ratio_detect(state, rx_clk_freq,
					      hdmi_rx->timings->timings.bt.
					      pixelclock / 1000,
					      hdmi_rx->vic_code,
					      hdmi_rx->pixel_encoding,
					      tmds_bit_clock_ratio);

	data_rate_change = (clk_ratio != clk_ratio_detected);
	if (data_rate_change) {
		dev_dbg(&hdmi_rx->pdev->dev,
				"TMDS/pixel clock ratio mismatch detected (programmed: %0d, detected: %0d)\n",
				clk_ratio, clk_ratio_detected);

		/* Reconfigure the PHY */
		pre_data_rate_change(state);
		pma_rx_clk_signal_detect(state);
		rx_clk_freq = pma_rx_clk_freq_detect(state);
		pma_pll_config(state, rx_clk_freq, clk_ratio_detected,
			       tmds_bit_clock_ratio, data_rate_change);
	} else
		dev_info(&hdmi_rx->pdev->dev, "TMDS/pixel clock ratio correct\n");

	get_color_depth(hdmi_rx, clk_ratio_detected);
	dev_dbg(&hdmi_rx->pdev->dev, "Get colordepth is %d bpc\n", hdmi_rx->color_depth);

	/* Do post PHY programming settings */
	CDN_API_MainControl_blocking(state, 0x80, &sts);
	dev_dbg(&hdmi_rx->pdev->dev,
				"CDN_API_MainControl_blocking() Stage 2 complete.\n");

	/* Initialize HDMI RX */
	CDN_API_HDMIRX_Init_blocking(state);
	dev_dbg(&hdmi_rx->pdev->dev,
				"CDN_API_HDMIRX_Init_blocking() complete.\n");

	/* Initialize HDMI RX CEC */
	CDN_API_General_Write_Register_blocking(state,
				ADDR_SINK_CAR + (SINK_CEC_CAR << 2),
				F_SINK_CEC_SYS_CLK_EN(1) |
				F_SINK_CEC_SYS_CLK_RSTN_EN(1));
	return 0;
}

/* Stop for the HDMI-RX */
void hdmirx_stop(state_struct *state)
{
	CDN_API_HDMIRX_Stop_blocking(state);
}

void hdmirx_phy_pix_engine_reset(state_struct *state)
{
	GENERAL_Read_Register_response regresp;

	CDN_API_General_Read_Register_blocking(state, ADDR_SINK_CAR +
					       (SINK_MHL_HD_CAR << 2),
					       &regresp);
	CDN_API_General_Write_Register_blocking(state, ADDR_SINK_CAR +
						(SINK_MHL_HD_CAR << 2),
						regresp.val & 0x3F);
	CDN_API_General_Write_Register_blocking(state, ADDR_SINK_CAR +
						(SINK_MHL_HD_CAR << 2),
						regresp.val);
}
