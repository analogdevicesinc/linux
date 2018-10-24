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
#include <linux/kernel.h>
#include <drm/drm_dp_helper.h>

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
	}

	DRM_INFO("CDN_API_CheckAlive returned ret = %d\n", ret);

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &resp);
	DRM_INFO("CDN_API_MainControl_blocking (ret = %d resp = %u)\n",
		ret, resp);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		sizeof(echo_msg), CDN_BUS_TYPE_APB);
	if (strncmp(echo_msg, echo_resp, sizeof(echo_msg)) != 0) {
		DRM_ERROR("CDN_API_General_Test_Echo_Ext_blocking - echo test failed, check firmware!");
		return -ENXIO;
	}
	DRM_INFO("CDN_API_General_Test_Echo_Ext_blocking (ret = %d echo_resp = %s)\n",
		ret, echo_resp);

	/* Line swaping */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SOURCD_PHY +
						(LANES_CONFIG << 2),
						0x00400000 |
						hdp->dp_lane_mapping);
	DRM_INFO("CDN_API_General_Write_Register_blockin ... setting LANES_CONFIG\n");

	return 0;
}

int dp_phy_init(state_struct *state, struct drm_display_mode *mode, int format,
		int color_depth)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int max_link_rate = hdp->link_rate;
	int num_lanes = 4;
	int ret;

	/* reset phy */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, NULL, 0);

	/* PHY initialization while phy reset pin is active */
	AFE_init(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_init\n");

	/* In this point the phy reset should be deactivated */
	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, NULL, 1);
	DRM_INFO("deasserted reset\n");

	/* PHY power set */
	AFE_power(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_power exit\n");

	/* Video off */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 0);
	DRM_INFO("CDN_API_DPTX_SetVideo_blocking (ret = %d)\n", ret);

	return true;
}

#ifdef DEBUG

void print_header(void)
{
	/*       "0x00000000: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f"*/
	DRM_INFO("          : 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n"
		 );
	DRM_INFO("-----------------------------------------------------------\n"
		 );
}

static void print_bytes(unsigned int addr, unsigned char *buf, unsigned int size)
{
	int i, index = 0;
	char line[160];

	if (((size + 11) * 3) > sizeof(line))
		return;

	index += sprintf(line, "0x%08x:", addr);
	for (i = 0; i < size; i++)
		index += sprintf(&line[index], " %02x", buf[i]);
	DRM_INFO("%s\n", line);

}

static int dump_dpcd(state_struct *state)
{
	int ret;

	DPTX_Read_DPCD_response resp_dpcd;

	print_header();

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x0, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x100, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x110, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x200, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x210, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x220, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x700, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x710, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x720, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}
	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);

	ret = CDN_API_DPTX_Read_DPCD_blocking(state, 0x10, 0x730, &resp_dpcd,
					      CDN_BUS_TYPE_APB);
	if (ret) {
		DRM_INFO("_debug: function returned with status %d\n", ret);
		return -1;
	}

	print_bytes(resp_dpcd.addr, resp_dpcd.buff, resp_dpcd.size);
	return 0;
}
#endif

static bool dp_check_link_status(state_struct *state, u8 num_lanes)
{
	u8 link_status[DP_LINK_STATUS_SIZE];
	DPTX_Read_DPCD_response read_resp;
	CDN_API_STATUS status;

	status = CDN_API_DPTX_Read_DPCD_blocking(state,
					      DP_LINK_STATUS_SIZE,
					      DP_LANE0_1_STATUS,
					      &read_resp,
					      CDN_BUS_TYPE_APB);

	memcpy(link_status, read_resp.buff, DP_LINK_STATUS_SIZE);

	if (status != CDN_OK) {
		return false;
	}

	DRM_DEBUG("link status 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		 link_status[0],link_status[1],link_status[2],
		 link_status[3],link_status[4],link_status[5]);

	/* if link training is requested we should perform it always */
	return drm_dp_channel_eq_ok(link_status, num_lanes);
}

static int dp_get_training_status(state_struct *state)
{
	uint32_t evt;
	uint8_t eventId;
	uint8_t HPDevents;

	do {
		do {
			CDN_API_Get_Event(state, &evt);
			if (evt != 0)
				DRM_DEBUG("_Get_Event %d\n", evt);
		} while ((evt & 2) == 0);
		CDN_API_DPTX_ReadEvent_blocking(state, &eventId, &HPDevents);
		DRM_DEBUG("ReadEvent  ID = %d HPD = %d\n", eventId, HPDevents);

		switch (eventId) {
		case 0x01:
			DRM_INFO("INFO: Full link training started\n");
			break;
		case 0x02:
			DRM_INFO("INFO: Fast link training started\n");
			break;
		case 0x04:
			DRM_INFO("INFO: Clock recovery phase finished\n");
			break;
		case 0x08:
			DRM_INFO("INFO: Channel equalization phase finished (this is last part meaning training finished)\n");
			break;
		case 0x10:
			DRM_INFO("INFO: Fast link training finished\n");
			break;
		case 0x20:
			DRM_INFO("ERROR: Clock recovery phase failed\n");
			return -1;
		case 0x40:
			DRM_INFO("ERROR: Channel equalization phase failed\n");
			return -1;
		case 0x80:
			DRM_INFO("ERROR: Fast link training failed\n");
			return -1;
		default:
			DRM_INFO("ERROR: Invalid ID:%x\n", eventId);
			return -1;
		}
	} while (eventId != 0x08 && eventId != 0x10);

	return 0;
}

#define aux_to_hdp(x) container_of(x, struct imx_hdp, aux)

/*
 * This function only implements native DPDC reads and writes
 */
static ssize_t dp_aux_transfer(struct drm_dp_aux *aux,
		struct drm_dp_aux_msg *msg)
{
	struct imx_hdp *hdp = aux_to_hdp(aux);
	bool native = msg->request & (DP_AUX_NATIVE_WRITE & DP_AUX_NATIVE_READ);
	CDN_API_STATUS status;

	DRM_DEBUG("\n");
	DRM_INFO("%s() msg->request 0x%x msg->size 0x%x\n",
	       __func__, msg->request, (unsigned int)msg->size);


	/* Ignore address only message */
	if ((msg->size == 0) || (msg->buffer == NULL)) {
		msg->reply = native ?
			DP_AUX_NATIVE_REPLY_ACK : DP_AUX_I2C_REPLY_ACK;
		return msg->size;
	}

	if (!native) {
		pr_err("%s: only native messages supported\n",
			__func__);
		return -EINVAL;
	}

	/* msg sanity check */
	if (msg->size > DP_AUX_MAX_PAYLOAD_BYTES) {
		pr_err("%s: invalid msg: size(%zu), request(%x)\n",
			__func__, msg->size, (unsigned int)msg->request);
		return -EINVAL;
	}

	if (msg->request == DP_AUX_NATIVE_WRITE) {
		DPTX_Write_DPCD_response write_resp;

		status = CDN_API_DPTX_Write_DPCD_blocking(&hdp->state,
							  msg->size,
							  msg->address,
							  (u8 *)msg->buffer,
							  &write_resp,
							  CDN_BUS_TYPE_APB);

		if (status != CDN_OK)
			return -EIO;
		/* fixme: is this right? */
		//return  msg->size;
	}

	if (msg->request == DP_AUX_NATIVE_READ) {
		DPTX_Read_DPCD_response read_resp;

		status = CDN_API_DPTX_Read_DPCD_blocking(&hdp->state,
						      msg->size,
						      msg->address,
						      &read_resp,
						      CDN_BUS_TYPE_APB);
		if (status != CDN_OK)
			return -EIO;
		memcpy(msg->buffer, read_resp.buff, read_resp.size);
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
#ifdef DEBUG
		print_bytes(read_resp.addr, read_resp.buff, read_resp.size);
#endif
		return  msg->size;
	}
	return 0;
}

/* Max Link Rate: 06h (1.62Gbps), 0Ah (2.7Gbps), 14h (5.4Gbps),
 * 1Eh (8.1Gbps)--N/A
 */
void dp_mode_set(state_struct *state,
			struct drm_display_mode *mode,
			int format,
			int color_depth,
			int max_link_rate)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int ret;
	u8 training_retries = 10, training_restarts = 10;
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
	u8 lane_mapping = hdp->dp_lane_mapping;
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
	u8 link_rate = RATE_1_6;
	struct drm_dp_link link;

#ifdef DEBUG
	S_LINK_STAT rls;
#endif
	char linkid[6];

	DRM_INFO("dp_mode_set()\n");

	drm_dp_downstream_id(&hdp->aux, linkid);
	DRM_INFO("DP link id: %s, 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		 linkid, linkid[0], linkid[1], linkid[2], linkid[3], linkid[4],
		 linkid[5]);

	drm_dp_link_probe(&hdp->aux, &link);
	DRM_INFO("DP revision: 0x%x\n", link.revision);
	DRM_INFO("DP rate: %d Mbps\n", link.rate/100);
	DRM_INFO("DP number of lanes: %d\n", link.num_lanes);
	DRM_INFO("DP capabilities: 0x%lx\n", link.capabilities);

	/* always use the number of lanes from the display*/
	num_lanes = link.num_lanes;

	/* Use the lower link rate if dp_link_rate is set */
	if (hdp->dp_link_rate != 0) {
		link_rate = min(hdp->dp_link_rate,
			(u32)(drm_dp_link_rate_to_bw_code(link.rate)));
		DRM_INFO("DP actual link rate:  0x%x\n", link_rate);
		hdp->link_rate = link_rate;

		/* need change the link rate */
		hdp->ops->phy_init(state,
				   mode,
				   format,
				   color_depth);
	}

	if (hdp->is_edp) {
		/* use the eDP supported rates */
		switch (link_rate) {
		case AFE_LINK_RATE_1_6:
			sym_rate = RATE_1_6;
			break;
		case AFE_LINK_RATE_2_1:
			sym_rate = RATE_2_1;
			break;
		case AFE_LINK_RATE_2_4:
			sym_rate = RATE_2_4;
			break;
		case AFE_LINK_RATE_2_7:
			sym_rate = RATE_2_7;
			break;
		case AFE_LINK_RATE_3_2:
			sym_rate = RATE_3_2;
			break;
		case AFE_LINK_RATE_4_3:
			sym_rate = RATE_4_3;
			break;
		case AFE_LINK_RATE_5_4:
			sym_rate = RATE_5_4;
			break;
			/*case AFE_LINK_RATE_8_1: sym_rate = RATE_8_1; break; */
		default:
			sym_rate = RATE_1_6;
		}
	} else {
		switch (link_rate) {
		case 0x0a:
			sym_rate = RATE_2_7;
			break;
		case 0x14:
			sym_rate = RATE_5_4;
			break;
		default:
			sym_rate = RATE_1_6;
		}
	}

	ret = CDN_API_DPTX_SetHostCap_blocking(state,
		link_rate,
		(num_lanes & 0x7) | ((ssc & 1) << 3) | ((scrambler & 1) << 4),
		(max_vswing & 0x3) | ((force_max_vswing & 1) << 4),
		(max_preemph & 0x3) | ((force_max_preemph & 1) << 4),
		supp_test_patterns,
		no_aux_training, /* fast link training */
		lane_mapping,
		ext_host_cap
		);
	DRM_INFO("CDN_API_DPTX_SetHostCap_blocking (ret = %d)\n", ret);

	ret = CDN_API_DPTX_Set_VIC_blocking(state,
		mode,
		bits_per_subpixel,
		num_lanes,
		sym_rate,
		format,
		stereo,
		bt_type,
		transfer_unit
		);
	DRM_INFO("CDN_API_DPTX_Set_VIC_blocking (ret = %d)\n", ret);

	training_restarts=5;
	do {

		do {
			ret = CDN_API_DPTX_TrainingControl_blocking(state, 1);
			DRM_INFO("CDN_API_DPTX_TrainingControl_* (ret = %d) start\n",
				   ret);
			if ((dp_get_training_status(state) == 0) /*&&
			     dp_check_link_status(state, num_lanes)*/)
				break;
			training_retries--;

			ret = CDN_API_DPTX_TrainingControl_blocking(state, 0);
			DRM_INFO("CDN_API_DPTX_TrainingControl_* (ret = %d) stop\n",
				   ret);
			udelay(1000);

		} while (training_retries > 0);

		udelay(1000);

		if (dp_check_link_status(state, num_lanes) == true) {
		        DRM_INFO("Link is good - Training complete\n");
		        break;
		} else {
			DRM_INFO("Link is bad - need to restart training\n");
			training_restarts--;
			training_retries = 20;

			ret = CDN_API_DPTX_TrainingControl_blocking(state, 0);
			DRM_INFO("CDN_API_DPTX_TrainingControl_* (ret = %d) stop\n",
				   ret);
			udelay(1000);
		}


	} while (training_restarts > 0);

	DRM_INFO("dp_check_link_status %d\n", dp_check_link_status(state, num_lanes));

	/* Set video on */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 1);
	DRM_INFO("CDN_API_DPTX_SetVideo_blocking (ret = %d)\n", ret);

	udelay(1000);

#ifdef DEBUG
	ret = CDN_API_DPTX_ReadLinkStat_blocking(state, &rls);
	DRM_INFO("INFO: Get Read Link Status (ret = %d resp: rate: %d, lanes: %d, vswing 0..3: %d %d %d, preemp 0..3: %d %d %d\n",
		 ret, rls.rate, rls.lanes,
		 rls.swing[0], rls.swing[1], rls.swing[2],
		 rls.preemphasis[0], rls.preemphasis[1],
		 rls.preemphasis[2]);
	dump_dpcd(state);
#endif
}

int dp_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	DPTX_Read_EDID_response edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = CDN_ERROR_NOT_SUPPORTED;

	if (buf == NULL) {
		return -EINVAL;
	}

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

	DRM_INFO("dp_get_edid_block (ret = %d) block %d\n", ret, block);
	if (ret == CDN_OK) {
		memcpy(buf, edidResp.buff, 128);
		return 0;
	}

	memset(buf, 0, 128);
	return -EIO;
}

int dp_get_hpd_state(state_struct *state, u8 *hpd)
{
	int ret;

	ret = CDN_API_DPTX_GetHpdStatus_blocking(state, hpd);
	return ret;
}

void dp_phy_pix_engine_reset_t28hpc(state_struct *state)
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


int dp_phy_init_t28hpc(state_struct *state,
		       struct drm_display_mode *mode,
		       int format,
		       int color_depth)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int max_link_rate = hdp->link_rate;
	int num_lanes = 4;
	int ret;
	u8 lane_mapping = hdp->dp_lane_mapping;

	/* reset phy */
	imx_hdp_call(hdp, phy_reset, 0, &hdp->mem, 0);
	DRM_INFO("asserted HDP PHY reset\n");

	dp_phy_pix_engine_reset_t28hpc(state);
	DRM_INFO("pixel engine reset\n");

	/* Line swaping */
	CDN_API_General_Write_Register_blocking(state,
						ADDR_SOURCD_PHY +
						(LANES_CONFIG << 2),
						0x00400000 | lane_mapping);
	DRM_INFO("CDN_*_Write_Register_blocking ... setting LANES_CONFIG %x\n",
		 lane_mapping);

	/* PHY initialization while phy reset pin is active */
	afe_init_t28hpc(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_init\n");

	/* In this point the phy reset should be deactivated */
	imx_hdp_call(hdp, phy_reset, 0, &hdp->mem, 1);
	DRM_INFO("deasserted HDP PHY reset\n");

	/* PHY power set */
	afe_power_t28hpc(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);
	DRM_INFO("AFE_power exit\n");

	/* Video off */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 0);
	DRM_INFO("CDN_API_DPTX_SetVideo_blocking (ret = %d)\n", ret);

	return true;
}


int dp_aux_init(state_struct *state,
		  struct device *dev)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	int ret;

	hdp->aux.name = "imx_dp_aux";
	hdp->aux.dev = dev;
	hdp->aux.transfer = dp_aux_transfer;

	ret = drm_dp_aux_register(&hdp->aux);

	return ret;
}

int dp_aux_destroy(state_struct *state)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);

	drm_dp_aux_unregister(&hdp->aux);

	return 0;
}
