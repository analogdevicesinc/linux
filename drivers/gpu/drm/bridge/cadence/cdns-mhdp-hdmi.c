/*
 * Copyright 2019-2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/drm_vblank.h>
#include <drm/drm_print.h>
#include <linux/io.h>
#include <drm/bridge/cdns-mhdp.h>
#include <linux/regmap.h>

void cdns_mhdp_infoframe_set(struct cdns_mhdp_device *mhdp,
					u8 entry_id, u8 packet_len, u8 *packet, u8 packet_type)
{
	u32 *packet32, len32;
	u32 val, i;

	/* invalidate entry */
	val = F_ACTIVE_IDLE_TYPE(1) | F_PKT_ALLOC_ADDRESS(entry_id);
	cdns_mhdp_bus_write(val, mhdp, SOURCE_PIF_PKT_ALLOC_REG);
	cdns_mhdp_bus_write(F_PKT_ALLOC_WR_EN(1), mhdp, SOURCE_PIF_PKT_ALLOC_WR_EN);

	/* flush fifo 1 */
	cdns_mhdp_bus_write(F_FIFO1_FLUSH(1), mhdp, SOURCE_PIF_FIFO1_FLUSH);

	/* write packet into memory */
	packet32 = (u32 *)packet;
	len32 = packet_len / 4;
	for (i = 0; i < len32; i++)
		cdns_mhdp_bus_write(F_DATA_WR(packet32[i]), mhdp, SOURCE_PIF_DATA_WR);

	/* write entry id */
	cdns_mhdp_bus_write(F_WR_ADDR(entry_id), mhdp, SOURCE_PIF_WR_ADDR);

	/* write request */
	cdns_mhdp_bus_write(F_HOST_WR(1), mhdp, SOURCE_PIF_WR_REQ);

	/* update entry */
	val =  F_ACTIVE_IDLE_TYPE(1) | F_TYPE_VALID(1) |
			F_PACKET_TYPE(packet_type) | F_PKT_ALLOC_ADDRESS(entry_id);
	cdns_mhdp_bus_write(val, mhdp, SOURCE_PIF_PKT_ALLOC_REG);

	cdns_mhdp_bus_write(F_PKT_ALLOC_WR_EN(1), mhdp, SOURCE_PIF_PKT_ALLOC_WR_EN);
}

int cdns_hdmi_get_edid_block(void *data, u8 *edid,
			  u32 block, size_t length)
{
	struct cdns_mhdp_device *mhdp = data;
	u8 msg[2], reg[5], i;
	int ret;

	mutex_lock(&mhdp->api_lock);

	for (i = 0; i < 4; i++) {
		msg[0] = block / 2;
		msg[1] = block % 2;

		ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_HDMI_TX, HDMI_TX_EDID,
					  sizeof(msg), msg);
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDMI_TX,
						      HDMI_TX_EDID, sizeof(reg) + length);
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
		if (ret)
			continue;

		ret = cdns_mhdp_mailbox_read_receive(mhdp, edid, length);
		if (ret)
			continue;

		if ((reg[3] << 8 | reg[4]) == length)
			break;
	}

	mutex_unlock(&mhdp->api_lock);

	if (ret)
		DRM_ERROR("get block[%d] edid failed: %d\n", block, ret);
	return ret;
}

int cdns_hdmi_scdc_read(struct cdns_mhdp_device *mhdp, u8 addr, u8 *data)
{
	u8 msg[4], reg[6];
	int ret;

	mutex_lock(&mhdp->api_lock);

	msg[0] = 0x54;
	msg[1] = addr;
	msg[2] = 0;
	msg[3] = 1;
	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_HDMI_TX, HDMI_TX_READ,
				  sizeof(msg), msg);
	if (ret)
		goto err_scdc_read;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDMI_TX,
					      HDMI_TX_READ, sizeof(reg));
	if (ret)
		goto err_scdc_read;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_scdc_read;

	*data = reg[5];

err_scdc_read:
	mutex_unlock(&mhdp->api_lock);
	if (ret)
		DRM_ERROR("scdc read failed: %d\n", ret);
	return ret;
}

int cdns_hdmi_scdc_write(struct cdns_mhdp_device *mhdp, u8 addr, u8 value)
{
	u8 msg[5], reg[5];
	int ret;

	mutex_lock(&mhdp->api_lock);

	msg[0] = 0x54;
	msg[1] = addr;
	msg[2] = 0;
	msg[3] = 1;
	msg[4] = value;
	ret = cdns_mhdp_mailbox_send(mhdp, MB_MODULE_ID_HDMI_TX, HDMI_TX_WRITE,
				  sizeof(msg), msg);
	if (ret)
		goto err_scdc_write;

	ret = cdns_mhdp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDMI_TX,
					      HDMI_TX_WRITE, sizeof(reg));
	if (ret)
		goto err_scdc_write;

	ret = cdns_mhdp_mailbox_read_receive(mhdp, reg, sizeof(reg));
	if (ret)
		goto err_scdc_write;

	if (reg[0] != 0)
		ret = -EINVAL;

err_scdc_write:
	mutex_unlock(&mhdp->api_lock);
	if (ret)
		DRM_ERROR("scdc write failed: %d\n", ret);
	return ret;
}

int cdns_hdmi_ctrl_init(struct cdns_mhdp_device *mhdp,
				 int protocol,
				 u32 char_rate)
{
	u32 reg0;
	u32 reg1;
	u32 val;
	int ret;

	/* Set PHY to HDMI data */
	ret = cdns_mhdp_reg_write(mhdp, PHY_DATA_SEL, F_SOURCE_PHY_MHDP_SEL(1));
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, HDTX_HPD,
					F_HPD_VALID_WIDTH(4) | F_HPD_GLITCH_WIDTH(0));
	if (ret < 0)
		return ret;

	/* open CARS */
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_PHY_CAR, 0xF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_HDTX_CAR, 0xFF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_PKT_CAR, 0xF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_AIF_CAR, 0xF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_CIPHER_CAR, 0xF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_CRYPTO_CAR, 0xF);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, SOURCE_CEC_CAR, 3);
	if (ret < 0)
		return ret;

	reg0 = reg1 = 0x7c1f;
	if (protocol == MODE_HDMI_2_0 && char_rate >= 340000) {
		reg0 = 0;
		reg1 = 0xFFFFF;
	}
	ret = cdns_mhdp_reg_write(mhdp, HDTX_CLOCK_REG_0, reg0);
	if (ret < 0)
		return ret;
	ret = cdns_mhdp_reg_write(mhdp, HDTX_CLOCK_REG_1, reg1);
	if (ret < 0)
		return ret;

	/* set hdmi mode and preemble mode data enable */
	val = F_HDMI_MODE(protocol) | F_HDMI2_PREAMBLE_EN(1) |  F_DATA_EN(1) |
			F_HDMI2_CTRL_IL_MODE(1) | F_BCH_EN(1) | F_PIC_3D(0XF) | F_CLEAR_AVMUTE(1);
	ret = cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);

	return ret;
}

int cdns_hdmi_mode_config(struct cdns_mhdp_device *mhdp,
					      struct drm_display_mode *mode,
						  struct video_info *video_info)
{
	int ret;
	u32 val;
	u32 vsync_lines = mode->vsync_end - mode->vsync_start;
	u32 eof_lines = mode->vsync_start - mode->vdisplay;
	u32 sof_lines = mode->vtotal - mode->vsync_end;
	u32 hblank = mode->htotal - mode->hdisplay;
	u32 hactive = mode->hdisplay;
	u32 vblank = mode->vtotal - mode->vdisplay;
	u32 vactive = mode->vdisplay;
	u32 hfront = mode->hsync_start - mode->hdisplay;
	u32 hback = mode->htotal - mode->hsync_end;
	u32 vfront = eof_lines;
	u32 hsync = hblank - hfront - hback;
	u32 vsync = vsync_lines;
	u32 vback = sof_lines;
	u32 v_h_polarity = ((mode->flags & DRM_MODE_FLAG_NHSYNC) ? 0 : 1) +
						((mode->flags & DRM_MODE_FLAG_NVSYNC) ? 0 : 2);

	ret = cdns_mhdp_reg_write(mhdp, SCHEDULER_H_SIZE, (hactive << 16) + hblank);
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, SCHEDULER_V_SIZE, (vactive << 16) + vblank);
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, HDTX_SIGNAL_FRONT_WIDTH, (vfront << 16) + hfront);
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, HDTX_SIGNAL_SYNC_WIDTH, (vsync << 16) + hsync);
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, HDTX_SIGNAL_BACK_WIDTH, (vback << 16) + hback);
	if (ret < 0)
		return ret;

	ret = cdns_mhdp_reg_write(mhdp, HSYNC2VSYNC_POL_CTRL, v_h_polarity);
	if (ret < 0)
		return ret;

	/* Reset Data Enable */
	val = cdns_mhdp_reg_read(mhdp, HDTX_CONTROLLER);
	val &= ~F_DATA_EN(1);
	ret = cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);
	if (ret < 0)
		return ret;

	/* Set bpc */
	val &= ~F_VIF_DATA_WIDTH(3);
	switch (video_info->color_depth) {
	case 10:
		val |= F_VIF_DATA_WIDTH(1);
		break;
	case 12:
		val |= F_VIF_DATA_WIDTH(2);
		break;
	case 16:
		val |= F_VIF_DATA_WIDTH(3);
		break;
	case 8:
	default:
		val |= F_VIF_DATA_WIDTH(0);
		break;
	}

	/* select color encoding */
	val &= ~F_HDMI_ENCODING(3);
	switch (video_info->color_fmt) {
	case YCBCR_4_4_4:
		val |= F_HDMI_ENCODING(2);
		break;
	case YCBCR_4_2_2:
		val |= F_HDMI_ENCODING(1);
		break;
	case YCBCR_4_2_0:
		val |= F_HDMI_ENCODING(3);
		break;
	case PXL_RGB:
	default:
		val |= F_HDMI_ENCODING(0);
		break;
	}

	ret = cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);
	if (ret < 0)
		return ret;

	/* set data enable */
	val |= F_DATA_EN(1);
	ret = cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);

	return ret;
}

int cdns_hdmi_disable_gcp(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	val = cdns_mhdp_reg_read(mhdp, HDTX_CONTROLLER);
	val &= ~F_GCP_EN(1);

	return cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);
}

int cdns_hdmi_enable_gcp(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	val = cdns_mhdp_reg_read(mhdp, HDTX_CONTROLLER);
	val |= F_GCP_EN(1);

	return cdns_mhdp_reg_write(mhdp, HDTX_CONTROLLER, val);
}
