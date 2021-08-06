/*
 * Copyright 2018-2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "cdns-mhdp-hdmirx.h"
#include "cdns-hdmirx-phy.h"

#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/ktime.h>

u8 block0[128] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x3B, 0x10, 0x01, 0x00, 0x9B, 0x5F, 0x02, 0x00,
	0x19, 0x1C, 0x01, 0x03, 0x81, 0x3C, 0x22, 0x78,
	0x9F, 0x30, 0x35, 0xA7, 0x55, 0x4E, 0xA3, 0x26,
	0x0F, 0x50, 0x54, 0x20, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,
	0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E,
	0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80,
	0xB0, 0x58, 0x8A, 0x00, 0x20, 0xC2, 0x31, 0x00,
	0x00, 0x1E, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x18,
	0x3D, 0x19, 0x87, 0x1E, 0x00, 0x0A, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x4E, 0x58, 0x50, 0x20, 0x69, 0x4D, 0x58,
	0x38, 0x51, 0x4D, 0x0A, 0x20, 0x20, 0x01, 0x8E,
};

u8 block1[128] = {

	0x02, 0x03, 0x21, 0x71, 0x46, 0x91, 0x04, 0x03,
	0x13, 0x1F, 0x10, 0x23, 0x09, 0x07, 0x07, 0x6D,
	0x03, 0x0C, 0x00, 0x10, 0x00, 0x00, 0x3C, 0x20,
	0x00, 0x60, 0x01, 0x02, 0x03, 0x83, 0x01, 0x00,
	0x00, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A,
	0x80, 0xB0, 0x58, 0x8A, 0x00, 0x20, 0xC2, 0x31,
	0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96,
};

struct S_HDMI_SCDC_SET_MSG scdcexampledata = {
	.sink_ver = 1,
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

#define FW_IRAM_OFFSET		0x2000
#define FW_IRAM_SIZE		0x10000
#define FW_DRAM_SIZE		0x8000

int infoframe_poll(struct cdns_hdmirx_device *hdmirx, u8 type, u8 *buf, u16 timeout_ms)
{
	ktime_t timeout;

	u32 reg;
	int ret = -1;
	u8 i, cs;

	mutex_lock(&hdmirx->pif_mutex);

	timeout = ktime_timeout_ms(timeout_ms);

	/* Packet type to wait for so we don't get anything unexpected
	 * while we are invalidating packets.
	 * We only use one infoframe check location, but we keep TYPE1
	 * to mop up some strange behaviour of some sources with 0x00
	 * types...
	 */
	cdns_hdmirx_bus_write(F_INFO_TYPE1(0x00) | F_INFO_TYPE2(0x00),
					hdmirx, PKT_INFO_TYPE_CFG1);

	/* Enable masking of invalid packets */
	cdns_hdmirx_bus_write(1, hdmirx, PKT_TRANS_CTRL);

	/*
	 * Unmask interrupts for TYPE2 packet reception
	 * and packet transfer PKT transfer complete
	 */
	cdns_hdmirx_bus_write(~0x10002, hdmirx, PKT_INT_MASK);

	/* Read to clear any existing interrupts, don't care about these */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);

	/* Check cleared */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
	if (reg > 0) {
		dev_dbg(&hdmirx->pdev->dev, "Unexpected PKT_INT_STATUS 0,\
					should be cleared after read, got 0x%08X\n", reg);
		goto exit;
	}

	/* Invalidate existing packet data by setting all locations to 0xdeaddead*/
	for (i = 0; i < 2; ++i)
		cdns_hdmirx_bus_write(0xdeaddead, hdmirx, (PKT_INFO_HEADER + i * 4));

	/* Trigger writing of above data into memory. */
	cdns_hdmirx_bus_write(F_PACKET_RDN_WR(0x1) | F_PACKET_NUM(0x1), hdmirx, PKT_INFO_CTRL);

	/* Wait for indication of process completion */
	do {
		reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
		if (ktime_after(ktime_get(), timeout)) {
			dev_dbg(&hdmirx->pdev->dev, "Infoframe poll failed to invalidate existing packet data\n");
			goto exit;
		}
	} while (!(reg & (1 << 16)));
	/* Check cleared */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
	if (reg > 0) {
		dev_dbg(&hdmirx->pdev->dev, "Unexpected PKT_INT_STATUS 1, should be cleared after read, got 0x%08X\n", reg);
		goto exit;
	}

	/* Double check what we told controller to write,
	 * this should be reflecting what we
	 * wrote above and not the memory data yet...
	*/
	for (i = 0; i < 2; ++i) {
		reg = cdns_hdmirx_bus_read(hdmirx, (PKT_INFO_HEADER + i * 4));
		if (reg != 0xdeaddead)
			dev_dbg(&hdmirx->pdev->dev, "Readback reg check for infoframe invalidation failed, got 0x%08X on word %d\n", reg, i);
	}

	/* Now do a readback check just to make sure the writes happened properly, this uses
	 * the same process as above except we have RDN_WR set to 0 to indicate read*/
	cdns_hdmirx_bus_write(F_PACKET_RDN_WR(0x0) | F_PACKET_NUM(0x1), hdmirx, PKT_INFO_CTRL);
	/* Wait for indication of process completion */
	do {
		reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
		if (ktime_after(ktime_get(), timeout)) {
			dev_dbg(&hdmirx->pdev->dev, "Infoframe poll failed to readback packet data\n");
			goto exit;
		}
	} while (!(reg & (1 << 16)));

	/* Compare values */
	for (i = 0; i < 2; ++i) {
		reg = cdns_hdmirx_bus_read(hdmirx, (PKT_INFO_HEADER + i*4));
		if (reg != 0xdeaddead)
			dev_dbg(&hdmirx->pdev->dev, "Readback mem check for infoframe invalidation failed, got 0x%08X on word %d\n", reg, i);
	}
	/* Check cleared */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
	if (reg > 0) {
		dev_dbg(&hdmirx->pdev->dev, "Unexpected PKT_INT_STATUS 2, should be cleared after read, got 0x%08X\n", reg);
		goto exit;
	}

	/* Set packet type to wait for */
	cdns_hdmirx_bus_write(F_INFO_TYPE2(type), hdmirx, PKT_INFO_TYPE_CFG1);

	/* Wait for Infoframe of TYPE2 type */
	do {
		reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
		if (ktime_after(ktime_get(), timeout)) {
			dev_dbg(&hdmirx->pdev->dev, "Infoframe poll did not see requested infoframe type 0x%02X\n", type);
			goto exit;
		}
	} while (!(reg & (1 << 1)));

	/* Check cleared */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
	if (reg > 0) {
		dev_dbg(&hdmirx->pdev->dev, "Unexpected PKT_INT_STATUS 3, should be cleared after read, got 0x%08X\n", reg);
		goto exit;
	}

	/* Load Infoframe contents to registers */
	cdns_hdmirx_bus_write(F_PACKET_RDN_WR(0x0) | F_PACKET_NUM(0x1), hdmirx, PKT_INFO_CTRL);
	do {
		reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
		if (ktime_after(ktime_get(), timeout)) {
			dev_dbg(&hdmirx->pdev->dev, "Infoframe poll failed to load packet data\n");
			goto exit;
		}
	} while (!(reg & (1 << 16)));

	/* Check cleared */
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);
	if (reg > 0) {
		dev_dbg(&hdmirx->pdev->dev, "Unexpected PKT_INT_STATUS 4, should be cleared after read, got 0x%08X\n", reg);
		goto exit;
	}

	/* Read data */
	for (i = 0; i < 8; ++i) {
		reg = cdns_hdmirx_bus_read(hdmirx, PKT_INFO_HEADER + i * 4);
		put_unaligned_le32(reg, buf + i * 4);
	}

	if (buf[0] != type)
		dev_dbg(&hdmirx->pdev->dev, "Infoframe buf[0] read did not match type 0x%02X got 0x%02X\n", type, buf[0]);

	/* Check if InfoFrame size is within maximum range */
	if (buf[2] > 31) {
		dev_dbg(&hdmirx->pdev->dev, "Infoframe poll size exceeds maximum range\n");
		goto exit;
	}
	/* Veryfy checksum */
	for (cs = 0, i = 0; i < 5 + buf[2]; ++i)
		cs += buf[i];
	if (cs) {
		dev_dbg(&hdmirx->pdev->dev, "Infoframe poll checksum check failed\n");
		goto exit;
	}

	ret = 0;

exit:
	/* Disable and clear interrupts */
	cdns_hdmirx_bus_write(~0x00000, hdmirx, PKT_INT_MASK);
	/* Disable detection of Infoframes */
	cdns_hdmirx_bus_write(F_INFO_TYPE1(0x00) | F_INFO_TYPE2(0x00), hdmirx, PKT_INFO_TYPE_CFG1);
	reg = cdns_hdmirx_bus_read(hdmirx, PKT_INT_STATUS);

	mutex_unlock(&hdmirx->pif_mutex);

	return ret;
}

/*
 * Exemplary code to configure the controller for the AVI_InfoFrame reception.
 * Returns -1 on error, 0 if AVI did not change, 1 if AVI changed.
 */
int cdns_hdmirx_get_avi_infoframe(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms)
{
	int ret;
	u8 buf[32];
	/* AVI Data Byte 1 is at offset 5 */
	u8 *avi = &buf[5];

	/* Try to receive AVI */
	ret = infoframe_poll(hdmirx, HDMI_INFOFRAME_TYPE_AVI, buf, timeout_ms);
	if (ret < 0)
		return -1;

	/* Check if header is valid (version 2 only) */
	if (buf[0] != 0x82 || buf[1] != 0x02 || buf[2] != 0x0D || buf[3] != 0) {
		dev_dbg(&hdmirx->pdev->dev, "AVI Infoframe header is not valid, got %.2X %.2X %.2X %.2X\n",
				buf[0], buf[1], buf[2], buf[3]);
		dev_dbg(&hdmirx->pdev->dev, "--- Got: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n",
			buf[0], buf[1], buf[2], buf[3], buf[4],
			buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
			buf[12], buf[13], buf[14], buf[15], buf[16], buf[17]);
		return -1;
	}

	/*
	 * Check if received AVI differs from previous one.
	 * Compare only data bytes 1-5.
	 */
	if (!memcmp(hdmirx->avi, avi, 5))
		return 0;

	dev_dbg(&hdmirx->pdev->dev, "--- avi infoframe new: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n",
		buf[0], buf[1], buf[2], buf[3], buf[4],
		buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
		buf[12], buf[13], buf[14], buf[15], buf[16], buf[17]);
	dev_dbg(&hdmirx->pdev->dev, "--- avi infoframe old:                %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n",
		hdmirx->avi[0], hdmirx->avi[1], hdmirx->avi[2],
		hdmirx->avi[3], hdmirx->avi[4], hdmirx->avi[5],
		hdmirx->avi[6], hdmirx->avi[7], hdmirx->avi[8],
		hdmirx->avi[9], hdmirx->avi[10], hdmirx->avi[11],
		hdmirx->avi[12]);

	memcpy(hdmirx->avi, avi, sizeof(hdmirx->avi));

	hdmirx->vic_code = avi[3] & 0x7F; /* VIC value */
	hdmirx->pixel_encoding = avi[0] >> 5 & 3; /* Y value */

	return 1;
}

int cdns_hdmirx_get_vendor_infoframe(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms)
{
	u32 ieee_oui;
	int ret;
	u8 buf[32];

	dev_dbg(&hdmirx->pdev->dev, "%s\n", __func__);

	/* Try to receive AVI */
	ret = infoframe_poll(hdmirx, HDMI_INFOFRAME_TYPE_VENDOR, buf, timeout_ms);
	if (ret < 0)
		return -1;

	/* Check if header is valid (version 1 only) */
	if (buf[0] != 0x81 || buf[1] != 0x01) {
		dev_dbg(&hdmirx->pdev->dev, "Vendor Infoframe header is not valid, got %.2X %.2X\n", buf[0], buf[1]);
		return -1;
	}

	/*
	 * Check if received Vnd differs from previous one.
	 * Compare only data bytes 1-10.
	 */
	if (!memcmp(hdmirx->vnd, buf, 10))
		return 0;

	dev_dbg(&hdmirx->pdev->dev, "--- Vendor infoframe: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X",
		buf[0], buf[1], buf[2], buf[3], buf[4],
		buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
		buf[12], buf[13], buf[14], buf[15]);

	dev_dbg(&hdmirx->pdev->dev, "---                 : %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X",
		buf[16], buf[16], buf[18], buf[19], buf[20],
		buf[21], buf[22], buf[23], buf[24], buf[25], buf[26], buf[27],
		buf[28], buf[29], buf[30], buf[31]);

	memcpy(hdmirx->vnd, buf, sizeof(hdmirx->vnd));

	ieee_oui = buf[5] | buf[6] << 8 | buf[7] << 16;

	/* Get IEEE OUI */
	if (ieee_oui == 0x000C03)
		dev_info(&hdmirx->pdev->dev, "HDMI 1.4b Vendor Specific Infoframe\n");
	else if (ieee_oui == 0xC45DD8)
		dev_info(&hdmirx->pdev->dev, "HDMI 2.0 Vendor Specific Infoframe\n");
	else
		dev_err(&hdmirx->pdev->dev,
			"Error Vendro Infoframe IEEE OUI=0x%6X\n", ieee_oui);

	/* Extened resoluction format */
	if ((buf[8] >> 5 & 0x07) == 1) {
		hdmirx->hdmi_vic = buf[9];
		dev_info(&hdmirx->pdev->dev, "hdmi_vic=%d\n", hdmirx->hdmi_vic);
	}

	return 1;
}

static void get_color_depth(struct cdns_hdmirx_device *hdmirx, int clk_ratio)
{
	u8 pixel_encoding = hdmirx->pixel_encoding;

	hdmirx->color_depth = 8;

	switch (pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		switch (clk_ratio) {
		case CLK_RATIO_1_1:
			hdmirx->color_depth = 8;
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
			hdmirx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
			hdmirx->color_depth = 8;
			break;
		case CLK_RATIO_5_8:
			hdmirx->color_depth = 10;
			break;
		case CLK_RATIO_3_4:
			hdmirx->color_depth = 12;
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
			hdmirx->color_depth = 8;
			break;
		case CLK_RATIO_5_4:
			hdmirx->color_depth = 10;
			break;
		case CLK_RATIO_3_2:
			hdmirx->color_depth = 12;
			break;
		case CLK_RATIO_2_1:
			hdmirx->color_depth = 16;
			break;
		case CLK_RATIO_1_2:
		case CLK_RATIO_5_8:
		case CLK_RATIO_3_4:
		default:
			pr_err("RGB/YUV444 Not supported clk ration\n");
		}
	}

	switch (pixel_encoding) {
	case PIXEL_ENCODING_YUV422:
		if ((0 != hdmirx->vic_code) || (0 == hdmirx->hdmi_vic))
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d %dbit YUV422\n",
					hdmirx->vic_code,  hdmirx->color_depth);
		else
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d and HDMI_VIC=%d %dbit YUV422\n",
					hdmirx->vic_code, hdmirx->hdmi_vic,  hdmirx->color_depth);
		break;
	case PIXEL_ENCODING_YUV420:
		if ((0 != hdmirx->vic_code) || (0 == hdmirx->hdmi_vic))
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d %dbit YUV420\n",
				hdmirx->vic_code,  hdmirx->color_depth);
		else
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d and HDMI_VIC=%d %dbit YUV422\n",
				hdmirx->vic_code, hdmirx->hdmi_vic,  hdmirx->color_depth);
			break;
	case PIXEL_ENCODING_YUV444:
		if ((0 != hdmirx->vic_code) || (0 == hdmirx->hdmi_vic))
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d %dbit YUV444\n",
				hdmirx->vic_code,  hdmirx->color_depth);
		else
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d and HDMI_VIC=%d %dbit YUV422\n",
				hdmirx->vic_code, hdmirx->hdmi_vic,  hdmirx->color_depth);
		break;
	case PIXEL_ENCODING_RGB:
		if ((0 != hdmirx->vic_code) || (0 == hdmirx->hdmi_vic))
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d %dbit RGB\n",
				hdmirx->vic_code, hdmirx->color_depth);
		else
			dev_info(&hdmirx->pdev->dev, "Detect mode VIC %d and HDMI_VIC=%d %dbit YUV422\n",
				hdmirx->vic_code, hdmirx->hdmi_vic,  hdmirx->color_depth);
		break;
	default:
		dev_err(&hdmirx->pdev->dev, "Unknow color format\n");
	}
}

/* Set edid data sample */
static void hdmirx_edid_set(struct cdns_hdmirx_device *hdmirx)
{
	/* Set EDID - block 0 */
	cdns_hdmirx_set_edid(hdmirx, 0, 0, &block0[0]);
	/* Set EDID - block 1 */
	cdns_hdmirx_set_edid(hdmirx, 0, 1, &block1[0]);
	dev_dbg(&hdmirx->pdev->dev, "EDID block 0/1 set complete.\n");
}

/* Set SCDC data sample */
static void hdmirx_scdc_set(struct cdns_hdmirx_device *hdmirx)
{
	cdns_hdmirx_set_scdc_slave(hdmirx, &scdcexampledata);
	dev_dbg(&hdmirx->pdev->dev, "SCDC set complete.\n");
}

/* Set AVI data */
static inline void hdmirx_avi_set(struct cdns_hdmirx_device *hdmirx)
{
	memset(hdmirx->avi, 0, sizeof(hdmirx->avi));
}

static int hdmirx_firmware_write_section(struct cdns_hdmirx_device *hdmirx,
					const u8 *data, int size, int addr)
{
	int i;

	for (i = 0; i < size; i += 4) {
		u32 val = (unsigned int)data[i] << 0 |
					(unsigned int)data[i + 1] << 8 |
					(unsigned int)data[i + 2] << 16 |
					(unsigned int)data[i + 3] << 24;
		cdns_hdmirx_bus_write(val, hdmirx, addr + i);
	}

	return 0;
}

static void hdmirx_firmware_load_cont(const struct firmware *fw, void *context)
{
	struct cdns_hdmirx_device *hdmirx = context;

	hdmirx->fw = fw;
}

static int hdmirx_firmware_load(struct cdns_hdmirx_device *hdmirx)
{
	const u8 *iram;
	const u8 *dram;
	int ret;

	/* skip fw loading if none is specified */
	if (!hdmirx->firmware_name)
		return 0;

	if (!hdmirx->fw) {
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						hdmirx->firmware_name,
						&hdmirx->pdev->dev, GFP_KERNEL,
						hdmirx,
						hdmirx_firmware_load_cont);
		if (ret < 0) {
			DRM_ERROR("failed to load firmware\n");
			return -ENOENT;
		}
	} else {
		iram = hdmirx->fw->data + FW_IRAM_OFFSET;
		dram = iram + FW_IRAM_SIZE;

		hdmirx_firmware_write_section(hdmirx, iram, FW_IRAM_SIZE, ADDR_IMEM);
		hdmirx_firmware_write_section(hdmirx, dram, FW_DRAM_SIZE, ADDR_DMEM);
	}

	DRM_INFO("Started RX firmware!\n");

	return 0;
}

int cdns_hdmirx_init(struct cdns_hdmirx_device *hdmirx)
{
	int ret = 0;

	hdmirx_firmware_load(hdmirx);

	/* Set uCPU Clock frequency for FW's use [MHz]; */
	cdns_hdmirx_set_clock(hdmirx, 200);

	/* Relase uCPU */
	cdns_hdmirx_bus_write(0, hdmirx, APB_CTRL);

	/* Check if the firmware is running */
	ret = cdns_hdmirx_check_alive(hdmirx);
	if (ret == false) {
		pr_err("NO HDMI RX FW running\n");
		return -ENXIO;
	}

	/* Set driver and firmware active */
	cdns_hdmirx_maincontrol(hdmirx, 1);
	cdns_hdmirx_sethpd(hdmirx, 0);

	cdns_hdmirx_maincontrol(hdmirx, 0);
	imx8qm_hdmi_phy_reset(hdmirx, 0);
	cdns_hdmirx_general_assertphyreset(hdmirx);

	cdns_hdmirx_maincontrol(hdmirx, 1);
	msleep(500);

	/* Set sample edid and scdc */
	hdmirx_edid_set(hdmirx);
	hdmirx_scdc_set(hdmirx);
	hdmirx_avi_set(hdmirx);
	msleep(200);

	return ret;
}

void cdns_hdmirx_hotplug_trigger(struct cdns_hdmirx_device *hdmirx)
{
	dev_dbg(&hdmirx->pdev->dev, "%s Triggering a 200ms HPD event\n", __func__);
	/* Clear HPD */
	cdns_hdmirx_sethpd(hdmirx, 0);
	/* provide minimum low pulse length (100ms) */
	msleep(200);
	cdns_hdmirx_sethpd(hdmirx, 1);
}

static void hdmirx_phy_pix_engine_reset(struct cdns_hdmirx_device *hdmirx)
{
	u32 val;

	val = cdns_hdmirx_reg_read(hdmirx, SINK_MHL_HD_CAR);
	cdns_hdmirx_reg_write(hdmirx, SINK_MHL_HD_CAR, val & 0x3D);
	cdns_hdmirx_reg_write(hdmirx, SINK_MHL_HD_CAR, val);
}

#ifdef debug
static void do_test_fw_read_check(struct cdns_hdmirx_device *hdmirx)
{
	u32 val;

	val = cdns_hdmirx_reg_read(hdmirx, TMDS_CH0_ERR_CNT);
	dev_info(&hdmirx->pdev->dev, "Got TMDS_CH0_ERR_CNT: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_CH1_ERR_CNT);
	dev_info(&hdmirx->pdev->dev, "Got TMDS_CH1_ERR_CNT: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_CH2_ERR_CNT);
	dev_info(&hdmirx->pdev->dev, "Got TMDS_CH2_ERR_CNT: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);
	dev_info(&hdmirx->pdev->dev, "Got TMDS_DEC_ST: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, 0x5834);
	dev_info(&hdmirx->pdev->dev, "Got 0x5834: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, PKT_AVI_DATA_LOW);
	dev_info(&hdmirx->pdev->dev, "Got PKT_AVI_DATA_LOW: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, PKT_AVI_DATA_HIGH);
	dev_info(&hdmirx->pdev->dev, "Got PKT_AVI_DATA_HIGH: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);
	dev_info(&hdmirx->pdev->dev, "Got PKT_ERR_CNT_HEADER: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_01);
	dev_info(&hdmirx->pdev->dev, "Got PKT_ERR_CNT_01: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_23);
	dev_info(&hdmirx->pdev->dev, "Got PKT_ERR_CNT_23: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, 0x5828);
	dev_info(&hdmirx->pdev->dev, "Got 0x5828: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, 0x582c);
	dev_info(&hdmirx->pdev->dev, "Got 0x582C: 0x%08X\n", val);
	val = cdns_hdmirx_reg_read(hdmirx, 0x5830);
	dev_info(&hdmirx->pdev->dev, "Got 0x5830: 0x%08X\n", val);
}
#endif

int cdns_hdmirx_phyinit(struct cdns_hdmirx_device *hdmirx)
{
	cdns_hdmirx_maincontrol(hdmirx, 0x40);
	dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(0x40)\n", __func__);
	imx8qm_hdmi_phy_reset(hdmirx, 0);
	cdns_hdmirx_general_assertphyreset(hdmirx);

	cdns_hdmirx_maincontrol(hdmirx, 1);
	dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(1)\n", __func__);
	cdns_hdmirx_general_deassertphyreset(hdmirx);

	if (hdmirx->rescal_val > 0)
		set_rescal_code(hdmirx, hdmirx->rescal_val);
	else
		reset_rescal_code(hdmirx);

	set_slicer_tune_val(hdmirx, hdmirx->slicer_tune_val);

	/* First check to see if PHY is in reset, if so we need to
	 * do pma_config then arc_config, this is always going to be
	 * the case for now... */
	if (phy_in_reset(hdmirx)) {
		/* Configure the PHY */
		dev_info(&hdmirx->pdev->dev, "Doing initial PHY configuration\n");
		pma_config(hdmirx);
		dev_info(&hdmirx->pdev->dev, "Releasing PHY reset\n");
		imx8qm_hdmi_phy_reset(hdmirx, 1);
		dev_info(&hdmirx->pdev->dev, "Waiting for pma_cmn_ready\n");
		if (pma_cmn_ready(hdmirx) < 0) {
			dev_err(&hdmirx->pdev->dev, "pma_cmn_ready failed\n");
			dev_info(&hdmirx->pdev->dev, "Setting PHY reset\n");
			imx8qm_hdmi_phy_reset(hdmirx, 0);
			return -1;
		}

		/* init ARC */
		arc_config(hdmirx);
		get_rescal_code(hdmirx);

		/* Don't care about this initial status read,
		 * just clearing it out first before triggering HPD...
		 */
		{
			uint32_t events;
			msleep(200);
			events = cdns_hdmirx_bus_read(hdmirx, SW_EVENTS1);
			cdns_hdmirx_hotplug_trigger(hdmirx);
			cdns_hdmirx_wait_edid_read(hdmirx);
			mdelay(200);
		}
	} else {
		dev_dbg(&hdmirx->pdev->dev, "Prepare for rate change\n");
		pre_data_rate_change(hdmirx);
	}

	get_rescal_code(hdmirx);
	return 0;
}

int hdmirx_config(struct cdns_hdmirx_device *hdmirx)
{
	struct S_HDMI_SCDC_GET_MSG *scdcData = &hdmirx->scdcData;
	u8 clk_ratio, clk_ratio_detected;
	u8 data_rate_change = 0;
	u8 scrambling_en;
	u32 tmds_bit_clock_ratio;
	int ret;
	ktime_t timeout;
	u32 val;
	u8 null_info[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	dev_dbg(&hdmirx->pdev->dev, "Prepare for rate change\n");
	pre_data_rate_change(hdmirx);

	/* Detect rx clk signal */
	if (pma_rx_clk_signal_detect(hdmirx)) {
		dev_err(&hdmirx->pdev->dev, "Common rx_clk signal detect failed\n");
		return -1;
	} else
		dev_dbg(&hdmirx->pdev->dev, "pma_rx_clk_signal detected\n");

	/* Get TMDS_Bit_Clock_Ratio and Scrambling setting */
	cdns_hdmirx_get_scdc_slave(hdmirx, scdcData);
	tmds_bit_clock_ratio = ((scdcData->TMDS_Config & (1 << 1)) >> 1) ?
			       TMDS_BIT_CLOCK_RATIO_1_40 :
			       TMDS_BIT_CLOCK_RATIO_1_10;
	scrambling_en = scdcData->TMDS_Config & (1 << 0);

	dev_dbg(&hdmirx->pdev->dev,
		"TMDS ratio: 1/%0d, Scrambling %0d).\n", tmds_bit_clock_ratio, scrambling_en);

	/* Get TMDS clock frequency */
	hdmirx->tmds_clk = cdns_hdmirx_get_stable_tmds(hdmirx);
	if (hdmirx->tmds_clk <= 0) {
		dev_err(&hdmirx->pdev->dev, "detect tmds clock failed\n");
		return -1;
	}
	dev_info(&hdmirx->pdev->dev, "Detect TMDS clock freq: %d kHz\n",
		 hdmirx->tmds_clk);

	/* Start from TMDS/pixel clock ratio of 1:1.
	 * It affects only pixel clock frequency as the character/data clocks
	 * are generated based on a measured TMDS clock.
	 * This guarantees that the TMDS characters are correctly decoded in
	 * the controller regardless of the pixel clock ratio being
	 * programmed.
	 */
	clk_ratio = CLK_RATIO_1_1;

	ret = pma_pll_config(hdmirx, hdmirx->tmds_clk, clk_ratio,
			     tmds_bit_clock_ratio, data_rate_change);
	if (ret < 0) {
		dev_err(&hdmirx->pdev->dev, "pma_pll_config failed, this is a critical error will try resetting PHY\n");
		cdns_hdmirx_phyinit(hdmirx);
		return -1;
	}

#ifdef debug
	do_test_fw_read_check(hdmirx);
#endif

	/* Setup the scrambling mode */
	cdns_hdmirx_reg_write(hdmirx, TMDS_SCR_CTRL, F_SCRAMBLER_MODE(scrambling_en));
	dev_dbg(&hdmirx->pdev->dev,
				"Scrambling %s.\n", (scrambling_en) ? "enabled" : "disabled");

	/*Just to initiate the counters: */
	cdns_hdmirx_reg_write(hdmirx, TMDS_SCR_CNT_INT_CTRL,
					F_SCRAMBLER_SSCP_LINE_DET_THR(0) |
					F_SCRAMBLER_CTRL_LINE_DET_THR(0));

	cdns_hdmirx_reg_write(hdmirx, TMDS_SCR_VALID_CTRL,
						F_SCRAMBLER_SSCP_LINE_VALID_THR(1) |
						F_SCRAMBLER_CTRL_LINE_VALID_THR(0));

	/* Clear the TMDS decoder */
	cdns_hdmirx_reg_write(hdmirx, TMDS_DEC_CTRL,
						F_DECODER_ERR_CORR_EN(1) |
						F_TMDS_DECODER_SW_RST(1));

	/* Read to clear register for status */
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);

	cdns_hdmirx_reg_write(hdmirx, TMDS_DEC_CTRL,
						F_DECODER_ERR_CORR_EN(1) |
						F_TMDS_DECODER_SW_RST(0));

	/* Wait for lock to TMDS datastream before continuing...*/
	timeout = ktime_timeout_ms(50);
	do {
		val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);
		dev_dbg(&hdmirx->pdev->dev, "Got TMDS_DEC_ST: 0x%08X\n", val);

		if (ktime_after(ktime_get(), timeout)) {
			dev_err(&hdmirx->pdev->dev, "Timeout locking to TMDS datastream\n");
			return -1;
		}
	} while ((val & 0x200) == 0);

	/* Do post PHY programming settings */
	cdns_hdmirx_maincontrol(hdmirx, 0x80);
	dev_dbg(&hdmirx->pdev->dev, "MainControl() Initial Stage 2 complete.\n");

	/* Clear down VIF side in case we weren't able to do so before...*/
	cdns_hdmirx_reg_write(hdmirx, VIDEO_UNPACK_CTRL, 0);
	cdns_hdmirx_reg_write(hdmirx, VANLYZ_CTRL, F_VANLYZ_RESET(1));

	memcpy(hdmirx->avi, null_info, sizeof(hdmirx->avi));
	memcpy(hdmirx->vnd, null_info, sizeof(hdmirx->vnd));
	hdmirx->hdmi_vic = 0;
	hdmirx->vic_code = 0;
	hdmirx->pixel_encoding = PIXEL_ENCODING_RGB;

	/* The PHY got programmed with the assumed TMDS/pixel clock ratio of 1:1.
	 * Implement the link training procedure to find out the real clock ratio:
	 * 1. Wait for AVI InfoFrame packet
	 * 2. Get the VIC code and pixel encoding from the packet
	 * 3. Evaluate the TMDS/pixel clock ratio based on the vic_table.c
	 * 4. Compare the programmed clock ratio with evaluated one
	 * 5. If mismatch found - reprogram the PHY
	 * 6. Enable the video data path in the controller */

	if (hdmirx->allow_hdcp) {
		struct hdcprx_status status;
		u8 hdcp_ver;

		/* First check to see what was the HDCP status */
		cdns_hdcprx_get_status(hdmirx, &status);

		/* Now do a quick check to see if we can receive an AVI Infoframe.
		   If so then all good can skip re-authentication, otherwise if
		   there had been some HDCP activity then try to authenticate..
		*/
		ret = cdns_hdmirx_get_avi_infoframe(hdmirx, 200);

		/* It is difficult to avoid a race condition here as we don't know
		   when authentication completed vs the above stage 2 call which is
		   needed to activate parts of the firmware and init some FIFOs in
		   the controller so we will just wait for any authentication to
		   complete then request a re-authentication
		*/
		hdcp_ver = (status.flags >> 1) & 0x3;
		if ((ret < 0) && (hdcp_ver > 0)) {	/* Possibly authenticating */
			cdns_hdcprx_wait_auth_complete(hdmirx, 2000);
			dev_dbg(&hdmirx->pdev->dev, "Requesting HDCP re-authentication\n");
			cdns_hdcprx_reauth_req_wait(hdmirx, 2000);
		}
		/* Clear out any logged errors by reading */
		cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);
	}

	ret = cdns_hdmirx_get_avi_infoframe(hdmirx, 2500);

	hdmirx_phy_pix_engine_reset(hdmirx);

	/* We also want to check what the decoder status was in case above failed... */
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);
	/* Read to get proper value */
	val = cdns_hdmirx_reg_read(hdmirx, TMDS_DEC_ST);
	dev_dbg(&hdmirx->pdev->dev, "Got TMDS_DEC_ST: 0x%08X\n", val);

	if (ret < 0) {
		if ((val & 0x200) == 0) {
			dev_dbg(&hdmirx->pdev->dev, "Lost link alignment.\n");
			goto error_exit;
		} else if ((val & 0x1) == 0) {
			dev_dbg(&hdmirx->pdev->dev, "\nLink looks to be in DVI mode.\n");
			dev_dbg(&hdmirx->pdev->dev, "Not currently handled by driver, exiting...\n\n");
			goto error_exit;
		} else {
			dev_err(&hdmirx->pdev->dev, "Get AVI info frame failed\n");
			goto error_exit;
		}
	}

	ret = cdns_hdmirx_get_vendor_infoframe(hdmirx, 250);
	if (ret < 0)
		dev_dbg(&hdmirx->pdev->dev, "No Vendor info frame\n");

	dev_dbg(&hdmirx->pdev->dev, "VIC: %0d, pixel_encoding: %0d.\n",
			hdmirx->vic_code, hdmirx->pixel_encoding);
	ret = cdns_hdmirx_frame_timing(hdmirx);
	if (ret < 0) {
		dev_err(&hdmirx->pdev->dev, "Get frame timing failed\n\n");
		goto error_exit;
	}

	clk_ratio_detected = clk_ratio_detect(hdmirx, hdmirx->tmds_clk,
					      hdmirx->timings->timings.bt.
					      pixelclock / 1000,
					      hdmirx->vic_code,
					      hdmirx->pixel_encoding,
					      tmds_bit_clock_ratio);

	data_rate_change = (clk_ratio != clk_ratio_detected);
	if (data_rate_change) {
		dev_dbg(&hdmirx->pdev->dev,
				"TMDS/pixel clock ratio mismatch detected (programmed: %0d, detected: %0d)\n",
				clk_ratio, clk_ratio_detected);

		cdns_hdmirx_maincontrol(hdmirx, 0x40);
		dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(0x40)\n", __func__);
		cdns_hdmirx_maincontrol(hdmirx, 1);
		dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(1)\n", __func__);

		/* Reconfigure the PHY */
		dev_dbg(&hdmirx->pdev->dev, "Prepare to change rate\n");
		pre_data_rate_change(hdmirx);

		/* Re-use last measured TMDS frequency... */
		ret = pma_pll_config(hdmirx, hdmirx->tmds_clk, clk_ratio_detected,
			       tmds_bit_clock_ratio, data_rate_change);
		if (ret < 0) {
			dev_err(&hdmirx->pdev->dev, "pma_pll_config failed\n");
			return -1;
		}

		/* Do post PHY programming settings */
		cdns_hdmirx_maincontrol(hdmirx, 0x80);
		dev_dbg(&hdmirx->pdev->dev, "MainControl() Initial Stage 2 complete.\n");

		if (hdmirx->allow_hdcp) {
			struct hdcprx_status status;
			u8 hdcp_ver;

			/* First check to see what was the HDCP status */
			cdns_hdcprx_get_status(hdmirx, &status);

			/* Now do a quick check to see if we can receive an AVI Infoframe.
			 * If so then all good can skip re-authentication, otherwise if
			 * there had been some HDCP activity then try to authenticate..
			 */
			ret = cdns_hdmirx_get_avi_infoframe(hdmirx, 200);

			/* It is difficult to avoid a race condition here as we don't know
			 * when authentication completed vs the above stage 2 call which is
			 * needed to activate parts of the firmware and init some FIFOs in
			 * the controller so we will just wait for any authentication to
			 * complete then request a re-authentication
			 */
			hdcp_ver = (status.flags >> 1) & 0x3;
			if ((ret < 0) && (hdcp_ver > 0)) {	/* Possibly authenticating */
				cdns_hdcprx_wait_auth_complete(hdmirx, 2000);
				dev_dbg(&hdmirx->pdev->dev, "Requesting HDCP re-authentication\n");
				cdns_hdcprx_reauth_req_wait(hdmirx, 2000);
			}
			/* Clear out any logged errors by reading */
			cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);
		}
		hdmirx_phy_pix_engine_reset(hdmirx);

	} else
		dev_dbg(&hdmirx->pdev->dev, "TMDS/pixel clock ratio correct\n");

	get_color_depth(hdmirx, clk_ratio_detected);
	get_rescal_code(hdmirx);
	return 0;

error_exit:
	cdns_hdmirx_maincontrol(hdmirx, 0x40);
	dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(0x40)\n", __func__);
	cdns_hdmirx_maincontrol(hdmirx, 1);
	dev_dbg(&hdmirx->pdev->dev, "%s(): called MainControl(1)\n", __func__);

	return -1;
}

/* Bring-up sequence for the HDMI-RX */
int cdns_hdmirx_startup(struct cdns_hdmirx_device *hdmirx)
{
	int ret = 0;
	u8 val;

	ret = hdmirx_config(hdmirx);
	if (ret) {
		dev_err(&hdmirx->pdev->dev, "PHY configuration failed\n");
		hdmirx->tmds_clk = -1;
		return ret;
	}

	/* Initialize HDMI RX */
	cdns_hdmirx_reg_write(hdmirx, VIDEO_UNPACK_CTRL, F_CD_ENABLE(1));
	val = F_VANLYZ_START(1) | F_VANLYZ_FRAMES_CHECK_EN(1) | F_VANLYZ_FORMAT_FINDER_EN(1);
	cdns_hdmirx_reg_write(hdmirx, VANLYZ_CTRL, val);

	dev_dbg(&hdmirx->pdev->dev,
				"HDMIRX_Init() complete.\n");

	/* Initialize HDMI RX CEC */
	cdns_hdmirx_reg_write(hdmirx, SINK_CEC_CAR,
				F_SINK_CEC_SYS_CLK_EN(1) |
				F_SINK_CEC_SYS_CLK_RSTN_EN(1));
	return 0;
}
