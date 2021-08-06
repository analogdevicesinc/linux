/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <asm/unaligned.h>
#include <drm/bridge/cdns-mhdp.h>
#include "cdns-mhdp-hdmirx.h"

#define MAILBOX_RETRY_US		1000
#define MAILBOX_TIMEOUT_US		5000000

#define hdmirx_readx_poll_timeout(op, addr, offset, val, cond, sleep_us, timeout_us)	\
({ \
	u64 __timeout_us = (timeout_us); \
	unsigned long __sleep_us = (sleep_us); \
	ktime_t __timeout = ktime_add_us(ktime_get(), __timeout_us); \
	might_sleep_if((__sleep_us) != 0); \
	for (;;) { \
		(val) = op(addr, offset); \
		if (cond) \
			break; \
		if (__timeout_us && \
		    ktime_compare(ktime_get(), __timeout) > 0) { \
			(val) = op(addr, offset); \
			break; \
		} \
		if (__sleep_us) \
			usleep_range((__sleep_us >> 2) + 1, __sleep_us); \
	} \
	(cond) ? 0 : -ETIMEDOUT; \
})


u32 cdns_hdmirx_bus_read(struct cdns_hdmirx_device *hdmirx, u32 offset)
{
	u32 val;

	mutex_lock(&hdmirx->iolock);

	if (hdmirx->bus_type == BUS_TYPE_LOW4K_HDMI_RX) {
		/* Remap address to low 4K memory */
		writel(offset >> 12, hdmirx->regs_sec + 4);
		val = readl((offset & 0xfff) + hdmirx->regs_base);
	} else
		val = readl(hdmirx->regs_base + offset);

	mutex_unlock(&hdmirx->iolock);

	return val;
}

void cdns_hdmirx_bus_write(u32 val, struct cdns_hdmirx_device *hdmirx, u32 offset)
{
	mutex_lock(&hdmirx->iolock);

	if (hdmirx->bus_type == BUS_TYPE_LOW4K_HDMI_RX) {
		/* Remap address to low 4K memory */
		writel(offset >> 12, hdmirx->regs_sec + 4);
		writel(val, (offset & 0xfff) + hdmirx->regs_base);
	} else
		writel(val, hdmirx->regs_base + offset);

	mutex_unlock(&hdmirx->iolock);
}

bool cdns_hdmirx_check_alive(struct cdns_hdmirx_device *hdmirx)
{
	u32  alive, newalive;
	u8 retries_left = 50;

	alive = cdns_hdmirx_bus_read(hdmirx, KEEP_ALIVE);

	while (retries_left--) {
		udelay(2);

		newalive = cdns_hdmirx_bus_read(hdmirx, KEEP_ALIVE);
		if (alive == newalive)
			continue;
		return true;
	}
	return false;
}

int hdmirx_mailbox_read(struct cdns_hdmirx_device *hdmirx)
{
	int val, ret;

	ret = hdmirx_readx_poll_timeout(cdns_hdmirx_bus_read, hdmirx, MAILBOX_EMPTY_ADDR,
				 val, !val, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	return cdns_hdmirx_bus_read(hdmirx, MAILBOX0_RD_DATA) & 0xff;
}

static int hdmirx_mailbox_write(struct cdns_hdmirx_device *hdmirx, u8 val)
{
	int ret, full;

	ret = hdmirx_readx_poll_timeout(cdns_hdmirx_bus_read, hdmirx, MAILBOX_FULL_ADDR,
				 full, !full, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	cdns_hdmirx_bus_write(val, hdmirx, MAILBOX0_WR_DATA);

	return 0;
}

int cdns_hdmirx_mailbox_validate_receive(struct cdns_hdmirx_device *hdmirx,
					      u8 module_id, u8 opcode, u16 req_size)
{
	u32 mbox_size, i;
	u8 header[4];
	int ret;

	/* read the header of the message */
	for (i = 0; i < 4; i++) {
		ret = hdmirx_mailbox_read(hdmirx);
		if (ret < 0)
			return ret;

		header[i] = ret;
	}

	mbox_size = get_unaligned_be16(header + 2);

	if (opcode != header[0] || module_id != header[1] ||
	    req_size != mbox_size) {
		/*
		 * If the message in mailbox is not what we want, we need to
		 * clear the mailbox by reading its contents.
		 */
		for (i = 0; i < mbox_size; i++)
			if (hdmirx_mailbox_read(hdmirx) < 0)
				break;

		return -EINVAL;
	}

	return 0;
}

int cdns_hdmirx_mailbox_read_receive(struct cdns_hdmirx_device *hdmirx,
					  u8 *buff, u16 buff_size)
{
	u32 i;
	int ret;

	for (i = 0; i < buff_size; i++) {
		ret = hdmirx_mailbox_read(hdmirx);
		if (ret < 0)
			return ret;

		buff[i] = ret;
	}

	return 0;
}

int cdns_hdmirx_mailbox_send(struct cdns_hdmirx_device *hdmirx, u8 module_id,
				  u8 opcode, u16 size, u8 *message)
{
	u8 header[4];
	int ret, i;

	header[0] = opcode;
	header[1] = module_id;
	put_unaligned_be16(size, header + 2);

	for (i = 0; i < 4; i++) {
		ret = hdmirx_mailbox_write(hdmirx, header[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < size; i++) {
		ret = hdmirx_mailbox_write(hdmirx, message[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int cdns_hdmirx_reg_read(struct cdns_hdmirx_device *hdmirx, u32 addr)
{
	u8 msg[4], resp[8];
	u32 val;
	int ret;

	if (addr == 0) {
		ret = -EINVAL;
		goto err_reg_read;
	}

	put_unaligned_be32(addr, msg);

	ret = cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
				     GENERAL_READ_REGISTER,
				     sizeof(msg), msg);
	if (ret)
		goto err_reg_read;

	ret = cdns_hdmirx_mailbox_validate_receive(hdmirx, MB_MODULE_ID_GENERAL,
						 GENERAL_READ_REGISTER,
						 sizeof(resp));
	if (ret)
		goto err_reg_read;

	ret = cdns_hdmirx_mailbox_read_receive(hdmirx, resp, sizeof(resp));
	if (ret)
		goto err_reg_read;

	/* Returned address value should be the same as requested */
	if (memcmp(msg, resp, sizeof(msg))) {
		ret = -EINVAL;
		goto err_reg_read;
	}

	val = get_unaligned_be32(resp + 4);

	return val;
err_reg_read:
	dev_err(&hdmirx->pdev->dev, "Failed to read register.\n");

	return ret;
}

int cdns_hdmirx_reg_write(struct cdns_hdmirx_device *hdmirx, u32 addr, u32 val)
{
	u8 msg[8];

	put_unaligned_be32(addr, msg);
	put_unaligned_be32(val, msg + 4);

	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
				      GENERAL_WRITE_REGISTER, sizeof(msg), msg);
}

int cdns_hdmirx_reg_write_bit(struct cdns_hdmirx_device *hdmirx, u16 addr,
				   u8 start_bit, u8 bits_no, u32 val)
{
	u8 field[8];

	put_unaligned_be16(addr, field);
	field[2] = start_bit;
	field[3] = bits_no;
	put_unaligned_be32(val, field + 4);

	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_DP_TX,
				      DPTX_WRITE_FIELD, sizeof(field), field);
}

int cdns_hdmirx_set_edid(struct cdns_hdmirx_device *hdmirx,
						u8 segment,	u8 extension, u8 *edid)
{
	struct hdmirx_edid_set_msg msg;

	msg.segment = segment;
	msg.extension = extension;
	memcpy(msg.edid, edid, 128);
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDMI_RX,
			HDMI_RX_SET_EDID, sizeof(struct hdmirx_edid_set_msg), (u8 *)&msg);
}

int cdns_hdmirx_set_scdc_slave(struct cdns_hdmirx_device *hdmirx,
							struct S_HDMI_SCDC_SET_MSG *scdcdata)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDMI_RX,
			HDMI_RX_SCDC_SET, sizeof(struct S_HDMI_SCDC_SET_MSG), (u8 *)scdcdata);
}

int cdns_hdmirx_get_scdc_slave(struct cdns_hdmirx_device *hdmirx,
					     struct S_HDMI_SCDC_GET_MSG *scdcdata)
{
	int len, ret;

	ret = cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDMI_RX,
				     HDMI_RX_SCDC_GET, 0, NULL);
	if (ret)
		goto err_scdc_get;

	len = sizeof(struct S_HDMI_SCDC_GET_MSG);
	ret = cdns_hdmirx_mailbox_validate_receive(hdmirx, MB_MODULE_ID_HDMI_RX,
						 HDMI_RX_SCDC_GET, len);
	if (ret)
		goto err_scdc_get;

	ret = cdns_hdmirx_mailbox_read_receive(hdmirx, (u8 *)scdcdata, len);
	if (ret)
		goto err_scdc_get;

err_scdc_get:
	if (ret)
		dev_err(&hdmirx->pdev->dev, "hdmirx scdc get failed: %d\n", ret);
	return ret;
}

int cdns_hdmirx_sethpd(struct cdns_hdmirx_device *hdmirx, u8 hpd)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDMI_RX,
			HDMI_RX_SET_HPD, sizeof(hpd), &hpd);
}

int cdns_hdmirx_audioautoconfig(
						struct cdns_hdmirx_device *hdmirx,
						u8 max_ch_num,
						u8 i2s_ports_num,
						u8 dis_port3,
						u8 enc_sample_width,
						u8 i2s_sample_width)
{
	u32 regread;
	u8 num_of_pairs_of_channels_per_port = max_ch_num / (i2s_ports_num * 2);
	u8 enc_size_code;
	u8 i2s_size_code;
	u8 i2s_port3_dis = (dis_port3 != 0 && i2s_ports_num == 4) ? 1 : 0;
	u8 times = 0;

	/* Valid values: 1/2/4. */
	/* 3 ports can be emulated with 'i2s_ports_num = 4' and 'dis_port3 = 1'. */
	if (i2s_ports_num == 0 || i2s_ports_num == 3 || i2s_ports_num > 4)
		return -1;

	/* 'dis_port3' makes sense only with 4 ports enabled */
	if (dis_port3 != 0 && i2s_ports_num < 4)
		return -1;

	switch (enc_sample_width) {
	case 16:
		enc_size_code = 0x0;
		break;
	case 24:
		enc_size_code = 0x1;
		break;
	case 32:
		enc_size_code = 0x2;
		break;
	default:
		return -1;
	}

	switch (i2s_sample_width) {
	case 16:
		i2s_size_code = 0x0;
		break;
	case 24:
		i2s_size_code = 0x1;
		break;
	case 32:
		i2s_size_code = 0x2;
		break;
	default:
		return -1;
	}

	/* Maximum number of channels has to be in range from 2 to 32 */
	if (max_ch_num < 2 || max_ch_num > 32)
		return -1;
	/* Maximum number of channels has to be power of 2 */
	else if (max_ch_num & (max_ch_num - 1))
		return -1;
	/* Each active port shall carry the same number of sub-channels */
	else if (max_ch_num % i2s_ports_num)
		return -1;

	/* Disable ACR during configuration */
	cdns_hdmirx_bus_write(F_ACR_SW_RESET(1), hdmirx, ACR_CFG);

	/* Configuring audio FIFO */
	cdns_hdmirx_bus_write(
				F_CFG_FIFO_SW_RST(0) | F_CFG_INDEX_SYNC_EN(1) |
				F_CFG_FIFO_DIR(1) |   F_CFG_DIS_PORT3(i2s_port3_dis),
				hdmirx, FIFO_CNTL_ADDR);

	/* Configuring audio parameters */
	cdns_hdmirx_bus_write(
				F_ENC_LOW_INDEX_MSB(0) | F_SINK_AUDIO_CH_NUM(max_ch_num - 1) |
				F_ENC_SAMPLE_JUST(0x0) | F_ENC_SMPL_WIDTH(enc_size_code) |
				F_I2S_ENC_WL_SIZE(i2s_size_code) | F_CNTL_SMPL_ONLY_EN(1) |
				F_CNTL_TYPE_OVRD(0x0) | F_CNTL_TYPE_OVRD_EN(0) |
				F_I2S_ENC_PORT_EN((1 << i2s_ports_num) - 1) | F_WS_POLARITY(0),
				hdmirx, AUDIO_SINK_CNFG);

	/* Waiting for N value... */
	do {
		regread = cdns_hdmirx_bus_read(hdmirx, AIF_ACR_N_ST);
		times++;
		udelay(10);
	} while (!(regread) && times < 100);

	if (times == 100)
		return -1;

	/* Enable ACR */
	cdns_hdmirx_bus_write(F_ACR_SW_RESET(0), hdmirx, ACR_CFG);

	/* Important:
	 * Write to AIF_ACR_N_OFST_CFG register is interpreted as new N_CTS value.
	 * The ACR has to be enabled (reset released) to register that event.
	 */
	cdns_hdmirx_bus_write(F_ACR_N_OFFSET(regread * (num_of_pairs_of_channels_per_port - 1)),
				hdmirx, AIF_ACR_N_OFST_CFG);

	/* Enable sample decoder */
	cdns_hdmirx_bus_write(F_PKT2SMPL_EN(1), hdmirx, PKT2SMPL_CNTL);

	/* Enable I2S encoder */
	cdns_hdmirx_bus_write(F_I2S_ENC_START(1), hdmirx, AUDIO_SINK_CNTL);

	return 0;
}

/* common API function HDMI TX && RX */
void cdns_hdmirx_set_clock(struct cdns_hdmirx_device *hdmirx, int clk)
{
	cdns_hdmirx_bus_write(clk, hdmirx, SW_CLK_H);
}

int cdns_hdmirx_maincontrol(struct cdns_hdmirx_device *hdmirx, u8 mode)
{
	u8 status;
	int ret;

	ret = cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
				GENERAL_MAIN_CONTROL, sizeof(mode), &mode);
	if (ret)
		goto err_main_ctrl;

	ret = cdns_hdmirx_mailbox_validate_receive(hdmirx, MB_MODULE_ID_GENERAL,
							GENERAL_MAIN_CONTROL, sizeof(status));
	if (ret)
		goto err_main_ctrl;

	ret = cdns_hdmirx_mailbox_read_receive(hdmirx, &status, sizeof(status));
	if (ret)
		goto err_main_ctrl;

	return status;

err_main_ctrl:
	dev_err(&hdmirx->pdev->dev, "hdmirx set mode failed: %d\n", ret);
	return ret;
}

int cdns_hdmirx_general_assertphyreset(struct cdns_hdmirx_device *hdmirx)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
				GENERAL_ASSERT_PHY_BUS_RESET, 0, NULL);
}

int cdns_hdmirx_general_deassertphyreset(struct cdns_hdmirx_device *hdmirx)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
				GENERAL_DEASSERT_PHY_BUS_RESET, 0, NULL);
}

int cdns_hdmirx_general_loadhdcprx(struct cdns_hdmirx_device *hdmirx)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
			GENERAL_LOAD_HDCP_RX, 0, NULL);
}

int cdns_hdmirx_general_unloadhdcprx(struct cdns_hdmirx_device *hdmirx)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL,
			GENERAL_UNLOAD_HDCP_RX, 0, NULL);
}

int cdns_hdmirx_read_hpd(struct cdns_hdmirx_device *hdmirx)
{
	u8 status;
	int ret;

	ret = cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_GENERAL, GENERAL_GET_HPD_STATE,
				  0, NULL);
	if (ret)
		goto err_get_hpd;

	ret = cdns_hdmirx_mailbox_validate_receive(hdmirx, MB_MODULE_ID_GENERAL,
							GENERAL_GET_HPD_STATE, sizeof(status));
	if (ret)
		goto err_get_hpd;

	ret = cdns_hdmirx_mailbox_read_receive(hdmirx, &status, sizeof(status));
	if (ret)
		goto err_get_hpd;

	return status;

err_get_hpd:
	DRM_ERROR("read hpd  failed: %d\n", ret);
	return ret;
}
