/*
 * Cadence HDCP API driver
 *
 * Copyright 2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/unaligned.h>
#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_print.h>

#include "cdns-mhdp.h"

static u32 mhdp_hdcp_bus_read(struct cdns_mhdp_device *mhdp, u32 offset)
{
	u32 val;

	mutex_lock(&mhdp->iolock);

	if (mhdp->bus_type == BUS_TYPE_LOW4K_APB) {
		/* Remap address to low 4K APB bus */
		writel(offset >> 12, mhdp->regs_sec + 8);
		val = readl((offset & 0xfff) + mhdp->regs_base);
	} else if (mhdp->bus_type == BUS_TYPE_NORMAL_APB)
		val = readl(mhdp->regs_sec + offset);
	else
		val = readl(mhdp->regs_base + offset);

	mutex_unlock(&mhdp->iolock);

	return val;
}

static void mhdp_hdcp_bus_write(u32 val, struct cdns_mhdp_device *mhdp, u32 offset)
{
	mutex_lock(&mhdp->iolock);

	if (mhdp->bus_type == BUS_TYPE_LOW4K_APB) {
		/* Remap address to low 4K APB bus */
		writel(offset >> 12, mhdp->regs_sec + 8);
		writel(val, (offset & 0xfff) + mhdp->regs_base);
	} else if (mhdp->bus_type == BUS_TYPE_NORMAL_APB)
		writel(val, mhdp->regs_sec + offset);

	mutex_unlock(&mhdp->iolock);
}

static int mhdp_hdcp_mailbox_read(struct cdns_mhdp_device *mhdp)
{
	int val, ret;

	ret = mhdp_readx_poll_timeout(mhdp_hdcp_bus_read, mhdp, MAILBOX_EMPTY_ADDR,
				 val, !val, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	return mhdp_hdcp_bus_read(mhdp, MAILBOX0_RD_DATA) & 0xff;
}

static int mhdp_hdcp_mailbox_write(struct cdns_mhdp_device *mhdp, u8 val)
{
	int ret, full;

	ret = mhdp_readx_poll_timeout(mhdp_hdcp_bus_read, mhdp, MAILBOX_FULL_ADDR,
				 full, !full, MAILBOX_RETRY_US,
				 MAILBOX_TIMEOUT_US);
	if (ret < 0)
		return ret;

	mhdp_hdcp_bus_write(val, mhdp, MAILBOX0_WR_DATA);

	return 0;
}

static int mhdp_hdcp_mailbox_validate_receive(struct cdns_mhdp_device *mhdp,
					      u8 module_id, u8 opcode, u16 req_size)
{
	u32 mbox_size, i;
	u8 header[4];
	int ret;

	/* read the header of the message */
	for (i = 0; i < 4; i++) {
		ret = mhdp_hdcp_mailbox_read(mhdp);
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
			if (mhdp_hdcp_mailbox_read(mhdp) < 0)
				break;

		return -EINVAL;
	}

	return 0;
}

static int mhdp_hdcp_mailbox_read_receive(struct cdns_mhdp_device *mhdp,
					  u8 *buff, u16 buff_size)
{
	u32 i;
	int ret;

	for (i = 0; i < buff_size; i++) {
		ret = mhdp_hdcp_mailbox_read(mhdp);
		if (ret < 0)
			return ret;

		buff[i] = ret;
	}

	return 0;
}

static int mhdp_hdcp_mailbox_send(struct cdns_mhdp_device *mhdp, u8 module_id,
				  u8 opcode, u16 size, u8 *message)
{
	u8 header[4];
	int ret, i;

	header[0] = opcode;
	header[1] = module_id;
	put_unaligned_be16(size, header + 2);

	for (i = 0; i < 4; i++) {
		ret = mhdp_hdcp_mailbox_write(mhdp, header[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < size; i++) {
		ret = mhdp_hdcp_mailbox_write(mhdp, message[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/* HDCP API */
int cdns_mhdp_hdcp_tx_config(struct cdns_mhdp_device *mhdp, u8 config)
{
	int ret;

	mutex_lock(&mhdp->api_lock);
	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				     HDCP_TX_CONFIGURATION, sizeof(config), &config);

	mutex_unlock(&mhdp->api_lock);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp_tx_config);

int cdns_mhdp_hdcp2_tx_respond_km(struct cdns_mhdp_device *mhdp,
					u8 *msg, u16 len)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_RESPOND_KM, len, msg);
	mutex_unlock(&mhdp->api_lock);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp2_tx_respond_km);

int cdns_mhdp_hdcp_tx_status_req(struct cdns_mhdp_device *mhdp,
					u8 *status, u16 len)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				     HDCP_TX_STATUS_CHANGE, 0, NULL);
	if (ret)
		goto err_tx_req;

	ret = mhdp_hdcp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDCP_TX,
						 HDCP_TX_STATUS_CHANGE, len);
	if (ret)
		goto err_tx_req;

	ret = mhdp_hdcp_mailbox_read_receive(mhdp, status, len);
	if (ret)
		goto err_tx_req;

err_tx_req:
	if (ret)
		DRM_ERROR("hdcp tx status req failed: %d\n", ret);
	mutex_unlock(&mhdp->api_lock);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp_tx_status_req);

int cdns_mhdp_hdcp2_tx_is_km_stored_req(struct cdns_mhdp_device *mhdp, u8 *data, u16 len)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_IS_KM_STORED, 0, NULL);
	if (ret)
		goto err_is_km;

	ret = mhdp_hdcp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_IS_KM_STORED, len);
	if (ret)
		goto err_is_km;

	ret = mhdp_hdcp_mailbox_read_receive(mhdp, data, len);

err_is_km:
	if (ret)
		DRM_ERROR("hdcp2 tx is km stored req failed: %d\n", ret);
	mutex_unlock(&mhdp->api_lock);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp2_tx_is_km_stored_req);

int cdns_mhdp_hdcp2_tx_store_km(struct cdns_mhdp_device *mhdp,
					u8 *resp, u16 len)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_STORE_KM, 0, NULL);
	if (ret)
		goto err_store_km;

	ret = mhdp_hdcp_mailbox_validate_receive(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP2_TX_STORE_KM, len);
	if (ret)
		goto err_store_km;

	ret = mhdp_hdcp_mailbox_read_receive(mhdp, resp, len);

err_store_km:
	mutex_unlock(&mhdp->api_lock);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp2_tx_store_km);

int cdns_mhdp_hdcp_tx_is_receiver_id_valid(struct cdns_mhdp_device *mhdp,
					u8 *rx_id, u8 *num)
{
	u32 mbox_size, i;
	u8 header[4];
	u8 temp;
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				      HDCP_TX_IS_RECEIVER_ID_VALID, 0, NULL);
	if (ret)
		goto err_rx_id;

	/* read the header of the message */
	for (i = 0; i < 4; i++) {
		ret = mhdp_hdcp_mailbox_read(mhdp);
		if (ret < 0) {

			mutex_unlock(&mhdp->api_lock);
			return ret;
		}

		header[i] = ret;
	}

	mbox_size = get_unaligned_be16(header + 2);

	if (header[0] != HDCP_TX_IS_RECEIVER_ID_VALID ||
	    header[1] != MB_MODULE_ID_HDCP_TX){

		mutex_unlock(&mhdp->api_lock);
		return -EINVAL;
	}
	/* First get num of receivers */
	ret = mhdp_hdcp_mailbox_read_receive(mhdp, num, 1);
	if (ret)
		goto err_rx_id;

	/* skip second data */
	ret = mhdp_hdcp_mailbox_read_receive(mhdp, &temp, 1);
	if (ret)
		goto err_rx_id;

	/* get receivers ID */
	ret = mhdp_hdcp_mailbox_read_receive(mhdp, rx_id, mbox_size - 2);

err_rx_id:
	mutex_unlock(&mhdp->api_lock);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp_tx_is_receiver_id_valid);

int cdns_mhdp_hdcp_tx_respond_receiver_id_valid(
				struct cdns_mhdp_device *mhdp, u8 val)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				     HDCP_TX_RESPOND_RECEIVER_ID_VALID,
				     sizeof(val), &val);
	mutex_unlock(&mhdp->api_lock);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp_tx_respond_receiver_id_valid);

int cdns_mhdp_hdcp_tx_reauth(struct cdns_mhdp_device *mhdp, u8 msg)
{
	int ret;

	mutex_lock(&mhdp->api_lock);

	ret = mhdp_hdcp_mailbox_send(mhdp, MB_MODULE_ID_HDCP_TX,
				     HDCP_TX_DO_AUTH_REQ, sizeof(msg), &msg);

	mutex_unlock(&mhdp->api_lock);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_hdcp_tx_reauth);
