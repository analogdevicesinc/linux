/*
 * Copyright 2020 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include "cdns-mhdp-hdmirx.h"

static int hdcprx_setconfig(struct cdns_hdmirx_device *hdmirx,
			 struct hdcprx_config *cfg)
{
	u8 msg[3];

	msg[0] = cfg->activate;
	msg[0] |= cfg->version << 1;
	msg[0] |= cfg->repeater << 3;
	msg[0] |= cfg->use_secondary_link << 4;
	msg[0] |= cfg->use_km_key << 5;
	msg[1] = cfg->bcaps;
	msg[2] = cfg->bstatus;

	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDCP_RX,
				      HDCP_RX_SET_CONFIG, 3, msg);
}

static int hdcprx_getstatus(struct cdns_hdmirx_device *hdmirx,
			 struct hdcprx_status *status)
{
	int ret;

	ret = cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDCP_RX,
				      HDCP_RX_GET_STATUS, 0, NULL);
	if (ret)
		goto err_get_sts;

	ret = cdns_hdmirx_mailbox_validate_receive(hdmirx, MB_MODULE_ID_HDCP_RX,
						 HDCP_RX_GET_STATUS, sizeof(struct hdcprx_status));
	if (ret)
		goto err_get_sts;

	ret = cdns_hdmirx_mailbox_read_receive(hdmirx, (u8 *)status, sizeof(struct hdcprx_status));
	if (ret)
		goto err_get_sts;

err_get_sts:
	if (ret)
		dev_err(&hdmirx->pdev->dev, "hdcp rx status req failed: %d\n", ret);
	return ret;
}

static int hdcprx_notsync(struct cdns_hdmirx_device *hdmirx)
{
	return cdns_hdmirx_mailbox_send(hdmirx, MB_MODULE_ID_HDCP_RX,
				      HDCP_RX_NOT_SYNC, 0, NULL);
}

void cdns_hdcprx_enable(struct cdns_hdmirx_device *hdmirx)
{
    int ret;
	int version = HDCPRX_VERSION_2;
	struct hdcprx_config config = { 0 };

	config.activate = 1;
	config.version = version;
	config.repeater = 0;
	config.use_secondary_link = 0;
	config.use_km_key = 1;
	config.bcaps = 0x80;
	config.bstatus = 0x1000; /* HDMI mode */

	/* This was handled by the SECO */
	ret = hdcprx_setconfig(hdmirx, &config);
	if (ret)
		dev_warn(&hdmirx->pdev->dev, "%s(), could not enable the HDCP\n", __func__);
}

void cdns_hdcprx_disable(struct cdns_hdmirx_device *hdmirx)
{
	int ret;
	int version = HDCPRX_VERSION_2;
	struct hdcprx_config config = { 0 };

	config.activate = 0;
	config.version = version;
	config.repeater = 0;
	config.use_secondary_link = 0;
	config.use_km_key = 0;
	config.bcaps = 0x00;
	config.bstatus = 0x1000; /* HDMI mode */

	/* This was handled by the SECO */
	ret = hdcprx_setconfig(hdmirx, &config);
	if (ret)
		dev_warn(&hdmirx->pdev->dev, "%s(), could not disable the HDCP\n", __func__);
}

int cdns_hdcprx_get_status(struct cdns_hdmirx_device *hdmirx,
			   struct hdcprx_status *status)
{
	/* TODO: implement HDCP status checking if needed */
	memset(status, 0, sizeof(struct hdcprx_status));

	return hdcprx_getstatus(hdmirx, status);
}

int cdns_hdcprx_wait_auth_complete(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms)
{
	struct hdcprx_status status;
	u8 key_arrived;
	ktime_t timeout = ktime_timeout_ms(timeout_ms);

	dev_dbg(&hdmirx->pdev->dev, "Wait for HDCP authentication to complete\n");
	do {
		if (!cdns_hdcprx_get_status(hdmirx, &status)) {
			dev_dbg(&hdmirx->pdev->dev, "Failed to get HDCP status\n");
			return -1;
		}
		if (ktime_after(ktime_get(), timeout)) {
			dev_dbg(&hdmirx->pdev->dev, "Timed out waiting for HDCP authentication to complete\n");
			return -1;
		}
		key_arrived = status.flags & 0x1;
		msleep(10);
	} while (key_arrived == 0);

    /* Clear out error register */
	cdns_hdmirx_reg_read(hdmirx, PKT_ERR_CNT_HEADER);

    return 0;
}

static int hdmirx_hdcp_print_status(struct cdns_hdmirx_device *hdmirx)
{
	struct hdcprx_status status;
	int ret;

	ret = cdns_hdcprx_get_status(hdmirx, &status);
	if (!ret) {
	    dev_info(&hdmirx->pdev->dev, "HCDP key_arrived 0x%02x\n", status.flags & 1);
	    dev_info(&hdmirx->pdev->dev, "HCDP hdcp_ver    0x%02x\n", (status.flags >> 1) & 0x3);
	    dev_info(&hdmirx->pdev->dev, "HCDP error       0x%02x\n", (status.flags >> 4) & 0xF);
	    dev_info(&hdmirx->pdev->dev, "HCDP aksv[] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		    status.aksv[0],
		    status.aksv[1],
		    status.aksv[2],
		    status.aksv[3],
		    status.aksv[4]);
	    dev_info(&hdmirx->pdev->dev, "HCDP ainfo      0x%02x\n", status.ainfo);
	}

	return ret;
}

/* As above but wait for completion */
int cdns_hdcprx_reauth_req_wait(struct cdns_hdmirx_device *hdmirx, u16 timeout_ms)
{
	int ret;

	ret = hdcprx_notsync(hdmirx);
	if (ret) {
		dev_info(&hdmirx->pdev->dev, "%s(), could not request reauthentication for the HDCP\n", __func__);
		return ret;
	} else
		dev_info(&hdmirx->pdev->dev, "%s(), requested HDCP re-authentication\n", __func__);
	cdns_hdcprx_wait_auth_complete(hdmirx, timeout_ms);
	ret = hdmirx_hdcp_print_status(hdmirx);

    return ret;
}
