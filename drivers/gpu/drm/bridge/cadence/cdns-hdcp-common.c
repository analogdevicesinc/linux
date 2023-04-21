/*
 * Cadence HDMI/DP HDCP driver
 *
 * Copyright 2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <drm/bridge/cdns-mhdp.h>
#include <drm/display/drm_hdcp_helper.h>
#include <drm/drm_print.h>
#include <linux/firmware.h>

#include "cdns-mhdp-hdcp.h"

/* Default will be to use KM unless it has been explicitly */
#ifndef HDCP_USE_KMKEY
	#define HDCP_USE_KMKEY 1
#endif

#define CDNS_HDCP_ACTIVATE (0x1 << 2)

#define IMX_FW_TIMEOUT_MS      (64 * 1000)
#define IMX_HDCP_PAIRING_FIRMWARE "imx/hdcp-pairing.bin"

#define GENERAL_BUS_SETTINGS_DPCD_BUS_BIT           0
#define GENERAL_BUS_SETTINGS_DPCD_BUS_LOCK_BIT      1
#define GENERAL_BUS_SETTINGS_HDCP_BUS_BIT           2
#define GENERAL_BUS_SETTINGS_HDCP_BUS_LOCK_BIT      3
#define GENERAL_BUS_SETTINGS_CAPB_OWNER_BIT         4
#define GENERAL_BUS_SETTINGS_CAPB_OWNER_LOCK_BIT    5

#define GENERAL_BUS_SETTINGS_RESP_DPCD_BUS_BIT      0
#define GENERAL_BUS_SETTINGS_RESP_HDCP_BUS_BIT      1
#define GENERAL_BUS_SETTINGS_RESP_CAPB_OWNER_BIT    2

/* HDCP TX ports working mode (HDCP 2.2 or 1.4) */
enum {
	HDCP_TX_2,		/* lock only with HDCP2 */
	HDCP_TX_1,		/* lock only with HDCP1 */
	HDCP_TX_BOTH,	/* lock on HDCP2 or 1 depend on other side */
};

/* HDCP TX ports stream type (relevant if receiver is repeater) */
enum {
	HDCP_CONTENT_TYPE_0,  /* May be transmitted by
							 The HDCP Repeater to all HDCP Devices. */
	HDCP_CONTENT_TYPE_1,  /* Must not be transmitted by the HDCP Repeater to
							 HDCP 1.x-compliant Devices and HDCP 2.0-compliant Repeaters */
};

/* different error types for HDCP_TX_STATUS_CHANGE */
enum {
	HDCP_TRAN_ERR_NO_ERROR,
	HDCP_TRAN_ERR_HPD_IS_DOWN,
	HDCP_TRAN_ERR_SRM_FAILURE,
	HDCP_TRAN_ERR_SIGNATURE_VERIFICATION,
	HDCP_TRAN_ERR_H_TAG_DIFF_H,
	HDCP_TRAN_ERR_V_TAG_DIFF_V,
	HDCP_TRAN_ERR_LOCALITY_CHECK,
	HDCP_TRAN_ERR_DDC,
	HDCP_TRAN_ERR_REAUTH_REQ,
	HDCP_TRAN_ERR_TOPOLOGY,
	HDCP_TRAN_ERR_HDCP_RSVD1,
	HDCP_TRAN_ERR_HDMI_CAPABILITY,
	HDCP_TRAN_ERR_RI,
	HDCP_TRAN_ERR_WATCHDOG_EXPIRED,
};

static char const *g_last_error[16] = {
	"No Error",
	"HPD is down",
	"SRM failure",
	"Signature verification error",
	"h tag != h",
	"V tag diff v",
	"Locality check",
	"DDC error",
	"REAUTH_REQ",
	"Topology error",
	"Verify receiver ID list failed",
	"HDCP_RSVD1 was not 0,0,0",
	"HDMI capability or mode",
	"RI result was different than expected",
	"WatchDog expired",
	"Repeater integrity failed"
};

#define HDCP_MAX_RECEIVERS 32
#define HDCP_RECEIVER_ID_SIZE_BYTES 5
#define HPD_EVENT 1
#define HDCP_STATUS_SIZE         0x5
#define HDCP_PORT_STS_AUTH       0x1
#define HDCP_PORT_STS_REPEATER   0x2
#define HDCP_PORT_STS_TYPE_MASK  0xc
#define HDCP_PORT_STS_TYPE_SHIFT 0x2
#define HDCP_PORT_STS_AUTH_STREAM_ID_SHIFT 0x4
#define HDCP_PORT_STS_AUTH_STREAM_ID_MASK 0x10
#define HDCP_PORT_STS_LAST_ERR_SHIFT 0x5
#define HDCP_PORT_STS_LAST_ERR_MASK  (0x0F << 5)
#define GET_HDCP_PORT_STS_LAST_ERR(__sts__) \
	(((__sts__) & HDCP_PORT_STS_LAST_ERR_MASK) >> \
	 HDCP_PORT_STS_LAST_ERR_SHIFT)
#define HDCP_PORT_STS_1_1_FEATURES   0x200

#define HDCP_CONFIG_NONE    ((u8) 0)
#define HDCP_CONFIG_1_4     ((u8) 1) /* use HDCP 1.4 only */
#define HDCP_CONFIG_2_2     ((u8) 2) /* use HDCP 2.2 only */

/* Default timeout to use for wait4event in milliseconds */
#define HDCP_EVENT_TO_DEF 800
/* Timeout value to use for repeater receiver ID check, spec says 3s */
#define HDCP_EVENT_TO_RPT 3500

static int cdns_hdcp_check_link(struct cdns_mhdp_device *mhdp);

static void print_port_status(u16 sts)
{
	char const *rx_type[4] = { "Unknown", "HDCP 1", "HDCP 2", "Unknown" };

	DRM_DEBUG_KMS("INFO: HDCP Port Status: 0x%04x\n", sts);
	DRM_DEBUG_KMS(" Authenticated: %d\n", sts & HDCP_PORT_STS_AUTH);
	DRM_DEBUG_KMS(" Receiver is repeater: %d\n", sts & HDCP_PORT_STS_REPEATER);
	DRM_DEBUG_KMS(" RX Type: %s\n",
				rx_type[(sts & HDCP_PORT_STS_TYPE_MASK) >> HDCP_PORT_STS_TYPE_SHIFT]);
	DRM_DEBUG_KMS(" AuthStreamId: %d\n", sts & HDCP_PORT_STS_AUTH_STREAM_ID_MASK);
	DRM_DEBUG_KMS(" Last Error: %s\n",
				g_last_error[(sts & HDCP_PORT_STS_LAST_ERR_MASK) >> HDCP_PORT_STS_LAST_ERR_SHIFT]);
	DRM_DEBUG_KMS(" Enable 1.1 Features: %d\n", sts & HDCP_PORT_STS_1_1_FEATURES);
}

static void print_events(u8 events)
{
	if (events & HDMI_TX_HPD_EVENT)
		DRM_INFO("INFO: HDMI_TX_HPD_EVENT\n");
	if (events & HDCPTX_STATUS_EVENT)
		DRM_INFO("INFO: HDCPTX_STATUS_EVENT\n");
	if (events & HDCPTX_IS_KM_STORED_EVENT)
		DRM_INFO("INFO: HDCPTX_IS_KM_STORED_EVENT\n");
	if (events & HDCPTX_STORE_KM_EVENT)
		DRM_INFO("INFO: HDCPTX_STORE_KM_EVENT\n");
	if (events & HDCPTX_IS_RECEIVER_ID_VALID_EVENT)
		DRM_INFO("INFO: HDCPTX_IS_RECEIVER_ID_VALID_EVENT\n");
}

static u8 wait4event(struct cdns_mhdp_device *mhdp, u8 *events,
			       u32 event_to_wait, u32 timeout_ms)
{
	u8 reg_events;
	u8 returned_events;
	u8 event_mask = event_to_wait | HDCPTX_STATUS_EVENT;
	unsigned timeout;

	timeout = timeout_ms;
	do {
		if (timeout == 0)
			goto timeout_err;
		timeout--;
		udelay(1000);
		reg_events = cdns_mhdp_get_event(mhdp);
		*events |= reg_events;
	} while (((event_mask & *events) == 0) && (event_to_wait > HDMI_TX_HPD_EVENT));

	returned_events = *events & event_mask;
	if (*events != returned_events) {
		u32 unexpected_events = ~event_mask & *events;

		DRM_INFO("INFO: %s() all 0x%08x  expected 0x%08x unexpected 0x%08x",
			 __func__, *events, returned_events, unexpected_events);
		DRM_INFO("INFO: %s() All events:\n", __func__);
		print_events(*events);

		DRM_INFO("INFO: %s() expected events:\n", __func__);
		print_events(returned_events);

		DRM_INFO("INFO: %s() unexpected events:\n", __func__);
		print_events(unexpected_events);
	} else
		print_events(*events);

	*events &= ~event_mask;

	return returned_events;

timeout_err:
	DRM_INFO("INFO: %s() Timed out with events:\n", __func__);
	print_events(event_to_wait);
	return 0;
}

static u16 cdns_hdcp_get_status(struct cdns_mhdp_device *mhdp)
{
	u8 hdcp_status[HDCP_STATUS_SIZE];
	u16 hdcp_port_status;

	cdns_mhdp_hdcp_tx_status_req(mhdp, hdcp_status, HDCP_STATUS_SIZE);
	hdcp_port_status = (hdcp_status[0] << 8) | hdcp_status[1];

	return hdcp_port_status;
}

static inline u8 check_event(u8 events, u8 tested)
{
	if ((events & tested) == 0)
		return 0;
	return 1;
}

/* Prints status. Returns error code (0 = no error) */
static u8 cdns_hdcp_handle_status(u16 status)
{
	print_port_status(status);
	if (status & HDCP_PORT_STS_LAST_ERR_MASK)
		DRM_ERROR("ERROR: HDCP error was set to %s\n",
			  g_last_error[((status & HDCP_PORT_STS_LAST_ERR_MASK)
					>> HDCP_PORT_STS_LAST_ERR_SHIFT)]);
	return GET_HDCP_PORT_STS_LAST_ERR(status);
}

static int cdns_hdcp_set_config(struct cdns_mhdp_device *mhdp, u8 hdcp_config)
{
	u8 bus_config, retEvents;
	u16 hdcp_port_status;
	int ret;

	/* Clearing out existing events */
	wait4event(mhdp, &mhdp->hdcp.events, HDMI_TX_HPD_EVENT, HDCP_EVENT_TO_DEF);
	mhdp->hdcp.events = 0;

	if (!strncmp("imx8mq-hdmi", mhdp->plat_data->plat_name, 11)) {
		DRM_DEBUG_KMS("INFO: Switching HDCP Commands to SAPB.\n");
		bus_config = (1 << GENERAL_BUS_SETTINGS_HDCP_BUS_BIT);
		ret = cdns_mhdp_apb_conf(mhdp, bus_config);
		if (ret) {
			DRM_ERROR("Failed to set APB configuration.\n");
			if (ret & (1 << GENERAL_BUS_SETTINGS_RESP_HDCP_BUS_BIT))/* 1 - locked */
				DRM_ERROR("Failed to switch HDCP to SAPB Mailbox\n");
			return -1;
		}
		DRM_DEBUG_KMS("INFO: HDCP switched to SAPB\n");
	}

	/* HDCP 2.2(and/or 1.4) | activate | km-key |  0 */
	hdcp_config |=  CDNS_HDCP_ACTIVATE | (HDCP_USE_KMKEY << 4) | (HDCP_CONTENT_TYPE_0 << 3);

	DRM_DEBUG_KMS("INFO: Enabling HDCP...\n");
	ret = cdns_mhdp_hdcp_tx_config(mhdp, hdcp_config);
	if (ret < 0)
		DRM_DEBUG_KMS("cdns_mhdp_hdcp_tx_config failed\n");

	/* Wait until HDCP_TX_STATUS EVENT appears */
	DRM_DEBUG_KMS("INFO: wait4event -> HDCPTX_STATUS_EVENT\n");
	retEvents = wait4event(mhdp, &mhdp->hdcp.events, HDCPTX_STATUS_EVENT, HDCP_EVENT_TO_DEF);

	/* Set TX STATUS REQUEST */
	DRM_DEBUG_KMS("INFO: Getting port status\n");
	hdcp_port_status = cdns_hdcp_get_status(mhdp);
	if (cdns_hdcp_handle_status(hdcp_port_status) != 0)
		return -1;

	return 0;
}

static int cdns_hdcp_auth_check(struct cdns_mhdp_device *mhdp)
{
	u16 hdcp_port_status;
	int ret;

	DRM_DEBUG_KMS("INFO: wait4event -> HDCPTX_STATUS_EVENT\n");
	mhdp->hdcp.events = wait4event(mhdp, &mhdp->hdcp.events, HDCPTX_STATUS_EVENT, HDCP_EVENT_TO_DEF+HDCP_EVENT_TO_DEF);
	if (mhdp->hdcp.events == 0)
		return -1;

	DRM_DEBUG_KMS("HDCP: HDCPTX_STATUS_EVENT\n");
	hdcp_port_status = cdns_hdcp_get_status(mhdp);
	ret = cdns_hdcp_handle_status(hdcp_port_status);
	if (ret != 0) {
		if (ret == HDCP_TRAN_ERR_REAUTH_REQ) {
			DRM_ERROR("HDCP_TRAN_ERR_REAUTH_REQ-->one more try!\n");
			return 1;
		} else
			return -1;
	}

	if (hdcp_port_status & HDCP_PORT_STS_AUTH) {
		DRM_INFO("Authentication completed successfully!\n");
		/* Dump hdmi and phy register */
		mhdp->hdcp.state = HDCP_STATE_AUTHENTICATED;
		mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_ENABLED;
		schedule_work(&mhdp->hdcp.prop_work);
		return 0;
	}

	DRM_WARN("Authentication failed\n");
	mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;
	return -1;
}

inline void cdns_hdcp_swap_id(u8 *in, u8 *out)
{
	int i;

	for (i = 0; i < HDCP_RECEIVER_ID_SIZE_BYTES; i++)
		out[HDCP_RECEIVER_ID_SIZE_BYTES - (i + 1)] = in[i];
}

inline void cdns_hdcp_swap_list(u8 *list_in, u8 *list_out, int num_ids)
{
	int i;

	for (i = 0; i < num_ids; i++)
		cdns_hdcp_swap_id(&list_in[i * HDCP_RECEIVER_ID_SIZE_BYTES],
				 &list_out[i * HDCP_RECEIVER_ID_SIZE_BYTES]);
}

static int cdns_hdcp_check_receviers(struct cdns_mhdp_device *mhdp)
{
	u8 ret_events;
	u8 hdcp_num_rec, i;
	u8 hdcp_rec_id[HDCP_MAX_RECEIVERS][HDCP_RECEIVER_ID_SIZE_BYTES];
	u8 hdcp_rec_id_temp[HDCP_MAX_RECEIVERS][HDCP_RECEIVER_ID_SIZE_BYTES];
	u16 hdcp_port_status = 0;
	int ret;

	DRM_INFO("INFO: Waiting for Receiver ID valid event\n");
	ret_events = 0;
	do {
		u8 events = 0;
		u8 hdcp_last_error = 0;
		events = check_event(ret_events,
				     HDCPTX_IS_RECEIVER_ID_VALID_EVENT);
		DRM_DEBUG_KMS("INFO: Waiting HDCPTX_IS_RECEIVER_ID_VALID_EVENT\n");
		ret_events = wait4event(mhdp, &mhdp->hdcp.events,
					HDCPTX_IS_RECEIVER_ID_VALID_EVENT,
					(mhdp->hdcp.sink_is_repeater ?
					 HDCP_EVENT_TO_RPT : HDCP_EVENT_TO_DEF));
		if (ret_events == 0) {
			/* time out occurred, return error */
			DRM_ERROR("HDCP error did not get receiver IDs\n");
			return -1;
		}
		if (check_event(ret_events, HDCPTX_STATUS_EVENT) != 0) {
			/* There was a status update, could be due to HPD
			   going down or some other error, check if an error
			   was set, if so exit.
			*/
			hdcp_port_status = cdns_hdcp_get_status(mhdp);
			hdcp_last_error = GET_HDCP_PORT_STS_LAST_ERR(hdcp_port_status);
			if (cdns_hdcp_handle_status(hdcp_port_status)) {
				DRM_ERROR("HDCP error no: %u\n", hdcp_last_error);
				return -1;
			} else {
				/* No error logged, keep going.
				 * If this somehow happened at same time, then need to
				 * put the HDCPTX_STATUS_EVENT back into the global
				 * events pool and checked later. */
				mhdp->hdcp.events |= HDCPTX_STATUS_EVENT;

				/* Special condition when connected to HDCP 1.4 repeater
				 * with no downstream devices attached, then will not
				 * get receiver ID list but instead will reach
				 * authenticated state. */
				if ((mhdp->hdcp.hdcp_version == HDCP_TX_1) && (mhdp->hdcp.sink_is_repeater == 1) &&
				    ((hdcp_port_status & HDCP_PORT_STS_AUTH) == HDCP_PORT_STS_AUTH)) {
					DRM_INFO("Connected to HDCP 1.4 repeater with no downstream devices!\n");
					return 0;
				}

				msleep(20);
			}
		}
	} while (check_event(ret_events,
			     HDCPTX_IS_RECEIVER_ID_VALID_EVENT) == 0);

	DRM_INFO("INFO: Requesting Receivers ID's\n");

	hdcp_num_rec = 0;
	memset(&hdcp_rec_id, 0, sizeof(hdcp_rec_id));

	ret = cdns_mhdp_hdcp_tx_is_receiver_id_valid(mhdp, (u8 *)hdcp_rec_id, &hdcp_num_rec);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to hdcp tx receiver ID.\n");
		return -1;
	}

	if (hdcp_num_rec == 0) {
		DRM_DEBUG_KMS("WARN: Failed to get receiver list\n");
		/* Unknown problem, return error */
		return -1;
	}

	DRM_INFO("INFO: Number of Receivers: %d\n", hdcp_num_rec);

	for (i = 0; i < hdcp_num_rec; ++i) {
		DRM_INFO("\tReceiver ID%2d: %.2X%.2X%.2X%.2X%.2X\n",
			 i,
			 hdcp_rec_id[i][0],
			 hdcp_rec_id[i][1],
			 hdcp_rec_id[i][2],
			 hdcp_rec_id[i][3],
			 hdcp_rec_id[i][4]
			);
	}

	/* swap ids byte order */
	cdns_hdcp_swap_list(&hdcp_rec_id[0][0],
			   &hdcp_rec_id_temp[0][0], hdcp_num_rec);

	/* Check Receiver ID's against revocation list in SRM */
	if (drm_hdcp_check_ksvs_revoked(mhdp->drm_dev, (u8 *)hdcp_rec_id_temp, hdcp_num_rec)) {
		mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;
		DRM_ERROR("INFO: Receiver check fails\n");
		return -1;
	}

	ret = cdns_mhdp_hdcp_tx_respond_receiver_id_valid(mhdp, 1);
	DRM_INFO("INFO: Responding with Receiver ID's OK!, ret=%d\n", ret);
	return ret;
}

#ifdef STORE_PAIRING
static int cdns_hdcp_get_stored_pairing(struct cdns_mhdp_device *mhdp)
{
	int ret = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(IMX_FW_TIMEOUT_MS);
	unsigned long sleep = 1000;
	const struct firmware *fw;

	DRM_DEBUG_KMS("%s()\n", __func__);

	while (time_before(jiffies, timeout)) {
		ret = request_firmware(&fw, cdns_hdcp_PAIRING_FIRMWARE, mhdp->dev);
		if (ret == -ENOENT) {
			msleep(sleep);
			sleep *= 2;
			continue;
		} else if (ret) {
			DRM_DEV_INFO(mhdp->dev, "HDCP pairing data not found\n");
			goto out;
		}

		mhdp->hdcp.num_paired = fw->size /
			sizeof(struct hdcp_trans_pairing_data);
		if (mhdp->hdcp.num_paired > MAX_STORED_KM) {
			/* todo: handle dropping */
			mhdp->hdcp.num_paired = MAX_STORED_KM;
			DRM_DEV_INFO(mhdp->dev,
				     "too many paired receivers - dropping older entries\n");
		}
		memcpy(&mhdp->hdcp.pairing[0], fw->data,
		       sizeof(struct hdcp_trans_pairing_data) * mhdp->hdcp.num_paired);
		release_firmware(fw);
		goto out;
	}

	DRM_DEV_ERROR(mhdp->dev, "Timed out trying to load firmware\n");
	ret = -ETIMEDOUT;
	out:
	return ret;
}
#endif

static int cdns_hdcp_find_km_store(struct cdns_mhdp_device *mhdp,
				  u8 receiver[HDCP_PAIRING_R_ID])
{
	int i;

	DRM_DEBUG_KMS("%s()\n", __func__);
	for (i = 0; i < mhdp->hdcp.num_paired; i++) {
		if (memcmp(receiver, mhdp->hdcp.pairing[i].receiver_id,
			   HDCP_PAIRING_R_ID) == 0) {
			DRM_INFO("HDCP: found receiver id: 0x%x%x%x%x%x\n",
			receiver[0], receiver[1], receiver[2], receiver[3], receiver[4]);
			return i;
		}
	}
	DRM_INFO("HDCP: receiver id: 0x%x%x%x%x%x not stored\n",
		 receiver[0], receiver[1], receiver[2], receiver[3], receiver[4]);
	return -1;
}

static int cdns_hdcp_store_km(struct cdns_mhdp_device *mhdp,
			     struct hdcp_trans_pairing_data *pairing,
			     int stored_km_index)
{
	int i, temp_index;
	struct hdcp_trans_pairing_data temp_pairing;

	DRM_DEBUG_KMS("%s()\n", __func__);

	if (stored_km_index < 0) {
		/* drop one entry if array is full */
		if (mhdp->hdcp.num_paired == MAX_STORED_KM)
			mhdp->hdcp.num_paired--;

		temp_index = mhdp->hdcp.num_paired;
		mhdp->hdcp.num_paired++;
		if (!pairing) {
			DRM_ERROR("NULL HDCP pairing data!\n");
			return  -1;
		} else
			/* save the new stored km */
			temp_pairing = *pairing;
	} else {
		/* save the current stored km */
		temp_index = stored_km_index;
		temp_pairing = mhdp->hdcp.pairing[stored_km_index];
	}

	/* move entries one slot to the end */
	for (i = temp_index; i > 0; i--)
		mhdp->hdcp.pairing[i] = mhdp->hdcp.pairing[i - 1];

	/* save the current/new entry at the beginning */
	mhdp->hdcp.pairing[0] = temp_pairing;

	return 0;
}

static inline int cdns_hdcp_auth_22(struct cdns_mhdp_device *mhdp)
{
	int km_idx = -1;
	u8 retEvents;
	u16 hdcp_port_status;
	u8 resp[HDCP_STATUS_SIZE];
	struct hdcp_trans_pairing_data pairing;
	int ret;

	DRM_DEBUG_KMS("HDCP: Start 2.2 Authentication\n");
	mhdp->hdcp.sink_is_repeater = 0;

	/* Wait until HDCP2_TX_IS_KM_STORED EVENT appears */
	retEvents = 0;
	DRM_DEBUG_KMS("INFO: Wait until HDCP2_TX_IS_KM_STORED EVENT appears\n");
	while (check_event(retEvents, HDCPTX_IS_KM_STORED_EVENT) == 0) {
		DRM_DEBUG_KMS("INFO: Waiting FOR _IS_KM_STORED EVENT\n");
		retEvents = wait4event(mhdp, &mhdp->hdcp.events,
				       HDCPTX_IS_KM_STORED_EVENT, HDCP_EVENT_TO_DEF);
		if (retEvents == 0)
			/* time out occurred, return error */
			return -1;
		if (check_event(retEvents, HDCPTX_STATUS_EVENT) != 0) {
			/* There was a status update, could be due to HPD
			   going down or some other error, check if an error
			   was set, if so exit.
			*/
			hdcp_port_status = cdns_hdcp_get_status(mhdp);
			if (cdns_hdcp_handle_status(hdcp_port_status) != 0)
				return -1;
		}
	}

	DRM_DEBUG_KMS("HDCP: HDCPTX_IS_KM_STORED_EVENT\n");

	/* Set HDCP2 TX KM STORED REQUEST */
	ret = cdns_mhdp_hdcp2_tx_is_km_stored_req(mhdp, resp, HDCP_STATUS_SIZE);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to hdcp2 tx km stored.\n");
		return -1;
	}

	DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_IS_KM_STORED_REQ_blocking\n");
	DRM_DEBUG_KMS("HDCP: Receiver ID: 0x%x%x%x%x%x\n",
		      resp[0], resp[1], resp[2], resp[3], resp[4]);

	km_idx = cdns_hdcp_find_km_store(mhdp, resp);

	/* Check if KM is stored */
	if (km_idx >= 0) {
		DRM_DEBUG_KMS("INFO: KM is stored\n");
		/* Set HDCP2 TX RESPOND KM with stored KM */
		ret = cdns_mhdp_hdcp2_tx_respond_km(mhdp, (u8 *)&mhdp->hdcp.pairing[km_idx],
				sizeof(struct hdcp_trans_pairing_data));

		DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_RESPOND_KM_blocking, ret=%d\n", ret);
	} else { /* KM is not stored */
		/* Set HDCP2 TX RESPOND KM with empty data */
		ret = cdns_mhdp_hdcp2_tx_respond_km(mhdp, NULL, 0);
		DRM_DEBUG_KMS("INFO: KM is not stored ret=%d\n", ret);
	}

	if (cdns_hdcp_check_receviers(mhdp))
		return -1;

	/* Check if KM is not stored */
	if (km_idx < 0) {
		int loop_cnt = 0;

		/* Wait until HDCP2_TX_STORE_KM EVENT appears */
		retEvents = 0;
		DRM_DEBUG_KMS("INFO: wait4event -> HDCPTX_STORE_KM_EVENT\n");
		while (check_event(retEvents, HDCPTX_STORE_KM_EVENT) == 0) {
			retEvents = wait4event(mhdp, &mhdp->hdcp.events,
					       HDCPTX_STORE_KM_EVENT, HDCP_EVENT_TO_DEF);
			if (check_event(retEvents, HDCPTX_STATUS_EVENT)
			    != 0) {
				hdcp_port_status = cdns_hdcp_get_status(mhdp);
				if (cdns_hdcp_handle_status(hdcp_port_status)
				    != 0)
					return -1;
			}
			if (loop_cnt > 2) {
				DRM_ERROR("Did not get event HDCPTX_STORE_KM_EVENT in time\n");
				return -1;
			} else
				loop_cnt++;
		}
		DRM_DEBUG_KMS("HDCP: HDCPTX_STORE_KM_EVENT\n");

		/* Set HDCP2_TX_STORE_KM REQUEST */
		ret = cdns_mhdp_hdcp2_tx_store_km(mhdp, (u8 *)&pairing, sizeof(struct  hdcp_trans_pairing_data));
		DRM_DEBUG_KMS("HDCP: CDN_API_HDCP2_TX_STORE_KM_REQ_blocking ret=%d\n", ret);
		cdns_hdcp_store_km(mhdp, &pairing, km_idx);
	} else
		cdns_hdcp_store_km(mhdp, NULL, km_idx);

	/* Check if device was a repeater */
	hdcp_port_status = cdns_hdcp_get_status(mhdp);

	/* Exit if there was any errors logged at this point... */
	if (GET_HDCP_PORT_STS_LAST_ERR(hdcp_port_status) > 0) {
		cdns_hdcp_handle_status(hdcp_port_status);
		return -1;
	}

	if (hdcp_port_status & HDCP_PORT_STS_REPEATER)
		mhdp->hdcp.sink_is_repeater = 1;

	/* If sink was a repeater, we will be getting additional IDs to validate...
	 * Note that this one may take some time since spec allows up to 3s... */
	if (mhdp->hdcp.sink_is_repeater)
		if (cdns_hdcp_check_receviers(mhdp))
			return -1;

	/* Slight delay to allow firmware to finish setting up authenticated state */
	msleep(300);

	DRM_INFO("Finished cdns_hdcp_auth_22\n");
	return 0;
}

static inline int cdns_hdcp_auth_14(struct cdns_mhdp_device *mhdp)
{
	u16 hdcp_port_status;
	int ret = 0;

	DRM_DEBUG_KMS("HDCP: Starting 1.4 Authentication\n");
	mhdp->hdcp.sink_is_repeater = 0;

	ret = cdns_hdcp_check_receviers(mhdp);
	if (ret)
		return -1;

	/* Check if device was a repeater */
	hdcp_port_status = cdns_hdcp_get_status(mhdp);

	/* Exit if there was any errors logged at this point... */
	if (GET_HDCP_PORT_STS_LAST_ERR(hdcp_port_status) > 0) {
		cdns_hdcp_handle_status(hdcp_port_status);
		return -1;
	}

	if (hdcp_port_status & HDCP_PORT_STS_REPEATER) {
		DRM_INFO("Connected to a repeater\n");
		mhdp->hdcp.sink_is_repeater = 1;
	} else
		DRM_INFO("Connected to a normal sink\n");

	/* If sink was a repeater, we will be getting additional IDs to validate...
	 * Note that this one may take some time since spec allows up to 3s... */
	if (mhdp->hdcp.sink_is_repeater)
		ret = cdns_hdcp_check_receviers(mhdp);

	/* Slight delay to allow firmware to finish setting up authenticated state */
	msleep(300);

	return ret;
}

static int cdns_hdcp_auth(struct cdns_mhdp_device *mhdp, u8 hdcp_config)
{
	int ret = 0;

	DRM_DEBUG_KMS("HDCP: Start Authentication\n");

	if (mhdp->hdcp.reauth_in_progress == 0) {
		ret = cdns_hdcp_set_config(mhdp, hdcp_config);
		if (ret) {
			DRM_ERROR("cdns_hdcp_set_config failed\n");
			return -1;
		}
	}

	mhdp->hdcp.reauth_in_progress = 0;
	mhdp->hdcp.sink_is_repeater   = 0;
	mhdp->hdcp.hdcp_version       = hdcp_config;

	do {
		if (mhdp->hdcp.cancel == 1) {
			DRM_ERROR("mhdp->hdcp.cancel is TRUE\n");
			return -ECANCELED;
		}

		if (hdcp_config == HDCP_TX_1)
			ret = cdns_hdcp_auth_14(mhdp);
		else
			ret = cdns_hdcp_auth_22(mhdp);
		if (ret) {
			u16 hdcp_port_status;
			DRM_ERROR("cdns_hdcp_auth_%s failed\n",
				  (hdcp_config == HDCP_TX_1) ? "14" : "22");
			hdcp_port_status = cdns_hdcp_get_status(mhdp);
			cdns_hdcp_handle_status(hdcp_port_status);
			return -1;
		}

		ret = cdns_hdcp_auth_check(mhdp);
	} while (ret == 1);

	return ret;
}

static int _cdns_hdcp_disable(struct cdns_mhdp_device *mhdp)
{
	int ret = 0;
	u8 hdcp_cfg = (HDCP_USE_KMKEY << 4);

	DRM_DEBUG_KMS("[%s:%d] HDCP is being disabled...\n",
		      mhdp->connector.base.name, mhdp->connector.base.base.id);
	DRM_DEBUG_KMS("INFO: Disabling HDCP...\n");

	ret = cdns_mhdp_hdcp_tx_config(mhdp, hdcp_cfg);
	if (ret < 0)
		DRM_DEBUG_KMS("cdns_mhdp_hdcp_tx_config failed\n");

	DRM_DEBUG_KMS("HDCP is disabled\n");

	mhdp->hdcp.events = 0;

	return ret;
}

static int _cdns_hdcp_enable(struct cdns_mhdp_device *mhdp)
{
	int i, ret = 0, tries = 9, tries14 = 50;
	u8 hpd_sts;

	hpd_sts = cdns_mhdp_read_hpd(mhdp);
	if (hpd_sts == 0) {
		dev_info(mhdp->dev, "%s HDP detected low, set state to DISABLING\n", __func__);
		mhdp->hdcp.state = HDCP_STATE_DISABLING;
		return -1;
	}

	DRM_DEBUG_KMS("[%s:%d] HDCP is being enabled...\n",
		      mhdp->connector.base.name, mhdp->connector.base.base.id);

	mhdp->hdcp.events = 0;

	/* Incase of authentication failures, HDCP spec expects reauth. */
	/* TBD should this actually try 2.2 n times then 1.4? */
	for (i = 0; i < tries; i++) {
		if (mhdp->hdcp.config & HDCP_CONFIG_2_2) {
			ret = cdns_hdcp_auth(mhdp, HDCP_TX_2);
			if (ret == 0)
				return 0;
			else if (ret == -ECANCELED)
				return ret;
			_cdns_hdcp_disable(mhdp);
		}
	}

	for (i = 0; i < tries14; i++) {
		if (mhdp->hdcp.config & HDCP_CONFIG_1_4) {
			ret = cdns_hdcp_auth(mhdp, HDCP_TX_1);
			if (ret == 0)
				return 0;
			else if (ret == -ECANCELED)
				return ret;
			_cdns_hdcp_disable(mhdp);
		}
		DRM_DEBUG_KMS("HDCP Auth failure (%d)\n", ret);
	}

	DRM_ERROR("HDCP authentication failed (%d tries/%d)\n", tries, ret);
	return ret;
}

static void cdns_hdcp_check_work(struct work_struct *work)
{
	struct cdns_mhdp_hdcp *hdcp = container_of(work,
					     struct cdns_mhdp_hdcp, check_work.work);
	struct cdns_mhdp_device *mhdp = container_of(hdcp,
					   struct cdns_mhdp_device, hdcp);

	/* todo: maybe we don't need to always schedule */
	cdns_hdcp_check_link(mhdp);
	schedule_delayed_work(&hdcp->check_work, 50);
}

static void cdns_hdcp_prop_work(struct work_struct *work)
{
	struct cdns_mhdp_hdcp *hdcp = container_of(work,
					     struct cdns_mhdp_hdcp, prop_work);
	struct cdns_mhdp_device *mhdp = container_of(hdcp,
					   struct cdns_mhdp_device, hdcp);

	struct drm_device *dev = mhdp->drm_dev;

	drm_modeset_lock(&dev->mode_config.connection_mutex, NULL);
	mutex_lock(&mhdp->hdcp.mutex);

	/*
	 * This worker is only used to flip between ENABLED/DESIRED. Either of
	 * those to UNDESIRED is handled by core. If hdcp_value == UNDESIRED,
	 * we're running just after hdcp has been disabled, so just exit
	 */
	if (mhdp->hdcp.value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		drm_hdcp_update_content_protection(&mhdp->connector.base,
				mhdp->hdcp.value);
	}

	mutex_unlock(&mhdp->hdcp.mutex);
	drm_modeset_unlock(&dev->mode_config.connection_mutex);
}

static void show_hdcp_supported(struct cdns_mhdp_device *mhdp)
{
	if ((mhdp->hdcp.config & (HDCP_CONFIG_1_4 | HDCP_CONFIG_2_2)) ==
		    (HDCP_CONFIG_1_4 | HDCP_CONFIG_2_2))
		DRM_INFO("Both HDCP 1.4 and 2.2 are enabled\n");
	else if (mhdp->hdcp.config & HDCP_CONFIG_1_4)
		DRM_INFO("Only HDCP 1.4 is enabled\n");
	else if (mhdp->hdcp.config & HDCP_CONFIG_2_2)
		DRM_INFO("Only HDCP 2.2 is enabled\n");
	else
		DRM_INFO("HDCP is disabled\n");
}

static ssize_t HDCPTX_do_reauth_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count);
static struct device_attribute HDCPTX_do_reauth = __ATTR_WO(HDCPTX_do_reauth);

static ssize_t HDCPTX_do_reauth_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	ret = cdns_mhdp_hdcp_tx_reauth(mhdp, 1);
	if (ret < 0) {
		dev_err(dev, "%s cdns_mhdp_hdcp_tx_reauth failed\n", __func__);
		return -1;
	}

	return count;
}

static ssize_t HDCPTX_Version_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t HDCPTX_Version_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count);
static struct device_attribute HDCPTX_Version = __ATTR_RW(HDCPTX_Version);

static ssize_t HDCPTX_Version_store(struct device *dev,
				    struct device_attribute *attr, const char *buf, size_t count)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int value, ret;

	ret = kstrtoint(buf, 10, &value);
	if (ret != 0)
		return -EINVAL;

	if (value == 2)
		mhdp->hdcp.config = 2;
	else if (value == 1)
		mhdp->hdcp.config = 1;
	else if (value == 3)
		mhdp->hdcp.config = 3;
	else
		mhdp->hdcp.config = 0;

	return count;
}

ssize_t HDCPTX_Version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mhdp->hdcp.config);
}

static ssize_t HDCPTX_Status_show(struct device *dev,
				  struct device_attribute *attr, char *buf);
static ssize_t HDCPTX_Status_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count);
static struct device_attribute HDCPTX_Status = __ATTR_RW(HDCPTX_Status);

ssize_t HDCPTX_Status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	switch (mhdp->hdcp.state) {
	case HDCP_STATE_NO_AKSV:
		return sprintf(buf, "%d :HDCP_STATE_NO_AKSV\n", mhdp->hdcp.state);
	case HDCP_STATE_INACTIVE:
		return sprintf(buf, "%d :HDCP_STATE_INACTIVE\n", mhdp->hdcp.state);
	case HDCP_STATE_ENABLING:
		return sprintf(buf, "%d :HDCP_STATE_ENABLING\n", mhdp->hdcp.state);
	case HDCP_STATE_AUTHENTICATING:
		return sprintf(buf, "%d :HDCP_STATE_AUTHENTICATING\n", mhdp->hdcp.state);
	case HDCP_STATE_AUTHENTICATED:
		return sprintf(buf, "%d :HDCP_STATE_AUTHENTICATED\n", mhdp->hdcp.state);
	case HDCP_STATE_DISABLING:
		return sprintf(buf, "%d :HDCP_STATE_DISABLING\n", mhdp->hdcp.state);
	case HDCP_STATE_AUTH_FAILED:
		return sprintf(buf, "%d :HDCP_STATE_AUTH_FAILED\n", mhdp->hdcp.state);
	default:
		return sprintf(buf, "%d :HDCP_STATE don't exist\n", mhdp->hdcp.state);
	}
}

ssize_t HDCPTX_Status_store(struct device *dev,
			    struct device_attribute *attr, const char *buf, size_t count)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int value, ret;

	if (count == 2) {
		ret = kstrtoint(buf, 10, &value);
		if (ret != 0)
			return -EINVAL;

		if ((value >= HDCP_STATE_NO_AKSV) && (value <= HDCP_STATE_AUTH_FAILED)) {
			mhdp->hdcp.state = value;
			return count;
		}
		dev_err(dev, "%s &hdp->state invalid\n", __func__);
		return -1;
	}

	dev_info(dev, "%s &hdp->state desired %s count=%d\n ", __func__, buf, (int)count);

	if (strncmp(buf, "HDCP_STATE_NO_AKSV", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_NO_AKSV;
	else if (strncmp(buf, "HDCP_STATE_INACTIVE", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_INACTIVE;
	else if (strncmp(buf, "HDCP_STATE_ENABLING", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_ENABLING;
	else if (strncmp(buf, "HDCP_STATE_AUTHENTICATING", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_AUTHENTICATING;
	else if (strncmp(buf, "HDCP_STATE_AUTHENTICATED", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_AUTHENTICATED;
	else if (strncmp(buf, "HDCP_STATE_DISABLING", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_DISABLING;
	else if (strncmp(buf, "HDCP_STATE_AUTH_FAILED", count - 1) == 0)
		mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;
	else
		dev_err(dev, "%s &hdp->state invalid\n", __func__);
	return -1;
}

void cnds_hdcp_create_device_files(struct cdns_mhdp_device *mhdp)
{

	if (device_create_file(mhdp->dev, &HDCPTX_do_reauth)) {
		DRM_ERROR("Unable to create HDCPTX_do_reauth sysfs\n");
		device_remove_file(mhdp->dev, &HDCPTX_do_reauth);
	}

	if (device_create_file(mhdp->dev, &HDCPTX_Version)) {
		DRM_ERROR("Unable to create HDCPTX_Version sysfs\n");
		device_remove_file(mhdp->dev, &HDCPTX_Version);
	}

	if (device_create_file(mhdp->dev, &HDCPTX_Status)) {
		DRM_ERROR(KERN_ERR "Unable to create HDCPTX_Status sysfs\n");
		device_remove_file(mhdp->dev, &HDCPTX_Status);
	}
}
EXPORT_SYMBOL(cnds_hdcp_create_device_files);

void cnds_hdcp_remove_device_files(struct cdns_mhdp_device *mhdp)
{
	device_remove_file(mhdp->dev, &HDCPTX_Status);
	device_remove_file(mhdp->dev, &HDCPTX_Version);
	device_remove_file(mhdp->dev, &HDCPTX_do_reauth);
}
EXPORT_SYMBOL(cnds_hdcp_remove_device_files);

#ifdef DEBUG
void cdns_hdcp_show_pairing(struct cdns_mhdp_device *mhdp, struct hdcp_trans_pairing_data *p)
{
	char s[80];
	int i, k;

	DRM_INFO("Reveiver ID: %.2X%.2X%.2X%.2X%.2X\n",
		 p->receiver_id[0],
		 p->receiver_id[1],
		 p->receiver_id[2],
		 p->receiver_id[3],
		 p->receiver_id[4]);
	for (k = 0, i = 0; k < 16; k++)
		i += snprintf(&s[i], sizeof(s), "%02x", p->m[k]);

	DRM_INFO("\tm: %s\n", s);

	for (k = 0, i = 0; k < 16; k++)
		i += snprintf(&s[i], sizeof(s), "%02x", p->km[k]);

	DRM_INFO("\tkm: %s\n", s);

	for (k = 0, i = 0; k < 16; k++)
		i += snprintf(&s[i], sizeof(s), "%02x", p->ekh[k]);

	DRM_INFO("\tekh: %s\n", s);
}
#endif

static int cdns_hdcp_dump_pairing(struct seq_file *s, void *data)
{
	struct cdns_mhdp_device *mhdp = data;
#ifdef DEBUG
	int i;
	for (i = 0; i < mhdp->hdcp.num_paired; i++)
		cdns_hdcp_show_pairing(mhdp, &mhdp->hdcp.pairing[i]);
#endif
	return seq_write(s, &mhdp->hdcp.pairing[0],
		  mhdp->hdcp.num_paired * sizeof(struct hdcp_trans_pairing_data));
}

static int cdns_hdcp_pairing_show(struct seq_file *s, void *data)
{
	return cdns_hdcp_dump_pairing(s, s->private);
}

static int cdns_hdcp_dump_pairing_open(struct inode *inode, struct file *file)
{
	return single_open(file, cdns_hdcp_pairing_show, inode->i_private);
}

static const struct file_operations cdns_hdcp_dump_fops = {
	.open		= cdns_hdcp_dump_pairing_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void cdns_hdcp_debugfs_init(struct cdns_mhdp_device *mhdp)
{
	struct dentry *d, *root;

	root = debugfs_create_dir("imx-hdcp", NULL);
	if (IS_ERR(root) || !root)
		goto err;

	d = debugfs_create_file("dump_pairing", 0444, root, mhdp,
				&cdns_hdcp_dump_fops);
	if (!d)
		goto err;
	return;

err:
	dev_err(mhdp->dev, "Unable to create debugfs entries\n");
}

int cdns_hdcp_init(struct cdns_mhdp_device *mhdp, struct device_node *of_node)
{
	const char *compat;
	u32 temp;
	int ret;

	ret = of_property_read_string(of_node, "compatible", &compat);
	if (ret) {
		DRM_ERROR("Failed to compatible dts string\n");
		return ret;
	}

	if (!(strstr(compat, "hdmi") || strstr(compat, "dp")))
		return -EPERM;

	ret = of_property_read_u32(of_node, "hdcp-config", &temp);
	if (ret) {
		/* using highest level by default */
		mhdp->hdcp.config = HDCP_CONFIG_2_2;
		DRM_INFO("Failed to get HDCP config - using HDCP 2.2 only\n");
	} else {
		mhdp->hdcp.config = temp;
		show_hdcp_supported(mhdp);
	}

	cdns_hdcp_debugfs_init(mhdp);

#ifdef USE_DEBUG_KEYS  /* reserve for hdcp test key */
	{
		u8 hdcp_cfg;
		hdcp_cfg = HDCP_TX_2 | (HDCP_USE_KMKEY << 4) | (HDCP_CONTENT_TYPE_0 << 3);
		imx_hdmi_load_test_keys(mhdp, &hdcp_cfg);
	}
#endif

	mhdp->hdcp.state = HDCP_STATE_INACTIVE;

	mutex_init(&mhdp->hdcp.mutex);
	INIT_DELAYED_WORK(&mhdp->hdcp.check_work, cdns_hdcp_check_work);
	INIT_WORK(&mhdp->hdcp.prop_work, cdns_hdcp_prop_work);

	return 0;
}
EXPORT_SYMBOL_GPL(cdns_hdcp_init);

int cdns_hdcp_enable(struct cdns_mhdp_device *mhdp)
{
	int ret = 0;

	mhdp->hdcp.reauth_in_progress = 0;

#ifdef STORE_PAIRING
	cdns_hdcp_get_stored_pairing(mhdp);
#endif
	msleep(500);

	mutex_lock(&mhdp->hdcp.mutex);

	mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	mhdp->hdcp.state = HDCP_STATE_ENABLING;
	mhdp->hdcp.cancel = 0;

	schedule_work(&mhdp->hdcp.prop_work);
	schedule_delayed_work(&mhdp->hdcp.check_work, 50);

	mutex_unlock(&mhdp->hdcp.mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_hdcp_enable);

int cdns_hdcp_disable(struct cdns_mhdp_device *mhdp)
{
	int ret = 0;

	cancel_delayed_work_sync(&mhdp->hdcp.check_work);

	mutex_lock(&mhdp->hdcp.mutex);
	if (mhdp->hdcp.value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
		mhdp->hdcp.state = HDCP_STATE_DISABLING;
		mhdp->hdcp.cancel = 1;
		schedule_work(&mhdp->hdcp.prop_work);
	}

	mutex_unlock(&mhdp->hdcp.mutex);

	/* Make sure HDCP_STATE_DISABLING state is handled */
	cdns_hdcp_check_link(mhdp);

	return ret;
}
EXPORT_SYMBOL_GPL(cdns_hdcp_disable);

void cdns_hdcp_atomic_check(struct drm_connector *connector,
			   struct drm_connector_state *old_state,
			   struct drm_connector_state *new_state)
{
	u64 old_cp = old_state->content_protection;
	u64 new_cp = new_state->content_protection;
	struct drm_crtc_state *crtc_state;

	if (!new_state->crtc) {
		/*
		 * If the connector is being disabled with CP enabled, mark it
		 * desired so it's re-enabled when the connector is brought back
		 */
		if (old_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED)
			new_state->content_protection =
				DRM_MODE_CONTENT_PROTECTION_DESIRED;
		return;
	}

	/*
	 * Nothing to do if the state didn't change, or HDCP was activated since
	 * the last commit
	 */
	if (old_cp == new_cp ||
	    (old_cp == DRM_MODE_CONTENT_PROTECTION_DESIRED &&
	     new_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED))
		return;

	crtc_state = drm_atomic_get_new_crtc_state(new_state->state, new_state->crtc);
	crtc_state->mode_changed = true;
}
EXPORT_SYMBOL_GPL(cdns_hdcp_atomic_check);

static int cdns_hdcp_check_link(struct cdns_mhdp_device *mhdp)
{
	u16 hdcp_port_status = 0;
	u8 hdcp_last_error = 0;
	u8 hpd_sts;
	int ret = 0;

	mhdp->hdcp.reauth_in_progress = 0;
	mutex_lock(&mhdp->lock);

	if (mhdp->hdcp.state == HDCP_STATE_INACTIVE)
		goto out;

	if (mhdp->hdcp.state == HDCP_STATE_DISABLING) {
		_cdns_hdcp_disable(mhdp);
		mhdp->hdcp.state = HDCP_STATE_INACTIVE;
		goto out;
	}

	if ((mhdp->hdcp.state == HDCP_STATE_AUTHENTICATED)  ||
		(mhdp->hdcp.state == HDCP_STATE_AUTHENTICATING) ||
		(mhdp->hdcp.state == HDCP_STATE_REAUTHENTICATING) ||
		(mhdp->hdcp.state == HDCP_STATE_ENABLING)) {

		/* In active states, check the HPD signal. Because of the IRQ
		 * debounce delay, the state might not reflect the disconnection.
		 * The FW could already have detected the HDP down and reported error */
		hpd_sts = cdns_mhdp_read_hpd(mhdp);
		if (hpd_sts == 0) {
			mhdp->hdcp.state = HDCP_STATE_DISABLING;
			goto out;
		}
	}

/* TODO items:
    Need to make sure that any requests from the firmware are actually
    processed so want to remove this first jump to 'out', i.e. process
    reauthentication requests, cleanup errors and repeater receiver id
    checks.
*/
	if (mhdp->hdcp.state == HDCP_STATE_AUTHENTICATED) {
		/* get port status */
		hdcp_port_status = cdns_hdcp_get_status(mhdp);
		hdcp_last_error = GET_HDCP_PORT_STS_LAST_ERR(hdcp_port_status);
		if (hdcp_last_error == HDCP_TRAN_ERR_REAUTH_REQ) {
			DRM_INFO("Sink requesting re-authentication\n");
			mhdp->hdcp.state = HDCP_STATE_REAUTHENTICATING;
		} else if (hdcp_last_error) {
			DRM_ERROR("HDCP error no: %u\n", hdcp_last_error);

			if (mhdp->hdcp.value == DRM_MODE_CONTENT_PROTECTION_UNDESIRED)
				goto out;
			if (hdcp_port_status &  HDCP_PORT_STS_AUTH) {
				if (mhdp->hdcp.value !=
					    DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
					mhdp->hdcp.value =
						DRM_MODE_CONTENT_PROTECTION_ENABLED;
					schedule_work(&mhdp->hdcp.prop_work);
					goto out;
				}
			}

			mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;

		} else if (mhdp->hdcp.sink_is_repeater) {
			u8 new_events;
			/* Check events... and process if HDCPTX_IS_RECEIVER_ID_VALID_EVENT. */
			new_events = cdns_mhdp_get_event(mhdp);
			mhdp->hdcp.events |= new_events;
			if (check_event(mhdp->hdcp.events, HDCPTX_IS_RECEIVER_ID_VALID_EVENT)) {
				DRM_INFO("Sink repeater updating receiver ID list...\n");
				if (cdns_hdcp_check_receviers(mhdp))
					mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;
			}
		}
	}

	if (mhdp->hdcp.state == HDCP_STATE_REAUTHENTICATING) {
		/* For now just deal with HDCP2.2 */
		if (mhdp->hdcp.hdcp_version == HDCP_TX_2)
			mhdp->hdcp.reauth_in_progress = 1;
		else
			mhdp->hdcp.state = HDCP_STATE_AUTH_FAILED;
	}

	if (mhdp->hdcp.state == HDCP_STATE_ENABLING) {
		mhdp->hdcp.state = HDCP_STATE_AUTHENTICATING;
		ret = _cdns_hdcp_enable(mhdp);
		if (ret == -ECANCELED)
			goto out;
		else if (ret) {
			DRM_ERROR("Failed to enable hdcp (%d)\n", ret);
			mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
			schedule_work(&mhdp->hdcp.prop_work);
			goto out;
		}
	}

	if ((mhdp->hdcp.state == HDCP_STATE_AUTH_FAILED) ||
			(mhdp->hdcp.state == HDCP_STATE_REAUTHENTICATING)) {

		print_port_status(hdcp_port_status);
		if (mhdp->hdcp.state == HDCP_STATE_AUTH_FAILED) {
			DRM_DEBUG_KMS("[%s:%d] HDCP link failed, retrying authentication 0x%2x\n",
				      mhdp->connector.base.name, mhdp->connector.base.base.id, hdcp_port_status);
			ret = _cdns_hdcp_disable(mhdp);
			if (ret) {
				DRM_ERROR("Failed to disable hdcp (%d)\n", ret);
				mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
				schedule_work(&mhdp->hdcp.prop_work);
				goto out;
			}
		} else
			DRM_DEBUG_KMS("[%s:%d] HDCP attempt reauthentication 0x%2x\n",
				      mhdp->connector.base.name, mhdp->connector.base.base.id, hdcp_port_status);

		ret = _cdns_hdcp_enable(mhdp);
		if (ret == -ECANCELED)
			goto out;
		else if (ret) {
			DRM_ERROR("Failed to enable hdcp (%d)\n", ret);
			mhdp->hdcp.value = DRM_MODE_CONTENT_PROTECTION_DESIRED;
			schedule_work(&mhdp->hdcp.prop_work);
			goto out;
		}
	}

out:
	mutex_unlock(&mhdp->lock);

	return ret;
}
