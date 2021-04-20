/*
 * Copyright 2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef CDNS_HDMI_HDCP_H
#define CDNS_HDMI_HDCP_H

int cdns_mhdp_hdcp2_tx_respond_km(struct cdns_mhdp_device *mhdp,
					u8 *msg, u16 len);
int cdns_mhdp_hdcp_tx_config(struct cdns_mhdp_device *mhdp, u8 config);
int cdns_mhdp_hdcp_tx_status_req(struct cdns_mhdp_device *mhdp,
					u8 *status, u16 len);
int cdns_mhdp_hdcp2_tx_is_km_stored_req(struct cdns_mhdp_device *mhdp, u8 *data, u16 len);
int cdns_mhdp_hdcp2_tx_store_km(struct cdns_mhdp_device *mhdp,
					u8 *reg, u16 len);
int cdns_mhdp_hdcp_tx_is_receiver_id_valid(struct cdns_mhdp_device *mhdp,
					u8 *rx_id, u8 *num);
int cdns_mhdp_hdcp_tx_respond_receiver_id_valid(struct cdns_mhdp_device *mhdp,
					u8 val);
int cdns_mhdp_hdcp_tx_test_keys(struct cdns_mhdp_device *mhdp, u8 type, u8 resp);
int cdns_mhdp_hdcp_tx_reauth(struct cdns_mhdp_device *mhdp, u8 msg);

#endif /* CDNS_HDMI_HDCP_H */
