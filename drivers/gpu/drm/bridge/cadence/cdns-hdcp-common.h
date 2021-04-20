/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef CDNS_HDCP_COMMON_H
#define CDNS_HDCP_COMMON_H

int cdns_hdcp_init(struct cdns_mhdp_device *mhdp, struct device_node *of_node);
int cdns_hdcp_enable(struct cdns_mhdp_device *mhdp);
int cdns_hdcp_disable(struct cdns_mhdp_device *mhdp);
void cdns_hdcp_atomic_check(struct drm_connector *connector,
				 struct drm_connector_state *old_state,
				 struct drm_connector_state *new_state);
void cnds_hdcp_create_device_files(struct cdns_mhdp_device *mhdp);

#endif /* CDNS_HDCP_COMMON_H */
