/*
 * Copyright 2018,2021 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LCDIF_KMS_H
#define __LCDIF_KMS_H

#include <drm/drm_atomic.h>
#include <drm/drm_connector.h>

extern const struct drm_mode_config_funcs lcdif_drm_mode_config_funcs;
extern struct drm_mode_config_helper_funcs lcdif_drm_mode_config_helpers;

static inline bool
lcdif_drm_connector_is_self_refresh_aware(struct drm_atomic_state *state)
{
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	int i;

	for_each_new_connector_in_state(state, conn, conn_state, i) {
		if (conn_state->self_refresh_aware)
			return true;
	}

	return false;
}

#endif
