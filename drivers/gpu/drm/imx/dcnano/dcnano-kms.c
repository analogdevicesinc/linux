// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020,2021 NXP
 */

#include <linux/of.h>
#include <linux/of_graph.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "dcnano-drv.h"
#include "dcnano-reg.h"

static int dcnano_kms_init(struct dcnano_dev *dcnano)
{
	struct drm_device *drm = &dcnano->base;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	struct device_node *np = drm->dev->of_node;
	struct device_node *port, *ep, *remote;
	struct of_endpoint endpoint;
	u32 port_id;
	bool found_ep = false;
	int ret;

	ret = dcnano_crtc_init(dcnano);
	if (ret)
		return ret;

	for (port_id = 0; port_id < DCNANO_PORT_NUM; port_id++) {
		port = of_graph_get_port_by_id(np, port_id);
		if (!port) {
			drm_err(drm, "failed to get output port%u\n", port_id);
			return -EINVAL;
		}

		for_each_child_of_node(port, ep) {
			remote = of_graph_get_remote_port_parent(ep);
			if (!remote || !of_device_is_available(remote) ||
			    !of_device_is_available(remote->parent)) {
				of_node_put(remote);
				continue;
			}

			of_node_put(remote);

			ret = of_graph_parse_endpoint(ep, &endpoint);
			if (ret) {
				drm_err(drm,
					"failed to parse endpoint of port%u: %d\n",
					port_id, ret);
				of_node_put(ep);
				of_node_put(port);
				return ret;
			}

			ret = drm_of_find_panel_or_bridge(np,
							  port_id, endpoint.id,
							  &panel, &bridge);
			if (ret) {
				if (ret == -ENODEV) {
					drm_dbg(drm,
						"no panel or bridge on port%u ep%d\n",
						port_id, endpoint.id);
					continue;
				} else if (ret != -EPROBE_DEFER) {
					drm_err(drm,
						"failed to find panel or bridge on port%u ep%d: %d\n",
						port_id, endpoint.id, ret);
				}
				of_node_put(ep);
				of_node_put(port);
				return ret;
			}

			found_ep = true;
			break;
		}

		of_node_put(port);
		dcnano->port = port_id;

		if (found_ep) {
			drm_dbg(drm, "found valid endpoint%d @ port%u\n",
				endpoint.id, port_id);
			break;
		}
	}

	if (!found_ep) {
		drm_info(drm, "no valid endpoint\n");
		return 0;
	}

	if (panel) {
		bridge = devm_drm_panel_bridge_add(drm->dev, panel);
		if (IS_ERR(bridge)) {
			ret = PTR_ERR(bridge);
			drm_err(drm,
				"failed to add panel bridge on port%u ep%d: %d\n",
				port_id, endpoint.id, ret);
			goto err;
		}
	}

	dcnano->encoder.possible_crtcs = drm_crtc_mask(&dcnano->crtc);
	ret = drm_simple_encoder_init(drm, &dcnano->encoder,
				      DRM_MODE_ENCODER_NONE);
	if (ret) {
		drm_err(drm, "failed to initialize encoder on port%u ep%d: %d\n",
			port_id, endpoint.id, ret);
		goto err;
	}

	ret = drm_bridge_attach(&dcnano->encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		drm_err(drm,
			"failed to attach bridge to encoder on port%u ep%d: %d\n",
			port_id, endpoint.id, ret);
		goto err;
	}

	connector = drm_bridge_connector_init(drm, &dcnano->encoder);
	if (IS_ERR(connector)) {
		ret = PTR_ERR(connector);
		drm_err(drm,
			"failed to initialize bridge connector on port%u ep%d: %d\n",
			port_id, endpoint.id, ret);
		goto err;
	}

	ret = drm_connector_attach_encoder(connector, &dcnano->encoder);
	if (ret)
		drm_err(drm,
			"failed to attach encoder to connector on port%u ep%d: %d\n",
			port_id, endpoint.id, ret);
err:
	return ret;
}

static const struct drm_mode_config_funcs dcnano_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create,
	.atomic_check  = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs dcnano_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

int dcnano_kms_prepare(struct dcnano_dev *dcnano)
{
	struct drm_device *drm = &dcnano->base;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	ret = dcnano_kms_init(dcnano);
	if (ret)
		return ret;

	drm->mode_config.min_width	= 32;
	drm->mode_config.min_height	= 32;
	drm->mode_config.max_width	= 1280;
	drm->mode_config.max_height	= 1280;
	drm->mode_config.funcs		= &dcnano_mode_config_funcs;
	drm->mode_config.helper_private	= &dcnano_mode_config_helpers;
	drm->max_vblank_count		= DEBUGCOUNTERVALUE_MAX;

	ret = drm_vblank_init(drm, 1);
	if (ret < 0) {
		drm_err(drm, "failed to initialize vblank: %d\n", ret);
		return ret;
	}

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	return 0;
}
