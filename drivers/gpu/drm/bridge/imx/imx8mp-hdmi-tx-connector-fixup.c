// SPDX-License-Identifier: GPL-2.0
/*
 * Add an hdmi-connector node to boards using the imx8mp hdmi_tx which
 * don't have one. This is needed for the i.MX LCDIF to work with
 * DRM_BRIDGE_ATTACH_NO_CONNECTOR.
 *
 * Copyright (C) 2026 GE HealthCare
 * Author: Luca Ceresoli <luca.ceresoli@bootlin.com>
 */

#include <linux/cleanup.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_graph.h>

/* Embedded dtbo symbols created by cmd_wrap_S_dtb in scripts/Makefile.dtbs */
extern char __dtbo_imx8mp_hdmi_tx_connector_fixup_begin[];
extern char __dtbo_imx8mp_hdmi_tx_connector_fixup_end[];

static int __init imx8mp_hdmi_tx_connector_fixup_init(void)
{
	struct device_node *soc      __free(device_node) = NULL;
	struct device_node *hdmi_tx  __free(device_node) = NULL;
	struct device_node *endpoint __free(device_node) = NULL;
	void *dtbo_start;
	u32 dtbo_size;
	int ovcs_id;
	int err;

	soc = of_find_node_by_path("/soc@0");
	if (!soc)
		return 0;

	/* This applies to i.MX8MP only, do nothing on other systems */
	if (!of_device_is_compatible(soc, "fsl,imx8mp-soc"))
		return 0;

	hdmi_tx = of_find_node_by_path("/soc@0/bus@32c00000/hdmi@32fd8000");
	if (!of_device_is_available(hdmi_tx))
		return 0;

	/* If endpoint exists, assume an hdmi-connector exists already */
	endpoint = of_graph_get_endpoint_by_regs(hdmi_tx, 1, -1);
	if (endpoint)
		return 0;

	dtbo_start = __dtbo_imx8mp_hdmi_tx_connector_fixup_begin;
	dtbo_size = __dtbo_imx8mp_hdmi_tx_connector_fixup_end -
		    __dtbo_imx8mp_hdmi_tx_connector_fixup_begin;

	err = of_overlay_fdt_apply(dtbo_start, dtbo_size, &ovcs_id, NULL);
	if (err)
		err = of_overlay_remove(&ovcs_id);

	return err;
}

subsys_initcall(imx8mp_hdmi_tx_connector_fixup_init);
