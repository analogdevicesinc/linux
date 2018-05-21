/*
 * i.MX drm driver - Northwest Logic MIPI DSI display driver
 *
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/bridge/nwl_dsi.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>
#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx8mq-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/phy/phy-mixel-mipi-dsi.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <soc/imx8/sc/sci.h>
#include <video/videomode.h>

#include "imx-drm.h"

#define DRIVER_NAME "nwl_dsi-imx"

/* 8MQ SRC specific registers */
#define SRC_MIPIPHY_RCR				0x28
#define RESET_BYTE_N				BIT(1)
#define RESET_N					BIT(2)
#define DPI_RESET_N				BIT(3)
#define ESC_RESET_N				BIT(4)
#define PCLK_RESET_N				BIT(5)

#define DC_ID(x)	SC_R_DC_ ## x
#define MIPI_ID(x)	SC_R_MIPI_ ## x
#define SYNC_CTRL(x)	SC_C_SYNC_CTRL ## x
#define PXL_VLD(x)	SC_C_PXL_LINK_MST ## x ## _VLD
#define PXL_ADDR(x)	SC_C_PXL_LINK_MST ## x ## _ADDR

/* Possible clocks */
#define CLK_PIXEL	"pixel"
#define CLK_CORE	"core"
#define CLK_BYPASS	"bypass"
#define CLK_PHYREF	"phy_ref"

/* Possible valid PHY reference clock rates*/
u32 phyref_rates[] = {
	24000000,
	25000000,
	27000000,
};

struct imx_mipi_dsi {
	struct drm_encoder		encoder;
	struct drm_bridge		bridge;
	struct drm_bridge		*next_bridge;
	struct device			*dev;
	struct phy			*phy;

	/* Optional external regs */
	struct regmap			*csr;
	struct regmap			*reset;
	struct regmap			*mux_sel;

	/* Optional clocks */
	struct clk_config		*clk_config;
	size_t				clk_num;

	u32 tx_ulps_reg;
	u32 pxl2dpi_reg;

	unsigned long			bit_clk;
	unsigned long			pix_clk;
	u32				phyref_rate;
	u32				instance;
	u32				sync_pol;
	u32				power_on_delay;
	bool				no_clk_reset;
	bool				enabled;
	bool				suspended;
};

struct clk_config {
	const char *id;
	struct clk *clk;
	bool present;
	bool enabled;
	u32 rate;
};

enum imx_ext_regs {
	IMX_REG_CSR = BIT(1),
	IMX_REG_SRC = BIT(2),
	IMX_REG_GPR = BIT(3),
};

struct devtype {
	int (*poweron)(struct imx_mipi_dsi *);
	int (*poweroff)(struct imx_mipi_dsi *);
	u32 ext_regs; /* required external registers */
	u32 tx_ulps_reg;
	u32 pxl2dpi_reg;
	u8 max_instances;
	struct clk_config clk_config[4];
};

static int imx8qm_dsi_poweron(struct imx_mipi_dsi *dsi);
static int imx8qm_dsi_poweroff(struct imx_mipi_dsi *dsi);
static struct devtype imx8qm_dev = {
	.poweron = &imx8qm_dsi_poweron,
	.poweroff = &imx8qm_dsi_poweroff,
	.clk_config = {
		{ .id = CLK_CORE,   .present = false },
		{ .id = CLK_PIXEL,  .present = true },
		{ .id = CLK_BYPASS, .present = true },
		{ .id = CLK_PHYREF, .present = true },
	},
	.ext_regs = IMX_REG_CSR,
	.tx_ulps_reg   = 0x00,
	.pxl2dpi_reg   = 0x04,
	.max_instances =    2,
};

static int imx8qxp_dsi_poweron(struct imx_mipi_dsi *dsi);
static int imx8qxp_dsi_poweroff(struct imx_mipi_dsi *dsi);
static struct devtype imx8qxp_dev = {
	.poweron = &imx8qxp_dsi_poweron,
	.poweroff = &imx8qxp_dsi_poweroff,
	.clk_config = {
		{ .id = CLK_CORE,   .present = false },
		{ .id = CLK_PIXEL,  .present = true },
		{ .id = CLK_BYPASS, .present = true },
		{ .id = CLK_PHYREF, .present = true },
	},
	.ext_regs = IMX_REG_CSR,
	.tx_ulps_reg   = 0x30,
	.pxl2dpi_reg   = 0x40,
	.max_instances =    2,
};

static int imx8mq_dsi_poweron(struct imx_mipi_dsi *dsi);
static int imx8mq_dsi_poweroff(struct imx_mipi_dsi *dsi);
static struct devtype imx8mq_dev = {
	.poweron = &imx8mq_dsi_poweron,
	.poweroff = &imx8mq_dsi_poweroff,
	.clk_config = {
		{ .id = CLK_CORE,   .present = true },
		{ .id = CLK_PIXEL,  .present = false },
		{ .id = CLK_BYPASS, .present = false },
		{ .id = CLK_PHYREF, .present = true },
	},
	.ext_regs = IMX_REG_SRC | IMX_REG_GPR,
	.max_instances = 1,
};

static const struct of_device_id imx_nwl_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-mipi-dsi", .data = &imx8qm_dev, },
	{ .compatible = "fsl,imx8qxp-mipi-dsi", .data = &imx8qxp_dev, },
	{ .compatible = "fsl,imx8mq-mipi-dsi_drm", .data = &imx8mq_dev, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_nwl_dsi_dt_ids);

static inline struct imx_mipi_dsi *encoder_to_dsi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct imx_mipi_dsi, encoder);
}

static void imx_nwl_dsi_set_clocks(struct imx_mipi_dsi *dsi, bool enable)
{
	struct device *dev = dsi->dev;
	const char *id;
	struct clk *clk;
	unsigned long new_rate, cur_rate;
	bool enabled;
	size_t i;

	for (i = 0; i < dsi->clk_num; i++) {
		if (!dsi->clk_config[i].present)
			continue;
		id = dsi->clk_config[i].id;
		clk = dsi->clk_config[i].clk;
		new_rate = dsi->clk_config[i].rate;
		cur_rate = clk_get_rate(clk);
		enabled = dsi->clk_config[i].enabled;

		/* BYPASS clk must have the same rate as PHY_REF clk */
		if (!strcmp(id, CLK_BYPASS) || !strcmp(id, CLK_PHYREF))
			new_rate = dsi->phyref_rate;

		if (enable) {
			if (enabled && new_rate != cur_rate)
				clk_disable_unprepare(clk);
			else if (enabled && new_rate == cur_rate)
				continue;
			if (new_rate > 0)
				clk_set_rate(clk, new_rate);
			clk_prepare_enable(clk);
			dsi->clk_config[i].enabled = true;
			cur_rate = clk_get_rate(clk);
			DRM_DEV_DEBUG_DRIVER(dev,
				"Enabled %s clk (rate: req=%lu act=%lu)\n",
				id, new_rate, cur_rate);
		} else if (enabled) {
			clk_disable_unprepare(clk);
			dsi->clk_config[i].enabled = false;
			DRM_DEV_DEBUG_DRIVER(dev, "Disabled %s clk\n", id);
		}
	}
}

/*
 * v2 is true for QXP
 * On QM, we have 2 DPUs, each one with a MIPI-DSI link
 * On QXP, we have 1 DPU with two MIPI-DSI links
 * Because of this, we will have different initialization
 * paths for MIPI0 and MIPI1 on QM vs QXP
 */
static int imx8q_dsi_poweron(struct imx_mipi_dsi *dsi, bool v2)
{
	struct device *dev = dsi->dev;
	int ret = 0;
	sc_err_t sci_err = 0;
	sc_ipc_t ipc_handle = 0;
	u32 inst = dsi->instance;
	u32 mu_id;
	sc_rsrc_t mipi_id, dc_id;
	sc_ctrl_t mipi_ctrl;

	sci_err = sc_ipc_getMuID(&mu_id);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev, "Failed to get MU ID (%d)\n", sci_err);
		return -ENODEV;
	}
	sci_err = sc_ipc_open(&ipc_handle, mu_id);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev, "Failed to open IPC (%d)\n", sci_err);
		return -ENODEV;
	}

	mipi_id = inst?MIPI_ID(1):MIPI_ID(0);
	dc_id = (!v2 && inst)?DC_ID(1):DC_ID(0);
	DRM_DEV_DEBUG_DRIVER(dev, "MIPI ID: %d DC ID: %d\n",
			     mipi_id,
			     dc_id);

	/* Initialize Pixel Link */
	mipi_ctrl = (v2 && inst)?PXL_ADDR(2):PXL_ADDR(1);
	sci_err = sc_misc_set_control(ipc_handle,
				      dc_id,
				      mipi_ctrl,
				      0);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev,
			"Failed to set SC_C_PXL_LINK_MST%d_ADDR (%d)\n",
			inst,
			sci_err);
		ret = -ENODEV;
		goto err_ipc;
	}

	mipi_ctrl = (v2 && inst)?PXL_VLD(2):PXL_VLD(1);
	sci_err = sc_misc_set_control(ipc_handle,
				      dc_id,
				      mipi_ctrl,
				      1);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev,
			"Failed to set SC_C_PXL_LINK_MST%d_VLD (%d)\n",
			inst + 1,
			sci_err);
		ret = -ENODEV;
		goto err_ipc;
	}

	mipi_ctrl = (v2 && inst)?SYNC_CTRL(1):SYNC_CTRL(0);
	sci_err = sc_misc_set_control(ipc_handle,
				      dc_id,
				      mipi_ctrl,
				      1);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev,
			"Failed to set SC_C_SYNC_CTRL%d (%d)\n",
			inst,
			sci_err);
		ret = -ENODEV;
		goto err_ipc;
	}

	if (v2) {
		sci_err = sc_misc_set_control(ipc_handle,
				mipi_id, SC_C_MODE, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				      "Failed to set SC_C_MODE (%d)\n",
				      sci_err);
		sci_err = sc_misc_set_control(ipc_handle,
				mipi_id, SC_C_DUAL_MODE, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				      "Failed to set SC_C_DUAL_MODE (%d)\n",
				      sci_err);
		sci_err = sc_misc_set_control(ipc_handle,
				mipi_id, SC_C_PXL_LINK_SEL, inst);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				     "Failed to set SC_C_PXL_LINK_SEL (%d)\n",
				     sci_err);
	}

	/* Assert DPI and MIPI bits */
	sci_err = sc_misc_set_control(ipc_handle,
				      mipi_id,
				      SC_C_DPI_RESET,
				      1);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev,
			"Failed to assert DPI reset (%d)\n",
			sci_err);
		ret = -ENODEV;
		goto err_ipc;
	}

	sci_err = sc_misc_set_control(ipc_handle,
				      mipi_id,
				      SC_C_MIPI_RESET,
				      1);
	if (sci_err != SC_ERR_NONE) {
		DRM_DEV_ERROR(dev,
			"Failed to assert MIPI reset (%d)\n",
			sci_err);
		ret = -ENODEV;
		goto err_ipc;
	}

	regmap_write(dsi->csr,
		     dsi->tx_ulps_reg,
		     0);
	regmap_write(dsi->csr,
		     dsi->pxl2dpi_reg,
		     DPI_24_BIT);

	sc_ipc_close(ipc_handle);
	return ret;

err_ipc:
	sc_ipc_close(ipc_handle);
	return ret;
}

static int imx8q_dsi_poweroff(struct imx_mipi_dsi *dsi, bool v2)
{
	struct device *dev = dsi->dev;
	sc_err_t sci_err = 0;
	sc_ipc_t ipc_handle = 0;
	u32 mu_id;
	u32 inst = dsi->instance;
	sc_rsrc_t mipi_id, dc_id;

	mipi_id = (inst)?SC_R_MIPI_1:SC_R_MIPI_0;
	if (v2)
		dc_id = SC_R_DC_0;
	else
		dc_id = (inst)?SC_R_DC_1:SC_R_DC_0;

	/* Deassert DPI and MIPI bits */
	if (sc_ipc_getMuID(&mu_id) == SC_ERR_NONE &&
	    sc_ipc_open(&ipc_handle, mu_id) == SC_ERR_NONE) {
		sci_err = sc_misc_set_control(ipc_handle,
				mipi_id, SC_C_DPI_RESET, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				"Failed to deassert DPI reset (%d)\n",
				sci_err);

		sci_err = sc_misc_set_control(ipc_handle,
				mipi_id, SC_C_MIPI_RESET, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				"Failed to deassert MIPI reset (%d)\n",
				sci_err);

		sci_err = sc_misc_set_control(ipc_handle,
				dc_id, SC_C_SYNC_CTRL0, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				"Failed to reset SC_C_SYNC_CTRL0 (%d)\n",
				sci_err);

		sci_err = sc_misc_set_control(ipc_handle,
				dc_id, SC_C_PXL_LINK_MST1_VLD, 0);
		if (sci_err != SC_ERR_NONE)
			DRM_DEV_ERROR(dev,
				"Failed to reset SC_C_SYNC_CTRL0 (%d)\n",
				sci_err);
	}

	return 0;
}

static int imx8qm_dsi_poweron(struct imx_mipi_dsi *dsi)
{
	return imx8q_dsi_poweron(dsi, false);
}

static int imx8qm_dsi_poweroff(struct imx_mipi_dsi *dsi)
{
	return imx8q_dsi_poweroff(dsi, false);
}

static int imx8qxp_dsi_poweron(struct imx_mipi_dsi *dsi)
{
	return imx8q_dsi_poweron(dsi, true);
}

static int imx8qxp_dsi_poweroff(struct imx_mipi_dsi *dsi)
{
	return imx8q_dsi_poweroff(dsi, true);
}

static int imx8mq_dsi_poweron(struct imx_mipi_dsi *dsi)
{
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   PCLK_RESET_N, PCLK_RESET_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   ESC_RESET_N, ESC_RESET_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   RESET_BYTE_N, RESET_BYTE_N);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   DPI_RESET_N, DPI_RESET_N);

	return 0;
}

static int imx8mq_dsi_poweroff(struct imx_mipi_dsi *dsi)
{
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   PCLK_RESET_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   ESC_RESET_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   RESET_BYTE_N, 0);
	regmap_update_bits(dsi->reset, SRC_MIPIPHY_RCR,
			   DPI_RESET_N, 0);

	return 0;
}

static void imx_nwl_dsi_enable(struct imx_mipi_dsi *dsi)
{
	struct device *dev = dsi->dev;
	const struct of_device_id *of_id = of_match_device(imx_nwl_dsi_dt_ids,
							   dev);
	const struct devtype *devtype = of_id->data;
	unsigned long bit_clk, min_sleep, max_sleep;
	int ret;

	if (dsi->enabled)
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	/*
	 * TODO: we are doing this here, because the ADV7535 which is a drm
	 * bridge, may change the DSI parameters in mode_set. One of the
	 * changed parameter is DSI lanes, which affects the PHY settings.
	 * This is why, we need run this function again, here, in order
	 * to correctly set-up the PHY. Since we can't do anything here, we
	 * will ignore it's status.
	 * In the future, maybe it will be best to move the PHY handling
	 * into the DSI host driver.
	 */
	bit_clk = nwl_dsi_get_bit_clock(dsi->next_bridge, dsi->pix_clk);
	if (bit_clk != dsi->bit_clk) {
		mixel_phy_mipi_set_phy_speed(dsi->phy,
			bit_clk,
			dsi->phyref_rate,
			false);
		dsi->bit_clk = bit_clk;
	}

	/*
	 * On some systems we need to wait some time before enabling the
	 * phy_ref clock, in order to allow the parent PLL to become stable
	 */
	if (dsi->power_on_delay > 20) {
		msleep(dsi->power_on_delay);
	} else if (dsi->power_on_delay > 0) {
		max_sleep = dsi->power_on_delay * 1000;
		min_sleep = 1000;
		if (max_sleep > 6000)
			min_sleep = max_sleep - 5000;
		usleep_range(min_sleep, max_sleep);
	}

	request_bus_freq(BUS_FREQ_HIGH);

	imx_nwl_dsi_set_clocks(dsi, true);

	ret = devtype->poweron(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to power on DSI (%d)\n", ret);
		return;
	}

	dsi->enabled = true;
}

static void imx_nwl_dsi_disable(struct imx_mipi_dsi *dsi)
{
	struct device *dev = dsi->dev;
	const struct of_device_id *of_id = of_match_device(imx_nwl_dsi_dt_ids,
							   dev);
	const struct devtype *devtype = of_id->data;

	if (!dsi->enabled)
		return;

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	if (!dsi->no_clk_reset)
		devtype->poweroff(dsi);

	imx_nwl_dsi_set_clocks(dsi, false);

	release_bus_freq(BUS_FREQ_HIGH);

	dsi->enabled = false;
}

static void imx_nwl_update_sync_polarity(unsigned int *flags, u32 sync_pol)
{
	/* Make sure all flags are set-up accordingly */
	if (sync_pol) {
		*flags |= DRM_MODE_FLAG_PHSYNC;
		*flags |= DRM_MODE_FLAG_PVSYNC;
		*flags &= ~DRM_MODE_FLAG_NHSYNC;
		*flags &= ~DRM_MODE_FLAG_NVSYNC;
	} else {
		*flags &= ~DRM_MODE_FLAG_PHSYNC;
		*flags &= ~DRM_MODE_FLAG_PVSYNC;
		*flags |= DRM_MODE_FLAG_NHSYNC;
		*flags |= DRM_MODE_FLAG_NVSYNC;
	}
}

/*
 * This function will try the required phy speed for current mode
 * If the phy speed can be achieved, the phy will save the speed
 * configuration
 */
static int imx_nwl_try_phy_speed(struct imx_mipi_dsi *dsi,
			    struct drm_display_mode *mode)
{
	struct device *dev = dsi->dev;
	unsigned long pixclock;
	unsigned long bit_clk;
	size_t i, num_rates = ARRAY_SIZE(phyref_rates);
	int ret = 0;

	pixclock = mode->clock * 1000;
	/*
	 * DSI host should know the required bit clock, since it has info
	 * about bits-per-pixel and number of lanes from DSI device
	 */
	bit_clk = nwl_dsi_get_bit_clock(dsi->next_bridge, pixclock);

	/* If bit_clk is the same with current, we're good */
	if (bit_clk == dsi->bit_clk)
		return 0;

	for (i = 0; i < num_rates; i++) {
		dsi->phyref_rate = phyref_rates[i];
		DRM_DEV_DEBUG_DRIVER(dev, "Trying PHY ref rate: %u\n",
			dsi->phyref_rate);
		ret = mixel_phy_mipi_set_phy_speed(dsi->phy,
			bit_clk,
			dsi->phyref_rate,
			false);
		/* Pick the first non-failing rate */
		if (!ret)
			break;
	}
	if (ret < 0) {
		DRM_DEV_ERROR(dev,
			"Cannot setup PHY for mode: %ux%u @%d kHz\n",
			mode->hdisplay,
			mode->vdisplay,
			mode->clock);
		DRM_DEV_ERROR(dev, "PHY_REF clk: %u, bit clk: %lu\n",
			dsi->phyref_rate, bit_clk);
	} else {
		dsi->bit_clk = bit_clk;
		dsi->pix_clk = pixclock;
	}

	return ret;
}

static void imx_nwl_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_mipi_dsi *dsi = encoder_to_dsi(encoder);

	pm_runtime_get_sync(dsi->dev);
	imx_nwl_dsi_enable(dsi);
}

static void imx_nwl_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct imx_mipi_dsi *dsi = encoder_to_dsi(encoder);

	imx_nwl_dsi_disable(dsi);
	pm_runtime_put_sync(dsi->dev);
}

static int imx_nwl_dsi_encoder_atomic_check(struct drm_encoder *encoder,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx_mipi_dsi *dsi = encoder_to_dsi(encoder);
	unsigned int *flags = &crtc_state->adjusted_mode.flags;

	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;
	imx_nwl_update_sync_polarity(flags, dsi->sync_pol);

	/* Try to see if the phy can satisfy the current mode */
	return imx_nwl_try_phy_speed(dsi, &crtc_state->adjusted_mode);
}

static const struct drm_encoder_helper_funcs
imx_nwl_dsi_encoder_helper_funcs = {
	.enable = imx_nwl_dsi_encoder_enable,
	.disable = imx_nwl_dsi_encoder_disable,
	.atomic_check = imx_nwl_dsi_encoder_atomic_check,
};

static void imx_nwl_dsi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs imx_nwl_dsi_encoder_funcs = {
	.destroy = imx_nwl_dsi_encoder_destroy,
};


static void imx_nwl_dsi_bridge_enable(struct drm_bridge *bridge)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;

	imx_nwl_dsi_enable(dsi);
	pm_runtime_get_sync(dsi->dev);
}

static void imx_nwl_dsi_bridge_disable(struct drm_bridge *bridge)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;

	imx_nwl_dsi_disable(dsi);
	pm_runtime_put_sync(dsi->dev);
}

static bool imx_nwl_dsi_bridge_mode_fixup(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;
	unsigned int *flags = &adjusted_mode->flags;

	imx_nwl_update_sync_polarity(flags, dsi->sync_pol);

	return (imx_nwl_try_phy_speed(dsi, adjusted_mode) == 0);
}

static int imx_nwl_dsi_bridge_attach(struct drm_bridge *bridge)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;
	struct drm_encoder *encoder = bridge->encoder;
	int ret = 0;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "id = %s\n",
			     (dsi->instance)?"DSI1":"DSI0");
	if (!encoder) {
		DRM_DEV_ERROR(dsi->dev, "Parent encoder object not found\n");
		return -ENODEV;
	}

	/* Attach the next bridge in chain */
	ret = drm_bridge_attach(encoder, dsi->next_bridge, bridge);
	if (ret)
		DRM_DEV_ERROR(dsi->dev, "Failed to attach bridge! (%d)\n",
			      ret);

	return ret;
}

static void imx_nwl_dsi_bridge_detach(struct drm_bridge *bridge)
{
	struct imx_mipi_dsi *dsi = bridge->driver_private;

	DRM_DEV_DEBUG_DRIVER(dsi->dev, "id = %s\n",
			     (dsi->instance)?"DSI1":"DSI0");
	/*
	 * The next bridge in chain will be automatically detached, there is
	 * no need for us to detach it.
	 */
}

static const struct drm_bridge_funcs imx_nwl_dsi_bridge_funcs = {
	.enable = imx_nwl_dsi_bridge_enable,
	.disable = imx_nwl_dsi_bridge_disable,
	.mode_fixup = imx_nwl_dsi_bridge_mode_fixup,
	.attach = imx_nwl_dsi_bridge_attach,
	.detach = imx_nwl_dsi_bridge_detach,
};

static int imx_nwl_dsi_parse_of(struct device *dev, bool as_bridge)
{
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id = of_match_device(imx_nwl_dsi_dt_ids,
							   dev);
	const struct devtype *devtype = of_id->data;
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);
	struct clk *clk;
	const char *clk_id;
	size_t i, clk_config_sz;
	int id;
	u32 mux_val;
	int ret = 0;

	id = of_alias_get_id(np, "mipi_dsi");
	if (id < 0) {
		dev_err(dev, "No mipi_dsi alias found!");
		return id;
	}
	if (id > devtype->max_instances - 1) {
		dev_err(dev, "Too many instances! (cur: %d, max: %d)\n",
			id, devtype->max_instances);
		return -ENODEV;
	}
	dsi->instance = id;

	dsi->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(dsi->phy)) {
		ret = PTR_ERR(dsi->phy);
		dev_err(dev, "Could not get PHY (%d)\n", ret);
		return ret;
	}

	/* Look for optional clocks */
	dsi->clk_num = ARRAY_SIZE(devtype->clk_config);
	dsi->clk_config = devm_kcalloc(dev,
				dsi->clk_num,
				sizeof(struct clk_config),
				GFP_KERNEL);
	clk_config_sz = dsi->clk_num * sizeof(struct clk_config);
	memcpy(dsi->clk_config, devtype->clk_config, clk_config_sz);

	for (i = 0; i < dsi->clk_num; i++) {
		if (!dsi->clk_config[i].present)
			continue;

		clk_id = dsi->clk_config[i].id;
		clk = devm_clk_get(dev, clk_id);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(dev, "Failed to get %s clock (%d)\n",
				clk_id, ret);
			return ret;
		}
		dev_dbg(dev, "Setup clk %s (rate: %lu)\n",
			clk_id, clk_get_rate(clk));
		dsi->clk_config[i].clk = clk;
	}

	dsi->tx_ulps_reg = devtype->tx_ulps_reg;
	dsi->pxl2dpi_reg = devtype->pxl2dpi_reg;

	of_property_read_u32(np, "sync-pol", &dsi->sync_pol);
	of_property_read_u32(np, "pwr-delay", &dsi->power_on_delay);

	/* Look for optional regmaps */
	dsi->csr = syscon_regmap_lookup_by_phandle(np, "csr");
	if (IS_ERR(dsi->csr) && (devtype->ext_regs & IMX_REG_CSR)) {
		ret = PTR_ERR(dsi->csr);
		dev_err(dev, "Failed to get CSR regmap (%d)\n", ret);
		return ret;
	}
	dsi->reset = syscon_regmap_lookup_by_phandle(np, "src");
	if (IS_ERR(dsi->reset) && (devtype->ext_regs & IMX_REG_SRC)) {
		ret = PTR_ERR(dsi->reset);
		dev_err(dev, "Failed to get SRC regmap (%d)\n", ret);
		return ret;
	}
	dsi->mux_sel = syscon_regmap_lookup_by_phandle(np, "mux-sel");
	if (IS_ERR(dsi->mux_sel) && (devtype->ext_regs & IMX_REG_GPR)) {
		ret = PTR_ERR(dsi->mux_sel);
		dev_err(dev, "Failed to get GPR regmap (%d)\n", ret);
		return ret;
	}
	if (IS_ERR(dsi->mux_sel))
		return 0;

	mux_val = IMX8MQ_GPR13_MIPI_MUX_SEL;
	if (as_bridge)
		mux_val = 0;
	dev_info(dev, "Using %s as input source\n",
		(mux_val)?"DCSS":"LCDIF");
	regmap_update_bits(dsi->mux_sel,
		   IOMUXC_GPR13,
		   IMX8MQ_GPR13_MIPI_MUX_SEL,
		   mux_val);

	dsi->no_clk_reset = of_property_read_bool(np, "no_clk_reset");

	return 0;
}

static int imx_nwl_dsi_bind(struct device *dev,
			struct device *master,
			void *data)
{
	struct drm_device *drm = data;
	struct drm_bridge *next_bridge = NULL;
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);
	int ret = 0;

	ret = imx_nwl_dsi_parse_of(dev, false);
	if (ret)
		return ret;

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	/* Re-validate the bridge */
	if (dsi->next_bridge)
		next_bridge = of_drm_find_bridge(dsi->next_bridge->of_node);
	dsi->next_bridge = next_bridge;

	if (!dsi->next_bridge) {
		dev_warn(dev, "No bridge found, skipping encoder creation\n");
		return ret;
	}

	ret = imx_drm_encoder_parse_of(drm, &dsi->encoder, dev->of_node);
	if (ret)
		return ret;

	drm_encoder_helper_add(&dsi->encoder,
			       &imx_nwl_dsi_encoder_helper_funcs);
	ret = drm_encoder_init(drm,
			       &dsi->encoder,
			       &imx_nwl_dsi_encoder_funcs,
			       DRM_MODE_ENCODER_DSI,
			       NULL);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to init DSI encoder (%d)\n", ret);
		return ret;
	}

	dsi->next_bridge->encoder = &dsi->encoder;
	dsi->encoder.bridge = dsi->next_bridge;
	ret = drm_bridge_attach(&dsi->encoder, dsi->next_bridge, NULL);
	if (ret)
		drm_encoder_cleanup(&dsi->encoder);

	return ret;
}

static void imx_nwl_dsi_unbind(struct device *dev,
			   struct device *master,
			   void *data)
{
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);
	struct drm_bridge *next_bridge = NULL;

	DRM_DEV_DEBUG_DRIVER(dev, "id = %s\n", (dsi->instance)?"DSI1":"DSI0");

	/*
	 * At this point, our next bridge in chain might be already removed,
	 * so update it's status.
	 */
	if (dsi->next_bridge)
		next_bridge = of_drm_find_bridge(dsi->next_bridge->of_node);
	dsi->next_bridge = next_bridge;

	if (dsi->enabled)
		imx_nwl_dsi_encoder_disable(&dsi->encoder);

	if (dsi->encoder.dev)
		drm_encoder_cleanup(&dsi->encoder);
}

static const struct component_ops imx_nwl_dsi_component_ops = {
	.bind	= imx_nwl_dsi_bind,
	.unbind	= imx_nwl_dsi_unbind,
};

static int imx_nwl_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *remote_node, *endpoint;
	int remote_ports = 0;
	struct imx_mipi_dsi *dsi;
	int ret = 0;

	if (!np)
		return -ENODEV;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	/* Search for next bridge (usually the DSI HOST bridge) */
	endpoint = of_graph_get_next_endpoint(np, NULL);
	while (endpoint && !dsi->next_bridge) {
		remote_node = of_graph_get_remote_port_parent(endpoint);
		if (!remote_node) {
			dev_err(dev, "No endpoint found!\n");
			return -ENODEV;
		}

		dsi->next_bridge = of_drm_find_bridge(remote_node);
		of_node_put(remote_node);
		endpoint = of_graph_get_next_endpoint(np, endpoint);
		if (!of_device_is_available(remote_node))
			continue;
		remote_ports++;
	};

	/*
	 * Normally, we should have two remote ports: one is our input source,
	 * while the second is the NWL host bridge. This bridge can be disabled
	 * if the connector fails to find a physical device. In this case, we
	 * should continue and do nothing, so that DRM master can bind all the
	 * components.
	 */
	if (!dsi->next_bridge && remote_ports == 2) {
		dev_warn(dev, "Waiting for DSI host bridge\n");
		return -EPROBE_DEFER;
	}

	dsi->dev = dev;
	dev_set_drvdata(dev, dsi);

	pm_runtime_enable(dev);

	if (of_property_read_bool(dev->of_node, "as_bridge")) {
		ret = imx_nwl_dsi_parse_of(dev, true);
		if (ret)
			return ret;
		/* Create our bridge */
		dsi->bridge.driver_private = dsi;
		dsi->bridge.funcs = &imx_nwl_dsi_bridge_funcs;
		dsi->bridge.of_node = np;

		ret = drm_bridge_add(&dsi->bridge);
		if (ret) {
			dev_err(dev, "Failed to add imx-nwl-dsi bridge (%d)\n",
				ret);
			return ret;
		}
		dev_info(dev, "Added drm bridge!");
		return 0;
	}

	return component_add(&pdev->dev, &imx_nwl_dsi_component_ops);
}

static int imx_nwl_dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_nwl_dsi_component_ops);

	return 0;
}

#ifdef CONFIG_PM
static int imx_nwl_suspend(struct device *dev)
{
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);

	if (!dsi->enabled)
		return 0;

	if (dsi->next_bridge)
		drm_bridge_disable(dsi->next_bridge);
	imx_nwl_dsi_disable(dsi);
	dsi->suspended = true;

	return 0;
}

static int imx_nwl_resume(struct device *dev)
{
	struct imx_mipi_dsi *dsi = dev_get_drvdata(dev);

	if (!dsi->suspended)
		return 0;

	imx_nwl_dsi_enable(dsi);
	if (dsi->next_bridge)
		drm_bridge_enable(dsi->next_bridge);
	dsi->suspended = false;

	return 0;
}

#endif

static const struct dev_pm_ops imx_nwl_pm_ops = {
	SET_RUNTIME_PM_OPS(imx_nwl_suspend, imx_nwl_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(imx_nwl_suspend, imx_nwl_resume)
};

static struct platform_driver imx_nwl_dsi_driver = {
	.probe		= imx_nwl_dsi_probe,
	.remove		= imx_nwl_dsi_remove,
	.driver		= {
		.of_match_table = imx_nwl_dsi_dt_ids,
		.name	= DRIVER_NAME,
		.pm	= &imx_nwl_pm_ops,
	},
};

module_platform_driver(imx_nwl_dsi_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX Northwest Logic MIPI-DSI driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
