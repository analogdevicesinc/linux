/*
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/of_device.h>

#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "imx-dp.h"
#include "../imx-drm.h"

struct drm_display_mode *g_mode;

static struct drm_display_mode edid_cea_modes[] = {
	/* 3 - 720x480@60Hz */
	{ DRM_MODE("720x480", DRM_MODE_TYPE_DRIVER, 27000, 720, 736,
		   798, 858, 0, 480, 489, 495, 525, 0,
		   DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 4 - 1280x720@60Hz */
	{ DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
		   1430, 1650, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 16 - 1920x1080@60Hz */
	{ DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008,
		   2052, 2200, 0, 1080, 1084, 1089, 1125, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 97 - 3840x2160@60Hz */
	{ DRM_MODE("3840x2160", DRM_MODE_TYPE_DRIVER, 594000,
		   3840, 4016, 4104, 4400, 0,
		   2160, 2168, 2178, 2250, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 60, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
	/* 96 - 3840x2160@30Hz */
	{ DRM_MODE("3840x2160", DRM_MODE_TYPE_DRIVER, 297000,
		   3840, 4016, 4104, 4400, 0,
		   2160, 2168, 2178, 2250, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
	  .vrefresh = 30, .picture_aspect_ratio = HDMI_PICTURE_ASPECT_16_9, },
};

static inline struct imx_hdp *enc_to_imx_hdp(struct drm_encoder *e)
{
	return container_of(e, struct imx_hdp, encoder);
}

static void imx_hdp_state_init(struct imx_hdp *hdp)
{
	state_struct *state = &hdp->state;

	memset(state, 0, sizeof(state_struct));
	mutex_init(&state->mutex);

	state->mem = &hdp->mem;
	state->rw = hdp->rw;
	state->edp = hdp->is_edp;
}

#ifdef CONFIG_IMX_HDP_CEC
static void imx_hdp_cec_init(struct imx_hdp *hdp)
{
	state_struct *state = &hdp->state;
	struct imx_cec_dev *cec = &hdp->cec;
	u32 clk_MHz;

	memset(cec, 0, sizeof(struct imx_cec_dev));

	CDN_API_GetClock(state, &clk_MHz);
	cec->clk_div = clk_MHz * 10;
	cec->dev = hdp->dev;
	cec->mem = &hdp->mem;
	cec->rw = hdp->rw;
}
#endif

static void imx8qm_pixel_link_mux(state_struct *state, struct drm_display_mode *mode)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	u32 val;

	val = 4; /* RGB */
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= 1 << PL_MUX_CTL_VCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= 1 << PL_MUX_CTL_HCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		val |= 0x2;

	writel(val, hdp->mem.ss_base + CSR_PIXEL_LINK_MUX_CTL);
}

int imx8qm_pixel_link_init(state_struct *state)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	sc_err_t sciErr;

	sciErr = sc_ipc_getMuID(&hdp->mu_id);
	if (sciErr != SC_ERR_NONE) {
		DRM_ERROR("Cannot obtain MU ID\n");
		return -EINVAL;
	}

	sciErr = sc_ipc_open(&hdp->ipcHndl, hdp->mu_id);
	if (sciErr != SC_ERR_NONE) {
		DRM_ERROR("sc_ipc_open failed! (sciError = %d)\n", sciErr);
		return -EINVAL;
	}

	/* config dpu1 di0 to hdmi/dp mode */
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_ADDR, 1);
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_VLD, 1);
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_SYNC_CTRL0, 1);

	return true;
}

void imx8qm_pixel_link_deinit(state_struct *state)
{
	struct imx_hdp *hdp = state_to_imx_hdp(state);

	/* config dpu1 di0 to default mode */
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_ADDR, 0);
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_VLD, 0);
	sc_misc_set_control(hdp->ipcHndl, SC_R_DC_0, SC_C_SYNC_CTRL0, 0);

	sc_ipc_close(hdp->mu_id);
}

void imx8qm_phy_reset(sc_ipc_t ipcHndl, struct hdp_mem *mem, u8 reset)
{
	sc_err_t sciErr;
	/* set the pixel link mode and pixel type */
	sciErr = sc_misc_set_control(ipcHndl, SC_R_HDMI, SC_C_PHY_RESET, reset);
	if (sciErr != SC_ERR_NONE)
		DRM_ERROR("SC_R_HDMI PHY reset failed %d!\n", sciErr);
}

void imx8mq_phy_reset(sc_ipc_t ipcHndl, struct hdp_mem *mem, u8 reset)
{
	void *tmp_addr = mem->rst_base;

	if (reset)
		__raw_writel(0x8,
			     (volatile unsigned int *)(tmp_addr+0x4)); /*set*/
	else
		__raw_writel(0x8,
			     (volatile unsigned int *)(tmp_addr+0x8)); /*clear*/


	return;
}

int imx8qm_clock_init(struct hdp_clks *clks)
{
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	struct device *dev = hdp->dev;

	clks->av_pll = devm_clk_get(dev, "av_pll");
	if (IS_ERR(clks->av_pll)) {
		dev_warn(dev, "failed to get av pll clk\n");
		return PTR_ERR(clks->av_pll);
	}

	clks->dig_pll = devm_clk_get(dev, "dig_pll");
	if (IS_ERR(clks->dig_pll)) {
		dev_warn(dev, "failed to get dig pll clk\n");
		return PTR_ERR(clks->dig_pll);
	}

	clks->clk_ipg = devm_clk_get(dev, "clk_ipg");
	if (IS_ERR(clks->clk_ipg)) {
		dev_warn(dev, "failed to get dp ipg clk\n");
		return PTR_ERR(clks->clk_ipg);
	}

	clks->clk_core = devm_clk_get(dev, "clk_core");
	if (IS_ERR(clks->clk_core)) {
		dev_warn(dev, "failed to get hdp core clk\n");
		return PTR_ERR(clks->clk_core);
	}

	clks->clk_pxl = devm_clk_get(dev, "clk_pxl");
	if (IS_ERR(clks->clk_pxl)) {
		dev_warn(dev, "failed to get pxl clk\n");
		return PTR_ERR(clks->clk_pxl);
	}

	clks->clk_pxl_mux = devm_clk_get(dev, "clk_pxl_mux");
	if (IS_ERR(clks->clk_pxl_mux)) {
		dev_warn(dev, "failed to get pxl mux clk\n");
		return PTR_ERR(clks->clk_pxl_mux);
	}

	clks->clk_pxl_link = devm_clk_get(dev, "clk_pxl_link");
	if (IS_ERR(clks->clk_pxl_mux)) {
		dev_warn(dev, "failed to get pxl link clk\n");
		return PTR_ERR(clks->clk_pxl_link);
	}

	clks->clk_hdp = devm_clk_get(dev, "clk_hdp");
	if (IS_ERR(clks->clk_hdp)) {
		dev_warn(dev, "failed to get hdp clk\n");
		return PTR_ERR(clks->clk_hdp);
	}

	clks->clk_phy = devm_clk_get(dev, "clk_phy");
	if (IS_ERR(clks->clk_phy)) {
		dev_warn(dev, "failed to get phy clk\n");
		return PTR_ERR(clks->clk_phy);
	}
	clks->clk_apb = devm_clk_get(dev, "clk_apb");
	if (IS_ERR(clks->clk_apb)) {
		dev_warn(dev, "failed to get apb clk\n");
		return PTR_ERR(clks->clk_apb);
	}
	clks->clk_lis = devm_clk_get(dev, "clk_lis");
	if (IS_ERR(clks->clk_lis)) {
		dev_warn(dev, "failed to get lis clk\n");
		return PTR_ERR(clks->clk_lis);
	}
	clks->clk_msi = devm_clk_get(dev, "clk_msi");
	if (IS_ERR(clks->clk_msi)) {
		dev_warn(dev, "failed to get msi clk\n");
		return PTR_ERR(clks->clk_msi);
	}
	clks->clk_lpcg = devm_clk_get(dev, "clk_lpcg");
	if (IS_ERR(clks->clk_lpcg)) {
		dev_warn(dev, "failed to get lpcg clk\n");
		return PTR_ERR(clks->clk_lpcg);
	}
	clks->clk_even = devm_clk_get(dev, "clk_even");
	if (IS_ERR(clks->clk_even)) {
		dev_warn(dev, "failed to get even clk\n");
		return PTR_ERR(clks->clk_even);
	}
	clks->clk_dbl = devm_clk_get(dev, "clk_dbl");
	if (IS_ERR(clks->clk_dbl)) {
		dev_warn(dev, "failed to get dbl clk\n");
		return PTR_ERR(clks->clk_dbl);
	}
	clks->clk_vif = devm_clk_get(dev, "clk_vif");
	if (IS_ERR(clks->clk_vif)) {
		dev_warn(dev, "failed to get vif clk\n");
		return PTR_ERR(clks->clk_vif);
	}
	clks->clk_apb_csr = devm_clk_get(dev, "clk_apb_csr");
	if (IS_ERR(clks->clk_apb_csr)) {
		dev_warn(dev, "failed to get apb csr clk\n");
		return PTR_ERR(clks->clk_apb_csr);
	}
	clks->clk_apb_ctrl = devm_clk_get(dev, "clk_apb_ctrl");
	if (IS_ERR(clks->clk_apb_ctrl)) {
		dev_warn(dev, "failed to get apb ctrl clk\n");
		return PTR_ERR(clks->clk_apb_ctrl);
	}
	clks->clk_i2s = devm_clk_get(dev, "clk_i2s");
	if (IS_ERR(clks->clk_i2s)) {
		dev_warn(dev, "failed to get i2s clk\n");
		return PTR_ERR(clks->clk_i2s);
	}
	clks->clk_i2s_bypass = devm_clk_get(dev, "clk_i2s_bypass");
	if (IS_ERR(clks->clk_i2s_bypass)) {
		dev_err(dev, "failed to get i2s bypass clk\n");
		return PTR_ERR(clks->clk_i2s_bypass);
	}
	return true;
}

int imx8qm_pixel_clock_enable(struct hdp_clks *clks)
{
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	struct device *dev = hdp->dev;
	int ret;

	ret = clk_prepare_enable(clks->av_pll);
	if (ret < 0) {
		dev_err(dev, "%s, pre av pll  error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_pxl_mux);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl mux error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_pxl_link);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk pxl link error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_vif);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk vif error\n", __func__);
		return ret;
	}
	return ret;

}

void imx8qm_pixel_clock_disable(struct hdp_clks *clks)
{
	clk_disable_unprepare(clks->clk_vif);
	clk_disable_unprepare(clks->clk_pxl);
	clk_disable_unprepare(clks->clk_pxl_link);
	clk_disable_unprepare(clks->clk_pxl_mux);
	clk_disable_unprepare(clks->av_pll);
}

void imx8qm_dp_pixel_clock_set_rate(struct hdp_clks *clks)
{
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	unsigned int pclock = hdp->video.cur_mode.clock * 1000;

	if (!hdp->is_digpll_dp_pclock) {
		sc_err_t sci_err = 0;
		sc_ipc_t ipc_handle = 0;
		u32 mu_id;

		sci_err = sc_ipc_getMuID(&mu_id);

		if (sci_err != SC_ERR_NONE)
			pr_err("Failed to get MU ID (%d)\n", sci_err);
		sci_err = sc_ipc_open(&ipc_handle, mu_id);

		if (sci_err != SC_ERR_NONE)
			pr_err("Failed to open IPC (%d)\n", sci_err);

		clk_set_rate(clks->av_pll, pclock);

		/* Enable the 24MHz for HDP PHY */
		sc_misc_set_control(ipc_handle, SC_R_HDMI, SC_C_MODE, 1);

		sc_ipc_close(ipc_handle);
	} else
		clk_set_rate(clks->av_pll, 24000000);

	if (hdp->dual_mode == true) {
		clk_set_rate(clks->clk_pxl, pclock/2);
		clk_set_rate(clks->clk_pxl_link, pclock/2);
	} else {
		clk_set_rate(clks->clk_pxl, pclock);
		clk_set_rate(clks->clk_pxl_link, pclock);
	}
	clk_set_rate(clks->clk_pxl_mux, pclock);
}

void imx8qm_hdmi_pixel_clock_set_rate(struct hdp_clks *clks)
{
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	unsigned int pclock = hdp->video.cur_mode.clock * 1000;

	/* pixel clock for HDMI */
	clk_set_rate(clks->av_pll, pclock);

	if (hdp->dual_mode == true) {
		clk_set_rate(clks->clk_pxl, pclock/2);
		clk_set_rate(clks->clk_pxl_link, pclock/2);
	} else {
		clk_set_rate(clks->clk_pxl_link, pclock);
		clk_set_rate(clks->clk_pxl, pclock);
	}
	clk_set_rate(clks->clk_pxl_mux, pclock);
}

int imx8qm_ipg_clock_enable(struct hdp_clks *clks)
{
	int ret;
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	struct device *dev = hdp->dev;

	ret = clk_prepare_enable(clks->dig_pll);
	if (ret < 0) {
		dev_err(dev, "%s, pre dig pll error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_ipg);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk_ipg error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_core);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk core error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_hdp);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk hdp error\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_phy);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk phy\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(clks->clk_apb);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_lis);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk lis error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_lpcg);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk lpcg error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_msi);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk msierror\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_even);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk even error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_dbl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk dbl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_apb_csr);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb csr error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_apb_ctrl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb ctrl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_i2s);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk i2s error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->clk_i2s_bypass);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk i2s bypass error\n", __func__);
		return ret;
	}
	return ret;
}

void imx8qm_ipg_clock_disable(struct hdp_clks *clks)
{
}

void imx8qm_ipg_clock_set_rate(struct hdp_clks *clks)
{
	struct imx_hdp *hdp = clks_to_imx_hdp(clks);
	u32 clk_rate, desired_rate;

	if (hdp->is_digpll_dp_pclock)
		desired_rate = PLL_1188MHZ;
	else
		desired_rate = PLL_675MHZ;

	/* hdmi/dp ipg/core clock */
	clk_rate = clk_get_rate(clks->dig_pll);

	if (clk_rate != desired_rate) {
		pr_warn("%s, dig_pll was %u MHz, changing to %u MHz\n",
			__func__, clk_rate/1000000,
			desired_rate/1000000);
	}

	if (hdp->is_digpll_dp_pclock) {
		clk_set_rate(clks->dig_pll,  desired_rate);
		clk_set_rate(clks->clk_core, desired_rate/10);
		clk_set_rate(clks->clk_ipg,  desired_rate/12);
		clk_set_rate(clks->av_pll, 24000000);
	} else {
		clk_set_rate(clks->dig_pll,  desired_rate);
		clk_set_rate(clks->clk_core, desired_rate/5);
		clk_set_rate(clks->clk_ipg,  desired_rate/8);
	}
}

static u8 imx_hdp_link_rate(struct drm_display_mode *mode)
{
	if (mode->clock < 297000)
		return AFE_LINK_RATE_1_6;
	else if (mode->clock > 297000)
		return AFE_LINK_RATE_5_4;
	else
		return AFE_LINK_RATE_2_7;
}

static void imx_hdp_mode_setup(struct imx_hdp *hdp, struct drm_display_mode *mode)
{
	int ret;

	/* set pixel clock before video mode setup */
	imx_hdp_call(hdp, pixel_clock_disable, &hdp->clks);

	imx_hdp_call(hdp, pixel_clock_set_rate, &hdp->clks);

	imx_hdp_call(hdp, pixel_clock_enable, &hdp->clks);

	/* Config pixel link mux */
	imx_hdp_call(hdp, pixel_link_mux, &hdp->state, mode);

	hdp->link_rate = imx_hdp_link_rate(mode);

	/* mode set */
	ret = imx_hdp_call(hdp, phy_init, &hdp->state, mode, hdp->format, hdp->bpc);
	if (ret < 0) {
		DRM_ERROR("Failed to initialise HDP PHY\n");
		return;
	}
	imx_hdp_call(hdp, mode_set, &hdp->state, mode,
		     hdp->format, hdp->bpc, hdp->link_rate);

	/* Get vic of CEA-861 */
	hdp->vic = drm_match_cea_mode(mode);
}

bool imx_hdp_bridge_mode_fixup(struct drm_bridge *bridge,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode)
{
	struct imx_hdp *hdp = bridge->driver_private;
	struct drm_display_info *di = &hdp->connector.display_info;
	int vic = drm_match_cea_mode(mode);

	if (vic < 0)
		return false;

	if (vic == VIC_MODE_97_60Hz &&
	    (di->color_formats & DRM_COLOR_FORMAT_YCRCB420) &&
	    (di->hdmi.y420_dc_modes & DRM_EDID_YCBCR420_DC_30)) {
		hdp->bpc = 10;
		hdp->format = YCBCR_4_2_0;
		return true;
	}

	hdp->bpc = 8;
	hdp->format = PXL_RGB;

	return true;
}

static void imx_hdp_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *orig_mode,
				    struct drm_display_mode *mode)
{
	struct imx_hdp *hdp = bridge->driver_private;

	mutex_lock(&hdp->mutex);

	memcpy(&hdp->video.cur_mode, mode, sizeof(hdp->video.cur_mode));
	imx_hdp_mode_setup(hdp, mode);
	/* Store the display mode for plugin/DKMS poweron events */
	memcpy(&hdp->video.pre_mode, mode, sizeof(hdp->video.pre_mode));

	mutex_unlock(&hdp->mutex);
}

static void imx_hdp_bridge_disable(struct drm_bridge *bridge)
{
}

static void imx_hdp_bridge_enable(struct drm_bridge *bridge)
{
}

static enum drm_connector_status
imx_hdp_connector_detect(struct drm_connector *connector, bool force)
{
	struct imx_hdp *hdp = container_of(connector,
						struct imx_hdp, connector);
	int ret;
	u8 hpd = 0xf;

	ret = imx_hdp_call(hdp, get_hpd_state, &hdp->state, &hpd);
	if (ret > 0)
		return connector_status_unknown;

	if (hpd == 1)
		/* Cable Connected */
		return connector_status_connected;
	else if (hpd == 0)
		/* Cable Disconnedted */
		return connector_status_disconnected;
	else {
		/* Cable status unknown */
		DRM_INFO("Unknow cable status, hdp=%u\n", hpd);
		return connector_status_unknown;
	}
}

static int imx_hdp_default_video_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < ARRAY_SIZE(edid_cea_modes); i++) {
		mode = drm_mode_create(connector->dev);
		if (!mode)
			return -EINVAL;
		drm_mode_copy(mode, &edid_cea_modes[i]);
		mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}
	return i;
}

static int imx_hdp_connector_get_modes(struct drm_connector *connector)
{
	struct imx_hdp *hdp = container_of(connector, struct imx_hdp, connector);
	struct edid *edid;
	int num_modes = 0;

	if (hdp->is_edid == true) {
		edid = drm_do_get_edid(connector, hdp->ops->get_edid_block, &hdp->state);
		if (edid) {
			dev_dbg(hdp->dev, "%x,%x,%x,%x,%x,%x,%x,%x\n",
					edid->header[0], edid->header[1], edid->header[2], edid->header[3],
					edid->header[4], edid->header[5], edid->header[6], edid->header[7]);
			drm_mode_connector_update_edid_property(connector, edid);
			num_modes = drm_add_edid_modes(connector, edid);
			if (num_modes == 0) {
				dev_dbg(hdp->dev, "Invalid edid, use default video modes\n");
				num_modes = imx_hdp_default_video_modes(connector);
			} else
				/* Store the ELD */
				drm_edid_to_eld(connector, edid);
			kfree(edid);
		} else {
				dev_dbg(hdp->dev, "failed to get edid, use default video modes\n");
				num_modes = imx_hdp_default_video_modes(connector);
		}
	} else {
		dev_dbg(hdp->dev, "No EDID function, use default video mode\n");
		num_modes = imx_hdp_default_video_modes(connector);
	}

	return num_modes;
}

static enum drm_mode_status
imx_hdp_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	struct imx_hdp *hdp = container_of(connector, struct imx_hdp,
					     connector);
	enum drm_mode_status mode_status = MODE_OK;
	struct drm_cmdline_mode *cmdline_mode;
	int ret;

	cmdline_mode = &connector->cmdline_mode;

	/* cmdline mode is the max support video mode when edid disabled */
	if (!hdp->is_edid) {
		if (cmdline_mode->xres != 0 &&
			cmdline_mode->xres < mode->hdisplay)
			return MODE_BAD_HVALUE;
	}

	if (hdp->is_4kp60 && mode->clock > 594000)
		return MODE_CLOCK_HIGH;
	else if (!hdp->is_4kp60 && mode->clock > 297000)
		return MODE_CLOCK_HIGH;

	ret = imx_hdp_call(hdp, pixel_clock_range, mode);
	if (ret == 0) {
		DRM_DEBUG("pixel clock %d out of range\n", mode->clock);
		return MODE_CLOCK_RANGE;
	}

	/* 4096x2160 is not supported now */
	if (mode->hdisplay > 3840)
		return MODE_BAD_HVALUE;

	if (mode->vdisplay > 2160)
		return MODE_BAD_VVALUE;


	return mode_status;
}

static void imx_hdp_connector_force(struct drm_connector *connector)
{
	struct imx_hdp *hdp = container_of(connector, struct imx_hdp,
					     connector);

	mutex_lock(&hdp->mutex);
	hdp->force = connector->force;
	mutex_unlock(&hdp->mutex);
}

static int imx_hdp_set_property(struct drm_connector *connector,
				struct drm_connector_state *state,
				struct drm_property *property, uint64_t val)
{
	struct imx_hdp *hdp = container_of(connector, struct imx_hdp,
					   connector);
	int ret;
	union hdmi_infoframe frame;
	struct hdr_static_metadata *hdr_metadata;

	if (state->hdr_source_metadata_blob_ptr &&
	    state->hdr_source_metadata_blob_ptr->length &&
	    hdp->ops->write_hdr_metadata) {
		hdr_metadata = (struct hdr_static_metadata *)
				state->hdr_source_metadata_blob_ptr->data;

		ret = drm_hdmi_infoframe_set_hdr_metadata(&frame.drm,
							  hdr_metadata);

		if (ret < 0) {
			DRM_ERROR("could not set HDR metadata in infoframe\n");
			return ret;
		}

		hdp->ops->write_hdr_metadata(&hdp->state, &frame);
	}

	return 0;
}

static const struct drm_connector_funcs imx_hdp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = imx_hdp_connector_detect,
	.destroy = drm_connector_cleanup,
	.force = imx_hdp_connector_force,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_set_property = imx_hdp_set_property,
};

static const struct drm_connector_helper_funcs imx_hdp_connector_helper_funcs = {
	.get_modes = imx_hdp_connector_get_modes,
	.mode_valid = imx_hdp_connector_mode_valid,
};

static const struct drm_bridge_funcs imx_hdp_bridge_funcs = {
	.enable = imx_hdp_bridge_enable,
	.disable = imx_hdp_bridge_disable,
	.mode_set = imx_hdp_bridge_mode_set,
	.mode_fixup = imx_hdp_bridge_mode_fixup,
};

static void imx_hdp_imx_encoder_disable(struct drm_encoder *encoder)
{
}

static void imx_hdp_imx_encoder_enable(struct drm_encoder *encoder)
{
	struct imx_hdp *hdp = container_of(encoder, struct imx_hdp, encoder);
	union hdmi_infoframe frame;
	struct hdr_static_metadata *hdr_metadata;
	struct drm_connector_state *conn_state = hdp->connector.state;
	int ret = 0;

	if (!hdp->ops->write_hdr_metadata)
		return;

	if (hdp->hdr_metadata_present) {
		hdr_metadata = (struct hdr_static_metadata *)
				conn_state->hdr_source_metadata_blob_ptr->data;

		ret = drm_hdmi_infoframe_set_hdr_metadata(&frame.drm,
							  hdr_metadata);
	} else {
		hdr_metadata = devm_kzalloc(hdp->dev,
					    sizeof(struct hdr_static_metadata),
					    GFP_KERNEL);
		hdr_metadata->eotf = 0;

		ret = drm_hdmi_infoframe_set_hdr_metadata(&frame.drm,
							  hdr_metadata);

		devm_kfree(hdp->dev, hdr_metadata);
	}

	if (ret < 0) {
		DRM_ERROR("could not set HDR metadata in infoframe\n");
		return;
	}

	hdp->ops->write_hdr_metadata(&hdp->state, &frame);
}

static int imx_hdp_imx_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct imx_hdp *hdp = container_of(encoder, struct imx_hdp, encoder);

	imx_crtc_state->bus_format = MEDIA_BUS_FMT_RGB101010_1X30;

	if (conn_state->hdr_metadata_changed &&
	    conn_state->hdr_source_metadata_blob_ptr &&
	    conn_state->hdr_source_metadata_blob_ptr->length)
		hdp->hdr_metadata_present = true;

	return 0;
}

static const struct drm_encoder_helper_funcs imx_hdp_imx_encoder_helper_funcs = {
	.enable     = imx_hdp_imx_encoder_enable,
	.disable    = imx_hdp_imx_encoder_disable,
	.atomic_check = imx_hdp_imx_encoder_atomic_check,
};

static const struct drm_encoder_funcs imx_hdp_imx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int imx8mq_hdp_read(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = mem->regs_base + addr;
	temp = __raw_readl((volatile unsigned int *)tmp_addr);
	*value = temp;
	return 0;
}

static int imx8mq_hdp_write(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = mem->regs_base + addr;

	__raw_writel(value, (volatile unsigned int *)tmp_addr);
	return 0;
}

static int imx8mq_hdp_sread(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = mem->ss_base + addr;
	temp = __raw_readl((volatile unsigned int *)tmp_addr);
	*value = temp;
	return 0;
}

static int imx8mq_hdp_swrite(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = mem->ss_base + addr;
	__raw_writel(value, (volatile unsigned int *)tmp_addr);
	return 0;
}

static int imx8qm_hdp_read(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = (addr & 0xfff) + mem->regs_base;
	void *off_addr = 0x8 + mem->ss_base;;

	__raw_writel(addr >> 12, off_addr);
	temp = __raw_readl((volatile unsigned int *)tmp_addr);

	*value = temp;
	return 0;
}

static int imx8qm_hdp_write(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = (addr & 0xfff) + mem->regs_base;
	void *off_addr = 0x8 + mem->ss_base;;

	__raw_writel(addr >> 12, off_addr);

	__raw_writel(value, (volatile unsigned int *) tmp_addr);

	return 0;
}

static int imx8qm_hdp_sread(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = (addr & 0xfff) + mem->regs_base;
	void *off_addr = 0xc + mem->ss_base;;

	__raw_writel(addr >> 12, off_addr);

	temp = __raw_readl((volatile unsigned int *)tmp_addr);
	*value = temp;
	return 0;
}

static int imx8qm_hdp_swrite(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = (addr & 0xfff) + mem->regs_base;
	void *off_addr = 0xc + mem->ss_base;

	__raw_writel(addr >> 12, off_addr);
	__raw_writel(value, (volatile unsigned int *)tmp_addr);

	return 0;
}

static struct hdp_rw_func imx8qm_rw = {
	.read_reg = imx8qm_hdp_read,
	.write_reg = imx8qm_hdp_write,
	.sread_reg = imx8qm_hdp_sread,
	.swrite_reg = imx8qm_hdp_swrite,
};

static struct hdp_ops imx8qm_dp_ops = {
#ifdef DEBUG_FW_LOAD
	.fw_load = dp_fw_load,
#endif
	.fw_init = dp_fw_init,
	.phy_init = dp_phy_init,
	.mode_set = dp_mode_set,
	.get_edid_block = dp_get_edid_block,
	.get_hpd_state = dp_get_hpd_state,

	.phy_reset = imx8qm_phy_reset,
	.pixel_link_init = imx8qm_pixel_link_init,
	.pixel_link_deinit = imx8qm_pixel_link_deinit,
	.pixel_link_mux = imx8qm_pixel_link_mux,

	.clock_init = imx8qm_clock_init,
	.ipg_clock_set_rate = imx8qm_ipg_clock_set_rate,
	.ipg_clock_enable = imx8qm_ipg_clock_enable,
	.ipg_clock_disable = imx8qm_ipg_clock_disable,
	.pixel_clock_set_rate = imx8qm_dp_pixel_clock_set_rate,
	.pixel_clock_enable = imx8qm_pixel_clock_enable,
	.pixel_clock_disable = imx8qm_pixel_clock_disable,
};

static struct hdp_ops imx8qm_hdmi_ops = {
#ifdef DEBUG_FW_LOAD
	.fw_load = hdmi_fw_load,
#endif
	.fw_init = hdmi_fw_init,
	.phy_init = hdmi_phy_init_ss28fdsoi,
	.mode_set = hdmi_mode_set_ss28fdsoi,
	.get_edid_block = hdmi_get_edid_block,
	.get_hpd_state = hdmi_get_hpd_state,

	.phy_reset = imx8qm_phy_reset,
	.pixel_link_init = imx8qm_pixel_link_init,
	.pixel_link_deinit = imx8qm_pixel_link_deinit,
	.pixel_link_mux = imx8qm_pixel_link_mux,

	.clock_init = imx8qm_clock_init,
	.ipg_clock_set_rate = imx8qm_ipg_clock_set_rate,
	.ipg_clock_enable = imx8qm_ipg_clock_enable,
	.ipg_clock_disable = imx8qm_ipg_clock_disable,
	.pixel_clock_set_rate = imx8qm_hdmi_pixel_clock_set_rate,
	.pixel_clock_enable = imx8qm_pixel_clock_enable,
	.pixel_clock_disable = imx8qm_pixel_clock_disable,
};

static struct hdp_devtype imx8qm_dp_devtype = {
	.is_edid = false,
	.is_4kp60 = false,
	.audio_type = CDN_DPTX,
	.ops = &imx8qm_dp_ops,
	.rw = &imx8qm_rw,
};

static struct hdp_devtype imx8qm_hdmi_devtype = {
	.is_edid = false,
	.is_4kp60 = false,
	.audio_type = CDN_HDMITX_TYPHOON,
	.ops = &imx8qm_hdmi_ops,
	.rw = &imx8qm_rw,
};

static struct hdp_rw_func imx8mq_rw = {
	.read_reg = imx8mq_hdp_read,
	.write_reg = imx8mq_hdp_write,
	.sread_reg = imx8mq_hdp_sread,
	.swrite_reg = imx8mq_hdp_swrite,
};

static struct hdp_ops imx8mq_ops = {
	.phy_init = hdmi_phy_init_t28hpc,
	.mode_set = hdmi_mode_set_t28hpc,
	.get_edid_block = hdmi_get_edid_block,
	.get_hpd_state = hdmi_get_hpd_state,
	.write_hdr_metadata = hdmi_write_hdr_metadata,
	.pixel_clock_range = pixel_clock_range_t28hpc,
};

static struct hdp_devtype imx8mq_hdmi_devtype = {
	.is_edid = true,
	.is_4kp60 = true,
	.audio_type = CDN_HDMITX_KIRAN,
	.ops = &imx8mq_ops,
	.rw = &imx8mq_rw,
};

static struct hdp_ops imx8mq_dp_ops = {
	.phy_init = dp_phy_init_t28hpc,
	.mode_set = dp_mode_set,
	.get_edid_block = dp_get_edid_block,
	.get_hpd_state = dp_get_hpd_state,
	.phy_reset = imx8mq_phy_reset,
};

static struct hdp_devtype imx8mq_dp_devtype = {
	.is_edid = true,
	.is_4kp60 = true,
	.audio_type = CDN_DPTX,
	.ops = &imx8mq_dp_ops,
	.rw = &imx8mq_rw,
};

static const struct of_device_id imx_hdp_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-hdmi", .data = &imx8qm_hdmi_devtype},
	{ .compatible = "fsl,imx8qm-dp", .data = &imx8qm_dp_devtype},
	{ .compatible = "fsl,imx8mq-hdmi", .data = &imx8mq_hdmi_devtype},
	{ .compatible = "fsl,imx8mq-dp", .data = &imx8mq_dp_devtype},
	{ }
};
MODULE_DEVICE_TABLE(of, imx_hdp_dt_ids);

static void hotplug_work_func(struct work_struct *work)
{
	struct imx_hdp *hdp = container_of(work, struct imx_hdp,
								hotplug_work.work);
	struct drm_connector *connector = &hdp->connector;

	drm_helper_hpd_irq_event(connector->dev);

	if (connector->status == connector_status_connected) {
		/* Cable Connected */
		/* For HDMI2.0 SCDC should setup again.
		 * So recovery pre video mode if it is 4Kp60 */
		if (drm_mode_equal(&hdp->video.pre_mode, &edid_cea_modes[3]))
			imx_hdp_mode_setup(hdp, &hdp->video.pre_mode);
		DRM_INFO("HDMI/DP Cable Plug In\n");
		enable_irq(hdp->irq[HPD_IRQ_OUT]);
	} else if (connector->status == connector_status_disconnected) {
		/* Cable Disconnedted  */
		DRM_INFO("HDMI/DP Cable Plug Out\n");
		enable_irq(hdp->irq[HPD_IRQ_IN]);
	}
}

static irqreturn_t imx_hdp_irq_thread(int irq, void *data)
{
	struct imx_hdp *hdp = data;

	disable_irq_nosync(irq);

	mod_delayed_work(system_wq, &hdp->hotplug_work,
			msecs_to_jiffies(HOTPLUG_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static int imx_hdp_imx_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct imx_hdp *hdp;
	const struct of_device_id *of_id =
			of_match_device(imx_hdp_dt_ids, dev);
	const struct hdp_devtype *devtype = of_id->data;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	struct resource *res;
	u8 hpd;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	hdp = devm_kzalloc(&pdev->dev, sizeof(*hdp), GFP_KERNEL);
	if (!hdp)
		return -ENOMEM;

	hdp->dev = &pdev->dev;
	encoder = &hdp->encoder;
	bridge = &hdp->bridge;
	connector = &hdp->connector;

	mutex_init(&hdp->mutex);

	hdp->irq[HPD_IRQ_IN] = platform_get_irq_byname(pdev, "plug_in");
	if (hdp->irq[HPD_IRQ_IN] < 0)
		dev_info(&pdev->dev, "No plug_in irq number\n");

	hdp->irq[HPD_IRQ_OUT] = platform_get_irq_byname(pdev, "plug_out");
	if (hdp->irq[HPD_IRQ_OUT] < 0)
		dev_info(&pdev->dev, "No plug_out irq number\n");

	/* register map */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdp->mem.regs_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdp->mem.regs_base)) {
		dev_err(dev, "Failed to get HDP CTRL base register\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	hdp->mem.ss_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdp->mem.ss_base)) {
		dev_err(dev, "Failed to get HDP CRS base register\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	hdp->mem.rst_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdp->mem.rst_base)) {
		dev_warn(dev, "Failed to get HDP RESET base register\n");
	}

	hdp->is_cec = of_property_read_bool(pdev->dev.of_node, "fsl,cec");

	hdp->is_digpll_dp_pclock = of_property_read_bool(pdev->dev.of_node, "fsl,use_digpll_pclock");

	hdp->is_edp = of_property_read_bool(pdev->dev.of_node, "fsl,edp");

	ret = of_property_read_u32(pdev->dev.of_node,
				       "lane_mapping",
				       &hdp->lane_mapping);
	if (ret) {
		hdp->lane_mapping = 0x1b;
		dev_warn(dev, "Failed to get lane_mapping - using default\n");
	}
	dev_info(dev, "lane_mapping 0x%02x\n", hdp->lane_mapping);

	ret = of_property_read_u32(pdev->dev.of_node,
				       "edp_link_rate",
				       &hdp->edp_link_rate);
	if (ret) {
		hdp->edp_link_rate = 0;
		dev_warn(dev, "Failed to get dp_link_rate - using default\n");
	}
	dev_info(dev, "edp_link_rate 0x%02x\n", hdp->edp_link_rate);

	ret = of_property_read_u32(pdev->dev.of_node,
				       "edp_num_lanes",
				       &hdp->edp_num_lanes);
	if (ret) {
		hdp->edp_num_lanes = 4;
		dev_warn(dev, "Failed to get dp_num_lanes - using default\n");
	}
	dev_info(dev, "dp_num_lanes 0x%02x\n", hdp->edp_num_lanes);

	hdp->is_edid = devtype->is_edid;
	hdp->is_4kp60 = devtype->is_4kp60;
	hdp->audio_type = devtype->audio_type;
	hdp->ops = devtype->ops;
	hdp->rw = devtype->rw;
	hdp->bpc = 8;
	hdp->format = PXL_RGB;

	/* HDP controller init */
	imx_hdp_state_init(hdp);

	hdp->link_rate = AFE_LINK_RATE_1_6;

	hdp->dual_mode = false;

	ret = imx_hdp_call(hdp, pixel_link_init, &hdp->state);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize clock %d\n", ret);
		return ret;
	}

	ret = imx_hdp_call(hdp, clock_init, &hdp->clks);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize clock\n");
		return ret;
	}

	imx_hdp_call(hdp, ipg_clock_set_rate, &hdp->clks);

	ret = imx_hdp_call(hdp, ipg_clock_enable, &hdp->clks);
	if (ret < 0) {
		DRM_ERROR("Failed to initialize IPG clock\n");
		return ret;
	}
	imx_hdp_call(hdp, pixel_clock_enable, &hdp->clks);

	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, &hdp->mem, 0);

	imx_hdp_call(hdp, fw_load, &hdp->state);

	ret = imx_hdp_call(hdp, fw_init, &hdp->state);
	if (ret < 0) {
		DRM_ERROR("Failed to initialise HDP firmware\n");
		return ret;
	}

	/* Pixel Format - 1 RGB, 2 YCbCr 444, 3 YCbCr 420 */
	/* bpp (bits per subpixel) - 8 24bpp, 10 30bpp, 12 36bpp, 16 48bpp */
	/* default set hdmi to 1080p60 mode */
	ret = imx_hdp_call(hdp, phy_init, &hdp->state, &edid_cea_modes[2],
			   hdp->format, hdp->bpc);
	if (ret < 0) {
		DRM_ERROR("Failed to initialise HDP PHY\n");
		return ret;
	}

	/* encoder */
	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	/* encoder */
	drm_encoder_helper_add(encoder, &imx_hdp_imx_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &imx_hdp_imx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	/* bridge */
	bridge->encoder = encoder;
	bridge->driver_private = hdp;
	bridge->funcs = &imx_hdp_bridge_funcs;
	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret) {
		DRM_ERROR("Failed to initialize bridge with drm\n");
		return -EINVAL;
	}

	encoder->bridge = bridge;
	hdp->connector.polled = DRM_CONNECTOR_POLL_HPD;
	hdp->connector.ycbcr_420_allowed = true;

	/* connector */
	drm_connector_helper_add(connector,
				 &imx_hdp_connector_helper_funcs);

	drm_connector_init(drm, connector,
			   &imx_hdp_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);

	drm_object_attach_property(&connector->base,
		connector->dev->mode_config.hdr_source_metadata_property, 0);

	drm_mode_connector_attach_encoder(connector, encoder);

	dev_set_drvdata(dev, hdp);

	INIT_DELAYED_WORK(&hdp->hotplug_work, hotplug_work_func);

	/* Check cable states before enable irq */
	imx_hdp_call(hdp, get_hpd_state, &hdp->state, &hpd);

	/* Enable Hotplug Detect IRQ thread */
	if (hdp->irq[HPD_IRQ_IN] > 0) {
		irq_set_status_flags(hdp->irq[HPD_IRQ_IN], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdp->irq[HPD_IRQ_IN],
						NULL, imx_hdp_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdp);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
							hdp->irq[HPD_IRQ_IN]);
			goto err_irq;
		}
		/* Cable Disconnedted, enable Plug in IRQ */
		if (hpd == 0)
			enable_irq(hdp->irq[HPD_IRQ_IN]);
	}
	if (hdp->irq[HPD_IRQ_OUT] > 0) {
		irq_set_status_flags(hdp->irq[HPD_IRQ_OUT], IRQ_NOAUTOEN);
		ret = devm_request_threaded_irq(dev, hdp->irq[HPD_IRQ_OUT],
						NULL, imx_hdp_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hdp);
		if (ret) {
			dev_err(&pdev->dev, "can't claim irq %d\n",
							hdp->irq[HPD_IRQ_OUT]);
			goto err_irq;
		}
		/* Cable Connected, enable Plug out IRQ */
		if (hpd == 1)
			enable_irq(hdp->irq[HPD_IRQ_OUT]);
	}
#ifdef CONFIG_IMX_HDP_CEC
	if (hdp->is_cec) {
		imx_hdp_cec_init(hdp);
		imx_cec_register(&hdp->cec);
	}
#endif

	imx_hdp_register_audio_driver(dev);

	return 0;
err_irq:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void imx_hdp_imx_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_hdp *hdp = dev_get_drvdata(dev);

#ifdef CONFIG_IMX_HDP_CEC
	if (hdp->is_cec)
		imx_cec_unregister(&hdp->cec);
#endif
	imx_hdp_call(hdp, pixel_clock_disable, &hdp->clks);
	imx_hdp_call(hdp, pixel_link_deinit, &hdp->state);
}

static const struct component_ops imx_hdp_imx_ops = {
	.bind	= imx_hdp_imx_bind,
	.unbind	= imx_hdp_imx_unbind,
};

static int imx_hdp_imx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &imx_hdp_imx_ops);
}

static int imx_hdp_imx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &imx_hdp_imx_ops);

	return 0;
}

static struct platform_driver imx_hdp_imx_platform_driver = {
	.probe  = imx_hdp_imx_probe,
	.remove = imx_hdp_imx_remove,
	.driver = {
		.name = "i.mx8-hdp",
		.of_match_table = imx_hdp_dt_ids,
	},
};

module_platform_driver(imx_hdp_imx_platform_driver);

MODULE_AUTHOR("Sandor Yu <Sandor.yu@nxp.com>");
MODULE_DESCRIPTION("IMX8QM DP Display Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dp-hdmi-imx");
