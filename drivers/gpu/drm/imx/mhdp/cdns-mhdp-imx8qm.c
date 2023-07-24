/*
 * Copyright 2019-2022 NXP
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license version 2 as
 * published by the free software foundation.
 */
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/sci.h>
#include <linux/firmware.h>
#include <linux/pm_domain.h>
#include <linux/clk.h>
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>

#include "cdns-mhdp-imx.h"

#define FW_IRAM_OFFSET		0x2000
#define FW_IRAM_SIZE		0x10000
#define FW_DRAM_SIZE		0x8000

#define PLL_800MHZ (800000000)

#define HDP_DUAL_MODE_MIN_PCLK_RATE	300000	/* KHz */
#define HDP_SINGLE_MODE_MAX_WIDTH	2560

#define CSR_PIXEL_LINK_MUX_CTL		0x00
#define CSR_PIXEL_LINK_MUX_VCP_OFFSET		5
#define CSR_PIXEL_LINK_MUX_HCP_OFFSET		4

static bool imx8qm_video_dual_mode(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	return (mode->clock > HDP_DUAL_MODE_MIN_PCLK_RATE ||
		mode->hdisplay > HDP_SINGLE_MODE_MAX_WIDTH) ? true : false;
}

static void imx8qm_pixel_link_mux(struct imx_mhdp_device *imx_mhdp)
{
	struct drm_display_mode *mode = &imx_mhdp->mhdp.mode;
	bool dual_mode;
	u32 val;

	dual_mode = imx8qm_video_dual_mode(&imx_mhdp->mhdp);

	val = 0x4;	/* RGB */
	if (dual_mode)
		val |= 0x2;	/* pixel link 0 and 1 are active */
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= 1 << CSR_PIXEL_LINK_MUX_VCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= 1 << CSR_PIXEL_LINK_MUX_HCP_OFFSET;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		val |= 0x2;

	writel(val, imx_mhdp->mhdp.regs_sec);
}

static void imx8qm_pixel_link_valid(u32 dual_mode)
{
	struct imx_sc_ipc *handle;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_ERROR("Failed to get scu ipc handle (%d)\n", ret);
		return;
	}

	imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST1_VLD, 1);
	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST2_VLD, 1);
}

static void imx8qm_pixel_link_invalid(u32 dual_mode)
{
	struct imx_sc_ipc *handle;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_ERROR("Failed to get scu ipc handle (%d)\n", ret);
		return;
	}

	imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST1_VLD, 0);
	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_PXL_LINK_MST2_VLD, 0);
}

static void imx8qm_pixel_link_sync_enable(u32 dual_mode)
{
	struct imx_sc_ipc *handle;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_ERROR("Failed to get scu ipc handle (%d)\n", ret);
		return;
	}

	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL, 3);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL0, 1);
}

static void imx8qm_pixel_link_sync_disable(u32 dual_mode)
{
	struct imx_sc_ipc *handle;
	int ret = 0;

	ret = imx_scu_get_handle(&handle);
	if (ret) {
		DRM_ERROR("Failed to get scu ipc handle (%d)\n", ret);
		return;
	}

	if (dual_mode)
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL, 0);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_DC_0, IMX_SC_C_SYNC_CTRL0, 0);
}

void imx8qm_phy_reset(u8 reset)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	/* set the pixel link mode and pixel type */
	imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_PHY_RESET, reset);
}

static void imx8qm_clk_mux(u8 is_dp)
{
	struct imx_sc_ipc *handle;

	imx_scu_get_handle(&handle);

	if (is_dp)
		/* Enable the 24MHz for HDP PHY */
		imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_MODE, 1);
	else
		imx_sc_misc_set_control(handle, IMX_SC_R_HDMI, IMX_SC_C_MODE, 0);
}

int imx8qm_clocks_init(struct imx_mhdp_device *imx_mhdp)
{
	struct device *dev = imx_mhdp->mhdp.dev;
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clks->dig_pll = devm_clk_get(dev, "dig_pll");
	if (IS_ERR(clks->dig_pll)) {
		dev_warn(dev, "failed to get dig pll clk\n");
		return PTR_ERR(clks->dig_pll);
	}

	clks->av_pll = devm_clk_get(dev, "av_pll");
	if (IS_ERR(clks->av_pll)) {
		dev_warn(dev, "failed to get av pll clk\n");
		return PTR_ERR(clks->av_pll);
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
	if (IS_ERR(clks->clk_pxl_link)) {
		dev_warn(dev, "failed to get pxl link clk\n");
		return PTR_ERR(clks->clk_pxl_link);
	}

	clks->lpcg_hdp = devm_clk_get(dev, "lpcg_hdp");
	if (IS_ERR(clks->lpcg_hdp)) {
		dev_warn(dev, "failed to get lpcg hdp clk\n");
		return PTR_ERR(clks->lpcg_hdp);
	}

	clks->lpcg_msi = devm_clk_get(dev, "lpcg_msi");
	if (IS_ERR(clks->lpcg_msi)) {
		dev_warn(dev, "failed to get lpcg msi clk\n");
		return PTR_ERR(clks->lpcg_msi);
	}

	clks->lpcg_pxl = devm_clk_get(dev, "lpcg_pxl");
	if (IS_ERR(clks->lpcg_pxl)) {
		dev_warn(dev, "failed to get lpcg pxl clk\n");
		return PTR_ERR(clks->lpcg_pxl);
	}

	clks->lpcg_vif = devm_clk_get(dev, "lpcg_vif");
	if (IS_ERR(clks->lpcg_vif)) {
		dev_warn(dev, "failed to get lpcg vif clk\n");
		return PTR_ERR(clks->lpcg_vif);
	}

	clks->lpcg_lis = devm_clk_get(dev, "lpcg_lis");
	if (IS_ERR(clks->lpcg_lis)) {
		dev_warn(dev, "failed to get lpcg lis clk\n");
		return PTR_ERR(clks->lpcg_lis);
	}

	clks->lpcg_apb = devm_clk_get(dev, "lpcg_apb");
	if (IS_ERR(clks->lpcg_apb)) {
		dev_warn(dev, "failed to get lpcg apb clk\n");
		return PTR_ERR(clks->lpcg_apb);
	}

	clks->lpcg_apb_csr = devm_clk_get(dev, "lpcg_apb_csr");
	if (IS_ERR(clks->lpcg_apb_csr)) {
		dev_warn(dev, "failed to get apb csr clk\n");
		return PTR_ERR(clks->lpcg_apb_csr);
	}

	clks->lpcg_apb_ctrl = devm_clk_get(dev, "lpcg_apb_ctrl");
	if (IS_ERR(clks->lpcg_apb_ctrl)) {
		dev_warn(dev, "failed to get lpcg apb ctrl clk\n");
		return PTR_ERR(clks->lpcg_apb_ctrl);
	}

	clks->clk_i2s_bypass = devm_clk_get(dev, "clk_i2s_bypass");
	if (IS_ERR(clks->clk_i2s_bypass)) {
		dev_err(dev, "failed to get i2s bypass clk\n");
		return PTR_ERR(clks->clk_i2s_bypass);
	}

	clks->lpcg_i2s = devm_clk_get(dev, "lpcg_i2s");
	if (IS_ERR(clks->lpcg_i2s)) {
		dev_err(dev, "failed to get lpcg i2s clk\n");
		return PTR_ERR(clks->lpcg_i2s);
	}
	return true;
}

static int imx8qm_pixel_clk_enable(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;
	struct device *dev = imx_mhdp->mhdp.dev;
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
	ret = clk_prepare_enable(clks->lpcg_vif);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk vif error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_pxl);
	if (ret < 0) {
		dev_err(dev, "%s, pre lpcg pxl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_hdp);
	if (ret < 0) {
		dev_err(dev, "%s, pre lpcg hdp error\n", __func__);
		return ret;
	}
	return ret;
}

static void imx8qm_pixel_clk_disable(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clk_disable_unprepare(clks->lpcg_pxl);
	clk_disable_unprepare(clks->lpcg_hdp);
	clk_disable_unprepare(clks->lpcg_vif);
	clk_disable_unprepare(clks->clk_pxl);
	clk_disable_unprepare(clks->clk_pxl_link);
	clk_disable_unprepare(clks->clk_pxl_mux);
	clk_disable_unprepare(clks->av_pll);
}

static void imx8qm_pixel_clk_set_rate(struct imx_mhdp_device *imx_mhdp, u32 pclock)
{
	bool dual_mode = imx8qm_video_dual_mode(&imx_mhdp->mhdp);
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	/* pixel clock for HDMI */
	clk_set_rate(clks->av_pll, pclock);

	if (dual_mode == true) {
		clk_set_rate(clks->clk_pxl, pclock/2);
		clk_set_rate(clks->clk_pxl_link, pclock/2);
	} else {
		clk_set_rate(clks->clk_pxl_link, pclock);
		clk_set_rate(clks->clk_pxl, pclock);
	}
	clk_set_rate(clks->clk_pxl_mux, pclock);
}

static int imx8qm_ipg_clk_enable(struct imx_mhdp_device *imx_mhdp)
{
	int ret;
	struct imx_hdp_clks *clks = &imx_mhdp->clks;
	struct device *dev = imx_mhdp->mhdp.dev;

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

	ret = clk_prepare_enable(clks->lpcg_apb);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_lis);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk lis error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_msi);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk msierror\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_apb_csr);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb csr error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_apb_ctrl);
	if (ret < 0) {
		dev_err(dev, "%s, pre clk apb ctrl error\n", __func__);
		return ret;
	}
	ret = clk_prepare_enable(clks->lpcg_i2s);
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

static void imx8qm_ipg_clk_disable(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	clk_disable_unprepare(clks->clk_i2s_bypass);
	clk_disable_unprepare(clks->lpcg_i2s);
	clk_disable_unprepare(clks->lpcg_apb_ctrl);
	clk_disable_unprepare(clks->lpcg_apb_csr);
	clk_disable_unprepare(clks->lpcg_msi);
	clk_disable_unprepare(clks->lpcg_lis);
	clk_disable_unprepare(clks->lpcg_apb);
	clk_disable_unprepare(clks->clk_core);
	clk_disable_unprepare(clks->clk_ipg);
	clk_disable_unprepare(clks->dig_pll);
}

static void imx8qm_ipg_clk_set_rate(struct imx_mhdp_device *imx_mhdp)
{
	struct imx_hdp_clks *clks = &imx_mhdp->clks;

	/* ipg/core clock */
	clk_set_rate(clks->dig_pll,  PLL_800MHZ);
	clk_set_rate(clks->clk_core, PLL_800MHZ/4);
	clk_set_rate(clks->clk_ipg,  PLL_800MHZ/8);
}

static void imx8qm_detach_pm_domains(struct imx_mhdp_device *imx_mhdp)
{
	if (imx_mhdp->pd_pll1_link && !IS_ERR(imx_mhdp->pd_pll1_link))
		device_link_del(imx_mhdp->pd_pll1_link);
	if (imx_mhdp->pd_pll1_dev && !IS_ERR(imx_mhdp->pd_pll1_dev))
		dev_pm_domain_detach(imx_mhdp->pd_pll1_dev, true);

	if (imx_mhdp->pd_pll0_link && !IS_ERR(imx_mhdp->pd_pll0_link))
		device_link_del(imx_mhdp->pd_pll0_link);
	if (imx_mhdp->pd_pll0_dev && !IS_ERR(imx_mhdp->pd_pll0_dev))
		dev_pm_domain_detach(imx_mhdp->pd_pll0_dev, true);

	if (imx_mhdp->pd_mhdp_link && !IS_ERR(imx_mhdp->pd_mhdp_link))
		device_link_del(imx_mhdp->pd_mhdp_link);
	if (imx_mhdp->pd_mhdp_dev && !IS_ERR(imx_mhdp->pd_mhdp_dev))
		dev_pm_domain_detach(imx_mhdp->pd_mhdp_dev, true);

	imx_mhdp->pd_mhdp_dev = NULL;
	imx_mhdp->pd_mhdp_link = NULL;
	imx_mhdp->pd_pll0_dev = NULL;
	imx_mhdp->pd_pll0_link = NULL;
	imx_mhdp->pd_pll1_dev = NULL;
	imx_mhdp->pd_pll1_link = NULL;
}

static int imx8qm_attach_pm_domains(struct imx_mhdp_device *imx_mhdp)
{
	struct device *dev = imx_mhdp->mhdp.dev;
	u32 flags = DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;
	int ret = 0;

	imx_mhdp->pd_mhdp_dev = dev_pm_domain_attach_by_name(dev, "hdmi");
	if (IS_ERR(imx_mhdp->pd_mhdp_dev)) {
		ret = PTR_ERR(imx_mhdp->pd_mhdp_dev);
		dev_err(dev, "Failed to attach dc pd dev: %d\n", ret);
		goto fail;
	}
	imx_mhdp->pd_mhdp_link = device_link_add(dev, imx_mhdp->pd_mhdp_dev, flags);
	if (IS_ERR(imx_mhdp->pd_mhdp_link)) {
		ret = PTR_ERR(imx_mhdp->pd_mhdp_link);
		dev_err(dev, "Failed to add device link to dc pd dev: %d\n",
			ret);
		goto fail;
	}

	imx_mhdp->pd_pll0_dev = dev_pm_domain_attach_by_name(dev, "pll0");
	if (IS_ERR(imx_mhdp->pd_pll0_dev)) {
		ret = PTR_ERR(imx_mhdp->pd_pll0_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	imx_mhdp->pd_pll0_link = device_link_add(dev, imx_mhdp->pd_pll0_dev, flags);
	if (IS_ERR(imx_mhdp->pd_pll0_link)) {
		ret = PTR_ERR(imx_mhdp->pd_pll0_link);
		dev_err(dev, "Failed to add device link to pll0 pd dev: %d\n",
			ret);
		goto fail;
	}

	imx_mhdp->pd_pll1_dev = dev_pm_domain_attach_by_name(dev, "pll1");
	if (IS_ERR(imx_mhdp->pd_pll1_dev)) {
		ret = PTR_ERR(imx_mhdp->pd_pll1_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	imx_mhdp->pd_pll1_link = device_link_add(dev, imx_mhdp->pd_pll1_dev, flags);
	if (IS_ERR(imx_mhdp->pd_pll1_link)) {
		ret = PTR_ERR(imx_mhdp->pd_pll1_link);
		dev_err(dev, "Failed to add device link to pll1 pd dev: %d\n",
			ret);
		goto fail;
	}
fail:
	imx8qm_detach_pm_domains(imx_mhdp);
	return ret;
}

int cdns_mhdp_power_on_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);
	/* Power on PM Domains */

	imx8qm_attach_pm_domains(imx_mhdp);

	/* clock init and  rate set */
	imx8qm_clocks_init(imx_mhdp);

	imx8qm_ipg_clk_set_rate(imx_mhdp);

	/* Init pixel clock with 148.5MHz before FW init */
	imx8qm_pixel_clk_set_rate(imx_mhdp, 148500000);

	imx8qm_ipg_clk_enable(imx_mhdp);

	imx8qm_clk_mux(imx_mhdp->mhdp.plat_data->is_dp);

	imx8qm_pixel_clk_enable(imx_mhdp);

	imx8qm_phy_reset(1);

	return 0;
}

int cdns_mhdp_power_off_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);

	imx8qm_phy_reset(0);

	/* disable pixel and ipg clock */
	imx8qm_pixel_clk_disable(imx_mhdp);
	imx8qm_ipg_clk_disable(imx_mhdp);

	imx8qm_detach_pm_domains(imx_mhdp);
	return 0;
}

void cdns_mhdp_plat_deinit_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);
	bool dual_mode = imx8qm_video_dual_mode(&imx_mhdp->mhdp);

	imx8qm_pixel_link_sync_disable(dual_mode);
	imx8qm_pixel_link_invalid(dual_mode);
}

void cdns_mhdp_plat_init_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);
	bool dual_mode = imx8qm_video_dual_mode(&imx_mhdp->mhdp);

	imx8qm_pixel_link_valid(dual_mode);
	imx8qm_pixel_link_sync_enable(dual_mode);
}

void cdns_mhdp_pclk_rate_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);

	mutex_lock(&mhdp->iolock);

	/* set pixel clock before video mode setup */
	imx8qm_pixel_clk_disable(imx_mhdp);

	imx8qm_pixel_clk_set_rate(imx_mhdp, imx_mhdp->mhdp.mode.clock * 1000);

	imx8qm_pixel_clk_enable(imx_mhdp);

	mutex_unlock(&mhdp->iolock);

	/* Config pixel link mux */
	imx8qm_pixel_link_mux(imx_mhdp);
}

int cdns_mhdp_firmware_write_section(struct imx_mhdp_device *imx_mhdp,
					const u8 *data, int size, int addr)
{
	int i;

	for (i = 0; i < size; i += 4) {
		u32 val = (unsigned int)data[i] << 0 |
					(unsigned int)data[i + 1] << 8 |
					(unsigned int)data[i + 2] << 16 |
					(unsigned int)data[i + 3] << 24;
		cdns_mhdp_bus_write(val, &imx_mhdp->mhdp, addr + i);
	}

	return 0;
}

static void cdns_mhdp_firmware_load_cont(const struct firmware *fw, void *context)
{
	struct imx_mhdp_device *imx_mhdp = context;

	imx_mhdp->fw = fw;
}

static int cdns_mhdp_firmware_load(struct imx_mhdp_device *imx_mhdp)
{
	const u8 *iram;
	const u8 *dram;
	u32 rate;
	int ret;

	/* configure HDMI/DP core clock */
	rate = clk_get_rate(imx_mhdp->clks.clk_core);
	if (imx_mhdp->mhdp.is_ls1028a)
		rate = rate / 4;

	cdns_mhdp_set_fw_clk(&imx_mhdp->mhdp, rate);

	/* skip fw loading if none is specified */
	if (!imx_mhdp->firmware_name)
		goto out;

	if (!imx_mhdp->fw) {
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_NOUEVENT,
						imx_mhdp->firmware_name,
						imx_mhdp->mhdp.dev, GFP_KERNEL,
						imx_mhdp,
						cdns_mhdp_firmware_load_cont);
		if (ret < 0) {
			DRM_ERROR("failed to load firmware\n");
			return -ENOENT;
		}
	} else {
		iram = imx_mhdp->fw->data + FW_IRAM_OFFSET;
		dram = iram + FW_IRAM_SIZE;

		cdns_mhdp_firmware_write_section(imx_mhdp, iram, FW_IRAM_SIZE, ADDR_IMEM);
		cdns_mhdp_firmware_write_section(imx_mhdp, dram, FW_DRAM_SIZE, ADDR_DMEM);
	}

out:
	/* un-reset ucpu */
	cdns_mhdp_bus_write(0, &imx_mhdp->mhdp, APB_CTRL);
	DRM_INFO("Started firmware!\n");

	return 0;
}

int cdns_mhdp_firmware_init_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);
	int ret;

	/* load firmware */
	ret = cdns_mhdp_firmware_load(imx_mhdp);
	if (ret)
		return ret;

	ret = cdns_mhdp_check_alive(&imx_mhdp->mhdp);
	if (ret == false) {
		DRM_ERROR("NO HDMI FW running\n");
		return -ENXIO;
	}

	/* turn on IP activity */
	cdns_mhdp_set_firmware_active(&imx_mhdp->mhdp, 1);

	if (mhdp->is_dp) {
		ret = cdns_mhdp_set_maximum_defer_retry(mhdp,
							mhdp->i2c_over_aux_retries);
		if (!ret)
			DRM_INFO("set maximum defer retry to %d",
				 mhdp->i2c_over_aux_retries);
	}

	DRM_INFO("HDP FW Version - ver %d verlib %d\n",
			cdns_mhdp_bus_read(mhdp, VER_L) + (cdns_mhdp_bus_read(mhdp, VER_H) << 8),
			cdns_mhdp_bus_read(mhdp, VER_LIB_H_ADDR) + (cdns_mhdp_bus_read(mhdp, VER_LIB_H_ADDR) << 8));

	return 0;
}

int cdns_mhdp_suspend_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);

	imx8qm_pixel_clk_disable(imx_mhdp);

	return 0;
}

int cdns_mhdp_resume_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct imx_mhdp_device *imx_mhdp =
				container_of(mhdp, struct imx_mhdp_device, mhdp);

	imx8qm_pixel_clk_enable(imx_mhdp);

	return cdns_mhdp_firmware_init_imx8qm(mhdp);
}
