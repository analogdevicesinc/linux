// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Brian Masney <masneyb@onstation.org>
 *
 * Based on MSM bus code from downstream MSM kernel sources.
 * Copyright (c) 2012-2013 The Linux Foundation. All rights reserved.
 *
 * Based on qcs404.c
 * Copyright (C) 2019 Linaro Ltd
 *
 * Here's a rough representation that shows the various buses that form the
 * Network On Chip (NOC) for the msm8974:
 *
 *                         Multimedia Subsystem (MMSS)
 *         |----------+-----------------------------------+-----------|
 *                    |                                   |
 *                    |                                   |
 *        Config      |                     Bus Interface | Memory Controller
 *       |------------+-+-----------|        |------------+-+-----------|
 *                      |                                   |
 *                      |                                   |
 *                      |             System                |
 *     |--------------+-+---------------------------------+-+-------------|
 *                    |                                   |
 *                    |                                   |
 *        Peripheral  |                           On Chip | Memory (OCMEM)
 *       |------------+-------------|        |------------+-------------|
 */

#include <dt-bindings/interconnect/qcom,msm8974.h>

#include <linux/args.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "icc-rpm.h"

enum {
	MSM8974_BIMC_MAS_AMPSS_M0 = 1,
	MSM8974_BIMC_MAS_AMPSS_M1,
	MSM8974_BIMC_MAS_MSS_PROC,
	MSM8974_BIMC_TO_MNOC,
	MSM8974_BIMC_TO_SNOC,
	MSM8974_BIMC_SLV_EBI_CH0,
	MSM8974_BIMC_SLV_AMPSS_L2,
	MSM8974_CNOC_MAS_RPM_INST,
	MSM8974_CNOC_MAS_RPM_DATA,
	MSM8974_CNOC_MAS_RPM_SYS,
	MSM8974_CNOC_MAS_DEHR,
	MSM8974_CNOC_MAS_QDSS_DAP,
	MSM8974_CNOC_MAS_SPDM,
	MSM8974_CNOC_MAS_TIC,
	MSM8974_CNOC_SLV_CLK_CTL,
	MSM8974_CNOC_SLV_CNOC_MSS,
	MSM8974_CNOC_SLV_SECURITY,
	MSM8974_CNOC_SLV_TCSR,
	MSM8974_CNOC_SLV_TLMM,
	MSM8974_CNOC_SLV_CRYPTO_0_CFG,
	MSM8974_CNOC_SLV_CRYPTO_1_CFG,
	MSM8974_CNOC_SLV_IMEM_CFG,
	MSM8974_CNOC_SLV_MESSAGE_RAM,
	MSM8974_CNOC_SLV_BIMC_CFG,
	MSM8974_CNOC_SLV_BOOT_ROM,
	MSM8974_CNOC_SLV_PMIC_ARB,
	MSM8974_CNOC_SLV_SPDM_WRAPPER,
	MSM8974_CNOC_SLV_DEHR_CFG,
	MSM8974_CNOC_SLV_MPM,
	MSM8974_CNOC_SLV_QDSS_CFG,
	MSM8974_CNOC_SLV_RBCPR_CFG,
	MSM8974_CNOC_SLV_RBCPR_QDSS_APU_CFG,
	MSM8974_CNOC_TO_SNOC,
	MSM8974_CNOC_SLV_CNOC_ONOC_CFG,
	MSM8974_CNOC_SLV_CNOC_MNOC_MMSS_CFG,
	MSM8974_CNOC_SLV_CNOC_MNOC_CFG,
	MSM8974_CNOC_SLV_PNOC_CFG,
	MSM8974_CNOC_SLV_SNOC_MPU_CFG,
	MSM8974_CNOC_SLV_SNOC_CFG,
	MSM8974_CNOC_SLV_EBI1_DLL_CFG,
	MSM8974_CNOC_SLV_PHY_APU_CFG,
	MSM8974_CNOC_SLV_EBI1_PHY_CFG,
	MSM8974_CNOC_SLV_RPM,
	MSM8974_CNOC_SLV_SERVICE_CNOC,
	MSM8974_MNOC_MAS_GRAPHICS_3D,
	MSM8974_MNOC_MAS_JPEG,
	MSM8974_MNOC_MAS_MDP_PORT0,
	MSM8974_MNOC_MAS_VIDEO_P0,
	MSM8974_MNOC_MAS_VIDEO_P1,
	MSM8974_MNOC_MAS_VFE,
	MSM8974_MNOC_TO_CNOC,
	MSM8974_MNOC_TO_BIMC,
	MSM8974_MNOC_SLV_CAMERA_CFG,
	MSM8974_MNOC_SLV_DISPLAY_CFG,
	MSM8974_MNOC_SLV_OCMEM_CFG,
	MSM8974_MNOC_SLV_CPR_CFG,
	MSM8974_MNOC_SLV_CPR_XPU_CFG,
	MSM8974_MNOC_SLV_MISC_CFG,
	MSM8974_MNOC_SLV_MISC_XPU_CFG,
	MSM8974_MNOC_SLV_VENUS_CFG,
	MSM8974_MNOC_SLV_GRAPHICS_3D_CFG,
	MSM8974_MNOC_SLV_MMSS_CLK_CFG,
	MSM8974_MNOC_SLV_MMSS_CLK_XPU_CFG,
	MSM8974_MNOC_SLV_MNOC_MPU_CFG,
	MSM8974_MNOC_SLV_ONOC_MPU_CFG,
	MSM8974_MNOC_SLV_SERVICE_MNOC,
	MSM8974_OCMEM_NOC_TO_OCMEM_VNOC,
	MSM8974_OCMEM_MAS_JPEG_OCMEM,
	MSM8974_OCMEM_MAS_MDP_OCMEM,
	MSM8974_OCMEM_MAS_VIDEO_P0_OCMEM,
	MSM8974_OCMEM_MAS_VIDEO_P1_OCMEM,
	MSM8974_OCMEM_MAS_VFE_OCMEM,
	MSM8974_OCMEM_MAS_CNOC_ONOC_CFG,
	MSM8974_OCMEM_SLV_SERVICE_ONOC,
	MSM8974_OCMEM_VNOC_TO_SNOC,
	MSM8974_OCMEM_VNOC_TO_OCMEM_NOC,
	MSM8974_OCMEM_VNOC_MAS_GFX3D,
	MSM8974_OCMEM_SLV_OCMEM,
	MSM8974_PNOC_MAS_PNOC_CFG,
	MSM8974_PNOC_MAS_SDCC_1,
	MSM8974_PNOC_MAS_SDCC_3,
	MSM8974_PNOC_MAS_SDCC_4,
	MSM8974_PNOC_MAS_SDCC_2,
	MSM8974_PNOC_MAS_TSIF,
	MSM8974_PNOC_MAS_BAM_DMA,
	MSM8974_PNOC_MAS_BLSP_2,
	MSM8974_PNOC_MAS_USB_HSIC,
	MSM8974_PNOC_MAS_BLSP_1,
	MSM8974_PNOC_MAS_USB_HS,
	MSM8974_PNOC_TO_SNOC,
	MSM8974_PNOC_SLV_SDCC_1,
	MSM8974_PNOC_SLV_SDCC_3,
	MSM8974_PNOC_SLV_SDCC_2,
	MSM8974_PNOC_SLV_SDCC_4,
	MSM8974_PNOC_SLV_TSIF,
	MSM8974_PNOC_SLV_BAM_DMA,
	MSM8974_PNOC_SLV_BLSP_2,
	MSM8974_PNOC_SLV_USB_HSIC,
	MSM8974_PNOC_SLV_BLSP_1,
	MSM8974_PNOC_SLV_USB_HS,
	MSM8974_PNOC_SLV_PDM,
	MSM8974_PNOC_SLV_PERIPH_APU_CFG,
	MSM8974_PNOC_SLV_PNOC_MPU_CFG,
	MSM8974_PNOC_SLV_PRNG,
	MSM8974_PNOC_SLV_SERVICE_PNOC,
	MSM8974_SNOC_MAS_LPASS_AHB,
	MSM8974_SNOC_MAS_QDSS_BAM,
	MSM8974_SNOC_MAS_SNOC_CFG,
	MSM8974_SNOC_TO_BIMC,
	MSM8974_SNOC_TO_CNOC,
	MSM8974_SNOC_TO_PNOC,
	MSM8974_SNOC_TO_OCMEM_VNOC,
	MSM8974_SNOC_MAS_CRYPTO_CORE0,
	MSM8974_SNOC_MAS_CRYPTO_CORE1,
	MSM8974_SNOC_MAS_LPASS_PROC,
	MSM8974_SNOC_MAS_MSS,
	MSM8974_SNOC_MAS_MSS_NAV,
	MSM8974_SNOC_MAS_OCMEM_DMA,
	MSM8974_SNOC_MAS_WCSS,
	MSM8974_SNOC_MAS_QDSS_ETR,
	MSM8974_SNOC_MAS_USB3,
	MSM8974_SNOC_SLV_AMPSS,
	MSM8974_SNOC_SLV_LPASS,
	MSM8974_SNOC_SLV_USB3,
	MSM8974_SNOC_SLV_WCSS,
	MSM8974_SNOC_SLV_OCIMEM,
	MSM8974_SNOC_SLV_SNOC_OCMEM,
	MSM8974_SNOC_SLV_SERVICE_SNOC,
	MSM8974_SNOC_SLV_QDSS_STM,
};

static int msm8974_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;

	return 0;
};

static struct qcom_icc_node mas_ampss_m0 = {
	.name = "mas_ampss_m0",
	.id = MSM8974_BIMC_MAS_AMPSS_M0,
	.buswidth = 8,
	.mas_rpm_id = 0,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_ampss_m1 = {
	.name = "mas_ampss_m1",
	.id = MSM8974_BIMC_MAS_AMPSS_M1,
	.buswidth = 8,
	.mas_rpm_id = 0,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_mss_proc = {
	.name = "mas_mss_proc",
	.id = MSM8974_BIMC_MAS_MSS_PROC,
	.buswidth = 8,
	.mas_rpm_id = 1,
	.slv_rpm_id = -1,
};

static const u16 bimc_to_mnoc_links[] = {
	MSM8974_BIMC_SLV_EBI_CH0
};

static struct qcom_icc_node bimc_to_mnoc = {
	.name = "bimc_to_mnoc",
	.id = MSM8974_BIMC_TO_MNOC,
	.buswidth = 8,
	.mas_rpm_id = 2,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(bimc_to_mnoc_links),
	.links = bimc_to_mnoc_links,
};

static const u16 bimc_to_snoc_links[] = {
	MSM8974_SNOC_TO_BIMC,
	MSM8974_BIMC_SLV_EBI_CH0,
	MSM8974_BIMC_MAS_AMPSS_M0
};

static struct qcom_icc_node bimc_to_snoc = {
	.name = "bimc_to_snoc",
	.id = MSM8974_BIMC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 3,
	.slv_rpm_id = 2,
	.num_links = ARRAY_SIZE(bimc_to_snoc_links),
	.links = bimc_to_snoc_links,
};

static struct qcom_icc_node slv_ebi_ch0 = {
	.name = "slv_ebi_ch0",
	.id = MSM8974_BIMC_SLV_EBI_CH0,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 0,
};

static struct qcom_icc_node slv_ampss_l2 = {
	.name = "slv_ampss_l2",
	.id = MSM8974_BIMC_SLV_AMPSS_L2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 1,
};

static struct qcom_icc_node * const msm8974_bimc_nodes[] = {
	[BIMC_MAS_AMPSS_M0] = &mas_ampss_m0,
	[BIMC_MAS_AMPSS_M1] = &mas_ampss_m1,
	[BIMC_MAS_MSS_PROC] = &mas_mss_proc,
	[BIMC_TO_MNOC] = &bimc_to_mnoc,
	[BIMC_TO_SNOC] = &bimc_to_snoc,
	[BIMC_SLV_EBI_CH0] = &slv_ebi_ch0,
	[BIMC_SLV_AMPSS_L2] = &slv_ampss_l2,
};

static const struct qcom_icc_desc msm8974_bimc = {
	.nodes = msm8974_bimc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_bimc_nodes),
	.bus_clk_desc = &bimc_clk,
	.get_bw = msm8974_get_bw,
	.ignore_enxio = true,
};

static struct qcom_icc_node mas_rpm_inst = {
	.name = "mas_rpm_inst",
	.id = MSM8974_CNOC_MAS_RPM_INST,
	.buswidth = 8,
	.mas_rpm_id = 45,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_rpm_data = {
	.name = "mas_rpm_data",
	.id = MSM8974_CNOC_MAS_RPM_DATA,
	.buswidth = 8,
	.mas_rpm_id = 46,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_rpm_sys = {
	.name = "mas_rpm_sys",
	.id = MSM8974_CNOC_MAS_RPM_SYS,
	.buswidth = 8,
	.mas_rpm_id = 47,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_dehr = {
	.name = "mas_dehr",
	.id = MSM8974_CNOC_MAS_DEHR,
	.buswidth = 8,
	.mas_rpm_id = 48,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_qdss_dap = {
	.name = "mas_qdss_dap",
	.id = MSM8974_CNOC_MAS_QDSS_DAP,
	.buswidth = 8,
	.mas_rpm_id = 49,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_spdm = {
	.name = "mas_spdm",
	.id = MSM8974_CNOC_MAS_SPDM,
	.buswidth = 8,
	.mas_rpm_id = 50,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_tic = {
	.name = "mas_tic",
	.id = MSM8974_CNOC_MAS_TIC,
	.buswidth = 8,
	.mas_rpm_id = 51,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node slv_clk_ctl = {
	.name = "slv_clk_ctl",
	.id = MSM8974_CNOC_SLV_CLK_CTL,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 47,
};

static struct qcom_icc_node slv_cnoc_mss = {
	.name = "slv_cnoc_mss",
	.id = MSM8974_CNOC_SLV_CNOC_MSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 48,
};

static struct qcom_icc_node slv_security = {
	.name = "slv_security",
	.id = MSM8974_CNOC_SLV_SECURITY,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 49,
};

static struct qcom_icc_node slv_tcsr = {
	.name = "slv_tcsr",
	.id = MSM8974_CNOC_SLV_TCSR,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 50,
};

static struct qcom_icc_node slv_tlmm = {
	.name = "slv_tlmm",
	.id = MSM8974_CNOC_SLV_TLMM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 51,
};

static struct qcom_icc_node slv_crypto_0_cfg = {
	.name = "slv_crypto_0_cfg",
	.id = MSM8974_CNOC_SLV_CRYPTO_0_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 52,
};

static struct qcom_icc_node slv_crypto_1_cfg = {
	.name = "slv_crypto_1_cfg",
	.id = MSM8974_CNOC_SLV_CRYPTO_1_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 53,
};

static struct qcom_icc_node slv_imem_cfg = {
	.name = "slv_imem_cfg",
	.id = MSM8974_CNOC_SLV_IMEM_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 54,
};

static struct qcom_icc_node slv_message_ram = {
	.name = "slv_message_ram",
	.id = MSM8974_CNOC_SLV_MESSAGE_RAM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 55,
};

static struct qcom_icc_node slv_bimc_cfg = {
	.name = "slv_bimc_cfg",
	.id = MSM8974_CNOC_SLV_BIMC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 56,
};

static struct qcom_icc_node slv_boot_rom = {
	.name = "slv_boot_rom",
	.id = MSM8974_CNOC_SLV_BOOT_ROM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 57,
};

static struct qcom_icc_node slv_pmic_arb = {
	.name = "slv_pmic_arb",
	.id = MSM8974_CNOC_SLV_PMIC_ARB,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 59,
};

static struct qcom_icc_node slv_spdm_wrapper = {
	.name = "slv_spdm_wrapper",
	.id = MSM8974_CNOC_SLV_SPDM_WRAPPER,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 60,
};

static struct qcom_icc_node slv_dehr_cfg = {
	.name = "slv_dehr_cfg",
	.id = MSM8974_CNOC_SLV_DEHR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 61,
};

static struct qcom_icc_node slv_mpm = {
	.name = "slv_mpm",
	.id = MSM8974_CNOC_SLV_MPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 62,
};

static struct qcom_icc_node slv_qdss_cfg = {
	.name = "slv_qdss_cfg",
	.id = MSM8974_CNOC_SLV_QDSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 63,
};

static struct qcom_icc_node slv_rbcpr_cfg = {
	.name = "slv_rbcpr_cfg",
	.id = MSM8974_CNOC_SLV_RBCPR_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 64,
};

static struct qcom_icc_node slv_rbcpr_qdss_apu_cfg = {
	.name = "slv_rbcpr_qdss_apu_cfg",
	.id = MSM8974_CNOC_SLV_RBCPR_QDSS_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 65,
};

static struct qcom_icc_node cnoc_to_snoc = {
	.name = "cnoc_to_snoc",
	.id = MSM8974_CNOC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 52,
	.slv_rpm_id = 75,
};

static struct qcom_icc_node slv_cnoc_onoc_cfg = {
	.name = "slv_cnoc_onoc_cfg",
	.id = MSM8974_CNOC_SLV_CNOC_ONOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 68,
};

static struct qcom_icc_node slv_cnoc_mnoc_mmss_cfg = {
	.name = "slv_cnoc_mnoc_mmss_cfg",
	.id = MSM8974_CNOC_SLV_CNOC_MNOC_MMSS_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 58,
};

static struct qcom_icc_node slv_cnoc_mnoc_cfg = {
	.name = "slv_cnoc_mnoc_cfg",
	.id = MSM8974_CNOC_SLV_CNOC_MNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 66,
};

static struct qcom_icc_node slv_pnoc_cfg = {
	.name = "slv_pnoc_cfg",
	.id = MSM8974_CNOC_SLV_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 69,
};

static struct qcom_icc_node slv_snoc_mpu_cfg = {
	.name = "slv_snoc_mpu_cfg",
	.id = MSM8974_CNOC_SLV_SNOC_MPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 67,
};

static struct qcom_icc_node slv_snoc_cfg = {
	.name = "slv_snoc_cfg",
	.id = MSM8974_CNOC_SLV_SNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 70,
};

static struct qcom_icc_node slv_ebi1_dll_cfg = {
	.name = "slv_ebi1_dll_cfg",
	.id = MSM8974_CNOC_SLV_EBI1_DLL_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 71,
};

static struct qcom_icc_node slv_phy_apu_cfg = {
	.name = "slv_phy_apu_cfg",
	.id = MSM8974_CNOC_SLV_PHY_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 72,
};

static struct qcom_icc_node slv_ebi1_phy_cfg = {
	.name = "slv_ebi1_phy_cfg",
	.id = MSM8974_CNOC_SLV_EBI1_PHY_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 73,
};

static struct qcom_icc_node slv_rpm = {
	.name = "slv_rpm",
	.id = MSM8974_CNOC_SLV_RPM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 74,
};

static struct qcom_icc_node slv_service_cnoc = {
	.name = "slv_service_cnoc",
	.id = MSM8974_CNOC_SLV_SERVICE_CNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 76,
};

static struct qcom_icc_node * const msm8974_cnoc_nodes[] = {
	[CNOC_MAS_RPM_INST] = &mas_rpm_inst,
	[CNOC_MAS_RPM_DATA] = &mas_rpm_data,
	[CNOC_MAS_RPM_SYS] = &mas_rpm_sys,
	[CNOC_MAS_DEHR] = &mas_dehr,
	[CNOC_MAS_QDSS_DAP] = &mas_qdss_dap,
	[CNOC_MAS_SPDM] = &mas_spdm,
	[CNOC_MAS_TIC] = &mas_tic,
	[CNOC_SLV_CLK_CTL] = &slv_clk_ctl,
	[CNOC_SLV_CNOC_MSS] = &slv_cnoc_mss,
	[CNOC_SLV_SECURITY] = &slv_security,
	[CNOC_SLV_TCSR] = &slv_tcsr,
	[CNOC_SLV_TLMM] = &slv_tlmm,
	[CNOC_SLV_CRYPTO_0_CFG] = &slv_crypto_0_cfg,
	[CNOC_SLV_CRYPTO_1_CFG] = &slv_crypto_1_cfg,
	[CNOC_SLV_IMEM_CFG] = &slv_imem_cfg,
	[CNOC_SLV_MESSAGE_RAM] = &slv_message_ram,
	[CNOC_SLV_BIMC_CFG] = &slv_bimc_cfg,
	[CNOC_SLV_BOOT_ROM] = &slv_boot_rom,
	[CNOC_SLV_PMIC_ARB] = &slv_pmic_arb,
	[CNOC_SLV_SPDM_WRAPPER] = &slv_spdm_wrapper,
	[CNOC_SLV_DEHR_CFG] = &slv_dehr_cfg,
	[CNOC_SLV_MPM] = &slv_mpm,
	[CNOC_SLV_QDSS_CFG] = &slv_qdss_cfg,
	[CNOC_SLV_RBCPR_CFG] = &slv_rbcpr_cfg,
	[CNOC_SLV_RBCPR_QDSS_APU_CFG] = &slv_rbcpr_qdss_apu_cfg,
	[CNOC_TO_SNOC] = &cnoc_to_snoc,
	[CNOC_SLV_CNOC_ONOC_CFG] = &slv_cnoc_onoc_cfg,
	[CNOC_SLV_CNOC_MNOC_MMSS_CFG] = &slv_cnoc_mnoc_mmss_cfg,
	[CNOC_SLV_CNOC_MNOC_CFG] = &slv_cnoc_mnoc_cfg,
	[CNOC_SLV_PNOC_CFG] = &slv_pnoc_cfg,
	[CNOC_SLV_SNOC_MPU_CFG] = &slv_snoc_mpu_cfg,
	[CNOC_SLV_SNOC_CFG] = &slv_snoc_cfg,
	[CNOC_SLV_EBI1_DLL_CFG] = &slv_ebi1_dll_cfg,
	[CNOC_SLV_PHY_APU_CFG] = &slv_phy_apu_cfg,
	[CNOC_SLV_EBI1_PHY_CFG] = &slv_ebi1_phy_cfg,
	[CNOC_SLV_RPM] = &slv_rpm,
	[CNOC_SLV_SERVICE_CNOC] = &slv_service_cnoc,
};

static const struct qcom_icc_desc msm8974_cnoc = {
	.nodes = msm8974_cnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_cnoc_nodes),
	.bus_clk_desc = &bus_2_clk,
	.get_bw = msm8974_get_bw,
	.ignore_enxio = true,
};

static const u16 mas_graphics_3d_links[] = {
	MSM8974_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_graphics_3d = {
	.name = "mas_graphics_3d",
	.id = MSM8974_MNOC_MAS_GRAPHICS_3D,
	.buswidth = 16,
	.mas_rpm_id = 6,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_graphics_3d_links),
	.links = mas_graphics_3d_links,
};

static const u16 mas_jpeg_links[] = {
	MSM8974_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_jpeg = {
	.name = "mas_jpeg",
	.id = MSM8974_MNOC_MAS_JPEG,
	.buswidth = 16,
	.mas_rpm_id = 7,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_jpeg_links),
	.links = mas_jpeg_links,
};

static const u16 mas_mdp_port0_links[] = {
	MSM8974_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_mdp_port0 = {
	.name = "mas_mdp_port0",
	.id = MSM8974_MNOC_MAS_MDP_PORT0,
	.buswidth = 16,
	.mas_rpm_id = 8,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_mdp_port0_links),
	.links = mas_mdp_port0_links,
};

static struct qcom_icc_node mas_video_p0 = {
	.name = "mas_video_p0",
	.id = MSM8974_MNOC_MAS_VIDEO_P0,
	.buswidth = 16,
	.mas_rpm_id = 9,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_video_p1 = {
	.name = "mas_video_p1",
	.id = MSM8974_MNOC_MAS_VIDEO_P1,
	.buswidth = 16,
	.mas_rpm_id = 10,
	.slv_rpm_id = -1,
};

static const u16 mas_vfe_links[] = {
	MSM8974_MNOC_TO_BIMC
};

static struct qcom_icc_node mas_vfe = {
	.name = "mas_vfe",
	.id = MSM8974_MNOC_MAS_VFE,
	.buswidth = 16,
	.mas_rpm_id = 11,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_vfe_links),
	.links = mas_vfe_links,
};

static struct qcom_icc_node mnoc_to_cnoc = {
	.name = "mnoc_to_cnoc",
	.id = MSM8974_MNOC_TO_CNOC,
	.buswidth = 16,
	.mas_rpm_id = 4,
	.slv_rpm_id = -1,
};

static const u16 mnoc_to_bimc_links[] = {
	MSM8974_BIMC_TO_MNOC
};

static struct qcom_icc_node mnoc_to_bimc = {
	.name = "mnoc_to_bimc",
	.id = MSM8974_MNOC_TO_BIMC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 16,
	.num_links = ARRAY_SIZE(mnoc_to_bimc_links),
	.links = mnoc_to_bimc_links,
};

static struct qcom_icc_node slv_camera_cfg = {
	.name = "slv_camera_cfg",
	.id = MSM8974_MNOC_SLV_CAMERA_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 3,
};

static struct qcom_icc_node slv_display_cfg = {
	.name = "slv_display_cfg",
	.id = MSM8974_MNOC_SLV_DISPLAY_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 4,
};

static struct qcom_icc_node slv_ocmem_cfg = {
	.name = "slv_ocmem_cfg",
	.id = MSM8974_MNOC_SLV_OCMEM_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 5,
};

static struct qcom_icc_node slv_cpr_cfg = {
	.name = "slv_cpr_cfg",
	.id = MSM8974_MNOC_SLV_CPR_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 6,
};

static struct qcom_icc_node slv_cpr_xpu_cfg = {
	.name = "slv_cpr_xpu_cfg",
	.id = MSM8974_MNOC_SLV_CPR_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 7,
};

static struct qcom_icc_node slv_misc_cfg = {
	.name = "slv_misc_cfg",
	.id = MSM8974_MNOC_SLV_MISC_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 8,
};

static struct qcom_icc_node slv_misc_xpu_cfg = {
	.name = "slv_misc_xpu_cfg",
	.id = MSM8974_MNOC_SLV_MISC_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 9,
};

static struct qcom_icc_node slv_venus_cfg = {
	.name = "slv_venus_cfg",
	.id = MSM8974_MNOC_SLV_VENUS_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 10,
};

static struct qcom_icc_node slv_graphics_3d_cfg = {
	.name = "slv_graphics_3d_cfg",
	.id = MSM8974_MNOC_SLV_GRAPHICS_3D_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 11,
};

static struct qcom_icc_node slv_mmss_clk_cfg = {
	.name = "slv_mmss_clk_cfg",
	.id = MSM8974_MNOC_SLV_MMSS_CLK_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 12,
};

static struct qcom_icc_node slv_mmss_clk_xpu_cfg = {
	.name = "slv_mmss_clk_xpu_cfg",
	.id = MSM8974_MNOC_SLV_MMSS_CLK_XPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 13,
};

static struct qcom_icc_node slv_mnoc_mpu_cfg = {
	.name = "slv_mnoc_mpu_cfg",
	.id = MSM8974_MNOC_SLV_MNOC_MPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 14,
};

static struct qcom_icc_node slv_onoc_mpu_cfg = {
	.name = "slv_onoc_mpu_cfg",
	.id = MSM8974_MNOC_SLV_ONOC_MPU_CFG,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 15,
};

static struct qcom_icc_node slv_service_mnoc = {
	.name = "slv_service_mnoc",
	.id = MSM8974_MNOC_SLV_SERVICE_MNOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 17,
};

static struct qcom_icc_node * const msm8974_mnoc_nodes[] = {
	[MNOC_MAS_GRAPHICS_3D] = &mas_graphics_3d,
	[MNOC_MAS_JPEG] = &mas_jpeg,
	[MNOC_MAS_MDP_PORT0] = &mas_mdp_port0,
	[MNOC_MAS_VIDEO_P0] = &mas_video_p0,
	[MNOC_MAS_VIDEO_P1] = &mas_video_p1,
	[MNOC_MAS_VFE] = &mas_vfe,
	[MNOC_TO_CNOC] = &mnoc_to_cnoc,
	[MNOC_TO_BIMC] = &mnoc_to_bimc,
	[MNOC_SLV_CAMERA_CFG] = &slv_camera_cfg,
	[MNOC_SLV_DISPLAY_CFG] = &slv_display_cfg,
	[MNOC_SLV_OCMEM_CFG] = &slv_ocmem_cfg,
	[MNOC_SLV_CPR_CFG] = &slv_cpr_cfg,
	[MNOC_SLV_CPR_XPU_CFG] = &slv_cpr_xpu_cfg,
	[MNOC_SLV_MISC_CFG] = &slv_misc_cfg,
	[MNOC_SLV_MISC_XPU_CFG] = &slv_misc_xpu_cfg,
	[MNOC_SLV_VENUS_CFG] = &slv_venus_cfg,
	[MNOC_SLV_GRAPHICS_3D_CFG] = &slv_graphics_3d_cfg,
	[MNOC_SLV_MMSS_CLK_CFG] = &slv_mmss_clk_cfg,
	[MNOC_SLV_MMSS_CLK_XPU_CFG] = &slv_mmss_clk_xpu_cfg,
	[MNOC_SLV_MNOC_MPU_CFG] = &slv_mnoc_mpu_cfg,
	[MNOC_SLV_ONOC_MPU_CFG] = &slv_onoc_mpu_cfg,
	[MNOC_SLV_SERVICE_MNOC] = &slv_service_mnoc,
};

static const struct qcom_icc_desc msm8974_mnoc = {
	.nodes = msm8974_mnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_mnoc_nodes),
	.get_bw = msm8974_get_bw,
	.ignore_enxio = true,
};

static const u16 ocmem_noc_to_ocmem_vnoc_links[] = {
	MSM8974_OCMEM_SLV_OCMEM
};

static struct qcom_icc_node ocmem_noc_to_ocmem_vnoc = {
	.name = "ocmem_noc_to_ocmem_vnoc",
	.id = MSM8974_OCMEM_NOC_TO_OCMEM_VNOC,
	.buswidth = 16,
	.mas_rpm_id = 54,
	.slv_rpm_id = 78,
	.num_links = ARRAY_SIZE(ocmem_noc_to_ocmem_vnoc_links),
	.links = ocmem_noc_to_ocmem_vnoc_links,
};

static struct qcom_icc_node mas_jpeg_ocmem = {
	.name = "mas_jpeg_ocmem",
	.id = MSM8974_OCMEM_MAS_JPEG_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = 13,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_mdp_ocmem = {
	.name = "mas_mdp_ocmem",
	.id = MSM8974_OCMEM_MAS_MDP_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = 14,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_video_p0_ocmem = {
	.name = "mas_video_p0_ocmem",
	.id = MSM8974_OCMEM_MAS_VIDEO_P0_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = 15,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_video_p1_ocmem = {
	.name = "mas_video_p1_ocmem",
	.id = MSM8974_OCMEM_MAS_VIDEO_P1_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = 16,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_vfe_ocmem = {
	.name = "mas_vfe_ocmem",
	.id = MSM8974_OCMEM_MAS_VFE_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = 17,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_cnoc_onoc_cfg = {
	.name = "mas_cnoc_onoc_cfg",
	.id = MSM8974_OCMEM_MAS_CNOC_ONOC_CFG,
	.buswidth = 16,
	.mas_rpm_id = 12,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node slv_service_onoc = {
	.name = "slv_service_onoc",
	.id = MSM8974_OCMEM_SLV_SERVICE_ONOC,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 19,
};

static struct qcom_icc_node slv_ocmem = {
	.name = "slv_ocmem",
	.id = MSM8974_OCMEM_SLV_OCMEM,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = 18,
};

/* Virtual NoC is needed for connection to OCMEM */
static const u16 ocmem_vnoc_to_onoc_links[] = {
	MSM8974_OCMEM_NOC_TO_OCMEM_VNOC
};

static struct qcom_icc_node ocmem_vnoc_to_onoc = {
	.name = "ocmem_vnoc_to_onoc",
	.id = MSM8974_OCMEM_VNOC_TO_OCMEM_NOC,
	.buswidth = 16,
	.mas_rpm_id = 56,
	.slv_rpm_id = 79,
	.num_links = ARRAY_SIZE(ocmem_vnoc_to_onoc_links),
	.links = ocmem_vnoc_to_onoc_links,
};

static struct qcom_icc_node ocmem_vnoc_to_snoc = {
	.name = "ocmem_vnoc_to_snoc",
	.id = MSM8974_OCMEM_VNOC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 57,
	.slv_rpm_id = 80,
};

static const u16 mas_v_ocmem_gfx3d_links[] = {
	MSM8974_OCMEM_VNOC_TO_OCMEM_NOC
};

static struct qcom_icc_node mas_v_ocmem_gfx3d = {
	.name = "mas_v_ocmem_gfx3d",
	.id = MSM8974_OCMEM_VNOC_MAS_GFX3D,
	.buswidth = 8,
	.mas_rpm_id = 55,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_v_ocmem_gfx3d_links),
	.links = mas_v_ocmem_gfx3d_links,
};


static struct qcom_icc_node * const msm8974_onoc_nodes[] = {
	[OCMEM_NOC_TO_OCMEM_VNOC] = &ocmem_noc_to_ocmem_vnoc,
	[OCMEM_MAS_JPEG_OCMEM] = &mas_jpeg_ocmem,
	[OCMEM_MAS_MDP_OCMEM] = &mas_mdp_ocmem,
	[OCMEM_MAS_VIDEO_P0_OCMEM] = &mas_video_p0_ocmem,
	[OCMEM_MAS_VIDEO_P1_OCMEM] = &mas_video_p1_ocmem,
	[OCMEM_MAS_VFE_OCMEM] = &mas_vfe_ocmem,
	[OCMEM_MAS_CNOC_ONOC_CFG] = &mas_cnoc_onoc_cfg,
	[OCMEM_SLV_SERVICE_ONOC] = &slv_service_onoc,
	[OCMEM_VNOC_TO_SNOC] = &ocmem_vnoc_to_snoc,
	[OCMEM_VNOC_TO_OCMEM_NOC] = &ocmem_vnoc_to_onoc,
	[OCMEM_VNOC_MAS_GFX3D] = &mas_v_ocmem_gfx3d,
	[OCMEM_SLV_OCMEM] = &slv_ocmem,
};

static const struct qcom_icc_desc msm8974_onoc = {
	.nodes = msm8974_onoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_onoc_nodes),
	.bus_clk_desc = &gpu_mem_2_clk,
	.get_bw = msm8974_get_bw,
	.ignore_enxio = true,
};

static struct qcom_icc_node mas_pnoc_cfg = {
	.name = "mas_pnoc_cfg",
	.id = MSM8974_PNOC_MAS_PNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = 43,
	.slv_rpm_id = -1,
};

static const u16 mas_sdcc_1_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_1 = {
	.name = "mas_sdcc_1",
	.id = MSM8974_PNOC_MAS_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = 33,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_1_links),
	.links = mas_sdcc_1_links,
};

static const u16 mas_sdcc_3_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_3 = {
	.name = "mas_sdcc_3",
	.id = MSM8974_PNOC_MAS_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = 34,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_3_links),
	.links = mas_sdcc_3_links,
};

static const u16 mas_sdcc_4_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_4 = {
	.name = "mas_sdcc_4",
	.id = MSM8974_PNOC_MAS_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = 36,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_4_links),
	.links = mas_sdcc_4_links,
};

static const u16 mas_sdcc_2_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_sdcc_2 = {
	.name = "mas_sdcc_2",
	.id = MSM8974_PNOC_MAS_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = 35,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_sdcc_2_links),
	.links = mas_sdcc_2_links,
};

static const u16 mas_tsif_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_tsif = {
	.name = "mas_tsif",
	.id = MSM8974_PNOC_MAS_TSIF,
	.buswidth = 8,
	.mas_rpm_id = 37,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_tsif_links),
	.links = mas_tsif_links,
};

static struct qcom_icc_node mas_bam_dma = {
	.name = "mas_bam_dma",
	.id = MSM8974_PNOC_MAS_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = 38,
	.slv_rpm_id = -1,
};

static const u16 mas_blsp_2_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_blsp_2 = {
	.name = "mas_blsp_2",
	.id = MSM8974_PNOC_MAS_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = 39,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_blsp_2_links),
	.links = mas_blsp_2_links,
};

static const u16 mas_usb_hsic_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_usb_hsic = {
	.name = "mas_usb_hsic",
	.id = MSM8974_PNOC_MAS_USB_HSIC,
	.buswidth = 8,
	.mas_rpm_id = 40,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_usb_hsic_links),
	.links = mas_usb_hsic_links,
};

static const u16 mas_blsp_1_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_blsp_1 = {
	.name = "mas_blsp_1",
	.id = MSM8974_PNOC_MAS_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = 41,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_blsp_1_links),
	.links = mas_blsp_1_links,
};

static const u16 mas_usb_hs_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node mas_usb_hs = {
	.name = "mas_usb_hs",
	.id = MSM8974_PNOC_MAS_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = 42,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_usb_hs_links),
	.links = mas_usb_hs_links,
};

static const u16 pnoc_to_snoc_links[] = {
	MSM8974_SNOC_TO_PNOC,
	MSM8974_PNOC_SLV_PRNG
};

static struct qcom_icc_node pnoc_to_snoc = {
	.name = "pnoc_to_snoc",
	.id = MSM8974_PNOC_TO_SNOC,
	.buswidth = 8,
	.mas_rpm_id = 44,
	.slv_rpm_id = 45,
	.num_links = ARRAY_SIZE(pnoc_to_snoc_links),
	.links = pnoc_to_snoc_links,
};

static struct qcom_icc_node slv_sdcc_1 = {
	.name = "slv_sdcc_1",
	.id = MSM8974_PNOC_SLV_SDCC_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 31,
};

static struct qcom_icc_node slv_sdcc_3 = {
	.name = "slv_sdcc_3",
	.id = MSM8974_PNOC_SLV_SDCC_3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 32,
};

static struct qcom_icc_node slv_sdcc_2 = {
	.name = "slv_sdcc_2",
	.id = MSM8974_PNOC_SLV_SDCC_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 33,
};

static struct qcom_icc_node slv_sdcc_4 = {
	.name = "slv_sdcc_4",
	.id = MSM8974_PNOC_SLV_SDCC_4,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 34,
};

static struct qcom_icc_node slv_tsif = {
	.name = "slv_tsif",
	.id = MSM8974_PNOC_SLV_TSIF,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 35,
};

static struct qcom_icc_node slv_bam_dma = {
	.name = "slv_bam_dma",
	.id = MSM8974_PNOC_SLV_BAM_DMA,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 36,
};

static struct qcom_icc_node slv_blsp_2 = {
	.name = "slv_blsp_2",
	.id = MSM8974_PNOC_SLV_BLSP_2,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 37,
};

static struct qcom_icc_node slv_usb_hsic = {
	.name = "slv_usb_hsic",
	.id = MSM8974_PNOC_SLV_USB_HSIC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 38,
};

static struct qcom_icc_node slv_blsp_1 = {
	.name = "slv_blsp_1",
	.id = MSM8974_PNOC_SLV_BLSP_1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 39,
};

static struct qcom_icc_node slv_usb_hs = {
	.name = "slv_usb_hs",
	.id = MSM8974_PNOC_SLV_USB_HS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 40,
};

static struct qcom_icc_node slv_pdm = {
	.name = "slv_pdm",
	.id = MSM8974_PNOC_SLV_PDM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 41,
};

static struct qcom_icc_node slv_periph_apu_cfg = {
	.name = "slv_periph_apu_cfg",
	.id = MSM8974_PNOC_SLV_PERIPH_APU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 42,
};

static struct qcom_icc_node slv_pnoc_mpu_cfg = {
	.name = "slv_pnoc_mpu_cfg",
	.id = MSM8974_PNOC_SLV_PNOC_MPU_CFG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 43,
};

static const u16 slv_prng_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node slv_prng = {
	.name = "slv_prng",
	.id = MSM8974_PNOC_SLV_PRNG,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 44,
	.num_links = ARRAY_SIZE(slv_prng_links),
	.links = slv_prng_links,
};

static struct qcom_icc_node slv_service_pnoc = {
	.name = "slv_service_pnoc",
	.id = MSM8974_PNOC_SLV_SERVICE_PNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 46,
};

static struct qcom_icc_node * const msm8974_pnoc_nodes[] = {
	[PNOC_MAS_PNOC_CFG] = &mas_pnoc_cfg,
	[PNOC_MAS_SDCC_1] = &mas_sdcc_1,
	[PNOC_MAS_SDCC_3] = &mas_sdcc_3,
	[PNOC_MAS_SDCC_4] = &mas_sdcc_4,
	[PNOC_MAS_SDCC_2] = &mas_sdcc_2,
	[PNOC_MAS_TSIF] = &mas_tsif,
	[PNOC_MAS_BAM_DMA] = &mas_bam_dma,
	[PNOC_MAS_BLSP_2] = &mas_blsp_2,
	[PNOC_MAS_USB_HSIC] = &mas_usb_hsic,
	[PNOC_MAS_BLSP_1] = &mas_blsp_1,
	[PNOC_MAS_USB_HS] = &mas_usb_hs,
	[PNOC_TO_SNOC] = &pnoc_to_snoc,
	[PNOC_SLV_SDCC_1] = &slv_sdcc_1,
	[PNOC_SLV_SDCC_3] = &slv_sdcc_3,
	[PNOC_SLV_SDCC_2] = &slv_sdcc_2,
	[PNOC_SLV_SDCC_4] = &slv_sdcc_4,
	[PNOC_SLV_TSIF] = &slv_tsif,
	[PNOC_SLV_BAM_DMA] = &slv_bam_dma,
	[PNOC_SLV_BLSP_2] = &slv_blsp_2,
	[PNOC_SLV_USB_HSIC] = &slv_usb_hsic,
	[PNOC_SLV_BLSP_1] = &slv_blsp_1,
	[PNOC_SLV_USB_HS] = &slv_usb_hs,
	[PNOC_SLV_PDM] = &slv_pdm,
	[PNOC_SLV_PERIPH_APU_CFG] = &slv_periph_apu_cfg,
	[PNOC_SLV_PNOC_MPU_CFG] = &slv_pnoc_mpu_cfg,
	[PNOC_SLV_PRNG] = &slv_prng,
	[PNOC_SLV_SERVICE_PNOC] = &slv_service_pnoc,
};

static const struct qcom_icc_desc msm8974_pnoc = {
	.nodes = msm8974_pnoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_pnoc_nodes),
	.bus_clk_desc = &bus_0_clk,
	.get_bw = msm8974_get_bw,
	.keep_alive = true,
	.ignore_enxio = true,
};

static struct qcom_icc_node mas_lpass_ahb = {
	.name = "mas_lpass_ahb",
	.id = MSM8974_SNOC_MAS_LPASS_AHB,
	.buswidth = 8,
	.mas_rpm_id = 18,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_qdss_bam = {
	.name = "mas_qdss_bam",
	.id = MSM8974_SNOC_MAS_QDSS_BAM,
	.buswidth = 8,
	.mas_rpm_id = 19,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_snoc_cfg = {
	.name = "mas_snoc_cfg",
	.id = MSM8974_SNOC_MAS_SNOC_CFG,
	.buswidth = 8,
	.mas_rpm_id = 20,
	.slv_rpm_id = -1,
};

static const u16 snoc_to_bimc_links[] = {
	MSM8974_BIMC_TO_SNOC
};

static struct qcom_icc_node snoc_to_bimc = {
	.name = "snoc_to_bimc",
	.id = MSM8974_SNOC_TO_BIMC,
	.buswidth = 8,
	.mas_rpm_id = 21,
	.slv_rpm_id = 24,
	.num_links = ARRAY_SIZE(snoc_to_bimc_links),
	.links = snoc_to_bimc_links,
};

static struct qcom_icc_node snoc_to_cnoc = {
	.name = "snoc_to_cnoc",
	.id = MSM8974_SNOC_TO_CNOC,
	.buswidth = 8,
	.mas_rpm_id = 22,
	.slv_rpm_id = 25,
};

static const u16 snoc_to_pnoc_links[] = {
	MSM8974_PNOC_TO_SNOC
};

static struct qcom_icc_node snoc_to_pnoc = {
	.name = "snoc_to_pnoc",
	.id = MSM8974_SNOC_TO_PNOC,
	.buswidth = 8,
	.mas_rpm_id = 29,
	.slv_rpm_id = 28,
	.num_links = ARRAY_SIZE(snoc_to_pnoc_links),
	.links = snoc_to_pnoc_links,
};

static const u16 snoc_to_ocmem_vnoc_links[] = {
	MSM8974_OCMEM_VNOC_TO_OCMEM_NOC
};

static struct qcom_icc_node snoc_to_ocmem_vnoc = {
	.name = "snoc_to_ocmem_vnoc",
	.id = MSM8974_SNOC_TO_OCMEM_VNOC,
	.buswidth = 8,
	.mas_rpm_id = 53,
	.slv_rpm_id = 77,
	.num_links = ARRAY_SIZE(snoc_to_ocmem_vnoc_links),
	.links = snoc_to_ocmem_vnoc_links,
};

static const u16 mas_crypto_core0_links[] = {
	MSM8974_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_crypto_core0 = {
	.name = "mas_crypto_core0",
	.id = MSM8974_SNOC_MAS_CRYPTO_CORE0,
	.buswidth = 8,
	.mas_rpm_id = 23,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_crypto_core0_links),
	.links = mas_crypto_core0_links,
};

static struct qcom_icc_node mas_crypto_core1 = {
	.name = "mas_crypto_core1",
	.id = MSM8974_SNOC_MAS_CRYPTO_CORE1,
	.buswidth = 8,
	.mas_rpm_id = 24,
	.slv_rpm_id = -1,
};

static const u16 mas_lpass_proc_links[] = {
	MSM8974_SNOC_TO_OCMEM_VNOC
};

static struct qcom_icc_node mas_lpass_proc = {
	.name = "mas_lpass_proc",
	.id = MSM8974_SNOC_MAS_LPASS_PROC,
	.buswidth = 8,
	.mas_rpm_id = 25,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_lpass_proc_links),
	.links = mas_lpass_proc_links,
};

static struct qcom_icc_node mas_mss = {
	.name = "mas_mss",
	.id = MSM8974_SNOC_MAS_MSS,
	.buswidth = 8,
	.mas_rpm_id = 26,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_mss_nav = {
	.name = "mas_mss_nav",
	.id = MSM8974_SNOC_MAS_MSS_NAV,
	.buswidth = 8,
	.mas_rpm_id = 27,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_ocmem_dma = {
	.name = "mas_ocmem_dma",
	.id = MSM8974_SNOC_MAS_OCMEM_DMA,
	.buswidth = 8,
	.mas_rpm_id = 28,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_wcss = {
	.name = "mas_wcss",
	.id = MSM8974_SNOC_MAS_WCSS,
	.buswidth = 8,
	.mas_rpm_id = 30,
	.slv_rpm_id = -1,
};

static struct qcom_icc_node mas_qdss_etr = {
	.name = "mas_qdss_etr",
	.id = MSM8974_SNOC_MAS_QDSS_ETR,
	.buswidth = 8,
	.mas_rpm_id = 31,
	.slv_rpm_id = -1,
};

static const u16 mas_usb3_links[] = {
	MSM8974_SNOC_TO_BIMC
};

static struct qcom_icc_node mas_usb3 = {
	.name = "mas_usb3",
	.id = MSM8974_SNOC_MAS_USB3,
	.buswidth = 8,
	.mas_rpm_id = 32,
	.slv_rpm_id = -1,
	.num_links = ARRAY_SIZE(mas_usb3_links),
	.links = mas_usb3_links,
};

static struct qcom_icc_node slv_ampss = {
	.name = "slv_ampss",
	.id = MSM8974_SNOC_SLV_AMPSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 20,
};

static struct qcom_icc_node slv_lpass = {
	.name = "slv_lpass",
	.id = MSM8974_SNOC_SLV_LPASS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 21,
};

static struct qcom_icc_node slv_usb3 = {
	.name = "slv_usb3",
	.id = MSM8974_SNOC_SLV_USB3,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 22,
};

static struct qcom_icc_node slv_wcss = {
	.name = "slv_wcss",
	.id = MSM8974_SNOC_SLV_WCSS,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 23,
};

static struct qcom_icc_node slv_ocimem = {
	.name = "slv_ocimem",
	.id = MSM8974_SNOC_SLV_OCIMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 26,
};

static struct qcom_icc_node slv_snoc_ocmem = {
	.name = "slv_snoc_ocmem",
	.id = MSM8974_SNOC_SLV_SNOC_OCMEM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 27,
};

static struct qcom_icc_node slv_service_snoc = {
	.name = "slv_service_snoc",
	.id = MSM8974_SNOC_SLV_SERVICE_SNOC,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 29,
};

static struct qcom_icc_node slv_qdss_stm = {
	.name = "slv_qdss_stm",
	.id = MSM8974_SNOC_SLV_QDSS_STM,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = 30,
};

static struct qcom_icc_node * const msm8974_snoc_nodes[] = {
	[SNOC_MAS_LPASS_AHB] = &mas_lpass_ahb,
	[SNOC_MAS_QDSS_BAM] = &mas_qdss_bam,
	[SNOC_MAS_SNOC_CFG] = &mas_snoc_cfg,
	[SNOC_TO_BIMC] = &snoc_to_bimc,
	[SNOC_TO_CNOC] = &snoc_to_cnoc,
	[SNOC_TO_PNOC] = &snoc_to_pnoc,
	[SNOC_TO_OCMEM_VNOC] = &snoc_to_ocmem_vnoc,
	[SNOC_MAS_CRYPTO_CORE0] = &mas_crypto_core0,
	[SNOC_MAS_CRYPTO_CORE1] = &mas_crypto_core1,
	[SNOC_MAS_LPASS_PROC] = &mas_lpass_proc,
	[SNOC_MAS_MSS] = &mas_mss,
	[SNOC_MAS_MSS_NAV] = &mas_mss_nav,
	[SNOC_MAS_OCMEM_DMA] = &mas_ocmem_dma,
	[SNOC_MAS_WCSS] = &mas_wcss,
	[SNOC_MAS_QDSS_ETR] = &mas_qdss_etr,
	[SNOC_MAS_USB3] = &mas_usb3,
	[SNOC_SLV_AMPSS] = &slv_ampss,
	[SNOC_SLV_LPASS] = &slv_lpass,
	[SNOC_SLV_USB3] = &slv_usb3,
	[SNOC_SLV_WCSS] = &slv_wcss,
	[SNOC_SLV_OCIMEM] = &slv_ocimem,
	[SNOC_SLV_SNOC_OCMEM] = &slv_snoc_ocmem,
	[SNOC_SLV_SERVICE_SNOC] = &slv_service_snoc,
	[SNOC_SLV_QDSS_STM] = &slv_qdss_stm,
};

static const struct qcom_icc_desc msm8974_snoc = {
	.nodes = msm8974_snoc_nodes,
	.num_nodes = ARRAY_SIZE(msm8974_snoc_nodes),
	.bus_clk_desc = &bus_1_clk,
	.get_bw = msm8974_get_bw,
	.ignore_enxio = true,
};

static const struct of_device_id msm8974_noc_of_match[] = {
	{ .compatible = "qcom,msm8974-bimc", .data = &msm8974_bimc},
	{ .compatible = "qcom,msm8974-cnoc", .data = &msm8974_cnoc},
	{ .compatible = "qcom,msm8974-mmssnoc", .data = &msm8974_mnoc},
	{ .compatible = "qcom,msm8974-ocmemnoc", .data = &msm8974_onoc},
	{ .compatible = "qcom,msm8974-pnoc", .data = &msm8974_pnoc},
	{ .compatible = "qcom,msm8974-snoc", .data = &msm8974_snoc},
	{ },
};
MODULE_DEVICE_TABLE(of, msm8974_noc_of_match);

static struct platform_driver msm8974_noc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-msm8974",
		.of_match_table = msm8974_noc_of_match,
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(msm8974_noc_driver);
MODULE_DESCRIPTION("Qualcomm MSM8974 NoC driver");
MODULE_AUTHOR("Brian Masney <masneyb@onstation.org>");
MODULE_LICENSE("GPL v2");
