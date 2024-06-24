/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 */

#ifndef UFS_QCOM_H_
#define UFS_QCOM_H_

#include <linux/reset-controller.h>
#include <linux/reset.h>
#include <soc/qcom/ice.h>
#include <ufs/ufshcd.h>

#define MPHY_TX_FSM_STATE       0x41
#define TX_FSM_HIBERN8          0x1
#define HBRN8_POLL_TOUT_MS      100
#define DEFAULT_CLK_RATE_HZ     1000000
#define MAX_SUPP_MAC		64
#define MAX_ESI_VEC		32

#define UFS_HW_VER_MAJOR_MASK	GENMASK(31, 28)
#define UFS_HW_VER_MINOR_MASK	GENMASK(27, 16)
#define UFS_HW_VER_STEP_MASK	GENMASK(15, 0)
#define UFS_DEV_VER_MAJOR_MASK	GENMASK(7, 4)

#define UFS_QCOM_LIMIT_HS_RATE		PA_HS_MODE_B

/* QCOM UFS host controller vendor specific registers */
enum {
	REG_UFS_SYS1CLK_1US                 = 0xC0,
	REG_UFS_TX_SYMBOL_CLK_NS_US         = 0xC4,
	REG_UFS_LOCAL_PORT_ID_REG           = 0xC8,
	REG_UFS_PA_ERR_CODE                 = 0xCC,
	/* On older UFS revisions, this register is called "RETRY_TIMER_REG" */
	REG_UFS_PARAM0                      = 0xD0,
	/* On older UFS revisions, this register is called "REG_UFS_PA_LINK_STARTUP_TIMER" */
	REG_UFS_CFG0                        = 0xD8,
	REG_UFS_CFG1                        = 0xDC,
	REG_UFS_CFG2                        = 0xE0,
	REG_UFS_HW_VERSION                  = 0xE4,

	UFS_TEST_BUS				= 0xE8,
	UFS_TEST_BUS_CTRL_0			= 0xEC,
	UFS_TEST_BUS_CTRL_1			= 0xF0,
	UFS_TEST_BUS_CTRL_2			= 0xF4,
	UFS_UNIPRO_CFG				= 0xF8,

	/*
	 * QCOM UFS host controller vendor specific registers
	 * added in HW Version 3.0.0
	 */
	UFS_AH8_CFG				= 0xFC,

	REG_UFS_CFG3				= 0x271C,

	REG_UFS_DEBUG_SPARE_CFG			= 0x284C,
};

/* QCOM UFS host controller vendor specific debug registers */
enum {
	UFS_DBG_RD_REG_UAWM			= 0x100,
	UFS_DBG_RD_REG_UARM			= 0x200,
	UFS_DBG_RD_REG_TXUC			= 0x300,
	UFS_DBG_RD_REG_RXUC			= 0x400,
	UFS_DBG_RD_REG_DFC			= 0x500,
	UFS_DBG_RD_REG_TRLUT			= 0x600,
	UFS_DBG_RD_REG_TMRLUT			= 0x700,
	UFS_UFS_DBG_RD_REG_OCSC			= 0x800,

	UFS_UFS_DBG_RD_DESC_RAM			= 0x1500,
	UFS_UFS_DBG_RD_PRDT_RAM			= 0x1700,
	UFS_UFS_DBG_RD_RESP_RAM			= 0x1800,
	UFS_UFS_DBG_RD_EDTL_RAM			= 0x1900,
};

enum {
	UFS_MEM_CQIS_VS		= 0x8,
};

#define UFS_CNTLR_2_x_x_VEN_REGS_OFFSET(x)	(0x000 + x)
#define UFS_CNTLR_3_x_x_VEN_REGS_OFFSET(x)	(0x400 + x)

/* bit definitions for REG_UFS_CFG0 register */
#define QUNIPRO_G4_SEL		BIT(5)

/* bit definitions for REG_UFS_CFG1 register */
#define QUNIPRO_SEL		BIT(0)
#define UFS_PHY_SOFT_RESET	BIT(1)
#define UTP_DBG_RAMS_EN		BIT(17)
#define TEST_BUS_EN		BIT(18)
#define TEST_BUS_SEL		GENMASK(22, 19)
#define UFS_REG_TEST_BUS_EN	BIT(30)

/* bit definitions for REG_UFS_CFG2 register */
#define UAWM_HW_CGC_EN		BIT(0)
#define UARM_HW_CGC_EN		BIT(1)
#define TXUC_HW_CGC_EN		BIT(2)
#define RXUC_HW_CGC_EN		BIT(3)
#define DFC_HW_CGC_EN		BIT(4)
#define TRLUT_HW_CGC_EN		BIT(5)
#define TMRLUT_HW_CGC_EN	BIT(6)
#define OCSC_HW_CGC_EN		BIT(7)

/* bit definitions for REG_UFS_CFG3 register */
#define ESI_VEC_MASK		GENMASK(22, 12)

/* bit definitions for REG_UFS_PARAM0 */
#define MAX_HS_GEAR_MASK	GENMASK(6, 4)
#define UFS_QCOM_MAX_GEAR(x)	FIELD_GET(MAX_HS_GEAR_MASK, (x))

/* bit definition for UFS_UFS_TEST_BUS_CTRL_n */
#define TEST_BUS_SUB_SEL_MASK	GENMASK(4, 0)  /* All XXX_SEL fields are 5 bits wide */

#define REG_UFS_CFG2_CGC_EN_ALL (UAWM_HW_CGC_EN | UARM_HW_CGC_EN |\
				 TXUC_HW_CGC_EN | RXUC_HW_CGC_EN |\
				 DFC_HW_CGC_EN | TRLUT_HW_CGC_EN |\
				 TMRLUT_HW_CGC_EN | OCSC_HW_CGC_EN)

/* QUniPro Vendor specific attributes */
#define PA_VS_CONFIG_REG1	0x9000
#define DME_VS_CORE_CLK_CTRL	0xD002
/* bit and mask definitions for DME_VS_CORE_CLK_CTRL attribute */
#define CLK_1US_CYCLES_MASK_V4				GENMASK(27, 16)
#define CLK_1US_CYCLES_MASK				GENMASK(7, 0)
#define DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT	BIT(8)
#define PA_VS_CORE_CLK_40NS_CYCLES			0x9007
#define PA_VS_CORE_CLK_40NS_CYCLES_MASK			GENMASK(6, 0)


/* QCOM UFS host controller core clk frequencies */
#define UNIPRO_CORE_CLK_FREQ_37_5_MHZ          38
#define UNIPRO_CORE_CLK_FREQ_75_MHZ            75
#define UNIPRO_CORE_CLK_FREQ_100_MHZ           100
#define UNIPRO_CORE_CLK_FREQ_150_MHZ           150
#define UNIPRO_CORE_CLK_FREQ_300_MHZ           300
#define UNIPRO_CORE_CLK_FREQ_201_5_MHZ         202
#define UNIPRO_CORE_CLK_FREQ_403_MHZ           403

static inline void
ufs_qcom_get_controller_revision(struct ufs_hba *hba,
				 u8 *major, u16 *minor, u16 *step)
{
	u32 ver = ufshcd_readl(hba, REG_UFS_HW_VERSION);

	*major = FIELD_GET(UFS_HW_VER_MAJOR_MASK, ver);
	*minor = FIELD_GET(UFS_HW_VER_MINOR_MASK, ver);
	*step = FIELD_GET(UFS_HW_VER_STEP_MASK, ver);
};

static inline void ufs_qcom_assert_reset(struct ufs_hba *hba)
{
	ufshcd_rmwl(hba, UFS_PHY_SOFT_RESET, UFS_PHY_SOFT_RESET, REG_UFS_CFG1);

	/*
	 * Dummy read to ensure the write takes effect before doing any sort
	 * of delay
	 */
	ufshcd_readl(hba, REG_UFS_CFG1);
}

static inline void ufs_qcom_deassert_reset(struct ufs_hba *hba)
{
	ufshcd_rmwl(hba, UFS_PHY_SOFT_RESET, 0, REG_UFS_CFG1);

	/*
	 * Dummy read to ensure the write takes effect before doing any sort
	 * of delay
	 */
	ufshcd_readl(hba, REG_UFS_CFG1);
}

/* Host controller hardware version: major.minor.step */
struct ufs_hw_version {
	u16 step;
	u16 minor;
	u8 major;
};

struct ufs_qcom_testbus {
	u8 select_major;
	u8 select_minor;
};

struct gpio_desc;

struct ufs_qcom_host {
	struct phy *generic_phy;
	struct ufs_hba *hba;
	struct ufs_pa_layer_attr dev_req_params;
	struct clk_bulk_data *clks;
	u32 num_clks;
	bool is_lane_clks_enabled;

	struct icc_path *icc_ddr;
	struct icc_path *icc_cpu;

#ifdef CONFIG_SCSI_UFS_CRYPTO
	struct qcom_ice *ice;
#endif

	void __iomem *dev_ref_clk_ctrl_mmio;
	bool is_dev_ref_clk_enabled;
	struct ufs_hw_version hw_ver;

	u32 dev_ref_clk_en_mask;

	struct ufs_qcom_testbus testbus;

	/* Reset control of HCI */
	struct reset_control *core_reset;
	struct reset_controller_dev rcdev;

	struct gpio_desc *device_reset;

	struct ufs_host_params host_params;
	u32 phy_gear;

	bool esi_enabled;
};

static inline u32
ufs_qcom_get_debug_reg_offset(struct ufs_qcom_host *host, u32 reg)
{
	if (host->hw_ver.major <= 0x02)
		return UFS_CNTLR_2_x_x_VEN_REGS_OFFSET(reg);

	return UFS_CNTLR_3_x_x_VEN_REGS_OFFSET(reg);
};

#define ufs_qcom_is_link_off(hba) ufshcd_is_link_off(hba)
#define ufs_qcom_is_link_active(hba) ufshcd_is_link_active(hba)
#define ufs_qcom_is_link_hibern8(hba) ufshcd_is_link_hibern8(hba)
#define ceil(freq, div) ((freq) % (div) == 0 ? ((freq)/(div)) : ((freq)/(div) + 1))

int ufs_qcom_testbus_config(struct ufs_qcom_host *host);

#endif /* UFS_QCOM_H_ */
