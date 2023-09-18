// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Socionext Inc.
 *   Author: Masahiro Yamada <yamada.masahiro@socionext.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "sdhci-pltfm.h"

/* HRS - Host Register Set (specific to Cadence) */
#define SDHCI_CDNS_HRS04		0x10		/* PHY access port */
#define SDHCI_CDNS_HRS05        0x14        /* PHY data access port */
#define   SDHCI_CDNS_HRS04_ACK			BIT(26)
#define   SDHCI_CDNS_HRS04_RD			BIT(25)
#define   SDHCI_CDNS_HRS04_WR			BIT(24)
#define   SDHCI_CDNS_HRS04_RDATA		GENMASK(23, 16)
#define   SDHCI_CDNS_HRS04_WDATA		GENMASK(15, 8)
#define   SDHCI_CDNS_HRS04_ADDR			GENMASK(5, 0)

#define SDHCI_CDNS_HRS06		0x18		/* eMMC control */
#define   SDHCI_CDNS_HRS06_TUNE_UP		BIT(15)
#define   SDHCI_CDNS_HRS06_TUNE			GENMASK(13, 8)
#define   SDHCI_CDNS_HRS06_MODE			GENMASK(2, 0)
#define   SDHCI_CDNS_HRS06_MODE_SD		0x0
#define   SDHCI_CDNS_HRS06_MODE_MMC_SDR		0x2
#define   SDHCI_CDNS_HRS06_MODE_MMC_DDR		0x3
#define   SDHCI_CDNS_HRS06_MODE_MMC_HS200	0x4
#define   SDHCI_CDNS_HRS06_MODE_MMC_HS400	0x5
#define   SDHCI_CDNS_HRS06_MODE_MMC_HS400ES	0x6

/* PHY specific register */
/* HRS register to set after SDMMC reset */
#define SDHCI_CDNS_HRS00	0x0
#define SDHCI_CDNS_HRS07	0x1C        /* IO_DELAY_INFO_REG */
#define SDHCI_CDNS_HRS07_RW_COMPENSATE  GENMASK(20, 16) /* RW_COMPENSATE */
#define SDHCI_CDNS_HRS07_IDELAY_VAL     GENMASK(4, 0)   /* IDELAY_VAL */
/* TODO: check DV dfi_init val=9 */
#define SDHCI_CDNS_HRS07_RW_COMPENSATE_DATA 0x9
/* TODO: check DV dfi_init val=8 ; DDR Mode */
#define SDHCI_CDNS_HRS07_RW_COMPENSATE_DATA_DDR 0x8
#define SDHCI_CDNS_HRS07_IDELAY_VAL_DATA    0x0

#define SDHCI_CDNS_HRS09	0x024
#define SDHCI_CDNS_HRS09_PHY_SW_RESET       BIT(0)  /* PHY_SW_RESET */
#define SDHCI_CDNS_HRS09_PHY_INIT_COMPLETE  BIT(1)  /* PHY_INIT_COMPLETE */
#define SDHCI_CDNS_HRS09_RDDATA_EN  BIT(16) /* RDDATA_EN */
#define SDHCI_CDNS_HRS09_RDCMD_EN   BIT(15) /* RDCMD_EN */
#define SDHCI_CDNS_HRS09_EXTENDED_WR_MODE   BIT(3)  /* EXTENDED_WR_MODE */
#define SDHCI_CDNS_HRS09_EXTENDED_RD_MODE   BIT(2)  /* EXTENDED_RD_MODE */

#define SDHCI_CDNS_HRS10        0x28        /* PHY reset port */
#define SDHCI_CDNS_HRS10_HCSDCLKADJ     GENMASK(19, 16) /* HCSDCLKADJ */
#define SDHCI_CDNS_HRS10_HCSDCLKADJ_DATA    0x0         /* HCSDCLKADJ DATA */
/* HCSDCLKADJ DATA; DDR Mode */
#define SDHCI_CDNS_HRS10_HCSDCLKADJ_DATA_DDR    0x2
#define SDHCI_CDNS_HRS16        0x40        /* CMD_DATA_OUTPUT */

/* SRS - Slot Register Set (SDHCI-compatible) */
#define SDHCI_CDNS_SRS_BASE	0x200
#define SDHCI_CDNS_SRS09	0x224
#define SDHCI_CDNS_SRS10	0x228
#define SDHCI_CDNS_SRS11	0x22c
#define SDHCI_CDNS_SRS12	0x230
#define SDHCI_CDNS_SRS13	0x234
#define SDHCI_CDNS_SRS09_CI	BIT(16)
#define SDHCI_CDNS_SRS13_DATA	0xffffffff
#define SD_HOST_CLK 200000000

/* PHY */
#define SDHCI_CDNS_PHY_DLY_SD_HS	0x00
#define SDHCI_CDNS_PHY_DLY_SD_DEFAULT	0x01
#define SDHCI_CDNS_PHY_DLY_UHS_SDR12	0x02
#define SDHCI_CDNS_PHY_DLY_UHS_SDR25	0x03
#define SDHCI_CDNS_PHY_DLY_UHS_SDR50	0x04
#define SDHCI_CDNS_PHY_DLY_UHS_DDR50	0x05
#define SDHCI_CDNS_PHY_DLY_EMMC_LEGACY	0x06
#define SDHCI_CDNS_PHY_DLY_EMMC_SDR	0x07
#define SDHCI_CDNS_PHY_DLY_EMMC_DDR	0x08
#define SDHCI_CDNS_PHY_DLY_SDCLK	0x0b
#define SDHCI_CDNS_PHY_DLY_HSMMC	0x0c
#define SDHCI_CDNS_PHY_DLY_STROBE	0x0d
/* PHY register values */
#define PHY_DQ_TIMING_REG					0x2000
#define PHY_DQS_TIMING_REG					0x2004
#define PHY_GATE_LPBK_CTRL_REG				0x2008
#define PHY_DLL_MASTER_CTRL_REG				0x200C
#define PHY_DLL_SLAVE_CTRL_REG				0x2010
#define PHY_CTRL_REG						0x2080
#define USE_EXT_LPBK_DQS					BIT(22)
#define USE_LPBK_DQS						BIT(21)
#define USE_PHONY_DQS						BIT(20)
#define USE_PHONY_DQS_CMD					BIT(19)
#define SYNC_METHOD							BIT(31)
#define SW_HALF_CYCLE_SHIFT					BIT(28)
#define RD_DEL_SEL							GENMASK(24, 19)
#define RD_DEL_SEL_DATA						0x34
#define GATE_CFG_ALWAYS_ON					BIT(6)
#define UNDERRUN_SUPPRESS					BIT(18)
#define PARAM_DLL_BYPASS_MODE				BIT(23)
#define PARAM_PHASE_DETECT_SEL				GENMASK(22, 20)
#define PARAM_DLL_START_POINT				GENMASK(7, 0)
#define PARAM_PHASE_DETECT_SEL_DATA			0x2
#define PARAM_DLL_START_POINT_DATA			0x4
#define PARAM_DLL_START_POINT_DATA_SDR50	254

#define READ_DQS_CMD_DELAY		GENMASK(31, 24)
#define CLK_WRDQS_DELAY			GENMASK(23, 16)
#define CLK_WR_DELAY			GENMASK(15, 8)
#define READ_DQS_DELAY			GENMASK(7, 0)
#define READ_DQS_CMD_DELAY_DATA	0x0
#define CLK_WRDQS_DELAY_DATA	0x0
#define CLK_WR_DELAY_DATA		0x0
#define READ_DQS_DELAY_DATA		0x0

#define PHONY_DQS_TIMING		GENMASK(9, 4)
#define PHONY_DQS_TIMING_DATA	0x0

#define IO_MASK_ALWAYS_ON		BIT(31)
#define IO_MASK_END				GENMASK(29, 27)
#define IO_MASK_START			GENMASK(26, 24)
#define DATA_SELECT_OE_END		GENMASK(2, 0)
#define IO_MASK_END_DATA		0x5
/* DDR Mode */
#define IO_MASK_END_DATA_DDR	0x2
#define IO_MASK_START_DATA		0x0
#define DATA_SELECT_OE_END_DATA 0x1

/*
 * The tuned val register is 6 bit-wide, but not the whole of the range is
 * available.  The range 0-42 seems to be available (then 43 wraps around to 0)
 * but I am not quite sure if it is official.  Use only 0 to 39 for safety.
 */
#define SDHCI_CDNS_MAX_TUNING_LOOP	40

struct sdhci_cdns_phy_param {
	u32 addr;
	u32 data;
	u32 offset;
};

struct sdhci_cdns_priv {
	void __iomem *hrs_addr;
	void __iomem *ctl_addr;	/* write control */
	spinlock_t wrlock;	/* write lock */
	bool enhanced_strobe;
	void (*priv_writel)(struct sdhci_cdns_priv *priv, u32 val, void __iomem *reg);
	struct reset_control *rst_hw;
	unsigned int nr_phy_params;
	struct sdhci_cdns_phy_param phy_params[];
};

struct sdhci_cdns_phy_cfg {
	const char *property;
	u32 addr;
	u32 offset;
};

struct sdhci_cdns_drv_data {
	int (*init)(struct platform_device *pdev);
	const struct sdhci_pltfm_data pltfm_data;
};

static const struct sdhci_cdns_phy_cfg sdhci_cdns_phy_cfgs[] = {
	{ "cdns,phy-input-delay-sd-highspeed", SDHCI_CDNS_PHY_DLY_SD_HS, },
	{ "cdns,phy-input-delay-legacy", SDHCI_CDNS_PHY_DLY_SD_DEFAULT, },
	{ "cdns,phy-input-delay-sd-uhs-sdr12", SDHCI_CDNS_PHY_DLY_UHS_SDR12, },
	{ "cdns,phy-input-delay-sd-uhs-sdr25", SDHCI_CDNS_PHY_DLY_UHS_SDR25, },
	{ "cdns,phy-input-delay-sd-uhs-sdr50", SDHCI_CDNS_PHY_DLY_UHS_SDR50, },
	{ "cdns,phy-input-delay-sd-uhs-ddr50", SDHCI_CDNS_PHY_DLY_UHS_DDR50, },
	{ "cdns,phy-input-delay-mmc-highspeed", SDHCI_CDNS_PHY_DLY_EMMC_SDR, },
	{ "cdns,phy-input-delay-mmc-ddr", SDHCI_CDNS_PHY_DLY_EMMC_DDR, },
	{ "cdns,phy-dll-delay-sdclk", SDHCI_CDNS_PHY_DLY_SDCLK, },
	{ "cdns,phy-dll-delay-sdclk-hsmmc", SDHCI_CDNS_PHY_DLY_HSMMC, },
	{ "cdns,phy-dll-delay-strobe", SDHCI_CDNS_PHY_DLY_STROBE, },
	{ "cdns,phy-use-ext-lpbk-dqs", PHY_DQS_TIMING_REG, 22,},
	{ "cdns,phy-use-lpbk-dqs", PHY_DQS_TIMING_REG, 21,},
	{ "cdns,phy-use-phony-dqs", PHY_DQS_TIMING_REG, 20,},
	{ "cdns,phy-use-phony-dqs-cmd", PHY_DQS_TIMING_REG, 19,},
	{ "cdns,phy-io-mask-always-on", PHY_DQ_TIMING_REG, 31,},
	{ "cdns,phy-io-mask-end", PHY_DQ_TIMING_REG, 27,},
	{ "cdns,phy-io-mask-start", PHY_DQ_TIMING_REG, 24,},
	{ "cdns,phy-data-select-oe-end", PHY_DQ_TIMING_REG, 0,},
	{ "cdns,phy-sync-method", PHY_GATE_LPBK_CTRL_REG, 31,},
	{ "cdns,phy-sw-half-cycle-shift", PHY_GATE_LPBK_CTRL_REG, 28,},
	{ "cdns,phy-rd-del-sel", PHY_GATE_LPBK_CTRL_REG, 19,},
	{ "cdns,phy-underrun-suppress", PHY_GATE_LPBK_CTRL_REG, 18,},
	{ "cdns,phy-gate-cfg-always-on", PHY_GATE_LPBK_CTRL_REG, 6,},
	{ "cdns,phy-param-dll-bypass-mode", PHY_DLL_MASTER_CTRL_REG, 23,},
	{ "cdns,phy-param-phase-detect-sel", PHY_DLL_MASTER_CTRL_REG, 20,},
	{ "cdns,phy-param-dll-start-point", PHY_DLL_MASTER_CTRL_REG, 0,},
	{ "cdns,phy-read-dqs-cmd-delay", PHY_DLL_SLAVE_CTRL_REG, 24,},
	{ "cdns,phy-clk-wrdqs-delay", PHY_DLL_SLAVE_CTRL_REG, 16,},
	{ "cdns,phy-clk-wr-delay", PHY_DLL_SLAVE_CTRL_REG, 8,},
	{ "cdns,phy-read-dqs-delay", PHY_DLL_SLAVE_CTRL_REG, 0,},
	{ "cdns,phy-phony-dqs-timing", PHY_CTRL_REG, 4,},
	{ "cdns,hrs09-rddata-en", SDHCI_CDNS_HRS09, 16,},
	{ "cdns,hrs09-rdcmd-en", SDHCI_CDNS_HRS09, 15,},
	{ "cdns,hrs09-extended-wr-mode", SDHCI_CDNS_HRS09, 3,},
	{ "cdns,hrs09-extended-rd-mode", SDHCI_CDNS_HRS09, 2,},
	{ "cdns,hrs10-hcsdclkadj", SDHCI_CDNS_HRS10, 16,},
	{ "cdns,hrs16-wrdata1-sdclk-dly", SDHCI_CDNS_HRS16, 28,},
	{ "cdns,hrs16-wrdata0-sdclk-dly", SDHCI_CDNS_HRS16, 24,},
	{ "cdns,hrs16-wrcmd1-sdclk-dly", SDHCI_CDNS_HRS16, 20,},
	{ "cdns,hrs16-wrcmd0-sdclk-dly", SDHCI_CDNS_HRS16, 16,},
	{ "cdns,hrs16-wrdata1-dly", SDHCI_CDNS_HRS16, 12,},
	{ "cdns,hrs16-wrdata0-dly", SDHCI_CDNS_HRS16, 8,},
	{ "cdns,hrs16-wrcmd1-dly", SDHCI_CDNS_HRS16, 4,},
	{ "cdns,hrs16-wrcmd0-dly", SDHCI_CDNS_HRS16, 0,},
	{ "cdns,hrs07-rw-compensate", SDHCI_CDNS_HRS07, 16,},
	{ "cdns,hrs07-idelay-val", SDHCI_CDNS_HRS07, 0,},
};

static inline void cdns_writel(struct sdhci_cdns_priv *priv, u32 val,
			       void __iomem *reg)
{
	writel(val, reg);
}

static int sdhci_cdns_write_phy_reg(struct sdhci_cdns_priv *priv,
				    u8 addr, u8 data)
{
	void __iomem *reg = priv->hrs_addr + SDHCI_CDNS_HRS04;
	u32 tmp;
	int ret;

	ret = readl_poll_timeout(reg, tmp, !(tmp & SDHCI_CDNS_HRS04_ACK),
				 0, 10);
	if (ret)
		return ret;

	tmp = FIELD_PREP(SDHCI_CDNS_HRS04_WDATA, data) |
	      FIELD_PREP(SDHCI_CDNS_HRS04_ADDR, addr);
	priv->priv_writel(priv, tmp, reg);

	tmp |= SDHCI_CDNS_HRS04_WR;
	priv->priv_writel(priv, tmp, reg);

	ret = readl_poll_timeout(reg, tmp, tmp & SDHCI_CDNS_HRS04_ACK, 0, 10);
	if (ret)
		return ret;

	tmp &= ~SDHCI_CDNS_HRS04_WR;
	priv->priv_writel(priv, tmp, reg);

	ret = readl_poll_timeout(reg, tmp, !(tmp & SDHCI_CDNS_HRS04_ACK),
				 0, 10);

	return ret;
}

static int sdhci_cdns_write_phy_reg_mask(struct sdhci_cdns_priv *priv,
			u32 addr, u32 data, u32 mask)
{
	u32 tmp;

	tmp = addr;

	/* get PHY address */
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS04);

	/* read current PHY register value, before write */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS05);

	tmp &= ~mask;
	tmp |= data;

	/* write operation */
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS05);

	/* re-read PHY address */
	writel(addr, priv->hrs_addr + SDHCI_CDNS_HRS04);

	/* re-read current PHY register value, check */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS05);

	return 0;
}

static u32 sdhci_cdns_read_phy_reg(struct sdhci_cdns_priv *priv,
			u32 addr)
{
	u32 tmp;

	tmp = addr;

	/* get PHY address */
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS04);

	/* read current PHY register value, before write */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS05);

	return tmp;
}

static int sdhci_cdns_dfi_phy_val(struct sdhci_cdns_priv *priv, u32 reg)
{
	int i;
	u32 tmp;

	tmp = 0;

	for (i = 0; i < priv->nr_phy_params; i++) {
		if (priv->phy_params[i].addr == reg)
			tmp |= priv->phy_params[i].data << priv->phy_params[i].offset;
	}

	return tmp;
}

static int sdhci_cdns_combophy_init_sd_dfi_init(struct sdhci_cdns_priv *priv)
{
	int ret = 0;
	u32 mask = 0x0;
	u32 tmp = 0;

	writel(0x0, priv->hrs_addr + SDHCI_CDNS_SRS11);
	writel(1<<0, priv->hrs_addr + SDHCI_CDNS_HRS00);
	while ((readl(priv->hrs_addr + SDHCI_CDNS_HRS00) & 1<<0) == 1)

	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & ~1;
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	tmp = (1 << 22) | (1 << 21) | (1 << 20) | (1 << 19);
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DQS_TIMING_REG, tmp, tmp);

	tmp = (1 << 31) | (0 << 28) | (52 << 19) | (1 << 18) | (1 << 6);
	mask = SYNC_METHOD | SW_HALF_CYCLE_SHIFT | RD_DEL_SEL | UNDERRUN_SUPPRESS |
			GATE_CFG_ALWAYS_ON;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_GATE_LPBK_CTRL_REG, tmp,
			mask);

	tmp = (1 << 23) | (2 << 20) | (4 << 0);
	mask = PARAM_DLL_BYPASS_MODE | PARAM_PHASE_DETECT_SEL |
			PARAM_DLL_START_POINT;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DLL_MASTER_CTRL_REG, tmp,
			mask);

	tmp = (0 << 24) | (0 << 16) | (0 << 8) | (0 << 0);
	mask = READ_DQS_CMD_DELAY | CLK_WRDQS_DELAY | CLK_WR_DELAY | READ_DQS_DELAY;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DLL_SLAVE_CTRL_REG, tmp,
			mask);

	writel(0x2080, priv->hrs_addr + SDHCI_CDNS_HRS04);
	tmp &= ~(0x3f << 4);
	mask = PHONY_DQS_TIMING;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_CTRL_REG, tmp, mask);

	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) | 1;
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	while (~readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & (1 << 1))

	tmp = sdhci_cdns_read_phy_reg(priv, PHY_DQ_TIMING_REG) & 0x07FFFF8;
	tmp |= (0 << 31) | (0 << 27) | (0 << 24) | (1 << 0);
	mask = IO_MASK_ALWAYS_ON | IO_MASK_END | IO_MASK_START | DATA_SELECT_OE_END;

	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DQ_TIMING_REG, tmp, mask);

	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & 0xFFFE7FF3;

	tmp |= (1 << 16) | (1 << 15) | (1 << 3) | (1 << 2);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS10) & 0xFFF0FFFF;

	tmp |= (0 << 16);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS10);

	tmp = (0 << 28) | (0 << 24) | (0 << 20) | (0 << 16) | (0 << 12) | (1 << 8) |
			(0 << 4) | (1 << 0);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS16);

	tmp = (9 << 16) | (0 << 0);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS07);

	return ret;
}

static int sdhci_cdns_combophy_init_sd_gen(struct sdhci_cdns_priv *priv)
{
	u32 tmp;
	int ret = 0;
	u32 mask = 0x0;

	/* step 1, switch on DLL_RESET */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & ~1;
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	/* step 2, program PHY_DQS_TIMING_REG */
	tmp = sdhci_cdns_dfi_phy_val(priv, PHY_DQS_TIMING_REG);
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DQS_TIMING_REG, tmp, tmp);

	/* step 3, program PHY_GATE_LPBK_CTRL_REG */
	tmp = sdhci_cdns_dfi_phy_val(priv, PHY_GATE_LPBK_CTRL_REG);
	mask = SYNC_METHOD | SW_HALF_CYCLE_SHIFT | RD_DEL_SEL | UNDERRUN_SUPPRESS |
	GATE_CFG_ALWAYS_ON;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_GATE_LPBK_CTRL_REG, tmp,
	mask);

	/* step 4, program PHY_DLL_MASTER_CTRL_REG */
	tmp = sdhci_cdns_dfi_phy_val(priv, PHY_DLL_MASTER_CTRL_REG);
	mask = PARAM_DLL_BYPASS_MODE | PARAM_PHASE_DETECT_SEL |
	PARAM_DLL_START_POINT;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DLL_MASTER_CTRL_REG, tmp,
	mask);

	/* step 5, program PHY_DLL_SLAVE_CTRL_REG */
	tmp = sdhci_cdns_dfi_phy_val(priv, PHY_DLL_SLAVE_CTRL_REG);
	mask = READ_DQS_CMD_DELAY | CLK_WRDQS_DELAY | CLK_WR_DELAY | READ_DQS_DELAY;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DLL_SLAVE_CTRL_REG, tmp,
	mask);

	/* step 7, switch off DLL_RESET */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) | 1;
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	/* step 8, polling PHY_INIT_COMPLETE */
	while (~readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & (1 << 1))
	/* polling for PHY_INIT_COMPLETE bit */

	/* step 9, program PHY_DQ_TIMING_REG */
	tmp = sdhci_cdns_read_phy_reg(priv, PHY_DQ_TIMING_REG) & 0x07FFFF8;
	tmp |= sdhci_cdns_dfi_phy_val(priv, PHY_DQ_TIMING_REG);
	mask = IO_MASK_ALWAYS_ON | IO_MASK_END | IO_MASK_START | DATA_SELECT_OE_END;
	ret = sdhci_cdns_write_phy_reg_mask(priv, PHY_DQ_TIMING_REG, tmp, mask);

	/* step 10, program HRS09, register 42 */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS09) & 0xFFFE7FF3;

	tmp |= sdhci_cdns_dfi_phy_val(priv, SDHCI_CDNS_HRS09);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS09);

	/* step 11, program HRS10, register 43 */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS10) & 0xFFF0FFFF;
	tmp |= sdhci_cdns_dfi_phy_val(priv, SDHCI_CDNS_HRS10);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS10);

	/* step 12, program HRS16, register 48 */
	tmp = sdhci_cdns_dfi_phy_val(priv, SDHCI_CDNS_HRS16);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS16);

	/* step 13, program HRS07, register 40 */
	tmp = sdhci_cdns_dfi_phy_val(priv, SDHCI_CDNS_HRS07);
	writel(tmp, priv->hrs_addr + SDHCI_CDNS_HRS07);
	/* end of combophy init */

	return ret;
}


static unsigned int sdhci_cdns_phy_param_count(struct device_node *np)
{
	unsigned int count = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(sdhci_cdns_phy_cfgs); i++)
		if (of_property_read_bool(np, sdhci_cdns_phy_cfgs[i].property))
			count++;

	return count;
}

static void sdhci_cdns_phy_param_parse(struct device_node *np,
				       struct sdhci_cdns_priv *priv)
{
	struct sdhci_cdns_phy_param *p = priv->phy_params;
	u32 val;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(sdhci_cdns_phy_cfgs); i++) {
		ret = of_property_read_u32(np, sdhci_cdns_phy_cfgs[i].property,
					   &val);
		if (ret)
			continue;

		p->addr = sdhci_cdns_phy_cfgs[i].addr;
		p->data = val;
		p->offset = sdhci_cdns_phy_cfgs[i].offset;
		p++;
	}
}

static int sdhci_cdns_phy_init(struct sdhci_cdns_priv *priv)
{
	int ret, i;

	for (i = 0; i < priv->nr_phy_params; i++) {
		if (priv->phy_params[i].offset)
			break;

		ret = sdhci_cdns_write_phy_reg(priv, priv->phy_params[i].addr,
					       priv->phy_params[i].data);
		if (ret)
			return ret;
	}

	return 0;
}

static void *sdhci_cdns_priv(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return sdhci_pltfm_priv(pltfm_host);
}

static unsigned int sdhci_cdns_get_timeout_clock(struct sdhci_host *host)
{
	/*
	 * Cadence's spec says the Timeout Clock Frequency is the same as the
	 * Base Clock Frequency.
	 */
	return host->max_clk;
}

static void sdhci_cdns_set_emmc_mode(struct sdhci_cdns_priv *priv, u32 mode)
{
	u32 tmp;

	/* The speed mode for eMMC is selected by HRS06 register */
	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS06);
	tmp &= ~SDHCI_CDNS_HRS06_MODE;
	tmp |= FIELD_PREP(SDHCI_CDNS_HRS06_MODE, mode);
	priv->priv_writel(priv, tmp, priv->hrs_addr + SDHCI_CDNS_HRS06);
}

static u32 sdhci_cdns_get_emmc_mode(struct sdhci_cdns_priv *priv)
{
	u32 tmp;

	tmp = readl(priv->hrs_addr + SDHCI_CDNS_HRS06);
	return FIELD_GET(SDHCI_CDNS_HRS06_MODE, tmp);
}

static int sdhci_cdns_set_tune_val(struct sdhci_host *host, unsigned int val)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	void __iomem *reg = priv->hrs_addr + SDHCI_CDNS_HRS06;
	u32 tmp;
	int i, ret;

	if (WARN_ON(!FIELD_FIT(SDHCI_CDNS_HRS06_TUNE, val)))
		return -EINVAL;

	tmp = readl(reg);
	tmp &= ~SDHCI_CDNS_HRS06_TUNE;
	tmp |= FIELD_PREP(SDHCI_CDNS_HRS06_TUNE, val);

	/*
	 * Workaround for IP errata:
	 * The IP6116 SD/eMMC PHY design has a timing issue on receive data
	 * path. Send tune request twice.
	 */
	for (i = 0; i < 2; i++) {
		tmp |= SDHCI_CDNS_HRS06_TUNE_UP;
		priv->priv_writel(priv, tmp, reg);

		ret = readl_poll_timeout(reg, tmp,
					 !(tmp & SDHCI_CDNS_HRS06_TUNE_UP),
					 0, 1);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * In SD mode, software must not use the hardware tuning and instead perform
 * an almost identical procedure to eMMC.
 */
static int sdhci_cdns_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	int cur_streak = 0;
	int max_streak = 0;
	int end_of_streak = 0;
	int i;

	/*
	 * Do not execute tuning for UHS_SDR50 or UHS_DDR50.
	 * The delay is set by probe, based on the DT properties.
	 */
	if (host->timing != MMC_TIMING_MMC_HS200 &&
	    host->timing != MMC_TIMING_UHS_SDR104)
		return 0;

	for (i = 0; i < SDHCI_CDNS_MAX_TUNING_LOOP; i++) {
		if (sdhci_cdns_set_tune_val(host, i) ||
		    mmc_send_tuning(host->mmc, opcode, NULL)) { /* bad */
			cur_streak = 0;
		} else { /* good */
			cur_streak++;
			if (cur_streak > max_streak) {
				max_streak = cur_streak;
				end_of_streak = i;
			}
		}
	}

	if (!max_streak) {
		dev_err(mmc_dev(host->mmc), "no tuning point found\n");
		return -EIO;
	}

	return sdhci_cdns_set_tune_val(host, end_of_streak - max_streak / 2);
}

static void sdhci_cdns_set_uhs_signaling(struct sdhci_host *host,
					 unsigned int timing)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	u32 mode;

	switch (timing) {
	case MMC_TIMING_MMC_HS:
		mode = SDHCI_CDNS_HRS06_MODE_MMC_SDR;
		break;
	case MMC_TIMING_MMC_DDR52:
		mode = SDHCI_CDNS_HRS06_MODE_MMC_DDR;
		break;
	case MMC_TIMING_MMC_HS200:
		mode = SDHCI_CDNS_HRS06_MODE_MMC_HS200;
		break;
	case MMC_TIMING_MMC_HS400:
		if (priv->enhanced_strobe)
			mode = SDHCI_CDNS_HRS06_MODE_MMC_HS400ES;
		else
			mode = SDHCI_CDNS_HRS06_MODE_MMC_HS400;
		break;
	default:
		mode = SDHCI_CDNS_HRS06_MODE_SD;
		break;
	}

	sdhci_cdns_set_emmc_mode(priv, mode);

	/* For SD, fall back to the default handler */
	if (mode == SDHCI_CDNS_HRS06_MODE_SD)
		sdhci_set_uhs_signaling(host, timing);
}

/* Elba control register bits [6:3] are byte-lane enables */
#define ELBA_BYTE_ENABLE_MASK(x)	((x) << 3)

/*
 * The Pensando Elba SoC explicitly controls byte-lane enabling on writes
 * which includes writes to the HRS registers.  The write lock (wrlock)
 * is used to ensure byte-lane enable, using write control (ctl_addr),
 * occurs before the data write.
 */
static void elba_priv_writel(struct sdhci_cdns_priv *priv, u32 val,
			     void __iomem *reg)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->wrlock, flags);
	writel(GENMASK(7, 3), priv->ctl_addr);
	writel(val, reg);
	spin_unlock_irqrestore(&priv->wrlock, flags);
}

static void elba_write_l(struct sdhci_host *host, u32 val, int reg)
{
	elba_priv_writel(sdhci_cdns_priv(host), val, host->ioaddr + reg);
}

static void elba_write_w(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	u32 shift = reg & GENMASK(1, 0);
	unsigned long flags;
	u32 byte_enables;

	byte_enables = GENMASK(1, 0) << shift;
	spin_lock_irqsave(&priv->wrlock, flags);
	writel(ELBA_BYTE_ENABLE_MASK(byte_enables), priv->ctl_addr);
	writew(val, host->ioaddr + reg);
	spin_unlock_irqrestore(&priv->wrlock, flags);
}

static void elba_write_b(struct sdhci_host *host, u8 val, int reg)
{
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	u32 shift = reg & GENMASK(1, 0);
	unsigned long flags;
	u32 byte_enables;

	byte_enables = BIT(0) << shift;
	spin_lock_irqsave(&priv->wrlock, flags);
	writel(ELBA_BYTE_ENABLE_MASK(byte_enables), priv->ctl_addr);
	writeb(val, host->ioaddr + reg);
	spin_unlock_irqrestore(&priv->wrlock, flags);
}

static const struct sdhci_ops sdhci_elba_ops = {
	.write_l = elba_write_l,
	.write_w = elba_write_w,
	.write_b = elba_write_b,
	.set_clock = sdhci_set_clock,
	.get_timeout_clock = sdhci_cdns_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_cdns_set_uhs_signaling,
};

static int elba_drv_init(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	void __iomem *ioaddr;

	host->mmc->caps |= MMC_CAP_1_8V_DDR | MMC_CAP_8_BIT_DATA;
	spin_lock_init(&priv->wrlock);

	/* Byte-lane control register */
	ioaddr = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(ioaddr))
		return PTR_ERR(ioaddr);

	priv->ctl_addr = ioaddr;
	priv->priv_writel = elba_priv_writel;
	writel(ELBA_BYTE_ENABLE_MASK(0xf), priv->ctl_addr);

	return 0;
}

static void sdhci_cdns_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_cdns_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	if (host->mmc->actual_clock == 200000000)
		ret = sdhci_cdns_combophy_init_sd_gen(priv);
	else
		pr_info("%s: skip setting HRS regs.\n", __func__);

	sdhci_set_clock(host, clock);
}

static const struct sdhci_ops sdhci_cdns_ops = {
	.set_clock = sdhci_cdns_set_clock,
	.get_timeout_clock = sdhci_cdns_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.platform_execute_tuning = sdhci_cdns_execute_tuning,
	.set_uhs_signaling = sdhci_cdns_set_uhs_signaling,
};

static const struct sdhci_cdns_drv_data sdhci_cdns_uniphier_drv_data = {
	.pltfm_data = {
		.ops = &sdhci_cdns_ops,
		.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	},
};

static const struct sdhci_cdns_drv_data sdhci_elba_drv_data = {
	.init = elba_drv_init,
	.pltfm_data = {
		.ops = &sdhci_elba_ops,
	},
};

static const struct sdhci_cdns_drv_data sdhci_cdns_drv_data = {
	.pltfm_data = {
		.ops = &sdhci_cdns_ops,
	},
};

static const struct sdhci_pltfm_data sdhci_cdns_agilex5_pltfm_data = {
	.ops = &sdhci_cdns_ops,
	.quirks2 = SDHCI_QUIRK2_40_BIT_DMA_MASK,
};

static void sdhci_cdns_hs400_enhanced_strobe(struct mmc_host *mmc,
					     struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);
	u32 mode;

	priv->enhanced_strobe = ios->enhanced_strobe;

	mode = sdhci_cdns_get_emmc_mode(priv);

	if (mode == SDHCI_CDNS_HRS06_MODE_MMC_HS400 && ios->enhanced_strobe)
		sdhci_cdns_set_emmc_mode(priv,
					 SDHCI_CDNS_HRS06_MODE_MMC_HS400ES);

	if (mode == SDHCI_CDNS_HRS06_MODE_MMC_HS400ES && !ios->enhanced_strobe)
		sdhci_cdns_set_emmc_mode(priv,
					 SDHCI_CDNS_HRS06_MODE_MMC_HS400);
}

static void sdhci_cdns_mmc_hw_reset(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_cdns_priv *priv = sdhci_cdns_priv(host);

	dev_dbg(mmc_dev(host->mmc), "emmc hardware reset\n");

	reset_control_assert(priv->rst_hw);
	/* For eMMC, minimum is 1us but give it 3us for good measure */
	udelay(3);

	reset_control_deassert(priv->rst_hw);
	/* For eMMC, minimum is 200us but give it 300us for good measure */
	usleep_range(300, 1000);
}

static int sdhci_cdns_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	const struct sdhci_cdns_drv_data *data;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_cdns_priv *priv;
	struct clk *clk;
	unsigned int nr_phy_params;
	int ret;
	struct device *dev = &pdev->dev;
	static const u16 version = SDHCI_SPEC_400 << SDHCI_SPEC_VER_SHIFT;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	data = of_device_get_match_data(dev);
	if (!data)
		data = &sdhci_cdns_drv_data;

	nr_phy_params = sdhci_cdns_phy_param_count(dev->of_node);
	host = sdhci_pltfm_init(pdev, &data->pltfm_data,
				struct_size(priv, phy_params, nr_phy_params));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = clk;

	priv = sdhci_pltfm_priv(pltfm_host);
	priv->nr_phy_params = nr_phy_params;
	priv->hrs_addr = host->ioaddr;
	priv->enhanced_strobe = false;
	priv->priv_writel = cdns_writel;
	host->ioaddr += SDHCI_CDNS_SRS_BASE;
	host->mmc_host_ops.hs400_enhanced_strobe =
				sdhci_cdns_hs400_enhanced_strobe;
	if (data->init) {
		ret = data->init(pdev);
		if (ret)
			goto free;
	}
	sdhci_enable_v4_mode(host);
	__sdhci_read_caps(host, &version, NULL, NULL);

	sdhci_get_of_property(pdev);

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto free;

	sdhci_cdns_phy_param_parse(dev->of_node, priv);

	ret = sdhci_cdns_phy_init(priv);
	if (ret)
		goto free;

	if (host->mmc->caps & MMC_CAP_HW_RESET) {
		priv->rst_hw = devm_reset_control_get_optional_exclusive(dev, NULL);
		if (IS_ERR(priv->rst_hw)) {
			ret = dev_err_probe(mmc_dev(host->mmc), PTR_ERR(priv->rst_hw),
					    "reset controller error\n");
			goto free;
		}
		if (priv->rst_hw)
			host->mmc_host_ops.card_hw_reset = sdhci_cdns_mmc_hw_reset;
	}

	ret = sdhci_cdns_combophy_init_sd_dfi_init(priv);
	if (ret)
		goto free;

	ret = sdhci_add_host(host);
	if (ret)
		goto free;

	return 0;
free:
	sdhci_pltfm_free(pdev);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int sdhci_cdns_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_cdns_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		return ret;

	ret = sdhci_cdns_phy_init(priv);
	if (ret)
		goto disable_clk;

	ret = sdhci_resume_host(host);
	if (ret)
		goto disable_clk;

	return 0;

disable_clk:
	clk_disable_unprepare(pltfm_host->clk);

	return ret;
}
#endif

static const struct dev_pm_ops sdhci_cdns_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_pltfm_suspend, sdhci_cdns_resume)
};

static const struct of_device_id sdhci_cdns_match[] = {
	{
		.compatible = "socionext,uniphier-sd4hc",
		.data = &sdhci_cdns_uniphier_drv_data,
	},
	{
		.compatible = "amd,pensando-elba-sd4hc",
		.data = &sdhci_elba_drv_data,
	},
	{ .compatible = "cdns,sd4hc" },
	{
		.compatible = "intel,agilex5-sd4hc",
		.data = &sdhci_cdns_agilex5_pltfm_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sdhci_cdns_match);

static struct platform_driver sdhci_cdns_driver = {
	.driver = {
		.name = "sdhci-cdns",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm = &sdhci_cdns_pm_ops,
		.of_match_table = sdhci_cdns_match,
	},
	.probe = sdhci_cdns_probe,
	.remove_new = sdhci_pltfm_remove,
};
module_platform_driver(sdhci_cdns_driver);

MODULE_AUTHOR("Masahiro Yamada <yamada.masahiro@socionext.com>");
MODULE_DESCRIPTION("Cadence SD/SDIO/eMMC Host Controller Driver");
MODULE_LICENSE("GPL");
