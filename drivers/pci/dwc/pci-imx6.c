/*
 * PCIe host controller driver for Freescale i.MX6 SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <dt-bindings/soc/imx8_hsio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/busfreq-imx.h>
#include <linux/regulator/consumer.h>

#include "pcie-designware.h"

#define to_imx6_pcie(x)	dev_get_drvdata((x)->dev)

enum imx6_pcie_variants {
	IMX6Q,
	IMX6SX,
	IMX6QP,
	IMX7D,
	IMX8QM,
	IMX8MQ,
};

/*
 * The default value of the reserved ddr memory
 * used to verify EP/RC memory space access operations.
 * The layout of the 1G ddr on SD boards
 * [imx6qdl-sd-ard boards]0x1000_0000 ~ 0x4FFF_FFFF
 * [imx6sx,imx7d platforms]0x8000_0000 ~ 0xBFFF_FFFF
 *
 */
static u32 ddr_test_region = 0, test_region_size = SZ_2M;

struct imx6_pcie {
	struct dw_pcie		*pci;
	u32 			ext_osc;
	u32			ctrl_id;
	u32			cpu_base;
	int			clkreq_gpio;
	int			dis_gpio;
	int			power_on_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie;
	struct clk		*pcie_ext;
	struct clk		*pcie_ext_src;
	struct regmap		*iomuxc_gpr;
	enum imx6_pcie_variants variant;
	u32			hsio_cfg;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	int			link_gen;
	struct regulator	*vpcie;
	struct regmap		*reg_src;
	struct regmap		*reg_gpc;
	void __iomem		*phy_base;
	struct regulator	*pcie_phy_regulator;
	struct regulator	*pcie_bus_regulator;
};

/* Parameters for the waiting for PCIe PHY PLL to lock on i.MX7 */
#define PHY_PLL_LOCK_WAIT_MAX_RETRIES	2000
#define PHY_PLL_LOCK_WAIT_USLEEP_MIN	50
#define PHY_PLL_LOCK_WAIT_USLEEP_MAX	200

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

#define PCIE_RC_LCSR				0x80

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PL_PFLR (PL_OFFSET + 0x08)
#define PCIE_PL_PFLR_LINK_STATE_MASK		(0x3f << 16)
#define PCIE_PL_PFLR_FORCE_LINK			(1 << 15)
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING	(1 << 29)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_UP		(1 << 4)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)

#define PCIE_MISC_CTRL			(PL_OFFSET + 0x1BC)
#define PCIE_MISC_DBI_RO_WR_EN		BIT(0)

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D
#define PCIE_PHY_RX_ASIC_OUT_VALID	(1 << 0)

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

#define SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO 0x0011
/* FIELD: RES_ACK_IN_OVRD [15:15]
 * FIELD: RES_ACK_IN [14:14]
 * FIELD: RES_REQ_IN_OVRD [13:13]
 * FIELD: RES_REQ_IN [12:12]
 * FIELD: RTUNE_REQ_OVRD [11:11]
 * FIELD: RTUNE_REQ [10:10]
 * FIELD: MPLL_MULTIPLIER_OVRD [9:9]
 * FIELD: MPLL_MULTIPLIER [8:2]
 * FIELD: MPLL_EN_OVRD [1:1]
 * FIELD: MPLL_EN [0:0]
 */

#define SSP_CR_SUP_DIG_ATEOVRD 0x0010
/* FIELD: ateovrd_en [2:2]
 * FIELD: ref_usb2_en [1:1]
 * FIELD: ref_clkdiv2 [0:0]
 */

/* iMX7 PCIe PHY registers */
#define PCIE_PHY_CMN_REG15	0x54
#define PCIE_PHY_CMN_REG15_DLY_4	(1 << 2)
#define PCIE_PHY_CMN_REG15_PLL_PD	(1 << 5)
#define PCIE_PHY_CMN_REG15_OVRD_PLL_PD	(1 << 7)

/* iMX8 HSIO registers */
#define IMX8QM_LPCG_PHYX2_OFFSET		0x00000
#define IMX8QM_LPCG_PHYX1_OFFSET		0x10000
#define IMX8QM_CSR_PHYX2_OFFSET			0x90000
#define IMX8QM_CSR_PHYX1_OFFSET			0xA0000
#define IMX8QM_CSR_PHYX_STTS0_OFFSET		0x4
#define IMX8QM_CSR_PCIEA_OFFSET			0xB0000
#define IMX8QM_CSR_PCIEB_OFFSET			0xC0000
#define IMX8QM_CSR_PCIE_CTRL1_OFFSET		0x4
#define IMX8QM_CSR_PCIE_CTRL2_OFFSET		0x8
#define IMX8QM_CSR_PCIE_STTS0_OFFSET		0xC
#define IMX8QM_CSR_MISC_OFFSET			0xE0000

#define IMX8QM_LPCG_PHY_PCG0			BIT(1)
#define IMX8QM_LPCG_PHY_PCG1			BIT(5)

#define IMX8QM_CTRL_LTSSM_ENABLE		BIT(4)
#define IMX8QM_CTRL_BUTTON_RST_N		BIT(21)
#define IMX8QM_CTRL_PERST_N			BIT(22)
#define IMX8QM_CTRL_POWER_UP_RST_N		BIT(23)

#define IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST	BIT(19)
#define IMX8QM_STTS0_LANE0_TX_PLL_LOCK		BIT(4)
#define IMX8QM_STTS0_LANE1_TX_PLL_LOCK		BIT(12)

#define IMX8QM_PCIE_TYPE_MASK			(0xF << 24)

#define IMX8QM_PHYX2_CTRL0_APB_MASK		0x3
#define IMX8QM_PHY_APB_RSTN_0			BIT(0)
#define IMX8QM_PHY_APB_RSTN_1			BIT(1)

#define IMX8QM_MISC_IOB_RXENA			BIT(0)
#define IMX8QM_MISC_IOB_TXENA			BIT(1)
#define IMX8QM_CSR_MISC_IOB_A_0_TXOE		BIT(2)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK	(0x3 << 3)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_2		BIT(4)
#define IMX8QM_MISC_PHYX1_EPCS_SEL		BIT(12)
#define IMX8QM_MISC_PCIE_AB_SELECT		BIT(13)

#define IMX8MQ_SRC_PCIEPHY_RCR_OFFSET		0x2C
#define IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET		0x48
#define IMX8MQ_PCIEPHY_PWR_ON_RST		BIT(0)
#define IMX8MQ_PCIEPHY_G_RST			BIT(1)
#define IMX8MQ_PCIEPHY_BTN			BIT(2)
#define IMX8MQ_PCIEPHY_PERST			BIT(3)
#define IMX8MQ_PCIE_CTRL_APPS_EN		BIT(6)
#define IMX8MQ_PCIE_CTRL_APPS_TURNOFF		BIT(11)

#define IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET	0xEC
#define IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET	0xF8
#define IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET	0x104
#define IMX8MQ_GPC_PGC_PCIE_CTRL_OFFSET		0xC40
#define IMX8MQ_GPC_PGC_PCIE2_CTRL_OFFSET	0xF00
#define IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN		BIT(3)
#define IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ	BIT(1)
#define IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET		12
#define IMX8MQ_GPC_PCG_PCIE_CTRL_PCR		BIT(0)
#define IMX8MQ_GPR_PCIE_REF_USE_PAD		BIT(9)

static int pcie_phy_poll_ack(struct imx6_pcie *imx6_pcie, int exp_val)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(struct imx6_pcie *imx6_pcie, int addr)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	return pcie_phy_poll_ack(imx6_pcie, 0);
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(struct imx6_pcie *imx6_pcie, int addr, int *data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, phy_ctl);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x00);

	return pcie_phy_poll_ack(imx6_pcie, 0);
}

static int pcie_phy_write(struct imx6_pcie *imx6_pcie, int addr, int data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack */
	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, 0);
	if (ret)
		return ret;

	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x0);

	return 0;
}

static void imx6_pcie_reset_phy(struct imx6_pcie *imx6_pcie)
{
	u32 tmp;

	if (imx6_pcie->variant == IMX6Q || imx6_pcie->variant == IMX6SX
	    || imx6_pcie->variant == IMX6QP) {
		pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
		tmp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
			PHY_RX_OVRD_IN_LO_RX_PLL_EN);
		pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);

		usleep_range(2000, 3000);

		pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
		tmp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
			  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
		pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);
	}
}

#ifdef CONFIG_ARM
/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);
	unsigned long instr = *(unsigned long *)pc;
	int reg = (instr >> 12) & 15;

	/*
	 * If the instruction being executed was a read,
	 * make it look like it read all-ones.
	 */
	if ((instr & 0x0c100000) == 0x04100000) {
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
		regs->ARM_pc += 4;
		return 0;
	}

	if ((instr & 0x0e100090) == 0x00100090) {
		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
		return 0;
	}

	return 1;
}
#endif

static void imx6_pcie_assert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct device *dev = imx6_pcie->pci->dev;
	u32 val;

	switch (imx6_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN);
		/* Force PCIe PHY reset */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST,
				   IMX6Q_GPR1_PCIE_SW_RST);
		break;
	case IMX6Q:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 1 << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 0 << 16);
		break;
	case IMX7D:
		/* G_RST */
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(1), BIT(1));
		/* BTNRST */
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(2), BIT(2));
		break;
	case IMX8QM:
		val = IMX8QM_CSR_PCIEA_OFFSET + imx6_pcie->ctrl_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_BUTTON_RST_N,
				IMX8QM_CTRL_BUTTON_RST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PERST_N,
				IMX8QM_CTRL_PERST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_POWER_UP_RST_N,
				IMX8QM_CTRL_POWER_UP_RST_N);
		break;
	case IMX8MQ:
		if (imx6_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		/* Do RSTs */
		regmap_update_bits(imx6_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_BTN,
				IMX8MQ_PCIEPHY_BTN);
		regmap_update_bits(imx6_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_G_RST,
				IMX8MQ_PCIEPHY_G_RST);
	}

	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		int ret = regulator_disable(imx6_pcie->vpcie);

		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static int imx6_pcie_enable_ref_clk(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret = 0;

	switch (imx6_pcie->variant) {
	case IMX6SX:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN, 0);
		break;
	case IMX6QP: 		/* FALLTHROUGH */
	case IMX6Q:
		/* power up core phy and enable ref clock */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
		/*
		 * the async reset input need ref clock to sync internally,
		 * when the ref clock comes after reset, internal synced
		 * reset time is too short, cannot meet the requirement.
		 * add one ~10us delay here.
		 */
		udelay(10);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
		break;
	case IMX7D:
	case IMX8MQ:
		break;
	case IMX8QM:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}
		break;
	}

	return ret;
}

static int imx7d_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	unsigned int retries;
	struct device *dev = imx6_pcie->pci->dev;

	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES; retries++) {
		regmap_read(imx6_pcie->iomuxc_gpr, IOMUXC_GPR22, &val);

		if (val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED)
			return 0;

		usleep_range(PHY_PLL_LOCK_WAIT_USLEEP_MIN,
			     PHY_PLL_LOCK_WAIT_USLEEP_MAX);
	}

	dev_err(dev, "PCIe PLL lock timeout\n");
	return -ENODEV;
}

static int imx8_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val, tmp, orig;
	unsigned int retries;

	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES; retries++) {
		if (imx6_pcie->hsio_cfg == PCIEAX2SATA) {
			regmap_read(imx6_pcie->iomuxc_gpr,
				    IMX8QM_CSR_PHYX2_OFFSET + 0x4,
				    &tmp);
			orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			orig |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			tmp &= orig;
			if (tmp == orig) {
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_LPCG_PHYX2_OFFSET,
					IMX8QM_LPCG_PHY_PCG0
					| IMX8QM_LPCG_PHY_PCG1,
					IMX8QM_LPCG_PHY_PCG0
					| IMX8QM_LPCG_PHY_PCG1);
				break;
			}
		}

		if (imx6_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
			regmap_read(imx6_pcie->iomuxc_gpr,
				    IMX8QM_CSR_PHYX2_OFFSET + 0x4,
				    &tmp);
			tmp &= IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			if (tmp == IMX8QM_STTS0_LANE0_TX_PLL_LOCK) {
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_LPCG_PHYX2_OFFSET,
					IMX8QM_LPCG_PHY_PCG0,
					IMX8QM_LPCG_PHY_PCG0);
				break;
			}
		}

		if (imx6_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
			val = IMX8QM_CSR_PHYX2_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_read(imx6_pcie->iomuxc_gpr,
				    val + IMX8QM_CSR_PHYX_STTS0_OFFSET,
				    &tmp);
			orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			if (imx6_pcie->ctrl_id == 0) /* pciea 2 lanes */
				orig |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			tmp &= orig;
			if (tmp == orig) {
				val = IMX8QM_CSR_PHYX2_OFFSET
					+ imx6_pcie->ctrl_id * SZ_64K;
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val, IMX8QM_LPCG_PHY_PCG0,
					IMX8QM_LPCG_PHY_PCG0);
				break;
			}
		}
		udelay(10);
	}

	if (retries >= PHY_PLL_LOCK_WAIT_MAX_RETRIES) {
		pr_info("pcie phy pll can't be locked.\n");
		return -ENODEV;
	} else {
		return 0;
	}
}

static int imx6_pcie_deassert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret;
	u32 val;

	if (imx6_pcie->vpcie && !regulator_is_enabled(imx6_pcie->vpcie)) {
		ret = regulator_enable(imx6_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return ret;
		}
	}

	if (gpio_is_valid(imx6_pcie->power_on_gpio))
		gpio_set_value_cansleep(imx6_pcie->power_on_gpio, 1);

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(imx6_pcie->pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie clock\n");
		goto err_pcie;
	}

	if (imx6_pcie->ext_osc && (imx6_pcie->variant != IMX8QM)) {
		clk_set_parent(imx6_pcie->pcie_ext,
				imx6_pcie->pcie_ext_src);
		ret = clk_prepare_enable(imx6_pcie->pcie_ext);
		if (ret) {
			dev_err(dev, "unable to enable pcie_ext clock\n");
			goto err_pcie_bus;
		}
	} else {
		ret = clk_prepare_enable(imx6_pcie->pcie_bus);
		if (ret) {
			dev_err(dev, "unable to enable pcie_bus clock\n");
			goto err_pcie_bus;
		}
	}

	ret = clk_prepare_enable(imx6_pcie->pcie_phy);
	if (ret) {
		dev_err(dev, "unable to enable pcie_phy clock\n");
		goto err_pcie_phy;
	}

	ret = imx6_pcie_enable_ref_clk(imx6_pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie ref clock\n");
		goto err_ref_clk;
	}

	/* allow the clocks to stabilize */
	udelay(200);

	switch (imx6_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		udelay(200);

		/* Configure the PHY when 100Mhz external OSC is used as input clock */
		if (!imx6_pcie->ext_osc)
			break;

		mdelay(4);
		pcie_phy_read(imx6_pcie, SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO, &val);
		/* MPLL_MULTIPLIER [8:2] */
		val &= ~(0x7F << 2);
		val |= (0x19 << 2);
		/* MPLL_MULTIPLIER_OVRD [9:9] */
		val |= (0x1 << 9);
		pcie_phy_write(imx6_pcie, SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO, val);
		mdelay(4);

		pcie_phy_read(imx6_pcie, SSP_CR_SUP_DIG_ATEOVRD, &val);
		/* ref_clkdiv2 [0:0] */
		val &= ~0x1;
		/* ateovrd_en [2:2] */
		val |=  0x4;
		pcie_phy_write(imx6_pcie, SSP_CR_SUP_DIG_ATEOVRD, val);
		mdelay(4);

		break;
	case IMX6Q:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);
		/*
		 * some delay are required by 6qp, after the SW_RST is
		 * cleared, before access the cfg register.
		 */
		udelay(200);
		break;
	case IMX7D:
		/* wait for more than 10us to release phy g_rst and btnrst */
		udelay(10);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), 0);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(1), 0);

		/* Add the workaround for ERR010728 */
		if (unlikely(imx6_pcie->phy_base == NULL)) {
			pr_err("phy base shouldn't be null.\n");
		} else {
			writel(PCIE_PHY_CMN_REG15_DLY_4,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG15);
			writel(PCIE_PHY_CMN_REG15_DLY_4
			       | PCIE_PHY_CMN_REG15_PLL_PD
			       | PCIE_PHY_CMN_REG15_OVRD_PLL_PD,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG15);
			writel(PCIE_PHY_CMN_REG15_DLY_4,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG15);
		}

		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(2), 0);

		/* wait for phy pll lock firstly. */
		if (imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie))
			ret = -ENODEV;
		break;
	case IMX8QM:
		/* wait for phy pll lock firstly. */
		if (imx8_pcie_wait_for_phy_pll_lock(imx6_pcie))
			ret = -ENODEV;
		break;
	case IMX8MQ:
		/* wait for more than 10us to release phy g_rst and btnrst */
		udelay(10);
		if (imx6_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx6_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_BTN, 0);
		regmap_update_bits(imx6_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_G_RST, 0);
		regmap_update_bits(imx6_pcie->reg_src, val,
				IMX8MQ_PCIE_CTRL_APPS_EN, 0);
		break;

	}

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					imx6_pcie->gpio_active_high);
		mdelay(20);
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					!imx6_pcie->gpio_active_high);
		mdelay(20);
	}

	return ret;

err_ref_clk:
	clk_disable_unprepare(imx6_pcie->pcie_phy);
err_pcie_phy:
	if (!imx6_pcie->ext_osc || (imx6_pcie->variant == IMX8QM))
		clk_disable_unprepare(imx6_pcie->pcie_bus);
err_pcie_bus:
	clk_disable_unprepare(imx6_pcie->pcie);
err_pcie:
	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		ret = regulator_disable(imx6_pcie->vpcie);
		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
	return ret;
}

static void imx6_pcie_phy_pwr_up(struct imx6_pcie *imx6_pcie)
{
	u32 val, offset;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	if (imx6_pcie->variant != IMX8MQ)
		return;
	/*
	 * Power up PHY.
	 * pcie phy ref clock select by gpr configuration.
	 * 1? external osc : internal pll
	 */

	if (imx6_pcie->ctrl_id == 0)
		offset = 0;
	else
		offset = IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET;

	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset);
	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset);

	regmap_read(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			&val);
	while (val & (IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset)) {
		regmap_read(imx6_pcie->reg_gpc,
				IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
				&val);
		if (time_after(jiffies, timeout)) {
			pr_err("CAN NOT PWR UP PCIE%d PHY!\n",
					imx6_pcie->ctrl_id);
			break;
		}
	}
	udelay(1);
}

static void imx6_pcie_phy_pwr_dn(struct imx6_pcie *imx6_pcie)
{
	u32 val, offset;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	if (imx6_pcie->variant != IMX8MQ)
		return;
	/*
	 * Power up PHY.
	 * pcie phy ref clock select by gpr configuration.
	 * 1? external osc : internal pll
	 */

	if (imx6_pcie->ctrl_id == 0) {
		offset = 0;
		regmap_update_bits(imx6_pcie->reg_gpc,
				IMX8MQ_GPC_PGC_PCIE_CTRL_OFFSET,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR);
	} else {
		offset = IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET;
		regmap_update_bits(imx6_pcie->reg_gpc,
				IMX8MQ_GPC_PGC_PCIE2_CTRL_OFFSET,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR);
	}

	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset);
	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset);

	regmap_read(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
			&val);
	while (val & (IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset)) {
		regmap_read(imx6_pcie->reg_gpc,
				IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
				&val);
		if (time_after(jiffies, timeout)) {
			pr_err("CAN NOT PWR DN PCIE%d PHY!\n",
					imx6_pcie->ctrl_id);
			break;
		}
	}
	udelay(1);
}

static void imx6_pcie_init_phy(struct imx6_pcie *imx6_pcie)
{
	u32 tmp, val;
	int i, ret;

	if (imx6_pcie->variant == IMX8QM) {
		switch (imx6_pcie->hsio_cfg) {
		case PCIEAX2SATA:
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 1.
			 * bit13 phy_ab_select 0.
			 */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				0);
			break;

		case PCIEAX1PCIEBX1SATA:
			if (imx6_pcie->ctrl_id)
				tmp = IMX8QM_PHY_APB_RSTN_1;
			else
				tmp = IMX8QM_PHY_APB_RSTN_0;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK, tmp);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
			break;

		case PCIEAX2PCIEBX1:
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 0.
			 * bit13 phy_ab_select 1.
			 */
			if (imx6_pcie->ctrl_id)
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX1_OFFSET,
					IMX8QM_PHY_APB_RSTN_0,
					IMX8QM_PHY_APB_RSTN_0);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX2_OFFSET,
					IMX8QM_PHYX2_CTRL0_APB_MASK,
					IMX8QM_PHY_APB_RSTN_0
					| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
			break;
		}

		if (imx6_pcie->ext_osc) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				IMX8QM_MISC_IOB_RXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				0);
		} else {
			/* Try to used the internal pll as ref clk */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				IMX8QM_MISC_IOB_TXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_2);
		}

		/* bit19 PM_REQ_CORE_RST of pciex2_stts0 should be cleared. */
		for (i = 0; i < 100; i++) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_read(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&tmp);
			if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) == 0)
				break;
			udelay(10);
		}

		if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) != 0)
			pr_err("ERROR PM_REQ_CORE_RST is still set.\n");
	} else if (imx6_pcie->variant == IMX8MQ) {
		imx6_pcie_phy_pwr_up(imx6_pcie);

		if (imx6_pcie->ctrl_id == 0)
			val = IOMUXC_GPR14;
		else
			val = IOMUXC_GPR16;

		regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
				IMX8MQ_GPR_PCIE_REF_USE_PAD,
				IMX8MQ_GPR_PCIE_REF_USE_PAD);
	} else if (imx6_pcie->variant == IMX7D) {
		/* Enable PCIe PHY 1P0D */
		regulator_set_voltage(imx6_pcie->pcie_phy_regulator,
				1000000, 1000000);
		ret = regulator_enable(imx6_pcie->pcie_phy_regulator);
		if (ret)
			dev_err(imx6_pcie->pci->dev,
				"failed to enable pcie regulator\n");

		/* pcie phy ref clock select; 1? internal pll : external osc */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL, 0);
	} else if (imx6_pcie->variant == IMX6SX) {
		/* Force PCIe PHY reset */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				IMX6SX_GPR5_PCIE_BTNRST_RESET,
				IMX6SX_GPR5_PCIE_BTNRST_RESET);

		regulator_set_voltage(imx6_pcie->pcie_phy_regulator,
				1100000, 1100000);
		ret = regulator_enable(imx6_pcie->pcie_phy_regulator);
		if (ret)
			dev_err(imx6_pcie->pci->dev,
				"failed to enable pcie regulator.\n");
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);
	}

	if (imx6_pcie->pcie_bus_regulator != NULL) {
		ret = regulator_enable(imx6_pcie->pcie_bus_regulator);
		if (ret)
			dev_err(imx6_pcie->pci->dev, "failed to enable pcie regulator.\n");
	}

	if ((imx6_pcie->variant == IMX6Q) || (imx6_pcie->variant == IMX6QP)
					  || (imx6_pcie->variant == IMX6SX)) {
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, IMX6Q_GPR12_LOS_LEVEL_9);

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN1,
				   imx6_pcie->tx_deemph_gen1 << 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB,
				   imx6_pcie->tx_deemph_gen2_3p5db << 6);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB,
				   imx6_pcie->tx_deemph_gen2_6db << 12);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_FULL,
				   imx6_pcie->tx_swing_full << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_LOW,
				   imx6_pcie->tx_swing_low << 25);
	}

	/* configure the device type */
	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		if (imx6_pcie->variant == IMX8QM) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ENDPOINT << 24);
		} else {
			if (unlikely(imx6_pcie->ctrl_id))
				/* iMX8MQ second PCIE */
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE >> 4,
						PCI_EXP_TYPE_ENDPOINT << 8);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE,
						PCI_EXP_TYPE_ENDPOINT << 12);
		}
	} else {
		if (imx6_pcie->variant == IMX8QM) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ROOT_PORT << 24);
		} else {
			if (unlikely(imx6_pcie->ctrl_id))
				/* iMX8MQ second PCIE */
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE >> 4,
						PCI_EXP_TYPE_ROOT_PORT << 8);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE,
						PCI_EXP_TYPE_ROOT_PORT << 12);
		}
	}
}

static int imx6_pcie_wait_for_link(struct imx6_pcie *imx6_pcie)
{
	int count = 20000;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	/* check if the link is up or not */
	while (!dw_pcie_link_up(pci)) {
		udelay(10);
		if (--count)
			continue;

		dev_err(dev, "phy link never came up\n");
		dev_dbg(dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
			dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R0),
			dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1));
		return -ETIMEDOUT;
	}

	return 0;
}

static int imx6_pcie_wait_for_speed_change(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		udelay(100);
	}

	dev_err(dev, "Speed change timeout\n");
	return -EINVAL;
}

static irqreturn_t imx6_pcie_msi_handler(int irq, void *arg)
{
	struct imx6_pcie *imx6_pcie = arg;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	return dw_handle_msi_irq(pp);
}

static int imx6_pcie_establish_link(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	int ret;

	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	if (!IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
		tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
		tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
		tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
		dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);
	}

	/* Start LTSSM. */
	if (imx6_pcie->variant == IMX8QM) {
		/* Bit4 of the CTRL2 */
		tmp = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->ctrl_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				tmp + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE,
				IMX8QM_CTRL_LTSSM_ENABLE);
	} else if ((imx6_pcie->variant == IMX7D)
			|| (imx6_pcie->variant == IMX8MQ)) {
		if (imx6_pcie->ctrl_id == 0)
			tmp = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			tmp = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx6_pcie->reg_src, tmp,
				IMX8MQ_PCIE_CTRL_APPS_EN,
				IMX8MQ_PCIE_CTRL_APPS_EN);
	} else {
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);
	}

	ret = imx6_pcie_wait_for_link(imx6_pcie);
	if (ret)
		goto err_reset_phy;

	if (imx6_pcie->link_gen == 2) {
		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
		tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
		tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
		dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		if (imx6_pcie->variant != IMX7D) {
			/*
			 * On i.MX7, DIRECT_SPEED_CHANGE behaves differently
			 * from i.MX6 family when no link speed transition
			 * occurs and we go Gen1 -> yep, Gen1. The difference
			 * is that, in such case, it will not be cleared by HW
			 * which will cause the following code to report false
			 * failure.
			 */

			ret = imx6_pcie_wait_for_speed_change(imx6_pcie);
			if (ret) {
				dev_info(dev, "Roll back to GEN1 link!\n");
			}
		}

		/* Make sure link training is finished as well! */
		ret = imx6_pcie_wait_for_link(imx6_pcie);
		if (ret) {
			dev_err(dev, "Failed to bring link up!\n");
			goto err_reset_phy;
		}
	} else {
		dev_info(dev, "Link: Gen2 disabled\n");
	}

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCSR);
	dev_info(dev, "Link up, Gen%i\n", (tmp >> 16) & 0xf);
	return 0;

err_reset_phy:
	dev_dbg(dev, "PHY DEBUG_R0=0x%08x DEBUG_R1=0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R0),
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1));
	imx6_pcie_reset_phy(imx6_pcie);

	if (!IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
		clk_disable_unprepare(imx6_pcie->pcie);
		if (!imx6_pcie->ext_osc || (imx6_pcie->variant == IMX8QM))
			clk_disable_unprepare(imx6_pcie->pcie_bus);
		clk_disable_unprepare(imx6_pcie->pcie_phy);
		if (imx6_pcie->variant == IMX6SX
		    || imx6_pcie->variant == IMX8QM)
			clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		release_bus_freq(BUS_FREQ_HIGH);
		if ((imx6_pcie->variant == IMX7D)
				|| (imx6_pcie->variant == IMX8QM))
			pm_runtime_put_sync(pci->dev);
		if (imx6_pcie->variant == IMX8MQ)
			imx6_pcie_phy_pwr_dn(imx6_pcie);
		if (imx6_pcie->pcie_phy_regulator != NULL)
			regulator_disable(imx6_pcie->pcie_phy_regulator);
		if (imx6_pcie->pcie_bus_regulator != NULL)
			regulator_disable(imx6_pcie->pcie_bus_regulator);
	}

	return ret;
}

static int imx6_pcie_host_init(struct pcie_port *pp)
{
	int ret;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	/* enable disp_mix power domain */
	if ((imx6_pcie->variant == IMX7D) || (imx6_pcie->variant == IMX8QM))
		pm_runtime_get_sync(pci->dev);

	imx6_pcie_assert_core_reset(imx6_pcie);
	imx6_pcie_init_phy(imx6_pcie);
	ret = imx6_pcie_deassert_core_reset(imx6_pcie);
	if (ret < 0)
		return ret;

	/* set up the cpu address offset */
	if (imx6_pcie->cpu_base)
		pp->cpu_addr_offset = imx6_pcie->cpu_base - pp->mem_base;
	else
		pp->cpu_addr_offset = 0;

	if (imx6_pcie->variant == IMX8QM) {
		if (dw_pcie_readl_dbi(pci, PCIE_MISC_CTRL) == 0)
			dw_pcie_writel_dbi(pci, PCIE_MISC_CTRL,
					PCIE_MISC_DBI_RO_WR_EN);
	}

	dw_pcie_setup_rc(pp);
	ret = imx6_pcie_establish_link(imx6_pcie);
	if (ret < 0)
		return ret;

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	return 0;
}

static int imx6_pcie_link_up(struct dw_pcie *pci)
{
	return dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1) &
			PCIE_PHY_DEBUG_R1_XMLH_LINK_UP;
}

static const struct dw_pcie_host_ops imx6_pcie_host_ops = {
	.host_init = imx6_pcie_host_init,
};

static int imx6_add_pcie_port(struct imx6_pcie *imx6_pcie,
			      struct platform_device *pdev)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq <= 0) {
			dev_err(dev, "failed to get MSI irq\n");
			return -ENODEV;
		}

		ret = devm_request_irq(dev, pp->msi_irq,
				       imx6_pcie_msi_handler,
				       IRQF_SHARED | IRQF_NO_THREAD,
				       "mx6-pcie-msi", imx6_pcie);
		if (ret) {
			dev_err(dev, "failed to request MSI irq\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &imx6_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = imx6_pcie_link_up,
};

static ssize_t imx_pcie_bar0_addr_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;

	return sprintf(buf, "imx-pcie-bar0-addr-info start 0x%08x\n",
			readl(pci->dbi_base + PCI_BASE_ADDRESS_0));
}

static ssize_t imx_pcie_bar0_addr_start(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 bar_start;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;

	sscanf(buf, "%x\n", &bar_start);
	writel(bar_start, pci->dbi_base + PCI_BASE_ADDRESS_0);

	return count;
}

static void imx_pcie_regions_setup(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	if (imx6_pcie->variant == IMX7D && ddr_test_region == 0)
		ddr_test_region = 0xb0000000;
	else if (imx6_pcie->variant == IMX6SX && ddr_test_region == 0)
		ddr_test_region = 0xb0000000;
	else if (ddr_test_region == 0)
		ddr_test_region = 0x40000000;

	/*
	 * region2 outbound used to access rc/ep mem
	 * in imx6 pcie ep/rc validation system
	 */
	writel(2, pci->dbi_base + 0x900);
	writel((u32)pp->mem_base, pci->dbi_base + 0x90c);
	writel(0, pci->dbi_base + 0x910);
	writel((u32)pp->mem_base + test_region_size, pci->dbi_base + 0x914);

	writel(ddr_test_region, pci->dbi_base + 0x918);
	writel(0, pci->dbi_base + 0x91c);
	writel(0, pci->dbi_base + 0x904);
	writel(1 << 31, pci->dbi_base + 0x908);
}

static ssize_t imx_pcie_memw_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "imx-pcie-rc-memw-info start 0x%08x, size 0x%08x\n",
			ddr_test_region, test_region_size);
}

static ssize_t
imx_pcie_memw_start(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_start;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	sscanf(buf, "%x\n", &memw_start);

	if (imx6_pcie->variant == IMX7D || imx6_pcie->variant == IMX6SX) {
		if (memw_start < 0x80000000 || memw_start > 0xb0000000) {
			dev_err(dev, "Invalid memory start addr.\n");
			dev_info(dev, "e.x: echo 0xb0000000 > /sys/...");
			return -1;
		}
	} else {
		if (memw_start < 0x10000000 || memw_start > 0x40000000) {
			dev_err(dev, "Invalid imx6q sd memory start addr.\n");
			dev_info(dev, "e.x: echo 0x30000000 > /sys/...");
			return -1;
		}
	}

	if (ddr_test_region != memw_start) {
		ddr_test_region = memw_start;
		imx_pcie_regions_setup(dev);
	}

	return count;
}

static ssize_t
imx_pcie_memw_size(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	u32 memw_size;

	sscanf(buf, "%x\n", &memw_size);

	if ((memw_size > (SZ_16M - SZ_1M)) || (memw_size < SZ_64K)) {
		dev_err(dev, "Invalid, should be [SZ_64K,SZ_16M - SZ_1MB].\n");
		dev_info(dev, "For example: echo 0x200000 > /sys/...");
		return -1;
	}

	if (test_region_size != memw_size) {
		test_region_size = memw_size;
		imx_pcie_regions_setup(dev);
	}

	return count;
}

static DEVICE_ATTR(memw_info, S_IRUGO, imx_pcie_memw_info, NULL);
static DEVICE_ATTR(memw_start_set, S_IWUSR, NULL, imx_pcie_memw_start);
static DEVICE_ATTR(memw_size_set, S_IWUSR, NULL, imx_pcie_memw_size);
static DEVICE_ATTR(ep_bar0_addr, S_IWUSR | S_IRUGO, imx_pcie_bar0_addr_info,
		imx_pcie_bar0_addr_start);

static struct attribute *imx_pcie_attrs[] = {
	/*
	 * The start address, and the limitation (64KB ~ (16MB - 1MB))
	 * of the ddr mem window reserved by RC, and used for EP to access.
	 * BTW, these attrs are only configured at EP side.
	 */
	&dev_attr_memw_info.attr,
	&dev_attr_memw_start_set.attr,
	&dev_attr_memw_size_set.attr,
	&dev_attr_ep_bar0_addr.attr,
	NULL
};

static struct attribute_group imx_pcie_attrgroup = {
	.attrs	= imx_pcie_attrs,
};

static void imx6_pcie_setup_ep(struct dw_pcie *pci)
{
	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	writel(readl(pci->dbi_base + PCI_COMMAND)
			| PCI_COMMAND_IO
			| PCI_COMMAND_MEMORY
			| PCI_COMMAND_MASTER,
			pci->dbi_base + PCI_COMMAND);

	/*
	 * configure the class_rev(emaluate one memory ram ep device),
	 * bar0 and bar1 of ep
	 */
	writel(0xdeadbeaf, pci->dbi_base + PCI_VENDOR_ID);
	writel(readl(pci->dbi_base + PCI_CLASS_REVISION)
			| (PCI_CLASS_MEMORY_RAM	<< 16),
			pci->dbi_base + PCI_CLASS_REVISION);
	writel(0xdeadbeaf, pci->dbi_base
			+ PCI_SUBSYSTEM_VENDOR_ID);

	/* 32bit none-prefetchable 8M bytes memory on bar0 */
	writel(0x0, pci->dbi_base + PCI_BASE_ADDRESS_0);
	writel(SZ_8M - 1, pci->dbi_base + (1 << 12)
			+ PCI_BASE_ADDRESS_0);

	/* None used bar1 */
	writel(0x0, pci->dbi_base + PCI_BASE_ADDRESS_1);
	writel(0, pci->dbi_base + (1 << 12) + PCI_BASE_ADDRESS_1);

	/* 4K bytes IO on bar2 */
	writel(0x1, pci->dbi_base + PCI_BASE_ADDRESS_2);
	writel(SZ_4K - 1, pci->dbi_base + (1 << 12) +
			PCI_BASE_ADDRESS_2);

	/*
	 * 32bit prefetchable 1M bytes memory on bar3
	 * FIXME BAR MASK3 is not changable, the size
	 * is fixed to 256 bytes.
	 */
	writel(0x8, pci->dbi_base + PCI_BASE_ADDRESS_3);
	writel(SZ_1M - 1, pci->dbi_base + (1 << 12)
			+ PCI_BASE_ADDRESS_3);

	/*
	 * 64bit prefetchable 1M bytes memory on bar4-5.
	 * FIXME BAR4,5 are not enabled yet
	 */
	writel(0xc, pci->dbi_base + PCI_BASE_ADDRESS_4);
	writel(SZ_1M - 1, pci->dbi_base + (1 << 12)
			+ PCI_BASE_ADDRESS_4);
	writel(0, pci->dbi_base + (1 << 12) + PCI_BASE_ADDRESS_5);
}

#ifdef CONFIG_PM_SLEEP
/* PM_TURN_OFF */
static void pci_imx_pm_turn_off(struct imx6_pcie *imx6_pcie)
{
	/* PM_TURN_OFF */
	if (imx6_pcie->variant == IMX7D) {
		regmap_update_bits(imx6_pcie->reg_src, 0x2c,
				BIT(11), BIT(11));
		regmap_update_bits(imx6_pcie->reg_src, 0x2c,
				BIT(11), 0);
	} else if (imx6_pcie->variant == IMX6SX) {
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
	} else if (imx6_pcie->variant == IMX6QP) {
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF, 0);
	} else {
		pr_info("Info: don't support pm_turn_off yet.\n");
		return;
	}

	udelay(1000);
	if (gpio_is_valid(imx6_pcie->reset_gpio))
		gpio_set_value_cansleep(imx6_pcie->reset_gpio, 0);
}

static int pci_imx_suspend_noirq(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pci->pp;

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_cfg_store(pp);

	pci_imx_pm_turn_off(imx6_pcie);

	if (imx6_pcie->variant == IMX7D || imx6_pcie->variant == IMX6SX ||
	    imx6_pcie->variant == IMX6QP || imx6_pcie->variant == IMX8QM) {
		/* Disable clks */
		clk_disable_unprepare(imx6_pcie->pcie);
		clk_disable_unprepare(imx6_pcie->pcie_phy);
		if (!imx6_pcie->ext_osc || (imx6_pcie->variant == IMX8QM))
			clk_disable_unprepare(imx6_pcie->pcie_bus);
		if (imx6_pcie->variant == IMX6SX
		    || imx6_pcie->variant == IMX8QM)
			clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		else if (imx6_pcie->variant == IMX7D)
			/* turn off external osc input */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					BIT(5), BIT(5));
		else if (imx6_pcie->variant == IMX6QP) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
					IMX6Q_GPR1_PCIE_REF_CLK_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
					IMX6Q_GPR1_PCIE_TEST_PD,
					IMX6Q_GPR1_PCIE_TEST_PD);
		}
		release_bus_freq(BUS_FREQ_HIGH);

		/* Power down PCIe PHY. */
		if (imx6_pcie->pcie_phy_regulator != NULL)
			regulator_disable(imx6_pcie->pcie_phy_regulator);
		if (imx6_pcie->pcie_bus_regulator != NULL)
			regulator_disable(imx6_pcie->pcie_bus_regulator);
		if (gpio_is_valid(imx6_pcie->power_on_gpio))
			gpio_set_value_cansleep(imx6_pcie->power_on_gpio, 0);
	} else {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
	}

	return 0;
}

static int pci_imx_resume_noirq(struct device *dev)
{
	u32 val;
	int ret = 0;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pci->pp;

	if (imx6_pcie->variant == IMX7D || imx6_pcie->variant == IMX6SX ||
	    imx6_pcie->variant == IMX6QP || imx6_pcie->variant == IMX8QM) {
		if (imx6_pcie->variant == IMX8QM) {
			/* Bit4 of the CTRL2 */
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_LTSSM_ENABLE, 0);
		} else if (imx6_pcie->variant == IMX7D) {
			regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), 0);
		} else {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					IMX6Q_GPR12_PCIE_CTL_2, 0);
		}

		imx6_pcie_assert_core_reset(imx6_pcie);

		imx6_pcie_init_phy(imx6_pcie);

		imx6_pcie_deassert_core_reset(imx6_pcie);

		/*
		 * controller maybe turn off, re-configure again
		 */
		dw_pcie_setup_rc(pp);

		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_cfg_restore(pp);

		if (imx6_pcie->variant == IMX8QM) {
			/* wait for phy pll lock firstly. */
			imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);

			/* Bit4 of the CTRL2 */
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_LTSSM_ENABLE,
					IMX8QM_CTRL_LTSSM_ENABLE);
		} else if (imx6_pcie->variant == IMX7D) {
			/* wait for phy pll lock firstly. */
			imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
			regmap_update_bits(imx6_pcie->reg_src, 0x2c,
					BIT(6), BIT(6));
		} else {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					IMX6Q_GPR12_PCIE_CTL_2,
					IMX6Q_GPR12_PCIE_CTL_2);
		}

		ret = imx6_pcie_wait_for_link(imx6_pcie);
		if (ret < 0)
			pr_info("pcie link is down after resume.\n");
	} else {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD, 0);
	}

	return 0;
}

static const struct dev_pm_ops pci_imx_pm_ops = {
	.suspend_noirq = pci_imx_suspend_noirq,
	.resume_noirq = pci_imx_resume_noirq,
	.freeze_noirq = pci_imx_suspend_noirq,
	.thaw_noirq = pci_imx_resume_noirq,
	.poweroff_noirq = pci_imx_suspend_noirq,
	.restore_noirq = pci_imx_resume_noirq,
};
#endif

static int __init imx6_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct imx6_pcie *imx6_pcie;
	struct device_node *np;
	struct resource *res;
	struct device_node *node = dev->of_node;
	int ret;

	imx6_pcie = devm_kzalloc(dev, sizeof(*imx6_pcie), GFP_KERNEL);
	if (!imx6_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	imx6_pcie->pci = pci;
	imx6_pcie->variant =
		(enum imx6_pcie_variants)of_device_get_match_data(dev);

	if (of_property_read_u32(node, "hsio-cfg", &imx6_pcie->hsio_cfg))
		imx6_pcie->hsio_cfg = 0;

	if (of_property_read_u32(node, "ctrl-id", &imx6_pcie->ctrl_id))
		imx6_pcie->ctrl_id = 0;

	if (of_property_read_u32(node, "cpu-base-addr", &imx6_pcie->cpu_base))
		imx6_pcie->cpu_base = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx-pcie-phy");
	if (np != NULL) {
		imx6_pcie->phy_base = of_iomap(np, 0);
		WARN_ON(!imx6_pcie->phy_base);
	} else {
		imx6_pcie->phy_base = NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res)
		pci->dbi_base = devm_ioremap_resource(dev, res);
	else
		dev_err(dev, "missing *dbi* reg space\n");
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	/* Fetch GPIOs */
	imx6_pcie->clkreq_gpio = of_get_named_gpio(node, "clkreq-gpio", 0);
	if (gpio_is_valid(imx6_pcie->clkreq_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->clkreq_gpio,
					    GPIOF_OUT_INIT_LOW, "PCIe CLKREQ");
		if (ret) {
			dev_err(&pdev->dev, "unable to get clkreq gpio\n");
			return ret;
		}
	}

	imx6_pcie->dis_gpio = of_get_named_gpio(node, "disable-gpio", 0);
	if (gpio_is_valid(imx6_pcie->dis_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->dis_gpio,
					    GPIOF_OUT_INIT_HIGH, "PCIe DIS");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable gpio\n");
			return ret;
		}
	}

	imx6_pcie->power_on_gpio = of_get_named_gpio(node, "power-on-gpio", 0);
	if (gpio_is_valid(imx6_pcie->power_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					    imx6_pcie->power_on_gpio,
					    GPIOF_OUT_INIT_LOW,
					    "PCIe power enable");
		if (ret) {
			dev_err(&pdev->dev, "unable to get power-on gpio\n");
			return ret;
		}
	}

	imx6_pcie->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	imx6_pcie->gpio_active_high = of_property_read_bool(node,
						"reset-gpio-active-high");
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, imx6_pcie->reset_gpio,
				imx6_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	} else if (imx6_pcie->reset_gpio == -EPROBE_DEFER) {
		return imx6_pcie->reset_gpio;
	}

	/* Fetch clocks */
	imx6_pcie->pcie_phy = devm_clk_get(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pcie_phy)) {
		dev_err(dev, "pcie_phy clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_phy);
	}

	imx6_pcie->pcie_bus = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(imx6_pcie->pcie_bus)) {
		dev_err(dev, "pcie_bus clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_bus);
	}

	if (of_property_read_u32(node, "ext_osc", &imx6_pcie->ext_osc) < 0)
		imx6_pcie->ext_osc = 0;

	if (imx6_pcie->ext_osc && (imx6_pcie->variant != IMX8QM)) {
		imx6_pcie->pcie_ext = devm_clk_get(&pdev->dev, "pcie_ext");
		if (IS_ERR(imx6_pcie->pcie_ext)) {
			dev_err(&pdev->dev,
				"pcie_ext clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_ext);
		}

		imx6_pcie->pcie_ext_src = devm_clk_get(&pdev->dev,
				"pcie_ext_src");
		if (IS_ERR(imx6_pcie->pcie_ext_src)) {
			dev_err(&pdev->dev,
				"pcie_ext_src clk src missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_ext_src);
		}
	}

	imx6_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx6_pcie->pcie)) {
		dev_err(dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie);
	}

	if (imx6_pcie->variant == IMX6QP) {
		imx6_pcie->pcie_bus_regulator = devm_regulator_get(dev,
				"pcie-bus");
		if (IS_ERR(imx6_pcie->pcie_bus_regulator))
			imx6_pcie->pcie_bus_regulator = NULL;
	} else {
		imx6_pcie->pcie_bus_regulator = NULL;
	}

	/* Grab GPR config register range */
	if (imx6_pcie->variant == IMX7D) {
		imx6_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx7d-iomuxc-gpr");
		imx6_pcie->reg_src =
			 syscon_regmap_lookup_by_compatible("fsl,imx7d-src");
		if (IS_ERR(imx6_pcie->reg_src)) {
			dev_err(&pdev->dev,
				"imx7d pcie phy src missing or invalid\n");
			return PTR_ERR(imx6_pcie->reg_src);
		}
		imx6_pcie->pcie_phy_regulator = devm_regulator_get(&pdev->dev,
				"pcie-phy");
	} else if (imx6_pcie->variant == IMX8MQ) {
		imx6_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx8mq-iomuxc-gpr");
		imx6_pcie->reg_src =
			 syscon_regmap_lookup_by_compatible("fsl,imx8mq-src");
		if (IS_ERR(imx6_pcie->reg_src)) {
			dev_err(&pdev->dev,
				"imx8mq pcie phy src missing or invalid\n");
			return PTR_ERR(imx6_pcie->reg_src);
		}
		imx6_pcie->reg_gpc =
			 syscon_regmap_lookup_by_compatible("fsl,imx8mq-gpc");
		if (IS_ERR(imx6_pcie->reg_gpc)) {
			dev_err(&pdev->dev,
				"imx8mq pcie phy src missing or invalid\n");
			return PTR_ERR(imx6_pcie->reg_gpc);
		}
	} else if (imx6_pcie->variant == IMX6SX) {
		imx6_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}

		imx6_pcie->pcie_phy_regulator = devm_regulator_get(&pdev->dev,
				"pcie-phy");

		imx6_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx6sx-iomuxc-gpr");
	} else if (imx6_pcie->variant == IMX8QM) {
		imx6_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_phandle(node, "hsio");
		imx6_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}
	} else {
		imx6_pcie->iomuxc_gpr =
		 syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	}

	if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
		dev_err(dev, "unable to find iomuxc registers\n");
		return PTR_ERR(imx6_pcie->iomuxc_gpr);
	}

	/* Grab PCIe PHY Tx Settings */
	if (of_property_read_u32(node, "fsl,tx-deemph-gen1",
				 &imx6_pcie->tx_deemph_gen1))
		imx6_pcie->tx_deemph_gen1 = 20;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx6_pcie->tx_deemph_gen2_3p5db))
		imx6_pcie->tx_deemph_gen2_3p5db = 20;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx6_pcie->tx_deemph_gen2_6db))
		imx6_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx6_pcie->tx_swing_full))
		imx6_pcie->tx_swing_full = 115;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx6_pcie->tx_swing_low))
		imx6_pcie->tx_swing_low = 115;

	/* Limit link speed */
	ret = of_property_read_u32(node, "fsl,max-link-speed",
				   &imx6_pcie->link_gen);
	if (ret)
		imx6_pcie->link_gen = 1;

	imx6_pcie->vpcie = devm_regulator_get_optional(&pdev->dev, "vpcie");
	if (IS_ERR(imx6_pcie->vpcie)) {
		if (PTR_ERR(imx6_pcie->vpcie) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		imx6_pcie->vpcie = NULL;
	}

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		int i;
		void *test_reg1, *test_reg2;
		void __iomem *pcie_arb_base_addr;
		struct timeval tv1s, tv1e, tv2s, tv2e;
		u32 val, tv_count1, tv_count2;
		struct device_node *np = node;
		struct pcie_port *pp = &pci->pp;
		LIST_HEAD(res);
		struct resource_entry *win, *tmp;

		/* add attributes for device */
		ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
		if (ret)
			return -EINVAL;

		ret = of_pci_get_host_bridge_resources(np, 0, 0xff, &res,
						       &pp->io_base);
		if (ret)
			return ret;

		ret = devm_request_pci_bus_resources(&pdev->dev, &res);
		if (ret) {
			dev_err(dev, "missing ranges property\n");
			pci_free_resource_list(&res);
			return ret;
		}

		/* Get the I/O and memory ranges from DT */
		resource_list_for_each_entry_safe(win, tmp, &res) {
			switch (resource_type(win->res)) {
			case IORESOURCE_MEM:
				pp->mem = win->res;
				pp->mem->name = "MEM";
				pp->mem_size = resource_size(pp->mem);
				pp->mem_bus_addr = pp->mem->start - win->offset;
				break;
			}
		}

		pp->mem_base = pp->mem->start;
		pp->ops = &imx6_pcie_host_ops;

		/* enable disp_mix power domain */
		if (imx6_pcie->variant == IMX7D)
			pm_runtime_get_sync(dev);

		imx6_pcie_assert_core_reset(imx6_pcie);
		imx6_pcie_init_phy(imx6_pcie);
		imx6_pcie_deassert_core_reset(imx6_pcie);

		/*
		 * iMX6SX PCIe has the stand-alone power domain.
		 * refer to the initialization for iMX6SX PCIe,
		 * release the PCIe PHY reset here,
		 * before LTSSM enable is set
		 * .
		 */
		if (imx6_pcie->variant == IMX6SX)
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
					BIT(19), 0 << 19);

		/* assert LTSSM enable */
		if (imx6_pcie->variant == IMX8QM) {
			/* Bit4 of the CTRL2 */
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx6_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_LTSSM_ENABLE,
					IMX8QM_CTRL_LTSSM_ENABLE);
		} else if (imx6_pcie->variant == IMX7D) {
			regmap_update_bits(imx6_pcie->reg_src, 0x2c,
					BIT(6), BIT(6));
		} else {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
					IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);
		}

		dev_info(&pdev->dev, "PCIe EP: waiting for link up...\n");

		platform_set_drvdata(pdev, imx6_pcie);
		/* link is indicated by the bit4 of DB_R1 register */
		do {
			usleep_range(10, 20);
		} while ((readl(pci->dbi_base + PCIE_PHY_DEBUG_R1) & 0x10) == 0);

		imx6_pcie_setup_ep(pci);

		imx_pcie_regions_setup(&pdev->dev);

		/* self io test */
		test_reg1 = devm_kzalloc(&pdev->dev,
				test_region_size, GFP_KERNEL);
		if (!test_reg1) {
			pr_err("pcie ep: can't alloc the test reg1.\n");
			ret = PTR_ERR(test_reg1);
			return ret;
		}

		test_reg2 = devm_kzalloc(&pdev->dev,
				test_region_size, GFP_KERNEL);
		if (!test_reg2) {
			pr_err("pcie ep: can't alloc the test reg2.\n");
			ret = PTR_ERR(test_reg1);
			return ret;
		}

		/*
		 * FIXME when the ddr_test_region is mapped as cache-able,
		 * system hang when read the ddr memory content back from rc
		 * reserved ddr memory after write the ddr_test_region
		 * content to rc.
		 */
		pcie_arb_base_addr = ioremap_nocache(pp->mem_base,
					test_region_size);

		if (!pcie_arb_base_addr) {
			pr_err("error with ioremap in ep selftest\n");
			ret = PTR_ERR(pcie_arb_base_addr);
			return ret;
		}

		for (i = 0; i < test_region_size; i = i + 4) {
			writel(0xE6600D00 + i, test_reg1 + i);
			writel(0xDEADBEAF, test_reg2 + i);
		}

		/* PCIe EP start the data transfer after link up */
		pr_info("pcie ep: Starting data transfer...\n");
		do_gettimeofday(&tv1s);

		memcpy((unsigned int *)pcie_arb_base_addr,
				(unsigned int *)test_reg1,
				test_region_size);

		do_gettimeofday(&tv1e);

		do_gettimeofday(&tv2s);

		memcpy((unsigned int *)test_reg2,
				(unsigned int *)pcie_arb_base_addr,
				test_region_size);

		do_gettimeofday(&tv2e);
		if (memcmp(test_reg2, test_reg1, test_region_size) == 0) {
			tv_count1 = (tv1e.tv_sec - tv1s.tv_sec)
				* USEC_PER_SEC
				+ tv1e.tv_usec - tv1s.tv_usec;
			tv_count2 = (tv2e.tv_sec - tv2s.tv_sec)
				* USEC_PER_SEC
				+ tv2e.tv_usec - tv2s.tv_usec;

			pr_info("pcie ep: Data transfer is successful."
					" tv_count1 %dus,"
					" tv_count2 %dus.\n",
					tv_count1, tv_count2);
			pr_info("pcie ep: Data write speed:%ldMB/s.\n",
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count1));
			pr_info("pcie ep: Data read speed:%ldMB/s.\n",
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count2));
		} else {
			pr_info("pcie ep: Data transfer is failed.\n");
		} /* end of self io test. */
	} else {
		platform_set_drvdata(pdev, imx6_pcie);

		ret = imx6_add_pcie_port(imx6_pcie, pdev);
		if (ret < 0)
			return ret;

		if (IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS))
			imx_pcie_regions_setup(&pdev->dev);
	}
	return 0;
}

static void imx6_pcie_shutdown(struct platform_device *pdev)
{
	struct imx6_pcie *imx6_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	if (imx6_pcie->variant != IMX8QM)
		imx6_pcie_assert_core_reset(imx6_pcie);
}

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = (void *)IMX6Q,  },
	{ .compatible = "fsl,imx6sx-pcie", .data = (void *)IMX6SX, },
	{ .compatible = "fsl,imx6qp-pcie", .data = (void *)IMX6QP, },
	{ .compatible = "fsl,imx7d-pcie",  .data = (void *)IMX7D,  },
	{ .compatible = "fsl,imx8qm-pcie", .data = (void *)IMX8QM, },
	{ .compatible = "fsl,imx8mq-pcie", .data = (void *)IMX8MQ, },
	{},
};

static struct platform_driver imx6_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.of_match_table = imx6_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &pci_imx_pm_ops,
	},
	.probe    = imx6_pcie_probe,
	.shutdown = imx6_pcie_shutdown,
};

static int __init imx6_pcie_init(void)
{
#ifdef CONFIG_ARM
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx6q_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx6q_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
#endif

	return platform_driver_register(&imx6_pcie_driver);
}
device_initcall(imx6_pcie_init);
