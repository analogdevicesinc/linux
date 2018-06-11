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
#include <linux/of_irq.h>
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

#define to_imx_pcie(x)	dev_get_drvdata((x)->dev)

enum imx_pcie_variants {
	IMX6Q,
	IMX6SX,
	IMX6QP,
	IMX7D,
	IMX8QM,
	IMX8QXP,
	IMX8MQ,
	IMX8MM,
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
static bool dma_w_end, dma_r_end, dma_en;

struct imx_pcie {
	struct dw_pcie		*pci;
	u32 			ext_osc;
	u32			ctrl_id;
	u32			cpu_base;
	u32			hard_wired;
	int			clkreq_gpio;
	int			dis_gpio;
	int			power_on_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie_per;
	struct clk		*pcie;
	struct clk		*pcie_ext_src;
	struct regmap		*iomuxc_gpr;
	enum imx_pcie_variants variant;
	u32			hsio_cfg;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	u32			dma_unroll_offset;
	int			link_gen;
	struct regulator	*vpcie;
	struct regmap		*reg_src;
	struct regmap		*reg_gpc;
	void __iomem		*phy_base;
	struct regulator	*pcie_phy_regulator;
	struct regulator	*pcie_bus_regulator;
	struct regulator	*epdev_on;
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
#define PCIE_PORT_LINK_CONTROL		0x710
#define PORT_LINK_MODE_MASK		(0x3f << 16)
#define PORT_LINK_MODE_1_LANES		(0x1 << 16)
#define PORT_LINK_MODE_2_LANES		(0x3 << 16)

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
#define PORT_LOGIC_LINK_WIDTH_MASK	(0x1f << 8)
#define PORT_LOGIC_LINK_WIDTH_1_LANES	(0x1 << 8)
#define PORT_LOGIC_LINK_WIDTH_2_LANES	(0x2 << 8)

#define PCIE_MISC_CTRL			(PL_OFFSET + 0x1BC)
#define PCIE_MISC_DBI_RO_WR_EN		BIT(0)

#define PCIE_ATU_VIEWPORT		0x900

/* DMA registers */
#define MAX_PCIE_DMA_CHANNELS	8
#define DMA_UNROLL_CDM_OFFSET	(0x7 << 19)
#define DMA_REG_OFFSET		0x970
#define DMA_CTRL_VIEWPORT_OFF	(DMA_REG_OFFSET + 0x8)
#define DMA_WRITE_ENGINE_EN_OFF	(DMA_REG_OFFSET + 0xC)
#define DMA_WRITE_ENGINE_EN	BIT(0)
#define DMA_WRITE_DOORBELL	(DMA_REG_OFFSET + 0x10)
#define DMA_READ_ENGINE_EN_OFF	(DMA_REG_OFFSET + 0x2C)
#define DMA_READ_ENGINE_EN	BIT(0)
#define DMA_READ_DOORBELL	(DMA_REG_OFFSET + 0x30)
#define DMA_WRITE_INT_STS	(DMA_REG_OFFSET + 0x4C)
#define DMA_WRITE_INT_MASK	(DMA_REG_OFFSET + 0x54)
#define DMA_WRITE_INT_CLR	(DMA_REG_OFFSET + 0x58)
#define DMA_READ_INT_STS	(DMA_REG_OFFSET + 0xA0)
#define DMA_READ_INT_MASK	(DMA_REG_OFFSET + 0xA8)
#define DMA_READ_INT_CLR	(DMA_REG_OFFSET + 0xAC)
#define DMA_DONE_INT_STS	0xFF
#define DMA_ABORT_INT_STS	(0xFF << 16)
#define DMA_VIEWPOT_SEL_OFF	(DMA_REG_OFFSET + 0xFC)
#define DMA_CHANNEL_CTRL_1	(DMA_REG_OFFSET + 0x100)
#define DMA_CHANNEL_CTRL_1_LIE	BIT(3)
#define DMA_CHANNEL_CTRL_2	(DMA_REG_OFFSET + 0x104)
#define DMA_TRANSFER_SIZE	(DMA_REG_OFFSET + 0x108)
#define DMA_SAR_LOW		(DMA_REG_OFFSET + 0x10C)
#define DMA_SAR_HIGH		(DMA_REG_OFFSET + 0x110)
#define DMA_DAR_LOW		(DMA_REG_OFFSET + 0x114)
#define DMA_DAR_HIGH		(DMA_REG_OFFSET + 0x118)

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
#define PCIE_PHY_CMN_REG4		0x14
#define PCIE_PHY_CMN_REG4_DCC_FB_EN	(0x29)

#define PCIE_PHY_CMN_REG24		0x90
#define PCIE_PHY_CMN_REG24_RX_EQ	BIT(6)
#define PCIE_PHY_CMN_REG24_RX_EQ_SEL	BIT(3)

#define PCIE_PHY_CMN_REG26		0x98
#define PCIE_PHY_CMN_REG26_ATT_MODE	0xBC

#define PCIE_PHY_CMN_REG62			0x188
#define PCIE_PHY_CMN_REG62_PLL_CLK_OUT		0x08
#define PCIE_PHY_CMN_REG64			0x190
#define PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM	0x8C
#define PCIE_PHY_CMN_REG75			0x1D4
#define PCIE_PHY_CMN_REG75_PLL_DONE		0x3

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
#define IMX8QM_CTRL_READY_ENTR_L23		BIT(5)
#define IMX8QM_CTRL_PM_XMT_TURNOFF		BIT(9)
#define IMX8QM_CTRL_BUTTON_RST_N		BIT(21)
#define IMX8QM_CTRL_PERST_N			BIT(22)
#define IMX8QM_CTRL_POWER_UP_RST_N		BIT(23)

#define IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2	BIT(13)
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
#define IMX8MQ_PCIEPHY_DOMAIN_EN		(BIT(31) | (0xF << 24))
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
#define IMX8MM_GPR_PCIE_REF_CLK_SEL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_PLL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_EXT		(0x2 << 24)
#define IMX8MM_GPR_PCIE_AUX_EN			BIT(19)
#define IMX8MM_GPR_PCIE_CMN_RST			BIT(18)
#define IMX8MM_GPR_PCIE_POWER_OFF		BIT(17)
#define IMX8MM_GPR_PCIE_SSC_EN			BIT(16)

static int pcie_phy_poll_ack(struct imx_pcie *imx_pcie, int exp_val)
{
	struct dw_pcie *pci = imx_pcie->pci;
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

static int pcie_phy_wait_ack(struct imx_pcie *imx_pcie, int addr)
{
	struct dw_pcie *pci = imx_pcie->pci;
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	ret = pcie_phy_poll_ack(imx_pcie, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	return pcie_phy_poll_ack(imx_pcie, 0);
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(struct imx_pcie *imx_pcie, int addr, int *data)
{
	struct dw_pcie *pci = imx_pcie->pci;
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(imx_pcie, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, phy_ctl);

	ret = pcie_phy_poll_ack(imx_pcie, 1);
	if (ret)
		return ret;

	val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x00);

	return pcie_phy_poll_ack(imx_pcie, 0);
}

static int pcie_phy_write(struct imx_pcie *imx_pcie, int addr, int data)
{
	struct dw_pcie *pci = imx_pcie->pci;
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(imx_pcie, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	ret = pcie_phy_poll_ack(imx_pcie, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx_pcie, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack */
	ret = pcie_phy_poll_ack(imx_pcie, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx_pcie, 0);
	if (ret)
		return ret;

	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x0);

	return 0;
}

static void imx_pcie_reset_phy(struct imx_pcie *imx_pcie)
{
	u32 tmp;

	if (imx_pcie->variant == IMX6Q || imx_pcie->variant == IMX6SX
	    || imx_pcie->variant == IMX6QP) {
		pcie_phy_read(imx_pcie, PHY_RX_OVRD_IN_LO, &tmp);
		tmp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
			PHY_RX_OVRD_IN_LO_RX_PLL_EN);
		pcie_phy_write(imx_pcie, PHY_RX_OVRD_IN_LO, tmp);

		usleep_range(2000, 3000);

		pcie_phy_read(imx_pcie, PHY_RX_OVRD_IN_LO, &tmp);
		tmp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
			  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
		pcie_phy_write(imx_pcie, PHY_RX_OVRD_IN_LO, tmp);
	}
}

#ifdef CONFIG_ARM
/*  Added for PCI abort handling */
static int imx_pcie_abort_handler(unsigned long addr,
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

static void imx_pcie_assert_core_reset(struct imx_pcie *imx_pcie)
{
	struct device *dev = imx_pcie->pci->dev;
	u32 val;
	int i;

	switch (imx_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN);
		/* Force PCIe PHY reset */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET);
		break;
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST,
				   IMX6Q_GPR1_PCIE_SW_RST);
		break;
	case IMX6Q:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 1 << 18);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 0 << 16);
		break;
	case IMX7D:
		/* G_RST */
		regmap_update_bits(imx_pcie->reg_src, 0x2c, BIT(1), BIT(1));
		/* BTNRST */
		regmap_update_bits(imx_pcie->reg_src, 0x2c, BIT(2), BIT(2));
		break;
	case IMX8QXP:
			val = IMX8QM_CSR_PCIEB_OFFSET;
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_BUTTON_RST_N,
					IMX8QM_CTRL_BUTTON_RST_N);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_PERST_N,
					IMX8QM_CTRL_PERST_N);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_POWER_UP_RST_N,
					IMX8QM_CTRL_POWER_UP_RST_N);
		break;
	case IMX8QM:
		for (i = 0; i <= imx_pcie->ctrl_id; i++) {
			val = IMX8QM_CSR_PCIEA_OFFSET + i * SZ_64K;
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_BUTTON_RST_N,
					IMX8QM_CTRL_BUTTON_RST_N);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_PERST_N,
					IMX8QM_CTRL_PERST_N);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_POWER_UP_RST_N,
					IMX8QM_CTRL_POWER_UP_RST_N);
		}
		break;
	case IMX8MQ:
	case IMX8MM:
		if (imx_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		/* Do RSTs */
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_BTN | IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_BTN | IMX8MQ_PCIEPHY_DOMAIN_EN);
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_G_RST |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_G_RST |
				IMX8MQ_PCIEPHY_DOMAIN_EN);
	}

	if (imx_pcie->vpcie && regulator_is_enabled(imx_pcie->vpcie) > 0) {
		int ret = regulator_disable(imx_pcie->vpcie);

		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static int imx_pcie_enable_ref_clk(struct imx_pcie *imx_pcie)
{
	struct dw_pcie *pci = imx_pcie->pci;
	struct device *dev = pci->dev;
	int ret = 0;

	switch (imx_pcie->variant) {
	case IMX6SX:
		ret = clk_prepare_enable(imx_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}

		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN, 0);
		break;
	case IMX6QP: 		/* FALLTHROUGH */
	case IMX6Q:
		/* power up core phy and enable ref clock */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
		/*
		 * the async reset input need ref clock to sync internally,
		 * when the ref clock comes after reset, internal synced
		 * reset time is too short, cannot meet the requirement.
		 * add one ~10us delay here.
		 */
		udelay(10);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		break;
	case IMX8QXP:
	case IMX8QM:
		ret = clk_prepare_enable(imx_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}
		ret = clk_prepare_enable(imx_pcie->pcie_per);
		if (ret) {
			dev_err(dev, "unable to enable pcie_per clock\n");
			clk_disable_unprepare(imx_pcie->pcie_inbound_axi);
			break;
		}

		break;
	}

	return ret;
}

static int imx7d_pcie_wait_for_phy_pll_lock(struct imx_pcie *imx_pcie)
{
	u32 val;
	unsigned int retries;
	struct device *dev = imx_pcie->pci->dev;

	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES; retries++) {
		regmap_read(imx_pcie->iomuxc_gpr, IOMUXC_GPR22, &val);

		if (val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED)
			return 0;

		usleep_range(PHY_PLL_LOCK_WAIT_USLEEP_MIN,
			     PHY_PLL_LOCK_WAIT_USLEEP_MAX);
	}

	dev_err(dev, "PCIe PLL lock timeout\n");
	return -ENODEV;
}

static int imx8_pcie_wait_for_phy_pll_lock(struct imx_pcie *imx_pcie)
{
	u32 val, tmp, orig;
	unsigned int retries;
	struct dw_pcie *pci = imx_pcie->pci;
	struct device *dev = pci->dev;

	if (imx_pcie->variant == IMX8MM) {
		tmp = readl(imx_pcie->phy_base + PCIE_PHY_CMN_REG75);
		for (retries = 0; retries < 100; retries++) {
			if (tmp == PCIE_PHY_CMN_REG75_PLL_DONE)
				break;
			udelay(10);
		}
	} else if (imx_pcie->variant == IMX8QXP
			|| imx_pcie->variant == IMX8QM) {
		for (retries = 0; retries < 100; retries++) {
			if (imx_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
				regmap_read(imx_pcie->iomuxc_gpr,
					    IMX8QM_CSR_PHYX2_OFFSET + 0x4,
					    &tmp);
				if (imx_pcie->ctrl_id == 0) /* pciea 1 lanes */
					orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
				else /* pcieb 1 lanes */
					orig = IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
				tmp &= orig;
				if (tmp == orig) {
					regmap_update_bits(imx_pcie->iomuxc_gpr,
						IMX8QM_LPCG_PHYX2_OFFSET,
						IMX8QM_LPCG_PHY_PCG0
						| IMX8QM_LPCG_PHY_PCG1,
						IMX8QM_LPCG_PHY_PCG0
						| IMX8QM_LPCG_PHY_PCG1);
					break;
				}
			}

			if (imx_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
				val = IMX8QM_CSR_PHYX2_OFFSET
					+ imx_pcie->ctrl_id * SZ_64K;
				regmap_read(imx_pcie->iomuxc_gpr,
					    val + IMX8QM_CSR_PHYX_STTS0_OFFSET,
					    &tmp);
				orig = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
				if (imx_pcie->ctrl_id == 0) /* pciea 2 lanes */
					orig |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
				tmp &= orig;
				if (tmp == orig) {
					val = IMX8QM_CSR_PHYX2_OFFSET
						+ imx_pcie->ctrl_id * SZ_64K;
					regmap_update_bits(imx_pcie->iomuxc_gpr,
						val, IMX8QM_LPCG_PHY_PCG0,
						IMX8QM_LPCG_PHY_PCG0);
					break;
				}
			}
			udelay(10);
		}
	}

	if (retries >= PHY_PLL_LOCK_WAIT_MAX_RETRIES) {
		dev_info(dev, "pcie phy pll can't be locked.\n");
		return -ENODEV;
	} else {
		dev_info(dev, "pcie phy pll is locked.\n");
		return 0;
	}
}

static int imx_pcie_deassert_core_reset(struct imx_pcie *imx_pcie)
{
	struct dw_pcie *pci = imx_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = pci->dev;
	int ret, i;
	u32 val, tmp;

	if (imx_pcie->vpcie && !regulator_is_enabled(imx_pcie->vpcie)) {
		ret = regulator_enable(imx_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return ret;
		}
	}

	if (gpio_is_valid(imx_pcie->power_on_gpio))
		gpio_set_value_cansleep(imx_pcie->power_on_gpio, 1);

	ret = clk_prepare_enable(imx_pcie->pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie clock\n");
		goto err_pcie;
	}

	if (imx_pcie->ext_osc && (imx_pcie->variant == IMX6QP))
		clk_set_parent(imx_pcie->pcie_bus,
				imx_pcie->pcie_ext_src);
	ret = clk_prepare_enable(imx_pcie->pcie_bus);
	if (ret) {
		dev_err(dev, "unable to enable pcie_bus clock\n");
		goto err_pcie_bus;
	}

	ret = clk_prepare_enable(imx_pcie->pcie_phy);
	if (ret) {
		dev_err(dev, "unable to enable pcie_phy clock\n");
		goto err_pcie_phy;
	}

	ret = imx_pcie_enable_ref_clk(imx_pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie ref clock\n");
		goto err_ref_clk;
	}

	/* allow the clocks to stabilize */
	udelay(200);

	switch (imx_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		udelay(200);

		/* Configure the PHY when 100Mhz external OSC is used as input clock */
		if (!imx_pcie->ext_osc)
			break;

		mdelay(4);
		pcie_phy_read(imx_pcie, SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO, &val);
		/* MPLL_MULTIPLIER [8:2] */
		val &= ~(0x7F << 2);
		val |= (0x19 << 2);
		/* MPLL_MULTIPLIER_OVRD [9:9] */
		val |= (0x1 << 9);
		pcie_phy_write(imx_pcie, SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO, val);
		mdelay(4);

		pcie_phy_read(imx_pcie, SSP_CR_SUP_DIG_ATEOVRD, &val);
		/* ref_clkdiv2 [0:0] */
		val &= ~0x1;
		/* ateovrd_en [2:2] */
		val |=  0x4;
		pcie_phy_write(imx_pcie, SSP_CR_SUP_DIG_ATEOVRD, val);
		mdelay(4);

		break;
	case IMX6Q:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
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
		regmap_update_bits(imx_pcie->reg_src, 0x2c, BIT(6), 0);
		regmap_update_bits(imx_pcie->reg_src, 0x2c, BIT(1), 0);

		/* Add the workaround for ERR010728 */
		if (unlikely(imx_pcie->phy_base == NULL)) {
			dev_err(dev, "phy base shouldn't be null.\n");
		} else {
			/* De-assert DCC_FB_EN by writing data "0x29". */
			writel(PCIE_PHY_CMN_REG4_DCC_FB_EN,
			       imx_pcie->phy_base + PCIE_PHY_CMN_REG4);
			/* Assert RX_EQS and RX_EQS_SEL */
			writel(PCIE_PHY_CMN_REG24_RX_EQ_SEL
				| PCIE_PHY_CMN_REG24_RX_EQ,
			       imx_pcie->phy_base + PCIE_PHY_CMN_REG24);
			/* Assert ATT_MODE by writing data "0xBC". */
			writel(PCIE_PHY_CMN_REG26_ATT_MODE,
			       imx_pcie->phy_base + PCIE_PHY_CMN_REG26);
		}

		regmap_update_bits(imx_pcie->reg_src, 0x2c, BIT(2), 0);

		/* wait for phy pll lock firstly. */
		if (imx7d_pcie_wait_for_phy_pll_lock(imx_pcie))
			ret = -ENODEV;
		break;
	case IMX8QXP:
	case IMX8QM:
		/* bit19 PM_REQ_CORE_RST of pciex#_stts0 should be cleared. */
		for (i = 0; i < 100; i++) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx_pcie->ctrl_id * SZ_64K;
			regmap_read(imx_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&tmp);
			if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) == 0)
				break;
			udelay(10);
		}

		if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) != 0)
			dev_err(dev, "ERROR PM_REQ_CORE_RST is still set.\n");

		/* wait for phy pll lock firstly. */
		if (imx8_pcie_wait_for_phy_pll_lock(imx_pcie)) {
			ret = -ENODEV;
			break;
		}

		/* set up the cpu address offset */
		if (imx_pcie->cpu_base)
			pp->cpu_addr_offset = imx_pcie->cpu_base
				- pp->mem_base;
		else
			pp->cpu_addr_offset = 0;

		if (dw_pcie_readl_dbi(pci, PCIE_MISC_CTRL) == 0)
			dw_pcie_writel_dbi(pci, PCIE_MISC_CTRL,
					PCIE_MISC_DBI_RO_WR_EN);
		break;
	case IMX8MM:
	case IMX8MQ:
		/* wait for more than 10us to release phy g_rst and btnrst */
		udelay(10);
		if (imx_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_BTN |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIEPHY_G_RST |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_DOMAIN_EN);

		udelay(100);
		/* wait for phy pll lock firstly. */
		if (imx8_pcie_wait_for_phy_pll_lock(imx_pcie)) {
			ret = -ENODEV;
			break;
		}

		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIE_CTRL_APPS_EN |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		break;
	}

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx_pcie->reset_gpio,
					imx_pcie->gpio_active_high);
		mdelay(20);
		gpio_set_value_cansleep(imx_pcie->reset_gpio,
					!imx_pcie->gpio_active_high);
		mdelay(20);
	}

	if (ret == 0)
		return ret;

err_ref_clk:
	clk_disable_unprepare(imx_pcie->pcie_phy);
err_pcie_phy:
	clk_disable_unprepare(imx_pcie->pcie_bus);
err_pcie_bus:
	clk_disable_unprepare(imx_pcie->pcie);
err_pcie:
	if (imx_pcie->vpcie && regulator_is_enabled(imx_pcie->vpcie) > 0) {
		ret = regulator_disable(imx_pcie->vpcie);
		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
	return ret;
}

static void imx_pcie_phy_pwr_up(struct imx_pcie *imx_pcie)
{
	u32 val, offset;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	struct device *dev = imx_pcie->pci->dev;

	if ((imx_pcie->variant != IMX8MQ) && (imx_pcie->variant != IMX8MM))
		return;
	/*
	 * Power up PHY.
	 * pcie phy ref clock select by gpr configuration.
	 * 1? external osc : internal pll
	 */

	if (imx_pcie->ctrl_id == 0)
		offset = 0;
	else
		offset = IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET;

	regmap_update_bits(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset);
	regmap_update_bits(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset);

	regmap_read(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			&val);
	while (val & (IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset)) {
		regmap_read(imx_pcie->reg_gpc,
				IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
				&val);
		if (time_after(jiffies, timeout)) {
			dev_err(dev, "CAN NOT PWR UP PCIE%d PHY!\n",
					imx_pcie->ctrl_id);
			break;
		}
	}
	udelay(1);
}

static void imx_pcie_phy_pwr_dn(struct imx_pcie *imx_pcie)
{
	u32 val, offset;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	struct device *dev = imx_pcie->pci->dev;

	if ((imx_pcie->variant != IMX8MQ) && (imx_pcie->variant != IMX8MM))
		return;
	/*
	 * Power up PHY.
	 * pcie phy ref clock select by gpr configuration.
	 * 1? external osc : internal pll
	 */

	if (imx_pcie->ctrl_id == 0) {
		offset = 0;
		regmap_update_bits(imx_pcie->reg_gpc,
				IMX8MQ_GPC_PGC_PCIE_CTRL_OFFSET,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR);
	} else {
		offset = IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET;
		regmap_update_bits(imx_pcie->reg_gpc,
				IMX8MQ_GPC_PGC_PCIE2_CTRL_OFFSET,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR,
				IMX8MQ_GPC_PCG_PCIE_CTRL_PCR);
	}

	regmap_update_bits(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset);
	regmap_update_bits(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset);

	regmap_read(imx_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
			&val);
	while (val & (IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset)) {
		regmap_read(imx_pcie->reg_gpc,
				IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET,
				&val);
		if (time_after(jiffies, timeout)) {
			dev_err(dev, "CAN NOT PWR DN PCIE%d PHY!\n",
					imx_pcie->ctrl_id);
			break;
		}
	}
	udelay(1);
}

static void imx_pcie_init_phy(struct imx_pcie *imx_pcie)
{
	u32 tmp, val;
	int ret;

	if (imx_pcie->variant == IMX8QM
			|| imx_pcie->variant == IMX8QXP) {
		switch (imx_pcie->hsio_cfg) {
		case PCIEAX2SATA:
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 1.
			 * bit13 phy_ab_select 0.
			 */
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				0);
			break;

		case PCIEAX1PCIEBX1SATA:
			tmp = IMX8QM_PHY_APB_RSTN_1;
			tmp |= IMX8QM_PHY_APB_RSTN_0;
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK, tmp);

			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
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
			if (imx_pcie->ctrl_id)
				regmap_update_bits(imx_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX1_OFFSET,
					IMX8QM_PHY_APB_RSTN_0,
					IMX8QM_PHY_APB_RSTN_0);
			else
				regmap_update_bits(imx_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX2_OFFSET,
					IMX8QM_PHYX2_CTRL0_APB_MASK,
					IMX8QM_PHY_APB_RSTN_0
					| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				0);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
			break;
		}

		if (imx_pcie->ext_osc) {
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				IMX8QM_MISC_IOB_RXENA);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				0);
		} else {
			/* Try to used the internal pll as ref clk */
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				0);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				IMX8QM_MISC_IOB_TXENA);
			regmap_update_bits(imx_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_2);
		}
	} else if (imx_pcie->variant == IMX8MQ || imx_pcie->variant == IMX8MM) {
		imx_pcie_phy_pwr_up(imx_pcie);

		if (imx_pcie->ctrl_id == 0)
			val = IOMUXC_GPR14;
		else
			val = IOMUXC_GPR16;

		if (imx_pcie->ext_osc) {
			regmap_update_bits(imx_pcie->iomuxc_gpr, val,
					IMX8MQ_GPR_PCIE_REF_USE_PAD,
					IMX8MQ_GPR_PCIE_REF_USE_PAD);
			if (imx_pcie->variant == IMX8MM) {
				dev_info(imx_pcie->pci->dev,
					"Initialize PHY with EXT REfCLK!.\n");
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MQ_GPR_PCIE_REF_USE_PAD,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_REF_CLK_SEL,
						IMX8MM_GPR_PCIE_REF_CLK_SEL);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_AUX_EN,
						IMX8MM_GPR_PCIE_AUX_EN);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_POWER_OFF,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_SSC_EN,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_REF_CLK_SEL,
						IMX8MM_GPR_PCIE_REF_CLK_EXT);
				udelay(100);
				/* Do the PHY common block reset */
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_CMN_RST,
						IMX8MM_GPR_PCIE_CMN_RST);
				udelay(200);
				dev_info(imx_pcie->pci->dev,
					"PHY Initialization End!.\n");
			}
		} else {
			if (imx_pcie->variant == IMX8MM) {
				/* Configure the internal PLL as REF clock */
				dev_info(imx_pcie->pci->dev,
					"Initialize PHY with PLL REfCLK!.\n");
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MQ_GPR_PCIE_REF_USE_PAD,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_REF_CLK_SEL,
						IMX8MM_GPR_PCIE_REF_CLK_SEL);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_AUX_EN,
						IMX8MM_GPR_PCIE_AUX_EN);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_POWER_OFF,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_SSC_EN,
						0);
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_REF_CLK_SEL,
						IMX8MM_GPR_PCIE_REF_CLK_PLL);
				udelay(100);
				/* Configure the PHY */
				writel(PCIE_PHY_CMN_REG62_PLL_CLK_OUT,
				       imx_pcie->phy_base + PCIE_PHY_CMN_REG62);
				writel(PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM,
				       imx_pcie->phy_base + PCIE_PHY_CMN_REG64);

				/* Do the PHY common block reset */
				regmap_update_bits(imx_pcie->iomuxc_gpr, val,
						IMX8MM_GPR_PCIE_CMN_RST,
						IMX8MM_GPR_PCIE_CMN_RST);
				udelay(200);
				dev_info(imx_pcie->pci->dev,
					"PHY Initialization End!.\n");
			} else {
				dev_err(imx_pcie->pci->dev,
					"Don't support internal PLL.\n");
			}
		}
	} else if (imx_pcie->variant == IMX7D) {
		/* Enable PCIe PHY 1P0D */
		regulator_set_voltage(imx_pcie->pcie_phy_regulator,
				1000000, 1000000);
		ret = regulator_enable(imx_pcie->pcie_phy_regulator);
		if (ret)
			dev_err(imx_pcie->pci->dev,
				"failed to enable pcie regulator\n");

		/* pcie phy ref clock select; 1? internal pll : external osc */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL, 0);
	} else if (imx_pcie->variant == IMX6SX) {
		/* Force PCIe PHY reset */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR5,
				IMX6SX_GPR5_PCIE_BTNRST_RESET,
				IMX6SX_GPR5_PCIE_BTNRST_RESET);

		regulator_set_voltage(imx_pcie->pcie_phy_regulator,
				1100000, 1100000);
		ret = regulator_enable(imx_pcie->pcie_phy_regulator);
		if (ret)
			dev_err(imx_pcie->pci->dev,
				"failed to enable pcie regulator.\n");
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);
	}

	if (imx_pcie->pcie_bus_regulator != NULL) {
		ret = regulator_enable(imx_pcie->pcie_bus_regulator);
		if (ret)
			dev_err(imx_pcie->pci->dev, "failed to enable pcie regulator.\n");
	}

	if ((imx_pcie->variant == IMX6Q) || (imx_pcie->variant == IMX6QP)
					  || (imx_pcie->variant == IMX6SX)) {
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, IMX6Q_GPR12_LOS_LEVEL_9);

		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN1,
				   imx_pcie->tx_deemph_gen1 << 0);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB,
				   imx_pcie->tx_deemph_gen2_3p5db << 6);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB,
				   imx_pcie->tx_deemph_gen2_6db << 12);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_FULL,
				   imx_pcie->tx_swing_full << 18);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_LOW,
				   imx_pcie->tx_swing_low << 25);
	}

	/* configure the device type */
	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		if (imx_pcie->variant == IMX8QM
				|| imx_pcie->variant == IMX8QXP) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ENDPOINT << 24);
		} else {
			if (unlikely(imx_pcie->ctrl_id))
				/* iMX8MQ second PCIE */
				regmap_update_bits(imx_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE >> 4,
						PCI_EXP_TYPE_ENDPOINT << 8);
			else
				regmap_update_bits(imx_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE,
						PCI_EXP_TYPE_ENDPOINT << 12);
		}
	} else {
		if (imx_pcie->variant == IMX8QM
				|| imx_pcie->variant == IMX8QXP) {
			val = IMX8QM_CSR_PCIEA_OFFSET
				+ imx_pcie->ctrl_id * SZ_64K;
			regmap_update_bits(imx_pcie->iomuxc_gpr,
					val, IMX8QM_PCIE_TYPE_MASK,
					PCI_EXP_TYPE_ROOT_PORT << 24);
		} else {
			if (unlikely(imx_pcie->ctrl_id))
				/* iMX8MQ second PCIE */
				regmap_update_bits(imx_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE >> 4,
						PCI_EXP_TYPE_ROOT_PORT << 8);
			else
				regmap_update_bits(imx_pcie->iomuxc_gpr,
						IOMUXC_GPR12,
						IMX6Q_GPR12_DEVICE_TYPE,
						PCI_EXP_TYPE_ROOT_PORT << 12);
		}
	}
}

static int imx_pcie_wait_for_link(struct imx_pcie *imx_pcie)
{
	int count = 20000;
	struct dw_pcie *pci = imx_pcie->pci;
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

static int imx_pcie_wait_for_speed_change(struct imx_pcie *imx_pcie)
{
	struct dw_pcie *pci = imx_pcie->pci;
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

static irqreturn_t imx_pcie_msi_handler(int irq, void *arg)
{
	struct imx_pcie *imx_pcie = arg;
	struct dw_pcie *pci = imx_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	return dw_handle_msi_irq(pp);
}

static void pci_imx_clk_disable(struct device *dev)
{
	u32 val;
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);

	clk_disable_unprepare(imx_pcie->pcie);
	clk_disable_unprepare(imx_pcie->pcie_phy);
	clk_disable_unprepare(imx_pcie->pcie_bus);
	switch (imx_pcie->variant) {
	case IMX6Q:
		break;
	case IMX6SX:
		clk_disable_unprepare(imx_pcie->pcie_inbound_axi);
		break;
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_REF_CLK_EN, 0);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
		break;
	case IMX7D:
		/* turn off external osc input */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				BIT(5), BIT(5));
		break;
	case IMX8MQ:
	case IMX8MM:
		if (imx_pcie->ctrl_id == 0)
			val = IOMUXC_GPR14;
		else
			val = IOMUXC_GPR16;

		regmap_update_bits(imx_pcie->iomuxc_gpr, val,
				IMX8MQ_GPR_PCIE_REF_USE_PAD, 0);
		break;
	case IMX8QXP:
	case IMX8QM:
		clk_disable_unprepare(imx_pcie->pcie_per);
		clk_disable_unprepare(imx_pcie->pcie_inbound_axi);
		break;
	}
}

static void pci_imx_ltssm_enable(struct device *dev)
{
	u32 val;
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);

	switch (imx_pcie->variant) {
	case IMX6Q:
	case IMX6SX:
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_CTL_2,
				IMX6Q_GPR12_PCIE_CTL_2);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		if (imx_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIE_CTRL_APPS_EN |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIE_CTRL_APPS_EN |
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		break;
	case IMX8QXP:
	case IMX8QM:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx_pcie->ctrl_id * SZ_64K;
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE,
				IMX8QM_CTRL_LTSSM_ENABLE);
		break;
	}

}

static int imx_pcie_establish_link(struct imx_pcie *imx_pcie)
{
	struct dw_pcie *pci = imx_pcie->pci;
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
	pci_imx_ltssm_enable(dev);

	ret = imx_pcie_wait_for_link(imx_pcie);
	if (ret)
		goto err_reset_phy;

	if (imx_pcie->link_gen >= 2) {
		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
		tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
		tmp |= imx_pcie->link_gen;
		dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		if (imx_pcie->variant != IMX7D) {
			/*
			 * On i.MX7, DIRECT_SPEED_CHANGE behaves differently
			 * from i.MX6 family when no link speed transition
			 * occurs and we go Gen1 -> yep, Gen1. The difference
			 * is that, in such case, it will not be cleared by HW
			 * which will cause the following code to report false
			 * failure.
			 */

			ret = imx_pcie_wait_for_speed_change(imx_pcie);
			if (ret) {
				dev_info(dev, "Roll back to GEN1 link!\n");
			}
		}

		/* Make sure link training is finished as well! */
		ret = imx_pcie_wait_for_link(imx_pcie);
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
	imx_pcie_reset_phy(imx_pcie);

	if (!IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
		pci_imx_clk_disable(dev);
		pm_runtime_put_sync(pci->dev);
		imx_pcie_phy_pwr_dn(imx_pcie);
		if (imx_pcie->pcie_phy_regulator != NULL)
			regulator_disable(imx_pcie->pcie_phy_regulator);
		if (imx_pcie->pcie_bus_regulator != NULL)
			regulator_disable(imx_pcie->pcie_bus_regulator);
	}

	return ret;
}

static int imx_pcie_host_init(struct pcie_port *pp)
{
	int ret;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx_pcie *imx_pcie = to_imx_pcie(pci);

	/* enable disp_mix power domain */
	pm_runtime_get_sync(pci->dev);

	imx_pcie_assert_core_reset(imx_pcie);
	imx_pcie_init_phy(imx_pcie);
	ret = imx_pcie_deassert_core_reset(imx_pcie);
	if (ret < 0)
		return ret;

	if (!IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		dw_pcie_setup_rc(pp);
		ret = imx_pcie_establish_link(imx_pcie);
		if (ret < 0)
			return ret;

		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_init(pp);
	}

	return 0;
}

static int imx_pcie_link_up(struct dw_pcie *pci)
{
	return dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1) &
			PCIE_PHY_DEBUG_R1_XMLH_LINK_UP;
}

static const struct dw_pcie_host_ops imx_pcie_host_ops = {
	.host_init = imx_pcie_host_init,
};

static int imx_add_pcie_port(struct imx_pcie *imx_pcie,
			      struct platform_device *pdev)
{
	struct dw_pcie *pci = imx_pcie->pci;
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
				       imx_pcie_msi_handler,
				       IRQF_SHARED | IRQF_NO_THREAD,
				       "mx6-pcie-msi", imx_pcie);
		if (ret) {
			dev_err(dev, "failed to request MSI irq\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &imx_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = imx_pcie_link_up,
};

static ssize_t imx_pcie_bar0_addr_info(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx_pcie->pci;

	return sprintf(buf, "imx-pcie-bar0-addr-info start 0x%08x\n",
			readl(pci->dbi_base + PCI_BASE_ADDRESS_0));
}

static ssize_t imx_pcie_bar0_addr_start(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u32 bar_start;
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx_pcie->pci;

	sscanf(buf, "%x\n", &bar_start);
	writel(bar_start, pci->dbi_base + PCI_BASE_ADDRESS_0);

	return count;
}

static void imx_pcie_regions_setup(struct device *dev)
{
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);
	struct dw_pcie *pci = imx_pcie->pci;
	struct pcie_port *pp = &pci->pp;

	switch (imx_pcie->variant) {
	case IMX8QM:
	case IMX8QXP:
	case IMX8MQ:
	case IMX8MM:
		/*
		 * RPMSG reserved 4Mbytes, but only used up to 2Mbytes.
		 * The left 2Mbytes can be used here.
		 */
		if (ddr_test_region == 0)
			dev_err(dev, "invalid ddr test region.\n");
		break;
	case IMX6SX:
	case IMX7D:
		ddr_test_region = 0xb0000000;
		break;

	case IMX6Q:
	case IMX6QP:
		ddr_test_region = 0x40000000;
		break;
	}
	dev_info(dev, "ddr_test_region is 0x%08x.\n", ddr_test_region);

	dw_pcie_prog_outbound_atu(pci, 2, 0, pp->mem_base,
				  ddr_test_region, test_region_size);
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
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);

	sscanf(buf, "%x\n", &memw_start);

	if (imx_pcie->variant == IMX7D || imx_pcie->variant == IMX6SX) {
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

static ssize_t imx_pcie_bus_freq(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u32 bus_freq;

	ret = sscanf(buf, "%x\n", &bus_freq);
	if (ret != 1)
		return -EINVAL;
	if (bus_freq) {
		dev_info(dev, "pcie request bus freq high.\n");
		request_bus_freq(BUS_FREQ_HIGH);
	} else {
		dev_info(dev, "pcie release bus freq high.\n");
		release_bus_freq(BUS_FREQ_HIGH);
	}

	return count;
}

static DEVICE_ATTR(memw_info, S_IRUGO, imx_pcie_memw_info, NULL);
static DEVICE_ATTR(memw_start_set, S_IWUSR, NULL, imx_pcie_memw_start);
static DEVICE_ATTR(memw_size_set, S_IWUSR, NULL, imx_pcie_memw_size);
static DEVICE_ATTR(ep_bar0_addr, S_IWUSR | S_IRUGO, imx_pcie_bar0_addr_info,
		imx_pcie_bar0_addr_start);
static DEVICE_ATTR(bus_freq, 0200, NULL, imx_pcie_bus_freq);

static struct attribute *imx_pcie_ep_attrs[] = {
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

static struct attribute *imx_pcie_rc_attrs[] = {
	&dev_attr_bus_freq.attr,
	NULL
};

static struct attribute_group imx_pcie_attrgroup = {
	.attrs	= imx_pcie_ep_attrs,
};

static void imx_pcie_setup_ep(struct dw_pcie *pci)
{
	int ret;
	u32 val;
	u32 lanes;
	struct device_node *np = pci->dev->of_node;

	ret = of_property_read_u32(np, "num-lanes", &lanes);
	if (ret)
		lanes = 0;

	/* set the number of lanes */
	val = dw_pcie_readl_dbi(pci, PCIE_PORT_LINK_CONTROL);
	val &= ~PORT_LINK_MODE_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	default:
		dev_err(pci->dev, "num-lanes %u: invalid value\n", lanes);
		return;
	}
	dw_pcie_writel_dbi(pci, PCIE_PORT_LINK_CONTROL, val);

	/* set link width speed control register */
	val = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	}
	dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, val);

	/* get iATU unroll support */
	val = dw_pcie_readl_dbi(pci, PCIE_ATU_VIEWPORT);
	if (val == 0xffffffff)
		pci->iatu_unroll_enabled = 1;
	dev_info(pci->dev, "iATU unroll: %s\n",
		pci->iatu_unroll_enabled ? "enabled" : "disabled");

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
	writel((readl(pci->dbi_base + PCI_CLASS_REVISION) & 0xFFFF)
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
static void pci_imx_pm_turn_off(struct imx_pcie *imx_pcie)
{
	int i;
	u32 dst, val;
	struct device *dev = imx_pcie->pci->dev;

	/* PM_TURN_OFF */
	switch (imx_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		if (imx_pcie->ctrl_id == 0)
			dst = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			dst = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx_pcie->reg_src, dst,
				IMX8MQ_PCIE_CTRL_APPS_TURNOFF |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIE_CTRL_APPS_TURNOFF |
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		regmap_update_bits(imx_pcie->reg_src, dst,
				IMX8MQ_PCIE_CTRL_APPS_TURNOFF |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		break;
	case IMX8QXP:
	case IMX8QM:
		dst = IMX8QM_CSR_PCIEA_OFFSET + imx_pcie->ctrl_id * SZ_64K;
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				IMX8QM_CTRL_PM_XMT_TURNOFF);
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				0);
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23,
				IMX8QM_CTRL_READY_ENTR_L23);
		/* check the L2 is entered or not. */
		for (i = 0; i < 10000; i++) {
			regmap_read(imx_pcie->iomuxc_gpr,
					dst + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&val);
			if (val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2)
				break;
			udelay(10);
		}
		if ((val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2) == 0)
			dev_err(dev, "PCIE%d can't enter into L2.\n",
					imx_pcie->ctrl_id);
		break;
	case IMX6Q:
		dev_info(dev, "Info: don't support pm_turn_off yet.\n");
		return;
	}

	udelay(1000);
	if (gpio_is_valid(imx_pcie->reset_gpio))
		gpio_set_value_cansleep(imx_pcie->reset_gpio, 0);
}

static int pci_imx_suspend_noirq(struct device *dev)
{
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx_pcie->pci->pp;

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_cfg_store(pp);

	pci_imx_pm_turn_off(imx_pcie);

	if (unlikely(imx_pcie->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
	} else {
		pci_imx_clk_disable(dev);

		imx_pcie_phy_pwr_dn(imx_pcie);
		/* Power down PCIe PHY. */
		if (imx_pcie->pcie_phy_regulator != NULL)
			regulator_disable(imx_pcie->pcie_phy_regulator);
		if (imx_pcie->pcie_bus_regulator != NULL)
			regulator_disable(imx_pcie->pcie_bus_regulator);
	}

	return 0;
}

static void pci_imx_ltssm_disable(struct device *dev)
{
	u32 val;
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);

	switch (imx_pcie->variant) {
	case IMX6Q:
	case IMX6SX:
	case IMX6QP:
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6Q_GPR12_PCIE_CTL_2, 0);
		break;
	case IMX7D:
	case IMX8MQ:
	case IMX8MM:
		if (imx_pcie->ctrl_id == 0)
			val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
		else
			val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
		regmap_update_bits(imx_pcie->reg_src, val,
				IMX8MQ_PCIE_CTRL_APPS_EN |
				IMX8MQ_PCIEPHY_DOMAIN_EN,
				IMX8MQ_PCIEPHY_DOMAIN_EN);
		break;
	case IMX8QXP:
	case IMX8QM:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx_pcie->ctrl_id * SZ_64K;
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE, 0);
		regmap_update_bits(imx_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23, 0);
		break;
	}
}

static int pci_imx_resume_noirq(struct device *dev)
{
	int ret = 0;
	struct imx_pcie *imx_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx_pcie->pci->pp;

	if (unlikely(imx_pcie->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD, 0);
	} else {
		pci_imx_ltssm_disable(dev);
		imx_pcie_assert_core_reset(imx_pcie);
		imx_pcie_init_phy(imx_pcie);
		ret = imx_pcie_deassert_core_reset(imx_pcie);
		if (ret < 0)
			return ret;

		/*
		 * controller maybe turn off, re-configure again
		 */
		dw_pcie_setup_rc(pp);

		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_cfg_restore(pp);
		pci_imx_ltssm_enable(dev);

		ret = imx_pcie_wait_for_link(imx_pcie);
		if (ret < 0)
			dev_info(dev, "pcie link is down after resume.\n");
	}

	return ret;
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

static irqreturn_t imx_pcie_dma_isr(int irq, void *param)
{
	u32 irqs, offset;
	struct pcie_port *pp = (struct pcie_port *)param;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx_pcie *imx_pcie = to_imx_pcie(pci);

	offset = imx_pcie->dma_unroll_offset;

	/* check write isr */
	irqs = readl(pci->dbi_base + offset + DMA_WRITE_INT_STS);
	if (irqs & DMA_DONE_INT_STS) {
		/* write 1 clear */
		writel(irqs & DMA_DONE_INT_STS,
				pci->dbi_base + offset + DMA_WRITE_INT_CLR);
		dma_w_end = 1;
	} else if (irqs & DMA_ABORT_INT_STS) {
		pr_info("imx pcie dma write error 0x%0x.\n", irqs);
	}
	/* check read isr */
	irqs = readl(pci->dbi_base + offset + DMA_READ_INT_STS);
	if (irqs & DMA_DONE_INT_STS) {
		/* write 1 clear */
		writel(irqs & DMA_DONE_INT_STS,
				pci->dbi_base + offset + DMA_READ_INT_CLR);
		dma_r_end = 1;
	} else if (irqs & DMA_ABORT_INT_STS) {
		pr_info("imx pcie dma read error 0x%0x.", irqs);
	}
	return IRQ_HANDLED;
}

/**
 * imx_pcie_local_dma_start - Start one local iMX PCIE DMA.
 * @pp: the port start the dma transmission.
 * @dir: direction of the dma, 1 read, 0 write;
 * @chl: the channel num of the iMX PCIE DMA(0 - 7).
 * @src: source DMA address.
 * @dst: destination DMA address.
 * @len: transfer length.
 */
static int imx_pcie_local_dma_start(struct pcie_port *pp, bool dir,
		unsigned int chl, dma_addr_t src, dma_addr_t dst,
		unsigned int len)
{
	u32 offset, doorbell, unroll_cal;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx_pcie *imx_pcie = to_imx_pcie(pci);

	if (pp == NULL)
		return -EINVAL;
	if (chl > MAX_PCIE_DMA_CHANNELS)
		return -EINVAL;

	offset = imx_pcie->dma_unroll_offset;
	/* enable dma engine, dir 1:read. 0:write. */
	if (dir)
		writel(DMA_READ_ENGINE_EN,
				pci->dbi_base + offset
				+ DMA_READ_ENGINE_EN_OFF);
	else
		writel(DMA_WRITE_ENGINE_EN,
				pci->dbi_base + offset
				+ DMA_WRITE_ENGINE_EN_OFF);
	writel(0x0, pci->dbi_base + offset + DMA_WRITE_INT_MASK);
	writel(0x0, pci->dbi_base + offset + DMA_READ_INT_MASK);
	 /* ch dir and ch num */
	if (offset == 0) {
		writel((dir << 31) | chl, pci->dbi_base + DMA_VIEWPOT_SEL_OFF);
		writel(DMA_CHANNEL_CTRL_1_LIE,
				pci->dbi_base + DMA_CHANNEL_CTRL_1);
		writel(0x0, pci->dbi_base + DMA_CHANNEL_CTRL_2);
		writel(len, pci->dbi_base + DMA_TRANSFER_SIZE);
		writel((u32)src, pci->dbi_base + DMA_SAR_LOW);
		writel(0x0, pci->dbi_base + DMA_SAR_HIGH);
		writel((u32)dst, pci->dbi_base + DMA_DAR_LOW);
		writel(0x0, pci->dbi_base + DMA_DAR_HIGH);
	} else {
		unroll_cal = DMA_UNROLL_CDM_OFFSET
			+ 0x200 * (chl + 1) + 0x100 * dir;
		writel(DMA_CHANNEL_CTRL_1_LIE, pci->dbi_base + unroll_cal);
		writel(0x0, pci->dbi_base + unroll_cal + 0x4);
		writel(len, pci->dbi_base + unroll_cal + 0x8);
		writel((u32)src, pci->dbi_base + unroll_cal + 0xc);
		writel(0x0, pci->dbi_base + unroll_cal + 0x10);
		writel((u32)dst, pci->dbi_base + unroll_cal + 0x14);
		writel(0x0, pci->dbi_base + unroll_cal + 0x18);
	}

	doorbell = dir ? DMA_READ_DOORBELL : DMA_WRITE_DOORBELL;
	writel(chl, pci->dbi_base + offset + doorbell);

	return 0;
}

static int __init imx_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct imx_pcie *imx_pcie;
	struct resource *res, reserved_res;
	struct device_node *reserved_node, *node = dev->of_node;
	int ret;

	imx_pcie = devm_kzalloc(dev, sizeof(*imx_pcie), GFP_KERNEL);
	if (!imx_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	imx_pcie->pci = pci;
	imx_pcie->variant =
		(enum imx_pcie_variants)of_device_get_match_data(dev);

	if (of_property_read_u32(node, "hsio-cfg", &imx_pcie->hsio_cfg))
		imx_pcie->hsio_cfg = 0;

	if (of_property_read_u32(node, "ctrl-id", &imx_pcie->ctrl_id))
		imx_pcie->ctrl_id = 0;

	if (of_property_read_u32(node, "cpu-base-addr", &imx_pcie->cpu_base))
		imx_pcie->cpu_base = 0;
	if (of_property_read_u32(node, "hard-wired", &imx_pcie->hard_wired))
		imx_pcie->hard_wired = 0;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	if (res)
		imx_pcie->phy_base = devm_ioremap_resource(dev, res);
	else
		imx_pcie->phy_base = NULL;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res)
		pci->dbi_base = devm_ioremap_resource(dev, res);
	else
		dev_err(dev, "missing *dbi* reg space\n");
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	reserved_node = of_parse_phandle(node, "reserved-region", 0);
	if (!reserved_node) {
		dev_info(dev, "no reserved region node.\n");
	} else {
		if (of_address_to_resource(reserved_node, 0, &reserved_res)) {
			dev_err(dev, "failed to get reserved region address\n");
			of_node_put(reserved_node);
			return -EINVAL;
		}
		ddr_test_region = reserved_res.start + SZ_2M;
		of_node_put(reserved_node);
	}

	/* Fetch GPIOs */
	imx_pcie->clkreq_gpio = of_get_named_gpio(node, "clkreq-gpio", 0);
	if (gpio_is_valid(imx_pcie->clkreq_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx_pcie->clkreq_gpio,
					    GPIOF_OUT_INIT_LOW, "PCIe CLKREQ");
		if (ret) {
			dev_err(&pdev->dev, "unable to get clkreq gpio\n");
			return ret;
		}
	} else if (imx_pcie->clkreq_gpio == -EPROBE_DEFER) {
		return imx_pcie->clkreq_gpio;
	}

	imx_pcie->dis_gpio = of_get_named_gpio(node, "disable-gpio", 0);
	if (gpio_is_valid(imx_pcie->dis_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx_pcie->dis_gpio,
					    GPIOF_OUT_INIT_HIGH, "PCIe DIS");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable gpio\n");
			return ret;
		}
	} else if (imx_pcie->dis_gpio == -EPROBE_DEFER) {
		return imx_pcie->dis_gpio;
	}

	imx_pcie->power_on_gpio = of_get_named_gpio(node, "power-on-gpio", 0);
	if (gpio_is_valid(imx_pcie->power_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					    imx_pcie->power_on_gpio,
					    GPIOF_OUT_INIT_LOW,
					    "PCIe power enable");
		if (ret) {
			dev_err(&pdev->dev, "unable to get power-on gpio\n");
			return ret;
		}
	} else if (imx_pcie->power_on_gpio == -EPROBE_DEFER) {
		return imx_pcie->power_on_gpio;
	}

	imx_pcie->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	imx_pcie->gpio_active_high = of_property_read_bool(node,
						"reset-gpio-active-high");
	if (gpio_is_valid(imx_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, imx_pcie->reset_gpio,
				imx_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	} else if (imx_pcie->reset_gpio == -EPROBE_DEFER) {
		return imx_pcie->reset_gpio;
	}

	imx_pcie->epdev_on = devm_regulator_get(&pdev->dev, "epdev_on");
	if (IS_ERR(imx_pcie->epdev_on)) {
		if (PTR_ERR(imx_pcie->epdev_on) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "no ep regulator found\n");
		imx_pcie->epdev_on = NULL;
	} else {
		ret = regulator_enable(imx_pcie->epdev_on);
		if (ret)
			dev_err(dev, "failed to enable the epdev_on regulator\n");
	}

	/* Fetch clocks */
	imx_pcie->pcie_phy = devm_clk_get(dev, "pcie_phy");
	if (IS_ERR(imx_pcie->pcie_phy)) {
		dev_err(dev, "pcie_phy clock source missing or invalid\n");
		return PTR_ERR(imx_pcie->pcie_phy);
	}

	imx_pcie->pcie_bus = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(imx_pcie->pcie_bus)) {
		dev_err(dev, "pcie_bus clock source missing or invalid\n");
		return PTR_ERR(imx_pcie->pcie_bus);
	}

	if (of_property_read_u32(node, "ext_osc", &imx_pcie->ext_osc) < 0)
		imx_pcie->ext_osc = 0;

	if (imx_pcie->ext_osc && (imx_pcie->variant == IMX6QP)) {
		/* Change the pcie_bus clock to pcie external OSC */
		imx_pcie->pcie_bus = devm_clk_get(&pdev->dev, "pcie_ext");
		if (IS_ERR(imx_pcie->pcie_bus)) {
			dev_err(&pdev->dev,
				"pcie_bus clock source missing or invalid\n");
			return PTR_ERR(imx_pcie->pcie_bus);
		}

		imx_pcie->pcie_ext_src = devm_clk_get(&pdev->dev,
				"pcie_ext_src");
		if (IS_ERR(imx_pcie->pcie_ext_src)) {
			dev_err(&pdev->dev,
				"pcie_ext_src clk src missing or invalid\n");
			return PTR_ERR(imx_pcie->pcie_ext_src);
		}
	}

	imx_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx_pcie->pcie)) {
		dev_err(dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(imx_pcie->pcie);
	}

	if (imx_pcie->variant == IMX6QP) {
		imx_pcie->pcie_bus_regulator = devm_regulator_get(dev,
				"pcie-bus");
		if (PTR_ERR(imx_pcie->pcie_bus_regulator) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (IS_ERR(imx_pcie->pcie_bus_regulator))
			imx_pcie->pcie_bus_regulator = NULL;
	} else {
		imx_pcie->pcie_bus_regulator = NULL;
	}

	/* Grab GPR config register range */
	if (imx_pcie->variant == IMX7D) {
		imx_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx7d-iomuxc-gpr");
		imx_pcie->reg_src =
			 syscon_regmap_lookup_by_compatible("fsl,imx7d-src");
		if (IS_ERR(imx_pcie->reg_src)) {
			dev_err(&pdev->dev,
				"imx7d pcie phy src missing or invalid\n");
			return PTR_ERR(imx_pcie->reg_src);
		}
		imx_pcie->pcie_phy_regulator = devm_regulator_get(&pdev->dev,
				"pcie-phy");
	} else if (imx_pcie->variant == IMX8MQ || imx_pcie->variant == IMX8MM) {
		imx_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx7d-iomuxc-gpr");
		imx_pcie->reg_src =
			 syscon_regmap_lookup_by_compatible("fsl,imx8mq-src");
		if (IS_ERR(imx_pcie->reg_src)) {
			dev_err(&pdev->dev,
				"imx8mq pcie phy src missing or invalid\n");
			return PTR_ERR(imx_pcie->reg_src);
		}
		imx_pcie->reg_gpc =
			 syscon_regmap_lookup_by_compatible("fsl,imx8mq-gpc");
		if (IS_ERR(imx_pcie->reg_gpc)) {
			dev_err(&pdev->dev,
				"imx8mq pcie phy src missing or invalid\n");
			return PTR_ERR(imx_pcie->reg_gpc);
		}
	} else if (imx_pcie->variant == IMX6SX) {
		imx_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx_pcie->pcie_inbound_axi);
		}

		imx_pcie->pcie_phy_regulator = devm_regulator_get(&pdev->dev,
				"pcie-phy");

		imx_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_compatible
			 ("fsl,imx6sx-iomuxc-gpr");
	} else if (imx_pcie->variant == IMX8QM
			|| imx_pcie->variant == IMX8QXP) {
		imx_pcie->pcie_per = devm_clk_get(dev, "pcie_per");
		if (IS_ERR(imx_pcie->pcie_per)) {
			dev_err(dev, "pcie_per clock source missing or invalid\n");
			return PTR_ERR(imx_pcie->pcie_per);
		}

		imx_pcie->iomuxc_gpr =
			 syscon_regmap_lookup_by_phandle(node, "hsio");
		imx_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx_pcie->pcie_inbound_axi);
		}
	} else {
		imx_pcie->iomuxc_gpr =
		 syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	}

	if (IS_ERR(imx_pcie->iomuxc_gpr)) {
		dev_err(dev, "unable to find iomuxc registers\n");
		return PTR_ERR(imx_pcie->iomuxc_gpr);
	}

	/* Grab PCIe PHY Tx Settings */
	if (of_property_read_u32(node, "fsl,tx-deemph-gen1",
				 &imx_pcie->tx_deemph_gen1))
		imx_pcie->tx_deemph_gen1 = 20;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx_pcie->tx_deemph_gen2_3p5db))
		imx_pcie->tx_deemph_gen2_3p5db = 20;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx_pcie->tx_deemph_gen2_6db))
		imx_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx_pcie->tx_swing_full))
		imx_pcie->tx_swing_full = 115;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx_pcie->tx_swing_low))
		imx_pcie->tx_swing_low = 115;

	/* Limit link speed */
	ret = of_property_read_u32(node, "fsl,max-link-speed",
				   &imx_pcie->link_gen);
	if (ret)
		imx_pcie->link_gen = 1;

	imx_pcie->vpcie = devm_regulator_get_optional(&pdev->dev, "vpcie");
	if (IS_ERR(imx_pcie->vpcie)) {
		if (PTR_ERR(imx_pcie->vpcie) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		imx_pcie->vpcie = NULL;
	}

	platform_set_drvdata(pdev, imx_pcie);

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)
			&& (imx_pcie->hard_wired == 0)) {
		int i = 0, irq;
		void *test_reg1, *test_reg2;
		dma_addr_t test_reg1_dma, test_reg2_dma;
		void __iomem *pcie_arb_base_addr;
		struct timeval tv1s, tv1e, tv2s, tv2e;
		u32 val, tv_count1, tv_count2;
		struct device_node *np = node;
		struct pcie_port *pp = &pci->pp;
		LIST_HEAD(res);
		struct resource_entry *win, *tmp;
		unsigned long timeout = jiffies + msecs_to_jiffies(300000);

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
		pp->ops = &imx_pcie_host_ops;
		dev_info(dev, " try to initialize pcie ep.\n");
		ret = imx_pcie_host_init(pp);
		if (ret) {
			dev_info(dev, " fail to initialize pcie ep.\n");
			return ret;
		}

		imx_pcie_setup_ep(pci);
		platform_set_drvdata(pdev, imx_pcie);
		imx_pcie_regions_setup(dev);

		/*
		 * iMX6SX PCIe has the stand-alone power domain.
		 * refer to the initialization for iMX6SX PCIe,
		 * release the PCIe PHY reset here,
		 * before LTSSM enable is set
		 * .
		 */
		if (imx_pcie->variant == IMX6SX)
			regmap_update_bits(imx_pcie->iomuxc_gpr, IOMUXC_GPR5,
					BIT(19), 0 << 19);

		/* assert LTSSM enable */
		pci_imx_ltssm_enable(dev);

		dev_info(dev, "PCIe EP: waiting for link up...\n");
		/* link is indicated by the bit4 of DB_R1 register */
		do {
			usleep_range(10, 20);
			if (time_after(jiffies, timeout)) {
				dev_info(dev, "PCIe EP: link down.\n");
				return 0;
			}
		} while ((readl(pci->dbi_base + PCIE_PHY_DEBUG_R1) & 0x10) == 0);

		/* self io test */
		/* Check the DMA INT exist or not */
		irq = of_irq_get(node, 1);
		if (irq > 0)
			dma_en = 1;
		else
			dma_en = 0;
		if (dma_en) {
			/* configure the DMA INT ISR */
			ret = request_irq(irq, imx_pcie_dma_isr,
					  IRQF_SHARED, "imx-pcie-dma", pp);
			if (ret) {
				pr_err("register interrupt %d failed, rc %d\n",
					irq, ret);
				dma_en = 0;
			}
			test_reg1 = dma_alloc_coherent(dev, test_region_size,
					&test_reg1_dma, GFP_KERNEL);
			test_reg2 = dma_alloc_coherent(dev, test_region_size,
					&test_reg2_dma, GFP_KERNEL);
			if (!(test_reg1 && test_reg2))
				dma_en = 0; /* Roll back to PIO. */
			dma_r_end = dma_w_end = 0;

			val = readl(pci->dbi_base + DMA_CTRL_VIEWPORT_OFF);
			if (val == 0xffffffff)
				imx_pcie->dma_unroll_offset =
					DMA_UNROLL_CDM_OFFSET - DMA_REG_OFFSET;
			else
				imx_pcie->dma_unroll_offset = 0;
		}

		if (unlikely(dma_en == 0)) {
			test_reg1 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg1) {
				ret = -ENOMEM;
				return ret;
			}

			test_reg2 = devm_kzalloc(&pdev->dev,
					test_region_size, GFP_KERNEL);
			if (!test_reg2) {
				ret = -ENOMEM;
				return ret;
			}
		}

		pcie_arb_base_addr = ioremap_nocache(pp->mem_base,
					test_region_size);
		if (!pcie_arb_base_addr) {
			dev_err(dev, "ioremap error in ep io test\n");
			ret = -ENOMEM;
			return ret;
		}

		for (i = 0; i < test_region_size; i = i + 4) {
			writel(0xE6600D00 + i, test_reg1 + i);
			writel(0xDEADBEAF, test_reg2 + i);
		}

		/* PCIe EP start the data transfer after link up */
		dev_info(dev, "pcie ep: Starting data transfer...\n");
		do_gettimeofday(&tv1s);

		/* EP write the test region to remote RC's DDR memory */
		if (dma_en) {
			imx_pcie_local_dma_start(pp, 0, 0, test_reg1_dma,
					pp->mem_base + pp->cpu_addr_offset,
					test_region_size);
			timeout = jiffies + msecs_to_jiffies(300);
			do {
				udelay(1);
				if (time_after(jiffies, timeout)) {
					dev_info(dev, "dma write no end ...\n");
					break;
				}
			} while (!dma_w_end);
		} else {
			memcpy((unsigned int *)pcie_arb_base_addr,
					(unsigned int *)test_reg1,
					test_region_size);
		}

		do_gettimeofday(&tv1e);

		do_gettimeofday(&tv2s);
		/* EP read the test region back from remote RC's DDR memory */
		if (dma_en) {
			imx_pcie_local_dma_start(pp, 1, 0,
					pp->mem_base + pp->cpu_addr_offset,
					test_reg2_dma, test_region_size);
			timeout = jiffies + msecs_to_jiffies(300);
			do {
				udelay(1);
				if (time_after(jiffies, timeout)) {
					dev_info(dev, "dma read no end\n");
					break;
				}
			} while (!dma_r_end);
		} else {
			memcpy((unsigned int *)test_reg2,
					(unsigned int *)pcie_arb_base_addr,
					test_region_size);
		}

		do_gettimeofday(&tv2e);
		if (memcmp(test_reg2, test_reg1, test_region_size) == 0) {
			tv_count1 = (tv1e.tv_sec - tv1s.tv_sec)
				* USEC_PER_SEC
				+ tv1e.tv_usec - tv1s.tv_usec;
			tv_count2 = (tv2e.tv_sec - tv2s.tv_sec)
				* USEC_PER_SEC
				+ tv2e.tv_usec - tv2s.tv_usec;

			dev_info(dev, "pcie ep: Data %s transfer is successful."
					" tv_count1 %dus,"
					" tv_count2 %dus.\n",
					dma_en ? "DMA" : "PIO",
					tv_count1, tv_count2);
			dev_info(dev, "pcie ep: Data write speed:%ldMB/s.\n",
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count1));
			dev_info(dev, "pcie ep: Data read speed:%ldMB/s.\n",
					((test_region_size/1024)
					   * MSEC_PER_SEC)
					/(tv_count2));
		} else {
			dev_info(dev, "pcie ep: Data transfer is failed.\n");
		} /* end of self io test. */
	} else {
		/* add attributes for bus freq */
		imx_pcie_attrgroup.attrs = imx_pcie_rc_attrs;
		ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
		if (ret)
			return -EINVAL;

		ret = imx_add_pcie_port(imx_pcie, pdev);
		if (ret < 0) {
			if (IS_ENABLED(CONFIG_PCI_IMX6_COMPLIANCE_TEST)) {
				/* The PCIE clocks wouldn't be turned off */
				dev_info(dev, "To do the compliance tests.\n");
				ret = 0;
			} else {
				dev_err(dev, "unable to add pcie port.\n");
			}
			return ret;
		}
		if (IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS)
				&& (imx_pcie->hard_wired == 0))
			imx_pcie_regions_setup(&pdev->dev);
	}
	return 0;
}

static void imx_pcie_shutdown(struct platform_device *pdev)
{
	struct imx_pcie *imx_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	if (imx_pcie->variant == IMX6Q)
		imx_pcie_assert_core_reset(imx_pcie);
}

static const struct of_device_id imx_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = (void *)IMX6Q,  },
	{ .compatible = "fsl,imx6sx-pcie", .data = (void *)IMX6SX, },
	{ .compatible = "fsl,imx6qp-pcie", .data = (void *)IMX6QP, },
	{ .compatible = "fsl,imx7d-pcie",  .data = (void *)IMX7D,  },
	{ .compatible = "fsl,imx8qm-pcie", .data = (void *)IMX8QM, },
	{ .compatible = "fsl,imx8qxp-pcie", .data = (void *)IMX8QXP, },
	{ .compatible = "fsl,imx8mq-pcie", .data = (void *)IMX8MQ, },
	{ .compatible = "fsl,imx8mm-pcie", .data = (void *)IMX8MM, },
	{},
};

static struct platform_driver imx_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.of_match_table = imx_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &pci_imx_pm_ops,
	},
	.probe    = imx_pcie_probe,
	.shutdown = imx_pcie_shutdown,
};

static int __init imx_pcie_init(void)
{
#ifdef CONFIG_ARM
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
#endif

	return platform_driver_register(&imx_pcie_driver);
}
device_initcall(imx_pcie_init);
