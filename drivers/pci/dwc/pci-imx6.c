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
	int			dis_gpio;
	int			power_on_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie;
	struct regmap		*iomuxc_gpr;
	enum imx6_pcie_variants variant;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	int			link_gen;
	struct regulator	*vpcie;
	struct regmap		*reg_src;
	struct regulator	*pcie_phy_regulator;
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

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D
#define PCIE_PHY_RX_ASIC_OUT_VALID	(1 << 0)

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

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

static void imx6_pcie_assert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct device *dev = imx6_pcie->pci->dev;

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
		break;
	}

	return ret;
}

static void imx7d_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	unsigned int retries;
	struct device *dev = imx6_pcie->pci->dev;

	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES; retries++) {
		regmap_read(imx6_pcie->iomuxc_gpr, IOMUXC_GPR22, &val);

		if (val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED)
			return;

		usleep_range(PHY_PLL_LOCK_WAIT_USLEEP_MIN,
			     PHY_PLL_LOCK_WAIT_USLEEP_MAX);
	}

	dev_err(dev, "PCIe PLL lock timeout\n");
}

static void imx6_pcie_deassert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	if (imx6_pcie->vpcie && !regulator_is_enabled(imx6_pcie->vpcie)) {
		ret = regulator_enable(imx6_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return;
		}
	}

	if (gpio_is_valid(imx6_pcie->power_on_gpio))
		gpio_set_value_cansleep(imx6_pcie->power_on_gpio, 1);

	request_bus_freq(BUS_FREQ_HIGH);
	ret = clk_prepare_enable(imx6_pcie->pcie_phy);
	if (ret) {
		dev_err(dev, "unable to enable pcie_phy clock\n");
		goto err_pcie_phy;
	}

	if (!IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS) &&
	    !IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS)) {
		ret = clk_prepare_enable(imx6_pcie->pcie_bus);
		if (ret) {
			dev_err(dev, "unable to enable pcie_bus clock\n");
			goto err_pcie_bus;
		}
	}

	ret = clk_prepare_enable(imx6_pcie->pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie clock\n");
		goto err_pcie;
	}

	ret = imx6_pcie_enable_ref_clk(imx6_pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie ref clock\n");
		goto err_ref_clk;
	}

	/* allow the clocks to stabilize */
	udelay(200);

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					imx6_pcie->gpio_active_high);
		msleep(100);
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					!imx6_pcie->gpio_active_high);
	}

	switch (imx6_pcie->variant) {
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		usleep_range(200, 500);
		break;
	case IMX6Q:		/* Nothing to do */
		break;
	case IMX7D:
		/* wait for more than 10us to release phy g_rst and btnrst */
		udelay(10);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), 0);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(1), 0);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(2), 0);

		/* wait for phy pll lock firstly. */
		imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	}

	return;

err_ref_clk:
	clk_disable_unprepare(imx6_pcie->pcie);
err_pcie:
	if (!IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS) &&
	    !IS_ENABLED(CONFIG_RC_MODE_IN_EP_RC_SYS))
		clk_disable_unprepare(imx6_pcie->pcie_bus);
err_pcie_bus:
	clk_disable_unprepare(imx6_pcie->pcie_phy);
err_pcie_phy:
	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		ret = regulator_disable(imx6_pcie->vpcie);
		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static void imx6_pcie_init_phy(struct imx6_pcie *imx6_pcie)
{
	int ret;

	if (imx6_pcie->variant == IMX6SX)
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);

	if (imx6_pcie->variant == IMX7D) {
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
	} else {
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, 9 << 4);

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
	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS))
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_DEVICE_TYPE,
				   PCI_EXP_TYPE_ENDPOINT << 12);
	else
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_DEVICE_TYPE,
				   PCI_EXP_TYPE_ROOT_PORT << 12);
}

static int imx6_pcie_wait_for_link(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	/* check if the link is up or not */
	if (!dw_pcie_wait_for_link(pci))
		return 0;

	dev_dbg(dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R0),
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1));
	return -ETIMEDOUT;
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
		usleep_range(100, 1000);
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
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

	/* Start LTSSM. */
	if (imx6_pcie->variant == IMX7D)
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), BIT(6));
	else
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);

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
				dev_err(dev, "Failed to bring link up!\n");
				goto err_reset_phy;
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
	return ret;
}

static int imx6_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	/* enable disp_mix power domain */
	if (imx6_pcie->variant == IMX7D)
		pm_runtime_get_sync(pci->dev);

	imx6_pcie_assert_core_reset(imx6_pcie);
	imx6_pcie_init_phy(imx6_pcie);
	imx6_pcie_deassert_core_reset(imx6_pcie);
	dw_pcie_setup_rc(pp);
	imx6_pcie_establish_link(imx6_pcie);

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

	if (imx6_pcie->variant == IMX7D) {
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

	if (imx6_pcie->variant == IMX7D) {
		/* Disable clks */
		clk_disable_unprepare(imx6_pcie->pcie);
		clk_disable_unprepare(imx6_pcie->pcie_phy);
		clk_disable_unprepare(imx6_pcie->pcie_bus);
		/* turn off external osc input */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				BIT(5), BIT(5));
		release_bus_freq(BUS_FREQ_HIGH);

		/* Power down PCIe PHY. */
		regulator_disable(imx6_pcie->pcie_phy_regulator);
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
	int ret = 0;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pci->pp;

	if (imx6_pcie->variant == IMX7D) {
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), 0);

		imx6_pcie_assert_core_reset(imx6_pcie);

		imx6_pcie_init_phy(imx6_pcie);

		imx6_pcie_deassert_core_reset(imx6_pcie);

		/*
		 * controller maybe turn off, re-configure again
		 */
		dw_pcie_setup_rc(pp);

		if (IS_ENABLED(CONFIG_PCI_MSI))
			dw_pcie_msi_cfg_restore(pp);

		/* wait for phy pll lock firstly. */
		imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
		regmap_update_bits(imx6_pcie->reg_src, 0x2c, BIT(6), BIT(6));

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
	struct resource *dbi_base;
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

	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
		/* add attributes for device */
		ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
		if (ret)
			return -EINVAL;
	}

	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	/* Fetch GPIOs */
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

	imx6_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx6_pcie->pcie)) {
		dev_err(dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie);
	}

	switch (imx6_pcie->variant) {
	case IMX6SX:
		imx6_pcie->pcie_inbound_axi = devm_clk_get(dev,
							   "pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(dev, "pcie_inbound_axi clock missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}
		break;
	default:
		break;
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
		imx6_pcie->tx_deemph_gen1 = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx6_pcie->tx_deemph_gen2_3p5db))
		imx6_pcie->tx_deemph_gen2_3p5db = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx6_pcie->tx_deemph_gen2_6db))
		imx6_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx6_pcie->tx_swing_full))
		imx6_pcie->tx_swing_full = 127;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx6_pcie->tx_swing_low))
		imx6_pcie->tx_swing_low = 127;

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
		u32 tv_count1, tv_count2;
		struct device_node *np = node;
		struct of_pci_range range;
		struct of_pci_range_parser parser;
		unsigned long restype;
		struct pcie_port *pp = &pci->pp;

		if (of_pci_range_parser_init(&parser, np)) {
			dev_err(dev, "missing ranges property\n");
			return -EINVAL;
		}

		/* Get the memory ranges from DT */
		for_each_of_pci_range(&parser, &range) {
			restype = range.flags & IORESOURCE_TYPE_BITS;
			if (restype == IORESOURCE_MEM) {
				of_pci_range_to_resource(&range, np, pp->mem);
				pp->mem->name = "MEM";
			}
		}

		pp->mem_base = pp->mem->start;

		/* enable disp_mix power domain */
		if (imx6_pcie->variant == IMX7D)
			pm_runtime_get_sync(dev);

		imx6_pcie_assert_core_reset(imx6_pcie);
		imx6_pcie_init_phy(imx6_pcie);
		imx6_pcie_deassert_core_reset(imx6_pcie);

		/* assert LTSSM enable */
		if (imx6_pcie->variant == IMX7D) {
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
		if (imx6_pcie->variant == IMX7D)
			pcie_arb_base_addr = ioremap_nocache(pp->mem_base,
					test_region_size);
		else
			pcie_arb_base_addr = ioremap_cache(pp->mem_base,
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
	imx6_pcie_assert_core_reset(imx6_pcie);
}

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = (void *)IMX6Q,  },
	{ .compatible = "fsl,imx6sx-pcie", .data = (void *)IMX6SX, },
	{ .compatible = "fsl,imx6qp-pcie", .data = (void *)IMX6QP, },
	{ .compatible = "fsl,imx7d-pcie",  .data = (void *)IMX7D,  },
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
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx6q_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx6q_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");

	return platform_driver_register(&imx6_pcie_driver);
}
device_initcall(imx6_pcie_init);
