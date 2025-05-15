// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024-2025, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/string.h>
#include <linux/bitfield.h>
#include <linux/ethtool.h>
#include "adrv906x-net.h"
#include "adrv906x-cmn.h"

#define EMAC_CMN_DIGITAL_CTRL0               0x0010
#define   EMAC_CMN_RX_LINK0_EN               BIT(0)
#define   EMAC_CMN_RX_LINK1_EN               BIT(1)
#define   EMAC_CMN_TX_LINK0_EN               BIT(4)
#define   EMAC_CMN_TX_LINK1_EN               BIT(5)
#define   EMAC_CMN_SW_LINK0_BYPASS_EN        BIT(8)
#define   EMAC_CMN_SW_LINK1_BYPASS_EN        BIT(9)
#define   EMAC_CMN_SW_PORT0_EN               BIT(12)
#define   EMAC_CMN_SW_PORT1_EN               BIT(13)
#define   EMAC_CMN_SW_PORT2_EN               BIT(14)
#define   EMAC_CMN_MACSEC_BYPASS_EN          BIT(16)
#define   EMAC_CMN_CDR_DIV_PORT0_EN          BIT(20)
#define   EMAC_CMN_CDR_DIV_PORT1_EN          BIT(21)
#define   EMAC_CMN_CDR_SEL                   BIT(24)
#define EMAC_CMN_DIGITAL_CTRL1               0x0014
#define   EMAC_CMN_RECOVERED_CLK_DIV_0       GENMASK(12, 0)
#define   EMAC_CMN_RECOVERED_CLK_DIV_1       GENMASK(28, 16)
#define EMAC_CMN_DIGITAL_CTRL2               0x0018
#define   EMAC_CMN_LOOPBACK_BYPASS_MAC_0     BIT(28)
#define   EMAC_CMN_LOOPBACK_BYPASS_MAC_1     BIT(29)
#define   EMAC_CMN_LOOPBACK_BYPASS_PCS_0     BIT(24)
#define   EMAC_CMN_LOOPBACK_BYPASS_PCS_1     BIT(25)
#define   EMAC_CMN_LOOPBACK_BYPASS_DESER_0   BIT(20)
#define   EMAC_CMN_LOOPBACK_BYPASS_DESER_1   BIT(21)
#define   EMAC_CMN_TX_BIT_REPEAT_RATIO       BIT(0)
#define EMAC_CMN_DIGITAL_CTRL3               0x001c
#define   EMAC_CMN_SW_PORT2_DSA_INSERT_EN    BIT(20)
#define EMAC_CMN_DIGITAL_CTRL4               0x0020
#define   EMAC_CMN_PCS_STATUS_NE_CNT_0       GENMASK(7, 0)
#define   EMAC_CMN_PCS_STATUS_NE_CNT_1       GENMASK(15, 8)
#define   EMAC_CMN_CLEAR_PCS_STATUS_NE_CNT   BIT(16)
#define EMAC_CMN_RST_REG                     0x0030
#define EMAC_CMN_PHY_CTRL                    0x0040
#define   EMAC_CMN_RXDES_DIG_RESET_N_0       BIT(0)
#define   EMAC_CMN_RXDES_DIG_RESET_N_1       BIT(1)
#define   EMAC_CMN_RXDES_FORCE_LANE_PD_0     BIT(4)
#define   EMAC_CMN_RXDES_FORCE_LANE_PD_1     BIT(5)
#define   EMAC_CMN_TXSER_DIG_RESET_N_0       BIT(8)
#define   EMAC_CMN_TXSER_DIG_RESET_N_1       BIT(9)
#define   EMAC_CMN_TXSER_FORCE_LANE_PD_0     BIT(12)
#define   EMAC_CMN_TXSER_FORCE_LANE_PD_1     BIT(13)
#define   EMAC_CMN_SERDES_REG_RESET_N        BIT(16)
#define   EMAC_CMN_TXSER_SYNC_TRIGGER_0      BIT(20)
#define   EMAC_CMN_TXSER_SYNC_TRIGGER_1      BIT(21)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_EN_0  BIT(24)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_EN_1  BIT(25)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_VAL_0 BIT(28)
#define   EMAC_CMN_TXSER_SYNC_OVERRIDE_VAL_1 BIT(29)
#define EMAC_CMN_PLL_CTRL                    0x0050
#define   EMAC_CMN_PLL_MEM_MAP_RESET_N       BIT(0)
#define EMAC_CMN_GPIO_SELECT                 0x0060
#define EMAC_CMN_EMAC_SPARE                  0x3000

void adrv906x_eth_cmn_pll_reset(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val;

	mutex_lock(&eth_if->mtx);

	val = ioread32(regs + EMAC_CMN_PLL_CTRL);
	val &= ~EMAC_CMN_PLL_MEM_MAP_RESET_N;
	iowrite32(val, regs + EMAC_CMN_PLL_CTRL);
	usleep_range(50, 60);
	val |= EMAC_CMN_PLL_MEM_MAP_RESET_N;
	iowrite32(val, regs + EMAC_CMN_PLL_CTRL);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_pll_reset);

void adrv906x_eth_cmn_ser_tx_sync_trigger(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, bit_mask;

	mutex_lock(&eth_if->mtx);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_TXSER_SYNC_TRIGGER_0 : EMAC_CMN_TXSER_SYNC_TRIGGER_1;
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val |= bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val &= ~bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_ser_tx_sync_trigger);

void adrv906x_eth_cmn_ser_pwr_down(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, bit_mask;

	mutex_lock(&eth_if->mtx);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_TXSER_FORCE_LANE_PD_0 : EMAC_CMN_TXSER_FORCE_LANE_PD_1;
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val |= bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(10, 20);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_ser_pwr_down);

void adrv906x_eth_cmn_ser_pwr_up_and_reset(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, bit_mask;

	mutex_lock(&eth_if->mtx);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_TXSER_FORCE_LANE_PD_0 : EMAC_CMN_TXSER_FORCE_LANE_PD_1;
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val &= ~bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_TXSER_DIG_RESET_N_0 : EMAC_CMN_TXSER_DIG_RESET_N_1;
	val &= ~bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val |= bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_ser_pwr_up_and_reset);

void adrv906x_eth_cmn_deser_pwr_down(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, bit_mask;

	mutex_lock(&eth_if->mtx);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_RXDES_FORCE_LANE_PD_0 : EMAC_CMN_RXDES_FORCE_LANE_PD_1;
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val |= bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(10, 20);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_deser_pwr_down);

void adrv906x_eth_cmn_deser_pwr_up_and_reset(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, bit_mask;

	mutex_lock(&eth_if->mtx);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_RXDES_FORCE_LANE_PD_0 : EMAC_CMN_RXDES_FORCE_LANE_PD_1;
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val &= ~bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);

	bit_mask = (adrv906x_dev->port == 0) ?
		   EMAC_CMN_RXDES_DIG_RESET_N_0 : EMAC_CMN_RXDES_DIG_RESET_N_1;
	val &= ~bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val |= bit_mask;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);

	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_deser_pwr_up_and_reset);

int adrv906x_eth_cmn_rst_reg(void __iomem *regs)
{
	unsigned int val;

	val = REGMAP_RESET_SWITCH
	      | REGMAP_RESET_PCS_MAC0
	      | REGMAP_RESET_PCS_MAC1
	      | REGMAP_RESET_MACSEC0
	      | REGMAP_RESET_MACSEC1;

	iowrite32(val, regs + EMAC_CMN_RST_REG);
	iowrite32(0, regs + EMAC_CMN_RST_REG);

	return 0;
}

void adrv906x_eth_cmn_recovered_clk_config(struct adrv906x_eth_dev *adrv906x_dev)
{
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	struct net_device *ndev = adrv906x_dev->ndev;
	struct phy_device *phydev = ndev->phydev;
	u32 val;

	mutex_lock(&eth_if->mtx);
	val = (phydev->speed == SPEED_25000) ? eth_if->recovered_clk_div_25g - 1 :
	      eth_if->recovered_clk_div_10g - 1;
	val = FIELD_PREP(EMAC_CMN_RECOVERED_CLK_DIV_0, val);
	val |= FIELD_PREP(EMAC_CMN_RECOVERED_CLK_DIV_1, val);
	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL1);
	mutex_unlock(&eth_if->mtx);
}

void adrv906x_eth_cmn_mode_cfg(struct adrv906x_eth_dev *adrv906x_dev)
{
	void __iomem *regs = adrv906x_dev->parent->emac_cmn_regs;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct net_device *ndev = adrv906x_dev->ndev;
	struct phy_device *phydev = ndev->phydev;
	u32 val;

	mutex_lock(&eth_if->mtx);
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);

	if (phydev->speed == SPEED_10000)
		val |= EMAC_CMN_TX_BIT_REPEAT_RATIO;
	else
		val &= ~EMAC_CMN_TX_BIT_REPEAT_RATIO;

	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);
	mutex_unlock(&eth_if->mtx);
}

void adrv906x_eth_cmn_init(void __iomem *regs, bool switch_enabled, bool macsec_enabled)
{
	unsigned int val1, val2, val3;

	val1 = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val2 = ioread32(regs + EMAC_CMN_DIGITAL_CTRL0);
	val3 = ioread32(regs + EMAC_CMN_DIGITAL_CTRL3);

	val1 |= EMAC_CMN_RXDES_FORCE_LANE_PD_0 |
		EMAC_CMN_RXDES_FORCE_LANE_PD_1 |
		EMAC_CMN_TXSER_FORCE_LANE_PD_0 |
		EMAC_CMN_TXSER_FORCE_LANE_PD_1;
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(10, 20);
	val1 &= ~(EMAC_CMN_RXDES_FORCE_LANE_PD_0 |
		  EMAC_CMN_RXDES_FORCE_LANE_PD_1 |
		  EMAC_CMN_TXSER_FORCE_LANE_PD_0 |
		  EMAC_CMN_TXSER_FORCE_LANE_PD_1);
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);

	val1 &= ~EMAC_CMN_SERDES_REG_RESET_N;
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val1 |= EMAC_CMN_SERDES_REG_RESET_N;
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);

	val1 &= ~(EMAC_CMN_TXSER_DIG_RESET_N_0 |
		  EMAC_CMN_TXSER_DIG_RESET_N_1);
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val1 |= EMAC_CMN_TXSER_DIG_RESET_N_0 |
		EMAC_CMN_TXSER_DIG_RESET_N_1;
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);

	val1 &= ~(EMAC_CMN_RXDES_DIG_RESET_N_0 |
		  EMAC_CMN_RXDES_DIG_RESET_N_1);
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val1 |= EMAC_CMN_RXDES_DIG_RESET_N_0 |
		EMAC_CMN_RXDES_DIG_RESET_N_1;
	iowrite32(val1, regs + EMAC_CMN_PHY_CTRL);

	val2 |= EMAC_CMN_RX_LINK0_EN
		| EMAC_CMN_RX_LINK1_EN
		| EMAC_CMN_TX_LINK0_EN
		| EMAC_CMN_TX_LINK1_EN;
#if IS_ENABLED(CONFIG_MACSEC)
	if (macsec_enabled)
		val2 &= ~EMAC_CMN_MACSEC_BYPASS_EN;
#endif

	if (switch_enabled) {
		val2 |= EMAC_CMN_SW_PORT0_EN |
			EMAC_CMN_SW_PORT1_EN |
			EMAC_CMN_SW_PORT2_EN;
		val2 &= ~(EMAC_CMN_SW_LINK0_BYPASS_EN |
			  EMAC_CMN_SW_LINK1_BYPASS_EN);
		val3 |= EMAC_CMN_SW_PORT2_DSA_INSERT_EN;
	} else {
		val2 |= EMAC_CMN_SW_LINK0_BYPASS_EN |
			EMAC_CMN_SW_LINK1_BYPASS_EN;
		val2 &= ~(EMAC_CMN_SW_PORT0_EN |
			  EMAC_CMN_SW_PORT1_EN |
			  EMAC_CMN_SW_PORT2_EN);
		val3 &= ~EMAC_CMN_SW_PORT2_DSA_INSERT_EN;
	}

	iowrite32(val2, regs + EMAC_CMN_DIGITAL_CTRL0);
	iowrite32(val3, regs + EMAC_CMN_DIGITAL_CTRL3);
}

void adrv906x_cmn_pcs_link_drop_cnt_clear(struct adrv906x_eth_if *adrv906x_eth)
{
	void __iomem *regs;
	u32 val;

	regs = adrv906x_eth->emac_cmn_regs;

	mutex_lock(&adrv906x_eth->mtx);
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL4);
	val |= EMAC_CMN_CLEAR_PCS_STATUS_NE_CNT;
	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL4);
	val &= ~EMAC_CMN_CLEAR_PCS_STATUS_NE_CNT;
	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL4);
	mutex_unlock(&adrv906x_eth->mtx);
}

ssize_t adrv906x_cmn_pcs_link_drop_cnt_get(struct adrv906x_eth_if *adrv906x_eth, char *buf)
{
	void __iomem *regs;
	u8 cnt0, cnt1;
	unsigned long offset;
	u32 val;

	regs = adrv906x_eth->emac_cmn_regs;

	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL4);
	cnt0 = FIELD_GET(EMAC_CMN_PCS_STATUS_NE_CNT_0, val);
	cnt1 = FIELD_GET(EMAC_CMN_PCS_STATUS_NE_CNT_1, val);

	offset = sprintf(buf, "port 0 link failures: %d\n", cnt0);
	offset += sprintf(buf + offset, "port 1 link failures: %d\n", cnt1);

	return offset;
}

ssize_t adrv906x_cmn_recovered_clock_output_get(struct device *dev, char *buf)
{
	struct adrv906x_eth_dev *adrv906x_dev;
	int enabled, selected, result;
	void __iomem *regs;
	unsigned int val;

	adrv906x_dev = dev_get_drvdata(dev);
	regs = adrv906x_dev->parent->emac_cmn_regs;
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL0);

	enabled = (adrv906x_dev->port == 0) ?
		  FIELD_GET(EMAC_CMN_CDR_DIV_PORT0_EN, val) :
		  FIELD_GET(EMAC_CMN_CDR_DIV_PORT1_EN, val);
	selected = (FIELD_GET(EMAC_CMN_CDR_SEL, val) == adrv906x_dev->port) ? 1 : 0;
	result = sprintf(buf, "Selected as recovered_clk source: %s\n",
			 selected && enabled ? "Yes" : "No");

	return result;
}

ssize_t adrv906x_cmn_recovered_clock_output_set(struct device *dev, const char *buf, size_t cnt)
{
	struct adrv906x_eth_dev *adrv906x_dev;
	void __iomem *regs;
	int enable, err;
	u32 val;

	adrv906x_dev = dev_get_drvdata(dev);
	regs = adrv906x_dev->parent->emac_cmn_regs;

	err = kstrtoint(buf, 10, &enable);
	if (err) {
		dev_err(dev, "recovered_clock_output: invalid input");
		return err;
	}
	if (enable < 0 || enable > 1) {
		dev_err(dev, "recovered_clock_output: input out of range");
		return -EINVAL;
	}

	mutex_lock(&adrv906x_dev->parent->mtx);
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL0);
	if (enable) {
		if (adrv906x_dev->port == 0) {
			val |= EMAC_CMN_CDR_DIV_PORT0_EN;
			val &= ~EMAC_CMN_CDR_SEL;
		} else {
			val |= EMAC_CMN_CDR_DIV_PORT1_EN;
			val |= EMAC_CMN_CDR_SEL;
		}
	} else {
		val &= (adrv906x_dev->port == 0) ? ~EMAC_CMN_CDR_DIV_PORT0_EN :
		       ~EMAC_CMN_CDR_DIV_PORT1_EN;
	}

	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL0);
	mutex_unlock(&adrv906x_dev->parent->mtx);

	return cnt;
}

void adrv906x_cmn_set_phy_loopback(struct adrv906x_eth_dev *adrv906x_dev, bool enable)
{
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs;
	u32 ctrl0, ctrl2, loopback_bit, enable_bits;

	if (adrv906x_dev->port == 0) {
		loopback_bit = EMAC_CMN_LOOPBACK_BYPASS_DESER_0;
		enable_bits = EMAC_CMN_RX_LINK0_EN | EMAC_CMN_TX_LINK0_EN;
	} else {
		loopback_bit = EMAC_CMN_LOOPBACK_BYPASS_DESER_1;
		enable_bits = EMAC_CMN_RX_LINK1_EN | EMAC_CMN_TX_LINK1_EN;
	}

	regs = adrv906x_dev->parent->emac_cmn_regs;

	mutex_lock(&eth_if->mtx);
	ctrl0 = ioread32(regs + EMAC_CMN_DIGITAL_CTRL0);
	ctrl2 = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);

	ctrl0 &= ~enable_bits;
	iowrite32(ctrl0, regs + EMAC_CMN_DIGITAL_CTRL0);

	if (enable)
		ctrl2 |= loopback_bit;
	else
		ctrl2 &= ~loopback_bit;

	iowrite32(ctrl2, regs + EMAC_CMN_DIGITAL_CTRL2);

	ctrl0 |= enable_bits;
	iowrite32(ctrl0, regs + EMAC_CMN_DIGITAL_CTRL0);
	mutex_unlock(&eth_if->mtx);
}

void adrv906x_cmn_set_mac_loopback(struct adrv906x_eth_dev *adrv906x_dev, bool enable)
{
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs;
	u32 val, loopback_bit;

	regs = adrv906x_dev->parent->emac_cmn_regs;
	loopback_bit = adrv906x_dev->port == 0 ?
		       EMAC_CMN_LOOPBACK_BYPASS_MAC_0 : EMAC_CMN_LOOPBACK_BYPASS_MAC_1;

	mutex_lock(&eth_if->mtx);
	if (enable) {
		val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);
		val |= loopback_bit;
		iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);
	} else {
		val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);
		val &= ~loopback_bit;
		iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);
	}
	mutex_unlock(&eth_if->mtx);
}
