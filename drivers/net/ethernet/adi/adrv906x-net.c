// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/if_vlan.h>
#include <linux/string.h>
#include <linux/bitfield.h>
#include "adrv906x-ndma.h"
#include "adrv906x-macsec-ext.h"
#include "adrv906x-mac.h"
#include "adrv906x-switch.h"
#include "adrv906x-ethtool.h"
#include "adrv906x-net.h"
#include "adrv906x-mdio.h"
#include "adrv906x-tsu.h"

/* OIF register RX */
#define OIF_RX_CTRL_EN                      BIT(0)
/* OIF register TX */
#define OIF_CFG_TX_EN                       BIT(0)
#define OIF_CFG_TX_IPG                      GENMASK(10, 8)
#define OIF_CFG_TX_IPG_VAL                  0x600
/* default recovered clk divs */
#define DEFAULT_RECOVERED_CLK_DIV_10G       22
#define DEFAULT_RECOVERED_CLK_DIV_25G       55

static char *macaddr[MAX_NETDEV_NUM];
module_param_array(macaddr, charp, 0, 0644);
MODULE_PARM_DESC(macaddr, "set dev0 and dev1 mac addresses via kernel module parameter");
static u64 default_multicast_list[MAX_MULTICAST_FILTERS] = {
	0x0000011B19000000, 0x00000180C200000E, 0x00000180C2000003
};

void adrv906x_eth_cmn_serdes_tx_sync_trigger(struct net_device *ndev, u32 lane)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val, trig;

	trig = (lane == 0) ? EMAC_CMN_TXSER_SYNC_TRIGGER_0 : EMAC_CMN_TXSER_SYNC_TRIGGER_1;

	mutex_lock(&eth_if->mtx);
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val |= trig;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val &= ~trig;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_serdes_tx_sync_trigger);

void adrv906x_eth_cmn_serdes_reset_4pack(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	unsigned int val;

	mutex_lock(&eth_if->mtx);
	val = ioread32(regs + EMAC_CMN_PHY_CTRL);
	val &= ~EMAC_CMN_SERDES_REG_RESET_N;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	usleep_range(1, 10);
	val |= EMAC_CMN_SERDES_REG_RESET_N;
	iowrite32(val, regs + EMAC_CMN_PHY_CTRL);
	mutex_unlock(&eth_if->mtx);
}
EXPORT_SYMBOL(adrv906x_eth_cmn_serdes_reset_4pack);

struct adrv906x_macsec_priv *adrv906x_macsec_get(struct net_device *netdev)
{
#if IS_ENABLED(CONFIG_MACSEC)
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(netdev);

	return &adrv906x_dev->macsec;
#else
	return NULL;
#endif // IS_ENABLED(CONFIG_MACSEC)
}

static int adrv906x_eth_cmn_rst_reg(void __iomem *regs)
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

static void adrv906x_eth_cmn_recovered_clk_config(struct adrv906x_eth_dev *adrv906x_dev)
{
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	void __iomem *regs = eth_if->emac_cmn_regs;
	u32 val;

	val = (adrv906x_dev->link_speed == SPEED_25000) ? eth_if->recovered_clk_div_25g - 1 :
	      eth_if->recovered_clk_div_10g - 1;
	val = FIELD_PREP(EMAC_CMN_RECOVERED_CLK_DIV_0, val);
	val |= FIELD_PREP(EMAC_CMN_RECOVERED_CLK_DIV_1, val);
	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL1);
}

static void adrv906x_eth_cmn_mode_cfg(struct adrv906x_eth_dev *adrv906x_dev)
{
	void __iomem *regs = adrv906x_dev->parent->emac_cmn_regs;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	u32 val;

	mutex_lock(&eth_if->mtx);
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL2);

	if (adrv906x_dev->link_speed == SPEED_10000)
		val |= EMAC_CMN_TX_BIT_REPEAT_RATIO;
	else
		val &= ~EMAC_CMN_TX_BIT_REPEAT_RATIO;

	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL2);
	mutex_unlock(&eth_if->mtx);
}

static void adrv906x_eth_cdr_get_recovered_clk_divs(struct device_node *np,
						    struct adrv906x_eth_if *eth_if)
{
	struct device *dev = eth_if->dev;
	u32 val;

	if (of_property_read_u32(np, "recovered_clk_10g", &val)) {
		eth_if->recovered_clk_div_10g = DEFAULT_RECOVERED_CLK_DIV_10G;
		dev_info(dev, "dt: recovered_clk_10g is missing, use default %d",
			 eth_if->recovered_clk_div_10g);
	} else {
		eth_if->recovered_clk_div_10g = val;
	}
	if (of_property_read_u32(np, "recovered_clk_25g", &val)) {
		eth_if->recovered_clk_div_25g = DEFAULT_RECOVERED_CLK_DIV_25G;
		dev_info(dev, "dt: recovered_clk_25g is missing, use default %d",
			 eth_if->recovered_clk_div_25g);
	} else {
		eth_if->recovered_clk_div_25g = val;
	}
}

static void adrv906x_eth_cmn_init(void __iomem *regs, bool switch_enabled, bool macsec_enabled)
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

static ssize_t adrv906x_pcs_link_drop_cnt_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t cnt)
{
	struct adrv906x_eth_if *adrv906x_eth;
	void __iomem *regs;
	u32 val;

	adrv906x_eth = dev_get_drvdata(dev);
	regs = adrv906x_eth->emac_cmn_regs;

	mutex_lock(&adrv906x_eth->mtx);
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL4);
	val |= EMAC_CMN_CLEAR_PCS_STATUS_NE_CNT;
	iowrite32(val, regs + EMAC_CMN_DIGITAL_CTRL4);
	mutex_unlock(&adrv906x_eth->mtx);

	return cnt;
}

static ssize_t adrv906x_pcs_link_drop_cnt_show(struct device *dev,
					       struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_if *adrv906x_eth;
	void __iomem *regs;
	u8 cnt0, cnt1;
	int offset;
	u32 val;

	adrv906x_eth = dev_get_drvdata(dev);
	regs = adrv906x_eth->emac_cmn_regs;

	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL4);
	cnt0 = FIELD_GET(EMAC_CMN_PCS_STATUS_NE_CNT_0, val);
	cnt1 = FIELD_GET(EMAC_CMN_PCS_STATUS_NE_CNT_1, val);

	offset = sprintf(buf, "port 0 link failures: %d\n", cnt0);
	offset += sprintf(buf + offset, "port 1 link failures: %d\n", cnt1);

	return offset;
}

static DEVICE_ATTR_RW(adrv906x_pcs_link_drop_cnt);

static struct attribute *adrv906x_eth_debug_attrs[] = {
	&dev_attr_adrv906x_pcs_link_drop_cnt.attr,
	NULL,
};

ATTRIBUTE_GROUPS(adrv906x_eth_debug);

static ssize_t adrv906x_eth_cdr_div_out_enable_show(struct device *dev,
						    struct device_attribute *attr, char *buf)
{
	struct adrv906x_eth_dev *adrv906x_dev;
	int result, cdr_selected;
	u8 cdr_div_out_enable;
	void __iomem *regs;
	unsigned int val;

	adrv906x_dev = dev_get_drvdata(dev);
	regs = adrv906x_dev->parent->emac_cmn_regs;
	val = ioread32(regs + EMAC_CMN_DIGITAL_CTRL0);

	cdr_div_out_enable = (adrv906x_dev->port == 0) ?
			     FIELD_GET(EMAC_CMN_CDR_DIV_PORT0_EN, val) :
			     FIELD_GET(EMAC_CMN_CDR_DIV_PORT1_EN, val);
	cdr_selected = (FIELD_GET(EMAC_CMN_CDR_SEL, val) == adrv906x_dev->port) ? 1 : 0;
	result = sprintf(buf, "%s CDR enabled: %i\n%s CDR selected: %i\n", kobject_name(&dev->kobj),
			 cdr_div_out_enable, kobject_name(&dev->kobj), cdr_selected);

	return result;
}

static ssize_t adrv906x_eth_cdr_div_out_enable_store(struct device *dev,
						     struct device_attribute *attr,
						     const char *buf, size_t cnt)
{
	struct adrv906x_eth_dev *adrv906x_dev;
	void __iomem *regs;
	int enable, err;
	u32 val;

	adrv906x_dev = dev_get_drvdata(dev);
	regs = adrv906x_dev->parent->emac_cmn_regs;

	err = kstrtoint(buf, 10, &enable);
	if (err) {
		dev_err(dev, "adrv906x_eth_cdr_div_out_enable: Invalid input");
		return err;
	}
	if (enable < 0 || enable > 1) {
		dev_err(dev, "adrv906x_eth_cdr_div_out_enable: input out of range");
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

static DEVICE_ATTR_RW(adrv906x_eth_cdr_div_out_enable);

static void adrv906x_eth_adjust_link(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_tsu *tsu;
	struct phy_device *phydev = ndev->phydev;

	tsu = &(adrv906x_dev->tsu);
	adrv906x_tsu_set_speed(tsu, phydev);

	if (!phydev->link) {
		if (adrv906x_dev->link_speed) {
			adrv906x_dev->link_speed = 0;
			netdev_info(ndev, "%s: link down", ndev->name);
		}
		return;
	}

	if (adrv906x_dev->link_speed == phydev->speed &&
	    adrv906x_dev->link_duplex == phydev->duplex)
		return;

	adrv906x_dev->link_speed = phydev->speed;
	adrv906x_dev->link_duplex = phydev->duplex;

	adrv906x_eth_cmn_mode_cfg(adrv906x_dev);
	adrv906x_eth_cmn_recovered_clk_config(adrv906x_dev);

	phy_print_status(phydev);
}

/* TBD - add clock, phy configs etc */
static int adrv906x_eth_phylink_register(struct net_device *ndev, struct device_node *port_np)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct device *dev = adrv906x_dev->dev;
	struct device_node *phy_node;
	struct phy_device *phy_dev;
	phy_interface_t iface;
	int ret;

	phy_node = of_parse_phandle(port_np, "phy-handle", 0);
	if (!phy_node) {
		dev_err(dev, "dt: failed to retrieve phy phandle");
		return -ENODEV;
	}

	ret = of_get_phy_mode(port_np, &iface);
	if (ret) {
		dev_err(dev, "dt: phy-mode property missing");
		return -EINVAL;
	}

	phy_dev = of_phy_connect(ndev, phy_node, &adrv906x_eth_adjust_link, 0, iface);
	if (!phy_dev) {
		netdev_err(ndev, "could not connect to PHY");
		return -ENODEV;
	}

	return 0;
}

static void __add_tx_hw_tstamp(struct sk_buff *skb, struct timespec64 ts)
{
	struct skb_shared_hwtstamps shhwtstamps;

	if (skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS) {
		memset(&shhwtstamps, 0, sizeof(shhwtstamps));
		shhwtstamps.hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
		skb_tstamp_tx(skb, &shhwtstamps);
	}
}

static void __add_rx_hw_tstamp(struct sk_buff *skb, struct timespec64 ts)
{
	struct skb_shared_hwtstamps *hwtstamps;

	hwtstamps = skb_hwtstamps(skb);
	hwtstamps->hwtstamp = ktime_set(ts.tv_sec, ts.tv_nsec);
}

static void adrv906x_eth_tx_callback(struct sk_buff *skb, unsigned int port_id,
				     struct timespec64 ts, void *cb_param)
{
	struct net_device *ndev = (struct net_device *)cb_param;
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *adrv906x_eth = adrv906x_dev->parent;
	unsigned long flags;

	adrv906x_dev = adrv906x_eth->adrv906x_dev[port_id];
	ndev = adrv906x_dev->ndev;

	spin_lock_irqsave(&adrv906x_dev->lock, flags);
	if (--adrv906x_dev->tx_frames_pending < adrv906x_eth->tx_max_frames_pending)
		netif_wake_queue(ndev);
	spin_unlock_irqrestore(&adrv906x_dev->lock, flags);

	__add_tx_hw_tstamp(skb, ts);
	dev_kfree_skb(skb);
}

static int adrv906x_eth_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	int port = adrv906x_dev->port;
	bool hw_tstamp_en, dsa_en;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&adrv906x_dev->lock, flags);
	if (++adrv906x_dev->tx_frames_pending >= eth_if->tx_max_frames_pending)
		netif_stop_queue(ndev);
	spin_unlock_irqrestore(&adrv906x_dev->lock, flags);

	hw_tstamp_en = (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) ? 1 : 0;
	if (hw_tstamp_en)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	skb_tx_timestamp(skb);

	dsa_en = es->enabled ? 1 : 0;

	ret = adrv906x_ndma_start_xmit(ndma_dev, skb, port, hw_tstamp_en, dsa_en);

	return ret ? NETDEV_TX_BUSY : NETDEV_TX_OK;
}

static void __skb_pvid_pop(struct net_device *ndev, struct sk_buff *skb)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	struct ethhdr *hdr = (struct ethhdr *)skb->data;
	unsigned short ether_type = ntohs(hdr->h_proto);
	struct vlan_hdr *vhdr;
	unsigned short vlan_tag;

	if (!es->enabled || ether_type != ETH_P_8021Q)
		return;

	vhdr = (struct vlan_hdr *)(skb->data + ETH_HLEN);
	vlan_tag = ntohs(vhdr->h_vlan_TCI);

	if ((vlan_tag & VLAN_VID_MASK) == es->pvid) {
		memmove(skb->data + VLAN_HLEN, skb->data, 2 * ETH_ALEN);
		skb_pull(skb, VLAN_HLEN);
	}
}

void adrv906x_eth_rx_callback(struct sk_buff *skb, unsigned int port_id,
			      struct timespec64 ts, void *cb_param)
{
	struct net_device *ndev = (struct net_device *)cb_param;
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;

	/* network interface is selected base on port_id from ndma rx status header */
	adrv906x_dev = eth_if->adrv906x_dev[port_id];
	ndev = adrv906x_dev->ndev;

	__skb_pvid_pop(ndev, skb);
	__add_rx_hw_tstamp(skb, ts);
	skb->protocol = eth_type_trans(skb, ndev);
	netif_receive_skb(skb);
}

static void adrv906x_eth_multicast_list(struct net_device *ndev)
{
	struct adrv906x_mac *mac;
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);

	mac = &adrv906x_dev->mac;

	if (ndev->flags & IFF_PROMISC)
		/* Promiscuous mode. */
		adrv906x_mac_promiscuous_mode_en(mac);
	else
		adrv906x_mac_promiscuous_mode_dis(mac);
}

static inline void __eth_hw_addr_gen(struct net_device *dev, const u8 *base_addr,
				     unsigned int id)
{
	u64 u = ether_addr_to_u64(base_addr);
	u8 addr[ETH_ALEN];

	u += id;
	u64_to_ether_addr(u, addr);
	memcpy(dev->dev_addr, addr, ETH_ALEN);
}

static int __set_mac_address(struct adrv906x_eth_dev *adrv906x_dev, struct device_node *port_np)
{
	struct device *dev = adrv906x_dev->dev;
	struct net_device *ndev = adrv906x_dev->ndev;
	int port = adrv906x_dev->port;
	const unsigned char *tmpaddr;
	struct sockaddr addr;

	memset(addr.sa_data, 0, sizeof(addr.sa_data));

	/* try to get mac address in following order:
	 *
	 * 1) module parameter via kernel command line
	 *     macaddr="dev 0 mac addr","dev 1 mac addr"
	 */
	if (macaddr[port])
		mac_pton(macaddr[port], addr.sa_data);
	/* 2) mac address in the device tree
	 * it is filled in by u-boot if it
	 * is not set
	 */
	if (!is_valid_ether_addr(addr.sa_data)) {
		if (port_np) {
			tmpaddr = of_get_mac_address(port_np);
			if (!IS_ERR(tmpaddr))
				ether_addr_copy(addr.sa_data, tmpaddr);
		}
	}
	/* 3) random mac address */
	if (!is_valid_ether_addr(addr.sa_data)) {
		/* Report it and use a random ethernet address instead */
		dev_warn(dev, "invalid MAC address: %pM, generate a random addr", addr.sa_data);
		/* assign random address */
		eth_random_addr(addr.sa_data);
	}

	return eth_mac_addr(ndev, &addr);
}

static void __set_default_multicast_filters(struct adrv906x_eth_dev *adrv906x_dev)
{
	struct adrv906x_mac *mac = &adrv906x_dev->mac;
	int i;

	for (i = 0; i < MAX_MULTICAST_FILTERS; i++)
		adrv906x_mac_set_multicast_filter(mac, default_multicast_list[i], i);
}

static int adrv906x_eth_change_mtu(struct net_device *ndev, int new_mtu)
{
	if (netif_running(ndev))
		return -EBUSY;

	ndev->mtu = new_mtu;

	netdev_info(ndev, "set mtu size to %d", ndev->mtu);

	return 0;
}

static int adrv906x_get_hwtstamp_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);

	if (copy_to_user(ifr->ifr_data, &adrv906x_dev->tstamp_config,
			 sizeof(adrv906x_dev->tstamp_config)))
		return -EFAULT;
	return 0;
}

static int adrv906x_set_hwtstamp_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct adrv906x_eth_if *eth_if = adrv906x_dev->parent;
	struct hwtstamp_config config;
	u32 ptp_mode;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		/* TODO  clear timestamp flag */
		break;

	case HWTSTAMP_TX_ON:
		/* TODO  set timestamp flag */
		break;

	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_ALL:
		/* time stamp any incoming packet */
		config.rx_filter = HWTSTAMP_FILTER_ALL;

		ptp_mode = eth_if->ethswitch.enabled ? NDMA_PTP_MODE_1 : NDMA_PTP_MODE_4;
		adrv906x_ndma_set_ptp_mode(ndma_dev, ptp_mode);

		break;
	default:
		return -ERANGE;
	}

	if (copy_to_user(ifr->ifr_data, &config, sizeof(config)))
		return -EFAULT;

	memcpy(&adrv906x_dev->tstamp_config, &config, sizeof(config));

	return 0;
}

static int adrv906x_config_eth_recov_clk(struct device *dev,
					 struct device_node *np)
{
	u32 reg, len;
	void *ptr;

	if (of_property_read_u32_index(np, "reg", 0, &reg))
		dev_err(dev, "dt: eth_recov_clk_np failed - skipping");
	if (of_property_read_u32_index(np, "reg", 1, &len))
		dev_err(dev, "dt: eth_recov_clk_np failed - skipping");

	ptr = ioremap(reg, len);
	iowrite32(0x1, ptr);
	iounmap(ptr);

	return 0;
}

static int adrv906x_get_oran_if_reg_addr(struct adrv906x_eth_dev *adrv906x_dev,
					 struct device_node *np)
{
	struct device *dev = adrv906x_dev->dev;
	u8 port_id = adrv906x_dev->port;
	u32 reg, len;

	/* get oif_rx address */
	if (of_property_read_u32_index(np, "reg", port_id * 4 + 0, &reg))
		goto err;
	if (of_property_read_u32_index(np, "reg", port_id * 4 + 1, &len))
		goto err;
	adrv906x_dev->oif.oif_rx = devm_ioremap(dev, reg, len);
	if (!adrv906x_dev->oif.oif_rx) {
		dev_err(dev, "ioremap oif rx reg failed!");
		return -ENOMEM;
	}

	/* get oif_tx address */
	if (of_property_read_u32_index(np, "reg", port_id * 4 + 2, &reg))
		goto err;
	if (of_property_read_u32_index(np, "reg", port_id * 4 + 3, &len))
		goto err;
	adrv906x_dev->oif.oif_tx = devm_ioremap(dev, reg, len);
	if (!adrv906x_dev->oif.oif_tx) {
		dev_err(dev, "ioremap oif tx reg failed!");
		return -ENOMEM;
	}
	return 0;
err:
	dev_err(dev, "check the reg property of oran_oif node");
	return -1;
}

static int adrv906x_eth_oran_if_tx_dis(struct adrv906x_oran_if *oif)
{
	u32 val;

	if (oif->oif_tx) {
		val = ioread32(oif->oif_tx) & ~OIF_CFG_TX_EN;
		iowrite32(val, oif->oif_tx);
	}
	return 0;
}

static int adrv906x_eth_oran_if_en(struct adrv906x_oran_if *oif)
{
	u32 val;

	if (oif->oif_rx && oif->oif_tx) {
		val = ioread32(oif->oif_rx);
		val |= OIF_RX_CTRL_EN;
		iowrite32(val, oif->oif_rx);

		val = ioread32(oif->oif_tx);
		val |= (OIF_CFG_TX_EN | OIF_CFG_TX_IPG_VAL);
		iowrite32(val, oif->oif_tx);
	}
	return 0;
}

static int adrv906x_eth_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	int ret;

	switch (cmd) {
	case SIOCGHWTSTAMP:
		ret = adrv906x_get_hwtstamp_config(ndev, ifr);
		break;

	case SIOCSHWTSTAMP:
		ret = adrv906x_set_hwtstamp_config(ndev, ifr);
		break;

	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int adrv906x_eth_switch_reset_soft_pre(void *arg)
{
	struct adrv906x_eth_if *eth_if = (struct adrv906x_eth_if *)arg;
	int ret;
	int i;

	for (i = 0; i < MAX_NETDEV_NUM; i++)
		adrv906x_mac_rx_path_dis(&eth_if->adrv906x_dev[i]->mac);

	ret = adrv906x_eth_oran_if_tx_dis(&eth_if->adrv906x_dev[1]->oif);

	return ret;
}

static int adrv906x_eth_switch_reset_soft_post(void *arg)
{
	struct adrv906x_eth_if *eth_if = (struct adrv906x_eth_if *)arg;
	int ret;
	int i;

	ret = adrv906x_eth_oran_if_en(&eth_if->adrv906x_dev[1]->oif);
	for (i = 0; i < MAX_NETDEV_NUM; i++)
		adrv906x_mac_rx_path_en(&eth_if->adrv906x_dev[i]->mac);

	return ret;
}

static int adrv906x_eth_open(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct device *dev = adrv906x_dev->dev;
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;

	dev_info(dev, "%s called", __func__);

	adrv906x_eth_oran_if_en(&adrv906x_dev->oif);

	phy_start(ndev->phydev);

	adrv906x_ndma_open(ndma_dev, adrv906x_eth_tx_callback, adrv906x_eth_rx_callback, ndev, false);

#if IS_ENABLED(CONFIG_MACSEC)
	if (adrv906x_dev->macsec.enabled)
		adrv906x_macsec_commonport_status_update(ndev);
#endif // IS_ENABLED(CONFIG_MACSEC)

	adrv906x_dev->tx_frames_pending = 0;

	netif_start_queue(ndev);
	return 0;
}

static int adrv906x_eth_stop(struct net_device *ndev)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);
	struct adrv906x_ndma_dev *ndma_dev = adrv906x_dev->ndma_dev;
	struct device *dev = adrv906x_dev->dev;

	dev_info(dev, "%s called", __func__);
	netif_stop_queue(ndev);
	adrv906x_ndma_close(ndma_dev);
	if (ndev->phydev)
		phy_stop(ndev->phydev);
	return 0;
}

static void adrv906x_get_rtnl_stats(struct adrv906x_eth_dev *adrv906x_dev)
{
	mutex_lock(&adrv906x_dev->mac.mac_hw_stats_lock);
	adrv906x_dev->rtnl_stats.tx_packets =
		adrv906x_dev->mac.hw_stats_tx.general_stats.unicast_pkts +
		adrv906x_dev->mac.hw_stats_tx.general_stats.multicast_pkts +
		adrv906x_dev->mac.hw_stats_tx.general_stats.broadcast_pkts;
	adrv906x_dev->rtnl_stats.tx_errors =
		adrv906x_dev->mac.hw_stats_tx.underflow +
		adrv906x_dev->mac.hw_stats_tx.general_stats.undersize_pkts +
		adrv906x_dev->mac.hw_stats_tx.general_stats.oversize_pkts;
	adrv906x_dev->rtnl_stats.tx_fifo_errors = adrv906x_dev->mac.hw_stats_tx.underflow;
	adrv906x_dev->rtnl_stats.tx_dropped =
		adrv906x_dev->mac.hw_stats_tx.general_stats.drop_events;
	adrv906x_dev->rtnl_stats.rx_packets =
		adrv906x_dev->mac.hw_stats_rx.general_stats.unicast_pkts +
		adrv906x_dev->mac.hw_stats_rx.general_stats.multicast_pkts +
		adrv906x_dev->mac.hw_stats_rx.general_stats.broadcast_pkts;
	adrv906x_dev->rtnl_stats.rx_errors =
		adrv906x_dev->mac.hw_stats_rx.general_stats.undersize_pkts +
		adrv906x_dev->mac.hw_stats_rx.general_stats.oversize_pkts +
		adrv906x_dev->mac.hw_stats_rx.crc_errors +
		adrv906x_dev->mac.hw_stats_rx.fragments +
		adrv906x_dev->mac.hw_stats_rx.jabbers +
		adrv906x_dev->mac.hw_stats_rx.mac_framing_error +
		adrv906x_dev->mac.hw_stats_rx.rs_framing_error;
	adrv906x_dev->rtnl_stats.rx_length_errors =
		adrv906x_dev->mac.hw_stats_rx.general_stats.undersize_pkts +
		adrv906x_dev->mac.hw_stats_rx.general_stats.oversize_pkts;
	adrv906x_dev->rtnl_stats.rx_over_errors = adrv906x_dev->mac.hw_stats_rx.overflow;
	adrv906x_dev->rtnl_stats.rx_crc_errors = adrv906x_dev->mac.hw_stats_rx.crc_errors;
	adrv906x_dev->rtnl_stats.rx_frame_errors = adrv906x_dev->mac.hw_stats_rx.mac_framing_error +
						   adrv906x_dev->mac.hw_stats_rx.rs_framing_error;
	adrv906x_dev->rtnl_stats.rx_fifo_errors = adrv906x_dev->mac.hw_stats_rx.overflow;
	adrv906x_dev->rtnl_stats.rx_missed_errors = adrv906x_dev->mac.hw_stats_rx.overflow;
	adrv906x_dev->rtnl_stats.rx_dropped =
		adrv906x_dev->mac.hw_stats_rx.mc_drop +
		adrv906x_dev->mac.hw_stats_rx.general_stats.drop_events;
	adrv906x_dev->rtnl_stats.multicast =
		adrv906x_dev->mac.hw_stats_rx.general_stats.multicast_pkts;
	mutex_unlock(&adrv906x_dev->mac.mac_hw_stats_lock);
}

static void adrv906x_eth_get_stats64(struct net_device *ndev,
				     struct rtnl_link_stats64 *stats)
{
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(ndev);

	adrv906x_get_rtnl_stats(adrv906x_dev);

	memcpy(stats, &adrv906x_dev->rtnl_stats, sizeof(*stats));
}

static const struct net_device_ops adrv906x_eth_ops = {
	.ndo_open		= adrv906x_eth_open,
	.ndo_stop		= adrv906x_eth_stop,
	.ndo_start_xmit		= adrv906x_eth_start_xmit,
	.ndo_set_rx_mode	= adrv906x_eth_multicast_list,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_mtu		= adrv906x_eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= adrv906x_eth_get_stats64,
	.ndo_do_ioctl		= adrv906x_eth_ioctl,
};

static const struct of_device_id adrv906x_eth_dt_ids[] = {
	{ .compatible = "adi,adrv906x-net", },
	{}
};

MODULE_DEVICE_TABLE(of, adrv906x_eth_dt_ids);

static int adrv906x_get_mac_reg_addr(struct adrv906x_eth_dev *adrv906x_dev,
				     struct device_node *port_np)
{
	u32 reg, len;
	struct device *dev = adrv906x_dev->dev;

	/* Get xmac device address */
	of_property_read_u32_index(port_np, "reg", 0, &reg);
	of_property_read_u32_index(port_np, "reg", 1, &len);
	adrv906x_dev->mac.xmac = devm_ioremap(dev, reg, len);
	if (!adrv906x_dev->mac.xmac) {
		dev_err(dev, "ioremap xmac failed");
		return -ENOMEM;
	}

	/* Get emac_tx device address */
	of_property_read_u32_index(port_np, "reg", 2, &reg);
	of_property_read_u32_index(port_np, "reg", 3, &len);
	adrv906x_dev->mac.emac_tx = devm_ioremap(dev, reg, len);
	if (!adrv906x_dev->mac.emac_tx) {
		dev_err(dev, "ioremap emac tx failed");
		return -ENOMEM;
	}

	/* Get emac_rx device address */
	of_property_read_u32_index(port_np, "reg", 4, &reg);
	of_property_read_u32_index(port_np, "reg", 5, &len);
	adrv906x_dev->mac.emac_rx = devm_ioremap(dev, reg, len);
	if (!adrv906x_dev->mac.emac_rx) {
		dev_err(dev, "ioremap emac rx failed");
		return -ENOMEM;
	}

	return 0;
}

static struct device_node *
adrv906x_get_eth_child_node(struct device_node *ether_np, int id)
{
	struct device_node *port_np;
	unsigned int port_id;

	for_each_child_of_node(ether_np, port_np) {
		/* It is not a 'port' node, continue */
		if (strcmp(port_np->name, "port"))
			continue;

		if (of_property_read_u32(port_np, "id", &port_id) < 0)
			continue;

		if (port_id == id)
			return port_np;
	}
	/* Not found */
	return NULL;
}

static int adrv906x_eth_dev_reg(struct platform_device *pdev,
				struct adrv906x_eth_dev **adrv906x_dev)
{
	struct net_device *ndev;
	struct adrv906x_eth_dev *priv;
	struct device *dev = &pdev->dev;
	int ret;

	ndev = devm_alloc_etherdev_mqs(dev, sizeof(struct adrv906x_eth_dev), 1, 1);
	ndev->netdev_ops = &adrv906x_eth_ops;
	ndev->ethtool_ops = &adrv906x_ethtool_ops;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;
	priv->pdev = pdev;
	ret = register_netdev(ndev);

	if (ret) {
		dev_err(dev, "error %d initializing ethernet device ...", ret);
		return ret;
	}
	*adrv906x_dev = priv;

	dev_info(dev, "initialized %s", ndev->name);

	return 0;
}

static int adrv906x_eth_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct adrv906x_eth_if *eth_if;
	struct adrv906x_eth_dev *adrv906x_dev;
	struct adrv906x_tsu *tsu;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *eth_ports_np, *port_np, *oran_if_np, *eth_recov_clk_np, *ndma_np, *mdio_np;
	struct adrv906x_ndma_dev *ndma_devs[MAX_NETDEV_NUM] = { NULL };
	unsigned int ndma_num;
	int ret, i;

	eth_if = devm_kzalloc(dev, sizeof(struct adrv906x_eth_if), GFP_KERNEL);
	if (!eth_if)
		return -ENOMEM;

	eth_if->dev = dev;
	dev_set_drvdata(dev, eth_if);

	eth_if->emac_cmn_regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eth_if->emac_cmn_regs)) {
		dev_err(dev, "dt: reg-property missing or broken for common regs");
		ret = PTR_ERR(eth_if->emac_cmn_regs);
		goto error;
	}

	adrv906x_eth_cmn_rst_reg(eth_if->emac_cmn_regs);

	mutex_init(&eth_if->mtx);

	adrv906x_eth_cdr_get_recovered_clk_divs(np, eth_if);

	mdio_np = of_get_child_by_name(np, "mdio_if");
	adrv906x_mdio_probe(pdev, ndev, mdio_np);

	/* Get child node ethernet-ports */
	eth_ports_np = of_get_child_by_name(np, "ethernet-ports");
	if (!eth_ports_np) {
		dev_err(dev, "dt: ethernet-ports property missing");
		ret = -ENODEV;
		goto error;
	}

	oran_if_np = of_get_child_by_name(np, "oran_if");
	if (!oran_if_np)
		dev_warn(dev, "dt: oran_if node missing - skipping");

	eth_recov_clk_np = of_get_child_by_name(np, "eth_recov_clk");
	if (eth_recov_clk_np)
		adrv906x_config_eth_recov_clk(dev, eth_recov_clk_np);
	else
		dev_warn(dev, "dt: eth_recov_clk_np node missing - skipping");

	for (i = 0; i < MAX_NETDEV_NUM; i++) {
		/* Get port@i of node ethernet-ports */
		port_np = adrv906x_get_eth_child_node(eth_ports_np, i);
		if (!port_np)
			continue;

		ret = adrv906x_eth_dev_reg(pdev, &adrv906x_dev);
		if (ret)
			goto error;

		tsu = &(adrv906x_dev->tsu);
		ret = adrv906x_tsu_setup(pdev, tsu, port_np);
		if (ret)
			goto error;

		eth_if->adrv906x_dev[i] = adrv906x_dev;
		adrv906x_dev->parent = eth_if;
		adrv906x_dev->port = i;
		adrv906x_dev->dev = dev;
		ndev = adrv906x_dev->ndev;
		ndev->max_mtu = ETH_DATA_LEN; /* TODO: when MSP fragmentation will be supported, */
		/* this should be changed to ETH_MAX_MTU */
		ndev->min_mtu = ETH_MIN_MTU;
		ndev->mtu = ETH_DATA_LEN;
		/* Headroom required for ndma headers for tx packets */
		ndev->needed_headroom += NDMA_TX_HDR_SOF_SIZE;

		ret = device_create_file(&adrv906x_dev->ndev->dev,
					 &dev_attr_adrv906x_eth_cdr_div_out_enable);
		dev_set_drvdata(&adrv906x_dev->ndev->dev, adrv906x_dev);
		if (ret)
			goto error_delete_cdr_div_out_enable_sysfs;

#if IS_ENABLED(CONFIG_MACSEC)
		adrv906x_macsec_probe(pdev, ndev, port_np);
#endif // IS_ENABLED(CONFIG_MACSEC)

		/* read ndma dt */
		ndma_np = of_parse_phandle(port_np, "ndma-handle", 0);
		if (!ndma_np) {
			dev_err(dev, "dt: failed to retrieve ndma phandle from device tree");
			return -ENODEV;
		}

		if (of_property_read_u32(ndma_np, "id", &ndma_num)) {
			dev_err(dev, "dt: failed to retrieve ndma device id from device tree");
			return -ENODEV;
		}

		if (!ndma_devs[ndma_num]) {
			ndma_devs[ndma_num] = devm_kzalloc(dev, sizeof(struct adrv906x_ndma_dev),
							   GFP_KERNEL);
			if (!ndma_devs[ndma_num])
				return -ENOMEM;

			ret = adrv906x_ndma_probe(pdev, ndev, ndma_np,
						  ndma_devs[ndma_num]);
			if (ret) {
				dev_err(dev, "failed to probe ndma device");
				goto error_unregister_netdev;
			}
		}

		adrv906x_dev->ndma_dev = ndma_devs[ndma_num];
		dev_info(dev, "%s: connected to ndma%d", ndev->name, ndma_num);

		ret = adrv906x_get_mac_reg_addr(adrv906x_dev, port_np);
		if (ret) {
			dev_err(dev, "failed to get mac register addresses");
			goto error_unregister_netdev;
		}

		__set_mac_address(adrv906x_dev, port_np);
		adrv906x_mac_init(&adrv906x_dev->mac, NDMA_MAX_FRAME_SIZE_VALUE);

		dev_info(dev, "inited mac: id:0x%x, ver:0x%x, cap:0x%x",
			 adrv906x_dev->mac.id, adrv906x_dev->mac.version, adrv906x_dev->mac.cap);

		__set_default_multicast_filters(adrv906x_dev);

		ret = adrv906x_eth_phylink_register(ndev, port_np);
		if (ret)
			goto error_unregister_netdev;

		if (oran_if_np) {
			adrv906x_get_oran_if_reg_addr(adrv906x_dev, oran_if_np);
			adrv906x_eth_oran_if_en(&adrv906x_dev->oif);
		}

		spin_lock_init(&adrv906x_dev->lock);
	}

	ret = adrv906x_switch_probe(&eth_if->ethswitch, pdev,
				    &adrv906x_eth_switch_reset_soft_pre,
				    &adrv906x_eth_switch_reset_soft_post);
	if (ret)
		dev_warn(dev, "failed to probe switch - falling back to non-switch mode");

	mutex_lock(&eth_if->mtx);
#if IS_ENABLED(CONFIG_MACSEC)
	adrv906x_eth_cmn_init(eth_if->emac_cmn_regs, eth_if->ethswitch.enabled,
			      adrv906x_dev->macsec.enabled);
#else
	adrv906x_eth_cmn_init(eth_if->emac_cmn_regs, eth_if->ethswitch.enabled, false);
#endif
	mutex_unlock(&eth_if->mtx);

	if (eth_if->ethswitch.enabled) {
		ret = adrv906x_switch_init(&eth_if->ethswitch);
		if (ret)
			goto error_unregister_netdev;
	}

	eth_if->tx_max_frames_pending =
		eth_if->ethswitch.enabled ? NDMA_RING_SIZE / 2 : NDMA_RING_SIZE;

	platform_set_drvdata(pdev, eth_if);

	ret = sysfs_create_group(&pdev->dev.kobj, *adrv906x_eth_debug_groups);
	if (ret)
		goto error_delete_groups;

	return 0;

error_delete_cdr_div_out_enable_sysfs:
	device_remove_file(&eth_if->adrv906x_dev[i]->ndev->dev,
			   &dev_attr_adrv906x_eth_cdr_div_out_enable);
	dev_set_drvdata(&eth_if->adrv906x_dev[i]->ndev->dev, NULL);
error_delete_groups:
	sysfs_remove_groups(&pdev->dev.kobj, adrv906x_eth_debug_groups);
error_unregister_netdev:
	for (i = 0; i < MAX_NETDEV_NUM; i++)
		if (eth_if->adrv906x_dev[i] && eth_if->adrv906x_dev[i]->ndev)
			unregister_netdev(eth_if->adrv906x_dev[i]->ndev);
error:
	return ret;
}

static int adrv906x_eth_remove(struct platform_device *pdev)
{
	struct adrv906x_eth_if *eth_if = platform_get_drvdata(pdev);
	struct adrv906x_eth_switch *es = &eth_if->ethswitch;
	struct net_device *ndev;
	int i;

	mutex_destroy(&eth_if->mtx);
	sysfs_remove_groups(&pdev->dev.kobj, adrv906x_eth_debug_groups);

	for (i = 0; i < MAX_NETDEV_NUM; i++) {
		if (eth_if->adrv906x_dev[i]) {
			ndev = eth_if->adrv906x_dev[i]->ndev;
			device_remove_file(&eth_if->adrv906x_dev[i]->ndev->dev,
					   &dev_attr_adrv906x_eth_cdr_div_out_enable);
			dev_set_drvdata(&eth_if->adrv906x_dev[i]->ndev->dev, NULL);
			phy_disconnect(ndev->phydev);
			unregister_netdev(ndev);
			adrv906x_mac_cleanup(&eth_if->adrv906x_dev[i]->mac);
		}
	}

	for (i = 0; i < MAX_NETDEV_NUM; i++) {
		adrv906x_ndma_remove(eth_if->adrv906x_dev[i]->ndma_dev);
		if (es->enabled)
			break;
	}

	return 0;
}

static struct platform_driver adrv906x_eth_drv = {
	.probe			= adrv906x_eth_probe,
	.remove			= adrv906x_eth_remove,
	.driver			= {
		.name		= "adrv906x-net",
		.of_match_table = of_match_ptr(adrv906x_eth_dt_ids),
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(adrv906x_eth_drv);
MODULE_DESCRIPTION("ADRV906X Gigabit Ethernet driver");
MODULE_LICENSE("GPL");
