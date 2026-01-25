// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Analog Devices, Inc. ADIN1140 10BASE-T1S MAC-PHY
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/of.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/oa_tc6.h>

struct adin1140_priv {
	struct net_device *netdev;
	struct spi_device *spi;
	struct oa_tc6 *tc6;
	struct phy_device *phydev;
	struct mii_bus *mdiobus;
};

#define MMS_REG(m, r)	((((m) & GENMASK(3, 0)) << 16) | ((r) & GENMASK(15, 0)))

#define OA_TC6_REG_CONFIG0			0x0004
#define OA_TC6_CONFIG0_SYNC			BIT(15)
#define OA_TC6_CONFIG0_TXFCSVE			BIT(14)
#define OA_TC6_CONFIG0_RFA_ZARFE		BIT(12)
#define OA_TC6_CONFIG0_TXCTE			BIT(9)
#define OA_TC6_CONFIG0_RXCTE			BIT(8)
#define OA_TC6_CONFIG0_FTSE			BIT(7)
#define OA_TC6_CONFIG0_FTSS			BIT(6)
#define OA_TC6_CONFIG0_PROTE			BIT(5)

#define OA_TC6_CONFIG0_TXCTHRESH		10U
#define ADIN1140_CONFIG0_TXCTHRESH_CREDIT_8	0x2

#define OA_CONFIG2 				MMS_REG(0x0, 0x006)
#define OA_CONFIG2_FWD_UNK2HOST 		BIT(2)
#define OA_CONFIG2_LO_PRIO_FIFO_CRC_APPEND	BIT(17)
#define OA_CONFIG2_TX_RDY_ON_EMPTY		BIT(8)
#define OA_CONFIG2_FWD_UNK2HOST			BIT(2)
#define OA_CONFIG2_CPS_64			GENMASK(2, 1)

#define ADIN1140_MAC_P1_LOOP_ADDR		MMS_REG(0x1, 0x00C4)

#define ADIN1140_MAC_ADDR_FILT_UPR		MMS_REG(0x1, 0x50)
#define ADIN1140_MAC_ADDR_FILT_APPLY2PORT1	BIT(30)
#define ADIN1140_MAC_ADDR_FILT_TO_HOST		BIT(16)

#define ADIN1140_MAC_ADDR_FILT_LWR		MMS_REG(0x1, 0x51)

#define ADIN1140_MAC_ADDR_MASK_UPR		MMS_REG(0x1, 0x70)
#define ADIN1140_MAC_ADDR_MASK_LWR		MMS_REG(0x1, 0x71)

#define ADIN1140_MAC_FILT_TABLE_MULTICAST_SLOT	2U
#define ADIN1140_MAC_FILT_TABLE_BROADCAST_SLOT	4U
#define ADIN1140_MAC_FILT_TABLE_UNICAST_SLOT	6U
#define ADIN1140_MAC_FILT_TABLE_MAX_SLOT	30U

/**
 *  Write a MAC address to the filter table
 *  to enable reception of frames with that MAC address.
 */
static int adin1140_mac_filter_write(struct adin1140_priv *priv, const u8 *addr,
				   int slot)
{
	int ret;

	if (slot > ADIN1140_MAC_FILT_TABLE_MAX_SLOT) {
		dev_err(&priv->spi->dev, "Failed to add address to filter table - max capacity (16) reached");

		return -ENOSPC;
	}
	/* Write to upper register must precede write to lower register */
	ret = oa_tc6_write_register(priv->tc6, (ADIN1140_MAC_ADDR_FILT_UPR + slot),
			      get_unaligned_be16(&addr[0]) |
				      ADIN1140_MAC_ADDR_FILT_APPLY2PORT1 |
				      ADIN1140_MAC_ADDR_FILT_TO_HOST);
	if (ret < 0) {
		return ret;
	}

	return oa_tc6_write_register(priv->tc6, (ADIN1140_MAC_ADDR_FILT_LWR + slot),
				get_unaligned_be32(&addr[2]));
}

/**
 * Enable reception of unicast frames.
 */
static int adin1140_filter_unicast(struct adin1140_priv *priv)
{
	return adin1140_mac_filter_write(priv, priv->netdev->dev_addr,
			ADIN1140_MAC_FILT_TABLE_UNICAST_SLOT);
}

/**
 * Enable reception of multicast frames.
 */
static int adin1140_filter_multicast(struct adin1140_priv *priv)
{
	int rc;
	u8 multicast_addr[ETH_ALEN] = {1U, 0, 0, 0, 0, 0};
	u8 mask[ETH_ALEN] = {1U, 0, 0, 0, 0, 0};

	rc = adin1140_mac_filter_write(priv, multicast_addr,
			ADIN1140_MAC_FILT_TABLE_MULTICAST_SLOT);
	if (rc < 0) {
		return rc;
	}

	rc = oa_tc6_write_register(priv->tc6,
			(ADIN1140_MAC_ADDR_MASK_UPR + ADIN1140_MAC_FILT_TABLE_MULTICAST_SLOT),
			get_unaligned_be16(&mask[0]));
	if (rc < 0) {
		return rc;
	}

	return oa_tc6_write_register(priv->tc6,
			(ADIN1140_MAC_ADDR_MASK_LWR + ADIN1140_MAC_FILT_TABLE_MULTICAST_SLOT),
			get_unaligned_be32(&mask[2]));
}

/**
 * Set up default MAC address filter table entries.
 */
static int adin1140_default_filter_config(struct adin1140_priv *priv)
{
	int rc;
	u8 broadcast_addr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	/* Enable reception of broadcast frames */
	rc = adin1140_mac_filter_write(priv, broadcast_addr, ADIN1140_MAC_FILT_TABLE_BROADCAST_SLOT);
	if (rc < 0) {
		return rc;
	}

	/* Enable reception of unicast frames */
	rc = adin1140_filter_unicast(priv);
	if (rc < 0) {
		return rc;
	}

	/* Enable reception of multicast frames */
	return adin1140_filter_multicast(priv);
}

static int adin1140_configure(struct adin1140_priv *priv)
{
	u32 val;
	int ret;

	ret = oa_tc6_read_register(priv->tc6, OA_TC6_REG_CONFIG0, &val);
	if (ret < 0) {
		return ret;
	}

	/* Zero-Align Receive Frame Enable */
	val |= OA_TC6_CONFIG0_RFA_ZARFE;

	/* Transmit Frame Check Sequence Validation must be disabled
	 * to allow CRC appending by MAC (CONFIG2.CRC_APPEND)
	 */
	val &= ~OA_TC6_CONFIG0_TXFCSVE;
	val |= OA_CONFIG2_CPS_64;

	ret = oa_tc6_write_register(priv->tc6, OA_TC6_REG_CONFIG0, val);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "Failed to update CONFIG0 register [%d]", ret);
		return ret;
	}

	/* Disable MAC loopback */
	ret = oa_tc6_write_register(priv->tc6, ADIN1140_MAC_P1_LOOP_ADDR, 0x0);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "Failed to disable MAC loopback [%d]", ret);
		return ret;
	}

	/* Configure default MAC address filters */
	ret = adin1140_default_filter_config(priv);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "Failed to set up MAC filter table [%d]", ret);
		return ret;
	}

	return 0;
}

/**
 * Enable promiscuous mode.
 * Frames with an unknown destination MAC address are forwarded to SPI host.
 */
static int adin1140_promiscuous_mode_en(struct adin1140_priv *priv)
{
	int rc;
	u32 val;

	rc = oa_tc6_read_register(priv->tc6, OA_CONFIG2, &val);
	if (rc < 0) {
		return rc;
	}

	val |= OA_CONFIG2_FWD_UNK2HOST;

	return oa_tc6_write_register(priv->tc6, OA_CONFIG2, val);
}

static int adin1140_open(struct net_device *netdev)
{
	phy_start(netdev->phydev);
	netif_carrier_on(netdev);
	netif_start_queue(netdev);

	return 0;
}

static int adin1140_close(struct net_device *netdev)
{
	netif_stop_queue(netdev);
	netif_carrier_off(netdev);
	phy_stop(netdev->phydev);

	return 0;
}

static netdev_tx_t adin1140_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	struct adin1140_priv *priv = netdev_priv(netdev);

	/* Pad frames to minimum Ethernet frame size (60 bytes without FCS).
	 * The MAC will append the FCS, so we need to ensure the frame is
	 * at least ETH_ZLEN bytes to prevent "RX frame undersize" errors
	 * on the receiving side.
	 */
	if (skb_put_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;

	return oa_tc6_start_xmit(priv->tc6, skb);
}

static int adin1140_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	if (!netif_running(netdev))
		return -EINVAL;

	return phy_do_ioctl(netdev, rq, cmd);
}

static int adin1140_set_mac_address(struct net_device *netdev, void *addr)
{
	struct adin1140_priv *priv = netdev_priv(netdev);
	struct sockaddr *address = addr;
	int ret;

	ret = eth_prepare_mac_addr_change(netdev, addr);
	if (ret < 0)
		return ret;

	if (ether_addr_equal(address->sa_data, netdev->dev_addr))
		return 0;

	ret = adin1140_mac_filter_write(priv, address->sa_data,
			ADIN1140_MAC_FILT_TABLE_UNICAST_SLOT);
        if (ret)
                return ret;

	eth_commit_mac_addr_change(netdev, addr);

	return 0;
}

static void adin1140_ndo_get_stats64(struct net_device *dev,
				     struct rtnl_link_stats64 *storage)
{
	struct adin1140_priv *priv = netdev_priv(dev);

	storage->rx_packets = priv->netdev->stats.rx_packets;
	storage->tx_packets = priv->netdev->stats.tx_packets;

	storage->rx_bytes = priv->netdev->stats.rx_bytes;
	storage->tx_bytes = priv->netdev->stats.tx_bytes;
}

static const struct net_device_ops adin1140_netdev_ops = {
	.ndo_open = adin1140_open,
	.ndo_stop = adin1140_close,
	.ndo_start_xmit	= adin1140_start_xmit,
	.ndo_set_mac_address = adin1140_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_eth_ioctl = adin1140_ioctl,
	.ndo_get_stats64 = adin1140_ndo_get_stats64,
};

static const struct ethtool_ops adin1140_ethtool_ops = {
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
};

static int adin1140_get_phy_c45_mms(int devnum)
{
	switch (devnum) {
	case MDIO_MMD_VEND2:
		return 4;
	default:
		return -EOPNOTSUPP;
	}
}

static int adin1140_mdiobus_read(struct mii_bus *bus, int addr, int regnum)
{
	struct oa_tc6 *tc6 = bus->priv;
	u32 discard;
	u32 regval;
	bool ret;

	ret = oa_tc6_read_register(tc6, 0xFF00 | (regnum & GENMASK(3, 0)),
				   &regval);
	if (ret)
		return ret;

	/* After a direct PHY access, we have to do an additional register read,
	 * otherwise, a HDRB error will be generated
	 */
	oa_tc6_read_register(tc6, 0x0, &discard);

	return regval;
}

static int adin1140_mdiobus_write(struct mii_bus *bus, int addr, int regnum,
				u16 val)
{
	struct oa_tc6 *tc6 = bus->priv;
	u32 discard;
	int ret;

	ret = oa_tc6_write_register(tc6, 0xFF00 | (regnum & GENMASK(3, 0)),
				     val);
	if (ret)
		return ret;

	/* After a direct PHY access, we have to do an additional register read,
	 * otherwise, a HDRB error will be generated
	 */
	oa_tc6_read_register(tc6, 0x0, &discard);

	return 0;
}

static int adin1140_mdiobus_read_c45(struct mii_bus *bus, int addr, int devnum,
				     int regnum)
{
	struct oa_tc6 *tc6 = bus->priv;
	u32 regval;
	int ret;

	ret = adin1140_get_phy_c45_mms(devnum);
	if (ret < 0)
		return ret;

	ret = oa_tc6_read_register(tc6, (ret << 16) | regnum, &regval);
	if (ret)
		return ret;

	return regval;
}

static int adin1140_mdiobus_write_c45(struct mii_bus *bus, int addr, int devnum,
				      int regnum, u16 val)
{
	struct oa_tc6 *tc6 = bus->priv;
	int ret;

	ret = adin1140_get_phy_c45_mms(devnum);
	if (ret < 0)
		return ret;

	return oa_tc6_write_register(tc6, (ret << 16) | regnum, val);
}

static int adin1140_mdio_register(struct adin1140_priv *priv)
{
	int ret;

	priv->mdiobus = devm_mdiobus_alloc(&priv->spi->dev);
	if (!priv->mdiobus){
		netdev_err(priv->netdev, "MDIO bus alloc failed\n");

		return -ENOMEM;
	}

	priv->mdiobus->priv = priv->tc6;
	priv->mdiobus->name = "oa-tc6-mdio";
	priv->mdiobus->parent = &priv->spi->dev;
	priv->mdiobus->read = adin1140_mdiobus_read;
	priv->mdiobus->write = adin1140_mdiobus_write;
	priv->mdiobus->read_c45 = adin1140_mdiobus_read_c45;
	priv->mdiobus->write_c45 = adin1140_mdiobus_write_c45;

	snprintf(priv->mdiobus->id, ARRAY_SIZE(priv->mdiobus->id), "%s",
		dev_name(&priv->netdev->dev));

	ret = devm_mdiobus_register(&priv->spi->dev, priv->mdiobus);
	if (ret){
		netdev_err(priv->netdev, "MDIO bus register error\n");
		return ret;
	}

	return 0;
}

static void adin1140_handle_link_change(struct net_device *netdev)
{
	phy_print_status(netdev->phydev);
}

static int adin1140_phy_init(struct adin1140_priv *priv)
{
	int ret;

	ret = adin1140_mdio_register(priv);
	if (ret)
		return ret;

	priv->phydev = phy_find_first(priv->mdiobus);
	if (!priv->phydev){
		netdev_err(priv->netdev, "PHY not found\n");

		return ret;
	}

	priv->phydev->is_internal = true;
	ret = phy_connect_direct(priv->netdev, priv->phydev,
				 &adin1140_handle_link_change,
				 PHY_INTERFACE_MODE_INTERNAL);
	if (ret){
		netdev_err(priv->netdev, "Can't attach PHY to %s\n",
			   priv->mdiobus->id);

		return ret;
	}

	phy_attached_info(priv->netdev->phydev);

	return 0;
}

static void adin1140_phy_unregister(struct adin1140_priv *priv)
{
	phy_disconnect(priv->phydev);
}

static int adin1140_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct adin1140_priv *priv;
	int ret;

	netdev = devm_alloc_etherdev(&spi->dev, sizeof(struct adin1140_priv));
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	priv->tc6 = oa_tc6_init(spi, netdev);
	if (!priv->tc6)
		return -ENODEV;

	/* Get the MAC address from the SPI device tree node */
	if (device_get_ethdev_address(&spi->dev, netdev))
		eth_hw_addr_random(netdev);

	ret = adin1140_phy_init(priv);
	if (ret)
		goto oa_tc6_exit;

	ret = adin1140_configure(priv);
	if (ret < 0) {
		dev_err(&spi->dev, "Could not configure ADIN1140 %d\n", ret);
		goto free_phy;
	}

	netdev->if_port = IF_PORT_10BASET;
	netdev->irq = spi->irq;
	netdev->netdev_ops = &adin1140_netdev_ops;
	netdev->ethtool_ops = &adin1140_ethtool_ops;

	ret = devm_register_netdev(&spi->dev, netdev);
	if (ret) {
		dev_err(&spi->dev, "Register netdev failed (ret = %d)", ret);
		goto free_phy;
	}

        return 0;

free_phy:
	adin1140_phy_unregister(priv);
oa_tc6_exit:
	oa_tc6_exit(priv->tc6);

	return ret;
}

static void adin1140_remove(struct spi_device *spi)
{
	struct adin1140_priv *priv = spi_get_drvdata(spi);

	oa_tc6_exit(priv->tc6);
	adin1140_phy_unregister(priv);
}

static const struct spi_device_id adin1140_spi_id[] = {
	{ .name = "adin1140" },
	{},
};
MODULE_DEVICE_TABLE(spi, adin1140_spi_id);

static const struct of_device_id adin1140_match_table[] = {
	{ .compatible = "adi,adin1140" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, adin1140_match_table);

static struct spi_driver adin1140_driver = {
	.driver = {
		.name = "adin1140",
		.of_match_table = adin1140_match_table,
	 },
	.probe = adin1140_probe,
	.remove = adin1140_remove,
	.id_table = adin1140_spi_id,
};
module_spi_driver(adin1140_driver);

MODULE_DESCRIPTION("Analog Devices, Inc. ADIN1140 10BASE-T1S MAC-PHY");
MODULE_AUTHOR("Ciprian Regus <ciprian.regus@analog.com>");
MODULE_LICENSE("GPL");
