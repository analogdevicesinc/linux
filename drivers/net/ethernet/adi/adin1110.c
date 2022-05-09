// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* ADIN1110 Low Power 10BASE-T1L Ethernet MAC-PH
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define ADIN1110_PHY_ID				0x1

#define ADIN1110_CONFIG2			0x06
#define   ADIN1110_CRC_APPEND			BIT(5)
#define   ADIN1110_FWD_UNK2HOST			BIT(2)

#define ADIN1110_STATUS0			0x08

#define ADIN1110_STATUS1			0x09
#define   ADIN1110_SPI_ERR			BIT(10)
#define   ADIN1110_RX_RDY			BIT(4)
#define   ADIN1110_TX_RDY			BIT(3)

#define ADIN1110_IMASK1				0x0D
#define   ADIN1110_SPI_ERR_IRQ			BIT(10)
#define   ADIN1110_RX_RDY_IRQ			BIT(4)
#define   ADIN1110_TX_RDY_IRQ			BIT(3)
#define   ADIN1110_LINK_CHANGE_IRQ		BIT(1)

#define ADIN1110_MDIOACC			0x20
#define   ADIN1110_MDIO_TRDONE			BIT(31)
#define   ADIN1110_MDIO_TAERR			BIT(30)
#define   ADIN1110_MDIO_ST			GENMASK(29, 28)
#define   ADIN1110_MDIO_OP			GENMASK(27, 26)
#define   ADIN1110_MDIO_PRTAD			GENMASK(25, 21)
#define   ADIN1110_MDIO_DEVAD			GENMASK(20, 16)
#define   ADIN1110_MDIO_DATA			GENMASK(15, 0)

#define ADIN1110_TX_FSIZE			0x30
#define ADIN1110_TX				0x31
#define ADIN1110_TX_SPACE			0x32
#define ADIN1110_RX_THRESH			0x33
#define ADIN1110_TX_THRESH			0x34

#define ADIN1110_FIFO_CLR			0x36
#define ADIN1110_FIFO_SIZE			0x3E
#define ADIN1110_TFC				0x3F

#define ADIN1110_MAC_ADDR_FILTER_UPR		0x50
#define   ADIN1110_MAC_ADDR_APPLY2PORT		BIT(30)
#define   ADIN1110_MAC_ADDR_HOST_PRI		BIT(19)
#define   ADIN1110_MAC_ADDR_TO_HOST		BIT(16)
#define   ADIN1110_MAC_ADDR			GENMASK(15, 0)

#define ADIN1110_MAC_ADDR_FILTER_LWR		0x51

#define ADIN1110_MAC_ADDR_MASK_UPR		0x70
#define ADIN1110_MAC_ADDR_MASK_LWR		0x71

#define ADIN1110_RX_FSIZE			0x90
#define ADIN1110_RX				0x91

#define ADIN1110_RX_FRM_CNT			0xA0
#define ADIN1110_RX_MCAST_CNT			0xA2
#define ADIN1110_RX_CRC_ERR_CNT			0xA4
#define ADIN1110_RX_ALGN_ERR_CNT		0xA5
#define ADIN1110_RX_LS_ERR_CNT			0xA6
#define ADIN1110_RX_PHY_ERR_CNT			0xA7
#define ADIN1110_RX_DROP_FULL_CNT		0xAC

#define ADIN1110_TX_FRM_CNT			0xA8
#define ADIN1110_TX_MCAST_CNT			0xAA

#define ADIN1110_CLEAR_STATUS0			0x1F7F
#define ADIN1110_CLEAR_STATUS1			0x1F01D0A

/* MDIO_OP codes */
#define ADIN1110_MDIO_OP_WR			0x1
#define ADIN1110_MDIO_OP_RD			0x3

#define ADIN1110_CD				BIT(7)
#define ADIN1110_WRITE				BIT(5)

#define ADIN1110_MAX_BUFF			2048
#define ADIN1110_WR_HEADER_LEN			2
#define ADIN1110_FRAME_HEADER_LEN		2
#define ADIN1110_INTERNAL_SIZE_HEADER_LEN	2
#define ADIN1110_RD_HEADER_LEN			3
#define ADIN1110_REG_LEN			4
#define ADIN1110_FEC_LEN			4

#define ADIN1110_PHY_ID_VAL			0x0283BC91

#define ADIN1110_TX_SPACE_MAX			0x0FFF

DECLARE_CRC8_TABLE(adin1110_crc_table);

struct adin1110_priv {
	struct mutex		lock; /* protect spi */
	spinlock_t		state_lock; /* protect RX mode */
	struct work_struct	tx_work;
	u32			tx_space;
	u64			rx_bytes;
	u64			tx_bytes;
	struct work_struct	rx_mode_work;
	u32			flags;
	struct sk_buff_head	txq;
	struct mii_bus		*mii_bus;
	struct phy_device	*phydev;
	struct net_device	*netdev;
	struct spi_device	*spidev;
	bool			append_crc;
	int			irq;
	u8			data[ADIN1110_MAX_BUFF] ____cacheline_aligned;
};

static u8 adin1110_crc_data(u8 *data, u32 len)
{
	return crc8(adin1110_crc_table, data, len, 0);
}

static int adin1110_read_reg(struct adin1110_priv *priv, u16 reg, u32 *val)
{
	struct spi_transfer t[2] = {0};
	__le16 __reg = cpu_to_le16(reg);
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	u32 read_len = ADIN1110_REG_LEN;
	int ret;

	priv->data[0] = ADIN1110_CD | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	priv->data[2] = 0x00;

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		priv->data[3] = 0x00;
		header_len++;
	}

	t[0].tx_buf = &priv->data[0];
	t[0].len = header_len;

	if (priv->append_crc)
		read_len++;

	memset(&priv->data[header_len], 0, read_len);
	t[1].rx_buf = &priv->data[header_len];
	t[1].len = read_len;

	ret = spi_sync_transfer(priv->spidev, t, 2);
	if (ret)
		return ret;

	if (priv->append_crc) {
		u8 recv_crc;
		u8 crc;

		crc = adin1110_crc_data(&priv->data[header_len], ADIN1110_REG_LEN);
		recv_crc = priv->data[header_len + ADIN1110_REG_LEN];

		if (crc != recv_crc) {
			netdev_err(priv->netdev, "CRC error.");
			return -EBADMSG;
		}
	}

	*val = get_unaligned_be32(&priv->data[header_len]);

	return ret;
}

static int adin1110_write_reg(struct adin1110_priv *priv, u16 reg, u32 val)
{
	u32 header_len = ADIN1110_WR_HEADER_LEN;
	u32 write_len = ADIN1110_REG_LEN;
	__le16 __reg = cpu_to_le16(reg);

	priv->data[0] = ADIN1110_CD | ADIN1110_WRITE | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], header_len);
		header_len++;
	}

	put_unaligned_be32(val, &priv->data[header_len]);
	if (priv->append_crc) {
		priv->data[header_len + write_len] = adin1110_crc_data(&priv->data[header_len],
								       write_len);
		write_len++;
	}

	return spi_write(priv->spidev, &priv->data[0], header_len + write_len);
}

static int adin1110_set_bits(struct adin1110_priv *priv, u16 reg, unsigned long mask,
			     unsigned long val)
{
	u32 write_val;
	int ret;

	ret = adin1110_read_reg(priv, reg, &write_val);
	if (ret < 0)
		return ret;

	set_mask_bits(&write_val, mask, val);

	return adin1110_write_reg(priv, reg, write_val);
}

static int adin1110_round_len(int len)
{
	/* can read/write only mutiples of 4 bytes of payload */
	len = ALIGN(len, 4);

	/* NOTE: ADIN1110_WR_HEADER_LEN should be used for write ops. */
	if (len + ADIN1110_RD_HEADER_LEN > ADIN1110_MAX_BUFF)
		return -EINVAL;

	return len;
}

static int adin1110_read_fifo(struct adin1110_priv *priv)
{
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	__le16 __reg = cpu_to_le16(ADIN1110_RX);
	struct spi_transfer t[2] = {0};
	struct sk_buff *rxb;
	u32 frame_size;
	u32 frame_size_no_fcs;
	int round_len;
	int ret;

	ret = adin1110_read_reg(priv, ADIN1110_RX_FSIZE, &frame_size);
	if (ret < 0)
		return ret;

	/* the read frame size includes the extra 2 bytes from the  ADIN1110 frame header */
	if (frame_size < ADIN1110_FRAME_HEADER_LEN + ADIN1110_FEC_LEN)
		return ret;

	round_len = adin1110_round_len(frame_size);
	if (round_len < 0)
		return ret;

	frame_size_no_fcs = frame_size - ADIN1110_FRAME_HEADER_LEN - ADIN1110_FEC_LEN;

	rxb = netdev_alloc_skb(priv->netdev, frame_size_no_fcs);
	if (!rxb)
		return -ENOMEM;

	memset(priv->data, 0, round_len + ADIN1110_RD_HEADER_LEN);

	priv->data[0] = ADIN1110_CD | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	priv->data[2] = 0x00;

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		priv->data[3] = 0x00;
		header_len++;
	}

	t[0].tx_buf = &priv->data[0];
	t[0].len = header_len;

	t[1].rx_buf = &priv->data[header_len];
	t[1].len = round_len;

	ret = spi_sync_transfer(priv->spidev, t, 2);
	if (ret) {
		kfree_skb(rxb);
		return ret;
	}

	skb_put(rxb, frame_size_no_fcs);
	skb_copy_to_linear_data(rxb, &priv->data[header_len + ADIN1110_FRAME_HEADER_LEN],
				frame_size_no_fcs);

	rxb->protocol = eth_type_trans(rxb, priv->netdev);

	netif_rx_ni(rxb);

	priv->rx_bytes += frame_size - ADIN1110_FRAME_HEADER_LEN;

	return 0;
}

static int adin1110_write_fifo(struct adin1110_priv *priv, struct sk_buff *txb)
{
	__le16 __reg = cpu_to_le16(ADIN1110_TX);
	u32 header_len = ADIN1110_WR_HEADER_LEN;
	int padding = 0;
	int round_len;
	int padded_len;
	int ret;

	/* Pad frame to 64 byte length,
	 * MAC nor PHY will otherwise add the
	 * required padding.
	 * The FEC will be added by the MAC internally.
	 */
	if (txb->len + ADIN1110_FEC_LEN < 64)
		padding = 64 - (txb->len + ADIN1110_FEC_LEN);

	padded_len = txb->len + padding + ADIN1110_FRAME_HEADER_LEN;

	round_len = adin1110_round_len(padded_len);
	if (round_len < 0)
		return round_len;

	ret = adin1110_write_reg(priv, ADIN1110_TX_FSIZE, padded_len);
	if (ret < 0)
		return ret;

	memset(priv->data, 0, round_len + ADIN1110_WR_HEADER_LEN);

	priv->data[0] = ADIN1110_CD | ADIN1110_WRITE | FIELD_GET(GENMASK(12, 8), __reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), __reg);
	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		header_len++;
	}

	memcpy(&priv->data[header_len + ADIN1110_FRAME_HEADER_LEN], txb->data, txb->len);

	ret = spi_write(priv->spidev, &priv->data[0], round_len + header_len);
	if (ret < 0)
		return ret;

	priv->tx_bytes += txb->len;

	return 0;
}

static int adin1110_read_mdio_acc(struct adin1110_priv *priv)
{
	u32 val;
	int ret;

	ret = adin1110_read_reg(priv, ADIN1110_MDIOACC, &val);
	if (ret < 0)
		return 0;

	return val;
}

static int adin1110_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct adin1110_priv *priv = bus->priv;
	u32 val = 0;
	int ret;

	mutex_lock(&priv->lock);

	val |= FIELD_PREP(ADIN1110_MDIO_OP, ADIN1110_MDIO_OP_RD);
	val |= FIELD_PREP(ADIN1110_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_DEVAD, reg);

	/* write the clause 22 read command to the chip */
	ret = adin1110_write_reg(priv, ADIN1110_MDIOACC, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	/* ADIN1110_MDIO_TRDONE BIT of the ADIN1110_MDIOACC
	 * register is set when the read is done.
	 * After the transaction is done, ADIN1110_MDIO_DATA
	 * bitfield of ADIN1110_MDIOACC register will contain
	 * the requested register value.
	 */
	ret = readx_poll_timeout(adin1110_read_mdio_acc, priv, val, (val & ADIN1110_MDIO_TRDONE),
				 10000, 30000);
	mutex_unlock(&priv->lock);

	if (ret < 0)
		return ret;

	return (val & ADIN1110_MDIO_DATA);
}

static int adin1110_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 reg_val)
{
	struct adin1110_priv *priv = bus->priv;
	u32 val = 0;
	int ret;

	mutex_lock(&priv->lock);

	val |= FIELD_PREP(ADIN1110_MDIO_OP, ADIN1110_MDIO_OP_WR);
	val |= FIELD_PREP(ADIN1110_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_DEVAD, reg);
	val |= FIELD_PREP(ADIN1110_MDIO_DATA, reg_val);

	/* write the clause 22 write command to the chip */
	ret = adin1110_write_reg(priv, ADIN1110_MDIOACC, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = readx_poll_timeout(adin1110_read_mdio_acc, priv, val, (val & ADIN1110_MDIO_TRDONE),
				 10000, 30000);
	mutex_unlock(&priv->lock);

	return ret;
}

/* ADIN1110 MAC-PHY contains an ADIN1100 PHY.
 * By registering a new MDIO bus we allow the PAL to discover
 * the encapsulated PHY and probe the ADIN1100 driver.
 */
static int adin1110_register_mdiobus(struct adin1110_priv *priv, struct device *dev)
{
	struct mii_bus *mii_bus;
	int ret;

	mii_bus = devm_mdiobus_alloc(dev);
	if (!mii_bus)
		return -ENOMEM;

	mii_bus->name = "adin1110_eth_mii";
	mii_bus->read = adin1110_mdio_read;
	mii_bus->write = adin1110_mdio_write;
	mii_bus->priv = priv;
	mii_bus->parent = dev;
	mii_bus->phy_mask = ~((u32)BIT(0));
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	ret = devm_mdiobus_register(dev, mii_bus);
	if (ret)
		return ret;

	priv->mii_bus = mii_bus;

	return 0;
}

static void adin1110_read_frames(struct adin1110_priv *priv)
{
	u32 status1;
	int ret;

	while (1) {
		ret = adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);
		if (ret < 0)
			return;

		if (!(status1 & ADIN1110_RX_RDY))
			break;

		ret = adin1110_read_fifo(priv);
		if (ret < 0)
			return;
	}
}

static irqreturn_t adin1110_irq(int irq, void *p)
{
	struct adin1110_priv *priv = p;
	u32 status1;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);

	if (priv->append_crc && (status1 & ADIN1110_SPI_ERR))
		netdev_warn(priv->netdev, "SPI CRC error on write.\n");

	ret = adin1110_read_reg(priv, ADIN1110_TX_SPACE, &val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return IRQ_HANDLED;
	}

	/* TX FIFO space is expressed in half-words */
	priv->tx_space = 2 * val;

	if (status1 & ADIN1110_RX_RDY)
		adin1110_read_frames(priv);

	/* clear IRQ sources */
	adin1110_write_reg(priv, ADIN1110_STATUS0, ADIN1110_CLEAR_STATUS0);
	adin1110_write_reg(priv, ADIN1110_STATUS1, ADIN1110_CLEAR_STATUS1);

	mutex_unlock(&priv->lock);

	if (priv->tx_space > 0)
		netif_wake_queue(priv->netdev);

	return IRQ_HANDLED;
}

/* ADIN1110 can filter up to 16 MAC addresses, mac_nr here is the slot used */
static int adin1110_write_mac_address(struct adin1110_priv *priv, int mac_nr, u8 *addr, u8 *mask)
{
	u32 offset = mac_nr * 2;
	int ret;
	u32 val;

	/* tell MAC to forward this DA to host */
	val = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN1110_MAC_ADDR_TO_HOST;
	val |= get_unaligned_be16(&addr[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&addr[2]);
	ret =  adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + offset, val);
	if (ret < 0)
		return ret;

	val = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN1110_MAC_ADDR_TO_HOST;
	val |= get_unaligned_be16(&mask[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&mask[2]);
	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_LWR + offset, val);
}

static int adin1110_clear_mac_address(struct adin1110_priv *priv, int mac_nr)
{
	u32 offset = mac_nr * 2;
	int ret;

	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + offset, 0);
	if (ret < 0)
		return ret;

	ret =  adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + offset, 0);
	if (ret < 0)
		return ret;

	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_UPR + offset, 0);
	if (ret < 0)
		return ret;

	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_LWR + offset, 0);
}

static int adin1110_multicast_filter(struct adin1110_priv *priv, int mac_nr, bool accept_multicast)
{
	u8 mask[ETH_ALEN] = {0};
	u8 mac[ETH_ALEN] = {0};

	if (accept_multicast) {
		mask[0] = BIT(1);
		mac[0] = BIT(1);

		return adin1110_write_mac_address(priv, mac_nr, mac, mask);
	}

	return adin1110_clear_mac_address(priv, mac_nr);
}

static int adin1110_set_mac_address(struct net_device *netdev, void *addr)
{
	struct adin1110_priv *priv = netdev_priv(netdev);
	struct sockaddr *sa = addr;
	u8 mask[ETH_ALEN];

	if (netif_running(netdev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	ether_addr_copy(netdev->dev_addr, sa->sa_data);
	memset(mask, 0xFF, ETH_ALEN);

	return adin1110_write_mac_address(priv, 0, netdev->dev_addr, mask);
}

static int adin1110_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	if (!netif_running(netdev))
		return -EINVAL;

	if (!netdev->phydev)
		return -ENODEV;

	return phy_mii_ioctl(netdev->phydev, rq, cmd);
}

static void adin1110_rx_mode_work(struct work_struct *work)
{
	struct adin1110_priv *priv = container_of(work, struct adin1110_priv, rx_mode_work);

	mutex_lock(&priv->lock);

	adin1110_set_bits(priv, ADIN1110_CONFIG2, ADIN1110_FWD_UNK2HOST,
			  (priv->flags & IFF_PROMISC) ? ADIN1110_FWD_UNK2HOST : 0);

	adin1110_multicast_filter(priv, 2, !!(priv->flags & IFF_ALLMULTI));

	mutex_unlock(&priv->lock);
}

static void adin1110_set_rx_mode(struct net_device *dev)
{
	struct adin1110_priv *priv = netdev_priv(dev);

	spin_lock(&priv->state_lock);

	priv->flags = dev->flags;
	schedule_work(&priv->rx_mode_work);

	spin_unlock(&priv->state_lock);
}

static int adin1110_init_mac(struct adin1110_priv *priv)
{
	struct net_device *netdev = priv->netdev;
	u8 mask[ETH_ALEN];
	u8 mac[ETH_ALEN];
	int ret;

	memset(mask, 0xFF, ETH_ALEN);
	ret = adin1110_write_mac_address(priv, 0, netdev->dev_addr, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set MAC address: %pM, %d\n", mac, ret);
		return ret;
	}

	memset(mac, 0xFF, ETH_ALEN);
	ret = adin1110_write_mac_address(priv, 1, mac, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set Broadcast MAC address: %d\n", ret);
		return ret;
	}

	return 0;
}

static int adin1110_net_open(struct net_device *net_dev)
{
	struct adin1110_priv *priv = netdev_priv(net_dev);
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	val = ADIN1110_CRC_APPEND;
	ret = adin1110_set_bits(priv, ADIN1110_CONFIG2, val, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	val = ADIN1110_TX_RDY_IRQ | ADIN1110_RX_RDY_IRQ | ADIN1110_LINK_CHANGE_IRQ |
	      ADIN1110_SPI_ERR_IRQ;
	ret = adin1110_set_bits(priv, ADIN1110_IMASK1, val, 0);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to enable chip IRQs: %d\n", ret);
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = adin1110_read_reg(priv, ADIN1110_TX_SPACE, &val);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to read TX FIFO space: %d\n", ret);
		mutex_unlock(&priv->lock);
		return ret;
	}

	priv->tx_space = 2 * val;

	ret = adin1110_init_mac(priv);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	mutex_unlock(&priv->lock);

	phy_start(priv->phydev);

	/* ADIN1110 INT_N pin will be used to signal the host */
	ret = request_threaded_irq(net_dev->irq, NULL, adin1110_irq,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   net_dev->name, priv);

	if (ret < 0) {
		netdev_err(net_dev, "Failed to get IRQ: %d\n", ret);
		return ret;
	}

	netif_start_queue(priv->netdev);

	return ret;
}

static int adin1110_net_stop(struct net_device *net_dev)
{
	struct adin1110_priv *priv = netdev_priv(net_dev);

	netif_stop_queue(priv->netdev);
	flush_work(&priv->tx_work);
	phy_stop(priv->phydev);
	free_irq(net_dev->irq, priv);

	return 0;
}

static void adin1110_tx_work(struct work_struct *work)
{
	struct adin1110_priv *priv = container_of(work, struct adin1110_priv, tx_work);
	struct sk_buff *txb;
	bool last;
	int ret;

	mutex_lock(&priv->lock);

	last = skb_queue_empty(&priv->txq);

	while (!last) {
		txb = skb_dequeue(&priv->txq);
		last = skb_queue_empty(&priv->txq);

		if (txb) {
			ret = adin1110_write_fifo(priv, txb);
			if (ret < 0)
				netdev_err(priv->netdev, "Frame write error: %d\n", ret);

			dev_kfree_skb(txb);
		}
	}

	mutex_unlock(&priv->lock);
}

static netdev_tx_t adin1110_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct adin1110_priv *priv = netdev_priv(dev);
	netdev_tx_t netdev_ret = NETDEV_TX_OK;
	u32 tx_space_needed;

	spin_lock(&priv->state_lock);

	tx_space_needed = skb->len + ADIN1110_FRAME_HEADER_LEN + ADIN1110_INTERNAL_SIZE_HEADER_LEN;
	if (tx_space_needed > priv->tx_space) {
		netif_stop_queue(dev);
		netdev_ret = NETDEV_TX_BUSY;
	} else {
		priv->tx_space -= tx_space_needed;
		skb_queue_tail(&priv->txq, skb);
	}

	spin_unlock(&priv->state_lock);

	schedule_work(&priv->tx_work);

	return netdev_ret;
}

void adin1110_ndo_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
	struct adin1110_priv *priv = netdev_priv(dev);
	u32 val;

	mutex_lock(&priv->lock);

	adin1110_read_reg(priv, ADIN1110_RX_FRM_CNT, &val);
	storage->rx_packets = val;

	adin1110_read_reg(priv, ADIN1110_TX_FRM_CNT, &val);
	storage->tx_packets = val;

	storage->rx_bytes = priv->rx_bytes;
	storage->tx_bytes = priv->tx_bytes;

	adin1110_read_reg(priv, ADIN1110_RX_CRC_ERR_CNT, &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_ALGN_ERR_CNT, &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_LS_ERR_CNT, &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_PHY_ERR_CNT, &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_DROP_FULL_CNT, &val);
	storage->rx_dropped = val;

	adin1110_read_reg(priv, ADIN1110_RX_MCAST_CNT, &val);
	storage->multicast = val;

	mutex_unlock(&priv->lock);
}

static const struct net_device_ops adin1110_netdev_ops = {
	.ndo_open		= adin1110_net_open,
	.ndo_stop		= adin1110_net_stop,
	.ndo_do_ioctl		= adin1110_ioctl,
	.ndo_start_xmit		= adin1110_start_xmit,
	.ndo_set_mac_address	= adin1110_set_mac_address,
	.ndo_set_rx_mode	= adin1110_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= adin1110_ndo_get_stats64,
};

static void adin1110_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *di)
{
	strscpy(di->driver, "ADIN1110", sizeof(di->driver));
	strscpy(di->version, "1.00", sizeof(di->version));
	strscpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static const struct ethtool_ops adin1110_ethtool_ops = {
	.get_drvinfo		= adin1110_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};

static void adin1110_adjust_link(struct net_device *dev)
{
	struct phy_device *phydev = dev->phydev;

	if (!phydev->link)
		phy_print_status(phydev);
}

/* PHY ID is stored in the MAC registers too, check spi connection by reading it */
static int adin1110_check_spi(struct adin1110_priv *priv)
{
	int ret;
	u32 val;

	ret = adin1110_read_reg(priv, ADIN1110_PHY_ID, &val);
	if (ret < 0)
		return ret;

	if (val != ADIN1110_PHY_ID_VAL) {
		netdev_err(priv->netdev, "PHY ID read: %x\n", val);
		return -EIO;
	}

	snprintf(priv->netdev->name, IFNAMSIZ, "adin1110-%u", priv->spidev->chip_select);

	return 0;
}

static void adin1110_disconnect_phy(void *data)
{
	phy_disconnect(data);
}

static int adin1110_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adin1110_priv *priv;
	struct net_device *netdev;
	const u8 *mac_addr;
	u8 mac[ETH_ALEN];
	int ret;

	netdev = devm_alloc_etherdev(dev, sizeof(struct adin1110_priv));
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->spidev = spi;
	priv->netdev = netdev;
	spi->bits_per_word = 8;
	SET_NETDEV_DEV(netdev, dev);

	mutex_init(&priv->lock);
	spin_lock_init(&priv->state_lock);

	INIT_WORK(&priv->tx_work, adin1110_tx_work);
	INIT_WORK(&priv->rx_mode_work, adin1110_rx_mode_work);

	/* use of CRC on control and data transactions is pin dependent */
	priv->append_crc = device_property_read_bool(dev, "adi,spi-crc");
	if (priv->append_crc)
		crc8_populate_msb(adin1110_crc_table, 0x7);

	mac_addr = device_get_mac_address(dev, mac, ETH_ALEN);
	if (!mac_addr) {
		netdev_err(netdev, "MAC address invalid: %pM, %d\n", mac, ret);
		return -EINVAL;
	}

	ether_addr_copy(netdev->dev_addr, mac);

	ret = adin1110_check_spi(priv);
	if (ret < 0) {
		netdev_err(netdev, "SPI read failed: %d\n", ret);
		return ret;
	}

	ret = adin1110_register_mdiobus(priv, dev);
	if (ret < 0) {
		netdev_err(netdev, "Could not register MDIO bus %d\n", ret);
		return ret;
	}

	skb_queue_head_init(&priv->txq);

	netdev->irq = spi->irq;

	netif_carrier_off(priv->netdev);

	/* FIXME: This should be changed to 10BASET1L when introduced to PAL */
	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &adin1110_netdev_ops;
	netdev->ethtool_ops = &adin1110_ethtool_ops;

	ret = devm_register_netdev(dev, netdev);
	if (ret) {
		dev_err(dev, "failed to register network device\n");
		return ret;
	}

	/* there is only one PHY connected to our registered MDIO bus */
	priv->phydev = phy_find_first(priv->mii_bus);
	if (!priv->phydev)
		return -ENODEV;

	priv->phydev = phy_connect(netdev, phydev_name(priv->phydev),
				   adin1110_adjust_link, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(priv->phydev))
		return PTR_ERR(priv->phydev);

	return devm_add_action_or_reset(dev, adin1110_disconnect_phy, priv->phydev);
}

static const struct of_device_id adin1110_match_table[] = {
	{ .compatible = "adi,adin1110" },
	{ }
};
MODULE_DEVICE_TABLE(of, adin1110_match_table);

static struct spi_driver adin1110_driver = {
	.driver = {
		.name = "adin1110",
		.of_match_table = adin1110_match_table,
	},
	.probe = adin1110_probe,
};
module_spi_driver(adin1110_driver);

MODULE_DESCRIPTION("ADIN1110 Network driver");
MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");
