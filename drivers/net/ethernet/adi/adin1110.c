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

#define ADIN1110_CONFIG1			0x04
#define   ADIN1110_CONFIG1_SYNC			BIT(15)

#define ADIN1110_CONFIG2			0x06
#define   ADIN211_FWD_UNK2PORT			GENMASK(14, 13)
#define   ADIN2111_P2_FWD_UNK2HOST		BIT(12)
#define   ADIN1110_CRC_APPEND			BIT(5)
#define   ADIN1110_FWD_UNK2HOST			BIT(2)

#define ADIN1110_STATUS0			0x08

#define ADIN1110_STATUS1			0x09
#define   ADIN2111_P2_RX_RDY			BIT(17)
#define   ADIN1110_SPI_ERR			BIT(10)
#define   ADIN1110_RX_RDY			BIT(4)
#define   ADIN1110_TX_RDY			BIT(3)

#define ADIN1110_IMASK1				0x0D
#define   ADIN2111_RX_RDY_IRQ			BIT(17)
#define   ADIN1110_SPI_ERR_IRQ			BIT(10)
#define   ADIN1110_RX_RDY_IRQ			BIT(4)
#define   ADIN1110_TX_RDY_IRQ			BIT(3)

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
#define   ADIN2111_MAC_ADDR_APPLY2PORT2		BIT(31)
#define   ADIN1110_MAC_ADDR_APPLY2PORT		BIT(30)
#define   ADIN1110_MAC_ADDR_HOST_PRI		BIT(19)
#define   ADIN1110_MAC_ADDR_TO_HOST		BIT(16)
#define   ADIN1110_MAC_ADDR			GENMASK(15, 0)

#define ADIN1110_MAC_ADDR_FILTER_LWR		0x51

#define ADIN1110_MAC_ADDR_MASK_UPR		0x70
#define ADIN1110_MAC_ADDR_MASK_LWR		0x71

#define ADIN1110_RX_FSIZE			0x90
#define ADIN1110_RX				0x91

#define ADIN1110_RX_FRM_CNT(x)			(0xA0 + ((x) * 0x30))
#define ADIN1110_RX_MCAST_CNT(x)		(0xA2 + ((x) * 0x30))
#define ADIN1110_RX_CRC_ERR_CNT(x)		(0xA4 + ((x) * 0x30))
#define ADIN1110_RX_ALGN_ERR_CNT(x)		(0xA5 + ((x) * 0x30))
#define ADIN1110_RX_LS_ERR_CNT(x)		(0xA6 + ((x) * 0x30))
#define ADIN1110_RX_PHY_ERR_CNT(x)		(0xA7 + ((x) * 0x30))
#define ADIN1110_RX_DROP_FULL_CNT(x)		(0xAC + ((x) * 0x30))

#define ADIN1110_TX_FRM_CNT(x)			(0xA8 + ((x) * 0x30))
#define ADIN1110_TX_MCAST_CNT(x)		(0xAA + ((x) * 0x30))

#define ADIN2111_RX_P2_FSIZE			0xC0
#define ADIN2111_RX_P2				0xC1

#define ADIN1110_CLEAR_STATUS0			0xFFF
#define ADIN1110_CLEAR_STATUS1			U32_MAX

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
#define ADIN2111_PHY_ID_VAL			0x0283BCA1

#define ADIN1110_TX_SPACE_MAX			0x0FFF

#define ADIN_MAC_MAX_PORTS			2

#define ADIN_MAC_ADDR_SLOT			0
#define ADIN_MAC_BROADCAST_ADDR_SLOT		1
#define ADIN_MAC_MULTICAST_ADDR_SLOT		2

DECLARE_CRC8_TABLE(adin1110_crc_table);

enum adin1110_chips_id {
	ADIN1110_MAC = 0,
	ADIN2111_MAC,
};

struct adin1110_cfg {
	enum adin1110_chips_id	id;
	char			name[MDIO_NAME_SIZE];
	u32			phy_ids[PHY_MAX_ADDR];
	u32			ports_nr;
	u32			phy_id_val;
};

struct adin1110_port_priv {
	struct adin1110_priv		*priv;
	struct net_device		*netdev;
	struct phy_device		*phydev;
	struct work_struct		tx_work;
	u64				rx_bytes;
	u64				tx_bytes;
	struct work_struct		rx_mode_work;
	u32				flags;
	struct sk_buff_head		txq;
	u32				nr;
	struct adin1110_cfg		*cfg;
};

struct adin1110_priv {
	struct mutex			lock; /* protect spi */
	spinlock_t			state_lock; /* protect RX mode */
	struct mii_bus			*mii_bus;
	struct spi_device		*spidev;
	bool				append_crc;
	struct adin1110_cfg		*cfg;
	u32				tx_space;
	u32				irq_mask;
	int				irq;
	struct adin1110_port_priv	*ports[ADIN_MAC_MAX_PORTS];
	char				mii_bus_name[MII_BUS_ID_SIZE];
	u8				data[ADIN1110_MAX_BUFF] ____cacheline_aligned;
};

struct adin1110_cfg adin1110_cfgs[] = {
	{
		.id = ADIN1110_MAC,
		.name = "adin1110",
		.phy_ids = {1},
		.ports_nr = 1,
		.phy_id_val = ADIN1110_PHY_ID_VAL,
	},
	{
		.id = ADIN2111_MAC,
		.name = "adin2111",
		.phy_ids = {1, 2},
		.ports_nr = 2,
		.phy_id_val = ADIN2111_PHY_ID_VAL,
	},
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
			dev_err(&priv->spidev->dev, "CRC error.");
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

static int adin1110_read_fifo(struct adin1110_port_priv *port_priv)
{
	struct adin1110_priv *priv = port_priv->priv;
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	struct spi_transfer t[2] = {0};
	u32 frame_size_no_fcs;
	struct sk_buff *rxb;
	u32 frame_size;
	int round_len;
	__le16 __reg;
	int ret;

	if (!port_priv->nr) {
		__reg = cpu_to_le16(ADIN1110_RX);
		ret = adin1110_read_reg(priv, ADIN1110_RX_FSIZE, &frame_size);
	} else {
		__reg = cpu_to_le16(ADIN2111_RX_P2);
		ret = adin1110_read_reg(priv, ADIN2111_RX_P2_FSIZE, &frame_size);
	}

	if (ret < 0)
		return ret;

	/* the read frame size includes the extra 2 bytes from the  ADIN1110 frame header */
	if (frame_size < ADIN1110_FRAME_HEADER_LEN + ADIN1110_FEC_LEN)
		return ret;

	round_len = adin1110_round_len(frame_size);
	if (round_len < 0)
		return ret;

	frame_size_no_fcs = frame_size - ADIN1110_FRAME_HEADER_LEN - ADIN1110_FEC_LEN;

	rxb = netdev_alloc_skb(port_priv->netdev, frame_size_no_fcs);
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

	rxb->protocol = eth_type_trans(rxb, port_priv->netdev);

	netif_rx_ni(rxb);

	port_priv->rx_bytes += frame_size - ADIN1110_FRAME_HEADER_LEN;

	return 0;
}

static int adin1110_write_fifo(struct adin1110_port_priv *port_priv, struct sk_buff *txb)
{
	struct adin1110_priv *priv = port_priv->priv;
	__le16 __reg = cpu_to_le16(ADIN1110_TX);
	u32 header_len = ADIN1110_WR_HEADER_LEN;
	__be16 frame_header;
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

	/* mention the port on which to send the frame in the frame header */
	frame_header = cpu_to_be16(port_priv->nr);
	memcpy(&priv->data[header_len], &frame_header, ADIN1110_FRAME_HEADER_LEN);

	memcpy(&priv->data[header_len + ADIN1110_FRAME_HEADER_LEN], txb->data, txb->len);

	ret = spi_write(priv->spidev, &priv->data[0], round_len + header_len);
	if (ret < 0)
		return ret;

	port_priv->tx_bytes += txb->len;

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
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, phy_id);
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
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, phy_id);
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
 * ADIN2111 MAC-PHY contains two ADIN1100 PHYs.
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

	snprintf(priv->mii_bus_name, MII_BUS_ID_SIZE, "%s-%u",
		 priv->cfg->name, priv->spidev->chip_select);

	mii_bus->name = priv->mii_bus_name;
	mii_bus->read = adin1110_mdio_read;
	mii_bus->write = adin1110_mdio_write;
	mii_bus->priv = priv;
	mii_bus->parent = dev;
	mii_bus->phy_mask = ~((u32)GENMASK(2, 0));
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	ret = devm_mdiobus_register(dev, mii_bus);
	if (ret)
		return ret;

	priv->mii_bus = mii_bus;

	return 0;
}

static bool adin1110_port_rx_ready(struct adin1110_port_priv *port_priv, u32 status)
{
	if (!port_priv->nr)
		return !!(status & ADIN1110_RX_RDY);
	else
		return !!(status & ADIN2111_P2_RX_RDY);
}

static void adin1110_read_frames(struct adin1110_port_priv *port_priv)
{
	struct adin1110_priv *priv = port_priv->priv;
	u32 status1;
	int ret;

	while (1) {
		ret = adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);
		if (ret < 0)
			return;

		if (!adin1110_port_rx_ready(port_priv, status1))
			break;

		ret = adin1110_read_fifo(port_priv);
		if (ret < 0)
			return;
	}
}

static void adin1110_wake_queues(struct adin1110_priv *priv)
{
	int i;

	for (i = 0; i < priv->cfg->ports_nr; i++)
		netif_wake_queue(priv->ports[i]->netdev);
}

static irqreturn_t adin1110_irq(int irq, void *p)
{
	struct adin1110_priv *priv = p;
	u32 status1;
	u32 val;
	int ret;
	int i;

	mutex_lock(&priv->lock);

	adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);

	if (priv->append_crc && (status1 & ADIN1110_SPI_ERR))
		dev_warn(&priv->spidev->dev, "SPI CRC error on write.\n");

	ret = adin1110_read_reg(priv, ADIN1110_TX_SPACE, &val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return IRQ_HANDLED;
	}

	/* TX FIFO space is expressed in half-words */
	priv->tx_space = 2 * val;

	for (i = 0; i < priv->cfg->ports_nr; i++) {
		if (adin1110_port_rx_ready(priv->ports[i], status1))
			adin1110_read_frames(priv->ports[i]);
	}

	/* clear IRQ sources */
	ret = adin1110_write_reg(priv, ADIN1110_STATUS0, ADIN1110_CLEAR_STATUS0);
	ret = adin1110_write_reg(priv, ADIN1110_STATUS1, priv->irq_mask);

	mutex_unlock(&priv->lock);

	if (priv->tx_space > 0)
		adin1110_wake_queues(priv);

	return IRQ_HANDLED;
}

/* ADIN1110 can filter up to 16 MAC addresses, mac_nr here is the slot used */
static int adin1110_write_mac_address(struct adin1110_port_priv *port_priv, int mac_nr, u8 *addr,
				      u8 *mask)
{
	struct adin1110_priv *priv = port_priv->priv;
	u32 offset = mac_nr * 2;
	u32 port_rules;
	int ret;
	u32 val;

	port_rules = ADIN1110_MAC_ADDR_APPLY2PORT;
	if (priv->cfg->id == ADIN2111_MAC)
		port_rules |= ADIN2111_MAC_ADDR_APPLY2PORT2;

	/* tell MAC to forward this DA to host */
	val = port_rules | ADIN1110_MAC_ADDR_TO_HOST;
	val |= get_unaligned_be16(&addr[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&addr[2]);
	ret =  adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be16(&mask[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_UPR + offset, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&mask[2]);
	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_LWR + offset, val);
}

static int adin1110_clear_mac_address(struct adin1110_port_priv *port_priv, int mac_nr)
{
	struct adin1110_priv *priv = port_priv->priv;
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

static int adin1110_multicast_filter(struct adin1110_port_priv *port_priv, int mac_nr,
				     bool accept_multicast)
{
	u8 mask[ETH_ALEN] = {0};
	u8 mac[ETH_ALEN] = {0};

	if (accept_multicast) {
		mask[0] = BIT(1);
		mac[0] = BIT(1);

		return adin1110_write_mac_address(port_priv, mac_nr, mac, mask);
	}

	return adin1110_clear_mac_address(port_priv, mac_nr);
}

static int adin1110_set_mac_address(struct net_device *netdev, void *addr)
{
	struct adin1110_port_priv *port_priv = netdev_priv(netdev);
	struct sockaddr *sa = addr;
	u8 mask[ETH_ALEN];

	if (netif_running(netdev))
		return -EBUSY;

	if (!is_valid_ether_addr(sa->sa_data))
		return -EADDRNOTAVAIL;

	ether_addr_copy(netdev->dev_addr, sa->sa_data);
	memset(mask, 0xFF, ETH_ALEN);

	return adin1110_write_mac_address(port_priv, ADIN_MAC_ADDR_SLOT, netdev->dev_addr, mask);
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
	struct adin1110_port_priv *port_priv = container_of(work, struct adin1110_port_priv, rx_mode_work);
	struct adin1110_priv *priv = port_priv->priv;
	u32 mask;

	if (!port_priv->nr)
		mask = ADIN1110_FWD_UNK2HOST;
	else
		mask = ADIN2111_P2_FWD_UNK2HOST;

	mutex_lock(&priv->lock);

	adin1110_set_bits(priv, ADIN1110_CONFIG2, mask,
			  (port_priv->flags & IFF_PROMISC) ? mask : 0);

	adin1110_multicast_filter(port_priv, ADIN_MAC_MULTICAST_ADDR_SLOT,
				  !!(port_priv->flags & IFF_ALLMULTI));

	mutex_unlock(&priv->lock);
}

static void adin1110_set_rx_mode(struct net_device *dev)
{
	struct adin1110_port_priv *port_priv = netdev_priv(dev);
	struct adin1110_priv *priv = port_priv->priv;

	spin_lock(&priv->state_lock);

	port_priv->flags = dev->flags;
	schedule_work(&port_priv->rx_mode_work);

	spin_unlock(&priv->state_lock);
}

static int adin1110_init_mac(struct adin1110_port_priv *port_priv)
{
	struct net_device *netdev = port_priv->netdev;
	u8 mask[ETH_ALEN];
	u8 mac[ETH_ALEN];
	int ret;

	memset(mask, 0xFF, ETH_ALEN);
	ret = adin1110_write_mac_address(port_priv, ADIN_MAC_ADDR_SLOT, netdev->dev_addr, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set MAC address: %pM, %d\n", mac, ret);
		return ret;
	}

	memset(mac, 0xFF, ETH_ALEN);
	ret = adin1110_write_mac_address(port_priv, ADIN_MAC_BROADCAST_ADDR_SLOT, mac, mask);
	if (ret < 0) {
		netdev_err(netdev, "Could not set Broadcast MAC address: %d\n", ret);
		return ret;
	}

	return 0;
}

static int adin1110_net_open(struct net_device *net_dev)
{
	struct adin1110_port_priv *port_priv = netdev_priv(net_dev);
	struct adin1110_priv *priv = port_priv->priv;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	/* Configure MAC to compute and append the FCS itself.
	 * If ADIN2111 configure MAC to forward unknown host to other port.
	 */
	val = ADIN1110_CRC_APPEND;
	if (priv->cfg->id == ADIN2111_MAC)
		val |= ADIN211_FWD_UNK2PORT;

	ret = adin1110_set_bits(priv, ADIN1110_CONFIG2, val, val);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	val = ADIN1110_TX_RDY_IRQ | ADIN1110_RX_RDY_IRQ | ADIN1110_SPI_ERR_IRQ;
	if (priv->cfg->id == ADIN2111_MAC)
		val |= ADIN2111_RX_RDY_IRQ;

	priv->irq_mask = val;
	ret = adin1110_write_reg(priv, ADIN1110_IMASK1, ~val);
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

	ret = adin1110_init_mac(port_priv);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = adin1110_set_bits(priv, ADIN1110_CONFIG1, ADIN1110_CONFIG1_SYNC,
				ADIN1110_CONFIG1_SYNC);
	if (ret < 0) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	mutex_unlock(&priv->lock);

	phy_start(port_priv->phydev);

	netif_start_queue(net_dev);

	return ret;
}

static int adin1110_net_stop(struct net_device *net_dev)
{
	struct adin1110_port_priv *port_priv = netdev_priv(net_dev);

	netif_stop_queue(port_priv->netdev);
	flush_work(&port_priv->tx_work);
	phy_stop(port_priv->phydev);

	return 0;
}

static void adin1110_tx_work(struct work_struct *work)
{
	struct adin1110_port_priv *port_priv = container_of(work, struct adin1110_port_priv, tx_work);
	struct adin1110_priv *priv = port_priv->priv;
	struct sk_buff *txb;
	bool last;
	int ret;

	mutex_lock(&priv->lock);

	last = skb_queue_empty(&port_priv->txq);

	while (!last) {
		txb = skb_dequeue(&port_priv->txq);
		last = skb_queue_empty(&port_priv->txq);

		if (txb) {
			ret = adin1110_write_fifo(port_priv, txb);
			if (ret < 0)
				netdev_err(port_priv->netdev, "Frame write error: %d\n", ret);

			dev_kfree_skb(txb);
		}
	}

	mutex_unlock(&priv->lock);
}

static netdev_tx_t adin1110_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct adin1110_port_priv *port_priv = netdev_priv(dev);
	struct adin1110_priv *priv = port_priv->priv;
	netdev_tx_t netdev_ret = NETDEV_TX_OK;
	u32 tx_space_needed;

	spin_lock(&priv->state_lock);

	tx_space_needed = skb->len + ADIN1110_FRAME_HEADER_LEN + ADIN1110_INTERNAL_SIZE_HEADER_LEN;
	if (tx_space_needed > priv->tx_space) {
		netif_stop_queue(dev);
		netdev_ret = NETDEV_TX_BUSY;
	} else {
		priv->tx_space -= tx_space_needed;
		skb_queue_tail(&port_priv->txq, skb);
	}

	spin_unlock(&priv->state_lock);

	schedule_work(&port_priv->tx_work);

	return netdev_ret;
}

void adin1110_ndo_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
	struct adin1110_port_priv *port_priv = netdev_priv(dev);
	struct adin1110_priv *priv = port_priv->priv;
	u32 val;

	mutex_lock(&priv->lock);

	adin1110_read_reg(priv, ADIN1110_RX_FRM_CNT(port_priv->nr), &val);
	storage->rx_packets = val;

	adin1110_read_reg(priv, ADIN1110_TX_FRM_CNT(port_priv->nr), &val);
	storage->tx_packets = val;

	storage->rx_bytes = port_priv->rx_bytes;
	storage->tx_bytes = port_priv->tx_bytes;

	adin1110_read_reg(priv, ADIN1110_RX_CRC_ERR_CNT(port_priv->nr), &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_ALGN_ERR_CNT(port_priv->nr), &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_LS_ERR_CNT(port_priv->nr), &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_PHY_ERR_CNT(port_priv->nr), &val);
	storage->rx_errors += val;

	adin1110_read_reg(priv, ADIN1110_RX_DROP_FULL_CNT(port_priv->nr), &val);
	storage->rx_dropped = val;

	adin1110_read_reg(priv, ADIN1110_RX_MCAST_CNT(port_priv->nr), &val);
	storage->multicast = val;

	mutex_unlock(&priv->lock);
}

static int adin1110_port_get_port_parent_id(struct net_device *dev,
					    struct netdev_phys_item_id *ppid)
{
	struct adin1110_port_priv *port_priv = netdev_priv(dev);
	struct adin1110_priv *priv = port_priv->priv;

	ppid->id_len = strnlen(priv->mii_bus_name, MII_BUS_ID_SIZE);
	memcpy(ppid->id, priv->mii_bus_name, ppid->id_len);

	return 0;
}

static int adin1110_ndo_get_phys_port_name(struct net_device *dev, char *name, size_t len)
{
	struct adin1110_port_priv *port_priv = netdev_priv(dev);
	int err;

	err = snprintf(name, len, "p%d", port_priv->nr);
	if (err >= len)
		return -EINVAL;

	return 0;
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
	.ndo_get_port_parent_id	= adin1110_port_get_port_parent_id,
	.ndo_get_phys_port_name	= adin1110_ndo_get_phys_port_name,
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

	if (val != priv->cfg->phy_id_val) {
		dev_err(&priv->spidev->dev, "PHY ID read: %x\n", val);
		return -EIO;
	}

	return 0;
}

static void adin1110_disconnect_phy(void *data)
{
	phy_disconnect(data);
}

static int adin1110_probe_netdevs(struct adin1110_priv *priv)
{
	struct device *dev = &priv->spidev->dev;
	struct adin1110_port_priv *port_priv;
	struct net_device *netdev;
	const u8 *mac_addr;
	u8 mac[ETH_ALEN];
	int ret;
	int i;

	for (i = 0; i < priv->cfg->ports_nr; i++) {
		netdev = devm_alloc_etherdev(dev, sizeof(*port_priv));
		if (!netdev)
			return -ENOMEM;

		port_priv = netdev_priv(netdev);
		port_priv->netdev = netdev;
		port_priv->priv = priv;
		port_priv->cfg = priv->cfg;
		port_priv->nr = i;
		priv->ports[i] = port_priv;
		SET_NETDEV_DEV(netdev, dev);

		mac_addr = device_get_mac_address(dev, mac, ETH_ALEN);
		if (!mac_addr) {
			netdev_err(netdev, "MAC address invalid: %pM\n", mac);
			return -EINVAL;
		}

		ether_addr_copy(netdev->dev_addr, mac);

		netdev->irq = priv->spidev->irq;
		INIT_WORK(&port_priv->tx_work, adin1110_tx_work);
		INIT_WORK(&port_priv->rx_mode_work, adin1110_rx_mode_work);
		skb_queue_head_init(&port_priv->txq);

		netif_carrier_off(netdev);

		/* FIXME: This should be changed to 10BASET1L when introduced to PAL */
		netdev->if_port = IF_PORT_10BASET;
		netdev->netdev_ops = &adin1110_netdev_ops;
		netdev->ethtool_ops = &adin1110_ethtool_ops;

		switch (priv->cfg->id) {
		case ADIN1110_MAC:
			snprintf(port_priv->netdev->name, IFNAMSIZ, "%s-%u", priv->cfg->name,
				 priv->spidev->chip_select);
			break;
		case ADIN2111_MAC:
			snprintf(port_priv->netdev->name, IFNAMSIZ, "%s-%u-p%d", priv->cfg->name,
				 priv->spidev->chip_select, i);
			break;
		default:
			return -EINVAL;
		}

		ret = devm_register_netdev(dev, netdev);
		if (ret < 0) {
			dev_err(dev, "failed to register network device\n");
			return ret;
		}

		port_priv->phydev = get_phy_device(priv->mii_bus, i + 1, false);
		if (!port_priv->phydev) {
			netdev_err(netdev, "Could not find PHY with device address: %d.\n", i);
			return -ENODEV;
		}

		port_priv->phydev = phy_connect(netdev, phydev_name(port_priv->phydev),
						adin1110_adjust_link, PHY_INTERFACE_MODE_MII);
		if (IS_ERR(port_priv->phydev)) {
			netdev_err(netdev, "Could not connect PHY with device address: %d.\n", i);
			return PTR_ERR(port_priv->phydev);
		}

		ret = devm_add_action_or_reset(dev, adin1110_disconnect_phy, port_priv->phydev);
		if (ret < 0)
			return ret;
	}

	/* ADIN1110 INT_N pin will be used to signal the host */
	return devm_request_threaded_irq(dev, priv->spidev->irq, NULL, adin1110_irq,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(dev), priv);
}

static int adin1110_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct device *dev = &spi->dev;
	struct adin1110_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct adin1110_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spidev = spi;
	priv->cfg = &adin1110_cfgs[dev_id->driver_data];
	spi->bits_per_word = 8;

	mutex_init(&priv->lock);
	spin_lock_init(&priv->state_lock);

	/* use of CRC on control and data transactions is pin dependent */
	priv->append_crc = device_property_read_bool(dev, "adi,spi-crc");
	if (priv->append_crc)
		crc8_populate_msb(adin1110_crc_table, 0x7);

	ret = adin1110_check_spi(priv);
	if (ret < 0) {
		dev_err(dev, "SPI read failed: %d\n", ret);
		return ret;
	}

	ret = adin1110_register_mdiobus(priv, dev);
	if (ret < 0) {
		dev_err(dev, "Could not register MDIO bus %d\n", ret);
		return ret;
	}

	return adin1110_probe_netdevs(priv);
}

static const struct of_device_id adin1110_match_table[] = {
	{ .compatible = "adi,adin1110" },
	{ .compatible = "adi,adin2111" },
	{ }
};
MODULE_DEVICE_TABLE(of, adin1110_match_table);

static const struct spi_device_id adin1110_spi_id[] = {
	{ .name = "adin1110", .driver_data = ADIN1110_MAC },
	{ .name = "adin2111", .driver_data = ADIN2111_MAC },
	{ }
};

static struct spi_driver adin1110_driver = {
	.driver = {
		.name = "adin1110",
		.of_match_table = adin1110_match_table,
	},
	.probe = adin1110_probe,
	.id_table = adin1110_spi_id,
};
module_spi_driver(adin1110_driver);

MODULE_DESCRIPTION("ADIN1110 Network driver");
MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");
