// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/bitfield.h>
#include <net/rtnetlink.h>
#include "adrv906x-mac.h"

void adrv906x_mac_promiscuous_mode_en(struct adrv906x_mac *mac)
{
	void __iomem *emac_rx = mac->emac_rx;
	unsigned int val;

	val = ioread32(emac_rx + MAC_RX_CTRL);
	val |= MAC_RX_PROMISCUOUS_MODE_EN;
	iowrite32(val, emac_rx + MAC_RX_CTRL);
}

void adrv906x_mac_promiscuous_mode_dis(struct adrv906x_mac *mac)
{
	void __iomem *emac_rx = mac->emac_rx;
	unsigned int val;

	val = ioread32(emac_rx + MAC_RX_CTRL);
	val &= ~MAC_RX_PROMISCUOUS_MODE_EN;
	iowrite32(val, emac_rx + MAC_RX_CTRL);
}

static void adrv906x_mac_tx_path_en(struct adrv906x_mac *mac)
{
	void __iomem *emac_tx = mac->emac_tx;
	unsigned int val;

	val = ioread32(emac_tx + MAC_TX_CTRL);
	val |= MAC_TX_PATH_EN;
	iowrite32(val, emac_tx + MAC_TX_CTRL);
}

static void adrv906x_mac_tx_path_dis(struct adrv906x_mac *mac)
{
	void __iomem *emac_tx = mac->emac_tx;
	unsigned int val;

	val = ioread32(emac_tx + MAC_TX_CTRL);
	val &= ~MAC_TX_PATH_EN;
	iowrite32(val, emac_tx + MAC_TX_CTRL);
}

void adrv906x_mac_rx_path_en(struct adrv906x_mac *mac)
{
	void __iomem *emac_rx = mac->emac_rx;
	unsigned int val;

	val = ioread32(emac_rx + MAC_RX_CTRL);
	val |= MAC_RX_PATH_EN;
	iowrite32(val, emac_rx + MAC_RX_CTRL);
}

void adrv906x_mac_rx_path_dis(struct adrv906x_mac *mac)
{
	void __iomem *emac_rx = mac->emac_rx;
	unsigned int val;

	val = ioread32(emac_rx + MAC_RX_CTRL);
	val &= ~MAC_RX_PATH_EN;
	iowrite32(val, emac_rx + MAC_RX_CTRL);
}

void adrv906x_mac_set_path(struct adrv906x_mac *mac, bool enable)
{
	if (enable) {
		adrv906x_mac_rx_path_en(mac);
		adrv906x_mac_tx_path_en(mac);
	} else {
		adrv906x_mac_tx_path_dis(mac);
		adrv906x_mac_rx_path_dis(mac);
	}
}

void adrv906x_mac_set_multicast_filter(struct adrv906x_mac *mac, u64 mac_addr, int filter_id)
{
	void __iomem *emac_rx = mac->emac_rx;
	unsigned int low, high, val;

	low = FIELD_GET(0x0000FFFFFFFF, mac_addr);
	high = FIELD_GET(0xFFFF00000000, mac_addr);

	iowrite32(low, emac_rx + CFG_MULT_ADDR0_LOW + filter_id * 4);
	iowrite32(high, emac_rx + CFG_MULT_ADDR0_HIGH + filter_id * 4);
	val = ioread32(emac_rx + MAC_RX_CTRL);
	val |= MAC_RX_PERMITTABLE_ADDR0_EN << filter_id;
	iowrite32(val, emac_rx + MAC_RX_CTRL);
}

static void adrv906x_mac_set_mfs(struct adrv906x_mac *mac, unsigned int mfs)
{
	unsigned int val_tx, val_rx;

	val_tx = ioread32(mac->emac_tx + MAC_TX_CTRL);
	val_rx = ioread32(mac->emac_rx + MAC_RX_CTRL);

	val_tx &= ~MAC_TX_MFS;
	val_tx |= FIELD_PREP(MAC_TX_MFS, mfs);
	val_rx &= ~MAC_RX_MFS;
	val_rx |= FIELD_PREP(MAC_RX_MFS, mfs);

	iowrite32(val_tx, mac->emac_tx + MAC_TX_CTRL);
	iowrite32(val_rx, mac->emac_rx + MAC_RX_CTRL);
}

static void adrv906x_mac_update_general_stats(void __iomem *base,
					      struct adrv906x_mac_general_stats *gs)
{
	unsigned int val;

	val = ioread32(base + GMAC_STAT_DROP_EVENTS);
	gs->drop_events += val;
	val = ioread32(base + GMAC_STAT_OCTETS);
	gs->octets += val;
	val = ioread32(base + GMAC_STAT_PKTS);
	gs->pkts += val;
	val = ioread32(base + GMAC_STAT_BROADCAST_PKTS);
	gs->broadcast_pkts += val;
	val = ioread32(base + GMAC_STAT_MULTICAST_PKTS);
	gs->multicast_pkts += val;
	val = ioread32(base + GMAC_STAT_UNICAST_PKTS);
	gs->unicast_pkts += val;
	val = ioread32(base + GMAC_STAT_UNDERSIZE_PKTS);
	gs->undersize_pkts += val;
	val = ioread32(base + GMAC_STAT_OVERSIZE_PKTS);
	gs->oversize_pkts += val;
	val = ioread32(base + GMAC_STAT_PKTS_64_OCTETS);
	gs->pkts_64_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_65TO127_OCTETS);
	gs->pkts_65to127_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_128TO255_OCTETS);
	gs->pkts_128to255_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_256TO511_OCTETS);
	gs->pkts_256to511_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_512TO1023_OCTETS);
	gs->pkts_512to1023_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_1024TO1518_OCTETS);
	gs->pkts_1024to1518_octets += val;
	val = ioread32(base + GMAC_STAT_PKTS_1519TOX_OCTETS);
	gs->pkts_1519tox_octets += val;
}

static void adrv906x_mac_update_hw_stats(struct work_struct *work)
{
	struct adrv906x_mac *mac = container_of(work, struct adrv906x_mac, update_stats.work);
	unsigned int val;

	rtnl_lock();
	val = ioread32(mac->xmac + MAC_GENERAL_CONTROL);
	val |= TX_STATS_SNAPSHOT_BIT | RX_STATS_SNAPSHOT_BIT;
	iowrite32(val, mac->xmac + MAC_GENERAL_CONTROL);
	val = ioread32(mac->emac_tx + MAC_TX_STAT_UNDERFLOW);
	mac->hw_stats_tx.underflow += val;
	val = ioread32(mac->emac_tx + MAC_TX_STAT_PADDED);
	mac->hw_stats_tx.padded += val;
	adrv906x_mac_update_general_stats(mac->emac_tx,
					  &mac->hw_stats_tx.general_stats);

	val = ioread32(mac->emac_rx + MAC_RX_STAT_OVERFLOW);
	mac->hw_stats_rx.overflow += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_CRC_ERRORS);
	mac->hw_stats_rx.crc_errors += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_MC_DROP);
	mac->hw_stats_rx.mc_drop += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_FRAGMENTS);
	mac->hw_stats_rx.fragments += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_JABBERS);
	mac->hw_stats_rx.jabbers += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_MAC_FRAMING_ERROR);
	mac->hw_stats_rx.mac_framing_error += val;
	val = ioread32(mac->emac_rx + MAC_RX_STAT_RS_FRAMING_ERROR);
	mac->hw_stats_rx.rs_framing_error += val;
	adrv906x_mac_update_general_stats(mac->emac_rx,
					  &mac->hw_stats_rx.general_stats);
	rtnl_unlock();

	mod_delayed_work(system_long_wq, &mac->update_stats, msecs_to_jiffies(1000));
}

void adrv906x_mac_cleanup(struct adrv906x_mac *mac)
{
	cancel_delayed_work(&mac->update_stats);
}

int adrv906x_mac_init(struct adrv906x_mac *mac, unsigned int size)
{
	adrv906x_mac_set_mfs(mac, size);
	adrv906x_mac_promiscuous_mode_en(mac);

	mac->id = ioread32(mac->xmac + MAC_IP_ID);
	mac->version = ioread32(mac->xmac + MAC_IP_VERSION);
	mac->cap = ioread32(mac->xmac + MAC_IP_CAPABILITIES);

	adrv906x_mac_tx_path_dis(mac);
	adrv906x_mac_rx_path_dis(mac);

	INIT_DELAYED_WORK(&mac->update_stats, adrv906x_mac_update_hw_stats);
	mod_delayed_work(system_long_wq, &mac->update_stats, msecs_to_jiffies(1000));

	return 0;
}

MODULE_LICENSE("GPL");
