// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_MAC_H__
#define __ADRV906X_MAC_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/io.h>

#define MAC_IP_ID                        0x00000000
#define MAC_IP_VERSION                   0x00000004
#define MAC_IP_CAPABILITIES              0x0000000C
#define MAC_GENERAL_CONTROL              0x00000010
#define   TX_STATS_SNAPSHOT_BIT          BIT(8)
#define   RX_STATS_SNAPSHOT_BIT          BIT(12)

#define MAC_TX_CTRL                      0x00000000
#define   MAC_TX_PATH_EN                 BIT(0)
#define   MAC_TX_MFS                     GENMASK(29, 16)
#define MAC_TX_STAT_UNDERFLOW            0x00000060
#define MAC_TX_STAT_PADDED               0x00000068

#define MAC_RX_CTRL                      0x00000000
#define   MAC_RX_PATH_EN                 BIT(0)
#define   MAC_RX_PROMISCUOUS_MODE_EN     BIT(3)
#define   MAC_RX_PERMITTABLE_ADDR0_EN    BIT(4)
#define   MAC_RX_PERMITTABLE_ADDR1_EN    BIT(5)
#define   MAC_RX_PERMITTABLE_ADDR2_EN    BIT(6)
#define   MAC_RX_MFS                     GENMASK(29, 16)
#define MAC_RX_STAT_OVERFLOW             0x00000060
#define MAC_RX_STAT_CRC_ERRORS           0x00000068
#define MAC_RX_STAT_MC_DROP              0x00000070
#define MAC_RX_STAT_FRAGMENTS            0x00000078
#define MAC_RX_STAT_JABBERS              0x00000080
#define MAC_RX_STAT_MAC_FRAMING_ERROR    0x0000008C
#define MAC_RX_STAT_RS_FRAMING_ERROR     0x00000094

#define GMAC_STAT_DROP_EVENTS            0x00000100
#define GMAC_STAT_OCTETS                 0x00000108
#define GMAC_STAT_PKTS                   0x00000110
#define GMAC_STAT_BROADCAST_PKTS         0x00000118
#define GMAC_STAT_MULTICAST_PKTS         0x00000120
#define GMAC_STAT_UNICAST_PKTS           0x00000128
#define GMAC_STAT_UNDERSIZE_PKTS         0x00000130
#define GMAC_STAT_OVERSIZE_PKTS          0x00000138
#define GMAC_STAT_PKTS_64_OCTETS         0x00000140
#define GMAC_STAT_PKTS_65TO127_OCTETS    0x00000148
#define GMAC_STAT_PKTS_128TO255_OCTETS   0x00000150
#define GMAC_STAT_PKTS_256TO511_OCTETS   0x00000158
#define GMAC_STAT_PKTS_512TO1023_OCTETS  0x00000160
#define GMAC_STAT_PKTS_1024TO1518_OCTETS 0x00000168
#define GMAC_STAT_PKTS_1519TOX_OCTETS    0x00000170

#define CFG_MULT_ADDR0_LOW               0x00000020
#define CFG_MULT_ADDR1_LOW               0x00000024
#define CFG_MULT_ADDR2_LOW               0x00000028
#define CFG_MULT_ADDR0_HIGH              0x00000030
#define CFG_MULT_ADDR1_HIGH              0x00000034
#define CFG_MULT_ADDR2_HIGH              0x00000038

struct adrv906x_mac_general_stats {
	u64 drop_events;
	u64 octets;
	u64 pkts;
	u64 broadcast_pkts;
	u64 multicast_pkts;
	u64 unicast_pkts;
	u64 undersize_pkts;
	u64 oversize_pkts;
	u64 pkts_64_octets;
	u64 pkts_65to127_octets;
	u64 pkts_128to255_octets;
	u64 pkts_256to511_octets;
	u64 pkts_512to1023_octets;
	u64 pkts_1024to1518_octets;
	u64 pkts_1519tox_octets;
};

struct adrv906x_mac_tx_stats {
	u64 underflow;
	u64 padded;
	struct adrv906x_mac_general_stats general_stats;
};

struct adrv906x_mac_rx_stats {
	u64 overflow;
	u64 crc_errors;
	u64 mc_drop;
	u64 fragments;
	u64 jabbers;
	u64 mac_framing_error;
	u64 rs_framing_error;
	struct adrv906x_mac_general_stats general_stats;
};

struct adrv906x_mac {
	unsigned int id;
	unsigned int version;
	unsigned int cap;
	void __iomem *xmac;
	void __iomem *emac_tx;
	void __iomem *emac_rx;
	struct adrv906x_mac_tx_stats hw_stats_tx;
	struct adrv906x_mac_rx_stats hw_stats_rx;
	struct delayed_work update_stats;
};

void adrv906x_mac_promiscuous_mode_en(struct adrv906x_mac *mac);
void adrv906x_mac_promiscuous_mode_dis(struct adrv906x_mac *mac);
void adrv906x_mac_rx_path_en(struct adrv906x_mac *mac);
void adrv906x_mac_rx_path_dis(struct adrv906x_mac *mac);
void adrv906x_mac_set_multicast_filter(struct adrv906x_mac *mac, u64 mac_addr, int filter_id);
void adrv906x_mac_cleanup(struct adrv906x_mac *mac);
int adrv906x_mac_init(struct adrv906x_mac *mac, unsigned int size);
void adrv906x_mac_set_path(struct adrv906x_mac *mac, bool enable);

#endif /* __ADRV906X_MAC_H__ */
