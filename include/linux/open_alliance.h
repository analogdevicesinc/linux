// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/**
 * Open Alliance SPI protocol driver for 10BASE-T1x Ethernet MAC-PHY
 *
 * Copyright 2021 Analog Devices Inc.
 */

 #ifndef __OPEN_ALLIANCE_H
 #define __OPEN_ALLIANCE_H

#include <linux/cache.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>

#include <asm/unaligned.h>

/* Register Memory Maps */
#define OPEN_ALLIANCE_CTRL_ST		0

/* Register Memory Map 0 - Standard Control and Status Registers */

/* Configuration Register #0 */
#define OPEN_ALLIANCE_CONFIG0		0x0004
#define   OPEN_ALLIANCE_CONFIG0_SYNC	BIT(15)
#define   OPEN_ALLIANCE_CONFIG0_ZARFE	BIT(12)

/* Buffer Status Register */
#define OPEN_ALLIANCE_BUFSTS		0x000B
#define   OPEN_ALLIANCE_BUFSTS_TXC	GENMASK(15, 8)
#define   OPEN_ALLIANCE_BUFSTS_RCA	GENMASK(7, 0)

/* Transmit/Receiver Data common bitfields */
#define OPEN_ALLIANCE_VS	GENMASK(23, 22)
#define OPEN_ALLIANCE_DV	BIT(21)
#define OPEN_ALLIANCE_SV	BIT(20)
#define OPEN_ALLIANCE_SWO	GENMASK(19, 16)
#define OPEN_ALLIANCE_EV	BIT(14)
#define OPEN_ALLIANCE_EBO	GENMASK(13, 8)
#define OPEN_ALLIANCE_P		BIT(0)

/* Transmit Data Header bitfields */
#define OPEN_ALLIANCE_DNC	BIT(31)
#define OPEN_ALLIANCE_SEQ	BIT(30)
#define OPEN_ALLIANCE_NORX	BIT(29)
#define OPEN_ALLIANCE_TSC	GENMASK(7, 6)

/* Receive Data Footer bitfields */
#define OPEN_ALLIANCE_EXST	BIT(31)
#define OPEN_ALLIANCE_HDRB	BIT(30)
#define OPEN_ALLIANCE_SYNC	BIT(29)
#define OPEN_ALLIANCE_RCA	GENMASK(28, 24)
#define OPEN_ALLIANCE_FD	BIT(15)
#define OPEN_ALLIANCE_RTSA	BIT(7)
#define OPEN_ALLIANCE_RTSP	BIT(6)
#define OPEN_ALLIANCE_TXC	GENMASK(5, 1)

/* Control Command Header bitfields */
#define OPEN_ALLIANCE_WNR	BIT(29)
#define OPEN_ALLIANCE_AID	BIT(28)
#define OPEN_ALLIANCE_MMS	GENMASK(27, 24)
#define OPEN_ALLIANCE_ADDR	GENMASK(23, 8)
#define OPEN_ALLIANCE_LEN	GENMASK(7, 1)

/* Register Memory Maps */
#define OPEN_ALLIANCE_MMS_CTRL		0
#define OPEN_ALLIANCE_MMS_MAC		1

#define OPEN_ALLIANCE_MAX_BUFF		2048
#define OPEN_ALLIANCE_MAX_CHUNKS_NR	2048
#define OPEN_ALLIANCE_MAX_CHUNK_DATA	64
#define OPEN_ALLIANCE_HEADER_SIZE	4
#define OPEN_ALLIANCE_MAX_REG_SIZE	4
#define OPEN_ALLIANCE_MAX_PORTS		4

struct open_alliance_data_tx_chunk {
	u8 dnc:1;
	u8 seq:1;
	u8 norx:1;
	u8 vs:2;
	u8 dv:1;
	u8 sv:1;
	u8 swo:4;
	u8 ev:1;
	u8 ebo:6;
	u8 tsc:2;
	u8 data[OPEN_ALLIANCE_MAX_CHUNK_DATA];
};

struct open_alliance_data_rx_chunk {
	u8 exst:1;
	u8 hdrb:1;
	u8 sync:1;
	u8 rca:5;
	u8 vs:2;
	u8 dv:1;
	u8 sv:1;
	u8 swo:4;
	u8 fd:1;
	u8 ev:1;
	u8 ebo:6;
	u8 rtsa:1;
	u8 rtsp:1;
	u8 txc:5;
	u8 tsc:2;
	u8 data[OPEN_ALLIANCE_MAX_CHUNK_DATA];
};

struct open_alliance_ctrl_tx_chunk {
	u8 wnr:1;
	u8 aid:1;
	u8 mms:3;
	u16 addr;
	u8 len:7;
	u8 data[OPEN_ALLIANCE_MAX_CHUNK_DATA];
};

struct open_alliance_rx_frame {
	int port;
	int chunks_len;
};

enum open_alliance_stats {
	OA_RX_CHUNKS_RECEIVED = 0,
	OA_TX_CHUNKS_SENT,
	OA_RX_END_VALID_RECEIVED,
	OA_RX_CHUNKS_TO_FRAMES,
	OA_CHUNKS_TRANSFERED,
};

struct open_alliance_statistic {
	enum open_alliance_stats nr;
	const char name[ETH_GSTRING_LEN];
};

static const struct open_alliance_statistic open_alliance_statistics[] = {
	{ OA_RX_CHUNKS_RECEIVED, "RX valid data chunks received", },
	{ OA_TX_CHUNKS_SENT, "TX chunks sent", },
	{ OA_RX_END_VALID_RECEIVED, "RX end valid chunks received", },
	{ OA_RX_CHUNKS_TO_FRAMES, "RX chunks to frames", },
	{ OA_CHUNKS_TRANSFERED, "OA chunks transfered", },
};

struct open_alliance {
	struct mutex		spi_lock; /* protect spi */
	spinlock_t		rx_lock; /* protect rx chunks fifo */
	spinlock_t		tx_lock; /* protect tx chunks fifo */
	struct spi_device	*spidev;
	struct net_device	*netdevs[OPEN_ALLIANCE_MAX_PORTS];
	u64			statistics[ARRAY_SIZE(open_alliance_statistics)];
	u32			chunk_size;
	bool			protected;
	u32			nr_ports;
	u32			vs_mask;
	struct napi_struct	napi;
	bool			enabled;
	int 			(*set_rx_irq)(void *, bool);
	void 			*priv;
	struct work_struct	irq_rx_work; /* enable/disable RX Ready IRQs */
	struct task_struct	*spi_thread;
	u32			txc;
	u32			rca;
	bool			rx_irq_masked;
	DECLARE_KFIFO_PTR(tx_chunks_kfifo, struct open_alliance_data_tx_chunk);
	DECLARE_KFIFO_PTR(rx_ports_fifo, struct open_alliance_rx_frame);
	DECLARE_KFIFO_PTR(rx_chunks_fifos, struct open_alliance_data_rx_chunk)[OPEN_ALLIANCE_MAX_PORTS];
	u8			frame_data[OPEN_ALLIANCE_MAX_BUFF];
	u8			tx_data[OPEN_ALLIANCE_MAX_BUFF];
	u8			rx_data[OPEN_ALLIANCE_MAX_BUFF] ____cacheline_aligned;
};

int open_alliance_net_open(struct open_alliance *oa);
int open_alliance_net_stop(struct open_alliance *oa);
int open_alliance_write_reg(struct open_alliance *oa, u8 mms, u16 addr,
			    u32 val);
int open_alliance_read_reg(struct open_alliance *oa, u8 mms, u16 addr,
			   u32 *val);
int open_alliance_write_txb(struct open_alliance *oa, struct sk_buff *txb, u8 vs);
int open_alliance_read_rxb(struct open_alliance *oa);
int open_alliance_tx_space(struct open_alliance *oa);
struct open_alliance *open_alliance_init(struct spi_device *spidev,
					 struct net_device *netdevs[OPEN_ALLIANCE_MAX_PORTS],
					 u32 nr_ports,
					 u32 vs_mask,
					 int (*set_rx_irq)(void *, bool),
				 	 void *p);

int open_alliance_remove(struct open_alliance *oa);
void open_alliance_ethtool_get_strings(struct open_alliance *oa, u32 sset, u8 *data);
int open_alliance_ethtool_get_sset_count(struct open_alliance *oa, int sset);
void open_alliance_ethtool_get_stats(struct open_alliance *oa, struct ethtool_stats *stats,
				     u64 *data);

#endif /* __OPEN_ALLIANCE_H */
