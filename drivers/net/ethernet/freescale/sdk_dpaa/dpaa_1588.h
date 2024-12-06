/* Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#ifndef __DPAA_1588_H__
#define __DPAA_1588_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/circ_buf.h>
#include <linux/fsl_qman.h>

#define DEFAULT_PTP_RX_BUF_SZ		256
#define DEFAULT_PTP_TX_BUF_SZ		256

/* 1588 private ioctl calls */
#define PTP_ENBL_TXTS_IOCTL	SIOCDEVPRIVATE
#define PTP_DSBL_TXTS_IOCTL	(SIOCDEVPRIVATE + 1)
#define PTP_ENBL_RXTS_IOCTL	(SIOCDEVPRIVATE + 2)
#define PTP_DSBL_RXTS_IOCTL	(SIOCDEVPRIVATE + 3)
#define PTP_GET_TX_TIMESTAMP	(SIOCDEVPRIVATE + 4)
#define PTP_GET_RX_TIMESTAMP	(SIOCDEVPRIVATE + 5)
#define PTP_SET_TIME		(SIOCDEVPRIVATE + 6)
#define PTP_GET_TIME		(SIOCDEVPRIVATE + 7)
#define PTP_SET_FIPER_ALARM	(SIOCDEVPRIVATE + 8)
#define PTP_SET_ADJ		(SIOCDEVPRIVATE + 9)
#define PTP_GET_ADJ		(SIOCDEVPRIVATE + 10)
#define PTP_CLEANUP_TS		(SIOCDEVPRIVATE + 11)

/* PTP V2 message type */
enum {
	PTP_MSGTYPE_SYNC		= 0x0,
	PTP_MSGTYPE_DELREQ		= 0x1,
	PTP_MSGTYPE_PDELREQ		= 0x2,
	PTP_MSGTYPE_PDELRESP		= 0x3,
	PTP_MSGTYPE_FLWUP		= 0x8,
	PTP_MSGTYPE_DELRESP		= 0x9,
	PTP_MSGTYPE_PDELRES_FLWUP	= 0xA,
	PTP_MSGTYPE_ANNOUNCE		= 0xB,
	PTP_MSGTYPE_SGNLNG		= 0xC,
	PTP_MSGTYPE_MNGMNT		= 0xD,
};

/* Byte offset of data in the PTP V2 headers */
#define PTP_OFFS_MSG_TYPE		0
#define PTP_OFFS_VER_PTP		1
#define PTP_OFFS_MSG_LEN		2
#define PTP_OFFS_DOM_NMB		4
#define PTP_OFFS_FLAGS			6
#define PTP_OFFS_CORFIELD		8
#define PTP_OFFS_SRCPRTID		20
#define PTP_OFFS_SEQ_ID			30
#define PTP_OFFS_CTRL			32
#define PTP_OFFS_LOGMEAN		33

#define PTP_IP_OFFS			14
#define PTP_UDP_OFFS			34
#define PTP_HEADER_OFFS			42
#define PTP_MSG_TYPE_OFFS		(PTP_HEADER_OFFS + PTP_OFFS_MSG_TYPE)
#define PTP_SPORT_ID_OFFS		(PTP_HEADER_OFFS + PTP_OFFS_SRCPRTID)
#define PTP_SEQ_ID_OFFS			(PTP_HEADER_OFFS + PTP_OFFS_SEQ_ID)
#define PTP_CTRL_OFFS			(PTP_HEADER_OFFS + PTP_OFFS_CTRL)

/* 1588-2008 network protocol enumeration values */
#define DPA_PTP_PROT_IPV4		1
#define DPA_PTP_PROT_IPV6		2
#define DPA_PTP_PROT_802_3		3
#define DPA_PTP_PROT_DONTCARE		0xFFFF

#define DPA_PTP_SOURCE_PORT_LENGTH	10
#define DPA_PTP_HEADER_SZE		34
#define DPA_ETYPE_LEN			2
#define DPA_VLAN_TAG_LEN		4
#define NANOSEC_PER_SECOND		1000000000

/* The threshold between the current found one and the oldest one */
#define TS_ACCUMULATION_THRESHOLD	50

/* Struct needed to identify a timestamp */
struct dpa_ptp_ident {
	u8	version;
	u8	msg_type;
	u16	netw_prot;
	u16	seq_id;
	u8	snd_port_id[DPA_PTP_SOURCE_PORT_LENGTH];
};

/* Timestamp format in 1588-2008 */
struct dpa_ptp_time {
	u64	sec;	/* just 48 bit used */
	u32	nsec;
};

/* needed for timestamp data over ioctl */
struct dpa_ptp_data {
	struct dpa_ptp_ident	ident;
	struct dpa_ptp_time	ts;
};

struct dpa_ptp_circ_buf {
	struct circ_buf circ_buf;
	u32 size;
	spinlock_t ptp_lock;
};

/* PTP TSU control structure */
struct dpa_ptp_tsu {
	struct dpa_priv_s *dpa_priv;
	bool valid;
	struct dpa_ptp_circ_buf rx_timestamps;
	struct dpa_ptp_circ_buf tx_timestamps;

	/* HW timestamping over ioctl enabled flag */
	int hwts_tx_en_ioctl;
	int hwts_rx_en_ioctl;
};

extern int dpa_ptp_init(struct dpa_priv_s *priv);
extern void dpa_ptp_cleanup(struct dpa_priv_s *priv);
extern void dpa_ptp_store_txstamp(const struct dpa_priv_s *priv,
				struct sk_buff *skb, void *data);
extern void dpa_ptp_store_rxstamp(const struct dpa_priv_s *priv,
				struct sk_buff *skb, void *data);
extern int dpa_ioctl_1588(struct net_device *dev, struct ifreq *ifr, int cmd);
#endif
