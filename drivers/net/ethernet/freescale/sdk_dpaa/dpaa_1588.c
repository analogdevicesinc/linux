/* Copyright (C) 2011 Freescale Semiconductor, Inc.
 * Copyright (C) 2009 IXXAT Automation, GmbH
 *
 * DPAA Ethernet Driver -- IEEE 1588 interface functionality
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
#include <linux/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <asm/div64.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "dpaa_1588.h"
#include "mac.h"

static int dpa_ptp_init_circ(struct dpa_ptp_circ_buf *ptp_buf, u32 size)
{
	struct circ_buf *circ_buf = &ptp_buf->circ_buf;

	circ_buf->buf = vmalloc(sizeof(struct dpa_ptp_data) * size);
	if (!circ_buf->buf)
		return 1;

	circ_buf->head = 0;
	circ_buf->tail = 0;
	ptp_buf->size = size;
	spin_lock_init(&ptp_buf->ptp_lock);

	return 0;
}

static void dpa_ptp_reset_circ(struct dpa_ptp_circ_buf *ptp_buf, u32 size)
{
	struct circ_buf *circ_buf = &ptp_buf->circ_buf;

	circ_buf->head = 0;
	circ_buf->tail = 0;
	ptp_buf->size = size;
}

static int dpa_ptp_insert(struct dpa_ptp_circ_buf *ptp_buf,
			  struct dpa_ptp_data *data)
{
	struct circ_buf *circ_buf = &ptp_buf->circ_buf;
	int size = ptp_buf->size;
	struct dpa_ptp_data *tmp;
	unsigned long flags;
	int head, tail;

	spin_lock_irqsave(&ptp_buf->ptp_lock, flags);

	head = circ_buf->head;
	tail = circ_buf->tail;

	if (CIRC_SPACE(head, tail, size) <= 0)
		circ_buf->tail = (tail + 1) & (size - 1);

	tmp = (struct dpa_ptp_data *)(circ_buf->buf) + head;
	memcpy(tmp, data, sizeof(struct dpa_ptp_data));

	circ_buf->head = (head + 1) & (size - 1);

	spin_unlock_irqrestore(&ptp_buf->ptp_lock, flags);

	return 0;
}

static int dpa_ptp_is_ident_match(struct dpa_ptp_ident *dst,
				  struct dpa_ptp_ident *src)
{
	int ret;

	if ((dst->version != src->version) || (dst->msg_type != src->msg_type))
		return 0;

	if ((dst->netw_prot == src->netw_prot)
			|| src->netw_prot == DPA_PTP_PROT_DONTCARE) {
		if (dst->seq_id != src->seq_id)
			return 0;

		ret = memcmp(dst->snd_port_id, src->snd_port_id,
				DPA_PTP_SOURCE_PORT_LENGTH);
		if (ret)
			return 0;
		else
			return 1;
	}

	return 0;
}

static int dpa_ptp_find_and_remove(struct dpa_ptp_circ_buf *ptp_buf,
				   struct dpa_ptp_ident *ident,
				   struct dpa_ptp_time *ts)
{
	struct circ_buf *circ_buf = &ptp_buf->circ_buf;
	int size = ptp_buf->size;
	int head, tail, idx;
	unsigned long flags;
	struct dpa_ptp_data *tmp, *tmp2;
	struct dpa_ptp_ident *tmp_ident;

	spin_lock_irqsave(&ptp_buf->ptp_lock, flags);

	head = circ_buf->head;
	tail = idx = circ_buf->tail;

	if (CIRC_CNT(head, tail, size) == 0) {
		spin_unlock_irqrestore(&ptp_buf->ptp_lock, flags);
		return 1;
	}

	while (idx != head) {
		tmp = (struct dpa_ptp_data *)(circ_buf->buf) + idx;
		tmp_ident = &tmp->ident;
		if (dpa_ptp_is_ident_match(tmp_ident, ident))
			break;
		idx = (idx + 1) & (size - 1);
	}

	if (idx == head) {
		spin_unlock_irqrestore(&ptp_buf->ptp_lock, flags);
		return 1;
	}

	ts->sec = tmp->ts.sec;
	ts->nsec = tmp->ts.nsec;

	if (idx != tail) {
		if (CIRC_CNT(idx, tail, size) > TS_ACCUMULATION_THRESHOLD) {
			tail = circ_buf->tail =
				(idx - TS_ACCUMULATION_THRESHOLD) & (size - 1);
		}

		while (CIRC_CNT(idx, tail, size) > 0) {
			tmp = (struct dpa_ptp_data *)(circ_buf->buf) + idx;
			idx = (idx - 1) & (size - 1);
			tmp2 = (struct dpa_ptp_data *)(circ_buf->buf) + idx;
			*tmp = *tmp2;
		}
	}
	circ_buf->tail = (tail + 1) & (size - 1);

	spin_unlock_irqrestore(&ptp_buf->ptp_lock, flags);

	return 0;
}

/* Parse the PTP packets
 *
 * The PTP header can be found in an IPv4 packet, IPv6 patcket or in
 * an IEEE802.3 ethernet frame. This function returns the position of
 * the PTP packet or NULL if no PTP found
 */
static u8 *dpa_ptp_parse_packet(struct sk_buff *skb, u16 *eth_type)
{
	u8 *pos = skb->data + ETH_ALEN + ETH_ALEN;
	u8 *ptp_loc = NULL;
	u8 msg_type;
	u32 access_len = ETH_ALEN + ETH_ALEN + DPA_ETYPE_LEN;
	struct iphdr *iph;
	struct udphdr *udph;
	struct ipv6hdr *ipv6h;

	/* when we can receive S/G frames we need to check the data we want to
	 * access is in the linear skb buffer
	 */
	if (!pskb_may_pull(skb, access_len))
		return NULL;

	*eth_type = *((u16 *)pos);

	/* Check if inner tag is here */
	if (*eth_type == ETH_P_8021Q) {
		access_len += DPA_VLAN_TAG_LEN;

		if (!pskb_may_pull(skb, access_len))
			return NULL;

		pos += DPA_VLAN_TAG_LEN;
		*eth_type = *((u16 *)pos);
	}

	pos += DPA_ETYPE_LEN;

	switch (*eth_type) {
	/* Transport of PTP over Ethernet */
	case ETH_P_1588:
		ptp_loc = pos;

		if (!pskb_may_pull(skb, access_len + PTP_OFFS_MSG_TYPE + 1))
			return NULL;

		msg_type = *((u8 *)(ptp_loc + PTP_OFFS_MSG_TYPE)) & 0xf;
		if ((msg_type == PTP_MSGTYPE_SYNC)
			|| (msg_type == PTP_MSGTYPE_DELREQ)
			|| (msg_type == PTP_MSGTYPE_PDELREQ)
			|| (msg_type == PTP_MSGTYPE_PDELRESP))
				return ptp_loc;
		break;
	/* Transport of PTP over IPv4 */
	case ETH_P_IP:
		iph = (struct iphdr *)pos;
		access_len += sizeof(struct iphdr);

		if (!pskb_may_pull(skb, access_len))
			return NULL;

		if (ntohs(iph->protocol) != IPPROTO_UDP)
			return NULL;

		access_len += iph->ihl * 4 - sizeof(struct iphdr) +
				sizeof(struct udphdr);

		if (!pskb_may_pull(skb, access_len))
			return NULL;

		pos += iph->ihl * 4;
		udph = (struct udphdr *)pos;
		if (ntohs(udph->dest) != 319)
			return NULL;
		ptp_loc = pos + sizeof(struct udphdr);
		break;
	/* Transport of PTP over IPv6 */
	case ETH_P_IPV6:
		ipv6h = (struct ipv6hdr *)pos;

		access_len += sizeof(struct ipv6hdr) + sizeof(struct udphdr);

		if (ntohs(ipv6h->nexthdr) != IPPROTO_UDP)
			return NULL;

		pos += sizeof(struct ipv6hdr);
		udph = (struct udphdr *)pos;
		if (ntohs(udph->dest) != 319)
			return NULL;
		ptp_loc = pos + sizeof(struct udphdr);
		break;
	default:
		break;
	}

	return ptp_loc;
}

static int dpa_ptp_store_stamp(const struct dpa_priv_s *priv,
		struct sk_buff *skb, void *data, enum port_type rx_tx,
		struct dpa_ptp_data *ptp_data)
{
	u64 nsec;
	u32 mod;
	u8 *ptp_loc;
	u16 eth_type;

	ptp_loc = dpa_ptp_parse_packet(skb, &eth_type);
	if (!ptp_loc)
		return -EINVAL;

	switch (eth_type) {
	case ETH_P_IP:
		ptp_data->ident.netw_prot = DPA_PTP_PROT_IPV4;
		break;
	case ETH_P_IPV6:
		ptp_data->ident.netw_prot = DPA_PTP_PROT_IPV6;
		break;
	case ETH_P_1588:
		ptp_data->ident.netw_prot = DPA_PTP_PROT_802_3;
		break;
	default:
		return -EINVAL;
	}

	if (!pskb_may_pull(skb, ptp_loc - skb->data + PTP_OFFS_SEQ_ID + 2))
		return -EINVAL;

	ptp_data->ident.version = *(ptp_loc + PTP_OFFS_VER_PTP) & 0xf;
	ptp_data->ident.msg_type = *(ptp_loc + PTP_OFFS_MSG_TYPE) & 0xf;
	ptp_data->ident.seq_id = *((u16 *)(ptp_loc + PTP_OFFS_SEQ_ID));
	memcpy(ptp_data->ident.snd_port_id, ptp_loc + PTP_OFFS_SRCPRTID,
			DPA_PTP_SOURCE_PORT_LENGTH);

	nsec = dpa_get_timestamp_ns(priv, rx_tx, data);
	mod = do_div(nsec, NANOSEC_PER_SECOND);
	ptp_data->ts.sec = nsec;
	ptp_data->ts.nsec = mod;

	return 0;
}

void dpa_ptp_store_txstamp(const struct dpa_priv_s *priv,
				struct sk_buff *skb, void *data)
{
	struct dpa_ptp_tsu *tsu = priv->tsu;
	struct dpa_ptp_data ptp_tx_data;

	if (dpa_ptp_store_stamp(priv, skb, data, TX, &ptp_tx_data))
		return;

	dpa_ptp_insert(&tsu->tx_timestamps, &ptp_tx_data);
}

void dpa_ptp_store_rxstamp(const struct dpa_priv_s *priv,
				struct sk_buff *skb, void *data)
{
	struct dpa_ptp_tsu *tsu = priv->tsu;
	struct dpa_ptp_data ptp_rx_data;

	if (dpa_ptp_store_stamp(priv, skb, data, RX, &ptp_rx_data))
		return;

	dpa_ptp_insert(&tsu->rx_timestamps, &ptp_rx_data);
}

static uint8_t dpa_get_tx_timestamp(struct dpa_ptp_tsu *ptp_tsu,
				    struct dpa_ptp_ident *ident,
				    struct dpa_ptp_time *ts)
{
	struct dpa_ptp_tsu *tsu = ptp_tsu;
	struct dpa_ptp_time tmp;
	int flag;

	flag = dpa_ptp_find_and_remove(&tsu->tx_timestamps, ident, &tmp);
	if (!flag) {
		ts->sec = tmp.sec;
		ts->nsec = tmp.nsec;
		return 0;
	}

	return -1;
}

static uint8_t dpa_get_rx_timestamp(struct dpa_ptp_tsu *ptp_tsu,
				    struct dpa_ptp_ident *ident,
				    struct dpa_ptp_time *ts)
{
	struct dpa_ptp_tsu *tsu = ptp_tsu;
	struct dpa_ptp_time tmp;
	int flag;

	flag = dpa_ptp_find_and_remove(&tsu->rx_timestamps, ident, &tmp);
	if (!flag) {
		ts->sec = tmp.sec;
		ts->nsec = tmp.nsec;
		return 0;
	}

	return -1;
}

static void dpa_set_fiper_alarm(struct dpa_ptp_tsu *tsu,
		struct dpa_ptp_time *cnt_time)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u64 tmp, fiper;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(get_fm_handle(tsu->dpa_priv->net_dev));

	/* TMR_FIPER1 will pulse every second after ALARM1 expired */
	tmp = (u64)cnt_time->sec * NANOSEC_PER_SECOND + (u64)cnt_time->nsec;
	fiper = NANOSEC_PER_SECOND - DPA_PTP_NOMINAL_FREQ_PERIOD_NS;
	if (mac_dev->fm_rtc_set_alarm)
		mac_dev->fm_rtc_set_alarm(get_fm_handle(tsu->dpa_priv->net_dev),
					  0, tmp);
	if (mac_dev->fm_rtc_set_fiper)
		mac_dev->fm_rtc_set_fiper(get_fm_handle(tsu->dpa_priv->net_dev),
					  0, fiper);

	if (mac_dev->fm_rtc_enable)
		mac_dev->fm_rtc_enable(get_fm_handle(tsu->dpa_priv->net_dev));
}

static void dpa_get_curr_cnt(struct dpa_ptp_tsu *tsu,
		struct dpa_ptp_time *curr_time)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u64 tmp;
	u32 mod;

	if (mac_dev->fm_rtc_get_cnt)
		mac_dev->fm_rtc_get_cnt(get_fm_handle(tsu->dpa_priv->net_dev),
					&tmp);

	mod = do_div(tmp, NANOSEC_PER_SECOND);
	curr_time->sec = (u32)tmp;
	curr_time->nsec = mod;
}

static void dpa_set_1588cnt(struct dpa_ptp_tsu *tsu,
		struct dpa_ptp_time *cnt_time)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u64 tmp;

	tmp = (u64)cnt_time->sec * NANOSEC_PER_SECOND + (u64)cnt_time->nsec;

	if (mac_dev->fm_rtc_set_cnt)
		mac_dev->fm_rtc_set_cnt(get_fm_handle(tsu->dpa_priv->net_dev),
					tmp);

	/* Restart fiper two seconds later */
	cnt_time->sec += 2;
	cnt_time->nsec = 0;
	dpa_set_fiper_alarm(tsu, cnt_time);
}

static void dpa_get_drift(struct dpa_ptp_tsu *tsu, u32 *addend)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;
	u32 drift;

	if (mac_dev->fm_rtc_get_drift)
		mac_dev->fm_rtc_get_drift(get_fm_handle(tsu->dpa_priv->net_dev),
					  &drift);

	*addend = drift;
}

static void dpa_set_drift(struct dpa_ptp_tsu *tsu, u32 addend)
{
	struct mac_device *mac_dev = tsu->dpa_priv->mac_dev;

	if (mac_dev->fm_rtc_set_drift)
		mac_dev->fm_rtc_set_drift(get_fm_handle(tsu->dpa_priv->net_dev),
					  addend);
}

static void dpa_flush_timestamp(struct dpa_ptp_tsu *tsu)
{
	dpa_ptp_reset_circ(&tsu->rx_timestamps, DEFAULT_PTP_RX_BUF_SZ);
	dpa_ptp_reset_circ(&tsu->tx_timestamps, DEFAULT_PTP_TX_BUF_SZ);
}

int dpa_ioctl_1588(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct dpa_ptp_tsu *tsu = priv->tsu;
	struct mac_device *mac_dev = priv->mac_dev;
	struct dpa_ptp_data ptp_data;
	struct dpa_ptp_data *ptp_data_user;
	struct dpa_ptp_time act_time;
	u32 addend;
	int retval = 0;

	if (!tsu || !tsu->valid)
		return -ENODEV;

	switch (cmd) {
	case PTP_ENBL_TXTS_IOCTL:
		tsu->hwts_tx_en_ioctl = 1;
		if (mac_dev->fm_rtc_enable)
			mac_dev->fm_rtc_enable(get_fm_handle(dev));
		if (mac_dev->ptp_enable)
			mac_dev->ptp_enable(mac_dev->get_mac_handle(mac_dev));
		break;
	case PTP_DSBL_TXTS_IOCTL:
		tsu->hwts_tx_en_ioctl = 0;
		if (mac_dev->fm_rtc_disable)
			mac_dev->fm_rtc_disable(get_fm_handle(dev));
		if (mac_dev->ptp_disable)
			mac_dev->ptp_disable(mac_dev->get_mac_handle(mac_dev));
		break;
	case PTP_ENBL_RXTS_IOCTL:
		tsu->hwts_rx_en_ioctl = 1;
		break;
	case PTP_DSBL_RXTS_IOCTL:
		tsu->hwts_rx_en_ioctl = 0;
		break;
	case PTP_GET_RX_TIMESTAMP:
		ptp_data_user = (struct dpa_ptp_data *)ifr->ifr_data;
		if (copy_from_user(&ptp_data.ident,
				&ptp_data_user->ident, sizeof(ptp_data.ident)))
			return -EINVAL;

		if (dpa_get_rx_timestamp(tsu, &ptp_data.ident, &ptp_data.ts))
			return -EAGAIN;

		if (copy_to_user((void __user *)&ptp_data_user->ts,
				&ptp_data.ts, sizeof(ptp_data.ts)))
			return -EFAULT;
		break;
	case PTP_GET_TX_TIMESTAMP:
		ptp_data_user = (struct dpa_ptp_data *)ifr->ifr_data;
		if (copy_from_user(&ptp_data.ident,
				&ptp_data_user->ident, sizeof(ptp_data.ident)))
			return -EINVAL;

		if (dpa_get_tx_timestamp(tsu, &ptp_data.ident, &ptp_data.ts))
			return -EAGAIN;

		if (copy_to_user((void __user *)&ptp_data_user->ts,
				&ptp_data.ts, sizeof(ptp_data.ts)))
			return -EFAULT;
		break;
	case PTP_GET_TIME:
		dpa_get_curr_cnt(tsu, &act_time);
		if (copy_to_user(ifr->ifr_data, &act_time, sizeof(act_time)))
			return -EFAULT;
		break;
	case PTP_SET_TIME:
		if (copy_from_user(&act_time, ifr->ifr_data, sizeof(act_time)))
			return -EINVAL;
		dpa_set_1588cnt(tsu, &act_time);
		break;
	case PTP_GET_ADJ:
		dpa_get_drift(tsu, &addend);
		if (copy_to_user(ifr->ifr_data, &addend, sizeof(addend)))
			return -EFAULT;
		break;
	case PTP_SET_ADJ:
		if (copy_from_user(&addend, ifr->ifr_data, sizeof(addend)))
			return -EINVAL;
		dpa_set_drift(tsu, addend);
		break;
	case PTP_SET_FIPER_ALARM:
		if (copy_from_user(&act_time, ifr->ifr_data, sizeof(act_time)))
			return -EINVAL;
		dpa_set_fiper_alarm(tsu, &act_time);
		break;
	case PTP_CLEANUP_TS:
		dpa_flush_timestamp(tsu);
		break;
	default:
		return -EINVAL;
	}

	return retval;
}

int dpa_ptp_init(struct dpa_priv_s *priv)
{
	struct dpa_ptp_tsu *tsu;

	/* Allocate memory for PTP structure */
	tsu = kzalloc(sizeof(struct dpa_ptp_tsu), GFP_KERNEL);
	if (!tsu)
		return -ENOMEM;

	tsu->valid = TRUE;
	tsu->dpa_priv = priv;

	dpa_ptp_init_circ(&tsu->rx_timestamps, DEFAULT_PTP_RX_BUF_SZ);
	dpa_ptp_init_circ(&tsu->tx_timestamps, DEFAULT_PTP_TX_BUF_SZ);

	priv->tsu = tsu;

	return 0;
}
EXPORT_SYMBOL(dpa_ptp_init);

void dpa_ptp_cleanup(struct dpa_priv_s *priv)
{
	struct dpa_ptp_tsu *tsu = priv->tsu;

	tsu->valid = FALSE;
	vfree(tsu->rx_timestamps.circ_buf.buf);
	vfree(tsu->tx_timestamps.circ_buf.buf);

	kfree(tsu);
}
EXPORT_SYMBOL(dpa_ptp_cleanup);
