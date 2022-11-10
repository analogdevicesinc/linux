// SPDX-License-Identifier: GPL-2.0
/* Copyright 2016 Mans Rullgard <mans@mansr.com>
 * Copyright 2019 NXP
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/rtnetlink.h>
#include <linux/virtio_ring.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#ifdef CONFIG_IMX_SCU
#include <linux/firmware/imx/sci.h>
#endif
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/list.h>
#include <linux/bitops.h>

#define DRV_NAME "imx-shmem-net"

#define IMX_SHMEM_INTX_ENABLE	0x1

#define IMX_SHM_NET_STATE_RESET	0
#define IMX_SHM_NET_STATE_INIT	1
#define IMX_SHM_NET_STATE_READY	2
#define IMX_SHM_NET_STATE_RUN	3

#define IMX_SHM_NET_FLAG_RUN	0

#define IMX_SHM_NET_MTU_MIN 256
#define IMX_SHM_NET_MTU_MAX 65535
#define IMX_SHM_NET_MTU_DEF 16384

#define IMX_SHM_NET_FRAME_SIZE(s) ALIGN(18 + (s), SMP_CACHE_BYTES)

#define IMX_SHM_NET_VQ_ALIGN 64

/* (queue_size + vring_size) * 2 for TX&RX */
#define IMX_SHM_NET_DMA_SIZE (((IMX_SHM_NET_MTU_DEF * 16) + (64 * 1024)) * 2)

/* MU's Receiver Register 1 Full mask */
#define MU_SR_RF1_MASK1		BIT(26)
/* MU timeout in milliseconds when sending message */
#define IMX_SHM_NET_MU_TIMEOUT 2000
/* connected partition default id */
#define IMX_SHM_NET_DEFAULT_PART 3

#define SC_IRQ_GROUP_REBOOTED   5U      /* Partition reboot complete */

static const char * const state_name[] = {
	"RESET",
	"INIT",
	"READY",
	"RUN"
};

enum e_mess_state {
	MESS_STATE_NEW,
	MESS_STATE_PROCESSED,
};

struct imx_shmem_regs {
	u32 intxctrl;
	u32 istat;
	u32 ivpos;
	u32 doorbell;
	u32 lstate;
	u32 rstate;
};

struct imx_shm_net_queue {
	struct vring vr;
	u32 free_head;
	u32 num_free;
	u32 num_added;
	u16 last_avail_idx;
	u16 last_used_idx;

	void *data;
	void *end;
	u32 size;
	u32 head;
	u32 tail;
};

struct imx_shm_net_stats {
	u32 interrupts;
	u32 tx_packets;
	u32 tx_notify;
	u32 tx_pause;
	u32 rx_packets;
	u32 rx_notify;
	u32 napi_poll;
	u32 napi_complete;
	u32 napi_poll_n[10];
};

struct imx_shm_net {
	struct imx_shm_net_queue rx;
	struct imx_shm_net_queue tx;

	u32 vrsize;
	u32 qlen;
	u32 qsize;

	spinlock_t tx_free_lock;	/* protect available buffers */
	spinlock_t tx_clean_lock;	/* protect used buffers */

	struct napi_struct napi;

	unsigned long flags;

	struct workqueue_struct *state_wq;
	struct work_struct state_work;

	struct imx_shm_net_stats stats;

	struct imx_shmem_regs regs;
	void *shm;
	phys_addr_t shmaddr;
	resource_size_t shmlen;

	struct platform_device *pdev;

	void __iomem *mu_base;
	struct mutex state_lock;	/* protect state variable */
	struct clk *mu_clk;
	u32 remote_message;
	enum e_mess_state message_state; /* remote msg. processing status */
	u32 mub_partition;
	struct notifier_block pnotifier;
	struct list_head isn_node;
	struct mbox_client cl;
	struct mbox_chan *tx_ch;
	struct mbox_chan *rx_ch;
};

/* the head of the list of imx_shm_net devices */
static struct list_head imx_shm_net_head;

static void *imx_shm_net_desc_data(struct imx_shm_net *in,
				   struct imx_shm_net_queue *q,
				   struct vring_desc *desc,
				   u32 *len)
{
	u64 offs = READ_ONCE(desc->addr);
	u32 dlen = READ_ONCE(desc->len);
	u16 flags = READ_ONCE(desc->flags);
	void *data;

	if (flags)
		return NULL;

	if (offs >= in->shmlen)
		return NULL;

	data = in->shm + offs;

	if (data < q->data || data >= q->end)
		return NULL;

	if (dlen > q->end - data)
		return NULL;

	*len = dlen;

	return data;
}

static void imx_shm_net_init_queue(struct imx_shm_net *in,
				   struct imx_shm_net_queue *q,
				   void *mem, unsigned int len)
{
	memset(q, 0, sizeof(*q));

	vring_init(&q->vr, len, mem, IMX_SHM_NET_VQ_ALIGN);
	q->data = mem + in->vrsize;
	q->end = q->data + in->qsize;
	q->size = in->qsize;
}

static void imx_shm_net_init_queues(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	int ivpos = in->regs.ivpos;
	void *tx;
	void *rx;
	int i;

	tx = in->shm +  ivpos * in->shmlen / 2;
	rx = in->shm + !ivpos * in->shmlen / 2;

	memset(tx, 0, in->shmlen / 2);

	imx_shm_net_init_queue(in, &in->rx, rx, in->qlen);
	imx_shm_net_init_queue(in, &in->tx, tx, in->qlen);

	swap(in->rx.vr.used, in->tx.vr.used);

	in->tx.num_free = in->tx.vr.num;

	for (i = 0; i < in->tx.vr.num - 1; i++)
		in->tx.vr.desc[i].next = i + 1;
}

static int imx_shm_net_calc_qsize(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	unsigned int vrsize;
	unsigned int qsize;
	unsigned int qlen;

	for (qlen = 4096; qlen > 32; qlen >>= 1) {
		vrsize = vring_size(qlen, IMX_SHM_NET_VQ_ALIGN);
		vrsize = ALIGN(vrsize, IMX_SHM_NET_VQ_ALIGN);
		if (vrsize < in->shmlen / 16)
			break;
	}

	if (vrsize > in->shmlen / 2)
		return -EINVAL;

	qsize = in->shmlen / 2 - vrsize;

	if (qsize < 4 * IMX_SHM_NET_MTU_MIN)
		return -EINVAL;

	in->vrsize = vrsize;
	in->qlen = qlen;
	in->qsize = qsize;

	return 0;
}

static void imx_shm_net_notify_tx(struct imx_shm_net *in, unsigned int num)
{
	u16 evt, old, new;
	int ret;

	/* memory barrier */
	virt_mb();

	evt = READ_ONCE(vring_avail_event(&in->tx.vr));
	old = in->tx.last_avail_idx - num;
	new = in->tx.last_avail_idx;

	if (vring_need_event(evt, new, old)) {
		ret = mbox_send_message(in->tx_ch, &in->regs.lstate);
		if (ret < 0)
			dev_err(&in->pdev->dev, "%s send message error=%d!\n",
				__func__, ret);

		in->stats.tx_notify++;
	}
}

static void imx_shm_net_enable_rx_irq(struct imx_shm_net *in)
{
	vring_avail_event(&in->rx.vr) = in->rx.last_avail_idx;
	/* memory barrier */
	virt_wmb();
}

static void imx_shm_net_notify_rx(struct imx_shm_net *in, unsigned int num)
{
	u16 evt, old, new;
	int ret;

	/* memory barrier */
	virt_mb();

	evt = vring_used_event(&in->rx.vr);
	old = in->rx.last_used_idx - num;
	new = in->rx.last_used_idx;

	if (vring_need_event(evt, new, old)) {
		ret = mbox_send_message(in->tx_ch, &in->regs.lstate);
		if (ret < 0)
			dev_err(&in->pdev->dev, "%s send message error=%d!\n",
				__func__, ret);

		in->stats.rx_notify++;
	}
}

static void imx_shm_net_enable_tx_irq(struct imx_shm_net *in)
{
	vring_used_event(&in->tx.vr) = in->tx.last_used_idx;
	/* memory barrier */
	virt_wmb();
}

static bool imx_shm_net_rx_avail(struct imx_shm_net *in)
{
	/* memory barrier */
	virt_mb();
	return READ_ONCE(in->rx.vr.avail->idx) != in->rx.last_avail_idx;
}

static size_t imx_shm_net_tx_space(struct imx_shm_net *in)
{
	struct imx_shm_net_queue *tx = &in->tx;
	u32 tail = tx->tail;
	u32 head = tx->head;
	u32 space;

	if (head < tail)
		space = tail - head;
	else
		space = max(tx->size - head, tail);

	return space;
}

static bool imx_shm_net_tx_ok(struct imx_shm_net *in, unsigned int mtu)
{
	return in->tx.num_free >= 2 &&
		imx_shm_net_tx_space(in) >= 2 * IMX_SHM_NET_FRAME_SIZE(mtu);
}

static u32 imx_shm_net_tx_advance(struct imx_shm_net_queue *q, u32 *pos,
				  u32 len)
{
	u32 p = *pos;

	len = IMX_SHM_NET_FRAME_SIZE(len);

	if (q->size - p < len)
		p = 0;
	*pos = p + len;

	return p;
}

static int imx_shm_net_tx_frame(struct net_device *ndev, struct sk_buff *skb,
				bool xmit_more)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	struct imx_shm_net_queue *tx = &in->tx;
	struct vring *vr = &tx->vr;
	struct vring_desc *desc;
	unsigned int desc_idx;
	unsigned int avail;
	u32 head;
	void *buf;

	BUG_ON(tx->num_free < 1);

	spin_lock(&in->tx_free_lock);
	desc_idx = tx->free_head;
	desc = &vr->desc[desc_idx];
	tx->free_head = desc->next;
	tx->num_free--;
	spin_unlock(&in->tx_free_lock);

	head = imx_shm_net_tx_advance(tx, &tx->head, skb->len);

	buf = tx->data + head;
	skb_copy_and_csum_dev(skb, buf);

	desc->addr = buf - in->shm;
	desc->len = skb->len;
	desc->flags = 0;

	avail = tx->last_avail_idx++ & (vr->num - 1);
	vr->avail->ring[avail] = desc_idx;
	tx->num_added++;

	if (!xmit_more) {
		/* memory barrier */
		virt_store_release(&vr->avail->idx, tx->last_avail_idx);
		imx_shm_net_notify_tx(in, tx->num_added);
		tx->num_added = 0;
	}

	return 0;
}

static void imx_shm_net_tx_clean(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	struct imx_shm_net_queue *tx = &in->tx;
	struct vring_used_elem *used;
	struct vring *vr = &tx->vr;
	struct vring_desc *desc;
	struct vring_desc *fdesc;
	unsigned int num;
	u16 used_idx;
	u16 last;
	u32 fhead;

	if (!spin_trylock(&in->tx_clean_lock))
		return;

	/* memory barrier */
	used_idx = virt_load_acquire(&vr->used->idx);
	last = tx->last_used_idx;

	fdesc = NULL;
	fhead = 0;
	num = 0;

	while (last != used_idx) {
		void *data;
		u32 len;
		u32 tail;

		used = vr->used->ring + (last % vr->num);
		if (used->id >= vr->num || used->len != 1) {
			netdev_err(ndev, "invalid tx used->id %d ->len %d\n",
				   used->id, used->len);
			break;
		}

		desc = &vr->desc[used->id];

		data = imx_shm_net_desc_data(in, &in->tx, desc, &len);
		if (!data) {
			netdev_err(ndev, "bad tx descriptor, data == NULL\n");
			break;
		}

		tail = imx_shm_net_tx_advance(tx, &tx->tail, len);
		if (data != tx->data + tail) {
			netdev_err(ndev, "bad tx descriptor\n");
			break;
		}

		if (!num)
			fdesc = desc;
		else
			desc->next = fhead;

		fhead = used->id;
		last++;
		num++;
	}

	tx->last_used_idx = last;

	spin_unlock(&in->tx_clean_lock);

	if (num) {
		spin_lock(&in->tx_free_lock);
		fdesc->next = tx->free_head;
		tx->free_head = fhead;
		tx->num_free += num;
		BUG_ON(tx->num_free > vr->num);
		spin_unlock(&in->tx_free_lock);
	}
}

static struct vring_desc *imx_shm_net_rx_desc(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	struct imx_shm_net_queue *rx = &in->rx;
	struct vring *vr = &rx->vr;
	unsigned int avail;
	u16 avail_idx;

	/* memory barrier */
	avail_idx = virt_load_acquire(&vr->avail->idx);

	if (avail_idx == rx->last_avail_idx)
		return NULL;

	avail = vr->avail->ring[rx->last_avail_idx++ & (vr->num - 1)];
	if (avail >= vr->num) {
		netdev_err(ndev, "invalid rx avail %d\n", avail);
		return NULL;
	}

	return &vr->desc[avail];
}

static void imx_shm_net_rx_finish(struct imx_shm_net *in,
				  struct vring_desc *desc)
{
	struct imx_shm_net_queue *rx = &in->rx;
	struct vring *vr = &rx->vr;
	unsigned int desc_id = desc - vr->desc;
	unsigned int used;

	used = rx->last_used_idx++ & (vr->num - 1);
	vr->used->ring[used].id = desc_id;
	vr->used->ring[used].len = 1;

	/* memory barrier */
	virt_store_release(&vr->used->idx, rx->last_used_idx);
}

static int imx_shm_net_poll(struct napi_struct *napi, int budget)
{
	struct net_device *ndev = napi->dev;
	struct imx_shm_net *in = container_of(napi, struct imx_shm_net, napi);
	int received = 0;

	in->stats.napi_poll++;

	imx_shm_net_tx_clean(ndev);

	while (received < budget) {
		struct vring_desc *desc;
		struct sk_buff *skb;
		void *data;
		u32 len;

		desc = imx_shm_net_rx_desc(ndev);
		if (!desc)
			break;

		data = imx_shm_net_desc_data(in, &in->rx, desc, &len);
		if (!data) {
			netdev_err(ndev, "bad rx descriptor\n");
			break;
		}

		skb = napi_alloc_skb(napi, len);

		if (skb) {
			memcpy(skb_put(skb, len), data, len);
			skb->protocol = eth_type_trans(skb, ndev);
			napi_gro_receive(napi, skb);
		}

		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += len;

		imx_shm_net_rx_finish(in, desc);
		received++;
	}

	if (received < budget) {
		in->stats.napi_complete++;
		napi_complete_done(napi, received);
		imx_shm_net_enable_rx_irq(in);
		if (imx_shm_net_rx_avail(in))
			napi_schedule(napi);
	}

	if (received)
		imx_shm_net_notify_rx(in, received);

	in->stats.rx_packets += received;
	in->stats.napi_poll_n[received ? 1 + min(ilog2(received), 8) : 0]++;

	if (imx_shm_net_tx_ok(in, ndev->mtu))
		netif_wake_queue(ndev);

	return received;
}

static netdev_tx_t imx_shm_net_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	bool xmit_more = netdev_xmit_more();

	imx_shm_net_tx_clean(ndev);

	if (!imx_shm_net_tx_ok(in, ndev->mtu)) {
		imx_shm_net_enable_tx_irq(in);
		netif_stop_queue(ndev);
		xmit_more = false;
		in->stats.tx_pause++;
	}

	imx_shm_net_tx_frame(ndev, skb, xmit_more);

	in->stats.tx_packets++;
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	dev_consume_skb_any(skb);

	return NETDEV_TX_OK;
}

static void imx_shm_net_set_state(struct imx_shm_net *in, u32 state)
{
	int ret;

	/* memory barrier */
	virt_wmb();
	WRITE_ONCE(in->regs.lstate, state);
	dev_dbg(&in->pdev->dev, "%s %s\n", __func__, state_name[state]);
	ret = mbox_send_message(in->tx_ch, &state);
	if (ret < 0)
		dev_err(&in->pdev->dev, "%s send message error=%d!\n",
			__func__, ret);
}

static void imx_shm_net_run(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);

	if (in->regs.lstate < IMX_SHM_NET_STATE_READY)
		return;

	if (!netif_running(ndev))
		return;

	if (test_and_set_bit(IMX_SHM_NET_FLAG_RUN, &in->flags))
		return;

	netif_start_queue(ndev);
	napi_enable(&in->napi);
	napi_schedule(&in->napi);
	imx_shm_net_set_state(in, IMX_SHM_NET_STATE_RUN);
}

static void imx_shm_net_do_stop(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);

	imx_shm_net_set_state(in, IMX_SHM_NET_STATE_RESET);

	if (!test_and_clear_bit(IMX_SHM_NET_FLAG_RUN, &in->flags))
		return;

	netif_stop_queue(ndev);
	napi_disable(&in->napi);
}

static void imx_shm_net_state_change(struct work_struct *work)
{
	struct imx_shm_net *in = container_of(work, struct imx_shm_net,
			state_work);
	struct net_device *ndev = in->napi.dev;
	u32 rstate;

	mutex_lock(&in->state_lock);
	rstate = in->remote_message;
	if (in->message_state == MESS_STATE_PROCESSED)
		dev_dbg(&in->pdev->dev,
			"imx-shmem-net: ERROR message already processed!\n");

	in->message_state = MESS_STATE_PROCESSED;
	mutex_unlock(&in->state_lock);

	dev_dbg(&in->pdev->dev, "%s: rstate=%s lstate=%s\n",
		__func__, state_name[rstate], state_name[in->regs.lstate]);
	if (rstate != in->regs.rstate)
		dev_dbg(&in->pdev->dev, "rstate changed from %s to %s\n",
			state_name[in->regs.rstate], state_name[rstate]);

	switch (in->regs.lstate) {
	case IMX_SHM_NET_STATE_RESET:
		/* Wait for the remote to leave READY/RUN */
		/* before transitioning to INIT. */
		if (rstate < IMX_SHM_NET_STATE_READY)
			imx_shm_net_set_state(in, IMX_SHM_NET_STATE_INIT);
		break;

	case IMX_SHM_NET_STATE_INIT:
		/* Wait for the remote to leave RESET before performing the */
		/* initialization and moving to READY. */
		if (rstate > IMX_SHM_NET_STATE_RESET) {
			imx_shm_net_init_queues(ndev);
			imx_shm_net_set_state(in, IMX_SHM_NET_STATE_READY);

			rtnl_lock();
			call_netdevice_notifiers(NETDEV_CHANGEADDR, ndev);
			rtnl_unlock();
		}
		break;

	case IMX_SHM_NET_STATE_READY:
		/* Link is up and we are running */
		/* once the remote is in READY or RUN. */
		if (rstate >= IMX_SHM_NET_STATE_READY) {
			netif_carrier_on(ndev);
			imx_shm_net_run(ndev);
			break;
		}
		fallthrough;
	case IMX_SHM_NET_STATE_RUN:
		/* If the remote goes to RESET, */
		/* we need to follow immediately. */
		if (rstate == IMX_SHM_NET_STATE_RESET) {
			netif_carrier_off(ndev);
			imx_shm_net_do_stop(ndev);
		}
		break;
	}

	/* memory barrier */
	virt_wmb();
	WRITE_ONCE(in->regs.rstate, rstate);
}

static void imx_shm_net_check_state(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	u32 rstate;

	/* for now message carries only the rstate value */
	rstate = in->remote_message;

	if (rstate != in->regs.rstate ||
	    !test_bit(IMX_SHM_NET_FLAG_RUN, &in->flags)) {
		dev_dbg(&ndev->dev, "rstate changed -> queue a work to handle it\n");
		queue_work(in->state_wq, &in->state_work);
	} else {
		in->message_state = MESS_STATE_PROCESSED;
	}
}

static int imx_shm_net_open(struct net_device *ndev)
{
	dev_dbg(&ndev->dev, "calling %s()\n", __func__);
	netdev_reset_queue(ndev);
	ndev->operstate = IF_OPER_UP;
	imx_shm_net_run(ndev);

	return 0;
}

static int imx_shm_net_stop(struct net_device *ndev)
{
	ndev->operstate = IF_OPER_DOWN;
	imx_shm_net_do_stop(ndev);

	return 0;
}

static int imx_shm_net_change_mtu(struct net_device *ndev, int mtu)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	struct imx_shm_net_queue *tx = &in->tx;

	if (mtu < IMX_SHM_NET_MTU_MIN || mtu > IMX_SHM_NET_MTU_MAX)
		return -EINVAL;

	if (in->tx.size / mtu < 4)
		return -EINVAL;

	if (imx_shm_net_tx_space(in) < 2 * IMX_SHM_NET_FRAME_SIZE(mtu))
		return -EBUSY;

	if (in->tx.size - tx->head < IMX_SHM_NET_FRAME_SIZE(mtu) &&
	    tx->head < tx->tail)
		return -EBUSY;

	netif_tx_lock_bh(ndev);
	if (in->tx.size - tx->head < IMX_SHM_NET_FRAME_SIZE(mtu))
		tx->head = 0;
	netif_tx_unlock_bh(ndev);

	ndev->mtu = mtu;

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void imx_shm_net_poll_controller(struct net_device *ndev)
{
	struct imx_shm_net *in = netdev_priv(ndev);

	napi_schedule(&in->napi);
}
#endif

static const struct net_device_ops imx_shm_net_ops = {
		.ndo_open		= imx_shm_net_open,
		.ndo_stop		= imx_shm_net_stop,
		.ndo_start_xmit		= imx_shm_net_xmit,
		.ndo_change_mtu		= imx_shm_net_change_mtu,
		.ndo_set_mac_address	= eth_mac_addr,
		.ndo_validate_addr	= eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
		.ndo_poll_controller	= imx_shm_net_poll_controller,
#endif
};

static const char imx_shm_net_stats[][ETH_GSTRING_LEN] = {
		"interrupts",
		"tx_packets",
		"tx_notify",
		"tx_pause",
		"rx_packets",
		"rx_notify",
		"napi_poll",
		"napi_complete",
		"napi_poll_0",
		"napi_poll_1",
		"napi_poll_2",
		"napi_poll_4",
		"napi_poll_8",
		"napi_poll_16",
		"napi_poll_32",
		"napi_poll_64",
		"napi_poll_128",
		"napi_poll_256",
};

#define NUM_STATS ARRAY_SIZE(imx_shm_net_stats)

static int imx_shm_net_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset == ETH_SS_STATS)
		return NUM_STATS;

	return -EOPNOTSUPP;
}

static void imx_shm_net_get_strings(struct net_device *ndev, u32 sset, u8 *buf)
{
	if (sset == ETH_SS_STATS)
		memcpy(buf, &imx_shm_net_stats, sizeof(imx_shm_net_stats));
}

static void imx_shm_net_get_ethtool_stats(struct net_device *ndev,
					  struct ethtool_stats *estats, u64 *st)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	unsigned int n = 0;
	unsigned int i;

	st[n++] = in->stats.interrupts;
	st[n++] = in->stats.tx_packets;
	st[n++] = in->stats.tx_notify;
	st[n++] = in->stats.tx_pause;
	st[n++] = in->stats.rx_packets;
	st[n++] = in->stats.rx_notify;
	st[n++] = in->stats.napi_poll;
	st[n++] = in->stats.napi_complete;

	for (i = 0; i < ARRAY_SIZE(in->stats.napi_poll_n); i++)
		st[n++] = in->stats.napi_poll_n[i];

	memset(&in->stats, 0, sizeof(in->stats));
}

#define IMX_SHM_NET_REGS_LEN	(3 * sizeof(u32) + 6 * sizeof(u16))

static int imx_shm_net_get_regs_len(struct net_device *ndev)
{
	return IMX_SHM_NET_REGS_LEN;
}

static void imx_shm_net_get_regs(struct net_device *ndev,
				 struct ethtool_regs *regs, void *p)
{
	struct imx_shm_net *in = netdev_priv(ndev);
	u32 *reg32 = p;
	u16 *reg16;

	*reg32++ = in->regs.lstate;
	*reg32++ = in->regs.rstate;
	*reg32++ = in->qlen;

	reg16 = (u16 *)reg32;

	*reg16++ = in->tx.vr.avail ? in->tx.vr.avail->idx : 0;
	*reg16++ = in->tx.vr.used ? in->tx.vr.used->idx : 0;
	*reg16++ = in->tx.vr.avail ? vring_avail_event(&in->tx.vr) : 0;

	*reg16++ = in->rx.vr.avail ? in->rx.vr.avail->idx : 0;
	*reg16++ = in->rx.vr.used ? in->rx.vr.used->idx : 0;
	*reg16++ = in->rx.vr.avail ? vring_avail_event(&in->rx.vr) : 0;
}

static int imx_shm_partition_notify(struct notifier_block *nb,
				    unsigned long event, void *group)
{
	struct imx_shm_net *in = NULL;
	struct list_head *pos;
	struct net_device *ndev;

	/* Ignore other irqs */
	if (*(u8 *)group != SC_IRQ_GROUP_REBOOTED)
		return 0;

	pr_debug("%s: Partition reset detected!\n", DRV_NAME);

	/* browse all imx_shm_net devices */
	list_for_each(pos, &imx_shm_net_head) {
		in = list_entry(pos, struct imx_shm_net, isn_node);

		if (event & BIT(in->mub_partition)) {
			mutex_lock(&in->state_lock);
			in->remote_message = IMX_SHM_NET_STATE_RESET;
			in->message_state = MESS_STATE_NEW;
			mutex_unlock(&in->state_lock);

			ndev = platform_get_drvdata(in->pdev);

			imx_shm_net_check_state(ndev);
			napi_schedule_irqoff(&in->napi);

			dev_info(&in->pdev->dev, "Partition %d reset!\n",
				 in->mub_partition);
		}
	}

	return 0;
}

static void imx_shm_rx_callback(struct mbox_client *c, void *msg)
{
	u32 *data = msg;
	struct imx_shm_net *isndev = container_of(c,
			struct imx_shm_net, cl);
	struct net_device *ndev = platform_get_drvdata(isndev->pdev);

	isndev->stats.interrupts++;

	/* get message from receive buffer */
	mutex_lock(&isndev->state_lock);
	isndev->remote_message = *data;
	if (isndev->message_state == MESS_STATE_NEW)
		dev_dbg(&isndev->pdev->dev, "RX message overwritten while not yet processed!");
	isndev->message_state = MESS_STATE_NEW;

	imx_shm_net_check_state(ndev);
	mutex_unlock(&isndev->state_lock);

	napi_schedule_irqoff(&isndev->napi);
}

static int imx_shm_xtr_channel_init(struct imx_shm_net *isndev)
{
	struct platform_device *pdev = isndev->pdev;
	struct device *dev = &pdev->dev;
	struct mbox_client *cl;
	int ret = 0;

	cl = &isndev->cl;
	cl->dev = dev;
	cl->tx_block = false;
	cl->tx_tout = IMX_SHM_NET_MU_TIMEOUT;
	cl->knows_txdone = false;
	cl->rx_callback = imx_shm_rx_callback;

	/* if channels are not initialized yet, do it */
	if (!isndev->tx_ch || (IS_ERR(isndev->tx_ch)))
		isndev->tx_ch = mbox_request_channel_byname(cl, "tx");

	if (IS_ERR(isndev->tx_ch)) {
		ret = PTR_ERR(isndev->tx_ch);
		dev_info(cl->dev, "failed to request mbox tx chan, ret %d\n",
			 ret);
		goto err_out;
	}
	if (!isndev->rx_ch || (IS_ERR(isndev->rx_ch)))
		isndev->rx_ch = mbox_request_channel_byname(cl, "rx");

	if (IS_ERR(isndev->rx_ch)) {
		ret = PTR_ERR(isndev->rx_ch);
		dev_info(cl->dev, "failed to request mbox rx chan, ret %d\n",
			 ret);
		goto err_out;
	}

	return ret;

err_out:
	if (!IS_ERR(isndev->tx_ch))
		mbox_free_channel(isndev->tx_ch);
	if (!IS_ERR(isndev->rx_ch))
		mbox_free_channel(isndev->rx_ch);

	return ret;
}

/* enable reset notification MU */
static int mu_enable_reset_irq(struct net_device *ndev)
{
	int sciErr;
	struct imx_shm_net *isndev;
	struct device_node *np;

	isndev = netdev_priv(ndev);
	np = isndev->pdev->dev.of_node;

	/* Get muB partition id and enable irq in SCFW then */
	if (of_property_read_u32(np, "mub-partition",
				 &isndev->mub_partition))
		isndev->mub_partition = IMX_SHM_NET_DEFAULT_PART;
	dev_dbg(&isndev->pdev->dev, "watching reset from partition %d\n",
		isndev->mub_partition);

#ifdef CONFIG_IMX_SCU
	/* Request for the partition reset interrupt. */
	sciErr = imx_scu_irq_group_enable(SC_IRQ_GROUP_REBOOTED,
					  BIT(isndev->mub_partition), true);
	if (sciErr)
		dev_warn(&isndev->pdev->dev, "Cannot request partition reset interrupt\n");

	isndev->pnotifier.notifier_call = imx_shm_partition_notify;
	sciErr = imx_scu_irq_register_notifier(&isndev->pnotifier);
	if (sciErr) {
		imx_scu_irq_group_enable(SC_IRQ_GROUP_REBOOTED,
					 BIT(isndev->mub_partition), false);
		dev_warn(&isndev->pdev->dev, "Failed to register partition reset notifier\n");
	}
#endif
	return 0;
}

static const struct ethtool_ops imx_shm_net_ethtool_ops = {
		.get_sset_count		= imx_shm_net_get_sset_count,
		.get_strings		= imx_shm_net_get_strings,
		.get_ethtool_stats	= imx_shm_net_get_ethtool_stats,
		.get_regs_len		= imx_shm_net_get_regs_len,
		.get_regs		= imx_shm_net_get_regs,
};

static int imx_shm_net_probe(struct platform_device *pdev)
{
	struct net_device *ndev = NULL;
	struct imx_shm_net *in;
	resource_size_t shmaddr;
	resource_size_t shmlen;
	char *device_name;
	void *shm = NULL;
	u32 ivpos;
	int ret;

	/* check if 1st probe or another attempt after EAGAIN */
	if (pdev->dev.driver_data) {
		dev_dbg(&pdev->dev, "Retrying connection...\n");
		in = netdev_priv(platform_get_drvdata(pdev));
		goto retry;
	}

	if (of_property_read_bool(pdev->dev.of_node, "rxfirst")) {
		ivpos = 1;
		dev_info(&pdev->dev, "queue position is RX first\n");
	} else {
		ivpos = 0;
		dev_info(&pdev->dev, "queue position is TX first\n");
	}

	/* get shared coherent memory for buffers */
	if (of_reserved_mem_device_init(&pdev->dev)) {
		dev_err(&pdev->dev,
			"dev doesn't have specific DMA pool.\n");
		return -ENOMEM;
	}
	shmlen = IMX_SHM_NET_DMA_SIZE;
	shm = dma_alloc_coherent(&pdev->dev, IMX_SHM_NET_DMA_SIZE,
				 &shmaddr, GFP_KERNEL);
	if (!shm || !shmaddr)
		return -ENOMEM;

	dev_info(&pdev->dev, "allocated %d bytes in coherent mem @ 0x%x\n",
		 (uint)shmlen, (uint)shmaddr);

	device_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s[%s]", DRV_NAME,
				     dev_name(&pdev->dev));
	if (!device_name) {
		ret = -ENOMEM;
		goto err_free_dma;
	}

	ndev = alloc_etherdev(sizeof(*in));
	if (!ndev) {
		ret = -ENOMEM;
		goto err_free_dma;
	}
	dev_info(&pdev->dev, "allocated ethernet device %s\n", ndev->name);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	/* get struct 'imx_shm_net' stored as private data in ndev */
	in = netdev_priv(ndev);

	list_add(&in->isn_node, &imx_shm_net_head);

	in->shm = shm;
	in->shmaddr = shmaddr;
	in->shmlen = shmlen;
	in->pdev = pdev;
	in->regs.ivpos = ivpos;
	in->regs.rstate = IMX_SHM_NET_STATE_RESET;
	in->remote_message = IMX_SHM_NET_STATE_RESET;
	spin_lock_init(&in->tx_free_lock);
	spin_lock_init(&in->tx_clean_lock);
	mutex_init(&in->state_lock);

	ret = imx_shm_net_calc_qsize(ndev);
	if (ret)
		goto err_free;

	in->state_wq = alloc_ordered_workqueue(device_name, 0);
	if (!in->state_wq) {
		ret = -ENOMEM;
		goto err_free;
	}

	INIT_WORK(&in->state_work, imx_shm_net_state_change);

	eth_random_addr(ndev->dev_addr);
	ndev->netdev_ops = &imx_shm_net_ops;
	ndev->ethtool_ops = &imx_shm_net_ethtool_ops;
	ndev->mtu = min_t(u32, IMX_SHM_NET_MTU_DEF, in->qsize / 16);
	ndev->hw_features = NETIF_F_HW_CSUM | NETIF_F_SG;
	ndev->features = ndev->hw_features;

	netif_carrier_off(ndev);
	netif_napi_add(ndev, &in->napi, imx_shm_net_poll);

	ret = register_netdev(ndev);
	if (ret)
		goto err_wq;

	/* initialize Mailbox for RX/TX */
	ret = imx_shm_xtr_channel_init(in);
	if (ret) {
		dev_err(&in->pdev->dev, "unable to initialize Mailbox.\n");
		/* MU may not be ready yet, need to try later on */
		ret = -EPROBE_DEFER;
		goto err_unregister;
	}

	/* enable peer's reset notification */
	ret = mu_enable_reset_irq(ndev);
	if (ret)
		goto err_reset_irq;

	dev_info(&in->pdev->dev,
		 "Mailbox is ready for cross core communication!\n");

retry:
	/* notify reset */
	mutex_lock(&in->state_lock);
	in->regs.lstate = IMX_SHM_NET_STATE_RESET;
	in->message_state = MESS_STATE_NEW;
	mutex_unlock(&in->state_lock);

	/* only device with queue position TXfirst sends the first message */
	if (!ivpos) {
		ret = mbox_send_message(in->tx_ch, &in->regs.lstate);
		if (ret < 0)
			dev_err(&pdev->dev, "%s first message error=%d!\n",
				__func__, ret);

		dev_dbg(&pdev->dev, "%s sent first message\n", __func__);
	}

	return 0;

err_reset_irq:
#ifdef CONFIG_IMX_SCU
	imx_scu_irq_unregister_notifier(&in->pnotifier);
#endif

err_unregister:
	unregister_netdev(ndev);

err_wq:
	destroy_workqueue(in->state_wq);

err_free:
	list_del(&in->isn_node);
	free_netdev(ndev);

err_free_dma:
	dma_free_coherent(&pdev->dev, shmlen, shm, shmaddr);

	return ret;
}

static int imx_shm_net_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct imx_shm_net *in = netdev_priv(ndev);
	int ret;

	/* notify reset */
	in->regs.lstate = IMX_SHM_NET_STATE_RESET;
	ret = mbox_send_message(in->tx_ch, &in->regs.lstate);
	if (ret < 0)
		dev_err(&in->pdev->dev, "%s send message error=%d!\n",
			__func__, ret);

	if (!IS_ERR(in->tx_ch))
		mbox_free_channel(in->tx_ch);
	if (!IS_ERR(in->rx_ch))
		mbox_free_channel(in->rx_ch);

	imx_scu_irq_unregister_notifier(&in->pnotifier);

	dma_free_coherent(&pdev->dev, in->shmlen, in->shm, in->shmaddr);

	/* remove imx_shm_net's node from list */
	list_del(&in->isn_node);

	unregister_netdev(ndev);
	cancel_work_sync(&in->state_work);
	destroy_workqueue(in->state_wq);
	free_netdev(ndev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_shm_net_pm_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct imx_shm_net *in = netdev_priv(ndev);

	dev_info(&in->pdev->dev, "entering %s\n", __func__);

	return 0;
}

static int imx_shm_net_pm_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct imx_shm_net *in = netdev_priv(ndev);

	dev_info(&in->pdev->dev, "entering %s\n", __func__);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id of_isn_id[] = {
	{ .compatible = "fsl,imx-shmem-net", },
	{}
};
MODULE_DEVICE_TABLE(of, of_isn_id);

static const struct dev_pm_ops imx_shm_net_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(imx_shm_net_pm_suspend,
				      imx_shm_net_pm_resume)
};

static struct platform_driver imx_shm_net_driver = {
	.probe		= imx_shm_net_probe,
	.remove		= imx_shm_net_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &imx_shm_net_pm_ops,
		.of_match_table = of_isn_id,
	},
};

static int __init imx_shm_net_init(void)
{
	int ret;

	INIT_LIST_HEAD(&imx_shm_net_head);

	/* Add the device to the platform. */
	ret = platform_driver_register(&imx_shm_net_driver);

	return ret;
}

static void __exit imx_shm_net_cleanup(void)
{
	platform_driver_unregister(&imx_shm_net_driver);
}

module_init(imx_shm_net_init);

module_exit(imx_shm_net_cleanup);

MODULE_AUTHOR("Sebastien Fagard <sebastien.fagard@nxp.com>");
MODULE_LICENSE("GPL");
