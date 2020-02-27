/* Copyright (c) 2008-2011 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 @File          fman_test.c
 @Authors       Pistirica Sorin Andrei
 @Description   FM Linux test environment
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_platform.h>
#include <linux/ip.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/fsl_qman.h>
#include <linux/fsl_bman.h>

/* private headers */
#include "fm_ext.h"
#include "lnxwrp_fsl_fman.h"
#include "fm_port_ext.h"
#if (DPAA_VERSION == 11)
#include "../../Peripherals/FM/MAC/memac.h"
#endif
#include "fm_test_ioctls.h"
#include "fsl_fman_test.h"

#include "dpaa_eth.h"
#include "dpaa_eth_common.h"

#define FMT_FRM_WATERMARK   0xdeadbeefdeadbeeaLL

struct fmt_frame_s {
	ioc_fmt_buff_desc_t	buff;
	struct list_head	list;
};

struct fmt_fqs_s {
	struct qman_fq		fq_base;
	bool			init;
	struct fmt_port_s	*fmt_port_priv;
};

struct fmt_port_pcd_s {
	int		 num_queues;
	struct fmt_fqs_s *fmt_pcd_fqs;
	uint32_t	 fqid_base;
};

/* char dev structure: fm test port */
struct fmt_port_s {
	bool                valid;
	uint8_t             id;
	ioc_fmt_port_type   port_type;
	ioc_diag_mode       diag;
	bool		    compat_test_type;

	/* fm ports */
	/* ! for oh ports p_tx_fm_port_dev == p_rx_fm_port_dev &&
	 * p_tx_port == p_rx_port */
				/* t_LnxWrpFmPortDev */
	struct fm_port      *p_tx_port;
				/* t_LnxWrpFmPortDev->h_Dev: t_FmPort */
	void                *p_tx_fm_port_dev;
				/* t_LnxWrpFmPortDev */
	struct fm_port      *p_rx_port;
				/* t_LnxWrpFmPortDev->h_Dev: t_FmPort */
	void                *p_rx_fm_port_dev;

	void                *p_mac_dev;
	uint64_t            fm_phys_base_addr;

	/* read/write queue manipulation */
	spinlock_t          rx_q_lock;
	struct list_head    rx_q;

	/* tx queuee for injecting traffic */
	int                 num_of_tx_fqs;
	struct fmt_fqs_s    p_tx_fqs[FMAN_TEST_MAX_TX_FQS];

	/* pcd private queues manipulation */
	struct fmt_port_pcd_s fmt_port_pcd;

	/* debugging stuff */

#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	atomic_t enqueue_to_qman_frm;
	atomic_t enqueue_to_rxq;
	atomic_t dequeue_from_rxq;
	atomic_t not_enqueue_to_rxq_wrong_frm;
#endif

};

/* The devices. */
struct fmt_s {
	int major;
	struct fmt_port_s ports[IOC_FMT_MAX_NUM_OF_PORTS];
	struct class *fmt_class;
};

/* fm test structure */
static struct fmt_s fm_test;

#if (DPAA_VERSION == 11)
struct mac_priv_s {
        t_Handle        mac;
};
#endif

#define DTSEC_BASE_ADDR         0x000e0000
#define DTSEC_MEM_RANGE         0x00002000
#define MAC_1G_MACCFG1          0x00000100
#define MAC_1G_LOOP_MASK        0x00000100
static int set_1gmac_loopback(
		struct fmt_port_s *fmt_port,
		bool en)
{
#if (DPAA_VERSION <= 10)
	uint32_t dtsec_idx = fmt_port->id; /* dtsec for which port */
	uint32_t dtsec_idx_off = dtsec_idx * DTSEC_MEM_RANGE;
	phys_addr_t maccfg1_hw;
	void *maccfg1_map;
	uint32_t maccfg1_val;

	/* compute the maccfg1 register address */
	maccfg1_hw = fmt_port->fm_phys_base_addr +
			(phys_addr_t)(DTSEC_BASE_ADDR +
					dtsec_idx_off +
					MAC_1G_MACCFG1);

	/* map register */
	maccfg1_map = ioremap(maccfg1_hw, sizeof(u32));

	/* set register */
	maccfg1_val = in_be32(maccfg1_map);
	if (en)
		maccfg1_val |= MAC_1G_LOOP_MASK;
	else
		maccfg1_val &= ~MAC_1G_LOOP_MASK;
	out_be32(maccfg1_map, maccfg1_val);

	/* unmap register */
	iounmap(maccfg1_map);
#else
	struct mac_device *mac_dev;
	struct mac_priv_s *priv;
	t_Memac *p_memac;

	if (!fmt_port)
		return -EINVAL;

	mac_dev = (struct mac_device *)fmt_port->p_mac_dev;

	if (!mac_dev)
		return -EINVAL;

	priv = macdev_priv(mac_dev);

	if (!priv)
		return -EINVAL;

	p_memac = priv->mac;

	if (!p_memac)
		return -EINVAL;

	memac_set_loopback(p_memac->p_MemMap, en);
#endif
	return 0;
}

/* TODO: re-write this function */
static int set_10gmac_int_loopback(
		struct fmt_port_s *fmt_port,
		bool en)
{
#ifndef FM_10G_MAC_NO_CTRL_LOOPBACK
#define FM_10GMAC0_OFFSET               0x000f0000
#define FM_10GMAC_CMD_CONF_CTRL_OFFSET  0x8
#define CMD_CFG_LOOPBACK_EN             0x00000400

	uint64_t    base_addr, reg_addr;
	uint32_t    tmp_val;

	base_addr = fmt_port->fm_phys_base_addr + (FM_10GMAC0_OFFSET +
			((fmt_port->id-FM_MAX_NUM_OF_1G_RX_PORTS)*0x2000));

	base_addr = PTR_TO_UINT(ioremap(base_addr, 0x1000));

	reg_addr = base_addr + FM_10GMAC_CMD_CONF_CTRL_OFFSET;
	tmp_val = GET_UINT32(*((uint32_t  *)UINT_TO_PTR(reg_addr)));
	if (en)
		tmp_val |= CMD_CFG_LOOPBACK_EN;
	else
		tmp_val &= ~CMD_CFG_LOOPBACK_EN;
	WRITE_UINT32(*((uint32_t  *)UINT_TO_PTR(reg_addr)), tmp_val);

	iounmap(UINT_TO_PTR(base_addr));

	return 0;
#else
	_fmt_err("TGEC don't have internal-loopback.\n");
	return  -EPERM;
#endif
}

static int set_mac_int_loopback(struct fmt_port_s *fmt_port, bool en)
{
	int _err = 0;

	switch (fmt_port->port_type) {

	case e_IOC_FMT_PORT_T_RXTX:
	/* 1G port */
	if (fmt_port->id < FM_MAX_NUM_OF_1G_RX_PORTS)
		_err = set_1gmac_loopback(fmt_port, en);
	/* 10g port */
	else if ((fmt_port->id >= FM_MAX_NUM_OF_1G_RX_PORTS) &&
			(fmt_port->id < FM_MAX_NUM_OF_1G_RX_PORTS +
					FM_MAX_NUM_OF_10G_RX_PORTS)) {

		_err = set_10gmac_int_loopback(fmt_port, en);
	} else
		_err = -EINVAL;
	break;
	/* op port does not have MAC (loopback mode) */
	case e_IOC_FMT_PORT_T_OP:

	_err = 0;
	break;
	default:

	_err = -EPERM;
	break;
	}

	return _err;
}

static void enqueue_fmt_frame(
		struct fmt_port_s *fmt_port,
		struct fmt_frame_s *p_fmt_frame)
{
	spinlock_t *rx_q_lock = NULL;

	rx_q_lock = &fmt_port->rx_q_lock;

	spin_lock(rx_q_lock);
	list_add_tail(&p_fmt_frame->list, &fmt_port->rx_q);
	spin_unlock(rx_q_lock);

#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	atomic_inc(&fmt_port->enqueue_to_rxq);
#endif
}

static struct fmt_frame_s *dequeue_fmt_frame(
		struct fmt_port_s *fmt_port)
{
	struct fmt_frame_s *p_fmt_frame = NULL;
	spinlock_t *rx_q_lock = NULL;

	rx_q_lock = &fmt_port->rx_q_lock;

	spin_lock(rx_q_lock);

#define list_last_entry(ptr, type, member) list_entry((ptr)->prev, type, member)

	if (!list_empty(&fmt_port->rx_q)) {
		p_fmt_frame = list_last_entry(&fmt_port->rx_q,
						struct fmt_frame_s,
						list);
		list_del(&p_fmt_frame->list);

#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
		atomic_inc(&fmt_port->dequeue_from_rxq);
#endif
	}

	spin_unlock(rx_q_lock);

	return p_fmt_frame;
}

/* eth-dev -to- fmt port association */
struct fmt_port_s *match_dpa_to_fmt_port(
				struct dpa_priv_s *dpa_priv) {
	struct mac_device *mac_dev = dpa_priv->mac_dev;
	struct fm_port *fm_port = (struct fm_port  *) mac_dev;
	struct fmt_port_s *fmt_port = NULL;
	int i;

	_fmt_dbgr("calling...\n");

	/* find the FM-test-port object */
	for (i = 0; i < IOC_FMT_MAX_NUM_OF_PORTS; i++)
		if ((fm_test.ports[i].p_mac_dev &&
		     mac_dev == fm_test.ports[i].p_mac_dev) ||
		     fm_port == fm_test.ports[i].p_tx_port) {

			fmt_port = &fm_test.ports[i];
			break;
		}

	_fmt_dbgr("called\n");
	return fmt_port;
}

void dump_frame(
	uint8_t  *buffer,
	uint32_t size)
{
#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (i%16 == 0)
			printk(KERN_DEBUG "\n");
		printk(KERN_DEBUG "%2x ", *(buffer+i));
	}
#endif
	return;
}

bool test_and_steal_frame(struct fmt_port_s *fmt_port,
		uint32_t fqid,
		uint8_t  *buffer,
		uint32_t size)
{
	struct fmt_frame_s *p_fmt_frame = NULL;
	bool test_and_steal_frame_frame;
	uint32_t data_offset;
	uint32_t i;

	_fmt_dbgr("calling...\n");

	if (!fmt_port || !fmt_port->p_rx_fm_port_dev)
		return false;

	/* check watermark */
	test_and_steal_frame_frame = false;
	for (i = 0; i < size; i++) {
		uint64_t temp = *((uint64_t  *)(buffer + i));

		if (temp == (uint64_t) FMT_FRM_WATERMARK) {
			_fmt_dbgr("watermark found!\n");
			test_and_steal_frame_frame = true;
			break;
		}
	}

	if (!test_and_steal_frame_frame) {
#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
		atomic_inc(&fmt_port->not_enqueue_to_rxq_wrong_frm);
#endif
		_fmt_dbgr("NOT watermark found!\n");
		return false;
	}

	/* do not enqueue the tx conf/err frames */
	if ((fqid == FMT_TX_CONF_Q) || (fqid == FMT_TX_ERR_Q))
		goto _test_and_steal_frame_return_true;

	_fmt_dbgr("on port %d got FMUC frame\n", fmt_port->id);
	data_offset = FM_PORT_GetBufferDataOffset(
					fmt_port->p_rx_fm_port_dev);

	p_fmt_frame = kmalloc(sizeof(struct fmt_frame_s), GFP_KERNEL);

	/* dump frame... no more space left on device */
	if (p_fmt_frame == NULL) {
		_fmt_err("no space left on device!\n");
		goto _test_and_steal_frame_return_true;
	}

	memset(p_fmt_frame, 0, sizeof(struct fmt_frame_s));
	p_fmt_frame->buff.p_data = kmalloc(size * sizeof(uint8_t), GFP_KERNEL);

	/* No more space left on device*/
	if (p_fmt_frame->buff.p_data == NULL) {
		_fmt_err("no space left on device!\n");
		kfree(p_fmt_frame);
		goto _test_and_steal_frame_return_true;
	}

	p_fmt_frame->buff.size = size-data_offset;
	p_fmt_frame->buff.qid = fqid;

	memcpy(p_fmt_frame->buff.p_data,
		(uint8_t  *)PTR_MOVE(buffer, data_offset),
		p_fmt_frame->buff.size);

	memcpy(p_fmt_frame->buff.buff_context.fm_prs_res,
		FM_PORT_GetBufferPrsResult(fmt_port->p_rx_fm_port_dev,
						(char *)buffer),
		32);

	/* enqueue frame - this frame will go to us */
	enqueue_fmt_frame(fmt_port, p_fmt_frame);

_test_and_steal_frame_return_true:
	return true;
}

static int fmt_fq_release(const struct qm_fd *fd)
{
	struct dpa_bp *_dpa_bp;
	struct bm_buffer _bmb;

	if (fd->format == qm_fd_contig) {
		_dpa_bp = dpa_bpid2pool(fd->bpid);
		BUG_ON(IS_ERR(_dpa_bp));

		_bmb.hi	= fd->addr_hi;
		_bmb.lo	= fd->addr_lo;

		while (bman_release(_dpa_bp->pool, &_bmb, 1, 0))
			cpu_relax();

	} else {
		_fmt_err("frame not supported !\n");
		return -1;
	}

	return 0;
}

/* sync it w/ dpaa_eth.c: DPA_BP_HEAD */
#define DPA_BP_HEADROOM (DPA_TX_PRIV_DATA_SIZE + \
			fm_get_rx_extra_headroom() + \
			DPA_PARSE_RESULTS_SIZE + \
			DPA_HASH_RESULTS_SIZE)
#define MAC_HEADER_LENGTH 14
#define L2_AND_HEADROOM_OFF ((DPA_BP_HEADROOM) + (MAC_HEADER_LENGTH))

/* dpa ingress hooks definition */
enum dpaa_eth_hook_result fmt_rx_default_hook(
		struct sk_buff *skb,
		struct net_device *net_dev,
		u32 fqid)
{
	struct dpa_priv_s *dpa_priv = NULL;
	struct fmt_port_s *fmt_port = NULL;
	uint8_t *buffer;
	uint32_t buffer_len;

	_fmt_dbgr("calling...\n");

	dpa_priv = netdev_priv(net_dev);
	fmt_port = match_dpa_to_fmt_port(dpa_priv);

	/* conversion from skb to fd:
	 *  skb cames processed for L3, so we need to go back for
	 *  layer 2 offset */
	buffer = (uint8_t  *)(skb->data - ((int)L2_AND_HEADROOM_OFF));
	buffer_len = skb->len + ((int)L2_AND_HEADROOM_OFF);

	/* if is not out frame let dpa to handle it */
	if (test_and_steal_frame(fmt_port,
			FMT_RX_DFLT_Q,
			buffer,
			buffer_len))
		goto _fmt_rx_default_hook_stolen;

	_fmt_dbgr("called:DPAA_ETH_CONTINUE.\n");
	return DPAA_ETH_CONTINUE;

_fmt_rx_default_hook_stolen:
	dev_kfree_skb(skb);

	_fmt_dbgr("called:DPAA_ETH_STOLEN.\n");
	return DPAA_ETH_STOLEN;
}

enum dpaa_eth_hook_result fmt_rx_error_hook(
	struct net_device *net_dev,
	const struct qm_fd *fd,
	u32 fqid)
{
	struct dpa_priv_s *dpa_priv = NULL;
	struct dpa_bp *dpa_bp = NULL;
	struct fmt_port_s *fmt_port = NULL;
	void *fd_virt_addr = NULL;
	dma_addr_t addr = qm_fd_addr(fd);

	_fmt_dbgr("calling...\n");

	dpa_priv = netdev_priv(net_dev);
	fmt_port = match_dpa_to_fmt_port(dpa_priv);

	/* dpaa doesn't do this... we have to do it here */
	dpa_bp = dpa_bpid2pool(fd->bpid);
	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);

	fd_virt_addr = phys_to_virt(addr);
	/* if is not out frame let dpa to handle it */
	if (test_and_steal_frame(fmt_port,
			FMT_RX_ERR_Q,
			fd_virt_addr,
			fd->length20 + fd->offset)) {
		goto _fmt_rx_error_hook_stolen;
	}

	_fmt_dbgr("called:DPAA_ETH_CONTINUE.\n");
	return DPAA_ETH_CONTINUE;

_fmt_rx_error_hook_stolen:
	/* the frame data  doesn't matter,
	 * so, no mapping is needed */
	fmt_fq_release(fd);

	_fmt_dbgr("called:DPAA_ETH_STOLEN.\n");
	return DPAA_ETH_STOLEN;
}

enum dpaa_eth_hook_result fmt_tx_confirm_hook(
	struct net_device *net_dev,
	const struct qm_fd *fd,
	u32 fqid)
{
	struct dpa_priv_s *dpa_priv = NULL;
	struct fmt_port_s *fmt_port = NULL;
	dma_addr_t addr = qm_fd_addr(fd);
	void *fd_virt_addr = NULL;
	uint32_t fd_len = 0;

	_fmt_dbgr("calling...\n");

	dpa_priv = netdev_priv(net_dev);
	fmt_port = match_dpa_to_fmt_port(dpa_priv);

	fd_virt_addr = phys_to_virt(addr);
	fd_len = fd->length20 + fd->offset;

	if (fd_len > fm_get_max_frm()) {
		_fmt_err("tx confirm bad frame size: %u!\n", fd_len);
		goto _fmt_tx_confirm_hook_continue;
	}

	if (test_and_steal_frame(fmt_port,
			FMT_TX_CONF_Q,
			fd_virt_addr,
			fd_len))
		goto _fmt_tx_confirm_hook_stolen;

_fmt_tx_confirm_hook_continue:
	_fmt_dbgr("called:DPAA_ETH_CONTINUE.\n");
	return DPAA_ETH_CONTINUE;

_fmt_tx_confirm_hook_stolen:
	kfree(fd_virt_addr);

	_fmt_dbgr("called:DPAA_ETH_STOLEN.\n");
	return DPAA_ETH_STOLEN;
}

enum dpaa_eth_hook_result fmt_tx_confirm_error_hook(
	struct net_device *net_dev,
	const struct qm_fd *fd,
	u32 fqid)
{
	struct dpa_priv_s *dpa_priv = NULL;
	struct fmt_port_s *fmt_port = NULL;
	dma_addr_t addr = qm_fd_addr(fd);
	void *fd_virt_addr = NULL;
	uint32_t fd_len = 0;

	_fmt_dbgr("calling...\n");

	dpa_priv = netdev_priv(net_dev);
	fmt_port = match_dpa_to_fmt_port(dpa_priv);

	fd_virt_addr = phys_to_virt(addr);
	fd_len = fd->length20 + fd->offset;

	if (fd_len > fm_get_max_frm()) {
		_fmt_err("tx confirm err bad frame size: %u !\n", fd_len);
		goto _priv_ingress_tx_err_continue;
	}

	if (test_and_steal_frame(fmt_port, FMT_TX_ERR_Q, fd_virt_addr, fd_len))
		goto _priv_ingress_tx_err_stolen;

_priv_ingress_tx_err_continue:
	_fmt_dbgr("called:DPAA_ETH_CONTINUE.\n");
	return DPAA_ETH_CONTINUE;

_priv_ingress_tx_err_stolen:
	kfree(fd_virt_addr);

	_fmt_dbgr("called:DPAA_ETH_STOLEN.\n");
	return DPAA_ETH_STOLEN;
}

/* egress callbacks definition */
enum qman_cb_dqrr_result fmt_egress_dqrr(
		struct qman_portal         *portal,
		struct qman_fq             *fq,
		const struct qm_dqrr_entry *dqrr)
{
	/* this callback should never be called */
	BUG();
	return qman_cb_dqrr_consume;
}

static void  fmt_egress_error_dqrr(
		struct qman_portal *p,
		struct qman_fq *fq,
		const struct qm_mr_entry *msg)
{
	uint8_t *fd_virt_addr = NULL;

	/* tx failure, on the ern callback - release buffer */
	fd_virt_addr = (uint8_t  *)phys_to_virt(qm_fd_addr(&msg->ern.fd));
	kfree(fd_virt_addr);

	return;
}

static const struct qman_fq fmt_egress_fq = {
	.cb = { .dqrr = fmt_egress_dqrr,
		.ern = fmt_egress_error_dqrr,
		.fqs = NULL}
};

int fmt_fq_alloc(
	struct fmt_fqs_s *fmt_fqs,
	const struct qman_fq *qman_fq,
	uint32_t fqid,	uint32_t flags,
	uint16_t channel, uint8_t wq)
{
	int _errno = 0;

	_fmt_dbg("calling...\n");

	fmt_fqs->fq_base = *qman_fq;

	if (fqid == 0) {
		flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;
		flags &= ~QMAN_FQ_FLAG_NO_MODIFY;
	} else
		flags &= ~QMAN_FQ_FLAG_DYNAMIC_FQID;

	fmt_fqs->init = !(flags & QMAN_FQ_FLAG_NO_MODIFY);

	_errno = qman_create_fq(fqid, flags, &fmt_fqs->fq_base);
	if (_errno < 0) {
		_fmt_err("frame queues create failed.\n");
		return -EINVAL;
	}

	if (fmt_fqs->init) {
		struct qm_mcc_initfq initfq;

		initfq.we_mask          = QM_INITFQ_WE_DESTWQ;
		initfq.fqd.dest.channel = channel;
		initfq.fqd.dest.wq      = wq;

		_errno = qman_init_fq(&fmt_fqs->fq_base,
				QMAN_INITFQ_FLAG_SCHED,
				&initfq);
		if (_errno < 0) {
			_fmt_err("frame queues init erorr.\n");
			qman_destroy_fq(&fmt_fqs->fq_base, 0);
			return -EINVAL;
		}
	}

	_fmt_dbg("called.\n");
	return 0;
}

static int fmt_fq_free(struct fmt_fqs_s *fmt_fq)
{
	int _err = 0;

	_fmt_dbg("calling...\n");

	if (fmt_fq->init) {
		_err = qman_retire_fq(&fmt_fq->fq_base, NULL);
		if (unlikely(_err < 0))
			_fmt_err("qman_retire_fq(%u) = %d\n",
				qman_fq_fqid(&fmt_fq->fq_base), _err);

		_err = qman_oos_fq(&fmt_fq->fq_base);
		if (unlikely(_err < 0))
			_fmt_err("qman_oos_fq(%u) = %d\n",
				qman_fq_fqid(&fmt_fq->fq_base), _err);
	}

	qman_destroy_fq(&fmt_fq->fq_base, 0);

	_fmt_dbg("called.\n");
	return _err;
}

/* private pcd dqrr calbacks */
static enum qman_cb_dqrr_result fmt_pcd_dqrr(
		struct qman_portal *portal,
		struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct dpa_bp *dpa_bp = NULL;
	dma_addr_t addr = qm_fd_addr(&dq->fd);
	uint8_t *fd_virt_addr = NULL;
	struct fmt_port_s *fmt_port;
	struct fmt_port_pcd_s *fmt_port_pcd;
	uint32_t relative_fqid = 0;
	uint32_t fd_len = 0;

	_fmt_dbgr("calling...\n");

	/* upcast - from pcd_alloc_fq */
	fmt_port = ((struct fmt_fqs_s  *)fq)->fmt_port_priv;
	if (!fmt_port) {
		_fmt_err(" wrong fmt port -to- fq match.\n");
		goto _fmt_pcd_dqrr_return;
	}
	fmt_port_pcd = &fmt_port->fmt_port_pcd;

	relative_fqid = dq->fqid - fmt_port_pcd->fqid_base;
	_fmt_dbgr("pcd dqrr got frame on relative fq:%u@base:%u\n",
				relative_fqid, fmt_port_pcd->fqid_base);

	fd_len = dq->fd.length20 + dq->fd.offset;

	if (fd_len > fm_get_max_frm()) {
		_fmt_err("pcd dqrr wrong frame size: %u (%u:%u)!\n",
			fd_len, dq->fd.length20, dq->fd.offset);
		goto _fmt_pcd_dqrr_return;
	}

	dpa_bp = dpa_bpid2pool(dq->fd.bpid);
	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);

	fd_virt_addr = phys_to_virt(addr);
	if (!test_and_steal_frame(fmt_port, relative_fqid, fd_virt_addr,
								   fd_len)) {

#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
		atomic_inc(&fmt_port->not_enqueue_to_rxq_wrong_frm);
#endif
		_fmt_wrn("pcd dqrr unrecognized frame@fqid: %u,"
			 " frame len: %u (dropped).\n",
			dq->fqid, dq->fd.length20);
		dump_frame(fd_virt_addr, fd_len);
	}

_fmt_pcd_dqrr_return:
	/* no need to map again here */
	fmt_fq_release(&dq->fd);

	_fmt_dbgr("calle.\n");
	return qman_cb_dqrr_consume;
}

static void fmt_pcd_err_dqrr(
	struct qman_portal *qm,
	struct qman_fq *fq,
	const struct qm_mr_entry *msg)
{
	_fmt_err("this callback should never be called.\n");
	BUG();
	return;
}

static void fmt_pcd_fqs_dqrr(
	struct qman_portal *qm,
	struct qman_fq *fq,
	const struct qm_mr_entry *msg)
{
	_fmt_dbg(" fq state(0x%x)@fqid(%u.\n", msg->fq.fqs, msg->fq.fqid);
	return;
}

/* private pcd queue template */
static const struct qman_fq pcd_fq = {
	.cb = { .dqrr = fmt_pcd_dqrr,
		.ern = fmt_pcd_err_dqrr,
		.fqs = fmt_pcd_fqs_dqrr}
};

/* defined as weak in dpaa driver. */
/* ! parameters come from IOCTL call - US */
int dpa_alloc_pcd_fqids(
		struct device *dev,
		uint32_t num, uint8_t alignment,
		uint32_t *base_fqid)
{
	int _err = 0, i;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *dpa_priv = NULL;
	struct fmt_port_pcd_s *fmt_port_pcd = NULL;
	struct fmt_fqs_s *fmt_fqs = NULL;
	struct fmt_port_s *fmt_port = NULL;
	int num_allocated = 0;

	_fmt_dbg("calling...\n");

	net_dev = (typeof(net_dev))dev_get_drvdata(dev);
	dpa_priv = (typeof(dpa_priv))netdev_priv(net_dev);

	if (!netif_msg_probe(dpa_priv)) {
		_fmt_err("dpa not probe.\n");
		_err = -ENODEV;
		goto _pcd_alloc_fqs_err;
	}

	fmt_port = match_dpa_to_fmt_port(dpa_priv);
	if (!fmt_port) {
		_fmt_err("fmt port not found.");
		_err = -EINVAL;
		goto _pcd_alloc_fqs_err;
	}

	fmt_port_pcd = &fmt_port->fmt_port_pcd;

	num_allocated = qman_alloc_fqid_range(base_fqid, num, alignment, 0);

	if ((num_allocated <= 0) ||
	    (num_allocated < num) ||
	    (alignment && (*base_fqid) % alignment)) {
		*base_fqid = 0;
		_fmt_err("Failed to alloc pcd fqs rang.\n");
		_err = -EINVAL;
		goto _pcd_alloc_fqs_err;
	}

	_fmt_dbg("wanted %d fqs(align %d), got %d fqids@%u.\n",
				num, alignment, num_allocated, *base_fqid);

	/* alloc pcd queues */
	fmt_port_pcd->fmt_pcd_fqs = kmalloc(num_allocated *
						sizeof(struct fmt_fqs_s),
						GFP_KERNEL);
	fmt_port_pcd->num_queues = num_allocated;
	fmt_port_pcd->fqid_base = *base_fqid;
	fmt_fqs = fmt_port_pcd->fmt_pcd_fqs;

	/* alloc the pcd queues */
	for (i = 0; i < num_allocated; i++, fmt_fqs++) {
		_err = fmt_fq_alloc(
			fmt_fqs,
			&pcd_fq,
			(*base_fqid) + i, QMAN_FQ_FLAG_NO_ENQUEUE,
			dpa_priv->channel, 7);

		if (_err < 0)
			goto _pcd_alloc_fqs_err;

		/* upcast to identify from where the frames came from */
		fmt_fqs->fmt_port_priv = fmt_port;
	}

	_fmt_dbg("called.\n");
	return _err;
_pcd_alloc_fqs_err:
	if (num_allocated > 0)
		qman_release_fqid_range(*base_fqid, num_allocated);
	/*TODO: free fmt_pcd_fqs if are any */

	_fmt_dbg("called(_err:%d).\n", _err);
	return _err;
}

/* defined as weak in dpaa driver. */
int dpa_free_pcd_fqids(
	struct device *dev,
	uint32_t base_fqid)
{

	int _err = 0, i;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *dpa_priv = NULL;
	struct fmt_port_pcd_s *fmt_port_pcd = NULL;
	struct fmt_fqs_s *fmt_fqs = NULL;
	struct fmt_port_s *fmt_port = NULL;
	int num_allocated = 0;

	_fmt_dbg("calling...\n");

	net_dev = (typeof(net_dev))dev_get_drvdata(dev);
	dpa_priv = (typeof(dpa_priv))netdev_priv(net_dev);

	if (!netif_msg_probe(dpa_priv)) {
		_fmt_err("dpa not probe.\n");
		_err = -ENODEV;
		goto _pcd_free_fqs_err;
	}

	fmt_port = match_dpa_to_fmt_port(dpa_priv);
	if (!fmt_port) {
		_fmt_err("fmt port not found.");
		_err = -EINVAL;
		goto _pcd_free_fqs_err;
	}

	fmt_port_pcd = &fmt_port->fmt_port_pcd;
	num_allocated = fmt_port_pcd->num_queues;
	fmt_fqs = fmt_port_pcd->fmt_pcd_fqs;

	for (i = 0; i < num_allocated; i++, fmt_fqs++)
		fmt_fq_free(fmt_fqs);

	qman_release_fqid_range(base_fqid,num_allocated);

	kfree(fmt_port_pcd->fmt_pcd_fqs);
	memset(fmt_port_pcd, 0, sizeof(*fmt_port_pcd));

	/* debugging stuff */
#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	_fmt_dbg(" portid: %u.\n", fmt_port->id);
	_fmt_dbg(" frames enqueue to qman: %u.\n",
			atomic_read(&fmt_port->enqueue_to_qman_frm));
	_fmt_dbg(" frames enqueue to rxq: %u.\n",
			atomic_read(&fmt_port->enqueue_to_rxq));
	_fmt_dbg(" frames dequeue from rxq: %u.\n",
			atomic_read(&fmt_port->dequeue_from_rxq));
	_fmt_dbg(" frames not enqueue to rxq - wrong frm: %u.\n",
			atomic_read(&fmt_port->not_enqueue_to_rxq_wrong_frm));
	atomic_set(&fmt_port->enqueue_to_qman_frm, 0);
	atomic_set(&fmt_port->enqueue_to_rxq, 0);
	atomic_set(&fmt_port->dequeue_from_rxq, 0);
	atomic_set(&fmt_port->not_enqueue_to_rxq_wrong_frm, 0);
#endif
	return 0;

_pcd_free_fqs_err:
	return _err;
}

static int fmt_port_init(
	struct fmt_port_s *fmt_port,
	ioc_fmt_port_param_t *p_Params)
{
	struct device_node  *fm_node, *fm_port_node;
	const uint32_t      *uint32_prop;
	int                 _errno = 0, lenp = 0, i;
	static struct of_device_id fm_node_of_match[] = {
		{ .compatible = "fsl,fman", },
		{ /* end of list */ },
		};

	_fmt_dbg("calling...\n");

	/* init send/receive tu US list */
	INIT_LIST_HEAD(&fmt_port->rx_q);

	/* check parameters */
	if (p_Params->num_tx_queues > FMAN_TEST_MAX_TX_FQS ||
		p_Params->fm_port_id > IOC_FMT_MAX_NUM_OF_PORTS) {
		_fmt_dbg("wrong test parameters.\n");
		return -EINVAL;
	}

	/* set port parameters */
	fmt_port->num_of_tx_fqs = p_Params->num_tx_queues;
	fmt_port->id = p_Params->fm_port_id;
	fmt_port->port_type = p_Params->fm_port_type;
	fmt_port->diag = e_IOC_DIAG_MODE_NONE;

	/* init debugging stuff */
#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	atomic_set(&fmt_port->enqueue_to_qman_frm, 0);
	atomic_set(&fmt_port->enqueue_to_rxq, 0);
	atomic_set(&fmt_port->dequeue_from_rxq, 0);
	atomic_set(&fmt_port->not_enqueue_to_rxq_wrong_frm, 0);
#endif

	/* TODO: This should be done at probe time not at runtime
	 *	very ugly function */
	/* fill fmt port properties from dts */
	for_each_matching_node(fm_node, fm_node_of_match) {

	uint32_prop = (uint32_t *)of_get_property(fm_node,
						"cell-index", &lenp);
	if (unlikely(uint32_prop == NULL)) {
		_fmt_wrn("of_get_property(%s, cell-index) invalid",
						fm_node->full_name);
		return -EINVAL;
	}
	if (WARN_ON(lenp != sizeof(uint32_t))) {
		_fmt_wrn("of_get_property(%s, cell-index) invalid",
						fm_node->full_name);
		return -EINVAL;
	}

	if (*uint32_prop == p_Params->fm_id) {
		struct resource res;

		/* Get the FM address */
		_errno = of_address_to_resource(fm_node, 0, &res);
		if (unlikely(_errno < 0)) {
			_fmt_wrn("of_address_to_resource() = %u.\n", _errno);
			return -EINVAL;
		}

		fmt_port->fm_phys_base_addr = res.start;

		for_each_child_of_node(fm_node, fm_port_node) {
		struct platform_device    *of_dev;

		if (!of_device_is_available(fm_port_node))
			continue;

		uint32_prop = (uint32_t *)of_get_property(
						fm_port_node,
						"cell-index",
						&lenp);
		if (uint32_prop == NULL)
			continue;

		if (of_device_is_compatible(fm_port_node,
						"fsl,fman-port-oh") &&
			(fmt_port->port_type  == e_IOC_FMT_PORT_T_OP)) {

			if (*uint32_prop == fmt_port->id) {
				of_dev = of_find_device_by_node(fm_port_node);
				if (unlikely(of_dev == NULL)) {
					_fmt_wrn("fm id invalid\n");
					return -EINVAL;
				}

				fmt_port->p_tx_port =
						fm_port_bind(&of_dev->dev);
				fmt_port->p_tx_fm_port_dev =
						(void  *)fm_port_get_handle(
							fmt_port->p_tx_port);
				fmt_port->p_rx_port =
						fmt_port->p_tx_port;
				fmt_port->p_rx_fm_port_dev =
						fmt_port->p_tx_fm_port_dev;
				fmt_port->p_mac_dev = NULL;
			break;
			}
		} else if ((*uint32_prop == fmt_port->id) &&
			fmt_port->port_type == e_IOC_FMT_PORT_T_RXTX) {

			of_dev = of_find_device_by_node(fm_port_node);
			if (unlikely(of_dev == NULL)) {
				_fmt_wrn("dtb fm id invalid value");
				return -EINVAL;
			}

			if (of_device_is_compatible(fm_port_node,
					   "fsl,fman-port-1g-tx")) {
				fmt_port->p_tx_port =
						fm_port_bind(&of_dev->dev);
				fmt_port->p_tx_fm_port_dev = (void  *)
						fm_port_get_handle(
							fmt_port->p_tx_port);
			} else if (of_device_is_compatible(fm_port_node,
						      "fsl,fman-port-1g-rx")) {
				fmt_port->p_rx_port =
						fm_port_bind(&of_dev->dev);
				fmt_port->p_rx_fm_port_dev = (void  *)
						fm_port_get_handle(
							fmt_port->p_rx_port);
			} else if (of_device_is_compatible(fm_port_node,
							"fsl,fman-1g-mac") ||
				   of_device_is_compatible(fm_port_node,
							"fsl,fman-memac"))
				fmt_port->p_mac_dev =
						(typeof(fmt_port->p_mac_dev))
						dev_get_drvdata(&of_dev->dev);
			else
				continue;

			if (fmt_port->p_tx_fm_port_dev &&
			fmt_port->p_rx_fm_port_dev && fmt_port->p_mac_dev)
				break;
		} else if (((*uint32_prop + FM_MAX_NUM_OF_1G_RX_PORTS) ==
				fmt_port->id) &&
				fmt_port->port_type == e_IOC_FMT_PORT_T_RXTX) {

			of_dev = of_find_device_by_node(fm_port_node);
			if (unlikely(of_dev == NULL)) {
				_fmt_wrn("dtb fm id invalid value\n");
				return -EINVAL;
			}

			if (of_device_is_compatible(fm_port_node,
					     "fsl,fman-port-10g-tx")) {
				fmt_port->p_tx_port =
						fm_port_bind(&of_dev->dev);
				fmt_port->p_tx_fm_port_dev = (void *)
						fm_port_get_handle(
							fmt_port->p_tx_port);
			} else if (of_device_is_compatible(fm_port_node,
						     "fsl,fman-port-10g-rx")) {
				fmt_port->p_rx_port =
						fm_port_bind(&of_dev->dev);
				fmt_port->p_rx_fm_port_dev = (void *)
						fm_port_get_handle(
							fmt_port->p_rx_port);
			} else if (of_device_is_compatible(fm_port_node,
							"fsl,fman-10g-mac") ||
				   of_device_is_compatible(fm_port_node,
							"fsl,fman-memac"))
				fmt_port->p_mac_dev =
						(typeof(fmt_port->p_mac_dev))
						dev_get_drvdata(&of_dev->dev);
			else
				continue;

			if (fmt_port->p_tx_fm_port_dev &&
			fmt_port->p_rx_fm_port_dev && fmt_port->p_mac_dev)
				break;
			}
		} /* for_each_child */
	}
	} /* for each matching node */

	if (fmt_port->p_tx_fm_port_dev == 0 ||
		fmt_port->p_rx_fm_port_dev == 0) {

		_fmt_err("bad fm port pointers.\n");
		return -EINVAL;
	}

	_fmt_dbg("alloc %u tx queues.\n", fmt_port->num_of_tx_fqs);

	/* init fman test egress dynamic frame queues */
	for (i = 0; i < fmt_port->num_of_tx_fqs; i++) {
		int _errno;
		_errno = fmt_fq_alloc(
				&fmt_port->p_tx_fqs[i],
				&fmt_egress_fq,
				0,
				QMAN_FQ_FLAG_TO_DCPORTAL,
				fm_get_tx_port_channel(fmt_port->p_tx_port),
				i);

		if (_errno < 0) {
			_fmt_err("tx queues allocation failed.\n");
			/* TODO: memory leak here if 1 queue is allocated and
			* next queues are failing ... */
			return -EINVAL;
		}
	}

	/* port is valid and ready to use. */
	fmt_port->valid  = TRUE;

	_fmt_dbg("called.\n");
	return 0;
}

/* fm test chardev functions */
static int fmt_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);

	_fmt_dbg("calling...\n");

	if (file->private_data != NULL)
		return 0;

	/* The minor represent the port number.
	 * Set the port structure accordingly, thus all the operations
	 * will be done on this port. */
	if ((minor >= DEV_FM_TEST_PORTS_MINOR_BASE) &&
	    (minor < DEV_FM_TEST_MAX_MINORS))
		file->private_data = &fm_test.ports[minor];
	else
		return -ENXIO;

	_fmt_dbg("called.\n");
	return 0;
}

static int fmt_close(struct inode *inode, struct file *file)
{
	struct fmt_port_s *fmt_port = NULL;
	struct fmt_frame_s *fmt_frame = NULL;

	int err = 0;

	_fmt_dbg("calling...\n");

	fmt_port = file->private_data;
	if (!fmt_port)
		return -ENODEV;

	/* Close the current test port by invalidating it. */
	fmt_port->valid = FALSE;

	/* clean the fmt port queue */
	while ((fmt_frame = dequeue_fmt_frame(fmt_port)) != NULL) {
		if (fmt_frame && fmt_frame->buff.p_data){
		kfree(fmt_frame->buff.p_data);
		kfree(fmt_frame);
	}
	}

	/* !!! the qman queues are cleaning from fm_ioctl...
	 * - very ugly */

	_fmt_dbg("called.\n");
	return err;
}

static int fmt_ioctls(unsigned int minor,
			struct file *file,
			unsigned int cmd,
			unsigned long arg,
			bool compat)
{
	struct fmt_port_s *fmt_port = NULL;

	_fmt_dbg("IOCTL minor:%u "
		  " arg:0x%08lx ioctl cmd (0x%08x):(0x%02x:0x%02x.\n",
		   minor, arg, cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));

	fmt_port = file->private_data;
	if (!fmt_port) {
		_fmt_err("invalid fmt port.\n");
		return -ENODEV;
	}

	/* set test type properly */
	if (compat)
		fmt_port->compat_test_type = true;
	else
		fmt_port->compat_test_type = false;

	switch (cmd) {
	case FMT_PORT_IOC_INIT:
	{
		ioc_fmt_port_param_t param;

		if (fmt_port->valid) {
			_fmt_wrn("port is already initialized.\n");
			return -EFAULT;
		}
#if defined(CONFIG_COMPAT)
		if (compat) {
			if (copy_from_user(&param,
				(ioc_fmt_port_param_t  *)compat_ptr(arg),
				sizeof(ioc_fmt_port_param_t)))

				return -EFAULT;
		} else
#endif
		{
			if (copy_from_user(&param,
				(ioc_fmt_port_param_t  *) arg,
				sizeof(ioc_fmt_port_param_t)))

				return -EFAULT;
		}

		return fmt_port_init(fmt_port, &param);
	}

	case FMT_PORT_IOC_SET_DIAG_MODE:
		if (get_user(fmt_port->diag, (ioc_diag_mode  *)arg))
			return -EFAULT;

		if (fmt_port->diag == e_IOC_DIAG_MODE_CTRL_LOOPBACK)
			return set_mac_int_loopback(fmt_port, TRUE);
		else
			return set_mac_int_loopback(fmt_port, FALSE);
	break;

	case FMT_PORT_IOC_SET_DPAECHO_MODE:
	case FMT_PORT_IOC_SET_IP_HEADER_MANIP:
	default:
		_fmt_wrn("ioctl unimplemented minor:%u@ioctl"
			  " cmd:0x%08x(type:0x%02x, nr:0x%02x.\n",
			  minor, cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));
	return -EFAULT;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long fmt_compat_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long arg)
{
	unsigned int minor = iminor(file->f_path.dentry->d_inode);

	_fmt_dbg("calling...\n");
	return fmt_ioctls(minor, file, cmd, arg, true);
}
#endif

static long fmt_ioctl(
		struct file *file,
		unsigned int cmd,
		unsigned long arg)
{
	unsigned int minor = iminor(file->f_path.dentry->d_inode);
	unsigned int res;

	_fmt_dbg("calling...\n");

	fm_mutex_lock();
	res = fmt_ioctls(minor, file, cmd, arg, false);
	fm_mutex_unlock();

	_fmt_dbg("called.\n");

	return res;
}

#ifdef CONFIG_COMPAT
void copy_compat_test_frame_buffer(
		ioc_fmt_buff_desc_t *buff,
		ioc_fmt_compat_buff_desc_t *compat_buff)
{
	compat_buff->qid = buff->qid;
	compat_buff->p_data = ptr_to_compat(buff->p_data);
	compat_buff->size = buff->size;
	compat_buff->status = buff->status;

	compat_buff->buff_context.p_user_priv =
			ptr_to_compat(buff->buff_context.p_user_priv);
	memcpy(compat_buff->buff_context.fm_prs_res,
			buff->buff_context.fm_prs_res,
			FM_PRS_MAX * sizeof(uint8_t));
	memcpy(compat_buff->buff_context.fm_time_stamp,
			buff->buff_context.fm_time_stamp,
			FM_TIME_STAMP_MAX * sizeof(uint8_t));
}
#endif

ssize_t fmt_read(
		struct file *file,
		char __user *buf,
		size_t size,
		loff_t *ppos)
{
	struct fmt_port_s *fmt_port = NULL;
	struct fmt_frame_s *p_fmt_frame = NULL;
	ssize_t cnt = 0;

	fmt_port = file->private_data;
	if (!fmt_port || !fmt_port->valid) {
		_fmt_err("fmt port not valid!\n");
		return -ENODEV;
	}

	p_fmt_frame = dequeue_fmt_frame(fmt_port);
	if (p_fmt_frame == NULL)
		return 0;

	_fmt_dbgr("calling...\n");

#ifdef CONFIG_COMPAT
	if (fmt_port->compat_test_type){
		cnt = sizeof(ioc_fmt_compat_buff_desc_t);
	}
	else
#endif
	{
		cnt = sizeof(ioc_fmt_buff_desc_t);
	}

	if (size < cnt) {
		_fmt_err("illegal buffer-size!\n");
		cnt = 0;
		goto _fmt_read_return;
	}

        /* Copy structure */
#ifdef CONFIG_COMPAT
	if (fmt_port->compat_test_type) {
		{
			ioc_fmt_compat_buff_desc_t compat_buff;
			copy_compat_test_frame_buffer(&p_fmt_frame->buff,
								&compat_buff);

			if (copy_to_user(buf, &compat_buff, cnt)) {
				_fmt_err("copy_to_user failed!\n");
				goto _fmt_read_return;
			}
		}

		((ioc_fmt_compat_buff_desc_t  *)buf)->p_data =
			ptr_to_compat(buf+sizeof(ioc_fmt_compat_buff_desc_t));
		cnt += MIN(p_fmt_frame->buff.size, size-cnt);
	} else
#endif
	{
		if (copy_to_user(buf, &p_fmt_frame->buff, cnt)) {
			_fmt_err("copy_to_user failed!\n");
			goto _fmt_read_return;
		}

		((ioc_fmt_buff_desc_t  *)buf)->p_data =
				buf + sizeof(ioc_fmt_buff_desc_t);
		cnt += MIN(p_fmt_frame->buff.size, size-cnt);
	}

	if (size < cnt) {
		_fmt_err("illegal buffer-size!\n");
		goto _fmt_read_return;
	}

	/* copy frame */
#ifdef CONFIG_COMPAT
	if (fmt_port->compat_test_type) {
		if (copy_to_user(buf+sizeof(ioc_fmt_compat_buff_desc_t),
					p_fmt_frame->buff.p_data, cnt)) {
			_fmt_err("copy_to_user failed!\n");
			goto _fmt_read_return;
		}
	} else
#endif
	{
		if (copy_to_user(buf+sizeof(ioc_fmt_buff_desc_t),
					p_fmt_frame->buff.p_data, cnt)) {
			_fmt_err("copy_to_user failed!\n");
			goto _fmt_read_return;
		}
	}

_fmt_read_return:
	kfree(p_fmt_frame->buff.p_data);
	kfree(p_fmt_frame);

	_fmt_dbgr("called.\n");
	return cnt;
}

ssize_t fmt_write(
		struct file *file,
		const char __user *buf,
		size_t size,
		loff_t *ppos)
{
	struct fmt_port_s *fmt_port = NULL;
	ioc_fmt_buff_desc_t buff_desc;
#ifdef CONFIG_COMPAT
	ioc_fmt_compat_buff_desc_t buff_desc_compat;
#endif
	uint8_t *p_data = NULL;
	uint32_t data_offset;
	int _errno;
	t_DpaaFD fd;

	_fmt_dbgr("calling...\n");

	fmt_port = file->private_data;
	if (!fmt_port || !fmt_port->valid) {
		_fmt_err("fmt port not valid.\n");
		return -EINVAL;
	}

    /* If Compat (32B UserSpace - 64B KernelSpace)  */
#ifdef CONFIG_COMPAT
	if (fmt_port->compat_test_type) {
		if (size < sizeof(ioc_fmt_compat_buff_desc_t)) {
			_fmt_err("invalid buff_desc size.\n");
			return -EFAULT;
		}

		if (copy_from_user(&buff_desc_compat, buf,
					sizeof(ioc_fmt_compat_buff_desc_t)))
			return -EFAULT;

		buff_desc.qid = buff_desc_compat.qid;
		buff_desc.p_data = compat_ptr(buff_desc_compat.p_data);
		buff_desc.size = buff_desc_compat.size;
		buff_desc.status = buff_desc_compat.status;

		buff_desc.buff_context.p_user_priv =
			compat_ptr(buff_desc_compat.buff_context.p_user_priv);
		memcpy(buff_desc.buff_context.fm_prs_res,
				buff_desc_compat.buff_context.fm_prs_res,
				FM_PRS_MAX * sizeof(uint8_t));
		memcpy(buff_desc.buff_context.fm_time_stamp,
				buff_desc_compat.buff_context.fm_time_stamp,
				FM_TIME_STAMP_MAX * sizeof(uint8_t));
	} else
#endif
	{
		if (size < sizeof(ioc_fmt_buff_desc_t)) {
			_fmt_err("invalid buff_desc size.\n");
			return -EFAULT;
		}

		if (copy_from_user(&buff_desc, (ioc_fmt_buff_desc_t  *)buf,
					sizeof(ioc_fmt_buff_desc_t)))
			return -EFAULT;
	}

	data_offset = FM_PORT_GetBufferDataOffset(fmt_port->p_tx_fm_port_dev);
	p_data = kmalloc(buff_desc.size+data_offset, GFP_KERNEL);
	if (!p_data)
		return -ENOMEM;

	/* If Compat (32UserSpace - 64KernelSpace) the buff_desc.p_data is ok */
	if (copy_from_user((uint8_t *)PTR_MOVE(p_data, data_offset),
				buff_desc.p_data,
				buff_desc.size)) {
		kfree(p_data);
		return -EFAULT;
	}

	/* TODO: dma_map_single here (cannot access the bpool struct) */

	/* prepare fd */
	memset(&fd, 0, sizeof(fd));
	DPAA_FD_SET_ADDR(&fd, p_data);
	DPAA_FD_SET_OFFSET(&fd, data_offset);
	DPAA_FD_SET_LENGTH(&fd, buff_desc.size);

	_errno = qman_enqueue(&fmt_port->p_tx_fqs[buff_desc.qid].fq_base,
							(struct qm_fd *)&fd, 0);
	if (_errno) {
		buff_desc.status = (uint32_t)_errno;
		if (copy_to_user((ioc_fmt_buff_desc_t *)buf, &buff_desc,
						sizeof(ioc_fmt_buff_desc_t))) {
			kfree(p_data);
			return -EFAULT;
		}
	}

	/* for debugging */
#if defined(FMT_K_DBG) || defined(FMT_K_DBG_RUNTIME)
	atomic_inc(&fmt_port->enqueue_to_qman_frm);
#endif
	_fmt_dbgr("called.\n");
	return buff_desc.size;
}

/* fm test character device definition */
static const struct file_operations fmt_fops =
{
	.owner			= THIS_MODULE,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= fmt_compat_ioctl,
#endif
	.unlocked_ioctl		= fmt_ioctl,
	.open			= fmt_open,
	.release		= fmt_close,
	.read			= fmt_read,
	.write			= fmt_write,
};

static int fmt_init(void)
{
	int id;

	_fmt_dbg("calling...\n");

	/* Register to the /dev for IOCTL API */
	/* Register dynamically a new major number for the character device: */
	fm_test.major = register_chrdev(0, DEV_FM_TEST_NAME, &fmt_fops);
	if (fm_test.major  <= 0) {
		_fmt_wrn("Failed to allocate major number for device %s.\n",
							DEV_FM_TEST_NAME);
		return -ENODEV;
	}

	/* Creating class for FMan_test */
	fm_test.fmt_class = class_create(THIS_MODULE, DEV_FM_TEST_NAME);
	if (IS_ERR(fm_test.fmt_class)) {
		unregister_chrdev(fm_test.major, DEV_FM_TEST_NAME);
		_fmt_wrn("Error creating %s class.\n", DEV_FM_TEST_NAME);
		return -ENODEV;
	}

	for (id = 0; id < IOC_FMT_MAX_NUM_OF_PORTS; id++)
		if (NULL == device_create(fm_test.fmt_class, NULL,
				MKDEV(fm_test.major,
				DEV_FM_TEST_PORTS_MINOR_BASE + id), NULL,
				DEV_FM_TEST_NAME "%d", id)) {

			_fmt_err("Error creating %s device.\n",
							DEV_FM_TEST_NAME);
			return -ENODEV;
		}

	return 0;
}

static void  fmt_free(void)
{
	int id;

	for (id = 0; id < IOC_FMT_MAX_NUM_OF_PORTS; id++)
		device_destroy(fm_test.fmt_class, MKDEV(fm_test.major,
			DEV_FM_TEST_PORTS_MINOR_BASE + id));
	class_destroy(fm_test.fmt_class);
}

static int __init __cold fmt_load(void)
{
	struct dpaa_eth_hooks_s priv_dpaa_eth_hooks;

	/* set dpaa hooks for default queues */
	memset(&priv_dpaa_eth_hooks, 0, sizeof(priv_dpaa_eth_hooks));
	priv_dpaa_eth_hooks.rx_default = fmt_rx_default_hook;
	priv_dpaa_eth_hooks.rx_error = fmt_rx_error_hook;
	priv_dpaa_eth_hooks.tx_confirm = fmt_tx_confirm_hook;
	priv_dpaa_eth_hooks.tx_error = fmt_tx_confirm_error_hook;

	fsl_dpaa_eth_set_hooks(&priv_dpaa_eth_hooks);

	/* initialize the fman test environment */
	if (fmt_init() < 0) {
		_fmt_err("Failed to init FM-test modul.\n");
		fmt_free();
		return -ENODEV;
	}

	_fmt_inf("FSL FM test module loaded.\n");

	return 0;
}

static void __exit __cold fmt_unload(void)
{
	fmt_free();
	_fmt_inf("FSL FM test module unloaded.\n");
}

module_init(fmt_load);
module_exit(fmt_unload);
