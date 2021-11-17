/* Copyright 2008-2013 Freescale Semiconductor Inc.
 * Copyright 2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__
#else
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": " fmt
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/if_arp.h>	/* arp_hdr_len() */
#include <linux/if_vlan.h>	/* VLAN_HLEN */
#include <linux/icmp.h>		/* struct icmphdr */
#include <linux/ip.h>		/* struct iphdr */
#include <linux/ipv6.h>		/* struct ipv6hdr */
#include <linux/udp.h>		/* struct udphdr */
#include <linux/tcp.h>		/* struct tcphdr */
#include <linux/net.h>		/* net_ratelimit() */
#include <linux/if_ether.h>	/* ETH_P_IP and ETH_P_IPV6 */
#include <linux/highmem.h>
#include <linux/percpu.h>
#include <linux/dma-mapping.h>
#include <linux/fsl_bman.h>
#ifdef CONFIG_SOC_BUS
#include <linux/sys_soc.h>      /* soc_device_match */
#endif

#include "fsl_fman.h"
#include "fm_ext.h"
#include "fm_port_ext.h"

#include "mac.h"
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#ifdef CONFIG_FSL_DPAA_DBG_LOOP
#include "dpaa_debugfs.h"
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */
#ifdef CONFIG_FSL_DPAA_CEETM
#include "dpaa_eth_ceetm.h"
#endif

/* CREATE_TRACE_POINTS only needs to be defined once. Other dpa files
 * using trace events only need to #include <trace/events/sched.h>
 */
#define CREATE_TRACE_POINTS
#include "dpaa_eth_trace.h"

#define DPA_NAPI_WEIGHT		64

/* Valid checksum indication */
#define DPA_CSUM_VALID		0xFFFF

#define DPA_DESCRIPTION "FSL DPAA Ethernet driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Andy Fleming <afleming@freescale.com>");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

static uint8_t debug = -1;
module_param(debug, byte, S_IRUGO);
MODULE_PARM_DESC(debug, "Module/Driver verbosity level");

/* This has to work in tandem with the DPA_CS_THRESHOLD_xxx values. */
static uint16_t tx_timeout = 1000;
module_param(tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(tx_timeout, "The Tx timeout in ms");

static const char rtx[][3] = {
	[RX] = "RX",
	[TX] = "TX"
};

/* BM */

#define DPAA_ETH_MAX_PAD (L1_CACHE_BYTES * 8)

static uint8_t dpa_priv_common_bpid;

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
struct net_device *dpa_loop_netdevs[20];
#endif

#ifdef CONFIG_FSL_DPAA_CEETM
extern struct Qdisc_ops ceetm_qdisc_ops;
#endif

#ifdef CONFIG_PM

static int dpaa_suspend(struct device *dev)
{
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;
	struct mac_device	*mac_dev;
	int			err = 0;

	net_dev = dev_get_drvdata(dev);

	if (net_dev->flags & IFF_UP) {
		priv = netdev_priv(net_dev);
		mac_dev = priv->mac_dev;

		if (priv->wol & DPAA_WOL_MAGIC) {
			err = priv->mac_dev->set_wol(mac_dev->port_dev[RX],
				priv->mac_dev->get_mac_handle(mac_dev), true);
			if (err) {
				netdev_err(net_dev, "set_wol() = %d\n", err);
				goto set_wol_failed;
			}
		}

		err = fm_port_suspend(mac_dev->port_dev[RX]);
		if (err) {
			netdev_err(net_dev, "fm_port_suspend(RX) = %d\n", err);
			goto rx_port_suspend_failed;
		}

		err = fm_port_suspend(mac_dev->port_dev[TX]);
		if (err) {
			netdev_err(net_dev, "fm_port_suspend(TX) = %d\n", err);
			goto tx_port_suspend_failed;
		}
	}

	return 0;

tx_port_suspend_failed:
	fm_port_resume(mac_dev->port_dev[RX]);
rx_port_suspend_failed:
	if (priv->wol & DPAA_WOL_MAGIC) {
		priv->mac_dev->set_wol(mac_dev->port_dev[RX],
			priv->mac_dev->get_mac_handle(mac_dev), false);
	}
set_wol_failed:
	return err;
}

static int dpaa_resume(struct device *dev)
{
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;
	struct mac_device	*mac_dev;
	int			err = 0;

	net_dev = dev_get_drvdata(dev);

	if (net_dev->flags & IFF_UP) {
		priv = netdev_priv(net_dev);
		mac_dev = priv->mac_dev;

		err = fm_mac_resume(mac_dev->get_mac_handle(mac_dev));
		if (err) {
			netdev_err(net_dev, "fm_mac_resume = %d\n", err);
			goto resume_failed;
		}

		err = fm_port_resume(mac_dev->port_dev[TX]);
		if (err) {
			netdev_err(net_dev, "fm_port_resume(TX) = %d\n", err);
			goto resume_failed;
		}

		err = fm_port_resume(mac_dev->port_dev[RX]);
		if (err) {
			netdev_err(net_dev, "fm_port_resume(RX) = %d\n", err);
			goto resume_failed;
		}

		if (priv->wol & DPAA_WOL_MAGIC) {
			err = priv->mac_dev->set_wol(mac_dev->port_dev[RX],
				priv->mac_dev->get_mac_handle(mac_dev), false);
			if (err) {
				netdev_err(net_dev, "set_wol() = %d\n", err);
				goto resume_failed;
			}
		}
	}

	return 0;

resume_failed:
	return err;
}

static const struct dev_pm_ops dpaa_pm_ops = {
	.suspend = dpaa_suspend,
	.resume = dpaa_resume,
};

#define DPAA_PM_OPS (&dpaa_pm_ops)

#else /* CONFIG_PM */

#define DPAA_PM_OPS NULL

#endif /* CONFIG_PM */

/* Checks whether the checksum field in Parse Results array is valid
 * (equals 0xFFFF) and increments the .cse counter otherwise
 */
static inline void
dpa_csum_validation(const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd)
{
	dma_addr_t addr = qm_fd_addr(fd);
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	void *frm = phys_to_virt(addr);
	fm_prs_result_t *parse_result;

	if (unlikely(!frm))
		return;

	dma_sync_single_for_cpu(dpa_bp->dev, addr, DPA_RX_PRIV_DATA_SIZE +
				DPA_PARSE_RESULTS_SIZE, DMA_BIDIRECTIONAL);

	parse_result = (fm_prs_result_t *)(frm + DPA_RX_PRIV_DATA_SIZE);

	if (parse_result->cksum != DPA_CSUM_VALID)
		percpu_priv->rx_errors.cse++;
}

static void _dpa_rx_error(struct net_device *net_dev,
		const struct dpa_priv_s	*priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid)
{
	/* limit common, possibly innocuous Rx FIFO Overflow errors'
	 * interference with zero-loss convergence benchmark results.
	 */
	if (likely(fd->status & FM_FD_STAT_ERR_PHYSICAL))
		pr_warn_once("fsl-dpa: non-zero error counters in fman statistics (sysfs)\n");
	else
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_dbg(net_dev, "Err FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_RX_ERRORS);
#ifdef CONFIG_FSL_DPAA_HOOKS
	if (dpaa_eth_hooks.rx_error &&
		dpaa_eth_hooks.rx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* it's up to the hook to perform resource cleanup */
		return;
#endif
	percpu_priv->stats.rx_errors++;

	if (fd->status & FM_PORT_FRM_ERR_DMA)
		percpu_priv->rx_errors.dme++;
	if (fd->status & FM_PORT_FRM_ERR_PHYSICAL)
		percpu_priv->rx_errors.fpe++;
	if (fd->status & FM_PORT_FRM_ERR_SIZE)
		percpu_priv->rx_errors.fse++;
	if (fd->status & FM_PORT_FRM_ERR_PRS_HDR_ERR)
		percpu_priv->rx_errors.phe++;
	if (fd->status & FM_FD_STAT_L4CV)
		dpa_csum_validation(priv, percpu_priv, fd);

	dpa_fd_release(net_dev, fd);
}

static void _dpa_tx_error(struct net_device		*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff *skb;

	if (netif_msg_hw(priv) && net_ratelimit())
		netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_TX_ERRORS);
#ifdef CONFIG_FSL_DPAA_HOOKS
	if (dpaa_eth_hooks.tx_error &&
		dpaa_eth_hooks.tx_error(net_dev, fd, fqid) == DPAA_ETH_STOLEN)
		/* now the hook must ensure proper cleanup */
		return;
#endif
	percpu_priv->stats.tx_errors++;

	/* If we intended the buffers from this frame to go into the bpools
	 * when the FMan transmit was done, we need to put it in manually.
	 */
	if (fd->bpid != 0xff) {
		dpa_fd_release(net_dev, fd);
		return;
	}

	skb = _dpa_cleanup_tx_fd(priv, fd);
	dev_kfree_skb(skb);
}

/* Helper function to factor out frame validation logic on all Rx paths. Its
 * purpose is to extract from the Parse Results structure information about
 * the integrity of the frame, its checksum, the length of the parsed headers
 * and whether the frame is suitable for GRO.
 *
 * Assumes no parser errors, since any error frame is dropped before this
 * function is called.
 *
 * @skb		will have its ip_summed field overwritten;
 * @use_gro	will only be written with 0, if the frame is definitely not
 *		GRO-able; otherwise, it will be left unchanged;
 * @hdr_size	will be written with a safe value, at least the size of the
 *		headers' length.
 */
void __hot _dpa_process_parse_results(const fm_prs_result_t *parse_results,
				      const struct qm_fd *fd,
				      struct sk_buff *skb, int *use_gro)
{
	if (fd->status & FM_FD_STAT_L4CV) {
		/* The parser has run and performed L4 checksum validation.
		 * We know there were no parser errors (and implicitly no
		 * L4 csum error), otherwise we wouldn't be here.
		 */
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		/* Don't go through GRO for certain types of traffic that
		 * we know are not GRO-able, such as dgram-based protocols.
		 * In the worst-case scenarios, such as small-pkt terminating
		 * UDP, the extra GRO processing would be overkill.
		 *
		 * The only protocol the Parser supports that is also GRO-able
		 * is currently TCP.
		 */
		if (!fm_l4_frame_is_tcp(parse_results))
			*use_gro = 0;

		return;
	}

	/* We're here because either the parser didn't run or the L4 checksum
	 * was not verified. This may include the case of a UDP frame with
	 * checksum zero or an L4 proto other than TCP/UDP
	 */
	skb->ip_summed = CHECKSUM_NONE;

	/* Bypass GRO for unknown traffic or if no PCDs are applied */
	*use_gro = 0;
}

int dpaa_eth_poll(struct napi_struct *napi, int budget)
{
	struct dpa_napi_portal *np =
			container_of(napi, struct dpa_napi_portal, napi);

	int cleaned = qman_p_poll_dqrr(np->p, budget);

	if (cleaned < budget) {
		int tmp;
		napi_complete(napi);
		tmp = qman_p_irqsource_add(np->p, QM_PIRQ_DQRI);
		DPA_BUG_ON(tmp);
	}

	return cleaned;
}
EXPORT_SYMBOL(dpaa_eth_poll);

static void __hot _dpa_tx_conf(struct net_device	*net_dev,
			  const struct dpa_priv_s	*priv,
			  struct dpa_percpu_priv_s	*percpu_priv,
			  const struct qm_fd		*fd,
			  u32				 fqid)
{
	struct sk_buff	*skb;

	/* do we need the timestamp for the error frames? */

	if (unlikely(fd->status & FM_FD_STAT_TX_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_TX_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	/* hopefully we need not get the timestamp before the hook */
#ifdef CONFIG_FSL_DPAA_HOOKS
	if (dpaa_eth_hooks.tx_confirm && dpaa_eth_hooks.tx_confirm(net_dev,
		fd, fqid) == DPAA_ETH_STOLEN)
		/* it's the hook that must now perform cleanup */
		return;
#endif
	/* This might not perfectly reflect the reality, if the core dequeuing
	 * the Tx confirmation is different from the one that did the enqueue,
	 * but at least it'll show up in the total count.
	 */
	percpu_priv->tx_confirm++;

	skb = _dpa_cleanup_tx_fd(priv, fd);

	dev_kfree_skb(skb);
}

enum qman_cb_dqrr_result
priv_rx_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	int				*count_ptr;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	count_ptr = raw_cpu_ptr(priv->percpu_count);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	if (unlikely(dpaa_eth_refill_bpools(priv->dpa_bp, count_ptr)))
		/* Unable to refill the buffer pool due to insufficient
		 * system memory. Just release the frame back into the pool,
		 * otherwise we'll soon end up with an empty buffer pool.
		 */
		dpa_fd_release(net_dev, &dq->fd);
	else
		_dpa_rx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}


enum qman_cb_dqrr_result __hot
priv_rx_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	int                             *count_ptr;
	struct dpa_bp			*dpa_bp;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	dpa_bp = priv->dpa_bp;

	/* Trace the Rx fd */
	trace_dpa_rx_fd(net_dev, fq, &dq->fd);

	/* IRQ handler, non-migratable; safe to use raw_cpu_ptr here */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	count_ptr = raw_cpu_ptr(priv->percpu_count);

	if (unlikely(dpaa_eth_napi_schedule(percpu_priv, portal)))
		return qman_cb_dqrr_stop;

	/* Vale of plenty: make sure we didn't run out of buffers */

	if (unlikely(dpaa_eth_refill_bpools(dpa_bp, count_ptr)))
		/* Unable to refill the buffer pool due to insufficient
		 * system memory. Just release the frame back into the pool,
		 * otherwise we'll soon end up with an empty buffer pool.
		 */
		dpa_fd_release(net_dev, &dq->fd);
	else
		_dpa_rx(net_dev, portal, priv, percpu_priv, &dq->fd, fq->fqid,
			count_ptr);

	return qman_cb_dqrr_consume;
}

enum qman_cb_dqrr_result
priv_tx_conf_error_dqrr(struct qman_portal		*portal,
		      struct qman_fq			*fq,
		      const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	_dpa_tx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

enum qman_cb_dqrr_result __hot
priv_tx_conf_default_dqrr(struct qman_portal		*portal,
			struct qman_fq			*fq,
			const struct qm_dqrr_entry	*dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	/* Trace the fd */
	trace_dpa_tx_conf_fd(net_dev, fq, &dq->fd);

	/* Non-migratable context, safe to use raw_cpu_ptr */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	_dpa_tx_conf(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

void priv_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device	*net_dev;
	const struct dpa_priv_s	*priv;
	struct sk_buff *skb;
	struct dpa_percpu_priv_s	*percpu_priv;
	struct qm_fd fd = msg->ern.fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	/* Non-migratable context, safe to use raw_cpu_ptr */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);

	/* If we intended this buffer to go into the pool
	 * when the FM was done, we need to put it in
	 * manually.
	 */
	if (msg->ern.fd.bpid != 0xff) {
		dpa_fd_release(net_dev, &fd);
		return;
	}

	skb = _dpa_cleanup_tx_fd(priv, &fd);
	dev_kfree_skb_any(skb);
}

const struct dpa_fq_cbs_t private_fq_cbs = {
	.rx_defq = { .cb = { .dqrr = priv_rx_default_dqrr } },
	.tx_defq = { .cb = { .dqrr = priv_tx_conf_default_dqrr } },
	.rx_errq = { .cb = { .dqrr = priv_rx_error_dqrr } },
	.tx_errq = { .cb = { .dqrr = priv_tx_conf_error_dqrr } },
	.egress_ern = { .cb = { .ern = priv_ern } }
};
EXPORT_SYMBOL(private_fq_cbs);

static void dpaa_eth_napi_enable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++)
			napi_enable(&percpu_priv->np[j].napi);
	}
}

static void dpaa_eth_napi_disable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++)
			napi_disable(&percpu_priv->np[j].napi);
	}
}

static int __cold dpa_eth_priv_start(struct net_device *net_dev)
{
	int err;
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);

	dpaa_eth_napi_enable(priv);

	err = dpa_start(net_dev);
	if (err < 0)
		dpaa_eth_napi_disable(priv);

	return err;
}



static int __cold dpa_eth_priv_stop(struct net_device *net_dev)
{
	int _errno;
	struct dpa_priv_s *priv;

	_errno = dpa_stop(net_dev);
	/* Allow NAPI to consume any frame still in the Rx/TxConfirm
	 * ingress queues. This is to avoid a race between the current
	 * context and ksoftirqd which could leave NAPI disabled while
	 * in fact there's still Rx traffic to be processed.
	 */
	usleep_range(5000, 10000);

	priv = netdev_priv(net_dev);
	dpaa_eth_napi_disable(priv);

	return _errno;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void dpaa_eth_poll_controller(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv =
		raw_cpu_ptr(priv->percpu_priv);
	struct qman_portal *p;
	const struct qman_portal_config *pc;
	struct dpa_napi_portal *np;

	p = (struct qman_portal *)qman_get_affine_portal(smp_processor_id());
	pc = qman_p_get_portal_config(p);
	np = &percpu_priv->np[pc->index];

	qman_p_irqsource_remove(np->p, QM_PIRQ_DQRI);
	qman_p_poll_dqrr(np->p, np->napi.weight);
	qman_p_irqsource_add(np->p, QM_PIRQ_DQRI);
}
#endif

static const struct net_device_ops dpa_private_ops = {
	.ndo_open = dpa_eth_priv_start,
	.ndo_start_xmit = dpa_tx,
	.ndo_stop = dpa_eth_priv_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FMAN_PFC
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_set_rx_mode = dpa_set_rx_mode,
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_fix_features,
	.ndo_eth_ioctl = dpa_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = dpaa_eth_poll_controller,
#endif
};

static int dpa_private_napi_add(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		percpu_priv->np = devm_kzalloc(net_dev->dev.parent,
			qman_portal_max * sizeof(struct dpa_napi_portal),
			GFP_KERNEL);

		if (unlikely(percpu_priv->np == NULL)) {
			dev_err(net_dev->dev.parent, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}

		for (i = 0; i < qman_portal_max; i++)
			netif_napi_add(net_dev, &percpu_priv->np[i].napi,
					dpaa_eth_poll);
	}

	return 0;
}

void dpa_private_napi_del(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		if (percpu_priv->np) {
			for (i = 0; i < qman_portal_max; i++)
				netif_napi_del(&percpu_priv->np[i].napi);

			devm_kfree(net_dev->dev.parent, percpu_priv->np);
		}
	}
}
EXPORT_SYMBOL(dpa_private_napi_del);

static int dpa_private_netdev_init(struct net_device *net_dev)
{
	int i;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;
	const uint8_t *mac_addr;

	/* Although we access another CPU's private data here
	 * we do it at initialization so it is safe
	 */
	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		percpu_priv->net_dev = net_dev;
	}

	net_dev->netdev_ops = &dpa_private_ops;
	mac_addr = priv->mac_dev->addr;

	net_dev->mem_start = priv->mac_dev->res->start;
	net_dev->mem_end = priv->mac_dev->res->end;

	/* Configure the maximum MTU according to the FMan's MAXFRM */
	net_dev->min_mtu = ETH_MIN_MTU;
	net_dev->max_mtu = dpa_get_max_mtu();

	net_dev->hw_features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		NETIF_F_LLTX);

	/* Advertise S/G and HIGHDMA support for private interfaces */
	net_dev->hw_features |= NETIF_F_SG | NETIF_F_HIGHDMA;
	/* Recent kernels enable GSO automatically, if
	 * we declare NETIF_F_SG. For conformity, we'll
	 * still declare GSO explicitly.
	 */
	net_dev->features |= NETIF_F_GSO;

	/* Advertise GRO support */
	net_dev->features |= NETIF_F_GRO;

	return dpa_netdev_init(net_dev, mac_addr, tx_timeout);
}

static struct dpa_bp * __cold
dpa_priv_bp_probe(struct device *dev)
{
	struct dpa_bp *dpa_bp;

	dpa_bp = devm_kzalloc(dev, sizeof(*dpa_bp), GFP_KERNEL);
	if (unlikely(dpa_bp == NULL)) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return ERR_PTR(-ENOMEM);
	}

	dpa_bp->target_count = CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT;

	dpa_bp->free_buf_cb = _dpa_bp_free_pf;

	return dpa_bp;
}

/* Place all ingress FQs (Rx Default, Rx Error, PCD FQs) in a dedicated CGR.
 * We won't be sending congestion notifications to FMan; for now, we just use
 * this CGR to generate enqueue rejections to FMan in order to drop the frames
 * before they reach our ingress queues and eat up memory.
 */
static int dpaa_eth_priv_ingress_cgr_init(struct dpa_priv_s *priv)
{
	struct qm_mcc_initcgr initcgr;
	u32 cs_th;
	int err;

	err = qman_alloc_cgrid(&priv->ingress_cgr.cgrid);
	if (err < 0) {
		pr_err("Error %d allocating CGR ID\n", err);
		goto out_error;
	}

	/* Enable CS TD, but disable Congestion State Change Notifications. */
	memset(&initcgr, 0, sizeof(initcgr));
	initcgr.we_mask = QM_CGR_WE_CS_THRES;
	initcgr.cgr.cscn_en = QM_CGR_EN;
	cs_th = CONFIG_FSL_DPAA_INGRESS_CS_THRESHOLD;
	qm_cgr_cs_thres_set64(&initcgr.cgr.cs_thres, cs_th, 1);

	initcgr.we_mask |= QM_CGR_WE_CSTD_EN;
	initcgr.cgr.cstd_en = QM_CGR_EN;

	/* This is actually a hack, because this CGR will be associated with
	 * our affine SWP. However, we'll place our ingress FQs in it.
	 */
	err = qman_create_cgr(&priv->ingress_cgr, QMAN_CGR_FLAG_USE_INIT,
		&initcgr);
	if (err < 0) {
		pr_err("Error %d creating ingress CGR with ID %d\n", err,
			priv->ingress_cgr.cgrid);
		qman_release_cgrid(priv->ingress_cgr.cgrid);
		goto out_error;
	}
	pr_debug("Created ingress CGR %d for netdev with hwaddr %pM\n",
		 priv->ingress_cgr.cgrid, priv->mac_dev->addr);

	/* struct qman_cgr allows special cgrid values (i.e. outside the 0..255
	 * range), but we have no common initialization path between the
	 * different variants of the DPAA Eth driver, so we do it here rather
	 * than modifying every other variant than "private Eth".
	 */
	priv->use_ingress_cgr = true;

out_error:
	return err;
}

static int dpa_priv_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
		size_t count)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	int i;

	if (netif_msg_probe(priv))
		dev_dbg(net_dev->dev.parent,
			"Using private BM buffer pools\n");

	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;
		err = dpa_bp_alloc(&dpa_bp[i], net_dev->dev.parent);
		if (err < 0) {
			dpa_bp_free(priv);
			priv->dpa_bp = NULL;
			return err;
		}

		priv->dpa_bp = &dpa_bp[i];
	}

	dpa_priv_common_bpid = priv->dpa_bp->bpid;
	return 0;
}

static void dpa_priv_bp_seed(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	int i;

	/* Give each CPU an allotment of buffers */
	for_each_possible_cpu(i) {
		/* Although we access another CPU's counters here
		 * we do it at boot time so it is safe
		 */
		int *count_ptr = per_cpu_ptr(priv->percpu_count, i);

		dpaa_eth_refill_bpools(dpa_bp, count_ptr);
	}
}

static const struct of_device_id dpa_match[];

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
static int dpa_new_loop_id(void)
{
	static int if_id;

	return if_id++;
}
#endif

static int
dpaa_eth_priv_probe(struct platform_device *_of_dev)
{
	int err = 0, i, channel;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	size_t count = 1;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *priv = NULL;
	struct dpa_percpu_priv_s *percpu_priv;
	struct fm_port_fqs port_fqs;
	struct dpa_buffer_layout_s *buf_layout = NULL;
	struct mac_device *mac_dev;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/* Get the buffer pools assigned to this interface;
	 * run only once the default pool probing code
	 */
	dpa_bp = (dpa_bpid2pool(dpa_priv_common_bpid)) ? :
			dpa_priv_bp_probe(dev);
	if (IS_ERR(dpa_bp))
		return PTR_ERR(dpa_bp);

	/* Allocate this early, so we can store relevant information in
	 * the private area (needed by 1588 code in dpa_mac_probe)
	 */
	net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev_mq() failed\n");
		goto alloc_etherdev_mq_failed;
	}

	/* Do this here, so we can be verbose early */
	SET_NETDEV_DEV(net_dev, dev);
	dev_set_drvdata(dev, net_dev);

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	strcpy(priv->if_type, "private");

	priv->msg_enable = netif_msg_init(debug, -1);

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	priv->loop_id = dpa_new_loop_id();
	priv->loop_to = -1; /* disabled by default */
	dpa_loop_netdevs[priv->loop_id] = net_dev;
#endif

	mac_dev = dpa_mac_probe(_of_dev);
	if (IS_ERR(mac_dev) || !mac_dev) {
		err = PTR_ERR(mac_dev);
		goto mac_probe_failed;
	}

	/* We have physical ports, so we need to establish
	 * the buffer layout.
	 */
	buf_layout = devm_kzalloc(dev, 2 * sizeof(*buf_layout),
				  GFP_KERNEL);
	if (!buf_layout) {
		dev_err(dev, "devm_kzalloc() failed\n");
		goto alloc_failed;
	}
	dpa_set_buffers_layout(mac_dev, buf_layout);

	/* For private ports, need to compute the size of the default
	 * buffer pool, based on FMan port buffer layout;also update
	 * the maximum buffer size for private ports if necessary
	 */
	dpa_bp->size = dpa_bp_size(&buf_layout[RX]);

#ifdef CONFIG_FSL_DPAA_ETH_JUMBO_FRAME
	/* We only want to use jumbo frame optimization if we actually have
	 * L2 MAX FRM set for jumbo frames as well.
	 */
	if(fm_get_max_frm() < 9600)
		dev_warn(dev,
			"Invalid configuration: if jumbo frames support is on, FSL_FM_MAX_FRAME_SIZE should be set to 9600\n");
#endif

	INIT_LIST_HEAD(&priv->dpa_fq_list);

	memset(&port_fqs, 0, sizeof(port_fqs));

	err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list, &port_fqs, true, RX);
	if (!err)
		err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list,
				       &port_fqs, true, TX);

	if (err < 0)
		goto fq_probe_failed;

	/* bp init */

	err = dpa_priv_bp_create(net_dev, dpa_bp, count);

	if (err < 0)
		goto bp_create_failed;

	priv->mac_dev = mac_dev;

	channel = dpa_get_channel();

	if (channel < 0) {
		err = channel;
		goto get_channel_failed;
	}

	priv->channel = (uint16_t)channel;
	dpaa_eth_add_channel(priv->channel);

	dpa_fq_setup(priv, &private_fq_cbs, priv->mac_dev->port_dev[TX]);

	/* Create a congestion group for this netdev, with
	 * dynamically-allocated CGR ID.
	 * Must be executed after probing the MAC, but before
	 * assigning the egress FQs to the CGRs.
	 */
	err = dpaa_eth_cgr_init(priv);
	if (err < 0) {
		dev_err(dev, "Error initializing CGR\n");
		goto tx_cgr_init_failed;
	}
	err = dpaa_eth_priv_ingress_cgr_init(priv);
	if (err < 0) {
		dev_err(dev, "Error initializing ingress CGR\n");
		goto rx_cgr_init_failed;
	}

	/* Add the FQs to the interface, and make them active */
	err = dpa_fqs_init(dev,  &priv->dpa_fq_list, false);
	if (err < 0)
		goto fq_alloc_failed;

	priv->buf_layout = buf_layout;
	priv->tx_headroom = dpa_get_headroom(&priv->buf_layout[TX]);
	priv->rx_headroom = dpa_get_headroom(&priv->buf_layout[RX]);

	/* All real interfaces need their ports initialized */
	dpaa_eth_init_ports(mac_dev, dpa_bp, count, &port_fqs,
			buf_layout, dev);

#ifdef CONFIG_FMAN_PFC
	for (i = 0; i < CONFIG_FMAN_PFC_COS_COUNT; i++) {
		err = fm_port_set_pfc_priorities_mapping_to_qman_wq(
				mac_dev->port_dev[TX], i, i);
		if (unlikely(err != 0)) {
			dev_err(dev, "Error maping PFC %u to WQ %u\n", i, i);
			goto pfc_mapping_failed;
		}
	}
#endif

	priv->percpu_priv = devm_alloc_percpu(dev, *priv->percpu_priv);

	if (priv->percpu_priv == NULL) {
		dev_err(dev, "devm_alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		memset(percpu_priv, 0, sizeof(*percpu_priv));
	}

	priv->percpu_count = devm_alloc_percpu(dev, *priv->percpu_count);
	if (!priv->percpu_count) {
		dev_err(dev, "devm_alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}

	for_each_possible_cpu(i) {
		int *percpu_count = per_cpu_ptr(priv->percpu_count, i);
		*percpu_count = 0;
	}

	/* Initialize NAPI */
	err = dpa_private_napi_add(net_dev);

	if (err < 0)
		goto napi_add_failed;

	err = dpa_private_netdev_init(net_dev);

	dpa_priv_bp_seed(net_dev);

	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

#ifdef CONFIG_PM
	device_set_wakeup_capable(dev, true);
#endif

	pr_info("fsl_dpa: Probed interface %s\n", net_dev->name);

	return 0;

netdev_init_failed:
napi_add_failed:
	dpa_private_napi_del(net_dev);
alloc_percpu_failed:
#ifdef CONFIG_FMAN_PFC
pfc_mapping_failed:
#endif
	dpa_fq_free(dev, &priv->dpa_fq_list);
fq_alloc_failed:
	qman_delete_cgr_safe(&priv->ingress_cgr);
	qman_release_cgrid(priv->ingress_cgr.cgrid);
rx_cgr_init_failed:
	qman_delete_cgr_safe(&priv->cgr_data.cgr);
	qman_release_cgrid(priv->cgr_data.cgr.cgrid);
tx_cgr_init_failed:
get_channel_failed:
	dpa_bp_free(priv);
bp_create_failed:
fq_probe_failed:
alloc_failed:
mac_probe_failed:
	dev_set_drvdata(dev, NULL);
	free_netdev(net_dev);
alloc_etherdev_mq_failed:
	if (atomic_read(&dpa_bp->refs) == 0)
		devm_kfree(dev, dpa_bp);

	return err;
}

static const struct of_device_id dpa_match[] = {
	{
		.compatible	= "fsl,dpa-ethernet"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_match);

static struct platform_driver dpa_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= dpa_match,
		.owner		= THIS_MODULE,
		.pm		= DPAA_PM_OPS,
	},
	.probe		= dpaa_eth_priv_probe,
	.remove		= dpa_remove
};

static int __init __cold dpa_load(void)
{
	int	 _errno;

	pr_info(DPA_DESCRIPTION "\n");

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	dpa_debugfs_module_init();
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */

	/* initialise dpaa_eth mirror values */
	dpa_rx_extra_headroom = fm_get_rx_extra_headroom();
	dpa_max_frm = fm_get_max_frm();
	dpa_num_cpus = num_possible_cpus();

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	memset(dpa_loop_netdevs, 0, sizeof(dpa_loop_netdevs));
#endif

	_errno = platform_driver_register(&dpa_driver);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);

#ifdef CONFIG_FSL_DPAA_CEETM
	_errno = register_qdisc(&ceetm_qdisc_ops);
	if (unlikely(_errno))
		pr_err(KBUILD_MODNAME
		       ": %s:%hu:%s(): register_qdisc() = %d\n",
		       KBUILD_BASENAME ".c", __LINE__, __func__, _errno);
#endif

	return _errno;
}
module_init(dpa_load);

static void __exit __cold dpa_unload(void)
{
	pr_debug(KBUILD_MODNAME ": -> %s:%s()\n",
		KBUILD_BASENAME".c", __func__);

#ifdef CONFIG_FSL_DPAA_CEETM
	unregister_qdisc(&ceetm_qdisc_ops);
#endif

	platform_driver_unregister(&dpa_driver);

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	dpa_debugfs_module_exit();
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */

	/* Only one channel is used and needs to be relased after all
	 * interfaces are removed
	 */
	dpa_release_channel();

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
}
module_exit(dpa_unload);
