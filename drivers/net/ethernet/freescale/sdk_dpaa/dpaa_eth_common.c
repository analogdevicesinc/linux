/* Copyright 2008-2013 Freescale Semiconductor, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/highmem.h>
#include <linux/sort.h>
#include <linux/fsl_qman.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/if_vlan.h>	/* vlan_eth_hdr */
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#ifdef CONFIG_FSL_DPAA_1588
#include "dpaa_1588.h"
#endif
#ifdef CONFIG_FSL_DPAA_DBG_LOOP
#include "dpaa_debugfs.h"
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */
#include "mac.h"

/* Size in bytes of the FQ taildrop threshold */
#define DPA_FQ_TD		0x200000

static struct dpa_bp *dpa_bp_array[64];

int dpa_max_frm;
EXPORT_SYMBOL(dpa_max_frm);

int dpa_rx_extra_headroom;
EXPORT_SYMBOL(dpa_rx_extra_headroom);

int dpa_num_cpus = NR_CPUS;

static const struct fqid_cell tx_confirm_fqids[] = {
	{0, DPAA_ETH_TX_QUEUES}
};

static struct fqid_cell default_fqids[][3] = {
	[RX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_RX_QUEUES} },
	[TX] = { {0, 1}, {0, 1}, {0, DPAA_ETH_TX_QUEUES} }
};

static const char fsl_qman_frame_queues[][25] = {
	[RX] = "fsl,qman-frame-queues-rx",
	[TX] = "fsl,qman-frame-queues-tx"
};
#ifdef CONFIG_FSL_DPAA_HOOKS
/* A set of callbacks for hooking into the fastpath at different points. */
struct dpaa_eth_hooks_s dpaa_eth_hooks;
EXPORT_SYMBOL(dpaa_eth_hooks);
/* This function should only be called on the probe paths, since it makes no
 * effort to guarantee consistency of the destination hooks structure.
 */
void fsl_dpaa_eth_set_hooks(struct dpaa_eth_hooks_s *hooks)
{
	if (hooks)
		dpaa_eth_hooks = *hooks;
	else
		pr_err("NULL pointer to hooks!\n");
}
EXPORT_SYMBOL(fsl_dpaa_eth_set_hooks);
#endif

int dpa_netdev_init(struct net_device *net_dev,
		    const uint8_t *mac_addr,
		    uint16_t tx_timeout)
{
	int err;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;

	net_dev->priv_flags |= IFF_LIVE_ADDR_CHANGE;

	net_dev->features |= net_dev->hw_features;
	net_dev->vlan_features = net_dev->features;

	memcpy(net_dev->perm_addr, mac_addr, net_dev->addr_len);
	memcpy(net_dev->dev_addr, mac_addr, net_dev->addr_len);

	net_dev->ethtool_ops = &dpa_ethtool_ops;

	net_dev->needed_headroom = priv->tx_headroom;
	net_dev->watchdog_timeo = msecs_to_jiffies(tx_timeout);

	err = register_netdev(net_dev);
	if (err < 0) {
		dev_err(dev, "register_netdev() = %d\n", err);
		return err;
	}

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	/* create debugfs entry for this net_device */
	err = dpa_netdev_debugfs_create(net_dev);
	if (err) {
		unregister_netdev(net_dev);
		return err;
	}
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */

	return 0;
}
EXPORT_SYMBOL(dpa_netdev_init);

int __cold dpa_start(struct net_device *net_dev)
{
	int err, i;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	err = mac_dev->init_phy(net_dev, priv->mac_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			netdev_err(net_dev, "init_phy() = %d\n", err);
		return err;
	}

	for_each_port_device(i, mac_dev->port_dev) {
		err = fm_port_enable(mac_dev->port_dev[i]);
		if (err)
			goto mac_start_failed;
	}

	err = priv->mac_dev->start(mac_dev);
	if (err < 0) {
		if (netif_msg_ifup(priv))
			netdev_err(net_dev, "mac_dev->start() = %d\n", err);
		goto mac_start_failed;
	}

	netif_tx_start_all_queues(net_dev);

	return 0;

mac_start_failed:
	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

	return err;
}
EXPORT_SYMBOL(dpa_start);

int __cold dpa_stop(struct net_device *net_dev)
{
	int _errno, i, err;
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	netif_tx_stop_all_queues(net_dev);
	/* Allow the Fman (Tx) port to process in-flight frames before we
	 * try switching it off.
	 */
	usleep_range(5000, 10000);

	_errno = mac_dev->stop(mac_dev);
	if (unlikely(_errno < 0))
		if (netif_msg_ifdown(priv))
			netdev_err(net_dev, "mac_dev->stop() = %d\n",
					_errno);

	for_each_port_device(i, mac_dev->port_dev) {
		err = fm_port_disable(mac_dev->port_dev[i]);
		_errno = err ? err : _errno;
	}

	if (mac_dev->phy_dev)
		phy_disconnect(mac_dev->phy_dev);
	mac_dev->phy_dev = NULL;

	return _errno;
}
EXPORT_SYMBOL(dpa_stop);

void __cold dpa_timeout(struct net_device *net_dev)
{
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;

	priv = netdev_priv(net_dev);
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	if (netif_msg_timer(priv))
		netdev_crit(net_dev, "Transmit timeout!\n");

	percpu_priv->stats.tx_errors++;
}
EXPORT_SYMBOL(dpa_timeout);

/* net_device */

/**
 * @param net_dev the device for which statistics are calculated
 * @param stats the function fills this structure with the device's statistics
 * @return the address of the structure containing the statistics
 *
 * Calculates the statistics for the given device by adding the statistics
 * collected by each CPU.
 */
void __cold
dpa_get_stats64(struct net_device *net_dev,
		struct rtnl_link_stats64 *stats)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	u64 *cpustats;
	u64 *netstats = (u64 *)stats;
	int i, j;
	struct dpa_percpu_priv_s	*percpu_priv;
	int numstats = sizeof(struct rtnl_link_stats64) / sizeof(u64);

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		cpustats = (u64 *)&percpu_priv->stats;

		for (j = 0; j < numstats; j++)
			netstats[j] += cpustats[j];
	}
}
EXPORT_SYMBOL(dpa_get_stats64);

/* .ndo_init callback */
int dpa_ndo_init(struct net_device *net_dev)
{
	/* If fsl_fm_max_frm is set to a higher value than the all-common 1500,
	 * we choose conservatively and let the user explicitly set a higher
	 * MTU via ifconfig. Otherwise, the user may end up with different MTUs
	 * in the same LAN.
	 * If on the other hand fsl_fm_max_frm has been chosen below 1500,
	 * start with the maximum allowed.
	 */
	int init_mtu = min(dpa_get_max_mtu(), ETH_DATA_LEN);

	pr_debug("Setting initial MTU on net device: %d\n", init_mtu);
	net_dev->mtu = init_mtu;

	return 0;
}
EXPORT_SYMBOL(dpa_ndo_init);

int dpa_set_features(struct net_device *dev, netdev_features_t features)
{
	/* Not much to do here for now */
	dev->features = features;
	return 0;
}
EXPORT_SYMBOL(dpa_set_features);

netdev_features_t dpa_fix_features(struct net_device *dev,
		netdev_features_t features)
{
	netdev_features_t unsupported_features = 0;

	/* In theory we should never be requested to enable features that
	 * we didn't set in netdev->features and netdev->hw_features at probe
	 * time, but double check just to be on the safe side.
	 * We don't support enabling Rx csum through ethtool yet
	 */
	unsupported_features |= NETIF_F_RXCSUM;

	features &= ~unsupported_features;

	return features;
}
EXPORT_SYMBOL(dpa_fix_features);

#ifdef CONFIG_FSL_DPAA_TS
u64 dpa_get_timestamp_ns(const struct dpa_priv_s *priv, enum port_type rx_tx,
			const void *data)
{
	u64 *ts;

	ts = fm_port_get_buffer_time_stamp(priv->mac_dev->port_dev[rx_tx],
					   data);

	if (!ts || *ts == 0)
		return 0;

	be64_to_cpus(ts);

	return *ts;
}

int dpa_get_ts(const struct dpa_priv_s *priv, enum port_type rx_tx,
	struct skb_shared_hwtstamps *shhwtstamps, const void *data)
{
	u64 ns;

	ns = dpa_get_timestamp_ns(priv, rx_tx, data);

	if (ns == 0)
		return -EINVAL;

	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	shhwtstamps->hwtstamp = ns_to_ktime(ns);

	return 0;
}

static void dpa_ts_tx_enable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->ptp_enable)
		mac_dev->ptp_enable(mac_dev->get_mac_handle(mac_dev));

	priv->ts_tx_en = true;
}

static void dpa_ts_tx_disable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);

#if 0
/* the RTC might be needed by the Rx Ts, cannot disable here
 * no separate ptp_disable API for Rx/Tx, cannot disable here
 */
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(get_fm_handle(dev));

	if (mac_dev->ptp_disable)
		mac_dev->ptp_disable(mac_dev->get_mac_handle(mac_dev));
#endif

	priv->ts_tx_en = false;
}

static void dpa_ts_rx_enable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->ptp_enable)
		mac_dev->ptp_enable(mac_dev->get_mac_handle(mac_dev));

	priv->ts_rx_en = true;
}

static void dpa_ts_rx_disable(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);

#if 0
/* the RTC might be needed by the Tx Ts, cannot disable here
 * no separate ptp_disable API for Rx/Tx, cannot disable here
 */
	struct mac_device *mac_dev = priv->mac_dev;

	if (mac_dev->fm_rtc_disable)
		mac_dev->fm_rtc_disable(get_fm_handle(dev));

	if (mac_dev->ptp_disable)
		mac_dev->ptp_disable(mac_dev->get_mac_handle(mac_dev));
#endif

	priv->ts_rx_en = false;
}

static int dpa_ts_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct hwtstamp_config config;

	if (copy_from_user(&config, rq->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		dpa_ts_tx_disable(dev);
		break;
	case HWTSTAMP_TX_ON:
		dpa_ts_tx_enable(dev);
		break;
	default:
		return -ERANGE;
	}

	if (config.rx_filter == HWTSTAMP_FILTER_NONE)
		dpa_ts_rx_disable(dev);
	else {
		dpa_ts_rx_enable(dev);
		/* TS is set for all frame types, not only those requested */
		config.rx_filter = HWTSTAMP_FILTER_ALL;
	}

	return copy_to_user(rq->ifr_data, &config, sizeof(config)) ?
			-EFAULT : 0;
}
#endif /* CONFIG_FSL_DPAA_TS */

int dpa_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#ifdef CONFIG_FSL_DPAA_1588
	struct dpa_priv_s *priv = netdev_priv(dev);
#endif
	int ret = -EINVAL;

	if (!netif_running(dev))
		return -EINVAL;

	if (cmd == SIOCGMIIREG) {
		if (!dev->phydev)
			ret = -EINVAL;
		else
			ret = phy_mii_ioctl(dev->phydev, rq, cmd);
	}

#ifdef CONFIG_FSL_DPAA_TS
	if (cmd == SIOCSHWTSTAMP)
		return dpa_ts_ioctl(dev, rq, cmd);
#endif /* CONFIG_FSL_DPAA_TS */

#ifdef CONFIG_FSL_DPAA_1588
	if ((cmd >= PTP_ENBL_TXTS_IOCTL) && (cmd <= PTP_CLEANUP_TS)) {
		if (priv->tsu && priv->tsu->valid)
			ret = dpa_ioctl_1588(dev, rq, cmd);
		else
			ret = -ENODEV;
	}
#endif

	return ret;
}
EXPORT_SYMBOL(dpa_ioctl);

int __cold dpa_remove(struct platform_device *of_dev)
{
	int			err;
	struct device		*dev;
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;

	dev = &of_dev->dev;
	net_dev = dev_get_drvdata(dev);

	priv = netdev_priv(net_dev);

	dpaa_eth_sysfs_remove(dev);

	dev_set_drvdata(dev, NULL);
	unregister_netdev(net_dev);

	err = dpa_fq_free(dev, &priv->dpa_fq_list);

	qman_delete_cgr_safe(&priv->ingress_cgr);
	qman_release_cgrid(priv->ingress_cgr.cgrid);
	qman_delete_cgr_safe(&priv->cgr_data.cgr);
	qman_release_cgrid(priv->cgr_data.cgr.cgrid);

	dpa_private_napi_del(net_dev);

	dpa_bp_free(priv);

	if (priv->buf_layout)
		devm_kfree(dev, priv->buf_layout);

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	/* remove debugfs entry for this net_device */
	dpa_netdev_debugfs_remove(net_dev);
#endif /* CONFIG_FSL_DPAA_DBG_LOOP */

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid)
		dpa_ptp_cleanup(priv);
#endif

	free_netdev(net_dev);

	return err;
}
EXPORT_SYMBOL(dpa_remove);

struct mac_device * __cold __must_check
__attribute__((nonnull))
dpa_mac_probe(struct platform_device *_of_dev)
{
	struct device		*dpa_dev, *dev;
	struct device_node	*mac_node;
	struct platform_device	*of_dev;
	struct mac_device	*mac_dev;
#ifdef CONFIG_FSL_DPAA_1588
	int			 lenp;
	const phandle		*phandle_prop;
	struct net_device	*net_dev = NULL;
	struct dpa_priv_s	*priv = NULL;
	struct device_node	*timer_node;
#endif
	dpa_dev = &_of_dev->dev;

	mac_node = of_parse_phandle(_of_dev->dev.of_node, "fsl,fman-mac", 0);
	if (unlikely(mac_node == NULL)) {
		dev_err(dpa_dev, "Cannot find MAC device device tree node\n");
		return ERR_PTR(-EFAULT);
	}

	of_dev = of_find_device_by_node(mac_node);
	if (unlikely(of_dev == NULL)) {
		dev_err(dpa_dev, "of_find_device_by_node(%s) failed\n",
				mac_node->full_name);
		of_node_put(mac_node);
		return ERR_PTR(-EINVAL);
	}
	of_node_put(mac_node);

	dev = &of_dev->dev;

	mac_dev = dev_get_drvdata(dev);
	if (unlikely(mac_dev == NULL)) {
		dev_err(dpa_dev, "dev_get_drvdata(%s) failed\n",
				dev_name(dev));
		return ERR_PTR(-EINVAL);
	}

#ifdef CONFIG_FSL_DPAA_1588
	phandle_prop = of_get_property(mac_node, "ptp-timer", &lenp);
	if (phandle_prop && ((mac_dev->phy_if != PHY_INTERFACE_MODE_SGMII) ||
			((mac_dev->phy_if == PHY_INTERFACE_MODE_SGMII) &&
			 (mac_dev->speed == SPEED_1000)))) {
		timer_node = of_find_node_by_phandle(*phandle_prop);
		if (timer_node)
			net_dev = dev_get_drvdata(dpa_dev);
		if (timer_node && net_dev) {
			priv = netdev_priv(net_dev);
			if (!dpa_ptp_init(priv))
				dev_info(dev, "%s: ptp 1588 is initialized.\n",
						mac_node->full_name);
		}
	}
#endif

	return mac_dev;
}
EXPORT_SYMBOL(dpa_mac_probe);

int dpa_set_mac_address(struct net_device *net_dev, void *addr)
{
	const struct dpa_priv_s	*priv;
	int			 _errno;
	struct mac_device	*mac_dev;

	priv = netdev_priv(net_dev);

	_errno = eth_mac_addr(net_dev, addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev,
				       "eth_mac_addr() = %d\n",
				       _errno);
		return _errno;
	}

	mac_dev = priv->mac_dev;

	_errno = mac_dev->change_addr(mac_dev->get_mac_handle(mac_dev),
			net_dev->dev_addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev,
				       "mac_dev->change_addr() = %d\n",
				       _errno);
		return _errno;
	}

	return 0;
}
EXPORT_SYMBOL(dpa_set_mac_address);

void dpa_set_rx_mode(struct net_device *net_dev)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (!!(net_dev->flags & IFF_PROMISC) != priv->mac_dev->promisc) {
		priv->mac_dev->promisc = !priv->mac_dev->promisc;
		_errno = priv->mac_dev->set_promisc(
				priv->mac_dev->get_mac_handle(priv->mac_dev),
				priv->mac_dev->promisc);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			netdev_err(net_dev,
					   "mac_dev->set_promisc() = %d\n",
					   _errno);
	}

	_errno = priv->mac_dev->set_multi(net_dev, priv->mac_dev);
	if (unlikely(_errno < 0) && netif_msg_drv(priv))
		netdev_err(net_dev, "mac_dev->set_multi() = %d\n", _errno);
}
EXPORT_SYMBOL(dpa_set_rx_mode);

void dpa_set_buffers_layout(struct mac_device *mac_dev,
		struct dpa_buffer_layout_s *layout)
{
	struct fm_port_params params;

	/* Rx */
	layout[RX].priv_data_size = (uint16_t)DPA_RX_PRIV_DATA_SIZE;
	layout[RX].parse_results = true;
	layout[RX].hash_results = true;
#ifdef CONFIG_FSL_DPAA_TS
	layout[RX].time_stamp = true;
#endif
	fm_port_get_buff_layout_ext_params(mac_dev->port_dev[RX], &params);
	layout[RX].manip_extra_space = params.manip_extra_space;
	/* a value of zero for data alignment means "don't care", so align to
	 * a non-zero value to prevent FMD from using its own default
	 */
	layout[RX].data_align = params.data_align ? : DPA_FD_DATA_ALIGNMENT;

	/* Tx */
	layout[TX].priv_data_size = DPA_TX_PRIV_DATA_SIZE;
	layout[TX].parse_results = true;
	layout[TX].hash_results = true;
#ifdef CONFIG_FSL_DPAA_TS
	layout[TX].time_stamp = true;
#endif
	fm_port_get_buff_layout_ext_params(mac_dev->port_dev[TX], &params);
	layout[TX].manip_extra_space = params.manip_extra_space;
	layout[TX].data_align = params.data_align ? : DPA_FD_DATA_ALIGNMENT;
}
EXPORT_SYMBOL(dpa_set_buffers_layout);

int __attribute__((nonnull))
dpa_bp_alloc(struct dpa_bp *dpa_bp, struct device *dev)
{
	int err;
	struct bman_pool_params	 bp_params;

	if (dpa_bp->size == 0 || dpa_bp->config_count == 0) {
		pr_err("Buffer pool is not properly initialized! Missing size or initial number of buffers");
		return -EINVAL;
	}

	memset(&bp_params, 0, sizeof(struct bman_pool_params));
#ifdef CONFIG_FMAN_PFC
	bp_params.flags = BMAN_POOL_FLAG_THRESH;
	bp_params.thresholds[0] = bp_params.thresholds[2] =
			CONFIG_FSL_DPAA_ETH_REFILL_THRESHOLD;
	bp_params.thresholds[1] = bp_params.thresholds[3] =
			CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT;
#endif

	/* If the pool is already specified, we only create one per bpid */
	if (dpa_bpid2pool_use(dpa_bp->bpid))
		return 0;

	if (dpa_bp->bpid == 0)
		bp_params.flags |= BMAN_POOL_FLAG_DYNAMIC_BPID;
	else
		bp_params.bpid = dpa_bp->bpid;

	dpa_bp->pool = bman_new_pool(&bp_params);
	if (unlikely(dpa_bp->pool == NULL)) {
		pr_err("bman_new_pool() failed\n");
		return -ENODEV;
	}

	dpa_bp->bpid = (uint8_t)bman_get_params(dpa_bp->pool)->bpid;

	err = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (err) {
		pr_err("dma_coerce_mask_and_coherent() failed\n");
		goto bman_free_pool;
	}

	dpa_bp->dev = dev;

	if (dpa_bp->seed_cb) {
		err = dpa_bp->seed_cb(dpa_bp);
		if (err)
			goto bman_free_pool;
	}

	dpa_bpid2pool_map(dpa_bp->bpid, dpa_bp);

	return 0;

bman_free_pool:
	bman_free_pool(dpa_bp->pool);

	return err;
}
EXPORT_SYMBOL(dpa_bp_alloc);

void dpa_bp_drain(struct dpa_bp *bp)
{
	int ret, num = 8;

	do {
		struct bm_buffer bmb[8];
		int i;

		ret = bman_acquire(bp->pool, bmb, num, 0);
		if (ret < 0) {
			if (num == 8) {
				/* we have less than 8 buffers left;
				 * drain them one by one
				 */
				num = 1;
				ret = 1;
				continue;
			} else {
				/* Pool is fully drained */
				break;
			}
		}

		for (i = 0; i < num; i++) {
			dma_addr_t addr = bm_buf_addr(&bmb[i]);

			dma_unmap_single(bp->dev, addr, bp->size,
					DMA_BIDIRECTIONAL);

			bp->free_buf_cb(phys_to_virt(addr));
		}
	} while (ret > 0);
}
EXPORT_SYMBOL(dpa_bp_drain);

static void __cold __attribute__((nonnull))
_dpa_bp_free(struct dpa_bp *dpa_bp)
{
	struct dpa_bp *bp = dpa_bpid2pool(dpa_bp->bpid);

	/* the mapping between bpid and dpa_bp is done very late in the
	 * allocation procedure; if something failed before the mapping, the bp
	 * was not configured, therefore we don't need the below instructions
	 */
	if (!bp)
		return;

	if (!atomic_dec_and_test(&bp->refs))
		return;

	if (bp->free_buf_cb)
		dpa_bp_drain(bp);

	dpa_bp_array[bp->bpid] = NULL;
	bman_free_pool(bp->pool);
}

void __cold __attribute__((nonnull))
dpa_bp_free(struct dpa_priv_s *priv)
{
	int i;

	if (priv->dpa_bp)
		for (i = 0; i < priv->bp_count; i++)
			_dpa_bp_free(&priv->dpa_bp[i]);
}
EXPORT_SYMBOL(dpa_bp_free);

struct dpa_bp *dpa_bpid2pool(int bpid)
{
	return dpa_bp_array[bpid];
}
EXPORT_SYMBOL(dpa_bpid2pool);

void dpa_bpid2pool_map(int bpid, struct dpa_bp *dpa_bp)
{
	dpa_bp_array[bpid] = dpa_bp;
	atomic_set(&dpa_bp->refs, 1);
}

bool dpa_bpid2pool_use(int bpid)
{
	if (dpa_bpid2pool(bpid)) {
		atomic_inc(&dpa_bp_array[bpid]->refs);
		return true;
	}

	return false;
}

#ifdef CONFIG_FMAN_PFC
u16 dpa_select_queue(struct net_device *net_dev, struct sk_buff *skb,
		     struct net_device *sb_dev,
		     select_queue_fallback_t fallback)
{
	return dpa_get_queue_mapping(skb);
}
#endif

struct dpa_fq *dpa_fq_alloc(struct device *dev,
			    u32 fq_start,
			    u32 fq_count,
			    struct list_head *list,
			    enum dpa_fq_type fq_type)
{
	int i;
	struct dpa_fq *dpa_fq;

	dpa_fq = devm_kzalloc(dev, sizeof(*dpa_fq) * fq_count, GFP_KERNEL);
	if (dpa_fq == NULL)
		return NULL;

	for (i = 0; i < fq_count; i++) {
		dpa_fq[i].fq_type = fq_type;
		if (fq_type == FQ_TYPE_RX_PCD_HI_PRIO)
			dpa_fq[i].fqid = fq_start ?
					 DPAA_ETH_FQ_DELTA + fq_start + i : 0;
		else
			dpa_fq[i].fqid = fq_start ? fq_start + i : 0;

		list_add_tail(&dpa_fq[i].list, list);
	}

#ifdef CONFIG_FMAN_PFC
	if (fq_type == FQ_TYPE_TX)
		for (i = 0; i < fq_count; i++)
			dpa_fq[i].wq = i / dpa_num_cpus;
	else
#endif
		for (i = 0; i < fq_count; i++)
			_dpa_assign_wq(dpa_fq + i);

	return dpa_fq;
}
EXPORT_SYMBOL(dpa_fq_alloc);

/* Probing of FQs for MACful ports */
int dpa_fq_probe_mac(struct device *dev, struct list_head *list,
			    struct fm_port_fqs *port_fqs,
			    bool alloc_tx_conf_fqs,
			    enum port_type ptype)
{
	struct fqid_cell *fqids = NULL;
	const void *fqids_off = NULL;
	struct dpa_fq *dpa_fq = NULL;
	struct device_node *np = dev->of_node;
	int num_ranges;
	int i, lenp;

	if (ptype == TX && alloc_tx_conf_fqs) {
		if (!dpa_fq_alloc(dev, tx_confirm_fqids->start,
				  tx_confirm_fqids->count, list,
				  FQ_TYPE_TX_CONF_MQ))
			goto fq_alloc_failed;
	}

	fqids_off = of_get_property(np, fsl_qman_frame_queues[ptype], &lenp);
	if (fqids_off == NULL) {
		/* No dts definition, so use the defaults. */
		fqids = default_fqids[ptype];
		num_ranges = 3;
	} else {
		num_ranges = lenp / sizeof(*fqids);

		fqids = devm_kzalloc(dev, sizeof(*fqids) * num_ranges,
				GFP_KERNEL);
		if (fqids == NULL)
			goto fqids_alloc_failed;

		/* convert to CPU endianess */
		for (i = 0; i < num_ranges; i++) {
			fqids[i].start = be32_to_cpup(fqids_off +
					i * sizeof(*fqids));
			fqids[i].count = be32_to_cpup(fqids_off +
					i * sizeof(*fqids) + sizeof(__be32));
		}
	}

	for (i = 0; i < num_ranges; i++) {
		switch (i) {
		case 0:
			/* The first queue is the error queue */
			if (fqids[i].count != 1)
				goto invalid_error_queue;

			dpa_fq = dpa_fq_alloc(dev, fqids[i].start,
					      fqids[i].count, list,
					      ptype == RX ?
						FQ_TYPE_RX_ERROR :
						FQ_TYPE_TX_ERROR);
			if (dpa_fq == NULL)
				goto fq_alloc_failed;

			if (ptype == RX)
				port_fqs->rx_errq = &dpa_fq[0];
			else
				port_fqs->tx_errq = &dpa_fq[0];
			break;
		case 1:
			/* the second queue is the default queue */
			if (fqids[i].count != 1)
				goto invalid_default_queue;

			dpa_fq = dpa_fq_alloc(dev, fqids[i].start,
					      fqids[i].count, list,
					      ptype == RX ?
						FQ_TYPE_RX_DEFAULT :
						FQ_TYPE_TX_CONFIRM);
			if (dpa_fq == NULL)
				goto fq_alloc_failed;

			if (ptype == RX)
				port_fqs->rx_defq = &dpa_fq[0];
			else
				port_fqs->tx_defq = &dpa_fq[0];
			break;
		default:
			/* all subsequent queues are either RX* PCD or Tx */
			if (ptype == RX) {
				if (!dpa_fq_alloc(dev, fqids[i].start,
						  fqids[i].count, list,
						  FQ_TYPE_RX_PCD) ||
				    !dpa_fq_alloc(dev, fqids[i].start,
						  fqids[i].count, list,
						  FQ_TYPE_RX_PCD_HI_PRIO))
					goto fq_alloc_failed;
			} else {
				if (!dpa_fq_alloc(dev, fqids[i].start,
						  fqids[i].count, list,
						  FQ_TYPE_TX))
					goto fq_alloc_failed;
			}
			break;
		}
	}

	return 0;

fq_alloc_failed:
fqids_alloc_failed:
	dev_err(dev, "Cannot allocate memory for frame queues\n");
	return -ENOMEM;

invalid_default_queue:
invalid_error_queue:
	dev_err(dev, "Too many default or error queues\n");
	return -EINVAL;
}
EXPORT_SYMBOL(dpa_fq_probe_mac);

static u32 rx_pool_channel;
static DEFINE_SPINLOCK(rx_pool_channel_init);

int dpa_get_channel(void)
{
	spin_lock(&rx_pool_channel_init);
	if (!rx_pool_channel) {
		u32 pool;
		int ret = qman_alloc_pool(&pool);
		if (!ret)
			rx_pool_channel = pool;
	}
	spin_unlock(&rx_pool_channel_init);
	if (!rx_pool_channel)
		return -ENOMEM;
	return rx_pool_channel;
}
EXPORT_SYMBOL(dpa_get_channel);

void dpa_release_channel(void)
{
	qman_release_pool(rx_pool_channel);
}
EXPORT_SYMBOL(dpa_release_channel);

void dpaa_eth_add_channel(u16 channel)
{
	const cpumask_t *cpus = qman_affine_cpus();
	u32 pool = QM_SDQCR_CHANNELS_POOL_CONV(channel);
	int cpu;
	struct qman_portal *portal;

	for_each_cpu(cpu, cpus) {
		portal = (struct qman_portal *)qman_get_affine_portal(cpu);
		qman_p_static_dequeue_add(portal, pool);
	}
}
EXPORT_SYMBOL(dpaa_eth_add_channel);

/**
 * Congestion group state change notification callback.
 * Stops the device's egress queues while they are congested and
 * wakes them upon exiting congested state.
 * Also updates some CGR-related stats.
 */
static void dpaa_eth_cgscn(struct qman_portal *qm, struct qman_cgr *cgr,

	int congested)
{
	struct dpa_priv_s *priv = (struct dpa_priv_s *)container_of(cgr,
		struct dpa_priv_s, cgr_data.cgr);

	if (congested) {
		priv->cgr_data.congestion_start_jiffies = jiffies;
		netif_tx_stop_all_queues(priv->net_dev);
		priv->cgr_data.cgr_congested_count++;
	} else {
		priv->cgr_data.congested_jiffies +=
			(jiffies - priv->cgr_data.congestion_start_jiffies);
		netif_tx_wake_all_queues(priv->net_dev);
	}
}

int dpaa_eth_cgr_init(struct dpa_priv_s *priv)
{
	struct qm_mcc_initcgr initcgr;
	u32 cs_th;
	int err;

	err = qman_alloc_cgrid(&priv->cgr_data.cgr.cgrid);
	if (err < 0) {
		pr_err("Error %d allocating CGR ID\n", err);
		goto out_error;
	}
	priv->cgr_data.cgr.cb = dpaa_eth_cgscn;

	/* Enable Congestion State Change Notifications and CS taildrop */
	initcgr.we_mask = QM_CGR_WE_CSCN_EN | QM_CGR_WE_CS_THRES;
	initcgr.cgr.cscn_en = QM_CGR_EN;

	/* Set different thresholds based on the MAC speed.
	 * TODO: this may turn suboptimal if the MAC is reconfigured at a speed
	 * lower than its max, e.g. if a dTSEC later negotiates a 100Mbps link.
	 * In such cases, we ought to reconfigure the threshold, too.
	 */
	if (priv->mac_dev->if_support & SUPPORTED_10000baseT_Full)
		cs_th = CONFIG_FSL_DPAA_CS_THRESHOLD_10G;
	else
		cs_th = CONFIG_FSL_DPAA_CS_THRESHOLD_1G;
	qm_cgr_cs_thres_set64(&initcgr.cgr.cs_thres, cs_th, 1);

	initcgr.we_mask |= QM_CGR_WE_CSTD_EN;
	initcgr.cgr.cstd_en = QM_CGR_EN;

	err = qman_create_cgr(&priv->cgr_data.cgr, QMAN_CGR_FLAG_USE_INIT,
		&initcgr);
	if (err < 0) {
		pr_err("Error %d creating CGR with ID %d\n", err,
			priv->cgr_data.cgr.cgrid);
		qman_release_cgrid(priv->cgr_data.cgr.cgrid);
		goto out_error;
	}
	pr_debug("Created CGR %d for netdev with hwaddr %pM on QMan channel %d\n",
		 priv->cgr_data.cgr.cgrid, priv->mac_dev->addr,
		 priv->cgr_data.cgr.chan);

out_error:
	return err;
}
EXPORT_SYMBOL(dpaa_eth_cgr_init);

static inline void dpa_setup_ingress(const struct dpa_priv_s *priv,
				     struct dpa_fq *fq,
				     const struct qman_fq *template)
{
	fq->fq_base = *template;
	fq->net_dev = priv->net_dev;

	fq->flags = QMAN_FQ_FLAG_NO_ENQUEUE;
	fq->channel = priv->channel;
}

static inline void dpa_setup_egress(const struct dpa_priv_s *priv,
				    struct dpa_fq *fq,
				    struct fm_port *port,
				    const struct qman_fq *template)
{
	fq->fq_base = *template;
	fq->net_dev = priv->net_dev;

	if (port) {
		fq->flags = QMAN_FQ_FLAG_TO_DCPORTAL;
		fq->channel = (uint16_t)fm_get_tx_port_channel(port);
	} else {
		fq->flags = QMAN_FQ_FLAG_NO_MODIFY;
	}
}

void dpa_fq_setup(struct dpa_priv_s *priv, const struct dpa_fq_cbs_t *fq_cbs,
		struct fm_port *tx_port)
{
	struct dpa_fq *fq;
	uint16_t portals[NR_CPUS];
	int cpu, portal_cnt = 0, num_portals = 0;
	uint32_t pcd_fqid, pcd_fqid_hi_prio;
	const cpumask_t *affine_cpus = qman_affine_cpus();
	int egress_cnt = 0, conf_cnt = 0;

	/* Prepare for PCD FQs init */
	for_each_cpu(cpu, affine_cpus)
		portals[num_portals++] = qman_affine_channel(cpu);
	if (num_portals == 0)
		dev_err(priv->net_dev->dev.parent,
			"No Qman software (affine) channels found");

	pcd_fqid = (priv->mac_dev) ?
		DPAA_ETH_PCD_FQ_BASE(priv->mac_dev->res->start) : 0;
	pcd_fqid_hi_prio = (priv->mac_dev) ?
		DPAA_ETH_PCD_FQ_HI_PRIO_BASE(priv->mac_dev->res->start) : 0;

	/* Initialize each FQ in the list */
	list_for_each_entry(fq, &priv->dpa_fq_list, list) {
		switch (fq->fq_type) {
		case FQ_TYPE_RX_DEFAULT:
			BUG_ON(!priv->mac_dev);
			dpa_setup_ingress(priv, fq, &fq_cbs->rx_defq);
			break;
		case FQ_TYPE_RX_ERROR:
			BUG_ON(!priv->mac_dev);
			dpa_setup_ingress(priv, fq, &fq_cbs->rx_errq);
			break;
		case FQ_TYPE_RX_PCD:
			/* For MACless we can't have dynamic Rx queues */
			BUG_ON(!priv->mac_dev && !fq->fqid);
			dpa_setup_ingress(priv, fq, &fq_cbs->rx_defq);
			if (!fq->fqid)
				fq->fqid = pcd_fqid++;
			fq->channel = portals[portal_cnt];
			portal_cnt = (portal_cnt + 1) % num_portals;
			break;
		case FQ_TYPE_RX_PCD_HI_PRIO:
			/* For MACless we can't have dynamic Hi Pri Rx queues */
			BUG_ON(!priv->mac_dev && !fq->fqid);
			dpa_setup_ingress(priv, fq, &fq_cbs->rx_defq);
			if (!fq->fqid)
				fq->fqid = pcd_fqid_hi_prio++;
			fq->channel = portals[portal_cnt];
			portal_cnt = (portal_cnt + 1) % num_portals;
			break;
		case FQ_TYPE_TX:
			dpa_setup_egress(priv, fq, tx_port,
					 &fq_cbs->egress_ern);
			/* If we have more Tx queues than the number of cores,
			 * just ignore the extra ones.
			 */
			if (egress_cnt < DPAA_ETH_TX_QUEUES)
				priv->egress_fqs[egress_cnt++] = &fq->fq_base;
			break;
		case FQ_TYPE_TX_CONFIRM:
			BUG_ON(!priv->mac_dev);
			dpa_setup_ingress(priv, fq, &fq_cbs->tx_defq);
			break;
		case FQ_TYPE_TX_CONF_MQ:
			BUG_ON(!priv->mac_dev);
			dpa_setup_ingress(priv, fq, &fq_cbs->tx_defq);
			priv->conf_fqs[conf_cnt++] = &fq->fq_base;
			break;
		case FQ_TYPE_TX_ERROR:
			BUG_ON(!priv->mac_dev);
			dpa_setup_ingress(priv, fq, &fq_cbs->tx_errq);
			break;
		default:
			dev_warn(priv->net_dev->dev.parent,
				 "Unknown FQ type detected!\n");
			break;
		}
	}

	/* The number of Tx queues may be smaller than the number of cores, if
	 * the Tx queue range is specified in the device tree instead of being
	 * dynamically allocated.
	 * Make sure all CPUs receive a corresponding Tx queue.
	 */
	while (egress_cnt < DPAA_ETH_TX_QUEUES) {
		list_for_each_entry(fq, &priv->dpa_fq_list, list) {
			if (fq->fq_type != FQ_TYPE_TX)
				continue;
			priv->egress_fqs[egress_cnt++] = &fq->fq_base;
			if (egress_cnt == DPAA_ETH_TX_QUEUES)
				break;
		}
	}
}
EXPORT_SYMBOL(dpa_fq_setup);

int dpa_fq_init(struct dpa_fq *dpa_fq, bool td_enable)
{
	int			 _errno;
	const struct dpa_priv_s	*priv;
	struct device		*dev;
	struct qman_fq		*fq;
	struct qm_mcc_initfq	 initfq;
	struct qman_fq		*confq;
	int			queue_id;

	priv = netdev_priv(dpa_fq->net_dev);
	dev = dpa_fq->net_dev->dev.parent;

	if (dpa_fq->fqid == 0)
		dpa_fq->flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;

	dpa_fq->init = !(dpa_fq->flags & QMAN_FQ_FLAG_NO_MODIFY);

	_errno = qman_create_fq(dpa_fq->fqid, dpa_fq->flags, &dpa_fq->fq_base);
	if (_errno) {
		dev_err(dev, "qman_create_fq() failed\n");
		return _errno;
	}
	fq = &dpa_fq->fq_base;

	if (dpa_fq->init) {
		memset(&initfq, 0, sizeof(initfq));

		initfq.we_mask = QM_INITFQ_WE_FQCTRL;

		/* Try to reduce the number of portal interrupts for
		 * Tx Confirmation FQs.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX_CONFIRM)
			initfq.fqd.fq_ctrl |= QM_FQCTRL_HOLDACTIVE;

		/* FQ placement */
		initfq.we_mask |= QM_INITFQ_WE_DESTWQ;

		initfq.fqd.dest.channel	= dpa_fq->channel;
		initfq.fqd.dest.wq = dpa_fq->wq;

		/* Put all egress queues in a congestion group of their own.
		 * Sensu stricto, the Tx confirmation queues are Rx FQs,
		 * rather than Tx - but they nonetheless account for the
		 * memory footprint on behalf of egress traffic. We therefore
		 * place them in the netdev's CGR, along with the Tx FQs.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX ||
				dpa_fq->fq_type == FQ_TYPE_TX_CONFIRM ||
				dpa_fq->fq_type == FQ_TYPE_TX_CONF_MQ) {
			initfq.we_mask |= QM_INITFQ_WE_CGID;
			initfq.fqd.fq_ctrl |= QM_FQCTRL_CGE;
			initfq.fqd.cgid = (uint8_t)priv->cgr_data.cgr.cgrid;
			/* Set a fixed overhead accounting, in an attempt to
			 * reduce the impact of fixed-size skb shells and the
			 * driver's needed headroom on system memory. This is
			 * especially the case when the egress traffic is
			 * composed of small datagrams.
			 * Unfortunately, QMan's OAL value is capped to an
			 * insufficient value, but even that is better than
			 * no overhead accounting at all.
			 */
			initfq.we_mask |= QM_INITFQ_WE_OAC;
			initfq.fqd.oac_init.oac = QM_OAC_CG;
			initfq.fqd.oac_init.oal =
				(signed char)(min(sizeof(struct sk_buff) +
				priv->tx_headroom, (size_t)FSL_QMAN_MAX_OAL));
		}

		if (td_enable) {
			initfq.we_mask |= QM_INITFQ_WE_TDTHRESH;
			qm_fqd_taildrop_set(&initfq.fqd.td,
					DPA_FQ_TD, 1);
			initfq.fqd.fq_ctrl = QM_FQCTRL_TDE;
		}

		/* Configure the Tx confirmation queue, now that we know
		 * which Tx queue it pairs with.
		 */
		if (dpa_fq->fq_type == FQ_TYPE_TX) {
			queue_id = _dpa_tx_fq_to_id(priv, &dpa_fq->fq_base);
			if (queue_id >= 0) {
				confq = priv->conf_fqs[queue_id];
				if (confq) {
					initfq.we_mask |= QM_INITFQ_WE_CONTEXTA;
			/* ContextA: OVOM=1 (use contextA2 bits instead of ICAD)
			 *	     A2V=1 (contextA A2 field is valid)
			 *           A0V=1 (contextA A0 field is valid)
			 *	     B0V=1 (contextB field is valid)
			 * ContextA A2: EBD=1 (deallocate buffers inside FMan)
			 * ContextB B0(ASPID): 0 (absolute Virtual Storage ID)
			 */
					initfq.fqd.context_a.hi = 0x1e000000;
					initfq.fqd.context_a.lo = 0x80000000;
				}
			}
		}

		/* Put all *private* ingress queues in our "ingress CGR". */
		if (priv->use_ingress_cgr &&
				(dpa_fq->fq_type == FQ_TYPE_RX_DEFAULT ||
				 dpa_fq->fq_type == FQ_TYPE_RX_ERROR ||
				 dpa_fq->fq_type == FQ_TYPE_RX_PCD ||
				 dpa_fq->fq_type == FQ_TYPE_RX_PCD_HI_PRIO)) {
			initfq.we_mask |= QM_INITFQ_WE_CGID;
			initfq.fqd.fq_ctrl |= QM_FQCTRL_CGE;
			initfq.fqd.cgid = (uint8_t)priv->ingress_cgr.cgrid;
			/* Set a fixed overhead accounting, just like for the
			 * egress CGR.
			 */
			initfq.we_mask |= QM_INITFQ_WE_OAC;
			initfq.fqd.oac_init.oac = QM_OAC_CG;
			initfq.fqd.oac_init.oal =
				(signed char)(min(sizeof(struct sk_buff) +
				priv->tx_headroom, (size_t)FSL_QMAN_MAX_OAL));
		}

		/* Initialization common to all ingress queues */
		if (dpa_fq->flags & QMAN_FQ_FLAG_NO_ENQUEUE) {
			initfq.we_mask |= QM_INITFQ_WE_CONTEXTA;
			initfq.fqd.fq_ctrl |=
				QM_FQCTRL_CTXASTASHING | QM_FQCTRL_AVOIDBLOCK;
			initfq.fqd.context_a.stashing.exclusive =
				QM_STASHING_EXCL_DATA | QM_STASHING_EXCL_CTX |
				QM_STASHING_EXCL_ANNOTATION;
			initfq.fqd.context_a.stashing.data_cl = 2;
			initfq.fqd.context_a.stashing.annotation_cl = 1;
			initfq.fqd.context_a.stashing.context_cl =
				DIV_ROUND_UP(sizeof(struct qman_fq), 64);
		}

		_errno = qman_init_fq(fq, QMAN_INITFQ_FLAG_SCHED, &initfq);
		if (_errno < 0) {
			if (DPA_RX_PCD_HI_PRIO_FQ_INIT_FAIL(dpa_fq, _errno)) {
				dpa_fq->init = 0;
			} else {
				dev_err(dev, "qman_init_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);
				qman_destroy_fq(fq, 0);
			}
			return _errno;
		}
	}

	dpa_fq->fqid = qman_fq_fqid(fq);

	return 0;
}
EXPORT_SYMBOL(dpa_fq_init);

int __cold __attribute__((nonnull))
_dpa_fq_free(struct device *dev, struct qman_fq *fq)
{
	int			 _errno, __errno;
	struct dpa_fq		*dpa_fq;
	const struct dpa_priv_s	*priv;

	_errno = 0;

	dpa_fq = container_of(fq, struct dpa_fq, fq_base);
	priv = netdev_priv(dpa_fq->net_dev);

	if (dpa_fq->init) {
		_errno = qman_retire_fq(fq, NULL);
		if (unlikely(_errno < 0) && netif_msg_drv(priv))
			dev_err(dev, "qman_retire_fq(%u) = %d\n",
					qman_fq_fqid(fq), _errno);

		__errno = qman_oos_fq(fq);
		if (unlikely(__errno < 0) && netif_msg_drv(priv)) {
			dev_err(dev, "qman_oos_fq(%u) = %d\n",
					qman_fq_fqid(fq), __errno);
			if (_errno >= 0)
				_errno = __errno;
		}
	}

	qman_destroy_fq(fq, 0);
	list_del(&dpa_fq->list);

	return _errno;
}
EXPORT_SYMBOL(_dpa_fq_free);

int __cold __attribute__((nonnull))
dpa_fq_free(struct device *dev, struct list_head *list)
{
	int		 _errno, __errno;
	struct dpa_fq	*dpa_fq, *tmp;

	_errno = 0;
	list_for_each_entry_safe(dpa_fq, tmp, list, list) {
		__errno = _dpa_fq_free(dev, (struct qman_fq *)dpa_fq);
		if (unlikely(__errno < 0) && _errno >= 0)
			_errno = __errno;
	}

	return _errno;
}
EXPORT_SYMBOL(dpa_fq_free);

int dpa_fqs_init(struct device *dev, struct list_head *list, bool td_enable)
{
	int  _errno, __errno;
	struct dpa_fq	*dpa_fq, *tmp;
	static bool print_msg __read_mostly;

	_errno = 0;
	print_msg = true;
	list_for_each_entry_safe(dpa_fq, tmp, list, list) {
		__errno = dpa_fq_init(dpa_fq, td_enable);
		if (unlikely(__errno < 0) && _errno >= 0) {
			if (DPA_RX_PCD_HI_PRIO_FQ_INIT_FAIL(dpa_fq, __errno)) {
				if (print_msg) {
					dev_warn(dev,
						 "Skip RX PCD High Priority FQs initialization\n");
					print_msg = false;
				}
				if (_dpa_fq_free(dev, (struct qman_fq *)dpa_fq))
					dev_warn(dev,
						 "Error freeing frame queues\n");
			} else {
				_errno = __errno;
				break;
			}
		}
	}

	return _errno;
}
EXPORT_SYMBOL(dpa_fqs_init);
static void
dpaa_eth_init_tx_port(struct fm_port *port, struct dpa_fq *errq,
		struct dpa_fq *defq, struct dpa_buffer_layout_s *buf_layout)
{
	struct fm_port_params tx_port_param;
	bool frag_enabled = false;

	memset(&tx_port_param, 0, sizeof(tx_port_param));
	dpaa_eth_init_port(tx, port, tx_port_param, errq->fqid, defq->fqid,
			   buf_layout, frag_enabled);
}

static void
dpaa_eth_init_rx_port(struct fm_port *port, struct dpa_bp *bp, size_t count,
		struct dpa_fq *errq, struct dpa_fq *defq,
		struct dpa_buffer_layout_s *buf_layout)
{
	struct fm_port_params rx_port_param;
	int i;
	bool frag_enabled = false;

	memset(&rx_port_param, 0, sizeof(rx_port_param));
	count = min(ARRAY_SIZE(rx_port_param.pool_param), count);
	rx_port_param.num_pools = (uint8_t)count;
	for (i = 0; i < count; i++) {
		if (i >= rx_port_param.num_pools)
			break;
		rx_port_param.pool_param[i].id = bp[i].bpid;
		rx_port_param.pool_param[i].size = (uint16_t)bp[i].size;
	}

	dpaa_eth_init_port(rx, port, rx_port_param, errq->fqid, defq->fqid,
			   buf_layout, frag_enabled);
}

#if defined(CONFIG_FSL_SDK_FMAN_TEST)
/* Defined as weak, to be implemented by fman pcd tester. */
int dpa_alloc_pcd_fqids(struct device *, uint32_t, uint8_t, uint32_t *)
__attribute__((weak));

int dpa_free_pcd_fqids(struct device *, uint32_t) __attribute__((weak));
#else
int dpa_alloc_pcd_fqids(struct device *, uint32_t, uint8_t, uint32_t *);

int dpa_free_pcd_fqids(struct device *, uint32_t);

#endif /* CONFIG_FSL_SDK_FMAN_TEST */


int dpa_alloc_pcd_fqids(struct device *dev, uint32_t num,
				uint8_t alignment, uint32_t *base_fqid)
{
	dev_crit(dev, "callback not implemented!\n");

	return 0;
}

int dpa_free_pcd_fqids(struct device *dev, uint32_t base_fqid)
{

	dev_crit(dev, "callback not implemented!\n");

	return 0;
}

void dpaa_eth_init_ports(struct mac_device *mac_dev,
		struct dpa_bp *bp, size_t count,
		struct fm_port_fqs *port_fqs,
		struct dpa_buffer_layout_s *buf_layout,
		struct device *dev)
{
	struct fm_port_pcd_param rx_port_pcd_param;
	struct fm_port *rxport = mac_dev->port_dev[RX];
	struct fm_port *txport = mac_dev->port_dev[TX];

	dpaa_eth_init_tx_port(txport, port_fqs->tx_errq,
			      port_fqs->tx_defq, &buf_layout[TX]);
	dpaa_eth_init_rx_port(rxport, bp, count, port_fqs->rx_errq,
			      port_fqs->rx_defq, &buf_layout[RX]);

	rx_port_pcd_param.cba = dpa_alloc_pcd_fqids;
	rx_port_pcd_param.cbf = dpa_free_pcd_fqids;
	rx_port_pcd_param.dev = dev;
	fm_port_pcd_bind(rxport, &rx_port_pcd_param);
}
EXPORT_SYMBOL(dpaa_eth_init_ports);

void dpa_release_sgt(struct qm_sg_entry *sgt)
{
	struct dpa_bp *dpa_bp;
	struct bm_buffer bmb[DPA_BUFF_RELEASE_MAX];
	uint8_t i = 0, j;

	memset(bmb, 0, DPA_BUFF_RELEASE_MAX * sizeof(struct bm_buffer));

	do {
		dpa_bp = dpa_bpid2pool(qm_sg_entry_get_bpid(&sgt[i]));
		DPA_BUG_ON(!dpa_bp);

		j = 0;
		do {
			DPA_BUG_ON(qm_sg_entry_get_ext(&sgt[i]));
			bm_buffer_set64(&bmb[j], qm_sg_addr(&sgt[i]));

			j++; i++;
		} while (j < ARRAY_SIZE(bmb) &&
			!qm_sg_entry_get_final(&sgt[i-1]) &&
			qm_sg_entry_get_bpid(&sgt[i-1]) ==
			qm_sg_entry_get_bpid(&sgt[i]));

		while (bman_release(dpa_bp->pool, bmb, j, 0))
			cpu_relax();
	} while (!qm_sg_entry_get_final(&sgt[i-1]));
}
EXPORT_SYMBOL(dpa_release_sgt);

void __attribute__((nonnull))
dpa_fd_release(const struct net_device *net_dev, const struct qm_fd *fd)
{
	struct qm_sg_entry	*sgt;
	struct dpa_bp		*dpa_bp;
	struct bm_buffer	bmb;
	dma_addr_t		addr;
	void			*vaddr;

	bmb.opaque = 0;
	bm_buffer_set64(&bmb, qm_fd_addr(fd));

	dpa_bp = dpa_bpid2pool(fd->bpid);
	DPA_BUG_ON(!dpa_bp);

	if (fd->format == qm_fd_sg) {
		vaddr = phys_to_virt(qm_fd_addr(fd));
		sgt = vaddr + dpa_fd_offset(fd);

		dma_unmap_single(dpa_bp->dev, qm_fd_addr(fd), dpa_bp->size,
				 DMA_BIDIRECTIONAL);

		dpa_release_sgt(sgt);
		addr = dma_map_single(dpa_bp->dev, vaddr, dpa_bp->size,
				      DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
			dev_err(dpa_bp->dev, "DMA mapping failed");
			return;
		}
		bm_buffer_set64(&bmb, addr);
	}

	while (bman_release(dpa_bp->pool, &bmb, 1, 0))
		cpu_relax();
}
EXPORT_SYMBOL(dpa_fd_release);

void count_ern(struct dpa_percpu_priv_s *percpu_priv,
		      const struct qm_mr_entry *msg)
{
	switch (msg->ern.rc & QM_MR_RC_MASK) {
	case QM_MR_RC_CGR_TAILDROP:
		percpu_priv->ern_cnt.cg_tdrop++;
		break;
	case QM_MR_RC_WRED:
		percpu_priv->ern_cnt.wred++;
		break;
	case QM_MR_RC_ERROR:
		percpu_priv->ern_cnt.err_cond++;
		break;
	case QM_MR_RC_ORPWINDOW_EARLY:
		percpu_priv->ern_cnt.early_window++;
		break;
	case QM_MR_RC_ORPWINDOW_LATE:
		percpu_priv->ern_cnt.late_window++;
		break;
	case QM_MR_RC_FQ_TAILDROP:
		percpu_priv->ern_cnt.fq_tdrop++;
		break;
	case QM_MR_RC_ORPWINDOW_RETIRED:
		percpu_priv->ern_cnt.fq_retired++;
		break;
	case QM_MR_RC_ORP_ZERO:
		percpu_priv->ern_cnt.orp_zero++;
		break;
	}
}
EXPORT_SYMBOL(count_ern);

/**
 * Turn on HW checksum computation for this outgoing frame.
 * If the current protocol is not something we support in this regard
 * (or if the stack has already computed the SW checksum), we do nothing.
 *
 * Returns 0 if all goes well (or HW csum doesn't apply), and a negative value
 * otherwise.
 *
 * Note that this function may modify the fd->cmd field and the skb data buffer
 * (the Parse Results area).
 */
int dpa_enable_tx_csum(struct dpa_priv_s *priv,
	struct sk_buff *skb, struct qm_fd *fd, char *parse_results)
{
	fm_prs_result_t *parse_result;
	struct iphdr *iph;
	struct ipv6hdr *ipv6h = NULL;
	u8 l4_proto;
	u16 ethertype = ntohs(skb->protocol);
	int retval = 0;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	/* Note: L3 csum seems to be already computed in sw, but we can't choose
	 * L4 alone from the FM configuration anyway.
	 */

	/* Fill in some fields of the Parse Results array, so the FMan
	 * can find them as if they came from the FMan Parser.
	 */
	parse_result = (fm_prs_result_t *)parse_results;

	/* If we're dealing with VLAN, get the real Ethernet type */
	if (ethertype == ETH_P_8021Q) {
		/* We can't always assume the MAC header is set correctly
		 * by the stack, so reset to beginning of skb->data
		 */
		skb_reset_mac_header(skb);
		ethertype = ntohs(vlan_eth_hdr(skb)->h_vlan_encapsulated_proto);
	}

	/* Fill in the relevant L3 parse result fields
	 * and read the L4 protocol type
	 */
	switch (ethertype) {
	case ETH_P_IP:
		parse_result->l3r = cpu_to_be16(FM_L3_PARSE_RESULT_IPV4);
		iph = ip_hdr(skb);
		DPA_BUG_ON(iph == NULL);
		l4_proto = iph->protocol;
		break;
	case ETH_P_IPV6:
		parse_result->l3r = cpu_to_be16(FM_L3_PARSE_RESULT_IPV6);
		ipv6h = ipv6_hdr(skb);
		DPA_BUG_ON(ipv6h == NULL);
		l4_proto = ipv6h->nexthdr;
		break;
	default:
		/* We shouldn't even be here */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_alert(priv->net_dev,
				     "Can't compute HW csum for L3 proto 0x%x\n",
				     ntohs(skb->protocol));
		retval = -EIO;
		goto return_error;
	}

	/* Fill in the relevant L4 parse result fields */
	switch (l4_proto) {
	case IPPROTO_UDP:
		parse_result->l4r = FM_L4_PARSE_RESULT_UDP;
		break;
	case IPPROTO_TCP:
		parse_result->l4r = FM_L4_PARSE_RESULT_TCP;
		break;
	default:
		/* This can as well be a BUG() */
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_alert(priv->net_dev,
				     "Can't compute HW csum for L4 proto 0x%x\n",
				     l4_proto);
		retval = -EIO;
		goto return_error;
	}

	/* At index 0 is IPOffset_1 as defined in the Parse Results */
	parse_result->ip_off[0] = (uint8_t)skb_network_offset(skb);
	parse_result->l4_off = (uint8_t)skb_transport_offset(skb);

	/* Enable L3 (and L4, if TCP or UDP) HW checksum. */
	fd->cmd |= FM_FD_CMD_RPD | FM_FD_CMD_DTC;

	/* On P1023 and similar platforms fd->cmd interpretation could
	 * be disabled by setting CONTEXT_A bit ICMD; currently this bit
	 * is not set so we do not need to check; in the future, if/when
	 * using context_a we need to check this bit
	 */

return_error:
	return retval;
}
EXPORT_SYMBOL(dpa_enable_tx_csum);

#ifdef CONFIG_FSL_DPAA_CEETM
void dpa_enable_ceetm(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	priv->ceetm_en = true;
}
EXPORT_SYMBOL(dpa_enable_ceetm);

void dpa_disable_ceetm(struct net_device *dev)
{
	struct dpa_priv_s *priv = netdev_priv(dev);
	priv->ceetm_en = false;
}
EXPORT_SYMBOL(dpa_disable_ceetm);
#endif
