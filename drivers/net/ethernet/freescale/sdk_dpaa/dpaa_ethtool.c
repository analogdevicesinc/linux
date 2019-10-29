/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#include <linux/string.h>
#include <linux/of_platform.h>
#include <linux/net_tstamp.h>
#include <linux/fsl/ptp_qoriq.h>

#include "dpaa_eth.h"
#include "mac.h"                /* struct mac_device */
#include "dpaa_eth_common.h"

static const char dpa_stats_percpu[][ETH_GSTRING_LEN] = {
	"interrupts",
	"rx packets",
	"tx packets",
	"tx recycled",
	"tx confirm",
	"tx S/G",
	"rx S/G",
	"tx error",
	"rx error",
	"bp count"
};

static char dpa_stats_global[][ETH_GSTRING_LEN] = {
	/* dpa rx errors */
	"rx dma error",
	"rx frame physical error",
	"rx frame size error",
	"rx header error",
	"rx csum error",

	/* demultiplexing errors */
	"qman cg_tdrop",
	"qman wred",
	"qman error cond",
	"qman early window",
	"qman late window",
	"qman fq tdrop",
	"qman fq retired",
	"qman orp disabled",

	/* congestion related stats */
	"congestion time (ms)",
	"entered congestion",
	"congested (0/1)"
};

#define DPA_STATS_PERCPU_LEN ARRAY_SIZE(dpa_stats_percpu)
#define DPA_STATS_GLOBAL_LEN ARRAY_SIZE(dpa_stats_global)

static int __cold dpa_get_ksettings(struct net_device *net_dev,
		struct ethtool_link_ksettings *cmd)
{
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_dbg(net_dev, "phy device not initialized\n");
		return 0;
	}

	phy_ethtool_ksettings_get(priv->mac_dev->phy_dev, cmd);

	return 0;
}

static int __cold dpa_set_ksettings(struct net_device *net_dev,
		const struct ethtool_link_ksettings *cmd)
{
	int			 _errno;
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	_errno = phy_ethtool_ksettings_set(priv->mac_dev->phy_dev, cmd);
	if (unlikely(_errno < 0))
		netdev_err(net_dev, "phy_ethtool_ksettings_set() = %d\n", _errno);

	return _errno;
}

static void __cold dpa_get_drvinfo(struct net_device *net_dev,
		struct ethtool_drvinfo *drvinfo)
{
	int		 _errno;

	strncpy(drvinfo->driver, KBUILD_MODNAME,
		sizeof(drvinfo->driver) - 1)[sizeof(drvinfo->driver)-1] = 0;
	_errno = snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
			  "%X", 0);

	if (unlikely(_errno >= sizeof(drvinfo->fw_version))) {
		/* Truncated output */
		netdev_notice(net_dev, "snprintf() = %d\n", _errno);
	} else if (unlikely(_errno < 0)) {
		netdev_warn(net_dev, "snprintf() = %d\n", _errno);
		memset(drvinfo->fw_version, 0, sizeof(drvinfo->fw_version));
	}
	strncpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info)-1)[sizeof(drvinfo->bus_info)-1] = 0;
}

static uint32_t __cold dpa_get_msglevel(struct net_device *net_dev)
{
	return ((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable;
}

static void __cold dpa_set_msglevel(struct net_device *net_dev,
		uint32_t msg_enable)
{
	((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable = msg_enable;
}

static int __cold dpa_nway_reset(struct net_device *net_dev)
{
	int			 _errno;
	struct dpa_priv_s	*priv;

	priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}
	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	_errno = 0;
	if (priv->mac_dev->phy_dev->autoneg) {
		_errno = phy_start_aneg(priv->mac_dev->phy_dev);
		if (unlikely(_errno < 0))
			netdev_err(net_dev, "phy_start_aneg() = %d\n",
					_errno);
	}

	return _errno;
}

static void __cold dpa_get_pauseparam(struct net_device *net_dev,
		struct ethtool_pauseparam *epause)
{
	struct dpa_priv_s	*priv;
	struct mac_device       *mac_dev;
	struct phy_device       *phy_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	if (mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return;
	}

	phy_dev = mac_dev->phy_dev;
	if (unlikely(phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return;
	}

	epause->autoneg = mac_dev->autoneg_pause;
	epause->rx_pause = mac_dev->rx_pause_active;
	epause->tx_pause = mac_dev->tx_pause_active;
}

static int __cold dpa_set_pauseparam(struct net_device *net_dev,
		struct ethtool_pauseparam *epause)
{
	struct dpa_priv_s	*priv;
	struct mac_device       *mac_dev;
	struct phy_device       *phy_dev;
	int _errno;
	bool rx_pause, tx_pause;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	if (mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}

	phy_dev = mac_dev->phy_dev;
	if (unlikely(phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	if (!phy_validate_pause(phy_dev, epause))
		return -EINVAL;

	/* The MAC should know how to handle PAUSE frame autonegotiation before
	 * adjust_link is triggered by a forced renegotiation of sym/asym PAUSE
	 * settings.
	 */
	mac_dev->autoneg_pause = !!epause->autoneg;
	mac_dev->rx_pause_req = !!epause->rx_pause;
	mac_dev->tx_pause_req = !!epause->tx_pause;

	/* Determine the sym/asym advertised PAUSE capabilities from the desired
	 * rx/tx pause settings.
	 */
	phy_set_asym_pause(phy_dev, epause->rx_pause, epause->tx_pause);

	get_pause_cfg(mac_dev, &rx_pause, &tx_pause);
	_errno = set_mac_active_pause(mac_dev, rx_pause, tx_pause);
	if (unlikely(_errno < 0))
		netdev_err(net_dev, "set_mac_active_pause() = %d\n", _errno);

	return _errno;
}

#ifdef CONFIG_PM
static void dpa_get_wol(struct net_device *net_dev, struct ethtool_wolinfo *wol)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);

	wol->supported = 0;
	wol->wolopts = 0;

	if (!priv->wol || !device_can_wakeup(net_dev->dev.parent))
		return;

	if (priv->wol & DPAA_WOL_MAGIC) {
		wol->supported = WAKE_MAGIC;
		wol->wolopts = WAKE_MAGIC;
	}
}

static int dpa_set_wol(struct net_device *net_dev, struct ethtool_wolinfo *wol)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);

	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}

	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_dbg(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	if (!device_can_wakeup(net_dev->dev.parent) ||
		(wol->wolopts & ~WAKE_MAGIC))
		return -EOPNOTSUPP;

	priv->wol = 0;

	if (wol->wolopts & WAKE_MAGIC) {
		priv->wol = DPAA_WOL_MAGIC;
		device_set_wakeup_enable(net_dev->dev.parent, 1);
	} else {
		device_set_wakeup_enable(net_dev->dev.parent, 0);
	}

	return 0;
}
#endif

static int dpa_get_eee(struct net_device *net_dev, struct ethtool_eee *et_eee)
{
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);
	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}

	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	return phy_ethtool_get_eee(priv->mac_dev->phy_dev, et_eee);
}

static int dpa_set_eee(struct net_device *net_dev, struct ethtool_eee *et_eee)
{
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);
	if (priv->mac_dev == NULL) {
		netdev_info(net_dev, "This is a MAC-less interface\n");
		return -ENODEV;
	}

	if (unlikely(priv->mac_dev->phy_dev == NULL)) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	return phy_ethtool_set_eee(priv->mac_dev->phy_dev, et_eee);
}

static int dpa_get_sset_count(struct net_device *net_dev, int type)
{
	unsigned int total_stats, num_stats;

	num_stats   = num_online_cpus() + 1;
	total_stats = num_stats * DPA_STATS_PERCPU_LEN + DPA_STATS_GLOBAL_LEN;

	switch (type) {
	case ETH_SS_STATS:
		return total_stats;
	default:
		return -EOPNOTSUPP;
	}
}

static void copy_stats(struct dpa_percpu_priv_s *percpu_priv, int num_cpus,
			int crr_cpu, u64 bp_count, u64 *data)
{
	int num_stat_values = num_cpus + 1;
	int crr_stat = 0;

	/* update current CPU's stats and also add them to the total values */
	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->in_interrupt;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->in_interrupt;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->stats.rx_packets;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->stats.rx_packets;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->stats.tx_packets;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->stats.tx_packets;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->tx_returned;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->tx_returned;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->tx_confirm;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->tx_confirm;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->tx_frag_skbuffs;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->tx_frag_skbuffs;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->rx_sg;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->rx_sg;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->stats.tx_errors;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->stats.tx_errors;

	data[crr_stat * num_stat_values + crr_cpu] = percpu_priv->stats.rx_errors;
	data[crr_stat++ * num_stat_values + num_cpus] += percpu_priv->stats.rx_errors;

	data[crr_stat * num_stat_values + crr_cpu] = bp_count;
	data[crr_stat++ * num_stat_values + num_cpus] += bp_count;
}

static void dpa_get_ethtool_stats(struct net_device *net_dev,
		struct ethtool_stats *stats, u64 *data)
{
	u64 bp_count, cg_time, cg_num, cg_status;
	struct dpa_percpu_priv_s *percpu_priv;
	struct qm_mcr_querycgr query_cgr;
	struct dpa_rx_errors rx_errors;
	struct dpa_ern_cnt ern_cnt;
	struct dpa_priv_s *priv;
	unsigned int num_cpus, offset;
	struct dpa_bp *dpa_bp;
	int total_stats, i;

	total_stats = dpa_get_sset_count(net_dev, ETH_SS_STATS);
	priv     = netdev_priv(net_dev);
	dpa_bp   = priv->dpa_bp;
	num_cpus = num_online_cpus();
	bp_count = 0;

	memset(&rx_errors, 0, sizeof(struct dpa_rx_errors));
	memset(&ern_cnt, 0, sizeof(struct dpa_ern_cnt));
	memset(data, 0, total_stats * sizeof(u64));

	for_each_online_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		if (dpa_bp->percpu_count)
			bp_count = *(per_cpu_ptr(dpa_bp->percpu_count, i));

		rx_errors.dme += percpu_priv->rx_errors.dme;
		rx_errors.fpe += percpu_priv->rx_errors.fpe;
		rx_errors.fse += percpu_priv->rx_errors.fse;
		rx_errors.phe += percpu_priv->rx_errors.phe;
		rx_errors.cse += percpu_priv->rx_errors.cse;

		ern_cnt.cg_tdrop     += percpu_priv->ern_cnt.cg_tdrop;
		ern_cnt.wred         += percpu_priv->ern_cnt.wred;
		ern_cnt.err_cond     += percpu_priv->ern_cnt.err_cond;
		ern_cnt.early_window += percpu_priv->ern_cnt.early_window;
		ern_cnt.late_window  += percpu_priv->ern_cnt.late_window;
		ern_cnt.fq_tdrop     += percpu_priv->ern_cnt.fq_tdrop;
		ern_cnt.fq_retired   += percpu_priv->ern_cnt.fq_retired;
		ern_cnt.orp_zero     += percpu_priv->ern_cnt.orp_zero;

		copy_stats(percpu_priv, num_cpus, i, bp_count, data);
	}

	offset = (num_cpus + 1) * DPA_STATS_PERCPU_LEN;
	memcpy(data + offset, &rx_errors, sizeof(struct dpa_rx_errors));

	offset += sizeof(struct dpa_rx_errors) / sizeof(u64);
	memcpy(data + offset, &ern_cnt, sizeof(struct dpa_ern_cnt));

	/* gather congestion related counters */
	cg_num    = 0;
	cg_status = 0;
	cg_time   = jiffies_to_msecs(priv->cgr_data.congested_jiffies);
	if (qman_query_cgr(&priv->cgr_data.cgr, &query_cgr) == 0) {
		cg_num    = priv->cgr_data.cgr_congested_count;
		cg_status = query_cgr.cgr.cs;

		/* reset congestion stats (like QMan API does */
		priv->cgr_data.congested_jiffies   = 0;
		priv->cgr_data.cgr_congested_count = 0;
	}

	offset += sizeof(struct dpa_ern_cnt) / sizeof(u64);
	data[offset++] = cg_time;
	data[offset++] = cg_num;
	data[offset++] = cg_status;
}

static void dpa_get_strings(struct net_device *net_dev, u32 stringset, u8 *data)
{
	unsigned int i, j, num_cpus, size;
	char stat_string_cpu[ETH_GSTRING_LEN];
	u8 *strings;

	strings   = data;
	num_cpus  = num_online_cpus();
	size      = DPA_STATS_GLOBAL_LEN * ETH_GSTRING_LEN;

	for (i = 0; i < DPA_STATS_PERCPU_LEN; i++) {
		for (j = 0; j < num_cpus; j++) {
			snprintf(stat_string_cpu, ETH_GSTRING_LEN, "%s [CPU %d]", dpa_stats_percpu[i], j);
			memcpy(strings, stat_string_cpu, ETH_GSTRING_LEN);
			strings += ETH_GSTRING_LEN;
		}
		snprintf(stat_string_cpu, ETH_GSTRING_LEN, "%s [TOTAL]", dpa_stats_percpu[i]);
		memcpy(strings, stat_string_cpu, ETH_GSTRING_LEN);
		strings += ETH_GSTRING_LEN;
	}
	memcpy(strings, dpa_stats_global, size);
}

static int dpaa_get_ts_info(struct net_device *net_dev,
			    struct ethtool_ts_info *info)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct device *dev = priv->mac_dev->dev;
	struct device_node *mac_node = dev->of_node;
	struct device_node *fman_node = NULL, *ptp_node = NULL;
	struct platform_device *ptp_dev = NULL;
	struct ptp_qoriq *ptp = NULL;

	info->phc_index = -1;

	fman_node = of_get_parent(mac_node);
	if (fman_node)
		ptp_node = of_parse_phandle(fman_node, "ptimer-handle", 0);

	if (ptp_node)
		ptp_dev = of_find_device_by_node(ptp_node);

	if (ptp_dev)
		ptp = platform_get_drvdata(ptp_dev);

	if (ptp)
		info->phc_index = ptp->phc_index;

#ifdef CONFIG_FSL_DPAA_TS
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON);
	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);
#else
	info->so_timestamping = SOF_TIMESTAMPING_RX_SOFTWARE |
				SOF_TIMESTAMPING_SOFTWARE;
#endif

	return 0;
}

const struct ethtool_ops dpa_ethtool_ops = {
	.get_link_ksettings = dpa_get_ksettings,
	.set_link_ksettings = dpa_set_ksettings,
	.get_drvinfo = dpa_get_drvinfo,
	.get_msglevel = dpa_get_msglevel,
	.set_msglevel = dpa_set_msglevel,
	.nway_reset = dpa_nway_reset,
	.get_pauseparam = dpa_get_pauseparam,
	.set_pauseparam = dpa_set_pauseparam,
	.self_test = NULL, /* TODO invoke the cold-boot unit-test? */
	.get_link = ethtool_op_get_link,
	.get_eee = dpa_get_eee,
	.set_eee = dpa_set_eee,
	.get_sset_count = dpa_get_sset_count,
	.get_ethtool_stats = dpa_get_ethtool_stats,
	.get_strings = dpa_get_strings,
#ifdef CONFIG_PM
	.get_wol = dpa_get_wol,
	.set_wol = dpa_set_wol,
#endif
	.get_ts_info = dpaa_get_ts_info,
};
