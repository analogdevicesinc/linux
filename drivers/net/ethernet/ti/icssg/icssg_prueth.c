// SPDX-License-Identifier: GPL-2.0

/* Texas Instruments ICSSG Ethernet Driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/etherdevice.h>
#include <linux/genalloc.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/remoteproc/pruss.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

#include "icssg_prueth.h"
#include "icssg_mii_rt.h"
#include "../k3-cppi-desc-pool.h"

#define PRUETH_MODULE_DESCRIPTION "PRUSS ICSSG Ethernet driver"

/* CTRLMMR_ICSSG_RGMII_CTRL register bits */
#define ICSSG_CTRL_RGMII_ID_MODE                BIT(24)

static int emac_get_tx_ts(struct prueth_emac *emac,
			  struct emac_tx_ts_response *rsp)
{
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	int addr;

	addr = icssg_queue_pop(prueth, slice == 0 ?
			       ICSSG_TS_POP_SLICE0 : ICSSG_TS_POP_SLICE1);
	if (addr < 0)
		return addr;

	memcpy_fromio(rsp, prueth->shram.va + addr, sizeof(*rsp));
	/* return buffer back for to pool */
	icssg_queue_push(prueth, slice == 0 ?
			 ICSSG_TS_PUSH_SLICE0 : ICSSG_TS_PUSH_SLICE1, addr);

	return 0;
}

static void tx_ts_work(struct prueth_emac *emac)
{
	struct skb_shared_hwtstamps ssh;
	struct emac_tx_ts_response tsr;
	struct sk_buff *skb;
	int ret = 0;
	u32 hi_sw;
	u64 ns;

	/* There may be more than one pending requests */
	while (1) {
		ret = emac_get_tx_ts(emac, &tsr);
		if (ret) /* nothing more */
			break;

		if (tsr.cookie >= PRUETH_MAX_TX_TS_REQUESTS ||
		    !emac->tx_ts_skb[tsr.cookie]) {
			netdev_err(emac->ndev, "Invalid TX TS cookie 0x%x\n",
				   tsr.cookie);
			break;
		}

		skb = emac->tx_ts_skb[tsr.cookie];
		emac->tx_ts_skb[tsr.cookie] = NULL;	/* free slot */
		if (!skb) {
			netdev_err(emac->ndev, "Driver Bug! got NULL skb\n");
			break;
		}

		hi_sw = readl(emac->prueth->shram.va +
			      TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
		ns = icssg_ts_to_ns(hi_sw, tsr.hi_ts, tsr.lo_ts,
				    IEP_DEFAULT_CYCLE_TIME_NS);

		memset(&ssh, 0, sizeof(ssh));
		ssh.hwtstamp = ns_to_ktime(ns);

		skb_tstamp_tx(skb, &ssh);
		dev_consume_skb_any(skb);

		if (atomic_dec_and_test(&emac->tx_ts_pending))	/* no more? */
			break;
	}
}

static irqreturn_t prueth_tx_ts_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	/* currently only TX timestamp is being returned */
	tx_ts_work(emac);

	return IRQ_HANDLED;
}

static struct icssg_firmwares icssg_emac_firmwares[] = {
	{
		.pru = "ti-pruss/am65x-sr2-pru0-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu0-prueth-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru0-prueth-fw.elf",
	},
	{
		.pru = "ti-pruss/am65x-sr2-pru1-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu1-prueth-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru1-prueth-fw.elf",
	}
};

static int prueth_emac_start(struct prueth *prueth, struct prueth_emac *emac)
{
	struct icssg_firmwares *firmwares;
	struct device *dev = prueth->dev;
	int slice, ret;

	firmwares = icssg_emac_firmwares;

	slice = prueth_emac_slice(emac);
	if (slice < 0) {
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	ret = icssg_config(prueth, emac, slice);
	if (ret)
		return ret;

	ret = rproc_set_firmware(prueth->pru[slice], firmwares[slice].pru);
	ret = rproc_boot(prueth->pru[slice]);
	if (ret) {
		dev_err(dev, "failed to boot PRU%d: %d\n", slice, ret);
		return -EINVAL;
	}

	ret = rproc_set_firmware(prueth->rtu[slice], firmwares[slice].rtu);
	ret = rproc_boot(prueth->rtu[slice]);
	if (ret) {
		dev_err(dev, "failed to boot RTU%d: %d\n", slice, ret);
		goto halt_pru;
	}

	ret = rproc_set_firmware(prueth->txpru[slice], firmwares[slice].txpru);
	ret = rproc_boot(prueth->txpru[slice]);
	if (ret) {
		dev_err(dev, "failed to boot TX_PRU%d: %d\n", slice, ret);
		goto halt_rtu;
	}

	emac->fw_running = 1;
	return 0;

halt_rtu:
	rproc_shutdown(prueth->rtu[slice]);

halt_pru:
	rproc_shutdown(prueth->pru[slice]);

	return ret;
}

/* called back by PHY layer if there is change in link state of hw port*/
static void emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	struct prueth *prueth = emac->prueth;
	bool new_state = false;
	unsigned long flags;

	if (phydev->link) {
		/* check the mode of operation - full/half duplex */
		if (phydev->duplex != emac->duplex) {
			new_state = true;
			emac->duplex = phydev->duplex;
		}
		if (phydev->speed != emac->speed) {
			new_state = true;
			emac->speed = phydev->speed;
		}
		if (!emac->link) {
			new_state = true;
			emac->link = 1;
		}
	} else if (emac->link) {
		new_state = true;
		emac->link = 0;

		/* f/w should support 100 & 1000 */
		emac->speed = SPEED_1000;

		/* half duplex may not be supported by f/w */
		emac->duplex = DUPLEX_FULL;
	}

	if (new_state) {
		phy_print_status(phydev);

		/* update RGMII and MII configuration based on PHY negotiated
		 * values
		 */
		if (emac->link) {
			if (emac->duplex == DUPLEX_HALF)
				icssg_config_half_duplex(emac);
			/* Set the RGMII cfg for gig en and full duplex */
			icssg_update_rgmii_cfg(prueth->miig_rt, emac);

			/* update the Tx IPG based on 100M/1G speed */
			spin_lock_irqsave(&emac->lock, flags);
			icssg_config_ipg(emac);
			spin_unlock_irqrestore(&emac->lock, flags);
			icssg_config_set_speed(emac);
			emac_set_port_state(emac, ICSSG_EMAC_PORT_FORWARD);

		} else {
			emac_set_port_state(emac, ICSSG_EMAC_PORT_DISABLE);
		}
	}

	if (emac->link) {
		/* reactivate the transmit queue */
		netif_tx_wake_all_queues(ndev);
	} else {
		netif_tx_stop_all_queues(ndev);
		prueth_cleanup_tx_ts(emac);
	}
}

static enum hrtimer_restart emac_rx_timer_callback(struct hrtimer *timer)
{
	struct prueth_emac *emac =
			container_of(timer, struct prueth_emac, rx_hrtimer);
	int rx_flow = PRUETH_RX_FLOW_DATA;

	enable_irq(emac->rx_chns.irq[rx_flow]);
	return HRTIMER_NORESTART;
}

static int emac_phy_connect(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	struct net_device *ndev = emac->ndev;
	/* connect PHY */
	ndev->phydev = of_phy_connect(emac->ndev, emac->phy_node,
				      &emac_adjust_link, 0,
				      emac->phy_if);
	if (!ndev->phydev) {
		dev_err(prueth->dev, "couldn't connect to phy %s\n",
			emac->phy_node->full_name);
		return -ENODEV;
	}

	if (!emac->half_duplex) {
		dev_dbg(prueth->dev, "half duplex mode is not supported\n");
		phy_remove_link_mode(ndev->phydev, ETHTOOL_LINK_MODE_10baseT_Half_BIT);
		phy_remove_link_mode(ndev->phydev, ETHTOOL_LINK_MODE_100baseT_Half_BIT);
	}

	/* remove unsupported modes */
	phy_remove_link_mode(ndev->phydev, ETHTOOL_LINK_MODE_1000baseT_Half_BIT);
	phy_remove_link_mode(ndev->phydev, ETHTOOL_LINK_MODE_Pause_BIT);
	phy_remove_link_mode(ndev->phydev, ETHTOOL_LINK_MODE_Asym_Pause_BIT);

	if (emac->phy_if == PHY_INTERFACE_MODE_MII)
		phy_set_max_speed(ndev->phydev, SPEED_100);

	return 0;
}

static u64 prueth_iep_gettime(void *clockops_data, struct ptp_system_timestamp *sts)
{
	u32 hi_rollover_count, hi_rollover_count_r;
	struct prueth_emac *emac = clockops_data;
	struct prueth *prueth = emac->prueth;
	void __iomem *fw_hi_r_count_addr;
	void __iomem *fw_count_hi_addr;
	u32 iepcount_hi, iepcount_hi_r;
	unsigned long flags;
	u32 iepcount_lo;
	u64 ts = 0;

	fw_count_hi_addr = prueth->shram.va + TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET;
	fw_hi_r_count_addr = prueth->shram.va + TIMESYNC_FW_WC_HI_ROLLOVER_COUNT_OFFSET;

	local_irq_save(flags);
	do {
		iepcount_hi = icss_iep_get_count_hi(emac->iep);
		iepcount_hi += readl(fw_count_hi_addr);
		hi_rollover_count = readl(fw_hi_r_count_addr);
		ptp_read_system_prets(sts);
		iepcount_lo = icss_iep_get_count_low(emac->iep);
		ptp_read_system_postts(sts);

		iepcount_hi_r = icss_iep_get_count_hi(emac->iep);
		iepcount_hi_r += readl(fw_count_hi_addr);
		hi_rollover_count_r = readl(fw_hi_r_count_addr);
	} while ((iepcount_hi_r != iepcount_hi) ||
		 (hi_rollover_count != hi_rollover_count_r));
	local_irq_restore(flags);

	ts = ((u64)hi_rollover_count) << 23 | iepcount_hi;
	ts = ts * (u64)IEP_DEFAULT_CYCLE_TIME_NS + iepcount_lo;

	return ts;
}

static void prueth_iep_settime(void *clockops_data, u64 ns)
{
	struct icssg_setclock_desc __iomem *sc_descp;
	struct prueth_emac *emac = clockops_data;
	struct icssg_setclock_desc sc_desc;
	u64 cyclecount;
	u32 cycletime;
	int timeout;

	if (!emac->fw_running)
		return;

	sc_descp = emac->prueth->shram.va + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET;

	cycletime = IEP_DEFAULT_CYCLE_TIME_NS;
	cyclecount = ns / cycletime;

	memset(&sc_desc, 0, sizeof(sc_desc));
	sc_desc.margin = cycletime - 1000;
	sc_desc.cyclecounter0_set = cyclecount & GENMASK(31, 0);
	sc_desc.cyclecounter1_set = (cyclecount & GENMASK(63, 32)) >> 32;
	sc_desc.iepcount_set = ns % cycletime;
	sc_desc.CMP0_current = cycletime - 4; //Count from 0 to (cycle time)-4

	memcpy_toio(sc_descp, &sc_desc, sizeof(sc_desc));

	writeb(1, &sc_descp->request);

	timeout = 5;	/* fw should take 2-3 ms */
	while (timeout--) {
		if (readb(&sc_descp->acknowledgment))
			return;

		usleep_range(500, 1000);
	}

	dev_err(emac->prueth->dev, "settime timeout\n");
}

static int prueth_perout_enable(void *clockops_data,
				struct ptp_perout_request *req, int on,
				u64 *cmp)
{
	struct prueth_emac *emac = clockops_data;
	u32 reduction_factor = 0, offset = 0;
	struct timespec64 ts;
	u64 ns_period;

	if (!on)
		return 0;

	/* Any firmware specific stuff for PPS/PEROUT handling */
	ts.tv_sec = req->period.sec;
	ts.tv_nsec = req->period.nsec;
	ns_period = timespec64_to_ns(&ts);

	/* f/w doesn't support period less than cycle time */
	if (ns_period < IEP_DEFAULT_CYCLE_TIME_NS)
		return -ENXIO;

	reduction_factor = ns_period / IEP_DEFAULT_CYCLE_TIME_NS;
	offset = ns_period % IEP_DEFAULT_CYCLE_TIME_NS;

	/* f/w requires at least 1uS within a cycle so CMP
	 * can trigger after SYNC is enabled
	 */
	if (offset < 5 * NSEC_PER_USEC)
		offset = 5 * NSEC_PER_USEC;

	/* if offset is close to cycle time then we will miss
	 * the CMP event for last tick when IEP rolls over.
	 * In normal mode, IEP tick is 4ns.
	 * In slow compensation it could be 0ns or 8ns at
	 * every slow compensation cycle.
	 */
	if (offset > IEP_DEFAULT_CYCLE_TIME_NS - 8)
		offset = IEP_DEFAULT_CYCLE_TIME_NS - 8;

	/* we're in shadow mode so need to set upper 32-bits */
	*cmp = (u64)offset << 32;

	writel(reduction_factor, emac->prueth->shram.va +
		TIMESYNC_FW_WC_SYNCOUT_REDUCTION_FACTOR_OFFSET);

	writel(0, emac->prueth->shram.va +
		TIMESYNC_FW_WC_SYNCOUT_START_TIME_CYCLECOUNT_OFFSET);

	return 0;
}

const struct icss_iep_clockops prueth_iep_clockops = {
	.settime = prueth_iep_settime,
	.gettime = prueth_iep_gettime,
	.perout_enable = prueth_perout_enable,
};

/**
 * emac_ndo_open - EMAC device open
 * @ndev: network adapter device
 *
 * Called when system wants to start the interface.
 *
 * Return: 0 for a successful open, or appropriate error code
 */
static int emac_ndo_open(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret, i, num_data_chn = emac->tx_ch_num;
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	struct device *dev = prueth->dev;
	int max_rx_flows;
	int rx_flow;

	/* clear SMEM and MSMC settings for all slices */
	if (!prueth->emacs_initialized) {
		memset_io(prueth->msmcram.va, 0, prueth->msmcram.size);
		memset_io(prueth->shram.va, 0, ICSSG_CONFIG_OFFSET_SLICE1 * PRUETH_NUM_MACS);
	}

	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	icssg_class_set_mac_addr(prueth->miig_rt, slice, emac->mac_addr);
	icssg_ft1_set_mac_addr(prueth->miig_rt, slice, emac->mac_addr);

	icssg_class_default(prueth->miig_rt, slice, 0, false);

	/* Notify the stack of the actual queue counts. */
	ret = netif_set_real_num_tx_queues(ndev, num_data_chn);
	if (ret) {
		dev_err(dev, "cannot set real number of tx queues\n");
		return ret;
	}

	init_completion(&emac->cmd_complete);
	ret = prueth_init_tx_chns(emac);
	if (ret) {
		dev_err(dev, "failed to init tx channel: %d\n", ret);
		return ret;
	}

	max_rx_flows = PRUETH_MAX_RX_FLOWS;
	ret = prueth_init_rx_chns(emac, &emac->rx_chns, "rx",
				  max_rx_flows, PRUETH_MAX_RX_DESC);
	if (ret) {
		dev_err(dev, "failed to init rx channel: %d\n", ret);
		goto cleanup_tx;
	}

	ret = prueth_ndev_add_tx_napi(emac);
	if (ret)
		goto cleanup_rx;

	/* we use only the highest priority flow for now i.e. @irq[3] */
	rx_flow = PRUETH_RX_FLOW_DATA;
	ret = request_irq(emac->rx_chns.irq[rx_flow], prueth_rx_irq,
			  IRQF_TRIGGER_HIGH, dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request RX IRQ\n");
		goto cleanup_napi;
	}

	/* reset and start PRU firmware */
	ret = prueth_emac_start(prueth, emac);
	if (ret)
		goto free_rx_irq;

	icssg_mii_update_mtu(prueth->mii_rt, slice, ndev->max_mtu);

	if (!prueth->emacs_initialized) {
		ret = icss_iep_init(emac->iep, &prueth_iep_clockops,
				    emac, IEP_DEFAULT_CYCLE_TIME_NS);
	}

	ret = request_threaded_irq(emac->tx_ts_irq, NULL, prueth_tx_ts_irq,
				   IRQF_ONESHOT, dev_name(dev), emac);
	if (ret)
		goto stop;

	/* Prepare RX */
	ret = prueth_prepare_rx_chan(emac, &emac->rx_chns, PRUETH_MAX_PKT_SIZE);
	if (ret)
		goto free_tx_ts_irq;

	ret = k3_udma_glue_enable_rx_chn(emac->rx_chns.rx_chn);
	if (ret)
		goto reset_rx_chn;

	for (i = 0; i < emac->tx_ch_num; i++) {
		ret = k3_udma_glue_enable_tx_chn(emac->tx_chns[i].tx_chn);
		if (ret)
			goto reset_tx_chan;
	}

	/* Enable NAPI in Tx and Rx direction */
	for (i = 0; i < emac->tx_ch_num; i++)
		napi_enable(&emac->tx_chns[i].napi_tx);
	napi_enable(&emac->napi_rx);

	/* start PHY */
	phy_start(ndev->phydev);

	prueth->emacs_initialized++;

	queue_work(system_long_wq, &emac->stats_work.work);

	return 0;

reset_tx_chan:
	/* Since interface is not yet up, there is wouldn't be
	 * any SKB for completion. So set false to free_skb
	 */
	prueth_reset_tx_chan(emac, i, false);
reset_rx_chn:
	prueth_reset_rx_chan(&emac->rx_chns, max_rx_flows, false);
free_tx_ts_irq:
	free_irq(emac->tx_ts_irq, emac);
stop:
	prueth_emac_stop(emac);
free_rx_irq:
	free_irq(emac->rx_chns.irq[rx_flow], emac);
cleanup_napi:
	prueth_ndev_del_tx_napi(emac, emac->tx_ch_num);
cleanup_rx:
	prueth_cleanup_rx_chns(emac, &emac->rx_chns, max_rx_flows);
cleanup_tx:
	prueth_cleanup_tx_chns(emac);

	return ret;
}

/**
 * emac_ndo_stop - EMAC device stop
 * @ndev: network adapter device
 *
 * Called when system wants to stop or down the interface.
 *
 * Return: Always 0 (Success)
 */
static int emac_ndo_stop(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int rx_flow = PRUETH_RX_FLOW_DATA;
	int max_rx_flows;
	int ret, i;

	/* inform the upper layers. */
	netif_tx_stop_all_queues(ndev);

	/* block packets from wire */
	if (ndev->phydev)
		phy_stop(ndev->phydev);

	icssg_class_disable(prueth->miig_rt, prueth_emac_slice(emac));

	atomic_set(&emac->tdown_cnt, emac->tx_ch_num);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	/* tear down and disable UDMA channels */
	reinit_completion(&emac->tdown_complete);
	for (i = 0; i < emac->tx_ch_num; i++)
		k3_udma_glue_tdown_tx_chn(emac->tx_chns[i].tx_chn, false);

	ret = wait_for_completion_timeout(&emac->tdown_complete,
					  msecs_to_jiffies(1000));
	if (!ret)
		netdev_err(ndev, "tx teardown timeout\n");

	prueth_reset_tx_chan(emac, emac->tx_ch_num, true);
	for (i = 0; i < emac->tx_ch_num; i++) {
		napi_disable(&emac->tx_chns[i].napi_tx);
		hrtimer_cancel(&emac->tx_chns[i].tx_hrtimer);
	}

	max_rx_flows = PRUETH_MAX_RX_FLOWS;
	k3_udma_glue_tdown_rx_chn(emac->rx_chns.rx_chn, true);

	prueth_reset_rx_chan(&emac->rx_chns, max_rx_flows, true);

	napi_disable(&emac->napi_rx);
	hrtimer_cancel(&emac->rx_hrtimer);

	cancel_work_sync(&emac->rx_mode_work);

	/* Destroying the queued work in ndo_stop() */
	cancel_delayed_work_sync(&emac->stats_work);

	if (prueth->emacs_initialized == 1)
		icss_iep_exit(emac->iep);

	/* stop PRUs */
	prueth_emac_stop(emac);

	free_irq(emac->tx_ts_irq, emac);

	free_irq(emac->rx_chns.irq[rx_flow], emac);
	prueth_ndev_del_tx_napi(emac, emac->tx_ch_num);

	prueth_cleanup_rx_chns(emac, &emac->rx_chns, max_rx_flows);
	prueth_cleanup_tx_chns(emac);

	prueth->emacs_initialized--;

	return 0;
}

static void emac_ndo_set_rx_mode_work(struct work_struct *work)
{
	struct prueth_emac *emac = container_of(work, struct prueth_emac, rx_mode_work);
	struct net_device *ndev = emac->ndev;
	bool promisc, allmulti;

	if (!netif_running(ndev))
		return;

	promisc = ndev->flags & IFF_PROMISC;
	allmulti = ndev->flags & IFF_ALLMULTI;
	emac_set_port_state(emac, ICSSG_EMAC_PORT_UC_FLOODING_DISABLE);
	emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_DISABLE);

	if (promisc) {
		emac_set_port_state(emac, ICSSG_EMAC_PORT_UC_FLOODING_ENABLE);
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}

	if (allmulti) {
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}

	if (!netdev_mc_empty(ndev)) {
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}
}

/**
 * emac_ndo_set_rx_mode - EMAC set receive mode function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to set the receive mode of the device.
 *
 */
static void emac_ndo_set_rx_mode(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	queue_work(emac->cmd_wq, &emac->rx_mode_work);
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_ndo_open,
	.ndo_stop = emac_ndo_stop,
	.ndo_start_xmit = emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = emac_ndo_tx_timeout,
	.ndo_set_rx_mode = emac_ndo_set_rx_mode,
	.ndo_eth_ioctl = emac_ndo_ioctl,
	.ndo_get_stats64 = emac_ndo_get_stats64,
	.ndo_get_phys_port_name = emac_ndo_get_phys_port_name,
};

static int prueth_netdev_init(struct prueth *prueth,
			      struct device_node *eth_node)
{
	int ret, num_tx_chn = PRUETH_MAX_TX_QUEUES;
	struct prueth_emac *emac;
	struct net_device *ndev;
	enum prueth_port port;
	const char *irq_name;
	enum prueth_mac mac;

	port = prueth_node_port(eth_node);
	if (port == PRUETH_PORT_INVALID)
		return -EINVAL;

	mac = prueth_node_mac(eth_node);
	if (mac == PRUETH_MAC_INVALID)
		return -EINVAL;

	ndev = alloc_etherdev_mq(sizeof(*emac), num_tx_chn);
	if (!ndev)
		return -ENOMEM;

	emac = netdev_priv(ndev);
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	emac->cmd_wq = create_singlethread_workqueue("icssg_cmd_wq");
	if (!emac->cmd_wq) {
		ret = -ENOMEM;
		goto free_ndev;
	}
	INIT_WORK(&emac->rx_mode_work, emac_ndo_set_rx_mode_work);

	INIT_DELAYED_WORK(&emac->stats_work, emac_stats_work_handler);

	ret = pruss_request_mem_region(prueth->pruss,
				       port == PRUETH_PORT_MII0 ?
				       PRUSS_MEM_DRAM0 : PRUSS_MEM_DRAM1,
				       &emac->dram);
	if (ret) {
		dev_err(prueth->dev, "unable to get DRAM: %d\n", ret);
		ret = -ENOMEM;
		goto free_wq;
	}

	emac->tx_ch_num = 1;

	irq_name = "tx_ts0";
	if (emac->port_id == PRUETH_PORT_MII1)
		irq_name = "tx_ts1";
	emac->tx_ts_irq = platform_get_irq_byname_optional(prueth->pdev, irq_name);
	if (emac->tx_ts_irq < 0) {
		ret = dev_err_probe(prueth->dev, emac->tx_ts_irq, "could not get tx_ts_irq\n");
		goto free;
	}

	SET_NETDEV_DEV(ndev, prueth->dev);
	spin_lock_init(&emac->lock);
	mutex_init(&emac->cmd_lock);

	emac->phy_node = of_parse_phandle(eth_node, "phy-handle", 0);
	if (!emac->phy_node && !of_phy_is_fixed_link(eth_node)) {
		dev_err(prueth->dev, "couldn't find phy-handle\n");
		ret = -ENODEV;
		goto free;
	} else if (of_phy_is_fixed_link(eth_node)) {
		ret = of_phy_register_fixed_link(eth_node);
		if (ret) {
			ret = dev_err_probe(prueth->dev, ret,
					    "failed to register fixed-link phy\n");
			goto free;
		}

		emac->phy_node = eth_node;
	}

	ret = of_get_phy_mode(eth_node, &emac->phy_if);
	if (ret) {
		dev_err(prueth->dev, "could not get phy-mode property\n");
		goto free;
	}

	if (emac->phy_if != PHY_INTERFACE_MODE_MII &&
	    !phy_interface_mode_is_rgmii(emac->phy_if)) {
		dev_err(prueth->dev, "PHY mode unsupported %s\n", phy_modes(emac->phy_if));
		ret = -EINVAL;
		goto free;
	}

	/* AM65 SR2.0 has TX Internal delay always enabled by hardware
	 * and it is not possible to disable TX Internal delay. The below
	 * switch case block describes how we handle different phy modes
	 * based on hardware restriction.
	 */
	switch (emac->phy_if) {
	case PHY_INTERFACE_MODE_RGMII_ID:
		emac->phy_if = PHY_INTERFACE_MODE_RGMII_RXID;
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		emac->phy_if = PHY_INTERFACE_MODE_RGMII;
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_RXID:
		dev_err(prueth->dev, "RGMII mode without TX delay is not supported");
		ret = -EINVAL;
		goto free;
	default:
		break;
	}

	/* get mac address from DT and set private and netdev addr */
	ret = of_get_ethdev_address(eth_node, ndev);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	ndev->min_mtu = PRUETH_MIN_PKT_SIZE;
	ndev->max_mtu = PRUETH_MAX_MTU;
	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &icssg_ethtool_ops;
	ndev->hw_features = NETIF_F_SG;
	ndev->features = ndev->hw_features;

	netif_napi_add(ndev, &emac->napi_rx, emac_napi_rx_poll);
	hrtimer_init(&emac->rx_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL_PINNED);
	emac->rx_hrtimer.function = &emac_rx_timer_callback;
	prueth->emac[mac] = emac;

	return 0;

free:
	pruss_release_mem_region(prueth->pruss, &emac->dram);
free_wq:
	destroy_workqueue(emac->cmd_wq);
free_ndev:
	emac->ndev = NULL;
	prueth->emac[mac] = NULL;
	free_netdev(ndev);

	return ret;
}

static int prueth_probe(struct platform_device *pdev)
{
	struct device_node *eth_node, *eth_ports_node;
	struct device_node  *eth0_node = NULL;
	struct device_node  *eth1_node = NULL;
	struct genpool_data_align gp_data = {
		.align = SZ_64K,
	};
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct prueth *prueth;
	struct pruss *pruss;
	u32 msmc_ram_size;
	int i, ret;

	np = dev->of_node;

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	dev_set_drvdata(dev, prueth);
	prueth->pdev = pdev;
	prueth->pdata = *(const struct prueth_pdata *)device_get_match_data(dev);

	prueth->dev = dev;
	eth_ports_node = of_get_child_by_name(np, "ethernet-ports");
	if (!eth_ports_node)
		return -ENOENT;

	for_each_child_of_node(eth_ports_node, eth_node) {
		u32 reg;

		if (strcmp(eth_node->name, "port"))
			continue;
		ret = of_property_read_u32(eth_node, "reg", &reg);
		if (ret < 0) {
			dev_err(dev, "%pOF error reading port_id %d\n",
				eth_node, ret);
		}

		of_node_get(eth_node);

		if (reg == 0) {
			eth0_node = eth_node;
			if (!of_device_is_available(eth0_node)) {
				of_node_put(eth0_node);
				eth0_node = NULL;
			}
		} else if (reg == 1) {
			eth1_node = eth_node;
			if (!of_device_is_available(eth1_node)) {
				of_node_put(eth1_node);
				eth1_node = NULL;
			}
		} else {
			dev_err(dev, "port reg should be 0 or 1\n");
		}
	}

	of_node_put(eth_ports_node);

	/* At least one node must be present and available else we fail */
	if (!eth0_node && !eth1_node) {
		dev_err(dev, "neither port0 nor port1 node available\n");
		return -ENODEV;
	}

	if (eth0_node == eth1_node) {
		dev_err(dev, "port0 and port1 can't have same reg\n");
		of_node_put(eth0_node);
		return -ENODEV;
	}

	prueth->eth_node[PRUETH_MAC0] = eth0_node;
	prueth->eth_node[PRUETH_MAC1] = eth1_node;

	prueth->miig_rt = syscon_regmap_lookup_by_phandle(np, "ti,mii-g-rt");
	if (IS_ERR(prueth->miig_rt)) {
		dev_err(dev, "couldn't get ti,mii-g-rt syscon regmap\n");
		return -ENODEV;
	}

	prueth->mii_rt = syscon_regmap_lookup_by_phandle(np, "ti,mii-rt");
	if (IS_ERR(prueth->mii_rt)) {
		dev_err(dev, "couldn't get ti,mii-rt syscon regmap\n");
		return -ENODEV;
	}

	if (eth0_node) {
		ret = prueth_get_cores(prueth, ICSS_SLICE0, false);
		if (ret)
			goto put_cores;
	}

	if (eth1_node) {
		ret = prueth_get_cores(prueth, ICSS_SLICE1, false);
		if (ret)
			goto put_cores;
	}

	pruss = pruss_get(eth0_node ?
			  prueth->pru[ICSS_SLICE0] : prueth->pru[ICSS_SLICE1]);
	if (IS_ERR(pruss)) {
		ret = PTR_ERR(pruss);
		dev_err(dev, "unable to get pruss handle\n");
		goto put_cores;
	}

	prueth->pruss = pruss;

	ret = pruss_request_mem_region(pruss, PRUSS_MEM_SHRD_RAM2,
				       &prueth->shram);
	if (ret) {
		dev_err(dev, "unable to get PRUSS SHRD RAM2: %d\n", ret);
		goto put_pruss;
	}

	prueth->sram_pool = of_gen_pool_get(np, "sram", 0);
	if (!prueth->sram_pool) {
		dev_err(dev, "unable to get SRAM pool\n");
		ret = -ENODEV;

		goto put_mem;
	}

	msmc_ram_size = MSMC_RAM_SIZE;

	/* NOTE: FW bug needs buffer base to be 64KB aligned */
	prueth->msmcram.va =
		(void __iomem *)gen_pool_alloc_algo(prueth->sram_pool,
						    msmc_ram_size,
						    gen_pool_first_fit_align,
						    &gp_data);

	if (!prueth->msmcram.va) {
		ret = -ENOMEM;
		dev_err(dev, "unable to allocate MSMC resource\n");
		goto put_mem;
	}
	prueth->msmcram.pa = gen_pool_virt_to_phys(prueth->sram_pool,
						   (unsigned long)prueth->msmcram.va);
	prueth->msmcram.size = msmc_ram_size;
	memset_io(prueth->msmcram.va, 0, msmc_ram_size);
	dev_dbg(dev, "sram: pa %llx va %p size %zx\n", prueth->msmcram.pa,
		prueth->msmcram.va, prueth->msmcram.size);

	prueth->iep0 = icss_iep_get_idx(np, 0);
	if (IS_ERR(prueth->iep0)) {
		ret = dev_err_probe(dev, PTR_ERR(prueth->iep0), "iep0 get failed\n");
		prueth->iep0 = NULL;
		goto free_pool;
	}

	prueth->iep1 = icss_iep_get_idx(np, 1);
	if (IS_ERR(prueth->iep1)) {
		ret = dev_err_probe(dev, PTR_ERR(prueth->iep1), "iep1 get failed\n");
		goto put_iep0;
	}

	if (prueth->pdata.quirk_10m_link_issue) {
		/* Enable IEP1 for FW in 64bit mode as W/A for 10M FD link detect issue under TX
		 * traffic.
		 */
		icss_iep_init_fw(prueth->iep1);
	}

	/* setup netdev interfaces */
	if (eth0_node) {
		ret = prueth_netdev_init(prueth, eth0_node);
		if (ret) {
			dev_err_probe(dev, ret, "netdev init %s failed\n",
				      eth0_node->name);
			goto exit_iep;
		}

		if (of_find_property(eth0_node, "ti,half-duplex-capable", NULL))
			prueth->emac[PRUETH_MAC0]->half_duplex = 1;

		prueth->emac[PRUETH_MAC0]->iep = prueth->iep0;
	}

	if (eth1_node) {
		ret = prueth_netdev_init(prueth, eth1_node);
		if (ret) {
			dev_err_probe(dev, ret, "netdev init %s failed\n",
				      eth1_node->name);
			goto netdev_exit;
		}

		if (of_find_property(eth1_node, "ti,half-duplex-capable", NULL))
			prueth->emac[PRUETH_MAC1]->half_duplex = 1;

		prueth->emac[PRUETH_MAC1]->iep = prueth->iep0;
	}

	/* register the network devices */
	if (eth0_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC0]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII0");
			goto netdev_exit;
		}

		prueth->registered_netdevs[PRUETH_MAC0] = prueth->emac[PRUETH_MAC0]->ndev;

		ret = emac_phy_connect(prueth->emac[PRUETH_MAC0]);
		if (ret) {
			dev_err(dev,
				"can't connect to MII0 PHY, error -%d", ret);
			goto netdev_unregister;
		}
		phy_attached_info(prueth->emac[PRUETH_MAC0]->ndev->phydev);
	}

	if (eth1_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC1]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII1");
			goto netdev_unregister;
		}

		prueth->registered_netdevs[PRUETH_MAC1] = prueth->emac[PRUETH_MAC1]->ndev;
		ret = emac_phy_connect(prueth->emac[PRUETH_MAC1]);
		if (ret) {
			dev_err(dev,
				"can't connect to MII1 PHY, error %d", ret);
			goto netdev_unregister;
		}
		phy_attached_info(prueth->emac[PRUETH_MAC1]->ndev->phydev);
	}

	dev_info(dev, "TI PRU ethernet driver initialized: %s EMAC mode\n",
		 (!eth0_node || !eth1_node) ? "single" : "dual");

	if (eth1_node)
		of_node_put(eth1_node);
	if (eth0_node)
		of_node_put(eth0_node);
	return 0;

netdev_unregister:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		if (prueth->emac[i]->ndev->phydev) {
			phy_disconnect(prueth->emac[i]->ndev->phydev);
			prueth->emac[i]->ndev->phydev = NULL;
		}
		unregister_netdev(prueth->registered_netdevs[i]);
	}

netdev_exit:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}

exit_iep:
	if (prueth->pdata.quirk_10m_link_issue)
		icss_iep_exit_fw(prueth->iep1);
	icss_iep_put(prueth->iep1);

put_iep0:
	icss_iep_put(prueth->iep0);
	prueth->iep0 = NULL;
	prueth->iep1 = NULL;

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va, msmc_ram_size);

put_mem:
	pruss_release_mem_region(prueth->pruss, &prueth->shram);

put_pruss:
	pruss_put(prueth->pruss);

put_cores:
	if (eth1_node) {
		prueth_put_cores(prueth, ICSS_SLICE1);
		of_node_put(eth1_node);
	}

	if (eth0_node) {
		prueth_put_cores(prueth, ICSS_SLICE0);
		of_node_put(eth0_node);
	}

	return ret;
}

static void prueth_remove(struct platform_device *pdev)
{
	struct prueth *prueth = platform_get_drvdata(pdev);
	struct device_node *eth_node;
	int i;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		phy_stop(prueth->emac[i]->ndev->phydev);
		phy_disconnect(prueth->emac[i]->ndev->phydev);
		prueth->emac[i]->ndev->phydev = NULL;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}

	if (prueth->pdata.quirk_10m_link_issue)
		icss_iep_exit_fw(prueth->iep1);

	icss_iep_put(prueth->iep1);
	icss_iep_put(prueth->iep0);

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va,
		      MSMC_RAM_SIZE);

	pruss_release_mem_region(prueth->pruss, &prueth->shram);

	pruss_put(prueth->pruss);

	if (prueth->eth_node[PRUETH_MAC1])
		prueth_put_cores(prueth, ICSS_SLICE1);

	if (prueth->eth_node[PRUETH_MAC0])
		prueth_put_cores(prueth, ICSS_SLICE0);
}

static const struct prueth_pdata am654_icssg_pdata = {
	.fdqring_mode = K3_RINGACC_RING_MODE_MESSAGE,
	.quirk_10m_link_issue = 1,
};

static const struct prueth_pdata am64x_icssg_pdata = {
	.fdqring_mode = K3_RINGACC_RING_MODE_RING,
};

static const struct of_device_id prueth_dt_match[] = {
	{ .compatible = "ti,am654-icssg-prueth", .data = &am654_icssg_pdata },
	{ .compatible = "ti,am642-icssg-prueth", .data = &am64x_icssg_pdata },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, prueth_dt_match);

static struct platform_driver prueth_driver = {
	.probe = prueth_probe,
	.remove_new = prueth_remove,
	.driver = {
		.name = "icssg-prueth",
		.of_match_table = prueth_dt_match,
		.pm = &prueth_dev_pm_ops,
	},
};
module_platform_driver(prueth_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_AUTHOR("Md Danish Anwar <danishanwar@ti.com>");
MODULE_DESCRIPTION("PRUSS ICSSG Ethernet Driver");
MODULE_LICENSE("GPL");
