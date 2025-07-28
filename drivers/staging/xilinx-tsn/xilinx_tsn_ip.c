// SPDX-License-Identifier: GPL-2.0

/* Xilinx FPGA Xilinx TSN IP driver.
 *
 * Copyright (C) 2018 Xilinx, Inc. All rights reserved.
 *
 * Author: Priyadarshini Babu <priyadar@xilinx.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/phy.h>
#include <linux/udp.h>
#include <linux/mii.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/xilinx_phy.h>
#include <linux/platform_device.h>

#include "xilinx_axienet_tsn.h"
#include "xilinx_tsn_switch.h"

#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
#include "xilinx_tsn_ptp.h"
#include "xilinx_tsn_timer.h"
#endif

#define TSN_TX_BE_QUEUE  0
#define TSN_TX_RES_QUEUE 1
#define TSN_TX_ST_QUEUE  2

#define XAE_TEMAC1 0
#define XAE_TEMAC2 1
static const struct of_device_id tsn_ip_of_match[] = {
	{ .compatible = "xlnx,tsn-endpoint-ethernet-mac-1.0"},
	{ .compatible = "xlnx,tsn-endpoint-ethernet-mac-2.0"},
	{},
};

MODULE_DEVICE_TABLE(of, tsn_ip_of_match);

/**
 * tsn_ip_probe - TSN ip pointer probe function.
 * @pdev:	Pointer to platform device structure.
 *
 * Return: 0, on success
 *	    Non-zero error value on failure.
 *
 * This is the probe routine for TSN driver.
 */
static int tsn_ip_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("TSN endpoint ethernet mac Probe\n");

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret)
		pr_err("TSN endpoint probe error (%i)\n", ret);

	return ret;
}

static void tsn_ip_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
}

u16 axienet_tsn_select_queue(struct net_device *ndev, struct sk_buff *skb,
			     struct net_device *sb_dev)
{
	struct axienet_local *lp = netdev_priv(ndev);
#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	struct ethhdr *hdr = (struct ethhdr *)skb->data;
	const struct udphdr *udp;

	udp = udp_hdr(skb);
	if (hdr->h_proto == htons(ETH_P_1588) ||
	    (lp->current_rx_filter == HWTSTAMP_FILTER_PTP_V2_L4_EVENT &&
	     hdr->h_proto == htons(ETH_P_IP) && udp->dest == htons(0x013f))) {
		/* It is not possible to use a static queue number for PTP
		 * across various designs with multiple queues support as the
		 * hardcoded queue number may conflict with actual TX queue.
		 * Using num_tc is safe in all designs since the TX queues will
		 * be in the range [0, num_tc-1]
		 */
		return lp->num_tc;
	}
#endif
	if (lp->abl_reg & TSN_BRIDGEEP_EPONLY)
		return axienet_tsn_pcp_to_queue(ndev, skb);

	return BE_QUEUE_NUMBER;
}

/**
 * axienet_tsn_xmit - Starts the TSN transmission.
 * @skb:	sk_buff pointer that contains data to be Txed.
 * @ndev:	Pointer to net_device structure.
 *
 * Return: NETDEV_TX_OK, on success
 *	    Non-zero error value on failure.
 *
 * This function is invoked from upper layers to initiate transmission. The
 * function uses the next available free BDs and populates their fields to
 * start the transmission. Use axienet_ptp_xmit() for PTP 1588 packets and
 * use master EP xmit for other packets transmission.
 */
int axienet_tsn_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct net_device *master = lp->master;
	u16 map = skb_get_queue_mapping(skb);
	struct axienet_tsn_txq *txq = NULL;

#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	/* check if skb is a PTP frame ? */
	if (map == lp->num_tc)
		return axienet_ptp_xmit(skb, ndev);
#endif
	txq = &lp->txqs[map];
	if (lp->abl_reg & TSN_BRIDGEEP_EPONLY) {
		if (txq->is_tadma) /* ST Traffic */
			return axienet_tadma_xmit(skb, ndev, map);

		return axienet_queue_xmit_tsn(skb, ndev, map);
	}
	/* use EP to xmit non-PTP frames */
	skb->dev = master;
	dev_queue_xmit(skb);

	return NETDEV_TX_OK;
}

/**
 * axienet_tsn_probe - TSN mac probe function.
 * @pdev:	Pointer to platform device structure.
 * @lp:		Pointer to axienet local structure
 * @ndev:	Pointer to net_device structure.
 *
 * Return: 0, on success
 *	    Non-zero error value on failure.
 *
 * This is the probe for TSN mac nodes.
 */
int axienet_tsn_probe(struct platform_device *pdev,
		      struct axienet_local *lp,
		      struct net_device *ndev)
{
	int ret = 0;
#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	char irq_name[32];
#endif
	bool slave = false;
	u8     temac_no;
#if IS_ENABLED(CONFIG_XILINX_TSN_QBV)
	u32 qbv_addr, qbv_size;
#endif
	struct device_node *ep_node;
	struct axienet_local *ep_lp;

	slave = of_property_read_bool(pdev->dev.of_node,
				      "xlnx,tsn-slave");
	if (slave) {
		temac_no = XAE_TEMAC2;
		lp->switch_prt = PORT_MAC2;
	} else {
		temac_no = XAE_TEMAC1;
		lp->switch_prt = PORT_MAC1;
	}
#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	lp->current_rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
	sprintf(irq_name, "interrupt_ptp_rx_%d", temac_no + 1);
	lp->ptp_rx_irq = platform_get_irq_byname(pdev, irq_name);

	pr_info("ptp RX irq: %d %s\n", lp->ptp_rx_irq, irq_name);
	sprintf(irq_name, "interrupt_ptp_tx_%d", temac_no + 1);
	lp->ptp_tx_irq = platform_get_irq_byname(pdev, irq_name);
	pr_info("ptp TX irq: %d %s\n", lp->ptp_tx_irq, irq_name);
#endif
#if IS_ENABLED(CONFIG_XILINX_TSN_QBV)
	sprintf(irq_name, "tsn_switch_scheduler_irq_%d", temac_no + 1);
	lp->qbv_irq = platform_get_irq_byname(pdev, irq_name);

	/*Ignoring if the qbv_irq is not exist*/
	if (lp->qbv_irq > 0)
		pr_info("qbv_irq: %d %s\n", lp->qbv_irq, irq_name);
#endif
	spin_lock_init(&lp->ptp_tx_lock);

#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	if (temac_no == XAE_TEMAC1)
		lp->timer_priv = axienet_ptp_timer_probe((lp->regs + XAE_RTC_OFFSET), pdev);
#endif

	/* enable VLAN */
	lp->options |= XAE_OPTION_VLAN;
	axienet_setoptions_tsn(lp->ndev, lp->options);

	/* get the ep device */
	ep_node = of_parse_phandle(pdev->dev.of_node, "tsn,endpoint", 0);

	lp->abl_reg = axienet_ior(lp, XAE_TSN_ABL_OFFSET);

	/* in switch-mode, get the endpoint network device. in ep-only mode,
	 * the endpoint driver frees itself
	 */
	if (!(lp->abl_reg & TSN_BRIDGEEP_EPONLY) && ep_node) {
		lp->master = of_find_net_device_by_node(ep_node);
		if (!lp->master) {
			netdev_err(ndev, "Defer probe as EP is not probed\n");
			ret = -EPROBE_DEFER;
			goto err_1;
		}
	}

	/* in ep only case tie the data path to eth1 */
	if (lp->abl_reg & TSN_BRIDGEEP_EPONLY && temac_no == XAE_TEMAC1) {
		axienet_set_pcpmap(lp);
		ret = tsn_mcdma_probe(pdev, lp, ndev);
		if (ret) {
			dev_err(&pdev->dev, "Getting MCDMA resource failed\n");
			goto err_1;
		}
#if IS_ENABLED(CONFIG_AXIENET_HAS_TADMA)
		ret = axienet_tadma_probe(pdev, ndev);
		if (ret) {
			dev_err(&pdev->dev, "Getting TADMA resource failed\n");
			goto err_1;
		}
#endif
		axienet_init_tsn_txqs(lp, lp->num_tc);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize TX queues\n");
			goto err_1;
		}
	}

#if IS_ENABLED(CONFIG_XILINX_TSN_QBV)
	lp->qbv_regs = NULL;
	if (!(lp->abl_reg & TSN_BRIDGEEP_EPONLY)) {
		of_property_read_u32(pdev->dev.of_node, "xlnx,qbv-addr",
				     &qbv_addr);
		of_property_read_u32(pdev->dev.of_node, "xlnx,qbv-size",
				     &qbv_size);
	} else {
		struct resource res;

		/* get qbv info from ep_node */
		if (of_address_to_resource(ep_node, 0, &res) < 0)
			dev_err(&pdev->dev, "error reading reg property\n");
		qbv_addr = res.start;
		qbv_size = res.end - res.start;
	}
	lp->qbv_regs = devm_ioremap(&pdev->dev, qbv_addr, qbv_size);
	if (IS_ERR(lp->qbv_regs)) {
		dev_err(&pdev->dev, "ioremap failed for the qbv\n");
		ret = PTR_ERR(lp->qbv_regs);
		goto err_1;
	}
	ret = axienet_qbv_init(ndev);
#endif
	if (!(lp->abl_reg & TSN_BRIDGEEP_EPONLY)) {
		/* EP+Switch */
		/* store the slaves to master(ep) */
		ep_lp = netdev_priv(lp->master);
		ep_lp->slaves[temac_no] = ndev;
	}

	of_node_put(ep_node);
	return 0;
err_1:
	of_node_put(ep_node);
	return ret;
}

/**
 * axienet_device_reset - Reset and initialize the Axi Ethernet hardware.
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to reset and initialize the Axi Ethernet core. This
 * is typically called during initialization. It does a reset of the Axi DMA
 * Rx/Tx channels and initializes the Axi DMA BDs. Since Axi DMA reset lines
 * areconnected to Axi Ethernet reset lines, this in turn resets the Axi
 * Ethernet core. No separate hardware reset is done for the Axi Ethernet
 * core.
 */
static void axienet_device_reset(struct net_device *ndev)
{
	u32 axienet_status;
	struct axienet_local *lp = netdev_priv(ndev);

	lp->max_frm_size = XAE_MAX_VLAN_FRAME_SIZE;

	lp->options |= XAE_OPTION_VLAN;
	lp->options &= (~XAE_OPTION_JUMBO);

	if (ndev->mtu > XAE_MTU && ndev->mtu <= XAE_JUMBO_MTU) {
		lp->max_frm_size = ndev->mtu + VLAN_ETH_HLEN +
					XAE_TRL_SIZE;
		if (lp->max_frm_size <= lp->rxmem)
			lp->options |= XAE_OPTION_JUMBO;
	}

	axienet_status = axienet_ior(lp, XAE_RCW1_OFFSET);
	axienet_status &= ~XAE_RCW1_RX_MASK;
	axienet_iow(lp, XAE_RCW1_OFFSET, axienet_status);

	if (lp->axienet_config->mactype == XAXIENET_1G &&
	    !lp->eth_hasnobuf) {
		axienet_status = axienet_ior(lp, XAE_IP_OFFSET);
		if (axienet_status & XAE_INT_RXRJECT_MASK)
			axienet_iow(lp, XAE_IS_OFFSET, XAE_INT_RXRJECT_MASK);

		/* Enable Receive errors */
		axienet_iow(lp, XAE_IE_OFFSET, XAE_INT_RECV_ERROR_MASK);
	}

	axienet_iow(lp, XAE_FCC_OFFSET, XAE_FCC_FCRX_MASK);
	lp->axienet_config->setoptions(ndev, lp->options &
				       ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));

	axienet_set_mac_address_tsn(ndev, NULL);
	axienet_set_multicast_list_tsn(ndev);
	lp->axienet_config->setoptions(ndev, lp->options);

	netif_trans_update(ndev);
}

/**
 * axienet_tsn_open - TSN driver open routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *	    non-zero error value on failure
 *
 * This is the driver open routine. It calls phy_start to start the PHY device.
 * It also allocates interrupt service routines, enables the interrupt lines
 * and ISR handling. Axi Ethernet core is reset through Axi DMA core.
 */
int axienet_tsn_open(struct net_device *ndev)
{
	int ret = 0;
	struct axienet_local *lp = netdev_priv(ndev);
	struct phy_device *phydev = NULL;
	struct net_device *emac0_ndev;
	struct net_device *emac1_ndev;
	struct axienet_local *ep_node;
	u8 hw_addr_mask[ETH_ALEN];
	int i;

	axienet_device_reset(ndev);

	if (lp->phy_node) {
		phydev = of_phy_connect(lp->ndev, lp->phy_node,
					axienet_adjust_link_tsn,
					lp->phy_flags,
					lp->phy_mode);
		if (!phydev)
			dev_err(lp->dev, "of_phy_connect() failed\n");
		else
			phy_start(phydev);
	}
#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	INIT_WORK(&lp->tx_tstamp_work, axienet_tx_tstamp);
	skb_queue_head_init(&lp->ptp_txq);

	lp->ptp_rx_hw_pointer = 0;
	lp->ptp_rx_sw_pointer = 0xff;

	axienet_iow(lp, PTP_RX_CONTROL_OFFSET, PTP_RX_PACKET_CLEAR);

	ret = request_irq(lp->ptp_rx_irq, axienet_ptp_rx_irq,
			  0, "ptp_rx", ndev);
	if (ret)
		goto err_ptp_rx_irq;

	ret = request_irq(lp->ptp_tx_irq, axienet_ptp_tx_irq,
			  0, "ptp_tx", ndev);
	if (ret)
		goto err_ptp_tx_irq;
#endif

	if (lp->abl_reg & TSN_BRIDGEEP_EPONLY)
		tsn_data_path_open(ndev);

	if (lp->master) {
		ep_node = netdev_priv(lp->master);
		emac0_ndev = ep_node->slaves[0];
		emac1_ndev = ep_node->slaves[1];

		for (i = 0; i < ETH_ALEN; i++)
			hw_addr_mask[i] = 0xFF;
		hw_addr_mask[5] &= 0x000000F0;
		if (!ether_addr_equal_masked(emac0_ndev->dev_addr, emac1_ndev->dev_addr,
					     hw_addr_mask))
			netdev_warn(ndev, "MSB 44 bits of the MAC addresses of TSN EMAC0 and TSN EMAC1 are different");
		if (!ether_addr_equal_masked(emac0_ndev->dev_addr, ndev->dev_addr, hw_addr_mask))
			netdev_warn(ndev, "MSB 44 bits of the MAC addresses of TSN EMAC0 and TSN EP are different");
		if (!ether_addr_equal_masked(emac0_ndev->dev_addr, ndev->dev_addr, hw_addr_mask))
			netdev_warn(ndev, "MSB 44 bits of the MAC addresses of TSN EMAC1 and TSN EP are different");
	}

	netif_tx_start_all_queues(ndev);

	return ret;

#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
err_ptp_tx_irq:
	free_irq(lp->ptp_rx_irq, ndev);
err_ptp_rx_irq:
	return ret;
#endif
}

int axienet_tsn_stop(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);

#if IS_ENABLED(CONFIG_XILINX_TSN_PTP)
	free_irq(lp->ptp_tx_irq, ndev);
	free_irq(lp->ptp_rx_irq, ndev);
#endif
	if (lp->axienet_config->mactype == XAXIENET_1G && !lp->eth_hasnobuf)
		free_irq(lp->eth_irq, ndev);

	if (ndev->phydev)
		phy_disconnect(ndev->phydev);

	if (lp->abl_reg & TSN_BRIDGEEP_EPONLY)
		tsn_data_path_close(ndev);

	return 0;
}

static struct platform_driver tsn_ip_driver = {
	.probe = tsn_ip_probe,
	.remove = tsn_ip_remove,
	.driver = {
		 .name = "tsn_ip_axienet",
		 .of_match_table = tsn_ip_of_match,
	},
};

static int tsn_ip_init(void)
{
	int ret;

	ret = platform_driver_register(&tsn_ip_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&tsn_ep_driver);
	if (ret)
		goto err_unregister_ip;

	ret = platform_driver_register(&axienet_driver_tsn);
	if (ret)
		goto err_unregister_ep;

#if IS_ENABLED(CONFIG_XILINX_TSN_SWITCH)
	ret = platform_driver_register(&tsnswitch_driver);
	if (ret)
		goto err_unregister_axienet;
#endif

	ret = platform_driver_register(&tsn_ex_ep_driver);
	if (ret)
		goto err_unregister_switch;

	return 0;

err_unregister_switch:
#if IS_ENABLED(CONFIG_XILINX_TSN_SWITCH)
	platform_driver_unregister(&tsnswitch_driver);

err_unregister_axienet:
#endif
	platform_driver_unregister(&axienet_driver_tsn);

err_unregister_ep:
	platform_driver_unregister(&tsn_ep_driver);

err_unregister_ip:
	platform_driver_unregister(&tsn_ip_driver);
	return ret;
}

static void tsn_ip_exit(void)
{
	platform_driver_unregister(&tsn_ex_ep_driver);
#if IS_ENABLED(CONFIG_XILINX_TSN_SWITCH)
	platform_driver_unregister(&tsnswitch_driver);
#endif
	platform_driver_unregister(&axienet_driver_tsn);
	platform_driver_unregister(&tsn_ep_driver);
	platform_driver_unregister(&tsn_ip_driver);
}

module_init(tsn_ip_init);
module_exit(tsn_ip_exit);

MODULE_DESCRIPTION("Xilinx Axi Ethernet driver");
MODULE_AUTHOR("Xilinx");
MODULE_LICENSE("GPL");
