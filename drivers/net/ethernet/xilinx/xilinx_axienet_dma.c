// SPDX-License-Identifier: GPL-2.0

/* Xilinx AXI Ethernet (DMA programming)
 *
 * Copyright (c) 2008 Nissin Systems Co., Ltd.,  Yoshio Kashiwagi
 * Copyright (c) 2005-2008 DLA Systems,  David H. Lynch Jr. <dhlii@dlasys.net>
 * Copyright (c) 2008-2009 Secret Lab Technologies Ltd.
 * Copyright (c) 2010 - 2011 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2010 - 2011 PetaLogix
 * Copyright (c) 2010 - 2012 Xilinx, Inc.
 * Copyright (C) 2018 - 2022 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022 Advanced Micro Devices, Inc.
 *
 * This file contains helper functions for AXI DMA TX and RX programming.
 */

#include "xilinx_axienet.h"

/**
 * axienet_bd_free - Release buffer descriptor rings for individual dma queue
 * @ndev:	Pointer to the net_device structure
 * @q:		Pointer to DMA queue structure
 *
 * This function is helper function to axienet_dma_bd_release.
 */

void __maybe_unused axienet_bd_free(struct net_device *ndev,
				    struct axienet_dma_q *q)
{
	struct axienet_local *lp = netdev_priv(ndev);
	dma_addr_t phys;
	int i;

	for (i = 0; i < lp->rx_bd_num; i++) {
		phys = desc_get_phys_addr(lp, &q->rx_bd_v[i]);
		if (phys)
			dma_unmap_single(ndev->dev.parent, phys,
					 lp->max_frm_size, DMA_FROM_DEVICE);
		dev_kfree_skb((struct sk_buff *)
			      (q->rx_bd_v[i].sw_id_offset));
	}

	if (q->rx_bd_v) {
		dma_free_coherent(ndev->dev.parent,
				  sizeof(*q->rx_bd_v) * lp->rx_bd_num,
				  q->rx_bd_v,
				  q->rx_bd_p);
	}
	if (q->tx_bd_v) {
		dma_free_coherent(ndev->dev.parent,
				  sizeof(*q->tx_bd_v) * lp->tx_bd_num,
				  q->tx_bd_v,
				  q->tx_bd_p);
	}
	if (q->tx_bufs) {
		dma_free_coherent(ndev->dev.parent,
				  XAE_MAX_PKT_LEN * lp->tx_bd_num,
				  q->tx_bufs,
				  q->tx_bufs_dma);
	}
}

/**
 * __dma_txq_init - Setup buffer descriptor rings for individual Axi DMA-Tx
 * @ndev:	Pointer to the net_device structure
 * @q:		Pointer to DMA queue structure
 *
 * Return: 0, on success -ENOMEM, on failure
 *
 * This function is helper function to axienet_dma_q_init
 */
static int __dma_txq_init(struct net_device *ndev, struct axienet_dma_q *q)
{
	int i;
	struct axienet_local *lp = netdev_priv(ndev);

	q->tx_bd_ci = 0;
	q->tx_bd_tail = 0;

	q->tx_bd_v = dma_alloc_coherent(ndev->dev.parent,
					sizeof(*q->tx_bd_v) * lp->tx_bd_num,
					&q->tx_bd_p, GFP_KERNEL);
	if (!q->tx_bd_v)
		goto out;

	for (i = 0; i < lp->tx_bd_num; i++) {
		dma_addr_t addr = q->tx_bd_p +
				  sizeof(*q->tx_bd_v) *
				  ((i + 1) % lp->tx_bd_num);

		q->tx_bd_v[i].next = lower_32_bits(addr);
		if (lp->features & XAE_FEATURE_DMA_64BIT)
			q->tx_bd_v[i].next_msb = upper_32_bits(addr);
	}

	if (!q->eth_hasdre) {
		q->tx_bufs = dma_alloc_coherent(ndev->dev.parent,
						XAE_MAX_PKT_LEN * lp->tx_bd_num,
						&q->tx_bufs_dma,
						GFP_KERNEL);
		if (!q->tx_bufs)
			goto out;

		for (i = 0; i < lp->tx_bd_num; i++)
			q->tx_buf[i] = &q->tx_bufs[i * XAE_MAX_PKT_LEN];
	}
	return 0;
out:
	return -ENOMEM;
}

/**
 * __dma_rxq_init - Setup buffer descriptor rings for individual Axi DMA-Rx
 * @ndev:	Pointer to the net_device structure
 * @q:		Pointer to DMA queue structure
 *
 * Return: 0, on success -ENOMEM, on failure
 *
 * This function is helper function to axienet_dma_q_init
 */
static int __dma_rxq_init(struct net_device *ndev,
			  struct axienet_dma_q *q)
{
	int i;
	struct sk_buff *skb;
	struct axienet_local *lp = netdev_priv(ndev);
	/* Reset the indexes which are used for accessing the BDs */
	q->rx_bd_ci = 0;

	/* Allocate the Rx buffer descriptors. */
	q->rx_bd_v = dma_alloc_coherent(ndev->dev.parent,
					sizeof(*q->rx_bd_v) * lp->rx_bd_num,
					&q->rx_bd_p, GFP_KERNEL);
	if (!q->rx_bd_v)
		goto out;

	for (i = 0; i < lp->rx_bd_num; i++) {
		dma_addr_t addr;

		addr = q->rx_bd_p + sizeof(*q->rx_bd_v) *
			((i + 1) % lp->rx_bd_num);
		q->rx_bd_v[i].next = lower_32_bits(addr);
		if (lp->features & XAE_FEATURE_DMA_64BIT)
			q->rx_bd_v[i].next_msb = upper_32_bits(addr);

		skb = netdev_alloc_skb(ndev, lp->max_frm_size);
		if (!skb)
			goto out;

		/* Ensure that the skb is completely updated
		 * prio to mapping the DMA
		 */
		wmb();

		q->rx_bd_v[i].sw_id_offset = skb;

		addr = dma_map_single(lp->dev, skb->data,
				      lp->max_frm_size, DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(lp->dev, addr))) {
			netdev_err(ndev, "AXI DMA mapping error\n");
			goto out;
		}
		desc_set_phys_addr(lp, addr, &q->rx_bd_v[i]);

		q->rx_bd_v[i].cntrl = lp->max_frm_size;
	}

	return 0;
out:
	return -ENOMEM;
}

/**
 * axienet_dma_q_init - Setup buffer descriptor rings for individual Axi DMA
 * @ndev:	Pointer to the net_device structure
 * @q:		Pointer to DMA queue structure
 *
 * Return: 0, on success -ENOMEM, on failure
 *
 * This function is helper function to axienet_dma_bd_init
 */
int __maybe_unused axienet_dma_q_init(struct net_device *ndev,
				      struct axienet_dma_q *q)
{
	if (__dma_txq_init(ndev, q))
		goto out;

	if (__dma_rxq_init(ndev, q))
		goto out;

	axienet_dma_start(q);
	return 0;
out:
	axienet_dma_bd_release(ndev);
	return -ENOMEM;
}

/**
 * map_dma_q_irq - Map dma q based on interrupt number.
 * @irq:	irq number
 * @lp:		axienet local structure
 *
 * Return: DMA queue.
 *
 * This returns the DMA number on which interrupt has occurred.
 */
static int map_dma_q_irq(int irq, struct axienet_local *lp)
{
	int i;

	for_each_rx_dma_queue(lp, i) {
		if (irq == lp->dq[i]->tx_irq || irq == lp->dq[i]->rx_irq)
			return i;
	}
	pr_err("Error mapping DMA irq\n");
	return -ENODEV;
}

/**
 * axienet_tx_irq - Tx Done Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a TX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Tx done Isr.
 */
irqreturn_t __maybe_unused axienet_tx_irq(int irq, void *_ndev)
{
	u32 cr;
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct axienet_local *lp = netdev_priv(ndev);
	int i = map_dma_q_irq(irq, lp);
	struct axienet_dma_q *q;

	if (i < 0)
		return IRQ_NONE;

	q = lp->dq[i];

	status = axienet_dma_in32(q, XAXIDMA_TX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		/* Disable further TX completion interrupts and schedule
		 * NAPI to handle the completions.
		 */
		axienet_dma_out32(q, XAXIDMA_TX_SR_OFFSET, status);
		cr = q->tx_dma_cr;

		cr &= ~(XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);
		if (napi_schedule_prep(&q->napi_tx)) {
			axienet_dma_out32(q, XAXIDMA_TX_CR_OFFSET, cr);
			__napi_schedule(&q->napi_tx);
		}
		goto out;
	}

	if (!(status & XAXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;
	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		dev_err(&ndev->dev, "DMA Tx error 0x%x\n", status);
		dev_err(&ndev->dev, "Current BD is at: 0x%x%08x\n",
			q->tx_bd_v[q->tx_bd_ci].phys_msb,
			q->tx_bd_v[q->tx_bd_ci].phys);

		cr = axienet_dma_in32(q, XAXIDMA_TX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Write to the Tx channel control register */
		axienet_dma_out32(q, XAXIDMA_TX_CR_OFFSET, cr);

		cr = axienet_dma_in32(q, XAXIDMA_RX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Write to the Rx channel control register */
		axienet_dma_out32(q, XAXIDMA_RX_CR_OFFSET, cr);

		tasklet_schedule(&lp->dma_err_tasklet[i]);
		axienet_dma_out32(q, XAXIDMA_TX_SR_OFFSET, status);
	}
out:
	return IRQ_HANDLED;
}

/**
 * axienet_rx_irq - Rx Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a RX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Rx Isr. It invokes "axienet_recv" to complete the BD
 * processing.
 */
irqreturn_t __maybe_unused axienet_rx_irq(int irq, void *_ndev)
{
	u32 cr;
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct axienet_local *lp = netdev_priv(ndev);
	int i = map_dma_q_irq(irq, lp);
	struct axienet_dma_q *q;

	if (i < 0)
		return IRQ_NONE;

	q = lp->dq[i];

	status = axienet_dma_in32(q, XAXIDMA_RX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		cr = axienet_dma_in32(q, XAXIDMA_RX_CR_OFFSET);
		cr &= ~(XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);
		if (napi_schedule_prep(&q->napi_rx)) {
			axienet_dma_out32(q, XAXIDMA_RX_CR_OFFSET, cr);
			__napi_schedule(&q->napi_rx);
		}
	}

	if (!(status & XAXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;

	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		dev_err(&ndev->dev, "DMA Rx error 0x%x\n", status);
		dev_err(&ndev->dev, "Current BD is at: 0x%x%08x\n",
			q->rx_bd_v[q->rx_bd_ci].phys_msb,
			q->rx_bd_v[q->rx_bd_ci].phys);

		cr = axienet_dma_in32(q, XAXIDMA_TX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Finally write to the Tx channel control register */
		axienet_dma_out32(q, XAXIDMA_TX_CR_OFFSET, cr);

		cr = axienet_dma_in32(q, XAXIDMA_RX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
			/* write to the Rx channel control register */
		axienet_dma_out32(q, XAXIDMA_RX_CR_OFFSET, cr);

		tasklet_schedule(&lp->dma_err_tasklet[i]);
		axienet_dma_out32(q, XAXIDMA_RX_SR_OFFSET, status);
	}

	return IRQ_HANDLED;
}

/**
 * axienet_dma_err_handler - Tasklet handler for Axi DMA Error
 * @data:	Data passed
 *
 * Resets the Axi DMA and Axi Ethernet devices, and reconfigures the
 * Tx/Rx BDs.
 */
void __maybe_unused axienet_dma_err_handler(unsigned long data)
{
	u32 axienet_status;
	u32 i;
	struct axienet_dma_q *q = (struct axienet_dma_q *)data;
	struct axienet_local *lp = q->lp;
	struct net_device *ndev = lp->ndev;
	struct axidma_bd *cur_p;

	/* Don't bother if we are going to stop anyway */
	if (READ_ONCE(lp->stopping))
		return;

	napi_disable(&q->napi_tx);
	napi_disable(&q->napi_rx);

	lp->axienet_config->setoptions(ndev, lp->options &
				       ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));
	/* When we do an Axi Ethernet reset, it resets the complete core
	 * including the MDIO. MDIO must be disabled before resetting.
	 * Hold MDIO bus lock to avoid MDIO accesses during the reset.
	 */
	axienet_dma_stop(q);

	for (i = 0; i < lp->tx_bd_num; i++) {
		cur_p = &q->tx_bd_v[i];
		if (cur_p->cntrl) {
			dma_addr_t addr = desc_get_phys_addr(lp, cur_p);

			dma_unmap_single(ndev->dev.parent, addr,
					 (cur_p->cntrl &
					  XAXIDMA_BD_CTRL_LENGTH_MASK),
					 DMA_TO_DEVICE);
		}
		if (cur_p->tx_skb)
			dev_kfree_skb_irq((struct sk_buff *)cur_p->tx_skb);
		cur_p->phys = 0;
		cur_p->phys_msb = 0;
		cur_p->cntrl = 0;
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
		cur_p->sw_id_offset = 0;
		cur_p->tx_skb = 0;
	}

	for (i = 0; i < lp->rx_bd_num; i++) {
		cur_p = &q->rx_bd_v[i];
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
	}

	q->tx_bd_ci = 0;
	q->tx_bd_tail = 0;
	q->rx_bd_ci = 0;

	axienet_dma_start(q);
	if (lp->axienet_config->mactype != XAXIENET_10G_25G) {
		axienet_status = axienet_ior(lp, XAE_RCW1_OFFSET);
		axienet_status &= ~XAE_RCW1_RX_MASK;
		axienet_iow(lp, XAE_RCW1_OFFSET, axienet_status);
	}

	if (lp->axienet_config->mactype == XAXIENET_1_2p5G &&
	    !lp->eth_hasnobuf) {
		axienet_status = axienet_ior(lp, XAE_IP_OFFSET);
		if (axienet_status & XAE_INT_RXRJECT_MASK)
			axienet_iow(lp, XAE_IS_OFFSET, XAE_INT_RXRJECT_MASK);
	}

	if (lp->axienet_config->mactype != XAXIENET_10G_25G)
		axienet_iow(lp, XAE_FCC_OFFSET, XAE_FCC_FCRX_MASK);

	lp->axienet_config->setoptions(ndev, lp->options &
			   ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));
	axienet_set_mac_address(ndev, NULL);
	axienet_set_multicast_list(ndev);
	napi_enable(&q->napi_rx);
	napi_enable(&q->napi_tx);
	lp->axienet_config->setoptions(ndev, lp->options);
}
