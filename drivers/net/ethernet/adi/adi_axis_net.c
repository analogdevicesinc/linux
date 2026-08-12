// SPDX-License-Identifier: GPL-2.0
/*
 * Point-to-point Ethernet over a pair of AXI-Stream DMA channels.
 *
 * Carries Ethernet frames between two processors joined by AXI-Stream, using
 * one axi-dmac instance per direction on each side:
 *
 *   this side                                      far side
 *   ---------                                      --------
 *   tx: MEM_TO_DEV dmac --> AXIS FIFO --> DEV_TO_MEM dmac :rx
 *   rx: DEV_TO_MEM dmac <-- AXIS FIFO <-- MEM_TO_DEV dmac :tx
 *
 * The driver is deliberately side-agnostic: it only ever sees "a channel that
 * takes memory and produces a stream" and "a channel that consumes a stream
 * and writes memory". Which address space that memory lives in is resolved
 * entirely by the DMA API on the channel's device, so the same driver binds
 * unmodified on both ends of the link. On a PCIe-attached FPGA the DMAC's
 * platform device inherits forwarding dma_map_ops from the endpoint and the
 * addresses below are IOVAs; on the local processor they are physical
 * addresses. Nothing here needs to know.
 *
 * Framing
 * -------
 * There is nothing on the wire but the Ethernet frame: one frame per
 * descriptor, no in-band header. Length and boundary both come from the
 * hardware, which is why the axi-dmac instances need TLAST and TKEEP:
 *
 *  - TLAST closes the receive descriptor at the end of the frame. Without it a
 *    posted max-size descriptor stays open until enough bytes arrive to fill
 *    it and consecutive frames coalesce.
 *  - TKEEP marks the valid bytes of the trailing beat. Without it the
 *    receiving DMAC can only report whole beats, so DMA_LENGTH_ALIGN is the
 *    bus width and the measured length is rounded up to the next beat.
 *
 * With both, the reported transfer length is byte exact and the frame length is
 * rx_buf_size - residue. Three hardware properties are load bearing here, and
 * all three fail silently by making the residue useless rather than by
 * refusing to work:
 *
 *  - HAS_AXIS_TKEEP set on every instance, transmit and receive. On a stream
 *    destination it generates TKEEP; on a stream source it consumes it.
 *  - a pcore version of at least 4.2.a, which is where axi_dmac gains
 *    hw_partial_xfer and so reports a short transfer at all.
 *  - DMA_SG_TRANSFER off. axi_dmac_compute_residue() returns 0 unconditionally
 *    in hardware-SG mode, so residue would always read as a full buffer.
 *
 * A residue of zero means one of those is missing, or that a frame filled the
 * descriptor without a TLAST -- an MTU mismatch between the ends. rx_buf_size
 * is deliberately sized larger than the longest legal frame so a well-formed
 * frame always leaves a non-zero residue, which makes the two cases one check.
 *
 * Buffers
 * -------
 * Transmit copies into a preallocated coherent ring; receive hands its buffers
 * to the stack without copying. The asymmetry is forced by axi_dmac's
 * per-segment alignment rules, not chosen.
 *
 * axi_dmac_prep_slave_sg() validates every scatterlist entry with
 * axi_dmac_check_addr() and axi_dmac_check_len() and frees the whole descriptor
 * if any entry fails. That applies to interior entries, not just the last one,
 * because each segment is a separate AXI burst restarted at a fresh address
 * (see axi_dmac_start_transfer()) -- so no segment may begin or end mid-beat.
 * Transmitting an skb in place needs one entry per fragment, and neither
 * fragment offsets nor fragment lengths are multiples of the bus width. The
 * length constraint in particular cannot be fixed with headroom. So TX copies
 * into a page-aligned coherent slot.
 *
 * The transfer itself is not padded: with TKEEP on the transmit instance
 * DMA_LENGTH_ALIGN is zero, so an arbitrary frame length is accepted as-is and
 * the trailing beat is marked. Only the start address has to be aligned, and a
 * coherent allocation is.
 *
 * Receive is a single segment and has no such problem. A max-size buffer is
 * posted with skb->data aligned to %ADI_AXIS_NET_DMA_ALIGN, TLAST closes the
 * descriptor at the true frame end, and the residue says how much of the buffer
 * went unused. The buffer is then passed up and replaced.
 *
 * This is unrelated to axi_dmac's hw_sg capability (DMA_SG_TRANSFER), which
 * only decides whether the DMAC walks the segment list itself or the driver
 * feeds segments one at a time from the completion handler. Both modes support
 * multi-segment descriptors and both emit one TLAST per descriptor.
 *
 * Receive therefore pays a dma_map_single()/dma_unmap_single() pair per frame,
 * which on a translating host is an IOVA allocation plus an invalidation. That
 * replaces a full-frame memcpy, and %ADI_AXIS_NET_COPYBREAK keeps it off the
 * small-frame path where the copy was the cheaper of the two anyway.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/*
 * Ring depth, per direction. Must be a power of two. Deep enough that a burst
 * of small frames does not stall on completion latency, shallow enough that
 * the coherent footprint stays modest: 64 slots x 2 directions x ~9 KiB is
 * a little over 1 MiB.
 */
#define ADI_AXIS_NET_RING_SIZE		64

/*
 * Alignment applied to every DMA address. Transfer lengths need none, since
 * TKEEP makes DMA_LENGTH_ALIGN zero on both instances. Chosen to satisfy any
 * AXI data width up to 512-bit without having to interrogate the channel:
 * axi_dmac derives address_align_mask from the bus width
 * (max(dest_width, src_width) - 1) and there is no reliable public accessor
 * for it. dma_device.src_addr_widths / dst_addr_widths are populated as
 * BIT(width_in_bytes), which overflows for a 256-bit bus and so cannot be
 * used. Over-aligning costs a few padding bytes per frame and removes the
 * question.
 */
#define ADI_AXIS_NET_DMA_ALIGN		64

#define ADI_AXIS_NET_MIN_MTU		ETH_MIN_MTU
#define ADI_AXIS_NET_MAX_MTU		8192
#define ADI_AXIS_NET_DEFAULT_MTU	ETH_DATA_LEN

/*
 * Frames at or below this length are copied out and the receive buffer is
 * recycled instead of being handed to the stack. Two reasons, both about the
 * buffer being posted at full size regardless of how much arrives:
 *
 *  - skb->truesize would report the whole slot, so a 64-byte ARP frame would
 *    charge several kilobytes against the receiving socket.
 *  - the replacement buffer is a multi-page GFP_ATOMIC allocation made from
 *    the completion callback, which is exactly where it is least likely to
 *    succeed. Recycling avoids asking.
 */
#define ADI_AXIS_NET_COPYBREAK		256

/**
 * struct adi_axis_net_slot - one transmit ring entry
 * @vaddr: CPU address of the coherent buffer
 * @daddr: DMA address of the same buffer, valid for the owning channel
 * @priv: owning device, so completion callbacks need only this pointer
 * @index: position in the ring
 * @busy: set while a descriptor referencing this slot is outstanding
 */
struct adi_axis_net_slot {
	void *vaddr;
	dma_addr_t daddr;
	struct adi_axis_net *priv;
	unsigned int index;
	bool busy;
};

/**
 * struct adi_axis_net_rx_slot - one receive ring entry
 * @skb: buffer the DMAC is filling, or NULL if the ring entry is empty
 * @daddr: streaming mapping of @skb->data, valid while @mapped is set
 * @priv: owning device, so completion callbacks need only this pointer
 * @index: position in the ring
 * @mapped: set between a successful dma_map_single() and its unmap
 *
 * Unlike the transmit ring there is no coherent allocation and no @busy flag:
 * an skb is attached when the slot is first posted and stays attached until it
 * is handed to the stack and immediately replaced, so @skb is never NULL while
 * the interface is up. @mapped exists because ndo_stop() has to release the
 * mappings of descriptors that dmaengine_terminate_sync() discarded, whose
 * callbacks will never run.
 */
struct adi_axis_net_rx_slot {
	struct sk_buff *skb;
	dma_addr_t daddr;
	struct adi_axis_net *priv;
	unsigned int index;
	bool mapped;
};

/**
 * struct adi_axis_net - per-interface state
 * @dev: platform device this driver bound to
 * @netdev: the network interface
 * @tx_chan: MEM_TO_DEV channel, drains host memory into the stream
 * @rx_chan: DEV_TO_MEM channel, fills host memory from the stream
 * @tx_dma_dev: device TX buffers are allocated against
 * @rx_dma_dev: device RX buffers are allocated against
 * @tx: TX ring
 * @rx: RX ring
 * @tx_lock: protects @tx slot ownership against the completion tasklet
 * @tx_next: next slot to try for transmit
 * @buf_size: bytes per TX slot, already rounded to %ADI_AXIS_NET_DMA_ALIGN
 * @rx_buf_size: bytes posted per RX descriptor, sized from the MTU in force
 *	when the interface was brought up and rounded to
 *	%ADI_AXIS_NET_DMA_ALIGN. Separate from @buf_size so a 1500-byte MTU does
 *	not charge every delivered skb with a jumbo-sized truesize.
 * @running: cleared before teardown so completions stop resubmitting
 *
 * @tx_dma_dev and @rx_dma_dev are looked up once via
 * dmaengine_get_dma_device() rather than assumed to be @dev. The two channels
 * may live on different controllers, and on a PCIe-attached FPGA it is
 * specifically the controller's platform device that carries the forwarding
 * dma_map_ops -- allocating against @dev instead would produce addresses in
 * the wrong address space.
 */
struct adi_axis_net {
	struct device *dev;
	struct net_device *netdev;

	struct dma_chan *tx_chan;
	struct dma_chan *rx_chan;
	struct device *tx_dma_dev;
	struct device *rx_dma_dev;

	struct adi_axis_net_slot tx[ADI_AXIS_NET_RING_SIZE];
	struct adi_axis_net_rx_slot rx[ADI_AXIS_NET_RING_SIZE];

	spinlock_t tx_lock; /* protects tx[].busy, tx_next */
	unsigned int tx_next;

	size_t buf_size;
	size_t rx_buf_size;
	bool running;
};

static void adi_axis_net_free_ring(struct device *dma_dev,
				   struct adi_axis_net_slot *ring,
				   size_t buf_size)
{
	unsigned int i;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		if (!ring[i].vaddr)
			continue;
		dma_free_coherent(dma_dev, buf_size, ring[i].vaddr,
				  ring[i].daddr);
		ring[i].vaddr = NULL;
	}
}

static int adi_axis_net_alloc_ring(struct adi_axis_net *priv,
				   struct device *dma_dev,
				   struct adi_axis_net_slot *ring)
{
	unsigned int i;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		ring[i].vaddr = dma_alloc_coherent(dma_dev, priv->buf_size,
						   &ring[i].daddr, GFP_KERNEL);
		if (!ring[i].vaddr) {
			adi_axis_net_free_ring(dma_dev, ring, priv->buf_size);
			return -ENOMEM;
		}

		/*
		 * Coherent allocations are page-aligned, so this should never
		 * fire. Check anyway: a misaligned address is rejected by
		 * axi_dmac_prep_slave_sg() with a bare NULL return, which is
		 * indistinguishable from allocation failure at the call site.
		 */
		if (!IS_ALIGNED(ring[i].daddr, ADI_AXIS_NET_DMA_ALIGN)) {
			dev_err(priv->dev,
				"DMA address %pad not %u-byte aligned\n",
				&ring[i].daddr, ADI_AXIS_NET_DMA_ALIGN);
			adi_axis_net_free_ring(dma_dev, ring, priv->buf_size);
			return -EINVAL;
		}

		ring[i].priv = priv;
		ring[i].index = i;
		ring[i].busy = false;
	}

	return 0;
}

/*
 * Allocate a receive buffer with skb->data aligned to %ADI_AXIS_NET_DMA_ALIGN,
 * as axi_dmac_check_addr() requires. The overallocation is the worst-case
 * adjustment; netdev_alloc_skb() only guarantees NET_SKB_PAD, which is 32 on
 * some configurations and in any case says nothing about the absolute address.
 *
 * No NET_IP_ALIGN reservation is made, because it cannot be: the frame starts
 * where the DMAC writes, and that address has to be %ADI_AXIS_NET_DMA_ALIGN
 * aligned, which leaves the IP header at offset 14 -- 2 mod 4. Offsetting by
 * NET_IP_ALIGN to fix that would break the alignment axi_dmac_check_addr()
 * requires. Both ends of this link tolerate unaligned loads, and the
 * copybreak path below uses netdev_alloc_skb_ip_align() where it is free.
 */
static struct sk_buff *adi_axis_net_alloc_rx_skb(struct adi_axis_net *priv)
{
	struct sk_buff *skb;
	unsigned int pad;

	skb = netdev_alloc_skb(priv->netdev,
			       priv->rx_buf_size + ADI_AXIS_NET_DMA_ALIGN);
	if (!skb)
		return NULL;

	pad = ALIGN((unsigned long)skb->data, ADI_AXIS_NET_DMA_ALIGN) -
	      (unsigned long)skb->data;
	if (pad)
		skb_reserve(skb, pad);

	return skb;
}

/*
 * Attach a buffer to @slot if it has none, map it, and post a descriptor with
 * @complete as its completion callback. Called from ndo_open and, once the
 * completion handler is defined below, from the handler itself to repost --
 * @complete is a parameter so that recursion does not need a forward
 * declaration.
 *
 * The descriptor is posted at the full @rx_buf_size. The far side sends fewer
 * bytes and relies on TLAST to close it early, leaving the unused tail as the
 * residue. A frame longer than @rx_buf_size -- an MTU mismatch between the two
 * ends -- fills the descriptor without a TLAST and so reports no residue at
 * all, which the completion handler treats as an error.
 */
static int adi_axis_net_submit_rx(struct adi_axis_net *priv,
				  struct adi_axis_net_rx_slot *slot,
				  dma_async_tx_callback_result complete)
{
	struct dma_async_tx_descriptor *desc;

	if (!priv->running)
		return -ESHUTDOWN;

	if (!slot->skb) {
		slot->skb = adi_axis_net_alloc_rx_skb(priv);
		if (!slot->skb)
			return -ENOMEM;
	}

	slot->daddr = dma_map_single(priv->rx_dma_dev, slot->skb->data,
				     priv->rx_buf_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->rx_dma_dev, slot->daddr))
		return -ENOMEM;

	slot->mapped = true;

	/*
	 * dma_map_single() preserves the offset within the page, so an aligned
	 * virtual address yields an aligned handle. Check anyway: a misaligned
	 * address is rejected by axi_dmac_prep_slave_sg() with a bare NULL,
	 * which is indistinguishable from allocation failure at the call site.
	 */
	if (!IS_ALIGNED(slot->daddr, ADI_AXIS_NET_DMA_ALIGN)) {
		netdev_err_once(priv->netdev,
				"RX DMA address %pad not %u-byte aligned\n",
				&slot->daddr, ADI_AXIS_NET_DMA_ALIGN);
		goto err_unmap;
	}

	desc = dmaengine_prep_slave_single(priv->rx_chan, slot->daddr,
					   priv->rx_buf_size, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT);
	if (!desc)
		goto err_unmap;

	desc->callback_result = complete;
	desc->callback_param = slot;

	if (dma_submit_error(dmaengine_submit(desc)))
		goto err_unmap;

	return 0;

err_unmap:
	dma_unmap_single(priv->rx_dma_dev, slot->daddr, priv->rx_buf_size,
			 DMA_FROM_DEVICE);
	slot->mapped = false;
	return -EIO;
}

/*
 * Receive completion. Callbacks for one channel are serialised by dmaengine's
 * per-channel tasklet, so the ring needs no lock of its own.
 *
 * Every exit path leaves the slot posted with a buffer attached, so the ring
 * never shrinks: the replacement buffer is allocated before the filled one is
 * given away, and if that allocation fails the frame is dropped and the
 * existing buffer reposted.
 */
static void adi_axis_net_rx_callback(void *param,
				     const struct dmaengine_result *res)
{
	struct adi_axis_net_rx_slot *slot = param;
	struct adi_axis_net *priv = slot->priv;
	struct net_device *netdev = priv->netdev;
	struct sk_buff *skb = NULL;
	unsigned int len;
	int ret;

	dma_unmap_single(priv->rx_dma_dev, slot->daddr, priv->rx_buf_size,
			 DMA_FROM_DEVICE);
	slot->mapped = false;

	if (!priv->running)
		return;

	if (res->result != DMA_TRANS_NOERROR) {
		netdev_dbg(netdev, "RX transfer error %d\n", res->result);
		netdev->stats.rx_errors++;
		goto repost;
	}

	/*
	 * @rx_buf_size exceeds the longest legal frame, so a frame that arrived
	 * with a TLAST always leaves something unconsumed. A zero residue means
	 * either the descriptor filled without one -- the far side's MTU is
	 * larger than ours -- or the hardware cannot report a short transfer at
	 * all. See the residue prerequisites at the top of this file.
	 */
	if (!res->residue) {
		netdev_warn_once(netdev,
				 "RX descriptor reported no residue; check the far side MTU and that the DMAC has HAS_AXIS_TKEEP with DMA_SG_TRANSFER off\n");
		netdev->stats.rx_length_errors++;
		goto repost;
	}

	/*
	 * Bound the residue rather than the difference: @len is derived by
	 * subtraction, so a residue larger than the buffer would wrap it to
	 * something enormous instead of producing a runt.
	 */
	if (res->residue > priv->rx_buf_size - ETH_HLEN) {
		netdev_dbg(netdev, "residue %u leaves no frame, dropping\n",
			   res->residue);
		netdev->stats.rx_length_errors++;
		goto repost;
	}

	len = priv->rx_buf_size - res->residue;

	if (len <= ADI_AXIS_NET_COPYBREAK) {
		skb = netdev_alloc_skb_ip_align(netdev, len);
		if (!skb) {
			netdev->stats.rx_dropped++;
			goto repost;
		}
		skb_put_data(skb, slot->skb->data, len);
	} else {
		struct sk_buff *replacement = adi_axis_net_alloc_rx_skb(priv);

		if (!replacement) {
			netdev->stats.rx_dropped++;
			goto repost;
		}

		skb = slot->skb;
		slot->skb = replacement;

		skb_put(skb, len);
	}

	skb->protocol = eth_type_trans(skb, netdev);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	netif_rx(skb);

repost:
	/* -ESHUTDOWN just means ndo_stop got here first. */
	ret = adi_axis_net_submit_rx(priv, slot, adi_axis_net_rx_callback);
	if (!ret)
		dma_async_issue_pending(priv->rx_chan);
	else if (ret != -ESHUTDOWN)
		netdev_err_once(netdev, "failed to repost RX slot %u: %d\n",
				slot->index, ret);
}

static void adi_axis_net_tx_callback(void *param)
{
	struct adi_axis_net_slot *slot = param;
	struct adi_axis_net *priv = slot->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);
	slot->busy = false;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (netif_queue_stopped(priv->netdev) && priv->running)
		netif_wake_queue(priv->netdev);
}

static netdev_tx_t adi_axis_net_start_xmit(struct sk_buff *skb,
					   struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	struct dma_async_tx_descriptor *desc;
	struct adi_axis_net_slot *slot;
	unsigned int len = skb->len;
	unsigned long flags;

	if (len > priv->buf_size) {
		netdev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&priv->tx_lock, flags);

	slot = &priv->tx[priv->tx_next & (ADI_AXIS_NET_RING_SIZE - 1)];
	if (slot->busy) {
		/*
		 * Should be unreachable: the queue is stopped below as soon as
		 * the ring fills, and NETDEV_TX_BUSY is "a hard error unless
		 * there is no way your device can tell ahead of time"
		 * (Documentation/networking/driver.rst). Kept as a backstop
		 * only. No reference to the skb is retained and it is not
		 * freed, as the contract requires.
		 */
		netif_stop_queue(netdev);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return NETDEV_TX_BUSY;
	}

	slot->busy = true;
	priv->tx_next++;

	/* Stop in advance if that consumed the last free slot. */
	if (priv->tx[priv->tx_next & (ADI_AXIS_NET_RING_SIZE - 1)].busy)
		netif_stop_queue(netdev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

	/*
	 * skb_copy_bits() rather than skb_copy_from_linear_data(): the skb may
	 * be non-linear and the latter would silently copy only the head,
	 * transmitting garbage for the fragments.
	 */
	if (skb_copy_bits(skb, 0, slot->vaddr, len))
		goto err_drop;

	/*
	 * The exact frame length, unpadded. axi_dmac_check_len() would reject a
	 * length that is not a multiple of the bus width, but with TKEEP on this
	 * instance DMA_LENGTH_ALIGN is zero and there is nothing to round to.
	 */
	desc = dmaengine_prep_slave_single(priv->tx_chan, slot->daddr, len,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc)
		goto err_drop;

	desc->callback = adi_axis_net_tx_callback;
	desc->callback_param = slot;

	if (dma_submit_error(dmaengine_submit(desc)))
		goto err_drop;

	dma_async_issue_pending(priv->tx_chan);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += len;

	/* The payload is copied, so the skb is not needed past this point. */
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;

err_drop:
	spin_lock_irqsave(&priv->tx_lock, flags);
	slot->busy = false;
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	netdev->stats.tx_errors++;
	dev_kfree_skb_any(skb);

	/*
	 * Releasing the slot may have made room, but only wake a queue that was
	 * actually stopped, and never during teardown -- ndo_stop clears
	 * @running after stopping the queue and does not expect it back.
	 */
	if (netif_queue_stopped(netdev) && priv->running)
		netif_wake_queue(netdev);

	return NETDEV_TX_OK;
}

/*
 * Release every receive buffer and any mapping still outstanding. Called after
 * dmaengine_terminate_sync(), which discards queued descriptors without
 * running their callbacks -- so those mappings have to be torn down here.
 */
static void adi_axis_net_free_rx_ring(struct adi_axis_net *priv)
{
	unsigned int i;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		struct adi_axis_net_rx_slot *slot = &priv->rx[i];

		if (slot->mapped) {
			dma_unmap_single(priv->rx_dma_dev, slot->daddr,
					 priv->rx_buf_size, DMA_FROM_DEVICE);
			slot->mapped = false;
		}

		dev_kfree_skb(slot->skb);
		slot->skb = NULL;
	}
}

/*
 * Stop the receive channel and release its buffers. @running must already be
 * clear.
 *
 * The second terminate_sync() is not redundant. A completion that sampled
 * @running as true just before it was cleared can reach its repost after the
 * first terminate has already drained the queue, leaving a live descriptor
 * behind. Once the first terminate_sync() returns no callback is running, so
 * the clear is visible to every subsequent one and no further repost can
 * happen; the second terminate discards whatever that last racing repost
 * left. Without it, free_rx_ring() can unmap and free a buffer the engine
 * still owns.
 */
static void adi_axis_net_quiesce_rx(struct adi_axis_net *priv)
{
	dmaengine_terminate_sync(priv->rx_chan);
	dmaengine_terminate_sync(priv->rx_chan);

	adi_axis_net_free_rx_ring(priv);
}

static int adi_axis_net_open(struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	unsigned int i;
	int ret;

	/*
	 * Size receive buffers from the MTU in force now rather than from
	 * max_mtu, so a 1500-byte MTU does not charge a jumbo-sized truesize
	 * against every socket. There is no ndo_change_mtu, so the core writes
	 * netdev->mtu directly and a change while the interface is up would not
	 * be seen here; a frame that no longer fits is caught by the residue
	 * check, and the link comes back to the new size on the next ifdown/up.
	 *
	 * The + 1 before rounding guarantees the buffer is strictly larger than
	 * the longest legal frame even when that length is already aligned, so a
	 * frame terminated by TLAST can never consume the descriptor exactly.
	 * That is what lets a zero residue mean unambiguously "no TLAST".
	 */
	priv->rx_buf_size = ALIGN(VLAN_ETH_HLEN + READ_ONCE(netdev->mtu) + 1,
				  ADI_AXIS_NET_DMA_ALIGN);

	priv->running = true;

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		ret = adi_axis_net_submit_rx(priv, &priv->rx[i],
					     adi_axis_net_rx_callback);
		if (ret) {
			netdev_err(netdev, "failed to post RX slot %u: %d\n",
				   i, ret);
			goto err_terminate;
		}
	}

	dma_async_issue_pending(priv->rx_chan);

	netif_start_queue(netdev);
	netif_carrier_on(netdev);
	return 0;

err_terminate:
	priv->running = false;
	adi_axis_net_quiesce_rx(priv);

	return ret;
}

static int adi_axis_net_stop(struct net_device *netdev)
{
	struct adi_axis_net *priv = netdev_priv(netdev);
	unsigned int i;

	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	/*
	 * Clear @running before terminating so that a completion racing us
	 * neither reposts an RX slot nor rewakes the queue. terminate_sync()
	 * then guarantees no callback is still in flight, after which the
	 * rings can be torn down without holding tx_lock.
	 */
	priv->running = false;

	dmaengine_terminate_sync(priv->tx_chan);

	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++)
		priv->tx[i].busy = false;

	adi_axis_net_quiesce_rx(priv);

	return 0;
}

static const struct net_device_ops adi_axis_net_netdev_ops = {
	.ndo_open		= adi_axis_net_open,
	.ndo_stop		= adi_axis_net_stop,
	.ndo_start_xmit		= adi_axis_net_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

/*
 * Both channels must be single-direction slave channels pointing the right
 * way. axi_dmac advertises exactly one direction per instance
 * (dma_dev->directions = BIT(chan.direction)) and its prep_slave_sg() rejects
 * a mismatch with a bare NULL, so checking here turns a confusing
 * "failed to prepare descriptor" at runtime into a clear probe failure.
 */
static int adi_axis_net_check_chan(struct device *dev, struct dma_chan *chan,
				   enum dma_transfer_direction dir,
				   const char *name)
{
	if (!(chan->device->directions & BIT(dir))) {
		dev_err(dev, "\"%s\" channel does not support %s\n", name,
			dir == DMA_MEM_TO_DEV ? "MEM_TO_DEV" : "DEV_TO_MEM");
		return -EINVAL;
	}

	return 0;
}

static void adi_axis_net_release_chan(void *chan)
{
	dma_release_channel(chan);
}

static int adi_axis_net_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct adi_axis_net *priv;
	unsigned int i;
	int ret;

	netdev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	SET_NETDEV_DEV(netdev, dev);
	platform_set_drvdata(pdev, netdev);

	priv = netdev_priv(netdev);
	priv->dev = dev;
	priv->netdev = netdev;
	spin_lock_init(&priv->tx_lock);

	priv->tx_chan = dma_request_chan(dev, "tx");
	if (IS_ERR(priv->tx_chan))
		return dev_err_probe(dev, PTR_ERR(priv->tx_chan),
				     "failed to request \"tx\" DMA channel\n");

	ret = devm_add_action_or_reset(dev, adi_axis_net_release_chan,
				       priv->tx_chan);
	if (ret)
		return ret;

	priv->rx_chan = dma_request_chan(dev, "rx");
	if (IS_ERR(priv->rx_chan))
		return dev_err_probe(dev, PTR_ERR(priv->rx_chan),
				     "failed to request \"rx\" DMA channel\n");

	ret = devm_add_action_or_reset(dev, adi_axis_net_release_chan,
				       priv->rx_chan);
	if (ret)
		return ret;

	ret = adi_axis_net_check_chan(dev, priv->tx_chan, DMA_MEM_TO_DEV, "tx");
	if (ret)
		return ret;

	ret = adi_axis_net_check_chan(dev, priv->rx_chan, DMA_DEV_TO_MEM, "rx");
	if (ret)
		return ret;

	/*
	 * Allocate against the controllers, not against @dev. See the
	 * tx_dma_dev/rx_dma_dev note in struct adi_axis_net.
	 */
	priv->tx_dma_dev = dmaengine_get_dma_device(priv->tx_chan);
	priv->rx_dma_dev = dmaengine_get_dma_device(priv->rx_chan);

	priv->buf_size = ALIGN(VLAN_ETH_HLEN + ADI_AXIS_NET_MAX_MTU,
			       ADI_AXIS_NET_DMA_ALIGN);

	ret = adi_axis_net_alloc_ring(priv, priv->tx_dma_dev, priv->tx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to allocate TX ring\n");

	/*
	 * The receive ring holds no allocation until ndo_open: its buffers are
	 * skbs sized from the MTU then in force, not coherent slots.
	 */
	for (i = 0; i < ADI_AXIS_NET_RING_SIZE; i++) {
		priv->rx[i].priv = priv;
		priv->rx[i].index = i;
	}

	netdev->netdev_ops = &adi_axis_net_netdev_ops;
	netdev->min_mtu = ADI_AXIS_NET_MIN_MTU;
	netdev->max_mtu = ADI_AXIS_NET_MAX_MTU;
	netdev->mtu = ADI_AXIS_NET_DEFAULT_MTU;

	ret = platform_get_ethdev_address(dev, netdev);
	if (ret == -EPROBE_DEFER)
		goto err_free_rings;
	if (ret || !is_valid_ether_addr(netdev->dev_addr)) {
		eth_hw_addr_random(netdev);
		dev_info(dev, "using random MAC address %pM\n",
			 netdev->dev_addr);
	}

	/*
	 * There is no PHY and no way to sense the far side, so carrier tracks
	 * whether the DMA channels are armed. Register with it off: without
	 * this the link reports state UNKNOWN, and a network manager will
	 * happily install a default route over an interface that cannot carry
	 * a packet.
	 */
	netif_carrier_off(netdev);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register net device\n");
		goto err_free_rings;
	}

	return 0;

err_free_rings:
	adi_axis_net_free_ring(priv->tx_dma_dev, priv->tx, priv->buf_size);
	return ret;
}

static void adi_axis_net_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct adi_axis_net *priv = netdev_priv(netdev);

	/*
	 * unregister_netdev() closes the interface first, so ndo_stop() has
	 * already terminated both channels, waited out any in-flight callback
	 * and released the receive ring by the time the TX ring is freed.
	 */
	unregister_netdev(netdev);

	adi_axis_net_free_ring(priv->tx_dma_dev, priv->tx, priv->buf_size);
}

static const struct of_device_id adi_axis_net_of_match[] = {
	{ .compatible = "adi,axis-net" },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_axis_net_of_match);

static struct platform_driver adi_axis_net_driver = {
	.driver = {
		.name = "adi-axis-net",
		.of_match_table = adi_axis_net_of_match,
	},
	.probe = adi_axis_net_probe,
	.remove = adi_axis_net_remove,
};
module_platform_driver(adi_axis_net_driver);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("Point-to-point Ethernet over paired AXI-Stream DMA channels");
MODULE_LICENSE("GPL");
