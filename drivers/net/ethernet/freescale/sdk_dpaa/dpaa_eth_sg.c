/* Copyright 2012 Freescale Semiconductor Inc.
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
#include <linux/skbuff.h>
#include <linux/highmem.h>
#include <linux/fsl_bman.h>
#include <net/sock.h>

#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#ifdef CONFIG_FSL_DPAA_1588
#include "dpaa_1588.h"
#endif
#ifdef CONFIG_FSL_DPAA_CEETM
#include "dpaa_eth_ceetm.h"
#endif

/* DMA map and add a page frag back into the bpool.
 * @vaddr fragment must have been allocated with netdev_alloc_frag(),
 * specifically for fitting into @dpa_bp.
 */
static void dpa_bp_recycle_frag(struct dpa_bp *dpa_bp, unsigned long vaddr,
				int *count_ptr)
{
	struct bm_buffer bmb;
	dma_addr_t addr;

	bmb.opaque = 0;

	addr = dma_map_single(dpa_bp->dev, (void *)vaddr, dpa_bp->size,
			      DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		return;
	}

	bm_buffer_set64(&bmb, addr);

	while (bman_release(dpa_bp->pool, &bmb, 1, 0))
		cpu_relax();

	(*count_ptr)++;
}

static int _dpa_bp_add_8_bufs(const struct dpa_bp *dpa_bp)
{
	void *new_buf, *fman_buf;
	struct bm_buffer bmb[8];
	dma_addr_t addr;
	uint8_t i;
	struct device *dev = dpa_bp->dev;
	struct sk_buff *skb, **skbh;

	memset(bmb, 0, sizeof(struct bm_buffer) * 8);

	for (i = 0; i < 8; i++) {
		/* We'll prepend the skb back-pointer; can't use the DPA
		 * priv space, because FMan will overwrite it (from offset 0)
		 * if it ends up being the second, third, etc. fragment
		 * in a S/G frame.
		 *
		 * We only need enough space to store a pointer, but allocate
		 * an entire cacheline for performance reasons.
		 */
#ifdef FM_ERRATUM_A050385
		if (unlikely(fm_has_errata_a050385())) {
			struct page *new_page = alloc_page(GFP_ATOMIC);
			if (unlikely(!new_page))
				goto netdev_alloc_failed;
			new_buf = page_address(new_page);
		}
		else
#endif
		new_buf = netdev_alloc_frag(SMP_CACHE_BYTES + DPA_BP_RAW_SIZE);

		if (unlikely(!new_buf))
			goto netdev_alloc_failed;
		new_buf = PTR_ALIGN(new_buf, SMP_CACHE_BYTES);

		/* Apart from the buffer that will be used by the FMan, the
		 * skb also guarantees enough space to hold the backpointer
		 * in the headroom and the shared info at the end.
		 */
		skb = build_skb(new_buf,
				SMP_CACHE_BYTES + DPA_SKB_SIZE(dpa_bp->size) +
				SKB_DATA_ALIGN(sizeof(struct skb_shared_info)));
		if (unlikely(!skb)) {
			put_page(virt_to_head_page(new_buf));
			goto build_skb_failed;
		}

		/* Reserve SMP_CACHE_BYTES in the skb's headroom to store the
		 * backpointer. This area will not be synced to, or
		 * overwritten by, the FMan.
		 */
		skb_reserve(skb, SMP_CACHE_BYTES);

		/* We don't sync the first SMP_CACHE_BYTES of the buffer to
		 * the FMan. The skb backpointer is stored at the end of the
		 * reserved headroom. Otherwise it will be overwritten by the
		 * FMan.
		 * The buffer synced with the FMan starts right after the
		 * reserved headroom.
		 */
		fman_buf = new_buf + SMP_CACHE_BYTES;
		DPA_WRITE_SKB_PTR(skb, skbh, fman_buf, -1);

		addr = dma_map_single(dev, fman_buf,
				dpa_bp->size, DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dev, addr)))
			goto dma_map_failed;

		bm_buffer_set64(&bmb[i], addr);
	}

release_bufs:
	/* Release the buffers. In case bman is busy, keep trying
	 * until successful. bman_release() is guaranteed to succeed
	 * in a reasonable amount of time
	 */
	while (unlikely(bman_release(dpa_bp->pool, bmb, i, 0)))
		cpu_relax();
	return i;

dma_map_failed:
	kfree_skb(skb);

build_skb_failed:
netdev_alloc_failed:
	net_err_ratelimited("%s failed\n", __func__);
	WARN_ONCE(1, "Memory allocation failure on Rx\n");

	bm_buffer_set64(&bmb[i], 0);
	/* Avoid releasing a completely null buffer; bman_release() requires
	 * at least one buffer.
	 */
	if (likely(i))
		goto release_bufs;

	return 0;
}

/* Add buffers/(pages) for Rx processing whenever bpool count falls below
 * REFILL_THRESHOLD.
 */
int dpaa_eth_refill_bpools(struct dpa_bp *dpa_bp, int *countptr)
{
	int count = *countptr;
	int new_bufs;

	if (unlikely(count < CONFIG_FSL_DPAA_ETH_REFILL_THRESHOLD)) {
		do {
			new_bufs = _dpa_bp_add_8_bufs(dpa_bp);
			if (unlikely(!new_bufs)) {
				/* Avoid looping forever if we've temporarily
				 * run out of memory. We'll try again at the
				 * next NAPI cycle.
				 */
				break;
			}
			count += new_bufs;
		} while (count < CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT);

		*countptr = count;
		if (unlikely(count < CONFIG_FSL_DPAA_ETH_MAX_BUF_COUNT))
			return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(dpaa_eth_refill_bpools);

/* Cleanup function for outgoing frame descriptors that were built on Tx path,
 * either contiguous frames or scatter/gather ones.
 * Skb freeing is not handled here.
 *
 * This function may be called on error paths in the Tx function, so guard
 * against cases when not all fd relevant fields were filled in.
 *
 * Return the skb backpointer, since for S/G frames the buffer containing it
 * gets freed here.
 */
struct sk_buff *_dpa_cleanup_tx_fd(const struct dpa_priv_s *priv,
	const struct qm_fd *fd)
{
	const struct qm_sg_entry *sgt;
	int i;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	dma_addr_t addr = qm_fd_addr(fd);
	dma_addr_t sg_addr;
	struct sk_buff **skbh;
	struct sk_buff *skb = NULL;
	const enum dma_data_direction dma_dir = DMA_TO_DEVICE;
	int nr_frags;
	int sg_len;

	/* retrieve skb back pointer */
	DPA_READ_SKB_PTR(skb, skbh, phys_to_virt(addr), 0);

	if (unlikely(fd->format == qm_fd_sg)) {
		nr_frags = skb_shinfo(skb)->nr_frags;
		dma_unmap_single(dpa_bp->dev, addr,
				 dpa_fd_offset(fd) + DPA_SGT_SIZE,
				 dma_dir);

		/* The sgt buffer has been allocated with netdev_alloc_frag(),
		 * it's from lowmem.
		 */
		sgt = phys_to_virt(addr + dpa_fd_offset(fd));
#ifdef CONFIG_FSL_DPAA_1588
		if (priv->tsu && priv->tsu->valid &&
				priv->tsu->hwts_tx_en_ioctl)
			dpa_ptp_store_txstamp(priv, skb, (void *)skbh);
#endif
#ifdef CONFIG_FSL_DPAA_TS
		if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			struct skb_shared_hwtstamps shhwtstamps;

			dpa_get_ts(priv, TX, &shhwtstamps, (void *)skbh);
			skb_tstamp_tx(skb, &shhwtstamps);
		}
#endif /* CONFIG_FSL_DPAA_TS */

		/* sgt[0] is from lowmem, was dma_map_single()-ed */
		sg_addr = qm_sg_addr(&sgt[0]);
		sg_len = qm_sg_entry_get_len(&sgt[0]);
		dma_unmap_single(dpa_bp->dev, sg_addr, sg_len, dma_dir);

		/* remaining pages were mapped with dma_map_page() */
		for (i = 1; i <= nr_frags; i++) {
			DPA_BUG_ON(qm_sg_entry_get_ext(&sgt[i]));
			sg_addr = qm_sg_addr(&sgt[i]);
			sg_len = qm_sg_entry_get_len(&sgt[i]);
			dma_unmap_page(dpa_bp->dev, sg_addr, sg_len, dma_dir);
		}

		/* Free the page frag that we allocated on Tx */
		put_page(virt_to_head_page(sgt));
	} else {
		dma_unmap_single(dpa_bp->dev, addr,
				 skb_tail_pointer(skb) - (u8 *)skbh, dma_dir);
#ifdef CONFIG_FSL_DPAA_TS
		/* get the timestamp for non-SG frames */
#ifdef CONFIG_FSL_DPAA_1588
		if (priv->tsu && priv->tsu->valid &&
						priv->tsu->hwts_tx_en_ioctl)
			dpa_ptp_store_txstamp(priv, skb, (void *)skbh);
#endif
		if (unlikely(priv->ts_tx_en &&
				skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) {
			struct skb_shared_hwtstamps shhwtstamps;

			dpa_get_ts(priv, TX, &shhwtstamps, (void *)skbh);
			skb_tstamp_tx(skb, &shhwtstamps);
		}
#endif
	}

	return skb;
}
EXPORT_SYMBOL(_dpa_cleanup_tx_fd);

#ifndef CONFIG_FSL_DPAA_TS
bool dpa_skb_is_recyclable(struct sk_buff *skb)
{
	/* No recycling possible if skb buffer is kmalloc'ed  */
	if (skb->head_frag == 0)
		return false;

	/* or if it's an userspace buffer */
	if (skb_shinfo(skb)->flags & SKBFL_ZEROCOPY_ENABLE)
		return false;

	/* or if it's cloned or shared */
	if (skb_shared(skb) || skb_cloned(skb) ||
	    skb->fclone != SKB_FCLONE_UNAVAILABLE)
		return false;

	return true;
}
EXPORT_SYMBOL(dpa_skb_is_recyclable);

bool dpa_buf_is_recyclable(struct sk_buff *skb,
				  uint32_t min_size,
				  uint16_t min_offset,
				  unsigned char **new_buf_start)
{
	unsigned char *new;

	/* In order to recycle a buffer, the following conditions must be met:
	 * - buffer size no less than the buffer pool size
	 * - buffer size no higher than an upper limit (to avoid moving too much
	 *   system memory to the buffer pools)
	 * - buffer address aligned to cacheline bytes
	 * - offset of data from start of buffer no lower than a minimum value
	 * - offset of data from start of buffer no higher than a maximum value
	 * - the skb back-pointer is stored safely
	 */

	/* guarantee both the minimum size and the minimum data offset */
	new = min(skb_end_pointer(skb) - min_size, skb->data - min_offset);

	/* left align to the nearest cacheline */
	new = (unsigned char *)((unsigned long)new & ~(SMP_CACHE_BYTES - 1));

	/* Make sure there is enough space to store the skb back-pointer in
	 * the headroom, right before the start of the buffer.
	 *
	 * Guarantee that both maximum size and maximum data offsets aren't
	 * crossed.
	 */
	if (likely(new >= (skb->head + sizeof(void *)) &&
		   new >= (skb->data - DPA_MAX_FD_OFFSET) &&
		   skb_end_pointer(skb) - new <= DPA_RECYCLE_MAX_SIZE)) {
		*new_buf_start = new;
		return true;
	}

	return false;
}
EXPORT_SYMBOL(dpa_buf_is_recyclable);
#endif

/* Build a linear skb around the received buffer.
 * We are guaranteed there is enough room at the end of the data buffer to
 * accommodate the shared info area of the skb.
 */
static struct sk_buff *__hot contig_fd_to_skb(const struct dpa_priv_s *priv,
	const struct qm_fd *fd, int *use_gro)
{
	dma_addr_t addr = qm_fd_addr(fd);
	ssize_t fd_off = dpa_fd_offset(fd);
	void *vaddr;
	const fm_prs_result_t *parse_results;
	struct sk_buff *skb = NULL, **skbh;

	vaddr = phys_to_virt(addr);
	DPA_BUG_ON(!IS_ALIGNED((unsigned long)vaddr, SMP_CACHE_BYTES));

	/* Retrieve the skb and adjust data and tail pointers, to make sure
	 * forwarded skbs will have enough space on Tx if extra headers
	 * are added.
	 */
	DPA_READ_SKB_PTR(skb, skbh, vaddr, -1);

#ifdef CONFIG_FSL_DPAA_ETH_JUMBO_FRAME
	/* When using jumbo Rx buffers, we risk having frames dropped due to
	 * the socket backlog reaching its maximum allowed size.
	 * Use the frame length for the skb truesize instead of the buffer
	 * size, as this is the size of the data that actually gets copied to
	 * userspace.
	 * The stack may increase the payload. In this case, it will want to
	 * warn us that the frame length is larger than the truesize. We
	 * bypass the warning.
	 */
	skb->truesize = SKB_TRUESIZE(dpa_fd_length(fd));
#endif

	DPA_BUG_ON(fd_off != priv->rx_headroom);
	skb_reserve(skb, fd_off);
	skb_put(skb, dpa_fd_length(fd));

	/* Peek at the parse results for csum validation */
	parse_results = (const fm_prs_result_t *)(vaddr +
				DPA_RX_PRIV_DATA_SIZE);
	_dpa_process_parse_results(parse_results, fd, skb, use_gro);

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_rx_en_ioctl)
		dpa_ptp_store_rxstamp(priv, skb, vaddr);
#endif
#ifdef CONFIG_FSL_DPAA_TS
	if (priv->ts_rx_en)
		dpa_get_ts(priv, RX, skb_hwtstamps(skb), vaddr);
#endif /* CONFIG_FSL_DPAA_TS */

	return skb;
}


/* Build an skb with the data of the first S/G entry in the linear portion and
 * the rest of the frame as skb fragments.
 *
 * The page fragment holding the S/G Table is recycled here.
 */
static struct sk_buff *__hot sg_fd_to_skb(const struct dpa_priv_s *priv,
			       const struct qm_fd *fd, int *use_gro,
			       int *count_ptr)
{
	const struct qm_sg_entry *sgt;
	dma_addr_t addr = qm_fd_addr(fd);
	ssize_t fd_off = dpa_fd_offset(fd);
	dma_addr_t sg_addr;
	void *vaddr, *sg_vaddr;
	struct dpa_bp *dpa_bp;
	struct page *page, *head_page;
	int frag_offset, frag_len;
	int page_offset;
	int i;
	const fm_prs_result_t *parse_results;
	struct sk_buff *skb = NULL, *skb_tmp, **skbh;

	vaddr = phys_to_virt(addr);
	DPA_BUG_ON(!IS_ALIGNED((unsigned long)vaddr, SMP_CACHE_BYTES));

	dpa_bp = priv->dpa_bp;
	/* Iterate through the SGT entries and add data buffers to the skb */
	sgt = vaddr + fd_off;
	for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
		/* Extension bit is not supported */
		DPA_BUG_ON(qm_sg_entry_get_ext(&sgt[i]));

		/* We use a single global Rx pool */
		DPA_BUG_ON(dpa_bp !=
			   dpa_bpid2pool(qm_sg_entry_get_bpid(&sgt[i])));

		sg_addr = qm_sg_addr(&sgt[i]);
		sg_vaddr = phys_to_virt(sg_addr);
		DPA_BUG_ON(!IS_ALIGNED((unsigned long)sg_vaddr,
				SMP_CACHE_BYTES));

		dma_unmap_single(dpa_bp->dev, sg_addr, dpa_bp->size,
				 DMA_BIDIRECTIONAL);
		if (i == 0) {
			DPA_READ_SKB_PTR(skb, skbh, sg_vaddr, -1);
#ifdef CONFIG_FSL_DPAA_1588
			if (priv->tsu && priv->tsu->valid &&
			    priv->tsu->hwts_rx_en_ioctl)
				dpa_ptp_store_rxstamp(priv, skb, vaddr);
#endif
#ifdef CONFIG_FSL_DPAA_TS
			if (priv->ts_rx_en)
				dpa_get_ts(priv, RX, skb_hwtstamps(skb), vaddr);
#endif /* CONFIG_FSL_DPAA_TS */

			/* In the case of a SG frame, FMan stores the Internal
			 * Context in the buffer containing the sgt.
			 * Inspect the parse results before anything else.
			 */
			parse_results = (const fm_prs_result_t *)(vaddr +
						DPA_RX_PRIV_DATA_SIZE);
			_dpa_process_parse_results(parse_results, fd, skb,
						   use_gro);

			/* Make sure forwarded skbs will have enough space
			 * on Tx, if extra headers are added.
			 */
			DPA_BUG_ON(fd_off != priv->rx_headroom);
			skb_reserve(skb, fd_off);
			skb_put(skb, qm_sg_entry_get_len(&sgt[i]));
		} else {
			/* Not the first S/G entry; all data from buffer will
			 * be added in an skb fragment; fragment index is offset
			 * by one since first S/G entry was incorporated in the
			 * linear part of the skb.
			 *
			 * Caution: 'page' may be a tail page.
			 */
			DPA_READ_SKB_PTR(skb_tmp, skbh, sg_vaddr, -1);
			page = virt_to_page(sg_vaddr);
			head_page = virt_to_head_page(sg_vaddr);

			/* Free (only) the skbuff shell because its data buffer
			 * is already a frag in the main skb.
			 */
			get_page(head_page);
			dev_kfree_skb(skb_tmp);

			/* Compute offset in (possibly tail) page */
			page_offset = ((unsigned long)sg_vaddr &
					(PAGE_SIZE - 1)) +
				(page_address(page) - page_address(head_page));
			/* page_offset only refers to the beginning of sgt[i];
			 * but the buffer itself may have an internal offset.
			 */
			frag_offset = qm_sg_entry_get_offset(&sgt[i]) +
					page_offset;
			frag_len = qm_sg_entry_get_len(&sgt[i]);
			/* skb_add_rx_frag() does no checking on the page; if
			 * we pass it a tail page, we'll end up with
			 * bad page accounting and eventually with segafults.
			 */
			skb_add_rx_frag(skb, i - 1, head_page, frag_offset,
				frag_len, dpa_bp->size);
		}
		/* Update the pool count for the current {cpu x bpool} */
		(*count_ptr)--;

		if (qm_sg_entry_get_final(&sgt[i]))
			break;
	}
	WARN_ONCE(i == DPA_SGT_MAX_ENTRIES, "No final bit on SGT\n");

	/* recycle the SGT fragment */
	DPA_BUG_ON(dpa_bp != dpa_bpid2pool(fd->bpid));
	dpa_bp_recycle_frag(dpa_bp, (unsigned long)vaddr, count_ptr);
	return skb;
}

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
static inline int dpa_skb_loop(const struct dpa_priv_s *priv,
		struct sk_buff *skb)
{
	if (unlikely(priv->loop_to < 0))
		return 0; /* loop disabled by default */

	skb_push(skb, ETH_HLEN); /* compensate for eth_type_trans */
	/* Save the current CPU ID in order to maintain core affinity */
	skb_set_queue_mapping(skb, raw_smp_processor_id());
	dpa_tx(skb, dpa_loop_netdevs[priv->loop_to]);

	return 1; /* Frame Tx on the selected interface */
}
#endif

void __hot _dpa_rx(struct net_device *net_dev,
		struct qman_portal *portal,
		const struct dpa_priv_s *priv,
		struct dpa_percpu_priv_s *percpu_priv,
		const struct qm_fd *fd,
		u32 fqid,
		int *count_ptr)
{
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	dma_addr_t addr = qm_fd_addr(fd);
	u32 fd_status = fd->status;
	unsigned int skb_len;
	struct rtnl_link_stats64 *percpu_stats = &percpu_priv->stats;
	int use_gro = net_dev->features & NETIF_F_GRO;

	if (unlikely(fd_status & FM_FD_STAT_RX_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd_status & FM_FD_STAT_RX_ERRORS);

		percpu_stats->rx_errors++;
		goto _release_frame;
	}

	dpa_bp = priv->dpa_bp;
	DPA_BUG_ON(dpa_bp != dpa_bpid2pool(fd->bpid));

	/* prefetch the first 64 bytes of the frame or the SGT start */
	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);
	prefetch(phys_to_virt(addr) + dpa_fd_offset(fd));

	/* The only FD types that we may receive are contig and S/G */
	DPA_BUG_ON((fd->format != qm_fd_contig) && (fd->format != qm_fd_sg));

	if (likely(fd->format == qm_fd_contig)) {
#ifdef CONFIG_FSL_DPAA_HOOKS
		/* Execute the Rx processing hook, if it exists. */
		if (dpaa_eth_hooks.rx_default &&
			dpaa_eth_hooks.rx_default((void *)fd, net_dev,
					fqid) == DPAA_ETH_STOLEN) {
			/* won't count the rx bytes in */
			return;
		}
#endif
		skb = contig_fd_to_skb(priv, fd, &use_gro);
	} else {
		skb = sg_fd_to_skb(priv, fd, &use_gro, count_ptr);
		percpu_priv->rx_sg++;
	}

	/* Account for either the contig buffer or the SGT buffer (depending on
	 * which case we were in) having been removed from the pool.
	 */
	(*count_ptr)--;
	skb->protocol = eth_type_trans(skb, net_dev);

	skb_len = skb->len;

#ifdef CONFIG_FSL_DPAA_DBG_LOOP
	if (dpa_skb_loop(priv, skb)) {
		percpu_stats->rx_packets++;
		percpu_stats->rx_bytes += skb_len;
		return;
	}
#endif

	skb_record_rx_queue(skb, raw_smp_processor_id());

	if (use_gro) {
		const struct qman_portal_config *pc =
					qman_p_get_portal_config(portal);
		struct dpa_napi_portal *np = &percpu_priv->np[pc->index];

		np->p = portal;
		/* The stack doesn't report if the frame was dropped but it
		 * will increment rx_dropped automatically.
		 */
		napi_gro_receive(&np->napi, skb);
	} else if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
		return;

	percpu_stats->rx_packets++;
	percpu_stats->rx_bytes += skb_len;

	return;

_release_frame:
	dpa_fd_release(net_dev, fd);
}

int __hot skb_to_contig_fd(struct dpa_priv_s *priv,
			   struct sk_buff *skb, struct qm_fd *fd,
			   int *count_ptr, int *offset)
{
	struct sk_buff **skbh;
	dma_addr_t addr;
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	struct net_device *net_dev = priv->net_dev;
	int err;
	enum dma_data_direction dma_dir;
	unsigned char *buffer_start;
	int dma_map_size;

#ifndef CONFIG_FSL_DPAA_TS
	/* Check recycling conditions; only if timestamp support is not
	 * enabled, otherwise we need the fd back on tx confirmation
	 */

	/* We can recycle the buffer if:
	 * - the pool is not full
	 * - the buffer meets the skb recycling conditions
	 * - the buffer meets our own (size, offset, align) conditions
	 */
	if (likely((*count_ptr < dpa_bp->target_count) &&
		   dpa_skb_is_recyclable(skb) &&
		   dpa_buf_is_recyclable(skb, dpa_bp->size,
					 priv->tx_headroom, &buffer_start))) {
		/* Buffer is recyclable; use the new start address
		 * and set fd parameters and DMA mapping direction
		 */
		fd->bpid = dpa_bp->bpid;
		DPA_BUG_ON(skb->data - buffer_start > DPA_MAX_FD_OFFSET);
		fd->offset = (uint16_t)(skb->data - buffer_start);
		dma_dir = DMA_BIDIRECTIONAL;
		dma_map_size = dpa_bp->size;

		/* Store the skb back-pointer before the start of the buffer.
		 * Otherwise it will be overwritten by the FMan.
		 */
		DPA_WRITE_SKB_PTR(skb, skbh, buffer_start, -1);
		*offset = skb_headroom(skb) - fd->offset;
	} else
#endif
	{
		/* Not recyclable.
		 * We are guaranteed to have at least tx_headroom bytes
		 * available, so just use that for offset.
		 */
		fd->bpid = 0xff;
		buffer_start = skb->data - priv->tx_headroom;
		fd->offset = priv->tx_headroom;
		dma_dir = DMA_TO_DEVICE;
		dma_map_size = skb_tail_pointer(skb) - buffer_start;

		/* The buffer will be Tx-confirmed, but the TxConf cb must
		 * necessarily look at our Tx private data to retrieve the
		 * skbuff. Store the back-pointer inside the buffer.
		 */
		DPA_WRITE_SKB_PTR(skb, skbh, buffer_start, 0);
	}

	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb.
	 */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 ((char *)skbh) + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		return err;
	}

	/* Fill in the rest of the FD fields */
	fd->format = qm_fd_contig;
	fd->length20 = skb->len;
	fd->cmd |= FM_FD_CMD_FCO;

	/* Map the entire buffer size that may be seen by FMan, but no more */
	addr = dma_map_single(dpa_bp->dev, skbh, dma_map_size, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "dma_map_single() failed\n");
		return -EINVAL;
	}
	qm_fd_addr_set64(fd, addr);

	return 0;
}
EXPORT_SYMBOL(skb_to_contig_fd);

#ifdef FM_ERRATUM_A050385
/* Verify the conditions that trigger the A050385 errata:
 * - 4K memory address boundary crossings when the data/SG fragments aren't
 *   aligned to 256 bytes
 * - data and SG fragments that aren't aligned to 16 bytes
 * - SG fragments that aren't mod 16 bytes in size (except for the last
 *   fragment)
 */
static bool a050385_check_skb(struct sk_buff *skb, struct dpa_priv_s *priv)
{
	skb_frag_t *frag;
	int i, nr_frags;

	nr_frags = skb_shinfo(skb)->nr_frags;

	/* Check if the linear data is 16 byte aligned */
	if ((uintptr_t)skb->data % 16)
		return true;

	/* Check if the needed headroom crosses a 4K address boundary without
	 * being 256 byte aligned
	 */
	if (CROSS_4K(skb->data - priv->tx_headroom, priv->tx_headroom) &&
	    (((uintptr_t)skb->data - priv->tx_headroom) % 256))
		return true;

	/* Check if the linear data crosses a 4K address boundary without
	 * being 256 byte aligned
	 */
	if (CROSS_4K(skb->data, skb_headlen(skb)) &&
	    ((uintptr_t)skb->data % 256))
		return true;

	/* When using Scatter/Gather, the linear data becomes the first
	 * fragment in the list and must follow the same restrictions as the
	 * other fragments.
	 *
	 * Check if the linear data is mod 16 bytes in size.
	 */
	if (nr_frags && (skb_headlen(skb) % 16))
		return true;

	/* Check the SG fragments. They must follow the same rules as the
	 * linear data with and additional restriction: they must be multiple
	 * of 16 bytes in size to account for the hardware carryover effect.
	 */
	for (i = 0; i < nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i];

		/* Check if the fragment is a multiple of 16 bytes in size.
		 * The last fragment is exempt from this restriction.
		 */
		if ((i != (nr_frags - 1)) && (skb_frag_size(frag) % 16))
			return true;

		/* Check if the fragment is 16 byte aligned */
		if (skb_frag_off(frag) % 16)
			return true;

		/* Check if the fragment crosses a 4K address boundary. Since
		 * the alignment of previous fragments can influence the
		 * current fragment, checking for the 256 byte alignment
		 * isn't relevant.
		 */
		if (CROSS_4K(skb_frag_off(frag), skb_frag_size(frag)))
			return true;
	}

	return false;
}

/* Realign the skb by copying its contents at the start of a newly allocated
 * page. Build a new skb around the new buffer and release the old one.
 * A performance drop should be expected.
 */
static struct sk_buff *a050385_realign_skb(struct sk_buff *skb,
					   struct dpa_priv_s *priv)
{
	int trans_offset = skb_transport_offset(skb);
	int net_offset = skb_network_offset(skb);
	struct sk_buff *nskb = NULL;
	int nsize, headroom;
	struct page *npage;
	void *npage_addr;

	headroom = DPAA_A050385_HEADROOM;

	/* For the new skb we only need the old one's data (both non-paged and
	 * paged). We can skip the old tailroom.
	 *
	 * Make sure the skb_shinfo is cache-line aligned.
	 */
	nsize = SMP_CACHE_BYTES + DPA_SKB_SIZE(headroom + skb->len) +
		SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	/* Reserve enough memory to accommodate Jumbo frames */
	npage = alloc_pages(GFP_ATOMIC | __GFP_COMP, get_order(nsize));
	if (unlikely(!npage)) {
		WARN_ONCE(1, "Memory allocation failure\n");
		return NULL;
	}
	npage_addr = page_address(npage);

	nskb = build_skb(npage_addr, nsize);
	if (unlikely(!nskb))
		goto err;

	/* Reserve only the needed headroom in order to guarantee the data's
	 * alignment.
	 * Code borrowed and adapted from skb_copy().
	 */
	skb_reserve(nskb, headroom);
	skb_put(nskb, skb->len);
	if (skb_copy_bits(skb, 0, nskb->data, skb->len)) {
		WARN_ONCE(1, "skb parsing failure\n");
		goto err;
	}
	skb_copy_header(nskb, skb);

#ifdef CONFIG_FSL_DPAA_TS
	/* Copy relevant timestamp info from the old skb to the new */
	if (priv->ts_tx_en) {
		skb_shinfo(nskb)->tx_flags = skb_shinfo(skb)->tx_flags;
		skb_shinfo(nskb)->hwtstamps = skb_shinfo(skb)->hwtstamps;
		skb_shinfo(nskb)->tskey = skb_shinfo(skb)->tskey;
		if (skb->sk)
			skb_set_owner_w(nskb, skb->sk);
	}
#endif
	/* We move the headroom when we align it so we have to reset the
	 * network and transport header offsets relative to the new data
	 * pointer. The checksum offload relies on these offsets.
	 */
	skb_set_network_header(nskb, net_offset);
	skb_set_transport_header(nskb, trans_offset);

	return nskb;

err:
	if (nskb)
		dev_kfree_skb(nskb);
	put_page(npage);
	return NULL;
}
#endif

int __hot skb_to_sg_fd(struct dpa_priv_s *priv,
		       struct sk_buff *skb, struct qm_fd *fd)
{
	struct dpa_bp *dpa_bp = priv->dpa_bp;
	dma_addr_t addr;
	dma_addr_t sg_addr;
	struct sk_buff **skbh;
	struct net_device *net_dev = priv->net_dev;
	int sg_len, sgt_size;
	int err;

	struct qm_sg_entry *sgt;
	void *sgt_buf;
	skb_frag_t *frag;
	int i = 0, j = 0;
	int nr_frags;
	const enum dma_data_direction dma_dir = DMA_TO_DEVICE;

	nr_frags = skb_shinfo(skb)->nr_frags;
	fd->format = qm_fd_sg;

	/* The FMan reads 256 bytes from the start of the SGT regardless of
	 * its size. In accordance, we reserve the same amount of memory as
	 * well.
	 */
	sgt_size = DPA_SGT_SIZE;

	/* Get a page frag to store the SGTable, or a full page if the errata
	 * is in place and we need to avoid crossing a 4k boundary.
	 */
#ifdef FM_ERRATUM_A050385
	if (unlikely(fm_has_errata_a050385())) {
		struct page *new_page = alloc_page(GFP_ATOMIC);

		if (unlikely(!new_page))
			return -ENOMEM;
		sgt_buf = page_address(new_page);
	}
	else
#endif
		sgt_buf = netdev_alloc_frag(priv->tx_headroom + sgt_size);
	if (unlikely(!sgt_buf)) {
		dev_err(dpa_bp->dev, "netdev_alloc_frag() failed\n");
		return -ENOMEM;
	}

	/* it seems that the memory allocator does not zero the allocated mem */
	memset(sgt_buf, 0, priv->tx_headroom + sgt_size);

	/* Enable L3/L4 hardware checksum computation.
	 *
	 * We must do this before dma_map_single(DMA_TO_DEVICE), because we may
	 * need to write into the skb.
	 */
	err = dpa_enable_tx_csum(priv, skb, fd,
				 sgt_buf + DPA_TX_PRIV_DATA_SIZE);
	if (unlikely(err < 0)) {
		if (netif_msg_tx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "HW csum error: %d\n", err);
		goto csum_failed;
	}

	/* Assign the data from skb->data to the first SG list entry */
	sgt = (struct qm_sg_entry *)(sgt_buf + priv->tx_headroom);
	sg_len = skb_headlen(skb);
	qm_sg_entry_set_bpid(&sgt[0], 0xff);
	qm_sg_entry_set_offset(&sgt[0], 0);
	qm_sg_entry_set_len(&sgt[0], sg_len);
	qm_sg_entry_set_ext(&sgt[0], 0);
	qm_sg_entry_set_final(&sgt[0], 0);

	addr = dma_map_single(dpa_bp->dev, skb->data, sg_len, dma_dir);
	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		err = -EINVAL;
		goto sg0_map_failed;
	}

	qm_sg_entry_set64(&sgt[0], addr);

	/* populate the rest of SGT entries */
	for (i = 1; i <= nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i - 1];
		qm_sg_entry_set_bpid(&sgt[i], 0xff);
		qm_sg_entry_set_offset(&sgt[i], 0);
		qm_sg_entry_set_len(&sgt[i], frag->bv_len);
		qm_sg_entry_set_ext(&sgt[i], 0);

		if (i == nr_frags)
			qm_sg_entry_set_final(&sgt[i], 1);
		else
			qm_sg_entry_set_final(&sgt[i], 0);

		DPA_BUG_ON(!skb_frag_page(frag));
		addr = skb_frag_dma_map(dpa_bp->dev, frag, 0, frag->bv_len,
					dma_dir);
		if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
			dev_err(dpa_bp->dev, "DMA mapping failed");
			err = -EINVAL;
			goto sg_map_failed;
		}

		/* keep the offset in the address */
		qm_sg_entry_set64(&sgt[i], addr);
	}

	fd->length20 = skb->len;
	fd->offset = priv->tx_headroom;

	/* DMA map the SGT page
	 *
	 * It's safe to store the skb back-pointer inside the buffer since
	 * S/G frames are non-recyclable.
	 */
	DPA_WRITE_SKB_PTR(skb, skbh, sgt_buf, 0);
	addr = dma_map_single(dpa_bp->dev, sgt_buf,
			      priv->tx_headroom + sgt_size,
			      dma_dir);

	if (unlikely(dma_mapping_error(dpa_bp->dev, addr))) {
		dev_err(dpa_bp->dev, "DMA mapping failed");
		err = -EINVAL;
		goto sgt_map_failed;
	}

	qm_fd_addr_set64(fd, addr);
	fd->bpid = 0xff;
	fd->cmd |= FM_FD_CMD_FCO;

	return 0;

sgt_map_failed:
sg_map_failed:
	for (j = 0; j < i; j++) {
		sg_addr = qm_sg_addr(&sgt[j]);
		dma_unmap_page(dpa_bp->dev, sg_addr,
			       qm_sg_entry_get_len(&sgt[j]), dma_dir);
	}
sg0_map_failed:
csum_failed:
	put_page(virt_to_head_page(sgt_buf));

	return err;
}
EXPORT_SYMBOL(skb_to_sg_fd);

int __hot dpa_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	int queue_mapping = dpa_get_queue_mapping(skb);
	struct qman_fq *egress_fq, *conf_fq;

#ifdef CONFIG_FSL_DPAA_HOOKS
	/* If there is a Tx hook, run it. */
	if (dpaa_eth_hooks.tx &&
		dpaa_eth_hooks.tx(skb, net_dev) == DPAA_ETH_STOLEN)
		/* won't update any Tx stats */
		return NETDEV_TX_OK;
#endif

	priv = netdev_priv(net_dev);

#ifdef CONFIG_FSL_DPAA_CEETM
	if (priv->ceetm_en)
		return ceetm_tx(skb, net_dev);
#endif

	if (unlikely(queue_mapping >= DPAA_ETH_TX_QUEUES))
		queue_mapping = queue_mapping % DPAA_ETH_TX_QUEUES;

	egress_fq = priv->egress_fqs[queue_mapping];
	conf_fq = priv->conf_fqs[queue_mapping];

	return dpa_tx_extended(skb, net_dev, egress_fq, conf_fq);
}

int __hot dpa_tx_extended(struct sk_buff *skb, struct net_device *net_dev,
		struct qman_fq *egress_fq, struct qman_fq *conf_fq)
{
	struct dpa_priv_s	*priv;
	struct qm_fd		 fd;
	struct dpa_percpu_priv_s *percpu_priv;
	struct rtnl_link_stats64 *percpu_stats;
	int err = 0;
	bool nonlinear, skb_changed, skb_need_wa;
	int *countptr, offset = 0;
	struct sk_buff *nskb;
	struct netdev_queue *txq;
	int txq_id = skb_get_queue_mapping(skb);

	/* Flags to help optimize the A050385 errata restriction checks.
	 *
	 * First flag marks if the skb changed between the first A050385 check
	 * and the moment it's converted to an FD.
	 *
	 * The second flag marks if the skb needs to be realigned in order to
	 * avoid the errata.
	 *
	 * The flags should have minimal impact on platforms not impacted by
	 * the errata.
	 */
	skb_changed = false;
	skb_need_wa = false;

	priv = netdev_priv(net_dev);
	/* Non-migratable context, safe to use raw_cpu_ptr */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	percpu_stats = &percpu_priv->stats;
	countptr = raw_cpu_ptr(priv->percpu_count);

	clear_fd(&fd);

#ifdef FM_ERRATUM_A050385
	if (unlikely(fm_has_errata_a050385()) && a050385_check_skb(skb, priv))
		skb_need_wa = true;
#endif

	nonlinear = skb_is_nonlinear(skb);

#ifdef CONFIG_FSL_DPAA_1588
	if (priv->tsu && priv->tsu->valid && priv->tsu->hwts_tx_en_ioctl)
		fd.cmd |= FM_FD_CMD_UPD;
#endif
#ifdef CONFIG_FSL_DPAA_TS
	if (unlikely(priv->ts_tx_en &&
			skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP))
		fd.cmd |= FM_FD_CMD_UPD;
	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
#endif /* CONFIG_FSL_DPAA_TS */

	/* MAX_SKB_FRAGS is larger than our DPA_SGT_MAX_ENTRIES; make sure
	 * we don't feed FMan with more fragments than it supports.
	 * Btw, we're using the first sgt entry to store the linear part of
	 * the skb, so we're one extra frag short.
	 */
	if (nonlinear && !skb_need_wa &&
	    likely(skb_shinfo(skb)->nr_frags < DPA_SGT_MAX_ENTRIES)) {
		/* Just create a S/G fd based on the skb */
		err = skb_to_sg_fd(priv, skb, &fd);
		percpu_priv->tx_frag_skbuffs++;
	} else {
		/* Make sure we have enough headroom to accommodate private
		 * data, parse results, etc. Normally this shouldn't happen if
		 * we're here via the standard kernel stack.
		 */
		if (unlikely(skb_headroom(skb) < priv->tx_headroom)) {
			struct sk_buff *skb_new;

			skb_new = skb_realloc_headroom(skb, priv->tx_headroom);
			if (unlikely(!skb_new)) {
				dev_kfree_skb(skb);
				percpu_stats->tx_errors++;
				return NETDEV_TX_OK;
			}

			/* propagate the skb ownership information */
			if (skb->sk)
				skb_set_owner_w(skb_new, skb->sk);

			dev_kfree_skb(skb);
			skb = skb_new;
			skb_changed = true;
		}

		/* We're going to store the skb backpointer at the beginning
		 * of the data buffer, so we need a privately owned skb
		 *
		 * Under the A050385 errata, we are going to have a privately
		 * owned skb after realigning the current one, so no point in
		 * copying it here in that case.
		 */

		/* Code borrowed from skb_unshare(). */
		if (skb_cloned(skb) && !skb_need_wa) {
			nskb = skb_copy(skb, GFP_ATOMIC);
			kfree_skb(skb);
			skb = nskb;
			skb_changed = true;

			/* skb_copy() has now linearized the skbuff. */
		} else if (unlikely(nonlinear) && !skb_need_wa) {
			/* We are here because the egress skb contains
			 * more fragments than we support. In this case,
			 * we have no choice but to linearize it ourselves.
			 */
#ifdef FM_ERRATUM_A050385
			/* No point in linearizing the skb now if we are going
			 * to realign and linearize it again further down due
			 * to the A050385 errata
			 */
			if (unlikely(fm_has_errata_a050385()))
				skb_need_wa = true;
			else
#endif
				err = __skb_linearize(skb);
		}
		if (unlikely(!skb || err < 0))
			/* Common out-of-memory error path */
			goto enomem;

#ifdef FM_ERRATUM_A050385
		/* Verify the skb a second time if it has been updated since
		 * the previous check
		 */
		if (unlikely(fm_has_errata_a050385()) && skb_changed &&
		    a050385_check_skb(skb, priv))
			skb_need_wa = true;

		if (unlikely(fm_has_errata_a050385()) && skb_need_wa) {
			nskb = a050385_realign_skb(skb, priv);
			if (!nskb)
				goto skb_to_fd_failed;
			dev_kfree_skb(skb);
			skb = nskb;
		}
#endif

		err = skb_to_contig_fd(priv, skb, &fd, countptr, &offset);
	}
	if (unlikely(err < 0))
		goto skb_to_fd_failed;

	if (fd.bpid != 0xff) {
		skb_recycle(skb);
		/* skb_recycle() reserves NET_SKB_PAD as skb headroom,
		 * but we need the skb to look as if returned by build_skb().
		 * We need to manually adjust the tailptr as well.
		 */
		skb->data = skb->head + offset;
		skb_reset_tail_pointer(skb);

		(*countptr)++;
		percpu_priv->tx_returned++;
	}

	if (unlikely(dpa_xmit(priv, percpu_stats, &fd, egress_fq, conf_fq) < 0))
		goto xmit_failed;

	/* LLTX forces us to update our own jiffies for each netdev queue.
	 * Use the queue mapping registered in the skb.
	 */
	txq = netdev_get_tx_queue(net_dev, txq_id);
	txq->trans_start = jiffies;
	return NETDEV_TX_OK;

xmit_failed:
	if (fd.bpid != 0xff) {
		(*countptr)--;
		percpu_priv->tx_returned--;
		dpa_fd_release(net_dev, &fd);
		percpu_stats->tx_errors++;
		return NETDEV_TX_OK;
	}
	_dpa_cleanup_tx_fd(priv, &fd);
skb_to_fd_failed:
enomem:
	percpu_stats->tx_errors++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}
EXPORT_SYMBOL(dpa_tx_extended);
