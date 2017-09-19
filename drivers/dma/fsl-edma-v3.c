/*
 * drivers/dma/fsl-edma3-v3.c
 *
 * Copyright 2017 NXP .
 *
 * Driver for the Freescale eDMA engine v3. This driver based on fsl-edma3.c
 * but changed to meet the IP change on i.MX8QM: every dma channel is specific
 * to hardware. For example, channel 14 for LPUART1 receive request and channel
 * 13 for transmit requesst. The eDMA block can be found on i.MX8QM
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>

#include "virt-dma.h"

#define EDMA_CH_CSR			0x00
#define EDMA_CH_ES			0x04
#define EDMA_CH_INT			0x08
#define EDMA_CH_SBR			0x0C
#define EDMA_CH_PRI			0x10
#define EDMA_TCD_SADDR			0x20
#define EDMA_TCD_SOFF			0x24
#define EDMA_TCD_ATTR			0x26
#define EDMA_TCD_NBYTES			0x28
#define EDMA_TCD_SLAST			0x2C
#define EDMA_TCD_DADDR			0x30
#define EDMA_TCD_DOFF			0x34
#define EDMA_TCD_CITER_ELINK		0x36
#define EDMA_TCD_CITER			0x36
#define EDMA_TCD_DLAST_SGA		0x38
#define EDMA_TCD_CSR			0x3C
#define EDMA_TCD_BITER_ELINK		0x3E
#define EDMA_TCD_BITER			0x3E

#define EDMA_CH_SBR_RD		BIT(22)
#define EDMA_CH_SBR_WR		BIT(21)
#define EDMA_CH_CSR_ERQ		BIT(0)
#define EDMA_CH_CSR_EARQ	BIT(1)
#define EDMA_CH_CSR_EEI		BIT(2)
#define EDMA_CH_CSR_DONE	BIT(30)
#define EDMA_CH_CSR_ACTIVE	BIT(31)

#define EDMA_TCD_ATTR_DSIZE(x)		(((x) & 0x0007))
#define EDMA_TCD_ATTR_DMOD(x)		(((x) & 0x001F) << 3)
#define EDMA_TCD_ATTR_SSIZE(x)		(((x) & 0x0007) << 8)
#define EDMA_TCD_ATTR_SMOD(x)		(((x) & 0x001F) << 11)
#define EDMA_TCD_ATTR_SSIZE_8BIT	(0x0000)
#define EDMA_TCD_ATTR_SSIZE_16BIT	(0x0100)
#define EDMA_TCD_ATTR_SSIZE_32BIT	(0x0200)
#define EDMA_TCD_ATTR_SSIZE_64BIT	(0x0300)
#define EDMA_TCD_ATTR_SSIZE_16BYTE	(0x0400)
#define EDMA_TCD_ATTR_SSIZE_32BYTE	(0x0500)
#define EDMA_TCD_ATTR_SSIZE_64BYTE	(0x0600)
#define EDMA_TCD_ATTR_DSIZE_8BIT	(0x0000)
#define EDMA_TCD_ATTR_DSIZE_16BIT	(0x0001)
#define EDMA_TCD_ATTR_DSIZE_32BIT	(0x0002)
#define EDMA_TCD_ATTR_DSIZE_64BIT	(0x0003)
#define EDMA_TCD_ATTR_DSIZE_16BYTE	(0x0004)
#define EDMA_TCD_ATTR_DSIZE_32BYTE	(0x0005)
#define EDMA_TCD_ATTR_DSIZE_64BYTE	(0x0006)

#define EDMA_TCD_SOFF_SOFF(x)		(x)
#define EDMA_TCD_NBYTES_NBYTES(x)	(x)
#define EDMA_TCD_NBYTES_MLOFF(x)	(x << 10)
#define EDMA_TCD_NBYTES_DMLOE		(1 << 30)
#define EDMA_TCD_NBYTES_SMLOE		(1 << 31)
#define EDMA_TCD_SLAST_SLAST(x)		(x)
#define EDMA_TCD_DADDR_DADDR(x)		(x)
#define EDMA_TCD_CITER_CITER(x)		((x) & 0x7FFF)
#define EDMA_TCD_DOFF_DOFF(x)		(x)
#define EDMA_TCD_DLAST_SGA_DLAST_SGA(x)	(x)
#define EDMA_TCD_BITER_BITER(x)		((x) & 0x7FFF)

#define EDMA_TCD_CSR_START		BIT(0)
#define EDMA_TCD_CSR_INT_MAJOR		BIT(1)
#define EDMA_TCD_CSR_INT_HALF		BIT(2)
#define EDMA_TCD_CSR_D_REQ		BIT(3)
#define EDMA_TCD_CSR_E_SG		BIT(4)
#define EDMA_TCD_CSR_E_LINK		BIT(5)
#define EDMA_TCD_CSR_ACTIVE		BIT(6)
#define EDMA_TCD_CSR_DONE		BIT(7)

#define FSL_EDMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_8_BYTES) | \
				BIT(DMA_SLAVE_BUSWIDTH_16_BYTES))

#define ARGS_RX				BIT(0)
#define ARGS_REMOTE			BIT(1)
#define ARGS_DFIFO			BIT(2)

/* channel name template define in dts */
#define CHAN_PREFIX			"edma0-chan"
#define CHAN_POSFIX			"-tx"

struct fsl_edma3_hw_tcd {
	__le32	saddr;
	__le16	soff;
	__le16	attr;
	__le32	nbytes;
	__le32	slast;
	__le32	daddr;
	__le16	doff;
	__le16	citer;
	__le32	dlast_sga;
	__le16	csr;
	__le16	biter;
};

struct fsl_edma3_sw_tcd {
	dma_addr_t			ptcd;
	struct fsl_edma3_hw_tcd		*vtcd;
};

struct fsl_edma3_slave_config {
	enum dma_transfer_direction	dir;
	enum dma_slave_buswidth		addr_width;
	u32				dev_addr;
	u32				dev2_addr; /* source addr for dev2dev */
	u32				burst;
	u32				attr;
};

struct fsl_edma3_chan {
	struct virt_dma_chan		vchan;
	enum dma_status			status;
	struct fsl_edma3_engine		*edma3;
	struct fsl_edma3_desc		*edesc;
	struct fsl_edma3_slave_config	fsc;
	void __iomem			*membase;
	int				txirq;
	int				hw_chanid;
	int				priority;
	int				is_rxchan;
	int				is_remote;
	int				is_dfifo;
	struct dma_pool			*tcd_pool;
	u32				chn_real_count;
	char				txirq_name[32];
};

struct fsl_edma3_desc {
	struct virt_dma_desc		vdesc;
	struct fsl_edma3_chan		*echan;
	bool				iscyclic;
	unsigned int			n_tcds;
	struct fsl_edma3_sw_tcd		tcd[];
};

struct fsl_edma3_engine {
	struct dma_device	dma_dev;
	struct mutex		fsl_edma3_mutex;
	u32			n_chans;
	int			errirq;
	bool			swap;	/* remote/local swapped on Audio edma */
	struct fsl_edma3_chan	chans[];
};

static struct fsl_edma3_chan *to_fsl_edma3_chan(struct dma_chan *chan)
{
	return container_of(chan, struct fsl_edma3_chan, vchan.chan);
}

static struct fsl_edma3_desc *to_fsl_edma3_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct fsl_edma3_desc, vdesc);
}

static void fsl_edma3_enable_request(struct fsl_edma3_chan *fsl_chan)
{
	void __iomem *addr = fsl_chan->membase;
	u32 val;

	val = readl(addr + EDMA_CH_SBR);
	/* Remote/local swapped wrongly on iMX8 QM Audio edma */
	if (fsl_chan->edma3->swap) {
		if (!fsl_chan->is_rxchan)
			val |= EDMA_CH_SBR_RD;
		else
			val |= EDMA_CH_SBR_WR;
	} else {
		if (fsl_chan->is_rxchan)
			val |= EDMA_CH_SBR_RD;
		else
			val |= EDMA_CH_SBR_WR;
	}

	if (fsl_chan->is_remote)
		val &= ~(EDMA_CH_SBR_RD | EDMA_CH_SBR_WR);

	writel(val, addr + EDMA_CH_SBR);

	val = readl(addr + EDMA_CH_CSR);

	val |= EDMA_CH_CSR_ERQ;
	writel(val, addr + EDMA_CH_CSR);
}

static void fsl_edma3_disable_request(struct fsl_edma3_chan *fsl_chan)
{
	void __iomem *addr = fsl_chan->membase;
	u32 val = readl(addr + EDMA_CH_CSR);

	val &= ~EDMA_CH_CSR_ERQ;
	writel(val, addr + EDMA_CH_CSR);
}

static unsigned int fsl_edma3_get_tcd_attr(enum dma_slave_buswidth addr_width)
{
	switch (addr_width) {
	case 1:
		return EDMA_TCD_ATTR_SSIZE_8BIT | EDMA_TCD_ATTR_DSIZE_8BIT;
	case 2:
		return EDMA_TCD_ATTR_SSIZE_16BIT | EDMA_TCD_ATTR_DSIZE_16BIT;
	case 4:
		return EDMA_TCD_ATTR_SSIZE_32BIT | EDMA_TCD_ATTR_DSIZE_32BIT;
	case 8:
		return EDMA_TCD_ATTR_SSIZE_64BIT | EDMA_TCD_ATTR_DSIZE_64BIT;
	case 16:
		return EDMA_TCD_ATTR_SSIZE_16BYTE | EDMA_TCD_ATTR_DSIZE_16BYTE;
	case 32:
		return EDMA_TCD_ATTR_SSIZE_32BYTE | EDMA_TCD_ATTR_DSIZE_32BYTE;
	case 64:
		return EDMA_TCD_ATTR_SSIZE_64BYTE | EDMA_TCD_ATTR_DSIZE_64BYTE;
	default:
		return EDMA_TCD_ATTR_SSIZE_32BIT | EDMA_TCD_ATTR_DSIZE_32BIT;
	}
}

static void fsl_edma3_free_desc(struct virt_dma_desc *vdesc)
{
	struct fsl_edma3_desc *fsl_desc;
	int i;

	fsl_desc = to_fsl_edma3_desc(vdesc);
	for (i = 0; i < fsl_desc->n_tcds; i++)
		dma_pool_free(fsl_desc->echan->tcd_pool, fsl_desc->tcd[i].vtcd,
			      fsl_desc->tcd[i].ptcd);
	kfree(fsl_desc);
}

static int fsl_edma3_terminate_all(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	fsl_edma3_disable_request(fsl_chan);
	fsl_chan->edesc = NULL;
	vchan_get_all_descriptors(&fsl_chan->vchan, &head);
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&fsl_chan->vchan, &head);
	return 0;
}

static int fsl_edma3_pause(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	if (fsl_chan->edesc) {
		fsl_edma3_disable_request(fsl_chan);
		fsl_chan->status = DMA_PAUSED;
	}
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
	return 0;
}

static int fsl_edma3_resume(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	if (fsl_chan->edesc) {
		fsl_edma3_enable_request(fsl_chan);
		fsl_chan->status = DMA_IN_PROGRESS;
	}
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
	return 0;
}

static int fsl_edma3_slave_config(struct dma_chan *chan,
				 struct dma_slave_config *cfg)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);

	fsl_chan->fsc.dir = cfg->direction;
	if (cfg->direction == DMA_DEV_TO_MEM) {
		fsl_chan->fsc.dev_addr = cfg->src_addr;
		fsl_chan->fsc.addr_width = cfg->src_addr_width;
		fsl_chan->fsc.burst = cfg->src_maxburst;
		fsl_chan->fsc.attr = fsl_edma3_get_tcd_attr
					(cfg->src_addr_width);
	} else if (cfg->direction == DMA_MEM_TO_DEV) {
		fsl_chan->fsc.dev_addr = cfg->dst_addr;
		fsl_chan->fsc.addr_width = cfg->dst_addr_width;
		fsl_chan->fsc.burst = cfg->dst_maxburst;
		fsl_chan->fsc.attr = fsl_edma3_get_tcd_attr
					(cfg->dst_addr_width);
	} else if (cfg->direction == DMA_DEV_TO_DEV) {
		fsl_chan->fsc.dev2_addr = cfg->src_addr;
		fsl_chan->fsc.dev_addr = cfg->dst_addr;
		fsl_chan->fsc.addr_width = cfg->dst_addr_width;
		fsl_chan->fsc.burst = cfg->dst_maxburst;
		fsl_chan->fsc.attr = fsl_edma3_get_tcd_attr
					(cfg->dst_addr_width);
	} else {
			return -EINVAL;
	}
	return 0;
}

static size_t fsl_edma3_desc_residue(struct fsl_edma3_chan *fsl_chan,
		struct virt_dma_desc *vdesc, bool in_progress)
{
	struct fsl_edma3_desc *edesc = fsl_chan->edesc;
	void __iomem *addr = fsl_chan->membase;
	enum dma_transfer_direction dir = fsl_chan->fsc.dir;
	dma_addr_t cur_addr, dma_addr;
	size_t len, size;
	int i;

	/* calculate the total size in this desc */
	for (len = i = 0; i < fsl_chan->edesc->n_tcds; i++)
		len += le32_to_cpu(edesc->tcd[i].vtcd->nbytes)
			* le16_to_cpu(edesc->tcd[i].vtcd->biter);

	if (!in_progress)
		return len;

	if (dir == DMA_MEM_TO_DEV)
		cur_addr = readl(addr + EDMA_TCD_SADDR);
	else
		cur_addr = readl(addr + EDMA_TCD_DADDR);

	/* figure out the finished and calculate the residue */
	for (i = 0; i < fsl_chan->edesc->n_tcds; i++) {
		size = le32_to_cpu(edesc->tcd[i].vtcd->nbytes)
			* le16_to_cpu(edesc->tcd[i].vtcd->biter);
		if (dir == DMA_MEM_TO_DEV)
			dma_addr = le32_to_cpu(edesc->tcd[i].vtcd->saddr);
		else
			dma_addr = le32_to_cpu(edesc->tcd[i].vtcd->daddr);

		len -= size;
		if (cur_addr >= dma_addr && cur_addr < dma_addr + size) {
			len += dma_addr + size - cur_addr;
			break;
		}
	}

	return len;
}

static enum dma_status fsl_edma3_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(chan, cookie, txstate);
	if (status == DMA_COMPLETE) {
		spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
		txstate->residue = fsl_chan->chn_real_count;
		spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
		return status;
	}

	if (!txstate)
		return fsl_chan->status;

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&fsl_chan->vchan, cookie);
	if (fsl_chan->edesc && cookie == fsl_chan->edesc->vdesc.tx.cookie)
		txstate->residue = fsl_edma3_desc_residue(fsl_chan, vdesc,
								true);
	else if (vdesc)
		txstate->residue = fsl_edma3_desc_residue(fsl_chan, vdesc,
								false);
	else
		txstate->residue = 0;

	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);

	return fsl_chan->status;
}

static void fsl_edma3_set_tcd_regs(struct fsl_edma3_chan *fsl_chan,
				  struct fsl_edma3_hw_tcd *tcd)
{
	void __iomem *addr = fsl_chan->membase;
	/*
	 * TCD parameters are stored in struct fsl_edma3_hw_tcd in little
	 * endian format. However, we need to load the TCD registers in
	 * big- or little-endian obeying the eDMA engine model endian.
	 */
	writew(0, addr + EDMA_TCD_CSR);
	writel(le32_to_cpu(tcd->saddr), addr + EDMA_TCD_SADDR);
	writel(le32_to_cpu(tcd->daddr), addr + EDMA_TCD_DADDR);

	writew(le16_to_cpu(tcd->attr), addr + EDMA_TCD_ATTR);
	writew(le16_to_cpu(tcd->soff), addr + EDMA_TCD_SOFF);

	writel(le32_to_cpu(tcd->nbytes), addr + EDMA_TCD_NBYTES);
	writel(le32_to_cpu(tcd->slast), addr + EDMA_TCD_SLAST);

	writew(le16_to_cpu(tcd->citer), addr + EDMA_TCD_CITER);
	writew(le16_to_cpu(tcd->biter), addr + EDMA_TCD_BITER);
	writew(le16_to_cpu(tcd->doff), addr + EDMA_TCD_DOFF);

	writel(le32_to_cpu(tcd->dlast_sga), addr + EDMA_TCD_DLAST_SGA);

	/* Must clear CHa_CSR[DONE] bit before enable TCDa_CSR[ESG] */
	if ((EDMA_TCD_CSR_E_SG | le16_to_cpu(tcd->csr)) &&
		EDMA_CH_CSR_DONE | readl(addr + EDMA_CH_CSR))
		writel(EDMA_CH_CSR_DONE, addr + EDMA_CH_CSR);

	writew(le16_to_cpu(tcd->csr), addr + EDMA_TCD_CSR);
}

static inline
void fsl_edma3_fill_tcd(struct fsl_edma3_chan *fsl_chan,
			struct fsl_edma3_hw_tcd *tcd, u32 src, u32 dst,
			u16 attr, u16 soff, u32 nbytes, u32 slast, u16 citer,
			u16 biter, u16 doff, u32 dlast_sga, bool major_int,
			bool disable_req, bool enable_sg)
{
	u16 csr = 0;

	/*
	 * eDMA hardware SGs require the TCDs to be stored in little
	 * endian format irrespective of the register endian model.
	 * So we put the value in little endian in memory, waiting
	 * for fsl_edma3_set_tcd_regs doing the swap.
	 */
	tcd->saddr = cpu_to_le32(src);
	tcd->daddr = cpu_to_le32(dst);

	tcd->attr = cpu_to_le16(attr);

	tcd->soff = cpu_to_le16(EDMA_TCD_SOFF_SOFF(soff));

	if (fsl_chan->is_dfifo) {
		/* set mloff as -8 */
		nbytes |= EDMA_TCD_NBYTES_MLOFF(-8);
		/* enable DMLOE/SMLOE */
		if (fsl_chan->fsc.dir == DMA_MEM_TO_DEV) {
			nbytes |= EDMA_TCD_NBYTES_DMLOE;
			nbytes &= ~EDMA_TCD_NBYTES_SMLOE;
		} else {
			nbytes |= EDMA_TCD_NBYTES_SMLOE;
			nbytes &= ~EDMA_TCD_NBYTES_DMLOE;
		}
	}

	tcd->nbytes = cpu_to_le32(EDMA_TCD_NBYTES_NBYTES(nbytes));
	tcd->slast = cpu_to_le32(EDMA_TCD_SLAST_SLAST(slast));

	tcd->citer = cpu_to_le16(EDMA_TCD_CITER_CITER(citer));
	tcd->doff = cpu_to_le16(EDMA_TCD_DOFF_DOFF(doff));

	tcd->dlast_sga = cpu_to_le32(EDMA_TCD_DLAST_SGA_DLAST_SGA(dlast_sga));

	tcd->biter = cpu_to_le16(EDMA_TCD_BITER_BITER(biter));
	if (major_int)
		csr |= EDMA_TCD_CSR_INT_MAJOR;

	if (disable_req)
		csr |= EDMA_TCD_CSR_D_REQ;

	if (enable_sg)
		csr |= EDMA_TCD_CSR_E_SG;

	if (fsl_chan->is_rxchan)
		csr |= EDMA_TCD_CSR_ACTIVE;

	tcd->csr = cpu_to_le16(csr);
}

static struct fsl_edma3_desc *fsl_edma3_alloc_desc(struct fsl_edma3_chan
						*fsl_chan, int sg_len)
{
	struct fsl_edma3_desc *fsl_desc;
	int i;

	fsl_desc = kzalloc(sizeof(*fsl_desc) + sizeof(struct fsl_edma3_sw_tcd)
				* sg_len, GFP_ATOMIC);
	if (!fsl_desc)
		return NULL;

	fsl_desc->echan = fsl_chan;
	fsl_desc->n_tcds = sg_len;
	for (i = 0; i < sg_len; i++) {
		fsl_desc->tcd[i].vtcd = dma_pool_alloc(fsl_chan->tcd_pool,
					GFP_ATOMIC, &fsl_desc->tcd[i].ptcd);
		if (!fsl_desc->tcd[i].vtcd)
			goto err;
	}
	return fsl_desc;

err:
	while (--i >= 0)
		dma_pool_free(fsl_chan->tcd_pool, fsl_desc->tcd[i].vtcd,
				fsl_desc->tcd[i].ptcd);
	kfree(fsl_desc);
	return NULL;
}

static struct dma_async_tx_descriptor *fsl_edma3_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	struct fsl_edma3_desc *fsl_desc;
	dma_addr_t dma_buf_next;
	int sg_len, i;
	u32 src_addr, dst_addr, last_sg, nbytes;
	u16 soff, doff, iter;

	sg_len = buf_len / period_len;
	fsl_desc = fsl_edma3_alloc_desc(fsl_chan, sg_len);
	if (!fsl_desc)
		return NULL;
	fsl_desc->iscyclic = true;

	dma_buf_next = dma_addr;
	nbytes = fsl_chan->fsc.addr_width * fsl_chan->fsc.burst;
	iter = period_len / nbytes;

	for (i = 0; i < sg_len; i++) {
		if (dma_buf_next >= dma_addr + buf_len)
			dma_buf_next = dma_addr;

		/* get next sg's physical address */
		last_sg = fsl_desc->tcd[(i + 1) % sg_len].ptcd;

		if (fsl_chan->fsc.dir == DMA_MEM_TO_DEV) {
			src_addr = dma_buf_next;
			dst_addr = fsl_chan->fsc.dev_addr;
			soff = fsl_chan->fsc.addr_width;
			if (fsl_chan->is_dfifo)
				doff = 4;
			else
				doff = 0;
		} else if (fsl_chan->fsc.dir == DMA_DEV_TO_MEM) {
			src_addr = fsl_chan->fsc.dev_addr;
			dst_addr = dma_buf_next;
			if (fsl_chan->is_dfifo)
				soff = 4;
			else
				soff = 0;
			doff = fsl_chan->fsc.addr_width;
		} else {
			/* DMA_DEV_TO_DEV */
			src_addr = fsl_chan->fsc.dev2_addr;
			dst_addr = fsl_chan->fsc.dev_addr;
			soff = 0;
			doff = 0;
		}

		fsl_edma3_fill_tcd(fsl_chan, fsl_desc->tcd[i].vtcd, src_addr,
				dst_addr, fsl_chan->fsc.attr, soff, nbytes, 0,
				iter, iter, doff, last_sg, true, false, true);
		dma_buf_next += period_len;
	}

	return vchan_tx_prep(&fsl_chan->vchan, &fsl_desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *fsl_edma3_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	struct fsl_edma3_desc *fsl_desc;
	struct scatterlist *sg;
	u32 src_addr, dst_addr, last_sg, nbytes;
	u16 soff, doff, iter;
	int i;

	if (!is_slave_direction(fsl_chan->fsc.dir))
		return NULL;

	fsl_desc = fsl_edma3_alloc_desc(fsl_chan, sg_len);
	if (!fsl_desc)
		return NULL;
	fsl_desc->iscyclic = false;

	nbytes = fsl_chan->fsc.addr_width * fsl_chan->fsc.burst;
	for_each_sg(sgl, sg, sg_len, i) {
		/* get next sg's physical address */
		last_sg = fsl_desc->tcd[(i + 1) % sg_len].ptcd;

		if (fsl_chan->fsc.dir == DMA_MEM_TO_DEV) {
			src_addr = sg_dma_address(sg);
			dst_addr = fsl_chan->fsc.dev_addr;
			soff = fsl_chan->fsc.addr_width;
			doff = 0;
		} else if (fsl_chan->fsc.dir == DMA_DEV_TO_MEM) {
			src_addr = fsl_chan->fsc.dev_addr;
			dst_addr = sg_dma_address(sg);
			soff = 0;
			doff = fsl_chan->fsc.addr_width;
		} else {
			/* DMA_DEV_TO_DEV */
			src_addr = fsl_chan->fsc.dev2_addr;
			dst_addr = fsl_chan->fsc.dev_addr;
			soff = 0;
			doff = 0;
		}

		iter = sg_dma_len(sg) / nbytes;
		if (i < sg_len - 1) {
			last_sg = fsl_desc->tcd[(i + 1)].ptcd;
			fsl_edma3_fill_tcd(fsl_chan, fsl_desc->tcd[i].vtcd,
					src_addr, dst_addr, fsl_chan->fsc.attr,
					soff, nbytes, 0, iter, iter, doff,
					last_sg, false, false, true);
		} else {
			last_sg = 0;
			fsl_edma3_fill_tcd(fsl_chan, fsl_desc->tcd[i].vtcd,
					src_addr, dst_addr, fsl_chan->fsc.attr,
					soff, nbytes, 0, iter, iter, doff,
					last_sg, true, true, false);
		}
	}

	return vchan_tx_prep(&fsl_chan->vchan, &fsl_desc->vdesc, flags);
}

static void fsl_edma3_xfer_desc(struct fsl_edma3_chan *fsl_chan)
{
	struct virt_dma_desc *vdesc;

	vdesc = vchan_next_desc(&fsl_chan->vchan);
	if (!vdesc)
		return;
	fsl_chan->edesc = to_fsl_edma3_desc(vdesc);
	fsl_edma3_set_tcd_regs(fsl_chan, fsl_chan->edesc->tcd[0].vtcd);
	fsl_edma3_enable_request(fsl_chan);
	fsl_chan->status = DMA_IN_PROGRESS;
}

static size_t fsl_edma3_desc_residue(struct fsl_edma3_chan *fsl_chan,
		struct virt_dma_desc *vdesc, bool in_progress);

static void fsl_edma3_get_realcnt(struct fsl_edma3_chan *fsl_chan)
{
	fsl_chan->chn_real_count = fsl_edma3_desc_residue(fsl_chan, NULL, true);
}

static irqreturn_t fsl_edma3_tx_handler(int irq, void *dev_id)
{
	struct fsl_edma3_chan *fsl_chan = dev_id;
	unsigned int intr;
	void __iomem *base_addr;

	base_addr = fsl_chan->membase;

	intr = readl(base_addr + EDMA_CH_INT);
	if (!intr)
		return IRQ_NONE;

	writel(1, base_addr + EDMA_CH_INT);

	spin_lock(&fsl_chan->vchan.lock);

	/* Ignore this interrupt since channel has been disabled already */
	if (!fsl_chan->edesc)
		goto irq_handled;

	if (!fsl_chan->edesc->iscyclic) {
		fsl_edma3_get_realcnt(fsl_chan);
		list_del(&fsl_chan->edesc->vdesc.node);
		vchan_cookie_complete(&fsl_chan->edesc->vdesc);
		fsl_chan->edesc = NULL;
		fsl_chan->status = DMA_COMPLETE;
	} else {
		vchan_cyclic_callback(&fsl_chan->edesc->vdesc);
	}

	if (!fsl_chan->edesc)
		fsl_edma3_xfer_desc(fsl_chan);
irq_handled:
	spin_unlock(&fsl_chan->vchan.lock);

	return IRQ_HANDLED;
}

static void fsl_edma3_issue_pending(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);

	if (vchan_issue_pending(&fsl_chan->vchan) && !fsl_chan->edesc)
		fsl_edma3_xfer_desc(fsl_chan);

	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);
}

static struct dma_chan *fsl_edma3_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *ofdma)
{
	struct fsl_edma3_engine *fsl_edma3 = ofdma->of_dma_data;
	struct dma_chan *chan, *_chan;
	struct fsl_edma3_chan *fsl_chan;

	if (dma_spec->args_count != 3)
		return NULL;

	mutex_lock(&fsl_edma3->fsl_edma3_mutex);
	list_for_each_entry_safe(chan, _chan, &fsl_edma3->dma_dev.channels,
					device_node) {
		if (chan->client_count)
			continue;

		fsl_chan = to_fsl_edma3_chan(chan);
		if (fsl_chan->hw_chanid == dma_spec->args[0]) {
			chan = dma_get_slave_channel(chan);
			chan->device->privatecnt++;
			fsl_chan->priority = dma_spec->args[1];
			fsl_chan->is_rxchan = dma_spec->args[2] & ARGS_RX;
			fsl_chan->is_remote = dma_spec->args[2] & ARGS_REMOTE;
			fsl_chan->is_dfifo = dma_spec->args[2] & ARGS_DFIFO;
			mutex_unlock(&fsl_edma3->fsl_edma3_mutex);
			return chan;
		}
	}
	mutex_unlock(&fsl_edma3->fsl_edma3_mutex);
	return NULL;
}

static int fsl_edma3_alloc_chan_resources(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);

	fsl_chan->tcd_pool = dma_pool_create("tcd_pool", chan->device->dev,
				sizeof(struct fsl_edma3_hw_tcd),
				32, 0);
	return 0;
}

static void fsl_edma3_free_chan_resources(struct dma_chan *chan)
{
	struct fsl_edma3_chan *fsl_chan = to_fsl_edma3_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	fsl_edma3_disable_request(fsl_chan);
	fsl_chan->edesc = NULL;
	vchan_get_all_descriptors(&fsl_chan->vchan, &head);
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&fsl_chan->vchan, &head);
	dma_pool_destroy(fsl_chan->tcd_pool);
	fsl_chan->tcd_pool = NULL;
}

static int fsl_edma3_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma3_engine *fsl_edma3;
	struct fsl_edma3_chan *fsl_chan;
	struct resource *res;
	int len, chans;
	int ret, i;
	unsigned long irqflag = 0;

	ret = of_property_read_u32(np, "dma-channels", &chans);
	if (ret) {
		dev_err(&pdev->dev, "Can't get dma-channels.\n");
		return ret;
	}

	len = sizeof(*fsl_edma3) + sizeof(*fsl_chan) * chans;
	fsl_edma3 = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!fsl_edma3)
		return -ENOMEM;

	/* Audio edma rx/tx channel shared interrupt */
	if (of_property_read_bool(np, "shared-interrupt"))
		irqflag = IRQF_SHARED;

	fsl_edma3->swap = of_device_is_compatible(np, "fsl,imx8qm-adma");
	fsl_edma3->n_chans = chans;

	INIT_LIST_HEAD(&fsl_edma3->dma_dev.channels);
	for (i = 0; i < fsl_edma3->n_chans; i++) {
		struct fsl_edma3_chan *fsl_chan = &fsl_edma3->chans[i];
		const char *txirq_name = fsl_chan->txirq_name;
		char chanid[3], id_len = 0;
		char *p = chanid;
		unsigned long val;

		fsl_chan->edma3 = fsl_edma3;
		/* Get per channel membase */
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		fsl_chan->membase = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(fsl_chan->membase))
			return PTR_ERR(fsl_chan->membase);

		/* Get the hardware chanel id by the channel membase
		 * channel0:0x10000, channel1:0x20000... total 32 channels
		 */
		fsl_chan->hw_chanid = (res->start >> 16) & 0x1f;

		ret = of_property_read_string_index(np, "interrupt-names", i,
							&txirq_name);
		if (ret) {
			dev_err(&pdev->dev, "read interrupt-names fail.\n");
			return ret;
		}
		/* Get channel id length from dts, one-digit or double-digit */
		id_len = strlen(txirq_name) - strlen(CHAN_PREFIX) -
			 strlen(CHAN_POSFIX);
		if (id_len > 2) {
			dev_err(&pdev->dev, "%s is edmaX-chanX-tx in dts?\n",
				res->name);
			return -EINVAL;
		}
		/* Grab channel id from txirq_name */
		strncpy(p, txirq_name + strlen(CHAN_PREFIX), id_len);
		*(p + id_len) = '\0';

		/* check if the channel id match well with hw_chanid */
		ret = kstrtoul(chanid, 0, &val);
		if (ret || val != fsl_chan->hw_chanid) {
			dev_err(&pdev->dev, "%s,wrong id?\n", txirq_name);
			return -EINVAL;
		}

		/* request channel irq */
		fsl_chan->txirq = platform_get_irq_byname(pdev, txirq_name);
		if (fsl_chan->txirq < 0) {
			dev_err(&pdev->dev, "Can't get %s irq.\n", txirq_name);
			return fsl_chan->txirq;
		}

		ret = devm_request_irq(&pdev->dev, fsl_chan->txirq,
				fsl_edma3_tx_handler, irqflag, txirq_name,
				fsl_chan);
		if (ret) {
			dev_err(&pdev->dev, "Can't register %s IRQ.\n",
				txirq_name);
			return ret;
		}

		fsl_chan->vchan.desc_free = fsl_edma3_free_desc;
		vchan_init(&fsl_chan->vchan, &fsl_edma3->dma_dev);
	}

	mutex_init(&fsl_edma3->fsl_edma3_mutex);

	dma_cap_set(DMA_PRIVATE, fsl_edma3->dma_dev.cap_mask);
	dma_cap_set(DMA_SLAVE, fsl_edma3->dma_dev.cap_mask);
	dma_cap_set(DMA_CYCLIC, fsl_edma3->dma_dev.cap_mask);

	fsl_edma3->dma_dev.dev = &pdev->dev;
	fsl_edma3->dma_dev.device_alloc_chan_resources
		= fsl_edma3_alloc_chan_resources;
	fsl_edma3->dma_dev.device_free_chan_resources
		= fsl_edma3_free_chan_resources;
	fsl_edma3->dma_dev.device_tx_status = fsl_edma3_tx_status;
	fsl_edma3->dma_dev.device_prep_slave_sg = fsl_edma3_prep_slave_sg;
	fsl_edma3->dma_dev.device_prep_dma_cyclic = fsl_edma3_prep_dma_cyclic;
	fsl_edma3->dma_dev.device_config = fsl_edma3_slave_config;
	fsl_edma3->dma_dev.device_pause = fsl_edma3_pause;
	fsl_edma3->dma_dev.device_resume = fsl_edma3_resume;
	fsl_edma3->dma_dev.device_terminate_all = fsl_edma3_terminate_all;
	fsl_edma3->dma_dev.device_issue_pending = fsl_edma3_issue_pending;

	fsl_edma3->dma_dev.src_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma3->dma_dev.dst_addr_widths = FSL_EDMA_BUSWIDTHS;
	fsl_edma3->dma_dev.directions = BIT(DMA_DEV_TO_MEM) |
					BIT(DMA_MEM_TO_DEV) |
					BIT(DMA_DEV_TO_DEV);

	platform_set_drvdata(pdev, fsl_edma3);

	ret = dma_async_device_register(&fsl_edma3->dma_dev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register Freescale eDMA engine.\n");
		return ret;
	}

	ret = of_dma_controller_register(np, fsl_edma3_xlate, fsl_edma3);
	if (ret) {
		dev_err(&pdev->dev, "Can't register Freescale eDMA of_dma.\n");
		dma_async_device_unregister(&fsl_edma3->dma_dev);
		return ret;
	}

	return 0;
}

static int fsl_edma3_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_edma3_engine *fsl_edma3 = platform_get_drvdata(pdev);

	of_dma_controller_free(np);
	dma_async_device_unregister(&fsl_edma3->dma_dev);

	return 0;
}

static const struct of_device_id fsl_edma3_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-edma", },
	{ .compatible = "fsl,imx8qm-adma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_edma3_dt_ids);

static struct platform_driver fsl_edma3_driver = {
	.driver		= {
		.name	= "fsl-edma-v3",
		.of_match_table = fsl_edma3_dt_ids,
	},
	.probe          = fsl_edma3_probe,
	.remove		= fsl_edma3_remove,
};

static int __init fsl_edma3_init(void)
{
	return platform_driver_register(&fsl_edma3_driver);
}
subsys_initcall(fsl_edma3_init);

static void __exit fsl_edma3_exit(void)
{
	platform_driver_unregister(&fsl_edma3_driver);
}
module_exit(fsl_edma3_exit);

MODULE_ALIAS("platform:fsl-edma3");
MODULE_DESCRIPTION("Freescale eDMA-V3 engine driver");
MODULE_LICENSE("GPL v2");
