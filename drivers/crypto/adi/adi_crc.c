// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Support ADI SC5XX CRC HW acceleration.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 * Major Rearchitecture: April, 2023 by
 * Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/unaligned.h>
#include <crypto/internal/hash.h>
#include <crypto/scatterwalk.h>

#include "adi_crc.h"

#define DRIVER_NAME "adi-hmac-crc"
#define CHKSUM_DIGEST_SIZE           4
#define CHKSUM_BLOCK_SIZE            1
#define BUFLEN			  4096
#define CRC_TIMEOUT_MS		  1000

struct adi_crypto_crc {
	struct list_head	list;
	struct device		*dev;

	int			irq;
	struct dma_chan	*dma_ch;
	u32			poly;
	struct crc_register __iomem	*regs;

	/* Serialises use of the engine and the bounce buffer */
	struct mutex		hw_lock;
	struct completion	done;
	u8			*buf;
};

static struct adi_crypto_crc_list {
	struct list_head	dev_list;
	spinlock_t		lock;
} crc_list = {
	.dev_list = LIST_HEAD_INIT(crc_list.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(crc_list.lock),
};

struct adi_crypto_crc_reqctx {
	struct adi_crypto_crc	*crc;
	u32			result;		/* running CRC, seeded with the key */
};

struct adi_crypto_crc_ctx {
	u32			key;
};

static struct adi_crypto_crc *adi_crypto_crc_find_dev(void)
{
	struct adi_crypto_crc *crc;

	spin_lock_bh(&crc_list.lock);
	crc = list_first_entry_or_null(&crc_list.dev_list,
				       struct adi_crypto_crc, list);
	spin_unlock_bh(&crc_list.lock);

	return crc;
}

static int adi_crypto_crc_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	ctx->crc = adi_crypto_crc_find_dev();
	if (!ctx->crc)
		return -ENODEV;

	ctx->result = crc_ctx->key;

	return 0;
}

static int adi_crypto_crc_export(struct ahash_request *req, void *out)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int adi_crypto_crc_import(struct ahash_request *req, const void *in)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

/* Run @len bytes of the bounce buffer through the engine, chaining from @result */
static int adi_crypto_crc_run(struct adi_crypto_crc *crc, u32 *result,
			      unsigned int len)
{
	struct dma_slave_config dma_config = {
		.direction = DMA_MEM_TO_DEV,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
		.src_maxburst = 1,
		.dst_maxburst = 1,
	};
	struct dma_async_tx_descriptor *desc;
	struct scatterlist sg;
	int ret;

	sg_init_one(&sg, crc->buf, len);
	if (!dma_map_sg(crc->dev, &sg, 1, DMA_TO_DEVICE))
		return -ENOMEM;

	ret = dmaengine_slave_config(crc->dma_ch, &dma_config);
	if (ret) {
		dev_err(crc->dev, "Error configuring DMA channel\n");
		goto unmap;
	}

	desc = dmaengine_prep_slave_sg(crc->dma_ch, &sg, 1, DMA_MEM_TO_DEV, 0);
	if (!desc) {
		dev_err(crc->dev, "Unable to prep DMA engine\n");
		ret = -EIO;
		goto unmap;
	}

	reinit_completion(&crc->done);

	writel(0, &crc->regs->datacntrld);
	writel(BYTMIRR | (MODE_CALC_CRC << OPMODE_OFFSET), &crc->regs->control);
	writel(*result, &crc->regs->curresult);
	writel(CMPERRI | DCNTEXPI, &crc->regs->status);
	writel(CMPERRI | DCNTEXPI, &crc->regs->intrenset);

	/* set CRC data count before start DMA */
	writel(len >> 2, &crc->regs->datacnt);

	dmaengine_submit(desc);
	dma_async_issue_pending(crc->dma_ch);

	writel(readl(&crc->regs->control) | BLKEN, &crc->regs->control);

	if (!wait_for_completion_timeout(&crc->done,
					 msecs_to_jiffies(CRC_TIMEOUT_MS))) {
		dev_err(crc->dev, "CRC operation timed out\n");
		dmaengine_terminate_sync(crc->dma_ch);
		writel(readl(&crc->regs->control) & ~BLKEN, &crc->regs->control);
		ret = -ETIMEDOUT;
		goto unmap;
	}

	*result = readl(&crc->regs->result);

unmap:
	dma_unmap_sg(crc->dev, &sg, 1, DMA_TO_DEVICE);
	return ret;
}

static int adi_crypto_crc_update(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;
	unsigned int offset, len;
	int ret = 0;

	/* The engine consumes whole 32-bit words */
	if (req->nbytes % sizeof(u32))
		return -EINVAL;

	mutex_lock(&crc->hw_lock);

	for (offset = 0; offset < req->nbytes; offset += len) {
		len = min_t(unsigned int, req->nbytes - offset, BUFLEN);
		memcpy_from_sglist(crc->buf, req->src, offset, len);

		ret = adi_crypto_crc_run(crc, &ctx->result, len);
		if (ret)
			break;
	}

	mutex_unlock(&crc->hw_lock);

	return ret;
}

static int adi_crypto_crc_final(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	put_unaligned_be32(ctx->result ^ 0xFFFFFFFF, req->result);

	return 0;
}

static int adi_crypto_crc_finup(struct ahash_request *req)
{
	return adi_crypto_crc_update(req) ?: adi_crypto_crc_final(req);
}

static int adi_crypto_crc_digest(struct ahash_request *req)
{
	return adi_crypto_crc_init(req) ?: adi_crypto_crc_finup(req);
}

static int adi_crypto_crc_setkey(struct crypto_ahash *tfm, const u8 *key,
			unsigned int keylen)
{
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);

	if (keylen != CHKSUM_DIGEST_SIZE)
		return -EINVAL;

	crc_ctx->key = get_unaligned_be32(key);

	return 0;
}

static int adi_crypto_crc_init_tfm(struct crypto_ahash *tfm)
{
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);

	crc_ctx->key = 0;
	crypto_ahash_set_reqsize(tfm, sizeof(struct adi_crypto_crc_reqctx));

	return 0;
}

static struct ahash_alg algs = {
	.init		= adi_crypto_crc_init,
	.update		= adi_crypto_crc_update,
	.final		= adi_crypto_crc_final,
	.finup		= adi_crypto_crc_finup,
	.digest		= adi_crypto_crc_digest,
	.setkey		= adi_crypto_crc_setkey,
	.export		= adi_crypto_crc_export,
	.import		= adi_crypto_crc_import,
	.init_tfm	= adi_crypto_crc_init_tfm,
	.halg.digestsize	= CHKSUM_DIGEST_SIZE,
	.halg.statesize		= sizeof(struct adi_crypto_crc_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(crc32)",
		.cra_driver_name	= DRIVER_NAME,
		.cra_priority		= 100,
		/* There is no software implementation to fall back to */
		.cra_flags		= CRYPTO_ALG_ASYNC |
					  CRYPTO_ALG_OPTIONAL_KEY |
					  CRYPTO_ALG_NO_FALLBACK,
		.cra_blocksize		= CHKSUM_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct adi_crypto_crc_ctx),
		.cra_module		= THIS_MODULE,
	}
};

static irqreturn_t adi_crypto_crc_handler(int irq, void *dev_id)
{
	struct adi_crypto_crc *crc = dev_id;

	if (!(readl(&crc->regs->status) & DCNTEXP))
		return IRQ_NONE;

	writel(DCNTEXP, &crc->regs->status);
	writel(readl(&crc->regs->control) & ~BLKEN, &crc->regs->control);

	complete(&crc->done);

	return IRQ_HANDLED;
}

static const struct of_device_id adi_crypto_of_match[] = {
	{
		.compatible = "adi,hmac-crc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_crypto_of_match);

static int adi_crypto_crc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_crypto_crc *crc;
	unsigned int timeout = 100000;
	bool first;
	int ret;

	crc = devm_kzalloc(dev, sizeof(*crc), GFP_KERNEL);
	if (!crc)
		return -ENOMEM;

	crc->dev = dev;
	INIT_LIST_HEAD(&crc->list);
	mutex_init(&crc->hw_lock);
	init_completion(&crc->done);

	crc->buf = devm_kmalloc(dev, BUFLEN, GFP_KERNEL);
	if (!crc->buf)
		return -ENOMEM;

	crc->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(crc->regs))
		return dev_err_probe(dev, PTR_ERR(crc->regs), "Cannot map CRC IO\n");

	crc->irq = platform_get_irq(pdev, 0);
	if (crc->irq < 0)
		return crc->irq;

	ret = devm_request_irq(dev, crc->irq, adi_crypto_crc_handler,
			       IRQF_SHARED, dev_name(dev), crc);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to request ADI crc irq\n");

	crc->dma_ch = dma_request_chan(dev, "mdma_chan");
	if (IS_ERR(crc->dma_ch))
		return dev_err_probe(dev, PTR_ERR(crc->dma_ch),
				     "Unable to request DMA channel\n");

	of_property_read_u32(dev->of_node, "crypto_crc_poly", &crc->poly);

	writel(0, &crc->regs->control);
	writel(crc->poly, &crc->regs->poly);

	while (!(readl(&crc->regs->status) & LUTDONE) && (--timeout) > 0)
		cpu_relax();

	if (timeout == 0)
		dev_info(dev, "init crc poly timeout\n");

	platform_set_drvdata(pdev, crc);

	spin_lock(&crc_list.lock);
	first = list_empty(&crc_list.dev_list);
	list_add_tail(&crc->list, &crc_list.dev_list);
	spin_unlock(&crc_list.lock);

	if (first) {
		ret = crypto_register_ahash(&algs);
		if (ret) {
			dev_err(dev, "Can't register crypto ahash device\n");
			goto err_list;
		}
	}

	dev_info(dev, "initialized\n");

	return 0;

err_list:
	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	spin_unlock(&crc_list.lock);
	dma_release_channel(crc->dma_ch);

	return ret;
}

static void adi_crypto_crc_remove(struct platform_device *pdev)
{
	struct adi_crypto_crc *crc = platform_get_drvdata(pdev);
	bool last;

	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	last = list_empty(&crc_list.dev_list);
	spin_unlock(&crc_list.lock);

	if (last)
		crypto_unregister_ahash(&algs);

	dma_release_channel(crc->dma_ch);
}

static struct platform_driver adi_crypto_crc_driver = {
	.probe     = adi_crypto_crc_probe,
	.remove    = adi_crypto_crc_remove,
	.driver    = {
		.name  = DRIVER_NAME,
		.of_match_table = adi_crypto_of_match,
	},
};
module_platform_driver(adi_crypto_crc_driver);

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("ADI SC5XX CRC hardware crypto driver");
MODULE_LICENSE("GPL");
