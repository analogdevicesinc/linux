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

#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <asm/unaligned.h>
#include <linux/io.h>

#include "adi_crc.h"

DEFINE_MUTEX(crc_mutex);

#define CRC_CCRYPTO_QUEUE_LENGTH     5

#define DRIVER_NAME "adi-hmac-crc"
#define CHKSUM_DIGEST_SIZE           4
#define CHKSUM_BLOCK_SIZE            1
#define BUFLEN			  4096

#define CRC_MAX_DMA_DESC	   100

struct adi_crypto_crc {
	struct list_head	list;
	struct device		*dev;
	spinlock_t		lock;

	int			irq;
	struct dma_chan	*dma_ch;
	u32			poly;
	struct crc_register	*regs;

	struct ahash_request	*req;        /* current request in operation */
	struct dma_desc_array	*sg_cpu;     /* virt addr of sg dma descriptors */
	dma_addr_t		sg_dma;      /* phy addr of sg dma descriptors */
};

static struct adi_crypto_crc_list {
	struct list_head	dev_list;
	spinlock_t		lock;
} crc_list;

struct adi_crypto_crc_reqctx {
	struct adi_crypto_crc	*crc;

	unsigned int		total;		/* total request bytes */
	struct scatterlist	sg_list[1];     /* chained sg list */
};

struct adi_crypto_crc_ctx {
	struct adi_crypto_crc	*crc;
	u32			key;
	u8			xmit_buf[BUFLEN] __aligned(sizeof(u32));
};

static int adi_crypto_crc_init_hw(struct adi_crypto_crc *crc, u32 key)
{
	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	writel(0, &crc->regs->datacntrld);

	writel(BYTMIRR | (MODE_CALC_CRC << OPMODE_OFFSET), &crc->regs->control);

	writel(key, &crc->regs->curresult);

	/* setup CRC interrupts */
	writel(CMPERRI | DCNTEXPI, &crc->regs->status);
	writel(CMPERRI | DCNTEXPI, &crc->regs->intrenset);

	return 0;
}

static int adi_crypto_crc_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc;

	spin_lock_bh(&crc_list.lock);
	list_for_each_entry(crc, &crc_list.dev_list, list) {
		crc_ctx->crc = crc;
		break;
	}
	spin_unlock_bh(&crc_list.lock);

	ctx->crc = crc;
	ctx->total = 0;
	memset(ctx->sg_list, 0, 2 * sizeof(ctx->sg_list[0]));

	dev_dbg(crc->dev, "init: digest size: %d\n",
		crypto_ahash_digestsize(tfm));

	return adi_crypto_crc_init_hw(crc, crc_ctx->key);
}

static int adi_crypto_crc_export(struct ahash_request *req, void *out)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int adi_crypto_crc_import(struct ahash_request *req, const void *in)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static void adi_crypto_crc_config_dma(struct adi_crypto_crc *crc)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(crc->req);
	struct dma_slave_config dma_config = {0};
	struct dma_async_tx_descriptor *desc;
	int ret;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	dma_map_sg(crc->dev, ctx->sg_list, 1, DMA_TO_DEVICE);

	dma_config.direction = DMA_DEV_TO_MEM;
	dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma_config.src_maxburst = 1;
	dma_config.dst_maxburst = 1;

	ret = dmaengine_slave_config(crc->dma_ch, &dma_config);
	if (ret) {
		dev_err(crc->dev, "Error configuring DMA channel\n");
		return;
	}

	desc = dmaengine_prep_slave_sg(crc->dma_ch, ctx->sg_list, 1,
		DMA_MEM_TO_DEV, 0);

	if (!desc) {
		dev_err(crc->dev, "Unable to prep DMA engine\n");
		return;
	}

	dmaengine_submit(desc);
	dma_async_issue_pending(crc->dma_ch);
}

static int adi_crypto_crc_update(struct ahash_request *req)
{
	u32 reg;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;
	int ret = 0;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	//TODO Someday: Add ping pong buffers so we can prep multiple
	//xmit buffers while the engine is busy processing the first 4KB chunk

	if (!req->nbytes)
		return 0;

	if (ctx->total + req->nbytes <= BUFLEN) {
		mutex_lock_interruptible(&crc_mutex);

		scatterwalk_map_and_copy(crc_ctx->xmit_buf + ctx->total, req->src,
					0, req->nbytes, 0);

		ctx->total += req->nbytes;

		sg_init_one(ctx->sg_list, crc_ctx->xmit_buf, ctx->total);

		crc->req = req;

		/* set CRC data count before start DMA */
		writel((ctx->total & ~0x3) >> 2, &crc->regs->datacnt);

		/* setup and enable CRC DMA */
		adi_crypto_crc_config_dma(crc);

		/* finally kick off CRC operation */
		reg = readl(&crc->regs->control);
		writel(reg | BLKEN, &crc->regs->control);

		ctx->total = 0;
	} else {
		ret = -ENOBUFS;
	}

	return ret;
}

static int adi_crypto_crc_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	mutex_lock_interruptible(&crc_mutex);

	put_unaligned_be32(readl(&crc->regs->result) ^ 0xFFFFFFFF,
			crc->req->result);

	mutex_unlock(&crc_mutex);

	crc_ctx->key = 0;

	return 0;
}

static int adi_crypto_crc_finup(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;
	int ret = 0;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	ret = adi_crypto_crc_update(req);
	if (ret == -ENOBUFS)
		return ret;

	return adi_crypto_crc_final(req);
}

static int adi_crypto_crc_digest(struct ahash_request *req)
{
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	struct adi_crypto_crc *crc = ctx->crc;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

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

static int adi_crypto_crc_cra_init(struct crypto_tfm *tfm)
{
	struct adi_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);

	crc_ctx->key = 0;
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct adi_crypto_crc_reqctx));

	return 0;
}

static void adi_crypto_crc_cra_exit(struct crypto_tfm *tfm)
{
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
	.halg.digestsize	= CHKSUM_DIGEST_SIZE,
	.halg.statesize		= sizeof(struct adi_crypto_crc_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(crc32)",
		.cra_driver_name	= DRIVER_NAME,
		.cra_priority		= 100,
		.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
						CRYPTO_ALG_ASYNC |
						CRYPTO_ALG_OPTIONAL_KEY,
		.cra_blocksize		= CHKSUM_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct adi_crypto_crc_ctx),
		.cra_alignmask		= 3,
		.cra_module		= THIS_MODULE,
		.cra_init		= adi_crypto_crc_cra_init,
		.cra_exit		= adi_crypto_crc_cra_exit,
	}
};

static irqreturn_t adi_crypto_crc_handler(int irq, void *dev_id)
{
	struct adi_crypto_crc *crc = dev_id;
	struct adi_crypto_crc_reqctx *ctx = ahash_request_ctx(crc->req);
	u32 reg;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	if (readl(&crc->regs->status) & DCNTEXP) {
		writel(DCNTEXP, &crc->regs->status);

		reg = readl(&crc->regs->control);
		writel(reg & ~BLKEN, &crc->regs->control);

		if (crc->req->base.complete)
			crc->req->base.complete(&crc->req->base, 0);

		dma_unmap_sg(crc->dev, ctx->sg_list, 1, DMA_TO_DEVICE);

		mutex_unlock(&crc_mutex);

		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

/**
 *	adi_crypto_crc_suspend - suspend crc device
 *	@pdev: device being suspended
 *	@state: requested suspend state
 */
static __maybe_unused int adi_crypto_crc_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct adi_crypto_crc *crc = platform_get_drvdata(pdev);
	int i = 100000;

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	while ((readl(&crc->regs->control) & BLKEN) && --i)
		cpu_relax();

	if (i == 0)
		return -EBUSY;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adi_crypto_of_match[] = {
	{
		.compatible = "adi,hmac-crc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_crypto_of_match);
#endif

/**
 *	adi_crypto_crc_probe - Initialize module
 *
 */
static int adi_crypto_crc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct adi_crypto_crc *crc;
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	unsigned int timeout = 100000;
	int ret;

	dev_dbg(dev, "%s @ %d\n", __func__, __LINE__);

	crc = devm_kzalloc(dev, sizeof(*crc), GFP_KERNEL);
	if (!crc)
		return -ENOMEM;

	crc->dev = dev;

	INIT_LIST_HEAD(&crc->list);
	spin_lock_init(&crc->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	crc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR((void *)crc->regs)) {
		dev_err(&pdev->dev, "Cannot map CRC IO\n");
		return PTR_ERR((void *)crc->regs);
	}

	crc->irq = platform_get_irq(pdev, 0);
	if (crc->irq < 0) {
		dev_err(&pdev->dev, "No CRC DCNTEXP IRQ specified\n");
		return -ENOENT;
	}

	ret = devm_request_irq(dev, crc->irq, adi_crypto_crc_handler,
			IRQF_SHARED, dev_name(dev), crc);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request ADI crc irq\n");
		return ret;
	}

	match = of_match_device(of_match_ptr(adi_crypto_of_match), &pdev->dev);
	if (match) {
		crc->dma_ch = dma_request_chan(dev, "mdma_chan");
		of_property_read_u32(node, "crypto_crc_poly", &crc->poly);
	} else {
		dev_err(&pdev->dev, "Device Tree Error\n");
		return -ENOENT;
	}

	crc->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &crc->sg_dma, GFP_KERNEL);
	if (crc->sg_cpu == NULL) {
		ret = -ENOMEM;
		goto out_error_dma;
	}

	writel(0, &crc->regs->control);
	writel(crc->poly, &crc->regs->poly);

	while (!(readl(&crc->regs->status) & LUTDONE) && (--timeout) > 0)
		cpu_relax();

	if (timeout == 0)
		dev_info(&pdev->dev, "init crc poly timeout\n");

	platform_set_drvdata(pdev, crc);

	spin_lock(&crc_list.lock);
	list_add(&crc->list, &crc_list.dev_list);
	spin_unlock(&crc_list.lock);

	if (list_is_singular(&crc_list.dev_list)) {
		ret = crypto_register_ahash(&algs);
		if (ret) {
			dev_err(&pdev->dev,
				"Can't register crypto ahash device\n");
			goto out_error_dma;
		}
	}

	dev_info(&pdev->dev, "initialized\n");

	return 0;

out_error_dma:
	if (crc->sg_cpu)
		dma_free_coherent(&pdev->dev, PAGE_SIZE, crc->sg_cpu, crc->sg_dma);
	dma_release_channel(crc->dma_ch);

	return ret;
}

/**
 *	adi_crypto_crc_remove - Initialize module
 *
 */
static int adi_crypto_crc_remove(struct platform_device *pdev)
{
	struct adi_crypto_crc *crc = platform_get_drvdata(pdev);

	dev_dbg(crc->dev, "%s @ %d\n", __func__, __LINE__);

	if (!crc)
		return -ENODEV;

	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	spin_unlock(&crc_list.lock);

	crypto_unregister_ahash(&algs);
	dma_release_channel(crc->dma_ch);

	return 0;
}

static struct platform_driver adi_crypto_crc_driver = {
	.probe     = adi_crypto_crc_probe,
	.remove    = adi_crypto_crc_remove,
	.suspend   = adi_crypto_crc_suspend,
	.driver    = {
		.name  = DRIVER_NAME,
		.of_match_table = of_match_ptr(adi_crypto_of_match),
	},
};

/**
 *	adi_crypto_crc_mod_init - Initialize module
 *
 *	Checks the module params and registers the platform driver.
 *	Real work is in the platform probe function.
 */
static int __init adi_crypto_crc_mod_init(void)
{
	int ret;

	pr_info("ADI hardware CRC crypto driver\n");

	INIT_LIST_HEAD(&crc_list.dev_list);
	spin_lock_init(&crc_list.lock);

	ret = platform_driver_register(&adi_crypto_crc_driver);
	if (ret) {
		pr_err("unable to register driver\n");
		return ret;
	}

	return 0;
}

/**
 *	adi_crypto_crc_mod_exit - Deinitialize module
 */
static void __exit adi_crypto_crc_mod_exit(void)
{
	platform_driver_unregister(&adi_crypto_crc_driver);
}

module_init(adi_crypto_crc_mod_init);
module_exit(adi_crypto_crc_mod_exit);

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("ADI SC5XX CRC hardware crypto driver");
MODULE_LICENSE("GPL");
