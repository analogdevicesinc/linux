// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TRNG Driver for ADI SC5xx processors
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#define PKIC_POL_CTL		0x00
#define PKIC_EN_SET			0x0C
#define PKIC_EN_CLR			0x14

#define PKIC_EN_TRNG		(1 << 3)

#define TRNG_OUTPUT(x)		(x*4)
#define TRNG_STAT			0x10
#define TRNG_CTL			0x14
#define TRNG_CFG			0x18
#define TRNG_ALMCNT			0x1C
#define TRNG_FROEN			0x20
#define TRNG_FRODETUNE		0x24
#define TRNG_ALMMSK			0x28
#define TRNG_ALMSTP			0x2C

#define TRNG_STAT_RDY		(1 << 0)
#define TRNG_STAT_SHDNOVR	(1 << 1)
#define TRNG_STAT_STUCKOUT	(1 << 2)
#define TRNG_STAT_NOISEFAIL	(1 << 3)
#define TRNG_STAT_RUNFAIL	(1 << 4)
#define TRNG_STAT_LRUNFAIL	(1 << 5)
#define TRNG_STAT_PKRFAIL	(1 << 6)
#define TRNG_STAT_MBITFAIL	(1 << 7)
#define TRNG_STAT_ALL		  0xFF
#define TRNG_FRO_ALL		  0xFF

#define TRNG_CTL_TRNGEN			(1 << 10)

#define TRNG_CTL_STARTUPCYC_SHIFT		16
#define TRNG_CFG_MAXREFCYC_SHIFT		16
#define TRNG_CFG_MINREFCYC_SHIFT		0
#define TRNG_ALMCNT_ALMTHRESH_SHIFT		0
#define TRNG_ALMCNT_SHDNTHRESH_SHIFT	16

#define PING_PONG_BUFFER_SIZE 128

struct adi_fifo {
	u8 data[PING_PONG_BUFFER_SIZE];
	bool rdy;
	u32 pos;
	u32 size;
}  __packed __aligned(4);

struct adi_trng {
	struct clk *clk;
	void __iomem *base;
	void __iomem *pkic;
	struct hwrng rng;
	u32 startup_cycles;
	u32 minref_cycles;
	u32 maxref_cycles;
	u32 alarm_thresh;
	u32 shdn_thresh;

	//Poll the data instead of using irq
	u32 poll_data;

	//Ping pong fifo buffers
	struct adi_fifo fifo1;
	struct adi_fifo fifo2;
	struct adi_fifo *fifo_rd_curr;
	struct adi_fifo *fifo_wrt_curr;
};

static u32 adi_read_buffer(struct adi_trng *trng, u8 *data, size_t max, struct adi_fifo *fifo)
{
	u32 i, temp;
	u32 total = 0;

	for (i = 0; (i < max) && (fifo->pos < fifo->size); i++) {
		data[i] = fifo->data[fifo->pos];
		fifo->pos++;
		total++;
	}

	if (fifo->pos == fifo->size) {
		fifo->rdy = 0;
		fifo->pos = 0;
		trng->fifo_rd_curr = NULL;

		//Unmask TRNG RDY interrupt
		temp = readl(trng->base + TRNG_CTL) | TRNG_STAT_ALL;
		writel(temp, trng->base + TRNG_CTL);
	}

	return total;
}

static void adi_write_buffer(struct adi_trng *trng, struct adi_fifo *fifo)
{
	u32 i;
	u32 *data = (u32 *)&fifo->data[fifo->pos];

	for (i = 0; i < 4; i++)
		data[i] = readl(trng->base + TRNG_OUTPUT(i));

	fifo->pos += 16;

	if (fifo->pos == fifo->size) {
		fifo->rdy = 1;
		fifo->pos = 0;
		trng->fifo_wrt_curr = NULL;
	}
}

static int adi_trng_read(struct hwrng *rng, void *buf, size_t max,
			   bool wait)
{
	struct adi_trng *trng = container_of(rng, struct adi_trng, rng);
	u32 *data = buf;
	u32 i, stat, words, total;

	if (likely(!trng->poll_data)) {
		//Use ping pong buffers + interrupt
		if (trng->fifo_rd_curr == NULL) {
			if (trng->fifo1.rdy)
				trng->fifo_rd_curr = &trng->fifo1;
			else if (trng->fifo2.rdy)
				trng->fifo_rd_curr = &trng->fifo2;
		}

		if (trng->fifo_rd_curr != NULL)
			return adi_read_buffer(trng, (u8 *)data, max, trng->fifo_rd_curr);
	} else {
		//Poll data directly
		words = max / 4;
		if (words > 4)
			words = 4;

		stat = readl(trng->base + TRNG_STAT);

		if (stat & TRNG_STAT_RDY) {
			for (i = 0; i < words; i++) {
				data[i] = readl(trng->base + TRNG_OUTPUT(i));
				total += 4;
			}
			writel(TRNG_STAT_RDY, trng->base + TRNG_STAT);
			return total;
		}
	}

	return 0;
}

static void adi_trng_enable(struct adi_trng *trng)
{
	u32 temp;

	temp = trng->startup_cycles << TRNG_CTL_STARTUPCYC_SHIFT;

	//Enable TRNG Peripheral
	temp |= TRNG_CTL_TRNGEN;
	writel(temp, trng->base + TRNG_CTL);

	//Set TRNG interrupt polarity to high in PKIC
	writel(PKIC_EN_TRNG, trng->pkic + PKIC_POL_CTL);

	//Enable TRNG interrupt in PKIC
	writel(PKIC_EN_TRNG, trng->pkic + PKIC_EN_SET);

	//Enable TRNG interrupt masks
	temp |= TRNG_STAT_ALL;
	if (unlikely(trng->poll_data)) {
		//Disable the RDY IRQ enable mask,
		//as we're polling instead
		temp &= ~TRNG_STAT_RDY;
	}
	writel(temp, trng->base + TRNG_CTL);
}

static void adi_trng_disable(struct adi_trng *trng)
{
	//Clear TRNG control register
	writel(0x0, trng->base + TRNG_CTL);

	//Disable TRNG interrupt in PKIC
	writel(PKIC_EN_TRNG, trng->pkic + PKIC_EN_CLR);
}

static irqreturn_t adi_trng_irq(int irq, void *priv)
{
	struct adi_trng *trng = (struct adi_trng *)priv;
	u32 stat = readl(trng->base + TRNG_STAT);
	u32 temp;

	if (likely(!trng->poll_data)) {
		if (stat & TRNG_STAT_RDY) {
			if (trng->fifo_wrt_curr == NULL) {
				if (!trng->fifo1.rdy)
					trng->fifo_wrt_curr = &trng->fifo1;
				else if (!trng->fifo2.rdy)
					trng->fifo_wrt_curr = &trng->fifo2;
			}

			if (trng->fifo_wrt_curr != NULL)
				adi_write_buffer(trng, trng->fifo_wrt_curr);

			if (trng->fifo1.rdy && trng->fifo2.rdy) {
				//Mask TRNG interrupts
				temp = readl(trng->base + TRNG_CTL);
				writel(temp & ~TRNG_STAT_ALL, trng->base + TRNG_CTL);
			}

			writel(TRNG_STAT_RDY, trng->base + TRNG_STAT);
		}
	}

	if (unlikely(stat & TRNG_STAT_SHDNOVR)) {
		u32 alarm = readl(trng->base + TRNG_ALMSTP);
		u32 tune = readl(trng->base + TRNG_FRODETUNE);

		//Clear alarms
		writel(0x0, trng->base + TRNG_ALMMSK);
		writel(0x0, trng->base + TRNG_ALMSTP);

		//Invert tuning bit on failed FROs
		writel(tune ^ alarm, trng->base + TRNG_FRODETUNE);

		//Restart all FROs
		writel(TRNG_FRO_ALL, trng->base + TRNG_FROEN);

		pr_debug("adi-rng.c: TRNG_STAT_SHDNOVR was set\n");
		writel(TRNG_STAT_SHDNOVR | TRNG_STAT_RDY, trng->base + TRNG_STAT);
	}

	//From reference manual:
	//"Note that incidental alarm events are expected to occur during normal operation"
	if (unlikely(stat & TRNG_STAT_STUCKOUT)) {
		pr_debug("adi-rng.c: TRNG_STAT_STUCKOUT was set\n");
		writel(TRNG_STAT_STUCKOUT, trng->base + TRNG_STAT);
	}

	if (unlikely(stat & TRNG_STAT_NOISEFAIL)) {
		pr_debug("adi-rng.c: TRNG_STAT_NOISEFAIL was set\n");
		writel(TRNG_STAT_NOISEFAIL, trng->base + TRNG_STAT);
	}

	if (unlikely(stat & TRNG_STAT_RUNFAIL)) {
		pr_debug("adi-rng.c: TRNG_STAT_RUNFAIL was set\n");
		writel(TRNG_STAT_RUNFAIL, trng->base + TRNG_STAT);
	}

	if (unlikely(stat & TRNG_STAT_LRUNFAIL)) {
		pr_debug("adi-rng.c: TRNG_STAT_LRUNFAIL was set\n");
		writel(TRNG_STAT_LRUNFAIL, trng->base + TRNG_STAT);
	}

	if (unlikely(stat & TRNG_STAT_PKRFAIL)) {
		pr_debug("adi-rng.c: TRNG_STAT_PKRFAIL was set\n");
		writel(TRNG_STAT_PKRFAIL, trng->base + TRNG_STAT);
	}

	if (unlikely(stat & TRNG_STAT_MBITFAIL)) {
		pr_debug("adi-rng.c: TRNG_STAT_MBITFAIL was set\n");
		writel(TRNG_STAT_MBITFAIL, trng->base + TRNG_STAT);
	}

	return IRQ_HANDLED;
}

static int adi_trng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_trng *trng;
	struct resource *res;
	int ret;
	int irq;
	u32 temp;

	trng = devm_kzalloc(dev, sizeof(*trng), GFP_KERNEL);
	if (!trng)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	trng->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(trng->base))
		return PTR_ERR(trng->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	trng->pkic = devm_ioremap_resource(dev, res);
	if (IS_ERR(trng->pkic))
		return PTR_ERR(trng->pkic);

	if (of_property_read_u32(np, "startup-cycles", &trng->startup_cycles)) {
		dev_err(dev, "Missing startup-cycles\n");
		return -ENOENT;
	}

	if (of_property_read_u32(np, "minref-cycles", &trng->minref_cycles)) {
		dev_err(dev, "Missing minref-cycles\n");
		return -ENOENT;
	}

	if (of_property_read_u32(np, "maxref-cycles", &trng->maxref_cycles)) {
		dev_err(dev, "Missing maxref-cycles\n");
		return -ENOENT;
	}

	if (of_property_read_u32(np, "alarm-thresh", &trng->alarm_thresh)) {
		dev_err(dev, "Missing alarm-thresh\n");
		return -ENOENT;
	}

	if (of_property_read_u32(np, "shdn-thresh", &trng->shdn_thresh)) {
		dev_err(dev, "Missing shdn-thresh\n");
		return -ENOENT;
	}

	if (of_property_read_u32(np, "poll-data", &trng->poll_data)) {
		dev_err(dev, "Missing poll-data\n");
		return -ENOENT;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "Couldn't get irq %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(dev,
			irq, adi_trng_irq, 0, pdev->name, (void *)trng);
	if (ret) {
		dev_err(dev, "Unable to request irq\n");
		goto err;
	}

	trng->rng.name = pdev->name;
	trng->rng.read = adi_trng_read;

	//Disable TRNG
	writel(0x0, trng->base + TRNG_CTL);

	//Adjust min/max ref cycles in config register
	temp = trng->minref_cycles << TRNG_CFG_MINREFCYC_SHIFT;
	temp |= trng->maxref_cycles << TRNG_CFG_MAXREFCYC_SHIFT;
	writel(temp, trng->base + TRNG_CFG);

	//Set all FRO tuning values to 0 to start
	writel(0x0, trng->base + TRNG_FRODETUNE);

	//Enable all free running oscillators (FROs)
	writel(TRNG_FRO_ALL, trng->base + TRNG_FROEN);

	//Set ALMTHRESH+SHDNTHRESH
	temp = (trng->alarm_thresh << TRNG_ALMCNT_ALMTHRESH_SHIFT);
	temp |= (trng->shdn_thresh << TRNG_ALMCNT_SHDNTHRESH_SHIFT);
	writel(temp, trng->base + TRNG_ALMCNT);

	//Initialize ping pong buffers (only used if not polling)
	trng->fifo1.size = PING_PONG_BUFFER_SIZE;
	trng->fifo1.pos = 0;
	trng->fifo1.rdy = 0;
	trng->fifo2.size = PING_PONG_BUFFER_SIZE;
	trng->fifo2.pos = 0;
	trng->fifo2.rdy = 0;
	trng->fifo_wrt_curr = NULL;
	trng->fifo_rd_curr = NULL;

	ret = devm_hwrng_register(dev, &trng->rng);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, trng);

	adi_trng_enable(trng);

	return 0;
err:
	return ret;
}

static int adi_trng_remove(struct platform_device *pdev)
{
	struct adi_trng *trng = platform_get_drvdata(pdev);

	adi_trng_disable(trng);
	return 0;
}

static const struct of_device_id adi_trng_dt_ids[] = {
	{ .compatible = "adi,sc5xx-trng" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, adi_trng_dt_ids);

static struct platform_driver adi_trng_driver = {
	.probe		= adi_trng_probe,
	.remove		= adi_trng_remove,
	.driver		= {
		.name	= "adi-trng",
		.of_match_table = adi_trng_dt_ids,
	},
};

module_platform_driver(adi_trng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nathan Barrett-Morrison <nathan.morrison@timesys.com>");
