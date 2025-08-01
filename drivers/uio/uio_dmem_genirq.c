// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/uio/uio_dmem_genirq.c
 *
 * Userspace I/O platform driver with generic IRQ handling code.
 *
 * Copyright (C) 2012 Damian Hobson-Garcia
 *
 * Based on uio_pdrv_genirq.c by Magnus Damm
 */

#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_data/uio_dmem_genirq.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/irq.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#define DRIVER_NAME "uio_dmem_genirq"
#define DMEM_MAP_ERROR (~0)

struct uio_dmem_genirq_platdata {
	struct uio_info *uioinfo;
	spinlock_t lock;
	unsigned long flags;
	struct platform_device *pdev;
	unsigned int dmem_region_start;
	unsigned int num_dmem_regions;
	struct mutex alloc_lock;
	unsigned int refcnt;
};

/* Bits in uio_dmem_genirq_platdata.flags */
enum {
	UIO_IRQ_DISABLED = 0,
};

static int uio_dmem_genirq_open(struct uio_info *info, struct inode *inode)
{
	struct uio_dmem_genirq_platdata *priv = info->priv;
	struct uio_mem *uiomem;

	uiomem = &priv->uioinfo->mem[priv->dmem_region_start];

	mutex_lock(&priv->alloc_lock);
	while (!priv->refcnt && uiomem < &priv->uioinfo->mem[MAX_UIO_MAPS]) {
		void *addr;
		if (!uiomem->size)
			break;

		addr = dma_alloc_coherent(&priv->pdev->dev, uiomem->size,
					  &uiomem->dma_addr, GFP_KERNEL);
		uiomem->addr = addr ? (uintptr_t) addr : DMEM_MAP_ERROR;
		++uiomem;
	}
	priv->refcnt++;

	mutex_unlock(&priv->alloc_lock);
	/* Wait until the Runtime PM code has woken up the device */
	pm_runtime_get_sync(&priv->pdev->dev);
	return 0;
}

static int uio_dmem_genirq_release(struct uio_info *info, struct inode *inode)
{
	struct uio_dmem_genirq_platdata *priv = info->priv;
	struct uio_mem *uiomem;

	/* Tell the Runtime PM code that the device has become idle */
	pm_runtime_put_sync(&priv->pdev->dev);

	uiomem = &priv->uioinfo->mem[priv->dmem_region_start];

	mutex_lock(&priv->alloc_lock);

	priv->refcnt--;
	while (!priv->refcnt && uiomem < &priv->uioinfo->mem[MAX_UIO_MAPS]) {
		if (!uiomem->size)
			break;
		if (uiomem->addr) {
			dma_free_coherent(uiomem->dma_device, uiomem->size,
					  (void *) (uintptr_t) uiomem->addr,
					  uiomem->dma_addr);
		}
		uiomem->addr = DMEM_MAP_ERROR;
		++uiomem;
	}

	mutex_unlock(&priv->alloc_lock);
	return 0;
}

static irqreturn_t uio_dmem_genirq_handler(int irq, struct uio_info *dev_info)
{
	struct uio_dmem_genirq_platdata *priv = dev_info->priv;

	/* Just disable the interrupt in the interrupt controller, and
	 * remember the state so we can allow user space to enable it later.
	 */

	spin_lock(&priv->lock);
	if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
		disable_irq_nosync(irq);
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int uio_dmem_genirq_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	struct uio_dmem_genirq_platdata *priv = dev_info->priv;
	unsigned long flags;

	/* Allow user space to enable and disable the interrupt
	 * in the interrupt controller, but keep track of the
	 * state to prevent per-irq depth damage.
	 *
	 * Serialize this operation to support multiple tasks and concurrency
	 * with irq handler on SMP systems.
	 */

	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
			enable_irq(dev_info->irq);
	} else {
		if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
			disable_irq_nosync(dev_info->irq);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void uio_dmem_genirq_pm_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_disable(dev);
}

static int uio_dmem_genirq_probe(struct platform_device *pdev)
{
	struct uio_dmem_genirq_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct uio_info *uioinfo = &pdata->uioinfo;
	struct uio_dmem_genirq_platdata *priv;
	struct uio_mem *uiomem;
	int ret = -EINVAL;
	int i;

	if (pdev->dev.of_node) {
		/* alloc uioinfo for one device */
		uioinfo = devm_kzalloc(&pdev->dev, sizeof(*uioinfo), GFP_KERNEL);
		if (!uioinfo) {
			dev_err(&pdev->dev, "unable to kmalloc\n");
			return -ENOMEM;
		}
		uioinfo->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%pOFn",
					       pdev->dev.of_node);
		uioinfo->version = "devicetree";
		/* alloc pdata */
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "unable to kmalloc\n");
			return -ENOMEM;
		}
		pdata->num_dynamic_regions = 0;
		of_property_read_u32(pdev->dev.of_node,
			"uio,number-of-dynamic-regions",
			&pdata->num_dynamic_regions);
		pdata->dynamic_region_sizes =
			kzalloc(sizeof(*pdata->dynamic_region_sizes) *
				pdata->num_dynamic_regions, GFP_KERNEL);
		if (!pdata->dynamic_region_sizes) {
			dev_err(&pdev->dev, "unable to kmalloc\n");
			return -ENOMEM;
		}
		of_property_read_u32_array(pdev->dev.of_node,
			"uio,dynamic-regions-sizes",
			pdata->dynamic_region_sizes,
			pdata->num_dynamic_regions);
	}

	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_err(&pdev->dev, "missing platform_data\n");
		return -EINVAL;
	}

	if (uioinfo->handler || uioinfo->irqcontrol ||
	    uioinfo->irq_flags & IRQF_SHARED) {
		dev_err(&pdev->dev, "interrupt configuration error\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "unable to kmalloc\n");
		return -ENOMEM;
	}

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "DMA enable failed\n");
		return ret;
	}

	priv->uioinfo = uioinfo;
	spin_lock_init(&priv->lock);
	priv->flags = 0; /* interrupt is enabled to begin with */
	priv->pdev = pdev;
	mutex_init(&priv->alloc_lock);

	if (!uioinfo->irq) {
		/* Multiple IRQs are not supported */
		ret = platform_get_irq(pdev, 0);
		if (ret == -ENXIO && pdev->dev.of_node)
			ret = UIO_IRQ_NONE;
		else if (ret < 0)
			return ret;
		uioinfo->irq = ret;
	}

	if (uioinfo->irq) {
		struct irq_data *irq_data = irq_get_irq_data(uioinfo->irq);

		/*
		 * If a level interrupt, dont do lazy disable. Otherwise the
		 * irq will fire again since clearing of the actual cause, on
		 * device level, is done in userspace
		 * irqd_is_level_type() isn't used since isn't valid until
		 * irq is configured.
		 */
		if (irq_data &&
		    irqd_get_trigger_type(irq_data) & IRQ_TYPE_LEVEL_MASK) {
			dev_dbg(&pdev->dev, "disable lazy unmask\n");
			irq_set_status_flags(uioinfo->irq, IRQ_DISABLE_UNLAZY);
		}
	}

	uiomem = &uioinfo->mem[0];

	for (i = 0; i < pdev->num_resources; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = resource_size(r);
		++uiomem;
	}

	priv->dmem_region_start = uiomem - &uioinfo->mem[0];
	priv->num_dmem_regions = pdata->num_dynamic_regions;

	for (i = 0; i < pdata->num_dynamic_regions; ++i) {
		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" dynamic and fixed memory regions.\n");
			break;
		}
		uiomem->memtype = UIO_MEM_DMA_COHERENT;
		uiomem->dma_device = &pdev->dev;
		uiomem->addr = DMEM_MAP_ERROR;
		uiomem->size = pdata->dynamic_region_sizes[i];
		++uiomem;
	}

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	/* This driver requires no hardware specific kernel code to handle
	 * interrupts. Instead, the interrupt handler simply disables the
	 * interrupt in the interrupt controller. User space is responsible
	 * for performing hardware specific acknowledge and re-enabling of
	 * the interrupt in the interrupt controller.
	 *
	 * Interrupt sharing is not supported.
	 */

	uioinfo->handler = uio_dmem_genirq_handler;
	uioinfo->irqcontrol = uio_dmem_genirq_irqcontrol;
	uioinfo->open = uio_dmem_genirq_open;
	uioinfo->release = uio_dmem_genirq_release;
	uioinfo->priv = priv;

	/* Enable Runtime PM for this device:
	 * The device starts in suspended state to allow the hardware to be
	 * turned off by default. The Runtime PM bus code should power on the
	 * hardware and enable clocks at open().
	 */
	pm_runtime_enable(&pdev->dev);

	ret = devm_add_action_or_reset(&pdev->dev, uio_dmem_genirq_pm_disable, &pdev->dev);
	if (ret)
		return ret;

	return devm_uio_register_device(&pdev->dev, priv->uioinfo);
}

static int uio_dmem_genirq_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * In this driver pm_runtime_get_sync() and pm_runtime_put_sync()
	 * are used at open() and release() time. This allows the
	 * Runtime PM code to turn off power to the device while the
	 * device is unused, ie before open() and after release().
	 *
	 * This Runtime PM callback does not need to save or restore
	 * any registers since user space is responsbile for hardware
	 * register reinitialization after open().
	 */
	return 0;
}

static const struct dev_pm_ops uio_dmem_genirq_dev_pm_ops = {
	.runtime_suspend = uio_dmem_genirq_runtime_nop,
	.runtime_resume = uio_dmem_genirq_runtime_nop,
};

#ifdef CONFIG_OF
static struct of_device_id uio_of_genirq_match[] = {
	{ .compatible = "dmem-uio", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, uio_of_genirq_match);
module_param_string(of_id, uio_of_genirq_match[0].compatible, 128, 0);
MODULE_PARM_DESC(of_id, "Openfirmware id of the device to be handled by uio");
#endif

static struct platform_driver uio_dmem_genirq = {
	.probe = uio_dmem_genirq_probe,
	.driver = {
		.name = DRIVER_NAME,
		.pm = &uio_dmem_genirq_dev_pm_ops,
		.of_match_table = of_match_ptr(uio_of_genirq_match),
	},
};

module_platform_driver(uio_dmem_genirq);

MODULE_AUTHOR("Damian Hobson-Garcia");
MODULE_DESCRIPTION("Userspace I/O platform driver with dynamic memory.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
