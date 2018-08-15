/*
 * host.c - Cadence USB3 host controller driver
 *
 * Copyright 2017 NXP
 * Authors: Peter Chen <peter.chen@nxp.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/pm_runtime.h>
#include <linux/usb/of.h>

#include "../host/xhci.h"

#include "core.h"
#include "host-export.h"
#include "cdns3-nxp-reg-def.h"

static struct hc_driver __read_mostly xhci_cdns3_hc_driver;

static void xhci_cdns3_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= (XHCI_PLAT | XHCI_CDNS_HOST);
}

static int xhci_cdns3_setup(struct usb_hcd *hcd)
{
	int ret;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	u32 command;

	ret = xhci_gen_setup(hcd, xhci_cdns3_quirks);
	if (ret)
		return ret;
	/* set usbcmd.EU3S */
	command = readl(&xhci->op_regs->command);
	command |= CMD_PM_INDEX;
	writel(command, &xhci->op_regs->command);

	return 0;
}

struct cdns3_host {
	struct device dev;
	struct usb_hcd *hcd;
	struct cdns3 *cdns;
};

static int xhci_cdns3_bus_suspend(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	struct cdns3_host *host = container_of(dev, struct cdns3_host, dev);
	struct cdns3 *cdns = host->cdns;
	void __iomem *xhci_regs = cdns->xhci_regs;
	u32 value;
	int ret;

	ret = xhci_bus_suspend(hcd);
	if (ret)
		return ret;

	value = readl(xhci_regs + XECP_AUX_CTRL_REG1);
	value |= CFG_RXDET_P3_EN;
	writel(value, xhci_regs + XECP_AUX_CTRL_REG1);

	return 0;
}

static const struct xhci_driver_overrides xhci_cdns3_overrides __initconst = {
	.extra_priv_size = sizeof(struct xhci_hcd),
	.reset = xhci_cdns3_setup,
	.bus_suspend = xhci_cdns3_bus_suspend,
};

static irqreturn_t cdns3_host_irq(struct cdns3 *cdns)
{
	struct device *dev = cdns->host_dev;
	struct usb_hcd	*hcd;

	if (dev)
		hcd = dev_get_drvdata(dev);
	else
		return IRQ_NONE;

	if (hcd)
		return usb_hcd_irq(cdns->irq, hcd);
	else
		return IRQ_NONE;
}

static void cdns3_host_release(struct device *dev)
{
	struct cdns3_host *host = container_of(dev, struct cdns3_host, dev);

	dev_dbg(dev, "releasing '%s'\n", dev_name(dev));
	kfree(host);
}

static int cdns3_host_start(struct cdns3 *cdns)
{
	struct cdns3_host *host;
	struct device *dev;
	struct device *sysdev;
	struct xhci_hcd	*xhci;
	int ret;

	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	dev = &host->dev;
	dev->release = cdns3_host_release;
	dev->parent = cdns->dev;
	dev_set_name(dev, "xhci-cdns3");
	cdns->host_dev = dev;
	host->cdns = cdns;
	ret = device_register(dev);
	if (ret)
		goto err1;

	sysdev = cdns->dev;
	/* Try to set 64-bit DMA first */
	if (WARN_ON(!sysdev->dma_mask))
		/* Platform did not initialize dma_mask */
		ret = dma_coerce_mask_and_coherent(sysdev,
						   DMA_BIT_MASK(64));
	else
		ret = dma_set_mask_and_coherent(sysdev, DMA_BIT_MASK(64));

	/* If setting 64-bit DMA mask fails, fall back to 32-bit DMA mask */
	if (ret) {
		ret = dma_set_mask_and_coherent(sysdev, DMA_BIT_MASK(32));
		if (ret)
			return ret;
	}
	pm_runtime_set_active(dev);
	pm_runtime_no_callbacks(dev);
	pm_runtime_enable(dev);

	host->hcd = __usb_create_hcd(&xhci_cdns3_hc_driver, sysdev, dev,
			       dev_name(dev), NULL);
	if (!host->hcd) {
		ret = -ENOMEM;
		goto err2;
	}

	host->hcd->regs = cdns->xhci_regs;
	host->hcd->rsrc_start = cdns->xhci_res->start;
	host->hcd->rsrc_len = resource_size(cdns->xhci_res);

	device_wakeup_enable(host->hcd->self.controller);

	xhci = hcd_to_xhci(host->hcd);

	xhci->quirks = XHCI_SKIP_ACCESS_RESERVED_REG;
	xhci->main_hcd = host->hcd;
	xhci->shared_hcd = __usb_create_hcd(&xhci_cdns3_hc_driver, sysdev, dev,
			dev_name(dev), host->hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto err3;
	}
	host->hcd->tpl_support = of_usb_host_tpl_support(sysdev->of_node);
	xhci->shared_hcd->tpl_support = host->hcd->tpl_support;

	ret = usb_add_hcd(host->hcd, 0, IRQF_SHARED);
	if (ret)
		goto err4;

	ret = usb_add_hcd(xhci->shared_hcd, 0, IRQF_SHARED);
	if (ret)
		goto err5;

	device_set_wakeup_capable(dev, true);
	dev_dbg(dev, "%s ends\n", __func__);

	return 0;

err5:
	usb_remove_hcd(host->hcd);
err4:
	usb_put_hcd(xhci->shared_hcd);
err3:
	usb_put_hcd(host->hcd);
err2:
	device_del(dev);
err1:
	put_device(dev);
	cdns->host_dev = NULL;
	return ret;
}

static void cdns3_host_stop(struct cdns3 *cdns)
{
	struct device *dev = cdns->host_dev;
	struct usb_hcd	*hcd;
	struct xhci_hcd	*xhci;

	if (dev) {
		hcd = dev_get_drvdata(dev);
		xhci = hcd_to_xhci(hcd);
		usb_remove_hcd(xhci->shared_hcd);
		usb_remove_hcd(hcd);
		synchronize_irq(cdns->irq);
		usb_put_hcd(xhci->shared_hcd);
		usb_put_hcd(hcd);
		cdns->host_dev = NULL;
		pm_runtime_set_suspended(dev);
		pm_runtime_disable(dev);
		device_del(dev);
		put_device(dev);
	}
}

static int cdns3_host_suspend(struct cdns3 *cdns, bool do_wakeup)
{
	struct device *dev = cdns->host_dev;
	struct xhci_hcd	*xhci;

	if (!dev)
		return 0;

	xhci = hcd_to_xhci(dev_get_drvdata(dev));
	return xhci_suspend(xhci, do_wakeup);
}

static int cdns3_host_resume(struct cdns3 *cdns, bool hibernated)
{
	struct device *dev = cdns->host_dev;
	struct xhci_hcd	*xhci;

	if (!dev)
		return 0;

	xhci = hcd_to_xhci(dev_get_drvdata(dev));
	return xhci_resume(xhci, hibernated);
}

int cdns3_host_init(struct cdns3 *cdns)
{
	struct cdns3_role_driver *rdrv;

	rdrv = devm_kzalloc(cdns->dev, sizeof(*rdrv), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start	= cdns3_host_start;
	rdrv->stop	= cdns3_host_stop;
	rdrv->irq	= cdns3_host_irq;
	rdrv->suspend	= cdns3_host_suspend;
	rdrv->resume	= cdns3_host_resume;
	rdrv->name	= "host";
	cdns->roles[CDNS3_ROLE_HOST] = rdrv;

	return 0;
}

void cdns3_host_remove(struct cdns3 *cdns)
{
	cdns3_host_stop(cdns);
}

void __init cdns3_host_driver_init(void)
{
	xhci_init_driver(&xhci_cdns3_hc_driver, &xhci_cdns3_overrides);
}
