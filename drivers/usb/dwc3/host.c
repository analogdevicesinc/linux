// SPDX-License-Identifier: GPL-2.0
/*
 * host.c - DesignWare USB3 DRD Controller Host Glue
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - https://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 */

#include <linux/irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/of_device.h>

#include "../host/xhci-port.h"
#include "../host/xhci-caps.h"
#include "../host/xhci-plat.h"
#include "core.h"
#include <../drivers/usb/host/xhci.h>

#define XHCI_HCSPARAMS1		0x4
#define XHCI_PORTSC_BASE	0x400

static dwc3_wakeup_t dwc3_wakeup_fn;

/**
 * dwc3_power_off_all_roothub_ports - Power off all Root hub ports
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_power_off_all_roothub_ports(struct dwc3 *dwc)
{
	void __iomem *xhci_regs;
	u32 op_regs_base;
	int port_num;
	u32 offset;
	u32 reg;
	int i;

	/* xhci regs is not mapped yet, do it temperary here */
	if (dwc->xhci_resources[0].start) {
		xhci_regs = ioremap(dwc->xhci_resources[0].start, DWC3_XHCI_REGS_END);
		if (!xhci_regs) {
			dev_err(dwc->dev, "Failed to ioremap xhci_regs\n");
			return;
		}

		op_regs_base = HC_LENGTH(readl(xhci_regs));
		reg = readl(xhci_regs + XHCI_HCSPARAMS1);
		port_num = HCS_MAX_PORTS(reg);

		for (i = 1; i <= port_num; i++) {
			offset = op_regs_base + XHCI_PORTSC_BASE + 0x10 * (i - 1);
			reg = readl(xhci_regs + offset);
			reg &= ~PORT_POWER;
			writel(reg, xhci_regs + offset);
		}

		iounmap(xhci_regs);
	} else {
		dev_err(dwc->dev, "xhci base reg invalid\n");
	}
}

static void dwc3_xhci_plat_start(struct usb_hcd *hcd)
{
	struct platform_device *pdev;
	struct dwc3 *dwc;

	if (!usb_hcd_is_primary_hcd(hcd))
		return;

	pdev = to_platform_device(hcd->self.controller);
	dwc = dev_get_drvdata(pdev->dev.parent);

	dwc3_enable_susphy(dwc, true);
}

static const struct xhci_plat_priv dwc3_xhci_plat_quirk = {
	.plat_start = dwc3_xhci_plat_start,
};

 /* dwc3 host wakeup registration */
void dwc3_host_wakeup_register(dwc3_wakeup_t func)
{
	dwc3_wakeup_fn = func;
}
EXPORT_SYMBOL_GPL(dwc3_host_wakeup_register);

/* callback function */
void dwc3_host_wakeup_capable(struct device *dev, bool wakeup)
{
	if (dwc3_wakeup_fn)
		dwc3_wakeup_fn(dev, wakeup);
}
EXPORT_SYMBOL_GPL(dwc3_host_wakeup_capable);

static void dwc3_host_fill_xhci_irq_res(struct dwc3 *dwc,
					int irq, char *name)
{
	struct platform_device *pdev = to_platform_device(dwc->dev);
	struct device_node *np = dev_of_node(&pdev->dev);

	dwc->xhci_resources[1].start = irq;
	dwc->xhci_resources[1].end = irq;
	dwc->xhci_resources[1].flags = IORESOURCE_IRQ | irq_get_trigger_type(irq);
	if (!name && np)
		dwc->xhci_resources[1].name = of_node_full_name(pdev->dev.of_node);
	else
		dwc->xhci_resources[1].name = name;
}

static int dwc3_host_get_irq(struct dwc3 *dwc)
{
	struct platform_device	*dwc3_pdev = to_platform_device(dwc->dev);
	int irq;

	irq = platform_get_irq_byname_optional(dwc3_pdev, "host");
	if (irq > 0) {
		dwc3_host_fill_xhci_irq_res(dwc, irq, "host");
		goto out;
	}

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq_byname_optional(dwc3_pdev, "dwc_usb3");
	if (irq > 0) {
		dwc3_host_fill_xhci_irq_res(dwc, irq, "dwc_usb3");
		goto out;
	}

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq(dwc3_pdev, 0);
	if (irq > 0)
		dwc3_host_fill_xhci_irq_res(dwc, irq, NULL);

out:
	return irq;
}

int dwc3_host_init(struct dwc3 *dwc)
{
	struct property_entry	props[7];
	struct platform_device	*xhci;
	int			ret, irq;
	int			prop_idx = 0;
	struct platform_device	*dwc3_pdev = to_platform_device(dwc->dev);

	/*
	 * Some platforms need to power off all Root hub ports immediately after DWC3 set to host
	 * mode to avoid VBUS glitch happen when xhci get reset later.
	 */
	dwc3_power_off_all_roothub_ports(dwc);

	irq = dwc3_host_get_irq(dwc);
	if (irq < 0)
		return irq;

	xhci = platform_device_alloc("xhci-hcd", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(dwc->dev, "couldn't allocate xHCI device\n");
		return -ENOMEM;
	}

	xhci->dev.parent	= dwc->dev;

	dwc->xhci = xhci;

	ret = platform_device_add_resources(xhci, dwc->xhci_resources,
						DWC3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(dwc->dev, "couldn't add resources to xHCI device\n");
		goto err;
	}

	memset(props, 0, sizeof(struct property_entry) * ARRAY_SIZE(props));

	props[prop_idx++] = PROPERTY_ENTRY_BOOL("xhci-sg-trb-cache-size-quirk");

	props[prop_idx++] = PROPERTY_ENTRY_BOOL("write-64-hi-lo-quirk");

	if (dwc->usb3_lpm_capable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb3-lpm-capable");

	if (dwc->usb2_lpm_disable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb2-lpm-disable");

	if (device_property_read_bool(&dwc3_pdev->dev,
				      "snps,xhci-reset-on-resume"))
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("xhci-reset-on-resume");

	/**
	 * WORKAROUND: dwc3 revisions <=3.00a have a limitation
	 * where Port Disable command doesn't work.
	 *
	 * The suggested workaround is that we avoid Port Disable
	 * completely.
	 *
	 * This following flag tells XHCI to do just that.
	 */
	if (DWC3_VER_IS_WITHIN(DWC3, ANY, 300A))
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("quirk-broken-port-ped");

	if (prop_idx) {
		ret = device_create_managed_software_node(&xhci->dev, props, NULL);
		if (ret) {
			dev_err(dwc->dev, "failed to add properties to xHCI\n");
			goto err;
		}
	}

	ret = platform_device_add_data(xhci, &dwc3_xhci_plat_quirk,
				       sizeof(struct xhci_plat_priv));
	if (ret)
		goto err;

	phy_create_lookup(dwc->usb2_generic_phy[0], "usb2-phy",
			  dev_name(dwc->dev));
	phy_create_lookup(dwc->usb3_generic_phy[0], "usb3-phy",
			  dev_name(dwc->dev));

	if (dwc->dr_mode == USB_DR_MODE_OTG) {
		struct usb_phy *phy = usb_get_phy(USB_PHY_TYPE_USB3);

		if (!IS_ERR(phy)) {
			if (phy && phy->otg)
				otg_set_host(phy->otg,
					     (struct usb_bus *)0xdeadbeef);

			usb_put_phy(phy);
		}
	}

	ret = platform_device_add(xhci);
	if (ret) {
		dev_err(dwc->dev, "failed to register xHCI device\n");
		goto err;
	}

	if (dwc->sys_wakeup) {
		/* Restore wakeup setting if switched from device */
		device_wakeup_enable(dwc->sysdev);

		/* Pass on wakeup setting to the new xhci platform device */
		device_init_wakeup(&xhci->dev, true);
	}

	return 0;
err:
	platform_device_put(xhci);
	return ret;
}

void dwc3_host_exit(struct dwc3 *dwc)
{
	if (dwc->sys_wakeup)
		device_init_wakeup(&dwc->xhci->dev, false);

	dwc3_enable_susphy(dwc, false);
	platform_device_unregister(dwc->xhci);
	dwc->xhci = NULL;
}
