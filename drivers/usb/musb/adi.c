// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MUSB "glue layer" driver for ADI Sc58x/Sc57x platforms
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written by Timesys Corporation
 * Maintained by Analog Devices, Inc.
 *
 * Contact: Analog Devices, Inc <adsp-linux@analog.com>
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/usb/of.h>
#include <linux/soc/adi/cpu.h>

#include "adi.h"
#include "musb_core.h"

#define TIMER_DELAY (1 * HZ)

struct adi_musb_glue {
	struct device *dev;
	struct platform_device *musb;
	struct platform_device *phy;
};

#define glue_to_musb(glue) platform_get_drvdata(glue->musb)

static void musb_conn_timer_handler(struct timer_list *t)
{
	struct musb *musb = from_timer(musb, t, dev_timer);
	unsigned long flags;
	u16 val;
	static u8 toggle;

	spin_lock_irqsave(&musb->lock, flags);

	switch (musb->xceiv->otg->state) {
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_WAIT_BCON:
		/* Start a new session */
		val = musb_readw(musb->mregs, MUSB_DEVCTL);
		val &= ~MUSB_DEVCTL_SESSION;
		musb_writew(musb->mregs, MUSB_DEVCTL, val);
		val |= MUSB_DEVCTL_SESSION;
		musb_writew(musb->mregs, MUSB_DEVCTL, val);
		/* Check if musb is host or peripheral. */
		val = musb_readw(musb->mregs, MUSB_DEVCTL);

		if (!(val & MUSB_DEVCTL_BDEVICE)) {
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
		} else {
			/* Ignore VBUSERROR and SUSPEND IRQ */
			val = musb_readb(musb->mregs, MUSB_INTRUSBE);
			val &= ~MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSBE, val);

			val = MUSB_INTR_SUSPEND | MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSB, val);
			musb->xceiv->otg->state = OTG_STATE_B_IDLE;
		}
		mod_timer(&musb->dev_timer, jiffies + TIMER_DELAY);
		break;
	case OTG_STATE_B_IDLE:
		/*
		 * Start a new session.  It seems that MUSB needs taking
		 * some time to recognize the type of the plug inserted?
		 */
		val = musb_readw(musb->mregs, MUSB_DEVCTL);
		val |= MUSB_DEVCTL_SESSION;
		musb_writew(musb->mregs, MUSB_DEVCTL, val);
		val = musb_readw(musb->mregs, MUSB_DEVCTL);

		if (!(val & MUSB_DEVCTL_BDEVICE)) {
			musb->xceiv->otg->state = OTG_STATE_A_WAIT_BCON;
		} else {
			/* Ignore VBUSERROR and SUSPEND IRQ */
			val = musb_readb(musb->mregs, MUSB_INTRUSBE);
			val &= ~MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSBE, val);

			val = MUSB_INTR_SUSPEND | MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSB, val);

			/* Toggle the Soft Conn bit, so that we can response to
			 * the inserting of either A-plug or B-plug.
			 */
			if (toggle) {
				val = musb_readb(musb->mregs, MUSB_POWER);
				val &= ~MUSB_POWER_SOFTCONN;
				musb_writeb(musb->mregs, MUSB_POWER, val);
				toggle = 0;
			} else {
				val = musb_readb(musb->mregs, MUSB_POWER);
				val |= MUSB_POWER_SOFTCONN;
				musb_writeb(musb->mregs, MUSB_POWER, val);
				toggle = 1;
			}
		}

		/* The delay time is set to 1/4 second by default,
		 * shortening it, if accelerating A-plug detection
		 * is needed in OTG mode.
		 */
		mod_timer(&musb->dev_timer, jiffies + TIMER_DELAY / 4);
		break;
	default:
		dev_dbg(musb->controller, "%s state not handled\n",
			usb_otg_state_string(musb->xceiv->otg->state));
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	dev_dbg(musb->controller, "state is %s\n",
		usb_otg_state_string(musb->xceiv->otg->state));
}

static irqreturn_t adi_musb_interrupt(int irq, void *__hci)
{
	u8 devctl;
	unsigned long flags;
	irqreturn_t retval = IRQ_NONE;
	struct musb *musb = __hci;
	struct device *dev = musb->controller;
	struct musb_hdrc_platform_data *plat = dev_get_platdata(dev);

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	if (musb->int_usb & MUSB_INTR_VBUSERROR) {
		//dev_warn(&parent->dev, "VBUS error recovery\n");
		musb->int_usb &= ~MUSB_INTR_VBUSERROR;
		devctl = musb_readw(musb->mregs, MUSB_DEVCTL);
		devctl |= MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);
	}
	if (musb->int_usb || musb->int_tx || musb->int_rx) {
		musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);
		musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);
		musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);
		retval = musb_interrupt(musb);
	}

	if (plat->mode == MUSB_OTG) {
		/* Start sampling ID pin, when plug is removed from MUSB */
		if ((musb->xceiv->otg->state == OTG_STATE_B_IDLE
			|| musb->xceiv->otg->state == OTG_STATE_A_WAIT_BCON) ||
			(musb->int_usb & MUSB_INTR_DISCONNECT && is_host_active(musb))) {
			mod_timer(&musb->dev_timer, jiffies + TIMER_DELAY);
			MUSB_DEV_MODE(musb);
			musb->a_wait_bcon = TIMER_DELAY;
		}
	}

	if (musb->int_usb & MUSB_INTR_DISCONNECT && is_host_active(musb))
		musb_writeb(musb->ctrl_base, REG_USB_VBUS_CTL, 0x0);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static void adi_musb_reg_init(struct musb *musb)
{
	musb_writel(musb->ctrl_base, REG_USB_PLL_OSC, 20 << 1);
	musb_writeb(musb->ctrl_base, REG_USB_VBUS_CTL, 0x0);
	musb_writeb(musb->ctrl_base, REG_USB_PHY_CTL, 0x80);
	musb_writel(musb->ctrl_base, REG_USB_UTMI_CTL,
		    0x40 | musb_readl(musb->ctrl_base, REG_USB_UTMI_CTL));
}

static int adi_musb_init(struct musb *musb)
{
	int spu_securep_id, ret = 0;
	struct device *dev = musb->controller;
	struct platform_device *parent = to_platform_device(dev->parent);
	struct device_node *node = parent->dev.of_node;

	/*initialize spu */

	ret = of_property_read_u32(node, "spu_securep_id", (u32 *)&spu_securep_id);
	if (ret) {
		dev_err(&parent->dev, "failed to get spu id\n");
		return -ENXIO;
	}

	set_spu_securep_msec(spu_securep_id, true);

	musb->xceiv = devm_usb_get_phy_by_phandle(dev->parent, "phys", 0);
	if (IS_ERR_OR_NULL(musb->xceiv)) {
		dev_err(&parent->dev, "failed to allocate musb->xceiv\n");
		return -EPROBE_DEFER;
	}

	musb->isr = adi_musb_interrupt;
	timer_setup(&musb->dev_timer,
		    musb_conn_timer_handler, 0);
	adi_musb_reg_init(musb);

	return 0;
}

static int adi_musb_exit(struct musb *musb)
{
	usb_put_phy(musb->xceiv);

	return 0;
}

static int adi_musb_set_mode(struct musb *musb, u8 musb_mode)
{
	struct device *dev = musb->controller;
	struct platform_device *parent = to_platform_device(dev->parent);

	switch (musb_mode) {
	case MUSB_HOST:
		musb_writeb(musb->ctrl_base, REG_USB_ID_CTL, 0x1);
		break;
	case MUSB_PERIPHERAL:
		musb_writeb(musb->ctrl_base, REG_USB_ID_CTL, 0x3);
		break;
	case MUSB_OTG:
		musb_writeb(musb->ctrl_base, REG_USB_ID_CTL, 0x0);
		break;
	default:
		dev_err(&parent->dev, "Trying to set unsupported mode %u\n",
				musb_mode);
		return -EINVAL;
	}

	return 0;
}

static const struct musb_platform_ops adi_musb_ops = {
	.quirks = MUSB_DMA_INVENTRA,
	.init = adi_musb_init,
	.exit = adi_musb_exit,
#ifdef CONFIG_USB_INVENTRA_DMA
	.dma_init = musbhs_dma_controller_create,
	.dma_exit = musbhs_dma_controller_destroy,
#endif
	.set_mode = adi_musb_set_mode,
};

static int adi_musb_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	struct resource musb_resources[3];
	struct musb_hdrc_platform_data pdata;
	struct platform_device *musb;
	struct adi_musb_glue *glue;
	struct musb_hdrc_config *config;
	int class = 0;

	char probe_fail_msg[128] =
		"Unknown error occurred during probe";

	int ret = -ENOMEM;
	int val = 0;

	/*allocate glue struct */
	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (IS_ERR(glue)) {
		ret = -ENOMEM;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to allocate memory for driver data");
		goto err0;
	}

	/*alloc musb platform_device and register */
	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (IS_ERR(musb)) {
		ret = -ENOMEM;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to allocate device");
		goto err1;
	}

	musb->dev.parent	= &pdev->dev;
	glue->dev		= &pdev->dev;
	glue->musb		= musb;

	config = devm_kzalloc(&pdev->dev, sizeof(*config), GFP_KERNEL);
	if (IS_ERR(config)) {
		ret = -ENOMEM;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to allocate memory for config");
		goto err2;
	}

	ret = of_property_read_u32(node, "mentor,num-eps", (u32 *)&config->num_eps);
	ret = of_property_read_u32(node, "mentor,multipoint", (u32 *)&val);
	if (val)
		config->multipoint = true;

	ret = of_property_read_u32(node, "mentor,ram-bits", (u32 *)&config->ram_bits);
	ret = of_property_read_u32(node, "mode", (u32 *)&class);

	if (class == MUSB_OTG) {
#if IS_ENABLED(CONFIG_USB_MUSB_HOST)
		class = MUSB_HOST;
#elif IS_ENABLED(CONFIG_USB_MUSB_GADGET)
		class = MUSB_PERIPHERAL;
#endif
	}
	pdata.mode = class;

	ret = of_property_read_u32(node, "mentor,power", (u32 *)&val);
	pdata.power = val / 2;
	pdata.config = config;
	pdata.platform_ops = &adi_musb_ops;

	glue->phy = usb_phy_generic_register();
	if (IS_ERR(glue->phy))
		goto err2;

	platform_set_drvdata(pdev, glue);

	memset(musb_resources, 0x00, sizeof(*musb_resources) *
	       ARRAY_SIZE(musb_resources));

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mc");
	if (IS_ERR(res)) {
		ret = -ENOMEM;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to get resource 'mc'");
		goto err3;
	}
	musb_resources[0] = *res;

	ret = platform_get_irq(pdev, 0);
	if (!ret) {
		ret = -EINVAL;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to get IRQ 0");
		goto err3;
	}
	musb_resources[1].start = ret;
	musb_resources[1].flags = IORESOURCE_IRQ;
	musb_resources[1].name  = "mc";

	ret = platform_get_irq(pdev, 1);
	if (!ret) {
		ret = -EINVAL;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to get IRQ 1");
		goto err3;
	}
	musb_resources[2].start = ret;
	musb_resources[2].flags = IORESOURCE_IRQ;
	musb_resources[2].name  = "dma";

	ret = platform_device_add_resources(musb, musb_resources, 3);
	if (ret) {
		ret = -EINVAL;
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to add platform reosurces");
		goto err3;
	}

	ret = platform_device_add_data(musb, &pdata, sizeof(pdata));
	if (ret) {
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to add platform data");
		goto err3;
	}

	ret = platform_device_add(musb);
	if (ret) {
		snprintf(probe_fail_msg, sizeof(probe_fail_msg),
				"failed to add platform device");
		goto err3;
	}

	return 0;
err3:
	usb_phy_generic_unregister(glue->phy);

err2:
	platform_device_put(musb);

err1:
	kfree(glue);

err0:
	return dev_err_probe(&pdev->dev, ret, probe_fail_msg);
}

static void adi_musb_remove(struct platform_device *pdev)
{
	struct adi_musb_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	usb_phy_generic_unregister(glue->phy);
	kfree(glue);
}

static const struct of_device_id adi_musb_match[] = {
	{.compatible = "adi,musb",},
	{},
};
MODULE_DEVICE_TABLE(of, adi_musb_match);

static struct platform_driver adi_musb_driver = {
	.probe = adi_musb_probe,
	.remove = adi_musb_remove,
	.driver = {
		   .name = "musb-adi",
		   .of_match_table = adi_musb_match,
		   },
};
module_platform_driver(adi_musb_driver);

MODULE_DESCRIPTION("ADI MUSB Glue Layer");
MODULE_AUTHOR("Hao Liang <hliang1025@gmail.com>");
MODULE_LICENSE("GPL v2");
