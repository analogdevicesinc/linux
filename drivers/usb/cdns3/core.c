/**
 * core.c - Cadence USB3 DRD Controller Core file
 *
 * Copyright 2017 NXP
 *
 * Authors: Peter Chen <peter.chen@nxp.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/usb/of.h>
#include <linux/usb/phy.h>
#include <linux/extcon.h>

#include "cdns3-nxp-reg-def.h"
#include "core.h"
#include "host-export.h"
#include "gadget-export.h"

static void cdns3_usb_phy_init(void __iomem *regs)
{
	pr_debug("begin of %s\n", __func__);

	writel(0x0830, regs + PHY_PMA_CMN_CTRL1);
	writel(0x10, regs + TB_ADDR_CMN_DIAG_HSCLK_SEL);
	writel(0x00F0, regs + TB_ADDR_CMN_PLL0_VCOCAL_INIT_TMR);
	writel(0x0018, regs + TB_ADDR_CMN_PLL0_VCOCAL_ITER_TMR);
	writel(0x00D0, regs + TB_ADDR_CMN_PLL0_INTDIV);
	writel(0x4aaa, regs + TB_ADDR_CMN_PLL0_FRACDIV);
	writel(0x0034, regs + TB_ADDR_CMN_PLL0_HIGH_THR);
	writel(0x1ee, regs + TB_ADDR_CMN_PLL0_SS_CTRL1);
	writel(0x7F03, regs + TB_ADDR_CMN_PLL0_SS_CTRL2);
	writel(0x0020, regs + TB_ADDR_CMN_PLL0_DSM_DIAG);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_OVRD);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_FBH_OVRD);
	writel(0x0000, regs + TB_ADDR_CMN_DIAG_PLL0_FBL_OVRD);
	writel(0x0007, regs + TB_ADDR_CMN_DIAG_PLL0_V2I_TUNE);
	writel(0x0027, regs + TB_ADDR_CMN_DIAG_PLL0_CP_TUNE);
	writel(0x0008, regs + TB_ADDR_CMN_DIAG_PLL0_LF_PROG);
	writel(0x0022, regs + TB_ADDR_CMN_DIAG_PLL0_TEST_MODE);
	writel(0x000a, regs + TB_ADDR_CMN_PSM_CLK_CTRL);
	writel(0x139, regs + TB_ADDR_XCVR_DIAG_RX_LANE_CAL_RST_TMR);
	writel(0xbefc, regs + TB_ADDR_XCVR_PSM_RCTRL);

	writel(0x7799, regs + TB_ADDR_TX_PSC_A0);
	writel(0x7798, regs + TB_ADDR_TX_PSC_A1);
	writel(0x509b, regs + TB_ADDR_TX_PSC_A2);
	writel(0x3, regs + TB_ADDR_TX_DIAG_ECTRL_OVRD);
	writel(0x5098, regs + TB_ADDR_TX_PSC_A3);
	writel(0x2090, regs + TB_ADDR_TX_PSC_CAL);
	writel(0x2090, regs + TB_ADDR_TX_PSC_RDY);

	writel(0xA6FD, regs + TB_ADDR_RX_PSC_A0);
	writel(0xA6FD, regs + TB_ADDR_RX_PSC_A1);
	writel(0xA410, regs + TB_ADDR_RX_PSC_A2);
	writel(0x2410, regs + TB_ADDR_RX_PSC_A3);

	writel(0x23FF, regs + TB_ADDR_RX_PSC_CAL);
	writel(0x2010, regs + TB_ADDR_RX_PSC_RDY);

	writel(0x0020, regs + TB_ADDR_TX_TXCC_MGNLS_MULT_000);
	writel(0x00ff, regs + TB_ADDR_TX_DIAG_BGREF_PREDRV_DELAY);
	writel(0x0002, regs + TB_ADDR_RX_SLC_CU_ITER_TMR);
	writel(0x0013, regs + TB_ADDR_RX_SIGDET_HL_FILT_TMR);
	writel(0x0000, regs + TB_ADDR_RX_SAMP_DAC_CTRL);
	writel(0x1004, regs + TB_ADDR_RX_DIAG_SIGDET_TUNE);
	writel(0x4041, regs + TB_ADDR_RX_DIAG_LFPSDET_TUNE2);
	writel(0x0480, regs + TB_ADDR_RX_DIAG_BS_TM);
	writel(0x8006, regs + TB_ADDR_RX_DIAG_DFE_CTRL1);
	writel(0x003f, regs + TB_ADDR_RX_DIAG_ILL_IQE_TRIM4);
	writel(0x543f, regs + TB_ADDR_RX_DIAG_ILL_E_TRIM0);
	writel(0x543f, regs + TB_ADDR_RX_DIAG_ILL_IQ_TRIM0);
	writel(0x0000, regs + TB_ADDR_RX_DIAG_ILL_IQE_TRIM6);
	writel(0x8000, regs + TB_ADDR_RX_DIAG_RXFE_TM3);
	writel(0x0003, regs + TB_ADDR_RX_DIAG_RXFE_TM4);
	writel(0x2408, regs + TB_ADDR_RX_DIAG_LFPSDET_TUNE);
	writel(0x05ca, regs + TB_ADDR_RX_DIAG_DFE_CTRL3);
	writel(0x0258, regs + TB_ADDR_RX_DIAG_SC2C_DELAY);
	writel(0x1fff, regs + TB_ADDR_RX_REE_VGA_GAIN_NODFE);

	writel(0x02c6, regs + TB_ADDR_XCVR_PSM_CAL_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A0BYP_TMR);
	writel(0x02c6, regs + TB_ADDR_XCVR_PSM_A0IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A1IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A2IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A3IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A4IN_TMR);
	writel(0x0010, regs + TB_ADDR_XCVR_PSM_A5IN_TMR);

	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A0OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A1OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A2OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A3OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A4OUT_TMR);
	writel(0x0002, regs + TB_ADDR_XCVR_PSM_A5OUT_TMR);

	/* Change rx detect parameter */
	writel(0x960, regs + TB_ADDR_TX_RCVDET_EN_TMR);
	writel(0x01e0, regs + TB_ADDR_TX_RCVDET_ST_TMR);
	writel(0x0090, regs + TB_ADDR_XCVR_DIAG_LANE_FCM_EN_MGN_TMR);

	/* Force B Session Valid as 1 */
	writel(0x0060, regs + 0x380a4);

	udelay(10);

	pr_debug("end of %s\n", __func__);
}

static void cdns_set_role(struct cdns3 *cdns, enum cdns3_roles role)
{
	u32 value;
	int timeout_us = 100000;

	/* Wait clk value */
	value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
	writel(value, cdns->none_core_regs + USB3_SSPHY_STATUS);
	udelay(1);
	value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
	while ((value & 0xf0000000) != 0xf0000000 && timeout_us-- > 0) {
		value = readl(cdns->none_core_regs + USB3_SSPHY_STATUS);
		dev_dbg(cdns->dev, "clkvld:0x%x\n", value);
		udelay(1);
	}

	if (timeout_us <= 0)
		dev_err(cdns->dev, "wait clkvld timeout\n");

	/* Set all Reset bits */
	value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
	value |= ALL_SW_RESET;
	writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
	udelay(1);

	if (role == CDNS3_ROLE_HOST) {
		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value = (value & ~MODE_STRAP_MASK) | HOST_MODE | OC_DISABLE;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~PHYAHB_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		mdelay(1);
		cdns3_usb_phy_init(cdns->phy_regs);
		mdelay(1);

		value = readl(cdns->none_core_regs + USB3_INT_REG);
		value |= HOST_INT1_EN;
		writel(value, cdns->none_core_regs + USB3_INT_REG);

		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~ALL_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		dev_dbg(cdns->dev, "wait xhci_power_on_ready\n");

		value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while (!(value & HOST_POWER_ON_READY) && timeout_us-- > 0) {
			value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev, "wait xhci_power_on_ready timeout\n");

		mdelay(1);

		dev_dbg(cdns->dev, "switch to host role successfully\n");
	} else { /* gadget mode */
		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value = (value & ~MODE_STRAP_MASK) | DEV_MODE;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~PHYAHB_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		cdns3_usb_phy_init(cdns->phy_regs);
		value = readl(cdns->none_core_regs + USB3_INT_REG);
		value |= DEV_INT_EN;
		writel(value, cdns->none_core_regs + USB3_INT_REG);

		value = readl(cdns->none_core_regs + USB3_CORE_CTRL1);
		value &= ~ALL_SW_RESET;
		writel(value, cdns->none_core_regs + USB3_CORE_CTRL1);

		dev_dbg(cdns->dev, "wait gadget_power_on_ready\n");

		value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
		timeout_us = 100000;
		while (!(value & DEV_POWER_ON_READY) && timeout_us-- > 0) {
			value = readl(cdns->none_core_regs + USB3_CORE_STATUS);
			udelay(1);
		}

		if (timeout_us <= 0)
			dev_err(cdns->dev,
				"wait gadget_power_on_ready timeout\n");

		mdelay(1);

		dev_dbg(cdns->dev, "switch to gadget role successfully\n");
	}
}

static enum cdns3_roles cdns3_get_role(struct cdns3 *cdns)
{
	if (cdns->roles[CDNS3_ROLE_HOST] && cdns->roles[CDNS3_ROLE_GADGET]) {
		if (extcon_get_state(cdns->extcon, EXTCON_USB_HOST))
			return CDNS3_ROLE_HOST;
		else
			return CDNS3_ROLE_GADGET;
	} else {
		return cdns->roles[CDNS3_ROLE_HOST]
			? CDNS3_ROLE_HOST
			: CDNS3_ROLE_GADGET;
	}
}

/**
 * cdns3_core_init_role - initialize role of operation
 * @cdns: Pointer to cdns3 structure
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_core_init_role(struct cdns3 *cdns)
{
	struct device *dev = cdns->dev;
	enum usb_dr_mode dr_mode = usb_get_dr_mode(dev);

	cdns->role = CDNS3_ROLE_GADGET;
	if (dr_mode == USB_DR_MODE_UNKNOWN)
		dr_mode = USB_DR_MODE_OTG;

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_HOST) {
		if (cdns3_host_init(cdns))
			dev_info(dev, "doesn't support host\n");
	}

	if (dr_mode == USB_DR_MODE_OTG || dr_mode == USB_DR_MODE_PERIPHERAL) {
		if (cdns3_gadget_init(cdns))
			dev_info(dev, "doesn't support gadget\n");
	}

	if (!cdns->roles[CDNS3_ROLE_HOST] && !cdns->roles[CDNS3_ROLE_GADGET]) {
		dev_err(dev, "no supported roles\n");
		return -ENODEV;
	}

	return 0;
}

/**
 * cdns3_irq - interrupt handler for cdns3 core device
 *
 * @irq: irq number for cdns3 core device
 * @data: structure of cdns3
 *
 * Returns IRQ_HANDLED or IRQ_NONE
 */
static irqreturn_t cdns3_irq(int irq, void *data)
{
	struct cdns3 *cdns = data;

	/* Handle device/host interrupt */
	return cdns3_role(cdns)->irq(cdns);
}

static int cdns3_get_clks(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int ret = 0;

	cdns->cdns3_clks[0] = devm_clk_get(dev, "usb3_lpm_clk");
	if (IS_ERR(cdns->cdns3_clks[0])) {
		ret = PTR_ERR(cdns->cdns3_clks[0]);
		dev_err(dev, "Failed to get usb3_lpm_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[1] = devm_clk_get(dev, "usb3_bus_clk");
	if (IS_ERR(cdns->cdns3_clks[1])) {
		ret = PTR_ERR(cdns->cdns3_clks[1]);
		dev_err(dev, "Failed to get usb3_bus_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[2] = devm_clk_get(dev, "usb3_aclk");
	if (IS_ERR(cdns->cdns3_clks[2])) {
		ret = PTR_ERR(cdns->cdns3_clks[2]);
		dev_err(dev, "Failed to get usb3_aclk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[3] = devm_clk_get(dev, "usb3_ipg_clk");
	if (IS_ERR(cdns->cdns3_clks[3])) {
		ret = PTR_ERR(cdns->cdns3_clks[3]);
		dev_err(dev, "Failed to get usb3_ipg_clk, err=%d\n", ret);
		return ret;
	}

	cdns->cdns3_clks[4] = devm_clk_get(dev, "usb3_core_pclk");
	if (IS_ERR(cdns->cdns3_clks[4])) {
		ret = PTR_ERR(cdns->cdns3_clks[4]);
		dev_err(dev, "Failed to get usb3_core_pclk, err=%d\n", ret);
		return ret;
	}

	return 0;
}

static int cdns3_prepare_enable_clks(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int i, j, ret = 0;

	for (i = 0; i < CDNS3_NUM_OF_CLKS; i++) {
		ret = clk_prepare_enable(cdns->cdns3_clks[i]);
		if (ret) {
			dev_err(dev,
				"Failed to prepare/enable cdns3 clk, err=%d\n",
				ret);
			goto err;
		}
	}

	return ret;
err:
	for (j = i; j > 0; j--)
		clk_disable_unprepare(cdns->cdns3_clks[j - 1]);

	return ret;
}

static void cdns3_disable_unprepare_clks(struct device *dev)
{
	struct cdns3 *cdns = dev_get_drvdata(dev);
	int i;

	for (i = CDNS3_NUM_OF_CLKS - 1; i >= 0; i--)
		clk_disable_unprepare(cdns->cdns3_clks[i]);
}

static void cdns3_remove_roles(struct cdns3 *cdns)
{
	cdns3_gadget_remove(cdns);
	cdns3_host_remove(cdns);
}

static int cdsn3_do_role_switch(struct cdns3 *cdns, enum cdns3_roles role)
{
	int ret = 0;
	enum cdns3_roles current_role;

	dev_dbg(cdns->dev, "current role is %d, switch to %d\n",
			cdns->role, role);

	if (cdns->role == role)
		return 0;

	current_role = cdns->role;
	cdns3_role_stop(cdns);
	cdns_set_role(cdns, role);
	ret = cdns3_role_start(cdns, role);
	if (ret) {
		/* Back to current role */
		dev_err(cdns->dev, "set %d has failed, back to %d\n",
					role, current_role);
		cdns_set_role(cdns, current_role);
		ret = cdns3_role_start(cdns, current_role);
	}

	return ret;
}

/**
 * cdns3_role_switch - work queue handler for role switch
 *
 * @work: work queue item structure
 *
 */
static void cdns3_role_switch(struct work_struct *work)
{
	struct cdns3 *cdns = container_of(work, struct cdns3,
			role_switch_wq);
	bool host;

	host = extcon_get_state(cdns->extcon, EXTCON_USB_HOST);

	if (host)
		cdsn3_do_role_switch(cdns, CDNS3_ROLE_HOST);
	else
		cdsn3_do_role_switch(cdns, CDNS3_ROLE_GADGET);
}

static int cdns3_extcon_notifier(struct notifier_block *nb, unsigned long event,
			     void *ptr)
{
	struct cdns3 *cdns = container_of(nb, struct cdns3, extcon_nb);

	queue_work(system_power_efficient_wq, &cdns->role_switch_wq);

	return NOTIFY_DONE;
}

static int cdns3_register_extcon(struct cdns3 *cdns)
{
	struct extcon_dev *extcon;
	struct device *dev = cdns->dev;
	int ret;

	if (of_property_read_bool(dev->of_node, "extcon")) {
		extcon = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(extcon))
			return PTR_ERR(extcon);

		ret = devm_extcon_register_notifier(dev, extcon,
			EXTCON_USB_HOST, &cdns->extcon_nb);
		if (ret < 0) {
			dev_err(dev, "register Host Connector failed\n");
			return ret;
		}

		cdns->extcon = extcon;
		cdns->extcon_nb.notifier_call = cdns3_extcon_notifier;
	}

	return 0;
}

/**
 * cdns3_probe - probe for cdns3 core device
 * @pdev: Pointer to cdns3 core platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource	*res;
	struct cdns3 *cdns;
	void __iomem *regs;
	int ret;

	cdns = devm_kzalloc(dev, sizeof(*cdns), GFP_KERNEL);
	if (!cdns)
		return -ENOMEM;

	cdns->dev = dev;
	platform_set_drvdata(pdev, cdns);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	cdns->irq = res->start;

	/*
	 * Request memory region
	 * region-0: nxp wrap registers
	 * region-1: xHCI
	 * region-2: Peripheral
	 * region-3: PHY registers
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->none_core_regs = regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->xhci_regs = regs;
	cdns->xhci_res = res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->dev_regs	= regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	cdns->phy_regs = regs;

	ret = cdns3_get_clks(dev);
	if (ret)
		return ret;

	ret = cdns3_prepare_enable_clks(dev);
	if (ret)
		return ret;

	cdns->usbphy = devm_usb_get_phy_by_phandle(dev, "cdns3,usbphy", 0);
	if (IS_ERR(cdns->usbphy)) {
		ret = PTR_ERR(cdns->usbphy);
		if (ret == -ENODEV)
			ret = -EINVAL;
		goto err1;
	}

	ret = usb_phy_init(cdns->usbphy);
	if (ret)
		goto err1;

	INIT_WORK(&cdns->role_switch_wq, cdns3_role_switch);
	ret = cdns3_register_extcon(cdns);
	if (ret)
		goto err2;

	ret = cdns3_core_init_role(cdns);
	if (ret)
		goto err2;

	cdns->role = cdns3_get_role(cdns);
	dev_dbg(dev, "the init role is %d\n", cdns->role);
	cdns_set_role(cdns, cdns->role);
	ret = cdns3_role_start(cdns, cdns->role);
	if (ret) {
		dev_err(dev, "can't start %s role\n",
					cdns3_role(cdns)->name);
		goto err3;
	}

	ret = devm_request_irq(dev, cdns->irq, cdns3_irq, IRQF_SHARED,
			dev_name(dev), cdns);
	if (ret)
		goto err4;

	dev_dbg(dev, "Cadence USB3 core: probe succeed\n");

	return 0;

err4:
	cdns3_role_stop(cdns);
err3:
	cdns3_remove_roles(cdns);
err2:
	usb_phy_shutdown(cdns->usbphy);
err1:
	cdns3_disable_unprepare_clks(dev);
	return ret;
}

/**
 * cdns3_remove - unbind our drd driver and clean up
 * @pdev: Pointer to Linux platform device
 *
 * Returns 0 on success otherwise negative errno
 */
static int cdns3_remove(struct platform_device *pdev)
{
	struct cdns3 *cdns = platform_get_drvdata(pdev);

	cdns3_remove_roles(cdns);
	usb_phy_shutdown(cdns->usbphy);
	cdns3_disable_unprepare_clks(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_cdns3_match[] = {
	{ .compatible = "Cadence,usb3" },
	{ },
};
MODULE_DEVICE_TABLE(of, of_cdns3_match);
#endif

static struct platform_driver cdns3_driver = {
	.probe		= cdns3_probe,
	.remove		= cdns3_remove,
	.driver		= {
		.name	= "cdns-usb3",
		.of_match_table	= of_match_ptr(of_cdns3_match),
	},
};

static int __init cdns3_driver_platform_register(void)
{
	cdns3_host_driver_init();
	return platform_driver_register(&cdns3_driver);
}
module_init(cdns3_driver_platform_register);

static void __exit cdns3_driver_platform_unregister(void)
{
	platform_driver_unregister(&cdns3_driver);
}
module_exit(cdns3_driver_platform_unregister);

MODULE_ALIAS("platform:cdns3");
MODULE_AUTHOR("Peter Chen <peter.chen@nxp.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Cadence USB3 DRD Controller Driver");
