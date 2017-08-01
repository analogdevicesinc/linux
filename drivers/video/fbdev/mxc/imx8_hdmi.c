/*
 * Copyright 2017 NXP.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <video/mxc_edid.h>
#include <linux/mfd/syscon.h>

#include "mxc_dispdrv.h"
#include "./cdn_hdp/address.h"
#include "./cdn_hdp/apb_cfg.h"
#include "./cdn_hdp/API_General.h"
#include "./cdn_hdp/API_HDMITX.h"
#include "./cdn_hdp/API_AVI.h"
#include "./cdn_hdp/API_AFE.h"
#include "./cdn_hdp/API_AFE_t28hpc_hdmitx.h"
#include "./cdn_hdp/externs.h"
#include "./cdn_hdp/source_car.h"
#include "./cdn_hdp/source_phy.h"
#include "./cdn_hdp/source_vif.h"
#include "./cdn_hdp/vic_table.h"

#define DISPDRV_HDMI		"hdmi_disp"

void __iomem *g_sec_base;
void __iomem *g_regs_base;

struct pll_divider {
	unsigned int cm;	/* multiplier */
	unsigned int cn;	/* predivider */
	unsigned int co;	/* outdivider */
};

/* driver private data */
struct imx_hdmi_info {
	struct platform_device *pdev;
	void __iomem *regs_base;
	void __iomem *sec_base;
	void __iomem *phy_base;
	int power_on;
	u32 dphy_pll_config;
	int dev_id;
	int disp_id;
	int irq;
	int vic;
	struct clk *core_clk;
	struct clk *phy_ref_clk;
	struct mxc_dispdrv_handle *disp_hdmi;
	struct fb_videomode *mode;
	struct regulator *disp_power_on;
};

static int hdmi_init(int vic, int encoding, int color_depth)
{
	int ret;
	uint32_t character_freq_khz;

	bool pixel_clk_from_phy = 0;
	uint8_t echo_msg[] = "echo test";
	uint8_t echo_resp[sizeof(echo_msg) + 1];
	/* Parameterization: */

	/* VIC Mode - index from vic_table (see vic_table.c) */
	VIC_MODES vic_mode = vic;

	/* Pixel Format - 1 RGB, 2 YCbCr 444, 3 YCbCr 420 */
	VIC_PXL_ENCODING_FORMAT format = encoding;

	/*  B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bw_type = 0;

	/* bpp (bits per subpixel) - 8 24bpp, 10 30bpp, 12 36bpp, 16 48bpp */
	uint8_t bps = color_depth;

	/* Set HDMI TX Mode */
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	/* 0- pixel clock from phy */
	pixel_clk_from_phy = 1;

	if (vic_mode == VIC_MODE_97_60Hz)
		ptype = 2;

	/* Parameterization done */

	CDN_API_Init();
	printk("CDN_API_Init completed\n");

	ret = CDN_API_CheckAlive_blocking();
	printk("CDN_API_CheckAlive returned ret = %d\n", ret);

	ret = CDN_API_General_Test_Echo_Ext_blocking(echo_msg, echo_resp,
						     sizeof(echo_msg),
						     CDN_BUS_TYPE_APB);
	printk
	    ("CDN_API_General_Test_Echo_Ext_blocking - APB(ret = %d echo_resp = %s)\n",
	     ret, echo_resp);
	/* Configure PHY */
	character_freq_khz =
	    phy_cfg_hdp(4, vic_mode, bps, format, pixel_clk_from_phy);

	hdmi_tx_kiran_power_configuration_seq(4);

	/* Set the lane swapping */
	ret =
	    CDN_API_General_Write_Register_blocking(ADDR_SOURCD_PHY +
						    (LANES_CONFIG << 2),
						    F_SOURCE_PHY_LANE0_SWAP(0) |
						    F_SOURCE_PHY_LANE1_SWAP(1) |
						    F_SOURCE_PHY_LANE2_SWAP(2) |
						    F_SOURCE_PHY_LANE3_SWAP(3) |
						    F_SOURCE_PHY_COMB_BYPASS(0)
						    | F_SOURCE_PHY_20_10(1));
	printk
	    ("CDN_API_General_Write_Register_blocking LANES_CONFIG ret = %d\n",
	     ret);

	ret = CDN_API_HDMITX_Init_blocking();
	printk("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(ptype, character_freq_khz);
	printk("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);

	ret = CDN_API_Set_AVI(vic_mode, format, bw_type);
	printk("CDN_API_Set_AVI  ret = %d\n", ret);

	ret = CDN_API_HDMITX_SetVic_blocking(vic_mode, bps, format);
	printk("CDN_API_HDMITX_SetVic_blocking ret = %d\n", ret);

	udelay(10000);

	return 0;
}

static int imx_hdmi_disp_init(struct mxc_dispdrv_handle *disp,
			      struct mxc_dispdrv_setting *setting)
{
	return true;
}

static void imx_hdmi_disp_deinit(struct mxc_dispdrv_handle *disp)
{
}


static int imx_hdmi_enable(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	/* 0-480p, 1-720p, 2-1080p, 3-2160p */
	return 0;
}

static void imx_hdmi_disable(struct mxc_dispdrv_handle *disp,
			     struct fb_info *fbi)
{
}

static int imx_hdmi_setup(struct mxc_dispdrv_handle *disp, struct fb_info *fbi)
{
	return 0;
}

static struct mxc_dispdrv_driver imx_hdmi_drv = {
	.name = DISPDRV_HDMI,
	.init = imx_hdmi_disp_init,
	.deinit = imx_hdmi_disp_deinit,
	.enable = imx_hdmi_enable,
	.disable = imx_hdmi_disable,
	.setup = imx_hdmi_setup,
};

#if 0
static irqreturn_t imx_hdmi_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}
#endif

static int hdmi_clks_init(struct imx_hdmi_info *hinfo)
{
	return true;
}

/**
 * This function is called by the driver framework to initialize the HDMI
 * device.
 *
 * @param	pdev	The device structure for the HDMI passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int imx_hdmi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct imx_hdmi_info *imx_hdmi;
	struct resource *res;
	int ret = 0;
	u32 vmode_index;

	printk("%s\n", __func__);
	imx_hdmi = devm_kzalloc(&pdev->dev, sizeof(*imx_hdmi), GFP_KERNEL);
	if (!imx_hdmi)
		return -ENOMEM;
	imx_hdmi->pdev = pdev;

	/* Get HDMI CTRL base register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform mem resource\n");
		return -ENOMEM;
	}

	imx_hdmi->regs_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(imx_hdmi->regs_base))
		return -ENODEV;

	g_regs_base = imx_hdmi->regs_base;

	/* Get HDMI SEC base register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform mem resource\n");
		return -ENOMEM;
	}

	imx_hdmi->sec_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(imx_hdmi->sec_base))
		return -ENODEV;

	g_sec_base = imx_hdmi->sec_base;

	/* IRQ */
	imx_hdmi->irq = platform_get_irq(pdev, 0);
	if (imx_hdmi->irq < 0) {
		dev_err(&pdev->dev, "failed to get device irq\n");
		return -EINVAL;
	}
#if 0
	ret = devm_request_irq(&pdev->dev, imx_hdmi->irq,
			       imx_hdmi_irq_handler,
			       0, "imx_hdmi_cdn", imx_hdmi);
	if (ret) {
		dev_err(&pdev->dev, "failed to request hdmi irq\n");
		return ret;
	}
#endif

	hdmi_clks_init(imx_hdmi);

	ret = of_property_read_u32(np, "video-mode", &vmode_index);
	if (ret < 0)
		return -EINVAL;

	imx_hdmi->disp_hdmi = mxc_dispdrv_register(&imx_hdmi_drv);
	if (IS_ERR(imx_hdmi->disp_hdmi)) {
		dev_err(&pdev->dev, "mxc_dispdrv_register error\n");
		ret = PTR_ERR(imx_hdmi->disp_hdmi);
		goto dispdrv_reg_fail;
	}

	mxc_dispdrv_setdata(imx_hdmi->disp_hdmi, imx_hdmi);
	dev_set_drvdata(&pdev->dev, imx_hdmi);

	/* 0-480p, 1-720p, 2-1080p, 3-2160p60, 4-2160p30 */
	/* Pixel Format - 1 RGB, 2 YCbCr 444, 3 YCbCr 420 */
	if (vmode_index == 16)
		hdmi_init(2, 1, 8);
	else if (vmode_index == 97)
		hdmi_init(3, 1, 8);
	else if (vmode_index == 95)
		hdmi_init(4, 1, 8);
	else
		hdmi_init(1, 1, 8);
	dev_info(&pdev->dev, "i.MX HDMI driver probed\n");
	return ret;

dispdrv_reg_fail:
	return ret;
}

static int imx_hdmi_remove(struct platform_device *pdev)
{
	struct imx_hdmi_info *imx_hdmi = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(imx_hdmi->disp_hdmi);
	mxc_dispdrv_unregister(imx_hdmi->disp_hdmi);

	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static void imx_hdmi_shutdown(struct platform_device *pdev)
{
}


static const struct of_device_id imx_imx_hdmi_dt_ids[] = {
	{.compatible = "fsl,imx8mq-hdmi", .data = NULL,},
	{}
};

MODULE_DEVICE_TABLE(of, imx_imx_hdmi_dt_ids);

static struct platform_driver imx_hdmi_driver = {
	.driver = {
		   .of_match_table = imx_imx_hdmi_dt_ids,
		   .name = "imx_hdmi",
		   },
	.probe = imx_hdmi_probe,
	.remove = imx_hdmi_remove,
	.shutdown = imx_hdmi_shutdown,
};

static int __init imx_hdmi_init(void)
{
	int err;

	err = platform_driver_register(&imx_hdmi_driver);
	if (err) {
		pr_err("imx_hdmi_driver register failed\n");
		return err;
	}

	pr_debug("HDMI driver module loaded: %s\n",
		 imx_hdmi_driver.driver.name);

	return 0;
}

static void __exit imx_hdmi_exit(void)
{
	platform_driver_unregister(&imx_hdmi_driver);
}

module_init(imx_hdmi_init);
module_exit(imx_hdmi_exit);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX HDMI driver");
MODULE_LICENSE("GPL");
