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
#include "API_AFE_t28hpc_hdmitx.h"

#define DISPDRV_HDMI		"hdmi_disp"

struct pll_divider {
	unsigned int cm;	/* multiplier */
	unsigned int cn;	/* predivider */
	unsigned int co;	/* outdivider */
};

/* driver private data */
struct imx_hdmi_info {
	struct platform_device *pdev;
	void __iomem *regs_base;
	void __iomem *ss_base;
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

	state_struct state;
	struct hdp_rw_func *rw;
};

static int mx8mq_hdp_read(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = mem->regs_base + addr;
	temp = __raw_readl((volatile unsigned int *)tmp_addr);
	*value = temp;
	return 0;
}

static int mx8mq_hdp_write(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = mem->regs_base + addr;

	__raw_writel(value, (volatile unsigned int *)tmp_addr);
	return 0;
}

static int mx8mq_hdp_sread(struct hdp_mem *mem, unsigned int addr, unsigned int *value)
{
	unsigned int temp;
	void *tmp_addr = mem->ss_base + addr;
	temp = __raw_readl((volatile unsigned int *)tmp_addr);
	*value = temp;
	return 0;
}

static int mx8mq_hdp_swrite(struct hdp_mem *mem, unsigned int addr, unsigned int value)
{
	void *tmp_addr = mem->ss_base + addr;
	__raw_writel(value, (volatile unsigned int *)tmp_addr);
	return 0;
}

static struct hdp_rw_func imx8mq_rw = {
	.read_reg = mx8mq_hdp_read,
	.write_reg = mx8mq_hdp_write,
	.sread_reg = mx8mq_hdp_sread,
	.swrite_reg = mx8mq_hdp_swrite,
};

static void imx_hdmi_state_init(struct imx_hdmi_info *imx_hdmi)
{
	state_struct *state = &imx_hdmi->state;

	memset(state, 0, sizeof(state_struct));
	mutex_init(&state->mutex);

	state->mem.regs_base = imx_hdmi->regs_base;
	state->mem.ss_base = imx_hdmi->ss_base;
	state->rw = imx_hdmi->rw;

	state->rw->read_reg = mx8mq_hdp_read;
	state->rw->write_reg = mx8mq_hdp_write;
	state->rw->sread_reg = mx8mq_hdp_sread;
	state->rw->swrite_reg = mx8mq_hdp_swrite;
}

static int hdmi_init(struct imx_hdmi_info *imx_hdmi, int vic, int encoding, int color_depth)
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

	ret = CDN_API_CheckAlive_blocking(&imx_hdmi->state);
	printk("CDN_API_CheckAlive returned ret = %d\n", ret);
	if (ret != 0) {
		printk("NO HDMI FW running\n");
		return -EINVAL;
	}

	ret = CDN_API_General_Test_Echo_Ext_blocking(&imx_hdmi->state, echo_msg, echo_resp,
						     sizeof(echo_msg),
						     CDN_BUS_TYPE_APB);
	printk
	    ("CDN_API_General_Test_Echo_Ext_blocking - APB(ret = %d echo_resp = %s)\n",
	     ret, echo_resp);
	if (ret != 0) {
		printk("HDMI mailbox access failed\n");
		return -EINVAL;
	}

	/* Configure PHY */
	character_freq_khz =
	    phy_cfg_hdp_t28hpc(&imx_hdmi->state, 4, vic_mode, bps, format, pixel_clk_from_phy);

	hdmi_tx_power_configuration_seq(&imx_hdmi->state, 4);

	/* Set the lane swapping */
	ret =
	    CDN_API_General_Write_Register_blocking(&imx_hdmi->state, ADDR_SOURCD_PHY +
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

	ret = CDN_API_HDMITX_Init_blocking(&imx_hdmi->state);
	printk("CDN_API_STATUS CDN_API_HDMITX_Init_blocking  ret = %d\n", ret);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(&imx_hdmi->state, ptype, character_freq_khz);
	printk("CDN_API_HDMITX_Set_Mode_blocking ret = %d\n", ret);

	ret = CDN_API_Set_AVI(&imx_hdmi->state, vic_mode, format, bw_type);
	printk("CDN_API_Set_AVI  ret = %d\n", ret);

	ret = CDN_API_HDMITX_SetVic_blocking(&imx_hdmi->state, vic_mode, bps, format);
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


	/* Get HDMI SEC base register */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform mem resource\n");
		return -ENOMEM;
	}

	imx_hdmi->ss_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(imx_hdmi->ss_base))
		return -ENODEV;

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

	imx_hdmi->rw = &imx8mq_rw;

	imx_hdmi_state_init(imx_hdmi);

//	CDN_API_Init(&imx_hdmi->state);
	/* 0-480p, 1-720p, 2-1080p, 3-2160p60, 4-2160p30 */
	/* Pixel Format - 1 RGB, 2 YCbCr 444, 3 YCbCr 420 */
	if (vmode_index == 16)
		ret = hdmi_init(imx_hdmi, 2, 1, 8);
	else if (vmode_index == 97)
		ret = hdmi_init(imx_hdmi, 3, 1, 8);
	else if (vmode_index == 95)
		ret = hdmi_init(imx_hdmi, 4, 1, 8);
	else
		ret = hdmi_init(imx_hdmi, 1, 1, 8);

	if (ret < 0)
		return -EINVAL;

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
	{.compatible = "fsl,imx8mq-fb-hdmi", .data = NULL,},
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
