/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
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
#include <linux/mipi_dsi_northwest.h>
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
#include <video/mipi_display.h>
#include <video/mxc_edid.h>
#include <linux/mfd/syscon.h>

#include "mipi_dsi.h"

#define DISPDRV_MIPI			"mipi_dsi_northwest"
#define ROUND_UP(x)			((x)+1)
#define NS2PS_RATIO			(1000)
#define	MIPI_LCD_SLEEP_MODE_DELAY	(120)
#define MIPI_FIFO_TIMEOUT		msecs_to_jiffies(250)

static struct mipi_dsi_match_lcd mipi_dsi_lcd_db[] = {
#ifdef CONFIG_FB_MXC_TRULY_WVGA_SYNC_PANEL
	{
	 "TRULY-WVGA",
	 {mipid_hx8369_get_lcd_videomode, mipid_hx8369_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5079E
	{
	 "TRULY-WVGA-TFT3P5079E",
	 {mipid_otm8018b_get_lcd_videomode, mipid_otm8018b_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5581E
	{
	 "TRULY-WVGA-TFT3P5581E",
	 {mipid_hx8363_get_lcd_videomode, mipid_hx8363_lcd_setup}
	},
#endif
	{
	"", {NULL, NULL}
	}
};

enum mipi_dsi_mode {
	DSI_COMMAND_MODE,
	DSI_VIDEO_MODE
};

#define DSI_LP_MODE	0
#define DSI_HS_MODE	1

enum mipi_dsi_payload {
	DSI_PAYLOAD_CMD,
	DSI_PAYLOAD_VIDEO,
};

struct pll_divider {
	unsigned int cm;  /* multiplier */
	unsigned int cn;  /* predivider */
	unsigned int co;  /* outdivider */
};

static DECLARE_COMPLETION(dsi_rx_done);
static DECLARE_COMPLETION(dsi_tx_done);

static void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
			      uint8_t mode);
static int mipi_dsi_dcs_cmd(struct mipi_dsi_info *mipi_dsi,
			    u8 cmd, const u32 *param, int num);

static int mipi_dsi_lcd_init(struct mipi_dsi_info *mipi_dsi,
			     struct mxc_dispdrv_setting *setting)
{
	u32 data_lane_num, max_data_rate;
	int i, size, err = 0;
	struct fb_videomode *mipi_lcd_modedb;
	struct fb_videomode mode;
	struct device *dev = &mipi_dsi->pdev->dev;

	if (mipi_dsi->encoder) {
		err = of_property_read_u32(dev->of_node,
					   "data-lanes-num", &data_lane_num);
		if (err)
			goto err0;

		err = of_property_read_u32(dev->of_node,
					   "max-data-rate", &max_data_rate);
		if (err)
			goto err0;

		mipi_dsi->lcd_config->virtual_ch = 0;
		mipi_dsi->lcd_config->data_lane_num = data_lane_num;
		mipi_dsi->lcd_config->max_phy_clk = max_data_rate;
		mipi_dsi->lcd_config->dpi_fmt = MIPI_RGB888;
		setting->fbi->var.bits_per_pixel = 32;

		/* TODO Add bandwidth check */

		if (setting->fbi->fbops->fb_check_var)
			err = setting->fbi->fbops->fb_check_var(&setting->fbi->var,
								setting->fbi);
		if (err)
			goto err0;

		err = fb_add_videomode(mipi_dsi->mode,
				       &setting->fbi->modelist);
		if (err)
			goto err0;

		fb_videomode_to_var(&setting->fbi->var, mipi_dsi->mode);
		setting->fbi->mode = mipi_dsi->mode;
	err0:
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_lcd_db); i++) {
		if (!strcmp(mipi_dsi->lcd_panel,
			mipi_dsi_lcd_db[i].lcd_panel)) {
			mipi_dsi->lcd_callback =
				&mipi_dsi_lcd_db[i].lcd_callback;
			break;
		}
	}
	if (i == ARRAY_SIZE(mipi_dsi_lcd_db)) {
		dev_err(dev, "failed to find supported lcd panel.\n");
		return -EINVAL;
	}

	/* set default bpp to 32 if not set*/
	if (!setting->default_bpp)
		setting->default_bpp = 32;

	mipi_dsi->lcd_callback->get_mipi_lcd_videomode(&mipi_lcd_modedb, &size,
					&mipi_dsi->lcd_config);

	err = fb_find_mode(&setting->fbi->var, setting->fbi,
				setting->dft_mode_str,
				mipi_lcd_modedb, size, NULL,
				setting->default_bpp);
	if (err != 1)
		fb_videomode_to_var(&setting->fbi->var, mipi_lcd_modedb);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < size; i++) {
		fb_var_to_videomode(&mode, &setting->fbi->var);
		if (fb_mode_is_equal(&mode, mipi_lcd_modedb + i)) {
			err = fb_add_videomode(mipi_lcd_modedb + i,
					&setting->fbi->modelist);
			mipi_dsi->mode = mipi_lcd_modedb + i;
			break;
		}
	}

	if ((err < 0) || (size == i)) {
		dev_err(dev, "failed to add videomode.\n");
		return err;
	}

	setting->fbi->mode = mipi_dsi->mode;

	return 0;
}

static int mipi_dsi_disp_init(struct mxc_dispdrv_handle *disp,
			      struct mxc_dispdrv_setting *setting)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);
	struct device *dev = &mipi_dsi->pdev->dev;
	struct device_node *np = dev->of_node;
	struct reset_control *reset = NULL;
	int ret = 0;

	reset = of_reset_control_get(np, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	ret = mipi_dsi_lcd_init(mipi_dsi, setting);
	if (ret) {
		dev_err(dev, "failed to init mipi dsi lcd\n");
		goto out;
	}

	dev_info(dev, "MIPI DSI dispdv inited\n");

out:
	reset_control_put(reset);
	return ret;
}

static void mipi_dsi_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct mipi_dsi_info *mipi_dsi;

	mipi_dsi = mxc_dispdrv_getdata(disp);

	if (mipi_dsi->bl)
		backlight_device_unregister(mipi_dsi->bl);
}

static void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
			      uint8_t mode)
{
	uint32_t pkt_control;

	pkt_control = readl(mipi_dsi->mmio_base + HOST_PKT_CONTROL);

	switch (mode) {
	case DSI_LP_MODE:
		writel(0x1, mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
		break;
	case DSI_HS_MODE:
		writel(0x0, mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
		break;
	default:
		dev_err(&mipi_dsi->pdev->dev,
			"invalid dsi mode\n");
		return;
	}

	mdelay(1);
}

static uint32_t fmt_to_bpp(enum mipi_dsi_dpi_fmt dpi_fmt)
{
	switch (dpi_fmt) {
	case MIPI_RGB888:
		return 24;
	case MIPI_RGB565_PACKED:
		return 16;
	default:
		return 0;
	}
}

/*
static void dphy_calc_dividers(int *cm, int *cn, int *co)
{
}
*/

static int mipi_dsi_dphy_init(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t bpp, time_out = 100;
	uint32_t lock;
	uint32_t req_bit_clk;
	struct pll_divider div;
	struct fb_videomode *mode = mipi_dsi->mode;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1,
			   MIPI_ISO_DISABLE, MIPI_ISO_DISABLE);

	bpp = fmt_to_bpp(lcd_config->dpi_fmt);
	req_bit_clk = PICOS2KHZ(mode->pixclock) * bpp * 1000U;
	switch (lcd_config->data_lane_num) {
	case 1:
		break;
	case 2:
		req_bit_clk = req_bit_clk >> 1;
		break;
	case 4:
		req_bit_clk = req_bit_clk >> 2;
		break;
	default:
		dev_err(&mipi_dsi->pdev->dev,
			"requested data lane num is invalid\n");
		return -EINVAL;
	}

	if (mipi_dsi->encoder) {
		if (req_bit_clk > lcd_config->max_phy_clk)
			return -EINVAL;
	}

	if (!mipi_dsi->encoder) {
		/* PLL out clock = refclk * CM / (CN * CO)
		 * refclock = 24MHz
		 * pll vco = 24 * 40 / (3 * 1) = 320MHz
		 */
		div.cn = 0x10; /* 3  */
		div.cm = 0xc8; /* 40 */
		div.co = 0x0;  /* 1  */
	} else {
		/* pll vco = 24 * 63 / (5 * 1) = 302.4MHz */
		div.cn = 0x1C; /* 5  */
		div.cm = 0xDF; /* 63 */
		div.co = 0x0;  /* 1  */
	}

	writel(div.cn, mipi_dsi->mmio_base + DPHY_CN);
	writel(div.cm, mipi_dsi->mmio_base + DPHY_CM);
	writel(div.co, mipi_dsi->mmio_base + DPHY_CO);

	writel(0x25, mipi_dsi->mmio_base + DPHY_TST);
	writel(0x0, mipi_dsi->mmio_base + DPHY_PD_PLL);

	while(!(lock = readl(mipi_dsi->mmio_base + DPHY_LOCK))) {
		udelay(10);
		time_out--;
		if (time_out == 0) {
			dev_err(&mipi_dsi->pdev->dev, "cannot get the dphy lock = 0x%x\n", lock);
			return -EINVAL;
		}
	}
	printk(KERN_EMERG "%s: dphy lock = 0x%x\n", __func__, lock);

	writel(0x0, mipi_dsi->mmio_base + DPHY_LOCK_BYP);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RTERM_SEL);
	writel(0x0, mipi_dsi->mmio_base + DPHY_AUTO_PD_EN);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RXLPRP);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RXCDRP);
	writel(0x0, mipi_dsi->mmio_base + DPHY_M_PRG_HS_PREPARE);
	writel(0x0, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_PREPARE);
	writel(0x9, mipi_dsi->mmio_base + DPHY_M_PRG_HS_ZERO);
	writel(0x20, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_ZERO);
	writel(0x5, mipi_dsi->mmio_base + DPHY_M_PRG_HS_TRAIL);
	writel(0x5, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_TRAIL);
	writel(0x0, mipi_dsi->mmio_base + DPHY_PD_DPHY);

	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, DSI_PLL_EN);
	return 0;
}

static int mipi_dsi_host_init(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t lane_num;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	switch (lcd_config->data_lane_num) {
	case 1:
		lane_num = 0x0;
		break;
	case 2:
		lane_num = 0x1;
		break;
	default:
		/* Invalid lane num */
		return -EINVAL;
	}

	writel(lane_num, mipi_dsi->mmio_base + HOST_CFG_NUM_LANES);
	writel(mipi_dsi->encoder ? 0x0 : 0x1,
	       mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
	writel(0x1, mipi_dsi->mmio_base + HOST_CFG_T_PRE);
	writel(52, mipi_dsi->mmio_base + HOST_CFG_T_POST);
	writel(13, mipi_dsi->mmio_base + HOST_CFG_TX_GAP);
	writel(mipi_dsi->encoder ? 0x0 : 0x1,
	       mipi_dsi->mmio_base + HOST_CFG_AUTOINSERT_EOTP);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_EXTRA_CMDS_AFTER_EOTP);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_HTX_TO_COUNT);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_LRX_H_TO_COUNT);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_BTA_H_TO_COUNT);
	writel(0x3A98, mipi_dsi->mmio_base + HOST_CFG_TWAKEUP);

	return 0;
}

static int mipi_dsi_dpi_init(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t bpp, color_coding, pixel_fmt;
	struct fb_videomode *mode = mipi_dsi->mode;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	bpp = fmt_to_bpp(lcd_config->dpi_fmt);

	writel(mode->xres, mipi_dsi->mmio_base + DPI_PIXEL_PAYLOAD_SIZE);
	writel(mode->xres, mipi_dsi->mmio_base + DPI_PIXEL_FIFO_SEND_LEVEL);

	switch (bpp) {
	case 24:
		color_coding = 5;
		pixel_fmt = 3;
		break;
	case 16:
	case 18:
	default:
		break;
	}
	writel(color_coding, mipi_dsi->mmio_base + DPI_INTERFACE_COLOR_CODING);
	writel(pixel_fmt, mipi_dsi->mmio_base + DPI_PIXEL_FORMAT);
	writel(0x0, mipi_dsi->mmio_base + DPI_VSYNC_POLARITY);
	writel(0x0, mipi_dsi->mmio_base + DPI_HSYNC_POLARITY);
	writel(mipi_dsi->traffic_mode,
	       mipi_dsi->mmio_base + DPI_VIDEO_MODE);

	switch (mipi_dsi->traffic_mode) {
	case DSI_NON_BURST_WITH_SYNC_PULSE:
		writel(0x10, mipi_dsi->mmio_base + DPI_HFP);
		writel(0x60, mipi_dsi->mmio_base + DPI_HBP);
		writel(0xf0, mipi_dsi->mmio_base + DPI_HSA);
		break;
	case DSI_BURST_MODE:
		writel(mode->right_margin * (bpp >> 3),
		       mipi_dsi->mmio_base + DPI_HFP);
		writel(mode->left_margin * (bpp >> 3),
		       mipi_dsi->mmio_base + DPI_HBP);
		writel(mode->hsync_len * (bpp >> 3),
		       mipi_dsi->mmio_base + DPI_HSA);
		break;
	default:
		pr_debug("unsupport traffic mode: %d\n",
			 mipi_dsi->traffic_mode);
		return -EINVAL;
	}
	writel(0x0, mipi_dsi->mmio_base + DPI_ENABLE_MULT_PKTS);

	writel(mode->upper_margin, mipi_dsi->mmio_base + DPI_VBP);
	writel(mode->lower_margin, mipi_dsi->mmio_base + DPI_VFP);
	writel(0x1, mipi_dsi->mmio_base + DPI_BLLP_MODE);
	writel(0x0, mipi_dsi->mmio_base + DPI_USE_NULL_PKT_BLLP);

	writel(mode->yres - 1, mipi_dsi->mmio_base + DPI_VACTIVE);

	writel(0x0, mipi_dsi->mmio_base + DPI_VC);

	return 0;
}

static void mipi_dsi_init_interrupt(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t irqs_enable;

	/* disable all the irqs */
	writel(0xffffffff, mipi_dsi->mmio_base + HOST_IRQ_MASK);
	writel(0x7, mipi_dsi->mmio_base + HOST_IRQ_MASK2);

	irqs_enable = ~(HOST_IRQ_MASK_TX_PKT_DONE_MASK |
			HOST_IRQ_MASK_RX_PKT_HDR_RCVD_MASK);

	writel(irqs_enable, mipi_dsi->mmio_base + HOST_IRQ_MASK);
}

static int mipi_display_enter_sleep(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_OFF,
			       NULL, 0);
	if (err)
		return -EINVAL;
	msleep(50);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_ENTER_SLEEP_MODE,
			NULL, 0);
	if (err) {
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command sleep in error!\n");
	}
	msleep(MIPI_LCD_SLEEP_MODE_DELAY);

	return err;
}

static int mipi_display_exit_sleep(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE,
			NULL, 0);
	if (err) {
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command sleep-out error!\n");
		return err;
	}
	msleep(MIPI_LCD_SLEEP_MODE_DELAY);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_ON,
			NULL, 0);
	msleep(20);

	return err;
}

static void reset_dsi_domains(struct mipi_dsi_info *mipi_dsi, bool reset)
{
	/* escape domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_ESC_N, (reset ? 0 : DSI_RST_ESC_N));
	/* byte domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_BYTE_N, (reset ? 0 : DSI_RST_BYTE_N));

	/* dpi domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_DPI_N, (reset ? 0 : DSI_RST_DPI_N));
}

static int mipi_dsi_enable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi)
{
	int ret;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	if (!mipi_dsi->dsi_power_on)
		pm_runtime_get_sync(&mipi_dsi->pdev->dev);

	if (!mipi_dsi->lcd_inited) {
		ret = clk_set_rate(mipi_dsi->esc_clk, 80000000);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
					"clk enable error: %d!\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(mipi_dsi->esc_clk);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
			"clk enable error: %d!\n", ret);
			return -EINVAL;
		}

		if ((ret = mipi_dsi_dphy_init(mipi_dsi)) < 0)
			return ret;

		if ((ret = mipi_dsi_host_init(mipi_dsi)) < 0)
			return ret;

		mipi_dsi_dpi_init(mipi_dsi);

		reset_dsi_domains(mipi_dsi, 0);

		/* display_en */
		regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
				   DSI_SD, 0x0);
		/* normal cm */
		regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
				   DSI_CM, 0x0);
		msleep(20);

		if (!mipi_dsi->encoder) {
			ret = device_reset(&mipi_dsi->pdev->dev);
			if (ret) {
				dev_err(&mipi_dsi->pdev->dev,
						"failed to reset device: %d\n", ret);
				return -EINVAL;
			}
			msleep(60);

			mipi_dsi_init_interrupt(mipi_dsi);

			ret = mipi_dsi->lcd_callback->mipi_lcd_setup(mipi_dsi);
			if (ret < 0) {
				dev_err(&mipi_dsi->pdev->dev,
						"failed to init mipi lcd.\n");
				return ret;
			}
			mipi_dsi_set_mode(mipi_dsi, DSI_HS_MODE);
		}

		mipi_dsi->lcd_inited = 1;
	} else {
		ret = clk_prepare_enable(mipi_dsi->esc_clk);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
				"clk enable error: %d!\n", ret);
			return -EINVAL;
		}

		reset_dsi_domains(mipi_dsi, 0);

		if (!mipi_dsi->encoder) {
			ret = mipi_display_exit_sleep(mipi_dsi->disp_mipi);
			if (ret) {
				dev_err(&mipi_dsi->pdev->dev, "exit sleep failed\n");
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void mipi_dsi_wr_tx_header(struct mipi_dsi_info *mipi_dsi,
				  u8 di, u8 data0, u8 data1, u8 mode, u8 need_bta)
{
	uint32_t pkt_control = 0;
	uint16_t word_count = 0;

	word_count = data0 | (data1 << 8);
	pkt_control = HOST_PKT_CONTROL_WC(word_count) |
		      HOST_PKT_CONTROL_VC(0)	      |
		      HOST_PKT_CONTROL_DT(di)	      |
		      HOST_PKT_CONTROL_HS_SEL(mode)   |
		      HOST_PKT_CONTROL_BTA_TX(need_bta);

	dev_dbg(&mipi_dsi->pdev->dev, "pkt_control = %x\n", pkt_control);
	writel(pkt_control, mipi_dsi->mmio_base + HOST_PKT_CONTROL);
}

static void mipi_dsi_wr_tx_data(struct mipi_dsi_info *mipi_dsi,
				uint32_t tx_data)
{
	writel(tx_data, mipi_dsi->mmio_base + HOST_TX_PAYLOAD);
}

static void mipi_dsi_long_data_wr(struct mipi_dsi_info *mipi_dsi,
			const uint8_t *data0, uint32_t data_size)
{
	uint32_t data_cnt = 0, payload = 0;

	/* in case that data count is more than 4 */
	for (data_cnt = 0; data_cnt < data_size; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((data_size - data_cnt) < 4) {
			if ((data_size - data_cnt) == 3) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8) |
					  (data0[data_cnt + 2] << 16);
				dev_dbg(&mipi_dsi->pdev->dev, "count = 3 payload = %x, %x %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1], data0[data_cnt + 2]);
			} else if ((data_size - data_cnt) == 2) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8);
				dev_dbg(&mipi_dsi->pdev->dev, "count = 2 payload = %x, %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1]);
			} else if ((data_size - data_cnt) == 1) {
				payload = data0[data_cnt];
				dev_dbg(&mipi_dsi->pdev->dev, "count = 1 payload = %x, %x\n",
					payload, data0[data_cnt]);
			}

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		} else {
			payload = data0[data_cnt] |
				  (data0[data_cnt + 1] << 8) |
				  (data0[data_cnt + 2] << 16) |
				  (data0[data_cnt + 3] << 24);

			dev_dbg(&mipi_dsi->pdev->dev,
				"count = 4 payload = %x, %x %x %x %x\n",
                                payload, *(u8 *)(data0 + data_cnt),
                                data0[data_cnt + 1],
                                data0[data_cnt + 2],
                                data0[data_cnt + 3]);

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		}
	}
}

static int mipi_dsi_pkt_write(struct mipi_dsi_info *mipi_dsi,
			u8 data_type, const u32 *buf, int len)
{
	int ret = 0;
	struct platform_device *pdev = mipi_dsi->pdev;
	const uint8_t *data = (const uint8_t *)buf;

	if (len == 0)
		/* handle generic long write command */
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, data[0], data[1], DSI_LP_MODE, 0);
	else {
		reinit_completion(&dsi_tx_done);

		/* handle generic long write command */
		mipi_dsi_long_data_wr(mipi_dsi, data, len);
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, len & 0xff,
				      (len & 0xff00) >> 8, DSI_LP_MODE, 0);
	}

	/* send packet */
	writel(0x1, mipi_dsi->mmio_base + HOST_SEND_PACKET);
	ret = wait_for_completion_timeout(&dsi_tx_done, MIPI_FIFO_TIMEOUT);

	if (!ret) {
		dev_err(&pdev->dev, "wait tx done timeout!\n");
		return -ETIMEDOUT;
	}
	mdelay(10);

	return 0;
}

static uint32_t mipi_dsi_rd_rx_header(struct mipi_dsi_info *mipi_dsi)
{
	return readl(mipi_dsi->mmio_base + HOST_PKT_RX_PKT_HEADER);
}

static int mipi_dsi_pkt_read(struct mipi_dsi_info *mipi_dsi,
			     uint8_t data_type, uint32_t *buf, int len)
{
	int ret;
	uint32_t rx_hdr;
	struct platform_device *pdev = mipi_dsi->pdev;
	const uint8_t *data = (const uint8_t *)buf;

	if (len <= 4) {
		reinit_completion(&dsi_rx_done);

		mipi_dsi_wr_tx_header(mipi_dsi, data_type, data[0], data[1], DSI_LP_MODE, 1);
		writel(0x1, mipi_dsi->mmio_base + HOST_SEND_PACKET);

		ret = wait_for_completion_timeout(&dsi_rx_done, MIPI_FIFO_TIMEOUT);
		if (!ret) {
			dev_err(&pdev->dev, "wait rx done timeout!\n");
			return -ETIMEDOUT;
		}

		rx_hdr = mipi_dsi_rd_rx_header(mipi_dsi);
		dev_info(&pdev->dev, "rx: rx_hdr = 0x%x, data type = 0x%x, word_count = 0x%x\n",
				     rx_hdr, (rx_hdr >> 16) & 0x3f, rx_hdr & 0xffff);

		buf[0] = rx_hdr & 0xff;
	} else {
		/* TODO: add support later */
	}

	return 0;
}

static int mipi_dsi_dcs_cmd(struct mipi_dsi_info *mipi_dsi,
			    u8 cmd, const u32 *param, int num)
{
	int err = 0;
	u32 buf[DSI_CMD_BUF_MAXSIZE];

	switch (cmd) {
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_DISPLAY_OFF:
		buf[0] = cmd;
		buf[1] = 0x0;
		err = mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
		break;

	default:
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command:0x%x Not supported!\n", cmd);
		break;
	}

	return err;
}

static void mipi_dsi_disable(struct mxc_dispdrv_handle *disp,
			     struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	if (!mipi_dsi->encoder)
		mipi_display_enter_sleep(mipi_dsi->disp_mipi);

	clk_disable_unprepare(mipi_dsi->esc_clk);

	reset_dsi_domains(mipi_dsi, 1);
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, 0x0);
}

static int mipi_dsi_setup(struct mxc_dispdrv_handle *disp,
			  struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);
	int xres_virtual = fbi->var.xres_virtual;
	int yres_virtual = fbi->var.yres_virtual;
	int xoffset = fbi->var.xoffset;
	int yoffset = fbi->var.yoffset;
	int pixclock = fbi->var.pixclock;

	if (!mipi_dsi->mode)
		return 0;

	/* set the mode back to var in case userspace changes it */
	fb_videomode_to_var(&fbi->var, mipi_dsi->mode);

	/* restore some var entries cached */
	fbi->var.xres_virtual = xres_virtual;
	fbi->var.yres_virtual = yres_virtual;
	fbi->var.xoffset = xoffset;
	fbi->var.yoffset = yoffset;
	fbi->var.pixclock = pixclock;

	return 0;
}

static struct mxc_dispdrv_driver mipi_dsi_drv = {
	.name = DISPDRV_MIPI,
	.init = mipi_dsi_disp_init,
	.deinit = mipi_dsi_disp_deinit,
	.enable = mipi_dsi_enable,
	.disable = mipi_dsi_disable,
	.setup = mipi_dsi_setup,
};

static irqreturn_t mipi_dsi_irq_handler(int irq, void *data)
{
	uint32_t irq_status;
	struct mipi_dsi_info *mipi_dsi = data;
	struct platform_device *pdev = mipi_dsi->pdev;

	irq_status = readl(mipi_dsi->mmio_base + HOST_IRQ_STATUS);

	dev_dbg(&pdev->dev, "irq_status = 0x%x\n", irq_status);

	if (irq_status & HOST_IRQ_STATUS_TX_PKT_DONE) {
		dev_dbg(&pdev->dev, "payload tx finished\n");
		complete(&dsi_tx_done);
	}

	if (irq_status & HOST_IRQ_STATUS_RX_PKT_HDR_RCVD) {
		dev_dbg(&pdev->dev, "rx data finished\n");
		complete(&dsi_rx_done);
	}

	return IRQ_HANDLED;
}

/**
 * This function is called by the driver framework to initialize the MIPI DSI
 * device.
 *
 * @param	pdev	The device structure for the MIPI DSI passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mipi_dsi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mipi_dsi_info *mipi_dsi;
	struct device_node *endpoint = NULL, *remote;
	struct resource *res;
	const char *lcd_panel;
	int ret = 0;
	u32 vmode_index;

	mipi_dsi = devm_kzalloc(&pdev->dev, sizeof(*mipi_dsi), GFP_KERNEL);
	if (!mipi_dsi)
		return -ENOMEM;
	mipi_dsi->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform mem resource\n");
		return -ENOMEM;
	}

	mipi_dsi->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mipi_dsi->mmio_base))
		return -ENODEV;

	mipi_dsi->irq = platform_get_irq(pdev, 0);
	if (mipi_dsi->irq < 0) {
		dev_err(&pdev->dev, "failed to get device irq\n");
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, mipi_dsi->irq,
				mipi_dsi_irq_handler,
				0, "mipi_dsi_northwest", mipi_dsi);
	if (ret) {
		dev_err(&pdev->dev, "failed to request mipi dsi irq\n");
		return ret;
	}

	mipi_dsi->esc_clk = devm_clk_get(&pdev->dev, "mipi_dsi_clk");
	if (IS_ERR(mipi_dsi->esc_clk)) {
		dev_err(&pdev->dev, "failed to get esc clk\n");
		return PTR_ERR(mipi_dsi->esc_clk);
	}

	mipi_dsi->regmap = syscon_regmap_lookup_by_phandle(np, "sim");
	if (IS_ERR(mipi_dsi->regmap)) {
		dev_err(&pdev->dev, "failed to get parent regmap\n");
		return -EINVAL;
	}

	/* check whether an encoder exists */
	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (endpoint) {
		remote = of_graph_get_remote_port_parent(endpoint);
		if (!remote)
			return -EINVAL;

		ret = of_property_read_u32(remote, "video-mode", &vmode_index);
		if ((ret < 0) || (vmode_index >= ARRAY_SIZE(mxc_cea_mode)))
			return -EINVAL;

		mipi_dsi->mode = devm_kzalloc(&pdev->dev,
					      sizeof(struct fb_videomode),
					      GFP_KERNEL);
		if (!mipi_dsi->mode)
			return -ENOMEM;

		memcpy(mipi_dsi->mode, &mxc_cea_mode[vmode_index],
		       sizeof(struct fb_videomode));

		ret = of_property_read_u32(remote, "dsi-traffic-mode",
					   &mipi_dsi->traffic_mode);
		if (ret < 0 || mipi_dsi->traffic_mode > 2) {
			devm_kfree(&pdev->dev, mipi_dsi->mode);
			return -EINVAL;
		}

		mipi_dsi->lcd_config = devm_kzalloc(&pdev->dev,
						sizeof(struct mipi_lcd_config),
						GFP_KERNEL);
		if (!mipi_dsi->lcd_config) {
			kfree(mipi_dsi->mode);
			return -ENOMEM;
		}

		mipi_dsi->encoder = 1;
	} else
		/* Default, using 'BURST-MODE' for mipi panel */
		mipi_dsi->traffic_mode = 2;

	ret = of_property_read_string(np, "lcd_panel", &lcd_panel);
	if (ret) {
		dev_err(&pdev->dev, "failed to read lcd_panel property\n");
		return ret;
	}

	/* mipi VDDA is sw1 in PMIC which is always on */

	mipi_dsi->lcd_panel = kstrdup(lcd_panel, GFP_KERNEL);
	if (!mipi_dsi->lcd_panel) {
		dev_err(&pdev->dev, "failed to allocate lcd panel\n");
		ret = -ENOMEM;
	}

	mipi_dsi->disp_mipi = mxc_dispdrv_register(&mipi_dsi_drv);
	if (IS_ERR(mipi_dsi->disp_mipi)) {
		dev_err(&pdev->dev, "mxc_dispdrv_register error\n");
		ret = PTR_ERR(mipi_dsi->disp_mipi);
		goto dispdrv_reg_fail;
	}

	mipi_dsi->mipi_dsi_pkt_read  = mipi_dsi_pkt_read;
	mipi_dsi->mipi_dsi_pkt_write = mipi_dsi_pkt_write;
	mipi_dsi->mipi_dsi_dcs_cmd   = mipi_dsi_dcs_cmd;

	pm_runtime_enable(&pdev->dev);

	mxc_dispdrv_setdata(mipi_dsi->disp_mipi, mipi_dsi);
	dev_set_drvdata(&pdev->dev, mipi_dsi);

	dev_info(&pdev->dev, "i.MX MIPI DSI driver probed\n");
	return ret;

dispdrv_reg_fail:
	kfree(mipi_dsi->lcd_panel);
	return ret;
}

static int mipi_dsi_remove(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(mipi_dsi->disp_mipi);
	mxc_dispdrv_unregister(mipi_dsi->disp_mipi);

	kfree(mipi_dsi->lcd_panel);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static void mipi_dsi_shutdown(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (mipi_dsi->lcd_inited) {
		clk_prepare_enable(mipi_dsi->esc_clk);
		if (!mipi_dsi->encoder)
			mipi_display_enter_sleep(mipi_dsi->disp_mipi);

		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_PLL);
		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_DPHY);

		clk_disable_unprepare(mipi_dsi->esc_clk);

		mipi_dsi->lcd_inited = 0;
	}

	reset_dsi_domains(mipi_dsi, 1);
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, 0x0);
}

static const struct of_device_id imx_mipi_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx7ulp-mipi-dsi", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_mipi_dsi_dt_ids);

static int mipi_dsi_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	mipi_dsi->dsi_power_on = 0;

	return 0;
}

static int mipi_dsi_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (!mipi_dsi->dsi_power_on) {
		request_bus_freq(BUS_FREQ_HIGH);
		dev_dbg(dev, "mipi dsi busfreq high request.\n");

		mipi_dsi->dsi_power_on = 1;
	}

	return 0;
}

static int mipi_dsi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (unlikely(mipi_dsi->lcd_inited)) {
		clk_prepare_enable(mipi_dsi->esc_clk);

		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_PLL);
		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_DPHY);

		clk_disable_unprepare(mipi_dsi->esc_clk);
		mipi_dsi->lcd_inited = 0;
	}

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int mipi_dsi_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);

	return 0;
}

static const struct dev_pm_ops mipi_dsi_pm_ops = {
	.suspend = mipi_dsi_suspend,
	.resume  = mipi_dsi_resume,
	.runtime_suspend = mipi_dsi_runtime_suspend,
	.runtime_resume  = mipi_dsi_runtime_resume,
	.runtime_idle	 = NULL,
};

static struct platform_driver mipi_dsi_driver = {
	.driver = {
		.of_match_table = imx_mipi_dsi_dt_ids,
		.name = "mipi_dsi_northwest",
		.pm = &mipi_dsi_pm_ops,
	},
	.probe  = mipi_dsi_probe,
	.remove = mipi_dsi_remove,
	.shutdown = mipi_dsi_shutdown,
};

static int __init mipi_dsi_init(void)
{
	int err;

	err = platform_driver_register(&mipi_dsi_driver);
	if (err) {
		pr_err("mipi_dsi_driver register failed\n");
		return err;
	}

	pr_debug("MIPI DSI driver module loaded: %s\n", mipi_dsi_driver.driver.name);

	return 0;
}

static void __exit mipi_dsi_exit(void)
{
	platform_driver_unregister(&mipi_dsi_driver);
}

module_init(mipi_dsi_init);
module_exit(mipi_dsi_exit);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX MIPI DSI driver");
MODULE_LICENSE("GPL");
