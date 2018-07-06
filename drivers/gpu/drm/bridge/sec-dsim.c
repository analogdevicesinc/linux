/*
 * Samsung MIPI DSIM Bridge
 *
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <drm/bridge/sec_mipi_dsim.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <video/videomode.h>

/* dsim registers */
#define DSIM_VERSION			0x00
#define DSIM_STATUS			0x04
#define DSIM_RGB_STATUS			0x08
#define DSIM_SWRST			0x0c
#define DSIM_CLKCTRL			0x10
#define DSIM_TIMEOUT			0x14
#define DSIM_CONFIG			0x18
#define DSIM_ESCMODE			0x1c
#define DSIM_MDRESOL			0x20
#define DSIM_MVPORCH			0x24
#define DSIM_MHPORCH			0x28
#define DSIM_MSYNC			0x2c
#define DSIM_SDRESOL			0x30
#define DSIM_INTSRC			0x34
#define DSIM_INTMSK			0x38

/* packet */
#define DSIM_PKTHDR			0x3c
#define DSIM_PAYLOAD			0x40
#define DSIM_RXFIFO			0x44
#define DSIM_FIFOTHLD			0x48
#define DSIM_FIFOCTRL			0x4c
#define DSIM_MEMACCHR			0x50
#define DSIM_MULTI_PKT			0x78

/* pll control */
#define DSIM_PLLCTRL_1G			0x90
#define DSIM_PLLCTRL			0x94
#define DSIM_PLLCTRL1			0x98
#define DSIM_PLLCTRL2			0x9c
#define DSIM_PLLTMR			0xa0

/* dphy */
#define DSIM_PHYTIMING			0xb4
#define DSIM_PHYTIMING1			0xb8
#define DSIM_PHYTIMING2			0xbc

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/* register bit fields */
#define STATUS_PLLSTABLE		BIT(31)
#define STATUS_SWRSTRLS			BIT(20)
#define STATUS_TXREADYHSCLK		BIT(10)
#define STATUS_ULPSCLK			BIT(9)
#define STATUS_STOPSTATECLK		BIT(8)
#define STATUS_GET_ULPSDAT(x)		REG_GET(x,  7,  4)
#define STATUS_GET_STOPSTATEDAT(x)	REG_GET(x,  3,  0)

#define RGB_STATUS_CMDMODE_INSEL	BIT(31)
#define RGB_STATUS_GET_RGBSTATE(x)	REG_GET(x, 12,  0)

#define CLKCTRL_TXREQUESTHSCLK		BIT(31)
#define CLKCTRL_DPHY_SEL_1G		BIT(29)
#define CLKCTRL_DPHY_SEL_1P5G		(0x0 << 29)
#define CLKCTRL_ESCCLKEN		BIT(28)
#define CLKCTRL_PLLBYPASS		BIT(29)
#define CLKCTRL_BYTECLKSRC_DPHY_PLL	REG_PUT(0, 26, 25)
#define CLKCTRL_BYTECLKEN		BIT(24)
#define CLKCTRL_SET_LANEESCCLKEN(x)	REG_PUT(x, 23, 19)
#define CLKCTRL_SET_ESCPRESCALER(x)	REG_PUT(x, 15,  0)

#define TIMEOUT_SET_BTAOUT(x)		REG_PUT(x, 23, 16)
#define TIMEOUT_SET_LPDRTOUT(x)		REG_PUT(x, 15,  0)

#define CONFIG_NON_CONTINOUS_CLOCK_LANE	BIT(31)
#define CONFIG_CLKLANE_STOP_START	BIT(30)
#define CONFIG_MFLUSH_VS		BIT(29)
#define CONFIG_EOT_R03			BIT(28)
#define CONFIG_SYNCINFORM		BIT(27)
#define CONFIG_BURSTMODE		BIT(26)
#define CONFIG_VIDEOMODE		BIT(25)
#define CONFIG_AUTOMODE			BIT(24)
#define CONFIG_HSEDISABLEMODE		BIT(23)
#define CONFIG_HFPDISABLEMODE		BIT(22)
#define CONFIG_HBPDISABLEMODE		BIT(21)
#define CONFIG_HSADISABLEMODE		BIT(20)
#define CONFIG_SET_MAINVC(x)		REG_PUT(x, 19, 18)
#define CONFIG_SET_SUBVC(x)		REG_PUT(x, 17, 16)
#define CONFIG_SET_MAINPIXFORMAT(x)	REG_PUT(x, 14, 12)
#define CONFIG_SET_SUBPIXFORMAT(x)	REG_PUT(x, 10,  8)
#define CONFIG_SET_NUMOFDATLANE(x)	REG_PUT(x,  6,  5)
#define CONFIG_SET_LANEEN(x)		REG_PUT(x,  4,  0)

#define MDRESOL_MAINSTANDBY		BIT(31)
#define MDRESOL_SET_MAINVRESOL(x)	REG_PUT(x, 27, 16)
#define MDRESOL_SET_MAINHRESOL(x)	REG_PUT(x, 11,  0)

#define MVPORCH_SET_CMDALLOW(x)		REG_PUT(x, 31, 28)
#define MVPORCH_SET_STABLEVFP(x)	REG_PUT(x, 26, 16)
#define MVPORCH_SET_MAINVBP(x)		REG_PUT(x, 10,  0)

#define MHPORCH_SET_MAINHFP(x)		REG_PUT(x, 31, 16)
#define MHPORCH_SET_MAINHBP(x)		REG_PUT(x, 15,  0)

#define MSYNC_SET_MAINVSA(x)		REG_PUT(x, 31, 22)
#define MSYNC_SET_MAINHSA(x)		REG_PUT(x, 15,  0)

#define INTSRC_PLLSTABLE		BIT(31)
#define INTSRC_SWRSTRELEASE		BIT(30)
#define INTSRC_SFRPLFIFOEMPTY		BIT(29)
#define INTSRC_SFRPHFIFOEMPTY		BIT(28)
#define INTSRC_FRAMEDONE		BIT(24)
#define INTSRC_LPDRTOUT			BIT(21)
#define INTSRC_TATOUT			BIT(20)
#define INTSRC_RXDATDONE		BIT(18)
#define INTSRC_MASK			(INTSRC_PLLSTABLE	|	\
					 INTSRC_SWRSTRELEASE	|	\
					 INTSRC_SFRPLFIFOEMPTY	|	\
					 INTSRC_SFRPHFIFOEMPTY	|	\
					 INTSRC_FRAMEDONE	|	\
					 INTSRC_LPDRTOUT	|	\
					 INTSRC_TATOUT		|	\
					 INTSRC_RXDATDONE)

#define INTMSK_MSKPLLSTABLE		BIT(31)
#define INTMSK_MSKSWRELEASE		BIT(30)
#define INTMSK_MSKSFRPLFIFOEMPTY	BIT(29)
#define INTMSK_MSKSFRPHFIFOEMPTY	BIT(28)
#define INTMSK_MSKFRAMEDONE		BIT(24)
#define INTMSK_MSKLPDRTOUT		BIT(21)
#define INTMSK_MSKTATOUT		BIT(20)
#define INTMSK_MSKRXDATDONE		BIT(18)

#define FIFOCTRL_FULLRX			BIT(25)
#define FIFOCTRL_EMPTYRX		BIT(24)
#define FIFOCTRL_FULLHSFR		BIT(23)
#define FIFOCTRL_EMPTYHSFR		BIT(22)
#define FIFOCTRL_FULLLSFR		BIT(21)
#define FIFOCTRL_EMPTYLSFR		BIT(20)
#define FIFOCTRL_FULLHMAIN		BIT(11)
#define FIFOCTRL_EMPTYHMAIN		BIT(10)
#define FIFOCTRL_FULLLMAIN		BIT(9)
#define FIFOCTRL_EMPTYLMAIN		BIT(8)
#define FIFOCTRL_NINITRX		BIT(4)
#define FIFOCTRL_NINITSFR		BIT(3)
#define FIFOCTRL_NINITI80		BIT(2)
#define FIFOCTRL_NINITSUB		BIT(1)
#define FIFOCTRL_NINITMAIN		BIT(0)

#define PLLCTRL_DPDNSWAP_CLK		BIT(25)
#define PLLCTRL_DPDNSWAP_DAT		BIT(24)
#define PLLCTRL_PLLEN			BIT(23)
#define PLLCTRL_SET_PMS(x)		REG_PUT(x, 19,  1)

#define PHYTIMING_SET_M_TLPXCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING_SET_M_THSEXITCTL(x)	REG_PUT(x,  7,  0)

#define PHYTIMING1_SET_M_TCLKPRPRCTL(x)	 REG_PUT(x, 31, 24)
#define PHYTIMING1_SET_M_TCLKZEROCTL(x)	 REG_PUT(x, 23, 16)
#define PHYTIMING1_SET_M_TCLKPOSTCTL(x)	 REG_PUT(x, 15,  8)
#define PHYTIMING1_SET_M_TCLKTRAILCTL(x) REG_PUT(x,  7,  0)

#define PHYTIMING2_SET_M_THSPRPRCTL(x)	REG_PUT(x, 23, 16)
#define PHYTIMING2_SET_M_THSZEROCTL(x)	REG_PUT(x, 15,  8)
#define PHYTIMING2_SET_M_THSTRAILCTL(x)	REG_PUT(x,  7,  0)

#define dsim_read(dsim, reg)		readl(dsim->base + reg)
#define dsim_write(dsim, val, reg)	writel(val, dsim->base + reg)

/* fixed phy ref clk rate */
#define PHY_REF_CLK		27000000

#define MAX_MAIN_HRESOL		2047
#define MAX_MAIN_VRESOL		2047
#define MAX_SUB_HRESOL		1024
#define MAX_SUB_VRESOL		1024

/* in KHZ */
#define MAX_ESC_CLK_FREQ	20000

/* dsim all irqs index */
#define PLLSTABLE		1
#define SWRSTRELEASE		2
#define SFRPLFIFOEMPTY		3
#define SFRPHFIFOEMPTY		4
#define SYNCOVERRIDE		5
#define BUSTURNOVER		6
#define FRAMEDONE		7
#define LPDRTOUT		8
#define TATOUT			9
#define RXDATDONE		10
#define RXTE			11
#define RXACK			12
#define ERRRXECC		13
#define ERRRXCRC		14
#define ERRESC3			15
#define ERRESC2			16
#define ERRESC1			17
#define ERRESC0			18
#define ERRSYNC3		19
#define ERRSYNC2		20
#define ERRSYNC1		21
#define ERRSYNC0		22
#define ERRCONTROL3		23
#define ERRCONTROL2		24
#define ERRCONTROL1		25
#define ERRCONTROL0		26

#define to_sec_mipi_dsim(dsi) container_of(dsi, struct sec_mipi_dsim, dsi_host)

/* DSIM PLL configuration from spec:
 *
 * Fout(DDR) = (M * Fin) / (P * 2^S), so Fout / Fin = M / (P * 2^S)
 * Fin_pll   = Fin / P     (6 ~ 12 MHz)
 * S: [2:0], M: [12:3], P: [18:13], so
 * TODO: 'S' is in [0 ~ 3], 'M' is in, 'P' is in [1 ~ 33]
 *
 */

struct sec_mipi_dsim {
	struct mipi_dsi_host dsi_host;
	struct drm_connector connector;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	struct drm_bridge *next;
	struct drm_panel *panel;
	struct device *dev;

	void __iomem *base;
	int irq;

	struct clk *clk_cfg;
	struct clk *clk_pllref;
	struct clk *pclk;			/* pixel clock */

	/* kHz clocks */
	uint64_t pix_clk;
	uint64_t bit_clk;

	unsigned int lanes;
	unsigned int channel;			/* virtual channel */
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	unsigned int pms;
	unsigned int p;
	unsigned int m;
	unsigned int s;
	unsigned long long lp_data_rate;
	unsigned long long hs_data_rate;
	struct videomode vmode;

	struct completion pll_stable;
	const struct sec_mipi_dsim_plat_data *pdata;
};

/* For now, dsim only support one device attached */
static int sec_mipi_dsim_host_attach(struct mipi_dsi_host *host,
				     struct mipi_dsi_device *dsi)
{
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);
	const struct sec_mipi_dsim_plat_data *pdata = dsim->pdata;
	struct device *dev = dsim->dev;
	struct drm_panel *panel;

	if (!dsi->lanes || dsi->lanes > pdata->max_data_lanes) {
		dev_err(dev, "invalid data lanes number\n");
		return -EINVAL;
	}

	if (dsim->channel)
		return -EINVAL;

	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO)		||
	    !((dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)	||
	      (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE))) {
		dev_err(dev, "unsupported dsi mode\n");
		return -EINVAL;
	}

	if (dsi->format != MIPI_DSI_FMT_RGB888 &&
	    dsi->format != MIPI_DSI_FMT_RGB565 &&
	    dsi->format != MIPI_DSI_FMT_RGB666 &&
	    dsi->format != MIPI_DSI_FMT_RGB666_PACKED) {
		dev_err(dev, "unsupported pixel format: %#x\n", dsi->format);
		return -EINVAL;
	}

	if (!dsim->next) {
		/* 'dsi' must be panel device */
		panel = of_drm_find_panel(dsi->dev.of_node);

		if (!panel) {
			dev_err(dev, "refuse unknown dsi device attach\n");
			WARN_ON(!panel);
			return -ENODEV;
		}

		/* Don't support multiple panels */
		if (dsim->panel && panel && dsim->panel != panel) {
			dev_err(dev, "don't support multiple panels\n");
			return -EBUSY;
		}

		dsim->panel = panel;
	}

	dsim->lanes	 = dsi->lanes;
	dsim->channel	 = dsi->channel;
	dsim->format	 = dsi->format;
	dsim->mode_flags = dsi->mode_flags;

	/* TODO: support later */
#if 0
	if (dsim->connector.dev)
		drm_helper_hpd_irq_event(dsim->connector.dev);
#endif

	return 0;
}

static int sec_mipi_dsim_host_detach(struct mipi_dsi_host *host,
				     struct mipi_dsi_device *dsi)
{
	struct sec_mipi_dsim *dsim = to_sec_mipi_dsim(host);

	if (WARN_ON(!dsim->next && !dsim->panel))
		return -ENODEV;

	/* clear the saved dsi parameters */
	dsim->lanes	 = 0;
	dsim->channel	 = 0;
	dsim->format	 = 0;
	dsim->mode_flags = 0;

	return 0;
}

static ssize_t sec_mipi_dsim_host_transfer(struct mipi_dsi_host *host,
					   const struct mipi_dsi_msg *msg)
{
	/* TODO: Add support later */

	return 0;
}

static const struct mipi_dsi_host_ops sec_mipi_dsim_host_ops = {
	.attach   = sec_mipi_dsim_host_attach,
	.detach   = sec_mipi_dsim_host_detach,
	.transfer = sec_mipi_dsim_host_transfer,
};

static int sec_mipi_dsim_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct sec_mipi_dsim *dsim = bridge->driver_private;
	struct device *dev = dsim->dev;
	struct device_node *np = dev->of_node;
	struct device_node *endpoint, *remote;
	struct drm_bridge *next = NULL;
	struct drm_encoder *encoder = dsim->encoder;

	/* TODO: All bridges and planes should have already been added */

	/* A panel has been found, ignor other dsi devices */
	if (dsim->panel)
		return 0;

	/* find next bridge */
	endpoint = of_graph_get_next_endpoint(np, NULL);
	/* At least one endpoint should be existed */
	if (!endpoint)
		return -ENODEV;

	while(endpoint && !next) {
		remote = of_graph_get_remote_port_parent(endpoint);

		if (!remote || !of_device_is_available(remote)) {
			of_node_put(remote);
			endpoint = of_graph_get_next_endpoint(np, endpoint);
			continue;
		}

		next = of_drm_find_bridge(remote);
		if (next) {
			/* Found */
			of_node_put(endpoint);
			break;
		}

		endpoint = of_graph_get_next_endpoint(np, endpoint);
	}

	/* No valid dsi device attached */
	if (!next)
		return -ENODEV;

	/* duplicate bridges or next bridge exists */
	WARN_ON(bridge == next || bridge->next || dsim->next);

	dsim->next = next;
	next->encoder = encoder;
	ret = drm_bridge_attach(encoder, next, bridge);
	if (ret) {
		dev_err(dev, "Unable to attach bridge %s: %d\n",
			remote->name, ret);
		dsim->next = NULL;
		return ret;
	}

	/* bridge chains */
	bridge->next = next;

	return 0;
}

static int sec_mipi_dsim_config_pll(struct sec_mipi_dsim *dsim)
{
	int ret;
	uint32_t pllctrl = 0, status, data_lanes_en, stop;

	dsim_write(dsim, 0x8000, DSIM_PLLTMR);

	/* TODO: config dp/dn swap if requires */

	pllctrl |= PLLCTRL_SET_PMS(dsim->pms) | PLLCTRL_PLLEN;
	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);

	ret = wait_for_completion_timeout(&dsim->pll_stable, HZ / 10);
	if (!ret) {
		dev_err(dsim->dev, "wait for pll stable time out\n");
		return -EBUSY;
	}

	/* wait for clk & data lanes to go to stop state */
	mdelay(1);

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	status = dsim_read(dsim, DSIM_STATUS);
	if (!(status & STATUS_STOPSTATECLK)) {
		dev_err(dsim->dev, "clock is not in stop state\n");
		return -EBUSY;
	}

	stop = STATUS_GET_STOPSTATEDAT(status);
	if ((stop & data_lanes_en) != data_lanes_en) {
		dev_err(dsim->dev,
			"one or more data lanes is not in stop state\n");
		return -EBUSY;
	}

	return 0;
}

static void sec_mipi_dsim_set_main_mode(struct sec_mipi_dsim *dsim)
{
	uint32_t bpp, hfp_wc, hbp_wc, hsa_wc;
	uint32_t mdresol = 0, mvporch = 0, mhporch = 0, msync = 0;
	struct videomode *vmode = &dsim->vmode;

	mdresol |= MDRESOL_SET_MAINVRESOL(vmode->vactive) |
		   MDRESOL_SET_MAINHRESOL(vmode->hactive);
	dsim_write(dsim, mdresol, DSIM_MDRESOL);

	mvporch |= MVPORCH_SET_MAINVBP(vmode->vback_porch)    |
		   MVPORCH_SET_STABLEVFP(vmode->vfront_porch) |
		   MVPORCH_SET_CMDALLOW(0x0);
	dsim_write(dsim, mvporch, DSIM_MVPORCH);

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);

	/* calculate hfp & hbp word counts */
	hfp_wc = vmode->hfront_porch * (bpp >> 3) / dsim->lanes - 6;
	hbp_wc = vmode->hback_porch  * (bpp >> 3) / dsim->lanes - 6;
	mhporch |= MHPORCH_SET_MAINHFP(hfp_wc) |
		   MHPORCH_SET_MAINHBP(hbp_wc);

	dsim_write(dsim, mhporch, DSIM_MHPORCH);

	/* calculate hsa word counts */
	hsa_wc = vmode->hsync_len * (bpp >> 3) / dsim->lanes - 6;
	msync |= MSYNC_SET_MAINVSA(vmode->vsync_len) |
		 MSYNC_SET_MAINHSA(hsa_wc);

	dsim_write(dsim, msync, DSIM_MSYNC);
}

static void sec_mipi_dsim_config_dpi(struct sec_mipi_dsim *dsim)
{
	uint32_t config = 0, rgb_status = 0, data_lanes_en;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO)
		rgb_status &= ~RGB_STATUS_CMDMODE_INSEL;
	else
		rgb_status |= RGB_STATUS_CMDMODE_INSEL;

	dsim_write(dsim, rgb_status, DSIM_RGB_STATUS);

	if (dsim->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		config |= CONFIG_CLKLANE_STOP_START;

	if (dsim->mode_flags & MIPI_DSI_MODE_VSYNC_FLUSH)
		config |= CONFIG_MFLUSH_VS;

	/* disable EoT packets in HS mode */
	if (dsim->mode_flags & MIPI_DSI_MODE_EOT_PACKET)
		config |= CONFIG_EOT_R03;

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		config |= CONFIG_VIDEOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			config |= CONFIG_BURSTMODE;

		else if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			config |= CONFIG_SYNCINFORM;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_AUTO_VERT)
			config |= CONFIG_AUTOMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSE)
			config |= CONFIG_HSEDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HFP)
			config |= CONFIG_HFPDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HBP)
			config |= CONFIG_HBPDISABLEMODE;

		if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_HSA)
			config |= CONFIG_HSADISABLEMODE;
	}

	config |= CONFIG_SET_MAINVC(dsim->channel);

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO) {
		switch (dsim->format) {
		case MIPI_DSI_FMT_RGB565:
			config |= CONFIG_SET_MAINPIXFORMAT(0x4);
			break;
		case MIPI_DSI_FMT_RGB666_PACKED:
			config |= CONFIG_SET_MAINPIXFORMAT(0x5);
			break;
		case MIPI_DSI_FMT_RGB666:
			config |= CONFIG_SET_MAINPIXFORMAT(0x6);
			break;
		case MIPI_DSI_FMT_RGB888:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		default:
			config |= CONFIG_SET_MAINPIXFORMAT(0x7);
			break;
		}
	}

	/* config data lanes number and enable lanes */
	data_lanes_en = (0x1 << dsim->lanes) - 1;
	config |= CONFIG_SET_NUMOFDATLANE(dsim->lanes - 1);
	config |= CONFIG_SET_LANEEN(0x1 | data_lanes_en << 1);

	dsim_write(dsim, config, DSIM_CONFIG);
}

static void sec_mipi_dsim_config_dphy(struct sec_mipi_dsim *dsim)
{
	uint32_t phytiming = 0, phytiming1 = 0, phytiming2 = 0, timeout = 0;

	/* TODO: add a PHY timing table arranged by the pll Fout */

	phytiming  |= PHYTIMING_SET_M_TLPXCTL(6)	|
		      PHYTIMING_SET_M_THSEXITCTL(11);
	dsim_write(dsim, phytiming, DSIM_PHYTIMING);

	phytiming1 |= PHYTIMING1_SET_M_TCLKPRPRCTL(7)	|
		      PHYTIMING1_SET_M_TCLKZEROCTL(38)	|
		      PHYTIMING1_SET_M_TCLKPOSTCTL(13)	|
		      PHYTIMING1_SET_M_TCLKTRAILCTL(8);
	dsim_write(dsim, phytiming1, DSIM_PHYTIMING1);

	phytiming2 |= PHYTIMING2_SET_M_THSPRPRCTL(8)	|
		      PHYTIMING2_SET_M_THSZEROCTL(13)	|
		      PHYTIMING2_SET_M_THSTRAILCTL(11);
	dsim_write(dsim, phytiming2, DSIM_PHYTIMING2);

	timeout |= TIMEOUT_SET_BTAOUT(0xf)	|
		   TIMEOUT_SET_LPDRTOUT(0xf);
	dsim_write(dsim, 0xf000f, DSIM_TIMEOUT);
}

static void sec_mipi_dsim_init_fifo_pointers(struct sec_mipi_dsim *dsim)
{
	uint32_t fifoctrl, fifo_ptrs;

	fifoctrl = dsim_read(dsim, DSIM_FIFOCTRL);

	fifo_ptrs = FIFOCTRL_NINITRX	|
		    FIFOCTRL_NINITSFR	|
		    FIFOCTRL_NINITI80	|
		    FIFOCTRL_NINITSUB	|
		    FIFOCTRL_NINITMAIN;

	fifoctrl &= ~fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);

	fifoctrl |= fifo_ptrs;
	dsim_write(dsim, fifoctrl, DSIM_FIFOCTRL);
	udelay(500);
}

static void sec_mipi_dsim_config_clkctrl(struct sec_mipi_dsim *dsim)
{
	uint32_t clkctrl = 0, data_lanes_en;
	uint64_t byte_clk, esc_prescaler;

	clkctrl |= CLKCTRL_TXREQUESTHSCLK;

	/* using 1.5Gbps PHY */
	clkctrl |= CLKCTRL_DPHY_SEL_1P5G;

	clkctrl |= CLKCTRL_ESCCLKEN;

	clkctrl &= ~CLKCTRL_PLLBYPASS;

	clkctrl |= CLKCTRL_BYTECLKSRC_DPHY_PLL;

	clkctrl |= CLKCTRL_BYTECLKEN;

	data_lanes_en = (0x1 << dsim->lanes) - 1;
	clkctrl |= CLKCTRL_SET_LANEESCCLKEN(0x1 | data_lanes_en << 1);

	/* calculate esc prescaler from byte clock:
	 * EscClk = ByteClk / EscPrescaler;
	 */
	byte_clk = dsim->bit_clk >> 3;
	esc_prescaler = DIV_ROUND_UP_ULL(byte_clk, MAX_ESC_CLK_FREQ);
	clkctrl |= CLKCTRL_SET_ESCPRESCALER(esc_prescaler);

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_mipi_dsim_set_standby(struct sec_mipi_dsim *dsim,
				      bool standby)
{
	uint32_t mdresol = 0;

	mdresol = dsim_read(dsim, DSIM_MDRESOL);

	if (standby)
		mdresol |= MDRESOL_MAINSTANDBY;
	else
		mdresol &= ~MDRESOL_MAINSTANDBY;

	dsim_write(dsim, mdresol, DSIM_MDRESOL);
}

static int sec_mipi_dsim_check_pll_out(struct sec_mipi_dsim *dsim,
				       const struct drm_display_mode *mode)
{
	int bpp;
	uint64_t pix_clk, bit_clk, ref_clk;
	const struct sec_mipi_dsim_plat_data *pdata = dsim->pdata;

	bpp = mipi_dsi_pixel_format_to_bpp(dsim->format);
	if (bpp < 0)
		return -EINVAL;

	pix_clk = mode->clock * 1000;
	bit_clk = DIV_ROUND_UP_ULL(pix_clk * bpp, dsim->lanes);

	if (bit_clk > pdata->max_data_rate) {
		dev_err(dsim->dev,
			"reuest bit clk freq exceeds lane's maximum value\n");
		return -EINVAL;
	}

	dsim->pix_clk = DIV_ROUND_UP_ULL(pix_clk, 1000);
	dsim->bit_clk = DIV_ROUND_UP_ULL(bit_clk, 1000);

	if (dsim->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		ref_clk = PHY_REF_CLK / 1000;
		/* TODO: add PMS calculate and check
		 * Only support '1080p@60Hz' for now,
		 * add other modes support later
		 */
		dsim->pms = 0x4210;
	}

	return 0;
}

static void sec_mipi_dsim_bridge_enable(struct drm_bridge *bridge)
{
	int ret;
	struct sec_mipi_dsim *dsim = bridge->driver_private;

	/* At this moment, the dsim bridge's preceding encoder has
	 * already been enabled. So the dsim can be configed here
	 */

	/* config main display mode */
	sec_mipi_dsim_set_main_mode(dsim);

	/* config dsim dpi */
	sec_mipi_dsim_config_dpi(dsim);

	/* config dsim pll */
	ret = sec_mipi_dsim_config_pll(dsim);
	if (ret) {
		dev_err(dsim->dev, "dsim pll config failed: %d\n", ret);
		return;
	}

	/* config dphy timings */
	sec_mipi_dsim_config_dphy(dsim);

	/* initialize FIFO pointers */
	sec_mipi_dsim_init_fifo_pointers(dsim);

	/* config esc clock, byte clock and etc */
	sec_mipi_dsim_config_clkctrl(dsim);

	/* enable data transfer of dsim */
	sec_mipi_dsim_set_standby(dsim, true);
}

static void sec_mipi_dsim_disable_clkctrl(struct sec_mipi_dsim *dsim)
{
	uint32_t clkctrl;

	clkctrl = dsim_read(dsim, DSIM_CLKCTRL);

	clkctrl &= ~CLKCTRL_TXREQUESTHSCLK;

	clkctrl &= ~CLKCTRL_ESCCLKEN;

	clkctrl &= ~CLKCTRL_BYTECLKEN;

	dsim_write(dsim, clkctrl, DSIM_CLKCTRL);
}

static void sec_mipi_dsim_disable_pll(struct sec_mipi_dsim *dsim)
{
	uint32_t pllctrl;

	pllctrl  = dsim_read(dsim, DSIM_PLLCTRL);

	pllctrl &= ~PLLCTRL_PLLEN;

	dsim_write(dsim, pllctrl, DSIM_PLLCTRL);
}

static void sec_mipi_dsim_bridge_disable(struct drm_bridge *bridge)
{
	struct sec_mipi_dsim *dsim = bridge->driver_private;

	/* disable data transfer of dsim */
	sec_mipi_dsim_set_standby(dsim, false);

	/* disable esc clock & byte clock */
	sec_mipi_dsim_disable_clkctrl(dsim);

	/* disable dsim pll */
	sec_mipi_dsim_disable_pll(dsim);
}

static bool sec_mipi_dsim_bridge_mode_fixup(struct drm_bridge *bridge,
					    const struct drm_display_mode *mode,
					    struct drm_display_mode *adjusted_mode)
{
	int ret, private_flags;
	struct sec_mipi_dsim *dsim = bridge->driver_private;

	ret = sec_mipi_dsim_check_pll_out(dsim, mode);
	if (ret)
		return false;

	/* Since mipi dsi cannot do color conversion,
	 * so the pixel format output by mipi dsi should
	 * be the same with the pixel format recieved by
	 * mipi dsi. And the pixel format information needs
	 * to be passed to CRTC to be checked with the CRTC
	 * attached plane fb pixel format.
	 */
	switch (dsim->format) {
	case MIPI_DSI_FMT_RGB888:
		private_flags = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case MIPI_DSI_FMT_RGB666:
		private_flags = MEDIA_BUS_FMT_RGB666_1X24_CPADHI;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		private_flags = MEDIA_BUS_FMT_RGB666_1X18;
		break;
	case MIPI_DSI_FMT_RGB565:
		private_flags = MEDIA_BUS_FMT_RGB565_1X16;
		break;
	default:
		return false;
	}

	adjusted_mode->private_flags = private_flags;

	/* the 'bus_flags' in connector's display_info is useless
	 * for mipi dsim, since dsim only sends packets with no
	 * polarities information in the packets. But the dsim
	 * host has some polarities requirements for the CRTC:
	 * dsim only can accpet active high Vsync, Hsync and DE
	 * signals.
	 */
	if (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC) {
		adjusted_mode->flags &= ~DRM_MODE_FLAG_NHSYNC;
		adjusted_mode->flags |= DRM_MODE_FLAG_PHSYNC;
	}

	if (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC) {
		adjusted_mode->flags &= ~DRM_MODE_FLAG_NVSYNC;
		adjusted_mode->flags |= DRM_MODE_FLAG_PVSYNC;
	}

	return true;
}

static void sec_mipi_dsim_bridge_mode_set(struct drm_bridge *bridge,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted_mode)
{
	struct sec_mipi_dsim *dsim = bridge->driver_private;

	/* This hook is called when the display pipe is completely
	 * off. And since the pm runtime is implemented, the dsim
	 * hardware cannot be accessed at this moment. So move all
	 * the mode_set config to ->enable() hook.
	 * And this hook is called only when 'mode_changed' is true,
	 * so it is called not every time atomic commit.
	 */

	drm_display_mode_to_videomode(adjusted_mode, &dsim->vmode);
}

static const struct drm_bridge_funcs sec_mipi_dsim_bridge_funcs = {
	.attach     = sec_mipi_dsim_bridge_attach,
	.enable     = sec_mipi_dsim_bridge_enable,
	.disable    = sec_mipi_dsim_bridge_disable,
	.mode_set   = sec_mipi_dsim_bridge_mode_set,
	.mode_fixup = sec_mipi_dsim_bridge_mode_fixup,
};

void sec_mipi_dsim_suspend(struct device *dev)
{
	struct sec_mipi_dsim *dsim = dev_get_drvdata(dev);

	/* TODO: add dsim reset */

	clk_disable_unprepare(dsim->clk_cfg);

	clk_disable_unprepare(dsim->clk_pllref);
}
EXPORT_SYMBOL(sec_mipi_dsim_suspend);

void sec_mipi_dsim_resume(struct device *dev)
{
	struct sec_mipi_dsim *dsim = dev_get_drvdata(dev);

	clk_prepare_enable(dsim->clk_pllref);

	clk_prepare_enable(dsim->clk_cfg);

	/* TODO: add dsim de-reset */
}
EXPORT_SYMBOL(sec_mipi_dsim_resume);

static void __maybe_unused sec_mipi_dsim_irq_mask(struct sec_mipi_dsim *dsim,
						  int irq_idx)
{
	uint32_t intmsk;

	intmsk = dsim_read(dsim, DSIM_INTMSK);

	switch (irq_idx) {
	case PLLSTABLE:
		intmsk |= INTMSK_MSKPLLSTABLE;
		break;
	case SWRSTRELEASE:
		intmsk |= INTMSK_MSKSWRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intmsk |= INTMSK_MSKSFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intmsk |= INTMSK_MSKSFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intmsk |= INTMSK_MSKFRAMEDONE;
		break;
	case LPDRTOUT:
		intmsk |= INTMSK_MSKLPDRTOUT;
		break;
	case TATOUT:
		intmsk |= INTMSK_MSKTATOUT;
		break;
	case RXDATDONE:
		intmsk |= INTMSK_MSKRXDATDONE;
		break;
	default:
		/* unsupported irq */
		return;
	}

	writel(intmsk, dsim->base + DSIM_INTMSK);
}

static void sec_mipi_dsim_irq_unmask(struct sec_mipi_dsim *dsim,
				     int irq_idx)
{
	uint32_t intmsk;

	intmsk = dsim_read(dsim, DSIM_INTMSK);

	switch (irq_idx) {
	case PLLSTABLE:
		intmsk &= ~INTMSK_MSKPLLSTABLE;
		break;
	case SWRSTRELEASE:
		intmsk &= ~INTMSK_MSKSWRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intmsk &= ~INTMSK_MSKSFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intmsk &= ~INTMSK_MSKSFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intmsk &= ~INTMSK_MSKFRAMEDONE;
		break;
	case LPDRTOUT:
		intmsk &= ~INTMSK_MSKLPDRTOUT;
		break;
	case TATOUT:
		intmsk &= ~INTMSK_MSKTATOUT;
		break;
	case RXDATDONE:
		intmsk &= ~INTMSK_MSKRXDATDONE;
		break;
	default:
		/* unsupported irq */
		return;
	}

	dsim_write(dsim, intmsk, DSIM_INTMSK);
}

/* write 1 clear irq */
static void sec_mipi_dsim_irq_clear(struct sec_mipi_dsim *dsim,
				    int irq_idx)
{
	uint32_t intsrc = 0;

	switch (irq_idx) {
	case PLLSTABLE:
		intsrc |= INTSRC_PLLSTABLE;
		break;
	case SWRSTRELEASE:
		intsrc |= INTSRC_SWRSTRELEASE;
		break;
	case SFRPLFIFOEMPTY:
		intsrc |= INTSRC_SFRPLFIFOEMPTY;
		break;
	case SFRPHFIFOEMPTY:
		intsrc |= INTSRC_SFRPHFIFOEMPTY;
		break;
	case FRAMEDONE:
		intsrc |= INTSRC_FRAMEDONE;
		break;
	case LPDRTOUT:
		intsrc |= INTSRC_LPDRTOUT;
		break;
	case TATOUT:
		intsrc |= INTSRC_TATOUT;
		break;
	case RXDATDONE:
		intsrc |= INTSRC_RXDATDONE;
		break;
	default:
		/* unsupported irq */
		return;
	}

	dsim_write(dsim, intsrc, DSIM_INTSRC);
}

static void sec_mipi_dsim_irq_init(struct sec_mipi_dsim *dsim)
{
	sec_mipi_dsim_irq_unmask(dsim, PLLSTABLE);
	sec_mipi_dsim_irq_unmask(dsim, SWRSTRELEASE);

	if (dsim->panel) {
		sec_mipi_dsim_irq_unmask(dsim, SFRPLFIFOEMPTY);
		sec_mipi_dsim_irq_unmask(dsim, SFRPHFIFOEMPTY);
		sec_mipi_dsim_irq_unmask(dsim, LPDRTOUT);
		sec_mipi_dsim_irq_unmask(dsim, TATOUT);
		sec_mipi_dsim_irq_unmask(dsim, RXDATDONE);
	}
}

static irqreturn_t sec_mipi_dsim_irq_handler(int irq, void *data)
{
	uint32_t intsrc, status;
	struct sec_mipi_dsim *dsim = data;

	intsrc = dsim_read(dsim, DSIM_INTSRC);
	status = dsim_read(dsim, DSIM_STATUS);

	if (WARN_ON(!intsrc)) {
		dev_err(dsim->dev, "interrupt is not from dsim\n");
		return IRQ_NONE;
	}

	if (WARN_ON(!(intsrc & INTSRC_MASK))) {
		dev_warn(dsim->dev, "unenable irq happens: %#x\n", intsrc);
		/* just clear irqs */
		dsim_write(dsim, intsrc, DSIM_INTSRC);
		return IRQ_NONE;
	}

	if (intsrc & INTSRC_PLLSTABLE) {
		WARN_ON(!(status & STATUS_PLLSTABLE));
		sec_mipi_dsim_irq_clear(dsim, PLLSTABLE);
		complete(&dsim->pll_stable);
	}

	if (intsrc & INTSRC_SWRSTRELEASE)
		sec_mipi_dsim_irq_clear(dsim, SWRSTRELEASE);

	if (intsrc & INTSRC_SFRPLFIFOEMPTY)
		sec_mipi_dsim_irq_clear(dsim, SFRPLFIFOEMPTY);

	if (intsrc & INTSRC_SFRPHFIFOEMPTY)
		sec_mipi_dsim_irq_clear(dsim, SFRPHFIFOEMPTY);

	if (intsrc & INTSRC_LPDRTOUT)
		sec_mipi_dsim_irq_clear(dsim, LPDRTOUT);

	if (intsrc & INTSRC_TATOUT)
		sec_mipi_dsim_irq_clear(dsim, TATOUT);

	if (intsrc & INTSRC_RXDATDONE)
		sec_mipi_dsim_irq_clear(dsim, RXDATDONE);

	return IRQ_HANDLED;
}

static int sec_mipi_dsim_connector_get_modes(struct drm_connector *connector)
{
	/* TODO: add support later */

	return 0;
}

static const struct drm_connector_helper_funcs
	sec_mipi_dsim_connector_helper_funcs = {
	.get_modes = sec_mipi_dsim_connector_get_modes,
};

static enum drm_connector_status
	sec_mipi_dsim_connector_detect(struct drm_connector *connector,
				       bool force)
{
	/* TODO: add support later */

	return connector_status_connected;
}

static const struct drm_connector_funcs sec_mipi_dsim_connector_funcs = {
	.detect     = sec_mipi_dsim_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy    = drm_connector_cleanup,
	.reset      = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};

int sec_mipi_dsim_bind(struct device *dev, struct device *master, void *data,
		       struct drm_encoder *encoder, struct resource *res,
		       int irq, const struct sec_mipi_dsim_plat_data *pdata)
{
	int ret, version;
	struct drm_device *drm_dev = data;
	struct drm_bridge *bridge;
	struct drm_connector *connector;
	struct sec_mipi_dsim *dsim;

	dev_dbg(dev, "sec-dsim bridge bind begin\n");

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim) {
		dev_err(dev, "Unable to allocate 'dsim'\n");
		return -ENOMEM;
	}

	dsim->dev = dev;
	dsim->irq = irq;
	dsim->pdata = pdata;
	dsim->encoder = encoder;

	dsim->dsi_host.ops = &sec_mipi_dsim_host_ops;
	dsim->dsi_host.dev = dev;

	dsim->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsim->base))
		return PTR_ERR(dsim->base);

	dsim->clk_pllref = devm_clk_get(dev, "pll-ref");
	if (IS_ERR(dsim->clk_pllref)) {
		ret = PTR_ERR(dsim->clk_pllref);
		dev_err(dev, "Unable to get phy pll reference clock: %d\n", ret);
		return ret;
	}

	dsim->clk_cfg = devm_clk_get(dev, "cfg");
	if (IS_ERR(dsim->clk_cfg)) {
		ret = PTR_ERR(dsim->clk_cfg);
		dev_err(dev, "Unable to get configuration clock: %d\n", ret);
		return ret;
	}

	clk_prepare_enable(dsim->clk_cfg);
	version = dsim_read(dsim, DSIM_VERSION);
	WARN_ON(version != pdata->version);
	clk_disable_unprepare(dsim->clk_cfg);

	dev_info(dev, "version number is %#x\n", version);

	/* TODO: set pll ref clock rate to be fixed with 27MHz */
	ret = clk_set_rate(dsim->clk_pllref, PHY_REF_CLK);
	if (ret) {
		dev_err(dev, "failed to set pll ref clock rate\n");
		return ret;
	}

	ret = devm_request_irq(dev, dsim->irq,
			       sec_mipi_dsim_irq_handler,
			       0, dev_name(dev), dsim);
	if (ret) {
		dev_err(dev, "failed to request dsim irq: %d\n", ret);
		return ret;
	}

	init_completion(&dsim->pll_stable);

	/* Initialize and attach sec dsim bridge */
	bridge = devm_kzalloc(dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge) {
		dev_err(dev, "Unable to allocate 'bridge'\n");
		return -ENOMEM;
	}

	/* mipi dsi host needs to be registered before bridge attach, since:
	 * 1. Have Panel
	 *    The 'mipi_dsi_host_register' will allocate a mipi_dsi_device
	 *    if the dsi host node has a panel child node in DTB. And dsi
	 *    host ->attach() will be called in panel's probe().
	 *
	 * 2. Have Bridge
	 *    The dsi host ->attach() will be called through the below
	 *    'drm_bridge_attach()' which will attach next bridge in a
	 *    chain.
	 */
	ret = mipi_dsi_host_register(&dsim->dsi_host);
	if (ret) {
		dev_err(dev, "Unable to register mipi dsi host: %d\n", ret);
		return ret;
	}

	dsim->bridge = bridge;
	bridge->driver_private = dsim;
	bridge->funcs = &sec_mipi_dsim_bridge_funcs;
	bridge->of_node = dev->of_node;
	bridge->encoder = encoder;
	encoder->bridge = bridge;

	dev_set_drvdata(dev, dsim);

	/* attach sec dsim bridge and its next bridge if exists */
	ret = drm_bridge_attach(encoder, bridge, NULL);
	if (ret) {
		dev_err(dev, "Failed to attach bridge: %s\n", dev_name(dev));
		mipi_dsi_host_unregister(&dsim->dsi_host);
		return ret;
	}

	if (dsim->panel) {
		/* A panel has been attached */
		connector = &dsim->connector;

		drm_connector_helper_add(connector,
					 &sec_mipi_dsim_connector_helper_funcs);
		ret = drm_connector_init(drm_dev, connector,
					 &sec_mipi_dsim_connector_funcs,
					 DRM_MODE_CONNECTOR_DSI);
		if (ret)
			goto host_unregister;

		/* TODO */
		connector->dpms = DRM_MODE_DPMS_OFF;

		ret = drm_mode_connector_attach_encoder(connector, encoder);
		if (ret)
			goto cleanup_connector;

		ret = drm_panel_attach(dsim->panel, connector);
		if (ret)
			goto cleanup_connector;
	}

	sec_mipi_dsim_irq_init(dsim);
	dev_dbg(dev, "sec-dsim bridge bind end\n");

	return 0;

cleanup_connector:
	drm_connector_cleanup(connector);
host_unregister:
	mipi_dsi_host_unregister(&dsim->dsi_host);
	return ret;
}
EXPORT_SYMBOL(sec_mipi_dsim_bind);

void sec_mipi_dsim_unbind(struct device *dev, struct device *master, void *data)
{
	struct sec_mipi_dsim *dsim = dev_get_drvdata(dev);

	if (dsim->panel) {
		drm_panel_detach(dsim->panel);
		drm_connector_cleanup(&dsim->connector);
		dsim->panel = NULL;
	}

	mipi_dsi_host_unregister(&dsim->dsi_host);
}
EXPORT_SYMBOL(sec_mipi_dsim_unbind);

MODULE_DESCRIPTION("Samsung MIPI DSI Host Controller bridge driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
