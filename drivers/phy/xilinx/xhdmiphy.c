// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xilinx HDMI PHY
 *
 * Copyright (C) 2021 Xilinx, Inc.
 *
 * Author: Rajesh Gugulothu <gugulothu.rajesh@xilinx.com>
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-hdmi.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include "xhdmiphy.h"

struct xhdmiphy_lane {
	struct phy *phy;
	u32 share_laneclk;
	u8 direction;
	u8 lane;
	void *data;
};

static int xhdmiphy_init(struct phy *phy)
{
	struct xhdmiphy_lane *phy_lane = phy_get_drvdata(phy);
	struct xhdmiphy_dev *phy_dev = phy_lane->data;
	unsigned int ret;
	static int count;

	count++;

	if (count < (XHDMIPHY_MAX_LANES - 4))
		return 0;

	/* initialize HDMI phy */
	ret = xhdmiphy_init_phy(phy_dev);
	if (ret != 0) {
		dev_err(phy_dev->dev, "HDMI PHY initialization error\n");
		return -ENODEV;
	}
	count = 0;

	return 0;
}

static int xhdmiphy_reset(struct phy *phy)
{
	struct xhdmiphy_lane *phy_lane = phy_get_drvdata(phy);
	struct xhdmiphy_dev *phy_dev = phy_lane->data;

	if (!phy_lane->direction)
		xhdmiphy_ibufds_en(phy_dev, XHDMIPHY_DIR_TX, false);

	return 0;
}

static int xhdmiphy_clk_srcsel(struct xhdmiphy_dev *priv, u8 dir, u8 clksrc)
{
	if (priv->data && !priv->data->sel_mux(dir, clksrc))
		return 0;

	dev_dbg(priv->dev, "failed to select clock source\n");

	return -EIO;
}

static int xhdmiphy_set_lrate(struct xhdmiphy_dev *priv, u8 dir, u8 mode,
			      u64 lrate, u8 lanes)
{
	if (priv->data && !priv->data->set_linerate(dir, mode, lrate, lanes))
		return 0;

	dev_dbg(priv->dev, "failed to set linerate\n");

	return -EIO;
}

static int xhdmiphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct xhdmiphy_lane *phy_lane = phy_get_drvdata(phy);
	struct xhdmiphy_dev *phy_dev = phy_lane->data;
	struct phy_configure_opts_hdmi *cfg = &opts->hdmi;
	struct hdmiphy_callback *cb_ptr = &cfg->hdmiphycb;
	unsigned int ret = 0, chid;
	static int count_tx, count_rx;

	if (!phy_lane->direction) {
		count_rx++;
		if (count_rx < phy_dev->conf.rx_channels) {
			return 0;
		} else if (cfg->ibufds) {
			xhdmiphy_ibufds_en(phy_dev, XHDMIPHY_DIR_RX,
					   cfg->ibufds_en);
			cfg->ibufds = 0;
		} else if (cfg->tmdsclock_ratio_flag) {
			/* update TMDS clock ratio */
			phy_dev->rx_tmdsclock_ratio = cfg->tmdsclock_ratio;
			cfg->tmdsclock_ratio_flag = 0;
		} else if (cfg->phycb) {
			switch (cb_ptr->type) {
			case RX_INIT_CB:
				phy_dev->phycb[RX_INIT_CB].cb =
							cfg->hdmiphycb.cb;
				phy_dev->phycb[RX_INIT_CB].data =
							cfg->hdmiphycb.data;
				break;
			case RX_READY_CB:
				phy_dev->phycb[RX_READY_CB].cb =
							cfg->hdmiphycb.cb;
				phy_dev->phycb[RX_READY_CB].data =
							cfg->hdmiphycb.data;
				break;
			default:
				dev_info(phy_dev->dev,
					 "type - %d phy callback does't match\n\r",
					 cb_ptr->type);
				break;
			}
			cfg->phycb = 0;
		} else if (cfg->cal_mmcm_param) {
			ret = xhdmiphy_cal_mmcm_param(phy_dev,
						      XHDMIPHY_CHID_CH1,
						      XHDMIPHY_DIR_RX, cfg->ppc,
						      cfg->bpc);
			if (ret)
				dev_err(phy_dev->dev,
					"failed to update mmcm params\n\r");

			xhdmiphy_mmcm_start(phy_dev, XHDMIPHY_DIR_RX);
			cfg->cal_mmcm_param = 0;
		} else if (cfg->clkout1_obuftds) {
			xhdmiphy_clkout1_obuftds_en(phy_dev, XHDMIPHY_DIR_RX,
						    cfg->clkout1_obuftds_en);
			cfg->clkout1_obuftds_en = 0;
		} else if (cfg->config_hdmi20 && !cfg->config_hdmi21) {
			/* set Rx ch4 as clock */
			gpiod_set_value(phy_dev->rxch4_gpio, 0);
			xhdmiphy_hdmi20_conf(phy_dev, XHDMIPHY_DIR_RX);
			xhdmiphy_clk_srcsel(phy_dev, phy_lane->direction,
					    tmds_mode);
			xhdmiphy_set_lrate(phy_dev, phy_lane->direction, 0,
					   cfg->rx_refclk_hz, 0);
			cfg->config_hdmi20 = 0;
		} else if (!cfg->config_hdmi20 && cfg->config_hdmi21) {
			/*
			 * Phy needs to switch between rxch4 as data or
			 * clk based on FRL or TMDS mode. Note we can have TMDS
			 * mode in HDMI2.1
			 */
			gpiod_set_value(phy_dev->rxch4_gpio, 1);
			if (phy_dev->conf.rx_refclk_sel !=
			    phy_dev->conf.rx_frl_refclk_sel) {
				xhdmiphy_ibufds_en(phy_dev, XHDMIPHY_DIR_RX, 1);
			}

			xhdmiphy_hdmi21_conf(phy_dev, XHDMIPHY_DIR_RX,
					     cfg->linerate, cfg->nchannels);
			xhdmiphy_clk_srcsel(phy_dev, phy_lane->direction,
					    frl_mode);
			xhdmiphy_clkdet_freq_reset(phy_dev, XHDMIPHY_DIR_RX);
			xhdmiphy_set_lrate(phy_dev, phy_lane->direction, 1,
					   cfg->linerate, cfg->nchannels);
			cfg->config_hdmi21 = 0;
		} else if (cfg->rx_get_refclk) {
			cfg->rx_refclk_hz = phy_dev->rx_refclk_hz;
			cfg->rx_get_refclk = 0;
		} else if (cfg->reset_gt) {
			xhdmiphy_rst_gt_txrx(phy_dev, XHDMIPHY_CHID_CHA,
					     XHDMIPHY_DIR_RX, false);
			cfg->reset_gt = 0;
		}
		count_rx = 0;
	}

	if (phy_lane->direction) {
		count_tx++;

		if (count_tx < phy_dev->conf.tx_channels) {
			return 0;
		} else if (cfg->ibufds) {
			xhdmiphy_ibufds_en(phy_dev, XHDMIPHY_DIR_TX,
					   cfg->ibufds_en);
			cfg->ibufds = 0;
		} else if (cfg->config_hdmi20) {
			xhdmiphy_hdmi20_conf(phy_dev, XHDMIPHY_DIR_TX);
			cfg->config_hdmi20 = 0;
		} else if (cfg->get_samplerate) {
			cfg->samplerate = phy_dev->tx_samplerate;
			cfg->get_samplerate = 0;
		} else if (cfg->clkout1_obuftds) {
			xhdmiphy_clkout1_obuftds_en(phy_dev, XHDMIPHY_DIR_TX,
						    cfg->clkout1_obuftds_en);
			cfg->clkout1_obuftds_en = 0;
		} else if (cfg->tx_params) {
			xhdmiphy_clk_srcsel(phy_dev, phy_lane->direction,
					    tmds_mode);
			usleep_range(1000, 1100);
			phy_dev->tx_refclk_hz = cfg->tx_tmdsclk;

			if (phy_dev->conf.gt_type == XHDMIPHY_GTYE5 ||
			    phy_dev->conf.gt_type == XHDMIPHY_GTYP) {
				chid = XHDMIPHY_CHID_CMNA;
			} else {
				chid = XHDMIPHY_CHID_CHA;
			}

			ret = xhdmiphy_set_tx_param(phy_dev, chid, cfg->ppc,
						    cfg->bpc, cfg->fmt);
			if (ret)
				dev_err(phy_dev->dev,
					"unable to set requested tx resolutions\n\r");
			cfg->tx_params = 0;
			clk_set_rate(phy_dev->tmds_clk, phy_dev->tx_refclk_hz);
			dev_info(phy_dev->dev,
				 "tx_tmdsclk %lld\n", cfg->tx_tmdsclk);
			xhdmiphy_set_lrate(phy_dev, phy_lane->direction, 0,
					   cfg->tx_tmdsclk, 0);
		} else if (cfg->config_hdmi21) {
			if (phy_dev->conf.tx_refclk_sel !=
			    phy_dev->conf.tx_frl_refclk_sel) {
				xhdmiphy_ibufds_en(phy_dev, XHDMIPHY_DIR_TX, 1);
			}
			xhdmiphy_hdmi21_conf(phy_dev, XHDMIPHY_DIR_TX,
					     cfg->linerate, cfg->nchannels);
			if (phy_dev->conf.tx_refclk_sel ==
			    phy_dev->conf.tx_frl_refclk_sel) {
				xhdmiphy_clk_srcsel(phy_dev, phy_lane->direction,
						    frl_mode);
				xhdmiphy_clkdet_freq_reset(phy_dev,
							   XHDMIPHY_DIR_TX);
			}
			xhdmiphy_set_lrate(phy_dev, phy_lane->direction,
					   1, cfg->linerate,
					   cfg->nchannels);
			cfg->config_hdmi21 = 0;
		} else if (cfg->resetgtpll) {
			xhdmiphy_set(phy_dev, XHDMIPHY_TX_INIT_REG,
				     XHDMIPHY_TXRX_INIT_PLLGTRESET_ALL_MASK);
			xhdmiphy_clr(phy_dev, XHDMIPHY_TX_INIT_REG,
				     XHDMIPHY_TXRX_INIT_PLLGTRESET_ALL_MASK);
		}
		count_tx = 0;
	}

	return 0;
}

static const struct phy_ops xhdmiphy_phyops = {
	.configure	= xhdmiphy_configure,
	.reset		= xhdmiphy_reset,
	.init		= xhdmiphy_init,
	.owner		= THIS_MODULE,
};

static struct phy *xhdmiphy_xlate(struct device *dev,
				  const struct of_phandle_args *args)
{
	struct xhdmiphy_dev *priv = dev_get_drvdata(dev);
	struct xhdmiphy_lane *hdmiphy_lane = NULL;
	struct device_node *hdmiphynode = args->np;
	int index;

	if (args->args_count != 4) {
		dev_err(dev, "Invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}

	if (!of_device_is_available(hdmiphynode)) {
		dev_warn(dev, "requested PHY is disabled\n");
		return ERR_PTR(-ENODEV);
	}

	for (index = 0; index < of_get_child_count(dev->of_node); index++) {
		if (hdmiphynode == priv->lanes[index]->phy->dev.of_node) {
			hdmiphy_lane = priv->lanes[index];
			break;
		}
	}

	if (!hdmiphy_lane) {
		dev_err(dev, "failed to find appropriate phy\n");
		return ERR_PTR(-EINVAL);
	}

	hdmiphy_lane->share_laneclk = args->args[2];
	hdmiphy_lane->direction = args->args[3];

	return hdmiphy_lane->phy;
}

static irqreturn_t xhdmiphy_irq_handler(int irq, void *dev_id)
{
	struct xhdmiphy_dev *priv;

	priv = (struct xhdmiphy_dev *)dev_id;
	if (!priv)
		return IRQ_NONE;

	/*
	 * disable interrupts in the HDMI PHY, they are re-enabled once
	 * serviced
	 */
	if (priv->conf.gt_type == XHDMIPHY_GTYE5 ||
	    priv->conf.gt_type == XHDMIPHY_GTYP) {
		xhdmiphy_intr_dis(priv, XHDMIPHY_GTYE5_TX_ALL_MASK |
				  XHDMIPHY_GTYE5_RX_ALL_MASK);
	} else {
		xhdmiphy_intr_dis(priv, XHDMIPHY_INTR_ALL_MASK);
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t xhdmiphy_irq_thread(int irq, void *dev_id)
{
	struct xhdmiphy_dev *priv;
	u32 status;
	u32 event_mask;
	u32 event_ack;

	priv = (struct xhdmiphy_dev *)dev_id;
	if (!priv)
		return IRQ_NONE;

	/* call baremetal interrupt handler with mutex locked */
	mutex_lock(&priv->hdmiphy_mutex);

	status = xhdmiphy_read(priv, XHDMIPHY_INTR_STS_REG);
	dev_dbg(priv->dev, "xhdmiphy status = %x\n", status);

	if (priv->conf.gt_type != XHDMIPHY_GTYE5 &&
	    priv->conf.gt_type != XHDMIPHY_GTYP) {
		event_mask = XHDMIPHY_INTR_QPLL0_LOCK_MASK |
			     XHDMIPHY_INTR_CPLL_LOCK_MASK |
			     XHDMIPHY_INTR_QPLL1_LOCK_MASK |
			     XHDMIPHY_INTR_TXALIGNDONE_MASK |
			     XHDMIPHY_INTR_TXRESETDONE_MASK |
			     XHDMIPHY_INTR_RXRESETDONE_MASK |
			     XHDMIPHY_INTR_TXMMCMUSRCLK_LOCK_MASK |
			     XHDMIPHY_INTR_RXMMCMUSRCLK_LOCK_MASK;
	} else {
		if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_TX)
			event_mask = XHDMIPHY_GTYE5_TX_MASK;
		else if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_RX)
			event_mask = XHDMIPHY_GTYE5_RX_MASK;
		else
			event_mask = (XHDMIPHY_GTYE5_TX_MASK |
				      XHDMIPHY_GTYE5_RX_MASK);
	}

	event_ack = event_mask & status;
	if (event_ack)
		xhdmiphy_gt_handler(priv, event_ack, status);

	if ((priv->conf.gt_type != XHDMIPHY_GTYE5 &&
	     priv->conf.gt_type != XHDMIPHY_GTYP) ||
	    priv->conf.gt_direction == XHDMIPHY_DUPLEX) {
		event_mask =	XHDMIPHY_INTR_TXFREQCHANGE_MASK |
				XHDMIPHY_INTR_RXFREQCHANGE_MASK |
				XHDMIPHY_INTR_TXTMRTIMEOUT_MASK |
				XHDMIPHY_INTR_RXTMRTIMEOUT_MASK;
	} else {
		if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_TX)
			event_mask =	XHDMIPHY_INTR_TXTMRTIMEOUT_MASK |
					XHDMIPHY_INTR_TXFREQCHANGE_MASK;
		else if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_RX)
			event_mask =	XHDMIPHY_INTR_RXFREQCHANGE_MASK |
					XHDMIPHY_INTR_RXTMRTIMEOUT_MASK;
		else
			event_mask =	XHDMIPHY_INTR_RXFREQCHANGE_MASK |
					XHDMIPHY_INTR_RXTMRTIMEOUT_MASK |
					XHDMIPHY_INTR_TXTMRTIMEOUT_MASK |
					XHDMIPHY_INTR_TXFREQCHANGE_MASK;
	}

	event_ack = event_mask & status;
	if (event_ack)
		xhdmiphy_clkdet_handler(priv, event_ack, status);

	mutex_unlock(&priv->hdmiphy_mutex);

	/* enable interrupt requesting in the PHY */
	if (priv->conf.gt_type == XHDMIPHY_GTYE5 ||
	    priv->conf.gt_type == XHDMIPHY_GTYP) {
		if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_TX)
			xhdmiphy_intr_en(priv, XHDMIPHY_GTYE5_TX_ALL_MASK);
		else if (priv->conf.gt_direction == XHDMIPHY_SIMPLE_RX)
			xhdmiphy_intr_en(priv, XHDMIPHY_GTYE5_RX_ALL_MASK);
		else
			xhdmiphy_intr_en(priv, XHDMIPHY_GTYE5_TX_ALL_MASK |
					 XHDMIPHY_GTYE5_RX_ALL_MASK);
	} else {
		xhdmiphy_intr_en(priv, XHDMIPHY_INTR_ALL_MASK);
	}

	return IRQ_HANDLED;
}

static int xhdmiphy_parse_of(struct xhdmiphy_dev *priv)
{
	struct xhdmiphy_conf *xgtphycfg = &priv->conf;
	struct device *dev = priv->dev;
	struct device_node *node = dev->of_node;
	int rc, val;

	rc = of_property_read_u32(node, "xlnx,transceiver-type", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,transceiver-type");
		return rc;
	}

	if (val != XHDMIPHY_GTHE4 && val != XHDMIPHY_GTYE4 &&
	    val != XHDMIPHY_GTYE5 && val != XHDMIPHY_GTYP) {
		dev_err(priv->dev, "dt transceiver-type %d is invalid\n", val);
		return -EINVAL;
	}
	xgtphycfg->gt_type = val;

	rc = of_property_read_u32(node, "xlnx,input-pixels-per-clock", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,input-pixels-per-clock");
		return rc;
	}

	if (val != 4 && val != 8) {
		dev_err(priv->dev, "dt input-pixels-per-clock %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->ppc = val;

	rc = of_property_read_u32(node, "xlnx,nidru", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,nidru");
		return rc;
	}

	if (val != 0 && val != 1) {
		dev_err(priv->dev, "dt nidru %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->dru_present = val;

	rc = of_property_read_u32(node, "xlnx,nidru-refclk-sel", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,nidru-refclk-sel");
		return rc;
	}

	if (val < XHDMIPHY_PLL_REFCLKSEL_GTREFCLK0 - 1 &&
	    val > XHDMIPHY_PLL_REFCLKSEL_GTGREFCLK - 1) {
		dev_err(priv->dev, "dt nidru-refclk-sel %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->dru_refclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,rx-no-of-channels", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-no-of-channels");
		return rc;
	}

	if (val != 1 && val != 2 && val != 4) {
		dev_err(priv->dev, "dt rx-no-of-channels %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_channels = val;

	rc = of_property_read_u32(node, "xlnx,tx-no-of-channels", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-no-of-channels");
		return rc;
	}

	if (val != 1 && val != 2 && val != 4) {
		dev_err(priv->dev, "dt tx-no-of-channels %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_channels = val;

	rc = of_property_read_u32(node, "xlnx,rx-protocol", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-protocol");
		return rc;
	}

	if (val != XHDMIPHY_PROT_HDMI && val != XHDMIPHY_PROT_HDMI21 &&
	    val != XHDMIPHY_PROT_NONE) {
		dev_err(priv->dev, "dt rx-protocol %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_protocol = val;

	rc = of_property_read_u32(node, "xlnx,tx-protocol", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-protocol");
		return rc;
	}

	if (val != XHDMIPHY_PROT_HDMI && val != XHDMIPHY_PROT_HDMI21 &&
	    val != XHDMIPHY_PROT_NONE) {
		dev_err(priv->dev, "dt tx-protocol %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_protocol = val;

	rc = of_property_read_u32(node, "xlnx,rx-refclk-sel", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-refclk-sel");
		return rc;
	}

	if (val < XHDMIPHY_PLL_REFCLKSEL_GTREFCLK0 - 1 &&
	    val > XHDMIPHY_PLL_REFCLKSEL_GTGREFCLK - 1) {
		dev_err(priv->dev, "dt rx-refclk-sel %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_refclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,tx-refclk-sel", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-refclk-sel");
		return rc;
	}

	if (val < XHDMIPHY_PLL_REFCLKSEL_GTREFCLK0 - 1 &&
	    val > XHDMIPHY_PLL_REFCLKSEL_GTGREFCLK - 1) {
		dev_err(priv->dev, "dt tx-refclk-sel %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_refclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,rx-pll-selection", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-pll-selection");
		return rc;
	}

	if (xgtphycfg->gt_type == XHDMIPHY_GTYE5 ||
	    xgtphycfg->gt_type == XHDMIPHY_GTYP) {
		if (val != 7 && val != 8) {
			dev_err(priv->dev, "dt rx-pll-selection %d is invalid\n",
				val);
			return -EINVAL;
		}
	} else {
		if (val < 0 || val > 6) {
			dev_err(priv->dev, "dt rx-pll-selection %d is invalid\n",
				val);
			return -EINVAL;
		}
	}
	xgtphycfg->rx_pllclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,tx-pll-selection", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-pll-selection");
		return rc;
	}

	if (xgtphycfg->gt_type == XHDMIPHY_GTYE5 ||
	    xgtphycfg->gt_type == XHDMIPHY_GTYP) {
		if (val != 7 && val != 8) {
			dev_err(priv->dev, "dt tx-pll-selection %d is invalid\n",
				val);
			return -EINVAL;
		}
	} else {
		if (val < 0 || val > 6) {
			dev_err(priv->dev, "dt tx-pll-selection %d is invalid\n",
				val);
			return -EINVAL;
		}
	}
	xgtphycfg->tx_pllclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,transceiver-width", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,transceiver-width");
		return rc;
	}
	if (val != 2 && val != 4) {
		dev_err(priv->dev, "dt transceiver-width %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->transceiver_width = val;

	rc = of_property_read_u32(node, "xlnx,rx-max-gt-line-rate", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-max-gt-line-rate");
		return rc;
	}

	if (val != 3 && val != 6 && val != 8 && val != 10 && val != 12) {
		dev_err(priv->dev, "dt rx-max-gt-line-rate %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_maxrate = val;

	rc = of_property_read_u32(node, "xlnx,tx-max-gt-line-rate", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-max-gt-line-rate");
		return rc;
	}

	if (val != 3 && val != 6 && val != 8 && val != 10 && val != 12) {
		dev_err(priv->dev, "dt tx-max-gt-line-rate %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_maxrate = val;

	rc = of_property_read_u32(node, "xlnx,rx-clk-primitive", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-clk-primitive. Make MMCM as default value");
		val = XHDMIPHY_MMCM;
	}

	if (val != XHDMIPHY_MMCM && val != XHDMIPHY_PLL) {
		dev_err(priv->dev, "dt xlnx,rx-clk-primitive %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_clk_primitive = val;

	rc = of_property_read_u32(node, "xlnx,tx-clk-primitive", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-clk-primitive. make MMCM as default value");
		val = XHDMIPHY_MMCM;
	}

	if (val != XHDMIPHY_MMCM && val != XHDMIPHY_PLL) {
		dev_err(priv->dev, "dt xlnx,tx-clk-primitive %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_clk_primitive = val;

	rc = of_property_read_u32(node, "xlnx,use-gt-ch4-hdmi", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,use-gt-ch4-hdmi");
		return rc;
	}

	if (val != 0 && val != 1) {
		dev_err(priv->dev, "dt use-gt-ch4-hdmi %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->gt_as_tx_tmdsclk = val;

	rc = of_property_read_u32(node, "xlnx,rx-frl-refclk-sel", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,rx-frl-refclk-sel");
		return rc;
	}

	if (val < XHDMIPHY_PLL_REFCLKSEL_GTREFCLK0 - 1 &&
	    val > XHDMIPHY_PLL_REFCLKSEL_GTGREFCLK - 1) {
		dev_err(priv->dev, "dt rx-frl-refclk-sel %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->rx_frl_refclk_sel = val;

	rc = of_property_read_u32(node, "xlnx,tx-frl-refclk-sel", &val);
	if (rc < 0) {
		dev_err(priv->dev, "unable to parse %s property\n",
			"xlnx,tx-frl-refclk-sel");
		return rc;
	}

	if (val < XHDMIPHY_PLL_REFCLKSEL_GTREFCLK0 - 1 &&
	    val > XHDMIPHY_PLL_REFCLKSEL_GTGREFCLK - 1) {
		dev_err(priv->dev, "dt tx-frl-refclk-sel %d is invalid\n",
			val);
		return -EINVAL;
	}
	xgtphycfg->tx_frl_refclk_sel = val;

	priv->rxch4_gpio = devm_gpiod_get_optional(priv->dev,
						   "rxch4-sel", GPIOD_OUT_LOW);

	if (IS_ERR(priv->rxch4_gpio)) {
		if (PTR_ERR(priv->rxch4_gpio) != -EPROBE_DEFER)
			dev_err(priv->dev, "rxch4-sel not setup in DT\n");
		return PTR_ERR(priv->rxch4_gpio);
	}

	if (xgtphycfg->gt_type == XHDMIPHY_GTYE5 ||
	    xgtphycfg->gt_type == XHDMIPHY_GTYP) {
		/* GTYE5 & GTYP supports SIMPLE_TX, SIMPLE_RX and DUPLEX mode */
		rc = of_property_read_u32(node, "xlnx,gt-direction", &val);
		if (rc < 0) {
			dev_err(priv->dev, "unable to parse %s property\n",
				"xlnx,gt-direction");
			return rc;
		}

		if (val != XHDMIPHY_SIMPLE_TX &&
		    val != XHDMIPHY_SIMPLE_RX &&
		    val != XHDMIPHY_DUPLEX) {
			dev_err(priv->dev, "Invalid gt-direction %d\n", val);
			return -EINVAL;
		}
		xgtphycfg->gt_direction = val;
	}

	return rc;
}

static int xhdmiphy_clk_init(struct xhdmiphy_dev *priv)
{
	unsigned long dru_clk_rate = 0;
	int err;

	priv->axi_lite_clk = devm_clk_get(priv->dev, "vid_phy_axi4lite_aclk");
	if (IS_ERR(priv->axi_lite_clk))
		return dev_err_probe(priv->dev, PTR_ERR(priv->axi_lite_clk),
				     "failed to get vid_phy_axi4lite_aclk\n");

	priv->tmds_clk = devm_clk_get(priv->dev, "tmds_clock");
	if (IS_ERR(priv->tmds_clk))
		return dev_err_probe(priv->dev, PTR_ERR(priv->tmds_clk),
				     "failed to get tmds_clock\n");

	if (priv->conf.dru_present) {
		priv->dru_clk = devm_clk_get(priv->dev, "drpclk");
		if (IS_ERR(priv->dru_clk))
			return dev_err_probe(priv->dev, PTR_ERR(priv->dru_clk),
					     "failed to get drpclk\n");
	} else {
		dev_dbg(priv->dev, "DRU is not enabled from device tree\n");
	}

	err = clk_prepare_enable(priv->axi_lite_clk);
	if (err) {
		dev_err(priv->dev,
			"failed to enable axi-lite clk (%d)\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->tmds_clk);
	if (err) {
		dev_err(priv->dev, "failed to enable tmds_clk (%d)\n", err);
		goto err_disable_axiclk;
	}

	if (priv->conf.dru_present) {
		err = clk_prepare_enable(priv->dru_clk);
		if (err) {
			dev_err(priv->dev,
				"failed to enable nidru clk (%d)\n", err);
			goto err_disable_tmds_clk;
		}

		dru_clk_rate = clk_get_rate(priv->dru_clk);
		dev_dbg(priv->dev, "default dru-clk rate = %lu\n",
			dru_clk_rate);
		if (dru_clk_rate != XHDMIPHY_DRU_REF_CLK_HZ) {
			err = clk_set_rate(priv->dru_clk,
					   XHDMIPHY_DRU_REF_CLK_HZ);
			if (err) {
				dev_err(priv->dev,
					"Cannot set rate : %d\n", err);
				goto err_disable_dru_clk;
			}
			dru_clk_rate = clk_get_rate(priv->dru_clk);
			dev_dbg(priv->dev,
				"ref dru-clk rate = %lu\n", dru_clk_rate);
		}
	}

	priv->conf.drpclk_freq = dru_clk_rate;
	priv->conf.axilite_freq = clk_get_rate(priv->axi_lite_clk);

	return 0;

err_disable_dru_clk:
	clk_disable_unprepare(priv->dru_clk);
err_disable_tmds_clk:
	clk_disable_unprepare(priv->tmds_clk);
err_disable_axiclk:
	clk_disable_unprepare(priv->axi_lite_clk);

	return err;
}

static const struct of_device_id xhdmiphy_of_match[] = {
	{ .compatible = "xlnx,v-hdmi-phy1-1.0" },
	{ .compatible = "xlnx,v-hdmi-gt-controller-1.0" },
	{},
};

MODULE_DEVICE_TABLE(of, xhdmiphy_of_match);

static int xhdmiphy_probe(struct platform_device *pdev)
{
	struct device_node *child, *np = pdev->dev.of_node;
	struct xhdmiphy_dev *priv;
	struct phy_provider *provider;
	struct xhdmiphy_lane *hdmiphy_lane;
	struct platform_device *iface_pdev;
	struct device_node *fnode;
	struct resource *res;
	struct phy *phy;
	int index = 0, ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	ret = xhdmiphy_parse_of(priv);
	if (ret) {
		dev_err(priv->dev, "Error parsing device tree\n");
		return ret;
	}

	fnode = of_parse_phandle(np, "xlnx,hdmi-connector", 0);
	if (!fnode) {
		dev_err(&pdev->dev, "platform node not found\n");
		of_node_put(fnode);
	} else {
		iface_pdev = of_find_device_by_node(fnode);
		if (!iface_pdev) {
			of_node_put(np);
			return -ENODEV;
		}
		priv->data = dev_get_drvdata(&iface_pdev->dev);
		if (!priv->data) {
			dev_dbg(&pdev->dev,
				"platform device not found -EPROBE_DEFER\n");
			of_node_put(fnode);
			return -EPROBE_DEFER;
		}
		of_node_put(fnode);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->phy_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->phy_base))
		return PTR_ERR(priv->phy_base);

	mutex_init(&priv->hdmiphy_mutex);

	for_each_child_of_node(np, child) {
		if (index >= priv->conf.rx_channels + priv->conf.tx_channels) {
			dev_err(&pdev->dev,
				"MAX %d PHY Lanes are supported\n",
				(priv->conf.rx_channels + priv->conf.tx_channels));
			return -E2BIG;
		}

		phy = devm_phy_create(&pdev->dev, child, &xhdmiphy_phyops);
		if (IS_ERR(phy)) {
			dev_err(&pdev->dev, "failed to create HDMI PHY\n");
			return PTR_ERR(phy);
		}

		hdmiphy_lane = devm_kzalloc(&pdev->dev, sizeof(*hdmiphy_lane),
					    GFP_KERNEL);
		if (!hdmiphy_lane)
			return -ENOMEM;

		hdmiphy_lane->lane = index;
		hdmiphy_lane->share_laneclk = -1;
		priv->lanes[index] = hdmiphy_lane;

		priv->lanes[index]->phy = phy;
		phy_set_drvdata(phy, priv->lanes[index]);
		hdmiphy_lane->data = priv;
		index++;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev,
						priv->irq, xhdmiphy_irq_handler,
						xhdmiphy_irq_thread,
						IRQF_TRIGGER_HIGH,
						dev_name(priv->dev),
						priv);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, priv);

	ret = xhdmiphy_clk_init(priv);
	if (ret)
		return ret;

	provider = devm_of_phy_provider_register(&pdev->dev, xhdmiphy_xlate);
	if (IS_ERR(provider)) {
		dev_err(&pdev->dev, "registering provider failed\n");
		ret = PTR_ERR(provider);
		goto err_clk;
	}
	return 0;

err_clk:
	clk_disable_unprepare(priv->dru_clk);
	clk_disable_unprepare(priv->tmds_clk);
	clk_disable_unprepare(priv->axi_lite_clk);

	return ret;
}

static void xhdmiphy_remove(struct platform_device *pdev)
{
	struct xhdmiphy_dev *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->dru_clk);
	clk_disable_unprepare(priv->tmds_clk);
	clk_disable_unprepare(priv->axi_lite_clk);
}

static struct platform_driver xhdmiphy_driver = {
	.probe = xhdmiphy_probe,
	.remove = xhdmiphy_remove,
	.driver = {
		.name = "xilinx-hdmiphy",
		.of_match_table	= xhdmiphy_of_match,
	},
};
module_platform_driver(xhdmiphy_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajesh Gugulothu <gugulothu.rajesh@xilinx.com");
MODULE_DESCRIPTION("Xilinx HDMI PHY driver");
