// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021-2024 NXP

 * Portions of the code are derived from below files:
	* drivers/net/ethernet/stmicro/stmmac/stmmac_main.c
	* drivers/net/ethernet/stmicro/stmmac/stmmac_platform.c
 */

#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/uio_driver.h>
#include <linux/pm_runtime.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>
#include <linux/busfreq-imx.h>
#include <linux/of_mdio.h>
#include "fec.h"
#include <linux/pinctrl/consumer.h>
#include <linux/stmmac.h>
#include <linux/phylink.h>
#include <linux/mdio.h>

#include "../stmicro/stmmac/stmmac_platform.h"
#include "../stmicro/stmmac/stmmac.h"
#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/firmware/imx/sci.h>

struct fec_dev *fec_dev;
static const char fec_uio_version[] = "FEC UIO driver v1.0";
dma_addr_t bd_dma;
int bd_size;
struct bufdesc *cbd_base;
resource_size_t size;

#define NAME_LENGTH		30
#define DRIVER_NAME		"fec-uio"
#define FEC_MDIO_PM_TIMEOUT	100 /* ms */
#define FEC_PRIV_SIZE		200
#define ENABLE_ENET		BIT(8)
#define ETHER_EN		0x2
#define FEC_MAX_Q		3
#define RING_SIZE_TX		512
#define RING_SIZE_RX		512

/* FEC MII MMFR bits definition */
#define FEC_MMFR_ST             BIT(30)
#define FEC_MMFR_ST_C45         (0)
#define FEC_MMFR_OP_READ        (2 << 28)
#define FEC_MMFR_OP_READ_C45    (3 << 28)
#define FEC_MMFR_OP_WRITE       BIT(28)
#define FEC_MMFR_OP_ADDR_WRITE  (0)
#define FEC_MMFR_PA(v)          (((v) & 0x1f) << 23)
#define FEC_MMFR_RA(v)          (((v) & 0x1f) << 18)
#define FEC_MMFR_TA             (2 << 16)
#define FEC_MMFR_DATA(v)        ((v) & 0xffff)

#define MTL_MAX_RX_QUEUES       8
#define MTL_MAX_TX_QUEUES       8
static int phyaddr = -1;
module_param(phyaddr, int, 0444);
MODULE_PARM_DESC(phyaddr, "Physical device address");

/* PCS defines */
#define STMMAC_PCS_RGMII        (1 << 0)
#define STMMAC_PCS_SGMII        (1 << 1)
#define STMMAC_PCS_TBI          (1 << 2)
#define STMMAC_PCS_RTBI         (1 << 3)

static const char uio_device_name[] = "imx-fec-uio";
static int mii_cnt;
struct fec_uio_info {
	atomic_t ref; /* exclusive, only one open() at a time */
	struct uio_info uio_info;
	char name[NAME_LENGTH];
};

struct fec_dev {
	u32 index;
	struct device *dev;
	struct resource *res;
	struct fec_uio_info info;
};

struct fec_uio_devinfo {
	u32 quirks;
};

static const struct fec_uio_devinfo fec_imx8mm_info = {
	.quirks = FEC_QUIRK_ENET_MAC,
};

static struct platform_device_id fec_enet_uio_devtype[] = {
	{
		.name = DRIVER_NAME,
		.driver_data = (kernel_ulong_t)&fec_imx8mm_info,
	}, {
		.name = "imx8mm-fec",
		.driver_data = (kernel_ulong_t)&fec_imx8mm_info,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, fec_enet_uio_devtype);

static const struct of_device_id fec_enet_uio_ids[] = {
	{ .compatible = "fsl,imx8mm-fec-uio", .data = &fec_enet_uio_devtype },
	{ .compatible = "fsl,imx-enet-qos", .data = &fec_enet_uio_devtype },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, fec_enet_uio_ids);

static unsigned char macaddr[ETH_ALEN];
module_param_array(macaddr, byte, NULL, 0);
MODULE_PARM_DESC(macaddr, "FEC Ethernet MAC address");

struct imx_dwmac_ops {
	u32 addr_width;
	bool mac_rgmii_txclk_auto_adj;

	int (*fix_soc_reset)(void *priv, void __iomem *ioaddr);
	int (*set_intf_mode)(struct plat_stmmacenet_data *plat_dat);
};

struct imx_priv_data {
	struct device *dev;
	struct clk *clk_tx;
	struct clk *clk_mem;
	struct regmap *intf_regmap;
	u32 intf_reg_off;
	bool rmii_refclk_ext;
	void __iomem *base_addr;

	const struct imx_dwmac_ops *ops;
	struct plat_stmmacenet_data *plat_dat;
};

static int imx_dwmac_clks_config(void *priv, bool enabled)
{
	struct imx_priv_data *dwmac = priv;
	int ret = 0;

	if (enabled) {
		ret = clk_prepare_enable(dwmac->clk_mem);
		if (ret) {
			dev_err(dwmac->dev, "mem clock enable failed\n");
			return ret;
		}

		ret = clk_prepare_enable(dwmac->clk_tx);
		if (ret) {
			dev_err(dwmac->dev, "tx clock enable failed\n");
			clk_disable_unprepare(dwmac->clk_mem);
			return ret;
		}
	} else {
		clk_disable_unprepare(dwmac->clk_tx);
		clk_disable_unprepare(dwmac->clk_mem);
	}

	return ret;
}

static int imx_dwmac_init(struct platform_device *pdev, void *priv)
{
	struct plat_stmmacenet_data *plat_dat;
	struct imx_priv_data *dwmac = priv;
	int ret;

	plat_dat = dwmac->plat_dat;

	if (dwmac->ops->set_intf_mode) {
		ret = dwmac->ops->set_intf_mode(plat_dat);
		if (ret)
			return ret;
	}

	return 0;
}

static void imx_dwmac_fix_speed(void *priv, unsigned int speed, unsigned int mode)
{
	struct plat_stmmacenet_data *plat_dat;
	struct imx_priv_data *dwmac = priv;
	unsigned long rate;
	int err;

	plat_dat = dwmac->plat_dat;

	if (dwmac->ops->mac_rgmii_txclk_auto_adj ||
			(plat_dat->mac_interface == PHY_INTERFACE_MODE_RMII) ||
			(plat_dat->mac_interface == PHY_INTERFACE_MODE_MII))
		return;

	switch (speed) {
	case SPEED_1000:
		rate = 125000000;
		break;
	case SPEED_100:
		rate = 25000000;
		break;
	case SPEED_10:
		rate = 2500000;
		break;
	default:
		dev_err(dwmac->dev, "invalid speed %u\n", speed);
		return;
	}

	err = clk_set_rate(dwmac->clk_tx, rate);
	if (err < 0)
		dev_err(dwmac->dev, "failed to set tx rate %lu\n", rate);
}

static int
imx_dwmac_parse_dt(struct imx_priv_data *dwmac, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int err = 0;

	if (of_get_property(np, "snps,rmii_refclk_ext", NULL))
		dwmac->rmii_refclk_ext = true;

	dwmac->clk_tx = devm_clk_get(dev, "tx");
	if (IS_ERR(dwmac->clk_tx)) {
		dev_err(dev, "failed to get tx clock\n");
		return PTR_ERR(dwmac->clk_tx);
	}

	dwmac->clk_mem = NULL;

	if (of_machine_is_compatible("fsl,imx8dxl") ||
			of_machine_is_compatible("fsl,imx93")) {
		dwmac->clk_mem = devm_clk_get(dev, "mem");
		if (IS_ERR(dwmac->clk_mem)) {
			dev_err(dev, "failed to get mem clock\n");
			return PTR_ERR(dwmac->clk_mem);
		}
	}

	if (of_machine_is_compatible("fsl,imx8mp") ||
			of_machine_is_compatible("fsl,imx93")) {

		err = of_property_read_u32_index(np, "intf_mode", 1, &dwmac->intf_reg_off);
		if (err) {
			dev_err(dev, "Can't get intf mode reg offset (%d)\n", err);
			return err;
		}
	}

	return err;
}

static void stmmac_fpe_link_state_handle(struct stmmac_priv *priv, bool is_up)
{
	struct stmmac_fpe_cfg *fpe_cfg = &priv->fpe_cfg;
	unsigned long flags;

	timer_shutdown_sync(&fpe_cfg->verify_timer);

	spin_lock_irqsave(&fpe_cfg->lock, flags);

	if (is_up && fpe_cfg->pmac_enabled) {
		/* VERIFY process requires pmac enabled when NIC comes up */
		stmmac_fpe_configure(priv, priv->ioaddr, fpe_cfg,
				     priv->plat->tx_queues_to_use,
				     priv->plat->rx_queues_to_use,
				     false, true);

		/* New link => maybe new partner => new verification process */
		stmmac_fpe_apply(priv);
	} else {
		/* No link => turn off EFPE */
		stmmac_fpe_configure(priv, priv->ioaddr, fpe_cfg,
				     priv->plat->tx_queues_to_use,
				     priv->plat->rx_queues_to_use,
				     false, false);
	}

	spin_unlock_irqrestore(&fpe_cfg->lock, flags);
}

static void stmmac_mac_link_down(struct phylink_config *config,
		unsigned int mode, phy_interface_t interface)
{
	struct stmmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	stmmac_mac_set(priv, priv->ioaddr, false);
	priv->eee_active = false;
	priv->tx_lpi_enabled = false;
	priv->eee_enabled = stmmac_eee_init(priv);
	stmmac_set_eee_pls(priv, priv->hw, false);

	if (priv->dma_cap.fpesel)
		stmmac_fpe_link_state_handle(priv, false);
}

static void stmmac_mac_flow_ctrl(struct stmmac_priv *priv, u32 duplex)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;

	stmmac_flow_ctrl(priv, priv->hw, duplex, priv->flow_ctrl,
			priv->pause, tx_cnt);
}

static void stmmac_mac_link_up(struct phylink_config *config,
		struct phy_device *phy,
		unsigned int mode, phy_interface_t interface,
		int speed, int duplex,
		bool tx_pause, bool rx_pause)
{
	struct stmmac_priv *priv = netdev_priv(to_net_dev(config->dev));
	u32 ctrl;

	ctrl = readl(priv->ioaddr + MAC_CTRL_REG);
	ctrl &= ~priv->hw->link.speed_mask;

	if (interface == PHY_INTERFACE_MODE_USXGMII) {
		switch (speed) {
		case SPEED_10000:
			ctrl |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_5000:
			ctrl |= priv->hw->link.xgmii.speed5000;
			break;
		case SPEED_2500:
			ctrl |= priv->hw->link.xgmii.speed2500;
			break;
		default:
			return;
		}
	} else if (interface == PHY_INTERFACE_MODE_XLGMII) {
		switch (speed) {
		case SPEED_100000:
			ctrl |= priv->hw->link.xlgmii.speed100000;
			break;
		case SPEED_50000:
			ctrl |= priv->hw->link.xlgmii.speed50000;
			break;
		case SPEED_40000:
			ctrl |= priv->hw->link.xlgmii.speed40000;
			break;
		case SPEED_25000:
			ctrl |= priv->hw->link.xlgmii.speed25000;
			break;
		case SPEED_10000:
			ctrl |= priv->hw->link.xgmii.speed10000;
			break;
		case SPEED_2500:
			ctrl |= priv->hw->link.speed2500;
			break;
		case SPEED_1000:
			ctrl |= priv->hw->link.speed1000;
			break;
		default:
			return;
		}
	} else {
		switch (speed) {
		case SPEED_2500:
			ctrl |= priv->hw->link.speed2500;
			break;
		case SPEED_1000:
			ctrl |= priv->hw->link.speed1000;
			break;
		case SPEED_100:
			ctrl |= priv->hw->link.speed100;
			break;
		case SPEED_10:
			ctrl |= priv->hw->link.speed10;
			break;
		default:
			return;
		}
	}

	priv->speed = speed;

	if (priv->plat->fix_mac_speed)
		priv->plat->fix_mac_speed(priv->plat->bsp_priv, speed, mode);

	if (!duplex)
		ctrl &= ~priv->hw->link.duplex;
	else
		ctrl |= priv->hw->link.duplex;

	/* Flow Control operation */
	if (tx_pause && rx_pause)
		stmmac_mac_flow_ctrl(priv, duplex);

	writel(ctrl, priv->ioaddr + MAC_CTRL_REG);

	stmmac_mac_set(priv, priv->ioaddr, true);
	if (phy && priv->dma_cap.eee) {
		priv->eee_active = phy_init_eee(phy, 0) >= 0;
		priv->eee_enabled = stmmac_eee_init(priv);
		priv->tx_lpi_enabled = priv->eee_enabled;
		stmmac_set_eee_pls(priv, priv->hw, true);
	}

	if (priv->dma_cap.fpesel)
		stmmac_fpe_link_state_handle(priv, true);
}

static struct phylink_pcs *stmmac_mac_select_pcs(struct phylink_config *config,
		phy_interface_t interface)
{
	struct stmmac_priv *priv = netdev_priv(to_net_dev(config->dev));

	if (!priv->hw->xpcs)
		return NULL;

	return &priv->hw->xpcs->pcs;
}

static const struct phylink_mac_ops stmmac_phylink_mac_ops = {
	.mac_select_pcs = stmmac_mac_select_pcs,
	.mac_link_down = stmmac_mac_link_down,
	.mac_link_up = stmmac_mac_link_up,
};

static int stmmac_phy_setup(struct stmmac_priv *priv)
{
	struct stmmac_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;
	int max_speed = priv->plat->max_speed;
	int mode = priv->plat->phy_interface;
	struct fwnode_handle *fwnode;
	struct phylink *phylink;

	priv->phylink_config.dev = &priv->dev->dev;
	priv->phylink_config.type = PHYLINK_NETDEV;
	if (priv->plat->mdio_bus_data)
		priv->phylink_config.default_an_inband=
			mdio_bus_data->default_an_inband;

	/* Set the platform/firmware specified interface mode */
	__set_bit(mode, priv->phylink_config.supported_interfaces);

	/* If we have an xpcs, it defines which PHY interfaces are supported. */
	if (priv->hw->xpcs)
		xpcs_get_interfaces(priv->hw->xpcs,
				priv->phylink_config.supported_interfaces);

	priv->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
		MAC_10 | MAC_100;

	if (!max_speed || max_speed >= 1000)
		priv->phylink_config.mac_capabilities |= MAC_1000;

	priv->phylink_config.mac_managed_pm = true;

	fwnode = priv->plat->port_node;
	if (!fwnode)
		fwnode = dev_fwnode(priv->device);

	phylink = phylink_create(&priv->phylink_config, fwnode,
			mode, &stmmac_phylink_mac_ops);
	if (IS_ERR(phylink))
		return PTR_ERR(phylink);

	priv->phylink = phylink;
	return 0;
}

static int stmmac_init_phy(struct net_device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev);
	struct fwnode_handle *phy_fwnode;
	struct fwnode_handle *fwnode;
	int ret = 0;

	if (!phylink_expects_phy(priv->phylink))
		return 0;

	fwnode = priv->plat->port_node;
	if (!fwnode)
		fwnode = dev_fwnode(priv->device);

	if (fwnode)
		phy_fwnode = fwnode_get_phy_node(fwnode);
	else
		phy_fwnode = NULL;

	/* Some DT bindings do not set-up the PHY handle. Let's try to
	 * manually parse it
	 */
	if (!phy_fwnode || IS_ERR(phy_fwnode)) {
		int addr = priv->plat->phy_addr;
		struct phy_device *phydev;

		if (addr < 0) {
			netdev_err(priv->dev, "no phy found\n");
			return -ENODEV;
		}

		phydev = mdiobus_get_phy(priv->mii, addr);
		if (!phydev) {
			netdev_err(priv->dev, "no phy at addr %d\n", addr);
			return -ENODEV;
		}

		ret = phylink_connect_phy(priv->phylink, phydev);
	} else {
		fwnode_handle_put(phy_fwnode);
		ret = phylink_fwnode_phy_connect(priv->phylink, fwnode, 0);
	}

	return ret;
}

static void stmmac_check_pcs_mode(struct stmmac_priv *priv)
{
	int interface = priv->plat->mac_interface;

	if (priv->dma_cap.pcs) {
		if ((interface == PHY_INTERFACE_MODE_RGMII) ||
				(interface == PHY_INTERFACE_MODE_RGMII_ID) ||
				(interface == PHY_INTERFACE_MODE_RGMII_RXID) ||
				(interface == PHY_INTERFACE_MODE_RGMII_TXID)) {
			dev_info(priv->device, "PCS RGMII support enabled\n");
			priv->hw->pcs = STMMAC_PCS_RGMII;
		} else if (interface == PHY_INTERFACE_MODE_SGMII) {
			dev_info(priv->device, "PCS SGMII support enabled\n");
			priv->hw->pcs = STMMAC_PCS_SGMII;
		}
	}
}

static int stmmac_hw_init(struct stmmac_priv *priv)
{
	int ret;

	/* Initialize HW Interface */
	ret = stmmac_hwif_init(priv);
	if (ret)
		return ret;

	return 0;
}

static void stmmac_clk_csr_set(struct stmmac_priv *priv)
{
	u32 clk_rate;

	clk_rate = clk_get_rate(priv->plat->stmmac_clk);

	/* Platform provided default clk_csr would be assumed valid
	 * for all other cases except for the below mentioned ones.
	 * For values higher than the IEEE 802.3 specified frequency
	 * we can not estimate the proper divider as it is not known
	 * the frequency of clk_csr_i. So we do not change the default
	 * divider.
	 */
	if (!(priv->clk_csr & MAC_CSR_H_FRQ_MASK)) {
		if (clk_rate < CSR_F_35M)
			priv->clk_csr = STMMAC_CSR_20_35M;
		else if ((clk_rate >= CSR_F_35M) && (clk_rate < CSR_F_60M))
			priv->clk_csr = STMMAC_CSR_35_60M;
		else if ((clk_rate >= CSR_F_60M) && (clk_rate < CSR_F_100M))
			priv->clk_csr = STMMAC_CSR_60_100M;
		else if ((clk_rate >= CSR_F_100M) && (clk_rate < CSR_F_150M))
			priv->clk_csr = STMMAC_CSR_100_150M;
		else if ((clk_rate >= CSR_F_150M) && (clk_rate < CSR_F_250M))
			priv->clk_csr = STMMAC_CSR_150_250M;
		else if ((clk_rate >= CSR_F_250M) && (clk_rate <= CSR_F_300M))
			priv->clk_csr = STMMAC_CSR_250_300M;
	}

	if (priv->plat->flags & STMMAC_FLAG_HAS_SUN8I) {
		if (clk_rate > 160000000)
			priv->clk_csr = 0x03;
		else if (clk_rate > 80000000)
			priv->clk_csr = 0x02;
		else if (clk_rate > 40000000)
			priv->clk_csr = 0x01;
		else
			priv->clk_csr = 0;
	}

	if (priv->plat->has_xgmac) {
		if (clk_rate > 400000000)
			priv->clk_csr = 0x5;
		else if (clk_rate > 350000000)
			priv->clk_csr = 0x4;
		else if (clk_rate > 300000000)
			priv->clk_csr = 0x3;
		else if (clk_rate > 250000000)
			priv->clk_csr = 0x2;
		else if (clk_rate > 150000000)
			priv->clk_csr = 0x1;
		else
			priv->clk_csr = 0x0;
	}
}

static int stmmac_qos_dvr_probe(struct device *device,
		struct plat_stmmacenet_data *plat_dat,
		struct stmmac_resources *res)
{
	struct net_device *ndev = NULL;
	struct stmmac_priv *priv;
	int ret = 0;

	ndev = devm_alloc_etherdev_mqs(device, sizeof(struct stmmac_priv),
			MTL_MAX_TX_QUEUES, MTL_MAX_RX_QUEUES);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, device);

	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;
	priv->plat = plat_dat;
	priv->ioaddr = res->addr;
	priv->dev->base_addr = (unsigned long)res->addr;
	priv->plat->dma_cfg->multi_msi_en = (priv->plat->flags & STMMAC_FLAG_MULTI_MSI_EN);

	dev_set_drvdata(device, priv->dev);

	if ((phyaddr >= 0) && (phyaddr <= 31))
		priv->plat->phy_addr = phyaddr;

	if (priv->plat->stmmac_rst) {
		ret = reset_control_assert(priv->plat->stmmac_rst);
		reset_control_deassert(priv->plat->stmmac_rst);

		if (ret == -ENOTSUPP)
			reset_control_reset(priv->plat->stmmac_rst);
	}

	ret = reset_control_deassert(priv->plat->stmmac_ahb_rst);
	if (ret == -ENOTSUPP)
		dev_err(priv->device, "unable to bring out of ahb reset: %pe\n",
				ERR_PTR(ret));

	/* Init MAC and get the capabilities */
	ret = stmmac_hw_init(priv);
	if (ret)
		return -1;

	mutex_init(&priv->lock);

	/* If a specific clk_csr value is passed from the platform
	 * this means that the CSR Clock Range selection cannot be
	 * changed at run-time and it is fixed. Viceversa the driver'll try to
	 * set the MDC clock dynamically according to the csr actual
	 * clock input.*/

	if (priv->plat->clk_csr >= 0)
		priv->clk_csr = priv->plat->clk_csr;
	else
		stmmac_clk_csr_set(priv);

	/* Verify if RGMII/SGMII is supported */
	stmmac_check_pcs_mode(priv);

	pm_runtime_get_noresume(device);
	pm_runtime_set_active(device);
	pm_runtime_enable(device);


	if (priv->hw->pcs != STMMAC_PCS_TBI &&
			priv->hw->pcs != STMMAC_PCS_RTBI) {
		/* MDIO bus Registration */
		ret = stmmac_mdio_register(ndev);
		if (ret < 0) {
			dev_err(priv->device,
					"%s: MDIO bus (id: %d) registration failed",
					__func__, priv->plat->bus_id);
			return -1;
		}
	}

	ret = stmmac_phy_setup(priv);
	if (ret) {
		netdev_err(ndev, "failed to setup phy (%d)\n", ret);
		goto error_phy_setup;
	}

	if (priv->hw->pcs != STMMAC_PCS_TBI &&
			priv->hw->pcs != STMMAC_PCS_RTBI) {
		ret = stmmac_init_phy(ndev);
		if (ret) {
			netdev_err(priv->dev,
					"%s: Cannot attach to PHY (error: %d)\n",
					__func__, ret);
			return -1;
		}
	}

	/* Let pm_runtime_put() disable the clocks.
	 * If CONFIG_PM is not enabled, the clocks will stay powered.
	 */
	pm_runtime_put(device);

	return ret;

error_phy_setup:
	if (priv->hw->pcs != STMMAC_PCS_TBI &&
			priv->hw->pcs != STMMAC_PCS_RTBI)
		stmmac_mdio_unregister(ndev);
	return ret;
}

static int enet_qos_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct imx_priv_data *dwmac;
	const struct imx_dwmac_ops *data;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	data = of_device_get_match_data(&pdev->dev);
	if (!data) {
		dev_err(&pdev->dev, "failed to get match data\n");
		ret = -EINVAL;
		goto err_parse_dt;
	}

	dwmac->ops = data;
	dwmac->dev = &pdev->dev;

	ret = imx_dwmac_parse_dt(dwmac, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse OF data\n");
		goto err_parse_dt;
	}

	plat_dat->host_dma_width = dwmac->ops->addr_width;
	plat_dat->init = imx_dwmac_init;
	plat_dat->clks_config = imx_dwmac_clks_config;
	plat_dat->fix_mac_speed = imx_dwmac_fix_speed;
	plat_dat->bsp_priv = dwmac;
	dwmac->plat_dat = plat_dat;
	dwmac->base_addr = stmmac_res.addr;

	ret = imx_dwmac_clks_config(dwmac, true);
	if (ret)
		goto err_clks_config;

	ret = imx_dwmac_init(pdev, dwmac);
	if (ret)
		goto err_dwmac_init;

	dwmac->plat_dat->fix_soc_reset = dwmac->ops->fix_soc_reset;

	ret = stmmac_qos_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		return ret;

	return 0;

err_dwmac_init:
	imx_dwmac_clks_config(dwmac, false);
err_clks_config:
err_parse_dt:
	return ret;
}


#ifdef CONFIG_OF
static int fec_enet_uio_reset_phy(struct platform_device *pdev)
{
	int err, phy_reset;
	bool active_high = false;
	int msec = 1, phy_post_delay = 0;
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return 0;

	err = of_property_read_u32(np, "phy-reset-duration", &msec);
	/* A sane reset duration should not be longer than 1s */
	if (!err && msec > 1000)
		msec = 1;

	phy_reset = of_get_named_gpio(np, "phy-reset-gpios", 0);
	if (phy_reset == -EPROBE_DEFER)
		return phy_reset;
	else if (!gpio_is_valid(phy_reset))
		return 0;

	err = of_property_read_u32(np, "phy-reset-post-delay", &phy_post_delay);
	/* valid reset duration should be less than 1s */
	if (!err && phy_post_delay > 1000)
		return -EINVAL;

	active_high = of_property_read_bool(np, "phy-reset-active-high");

	err = devm_gpio_request_one(&pdev->dev, phy_reset,
			active_high ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
			"phy-reset");
	if (err) {
		dev_err(&pdev->dev, "failed to get phy-reset-gpios: %d\n", err);
		return err;
	}

	if (msec > 20)
		msleep(msec);
	else
		usleep_range(msec * 1000, msec * 1000 + 1000);

	gpio_set_value_cansleep(phy_reset, !active_high);

	if (!phy_post_delay)
		return 0;
	if (phy_post_delay > 20)
		msleep(phy_post_delay);
	else
		usleep_range(phy_post_delay * 1000,
				phy_post_delay * 1000 + 1000);

	return 0;
}

#else /* CONFIG_OF */
static int fec_enet_uio_reset_phy(struct platform_device *pdev)
{
	/* In case of platform probe, the reset has been done
	 * by machine code.
	 */
	return 0;
}
#endif /* CONFIG_OF */

static void fec_enet_uio_phy_reset_after_clk_enable(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct phy_device *phy_dev = ndev->phydev;

	if (phy_dev) {
		phy_reset_after_clk_enable(phy_dev);
	} else if (fep->phy_node) {
		/* If the PHY still is not bound to the MAC, but there is
		 * OF PHY node and a matching PHY device instance already,
		 * use the OF PHY node to obtain the PHY device instance,
		 * and then use that PHY device instance when triggering
		 * the PHY reset.
		 */
		phy_dev = of_phy_find_device(fep->phy_node);
		phy_reset_after_clk_enable(phy_dev);
		put_device(&phy_dev->mdio.dev);
	}
}

static int fec_enet_uio_mdio_wait(struct fec_enet_private *fep)
{
	uint ievent;
	int ret;

	ret = readl_poll_timeout_atomic(fep->hwp + FEC_IEVENT, ievent,
			ievent & FEC_ENET_MII, 2, 30000);

	if (!ret)
		writel(FEC_ENET_MII, fep->hwp + FEC_IEVENT);
	return ret;
}

static int fec_enet_uio_mdio_read_c22(struct mii_bus *bus, int mii_id, int regnum)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret = 0, frame_start, frame_addr, frame_op;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	/* C22 read */
	frame_op = FEC_MMFR_OP_READ;
	frame_start = FEC_MMFR_ST;
	frame_addr = regnum;

	/* start a read op */
	writel(frame_start | frame_op |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
			FEC_MMFR_TA, fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret) {
		netdev_err(fep->netdev, "MDIO read timeout\n");
		goto out;
	}

	ret = FEC_MMFR_DATA(readl(fep->hwp + FEC_MII_DATA));

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mdio_read_c45(struct mii_bus *bus, int mii_id,
				      int devad, int regnum)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret = 0, frame_start, frame_op;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	frame_start = FEC_MMFR_ST_C45;

	/* write address */
	writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(devad) |
			FEC_MMFR_TA | (regnum & 0xFFFF),
			fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret) {
		netdev_err(fep->netdev, "MDIO address write timeout\n");
		goto out;
	}

	frame_op = FEC_MMFR_OP_READ_C45;

	/* start a read op */
	writel(frame_start | frame_op |
	       FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(devad) |
	       FEC_MMFR_TA, fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret) {
		netdev_err(fep->netdev, "MDIO read timeout\n");
		goto out;
	}

	ret = FEC_MMFR_DATA(readl(fep->hwp + FEC_MII_DATA));

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mdio_write_c22(struct mii_bus *bus, int mii_id, int regnum,
					u16 value)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret, frame_start, frame_addr;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	/* C22 write */
	frame_start = FEC_MMFR_ST;
	frame_addr = regnum;

	/* start a write op */
	writel(frame_start | FEC_MMFR_OP_WRITE |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
		FEC_MMFR_TA | FEC_MMFR_DATA(value),
		fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret)
		netdev_err(fep->netdev, "MDIO write timeout\n");

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mdio_write_c45(struct mii_bus *bus, int mii_id,
				   int devad, int regnum, u16 value)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret, frame_start;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	frame_start = FEC_MMFR_ST_C45;

	/* write address */
	writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
	       FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(devad) |
	       FEC_MMFR_TA | (regnum & 0xFFFF),
	       fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret) {
		netdev_err(fep->netdev, "MDIO address write timeout\n");
		goto out;
	}

	/* start a write op */
	writel(frame_start | FEC_MMFR_OP_WRITE |
	       FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(devad) |
	       FEC_MMFR_TA | FEC_MMFR_DATA(value),
	       fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret)
		netdev_err(fep->netdev, "MDIO write timeout\n");

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mii_init(struct platform_device *pdev)
{
	static struct mii_bus *fec0_mii_bus;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	bool suppress_preamble = false;
	struct device_node *node;
	int err = -ENXIO;
	u32 mii_speed, holdtime;
	u32 bus_freq;

	/* The i.MX28 dual fec interfaces are not equal.
	 * Here are the differences:
	 *
	 *  - fec0 supports MII & RMII modes while fec1 only supports RMII
	 *  - fec0 acts as the 1588 time master while fec1 is slave
	 *  - external phys can only be configured by fec0
	 *
	 * That is to say fec1 can not work independently. It only works
	 * when fec0 is working. The reason behind this design is that the
	 * second interface is added primarily for Switch mode.
	 *
	 * Because of the last point above, both phys are attached on fec0
	 * mdio interface in board design, and need to be configured by
	 * fec0 mii_bus.
	 */
	if ((fep->quirks & FEC_QUIRK_SINGLE_MDIO) && fep->dev_id > 0) {
		/* fec1 uses fec0 mii_bus */
		if (mii_cnt && fec0_mii_bus) {
			fep->mii_bus = fec0_mii_bus;
			mii_cnt++;
			return 0;
		}
		return -ENOENT;
	}

	bus_freq = 2500000; /* 2.5MHz by default */
	node = of_get_child_by_name(pdev->dev.of_node, "mdio");
	if (node) {
		of_property_read_u32(node, "clock-frequency", &bus_freq);
		suppress_preamble = of_property_read_bool(node,
				"suppress-preamble");
	}

	/* Set MII speed to 2.5 MHz (= clk_get_rate() / 2 * phy_speed)
	 *
	 * The formula for FEC MDC is 'ref_freq / (MII_SPEED x 2)' while
	 * for ENET-MAC is 'ref_freq / ((MII_SPEED + 1) x 2)'.  The i.MX28
	 * Reference Manual has an error on this, and gets fixed on i.MX6Q
	 * document.
	 */
	mii_speed = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 5000000);
	if (fep->quirks & FEC_QUIRK_ENET_MAC)
		mii_speed--;
	if (mii_speed > 63) {
		dev_err(&pdev->dev,
				"fec clock (%lu) too fast to get right mii speed\n",
				clk_get_rate(fep->clk_ipg));
		err = -EINVAL;
		goto err_out;
	}

	/* The i.MX28 and i.MX6 types have another filed in the MSCR (aka
	 * MII_SPEED) register that defines the MDIO output hold time. Earlier
	 * versions are RAZ there, so just ignore the difference and write the
	 * register always.
	 * The minimal hold time according to IEE802.3 (clause 22) is 10 ns.
	 * HOLDTIME + 1 is the number of clk cycles the fec is holding the
	 * output.
	 * The HOLDTIME bitfield takes values between 0 and 7 (inclusive).
	 * Given that ceil(clkrate / 5000000) <= 64, the calculation for
	 * holdtime cannot result in a value greater than 3.
	 */
	holdtime = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 100000000) - 1;

	fep->phy_speed = mii_speed << 1 | holdtime << 8;

	if (suppress_preamble)
		fep->phy_speed |= BIT(7);

	if (fep->quirks & FEC_QUIRK_CLEAR_SETUP_MII) {
		/* Clear MMFR to avoid to generate MII event by writing MSCR.
		 * MII event generation condition:
		 * - writing MSCR:
		 *      - mmfr[31:0]_not_zero & mscr[7:0]_is_zero &
		 *        mscr_reg_data_in[7:0] != 0
		 * - writing MMFR:
		 *      - mscr[7:0]_not_zero
		 */
		writel(0, fep->hwp + FEC_MII_DATA);
	}

	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	/* Clear any pending transaction complete indication */
	writel(FEC_ENET_MII, fep->hwp + FEC_IEVENT);

	fep->mii_bus = mdiobus_alloc();
	if (!fep->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus->name = "fec_enet_mii_bus";
	fep->mii_bus->read = fec_enet_uio_mdio_read_c22;
	fep->mii_bus->write = fec_enet_uio_mdio_write_c22;
	fep->mii_bus->read_c45 = fec_enet_uio_mdio_read_c45;
	fep->mii_bus->write_c45 = fec_enet_uio_mdio_write_c45;
	snprintf(fep->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
			pdev->name, fep->dev_id + 1);
	fep->mii_bus->priv = fep;
	fep->mii_bus->parent = &pdev->dev;

	err = of_mdiobus_register(fep->mii_bus, node);
	of_node_put(node);
	if (err)
		goto err_out_free_mdiobus;

	mii_cnt++;

	/* save fec0 mii_bus */
	if (fep->quirks & FEC_QUIRK_SINGLE_MDIO)
		fec0_mii_bus = fep->mii_bus;

	return 0;

err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus);
err_out:
	return err;
}

static int fec_uio_open(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int fec_uio_release(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int fec_uio_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	u32 ret;
	u32 pfn;

	pfn = (info->mem[vma->vm_pgoff].addr) >> PAGE_SHIFT;

	if (vma->vm_pgoff)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start, pfn,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (ret) {
		/* Error Handle */
		pr_info("remap_pfn_range failed");
	}
	return ret;
}

static int __init fec_uio_init(struct fec_dev *fec_dev)
{
	struct fec_uio_info *fec_uio_info;
	int ret;

	fec_uio_info = &fec_dev->info;
	atomic_set(&fec_uio_info->ref, 0);
	fec_uio_info->uio_info.version = fec_uio_version;
	fec_uio_info->uio_info.name = fec_dev->info.name;

	fec_uio_info->uio_info.mem[0].name = "FEC_REG_SPACE";
	fec_uio_info->uio_info.mem[0].addr = fec_dev->res->start;
	fec_uio_info->uio_info.mem[0].size = 0x1000;
	fec_uio_info->uio_info.mem[0].internal_addr = 0;
	fec_uio_info->uio_info.mem[0].memtype = UIO_MEM_PHYS;

	fec_uio_info->uio_info.mem[1].name = "FEC_BD_SPACE";
	fec_uio_info->uio_info.mem[1].addr = bd_dma;
	fec_uio_info->uio_info.mem[1].size = bd_size;
	fec_uio_info->uio_info.mem[1].memtype = UIO_MEM_PHYS;

	fec_uio_info->uio_info.open = fec_uio_open;
	fec_uio_info->uio_info.release = fec_uio_release;
	/* Custom mmap function. */
	fec_uio_info->uio_info.mmap = fec_uio_mmap;
	fec_uio_info->uio_info.priv = fec_dev;

	ret = uio_register_device(fec_dev->dev, &fec_uio_info->uio_info);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			pr_info("UIO is not initialized, enetfec probe deferred");
		else
			dev_err(fec_dev->dev, "fec_uio: UIO registration failed\n");

		return ret;
	}
	return 0;
}

static void fec_enet_uio_mii_remove(struct fec_enet_private *fep)
{
	if (--mii_cnt == 0) {
		mdiobus_unregister(fep->mii_bus);
		mdiobus_free(fep->mii_bus);
	}
}

static int fec_enet_uio_init(struct net_device *ndev)
{
	unsigned int total_tx_ring_size = 0, total_rx_ring_size = 0;
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned int dsize = sizeof(struct bufdesc);
	unsigned short tx_ring_size, rx_ring_size;
	int ret, i;

	/* Check mask of the streaming and coherent API */
	ret = dma_set_mask_and_coherent(&fep->pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_warn(&fep->pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	tx_ring_size = RING_SIZE_TX;
	rx_ring_size = RING_SIZE_RX;

	for (i = 0; i < FEC_MAX_Q; i++) {
		total_tx_ring_size += tx_ring_size;
		total_rx_ring_size += rx_ring_size;
	}
	bd_size = (total_tx_ring_size + total_rx_ring_size) * dsize;

	/* Allocate memory for buffer descriptors. */
	cbd_base = dma_alloc_coherent(&fep->pdev->dev, bd_size, &bd_dma,
			GFP_KERNEL);
	if (!cbd_base) {
		ret = -ENOMEM;
		goto free_mem;
	}

	return 0;
free_mem:
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);
	return ret;
}

static int
enet_fec_probe(struct platform_device *pdev)
{
	struct fec_uio_devinfo *dev_info;
	const struct of_device_id *of_id;
	struct fec_enet_private *fep;
	struct net_device *ndev;
	u32 ecntl = ETHER_EN;
	static int dev_id;
	bool reset_again;
	int ret = 0;

	/* Init network device */
	ndev = alloc_etherdev_mq(sizeof(struct fec_enet_private) +
			FEC_PRIV_SIZE, FEC_MAX_Q);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	/* setup board info structure */
	fep = netdev_priv(ndev);

	of_id = of_match_device(fec_enet_uio_ids, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;

	dev_info = (struct fec_uio_devinfo *)pdev->id_entry->driver_data;
	if (dev_info)
		fep->quirks = dev_info->quirks;

	/* Select default pin state */
	pinctrl_pm_select_default_state(&pdev->dev);

	/* allocate memory for uio structure */
	fec_dev = kzalloc(sizeof(*fec_dev), GFP_KERNEL);
	if (!fec_dev) {
		ret = -ENOMEM;
		goto failed_kzalloc;
	}
	snprintf(fec_dev->info.name, sizeof(fec_dev->info.name) - 1,
			"%s", uio_device_name);

	fec_dev->dev = &pdev->dev;
	fec_dev->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = resource_size(fec_dev->res);
	fep->hwp = devm_ioremap_resource(&pdev->dev, fec_dev->res);
	if (IS_ERR(fep->hwp)) {
		ret = PTR_ERR(fep->hwp);
		goto failed_ioremap;
	}
	fep->pdev = pdev;
	fep->dev_id = dev_id++;

	platform_set_drvdata(pdev, ndev);

	request_bus_freq(BUS_FREQ_HIGH);

	fep->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(fep->clk_ipg)) {
		ret = PTR_ERR(fep->clk_ipg);
		goto failed_clk_ipg_res_release;
	}

	fep->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(fep->clk_ahb)) {
		ret = PTR_ERR(fep->clk_ahb);
		goto failed_clk_ahb_res_release;
	}

	/* enet_out is optional, depends on board */
	fep->clk_enet_out = devm_clk_get_optional(&pdev->dev, "enet_out");
	if (IS_ERR(fep->clk_enet_out)) {
		ret = PTR_ERR(fep->clk_enet_out);
		goto failed_clk_enet_out_res_release;
	}

	/* clk_ref is optional, depends on board */
	fep->clk_ref = devm_clk_get_optional(&pdev->dev, "enet_clk_ref");
	if (IS_ERR(fep->clk_ref)) {
		ret = PTR_ERR(fep->clk_ref);
		goto failed_clk_ref_res_release;
	}

	ret = clk_prepare_enable(fep->clk_enet_out);
	if (ret)
		goto failed_clk;

	ret = clk_prepare_enable(fep->clk_ref);
	if (ret)
		goto failed_clk_ref;

	fec_enet_uio_phy_reset_after_clk_enable(ndev);

	ret = clk_prepare_enable(fep->clk_ipg);
	if (ret)
		goto failed_clk_ipg;

	ret = clk_prepare_enable(fep->clk_ahb);
	if (ret)
		goto failed_clk_ahb;

	fep->reg_phy = devm_regulator_get_optional(&pdev->dev, "phy");
	if (!IS_ERR(fep->reg_phy)) {
		ret = regulator_enable(fep->reg_phy);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable phy regulator: %d\n", ret);
			goto failed_regulator;
		}
	} else {
		if (PTR_ERR(fep->reg_phy) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto failed_regulator;
		}
		fep->reg_phy = NULL;
	}

	pm_runtime_enable(&pdev->dev);
	ret = fec_enet_uio_reset_phy(pdev);
	if (ret) {
		dev_err(&pdev->dev, "fec-uio reset phy FAILED %d\n", ret);
		goto failed_reset;
	}
	ret = fec_enet_uio_init(ndev);
	if (ret) {
		dev_err(&pdev->dev, "fec-uio init FAILED %d\n", ret);
		goto failed_init;
	}

	/* Register UIO */
	ret = fec_uio_init(fec_dev);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			dev_info(&pdev->dev,
					"Driver request probe retry: %s\n", __func__);
		else
			dev_err(&pdev->dev, "UIO init Failed\n");

		goto failed_uio_init;
	}
	dev_info(fec_dev->dev, "UIO device \"%s\" initialized\n",
			fec_dev->info.name);

	if (fep->quirks & FEC_QUIRK_ENET_MAC) {
		/* enable ENET endian swap */
		ecntl |= ENABLE_ENET;
		/* enable ENET store and forward mode */
		writel(ENABLE_ENET, fep->hwp + FEC_X_WMRK);
	}

	/* And last, enable the transmit and receive processing */
	writel(ecntl, fep->hwp + FEC_ECNTRL);

	ret = fec_enet_uio_mii_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "fec-uio mii_init FAILED %d\n", ret);
		goto failed_mii_init;
	}
	pm_runtime_get_sync(&fep->pdev->dev);
	if (ndev->phydev && ndev->phydev->drv)
		reset_again = false;
	else
		reset_again = true;

	return 0;

failed_mii_init:
	fec_enet_uio_mii_remove(fep);
failed_uio_init:
	uio_unregister_device(&fec_dev->info.uio_info);
failed_init:
	if (cbd_base)
		dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);
failed_reset:
	pm_runtime_disable(&pdev->dev);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);
failed_regulator:
	clk_disable_unprepare(fep->clk_ahb);
failed_clk_ahb:
	clk_disable_unprepare(fep->clk_ipg);
failed_clk_ipg:
	clk_disable_unprepare(fep->clk_ref);
failed_clk_ref:
	clk_disable_unprepare(fep->clk_enet_out);
failed_clk:
	devm_clk_put(&pdev->dev, fep->clk_ref);
failed_clk_ref_res_release:
	devm_clk_put(&pdev->dev, fep->clk_enet_out);
failed_clk_enet_out_res_release:
	devm_clk_put(&pdev->dev, fep->clk_ahb);
failed_clk_ahb_res_release:
	devm_clk_put(&pdev->dev, fep->clk_ipg);
failed_clk_ipg_res_release:
	release_bus_freq(BUS_FREQ_HIGH);
	dev_id--;
failed_ioremap:
	kfree(fec_dev);
	if (fep->hwp)
		devm_release_mem_region(&pdev->dev, fec_dev->res->start, size);
failed_kzalloc:
	free_netdev(ndev);

	return ret;
}

static void 
fec_enet_uio_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	fec_enet_uio_mii_remove(fep);
	uio_unregister_device(&fec_dev->info.uio_info);
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);

	if (fep->hwp)
		devm_release_mem_region(&pdev->dev, fec_dev->res->start, size);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);

	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	clk_disable_unprepare(fep->clk_ref);
	clk_disable_unprepare(fep->clk_enet_out);
	devm_clk_put(&pdev->dev, fep->clk_ref);
	devm_clk_put(&pdev->dev, fep->clk_enet_out);
	devm_clk_put(&pdev->dev, fep->clk_ahb);
	devm_clk_put(&pdev->dev, fep->clk_ipg);
	pm_runtime_disable(&pdev->dev);
	release_bus_freq(BUS_FREQ_HIGH);
	fep->dev_id--;
	kfree(fec_dev);
	free_netdev(ndev);
	dev_info(fec_dev->dev, "\"%s\" successfully removed \n",
		 fec_dev->info.name);
}

static int
fec_enet_uio_probe(struct platform_device *pdev)
{
	const char *comp_str;
	int ret;

	comp_str = of_get_property(pdev->dev.of_node, "compatible", NULL);
	if (!comp_str) {
		dev_err(&pdev->dev, "Ethernet compatible is missing.\n");
		return -EINVAL;
	}

	/* This is for the ENET-QOS ethernet (i.MX8MP & i.MX93 supported)*/
	if (!strcmp(comp_str, "fsl,imx-enet-qos")) {
		ret = enet_qos_probe(pdev);

		if (ret)
			dev_err(&pdev->dev, "QOS port probe FAILED %d\n", ret);
		else
			printk("ENET-QOS successfully initialized \n");
	} else {
		ret = enet_fec_probe(pdev);

		if (ret)
			dev_err(&pdev->dev, "FEC port probe FAILED %d\n", ret);
	}

	return ret;
}

static struct platform_driver fec_enet_uio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = fec_enet_uio_ids,
		.suppress_bind_attrs = true,
	},
	.id_table = fec_enet_uio_devtype,
	.prevent_deferred_probe = false,
	.probe = fec_enet_uio_probe,
	.remove = fec_enet_uio_remove,
};

module_platform_driver(fec_enet_uio_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX FEC UIO Driver");
