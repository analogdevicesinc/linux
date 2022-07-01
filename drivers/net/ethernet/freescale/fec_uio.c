// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 */

#include <linux/kernel.h>
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

struct fec_dev *fec_dev;
static const char fec_uio_version[] = "FEC UIO driver v1.0";
dma_addr_t bd_dma;
int bd_size;
struct bufdesc *cbd_base;

#define NAME_LENGTH		30
#define DRIVER_NAME		"fec-uio"
#define FEC_MDIO_PM_TIMEOUT	100 /* ms */
#define FEC_PRIV_SIZE		200
#define ENABLE_ENET		BIT(8)
#define ETHER_EN		0x2
#define FEC_MAX_Q		3


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
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, fec_enet_uio_ids);

static unsigned char macaddr[ETH_ALEN];
module_param_array(macaddr, byte, NULL, 0);
MODULE_PARM_DESC(macaddr, "FEC Ethernet MAC address");

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

static int fec_enet_uio_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret = 0, frame_start, frame_addr, frame_op;
	bool is_c45 = !!(regnum & MII_ADDR_C45);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	if (is_c45) {
		frame_start = FEC_MMFR_ST_C45;

		/* write address */
		frame_addr = (regnum >> 16);
		writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
			FEC_MMFR_TA | (regnum & 0xFFFF),
			fep->hwp + FEC_MII_DATA);

		/* wait for end of transfer */
		ret = fec_enet_uio_mdio_wait(fep);
		if (ret) {
			netdev_err(fep->netdev, "MDIO address write timeout\n");
			goto out;
		}
		frame_op = FEC_MMFR_OP_READ_C45;
	} else {
		/* C22 read */
		frame_op = FEC_MMFR_OP_READ;
		frame_start = FEC_MMFR_ST;
		frame_addr = regnum;
	}
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

static int fec_enet_uio_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				   u16 value)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret, frame_start, frame_addr;
	bool is_c45 = !!(regnum & MII_ADDR_C45);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	if (is_c45) {
		frame_start = FEC_MMFR_ST_C45;

		/* write address */
		frame_addr = (regnum >> 16);
		writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
			FEC_MMFR_TA | (regnum & 0xFFFF),
			fep->hwp + FEC_MII_DATA);

		/* wait for end of transfer */
		ret = fec_enet_uio_mdio_wait(fep);
		if (ret) {
			netdev_err(fep->netdev, "MDIO address write timeout\n");
			goto out;
		}
	} else {
		/* C22 write */
		frame_start = FEC_MMFR_ST;
		frame_addr = regnum;
	}

	/* start a write op */
	writel(frame_start | FEC_MMFR_OP_WRITE |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
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
	fep->mii_bus->read = fec_enet_uio_mdio_read;
	fep->mii_bus->write = fec_enet_uio_mdio_write;
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
	if (ret == -517)
		return ret;
	if (ret) {
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

	tx_ring_size = TX_RING_SIZE;
	rx_ring_size = RX_RING_SIZE;

	for (i = 0; i < FEC_ENET_MAX_TX_QS; i++)
		total_tx_ring_size += tx_ring_size;
	for (i = 0; i < FEC_ENET_MAX_RX_QS; i++)
		total_rx_ring_size += rx_ring_size;

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
fec_enet_uio_probe(struct platform_device *pdev)
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
	if (!fec_dev)
		return -ENOMEM;

	snprintf(fec_dev->info.name, sizeof(fec_dev->info.name) - 1,
		 "%s", uio_device_name);

	fec_dev->dev = &pdev->dev;

	fec_dev->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fep->hwp = ioremap(fec_dev->res->start, 0x1000);
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
		goto failed_clk;
	}

	fep->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(fep->clk_ahb)) {
		ret = PTR_ERR(fep->clk_ahb);
		goto failed_clk;
	}

	/* enet_out is optional, depends on board */
	fep->clk_enet_out = devm_clk_get(&pdev->dev, "enet_out");
	if (IS_ERR(fep->clk_enet_out))
		fep->clk_enet_out = NULL;

	/* clk_ref is optional, depends on board */
	fep->clk_ref = devm_clk_get(&pdev->dev, "enet_clk_ref");
	if (IS_ERR(fep->clk_ref))
		fep->clk_ref = NULL;

	ret = clk_prepare_enable(fep->clk_enet_out);
	if (ret)
		return ret;

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
	if (ret)
		goto failed_reset;

	ret = fec_enet_uio_init(ndev);
	if (ret)
		goto failed_init;

	/* Register UIO */
	ret = fec_uio_init(fec_dev);
	if (ret) {
		if (ret == -517) {
			dev_info(&pdev->dev,
				 "Driver request probe retry: %s\n", __func__);
			goto out_unmap;
		} else {
			dev_err(&pdev->dev, "UIO init Failed\n");
			goto abort;
		}
	}
	dev_info(fec_dev->dev, "UIO device full name %s initialized\n",
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
	if (ret)
		goto failed_mii_init;

	pm_runtime_get_sync(&fep->pdev->dev);
	if (ndev->phydev && ndev->phydev->drv)
		reset_again = false;
	else
		reset_again = true;

	return 0;

failed_mii_init:
failed_init:
failed_reset:
	pm_runtime_disable(&pdev->dev);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);
failed_clk_ref:
	clk_disable_unprepare(fep->clk_enet_out);
failed_regulator:
	clk_disable_unprepare(fep->clk_ahb);
failed_clk_ahb:
	clk_disable_unprepare(fep->clk_ipg);
failed_clk_ipg:
	clk_disable_unprepare(fep->clk_enet_out);
	clk_disable_unprepare(fep->clk_ref);
failed_clk:
	release_bus_freq(BUS_FREQ_HIGH);
	dev_id--;
failed_ioremap:
	free_netdev(ndev);

	return ret;
out_unmap:
	dev_id--;
	kfree(fec_dev);
	iounmap(fep->hwp);
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);
	free_netdev(ndev);
	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	clk_disable_unprepare(fep->clk_enet_out);
	clk_disable_unprepare(fep->clk_ref);
	pm_runtime_disable(&pdev->dev);

	return -EPROBE_DEFER;
abort:
	return ret;
}

static int
fec_enet_uio_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	kfree(fec_dev);
	iounmap(fep->hwp);
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);

	uio_unregister_device(&fec_dev->info.uio_info);

	fec_enet_uio_mii_remove(fep);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);

	free_netdev(ndev);

	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	pm_runtime_disable(&pdev->dev);

	return 0;
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
