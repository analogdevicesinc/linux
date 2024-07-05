// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NXP NETC Privileged Register Block (PRB) and Integrated
 * Endpoint Register Block (IERB) driver
 *
 * Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fsl/netc_prb_ierb.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

/* NETC Block Control NETCMIX register */
#define CFG_LINK_IO_VAR		0xc
#define  IO_VAR_16FF_16G_SERDES	0x1
#define  IO_VAR(port, var)	(((var) & 0xf) << ((port) << 2))

#define CFG_LINK_MII_PROT	0x10
#define  MII_PROT_MII		0x0
#define  MII_PROT_RMII		0x1
#define  MII_PROT_RGMII		0x2
#define  MII_PROT_SERIAL	0x3
#define  MII_PROT(port, prot)	(((prot) & 0xf) << ((port) << 2))

#define CFG_LINK_PCS_PROT_0	0x14
#define CFG_LINK_PCS_PROT_1	0x18
#define CFG_LINK_PCS_PROT_2	0x1c
#define  PCS_PROT_1G_SGMII	BIT(0)
#define  PCS_PROT_2500M_SGMII	BIT(1)
#define  PCS_PROT_XFI		BIT(3)
#define  PCS_PROT_SFI		BIT(4)
#define  PCS_PROT_10G_SXGMII	BIT(6)

#define CFG_SRC_ID		0x20

/* NETC privileged register block register */
#define PRB_NETCRR		0x100	/* NETC reset register */
#define  NETCRR_SR		BIT(0)
#define  NETCRR_LOCK		BIT(1)

#define PRB_NETCSR		0x104	/* NETC status register */
#define  NETCSR_ERROR		BIT(0)
#define  NETCSR_STATE		BIT(1)

/* NETC integrated endpoint register block register */
#define IERB_CAPR(a)		(0x0 + 0x4 * (a))
#define IERB_ITTMCAPR		0x30
#define IERB_HBTMAR		0x100
#define IERB_EMDIOMCR		0x314
#define IERB_T0MCR		0x414
#define IERB_TGSM0CAPR		0x808
#define IERB_LCAPR(a)		(0x1000 + 0x40 * (a))
#define IERB_LMCAPR(a)		(0x1004 + 0x40 * (a))
#define IERB_LIOCAPR(a)		(0x1008 + 0x40 * (a))
#define IERB_LBCR(a)		(0x1010 + 0x40 * (a))
#define IERB_EBCR1(a)		(0x3004 + 0x100 * (a))
#define IERB_EBCR2(a)		(0x3008 + 0x100 * (a))
#define IERB_EVFRIDAR(a)	(0x3010 + 0x100 * (a))
#define IERB_EMCR(a)		(0x3014 + 0x100 * (a))
#define IERB_EIPFTMAR(a)	(0x3088 + 0x100 * (a))

#define IERB_EMDIOFAUXR		0x344
#define IERB_T0FAUXR		0x444
#define IERB_EFAUXR(a)		(0x3044 + 0x100 * (a))
#define IERB_VFAUXR(a)		(0x4004 + 0x40 * (a))
#define FAUXR_LDID		GENMASK(3, 0)

#define SAI_CLK_SEL		0x4
#define SAI_CLK_SEL_BIT5	BIT(5)
#define SAI_CLK_SEL_BIT10	BIT(10)

struct netc_prb_ierb {
	void __iomem *prb_base;
	void __iomem *ierb_base;
	struct clk *ipg_clk;

	struct regmap *netcmix;
	struct platform_device *pdev;
	struct dentry *debugfs_root;
	u32 *ifmode;
	u32 *rmii_clk_dir;

	struct device *emdio;
};

static struct netc_prb_ierb *netc_pi;

static void netc_reg_write(void __iomem *base, u32 offset, u32 val)
{
	iowrite32(val, base + offset);
}

static u32 netc_reg_read(void __iomem *base, u32 offset)
{
	return ioread32(base + offset);
}

static void netc_netcmix_init(struct platform_device *pdev)
{
	struct netc_prb_ierb *pi;

	pi = platform_get_drvdata(pdev);

	/* Configure Link I/O variant */
	regmap_write(pi->netcmix, CFG_LINK_IO_VAR,
		     IO_VAR(2, IO_VAR_16FF_16G_SERDES));

	/* Configure Link MII port */
	regmap_write(pi->netcmix, CFG_LINK_MII_PROT,
		     MII_PROT(0, pi->ifmode[0]) |
		     MII_PROT(1, pi->ifmode[1]) |
		     MII_PROT(2, pi->ifmode[2]));

	/* Configure Link0/1/2 PCS protocol */
	regmap_write(pi->netcmix, CFG_LINK_PCS_PROT_0, 0);
	regmap_write(pi->netcmix, CFG_LINK_PCS_PROT_1, 0);
	regmap_write(pi->netcmix, CFG_LINK_PCS_PROT_2,
		     PCS_PROT_10G_SXGMII);
	if (pi->rmii_clk_dir)
		regmap_write(pi->netcmix, SAI_CLK_SEL,
			     pi->rmii_clk_dir[0] ? SAI_CLK_SEL_BIT5 : 0 |
			     pi->rmii_clk_dir[1] ? SAI_CLK_SEL_BIT10 : 0);
}

static bool netc_ierb_is_locked(struct netc_prb_ierb *pi)
{
	return !!(netc_reg_read(pi->prb_base, PRB_NETCRR) & NETCRR_LOCK);
}

static int netc_lock_ierb(struct netc_prb_ierb *pi)
{
	u32 val;

	netc_reg_write(pi->prb_base, PRB_NETCRR, NETCRR_LOCK);

	return read_poll_timeout(netc_reg_read, val, !(val & NETCSR_STATE),
				 100, 2000, false, pi->prb_base, PRB_NETCSR);
}

static int netc_unlock_ierb_with_warm_reset(struct netc_prb_ierb *pi)
{
	u32 val;

	netc_reg_write(pi->prb_base, PRB_NETCRR, 0);

	return read_poll_timeout(netc_reg_read, val, !(val & NETCRR_LOCK),
				 1000, 100000, true, pi->prb_base, PRB_NETCRR);
}

static void netc_ierb_init_ldid(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct netc_prb_ierb *pi;

	pi = platform_get_drvdata(pdev);
	if (of_device_is_compatible(node, "fsl,imx95-netc-prb-ierb")) {
		/* EMDIO : No MSI-X intterupt */
		netc_reg_write(pi->ierb_base, IERB_EMDIOFAUXR, 0);
		/* ENETC0 PF */
		netc_reg_write(pi->ierb_base, IERB_EFAUXR(0), 0);
		/* ENETC0 VF0 */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(0), 1);
		/* ENETC0 VF1 */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(1), 2);
		/* ENETC1 PF */
		netc_reg_write(pi->ierb_base, IERB_EFAUXR(1), 3);
		/* ENETC1 VF0 : Disabled on 19x19 board dts */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(2), 5);
		/* ENETC1 VF1 : Disabled on 19x19 board dts */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(3), 6);
		/* ENETC2 PF */
		netc_reg_write(pi->ierb_base, IERB_EFAUXR(2), 4);
		/* ENETC2 VF0 : Disabled on 15x15 board dts */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(4), 5);
		/* ENETC2 VF1 : Disabled on 15x15 board dts */
		netc_reg_write(pi->ierb_base, IERB_VFAUXR(5), 6);
		/* NETC TIMER */
		netc_reg_write(pi->ierb_base, IERB_T0FAUXR, 7);
	}
}

static int netc_ierb_init(struct platform_device *pdev)
{
	struct netc_prb_ierb *pi;
	int err;

	pi = platform_get_drvdata(pdev);
	if (netc_ierb_is_locked(pi)) {
		err = netc_unlock_ierb_with_warm_reset(pi);
		if (err) {
			dev_err(&pdev->dev, "Unlock IERB failed.\n");
			return err;
		}
	}

	netc_ierb_init_ldid(pdev);

	err = netc_lock_ierb(pi);
	if (err) {
		dev_err(&pdev->dev, "Lock IERB failed.\n");
		return err;
	}

	return 0;
}

void netc_prb_ierb_register_emdio(struct device *emdio)
{
	netc_pi->emdio = emdio;
}
EXPORT_SYMBOL_GPL(netc_prb_ierb_register_emdio);

int netc_prb_ierb_check_emdio_state(void)
{
	if (!netc_pi->emdio)
		return -EPROBE_DEFER;

	if (IS_ERR(netc_pi->emdio))
		return PTR_ERR(netc_pi->emdio);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_prb_ierb_check_emdio_state);

int netc_prb_ierb_add_emdio_consumer(struct device *consumer)
{
	struct device_link *link;

	link = device_link_add(consumer, netc_pi->emdio, DL_FLAG_PM_RUNTIME |
			       DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (!link)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(netc_prb_ierb_add_emdio_consumer);

#if IS_ENABLED(CONFIG_DEBUG_FS)
static int netc_prb_show(struct seq_file *s, void *data)
{
	struct netc_prb_ierb *pi = s->private;
	u32 val;

	val = netc_reg_read(pi->prb_base, PRB_NETCRR);
	seq_printf(s, "[PRB NETCRR] Lock:%d SR:%d\n",
		   (val & NETCRR_LOCK) ? 1 : 0,
		   (val & NETCRR_SR) ? 1 : 0);

	val = netc_reg_read(pi->prb_base, PRB_NETCSR);
	seq_printf(s, "[PRB NETCSR] State:%d Error:%d\n",
		   (val & NETCSR_STATE) ? 1 : 0,
		   (val & NETCSR_ERROR) ? 1 : 0);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_prb);

static int netc_ierb_ldid_show(struct seq_file *s, void *data)
{
	struct netc_prb_ierb *pi = s->private;
	u32 val;
	int i;

	val = netc_reg_read(pi->ierb_base, IERB_EMDIOFAUXR);
	seq_printf(s, "EMDIO LDID:%u\n", val);

	val = netc_reg_read(pi->ierb_base, IERB_T0FAUXR);
	seq_printf(s, "Timer LDID:%u\n", val);

	for (i = 0; i < 3; i++) {
		val = netc_reg_read(pi->ierb_base, IERB_EFAUXR(i));
		seq_printf(s, "ENETC%d LDID:%u\n", i, val);
	}

	for (i = 0; i < 6; i++) {
		val = netc_reg_read(pi->ierb_base, IERB_VFAUXR(i));
		seq_printf(s, "VSI%d LDID:%u\n", i, val);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_ierb_ldid);

static int netc_ierb_config_show(struct seq_file *s, void *data)
{
	struct netc_prb_ierb *pi = s->private;
	u32 val;
	int i;

	for (i = 0; i < 4; i++) {
		val = netc_reg_read(pi->ierb_base, IERB_CAPR(i));
		seq_printf(s, "IERB_CAPR%d:0x%08x\n", i, val);
	}

	val = netc_reg_read(pi->ierb_base, IERB_ITTMCAPR);
	seq_printf(s, "IERB_ITTMCAPR:0x%08x\n", val);

	val = netc_reg_read(pi->ierb_base, IERB_HBTMAR);
	seq_printf(s, "IERB_HBTMAR:0x%08x\n", val);

	val = netc_reg_read(pi->ierb_base, IERB_EMDIOMCR);
	seq_printf(s, "IERB_EMDIOMCR:0x%08x\n", val);

	val = netc_reg_read(pi->ierb_base, IERB_T0MCR);
	seq_printf(s, "IERB_T0MCR:0x%08x\n", val);

	val = netc_reg_read(pi->ierb_base, IERB_TGSM0CAPR);
	seq_printf(s, "IERB_TGSM0CAPR:0x%08x\n", val);

	for (i = 0; i < 3; i++) {
		val = netc_reg_read(pi->ierb_base, IERB_LCAPR(i));
		seq_printf(s, "IERB_L%dCAPR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_LMCAPR(i));
		seq_printf(s, "IERB_L%dMCAPR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_LIOCAPR(i));
		seq_printf(s, "IERB_L%dIOCAPR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_LBCR(i));
		seq_printf(s, "IERB_L%dBCR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_EBCR1(i));
		seq_printf(s, "IERB_E%dBCR1:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_EBCR2(i));
		seq_printf(s, "IERB_E%dBCR2:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_EVFRIDAR(i));
		seq_printf(s, "IERB_E%dVFRIDAR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_EMCR(i));
		seq_printf(s, "IERB_E%dMCR:0x%08x\n", i, val);

		val = netc_reg_read(pi->ierb_base, IERB_EIPFTMAR(i));
		seq_printf(s, "IERB_E%dIPFTMAR:0x%08x\n", i, val);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(netc_ierb_config);

static void netc_prb_ierb_create_debugfs(struct netc_prb_ierb *pi)
{
	struct dentry *root;

	root = debugfs_create_dir("netc_prb_ierb", NULL);
	if (IS_ERR(root))
		return;

	pi->debugfs_root = root;

	debugfs_create_file("prb", 0444, root, pi, &netc_prb_fops);
	debugfs_create_file("ierb_ldid", 0444, root, pi, &netc_ierb_ldid_fops);
	debugfs_create_file("ierb_cfg", 0444, root, pi, &netc_ierb_config_fops);
}

static void netc_prb_ierb_remove_debugfs(struct netc_prb_ierb *pi)
{
	debugfs_remove_recursive(pi->debugfs_root);
	pi->debugfs_root = NULL;
}

#else

static void netc_prb_ierb_create_debugfs(struct netc_prb_ierb *pi)
{
}

static void netc_prb_ierb_remove_debugfs(struct netc_prb_ierb *pi)
{
}
#endif

static int netc_prb_check_error(struct netc_prb_ierb *pi)
{
	u32 val;

	val = netc_reg_read(pi->prb_base, PRB_NETCSR);
	if (val & NETCSR_ERROR)
		return -1;

	return 0;
}

static int netc_prb_parse_if(struct netc_prb_ierb *pi, struct device *dev)
{
	struct device_node *node = dev->of_node;
	int ret, count;

	count = of_property_count_u32_elems(node, "netc-interfaces");
	if (count < 1)
		return -EINVAL;
	pi->ifmode = devm_kcalloc(dev, count, sizeof(pi->ifmode), GFP_KERNEL);

	ret = of_property_read_u32_array(node, "netc-interfaces",
					 pi->ifmode, count);
	if (ret)
		return ret;

	if (pi->ifmode[0] == MII_PROT_RMII || pi->ifmode[1] == MII_PROT_RMII) {
		count = of_property_count_u32_elems(node, "netc-rmii-clk-dir");
		if (count < 1) {
			dev_err(dev, "Missing netc-rmii-clk-dir property.\n");
			return -EINVAL;
		}
		pi->rmii_clk_dir = devm_kcalloc(dev, count,
						sizeof(pi->rmii_clk_dir),
						GFP_KERNEL);

		ret = of_property_read_u32_array(node, "netc-rmii-clk-dir",
						 pi->rmii_clk_dir, count);
		if (ret)
			return ret;
	}

	return 0;
}

static int netc_prb_ierb_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct netc_prb_ierb *pi;
	struct regmap *netcmix;
	void __iomem *regs;
	int err;

	if (!node || !of_device_is_available(node)) {
		dev_info(dev, "Device is disabled, skipping\n");
		return -ENODEV;
	}

	pi = devm_kzalloc(dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	pi->ipg_clk = devm_clk_get_optional(dev, "ipg_clk");
	if (IS_ERR(pi->ipg_clk)) {
		dev_err(dev, "Get ipg_clk failed\n");
		err = PTR_ERR(pi->ipg_clk);
		goto free_pi;
	}

	err = clk_prepare_enable(pi->ipg_clk);
	if (err) {
		dev_err(dev, "Enable ipg_clk failed\n");
		goto disable_ipg_clk;
	}

	netcmix = syscon_regmap_lookup_by_compatible("fsl,imx95-netcmix-blk-ctrl");
	if (IS_ERR(netcmix)) {
		err = PTR_ERR(netcmix);
		dev_err(dev, "No syscon regmap\n");
		goto disable_ipg_clk;
	}
	pi->netcmix = netcmix;

	regs = devm_platform_ioremap_resource_byname(pdev, "ierb_base");
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		dev_err(dev, "Missing IERB resource.\n");
		goto disable_ipg_clk;
	}
	pi->ierb_base = regs;

	regs = devm_platform_ioremap_resource_byname(pdev, "prb_base");
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		dev_err(dev, "Missing PRB resource.\n");
		goto disable_ipg_clk;
	}
	pi->prb_base = regs;
	pi->pdev = pdev;

	platform_set_drvdata(pdev, pi);

	netc_prb_parse_if(pi, &pdev->dev);
	netc_netcmix_init(pdev);

	err = netc_ierb_init(pdev);
	if (err) {
		dev_err(dev, "Initializing IERB failed.\n");
		goto disable_ipg_clk;
	}

	if (netc_prb_check_error(pi) < 0)
		dev_warn(dev, "The current IERB configuration is invalid.\n");

	netc_pi = pi;
	netc_prb_ierb_create_debugfs(pi);

	err = of_platform_populate(node, NULL, NULL, dev);
	if (err) {
		dev_err(dev, "of_platform_populate failed\n");
		goto remove_debugfs;
	}

	return 0;

remove_debugfs:
	netc_prb_ierb_remove_debugfs(pi);
	netc_pi = NULL;
disable_ipg_clk:
	clk_disable_unprepare(pi->ipg_clk);
free_pi:
	devm_kfree(dev, pi);

	return err;
}

static void netc_prb_ierb_remove(struct platform_device *pdev)
{
	struct netc_prb_ierb *pi = platform_get_drvdata(pdev);

	of_platform_depopulate(&pdev->dev);
	netc_prb_ierb_remove_debugfs(pi);
	netc_pi = NULL;
	clk_disable_unprepare(pi->ipg_clk);
	devm_kfree(&pdev->dev, pi);
}

static const struct of_device_id netc_prb_ierb_match[] = {
	{ .compatible = "fsl,imx95-netc-prb-ierb", },
	{},
};
MODULE_DEVICE_TABLE(of, netc_prb_ierb_match);

static struct platform_driver netc_prb_ierb_driver = {
	.driver = {
		.name = "fsl-netc-prb-ierb",
		.of_match_table = netc_prb_ierb_match,
	},
	.probe = netc_prb_ierb_probe,
	.remove = netc_prb_ierb_remove,
};

module_platform_driver(netc_prb_ierb_driver);

MODULE_AUTHOR("Wei Fang <wei.fang@nxp.com>");
MODULE_DESCRIPTION("NXP NETC PRB and IERB Driver");
MODULE_LICENSE("Dual BSD/GPL");
