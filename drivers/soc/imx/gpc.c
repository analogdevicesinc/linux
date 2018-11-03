/*
 * Copyright 2015-2017 Pengutronix, Lucas Stach <kernel@pengutronix.de>
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <soc/imx/revision.h>

#define GPC_CNTR		0x000

#define GPC_CNTR_PCIE_PHY_PDU_SHIFT	0x7
#define GPC_CNTR_PCIE_PHY_PDN_SHIFT	0x6
#define PGC_PCIE_PHY_CTRL		0x200
#define PGC_PCIE_PHY_PDN_EN		0x1

#define GPC_PGC_CTRL_OFFS	0x0
#define GPC_PGC_PUPSCR_OFFS	0x4
#define GPC_PGC_PDNSCR_OFFS	0x8
#define GPC_PGC_SW2ISO_SHIFT	0x8
#define GPC_PGC_SW_SHIFT	0x0

#define GPC_PGC_PCI_PDN		0x200
#define GPC_PGC_PCI_SR		0x20c
#define GPC_PGC_DISP_PGCR_OFFSET       0x240
#define GPC_PGC_DISP_PUPSCR_OFFSET     0x244
#define GPC_PGC_DISP_PDNSCR_OFFSET     0x248
#define GPC_PGC_DISP_SR_OFFSET         0x24c

#define GPC_PGC_GPU_PDN		0x260
#define GPC_PGC_GPU_PUPSCR	0x264
#define GPC_PGC_GPU_PDNSCR	0x268
#define GPC_PGC_GPU_SR		0x26c

#define GPC_PGC_DISP_PDN	0x240
#define GPC_PGC_DISP_SR		0x24c

#define GPC_CLK_MAX		10
#define DEFAULT_IPG_RATE		66000000
#define GPC_PU_UP_DELAY_MARGIN		2

#define PGC_DOMAIN_FLAG_NO_PD		BIT(0)

static void __iomem *gpc_base;
static struct clk *ipg_clk;

static inline bool cpu_is_imx6sx(void)
{
	return of_machine_is_compatible("fsl,imx6sx");
}

static inline bool cpu_is_imx6sl(void)
{
	return of_machine_is_compatible("fsl,imx6sl");
}

static inline bool cpu_is_imx6q(void)
{
	return of_machine_is_compatible("fsl,imx6q");
}

struct imx_pm_domain {
	struct generic_pm_domain base;
	struct regmap *regmap;
	struct regulator *supply;
	struct clk *clk[GPC_CLK_MAX];
	int num_clks;
	unsigned int reg_offs;
	signed char cntr_pdn_bit;
	unsigned int ipg_rate_mhz;
	unsigned int flags;
};

static inline struct imx_pm_domain *
to_imx_pm_domain(struct generic_pm_domain *genpd)
{
	return container_of(genpd, struct imx_pm_domain, base);
}

static void _imx6_pm_domain_power_off(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);
	int iso, iso2sw;
	u32 val;

	/* Read ISO and ISO2SW power down delays */
	regmap_read(pd->regmap, pd->reg_offs + GPC_PGC_PUPSCR_OFFS, &val);
	iso = val & 0x3f;
	iso2sw = (val >> 8) & 0x3f;

	/* Gate off domain when powered down */
	regmap_update_bits(pd->regmap, pd->reg_offs + GPC_PGC_CTRL_OFFS,
			   0x1, 0x1);

	/* Request GPC to power down domain */
	val = BIT(pd->cntr_pdn_bit);
	regmap_update_bits(pd->regmap, GPC_CNTR, val, val);

	/* Wait ISO + ISO2SW IPG clock cycles */
	udelay(DIV_ROUND_UP(iso + iso2sw, pd->ipg_rate_mhz));

	while (readl_relaxed(gpc_base + GPC_CNTR) & val)
		;
}

static int imx6_pm_domain_power_off(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);

	if (pd->flags & PGC_DOMAIN_FLAG_NO_PD)
		return -EBUSY;

	_imx6_pm_domain_power_off(genpd);

	if (pd->supply)
		regulator_disable(pd->supply);

	return 0;
}

static void _imx6_pm_domain_power_on(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);
	int i;
	u32 val, ipg_rate = clk_get_rate(ipg_clk);

	if (ipg_rate == 0) {
		WARN_ON(1);
		return;
	}

	/* Enable reset clocks for all devices in the domain */
	for (i = 0; i < pd->num_clks; i++)
		clk_prepare_enable(pd->clk[i]);

	/* Gate off domain when powered down */
	regmap_update_bits(pd->regmap, pd->reg_offs + GPC_PGC_CTRL_OFFS,
			   0x1, 0x1);

	/* Request GPC to power up domain */
	val = BIT(pd->cntr_pdn_bit + 1);
	regmap_update_bits(pd->regmap, GPC_CNTR, val, val);

	while (readl_relaxed(gpc_base + GPC_CNTR) & val)
		;
	/* Wait power switch done */
	udelay(2 * DEFAULT_IPG_RATE / ipg_rate +
		GPC_PU_UP_DELAY_MARGIN);

	/* Disable reset clocks for all devices in the domain */
	for (i = 0; i < pd->num_clks; i++)
		clk_disable_unprepare(pd->clk[i]);
}

static int imx6_pm_domain_power_on(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);
	int ret;

	if (pd->supply) {
		ret = regulator_enable(pd->supply);
		if (ret) {
			pr_err("%s: failed to enable regulator: %d\n",
			       __func__, ret);
			return ret;
		}
	}

	_imx6_pm_domain_power_on(genpd);

	return 0;
}

static int imx6_pm_dispmix_on(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);
	u32 val = readl_relaxed(gpc_base + GPC_CNTR);
	u32 ipg_rate = clk_get_rate(ipg_clk);
	int i;

	if (ipg_rate == 0) {
		WARN_ON(1);
		return -EINVAL;
	}

	if ((cpu_is_imx6sl() &&
		imx_get_soc_revision() >= IMX_CHIP_REVISION_1_2) || cpu_is_imx6sx()) {

		/* Enable reset clocks for all devices in the disp domain */
		for (i = 0; i < pd->num_clks; i++)
			clk_prepare_enable(pd->clk[i]);

		writel_relaxed(0x0, gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
		writel_relaxed(0x20 | val, gpc_base + GPC_CNTR);
		while (readl_relaxed(gpc_base + GPC_CNTR) & 0x20)
			;

		writel_relaxed(0x1, gpc_base + GPC_PGC_DISP_SR_OFFSET);

		/* Wait power switch done */
		udelay(2 * DEFAULT_IPG_RATE / ipg_rate +
			GPC_PU_UP_DELAY_MARGIN);

		/* Disable reset clocks for all devices in the disp domain */
		for (i = 0; i < pd->num_clks; i++)
			clk_disable_unprepare(pd->clk[i]);
	}
	return 0;
}

static int imx6_pm_dispmix_off(struct generic_pm_domain *genpd)
{
	struct imx_pm_domain *pd = to_imx_pm_domain(genpd);
	u32 val = readl_relaxed(gpc_base + GPC_CNTR);
	int i;

	if ((cpu_is_imx6sl() &&
		imx_get_soc_revision() >= IMX_CHIP_REVISION_1_2) || cpu_is_imx6sx()) {

		/* Enable reset clocks for all devices in the disp domain */
		for (i = 0; i < pd->num_clks; i++)
			clk_prepare_enable(pd->clk[i]);

		writel_relaxed(0xFFFFFFFF,
				gpc_base + GPC_PGC_DISP_PUPSCR_OFFSET);
		writel_relaxed(0xFFFFFFFF,
				gpc_base + GPC_PGC_DISP_PDNSCR_OFFSET);
		writel_relaxed(0x1, gpc_base + GPC_PGC_DISP_PGCR_OFFSET);
		writel_relaxed(0x10 | val, gpc_base + GPC_CNTR);
		while (readl_relaxed(gpc_base + GPC_CNTR) & 0x10)
			;

		/* Disable reset clocks for all devices in the disp domain */
		for (i = 0; i < pd->num_clks; i++)
			clk_disable_unprepare(pd->clk[i]);
	}
	return 0;
}

static int imx_pgc_get_clocks(struct device *dev, struct imx_pm_domain *domain)
{
	int i, ret;

	for (i = 0; ; i++) {
		struct clk *clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;
		if (i >= GPC_CLK_MAX) {
			dev_err(dev, "more than %d clocks\n", GPC_CLK_MAX);
			ret = -EINVAL;
			goto clk_err;
		}
		domain->clk[i] = clk;
	}
	domain->num_clks = i;

	return 0;

clk_err:
	while (i--)
		clk_put(domain->clk[i]);

	return ret;
}

static void imx_pgc_put_clocks(struct imx_pm_domain *domain)
{
	int i;

	for (i = domain->num_clks - 1; i >= 0; i--)
		clk_put(domain->clk[i]);
}

static int imx_pgc_parse_dt(struct device *dev, struct imx_pm_domain *domain)
{
	/* try to get the domain supply regulator */
	domain->supply = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(domain->supply)) {
		if (PTR_ERR(domain->supply) == -ENODEV)
			domain->supply = NULL;
		else
			return PTR_ERR(domain->supply);
	}

	/* try to get all clocks needed for reset propagation */
	return imx_pgc_get_clocks(dev, domain);
}

static void imx_gpc_handle_ldobypass(struct platform_device *pdev);

static int imx_pgc_power_domain_probe(struct platform_device *pdev)
{
	struct imx_pm_domain *domain = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	int ret;

	/* if this PD is associated with a DT node try to parse it */
	if (dev->of_node) {
		ret = imx_pgc_parse_dt(dev, domain);
		if (ret)
			return ret;
	}

	/* initially power on the domain */
	if (domain->base.power_on)
		domain->base.power_on(&domain->base);

	if (IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)) {
		pm_genpd_init(&domain->base, NULL, false);
		ret = of_genpd_add_provider_simple(dev->of_node, &domain->base);
		if (ret)
			goto genpd_err;
	}

	device_link_add(dev, dev->parent, DL_FLAG_AUTOREMOVE);

	/* Mark PU regulator as bypass */
	if (pdev->id == 1)
		imx_gpc_handle_ldobypass(to_platform_device(pdev->dev.parent));

	return 0;

genpd_err:
	pm_genpd_remove(&domain->base);
	imx_pgc_put_clocks(domain);

	return ret;
}

static int imx_pgc_power_domain_remove(struct platform_device *pdev)
{
	struct imx_pm_domain *domain = pdev->dev.platform_data;

	if (IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)) {
		of_genpd_del_provider(pdev->dev.of_node);
		pm_genpd_remove(&domain->base);
		imx_pgc_put_clocks(domain);
	}

	return 0;
}

static const struct platform_device_id imx_pgc_power_domain_id[] = {
	{ "imx-pgc-power-domain"},
	{ },
};

static struct platform_driver imx_pgc_power_domain_driver = {
	.driver = {
		.name = "imx-pgc-pd",
	},
	.probe = imx_pgc_power_domain_probe,
	.remove = imx_pgc_power_domain_remove,
	.id_table = imx_pgc_power_domain_id,
};
builtin_platform_driver(imx_pgc_power_domain_driver)

#define GPC_PGC_DOMAIN_ARM	0
#define GPC_PGC_DOMAIN_PU	1
#define GPC_PGC_DOMAIN_DISPLAY	2

static struct genpd_power_state imx6_pm_domain_pu_state = {
	.power_off_latency_ns = 25000,
	.power_on_latency_ns = 2000000,
};

static struct imx_pm_domain imx_gpc_domains[] = {
	{
		.base = {
			.name = "ARM",
		},
	}, {
		.base = {
			.name = "PU",
			.power_off = imx6_pm_domain_power_off,
			.power_on = imx6_pm_domain_power_on,
			.states = &imx6_pm_domain_pu_state,
			.state_count = 1,
		},
		.reg_offs = 0x260,
		.cntr_pdn_bit = 0,
	}, {
		.base = {
			.name = "DISPLAY",
			.power_off = imx6_pm_dispmix_off,
			.power_on = imx6_pm_dispmix_on,
		},
		.reg_offs = 0x240,
		.cntr_pdn_bit = 4,
	}
};

struct imx_gpc_dt_data {
	int num_domains;
	bool err009619_present;
};

static const struct imx_gpc_dt_data imx6q_dt_data = {
	.num_domains = 2,
	.err009619_present = false,
};

static const struct imx_gpc_dt_data imx6qp_dt_data = {
	.num_domains = 2,
	.err009619_present = true,
};

static const struct imx_gpc_dt_data imx6sl_dt_data = {
	.num_domains = 3,
	.err009619_present = false,
};

static const struct imx_gpc_dt_data imx6sx_dt_data = {
	.num_domains = 3,
	.err009619_present = false,
};

static const struct of_device_id imx_gpc_dt_ids[] = {
	{ .compatible = "fsl,imx6q-gpc", .data = &imx6q_dt_data },
	{ .compatible = "fsl,imx6qp-gpc", .data = &imx6qp_dt_data },
	{ .compatible = "fsl,imx6sl-gpc", .data = &imx6sl_dt_data },
	{ .compatible = "fsl,imx6sx-gpc", .data = &imx6sx_dt_data },
	{ }
};

static const struct regmap_range yes_ranges[] = {
	regmap_reg_range(GPC_CNTR, GPC_CNTR),
	regmap_reg_range(GPC_PGC_PCI_PDN, GPC_PGC_PCI_SR),
	regmap_reg_range(GPC_PGC_GPU_PDN, GPC_PGC_GPU_SR),
	regmap_reg_range(GPC_PGC_DISP_PDN, GPC_PGC_DISP_SR),
};

static const struct regmap_access_table access_table = {
	.yes_ranges	= yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(yes_ranges),
};

static const struct regmap_config imx_gpc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.rd_table = &access_table,
	.wr_table = &access_table,
	.max_register = 0x2ac,
};

static struct generic_pm_domain *imx_gpc_onecell_domains[] = {
	&imx_gpc_domains[0].base,
	&imx_gpc_domains[1].base,
};

static struct genpd_onecell_data imx_gpc_onecell_data = {
	.domains = imx_gpc_onecell_domains,
	.num_domains = 2,
};

/* exported for suspend/resume code in arch/arm/mach-imx/gpc.c */
void _imx6_pm_pu_power_off(void)
{
	_imx6_pm_domain_power_off(&imx_gpc_domains[GPC_PGC_DOMAIN_PU].base);
}
EXPORT_SYMBOL_GPL(_imx6_pm_pu_power_off);

void _imx6_pm_pu_power_on(void)
{
	_imx6_pm_domain_power_on(&imx_gpc_domains[GPC_PGC_DOMAIN_PU].base);
}
EXPORT_SYMBOL_GPL(_imx6_pm_pu_power_on);

static int imx_gpc_old_dt_init(struct device *dev, struct regmap *regmap,
			       unsigned int num_domains)
{
	struct clk *clk;
	struct imx_pm_domain *domain;
	bool is_off;
	int pu_clks, disp_clks, ipg_clks = 1;
	int i = 0, k = 0, ret;

	struct imx_pm_domain *pu_domain = &imx_gpc_domains[GPC_PGC_DOMAIN_PU];
	struct imx_pm_domain *disp_domain = &imx_gpc_domains[GPC_PGC_DOMAIN_DISPLAY];

	if ((cpu_is_imx6sl() &&
	     imx_get_soc_revision() >= IMX_CHIP_REVISION_1_2)) {
		pu_clks = 2;
		disp_clks = 5;
	} else if (cpu_is_imx6sx()) {
		pu_clks = 1;
		disp_clks = 7;
	} else {
		pu_clks = 6;
		disp_clks = 0;
	}

	/* Get pu domain clks */
	for (i = 0; i < pu_clks ; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;
		pu_domain->clk[i] = clk;
	}
	pu_domain->num_clks = i;

	ipg_clk = of_clk_get(dev->of_node, pu_clks);

	/* Get disp domain clks */
	for (i = pu_clks + ipg_clks; i < pu_clks + ipg_clks + disp_clks;
		i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;
		disp_domain->clk[k++] = clk;
	}
	disp_domain->num_clks = k;

	for (i = 0; i < num_domains; i++) {
		domain = &imx_gpc_domains[i];
		domain->regmap = regmap;
		domain->ipg_rate_mhz = 66;

		if (i == GPC_PGC_DOMAIN_PU) {
			domain->supply = devm_regulator_get(dev, "pu");
			if (IS_ERR(domain->supply))
				return PTR_ERR(domain->supply);;

			domain->base.power_on(&domain->base);
		}
	}

	is_off = IS_ENABLED(CONFIG_PM);
	if (is_off && !(cpu_is_imx6q() &&
		imx_get_soc_revision() == IMX_CHIP_REVISION_2_0)) {
		_imx6_pm_domain_power_off(&pu_domain->base);
	} else {
		/*
		 * Enable power if compiled without CONFIG_PM in case the
		 * bootloader disabled it.
		 */
		imx6_pm_domain_power_on(&pu_domain->base);
	}

	for (i = 0; i < num_domains; i++)
		pm_genpd_init(&imx_gpc_domains[i].base, NULL, is_off);

	if (IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)) {
		ret = of_genpd_add_provider_onecell(dev->of_node,
						    &imx_gpc_onecell_data);
		if (ret)
			goto genpd_err;
	}

	return 0;

genpd_err:
	for (i = 0; i < num_domains; i++) {
		pm_genpd_remove(&imx_gpc_domains[i].base);
		imx_pgc_put_clocks(&imx_gpc_domains[i]);
	}
	return ret;
}

static void imx_gpc_handle_ldobypass(struct platform_device *pdev)
{
	struct regulator *pu_reg = imx_gpc_domains[GPC_PGC_DOMAIN_PU].supply;
	u32 bypass = 0;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "fsl,ldo-bypass", &bypass);
	if (ret && ret != -EINVAL)
		dev_warn(&pdev->dev, "failed to read fsl,ldo-bypass property: %d\n", ret);

	/* We only bypass pu since arm and soc has been set in u-boot */
	if (pu_reg && bypass) {
		regulator_allow_bypass(pu_reg, true);
	}
}

static struct notifier_block nb_pcie;

static int imx_pcie_regulator_notify(struct notifier_block *nb,
					unsigned long event,
					void *ignored)
{
	u32 value = readl_relaxed(gpc_base + GPC_CNTR);

	switch (event) {
	case REGULATOR_EVENT_PRE_DO_ENABLE:
		value |= 1 << GPC_CNTR_PCIE_PHY_PDU_SHIFT;
		writel_relaxed(value, gpc_base + GPC_CNTR);
		break;
	case REGULATOR_EVENT_PRE_DO_DISABLE:
		value |= 1 << GPC_CNTR_PCIE_PHY_PDN_SHIFT;
		writel_relaxed(value, gpc_base + GPC_CNTR);
		writel_relaxed(PGC_PCIE_PHY_PDN_EN,
				gpc_base + PGC_PCIE_PHY_CTRL);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int imx_gpc_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(imx_gpc_dt_ids, &pdev->dev);
	const struct imx_gpc_dt_data *of_id_data = of_id->data;
	struct device_node *pgc_node;
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	int ret;

	pgc_node = of_get_child_by_name(pdev->dev.of_node, "pgc");

	/* bail out if DT too old and doesn't provide the necessary info */
	if (!of_property_read_bool(pdev->dev.of_node, "#power-domain-cells") &&
	    !pgc_node)
		return 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	gpc_base = base;

	regmap = devm_regmap_init_mmio_clk(&pdev->dev, NULL, base,
					   &imx_gpc_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&pdev->dev, "failed to init regmap: %d\n",
			ret);
		return ret;
	}

	/* Disable PU power down in normal operation if ERR009619 is present */
	if (of_id_data->err009619_present)
		imx_gpc_domains[GPC_PGC_DOMAIN_PU].flags |=
				PGC_DOMAIN_FLAG_NO_PD;

	if (!pgc_node) {
		ret = imx_gpc_old_dt_init(&pdev->dev, regmap,
					  of_id_data->num_domains);
		if (ret)
			return ret;

		imx_gpc_handle_ldobypass(pdev);
	} else {
		struct imx_pm_domain *domain;
		struct platform_device *pd_pdev;
		struct device_node *np;
		unsigned int ipg_rate_mhz;
		int domain_index;

		ipg_clk = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(ipg_clk))
			return PTR_ERR(ipg_clk);
		ipg_rate_mhz = clk_get_rate(ipg_clk) / 1000000;

		for_each_child_of_node(pgc_node, np) {
			ret = of_property_read_u32(np, "reg", &domain_index);
			if (ret) {
				of_node_put(np);
				return ret;
			}
			if (domain_index >= of_id_data->num_domains)
				continue;

			domain = &imx_gpc_domains[domain_index];
			domain->regmap = regmap;
			domain->ipg_rate_mhz = ipg_rate_mhz;

			pd_pdev = platform_device_alloc("imx-pgc-power-domain",
							domain_index);
			if (!pd_pdev) {
				of_node_put(np);
				return -ENOMEM;
			}
			pd_pdev->dev.platform_data = domain;
			pd_pdev->dev.parent = &pdev->dev;
			pd_pdev->dev.of_node = np;

			ret = platform_device_add(pd_pdev);
			if (ret) {
				platform_device_put(pd_pdev);
				of_node_put(np);
				return ret;
			}
		}
	}

	if (of_machine_is_compatible("fsl,imx6sx")) {
		struct regulator *pcie_reg;

		pcie_reg = devm_regulator_get(&pdev->dev, "pcie-phy");
		if (IS_ERR(pcie_reg)) {
			ret = PTR_ERR(pcie_reg);
			dev_info(&pdev->dev, "pcie regulator not ready.\n");
			return ret;
		}
		nb_pcie.notifier_call = &imx_pcie_regulator_notify;

		ret = regulator_register_notifier(pcie_reg, &nb_pcie);
		if (ret) {
			dev_err(&pdev->dev,
				"pcie regulator notifier request failed\n");
			return ret;
		}
	}

	return 0;
}

static int imx_gpc_remove(struct platform_device *pdev)
{
	struct device_node *pgc_node;
	int ret;

	pgc_node = of_get_child_by_name(pdev->dev.of_node, "pgc");

	/* bail out if DT too old and doesn't provide the necessary info */
	if (!of_property_read_bool(pdev->dev.of_node, "#power-domain-cells") &&
	    !pgc_node)
		return 0;

	/*
	 * If the old DT binding is used the toplevel driver needs to
	 * de-register the power domains
	 */
	if (!pgc_node) {
		of_genpd_del_provider(pdev->dev.of_node);

		ret = pm_genpd_remove(&imx_gpc_domains[GPC_PGC_DOMAIN_PU].base);
		if (ret)
			return ret;
		imx_pgc_put_clocks(&imx_gpc_domains[GPC_PGC_DOMAIN_PU]);

		ret = pm_genpd_remove(&imx_gpc_domains[GPC_PGC_DOMAIN_ARM].base);
		if (ret)
			return ret;
	}

	return 0;
}

static struct platform_driver imx_gpc_driver = {
	.driver = {
		.name = "imx-gpc",
		.of_match_table = imx_gpc_dt_ids,
	},
	.probe = imx_gpc_probe,
	.remove = imx_gpc_remove,
};
builtin_platform_driver(imx_gpc_driver)
