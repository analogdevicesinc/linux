/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm_clock.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>

#include <soc/imx8/sc/sci.h>

#include "pm-domain-imx8.h"

static sc_ipc_t pm_ipc_handle;
static sc_rsrc_t early_power_on_rsrc[] = {
	SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST,
	SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST, SC_R_LAST,
};
static sc_rsrc_t rsrc_debug_console;

#define IMX8_WU_MAX_IRQS	(((SC_R_LAST + 31) / 32 ) * 32 )
static sc_rsrc_t irq2rsrc[IMX8_WU_MAX_IRQS];
static sc_rsrc_t wakeup_rsrc_id[IMX8_WU_MAX_IRQS / 32];
static DEFINE_SPINLOCK(imx8_wu_lock);
static DEFINE_MUTEX(rsrc_pm_list_lock);

enum imx_pd_state {
	PD_LP,
	PD_OFF,
};

struct clk_stat {
	struct clk *clk;
	struct clk *parent;
	unsigned long rate;
};

static int imx8_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct imx8_pm_domain *pd;
	sc_err_t sci_err = SC_ERR_NONE;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	if (pd->rsrc_id == SC_R_LAST)
		return 0;

	/* keep uart console power on for no_console_suspend */
	if (pd->rsrc_id == rsrc_debug_console &&
		!console_suspend_enabled && !power_on)
		return 0;

	/* keep resource power on if it is a wakeup source */
	if (!power_on && ((1 << pd->rsrc_id % 32) &
		wakeup_rsrc_id[pd->rsrc_id / 32]))
		return 0;

	sci_err = sc_pm_set_resource_power_mode(pm_ipc_handle, pd->rsrc_id,
		(power_on) ? SC_PM_PW_MODE_ON :
		pd->pd.state_idx ? SC_PM_PW_MODE_OFF : SC_PM_PW_MODE_LP);
	if (sci_err)
		pr_err("Failed power operation on resource %d\n", pd->rsrc_id);

	return 0;
}

static int imx8_pd_power_on(struct generic_pm_domain *domain)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk;
	int ret = 0;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	ret = imx8_pd_power(domain, true);

	if (!list_empty(&pd->clks) && (pd->pd.state_idx == PD_OFF)) {

		if (pd->clk_state_saved) {
			/*
			 * The SS is powered on restore the clock rates that
			 * may be lost.
			 */
			list_for_each_entry(imx8_rsrc_clk, &pd->clks, node) {

				if (imx8_rsrc_clk->parent)
					clk_set_parent(imx8_rsrc_clk->clk,
						imx8_rsrc_clk->parent);

				if (imx8_rsrc_clk->rate) {
					/*
					 * Need to read the clock so that rate in
					 * Linux is reset.
					 */
					clk_get_rate(imx8_rsrc_clk->clk);
					/* Restore the clock rate. */
					clk_set_rate(imx8_rsrc_clk->clk,
						imx8_rsrc_clk->rate);
				}
			}
		} else if (pd->clk_state_may_lost) {
			struct clk_stat *clk_stats;
			int count = 0;
			int i = 0;
			/*
			 * The SS is powered down before without saving clk rates,
			 * try to restore the lost clock rates if any
			 *
			 * As a parent clk rate restore will cause the clk recalc
			 * to all possible child clks which may result in child clk
			 * previous state lost due to power domain lost before,  we
			 * have to first walk through all child clks to retrieve the
			 * state via clk_hw_get_rate which bypassed the clk recalc,
			 * then we can restore them one by one.
			 */
			list_for_each_entry(imx8_rsrc_clk, &pd->clks, node)
				count++;

			clk_stats = kzalloc(count * sizeof(*clk_stats), GFP_KERNEL);
			if (!clk_stats) {
				pr_warn("%s: failed to alloc mem for clk state recovery\n", pd->name);
				return -ENOMEM;
			}

			list_for_each_entry(imx8_rsrc_clk, &pd->clks, node) {
				clk_stats[i].clk = imx8_rsrc_clk->clk;
				clk_stats[i].parent = clk_get_parent(imx8_rsrc_clk->clk);
				clk_stats[i].rate = clk_hw_get_rate(__clk_get_hw(imx8_rsrc_clk->clk));
				i++;
			}

			for (i = 0; i < count; i++) {
				/* restore parent first */
				if (clk_stats[i].parent)
					clk_set_parent(clk_stats[i].clk, clk_stats[i].parent);

				if (clk_stats[i].rate) {
					/* invalid cached rate first by get rate once */
					clk_get_rate(clk_stats[i].clk);
					/* restore the lost rate */
					clk_set_rate(clk_stats[i].clk, clk_stats[i].rate);
				}
			}

			kfree(clk_stats);
		}
	}

	return ret;
}

static int imx8_pd_power_off(struct generic_pm_domain *domain)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk;

	pd = container_of(domain, struct imx8_pm_domain, pd);

	if (!list_empty(&pd->clks) && (pd->pd.state_idx == PD_OFF)) {
		/*
		 * The SS is going to be powered off, store the clock rates
		 * that may be lost.
		 */
		list_for_each_entry(imx8_rsrc_clk, &pd->clks, node) {
			imx8_rsrc_clk->parent = clk_get_parent(imx8_rsrc_clk->clk);
			imx8_rsrc_clk->rate = clk_hw_get_rate(__clk_get_hw(imx8_rsrc_clk->clk));
		}
		pd->clk_state_saved = true;
		pd->clk_state_may_lost = false;
	} else if (pd->pd.state_idx == PD_OFF) {
		pd->clk_state_saved = false;
		pd->clk_state_may_lost = true;
	} else {
		pd->clk_state_saved = false;
		pd->clk_state_may_lost = false;
	}
	return imx8_pd_power(domain, false);
}

static int imx8_attach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	struct imx8_pm_domain *pd;
	struct device_node *node = dev->of_node;
	struct of_phandle_args clkspec;
	int rc, index, num_clks;

	pd = container_of(genpd, struct imx8_pm_domain, pd);

	num_clks = of_count_phandle_with_args(node, "assigned-clocks",
						"#clock-cells");
	if (num_clks == -EINVAL)
		pr_err("%s: Invalid value of assigned-clocks property at %s\n",
			pd->name, node->full_name);

	for (index = 0; index < num_clks; index++) {
		struct imx8_pm_rsrc_clks *imx8_rsrc_clk;

		rc = of_parse_phandle_with_args(node, "assigned-clocks",
					"#clock-cells", index, &clkspec);
		if (rc < 0) {
			/* skip empty (null) phandles */
			if (rc == -ENOENT)
				continue;
			else
				return rc;
		}
		if (clkspec.np == node)
			return 0;

		imx8_rsrc_clk = devm_kzalloc(dev, sizeof(*imx8_rsrc_clk),
					GFP_KERNEL);
		if (!imx8_rsrc_clk)
			return -ENOMEM;

		imx8_rsrc_clk->clk = of_clk_get_from_provider(&clkspec);
		if (!IS_ERR(imx8_rsrc_clk->clk))
			list_add_tail(&imx8_rsrc_clk->node, &pd->clks);
	}
	return 0;
}

static void imx8_detach_dev(struct generic_pm_domain *genpd, struct device *dev)
{
	struct imx8_pm_domain *pd;
	struct imx8_pm_rsrc_clks *imx8_rsrc_clk, *tmp;

	pd = container_of(genpd, struct imx8_pm_domain, pd);

	/* Free all the clock entry nodes. */
	if (list_empty(&pd->clks))
		return;

	list_for_each_entry_safe(imx8_rsrc_clk, tmp, &pd->clks, node) {
		list_del(&imx8_rsrc_clk->node);
		devm_kfree(dev, imx8_rsrc_clk);
	}
}

static void imx8_pm_domains_resume(void)
{
	sc_err_t sci_err = SC_ERR_NONE;
	int i;

	for (i = 0; i < (sizeof(early_power_on_rsrc) /
		sizeof(sc_rsrc_t)); i++) {
		if (early_power_on_rsrc[i] != SC_R_LAST) {
			sci_err = sc_pm_set_resource_power_mode(pm_ipc_handle,
				early_power_on_rsrc[i], SC_PM_PW_MODE_ON);
			if (sci_err != SC_ERR_NONE)
				pr_err("fail to power on resource %d\n",
					early_power_on_rsrc[i]);
		}
	}
}

struct syscore_ops imx8_pm_domains_syscore_ops = {
	.resume = imx8_pm_domains_resume,
};

static void imx8_pd_setup(struct imx8_pm_domain *pd)
{
	pd->pd.states = kzalloc(2 * sizeof(struct genpd_power_state), GFP_KERNEL);
	BUG_ON(!pd->pd.states);

	pd->pd.power_off = imx8_pd_power_off;
	pd->pd.power_on = imx8_pd_power_on;
	pd->pd.attach_dev = imx8_attach_dev;
	pd->pd.detach_dev = imx8_detach_dev;

	pd->pd.states[0].power_off_latency_ns = 25000;
	pd->pd.states[0].power_on_latency_ns =  25000;
	pd->pd.states[1].power_off_latency_ns = 2500000;
	pd->pd.states[1].power_on_latency_ns =  2500000;

	pd->pd.state_count = 2;
}

static int __init imx8_add_pm_domains(struct device_node *parent,
					struct generic_pm_domain *genpd_parent)
{
	struct device_node *np;
	static int index;

	for_each_child_of_node(parent, np) {
		struct imx8_pm_domain *imx8_pd;
		sc_rsrc_t rsrc_id;
		u32 wakeup_irq;

		if (!of_device_is_available(np))
			continue;

		imx8_pd = kzalloc(sizeof(*imx8_pd), GFP_KERNEL);
		if (!imx8_pd)
			return -ENOMEM;

		if (!of_property_read_string(np, "name", &imx8_pd->pd.name))
			imx8_pd->name = imx8_pd->pd.name;

		if (!of_property_read_u32(np, "reg", &rsrc_id))
			imx8_pd->rsrc_id = rsrc_id;

		if (imx8_pd->rsrc_id != SC_R_LAST) {
			imx8_pd_setup(imx8_pd);

			if (of_property_read_bool(np, "early_power_on")
				&& index < (sizeof(early_power_on_rsrc) /
				sizeof(sc_rsrc_t))) {
				early_power_on_rsrc[index++] = imx8_pd->rsrc_id;
			}
			if (of_property_read_bool(np, "debug_console"))
				rsrc_debug_console = imx8_pd->rsrc_id;
			if (!of_property_read_u32(np, "wakeup-irq",
				&wakeup_irq))
				irq2rsrc[wakeup_irq] = imx8_pd->rsrc_id;
		}
		INIT_LIST_HEAD(&imx8_pd->clks);
		pm_genpd_init(&imx8_pd->pd, NULL, true);

		if (genpd_parent)
			pm_genpd_add_subdomain(genpd_parent, &imx8_pd->pd);

		of_genpd_add_provider_simple(np, &imx8_pd->pd);

		imx8_add_pm_domains(np, &imx8_pd->pd);
	}
	return 0;
}

static int __init imx8_init_pm_domains(void)
{
	struct device_node *np;
	sc_err_t sci_err;
	sc_rsrc_t rsrc_id;
	uint32_t mu_id;

	/* skip pm domains for non-SCFW system */
	if (!of_find_compatible_node(NULL, NULL, "nxp,imx8-pd"))
		return 0;

	pr_info("***** imx8_init_pm_domains *****\n");

	for_each_compatible_node(np, NULL, "nxp,imx8-pd") {
		struct imx8_pm_domain *imx8_pd;

		if (!of_device_is_available(np))
			continue;

		imx8_pd = kzalloc(sizeof(struct imx8_pm_domain), GFP_KERNEL);
		if (!imx8_pd) {
			pr_err("%s: failed to allocate memory for domain\n",
				__func__);
			return -ENOMEM;
		}
		if (!of_property_read_string(np, "name", &imx8_pd->pd.name))
			imx8_pd->name = imx8_pd->pd.name;

		if (!of_property_read_u32(np, "reg", &rsrc_id))
			imx8_pd->rsrc_id = rsrc_id;

		if (imx8_pd->rsrc_id != SC_R_LAST)
			imx8_pd_setup(imx8_pd);

		INIT_LIST_HEAD(&imx8_pd->clks);

		pm_genpd_init(&imx8_pd->pd, NULL, true);
		of_genpd_add_provider_simple(np, &imx8_pd->pd);
		imx8_add_pm_domains(np, &imx8_pd->pd);
	}

	sci_err = sc_ipc_getMuID(&mu_id);
	if (sci_err != SC_ERR_NONE) {
		pr_info("Cannot obtain MU ID\n");
		return sci_err;
	}

	sci_err = sc_ipc_open(&pm_ipc_handle, mu_id);
	register_syscore_ops(&imx8_pm_domains_syscore_ops);

	return 0;
}

early_initcall(imx8_init_pm_domains);

static int imx8_wu_irq_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned int idx = irq2rsrc[d->hwirq] / 32;
	u32 mask  = 1 << irq2rsrc[d->hwirq] % 32;

	spin_lock(&imx8_wu_lock);
	wakeup_rsrc_id[idx] = on ? wakeup_rsrc_id[idx] | mask :
				wakeup_rsrc_id[idx] & ~mask;
	spin_unlock(&imx8_wu_lock);

	return 0;
}

static struct irq_chip imx8_wu_chip = {
	.name			= "IMX8-WU",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= irq_chip_mask_parent,
	.irq_unmask		= irq_chip_unmask_parent,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.irq_set_wake		= imx8_wu_irq_set_wake,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
};

static int imx8_wu_domain_translate(struct irq_domain *d,
				    struct irq_fwspec *fwspec,
				    unsigned long *hwirq,
				    unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;
		/* No PPI should point to this domain */
		if (fwspec->param[0] != 0)
			return -EINVAL;
		*hwirq = fwspec->param[1];
		*type = fwspec->param[2];
		return 0;
	}

	return -EINVAL;
}

static int imx8_wu_domain_alloc(struct irq_domain *domain,
				  unsigned int irq,
				  unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	int i;

	if (fwspec->param_count != 3)
		return -EINVAL;	/* Not GIC compliant */
	if (fwspec->param[0] != 0)
		return -EINVAL;	/* No PPI should point to this domain */

	hwirq = fwspec->param[1];
	if (hwirq >= IMX8_WU_MAX_IRQS)
		return -EINVAL;	/* Can't deal with this */

	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, irq + i, hwirq + i,
					      &imx8_wu_chip, NULL);

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;

	return irq_domain_alloc_irqs_parent(domain, irq, nr_irqs,
					    &parent_fwspec);
}

static const struct irq_domain_ops imx8_wu_domain_ops = {
	.translate = imx8_wu_domain_translate,
	.alloc	= imx8_wu_domain_alloc,
	.free	= irq_domain_free_irqs_common,
};

static int __init imx8_wu_init(struct device_node *node,
			       struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;

	if (!parent) {
		pr_err("%s: no parent, giving up\n", node->full_name);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%s: unable to obtain parent domain\n", node->full_name);
		return -ENXIO;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, IMX8_WU_MAX_IRQS,
					  node, &imx8_wu_domain_ops,
					  NULL);
	if (!domain)
		return -ENOMEM;

	return 0;
}
IRQCHIP_DECLARE(imx8_wakeup_unit, "fsl,imx8-wu", imx8_wu_init);

/***        debugfs support        ***/

#ifdef CONFIG_DEBUG_FS
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/kobject.h>

#define SC_PM_PW_MODE_FAIL	(SC_PM_PW_MODE_ON + 1)
static struct dentry *imx8_rsrc_pm_debugfs_dir;

static int imx8_rsrc_pm_summary_one(struct seq_file *s,
				sc_rsrc_t rsrc_id)
{
	static const char * const status_lookup[] = {
		[SC_PM_PW_MODE_OFF] = "OFF",
		[SC_PM_PW_MODE_STBY] = "STBY",
		[SC_PM_PW_MODE_LP] = "LP",
		[SC_PM_PW_MODE_ON] = "ON",
		[SC_PM_PW_MODE_FAIL] = "FAIL",
	};
	sc_err_t sci_err = SC_ERR_NONE;
	sc_pm_power_mode_t mode;
	char state[16];

	sci_err = sc_pm_get_resource_power_mode(pm_ipc_handle, rsrc_id, &mode);
	if (sci_err) {
		pr_debug("failed to get power mode on resource %d, ret %d\n",
			rsrc_id, sci_err);
		mode = SC_PM_PW_MODE_FAIL;
	}

	if (WARN_ON(mode >= ARRAY_SIZE(status_lookup)))
		return 0;

	snprintf(state, sizeof(state), "%s", status_lookup[mode]);
	seq_printf(s, "%-30d  %-15s ", rsrc_id, state);
	seq_puts(s, "\n");

	return 0;
}

static int imx8_rsrc_pm_summary_show(struct seq_file *s, void *data)
{
	int ret = 0;
	int i;

	seq_puts(s, "resource_id                    power_mode\n");
	seq_puts(s, "---------------------------------------------\n");

	ret = mutex_lock_interruptible(&rsrc_pm_list_lock);
	if (ret)
		return -ERESTARTSYS;

	for (i = 0; i < SC_R_LAST; i++) {
		ret = imx8_rsrc_pm_summary_one(s, i);
		if (ret)
			break;
	}
	mutex_unlock(&rsrc_pm_list_lock);

	return ret;
}

static int imx8_rsrc_pm_summary_open(struct inode *inode, struct file *file)
{
	return single_open(file, imx8_rsrc_pm_summary_show, NULL);
}

static const struct file_operations imx8_rsrc_pm_summary_fops = {
	.open = imx8_rsrc_pm_summary_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init imx8_pm_debug_init(void)
{
	struct dentry *d;

	/* skip for non-SCFW system */
	if (!of_find_compatible_node(NULL, NULL, "nxp,imx8-pd"))
		return 0;

	imx8_rsrc_pm_debugfs_dir = debugfs_create_dir("imx_rsrc_pm", NULL);

	if (!imx8_rsrc_pm_debugfs_dir)
		return -ENOMEM;

	d = debugfs_create_file("imx_rsrc_pm_summary", 0444,
			imx8_rsrc_pm_debugfs_dir, NULL,
			&imx8_rsrc_pm_summary_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
late_initcall(imx8_pm_debug_init);

static void __exit imx8_rsrc_pm_debug_exit(void)
{
	debugfs_remove_recursive(imx8_rsrc_pm_debugfs_dir);
}
__exitcall(imx8_rsrc_pm_debug_exit);
#endif /* CONFIG_DEBUG_FS */
