/* Copyright 2008-2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "bman_low.h"
#ifdef CONFIG_HOTPLUG_CPU
#include <linux/cpu.h>
#endif
/*
 * Global variables of the max portal/pool number this bman version supported
 */
u16 bman_ip_rev;
EXPORT_SYMBOL(bman_ip_rev);
u16 bman_pool_max;
EXPORT_SYMBOL(bman_pool_max);
static u16 bman_portal_max;

/* After initialising cpus that own shared portal configs, we cache the
 * resulting portals (ie. not just the configs) in this array. Then we
 * initialise slave cpus that don't have their own portals, redirecting them to
 * portals from this cache in a round-robin assignment. */
static struct bman_portal *shared_portals[NR_CPUS];
static int num_shared_portals;
static int shared_portals_idx;
static LIST_HEAD(unused_pcfgs);
static DEFINE_SPINLOCK(unused_pcfgs_lock);
static void *affine_bportals[NR_CPUS];

static int __init fsl_bpool_init(struct device_node *node)
{
	int ret;
	u32 *thresh, *bpid = (u32 *)of_get_property(node, "fsl,bpid", &ret);
	if (!bpid || (ret != 4)) {
		pr_err("Can't get %s property 'fsl,bpid'\n", node->full_name);
		return -ENODEV;
	}
	thresh = (u32 *)of_get_property(node, "fsl,bpool-thresholds", &ret);
	if (thresh) {
		if (ret != 16) {
			pr_err("Invalid %s property '%s'\n",
				node->full_name, "fsl,bpool-thresholds");
			return -ENODEV;
		}
	}
	if (thresh) {
#ifdef CONFIG_FSL_BMAN_CONFIG
		ret = bm_pool_set(be32_to_cpu(*bpid), thresh);
		if (ret)
			pr_err("No CCSR node for %s property '%s'\n",
				node->full_name, "fsl,bpool-thresholds");
		return ret;
#else
		pr_err("Ignoring %s property '%s', no CCSR support\n",
			node->full_name, "fsl,bpool-thresholds");
#endif
	}
	return 0;
}

static int __init fsl_bpid_range_init(struct device_node *node)
{
	int ret;
	u32 *range = (u32 *)of_get_property(node, "fsl,bpid-range", &ret);
	if (!range) {
		pr_err("No 'fsl,bpid-range' property in node %s\n",
			node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("'fsl,bpid-range' is not a 2-cell range in node %s\n",
			node->full_name);
		return -EINVAL;
	}
	bman_seed_bpid_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	pr_info("Bman: BPID allocator includes range %d:%d\n",
		be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	return 0;
}

static struct bm_portal_config * __init parse_pcfg(struct device_node *node)
{
	struct bm_portal_config *pcfg;
	const u32 *index;
	int irq, ret;
	resource_size_t len;

	pcfg = kmalloc(sizeof(*pcfg), GFP_KERNEL);
	if (!pcfg) {
		pr_err("can't allocate portal config");
		return NULL;
	}

	if (of_device_is_compatible(node, "fsl,bman-portal-1.0") ||
		of_device_is_compatible(node, "fsl,bman-portal-1.0.0")) {
		bman_ip_rev = BMAN_REV10;
		bman_pool_max = 64;
		bman_portal_max = 10;
	} else if (of_device_is_compatible(node, "fsl,bman-portal-2.0") ||
		of_device_is_compatible(node, "fsl,bman-portal-2.0.8")) {
		bman_ip_rev = BMAN_REV20;
		bman_pool_max = 8;
		bman_portal_max = 3;
	} else if (of_device_is_compatible(node, "fsl,bman-portal-2.1.0")) {
		bman_ip_rev = BMAN_REV21;
		bman_pool_max = 64;
		bman_portal_max = 50;
	} else if (of_device_is_compatible(node, "fsl,bman-portal-2.1.1")) {
		bman_ip_rev = BMAN_REV21;
		bman_pool_max = 64;
		bman_portal_max = 25;
	} else if (of_device_is_compatible(node, "fsl,bman-portal-2.1.2")) {
		bman_ip_rev = BMAN_REV21;
		bman_pool_max = 64;
		bman_portal_max = 18;
	} else if (of_device_is_compatible(node, "fsl,bman-portal-2.1.3")) {
		bman_ip_rev = BMAN_REV21;
		bman_pool_max = 64;
		bman_portal_max = 10;
	} else {
		pr_warn("unknown BMan version in portal node,"
			"default to rev1.0\n");
		bman_ip_rev = BMAN_REV10;
		bman_pool_max = 64;
		bman_portal_max = 10;
	}

	ret = of_address_to_resource(node, DPA_PORTAL_CE,
				&pcfg->addr_phys[DPA_PORTAL_CE]);
	if (ret) {
		pr_err("Can't get %s property 'reg::CE'\n", node->full_name);
		goto err;
	}
	ret = of_address_to_resource(node, DPA_PORTAL_CI,
				&pcfg->addr_phys[DPA_PORTAL_CI]);
	if (ret) {
		pr_err("Can't get %s property 'reg::CI'\n", node->full_name);
		goto err;
	}

	index = of_get_property(node, "cell-index", &ret);
	if (!index || (ret != 4)) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"cell-index");
		goto err;
	}
	if (be32_to_cpu(*index) >= bman_portal_max) {
		pr_err("BMan portal cell index %d out of range, max %d\n",
		       be32_to_cpu(*index), bman_portal_max);
		goto err;
	}

	pcfg->public_cfg.cpu = -1;

	irq = irq_of_parse_and_map(node, 0);
	if (irq == 0) {
		pr_err("Can't get %s property 'interrupts'\n", node->full_name);
		goto err;
	}
	pcfg->public_cfg.irq = irq;
	pcfg->public_cfg.index = be32_to_cpu(*index);
	bman_depletion_fill(&pcfg->public_cfg.mask);

	len = resource_size(&pcfg->addr_phys[DPA_PORTAL_CE]);
	if (len != (unsigned long)len)
		goto err;

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	pcfg->addr_virt[DPA_PORTAL_CE] = ioremap_cache_ns(
                                pcfg->addr_phys[DPA_PORTAL_CE].start,
                                resource_size(&pcfg->addr_phys[DPA_PORTAL_CE]));
        pcfg->addr_virt[DPA_PORTAL_CI] = ioremap(
                                pcfg->addr_phys[DPA_PORTAL_CI].start,
                                resource_size(&pcfg->addr_phys[DPA_PORTAL_CI]));

#else
	pcfg->addr_virt[DPA_PORTAL_CE] =
		memremap(pcfg->addr_phys[DPA_PORTAL_CE].start,
			 (unsigned long)len, MEMREMAP_WB);

	pcfg->addr_virt[DPA_PORTAL_CI] =
		ioremap(pcfg->addr_phys[DPA_PORTAL_CI].start,
			resource_size(&pcfg->addr_phys[DPA_PORTAL_CI]));

#endif
	/* disable bp depletion */
	__raw_writel(0x0, pcfg->addr_virt[DPA_PORTAL_CI] + BM_REG_SCN(0));
	__raw_writel(0x0, pcfg->addr_virt[DPA_PORTAL_CI] + BM_REG_SCN(1));
	return pcfg;
err:
	kfree(pcfg);
	return NULL;
}

static struct bm_portal_config *get_pcfg(struct list_head *list)
{
	struct bm_portal_config *pcfg;
	if (list_empty(list))
		return NULL;
	pcfg = list_entry(list->prev, struct bm_portal_config, list);
	list_del(&pcfg->list);
	return pcfg;
}

static struct bm_portal_config *get_pcfg_idx(struct list_head *list,
					     uint32_t idx)
{
	struct bm_portal_config *pcfg;
	if (list_empty(list))
		return NULL;
	list_for_each_entry(pcfg, list, list) {
		if (pcfg->public_cfg.index == idx) {
			list_del(&pcfg->list);
			return pcfg;
		}
	}
	return NULL;
}

struct bm_portal_config *bm_get_unused_portal(void)
{
	return bm_get_unused_portal_idx(QBMAN_ANY_PORTAL_IDX);
}

struct bm_portal_config *bm_get_unused_portal_idx(uint32_t idx)
{
	struct bm_portal_config *ret;
	spin_lock(&unused_pcfgs_lock);
	if (idx == QBMAN_ANY_PORTAL_IDX)
		ret = get_pcfg(&unused_pcfgs);
	else
		ret = get_pcfg_idx(&unused_pcfgs, idx);
	spin_unlock(&unused_pcfgs_lock);
	return ret;
}

void bm_put_unused_portal(struct bm_portal_config *pcfg)
{
	spin_lock(&unused_pcfgs_lock);
	list_add(&pcfg->list, &unused_pcfgs);
	spin_unlock(&unused_pcfgs_lock);
}

static struct bman_portal *init_pcfg(struct bm_portal_config *pcfg)
{
	struct bman_portal *p;
	p = bman_create_affine_portal(pcfg);
	if (p) {
#ifdef CONFIG_FSL_DPA_PIRQ_SLOW
		bman_p_irqsource_add(p, BM_PIRQ_RCRI | BM_PIRQ_BSCN);
#endif
		pr_info("Bman portal %sinitialised, cpu %d\n",
			pcfg->public_cfg.is_shared ? "(shared) " : "",
			pcfg->public_cfg.cpu);
		affine_bportals[pcfg->public_cfg.cpu] = p;
	} else
		pr_crit("Bman portal failure on cpu %d\n",
			pcfg->public_cfg.cpu);
	return p;
}

static void init_slave(int cpu)
{
	struct bman_portal *p;
	p = bman_create_affine_slave(shared_portals[shared_portals_idx++], cpu);
	if (!p)
		pr_err("Bman slave portal failure on cpu %d\n", cpu);
	else
		pr_info("Bman portal %sinitialised, cpu %d\n", "(slave) ", cpu);
	if (shared_portals_idx >= num_shared_portals)
		shared_portals_idx = 0;
	affine_bportals[cpu] = p;
}

/* Bootarg "bportals=[...]" has the same syntax as "qportals=", and so the
 * parsing is in dpa_sys.h. The syntax is a comma-separated list of indexes
 * and/or ranges of indexes, with each being optionally prefixed by "s" to
 * explicitly mark it or them for sharing.
 *    Eg;
 *	  bportals=s0,1-3,s4
 * means that cpus 1,2,3 get "unshared" portals, cpus 0 and 4 get "shared"
 * portals, and any remaining cpus share the portals that are assigned to cpus 0
 * or 4, selected in a round-robin fashion. (In this example, cpu 5 would share
 * cpu 0's portal, cpu 6 would share cpu4's portal, and cpu 7 would share cpu
 * 0's portal.) */
static struct cpumask want_unshared __initdata; /* cpus requested without "s" */
static struct cpumask want_shared __initdata; /* cpus requested with "s" */

static int __init parse_bportals(char *str)
{
	return parse_portals_bootarg(str, &want_shared, &want_unshared,
				     "bportals");
}
__setup("bportals=", parse_bportals);

static int bman_offline_cpu(unsigned int cpu)
{
	struct bman_portal *p;
	const struct bm_portal_config *pcfg;
	p = (struct bman_portal *)affine_bportals[cpu];
	if (p) {
		pcfg = bman_get_bm_portal_config(p);
		if (pcfg)
			irq_set_affinity(pcfg->public_cfg.irq, cpumask_of(0));
	}
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int bman_online_cpu(unsigned int cpu)
{
	struct bman_portal *p;
	const struct bm_portal_config *pcfg;
	p = (struct bman_portal *)affine_bportals[cpu];
	if (p) {
		pcfg = bman_get_bm_portal_config(p);
		if (pcfg)
			irq_set_affinity(pcfg->public_cfg.irq, cpumask_of(cpu));
	}
	return 0;
}
#endif /* CONFIG_HOTPLUG_CPU */

/* Initialise the Bman driver. The meat of this function deals with portals. The
 * following describes the flow of portal-handling, the code "steps" refer to
 * this description;
 * 1. Portal configs are parsed from the device-tree into 'unused_pcfgs', with
 *    ::cpu==-1. Regions and interrupts are mapped (but interrupts are not
 *    bound).
 * 2. The "want_shared" and "want_unshared" lists (as filled by the
 *    "bportals=[...]" bootarg) are processed, allocating portals and assigning
 *    them to cpus, placing them in the relevant list and setting ::cpu as
 *    appropriate. If no "bportals" bootarg was present, the defaut is to try to
 *    assign portals to all online cpus at the time of driver initialisation.
 *    Any failure to allocate portals (when parsing the "want" lists or when
 *    using default behaviour) will be silently tolerated (the "fixup" logic in
 *    step 3 will determine what happens in this case).
 * 3. Do fixups relative to cpu_online_mask(). If no portals are marked for
 *    sharing and sharing is required (because not all cpus have been assigned
 *    portals), then one portal will marked for sharing. Conversely if no
 *    sharing is required, any portals marked for sharing will not be shared. It
 *    may be that sharing occurs when it wasn't expected, if portal allocation
 *    failed to honour all the requested assignments (including the default
 *    assignments if no bootarg is present).
 * 4. Unshared portals are initialised on their respective cpus.
 * 5. Shared portals are initialised on their respective cpus.
 * 6. Each remaining cpu is initialised to slave to one of the shared portals,
 *    which are selected in a round-robin fashion.
 * Any portal configs left unused are available for USDPAA allocation.
 */
__init int bman_init(void)
{
	struct cpumask slave_cpus;
	struct cpumask unshared_cpus = *cpu_none_mask;
	struct cpumask shared_cpus = *cpu_none_mask;
	LIST_HEAD(unshared_pcfgs);
	LIST_HEAD(shared_pcfgs);
	struct device_node *dn;
	struct bm_portal_config *pcfg;
	struct bman_portal *p;
	int cpu, ret;
	struct cpumask offline_cpus;

	/* Initialise the Bman (CCSR) device */
	for_each_compatible_node(dn, NULL, "fsl,bman") {
		if (!bman_init_ccsr(dn))
			pr_info("Bman err interrupt handler present\n");
		else
			pr_err("Bman CCSR setup failed\n");
	}
	/* Initialise any declared buffer pools */
	for_each_compatible_node(dn, NULL, "fsl,bpool") {
		ret = fsl_bpool_init(dn);
		if (ret)
			return ret;
	}
	/* Step 1. See comments at the beginning of the file. */
	for_each_compatible_node(dn, NULL, "fsl,bman-portal") {
		if (!of_device_is_available(dn))
			continue;
		pcfg = parse_pcfg(dn);
		if (pcfg)
			list_add_tail(&pcfg->list, &unused_pcfgs);
	}
	/* Step 2. */
	for_each_possible_cpu(cpu) {
		if (cpumask_test_cpu(cpu, &want_shared)) {
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &shared_pcfgs);
			cpumask_set_cpu(cpu, &shared_cpus);
		}
		if (cpumask_test_cpu(cpu, &want_unshared)) {
			if (cpumask_test_cpu(cpu, &shared_cpus))
				continue;
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &unshared_pcfgs);
			cpumask_set_cpu(cpu, &unshared_cpus);
		}
	}
	if (list_empty(&shared_pcfgs) && list_empty(&unshared_pcfgs)) {
		/* Default, give an unshared portal to each online cpu */
		for_each_online_cpu(cpu) {
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &unshared_pcfgs);
			cpumask_set_cpu(cpu, &unshared_cpus);
		}
	}
	/* Step 3. */
	cpumask_andnot(&slave_cpus, cpu_possible_mask, &shared_cpus);
	cpumask_andnot(&slave_cpus, &slave_cpus, &unshared_cpus);
	if (cpumask_empty(&slave_cpus)) {
		/* No sharing required */
		if (!list_empty(&shared_pcfgs)) {
			/* Migrate "shared" to "unshared" */
			cpumask_or(&unshared_cpus, &unshared_cpus,
				   &shared_cpus);
			cpumask_clear(&shared_cpus);
			list_splice_tail(&shared_pcfgs, &unshared_pcfgs);
			INIT_LIST_HEAD(&shared_pcfgs);
		}
	} else {
		/* Sharing required */
		if (list_empty(&shared_pcfgs)) {
			/* Migrate one "unshared" to "shared" */
			pcfg = get_pcfg(&unshared_pcfgs);
			if (!pcfg) {
				pr_crit("No BMan portals available!\n");
				return 0;
			}
			cpumask_clear_cpu(pcfg->public_cfg.cpu, &unshared_cpus);
			cpumask_set_cpu(pcfg->public_cfg.cpu, &shared_cpus);
			list_add_tail(&pcfg->list, &shared_pcfgs);
		}
	}
	/* Step 4. */
	list_for_each_entry(pcfg, &unshared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 0;
		p = init_pcfg(pcfg);
		if (!p) {
			pr_crit("Unable to initialize bman portal\n");
			return 0;
		}
	}
	/* Step 5. */
	list_for_each_entry(pcfg, &shared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 1;
		p = init_pcfg(pcfg);
		if (p)
			shared_portals[num_shared_portals++] = p;
	}
	/* Step 6. */
	if (!cpumask_empty(&slave_cpus))
		for_each_cpu(cpu, &slave_cpus)
			init_slave(cpu);
	pr_info("Bman portals initialised\n");
	cpumask_andnot(&offline_cpus, cpu_possible_mask, cpu_online_mask);
	for_each_cpu(cpu, &offline_cpus)
		bman_offline_cpu(cpu);
#ifdef CONFIG_HOTPLUG_CPU
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					"soc/qbman_portal:online",
					bman_online_cpu, bman_offline_cpu);
	if (ret < 0) {
		pr_err("bman: failed to register hotplug callbacks.\n");
		return 0;
	}
#endif
	return 0;
}

__init int bman_resource_init(void)
{
	struct device_node *dn;
	int ret;

	/* Initialise BPID allocation ranges */
	for_each_compatible_node(dn, NULL, "fsl,bpid-range") {
		ret = fsl_bpid_range_init(dn);
		if (ret)
			return ret;
	}
	return 0;
}

#ifdef CONFIG_SUSPEND
void suspend_unused_bportal(void)
{
	struct bm_portal_config *pcfg;

	if (list_empty(&unused_pcfgs))
		return;

	list_for_each_entry(pcfg, &unused_pcfgs, list) {
#ifdef CONFIG_PM_DEBUG
		pr_info("Need to save bportal %d\n", pcfg->public_cfg.index);
#endif
		/* save isdr, disable all via isdr, clear isr */
		pcfg->saved_isdr =
			__raw_readl(pcfg->addr_virt[DPA_PORTAL_CI] + 0xe08);
		__raw_writel(0xffffffff, pcfg->addr_virt[DPA_PORTAL_CI] +
					0xe08);
		__raw_writel(0xffffffff, pcfg->addr_virt[DPA_PORTAL_CI] +
					0xe00);
	}
	return;
}

void resume_unused_bportal(void)
{
	struct bm_portal_config *pcfg;

	if (list_empty(&unused_pcfgs))
		return;

	list_for_each_entry(pcfg, &unused_pcfgs, list) {
#ifdef CONFIG_PM_DEBUG
		pr_info("Need to resume bportal %d\n", pcfg->public_cfg.index);
#endif
		/* restore isdr */
		__raw_writel(pcfg->saved_isdr,
				pcfg->addr_virt[DPA_PORTAL_CI] + 0xe08);
	}
	return;
}
#endif
