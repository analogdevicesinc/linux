// SPDX-License-Identifier: GPL-2.0-only
/* Common code for 32 and 64-bit NUMA */
#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/mmzone.h>
#include <linux/ctype.h>
#include <linux/nodemask.h>
#include <linux/sched.h>
#include <linux/topology.h>
#include <linux/sort.h>
#include <linux/numa_memblks.h>

#include <asm/e820/api.h>
#include <asm/proto.h>
#include <asm/dma.h>
#include <asm/amd_nb.h>

#include "numa_internal.h"

int numa_off;

static __init int numa_setup(char *opt)
{
	if (!opt)
		return -EINVAL;
	if (!strncmp(opt, "off", 3))
		numa_off = 1;
	if (!strncmp(opt, "fake=", 5))
		return numa_emu_cmdline(opt + 5);
	if (!strncmp(opt, "noacpi", 6))
		disable_srat();
	if (!strncmp(opt, "nohmat", 6))
		disable_hmat();
	return 0;
}
early_param("numa", numa_setup);

/*
 * apicid, cpu, node mappings
 */
s16 __apicid_to_node[MAX_LOCAL_APIC] = {
	[0 ... MAX_LOCAL_APIC-1] = NUMA_NO_NODE
};

int numa_cpu_node(int cpu)
{
	u32 apicid = early_per_cpu(x86_cpu_to_apicid, cpu);

	if (apicid != BAD_APICID)
		return __apicid_to_node[apicid];
	return NUMA_NO_NODE;
}

cpumask_var_t node_to_cpumask_map[MAX_NUMNODES];
EXPORT_SYMBOL(node_to_cpumask_map);

/*
 * Map cpu index to node index
 */
DEFINE_EARLY_PER_CPU(int, x86_cpu_to_node_map, NUMA_NO_NODE);
EXPORT_EARLY_PER_CPU_SYMBOL(x86_cpu_to_node_map);

void numa_set_node(int cpu, int node)
{
	int *cpu_to_node_map = early_per_cpu_ptr(x86_cpu_to_node_map);

	/* early setting, no percpu area yet */
	if (cpu_to_node_map) {
		cpu_to_node_map[cpu] = node;
		return;
	}

#ifdef CONFIG_DEBUG_PER_CPU_MAPS
	if (cpu >= nr_cpu_ids || !cpu_possible(cpu)) {
		printk(KERN_ERR "numa_set_node: invalid cpu# (%d)\n", cpu);
		dump_stack();
		return;
	}
#endif
	per_cpu(x86_cpu_to_node_map, cpu) = node;

	set_cpu_numa_node(cpu, node);
}

void numa_clear_node(int cpu)
{
	numa_set_node(cpu, NUMA_NO_NODE);
}

/*
 * Allocate node_to_cpumask_map based on number of available nodes
 * Requires node_possible_map to be valid.
 *
 * Note: cpumask_of_node() is not valid until after this is done.
 * (Use CONFIG_DEBUG_PER_CPU_MAPS to check this.)
 */
void __init setup_node_to_cpumask_map(void)
{
	unsigned int node;

	/* setup nr_node_ids if not done yet */
	if (nr_node_ids == MAX_NUMNODES)
		setup_nr_node_ids();

	/* allocate the map */
	for (node = 0; node < nr_node_ids; node++)
		alloc_bootmem_cpumask_var(&node_to_cpumask_map[node]);

	/* cpumask_of_node() will now work */
	pr_debug("Node to cpumask map for %u nodes\n", nr_node_ids);
}

static int __init numa_register_nodes(void)
{
	int nid;

	if (!memblock_validate_numa_coverage(SZ_1M))
		return -EINVAL;

	/* Finally register nodes. */
	for_each_node_mask(nid, node_possible_map) {
		unsigned long start_pfn, end_pfn;

		/*
		 * Note, get_pfn_range_for_nid() depends on
		 * memblock_set_node() having already happened
		 */
		get_pfn_range_for_nid(nid, &start_pfn, &end_pfn);
		if (start_pfn >= end_pfn)
			continue;

		alloc_node_data(nid);
		node_set_online(nid);
	}

	/* Dump memblock with node info and return. */
	memblock_dump_all();
	return 0;
}

/*
 * There are unfortunately some poorly designed mainboards around that
 * only connect memory to a single CPU. This breaks the 1:1 cpu->node
 * mapping. To avoid this fill in the mapping for all possible CPUs,
 * as the number of CPUs is not known yet. We round robin the existing
 * nodes.
 */
static void __init numa_init_array(void)
{
	int rr, i;

	rr = first_node(node_online_map);
	for (i = 0; i < nr_cpu_ids; i++) {
		if (early_cpu_to_node(i) != NUMA_NO_NODE)
			continue;
		numa_set_node(i, rr);
		rr = next_node_in(rr, node_online_map);
	}
}

static int __init numa_init(int (*init_func)(void))
{
	int i;
	int ret;

	for (i = 0; i < MAX_LOCAL_APIC; i++)
		set_apicid_to_node(i, NUMA_NO_NODE);

	ret = numa_memblks_init(init_func, /* memblock_force_top_down */ true);
	if (ret < 0)
		return ret;

	ret = numa_register_nodes();
	if (ret < 0)
		return ret;

	for (i = 0; i < nr_cpu_ids; i++) {
		int nid = early_cpu_to_node(i);

		if (nid == NUMA_NO_NODE)
			continue;
		if (!node_online(nid))
			numa_clear_node(i);
	}
	numa_init_array();

	return 0;
}

/**
 * dummy_numa_init - Fallback dummy NUMA init
 *
 * Used if there's no underlying NUMA architecture, NUMA initialization
 * fails, or NUMA is disabled on the command line.
 *
 * Must online at least one node and add memory blocks that cover all
 * allowed memory.  This function must not fail.
 */
static int __init dummy_numa_init(void)
{
	printk(KERN_INFO "%s\n",
	       numa_off ? "NUMA turned off" : "No NUMA configuration found");
	printk(KERN_INFO "Faking a node at [mem %#018Lx-%#018Lx]\n",
	       0LLU, PFN_PHYS(max_pfn) - 1);

	node_set(0, numa_nodes_parsed);
	numa_add_memblk(0, 0, PFN_PHYS(max_pfn));

	return 0;
}

/**
 * x86_numa_init - Initialize NUMA
 *
 * Try each configured NUMA initialization method until one succeeds.  The
 * last fallback is dummy single node config encompassing whole memory and
 * never fails.
 */
void __init x86_numa_init(void)
{
	if (!numa_off) {
#ifdef CONFIG_ACPI_NUMA
		if (!numa_init(x86_acpi_numa_init))
			return;
#endif
#ifdef CONFIG_AMD_NUMA
		if (!numa_init(amd_numa_init))
			return;
#endif
		if (acpi_disabled && !numa_init(of_numa_init))
			return;
	}

	numa_init(dummy_numa_init);
}


/*
 * A node may exist which has one or more Generic Initiators but no CPUs and no
 * memory.
 *
 * This function must be called after init_cpu_to_node(), to ensure that any
 * memoryless CPU nodes have already been brought online, and before the
 * node_data[nid] is needed for zone list setup in build_all_zonelists().
 *
 * When this function is called, any nodes containing either memory and/or CPUs
 * will already be online and there is no need to do anything extra, even if
 * they also contain one or more Generic Initiators.
 */
void __init init_gi_nodes(void)
{
	int nid;

	/*
	 * Exclude this node from
	 * bringup_nonboot_cpus
	 *  cpu_up
	 *   __try_online_node
	 *    register_one_node
	 * because node_subsys is not initialized yet.
	 * TODO remove dependency on node_online
	 */
	for_each_node_state(nid, N_GENERIC_INITIATOR)
		if (!node_online(nid))
			node_set_online(nid);
}

/*
 * Setup early cpu_to_node.
 *
 * Populate cpu_to_node[] only if x86_cpu_to_apicid[],
 * and apicid_to_node[] tables have valid entries for a CPU.
 * This means we skip cpu_to_node[] initialisation for NUMA
 * emulation and faking node case (when running a kernel compiled
 * for NUMA on a non NUMA box), which is OK as cpu_to_node[]
 * is already initialized in a round robin manner at numa_init_array,
 * prior to this call, and this initialization is good enough
 * for the fake NUMA cases.
 *
 * Called before the per_cpu areas are setup.
 */
void __init init_cpu_to_node(void)
{
	int cpu;
	u32 *cpu_to_apicid = early_per_cpu_ptr(x86_cpu_to_apicid);

	BUG_ON(cpu_to_apicid == NULL);

	for_each_possible_cpu(cpu) {
		int node = numa_cpu_node(cpu);

		if (node == NUMA_NO_NODE)
			continue;

		/*
		 * Exclude this node from
		 * bringup_nonboot_cpus
		 *  cpu_up
		 *   __try_online_node
		 *    register_one_node
		 * because node_subsys is not initialized yet.
		 * TODO remove dependency on node_online
		 */
		if (!node_online(node))
			node_set_online(node);

		numa_set_node(cpu, node);
	}
}

#ifndef CONFIG_DEBUG_PER_CPU_MAPS

# ifndef CONFIG_NUMA_EMU
void numa_add_cpu(unsigned int cpu)
{
	cpumask_set_cpu(cpu, node_to_cpumask_map[early_cpu_to_node(cpu)]);
}

void numa_remove_cpu(unsigned int cpu)
{
	cpumask_clear_cpu(cpu, node_to_cpumask_map[early_cpu_to_node(cpu)]);
}
# endif	/* !CONFIG_NUMA_EMU */

#else	/* !CONFIG_DEBUG_PER_CPU_MAPS */

int __cpu_to_node(int cpu)
{
	if (early_per_cpu_ptr(x86_cpu_to_node_map)) {
		printk(KERN_WARNING
			"cpu_to_node(%d): usage too early!\n", cpu);
		dump_stack();
		return early_per_cpu_ptr(x86_cpu_to_node_map)[cpu];
	}
	return per_cpu(x86_cpu_to_node_map, cpu);
}
EXPORT_SYMBOL(__cpu_to_node);

/*
 * Same function as cpu_to_node() but used if called before the
 * per_cpu areas are setup.
 */
int early_cpu_to_node(int cpu)
{
	if (early_per_cpu_ptr(x86_cpu_to_node_map))
		return early_per_cpu_ptr(x86_cpu_to_node_map)[cpu];

	if (!cpu_possible(cpu)) {
		printk(KERN_WARNING
			"early_cpu_to_node(%d): no per_cpu area!\n", cpu);
		dump_stack();
		return NUMA_NO_NODE;
	}
	return per_cpu(x86_cpu_to_node_map, cpu);
}

void debug_cpumask_set_cpu(unsigned int cpu, int node, bool enable)
{
	struct cpumask *mask;

	if (node == NUMA_NO_NODE) {
		/* early_cpu_to_node() already emits a warning and trace */
		return;
	}
	mask = node_to_cpumask_map[node];
	if (!cpumask_available(mask)) {
		pr_err("node_to_cpumask_map[%i] NULL\n", node);
		dump_stack();
		return;
	}

	if (enable)
		cpumask_set_cpu(cpu, mask);
	else
		cpumask_clear_cpu(cpu, mask);

	printk(KERN_DEBUG "%s cpu %d node %d: mask now %*pbl\n",
		enable ? "numa_add_cpu" : "numa_remove_cpu",
		cpu, node, cpumask_pr_args(mask));
	return;
}

# ifndef CONFIG_NUMA_EMU
static void numa_set_cpumask(int cpu, bool enable)
{
	debug_cpumask_set_cpu(cpu, early_cpu_to_node(cpu), enable);
}

void numa_add_cpu(unsigned int cpu)
{
	numa_set_cpumask(cpu, true);
}

void numa_remove_cpu(unsigned int cpu)
{
	numa_set_cpumask(cpu, false);
}
# endif	/* !CONFIG_NUMA_EMU */

/*
 * Returns a pointer to the bitmask of CPUs on Node 'node'.
 */
const struct cpumask *cpumask_of_node(int node)
{
	if ((unsigned)node >= nr_node_ids) {
		printk(KERN_WARNING
			"cpumask_of_node(%d): (unsigned)node >= nr_node_ids(%u)\n",
			node, nr_node_ids);
		dump_stack();
		return cpu_none_mask;
	}
	if (!cpumask_available(node_to_cpumask_map[node])) {
		printk(KERN_WARNING
			"cpumask_of_node(%d): no node_to_cpumask_map!\n",
			node);
		dump_stack();
		return cpu_online_mask;
	}
	return node_to_cpumask_map[node];
}
EXPORT_SYMBOL(cpumask_of_node);

#endif	/* !CONFIG_DEBUG_PER_CPU_MAPS */

#ifdef CONFIG_NUMA_EMU
void __init numa_emu_update_cpu_to_node(int *emu_nid_to_phys,
					unsigned int nr_emu_nids)
{
	int i, j;

	/*
	 * Transform __apicid_to_node table to use emulated nids by
	 * reverse-mapping phys_nid.  The maps should always exist but fall
	 * back to zero just in case.
	 */
	for (i = 0; i < ARRAY_SIZE(__apicid_to_node); i++) {
		if (__apicid_to_node[i] == NUMA_NO_NODE)
			continue;
		for (j = 0; j < nr_emu_nids; j++)
			if (__apicid_to_node[i] == emu_nid_to_phys[j])
				break;
		__apicid_to_node[i] = j < nr_emu_nids ? j : 0;
	}
}

u64 __init numa_emu_dma_end(void)
{
	return PFN_PHYS(MAX_DMA32_PFN);
}
#endif /* CONFIG_NUMA_EMU */
