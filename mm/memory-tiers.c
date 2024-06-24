// SPDX-License-Identifier: GPL-2.0
#include <linux/slab.h>
#include <linux/lockdep.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/memory.h>
#include <linux/memory-tiers.h>
#include <linux/notifier.h>

#include "internal.h"

struct memory_tier {
	/* hierarchy of memory tiers */
	struct list_head list;
	/* list of all memory types part of this tier */
	struct list_head memory_types;
	/*
	 * start value of abstract distance. memory tier maps
	 * an abstract distance  range,
	 * adistance_start .. adistance_start + MEMTIER_CHUNK_SIZE
	 */
	int adistance_start;
	struct device dev;
	/* All the nodes that are part of all the lower memory tiers. */
	nodemask_t lower_tier_mask;
};

struct demotion_nodes {
	nodemask_t preferred;
};

struct node_memory_type_map {
	struct memory_dev_type *memtype;
	int map_count;
};

static DEFINE_MUTEX(memory_tier_lock);
static LIST_HEAD(memory_tiers);
/*
 * The list is used to store all memory types that are not created
 * by a device driver.
 */
static LIST_HEAD(default_memory_types);
static struct node_memory_type_map node_memory_types[MAX_NUMNODES];
struct memory_dev_type *default_dram_type;

static const struct bus_type memory_tier_subsys = {
	.name = "memory_tiering",
	.dev_name = "memory_tier",
};

#ifdef CONFIG_MIGRATION
static int top_tier_adistance;
/*
 * node_demotion[] examples:
 *
 * Example 1:
 *
 * Node 0 & 1 are CPU + DRAM nodes, node 2 & 3 are PMEM nodes.
 *
 * node distances:
 * node   0    1    2    3
 *    0  10   20   30   40
 *    1  20   10   40   30
 *    2  30   40   10   40
 *    3  40   30   40   10
 *
 * memory_tiers0 = 0-1
 * memory_tiers1 = 2-3
 *
 * node_demotion[0].preferred = 2
 * node_demotion[1].preferred = 3
 * node_demotion[2].preferred = <empty>
 * node_demotion[3].preferred = <empty>
 *
 * Example 2:
 *
 * Node 0 & 1 are CPU + DRAM nodes, node 2 is memory-only DRAM node.
 *
 * node distances:
 * node   0    1    2
 *    0  10   20   30
 *    1  20   10   30
 *    2  30   30   10
 *
 * memory_tiers0 = 0-2
 *
 * node_demotion[0].preferred = <empty>
 * node_demotion[1].preferred = <empty>
 * node_demotion[2].preferred = <empty>
 *
 * Example 3:
 *
 * Node 0 is CPU + DRAM nodes, Node 1 is HBM node, node 2 is PMEM node.
 *
 * node distances:
 * node   0    1    2
 *    0  10   20   30
 *    1  20   10   40
 *    2  30   40   10
 *
 * memory_tiers0 = 1
 * memory_tiers1 = 0
 * memory_tiers2 = 2
 *
 * node_demotion[0].preferred = 2
 * node_demotion[1].preferred = 0
 * node_demotion[2].preferred = <empty>
 *
 */
static struct demotion_nodes *node_demotion __read_mostly;
#endif /* CONFIG_MIGRATION */

static BLOCKING_NOTIFIER_HEAD(mt_adistance_algorithms);

/* The lock is used to protect `default_dram_perf*` info and nid. */
static DEFINE_MUTEX(default_dram_perf_lock);
static bool default_dram_perf_error;
static struct access_coordinate default_dram_perf;
static int default_dram_perf_ref_nid = NUMA_NO_NODE;
static const char *default_dram_perf_ref_source;

static inline struct memory_tier *to_memory_tier(struct device *device)
{
	return container_of(device, struct memory_tier, dev);
}

static __always_inline nodemask_t get_memtier_nodemask(struct memory_tier *memtier)
{
	nodemask_t nodes = NODE_MASK_NONE;
	struct memory_dev_type *memtype;

	list_for_each_entry(memtype, &memtier->memory_types, tier_sibling)
		nodes_or(nodes, nodes, memtype->nodes);

	return nodes;
}

static void memory_tier_device_release(struct device *dev)
{
	struct memory_tier *tier = to_memory_tier(dev);
	/*
	 * synchronize_rcu in clear_node_memory_tier makes sure
	 * we don't have rcu access to this memory tier.
	 */
	kfree(tier);
}

static ssize_t nodelist_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	int ret;
	nodemask_t nmask;

	mutex_lock(&memory_tier_lock);
	nmask = get_memtier_nodemask(to_memory_tier(dev));
	ret = sysfs_emit(buf, "%*pbl\n", nodemask_pr_args(&nmask));
	mutex_unlock(&memory_tier_lock);
	return ret;
}
static DEVICE_ATTR_RO(nodelist);

static struct attribute *memtier_dev_attrs[] = {
	&dev_attr_nodelist.attr,
	NULL
};

static const struct attribute_group memtier_dev_group = {
	.attrs = memtier_dev_attrs,
};

static const struct attribute_group *memtier_dev_groups[] = {
	&memtier_dev_group,
	NULL
};

static struct memory_tier *find_create_memory_tier(struct memory_dev_type *memtype)
{
	int ret;
	bool found_slot = false;
	struct memory_tier *memtier, *new_memtier;
	int adistance = memtype->adistance;
	unsigned int memtier_adistance_chunk_size = MEMTIER_CHUNK_SIZE;

	lockdep_assert_held_once(&memory_tier_lock);

	adistance = round_down(adistance, memtier_adistance_chunk_size);
	/*
	 * If the memtype is already part of a memory tier,
	 * just return that.
	 */
	if (!list_empty(&memtype->tier_sibling)) {
		list_for_each_entry(memtier, &memory_tiers, list) {
			if (adistance == memtier->adistance_start)
				return memtier;
		}
		WARN_ON(1);
		return ERR_PTR(-EINVAL);
	}

	list_for_each_entry(memtier, &memory_tiers, list) {
		if (adistance == memtier->adistance_start) {
			goto link_memtype;
		} else if (adistance < memtier->adistance_start) {
			found_slot = true;
			break;
		}
	}

	new_memtier = kzalloc(sizeof(struct memory_tier), GFP_KERNEL);
	if (!new_memtier)
		return ERR_PTR(-ENOMEM);

	new_memtier->adistance_start = adistance;
	INIT_LIST_HEAD(&new_memtier->list);
	INIT_LIST_HEAD(&new_memtier->memory_types);
	if (found_slot)
		list_add_tail(&new_memtier->list, &memtier->list);
	else
		list_add_tail(&new_memtier->list, &memory_tiers);

	new_memtier->dev.id = adistance >> MEMTIER_CHUNK_BITS;
	new_memtier->dev.bus = &memory_tier_subsys;
	new_memtier->dev.release = memory_tier_device_release;
	new_memtier->dev.groups = memtier_dev_groups;

	ret = device_register(&new_memtier->dev);
	if (ret) {
		list_del(&new_memtier->list);
		put_device(&new_memtier->dev);
		return ERR_PTR(ret);
	}
	memtier = new_memtier;

link_memtype:
	list_add(&memtype->tier_sibling, &memtier->memory_types);
	return memtier;
}

static struct memory_tier *__node_get_memory_tier(int node)
{
	pg_data_t *pgdat;

	pgdat = NODE_DATA(node);
	if (!pgdat)
		return NULL;
	/*
	 * Since we hold memory_tier_lock, we can avoid
	 * RCU read locks when accessing the details. No
	 * parallel updates are possible here.
	 */
	return rcu_dereference_check(pgdat->memtier,
				     lockdep_is_held(&memory_tier_lock));
}

#ifdef CONFIG_MIGRATION
bool node_is_toptier(int node)
{
	bool toptier;
	pg_data_t *pgdat;
	struct memory_tier *memtier;

	pgdat = NODE_DATA(node);
	if (!pgdat)
		return false;

	rcu_read_lock();
	memtier = rcu_dereference(pgdat->memtier);
	if (!memtier) {
		toptier = true;
		goto out;
	}
	if (memtier->adistance_start <= top_tier_adistance)
		toptier = true;
	else
		toptier = false;
out:
	rcu_read_unlock();
	return toptier;
}

void node_get_allowed_targets(pg_data_t *pgdat, nodemask_t *targets)
{
	struct memory_tier *memtier;

	/*
	 * pg_data_t.memtier updates includes a synchronize_rcu()
	 * which ensures that we either find NULL or a valid memtier
	 * in NODE_DATA. protect the access via rcu_read_lock();
	 */
	rcu_read_lock();
	memtier = rcu_dereference(pgdat->memtier);
	if (memtier)
		*targets = memtier->lower_tier_mask;
	else
		*targets = NODE_MASK_NONE;
	rcu_read_unlock();
}

/**
 * next_demotion_node() - Get the next node in the demotion path
 * @node: The starting node to lookup the next node
 *
 * Return: node id for next memory node in the demotion path hierarchy
 * from @node; NUMA_NO_NODE if @node is terminal.  This does not keep
 * @node online or guarantee that it *continues* to be the next demotion
 * target.
 */
int next_demotion_node(int node)
{
	struct demotion_nodes *nd;
	int target;

	if (!node_demotion)
		return NUMA_NO_NODE;

	nd = &node_demotion[node];

	/*
	 * node_demotion[] is updated without excluding this
	 * function from running.
	 *
	 * Make sure to use RCU over entire code blocks if
	 * node_demotion[] reads need to be consistent.
	 */
	rcu_read_lock();
	/*
	 * If there are multiple target nodes, just select one
	 * target node randomly.
	 *
	 * In addition, we can also use round-robin to select
	 * target node, but we should introduce another variable
	 * for node_demotion[] to record last selected target node,
	 * that may cause cache ping-pong due to the changing of
	 * last target node. Or introducing per-cpu data to avoid
	 * caching issue, which seems more complicated. So selecting
	 * target node randomly seems better until now.
	 */
	target = node_random(&nd->preferred);
	rcu_read_unlock();

	return target;
}

static void disable_all_demotion_targets(void)
{
	struct memory_tier *memtier;
	int node;

	for_each_node_state(node, N_MEMORY) {
		node_demotion[node].preferred = NODE_MASK_NONE;
		/*
		 * We are holding memory_tier_lock, it is safe
		 * to access pgda->memtier.
		 */
		memtier = __node_get_memory_tier(node);
		if (memtier)
			memtier->lower_tier_mask = NODE_MASK_NONE;
	}
	/*
	 * Ensure that the "disable" is visible across the system.
	 * Readers will see either a combination of before+disable
	 * state or disable+after.  They will never see before and
	 * after state together.
	 */
	synchronize_rcu();
}

static void dump_demotion_targets(void)
{
	int node;

	for_each_node_state(node, N_MEMORY) {
		struct memory_tier *memtier = __node_get_memory_tier(node);
		nodemask_t preferred = node_demotion[node].preferred;

		if (!memtier)
			continue;

		if (nodes_empty(preferred))
			pr_info("Demotion targets for Node %d: null\n", node);
		else
			pr_info("Demotion targets for Node %d: preferred: %*pbl, fallback: %*pbl\n",
				node, nodemask_pr_args(&preferred),
				nodemask_pr_args(&memtier->lower_tier_mask));
	}
}

/*
 * Find an automatic demotion target for all memory
 * nodes. Failing here is OK.  It might just indicate
 * being at the end of a chain.
 */
static void establish_demotion_targets(void)
{
	struct memory_tier *memtier;
	struct demotion_nodes *nd;
	int target = NUMA_NO_NODE, node;
	int distance, best_distance;
	nodemask_t tier_nodes, lower_tier;

	lockdep_assert_held_once(&memory_tier_lock);

	if (!node_demotion)
		return;

	disable_all_demotion_targets();

	for_each_node_state(node, N_MEMORY) {
		best_distance = -1;
		nd = &node_demotion[node];

		memtier = __node_get_memory_tier(node);
		if (!memtier || list_is_last(&memtier->list, &memory_tiers))
			continue;
		/*
		 * Get the lower memtier to find the  demotion node list.
		 */
		memtier = list_next_entry(memtier, list);
		tier_nodes = get_memtier_nodemask(memtier);
		/*
		 * find_next_best_node, use 'used' nodemask as a skip list.
		 * Add all memory nodes except the selected memory tier
		 * nodelist to skip list so that we find the best node from the
		 * memtier nodelist.
		 */
		nodes_andnot(tier_nodes, node_states[N_MEMORY], tier_nodes);

		/*
		 * Find all the nodes in the memory tier node list of same best distance.
		 * add them to the preferred mask. We randomly select between nodes
		 * in the preferred mask when allocating pages during demotion.
		 */
		do {
			target = find_next_best_node(node, &tier_nodes);
			if (target == NUMA_NO_NODE)
				break;

			distance = node_distance(node, target);
			if (distance == best_distance || best_distance == -1) {
				best_distance = distance;
				node_set(target, nd->preferred);
			} else {
				break;
			}
		} while (1);
	}
	/*
	 * Promotion is allowed from a memory tier to higher
	 * memory tier only if the memory tier doesn't include
	 * compute. We want to skip promotion from a memory tier,
	 * if any node that is part of the memory tier have CPUs.
	 * Once we detect such a memory tier, we consider that tier
	 * as top tiper from which promotion is not allowed.
	 */
	list_for_each_entry_reverse(memtier, &memory_tiers, list) {
		tier_nodes = get_memtier_nodemask(memtier);
		nodes_and(tier_nodes, node_states[N_CPU], tier_nodes);
		if (!nodes_empty(tier_nodes)) {
			/*
			 * abstract distance below the max value of this memtier
			 * is considered toptier.
			 */
			top_tier_adistance = memtier->adistance_start +
						MEMTIER_CHUNK_SIZE - 1;
			break;
		}
	}
	/*
	 * Now build the lower_tier mask for each node collecting node mask from
	 * all memory tier below it. This allows us to fallback demotion page
	 * allocation to a set of nodes that is closer the above selected
	 * preferred node.
	 */
	lower_tier = node_states[N_MEMORY];
	list_for_each_entry(memtier, &memory_tiers, list) {
		/*
		 * Keep removing current tier from lower_tier nodes,
		 * This will remove all nodes in current and above
		 * memory tier from the lower_tier mask.
		 */
		tier_nodes = get_memtier_nodemask(memtier);
		nodes_andnot(lower_tier, lower_tier, tier_nodes);
		memtier->lower_tier_mask = lower_tier;
	}

	dump_demotion_targets();
}

#else
static inline void establish_demotion_targets(void) {}
#endif /* CONFIG_MIGRATION */

static inline void __init_node_memory_type(int node, struct memory_dev_type *memtype)
{
	if (!node_memory_types[node].memtype)
		node_memory_types[node].memtype = memtype;
	/*
	 * for each device getting added in the same NUMA node
	 * with this specific memtype, bump the map count. We
	 * Only take memtype device reference once, so that
	 * changing a node memtype can be done by droping the
	 * only reference count taken here.
	 */

	if (node_memory_types[node].memtype == memtype) {
		if (!node_memory_types[node].map_count++)
			kref_get(&memtype->kref);
	}
}

static struct memory_tier *set_node_memory_tier(int node)
{
	struct memory_tier *memtier;
	struct memory_dev_type *memtype = default_dram_type;
	int adist = MEMTIER_ADISTANCE_DRAM;
	pg_data_t *pgdat = NODE_DATA(node);


	lockdep_assert_held_once(&memory_tier_lock);

	if (!node_state(node, N_MEMORY))
		return ERR_PTR(-EINVAL);

	mt_calc_adistance(node, &adist);
	if (!node_memory_types[node].memtype) {
		memtype = mt_find_alloc_memory_type(adist, &default_memory_types);
		if (IS_ERR(memtype)) {
			memtype = default_dram_type;
			pr_info("Failed to allocate a memory type. Fall back.\n");
		}
	}

	__init_node_memory_type(node, memtype);

	memtype = node_memory_types[node].memtype;
	node_set(node, memtype->nodes);
	memtier = find_create_memory_tier(memtype);
	if (!IS_ERR(memtier))
		rcu_assign_pointer(pgdat->memtier, memtier);
	return memtier;
}

static void destroy_memory_tier(struct memory_tier *memtier)
{
	list_del(&memtier->list);
	device_unregister(&memtier->dev);
}

static bool clear_node_memory_tier(int node)
{
	bool cleared = false;
	pg_data_t *pgdat;
	struct memory_tier *memtier;

	pgdat = NODE_DATA(node);
	if (!pgdat)
		return false;

	/*
	 * Make sure that anybody looking at NODE_DATA who finds
	 * a valid memtier finds memory_dev_types with nodes still
	 * linked to the memtier. We achieve this by waiting for
	 * rcu read section to finish using synchronize_rcu.
	 * This also enables us to free the destroyed memory tier
	 * with kfree instead of kfree_rcu
	 */
	memtier = __node_get_memory_tier(node);
	if (memtier) {
		struct memory_dev_type *memtype;

		rcu_assign_pointer(pgdat->memtier, NULL);
		synchronize_rcu();
		memtype = node_memory_types[node].memtype;
		node_clear(node, memtype->nodes);
		if (nodes_empty(memtype->nodes)) {
			list_del_init(&memtype->tier_sibling);
			if (list_empty(&memtier->memory_types))
				destroy_memory_tier(memtier);
		}
		cleared = true;
	}
	return cleared;
}

static void release_memtype(struct kref *kref)
{
	struct memory_dev_type *memtype;

	memtype = container_of(kref, struct memory_dev_type, kref);
	kfree(memtype);
}

struct memory_dev_type *alloc_memory_type(int adistance)
{
	struct memory_dev_type *memtype;

	memtype = kmalloc(sizeof(*memtype), GFP_KERNEL);
	if (!memtype)
		return ERR_PTR(-ENOMEM);

	memtype->adistance = adistance;
	INIT_LIST_HEAD(&memtype->tier_sibling);
	memtype->nodes  = NODE_MASK_NONE;
	kref_init(&memtype->kref);
	return memtype;
}
EXPORT_SYMBOL_GPL(alloc_memory_type);

void put_memory_type(struct memory_dev_type *memtype)
{
	kref_put(&memtype->kref, release_memtype);
}
EXPORT_SYMBOL_GPL(put_memory_type);

void init_node_memory_type(int node, struct memory_dev_type *memtype)
{

	mutex_lock(&memory_tier_lock);
	__init_node_memory_type(node, memtype);
	mutex_unlock(&memory_tier_lock);
}
EXPORT_SYMBOL_GPL(init_node_memory_type);

void clear_node_memory_type(int node, struct memory_dev_type *memtype)
{
	mutex_lock(&memory_tier_lock);
	if (node_memory_types[node].memtype == memtype || !memtype)
		node_memory_types[node].map_count--;
	/*
	 * If we umapped all the attached devices to this node,
	 * clear the node memory type.
	 */
	if (!node_memory_types[node].map_count) {
		memtype = node_memory_types[node].memtype;
		node_memory_types[node].memtype = NULL;
		put_memory_type(memtype);
	}
	mutex_unlock(&memory_tier_lock);
}
EXPORT_SYMBOL_GPL(clear_node_memory_type);

struct memory_dev_type *mt_find_alloc_memory_type(int adist, struct list_head *memory_types)
{
	struct memory_dev_type *mtype;

	list_for_each_entry(mtype, memory_types, list)
		if (mtype->adistance == adist)
			return mtype;

	mtype = alloc_memory_type(adist);
	if (IS_ERR(mtype))
		return mtype;

	list_add(&mtype->list, memory_types);

	return mtype;
}
EXPORT_SYMBOL_GPL(mt_find_alloc_memory_type);

void mt_put_memory_types(struct list_head *memory_types)
{
	struct memory_dev_type *mtype, *mtn;

	list_for_each_entry_safe(mtype, mtn, memory_types, list) {
		list_del(&mtype->list);
		put_memory_type(mtype);
	}
}
EXPORT_SYMBOL_GPL(mt_put_memory_types);

/*
 * This is invoked via `late_initcall()` to initialize memory tiers for
 * CPU-less memory nodes after driver initialization, which is
 * expected to provide `adistance` algorithms.
 */
static int __init memory_tier_late_init(void)
{
	int nid;

	guard(mutex)(&memory_tier_lock);
	for_each_node_state(nid, N_MEMORY) {
		/*
		 * Some device drivers may have initialized memory tiers
		 * between `memory_tier_init()` and `memory_tier_late_init()`,
		 * potentially bringing online memory nodes and
		 * configuring memory tiers. Exclude them here.
		 */
		if (node_memory_types[nid].memtype)
			continue;

		set_node_memory_tier(nid);
	}

	establish_demotion_targets();

	return 0;
}
late_initcall(memory_tier_late_init);

static void dump_hmem_attrs(struct access_coordinate *coord, const char *prefix)
{
	pr_info(
"%sread_latency: %u, write_latency: %u, read_bandwidth: %u, write_bandwidth: %u\n",
		prefix, coord->read_latency, coord->write_latency,
		coord->read_bandwidth, coord->write_bandwidth);
}

int mt_set_default_dram_perf(int nid, struct access_coordinate *perf,
			     const char *source)
{
	guard(mutex)(&default_dram_perf_lock);
	if (default_dram_perf_error)
		return -EIO;

	if (perf->read_latency + perf->write_latency == 0 ||
	    perf->read_bandwidth + perf->write_bandwidth == 0)
		return -EINVAL;

	if (default_dram_perf_ref_nid == NUMA_NO_NODE) {
		default_dram_perf = *perf;
		default_dram_perf_ref_nid = nid;
		default_dram_perf_ref_source = kstrdup(source, GFP_KERNEL);
		return 0;
	}

	/*
	 * The performance of all default DRAM nodes is expected to be
	 * same (that is, the variation is less than 10%).  And it
	 * will be used as base to calculate the abstract distance of
	 * other memory nodes.
	 */
	if (abs(perf->read_latency - default_dram_perf.read_latency) * 10 >
	    default_dram_perf.read_latency ||
	    abs(perf->write_latency - default_dram_perf.write_latency) * 10 >
	    default_dram_perf.write_latency ||
	    abs(perf->read_bandwidth - default_dram_perf.read_bandwidth) * 10 >
	    default_dram_perf.read_bandwidth ||
	    abs(perf->write_bandwidth - default_dram_perf.write_bandwidth) * 10 >
	    default_dram_perf.write_bandwidth) {
		pr_info(
"memory-tiers: the performance of DRAM node %d mismatches that of the reference\n"
"DRAM node %d.\n", nid, default_dram_perf_ref_nid);
		pr_info("  performance of reference DRAM node %d:\n",
			default_dram_perf_ref_nid);
		dump_hmem_attrs(&default_dram_perf, "    ");
		pr_info("  performance of DRAM node %d:\n", nid);
		dump_hmem_attrs(perf, "    ");
		pr_info(
"  disable default DRAM node performance based abstract distance algorithm.\n");
		default_dram_perf_error = true;
		return -EINVAL;
	}

	return 0;
}

int mt_perf_to_adistance(struct access_coordinate *perf, int *adist)
{
	guard(mutex)(&default_dram_perf_lock);
	if (default_dram_perf_error)
		return -EIO;

	if (perf->read_latency + perf->write_latency == 0 ||
	    perf->read_bandwidth + perf->write_bandwidth == 0)
		return -EINVAL;

	if (default_dram_perf_ref_nid == NUMA_NO_NODE)
		return -ENOENT;

	/*
	 * The abstract distance of a memory node is in direct proportion to
	 * its memory latency (read + write) and inversely proportional to its
	 * memory bandwidth (read + write).  The abstract distance, memory
	 * latency, and memory bandwidth of the default DRAM nodes are used as
	 * the base.
	 */
	*adist = MEMTIER_ADISTANCE_DRAM *
		(perf->read_latency + perf->write_latency) /
		(default_dram_perf.read_latency + default_dram_perf.write_latency) *
		(default_dram_perf.read_bandwidth + default_dram_perf.write_bandwidth) /
		(perf->read_bandwidth + perf->write_bandwidth);

	return 0;
}
EXPORT_SYMBOL_GPL(mt_perf_to_adistance);

/**
 * register_mt_adistance_algorithm() - Register memory tiering abstract distance algorithm
 * @nb: The notifier block which describe the algorithm
 *
 * Return: 0 on success, errno on error.
 *
 * Every memory tiering abstract distance algorithm provider needs to
 * register the algorithm with register_mt_adistance_algorithm().  To
 * calculate the abstract distance for a specified memory node, the
 * notifier function will be called unless some high priority
 * algorithm has provided result.  The prototype of the notifier
 * function is as follows,
 *
 *   int (*algorithm_notifier)(struct notifier_block *nb,
 *                             unsigned long nid, void *data);
 *
 * Where "nid" specifies the memory node, "data" is the pointer to the
 * returned abstract distance (that is, "int *adist").  If the
 * algorithm provides the result, NOTIFY_STOP should be returned.
 * Otherwise, return_value & %NOTIFY_STOP_MASK == 0 to allow the next
 * algorithm in the chain to provide the result.
 */
int register_mt_adistance_algorithm(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&mt_adistance_algorithms, nb);
}
EXPORT_SYMBOL_GPL(register_mt_adistance_algorithm);

/**
 * unregister_mt_adistance_algorithm() - Unregister memory tiering abstract distance algorithm
 * @nb: the notifier block which describe the algorithm
 *
 * Return: 0 on success, errno on error.
 */
int unregister_mt_adistance_algorithm(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&mt_adistance_algorithms, nb);
}
EXPORT_SYMBOL_GPL(unregister_mt_adistance_algorithm);

/**
 * mt_calc_adistance() - Calculate abstract distance with registered algorithms
 * @node: the node to calculate abstract distance for
 * @adist: the returned abstract distance
 *
 * Return: if return_value & %NOTIFY_STOP_MASK != 0, then some
 * abstract distance algorithm provides the result, and return it via
 * @adist.  Otherwise, no algorithm can provide the result and @adist
 * will be kept as it is.
 */
int mt_calc_adistance(int node, int *adist)
{
	return blocking_notifier_call_chain(&mt_adistance_algorithms, node, adist);
}
EXPORT_SYMBOL_GPL(mt_calc_adistance);

static int __meminit memtier_hotplug_callback(struct notifier_block *self,
					      unsigned long action, void *_arg)
{
	struct memory_tier *memtier;
	struct memory_notify *arg = _arg;

	/*
	 * Only update the node migration order when a node is
	 * changing status, like online->offline.
	 */
	if (arg->status_change_nid < 0)
		return notifier_from_errno(0);

	switch (action) {
	case MEM_OFFLINE:
		mutex_lock(&memory_tier_lock);
		if (clear_node_memory_tier(arg->status_change_nid))
			establish_demotion_targets();
		mutex_unlock(&memory_tier_lock);
		break;
	case MEM_ONLINE:
		mutex_lock(&memory_tier_lock);
		memtier = set_node_memory_tier(arg->status_change_nid);
		if (!IS_ERR(memtier))
			establish_demotion_targets();
		mutex_unlock(&memory_tier_lock);
		break;
	}

	return notifier_from_errno(0);
}

static int __init memory_tier_init(void)
{
	int ret, node;
	struct memory_tier *memtier;

	ret = subsys_virtual_register(&memory_tier_subsys, NULL);
	if (ret)
		panic("%s() failed to register memory tier subsystem\n", __func__);

#ifdef CONFIG_MIGRATION
	node_demotion = kcalloc(nr_node_ids, sizeof(struct demotion_nodes),
				GFP_KERNEL);
	WARN_ON(!node_demotion);
#endif
	mutex_lock(&memory_tier_lock);
	/*
	 * For now we can have 4 faster memory tiers with smaller adistance
	 * than default DRAM tier.
	 */
	default_dram_type = mt_find_alloc_memory_type(MEMTIER_ADISTANCE_DRAM,
						      &default_memory_types);
	if (IS_ERR(default_dram_type))
		panic("%s() failed to allocate default DRAM tier\n", __func__);

	/*
	 * Look at all the existing N_MEMORY nodes and add them to
	 * default memory tier or to a tier if we already have memory
	 * types assigned.
	 */
	for_each_node_state(node, N_MEMORY) {
		if (!node_state(node, N_CPU))
			/*
			 * Defer memory tier initialization on
			 * CPUless numa nodes. These will be initialized
			 * after firmware and devices are initialized.
			 */
			continue;

		memtier = set_node_memory_tier(node);
		if (IS_ERR(memtier))
			/*
			 * Continue with memtiers we are able to setup
			 */
			break;
	}
	establish_demotion_targets();
	mutex_unlock(&memory_tier_lock);

	hotplug_memory_notifier(memtier_hotplug_callback, MEMTIER_HOTPLUG_PRI);
	return 0;
}
subsys_initcall(memory_tier_init);

bool numa_demotion_enabled = false;

#ifdef CONFIG_MIGRATION
#ifdef CONFIG_SYSFS
static ssize_t demotion_enabled_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s\n",
			  numa_demotion_enabled ? "true" : "false");
}

static ssize_t demotion_enabled_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	ssize_t ret;

	ret = kstrtobool(buf, &numa_demotion_enabled);
	if (ret)
		return ret;

	return count;
}

static struct kobj_attribute numa_demotion_enabled_attr =
	__ATTR_RW(demotion_enabled);

static struct attribute *numa_attrs[] = {
	&numa_demotion_enabled_attr.attr,
	NULL,
};

static const struct attribute_group numa_attr_group = {
	.attrs = numa_attrs,
};

static int __init numa_init_sysfs(void)
{
	int err;
	struct kobject *numa_kobj;

	numa_kobj = kobject_create_and_add("numa", mm_kobj);
	if (!numa_kobj) {
		pr_err("failed to create numa kobject\n");
		return -ENOMEM;
	}
	err = sysfs_create_group(numa_kobj, &numa_attr_group);
	if (err) {
		pr_err("failed to register numa group\n");
		goto delete_obj;
	}
	return 0;

delete_obj:
	kobject_put(numa_kobj);
	return err;
}
subsys_initcall(numa_init_sysfs);
#endif /* CONFIG_SYSFS */
#endif
