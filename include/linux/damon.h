/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DAMON api
 *
 * Author: SeongJae Park <sj@kernel.org>
 */

#ifndef _DAMON_H_
#define _DAMON_H_

#include <linux/memcontrol.h>
#include <linux/mutex.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/random.h>

/* Minimal region size.  Every damon_region is aligned by this. */
#define DAMON_MIN_REGION	PAGE_SIZE
/* Max priority score for DAMON-based operation schemes */
#define DAMOS_MAX_SCORE		(99)

/* Get a random number in [l, r) */
static inline unsigned long damon_rand(unsigned long l, unsigned long r)
{
	return l + get_random_u32_below(r - l);
}

/**
 * struct damon_addr_range - Represents an address region of [@start, @end).
 * @start:	Start address of the region (inclusive).
 * @end:	End address of the region (exclusive).
 */
struct damon_addr_range {
	unsigned long start;
	unsigned long end;
};

/**
 * struct damon_size_range - Represents size for filter to operate on [@min, @max].
 * @min:	Min size (inclusive).
 * @max:	Max size (inclusive).
 */
struct damon_size_range {
	unsigned long min;
	unsigned long max;
};

/**
 * struct damon_region - Represents a monitoring target region.
 * @ar:			The address range of the region.
 * @sampling_addr:	Address of the sample for the next access check.
 * @nr_accesses:	Access frequency of this region.
 * @nr_accesses_bp:	@nr_accesses in basis point (0.01%) that updated for
 *			each sampling interval.
 * @list:		List head for siblings.
 * @age:		Age of this region.
 *
 * @nr_accesses is reset to zero for every &damon_attrs->aggr_interval and be
 * increased for every &damon_attrs->sample_interval if an access to the region
 * during the last sampling interval is found.  The update of this field should
 * not be done with direct access but with the helper function,
 * damon_update_region_access_rate().
 *
 * @nr_accesses_bp is another representation of @nr_accesses in basis point
 * (1 in 10,000) that updated for every &damon_attrs->sample_interval in a
 * manner similar to moving sum.  By the algorithm, this value becomes
 * @nr_accesses * 10000 for every &struct damon_attrs->aggr_interval.  This can
 * be used when the aggregation interval is too huge and therefore cannot wait
 * for it before getting the access monitoring results.
 *
 * @age is initially zero, increased for each aggregation interval, and reset
 * to zero again if the access frequency is significantly changed.  If two
 * regions are merged into a new region, both @nr_accesses and @age of the new
 * region are set as region size-weighted average of those of the two regions.
 */
struct damon_region {
	struct damon_addr_range ar;
	unsigned long sampling_addr;
	unsigned int nr_accesses;
	unsigned int nr_accesses_bp;
	struct list_head list;

	unsigned int age;
/* private: Internal value for age calculation. */
	unsigned int last_nr_accesses;
};

/**
 * struct damon_target - Represents a monitoring target.
 * @pid:		The PID of the virtual address space to monitor.
 * @nr_regions:		Number of monitoring target regions of this target.
 * @regions_list:	Head of the monitoring target regions of this target.
 * @list:		List head for siblings.
 *
 * Each monitoring context could have multiple targets.  For example, a context
 * for virtual memory address spaces could have multiple target processes.  The
 * @pid should be set for appropriate &struct damon_operations including the
 * virtual address spaces monitoring operations.
 */
struct damon_target {
	struct pid *pid;
	unsigned int nr_regions;
	struct list_head regions_list;
	struct list_head list;
};

/**
 * enum damos_action - Represents an action of a Data Access Monitoring-based
 * Operation Scheme.
 *
 * @DAMOS_WILLNEED:	Call ``madvise()`` for the region with MADV_WILLNEED.
 * @DAMOS_COLD:		Call ``madvise()`` for the region with MADV_COLD.
 * @DAMOS_PAGEOUT:	Reclaim the region.
 * @DAMOS_HUGEPAGE:	Call ``madvise()`` for the region with MADV_HUGEPAGE.
 * @DAMOS_NOHUGEPAGE:	Call ``madvise()`` for the region with MADV_NOHUGEPAGE.
 * @DAMOS_LRU_PRIO:	Prioritize the region on its LRU lists.
 * @DAMOS_LRU_DEPRIO:	Deprioritize the region on its LRU lists.
 * @DAMOS_MIGRATE_HOT:  Migrate the regions prioritizing warmer regions.
 * @DAMOS_MIGRATE_COLD:	Migrate the regions prioritizing colder regions.
 * @DAMOS_STAT:		Do nothing but count the stat.
 * @NR_DAMOS_ACTIONS:	Total number of DAMOS actions
 *
 * The support of each action is up to running &struct damon_operations.
 * Refer to 'Operation Action' section of Documentation/mm/damon/design.rst for
 * status of the supports.
 *
 * Note that DAMOS_PAGEOUT doesn't trigger demotions.
 */
enum damos_action {
	DAMOS_WILLNEED,
	DAMOS_COLD,
	DAMOS_PAGEOUT,
	DAMOS_HUGEPAGE,
	DAMOS_NOHUGEPAGE,
	DAMOS_LRU_PRIO,
	DAMOS_LRU_DEPRIO,
	DAMOS_MIGRATE_HOT,
	DAMOS_MIGRATE_COLD,
	DAMOS_STAT,		/* Do nothing but only record the stat */
	NR_DAMOS_ACTIONS,
};

/**
 * enum damos_quota_goal_metric - Represents the metric to be used as the goal
 *
 * @DAMOS_QUOTA_USER_INPUT:	User-input value.
 * @DAMOS_QUOTA_SOME_MEM_PSI_US:	System level some memory PSI in us.
 * @DAMOS_QUOTA_NODE_MEM_USED_BP:	MemUsed ratio of a node.
 * @DAMOS_QUOTA_NODE_MEM_FREE_BP:	MemFree ratio of a node.
 * @NR_DAMOS_QUOTA_GOAL_METRICS:	Number of DAMOS quota goal metrics.
 *
 * Metrics equal to larger than @NR_DAMOS_QUOTA_GOAL_METRICS are unsupported.
 */
enum damos_quota_goal_metric {
	DAMOS_QUOTA_USER_INPUT,
	DAMOS_QUOTA_SOME_MEM_PSI_US,
	DAMOS_QUOTA_NODE_MEM_USED_BP,
	DAMOS_QUOTA_NODE_MEM_FREE_BP,
	NR_DAMOS_QUOTA_GOAL_METRICS,
};

/**
 * struct damos_quota_goal - DAMOS scheme quota auto-tuning goal.
 * @metric:		Metric to be used for representing the goal.
 * @target_value:	Target value of @metric to achieve with the tuning.
 * @current_value:	Current value of @metric.
 * @last_psi_total:	Last measured total PSI
 * @nid:		Node id.
 * @list:		List head for siblings.
 *
 * Data structure for getting the current score of the quota tuning goal.  The
 * score is calculated by how close @current_value and @target_value are.  Then
 * the score is entered to DAMON's internal feedback loop mechanism to get the
 * auto-tuned quota.
 *
 * If @metric is DAMOS_QUOTA_USER_INPUT, @current_value should be manually
 * entered by the user, probably inside the kdamond callbacks.  Otherwise,
 * DAMON sets @current_value with self-measured value of @metric.
 *
 * If @metric is DAMOS_QUOTA_NODE_MEM_{USED,FREE}_BP, @nid represents the node
 * id of the target node to account the used/free memory.
 */
struct damos_quota_goal {
	enum damos_quota_goal_metric metric;
	unsigned long target_value;
	unsigned long current_value;
	/* metric-dependent fields */
	union {
		u64 last_psi_total;
		int nid;
	};
	struct list_head list;
};

/**
 * struct damos_quota - Controls the aggressiveness of the given scheme.
 * @reset_interval:	Charge reset interval in milliseconds.
 * @ms:			Maximum milliseconds that the scheme can use.
 * @sz:			Maximum bytes of memory that the action can be applied.
 * @goals:		Head of quota tuning goals (&damos_quota_goal) list.
 * @esz:		Effective size quota in bytes.
 *
 * @weight_sz:		Weight of the region's size for prioritization.
 * @weight_nr_accesses:	Weight of the region's nr_accesses for prioritization.
 * @weight_age:		Weight of the region's age for prioritization.
 *
 * To avoid consuming too much CPU time or IO resources for applying the
 * &struct damos->action to large memory, DAMON allows users to set time and/or
 * size quotas.  The quotas can be set by writing non-zero values to &ms and
 * &sz, respectively.  If the time quota is set, DAMON tries to use only up to
 * &ms milliseconds within &reset_interval for applying the action.  If the
 * size quota is set, DAMON tries to apply the action only up to &sz bytes
 * within &reset_interval.
 *
 * To convince the different types of quotas and goals, DAMON internally
 * converts those into one single size quota called "effective quota".  DAMON
 * internally uses it as the only one real quota.  The conversion is made as
 * follows.
 *
 * The time quota is transformed to a size quota using estimated throughput of
 * the scheme's action.  DAMON then compares it against &sz and uses smaller
 * one as the effective quota.
 *
 * If @goals is not empty, DAMON calculates yet another size quota based on the
 * goals using its internal feedback loop algorithm, for every @reset_interval.
 * Then, if the new size quota is smaller than the effective quota, it uses the
 * new size quota as the effective quota.
 *
 * The resulting effective size quota in bytes is set to @esz.
 *
 * For selecting regions within the quota, DAMON prioritizes current scheme's
 * target memory regions using the &struct damon_operations->get_scheme_score.
 * You could customize the prioritization logic by setting &weight_sz,
 * &weight_nr_accesses, and &weight_age, because monitoring operations are
 * encouraged to respect those.
 */
struct damos_quota {
	unsigned long reset_interval;
	unsigned long ms;
	unsigned long sz;
	struct list_head goals;
	unsigned long esz;

	unsigned int weight_sz;
	unsigned int weight_nr_accesses;
	unsigned int weight_age;

/* private: */
	/* For throughput estimation */
	unsigned long total_charged_sz;
	unsigned long total_charged_ns;

	/* For charging the quota */
	unsigned long charged_sz;
	unsigned long charged_from;
	struct damon_target *charge_target_from;
	unsigned long charge_addr_from;

	/* For prioritization */
	unsigned int min_score;

	/* For feedback loop */
	unsigned long esz_bp;
};

/**
 * enum damos_wmark_metric - Represents the watermark metric.
 *
 * @DAMOS_WMARK_NONE:		Ignore the watermarks of the given scheme.
 * @DAMOS_WMARK_FREE_MEM_RATE:	Free memory rate of the system in [0,1000].
 * @NR_DAMOS_WMARK_METRICS:	Total number of DAMOS watermark metrics
 */
enum damos_wmark_metric {
	DAMOS_WMARK_NONE,
	DAMOS_WMARK_FREE_MEM_RATE,
	NR_DAMOS_WMARK_METRICS,
};

/**
 * struct damos_watermarks - Controls when a given scheme should be activated.
 * @metric:	Metric for the watermarks.
 * @interval:	Watermarks check time interval in microseconds.
 * @high:	High watermark.
 * @mid:	Middle watermark.
 * @low:	Low watermark.
 *
 * If &metric is &DAMOS_WMARK_NONE, the scheme is always active.  Being active
 * means DAMON does monitoring and applying the action of the scheme to
 * appropriate memory regions.  Else, DAMON checks &metric of the system for at
 * least every &interval microseconds and works as below.
 *
 * If &metric is higher than &high, the scheme is inactivated.  If &metric is
 * between &mid and &low, the scheme is activated.  If &metric is lower than
 * &low, the scheme is inactivated.
 */
struct damos_watermarks {
	enum damos_wmark_metric metric;
	unsigned long interval;
	unsigned long high;
	unsigned long mid;
	unsigned long low;

/* private: */
	bool activated;
};

/**
 * struct damos_stat - Statistics on a given scheme.
 * @nr_tried:	Total number of regions that the scheme is tried to be applied.
 * @sz_tried:	Total size of regions that the scheme is tried to be applied.
 * @nr_applied:	Total number of regions that the scheme is applied.
 * @sz_applied:	Total size of regions that the scheme is applied.
 * @sz_ops_filter_passed:
 *		Total bytes that passed ops layer-handled DAMOS filters.
 * @qt_exceeds: Total number of times the quota of the scheme has exceeded.
 *
 * "Tried an action to a region" in this context means the DAMOS core logic
 * determined the region as eligible to apply the action.  The access pattern
 * (&struct damos_access_pattern), quotas (&struct damos_quota), watermarks
 * (&struct damos_watermarks) and filters (&struct damos_filter) that handled
 * on core logic can affect this.  The core logic asks the operation set
 * (&struct damon_operations) to apply the action to the region.
 *
 * "Applied an action to a region" in this context means the operation set
 * (&struct damon_operations) successfully applied the action to the region, at
 * least to a part of the region.  The filters (&struct damos_filter) that
 * handled on operation set layer and type of the action and pages of the
 * region can affect this.  For example, if a filter is set to exclude
 * anonymous pages and the region has only anonymous pages, the region will be
 * failed at applying the action.  If the action is &DAMOS_PAGEOUT and all
 * pages of the region are already paged out, the region will be failed at
 * applying the action.
 */
struct damos_stat {
	unsigned long nr_tried;
	unsigned long sz_tried;
	unsigned long nr_applied;
	unsigned long sz_applied;
	unsigned long sz_ops_filter_passed;
	unsigned long qt_exceeds;
};

/**
 * enum damos_filter_type - Type of memory for &struct damos_filter
 * @DAMOS_FILTER_TYPE_ANON:	Anonymous pages.
 * @DAMOS_FILTER_TYPE_ACTIVE:	Active pages.
 * @DAMOS_FILTER_TYPE_MEMCG:	Specific memcg's pages.
 * @DAMOS_FILTER_TYPE_YOUNG:	Recently accessed pages.
 * @DAMOS_FILTER_TYPE_HUGEPAGE_SIZE:	Page is part of a hugepage.
 * @DAMOS_FILTER_TYPE_UNMAPPED:	Unmapped pages.
 * @DAMOS_FILTER_TYPE_ADDR:	Address range.
 * @DAMOS_FILTER_TYPE_TARGET:	Data Access Monitoring target.
 * @NR_DAMOS_FILTER_TYPES:	Number of filter types.
 *
 * The anon pages type and memcg type filters are handled by underlying
 * &struct damon_operations as a part of scheme action trying, and therefore
 * accounted as 'tried'.  In contrast, other types are handled by core layer
 * before trying of the action and therefore not accounted as 'tried'.
 *
 * The support of the filters that handled by &struct damon_operations depend
 * on the running &struct damon_operations.
 * &enum DAMON_OPS_PADDR supports both anon pages type and memcg type filters,
 * while &enum DAMON_OPS_VADDR and &enum DAMON_OPS_FVADDR don't support any of
 * the two types.
 */
enum damos_filter_type {
	DAMOS_FILTER_TYPE_ANON,
	DAMOS_FILTER_TYPE_ACTIVE,
	DAMOS_FILTER_TYPE_MEMCG,
	DAMOS_FILTER_TYPE_YOUNG,
	DAMOS_FILTER_TYPE_HUGEPAGE_SIZE,
	DAMOS_FILTER_TYPE_UNMAPPED,
	DAMOS_FILTER_TYPE_ADDR,
	DAMOS_FILTER_TYPE_TARGET,
	NR_DAMOS_FILTER_TYPES,
};

/**
 * struct damos_filter - DAMOS action target memory filter.
 * @type:	Type of the target memory.
 * @matching:	Whether this is for @type-matching memory.
 * @allow:	Whether to include or exclude the @matching memory.
 * @memcg_id:	Memcg id of the question if @type is DAMOS_FILTER_MEMCG.
 * @addr_range:	Address range if @type is DAMOS_FILTER_TYPE_ADDR.
 * @target_idx:	Index of the &struct damon_target of
 *		&damon_ctx->adaptive_targets if @type is
 *		DAMOS_FILTER_TYPE_TARGET.
 * @sz_range:	Size range if @type is DAMOS_FILTER_TYPE_HUGEPAGE_SIZE.
 * @list:	List head for siblings.
 *
 * Before applying the &damos->action to a memory region, DAMOS checks if each
 * byte of the region matches to this given condition and avoid applying the
 * action if so.  Support of each filter type depends on the running &struct
 * damon_operations and the type.  Refer to &enum damos_filter_type for more
 * details.
 */
struct damos_filter {
	enum damos_filter_type type;
	bool matching;
	bool allow;
	union {
		unsigned short memcg_id;
		struct damon_addr_range addr_range;
		int target_idx;
		struct damon_size_range sz_range;
	};
	struct list_head list;
};

struct damon_ctx;
struct damos;

/**
 * struct damos_walk_control - Control damos_walk().
 *
 * @walk_fn:	Function to be called back for each region.
 * @data:	Data that will be passed to walk functions.
 *
 * Control damos_walk(), which requests specific kdamond to invoke the given
 * function to each region that eligible to apply actions of the kdamond's
 * schemes.  Refer to damos_walk() for more details.
 */
struct damos_walk_control {
	void (*walk_fn)(void *data, struct damon_ctx *ctx,
			struct damon_target *t, struct damon_region *r,
			struct damos *s, unsigned long sz_filter_passed);
	void *data;
/* private: internal use only */
	/* informs if the kdamond finished handling of the walk request */
	struct completion completion;
	/* informs if the walk is canceled. */
	bool canceled;
};

/**
 * struct damos_access_pattern - Target access pattern of the given scheme.
 * @min_sz_region:	Minimum size of target regions.
 * @max_sz_region:	Maximum size of target regions.
 * @min_nr_accesses:	Minimum ``->nr_accesses`` of target regions.
 * @max_nr_accesses:	Maximum ``->nr_accesses`` of target regions.
 * @min_age_region:	Minimum age of target regions.
 * @max_age_region:	Maximum age of target regions.
 */
struct damos_access_pattern {
	unsigned long min_sz_region;
	unsigned long max_sz_region;
	unsigned int min_nr_accesses;
	unsigned int max_nr_accesses;
	unsigned int min_age_region;
	unsigned int max_age_region;
};

/**
 * struct damos_migrate_dests - Migration destination nodes and their weights.
 * @node_id_arr:	Array of migration destination node ids.
 * @weight_arr:		Array of migration weights for @node_id_arr.
 * @nr_dests:		Length of the @node_id_arr and @weight_arr arrays.
 *
 * @node_id_arr is an array of the ids of migration destination nodes.
 * @weight_arr is an array of the weights for those.  The weights in
 * @weight_arr are for nodes in @node_id_arr of same array index.
 */
struct damos_migrate_dests {
	unsigned int *node_id_arr;
	unsigned int *weight_arr;
	size_t nr_dests;
};

/**
 * struct damos - Represents a Data Access Monitoring-based Operation Scheme.
 * @pattern:		Access pattern of target regions.
 * @action:		&damos_action to be applied to the target regions.
 * @apply_interval_us:	The time between applying the @action.
 * @quota:		Control the aggressiveness of this scheme.
 * @wmarks:		Watermarks for automated (in)activation of this scheme.
 * @migrate_dests:	Destination nodes if @action is "migrate_{hot,cold}".
 * @target_nid:		Destination node if @action is "migrate_{hot,cold}".
 * @filters:		Additional set of &struct damos_filter for &action.
 * @ops_filters:	ops layer handling &struct damos_filter objects list.
 * @last_applied:	Last @action applied ops-managing entity.
 * @stat:		Statistics of this scheme.
 * @list:		List head for siblings.
 *
 * For each @apply_interval_us, DAMON finds regions which fit in the
 * &pattern and applies &action to those. To avoid consuming too much
 * CPU time or IO resources for the &action, &quota is used.
 *
 * If @apply_interval_us is zero, &damon_attrs->aggr_interval is used instead.
 *
 * To do the work only when needed, schemes can be activated for specific
 * system situations using &wmarks.  If all schemes that registered to the
 * monitoring context are inactive, DAMON stops monitoring either, and just
 * repeatedly checks the watermarks.
 *
 * @migrate_dests specifies multiple migration target nodes with different
 * weights for migrate_hot or migrate_cold actions.  @target_nid is ignored if
 * this is set.
 *
 * @target_nid is used to set the migration target node for migrate_hot or
 * migrate_cold actions, and @migrate_dests is unset.
 *
 * Before applying the &action to a memory region, &struct damon_operations
 * implementation could check pages of the region and skip &action to respect
 * &filters
 *
 * The minimum entity that @action can be applied depends on the underlying
 * &struct damon_operations.  Since it may not be aligned with the core layer
 * abstract, namely &struct damon_region, &struct damon_operations could apply
 * @action to same entity multiple times.  Large folios that underlying on
 * multiple &struct damon region objects could be such examples.  The &struct
 * damon_operations can use @last_applied to avoid that.  DAMOS core logic
 * unsets @last_applied when each regions walking for applying the scheme is
 * finished.
 *
 * After applying the &action to each region, &stat_count and &stat_sz is
 * updated to reflect the number of regions and total size of regions that the
 * &action is applied.
 */
struct damos {
	struct damos_access_pattern pattern;
	enum damos_action action;
	unsigned long apply_interval_us;
/* private: internal use only */
	/*
	 * number of sample intervals that should be passed before applying
	 * @action
	 */
	unsigned long next_apply_sis;
	/* informs if ongoing DAMOS walk for this scheme is finished */
	bool walk_completed;
	/*
	 * If the current region in the filtering stage is allowed by core
	 * layer-handled filters.  If true, operations layer allows it, too.
	 */
	bool core_filters_allowed;
	/* whether to reject core/ops filters umatched regions */
	bool core_filters_default_reject;
	bool ops_filters_default_reject;
/* public: */
	struct damos_quota quota;
	struct damos_watermarks wmarks;
	union {
		struct {
			int target_nid;
			struct damos_migrate_dests migrate_dests;
		};
	};
	struct list_head filters;
	struct list_head ops_filters;
	void *last_applied;
	struct damos_stat stat;
	struct list_head list;
};

/**
 * enum damon_ops_id - Identifier for each monitoring operations implementation
 *
 * @DAMON_OPS_VADDR:	Monitoring operations for virtual address spaces
 * @DAMON_OPS_FVADDR:	Monitoring operations for only fixed ranges of virtual
 *			address spaces
 * @DAMON_OPS_PADDR:	Monitoring operations for the physical address space
 * @NR_DAMON_OPS:	Number of monitoring operations implementations
 */
enum damon_ops_id {
	DAMON_OPS_VADDR,
	DAMON_OPS_FVADDR,
	DAMON_OPS_PADDR,
	NR_DAMON_OPS,
};

/**
 * struct damon_operations - Monitoring operations for given use cases.
 *
 * @id:				Identifier of this operations set.
 * @init:			Initialize operations-related data structures.
 * @update:			Update operations-related data structures.
 * @prepare_access_checks:	Prepare next access check of target regions.
 * @check_accesses:		Check the accesses to target regions.
 * @get_scheme_score:		Get the score of a region for a scheme.
 * @apply_scheme:		Apply a DAMON-based operation scheme.
 * @target_valid:		Determine if the target is valid.
 * @cleanup_target:		Clean up each target before deallocation.
 * @cleanup:			Clean up the context.
 *
 * DAMON can be extended for various address spaces and usages.  For this,
 * users should register the low level operations for their target address
 * space and usecase via the &damon_ctx.ops.  Then, the monitoring thread
 * (&damon_ctx.kdamond) calls @init and @prepare_access_checks before starting
 * the monitoring, @update after each &damon_attrs.ops_update_interval, and
 * @check_accesses, @target_valid and @prepare_access_checks after each
 * &damon_attrs.sample_interval.
 *
 * Each &struct damon_operations instance having valid @id can be registered
 * via damon_register_ops() and selected by damon_select_ops() later.
 * @init should initialize operations-related data structures.  For example,
 * this could be used to construct proper monitoring target regions and link
 * those to @damon_ctx.adaptive_targets.
 * @update should update the operations-related data structures.  For example,
 * this could be used to update monitoring target regions for current status.
 * @prepare_access_checks should manipulate the monitoring regions to be
 * prepared for the next access check.
 * @check_accesses should check the accesses to each region that made after the
 * last preparation and update the number of observed accesses of each region.
 * It should also return max number of observed accesses that made as a result
 * of its update.  The value will be used for regions adjustment threshold.
 * @get_scheme_score should return the priority score of a region for a scheme
 * as an integer in [0, &DAMOS_MAX_SCORE].
 * @apply_scheme is called from @kdamond when a region for user provided
 * DAMON-based operation scheme is found.  It should apply the scheme's action
 * to the region and return bytes of the region that the action is successfully
 * applied.  It should also report how many bytes of the region has passed
 * filters (&struct damos_filter) that handled by itself.
 * @target_valid should check whether the target is still valid for the
 * monitoring.
 * @cleanup_target is called before the target will be deallocated.
 * @cleanup is called from @kdamond just before its termination.
 */
struct damon_operations {
	enum damon_ops_id id;
	void (*init)(struct damon_ctx *context);
	void (*update)(struct damon_ctx *context);
	void (*prepare_access_checks)(struct damon_ctx *context);
	unsigned int (*check_accesses)(struct damon_ctx *context);
	int (*get_scheme_score)(struct damon_ctx *context,
			struct damon_target *t, struct damon_region *r,
			struct damos *scheme);
	unsigned long (*apply_scheme)(struct damon_ctx *context,
			struct damon_target *t, struct damon_region *r,
			struct damos *scheme, unsigned long *sz_filter_passed);
	bool (*target_valid)(struct damon_target *t);
	void (*cleanup_target)(struct damon_target *t);
	void (*cleanup)(struct damon_ctx *context);
};

/*
 * struct damon_call_control - Control damon_call().
 *
 * @fn:			Function to be called back.
 * @data:		Data that will be passed to @fn.
 * @repeat:		Repeat invocations.
 * @return_code:	Return code from @fn invocation.
 * @dealloc_on_cancel:	De-allocate when canceled.
 *
 * Control damon_call(), which requests specific kdamond to invoke a given
 * function.  Refer to damon_call() for more details.
 */
struct damon_call_control {
	int (*fn)(void *data);
	void *data;
	bool repeat;
	int return_code;
	bool dealloc_on_cancel;
/* private: internal use only */
	/* informs if the kdamond finished handling of the request */
	struct completion completion;
	/* informs if the kdamond canceled @fn infocation */
	bool canceled;
	/* List head for siblings. */
	struct list_head list;
};

/**
 * struct damon_intervals_goal - Monitoring intervals auto-tuning goal.
 *
 * @access_bp:		Access events observation ratio to achieve in bp.
 * @aggrs:		Number of aggregations to achieve @access_bp within.
 * @min_sample_us:	Minimum resulting sampling interval in microseconds.
 * @max_sample_us:	Maximum resulting sampling interval in microseconds.
 *
 * DAMON automatically tunes &damon_attrs->sample_interval and
 * &damon_attrs->aggr_interval aiming the ratio in bp (1/10,000) of
 * DAMON-observed access events to theoretical maximum amount within @aggrs
 * aggregations be same to @access_bp.  The logic increases
 * &damon_attrs->aggr_interval and &damon_attrs->sampling_interval in same
 * ratio if the current access events observation ratio is lower than the
 * target for each @aggrs aggregations, and vice versa.
 *
 * If @aggrs is zero, the tuning is disabled and hence this struct is ignored.
 */
struct damon_intervals_goal {
	unsigned long access_bp;
	unsigned long aggrs;
	unsigned long min_sample_us;
	unsigned long max_sample_us;
};

/**
 * struct damon_attrs - Monitoring attributes for accuracy/overhead control.
 *
 * @sample_interval:		The time between access samplings.
 * @aggr_interval:		The time between monitor results aggregations.
 * @ops_update_interval:	The time between monitoring operations updates.
 * @intervals_goal:		Intervals auto-tuning goal.
 * @min_nr_regions:		The minimum number of adaptive monitoring
 *				regions.
 * @max_nr_regions:		The maximum number of adaptive monitoring
 *				regions.
 *
 * For each @sample_interval, DAMON checks whether each region is accessed or
 * not during the last @sample_interval.  If such access is found, DAMON
 * aggregates the information by increasing &damon_region->nr_accesses for
 * @aggr_interval time.  For each @aggr_interval, the count is reset.  DAMON
 * also checks whether the target memory regions need update (e.g., by
 * ``mmap()`` calls from the application, in case of virtual memory monitoring)
 * and applies the changes for each @ops_update_interval.  All time intervals
 * are in micro-seconds.  Please refer to &struct damon_operations and &struct
 * damon_call_control for more detail.
 */
struct damon_attrs {
	unsigned long sample_interval;
	unsigned long aggr_interval;
	unsigned long ops_update_interval;
	struct damon_intervals_goal intervals_goal;
	unsigned long min_nr_regions;
	unsigned long max_nr_regions;
/* private: internal use only */
	/*
	 * @aggr_interval to @sample_interval ratio.
	 * Core-external components call damon_set_attrs() with &damon_attrs
	 * that this field is unset.  In the case, damon_set_attrs() sets this
	 * field of resulting &damon_attrs.  Core-internal components such as
	 * kdamond_tune_intervals() calls damon_set_attrs() with &damon_attrs
	 * that this field is set.  In the case, damon_set_attrs() just keep
	 * it.
	 */
	unsigned long aggr_samples;
};

/**
 * struct damon_ctx - Represents a context for each monitoring.  This is the
 * main interface that allows users to set the attributes and get the results
 * of the monitoring.
 *
 * @attrs:		Monitoring attributes for accuracy/overhead control.
 * @kdamond:		Kernel thread who does the monitoring.
 * @kdamond_lock:	Mutex for the synchronizations with @kdamond.
 *
 * For each monitoring context, one kernel thread for the monitoring is
 * created.  The pointer to the thread is stored in @kdamond.
 *
 * Once started, the monitoring thread runs until explicitly required to be
 * terminated or every monitoring target is invalid.  The validity of the
 * targets is checked via the &damon_operations.target_valid of @ops.  The
 * termination can also be explicitly requested by calling damon_stop().
 * The thread sets @kdamond to NULL when it terminates. Therefore, users can
 * know whether the monitoring is ongoing or terminated by reading @kdamond.
 * Reads and writes to @kdamond from outside of the monitoring thread must
 * be protected by @kdamond_lock.
 *
 * Note that the monitoring thread protects only @kdamond via @kdamond_lock.
 * Accesses to other fields must be protected by themselves.
 *
 * @ops:	Set of monitoring operations for given use cases.
 * @addr_unit:	Scale factor for core to ops address conversion.
 * @min_sz_region:		Minimum region size.
 * @adaptive_targets:	Head of monitoring targets (&damon_target) list.
 * @schemes:		Head of schemes (&damos) list.
 */
struct damon_ctx {
	struct damon_attrs attrs;

/* private: internal use only */
	/* number of sample intervals that passed since this context started */
	unsigned long passed_sample_intervals;
	/*
	 * number of sample intervals that should be passed before next
	 * aggregation
	 */
	unsigned long next_aggregation_sis;
	/*
	 * number of sample intervals that should be passed before next ops
	 * update
	 */
	unsigned long next_ops_update_sis;
	/*
	 * number of sample intervals that should be passed before next
	 * intervals tuning
	 */
	unsigned long next_intervals_tune_sis;
	/* for waiting until the execution of the kdamond_fn is started */
	struct completion kdamond_started;
	/* for scheme quotas prioritization */
	unsigned long *regions_score_histogram;

	/* lists of &struct damon_call_control */
	struct list_head call_controls;
	struct mutex call_controls_lock;

	struct damos_walk_control *walk_control;
	struct mutex walk_control_lock;

/* public: */
	struct task_struct *kdamond;
	struct mutex kdamond_lock;

	struct damon_operations ops;
	unsigned long addr_unit;
	unsigned long min_sz_region;

	struct list_head adaptive_targets;
	struct list_head schemes;
};

static inline struct damon_region *damon_next_region(struct damon_region *r)
{
	return container_of(r->list.next, struct damon_region, list);
}

static inline struct damon_region *damon_prev_region(struct damon_region *r)
{
	return container_of(r->list.prev, struct damon_region, list);
}

static inline struct damon_region *damon_last_region(struct damon_target *t)
{
	return list_last_entry(&t->regions_list, struct damon_region, list);
}

static inline struct damon_region *damon_first_region(struct damon_target *t)
{
	return list_first_entry(&t->regions_list, struct damon_region, list);
}

static inline unsigned long damon_sz_region(struct damon_region *r)
{
	return r->ar.end - r->ar.start;
}


#define damon_for_each_region(r, t) \
	list_for_each_entry(r, &t->regions_list, list)

#define damon_for_each_region_from(r, t) \
	list_for_each_entry_from(r, &t->regions_list, list)

#define damon_for_each_region_safe(r, next, t) \
	list_for_each_entry_safe(r, next, &t->regions_list, list)

#define damon_for_each_target(t, ctx) \
	list_for_each_entry(t, &(ctx)->adaptive_targets, list)

#define damon_for_each_target_safe(t, next, ctx)	\
	list_for_each_entry_safe(t, next, &(ctx)->adaptive_targets, list)

#define damon_for_each_scheme(s, ctx) \
	list_for_each_entry(s, &(ctx)->schemes, list)

#define damon_for_each_scheme_safe(s, next, ctx) \
	list_for_each_entry_safe(s, next, &(ctx)->schemes, list)

#define damos_for_each_quota_goal(goal, quota) \
	list_for_each_entry(goal, &quota->goals, list)

#define damos_for_each_quota_goal_safe(goal, next, quota) \
	list_for_each_entry_safe(goal, next, &(quota)->goals, list)

#define damos_for_each_filter(f, scheme) \
	list_for_each_entry(f, &(scheme)->filters, list)

#define damos_for_each_filter_safe(f, next, scheme) \
	list_for_each_entry_safe(f, next, &(scheme)->filters, list)

#define damos_for_each_ops_filter(f, scheme) \
	list_for_each_entry(f, &(scheme)->ops_filters, list)

#define damos_for_each_ops_filter_safe(f, next, scheme) \
	list_for_each_entry_safe(f, next, &(scheme)->ops_filters, list)

#ifdef CONFIG_DAMON

struct damon_region *damon_new_region(unsigned long start, unsigned long end);

/*
 * Add a region between two other regions
 */
static inline void damon_insert_region(struct damon_region *r,
		struct damon_region *prev, struct damon_region *next,
		struct damon_target *t)
{
	__list_add(&r->list, &prev->list, &next->list);
	t->nr_regions++;
}

void damon_add_region(struct damon_region *r, struct damon_target *t);
void damon_destroy_region(struct damon_region *r, struct damon_target *t);
int damon_set_regions(struct damon_target *t, struct damon_addr_range *ranges,
		unsigned int nr_ranges, unsigned long min_sz_region);
void damon_update_region_access_rate(struct damon_region *r, bool accessed,
		struct damon_attrs *attrs);

struct damos_filter *damos_new_filter(enum damos_filter_type type,
		bool matching, bool allow);
void damos_add_filter(struct damos *s, struct damos_filter *f);
bool damos_filter_for_ops(enum damos_filter_type type);
void damos_destroy_filter(struct damos_filter *f);

struct damos_quota_goal *damos_new_quota_goal(
		enum damos_quota_goal_metric metric,
		unsigned long target_value);
void damos_add_quota_goal(struct damos_quota *q, struct damos_quota_goal *g);
void damos_destroy_quota_goal(struct damos_quota_goal *goal);

struct damos *damon_new_scheme(struct damos_access_pattern *pattern,
			enum damos_action action,
			unsigned long apply_interval_us,
			struct damos_quota *quota,
			struct damos_watermarks *wmarks,
			int target_nid);
void damon_add_scheme(struct damon_ctx *ctx, struct damos *s);
void damon_destroy_scheme(struct damos *s);
int damos_commit_quota_goals(struct damos_quota *dst, struct damos_quota *src);

struct damon_target *damon_new_target(void);
void damon_add_target(struct damon_ctx *ctx, struct damon_target *t);
bool damon_targets_empty(struct damon_ctx *ctx);
void damon_free_target(struct damon_target *t);
void damon_destroy_target(struct damon_target *t, struct damon_ctx *ctx);
unsigned int damon_nr_regions(struct damon_target *t);

struct damon_ctx *damon_new_ctx(void);
void damon_destroy_ctx(struct damon_ctx *ctx);
int damon_set_attrs(struct damon_ctx *ctx, struct damon_attrs *attrs);
void damon_set_schemes(struct damon_ctx *ctx,
			struct damos **schemes, ssize_t nr_schemes);
int damon_commit_ctx(struct damon_ctx *old_ctx, struct damon_ctx *new_ctx);
int damon_nr_running_ctxs(void);
bool damon_is_registered_ops(enum damon_ops_id id);
int damon_register_ops(struct damon_operations *ops);
int damon_select_ops(struct damon_ctx *ctx, enum damon_ops_id id);

static inline bool damon_target_has_pid(const struct damon_ctx *ctx)
{
	return ctx->ops.id == DAMON_OPS_VADDR || ctx->ops.id == DAMON_OPS_FVADDR;
}

static inline unsigned int damon_max_nr_accesses(const struct damon_attrs *attrs)
{
	/* {aggr,sample}_interval are unsigned long, hence could overflow */
	return min(attrs->aggr_interval / attrs->sample_interval,
			(unsigned long)UINT_MAX);
}


bool damon_initialized(void);
int damon_start(struct damon_ctx **ctxs, int nr_ctxs, bool exclusive);
int damon_stop(struct damon_ctx **ctxs, int nr_ctxs);
bool damon_is_running(struct damon_ctx *ctx);

int damon_call(struct damon_ctx *ctx, struct damon_call_control *control);
int damos_walk(struct damon_ctx *ctx, struct damos_walk_control *control);

int damon_set_region_biggest_system_ram_default(struct damon_target *t,
				unsigned long *start, unsigned long *end);

#endif	/* CONFIG_DAMON */

#endif	/* _DAMON_H */
