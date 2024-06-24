/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _PERF_ANNOTATE_DATA_H
#define _PERF_ANNOTATE_DATA_H

#include <errno.h>
#include <linux/compiler.h>
#include <linux/rbtree.h>
#include <linux/types.h>

struct annotated_op_loc;
struct debuginfo;
struct evsel;
struct hist_browser_timer;
struct hist_entry;
struct map_symbol;
struct thread;

/**
 * struct annotated_member - Type of member field
 * @node: List entry in the parent list
 * @children: List head for child nodes
 * @type_name: Name of the member type
 * @var_name: Name of the member variable
 * @offset: Offset from the outer data type
 * @size: Size of the member field
 *
 * This represents a member type in a data type.
 */
struct annotated_member {
	struct list_head node;
	struct list_head children;
	char *type_name;
	char *var_name;
	int offset;
	int size;
};

/**
 * struct type_hist_entry - Histogram entry per offset
 * @nr_samples: Number of samples
 * @period: Count of event
 */
struct type_hist_entry {
	int nr_samples;
	u64 period;
};

/**
 * struct type_hist - Type histogram for each event
 * @nr_samples: Total number of samples in this data type
 * @period: Total count of the event in this data type
 * @offset: Array of histogram entry
 */
struct type_hist {
	u64			nr_samples;
	u64			period;
	struct type_hist_entry	addr[];
};

/**
 * struct annotated_data_type - Data type to profile
 * @node: RB-tree node for dso->type_tree
 * @self: Actual type information
 * @nr_histogram: Number of histogram entries
 * @histograms: An array of pointers to histograms
 *
 * This represents a data type accessed by samples in the profile data.
 */
struct annotated_data_type {
	struct rb_node node;
	struct annotated_member self;
	int nr_histograms;
	struct type_hist **histograms;
};

extern struct annotated_data_type unknown_type;
extern struct annotated_data_type stackop_type;
extern struct annotated_data_type canary_type;

/**
 * struct data_loc_info - Data location information
 * @arch: CPU architecture info
 * @thread: Thread info
 * @ms: Map and Symbol info
 * @ip: Instruction address
 * @var_addr: Data address (for global variables)
 * @cpumode: CPU execution mode
 * @op: Instruction operand location (regs and offset)
 * @di: Debug info
 * @fbreg: Frame base register
 * @fb_cfa: Whether the frame needs to check CFA
 * @type_offset: Final offset in the type
 */
struct data_loc_info {
	/* These are input field, should be filled by caller */
	struct arch *arch;
	struct thread *thread;
	struct map_symbol *ms;
	u64 ip;
	u64 var_addr;
	u8 cpumode;
	struct annotated_op_loc *op;

	/* These are used internally */
	struct debuginfo *di;
	int fbreg;
	bool fb_cfa;

	/* This is for the result */
	int type_offset;
};

/**
 * struct annotated_data_stat - Debug statistics
 * @total: Total number of entry
 * @no_sym: No symbol or map found
 * @no_insn: Failed to get disasm line
 * @no_insn_ops: The instruction has no operands
 * @no_mem_ops: The instruction has no memory operands
 * @no_reg: Failed to extract a register from the operand
 * @no_dbginfo: The binary has no debug information
 * @no_cuinfo: Failed to find a compile_unit
 * @no_var: Failed to find a matching variable
 * @no_typeinfo: Failed to get a type info for the variable
 * @invalid_size: Failed to get a size info of the type
 * @bad_offset: The access offset is out of the type
 */
struct annotated_data_stat {
	int total;
	int no_sym;
	int no_insn;
	int no_insn_ops;
	int no_mem_ops;
	int no_reg;
	int no_dbginfo;
	int no_cuinfo;
	int no_var;
	int no_typeinfo;
	int invalid_size;
	int bad_offset;
	int insn_track;
};
extern struct annotated_data_stat ann_data_stat;

#ifdef HAVE_DWARF_SUPPORT

/* Returns data type at the location (ip, reg, offset) */
struct annotated_data_type *find_data_type(struct data_loc_info *dloc);

/* Update type access histogram at the given offset */
int annotated_data_type__update_samples(struct annotated_data_type *adt,
					struct evsel *evsel, int offset,
					int nr_samples, u64 period);

/* Release all data type information in the tree */
void annotated_data_type__tree_delete(struct rb_root *root);

/* Release all global variable information in the tree */
void global_var_type__tree_delete(struct rb_root *root);

int hist_entry__annotate_data_tty(struct hist_entry *he, struct evsel *evsel);

#else /* HAVE_DWARF_SUPPORT */

static inline struct annotated_data_type *
find_data_type(struct data_loc_info *dloc __maybe_unused)
{
	return NULL;
}

static inline int
annotated_data_type__update_samples(struct annotated_data_type *adt __maybe_unused,
				    struct evsel *evsel __maybe_unused,
				    int offset __maybe_unused,
				    int nr_samples __maybe_unused,
				    u64 period __maybe_unused)
{
	return -1;
}

static inline void annotated_data_type__tree_delete(struct rb_root *root __maybe_unused)
{
}

static inline void global_var_type__tree_delete(struct rb_root *root __maybe_unused)
{
}

static inline int hist_entry__annotate_data_tty(struct hist_entry *he __maybe_unused,
						struct evsel *evsel __maybe_unused)
{
	return -1;
}

#endif /* HAVE_DWARF_SUPPORT */

#ifdef HAVE_SLANG_SUPPORT
int hist_entry__annotate_data_tui(struct hist_entry *he, struct evsel *evsel,
				  struct hist_browser_timer *hbt);
#else
static inline int hist_entry__annotate_data_tui(struct hist_entry *he __maybe_unused,
						struct evsel *evsel __maybe_unused,
						struct hist_browser_timer *hbt __maybe_unused)
{
	return -1;
}
#endif /* HAVE_SLANG_SUPPORT */

#endif /* _PERF_ANNOTATE_DATA_H */
