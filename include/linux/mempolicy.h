/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NUMA memory policies for Linux.
 * Copyright 2003,2004 Andi Kleen SuSE Labs
 */
#ifndef _LINUX_MEMPOLICY_H
#define _LINUX_MEMPOLICY_H 1

#include <linux/sched.h>
#include <linux/mmzone.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/spinlock.h>
#include <linux/nodemask.h>
#include <linux/pagemap.h>
#include <uapi/linux/mempolicy.h>

struct mm_struct;

#define NO_INTERLEAVE_INDEX (-1UL)	/* use task il_prev for interleaving */

#ifdef CONFIG_NUMA

/*
 * Describe a memory policy.
 *
 * A mempolicy can be either associated with a process or with a VMA.
 * For VMA related allocations the VMA policy is preferred, otherwise
 * the process policy is used. Interrupts ignore the memory policy
 * of the current process.
 *
 * Locking policy for interleave:
 * In process context there is no locking because only the process accesses
 * its own state. All vma manipulation is somewhat protected by a down_read on
 * mmap_lock.
 *
 * Freeing policy:
 * Mempolicy objects are reference counted.  A mempolicy will be freed when
 * mpol_put() decrements the reference count to zero.
 *
 * Duplicating policy objects:
 * mpol_dup() allocates a new mempolicy and copies the specified mempolicy
 * to the new storage.  The reference count of the new object is initialized
 * to 1, representing the caller of mpol_dup().
 */
struct mempolicy {
	atomic_t refcnt;
	unsigned short mode; 	/* See MPOL_* above */
	unsigned short flags;	/* See set_mempolicy() MPOL_F_* above */
	nodemask_t nodes;	/* interleave/bind/perfer */
	int home_node;		/* Home node to use for MPOL_BIND and MPOL_PREFERRED_MANY */

	union {
		nodemask_t cpuset_mems_allowed;	/* relative to these nodes */
		nodemask_t user_nodemask;	/* nodemask passed by user */
	} w;
};

/*
 * Support for managing mempolicy data objects (clone, copy, destroy)
 * The default fast path of a NULL MPOL_DEFAULT policy is always inlined.
 */

extern void __mpol_put(struct mempolicy *pol);
static inline void mpol_put(struct mempolicy *pol)
{
	if (pol)
		__mpol_put(pol);
}

/*
 * Does mempolicy pol need explicit unref after use?
 * Currently only needed for shared policies.
 */
static inline int mpol_needs_cond_ref(struct mempolicy *pol)
{
	return (pol && (pol->flags & MPOL_F_SHARED));
}

static inline void mpol_cond_put(struct mempolicy *pol)
{
	if (mpol_needs_cond_ref(pol))
		__mpol_put(pol);
}

extern struct mempolicy *__mpol_dup(struct mempolicy *pol);
static inline struct mempolicy *mpol_dup(struct mempolicy *pol)
{
	if (pol)
		pol = __mpol_dup(pol);
	return pol;
}

static inline void mpol_get(struct mempolicy *pol)
{
	if (pol)
		atomic_inc(&pol->refcnt);
}

extern bool __mpol_equal(struct mempolicy *a, struct mempolicy *b);
static inline bool mpol_equal(struct mempolicy *a, struct mempolicy *b)
{
	if (a == b)
		return true;
	return __mpol_equal(a, b);
}

/*
 * Tree of shared policies for a shared memory region.
 */
struct shared_policy {
	struct rb_root root;
	rwlock_t lock;
};
struct sp_node {
	struct rb_node nd;
	pgoff_t start, end;
	struct mempolicy *policy;
};

int vma_dup_policy(struct vm_area_struct *src, struct vm_area_struct *dst);
void mpol_shared_policy_init(struct shared_policy *sp, struct mempolicy *mpol);
int mpol_set_shared_policy(struct shared_policy *sp,
			   struct vm_area_struct *vma, struct mempolicy *mpol);
void mpol_free_shared_policy(struct shared_policy *sp);
struct mempolicy *mpol_shared_policy_lookup(struct shared_policy *sp,
					    pgoff_t idx);

struct mempolicy *get_task_policy(struct task_struct *p);
struct mempolicy *__get_vma_policy(struct vm_area_struct *vma,
		unsigned long addr, pgoff_t *ilx);
struct mempolicy *get_vma_policy(struct vm_area_struct *vma,
		unsigned long addr, int order, pgoff_t *ilx);
bool vma_policy_mof(struct vm_area_struct *vma);

extern void numa_default_policy(void);
extern void numa_policy_init(void);
extern void mpol_rebind_task(struct task_struct *tsk, const nodemask_t *new);
extern void mpol_rebind_mm(struct mm_struct *mm, nodemask_t *new);

extern int huge_node(struct vm_area_struct *vma,
				unsigned long addr, gfp_t gfp_flags,
				struct mempolicy **mpol, nodemask_t **nodemask);
extern bool init_nodemask_of_mempolicy(nodemask_t *mask);
extern bool mempolicy_in_oom_domain(struct task_struct *tsk,
				const nodemask_t *mask);
extern unsigned int mempolicy_slab_node(void);

extern enum zone_type policy_zone;

static inline void check_highest_zone(enum zone_type k)
{
	if (k > policy_zone && k != ZONE_MOVABLE)
		policy_zone = k;
}

int do_migrate_pages(struct mm_struct *mm, const nodemask_t *from,
		     const nodemask_t *to, int flags);


#ifdef CONFIG_TMPFS
extern int mpol_parse_str(char *str, struct mempolicy **mpol);
#endif

extern void mpol_to_str(char *buffer, int maxlen, struct mempolicy *pol);

/* Check if a vma is migratable */
extern bool vma_migratable(struct vm_area_struct *vma);

int mpol_misplaced(struct folio *folio, struct vm_fault *vmf,
					unsigned long addr);
extern void mpol_put_task_policy(struct task_struct *);

static inline bool mpol_is_preferred_many(struct mempolicy *pol)
{
	return  (pol->mode == MPOL_PREFERRED_MANY);
}

extern bool apply_policy_zone(struct mempolicy *policy, enum zone_type zone);

#else

struct mempolicy {};

static inline struct mempolicy *get_task_policy(struct task_struct *p)
{
	return NULL;
}

static inline bool mpol_equal(struct mempolicy *a, struct mempolicy *b)
{
	return true;
}

static inline void mpol_put(struct mempolicy *pol)
{
}

static inline void mpol_cond_put(struct mempolicy *pol)
{
}

static inline void mpol_get(struct mempolicy *pol)
{
}

struct shared_policy {};

static inline void mpol_shared_policy_init(struct shared_policy *sp,
						struct mempolicy *mpol)
{
}

static inline void mpol_free_shared_policy(struct shared_policy *sp)
{
}

static inline struct mempolicy *
mpol_shared_policy_lookup(struct shared_policy *sp, pgoff_t idx)
{
	return NULL;
}

static inline struct mempolicy *get_vma_policy(struct vm_area_struct *vma,
				unsigned long addr, int order, pgoff_t *ilx)
{
	*ilx = 0;
	return NULL;
}

static inline int
vma_dup_policy(struct vm_area_struct *src, struct vm_area_struct *dst)
{
	return 0;
}

static inline void numa_policy_init(void)
{
}

static inline void numa_default_policy(void)
{
}

static inline void mpol_rebind_task(struct task_struct *tsk,
				const nodemask_t *new)
{
}

static inline void mpol_rebind_mm(struct mm_struct *mm, nodemask_t *new)
{
}

static inline int huge_node(struct vm_area_struct *vma,
				unsigned long addr, gfp_t gfp_flags,
				struct mempolicy **mpol, nodemask_t **nodemask)
{
	*mpol = NULL;
	*nodemask = NULL;
	return 0;
}

static inline bool init_nodemask_of_mempolicy(nodemask_t *m)
{
	return false;
}

static inline int do_migrate_pages(struct mm_struct *mm, const nodemask_t *from,
				   const nodemask_t *to, int flags)
{
	return 0;
}

static inline void check_highest_zone(int k)
{
}

#ifdef CONFIG_TMPFS
static inline int mpol_parse_str(char *str, struct mempolicy **mpol)
{
	return 1;	/* error */
}
#endif

static inline int mpol_misplaced(struct folio *folio,
				 struct vm_fault *vmf,
				 unsigned long address)
{
	return -1; /* no node preference */
}

static inline void mpol_put_task_policy(struct task_struct *task)
{
}

static inline bool mpol_is_preferred_many(struct mempolicy *pol)
{
	return  false;
}

#endif /* CONFIG_NUMA */
#endif
