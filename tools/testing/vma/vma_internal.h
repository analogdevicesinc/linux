/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * vma_internal.h
 *
 * Header providing userland wrappers and shims for the functionality provided
 * by mm/vma_internal.h.
 *
 * We make the header guard the same as mm/vma_internal.h, so if this shim
 * header is included, it precludes the inclusion of the kernel one.
 */

#ifndef __MM_VMA_INTERNAL_H
#define __MM_VMA_INTERNAL_H

#define __private
#define __bitwise
#define __randomize_layout

#define CONFIG_MMU
#define CONFIG_PER_VMA_LOCK

#include <stdlib.h>

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/maple_tree.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/refcount.h>
#include <linux/slab.h>

extern unsigned long stack_guard_gap;
#ifdef CONFIG_MMU
extern unsigned long mmap_min_addr;
extern unsigned long dac_mmap_min_addr;
#else
#define mmap_min_addr		0UL
#define dac_mmap_min_addr	0UL
#endif

#define VM_WARN_ON(_expr) (WARN_ON(_expr))
#define VM_WARN_ON_ONCE(_expr) (WARN_ON_ONCE(_expr))
#define VM_WARN_ON_VMG(_expr, _vmg) (WARN_ON(_expr))
#define VM_BUG_ON(_expr) (BUG_ON(_expr))
#define VM_BUG_ON_VMA(_expr, _vma) (BUG_ON(_expr))

#define MMF_HAS_MDWE	28

#define VM_NONE		0x00000000
#define VM_READ		0x00000001
#define VM_WRITE	0x00000002
#define VM_EXEC		0x00000004
#define VM_SHARED	0x00000008
#define VM_MAYREAD	0x00000010
#define VM_MAYWRITE	0x00000020
#define VM_MAYEXEC	0x00000040
#define VM_GROWSDOWN	0x00000100
#define VM_PFNMAP	0x00000400
#define VM_LOCKED	0x00002000
#define VM_IO           0x00004000
#define VM_SEQ_READ	0x00008000	/* App will access data sequentially */
#define VM_RAND_READ	0x00010000	/* App will not benefit from clustered reads */
#define VM_DONTEXPAND	0x00040000
#define VM_LOCKONFAULT	0x00080000
#define VM_ACCOUNT	0x00100000
#define VM_NORESERVE	0x00200000
#define VM_MIXEDMAP	0x10000000
#define VM_STACK	VM_GROWSDOWN
#define VM_SHADOW_STACK	VM_NONE
#define VM_SOFTDIRTY	0
#define VM_ARCH_1	0x01000000	/* Architecture-specific flag */
#define VM_GROWSUP	VM_NONE

#define VM_ACCESS_FLAGS (VM_READ | VM_WRITE | VM_EXEC)
#define VM_SPECIAL (VM_IO | VM_DONTEXPAND | VM_PFNMAP | VM_MIXEDMAP)

#ifdef CONFIG_STACK_GROWSUP
#define VM_STACK	VM_GROWSUP
#define VM_STACK_EARLY	VM_GROWSDOWN
#else
#define VM_STACK	VM_GROWSDOWN
#define VM_STACK_EARLY	0
#endif

#define DEFAULT_MAP_WINDOW	((1UL << 47) - PAGE_SIZE)
#define TASK_SIZE_LOW		DEFAULT_MAP_WINDOW
#define TASK_SIZE_MAX		DEFAULT_MAP_WINDOW
#define STACK_TOP		TASK_SIZE_LOW
#define STACK_TOP_MAX		TASK_SIZE_MAX

/* This mask represents all the VMA flag bits used by mlock */
#define VM_LOCKED_MASK	(VM_LOCKED | VM_LOCKONFAULT)

#define TASK_EXEC ((current->personality & READ_IMPLIES_EXEC) ? VM_EXEC : 0)

#define VM_DATA_FLAGS_TSK_EXEC	(VM_READ | VM_WRITE | TASK_EXEC | \
				 VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC)

#define VM_DATA_DEFAULT_FLAGS	VM_DATA_FLAGS_TSK_EXEC

#define VM_STARTGAP_FLAGS (VM_GROWSDOWN | VM_SHADOW_STACK)

#define VM_STACK_DEFAULT_FLAGS VM_DATA_DEFAULT_FLAGS
#define VM_STACK_FLAGS	(VM_STACK | VM_STACK_DEFAULT_FLAGS | VM_ACCOUNT)
#define VM_STACK_INCOMPLETE_SETUP (VM_RAND_READ | VM_SEQ_READ | VM_STACK_EARLY)

#define RLIMIT_STACK		3	/* max stack size */
#define RLIMIT_MEMLOCK		8	/* max locked-in-memory address space */

#define CAP_IPC_LOCK         14

#ifdef CONFIG_64BIT
#define VM_SEALED_BIT	42
#define VM_SEALED	BIT(VM_SEALED_BIT)
#else
#define VM_SEALED	VM_NONE
#endif

#define FIRST_USER_ADDRESS	0UL
#define USER_PGTABLES_CEILING	0UL

#define vma_policy(vma) NULL

#define down_write_nest_lock(sem, nest_lock)

#define pgprot_val(x)		((x).pgprot)
#define __pgprot(x)		((pgprot_t) { (x) } )

#define for_each_vma(__vmi, __vma)					\
	while (((__vma) = vma_next(&(__vmi))) != NULL)

/* The MM code likes to work with exclusive end addresses */
#define for_each_vma_range(__vmi, __vma, __end)				\
	while (((__vma) = vma_find(&(__vmi), (__end))) != NULL)

#define offset_in_page(p)	((unsigned long)(p) & ~PAGE_MASK)

#define PHYS_PFN(x)	((unsigned long)((x) >> PAGE_SHIFT))

#define test_and_set_bit(nr, addr) __test_and_set_bit(nr, addr)
#define test_and_clear_bit(nr, addr) __test_and_clear_bit(nr, addr)

#define TASK_SIZE ((1ul << 47)-PAGE_SIZE)

#define AS_MM_ALL_LOCKS 2

/* We hardcode this for now. */
#define sysctl_max_map_count 0x1000000UL

#define pgoff_t unsigned long
typedef unsigned long	pgprotval_t;
typedef struct pgprot { pgprotval_t pgprot; } pgprot_t;
typedef unsigned long vm_flags_t;
typedef __bitwise unsigned int vm_fault_t;

/*
 * The shared stubs do not implement this, it amounts to an fprintf(STDERR,...)
 * either way :)
 */
#define pr_warn_once pr_err

#define data_race(expr) expr

#define ASSERT_EXCLUSIVE_WRITER(x)

/**
 * swap - swap values of @a and @b
 * @a: first value
 * @b: second value
 */
#define swap(a, b) \
	do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)

struct kref {
	refcount_t refcount;
};

/*
 * Define the task command name length as enum, then it can be visible to
 * BPF programs.
 */
enum {
	TASK_COMM_LEN = 16,
};

/*
 * Flags for bug emulation.
 *
 * These occupy the top three bytes.
 */
enum {
	READ_IMPLIES_EXEC =	0x0400000,
};

struct task_struct {
	char comm[TASK_COMM_LEN];
	pid_t pid;
	struct mm_struct *mm;

	/* Used for emulating ABI behavior of previous Linux versions: */
	unsigned int			personality;
};

struct task_struct *get_current(void);
#define current get_current()

struct anon_vma {
	struct anon_vma *root;
	struct rb_root_cached rb_root;

	/* Test fields. */
	bool was_cloned;
	bool was_unlinked;
};

struct anon_vma_chain {
	struct anon_vma *anon_vma;
	struct list_head same_vma;
};

struct anon_vma_name {
	struct kref kref;
	/* The name needs to be at the end because it is dynamically sized. */
	char name[];
};

struct vma_iterator {
	struct ma_state mas;
};

#define VMA_ITERATOR(name, __mm, __addr)				\
	struct vma_iterator name = {					\
		.mas = {						\
			.tree = &(__mm)->mm_mt,				\
			.index = __addr,				\
			.node = NULL,					\
			.status = ma_start,				\
		},							\
	}

struct address_space {
	struct rb_root_cached	i_mmap;
	unsigned long		flags;
	atomic_t		i_mmap_writable;
};

struct vm_userfaultfd_ctx {};
struct mempolicy {};
struct mmu_gather {};
struct mutex {};
#define DEFINE_MUTEX(mutexname) \
	struct mutex mutexname = {}

#define DECLARE_BITMAP(name, bits) \
	unsigned long name[BITS_TO_LONGS(bits)]

#define NUM_MM_FLAG_BITS (64)
typedef struct {
	__private DECLARE_BITMAP(__mm_flags, NUM_MM_FLAG_BITS);
} mm_flags_t;

struct mm_struct {
	struct maple_tree mm_mt;
	int map_count;			/* number of VMAs */
	unsigned long total_vm;	   /* Total pages mapped */
	unsigned long locked_vm;   /* Pages that have PG_mlocked set */
	unsigned long data_vm;	   /* VM_WRITE & ~VM_SHARED & ~VM_STACK */
	unsigned long exec_vm;	   /* VM_EXEC & ~VM_WRITE & ~VM_STACK */
	unsigned long stack_vm;	   /* VM_STACK */

	unsigned long def_flags;

	mm_flags_t flags; /* Must use mm_flags_* helpers to access */
};

struct vm_area_struct;


/* What action should be taken after an .mmap_prepare call is complete? */
enum mmap_action_type {
	MMAP_NOTHING,		/* Mapping is complete, no further action. */
	MMAP_REMAP_PFN,		/* Remap PFN range. */
	MMAP_IO_REMAP_PFN,	/* I/O remap PFN range. */
};

/*
 * Describes an action an mmap_prepare hook can instruct to be taken to complete
 * the mapping of a VMA. Specified in vm_area_desc.
 */
struct mmap_action {
	union {
		/* Remap range. */
		struct {
			unsigned long start;
			unsigned long start_pfn;
			unsigned long size;
			pgprot_t pgprot;
		} remap;
	};
	enum mmap_action_type type;

	/*
	 * If specified, this hook is invoked after the selected action has been
	 * successfully completed. Note that the VMA write lock still held.
	 *
	 * The absolute minimum ought to be done here.
	 *
	 * Returns 0 on success, or an error code.
	 */
	int (*success_hook)(const struct vm_area_struct *vma);

	/*
	 * If specified, this hook is invoked when an error occurred when
	 * attempting the selection action.
	 *
	 * The hook can return an error code in order to filter the error, but
	 * it is not valid to clear the error here.
	 */
	int (*error_hook)(int err);

	/*
	 * This should be set in rare instances where the operation required
	 * that the rmap should not be able to access the VMA until
	 * completely set up.
	 */
	bool hide_from_rmap_until_complete :1;
};

/*
 * Describes a VMA that is about to be mmap()'ed. Drivers may choose to
 * manipulate mutable fields which will cause those fields to be updated in the
 * resultant VMA.
 *
 * Helper functions are not required for manipulating any field.
 */
struct vm_area_desc {
	/* Immutable state. */
	const struct mm_struct *const mm;
	struct file *const file; /* May vary from vm_file in stacked callers. */
	unsigned long start;
	unsigned long end;

	/* Mutable fields. Populated with initial state. */
	pgoff_t pgoff;
	struct file *vm_file;
	vm_flags_t vm_flags;
	pgprot_t page_prot;

	/* Write-only fields. */
	const struct vm_operations_struct *vm_ops;
	void *private_data;

	/* Take further action? */
	struct mmap_action action;
};

struct file_operations {
	int (*mmap)(struct file *, struct vm_area_struct *);
	int (*mmap_prepare)(struct vm_area_desc *);
};

struct file {
	struct address_space	*f_mapping;
	const struct file_operations	*f_op;
};

#define VMA_LOCK_OFFSET	0x40000000

typedef struct { unsigned long v; } freeptr_t;

struct vm_area_struct {
	/* The first cache line has the info for VMA tree walking. */

	union {
		struct {
			/* VMA covers [vm_start; vm_end) addresses within mm */
			unsigned long vm_start;
			unsigned long vm_end;
		};
		freeptr_t vm_freeptr; /* Pointer used by SLAB_TYPESAFE_BY_RCU */
	};

	struct mm_struct *vm_mm;	/* The address space we belong to. */
	pgprot_t vm_page_prot;          /* Access permissions of this VMA. */

	/*
	 * Flags, see mm.h.
	 * To modify use vm_flags_{init|reset|set|clear|mod} functions.
	 */
	union {
		const vm_flags_t vm_flags;
		vm_flags_t __private __vm_flags;
	};

#ifdef CONFIG_PER_VMA_LOCK
	/*
	 * Can only be written (using WRITE_ONCE()) while holding both:
	 *  - mmap_lock (in write mode)
	 *  - vm_refcnt bit at VMA_LOCK_OFFSET is set
	 * Can be read reliably while holding one of:
	 *  - mmap_lock (in read or write mode)
	 *  - vm_refcnt bit at VMA_LOCK_OFFSET is set or vm_refcnt > 1
	 * Can be read unreliably (using READ_ONCE()) for pessimistic bailout
	 * while holding nothing (except RCU to keep the VMA struct allocated).
	 *
	 * This sequence counter is explicitly allowed to overflow; sequence
	 * counter reuse can only lead to occasional unnecessary use of the
	 * slowpath.
	 */
	unsigned int vm_lock_seq;
#endif

	/*
	 * A file's MAP_PRIVATE vma can be in both i_mmap tree and anon_vma
	 * list, after a COW of one of the file pages.	A MAP_SHARED vma
	 * can only be in the i_mmap tree.  An anonymous MAP_PRIVATE, stack
	 * or brk vma (with NULL file) can only be in an anon_vma list.
	 */
	struct list_head anon_vma_chain; /* Serialized by mmap_lock &
					  * page_table_lock */
	struct anon_vma *anon_vma;	/* Serialized by page_table_lock */

	/* Function pointers to deal with this struct. */
	const struct vm_operations_struct *vm_ops;

	/* Information about our backing store: */
	unsigned long vm_pgoff;		/* Offset (within vm_file) in PAGE_SIZE
					   units */
	struct file * vm_file;		/* File we map to (can be NULL). */
	void * vm_private_data;		/* was vm_pte (shared mem) */

#ifdef CONFIG_SWAP
	atomic_long_t swap_readahead_info;
#endif
#ifndef CONFIG_MMU
	struct vm_region *vm_region;	/* NOMMU mapping region */
#endif
#ifdef CONFIG_NUMA
	struct mempolicy *vm_policy;	/* NUMA policy for the VMA */
#endif
#ifdef CONFIG_NUMA_BALANCING
	struct vma_numab_state *numab_state;	/* NUMA Balancing state */
#endif
#ifdef CONFIG_PER_VMA_LOCK
	/* Unstable RCU readers are allowed to read this. */
	refcount_t vm_refcnt;
#endif
	/*
	 * For areas with an address space and backing store,
	 * linkage into the address_space->i_mmap interval tree.
	 *
	 */
	struct {
		struct rb_node rb;
		unsigned long rb_subtree_last;
	} shared;
#ifdef CONFIG_ANON_VMA_NAME
	/*
	 * For private and shared anonymous mappings, a pointer to a null
	 * terminated string containing the name given to the vma, or NULL if
	 * unnamed. Serialized by mmap_lock. Use anon_vma_name to access.
	 */
	struct anon_vma_name *anon_name;
#endif
	struct vm_userfaultfd_ctx vm_userfaultfd_ctx;
} __randomize_layout;

struct vm_fault {};

struct vm_operations_struct {
	void (*open)(struct vm_area_struct * area);
	/**
	 * @close: Called when the VMA is being removed from the MM.
	 * Context: User context.  May sleep.  Caller holds mmap_lock.
	 */
	void (*close)(struct vm_area_struct * area);
	/* Called any time before splitting to check if it's allowed */
	int (*may_split)(struct vm_area_struct *area, unsigned long addr);
	int (*mremap)(struct vm_area_struct *area);
	/*
	 * Called by mprotect() to make driver-specific permission
	 * checks before mprotect() is finalised.   The VMA must not
	 * be modified.  Returns 0 if mprotect() can proceed.
	 */
	int (*mprotect)(struct vm_area_struct *vma, unsigned long start,
			unsigned long end, unsigned long newflags);
	vm_fault_t (*fault)(struct vm_fault *vmf);
	vm_fault_t (*huge_fault)(struct vm_fault *vmf, unsigned int order);
	vm_fault_t (*map_pages)(struct vm_fault *vmf,
			pgoff_t start_pgoff, pgoff_t end_pgoff);
	unsigned long (*pagesize)(struct vm_area_struct * area);

	/* notification that a previously read-only page is about to become
	 * writable, if an error is returned it will cause a SIGBUS */
	vm_fault_t (*page_mkwrite)(struct vm_fault *vmf);

	/* same as page_mkwrite when using VM_PFNMAP|VM_MIXEDMAP */
	vm_fault_t (*pfn_mkwrite)(struct vm_fault *vmf);

	/* called by access_process_vm when get_user_pages() fails, typically
	 * for use by special VMAs. See also generic_access_phys() for a generic
	 * implementation useful for any iomem mapping.
	 */
	int (*access)(struct vm_area_struct *vma, unsigned long addr,
		      void *buf, int len, int write);

	/* Called by the /proc/PID/maps code to ask the vma whether it
	 * has a special name.  Returning non-NULL will also cause this
	 * vma to be dumped unconditionally. */
	const char *(*name)(struct vm_area_struct *vma);

#ifdef CONFIG_NUMA
	/*
	 * set_policy() op must add a reference to any non-NULL @new mempolicy
	 * to hold the policy upon return.  Caller should pass NULL @new to
	 * remove a policy and fall back to surrounding context--i.e. do not
	 * install a MPOL_DEFAULT policy, nor the task or system default
	 * mempolicy.
	 */
	int (*set_policy)(struct vm_area_struct *vma, struct mempolicy *new);

	/*
	 * get_policy() op must add reference [mpol_get()] to any policy at
	 * (vma,addr) marked as MPOL_SHARED.  The shared policy infrastructure
	 * in mm/mempolicy.c will do this automatically.
	 * get_policy() must NOT add a ref if the policy at (vma,addr) is not
	 * marked as MPOL_SHARED. vma policies are protected by the mmap_lock.
	 * If no [shared/vma] mempolicy exists at the addr, get_policy() op
	 * must return NULL--i.e., do not "fallback" to task or system default
	 * policy.
	 */
	struct mempolicy *(*get_policy)(struct vm_area_struct *vma,
					unsigned long addr, pgoff_t *ilx);
#endif
#ifdef CONFIG_FIND_NORMAL_PAGE
	/*
	 * Called by vm_normal_page() for special PTEs in @vma at @addr. This
	 * allows for returning a "normal" page from vm_normal_page() even
	 * though the PTE indicates that the "struct page" either does not exist
	 * or should not be touched: "special".
	 *
	 * Do not add new users: this really only works when a "normal" page
	 * was mapped, but then the PTE got changed to something weird (+
	 * marked special) that would not make pte_pfn() identify the originally
	 * inserted page.
	 */
	struct page *(*find_normal_page)(struct vm_area_struct *vma,
					 unsigned long addr);
#endif /* CONFIG_FIND_NORMAL_PAGE */
};

struct vm_unmapped_area_info {
#define VM_UNMAPPED_AREA_TOPDOWN 1
	unsigned long flags;
	unsigned long length;
	unsigned long low_limit;
	unsigned long high_limit;
	unsigned long align_mask;
	unsigned long align_offset;
	unsigned long start_gap;
};

struct pagetable_move_control {
	struct vm_area_struct *old; /* Source VMA. */
	struct vm_area_struct *new; /* Destination VMA. */
	unsigned long old_addr; /* Address from which the move begins. */
	unsigned long old_end; /* Exclusive address at which old range ends. */
	unsigned long new_addr; /* Address to move page tables to. */
	unsigned long len_in; /* Bytes to remap specified by user. */

	bool need_rmap_locks; /* Do rmap locks need to be taken? */
	bool for_stack; /* Is this an early temp stack being moved? */
};

#define PAGETABLE_MOVE(name, old_, new_, old_addr_, new_addr_, len_)	\
	struct pagetable_move_control name = {				\
		.old = old_,						\
		.new = new_,						\
		.old_addr = old_addr_,					\
		.old_end = (old_addr_) + (len_),			\
		.new_addr = new_addr_,					\
		.len_in = len_,						\
	}

static inline void vma_iter_invalidate(struct vma_iterator *vmi)
{
	mas_pause(&vmi->mas);
}

static inline pgprot_t pgprot_modify(pgprot_t oldprot, pgprot_t newprot)
{
	return __pgprot(pgprot_val(oldprot) | pgprot_val(newprot));
}

static inline pgprot_t vm_get_page_prot(vm_flags_t vm_flags)
{
	return __pgprot(vm_flags);
}

static inline bool is_shared_maywrite(vm_flags_t vm_flags)
{
	return (vm_flags & (VM_SHARED | VM_MAYWRITE)) ==
		(VM_SHARED | VM_MAYWRITE);
}

static inline bool vma_is_shared_maywrite(struct vm_area_struct *vma)
{
	return is_shared_maywrite(vma->vm_flags);
}

static inline struct vm_area_struct *vma_next(struct vma_iterator *vmi)
{
	/*
	 * Uses mas_find() to get the first VMA when the iterator starts.
	 * Calling mas_next() could skip the first entry.
	 */
	return mas_find(&vmi->mas, ULONG_MAX);
}

/*
 * WARNING: to avoid racing with vma_mark_attached()/vma_mark_detached(), these
 * assertions should be made either under mmap_write_lock or when the object
 * has been isolated under mmap_write_lock, ensuring no competing writers.
 */
static inline void vma_assert_attached(struct vm_area_struct *vma)
{
	WARN_ON_ONCE(!refcount_read(&vma->vm_refcnt));
}

static inline void vma_assert_detached(struct vm_area_struct *vma)
{
	WARN_ON_ONCE(refcount_read(&vma->vm_refcnt));
}

static inline void vma_assert_write_locked(struct vm_area_struct *);
static inline void vma_mark_attached(struct vm_area_struct *vma)
{
	vma_assert_write_locked(vma);
	vma_assert_detached(vma);
	refcount_set_release(&vma->vm_refcnt, 1);
}

static inline void vma_mark_detached(struct vm_area_struct *vma)
{
	vma_assert_write_locked(vma);
	vma_assert_attached(vma);
	/* We are the only writer, so no need to use vma_refcount_put(). */
	if (unlikely(!refcount_dec_and_test(&vma->vm_refcnt))) {
		/*
		 * Reader must have temporarily raised vm_refcnt but it will
		 * drop it without using the vma since vma is write-locked.
		 */
	}
}

extern const struct vm_operations_struct vma_dummy_vm_ops;

extern unsigned long rlimit(unsigned int limit);

static inline void vma_init(struct vm_area_struct *vma, struct mm_struct *mm)
{
	memset(vma, 0, sizeof(*vma));
	vma->vm_mm = mm;
	vma->vm_ops = &vma_dummy_vm_ops;
	INIT_LIST_HEAD(&vma->anon_vma_chain);
	vma->vm_lock_seq = UINT_MAX;
}

/*
 * These are defined in vma.h, but sadly vm_stat_account() is referenced by
 * kernel/fork.c, so we have to these broadly available there, and temporarily
 * define them here to resolve the dependency cycle.
 */

#define is_exec_mapping(flags) \
	((flags & (VM_EXEC | VM_WRITE | VM_STACK)) == VM_EXEC)

#define is_stack_mapping(flags) \
	(((flags & VM_STACK) == VM_STACK) || (flags & VM_SHADOW_STACK))

#define is_data_mapping(flags) \
	((flags & (VM_WRITE | VM_SHARED | VM_STACK)) == VM_WRITE)

static inline void vm_stat_account(struct mm_struct *mm, vm_flags_t flags,
				   long npages)
{
	WRITE_ONCE(mm->total_vm, READ_ONCE(mm->total_vm)+npages);

	if (is_exec_mapping(flags))
		mm->exec_vm += npages;
	else if (is_stack_mapping(flags))
		mm->stack_vm += npages;
	else if (is_data_mapping(flags))
		mm->data_vm += npages;
}

#undef is_exec_mapping
#undef is_stack_mapping
#undef is_data_mapping

/* Currently stubbed but we may later wish to un-stub. */
static inline void vm_acct_memory(long pages);
static inline void vm_unacct_memory(long pages)
{
	vm_acct_memory(-pages);
}

static inline void mapping_allow_writable(struct address_space *mapping)
{
	atomic_inc(&mapping->i_mmap_writable);
}

static inline void vma_set_range(struct vm_area_struct *vma,
				 unsigned long start, unsigned long end,
				 pgoff_t pgoff)
{
	vma->vm_start = start;
	vma->vm_end = end;
	vma->vm_pgoff = pgoff;
}

static inline
struct vm_area_struct *vma_find(struct vma_iterator *vmi, unsigned long max)
{
	return mas_find(&vmi->mas, max - 1);
}

static inline int vma_iter_clear_gfp(struct vma_iterator *vmi,
			unsigned long start, unsigned long end, gfp_t gfp)
{
	__mas_set_range(&vmi->mas, start, end - 1);
	mas_store_gfp(&vmi->mas, NULL, gfp);
	if (unlikely(mas_is_err(&vmi->mas)))
		return -ENOMEM;

	return 0;
}

static inline void mmap_assert_locked(struct mm_struct *);
static inline struct vm_area_struct *find_vma_intersection(struct mm_struct *mm,
						unsigned long start_addr,
						unsigned long end_addr)
{
	unsigned long index = start_addr;

	mmap_assert_locked(mm);
	return mt_find(&mm->mm_mt, &index, end_addr - 1);
}

static inline
struct vm_area_struct *vma_lookup(struct mm_struct *mm, unsigned long addr)
{
	return mtree_load(&mm->mm_mt, addr);
}

static inline struct vm_area_struct *vma_prev(struct vma_iterator *vmi)
{
	return mas_prev(&vmi->mas, 0);
}

static inline void vma_iter_set(struct vma_iterator *vmi, unsigned long addr)
{
	mas_set(&vmi->mas, addr);
}

static inline bool vma_is_anonymous(struct vm_area_struct *vma)
{
	return !vma->vm_ops;
}

/* Defined in vma.h, so temporarily define here to avoid circular dependency. */
#define vma_iter_load(vmi) \
	mas_walk(&(vmi)->mas)

static inline struct vm_area_struct *
find_vma_prev(struct mm_struct *mm, unsigned long addr,
			struct vm_area_struct **pprev)
{
	struct vm_area_struct *vma;
	VMA_ITERATOR(vmi, mm, addr);

	vma = vma_iter_load(&vmi);
	*pprev = vma_prev(&vmi);
	if (!vma)
		vma = vma_next(&vmi);
	return vma;
}

#undef vma_iter_load

static inline void vma_iter_init(struct vma_iterator *vmi,
		struct mm_struct *mm, unsigned long addr)
{
	mas_init(&vmi->mas, &mm->mm_mt, addr);
}

/* Stubbed functions. */

static inline struct anon_vma_name *anon_vma_name(struct vm_area_struct *vma)
{
	return NULL;
}

static inline bool is_mergeable_vm_userfaultfd_ctx(struct vm_area_struct *vma,
					struct vm_userfaultfd_ctx vm_ctx)
{
	return true;
}

static inline bool anon_vma_name_eq(struct anon_vma_name *anon_name1,
				    struct anon_vma_name *anon_name2)
{
	return true;
}

static inline void might_sleep(void)
{
}

static inline unsigned long vma_pages(struct vm_area_struct *vma)
{
	return (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
}

static inline void fput(struct file *file)
{
}

static inline void mpol_put(struct mempolicy *pol)
{
}

static inline void lru_add_drain(void)
{
}

static inline void tlb_gather_mmu(struct mmu_gather *tlb, struct mm_struct *mm)
{
}

static inline void update_hiwater_rss(struct mm_struct *mm)
{
}

static inline void update_hiwater_vm(struct mm_struct *mm)
{
}

static inline void unmap_vmas(struct mmu_gather *tlb, struct ma_state *mas,
		      struct vm_area_struct *vma, unsigned long start_addr,
		      unsigned long end_addr, unsigned long tree_end,
		      bool mm_wr_locked)
{
}

static inline void free_pgtables(struct mmu_gather *tlb, struct ma_state *mas,
		   struct vm_area_struct *vma, unsigned long floor,
		   unsigned long ceiling, bool mm_wr_locked)
{
}

static inline void mapping_unmap_writable(struct address_space *mapping)
{
}

static inline void flush_dcache_mmap_lock(struct address_space *mapping)
{
}

static inline void tlb_finish_mmu(struct mmu_gather *tlb)
{
}

static inline struct file *get_file(struct file *f)
{
	return f;
}

static inline int vma_dup_policy(struct vm_area_struct *src, struct vm_area_struct *dst)
{
	return 0;
}

static inline int anon_vma_clone(struct vm_area_struct *dst, struct vm_area_struct *src)
{
	/* For testing purposes. We indicate that an anon_vma has been cloned. */
	if (src->anon_vma != NULL) {
		dst->anon_vma = src->anon_vma;
		dst->anon_vma->was_cloned = true;
	}

	return 0;
}

static inline void vma_start_write(struct vm_area_struct *vma)
{
	/* Used to indicate to tests that a write operation has begun. */
	vma->vm_lock_seq++;
}

static inline void vma_adjust_trans_huge(struct vm_area_struct *vma,
					 unsigned long start,
					 unsigned long end,
					 struct vm_area_struct *next)
{
}

static inline void hugetlb_split(struct vm_area_struct *, unsigned long) {}

static inline void vma_iter_free(struct vma_iterator *vmi)
{
	mas_destroy(&vmi->mas);
}

static inline
struct vm_area_struct *vma_iter_next_range(struct vma_iterator *vmi)
{
	return mas_next_range(&vmi->mas, ULONG_MAX);
}

static inline void vm_acct_memory(long pages)
{
}

static inline void vma_interval_tree_insert(struct vm_area_struct *vma,
					    struct rb_root_cached *rb)
{
}

static inline void vma_interval_tree_remove(struct vm_area_struct *vma,
					    struct rb_root_cached *rb)
{
}

static inline void flush_dcache_mmap_unlock(struct address_space *mapping)
{
}

static inline void anon_vma_interval_tree_insert(struct anon_vma_chain *avc,
						 struct rb_root_cached *rb)
{
}

static inline void anon_vma_interval_tree_remove(struct anon_vma_chain *avc,
						 struct rb_root_cached *rb)
{
}

static inline void uprobe_mmap(struct vm_area_struct *vma)
{
}

static inline void uprobe_munmap(struct vm_area_struct *vma,
				 unsigned long start, unsigned long end)
{
}

static inline void i_mmap_lock_write(struct address_space *mapping)
{
}

static inline void anon_vma_lock_write(struct anon_vma *anon_vma)
{
}

static inline void vma_assert_write_locked(struct vm_area_struct *vma)
{
}

static inline void unlink_anon_vmas(struct vm_area_struct *vma)
{
	/* For testing purposes, indicate that the anon_vma was unlinked. */
	vma->anon_vma->was_unlinked = true;
}

static inline void anon_vma_unlock_write(struct anon_vma *anon_vma)
{
}

static inline void i_mmap_unlock_write(struct address_space *mapping)
{
}

static inline void anon_vma_merge(struct vm_area_struct *vma,
				  struct vm_area_struct *next)
{
}

static inline int userfaultfd_unmap_prep(struct vm_area_struct *vma,
					 unsigned long start,
					 unsigned long end,
					 struct list_head *unmaps)
{
	return 0;
}

static inline void mmap_write_downgrade(struct mm_struct *mm)
{
}

static inline void mmap_read_unlock(struct mm_struct *mm)
{
}

static inline void mmap_write_unlock(struct mm_struct *mm)
{
}

static inline int mmap_write_lock_killable(struct mm_struct *mm)
{
	return 0;
}

static inline bool can_modify_mm(struct mm_struct *mm,
				 unsigned long start,
				 unsigned long end)
{
	return true;
}

static inline void arch_unmap(struct mm_struct *mm,
				 unsigned long start,
				 unsigned long end)
{
}

static inline void mmap_assert_locked(struct mm_struct *mm)
{
}

static inline bool mpol_equal(struct mempolicy *a, struct mempolicy *b)
{
	return true;
}

static inline void khugepaged_enter_vma(struct vm_area_struct *vma,
			  vm_flags_t vm_flags)
{
}

static inline bool mapping_can_writeback(struct address_space *mapping)
{
	return true;
}

static inline bool is_vm_hugetlb_page(struct vm_area_struct *vma)
{
	return false;
}

static inline bool vma_soft_dirty_enabled(struct vm_area_struct *vma)
{
	return false;
}

static inline bool userfaultfd_wp(struct vm_area_struct *vma)
{
	return false;
}

static inline void mmap_assert_write_locked(struct mm_struct *mm)
{
}

static inline void mutex_lock(struct mutex *lock)
{
}

static inline void mutex_unlock(struct mutex *lock)
{
}

static inline bool mutex_is_locked(struct mutex *lock)
{
	return true;
}

static inline bool signal_pending(void *p)
{
	return false;
}

static inline bool is_file_hugepages(struct file *file)
{
	return false;
}

static inline int security_vm_enough_memory_mm(struct mm_struct *mm, long pages)
{
	return 0;
}

static inline bool may_expand_vm(struct mm_struct *mm, vm_flags_t flags,
				 unsigned long npages)
{
	return true;
}

static inline void vm_flags_init(struct vm_area_struct *vma,
				 vm_flags_t flags)
{
	vma->__vm_flags = flags;
}

static inline void vm_flags_set(struct vm_area_struct *vma,
				vm_flags_t flags)
{
	vma_start_write(vma);
	vma->__vm_flags |= flags;
}

static inline void vm_flags_clear(struct vm_area_struct *vma,
				  vm_flags_t flags)
{
	vma_start_write(vma);
	vma->__vm_flags &= ~flags;
}

static inline int shmem_zero_setup(struct vm_area_struct *vma)
{
	return 0;
}

static inline void vma_set_anonymous(struct vm_area_struct *vma)
{
	vma->vm_ops = NULL;
}

static inline void ksm_add_vma(struct vm_area_struct *vma)
{
}

static inline void perf_event_mmap(struct vm_area_struct *vma)
{
}

static inline bool vma_is_dax(struct vm_area_struct *vma)
{
	return false;
}

static inline struct vm_area_struct *get_gate_vma(struct mm_struct *mm)
{
	return NULL;
}

bool vma_wants_writenotify(struct vm_area_struct *vma, pgprot_t vm_page_prot);

/* Update vma->vm_page_prot to reflect vma->vm_flags. */
static inline void vma_set_page_prot(struct vm_area_struct *vma)
{
	vm_flags_t vm_flags = vma->vm_flags;
	pgprot_t vm_page_prot;

	/* testing: we inline vm_pgprot_modify() to avoid clash with vma.h. */
	vm_page_prot = pgprot_modify(vma->vm_page_prot, vm_get_page_prot(vm_flags));

	if (vma_wants_writenotify(vma, vm_page_prot)) {
		vm_flags &= ~VM_SHARED;
		/* testing: we inline vm_pgprot_modify() to avoid clash with vma.h. */
		vm_page_prot = pgprot_modify(vm_page_prot, vm_get_page_prot(vm_flags));
	}
	/* remove_protection_ptes reads vma->vm_page_prot without mmap_lock */
	WRITE_ONCE(vma->vm_page_prot, vm_page_prot);
}

static inline bool arch_validate_flags(vm_flags_t flags)
{
	return true;
}

static inline void vma_close(struct vm_area_struct *vma)
{
}

static inline int mmap_file(struct file *file, struct vm_area_struct *vma)
{
	return 0;
}

static inline unsigned long stack_guard_start_gap(struct vm_area_struct *vma)
{
	if (vma->vm_flags & VM_GROWSDOWN)
		return stack_guard_gap;

	/* See reasoning around the VM_SHADOW_STACK definition */
	if (vma->vm_flags & VM_SHADOW_STACK)
		return PAGE_SIZE;

	return 0;
}

static inline unsigned long vm_start_gap(struct vm_area_struct *vma)
{
	unsigned long gap = stack_guard_start_gap(vma);
	unsigned long vm_start = vma->vm_start;

	vm_start -= gap;
	if (vm_start > vma->vm_start)
		vm_start = 0;
	return vm_start;
}

static inline unsigned long vm_end_gap(struct vm_area_struct *vma)
{
	unsigned long vm_end = vma->vm_end;

	if (vma->vm_flags & VM_GROWSUP) {
		vm_end += stack_guard_gap;
		if (vm_end < vma->vm_end)
			vm_end = -PAGE_SIZE;
	}
	return vm_end;
}

static inline int is_hugepage_only_range(struct mm_struct *mm,
					unsigned long addr, unsigned long len)
{
	return 0;
}

static inline bool vma_is_accessible(struct vm_area_struct *vma)
{
	return vma->vm_flags & VM_ACCESS_FLAGS;
}

static inline bool capable(int cap)
{
	return true;
}

static inline bool mlock_future_ok(const struct mm_struct *mm,
		vm_flags_t vm_flags, unsigned long bytes)
{
	unsigned long locked_pages, limit_pages;

	if (!(vm_flags & VM_LOCKED) || capable(CAP_IPC_LOCK))
		return true;

	locked_pages = bytes >> PAGE_SHIFT;
	locked_pages += mm->locked_vm;

	limit_pages = rlimit(RLIMIT_MEMLOCK);
	limit_pages >>= PAGE_SHIFT;

	return locked_pages <= limit_pages;
}

static inline int __anon_vma_prepare(struct vm_area_struct *vma)
{
	struct anon_vma *anon_vma = calloc(1, sizeof(struct anon_vma));

	if (!anon_vma)
		return -ENOMEM;

	anon_vma->root = anon_vma;
	vma->anon_vma = anon_vma;

	return 0;
}

static inline int anon_vma_prepare(struct vm_area_struct *vma)
{
	if (likely(vma->anon_vma))
		return 0;

	return __anon_vma_prepare(vma);
}

static inline void userfaultfd_unmap_complete(struct mm_struct *mm,
					      struct list_head *uf)
{
}

# define ACCESS_PRIVATE(p, member) ((p)->member)

static inline bool mm_flags_test(int flag, const struct mm_struct *mm)
{
	return test_bit(flag, ACCESS_PRIVATE(&mm->flags, __mm_flags));
}

/*
 * Denies creating a writable executable mapping or gaining executable permissions.
 *
 * This denies the following:
 *
 *     a)      mmap(PROT_WRITE | PROT_EXEC)
 *
 *     b)      mmap(PROT_WRITE)
 *             mprotect(PROT_EXEC)
 *
 *     c)      mmap(PROT_WRITE)
 *             mprotect(PROT_READ)
 *             mprotect(PROT_EXEC)
 *
 * But allows the following:
 *
 *     d)      mmap(PROT_READ | PROT_EXEC)
 *             mmap(PROT_READ | PROT_EXEC | PROT_BTI)
 *
 * This is only applicable if the user has set the Memory-Deny-Write-Execute
 * (MDWE) protection mask for the current process.
 *
 * @old specifies the VMA flags the VMA originally possessed, and @new the ones
 * we propose to set.
 *
 * Return: false if proposed change is OK, true if not ok and should be denied.
 */
static inline bool map_deny_write_exec(unsigned long old, unsigned long new)
{
	/* If MDWE is disabled, we have nothing to deny. */
	if (mm_flags_test(MMF_HAS_MDWE, current->mm))
		return false;

	/* If the new VMA is not executable, we have nothing to deny. */
	if (!(new & VM_EXEC))
		return false;

	/* Under MDWE we do not accept newly writably executable VMAs... */
	if (new & VM_WRITE)
		return true;

	/* ...nor previously non-executable VMAs becoming executable. */
	if (!(old & VM_EXEC))
		return true;

	return false;
}

static inline int mapping_map_writable(struct address_space *mapping)
{
	return atomic_inc_unless_negative(&mapping->i_mmap_writable) ?
		0 : -EPERM;
}

static inline unsigned long move_page_tables(struct pagetable_move_control *pmc)
{
	return 0;
}

static inline void free_pgd_range(struct mmu_gather *tlb,
			unsigned long addr, unsigned long end,
			unsigned long floor, unsigned long ceiling)
{
}

static inline int ksm_execve(struct mm_struct *mm)
{
	return 0;
}

static inline void ksm_exit(struct mm_struct *mm)
{
}

static inline void vma_lock_init(struct vm_area_struct *vma, bool reset_refcnt)
{
	if (reset_refcnt)
		refcount_set(&vma->vm_refcnt, 0);
}

static inline void vma_numab_state_init(struct vm_area_struct *vma)
{
}

static inline void vma_numab_state_free(struct vm_area_struct *vma)
{
}

static inline void dup_anon_vma_name(struct vm_area_struct *orig_vma,
				     struct vm_area_struct *new_vma)
{
}

static inline void free_anon_vma_name(struct vm_area_struct *vma)
{
}

/* Declared in vma.h. */
static inline void set_vma_from_desc(struct vm_area_struct *vma,
		struct vm_area_desc *desc);

static inline void mmap_action_prepare(struct mmap_action *action,
					   struct vm_area_desc *desc)
{
}

static inline int mmap_action_complete(struct mmap_action *action,
					   struct vm_area_struct *vma)
{
	return 0;
}

static inline int __compat_vma_mmap(const struct file_operations *f_op,
		struct file *file, struct vm_area_struct *vma)
{
	struct vm_area_desc desc = {
		.mm = vma->vm_mm,
		.file = file,
		.start = vma->vm_start,
		.end = vma->vm_end,

		.pgoff = vma->vm_pgoff,
		.vm_file = vma->vm_file,
		.vm_flags = vma->vm_flags,
		.page_prot = vma->vm_page_prot,

		.action.type = MMAP_NOTHING, /* Default */
	};
	int err;

	err = f_op->mmap_prepare(&desc);
	if (err)
		return err;

	mmap_action_prepare(&desc.action, &desc);
	set_vma_from_desc(vma, &desc);
	return mmap_action_complete(&desc.action, vma);
}

static inline int compat_vma_mmap(struct file *file,
		struct vm_area_struct *vma)
{
	return __compat_vma_mmap(file->f_op, file, vma);
}

/* Did the driver provide valid mmap hook configuration? */
static inline bool can_mmap_file(struct file *file)
{
	bool has_mmap = file->f_op->mmap;
	bool has_mmap_prepare = file->f_op->mmap_prepare;

	/* Hooks are mutually exclusive. */
	if (WARN_ON_ONCE(has_mmap && has_mmap_prepare))
		return false;
	if (!has_mmap && !has_mmap_prepare)
		return false;

	return true;
}

static inline int vfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	if (file->f_op->mmap_prepare)
		return compat_vma_mmap(file, vma);

	return file->f_op->mmap(file, vma);
}

static inline int vfs_mmap_prepare(struct file *file, struct vm_area_desc *desc)
{
	return file->f_op->mmap_prepare(desc);
}

static inline void fixup_hugetlb_reservations(struct vm_area_struct *vma)
{
}

static inline void vma_set_file(struct vm_area_struct *vma, struct file *file)
{
	/* Changing an anonymous vma with this is illegal */
	get_file(file);
	swap(vma->vm_file, file);
	fput(file);
}

static inline bool shmem_file(struct file *file)
{
	return false;
}

static inline vm_flags_t ksm_vma_flags(const struct mm_struct *mm,
		const struct file *file, vm_flags_t vm_flags)
{
	return vm_flags;
}

static inline void remap_pfn_range_prepare(struct vm_area_desc *desc, unsigned long pfn)
{
}

static inline int remap_pfn_range_complete(struct vm_area_struct *vma, unsigned long addr,
		unsigned long pfn, unsigned long size, pgprot_t pgprot)
{
	return 0;
}

static inline int do_munmap(struct mm_struct *, unsigned long, size_t,
		struct list_head *uf)
{
	return 0;
}

#endif	/* __MM_VMA_INTERNAL_H */
