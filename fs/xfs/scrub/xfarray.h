/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021-2023 Oracle.  All Rights Reserved.
 * Author: Darrick J. Wong <djwong@kernel.org>
 */
#ifndef __XFS_SCRUB_XFARRAY_H__
#define __XFS_SCRUB_XFARRAY_H__

/* xfile array index type, along with cursor initialization */
typedef uint64_t		xfarray_idx_t;
#define XFARRAY_NULLIDX		((__force xfarray_idx_t)-1ULL)
#define XFARRAY_CURSOR_INIT	((__force xfarray_idx_t)0)

/* Iterate each index of an xfile array. */
#define foreach_xfarray_idx(array, idx) \
	for ((idx) = XFARRAY_CURSOR_INIT; \
	     (idx) < xfarray_length(array); \
	     (idx)++)

struct xfarray {
	/* Underlying file that backs the array. */
	struct xfile	*xfile;

	/* Number of array elements. */
	xfarray_idx_t	nr;

	/* Maximum possible array size. */
	xfarray_idx_t	max_nr;

	/* Number of unset slots in the array below @nr. */
	uint64_t	unset_slots;

	/* Size of an array element. */
	size_t		obj_size;

	/* log2 of array element size, if possible. */
	int		obj_size_log;
};

int xfarray_create(const char *descr, unsigned long long required_capacity,
		size_t obj_size, struct xfarray **arrayp);
void xfarray_destroy(struct xfarray *array);
int xfarray_load(struct xfarray *array, xfarray_idx_t idx, void *ptr);
int xfarray_unset(struct xfarray *array, xfarray_idx_t idx);
int xfarray_store(struct xfarray *array, xfarray_idx_t idx, const void *ptr);
int xfarray_store_anywhere(struct xfarray *array, const void *ptr);
bool xfarray_element_is_null(struct xfarray *array, const void *ptr);
void xfarray_truncate(struct xfarray *array);
unsigned long long xfarray_bytes(struct xfarray *array);

/*
 * Load an array element, but zero the buffer if there's no data because we
 * haven't stored to that array element yet.
 */
static inline int
xfarray_load_sparse(
	struct xfarray	*array,
	uint64_t	idx,
	void		*rec)
{
	int		error = xfarray_load(array, idx, rec);

	if (error == -ENODATA) {
		memset(rec, 0, array->obj_size);
		return 0;
	}
	return error;
}

/* Append an element to the array. */
static inline int xfarray_append(struct xfarray *array, const void *ptr)
{
	return xfarray_store(array, array->nr, ptr);
}

uint64_t xfarray_length(struct xfarray *array);
int xfarray_load_next(struct xfarray *array, xfarray_idx_t *idx, void *rec);

/*
 * Iterate the non-null elements in a sparse xfarray.  Callers should
 * initialize *idx to XFARRAY_CURSOR_INIT before the first call; on return, it
 * will be set to one more than the index of the record that was retrieved.
 * Returns 1 if a record was retrieved, 0 if there weren't any more records, or
 * a negative errno.
 */
static inline int
xfarray_iter(
	struct xfarray	*array,
	xfarray_idx_t	*idx,
	void		*rec)
{
	int ret = xfarray_load_next(array, idx, rec);

	if (ret == -ENODATA)
		return 0;
	if (ret == 0)
		return 1;
	return ret;
}

/* Declarations for xfile array sort functionality. */

typedef cmp_func_t xfarray_cmp_fn;

/* Perform an in-memory heapsort for small subsets. */
#define XFARRAY_ISORT_SHIFT		(4)
#define XFARRAY_ISORT_NR		(1U << XFARRAY_ISORT_SHIFT)

/* Evalulate this many points to find the qsort pivot. */
#define XFARRAY_QSORT_PIVOT_NR		(9)

struct xfarray_sortinfo {
	struct xfarray		*array;

	/* Comparison function for the sort. */
	xfarray_cmp_fn		cmp_fn;

	/* Maximum height of the partition stack. */
	uint8_t			max_stack_depth;

	/* Current height of the partition stack. */
	int8_t			stack_depth;

	/* Maximum stack depth ever used. */
	uint8_t			max_stack_used;

	/* XFARRAY_SORT_* flags; see below. */
	unsigned int		flags;

	/* next time we want to cond_resched() */
	struct xchk_relax	relax;

	/* Cache a folio here for faster scanning for pivots */
	struct folio		*folio;

	/* First array index in folio that is completely readable */
	xfarray_idx_t		first_folio_idx;

	/* Last array index in folio that is completely readable */
	xfarray_idx_t		last_folio_idx;

#ifdef DEBUG
	/* Performance statistics. */
	uint64_t		loads;
	uint64_t		stores;
	uint64_t		compares;
	uint64_t		heapsorts;
#endif
	/*
	 * Extra bytes are allocated beyond the end of the structure to store
	 * quicksort information.  C does not permit multiple VLAs per struct,
	 * so we document all of this in a comment.
	 *
	 * Pretend that we have a typedef for array records:
	 *
	 * typedef char[array->obj_size]	xfarray_rec_t;
	 *
	 * First comes the quicksort partition stack:
	 *
	 * xfarray_idx_t	lo[max_stack_depth];
	 * xfarray_idx_t	hi[max_stack_depth];
	 *
	 * union {
	 *
	 * If for a given subset we decide to use an in-memory sort, we use a
	 * block of scratchpad records here to compare items:
	 *
	 * 	xfarray_rec_t	scratch[ISORT_NR];
	 *
	 * Otherwise, we want to partition the records to partition the array.
	 * We store the chosen pivot record at the start of the scratchpad area
	 * and use the rest to sample some records to estimate the median.
	 * The format of the qsort_pivot array enables us to use the kernel
	 * heapsort function to place the median value in the middle.
	 *
	 * 	struct {
	 * 		xfarray_rec_t	pivot;
	 * 		struct {
	 *			xfarray_rec_t	rec;  (rounded up to 8 bytes)
	 * 			xfarray_idx_t	idx;
	 *		} qsort_pivot[QSORT_PIVOT_NR];
	 * 	};
	 * }
	 */
};

/* Sort can be interrupted by a fatal signal. */
#define XFARRAY_SORT_KILLABLE	(1U << 0)

int xfarray_sort(struct xfarray *array, xfarray_cmp_fn cmp_fn,
		unsigned int flags);

#endif /* __XFS_SCRUB_XFARRAY_H__ */
