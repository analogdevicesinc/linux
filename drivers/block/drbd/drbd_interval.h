/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2011, LINBIT HA-Solutions GmbH.
 */

#ifndef __DRBD_INTERVAL_H
#define __DRBD_INTERVAL_H

#include <linux/types.h>
#include <linux/rbtree.h>

/*
 * Interval types stored directly in drbd_interval so that we can handle
 * conflicts without having to inspect the containing object. The value 0 is
 * reserved for uninitialized intervals.
 */
enum drbd_interval_type {
	INTERVAL_LOCAL_WRITE = 1,
	INTERVAL_PEER_WRITE,
	INTERVAL_LOCAL_READ,
	INTERVAL_PEER_READ,
	INTERVAL_RESYNC_WRITE, /* C_SYNC_TARGET */
	INTERVAL_RESYNC_READ, /* C_SYNC_SOURCE */
	INTERVAL_OV_READ_SOURCE, /* C_VERIFY_S */
	INTERVAL_OV_READ_TARGET, /* C_VERIFY_T */
};

enum drbd_interval_flags {
	/* Someone is waiting on device->misc_wait for this to make progress. */
	INTERVAL_WAITING,

	/* This has been completed already; ignore for conflict detection. */
	INTERVAL_COMPLETED,
};

struct drbd_interval {
	struct rb_node rb;
	sector_t sector;		/* start sector of the interval */
	sector_t end;			/* highest interval end in subtree */
	unsigned int size;		/* size in bytes */
	enum drbd_interval_type type;	/* what type of interval this is */
	unsigned long flags;

	/* to resume a partially successful drbd_al_begin_io_nonblock(); */
	unsigned int partially_in_al_next_enr;
};

static inline bool drbd_interval_is_local(struct drbd_interval *i)
{
	return i->type == INTERVAL_LOCAL_READ || i->type == INTERVAL_LOCAL_WRITE;
}

static inline void drbd_clear_interval(struct drbd_interval *i)
{
	RB_CLEAR_NODE(&i->rb);
}

static inline bool drbd_interval_empty(struct drbd_interval *i)
{
	return RB_EMPTY_NODE(&i->rb);
}

bool drbd_insert_interval(struct rb_root *root, struct drbd_interval *this);
bool drbd_contains_interval(struct rb_root *root, sector_t sector,
			    struct drbd_interval *interval);
void drbd_remove_interval(struct rb_root *root, struct drbd_interval *this);
struct drbd_interval *drbd_find_overlap(struct rb_root *root, sector_t sector,
					unsigned int size);
struct drbd_interval *drbd_next_overlap(struct drbd_interval *i,
					sector_t sector, unsigned int size);

#define drbd_for_each_overlap(i, root, sector, size)		\
	for (i = drbd_find_overlap(root, sector, size);		\
	     i;							\
	     i = drbd_next_overlap(i, sector, size))

#endif  /* __DRBD_INTERVAL_H */
