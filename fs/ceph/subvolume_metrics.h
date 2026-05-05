/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _FS_CEPH_SUBVOLUME_METRICS_H
#define _FS_CEPH_SUBVOLUME_METRICS_H

#include <linux/types.h>
#include <linux/rbtree.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/atomic.h>

struct seq_file;
struct ceph_mds_client;
struct ceph_inode_info;

/**
 * struct ceph_subvol_metric_snapshot - Point-in-time snapshot of subvolume metrics
 * @subvolume_id: Subvolume identifier (inode number of subvolume root)
 * @read_ops: Number of read operations since last snapshot
 * @write_ops: Number of write operations since last snapshot
 * @read_bytes: Total bytes read since last snapshot
 * @write_bytes: Total bytes written since last snapshot
 * @read_latency_us: Sum of read latencies in microseconds (for avg calculation)
 * @write_latency_us: Sum of write latencies in microseconds (for avg calculation)
 */
struct ceph_subvol_metric_snapshot {
	u64 subvolume_id;
	u64 read_ops;
	u64 write_ops;
	u64 read_bytes;
	u64 write_bytes;
	u64 read_latency_us;
	u64 write_latency_us;
};

/**
 * struct ceph_subvolume_metrics_tracker - Tracks per-subvolume I/O metrics
 * @lock: Protects @tree and @nr_entries during concurrent access
 * @tree: Red-black tree of per-subvolume entries, keyed by subvolume_id
 * @nr_entries: Number of entries currently in @tree
 * @enabled: Whether collection is enabled (requires MDS feature support)
 * @snapshot_attempts: Debug counter: total ceph_subvolume_metrics_snapshot() calls
 * @snapshot_empty: Debug counter: snapshots that found no data to report
 * @snapshot_failures: Debug counter: snapshots that failed to allocate memory
 * @record_calls: Debug counter: total ceph_subvolume_metrics_record() calls
 * @record_disabled: Debug counter: record calls skipped because disabled
 * @record_no_subvol: Debug counter: record calls skipped (no subvolume_id)
 * @total_read_ops: Cumulative read ops across all snapshots (never reset)
 * @total_read_bytes: Cumulative bytes read across all snapshots (never reset)
 * @total_write_ops: Cumulative write ops across all snapshots (never reset)
 * @total_write_bytes: Cumulative bytes written across all snapshots (never reset)
 */
struct ceph_subvolume_metrics_tracker {
	spinlock_t lock;
	struct rb_root_cached tree;
	u32 nr_entries;
	bool enabled;
	atomic64_t snapshot_attempts;
	atomic64_t snapshot_empty;
	atomic64_t snapshot_failures;
	atomic64_t record_calls;
	atomic64_t record_disabled;
	atomic64_t record_no_subvol;
	atomic64_t total_read_ops;
	atomic64_t total_read_bytes;
	atomic64_t total_write_ops;
	atomic64_t total_write_bytes;
};

void ceph_subvolume_metrics_init(struct ceph_subvolume_metrics_tracker *tracker);
void ceph_subvolume_metrics_destroy(struct ceph_subvolume_metrics_tracker *tracker);
void ceph_subvolume_metrics_enable(struct ceph_subvolume_metrics_tracker *tracker,
				   bool enable);
void ceph_subvolume_metrics_record(struct ceph_subvolume_metrics_tracker *tracker,
				   u64 subvol_id, bool is_write,
				   size_t size, u64 latency_us);
int ceph_subvolume_metrics_snapshot(struct ceph_subvolume_metrics_tracker *tracker,
				    struct ceph_subvol_metric_snapshot **out,
				    u32 *nr, bool consume);
void ceph_subvolume_metrics_free_snapshot(struct ceph_subvol_metric_snapshot *snapshot);
void ceph_subvolume_metrics_dump(struct ceph_subvolume_metrics_tracker *tracker,
				 struct seq_file *s);

void ceph_subvolume_metrics_record_io(struct ceph_mds_client *mdsc,
				      struct ceph_inode_info *ci,
				      bool is_write, size_t bytes,
				      ktime_t start, ktime_t end);

static inline bool ceph_subvolume_metrics_enabled(
		const struct ceph_subvolume_metrics_tracker *tracker)
{
	return READ_ONCE(tracker->enabled);
}

int __init ceph_subvolume_metrics_cache_init(void);
void ceph_subvolume_metrics_cache_destroy(void);

#endif /* _FS_CEPH_SUBVOLUME_METRICS_H */
