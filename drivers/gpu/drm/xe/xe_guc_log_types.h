/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef _XE_GUC_LOG_TYPES_H_
#define _XE_GUC_LOG_TYPES_H_

#include <linux/types.h>
#include "abi/guc_log_abi.h"

#include "xe_uc_fw_types.h"

struct xe_bo;

/**
 * struct xe_guc_dump_cache - cached compressed copy of a dump blob
 *
 * A snapshot is printed more than once: __xe_devcoredump_read() walks it first
 * with a NULL buffer to measure the output size, then again - once per chunk -
 * to fill the user buffer. Compressing on every pass would be both wasteful and
 * unsafe, as any difference between the measured size and the emitted bytes
 * corrupts the dump. So compress at most once and replay the result.
 */
struct xe_guc_dump_cache {
	/** @data: compressed bytes, allocation padded up to a u32 boundary */
	char *data;
	/** @size: number of valid compressed bytes in @data */
	size_t size;
	/** @attempted: compression has been attempted; @data is final either way */
	bool attempted;
};

/**
 * struct xe_guc_log_snapshot:
 * Capture of the GuC log plus various state useful for decoding the log
 */
struct xe_guc_log_snapshot {
	/** @size: Size in bytes of the @copy allocation */
	size_t size;
	/** @copy: Host memory copy of the log buffer for later dumping, split into chunks */
	void **copy;
	/** @num_chunks: Number of chunks within @copy */
	int num_chunks;
	/** @ktime: Kernel time the snapshot was taken */
	u64 ktime;
	/** @stamp: GuC timestamp at which the snapshot was taken */
	u64 stamp;
	/** @level: GuC log verbosity level */
	u32 level;
	/** @ver_found: GuC firmware version */
	struct xe_uc_fw_version ver_found;
	/** @ver_want: GuC firmware version that driver expected */
	struct xe_uc_fw_version ver_want;
	/** @path: Path of GuC firmware blob */
	const char *path;
	/** @dump_cache: compressed copy of @copy, built on first print */
	struct xe_guc_dump_cache dump_cache;
};

/**
 * struct xe_guc_log - GuC log
 */
struct xe_guc_log {
	/** @level: GuC log level */
	u32 level;
	/** @bo: Xe BO for GuC log */
	struct xe_bo *bo;
	/** @stats: logging related stats */
	struct {
		u32 sampled_overflow;
		u32 overflow;
		u32 flush;
	} stats[GUC_LOG_BUFFER_TYPE_MAX];
};

#endif
