/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023 Intel Corporation
 */

#ifndef _XE_DEVCOREDUMP_TYPES_H_
#define _XE_DEVCOREDUMP_TYPES_H_

#include <linux/ktime.h>
#include <linux/mutex.h>

#include "regs/xe_reg_defs.h"
#include "xe_hw_engine_types.h"

struct xe_device;
struct xe_gt;

/**
 * struct xe_reg_desc - Register descriptor struct to make reg list
 *
 * This struct is used as descriptor containing reg address and name
 * of the register to dump its value during coredump for debug purpose.
 */
struct xe_reg_desc {
	union {
		/** @reg: The register */
		struct xe_reg reg;
		/** @mcr_reg: The mcr register */
		struct xe_reg_mcr mcr_reg;
	};
	/** @name: Name of the register */
	const char *name;
};

/**
 * struct xe_reg_desc_list - Struct for register descriptor list
 *
 * This struct is used to reference the corresponding reglist of
 * various component
 */
struct xe_reg_desc_list {
	/** @regs: Pointer to an array of register descriptors. */
	const struct xe_reg_desc *regs;
	/** @num_regs: number of registers in the descriptor list*/
	u32 num_regs;
};

/**
 * struct xe_dbg_reg_snapshot - Register snapshot
 *
 * This struct contains the register values captured at the time of the crash.
 * It is used to store the register values in a human-readable format for
 * debugging purposes.
 */
struct xe_dbg_reg_snapshot {
	/** @desc: register descriptor list. */
	const struct xe_reg_desc_list *desc;
	/** @num_regs: Number of registers needed for capture. */
	u32 num_regs;
	/** @values: register values*/
	u32 values[];
};

/**
 * struct xe_devcoredump_snapshot - Crash snapshot
 *
 * This struct contains all the useful information quickly captured at the time
 * of the crash. So, any subsequent reads of the coredump points to a data that
 * shows the state of the GPU of when the issue has happened.
 */
struct xe_devcoredump_snapshot {
	/** @snapshot_time:  Time of this capture. */
	ktime_t snapshot_time;
	/** @boot_time:  Relative boot time so the uptime can be calculated. */
	ktime_t boot_time;
	/** @process_name: Name of process that triggered this gpu hang */
	char process_name[TASK_COMM_LEN];
	/** @pid: Process id of process that triggered this gpu hang */
	pid_t pid;
	/** @reason: The reason the coredump was triggered */
	char *reason;

	/** @gt: Affected GT, used by forcewake for delayed capture */
	struct xe_gt *gt;

	/** @gt_fuse: Fuse snapshot */
	struct xe_dbg_reg_snapshot *gt_fuse;
	/** @media_fuse: Media Fuse snapshot */
	struct xe_dbg_reg_snapshot *media_fuse;
	/** @gt_rpm: GT RPM snapshot */
	struct xe_dbg_reg_snapshot *gt_rpm;
	/** @media_gt_rpm: Media GT RPM snapshot */
	struct xe_dbg_reg_snapshot *media_gt_rpm;

	/** @gt_guc_reg: gt guc reg snapshot */
	struct xe_dbg_reg_snapshot *gt_guc_reg;
	/** @media_gt_guc_reg: media gt guc reg snapshot */
	struct xe_dbg_reg_snapshot *media_gt_guc_reg;

	/** @work: Workqueue for deferred capture outside of signaling context */
	struct work_struct work;

	/** @guc: GuC snapshots */
	struct {
		/** @guc.ct: GuC CT snapshot */
		struct xe_guc_ct_snapshot *ct;
		/** @guc.log: GuC log snapshot */
		struct xe_guc_log_snapshot *log;
	} guc;

	/** @gam_pf_report: GAM PF report snapshot */
	struct xe_dbg_reg_snapshot *gam_pf_report;
	/** @ns: Node status snapshot */
	struct xe_dbg_reg_snapshot *ns;

	/** @ge: GuC Submission Engine snapshot */
	struct xe_guc_submit_exec_queue_snapshot *ge;

	/** @hwe: HW Engine snapshot array */
	struct xe_hw_engine_snapshot *hwe[XE_NUM_HW_ENGINES];
	/** @job: Snapshot of job state */
	struct xe_sched_job_snapshot *job;
	/**
	 * @matched_node: The matched capture node for timedout job
	 * this single-node tracker works because devcoredump will always only
	 * produce one hw-engine capture per devcoredump event
	 */
	struct __guc_capture_parsed_output *matched_node;
	/** @vm: Snapshot of VM state */
	struct xe_vm_snapshot *vm;

	/** @read: devcoredump in human readable format */
	struct {
		/** @read.size: size of devcoredump in human readable format */
		ssize_t size;
		/** @read.chunk_position: position of devcoredump chunk */
		ssize_t chunk_position;
		/** @read.buffer: buffer of devcoredump in human readable format */
		char *buffer;
	} read;
};

/**
 * struct xe_devcoredump - Xe devcoredump main structure
 *
 * This struct represents the live and active dev_coredump node.
 * It is created/populated at the time of a crash/error. Then it
 * is read later when user access the device coredump data file
 * for reading the information.
 */
struct xe_devcoredump {
	/** @lock: protects access to entire structure */
	struct mutex lock;
	/** @captured: The snapshot of the first hang has already been taken */
	bool captured;
	/** @snapshot: Snapshot is captured at time of the first crash */
	struct xe_devcoredump_snapshot snapshot;
};

#endif
