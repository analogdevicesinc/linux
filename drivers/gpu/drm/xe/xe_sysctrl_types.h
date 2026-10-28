/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_SYSCTRL_TYPES_H_
#define _XE_SYSCTRL_TYPES_H_

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue_types.h>

#include "xe_sysctrl_mailbox_types.h"

struct xe_mmio;
struct dentry;

/**
 * struct xe_sysctrl_debugfs_entry - Debugfs entry for a raw mailbox test command
 */
struct xe_sysctrl_debugfs_entry {
	/** @sc: Back pointer to parent sysctrl instance */
	struct xe_sysctrl *sc;

	/** @group: Command group ID */
	u8 group;

	/** @command: Command ID within group */
	u8 command;

	/** @lock: Protects @status, @response_len and @response_buf below */
	struct mutex lock;

	/** @response_buf: Response data buffer, sized to the maximum mailbox message */
	u8 response_buf[XE_SYSCTRL_MB_MAX_MESSAGE_SIZE];

	/** @response_len: Actual response length from firmware */
	size_t response_len;

	/** @status: Last command result */
	int status;

	/** @timeout_ms: Last mailbox response timeout used (0 for default) */
	unsigned int timeout_ms;
};

/**
 * struct xe_sysctrl - System Controller driver context
 *
 * This structure maintains the runtime state for System Controller
 * communication. All fields are initialized during xe_sysctrl_init()
 * and protected appropriately for concurrent access.
 */
struct xe_sysctrl {
	/** @mmio: MMIO region for system control registers */
	struct xe_mmio *mmio;

	/** @cmd_lock: Mutex protecting mailbox command operations */
	struct mutex cmd_lock;

	/** @work: Pending events worker */
	struct work_struct work;

	/** @event_lock: Mutex protecting pending events */
	struct mutex event_lock;

	/** @debugfs: Debugfs entries */
	struct {
		/** @debugfs.root: Root debugfs directory */
		struct dentry *root;

		/** @debugfs.loopback: Loopback test entry */
		struct xe_sysctrl_debugfs_entry loopback;

		/** @debugfs.ras_error_inject: RAS error injection test entry */
		struct xe_sysctrl_debugfs_entry ras_error_inject;

		/** @debugfs.mailbox: Generic, user-parameterized mailbox entry */
		struct xe_sysctrl_debugfs_entry mailbox;
	} debugfs;
};

#endif
