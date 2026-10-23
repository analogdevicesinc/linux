/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_SYSCTRL_DEBUGFS_H_
#define _XE_SYSCTRL_DEBUGFS_H_

struct dentry;
struct xe_sysctrl;

void xe_sysctrl_debugfs_register(struct xe_sysctrl *sc, struct dentry *parent);

#endif
