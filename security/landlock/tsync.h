/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Landlock LSM - Cross-thread ruleset enforcement
 *
 * Copyright 2025 Google LLC
 */

#ifndef _SECURITY_LANDLOCK_TSYNC_H
#define _SECURITY_LANDLOCK_TSYNC_H

#include "cred.h"

int landlock_restrict_sibling_threads(const struct cred *old_cred,
				      const struct cred *new_cred);

#endif /* _SECURITY_LANDLOCK_TSYNC_H */
