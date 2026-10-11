/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OSF_CORE_H
#define _OSF_CORE_H

#include <linux/types.h>

#include "osf_protocol.h"

struct device;

struct osf_status_cache {
	u32 uptime_s;
	u32 status_flags;
	u32 error_flags;
	u32 dropped_frames;
	u64 sequence;
	bool valid;
};

struct osf_device {
	struct device *dev;
	struct osf_status_cache status_cache;
	u64 last_sequence;
};

void osf_core_init(struct osf_device *osf, struct device *dev);
int osf_core_receive_frame(struct osf_device *osf, const u8 *buf, size_t len);

#endif
