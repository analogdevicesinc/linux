/* SPDX-License-Identifier: GPL-2.0 */
#ifndef LINUX_DEVICE_ID_CPU_FEATURE_H
#define LINUX_DEVICE_ID_CPU_FEATURE_H

#ifdef __KERNEL__
#include <linux/types.h>
#endif

/*
 * Generic table type for matching CPU features.
 * @feature:	the bit number of the feature (0 - 65535)
 */

struct cpu_feature {
	__u16	feature;
};

#endif /* ifndef LINUX_DEVICE_ID_CPU_FEATURE_H */
