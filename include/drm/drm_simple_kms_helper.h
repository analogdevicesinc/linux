/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 Noralf Trønnes
 */

/*
 * Simple KMS helpers are deprected in favor of regular atomic helpers. Do not
 * use the min new code.
 */

#ifndef __LINUX_DRM_SIMPLE_KMS_HELPER_H
#define __LINUX_DRM_SIMPLE_KMS_HELPER_H

#include <linux/types.h>

struct drm_device;

void *__drmm_simple_encoder_alloc(struct drm_device *dev, size_t size,
				  size_t offset, int encoder_type);

#define drmm_simple_encoder_alloc(dev, type, member, encoder_type) \
	((type *)__drmm_simple_encoder_alloc(dev, sizeof(type), \
					     offsetof(type, member), \
					     encoder_type))

#endif /* __LINUX_DRM_SIMPLE_KMS_HELPER_H */
