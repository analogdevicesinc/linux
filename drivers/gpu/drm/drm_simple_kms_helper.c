// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Noralf Trønnes
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

void *__drmm_simple_encoder_alloc(struct drm_device *dev, size_t size,
				  size_t offset, int encoder_type)
{
	return __drmm_encoder_alloc(dev, size, offset, NULL, encoder_type,
				    NULL);
}
EXPORT_SYMBOL(__drmm_simple_encoder_alloc);

MODULE_DESCRIPTION("Helpers for drivers for simple display hardware");
MODULE_LICENSE("GPL");
