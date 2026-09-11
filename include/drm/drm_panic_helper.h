/* SPDX-License-Identifier: GPL-2.0 or MIT */

#ifndef __DRM_DRM_PANIC_HELPER_H_
#define __DRM_DRM_PANIC_HELPER_H_

#include <linux/types.h>

enum drm_panic_type;

struct drm_plane;

/* drm_panic_helper.c */
int drm_plane_helper_display_panic_screen(struct drm_plane *plane,
					  const char *description,
					  enum drm_panic_type panic_type,
					  u32 fg_color, u32 bg_color,
					  unsigned int qr_version);

#if IS_ENABLED(CONFIG_DRM_PANIC)
#define DRM_PANIC_PLANE_FUNCS \
	.display_panic_screen = drm_plane_helper_display_panic_screen
#else
#define DRM_PANIC_PLANE_FUNCS \
	.display_panic_screen = NULL
#endif

#endif
