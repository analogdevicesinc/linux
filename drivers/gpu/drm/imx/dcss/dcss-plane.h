#ifndef __DCSS_PLANE_H__
#define __DCSS_PLANE_H__

#include <drm/drm_crtc.h>

struct dcss_plane {
	struct drm_plane base;
	struct dcss_soc *dcss;

	int alpha_val;
	struct drm_property *alpha_prop;

	int use_global_val;
	struct drm_property *use_global_prop;

	int ch_num;
};

struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   struct dcss_soc *dcss,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos);

#endif /* __DCSS_PLANE_H__ */
