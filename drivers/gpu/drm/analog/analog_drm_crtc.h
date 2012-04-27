#ifndef _ANALOG_DRM_CRTC_H_
#define _ANALOG_DRM_CRTC_H_

struct drm_device;
struct drm_crtc;

struct drm_crtc* analog_drm_crtc_create(struct drm_device *dev);

#endif
