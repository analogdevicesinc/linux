#ifndef _ANALOG_DRM_FBDEV_H_
#define _ANALOG_DRM_FBDEV_H_

int analog_drm_fbdev_init(struct drm_device *dev);
void analog_drm_fbdev_fini(struct drm_device *dev);
void analog_drm_fbdev_restore_mode(struct drm_device *dev);

void analog_drm_mode_config_init(struct drm_device *dev);

struct analog_drm_gem_obj *analog_drm_fb_get_gem_obj(struct drm_framebuffer *fb);

#endif
