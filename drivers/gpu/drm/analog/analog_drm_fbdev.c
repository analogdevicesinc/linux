#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/module.h>

#include "analog_drm_drv.h"
#include "analog_drm_gem.h"

#define MAX_CONNECTOR		1
#define PREFERRED_BPP		32

#define to_analog_fbdev(x)	container_of(x, struct analog_drm_fbdev,\
				fb_helper)

#define to_analog_fb(x)	container_of(x, struct analog_drm_fb,\
				fb)

struct analog_drm_fb {
	struct drm_framebuffer		fb;
	struct analog_drm_gem_obj	*obj;
};

struct analog_drm_fbdev {
	struct drm_fb_helper fb_helper;
	struct analog_drm_fb *fb;
};

static void analog_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct analog_drm_fb *afb = to_analog_fb(fb);

	if (afb->obj)
		drm_gem_object_unreference_unlocked(&afb->obj->base);

	drm_framebuffer_cleanup(fb);
	kfree(afb);
}

static int analog_drm_fb_create_handle(struct drm_framebuffer *fb,
    struct drm_file *file_priv, unsigned int *handle)
{
	struct analog_drm_fb *afb = to_analog_fb(fb);

	return drm_gem_handle_create(file_priv,
			&afb->obj->base, handle);
}

static struct drm_framebuffer_funcs analog_drm_fb_funcs = {
	.destroy	= analog_drm_fb_destroy,
	.create_handle	= analog_drm_fb_create_handle,
};

static struct analog_drm_fb *analog_drm_fb_alloc(struct drm_device *dev,
	struct drm_mode_fb_cmd2 *mode_cmd, struct analog_drm_gem_obj *obj)
{
	struct analog_drm_fb *afb;
	int ret;

	afb = kzalloc(sizeof(*afb), GFP_KERNEL);
	if (!afb)
		return ERR_PTR(-ENOMEM);

	afb->obj = obj;
	ret = drm_framebuffer_init(dev, &afb->fb, &analog_drm_fb_funcs);
	if (ret) {
		DRM_ERROR("failed to initialize framebuffer.\n");
		return ERR_PTR(ret);
	}

	drm_helper_mode_fill_fb_struct(&afb->fb, mode_cmd);

	return afb;
}

static struct drm_framebuffer *analog_drm_fb_create(struct drm_device *dev,
	struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct analog_drm_fb *afb;
	struct drm_gem_object *obj;

	obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		return ERR_PTR(-ENXIO);
	}

	afb = analog_drm_fb_alloc(dev, mode_cmd, to_analog_gem_obj(obj));
	if (IS_ERR(afb))
		return ERR_CAST(afb);

	return &afb->fb;
}

struct analog_drm_gem_obj *analog_drm_fb_get_gem_obj(struct drm_framebuffer *fb)
{
	struct analog_drm_fb *afb = to_analog_fb(fb);

	return afb->obj;
}

static void analog_drm_output_poll_changed(struct drm_device *dev)
{
	struct analog_drm_private *private = dev->dev_private;

	if (private->fbdev)
		drm_fb_helper_hotplug_event(&private->fbdev->fb_helper);
}

static struct drm_mode_config_funcs analog_drm_mode_config_funcs = {
	.fb_create = analog_drm_fb_create,
	.output_poll_changed = analog_drm_output_poll_changed,
};

void analog_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &analog_drm_mode_config_funcs;
}

static struct fb_ops analog_drm_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int analog_drm_fbdev_create(struct drm_fb_helper *helper,
				    struct drm_fb_helper_surface_size *sizes)
{
	struct analog_drm_fbdev *afbdev = to_analog_fbdev(helper);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct analog_drm_gem_obj *obj;
	struct drm_framebuffer *fb;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	DRM_DEBUG_KMS("surface width(%d), height(%d) and bpp(%d\n",
			sizes->surface_width, sizes->surface_height,
			sizes->surface_bpp);

	sizes->surface_bpp = 32;
	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * (sizes->surface_bpp >> 3);
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;
	obj = analog_drm_gem_alloc(dev, size);
	if (!obj)
		return -ENOMEM;

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		DRM_ERROR("failed to allocate fb info.\n");
		ret = -ENOMEM;
		goto out;
	}

	afbdev->fb = analog_drm_fb_alloc(dev, &mode_cmd, obj);
	if (IS_ERR(afbdev->fb)) {
		DRM_ERROR("failed to create drm framebuffer.\n");
		ret = PTR_ERR(afbdev->fb);
		goto out;
	}

	fb = &afbdev->fb->fb;
	helper->fb = fb;
	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &analog_drm_fb_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		DRM_ERROR("failed to allocate cmap.\n");
		goto out;
	}

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb->width, fb->height);

	offset = fbi->var.xoffset * (fb->bits_per_pixel >> 3);
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = (resource_size_t)obj->dma_addr;
	fbi->screen_base = obj->addr + offset;
	fbi->fix.smem_start = (unsigned long)(obj->dma_addr + offset);
	fbi->screen_size = size;
	fbi->fix.smem_len = size;

	return 0;

/* TODO: gem obj, free fb */
out:
	return ret;
}

static int analog_drm_fbdev_probe(struct drm_fb_helper *helper,
				   struct drm_fb_helper_surface_size *sizes)
{
	int ret = 0;

	if (!helper->fb) {
		ret = analog_drm_fbdev_create(helper, sizes);
		if (ret < 0) {
			DRM_ERROR("failed to create fbdev.\n");
			return ret;
		}

		/*
		 * fb_helper expects a value more than 1 if succeed
		 * because register_framebuffer() should be called.
		 */
		ret = 1;
	}


	printk("%s:%s[%d]\n", __FILE__, __func__, __LINE__);
	return ret;
}

static struct drm_fb_helper_funcs analog_drm_fb_helper_funcs = {
	.fb_probe =	analog_drm_fbdev_probe,
};

int analog_drm_fbdev_init(struct drm_device *dev)
{
	struct analog_drm_fbdev *afbdev;
	struct analog_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	afbdev = kzalloc(sizeof(*afbdev), GFP_KERNEL);
	if (!afbdev) {
		DRM_ERROR("failed to allocate drm fbdev.\n");
		return -ENOMEM;
	}

	afbdev->fb_helper.funcs = &analog_drm_fb_helper_funcs;
	private->fbdev = afbdev;
	helper = &afbdev->fb_helper;

	ret = drm_fb_helper_init(dev, helper, 1, 1);
	if (ret < 0) {
		DRM_ERROR("failed to initialize drm fb helper.\n");
		goto err_init;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		DRM_ERROR("failed to register drm_fb_helper_connector.\n");
		goto err_setup;

	}

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		DRM_ERROR("failed to set up hw configuration.\n");
		goto err_setup;
	}

	return 0;

err_setup:
	drm_fb_helper_fini(helper);

err_init:
	private->fbdev = NULL;
	kfree(afbdev);

	return ret;
}

void analog_drm_fbdev_fini(struct drm_device *dev)
{
	struct analog_drm_private *private = dev->dev_private;
	struct analog_drm_fbdev *afbdev;

	if (!private || !private->fbdev)
		return;

	afbdev = private->fbdev;

	private->fbdev = NULL;

	if (afbdev->fb_helper.fbdev) {
		struct fb_info *info;
		int ret;

		info = afbdev->fb_helper.fbdev;
		ret = unregister_framebuffer(info);
		if (ret < 0)
			DRM_DEBUG_KMS("failed unregister_framebuffer()\n");

		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);

		framebuffer_release(info);
	}

	drm_fb_helper_fini(&afbdev->fb_helper);
	kfree(afbdev);
}

void analog_drm_fbdev_restore_mode(struct drm_device *dev)
{
	struct analog_drm_private *private = dev->dev_private;

	if (!private || !private->fbdev)
		return;

	drm_fb_helper_restore_fbdev_mode(&private->fbdev->fb_helper);
}
