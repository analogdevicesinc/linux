/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _VKMS_DRV_H_
#define _VKMS_DRV_H_

#include <linux/hrtimer.h>

#include <drm/drm.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_writeback.h>

#define DEFAULT_DEVICE_NAME "vkms"

#define XRES_MIN    10
#define YRES_MIN    10

#define XRES_DEF  1024
#define YRES_DEF   768

#define XRES_MAX  8192
#define YRES_MAX  8192

#define NUM_OVERLAY_PLANES 8

#define VKMS_LUT_SIZE 256

/**
 * struct vkms_frame_info - Structure to store the state of a frame
 *
 * @fb: backing drm framebuffer
 * @src: source rectangle of this frame in the source framebuffer, stored in 16.16 fixed-point form
 * @dst: destination rectangle in the crtc buffer, stored in whole pixel units
 * @map: see @drm_shadow_plane_state.data
 * @rotation: rotation applied to the source.
 *
 * @src and @dst should have the same size modulo the rotation.
 */
struct vkms_frame_info {
	struct drm_framebuffer *fb;
	struct drm_rect src, dst;
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	unsigned int rotation;
};

/**
 * struct pixel_argb_u16 - Internal representation of a pixel color.
 * @a: Alpha component value, stored in 16 bits, without padding, using
 *     machine endianness
 * @r: Red component value, stored in 16 bits, without padding, using
 *     machine endianness
 * @g: Green component value, stored in 16 bits, without padding, using
 *     machine endianness
 * @b: Blue component value, stored in 16 bits, without padding, using
 *     machine endianness
 *
 * The goal of this structure is to keep enough precision to ensure
 * correct composition results in VKMS and simplifying color
 * manipulation by splitting each component into its own field.
 * Caution: the byte ordering of this structure is machine-dependent,
 * you can't cast it directly to AR48 or xR48.
 */
struct pixel_argb_u16 {
	u16 a, r, g, b;
};

struct line_buffer {
	size_t n_pixels;
	struct pixel_argb_u16 *pixels;
};

/**
 * typedef pixel_write_t - These functions are used to read a pixel from a
 * &struct pixel_argb_u16, convert it in a specific format and write it in the @out_pixel
 * buffer.
 *
 * @out_pixel: destination address to write the pixel
 * @in_pixel: pixel to write
 */
typedef void (*pixel_write_t)(u8 *out_pixel, const struct pixel_argb_u16 *in_pixel);

struct vkms_writeback_job {
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	struct vkms_frame_info wb_frame_info;
	pixel_write_t pixel_write;
};

/**
 * enum pixel_read_direction - Enum used internally by VKMS to represent a reading direction in a
 * plane.
 */
enum pixel_read_direction {
	READ_BOTTOM_TO_TOP,
	READ_TOP_TO_BOTTOM,
	READ_RIGHT_TO_LEFT,
	READ_LEFT_TO_RIGHT
};

struct vkms_plane_state;

/**
 * typedef pixel_read_line_t - These functions are used to read a pixel line in the source frame,
 * convert it to `struct pixel_argb_u16` and write it to @out_pixel.
 *
 * @plane: plane used as source for the pixel value
 * @x_start: X (width) coordinate of the first pixel to copy. The caller must ensure that x_start
 * is non-negative and smaller than @plane->frame_info->fb->width.
 * @y_start: Y (height) coordinate of the first pixel to copy. The caller must ensure that y_start
 * is non-negative and smaller than @plane->frame_info->fb->height.
 * @direction: direction to use for the copy, starting at @x_start/@y_start
 * @count: number of pixels to copy
 * @out_pixel: pointer where to write the pixel values. They will be written from @out_pixel[0]
 * (included) to @out_pixel[@count] (excluded). The caller must ensure that out_pixel have a
 * length of at least @count.
 */
typedef void (*pixel_read_line_t)(const struct vkms_plane_state *plane, int x_start,
				  int y_start, enum pixel_read_direction direction, int count,
				  struct pixel_argb_u16 out_pixel[]);

/**
 * struct conversion_matrix - Matrix to use for a specific encoding and range
 *
 * @matrix: Conversion matrix from yuv to rgb. The matrix is stored in a row-major manner and is
 * used to compute rgb values from yuv values:
 *     [[r],[g],[b]] = @matrix * [[y],[u],[v]]
 *   OR for yvu formats:
 *     [[r],[g],[b]] = @matrix * [[y],[v],[u]]
 *  The values of the matrix are signed fixed-point values with 32 bits fractional part.
 * @y_offset: Offset to apply on the y value.
 */
struct conversion_matrix {
	s64 matrix[3][3];
	int y_offset;
};

/**
 * struct vkms_plane_state - Driver specific plane state
 * @base: base plane state
 * @frame_info: data required for composing computation
 * @pixel_read_line: function to read a pixel line in this plane. The creator of a
 *		     struct vkms_plane_state must ensure that this pointer is valid
 * @conversion_matrix: matrix used for yuv formats to convert to rgb
 */
struct vkms_plane_state {
	struct drm_shadow_plane_state base;
	struct vkms_frame_info *frame_info;
	pixel_read_line_t pixel_read_line;
	struct conversion_matrix conversion_matrix;
};

struct vkms_plane {
	struct drm_plane base;
};

struct vkms_color_lut {
	struct drm_color_lut *base;
	size_t lut_length;
	s64 channel_value2index_ratio;
};

/**
 * struct vkms_crtc_state - Driver specific CRTC state
 *
 * @base: base CRTC state
 * @composer_work: work struct to compose and add CRC entries
 *
 * @num_active_planes: Number of active planes
 * @active_planes: List containing all the active planes (counted by
 *		   @num_active_planes). They should be stored in z-order.
 * @active_writeback: Current active writeback job
 * @gamma_lut: Look up table for gamma used in this CRTC
 * @crc_pending: Protected by @vkms_output.composer_lock, true when the frame CRC is not computed
 *		 yet. Used by vblank to detect if the composer is too slow.
 * @wb_pending: Protected by @vkms_output.composer_lock, true when a writeback frame is requested.
 * @frame_start: Protected by @vkms_output.composer_lock, saves the frame number before the start
 *		 of the composition process.
 * @frame_end: Protected by @vkms_output.composer_lock, saves the last requested frame number.
 *	       This is used to generate enough CRC entries when the composition worker is too slow.
 */
struct vkms_crtc_state {
	struct drm_crtc_state base;
	struct work_struct composer_work;

	int num_active_planes;
	struct vkms_plane_state **active_planes;
	struct vkms_writeback_job *active_writeback;
	struct vkms_color_lut gamma_lut;

	bool crc_pending;
	bool wb_pending;
	u64 frame_start;
	u64 frame_end;
};

/**
 * struct vkms_output - Internal representation of all output components in VKMS
 *
 * @crtc: Base CRTC in DRM
 * @encoder: DRM encoder used for this output
 * @connector: DRM connector used for this output
 * @wb_connecter: DRM writeback connector used for this output
 * @vblank_hrtimer: Timer used to trigger the vblank
 * @period_ns: vblank period, in nanoseconds, used to configure @vblank_hrtimer and to compute
 *	       vblank timestamps
 * @composer_workq: Ordered workqueue for @composer_state.composer_work.
 * @lock: Lock used to protect concurrent access to the composer
 * @composer_enabled: Protected by @lock, true when the VKMS composer is active (crc needed or
 *		      writeback)
 * @composer_state: Protected by @lock, current state of this VKMS output
 * @composer_lock: Lock used internally to protect @composer_state members
 */
struct vkms_output {
	struct drm_crtc crtc;
	struct drm_writeback_connector wb_connector;
	struct drm_encoder wb_encoder;
	struct workqueue_struct *composer_workq;
	spinlock_t lock;

	bool composer_enabled;
	struct vkms_crtc_state *composer_state;

	spinlock_t composer_lock;
};

struct vkms_config;

/**
 * struct vkms_device - Description of a VKMS device
 *
 * @drm - Base device in DRM
 * @faux_dev - Associated faux device
 * @output - Configuration and sub-components of the VKMS device
 * @config: Configuration used in this VKMS device
 */
struct vkms_device {
	struct drm_device drm;
	struct faux_device *faux_dev;
	const struct vkms_config *config;
};

/*
 * The following helpers are used to convert a member of a struct into its parent.
 */

#define drm_crtc_to_vkms_output(target) \
	container_of(target, struct vkms_output, crtc)

#define drm_device_to_vkms_device(target) \
	container_of(target, struct vkms_device, drm)

#define to_vkms_crtc_state(target)\
	container_of(target, struct vkms_crtc_state, base)

#define to_vkms_plane_state(target)\
	container_of(target, struct vkms_plane_state, base.base)

/**
 * vkms_crtc_init() - Initialize a CRTC for VKMS
 * @dev: DRM device associated with the VKMS buffer
 * @crtc: uninitialized CRTC device
 * @primary: primary plane to attach to the CRTC
 * @cursor: plane to attach to the CRTC
 */
struct vkms_output *vkms_crtc_init(struct drm_device *dev,
				   struct drm_plane *primary,
				   struct drm_plane *cursor);

/**
 * vkms_output_init() - Initialize all sub-components needed for a VKMS device.
 *
 * @vkmsdev: VKMS device to initialize
 */
int vkms_output_init(struct vkms_device *vkmsdev);

/**
 * vkms_plane_init() - Initialize a plane
 *
 * @vkmsdev: VKMS device containing the plane
 * @type: type of plane to initialize
 */
struct vkms_plane *vkms_plane_init(struct vkms_device *vkmsdev,
				   enum drm_plane_type type);

/* CRC Support */
const char *const *vkms_get_crc_sources(struct drm_crtc *crtc,
					size_t *count);
int vkms_set_crc_source(struct drm_crtc *crtc, const char *src_name);
int vkms_verify_crc_source(struct drm_crtc *crtc, const char *source_name,
			   size_t *values_cnt);

/* Composer Support */
void vkms_composer_worker(struct work_struct *work);
void vkms_set_composer(struct vkms_output *out, bool enabled);
void vkms_writeback_row(struct vkms_writeback_job *wb, const struct line_buffer *src_buffer, int y);

/* Writeback */
int vkms_enable_writeback_connector(struct vkms_device *vkmsdev, struct vkms_output *vkms_out);

#endif /* _VKMS_DRV_H_ */
