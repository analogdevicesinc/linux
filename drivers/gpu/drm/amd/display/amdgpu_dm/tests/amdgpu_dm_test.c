// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * KUnit tests for amdgpu_dm.c
 *
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include <kunit/test.h>
#include <linux/pci.h>
#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_plane.h>
#include <drm/drm_property.h>
#include <drm/drm_vblank.h>
#include <drm/drm_writeback.h>

#include "dc.h"
#include "dm_services_types.h"
#include "dmub/dmub_srv.h"
#include "inc/core_types.h"
#include "logger_types.h"
#include "amd_shared.h"
#include "amdgpu.h"
#include "amdgpu_mode.h"
#include "amdgpu_dm.h"
#include "amdgpu_dm_kunit_test_helpers.h"

/* Tests for simple DM callbacks */

/**
 * dm_test_wait_for_idle - Test placeholder wait-for-idle callback returns success
 * @test: The KUnit test context
 */
static void dm_test_wait_for_idle(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_wait_for_idle(NULL), 0);
}

/**
 * dm_test_soft_reset - Test placeholder soft-reset callback returns success
 * @test: The KUnit test context
 */
static void dm_test_soft_reset(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_soft_reset(NULL), 0);
}

/**
 * dm_test_set_clockgating_state - Test placeholder clockgating callback returns success
 * @test: The KUnit test context
 */
static void dm_test_set_clockgating_state(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_set_clockgating_state(NULL, AMD_CG_STATE_GATE), 0);
}

/**
 * dm_test_set_powergating_state - Test placeholder powergating callback returns success
 * @test: The KUnit test context
 */
static void dm_test_set_powergating_state(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_set_powergating_state(NULL, AMD_PG_STATE_GATE), 0);
}

/**
 * dm_test_bandwidth_update - Test placeholder bandwidth update is callable
 * @test: The KUnit test context
 */
static void dm_test_bandwidth_update(struct kunit *test)
{
	dm_bandwidth_update(NULL);
}

/**
 * dm_test_crtc_complete_writeback_no_connector - Test no writeback connector returns false
 * @test: The KUnit test context
 */
static void dm_test_crtc_complete_writeback_no_connector(struct kunit *test)
{
	struct amdgpu_crtc *acrtc;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	KUNIT_EXPECT_FALSE(test, amdgpu_dm_crtc_complete_writeback(acrtc));
}

/**
 * dm_test_crtc_complete_writeback_not_pending - Test non-pending writeback returns false
 * @test: The KUnit test context
 */
static void dm_test_crtc_complete_writeback_not_pending(struct kunit *test)
{
	struct amdgpu_crtc *acrtc;
	struct drm_writeback_connector *wb_conn;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);
	wb_conn = kunit_kzalloc(test, sizeof(*wb_conn), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, wb_conn);

	spin_lock_init(&wb_conn->job_lock);
	acrtc->wb_conn = wb_conn;
	acrtc->wb_pending = false;

	KUNIT_EXPECT_FALSE(test, amdgpu_dm_crtc_complete_writeback(acrtc));
}

/**
 * dm_test_vblank_get_counter_out_of_range - Test out-of-range CRTC returns zero
 * @test: The KUnit test context
 */
static void dm_test_vblank_get_counter_out_of_range(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);

	adev->mode_info.num_crtc = 1;

	KUNIT_EXPECT_EQ(test, dm_vblank_get_counter(adev, 1), 0U);
}

/**
 * dm_test_vblank_get_counter_no_stream - Test missing stream returns zero
 * @test: The KUnit test context
 */
static void dm_test_vblank_get_counter_no_stream(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct amdgpu_crtc *acrtc;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	adev->mode_info.num_crtc = 1;
	adev->mode_info.crtcs[0] = acrtc;

	KUNIT_EXPECT_EQ(test, dm_vblank_get_counter(adev, 0), 0U);
}

/**
 * dm_test_crtc_get_scanoutpos_invalid_crtc - Test invalid CRTC returns -EINVAL
 * @test: The KUnit test context
 */
static void dm_test_crtc_get_scanoutpos_invalid_crtc(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	u32 vbl = 0;
	u32 position = 0;

	adev->mode_info.num_crtc = 1;

	KUNIT_EXPECT_EQ(test, dm_crtc_get_scanoutpos(adev, -1, &vbl, &position),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dm_crtc_get_scanoutpos(adev, 1, &vbl, &position),
			-EINVAL);
}

/**
 * dm_test_crtc_get_scanoutpos_no_stream - Test missing stream returns zero
 * @test: The KUnit test context
 */
static void dm_test_crtc_get_scanoutpos_no_stream(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct amdgpu_crtc *acrtc;
	u32 vbl = 0;
	u32 position = 0;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	adev->mode_info.num_crtc = 1;
	adev->mode_info.crtcs[0] = acrtc;

	KUNIT_EXPECT_EQ(test, dm_crtc_get_scanoutpos(adev, 0, &vbl, &position), 0);
	KUNIT_EXPECT_EQ(test, vbl, 0U);
	KUNIT_EXPECT_EQ(test, position, 0U);
}

/**
 * dm_test_atomic_get_new_state_empty - Test empty atomic state has no DM state
 * @test: The KUnit test context
 */
static void dm_test_atomic_get_new_state_empty(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);
	state->dev = &adev->ddev;

	KUNIT_EXPECT_NULL(test, dm_atomic_get_new_state(state));
}

/**
 * dm_test_atomic_get_new_state_match - Test atomic state returns matching DM private state
 * @test: The KUnit test context
 */
static void dm_test_atomic_get_new_state_match(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct dm_atomic_state *dm_state;
	struct drm_atomic_commit *state;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);

	dm_state = kunit_kzalloc(test, sizeof(*dm_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_state);

	state->private_objs = kunit_kzalloc(test, sizeof(*state->private_objs),
					    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->private_objs);

	state->dev = &adev->ddev;
	state->num_private_objs = 1;
	state->private_objs[0].ptr = &adev->dm.atomic_obj;
	state->private_objs[0].new_state = &dm_state->base;

	KUNIT_EXPECT_PTR_EQ(test, dm_atomic_get_new_state(state), dm_state);
}

/**
 * dm_test_atomic_destroy_state_no_context - Test destroying DM atomic state without a DC context
 * @test: The KUnit test context
 */
static void dm_test_atomic_destroy_state_no_context(struct kunit *test)
{
	struct dm_atomic_state *dm_state;

	/*
	 * Use kzalloc(), not kunit_kzalloc(): dm_atomic_destroy_state() frees
	 * the state itself, so KUnit-managed memory would be double-freed.
	 */
	dm_state = kzalloc(sizeof(*dm_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_state);

	/* context == NULL: dc_state_release() is skipped and the state is freed. */
	dm_atomic_destroy_state(NULL, &dm_state->base);
}

/* Tests for dm_plane_layer_index_cmp() */

/**
 * dm_test_plane_layer_index_cmp_equal - Test Plane layer index cmp equal
 * @test: The KUnit test context
 */
static void dm_test_plane_layer_index_cmp_equal(struct kunit *test)
{
	struct dc_plane_state *plane_a;
	struct dc_plane_state *plane_b;
	struct dc_surface_update sa, sb;

	plane_a = kunit_kzalloc(test, sizeof(*plane_a), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_a);
	plane_b = kunit_kzalloc(test, sizeof(*plane_b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_b);

	plane_a->layer_index = 5;
	plane_b->layer_index = 5;
	sa.surface = plane_a;
	sb.surface = plane_b;

	KUNIT_EXPECT_EQ(test, dm_plane_layer_index_cmp(&sa, &sb), 0);
}

/**
 * dm_test_plane_layer_index_cmp_descending - Test Plane layer index cmp descending
 * @test: The KUnit test context
 */
static void dm_test_plane_layer_index_cmp_descending(struct kunit *test)
{
	struct dc_plane_state *plane_a;
	struct dc_plane_state *plane_b;
	struct dc_surface_update sa, sb;

	plane_a = kunit_kzalloc(test, sizeof(*plane_a), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_a);
	plane_b = kunit_kzalloc(test, sizeof(*plane_b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_b);

	plane_a->layer_index = 3;
	plane_b->layer_index = 7;
	sa.surface = plane_a;
	sb.surface = plane_b;

	/* b has higher index, so cmp(a,b) = b - a > 0 (b sorts first) */
	KUNIT_EXPECT_GT(test, dm_plane_layer_index_cmp(&sa, &sb), 0);
}

/**
 * dm_test_plane_layer_index_cmp_ascending - Test Plane layer index cmp ascending
 * @test: The KUnit test context
 */
static void dm_test_plane_layer_index_cmp_ascending(struct kunit *test)
{
	struct dc_plane_state *plane_a;
	struct dc_plane_state *plane_b;
	struct dc_surface_update sa, sb;

	plane_a = kunit_kzalloc(test, sizeof(*plane_a), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_a);
	plane_b = kunit_kzalloc(test, sizeof(*plane_b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, plane_b);

	plane_a->layer_index = 9;
	plane_b->layer_index = 2;
	sa.surface = plane_a;
	sb.surface = plane_b;

	/* a has higher index, so cmp(a,b) = b - a < 0 (a sorts first) */
	KUNIT_EXPECT_LT(test, dm_plane_layer_index_cmp(&sa, &sb), 0);
}

/* Tests for fill_plane_color_attributes() */

/**
 * dm_test_fill_color_attr_rgb_format - Test Fill color attr rgb format
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_rgb_format(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	/* RGB format: should return 0 and set SRGB regardless of encoding */
	plane_state.color_encoding = DRM_COLOR_YCBCR_BT709;
	plane_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_GRPH_ARGB8888,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space, (int)COLOR_SPACE_SRGB);
}

/**
 * dm_test_fill_color_attr_bt601_full - Test Fill color attr bt601 full
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt601_full(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT601;
	plane_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space, (int)COLOR_SPACE_YCBCR601);
}

/**
 * dm_test_fill_color_attr_bt601_limited - Test Fill color attr bt601 limited
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt601_limited(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT601;
	plane_state.color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space,
			(int)COLOR_SPACE_YCBCR601_LIMITED);
}

/**
 * dm_test_fill_color_attr_bt709_full - Test Fill color attr bt709 full
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt709_full(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT709;
	plane_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space, (int)COLOR_SPACE_YCBCR709);
}

/**
 * dm_test_fill_color_attr_bt709_limited - Test Fill color attr bt709 limited
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt709_limited(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT709;
	plane_state.color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space,
			(int)COLOR_SPACE_YCBCR709_LIMITED);
}

/**
 * dm_test_fill_color_attr_bt2020_full - Test Fill color attr bt2020 full
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt2020_full(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT2020;
	plane_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space,
			(int)COLOR_SPACE_2020_YCBCR_FULL);
}

/**
 * dm_test_fill_color_attr_bt2020_limited - Test Fill color attr bt2020 limited
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_bt2020_limited(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = DRM_COLOR_YCBCR_BT2020;
	plane_state.color_range = DRM_COLOR_YCBCR_LIMITED_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, (int)color_space,
			(int)COLOR_SPACE_2020_YCBCR_LIMITED);
}

/**
 * dm_test_fill_color_attr_invalid_encoding - Test Fill color attr invalid encoding
 * @test: The KUnit test context
 */
static void dm_test_fill_color_attr_invalid_encoding(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	enum dc_color_space color_space = COLOR_SPACE_UNKNOWN;
	int ret;

	plane_state.color_encoding = 99;
	plane_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ret = fill_plane_color_attributes(&plane_state,
					  SURFACE_PIXEL_FORMAT_VIDEO_420_YCbCr,
					  &color_space);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/* Tests for modereset_required() */

/**
 * dm_test_modereset_required_when_inactive_and_modeset - Test Modereset required when inactive and modeset
 * @test: The KUnit test context
 */
static void dm_test_modereset_required_when_inactive_and_modeset(struct kunit *test)
{
	struct drm_crtc_state crtc_state = { 0 };

	crtc_state.active = false;
	crtc_state.mode_changed = true;

	KUNIT_EXPECT_TRUE(test, modereset_required(&crtc_state));
}

/**
 * dm_test_modereset_not_required_when_active_and_modeset - Test Modereset not required when active and modeset
 * @test: The KUnit test context
 */
static void dm_test_modereset_not_required_when_active_and_modeset(struct kunit *test)
{
	struct drm_crtc_state crtc_state = { 0 };

	crtc_state.active = true;
	crtc_state.mode_changed = true;

	KUNIT_EXPECT_FALSE(test, modereset_required(&crtc_state));
}

/**
 * dm_test_modereset_not_required_when_inactive_without_modeset - Test Modereset not required when inactive without modeset
 * @test: The KUnit test context
 */
static void dm_test_modereset_not_required_when_inactive_without_modeset(struct kunit *test)
{
	struct drm_crtc_state crtc_state = { 0 };

	crtc_state.active = false;
	crtc_state.mode_changed = false;

	KUNIT_EXPECT_FALSE(test, modereset_required(&crtc_state));
}

/* Tests for is_scaling_state_different() */

/**
 * dm_test_scaling_state_same - Test identical scaling states compare equal
 * @test: The KUnit test context
 */
static void dm_test_scaling_state_same(struct kunit *test)
{
	struct dm_connector_state *a;
	struct dm_connector_state *b;

	a = kunit_kzalloc(test, sizeof(*a), GFP_KERNEL);
	b = kunit_kzalloc(test, sizeof(*b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, a);
	KUNIT_ASSERT_NOT_NULL(test, b);

	a->scaling = RMX_FULL;
	a->underscan_enable = false;
	*b = *a;

	KUNIT_EXPECT_FALSE(test, is_scaling_state_different(a, b));
}

/**
 * dm_test_scaling_state_scaling_changed - Test differing scaling mode is detected
 * @test: The KUnit test context
 */
static void dm_test_scaling_state_scaling_changed(struct kunit *test)
{
	struct dm_connector_state *a;
	struct dm_connector_state *b;

	a = kunit_kzalloc(test, sizeof(*a), GFP_KERNEL);
	b = kunit_kzalloc(test, sizeof(*b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, a);
	KUNIT_ASSERT_NOT_NULL(test, b);

	a->scaling = RMX_FULL;
	b->scaling = RMX_CENTER;

	KUNIT_EXPECT_TRUE(test, is_scaling_state_different(a, b));
}

/**
 * dm_test_scaling_state_underscan_enabled - Test enabling underscan with borders differs
 * @test: The KUnit test context
 */
static void dm_test_scaling_state_underscan_enabled(struct kunit *test)
{
	struct dm_connector_state *old_state;
	struct dm_connector_state *new_state;

	old_state = kunit_kzalloc(test, sizeof(*old_state), GFP_KERNEL);
	new_state = kunit_kzalloc(test, sizeof(*new_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_state);
	KUNIT_ASSERT_NOT_NULL(test, new_state);

	/* new enables underscan with non-zero borders, old has it disabled */
	new_state->underscan_enable = true;
	new_state->underscan_hborder = 16;
	new_state->underscan_vborder = 16;
	old_state->underscan_enable = false;

	KUNIT_EXPECT_TRUE(test, is_scaling_state_different(new_state, old_state));
}

/**
 * dm_test_scaling_state_underscan_disabled - Test disabling underscan with borders differs
 * @test: The KUnit test context
 */
static void dm_test_scaling_state_underscan_disabled(struct kunit *test)
{
	struct dm_connector_state *old_state;
	struct dm_connector_state *new_state;

	old_state = kunit_kzalloc(test, sizeof(*old_state), GFP_KERNEL);
	new_state = kunit_kzalloc(test, sizeof(*new_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_state);
	KUNIT_ASSERT_NOT_NULL(test, new_state);

	old_state->underscan_enable = true;
	old_state->underscan_hborder = 16;
	old_state->underscan_vborder = 16;
	new_state->underscan_enable = false;

	KUNIT_EXPECT_TRUE(test, is_scaling_state_different(new_state, old_state));
}

/**
 * dm_test_scaling_state_underscan_border_changed - Test changed underscan borders differ
 * @test: The KUnit test context
 */
static void dm_test_scaling_state_underscan_border_changed(struct kunit *test)
{
	struct dm_connector_state *a;
	struct dm_connector_state *b;

	a = kunit_kzalloc(test, sizeof(*a), GFP_KERNEL);
	b = kunit_kzalloc(test, sizeof(*b), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, a);
	KUNIT_ASSERT_NOT_NULL(test, b);

	a->underscan_enable = true;
	a->underscan_hborder = 16;
	a->underscan_vborder = 16;
	*b = *a;
	b->underscan_hborder = 32;

	KUNIT_EXPECT_TRUE(test, is_scaling_state_different(a, b));
}

/* Tests for set_multisync_trigger_params() */

/**
 * dm_test_multisync_trigger_disabled - Test disabled reset leaves params untouched
 * @test: The KUnit test context
 */
static void dm_test_multisync_trigger_disabled(struct kunit *test)
{
	struct dc_stream_state *stream;

	stream = kunit_kzalloc(test, sizeof(*stream), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream);

	stream->triggered_crtc_reset.enabled = false;
	stream->triggered_crtc_reset.event = CRTC_EVENT_VSYNC_FALLING;
	stream->triggered_crtc_reset.delay = TRIGGER_DELAY_NEXT_LINE;

	set_multisync_trigger_params(stream);

	/* Nothing should change when the reset trigger is disabled */
	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.event,
			(int)CRTC_EVENT_VSYNC_FALLING);
	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.delay,
			(int)TRIGGER_DELAY_NEXT_LINE);
}

/**
 * dm_test_multisync_trigger_rising - Test positive vsync polarity selects rising edge
 * @test: The KUnit test context
 */
static void dm_test_multisync_trigger_rising(struct kunit *test)
{
	struct dc_stream_state *stream;
	struct dc_stream_state *master;

	stream = kunit_kzalloc(test, sizeof(*stream), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream);
	master = kunit_kzalloc(test, sizeof(*master), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, master);

	master->timing.flags.VSYNC_POSITIVE_POLARITY = 1;
	stream->triggered_crtc_reset.enabled = true;
	stream->triggered_crtc_reset.event_source = master;

	set_multisync_trigger_params(stream);

	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.event,
			(int)CRTC_EVENT_VSYNC_RISING);
	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.delay,
			(int)TRIGGER_DELAY_NEXT_PIXEL);
}

/**
 * dm_test_multisync_trigger_falling - Test negative vsync polarity selects falling edge
 * @test: The KUnit test context
 */
static void dm_test_multisync_trigger_falling(struct kunit *test)
{
	struct dc_stream_state *stream;
	struct dc_stream_state *master;

	stream = kunit_kzalloc(test, sizeof(*stream), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream);
	master = kunit_kzalloc(test, sizeof(*master), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, master);

	master->timing.flags.VSYNC_POSITIVE_POLARITY = 0;
	stream->triggered_crtc_reset.enabled = true;
	stream->triggered_crtc_reset.event_source = master;

	set_multisync_trigger_params(stream);

	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.event,
			(int)CRTC_EVENT_VSYNC_FALLING);
	KUNIT_EXPECT_EQ(test, (int)stream->triggered_crtc_reset.delay,
			(int)TRIGGER_DELAY_NEXT_PIXEL);
}

/* Tests for set_master_stream() */

/**
 * dm_test_master_stream_highest_refresh - Test highest refresh-rate stream becomes master
 * @test: The KUnit test context
 */
static void dm_test_master_stream_highest_refresh(struct kunit *test)
{
	struct dc_stream_state *stream0, *stream1;
	struct dc_stream_state *stream_set[2];

	stream0 = kunit_kzalloc(test, sizeof(*stream0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream0);
	stream1 = kunit_kzalloc(test, sizeof(*stream1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream1);
	stream_set[0] = stream0;
	stream_set[1] = stream1;

	/* stream0: 60Hz, stream1: 120Hz -> stream1 is master */
	stream0->triggered_crtc_reset.enabled = true;
	stream0->timing.pix_clk_100hz = 1485000;
	stream0->timing.h_total = 2200;
	stream0->timing.v_total = 1125;

	stream1->triggered_crtc_reset.enabled = true;
	stream1->timing.pix_clk_100hz = 2970000;
	stream1->timing.h_total = 2200;
	stream1->timing.v_total = 1125;

	set_master_stream(stream_set, 2);

	KUNIT_EXPECT_PTR_EQ(test, stream0->triggered_crtc_reset.event_source,
			    stream1);
	KUNIT_EXPECT_PTR_EQ(test, stream1->triggered_crtc_reset.event_source,
			    stream1);
}

/**
 * dm_test_master_stream_defaults_to_first - Test default master when none triggered
 * @test: The KUnit test context
 *
 * When no stream has the reset trigger enabled, master_stream stays 0 and all
 * streams point at the first stream as their event source.
 */
static void dm_test_master_stream_defaults_to_first(struct kunit *test)
{
	struct dc_stream_state *stream0, *stream1;
	struct dc_stream_state *stream_set[2];

	stream0 = kunit_kzalloc(test, sizeof(*stream0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream0);
	stream1 = kunit_kzalloc(test, sizeof(*stream1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, stream1);
	stream_set[0] = stream0;
	stream_set[1] = stream1;

	set_master_stream(stream_set, 2);

	KUNIT_EXPECT_PTR_EQ(test, stream0->triggered_crtc_reset.event_source,
			    stream0);
	KUNIT_EXPECT_PTR_EQ(test, stream1->triggered_crtc_reset.event_source,
			    stream0);
}

/* Tests for is_content_protection_different() */

struct dm_test_cp_ctx {
	struct amdgpu_dm_connector *aconnector;
	struct dm_connector_state *new_dm;	/* also connector->state */
	struct dm_connector_state *old_dm;
	struct drm_crtc_state *new_crtc;
	struct drm_crtc_state *old_crtc;
};

static struct dm_test_cp_ctx *dm_test_cp_ctx_alloc(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	ctx->aconnector = kunit_kzalloc(test, sizeof(*ctx->aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->aconnector);
	ctx->new_dm = kunit_kzalloc(test, sizeof(*ctx->new_dm), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->new_dm);
	ctx->old_dm = kunit_kzalloc(test, sizeof(*ctx->old_dm), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->old_dm);
	ctx->new_crtc = kunit_kzalloc(test, sizeof(*ctx->new_crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->new_crtc);
	ctx->old_crtc = kunit_kzalloc(test, sizeof(*ctx->old_crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->old_crtc);

	/* connector->state must be the new dm connector state */
	ctx->aconnector->base.state = &ctx->new_dm->base;
	ctx->aconnector->base.dpms = DRM_MODE_DPMS_ON;

	return ctx;
}

static bool dm_test_cp_diff(struct dm_test_cp_ctx *ctx)
{
	return is_content_protection_different(ctx->new_crtc, ctx->old_crtc,
					       &ctx->new_dm->base,
					       &ctx->old_dm->base,
					       &ctx->aconnector->base, NULL);
}

/**
 * dm_test_cp_diff_hdcp_type_change - Test an HDCP content-type change forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_hdcp_type_change(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.hdcp_content_type = 0;
	ctx->new_dm->base.hdcp_content_type = 1;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_ENABLED;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_EQ(test, (int)ctx->new_dm->base.content_protection,
			(int)DRM_MODE_CONTENT_PROTECTION_DESIRED);
}

/**
 * dm_test_cp_diff_reenable_mode_changed - Test ENABLED->DESIRED with modeset forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_reenable_mode_changed(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_ENABLED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_crtc->mode_changed = true;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_EQ(test, (int)ctx->new_dm->base.content_protection,
			(int)DRM_MODE_CONTENT_PROTECTION_DESIRED);
}

/**
 * dm_test_cp_diff_reenable_no_change - Test ENABLED->DESIRED without modeset restores ENABLED
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_reenable_no_change(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_ENABLED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_crtc->mode_changed = false;

	KUNIT_EXPECT_FALSE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_EQ(test, (int)ctx->new_dm->base.content_protection,
			(int)DRM_MODE_CONTENT_PROTECTION_ENABLED);
}

/**
 * dm_test_cp_diff_undesired - Test UNDESIRED->UNDESIRED needs no update
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_undesired(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;

	KUNIT_EXPECT_FALSE(test, dm_test_cp_diff(ctx));
}

/**
 * dm_test_cp_diff_desired_mode_changed - Test DESIRED->DESIRED with modeset forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_desired_mode_changed(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_crtc->mode_changed = true;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
}

/**
 * dm_test_cp_diff_desired_no_change - Test steady DESIRED->DESIRED needs no update
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_desired_no_change(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_crtc->mode_changed = false;

	KUNIT_EXPECT_FALSE(test, dm_test_cp_diff(ctx));
}

/**
 * dm_test_cp_diff_update_hdcp_hotplug - Test the update_hdcp hot-plug path forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_update_hdcp_hotplug(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);
	struct dc_sink *sink = kunit_kzalloc(test, sizeof(*sink), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, sink);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->update_hdcp = true;
	ctx->aconnector->base.dpms = DRM_MODE_DPMS_ON;
	ctx->aconnector->dc_sink = sink;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_FALSE(test, ctx->new_dm->update_hdcp);
}

/**
 * dm_test_cp_diff_stream_reenabled - Test the stream removed-and-re-enabled path forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_stream_reenabled(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);
	struct drm_crtc *crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, crtc);
	crtc->enabled = true;

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->update_hdcp = true;
	ctx->old_dm->base.crtc = NULL;
	ctx->new_dm->base.crtc = crtc;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_FALSE(test, ctx->new_dm->update_hdcp);
}

/**
 * dm_test_cp_diff_s3_undesired_to_enabled - Test the S3 UNDESIRED->ENABLED path forces true
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_s3_undesired_to_enabled(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_ENABLED;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
	KUNIT_EXPECT_EQ(test, (int)ctx->new_dm->base.content_protection,
			(int)DRM_MODE_CONTENT_PROTECTION_DESIRED);
}

/**
 * dm_test_cp_diff_desired_to_enabled - Test DESIRED->ENABLED needs no update
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_desired_to_enabled(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_ENABLED;

	KUNIT_EXPECT_FALSE(test, dm_test_cp_diff(ctx));
}

/**
 * dm_test_cp_diff_desired_to_undesired - Test DESIRED->UNDESIRED forces update
 * @test: The KUnit test context
 */
static void dm_test_cp_diff_desired_to_undesired(struct kunit *test)
{
	struct dm_test_cp_ctx *ctx = dm_test_cp_ctx_alloc(test);

	ctx->old_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_DESIRED;
	ctx->new_dm->base.content_protection = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;

	KUNIT_EXPECT_TRUE(test, dm_test_cp_diff(ctx));
}

/* Tests for dm_enable_per_frame_crtc_master_sync() */

/**
 * dm_test_per_frame_master_sync_single_stream - Test fewer than two streams is a no-op
 * @test: The KUnit test context
 */
static void dm_test_per_frame_master_sync_single_stream(struct kunit *test)
{
	struct dc_state *context;
	struct dc_stream_state *stream;

	context = kunit_kzalloc(test, sizeof(*context), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, context);
	stream = kunit_kzalloc(test, sizeof(*stream), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, stream);

	stream->triggered_crtc_reset.enabled = true;
	context->streams[0] = stream;
	context->stream_count = 1;

	dm_enable_per_frame_crtc_master_sync(context);

	/* < 2 streams: early return, event_source stays NULL */
	KUNIT_EXPECT_NULL(test, stream->triggered_crtc_reset.event_source);
}

/**
 * dm_test_per_frame_master_sync_two_streams - Test the master is picked and applied
 * @test: The KUnit test context
 */
static void dm_test_per_frame_master_sync_two_streams(struct kunit *test)
{
	struct dc_state *context;
	struct dc_stream_state *stream0, *stream1;

	context = kunit_kzalloc(test, sizeof(*context), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, context);
	stream0 = kunit_kzalloc(test, sizeof(*stream0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, stream0);
	stream1 = kunit_kzalloc(test, sizeof(*stream1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, stream1);

	/* stream0 60Hz, stream1 120Hz, both trigger-reset enabled */
	stream0->triggered_crtc_reset.enabled = true;
	stream0->timing.pix_clk_100hz = 1485000;
	stream0->timing.h_total = 2200;
	stream0->timing.v_total = 1125;
	stream1->triggered_crtc_reset.enabled = true;
	stream1->timing.pix_clk_100hz = 2970000;
	stream1->timing.h_total = 2200;
	stream1->timing.v_total = 1125;
	stream1->timing.flags.VSYNC_POSITIVE_POLARITY = 1;

	context->streams[0] = stream0;
	context->streams[1] = stream1;
	context->stream_count = 2;

	dm_enable_per_frame_crtc_master_sync(context);

	/* set_master_stream picks the highest refresh (stream1) as event source */
	KUNIT_EXPECT_PTR_EQ(test, stream0->triggered_crtc_reset.event_source,
			    stream1);
	KUNIT_EXPECT_PTR_EQ(test, stream1->triggered_crtc_reset.event_source,
			    stream1);
	/* set_multisync_trigger_params applied to enabled streams */
	KUNIT_EXPECT_EQ(test, (int)stream0->triggered_crtc_reset.event,
			(int)CRTC_EVENT_VSYNC_RISING);
	KUNIT_EXPECT_EQ(test, (int)stream0->triggered_crtc_reset.delay,
			(int)TRIGGER_DELAY_NEXT_PIXEL);
}

/**
 * dm_test_per_frame_master_sync_skips_null_stream - Test NULL stream entries are skipped
 * @test: The KUnit test context
 */
static void dm_test_per_frame_master_sync_skips_null_stream(struct kunit *test)
{
	struct dc_state *context;
	struct dc_stream_state *stream;

	context = kunit_kzalloc(test, sizeof(*context), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, context);
	stream = kunit_kzalloc(test, sizeof(*stream), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, stream);

	stream->triggered_crtc_reset.enabled = true;
	stream->timing.pix_clk_100hz = 1485000;
	stream->timing.h_total = 2200;
	stream->timing.v_total = 1125;
	context->streams[0] = stream;
	context->streams[1] = NULL;
	context->stream_count = 2;

	dm_enable_per_frame_crtc_master_sync(context);

	KUNIT_EXPECT_PTR_EQ(test, stream->triggered_crtc_reset.event_source,
			    stream);
}

/* Tests for amdgpu_dm_apply_delay_after_dpcd_poweroff() */

/**
 * dm_test_apply_delay_null_sink - Test a NULL sink returns without delay
 * @test: The KUnit test context
 */
static void dm_test_apply_delay_null_sink(struct kunit *test)
{
	/* NULL sink: early return, no delay, no dereference */
	amdgpu_dm_apply_delay_after_dpcd_poweroff(NULL, NULL);
}

/**
 * dm_test_apply_delay_zero_wait - Test a zero wait interval skips the delay
 * @test: The KUnit test context
 */
static void dm_test_apply_delay_zero_wait(struct kunit *test)
{
	struct dc_sink *sink;

	sink = kunit_kzalloc(test, sizeof(*sink), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, sink);

	/* wait == 0: no msleep, adev is unused so NULL is safe */
	sink->edid_caps.panel_patch.wait_after_dpcd_poweroff_ms = 0;
	amdgpu_dm_apply_delay_after_dpcd_poweroff(NULL, sink);
}

/**
 * dm_test_apply_delay_nonzero_wait - Test a non-zero wait interval executes delay path
 * @test: The KUnit test context
 */
static void dm_test_apply_delay_nonzero_wait(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct dc_sink *sink;

	sink = kunit_kzalloc(test, sizeof(*sink), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, sink);

	sink->edid_caps.panel_patch.wait_after_dpcd_poweroff_ms = 1;
	amdgpu_dm_apply_delay_after_dpcd_poweroff(adev, sink);
}

/*
 * Attach a DC stream to CRTC 0 of @adev so the scanout helpers walk the DC
 * resource context instead of bailing out early.
 */
static void dm_test_crtc_with_stream(struct kunit *test,
				     struct amdgpu_device *adev, struct dc *dc)
{
	struct dc_stream_state *stream;
	struct amdgpu_crtc *acrtc;
	struct dal_logger *logger;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	logger = kunit_kzalloc(test, sizeof(*logger), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, logger);
	logger->dev = &adev->ddev;
	dc->ctx->logger = logger;

	dc->current_state = dm_kunit_alloc_dc_state(test);
	KUNIT_ASSERT_NOT_NULL(test, dc->current_state);

	stream = dm_kunit_alloc_stream(test, NULL);
	stream->ctx = dc->ctx;

	adev->dm.dc = dc;
	adev->mode_info.num_crtc = 1;
	adev->mode_info.crtcs[0] = acrtc;
	acrtc->dm_irq_params.stream = stream;
}

/**
 * dm_test_vblank_get_counter_unmapped_stream - Test a stream without a pipe returns zero
 * @test: The KUnit test context
 */
static void dm_test_vblank_get_counter_unmapped_stream(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct dc *dc = dm_kunit_alloc_dc_with_ctx(test);

	dm_test_crtc_with_stream(test, adev, dc);

	KUNIT_EXPECT_EQ(test, dm_vblank_get_counter(adev, 0), 0U);
}

/**
 * dm_test_crtc_get_scanoutpos_unmapped_stream - Test scanout position for an unmapped stream
 * @test: The KUnit test context
 */
static void dm_test_crtc_get_scanoutpos_unmapped_stream(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct dc *dc = dm_kunit_alloc_dc_with_ctx(test);
	u32 vbl = 0xdeadbeef;
	u32 position = 0xdeadbeef;

	dm_test_crtc_with_stream(test, adev, dc);

	KUNIT_EXPECT_EQ(test, dm_crtc_get_scanoutpos(adev, 0, &vbl, &position), 0);
	KUNIT_EXPECT_EQ(test, vbl, 0U);
	KUNIT_EXPECT_EQ(test, position, 0U);
}

/**
 * dm_test_crtc_get_scanoutpos_exits_idle - Test idle optimizations are disabled first
 * @test: The KUnit test context
 */
static void dm_test_crtc_get_scanoutpos_exits_idle(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct dc *dc = dm_kunit_alloc_dc_with_ctx(test);
	u32 vbl = 0;
	u32 position = 0;

	dm_test_crtc_with_stream(test, adev, dc);
	dc->caps.ips_support = true;
	dc->idle_optimizations_allowed = true;

	KUNIT_EXPECT_EQ(test, dm_crtc_get_scanoutpos(adev, 0, &vbl, &position), 0);
}

static struct drm_atomic_commit *dm_test_alloc_commit(struct kunit *test,
						     struct amdgpu_device *adev)
{
	struct drm_atomic_commit *state;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);
	state->dev = &adev->ddev;

	return state;
}

/**
 * dm_test_atomic_get_state_already_acquired - Test an acquired DM state is returned as is
 * @test: The KUnit test context
 */
static void dm_test_atomic_get_state_already_acquired(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state = dm_test_alloc_commit(test, adev);
	struct dm_atomic_state *dm_state;
	struct dm_atomic_state *acquired;

	dm_state = kunit_kzalloc(test, sizeof(*dm_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_state);
	acquired = dm_state;

	KUNIT_EXPECT_EQ(test, dm_atomic_get_state(state, &acquired), 0);
	KUNIT_EXPECT_PTR_EQ(test, acquired, dm_state);
}

/**
 * dm_test_atomic_duplicate_state_no_context - Test duplication fails without a DC context
 * @test: The KUnit test context
 */
static void dm_test_atomic_duplicate_state_no_context(struct kunit *test)
{
	struct dm_atomic_state *old_state;
	struct drm_private_obj *obj;

	obj = kunit_kzalloc(test, sizeof(*obj), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, obj);
	old_state = kunit_kzalloc(test, sizeof(*old_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_state);

	obj->state = &old_state->base;

	KUNIT_EXPECT_NULL(test, dm_atomic_duplicate_state(obj));
}

/* Tests for add_affected_mst_dsc_crtcs() */

/**
 * dm_test_add_affected_mst_dsc_crtcs_no_connector - Test an empty state adds nothing
 * @test: The KUnit test context
 */
static void dm_test_add_affected_mst_dsc_crtcs_no_connector(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state = dm_test_alloc_commit(test, adev);
	struct drm_crtc *crtc;

	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);

	KUNIT_EXPECT_EQ(test, add_affected_mst_dsc_crtcs(state, crtc), 0);
}

/*
 * Build an atomic state holding a single connector bound to @crtc.
 */
static struct drm_atomic_commit *
dm_test_state_with_connector(struct kunit *test, struct drm_connector *connector,
			     struct drm_crtc *crtc)
{
	struct drm_connector_state *conn_state;
	struct drm_atomic_commit *state;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);
	conn_state = kunit_kzalloc(test, sizeof(*conn_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, conn_state);

	state->connectors = kunit_kzalloc(test, sizeof(*state->connectors),
					  GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->connectors);

	conn_state->crtc = crtc;
	state->num_connector = 1;
	state->connectors[0].ptr = connector;
	state->connectors[0].old_state = conn_state;
	state->connectors[0].new_state = conn_state;

	return state;
}

/**
 * dm_test_add_affected_mst_dsc_crtcs_writeback - Test writeback connectors are ignored
 * @test: The KUnit test context
 */
static void dm_test_add_affected_mst_dsc_crtcs_writeback(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);

	aconnector->base.connector_type = DRM_MODE_CONNECTOR_WRITEBACK;
	state = dm_test_state_with_connector(test, &aconnector->base, crtc);

	KUNIT_EXPECT_EQ(test, add_affected_mst_dsc_crtcs(state, crtc), 0);
}

/**
 * dm_test_add_affected_mst_dsc_crtcs_not_mst - Test a non-MST connector adds nothing
 * @test: The KUnit test context
 */
static void dm_test_add_affected_mst_dsc_crtcs_not_mst(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);

	aconnector->base.connector_type = DRM_MODE_CONNECTOR_DisplayPort;
	state = dm_test_state_with_connector(test, &aconnector->base, crtc);

	KUNIT_EXPECT_EQ(test, add_affected_mst_dsc_crtcs(state, crtc), 0);
}

/**
 * dm_test_add_affected_mst_dsc_crtcs_other_crtc - Test connectors on other CRTCs are skipped
 * @test: The KUnit test context
 */
static void dm_test_add_affected_mst_dsc_crtcs_other_crtc(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc, *other_crtc;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);
	other_crtc = kunit_kzalloc(test, sizeof(*other_crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, other_crtc);

	aconnector->base.connector_type = DRM_MODE_CONNECTOR_DisplayPort;
	state = dm_test_state_with_connector(test, &aconnector->base, other_crtc);

	KUNIT_EXPECT_EQ(test, add_affected_mst_dsc_crtcs(state, crtc), 0);
}

/**
 * dm_test_add_affected_mst_dsc_crtcs_disabled - Test a disabled connector uses its old state
 * @test: The KUnit test context
 */
static void dm_test_add_affected_mst_dsc_crtcs_disabled(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_connector_state *new_conn_state;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);
	new_conn_state = kunit_kzalloc(test, sizeof(*new_conn_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, new_conn_state);

	aconnector->base.connector_type = DRM_MODE_CONNECTOR_DisplayPort;
	state = dm_test_state_with_connector(test, &aconnector->base, crtc);
	state->connectors[0].new_state = new_conn_state;

	KUNIT_EXPECT_EQ(test, add_affected_mst_dsc_crtcs(state, crtc), 0);
}

/* Tests for amdgpu_dm_commit_cursors() */

/**
 * dm_test_commit_cursors_no_planes - Test an empty plane set is a no-op
 * @test: The KUnit test context
 */
static void dm_test_commit_cursors_no_planes(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);

	amdgpu_dm_commit_cursors(dm_test_alloc_commit(test, adev));
}

/*
 * Build a commit holding a single plane of @type with empty old and new plane
 * states.
 */
static struct drm_atomic_commit *
dm_test_commit_with_plane(struct kunit *test, struct amdgpu_device *adev,
			  enum drm_plane_type type)
{
	struct drm_atomic_commit *state = dm_test_alloc_commit(test, adev);
	struct drm_plane_state *old_state, *new_state;
	struct drm_plane *plane;

	plane = kunit_kzalloc(test, sizeof(*plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, plane);
	old_state = kunit_kzalloc(test, sizeof(*old_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_state);
	new_state = kunit_kzalloc(test, sizeof(*new_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, new_state);
	state->planes = kunit_kzalloc(test, sizeof(*state->planes), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->planes);

	plane->type = type;
	plane->dev = &adev->ddev;
	plane->state = new_state;
	adev->ddev.mode_config.num_total_plane = 1;
	state->planes[0].ptr = plane;
	state->planes[0].old_state = old_state;

	return state;
}

/**
 * dm_test_commit_cursors_skips_non_cursor - Test non-cursor planes are skipped
 * @test: The KUnit test context
 */
static void dm_test_commit_cursors_skips_non_cursor(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);

	amdgpu_dm_commit_cursors(dm_test_commit_with_plane(test, adev,
							   DRM_PLANE_TYPE_PRIMARY));
}

/**
 * dm_test_commit_cursors_updates_cursor - Test cursor planes reach the cursor update
 * @test: The KUnit test context
 */
static void dm_test_commit_cursors_updates_cursor(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);

	amdgpu_dm_commit_cursors(dm_test_commit_with_plane(test, adev,
							   DRM_PLANE_TYPE_CURSOR));
}

/**
 * dm_test_update_cursor_no_framebuffer - Test missing framebuffers leave the update untouched
 * @test: The KUnit test context
 */
static void dm_test_update_cursor_no_framebuffer(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state;
	struct dc_stream_update *update;
	struct drm_plane *plane;

	state = dm_test_commit_with_plane(test, adev, DRM_PLANE_TYPE_CURSOR);
	plane = state->planes[0].ptr;
	update = kunit_kzalloc(test, sizeof(*update), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, update);

	update->cursor_attributes = (void *)1;
	update->cursor_position = (void *)1;

	amdgpu_dm_update_cursor(plane, state->planes[0].old_state, update);

	KUNIT_EXPECT_PTR_EQ(test, update->cursor_attributes, (void *)1);
	KUNIT_EXPECT_PTR_EQ(test, update->cursor_position, (void *)1);
}

/**
 * dm_test_update_cursor_disables_stream - Test removing the framebuffer disables the cursor
 * @test: The KUnit test context
 */
static void dm_test_update_cursor_disables_stream(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state;
	struct dc_stream_update *update;
	struct drm_plane_state *old_plane_state;
	struct dm_crtc_state *crtc_state;
	struct amdgpu_framebuffer *afb;
	struct amdgpu_crtc *acrtc;
	struct drm_plane *plane;

	state = dm_test_commit_with_plane(test, adev, DRM_PLANE_TYPE_CURSOR);
	plane = state->planes[0].ptr;
	old_plane_state = state->planes[0].old_state;
	crtc_state = kunit_kzalloc(test, sizeof(*crtc_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc_state);
	afb = kunit_kzalloc(test, sizeof(*afb), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, afb);
	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);
	update = kunit_kzalloc(test, sizeof(*update), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, update);

	old_plane_state->fb = &afb->base;
	old_plane_state->crtc = &acrtc->base;
	acrtc->base.state = &crtc_state->base;
	crtc_state->stream = dm_kunit_alloc_stream(test, NULL);
	crtc_state->stream->cursor_position.enable = true;

	amdgpu_dm_update_cursor(plane, old_plane_state, update);

	KUNIT_EXPECT_FALSE(test, crtc_state->stream->cursor_position.enable);
	KUNIT_EXPECT_PTR_EQ(test, update->cursor_position,
			    &crtc_state->stream->cursor_position);
	KUNIT_EXPECT_NULL(test, update->cursor_attributes);
}

/* Tests for dm_arm_vblank_event() */

struct dm_test_vblank_ctx {
	struct amdgpu_device *adev;
	struct amdgpu_crtc *acrtc;
	struct dm_crtc_state *acrtc_state;
	struct drm_pending_vblank_event *event;
};

/*
 * A CRTC with one active plane and a pending vblank event. There is no
 * initialised vblank, so drm_crtc_vblank_get() fails, which the function under
 * test ignores.
 */
static struct dm_test_vblank_ctx *dm_test_vblank_ctx_alloc(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	ctx->adev = dm_kunit_alloc_adev(test);
	ctx->acrtc = kunit_kzalloc(test, sizeof(*ctx->acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->acrtc);
	ctx->acrtc_state = kunit_kzalloc(test, sizeof(*ctx->acrtc_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->acrtc_state);
	ctx->event = kunit_kzalloc(test, sizeof(*ctx->event), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->event);

	ctx->acrtc->base.dev = &ctx->adev->ddev;
	ctx->acrtc->base.state = &ctx->acrtc_state->base;
	ctx->acrtc_state->base.event = ctx->event;
	ctx->acrtc_state->active_planes = 1;

	return ctx;
}

static void dm_test_arm_vblank(struct dm_test_vblank_ctx *ctx, bool pflip_update,
			       bool cursor_update)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->adev->ddev.event_lock, flags);
	dm_arm_vblank_event(ctx->acrtc, ctx->acrtc_state, pflip_update,
			    cursor_update);
	spin_unlock_irqrestore(&ctx->adev->ddev.event_lock, flags);
}

static void dm_test_arm_vblank_pre_programming(struct dm_test_vblank_ctx *ctx,
					       bool pflip_update, bool cursor_update)
{
	struct dm_crtc_state *state = ctx->acrtc_state;
	struct amdgpu_crtc *acrtc = ctx->acrtc;
	unsigned long flags;

	spin_lock_irqsave(&ctx->adev->ddev.event_lock, flags);
	dm_arm_vblank_event_pre_programming(acrtc, state, pflip_update, cursor_update);
	spin_unlock_irqrestore(&ctx->adev->ddev.event_lock, flags);
}

/**
 * dm_test_arm_vblank_pre_programming_no_event - Test missing event takes no reference
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_pre_programming_no_event(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);
	struct drm_vblank_crtc *vblank;

	KUNIT_ASSERT_EQ(test, drm_vblank_init(&ctx->adev->ddev, 1), 0);
	vblank = drm_crtc_vblank_crtc(&ctx->acrtc->base);
	ctx->acrtc_state->base.event = NULL;

	dm_test_arm_vblank_pre_programming(ctx, true, false);

	KUNIT_EXPECT_EQ(test, atomic_read(&vblank->refcount), 0);
}

/**
 * dm_test_arm_vblank_pre_programming_no_planes - Test inactive CRTC takes no reference
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_pre_programming_no_planes(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);
	struct drm_vblank_crtc *vblank;

	KUNIT_ASSERT_EQ(test, drm_vblank_init(&ctx->adev->ddev, 1), 0);
	vblank = drm_crtc_vblank_crtc(&ctx->acrtc->base);
	ctx->acrtc_state->active_planes = 0;

	dm_test_arm_vblank_pre_programming(ctx, false, true);

	KUNIT_EXPECT_EQ(test, atomic_read(&vblank->refcount), 0);
}

/**
 * dm_test_arm_vblank_pre_programming_update - Test an update takes a vblank reference
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_pre_programming_update(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);
	struct drm_vblank_crtc *vblank;

	KUNIT_ASSERT_EQ(test, drm_vblank_init(&ctx->adev->ddev, 1), 0);
	vblank = drm_crtc_vblank_crtc(&ctx->acrtc->base);

	dm_test_arm_vblank_pre_programming(ctx, true, false);

	KUNIT_EXPECT_EQ(test, atomic_read(&vblank->refcount), 1);
	drm_crtc_vblank_put(&ctx->acrtc->base);
}

/**
 * dm_test_arm_vblank_event_no_event - Test a commit without an event is a no-op
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_event_no_event(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);

	ctx->acrtc_state->base.event = NULL;

	dm_test_arm_vblank(ctx, true, false);

	KUNIT_EXPECT_NULL(test, ctx->acrtc->event);
}

/**
 * dm_test_arm_vblank_event_no_active_planes - Test an event is left armed without planes
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_event_no_active_planes(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);

	ctx->acrtc_state->active_planes = 0;

	dm_test_arm_vblank(ctx, false, true);

	KUNIT_EXPECT_NULL(test, ctx->acrtc->event);
	KUNIT_EXPECT_PTR_EQ(test, ctx->acrtc_state->base.event, ctx->event);
}

/**
 * dm_test_arm_vblank_event_pflip - Test a page flip arms the flip ISR
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_event_pflip(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);

	dm_test_arm_vblank(ctx, true, false);

	KUNIT_EXPECT_PTR_EQ(test, ctx->acrtc->event, ctx->event);
	KUNIT_EXPECT_NULL(test, ctx->acrtc_state->base.event);
	KUNIT_EXPECT_EQ(test, (int)ctx->acrtc->pflip_status,
			(int)AMDGPU_FLIP_SUBMITTED);
}

/**
 * dm_test_arm_vblank_event_cursor - Test a cursor update consumes the event
 * @test: The KUnit test context
 */
static void dm_test_arm_vblank_event_cursor(struct kunit *test)
{
	struct dm_test_vblank_ctx *ctx = dm_test_vblank_ctx_alloc(test);

	dm_test_arm_vblank(ctx, false, true);

	KUNIT_EXPECT_PTR_EQ(test, ctx->acrtc->event, ctx->event);
	KUNIT_EXPECT_NULL(test, ctx->acrtc_state->base.event);
	KUNIT_EXPECT_EQ(test, (int)ctx->acrtc->pflip_status,
			(int)AMDGPU_FLIP_NONE);
}

/**
 * dm_test_update_pflip_irq_state_dcn - Test DCN skips the GRPH_PFLIP reapply
 * @test: The KUnit test context
 */
static void dm_test_update_pflip_irq_state_dcn(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct amdgpu_crtc *acrtc;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	adev->mode_info.num_crtc = 1;
	adev->ip_versions[DCE_HWIP][0] = IP_VERSION(3, 2, 0);

	dm_update_pflip_irq_state(adev, acrtc);
}

/*
 * Spy for the pageflip IRQ source: amdgpu_irq_update() always dispatches
 * through src->funcs->set(), which needs a registered IH ring on real
 * hardware. Recording the requested state instead keeps the DCE reapply path
 * reachable and observable.
 */
struct dm_test_irq_spy {
	unsigned int set_count;
	unsigned int last_type;
	enum amdgpu_interrupt_state last_state;
};

static struct dm_test_irq_spy dm_test_irq_spy_data;

static int dm_test_irq_set(struct amdgpu_device *adev,
			   struct amdgpu_irq_src *src, unsigned int type,
			   enum amdgpu_interrupt_state state)
{
	dm_test_irq_spy_data.set_count++;
	dm_test_irq_spy_data.last_type = type;
	dm_test_irq_spy_data.last_state = state;

	return 0;
}

static const struct amdgpu_irq_src_funcs dm_test_irq_funcs = {
	.set = dm_test_irq_set,
};

/**
 * dm_test_update_pflip_irq_state_dce - Test DCE reapplies the GRPH_PFLIP state
 * @test: The KUnit test context
 */
static void dm_test_update_pflip_irq_state_dce(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct amdgpu_crtc *acrtc;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	memset(&dm_test_irq_spy_data, 0, sizeof(dm_test_irq_spy_data));
	adev->mode_info.num_crtc = 2;
	adev->ip_versions[DCE_HWIP][0] = 0;
	adev->pageflip_irq.funcs = &dm_test_irq_funcs;
	acrtc->crtc_id = 1;
	spin_lock_init(&adev->irq.lock);

	dm_update_pflip_irq_state(adev, acrtc);

	KUNIT_EXPECT_EQ(test, dm_test_irq_spy_data.set_count, 1U);
	KUNIT_EXPECT_EQ(test, dm_test_irq_spy_data.last_type,
			(unsigned int)AMDGPU_CRTC_IRQ_VBLANK2);
	KUNIT_EXPECT_EQ(test, (int)dm_test_irq_spy_data.last_state,
			(int)AMDGPU_IRQ_STATE_DISABLE);
}

/* Tests for amdgpu_dm_crtc_mem_type_changed() */

struct dm_test_mem_type_ctx {
	struct amdgpu_device *adev;
	struct drm_atomic_commit *state;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state;
	struct drm_plane_state *new_plane_state;
};

/*
 * Register a single plane on the CRTC's plane mask. get_mem_type() walks
 * fb->obj[0] back to an amdgpu_bo, so the framebuffers are backed by fake
 * buffer objects with a TTM resource instead of a live TTM device.
 */
static struct drm_framebuffer *dm_test_alloc_fb(struct kunit *test,
						u32 mem_type)
{
	struct drm_framebuffer *fb;
	struct ttm_resource *res;
	struct amdgpu_bo *abo;

	fb = kunit_kzalloc(test, sizeof(*fb), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fb);
	abo = kunit_kzalloc(test, sizeof(*abo), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, abo);
	res = kunit_kzalloc(test, sizeof(*res), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, res);

	res->mem_type = mem_type;
	abo->tbo.resource = res;
	fb->obj[0] = &abo->tbo.base;

	return fb;
}

static struct dm_test_mem_type_ctx *dm_test_mem_type_ctx_alloc(struct kunit *test)
{
	struct dm_test_mem_type_ctx *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	ctx->adev = dm_kunit_alloc_adev(test);
	ctx->state = dm_test_alloc_commit(test, ctx->adev);
	ctx->crtc_state = kunit_kzalloc(test, sizeof(*ctx->crtc_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->crtc_state);
	ctx->plane = drm_kunit_helper_create_primary_plane(test, &ctx->adev->ddev,
							   NULL, NULL, NULL, 0, NULL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->plane);
	ctx->old_plane_state = kunit_kzalloc(test, sizeof(*ctx->old_plane_state),
					     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->old_plane_state);
	ctx->new_plane_state = kunit_kzalloc(test, sizeof(*ctx->new_plane_state),
					     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->new_plane_state);
	ctx->state->planes = kunit_kcalloc(test, ctx->plane->index + 1,
					   sizeof(*ctx->state->planes), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx->state->planes);

	ctx->crtc_state->plane_mask = drm_plane_mask(ctx->plane);
	ctx->state->planes[ctx->plane->index].ptr = ctx->plane;

	return ctx;
}

/**
 * dm_test_mem_type_changed_no_planes - Test an empty plane mask reports no change
 * @test: The KUnit test context
 */
static void dm_test_mem_type_changed_no_planes(struct kunit *test)
{
	struct dm_test_mem_type_ctx *ctx = dm_test_mem_type_ctx_alloc(test);

	ctx->crtc_state->plane_mask = 0;

	KUNIT_EXPECT_FALSE(test, amdgpu_dm_crtc_mem_type_changed(&ctx->adev->ddev,
								ctx->state,
								ctx->crtc_state));
}

/**
 * dm_test_mem_type_changed_missing_state - Test a plane without both states is skipped
 * @test: The KUnit test context
 */
static void dm_test_mem_type_changed_missing_state(struct kunit *test)
{
	struct dm_test_mem_type_ctx *ctx = dm_test_mem_type_ctx_alloc(test);

	ctx->state->planes[ctx->plane->index].new_state = ctx->new_plane_state;

	KUNIT_EXPECT_FALSE(test, amdgpu_dm_crtc_mem_type_changed(&ctx->adev->ddev,
								ctx->state,
								ctx->crtc_state));
}

/**
 * dm_test_mem_type_changed_same_domain - Test identical memory domains report no change
 * @test: The KUnit test context
 */
static void dm_test_mem_type_changed_same_domain(struct kunit *test)
{
	struct dm_test_mem_type_ctx *ctx = dm_test_mem_type_ctx_alloc(test);

	ctx->old_plane_state->fb = dm_test_alloc_fb(test, TTM_PL_VRAM);
	ctx->new_plane_state->fb = dm_test_alloc_fb(test, TTM_PL_VRAM);
	ctx->state->planes[ctx->plane->index].old_state = ctx->old_plane_state;
	ctx->state->planes[ctx->plane->index].new_state = ctx->new_plane_state;

	KUNIT_EXPECT_FALSE(test, amdgpu_dm_crtc_mem_type_changed(&ctx->adev->ddev,
								ctx->state,
								ctx->crtc_state));
}

/**
 * dm_test_mem_type_changed_different_domain - Test a domain migration is detected
 * @test: The KUnit test context
 */
static void dm_test_mem_type_changed_different_domain(struct kunit *test)
{
	struct dm_test_mem_type_ctx *ctx = dm_test_mem_type_ctx_alloc(test);

	ctx->old_plane_state->fb = dm_test_alloc_fb(test, TTM_PL_TT);
	ctx->new_plane_state->fb = dm_test_alloc_fb(test, TTM_PL_VRAM);
	ctx->state->planes[ctx->plane->index].old_state = ctx->old_plane_state;
	ctx->state->planes[ctx->plane->index].new_state = ctx->new_plane_state;

	KUNIT_EXPECT_TRUE(test, amdgpu_dm_crtc_mem_type_changed(&ctx->adev->ddev,
							       ctx->state,
							       ctx->crtc_state));
}

static struct kunit_case amdgpu_dm_tests[] = {
	/* Simple DM callbacks */
	KUNIT_CASE(dm_test_wait_for_idle),
	KUNIT_CASE(dm_test_soft_reset),
	KUNIT_CASE(dm_test_set_clockgating_state),
	KUNIT_CASE(dm_test_set_powergating_state),
	KUNIT_CASE(dm_test_bandwidth_update),
	KUNIT_CASE(dm_test_crtc_complete_writeback_no_connector),
	KUNIT_CASE(dm_test_crtc_complete_writeback_not_pending),
	KUNIT_CASE(dm_test_vblank_get_counter_out_of_range),
	KUNIT_CASE(dm_test_vblank_get_counter_no_stream),
	KUNIT_CASE(dm_test_crtc_get_scanoutpos_invalid_crtc),
	KUNIT_CASE(dm_test_crtc_get_scanoutpos_no_stream),
	KUNIT_CASE(dm_test_atomic_get_new_state_empty),
	KUNIT_CASE(dm_test_atomic_get_new_state_match),
	KUNIT_CASE(dm_test_atomic_destroy_state_no_context),
	/* dm_plane_layer_index_cmp */
	KUNIT_CASE(dm_test_plane_layer_index_cmp_equal),
	KUNIT_CASE(dm_test_plane_layer_index_cmp_descending),
	KUNIT_CASE(dm_test_plane_layer_index_cmp_ascending),
	/* fill_plane_color_attributes */
	KUNIT_CASE(dm_test_fill_color_attr_rgb_format),
	KUNIT_CASE(dm_test_fill_color_attr_bt601_full),
	KUNIT_CASE(dm_test_fill_color_attr_bt601_limited),
	KUNIT_CASE(dm_test_fill_color_attr_bt709_full),
	KUNIT_CASE(dm_test_fill_color_attr_bt709_limited),
	KUNIT_CASE(dm_test_fill_color_attr_bt2020_full),
	KUNIT_CASE(dm_test_fill_color_attr_bt2020_limited),
	KUNIT_CASE(dm_test_fill_color_attr_invalid_encoding),
	/* modereset_required */
	KUNIT_CASE(dm_test_modereset_required_when_inactive_and_modeset),
	KUNIT_CASE(dm_test_modereset_not_required_when_active_and_modeset),
	KUNIT_CASE(dm_test_modereset_not_required_when_inactive_without_modeset),
	/* is_scaling_state_different */
	KUNIT_CASE(dm_test_scaling_state_same),
	KUNIT_CASE(dm_test_scaling_state_scaling_changed),
	KUNIT_CASE(dm_test_scaling_state_underscan_enabled),
	KUNIT_CASE(dm_test_scaling_state_underscan_disabled),
	KUNIT_CASE(dm_test_scaling_state_underscan_border_changed),
	/* set_multisync_trigger_params */
	KUNIT_CASE(dm_test_multisync_trigger_disabled),
	KUNIT_CASE(dm_test_multisync_trigger_rising),
	KUNIT_CASE(dm_test_multisync_trigger_falling),
	/* set_master_stream */
	KUNIT_CASE(dm_test_master_stream_highest_refresh),
	KUNIT_CASE(dm_test_master_stream_defaults_to_first),
	/* is_content_protection_different */
	KUNIT_CASE(dm_test_cp_diff_hdcp_type_change),
	KUNIT_CASE(dm_test_cp_diff_reenable_mode_changed),
	KUNIT_CASE(dm_test_cp_diff_reenable_no_change),
	KUNIT_CASE(dm_test_cp_diff_undesired),
	KUNIT_CASE(dm_test_cp_diff_desired_mode_changed),
	KUNIT_CASE(dm_test_cp_diff_desired_no_change),
	KUNIT_CASE(dm_test_cp_diff_update_hdcp_hotplug),
	KUNIT_CASE(dm_test_cp_diff_stream_reenabled),
	KUNIT_CASE(dm_test_cp_diff_s3_undesired_to_enabled),
	KUNIT_CASE(dm_test_cp_diff_desired_to_enabled),
	KUNIT_CASE(dm_test_cp_diff_desired_to_undesired),
	/* dm_enable_per_frame_crtc_master_sync */
	KUNIT_CASE(dm_test_per_frame_master_sync_single_stream),
	KUNIT_CASE(dm_test_per_frame_master_sync_two_streams),
	KUNIT_CASE(dm_test_per_frame_master_sync_skips_null_stream),
	/* amdgpu_dm_apply_delay_after_dpcd_poweroff */
	KUNIT_CASE(dm_test_apply_delay_null_sink),
	KUNIT_CASE(dm_test_apply_delay_zero_wait),
	KUNIT_CASE(dm_test_apply_delay_nonzero_wait),
	/* dm_vblank_get_counter / dm_crtc_get_scanoutpos with a stream */
	KUNIT_CASE(dm_test_vblank_get_counter_unmapped_stream),
	KUNIT_CASE(dm_test_crtc_get_scanoutpos_unmapped_stream),
	KUNIT_CASE(dm_test_crtc_get_scanoutpos_exits_idle),
	/* dm_atomic_get_state / dm_atomic_duplicate_state */
	KUNIT_CASE(dm_test_atomic_get_state_already_acquired),
	KUNIT_CASE(dm_test_atomic_duplicate_state_no_context),
	/* add_affected_mst_dsc_crtcs */
	KUNIT_CASE(dm_test_add_affected_mst_dsc_crtcs_no_connector),
	KUNIT_CASE(dm_test_add_affected_mst_dsc_crtcs_writeback),
	KUNIT_CASE(dm_test_add_affected_mst_dsc_crtcs_not_mst),
	KUNIT_CASE(dm_test_add_affected_mst_dsc_crtcs_other_crtc),
	KUNIT_CASE(dm_test_add_affected_mst_dsc_crtcs_disabled),
	/* amdgpu_dm_commit_cursors */
	KUNIT_CASE(dm_test_commit_cursors_no_planes),
	KUNIT_CASE(dm_test_commit_cursors_skips_non_cursor),
	KUNIT_CASE(dm_test_commit_cursors_updates_cursor),
	KUNIT_CASE(dm_test_update_cursor_no_framebuffer),
	KUNIT_CASE(dm_test_update_cursor_disables_stream),
	/* dm_arm_vblank_event */
	KUNIT_CASE(dm_test_arm_vblank_event_no_event),
	KUNIT_CASE(dm_test_arm_vblank_event_no_active_planes),
	KUNIT_CASE(dm_test_arm_vblank_event_pflip),
	KUNIT_CASE(dm_test_arm_vblank_event_cursor),
	KUNIT_CASE(dm_test_arm_vblank_pre_programming_no_event),
	KUNIT_CASE(dm_test_arm_vblank_pre_programming_no_planes),
	KUNIT_CASE(dm_test_arm_vblank_pre_programming_update),
	/* dm_update_pflip_irq_state */
	KUNIT_CASE(dm_test_update_pflip_irq_state_dcn),
	KUNIT_CASE(dm_test_update_pflip_irq_state_dce),
	/* amdgpu_dm_crtc_mem_type_changed */
	KUNIT_CASE(dm_test_mem_type_changed_no_planes),
	KUNIT_CASE(dm_test_mem_type_changed_missing_state),
	KUNIT_CASE(dm_test_mem_type_changed_same_domain),
	KUNIT_CASE(dm_test_mem_type_changed_different_domain),
	{}
};

static struct kunit_suite amdgpu_dm_test_suite = {
	.name = "amdgpu_dm",
	.test_cases = amdgpu_dm_tests,
};

kunit_test_suite(amdgpu_dm_test_suite);

MODULE_AUTHOR("AMD");
MODULE_DESCRIPTION("KUnit tests for amdgpu_dm");
MODULE_LICENSE("Dual MIT/GPL");
