// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * KUnit tests for amdgpu_dm_cursor.c
 *
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include <kunit/test.h>
#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
#include <drm/drm_colorop.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>

#include "dc.h"
#include "amdgpu.h"
#include "amdgpu_mode.h"
#include "amdgpu_dm.h"
#include "amdgpu_dm_cursor.h"
#include "amdgpu_dm_kunit_test_helpers.h"

struct dm_cursor_fb_fixture {
	struct amdgpu_device *adev;
	struct amdgpu_crtc *acrtc;
	struct amdgpu_framebuffer *afb;
	struct drm_plane_state *plane_state;
};

static struct dm_cursor_fb_fixture dm_test_alloc_cursor_fb_fixture(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = { 0 };

	fixture.adev = dm_kunit_alloc_adev(test);
	fixture.acrtc = kunit_kzalloc(test, sizeof(*fixture.acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fixture.acrtc);
	fixture.afb = kunit_kzalloc(test, sizeof(*fixture.afb), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fixture.afb);
	fixture.plane_state = kunit_kzalloc(test, sizeof(*fixture.plane_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fixture.plane_state);

	fixture.acrtc->base.dev = &fixture.adev->ddev;
	fixture.acrtc->max_cursor_width = 256;
	fixture.acrtc->max_cursor_height = 256;
	fixture.afb->base.width = 64;
	fixture.afb->base.height = 64;
	fixture.afb->base.pitches[0] = 64 * 4;
	fixture.afb->base.format = drm_format_info(DRM_FORMAT_ARGB8888);
	fixture.plane_state->fb = &fixture.afb->base;
	fixture.plane_state->src_w = 64 << 16;
	fixture.plane_state->src_h = 64 << 16;

	return fixture;
}

/* Tests for dm_check_cursor_fb() */

/**
 * dm_test_check_cursor_fb_valid_linear - Test a supported linear cursor framebuffer
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_valid_linear(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.adev->family = AMDGPU_FAMILY_AI;

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			0);
}

/**
 * dm_test_check_cursor_fb_rejects_size - Test an oversized cursor framebuffer
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_rejects_size(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.afb->base.width = fixture.acrtc->max_cursor_width + 1;

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_rejects_cropping - Test cursor framebuffer cropping rejection
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_rejects_cropping(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.plane_state->src_w = 32 << 16;

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_rejects_pitch - Test unsupported cursor framebuffer pitch
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_rejects_pitch(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.afb->base.pitches[0] = 96 * 4;

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_rejects_unsupported_pitch - Test matching unsupported pitch
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_rejects_unsupported_pitch(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.afb->base.width = 32;
	fixture.afb->base.pitches[0] = 32 * 4;
	fixture.plane_state->src_w = 32 << 16;

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_rejects_tiling - Test tiled cursor framebuffer rejection
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_rejects_tiling(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.adev->family = AMDGPU_FAMILY_AI;
	fixture.afb->tiling_flags = AMDGPU_TILING_SET(SWIZZLE_MODE, 1);

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_gfx12_tiling - Test GFX12 cursor tiling decoding
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_gfx12_tiling(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.adev->family = AMDGPU_FAMILY_GC_12_0_0;
	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			0);

	fixture.afb->tiling_flags = AMDGPU_TILING_SET(GFX12_SWIZZLE_MODE, 1);
	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_pre_ai_tiling - Test legacy cursor tiling decoding
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_pre_ai_tiling(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.adev->family = AMDGPU_FAMILY_CZ;
	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			0);

	fixture.afb->tiling_flags = AMDGPU_TILING_SET(ARRAY_MODE, DC_ARRAY_1D_TILED_THIN1);
	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			-EINVAL);
}

/**
 * dm_test_check_cursor_fb_modifier_skips_tiling - Test modifier validation stays in DRM core
 * @test: The KUnit test context
 */
static void dm_test_check_cursor_fb_modifier_skips_tiling(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);

	fixture.adev->family = AMDGPU_FAMILY_AI;
	fixture.afb->base.flags = DRM_MODE_FB_MODIFIERS;
	fixture.afb->tiling_flags = AMDGPU_TILING_SET(SWIZZLE_MODE, 1);

	KUNIT_EXPECT_EQ(test,
			dm_check_cursor_fb(fixture.acrtc, fixture.plane_state, &fixture.afb->base),
			0);
}

/* Tests for amdgpu_dm_check_native_cursor_state() */

/**
 * dm_test_check_native_cursor_state_disabled - Test disabled cursor needs no validation
 * @test: The KUnit test context
 */
static void dm_test_check_native_cursor_state_disabled(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			amdgpu_dm_check_native_cursor_state(NULL, NULL, NULL, false),
			0);
}

/**
 * dm_test_check_native_cursor_state_rejects_offset - Test source offset rejection
 * @test: The KUnit test context
 */
static void dm_test_check_native_cursor_state_rejects_offset(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct amdgpu_crtc *acrtc;
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state;
	struct drm_plane_state *new_plane_state;
	int ret;

	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);
	plane = kunit_kzalloc(test, sizeof(*plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, plane);
	old_plane_state = kunit_kzalloc(test, sizeof(*old_plane_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_plane_state);
	new_plane_state = kunit_kzalloc(test, sizeof(*new_plane_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, new_plane_state);

	acrtc->base.dev = &adev->ddev;
	old_plane_state->crtc = &acrtc->base;
	new_plane_state->crtc = &acrtc->base;
	new_plane_state->src_x = 1;
	plane->state = old_plane_state;

	ret = amdgpu_dm_check_native_cursor_state(&acrtc->base, plane,
						  new_plane_state, true);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/**
 * dm_test_check_native_cursor_state_checks_fb - Test framebuffer validation propagation
 * @test: The KUnit test context
 */
static void dm_test_check_native_cursor_state_checks_fb(struct kunit *test)
{
	struct dm_cursor_fb_fixture fixture = dm_test_alloc_cursor_fb_fixture(test);
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state;
	int ret;

	plane = kunit_kzalloc(test, sizeof(*plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, plane);
	old_plane_state = kunit_kzalloc(test, sizeof(*old_plane_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_plane_state);
	old_plane_state->crtc = &fixture.acrtc->base;
	fixture.plane_state->crtc = &fixture.acrtc->base;
	plane->state = old_plane_state;

	ret = amdgpu_dm_check_native_cursor_state(&fixture.acrtc->base, plane,
						  fixture.plane_state, true);
	KUNIT_EXPECT_EQ(test, ret, 0);

	fixture.afb->base.width = fixture.acrtc->max_cursor_width + 1;
	ret = amdgpu_dm_check_native_cursor_state(&fixture.acrtc->base, plane,
						  fixture.plane_state, true);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/* Tests for dm_plane_color_pipeline_active() */

/**
 * dm_test_plane_color_pipeline_active - Test old and new colorop activity
 * @test: The KUnit test context
 */
static void dm_test_plane_color_pipeline_active(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state;
	struct drm_colorop *colorop;
	struct drm_colorop_state *old_colorop_state;
	struct drm_colorop_state *new_colorop_state;
	struct drm_plane *plane;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);
	colorop = kunit_kzalloc(test, sizeof(*colorop), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, colorop);
	old_colorop_state = kunit_kzalloc(test, sizeof(*old_colorop_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_colorop_state);
	new_colorop_state = kunit_kzalloc(test, sizeof(*new_colorop_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, new_colorop_state);
	plane = kunit_kzalloc(test, sizeof(*plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, plane);
	state->colorops = kunit_kzalloc(test, sizeof(*state->colorops), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->colorops);

	adev->ddev.mode_config.num_colorop = 1;
	state->dev = &adev->ddev;
	state->colorops[0].ptr = colorop;
	state->colorops[0].old_state = old_colorop_state;
	state->colorops[0].new_state = new_colorop_state;
	colorop->plane = plane;
	old_colorop_state->colorop = colorop;
	old_colorop_state->bypass = true;
	new_colorop_state->colorop = colorop;
	new_colorop_state->bypass = false;

	KUNIT_EXPECT_FALSE(test, dm_plane_color_pipeline_active(state, plane, true));
	KUNIT_EXPECT_TRUE(test, dm_plane_color_pipeline_active(state, plane, false));
}

/**
 * dm_test_plane_color_pipeline_ignores_other_plane - Test unrelated colorops are ignored
 * @test: The KUnit test context
 */
static void dm_test_plane_color_pipeline_ignores_other_plane(struct kunit *test)
{
	struct amdgpu_device *adev = dm_kunit_alloc_adev(test);
	struct drm_atomic_commit *state;
	struct drm_colorop *colorop;
	struct drm_colorop_state *colorop_state;
	struct drm_plane *colorop_plane;
	struct drm_plane *other_plane;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);
	colorop = kunit_kzalloc(test, sizeof(*colorop), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, colorop);
	colorop_state = kunit_kzalloc(test, sizeof(*colorop_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, colorop_state);
	colorop_plane = kunit_kzalloc(test, sizeof(*colorop_plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, colorop_plane);
	other_plane = kunit_kzalloc(test, sizeof(*other_plane), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, other_plane);
	state->colorops = kunit_kzalloc(test, sizeof(*state->colorops), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->colorops);

	adev->ddev.mode_config.num_colorop = 1;
	state->dev = &adev->ddev;
	state->colorops[0].ptr = colorop;
	state->colorops[0].new_state = colorop_state;
	colorop->plane = colorop_plane;
	colorop_state->colorop = colorop;

	KUNIT_EXPECT_FALSE(test, dm_plane_color_pipeline_active(state, other_plane, false));
}

/* Tests for amdgpu_dm_should_update_native_cursor() */

/**
 * dm_test_should_update_native_cursor_without_crtc - Test NULL crtc cases update native cursor
 * @test: The KUnit test context
 */
static void dm_test_should_update_native_cursor_without_crtc(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, amdgpu_dm_should_update_native_cursor(NULL, NULL, NULL, false));
	KUNIT_EXPECT_TRUE(test, amdgpu_dm_should_update_native_cursor(NULL, NULL, NULL, true));
}

/**
 * dm_test_should_update_native_cursor_disable_native - Test disable path reads old crtc cursor mode
 * @test: The KUnit test context
 */
static void dm_test_should_update_native_cursor_disable_native(struct kunit *test)
{
	struct dm_crtc_state *dm_crtc_state;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);

	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);

	dm_crtc_state = kunit_kzalloc(test, sizeof(*dm_crtc_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_crtc_state);

	state->crtcs = kunit_kzalloc(test, sizeof(*state->crtcs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->crtcs);

	crtc->index = 0;
	dm_crtc_state->cursor_mode = DM_CURSOR_NATIVE_MODE;
	state->crtcs[0].old_state = &dm_crtc_state->base;

	KUNIT_EXPECT_TRUE(test,
			  amdgpu_dm_should_update_native_cursor(state, crtc, NULL, false));
}

/**
 * dm_test_should_update_native_cursor_enable_overlay - Test enable path reads new crtc cursor mode
 * @test: The KUnit test context
 */
static void dm_test_should_update_native_cursor_enable_overlay(struct kunit *test)
{
	struct dm_crtc_state *dm_crtc_state;
	struct drm_atomic_commit *state;
	struct drm_crtc *crtc;

	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state);

	crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, crtc);

	dm_crtc_state = kunit_kzalloc(test, sizeof(*dm_crtc_state), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_crtc_state);

	state->crtcs = kunit_kzalloc(test, sizeof(*state->crtcs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->crtcs);

	crtc->index = 0;
	dm_crtc_state->cursor_mode = DM_CURSOR_OVERLAY_MODE;
	state->crtcs[0].new_state = &dm_crtc_state->base;

	KUNIT_EXPECT_FALSE(test,
			   amdgpu_dm_should_update_native_cursor(state, NULL, crtc, true));
}

/* Tests for dm_get_oriented_plane_size() */

/**
 * dm_test_oriented_plane_size_rotate_0 - Test Oriented plane size rotate 0
 * @test: The KUnit test context
 */
static void dm_test_oriented_plane_size_rotate_0(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int src_w = 0;
	int src_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_0;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;

	dm_get_oriented_plane_size(&plane_state, &src_w, &src_h);

	KUNIT_EXPECT_EQ(test, src_w, 1920);
	KUNIT_EXPECT_EQ(test, src_h, 1080);
}

/**
 * dm_test_oriented_plane_size_rotate_90 - Test Oriented plane size rotate 90
 * @test: The KUnit test context
 */
static void dm_test_oriented_plane_size_rotate_90(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int src_w = 0;
	int src_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_90;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;

	dm_get_oriented_plane_size(&plane_state, &src_w, &src_h);

	KUNIT_EXPECT_EQ(test, src_w, 1080);
	KUNIT_EXPECT_EQ(test, src_h, 1920);
}

/**
 * dm_test_oriented_plane_size_rotate_180 - Test Oriented plane size rotate 180
 * @test: The KUnit test context
 */
static void dm_test_oriented_plane_size_rotate_180(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int src_w = 0;
	int src_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_180;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;

	dm_get_oriented_plane_size(&plane_state, &src_w, &src_h);

	KUNIT_EXPECT_EQ(test, src_w, 1920);
	KUNIT_EXPECT_EQ(test, src_h, 1080);
}

/**
 * dm_test_oriented_plane_size_rotate_270 - Test Oriented plane size rotate 270
 * @test: The KUnit test context
 */
static void dm_test_oriented_plane_size_rotate_270(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int src_w = 0;
	int src_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_270;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;

	dm_get_oriented_plane_size(&plane_state, &src_w, &src_h);

	KUNIT_EXPECT_EQ(test, src_w, 1080);
	KUNIT_EXPECT_EQ(test, src_h, 1920);
}

/* Tests for dm_get_plane_scale() */

/**
 * dm_test_get_plane_scale_identity - Test Get plane scale identity
 * @test: The KUnit test context
 */
static void dm_test_get_plane_scale_identity(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int scale_w = 0;
	int scale_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_0;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;
	plane_state.crtc_w = 1920;
	plane_state.crtc_h = 1080;

	dm_get_plane_scale(&plane_state, &scale_w, &scale_h);

	KUNIT_EXPECT_EQ(test, scale_w, 1000);
	KUNIT_EXPECT_EQ(test, scale_h, 1000);
}

/**
 * dm_test_get_plane_scale_rotate_90_identity - Test Get plane scale rotate 90 identity
 * @test: The KUnit test context
 */
static void dm_test_get_plane_scale_rotate_90_identity(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int scale_w = 0;
	int scale_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_90;
	plane_state.src_w = 1920 << 16;
	plane_state.src_h = 1080 << 16;
	plane_state.crtc_w = 1080;
	plane_state.crtc_h = 1920;

	dm_get_plane_scale(&plane_state, &scale_w, &scale_h);

	KUNIT_EXPECT_EQ(test, scale_w, 1000);
	KUNIT_EXPECT_EQ(test, scale_h, 1000);
}

/**
 * dm_test_get_plane_scale_zero_src_width - Test Get plane scale zero src width
 * @test: The KUnit test context
 */
static void dm_test_get_plane_scale_zero_src_width(struct kunit *test)
{
	struct drm_plane_state plane_state = { 0 };
	int scale_w = 0;
	int scale_h = 0;

	plane_state.rotation = DRM_MODE_ROTATE_0;
	plane_state.src_w = 0;
	plane_state.src_h = 1080 << 16;
	plane_state.crtc_w = 100;
	plane_state.crtc_h = 200;

	dm_get_plane_scale(&plane_state, &scale_w, &scale_h);

	KUNIT_EXPECT_EQ(test, scale_w, 0);
	KUNIT_EXPECT_EQ(test, scale_h, 185);
}

static struct kunit_case amdgpu_dm_cursor_tests[] = {
	/* dm_check_cursor_fb */
	KUNIT_CASE(dm_test_check_cursor_fb_valid_linear),
	KUNIT_CASE(dm_test_check_cursor_fb_rejects_size),
	KUNIT_CASE(dm_test_check_cursor_fb_rejects_cropping),
	KUNIT_CASE(dm_test_check_cursor_fb_rejects_pitch),
	KUNIT_CASE(dm_test_check_cursor_fb_rejects_unsupported_pitch),
	KUNIT_CASE(dm_test_check_cursor_fb_rejects_tiling),
	KUNIT_CASE(dm_test_check_cursor_fb_gfx12_tiling),
	KUNIT_CASE(dm_test_check_cursor_fb_pre_ai_tiling),
	KUNIT_CASE(dm_test_check_cursor_fb_modifier_skips_tiling),
	/* amdgpu_dm_check_native_cursor_state */
	KUNIT_CASE(dm_test_check_native_cursor_state_disabled),
	KUNIT_CASE(dm_test_check_native_cursor_state_rejects_offset),
	KUNIT_CASE(dm_test_check_native_cursor_state_checks_fb),
	/* dm_plane_color_pipeline_active */
	KUNIT_CASE(dm_test_plane_color_pipeline_active),
	KUNIT_CASE(dm_test_plane_color_pipeline_ignores_other_plane),
	/* amdgpu_dm_should_update_native_cursor */
	KUNIT_CASE(dm_test_should_update_native_cursor_without_crtc),
	KUNIT_CASE(dm_test_should_update_native_cursor_disable_native),
	KUNIT_CASE(dm_test_should_update_native_cursor_enable_overlay),
	/* dm_get_oriented_plane_size */
	KUNIT_CASE(dm_test_oriented_plane_size_rotate_0),
	KUNIT_CASE(dm_test_oriented_plane_size_rotate_90),
	KUNIT_CASE(dm_test_oriented_plane_size_rotate_180),
	KUNIT_CASE(dm_test_oriented_plane_size_rotate_270),
	/* dm_get_plane_scale */
	KUNIT_CASE(dm_test_get_plane_scale_identity),
	KUNIT_CASE(dm_test_get_plane_scale_rotate_90_identity),
	KUNIT_CASE(dm_test_get_plane_scale_zero_src_width),
	{}
};

static struct kunit_suite amdgpu_dm_cursor_test_suite = {
	.name = "amdgpu_dm_cursor",
	.test_cases = amdgpu_dm_cursor_tests,
};

kunit_test_suite(amdgpu_dm_cursor_test_suite);

MODULE_AUTHOR("AMD");
MODULE_DESCRIPTION("KUnit tests for amdgpu_dm_cursor");
MODULE_LICENSE("Dual MIT/GPL");
