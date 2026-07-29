// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * KUnit tests for amdgpu_dm_cursor.c
 *
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include <kunit/test.h>
#include <drm/drm_atomic.h>
#include <drm/drm_blend.h>
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
