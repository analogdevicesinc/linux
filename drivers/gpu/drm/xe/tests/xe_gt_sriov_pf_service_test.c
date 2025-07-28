// SPDX-License-Identifier: GPL-2.0 AND MIT
/*
 * Copyright © 2024 Intel Corporation
 */

#include <kunit/test.h>

#include "xe_device.h"
#include "xe_kunit_helpers.h"
#include "xe_pci_test.h"

static int pf_service_test_init(struct kunit *test)
{
	struct xe_pci_fake_data fake = {
		.sriov_mode = XE_SRIOV_MODE_PF,
		.platform = XE_TIGERLAKE, /* some random platform */
		.subplatform = XE_SUBPLATFORM_NONE,
	};
	struct xe_device *xe;
	struct xe_gt *gt;

	test->priv = &fake;
	xe_kunit_helper_xe_device_test_init(test);

	xe = test->priv;
	KUNIT_ASSERT_EQ(test, xe_sriov_init(xe), 0);

	gt = xe_device_get_gt(xe, 0);
	pf_init_versions(gt);

	/*
	 * sanity check:
	 * - all supported platforms VF/PF ABI versions must be defined
	 * - base version can't be newer than latest
	 */
	KUNIT_ASSERT_NE(test, 0, gt->sriov.pf.service.version.base.major);
	KUNIT_ASSERT_NE(test, 0, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_LE(test, gt->sriov.pf.service.version.base.major,
			gt->sriov.pf.service.version.latest.major);
	if (gt->sriov.pf.service.version.base.major == gt->sriov.pf.service.version.latest.major)
		KUNIT_ASSERT_LE(test, gt->sriov.pf.service.version.base.minor,
				gt->sriov.pf.service.version.latest.minor);

	test->priv = gt;
	return 0;
}

static void pf_negotiate_any(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt, VF2PF_HANDSHAKE_MAJOR_ANY,
					     VF2PF_HANDSHAKE_MINOR_ANY,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.latest.minor);
}

static void pf_negotiate_base_match(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.base.major,
					     gt->sriov.pf.service.version.base.minor,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.base.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.base.minor);
}

static void pf_negotiate_base_newer(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.base.major,
					     gt->sriov.pf.service.version.base.minor + 1,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.base.major);
	KUNIT_ASSERT_GE(test, minor, gt->sriov.pf.service.version.base.minor);
	if (gt->sriov.pf.service.version.base.major == gt->sriov.pf.service.version.latest.major)
		KUNIT_ASSERT_LE(test, minor, gt->sriov.pf.service.version.latest.minor);
	else
		KUNIT_FAIL(test, "FIXME: don't know how to test multi-version yet!\n");
}

static void pf_negotiate_base_next(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.base.major + 1, 0,
					     &major, &minor));
	KUNIT_ASSERT_GE(test, major, gt->sriov.pf.service.version.base.major);
	KUNIT_ASSERT_LE(test, major, gt->sriov.pf.service.version.latest.major);
	if (major == gt->sriov.pf.service.version.latest.major)
		KUNIT_ASSERT_LE(test, minor, gt->sriov.pf.service.version.latest.minor);
	else
		KUNIT_FAIL(test, "FIXME: don't know how to test multi-version yet!\n");
}

static void pf_negotiate_base_older(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	if (!gt->sriov.pf.service.version.base.minor)
		kunit_skip(test, "no older minor\n");

	KUNIT_ASSERT_NE(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.base.major,
					     gt->sriov.pf.service.version.base.minor - 1,
					     &major, &minor));
}

static void pf_negotiate_base_prev(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_NE(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.base.major - 1, 1,
					     &major, &minor));
}

static void pf_negotiate_latest_match(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.latest.major,
					     gt->sriov.pf.service.version.latest.minor,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.latest.minor);
}

static void pf_negotiate_latest_newer(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.latest.major,
					     gt->sriov.pf.service.version.latest.minor + 1,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.latest.minor);
}

static void pf_negotiate_latest_next(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.latest.major + 1, 0,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.latest.minor);
}

static void pf_negotiate_latest_older(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	if (!gt->sriov.pf.service.version.latest.minor)
		kunit_skip(test, "no older minor\n");

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.latest.major,
					     gt->sriov.pf.service.version.latest.minor - 1,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major);
	KUNIT_ASSERT_EQ(test, minor, gt->sriov.pf.service.version.latest.minor - 1);
}

static void pf_negotiate_latest_prev(struct kunit *test)
{
	struct xe_gt *gt = test->priv;
	u32 major, minor;

	if (gt->sriov.pf.service.version.base.major == gt->sriov.pf.service.version.latest.major)
		kunit_skip(test, "no prev major");

	KUNIT_ASSERT_EQ(test, 0,
			pf_negotiate_version(gt,
					     gt->sriov.pf.service.version.latest.major - 1,
					     gt->sriov.pf.service.version.base.minor + 1,
					     &major, &minor));
	KUNIT_ASSERT_EQ(test, major, gt->sriov.pf.service.version.latest.major - 1);
	KUNIT_ASSERT_GE(test, major, gt->sriov.pf.service.version.base.major);
}

static struct kunit_case pf_service_test_cases[] = {
	KUNIT_CASE(pf_negotiate_any),
	KUNIT_CASE(pf_negotiate_base_match),
	KUNIT_CASE(pf_negotiate_base_newer),
	KUNIT_CASE(pf_negotiate_base_next),
	KUNIT_CASE(pf_negotiate_base_older),
	KUNIT_CASE(pf_negotiate_base_prev),
	KUNIT_CASE(pf_negotiate_latest_match),
	KUNIT_CASE(pf_negotiate_latest_newer),
	KUNIT_CASE(pf_negotiate_latest_next),
	KUNIT_CASE(pf_negotiate_latest_older),
	KUNIT_CASE(pf_negotiate_latest_prev),
	{}
};

static struct kunit_suite pf_service_suite = {
	.name = "pf_service",
	.test_cases = pf_service_test_cases,
	.init = pf_service_test_init,
};

kunit_test_suite(pf_service_suite);
