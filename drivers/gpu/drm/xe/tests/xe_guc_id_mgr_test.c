// SPDX-License-Identifier: GPL-2.0 AND MIT
/*
 * Copyright © 2024 Intel Corporation
 */

#include <kunit/test.h>

#include "xe_device.h"
#include "xe_kunit_helpers.h"
#include "xe_pci_test.h"

#define IDM_TEST_POOL_SIZE		8
#define IDM_TEST_PARTIAL_POOL_SIZE	((GUC_ID_MAX / 3) * 2)

static int guc_id_mgr_test_init(struct kunit *test)
{
	struct xe_pci_fake_data *fake;
	struct xe_guc_id_mgr *idm;

	/* Keep one logical suite: use PF fixture when IOV is enabled. */
	fake = kunit_kzalloc(test, sizeof(*fake), GFP_KERNEL);
	if (!fake)
		return -ENOMEM;

	*fake = (struct xe_pci_fake_data) {
		.sriov_mode = XE_SRIOV_MODE_PF,
		.platform = XE_TIGERLAKE, /* some random platform */
		.subplatform = XE_SUBPLATFORM_NONE,
	};
	test->priv = fake;

	xe_kunit_helper_xe_device_test_init(test);
	idm = &xe_root_mmio_gt(test->priv)->uc.guc.submission_state.idm;

	mutex_init(idm_mutex(idm));
	test->priv = idm;
	return 0;
}

static void bad_init(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;

	KUNIT_EXPECT_EQ(test, -EINVAL, idm_init(idm, 0, 0));
	KUNIT_EXPECT_EQ(test, -ERANGE, idm_init(idm, GUC_ID_MAX + 1, 0));
}

static void no_init(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;

	scoped_guard(mutex, idm_mutex(idm))
		KUNIT_EXPECT_EQ(test, -ENODATA, xe_guc_id_mgr_reserve_usable_locked(idm, 0));

	KUNIT_EXPECT_EQ(test, -ENODATA, xe_guc_id_mgr_reserve_shareable(idm, 1, 1));
}

static void init_fini(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, GUC_ID_MAX, 0));
	KUNIT_EXPECT_NOT_NULL(test, idm->bitmap);
	KUNIT_EXPECT_EQ(test, idm_total(idm), GUC_ID_MAX);
	__fini_idm(NULL, idm);
	KUNIT_EXPECT_NULL(test, idm->bitmap);
	KUNIT_EXPECT_EQ(test, idm_total(idm), 0);
}

static void check_init_small(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	unsigned int count = 8;

	KUNIT_ASSERT_EQ(test, xe_guc_id_mgr_init_small(idm, count), 0);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_usable(idm), count);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_shareable(idm), 0);
	KUNIT_EXPECT_EQ(test, idm_total(idm), count);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_usable(idm, count), 0);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_usable(idm, 1), -ENOSPC);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable(idm, 0, count), 0);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_shareable(idm, 1, 0), -ENODATA);
}

static void require_iov_config_or_skip(struct kunit *test)
{
	if (!IS_ENABLED(CONFIG_PCI_IOV))
		kunit_skip(test, "requires CONFIG_PCI_IOV");
}

static void check_init_shared(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_TRUE(test, IS_SRIOV_PF(idm_to_xe(idm)));
	KUNIT_ASSERT_EQ(test, xe_guc_id_mgr_init_shared(idm), 0);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_usable(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_shareable(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, idm_total(idm), GUC_ID_MAX);
}

static unsigned int test_idm_used_total(struct xe_guc_id_mgr *idm)
{
	lockdep_assert_held(idm_mutex(idm));

	return bitmap_weight(idm->bitmap, idm_total(idm));
}

static void check_used(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	unsigned int n;
	int id;

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, 2, 0));

	guard(mutex)(idm_mutex(idm));

	for (n = 0; n < idm_total(idm); n++) {
		kunit_info(test, "n=%u", n);
		KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), n);
		id = xe_guc_id_mgr_reserve_usable_locked(idm, 1);
		KUNIT_EXPECT_GE(test, id, 0);
		if (id < 0)
			break;
		KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), n + 1);
	}
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), idm_total(idm));
	if (n)
		idm_release_chunk_locked(idm, 0, n);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_quota(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int overlap = 10;
	const unsigned int tail = 10;
	const unsigned int usable = GUC_ID_MAX - tail;
	const unsigned int shareable = overlap + tail;
	const unsigned int spare = overlap + 1;
	int usable_id, tail_id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, usable, shareable));

	guard(mutex)(idm_mutex(idm));

	/* spare is larger than the overlap, but the PF-exclusive range is free. */
	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, usable - spare);
	KUNIT_EXPECT_GE(test, usable_id, 0);
	if (usable_id < 0)
		return;

	tail_id = xe_guc_id_mgr_reserve_shareable_locked(idm, tail, spare);
	KUNIT_EXPECT_EQ(test, tail_id, usable);
	if (tail_id < 0) {
		idm_release_chunk_locked(idm, usable_id, usable - spare);
		return;
	}

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_reserve_shareable_locked(idm, 1, spare), -EDQUOT);

	idm_release_chunk_locked(idm, usable_id, usable - spare);
	idm_release_chunk_locked(idm, tail_id, tail);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_all(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	unsigned int n;
	int id;

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, GUC_ID_MAX, 0));

	guard(mutex)(idm_mutex(idm));

	for (n = 0; n < idm_total(idm); n++) {
		id = xe_guc_id_mgr_reserve_usable_locked(idm, 1);
		KUNIT_EXPECT_GE(test, id, 0);
		if (id < 0)
			break;
	}
	if (n < idm_total(idm)) {
		if (n)
			idm_release_chunk_locked(idm, 0, n);
		return;
	}

	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), idm_total(idm));
	for (n = 0; n < idm_total(idm); n++)
		idm_release_chunk_locked(idm, n, 1);
}

static void check_limits(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int usable = 10;
	const unsigned int shareable = 20;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, usable, shareable));

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_usable(idm), usable);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_shareable(idm), shareable);
	KUNIT_EXPECT_EQ(test, idm_total(idm), usable + shareable);
}

static void check_overlap(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, GUC_ID_MAX, GUC_ID_MAX));

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_usable(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_shareable(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, idm_total(idm), GUC_ID_MAX);
}

static void check_overlap_alloc(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int usable_count = 2;
	const unsigned int shareable_count = 3;
	unsigned int total;
	int usable_id, shareable_id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, GUC_ID_MAX, GUC_ID_MAX));

	guard(mutex)(idm_mutex(idm));
	total = idm_total(idm);

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, usable_count);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0)
		return;

	shareable_id = xe_guc_id_mgr_reserve_shareable_locked(idm, shareable_count, 0);
	KUNIT_EXPECT_EQ(test, shareable_id, total - shareable_count);
	if (shareable_id < 0) {
		idm_release_chunk_locked(idm, usable_id, usable_count);
		return;
	}

	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), usable_count + shareable_count);
	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), usable_count + shareable_count);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), usable_count + shareable_count);

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_usable_locked(idm, usable_id, usable_count), 0);
	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_id,
							       shareable_count), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_overlap_release(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int count = 4;
	unsigned int total;
	int id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, GUC_ID_MAX, GUC_ID_MAX));

	guard(mutex)(idm_mutex(idm));
	total = idm_total(idm);

	id = xe_guc_id_mgr_reserve_usable_locked(idm, count);
	KUNIT_EXPECT_EQ(test, id, 0);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_shareable_locked(idm, id, count), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);

	id = xe_guc_id_mgr_reserve_shareable_locked(idm, count, 0);
	KUNIT_EXPECT_EQ(test, id, total - count);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable_locked(idm, id, count), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable_locked(idm, 0, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_shareable_locked(idm, 0, 0), -EINVAL);
}

static void check_overlap_partial_init(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int pools_size = IDM_TEST_PARTIAL_POOL_SIZE;
	unsigned int shareable_start;
	unsigned int rest;
	int usable_id, shareable_id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, pools_size, pools_size));

	guard(mutex)(idm_mutex(idm));

	shareable_start = idm_shareable_start(idm);
	rest = idm_total(idm) - pools_size;

	KUNIT_EXPECT_EQ(test, idm_total(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_usable(idm), pools_size);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_max_shareable(idm), pools_size);
	KUNIT_EXPECT_EQ(test, shareable_start, GUC_ID_MAX - pools_size);

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, pools_size);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0)
		return;

	shareable_id = xe_guc_id_mgr_reserve_shareable_locked(idm, rest, 0);
	KUNIT_EXPECT_EQ(test, shareable_id, pools_size);
	if (shareable_id < 0) {
		idm_release_chunk_locked(idm, usable_id, pools_size);
		return;
	}

	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), GUC_ID_MAX);
	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), pools_size);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), pools_size);

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_usable_locked(idm, usable_id, pools_size), 0);
	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_id, rest), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_overlap_partial_shared_range_blocking(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int pools_size = IDM_TEST_PARTIAL_POOL_SIZE;
	int usable_id, shareable_range_id, shareable_overlap_id;
	unsigned int shareable_start;
	unsigned int nonoverlap_size;
	unsigned int overlap_size;
	unsigned int total;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, pools_size, pools_size));

	guard(mutex)(idm_mutex(idm));

	total = idm_total(idm);
	shareable_start = idm_shareable_start(idm);
	nonoverlap_size = total - pools_size;
	overlap_size = pools_size - shareable_start;

	KUNIT_EXPECT_GT(test, nonoverlap_size, 0);
	KUNIT_EXPECT_GT(test, overlap_size, 0);

	shareable_range_id = xe_guc_id_mgr_reserve_shareable_locked(idm, nonoverlap_size, 0);
	KUNIT_EXPECT_EQ(test, shareable_range_id, pools_size);
	if (shareable_range_id < 0)
		return;

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, pools_size);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0) {
		idm_release_chunk_locked(idm, shareable_range_id, nonoverlap_size);
		return;
	}

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_shareable_locked(idm, 1, 0), -ENOSPC);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable_locked(idm, usable_id, pools_size), 0);

	shareable_overlap_id = xe_guc_id_mgr_reserve_shareable_locked(idm, overlap_size, 0);
	KUNIT_EXPECT_EQ(test, shareable_overlap_id, shareable_start);
	if (shareable_overlap_id < 0) {
		idm_release_chunk_locked(idm, shareable_range_id, nonoverlap_size);
		return;
	}

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, shareable_start);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0) {
		idm_release_chunk_locked(idm, shareable_overlap_id, overlap_size);
		idm_release_chunk_locked(idm, shareable_range_id, nonoverlap_size);
		return;
	}
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_usable_locked(idm, 1), -ENOSPC);

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_usable_locked(idm, usable_id, shareable_start),
			0);
	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_overlap_id,
							       overlap_size),
			0);
	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_range_id,
							       nonoverlap_size),
			0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_used_usable(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int count = 2;
	int id;

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, IDM_TEST_POOL_SIZE, 0));

	guard(mutex)(idm_mutex(idm));

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), 0);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), 0);

	id = xe_guc_id_mgr_reserve_usable_locked(idm, count);
	KUNIT_EXPECT_GE(test, id, 0);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), count);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), count);

	idm_release_chunk_locked(idm, id, count);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_used_shareable(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int count = 3;
	unsigned int shareable_start;
	int id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, IDM_TEST_POOL_SIZE, IDM_TEST_POOL_SIZE));

	guard(mutex)(idm_mutex(idm));
	shareable_start = idm_shareable_start(idm);

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), 0);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), 0);

	id = xe_guc_id_mgr_reserve_shareable_locked(idm, count, 0);
	KUNIT_EXPECT_EQ(test, id, shareable_start);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), 0);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), count);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), count);

	idm_release_chunk_locked(idm, id, count);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_used_mixed(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int usable_count = 2;
	const unsigned int shareable_count = 3;
	unsigned int shareable_start;
	int usable_id, shareable_id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, IDM_TEST_POOL_SIZE, IDM_TEST_POOL_SIZE));

	guard(mutex)(idm_mutex(idm));
	shareable_start = idm_shareable_start(idm);

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, usable_count);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0)
		return;

	shareable_id = xe_guc_id_mgr_reserve_shareable_locked(idm, shareable_count, 0);
	KUNIT_EXPECT_EQ(test, shareable_id, shareable_start);
	if (shareable_id < 0) {
		idm_release_chunk_locked(idm, usable_id, usable_count);
		return;
	}

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), usable_count);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), shareable_count);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), usable_count + shareable_count);

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_usable_locked(idm, usable_id, usable_count), 0);
	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_id,
							       shareable_count), 0);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_release_usable_range(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int count = 4;
	int id;

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, IDM_TEST_POOL_SIZE, 0));

	guard(mutex)(idm_mutex(idm));

	id = xe_guc_id_mgr_reserve_usable_locked(idm, count);
	KUNIT_EXPECT_EQ(test, id, 0);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_usable_locked(idm, count,
							    IDM_TEST_POOL_SIZE - count + 1),
			-ERANGE);
	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), count);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable_locked(idm, 0, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), count);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_usable_locked(idm, id, count), 0);
	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), 0);
}

static void check_release_shareable_range(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int count = 4;
	unsigned int shareable_start;
	int id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, IDM_TEST_POOL_SIZE, IDM_TEST_POOL_SIZE));

	guard(mutex)(idm_mutex(idm));

	shareable_start = idm_shareable_start(idm);
	id = xe_guc_id_mgr_reserve_shareable_locked(idm, count, 0);
	KUNIT_EXPECT_EQ(test, id, shareable_start);
	if (id < 0)
		return;

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_start - 1, count),
			-ERANGE);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), count);

	KUNIT_EXPECT_EQ(test,
			xe_guc_id_mgr_release_shareable_locked(idm, shareable_start, 0),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), count);

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_release_shareable_locked(idm, id, count), 0);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), 0);
}

static void check_split_boundaries(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int pool_size = IDM_TEST_POOL_SIZE;
	unsigned int shareable_start;
	int usable_id, shareable_id;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, pool_size, pool_size));

	guard(mutex)(idm_mutex(idm));

	shareable_start = idm_shareable_start(idm);

	usable_id = xe_guc_id_mgr_reserve_usable_locked(idm, pool_size);
	KUNIT_EXPECT_EQ(test, usable_id, 0);
	if (usable_id < 0)
		return;
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_usable_locked(idm, 1), -ENOSPC);

	shareable_id = xe_guc_id_mgr_reserve_shareable_locked(idm, pool_size, 0);
	KUNIT_EXPECT_EQ(test, shareable_id, shareable_start);
	if (shareable_id < 0) {
		idm_release_chunk_locked(idm, usable_id, pool_size);
		return;
	}
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_shareable_locked(idm, 1, 0), -ENOSPC);

	KUNIT_EXPECT_EQ(test, idm_used_usable(idm), pool_size);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), pool_size);

	idm_release_chunk_locked(idm, usable_id, pool_size);
	idm_release_chunk_locked(idm, shareable_id, pool_size);
	KUNIT_EXPECT_EQ(test, test_idm_used_total(idm), 0);
}

static void check_shareable_spare(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int pools_size = IDM_TEST_PARTIAL_POOL_SIZE;
	unsigned int shareable_start;
	unsigned int overlap_size;
	unsigned int shared_tail;
	int id_tail, id_overlap, id_extra;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, pools_size, pools_size));

	guard(mutex)(idm_mutex(idm));
	shareable_start = idm_shareable_start(idm);
	overlap_size = idm->usable - shareable_start;
	shared_tail = idm_total(idm) - idm->usable;

	KUNIT_EXPECT_GT(test, overlap_size, 0);
	KUNIT_EXPECT_GT(test, shared_tail, 0);

	id_tail = xe_guc_id_mgr_reserve_shareable_locked(idm, shared_tail, 1);
	KUNIT_EXPECT_EQ(test, id_tail, idm->usable);
	if (id_tail < 0)
		return;
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), shared_tail);

	id_overlap = xe_guc_id_mgr_reserve_shareable_locked(idm, overlap_size - 1, 1);
	KUNIT_EXPECT_EQ(test, id_overlap, shareable_start + 1);
	if (id_overlap < 0) {
		idm_release_chunk_locked(idm, id_tail, shared_tail);
		return;
	}
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), idm->shareable - 1);

	id_extra = xe_guc_id_mgr_reserve_shareable_locked(idm, 1, 1);
	KUNIT_EXPECT_EQ(test, id_extra, shareable_start);
	if (id_extra < 0) {
		idm_release_chunk_locked(idm, id_tail, shared_tail);
		idm_release_chunk_locked(idm, id_overlap, overlap_size - 1);
		return;
	}
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), idm->shareable);

	idm_release_chunk_locked(idm, id_tail, shared_tail);
	idm_release_chunk_locked(idm, id_overlap, overlap_size - 1);
	idm_release_chunk_locked(idm, id_extra, 1);
	KUNIT_EXPECT_EQ(test, idm_used_shareable(idm), 0);
}

static void check_shareable_spare_fragmented_tail(struct kunit *test)
{
	struct xe_guc_id_mgr *idm = test->priv;
	const unsigned int overlap = 40;
	const unsigned int tail = 50;
	const unsigned int usable = GUC_ID_MAX - tail;
	const unsigned int shareable = overlap + tail;
	const unsigned int stride = 5;
	const unsigned int count = overlap - 1;
	const unsigned int spare = 10;
	const unsigned int overlap_free = 30;
	const unsigned int tail_free = 20;
	unsigned int shareable_start;
	unsigned int i;

	require_iov_config_or_skip(test);

	KUNIT_ASSERT_EQ(test, 0, idm_init(idm, usable, shareable));

	guard(mutex)(idm_mutex(idm));
	shareable_start = idm_shareable_start(idm);
	KUNIT_EXPECT_EQ(test, idm->usable - shareable_start, overlap);

	/*
	 * Fragmented tail cannot fit @count IDs, so the request must fall back
	 * to the overlap, where it is limited by @spare.
	 */
	bitmap_fill(idm->bitmap, idm_total(idm));
	bitmap_clear(idm->bitmap, shareable_start + 1, count);
	for (i = idm->usable; i < idm_total(idm); i++) {
		if ((i - idm->usable) % stride)
			bitmap_clear(idm->bitmap, i, 1);
	}

	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_shareable_locked(idm, count, spare),
			-EDQUOT);

	bitmap_fill(idm->bitmap, idm_total(idm));
	bitmap_clear(idm->bitmap, shareable_start, overlap_free);
	bitmap_clear(idm->bitmap, idm_total(idm) - tail_free, tail_free);
	KUNIT_EXPECT_EQ(test, xe_guc_id_mgr_reserve_shareable_locked(idm, overlap_free, 1),
			-EDQUOT);

	bitmap_zero(idm->bitmap, idm_total(idm));
}

static struct kunit_case guc_id_mgr_test_cases[] = {
	KUNIT_CASE(bad_init),
	KUNIT_CASE(no_init),
	KUNIT_CASE(init_fini),
	KUNIT_CASE(check_init_small),
	KUNIT_CASE(check_init_shared),
	KUNIT_CASE(check_used),
	KUNIT_CASE(check_quota),
	KUNIT_CASE_SLOW(check_all),
	KUNIT_CASE(check_limits),
	KUNIT_CASE(check_overlap),
	KUNIT_CASE(check_overlap_alloc),
	KUNIT_CASE(check_overlap_release),
	KUNIT_CASE(check_overlap_partial_init),
	KUNIT_CASE(check_overlap_partial_shared_range_blocking),
	KUNIT_CASE(check_used_usable),
	KUNIT_CASE(check_used_shareable),
	KUNIT_CASE(check_used_mixed),
	KUNIT_CASE(check_release_usable_range),
	KUNIT_CASE(check_release_shareable_range),
	KUNIT_CASE(check_split_boundaries),
	KUNIT_CASE(check_shareable_spare),
	KUNIT_CASE(check_shareable_spare_fragmented_tail),
	{}
};

static struct kunit_suite guc_id_mgr_suite = {
	.name = "guc_idm",
	.test_cases = guc_id_mgr_test_cases,

	.init = guc_id_mgr_test_init,
	.exit = NULL,
};

kunit_test_suites(&guc_id_mgr_suite);
