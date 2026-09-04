// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * KUnit tests for amdgpu_dm_mst_types.c
 *
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include <kunit/test.h>

#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fixed.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_modeset_lock.h>
#include <drm/display/drm_dp.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dp_mst_helper.h>

#include "dc.h"
#include "dpcd_defs.h"
#include "dmub_cmd.h"
#include "amdgpu.h"
#include "amdgpu_mode.h"
#include "amdgpu_dm.h"
#include "amdgpu_dm_hdcp.h"
#include "amdgpu_dm_mst_types.h"
#include "amdgpu_dm_kunit_test_helpers.h"
#include "inc/link_service.h"

/*
 * Minimal mock DPCD backing store and AUX transfer callback used to exercise
 * the DPCD read paths without real hardware.
 */
static u8 dm_mst_test_dpcd[0x10];
static u8 dm_mst_test_desc_dpcd[0x10];
static struct aux_payload dm_mst_test_last_payload;
static int dm_mst_test_aux_transfer_raw_result;
static u8 dm_mst_test_aux_transfer_raw_reply;
static enum aux_return_code_type dm_mst_test_aux_transfer_raw_operation_result;
static ssize_t dm_mst_test_aux_transfer_override;
static ssize_t dm_mst_test_aux_write_override;

static int dm_mst_test_aux_transfer_raw(struct ddc_service *ddc,
						struct aux_payload *payload,
						enum aux_return_code_type *operation_result)
{
	size_t i;

	dm_mst_test_last_payload = *payload;
	*operation_result = dm_mst_test_aux_transfer_raw_operation_result;
	payload->reply[0] = dm_mst_test_aux_transfer_raw_reply;

	if (dm_mst_test_aux_transfer_raw_result)
		return dm_mst_test_aux_transfer_raw_result;

	if (payload->write)
		return 0;

	for (i = 0; i < payload->length; i++)
		payload->data[i] = dm_mst_test_dpcd[(payload->address + i) & 0xf];

	return payload->length;
}

static void dm_mst_test_setup_dm_aux(struct amdgpu_dm_dp_aux *dm_aux,
					    struct ddc_service *ddc,
					    struct dc_link *link,
					    struct dc *dc,
					    struct link_service *link_srv,
					    struct dc_context *ctx,
					    struct amdgpu_device *adev)
{
	memset(&dm_mst_test_last_payload, 0, sizeof(dm_mst_test_last_payload));
	dm_mst_test_aux_transfer_raw_result = 0;
	dm_mst_test_aux_transfer_raw_reply = 0;
	dm_mst_test_aux_transfer_raw_operation_result = AUX_RET_SUCCESS;
	link_srv->aux_transfer_raw = dm_mst_test_aux_transfer_raw;
	dc->link_srv = link_srv;
	link->dc = dc;
	ctx->driver_context = adev;
	ddc->link = link;
	ddc->ctx = ctx;
	dm_aux->ddc_service = ddc;
	dm_aux->aux.name = "dm_mst_test_dm_aux";
	dm_aux->aux.transfer = dm_dp_aux_transfer;
	drm_dp_aux_init(&dm_aux->aux);
	drm_dp_dpcd_set_probe(&dm_aux->aux, false);
}

static const struct dc_link_status *dm_mst_test_get_status(const struct dc_link *link)
{
	return &link->link_status;
}

static ssize_t dm_mst_test_aux_transfer(struct drm_dp_aux *aux,
					struct drm_dp_aux_msg *msg)
{
	size_t i;
	ssize_t ret;

	ret = dm_mst_test_aux_transfer_override;
	if (ret)
		return ret;

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_READ:
		for (i = 0; i < msg->size; i++)
			((u8 *)msg->buffer)[i] =
				dm_mst_test_dpcd[(msg->address + i) & 0xf];
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	case DP_AUX_NATIVE_WRITE:
		if (dm_mst_test_aux_write_override)
			return dm_mst_test_aux_write_override;
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		return msg->size;
	default:
		return -EINVAL;
	}
}

static struct amdgpu_dm_connector *dm_mst_test_alloc_sideband_connector(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct link_service *link_srv;
	struct dc_link *link;
	struct dc *dc;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);

	mutex_init(&aconnector->handle_mst_msg_ready);
	link_srv->get_status = dm_mst_test_get_status;
	dc->link_srv = link_srv;
	link->dc = dc;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_14;
	link->link_status.dpcd_caps = &link->dpcd_caps;
	aconnector->dc_link = link;
	aconnector->dm_dp_aux.aux.name = "dm_mst_test_sideband_aux";
	aconnector->dm_dp_aux.aux.transfer = dm_mst_test_aux_transfer;
	drm_dp_aux_init(&aconnector->dm_dp_aux.aux);
	drm_dp_dpcd_set_probe(&aconnector->dm_dp_aux.aux, false);

	memset(dm_mst_test_dpcd, 0, sizeof(dm_mst_test_dpcd));
	dm_mst_test_aux_transfer_override = 0;
	dm_mst_test_aux_write_override = 0;

	return aconnector;
}

static uint32_t dm_mst_test_dp_link_bandwidth_kbps(
	const struct dc_link *link,
	const struct dc_link_settings *link_settings)
{
	return 4320000;
}

static const struct dc_link_settings *dm_mst_test_dp_get_verified_link_cap(
	const struct dc_link *link)
{
	return &link->verified_link_cap;
}

static ssize_t dm_mst_test_desc_aux_transfer(struct drm_dp_aux *aux,
					     struct drm_dp_aux_msg *msg)
{
	size_t i;

	if ((msg->request & ~DP_AUX_I2C_MOT) != DP_AUX_NATIVE_READ)
		return -EINVAL;

	for (i = 0; i < msg->size; i++)
		((u8 *)msg->buffer)[i] = dm_mst_test_desc_dpcd[msg->address + i - DP_BRANCH_OUI];

	msg->reply = DP_AUX_NATIVE_REPLY_ACK;
	return msg->size;
}

/* Tests for needs_dsc_aux_workaround */

/**
 * dm_mst_test_needs_dsc_aux_workaround_match - Test workaround triggers for matching device
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns true when the link has
 * the specific branch device ID, DPCD rev 1.4, and sink count >= 2.
 */
static void dm_mst_test_needs_dsc_aux_workaround_match(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = DP_BRANCH_DEVICE_ID_90CC24;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_14;
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 2;

	KUNIT_EXPECT_TRUE(test, needs_dsc_aux_workaround(link));
}

/**
 * dm_mst_test_needs_dsc_aux_workaround_rev12 - Test workaround triggers for DPCD rev 1.2
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns true when the link has
 * the specific branch device ID, DPCD rev 1.2, and sink count >= 2.
 */
static void dm_mst_test_needs_dsc_aux_workaround_rev12(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = DP_BRANCH_DEVICE_ID_90CC24;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_12;
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 3;

	KUNIT_EXPECT_TRUE(test, needs_dsc_aux_workaround(link));
}

/**
 * dm_mst_test_needs_dsc_aux_workaround_wrong_dev_id - Test workaround skipped for wrong device
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns false when the branch
 * device ID does not match DP_BRANCH_DEVICE_ID_90CC24.
 */
static void dm_mst_test_needs_dsc_aux_workaround_wrong_dev_id(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = 0x123456;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_14;
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 2;

	KUNIT_EXPECT_FALSE(test, needs_dsc_aux_workaround(link));
}

/**
 * dm_mst_test_needs_dsc_aux_workaround_wrong_rev - Test workaround skipped for unsupported rev
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns false when the DPCD
 * revision is neither 1.2 nor 1.4.
 */
static void dm_mst_test_needs_dsc_aux_workaround_wrong_rev(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = DP_BRANCH_DEVICE_ID_90CC24;
	link->dpcd_caps.dpcd_rev.raw = 0x11; /* DPCD 1.1 */
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 2;

	KUNIT_EXPECT_FALSE(test, needs_dsc_aux_workaround(link));
}

/**
 * dm_mst_test_needs_dsc_aux_workaround_low_sink_count - Test workaround skipped for single sink
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns false when the sink
 * count is less than 2, even if device ID and DPCD rev match.
 */
static void dm_mst_test_needs_dsc_aux_workaround_low_sink_count(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = DP_BRANCH_DEVICE_ID_90CC24;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_14;
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 1;

	KUNIT_EXPECT_FALSE(test, needs_dsc_aux_workaround(link));
}

/**
 * dm_mst_test_needs_dsc_aux_workaround_zero_sink_count - Test workaround skipped for zero sinks
 * @test: KUnit test context
 *
 * Verify that needs_dsc_aux_workaround() returns false when the sink
 * count is zero, even if device ID and DPCD rev match.
 */
static void dm_mst_test_needs_dsc_aux_workaround_zero_sink_count(struct kunit *test)
{
	struct dc_link *link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, link);

	link->dpcd_caps.branch_dev_id = DP_BRANCH_DEVICE_ID_90CC24;
	link->dpcd_caps.dpcd_rev.raw = DPCD_REV_14;
	link->dpcd_caps.sink_count.bits.SINK_COUNT = 0;

	KUNIT_EXPECT_FALSE(test, needs_dsc_aux_workaround(link));
}

/* Tests for dm_mst_get_pbn_divider */

/**
 * dm_mst_test_pbn_divider_null_link - Test pbn_divider with NULL link
 * @test: KUnit test context
 *
 * Verify that dm_mst_get_pbn_divider() returns 0 when passed a NULL
 * link pointer without crashing.
 */
static void dm_mst_test_pbn_divider_null_link(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_mst_get_pbn_divider(NULL), 0U);
}

/**
 * dm_mst_test_pbn_divider_uses_link_bandwidth - Test pbn_divider with link cap
 * @test: KUnit test context
 *
 * Verify that dm_mst_get_pbn_divider() uses the DC link service to derive the
 * fixed-point PBN divider when a link is present.
 */
static void dm_mst_test_pbn_divider_uses_link_bandwidth(struct kunit *test)
{
	struct link_service *link_srv;
	struct dc_link *link;
	struct dc *dc;

	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);

	link_srv->dp_get_verified_link_cap = dm_mst_test_dp_get_verified_link_cap;
	link_srv->dp_link_bandwidth_kbps = dm_mst_test_dp_link_bandwidth_kbps;
	dc->link_srv = link_srv;
	link->dc = dc;

	KUNIT_EXPECT_EQ(test, dm_mst_get_pbn_divider(link),
			 (uint32_t)(dfixed_const(1000) / 100));
}

/* Tests for amdgpu_dm_mst_reset_mst_connector_setting */

/**
 * dm_mst_test_reset_connector_setting - Test MST connector setting reset
 * @test: KUnit test context
 *
 * Verify that amdgpu_dm_mst_reset_mst_connector_setting() clears the cached
 * EDID, DSC AUX, passthrough AUX, local bandwidth, and VC PBN state.
 */
static void dm_mst_test_reset_connector_setting(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_mst_port *port;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	port = kunit_kzalloc(test, sizeof(*port), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, port);

	aconnector->drm_edid = (const struct drm_edid *)test;
	aconnector->dsc_aux = (struct drm_dp_aux *)test;
	aconnector->mst_output_port = port;
	aconnector->mst_output_port->passthrough_aux = (struct drm_dp_aux *)test;
	aconnector->mst_local_bw = 12345;
	aconnector->vc_full_pbn = 678;

	amdgpu_dm_mst_reset_mst_connector_setting(aconnector);

	KUNIT_EXPECT_TRUE(test, aconnector->drm_edid == NULL);
	KUNIT_EXPECT_TRUE(test, aconnector->dsc_aux == NULL);
	KUNIT_EXPECT_TRUE(test, aconnector->mst_output_port->passthrough_aux == NULL);
	KUNIT_EXPECT_EQ(test, aconnector->mst_local_bw, 0U);
	KUNIT_EXPECT_EQ(test, aconnector->vc_full_pbn, 0U);
}

/* Tests for retrieve_downstream_port_device */

/**
 * dm_mst_test_retrieve_downstream_no_aux - Test retrieval bails out without AUX
 * @test: KUnit test context
 *
 * Verify that retrieve_downstream_port_device() returns false when the
 * connector has no DSC AUX channel and therefore cannot read DPCD.
 */
static void dm_mst_test_retrieve_downstream_no_aux(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);

	aconnector->dsc_aux = NULL;

	KUNIT_EXPECT_FALSE(test, retrieve_downstream_port_device(aconnector));
}

/**
 * dm_mst_test_retrieve_downstream_present - Test retrieval parses DPCD 0x05
 * @test: KUnit test context
 *
 * Verify that retrieve_downstream_port_device() reads DP_DOWNSTREAMPORT_PRESENT
 * over a mock AUX channel and caches the parsed downstream port fields.
 */
static void dm_mst_test_retrieve_downstream_present(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_aux *aux;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	aux = kunit_kzalloc(test, sizeof(*aux), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, aux);

	memset(dm_mst_test_dpcd, 0, sizeof(dm_mst_test_dpcd));
	/* PORT_PRESENT = 1, PORT_TYPE = 2 (0b101) */
	dm_mst_test_dpcd[DP_DOWNSTREAMPORT_PRESENT] = 0x05;
	dm_mst_test_aux_transfer_override = 0;

	aux->name = "dm_mst_test_aux";
	aux->transfer = dm_mst_test_aux_transfer;
	drm_dp_aux_init(aux);
	drm_dp_dpcd_set_probe(aux, false);
	aconnector->dsc_aux = aux;

	KUNIT_EXPECT_TRUE(test, retrieve_downstream_port_device(aconnector));
	KUNIT_EXPECT_EQ(test,
			(int)aconnector->mst_downstream_port_present.fields.PORT_PRESENT, 1);
	KUNIT_EXPECT_EQ(test,
			(int)aconnector->mst_downstream_port_present.fields.PORT_TYPE, 2);
}

/**
 * dm_mst_test_retrieve_downstream_aux_error - Test downstream read failure
 * @test: KUnit test context
 *
 * Verify that retrieve_downstream_port_device() returns false when the AUX
 * DPCD read fails.
 */
static void dm_mst_test_retrieve_downstream_aux_error(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_aux *aux;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	aux = kunit_kzalloc(test, sizeof(*aux), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, aux);

	dm_mst_test_aux_transfer_override = -EIO;
	aux->name = "dm_mst_test_aux";
	aux->transfer = dm_mst_test_aux_transfer;
	drm_dp_aux_init(aux);
	drm_dp_dpcd_set_probe(aux, false);
	aconnector->dsc_aux = aux;

	KUNIT_EXPECT_FALSE(test, retrieve_downstream_port_device(aconnector));

	dm_mst_test_aux_transfer_override = 0;
}

/* Tests for retrieve_branch_specific_data */

/**
 * dm_mst_test_retrieve_branch_no_parent - Test branch lookup needs a parent port
 * @test: KUnit test context
 *
 * Verify that retrieve_branch_specific_data() returns false when the MST
 * output port has no parent branch device to query.
 */
static void dm_mst_test_retrieve_branch_no_parent(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_mst_port *port;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	port = kunit_kzalloc(test, sizeof(*port), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, port);

	port->parent = NULL;
	aconnector->mst_output_port = port;

	KUNIT_EXPECT_FALSE(test, retrieve_branch_specific_data(aconnector));
}

/**
 * dm_mst_test_retrieve_branch_reads_oui - Test branch OUI parsing
 * @test: KUnit test context
 *
 * Verify that retrieve_branch_specific_data() reads the immediate upstream
 * branch descriptor and caches its IEEE OUI value on the connector.
 */
static void dm_mst_test_retrieve_branch_reads_oui(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_mst_topology_mgr *mgr;
	struct drm_dp_mst_branch *branch;
	struct drm_dp_mst_port *port;
	struct drm_dp_aux *aux;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	mgr = kunit_kzalloc(test, sizeof(*mgr), GFP_KERNEL);
	branch = kunit_kzalloc(test, sizeof(*branch), GFP_KERNEL);
	port = kunit_kzalloc(test, sizeof(*port), GFP_KERNEL);
	aux = kunit_kzalloc(test, sizeof(*aux), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, mgr);
	KUNIT_ASSERT_NOT_NULL(test, branch);
	KUNIT_ASSERT_NOT_NULL(test, port);
	KUNIT_ASSERT_NOT_NULL(test, aux);

	memset(dm_mst_test_desc_dpcd, 0, sizeof(dm_mst_test_desc_dpcd));
	dm_mst_test_desc_dpcd[0] = 0x12;
	dm_mst_test_desc_dpcd[1] = 0x34;
	dm_mst_test_desc_dpcd[2] = 0x56;

	aux->name = "dm_mst_test_desc_aux";
	aux->transfer = dm_mst_test_desc_aux_transfer;
	drm_dp_aux_init(aux);
	drm_dp_dpcd_set_probe(aux, false);
	mgr->aux = aux;
	port->parent = branch;
	port->mgr = mgr;
	port->aux.drm_dev = NULL;
	aconnector->mst_output_port = port;

	KUNIT_EXPECT_TRUE(test, retrieve_branch_specific_data(aconnector));
	KUNIT_EXPECT_EQ(test, aconnector->branch_ieee_oui, 0x123456U);
}

/**
 * dm_mst_test_aux_result_success - AUX_RET_SUCCESS preserves the input result.
 * @test: KUnit test context.
 *
 * On success the original (negative) transfer result must be returned unchanged.
 */
static void dm_mst_test_aux_result_success(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-5, AUX_RET_SUCCESS), (ssize_t)-5);
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(3, AUX_RET_SUCCESS), (ssize_t)3);
}

/**
 * dm_mst_test_aux_result_eio - HPD/unknown/protocol errors map to -EIO.
 * @test: KUnit test context.
 *
 * AUX_RET_ERROR_HPD_DISCON, AUX_RET_ERROR_UNKNOWN,
 * AUX_RET_ERROR_INVALID_OPERATION and AUX_RET_ERROR_PROTOCOL_ERROR all map to -EIO.
 */
static void dm_mst_test_aux_result_eio(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_HPD_DISCON),
			(ssize_t)-EIO);
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_UNKNOWN),
			(ssize_t)-EIO);
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_INVALID_OPERATION),
			(ssize_t)-EIO);
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_PROTOCOL_ERROR),
			(ssize_t)-EIO);
}

/**
 * dm_mst_test_aux_result_ebusy - invalid reply / engine acquire map to -EBUSY.
 * @test: KUnit test context.
 *
 * AUX_RET_ERROR_INVALID_REPLY and AUX_RET_ERROR_ENGINE_ACQUIRE map to -EBUSY.
 */
static void dm_mst_test_aux_result_ebusy(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_INVALID_REPLY),
			(ssize_t)-EBUSY);
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_ENGINE_ACQUIRE),
			(ssize_t)-EBUSY);
}

/**
 * dm_mst_test_aux_result_timeout - AUX_RET_ERROR_TIMEOUT maps to -ETIMEDOUT.
 * @test: KUnit test context.
 */
static void dm_mst_test_aux_result_timeout(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_dp_aux_transfer_result(-1, AUX_RET_ERROR_TIMEOUT),
			(ssize_t)-ETIMEDOUT);
}

/**
 * dm_mst_test_aux_transfer_native_read - native AUX read through DM callback.
 * @test: KUnit test context.
 *
 * The DM AUX transfer callback should build a read payload, call the DC link
 * service, and return the number of bytes provided by the fake backend.
 */
static void dm_mst_test_aux_transfer_native_read(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[3] = { 0 };
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	memset(dm_mst_test_dpcd, 0, sizeof(dm_mst_test_dpcd));
	dm_mst_test_dpcd[4] = 0xaa;
	dm_mst_test_dpcd[5] = 0xbb;
	dm_mst_test_dpcd[6] = 0xcc;
	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);

	ret = drm_dp_dpcd_read(&dm_aux->aux, 4, buffer, sizeof(buffer));

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)sizeof(buffer));
	KUNIT_EXPECT_EQ(test, buffer[0], (u8)0xaa);
	KUNIT_EXPECT_EQ(test, buffer[1], (u8)0xbb);
	KUNIT_EXPECT_EQ(test, buffer[2], (u8)0xcc);
	KUNIT_EXPECT_FALSE(test, dm_mst_test_last_payload.write);
	KUNIT_EXPECT_FALSE(test, dm_mst_test_last_payload.i2c_over_aux);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address, 4U);
}

/**
 * dm_mst_test_aux_transfer_native_write - native AUX write through DM callback.
 * @test: KUnit test context.
 *
 * A successful write with an ACK reply should report the requested write size
 * and pass a write payload into the fake DC link service.
 */
static void dm_mst_test_aux_transfer_native_write(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[2] = { 0x11, 0x22 };
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);

	ret = drm_dp_dpcd_write(&dm_aux->aux, 7, buffer, sizeof(buffer));

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)sizeof(buffer));
	KUNIT_EXPECT_TRUE(test, dm_mst_test_last_payload.write);
	KUNIT_EXPECT_FALSE(test, dm_mst_test_last_payload.i2c_over_aux);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address, 7U);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.length,
			(u32)sizeof(buffer));
}

/**
 * dm_mst_test_aux_transfer_partial_write - partial write reports byte count.
 * @test: KUnit test context.
 *
 * A positive write result from the DC link service should be interpreted as a
 * partial write and replaced with the first payload byte.
 */
static void dm_mst_test_aux_transfer_partial_write(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[2] = { 1, 0xaa };
	struct drm_dp_aux_msg msg = {
		.address = 7,
		.request = DP_AUX_NATIVE_WRITE,
		.buffer = buffer,
		.size = sizeof(buffer),
	};
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);
	dm_mst_test_aux_transfer_raw_result = 1;

	ret = dm_dp_aux_transfer(&dm_aux->aux, &msg);

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)buffer[0]);
	KUNIT_EXPECT_TRUE(test, dm_mst_test_last_payload.write);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address, 7U);
}

/**
 * dm_mst_test_aux_transfer_error_result - transfer errors are remapped.
 * @test: KUnit test context.
 *
 * A negative DC link service result should be converted through
 * dm_dp_aux_transfer_result() using the returned AUX operation result.
 */
static void dm_mst_test_aux_transfer_error_result(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[2] = { 0 };
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);
	dm_mst_test_aux_transfer_raw_result = -EIO;
	dm_mst_test_aux_transfer_raw_operation_result = AUX_RET_ERROR_TIMEOUT;

	ret = drm_dp_dpcd_read(&dm_aux->aux, 4, buffer, sizeof(buffer));

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)-ETIMEDOUT);
	KUNIT_EXPECT_FALSE(test, dm_mst_test_last_payload.write);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address, 4U);
}

/**
 * dm_mst_test_aux_transfer_hpd_discon_quirk - HPD disconnect quirk succeeds.
 * @test: KUnit test context.
 *
 * AUX_RET_ERROR_HPD_DISCON on the sideband down request address should be
 * treated as a successful transfer when the platform quirk is enabled.
 */
static void dm_mst_test_aux_transfer_hpd_discon_quirk(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[2] = { 2, 0 };
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);
	adev->dm.aux_hpd_discon_quirk = true;
	dm_mst_test_aux_transfer_raw_result = -EIO;
	dm_mst_test_aux_transfer_raw_operation_result = AUX_RET_ERROR_HPD_DISCON;

	ret = drm_dp_dpcd_write(&dm_aux->aux, DP_SIDEBAND_MSG_DOWN_REQ_BASE,
					 buffer, sizeof(buffer));

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)sizeof(buffer));
	KUNIT_EXPECT_TRUE(test, dm_mst_test_last_payload.write);
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address,
			DP_SIDEBAND_MSG_DOWN_REQ_BASE);
}

/**
 * dm_mst_test_aux_transfer_non_ack_reply - non-ACK AUX reply is logged.
 * @test: KUnit test context.
 *
 * A successful read with a nonzero reply byte should still return the backend
 * byte count while exercising the non-ACK reply handling path.
 */
static void dm_mst_test_aux_transfer_non_ack_reply(struct kunit *test)
{
	struct amdgpu_dm_dp_aux *dm_aux;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	struct link_service *link_srv;
	struct dc_context *ctx;
	u8 buffer[2] = { 0 };
	struct drm_dp_aux_msg msg = {
		.address = 4,
		.request = DP_AUX_NATIVE_READ,
		.buffer = buffer,
		.size = sizeof(buffer),
	};
	ssize_t ret;

	dm_aux = kunit_kzalloc(test, sizeof(*dm_aux), GFP_KERNEL);
	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = kunit_kzalloc(test, sizeof(*link), GFP_KERNEL);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dm_aux);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, link);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, ctx);

	dm_mst_test_setup_dm_aux(dm_aux, ddc, link, dc, link_srv, ctx, adev);
	dm_mst_test_aux_transfer_raw_reply = DP_AUX_NATIVE_REPLY_NACK;

	ret = dm_dp_aux_transfer(&dm_aux->aux, &msg);

	KUNIT_EXPECT_EQ(test, ret, (ssize_t)sizeof(buffer));
	KUNIT_EXPECT_EQ(test, dm_mst_test_last_payload.address, 4U);
}

/**
 * dm_mst_test_fill_payload_flags_native_write - native write request decode.
 * @test: KUnit test context.
 *
 * DP_AUX_NATIVE_WRITE clears i2c_over_aux and sets write; no I2C bits set.
 */
static void dm_mst_test_fill_payload_flags_native_write(struct kunit *test)
{
	struct aux_payload payload = { 0 };

	dm_dp_aux_fill_payload_flags(DP_AUX_NATIVE_WRITE, &payload);

	KUNIT_EXPECT_FALSE(test, payload.i2c_over_aux);
	KUNIT_EXPECT_TRUE(test, payload.write);
	KUNIT_EXPECT_FALSE(test, payload.mot);
	KUNIT_EXPECT_FALSE(test, payload.write_status_update);
}

/**
 * dm_mst_test_fill_payload_flags_native_read - native read request decode.
 * @test: KUnit test context.
 *
 * DP_AUX_NATIVE_READ keeps i2c_over_aux clear; the I2C_READ bit clears write.
 */
static void dm_mst_test_fill_payload_flags_native_read(struct kunit *test)
{
	struct aux_payload payload = { 0 };

	dm_dp_aux_fill_payload_flags(DP_AUX_NATIVE_READ, &payload);

	KUNIT_EXPECT_FALSE(test, payload.i2c_over_aux);
	KUNIT_EXPECT_FALSE(test, payload.write);
	KUNIT_EXPECT_FALSE(test, payload.mot);
}

/**
 * dm_mst_test_fill_payload_flags_i2c_read_mot - I2C read with MOT request decode.
 * @test: KUnit test context.
 *
 * DP_AUX_I2C_READ sets i2c_over_aux and clears write; DP_AUX_I2C_MOT sets mot.
 */
static void dm_mst_test_fill_payload_flags_i2c_read_mot(struct kunit *test)
{
	struct aux_payload payload = { 0 };

	dm_dp_aux_fill_payload_flags(DP_AUX_I2C_READ | DP_AUX_I2C_MOT, &payload);

	KUNIT_EXPECT_TRUE(test, payload.i2c_over_aux);
	KUNIT_EXPECT_FALSE(test, payload.write);
	KUNIT_EXPECT_TRUE(test, payload.mot);
}

/**
 * dm_mst_test_fill_payload_flags_write_status - write status update decode.
 * @test: KUnit test context.
 *
 * DP_AUX_I2C_WRITE_STATUS_UPDATE sets write_status_update.
 */
static void dm_mst_test_fill_payload_flags_write_status(struct kunit *test)
{
	struct aux_payload payload = { 0 };

	dm_dp_aux_fill_payload_flags(DP_AUX_I2C_WRITE | DP_AUX_I2C_WRITE_STATUS_UPDATE,
				     &payload);

	KUNIT_EXPECT_TRUE(test, payload.i2c_over_aux);
	KUNIT_EXPECT_TRUE(test, payload.write_status_update);
}

/**
 * dm_mst_test_msg_ready_mask - ESI mask selection per message-ready type.
 * @test: KUnit test context.
 *
 * DOWN_REP and UP_REQ each select their single bit; other types select both.
 */
static void dm_mst_test_msg_ready_mask(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_mst_msg_ready_mask(DOWN_REP_MSG_RDY_EVENT),
			(u8)DP_DOWN_REP_MSG_RDY);
	KUNIT_EXPECT_EQ(test, dm_mst_msg_ready_mask(UP_REQ_MSG_RDY_EVENT),
			(u8)DP_UP_REQ_MSG_RDY);
	KUNIT_EXPECT_EQ(test, dm_mst_msg_ready_mask(DOWN_OR_UP_MSG_RDY_EVENT),
			(u8)(DP_DOWN_REP_MSG_RDY | DP_UP_REQ_MSG_RDY));
	KUNIT_EXPECT_EQ(test, dm_mst_msg_ready_mask(NONE_MSG_RDY_EVENT),
			(u8)(DP_DOWN_REP_MSG_RDY | DP_UP_REQ_MSG_RDY));
}

/**
 * dm_mst_test_select_esi_dpcd_legacy - pre-1.2 DPCD ESI address/length.
 * @test: KUnit test context.
 *
 * For DPCD rev < 0x12 the legacy DP_SINK_COUNT address/length pair is selected.
 */
static void dm_mst_test_select_esi_dpcd_legacy(struct kunit *test)
{
	int dpcd_addr = -1;
	u8 dpcd_bytes_to_read = 0;

	dm_mst_select_esi_dpcd(0x11, &dpcd_addr, &dpcd_bytes_to_read);

	KUNIT_EXPECT_EQ(test, dpcd_addr, DP_SINK_COUNT);
	KUNIT_EXPECT_EQ(test, (int)dpcd_bytes_to_read,
			(int)(DP_LANE0_1_STATUS - DP_SINK_COUNT));
}

/**
 * dm_mst_test_select_esi_dpcd_esi - 1.2+ DPCD ESI address/length.
 * @test: KUnit test context.
 *
 * For DPCD rev >= 0x12 the ESI DP_SINK_COUNT_ESI address/length pair is selected.
 */
static void dm_mst_test_select_esi_dpcd_esi(struct kunit *test)
{
	int dpcd_addr = -1;
	u8 dpcd_bytes_to_read = 0;

	dm_mst_select_esi_dpcd(0x14, &dpcd_addr, &dpcd_bytes_to_read);

	KUNIT_EXPECT_EQ(test, dpcd_addr, DP_SINK_COUNT_ESI);
	KUNIT_EXPECT_EQ(test, (int)dpcd_bytes_to_read,
			(int)(DP_PSR_ERROR_STATUS - DP_SINK_COUNT_ESI));
}

/**
 * dm_mst_test_sideband_msg_ready_no_ready_bits - Test idle sideband event
 * @test: KUnit test context
 *
 * Verify that dm_handle_mst_sideband_msg_ready_event() returns cleanly when
 * the ESI read succeeds but no DOWN_REP/UP_REQ ready bits are set.
 */
static void dm_mst_test_sideband_msg_ready_no_ready_bits(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_sideband_connector(test);

	dm_handle_mst_sideband_msg_ready_event(&aconnector->mst_mgr,
					       DOWN_REP_MSG_RDY_EVENT);

	KUNIT_EXPECT_EQ(test, dm_mst_test_dpcd[1], (u8)0);
}

/**
 * dm_mst_test_sideband_msg_ready_read_error - Test ESI read failure path
 * @test: KUnit test context
 *
 * Verify that dm_handle_mst_sideband_msg_ready_event() returns cleanly when
 * the DPCD read fails before a ready bit can be handled.
 */
static void dm_mst_test_sideband_msg_ready_read_error(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_sideband_connector(test);
	dm_mst_test_aux_transfer_override = -EIO;

	dm_handle_mst_sideband_msg_ready_event(&aconnector->mst_mgr,
					       DOWN_REP_MSG_RDY_EVENT);

	KUNIT_EXPECT_EQ(test, dm_mst_test_dpcd[1], (u8)0);
	dm_mst_test_aux_transfer_override = 0;
}

/**
 * dm_mst_test_sideband_msg_ready_without_mst_state - Test ready bit no-op path
 * @test: KUnit test context
 *
 * Verify that a DOWN_REP ready bit is filtered and then ignored when the MST
 * topology manager is not enabled.
 */
static void dm_mst_test_sideband_msg_ready_without_mst_state(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_sideband_connector(test);
	dm_mst_test_dpcd[(DP_SINK_COUNT_ESI + 1) & 0xf] = DP_DOWN_REP_MSG_RDY;

	dm_handle_mst_sideband_msg_ready_event(&aconnector->mst_mgr,
					       DOWN_REP_MSG_RDY_EVENT);

	KUNIT_EXPECT_EQ(test, dm_mst_test_dpcd[(DP_SINK_COUNT_ESI + 1) & 0xf],
			 DP_DOWN_REP_MSG_RDY);
}

/**
 * dm_mst_test_down_rep_msg_ready_wrapper - Test DOWN_REP wrapper
 * @test: KUnit test context
 *
 * Verify that dm_handle_mst_down_rep_msg_ready() forwards to the generic MST
 * sideband handler with the DOWN_REP event selection.
 */
static void dm_mst_test_down_rep_msg_ready_wrapper(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_sideband_connector(test);

	dm_handle_mst_down_rep_msg_ready(&aconnector->mst_mgr);

	KUNIT_EXPECT_EQ(test, dm_mst_test_dpcd[1], (u8)0);
}

/**
 * dm_mst_test_initialize_dp_connector_edp - Test eDP initialization path
 * @test: KUnit test context
 *
 * Verify that amdgpu_dm_initialize_dp_connector() initializes the DP AUX state
 * and exits before MST topology setup for eDP connectors.
 */
static void dm_mst_test_initialize_dp_connector_edp(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_device *adev;
	struct ddc_service *ddc;
	struct dc_link *link;

	adev = dm_kunit_alloc_adev(test);
	link = dm_kunit_alloc_link(test);
	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ddc);

	adev->dm.adev = adev;
	adev->dm.ddev = &adev->ddev;
	link->ddc = ddc;
	aconnector = dm_kunit_alloc_connector(test, adev, link);
	aconnector->base.connector_type = DRM_MODE_CONNECTOR_eDP;

	amdgpu_dm_initialize_dp_connector(&adev->dm, aconnector, 5);

	KUNIT_EXPECT_TRUE(test, aconnector->dm_dp_aux.aux.transfer == dm_dp_aux_transfer);
	KUNIT_EXPECT_PTR_EQ(test, aconnector->dm_dp_aux.aux.drm_dev, &adev->ddev);
	KUNIT_EXPECT_PTR_EQ(test, aconnector->dm_dp_aux.ddc_service, ddc);
	KUNIT_EXPECT_PTR_EQ(test, aconnector->mst_mgr.dev, NULL);
	KUNIT_EXPECT_NOT_NULL(test, aconnector->dm_dp_aux.aux.name);
	if (aconnector->dm_dp_aux.aux.name)
		KUNIT_EXPECT_NOT_NULL(test, strstr(aconnector->dm_dp_aux.aux.name, "5"));

	drm_dp_cec_unregister_connector(&aconnector->dm_dp_aux.aux);
	kfree(aconnector->dm_dp_aux.aux.name);
}

static bool dm_mst_test_dp_get_max_link_enc_cap(const struct dc_link *link,
						struct dc_link_settings *cap)
{
	return true;
}

static void dm_mst_test_connector_destroy(struct drm_connector *connector)
{
}

static const struct drm_connector_funcs dm_mst_test_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.destroy = dm_mst_test_connector_destroy,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/**
 * dm_mst_test_initialize_dp_connector_mst - Test MST root initialization path
 * @test: KUnit test context
 *
 * Verify that amdgpu_dm_initialize_dp_connector() initializes the MST topology
 * manager for a non-eDP DisplayPort connector. This exercises the path past the
 * eDP early return, including dc_link_dp_get_max_link_enc_cap() and
 * drm_dp_mst_topology_mgr_init(). A fully initialized DRM mode config and
 * connector are required because the topology manager registers a private
 * atomic object and the subconnector property is attached to the connector.
 */
static void dm_mst_test_initialize_dp_connector_mst(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_device *adev;
	struct link_service *link_srv;
	struct ddc_service *ddc;
	struct dc_link *link;
	struct dc *dc;
	int ret;

	adev = dm_kunit_alloc_adev(test);

	ret = drmm_mode_config_init(&adev->ddev);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ddc = kunit_kzalloc(test, sizeof(*ddc), GFP_KERNEL);
	link = dm_kunit_alloc_link(test);
	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ddc);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);

	link_srv->dp_get_max_link_enc_cap = dm_mst_test_dp_get_max_link_enc_cap;
	dc->link_srv = link_srv;
	link->dc = dc;
	link->ddc = ddc;

	adev->dm.adev = adev;
	adev->dm.ddev = &adev->ddev;

	aconnector = dm_kunit_alloc_connector(test, adev, link);

	ret = drm_connector_init(&adev->ddev, &aconnector->base,
				 &dm_mst_test_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	KUNIT_ASSERT_EQ(test, ret, 0);

	amdgpu_dm_initialize_dp_connector(&adev->dm, aconnector, 7);

	KUNIT_EXPECT_TRUE(test, aconnector->dm_dp_aux.aux.transfer == dm_dp_aux_transfer);
	KUNIT_EXPECT_PTR_EQ(test, aconnector->mst_mgr.dev, &adev->ddev);
	KUNIT_EXPECT_PTR_EQ(test, aconnector->mst_mgr.aux, &aconnector->dm_dp_aux.aux);
	KUNIT_EXPECT_EQ(test, aconnector->mst_mgr.max_payloads, 4);
	KUNIT_EXPECT_TRUE(test, aconnector->mst_mgr.cbs != NULL);

	drm_dp_mst_topology_mgr_destroy(&aconnector->mst_mgr);
	drm_dp_cec_unregister_connector(&aconnector->dm_dp_aux.aux);
	kfree(aconnector->dm_dp_aux.aux.name);
	drm_connector_cleanup(&aconnector->base);
}

/**
 * dm_mst_test_atomic_best_encoder - Test MST encoder selection
 * @test: KUnit test context
 *
 * Verify that dm_mst_atomic_best_encoder() selects the MST encoder indexed by
 * the CRTC ID in the connector's new atomic state. This uses structural DRM
 * mocks only; registering connector/CRTC objects is unnecessary for this helper.
 */
static void dm_mst_test_atomic_best_encoder(struct kunit *test)
{
	struct drm_connector_state connector_state = { 0 };
	struct drm_atomic_commit state = { 0 };
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_device *adev;
	struct amdgpu_crtc *acrtc;
	unsigned int connector_index = 3;

	adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);
	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	acrtc = kunit_kzalloc(test, sizeof(*acrtc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, acrtc);

	aconnector->base.dev = &adev->ddev;
	aconnector->base.index = connector_index;
	acrtc->crtc_id = 2;
	connector_state.connector = &aconnector->base;
	connector_state.crtc = &acrtc->base;
	state.num_connector = connector_index + 1;
	state.connectors = kunit_kzalloc(test,
					 sizeof(*state.connectors) * state.num_connector,
					 GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state.connectors);
	state.connectors[connector_index].ptr = &aconnector->base;
	state.connectors[connector_index].new_state = &connector_state;

	KUNIT_EXPECT_PTR_EQ(test, dm_mst_atomic_best_encoder(&aconnector->base, &state),
			     &adev->dm.mst_encoders[2].base);
}

/**
 * dm_mst_test_create_fake_mst_encoders - Test fake MST encoder setup
 * @test: KUnit test context
 *
 * Verify that dm_dp_create_fake_mst_encoders() initializes the requested MST
 * encoders as DPMST encoders with the CRTC mask derived from the device state.
 */
static void dm_mst_test_create_fake_mst_encoders(struct kunit *test)
{
	struct amdgpu_device *adev;
	struct drm_device *drm;
	int i;

	adev = dm_kunit_alloc_adev(test);
	drm = &adev->ddev;
	adev->dm.display_indexes_num = 3;
	adev->mode_info.num_crtc = 3;

	dm_dp_create_fake_mst_encoders(adev);

	for (i = 0; i < adev->dm.display_indexes_num; i++) {
		struct drm_encoder *encoder = &adev->dm.mst_encoders[i].base;

		KUNIT_EXPECT_PTR_EQ(test, encoder->dev, drm);
		KUNIT_EXPECT_EQ(test, encoder->encoder_type, DRM_MODE_ENCODER_DPMST);
		KUNIT_EXPECT_EQ(test, encoder->possible_crtcs, 0x7U);
		KUNIT_EXPECT_TRUE(test, encoder->helper_private != NULL);
	}
}

/**
 * dm_mst_test_atomic_check_no_old_crtc - Test atomic check no-op path
 * @test: KUnit test context
 *
 * Verify that dm_dp_mst_atomic_check() returns success when the MST port's old
 * connector state has no CRTC, before MST topology state is required.
 */
static void dm_mst_test_atomic_check_no_old_crtc(struct kunit *test)
{
	struct drm_connector_state *old_conn_state;
	struct drm_connector_state *new_conn_state;
	struct drm_atomic_commit *state;
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_dm_connector *root;
	struct drm_dp_mst_port *port;
	unsigned int connector_index = 2;

	old_conn_state = kunit_kzalloc(test, sizeof(*old_conn_state), GFP_KERNEL);
	new_conn_state = kunit_kzalloc(test, sizeof(*new_conn_state), GFP_KERNEL);
	state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	root = kunit_kzalloc(test, sizeof(*root), GFP_KERNEL);
	port = kunit_kzalloc(test, sizeof(*port), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, old_conn_state);
	KUNIT_ASSERT_NOT_NULL(test, new_conn_state);
	KUNIT_ASSERT_NOT_NULL(test, state);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, root);
	KUNIT_ASSERT_NOT_NULL(test, port);

	aconnector->base.index = connector_index;
	aconnector->mst_root = root;
	aconnector->mst_output_port = port;
	port->connector = &aconnector->base;
	old_conn_state->connector = &aconnector->base;
	new_conn_state->connector = &aconnector->base;
	state->num_connector = connector_index + 1;
	state->connectors = kunit_kzalloc(test,
					 sizeof(*state->connectors) * state->num_connector,
					 GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->connectors);
	state->connectors[connector_index].ptr = &aconnector->base;
	state->connectors[connector_index].old_state = old_conn_state;
	state->connectors[connector_index].new_state = new_conn_state;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_atomic_check(&aconnector->base, state), 0);
}

/**
 * dm_mst_test_detect_unregistered - Test detect skips unregistered connector
 * @test: KUnit test context
 *
 * Verify that dm_dp_mst_detect() returns disconnected for an unregistered
 * connector before calling into the MST topology helper.
 */
static void dm_mst_test_detect_unregistered(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = kunit_kzalloc(test, sizeof(*aconnector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);

	aconnector->base.registration_state = DRM_CONNECTOR_UNREGISTERED;

	KUNIT_EXPECT_EQ(test,
			dm_dp_mst_detect(&aconnector->base, NULL, false),
			(int)connector_status_disconnected);
}

/*
 * Fake DC remote-sink backend. dc_link_add_remote_sink() and
 * dc_link_remove_remote_sink() are thin wrappers over these link_service
 * callbacks, so overriding them is enough to run the MST connector paths
 * without a DC core.
 */
static struct dc_sink *dm_mst_test_next_remote_sink;
static struct dc_sink *dm_mst_test_removed_sink;
static unsigned int dm_mst_test_add_remote_sink_calls;
static unsigned int dm_mst_test_remove_remote_sink_calls;

/* Signature copied verbatim from struct link_service::add_remote_sink. */
static struct dc_sink *dm_mst_test_add_remote_sink(
		struct dc_link *link,
		const uint8_t *edid,
		unsigned int len,
		struct dc_sink_init_data *init_data)
{
	struct dc_sink *sink = dm_mst_test_next_remote_sink;

	dm_mst_test_add_remote_sink_calls++;

	if (sink) {
		sink->link = link;
		sink->sink_signal = init_data->sink_signal;
		link->sink_count++;
	}

	return sink;
}

/* Signature copied verbatim from struct link_service::remove_remote_sink. */
static void dm_mst_test_remove_remote_sink(struct dc_link *link, struct dc_sink *sink)
{
	dm_mst_test_remove_remote_sink_calls++;
	dm_mst_test_removed_sink = sink;

	if (link->sink_count)
		link->sink_count--;
}

/*
 * dc_sink_release() frees the sink with kfree(), so these must not come from
 * the KUnit managed allocator.
 */
static struct dc_sink *dm_mst_test_alloc_sink(struct kunit *test)
{
	struct dc_sink *sink = kzalloc_obj(*sink);

	KUNIT_ASSERT_NOT_NULL(test, sink);
	kref_init(&sink->refcount);

	return sink;
}

struct dm_mst_test_child {
	struct amdgpu_device *adev;
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_dm_connector *root;
	struct drm_dp_mst_port *port;
	struct dc_link *link;
};

/*
 * Build an MST downstream connector hanging off a root connector. The topology
 * manager is left empty (no mst_primary) so the DRM MST helpers take their
 * "port is gone" paths instead of requiring a real branch device.
 */
static void dm_mst_test_init_child(struct kunit *test, struct dm_mst_test_child *child)
{
	struct link_service *link_srv;
	struct amdgpu_device *adev;
	struct dc *dc;
	int ret;

	adev = dm_kunit_alloc_adev(test);
	ret = drmm_mode_config_init(&adev->ddev);
	KUNIT_ASSERT_EQ(test, ret, 0);

	dc = kunit_kzalloc(test, sizeof(*dc), GFP_KERNEL);
	link_srv = kunit_kzalloc(test, sizeof(*link_srv), GFP_KERNEL);
	child->port = kunit_kzalloc(test, sizeof(*child->port), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dc);
	KUNIT_ASSERT_NOT_NULL(test, link_srv);
	KUNIT_ASSERT_NOT_NULL(test, child->port);

	link_srv->add_remote_sink = dm_mst_test_add_remote_sink;
	link_srv->remove_remote_sink = dm_mst_test_remove_remote_sink;
	dc->link_srv = link_srv;

	child->adev = adev;
	child->link = dm_kunit_alloc_link(test);
	child->link->dc = dc;

	child->root = dm_kunit_alloc_connector(test, adev, child->link);
	child->root->mst_mgr.dev = &adev->ddev;
	mutex_init(&child->root->mst_mgr.lock);
	drm_modeset_lock_init(&child->root->mst_mgr.base.lock);

	child->port->mgr = &child->root->mst_mgr;
	child->port->aux.name = "dm_mst_test_port_aux";
	child->port->aux.transfer = dm_mst_test_aux_transfer;
	drm_dp_aux_init(&child->port->aux);
	drm_dp_dpcd_set_probe(&child->port->aux, false);

	child->aconnector = dm_kunit_alloc_connector(test, adev, child->link);
	child->aconnector->mst_root = child->root;
	child->aconnector->mst_output_port = child->port;

	ret = drm_connector_init(&adev->ddev, &child->aconnector->base,
				 &dm_mst_test_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	KUNIT_ASSERT_EQ(test, ret, 0);
	/* The MST (un)register helpers log through the connector's kernel device. */
	child->aconnector->base.kdev = adev->ddev.dev;

	memset(dm_mst_test_dpcd, 0, sizeof(dm_mst_test_dpcd));
	dm_mst_test_aux_transfer_override = 0;
	dm_mst_test_aux_write_override = 0;
	dm_mst_test_next_remote_sink = NULL;
	dm_mst_test_removed_sink = NULL;
	dm_mst_test_add_remote_sink_calls = 0;
	dm_mst_test_remove_remote_sink_calls = 0;
}

static void dm_mst_test_fini_child(struct dm_mst_test_child *child)
{
	drm_edid_free(child->aconnector->drm_edid);
	drm_edid_free(child->port->cached_edid);
	drm_connector_cleanup(&child->aconnector->base);
	drm_modeset_lock_fini(&child->root->mst_mgr.base.lock);
	mutex_destroy(&child->root->mst_mgr.lock);
}

/* Tests for dm_dp_mst_get_modes */

/**
 * dm_mst_test_get_modes_no_edid_adds_default_sink - Test default remote sink
 * @test: KUnit test context
 *
 * When the remote EDID cannot be read, dm_dp_mst_get_modes() must clear the
 * MST_REMOTE_EDID status and register a default remote sink for the connector.
 */
static void dm_mst_test_get_modes_no_edid_adds_default_sink(struct kunit *test)
{
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);
	sink = dm_mst_test_alloc_sink(test);
	dm_mst_test_next_remote_sink = sink;
	child.aconnector->mst_status = MST_REMOTE_EDID;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_EQ(test, dm_mst_test_add_remote_sink_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, child.aconnector->dc_sink, sink);
	KUNIT_EXPECT_PTR_EQ(test, sink->priv, (void *)child.aconnector);
	KUNIT_EXPECT_EQ(test, child.aconnector->mst_status & MST_REMOTE_EDID, 0);

	dc_sink_release(sink);
	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_get_modes_no_edid_sink_alloc_fails - Test remote sink failure
 * @test: KUnit test context
 *
 * If DC cannot add the default remote sink, dm_dp_mst_get_modes() must bail out
 * with zero modes and leave the connector without a sink.
 */
static void dm_mst_test_get_modes_no_edid_sink_alloc_fails(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_EQ(test, dm_mst_test_add_remote_sink_calls, 1U);
	KUNIT_EXPECT_NULL(test, child.aconnector->dc_sink);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_get_modes_no_edid_keeps_existing_sink - Test cached sink reuse
 * @test: KUnit test context
 *
 * With no readable EDID but an existing remote sink, dm_dp_mst_get_modes() must
 * keep that sink and must not ask DC for another one.
 */
static void dm_mst_test_get_modes_no_edid_keeps_existing_sink(struct kunit *test)
{
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);
	sink = dm_mst_test_alloc_sink(test);
	child.aconnector->dc_sink = sink;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_EQ(test, dm_mst_test_add_remote_sink_calls, 0U);
	KUNIT_EXPECT_PTR_EQ(test, child.aconnector->dc_sink, sink);

	dc_sink_release(sink);
	dm_mst_test_fini_child(&child);
}

/* Minimal EDID base block: valid header, no extensions, correct checksum. */
static const u8 dm_mst_test_edid[EDID_LENGTH] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	[EDID_LENGTH - 1] = 0x06,
};

/**
 * dm_mst_test_get_modes_cached_edid_replaces_virtual_sink - Test EDID sink path
 * @test: KUnit test context
 *
 * With a cached EDID and a placeholder virtual sink, dm_dp_mst_get_modes() must
 * release the virtual sink, add a real remote sink built from the EDID, and
 * update the connector's mode list.
 */
static void dm_mst_test_get_modes_cached_edid_replaces_virtual_sink(struct kunit *test)
{
	struct dm_mst_test_child child;
	struct dc_sink *virtual_sink;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);

	child.aconnector->drm_edid = drm_edid_alloc(dm_mst_test_edid, sizeof(dm_mst_test_edid));
	KUNIT_ASSERT_NOT_NULL(test, child.aconnector->drm_edid);

	virtual_sink = dm_mst_test_alloc_sink(test);
	virtual_sink->sink_signal = SIGNAL_TYPE_VIRTUAL;
	child.aconnector->dc_sink = virtual_sink;

	sink = dm_mst_test_alloc_sink(test);
	dm_mst_test_next_remote_sink = sink;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_EQ(test, dm_mst_test_add_remote_sink_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, child.aconnector->dc_sink, sink);
	KUNIT_EXPECT_EQ(test, sink->sink_signal, SIGNAL_TYPE_DISPLAY_PORT_MST);
	KUNIT_EXPECT_PTR_EQ(test, sink->priv, (void *)child.aconnector);

	dc_sink_release(sink);
	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_get_modes_cached_edid_sink_alloc_fails - Test EDID sink failure
 * @test: KUnit test context
 *
 * If DC cannot add the remote sink built from the cached EDID,
 * dm_dp_mst_get_modes() must bail out with zero modes.
 */
static void dm_mst_test_get_modes_cached_edid_sink_alloc_fails(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);

	child.aconnector->drm_edid = drm_edid_alloc(dm_mst_test_edid, sizeof(dm_mst_test_edid));
	KUNIT_ASSERT_NOT_NULL(test, child.aconnector->drm_edid);

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_EQ(test, dm_mst_test_add_remote_sink_calls, 1U);
	KUNIT_EXPECT_NULL(test, child.aconnector->dc_sink);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_get_modes_restores_hdcp_properties - Test HDCP property restore
 * @test: KUnit test context
 *
 * A connector re-plugged at the same display index must have its content
 * protection state restored from the HDCP workqueue when the remote sink is
 * re-created.
 */
static void dm_mst_test_get_modes_restores_hdcp_properties(struct kunit *test)
{
	struct dm_connector_state *conn_state;
	struct dm_mst_test_child child;
	struct hdcp_workqueue *hdcp;
	struct dc_sink *sink;
	unsigned int index;

	dm_mst_test_init_child(test, &child);

	child.aconnector->drm_edid = drm_edid_alloc(dm_mst_test_edid, sizeof(dm_mst_test_edid));
	KUNIT_ASSERT_NOT_NULL(test, child.aconnector->drm_edid);

	/* drm_connector_cleanup() hands the state back to the DRM helper's kfree(). */
	conn_state = kzalloc_obj(*conn_state);
	hdcp = kunit_kzalloc(test, sizeof(*hdcp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, conn_state);
	KUNIT_ASSERT_NOT_NULL(test, hdcp);

	index = child.aconnector->base.index;
	hdcp->hdcp_content_type[index] = DRM_MODE_HDCP_CONTENT_TYPE1;
	hdcp->content_protection[index] = DRM_MODE_CONTENT_PROTECTION_ENABLED;
	child.adev->dm.hdcp_workqueue = hdcp;
	child.aconnector->base.state = &conn_state->base;

	sink = dm_mst_test_alloc_sink(test);
	dm_mst_test_next_remote_sink = sink;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_PTR_EQ(test, child.aconnector->dc_sink, sink);
	KUNIT_EXPECT_EQ(test, child.aconnector->base.state->hdcp_content_type,
			(unsigned int)DRM_MODE_HDCP_CONTENT_TYPE1);
	KUNIT_EXPECT_EQ(test, child.aconnector->base.state->content_protection,
			(unsigned int)DRM_MODE_CONTENT_PROTECTION_ENABLED);

	dc_sink_release(sink);
	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_get_modes_reads_remote_edid - Test remote EDID caching
 * @test: KUnit test context
 *
 * When the MST port is still in the topology, dm_dp_mst_get_modes() must read
 * its EDID, cache it on the connector and flag MST_REMOTE_EDID.
 */
static void dm_mst_test_get_modes_reads_remote_edid(struct kunit *test)
{
	struct drm_dp_mst_branch *mstb;
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);

	/* Minimal topology so drm_dp_mst_edid_read() can validate the port. */
	mstb = kunit_kzalloc(test, sizeof(*mstb), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, mstb);
	mstb->mgr = &child.root->mst_mgr;
	INIT_LIST_HEAD(&mstb->ports);
	list_add(&child.port->next, &mstb->ports);
	kref_init(&child.port->topology_kref);
	child.root->mst_mgr.mst_primary = mstb;

	child.port->cached_edid = drm_edid_alloc(dm_mst_test_edid, sizeof(dm_mst_test_edid));
	KUNIT_ASSERT_NOT_NULL(test, child.port->cached_edid);

	sink = dm_mst_test_alloc_sink(test);
	dm_mst_test_next_remote_sink = sink;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_get_modes(&child.aconnector->base), 0);

	KUNIT_EXPECT_NOT_NULL(test, child.aconnector->drm_edid);
	KUNIT_EXPECT_EQ(test, child.aconnector->mst_status & MST_REMOTE_EDID, (int)MST_REMOTE_EDID);
	KUNIT_EXPECT_PTR_EQ(test, child.aconnector->dc_sink, sink);

	dc_sink_release(sink);
	dm_mst_test_fini_child(&child);
}

/* Tests for dm_dp_mst_detect */

/**
 * dm_mst_test_detect_reads_port_dpcd_rev - Test DPCD revision probing
 * @test: KUnit test context
 *
 * For a peer device with an unknown DPCD revision, dm_dp_mst_detect() must probe
 * DP_DP13_DPCD_REV over the port AUX and cache the returned revision.
 */
static void dm_mst_test_detect_reads_port_dpcd_rev(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);
	child.port->pdt = DP_PEER_DEVICE_SST_SINK;
	/* Both DP_DP13_DPCD_REV and DP_DPCD_REV alias to offset 0 in the fake. */
	dm_mst_test_dpcd[0] = 0x13;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_detect(&child.aconnector->base, NULL, false),
			(int)connector_status_disconnected);

	KUNIT_EXPECT_EQ(test, (int)child.port->dpcd_rev, 0x13);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_detect_unknown_dpcd_rev - Test unreadable DPCD revision
 * @test: KUnit test context
 *
 * When both DPCD revision registers read back as zero, dm_dp_mst_detect() must
 * leave the cached revision cleared instead of reporting a bogus value.
 */
static void dm_mst_test_detect_unknown_dpcd_rev(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);
	child.port->pdt = DP_PEER_DEVICE_SST_SINK;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_detect(&child.aconnector->base, NULL, false),
			(int)connector_status_disconnected);

	KUNIT_EXPECT_EQ(test, (int)child.port->dpcd_rev, 0);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_detect_dpcd_read_error - Test unreachable port DPCD
 * @test: KUnit test context
 *
 * When the remote DPCD read is NAKed, dm_dp_mst_detect() must leave the cached
 * revision untouched instead of storing a garbage value.
 */
static void dm_mst_test_detect_dpcd_read_error(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);
	child.port->pdt = DP_PEER_DEVICE_SST_SINK;
	dm_mst_test_aux_transfer_override = -EIO;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_detect(&child.aconnector->base, NULL, false),
			(int)connector_status_disconnected);

	KUNIT_EXPECT_EQ(test, (int)child.port->dpcd_rev, 0);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_detect_disconnect_releases_sink - Test unplug sink release
 * @test: KUnit test context
 *
 * A port with no peer device must have its cached DPCD revision cleared, and the
 * resulting disconnected status must release the remote sink and reset the MST
 * connector state.
 */
static void dm_mst_test_detect_disconnect_releases_sink(struct kunit *test)
{
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);
	child.port->pdt = DP_PEER_DEVICE_NONE;
	child.port->dpcd_rev = 0x14;

	sink = dm_mst_test_alloc_sink(test);
	child.aconnector->dc_sink = sink;
	child.aconnector->dsc_aux = &child.port->aux;
	child.aconnector->mst_local_bw = 1234;
	child.link->sink_count = 1;

	KUNIT_EXPECT_EQ(test, dm_dp_mst_detect(&child.aconnector->base, NULL, false),
			(int)connector_status_disconnected);

	KUNIT_EXPECT_EQ(test, (int)child.port->dpcd_rev, 0);
	KUNIT_EXPECT_EQ(test, dm_mst_test_remove_remote_sink_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, dm_mst_test_removed_sink, sink);
	KUNIT_EXPECT_NULL(test, child.aconnector->dc_sink);
	KUNIT_EXPECT_NULL(test, child.aconnector->dsc_aux);
	KUNIT_EXPECT_EQ(test, child.aconnector->mst_local_bw, 0U);

	dm_mst_test_fini_child(&child);
}

/* Tests for amdgpu_dm_mst_connector_late_register */

/**
 * dm_mst_test_connector_late_register - Test MST connector late registration
 * @test: KUnit test context
 *
 * amdgpu_dm_mst_connector_late_register() must register the port's remote AUX
 * bus and report success.
 */
static void dm_mst_test_connector_late_register(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);

	KUNIT_EXPECT_EQ(test, amdgpu_dm_mst_connector_late_register(&child.aconnector->base), 0);

	dm_mst_test_fini_child(&child);
}

/* Tests for amdgpu_dm_mst_connector_early_unregister */

/**
 * dm_mst_test_connector_early_unregister_no_sink - Test unregister without sink
 * @test: KUnit test context
 *
 * With no remote sink attached, amdgpu_dm_mst_connector_early_unregister() must
 * only reset the MST status.
 */
static void dm_mst_test_connector_early_unregister_no_sink(struct kunit *test)
{
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);
	child.aconnector->mst_status = MST_REMOTE_EDID;

	amdgpu_dm_mst_connector_early_unregister(&child.aconnector->base);

	KUNIT_EXPECT_EQ(test, dm_mst_test_remove_remote_sink_calls, 0U);
	KUNIT_EXPECT_EQ(test, (int)child.aconnector->mst_status, (int)MST_STATUS_DEFAULT);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_connector_early_unregister_releases_sink - Test sink release
 * @test: KUnit test context
 *
 * When the port leaves the topology, amdgpu_dm_mst_connector_early_unregister()
 * must remove the remote sink from the link and reset the MST connector state.
 */
static void dm_mst_test_connector_early_unregister_releases_sink(struct kunit *test)
{
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);

	sink = dm_mst_test_alloc_sink(test);
	child.aconnector->dc_sink = sink;
	child.aconnector->vc_full_pbn = 42;
	child.link->sink_count = 1;

	amdgpu_dm_mst_connector_early_unregister(&child.aconnector->base);

	KUNIT_EXPECT_EQ(test, dm_mst_test_remove_remote_sink_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, dm_mst_test_removed_sink, sink);
	KUNIT_EXPECT_NULL(test, child.aconnector->dc_sink);
	KUNIT_EXPECT_EQ(test, child.aconnector->vc_full_pbn, 0U);
	KUNIT_EXPECT_EQ(test, (int)child.aconnector->mst_status, (int)MST_STATUS_DEFAULT);

	dm_mst_test_fini_child(&child);
}

/* Tests for dm_dp_mst_connector_destroy */

/*
 * dm_dp_mst_connector_destroy() frees both the connector and the MST port, so
 * they are allocated outside the KUnit managed allocator. The parent branch
 * device takes an extra malloc reference so it survives the port teardown.
 */
static struct amdgpu_dm_connector *
dm_mst_test_alloc_destroyable_connector(struct kunit *test, struct dm_mst_test_child *child)
{
	struct amdgpu_dm_connector *aconnector;
	struct drm_dp_mst_branch *mstb;
	struct drm_dp_mst_port *port;
	int ret;

	aconnector = kzalloc_obj(*aconnector);
	port = kzalloc_obj(*port);
	mstb = kunit_kzalloc(test, sizeof(*mstb), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, aconnector);
	KUNIT_ASSERT_NOT_NULL(test, port);
	KUNIT_ASSERT_NOT_NULL(test, mstb);

	mstb->mgr = &child->root->mst_mgr;
	kref_init(&mstb->malloc_kref);
	kref_get(&mstb->malloc_kref);

	port->mgr = &child->root->mst_mgr;
	port->parent = mstb;
	kref_init(&port->malloc_kref);

	aconnector->dc_link = child->link;
	aconnector->mst_root = child->root;
	aconnector->mst_output_port = port;

	ret = drm_connector_init(&child->adev->ddev, &aconnector->base,
				 &dm_mst_test_connector_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	KUNIT_ASSERT_EQ(test, ret, 0);

	return aconnector;
}

/**
 * dm_mst_test_connector_destroy_no_sink - Test destroy without a remote sink
 * @test: KUnit test context
 *
 * dm_dp_mst_connector_destroy() must clean up the DRM connector and drop the MST
 * port reference even when no remote sink was ever attached.
 */
static void dm_mst_test_connector_destroy_no_sink(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct dm_mst_test_child child;

	dm_mst_test_init_child(test, &child);
	aconnector = dm_mst_test_alloc_destroyable_connector(test, &child);

	dm_dp_mst_connector_destroy(&aconnector->base);

	KUNIT_EXPECT_EQ(test, dm_mst_test_remove_remote_sink_calls, 0U);

	dm_mst_test_fini_child(&child);
}

/**
 * dm_mst_test_connector_destroy_releases_sink - Test destroy releases the sink
 * @test: KUnit test context
 *
 * dm_dp_mst_connector_destroy() must remove the remote sink from the DC link
 * before tearing down the connector.
 */
static void dm_mst_test_connector_destroy_releases_sink(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct dm_mst_test_child child;
	struct dc_sink *sink;

	dm_mst_test_init_child(test, &child);
	aconnector = dm_mst_test_alloc_destroyable_connector(test, &child);

	sink = dm_mst_test_alloc_sink(test);
	aconnector->dc_sink = sink;
	child.link->sink_count = 1;

	dm_dp_mst_connector_destroy(&aconnector->base);

	KUNIT_EXPECT_EQ(test, dm_mst_test_remove_remote_sink_calls, 1U);
	KUNIT_EXPECT_PTR_EQ(test, dm_mst_test_removed_sink, sink);

	dm_mst_test_fini_child(&child);
}

/*
 * Sideband connector with a live topology manager and the DOWN_REP ready bit
 * armed, so dm_handle_mst_sideband_msg_ready_event() reaches its ack path.
 */
static struct amdgpu_dm_connector *
dm_mst_test_alloc_armed_sideband_connector(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;
	struct amdgpu_device *adev;
	int ret;

	adev = dm_kunit_alloc_adev(test);
	ret = drmm_mode_config_init(&adev->ddev);
	KUNIT_ASSERT_EQ(test, ret, 0);

	aconnector = dm_mst_test_alloc_sideband_connector(test);

	ret = drm_dp_mst_topology_mgr_init(&aconnector->mst_mgr, &adev->ddev,
					   &aconnector->dm_dp_aux.aux, 16, 4, 0);
	KUNIT_ASSERT_EQ(test, ret, 0);
	aconnector->mst_mgr.mst_state = true;
	dm_mst_test_dpcd[(DP_SINK_COUNT_ESI + 1) & 0xf] = DP_DOWN_REP_MSG_RDY;

	return aconnector;
}

static void dm_mst_test_free_armed_sideband_connector(struct amdgpu_dm_connector *aconnector)
{
	aconnector->mst_mgr.mst_state = false;
	drm_dp_mst_topology_mgr_destroy(&aconnector->mst_mgr);
}

/**
 * dm_mst_test_sideband_msg_ready_acks_down_rep - Test DOWN_REP ack handling
 * @test: KUnit test context
 *
 * With an active topology manager and the DOWN_REP ready bit set, the sideband
 * handler must acknowledge the event at DPCD and keep polling until the
 * iteration limit is reached, since the fake sideband message never completes.
 */
static void dm_mst_test_sideband_msg_ready_acks_down_rep(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_armed_sideband_connector(test);

	dm_handle_mst_sideband_msg_ready_event(&aconnector->mst_mgr, DOWN_REP_MSG_RDY_EVENT);

	KUNIT_EXPECT_EQ(test, aconnector->mst_mgr.sink_count, 0);

	dm_mst_test_free_armed_sideband_connector(aconnector);
}

/**
 * dm_mst_test_sideband_msg_ready_ack_write_fails - Test failed DPCD ack
 * @test: KUnit test context
 *
 * When the DPCD acknowledge write keeps failing, the sideband handler must give
 * up after the third retry instead of looping forever.
 */
static void dm_mst_test_sideband_msg_ready_ack_write_fails(struct kunit *test)
{
	struct amdgpu_dm_connector *aconnector;

	aconnector = dm_mst_test_alloc_armed_sideband_connector(test);
	dm_mst_test_aux_write_override = -EIO;

	dm_handle_mst_sideband_msg_ready_event(&aconnector->mst_mgr, DOWN_REP_MSG_RDY_EVENT);

	KUNIT_EXPECT_EQ(test, aconnector->mst_mgr.sink_count, 0);

	dm_mst_test_free_armed_sideband_connector(aconnector);
}

#if !defined(CONFIG_DRM_AMD_DC_FP)
/**
 * dm_mst_test_fp_guarded_public_stubs - Test FP-off public fallbacks
 * @test: KUnit test context
 *
 * When CONFIG_DRM_AMD_DC_FP is disabled, the public DSC validation helper
 * has no FP body and must return DC_OK without touching its arguments.
 */
static void dm_mst_test_fp_guarded_public_stubs(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dm_dp_mst_is_port_support_mode(NULL, NULL),
			(enum dc_status)DC_OK);
}
#endif

static struct kunit_case dm_mst_types_test_cases[] = {
	/* needs_dsc_aux_workaround tests */
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_match),
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_rev12),
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_wrong_dev_id),
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_wrong_rev),
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_low_sink_count),
	KUNIT_CASE(dm_mst_test_needs_dsc_aux_workaround_zero_sink_count),
	/* dm_mst_get_pbn_divider tests */
	KUNIT_CASE(dm_mst_test_pbn_divider_null_link),
	KUNIT_CASE(dm_mst_test_pbn_divider_uses_link_bandwidth),
	/* amdgpu_dm_mst_reset_mst_connector_setting tests */
	KUNIT_CASE(dm_mst_test_reset_connector_setting),
	/* retrieve_downstream_port_device tests */
	KUNIT_CASE(dm_mst_test_retrieve_downstream_no_aux),
	KUNIT_CASE(dm_mst_test_retrieve_downstream_present),
	KUNIT_CASE(dm_mst_test_retrieve_downstream_aux_error),
	/* retrieve_branch_specific_data tests */
	KUNIT_CASE(dm_mst_test_retrieve_branch_no_parent),
	KUNIT_CASE(dm_mst_test_retrieve_branch_reads_oui),
	/* dm_dp_aux_transfer_result tests */
	KUNIT_CASE(dm_mst_test_aux_result_success),
	KUNIT_CASE(dm_mst_test_aux_result_eio),
	KUNIT_CASE(dm_mst_test_aux_result_ebusy),
	KUNIT_CASE(dm_mst_test_aux_result_timeout),
	KUNIT_CASE(dm_mst_test_aux_transfer_native_read),
	KUNIT_CASE(dm_mst_test_aux_transfer_native_write),
	KUNIT_CASE(dm_mst_test_aux_transfer_partial_write),
	KUNIT_CASE(dm_mst_test_aux_transfer_error_result),
	KUNIT_CASE(dm_mst_test_aux_transfer_hpd_discon_quirk),
	KUNIT_CASE(dm_mst_test_aux_transfer_non_ack_reply),
	/* dm_dp_aux_fill_payload_flags tests */
	KUNIT_CASE(dm_mst_test_fill_payload_flags_native_write),
	KUNIT_CASE(dm_mst_test_fill_payload_flags_native_read),
	KUNIT_CASE(dm_mst_test_fill_payload_flags_i2c_read_mot),
	KUNIT_CASE(dm_mst_test_fill_payload_flags_write_status),
	/* dm_mst_msg_ready_mask tests */
	KUNIT_CASE(dm_mst_test_msg_ready_mask),
	/* dm_mst_select_esi_dpcd tests */
	KUNIT_CASE(dm_mst_test_select_esi_dpcd_legacy),
	KUNIT_CASE(dm_mst_test_select_esi_dpcd_esi),
	/* dm_handle_mst_sideband_msg_ready_event tests */
	KUNIT_CASE(dm_mst_test_sideband_msg_ready_no_ready_bits),
	KUNIT_CASE(dm_mst_test_sideband_msg_ready_read_error),
	KUNIT_CASE(dm_mst_test_sideband_msg_ready_without_mst_state),
	KUNIT_CASE(dm_mst_test_sideband_msg_ready_acks_down_rep),
	KUNIT_CASE(dm_mst_test_sideband_msg_ready_ack_write_fails),
	KUNIT_CASE(dm_mst_test_down_rep_msg_ready_wrapper),
	/* amdgpu_dm_initialize_dp_connector tests */
	KUNIT_CASE(dm_mst_test_initialize_dp_connector_edp),
	KUNIT_CASE(dm_mst_test_initialize_dp_connector_mst),
	/* dm_mst_atomic_best_encoder tests */
	KUNIT_CASE(dm_mst_test_atomic_best_encoder),
	/* dm_dp_create_fake_mst_encoders tests */
	KUNIT_CASE(dm_mst_test_create_fake_mst_encoders),
	/* dm_dp_mst_atomic_check tests */
	KUNIT_CASE(dm_mst_test_atomic_check_no_old_crtc),
	/* dm_dp_mst_detect tests */
	KUNIT_CASE(dm_mst_test_detect_unregistered),
	KUNIT_CASE(dm_mst_test_detect_reads_port_dpcd_rev),
	KUNIT_CASE(dm_mst_test_detect_unknown_dpcd_rev),
	KUNIT_CASE(dm_mst_test_detect_dpcd_read_error),
	KUNIT_CASE(dm_mst_test_detect_disconnect_releases_sink),
	/* dm_dp_mst_get_modes tests */
	KUNIT_CASE(dm_mst_test_get_modes_no_edid_adds_default_sink),
	KUNIT_CASE(dm_mst_test_get_modes_no_edid_sink_alloc_fails),
	KUNIT_CASE(dm_mst_test_get_modes_no_edid_keeps_existing_sink),
	KUNIT_CASE(dm_mst_test_get_modes_cached_edid_replaces_virtual_sink),
	KUNIT_CASE(dm_mst_test_get_modes_cached_edid_sink_alloc_fails),
	KUNIT_CASE(dm_mst_test_get_modes_restores_hdcp_properties),
	KUNIT_CASE(dm_mst_test_get_modes_reads_remote_edid),
	/* amdgpu_dm_mst_connector_late_register tests */
	KUNIT_CASE(dm_mst_test_connector_late_register),
	/* amdgpu_dm_mst_connector_early_unregister tests */
	KUNIT_CASE(dm_mst_test_connector_early_unregister_no_sink),
	KUNIT_CASE(dm_mst_test_connector_early_unregister_releases_sink),
	/* dm_dp_mst_connector_destroy tests */
	KUNIT_CASE(dm_mst_test_connector_destroy_no_sink),
	KUNIT_CASE(dm_mst_test_connector_destroy_releases_sink),
	/* CONFIG_DRM_AMD_DC_FP disabled public paths */
#if !defined(CONFIG_DRM_AMD_DC_FP)
	KUNIT_CASE(dm_mst_test_fp_guarded_public_stubs),
#endif
	{}
};

static struct kunit_suite dm_mst_types_test_suite = {
	.name = "amdgpu_dm_mst_types",
	.test_cases = dm_mst_types_test_cases,
};

kunit_test_suite(dm_mst_types_test_suite);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("KUnit tests for amdgpu_dm_mst_types");
