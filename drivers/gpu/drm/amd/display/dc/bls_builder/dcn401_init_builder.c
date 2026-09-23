// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "core_types.h"
#include "clk_mgr.h"
#include "dccg.h"
#include "resource.h"
#include "dc_dmub_srv.h"
#include "dmub_cmd.h"
#include "bls_builder/bls_builder_internal.h"

static const enum block_sequence_func dcn401_dc1_init_sequence[] = {
	LEGACY_CLK_MGR_INIT_CLOCKS,
	LEGACY_CLK_MGR_IS_DC_MODE_PRESENT,
	DCCG_INITIALIZE_CLOCKS,
	DMUB_HALT_FW,
};

static bool bls_append(struct block_sequence_state *sequence,
		enum block_sequence_func func,
		const union block_sequence_params *params)
{
	struct block_sequence *entry;

	if (!sequence || !sequence->steps || !sequence->num_steps ||
			!params || *sequence->num_steps >=
			DC2_BLS_MAX_STEPS)
		return false;

	entry = &sequence->steps[*sequence->num_steps];
	entry->params = *params;
	entry->func = func;
	(*sequence->num_steps)++;
	return true;
}

static enum bls_builder_result dcn401_check_initialization_support(
		const struct bls_builder_check_capability_params *params)
{
	struct dc *dc;

	if (!params || !params->dc)
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	dc = params->dc;
	if (!dc->ctx || !dc->res_pool || !dc->res_pool->dccg)
		return BLS_BUILD_ERROR_RESOURCE_MAPPING;
	if (dc->config.seamless_boot_edp_requested)
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static bool dcn401_initialization_check_capability(
		const struct bls_builder_check_capability_params *params)
{
	return dcn401_check_initialization_support(params) ==
			BLS_BUILD_SUCCESS;
}

static enum bls_builder_result dcn401_validate_initialization_step(
		const struct bls_builder_declarative_params *params,
		enum block_sequence_func func)
{
	struct bls_builder_output_init *out_init = NULL;
	struct clk_mgr *clk_mgr = params->dc->clk_mgr;
	struct dccg *dccg = params->dc->res_pool->dccg;

	if (params->out_targets)
		out_init = params->out_targets->DCN_DECL_INITIALIZATION;

	switch (func) {
	case LEGACY_CLK_MGR_INIT_CLOCKS:
		if (!clk_mgr || !clk_mgr->funcs ||
				!clk_mgr->funcs->init_clocks)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		if (!clk_mgr || !clk_mgr->funcs ||
				!clk_mgr->funcs->is_dc_mode_present ||
				!out_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	case DCCG_INITIALIZE_CLOCKS:
		if (!dccg->funcs || !dccg->funcs->dccg_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	case DMUB_HALT_FW: {
		struct dc_context *ctx = params->dc->ctx;
		struct dmub_srv *dmub;

		if (!ctx || !ctx->dmub_srv || !ctx->dmub_srv->dmub || !out_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		dmub = ctx->dmub_srv->dmub;
		if (!dmub->sw_init || !dmub->hw_funcs.set_gpint ||
				!dmub->hw_funcs.is_gpint_acked ||
				!dmub->hw_funcs.get_gpint_response)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	}
	default:
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
	}
}

static inline enum bls_builder_result dcn401_build_clk_mgr_init(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.legacy_clk_mgr_init_clocks_params.clk_mgr =
			params->dc->clk_mgr;
	if (!bls_append(sequence, LEGACY_CLK_MGR_INIT_CLOCKS, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static inline enum bls_builder_result dcn401_build_dc_mode_read(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};
	struct bls_builder_output_init *out_init =
			params->out_targets->DCN_DECL_INITIALIZATION;

	bls_params.legacy_clk_mgr_is_dc_mode_present_params.clk_mgr =
			params->dc->clk_mgr;
	bls_params.legacy_clk_mgr_is_dc_mode_present_params
			.out_dcmode_power_limits_present =
			&out_init->dcmode_power_limits_present;
	if (!bls_append(sequence, LEGACY_CLK_MGR_IS_DC_MODE_PRESENT,
			&bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static inline enum bls_builder_result dcn401_build_dccg_init(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.dccg_initialize_clocks_params.dccg =
			params->dc->res_pool->dccg;
	if (!bls_append(sequence, DCCG_INITIALIZE_CLOCKS, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static inline enum bls_builder_result dcn401_build_dmub_halt_fw(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};
	struct bls_builder_output_init *out_init =
			params->out_targets->DCN_DECL_INITIALIZATION;

	bls_params.dmub_gpint_params.command_code = DMUB_GPINT__STOP_FW;
	bls_params.dmub_gpint_params.param = 0;
	bls_params.dmub_gpint_params.response =
			&out_init->dmub_stop_fw_response;
	if (!bls_append(sequence, DMUB_HALT_FW, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static enum bls_builder_result dcn401_build_initialization_step(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence,
		enum block_sequence_func func)
{
	switch (func) {
	case LEGACY_CLK_MGR_INIT_CLOCKS:
		return dcn401_build_clk_mgr_init(params, sequence);
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		return dcn401_build_dc_mode_read(params, sequence);
	case DCCG_INITIALIZE_CLOCKS:
		return dcn401_build_dccg_init(params, sequence);
	case DMUB_HALT_FW:
		return dcn401_build_dmub_halt_fw(params, sequence);
	default:
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
	}
}

static enum bls_builder_result dcn401_build_initialization(
		struct bls_builder_declarative_params *params)
{
	struct bls_builder_check_capability_params check_params = {0};
	enum bls_builder_result result;
	struct block_sequence_state sequence;
	struct bls_builder_output_init *out_init = NULL;
	unsigned int i;

	if (!params)
		return dcn401_check_initialization_support(NULL);

	check_params.dc = params->dc;
	check_params.current_state = params->current_state;
	check_params.new_state = params->new_state;
	result =
			dcn401_check_initialization_support(&check_params);
	if (result != BLS_BUILD_SUCCESS)
		return result;

	params->head_num_steps = 0;
	params->tail_num_steps = 0;
	for (i = 0; i < ARRAY_SIZE(dcn401_dc1_init_sequence); i++) {
		result = dcn401_validate_initialization_step(params,
				dcn401_dc1_init_sequence[i]);
		if (result != BLS_BUILD_SUCCESS)
			return result;
	}

	sequence.steps = params->head_sequence;
	sequence.num_steps = &params->head_num_steps;
	if (params->out_targets)
		out_init = params->out_targets->DCN_DECL_INITIALIZATION;
	if (out_init) {
		out_init->dcmode_power_limits_present = false;
		out_init->dmub_stop_fw_response = 0;
	}

	/*
	 * LEGACY/pmfw-msg: preserve the clock-manager entry points until
	 * PMFW Messenger can express the same calls, order, and output.
	 */
	for (i = 0; i < ARRAY_SIZE(dcn401_dc1_init_sequence); i++) {
		result = dcn401_build_initialization_step(params, &sequence,
				dcn401_dc1_init_sequence[i]);
		if (result != BLS_BUILD_SUCCESS)
			return result;
	}

	return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
}

const struct bls_builder *dcn401_bls_builder_create(
		enum bls_builder_name name)
{
	static const struct bls_builder initialization_builder = {
		.name = DCN_DECL_INITIALIZATION,
		.kind = BLS_BUILDER_DECLARATIVE,
		.check_capability = dcn401_initialization_check_capability,
		.build.declarative = dcn401_build_initialization,
	};

	if (name != DCN_DECL_INITIALIZATION)
		return NULL;

	return &initialization_builder;
}
