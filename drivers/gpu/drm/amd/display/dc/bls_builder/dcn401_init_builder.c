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
#include "bls_builder/dcn401_init_builder.h"
#include "bls_builder/dcn401_init_builder_legacy.h"

static const enum block_sequence_func dcn401_dc2_init_sequence[] = {
	LEGACY_CLK_MGR_INIT_CLOCKS,
	LEGACY_CLK_MGR_IS_DC_MODE_PRESENT,
	DCCG_INITIALIZE_CLOCKS,
	DMUB_HALT_FW,
	PSP_RELOAD_DMUB_FW,
	DMUB_GPINT_SEND_AND_WAIT,
	DMUB_SEND_DMCUB_CMD,
	DMUB_INITIALIZE_DCN,
};

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
	case DMUB_GPINT_SEND_AND_WAIT:
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
	case PSP_RELOAD_DMUB_FW: {
		struct dmub_srv *dmub;

		if (!params->dc->ctx || !params->dc->ctx->dmub_srv ||
				!params->dc->ctx->dmub_srv->dmub)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		/*
		 * An explicit caller pointer wins, including a test double
		 * whose fb_info is still empty. Otherwise use the windows
		 * dmub_srv retained from the DM's earlier hw init.
		 */
		if (params->dmub_hw_params)
			return BLS_BUILD_SUCCESS;
		dmub = params->dc->ctx->dmub_srv->dmub;
		if (!dmub->fb_info ||
				dmub->fb_info->num_fb < DMUB_WINDOW_TOTAL)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	}
	case DMUB_SEND_DMCUB_CMD:
	case DMUB_INITIALIZE_DCN:
		if (!params->dc->ctx || !params->dc->ctx->dmub_srv ||
				!params->dc->ctx->dmub_srv->dmub || !out_init)
			return BLS_BUILD_ERROR_RESOURCE_MAPPING;
		return BLS_BUILD_SUCCESS;
	default:
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;
	}
}

enum bls_builder_result dcn401_build_clk_mgr_init(
		struct clk_mgr *clk_mgr, struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.legacy_clk_mgr_init_clocks_params.clk_mgr = clk_mgr;
	if (!bls_append(sequence, LEGACY_CLK_MGR_INIT_CLOCKS, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

enum bls_builder_result dcn401_build_dc_mode_read(
		struct clk_mgr *clk_mgr, bool *out_dcmode_power_limits_present,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.legacy_clk_mgr_is_dc_mode_present_params.clk_mgr = clk_mgr;
	bls_params.legacy_clk_mgr_is_dc_mode_present_params
			.out_dcmode_power_limits_present =
			out_dcmode_power_limits_present;
	if (!bls_append(sequence, LEGACY_CLK_MGR_IS_DC_MODE_PRESENT,
			&bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

enum bls_builder_result dcn401_build_dccg_init(
		struct dccg *dccg, struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.dccg_initialize_clocks_params.dccg = dccg;
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

static const struct dmub_srv_hw_params *dcn401_psp_hw_params(
		struct bls_builder_declarative_params *params)
{
	struct dmub_srv *dmub;

	if (params->dmub_hw_params)
		return params->dmub_hw_params;

	dmub = params->dc->ctx->dmub_srv->dmub;
	params->owned_dmub_fb_info = *dmub->fb_info;
	memset(&params->owned_dmub_hw_params, 0,
			sizeof(params->owned_dmub_hw_params));
	params->owned_dmub_hw_params.fb_info = &params->owned_dmub_fb_info;
	params->owned_dmub_hw_params.soc_fb_info = dmub->soc_fb_info;
	params->owned_dmub_hw_params.psp_version = dmub->psp_version;
	return &params->owned_dmub_hw_params;
}

static inline enum bls_builder_result dcn401_build_psp_reload(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};

	bls_params.psp_reload_dmub_fw_params.dmub =
			params->dc->ctx->dmub_srv->dmub;
	bls_params.psp_reload_dmub_fw_params.hw_params =
			dcn401_psp_hw_params(params);
	if (!bls_append(sequence, PSP_RELOAD_DMUB_FW, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static inline enum bls_builder_result dcn401_build_dmub_fw_version(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};
	struct bls_builder_output_init *out_init =
			params->out_targets->DCN_DECL_INITIALIZATION;

	bls_params.dmub_gpint_params.command_code =
			DMUB_GPINT__GET_FW_VERSION;
	bls_params.dmub_gpint_params.param = 0;
	bls_params.dmub_gpint_params.response =
			&out_init->dmub_abi.fw_version;
	if (!bls_append(sequence, DMUB_GPINT_SEND_AND_WAIT, &bls_params))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

bool dcn401_add_dmub_cmd(struct block_sequence_state *sequence,
		struct dc_context *ctx, union dmub_rb_cmd *cmd,
		enum dm_dmub_wait_type wait_type, bool continue_on_error)
{
	union block_sequence_params bls_params = {0};

	bls_params.send_dmcub_cmd_params.ctx = ctx;
	bls_params.send_dmcub_cmd_params.cmd = cmd;
	bls_params.send_dmcub_cmd_params.wait_type = wait_type;
	bls_params.send_dmcub_cmd_params.continue_on_error = continue_on_error;
	return bls_append(sequence, DMUB_SEND_DMCUB_CMD, &bls_params);
}

void dcn401_fill_query_feature_caps_cmd(union dmub_rb_cmd *cmd)
{
	memset(cmd, 0, sizeof(*cmd));
	cmd->query_feature_caps.header.type = DMUB_CMD__QUERY_FEATURE_CAPS;
	cmd->query_feature_caps.header.sub_type = 0;
	cmd->query_feature_caps.header.ret_status = 1;
	cmd->query_feature_caps.header.payload_bytes =
			sizeof(struct dmub_cmd_query_feature_caps_data);
}

static inline enum bls_builder_result dcn401_build_dmub_feature_caps(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	struct bls_builder_output_init *out_init =
			params->out_targets->DCN_DECL_INITIALIZATION;
	union dmub_rb_cmd *cmd = &out_init->dmub_query_feature_caps_cmd;

	dcn401_fill_query_feature_caps_cmd(cmd);
	if (!dcn401_add_dmub_cmd(sequence, params->dc->ctx, cmd,
			DM_DMUB_WAIT_TYPE_WAIT_WITH_REPLY, false))
		return BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE;

	return BLS_BUILD_SUCCESS;
}

static inline enum bls_builder_result dcn401_build_dmub_initialize_dcn(
		struct bls_builder_declarative_params *params,
		struct block_sequence_state *sequence)
{
	union block_sequence_params bls_params = {0};
	struct bls_builder_output_init *out_init =
			params->out_targets->DCN_DECL_INITIALIZATION;
	union dmub_rb_cmd *cmd = &out_init->dmub_initialize_dcn_cmd;

	/*
	 * No Initialize DCN opcode exists in dmub_cmd.h. A caller-supplied
	 * non-zero type is preserved for tests; zero stays an undefined
	 * command and cannot complete a successful build.
	 */
	if (!cmd->cmd_common.header.type)
		memset(cmd, 0, sizeof(*cmd));
	bls_params.send_dmcub_cmd_params.ctx = params->dc->ctx;
	bls_params.send_dmcub_cmd_params.cmd = cmd;
	bls_params.send_dmcub_cmd_params.wait_type =
			DM_DMUB_WAIT_TYPE_WAIT_WITH_REPLY;
	if (!bls_append(sequence, DMUB_INITIALIZE_DCN, &bls_params))
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
		return dcn401_build_clk_mgr_init(params->dc->clk_mgr, sequence);
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		return dcn401_build_dc_mode_read(params->dc->clk_mgr,
				&params->out_targets->DCN_DECL_INITIALIZATION
				->dcmode_power_limits_present, sequence);
	case DCCG_INITIALIZE_CLOCKS:
		return dcn401_build_dccg_init(params->dc->res_pool->dccg,
				sequence);
	case DMUB_HALT_FW:
		return dcn401_build_dmub_halt_fw(params, sequence);
	case PSP_RELOAD_DMUB_FW:
		return dcn401_build_psp_reload(params, sequence);
	case DMUB_GPINT_SEND_AND_WAIT:
		return dcn401_build_dmub_fw_version(params, sequence);
	case DMUB_SEND_DMCUB_CMD:
		return dcn401_build_dmub_feature_caps(params, sequence);
	case DMUB_INITIALIZE_DCN:
		return dcn401_build_dmub_initialize_dcn(params, sequence);
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
	for (i = 0; i < ARRAY_SIZE(dcn401_dc2_init_sequence); i++) {
		result = dcn401_validate_initialization_step(params,
				dcn401_dc2_init_sequence[i]);
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
		out_init->dmub_abi.fw_version = 0;
		memset(&out_init->dmub_abi.feature_caps, 0,
				sizeof(out_init->dmub_abi.feature_caps));
	}

	/*
	 * LEGACY/pmfw-msg: preserve the clock-manager entry points until
	 * PMFW Messenger can express the same calls, order, and output.
	 */
	for (i = 0; i < ARRAY_SIZE(dcn401_dc2_init_sequence); i++) {
		result = dcn401_build_initialization_step(params, &sequence,
				dcn401_dc2_init_sequence[i]);
		if (result != BLS_BUILD_SUCCESS)
			return result;
	}

	if (out_init &&
			out_init->dmub_initialize_dcn_cmd.cmd_common.header.type)
		return BLS_BUILD_SUCCESS;

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
	static const struct bls_builder legacy_initialization_builder = {
		.name = DCN_DECL_INITIALIZATION_LEGACY,
		.kind = BLS_BUILDER_DECLARATIVE,
		.check_capability = dcn401_dc1_initialization_check_capability,
		.build.declarative = dcn401_build_dc1_initialization,
	};

	switch (name) {
	case DCN_DECL_INITIALIZATION:
		return &initialization_builder;
	case DCN_DECL_INITIALIZATION_LEGACY:
		return &legacy_initialization_builder;
	default:
		return NULL;
	}
}
