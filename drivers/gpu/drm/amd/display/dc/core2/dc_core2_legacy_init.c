/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "dc.h"
#include "core_types.h"
#include "clk_mgr.h"
#include "dc_dmub_srv.h"
#include "bls_builder/bls_builder.h"
#include "bls_executor/bls_executor.h"
#include "workaround_manager/workaround_manager.h"
#include "core2/dc_core2_legacy_init.h"
#include "dc_api_dispatch/api_shim.h"

/*
 * dcn401_init_hw marks a link active when its DIG is already enabled,
 * and FEC enabled when it is also active. Applied after each stage that
 * reads the DIG or FEC state, before any later stage runs.
 */
static void core2_apply_dc1_link_states(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *info)
{
	unsigned int i;

	for (i = 0; i < dc->link_count; i++) {
		struct dc_link *link = dc->links[i];

		if (!info->s3.link_dig_enabled[i])
			continue;
		link->link_status.link_active = true;
		link->phy_state.symclk_state = SYMCLK_ON_TX_ON;
		if (info->s4.link_fec_active[i])
			link->fec_state = dc_link_fec_enabled;
	}
}

/*
 * The HUBBUB stage resolved the DMUB feature caps. Take the query reply
 * as dc_dmub_srv_query_caps_cmd does, retain dmub_abi for later builds,
 * then apply the caps and the bounding-box update, all before the next
 * stage runs.
 */
static void core2_apply_dmub_caps(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *info)
{
	const union dmub_rb_cmd *reply = &info->s11.dmub_query_feature_caps_cmd;
	struct dmub_srv *dmub = dc->ctx->dmub_srv->dmub;
	struct resource_pool *pool = dc->res_pool;
	struct dc2 *dc2 = TO_DC2(dc);
	const struct dmub_feature_caps *caps = &dc2->dmub_abi.feature_caps;
	bool dchub_ref_freq_changed;

	/*
	 * S-INT/dmub-msg: the rest of DC reads feature_caps from dmub_srv.
	 * Remove when those readers take dmub_abi.
	 */
	if (info->s11.dmub_caps_query_sent &&
			reply->query_feature_caps.header.ret_status == 0)
		memcpy(&dmub->feature_caps,
				&reply->query_feature_caps.query_feature_caps_data,
				sizeof(dmub->feature_caps));
	memset(&dc2->dmub_abi, 0, sizeof(dc2->dmub_abi));
	dc2->dmub_abi.feature_caps = dmub->feature_caps;
	dc2->dmub_abi_resolved = true;

	dc->caps.dmub_caps.psr = caps->psr;
	dc->caps.dmub_caps.mclk_sw = caps->fw_assisted_mclk_switch_ver > 0;
	dc->caps.dmub_caps.fams_ver = caps->fw_assisted_mclk_switch_ver;

	/* sw and fw FAMS versions must match for support */
	dc->debug.fams2_config.bits.enable &=
			dc->caps.dmub_caps.fams_ver == dc->debug.fams_version.ver;
	dchub_ref_freq_changed = pool->ref_clocks.dchub_ref_clock_inKhz / 1000 !=
			info->s2.dchub_ref_clock_mhz_before;
	if ((!dc->debug.fams2_config.bits.enable || dchub_ref_freq_changed) &&
			pool->funcs->update_bw_bounding_box &&
			dc->clk_mgr && dc->clk_mgr->bw_params)
		pool->funcs->update_bw_bounding_box(dc, dc->clk_mgr->bw_params);
}

/*
 * Move the output of the stage just executed into dc_state.hw_init_info,
 * where the next build reads it, then apply what dcn401_init_hw left on
 * dc by this point: the reference clocks, DC-mode limits, link state and
 * DMUB caps.
 */
static void core2_retain_init_stage(struct dc *dc,
		const struct bls_builder_legacy_hw_init_info *out)
{
	struct bls_builder_legacy_hw_init_info *info =
			&dc->current_state->hw_init_info;
	struct resource_pool *pool = dc->res_pool;

	switch (out->next_stage - 1) {
	case LEGACY_INIT_STAGE_CLOCKS:
		info->s1 = out->s1;
		pool->ref_clocks.xtalin_clock_inKhz = info->s1.xtalin_clock_inKhz;
		pool->ref_clocks.dccg_ref_clock_inKhz =
				info->s1.dccg_ref_clock_inKhz;
		if (dc->clk_mgr && dc->clk_mgr->funcs &&
				dc->clk_mgr->funcs->init_clocks)
			dc->caps.dcmode_power_limits_present =
					info->s1.dcmode_power_limits_present;
		break;
	case LEGACY_INIT_STAGE_DCHUB_REF:
		info->s2 = out->s2;
		pool->ref_clocks.dchub_ref_clock_inKhz =
				info->s2.dchub_ref_clock_inKhz;
		break;
	case LEGACY_INIT_STAGE_LINK_ENC:
		info->s3 = out->s3;
		core2_apply_dc1_link_states(dc, info);
		break;
	case LEGACY_INIT_STAGE_FEC:
		info->s4 = out->s4;
		core2_apply_dc1_link_states(dc, info);
		break;
	case LEGACY_INIT_STAGE_BLANK:
		info->s5 = out->s5;
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN:
		info->s6 = out->s6;
		break;
	case LEGACY_INIT_STAGE_MIN_CLOCKS:
		info->s7 = out->s7;
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN_EDP0:
		info->s8 = out->s8;
		break;
	case LEGACY_INIT_STAGE_POWER_DOWN_EDP1:
		info->s9 = out->s9;
		break;
	case LEGACY_INIT_STAGE_GATING:
		info->s10 = out->s10;
		break;
	case LEGACY_INIT_STAGE_HUBBUB:
		info->s11 = out->s11;
		if (info->s11.dmub_caps_read)
			core2_apply_dmub_caps(dc, info);
		break;
	case LEGACY_INIT_STAGE_POWER_STATE:
		info->s12 = out->s12;
		break;
	}
	info->next_stage = out->next_stage;
	info->init_done = out->init_done;
}

/*
 * Build, workaround, execute. Execute only a successful build and a
 * workaround pass that kept the stage within the sequence storage, and
 * do not fall back to legacy hwseq.
 */
static bool core2_run_init_stage(struct dc *dc,
		const struct bls_builder *builder,
		const struct workaround_manager *wam,
		struct bls_builder_declarative_params *params)
{
	struct workaround_context wa_ctx = {0};
	struct block_sequence_state sequence;
	enum dc2_bls_exec_result result;

	if (builder->build.declarative(params) != BLS_BUILD_SUCCESS)
		return false;

	sequence.steps = params->head_sequence;
	sequence.num_steps = &params->head_num_steps;
	wa_ctx.current_state = dc->current_state;
	wa_ctx.builder_name = builder->name;
	wa_ctx.dcn_version = dc->ctx->dce_version;
	wa_ctx.bug_wa = &dc->work_arounds;
	wa_ctx.debug = &dc->debug;
	if (workaround_manager_apply(wam, &sequence, &wa_ctx) != WAM_OK ||
			params->head_num_steps > DC2_BLS_MAX_STEPS)
		return false;

	/* dcn401_init_hw ignores the DMUB sends that continue on error. */
	result = dc2_bls_execute(TO_DC2(dc), params->head_sequence,
			params->head_num_steps);
	return result == DC2_BLS_EXEC_OK ||
			result == DC2_BLS_EXEC_CONTINUED_ON_ERROR;
}

bool core2_run_legacy_hardware_init(struct dc *dc,
		const struct workaround_manager *wam)
{
	struct bls_builder_declarative_params *params;
	struct bls_builder_legacy_hw_init_info *out;
	struct bls_builder_legacy_hw_init_info *info;
	struct bls_builder_output_targets out_targets = {0};
	const struct bls_builder *builder;
	unsigned int stage;

	if (!dc || !dc->current_state || !wam)
		return false;

	info = &dc->current_state->hw_init_info;
	memset(info, 0, sizeof(*info));
	info->next_stage = LEGACY_INIT_STAGE_CLOCKS;
	builder = bls_builder_create(dc, DCN_DECL_INITIALIZATION_LEGACY);
	if (!builder || builder->kind != BLS_BUILDER_DECLARATIVE ||
			!builder->build.declarative)
		return false;
	TO_DC2(dc)->dmub_abi_resolved = false;

	/*
	 * The declarative params hold 128 block-sequence entries, and the
	 * stage outputs hold DMUB command buffers. Keep both off the stack
	 * so the kernel frame stays under the 2048-byte limit.
	 */
	params = kvzalloc(sizeof(*params), GFP_KERNEL);
	out = kvzalloc(sizeof(*out), GFP_KERNEL);
	if (!params || !out)
		goto out;

	params->dc = dc;
	params->current_state = dc->current_state;
	out_targets.DCN_DECL_INITIALIZATION_LEGACY = out;
	params->out_targets = &out_targets;
	/* The builder reports init_done no later than its last stage. */
	for (stage = 0; stage < LEGACY_INIT_STAGE_COUNT && !info->init_done;
			stage++) {
		if (!core2_run_init_stage(dc, builder, wam, params))
			break;
		core2_retain_init_stage(dc, out);
	}
	info->init_success = info->init_done;

out:
	info->init_done = true;
	kvfree(params);
	kvfree(out);
	return info->init_success;
}
