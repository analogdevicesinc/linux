// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 */

#include "dm_services.h"
#include "clk_mgr.h"
#include "dccg.h"
#include "core_types.h"
#include "dc_dmub_srv.h"
#include "bls_executor/bls_executor.h"
#include "dmub_messenger/dmub_messenger.h"

static bool dc2_bls_step_supported(const struct dc2 *dc2,
		const struct block_sequence *step)
{
	const struct dc *dc = dc2 ? &dc2->dc : NULL;
	struct dmub_srv *dmub;
	struct clk_mgr *clk_mgr;
	struct dccg *dccg;
	struct dc_context *ctx;

	switch (step->func) {
	case LEGACY_CLK_MGR_INIT_CLOCKS:
		clk_mgr =
				step->params.legacy_clk_mgr_init_clocks_params.clk_mgr;
		return clk_mgr && clk_mgr->funcs &&
				clk_mgr->funcs->init_clocks;
	case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
		clk_mgr = step->params
				.legacy_clk_mgr_is_dc_mode_present_params.clk_mgr;
		return clk_mgr && clk_mgr->funcs &&
				clk_mgr->funcs->is_dc_mode_present &&
				step->params
				.legacy_clk_mgr_is_dc_mode_present_params
				.out_dcmode_power_limits_present;
	case DCCG_INITIALIZE_CLOCKS:
		dccg = step->params.dccg_initialize_clocks_params.dccg;
		return dccg && dccg->funcs && dccg->funcs->dccg_init;
	case DMUB_SEND_DMCUB_CMD:
		ctx = step->params.send_dmcub_cmd_params.ctx;
		if (!ctx || !ctx->dmub_srv || !ctx->dmub_srv->dmub ||
				!step->params.send_dmcub_cmd_params.cmd ||
				(ctx->dmub_srv->idle_allowed && !ctx->dc))
			return false;
		return true;
	case DMUB_GPINT_SEND_AND_WAIT:
		if (!dc || !dc->ctx || !dc->ctx->dmub_srv ||
				!dc->ctx->dmub_srv->dmub ||
				(dc->ctx->dmub_srv->idle_allowed &&
				!dc->ctx->dc))
			return false;
		dmub = dc->ctx->dmub_srv->dmub;
		return dmub->sw_init && dmub->hw_funcs.set_gpint &&
				dmub->hw_funcs.is_gpint_acked &&
				dmub->hw_funcs.get_gpint_response &&
				step->params.dmub_gpint_params.response;
	default:
		return false;
	}
}

static enum dc2_bls_exec_result dc2_bls_exec_dmub_send(
		const struct block_sequence *entry)
{
	struct dmub_messenger messenger;
	struct dmub_messenger_init_data init_data;
	struct dmub_send_cmd_params params;
	struct dc_context *ctx;

	ctx = entry->params.send_dmcub_cmd_params.ctx;
	init_data.ctx = ctx;
	init_data.dc_dmub_srv = ctx->dmub_srv;
	params.cmd = entry->params.send_dmcub_cmd_params.cmd;
	params.wait_type = entry->params.send_dmcub_cmd_params.wait_type;

	dmub_messenger_construct(&messenger, &init_data);
	if (messenger.funcs->inbox_cmd_send_and_wait(&messenger, &params) !=
			DMUB_MSG_OK)
		return DC2_BLS_EXEC_ERROR;

	return DC2_BLS_EXEC_OK;
}

static enum dc2_bls_exec_result dc2_bls_exec_dmub_gpint(
		struct dc2 *dc2, const struct block_sequence *entry)
{
	struct dmub_messenger messenger;
	struct dmub_messenger_init_data init_data;

	init_data.ctx = dc2->dc.ctx;
	init_data.dc_dmub_srv = dc2->dc.ctx->dmub_srv;
	dmub_messenger_construct(&messenger, &init_data);
	if (messenger.funcs->gpint_send_and_wait(&messenger,
			&entry->params.dmub_gpint_params) != DMUB_MSG_OK)
		return DC2_BLS_EXEC_ERROR;

	return DC2_BLS_EXEC_OK;
}

enum dc2_bls_exec_result dc2_bls_execute(
		struct dc2 *dc2,
		const struct block_sequence *steps,
		unsigned int num_steps)
{
	const struct block_sequence *entry;
	unsigned int i;

	if (!steps || num_steps > MAX_HWSS_BLOCK_SEQUENCE_SIZE)
		return DC2_BLS_EXEC_ERROR;

	for (i = 0; i < num_steps; i++)
		if (!dc2_bls_step_supported(dc2, &steps[i]))
			return DC2_BLS_EXEC_ERROR;

	for (i = 0; i < num_steps; i++) {
		entry = &steps[i];
		switch (entry->func) {
		/*
		 * LEGACY/pmfw-msg: remove both cases when PMFW Messenger can
		 * reproduce the same calls, order, and output.
		 */
		case LEGACY_CLK_MGR_INIT_CLOCKS:
			entry->params.legacy_clk_mgr_init_clocks_params
					.clk_mgr->funcs->init_clocks(
					entry->params
					.legacy_clk_mgr_init_clocks_params.clk_mgr);
			break;
		case LEGACY_CLK_MGR_IS_DC_MODE_PRESENT:
			*entry->params
					.legacy_clk_mgr_is_dc_mode_present_params
					.out_dcmode_power_limits_present =
					entry->params
					.legacy_clk_mgr_is_dc_mode_present_params
					.clk_mgr->funcs->is_dc_mode_present(
					entry->params
					.legacy_clk_mgr_is_dc_mode_present_params
					.clk_mgr);
			break;
		case DCCG_INITIALIZE_CLOCKS:
			entry->params.dccg_initialize_clocks_params.dccg->funcs->dccg_init(
					entry->params.dccg_initialize_clocks_params.dccg);
			break;
		case DMUB_SEND_DMCUB_CMD:
			if (dc2_bls_exec_dmub_send(entry) != DC2_BLS_EXEC_OK)
				return DC2_BLS_EXEC_ERROR;
			break;
		case DMUB_GPINT_SEND_AND_WAIT:
			if (dc2_bls_exec_dmub_gpint(dc2, entry) != DC2_BLS_EXEC_OK)
				return DC2_BLS_EXEC_ERROR;
			break;
		default:
			ASSERT(false);
			return DC2_BLS_EXEC_ERROR;
		}
	}

	return DC2_BLS_EXEC_OK;
}
