// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 */

#include "dm_services.h"
#include "clk_mgr.h"
#include "dccg.h"
#include "bls_executor/bls_executor.h"

static bool dc2_bls_step_supported(const struct block_sequence *step)
{
	struct clk_mgr *clk_mgr;
	struct dccg *dccg;

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
	default:
		return false;
	}
}

enum dc2_bls_exec_result dc2_bls_execute(
		const struct block_sequence *steps,
		unsigned int num_steps)
{
	const struct block_sequence *entry;
	unsigned int i;

	if (!steps || num_steps > MAX_HWSS_BLOCK_SEQUENCE_SIZE)
		return DC2_BLS_EXEC_ERROR;

	for (i = 0; i < num_steps; i++)
		if (!dc2_bls_step_supported(&steps[i]))
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
		default:
			ASSERT(false);
			return DC2_BLS_EXEC_ERROR;
		}
	}

	return DC2_BLS_EXEC_OK;
}
