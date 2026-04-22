// SPDX-License-Identifier: MIT
//
// Copyright 2026 Advanced Micro Devices, Inc.

#include "dml2_pmo_dcn42.h"
#include "lib_float_math.h"
#include "dml2_debug.h"
#include "dml2_pmo_dcn4_fams2.h"

/*
 * DCN42 PMO Policy Implementation
 * This implementation provides VBlank-only strategies for 1, 2, 3, and 4 display
 * configurations, ensuring p-state watermark support in the blank period only.
 */

static const struct dml2_pmo_pstate_strategy dcn42_strategy_list_1_display[] = {
	// VBlank only
	{
		.per_stream_pstate_method = { dml2_pstate_method_vblank, dml2_pstate_method_na, dml2_pstate_method_na, dml2_pstate_method_na },
		.allow_state_increase = true,
	},
};

static const int dcn42_strategy_list_1_display_size = sizeof(dcn42_strategy_list_1_display) / sizeof(struct dml2_pmo_pstate_strategy);

static const struct dml2_pmo_pstate_strategy dcn42_strategy_list_2_display[] = {
	// VBlank only for both displays
	{
		.per_stream_pstate_method = { dml2_pstate_method_vblank, dml2_pstate_method_vblank, dml2_pstate_method_na, dml2_pstate_method_na },
		.allow_state_increase = true,
	},
};

static const int dcn42_strategy_list_2_display_size = sizeof(dcn42_strategy_list_2_display) / sizeof(struct dml2_pmo_pstate_strategy);

static const struct dml2_pmo_pstate_strategy dcn42_strategy_list_3_display[] = {
	// VBlank only for all three displays
	{
		.per_stream_pstate_method = { dml2_pstate_method_vblank, dml2_pstate_method_vblank, dml2_pstate_method_vblank, dml2_pstate_method_na },
		.allow_state_increase = true,
	},
};

static const int dcn42_strategy_list_3_display_size = sizeof(dcn42_strategy_list_3_display) / sizeof(struct dml2_pmo_pstate_strategy);

static const struct dml2_pmo_pstate_strategy dcn42_strategy_list_4_display[] = {
	// VBlank only for all four displays
	{
		.per_stream_pstate_method = { dml2_pstate_method_vblank, dml2_pstate_method_vblank, dml2_pstate_method_vblank, dml2_pstate_method_vblank },
		.allow_state_increase = true,
	},
};

static const int dcn42_strategy_list_4_display_size = sizeof(dcn42_strategy_list_4_display) / sizeof(struct dml2_pmo_pstate_strategy);

bool pmo_dcn42_test_for_pstate_support(struct dml2_pmo_test_for_pstate_support_in_out *in_out)
{
	const struct dml2_pmo_scratch *s = &in_out->instance->scratch;
	const int REQUIRED_RESERVED_TIME =
		(int)in_out->instance->soc_bb->power_management_parameters.dram_clk_change_blackout_us;
	bool p_state_supported = true;
	unsigned int stream_index;

	if (in_out->base_display_config->display_config.overrides.all_streams_blanked)
		return true;

	if (s->pmo_dcn4.cur_pstate_candidate < 0)
		return false;

	for (stream_index = 0; stream_index < in_out->base_display_config->display_config.num_streams; stream_index++) {
		if (s->pmo_dcn4.pstate_strategy_candidates[s->pmo_dcn4.cur_pstate_candidate].per_stream_pstate_method[stream_index] == dml2_pstate_method_vblank) {
			if (dcn4_get_minimum_reserved_time_us_for_planes(in_out->base_display_config, s->pmo_dcn4.stream_plane_mask[stream_index]) < REQUIRED_RESERVED_TIME ||
			    dcn4_get_vactive_pstate_margin(in_out->base_display_config, s->pmo_dcn4.stream_plane_mask[stream_index]) > 0) {
				p_state_supported = false;
				break;
			}
		} else {
			p_state_supported = false;
			break;
		}
	}

	return p_state_supported;
}

bool pmo_dcn42_initialize(struct dml2_pmo_initialize_in_out *in_out)
{
	int i = 0;
	struct dml2_pmo_instance *pmo = in_out->instance;

	unsigned int base_list_size = 0;
	const struct dml2_pmo_pstate_strategy *base_list = NULL;
	unsigned int *expanded_list_size = NULL;
	struct dml2_pmo_pstate_strategy *expanded_list = NULL;

	DML_LOG_COMP_IF_ENTER();

	pmo->soc_bb = in_out->soc_bb;
	pmo->ip_caps = in_out->ip_caps;
	pmo->mpc_combine_limit = 2;
	pmo->odm_combine_limit = 4;
	pmo->mcg_clock_table_size = in_out->mcg_clock_table_size;

	/*
	 * DCN42 does not support FAMS features like SubVP and DRR.
	 * These parameters are initialized to safe values but won't be used
	 * since our strategies only use VBlank.
	 */
	pmo->fams_params.v2.subvp.refresh_rate_limit_max = 0;
	pmo->fams_params.v2.subvp.refresh_rate_limit_min = 0;
	pmo->fams_params.v2.drr.refresh_rate_limit_max = 0;
	pmo->fams_params.v2.drr.refresh_rate_limit_min = 0;

	pmo->options = in_out->options;

	/* Generate permutations of p-state configs from base strategy list */
	for (i = 0; i < PMO_DCN4_MAX_DISPLAYS; i++) {
		switch (i+1) {
		case 1:
			if (pmo->options->override_strategy_lists[i] && pmo->options->num_override_strategies_per_list[i]) {
				base_list = pmo->options->override_strategy_lists[i];
				base_list_size = pmo->options->num_override_strategies_per_list[i];
			} else {
				base_list = dcn42_strategy_list_1_display;
				base_list_size = dcn42_strategy_list_1_display_size;
			}

			expanded_list_size = &pmo->init_data.pmo_dcn4.num_expanded_strategies_per_list[i];
			expanded_list = pmo->init_data.pmo_dcn4.expanded_strategy_list_1_display;

			break;
		case 2:
			if (pmo->options->override_strategy_lists[i] && pmo->options->num_override_strategies_per_list[i]) {
				base_list = pmo->options->override_strategy_lists[i];
				base_list_size = pmo->options->num_override_strategies_per_list[i];
			} else {
				base_list = dcn42_strategy_list_2_display;
				base_list_size = dcn42_strategy_list_2_display_size;
			}

			expanded_list_size = &pmo->init_data.pmo_dcn4.num_expanded_strategies_per_list[i];
			expanded_list = pmo->init_data.pmo_dcn4.expanded_strategy_list_2_display;

			break;
		case 3:
			if (pmo->options->override_strategy_lists[i] && pmo->options->num_override_strategies_per_list[i]) {
				base_list = pmo->options->override_strategy_lists[i];
				base_list_size = pmo->options->num_override_strategies_per_list[i];
			} else {
				base_list = dcn42_strategy_list_3_display;
				base_list_size = dcn42_strategy_list_3_display_size;
			}

			expanded_list_size = &pmo->init_data.pmo_dcn4.num_expanded_strategies_per_list[i];
			expanded_list = pmo->init_data.pmo_dcn4.expanded_strategy_list_3_display;

			break;
		case 4:
			if (pmo->options->override_strategy_lists[i] && pmo->options->num_override_strategies_per_list[i]) {
				base_list = pmo->options->override_strategy_lists[i];
				base_list_size = pmo->options->num_override_strategies_per_list[i];
			} else {
				base_list = dcn42_strategy_list_4_display;
				base_list_size = dcn42_strategy_list_4_display_size;
			}

			expanded_list_size = &pmo->init_data.pmo_dcn4.num_expanded_strategies_per_list[i];
			expanded_list = pmo->init_data.pmo_dcn4.expanded_strategy_list_4_display;

			break;
		}

		DML_ASSERT(base_list_size <= PMO_DCN4_MAX_BASE_STRATEGIES);

		/*
		 * Populate list using DCN4 FAMS2 expansion function.
		 * Since our strategies only contain VBlank methods, the expansion
		 * will not introduce any FAMS-specific logic.
		 */
		pmo_dcn4_fams2_expand_base_pstate_strategies(
				base_list,
				base_list_size,
				i + 1,
				expanded_list,
				expanded_list_size);
	}

	DML_LOG_DEBUG("%s exit with true\n", __func__);
	DML_LOG_COMP_IF_EXIT();

	return true;
}
