/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "bls_builder/bls_builder.h"

struct clk_mgr;
struct dccg;
struct dc_context;

static inline bool bls_append(struct block_sequence_state *sequence,
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

enum bls_builder_result dcn401_build_clk_mgr_init(
		struct clk_mgr *clk_mgr, struct block_sequence_state *sequence);
enum bls_builder_result dcn401_build_dc_mode_read(
		struct clk_mgr *clk_mgr, bool *out_dcmode_power_limits_present,
		struct block_sequence_state *sequence);
enum bls_builder_result dcn401_build_dccg_init(
		struct dccg *dccg, struct block_sequence_state *sequence);
bool dcn401_add_dmub_cmd(struct block_sequence_state *sequence,
		struct dc_context *ctx, union dmub_rb_cmd *cmd,
		enum dm_dmub_wait_type wait_type, bool continue_on_error);
void dcn401_fill_query_feature_caps_cmd(union dmub_rb_cmd *cmd);
