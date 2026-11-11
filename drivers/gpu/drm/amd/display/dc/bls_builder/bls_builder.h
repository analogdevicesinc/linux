/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "dc.h"
#include "dmub_cmd.h"
#include "dmub/dmub_srv.h"
#include "hwss/hw_sequencer.h"
#include "bls_builder/dmub_abi.h"
#include "bls_builder/bls_builder_legacy_init.h"

#define DC2_BLS_MAX_STEPS 64

enum bls_builder_result {
	BLS_BUILD_SUCCESS,
	BLS_BUILD_ERROR_UNSUPPORTED_STATE_UPDATE,
	BLS_BUILD_ERROR_RESOURCE_MAPPING,
	BLS_BUILD_ERROR_DMUB_ABI_UNRESOLVED,
};

enum bls_builder_name {
	DCN_DECL_INITIALIZATION,
	DCN_DECL_INITIALIZATION_LEGACY,
	BLS_BUILDER_NAME_COUNT,
};

enum bls_builder_kind {
	BLS_BUILDER_DECLARATIVE,
	BLS_BUILDER_IMPERATIVE,
};

struct bls_builder_output_init {
	unsigned int xtalin_clock_inKhz;
	unsigned int dccg_ref_clock_inKhz;
	unsigned int dchub_ref_clock_inKhz;
	unsigned int dchub_ref_clock_inKhz_before;
	unsigned int backlight;
	bool dcmode_power_limits_present;
	unsigned int dmub_stop_fw_response;
	struct dmub_abi dmub_abi;
	union dmub_rb_cmd dmub_query_feature_caps_cmd;
	union dmub_rb_cmd dmub_initialize_dcn_cmd;
};

struct bls_builder_output_targets {
	union {
		struct bls_builder_output_init *DCN_DECL_INITIALIZATION;
		struct bls_builder_legacy_hw_init_info *DCN_DECL_INITIALIZATION_LEGACY;
	};
};

struct bls_builder_check_capability_params {
	struct dc *dc;
	const struct dc_state *current_state;
	const struct dc_state *new_state;
};

struct bls_builder_declarative_params {
	struct dc *dc;
	const struct dc_state *current_state;
	const struct dc_state *new_state;
	const struct dmub_abi *dmub_abi;
	const struct dmub_srv_hw_params *dmub_hw_params;
	/*
	 * Filled when the caller does not pass dmub_hw_params and dmub_srv
	 * already retained framebuffer windows. fb_info on dmub_srv is
	 * const, so the window table is copied here before the PSP entry
	 * points at owned_dmub_hw_params.
	 */
	struct dmub_srv_hw_params owned_dmub_hw_params;
	struct dmub_srv_fb_info owned_dmub_fb_info;
	struct bls_builder_output_targets *out_targets;
	struct block_sequence head_sequence[DC2_BLS_MAX_STEPS];
	unsigned int head_num_steps;
	struct block_sequence tail_sequence[DC2_BLS_MAX_STEPS];
	unsigned int tail_num_steps;
};

typedef bool (*bls_builder_check_capability)(
		const struct bls_builder_check_capability_params *params);

typedef enum bls_builder_result (*bls_builder_build_declarative)(
		struct bls_builder_declarative_params *params);

struct bls_builder {
	enum bls_builder_name name;
	enum bls_builder_kind kind;
	bls_builder_check_capability check_capability;
	union {
		bls_builder_build_declarative declarative;
	} build;
};

const struct bls_builder *bls_builder_create(struct dc *dc,
		enum bls_builder_name name);
