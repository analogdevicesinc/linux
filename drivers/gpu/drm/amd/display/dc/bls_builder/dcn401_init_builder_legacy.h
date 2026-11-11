/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "bls_builder/bls_builder.h"

/*
 * DC2_DC1_Wrapped: build stage params->current_state->hw_init_info
 * .next_stage of dcn401_init_hw into the head sequence. The stage
 * outputs land in out_targets->DCN_DECL_INITIALIZATION_LEGACY.
 */
enum bls_builder_result dcn401_build_dc1_initialization(
		struct bls_builder_declarative_params *params);
bool dcn401_dc1_initialization_check_capability(
		const struct bls_builder_check_capability_params *params);
