/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "bls_builder/bls_builder.h"

enum workaround_manager_result {
	WAM_OK,
	WAM_INVALID_INPUT,
	WAM_SIZE_EXCEEDED,
	WAM_INTERNAL_ERROR,
};

struct workaround_context {
	const struct dc_state *current_state;
	const struct dc_state *target;
	enum bls_builder_name builder_name;
	unsigned int dcn_version;
	const struct dc_bug_wa *bug_wa;
	const struct dc_debug_options *debug;
};

struct workaround {
	const char *name;
	bool enabled;
	bool (*is_applicable)(const struct block_sequence_state *bls,
			const struct workaround_context *ctx);
	enum workaround_manager_result (*apply)(
			struct block_sequence_state *bls,
			const struct workaround_context *ctx);
};

struct workaround_manager {
	const struct workaround *registry;
	unsigned int count;
};

const struct workaround_manager *dc2_workaround_manager_get(void);

enum workaround_manager_result workaround_manager_apply(
		const struct workaround_manager *wam,
		struct block_sequence_state *bls,
		const struct workaround_context *ctx);
