/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "bls_builder/bls_builder.h"
#include "dc_api_dispatch/api_shim.h"

enum dc2_bls_exec_result {
	DC2_BLS_EXEC_OK,
	DC2_BLS_EXEC_ERROR,
};

enum dc2_bls_exec_result dc2_bls_execute(
		struct dc2 *dc2,
		const struct block_sequence *steps,
		unsigned int num_steps);
