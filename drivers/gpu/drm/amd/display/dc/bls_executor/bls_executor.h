/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "bls_builder/bls_builder.h"

enum dc2_bls_exec_result {
	DC2_BLS_EXEC_OK,
	DC2_BLS_EXEC_ERROR,
};

enum dc2_bls_exec_result dc2_bls_execute(
		const struct block_sequence *steps,
		unsigned int num_steps);
