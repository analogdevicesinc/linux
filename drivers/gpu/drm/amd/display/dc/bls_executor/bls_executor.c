// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 */

#include "dm_services.h"
#include "dccg.h"
#include "bls_executor/bls_executor.h"

static bool dc2_bls_step_supported(const struct block_sequence *step)
{
	struct dccg *dccg;

	switch (step->func) {
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
