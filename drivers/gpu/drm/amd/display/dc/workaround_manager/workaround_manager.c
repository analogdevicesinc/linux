// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "workaround_manager/workaround_manager.h"

static const struct workaround_manager dc2_wam_empty = {
	.registry = NULL,
	.count = 0,
};

/* BSS, not the stack: 64 block_sequence entries exceed the kernel frame limit. */
static struct block_sequence dc2_wam_saved_steps[DC2_BLS_MAX_STEPS];

const struct workaround_manager *dc2_workaround_manager_get(void)
{
	return &dc2_wam_empty;
}

enum workaround_manager_result workaround_manager_apply(
		const struct workaround_manager *wam,
		struct block_sequence_state *bls,
		const struct workaround_context *ctx)
{
	struct block_sequence *saved_steps = dc2_wam_saved_steps;
	enum workaround_manager_result result;
	unsigned int saved_num;
	unsigned int i;

	if (!wam || !bls || !bls->steps || !bls->num_steps || !ctx ||
			*bls->num_steps > MAX_HWSS_BLOCK_SEQUENCE_SIZE ||
			(wam->count && !wam->registry))
		return WAM_INVALID_INPUT;

	/* Snapshot buffer is DC2_BLS_MAX_STEPS; larger built lists stay unmodified. */
	if (*bls->num_steps > DC2_BLS_MAX_STEPS) {
		if (!wam->count)
			return WAM_OK;
		return WAM_INVALID_INPUT;
	}

	saved_num = *bls->num_steps;
	memmove(saved_steps, bls->steps,
			saved_num * sizeof(saved_steps[0]));

	for (i = 0; i < wam->count; i++) {
		const struct workaround *wa = &wam->registry[i];

		if (!wa->enabled)
			continue;
		if (!wa->is_applicable || !wa->apply) {
			memmove(bls->steps, saved_steps,
					saved_num * sizeof(saved_steps[0]));
			*bls->num_steps = saved_num;
			return WAM_INTERNAL_ERROR;
		}
		if (!wa->is_applicable(bls, ctx))
			continue;
		result = wa->apply(bls, ctx);
		if (result != WAM_OK ||
				*bls->num_steps > MAX_HWSS_BLOCK_SEQUENCE_SIZE) {
			memmove(bls->steps, saved_steps,
					saved_num * sizeof(saved_steps[0]));
			*bls->num_steps = saved_num;
			if (result != WAM_OK)
				return result;
			return WAM_SIZE_EXCEEDED;
		}
	}

	return WAM_OK;
}
