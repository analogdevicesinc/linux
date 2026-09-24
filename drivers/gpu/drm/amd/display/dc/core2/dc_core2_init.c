// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "dc.h"
#include "bls_builder/bls_builder.h"
#include "bls_executor/bls_executor.h"
#include "core2/dc_core2_init.h"
#include "dc_api_dispatch/api_shim.h"

void core2_hardware_init(struct dc *dc)
{
	struct bls_builder_declarative_params *params;
	struct bls_builder_output_init *out_init;
	struct bls_builder_output_targets out_targets = {0};
	const struct bls_builder *builder;
	enum bls_builder_result result;

	if (!dc)
		return;

	builder = bls_builder_create(dc, DCN_DECL_INITIALIZATION);
	if (!builder || builder->kind != BLS_BUILDER_DECLARATIVE ||
			!builder->build.declarative)
		return;

	/*
	 * The declarative params hold 128 block-sequence entries, and the
	 * init output holds DMUB command buffers. Keep both off the stack
	 * so the kernel frame stays under the 2048-byte limit.
	 */
	params = kvzalloc(sizeof(*params), GFP_KERNEL);
	out_init = kvzalloc(sizeof(*out_init), GFP_KERNEL);
	if (!params || !out_init)
		goto out;

	params->dc = dc;
	out_targets.DCN_DECL_INITIALIZATION = out_init;
	params->out_targets = &out_targets;
	/*
	 * The builder copies DMUB windows retained on dmub_srv. Missing
	 * windows fail the build closed. Execute only a successful build,
	 * and do not fall back to legacy hwseq.
	 */
	result = builder->build.declarative(params);
	if (result == BLS_BUILD_SUCCESS)
		dc2_bls_execute(TO_DC2(dc), params->head_sequence,
				params->head_num_steps);

out:
	kvfree(params);
	kvfree(out_init);
}
