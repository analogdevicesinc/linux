// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "bls_builder/bls_builder_internal.h"

const struct bls_builder *bls_builder_create(struct dc *dc,
		enum bls_builder_name name)
{
	if (!dc || !dc->ctx)
		return NULL;

	switch (dc->ctx->dce_version) {
	case DCN_VERSION_4_01:
		return dcn401_bls_builder_create(name);
	default:
		return NULL;
	}
}
