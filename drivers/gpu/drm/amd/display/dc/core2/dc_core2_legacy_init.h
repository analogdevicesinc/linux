/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

struct dc;
struct workaround_manager;

/*
 * DC2_DC1_Wrapped hardware init: dcn401_init_hw as twelve staged BLS
 * builds, each passed through wam. A migration shim; it and
 * dcn401_init_builder_legacy.c are removed together.
 */
bool core2_run_legacy_hardware_init(struct dc *dc,
		const struct workaround_manager *wam);
