/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "dmub_cmd.h"

struct dmub_abi {
	unsigned int fw_version;
	struct dmub_feature_caps feature_caps;
};
