/* SPDX-License-Identifier: MIT */
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#pragma once

#include "dm_services_types.h"

struct dc_context;
struct dc_dmub_srv;
struct dmub_gpint_params;
union dmub_rb_cmd;

/*
 * Live DMUB_SEND_DMCUB_CMD still carries wait_type. Distinct inbox
 * primitives are unresolved in the DMUB Messenger spec, so this shim
 * forwards the builder-selected wait without interpreting the command.
 */
struct dmub_send_cmd_params {
	union dmub_rb_cmd *cmd;
	enum dm_dmub_wait_type wait_type;
};

struct dmub_messenger_init_data {
	struct dc_context *ctx;
	struct dc_dmub_srv *dc_dmub_srv;
};

enum dmub_msg_result {
	DMUB_MSG_OK,
	DMUB_MSG_NO_MSG,
	DMUB_MSG_TIMEOUT,
	DMUB_MSG_ERROR,
};

struct dmub_messenger;

struct dmub_messenger_funcs {
	enum dmub_msg_result (*inbox_cmd_send_and_wait)(
			struct dmub_messenger *messenger,
			const struct dmub_send_cmd_params *params);
	enum dmub_msg_result (*gpint_send_and_wait)(
			struct dmub_messenger *messenger,
			const struct dmub_gpint_params *params);
};

struct dmub_messenger {
	const struct dmub_messenger_funcs *funcs;
	struct dc_context *ctx;
	struct dc_dmub_srv *dc_dmub_srv;
};

void dmub_messenger_construct(struct dmub_messenger *messenger,
		const struct dmub_messenger_init_data *init_data);
