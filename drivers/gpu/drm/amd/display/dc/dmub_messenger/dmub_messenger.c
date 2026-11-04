// SPDX-License-Identifier: MIT
/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 */

#include "dm_services.h"
#include "dc_dmub_srv.h"
#include "hwss/hw_sequencer.h"
#include "dmub_messenger/dmub_messenger.h"

static enum dmub_msg_result dmub_inbox_cmd_send_and_wait(
		struct dmub_messenger *messenger,
		const struct dmub_send_cmd_params *params)
{
	if (!messenger || !messenger->ctx || !messenger->dc_dmub_srv ||
			!messenger->dc_dmub_srv->dmub || !params ||
			!params->cmd)
		return DMUB_MSG_ERROR;

	if (messenger->dc_dmub_srv->idle_allowed && !messenger->ctx->dc)
		return DMUB_MSG_ERROR;

	/*
	 * Transport only: copy the builder-built command through the
	 * injected DMUB service. Do not read the command header or payload.
	 */
	if (!dc_wake_and_execute_dmub_cmd(messenger->ctx, params->cmd,
			params->wait_type))
		return DMUB_MSG_ERROR;

	return DMUB_MSG_OK;
}

static enum dmub_msg_result dmub_gpint_send_and_wait(
		struct dmub_messenger *messenger,
		const struct dmub_gpint_params *params)
{
	struct dmub_srv *dmub;

	if (!messenger || !messenger->ctx || !messenger->dc_dmub_srv ||
			!messenger->dc_dmub_srv->dmub || !params ||
			!params->response)
		return DMUB_MSG_ERROR;

	if (messenger->dc_dmub_srv->idle_allowed && !messenger->ctx->dc)
		return DMUB_MSG_ERROR;

	dmub = messenger->dc_dmub_srv->dmub;
	if (!dmub->sw_init || !dmub->hw_funcs.set_gpint ||
			!dmub->hw_funcs.is_gpint_acked ||
			!dmub->hw_funcs.get_gpint_response)
		return DMUB_MSG_ERROR;

	/*
	 * Transport only: issue the builder-selected GPINT and write the
	 * raw response. Do not interpret the command or the result.
	 */
	if (!dc_wake_and_execute_gpint(messenger->ctx,
			(enum dmub_gpint_command)params->command_code,
			(uint16_t)params->param, params->response,
			DM_DMUB_WAIT_TYPE_WAIT_WITH_REPLY))
		return DMUB_MSG_ERROR;

	return DMUB_MSG_OK;
}

static const struct dmub_messenger_funcs dmub_messenger_funcs = {
	.inbox_cmd_send_and_wait = dmub_inbox_cmd_send_and_wait,
	.gpint_send_and_wait = dmub_gpint_send_and_wait,
};

void dmub_messenger_construct(struct dmub_messenger *messenger,
		const struct dmub_messenger_init_data *init_data)
{
	if (!messenger)
		return;

	messenger->funcs = &dmub_messenger_funcs;
	messenger->ctx = init_data ? init_data->ctx : NULL;
	messenger->dc_dmub_srv = init_data ? init_data->dc_dmub_srv : NULL;
}
