// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2017-2018 NXP
 */

#include <linux/fsl/mc.h>
#include <soc/fsl/dpaa2-io.h>
#include "dpseci.h"
#include "dpseci_cmd.h"

/**
 * dpseci_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpseci_id:	DPSECI unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an already created
 * object; an object may have been declared in the DPL or by calling the
 * dpseci_create() function.
 * This function returns a unique authentication token, associated with the
 * specific object ID and the specific MC portal; this token must be used in all
 * subsequent commands for this specific object.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_open(struct fsl_mc_io *mc_io, u32 cmd_flags, int dpseci_id,
		u16 *token)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_open *cmd_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_OPEN,
					  cmd_flags,
					  0);
	cmd_params = (struct dpseci_cmd_open *)cmd.params;
	cmd_params->dpseci_id = cpu_to_le32(dpseci_id);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	*token = mc_cmd_hdr_read_token(&cmd);

	return 0;
}

/**
 * dpseci_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * After this function is called, no further operations are allowed on the
 * object without opening a new control session.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_close(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token)
{
	struct fsl_mc_command cmd = { 0 };

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_CLOSE,
					  cmd_flags,
					  token);
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_create() - Create the DPSECI object
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token:	Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @obj_id:	returned object id
 *
 * Create the DPSECI object, allocate required resources and perform required
 * initialization.
 *
 * The object can be created either by declaring it in the DPL file, or by
 * calling this function.
 *
 * The function accepts an authentication token of a parent container that this
 * object should be assigned to. The token can be '0' so the object will be
 * assigned to the default container.
 * The newly created object can be opened with the returned object id and using
 * the container's associated tokens and MC portals.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_create(struct fsl_mc_io *mc_io, u16 dprc_token, u32 cmd_flags,
		  const struct dpseci_cfg *cfg, u32 *obj_id)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_create *cmd_params;
	int i, err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_CREATE,
					  cmd_flags,
					  dprc_token);
	cmd_params = (struct dpseci_cmd_create *)cmd.params;
	for (i = 0; i < 8; i++)
		cmd_params->priorities[i] = cfg->priorities[i];
	for (i = 0; i < 8; i++)
		cmd_params->priorities2[i] = cfg->priorities[8 + i];
	cmd_params->num_tx_queues = cfg->num_tx_queues;
	cmd_params->num_rx_queues = cfg->num_rx_queues;
	cmd_params->options = cpu_to_le32(cfg->options);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	*obj_id = mc_cmd_read_object_id(&cmd);

	return 0;
}

/**
 * dpseci_destroy() - Destroy the DPSECI object and release all its resources
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token: Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @object_id:	The object id; it must be a valid id within the container that
 *		created this object
 *
 * The function accepts the authentication token of the parent container that
 * created the object (not the one that currently owns the object). The object
 * is searched within parent using the provided 'object_id'.
 * All tokens to the object must be closed before calling destroy.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_destroy(struct fsl_mc_io *mc_io, u16 dprc_token, u32 cmd_flags,
		   u32 object_id)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_destroy *cmd_params;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_DESTROY,
					  cmd_flags,
					  dprc_token);
	cmd_params = (struct dpseci_cmd_destroy *)cmd.params;
	cmd_params->object_id = cpu_to_le32(object_id);

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_enable() - Enable the DPSECI, allow sending and receiving frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_enable(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token)
{
	struct fsl_mc_command cmd = { 0 };

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_ENABLE,
					  cmd_flags,
					  token);
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_disable() - Disable the DPSECI, stop sending and receiving frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_disable(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token)
{
	struct fsl_mc_command cmd = { 0 };

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_DISABLE,
					  cmd_flags,
					  token);

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_reset() - Reset the DPSECI, returns the object to initial state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_reset(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token)
{
	struct fsl_mc_command cmd = { 0 };

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_RESET,
					  cmd_flags,
					  token);
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_is_enabled() - Check if the DPSECI is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_is_enabled(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
		      int *en)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_rsp_is_enabled *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_IS_ENABLED,
					  cmd_flags,
					  token);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_is_enabled *)cmd.params;
	*en = dpseci_get_field(rsp_params->is_enabled, ENABLE);

	return 0;
}

/**
 * dpseci_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned Interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_irq_enable(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			  u8 irq_index, u8 *en)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_enable *cmd_params;
	struct dpseci_rsp_get_irq_enable *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_enable *)cmd.params;
	cmd_params->irq_index = irq_index;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_irq_enable *)cmd.params;
	*en = rsp_params->enable_state;

	return 0;
}

/**
 * dpseci_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes. The enable/disable control's the
 * overall interrupt state. If the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_set_irq_enable(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			  u8 irq_index, u8 en)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_enable *cmd_params;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_enable *)cmd.params;
	cmd_params->irq_index = irq_index;
	cmd_params->enable_state = en;

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently.
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_irq_mask(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 irq_index, u32 *mask)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_mask *cmd_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_mask *)cmd.params;
	cmd_params->irq_index = irq_index;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	*mask = le32_to_cpu(cmd_params->mask);

	return 0;
}

/**
 * dpseci_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @mask:	event mask to trigger interrupt;
 *		each bit:
 *			0 = ignore event
 *			1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_set_irq_mask(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 irq_index, u32 mask)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_mask *cmd_params;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_mask *)cmd.params;
	cmd_params->mask = cpu_to_le32(mask);
	cmd_params->irq_index = irq_index;

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_irq_status() - Get the current status of any pending interrupts
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_irq_status(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			  u8 irq_index, u32 *status)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_status *cmd_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(*status);
	cmd_params->irq_index = irq_index;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	*status = le32_to_cpu(cmd_params->status);

	return 0;
}

/**
 * dpseci_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_clear_irq_status(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			    u8 irq_index, u32 status)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_irq_status *cmd_params;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(status);
	cmd_params->irq_index = irq_index;

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_attributes() - Retrieve DPSECI attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_attributes(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			  struct dpseci_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_rsp_get_attributes *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_ATTR,
					  cmd_flags,
					  token);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_attributes *)cmd.params;
	attr->id = le32_to_cpu(rsp_params->id);
	attr->num_tx_queues = rsp_params->num_tx_queues;
	attr->num_rx_queues = rsp_params->num_rx_queues;
	attr->options = le32_to_cpu(rsp_params->options);

	return 0;
}

/**
 * dpseci_set_rx_queue() - Set Rx queue configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of priorities configured at
 *		DPSECI creation; use DPSECI_ALL_QUEUES to configure all
 *		Rx queues identically.
 * @cfg:	Rx queue configuration
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_set_rx_queue(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 queue, const struct dpseci_rx_queue_cfg *cfg)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_queue *cmd_params;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_SET_RX_QUEUE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_queue *)cmd.params;
	cmd_params->dest_id = cpu_to_le32(cfg->dest_cfg.dest_id);
	cmd_params->priority = cfg->dest_cfg.priority;
	cmd_params->queue = queue;
	dpseci_set_field(cmd_params->dest_type, DEST_TYPE,
			 cfg->dest_cfg.dest_type);
	cmd_params->user_ctx = cpu_to_le64(cfg->user_ctx);
	cmd_params->options = cpu_to_le32(cfg->options);
	dpseci_set_field(cmd_params->order_preservation_en, ORDER_PRESERVATION,
			 cfg->order_preservation_en);

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_rx_queue() - Retrieve Rx queue attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of priorities configured at
 *		DPSECI creation
 * @attr:	Returned Rx queue attributes
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_rx_queue(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 queue, struct dpseci_rx_queue_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_queue *cmd_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_RX_QUEUE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_queue *)cmd.params;
	cmd_params->queue = queue;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	attr->dest_cfg.dest_id = le32_to_cpu(cmd_params->dest_id);
	attr->dest_cfg.priority = cmd_params->priority;
	attr->dest_cfg.dest_type = dpseci_get_field(cmd_params->dest_type,
						    DEST_TYPE);
	attr->user_ctx = le64_to_cpu(cmd_params->user_ctx);
	attr->fqid = le32_to_cpu(cmd_params->fqid);
	attr->order_preservation_en =
		dpseci_get_field(cmd_params->order_preservation_en,
				 ORDER_PRESERVATION);

	return 0;
}

/**
 * dpseci_get_tx_queue() - Retrieve Tx queue attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of priorities configured at
 *		DPSECI creation
 * @attr:	Returned Tx queue attributes
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_tx_queue(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			u8 queue, struct dpseci_tx_queue_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_queue *cmd_params;
	struct dpseci_rsp_get_tx_queue *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_TX_QUEUE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_queue *)cmd.params;
	cmd_params->queue = queue;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_tx_queue *)cmd.params;
	attr->fqid = le32_to_cpu(rsp_params->fqid);
	attr->priority = rsp_params->priority;

	return 0;
}

/**
 * dpseci_get_sec_attr() - Retrieve SEC accelerator attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @attr:	Returned SEC attributes
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_sec_attr(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			struct dpseci_sec_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_rsp_get_sec_attr *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_SEC_ATTR,
					  cmd_flags,
					  token);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_sec_attr *)cmd.params;
	attr->ip_id = le16_to_cpu(rsp_params->ip_id);
	attr->major_rev = rsp_params->major_rev;
	attr->minor_rev = rsp_params->minor_rev;
	attr->era = rsp_params->era;
	attr->deco_num = rsp_params->deco_num;
	attr->zuc_auth_acc_num = rsp_params->zuc_auth_acc_num;
	attr->zuc_enc_acc_num = rsp_params->zuc_enc_acc_num;
	attr->snow_f8_acc_num = rsp_params->snow_f8_acc_num;
	attr->snow_f9_acc_num = rsp_params->snow_f9_acc_num;
	attr->crc_acc_num = rsp_params->crc_acc_num;
	attr->pk_acc_num = rsp_params->pk_acc_num;
	attr->kasumi_acc_num = rsp_params->kasumi_acc_num;
	attr->rng_acc_num = rsp_params->rng_acc_num;
	attr->md_acc_num = rsp_params->md_acc_num;
	attr->arc4_acc_num = rsp_params->arc4_acc_num;
	attr->des_acc_num = rsp_params->des_acc_num;
	attr->aes_acc_num = rsp_params->aes_acc_num;
	attr->ccha_acc_num = rsp_params->ccha_acc_num;
	attr->ptha_acc_num = rsp_params->ptha_acc_num;

	return 0;
}

/**
 * dpseci_get_sec_counters() - Retrieve SEC accelerator counters
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @counters:	Returned SEC counters
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_sec_counters(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
			    struct dpseci_sec_counters *counters)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_rsp_get_sec_counters *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_SEC_COUNTERS,
					  cmd_flags,
					  token);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_sec_counters *)cmd.params;
	counters->dequeued_requests =
		le64_to_cpu(rsp_params->dequeued_requests);
	counters->ob_enc_requests = le64_to_cpu(rsp_params->ob_enc_requests);
	counters->ib_dec_requests = le64_to_cpu(rsp_params->ib_dec_requests);
	counters->ob_enc_bytes = le64_to_cpu(rsp_params->ob_enc_bytes);
	counters->ob_prot_bytes = le64_to_cpu(rsp_params->ob_prot_bytes);
	counters->ib_dec_bytes = le64_to_cpu(rsp_params->ib_dec_bytes);
	counters->ib_valid_bytes = le64_to_cpu(rsp_params->ib_valid_bytes);

	return 0;
}

/**
 * dpseci_get_api_version() - Get Data Path SEC Interface API version
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path sec API
 * @minor_ver:	Minor version of data path sec API
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_api_version(struct fsl_mc_io *mc_io, u32 cmd_flags,
			   u16 *major_ver, u16 *minor_ver)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_rsp_get_api_version *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_API_VERSION,
					  cmd_flags, 0);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_api_version *)cmd.params;
	*major_ver = le16_to_cpu(rsp_params->major);
	*minor_ver = le16_to_cpu(rsp_params->minor);

	return 0;
}

/**
 * dpseci_set_opr() - Set Order Restoration configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @index:	The queue index
 * @options:	Configuration mode options; can be OPR_OPT_CREATE or
 *		OPR_OPT_RETIRE
 * @cfg:	Configuration options for the OPR
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_set_opr(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token, u8 index,
		   u8 options, struct opr_cfg *cfg)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_opr *cmd_params;

	cmd.header = mc_encode_cmd_header(
			DPSECI_CMDID_SET_OPR,
			cmd_flags,
			token);
	cmd_params = (struct dpseci_cmd_opr *)cmd.params;
	cmd_params->index = index;
	cmd_params->options = options;
	cmd_params->oloe = cfg->oloe;
	cmd_params->oeane = cfg->oeane;
	cmd_params->olws = cfg->olws;
	cmd_params->oa = cfg->oa;
	cmd_params->oprrws = cfg->oprrws;

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_opr() - Retrieve Order Restoration config and query
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @index:	The queue index
 * @cfg:	Returned OPR configuration
 * @qry:	Returned OPR query
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_opr(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token, u8 index,
		   struct opr_cfg *cfg, struct opr_qry *qry)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_opr *cmd_params;
	struct dpseci_rsp_get_opr *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSECI_CMDID_GET_OPR,
					  cmd_flags,
					  token);
	cmd_params = (struct dpseci_cmd_opr *)cmd.params;
	cmd_params->index = index;
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_rsp_get_opr *)cmd.params;
	qry->rip = dpseci_get_field(rsp_params->flags, OPR_RIP);
	qry->enable = dpseci_get_field(rsp_params->flags, OPR_ENABLE);
	cfg->oloe = rsp_params->oloe;
	cfg->oeane = rsp_params->oeane;
	cfg->olws = rsp_params->olws;
	cfg->oa = rsp_params->oa;
	cfg->oprrws = rsp_params->oprrws;
	qry->nesn = le16_to_cpu(rsp_params->nesn);
	qry->ndsn = le16_to_cpu(rsp_params->ndsn);
	qry->ea_tseq = le16_to_cpu(rsp_params->ea_tseq);
	qry->tseq_nlis = dpseci_get_field(rsp_params->tseq_nlis, OPR_TSEQ_NLIS);
	qry->ea_hseq = le16_to_cpu(rsp_params->ea_hseq);
	qry->hseq_nlis = dpseci_get_field(rsp_params->hseq_nlis, OPR_HSEQ_NLIS);
	qry->ea_hptr = le16_to_cpu(rsp_params->ea_hptr);
	qry->ea_tptr = le16_to_cpu(rsp_params->ea_tptr);
	qry->opr_vid = le16_to_cpu(rsp_params->opr_vid);
	qry->opr_id = le16_to_cpu(rsp_params->opr_id);

	return 0;
}

/**
 * dpseci_set_congestion_notification() - Set congestion group
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_set_congestion_notification(struct fsl_mc_io *mc_io, u32 cmd_flags,
	u16 token, const struct dpseci_congestion_notification_cfg *cfg)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_congestion_notification *cmd_params;

	cmd.header = mc_encode_cmd_header(
			DPSECI_CMDID_SET_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	cmd_params = (struct dpseci_cmd_congestion_notification *)cmd.params;
	cmd_params->dest_id = cpu_to_le32(cfg->dest_cfg.dest_id);
	cmd_params->notification_mode = cpu_to_le16(cfg->notification_mode);
	cmd_params->priority = cfg->dest_cfg.priority;
	dpseci_set_field(cmd_params->options, CGN_DEST_TYPE,
			 cfg->dest_cfg.dest_type);
	dpseci_set_field(cmd_params->options, CGN_UNITS, cfg->units);
	cmd_params->message_iova = cpu_to_le64(cfg->message_iova);
	cmd_params->message_ctx = cpu_to_le64(cfg->message_ctx);
	cmd_params->threshold_entry = cpu_to_le32(cfg->threshold_entry);
	cmd_params->threshold_exit = cpu_to_le32(cfg->threshold_exit);

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpseci_get_congestion_notification() - Get congestion group notification
 *	configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on success, error code otherwise
 */
int dpseci_get_congestion_notification(struct fsl_mc_io *mc_io, u32 cmd_flags,
	u16 token, struct dpseci_congestion_notification_cfg *cfg)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpseci_cmd_congestion_notification *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(
			DPSECI_CMDID_GET_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpseci_cmd_congestion_notification *)cmd.params;
	cfg->dest_cfg.dest_id = le32_to_cpu(rsp_params->dest_id);
	cfg->notification_mode = le16_to_cpu(rsp_params->notification_mode);
	cfg->dest_cfg.priority = rsp_params->priority;
	cfg->dest_cfg.dest_type = dpseci_get_field(rsp_params->options,
						   CGN_DEST_TYPE);
	cfg->units = dpseci_get_field(rsp_params->options, CGN_UNITS);
	cfg->message_iova = le64_to_cpu(rsp_params->message_iova);
	cfg->message_ctx = le64_to_cpu(rsp_params->message_ctx);
	cfg->threshold_entry = le32_to_cpu(rsp_params->threshold_entry);
	cfg->threshold_exit = le32_to_cpu(rsp_params->threshold_exit);

	return 0;
}
