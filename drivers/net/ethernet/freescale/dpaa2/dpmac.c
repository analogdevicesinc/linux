// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2019 NXP
 */
#include <linux/fsl/mc.h>
#include "dpmac.h"
#include "dpmac-cmd.h"

/**
 * dpmac_open() - Open a control session for the specified object.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpmac_id:	DPMAC unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpmac_create function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_open(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       int dpmac_id,
	       u16 *token)
{
	struct dpmac_cmd_open *cmd_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_OPEN,
					  cmd_flags,
					  0);
	cmd_params = (struct dpmac_cmd_open *)cmd.params;
	cmd_params->dpmac_id = cpu_to_le32(dpmac_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = mc_cmd_hdr_read_token(&cmd);

	return err;
}

/**
 * dpmac_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_close(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_CLOSE, cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_get_attributes - Retrieve DPMAC attributes.
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_get_attributes(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 struct dpmac_attr *attr)
{
	struct dpmac_rsp_get_attributes *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpmac_rsp_get_attributes *)cmd.params;
	attr->eth_if = rsp_params->eth_if;
	attr->link_type = rsp_params->link_type;
	attr->id = le16_to_cpu(rsp_params->id);
	attr->max_rate = le32_to_cpu(rsp_params->max_rate);

	return 0;
}

/**
 * dpmac_set_link_state() - Set the Ethernet link status
 * @mc_io:      Pointer to opaque I/O object
 * @cmd_flags:  Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:      Token of DPMAC object
 * @link_state: Link state configuration
 *
 * Return:      '0' on Success; Error code otherwise.
 */
int dpmac_set_link_state(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 struct dpmac_link_state *link_state)
{
	struct dpmac_cmd_set_link_state *cmd_params;
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_SET_LINK_STATE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_set_link_state *)cmd.params;
	cmd_params->options = cpu_to_le64(link_state->options);
	cmd_params->rate = cpu_to_le32(link_state->rate);
	dpmac_set_field(cmd_params->state, STATE, link_state->up);
	dpmac_set_field(cmd_params->state, STATE_VALID,
			link_state->state_valid);
	cmd_params->supported = cpu_to_le64(link_state->supported);
	cmd_params->advertising = cpu_to_le64(link_state->advertising);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_get_counter() - Read a specific DPMAC counter
 * @mc_io:	Pointer to opaque I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @id:		The requested counter ID
 * @value:	Returned counter value
 *
 * Return:	The requested counter; '0' otherwise.
 */
int dpmac_get_counter(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
		      enum dpmac_counter_id id, u64 *value)
{
	struct dpmac_cmd_get_counter *dpmac_cmd;
	struct dpmac_rsp_get_counter *dpmac_rsp;
	struct fsl_mc_command cmd = { 0 };
	int err = 0;

	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_COUNTER,
					  cmd_flags,
					  token);
	dpmac_cmd = (struct dpmac_cmd_get_counter *)cmd.params;
	dpmac_cmd->id = id;

	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	dpmac_rsp = (struct dpmac_rsp_get_counter *)cmd.params;
	*value = le64_to_cpu(dpmac_rsp->counter);

	return 0;
}

/**
 * dpmac_get_api_version() - Get Data Path MAC version
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path mac API
 * @minor_ver:	Minor version of data path mac API
 *
 * Return:  '0' on Success; Error code otherwise.
 */
int dpmac_get_api_version(struct fsl_mc_io *mc_io, u32 cmd_flags,
			  u16 *major_ver, u16 *minor_ver)
{
	struct dpmac_rsp_get_api_version *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_API_VERSION,
					  cmd_flags,
					  0);
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpmac_rsp_get_api_version *)cmd.params;
	*major_ver = le16_to_cpu(rsp_params->major);
	*minor_ver = le16_to_cpu(rsp_params->minor);

	return 0;
}

/**
 * dpmac_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_set_irq_enable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u8 en)
{
	struct dpmac_cmd_set_irq_enable *cmd_params;
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_set_irq_enable *)cmd.params;
	cmd_params->irq_index = irq_index;
	cmd_params->enable = en;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_get_irq_enable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u8 *en)
{
	struct dpmac_cmd_get_irq_enable *cmd_params;
	struct dpmac_rsp_get_irq_enable *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_get_irq_enable *)cmd.params;
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpmac_rsp_get_irq_enable *)cmd.params;
	*en = rsp_params->enabled;

	return 0;
}

/**
 * dpmac_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @mask:	Event mask to trigger interrupt;
 *		each bit:
 *			0 = ignore event
 *			1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_set_irq_mask(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u8 irq_index,
		       u32 mask)
{
	struct dpmac_cmd_set_irq_mask *cmd_params;
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_set_irq_mask *)cmd.params;
	cmd_params->mask = cpu_to_le32(mask);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_get_irq_mask(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u8 irq_index,
		       u32 *mask)
{
	struct dpmac_cmd_get_irq_mask *cmd_params;
	struct dpmac_rsp_get_irq_mask *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_get_irq_mask *)cmd.params;
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpmac_rsp_get_irq_mask *)cmd.params;
	*mask = le32_to_cpu(rsp_params->mask);

	return 0;
}

/**
 * dpmac_get_irq_status() - Get the current status of any pending interrupts.
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_get_irq_status(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u8 irq_index,
			 u32 *status)
{
	struct dpmac_cmd_get_irq_status *cmd_params;
	struct dpmac_rsp_get_irq_status *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_get_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(*status);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpmac_rsp_get_irq_status *)cmd.params;
	*status = le32_to_cpu(rsp_params->status);

	return 0;
}

/**
 * dpmac_set_protocol() - Reconfigure the DPMAC protocol
 * @mc_io:      Pointer to opaque I/O object
 * @cmd_flags:  Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:      Token of DPMAC object
 * @protocol:   New protocol for the DPMAC to be reconfigured in.
 *
 * Return:      '0' on Success; Error code otherwise.
 */
int dpmac_set_protocol(struct fsl_mc_io *mc_io, u32 cmd_flags, u16 token,
		       enum dpmac_eth_if protocol)
{
	struct dpmac_cmd_set_protocol *cmd_params;
	struct fsl_mc_command cmd = { 0 };

	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_SET_PROTOCOL,
					  cmd_flags, token);
	cmd_params = (struct dpmac_cmd_set_protocol *)cmd.params;
	cmd_params->eth_if = protocol;

	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_clear_irq_status() - Clear a pending interrupt's status
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPMAC object
 * @irq_index:	The interrupt index to configure
 * @status:	Bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpmac_clear_irq_status(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u8 irq_index,
			   u32 status)
{
	struct dpmac_cmd_clear_irq_status *cmd_params;
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpmac_cmd_clear_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(status);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpmac_get_link_cfg() - Get Ethernet link configuration
 * @mc_io:      Pointer to opaque I/O object
 * @cmd_flags:  Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:      Token of DPMAC object
 * @cfg:        Returned structure with the link configuration
 *
 * Return:      '0' on Success; Error code otherwise.
 */
int dpmac_get_link_cfg(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      struct dpmac_link_cfg *cfg)
{
	struct dpmac_rsp_get_link_cfg *rsp_params;
	struct fsl_mc_command cmd = { 0 };
	int err = 0;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPMAC_CMDID_GET_LINK_CFG,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpmac_rsp_get_link_cfg *)cmd.params;
	cfg->options = le64_to_cpu(rsp_params->options);
	cfg->rate = le32_to_cpu(rsp_params->rate);
	cfg->advertising = le64_to_cpu(rsp_params->advertising);

	return 0;
}
