/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2017-2018, Intel Corporation
 */

#ifndef __INTEL_SERVICE_CLIENT_H
#define __INTEL_SERVICE_CLIENT_H

/*
 * Service layer driver supports client names
 * @fpga: for FPGA configuration
 * @dummy: for integration/debug/trouble-shooting
 */
#define SVC_CLIENT_FPGA		"fpga"
#define SVC_CLIENT_DUMMY	"dummy"

/*
 * Status of the sent command, in bit number
 * @SVC_COMMAND_STATUS_RECONFIG_REQUEST_OK:
 * Secure firmware accepts the request of FPGA reconfiguration.
 * @SVC_STATUS_RECONFIG_BUFFER_SUBMITTED:
 * Service client successfully submits FPGA configuration
 * data buffer to secure firmware.
 * @SVC_COMMAND_STATUS_RECONFIG_BUFFER_DONE:
 * Secure firmware completes data process, ready to accept the
 * next WRITE transaction.
 * @SVC_COMMAND_STATUS_RECONFIG_COMPLETED:
 * Secure firmware completes FPGA configuration successfully, FPGA should
 * be in user mode.
 * @SVC_COMMAND_STATUS_RECONFIG_BUSY:
 * FPGA configuration is still in process.
 * @SVC_COMMAND_STATUS_RECONFIG_ERROR:
 * Error encountered during FPGA configuration.
 */
#define SVC_STATUS_RECONFIG_REQUEST_OK		0
#define SVC_STATUS_RECONFIG_BUFFER_SUBMITTED	1
#define SVC_STATUS_RECONFIG_BUFFER_DONE		2
#define SVC_STATUS_RECONFIG_COMPLETED		3
#define SVC_STATUS_RECONFIG_BUSY		4
#define SVC_STATUS_RECONFIG_ERROR		5

/*
 * Flag bit for COMMAND_RECONFIG
 * @COMMAND_RECONFIG_FLAG_PARTIAL
 * Set to FPGA configuration type (full or partial), the default
 * is full reconfig.
 */
#define COMMAND_RECONFIG_FLAG_PARTIAL   0

/* Timeout settings for FPGA manager driver */
#define SVC_RECONFIG_REQUEST_TIMEOUT_MS         100
#define SVC_RECONFIG_BUFFER_TIMEOUT_MS          240

struct intel_svc_chan;

/**
 * enum intel_svc_command_code - supporting service commands
 * @COMMAND_NOOP: do 'dummy' request for integration/debug/trouble-shootings
 * @COMMAND_RECONFIG: ask for FPGA configuration preparation, return status
 * is SVC_STATUS_RECONFIG_REQUEST_OK
 * @COMMAND_RECONFIG_DATA_SUBMIT: submit buffer(s) of bit-stream data for the
 * FPGA configuration, return status is SVC_STATUS_RECONFIG_BUFFER_SUBMITTED,
 * or SVC_STATUS_RECONFIG_ERROR
 * @COMMAND_RECONFIG_DATA_CLAIM: check the status of the configuration, return
 * status is SVC_STATUS_RECONFIG_COMPLETED, or SVC_STATUS_RECONFIG_BUSY, or
 * SVC_STATUS_RECONFIG_ERROR
 * @COMMAND_RECONFIG_STATUS: check the status of the configuration, return
 * status is SVC_STATUS_RECONFIG_COMPLETED, or  SVC_STATUS_RECONFIG_BUSY, or
 * SVC_STATUS_RECONFIG_ERROR
 */
enum intel_svc_command_code {
	COMMAND_NOOP = 0,
	COMMAND_RECONFIG,
	COMMAND_RECONFIG_DATA_SUBMIT,
	COMMAND_RECONFIG_DATA_CLAIM,
	COMMAND_RECONFIG_STATUS
};

/**
 * struct intel_svc_client_msg - message sent by client to service
 * @command: service command
 * @payload: starting address of data need be processed
 * @payload_length: data size in bytes
 */
struct intel_svc_client_msg {
	void *payload;
	size_t payload_length;
	enum intel_svc_command_code command;
};

/**
 * struct intel_command_reconfig_payload - reconfig payload
 * @flags: flag bit for the type of FPGA configuration
 */
struct intel_command_reconfig_payload {
	u32 flags;
};

/**
 * struct intel_svc_c_data - callback data structure from service layer
 * @status: the status of sent command
 * @kaddr1-3: used when status is SVC_COMMAND_STATUS_RECONFIG_BUFFER_DONE
 *
 * kaddr1 - address of 1st completed data block.
 * kaddr2 - address of 2nd completed data block.
 * kaddr3 - address of 3rd completed data block.
 */
struct intel_svc_c_data {
	u32 status;
	void *kaddr1;
	void *kaddr2;
	void *kaddr3;
};

/**
 * struct intel_svc_client - service client structure
 * @dev: the client device
 * @receive_callback: callback to provide service client the received data
 * @priv: client private data
 */
struct intel_svc_client {
	struct device *dev;
	void (*receive_cb)(struct intel_svc_client *client,
			   struct intel_svc_c_data *data);
	void *priv;
};

/**
 * request_svc_channel_byname() - request service channel
 * @client: identity of the client requesting the channel
 * @name: supporting client name defined above
 *
 * Return: a pointer to channel assigned to the client on success,
 * or ERR_PTR() on error.
 */
struct intel_svc_chan
*request_svc_channel_byname(struct intel_svc_client *client,
	const char *name);

/**
 * free_svc_channel() - free service channel.
 * @chan: service channel to be freed
 */
void free_svc_channel(struct intel_svc_chan *chan);

/**
 * intel_svc_allocate_memory() - allocate the momory
 * @chan: service channel assigned to the client
 * @size: number of bytes client requests
 *
 * Service layer allocates the requested number of bytes from the memory
 * pool for the client.
 *
 * Return: the starting address of allocated memory on success, or
 * ERR_PTR() on error.
 */
void *intel_svc_allocate_memory(struct intel_svc_chan *chan, size_t size);

/**
 * intel_svc_free_memory() - free allocated memory
 * @chan: service channel assigned to the client
 * @kaddr: starting address of memory to be free back to pool
 */
void intel_svc_free_memory(struct intel_svc_chan *chan, void *kaddr);

/**
 * intel_svc_send() - send a message to the remote
 * @chan: service channel assigned to the client
 * @msg: message data to be sent, in the format of struct intel_svc_client_msg
 *
 * Return: positive value for successful submission to the data queue created
 * by service layer driver, or -ENOBUFS if the data queue FIFO is full.
 */
int intel_svc_send(struct intel_svc_chan *chan, void *msg);
#endif

