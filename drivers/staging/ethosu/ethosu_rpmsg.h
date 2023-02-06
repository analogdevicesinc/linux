// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2022 NXP
 */

#ifndef ETHOSU_RPMSG_H
#define ETHOSU_RPMSG_H

#include <linux/idr.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include "ethosu_core_interface.h"

struct device;
struct ethosu_buffer;
struct ethosu_device;
struct ethosu_core_msg;

typedef void (*ethosu_rpmsg_cb)(void *user_arg, void *data);

struct ethosu_rpmsg {
	rwlock_t		lock;
	struct rpmsg_device	*rpdev;
	ethosu_rpmsg_cb		callback;
	void			*user_arg;
	struct completion       rpmsg_ready;
	struct idr              msg_idr;
	unsigned int            ping_count;
};

struct ethosu_rpmsg_msg {
	int  id;
	void (*fail)(struct ethosu_rpmsg_msg *msg);
	int  (*resend)(struct ethosu_rpmsg_msg *msg);
};

/**
 * ethosu_rpmsg_ping() - Send ping message
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_ping(struct ethosu_rpmsg *erp);

/**
 * ethosu_rpmsg_pong() - Send pong message
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_pong(struct ethosu_rpmsg *erp);

/**
 * ethosu_rpmsg_version_response - Send version request
 *
 * Return: 0 on success, else error code
 */
int ethosu_rpmsg_version_request(struct ethosu_rpmsg *erp);

/**
 * ethosu_rpmsg_capabilities_request - Send capabilities request
 *
 * Return: 0 on success, else error code
 */
int ethosu_rpmsg_capabilities_request(struct ethosu_rpmsg *erp,
				      struct ethosu_rpmsg_msg *rpmsg);

/**
 * ethosu_rpmsg_power_request - Send power request
 *
 * Return: 0 on success, else error code
 */
int ethosu_rpmsg_power_request(struct ethosu_rpmsg *erp,
			       enum ethosu_core_power_req_type power_type);

/**
 * ethosu_rpmsg_inference() - Send inference
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_inference(struct ethosu_rpmsg *erp,
			   struct ethosu_rpmsg_msg *rpmsg,
			   uint32_t ifm_count,
			   struct ethosu_buffer **ifm,
			   uint32_t ofm_count,
			   struct ethosu_buffer **ofm,
			   struct ethosu_buffer *network,
			   u32 network_index,
			   uint8_t *pmu_event_config,
			   uint8_t pmu_event_config_count,
			   uint8_t pmu_cycle_counter_enable,
			   uint32_t inference_type
			   );

/**
 * ethosu_rpmsg_network_info_request() - Send network info request
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_network_info_request(struct ethosu_rpmsg *erp,
				      struct ethosu_rpmsg_msg *rpmsg,
				      struct ethosu_buffer *network,
				      uint32_t network_index);

/**
 * ethosu_rpmsg_cancel_inference() - Send inference cancellation
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_cancel_inference(struct ethosu_rpmsg *erp,
				  struct ethosu_rpmsg_msg *rpmsg,
				  int inference_handle);

int ethosu_rpmsg_init(struct ethosu_rpmsg *erp,
		      ethosu_rpmsg_cb callback, void *user_arg);

int ethosu_rpmsg_deinit(struct ethosu_rpmsg *erp);

/**
 * ethosu_rpmsg_register() - Register the ethosu_rpmsg_msg in ethosu_rpmsg
 *
 * Return: 0 on success, else error code.
 */
int ethosu_rpmsg_register(struct ethosu_rpmsg *erp,
			  struct ethosu_rpmsg_msg *msg);

/**
 * ethosu_rpmsg_free_id() - Free the id of the ethosu_rpmsg_msg
 */
void ethosu_rpmsg_deregister(struct ethosu_rpmsg *erp,
			     struct ethosu_rpmsg_msg *msg);

/**
 * ethosu_rpmsg_find() - Find rpmsg message
 *
 * Return: a valid pointer on success, otherwise an error ptr.
 */
struct ethosu_rpmsg_msg *ethosu_rpmsg_find(struct ethosu_rpmsg *erq,
					   int msg_id);

/**
 * ethosu_rpmsg_fail() - Fail rpmsg messages
 *
 * Call fail() callback on all messages in pending list.
 */
void ethosu_rpmsg_fail(struct ethosu_rpmsg *erp);

/**
 * ethosu_rpmsg_resend() - Resend rpmsg messages
 *
 * Call resend() callback on all messages in pending list.
 */
void ethosu_rpmsg_resend(struct ethosu_rpmsg *erp);

#endif /* ETHOSU_RPMSG_H */
