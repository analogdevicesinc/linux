// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2022 NXP
 */

#ifndef ETHOSU_RPMSG_H
#define ETHOSU_RPMSG_H

#include <linux/types.h>
#include <linux/completion.h>
#include "ethosu_core_interface.h"

struct device;
struct ethosu_buffer;
struct ethosu_device;
struct ethosu_core_msg;

typedef void (*ethosu_rpmsg_cb)(void *user_arg, void *data);

struct ethosu_rpmsg {
	struct rpmsg_device	*rpdev;
	ethosu_rpmsg_cb		callback;
	void			*user_arg;
	struct completion       rpmsg_ready;
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
				      void *user_arg);

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
			   void *user_arg,
			   uint32_t ifm_count,
			   struct ethosu_buffer **ifm,
			   uint32_t ofm_count,
			   struct ethosu_buffer **ofm,
			   struct ethosu_buffer *network,
			   uint8_t *pmu_event_config,
			   uint8_t pmu_event_config_count,
			   uint8_t pmu_cycle_counter_enable,
			   uint32_t inference_type
			   );

int ethosu_rpmsg_init(struct ethosu_rpmsg *erp,
		      ethosu_rpmsg_cb callback, void *user_arg);

int ethosu_rpmsg_deinit(struct ethosu_rpmsg *erp);
#endif /* ETHOSU_RPMSG_H */
