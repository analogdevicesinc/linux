// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2022 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/resource.h>
#include <linux/rpmsg.h>
#include <linux/uio.h>
#include <linux/virtio.h>

#include "ethosu_buffer.h"
#include "ethosu_core_interface.h"
#include "ethosu_device.h"

struct ethosu_rpmsg *grp;

#define MSG "ethosu say hello!"

static void ethosu_core_set_size(struct ethosu_buffer *buf,
				 struct ethosu_core_buffer *cbuf)
{
	cbuf->ptr = (uint32_t)buf->dma_addr + buf->offset;
	cbuf->size = (uint32_t)buf->size;
}

static void ethosu_core_set_capacity(struct ethosu_buffer *buf,
				     struct ethosu_core_buffer *cbuf)
{
	cbuf->ptr = (uint32_t)buf->dma_addr + buf->offset + buf->size;
	cbuf->size = (uint32_t)buf->capacity - buf->offset - buf->size;
}

static int ethosu_rpmsg_send(struct ethosu_rpmsg *erp, uint32_t type)
{
	struct ethosu_core_msg msg;
	struct rpmsg_device *rpdev = erp->rpdev;
	int ret;

	msg.magic = ETHOSU_CORE_MSG_MAGIC;
	msg.type = type;
	msg.length = 0;

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			(void *)&msg, sizeof(msg),  true);

	ret = rpmsg_send(rpdev->ept, (void *)&msg, sizeof(msg));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int ethosu_rpmsg_ping(struct ethosu_rpmsg *erp)
{
	return ethosu_rpmsg_send(erp, ETHOSU_CORE_MSG_PING);
}

int ethosu_rpmsg_pong(struct ethosu_rpmsg *erp)
{
	return ethosu_rpmsg_send(erp, ETHOSU_CORE_MSG_PONG);
}

int ethosu_rpmsg_version_request(struct ethosu_rpmsg *erp)
{
	return ethosu_rpmsg_send(erp, ETHOSU_CORE_MSG_VERSION_REQ);
}

int ethosu_rpmsg_capabilities_request(struct ethosu_rpmsg *erp, void *user_arg)
{
	struct ethosu_core_msg msg = {
		.magic  = ETHOSU_CORE_MSG_MAGIC,
		.type   = ETHOSU_CORE_MSG_CAPABILITIES_REQ,
		.length = sizeof(struct ethosu_core_capabilities_req)
	};
	struct ethosu_core_capabilities_req req = {
		.user_arg = (uint64_t)user_arg
	};
	struct rpmsg_device *rpdev = erp->rpdev;
	uint8_t data[sizeof(struct ethosu_core_msg) +
		sizeof(struct ethosu_core_capabilities_req)];
	int ret;

	memcpy(data, &msg, sizeof(struct ethosu_core_msg));
	memcpy(data + sizeof(struct ethosu_core_msg), &req,
	       sizeof(struct ethosu_core_capabilities_req));

	ret = rpmsg_send(rpdev->ept, (void *)&data,
			 sizeof(struct ethosu_core_msg) +
			 sizeof(struct ethosu_core_capabilities_req));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}
	return 0;
}

int ethosu_rpmsg_power_request(struct ethosu_rpmsg *erp,
			       enum ethosu_core_power_req_type power_type)
{
	struct ethosu_core_msg msg = {
		.magic  = ETHOSU_CORE_MSG_MAGIC,
		.type   = ETHOSU_CORE_MSG_POWER_REQ,
		.length = sizeof(struct ethosu_core_power_req)
	};
	struct ethosu_core_power_req req;
	struct rpmsg_device *rpdev = erp->rpdev;
	uint8_t data[sizeof(struct ethosu_core_msg) +
		sizeof(struct ethosu_core_power_req)];
	int ret;

	req.type = power_type;
	memcpy(data, &msg, sizeof(struct ethosu_core_msg));
	memcpy(data + sizeof(struct ethosu_core_msg), &req,
	       sizeof(struct ethosu_core_power_req));

	ret = rpmsg_send(rpdev->ept, (void *)&data,
			 sizeof(struct ethosu_core_msg) +
			 sizeof(struct ethosu_core_power_req));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}
	return 0;
}

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
			   uint32_t inference_type)
{
	struct ethosu_core_msg msg = {
		.magic  = ETHOSU_CORE_MSG_MAGIC,
		.type   = ETHOSU_CORE_MSG_INFERENCE_REQ,
		.length = sizeof(struct ethosu_core_inference_req)
	};
	struct ethosu_core_inference_req req;
	struct rpmsg_device *rpdev = erp->rpdev;
	uint8_t data[sizeof(struct ethosu_core_msg) +
		sizeof(struct ethosu_core_inference_req)];
	int ret;
	uint32_t i;

	/* Verify that the uapi and core has the same number of pmus */
	if (pmu_event_config_count != ETHOSU_CORE_PMU_MAX) {
		dev_err(&rpdev->dev, "PMU count misconfigured.\n");

		return -EINVAL;
	}

	req.user_arg = (uint64_t)user_arg;
	req.ifm_count = ifm_count;
	req.ofm_count = ofm_count;
	req.pmu_cycle_counter_enable = pmu_cycle_counter_enable;
	req.inference_type = inference_type;

	for (i = 0; i < ifm_count; i++)
		ethosu_core_set_size(ifm[i], &req.ifm[i]);

	for (i = 0; i < ofm_count; i++)
		ethosu_core_set_capacity(ofm[i], &req.ofm[i]);

	for (i = 0; i < ETHOSU_CORE_PMU_MAX; i++)
		req.pmu_event_config[i] = pmu_event_config[i];

	ethosu_core_set_size(network, &req.network);

	memcpy(data, &msg, sizeof(struct ethosu_core_msg));
	memcpy(data + sizeof(struct ethosu_core_msg), &req,
	       sizeof(struct ethosu_core_inference_req));

	ret = rpmsg_send(rpdev->ept, (void *)&data,
			 sizeof(struct ethosu_core_msg) +
			 sizeof(struct ethosu_core_inference_req));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rpmsg_ethosu_cb(struct rpmsg_device *rpdev,
		void *data, int len, void *priv, u32 src)
{
	struct ethosu_rpmsg *rpmsg = dev_get_drvdata(&rpdev->dev);

	if (len == 0)
		return 0;

	dev_dbg(&rpdev->dev, "msg(<- src 0x%x) len %d\n", src, len);

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);

	rpmsg->callback(rpmsg->user_arg, data);

	return 0;
}

static int ethosu_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret;

	grp->rpdev = rpdev;
	dev_set_drvdata(&rpdev->dev, grp);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		 rpdev->src, rpdev->dst);

	ret = rpmsg_send(rpdev->ept, MSG, strlen(MSG));
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
	}
	complete(&grp->rpmsg_ready);

	return 0;
}

static void ethosu_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg ethosu driver is removed\n");
}

static struct rpmsg_device_id rpmsg_driver_ethosu_id_table[] = {
	{ .name = "rpmsg-ethosu-channel" },
	{ },
};

static struct rpmsg_driver ethosu_rpmsg_driver = {
	.drv.name       = KBUILD_MODNAME,
	.drv.owner      = THIS_MODULE,
	.id_table       = rpmsg_driver_ethosu_id_table,
	.probe          = ethosu_rpmsg_probe,
	.callback	= rpmsg_ethosu_cb,
	.remove         = ethosu_rpmsg_remove,
};

int ethosu_rpmsg_init(struct ethosu_rpmsg *erp,
		ethosu_rpmsg_cb callback, void *user_arg)
{
	grp = erp;
	erp->callback = callback;
	erp->user_arg = user_arg;

	return register_rpmsg_driver(&ethosu_rpmsg_driver);
}

int ethosu_rpmsg_deinit(struct ethosu_rpmsg *erp)
{
	erp->callback = NULL;
	erp->user_arg = NULL;
	erp->rpdev = NULL;

	unregister_rpmsg_driver(&ethosu_rpmsg_driver);
	return 0;
}
