// SPDX-License-Identifier: GPL-2.0+
/* Driver for virtio video device.
 *
 * Copyright 2019 OpenSynergy GmbH.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>

#include "virtio_video.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

static unsigned int debug;
module_param(debug, uint, 0644);

static unsigned int use_dma_mem = 1;
module_param(use_dma_mem, uint, 0644);
MODULE_PARM_DESC(use_dma_mem, "Try to allocate buffers from the DMA zone");

static void virtio_video_init_vq(struct virtio_video_queue *vvq,
				 void (*work_func)(struct work_struct *work))
{
	spin_lock_init(&vvq->qlock);
	init_waitqueue_head(&vvq->ack_queue);
	INIT_WORK(&vvq->dequeue_work, work_func);
}

static int virtio_video_query_cap_resp_buf(struct virtio_video *vv, void
					   **resp_buf, int queue_type)
{
	int ret = 0;
	int resp_size = vv->max_caps_len;

	*resp_buf = kzalloc(vv->max_caps_len, GFP_KERNEL);
	if (!*resp_buf) {
		ret = -ENOMEM;
		goto err;
	}

	vv->got_caps = false;
	ret = virtio_video_query_capability(vv, *resp_buf, resp_size,
					    queue_type);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "failed to query capability\n");
		goto err;
	}

	ret = wait_event_timeout(vv->wq, vv->got_caps, 5 * HZ);
	if (ret == 0) {
		v4l2_err(&vv->v4l2_dev, "timed out waiting for get caps\n");
		ret = -EIO;
		goto err;
	}

	return 0;
err:
	return ret;
}

static int virtio_video_init(struct virtio_video *vv)
{
	int ret = 0;
	void *input_resp_buf = NULL;
	void *output_resp_buf = NULL;

	if (!vv)
		return -EINVAL;

	ret = virtio_video_query_cap_resp_buf(vv, &input_resp_buf,
					      VIRTIO_VIDEO_QUEUE_TYPE_INPUT);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "failed to get input caps\n");
		goto err;
	}

	ret = virtio_video_query_cap_resp_buf(vv, &output_resp_buf,
					      VIRTIO_VIDEO_QUEUE_TYPE_OUTPUT);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "failed to get output caps\n");
		goto err;
	}

	ret = virtio_video_device_init(vv, input_resp_buf, output_resp_buf);
	if (ret)
		v4l2_err(&vv->v4l2_dev, "failed to initialize devices\n");

err:
	kfree(input_resp_buf);
	kfree(output_resp_buf);

	return ret;
};

static int virtio_video_probe(struct virtio_device *vdev)
{
	int ret;
	struct virtio_video *vv;
	struct virtqueue *vqs[2];
	struct virtqueue_info *vqs_info;
	struct device *dev = &vdev->dev;

	static const char * const names[] = { "control", "event" };
	static vq_callback_t *callbacks[] = {
		virtio_video_cmd_ack,
		virtio_video_event_ack
	};
	vv = devm_kzalloc(dev, sizeof(*vv), GFP_KERNEL);
	if (!vv)
		return -ENOMEM;

	dev->dma_mask = &vv->dma_mask;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	vv->vdev = vdev;
	vv->debug = debug;
	vv->use_dma_mem = use_dma_mem;
	vdev->priv = vv;

	spin_lock_init(&vv->resource_idr_lock);
	idr_init(&vv->resource_idr);
	mutex_init(&vv->stream_idr_lock);
	idr_init(&vv->stream_idr);

	init_waitqueue_head(&vv->wq);

	if (virtio_has_feature(vdev, VIRTIO_VIDEO_F_RESOURCE_NON_CONTIG))
		vv->supp_non_contig = true;

	vv->use_dma_api = !virtio_has_dma_quirk(vdev);
	dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));

	dev_set_name(dev, "%s.%i", DRIVER_NAME, vdev->index);
	ret = v4l2_device_register(dev, &vv->v4l2_dev);
	if (ret)
		goto err_v4l2_reg;

	virtio_video_init_vq(&vv->commandq, virtio_video_dequeue_cmd_func);
	virtio_video_init_vq(&vv->eventq, virtio_video_dequeue_event_func);

	vqs_info = devm_kcalloc(dev, 2, sizeof(*vqs_info), GFP_KERNEL);
	vqs_info[0].name = names[0];
	vqs_info[0].callback = callbacks[0];

	vqs_info[1].name = names[1];
	vqs_info[1].callback = callbacks[1];

	ret = virtio_find_vqs(vdev, 2, vqs, vqs_info, NULL);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "failed to find virt queues\n");
		goto err_vqs;
	}

	vv->commandq.vq = vqs[0];
	vv->eventq.vq = vqs[1];

	ret = virtio_video_alloc_vbufs(vv);
	if (ret) {
		v4l2_err(&vv->v4l2_dev, "failed to alloc vbufs\n");
		goto err_vbufs;
	}

	v4l2_info(&vv->v4l2_dev, "command vq, desc 0x%llx, used 0x%llx, avail 0x%llx, size 0x%x\n",
		  virtqueue_get_desc_addr(vv->commandq.vq),
		  virtqueue_get_used_addr(vv->commandq.vq),
		  virtqueue_get_avail_addr(vv->commandq.vq),
		  virtqueue_get_vring_size(vv->commandq.vq));
	v4l2_info(&vv->v4l2_dev, "event vq, desc 0x%llx, used 0x%llx, avail 0x%llx, size 0x%x\n",
		  virtqueue_get_desc_addr(vv->eventq.vq),
		  virtqueue_get_used_addr(vv->eventq.vq),
		  virtqueue_get_avail_addr(vv->eventq.vq),
		  virtqueue_get_vring_size(vv->eventq.vq));

	virtio_cread(vdev, struct virtio_video_config, max_caps_length,
		     &vv->max_caps_len);
	if (!vv->max_caps_len) {
		v4l2_err(&vv->v4l2_dev, "max_caps_len is zero\n");
		ret = -EINVAL;
		goto err_config;
	}

	virtio_cread(vdev, struct virtio_video_config, max_resp_length,
		     &vv->max_resp_len);
	if (!vv->max_resp_len) {
		v4l2_err(&vv->v4l2_dev, "max_resp_len is zero\n");
		ret = -EINVAL;
		goto err_config;
	}

	virtio_cread_bytes(vdev,
			   offsetof(struct virtio_video_config, device_name),
			   vv->device_name, sizeof(vv->device_name));

	ret = virtio_video_alloc_events(vv, vv->eventq.vq->num_free);
	if (ret)
		goto err_events;

	virtio_device_ready(vdev);
	vv->vq_ready = true;
	vv->got_caps = false;

	INIT_LIST_HEAD(&vv->devices_list);

	ret = virtio_video_init(vv);
	if (ret) {
		v4l2_err(&vv->v4l2_dev,
			 "failed to init virtio video\n");
		goto err_init;
	}

	return 0;

err_init:
err_events:
err_config:
	virtio_video_free_vbufs(vv);
err_vbufs:
	vdev->config->del_vqs(vdev);
err_vqs:
	v4l2_device_unregister(&vv->v4l2_dev);
err_v4l2_reg:
	devm_kfree(&vdev->dev, vv);

	return ret;
}

static void virtio_video_remove(struct virtio_device *vdev)
{
	struct virtio_video *vv = vdev->priv;

	virtio_video_device_deinit(vv);
	virtio_video_free_vbufs(vv);
	vdev->config->del_vqs(vdev);
	v4l2_device_unregister(&vv->v4l2_dev);
	devm_kfree(&vdev->dev, vv);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_VIDEO_DECODER, VIRTIO_DEV_ANY_ID },
	{ VIRTIO_ID_VIDEO_ENCODER, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {
	VIRTIO_VIDEO_F_RESOURCE_GUEST_PAGES,
	VIRTIO_VIDEO_F_RESOURCE_NON_CONTIG,
	VIRTIO_VIDEO_F_RESOURCE_VIRTIO_OBJECT,
};

static struct virtio_driver virtio_video_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name = DRIVER_NAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.probe = virtio_video_probe,
	.remove = virtio_video_remove,
};

module_virtio_driver(virtio_video_driver);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("virtio video driver");
MODULE_AUTHOR("Dmitry Sepp <dmitry.sepp@opensynergy.com>");
MODULE_AUTHOR("Kiran Pawar <kiran.pawar@opensynergy.com>");
MODULE_AUTHOR("Nikolay Martyanov <nikolay.martyanov@opensynergy.com>");
MODULE_AUTHOR("Samiullah Khawaja <samiullah.khawaja@opensynergy.com>");
MODULE_LICENSE("GPL");
