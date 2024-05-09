/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2022 Google LLC
 *
 * Header with tracepoints for virtio video driver.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM virtio_video

#if !defined(_VIRTIO_VIDEO_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _VIRTIO_VIDEO_TRACE_H

#include <linux/tracepoint.h>

#include "virtio_video.h"

DECLARE_EVENT_CLASS(virtio_video_resource_queue_class,
	TP_PROTO(int stream_id, int resource_id, uint32_t *data_size,
		uint8_t num_data_size, uint32_t queue_type, uint64_t timestamp),

	TP_ARGS(stream_id, resource_id, data_size, num_data_size, queue_type,
		timestamp),

	TP_STRUCT__entry(
		__field(int, stream_id)
		__field(int, resource_id)
		__field(uint32_t, queue_type)
		__field(uint32_t, data_size0)
		__field(uint32_t, data_size1)
		__field(uint32_t, data_size2)
		__field(uint32_t, data_size3)
		__field(uint64_t, timestamp)
	),


	TP_fast_assign(
		__entry->stream_id = stream_id;
		__entry->resource_id = resource_id;
		__entry->queue_type = queue_type;
		__entry->data_size0 = num_data_size > 0 ? data_size[0] : 0;
		__entry->data_size1 = num_data_size > 1 ? data_size[1] : 0;
		__entry->data_size2 = num_data_size > 2 ? data_size[2] : 0;
		__entry->data_size3 = num_data_size > 3 ? data_size[3] : 0;
		__entry->timestamp = timestamp;
	),

	TP_printk("stream_id=%d resource_id=%d queue_type=%s data_size0=%u data_size1=%u data_size2=%u data_size3=%u timestamp=%llu",
		__entry->stream_id, __entry->resource_id,
		__print_symbolic(__entry->queue_type,
			{ VIRTIO_VIDEO_QUEUE_TYPE_INPUT, "INPUT" },
			{ VIRTIO_VIDEO_QUEUE_TYPE_OUTPUT, "OUTPUT" }),
		__entry->data_size0, __entry->data_size1, __entry->data_size2,
		__entry->data_size3, __entry->timestamp)

);


DEFINE_EVENT(virtio_video_resource_queue_class, virtio_video_resource_queue,
	TP_PROTO(int stream_id, int resource_id, uint32_t *data_size,
		uint8_t num_data_size, uint32_t queue_type, uint64_t timestamp),

	TP_ARGS(stream_id, resource_id, data_size, num_data_size, queue_type,
		timestamp)
);

DEFINE_EVENT(virtio_video_resource_queue_class, virtio_video_resource_queue_done,
	TP_PROTO(int stream_id, int resource_id, uint32_t *data_size,
		uint8_t num_data_size, uint32_t queue_type, uint64_t timestamp),

	TP_ARGS(stream_id, resource_id, data_size, num_data_size, queue_type,
		timestamp)
);

DECLARE_EVENT_CLASS(virtio_video_cmd_class,
	TP_PROTO(struct virtio_video_cmd_hdr *hdr),

	TP_ARGS(hdr),

	TP_STRUCT__entry(
		__field(uint32_t, type)
		__field(uint32_t, stream_id)
	),


	TP_fast_assign(
		__entry->type = hdr->type;
		__entry->stream_id = hdr->stream_id;
	),

	TP_printk("type=%s stream_id=%u",
		__print_symbolic(__entry->type,
			{ VIRTIO_VIDEO_CMD_QUERY_CAPABILITY, "QUERY_CAPABILITY" },
			{ VIRTIO_VIDEO_CMD_STREAM_CREATE, "STREAM_CREATE" },
			{ VIRTIO_VIDEO_CMD_STREAM_DESTROY, "STREAM_DESTROY" },
			{ VIRTIO_VIDEO_CMD_STREAM_DRAIN, "STREAM_DRAIN" },
			{ VIRTIO_VIDEO_CMD_RESOURCE_CREATE, "RESOURCE_CREATE" },
			{ VIRTIO_VIDEO_CMD_RESOURCE_QUEUE, "RESOURCE_QUEUE" },
			{ VIRTIO_VIDEO_CMD_RESOURCE_DESTROY_ALL, "RESOURCE_DESTROY_ALL" },
			{ VIRTIO_VIDEO_CMD_QUEUE_CLEAR, "QUEUE_CLEAR" },
			{ VIRTIO_VIDEO_CMD_QUERY_CONTROL, "QUERY_CONTROL" },
			{ VIRTIO_VIDEO_CMD_GET_CONTROL, "GET_CONTROL" },
			{ VIRTIO_VIDEO_CMD_SET_CONTROL, "SET_CONTROL" },
			{ VIRTIO_VIDEO_CMD_GET_PARAMS_EXT, "GET_PARAMS_EXT" },
			{ VIRTIO_VIDEO_CMD_SET_PARAMS_EXT, "SET_PARAMS_EXT" }
		),
		__entry->stream_id)

);


DEFINE_EVENT(virtio_video_cmd_class, virtio_video_cmd,
	TP_PROTO(struct virtio_video_cmd_hdr *hdr),

	TP_ARGS(hdr)
);

DEFINE_EVENT(virtio_video_cmd_class, virtio_video_cmd_done,
	TP_PROTO(struct virtio_video_cmd_hdr *hdr),

	TP_ARGS(hdr)
);

#endif /* _VIRTIO_VIDEO_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_FILE trace
#define TRACE_INCLUDE_PATH ../../drivers/media/virtio/
#include <trace/define_trace.h>
