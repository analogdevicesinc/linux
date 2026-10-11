// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/types.h>

#include "osf_core.h"
#include "osf_stream.h"

#define OSF_RESERVED_MSG_FIRST		0x7f00
#define OSF_RESERVED_MSG_LAST		0x7fff
#define OSF_VENDOR_PRIVATE_FIRST	0x8000

void osf_core_init(struct osf_device *osf, struct device *dev)
{
	*osf = (struct osf_device) {
		.dev = dev,
	};
}

static int osf_core_handle_device_status(struct osf_device *osf,
					 const struct osf_frame *frame)
{
	struct osf_device_status status;
	int ret;

	ret = osf_protocol_decode_device_status(frame, &status);
	if (ret) {
		dev_warn_ratelimited(osf->dev,
				     "rejecting malformed device status: %d\n",
				     ret);
		return ret;
	}

	osf->status_cache = (struct osf_status_cache) {
		.uptime_s = status.uptime_s,
		.status_flags = status.status_flags,
		.error_flags = status.error_flags,
		.dropped_frames = status.dropped_frames,
		.sequence = frame->sequence,
		.valid = true,
	};
	osf->last_sequence = frame->sequence;

	return OSF_STREAM_FRAME_HANDLED;
}

int osf_core_receive_frame(struct osf_device *osf, const u8 *buf, size_t len)
{
	struct osf_frame frame;
	size_t frame_len;
	int ret;

	ret = osf_protocol_decode_frame(buf, len, &frame, &frame_len);
	if (ret)
		return ret;

	if (frame_len != len)
		return -EMSGSIZE;

	if (frame.protocol_major != OSF_PROTOCOL_MAJOR) {
		dev_dbg_ratelimited(osf->dev,
				    "ignoring unsupported protocol major %u\n",
				    frame.protocol_major);
		return OSF_STREAM_FRAME_IGNORED;
	}

	/*
	 * Handler failures are validated application rejections. Keep the
	 * trusted frame boundary and let the stream consume the complete frame.
	 */
	switch (frame.message_type) {
	case OSF_MSG_DEVICE_STATUS:
		ret = osf_core_handle_device_status(osf, &frame);
		return ret < 0 ? OSF_STREAM_FRAME_REJECTED : ret;
	default:
		if (frame.message_type >= OSF_RESERVED_MSG_FIRST &&
		    frame.message_type <= OSF_RESERVED_MSG_LAST) {
			dev_dbg_ratelimited(osf->dev,
					    "ignoring reserved message type %#x\n",
					    frame.message_type);
			return OSF_STREAM_FRAME_IGNORED;
		}
		if (frame.message_type >= OSF_VENDOR_PRIVATE_FIRST) {
			dev_dbg_ratelimited(osf->dev,
					    "ignoring vendor message type %#x\n",
					    frame.message_type);
			return OSF_STREAM_FRAME_IGNORED;
		}

		dev_dbg_ratelimited(osf->dev,
				    "ignoring unsupported message type %#x\n",
				    frame.message_type);
		return OSF_STREAM_FRAME_IGNORED;
	}
}
