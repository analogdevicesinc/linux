/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OSF_STREAM_H
#define _OSF_STREAM_H

#include <linux/types.h>

#define OSF_STREAM_MAX_FRAME_LEN	4096

/**
 * enum osf_stream_frame_result - validated frame callback result
 * @OSF_STREAM_FRAME_HANDLED: frame was processed successfully
 * @OSF_STREAM_FRAME_IGNORED: frame was valid but unsupported or ignored
 * @OSF_STREAM_FRAME_REJECTED: validated application processing failed
 *
 * A frame callback returns a negative errno only when a candidate could not
 * be validated and the parser may perform one-byte resynchronization.
 * Every nonnegative result must be one of these values. Such a result means
 * the CRC-valid frame boundary is trusted, so the parser must consume the
 * full frame.
 */
enum osf_stream_frame_result {
	OSF_STREAM_FRAME_HANDLED,
	OSF_STREAM_FRAME_IGNORED,
	OSF_STREAM_FRAME_REJECTED,
};

struct osf_stream_stats {
	u64 validated_frames;
	u64 handled_frames;
	u64 ignored_frames;
	u64 rejected_frames;
	u64 bad_magic_resyncs;
	u64 bad_crc_frames;
	u64 dropped_bytes;
};

struct osf_stream {
	int (*receive_frame)(void *context, const u8 *buf, size_t len);
	void *frame_context;
	u8 buf[OSF_STREAM_MAX_FRAME_LEN];
	size_t len;
	struct osf_stream_stats stats;
};

void osf_stream_init(struct osf_stream *stream,
		     int (*receive_frame)(void *context, const u8 *buf,
					  size_t len),
		     void *frame_context);
void osf_stream_reset(struct osf_stream *stream);
int osf_stream_receive_bytes(struct osf_stream *stream,
			     const u8 *buf, size_t len);

#endif
