// SPDX-License-Identifier: GPL-2.0-only

#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "osf_protocol.h"
#include "osf_stream.h"

#define OSF_STREAM_MAGIC_LEN	sizeof(__le32)
#define OSF_STREAM_MAX_PAYLOAD_LEN				\
	(OSF_STREAM_MAX_FRAME_LEN - OSF_FRAME_HEADER_LEN - OSF_FRAME_CRC_LEN)

static void osf_stream_discard(struct osf_stream *stream, size_t count)
{
	if (count >= stream->len) {
		stream->len = 0;
		return;
	}

	memmove(stream->buf, stream->buf + count, stream->len - count);
	stream->len -= count;
}

static void osf_stream_drop_invalid_head(struct osf_stream *stream)
{
	osf_stream_discard(stream, 1);
}

static bool osf_stream_magic_prefix_match(const u8 *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] != (u8)(OSF_FRAME_MAGIC >> (i * 8)))
			return false;
	}

	return true;
}

static size_t osf_stream_discard_to_magic(struct osf_stream *stream)
{
	size_t old_len = stream->len;
	size_t keep_len;

	for (size_t i = 0; i + OSF_STREAM_MAGIC_LEN <= stream->len; i++) {
		if (get_unaligned_le32(stream->buf + i) == OSF_FRAME_MAGIC) {
			if (i)
				osf_stream_discard(stream, i);
			return i;
		}
	}

	/*
	 * Keep a final 1-3 byte OSF_FRAME_MAGIC prefix so a magic split
	 * across receive_buf() calls can be completed by the next chunk.
	 */
	keep_len = min(stream->len, OSF_STREAM_MAGIC_LEN - 1);
	while (keep_len) {
		size_t offset = stream->len - keep_len;

		if (osf_stream_magic_prefix_match(stream->buf + offset, keep_len)) {
			if (offset)
				osf_stream_discard(stream, offset);
			return offset;
		}
		keep_len--;
	}

	stream->len = 0;
	return old_len;
}

static int osf_stream_process(struct osf_stream *stream)
{
	size_t discarded;
	size_t frame_len;
	u32 payload_len;
	int frame_result;
	int first_err = 0;

	while (stream->len) {
		discarded = osf_stream_discard_to_magic(stream);
		if (discarded) {
			stream->stats.bad_magic_resyncs++;
			stream->stats.dropped_bytes += discarded;
			if (!first_err)
				first_err = -EPROTO;
		}

		if (!stream->len)
			break;

		if (stream->len < OSF_FRAME_HEADER_LEN)
			break;

		if (get_unaligned_le16(stream->buf + 6) != OSF_FRAME_HEADER_LEN) {
			stream->stats.dropped_bytes++;
			osf_stream_drop_invalid_head(stream);
			if (!first_err)
				first_err = -EPROTO;
			continue;
		}

		payload_len = get_unaligned_le32(stream->buf + 10);
		if (payload_len > OSF_STREAM_MAX_PAYLOAD_LEN) {
			stream->stats.dropped_bytes++;
			osf_stream_drop_invalid_head(stream);
			if (!first_err)
				first_err = -EMSGSIZE;
			continue;
		}

		frame_len = OSF_FRAME_HEADER_LEN + payload_len + OSF_FRAME_CRC_LEN;
		if (stream->len < frame_len)
			break;

		frame_result = stream->receive_frame(stream->frame_context,
						     stream->buf, frame_len);
		if (frame_result < 0) {
			if (frame_result == -EBADMSG)
				stream->stats.bad_crc_frames++;

			/*
			 * Decoding failed before the frame was validated;
			 * payload_len is still untrusted. Drop only the current
			 * head and resynchronize.
			 */
			stream->stats.dropped_bytes++;
			osf_stream_drop_invalid_head(stream);
			if (!first_err)
				first_err = frame_result;
			continue;
		}

		/* Count exactly one outcome for every validated frame. */
		stream->stats.validated_frames++;
		switch (frame_result) {
		case OSF_STREAM_FRAME_HANDLED:
			stream->stats.handled_frames++;
			break;
		case OSF_STREAM_FRAME_IGNORED:
			stream->stats.ignored_frames++;
			break;
		case OSF_STREAM_FRAME_REJECTED:
			stream->stats.rejected_frames++;
			break;
		default:
			/*
			 * Preserve the validated boundary without scanning the
			 * payload for another magic value.
			 */
			stream->stats.rejected_frames++;
			if (!first_err)
				first_err = -EPROTO;
			break;
		}
		osf_stream_discard(stream, frame_len);
	}

	return first_err;
}

void osf_stream_reset(struct osf_stream *stream)
{
	if (!stream)
		return;

	stream->len = 0;
	memset(&stream->stats, 0, sizeof(stream->stats));
}

void osf_stream_init(struct osf_stream *stream,
		     int (*receive_frame)(void *context, const u8 *buf,
					  size_t len),
		     void *frame_context)
{
	if (!stream)
		return;

	stream->receive_frame = receive_frame;
	stream->frame_context = frame_context;
	osf_stream_reset(stream);
}

int osf_stream_receive_bytes(struct osf_stream *stream,
			     const u8 *buf, size_t len)
{
	size_t copy_len;
	size_t space;
	int first_err = 0;
	int ret;

	if (!stream || !stream->receive_frame || (!buf && len))
		return -EINVAL;

	if (!len)
		return osf_stream_process(stream);

	/*
	 * Continue processing this receive_buf() chunk after recoverable
	 * framing errors so later valid frames do not wait for another callback.
	 * first_err retains the first diagnostic return, while the serdev
	 * callback reports the full byte count consumed. Every validated
	 * callback result is consumed in full by osf_stream_process().
	 */
	while (len) {
		space = OSF_STREAM_MAX_FRAME_LEN - stream->len;
		if (!space) {
			stream->stats.dropped_bytes++;
			osf_stream_discard(stream, 1);
			if (!first_err)
				first_err = -EMSGSIZE;
			continue;
		}

		copy_len = min(len, space);
		memcpy(stream->buf + stream->len, buf, copy_len);
		stream->len += copy_len;
		buf += copy_len;
		len -= copy_len;

		ret = osf_stream_process(stream);
		if (ret && !first_err)
			first_err = ret;
	}

	return first_err;
}
