// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bits.h>
#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "osf_protocol.h"

#define OSF_CRC32_INIT		GENMASK(31, 0)
#define OSF_CRC32_XOROUT	GENMASK(31, 0)

static bool osf_sensor_type_valid(u16 sensor_type)
{
	return sensor_type >= OSF_SENSOR_ACCELEROMETER &&
	       sensor_type <= OSF_SENSOR_PROXIMITY;
}

static u32 osf_crc32_ieee(const u8 *buf, size_t len)
{
	return crc32_le(OSF_CRC32_INIT, buf, len) ^ OSF_CRC32_XOROUT;
}

int osf_protocol_decode_frame(const u8 *buf, size_t len,
			      struct osf_frame *frame, size_t *frame_len)
{
	u32 expected_crc;
	u32 actual_crc;
	u32 payload_len;
	size_t total_len;

	if (!buf || !frame || !frame_len)
		return -EINVAL;

	if (len < OSF_FRAME_MIN_LEN)
		return -EMSGSIZE;

	if (get_unaligned_le32(buf) != OSF_FRAME_MAGIC)
		return -EPROTO;

	if (get_unaligned_le16(buf + 6) != OSF_FRAME_HEADER_LEN)
		return -EPROTO;

	payload_len = get_unaligned_le32(buf + 10);
	if (payload_len > len - OSF_FRAME_MIN_LEN)
		return -EMSGSIZE;

	total_len = OSF_FRAME_HEADER_LEN + payload_len + OSF_FRAME_CRC_LEN;
	expected_crc = osf_crc32_ieee(buf, OSF_FRAME_HEADER_LEN + payload_len);
	actual_crc = get_unaligned_le32(buf + OSF_FRAME_HEADER_LEN + payload_len);

	if (actual_crc != expected_crc)
		return -EBADMSG;

	*frame = (struct osf_frame) {
		.protocol_major = buf[4],
		.protocol_minor = buf[5],
		.message_type = get_unaligned_le16(buf + 8),
		.payload_len = payload_len,
		.sequence = get_unaligned_le64(buf + 14),
		.timestamp_us = get_unaligned_le64(buf + 22),
		.flags = get_unaligned_le32(buf + 30),
		.payload = buf + OSF_FRAME_HEADER_LEN,
		.crc = actual_crc,
	};
	*frame_len = total_len;

	return 0;
}

int osf_protocol_decode_sensor_sample(const struct osf_frame *frame,
				      struct osf_sensor_sample *sample)
{
	u16 channel_count;
	u16 sample_format;
	u16 sensor_type;
	size_t expected_len;
	const u8 *payload;

	if (!frame || !sample || !frame->payload)
		return -EINVAL;

	if (frame->message_type != OSF_MSG_SENSOR_SAMPLE)
		return -EPROTO;

	if (frame->payload_len < OSF_SENSOR_SAMPLE_BASE_LEN)
		return -EMSGSIZE;

	payload = frame->payload;
	sensor_type = get_unaligned_le16(payload);
	channel_count = get_unaligned_le16(payload + 4);
	sample_format = get_unaligned_le16(payload + 6);

	if (!osf_sensor_type_valid(sensor_type))
		return -EPROTO;

	if (!channel_count)
		return -EPROTO;

	if (sample_format != OSF_SAMPLE_FORMAT_S32)
		return -EPROTO;

	expected_len = OSF_SENSOR_SAMPLE_BASE_LEN + channel_count * sizeof(__le32);
	if (frame->payload_len != expected_len)
		return -EMSGSIZE;

	*sample = (struct osf_sensor_sample) {
		.sensor_type = sensor_type,
		.sensor_index = get_unaligned_le16(payload + 2),
		.channel_count = channel_count,
		.sample_format = sample_format,
		.scale_nano = get_unaligned_le32(payload + 8),
		.samples = payload + OSF_SENSOR_SAMPLE_BASE_LEN,
	};

	return 0;
}

int osf_protocol_sensor_sample_value(const struct osf_sensor_sample *sample,
				     u16 index, s32 *value)
{
	if (!sample || !sample->samples || !value)
		return -EINVAL;

	if (index >= sample->channel_count)
		return -ERANGE;

	/* Samples are little-endian two's-complement signed values. */
	*value = get_unaligned_le32(sample->samples + index * sizeof(__le32));

	return 0;
}

int osf_protocol_decode_device_status(const struct osf_frame *frame,
				      struct osf_device_status *status)
{
	const u8 *payload;

	if (!frame || !status || !frame->payload)
		return -EINVAL;

	if (frame->message_type != OSF_MSG_DEVICE_STATUS)
		return -EPROTO;

	if (frame->payload_len != OSF_DEVICE_STATUS_LEN)
		return -EMSGSIZE;

	payload = frame->payload;
	*status = (struct osf_device_status) {
		.uptime_s = get_unaligned_le32(payload),
		.status_flags = get_unaligned_le32(payload + 4),
		.error_flags = get_unaligned_le32(payload + 8),
		.dropped_frames = get_unaligned_le32(payload + 12),
	};

	return 0;
}

int osf_protocol_decode_capability_report(const struct osf_frame *frame,
					  struct osf_capability_report *report)
{
	u16 capability_count;
	size_t expected_len;
	const u8 *payload;

	if (!frame || !report || !frame->payload)
		return -EINVAL;

	if (frame->message_type != OSF_MSG_CAPABILITY_REPORT)
		return -EPROTO;

	if (frame->payload_len < OSF_CAP_REPORT_BASE_LEN)
		return -EMSGSIZE;

	payload = frame->payload;
	capability_count = get_unaligned_le16(payload);

	expected_len = OSF_CAP_REPORT_BASE_LEN +
		       capability_count * OSF_CAP_SENSOR_ENTRY_LEN;
	if (frame->payload_len != expected_len)
		return -EMSGSIZE;

	*report = (struct osf_capability_report) {
		.capability_count = capability_count,
		.entries = payload + OSF_CAP_REPORT_BASE_LEN,
	};

	return 0;
}

int osf_protocol_decode_capability_entry(const struct osf_capability_report
					 *report, u16 index,
					 struct osf_capability_entry *entry)
{
	const u8 *payload;

	if (!report || !report->entries || !entry)
		return -EINVAL;

	if (index >= report->capability_count)
		return -ERANGE;

	payload = report->entries + index * OSF_CAP_SENSOR_ENTRY_LEN;
	*entry = (struct osf_capability_entry) {
		.sensor_type = get_unaligned_le16(payload),
		.sensor_index = get_unaligned_le16(payload + 2),
		.channel_count = get_unaligned_le16(payload + 4),
		.sample_format = get_unaligned_le16(payload + 6),
		.scale_nano = get_unaligned_le32(payload + 8),
		.flags = get_unaligned_le32(payload + 12),
	};

	return 0;
}
