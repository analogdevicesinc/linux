/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OSF_PROTOCOL_H
#define _OSF_PROTOCOL_H

#include <linux/bits.h>
#include <linux/types.h>

#define OSF_PROTOCOL_MAJOR		0
#define OSF_PROTOCOL_MINOR		0
#define OSF_FRAME_HEADER_LEN		38
#define OSF_FRAME_CRC_LEN		4
#define OSF_FRAME_MIN_LEN		(OSF_FRAME_HEADER_LEN + OSF_FRAME_CRC_LEN)
#define OSF_FRAME_MAGIC			0x3046534f /* "OSF0", little-endian */

#define OSF_SENSOR_SAMPLE_BASE_LEN	16
#define OSF_DEVICE_STATUS_LEN		20
#define OSF_CAP_REPORT_BASE_LEN		4
#define OSF_CAP_SENSOR_ENTRY_LEN		20
#define OSF_CAPABILITY_FLAGS_MASK	GENMASK(1, 0)

enum osf_message_type {
	OSF_MSG_SENSOR_SAMPLE		= 0x0001,
	OSF_MSG_DEVICE_STATUS		= 0x0002,
	OSF_MSG_CAPABILITY_REPORT	= 0x0003,
};

enum osf_sensor_type {
	OSF_SENSOR_ACCELEROMETER		= 0x0001,
	OSF_SENSOR_GYROSCOPE		= 0x0002,
	OSF_SENSOR_MAGNETOMETER		= 0x0003,
	OSF_SENSOR_BAROMETER		= 0x0004,
	OSF_SENSOR_TEMPERATURE		= 0x0005,
	OSF_SENSOR_HUMIDITY		= 0x0006,
	OSF_SENSOR_AMBIENT_LIGHT		= 0x0007,
	OSF_SENSOR_PROXIMITY		= 0x0008,
};

enum osf_sample_format {
	OSF_SAMPLE_FORMAT_S32		= 0x0001,
};

struct osf_frame {
	u8 protocol_major;
	u8 protocol_minor;
	u16 message_type;
	u32 payload_len;
	u64 sequence;
	u64 timestamp_us;
	u32 flags;
	/* payload points into the caller-owned frame buffer. */
	const u8 *payload;
	u32 crc;
};

struct osf_sensor_sample {
	u16 sensor_type;
	u16 sensor_index;
	u16 channel_count;
	u16 sample_format;
	u32 scale_nano;
	const u8 *samples;
};

struct osf_device_status {
	u32 uptime_s;
	u32 status_flags;
	u32 error_flags;
	u32 dropped_frames;
};

struct osf_capability_report {
	u16 capability_count;
	const u8 *entries;
};

struct osf_capability_entry {
	u16 sensor_type;
	u16 sensor_index;
	u16 channel_count;
	u16 sample_format;
	u32 scale_nano;
	u32 flags;
};

int osf_protocol_decode_frame(const u8 *buf, size_t len,
			      struct osf_frame *frame, size_t *frame_len);
int osf_protocol_decode_sensor_sample(const struct osf_frame *frame,
				      struct osf_sensor_sample *sample);
int osf_protocol_decode_device_status(const struct osf_frame *frame,
				      struct osf_device_status *status);
int osf_protocol_decode_capability_report(const struct osf_frame *frame,
					  struct osf_capability_report *report);
int osf_protocol_decode_capability_entry(const struct osf_capability_report
					 *report, u16 index,
					 struct osf_capability_entry *entry);
int osf_protocol_sensor_sample_value(const struct osf_sensor_sample *sample,
				     u16 index, s32 *value);

#endif
