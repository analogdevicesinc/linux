/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _OSF_CORE_H
#define _OSF_CORE_H

#include <linux/mutex.h>
#include <linux/types.h>

#include "osf_protocol.h"

#define OSF_MAX_SAMPLE_CHANNELS	3
#define OSF_MAX_CAPABILITIES	16

struct device;
struct iio_dev;

struct osf_latest_sample {
	u16 sensor_type;
	u16 sensor_index;
	u16 channel_count;
	u16 sample_format;
	u32 scale_nano;
	s32 values[OSF_MAX_SAMPLE_CHANNELS];
	u64 sequence;
	u64 timestamp_us;
	bool valid;
};

struct osf_capability_cache {
	u16 capability_count;
	struct osf_capability_entry entries[OSF_MAX_CAPABILITIES];
	u64 sequence;
	bool valid;
};

struct osf_status_cache {
	u32 uptime_s;
	u32 status_flags;
	u32 error_flags;
	u32 dropped_frames;
	u64 sequence;
	bool valid;
};

struct osf_iio_binding {
	u16 sensor_type;
	u16 sensor_index;
	struct iio_dev *indio_dev;
};

struct osf_device {
	struct device *dev;
	bool iio_ready;
	/* Protects session_fault and latest samples; nests outside buffer_lock. */
	struct mutex latest_lock;
	bool session_fault;
	struct osf_latest_sample latest_samples[OSF_MAX_CAPABILITIES];
	unsigned int latest_sample_count;
	struct osf_capability_cache capability_cache;
	struct osf_status_cache status_cache;
	struct osf_iio_binding iio_devs[OSF_MAX_CAPABILITIES];
	unsigned int iio_dev_count;
	u64 last_sequence;
};

void osf_core_init(struct osf_device *osf, struct device *dev);
/* Serialize start with receive_frame; pending entries own their data. */
void osf_core_start(struct osf_device *osf);
/* Stop the receive producer before teardown; init starts a fresh session. */
void osf_core_unregister_iio(struct osf_device *osf);
int osf_core_receive_frame(struct osf_device *osf, const u8 *buf, size_t len);
int osf_core_read_latest_sample(struct osf_device *osf, u16 sensor_type,
				u16 sensor_index, u16 channel,
				s32 *value);

#endif
