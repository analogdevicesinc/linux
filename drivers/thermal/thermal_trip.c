// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2008 Intel Corp
 *  Copyright (C) 2008 Zhang Rui <rui.zhang@intel.com>
 *  Copyright (C) 2008 Sujith Thomas <sujith.thomas@intel.com>
 *  Copyright 2022 Linaro Limited
 *
 * Thermal trips handling
 */
#include "thermal_core.h"

static const char *trip_type_names[] = {
	[THERMAL_TRIP_ACTIVE] = "active",
	[THERMAL_TRIP_PASSIVE] = "passive",
	[THERMAL_TRIP_HOT] = "hot",
	[THERMAL_TRIP_CRITICAL] = "critical",
};

const char *thermal_trip_type_name(enum thermal_trip_type trip_type)
{
	if (trip_type < THERMAL_TRIP_ACTIVE || trip_type > THERMAL_TRIP_CRITICAL)
		return "unknown";

	return trip_type_names[trip_type];
}

int for_each_thermal_trip(struct thermal_zone_device *tz,
			  int (*cb)(struct thermal_trip *, void *),
			  void *data)
{
	struct thermal_trip_desc *td;
	int ret;

	for_each_trip_desc(tz, td) {
		ret = cb(&td->trip, data);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(for_each_thermal_trip);

int thermal_zone_for_each_trip(struct thermal_zone_device *tz,
			       int (*cb)(struct thermal_trip *, void *),
			       void *data)
{
	int ret;

	mutex_lock(&tz->lock);
	ret = for_each_thermal_trip(tz, cb, data);
	mutex_unlock(&tz->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(thermal_zone_for_each_trip);

void thermal_zone_set_trips(struct thermal_zone_device *tz, int low, int high)
{
	int ret;

	lockdep_assert_held(&tz->lock);

	if (!tz->ops.set_trips)
		return;

	/* No need to change trip points */
	if (tz->prev_low_trip == low && tz->prev_high_trip == high)
		return;

	tz->prev_low_trip = low;
	tz->prev_high_trip = high;

	dev_dbg(&tz->device,
		"new temperature boundaries: %d < x < %d\n", low, high);

	/*
	 * Set a temperature window. When this window is left the driver
	 * must inform the thermal core via thermal_zone_device_update.
	 */
	ret = tz->ops.set_trips(tz, low, high);
	if (ret)
		dev_err(&tz->device, "Failed to set trips: %d\n", ret);
}

int thermal_zone_trip_id(const struct thermal_zone_device *tz,
			 const struct thermal_trip *trip)
{
	/*
	 * Assume the trip to be located within the bounds of the thermal
	 * zone's trips[] table.
	 */
	return trip_to_trip_desc(trip) - tz->trips;
}

void thermal_zone_set_trip_hyst(struct thermal_zone_device *tz,
				struct thermal_trip *trip, int hyst)
{
	WRITE_ONCE(trip->hysteresis, hyst);
	thermal_notify_tz_trip_change(tz, trip);
}

void thermal_zone_set_trip_temp(struct thermal_zone_device *tz,
				struct thermal_trip *trip, int temp)
{
	if (trip->temperature == temp)
		return;

	WRITE_ONCE(trip->temperature, temp);
	thermal_notify_tz_trip_change(tz, trip);

	if (temp == THERMAL_TEMP_INVALID) {
		struct thermal_trip_desc *td = trip_to_trip_desc(trip);

		if (tz->temperature >= td->threshold) {
			/*
			 * The trip has been crossed on the way up, so some
			 * adjustments are needed to compensate for the lack
			 * of it going forward.
			 */
			if (trip->type == THERMAL_TRIP_PASSIVE) {
				tz->passive--;
				WARN_ON_ONCE(tz->passive < 0);
			}
			thermal_zone_trip_down(tz, trip);
		}
		/*
		 * Invalidate the threshold to avoid triggering a spurious
		 * trip crossing notification when the trip becomes valid.
		 */
		td->threshold = INT_MAX;
	}
}
EXPORT_SYMBOL_GPL(thermal_zone_set_trip_temp);
