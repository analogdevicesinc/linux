/*
 * include/linux/input/adxl362.h
 *
 * Digital Accelerometer characteristics are highly application specific
 * and may vary between boards and models. The platform_data for the
 * device's "struct device" holds this information.
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_INPUT_ADXL362_H__
#define __LINUX_INPUT_ADXL362_H__

enum adxl_odr {
	ADXL_ODR_12_5HZ	= 13,
	ADXL_ODR_25HZ	= 25,
	ADXL_ODR_50HZ	= 50,
	ADXL_ODR_100HZ	= 100,
	ADXL_ODR_200HZ	= 200,
	ADXL_ODR_400HZ	= 400,
};

enum adxl_g_range {
	ADXL_RANGE_PM_2g = 2,
	ADXL_RANGE_PM_4g = 4,
	ADXL_RANGE_PM_8g = 8,
};

enum adxl_power_mode {
	ADXL_NORM_OPERATION	= 0,
	ADXL_LOW_NOISE_MODE	= 1,
	ADXL_ULTRA_LOW_NOISE_MODE = 2,
};

struct adxl362_platform_data {
	/*
	 * data_range:
	 * Measurement range selection +/- 2,4,8 g
	 */
	enum adxl_g_range data_range;

	/*
	 * low_power_mode:
	 * Power versus noise tradeoff.
	 */
	enum adxl_power_mode low_power_mode;

	/*
	 * data_rate:
	 * Selects the output data rate (ODR).
	 */
	enum adxl_odr data_rate;

	/*
	 * half_bw:
	 * Sets the anti-aliasing filter to 1/4 of the output data rate (ODR)
	 */
	bool half_bw;

	/*
	 * watermark_odr:
	 * The Watermark feature can be used to reduce the interrupt/poll load
	 * of the system. The FIFO fills up to watermark value in sample sets
	 * [1..170] and then generates an interrupt. Each ODR can have it's
	 * own watermark.
	 */
	u8 watermark_odr_12Hz;
	u8 watermark_odr_25Hz;
	u8 watermark_odr_50Hz;
	u8 watermark_odr_100Hz;
	u8 watermark_odr_200Hz;
	u8 watermark_odr_400Hz;

	/*
	 * When acceleration measurements are received from the ADXL362
	 * events are sent to the input event subsystem. The following settings
	 * select the event code for ABS x, y and z axis data
	 * respectively. The event codes can also be negated to further account
	 * for sensor orientation.
	 */
	s32 ev_code_x;	/* (+/-)ABS_X,Y,Z */
	s32 ev_code_y;	/* (+/-)ABS_X,Y,Z */
	s32 ev_code_z;	/* (+/-)ABS_X,Y,Z */
	s32 abs_fuzz;	/* input fuzz val */

	/*
	 * [in]activity_threshold:
	 * holds the threshold value for activity detection.
	 * The data format is unsigned. The scale factor is
	 * 1mg/LSB.
	 */
	u16 activity_threshold;
	u16 inactivity_threshold;

	/*
	 * [in]activity_time:
	 * is an unsigned time value representing the
	 * amount of time that acceleration must be [below]/above the value in
	 * [in]activity_threshold for [in]activity to be declared.
	 * The scale factor is 1ms/LSB.
	 */
	u32 inactivity_time;
	u32 activity_time;

	/*
	 * referenced_[in]activity_en:
	 * Sets [in]activity detection to operate in referenced mode opposed to
	 * absolute mode.
	 */
	bool referenced_activity_en;
	bool referenced_inactivity_en;

	/*
	 * Use ADXL362 INT2 pin instead of INT1 pin for interrupt output
	 */
	bool use_int2;

	/*
	 * Optional IRQ flags
	 */
	unsigned irqflags;
};
#endif
