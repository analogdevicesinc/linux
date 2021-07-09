/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADXL355 3-Axis Digital Accelerometer
 *
 * Copyright (c) 2021 Puranjay Mohan <puranjay12@gmail.com>
 */

#ifndef _ADXL355_H_
#define _ADXL355_H_

#include <linux/regmap.h>

/* ADXL355 Register Definitions */
#define ADXL355_DEVID_AD	0x00
#define ADXL355_DEVID_MST	0x01
#define ADXL355_PARTID		0x02
#define ADXL355_REVID		0x03
#define ADXL355_STATUS		0x04
#define ADXL355_FIFO_ENTRIES	0x05
#define ADXL355_TEMP2		0x06
#define ADXL355_XDATA3		0x08
#define ADXL355_YDATA3		0x0B
#define ADXL355_ZDATA3		0x0E
#define ADXL355_FIFO_DATA	0x11
#define ADXL355_OFFSET_X_H	0x1E
#define ADXL355_OFFSET_Y_H	0x20
#define ADXL355_OFFSET_Z_H	0x22
#define ADXL355_ACT_EN		0x24
#define ADXL355_ACT_THRESH_H	0x25
#define ADXL355_ACT_THRESH_L	0x26
#define ADXL355_ACT_COUNT	0x27
#define ADXL355_FILTER		0x28
#define ADXL355_FIFO_SAMPLES	0x29
#define ADXL355_INT_MAP		0x2A
#define ADXL355_SYNC		0x2B
#define ADXL355_RANGE		0x2C
#define ADXL355_POWER_CTL	0x2D
#define ADXL355_SELF_TEST	0x2E
#define ADXL355_RESET		0x2F

#define ADXL355_DEVID_AD_VAL	0xAD
#define ADXL355_DEVID_MST_VAL	0x1D
#define ADXL355_PARTID_VAL	0xED
#define ADXL355_REVID_VAL	0x01
#define ADXL355_RESET_CODE	0x52

#define ADXL355_POWER_CTL_MODE_MSK	GENMASK(1, 0)

#define ADXL355_FILTER_ODR_MSK			GENMASK(3, 0)
#define ADXL355_FILTER_ODR_MODE(x)		((x) & 0xF)
#define ADXL355_FILTER_HPF_MSK			GENMASK(6, 4)
#define ADXL355_FILTER_HPF_MODE(x)		(((x) & 0x7) << 4)

/*
 * The datasheet defines an intercept of 1885 LSB at 25 degC
 * and a slope of -9.05 LSB/C. The following formula can be used to find the
 * temperature:
 * Temp = ((RAW - 1885)/(-9.05)) + 25 but this doesn't follow the format of
 * the IIO which is Temp = (RAW + OFFSET) * SCALE. Hence using some rearranging
 * we get the scale as -110.49723 and offset as -2111.25
 */
#define TEMP_SCALE_VAL -110
#define TEMP_SCALE_VAL2 497238
#define TEMP_OFFSET_VAL -2111
#define TEMP_OFFSET_VAL2 250000

/*
 * At +/- 2g with 20-bit resolution, scale is given in datasheet as
 * 3.9ug/LSB = 0.0000039 * 9.80665 = 0.00003824593 m/s^2
 */
#define ADXL355_NSCALE	38245

extern const struct regmap_range adxl355_read_reg_range[];

extern const struct regmap_access_table adxl355_readable_regs_tbl;

extern const struct regmap_range adxl355_write_reg_range[];

extern const struct regmap_access_table adxl355_writeable_regs_tbl;

int adxl355_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name);
#endif /* _ADXL355_H_ */
