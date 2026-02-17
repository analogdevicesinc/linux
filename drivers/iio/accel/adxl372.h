/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ADXL371/ADXL372 3-Axis Digital Accelerometer
 *
 * Copyright 2018 Analog Devices Inc.
 */

#ifndef _ADXL372_H_
#define _ADXL372_H_

#define ADXL372_REVID	0x03

struct adxl372_chip_info {
	const char *name;
	const int *samp_freq_tbl;
	const int *bw_freq_tbl;
	unsigned int act_time_scale_us;
	unsigned int act_time_scale_low_us;
	unsigned int inact_time_scale_ms;
	unsigned int inact_time_scale_low_ms;
	unsigned int max_odr;
	bool has_fifo;
};

extern const struct adxl372_chip_info adxl371_chip_info;
extern const struct adxl372_chip_info adxl372_chip_info;

int adxl372_probe(struct device *dev, struct regmap *regmap,
		  int irq, const struct adxl372_chip_info *chip_info);
bool adxl372_readable_noinc_reg(struct device *dev, unsigned int reg);

#endif /* _ADXL372_H_ */
