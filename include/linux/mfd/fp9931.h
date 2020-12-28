/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 NXP
 */

#ifndef __FP9931_H_
#define __FP9931_H_

/* Reigster Addresses */

#define FP9931_TMST_VALUE		0x00
#define FP9931_VCOM_SETTING		0x01
#define FP9931_VPOS_VNEG_SETTING	0x02
#define FP9931_PWRON_DELAY		0x03
#define FP9931_CONTROL_REG1		0x0B
#define FP9931_CONTROL_REG2		0x0C

#define VPOS_VNEG_SETTING		GENMASK(5, 0)
#define PWRON_DELAY_tDLY1		GENMASK(1, 0)
#define PWRON_DELAY_tDLY2		GENMASK(3, 2)
#define PWRON_DELAY_tDLY3		GENMASK(5, 4)
#define PWRON_DELAY_tDLY4		GENMASK(7, 6)
#define CONTROL_REG1_V3P3_EN		BIT(1)
#define CONTROL_REG1_SS_TIME		GENMASK(7, 6)
#define CONTROL_REG2_VN_CL		GENMASK(1, 0)
#define CONTROL_REG2_VP_CL		GENMASK(3, 2)
#define CONTROL_REG2_FIX_RD_PTR		BIT(7)

enum {
	FP9931_DISPLAY,
	FP9931_VPOS,
	FP9931_VNEG,
	FP9931_VGH,
	FP9931_VGL,
	FP9931_VCOM,
	FP9931_V3P3,
};

struct fp9931 {
	struct device *dev;
	struct fp9931_platform_data *pdata;

	struct i2c_client *client;

	/* power up delay time: 0ms, 1ms, 2ms, 4ms */
	unsigned int vgl_pwrup;
	unsigned int vneg_pwrup;
	unsigned int vgh_pwrup;
	unsigned int vpos_pwrup;

	/* soft start time for all regulator
	 * voltages power on: 3ms ~ 6ms
	 */
	unsigned int ss_time;

	int gpio_pmic_wakeup;
	int gpio_pmic_pwrgood;
};

struct fp9931_platform_data {
	/* PMIC */
	struct fp9931_regulator_data *regulators;
	int num_regulators;
};

struct fp9931_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

int fp9931_reg_read(struct i2c_client *client, int reg_num, u8 *reg_val);
int fp9931_reg_write(struct i2c_client *client, int reg_num, u8 reg_val);
#endif
