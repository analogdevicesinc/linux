/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MAX77658_REGULATOR_H_
#define __MAX77658_REGULATOR_H_

/* Register map */
/* Regulator registers */
#define REG_CNFG_LDO0_A 0x48
#define REG_CNFG_LDO0_B 0x49
#define REG_CNFG_LDO1_A 0x4A
#define REG_CNFG_LDO1_B 0x4B

/* Bit Fields */
/* Config LDO A */
#define BITS_CONFIG_LDOx_A_TV_LDO	BITS(6, 0)

/* Config LDO B */
#define BITS_CONFIG_LDOx_B_EN_LDO	BITS(2, 0)


struct max77658_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

enum max77658_regulators {
	MAX77658_SAFEOUT0 = 0,
	MAX77658_SAFEOUT1 = 1,
	MAX77658_REG_MAX,
};

struct max77658_regulator_dev {
	struct device *dev;
	struct max77658_dev *max77658;
	struct regulator_dev **rdev;
	int num_regulators;
	struct max77658_regulator_data *regulators;
};

#endif // __MAX77658_REGULATOR_H_
