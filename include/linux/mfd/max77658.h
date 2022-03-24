/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MAX77658_MFD_H__
#define __MAX77658_MFD_H__

/* MAX77658 Top Devices */
#define MAX77658_NAME			"max77658"

/* MAX77658 PMIC Devices */
#define MAX77658_REGULATOR_NAME		MAX77658_NAME"-regulator"
#define MAX77658_CHARGER_NAME		MAX77658_NAME"-charger"
#define MAX77658_FUELGAUGE_NAME		MAX77658_NAME"-battery"

/* PMIC */
#define REG_INT_GLBL0	0x00
#define REG_INT_CHG	0x01
#define REG_INT_GLBL1	0x04
#define REG_ERCFLAG	0x05
#define REG_STAT_GLBL	0x06
#define REG_INT_M_CHG	0x07
#define REG_INTM_GLBL0	0x08
#define REG_INTM_GLBL1	0x09
#define REG_CNFG_GLBL	0x10
#define REG_CNFG_GPIO0	0x11
#define REG_CNFG_GPIO1	0x12
#define REG_CNFG_GPIO2	0x13
#define REG_CID		0x14
#define REG_CNFG_WDT	0x17

/* Bit Fields */
/* Global-0 Interrupt Registers */
#define BIT_INT_GLBL0_GPIO0_F	BIT(0)
#define BIT_INT_GLBL0_GPIO0_R	BIT(1)
#define BIT_INT_GLBL0_EN_R	BIT(2)
#define BIT_INT_GLBL0_EN_F	BIT(3)
#define BIT_INT_GLBL0_TJAL1_R	BIT(4)
#define BIT_INT_GLBL0_TJAL2_R	BIT(5)
#define BIT_INT_GLBL0_DOD1_R	BIT(6)
#define BIT_INT_GLBL0_DOD0_R	BIT(7)

/* Global-1 Interrupt Registers */
#define BIT_INT_GLBL1_GPIO1_F	BIT(0)
#define BIT_INT_GLBL1_GPIO1_R	BIT(1)
#define BIT_INT_GLBL1_SBB0_F	BIT(2)
#define BIT_INT_GLBL1_SBB1_F	BIT(3)
#define BIT_INT_GLBL1_SBB2_F	BIT(4)
#define BIT_INT_GLBL1_LDO0_F	BIT(5)
#define BIT_INT_GLBL1_LDO1_F	BIT(6)

/* Charger Interrupt Registers */
#define BIT_INT_THM		BIT(0)
#define BIT_INT_CHG		BIT(1)
#define BIT_INT_CHGIN		BIT(2)
#define BIT_INT_TJ_REG		BIT(3)
#define BIT_INT_CHGIN_CTRL	BIT(4)
#define BIT_INT_SYS_CTRL	BIT(5)
#define BIT_INT_SYS_CNFG	BIT(6)
#define BIT_INT_DIS_AICL	BIT(7)

/* Useful Macros */
#undef  __CONST_FFS
#define __CONST_FFS(_x) \
		((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
						((_x) & 0x04 ? 2 : 3)) :\
		((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
						((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
		((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
	((BIT(_end) - BIT(_start)) + BIT(_end))

/* Sub Modules Support */
enum {
	MAX77658_DEV_REGULATOR = 0,
	MAX77658_DEV_CHARGER,
	MAX77658_DEV_FUELGAUGE,
	/***/
	MAX77658_DEV_NUM_OF_DEVICES,
};

struct max77658_dev {
	void *pdata;
	struct device *dev;

	int irq;

	struct regmap_irq_chip_data *irqc_glbl0;
	struct regmap_irq_chip_data *irqc_glbl1;
	struct regmap_irq_chip_data *irqc_chg;

	struct i2c_client *pmic; /* 0x90, CLOGIC/SAFELDOS/CHARGER */
	struct i2c_client *fuel; /* 0x6C, FUEL GAUGE */

	struct regmap *regmap_pmic; /* CLOGIC/SAFELDOS/CHARGER */
	struct regmap *regmap_fuel; /* FUEL GAUGE */
};

#endif /* !__MAX77658_MFD_H__ */
