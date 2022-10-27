/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __MAX77541_MFD_H__
#define __MAX77541_MFD_H__

/*      REGISTERS       */

/*      GLOBAL CONFIG1       */
#define MAX77541_REG_INT_SRC                    0x00
#define MAX77541_REG_INT_SRC_M                  0x01
#define MAX77541_REG_TOPSYS_INT                 0x02
#define MAX77541_REG_TOPSYS_INT_M               0x03
#define MAX77541_REG_TOPSYS_STAT                0x04
#define MAX77541_REG_DEVICE_CFG1                0x06
#define MAX77541_REG_DEVICE_CFG2                0x07
#define MAX77541_REG_TOPSYS_CFG                 0x08
#define MAX77541_REG_PROT_CFG                   0x09
#define MAX77541_REG_EN_CTRL                    0x0B

/*      GLOBAL CONFIG2       */
#define MAX77541_REG_GLB_CFG1                   0x11

/*      BUCK CONFIG       */
#define MAX77541_REG_BUCK_INT                   0x20
#define MAX77541_REG_BUCK_INT_M                 0x21
#define MAX77541_REG_BUCK_STAT                  0x22

/*      BUCK1       */
#define MAX77541_REG_M1_VOUT                    0x23
#define MAX77541_REG_M1_CFG1                    0x25
#define MAX77541_REG_M1_CFG2                    0x26
#define MAX77541_REG_M1_CFG3                    0x27

/*      BUCK2       */
#define MAX77541_REG_M2_VOUT                    0x33
#define MAX77541_REG_M2_CFG1                    0x35
#define MAX77541_REG_M2_CFG2                    0x36
#define MAX77541_REG_M2_CFG3                    0x37

#define MAX77541_REG_NOT_AVAILABLE              0xFF

/* INTERRUPT MASKS*/
#define MAX77541_REG_INT_SRC_MASK               0x00
#define MAX77541_REG_TOPSYS_INT_MASK            0x00
#define MAX77541_REG_BUCK_INT_MASK              0x00

/*BITS OF REGISTERS*/

#define MAX77541_BIT_INT_SRC_TOPSYS             BIT(0)
#define MAX77541_BIT_INT_SRC_BUCK               BIT(1)

#define MAX77541_BIT_TOPSYS_INT_TJ_120C         BIT(0)
#define MAX77541_BIT_TOPSYS_INT_TJ_140C         BIT(1)
#define MAX77541_BIT_TOPSYS_INT_TSHDN           BIT(2)
#define MAX77541_BIT_TOPSYS_INT_UVLO            BIT(3)
#define MAX77541_BIT_TOPSYS_INT_ALT_SWO         BIT(4)
#define MAX77541_BIT_TOPSYS_INT_EXT_FREQ_DET    BIT(5)

#define MAX77541_BIT_BUCK_INT_M1_POK_FLT        BIT(0)
#define MAX77541_BIT_BUCK_INT_M2_POK_FLT        BIT(1)
#define MAX77541_BIT_BUCK_INT_M1_SCFLT          BIT(4)
#define MAX77541_BIT_BUCK_INT_M2_SCFLT          BIT(5)

#define MAX77541_BITS_DEVICE_CFG1_SEL1_LATCH    GENMASK(4, 0)
#define MAX77541_BITS_DEVICE_CFG2_SEL2_LATCH    GENMASK(4, 0)

#define MAX77541_BIT_TOPSYS_CFG_DIS_ALT_IN      BIT(0)

#define MAX77541_BITS_PROT_CFG_POK_TO           GENMASK(1, 0)
#define MAX77541_BIT_PROT_CFG_EN_FTMON          BIT(2)

#define MAX77541_BIT_M1_EN                      BIT(0)
#define MAX77541_BIT_M2_EN                      BIT(1)
#define MAX77541_BIT_M1_LPM                     BIT(4)
#define MAX77541_BIT_M2_LPM                     BIT(5)

#define MAX77541_BITS_GLB_CFG1_SSTOP_SR         GENMASK(2, 0)
#define MAX77541_BITS_GLB_CFG1_SSTRT_SR         GENMASK(5, 3)

#define MAX77541_BITS_MX_VOUT                   GENMASK(7, 0)

#define MAX77541_BITS_MX_CFG1_RU_SR             GENMASK(2, 0)
#define MAX77541_BITS_MX_CFG1_RD_SR             GENMASK(5, 3)
#define MAX77541_BITS_MX_CFG1_RNG               GENMASK(7, 6)

#define MAX77541_BIT_MX_CFG2_FPWM               BIT(0)
#define MAX77541_BIT_MX_CFG2_FSREN              BIT(1)
#define MAX77541_BITS_MX_CFG2_SS_PAT            GENMASK(3, 2)
#define MAX77541_BITS_MX_CFG2_SS_FREQ           GENMASK(5, 4)
#define MAX77541_BITS_MX_CFG2_SS_ENV            GENMASK(7, 6)

#define MAX77541_BITS_MX_CFG3_ILIM              GENMASK(1, 0)
#define MAX77541_BITS_MX_CFG3_FREQ              GENMASK(3, 2)
#define MAX77541_BIT_MX_CFG3_FTRAK              BIT(4)
#define MAX77541_BIT_MX_CFG3_RESRESH            BIT(5)
#define MAX77541_BIT_MX_CFG3_ADIS1              BIT(6)
#define MAX77541_BIT_MX_CFG3_ADIS100            BIT(7)

#define MAX77541_MAX_REGULATORS 2

/*      ADC       */
#define MAX77541_REG_ADC_INT                    0x70
#define MAX77541_REG_ADC_MSK                    0x71

#define MAX77541_REG_ADC_DATA_CH1               0x72
#define MAX77541_REG_ADC_DATA_CH2               0x73
#define MAX77541_REG_ADC_DATA_CH3               0x74
#define MAX77541_REG_ADC_DATA_CH6               0x77

#define MAX77541_BIT_ADC_INT_CH1_I              BIT(0)
#define MAX77541_BIT_ADC_INT_CH2_I              BIT(1)
#define MAX77541_BIT_ADC_INT_CH3_I              BIT(2)
#define MAX77541_BIT_ADC_INT_CH6_I              BIT(5)

enum dev_type {
	MAX77540 = 1,
	MAX77541 = 2,
};

enum max77541_regulators {
	MAX77541_BUCK1 = 1,
	MAX77541_BUCK2,
};

enum mx_range {
	LOW_RANGE,
	MID_RANGE,
	HIGH_RANGE,
	RESERVED
};

struct max77541_dev {
	void *pdata;
	struct device *dev;

	struct regmap_irq_chip_data *irq_data;
	struct regmap_irq_chip_data *irq_buck;
	struct regmap_irq_chip_data *irq_topsys;
	struct regmap_irq_chip_data *irq_adc;

	struct i2c_client *i2c;
	struct regmap *regmap;

	u8 type;
};

#endif /* __MAX77541_MFD_H__ */
