// SPDX-License-Identifier: GPL-2.0+
/*
 * ADF4377 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>

/* ADF4377 REG0000 Map */
#define ADF4377_SOFT_RESET_R_MSK        BIT(7)
#define ADF4377_SOFT_RESET_R(x)			FIELD_PREP(ADF4377_SOFT_RESET_R_MSK, x)
#define ADF4377_SOFT_RESET_MSK         	BIT(0)
#define ADF4377_SOFT_RESET(x)			FIELD_PREP(ADF4377_SOFT_RESET_MSK, x)
#define ADF4377_LSB_FIRST_R_MSK			BIT(6)
#define ADF4377_LSB_FIRST_R(x)       	FIELD_PREP(ADF4377_LSB_FIRST_R_MSK, x)
#define ADF4377_LSB_FIRST_MSK			BIT(1)
#define ADF4377_LSB_FIRST(x)       		FIELD_PREP(ADF4377_LSB_FIRST_MSK, x)
#define ADF4377_ADDRESS_ASC_R_MSK		BIT(5)
#define ADF4377_ADDRESS_ASC_R(x)		FIELD_PREP(ADF4377_ADDRESS_ASC_R_MSK, x)
#define ADF4377_ADDRESS_ASC_MSK			BIT(2)
#define ADF4377_ADDRESS_ASC(x)			FIELD_PREP(ADF4377_ADDRESS_ASC_MSK, x)
#define ADF4377_SDO_ACTIVE_R_MSK       	BIT(4)
#define ADF4377_SDO_ACTIVE_R(x)			FIELD_PREP(ADF4377_SDO_ACTIVE_R_MSK, x)
#define ADF4377_SDO_ACTIVE_MSK         	BIT(3)
#define ADF4377_SDO_ACTIVE(x)			FIELD_PREP(ADF4377_SDO_ACTIVE_MSK, x)

/* ADF4377 REG0000 Bit Definition */
#define ADF4377_SDO_ACTIVE_SPI_3W       0x0
#define ADF4377_SDO_ACTIVE_SPI_4W		0x1

#define ADF4377_ADDR_ASC_AUTO_DECR      0x0
#define ADF4377_ADDR_ASC_AUTO_INCR      0x1

#define ADF4377_LSB_FIRST_MSB           0x0
#define ADF4377_LSB_FIRST_LSB           0x1

#define ADF4377_SOFT_RESET_N_OP         0x0
#define ADF4377_SOFT_RESET_EN           0x1

/* ADF4377 REG0001 Map */
#define ADF4377_SINGLE_INSTR_MSK		BIT(7)
#define ADF4377_SINGLE_INSTR(x)   		FIELD_PREP(ADF4377_SINGLE_INSTRUCTION_MSK, x)
#define ADF4377_R001_RSV6               ((0x0) << 6)
#define ADF4377_MASTER_RB_CTRL_MSK		BIT(5)
#define ADF4377_MASTER_RB_CTRL(x)		FIELD_PREP(ADF4377_MASTER_RB_CTRL_MSK, x)
#define ADF4377_R001_RSV4               ((0x0) << 4)
#define ADF4377_R001_RSV2               ((0x0) << 2)
#define ADF4377_R001_RSV1               ((0x0) << 1)
#define ADF4377_R001_RESERVED           ((0x0 << 3) | (0x0 << 0))

/* ADF4377 REG0001 Bit Definition */
#define ADF4377_SPI_STREAM_EN           0x0
#define ADF4377_SPI_STREAM_DIS          0x1

#define ADF4377_RB_SLAVE_REG            0x0
#define ADF4377_RB_MASTER_REG           0x1

/* ADF4377 REG0003 Bit Definition */
#define ADF4377_R003_RESERVED           (0x0 << 4)
#define ADF4377_CHIP_TYPE               0x06

/* ADF4377 REG0004 Bit Definition */
#define ADF4377_PRODUCT_ID_LSB          0x0005

/* ADF4377 REG0005 Bit Definition */
#define ADF4377_PRODUCT_ID_MSB          0x0005

/* ADF4377 REG000A Map */
#define ADF4377_SCRATCHPAD_MSK			GENMASK(7, 0)
#define ADF4377_SCRATCHPAD(x)           FIELD_PREP(ADF4377_SCRATCHPAD_MSK, x)

/* ADF4377 REG000B Bit Definition */
#define ADF4377_SPI_REVISION            0x01

/* ADF4377 REG000C Bit Definition */
#define ADF4377_VENDOR_ID_LSB           0x456

/* ADF4377 REG000D Bit Definition */
#define ADF4377_VENDOR_ID_MSB			0x456

/* ADF4377 REG000F Bit Definition */
#define ADF4377_R00F_RSV1				0x14

/* ADF4377 REG0010 Map*/
#define ADF4377_N_INT_LSB_MSK			GENMASK(7, 0)
#define ADF4377_N_INT_LSB(x)            FIELD_PREP(ADF4377_N_INT_LSB_MSK, x)

/* ADF4377 REG0011 Map*/
#define ADF4377_EN_AUTOCAL_MSK			BIT(7)
#define ADF4377_EN_AUTOCAL(x)           FIELD_PREP(ADF4377_EN_AUTOCAL_MSK, x)
#define ADF4377_EN_RDBLR_MSK			BIT(6)
#define ADF4377_EN_RDBLR(x)             FIELD_PREP(ADF4377_EN_RDBLR_MSK, x)
#define ADF4377_DCLK_DIV2_MSK			GENMASK(5,4)
#define ADF4377_DCLK_DIV2(x)            FIELD_PREP(ADF4377_DCLK_DIV2_MSK, x)
#define ADF4377_N_INT_MSB_MSK			GENMASK(3,0)
#define ADF4377_N_INT_MSB(x)            FIELD_PREP(ADF4377_N_INT_MSB_MSK, x)

/* ADF4377 REG0011 Bit Definition */
#define ADF4377_VCO_CALIB_DIS           0x0
#define ADF4377_VCO_CALIB_EN            0x1

#define ADF4377_REF_DBLR_DIS            0x0
#define ADF4377_REF_DBLR_EN             0x1

#define ADF4377_DCLK_DIV2_1             0x0
#define ADF4377_DCLK_DIV2_2             0x1
#define ADF4377_DCLK_DIV2_4             0x2
#define ADF4377_DCLK_DIV2_8             0x3

/* ADF4377 REG0012 Map*/
#define ADF4377_CLKOUT_DIV_MSK			GENMASK(7, 6)
#define ADF4377_CLKOUT_DIV(x)           FIELD_PREP(ADF4377_CLKOUT_DIV_MSK, x)
#define ADF4377_R_DIV_MSK				GENMASK(5, 0)
#define ADF4377_R_DIV(x)                FIELD_PREP(ADF4377_R_DIV_MSK, x)

/* ADF4377 REG0012 Bit Definition */
#define ADF4377_CLKOUT_DIV_1            0x0
#define ADF4377_CLKOUT_DIV_2            0x1
#define ADF4377_CLKOUT_DIV_4            0x2
#define ADF4377_CLKOUT_DIV_8            0x3

#define ADF4377_MIN_R_DIV               0x00
#define ADF4378_MAX_R_DIV               0x3F

/* ADF4377 REG0013 Map */
#define ADF4377_R013_RSV1               (0x0 << 6)
#define ADF4377_M_VCO_CORE_MSK			GENMASK(5,4)
#define ADF4377_M_VCO_CORE(x)           FIELD_PREP(ADF4377_M_VCO_CORE_MSK, x)
#define ADF4377_M_VCO_BIAS_MSK          GENMASK(3,0)
#define ADF4377_M_VCO_BIAS(x)          	FIELD_PREP(ADF4377_M_VCO_BIAS_MSK, x)

/* ADF4377 REG0013 Bit Definition */
#define ADF4377_M_VCO_0                 0x0
#define ADF4377_M_VCO_1                 0x1
#define ADF4377_M_VCO_2                 0x2
#define ADF4377_M_VCO_3                 0x3

#define M_VCO_BIAS_MIN                  0xF
#define M_VCO_BIAS_MAX                  0x0

/* ADF4377 REG0014 Map */
#define ADF4377_M_VCO_BAND_MSK          GENMASK(7,0)
#define ADF4377_M_VCO_BAND(x)           FIELD_PREP(ADF4377_M_VCO_BAND_MSK, x)

/* ADF4377 REG0014 Bit Definition */
#define ADF4377_VCO_BAND_MIN            0xFF
#define ADF4377_VCO_BAND_MAX            0x00

/* ADF4377 REG0015 Map */
#define ADF4377_BLEED_I_LSB_MSK			GENMASK(7, 6)
#define ADF4377_BLEED_I_LSB(x)          FIELD_PREP(ADF4377_BLEED_I_LSB_MSK, x)
#define ADF4377_BLEED_POL_MSK			BIT(5)
#define ADF4377_BLEED_POL(x)            FIELD_PREP(ADF4377_BLEED_POL_MSK, x)
#define ADF4377_EN_BLEED_MSK			BIT(4)
#define ADF4377_EN_BLEED(x)             FIELD_PREP(ADF4377_EN_BLEED_MSK, x)
#define ADF4377_CP_I_MSK				GENMASK(3, 0)
#define ADF4377_CP_I(x)                 FIELD_PREP(ADF4377_CP_I_MSK, x)

/* ADF4377 REG0015 Bit Description */
#define ADF4377_CURRENT_SINK            0x0
#define ADF4377_CURRENT_SOURCE          0x1

#define ADF4377_BLEED_CURR_DIS          0x0
#define ADF4377_BLEED_CURR_EN           0x1

#define ADF4377_CP_0MA7                 0x0
#define ADF4377_CP_0MA9                 0x1
#define ADF4377_CP_1MA1                 0x2
#define ADF4377_CP_1MA3                 0x3
#define ADF4377_CP_1MA4                 0x4
#define ADF4377_CP_1MA8                 0x5
#define ADF4377_CP_2MA2                 0x6
#define ADF4377_CP_2MA5                 0x7
#define ADF4377_CP_2MA9                 0x8
#define ADF4377_CP_3MA6                 0x9
#define ADF4377_CP_4MA3                 0xA
#define ADF4377_CP_5MA0                 0xB
#define ADF4377_CP_5MA7                 0xC
#define ADF4377_CP_7MA2                 0xD
#define ADF4377_CP_8MA6                 0xE
#define ADF4377_CP_10MA1                0xF

/* ADF4377 REG0016 Map */

#define ADF4377_BLEED_I_MSB_MSK			GENMASK(7, 0)
#define ADF4377_BLEED_I_MSB(x)          FIELD_PREP(ADF4377_BLEED_I_MSB_MSK, x)

/* ADF4377 REG0017 Map */
#define ADF4377_INV_CLKOUT_MSK			BIT(7)
#define ADF4377_INV_CLKOUT(x)           FIELD_PREP(ADF4377_INV_CLKOUT_MSK, x)
#define ADF4377_N_DEL_MSK				GENMASK(6, 0)
#define ADF4377_N_DEL(x)                FIELD_PREP(ADF4377_N_DEL_MSK, x)

/* ADF4377 REG0017 Bit Definition */
#define ADF4377_CLKOUT_INV_DIS          0x0
#define ADF4377_CLKOUT_INV_EN           0x1

/* ADF4377 REG0018 Map */
#define ADF4377_CMOS_OV_MSK				BIT(7)
#define ADF4377_CMOS_OV(x)              FIELD_PREP(ADF4377_CMOS_OV_MSK, x)
#define ADF4377_R_DEL_MSK				GENMASK(6, 0)
#define ADF4377_R_DEL(x)                FIELD_PREP(ADF4377_R_DEL_MSK, x)

/* ADF4377 REG0018 Bit Definition */
#define ADF4377_1V8_LOGIC               0x0
#define ADF4377_3V3_LOGIC               0x1

#define ADF4377_R_DEL_MIN               0x00
#define ADF4377_R_DEL_MAX               0x7F

/* ADF4377 REG0019 Map */
#define ADF4377_CLKOUT2_OP_MSK			GENMASK(7, 6)
#define ADF4377_CLKOUT2_OP(x)           FIELD_PREP(ADF4377_CLKOUT2_OP_MSK, x)
#define ADF4377_CLKOUT1_OP_MSK			GENMASK(5, 4)
#define ADF4377_CLKOUT1_OP(x)           FIELD_PREP(ADF4377_CLKOUT1_OP_MSK, x)
#define ADF4377_PD_CLK_MSK				BIT(3)
#define ADF4377_PD_CLK(x)               FIELD_PREP(ADF4377_PD_CLK_MSK, x)
#define ADF4377_PD_RDET_MSK				BIT(2)
#define ADF4377_PD_RDET(x)              FIELD_PREP(ADF4377_PD_RDET_MSK, x)
#define ADF4377_PD_ADC_MSK				BIT(1)
#define ADF4377_PD_ADC(x)               FIELD_PREP(ADF4377_PD_ADC_MSK, x)
#define ADF4377_PD_CALADC_MSK			BIT(0)
#define ADF4377_PD_CALADC(x)            FIELD_PREP(ADF4377_PD_CALADC_MSK, x)

/* ADF4377 REG0019 Bit Definition */
#define ADF4377_CLKOUT_320MV            0x0
#define ADF4377_CLKOUT_427MV            0x1
#define ADF4377_CLKOUT_533MV            0x2
#define ADF4377_CLKOUT_640MV            0x3

#define ADF4377_PD_CLK_N_OP             0x0
#define ADF4377_PD_CLK_PD               0x1

#define ADF4377_PD_RDET_N_OP            0x0
#define ADF4377_PD_RDET_PD              0x1

#define ADF4377_PD_ADC_N_OP             0x0
#define ADF4377_PD_ADC_PD               0x1

#define ADF4377_PD_CALADC_N_OP          0x0
#define ADF4377_PD_CALADC_PD            0x1

/* ADF4377 REG001A Map */
#define ADF4377_PD_ALL_MSK				BIT(7)
#define ADF4377_PD_ALL(x)               FIELD_PREP(ADF4377_PD_ALL_MSK, x)
#define ADF4377_PD_RDIV_MSK				BIT(6)
#define ADF4377_PD_RDIV(x)              FIELD_PREP(ADF4377_PD_RDIV_MSK, x)
#define ADF4377_PD_NDIV_MSK				BIT(5)
#define ADF4377_PD_NDIV(x)              FIELD_PREP(ADF4377_PD_NDIV_MSK, x)
#define ADF4377_PD_VCO_MSK				BIT(4)
#define ADF4377_PD_VCO(x)               FIELD_PREP(ADF4377_PD_VCO_MSK, x)
#define ADF4377_PD_LD_MSK				BIT(3)
#define ADF4377_PD_LD(x)                FIELD_PREP(ADF4377_PD_LD_MSK, x)
#define ADF4377_PD_PFDCP_MSK			BIT(2)
#define ADF4377_PD_PFDCP(x)             FIELD_PREP(ADF4377_PD_PFDCP_MSK, x)
#define ADF4377_PD_CLKOUT1_MSK			BIT(1)
#define ADF4377_PD_CLKOUT1(x)           FIELD_PREP(ADF4377_PD_CLKOUT1_MSK, x)
#define ADF4377_PD_CLKOUT2_MSK			BIT(0)
#define ADF4377_PD_CLKOUT2(x)           FIELD_PREP(ADF4377_PD_CLKOUT2_MSK, x)

/* ADF4377 REG001A Bit Definition */
#define ADF4377_PD_ALL_N_OP             0x0
#define ADF4377_PD_ALL_PD               0x1

#define ADF4377_PD_RDIV_N_OP            0x0
#define ADF4377_PD_RDIV_PD              0x1

#define ADF4377_PD_NDIV_N_OP            0x0
#define ADF4377_PD_NDIV_PD              0x1

#define ADF4377_PD_VCO_N_OP             0x0
#define ADF4377_PD_VCO_PD               0x1

#define ADF4377_PD_LD_N_OP              0x0
#define ADF4377_PD_LD_PD                0x1

#define ADF4377_PD_PFDCP_N_OP           0x0
#define ADF4377_PD_PFDCP_PD             0x1

#define ADF4377_PD_CLKOUT1_N_OP         0x0
#define ADF4377_PD_CLKOUT1_PD           0x1

#define ADF4377_PD_CLKOUT2_N_OP         0x0
#define ADF4377_PD_CLKOUT2_PD           0x1

/* ADF4377 REG001B Map */
#define ADF4377_EN_LOL_MSK				BIT(7)
#define ADF4377_EN_LOL(x)               FIELD_PREP(ADF4377_EN_LOL_MSK, x)
#define ADF4377_LDWIN_PW_MSK			BIT(6)
#define ADF4377_LDWIN_PW(x)             FIELD_PREP(ADF4377_LDWIN_PW_MSK, x)
#define ADF4377_EN_LDWIN_MSK			BIT(5)
#define ADF4377_EN_LDWIN(x)             FIELD_PREP(ADF4377_EN_LDWIN_MSK, x)
#define ADF4377_LD_COUNT_MSK			GENMASK(4, 0)
#define ADF4377_LD_COUNT(x)             FIELD_PREP(ADF4377_LD_COUNT_MSK, x)

/* ADF4377 REG001B Bit Definition */
#define ADF4377_EN_LOL_DIS              0x0
#define ADF4377_EN_LOL_EN               0x1

#define ADF4377_LDWIN_PW_NARROW         0x0
#define ADF4377_LDWIN_PW_WIDE           0x1

#define ADF4377_EN_LDWIN_DIS            0x0
#define ADF4377_EN_LDWIN_EN             0x1

/* ADF4377 REG001C Map */
#define ADF4377_EN_DNCLK_MSK			BIT(7)
#define ADF4377_EN_DNCLK(x)            	FIELD_PREP(ADF4377_EN_DNCLK_MSK, x)
#define ADF4377_EN_DRCLK_MSK			BIT(6)
#define ADF4377_EN_DRCLK(x)             FIELD_PREP(ADF4377_EN_DRCLK_MSK, x)
#define ADF4377_R01C_RSV4               0x0 << 5
#define ADF4377_R01C_RSV3               0x0 << 4
#define ADF4377_R01C_RSV2               0x0 << 3
#define ADF4377_RST_LD_MSK				BIT(2)
#define ADF4377_RST_LD(x)               FIELD_PREP(ADF4377_RST_LD_MSK, x)
#define ADF4377_R01C_RSV1_MSK			BIT(0)
#define ADF4377_R01C_RSV1(x)			FIELD_PREP(ADF4377_R01C_RSV1_MSK, x)

/* ADF4377 REG001C Bit Definition */
#define ADF4377_EN_DNCLK_OFF            0x0
#define ADF4377_EN_DNCLK_ON             0x1

#define ADF4377_EN_DRCLK_OFF            0x0
#define ADF4377_EN_DRCLK_ON             0x1

#define ADF4377_RST_LD_INACTIVE         0x0
#define ADF4377_RST_LD_ACTIVE           0x1

/* ADF4377 REG001D Map */
#define ADF4377_MUXOUT_MSK				GENMASK(7, 4)
#define ADF4377_MUXOUT(x)               FIELD_PREP(ADF4377_MUXOUT_MSK, x)
#define ADF4377_R01D_RSV1               (0x0 << 3)
#define ADF4377_EN_CPTEST_MSK			BIT(2)
#define ADF4377_EN_CPTEST(x)            FIELD_PREP(ADF4377_EN_CPTEST_MSK, x)
#define ADF4377_CP_DOWN_MSK				BIT(1)
#define ADF4377_CP_DOWN(x)              FIELD_PREP(ADF4377_CP_DOWN_MSK, x)
#define ADF4377_CP_UP_MSK				BIT(0)
#define ADF4377_CP_UP(x)                FIELD_PREP(ADF4377_CP_UP_MSK, x)

/* ADF4377 REG001D Bit Definitons */
#define ADF4377_MUXOUT_HIGH_Z           0x0
#define ADF4377_MUXOUT_LKDET            0x1
#define ADF4377_MUXOUT_LOW              0x2
#define ADF4377_MUXOUT_DIV_RCLK_2       0x4
#define ADF4377_MUXOUT_DIV_NCLK_2       0x5
#define ADF4377_MUXOUT_HIGH             0x8
#define ADF4377_MUXOUT_REF_OK           0xE

#define ADF4377_EN_CPTEST_OFF           0x0
#define ADF4377_EN_CPTEST_ON            0x1

#define ADF4377_CP_DOWN_OFF             0x0
#define ADF4377_CP_DOWN_ON              0x1

#define ADF4377_CP_UP_OFF               0x0
#define ADF4377_CP_UP_ON                0x1

/* ADF4377 REG001F Map */
#define ADF4377_BST_REF_MSK				BIT(7)
#define ADF4377_BST_REF(x)              FIELD_PREP(ADF4377_BST_REF_MSK, x)
#define ADF4377_FILT_REF_MSK			BIT(6)
#define ADF4377_FILT_REF(x)             FIELD_PREP(ADF4377_FILT_REF_MSK, x)
#define ADF4377_REF_SEL_MSK				BIT(5)
#define ADF4377_REF_SEL(x)              FIELD_PREP(ADF4377_REF_SEL_MSK, x)
#define ADF4377_R01F_RSV1_MSK           GENMASK(2, 0)
#define ADF4377_R01F_RSV1(x) 			FIELD_PREP(ADF4377_R01F_RSV1_MSK, x)

/* ADF4377 REG001F Bit Description */
#define ADF4377_BST_LARGE_REF_IN        0x0
#define ADF4377_BST_SMALL_REF_IN        0x1

#define ADF4377_FILT_REF_OFF            0x0
#define ADF4377_FILT_REF_ON             0x1

#define ADF4377_REF_SEL_DMA             0x0
#define ADF4377_REF_SEL_LNA             0x1

/* ADF4377 REG0020 Map */
#define ADF4377_R020_RSV5               (0x0 << 6)
#define ADF4377_R020_RSV4               (0x0 << 5)
#define ADF4377_RST_SYS_MSK				BIT(4)
#define ADF4377_RST_SYS(x)              FIELD_PREP(ADF4377_RST_SYS_MSK, x)
#define ADF4377_EN_ADC_CLK_MSK			BIT(3)
#define ADF4377_EN_ADC_CLK(x)           FIELD_PREP(ADF4377_EN_ADC_CLK_MSK, x)
#define ADF4377_R020_RSV1_MSK           BIT(0)
#define ADF4377_R020_RSV1(x)			FIELD_PREP(ADF4377_R020_RSV1_MSK, x)

/* ADF4377 REG0020 Bit Description */
#define ADF4377_RST_SYS_INACTIVE        0x0
#define ADF4377_RST_SYS_ACTIVE          0x1

#define ADF4377_EN_ADC_CLK_DIS          0x0
#define ADF4377_EN_ADC_CLK_EN           0x1

/* ADF4377 REG0021 Map */
#define ADF4377_R021_RSV1               0xD3

/* ADF4377 REG0022 Map */
#define ADF4377_R022_RSV1               0x32

/* ADF4377 REG0023 Map */
#define ADF4377_R023_RSV1               0x18

/* ADF4377 REG0024 Map */
#define ADF4377_R024_RSV4               (0x0 << 7)
#define ADF4377_R024_RSV3               (0x0 << 3)
#define ADF4377_DCLK_MODE_MSK			BIT(2)
#define ADF4377_DCLK_MODE(x)            FIELD_PREP(ADF4377_DCLK_MODE_MSK, x)
#define ADF4377_R024_RSV2               (0x0 << 1)
#define ADF4377_R024_RSV1               (0x0 << 0)

/* ADF4377 REG0024 Bit Definition */
#define ADF4377_DCLK_MODE_DIS			0x0
#define ADF4377_DCLK_MODE_EN			0x1

/* ADF4377 REG0025 Map */
#define ADF4377_CLKODIV_DB_MSK			BIT(7)
#define ADF4377_CLKODIV_DB(x)           FIELD_PREP(ADF4377_CLKODIV_DB_MSK, x)
#define ADF4377_DCLK_DB_MSK				BIT(6)
#define ADF4377_DCLK_DB(x)              FIELD_PREP(ADF4377_DCLK_DB_MSK, x)
#define ADF4377_R025_RSV1_MSK           BIT(4) | BIT(2) | BIT(1)
#define ADF4377_R025_RSV1(x)			FIELD_PREP(ADF4377_R025_RSV1_MSK, x)

/* ADF4377 REG0025 Bit Definition */
#define ADF4377_CLKODIV_DB_DIS          0x0
#define ADF4377_CLKODIV_DB_EN           0x1

#define ADF4377_DCLK_DIV_DB_DIS         0x0
#define ADF4377_DCLK_DIV_DB_EN          0x1

/* ADF4377 REG0026 Map */
#define ADF4377_VCO_BAND_DIV_MSK    	GENMASK(7, 0)
#define ADF4377_VCO_BAND_DIV(x)         FIELD_PREP(ADF4377_VCO_BAND_DIV_MSK, x)

/* ADF4377 REG0026 Bit Definition */
#define ADF4377_VCO_BAND_DIV_MIN        0x00
#define ADF4377_VCO_BAND_DIV_MAX        0xFF

/* ADF4377 REG0027 Map */
#define ADF4377_SYNTH_LOCK_TO_LSB_MSK   GENMASK(7, 0)
#define ADF4377_SYNTH_LOCK_TO_LSB(x)   	FIELD_PREP(ADF4377_SYNTH_LOCK_TO_LSB_MSK, x)

/* ADF4377 REG0028 Map */
#define ADF4377_O_VCO_DB_MSK			BIT(7)
#define ADF4377_O_VCO_DB(x)             FIELD_PREP(ADF4377_O_VCO_DB_MSK, x)
#define ADF4377_SYNTH_LOCK_TO_MSB_MSK	GENMASK(6, 0)
#define ADF4377_SYNTH_LOCK_TO_MSB(x)   	FIELD_PREP(ADF4377_SYNTH_LOCK_TO_MSB_MSK, x)

/* ADF4377 REG0028 Bit Definition */
#define ADF4377_O_VCO_DB_DIS            0x0
#define ADF4377_O_VCO_DB_EN             0x1

/* ADF4377 REG0029 Map */
#define ADF4377_VCO_ALC_TO_LSB_MSK		GENMASK(7, 0)
#define ADF4377_VCO_ALC_TO_LSB(x)     	FIELD_PREP(ADF4377_VCO_ALC_TO_LSB_MSK, x)

/* ADF4377 REG002A Map */
#define ADF4377_DEL_CTRL_DB_MSK			BIT(7)
#define ADF4377_DEL_CTRL_DB(x)          FIELD_PREP(ADF4377_DEL_CTRL_DB_MSK, x)
#define ADF4377_VCO_ALC_TO_MSB_MSK		GENMASK(6, 0)
#define ADF4377_VCO_ALC_TO_MSB(x)      	FIELD_PREP(ADF4377_VCO_ALC_TO_MSB_MSK, x)

/* ADF4377 REG002A Bit Definition */
#define ADF4377_DEL_CTRL_DB_DIS         0x0
#define ADF4377_DEL_CTRL_DB_EN          0x1

/* ADF4377 REG002C Map */
#define ADF4377_R02C_RSV1               0xC0

/* ADF4377 REG002D Map */
#define ADF4377_ADC_CLK_DIV_MSK			GENMASK(7, 0)
#define ADF4377_ADC_CLK_DIV(x)          FIELD_PREP(ADF4377_ADC_CLK_DIV_MSK, x)

/* ADF4377 REG002E Map */
#define ADF4377_EN_ADC_CNV_MSK			BIT(7)
#define ADF4377_EN_ADC_CNV(x)           FIELD_PREP(ADF4377_EN_ADC_CNV_MSK, x)
#define ADF4377_R02E_RSV5               (0x0 << 6)
#define ADF4377_R02E_RSV4               (0x0 << 5)
#define ADF4377_R02E_RSV3               (0x0 << 4)
#define ADF4377_R02E_RSV2               (0x0 << 3)
#define ADF4377_R02E_RSV1               (0x0 << 2)
#define ADF4377_EN_ADC_MSK				BIT(1)
#define ADF4377_EN_ADC(x)               FIELD_PREP(ADF4377_EN_ADC_MSK, x)
#define ADF4377_ADC_A_CONV_MSK			BIT(0)
#define ADF4377_ADC_A_CONV(x)           FIELD_PREP(ADF4377_ADC_A_CONV_MSK, x)

/* ADF4377 REG002E Bit Definition */
#define ADF4377_EN_ADC_CNV_DIS          0x0
#define ADF4377_EN_ADC_CNV_EN           0x1

#define ADF4377_EN_ADC_DIS              0x0
#define ADF4377_EN_ADC_EN               0x1

#define ADF4377_ADC_A_CONV_ADC_ST_CNV   0x0
#define ADF4377_ADC_A_CONV_VCO_CALIB    0x1

/* ADF4377 REG002F Map */
#define ADF4377_R02F_RSV5               (0x0 << 7)
#define ADF4377_R02F_RSV4               (0x0 << 6)
#define ADF4377_R02F_RSV3               (0x0 << 5)
#define ADF4377_R02F_RSV2               (0x0 << 4)
#define ADF4377_R02F_RSV1               (0x0 << 3)
#define ADF4377_DCLK_DIV1_MSK			GENMASK(1, 0)
#define ADF4377_DCLK_DIV1(x)            FIELD_PREP(ADF4377_DCLK_DIV1_MSK, x)

/* ADF4377 REG002F Bit Definition */
#define ADF4377_DCLK_DIV1_1             0x0
#define ADF4377_DCLK_DIV1_2             0x1
#define ADF4377_DCLK_DIV1_8             0x2
#define ADF4377_DCLK_DIV1_32            0x3

/* ADF4377 REG0031 Map */
#define ADF4377_R031_RSV1				0x09

/* ADF4377 REG0032 Map */
#define ADF4377_R032_RSV5               (0x0 << 7)
#define ADF4377_ADC_CLK_SEL_MSK			BIT(6)
#define ADF4377_ADC_CLK_SEL(x)          FIELD_PREP(ADF4377_ADC_CLK_SEL_MSK, x)
#define ADF4377_R032_RSV1_MSK           BIT(3) | BIT(0)
#define ADF4377_R032_RSV1(x)			FIELD_PREP(ADF4377_R032_RSV1_MSK, x)

/* ADF4377 REG0032 Bit Definition */
#define ADF4377_ADC_CLK_SEL_N_OP        0x0
#define ADF4377_ADC_CLK_SEL_SPI_CLK     0x1

/* ADF4377 REG0033 Map */
#define ADF4377_R033_RSV1               0x18

/* ADF4377 REG0034 Map */
#define ADF4377_R034_RSV1               0x08

/* ADF4377 REG003A Map */
#define ADF4377_R03A_RSV1               0x5C

/* ADF4377 REG003B Map */
#define ADF4377_R03B_RSV1               0x2B

/* ADF4377 REG003D Map */
#define ADF4377_R03D_RSV2               (0x0 << 4)
#define ADF4377_O_VCO_BAND_MSK			BIT(3)
#define ADF4377_O_VCO_BAND(x)           FIELD_PREP(ADF4377_O_VCO_BAND_MSK, x)
#define ADF4377_O_VCO_CORE_MSK			BIT(2)
#define ADF4377_O_VCO_CORE(x)           FIELD_PREP(ADF4377_O_VCO_CORE_MSK, x)
#define ADF4377_O_VCO_BIAS_MSK			BIT(1)
#define ADF4377_O_VCO_BIAS(x)           FIELD_PREP(ADF4377_O_VCO_BIAS_MSK, x)
#define ADF4377_R03D_RSV1               (0x0 << 0)

/* ADF4377 REG003D Bit Definition */
#define ADF4377_O_VCO_BAND_VCO_CALIB    0x0
#define ADF4377_O_VCO_BAND_M_VCO        0x1

#define ADF4377_O_VCO_CORE_VCO_CALIB    0x0
#define ADF4377_O_VCO_CORE_M_VCO        0x1

#define ADF4377_O_VCO_BIAS_VCO_CALIB    0x0
#define ADF4377_O_VCO_BIAS_M_VCO        0x1

/* ADF4377 REG0042 Map */
#define ADF4377_R042_RSV1               0x05

/* ADF4377 REG0045 Map */
#define ADF4377_R045_RESERVED           (0x0 << 1)
#define ADF4377_ADC_ST_CNV_MSK			BIT(0)
#define ADF4377_ADC_ST_CNV(x)           FIELD_PREP(ADF4377_ADC_ST_CNV_MSK, x)

/* ADF4377 REG0045 Bit Definition */
#define ADF4377_ADC_ST_ADC_DIS          0x0
#define ADF4377_ADC_ST_ADC_EN           0x1

/* ADF4377 REG0049 Map */
#define ADF4377_EN_CLK2_MSK				BIT(7)
#define ADF4377_EN_CLK2(x)              FIELD_PREP(ADF4377_EN_CLK2_MSK, x)
#define ADF4377_EN_CLK1_MSK				BIT(6)
#define ADF4377_EN_CLK1(x)              FIELD_PREP(ADF4377_EN_CLK1_MSK, x)
#define ADF4377_R049_RSV2               (0x0 << 5)
#define ADF4377_R049_RSV1               (0x0 << 4)
#define ADF4377_REF_OK_MSK				BIT(3)
#define ADF4377_REF_OK(x)               FIELD_PREP(ADF4377_REF_OK_MSK, x)
#define ADF4377_ADC_BUSY_MSK			BIT(2)
#define ADF4377_ADC_BUSY(x)             FIELD_PREP(ADF4377_ADC_BUSY_MSK, x)
#define ADF4377_FSM_BUSY_MSK			BIT(1)
#define ADF4377_FSM_BUSY(x)             FIELD_PREP(ADF4377_FSM_BUSY_MSK, x)
#define ADF4377_LOCKED_MSK				BIT(0)
#define ADF4377_LOCKED(x)               FIELD_PREP(ADF4377_LOCKED_MSK, x)

/* ADF4377 REG004B Map */
#define ADF4377_R04B_RESERVED           (0x0 << 2)
#define ADF4377_VCO_CORE_MSK			GENMASK(1, 0)
#define ADF4377_VCO_CORE(x)             FIELD_PREP(ADF4377_VCO_CORE_MSK, x)

/* ADF4377 REG004C Map */
#define ADF4377_CHIP_TEMP_LSB_MSK		GENMASK(7, 0)
#define ADF4377_CHIP_TEMP_LSB(x)        FIELD_PREP(ADF4377_CHIP_TEMP_LSB_MSK, x)

/* ADF4377 REG004D Map */
#define ADF4377_R04D_RESERVED           (0x0 << 1)
#define ADF4377_CHIP_TEMP_MSB_MSK		BIT(0)
#define ADF4377_CHIP_TEMP_MSB(x)        FIELD_PREP(ADF4377_CHIP_TEMP_MSB_MSK, x)

/* ADF4377 REG004F Map */
#define ADF4377_VCO_BAND_MSK			GENMASK(7, 0)
#define ADF4377_VCO_BAND(x)             FIELD_PREP(ADF4377_VCO_BAND_MSK, x)

/* ADF4377 REG0054 Map */
#define ADF4377_CHIP_VERSION_MSK		GENMASK(7, 0)
#define ADF4377_CHIP_VERSION(x)         FIELD_PREP(ADF4377_CHIP_VERSION_MSK, x)

/* Specifications */
#define ADF4377_SPI_WRITE_CMD		    0x0
#define ADF4377_SPI_READ_CMD		    BIT(7)
#define ADF4377_BUFF_SIZE_BYTES		    3
#define ADF4377_MAX_VCO_FREQ		    12800000000ull /* Hz */
#define ADF4377_MIN_VCO_FREQ		    6400000000ull /* Hz */
#define ADF4377_MAX_REFIN_FREQ		    1000000000 /* Hz */
#define ADF4377_MIN_REFIN_FREQ		    10000000 /* Hz */
#define ADF4377_MAX_FREQ_PFD		    500000000 /* Hz */
#define ADF4377_MIN_FREQ_PFD		    3000000 /* Hz */
#define ADF4377_MAX_CLKPN_FREQ		    ADF4377_MAX_VCO_FREQ /* Hz */
#define ADF4377_MIN_CLKPN_FREQ		    (ADF4377_MIN_VCO_FREQ / 8) /* Hz */
#define ADF4377_FREQ_PFD_80MHZ		    80000000
#define ADF4377_FREQ_PFD_125MHZ		    125000000
#define ADF4377_FREQ_PFD_160MHZ		    160000000
#define ADF4377_FREQ_PFD_250MHZ		    250000000
#define ADF4377_FREQ_PFD_320MHZ		    320000000

/* ADF4377 Extra Definitions */
#define ADF4377_SPI_SCRATCHPAD		    0xA5
#define ADF4377_SPI_DUMMY_DATA		    0x00
#define ADF4377_CHECK_RANGE(freq, range) \
	((freq > ADF4377_MAX_ ## range) || (freq < ADF4377_MIN_ ## range))

enum supported_parts {
	ADF4377,
	ADF4378
};

struct adf4377 {
	struct spi_device *spi;
	struct regmap *regmap;


	/* Charge Pump Current */
	uint8_t cp_i;
	/* Reference doubler enable */
	uint8_t	ref_doubler_en;
	/* Reference Divider */
	uint32_t ref_div_factor;
	/* PFD Frequency */
	uint32_t f_pfd;
	/* Input Reference Clock */
	uint32_t clkin_freq;
	/* Output Amplitude */
	uint8_t	clkout_op;
	/* CLKOUT Divider */
	uint8_t clkout_div_sel;
	/* Feedback Divider (N) */
	uint16_t n_int;
	/* Output frequency */
	uint64_t f_clk;
	/* Output frequency of the VCO */
	uint64_t f_vco;

	struct mutex	lock;
};

static const struct regmap_config adf4377_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = 0x54,
};

static int adf4377_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int tx_val,
	unsigned int *rx_val)
{
	struct adf4377 *dev = iio_priv(indio_dev);

#if 1
#if 0
	uint32_t tmp;
regmap_read(dev->regmap, 0xc, &tmp);
		pr_err("%s: 0xc = 0x%x\n", __FUNCTION__, tmp);
#endif
mutex_lock(&dev->lock);
	if (rx_val)
		regmap_read(dev->regmap, reg, rx_val);
	else
		regmap_write(dev->regmap, reg, tx_val);
mutex_unlock(&dev->lock);
#else
	uint8_t wbuf[3];
	uint8_t rbuf[3];

	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = 3,
	};

	wbuf[0] = (reg >> 8) | (rx_val ? BIT(7) : 0);
	wbuf[1] = (reg & 0xff);
	wbuf[2] = 0x00;

	spi_sync_transfer(dev->spi, &t, 1);

	*rx_val = rbuf[2];
#endif


	return 0;
}

static const struct iio_info adf4377_info = {
	.debugfs_reg_access = &adf4377_reg_access,
};

static int adf4377_set_default(struct iio_dev *indio_dev)
{
	struct adf4377 *dev = iio_priv(indio_dev);

//regmap_write(dev->regmap, 0x00, 0x81);
//mdelay(500);

//regmap_write(dev->regmap, 0x00, 0x00);
//mdelay(500);


#if 1
	uint32_t tmp;
regmap_read(dev->regmap, 0xc, &tmp);
		pr_err("%s: 0xc = 0x%x\n", __FUNCTION__, tmp);
#endif
	regmap_write(dev->regmap, 0x0f, ADF4377_R00F_RSV1);
	regmap_update_bits(dev->regmap, 0x1c, ADF4377_R01C_RSV1_MSK, ADF4377_R01C_RSV1(0x1));
	regmap_update_bits(dev->regmap, 0x1f, ADF4377_R01F_RSV1_MSK, ADF4377_R01F_RSV1(0x7));
	regmap_update_bits(dev->regmap, 0x20, ADF4377_R020_RSV1_MSK, ADF4377_R020_RSV1(0x1));
	regmap_write(dev->regmap, 0x21, ADF4377_R021_RSV1);
	regmap_write(dev->regmap, 0x22, ADF4377_R022_RSV1);
	regmap_write(dev->regmap, 0x23, ADF4377_R023_RSV1);
	//regmap_update_bits(dev->regmap, 0x25, ADF4377_R025_RSV1_MSK, ADF4377_R025_RSV1(0xB));
regmap_update_bits(dev->regmap, 0x25, ADF4377_R025_RSV1_MSK, 0x16);
	regmap_write(dev->regmap, 0x2C, ADF4377_R02C_RSV1);
	regmap_write(dev->regmap, 0x31, ADF4377_R031_RSV1);
	//regmap_update_bits(dev->regmap, 0x32, ADF4377_R032_RSV1_MSK, ADF4377_R032_RSV1(0x9));
regmap_update_bits(dev->regmap, 0x32, ADF4377_R032_RSV1_MSK, 0x9);
	regmap_write(dev->regmap, 0x33, ADF4377_R033_RSV1);
	regmap_write(dev->regmap, 0x34, ADF4377_R034_RSV1);
	regmap_write(dev->regmap, 0x3A, ADF4377_R03A_RSV1);
	regmap_write(dev->regmap, 0x3B, ADF4377_R03B_RSV1);
	regmap_write(dev->regmap, 0x42, ADF4377_R042_RSV1);

	return 0;
}

int32_t adf4377_set_freq(struct iio_dev *indio_dev)
{
	struct adf4377 *dev = iio_priv(indio_dev);

	dev->clkout_div_sel = 0;

	if(ADF4377_CHECK_RANGE(dev->f_clk, CLKPN_FREQ))
		return -1;

	dev->f_vco = dev->f_clk;

	while (dev->f_vco < ADF4377_MIN_VCO_FREQ) {
		dev->f_vco <<= 1;
		dev->clkout_div_sel++;
	}

	dev->n_int = dev->f_clk / dev->f_pfd;

	regmap_update_bits(dev->regmap, 0x11,
			     ADF4377_EN_RDBLR_MSK | ADF4377_N_INT_MSB_MSK,
			     ADF4377_EN_RDBLR(dev->ref_doubler_en) | ADF4377_N_INT_MSB(dev->n_int >> 8));

	regmap_update_bits(dev->regmap, 0x12,
			     ADF4377_R_DIV_MSK | ADF4377_CLKOUT_DIV_MSK,
			     ADF4377_CLKOUT_DIV(dev->clkout_div_sel) | ADF4377_R_DIV(dev->ref_div_factor));

	regmap_write(dev->regmap, 0x10, ADF4377_N_INT_LSB(dev->n_int));

	mdelay(100);

	return 0;

}

static int adf4377_setup(struct iio_dev *indio_dev)
{
	struct adf4377 *dev = iio_priv(indio_dev);
	uint32_t f_div_rclk;
	uint8_t dclk_div1, dclk_div2, dclk_mode;
	uint16_t synth_lock_timeout, vco_alc_timeout, adc_clk_div, vco_band_div;

#if 0
	uint32_t tmp;
regmap_read(dev->regmap, 0xc, &tmp);
		pr_err("%s: 0xc = 0x%x\n", __FUNCTION__, tmp);
#endif
	/* Set Default Registers */
	adf4377_set_default(indio_dev);

	/* Update Charge Pump Current Value */
	regmap_update_bits(dev->regmap, 0x15, ADF4377_CP_I_MSK, ADF4377_CP_I(dev->cp_i));

	/*Compute PFD */
	if (!(dev->ref_doubler_en))
		do {
			dev->ref_div_factor++;
			dev->f_pfd = dev->clkin_freq / dev->ref_div_factor;
		} while (dev->f_pfd > ADF4377_MAX_FREQ_PFD);
	else
		dev->f_pfd = dev->clkin_freq * (1 + dev->ref_doubler_en);

	if(ADF4377_CHECK_RANGE(dev->f_pfd, FREQ_PFD))
		return -1;

	f_div_rclk = dev->f_pfd;

	if (dev->f_pfd <= ADF4377_FREQ_PFD_80MHZ) {
		dclk_div1 = ADF4377_DCLK_DIV1_1;
		dclk_div2 = ADF4377_DCLK_DIV2_1;
		dclk_mode = ADF4377_DCLK_MODE_DIS;
	} else if (dev->f_pfd <= ADF4377_FREQ_PFD_125MHZ) {
		dclk_div1 = ADF4377_DCLK_DIV1_1;
		dclk_div2 = ADF4377_DCLK_DIV2_1;
		dclk_mode = ADF4377_DCLK_MODE_EN;
	} else if (dev->f_pfd <= ADF4377_FREQ_PFD_160MHZ) {
		dclk_div1 = ADF4377_DCLK_DIV1_2;
		dclk_div2 = ADF4377_DCLK_DIV2_1;
		dclk_mode = ADF4377_DCLK_MODE_DIS;
		f_div_rclk /= 2;
	} else if (dev->f_pfd <= ADF4377_FREQ_PFD_250MHZ) {
		dclk_div1 = ADF4377_DCLK_DIV1_2;
		dclk_div2 = ADF4377_DCLK_DIV2_1;
		dclk_mode = ADF4377_DCLK_MODE_EN;
		f_div_rclk /= 2;
	} else if (dev->f_pfd <= ADF4377_FREQ_PFD_320MHZ) {
		dclk_div1 = ADF4377_DCLK_DIV1_2;
		dclk_div2 = ADF4377_DCLK_DIV2_2;
		dclk_mode = ADF4377_DCLK_MODE_DIS;
		f_div_rclk /= 4;
	} else {
		dclk_div1 = ADF4377_DCLK_DIV1_2;
		dclk_div2 = ADF4377_DCLK_DIV2_2;
		dclk_mode = ADF4377_DCLK_MODE_EN;
		f_div_rclk /= 4;
	}

	synth_lock_timeout = DIV_ROUND_UP(f_div_rclk, 50000);
	vco_alc_timeout = DIV_ROUND_UP(f_div_rclk, 20000);
	vco_band_div = DIV_ROUND_UP(f_div_rclk, 150000 * 16 * (1 << dclk_mode));
	adc_clk_div = DIV_ROUND_UP((f_div_rclk / 400000 - 2), 4);

	regmap_update_bits(dev->regmap, 0x1C,
			     ADF4377_EN_DNCLK_MSK | ADF4377_EN_DRCLK_MSK,
			     ADF4377_EN_DNCLK(ADF4377_EN_DNCLK_ON) | ADF4377_EN_DRCLK(
				     ADF4377_EN_DRCLK_ON));

	regmap_update_bits(dev->regmap, 0x11,
			     ADF4377_EN_AUTOCAL_MSK | ADF4377_DCLK_DIV2_MSK,
			     ADF4377_EN_AUTOCAL(ADF4377_VCO_CALIB_EN) | ADF4377_DCLK_DIV2(dclk_div2));

	regmap_update_bits(dev->regmap, 0x2E,
			     ADF4377_EN_ADC_CNV_MSK | ADF4377_EN_ADC_MSK | ADF4377_ADC_A_CONV_MSK,
			     ADF4377_EN_ADC_CNV(ADF4377_EN_ADC_CNV_EN) | ADF4377_EN_ADC(
				     ADF4377_EN_ADC_EN) | ADF4377_ADC_A_CONV(ADF4377_ADC_A_CONV_VCO_CALIB));

	regmap_update_bits(dev->regmap, 0x20, ADF4377_EN_ADC_CLK_MSK,
			     ADF4377_EN_ADC_CLK(ADF4377_EN_ADC_CLK_EN));

	regmap_update_bits(dev->regmap, 0x2F, ADF4377_DCLK_DIV1_MSK,
			     ADF4377_DCLK_DIV1(dclk_div1));

	regmap_update_bits(dev->regmap, 0x24, ADF4377_DCLK_MODE_MSK,
			     ADF4377_DCLK_MODE(dclk_mode));

	regmap_write(dev->regmap, 0x27,
				ADF4377_SYNTH_LOCK_TO_LSB(synth_lock_timeout));

	regmap_update_bits(dev->regmap, 0x28, ADF4377_SYNTH_LOCK_TO_MSB_MSK,
			     ADF4377_SYNTH_LOCK_TO_MSB(synth_lock_timeout >> 8));

	regmap_write(dev->regmap, 0x29,
				ADF4377_VCO_ALC_TO_LSB(vco_alc_timeout));

	regmap_update_bits(dev->regmap, 0x2A, ADF4377_VCO_ALC_TO_MSB_MSK,
			     ADF4377_VCO_ALC_TO_MSB(vco_alc_timeout >> 8));

	regmap_write(dev->regmap, 0x26,
				ADF4377_VCO_BAND_DIV(vco_band_div));

	regmap_write(dev->regmap, 0x2D,
				ADF4377_ADC_CLK_DIV(adc_clk_div));

	/* Power Up */
	regmap_write(dev->regmap, 0x1a,
				ADF4377_PD_ALL(ADF4377_PD_ALL_N_OP) |
				ADF4377_PD_RDIV(ADF4377_PD_RDIV_N_OP) | ADF4377_PD_NDIV(ADF4377_PD_NDIV_N_OP) |
				ADF4377_PD_VCO(ADF4377_PD_VCO_N_OP) | ADF4377_PD_LD(ADF4377_PD_LD_N_OP) |
				ADF4377_PD_PFDCP(ADF4377_PD_PFDCP_N_OP) | ADF4377_PD_CLKOUT1(
					ADF4377_PD_CLKOUT1_N_OP) |
				ADF4377_PD_CLKOUT2(ADF4377_PD_CLKOUT2_N_OP));

	adf4377_set_freq(indio_dev);

	/* Disable EN_DNCLK, EN_DRCLK */
	regmap_update_bits(dev->regmap, 0x1C,
			     ADF4377_EN_DNCLK_MSK | ADF4377_EN_DRCLK_MSK,
			     ADF4377_EN_DNCLK(ADF4377_EN_DNCLK_OFF) | ADF4377_EN_DRCLK(
				     ADF4377_EN_DRCLK_OFF));

	/* Disable EN_ADC_CLK */
	regmap_update_bits(dev->regmap, 0x20, ADF4377_EN_ADC_CLK_MSK,
			     ADF4377_EN_ADC_CLK(ADF4377_EN_ADC_CLK_DIS));

	/* Set output Amplitude */
	regmap_update_bits(dev->regmap, 0x19,
			     ADF4377_CLKOUT2_OP_MSK | ADF4377_CLKOUT1_OP_MSK,
			     ADF4377_CLKOUT1_OP(dev->clkout_op) | ADF4377_CLKOUT2_OP(dev->clkout_op));

	return 0;
}

static int adf4377_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adf4377 *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;
	dev = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);



	regmap = devm_regmap_init_spi(spi, &adf4377_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	

	
	dev->regmap = regmap;
	dev->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &adf4377_info;
	indio_dev->name = spi->dev.of_node->name;

		dev->clkin_freq = 500000000;
		dev->cp_i = ADF4377_CP_10MA1;
		//dev->muxout_select = ADF4377_MUXOUT_HIGH_Z;
		dev->ref_doubler_en = ADF4377_REF_DBLR_DIS;
		dev->f_clk = 10000000000;
		dev->clkout_op = ADF4377_CLKOUT_427MV;

mutex_init(&dev->lock);

adf4377_setup(indio_dev);

	ret = iio_device_register(indio_dev);
regmap_read(dev->regmap, 0xc, &ret);


adf4377_reg_access(indio_dev, 0xc, 0, &ret);

	dev_info(&spi->dev, "%s Probed 0x%x\n", indio_dev->name, ret);

	return 0;
}

static int adf4377_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id adf4377_id[] = {
	{ "adf4377", ADF4377 },
	{}
};
MODULE_DEVICE_TABLE(spi, adf4377_id);

static const struct of_device_id adf4377_of_match[] = {
	{ .compatible = "adi,adf4377" },
	{},
};
MODULE_DEVICE_TABLE(of, adf4377_of_match);

static struct spi_driver adf4377_driver = {
	.driver = {
			.name = "adf4377",
			.of_match_table = of_match_ptr(adf4377_of_match),
		},
	.probe = adf4377_probe,
	.remove = adf4377_remove,
	.id_table = adf4377_id,
};
module_spi_driver(adf4377_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4377");
MODULE_LICENSE("GPL v2");
