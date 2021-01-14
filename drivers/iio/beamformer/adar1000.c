// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR1000 4-Channel, X Band and Ku Band Beamformer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADAR1000_INTERFACE_CFG_A	0x00
#define ADAR1000_INTERFACE_CFG_B	0x01
#define ADAR1000_DEV_CFG		0x02
#define ADAR1000_CHIP_TYPE		0x03
#define ADAR1000_PRODUCT_ID_H		0x04
#define ADAR1000_PRODUCT_ID_L		0x05
#define ADAR1000_SCRATCH_PAD		0x0A
#define ADAR1000_SPI_REV		0x0B
#define ADAR1000_VENDOR_ID_H		0x0C
#define ADAR1000_VENDOR_ID_L		0x0D
#define ADAR1000_TRANSFER_REG		0x0F
#define ADAR1000_CH_RX_GAIN(x)		(0x10 + (x))
#define ADAR1000_CH_RX_PHASE_I(x)	(0x14 + 2 * (x))
#define ADAR1000_CH_RX_PHASE_Q(x)	(0x15 + 2 * (x))
#define ADAR1000_CH_TX_GAIN(x)		(0x1C + (x))
#define ADAR1000_CH_TX_PHASE_I(x)	(0x20 + 2 * (x))
#define ADAR1000_CH_TX_PHASE_Q(x)	(0x21 + 2 * (x))
#define ADAR1000_LD_WRK_REGS		0x28
#define ADAR1000_CH_PA_BIAS_ON(x)	(0x29 + (x))
#define ADAR1000_LNA_BIAS_ON		0x2D
#define ADAR1000_RX_ENABLES		0x2E
#define ADAR1000_TX_ENABLES		0x2F
#define ADAR1000_MISC_ENABLES		0x30
#define ADAR1000_SW_CTRL		0x31
#define ADAR1000_ADC_CTRL		0x32
#define ADAR1000_ADC_OUTPUT		0x33
#define ADAR1000_BIAS_CURRENT_RX_LNA	0x34
#define ADAR1000_BIAS_CURRENT_RX	0x35
#define ADAR1000_BIAS_CURRENT_TX	0x36
#define ADAR1000_BIAS_CURRENT_TX_DRV	0x37
#define ADAR1000_MEM_CTRL		0x38
#define ADAR1000_RX_CHX_MEM		0x39
#define ADAR1000_TX_CHX_MEM		0x3A
#define ADAR1000_RX_CH_MEM(x)		(0x3D + (x))
#define ADAR1000_TX_CH_MEM(x)		(0x41 + (x))
#define ADAR1000_REV_ID			0x45
#define ADAR1000_CH_PA_BIAS_OFF(x)	(0x46 + (x))
#define ADAR1000_LNA_BIAS_OFF		0x4A
#define ADAR1000_TX_TO_RX_DELAY_CTRL	0x4B
#define ADAR1000_RX_TO_TX_DELAY_CTRL	0x4C
#define ADAR1000_TX_BEAM_STEP_START	0x4D
#define ADAR1000_TX_BEAM_STEP_STOP	0x4E
#define ADAR1000_RX_BEAM_STEP_START	0x4F
#define ADAR1000_RX_BEAM_STEP_STOP	0x50
#define ADAR1000_RX_BIAS_RAM_CTL	0x51
#define ADAR1000_TX_BIAS_RAM_CTL	0x52
#define ADAR1000_LDO_TRIM_CTL_0		0x400
#define ADAR1000_LDO_TRIM_CTL_1		0x401

/* ADAR1000_INTERFACE_CFG_A */
#define ADAR1000_SOFTRESET		BIT(7)
#define ADAR1000_LSB_FIRST		BIT(6)
#define ADAR1000_ADDR_ASCN		BIT(5)
#define ADAR1000_SDOACTIVE		BIT(4)
#define ADAR1000_SDOACTIVE_		BIT(3)
#define ADAR1000_ADDR_ASCN_		BIT(2)
#define ADAR1000_LSB_FIRST_		BIT(1)
#define ADAR1000_SOFTRESET_		BIT(0)

/* ADAR1000_ADC_CTRL */
#define ADAR1000_ADC_CLKFREQ_SEL	BIT(7)
#define ADAR1000_AC_EN			BIT(6)
#define ADAR1000_CLK_EN			BIT(5)
#define ADAR1000_ST_CONV		BIT(4)
#define ADAR1000_MUX_SEL_MSK		GENMASK(3, 1)
#define ADAR1000_MUX_SEL(x)		FIELD_PREP(ADAR1000_MUX_SEL_MSK, x)
#define ADAR1000_ADC_EOC		BIT(0)

/* ADAR1000_CH_GAIN */
#define ADAR1000_CH_ATTN		BIT(7)
#define ADAR1000_VGA_GAIN_MSK		GENMASK(6, 0)
#define ADAR1000_VGA_GAIN(x)		FIELD_PREP(ADAR1000_VGA_GAIN_MSK, x)

/* ADAR1000_CH polarity and phase for I and Q */
#define ADAR1000_VM_POL			BIT(5)
#define ADAR1000_VM_GAIN_MSK		GENMASK(4, 0)
#define ADAR1000_VM_GAIN(x)		FIELD_PREP(ADAR1000_VM_GAIN_MSK, x)

/* ADAR1000 LDO trim select */
#define ADAR1000_LDO_TRIM_SEL_MSK	GENMASK(1, 0)
#define ADAR1000_LDO_TRIM_SEL(x)	FIELD_PREP(ADAR1000_LDO_TRIM_SEL_MSK, x)

/* ADAR1000_MEM_CTRL */
#define ADAR1000_SCAN_MODE_EN		BIT(7)
#define ADAR1000_BEAM_RAM_BYPASS	BIT(6)
#define ADAR1000_BIAS_RAM_BYPASS	BIT(5)
#define ADAR1000_TX_BEAM_STEP_EN	BIT(3)
#define ADAR1000_RX_BEAM_STEP_EN	BIT(2)
#define ADAR1000_TX_CHX_RAM_BYPASS	BIT(1)
#define ADAR1000_RX_CHX_RAM_BYPASS	BIT(0)

/* ADAR1000_CH_MEM */
#define CHX_RAM_FETCH			BIT(7)

/* ADAR1000_SW_CTRL */
#define ADAR1000_SW_DRV_TR_STATE	BIT(7)
#define ADAR1000_TX_EN			BIT(6)
#define ADAR1000_RX_EN			BIT(5)
#define ADAR1000_SW_DRV_EN_TR		BIT(4)
#define ADAR1000_SW_DRV_EN_POL		BIT(3)
#define ADAR1000_TR_SOURCE		BIT(2)
#define ADAR1000_TR_SPI			BIT(1)
#define ADAR1000_POL			BIT(0)

/* RX_ENABLES/TX_ENABLES */
#define ADAR1000_CH1_EN			BIT(6)
#define ADAR1000_CH2_EN			BIT(5)
#define ADAR1000_CH3_EN			BIT(4)
#define ADAR1000_CH4_EN			BIT(3)
#define ADAR1000_RX_LNA_EN		BIT(2)
#define ADAR1000_TX_DRV_EN		BIT(2)
#define ADAR1000_VM_EN			BIT(1)
#define ADAR1000_VGA_EN			BIT(0)

/* ADAR1000_MISC_ENABLES */
#define ADAR1000_SW_DRV_TR_MODE_SEL	BIT(7)
#define ADAR1000_BIAS_CTRL		BIT(6)
#define ADAR1000_BIAS_EN		BIT(5)
#define ADAR1000_LNA_BIAS_OUT_EN	BIT(4)
#define ADAR1000_CH_DET_EN(ch)		(0x08 >> (ch))

/* ADAR1000_LD_WRK_REGS */
#define ADAR1000_LDTX_OVERRIDE		BIT(1)
#define ADAR1000_LDRX_OVERRIDE		BIT(0)

/* ADAR1000_RX_BIAS_RAM_CTL/ADAR1000_TX_BIAS_RAM_CTL */
#define ADAR1000_BIAS_RAM_FETCH		BIT(3)

#define ADAR1000_SPI_ADDR_MSK		GENMASK(14, 13)
#define ADAR1000_SPI_ADDR(x)		FIELD_PREP(ADAR1000_SPI_ADDR_MSK, x)

/* ADAR1000_TX_TO_RX_DELAY_CTRL/ADAR1000_RX_TO_TX_DELAY_CTRL */
#define ADAR1000_DELAY1_MSK		GENMASK(7, 4)
#define ADAR1000_DELAY2_MSK		GENMASK(3, 0)

/* SPI write all */
#define ADAR1000_SPI_WR_ALL		0x800

#define ADAR1000_MAX_DEV		4

/* Memory access */
#define ADAR1000_RAM_ACCESS_RX	0x1000
#define ADAR1000_RAM_ACCESS_TX	0x1800

/* RAM register 8-bit each */
#define ADAR1000_RAM_BEAM_POS_0(ch, pos)	(0x04 * (ch) + (pos << 4))
#define ADAR1000_RAM_BEAM_POS_1(ch, pos)	(0x04 * (ch) + 0x01 + (pos << 4))
#define ADAR1000_RAM_BEAM_POS_2(ch, pos)	(0x04 * (ch) + 0x02 + (pos << 4))

#define ADAR1000_RAM_RX_BIAS_SET_0(pos)		(0x780 + (pos << 4))
#define ADAR1000_RAM_RX_BIAS_SET_1(pos)		(0x780 + (pos << 4) + 0x01)
#define ADAR1000_RAM_RX_BIAS_SET_2(pos)		(0x780 + (pos << 4) + 0x04)
#define ADAR1000_RAM_RX_BIAS_SET_3(pos)		(0x780 + (pos << 4) + 0x05)

#define ADAR1000_RAM_TX_BIAS_SET_0(pos)		(0x780 + (pos << 4))
#define ADAR1000_RAM_TX_BIAS_SET_1(pos)		(0x780 + (pos << 4) + 0x01)
#define ADAR1000_RAM_TX_BIAS_SET_2(pos)		(0x780 + (pos << 4) + 0x02)
#define ADAR1000_RAM_TX_BIAS_SET_3(pos)		(0x780 + (pos << 4) + 0x04)
#define ADAR1000_RAM_TX_BIAS_SET_4(pos)		(0x780 + (pos << 4) + 0x05)
#define ADAR1000_RAM_TX_BIAS_SET_5(pos)		(0x780 + (pos << 4) + 0x06)
#define ADAR1000_RAM_TX_BIAS_SET_6(pos)		(0x780 + (pos << 4) + 0x08)
#define ADAR1000_RAM_TX_BIAS_SET_7(pos)		(0x780 + (pos << 4) + 0x09)
#define ADAR1000_RAM_TX_BIAS_SET_8(pos)		(0x780 + (pos << 4) + 0x0C)
#define ADAR1000_RAM_TX_BIAS_SET_9(pos)		(0x780 + (pos << 4) + 0x0D)

/* Beam position Vector Modulator (VM) and VGA Decoding - (bit 0 to bit 23) */
#define ADAR1000_RAM_VGA_GAIN_MSK	GENMASK(6, 0)
#define ADAR1000_RAM_VGA_GAIN(x)	FIELD_PREP(ADAR1000_RAM_VGA_GAIN_MSK, x)
#define ADAR1000_RAM_ATTENUATOR		BIT(7)
#define ADAR1000_RAM_VM_I_GAIN_MSK	GENMASK(12, 8)
#define ADAR1000_RAM_VM_I_GAIN(x)	FIELD_PREP(ADAR1000_RAM_VM_I_GAIN_MSK, x)
#define ADAR1000_RAM_VM_I_POL		BIT(13)
#define ADAR1000_RAM_VM_Q_GAIN_MSK	GENMASK(20, 16)
#define ADAR1000_RAM_VM_Q_GAIN(x)	FIELD_PREP(ADAR1000_RAM_VM_Q_GAIN_MSK, x)
#define ADAR1000_RAM_VM_Q_POL		BIT(21)

/* Receiver Bias setting decoding - (bit 0 to bit 31) - 4 bytes */
#define ADAR1000_RAM_RX_EXT_LNA_OFF_MSK	GENMASK(7, 0)
#define ADAR1000_RAM_RX_EXT_LNA_OFF(x)	FIELD_PREP(ADAR1000_RAM_RX_EXT_LNA_OFF_MSK, x)
#define ADAR1000_RAM_RX_EXT_LNA_ON_MSK	GENMASK(15, 8)
#define ADAR1000_RAM_RX_EXT_LNA_ON(x)	FIELD_PREP(ADAR1000_RAM_RX_EXT_LNA_ON_MSK, x)
#define ADAR1000_RAM_RX_VGA_BIAS_MSK	GENMASK(19, 16)
#define ADAR1000_RAM_RX_VGA_BIAS(x)	FIELD_PREP(ADAR1000_RAM_RX_VGA_BIAS_MSK, x)
#define ADAR1000_RAM_RX_VM_BIAS_MSK	GENMASK(22, 20)
#define ADAR1000_RAM_RX_VM_BIAS(x)	FIELD_PREP(ADAR1000_RAM_RX_VM_BIAS_MSK, x)
#define ADAR1000_RAM_RX_LNA_BIAS_MSK	GENMASK(27, 23)
#define ADAR1000_RAM_RX_LNA_BIAS(x)	FIELD_PREP(ADAR1000_RAM_RX_LNA_BIAS_MSK, x)

/* Transmitter Bias setting decoding - 10 bytes (bit 0 to bit 79 */
#define ADAR1000_RAM_TX_EXT_PA1_BIAS_OFF_MSK	GENMASK(7, 0)
#define ADAR1000_RAM_TX_EXT_PA1_BIAS_OFF(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA1_BIAS_OFF_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA2_BIAS_OFF_MSK	GENMASK(15, 8)
#define ADAR1000_RAM_TX_EXT_PA2_BIAS_OFF(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA2_BIAS_OFF_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA3_BIAS_OFF_MSK	GENMASK(23, 16)
#define ADAR1000_RAM_TX_EXT_PA3_BIAS_OFF(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA3_BIAS_OFF_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA1_BIAS_ON_MSK	GENMASK(31, 24)
#define ADAR1000_RAM_TX_EXT_PA1_BIAS_ON(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA1_BIAS_ON_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA2_BIAS_ON_MSK	GENMASK(39, 32)
#define ADAR1000_RAM_TX_EXT_PA2_BIAS_ON(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA2_BIAS_ON_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA3_BIAS_ON_MSK	GENMASK(47, 40)
#define ADAR1000_RAM_TX_EXT_PA3_BIAS_ON(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA3_BIAS_ON_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA4_BIAS_OFF_MSK	GENMASK(55, 48)
#define ADAR1000_RAM_TX_EXT_PA4_BIAS_OFF(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA4_BIAS_OFF_MSK, x)
#define ADAR1000_RAM_TX_EXT_PA4_BIAS_ON_MSK	GENMASK(63, 56)
#define ADAR1000_RAM_TX_EXT_PA4_BIAS_ON(x)	FIELD_PREP(ADAR1000_RAM_TX_EXT_PA4_BIAS_ON_MSK, x)
#define ADAR1000_RAM_TX_VGA_BIAS_MSK		GENMASK(67, 64)
#define ADAR1000_RAM_TX_VGA_BIAS(x)		FIELD_PREP(ADAR1000_RAM_TX_VGA_BIAS_MSK, x)
#define ADAR1000_RAM_TX_VM_BIAS_MSK		GENMASK(70, 68)
#define ADAR1000_RAM_TX_VM_BIAS(x)		FIELD_PREP(ADAR1000_RAM_TX_VM_BIAS_MSK, x)
#define ADAR1000_RAM_TX_DRV_BIAS_MSK		GENMASK(74, 72)
#define ADAR1000_RAM_TX_DRV_BIAS(x)		FIELD_PREP(ADAR1000_RAM_TX_DRV_BIAS_MSK, x)

#define ADAR1000_RAM_BEAM_POS_MIN	0
#define ADAR1000_RAM_BEAM_POS_MAX	120

#define ADAR1000_RAM_BIAS_SET_MIN	1
#define ADAR1000_RAM_BIAS_SET_MAX	7

struct adar1000_phase {
	u32 val;
	u32 val2;
	u8 vm_gain_i;
	u8 vm_gain_q;
};

struct adar1000_beam_position {
	int atten;
	int gain_val;
	int phase_val;
	int phase_val2;
};

struct adar1000_rx_bias_setting {
	u8 lna_bias_off;
	u8 lna_bias_on;
	u8 bias_current_rx;
	u8 bias_current_rx_lna;
};

struct adar1000_tx_bias_setting {
	u8 ch1_pa_bias_off;
	u8 ch2_pa_bias_off;
	u8 ch3_pa_bias_off;
	u8 ch4_pa_bias_off;
	u8 ch1_pa_bias_on;
	u8 ch2_pa_bias_on;
	u8 ch3_pa_bias_on;
	u8 ch4_pa_bias_on;
	u8 bias_current_tx;
	u8 bias_current_tx_drv;
};

struct adar1000_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	u16			dev_addr;

	int			tx_phase[4];
	int			rx_phase[4];
	struct adar1000_phase	*pt_info;
	unsigned int		pt_size;
	struct bin_attribute	bin_pt;
	char			*bin_attr_buf;

	/* RAM memory members */
	u8			load_beam_idx;
	u8			save_beam_idx;
	struct adar1000_beam_position rx_beam_pos[121];
	struct adar1000_beam_position tx_beam_pos[121];

	u8			load_bias_idx;
	u8			save_bias_idx;
	struct adar1000_rx_bias_setting rx_bias[7];
	struct adar1000_tx_bias_setting tx_bias[7];
};

static const struct regmap_config adar1000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

/* Phase values Table 13, 14, 15, 16 page 34 of the datasheet - these values
 * keep the vector modulator constant.
 */
static const struct adar1000_phase adar1000_phase_values[] = {
	{0, 0, 0x3f, 0x20}, {2, 8125, 0x3f, 0x21}, {5, 6250, 0x3f, 0x23},
	{8, 4375, 0x3F, 0x24}, {11, 2500, 0x3F, 0x26}, {14, 625, 0x3E, 0x27},
	{16, 8750, 0x3E, 0x28}, {19, 6875, 0x3D, 0x2A}, {22, 5000, 0x3D, 0x2B},
	{25, 3125, 0x3C, 0x2D}, {28, 1250, 0x3C, 0x2E}, {30, 9375, 0x3B, 0x2F},
	{33, 7500, 0x3A, 0x30}, {36, 5625, 0x39, 0x31}, {39, 3750, 0x38, 0x33},
	{42, 1875, 0x37, 0x34}, {45, 0, 0x36, 0x35}, {47, 8125, 0x35, 0x36},
	{50, 6250, 0x34, 0x37}, {53, 4375, 0x33, 0x38}, {56, 2500, 0x32, 0x38},
	{59, 625, 0x30, 0x39}, {61, 8750, 0x2F, 0x3A}, {64, 6875, 0x2E, 0x3A},
	{67, 5000, 0x2C, 0x3B}, {70, 3125, 0x2B, 0x3C}, {73, 1250, 0x2A, 0x3C},
	{75, 9375, 0x28, 0x3C}, {78, 7500, 0x27, 0x3D}, {81, 5625, 0x25, 0x3D},
	{84, 3750, 0x24, 0x3D}, {87, 1875, 0x22, 0x3D}, {90, 0, 0x21, 0x3D},
	{92, 8125, 0x01, 0x3D}, {95, 6250, 0x03, 0x3D}, {98, 4375, 0x04, 0x3D},
	{101, 2500, 0x06, 0x3D}, {104, 625, 0x07, 0x3C}, {106, 8750, 0x08, 0x3C},
	{109, 6875, 0x0A, 0x3C}, {112, 5000, 0x0B, 0x3B}, {115, 3125, 0x0D, 0x3A},
	{118, 1250, 0x0E, 0x3A}, {120, 9375, 0x0F, 0x39}, {123, 7500, 0x11, 0x38},
	{126, 5625, 0x12, 0x38}, {129, 3750, 0x13, 0x37}, {132, 1875, 0x14, 0x36},
	{135, 0, 0x16, 0x35}, {137, 8125, 0x17, 0x34}, {140, 6250, 0x18, 0x33},
	{143, 4375, 0x19, 0x31}, {146, 2500, 0x19, 0x30}, {149, 625, 0x1A, 0x2F},
	{151, 8750, 0x1B, 0x2E}, {154, 6875, 0x1C, 0x2D}, {157, 5000, 0x1C, 0x2B},
	{160, 3125, 0x1D, 0x2A}, {163, 1250, 0x1E, 0x28}, {165, 9375, 0x1E, 0x27},
	{168, 7500, 0x1E, 0x26}, {171, 5625, 0x1F, 0x24}, {174, 3750, 0x1F, 0x23},
	{177, 1875, 0x1F, 0x21}, {180, 0, 0x1F, 0x20}, {182, 8125, 0x1F, 0x01},
	{185, 6250, 0x1F, 0x03}, {188, 4375, 0x1F, 0x04}, {191, 2500, 0x1F, 0x06},
	{194, 625, 0x1E, 0x07}, {196, 8750, 0x1E, 0x08}, {199, 6875, 0x1D, 0x0A},
	{202, 5000, 0x1D, 0x0B}, {205, 3125, 0x1C, 0x0D}, {208, 1250, 0x1C, 0x0E},
	{210, 9375, 0x1B, 0x0F}, {213, 7500, 0x1A, 0x10}, {216, 5625, 0x19, 0x11},
	{219, 3750, 0x18, 0x13}, {222, 1875, 0x17, 0x14}, {225, 0, 0x16, 0x15},
	{227, 8125, 0x15, 0x16}, {230, 6250, 0x14, 0x17}, {233, 4375, 0x13, 0x18},
	{236, 2500, 0x12, 0x18}, {239, 625, 0x10, 0x19}, {241, 8750, 0x0F, 0x1A},
	{244, 6875, 0x0E, 0x1A}, {247, 5000, 0x0C, 0x1B}, {250, 3125, 0x0B, 0x1C},
	{253, 1250, 0x0A, 0x1C}, {255, 9375, 0x08, 0x1C}, {258, 7500, 0x07, 0x1D},
	{261, 5625, 0x05, 0x1D}, {264, 3750, 0x04, 0x1D}, {267, 1875, 0x02, 0x1D},
	{270, 0, 0x01, 0x1D}, {272, 8125, 0x21, 0x1D}, {275, 6250, 0x23, 0x1D},
	{278, 4375, 0x24, 0x1D}, {281, 2500, 0x26, 0x1D}, {284, 625, 0x27, 0x1C},
	{286, 8750, 0x28, 0x1C}, {289, 6875, 0x2A, 0x1C}, {292, 5000, 0x2B, 0x1B},
	{295, 3125, 0x2D, 0x1A}, {298, 1250, 0x2E, 0x1A}, {300, 9375, 0x2F, 0x19},
	{303, 7500, 0x31, 0x18}, {306, 5625, 0x32, 0x18}, {309, 3750, 0x33, 0x17},
	{312, 1875, 0x34, 0x16}, {315, 0, 0x36, 0x15}, {317, 8125, 0x37, 0x14},
	{320, 6250, 0x38, 0x13}, {323, 4375, 0x39, 0x11}, {326, 2500, 0x39, 0x10},
	{329, 625, 0x3A, 0x0F}, {331, 8750, 0x3B, 0x0E}, {334, 6875, 0x3C, 0x0D},
	{337, 5000, 0x3C, 0x0B}, {340, 3125, 0x3D, 0x0A}, {343, 1250, 0x3E, 0x08},
	{345, 9375, 0x3E, 0x07}, {348, 7500, 0x3E, 0x06}, {351, 5625, 0x3F, 0x04},
	{354, 3750, 0x3F, 0x03}, {357, 1875, 0x3F, 0x01}
};

static int adar1000_mode_4wire(struct adar1000_state *st, bool enable)
{
	int ret;
	uint8_t tmp;

	if (enable)
		tmp = ADAR1000_SDOACTIVE | ADAR1000_SDOACTIVE_;
	else
		tmp = 0;

	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_INTERFACE_CFG_A, tmp);
	if (ret < 0)
		return ret;

	/* If device addr is 0 treat the broadcast scenario and reset all other
	 * devices back to 3-wire SPI mode
	 */
	if (st->dev_addr != 0 || !enable)
		return 0;

	ret = regmap_write(st->regmap, ADAR1000_SPI_ADDR(1) |
			   ADAR1000_INTERFACE_CFG_A, 0);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADAR1000_SPI_ADDR(2) |
			   ADAR1000_INTERFACE_CFG_A, 0);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, ADAR1000_SPI_ADDR(3) |
			    ADAR1000_INTERFACE_CFG_A, 0);
}

static int adar1000_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	if (readval)
		ret = regmap_read(st->regmap, st->dev_addr | reg, readval);
	else
		ret = regmap_write(st->regmap, st->dev_addr | reg, writeval);
	if (ret < 0)
		return ret;

	return adar1000_mode_4wire(st, 0);
}

static int adar1000_get_atten(struct adar1000_state *st, u32 ch_num, u8 output)
{
	u32 val, reg;
	int ret;

	if (output)
		reg = ADAR1000_CH_TX_GAIN(ch_num);
	else
		reg = ADAR1000_CH_RX_GAIN(ch_num);

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	val &= ~ADAR1000_CH_ATTN;

	return val;
}

static int adar1000_set_atten(struct adar1000_state *st, u32 atten_mdb,
			      u32 ch_num, u8 output)
{
	u32 reg;
	int ret;

	if (output)
		reg = ADAR1000_CH_TX_GAIN(ch_num);
	else
		reg = ADAR1000_CH_RX_GAIN(ch_num);

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, st->dev_addr | reg,
				 (u32)~ADAR1000_CH_ATTN, atten_mdb);
	if (ret < 0)
		return ret;

	return adar1000_mode_4wire(st, 0);
}

static int adar1000_read_adc(struct adar1000_state *st, u8 adc_ch, u32 *adc_data)
{
	u32 adc_ctrl;
	int ret, timeout = 100;

	/* Setup ADC operation */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_ADC_CTRL, ADAR1000_AC_EN | ADAR1000_CLK_EN |
			   ADAR1000_MUX_SEL(adc_ch) | ADAR1000_ST_CONV);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	do {
		ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_ADC_CTRL,
				  &adc_ctrl);
		if (ret < 0)
			return ret;

		timeout = timeout -1;
		if (timeout == 0)
			return -ENOENT;

		mdelay(1);
	} while (!(adc_ctrl & ADAR1000_ADC_EOC));

	/* Read ADC sample */
	ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_ADC_OUTPUT,
			  adc_data);
	if (ret < 0)
		return ret;

	/* Disable ADC */
	ret = regmap_write(st->regmap, st->dev_addr | ADAR1000_ADC_CTRL, 0);
	if (ret < 0)
		return ret;

	return adar1000_mode_4wire(st, 0);
}

static void adar1000_phase_search(struct adar1000_state *st, int val, int val2,
				  u32 *vm_gain_i, u32 *vm_gain_q, int *value_degree)
{
	int i, prev, next;

	val %= 360;
	if (val < 0)
		val += 360;

	for (i = 0; i < st->pt_size - 1; i++) {
		if (st->pt_info[i].val > val)
			break;
	}

	prev = st->pt_info[i - 1].val * 10000 +	st->pt_info[i - 1].val2;
	next = st->pt_info[i].val * 10000 + st->pt_info[i].val2;
	*value_degree = val * 10000 + val2 / 100;

	/* If value is over the last entry in the pt_info */
	if (next < *value_degree) {
		prev = next;
		next = st->pt_info[0].val * 10000 + st->pt_info[0].val2;
	}

	if ((*value_degree - prev) < (next - *value_degree)) {
		*vm_gain_i = st->pt_info[i - 1].vm_gain_i;
		*vm_gain_q = st->pt_info[i - 1].vm_gain_q;
		*value_degree = prev;
	} else {
		*vm_gain_i = st->pt_info[i].vm_gain_i;
		*vm_gain_q = st->pt_info[i].vm_gain_q;
		*value_degree = next;
	}
}

static int adar1000_set_phase(struct adar1000_state *st, u8 ch_num, u8 output,
			      int val, int val2)
{
	int ret, value;
	u32 vm_gain_i, vm_gain_q;
	u16 reg_i, reg_q;

	adar1000_phase_search(st, val, val2, &vm_gain_i, &vm_gain_q, &value);

	if (output) {
		reg_i = st->dev_addr | ADAR1000_CH_TX_PHASE_I(ch_num);
		reg_q = st->dev_addr | ADAR1000_CH_TX_PHASE_Q(ch_num);
		st->tx_phase[ch_num] = value;
	} else {
		reg_i = st->dev_addr | ADAR1000_CH_RX_PHASE_I(ch_num);
		reg_q = st->dev_addr | ADAR1000_CH_RX_PHASE_Q(ch_num);
		st->rx_phase[ch_num] = value;
	}

	ret = regmap_write(st->regmap, reg_i, vm_gain_i);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, reg_q, vm_gain_q);
}

static int adar1000_get_phase(struct adar1000_state *st, u8 ch_num, u8 output,
			      u32 *val, u32 *val2)
{
	if (output) {
		*val = st->tx_phase[ch_num] / 10000;
		*val2 = (st->tx_phase[ch_num] % 10000) * 100;
	} else {
		*val = st->rx_phase[ch_num] / 10000;
		*val2 = (st->rx_phase[ch_num] % 10000) * 100;
	}

	return IIO_VAL_INT_PLUS_MICRO;
}

static int adar1000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret, ch;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type != IIO_TEMP)
			ch = chan->channel + 1;
		else
			ch = chan->channel;

		ret = adar1000_read_adc(st, ch, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = adar1000_get_atten(st, chan->channel, chan->output);
		if (ret < 0)
			return ret;

		*val = -1 * (ret / 1000);
		*val2 = (ret % 1000) * 1000;

		if (!*val)
			*val2 *= -1;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
		return adar1000_get_phase(st, chan->channel, chan->output,
					  val, val2);
	default:
		return -EINVAL;
	}
};

static int adar1000_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct adar1000_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adar1000_set_atten(st, val, chan->channel,
					   chan->output);
	case IIO_CHAN_INFO_PHASE:
		return adar1000_set_phase(st, chan->channel, chan->output,
					  val, val2);
	default:
		return -EINVAL;
	};
}

static int adar1000_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

enum adar1000_iio_dev_attr {
	ADAR1000_RX_VGA,
	ADAR1000_RX_VM,
	ADAR1000_RX_LNA,
	ADAR1000_TX_VGA,
	ADAR1000_TX_VM,
	ADAR1000_TX_DRV,
	ADAR1000_LNABIAS_ON,
	ADAR1000_LNABIAS_OFF,
	ADAR1000_CUR_RX_LNA,
	ADAR1000_CUR_RX,
	ADAR1000_CUR_TX,
	ADAR1000_CUR_TX_DRV,
	ADAR1000_SW_DRV_TR_MODE_SEL_,
	ADAR1000_BIAS_CTRL_,
	ADAR1000_BIAS_EN_,
	ADAR1000_LNA_BIAS_OUT_EN_,
};

static ssize_t adar1000_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	bool readin;
	u8 readval;
	u16 reg = 0;
	int ret = 0;
	u32 val = 0, mask = 0;

	switch ((u32)this_attr->address) {
	case ADAR1000_RX_VGA:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_VGA_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_VGA_EN;
		break;
	case ADAR1000_RX_VM:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_VM_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_VM_EN;
		break;
	case ADAR1000_RX_LNA:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_RX_LNA_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_RX_LNA_EN;
		break;
	case ADAR1000_TX_VGA:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_VGA_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_VGA_EN;
		break;
	case ADAR1000_TX_VM:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_VM_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_VM_EN;
		break;
	case ADAR1000_TX_DRV:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_TX_DRV_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_TX_DRV_EN;
		break;
	case ADAR1000_LNABIAS_ON:
		reg = ADAR1000_LNA_BIAS_ON;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;
		break;
	case ADAR1000_LNABIAS_OFF:
		reg = ADAR1000_LNA_BIAS_OFF;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;
		break;
	case ADAR1000_CUR_RX_LNA:
		reg = ADAR1000_BIAS_CURRENT_RX_LNA;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		readval &= 0xf;
		break;
	case ADAR1000_CUR_RX:
		reg = ADAR1000_BIAS_CURRENT_RX;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		readval &= 0x7f;
		break;
	case ADAR1000_CUR_TX:
		reg = ADAR1000_BIAS_CURRENT_TX;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		readval &= 0x7f;
		break;
	case ADAR1000_CUR_TX_DRV:
		reg = ADAR1000_BIAS_CURRENT_TX_DRV;
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		readval &= 0x7;
		break;
	case ADAR1000_SW_DRV_TR_MODE_SEL_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_SW_DRV_TR_MODE_SEL;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_SW_DRV_TR_MODE_SEL;
		break;
	case ADAR1000_BIAS_CTRL_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_BIAS_CTRL;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_BIAS_CTRL;
		break;
	case ADAR1000_BIAS_EN_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_BIAS_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (!readin)
			val = ADAR1000_BIAS_EN;
		break;
	case ADAR1000_LNA_BIAS_OUT_EN_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_LNA_BIAS_OUT_EN;
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (readin)
			val = ADAR1000_LNA_BIAS_OUT_EN;
		break;
	default:
		return -EINVAL;
	}

	if (mask) {
		ret = adar1000_mode_4wire(st, 1);
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(st->regmap, st->dev_addr | reg, mask, val);
		if (ret < 0)
			return ret;

		ret = adar1000_mode_4wire(st, 0);
	} else {
		ret = regmap_write(st->regmap, st->dev_addr | reg, readval);
	}

	return ret ? ret : len;
}

static ssize_t adar1000_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret = 0;
	u16 reg = 0;
	unsigned int val, mask = 0;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	switch ((u32)this_attr->address) {
	case ADAR1000_RX_VGA:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_VGA_EN;
		break;
	case ADAR1000_RX_VM:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_VM_EN;
		break;
	case ADAR1000_RX_LNA:
		reg = ADAR1000_RX_ENABLES;
		mask = ADAR1000_RX_LNA_EN;
		break;
	case ADAR1000_TX_VGA:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_VGA_EN;
		break;
	case ADAR1000_TX_VM:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_VM_EN;
		break;
	case ADAR1000_TX_DRV:
		reg = ADAR1000_TX_ENABLES;
		mask = ADAR1000_TX_DRV_EN;
		break;
	case ADAR1000_LNABIAS_ON:
		reg = ADAR1000_LNA_BIAS_ON;
		break;
	case ADAR1000_LNABIAS_OFF:
		reg = ADAR1000_LNA_BIAS_OFF;
		break;
	case ADAR1000_CUR_RX_LNA:
		reg = ADAR1000_BIAS_CURRENT_RX_LNA;
		break;
	case ADAR1000_CUR_RX:
		reg = ADAR1000_BIAS_CURRENT_RX;
		break;
	case ADAR1000_CUR_TX:
		reg = ADAR1000_BIAS_CURRENT_TX;
		break;
	case ADAR1000_CUR_TX_DRV:
		reg = ADAR1000_BIAS_CURRENT_TX_DRV;
		break;
	case ADAR1000_SW_DRV_TR_MODE_SEL_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_SW_DRV_TR_MODE_SEL;
		break;
	case ADAR1000_BIAS_CTRL_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_BIAS_CTRL;
		break;
	case ADAR1000_BIAS_EN_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_BIAS_EN;
		break;
	case ADAR1000_LNA_BIAS_OUT_EN_:
		reg = ADAR1000_MISC_ENABLES;
		mask = ADAR1000_LNA_BIAS_OUT_EN;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	if (mask)
		val = !!(val & mask);

	return sprintf(buf, "%d\n", val);
}

static ssize_t adar1000_reset(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;

	/* Reset device */
	ret = regmap_write(st->regmap, st->dev_addr | ADAR1000_INTERFACE_CFG_A,
			   ADAR1000_SOFTRESET | ADAR1000_SOFTRESET_);
	if (ret < 0)
		return ret;

	/* Clear phase values */
	memset(st->tx_phase, 0, sizeof(st->tx_phase));
	memset(st->rx_phase, 0, sizeof(st->rx_phase));

	return ret ? ret : len;
}

static ssize_t adar1000_seq_en_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar1000_state *st = iio_priv(indio_dev);
	unsigned int val, mask;
	int ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_MEM_CTRL, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	mask = ADAR1000_TX_BEAM_STEP_EN | ADAR1000_RX_BEAM_STEP_EN;

	return sprintf(buf, "%d\n", (val & mask) == mask);
}

static ssize_t adar1000_seq_en_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar1000_state *st = iio_priv(indio_dev);
	bool readin;
	u8 val;
	int ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	/* Setup sequencer */
	if (readin)
		val = ADAR1000_TX_BEAM_STEP_EN | ADAR1000_RX_BEAM_STEP_EN;
	else
		val = 0;

	ret = regmap_update_bits(st->regmap, st->dev_addr | ADAR1000_MEM_CTRL,
				 ADAR1000_TX_BEAM_STEP_EN | ADAR1000_RX_BEAM_STEP_EN,
				 val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);

	return ret ? ret : len;
}

static ssize_t adar1000_gen_clk_cycles(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar1000_state *st = iio_priv(indio_dev);
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;
	u8 buff = 0xff;

	/* Generate clock cycles to load new data from RAM */
	t.tx_buf = &buff;
	t.bits_per_word = 8;
	t.len = 1;

	spi_message_init_with_transfers(&m, &t, 1);

	ret = spi_sync_locked(st->spi, &m);

	return ret ? ret : len;
}

enum adar1000_iio_delay_attr {
	ADAR1000_TX_RX_DELAY1,
	ADAR1000_TX_RX_DELAY2,
	ADAR1000_RX_TX_DELAY1,
	ADAR1000_RX_TX_DELAY2,
};

static ssize_t adar1000_delay_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u16 reg;
	u32 val;

	switch ((u32)this_attr->address) {
	case ADAR1000_TX_RX_DELAY1:
		reg = ADAR1000_TX_TO_RX_DELAY_CTRL;
		break;
	case ADAR1000_TX_RX_DELAY2:
		reg = ADAR1000_TX_TO_RX_DELAY_CTRL;
		break;
	case ADAR1000_RX_TX_DELAY1:
		reg = ADAR1000_RX_TO_TX_DELAY_CTRL;
		break;
	case ADAR1000_RX_TX_DELAY2:
		reg = ADAR1000_RX_TO_TX_DELAY_CTRL;
		break;
	default:
		return -EINVAL;
	}

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	if ((u32)this_attr->address % 2)
		val = FIELD_GET(ADAR1000_DELAY2_MSK, val);
	else
		val = FIELD_GET(ADAR1000_DELAY1_MSK, val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t adar1000_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u16 reg;
	u8 readval;

	ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

	switch ((u32)this_attr->address) {
	case ADAR1000_TX_RX_DELAY1:
		reg = ADAR1000_TX_TO_RX_DELAY_CTRL;
		readval = FIELD_PREP(ADAR1000_DELAY1_MSK, readval);
		break;
	case ADAR1000_TX_RX_DELAY2:
		reg = ADAR1000_TX_TO_RX_DELAY_CTRL;
		readval = FIELD_PREP(ADAR1000_DELAY2_MSK, readval);
		break;
	case ADAR1000_RX_TX_DELAY1:
		reg = ADAR1000_RX_TO_TX_DELAY_CTRL;
		readval = FIELD_PREP(ADAR1000_DELAY1_MSK, readval);
		break;
	case ADAR1000_RX_TX_DELAY2:
		reg = ADAR1000_RX_TO_TX_DELAY_CTRL;
		readval = FIELD_PREP(ADAR1000_DELAY2_MSK, readval);
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, st->dev_addr | reg, readval);

	return ret ? ret : len;
}

enum adar1000_iio_memctl_attr {
	ADAR1000_COMMON_MEM_EN,
	ADAR1000_BEAM_MEM_EN,
	ADAR1000_BIAS_MEM_EN,
	ADAR1000_STATIC_RX_BEAM_POS,
	ADAR1000_STATIC_TX_BEAM_POS,
};

static ssize_t adar1000_memctl_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u16 reg;
	u8 mask;
	u32 val;

	switch ((u32)this_attr->address) {
	case ADAR1000_COMMON_MEM_EN:
		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_TX_CHX_RAM_BYPASS | ADAR1000_RX_CHX_RAM_BYPASS;
		break;
	case ADAR1000_BEAM_MEM_EN:
		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_BEAM_RAM_BYPASS;
		break;
	case ADAR1000_BIAS_MEM_EN:
		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_BIAS_RAM_BYPASS;
		break;
	case ADAR1000_STATIC_RX_BEAM_POS:
		reg = ADAR1000_RX_CHX_MEM;
		mask = 0x7F;
		break;
	case ADAR1000_STATIC_TX_BEAM_POS:
		reg = ADAR1000_TX_CHX_MEM;
		mask = 0x7F;
		break;
	default:
		return -EINVAL;
	}

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	if ((u32)this_attr->address <= ADAR1000_BIAS_MEM_EN)
		return sprintf(buf, "%d\n", !(val & mask));
	else
		return sprintf(buf, "%d\n", val & mask);
}

static ssize_t adar1000_memctl_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u16 reg;
	u8 readval = 0, mask;
	bool readin;


	switch ((u32)this_attr->address) {
	case ADAR1000_COMMON_MEM_EN:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_TX_CHX_RAM_BYPASS | ADAR1000_RX_CHX_RAM_BYPASS;

		ret = adar1000_mode_4wire(st, 1);
		if (ret < 0)
			return ret;

		if (!readin)
			readval = mask;

		ret = regmap_update_bits(st->regmap, st->dev_addr | reg,
					 mask, readval);
		if (ret < 0)
			return ret;

		ret = adar1000_mode_4wire(st, 0);
		break;
	case ADAR1000_BEAM_MEM_EN:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_BEAM_RAM_BYPASS;

		ret = adar1000_mode_4wire(st, 1);
		if (ret < 0)
			return ret;

		if (!readin)
			readval = mask;

		ret = regmap_update_bits(st->regmap, st->dev_addr | reg,
					 mask, readval);
		if (ret < 0)
			return ret;

		ret = adar1000_mode_4wire(st, 0);
		break;
	case ADAR1000_BIAS_MEM_EN:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		reg = ADAR1000_MEM_CTRL;
		mask = ADAR1000_BIAS_RAM_BYPASS;

		ret = adar1000_mode_4wire(st, 1);
		if (ret < 0)
			return ret;

		if (!readin)
			readval = mask;

		ret = regmap_update_bits(st->regmap, st->dev_addr | reg,
					 mask, readval);
		if (ret < 0)
			return ret;

		ret = adar1000_mode_4wire(st, 0);
		break;
	case ADAR1000_STATIC_RX_BEAM_POS:
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, st->dev_addr | ADAR1000_RX_CHX_MEM,
				   readval | CHX_RAM_FETCH);
		break;
	case ADAR1000_STATIC_TX_BEAM_POS:
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, st->dev_addr | ADAR1000_TX_CHX_MEM,
				   readval | CHX_RAM_FETCH);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

enum adar1000_iio_swctl_attr {
	ADAR1000_SW_DRV_TR_STATE_,
	ADAR1000_TX_EN_,
	ADAR1000_RX_EN_,
	ADAR1000_SW_DRV_EN_TR_,
	ADAR1000_SW_DRV_EN_POL_,
	ADAR1000_TR_SOURCE_,
	ADAR1000_TR_SPI_,
	ADAR1000_POL_,
};

static ssize_t adar1000_swctl_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u32 val;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_SW_CTRL, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	switch ((u32)this_attr->address) {
	case ADAR1000_SW_DRV_TR_STATE_:
		val = FIELD_GET(ADAR1000_SW_DRV_TR_STATE, val);
		break;
	case ADAR1000_TX_EN_:
		val = FIELD_GET(ADAR1000_TX_EN, val);
		break;
	case ADAR1000_RX_EN_:
		val = FIELD_GET(ADAR1000_RX_EN, val);
		break;
	case ADAR1000_SW_DRV_EN_TR_:
		val = FIELD_GET(ADAR1000_SW_DRV_EN_TR, val);
		break;
	case ADAR1000_SW_DRV_EN_POL_:
		val = FIELD_GET(ADAR1000_SW_DRV_EN_POL, val);
		break;
	case ADAR1000_TR_SOURCE_:
		val = FIELD_GET(ADAR1000_TR_SOURCE, val);
		break;
	case ADAR1000_TR_SPI_:
		val = FIELD_GET(ADAR1000_TR_SPI, val);
		break;
	case ADAR1000_POL_:
		val = FIELD_GET(ADAR1000_POL, val);
		break;
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t adar1000_swctl_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u8 val = 0, mask;
	bool readin;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case ADAR1000_SW_DRV_TR_STATE_:
		mask = ADAR1000_SW_DRV_TR_STATE;
		break;
	case ADAR1000_TX_EN_:
		mask = ADAR1000_TX_EN;
		break;
	case ADAR1000_RX_EN_:
		mask = ADAR1000_RX_EN;
		break;
	case ADAR1000_SW_DRV_EN_TR_:
		mask = ADAR1000_SW_DRV_EN_TR;
		break;
	case ADAR1000_SW_DRV_EN_POL_:
		mask = ADAR1000_SW_DRV_EN_POL;
		break;
	case ADAR1000_TR_SOURCE_:
		mask = ADAR1000_TR_SOURCE;
		break;
	case ADAR1000_TR_SPI_:
		mask = ADAR1000_TR_SPI;
		break;
	case ADAR1000_POL_:
		mask = ADAR1000_POL;
		break;
	default:
		return -EINVAL;
	}

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	if (readin)
		val = mask;

	ret = regmap_update_bits(st->regmap, st->dev_addr | ADAR1000_SW_CTRL,
				 mask, val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);

	return ret ? ret : len;
}

enum adar1000_iio_ldwrk_attr {
	ADAR1000_LDTX,
	ADAR1000_LDRX,
};

static ssize_t adar1000_ldwrk_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	u8 val = 0;
	bool readin;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	switch ((u32)this_attr->address) {
	case ADAR1000_LDTX:
		val = ADAR1000_LDTX_OVERRIDE;
		break;
	case ADAR1000_LDRX:
		val = ADAR1000_LDRX_OVERRIDE;
		break;
	default:
		return -EINVAL;
	}

	if (readin)
		ret = regmap_write(st->regmap, st->dev_addr | ADAR1000_LD_WRK_REGS, val);

	return ret ? ret : len;
}


static IIO_DEVICE_ATTR(rx_vga_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_RX_VGA);

static IIO_DEVICE_ATTR(rx_vm_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_RX_VM);

static IIO_DEVICE_ATTR(rx_lna_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_RX_LNA);

static IIO_DEVICE_ATTR(tx_vga_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_TX_VGA);

static IIO_DEVICE_ATTR(tx_vm_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_TX_VM);

static IIO_DEVICE_ATTR(tx_drv_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_TX_DRV);

/* MISC_ENABLES */
static IIO_DEVICE_ATTR(sw_drv_tr_mode_sel, 0644,
		       adar1000_show, adar1000_store, ADAR1000_SW_DRV_TR_MODE_SEL_);
static IIO_DEVICE_ATTR(bias_ctrl, 0644,
		       adar1000_show, adar1000_store, ADAR1000_BIAS_CTRL_);
static IIO_DEVICE_ATTR(bias_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_BIAS_EN_);
static IIO_DEVICE_ATTR(lna_bias_out_enable, 0644,
		       adar1000_show, adar1000_store, ADAR1000_LNA_BIAS_OUT_EN_);

/* MEM_CTL attirbutes */
static IIO_DEVICE_ATTR(common_mem_enable, 0644,
		       adar1000_memctl_show, adar1000_memctl_store, ADAR1000_COMMON_MEM_EN);
static IIO_DEVICE_ATTR(beam_mem_enable, 0644,
		       adar1000_memctl_show, adar1000_memctl_store, ADAR1000_BEAM_MEM_EN);
static IIO_DEVICE_ATTR(bias_mem_enable, 0644,
		       adar1000_memctl_show, adar1000_memctl_store, ADAR1000_BIAS_MEM_EN);
static IIO_DEVICE_ATTR(static_rx_beam_pos_load, 0644,
		       adar1000_memctl_show, adar1000_memctl_store, ADAR1000_STATIC_RX_BEAM_POS);
static IIO_DEVICE_ATTR(static_tx_beam_pos_load, 0644,
		       adar1000_memctl_show, adar1000_memctl_store, ADAR1000_STATIC_TX_BEAM_POS);

/* SW_CTL attributes */
static IIO_DEVICE_ATTR(sw_drv_tr_state, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_SW_DRV_TR_STATE_);
static IIO_DEVICE_ATTR(tx_en, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_TX_EN_);
static IIO_DEVICE_ATTR(rx_en, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_RX_EN_);
static IIO_DEVICE_ATTR(sw_drv_en_tr, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_SW_DRV_EN_TR_);
static IIO_DEVICE_ATTR(sw_drv_en_pol, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_SW_DRV_EN_POL_);
static IIO_DEVICE_ATTR(tr_source, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_TR_SOURCE_);
static IIO_DEVICE_ATTR(tr_spi, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_TR_SPI_);
static IIO_DEVICE_ATTR(pol, 0644,
		       adar1000_swctl_show, adar1000_swctl_store, ADAR1000_POL_);

/* LNA BIAS setting */
static IIO_DEVICE_ATTR(lna_bias_off, 0644,
		       adar1000_show, adar1000_store, ADAR1000_LNABIAS_OFF);

static IIO_DEVICE_ATTR(lna_bias_on, 0644,
		       adar1000_show, adar1000_store, ADAR1000_LNABIAS_ON);

/* BIAS current configurations */
static IIO_DEVICE_ATTR(bias_current_rx_lna, 0644,
		       adar1000_show, adar1000_store, ADAR1000_CUR_RX_LNA);

static IIO_DEVICE_ATTR(bias_current_rx, 0644,
		       adar1000_show, adar1000_store, ADAR1000_CUR_RX);

static IIO_DEVICE_ATTR(bias_current_tx, 0644,
		       adar1000_show, adar1000_store, ADAR1000_CUR_TX);

static IIO_DEVICE_ATTR(bias_current_tx_drv, 0644,
		       adar1000_show, adar1000_store, ADAR1000_CUR_TX_DRV);

/* Reset attribute */
static IIO_DEVICE_ATTR(reset, 0200, NULL, adar1000_reset, 0);

/* Load working registers attributes */
static IIO_DEVICE_ATTR(tx_load_spi, 0200, NULL, adar1000_ldwrk_store, ADAR1000_LDTX);
static IIO_DEVICE_ATTR(rx_load_spi, 0200, NULL, adar1000_ldwrk_store, ADAR1000_LDRX);

/* Sequencer enable attribute - should be called before TR_LOAD */
static IIO_DEVICE_ATTR(sequencer_enable, 0644, adar1000_seq_en_show, adar1000_seq_en_store, 0);

/* Generate CLK cycles for SPI */
static IIO_DEVICE_ATTR(gen_clk_cycles, 0200, NULL, adar1000_gen_clk_cycles, 0);

/* Delay attributes */
static IIO_DEVICE_ATTR(tx_to_rx_delay_1, 0644,
		       adar1000_delay_show, adar1000_delay_store, ADAR1000_TX_RX_DELAY1);
static IIO_DEVICE_ATTR(tx_to_rx_delay_2, 0644,
		       adar1000_delay_show, adar1000_delay_store, ADAR1000_TX_RX_DELAY2);
static IIO_DEVICE_ATTR(rx_to_tx_delay_1, 0644,
		       adar1000_delay_show, adar1000_delay_store, ADAR1000_RX_TX_DELAY1);
static IIO_DEVICE_ATTR(rx_to_tx_delay_2, 0644,
		       adar1000_delay_show, adar1000_delay_store, ADAR1000_RX_TX_DELAY2);

static struct attribute *adar1000_attributes[] = {
	&iio_dev_attr_rx_vga_enable.dev_attr.attr,
	&iio_dev_attr_rx_vm_enable.dev_attr.attr,
	&iio_dev_attr_rx_lna_enable.dev_attr.attr,
	&iio_dev_attr_tx_vga_enable.dev_attr.attr,
	&iio_dev_attr_tx_vm_enable.dev_attr.attr,
	&iio_dev_attr_tx_drv_enable.dev_attr.attr,
	&iio_dev_attr_lna_bias_off.dev_attr.attr,
	&iio_dev_attr_lna_bias_on.dev_attr.attr,
	&iio_dev_attr_bias_current_rx_lna.dev_attr.attr,
	&iio_dev_attr_bias_current_rx.dev_attr.attr,
	&iio_dev_attr_bias_current_tx.dev_attr.attr,
	&iio_dev_attr_bias_current_tx_drv.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	&iio_dev_attr_sequencer_enable.dev_attr.attr,
	&iio_dev_attr_gen_clk_cycles.dev_attr.attr,
	&iio_dev_attr_sw_drv_tr_mode_sel.dev_attr.attr,
	&iio_dev_attr_bias_ctrl.dev_attr.attr,
	&iio_dev_attr_bias_enable.dev_attr.attr,
	&iio_dev_attr_lna_bias_out_enable.dev_attr.attr,
	&iio_dev_attr_tx_to_rx_delay_1.dev_attr.attr,
	&iio_dev_attr_tx_to_rx_delay_2.dev_attr.attr,
	&iio_dev_attr_rx_to_tx_delay_1.dev_attr.attr,
	&iio_dev_attr_rx_to_tx_delay_2.dev_attr.attr,
	&iio_dev_attr_common_mem_enable.dev_attr.attr,
	&iio_dev_attr_beam_mem_enable.dev_attr.attr,
	&iio_dev_attr_bias_mem_enable.dev_attr.attr,
	&iio_dev_attr_static_rx_beam_pos_load.dev_attr.attr,
	&iio_dev_attr_static_tx_beam_pos_load.dev_attr.attr,
	&iio_dev_attr_sw_drv_tr_state.dev_attr.attr,
	&iio_dev_attr_tx_en.dev_attr.attr,
	&iio_dev_attr_rx_en.dev_attr.attr,
	&iio_dev_attr_sw_drv_en_tr.dev_attr.attr,
	&iio_dev_attr_sw_drv_en_pol.dev_attr.attr,
	&iio_dev_attr_tr_source.dev_attr.attr,
	&iio_dev_attr_tr_spi.dev_attr.attr,
	&iio_dev_attr_pol.dev_attr.attr,
	&iio_dev_attr_tx_load_spi.dev_attr.attr,
	&iio_dev_attr_rx_load_spi.dev_attr.attr,
	NULL,
};

static const struct attribute_group adar1000_attribute_group = {
	.attrs = adar1000_attributes,
};

static const struct iio_info adar1000_info = {
	.read_raw = &adar1000_read_raw,
	.write_raw = &adar1000_write_raw,
	.write_raw_get_fmt = &adar1000_write_raw_get_fmt,
	.debugfs_reg_access = &adar1000_reg_access,
	.attrs = &adar1000_attribute_group,
};

/* RAM access - BEAM Position */
static int adar1000_beam_load(struct adar1000_state *st, u32 channel, bool tx,
			      u32 profile)
{
	if (profile < ADAR1000_RAM_BEAM_POS_MIN || profile > ADAR1000_RAM_BEAM_POS_MAX)
		return -EINVAL;

	st->load_beam_idx = profile;

	if (tx)
		return regmap_write(st->regmap, st->dev_addr | ADAR1000_TX_CH_MEM(channel),
				    CHX_RAM_FETCH | profile);
	else
		return regmap_write(st->regmap, st->dev_addr | ADAR1000_RX_CH_MEM(channel),
				    CHX_RAM_FETCH | profile);
}

static int adar1000_beam_save(struct adar1000_state *st, u32 channel, bool tx,
			      u32 profile, struct adar1000_beam_position beam)
{
	int phase_value;
	u16 ram_access_tx_rx;
	u32 vm_gain_i, vm_gain_q;
	struct reg_sequence regs[3] = {0};

	if (profile < ADAR1000_RAM_BEAM_POS_MIN || profile > ADAR1000_RAM_BEAM_POS_MAX)
		return -EINVAL;

	st->save_beam_idx = profile;

	/* Set phase value */
	adar1000_phase_search(st, beam.phase_val, beam.phase_val2,
			      &vm_gain_i, &vm_gain_q, &phase_value);
	beam.phase_val = phase_value / 10000;
	beam.phase_val2 = phase_value % 10000 * 100;

	/* Set gain value & save beam information */
	if (tx) {
		ram_access_tx_rx = ADAR1000_RAM_ACCESS_TX;
		st->tx_beam_pos[profile] = beam;
	} else {
		ram_access_tx_rx = ADAR1000_RAM_ACCESS_RX;
		st->rx_beam_pos[profile] = beam;
	}

	regs[0].reg = st->dev_addr | ADAR1000_RAM_BEAM_POS_0(channel, profile) | ram_access_tx_rx;

	if (beam.atten)
		regs[0].def = beam.gain_val | ADAR1000_CH_ATTN;
	else
		regs[0].def = beam.gain_val;

	regs[1].reg = st->dev_addr | ADAR1000_RAM_BEAM_POS_1(channel, profile) | ram_access_tx_rx;
	regs[1].def = vm_gain_i;
	regs[2].reg = st->dev_addr | ADAR1000_RAM_BEAM_POS_2(channel, profile) | ram_access_tx_rx;
	regs[2].def = vm_gain_q;

	return regmap_multi_reg_write(st->regmap, regs, ARRAY_SIZE(regs));
}

static int adar1000_bias_load(struct adar1000_state *st, u32 channel, bool tx,
			      u32 setting)
{
	if (setting < ADAR1000_RAM_BIAS_SET_MIN || setting > ADAR1000_RAM_BIAS_SET_MAX)
		return -EINVAL;

	/* Bias settings are from 1 to 7 so subtract 1 to get the index */
	st->load_bias_idx = setting - 1;

	if (tx)
		return regmap_write(st->regmap, st->dev_addr | ADAR1000_TX_BIAS_RAM_CTL,
				    ADAR1000_BIAS_RAM_FETCH  | (setting - 1));
	else
		return regmap_write(st->regmap, st->dev_addr | ADAR1000_RX_BIAS_RAM_CTL,
				    ADAR1000_BIAS_RAM_FETCH  | (setting - 1));
}

static int adar1000_bias_save(struct adar1000_state *st, u32 channel, bool tx,
			      u32 setting, void * const bias)
{
	int cnt;
	struct reg_sequence regs[10] = {0};

	if (setting < ADAR1000_RAM_BIAS_SET_MIN || setting > ADAR1000_RAM_BIAS_SET_MAX)
		return -EINVAL;

	/* Bias settings are from 1 to 7 so subtract 1 to get the index */
	st->save_bias_idx = setting - 1;

	if (tx) {
		regs[0].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_0(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[0].def = ((struct adar1000_tx_bias_setting*)bias)->ch1_pa_bias_off;
		regs[1].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_1(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[1].def = ((struct adar1000_tx_bias_setting*)bias)->ch2_pa_bias_off;
		regs[2].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_2(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[2].def = ((struct adar1000_tx_bias_setting*)bias)->ch3_pa_bias_off;

		regs[3].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_3(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[3].def = ((struct adar1000_tx_bias_setting*)bias)->ch1_pa_bias_on;
		regs[4].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_4(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[4].def = ((struct adar1000_tx_bias_setting*)bias)->ch2_pa_bias_on;
		regs[5].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_5(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[5].def = ((struct adar1000_tx_bias_setting*)bias)->ch3_pa_bias_on;

		regs[6].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_6(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[6].def = ((struct adar1000_tx_bias_setting*)bias)->ch4_pa_bias_off;
		regs[7].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_7(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[7].def = ((struct adar1000_tx_bias_setting*)bias)->ch4_pa_bias_on;
		regs[8].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_8(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[8].def = ((struct adar1000_tx_bias_setting*)bias)->bias_current_tx;
		regs[9].reg = st->dev_addr | ADAR1000_RAM_TX_BIAS_SET_9(setting) | ADAR1000_RAM_ACCESS_TX;
		regs[9].def = ((struct adar1000_tx_bias_setting*)bias)->bias_current_tx_drv;
		cnt = 10;

		st->tx_bias[setting -1] = *(struct adar1000_tx_bias_setting*)bias;
	} else {
		regs[0].reg = st->dev_addr | ADAR1000_RAM_RX_BIAS_SET_0(setting) | ADAR1000_RAM_ACCESS_RX;
		regs[0].def = ((struct adar1000_rx_bias_setting*)bias)->lna_bias_off;
		regs[1].reg = st->dev_addr | ADAR1000_RAM_RX_BIAS_SET_1(setting) | ADAR1000_RAM_ACCESS_RX;
		regs[1].def = ((struct adar1000_rx_bias_setting*)bias)->lna_bias_on;
		regs[2].reg = st->dev_addr | ADAR1000_RAM_RX_BIAS_SET_2(setting) | ADAR1000_RAM_ACCESS_RX;
		regs[2].def = ((struct adar1000_rx_bias_setting*)bias)->bias_current_rx;
		regs[3].reg = st->dev_addr | ADAR1000_RAM_RX_BIAS_SET_3(setting) | ADAR1000_RAM_ACCESS_RX;
		regs[3].def = ((struct adar1000_rx_bias_setting*)bias)->bias_current_rx_lna;
		cnt = 4;

		st->rx_bias[setting - 1] = *(struct adar1000_rx_bias_setting*)bias;
	}

	return regmap_multi_reg_write(st->regmap, regs, cnt);
}

enum beam_pos_info {
	BEAM_POS_LOAD,
	BEAM_POS_SAVE,
	BIAS_SET_LOAD,
	BIAS_SET_SAVE,
};

static int adar1000_bias_parse(struct adar1000_state *st, bool tx, const char *buf,
			       size_t len,  u64 *readin, void **value_bias)
{
	int ret;
	char *line, *ptr = (char*) buf;
	int val, i = 0, j = 0, cnt;
	u8 tmp[10];

	while ((line = strsep(&ptr, ","))) {
		if (line >= buf + len)
			break;

		if (j == 0) {
			ret = kstrtoull(buf, 10, readin);
			if (ret < 0)
				return ret;
			j++;
			continue;
		}

		ret = sscanf(line, "%d", &val);
		if (ret == 1)
			tmp[i] = val;
		i++;
	}

	if (tx)
		cnt = sizeof(struct adar1000_tx_bias_setting);
	else
		cnt = sizeof(struct adar1000_rx_bias_setting);


	*value_bias = devm_kzalloc(&st->spi->dev, cnt, GFP_KERNEL);
	if (!*value_bias)
		return -ENOMEM;

	memcpy(*value_bias, tmp, cnt);

	return 0;
}

static ssize_t adar1000_ram_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u64 readin;
	int ret = 0;

	switch (private) {
	case BEAM_POS_LOAD: {
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		ret = adar1000_beam_load(st, chan->channel, chan->output == 1,
					 readin);
		break;
	}
	case BEAM_POS_SAVE: {
		char *line, *ptr = (char*) buf;
		int val, val2, tmp[4], i = 0, j = 0;
		struct adar1000_beam_position value;

		while ((line = strsep(&ptr, ","))) {
			if (line >= buf + len)
				break;

			if (j == 0) {
				ret = kstrtoull(buf, 10, &readin);
				if (ret < 0)
					return ret;
				j++;
				continue;
			}

			ret = sscanf(line, "%d.%d", &val, &val2);
			if (ret == 1) {
				tmp[i] = val;
			} else if (ret == 2) {
				tmp[i] = val;
				tmp[i + 1] = val2;
			}
			i += ret;
		}

		value.atten = tmp[0];
		value.gain_val = tmp[1];
		value.phase_val = tmp[2];
		value.phase_val2 = tmp[3];

		ret = adar1000_beam_save(st, chan->channel,
					 chan->output == 1, readin, value);

		break;
	}
	case BIAS_SET_LOAD:
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		ret = adar1000_bias_load(st, chan->channel, chan->output == 1, readin);
		break;
	case BIAS_SET_SAVE: {
		void *value_bias = NULL;

		ret = adar1000_bias_parse(st, chan->output == 1, buf, len,
					  &readin, &value_bias);
		if (ret < 0)
			return ret;

		ret = adar1000_bias_save(st, chan->channel, chan->output == 1, readin, value_bias);
		break;
	}
	}

	return ret ? ret : len;
}

static ssize_t adar1000_ram_read(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u64 val = 0;
	size_t len = 0;
	int ret = 0;

	switch (private) {
	case BEAM_POS_SAVE:
		if (chan->output == 1)
			len += sprintf(buf, "%d, %d, %d, %d.%06u\n",
				       st->save_beam_idx,
				       st->tx_beam_pos[st->save_beam_idx].atten,
				       st->tx_beam_pos[st->save_beam_idx].gain_val,
				       st->tx_beam_pos[st->save_beam_idx].phase_val,
				       st->tx_beam_pos[st->save_beam_idx].phase_val2);
		else
			len += sprintf(buf, "%d, %d, %d, %d.%06u\n",
				       st->save_beam_idx,
				       st->rx_beam_pos[st->save_beam_idx].atten,
				       st->rx_beam_pos[st->save_beam_idx].gain_val,
				       st->rx_beam_pos[st->save_beam_idx].phase_val,
				       st->rx_beam_pos[st->save_beam_idx].phase_val2);
		val = st->save_beam_idx;
		break;
	case BEAM_POS_LOAD:
		val  = st->load_beam_idx;
		len += sprintf(buf + len, "%llu\n", val);
		break;
	case BIAS_SET_LOAD:
		val  = st->load_bias_idx + 1;
		len += sprintf(buf + len, "%llu\n", val);
		break;
	case BIAS_SET_SAVE:
		if (chan->output == 1)
			len += sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
				       st->save_bias_idx + 1,
				       st->tx_bias[st->save_bias_idx].ch1_pa_bias_off,
				       st->tx_bias[st->save_bias_idx].ch2_pa_bias_off,
				       st->tx_bias[st->save_bias_idx].ch3_pa_bias_off,
				       st->tx_bias[st->save_bias_idx].ch4_pa_bias_off,
				       st->tx_bias[st->save_bias_idx].ch1_pa_bias_on,
				       st->tx_bias[st->save_bias_idx].ch2_pa_bias_on,
				       st->tx_bias[st->save_bias_idx].ch3_pa_bias_on,
				       st->tx_bias[st->save_bias_idx].ch4_pa_bias_on,
				       st->tx_bias[st->save_bias_idx].bias_current_tx,
				       st->tx_bias[st->save_bias_idx].bias_current_tx_drv);
		else
			len += sprintf(buf, "%d, %d, %d, %d, %d\n",
				       st->save_bias_idx + 1,
				       st->rx_bias[st->save_bias_idx].lna_bias_off,
				       st->rx_bias[st->save_bias_idx].lna_bias_on,
				       st->rx_bias[st->save_bias_idx].bias_current_rx,
				       st->rx_bias[st->save_bias_idx].bias_current_rx_lna);
		val = st->save_beam_idx;
		break;
	default:
		ret = 0;
	}

	return ret < 0 ? ret : len;
}

enum adar1000_enables {
	ADAR1000_POWERDOWN,
	ADAR1000_DETECTOR,
	ADAR1000_PA_BIAS_ON,
	ADAR1000_PA_BIAS_OFF,
	ADAR1000_ATTEN,
};

static ssize_t adar1000_read_enable(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u16 reg = 0;
	unsigned int val;
	int ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	switch (private) {
	case ADAR1000_POWERDOWN:
		if (chan->output)
			reg = ADAR1000_TX_ENABLES;
		else
			reg = ADAR1000_RX_ENABLES;

		ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
		if (ret < 0)
			return ret;

		val = !(val & (0x40 >> chan->channel));

		break;
	case ADAR1000_DETECTOR:
		ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_MISC_ENABLES, &val);
		if (ret < 0)
			return ret;

		val = (val & 0xf) & (0x8 >> chan->channel);

		break;
	case ADAR1000_PA_BIAS_ON:
		ret = regmap_read(st->regmap, st->dev_addr |
				  ADAR1000_CH_PA_BIAS_ON(chan->channel), &val);
		if (ret < 0)
			return ret;
		break;
	case ADAR1000_PA_BIAS_OFF:
		ret = regmap_read(st->regmap, st->dev_addr |
				  ADAR1000_CH_PA_BIAS_OFF(chan->channel), &val);
		if (ret < 0)
			return ret;

		break;
	case ADAR1000_ATTEN:
		if (chan->output)
			reg = ADAR1000_CH_TX_GAIN(chan->channel);
		else
			reg = ADAR1000_CH_RX_GAIN(chan->channel);

		ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
		if (ret < 0)
			return ret;

		val = FIELD_GET(ADAR1000_CH_ATTN, val);
		break;
	}

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static ssize_t adar1000_write_enable(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u16 reg = 0;
	unsigned int val = 0, mask = 0;
	bool readin;
	u8 readval;
	int ret;

	switch (private) {
	case ADAR1000_POWERDOWN:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (chan->output)
			reg = ADAR1000_TX_ENABLES;
		else
			reg = ADAR1000_RX_ENABLES;

		mask = ADAR1000_CH1_EN >> chan->channel;
		if (!readin)
			val = ADAR1000_CH1_EN >> chan->channel;

		break;
	case ADAR1000_DETECTOR:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		mask = ADAR1000_CH_DET_EN(chan->channel);
		if (readin)
			val = ADAR1000_CH_DET_EN(chan->channel);

		reg = ADAR1000_MISC_ENABLES;
		break;
	case ADAR1000_PA_BIAS_ON:
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		reg = ADAR1000_CH_PA_BIAS_ON(chan->channel);
		break;
	case ADAR1000_PA_BIAS_OFF:
		ret = kstrtou8(buf, 10, &readval);
		if (ret)
			return ret;

		reg = ADAR1000_CH_PA_BIAS_OFF(chan->channel);
		break;
	case ADAR1000_ATTEN:
		ret = kstrtobool(buf, &readin);
		if (ret)
			return ret;

		if (chan->output)
			reg = ADAR1000_CH_TX_GAIN(chan->channel);
		else
			reg = ADAR1000_CH_RX_GAIN(chan->channel);

		mask = ADAR1000_CH_ATTN;
		if (readin)
			val = mask;

		break;
	}

	if (mask) {
		ret = adar1000_mode_4wire(st, 1);
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(st->regmap, st->dev_addr | reg, mask, val);
		if (ret < 0)
			return ret;

		ret = adar1000_mode_4wire(st, 0);
	} else {
		ret = regmap_write(st->regmap, st->dev_addr | reg, readval);
	}

	return ret ? ret : len;
}

enum adar1000_sequence {
	ADAR1000_SEQ_START,
	ADAR1000_SEQ_STOP
};

static ssize_t adar1000_seq_show(struct iio_dev *indio_dev,  uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u16 reg = 0;
	unsigned int val;
	int ret;

	ret = adar1000_mode_4wire(st, 1);
	if (ret < 0)
		return ret;

	switch(private) {
	case ADAR1000_SEQ_START:
		if (chan->output)
			reg = ADAR1000_TX_BEAM_STEP_START;
		else
			reg = ADAR1000_RX_BEAM_STEP_START;
		break;
	case ADAR1000_SEQ_STOP:
		if (chan->output)
			reg = ADAR1000_TX_BEAM_STEP_STOP;
		else
			reg = ADAR1000_RX_BEAM_STEP_STOP;
		break;
	}

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	ret = adar1000_mode_4wire(st, 0);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", val);
}

static ssize_t adar1000_seq_store(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan, const char *buf, size_t len)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u16 reg = 0;
	u8 readval;
	int ret;

	ret = kstrtou8(buf, 10, &readval);
	if (ret)
		return ret;

	switch(private) {
	case ADAR1000_SEQ_START:
		if (chan->output)
			reg = ADAR1000_TX_BEAM_STEP_START;
		else
			reg = ADAR1000_RX_BEAM_STEP_START;
		break;
	case ADAR1000_SEQ_STOP:
		if (chan->output)
			reg = ADAR1000_TX_BEAM_STEP_STOP;
		else
			reg = ADAR1000_RX_BEAM_STEP_STOP;
		break;
	}

	ret = regmap_write(st->regmap, st->dev_addr | reg, readval);

	return ret ? ret : len;
}

#define _ADAR1000_SEQ_INFO(_name, _private) { \
	.name = _name, \
	.read = adar1000_seq_show, \
	.write = adar1000_seq_store, \
	.shared = IIO_SHARED_BY_TYPE, \
	.private = _private, \
}

#define _ADAR1000_ENABLES_INFO(_name, _private) { \
	.name = _name, \
	.read = adar1000_read_enable, \
	.write = adar1000_write_enable, \
	.shared = IIO_SEPARATE, \
	.private = _private, \
}

#define _ADAR1000_BEAM_POS_INFO(_name, _ident) { \
	.name = _name, \
	.read = adar1000_ram_read, \
	.write = adar1000_ram_write, \
	.private = _ident, \
}

#define _ADAR1000_RAM_BIAS_INFO(_name, _private) { \
	.name = _name, \
	.read = adar1000_ram_read, \
	.write = adar1000_ram_write, \
	.shared = IIO_SHARED_BY_TYPE, \
	.private = _private, \
}

static const struct iio_chan_spec_ext_info adar1000_rx_ext_info[] = {
	_ADAR1000_BEAM_POS_INFO("beam_pos_load", BEAM_POS_LOAD),
	_ADAR1000_BEAM_POS_INFO("beam_pos_save", BEAM_POS_SAVE),
	_ADAR1000_RAM_BIAS_INFO("bias_set_load", BIAS_SET_LOAD),
	_ADAR1000_RAM_BIAS_INFO("bias_set_save", BIAS_SET_SAVE),
	_ADAR1000_SEQ_INFO("sequence_start", ADAR1000_SEQ_START),
	_ADAR1000_SEQ_INFO("sequence_end", ADAR1000_SEQ_STOP),
	_ADAR1000_ENABLES_INFO("powerdown", ADAR1000_POWERDOWN),
	_ADAR1000_ENABLES_INFO("attenuation", ADAR1000_ATTEN),
	{ },
};

static const struct iio_chan_spec_ext_info adar1000_tx_ext_info[] = {
	_ADAR1000_BEAM_POS_INFO("beam_pos_load", BEAM_POS_LOAD),
	_ADAR1000_BEAM_POS_INFO("beam_pos_save", BEAM_POS_SAVE),
	_ADAR1000_RAM_BIAS_INFO("bias_set_load", BIAS_SET_LOAD),
	_ADAR1000_RAM_BIAS_INFO("bias_set_save", BIAS_SET_SAVE),
	_ADAR1000_ENABLES_INFO("detector_en", ADAR1000_DETECTOR),
	_ADAR1000_SEQ_INFO("sequence_start", ADAR1000_SEQ_START),
	_ADAR1000_SEQ_INFO("sequence_end", ADAR1000_SEQ_STOP),
	_ADAR1000_ENABLES_INFO("powerdown", ADAR1000_POWERDOWN),
	_ADAR1000_ENABLES_INFO("pa_bias_on", ADAR1000_PA_BIAS_ON),
	_ADAR1000_ENABLES_INFO("pa_bias_off", ADAR1000_PA_BIAS_OFF),
	_ADAR1000_ENABLES_INFO("attenuation", ADAR1000_ATTEN),
	{ },
};

static const struct iio_chan_spec_ext_info adar1000_det_ext_info[] = {
	{ }
};

#define ADAR1000_RX_CHANNEL(_num)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
	.extend_name = "RX",					\
	.ext_info = adar1000_rx_ext_info,			\
}

#define ADAR1000_TX_CHANNEL(_num)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE) |			\
		BIT(IIO_CHAN_INFO_RAW),				\
	.extend_name = "TX",					\
	.ext_info = adar1000_tx_ext_info,			\
}

#define ADAR1000_TEMP_CHANNEL(_num)				\
{	.type = IIO_TEMP,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
}

static const struct iio_chan_spec adar1000_chan[] = {
	ADAR1000_TEMP_CHANNEL(0),
	ADAR1000_RX_CHANNEL(0),
	ADAR1000_RX_CHANNEL(1),
	ADAR1000_RX_CHANNEL(2),
	ADAR1000_RX_CHANNEL(3),
	ADAR1000_TX_CHANNEL(0),
	ADAR1000_TX_CHANNEL(1),
	ADAR1000_TX_CHANNEL(2),
	ADAR1000_TX_CHANNEL(3),
};

static void adar1000_free_pt(struct adar1000_state *st,
			     struct adar1000_phase *table)
{
	if (!table || table == adar1000_phase_values)
		return;

	devm_kfree(&st->spi->dev, table);
}

static struct adar1000_phase *adar1000_parse_pt(struct adar1000_state *st,
						char *data, u32 size)
{
	struct adar1000_phase *table;
	unsigned int model, count;
	bool header_found;
	int i = 0, ret = 0;
	char *line, *ptr = data;

	header_found = false;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size)
			break;

		if (line[0] == '#') /* skip comment lines */
			continue;

		if (!header_found) {

			ret = sscanf(line, " <phasetable ADAR%i table entries=%i>",
				     &model, &count);

			if (ret == 2) {
				if (!(model == 1000)) {
					ret = -EINVAL;
					goto out;
				}
				table = devm_kzalloc(&st->spi->dev,
					sizeof(struct adar1000_phase) * count,
					GFP_KERNEL);
				if (!table) {
					ret = -ENOMEM;
					goto out;
				}

				header_found = true;
				i = 0;

				continue;
			} else {
				header_found = false;
			}
		}

		if (header_found) {
			int a, b, c, d;

			ret = sscanf(line, " %i.%i,%i,%i", &a, &b, &c, &d);
			if (ret == 4) {
				if (i >= count)
					goto out;

				table[i].val = a;
				table[i].val2 = b;
				table[i].vm_gain_i = c;
				table[i].vm_gain_q = d;
				i++;
				continue;
			} else if (strstr(line, "</phasetable>")) {
				goto done;
			} else {
				dev_err(&st->spi->dev,
					"ERROR: Malformed phase table");
				goto out_free_table;
			}
		}
	}

done:
	if (header_found) {
		st->pt_size = count;
		return table;
	} else {
		return ERR_PTR(ret);
	}

out_free_table:
	adar1000_free_pt(st, table);

out:
	return ERR_PTR(ret);
}

static int adar1000_request_pt(struct adar1000_state *st)
{

	struct iio_dev *indio_dev;
	const struct firmware *fw;
	struct adar1000_phase *phase_table;
	const char *name;
	char *cpy;
	int ret;

	indio_dev = iio_priv_to_dev(st);

	if (of_property_read_string(indio_dev->dev.of_node,
				    "adi,phasetable-name", &name))
		return -ENOENT;

	ret = request_firmware(&fw, name, &st->spi->dev);
	if (ret) {
		dev_err(&st->spi->dev,
			"request_firmware(%s) failed with %i\n", name, ret);
		return ret;
	}

	cpy = devm_kzalloc(&st->spi->dev, fw->size, GFP_KERNEL);
	if (!cpy)
		goto out;

	memcpy(cpy, fw->data, fw->size);

	phase_table = adar1000_parse_pt(st, cpy, fw->size);
	if (IS_ERR_OR_NULL(phase_table)) {
		ret = PTR_ERR(phase_table);
		goto out;
	}

	adar1000_free_pt(st, st->pt_info);

	st->pt_info = phase_table;

out:
	release_firmware(fw);

	return ret;
}

static ssize_t adar1000_pt_bin_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *bin_attr,
				     char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adar1000_state *st = iio_priv(indio_dev);
	struct adar1000_phase *table;

	if (off == 0) {
		if (!st->bin_attr_buf) {
			st->bin_attr_buf = devm_kzalloc(&st->spi->dev,
							bin_attr->size,
							GFP_KERNEL);
			if (!st->bin_attr_buf)
				return -ENOMEM;
		} else {
			memset(st->bin_attr_buf, 0, bin_attr->size);
		}
	}

	memcpy(st->bin_attr_buf + off, buf, count);

	if (strnstr(st->bin_attr_buf, "</phasetable>", off + count) == NULL)
		return count;

	table = adar1000_parse_pt(st, st->bin_attr_buf, off + count);
	if (IS_ERR_OR_NULL(table))
		return PTR_ERR(table);

	adar1000_free_pt(st, st->pt_info);

	st->pt_info = table;

	return count;
}

static ssize_t adar1000_pt_bin_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret, i, len = 0;
	char *tab;

	tab = kzalloc(count, GFP_KERNEL);
	if (!tab)
		return -ENOMEM;

	len += snprintf(tab + len, count - len,
			"<phasetable ADAR%i table entries=%i>\n", 1000,
			st->pt_size);

	for (i = 0; i < st->pt_size; i++)
		len += snprintf(tab + len, count - len,
			"%d.%04u, 0x%.2X, 0x%.2X\n",
			st->pt_info[i].val,
			st->pt_info[i].val2,
			st->pt_info[i].vm_gain_i,
			st->pt_info[i].vm_gain_q);

	len += snprintf(tab + len, count - len, "</phasetable>\n");

	ret = memory_read_from_buffer(buf, count, &off, tab, len);

	kfree(tab);

	return ret;
}

static int adar1000_setup(struct iio_dev *indio_dev)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;

	/* Load phase values */
	ret = adar1000_request_pt(st);
	if (ret < 0) {
		st->pt_info = (struct adar1000_phase *)&adar1000_phase_values[0];
		st->pt_size = ARRAY_SIZE(adar1000_phase_values);
	}

	/* Reset device */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_INTERFACE_CFG_A,
			   ADAR1000_SOFTRESET | ADAR1000_SOFTRESET_);
	if (ret < 0)
		return ret;

	/* Adjust LDOs */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_LDO_TRIM_CTL_1,
			   ADAR1000_LDO_TRIM_SEL(2));
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, st->dev_addr | ADAR1000_LDO_TRIM_CTL_0, 0x55);
}

static int adar1000_probe(struct spi_device *spi)
{
	struct iio_dev **indio_dev;
	struct adar1000_state **st;
	struct device_node *child, *np = spi->dev.of_node;
	struct regmap *regmap;
	int ret, cnt = 0, num_dev;
	u32 tmp;

	num_dev = of_get_available_child_count(np);
	if (num_dev < 1 || num_dev > ADAR1000_MAX_DEV) {
		dev_err(&spi->dev, "Number of devices is incorrect (%d)\n",
			num_dev);
		return -ENODEV;
	}

	indio_dev = devm_kzalloc(&spi->dev, num_dev, sizeof(indio_dev));
	if (!indio_dev)
		return -ENOMEM;

	st = devm_kzalloc(&spi->dev, num_dev, sizeof(st));
	if (!st)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adar1000_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	for_each_available_child_of_node(np, child) {
		indio_dev[cnt] = devm_iio_device_alloc(&spi->dev,
						       sizeof(*st));
		if (!indio_dev[cnt])
			return -ENOMEM;

		st[cnt] = iio_priv(indio_dev[cnt]);
		spi_set_drvdata(spi, indio_dev[cnt]);
		st[cnt]->spi = spi;
		st[cnt]->regmap = regmap;

		ret = of_property_read_u32(child, "reg", &tmp);
		if (ret < 0)
			return ret;

		st[cnt]->dev_addr = ADAR1000_SPI_ADDR(tmp);

		sysfs_bin_attr_init(&st[cnt]->bin_pt);
		st[cnt]->bin_pt.attr.name = "phase_table_config";
		st[cnt]->bin_pt.attr.mode = 0644;
		st[cnt]->bin_pt.write = adar1000_pt_bin_write;
		st[cnt]->bin_pt.read = adar1000_pt_bin_read;
		st[cnt]->bin_pt.size = 4096;

		indio_dev[cnt]->dev.parent = &spi->dev;
		indio_dev[cnt]->dev.of_node = child;
		indio_dev[cnt]->name = child->name;
		indio_dev[cnt]->info = &adar1000_info;
		indio_dev[cnt]->modes = INDIO_DIRECT_MODE;
		indio_dev[cnt]->channels = adar1000_chan;
		indio_dev[cnt]->num_channels = ARRAY_SIZE(adar1000_chan);

		ret = adar1000_setup(indio_dev[cnt]);
		if (ret < 0) {
			dev_err(&spi->dev, "Setup failed (%d)\n", ret);
			return ret;
		}

		ret = devm_iio_device_register(&spi->dev, indio_dev[cnt]);
		if (ret < 0)
			return ret;

		ret = sysfs_create_bin_file(&indio_dev[cnt]->dev.kobj,
					    &st[cnt]->bin_pt);
		if (ret < 0)
			return ret;

		cnt++;
	}

	return 0;
}

static const struct of_device_id adar1000_of_match[] = {
	{ .compatible = "adi,adar1000"},
	{ }
};
MODULE_DEVICE_TABLE(of, adar1000_of_match);

static struct spi_driver adar1000_driver = {
	.driver = {
		.name	= "adar1000",
		.of_match_table = adar1000_of_match,
	},
	.probe		= adar1000_probe,
};
module_spi_driver(adar1000_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR1000 Beamformer");
MODULE_LICENSE("GPL v2");
