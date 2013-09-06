/*
 * AD9361
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9361_H_
#define IIO_FREQUENCY_AD9361_H_

#define SPI_CONFIGURATION		0x000

#define TX_ENABLE_FILTER		0x002
#define RX_ENABLE_FILTER		0x003
#define INPUT_SELECT			0x004
#define RFPLL_DIVIDERS			0x005
#define RX_CLOCK_DATA_DELAY		0x006
#define TX_CLOCK_DATA_DELAY		0x007

#define CLK_ENABLE			0x009
#define BBPLL				0x00A

#define TEMP_SENSE_1			0x00C
#define TEMP_SENSE_2			0x00D

#define PARALLEL_PORT_CONFIG1		0x010
#define PARALLEL_PORT_CONFIG2		0x011
#define PARALLEL_PORT_CONFIG3		0x012
#define ENSM_MODE			0x013
#define ENSM_CONFIG_1			0x014
#define ENSM_CONFIG_2			0x015
#define CALIBRATION_CONTROL		0x016
#define STATE				0x017

#define AUX_DAC1_CONFIG			0x01A
#define AUX_DAC2_CONFIG			0x01B
#define AUX_ADC_CLOCK_DIVIDER		0x01C
#define AUX_ADC_CONFIG			0x01D

#define AGC_ATTACK_DELAY			0x022
#define AGC_ATTACK_DELAY_MASK		0x3F

#define AUX_DAC_ENABLE_CONTROL		0x023
#define RX_LOAD_SYNTH_DELAY		0x024
#define TX_LOAD_SYNTH_DELAY		0x025
#define EXTERNAL_LNA_CONTROL		0x026
#define GPO_FORCE_INIT			0x027

#define CONTROL_OUTPUT_POINTER		0x035
#define CONTROL_OUTPUT_ENABLE		0x036
#define PRODUCT_ID                      0x037
#define PRODUCT_ID_MASK 0xF8
#define PRODUCT_ID_9361 0x08
#define REV_MASK	0x07

#define REFERENCE_CLOCK_CYCLES		0x03A

#define DIGITAL_IO_CONTROL		0x03B
#define LVDS_BIAS_CONTROL		0x03C
#define LVDS_INVERT_CONTROL1		0x03D
#define LVDS_INVERT_CONTROL2		0x03E

#define SDM_CONTROL_1			0x03F
#define FRAC_BB_FREQ_WORD_1		0x041
#define FRAC_BB_FREQ_WORD_2		0x042
#define FRAC_BB_FREQ_WORD_3		0x043
#define INT_BB_FREQ_WORD		0x044
#define CLOCK_CONTROL			0x045
#define CP_CURRENT			0x046
#define LOOP_FILTER_1			0x048
#define LOOP_FILTER_2			0x049
#define LOOP_FILTER_3			0x04A
#define VCO_CONTROL			0x04B
#define VCO_PROGRAM_1			0x04C
#define VCO_PROGRAM_2			0x04D
#define SDM_CONTROL			0x04E
#define CH1_OVERFLOW			0x05E

#define TX_FIR_COEFF_ADDR		0x060
#define TX_FIR_COEFF_WRDATA_LOW		0x061
#define TX_FIR_COEFF_WRDATA_HIGH	0x062
#define TX_FIR_COEFF_RDDATA_LOW		0x063
#define TX_FIR_COEFF_RDDATA_HIGH	0x064
#define TX_FIR_CONFIG			0x065

#define TX1_ATTEN_0			0x073
#define TX1_ATTEN_1			0x074
#define TX2_ATTEN_0			0x075
#define TX2_ATTEN_1			0x076
#define TX_ATTEN_OFFSET			0x077
#define TX2_DIG_ATTENUATION		0x07C

#define TX_SYMBOL_ATTEN_CONFIG		0x081

#define QUAD_CAL_NCO_FREQ_PHASE		0x0A0
#define QUAD_CAL_CONTROL		0x0A1
#define KEXP_1				0x0A2
#define KEXP_2				0x0A3
#define SETTLE_COUNT			0x0A4
#define MAG_FTEST_THRESH		0x0A5
#define MAG_FTEST_THRESH_2		0x0A6
#define QUAD_CAL_COUNT			0x0A9
#define TX_QUAD_FULL_LMT_GAIN		0x0AA
#define TX_QUAD_LPF_GAIN		0x0AE

#define TX_BBF_R1			0x0C2
#define TX_BBF_R2			0x0C3
#define TX_BBF_R3			0x0C4
#define TX_BBF_R4			0x0C5
#define TX_BBF_RP			0x0C6
#define TX_BBF_C1			0x0C7
#define TX_BBF_C2			0x0C8
#define TX_BBF_CP			0x0C9
#define TX_TUNE_CONTROL			0x0CA
#define CONFIG0				0x0D0
#define RESISTOR			0x0D1
#define CAPACITOR			0x0D2
#define TX_BBF_TUNE_DIVIDER		0x0D6
#define TX_BBF_TUNE_MODE		0x0D7

#define RX_FIR_COEFF_ADDR		0x0F0
#define RX_FIR_COEFF_WRDATA_LOW		0x0F1
#define RX_FIR_COEFF_WRDATA_HIGH	0x0F2
#define RX_FIR_COEFF_RDDATA_LOW		0x0F3
#define RX_FIR_COEFF_RDDATA_HIGH	0x0F4
#define RX_FIR_CONFIG			0x0F5
#define RX_FIR_GAIN			0x0F6

#define AGC_CONFIG_1			0x0FA
#define RX1_GAIN_CTRL(x)			(((x) & 0x3) << 0)
#define RX2_GAIN_CTRL(x)			(((x) & 0x3) << 2)
#define SLOW_ATTACK_HYBRID_MODE		(1 << 4)
#define DEC_PWR_GAIN_LOCK_EXIT		(1 << 5)
#define DEC_PWR_LOCK_LEVEL		(1 << 6)
#define DEC_PWR_LOW_PWR			(1 << 7)

#define AGC_CONFIG_2			0x0FB
#define MAN_GAIN_CTRL_RX1		(1 << 0)
#define MAN_GAIN_CTRL_RX2		(1 << 1)
#define DIG_GAIN_EN			(1 << 2)
#define AGC_USE_FULL_GAIN_TABLE		(1 << 3)
#define AGC_GAIN_UNLOCK_CTRL		(1 << 6)
#define AGC_SOFT_RESET			(1 << 7)


#define AGC_CONFIG_3			0x0FC
#define ADC_OVERRANGE_SAMPLE_SIZE(x)	(((x) & 0x7) << 0)
#define USE_AGC_FOR_LMTLPG_GAIN		(1 << 3)
#define INCDEC_LMT_GAIN			(1 << 4)
#define MANUAL_INCR_STEP_SIZE(x)		(((x) & 0x7) << 5)

#define MAX_LMT_FULL_GAIN		0x0FD
#define	PEAK_WAIT_TIME			0x0FE
#define MANUAL_DECR_STEP_SIZE(x)		(((x) & 0x7) << 5)
#define PEAK_OVERLOAD_WAIT_TIME(x)	(((x) & 0x1F) << 0)

#define	DIGITAL_GAIN			0x100
#define MAX_DIGITAL_GAIN_STEP_MASK	0xE0
#define MAX_DIGITAL_GAIN_MASK		0x1F

#define	AGC_LOCK_LEVEL			0x101
#define	AGC_LOCK_LEVEL_MASK		0x7F

#define	ADC_NOISE_COR_FACTOR		0x102
#define	GAIN_STEP_CONFIG_1		0x103
#define LMT_OVERLOAD_LARGE_INC_SIZE_MASK	0x1C

#define	ADC_SMALL_OVERLOAD_THRES		0x104
#define	ADC_LARGE_OVERLOAD_THRES		0x105
#define	GAIN_STEP_CONFIG_2		0x106
#define LARGE_LPF_GAIN_STEP(x)		(((x) & 0xF) << 0)

#define	LMT_SMALL_OVERLOAD_THRES		0x107
#define	LMT_SMALL_OVERLOAD_THRES_MASK	0x3F

#define	LMT_LARGE_OVERLOAD_THRES		0x108
#define RX1_MANUAL_LMT_FULL_GAIN		0x109
#define RX1_MANUAL_LPF_GAIN		0x10A
#define RX1_MANUAL_DIG_FORCE_GAIN	0x10B
#define RX2_MANUAL_LMT_FULL_GAIN		0x10C
#define RX2_MANUAL_LPF_GAIN		0x10D
#define RX2_MANUAL_DIG_FORCE_GAIN	0x10E
#define FAST_CONFIG_1			0x110
#define FAST_CONFIG_2_DELAY		0x111
#define FAST_CONFIG_2_DELAY_MASK		0x1F

#define FAST_ENERGY_LOST_THRES		0x112
#define FAST_STRONG_SIGNAL_THRES	0x113
#define FAST_LOW_POWER_THRES		0x114
#define FAST_STRONG_SIGNAL_FREEZE	0x115
#define FAST_FINAL_OVER_RANGE		0x116
#define FAST_ENERGY_DETECT_COUNT	0x117
#define FAST_AGCLL_UPPER_LIMIT		0x118
#define FAST_GAINLOCK_EXIT_COUNT	0x119
#define FAST_INI_LMT_GAIN_LIMIT		0x11A
#define FAST_INCREMENT_TIME		0x11B

#define SLOW_AGC_INNER_LOW_THRES		0x120
#define SLOW_AGC_INNER_LOW_THRES_MASK	0x7F
#define ADC_LMT_S_OVR_PREVENT_GAIN_INC	(1 << 7)

#define SLOW_LMT_OVERLOAD_COUNTER	0x121
#define LARGE_LMT_OVERL_EX_CNTR(x)	(((x) & 0xF) << 4)
#define SMALL_LMT_OVERL_EX_CNTR(x)	(((x) & 0xF) << 0)
#define SLOW_ADC_OVERLOAD_COUNTER	0x122
#define LARGE_ADC_OVERL_EX_CNTR(x)	(((x) & 0xF) << 4)
#define SMALL_ADC_OVERL_EX_CNTR(x)	(((x) & 0xF) << 0)


#define SLOW_GAIN_STEP_1		0x123
#define AGC_INNER_HIGH_THRESH_EX_STP(x)	(((x) & 0x7) << 4)
#define AGC_INNER_LOW_THRESH_EX_STP(x)	(((x) & 0x7) << 0)
#define IMMED_GAIN_CHANGE_ADC_OVER	(1 << 3)
#define IMMED_GAIN_CHANGE_LMT_OVER	(1 << 7)


#define SLOW_GAIN_UPDATE_COUNTER1	0x124
#define SLOW_GAIN_UPDATE_COUNTER2	0x125

#define SLOW_DIGITAL_SAT_COUNTER		0x128
#define DIGITAL_SAT_EX_COUNTER(x)	(((x) & 0xF) << 0)
#define SYNC_FOR_GAIN_COUNTER_EN		(1 << 4)
#define DOUBLE_GAIN_COUNTER		(1 << 5)

#define SLOW_OUTER_POWER_THRES		0x129
#define AGC_OUTER_HIGH_THRESH(x)		(((x) & 0xF) << 4)
#define AGC_OUTER_LOW_THRESH(x)		(((x) & 0xF) << 0)


#define SLOW_GAIN_STEP_2		0x12A
#define AGC_OUTER_HIGH_THRESH_EX_STP(x)	(((x) & 0xF) << 4)
#define AGC_OUTER_LOW_THRESH_EX_STP(x)	(((x) & 0xF) << 0)

#define EXT_LNA_HIGH_GAIN		0x12C
#define EXT_LNA_LOW_GAIN		0x12D
#define GAIN_TABLE_ADDRESS		0x130
#define GAIN_TABLE_WRITE_DATA1		0x131
#define GAIN_TABLE_WRITE_DATA2		0x132
#define GAIN_TABLE_WRITE_DATA3		0x133
#define GAIN_TABLE_READ_DATA1		0x134
#define GAIN_TABLE_READ_DATA2		0x135
#define GAIN_TABLE_READ_DATA3		0x136
#define GAIN_TABLE_CONFIG		0x137
#define GM_SUB_TABLE_ADDRESS		0x138
#define GM_SUB_TABLE_GAIN		0x139
#define GM_SUB_TABLE_BIAS_WRITE		0x13A
#define GM_SUB_TABLE_CTL_WRITE		0x13B
#define GM_SUB_TABLE_GAIN_READ		0x13C
#define GM_SUB_TABLE_CONFIG		0x13F

#define MEASURE_DURATION_0_1		0x150
#define MEASURE_DURATION_2_3		0x151
#define WEIGHT_0			0x152
#define WEIGHT_1			0x153
#define WEIGHT_2			0x154
#define WEIGHT_3			0x155
#define RSSI_DELAY			0x156
#define RSSI_WAIT_TIME			0x157
#define RSSI_CONFIG			0x158

#define RSSI_MODE_SEL(x)			(((x) & 0x7) << 2)
#define START_RSSI_MEAS			(1 << 5)
#define DEC_POWER_MEAS_DURATION		0x15C
#define DEC_POWER_MEAS_DURATION_MASK	0xF

#define CALIBRATION_CONFIG_1		0x169
#define CALIBRATION_CONFIG_2		0x16A
#define CALIBRATION_CONFIG_3		0x16B

#define WAIT_COUNT			0x185
#define RF_DC_OFFSET_COUNT		0x186
#define RF_DC_OFFSET_CONFIG_1		0x187
#define RF_DC_OFFSET_ATTEN		0x188
#define RF_TX_PDET_COUNT2		0x189
#define RF_DC_OFFSET_CONFIG_2		0x18B
#define BB_DC_OFFSET_SHIFT		0x190
#define BB_DC_OFFSET_COUNT		0x193
#define BB_DC_OFFSET_ATTEN		0x194

#define RX1_RSSI_SYMBOL			0x1A7
#define RX1_RSSI_PREAMBLE      		0x1A8
#define RX2_RSSI_SYMBOL     		0x1A9
#define RX2_RSSI_PREAMBLE  		0x1AA
#define SYMBOL_LSB  			0x1AB
#define PREAMBLE_LSB  			0x1AC

#define RX_MIX_GM_CONFIG		0x1C0

#define RX_MIX_LO_CM			0x1D5
#define RX_TIA_CONFIG			0x1DB
#define TIA1_C_LSB			0x1DC
#define TIA1_C_MSB			0x1DD
#define TIA2_C_LSB			0x1DE
#define TIA2_C_MSB			0x1DF

#define RX1_TUNE_CONTROL		0x1E2
#define RX2_TUNE_CONTROL		0x1E3
#define RX_BBF_R2346			0x1E6
#define RX_BBF_C1_MSB			0x1E7
#define RX_BBF_C1_LSB			0x1E8
#define RX_BBF_C2_MSB			0x1E9
#define RX_BBF_C2_LSB			0x1EA
#define RX_BBF_C3_MSB			0x1EB
#define RX_BBF_C3_LSB			0x1EC
#define RX_BBF_TUNE_DIVIDE		0x1F8
#define RX_BBF_TUNE_CONFIG		0x1F9
#define RX_BBBW_MHZ			0x1FB
#define RX_BBBW_KHZ			0x1FC
#define RX_PFD_CONFIG			0x230
#define RX_INTEGER_BYTE_0		0x231
#define RX_INTEGER_BYTE_1		0x232
#define RX_FRACTIONAL_BYTE_0		0x233
#define RX_FRACTIONAL_BYTE_1		0x234
#define RX_FRACTIONAL_BYTE_2		0x235
#define RX_FORCE_ALC			0x236
#define RX_FORCE_VCO_TUNE_0		0x237
#define RX_FORCE_VCO_TUNE_1		0x238
#define RX_ALC_VARACTOR			0x239
#define RX_VCO_OUTPUT			0x23A
#define RX_CP_CURRENT			0x23B
#define RX_CP_CONFIG			0x23D
#define RX_LOOP_FILTER_1		0x23E
#define RX_LOOP_FILTER_2		0x23F
#define RX_LOOP_FILTER_3		0x240
#define RX_VCO_BIAS_1			0x242
#define RX_VCO_BIAS_2			0x243
#define RX_CAL_STATUS			0x244
#define RX_VCO_CAL_REF			0x245
#define RX_VCO_PD_OVERRIDES		0x246
#define RX_CP_OVERRANGE_VCOLOCK		0x247
#define RX_VCO_LDO			0x248
#define RX_VCO_CAL			0x249
#define RX_VCO_VARACTOR_CONTROL_0	0x250
#define RX_VCO_VARACTOR_CONTROL_1	0x251
#define RX_FAST_LOCK_SETUP		0x25A
#define RX_FAST_LOCK_SETUP_INIT_DELAY	0x25B
#define RX_FAST_LOCK_PROGRAM_ADDR	0x25C
#define RX_FAST_LOCK_PROGRAM_DATA	0x25D
#define RX_FAST_LOCK_PROGRAM_READ	0x25E
#define RX_FAST_LOCK_PROGRAM_CONTROL	0x25F


#define RX_LO_GEN_POWER_MODE		0x261


#define TX_PFD_CONFIG			0x270
#define TX_INTEGER_BYTE_0		0x271
#define TX_INTEGER_BYTE_1		0x272
#define TX_FRACTIONAL_BYTE_0		0x273
#define TX_FRACTIONAL_BYTE_1		0x274
#define TX_FRACTIONAL_BYTE_2		0x275
#define TX_FORCE_ALC			0x276
#define TX_FORCE_VCO_TUNE_0		0x277
#define TX_FORCE_VCO_TUNE_1		0x278
#define TX_ALC_VARACT_OR		0x279
#define TX_VCO_OUTPUT			0x27A
#define TX_CP_CURRENT			0x27B
#define TX_CP_CONFIG			0x27D
#define TX_LOOP_FILTER_1		0x27E
#define TX_LOOP_FILTER_2		0x27F
#define TX_LOOP_FILTER_3		0x280
#define TX_VCO_BIAS_1			0x282
#define TX_VCO_BIAS_2			0x283
#define TX_CAL_STATUS			0x284
#define TX_VCO_CAL_REF			0x285
#define TX_VCO_PD_OVERRIDES		0x286
#define TX_CP_OVERRANGE_VCOLOCK		0x287
#define TX_VCO_LDO			0x288
#define TX_VCO_CAL			0x289
#define TX_VCO_VARACTOR_CONTROL_0	0x290
#define TX_VCO_VARACTOR_CONTROL_1	0x291
#define DCXO_COARSE_TUNE		0x292
#define DCXO_FINE_TUNE_HIGH		0x293
#define DCXO_FINE_TUNE_LOW		0x294
#define TX_FAST_LOCK_SETUP		0x29A
#define TX_FAST_LOCK_SETUP_INIT_DELAY	0x29B
#define TX_FAST_LOCK_PROGRAM_ADDR	0x29C
#define TX_FAST_LOCK_PROGRAM_DATA	0x29D
#define TX_FAST_LOCK_PROGRAM_READ	0x29E
#define TX_FAST_LOCK_PROGRAM_CONTROL	0x29F
#define TX_LO_GEN_POWER_MODE		0x2A1

#define BANDGAP_CONFIG0          	0x2A6
#define BANDGAP_CONFIG1         	0x2A8
#define REF_DIVIDE_CONFIG_1		0x2AB
#define REF_DIVIDE_CONFIG_2		0x2AC

#define GAIN_RX_1       		0x2B0
#define LPF_GAIN_RX_1     		0x2B1
#define DIG_GAIN_RX_1       		0x2B2
#define FAST_ATTACK_STATE     		0x2B3
#define SLOW_LOOP_STATE    		0x2B4
#define GAIN_RX_2       		0x2B5
#define LPF_GAIN_RX_2       	 	0x2B6
#define DIG_GAIN_RX_2     		0x2B7

#define CONTROL_REGISTER         	0x3DF

#define OBSERVE_CONFIG			0x3F5

#define THB3_DEC3			(BIT(5) | BIT(4))
#define THB2_DEC2			BIT(3)
#define THB1_DEC1			BIT(2)
#define TX_RX_FIR			(BIT(1) | BIT(0))

#define RX_VCO_DIV			(0xF << 0)
#define TX_VCO_DIV			(0xF << 4)

#define MAX_MBYTE_SPI			8

#define RFPLL_MODULUS			8388593UL
#define BBPLL_MODULUS			2088960UL

#define MIN_VCO_FREQ_HZ		6000000000
#define MAX_CARRIER_FREQ_HZ	6000000000
#define MIN_CARRIER_FREQ_HZ	47000000

#define MASK_VCO_OUTPUT			0x0F
#define MASK_VCO_VARACTOR		0x0F
#define MASK_VCO_BIAS_REF		0x07
#define MASK_VCO_BIAS_TCF		0x18

#define MASK_FORCE_VCO_TUNE1		0x78
#define MASK_VCO_VARACTOR_CONTROL1	0x0F
#define MASK_CP_CURRENT			0x3F
#define MASK_LOOP_FILTER3		0x0F

/* Calibration status Registers */
#define REG_CH1_OVERFLOW	0x05E
#define REG_RX_CAL_STATUS	0x244
#define REG_TX_CAL_STATUS	0x284
#define REG_CALIBRATION_CONTROL	0x016
#define REG_CALIBRATION_CONFIG1	0x169
#define REG_RX_CP_CONFIG	0x23D
#define REG_TX_CP_CONFIG	0x27D

#define VAL_CAL_CONF1_TRACKOFF	0xC0

/* Calibration status masks */
#define MASK_BBPLL_LOCK		0x80
#define MASK_RX_CP_CAL_VALID	0x80
#define MASK_TX_CP_CAL_VALID	0x80
#define MASK_RX_BB_TUNE		0x80
#define MASK_TX_BB_TUNE		0x40
#define MASK_DC_CAL_BBSTART	0x00
#define MASK_DC_CAL_RFSTART	0x01
#define MASK_TXQUAD_CAL		0x10

/*
 * 24 = 0x18 which is the AuxDAC1 word address
 * 26 = 0x1A which is the AuxDAC1 config address
 */
#define DAC1_WORD 	24
#define DAC1_CONFIG  	26

/* TX Attenuation Registers */
#define TX1_ATTEN0	0x073
#define TX1_ATTEN1	0x074
#define TX2_ATTEN0	0x075
#define TX2_ATTEN1	0x076
#define ATTEN0_MASK	0x000000FF
#define ATTEN1_MASK	0x00000100
#define ATTEN_MSB_BIT_MASK	0x01
#define MSB_SHIFT	8

#define TX_ENABLE	0x01
#define TX_DISABLE	0x00
#define TX_ENABLE_REG	0x002
#define TX1_ENABLE_MASK	0x40
#define TX2_ENABLE_MASK	0x80

#define RX_ENABLE	0x01
#define RX_DISABLE	0x00
#define RX_ENABLE_REG	0x003
#define RX1_ENABLE_MASK	0x40
#define RX2_ENABLE_MASK	0x80

#define RSSI_READBACK_REG 0x1A7
#define RSSI_CONFIG_REG	0x158
#define RSSI_DELAY_REG	0x156
#define RSSI_MEAS_DUR_10_REG	0x150
#define RSSI_MEAS_DUR_32_REG	0x151
#define RSSI_WEIGHT0_REG	0x152
#define RSSI_WEIGHT1_REG	0x153
#define RSSI_WEIGHT2_REG	0x154
#define RSSI_WEIGHT3_REG	0x155

#define RSSI_MAX_WEIGHT		255
#define RSSI_MEAS_MODE_MASK	0xE2
#define RSSI_GAIN_CHANGE_EN_AGC_MODE	0x14
/* RSSI delay reg value is decremented by RX sample rate divided by 8*/
#define RSSI_DELAY_5MHZ		(256 / 8)
#define RSSI_DELAY_10MHZ	(512 / 8)
#define RSSI_DELAY_15MHZ	(768 / 8)
#define RSSI_DELAY_20MHZ	(1024 / 8)

#define SUBFRAME_SIZE_5MHZ	7680
#define SUBFRAME_SIZE_10MHZ	15360
#define SUBFRAME_SIZE_15MHZ	23040
#define SUBFRAME_SIZE_20MHZ	30720

/* For 9 bit RSSI symbol/preamble value RSSI is equivalent to 0.25dB/LSB.
 * Since we can not do floating pt operations in kernel, we multiply
 * Resolution with RSSI_MULTIPLER, and pass this multipler to user space
 * which can convert rssi to floating pt again.
 */
#define RSSI_MULTIPLIER	100
#define RSSI_RESOLUTION	((int) (0.25 * RSSI_MULTIPLIER))
#define LSB_SHIFT	1
#define RSSI_LSB_MASK1	0x01
#define RSSI_LSB_MASK2	0x02

#define REG_RXEN_N_FILTER_CTRL		0x003
#define RX1_EN				0x40
#define RX2_EN				0x80

/* Gain index read/write register for MGC */
#define REG_RX1_MGC_FULL_TBL_IDX	0x109
#define REG_RX2_MGC_FULL_TBL_IDX	0x10c

/* Gain Index read back registers. They are
 * applicable for both AGC and MGC
 */
#define REG_RX1_FULL_TBL_IDX		0x2B0
#define REG_RX1_LPF_GAIN_IDX		0x2B1
#define REG_RX1_DIG_GAIN_IDX		0x2B2
#define REG_RX2_FULL_TBL_IDX		0x2B5
#define REG_RX2_LPF_GAIN_IDX		0x2B6
#define REG_RX2_DIG_GAIN_IDX		0x2B7
#define FULL_TBL_IDX_MASK		0x7f
#define LPF_IDX_MASK			0x1f
#define DIGITAL_IDX_MASK		0x1f
#define MAX_LMT_INDEX			40
#define MAX_LPF_GAIN			24
#define MAX_DIG_GAIN			31

#define REG_GAIN_TBL_ADDR		0x130
#define REG_GAIN_TBL_READ_DATA1		0x134
#define REG_GAIN_TBL_READ_DATA2		0x135
#define REG_GAIN_TBL_READ_DATA3		0x136
#define LNA_GAIN_MASK			0x60
#define MIXER_GAIN_MASK			0x1F
#define TIA_GAIN_MASK			0x20
#define LNA_SHIFT			5
#define MIXER_SHIFT			0
#define TIA_SHIFT			5

/*Fast attack state register*/
#define REG_FAST_ATK_STATE		0x2B3
#define FAST_ATK_MASK			0x7
#define RX1_FAST_ATK_SHIFT		0
#define RX2_FAST_ATK_SHIFT		4

/*Fast attack state machine states*/
#define FAST_ATK_RESET			0
#define FAST_ATK_PEAK_DETECT		1
#define FAST_ATK_PWR_MEASURE		2
#define FAST_ATK_FINAL_SETTELING	3
#define FAST_ATK_FINAL_OVER		4
#define FAST_ATK_GAIN_LOCKED		5

#define REG_AGC_CONF1			0x0FA

#define RX_GAIN_CTL_MASK		0x03
#define RX2_GAIN_CTL_SHIFT		2
#define RX1_GAIN_CTL_SHIFT		0

#define RX_GAIN_CTL_MGC				0x00
#define RX_GAIN_CTL_AGC_FAST_ATK		0x01
#define RX_GAIN_CTL_AGC_SLOW_ATK		0x02
#define RX_GAIN_CTL_AGC_SLOW_ATK_HYBD		0x03
#define SLOW_ATK_HYBD_BIT_EN			0x10

#define REG_AGC_CONF2			0x0FB
#define FULL_GAIN_TBL			0x08
#define DIGITAL_GAIN_EN			0x04

#define RXGAIN_FULL_TBL_MAX_IDX		90
#define RXGAIN_SPLIT_TBL_MAX_IDX	40


/*ENSM config1 register*/
#define REG_ENSM_CONF1			0x014
#define ENSM_CONF1_TO_ALERT		(1 << 0)
#define ENSM_CONF1_AUTO_GAIN_LOCK	(1 << 1)
#define ENSM_CONF1_FORCE_ALERT		(1 << 2)
#define ENSM_CONF1_LEVEL_MODE		(1 << 3)
#define ENSM_CONF1_ENSM_PIN_CTL_EN	(1 << 4)
#define ENSM_CONF1_FORCE_TX_ON		(1 << 5)
#define ENSM_CONF1_FORCE_RX_ON		(1 << 6)
#define ENSM_CONF_RX_EN_CAL		(1 << 7)

/*ENSM state - Read only*/
#define REG_DEV_STATE			0x017
#define ENSM_STATE_SHIFT		0x0
#define ENSM_STATE_MASK			0x0f

#define ENSM_STATE_SLEEP_WAIT		0x0
#define ENSM_STATE_ALERT		0x5
#define ENSM_STATE_TX			0x6
#define ENSM_STATE_TX_FLUSH		0x7
#define ENSM_STATE_RX			0x8
#define ENSM_STATE_RX_FLUSH		0x9
#define ENSM_STATE_FDD			0xA
#define ENSM_STATE_FDD_FLUSH		0xB
#define ENSM_STATE_INVALID		0xff

#define MIN_BBPLL_FREQ			715000000UL /* 715 MHz */
#define MAX_BBPLL_FREQ			1430000000UL /* 1430 MHz */
#define MAX_BBPLL_DIV			64
#define MIN_BBPLL_DIV			2

#define MIN_ADC_CLK			10500000UL /* 10.5MHz */
#define MAX_ADC_CLK			672000000UL /* 672 MHz */
#define MAX_DAC_CLK			(MAX_ADC_CLK / 2)

/* CALIBRATION_CONTROL */
#define RX_BB_TUNE_CAL		(1 << 7)
#define TX_BB_TUNE_CAL		(1 << 6)
#define RX_QUAD_CAL		(1 << 5)
#define TX_QUAD_CAL		(1 << 4)
#define RX_GAIN_STEP_CAL		(1 << 3)
#define TXMON_CAL		(1 << 2)
#define RFDC_CAL			(1 << 1)
#define BBDC_CAL			(1 << 0)

#define TX_FIR_GAIN_6DB		(1 << 0)
#define FIR_START_CLK		(1 << 1)
#define FIR_WRITE		(1 << 2)
#define FIR_SELECT(x)		(((x) & 0x3) << 3)
#define FIR_NUM_TAPS(x)		(((x) & 0x7) << 5)

struct SynthLUT {
	unsigned short VCO_MHz;
	unsigned char VCO_Output_Level;
	unsigned char VCO_Varactor;
	unsigned char VCO_Bias_Ref;
	unsigned char VCO_Bias_Tcf;
	unsigned char VCO_Cal_Offset;
	unsigned char VCO_Varactor_Reference;
	unsigned char Charge_Pump_Current;
	unsigned char LF_C2;
	unsigned char LF_C1;
	unsigned char LF_R1;
	unsigned char LF_C3;
	unsigned char LF_R3;
};

enum {
	LUT_FTDD_40,
	LUT_FTDD_60,
	LUT_FTDD_80,
	LUT_FTDD_ENT,
};

enum rx_gain_table_type {
	RXGAIN_FULL_TBL,
	RXGAIN_SPLIT_TBL,
};

enum rx_gain_table_name {
	TBL_200_1300_MHZ,
	TBL_1300_4000_MHZ,
	TBL_4000_6000_MHZ,
	RXGAIN_TBLS_END,
};

enum fir_dest {
	FIR_TX1 = 0x01,
	FIR_TX2 = 0x02,
	FIR_TX1_TX2 = 0x03,
	FIR_RX1 = 0x81,
	FIR_RX2 = 0x82,
	FIR_RX1_RX2 = 0x83,
	FIR_IS_RX = 0x80,
};

#define SYNTH_LUT_SIZE	53

const struct SynthLUT SynthLUT_FDD[LUT_FTDD_ENT][SYNTH_LUT_SIZE] = {
{
	{12605, 10, 0, 4, 0, 15, 8, 8, 12, 3, 14, 15, 11}, /* 40 MHz */
	{12245, 10, 0, 4, 0, 15, 8, 9, 12, 3, 14, 15, 11},
	{11906, 10, 0, 4, 0, 15, 8, 9, 12, 3, 14, 15, 11},
	{11588, 10, 0, 4, 0, 15, 8, 10, 12, 3, 14, 15, 11},
	{11288, 10, 0, 4, 0, 15, 8, 11, 12, 3, 14, 15, 11},
	{11007, 10, 0, 4, 0, 15, 8, 11, 12, 3, 14, 15, 11},
	{10742, 10, 0, 4, 0, 14, 8, 12, 12, 3, 14, 15, 11},
	{10492, 10, 0, 5, 1, 14, 9, 13, 12, 3, 14, 15, 11},
	{10258, 10, 0, 5, 1, 14, 9, 13, 12, 3, 14, 15, 11},
	{10036, 10, 0, 5, 1, 14, 9, 14, 12, 3, 14, 15, 11},
	{9827, 10, 0, 5, 1, 14, 9, 15, 12, 3, 14, 15, 11},
	{9631, 10, 0, 5, 1, 14, 9, 15, 12, 3, 14, 15, 11},
	{9445, 10, 0, 5, 1, 14, 9, 16, 12, 3, 14, 15, 11},
	{9269, 10, 0, 5, 1, 14, 9, 17, 12, 3, 14, 15, 11},
	{9103, 10, 0, 5, 1, 14, 9, 17, 12, 3, 14, 15, 11},
	{8946, 10, 0, 5, 1, 14, 9, 18, 12, 3, 14, 15, 11},
	{8797, 10, 1, 6, 1, 15, 11, 13, 12, 3, 14, 15, 11},
	{8655, 10, 1, 6, 1, 15, 11, 14, 12, 3, 14, 15, 11},
	{8520, 10, 1, 6, 1, 15, 11, 14, 12, 3, 14, 15, 11},
	{8392, 10, 1, 6, 1, 15, 11, 15, 12, 3, 14, 15, 11},
	{8269, 10, 1, 6, 1, 15, 11, 15, 12, 3, 14, 15, 11},
	{8153, 10, 1, 6, 1, 15, 11, 16, 12, 3, 14, 15, 11},
	{8041, 10, 1, 6, 1, 15, 11, 16, 12, 3, 14, 15, 11},
	{7934, 10, 1, 6, 1, 15, 11, 17, 12, 3, 14, 15, 11},
	{7831, 10, 1, 6, 1, 15, 11, 17, 12, 3, 14, 15, 11},
	{7733, 10, 1, 6, 1, 15, 11, 17, 12, 3, 14, 15, 11},
	{7638, 10, 1, 6, 1, 15, 11, 18, 12, 3, 14, 15, 11},
	{7547, 10, 1, 6, 1, 15, 11, 18, 12, 3, 14, 15, 11},
	{7459, 10, 1, 6, 1, 15, 11, 19, 12, 3, 14, 15, 11},
	{7374, 10, 1, 7, 2, 15, 12, 19, 12, 3, 14, 15, 11},
	{7291, 10, 1, 7, 2, 15, 12, 20, 12, 3, 14, 15, 11},
	{7212, 10, 1, 7, 2, 15, 12, 20, 12, 3, 14, 15, 11},
	{7135, 10, 1, 7, 2, 15, 14, 21, 12, 3, 14, 15, 11},
	{7061, 10, 1, 7, 2, 15, 14, 21, 12, 3, 14, 15, 11},
	{6988, 10, 1, 7, 2, 15, 14, 22, 12, 3, 14, 15, 11},
	{6918, 10, 1, 7, 2, 15, 14, 22, 12, 3, 14, 15, 11},
	{6850, 10, 1, 7, 2, 15, 14, 23, 12, 3, 14, 15, 11},
	{6784, 10, 1, 7, 2, 15, 14, 23, 12, 3, 14, 15, 11},
	{6720, 10, 1, 7, 2, 15, 14, 24, 12, 3, 14, 15, 11},
	{6658, 10, 1, 7, 2, 15, 14, 24, 12, 3, 14, 15, 11},
	{6597, 10, 1, 7, 2, 15, 14, 25, 12, 3, 14, 15, 11},
	{6539, 10, 1, 7, 2, 15, 14, 25, 12, 3, 14, 15, 11},
	{6482, 10, 1, 7, 2, 15, 14, 26, 12, 3, 14, 15, 11},
	{6427, 10, 1, 7, 2, 15, 14, 26, 12, 3, 14, 15, 11},
	{6373, 10, 3, 7, 3, 15, 12, 17, 12, 3, 14, 15, 11},
	{6321, 10, 3, 7, 3, 15, 12, 17, 12, 3, 14, 15, 11},
	{6270, 10, 3, 7, 3, 15, 12, 17, 12, 3, 14, 15, 11},
	{6222, 10, 3, 7, 3, 15, 12, 18, 12, 3, 14, 15, 11},
	{6174, 10, 3, 7, 3, 15, 12, 18, 12, 3, 14, 15, 11},
	{6128, 10, 3, 7, 3, 15, 12, 18, 12, 3, 14, 15, 11},
	{6083, 10, 3, 7, 3, 15, 12, 18, 12, 3, 14, 15, 11},
	{6040, 10, 3, 7, 3, 15, 12, 19, 12, 3, 14, 15, 11},
	{5997, 10, 3, 7, 3, 15, 12, 19, 12, 3, 14, 15, 11},
}, {
	{12605, 10, 0, 4, 0, 15, 8, 10, 15, 4, 13, 15, 10},  /* 60 MHz */
	{12245, 10, 0, 4, 0, 15, 8, 11, 15, 4, 13, 15, 10},
	{11906, 10, 0, 4, 0, 15, 8, 11, 15, 4, 13, 15, 10},
	{11588, 10, 0, 4, 0, 15, 8, 12, 15, 4, 13, 15, 10},
	{11288, 10, 0, 4, 0, 15, 8, 13, 15, 4, 13, 15, 10},
	{11007, 10, 0, 4, 0, 14, 8, 14, 15, 4, 13, 15, 10},
	{10742, 10, 0, 4, 0, 14, 8, 15, 15, 4, 13, 15, 10},
	{10492, 10, 0, 5, 1, 14, 9, 15, 15, 4, 13, 15, 10},
	{10258, 10, 0, 5, 1, 14, 9, 16, 15, 4, 13, 15, 10},
	{10036, 10, 0, 5, 1, 14, 9, 17, 15, 4, 13, 15, 10},
	{9827, 10, 0, 5, 1, 14, 9, 18, 15, 4, 13, 15, 10},
	{9631, 10, 0, 5, 1, 14, 9, 19, 15, 4, 13, 15, 10},
	{9445, 10, 0, 5, 1, 14, 9, 19, 15, 4, 13, 15, 10},
	{9269, 10, 0, 5, 1, 14, 9, 20, 15, 4, 13, 15, 10},
	{9103, 10, 0, 5, 1, 13, 9, 21, 15, 4, 13, 15, 10},
	{8946, 10, 0, 5, 1, 13, 9, 22, 15, 4, 13, 15, 10},
	{8797, 10, 1, 6, 1, 15, 11, 16, 15, 4, 13, 15, 10},
	{8655, 10, 1, 6, 1, 15, 11, 17, 15, 4, 13, 15, 10},
	{8520, 10, 1, 6, 1, 15, 11, 17, 15, 4, 13, 15, 10},
	{8392, 10, 1, 6, 1, 15, 11, 18, 15, 4, 13, 15, 10},
	{8269, 10, 1, 6, 1, 15, 11, 18, 15, 4, 13, 15, 10},
	{8153, 10, 1, 6, 1, 15, 11, 19, 15, 4, 13, 15, 10},
	{8041, 10, 1, 6, 1, 15, 11, 19, 15, 4, 13, 15, 10},
	{7934, 10, 1, 6, 1, 15, 11, 20, 15, 4, 13, 15, 10},
	{7831, 10, 1, 6, 1, 15, 11, 21, 15, 4, 13, 15, 10},
	{7733, 10, 1, 6, 1, 15, 11, 21, 15, 4, 13, 15, 10},
	{7638, 10, 1, 6, 1, 15, 11, 22, 15, 4, 13, 15, 10},
	{7547, 10, 1, 6, 1, 15, 11, 22, 15, 4, 13, 15, 10},
	{7459, 10, 1, 6, 1, 15, 11, 23, 15, 4, 13, 15, 10},
	{7374, 10, 1, 7, 2, 15, 12, 23, 15, 4, 13, 15, 10},
	{7291, 10, 1, 7, 2, 15, 12, 24, 15, 4, 13, 15, 10},
	{7212, 10, 1, 7, 2, 15, 12, 25, 15, 4, 13, 15, 10},
	{7135, 10, 1, 7, 2, 15, 14, 25, 15, 4, 13, 15, 10},
	{7061, 10, 1, 7, 2, 15, 14, 26, 15, 4, 13, 15, 10},
	{6988, 10, 1, 7, 2, 15, 14, 26, 15, 4, 13, 15, 10},
	{6918, 10, 1, 7, 2, 15, 14, 27, 15, 4, 13, 15, 10},
	{6850, 10, 1, 7, 2, 15, 14, 27, 15, 4, 13, 15, 10},
	{6784, 10, 1, 7, 2, 15, 14, 28, 15, 4, 13, 15, 10},
	{6720, 10, 1, 7, 2, 15, 14, 29, 15, 4, 13, 15, 10},
	{6658, 10, 1, 7, 2, 15, 14, 29, 15, 4, 13, 15, 10},
	{6597, 10, 1, 7, 2, 15, 14, 30, 15, 4, 13, 15, 10},
	{6539, 10, 1, 7, 2, 15, 14, 30, 15, 4, 13, 15, 10},
	{6482, 10, 1, 7, 2, 15, 14, 31, 15, 4, 13, 15, 10},
	{6427, 10, 1, 7, 2, 15, 14, 32, 15, 4, 13, 15, 10},
	{6373, 10, 3, 7, 3, 15, 12, 20, 15, 4, 13, 15, 10},
	{6321, 10, 3, 7, 3, 15, 12, 21, 15, 4, 13, 15, 10},
	{6270, 10, 3, 7, 3, 15, 12, 21, 15, 4, 13, 15, 10},
	{6222, 10, 3, 7, 3, 15, 12, 21, 15, 4, 13, 15, 10},
	{6174, 10, 3, 7, 3, 15, 12, 22, 15, 4, 13, 15, 10},
	{6128, 10, 3, 7, 3, 15, 12, 22, 15, 4, 13, 15, 10},
	{6083, 10, 3, 7, 3, 15, 12, 22, 15, 4, 13, 15, 10},
	{6040, 10, 3, 7, 3, 15, 12, 23, 15, 4, 13, 15, 10},
	{5997, 10, 3, 7, 3, 15, 12, 23, 15, 4, 13, 15, 10},
}, {
	{12605, 10, 0, 4, 0, 15, 8, 8, 13, 4, 13, 15, 9},   /* 80 MHz */
	{12245, 10, 0, 4, 0, 15, 8, 9, 13, 4, 13, 15, 9},
	{11906, 10, 0, 4, 0, 15, 8, 10, 13, 4, 13, 15, 9},
	{11588, 10, 0, 4, 0, 15, 8, 11, 13, 4, 13, 15, 9},
	{11288, 10, 0, 4, 0, 15, 8, 11, 13, 4, 13, 15, 9},
	{11007, 10, 0, 4, 0, 14, 8, 12, 13, 4, 13, 15, 9},
	{10742, 10, 0, 4, 0, 14, 8, 13, 13, 4, 13, 15, 9},
	{10492, 10, 0, 5, 1, 14, 9, 13, 13, 4, 13, 15, 9},
	{10258, 10, 0, 5, 1, 14, 9, 14, 13, 4, 13, 15, 9},
	{10036, 10, 0, 5, 1, 14, 9, 15, 13, 4, 13, 15, 9},
	{9827, 10, 0, 5, 1, 14, 9, 15, 13, 4, 13, 15, 9},
	{9631, 10, 0, 5, 1, 13, 9, 16, 13, 4, 13, 15, 9},
	{9445, 10, 0, 5, 1, 13, 9, 17, 13, 4, 13, 15, 9},
	{9269, 10, 0, 5, 1, 13, 9, 18, 13, 4, 13, 15, 9},
	{9103, 10, 0, 5, 1, 13, 9, 18, 13, 4, 13, 15, 9},
	{8946, 10, 0, 5, 1, 13, 9, 19, 13, 4, 13, 15, 9},
	{8797, 10, 1, 6, 1, 15, 11, 14, 13, 4, 13, 15, 9},
	{8655, 10, 1, 6, 1, 15, 11, 14, 13, 4, 13, 15, 9},
	{8520, 10, 1, 6, 1, 15, 11, 15, 13, 4, 13, 15, 9},
	{8392, 10, 1, 6, 1, 15, 11, 15, 13, 4, 13, 15, 9},
	{8269, 10, 1, 6, 1, 15, 11, 16, 13, 4, 13, 15, 9},
	{8153, 10, 1, 6, 1, 15, 11, 16, 13, 4, 13, 15, 9},
	{8041, 10, 1, 6, 1, 15, 11, 17, 13, 4, 13, 15, 9},
	{7934, 10, 1, 6, 1, 15, 11, 17, 13, 4, 13, 15, 9},
	{7831, 10, 1, 6, 1, 15, 11, 18, 13, 4, 13, 15, 9},
	{7733, 10, 1, 6, 1, 15, 11, 18, 13, 4, 13, 15, 9},
	{7638, 10, 1, 6, 1, 15, 11, 19, 13, 4, 13, 15, 9},
	{7547, 10, 1, 6, 1, 15, 11, 19, 13, 4, 13, 15, 9},
	{7459, 10, 1, 6, 1, 15, 11, 20, 13, 4, 13, 15, 9},
	{7374, 10, 1, 7, 2, 15, 12, 20, 13, 4, 13, 15, 9},
	{7291, 10, 1, 7, 2, 15, 12, 21, 13, 4, 13, 15, 9},
	{7212, 10, 1, 7, 2, 15, 12, 21, 13, 4, 13, 15, 9},
	{7135, 10, 1, 7, 2, 15, 14, 22, 13, 4, 13, 15, 9},
	{7061, 10, 1, 7, 2, 15, 14, 22, 13, 4, 13, 15, 9},
	{6988, 10, 1, 7, 2, 15, 14, 23, 13, 4, 13, 15, 9},
	{6918, 10, 1, 7, 2, 15, 14, 23, 13, 4, 13, 15, 9},
	{6850, 10, 1, 7, 2, 15, 14, 24, 13, 4, 13, 15, 9},
	{6784, 10, 1, 7, 2, 15, 14, 24, 13, 4, 13, 15, 9},
	{6720, 10, 1, 7, 2, 15, 14, 25, 13, 4, 13, 15, 9},
	{6658, 10, 1, 7, 2, 15, 14, 25, 13, 4, 13, 15, 9},
	{6597, 10, 1, 7, 2, 15, 14, 26, 13, 4, 13, 15, 9},
	{6539, 10, 1, 7, 2, 15, 14, 26, 13, 4, 13, 15, 9},
	{6482, 10, 1, 7, 2, 15, 14, 27, 13, 4, 13, 15, 9},
	{6427, 10, 1, 7, 2, 15, 14, 27, 13, 4, 13, 15, 9},
	{6373, 10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9},
	{6321, 10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9},
	{6270, 10, 3, 7, 3, 15, 12, 18, 13, 4, 13, 15, 9},
	{6222, 10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9},
	{6174, 10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9},
	{6128, 10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9},
	{6083, 10, 3, 7, 3, 15, 12, 19, 13, 4, 13, 15, 9},
	{6040, 10, 3, 7, 3, 15, 12, 20, 13, 4, 13, 15, 9},
	{5997, 10, 3, 7, 3, 15, 12, 20, 13, 4, 13, 15, 9},
}};

struct SynthLUT SynthLUT_TDD[LUT_FTDD_ENT][SYNTH_LUT_SIZE] = {
{
	{12605, 13, 0, 4, 0, 14, 0, 10, 12, 3, 14, 15, 11},   /* 40 MHz */
	{12245, 13, 0, 4, 0, 13, 0, 10, 12, 3, 14, 15, 11},
	{11906, 13, 0, 4, 0, 12, 0, 11, 12, 3, 14, 15, 11},
	{11588, 13, 0, 4, 0, 12, 0, 12, 12, 3, 14, 15, 11},
	{11288, 13, 0, 4, 0, 12, 0, 13, 12, 3, 14, 15, 11},
	{11007, 13, 0, 4, 0, 11, 0, 13, 12, 3, 14, 15, 11},
	{10742, 13, 0, 4, 0, 11, 0, 14, 12, 3, 14, 15, 11},
	{10492, 13, 0, 5, 1, 11, 0, 15, 12, 3, 14, 15, 11},
	{10258, 13, 0, 5, 1, 10, 0, 16, 12, 3, 14, 15, 11},
	{10036, 13, 0, 5, 1, 10, 0, 17, 12, 3, 14, 15, 11},
	{9827, 13, 0, 5, 1, 10, 0, 17, 12, 3, 14, 15, 11},
	{9631, 13, 0, 5, 1, 10, 0, 18, 12, 3, 14, 15, 11},
	{9445, 13, 0, 5, 1, 9, 0, 19, 12, 3, 14, 15, 11},
	{9269, 10, 0, 6, 1, 12, 0, 17, 12, 3, 14, 15, 11},
	{9103, 10, 0, 6, 1, 12, 0, 17, 12, 3, 14, 15, 11},
	{8946, 10, 0, 6, 1, 11, 0, 18, 12, 3, 14, 15, 11},
	{8797, 10, 0, 6, 1, 11, 0, 19, 12, 3, 14, 15, 11},
	{8655, 10, 0, 6, 1, 11, 0, 19, 12, 3, 14, 15, 11},
	{8520, 10, 0, 6, 1, 11, 0, 20, 12, 3, 14, 15, 11},
	{8392, 10, 0, 6, 1, 11, 0, 21, 12, 3, 14, 15, 11},
	{8269, 10, 0, 6, 1, 11, 0, 21, 12, 3, 14, 15, 11},
	{8153, 10, 0, 6, 1, 11, 0, 22, 12, 3, 14, 15, 11},
	{8041, 10, 0, 6, 1, 11, 0, 23, 12, 3, 14, 15, 11},
	{7934, 10, 0, 6, 1, 11, 0, 23, 12, 3, 14, 15, 11},
	{7831, 10, 0, 6, 1, 10, 0, 24, 12, 3, 14, 15, 11},
	{7733, 10, 0, 6, 1, 10, 0, 25, 12, 3, 14, 15, 11},
	{7638, 10, 0, 6, 1, 10, 0, 25, 12, 3, 14, 15, 11},
	{7547, 10, 0, 6, 1, 10, 0, 26, 12, 3, 14, 15, 11},
	{7459, 10, 0, 6, 1, 10, 0, 27, 12, 3, 14, 15, 11},
	{7374, 10, 0, 7, 2, 10, 0, 27, 12, 3, 14, 15, 11},
	{7291, 10, 0, 7, 2, 9, 0, 28, 12, 3, 14, 15, 11},
	{7212, 10, 0, 7, 2, 9, 0, 29, 12, 3, 14, 15, 11},
	{7135, 10, 0, 7, 2, 9, 0, 29, 12, 3, 14, 15, 11},
	{7061, 10, 0, 7, 2, 9, 0, 30, 12, 3, 14, 15, 11},
	{6988, 10, 0, 7, 2, 9, 0, 31, 12, 3, 14, 15, 11},
	{6918, 10, 0, 7, 2, 9, 0, 31, 12, 3, 14, 15, 11},
	{6850, 10, 0, 7, 2, 8, 0, 32, 12, 3, 14, 15, 11},
	{6784, 10, 0, 7, 2, 8, 0, 33, 12, 3, 14, 15, 11},
	{6720, 10, 0, 7, 2, 8, 0, 33, 12, 3, 14, 15, 11},
	{6658, 10, 0, 7, 2, 8, 0, 34, 12, 3, 14, 15, 11},
	{6597, 10, 0, 7, 2, 8, 0, 35, 12, 3, 14, 15, 11},
	{6539, 10, 0, 7, 2, 8, 0, 35, 12, 3, 14, 15, 11},
	{6482, 10, 0, 7, 2, 8, 0, 36, 12, 3, 14, 15, 11},
	{6427, 10, 0, 7, 2, 7, 0, 37, 12, 3, 14, 15, 11},
	{6373, 10, 0, 7, 3, 7, 0, 37, 12, 3, 14, 15, 11},
	{6321, 10, 0, 7, 3, 7, 0, 38, 12, 3, 14, 15, 11},
	{6270, 10, 0, 7, 3, 7, 0, 39, 12, 3, 14, 15, 11},
	{6222, 10, 0, 7, 3, 7, 0, 39, 12, 3, 14, 15, 11},
	{6174, 10, 0, 7, 3, 7, 0, 40, 12, 3, 14, 15, 11},
	{6128, 10, 0, 7, 3, 7, 0, 41, 12, 3, 14, 15, 11},
	{6083, 10, 0, 7, 3, 7, 0, 41, 12, 3, 14, 15, 11},
	{6040, 10, 0, 7, 3, 6, 0, 42, 12, 3, 14, 15, 11},
	{5997, 10, 0, 7, 3, 6, 0, 42, 12, 3, 14, 15, 11},
}, {
	{12605, 13, 0, 4, 0, 14, 0, 12, 15, 4, 13, 15, 10},   /* 60 MHz */
	{12245, 13, 0, 4, 0, 13, 0, 13, 15, 4, 13, 15, 10},
	{11906, 13, 0, 4, 0, 13, 0, 14, 15, 4, 13, 15, 10},
	{11588, 13, 0, 4, 0, 12, 0, 14, 15, 4, 13, 15, 10},
	{11288, 13, 0, 4, 0, 12, 0, 15, 15, 4, 13, 15, 10},
	{11007, 13, 0, 4, 0, 12, 0, 16, 15, 4, 13, 15, 10},
	{10742, 13, 0, 4, 0, 11, 0, 17, 15, 4, 13, 15, 10},
	{10492, 13, 0, 5, 1, 11, 0, 18, 15, 4, 13, 15, 10},
	{10258, 13, 0, 5, 1, 11, 0, 19, 15, 4, 13, 15, 10},
	{10036, 13, 0, 5, 1, 10, 0, 20, 15, 4, 13, 15, 10},
	{9827, 13, 0, 5, 1, 10, 0, 21, 15, 4, 13, 15, 10},
	{9631, 13, 0, 5, 1, 10, 0, 22, 15, 4, 13, 15, 10},
	{9445, 13, 0, 5, 1, 10, 0, 23, 15, 4, 13, 15, 10},
	{9269, 10, 0, 6, 1, 12, 0, 20, 15, 4, 13, 15, 10},
	{9103, 10, 0, 6, 1, 12, 0, 21, 15, 4, 13, 15, 10},
	{8946, 10, 0, 6, 1, 12, 0, 22, 15, 4, 13, 15, 10},
	{8797, 10, 0, 6, 1, 12, 0, 23, 15, 4, 13, 15, 10},
	{8655, 10, 0, 6, 1, 12, 0, 23, 15, 4, 13, 15, 10},
	{8520, 10, 0, 6, 1, 12, 0, 24, 15, 4, 13, 15, 10},
	{8392, 10, 0, 6, 1, 12, 0, 25, 15, 4, 13, 15, 10},
	{8269, 10, 0, 6, 1, 12, 0, 26, 15, 4, 13, 15, 10},
	{8153, 10, 0, 6, 1, 12, 0, 27, 15, 4, 13, 15, 10},
	{8041, 10, 0, 6, 1, 12, 0, 27, 15, 4, 13, 15, 10},
	{7934, 10, 0, 6, 1, 11, 0, 28, 15, 4, 13, 15, 10},
	{7831, 10, 0, 6, 1, 11, 0, 29, 15, 4, 13, 15, 10},
	{7733, 10, 0, 6, 1, 11, 0, 30, 15, 4, 13, 15, 10},
	{7638, 10, 0, 6, 1, 11, 0, 31, 15, 4, 13, 15, 10},
	{7547, 10, 0, 6, 1, 11, 0, 31, 15, 4, 13, 15, 10},
	{7459, 10, 0, 6, 1, 11, 0, 32, 15, 4, 13, 15, 10},
	{7374, 10, 0, 7, 2, 11, 0, 33, 15, 4, 13, 15, 10},
	{7291, 10, 0, 7, 2, 11, 0, 34, 15, 4, 13, 15, 10},
	{7212, 10, 0, 7, 2, 11, 0, 35, 15, 4, 13, 15, 10},
	{7135, 10, 0, 7, 2, 10, 0, 35, 15, 4, 13, 15, 10},
	{7061, 10, 0, 7, 2, 10, 0, 36, 15, 4, 13, 15, 10},
	{6988, 10, 0, 7, 2, 10, 0, 37, 15, 4, 13, 15, 10},
	{6918, 10, 0, 7, 2, 10, 0, 38, 15, 4, 13, 15, 10},
	{6850, 10, 0, 7, 2, 10, 0, 39, 15, 4, 13, 15, 10},
	{6784, 10, 0, 7, 2, 10, 0, 39, 15, 4, 13, 15, 10},
	{6720, 10, 0, 7, 2, 10, 0, 40, 15, 4, 13, 15, 10},
	{6658, 10, 0, 7, 2, 9, 0, 41, 15, 4, 13, 15, 10},
	{6597, 10, 0, 7, 2, 9, 0, 42, 15, 4, 13, 15, 10},
	{6539, 10, 0, 7, 2, 9, 0, 43, 15, 4, 13, 15, 10},
	{6482, 10, 0, 7, 2, 9, 0, 43, 15, 4, 13, 15, 10},
	{6427, 10, 0, 7, 2, 9, 0, 44, 15, 4, 13, 15, 10},
	{6373, 10, 0, 7, 3, 9, 0, 45, 15, 4, 13, 15, 10},
	{6321, 10, 0, 7, 3, 9, 0, 46, 15, 4, 13, 15, 10},
	{6270, 10, 0, 7, 3, 9, 0, 47, 15, 4, 13, 15, 10},
	{6222, 10, 0, 7, 3, 9, 0, 47, 15, 4, 13, 15, 10},
	{6174, 10, 0, 7, 3, 9, 0, 48, 15, 4, 13, 15, 10},
	{6128, 10, 0, 7, 3, 9, 0, 49, 15, 4, 13, 15, 10},
	{6083, 10, 0, 7, 3, 9, 0, 50, 15, 4, 13, 15, 10},
	{6040, 10, 0, 7, 3, 9, 0, 51, 15, 4, 13, 15, 10},
	{5997, 10, 0, 7, 3, 8, 0, 51, 15, 4, 13, 15, 10},
}, {
	{12605, 13, 0, 4, 0, 14, 0, 10, 13, 4, 13, 15, 9},   /* 80 MHz */
	{12245, 13, 0, 4, 0, 13, 0, 11, 13, 4, 13, 15, 9},
	{11906, 13, 0, 4, 0, 12, 0, 12, 13, 4, 13, 15, 9},
	{11588, 13, 0, 4, 0, 12, 0, 13, 13, 4, 13, 15, 9},
	{11288, 13, 0, 4, 0, 12, 0, 13, 13, 4, 13, 15, 9},
	{11007, 13, 0, 4, 0, 11, 0, 14, 13, 4, 13, 15, 9},
	{10742, 13, 0, 4, 0, 11, 0, 15, 13, 4, 13, 15, 9},
	{10492, 13, 0, 5, 1, 11, 0, 16, 13, 4, 13, 15, 9},
	{10258, 13, 0, 5, 1, 10, 0, 17, 13, 4, 13, 15, 9},
	{10036, 13, 0, 5, 1, 10, 0, 17, 13, 4, 13, 15, 9},
	{9827, 13, 0, 5, 1, 10, 0, 18, 13, 4, 13, 15, 9},
	{9631, 13, 0, 5, 1, 10, 0, 19, 13, 4, 13, 15, 9},
	{9445, 13, 0, 5, 1, 9, 0, 20, 13, 4, 13, 15, 9},
	{9269, 10, 0, 6, 1, 12, 0, 18, 13, 4, 13, 15, 9},
	{9103, 10, 0, 6, 1, 12, 0, 18, 13, 4, 13, 15, 9},
	{8946, 10, 0, 6, 1, 11, 0, 19, 13, 4, 13, 15, 9},
	{8797, 10, 0, 6, 1, 11, 0, 20, 13, 4, 13, 15, 9},
	{8655, 10, 0, 6, 1, 11, 0, 20, 13, 4, 13, 15, 9},
	{8520, 10, 0, 6, 1, 11, 0, 21, 13, 4, 13, 15, 9},
	{8392, 10, 0, 6, 1, 11, 0, 22, 13, 4, 13, 15, 9},
	{8269, 10, 0, 6, 1, 11, 0, 22, 13, 4, 13, 15, 9},
	{8153, 10, 0, 6, 1, 11, 0, 23, 13, 4, 13, 15, 9},
	{8041, 10, 0, 6, 1, 11, 0, 24, 13, 4, 13, 15, 9},
	{7934, 10, 0, 6, 1, 11, 0, 25, 13, 4, 13, 15, 9},
	{7831, 10, 0, 6, 1, 10, 0, 25, 13, 4, 13, 15, 9},
	{7733, 10, 0, 6, 1, 10, 0, 26, 13, 4, 13, 15, 9},
	{7638, 10, 0, 6, 1, 10, 0, 27, 13, 4, 13, 15, 9},
	{7547, 10, 0, 6, 1, 10, 0, 27, 13, 4, 13, 15, 9},
	{7459, 10, 0, 6, 1, 10, 0, 28, 13, 4, 13, 15, 9},
	{7374, 10, 0, 7, 2, 10, 0, 29, 13, 4, 13, 15, 9},
	{7291, 10, 0, 7, 2, 9, 0, 29, 13, 4, 13, 15, 9},
	{7212, 10, 0, 7, 2, 9, 0, 30, 13, 4, 13, 15, 9},
	{7135, 10, 0, 7, 2, 9, 0, 31, 13, 4, 13, 15, 9},
	{7061, 10, 0, 7, 2, 9, 0, 32, 13, 4, 13, 15, 9},
	{6988, 10, 0, 7, 2, 9, 0, 32, 13, 4, 13, 15, 9},
	{6918, 10, 0, 7, 2, 9, 0, 33, 13, 4, 13, 15, 9},
	{6850, 10, 0, 7, 2, 8, 0, 34, 13, 4, 13, 15, 9},
	{6784, 10, 0, 7, 2, 8, 0, 34, 13, 4, 13, 15, 9},
	{6720, 10, 0, 7, 2, 8, 0, 35, 13, 4, 13, 15, 9},
	{6658, 10, 0, 7, 2, 8, 0, 36, 13, 4, 13, 15, 9},
	{6597, 10, 0, 7, 2, 8, 0, 36, 13, 4, 13, 15, 9},
	{6539, 10, 0, 7, 2, 8, 0, 37, 13, 4, 13, 15, 9},
	{6482, 10, 0, 7, 2, 8, 0, 38, 13, 4, 13, 15, 9},
	{6427, 10, 0, 7, 2, 7, 0, 39, 13, 4, 13, 15, 9},
	{6373, 10, 0, 7, 3, 7, 0, 39, 13, 4, 13, 15, 9},
	{6321, 10, 0, 7, 3, 7, 0, 40, 13, 4, 13, 15, 9},
	{6270, 10, 0, 7, 3, 7, 0, 41, 13, 4, 13, 15, 9},
	{6222, 10, 0, 7, 3, 7, 0, 41, 13, 4, 13, 15, 9},
	{6174, 10, 0, 7, 3, 7, 0, 42, 13, 4, 13, 15, 9},
	{6128, 10, 0, 7, 3, 7, 0, 43, 13, 4, 13, 15, 9},
	{6083, 10, 0, 7, 3, 7, 0, 44, 13, 4, 13, 15, 9},
	{6040, 10, 0, 7, 3, 6, 0, 44, 13, 4, 13, 15, 9},
	{5997, 10, 0, 7, 3, 6, 0, 44, 13, 4, 13, 15, 9},
}};

/* Rx Gain Tables */

#define SIZE_FULL_TABLE		77

const unsigned char full_gain_table[RXGAIN_TBLS_END][SIZE_FULL_TABLE][3] =
{{  /* 800 MHz */
	{0x00, 0x00, 0x20}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
	{0x00, 0x01, 0x00}, {0x00, 0x02, 0x00}, {0x00, 0x03, 0x00},
	{0x00, 0x04, 0x00}, {0x00, 0x05, 0x00}, {0x01, 0x03, 0x20},
	{0x01, 0x04, 0x00}, {0x01, 0x05, 0x00}, {0x01, 0x06, 0x00},
	{0x01, 0x07, 0x00}, {0x01, 0x08, 0x00}, {0x01, 0x09, 0x00},
	{0x01, 0x0A, 0x00}, {0x01, 0x0B, 0x00}, {0x01, 0x0C, 0x00},
	{0x01, 0x0D, 0x00}, {0x01, 0x0E, 0x00}, {0x02, 0x09, 0x20},
	{0x02, 0x0A, 0x00}, {0x02, 0x0B, 0x00}, {0x02, 0x0C, 0x00},
	{0x02, 0x0D, 0x00}, {0x02, 0x0E, 0x00}, {0x02, 0x0F, 0x00},
	{0x02, 0x10, 0x00}, {0x02, 0x2B, 0x20}, {0x02, 0x2C, 0x00},
	{0x04, 0x27, 0x20}, {0x04, 0x28, 0x00}, {0x04, 0x29, 0x00},
	{0x04, 0x2A, 0x00}, {0x04, 0x2B, 0x00}, {0x24, 0x21, 0x20},
	{0x24, 0x22, 0x00}, {0x44, 0x20, 0x20}, {0x44, 0x21, 0x00},
	{0x44, 0x22, 0x00}, {0x44, 0x23, 0x00}, {0x44, 0x24, 0x00},
	{0x44, 0x25, 0x00}, {0x44, 0x26, 0x00}, {0x44, 0x27, 0x00},
	{0x44, 0x28, 0x00}, {0x44, 0x29, 0x00}, {0x44, 0x2A, 0x00},
	{0x44, 0x2B, 0x00}, {0x44, 0x2C, 0x00}, {0x44, 0x2D, 0x00},
	{0x44, 0x2E, 0x00}, {0x44, 0x2F, 0x00}, {0x44, 0x30, 0x00},
	{0x44, 0x31, 0x00}, {0x64, 0x2E, 0x20}, {0x64, 0x2F, 0x00},
	{0x64, 0x30, 0x00}, {0x64, 0x31, 0x00}, {0x64, 0x32, 0x00},
	{0x64, 0x33, 0x00}, {0x64, 0x34, 0x00}, {0x64, 0x35, 0x00},
	{0x64, 0x36, 0x00}, {0x64, 0x37, 0x00}, {0x64, 0x38, 0x00},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20}
},{  /* 2300 MHz */
	{0x00, 0x00, 0x20}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
	{0x00, 0x01, 0x00}, {0x00, 0x02, 0x00}, {0x00, 0x03, 0x00},
	{0x00, 0x04, 0x00}, {0x00, 0x05, 0x00}, {0x01, 0x03, 0x20},
	{0x01, 0x04, 0x00}, {0x01, 0x05, 0x00}, {0x01, 0x06, 0x00},
	{0x01, 0x07, 0x00}, {0x01, 0x08, 0x00}, {0x01, 0x09, 0x00},
	{0x01, 0x0A, 0x00}, {0x01, 0x0B, 0x00}, {0x01, 0x0C, 0x00},
	{0x01, 0x0D, 0x00}, {0x01, 0x0E, 0x00}, {0x02, 0x09, 0x20},
	{0x02, 0x0A, 0x00}, {0x02, 0x0B, 0x00}, {0x02, 0x0C, 0x00},
	{0x02, 0x0D, 0x00}, {0x02, 0x0E, 0x00}, {0x02, 0x0F, 0x00},
	{0x02, 0x10, 0x00}, {0x02, 0x2B, 0x20}, {0x02, 0x2C, 0x00},
	{0x04, 0x28, 0x20}, {0x04, 0x29, 0x00}, {0x04, 0x2A, 0x00},
	{0x04, 0x2B, 0x00}, {0x24, 0x20, 0x20}, {0x24, 0x21, 0x00},
	{0x44, 0x20, 0x20}, {0x44, 0x21, 0x00}, {0x44, 0x22, 0x00},
	{0x44, 0x23, 0x00}, {0x44, 0x24, 0x00}, {0x44, 0x25, 0x00},
	{0x44, 0x26, 0x00}, {0x44, 0x27, 0x00}, {0x44, 0x28, 0x00},
	{0x44, 0x29, 0x00}, {0x44, 0x2A, 0x00}, {0x44, 0x2B, 0x00},
	{0x44, 0x2C, 0x00}, {0x44, 0x2D, 0x00}, {0x44, 0x2E, 0x00},
	{0x44, 0x2F, 0x00}, {0x44, 0x30, 0x00}, {0x44, 0x31, 0x00},
	{0x44, 0x32, 0x00}, {0x64, 0x2E, 0x20}, {0x64, 0x2F, 0x00},
	{0x64, 0x30, 0x00}, {0x64, 0x31, 0x00}, {0x64, 0x32, 0x00},
	{0x64, 0x33, 0x00}, {0x64, 0x34, 0x00}, {0x64, 0x35, 0x00},
	{0x64, 0x36, 0x00}, {0x64, 0x37, 0x00}, {0x64, 0x38, 0x00},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20},
},{  /* 5500 MHz */
	{0x00, 0x00, 0x20}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00}, {0x00, 0x00, 0x00}, {0x00, 0x01, 0x00},
	{0x00, 0x02, 0x00}, {0x00, 0x03, 0x00}, {0x01, 0x01, 0x20},
	{0x01, 0x02, 0x00}, {0x01, 0x03, 0x00}, {0x01, 0x04, 0x20},
	{0x01, 0x05, 0x00}, {0x01, 0x06, 0x00}, {0x01, 0x07, 0x00},
	{0x01, 0x08, 0x00}, {0x01, 0x09, 0x00}, {0x01, 0x0A, 0x00},
	{0x01, 0x0B, 0x00}, {0x01, 0x0C, 0x00}, {0x02, 0x08, 0x20},
	{0x02, 0x09, 0x00}, {0x02, 0x0A, 0x00}, {0x02, 0x0B, 0x20},
	{0x02, 0x0C, 0x00}, {0x02, 0x0D, 0x00}, {0x02, 0x0E, 0x00},
	{0x02, 0x0F, 0x00}, {0x02, 0x2A, 0x20}, {0x02, 0x2B, 0x00},
	{0x04, 0x27, 0x20}, {0x04, 0x28, 0x00}, {0x04, 0x29, 0x00},
	{0x04, 0x2A, 0x00}, {0x04, 0x2B, 0x00}, {0x04, 0x2C, 0x00},
	{0x04, 0x2D, 0x00}, {0x24, 0x20, 0x20}, {0x24, 0x21, 0x00},
	{0x24, 0x22, 0x00}, {0x44, 0x20, 0x20}, {0x44, 0x21, 0x00},
	{0x44, 0x22, 0x00}, {0x44, 0x23, 0x00}, {0x44, 0x24, 0x00},
	{0x44, 0x25, 0x00}, {0x44, 0x26, 0x00}, {0x44, 0x27, 0x00},
	{0x44, 0x28, 0x00}, {0x44, 0x29, 0x00}, {0x44, 0x2A, 0x00},
	{0x44, 0x2B, 0x00}, {0x44, 0x2C, 0x00}, {0x44, 0x2D, 0x00},
	{0x44, 0x2E, 0x00}, {0x64, 0x2E, 0x20}, {0x64, 0x2F, 0x00},
	{0x64, 0x30, 0x00}, {0x64, 0x31, 0x00}, {0x64, 0x32, 0x00},
	{0x64, 0x33, 0x00}, {0x64, 0x34, 0x00}, {0x64, 0x35, 0x00},
	{0x64, 0x36, 0x00}, {0x64, 0x37, 0x00}, {0x64, 0x38, 0x00},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20}
}};

#define SIZE_SPLIT_TABLE		41

const unsigned char split_gain_table[RXGAIN_TBLS_END][SIZE_SPLIT_TABLE][3] =
{{  /* 800 MHz */
	{0x00, 0x18, 0x20}, {0x01, 0x18, 0x20}, {0x02, 0x18, 0x20},
	{0x03, 0x18, 0x20}, {0x04, 0x18, 0x20}, {0x05, 0x18, 0x20},
	{0x06, 0x18, 0x20}, {0x07, 0x18, 0x20}, {0x08, 0x18, 0x20},
	{0x09, 0x18, 0x20}, {0x0A, 0x18, 0x20}, {0x0B, 0x18, 0x20},
	{0x0C, 0x18, 0x20}, {0x0D, 0x18, 0x20}, {0x23, 0x18, 0x20},
	{0x24, 0x18, 0x20}, {0x43, 0x18, 0x20}, {0x44, 0x18, 0x20},
	{0x45, 0x18, 0x20}, {0x46, 0x18, 0x20}, {0x47, 0x18, 0x20},
	{0x48, 0x18, 0x20}, {0x43, 0x38, 0x20}, {0x44, 0x38, 0x20},
	{0x45, 0x38, 0x20}, {0x46, 0x38, 0x20}, {0x47, 0x38, 0x20},
	{0x48, 0x38, 0x20}, {0x63, 0x38, 0x20}, {0x64, 0x38, 0x20},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20},
},{  /* 2300 MHz */

	{0x00, 0x18, 0x20}, {0x01, 0x18, 0x20}, {0x02, 0x18, 0x20},
	{0x03, 0x18, 0x20}, {0x04, 0x18, 0x20}, {0x05, 0x18, 0x20},
	{0x06, 0x18, 0x20}, {0x07, 0x18, 0x20}, {0x08, 0x18, 0x20},
	{0x09, 0x18, 0x20}, {0x0A, 0x18, 0x20}, {0x0B, 0x18, 0x20},
	{0x0C, 0x18, 0x20}, {0x0D, 0x18, 0x20}, {0x23, 0x18, 0x20},
	{0x24, 0x18, 0x20}, {0x43, 0x18, 0x20}, {0x44, 0x18, 0x20},
	{0x45, 0x18, 0x20}, {0x46, 0x18, 0x20}, {0x47, 0x18, 0x20},
	{0x48, 0x18, 0x20}, {0x43, 0x38, 0x20}, {0x44, 0x38, 0x20},
	{0x45, 0x38, 0x20}, {0x46, 0x38, 0x20}, {0x47, 0x38, 0x20},
	{0x48, 0x38, 0x20}, {0x63, 0x38, 0x20}, {0x64, 0x38, 0x20},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20},
},{  /* 5500 MHz */
	{0x00, 0x18, 0x20}, {0x01, 0x18, 0x20}, {0x02, 0x18, 0x20},
	{0x03, 0x18, 0x20}, {0x04, 0x18, 0x20}, {0x05, 0x18, 0x20},
	{0x06, 0x18, 0x20}, {0x07, 0x18, 0x20}, {0x08, 0x18, 0x20},
	{0x09, 0x18, 0x20}, {0x0A, 0x18, 0x20}, {0x0B, 0x18, 0x20},
	{0x0C, 0x18, 0x20}, {0x0D, 0x18, 0x20}, {0x23, 0x18, 0x20},
	{0x24, 0x18, 0x20}, {0x43, 0x18, 0x20}, {0x44, 0x18, 0x20},
	{0x45, 0x18, 0x20}, {0x46, 0x18, 0x20}, {0x47, 0x18, 0x20},
	{0x48, 0x18, 0x20}, {0x43, 0x38, 0x20}, {0x44, 0x38, 0x20},
	{0x45, 0x38, 0x20}, {0x46, 0x38, 0x20}, {0x47, 0x38, 0x20},
	{0x48, 0x38, 0x20}, {0x63, 0x38, 0x20}, {0x64, 0x38, 0x20},
	{0x65, 0x38, 0x20}, {0x66, 0x38, 0x20}, {0x67, 0x38, 0x20},
	{0x68, 0x38, 0x20}, {0x69, 0x38, 0x20}, {0x6A, 0x38, 0x20},
	{0x6B, 0x38, 0x20}, {0x6C, 0x38, 0x20}, {0x6D, 0x38, 0x20},
	{0x6E, 0x38, 0x20}, {0x6F, 0x38, 0x20},
}};

/* Mixer GM Sub-table */

const unsigned char gm_st_gain[16]= {0x78, 0x74, 0x70, 0x6C, 0x68, 0x64, 0x60,
			 0x5C, 0x58, 0x54, 0x50, 0x4C, 0x48, 0x30, 0x18, 0x0};
const unsigned char gm_st_ctrl[16]= {0x0, 0xD, 0x15, 0x1B, 0x21, 0x25, 0x29,
			 0x2C, 0x2F, 0x31, 0x33, 0x34, 0x35, 0x3A, 0x3D, 0x3E};


const char lna_table[] = {6, 17, 19, 25};
const char tia_table[] = {-6, 0};
const char mixer_table[] = {0, 5, 11, 16,
			17, 18, 19, 20,
			21, 22, 23, 24,
			25, 26,	27, 28};

#endif

