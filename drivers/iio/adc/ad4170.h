/* SPDX-License-Identifier: GPL-2.0 */
/*
 * AD4170 ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

/* In development */
#define AD4170_NAME				"ad4170"
/* ADC Register Lengths*/

#define AD4170_READ_MASK BIT(14)
/* AD4170 registers */
#define AD4170_INTERFACE_CONFIG_A_REG		0x00
#define AD4170_INTERFACE_CONFIG_B_REG		0x01
#define AD4170_DEVICE_CONFIG_REG		0x02
#define AD4170_CHIP_TYPE_REG			0x03
#define AD4170_PRODUCT_ID_L_REG			0x04
#define AD4170_PRODUCT_ID_H_REG			0x05
#define AD4170_CHIP_GRADE_REG			0x06
#define AD4170_SCRATCH_PAD_REG			0x0a
#define AD4170_SPI_REVISION_REG			0x0b
#define AD4170_VENDOR_L_REG			0x0c
#define AD4170_VENDOR_H_REG			0x0d
#define AD4170_INTERFACE_CONFIG_C_REG		0x10
#define AD4170_INTERFACE_STATUS_A_REG		0x11
#define AD4170_STATUS_REG			0x14
#define AD4170_DATA_16b_REG			0x16
#define AD4170_DATA_16b_STATUS_REG		0x18
#define AD4170_DATA_24b_REG			0x1c
#define AD4170_DATA_24b_STATUS_REG		0x20
//#define AD4170_DATA_32b_REG			0x24
#define AD4170_DATA_PER_CHANNEL_REG(x)		(0x28 + 4 * (x))
#define AD4170_PIN_MUXING_REG			0x68
#define AD4170_CLOCK_CTRL_REG			0x6a
#define AD4170_STANDBY_CTRL_REG			0x6c
#define AD4170_POWER_DOWN_SW_REG		0x6e
#define AD4170_ADC_CTRL_REG			0x70
#define AD4170_ERROR_EN_REG			0x72
#define AD4170_ERROR_REG			0x74
//#define AD4170_INFO1			0x76 /* TODO: implement this when it's specified in doc. */
#define AD4170_CHANNEL_EN_REG			0x78
#define AD4170_CHANNEL_SETUP_REG(x)		(0x80 + 4 * (x))
#define AD4170_CHANNEL_MAP_REG(x)		(0x82 + 4 * (x))
#define AD4170_MISC_REG(x)			(0xc0 + 14 * (x))
#define AD4170_AFE_REG(x)			(0xc2 + 14 * (x))
#define AD4170_FILTER_REG(x)			(0xc4 + 14 * (x))
#define AD4170_FILTER_FS_REG(x)			(0xc6 + 14 * (x))
#define AD4170_OFFSET_REG(x)			(0xc8 + 14 * (x))
#define AD4170_GAIN_REG(x)			(0xcb + 14 * (x))
#define AD4170_REF_CONTROL_REG			0x130
#define AD4170_V_BIAS_REG			0x134
#define AD4170_I_PULLUP_REG			0x136
#define AD4170_CURRENT_SOURCE_REG(x)		(0x138 + 2 * (x))
#define AD4170_FIR_CONTROL_REG			0x140
#define AD4170_COEFF_WRITE_DATA_REG		0x144
#define AD4170_COEFF_READ_DATA_REG		0x147
#define AD4170_COEFF_ADDRESS_REG		0x148
#define AD4170_COEFF_WRRD_STB_REG		0x14d
#define AD4170_DAC_SPAN_REG			0x150
#define AD4170_DAC_CHANNEL_EN_REG		0x152
#define AD4170_DAC_HW_TOGGLE_MASK_REG		0x154
#define AD4170_DAC_HW_LDAC_MASK_REG		0x156
#define AD4170_DAC_DATA_REG(x)			(0x158 + 2 * (x))
#define AD4170_DAC_SW_TOGGLE_TRIGGERS_REG	0x168
#define AD4170_DAC_SW_LDAC_TRIGGERS_REG		0x16a
#define AD4170_DAC_INPUTA_REG(x)		(0x16c + 2 * (x))
#define AD4170_DAC_INPUTB_REG(x)		(0x17c + 2 * (x))
#define AD4170_GPIO_MODE_REG			0x190
#define AD4170_OUTPUT_DATA_REG			0x192
#define AD4170_INPUT_DATA_REG			0x194

/* AD4170_REG_INTERFACE_CONFIG_A */
#define AD4170_SW_RESET_MSK		(BIT(7) | BIT(0))
#define AD4170_RESET_SLEEP_US	        1000
#define AD4170_ADDR_ASCENSION_MSK	BIT(5)
#define AD4170_SDO_ENABLE_MSK		BIT(4)
#define AD4170_INT_REF_2_5V		2500000

/* AD4170_REG_INTERFACE_CONFIG_B */
#define AD4170_INTERFACE_CONFIG_B_SINGLE_INST_MSK		BIT(7)
#define AD4170_INTERFACE_CONFIG_B_SHORT_INSTRUCTION_MSK		BIT(3)

#define AD4170_INTERFACE_CONFIG_C_CRC_MSK	(GENMASK(7, 6) | GENMASK (1, 0))

/* AD4170_REG_DATA_STATUS */
#define AD4170_DATA_STATUS_MASTER_ERR_S_MSK			BIT(7)
#define AD4170_DATA_STATUS_POR_FLAG_S_MSK			BIT(6)
#define AD4170_DATA_STATUS_RDYB_MSK				BIT(5)
#define AD4170_DATA_STATUS_SETTLED_FIR_MSK			BIT(4)
#define AD4170_DATA_STATUS_CH_ACTIVE_MSK			GENMASK(3, 0)

/* AD4170_REG_PIN_MUXING */
#define AD4170_PIN_MUXING_CHAN_TO_GPIO_MSK		BIT(14)
#define AD4170_PIN_MUXING_DIG_AUX2_CTRL_MSK		GENMASK(7, 6)
#define AD4170_PIN_MUXING_DIG_AUX1_CTRL_MSK		GENMASK(5, 4)
#define AD4170_PIN_MUXING_SYNC_CTRL_MSK			GENMASK(3, 2)
#define AD4170_PIN_MUXING_DIG_OUT_STR_MSK		BIT(1)
#define AD4170_PIN_MUXING_SDO_RDBY_DLY_MSK		BIT(0)

/* AD4170_REG_CLOCK_CTRL */
#define AD4170_CLOCK_CTRL_DCLK_DIVIDE_MSK		GENMASK(7, 6)
#define AD4170_CLOCK_CTRL_CLOCKDIV_MSK			GENMASK(5, 4)
#define AD4170_CLOCK_CTRL_CLOCKSEL_MSK			GENMASK(1, 0)
#define AD4170_INT_FREQ_16MHZ				16000000
#define AD4170_EXT_FREQ_MHZ_MIN				1000000
#define AD4170_EXT_FREQ_MHZ_MAX				17000000

/* AD4170_REG_STANDBY_CTRL */
#define AD4170_STANDBY_CTRL_STB_EN_CLOCK_MSK			BIT(8)
#define AD4170_STANDBY_CTRL_STB_EN_IPULLUP_MSK			BIT(7)
#define AD4170_STANDBY_CTRL_STB_EN_DIAGNOSTICS_MSK		BIT(6)
#define AD4170_STANDBY_CTRL_STB_EN_DAC_MSK			BIT(5)
#define AD4170_STANDBY_CTRL_STB_EN_PDSW1_MSK			BIT(4)
#define AD4170_STANDBY_CTRL_STB_EN_PDSW0_MSK			BIT(3)
#define AD4170_STANDBY_CTRL_STB_EN_VBIAS_MSK			BIT(2)
#define AD4170_STANDBY_CTRL_STB_EN_IEXC_MSK			BIT(1)
#define AD4170_STANDBY_CTRL_STB_EN_REFERENCE_MSK		BIT(0)

/* AD4170_REG_POWER_DOWN_SW */
#define AD4170_POWER_DOWN_SW_PDSW1_MSK				BIT(1)
#define AD4170_POWER_DOWN_SW_PDSW0_MSK				BIT(0)

/* AD4170_REG_ADC_CTRL */
#define AD4170_ADC_CTRL_PARALLEL_FILT_EN_MSK			BIT(8)
#define AD4170_ADC_CTRL_MULTI_DATA_REG_SEL_MSK			BIT(7)
#define AD4170_ADC_CTRL_CONT_READ_STATUS_EN_MSK			BIT(6)
#define AD4170_REG_CTRL_CONT_READ_MSK				GENMASK(5, 4)
#define AD4170_REG_CTRL_MODE_MSK				GENMASK(3, 0)

/* AD4170_REG_ERROR_EN and AD4170_REG_ERROR */
#define AD4170_ERROR_DEVICE_ERROR_MSK				BIT(15)
#define AD4170_ERROR_DLDO_PSM_ERR_MSK				BIT(13)
#define AD4170_ERROR_ALDO_PSM_ERR_MSK				BIT(12)
#define AD4170_ERROR_IOUT3_COMP_ERR_MSK				BIT(11)
#define AD4170_ERROR_IOUT2_COMP_ERR_MSK				BIT(10)
#define AD4170_ERROR_IOUT1_COMP_ERR_MSK				BIT(9)
#define AD4170_ERROR_IOUT0_COMP_ERR_MSK				BIT(8)
#define AD4170_ERROR_REF_DIFF_MIN_ERR_MSK			BIT(7)
#define AD4170_ERROR_REF_OV_UV_ERR_MSK				BIT(6)
#define AD4170_ERROR_AINM_OV_UV_ERR_MSK				BIT(5)
#define AD4170_ERROR_AINP_OV_UV_ERR_MSK				BIT(4)
#define AD4170_ERROR_ADC_CONV_ERR_MSK				BIT(3)
#define AD4170_ERROR_MM_CRC_ERR_MSK				BIT(1)
#define AD4170_ERROR_ROM_CRC_ERR_MSK				BIT(0)

/* AD4170_REG_CHANNEL_EN */
#define AD4170_CHANNEL_EN(ch)					BIT(ch)

/* AD4170_REG_ADC_CHANNEL_SETUP */
#define AD4170_CHANNEL_SETUPN_REPEAT_N_MSK			GENMASK(15, 8)
#define AD4170_CHANNEL_SETUPN_DELAY_N_MSK			GENMASK(6, 4)
#define AD4170_CHANNEL_SETUPN_SETUP_N_MSK			GENMASK(2, 0)

/* AD4170_REG_ADC_CHANNEL_MAP */
#define AD4170_CHANNEL_MAPN_AINP_MSK				GENMASK(12, 8)
#define AD4170_CHANNEL_MAPN_AINM_MSK				GENMASK(4, 0)

/* AD4170_REG_ADC_SETUPS_MISC */
#define AD4170_ADC_SETUPS_MISC_CHOP_IEXC_MSK			GENMASK(15, 14)
#define AD4170_ADC_SETUPS_MISC_CHOP_ADC_MSK			GENMASK(9, 8)
#define AD4170_ADC_SETUPS_MISC_BURNOUT_MSK			GENMASK(1, 0)

/* AD4170_REG_ADC_SETUPS_AFE */
#define AD4170_ADC_SETUPS_AFE_REF_BUF_M_MSK			GENMASK(11, 10)
#define AD4170_ADC_SETUPS_AFE_REF_BUF_P_MSK			GENMASK(9, 8)
#define AD4170_ADC_SETUPS_AFE_REF_SELECT_MSK			GENMASK(6, 5)
#define AD4170_ADC_SETUPS_AFE_BIPOLAR_MSK			BIT(4)
#define AD4170_ADC_SETUPS_AFE_PGA_GAIN_MSK			GENMASK(3, 0)

/* AD4170_REG_ADC_SETUPS_FILTER */
#define AD4170_ADC_SETUPS_POST_FILTER_SEL_MSK			GENMASK(7, 4)
#define AD4170_ADC_SETUPS_FILTER_TYPE_MSK			GENMASK(3, 0)

/* AD4170 REG_OFFSET*/
#define AD4170_OFFSET_MSK					GENMASK(23, 0)

/* AD4170 REG_GAIN*/
#define AD4170_GAIN_MSK						GENMASK(23, 0)

/* AD4170_REG_CURRENT_SOURCE */
#define AD4170_CURRENT_SOURCE_I_OUT_PIN_MSK			GENMASK(12, 8)
#define AD4170_CURRENT_SOURCE_I_OUT_VAL_MSK			GENMASK(2, 0)

/* AD4170_REG_REF_CONTROL */
#define AD4170_REF_CONTROL_REF_EN_MSK				BIT(0)

/* AD4170_REG_FIR_CONTROL */
#define AD4170_FIR_CONTROL_IIR_MODE_MSK				BIT(15)
#define AD4170_FIR_CONTROL_FIR_MODE_MSK				GENMASK(14, 12)
#define AD4170_FIR_CONTROL_COEFF_SET_MSK			BIT(10)
#define AD4170_FIR_CONTROL_FIR_LENGTH_MSK			GENMASK(6, 0)

/* AD4170_REG_DAC_SPAN */
#define AD4170_REG_DAC_SPAN_DAC_GAIN_MSK			BIT(0)

/* AD4170_REG_DAC_CHANNEL_EN */
#define AD4170_REG_DAC_CHANNEL_EN_DAC_EN_MSK			BIT(0)

/* AD4170_REG_DAC_HW_TOGGLE_MASK */
#define AD4170_REG_DAC_HW_TOGGLE_MASK_HW_TOGGLE_EN_MSK		BIT(0)

/* AD4170_REG_DAC_HW_LDAC_MASK */
#define AD4170_REG_DAC_HW_LDAC_MASK_HW_LDAC_EN_MSK		BIT(0)

/* AD4170_REG_DAC_DATA */
#define AD4170_REG_DAC_DATA_MSK					GENMASK(11, 0)

/* AD4170_REG_DAC_SW_TOGGLE_TRIGGERS */
#define AD4170_REG_DAC_SW_TOGGLE_TRIGGERS_SW_TOGGLE_MSK		BIT(0)

/* AD4170_REG_DAC_SW_LDAC_TRIGGERS */
#define AD4170_REG_DAC_SW_LDAC_TRIGGERS_SW_LDAC_EN_MSK		BIT(0)

#define AD4170_GPIO_MODE_REG_MSK				GENMASK(1, 0)

#define AD4170_NUM_CHANNELS				16
#define AD4170_CHANNEL_INDEX_MAX			8
#define AD4170_MAX_ANALOG_PINS				8
#define AD4170_NUM_SETUPS				8
#define AD4170_NUM_CURRENT_SOURCE			4
#define AD4170_FIR_COEFF_MAX_LENGTH			72
#define AD4170_MAX_DIFF_INPUTS				4
#define AD4170_INVALID_SLOT				-1
#define AD4170_DEFAULT_ADC_GAIN_COEF			0x555555

enum ad4170_gpio_mode {
	AD4170_GPIO_DISABLED,
	AD4170_GPIO_INPUT,
	AD4170_GPIO_OUTPUT
};

/**
 * @enum ad4170_chan_to_gpio
 * @brief Enables Current Channel Number Be Output to GPIO Pins.
 */
enum ad4170_chan_to_gpio {
	/** Active Channel is Not Output to GPIO Pins. */
	AD4170_CHANNEL_NOT_TO_GPIO,
	/** Active Channel is Output to GPIO Pins. */
	AD4170_CHANNEL_TO_GPIO
};

/**
 * @enum ad4170_dig_aux1_ctrl
 * @brief Configures Functionality of DIG_AUX1 Pin.
 */
enum ad4170_dig_aux1_ctrl {
	/** Dig_Aux1 Pin Disabled. High Impedance */
	AD4170_DIG_AUX1_DISABLED,
	/** Dig_Aux1 Pin Configured as ADC Data Ready Output. */
	AD4170_DIG_AUX1_RDY,
	/** Dig_Aux1 Pin Configured as SYNC_OUT Output. */
	AD4170_DIG_AUX1_SYNC,
};

/**
 * @enum ad4170_dig_aux2_ctrl
 * @brief Configures Functionality of DIG_AUX2 Pin.
 */
enum ad4170_dig_aux2_ctrl {
	/** Dig_Aux2 Pin Disabled. High Impedance */
	AD4170_DIG_AUX2_DISABLED,
	/** Dig_Aux2 Pin Configured as DAC LDAC Input. */
	AD4170_DIG_AUX2_LDAC,
	/** Dig_Aux2 Pin Configured as START Input. */
	AD4170_DIG_AUX2_SYNC,
};

/**
 * @enum ad4170_sync_ctrl
 * @brief Configures SYNC_IN Pin for ADC Synchronization.
 */
enum ad4170_sync_ctrl {
	/** SYNC_IN Pin Disabled. */
	AD4170_SYNC_DISABLED,
	/** SYNC_IN Has Default SYNC Functionality. */
	AD4170_SYNC_STANDARD,
	/** SYNC_IN Has Alternative SYNC Functionality. */
	AD4170_SYNC_ALTERNATE
};

/**
 * @enum ad4170_dig_out_str
 * @brief Configures the drive strength of the Digital Outputs.
 */
enum ad4170_dig_out_str {
	/** Default Drive Strength. Recommended for higher IOVDD voltages. */
	AD4170_DIG_STR_DEFAULT,
	/** Increased Drive Strength. */
	AD4170_DIG_STR_HIGH
};

/**
 * @enum ad4170_sdo_rdby_dly
 * @brief Reset Interface on CS or SCLK.
 */
enum ad4170_sdo_rdby_dly {
	/** Reset on Last SCLK. */
	AD4170_SDO_RDY_SCLK,
	/** Reset on /CS Edge. */
	AD4170_SDO_RDY_CSB
};

/**
 * @struct ad4170_pin_muxing
 * @brief Pin_Muxing register settings.
 */
struct ad4170_pin_muxing {
	/** Enables Current Channel Number Be Output to GPIO Pins. */
	enum ad4170_chan_to_gpio chan_to_gpio;
	/** Configures Functionality of DIG_AUX2 Pin. */
	u8 dig_aux2_ctrl;
	/** Configures Functionality of DIG_AUX1 Pin. */
	u8 dig_aux1_ctrl;
	/** Configures SYNC_IN Pin for ADC Synchronization. */
	u8 sync_ctrl;
	/** Configures the drive strength of the Digital Outputs. */
	enum ad4170_dig_out_str dig_out_str;
	/** Reset Interface on CS or SCLK. */
	enum ad4170_sdo_rdby_dly sdo_rdby_dly;
};

/**
 * @enum ad4170_dclk_div
 * @brief Continuous Transmit Data Clock Divider.
 */
enum ad4170_dclk_div {
	/** DCLK Equals Master Clock Divide by 1. */
	AD4170_DCLKDIVBY1,
	/** DCLK Equals Master Clock Divide by 2. */
	AD4170_DCLKDIVBY2,
	/** DCLK Equals Master Clock Divide by 4. */
	AD4170_DCLKDIVBY4,
	/** DCLK Equals Master Clock Divide by 8. */
	AD4170_DCLKDIVBY8
};

/**
 * @enum ad4170_mclk_div
 * @brief Master Clock Divider.
 */
enum ad4170_mclk_div {
	/** Divide by 1. */
	AD4170_CLKDIVBY1,
	/** Divide by 2. */
	AD4170_CLKDIVBY2,
	/** Divide by 4. */
	AD4170_CLKDIVBY4,
	/** Divide by 8. */
	AD4170_CLKDIVBY8
};

/**
 * @enum ad4170_clocksel
 * @brief ADC Clock Select.
 */
enum ad4170_clocksel {
	/** Internal Oscillator. */
	AD4170_INTERNAL_OSC,
	/** Internal Oscillator, Output to XTAL2/CLKIO Pin. */
	AD4170_INTERNAL_OSC_OUTPUT,
	/** External Clock Input on XTAL2/CLKIO Pin. */
	AD4170_EXTERNAL_OSC,
	/** External Crystal on XTAL1 and XTAL2/CLKIO Pins. */
	AD4170_EXTERNAL_XTAL
};

/**
 * @struct ad4170_clock_ctrl
 * @brief Clock_Ctrl register settings.
 */
struct ad4170_clock_ctrl {
	/** Continuous Transmit Data Clock Divider. */
	enum ad4170_dclk_div dclk_divide;
	/** Master Clock Divider. */
	enum ad4170_mclk_div clockdiv;
	/** ADC Clock Select. */
	enum ad4170_clocksel clocksel;
};

/**
 * @enum ad4170_cont_read
 * @brief Configures continuous Data Register Read/Transmit.
 */
enum ad4170_cont_read {
	/* Disable Continuous Read/Transmit. */
	AD4170_CONT_READ_OFF,
	/*
	 * Enable Continuous Read.
	 * This enables continuous read of the ADC Data register over SPI.
	 */
	AD4170_CONT_READ_ON,
	/*
	 * Enable Continuous Transmit.
	 * This enables continuous transmit of the ADC Data register over TDM.
	 */
	AD4170_CONT_TRANSMIT_ON
};

/**
 * @enum ad4170_mode
 * @brief ADC Operating Mode.
 */
enum ad4170_mode {
	/*
	 * Continuous Conversion Mode.
	 * ADC converts continuously on the enabled channel(s) using Sinc-based
	 * filters.
	 */
	AD4170_MODE_CONT,
	/*
	 * Continuous Conversion Mode with FIR filter.
	 * ADC converts continuously on one channel using the FIR Filter.
	 */
	AD4170_MODE_CONT_FIR,
	/* Continuous Conversion Mode with IIR filter.
	 * ADC converts continuously on one channel using the IIR Filter.
	 */
	AD4170_MODE_CONT_IIR,
	/* Single conversion mode.
	 * ADC performs a single conversion (possibly repeated) on each enabled
	 * channel(s) using Sinc based filters.
	 */
	AD4170_MODE_SINGLE = 0x4,
	/* Standby Mode. Part enters Standby Mode. */
	AD4170_MODE_STANDBY,
	/* Power-Down Mode.
	 * All blocks are disabled in Power-Down Mode, including the LDO
	 * regulators and the serial interface.
	 */
	AD4170_MODE_POWER_DOWN,
	/* Idle Mode. ADC enters idle mode, part remains powered on. */
	AD4170_MODE_IDLE,
	/* System Offset Calibration Mode. */
	AD4170_MODE_SYS_OFFSET_CAL,
	/* System Gain Calibration Mode. */
	AD4170_MODE_SYS_GAIN_CAL,
	/* Self Offset Calibration Mode. */
	AD4170_MODE_SELF_OFFSET_CAL,
	/* Self Gain Calibration Mode. */
	AD4170_MODE_SELF_GAIN_CAL
};

/**
 * @struct ad4170_adc_ctrl
 * @brief ADC_Ctrl register settings.
 */
struct ad4170_adc_ctrl {
	/** Enables Multiple Channels to Be Converted in Parallel. */
	bool parallel_filt_en;
	/** Selects Between One or Multiple Data Registers. */
	bool multi_data_reg_sel;
	/** Enables Status Output in Continuous Read/Transmit. */
	bool cont_read_status_en;
	/** Continuous Data Register Read/Transmit Enable. */
	enum ad4170_cont_read cont_read;
	/** ADC Operating Mode. */
	enum ad4170_mode mode;
};

/**
 * @enum ad4170_delay_n
 * @brief Delay to Add After Channel Switch.
 */
enum ad4170_delay_n {
	/** 0 Delay. */
	AD4170_DLY_0,
	/** Delay 16 * Mod_Clk. */
	AD4170_DLY_16,
	/** Delay 256 * Mod_Clk. */
	AD4170_DLY_256,
	/** Delay 1024 * Mod_Clk. */
	AD4170_DLY_1024,
	/** Delay 2048 * Mod_Clk. */
	AD4170_DLY_2048,
	/** Delay 4096 * Mod_Clk. */
	AD4170_DLY_4096,
	/** Delay 8192 * Mod_Clk. */
	AD4170_DLY_8192,
	/** Delay 16384 * Mod_Clk. */
	AD4170_DLY_16384
};

/**
 * @enum ad4170_ain
 * @brief Multiplexer Positive/Negative Input for This Channel.
 */
enum ad4170_ain {
	AD4170_AIN0,
	AD4170_AIN1,
	AD4170_AIN2,
	AD4170_AIN3,
	AD4170_AIN4,
	AD4170_AIN5,
	AD4170_AIN6,
	AD4170_AIN7,
	AD4170_AIN8,
	AD4170_AIN9,
	AD4170_AIN10,
	AD4170_AIN11,
	AD4170_AIN12,
	AD4170_AIN13,
	AD4170_AIN14,
	AD4170_AIN15,
	AD4170_AIN16,
	AD4170_TEMP_SENSOR_P = 17,
	AD4170_TEMP_SENSOR_N = 17,
	AD4170_AVDD_AVSS_P = 18,
	AD4170_AVDD_AVSS_N = 18,
	AD4170_IOVDD_DGND_P = 19,
	AD4170_IOVDD_DGND_N = 19,
	AD4170_DAC,
	AD4170_ALDO,
	AD4170_DLDO,
	AD4170_AVSS,
	AD4170_DGND,
	AD4170_REFIN1_P,
	AD4170_REFIN1_N,
	AD4170_REFIN2_P,
	AD4170_REFIN2_N,
	AD4170_REFOUT,
	AD4170_OPEN = 31
};

/**
 * @struct ad4170_channel_setup
 * @brief Channel_Setup register settings.
 */
struct ad4170_channel_setup {
	/** Number of Times to Repeat This Channel. */
	u8 repeat_n;
	/** Delay to Add After Channel Switch. */
	enum ad4170_delay_n delay_n;
	/* Setup to Use for This Channel.
	 * A "Setup" includes the configuration for the AFE and the Digital
	 * Filter.
	 */
	u8 setup_n;
};

/**
 * @struct ad4170_channel_map
 * @brief Channel_Map register settings. Selects Analog Inputs for This Sequencer Channel.
 */
struct ad4170_channel_map {
	/** Multiplexer Positive Input. */
	enum ad4170_ain ainp;
	/** Multiplexer Negative Input. */
	enum ad4170_ain ainm;
};

/**
 * @enum ad4170_chop_iexc
 * @brief Excitation Current Chopping Control.
 */
enum ad4170_chop_iexc {
	/* No Chopping of Excitation Currents. */
	AD4170_CHOP_IEXC_OFF = AD4170_MISC_CHOP_IEXC_OFF,
	/* Chopping of Iout_A and Iout_B Excitation Currents. */
	AD4170_CHOP_IEXC_AB = AD4170_MISC_CHOP_IEXC_AB,
	/* Chopping of Iout_C and Iout_D Excitation Currents. */
	AD4170_CHOP_IEXC_CD = AD4170_MISC_CHOP_IEXC_CD,
	/* Chopping of Both Pairs of Excitation Currents. */
	AD4170_CHOP_IEXC_ABCD = AD4170_MISC_CHOP_IEXC_ABCD,
	/* Enum max value */
	AD4170_IEXC_CHOP_MAX
};

/**
 * @enum ad4170_chop_adc
 * @brief ADC/Mux Chopping Control.
 */
enum ad4170_chop_adc {
	/** No Chopping. */
	AD4170_CHOP_OFF,
	/** Chops Internal Mux. */
	AD4170_CHOP_MUX,
	/** Chops AC Excitation Using 4 GPIO Pins. */
	AD4170_CHOP_ACX_4PIN,
	/** Chops AC Excitation Using 2 GPIO Pins. */
	AD4170_CHOP_ACX_2PIN
};

/**
 * @enum ad4170_burnout
 * @brief Burnout Current Values.
 */
enum ad4170_burnout {
	AD4170_BURNOUT_OFF,
	AD4170_BURNOUT_100NA,
	AD4170_BURNOUT_2000NA,
	AD4170_BURNOUT_10000NA,
	AD4170_BURNOUT_MAX
};

/**
 * @struct ad4170_misc
 * @brief Misc register settings.
 */
struct ad4170_misc {
	/** Excitation Current Chopping Control. */
	enum ad4170_chop_iexc chop_iexc;
	/** ADC/Mux Chopping Control. */
	enum ad4170_chop_adc chop_adc;
	/** Burnout Current Values. */
	enum ad4170_burnout burnout;
};

/**
 * @enum ad4170_ref_buf
 * @brief REFIN Buffer Enable.
 */
enum ad4170_ref_buf {
	/** Pre-charge Buffer. */
	AD4170_REF_BUF_PRE,
	/** Full Buffer.*/
	AD4170_REF_BUF_FULL,
	/** Bypass */
	AD4170_REF_BUF_BYPASS
};

/**
 * @enum ad4170_ref_select
 * @brief ADC Reference Selection.
 */
enum ad4170_ref_select {
	AD4170_REFIN_REFIN1,
	AD4170_REFIN_REFIN2,
	AD4170_REFIN_REFOUT,
	AD4170_REFIN_AVDD,
	AD4170_REFIN_MAX
};

/**
 * @enum ad4170_pga_gain
 * @brief PGA Gain Selection.
 */
enum ad4170_pga_gain {
	/** PGA Gain = 1 */
	AD4170_PGA_GAIN_1,
	/** PGA Gain = 2 */
	AD4170_PGA_GAIN_2,
	/** PGA Gain = 4 */
	AD4170_PGA_GAIN_4,
	/** PGA Gain = 8 */
	AD4170_PGA_GAIN_8,
	/** PGA Gain = 16 */
	AD4170_PGA_GAIN_16,
	/** PGA Gain = 32 */
	AD4170_PGA_GAIN_32,
	/** PGA Gain = 64 */
	AD4170_PGA_GAIN_64,
	/** PGA Gain = 128 */
	AD4170_PGA_GAIN_128,
	/** PGA Gain = 0.5 */
	AD4170_PGA_GAIN_0P5,
	/*
	 * PGA Gain = 1 Pre-charge Buffer.
	 * Input currents may increase when the pre-charge-buffer is used.
	 */
	AD4170_PGA_GAIN_1_PRECHARGE,
	AD4170_PGA_GAIN_MAX
};

/**
 * @struct ad4170_afe
 * @brief AFE register settings.
 */
struct ad4170_afe {
	/* REFIN Buffer- Enable. */
	enum ad4170_ref_buf ref_buf_m;
	/* REFIN Buffer+ Enable. */
	enum ad4170_ref_buf ref_buf_p;
	/* ADC Reference Selection. */
	enum ad4170_ref_select ref_select;
	/* Select Bipolar or Unipolar ADC Span. */
	bool bipolar;
	/* PGA Gain Selection. */
	enum ad4170_pga_gain pga_gain;
};

/**
 * @enum ad4170_post_filter
 * @brief Optional Post-Filter configuration.
 */
enum ad4170_post_filter {
	/* No Post Filter. */
	AD4170_POST_FILTER_NONE,
	/* Post Filter for 50/60Hz Rejection with 40ms Settling. */
	AD4170_POST_FILTER_40MS,
	/* Post Filter for 50/60Hz Rejection with 50ms Settling. */
	AD4170_POST_FILTER_50MS,
	/* Post Filter for 50/60Hz Rejection with 60ms Settling. */
	AD4170_POST_FILTER_60MS,
	/* Post Filter for AC Excitation with 5ms Settling. */
	AD4170_POST_FILTER_FAST_AC,
	/* Post Filter for Average-By-16. */
	AD4170_POST_FILTER_AVG16,
	/* Post Filter for 60Hz Rejection with 16.7ms Settling. */
	AD4170_POST_FILTER_AVG20,
	/* Post Filter for 50Hz Rejection with 20ms Settling. */
	AD4170_POST_FILTER_AVG24
};

/**
 * @enum ad4170_filter_type
 * @brief Filter Mode for Sinc-Based Filters.
 */
enum ad4170_filter_type {
	/** Sinc5 Plus Average. */
	AD4170_FILT_SINC5_AVG = 0x0,
	/** Sinc5 */
	AD4170_FILT_SINC5 = 0x4,
	/** Sinc3 */
	AD4170_FILT_SINC3 = 0x6,
};

/**
 * @struct ad4170_filter
 * @brief Filter register settings.
 */
struct ad4170_filter {
	/* Optional Post-Filter configuration. */
	enum ad4170_post_filter post_filter_sel;
	/* Filter Mode for Sinc-Based Filters. */
	enum ad4170_filter_type filter_type;
};

/**
 * @struct ad4170_setup
 * @brief Sequencer Setup register settings.
 */
struct ad4170_setup {
	struct ad4170_misc misc;
	/* Configures Analog Front End - PGA, Reference, Buffers. */
	struct ad4170_afe afe;
	/* Selects Digital Filter Type. */
	struct ad4170_filter filter;
	/* Filter select word for Digital Filters. */
	unsigned int filter_fs;
	/* Digital Offset Adjustment Value. */
	u32 offset;
	/* Digital Gain Adjustment Value. */
	u32 gain;
};

/**
 * @struct ad4170_ref_control
 * @brief Ref_Control register settings.
 */
struct ad4170_ref_control {
	/** Internal Reference Enable. */
	bool ref_en;
};

/**
 * @enum ad4170_i_out_pin
 * @brief Current Source Destination.
 */
enum ad4170_i_out_pin {
	/* I_OUT is Available on AIN0. */
	AD4170_I_OUT_AIN0,
	/* I_OUT is Available on AIN1. */
	AD4170_I_OUT_AIN1,
	/* I_OUT is Available on AIN2. */
	AD4170_I_OUT_AIN2,
	/* I_OUT is Available on AIN3. */
	AD4170_I_OUT_AIN3,
	/* I_OUT is Available on AIN4. */
	AD4170_I_OUT_AIN4,
	/* I_OUT is Available on AIN5. */
	AD4170_I_OUT_AIN5,
	/* I_OUT is Available on AIN6. */
	AD4170_I_OUT_AIN6,
	/* I_OUT is Available on AIN7. */
	AD4170_I_OUT_AIN7,
	/* I_OUT is Available on AIN8. */
	AD4170_I_OUT_AIN8,
	/* I_OUT is Available on AIN9. */
	AD4170_I_OUT_AIN9,
	/* I_OUT is Available on AIN10. */
	AD4170_I_OUT_AIN10,
	/* I_OUT is Available on AIN11. */
	AD4170_I_OUT_AIN11,
	/* I_OUT is Available on AIN12. */
	AD4170_I_OUT_AIN12,
	/* I_OUT is Available on AIN13. */
	AD4170_I_OUT_AIN13,
	/* I_OUT is Available on AIN14. */
	AD4170_I_OUT_AIN14,
	/* I_OUT is Available on AIN15. */
	AD4170_I_OUT_AIN15,
	/* I_OUT is Available on AINCOM. */
	AD4170_I_OUT_AINCOM,
	/* I_OUT is Available on GPIO0. */
	AD4170_I_OUT_GPIO0,
	/* I_OUT is Available on GPIO1. */
	AD4170_I_OUT_GPIO1,
	/* I_OUT is Available on GPIO2. */
	AD4170_I_OUT_GPIO2,
	/* I_OUT is Available on GPIO3. */
	AD4170_I_OUT_GPIO3
};

/**
 * @enum ad4170_i_out_val
 * @brief Current Source Value.
 */
enum ad4170_i_out_val {
	AD4170_I_OUT_0UA,
	AD4170_I_OUT_10UA,
	AD4170_I_OUT_50UA,
	AD4170_I_OUT_100UA,
	AD4170_I_OUT_250UA,
	AD4170_I_OUT_500UA,
	AD4170_I_OUT_1000UA,
	AD4170_I_OUT_1500UA,
	AD4170_I_OUT_MAX
};

/**
 * @struct ad4170_current_source
 * @brief Current_Source register settings.
 */
struct ad4170_current_source {
	enum ad4170_i_out_pin i_out_pin;
	enum ad4170_i_out_val i_out_val;
};

/**
 * @enum ad4170_fir_mode
 * @brief Selects FIR Type.
 */
enum ad4170_fir_mode {
	/*
	 * FIR Default.
	 * Selects the default FIR filter and ignores any programmed FIR_Length
	 * and FIR Coefficient values.
	 */
	AD4170_FIR_DEFAULT,
	/** FIR Programmable with Odd Symmetric Coefficients. */
	AD4170_FIR_SYM_ODD,
	/** FIR Programmable with Even Symmetric Coefficients. */
	AD4170_FIR_SYM_EVEN,
	/** FIR Programmable with Odd Anti-Symmetric Coefficients. */
	AD4170_FIR_ANTISYM_ODD,
	/** FIR Programmable with Even Anti-Symmetric Coefficients. */
	AD4170_FIR_ANTISYM_EVEN,
	/** FIR Programmable with Asymmetric Coefficients. */
	AD4170_FIR_ASYM
};

/**
 * @enum ad4170_fir_coefficient set
 * @brief Selects FIR coefficient set.
 */
enum ad4170_fir_coeff_set {
	/* FIR set 0. Use coefficient addresses 0 to FIR_LENGTH-1 */
	AD4170_FIR_COEFF_SET0,
	/* FIR set 1. Use coefficient addresses 72 to 72 + FIR_LENGTH-1 */
	AD4170_FIR_COEFF_SET1
};

/**
 * @enum ad4170_fir_control
 * @brief FIR_Control register settings.
 */
struct ad4170_fir_control {
	/** Selects FIR Type. */
	enum ad4170_fir_mode fir_mode;
	/** Selects Which Set of FIR Coefficients to Use. */
	enum ad4170_fir_coeff_set coeff_set;
	/** Number of Programmed Coefficients, Max: 72. */
	u8 fir_length;
	/** Pointer to array holding the FIR coefficients */
	s32 *fir_coefficients;
};

/**
 * @enum ad4170_dac_gain
 * @brief DAC Output Span.
 */
enum ad4170_dac_gain {
	/** DAC Output Range is 0V to REFOUT. */
	AD4170_DAC_GAIN_1,
	/** DAC Output Range is 0V to 2*REFOUT. */
	AD4170_DAC_GAIN_2
};

/**
 * @struct ad4170_dac_config
 * @brief DAC Config settings (registers HW_LDAC_Mask, HW_Toggle_Mask, Channel_En and DAC_Span)
 */
struct ad4170_dac_config {
	/** Selects DAC0 Enabled/Disabled. */
	bool enabled;
	/** Select DAC0 Gain. */
	enum ad4170_dac_gain gain;
	/** Select DAC0 HW Toggle. */
	bool hw_toggle;
	/** Select DAC0 LDAC Toggle. */
	bool hw_ldac;
};

/**
 * @struct ad4170_config
 * @brief AD4170 configuration.
 */
struct ad4170_config {
	/** Pin_Muxing register settings. */
	struct ad4170_pin_muxing pin_muxing;
	/** Clock_Ctrl register settings. */
	struct ad4170_clock_ctrl clock_ctrl;
	/** Standby_Ctrl register settings. */
	u16 standby_ctrl;
	/** Power_Down_Sw register settings. */
	u16 powerdown_sw;
	/** Error_En register settings. */
	u16 error_en;
	/** ADC_Ctrl register settings. */
	struct ad4170_adc_ctrl adc_ctrl;
	/** Channel_En register settings. */
	u16 channel_en;
	/** Channel_Setup register settings. */
	struct ad4170_channel_setup ch_setup[AD4170_NUM_CHANNELS];
	/** Channel_Map register settings. */
	struct ad4170_channel_map map[AD4170_NUM_CHANNELS];
	/** Setups settings. */
	struct ad4170_setup setups[AD4170_NUM_SETUPS];
	/** Ref_Control register settings. */
	struct ad4170_ref_control ref_control;
	/** V_Bias register settings. */
	u16 v_bias;
	/** I_Pullup register settings. */
	u16 i_pullup;
	/** Current_Source register settings */
	struct ad4170_current_source current_source[AD4170_NUM_CURRENT_SOURCE];
	/** FIR_Control register settings. */
	struct ad4170_fir_control fir_control;
	/** DAC settings (registers HW_LDAC_Mask, HW_Toggle_Mask, Channel_En and DAC_Span). */
	struct ad4170_dac_config dac;
};

/**
 * @struct ad4170_spi_settings
 * @brief AD4170 SPI settings.
 */
struct ad4170_spi_settings {
	/** Select short instruction, 7-bit Addressing, instead of the default 15-bit Addressing. */
	bool short_instruction;
	/** Enable communication CRC generation and checking. */
	bool crc_enabled;
	/** Enable sync pattern loss detection and recovery. */
	bool sync_loss_detect;
};

static const unsigned int ad4170_reg_size[] = {
	[AD4170_INTERFACE_CONFIG_A_REG] = 1,
	[AD4170_INTERFACE_CONFIG_B_REG]	= 1,
	[AD4170_DEVICE_CONFIG_REG]	= 1,
	[AD4170_CHIP_TYPE_REG]	= 1,
	[AD4170_PRODUCT_ID_L_REG]	= 1,
	[AD4170_PRODUCT_ID_H_REG]	= 1,
	[AD4170_CHIP_GRADE_REG]	= 1,
	[AD4170_SCRATCH_PAD_REG]	= 1,
	[AD4170_SPI_REVISION_REG]	= 1,
	[AD4170_VENDOR_L_REG]	= 1,
	[AD4170_VENDOR_H_REG]	= 1,
	[AD4170_INTERFACE_CONFIG_C_REG]	= 1,
	[AD4170_INTERFACE_STATUS_A_REG]	= 1,
	[AD4170_STATUS_REG]	= 2,
	[AD4170_DATA_16b_REG]	= 2,
	[AD4170_DATA_16b_STATUS_REG]	= 3,
	[AD4170_DATA_24b_REG]	= 3,
	[AD4170_DATA_24b_STATUS_REG]	= 4,
	[AD4170_DATA_PER_CHANNEL_REG(0) ... AD4170_DATA_PER_CHANNEL_REG(AD4170_NUM_CHANNELS - 1)] = 3,
	[AD4170_PIN_MUXING_REG]	= 2,
	[AD4170_CLOCK_CTRL_REG]	= 2,
	[AD4170_STANDBY_CTRL_REG]	= 2,
	[AD4170_POWER_DOWN_SW_REG]	= 2,
	[AD4170_ADC_CTRL_REG]	= 2,
	[AD4170_ERROR_EN_REG]	= 2,
	[AD4170_ERROR_REG]	= 2,
	[AD4170_CHANNEL_EN_REG]	= 2,
	/*
	 * CHANNEL_SETUP and CHANNEL_MAP register are all 2 byte size each and
	 * their addresses are interleaved such that we have CHANNEL_SETUP0
	 * address followed by CHANNEL_MAP0 address, followed by CHANNEL_SETUP1,
	 * and so on until CHANNEL_MAP15.
	 * Thus, initialize the register size for them only once.
	 */
	[AD4170_CHANNEL_SETUP_REG(0) ... AD4170_CHANNEL_MAP_REG(AD4170_NUM_CHANNELS - 1)] = 2,
	/*
	 * MISC, AFE, FILTER, FILTER_FS, OFFSET, and GAIN register addresses are
	 * also interleaved but MISC, AFE, FILTER, FILTER_FS, OFFSET are 16-bit
	 * while OFFSET, GAIN are 24-bit registers so we can't init them all to
	 * the same size.
	 */
	/* Init MISC register size */
	[AD4170_MISC_REG(0)] = 2,
	[AD4170_MISC_REG(1)] = 2,
	[AD4170_MISC_REG(2)] = 2,
	[AD4170_MISC_REG(3)] = 2,
	[AD4170_MISC_REG(4)] = 2,
	[AD4170_MISC_REG(5)] = 2,
	[AD4170_MISC_REG(6)] = 2,
	[AD4170_MISC_REG(7)] = 2,
	/* Init AFE register size */
	[AD4170_AFE_REG(0)] = 2,
	[AD4170_AFE_REG(1)] = 2,
	[AD4170_AFE_REG(2)] = 2,
	[AD4170_AFE_REG(3)] = 2,
	[AD4170_AFE_REG(4)] = 2,
	[AD4170_AFE_REG(5)] = 2,
	[AD4170_AFE_REG(6)] = 2,
	[AD4170_AFE_REG(7)] = 2,
	/* Init FILTER register size */
	[AD4170_FILTER_REG(0)]	= 2,
	[AD4170_FILTER_REG(1)]	= 2,
	[AD4170_FILTER_REG(2)]	= 2,
	[AD4170_FILTER_REG(3)]	= 2,
	[AD4170_FILTER_REG(4)]	= 2,
	[AD4170_FILTER_REG(5)]	= 2,
	[AD4170_FILTER_REG(6)]	= 2,
	[AD4170_FILTER_REG(7)]	= 2,
	/* Init FILTER_FS register size */
	[AD4170_FILTER_FS_REG(0)]	= 2,
	[AD4170_FILTER_FS_REG(1)]	= 2,
	[AD4170_FILTER_FS_REG(2)]	= 2,
	[AD4170_FILTER_FS_REG(3)]	= 2,
	[AD4170_FILTER_FS_REG(4)]	= 2,
	[AD4170_FILTER_FS_REG(5)]	= 2,
	[AD4170_FILTER_FS_REG(6)]	= 2,
	[AD4170_FILTER_FS_REG(7)]	= 2,
	/* Init OFFSET register size */
	[AD4170_OFFSET_REG(0)]	= 3,
	[AD4170_OFFSET_REG(1)]	= 3,
	[AD4170_OFFSET_REG(2)]	= 3,
	[AD4170_OFFSET_REG(3)]	= 3,
	[AD4170_OFFSET_REG(4)]	= 3,
	[AD4170_OFFSET_REG(5)]	= 3,
	[AD4170_OFFSET_REG(6)]	= 3,
	[AD4170_OFFSET_REG(7)]	= 3,
	/* Init GAIN register size */
	[AD4170_GAIN_REG(0)]	= 3,
	[AD4170_GAIN_REG(1)]	= 3,
	[AD4170_GAIN_REG(2)]	= 3,
	[AD4170_GAIN_REG(3)]	= 3,
	[AD4170_GAIN_REG(4)]	= 3,
	[AD4170_GAIN_REG(5)]	= 3,
	[AD4170_GAIN_REG(6)]	= 3,
	[AD4170_GAIN_REG(7)]	= 3,
	[AD4170_REF_CONTROL_REG]	= 2,
	[AD4170_V_BIAS_REG]	= 2,
	[AD4170_I_PULLUP_REG]	= 2,
	[AD4170_CURRENT_SOURCE_REG(0)...AD4170_CURRENT_SOURCE_REG(AD4170_NUM_CURRENT_SOURCE - 1)] = 2,
	[AD4170_FIR_CONTROL_REG]	= 2,
	[AD4170_COEFF_WRITE_DATA_REG]	= 4,
	[AD4170_COEFF_READ_DATA_REG]	= 4,
	[AD4170_COEFF_ADDRESS_REG]	= 2,
	[AD4170_COEFF_WRRD_STB_REG]	= 2,
	[AD4170_DAC_SPAN_REG]	= 2,
	[AD4170_DAC_CHANNEL_EN_REG]	= 2,
	[AD4170_DAC_HW_TOGGLE_MASK_REG]	= 2,
	[AD4170_DAC_HW_LDAC_MASK_REG]	= 2,
	[AD4170_DAC_DATA_REG(0)]	= 2,
	[AD4170_DAC_SW_TOGGLE_TRIGGERS_REG]	= 2,
	[AD4170_DAC_SW_LDAC_TRIGGERS_REG]	= 2,
	[AD4170_DAC_INPUTA_REG(0)]	= 2,
	[AD4170_DAC_INPUTB_REG(0)]	= 2,
	[AD4170_GPIO_MODE_REG]	= 2,
	[AD4170_OUTPUT_DATA_REG]	= 2,
	[AD4170_INPUT_DATA_REG]	= 2,
};
