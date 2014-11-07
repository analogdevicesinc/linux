/*
 * AD9528 SPI Low Jitter Clock Generator
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9528_H_
#define IIO_FREQUENCY_AD9528_H_

enum outp_drv_mode {
	LVDS,		/* 0 */
	LVDS_BOOST,	/* 1 */
	HSTL,		/* 2 */
};

enum outp_signal_src {
	AD9528_VCO		= 0,
	AD9528_VCXO		= 1,
	AD9528_SYSREF_VCO	= 2,
	AD9528_SYSREF_VCXO	= 3,
	AD9528_VCXO_INV		= 5,
	AD9528_SYSREF_VCXO_INV	= 7,
};

enum ref_sel_mode {
	NONEREVERTIVE_STAY_ON_REFB,	/* 0 */
	REVERT_TO_REFA,	/* 1 */
	SELECT_REFA,	/* 2 */
	SELECT_REFB,	/* 3 */
	EXT_REF_SEL	/* 4 */
};

enum sysref_src {
	SYSREF_SRC_EXTERNAL,
	SYSREF_SRC_EXTERNAL_RESAMPLED,
	SYSREF_SRC_INTERNAL,
};

enum sysref_pattern_mode {
	PATTERN_NSHOT,
	PATTERN_CONTINUOUS,
	PATTERN_PRBS,
	PATTERN_STOP,
};

/**
 * struct ad9528_channel_spec - Output channel configuration
 *
 * @channel_num: Output channel number.
 * @sync_ignore_en: Ignore chip-level SYNC signal.
 * @output_dis: Disables, powers down the entire channel.
 * @driver_mode: Output driver mode (logic level family).
 * @divider_phase: Divider initial phase after a SYNC. Range 0..63
		   LSB = 1/2 of a period of the divider input clock.
 * @channel_divider: 10-bit channel divider.
 * @extended_name: Optional descriptive channel name.
 */

struct ad9528_channel_spec {
	unsigned		channel_num;
	bool			sync_ignore_en;
				 /* CH0..CH3 VCXO, CH4..CH9 VCO2 */
	bool			output_dis;
	enum outp_drv_mode	driver_mode;
	enum outp_signal_src	signal_source;
	unsigned char		divider_phase;
	unsigned short		channel_divider;
	char			extended_name[16];
};

enum rpole2_resistor {
	RPOLE2_900_OHM,
	RPOLE2_450_OHM,
	RPOLE2_300_OHM,
	RPOLE2_225_OHM,
};

enum rzero_resistor {
	RZERO_3250_OHM,
	RZERO_2750_OHM,
	RZERO_2250_OHM,
	RZERO_2100_OHM,
	RZERO_3000_OHM,
	RZERO_2500_OHM,
	RZERO_2000_OHM,
	RZERO_1850_OHM,
};

enum cpole1_capacitor {
	CPOLE1_0_PF,
	CPOLE1_8_PF,
	CPOLE1_16_PF,
	CPOLE1_24_PF,
	_CPOLE1_24_PF, /* place holder */
	CPOLE1_32_PF,
	CPOLE1_40_PF,
	CPOLE1_48_PF,
};

/**
 * struct ad9528_platform_data - platform specific information
 *
 * @vcxo_freq: External VCXO frequency in Hz
 * @spi3wire: SPI 3-Wire mode enable;
 * @refa_diff_rcv_en: REFA differential/single-ended input selection.
 * @refb_diff_rcv_en: REFB differential/single-ended input selection.
 * @osc_in_diff_en: OSC differential/ single-ended input selection.
 * @refa_cmos_neg_inp_en: REFA single-ended neg./pos. input enable.
 * @refb_cmos_neg_inp_en: REFB single-ended neg./pos. input enable.
 * @osc_in_cmos_neg_inp_en: OSC single-ended neg./pos. input enable.
 * @refa_r_div: PLL1 10-bit REFA R divider.
 * @refb_r_div: PLL1 10-bit REFB R divider.
 * @pll1_feedback_div: PLL1 10-bit Feedback N divider.
 * @pll1_feedback_src_vcxo: PLL1 Feedback source, True = VCXO, False = VCO
 * @pll1_charge_pump_current_nA: Magnitude of PLL1 charge pump current (nA).
 * @pll1_bypass_en: Bypass PLL1 - Single loop mode
 * @ref_mode: Reference selection mode.
 * @sysref_src: SYSREF pattern generator clock source
 * @sysref_k_div: SYSREF pattern generator K divider
 * @pll2_charge_pump_current_nA: Magnitude of PLL2 charge pump current (nA).
 * @pll2_ndiv_a_cnt: PLL2 Feedback N-divider, A Counter, range 0..4.
 * @pll2_ndiv_b_cnt: PLL2 Feedback N-divider, B Counter, range 0..63.
 * @pll2_freq_doubler_en: PLL2 frequency doubler enable.
 * @pll2_r1_div: PLL2 R1 divider, range 1..31.
 * @pll2_n2_div: PLL2 N2 divider, range 1..256.
 * @pll2_vco_diff_m1: VCO1 divider, range 3..5.
 * @rpole2: PLL2 loop filter Rpole resistor value.
 * @rzero: PLL2 loop filter Rzero resistor value.
 * @cpole1: PLL2 loop filter Cpole capacitor value.
 * @rzero_bypass_en: PLL2 loop filter Rzero bypass enable.
 * @num_channels: Array size of struct ad9528_channel_spec.
 * @channels: Pointer to channel array.
 * @name: Optional alternative iio device name.
 */

struct ad9528_platform_data {
	unsigned long			vcxo_freq;
	bool				spi3wire;

	/* Differential/ Single-Ended Input Configuration */
	bool				refa_diff_rcv_en;
	bool				refb_diff_rcv_en;
	bool				osc_in_diff_en;

	/*
	 * Valid if differential input disabled
	 * if false defaults to pos input
	 */
	bool				refa_cmos_neg_inp_en;
	bool				refb_cmos_neg_inp_en;
	bool				osc_in_cmos_neg_inp_en;

	/* PLL1 Setting */
	unsigned short			refa_r_div;
	unsigned short			refb_r_div;
	unsigned short			pll1_feedback_div;
	bool				pll1_feedback_src_vcxo;
	unsigned short			pll1_charge_pump_current_nA;
	bool				pll1_bypass_en;

	/* Reference */
	enum ref_sel_mode		ref_mode;
	enum sysref_src			sysref_src;
	unsigned short			sysref_k_div;

	/* PLL2 Setting */
	unsigned int			pll2_charge_pump_current_nA;
	unsigned char			pll2_ndiv_a_cnt;
	unsigned char			pll2_ndiv_b_cnt;
	bool				pll2_freq_doubler_en;
	unsigned char			pll2_r1_div;
	unsigned char			pll2_n2_div;
	unsigned char			pll2_vco_diff_m1; /* 3..5 */

	/* Loop Filter PLL2 */
	enum rpole2_resistor		rpole2;
	enum rzero_resistor		rzero;
	enum cpole1_capacitor		cpole1;
	bool				rzero_bypass_en;

	/* Output Channel Configuration */
	int				num_channels;
	struct ad9528_channel_spec	*channels;

	char				name[SPI_NAME_SIZE];
};

#endif /* IIO_FREQUENCY_AD9528_H_ */
