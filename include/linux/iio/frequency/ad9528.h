/*
 * AD9528 SPI Low Jitter Clock Generator
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9528_H_
#define IIO_FREQUENCY_AD9528_H_

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
	unsigned char		driver_mode;
	unsigned char		signal_source;
	unsigned char		divider_phase;
	unsigned short		channel_divider;
	char			extended_name[16];
};

/**
 * struct ad9528_platform_data - platform specific information
 *
 * @vcxo_freq: External VCXO frequency in Hz
 * @spi3wire: SPI 3-Wire mode enable;
 * @refa_en: REFA input enable.
 * @refb_en: REFB input enable.
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
 * @sysref_pattern_mode: SYSREF pattern mode
 * @sysref_k_div: SYSREF pattern generator K divider
 * @sysref_nshot_mode: SYSREF pattern NSHOT mode
 * @sysref_req_trigger_mode: SYSREF request trigger mode
 * @sysref_req_en: SYSREF request pin mode enable (default SPI mode)
 * @pll2_charge_pump_current_nA: Magnitude of PLL2 charge pump current (nA).
 * @pll2_freq_doubler_en: PLL2 frequency doubler enable.
 * @pll2_r1_div: PLL2 R1 divider, range 1..31.
 * @pll2_n2_div: PLL2 N2 divider, range 1..256.
 * @pll2_vco_div_m1: VCO1 divider, range 3..5.
 * @pll2_bypass_en: Bypass PLL2.
 * @rpole2: PLL2 loop filter Rpole resistor value.
 * @rzero: PLL2 loop filter Rzero resistor value.
 * @cpole1: PLL2 loop filter Cpole capacitor value.
 * @rzero_bypass_en: PLL2 loop filter Rzero bypass enable.
 * @num_channels: Array size of struct ad9528_channel_spec.
 * @channels: Pointer to channel array.
 * @stat0_pin_sel: Status Monitor Pin 0 function selection.
 * @stat1_pin_sel: Status Monitor Pin 1 function selection.
 * @name: Optional alternative iio device name.
 */

struct ad9528_platform_data {
	unsigned long			vcxo_freq;
	bool				spi3wire;

	/* REFA / REFB input configuration */
	bool				refa_en;
	bool				refb_en;

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
	unsigned char			ref_mode;
	unsigned char			sysref_src;
	unsigned char			sysref_pattern_mode;
	unsigned short			sysref_k_div;
	unsigned char			sysref_nshot_mode;
	unsigned char			sysref_req_trigger_mode;
	bool				sysref_req_en;
	u32				jdev_max_sysref_freq;

	/* PLL2 Setting */
	unsigned int			pll2_charge_pump_current_nA;
	bool				pll2_freq_doubler_en;
	unsigned char			pll2_r1_div;
	unsigned char			pll2_n2_div;
	unsigned char			pll2_vco_div_m1; /* 3..5 */
	bool				pll2_bypass_en;

	/* Loop Filter PLL2 */
	unsigned char			rpole2;
	unsigned char			rzero;
	unsigned char			cpole1;
	bool				rzero_bypass_en;

	/* Output Channel Configuration */
	int				num_channels;
	struct ad9528_channel_spec	*channels;

	char				name[SPI_NAME_SIZE];

	unsigned char			stat0_pin_func_sel;
	unsigned char			stat1_pin_func_sel;
};

#endif /* IIO_FREQUENCY_AD9528_H_ */
