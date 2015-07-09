/*
 * ADF5355 SPI PLL driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_PLL_ADF5355_H_
#define IIO_PLL_ADF5355_H_


/**
 * struct adf5355_platform_data - platform specific information
 * @name:		Optional device name.
 * @clkin:		REFin frequency in Hz.
 * @channel_spacing:	Channel spacing in Hz (influences MODULUS).
 * @power_up_frequency:	Optional, If set in Hz the PLL tunes to the desired
 *			frequency on probe.
 * @ref_div_factor:	Optional, if set the driver skips dynamic calculation
 *			and uses this default value instead.
 * @ref_doubler_en:	Enables reference doubler.
 * @ref_div2_en:		Enables reference divider.
 * @gpio_lock_detect:	Optional, if set with a valid GPIO number,
 *			pll lock state is tested upon read.
 *			If not used - set to -1.
 * @outa_en: 		Enables or disables the primary RF output
 * @outb_en: 		Enables or disables the auxiliary/high RF output
 * @outa_power		Set the value of the primary RF output power level
 * @outb_power		Set the value of the auxiliary/high RF output power level
 * @mute_till_lock_detect_en: If enabled, the supply current to the RF
			output stage is shut down until the device achieves lock,
			as determined by the digital lock detect circuitry.
 * @phase_detector_polarity_neg: When a passive loop filter or a noninverting
 * 			active loop filter is used, set to positive.
 * 			If an active filter with an inverting characteristic is
 * 			used, set this to negative.
 * @cp_neg_bleed_en:	Use of constant negative bleed. (recommended for most
 *			fractional-N applications)
 * @cp_gated_bleed_en:  Enables gated bleed.
 * @cp_curr_uA:		Set the charge pump current in uA. Set this value to
 * 			the charge pump current that the loop
 *			filter is designed with. For the lowest spurs, the
 *			0.9 mA setting is recommended.
 * @mux_out_sel:		Controls the on-chip multiplexer (MUXOUT).
 * @mux_out_3V3_en:	MUXOUT is programmable to two logic levels. Clear this to
 *			select 1.8 V logic, and set it to select 3.3 V logic.
 * @clock_shift:		Defines the rate shift (scaling) between the kernel
 * 			clock framework and the driver.
 * 			(max ADF5355 rate in Hz > ULONG_MAX)
 *
 */

 struct adf5355_platform_data {
 	char			name[32];
 	unsigned long		clkin;
 	unsigned long long	power_up_frequency;

 	u32			ref_div_factor; /* 10-bit R counter */
 	bool 			ref_diff_en;
 	bool			ref_doubler_en;
 	bool			ref_div2_en;

	bool			mux_out_3V3_en; /* otherwise 1V8 */
	u32			mux_out_sel;

	u32			cp_curr_uA;
	bool			cp_neg_bleed_en;
	bool			cp_gated_bleed_en;
	bool			phase_detector_polarity_neg;

	bool			mute_till_lock_detect_en;
	bool 			outb_en;
	bool 			outa_en;
	u32			outb_power;
	u32			outa_power;

	u32			clock_shift;

 	int			gpio_lock_detect;
 };

#endif /* IIO_PLL_ADF5355_H_ */
