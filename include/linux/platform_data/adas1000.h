#ifndef __PLATFORM_DATA_ADAS1000_H__
#define __PLATFORM_DATA_ADAS1000_H__

/**
 * struct adas1000_platform_data - ADAS1000 platform data
 * @enbale_vref_buffer: If true enable the vref buffer
 * @use_external_clock: If true use the external clock applied at the XTAL pin
 * @driver_external_common_mode: If true drive the internal common mode signal
 *	to the external common mode pin.
 * @use_external_common_mode: If true use the signal applied at the external
 *	common mode pin as the common mode signal.
 * @high_perfomance: If true configure the ADAS1000 for high performance mode.
 **/
struct adas1000_platform_data {
	bool enable_vref_buffer;
	bool use_external_clock;
	bool drive_external_common_mode;
	bool use_external_common_mode;
	bool high_performance;
};

#endif
