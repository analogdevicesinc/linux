/*
 * AD9508 SPI/I2C Clock Fanout Buffer with Output Dividers and Delay Adjust
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DT_BINDINGS_IIO_FREQUENCY_AD9508_H_
#define _DT_BINDINGS_IIO_FREQUENCY_AD9508_H_

/* Output Driver Mode
 * Use for adi,driver-mode */
#define DRIVER_MODE_LVDS_0_50	(0 << 1)
#define DRIVER_MODE_LVDS_0_75	(1 << 1)
#define DRIVER_MODE_LVDS_1_00	(2 << 1)
#define DRIVER_MODE_LVDS_1_25	(3 << 1)
#define DRIVER_MODE_HSTL	(4 << 1)
#define DRIVER_MODE_HSTL_BOOST	(5 << 1)
#define DRIVER_MODE_HIGHZ_CMOS	(6 << 1)

#define DRIVER_PHASE_FORCE_HIGH (0 << 4)
#define DRIVER_PHASE_NORMAL 	(1 << 4)
#define DRIVER_PHASE_INVERTING  (2 << 4)
#define DRIVER_PHASE_FORCE_LOW  (3 << 4)

#define DRIVER_PHASE_CMOS_N_FORCE_HIGH (0 << 10)
#define DRIVER_PHASE_CMOS_N_NORMAL	(1 << 10)
#define DRIVER_PHASE_CMOS_N_INVERTING  (2 << 10)
#define DRIVER_PHASE_CMOS_N_FORCE_LOW  (3 << 10)

#define DRIVER_CMOS_N_ENABLE	(1 << 12)

#define DRIVER_PHASE_CMOS_P_FORCE_HIGH (0 << 13)
#define DRIVER_PHASE_CMOS_P_NORMAL	(1 << 13)
#define DRIVER_PHASE_CMOS_P_INVERTING  (2 << 13)
#define DRIVER_PHASE_CMOS_P_FORCE_LOW  (3 << 13)

#define DRIVER_CMOS_P_ENABLE	(1 << 15)

#endif /* _DT_BINDINGS_IIO_FREQUENCY_AD9508_H_ */
