/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * Copyright 2024 NXP
 */

#ifndef _OX03C10_H_
#define _OX03C10_H_

#include <linux/v4l2-controls.h>

/* Custom controls and data structures used for passing data from userspace */

#define V4L2_CID_OX03C10_BASE				(V4L2_CID_USER_BASE | 0x1000)

#define V4L2_CID_OX03C10_EXPOSURE			(V4L2_CID_OX03C10_BASE + 0)
/*
 * Exposure structure that will be used with the V4L2_CID_OX03C10_EXPOSURE custom control. The
 * values are in double-rows time.
 */
struct ox03c10_exposure {
	__u16 dcg;
	__u16 spd;
	__u16 vs;
} __attribute__ ((__packed__));

#define V4L2_CID_OX03C10_ANALOGUE_GAIN			(V4L2_CID_OX03C10_BASE + 1)
/*
 * Analog gain structure that will be used with the V4L2_CID_OX03C10_ANALOGUE_GAIN custom control.
 * The gain values are Q4.4 fixed-point format for each capture mode.
 */
struct ox03c10_analog_gain {
	__u8 hcg;
	__u8 lcg;
	__u8 spd;
	__u8 vs;

} __attribute__ ((__packed__));

#define V4L2_CID_OX03C10_DIGITAL_GAIN			(V4L2_CID_OX03C10_BASE + 2)
/*
 * Digital gain structure that will be used with the V4L2_CID_OX03C10_DIGITAL_GAIN custom control.
 * The gain values are Q4.10 fixed-point format for each capture mode.
 */
struct ox03c10_digital_gain {
	__u16 hcg;
	__u16 lcg;
	__u16 spd;
	__u16 vs;
} __attribute__ ((__packed__));

/*
 * White balance gain control will accept an array of 4 ox03c10_wb_capture_gain structures, one for
 * each capture mode: [0] = HCG, [1] = LCG, [2] = SPD and [3] = VS
 */
#define V4L2_CID_OX03C10_WB_GAIN			(V4L2_CID_OX03C10_BASE + 3)
struct ox03c10_wb_capture_gain {
	__u16 b;
	__u16 gb;
	__u16 gr;
	__u16 r;
} __attribute__ ((__packed__));

/* PWL compression enable */
#define V4L2_CID_OX03C10_PWL_EN				(V4L2_CID_OX03C10_BASE + 4)

/* PWL compression parameters */
#define V4L2_CID_OX03C10_PWL_CTRL			(V4L2_CID_OX03C10_BASE + 5)
struct ox03c10_pwl_ctrl {
	__u8 pack24bit_sel:2;
	__u8 pwl_mode:2;
} __attribute__ ((__packed__));

#define OX03C10_PWL_MODE_12BIT				0
#define OX03C10_PWL_MODE_14BIT				1
#define OX03C10_PWL_MODE_16BIT				2
#define OX03C10_PWL_MODE_20BIT				3

/* PWL compression knee points LUT. The LUT will be passed on as a u8 array of size 132. */
#define V4L2_CID_OX03C10_PWL_KNEE_POINTS_LUT		(V4L2_CID_OX03C10_BASE + 6)

/* OTP correction */
#define V4L2_CID_OX03C10_OTP_CORRECTION			(V4L2_CID_OX03C10_BASE + 7)
struct ox03c10_otp_correction {
	__u32 val1; /* [23:16] = reg(0x7057), [15:8] = reg(0x7058), [7:0] = reg(0x7059) */
	__u32 val2; /* [23:16] = reg(0x705B), [15:8] = reg(0x705C), [7:0] = reg(0x705D) */
} __attribute__ ((__packed__));

#endif
