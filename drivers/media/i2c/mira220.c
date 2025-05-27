// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA220 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-cci.h>
#include <linux/unaligned.h>

/* Active pixel array is 1600 (H) x 1400 (V) pixels.
 * Physical resolution including buffer pixels: 1642 (H) x 1464 (V) pixels.
 */
#define MIRA220_NATIVE_WIDTH 1642U
#define MIRA220_NATIVE_HEIGHT 1464U
#define MIRA220_PIXEL_ARRAY_LEFT 21U
#define MIRA220_PIXEL_ARRAY_TOP 32U
#define MIRA220_PIXEL_ARRAY_WIDTH 1600U
#define MIRA220_PIXEL_ARRAY_HEIGHT 1400U

/* Mira220 does not support analog gain. */
#define MIRA220_ANALOG_GAIN_MIN 1
#define MIRA220_ANALOG_GAIN_MAX 1
#define MIRA220_ANALOG_GAIN_STEP 1
#define MIRA220_ANALOG_GAIN_DEFAULT MIRA220_ANALOG_GAIN_MIN

/* Bit depth */
#define MIRA220_BIT_DEPTH_REG CCI_REG8(0x209E)
#define MIRA220_BIT_DEPTH_12_BIT 0x02
#define MIRA220_BIT_DEPTH_10_BIT 0x04
#define MIRA220_BIT_DEPTH_8_BIT 0x06
#define MIRA220_CSI_DATA_TYPE_REG CCI_REG8(0x208D)
#define MIRA220_CSI_DATA_TYPE_12_BIT 0x04
#define MIRA220_CSI_DATA_TYPE_10_BIT 0x02
#define MIRA220_CSI_DATA_TYPE_8_BIT 0x01

/* Imager state master/slave registers */
#define MIRA220_IMAGER_STATE_REG CCI_REG8(0x1003)
#define MIRA220_IMAGER_STATE_STOP_AT_ROW 0x02
#define MIRA220_IMAGER_STATE_STOP_AT_FRAME 0x04
#define MIRA220_IMAGER_STATE_MASTER_CONTROL 0x10
#define MIRA220_IMAGER_STATE_SLAVE_CONTROL 0x08

/* Start image acquisition */
#define MIRA220_IMAGER_RUN_REG CCI_REG8(0x10F0)
#define MIRA220_IMAGER_RUN_START 0x01
#define MIRA220_IMAGER_RUN_STOP 0x00

/* Continuous running, not limited to nr of frames. */
#define MIRA220_IMAGER_RUN_CONT_REG CCI_REG8(0x1002)
#define MIRA220_IMAGER_RUN_CONT_ENABLE 0x04
#define MIRA220_IMAGER_RUN_CONT_DISABLE 0x00

/* Exposure time is indicated in number of rows */
#define MIRA220_EXP_TIME_REG CCI_REG16_LE(0x100C)

/* Vertical Blank */
#define MIRA220_VBLANK_REG CCI_REG16_LE(0x1012)

/* Horizontal flip */
#define MIRA220_HFLIP_REG CCI_REG8(0x209C)
#define MIRA220_HFLIP_ENABLE_MIRROR 1
#define MIRA220_HFLIP_DISABLE_MIRROR 0

/* Vertical flip */
#define MIRA220_VFLIP_REG CCI_REG8(0x1095)
#define MIRA220_VFLIP_ENABLE_FLIP 1
#define MIRA220_VFLIP_DISABLE_FLIP 0

/* OTP control */
#define MIRA220_OTP_CMD_REG CCI_REG8(0x0080)
#define MIRA220_OTP_CMD_UP 0x4
#define MIRA220_OTP_CMD_DOWN 0x8

/* Global sampling time */
#define MIRA220_GLOB_NUM_CLK_CYCLES 1928

/* External clock frequency is 38.4 M */
#define MIRA220_SUPPORTED_XCLK_FREQ 38400000

// Default exposure is adjusted to mode with smallest height
#define MIRA220_DEFAULT_EXPOSURE 1000
#define MIRA220_EXPOSURE_MIN 1
// Power on function timing
#define MIRA220_XCLR_MIN_DELAY_US 100000
#define MIRA220_XCLR_DELAY_RANGE_US 30

/* Pixel rate is an artificial value
 * This value is used for timing calculations
 * in combination with vblank/hblank
 */
#define MIRA220_PIXEL_RATE 384000000 //384M (x10)

#define MIRA220_HBLANK_1600x1400_304 1440

/* Test Pattern */
#define MIRA220_REG_TEST_PATTERN CCI_REG8(0x2091)
#define MIRA220_TEST_PATTERN_DISABLE 0x00
#define MIRA220_TEST_PATTERN_VERTICAL_GRADIENT 0x01

struct mira220_reg {
	u16 address;
	u8 val;
};

struct mira220_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};


/* Mode : resolution and related config&values */
struct mira220_mode {
	unsigned int width;
	unsigned int height;
	struct v4l2_rect crop;
	struct mira220_reg_list reg_list;
	u32 row_length;
	u32 pixel_rate;
	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 code;
};

static const struct cci_reg_sequence full_1600_1400_1500_12b_2lanes_reg_new[] = {
	/* Base configuration*/
	{ CCI_REG8(0x1003), 0x2 },
	{ CCI_REG8(0x6006), 0x0 },
	{ CCI_REG8(0x6012), 0x1 },
	{ CCI_REG8(0x6013), 0x0 },
	{ CCI_REG8(0x6006), 0x1 },
	{ CCI_REG8(0x205D), 0x0 },
	{ CCI_REG8(0x2063), 0x0 },
	{ CCI_REG8(0x24DC), 0x13 },
	{ CCI_REG8(0x24DD), 0x3 },
	{ CCI_REG8(0x24DE), 0x3 },
	{ CCI_REG8(0x24DF), 0x0 },
	{ CCI_REG8(0x4006), 0x8 },
	{ CCI_REG8(0x401C), 0x6F },
	{ CCI_REG8(0x204B), 0x3 },
	{ CCI_REG8(0x205B), 0x64 },
	{ CCI_REG8(0x205C), 0x0 },
	{ CCI_REG8(0x4018), 0x3F },
	{ CCI_REG8(0x403B), 0xB },
	{ CCI_REG8(0x403E), 0xE },
	{ CCI_REG8(0x402B), 0x6 },
	{ CCI_REG8(0x401E), 0x2 },
	{ CCI_REG8(0x4038), 0x3B },
	{ CCI_REG8(0x1077), 0x0 },
	{ CCI_REG8(0x1078), 0x0 },
	{ CCI_REG8(0x1009), 0x8 },
	{ CCI_REG8(0x100A), 0x0 },
	{ CCI_REG8(0x110F), 0x8 },
	{ CCI_REG8(0x1110), 0x0 },
	{ CCI_REG8(0x1006), 0x2 },
	{ CCI_REG8(0x402C), 0x64 },
	{ CCI_REG8(0x3064), 0x0 },
	{ CCI_REG8(0x3065), 0xF0 },
	{ CCI_REG8(0x4013), 0x13 },
	{ CCI_REG8(0x401F), 0x9 },
	{ CCI_REG8(0x4020), 0x13 },
	{ CCI_REG8(0x4044), 0x75 },
	{ CCI_REG8(0x4027), 0x0 },
	{ CCI_REG8(0x3215), 0x69 },
	{ CCI_REG8(0x3216), 0xF },
	{ CCI_REG8(0x322B), 0x69 },
	{ CCI_REG8(0x322C), 0xF },
	{ CCI_REG8(0x4051), 0x80 },
	{ CCI_REG8(0x4052), 0x10 },
	{ CCI_REG8(0x4057), 0x80 },
	{ CCI_REG8(0x4058), 0x10 },
	{ CCI_REG8(0x3212), 0x59 },
	{ CCI_REG8(0x4047), 0x8F },
	{ CCI_REG8(0x4026), 0x10 },
	{ CCI_REG8(0x4032), 0x53 },
	{ CCI_REG8(0x4036), 0x17 },
	{ CCI_REG8(0x50B8), 0xF4 },
	{ CCI_REG8(0x3016), 0x0 },
	{ CCI_REG8(0x3017), 0x2C },
	{ CCI_REG8(0x3018), 0x8C },
	{ CCI_REG8(0x3019), 0x45 },
	{ CCI_REG8(0x301A), 0x5 },
	{ CCI_REG8(0x3013), 0xA },
	{ CCI_REG8(0x301B), 0x0 },
	{ CCI_REG8(0x301C), 0x4 },
	{ CCI_REG8(0x301D), 0x88 },
	{ CCI_REG8(0x301E), 0x45 },
	{ CCI_REG8(0x301F), 0x5 },
	{ CCI_REG8(0x3020), 0x0 },
	{ CCI_REG8(0x3021), 0x4 },
	{ CCI_REG8(0x3022), 0x88 },
	{ CCI_REG8(0x3023), 0x45 },
	{ CCI_REG8(0x3024), 0x5 },
	{ CCI_REG8(0x3025), 0x0 },
	{ CCI_REG8(0x3026), 0x4 },
	{ CCI_REG8(0x3027), 0x88 },
	{ CCI_REG8(0x3028), 0x45 },
	{ CCI_REG8(0x3029), 0x5 },
	{ CCI_REG8(0x302F), 0x0 },
	{ CCI_REG8(0x3056), 0x0 },
	{ CCI_REG8(0x3057), 0x0 },
	{ CCI_REG8(0x3300), 0x1 },
	{ CCI_REG8(0x3301), 0x0 },
	{ CCI_REG8(0x3302), 0xB0 },
	{ CCI_REG8(0x3303), 0xB0 },
	{ CCI_REG8(0x3304), 0x16 },
	{ CCI_REG8(0x3305), 0x15 },
	{ CCI_REG8(0x3306), 0x1 },
	{ CCI_REG8(0x3307), 0x0 },
	{ CCI_REG8(0x3308), 0x30 },
	{ CCI_REG8(0x3309), 0xA0 },
	{ CCI_REG8(0x330A), 0x16 },
	{ CCI_REG8(0x330B), 0x15 },
	{ CCI_REG8(0x330C), 0x1 },
	{ CCI_REG8(0x330D), 0x0 },
	{ CCI_REG8(0x330E), 0x30 },
	{ CCI_REG8(0x330F), 0xA0 },
	{ CCI_REG8(0x3310), 0x16 },
	{ CCI_REG8(0x3311), 0x15 },
	{ CCI_REG8(0x3312), 0x1 },
	{ CCI_REG8(0x3313), 0x0 },
	{ CCI_REG8(0x3314), 0x30 },
	{ CCI_REG8(0x3315), 0xA0 },
	{ CCI_REG8(0x3316), 0x16 },
	{ CCI_REG8(0x3317), 0x15 },
	{ CCI_REG8(0x3318), 0x1 },
	{ CCI_REG8(0x3319), 0x0 },
	{ CCI_REG8(0x331A), 0x30 },
	{ CCI_REG8(0x331B), 0xA0 },
	{ CCI_REG8(0x331C), 0x16 },
	{ CCI_REG8(0x331D), 0x15 },
	{ CCI_REG8(0x331E), 0x1 },
	{ CCI_REG8(0x331F), 0x0 },
	{ CCI_REG8(0x3320), 0x30 },
	{ CCI_REG8(0x3321), 0xA0 },
	{ CCI_REG8(0x3322), 0x16 },
	{ CCI_REG8(0x3323), 0x15 },
	{ CCI_REG8(0x3324), 0x1 },
	{ CCI_REG8(0x3325), 0x0 },
	{ CCI_REG8(0x3326), 0x30 },
	{ CCI_REG8(0x3327), 0xA0 },
	{ CCI_REG8(0x3328), 0x16 },
	{ CCI_REG8(0x3329), 0x15 },
	{ CCI_REG8(0x332A), 0x2B },
	{ CCI_REG8(0x332B), 0x0 },
	{ CCI_REG8(0x332C), 0x30 },
	{ CCI_REG8(0x332D), 0xA0 },
	{ CCI_REG8(0x332E), 0x16 },
	{ CCI_REG8(0x332F), 0x15 },
	{ CCI_REG8(0x3330), 0x1 },
	{ CCI_REG8(0x3331), 0x0 },
	{ CCI_REG8(0x3332), 0x10 },
	{ CCI_REG8(0x3333), 0xA0 },
	{ CCI_REG8(0x3334), 0x16 },
	{ CCI_REG8(0x3335), 0x15 },
	{ CCI_REG8(0x3058), 0x8 },
	{ CCI_REG8(0x3059), 0x0 },
	{ CCI_REG8(0x305A), 0x9 },
	{ CCI_REG8(0x305B), 0x0 },
	{ CCI_REG8(0x3336), 0x1 },
	{ CCI_REG8(0x3337), 0x0 },
	{ CCI_REG8(0x3338), 0x90 },
	{ CCI_REG8(0x3339), 0xB0 },
	{ CCI_REG8(0x333A), 0x16 },
	{ CCI_REG8(0x333B), 0x15 },
	{ CCI_REG8(0x333C), 0x1F },
	{ CCI_REG8(0x333D), 0x0 },
	{ CCI_REG8(0x333E), 0x10 },
	{ CCI_REG8(0x333F), 0xA0 },
	{ CCI_REG8(0x3340), 0x16 },
	{ CCI_REG8(0x3341), 0x15 },
	{ CCI_REG8(0x3342), 0x52 },
	{ CCI_REG8(0x3343), 0x0 },
	{ CCI_REG8(0x3344), 0x10 },
	{ CCI_REG8(0x3345), 0x80 },
	{ CCI_REG8(0x3346), 0x16 },
	{ CCI_REG8(0x3347), 0x15 },
	{ CCI_REG8(0x3348), 0x1 },
	{ CCI_REG8(0x3349), 0x0 },
	{ CCI_REG8(0x334A), 0x10 },
	{ CCI_REG8(0x334B), 0x80 },
	{ CCI_REG8(0x334C), 0x16 },
	{ CCI_REG8(0x334D), 0x1D },
	{ CCI_REG8(0x334E), 0x1 },
	{ CCI_REG8(0x334F), 0x0 },
	{ CCI_REG8(0x3350), 0x50 },
	{ CCI_REG8(0x3351), 0x84 },
	{ CCI_REG8(0x3352), 0x16 },
	{ CCI_REG8(0x3353), 0x1D },
	{ CCI_REG8(0x3354), 0x18 },
	{ CCI_REG8(0x3355), 0x0 },
	{ CCI_REG8(0x3356), 0x10 },
	{ CCI_REG8(0x3357), 0x84 },
	{ CCI_REG8(0x3358), 0x16 },
	{ CCI_REG8(0x3359), 0x1D },
	{ CCI_REG8(0x335A), 0x80 },
	{ CCI_REG8(0x335B), 0x2 },
	{ CCI_REG8(0x335C), 0x10 },
	{ CCI_REG8(0x335D), 0xC4 },
	{ CCI_REG8(0x335E), 0x14 },
	{ CCI_REG8(0x335F), 0x1D },
	{ CCI_REG8(0x3360), 0xA5 },
	{ CCI_REG8(0x3361), 0x0 },
	{ CCI_REG8(0x3362), 0x10 },
	{ CCI_REG8(0x3363), 0x84 },
	{ CCI_REG8(0x3364), 0x16 },
	{ CCI_REG8(0x3365), 0x1D },
	{ CCI_REG8(0x3366), 0x1 },
	{ CCI_REG8(0x3367), 0x0 },
	{ CCI_REG8(0x3368), 0x90 },
	{ CCI_REG8(0x3369), 0x84 },
	{ CCI_REG8(0x336A), 0x16 },
	{ CCI_REG8(0x336B), 0x1D },
	{ CCI_REG8(0x336C), 0x12 },
	{ CCI_REG8(0x336D), 0x0 },
	{ CCI_REG8(0x336E), 0x10 },
	{ CCI_REG8(0x336F), 0x84 },
	{ CCI_REG8(0x3370), 0x16 },
	{ CCI_REG8(0x3371), 0x15 },
	{ CCI_REG8(0x3372), 0x32 },
	{ CCI_REG8(0x3373), 0x0 },
	{ CCI_REG8(0x3374), 0x30 },
	{ CCI_REG8(0x3375), 0x84 },
	{ CCI_REG8(0x3376), 0x16 },
	{ CCI_REG8(0x3377), 0x15 },
	{ CCI_REG8(0x3378), 0x26 },
	{ CCI_REG8(0x3379), 0x0 },
	{ CCI_REG8(0x337A), 0x10 },
	{ CCI_REG8(0x337B), 0x84 },
	{ CCI_REG8(0x337C), 0x16 },
	{ CCI_REG8(0x337D), 0x15 },
	{ CCI_REG8(0x337E), 0x80 },
	{ CCI_REG8(0x337F), 0x2 },
	{ CCI_REG8(0x3380), 0x10 },
	{ CCI_REG8(0x3381), 0xC4 },
	{ CCI_REG8(0x3382), 0x14 },
	{ CCI_REG8(0x3383), 0x15 },
	{ CCI_REG8(0x3384), 0xA9 },
	{ CCI_REG8(0x3385), 0x0 },
	{ CCI_REG8(0x3386), 0x10 },
	{ CCI_REG8(0x3387), 0x84 },
	{ CCI_REG8(0x3388), 0x16 },
	{ CCI_REG8(0x3389), 0x15 },
	{ CCI_REG8(0x338A), 0x41 },
	{ CCI_REG8(0x338B), 0x0 },
	{ CCI_REG8(0x338C), 0x10 },
	{ CCI_REG8(0x338D), 0x80 },
	{ CCI_REG8(0x338E), 0x16 },
	{ CCI_REG8(0x338F), 0x15 },
	{ CCI_REG8(0x3390), 0x2 },
	{ CCI_REG8(0x3391), 0x0 },
	{ CCI_REG8(0x3392), 0x10 },
	{ CCI_REG8(0x3393), 0xA0 },
	{ CCI_REG8(0x3394), 0x16 },
	{ CCI_REG8(0x3395), 0x15 },
	{ CCI_REG8(0x305C), 0x18 },
	{ CCI_REG8(0x305D), 0x0 },
	{ CCI_REG8(0x305E), 0x19 },
	{ CCI_REG8(0x305F), 0x0 },
	{ CCI_REG8(0x3396), 0x1 },
	{ CCI_REG8(0x3397), 0x0 },
	{ CCI_REG8(0x3398), 0x90 },
	{ CCI_REG8(0x3399), 0x30 },
	{ CCI_REG8(0x339A), 0x56 },
	{ CCI_REG8(0x339B), 0x57 },
	{ CCI_REG8(0x339C), 0x1 },
	{ CCI_REG8(0x339D), 0x0 },
	{ CCI_REG8(0x339E), 0x10 },
	{ CCI_REG8(0x339F), 0x20 },
	{ CCI_REG8(0x33A0), 0xD6 },
	{ CCI_REG8(0x33A1), 0x17 },
	{ CCI_REG8(0x33A2), 0x1 },
	{ CCI_REG8(0x33A3), 0x0 },
	{ CCI_REG8(0x33A4), 0x10 },
	{ CCI_REG8(0x33A5), 0x28 },
	{ CCI_REG8(0x33A6), 0xD6 },
	{ CCI_REG8(0x33A7), 0x17 },
	{ CCI_REG8(0x33A8), 0x3 },
	{ CCI_REG8(0x33A9), 0x0 },
	{ CCI_REG8(0x33AA), 0x10 },
	{ CCI_REG8(0x33AB), 0x20 },
	{ CCI_REG8(0x33AC), 0xD6 },
	{ CCI_REG8(0x33AD), 0x17 },
	{ CCI_REG8(0x33AE), 0x61 },
	{ CCI_REG8(0x33AF), 0x0 },
	{ CCI_REG8(0x33B0), 0x10 },
	{ CCI_REG8(0x33B1), 0x20 },
	{ CCI_REG8(0x33B2), 0xD6 },
	{ CCI_REG8(0x33B3), 0x15 },
	{ CCI_REG8(0x33B4), 0x1 },
	{ CCI_REG8(0x33B5), 0x0 },
	{ CCI_REG8(0x33B6), 0x10 },
	{ CCI_REG8(0x33B7), 0x20 },
	{ CCI_REG8(0x33B8), 0xD6 },
	{ CCI_REG8(0x33B9), 0x1D },
	{ CCI_REG8(0x33BA), 0x1 },
	{ CCI_REG8(0x33BB), 0x0 },
	{ CCI_REG8(0x33BC), 0x50 },
	{ CCI_REG8(0x33BD), 0x20 },
	{ CCI_REG8(0x33BE), 0xD6 },
	{ CCI_REG8(0x33BF), 0x1D },
	{ CCI_REG8(0x33C0), 0x2C },
	{ CCI_REG8(0x33C1), 0x0 },
	{ CCI_REG8(0x33C2), 0x10 },
	{ CCI_REG8(0x33C3), 0x20 },
	{ CCI_REG8(0x33C4), 0xD6 },
	{ CCI_REG8(0x33C5), 0x1D },
	{ CCI_REG8(0x33C6), 0x1 },
	{ CCI_REG8(0x33C7), 0x0 },
	{ CCI_REG8(0x33C8), 0x90 },
	{ CCI_REG8(0x33C9), 0x20 },
	{ CCI_REG8(0x33CA), 0xD6 },
	{ CCI_REG8(0x33CB), 0x1D },
	{ CCI_REG8(0x33CC), 0x83 },
	{ CCI_REG8(0x33CD), 0x0 },
	{ CCI_REG8(0x33CE), 0x10 },
	{ CCI_REG8(0x33CF), 0x20 },
	{ CCI_REG8(0x33D0), 0xD6 },
	{ CCI_REG8(0x33D1), 0x15 },
	{ CCI_REG8(0x33D2), 0x1 },
	{ CCI_REG8(0x33D3), 0x0 },
	{ CCI_REG8(0x33D4), 0x10 },
	{ CCI_REG8(0x33D5), 0x30 },
	{ CCI_REG8(0x33D6), 0xD6 },
	{ CCI_REG8(0x33D7), 0x15 },
	{ CCI_REG8(0x33D8), 0x1 },
	{ CCI_REG8(0x33D9), 0x0 },
	{ CCI_REG8(0x33DA), 0x10 },
	{ CCI_REG8(0x33DB), 0x20 },
	{ CCI_REG8(0x33DC), 0xD6 },
	{ CCI_REG8(0x33DD), 0x15 },
	{ CCI_REG8(0x33DE), 0x1 },
	{ CCI_REG8(0x33DF), 0x0 },
	{ CCI_REG8(0x33E0), 0x10 },
	{ CCI_REG8(0x33E1), 0x20 },
	{ CCI_REG8(0x33E2), 0x56 },
	{ CCI_REG8(0x33E3), 0x15 },
	{ CCI_REG8(0x33E4), 0x7 },
	{ CCI_REG8(0x33E5), 0x0 },
	{ CCI_REG8(0x33E6), 0x10 },
	{ CCI_REG8(0x33E7), 0x20 },
	{ CCI_REG8(0x33E8), 0x16 },
	{ CCI_REG8(0x33E9), 0x15 },
	{ CCI_REG8(0x3060), 0x26 },
	{ CCI_REG8(0x3061), 0x0 },
	{ CCI_REG8(0x302A), 0xFF },
	{ CCI_REG8(0x302B), 0xFF },
	{ CCI_REG8(0x302C), 0xFF },
	{ CCI_REG8(0x302D), 0xFF },
	{ CCI_REG8(0x302E), 0x3F },
	{ CCI_REG8(0x3013), 0xB },
	{ CCI_REG8(0x102B), 0x2C },
	{ CCI_REG8(0x102C), 0x1 },
	{ CCI_REG8(0x1035), 0x54 },
	{ CCI_REG8(0x1036), 0x0 },
	{ CCI_REG8(0x3090), 0x2A },
	{ CCI_REG8(0x3091), 0x1 },
	{ CCI_REG8(0x30C6), 0x5 },
	{ CCI_REG8(0x30C7), 0x0 },
	{ CCI_REG8(0x30C8), 0x0 },
	{ CCI_REG8(0x30C9), 0x0 },
	{ CCI_REG8(0x30CA), 0x0 },
	{ CCI_REG8(0x30CB), 0x0 },
	{ CCI_REG8(0x30CC), 0x0 },
	{ CCI_REG8(0x30CD), 0x0 },
	{ CCI_REG8(0x30CE), 0x0 },
	{ CCI_REG8(0x30CF), 0x5 },
	{ CCI_REG8(0x30D0), 0x0 },
	{ CCI_REG8(0x30D1), 0x0 },
	{ CCI_REG8(0x30D2), 0x0 },
	{ CCI_REG8(0x30D3), 0x0 },
	{ CCI_REG8(0x30D4), 0x0 },
	{ CCI_REG8(0x30D5), 0x0 },
	{ CCI_REG8(0x30D6), 0x0 },
	{ CCI_REG8(0x30D7), 0x0 },
	{ CCI_REG8(0x30F3), 0x5 },
	{ CCI_REG8(0x30F4), 0x0 },
	{ CCI_REG8(0x30F5), 0x0 },
	{ CCI_REG8(0x30F6), 0x0 },
	{ CCI_REG8(0x30F7), 0x0 },
	{ CCI_REG8(0x30F8), 0x0 },
	{ CCI_REG8(0x30F9), 0x0 },
	{ CCI_REG8(0x30FA), 0x0 },
	{ CCI_REG8(0x30FB), 0x0 },
	{ CCI_REG8(0x30D8), 0x5 },
	{ CCI_REG8(0x30D9), 0x0 },
	{ CCI_REG8(0x30DA), 0x0 },
	{ CCI_REG8(0x30DB), 0x0 },
	{ CCI_REG8(0x30DC), 0x0 },
	{ CCI_REG8(0x30DD), 0x0 },
	{ CCI_REG8(0x30DE), 0x0 },
	{ CCI_REG8(0x30DF), 0x0 },
	{ CCI_REG8(0x30E0), 0x0 },
	{ CCI_REG8(0x30E1), 0x5 },
	{ CCI_REG8(0x30E2), 0x0 },
	{ CCI_REG8(0x30E3), 0x0 },
	{ CCI_REG8(0x30E4), 0x0 },
	{ CCI_REG8(0x30E5), 0x0 },
	{ CCI_REG8(0x30E6), 0x0 },
	{ CCI_REG8(0x30E7), 0x0 },
	{ CCI_REG8(0x30E8), 0x0 },
	{ CCI_REG8(0x30E9), 0x0 },
	{ CCI_REG8(0x30F3), 0x5 },
	{ CCI_REG8(0x30F4), 0x2 },
	{ CCI_REG8(0x30F5), 0x0 },
	{ CCI_REG8(0x30F6), 0x17 },
	{ CCI_REG8(0x30F7), 0x1 },
	{ CCI_REG8(0x30F8), 0x0 },
	{ CCI_REG8(0x30F9), 0x0 },
	{ CCI_REG8(0x30FA), 0x0 },
	{ CCI_REG8(0x30FB), 0x0 },
	{ CCI_REG8(0x30D8), 0x3 },
	{ CCI_REG8(0x30D9), 0x1 },
	{ CCI_REG8(0x30DA), 0x0 },
	{ CCI_REG8(0x30DB), 0x19 },
	{ CCI_REG8(0x30DC), 0x1 },
	{ CCI_REG8(0x30DD), 0x0 },
	{ CCI_REG8(0x30DE), 0x0 },
	{ CCI_REG8(0x30DF), 0x0 },
	{ CCI_REG8(0x30E0), 0x0 },
	{ CCI_REG8(0x30A2), 0x5 },
	{ CCI_REG8(0x30A3), 0x2 },
	{ CCI_REG8(0x30A4), 0x0 },
	{ CCI_REG8(0x30A5), 0x22 },
	{ CCI_REG8(0x30A6), 0x0 },
	{ CCI_REG8(0x30A7), 0x0 },
	{ CCI_REG8(0x30A8), 0x0 },
	{ CCI_REG8(0x30A9), 0x0 },
	{ CCI_REG8(0x30AA), 0x0 },
	{ CCI_REG8(0x30AB), 0x5 },
	{ CCI_REG8(0x30AC), 0x2 },
	{ CCI_REG8(0x30AD), 0x0 },
	{ CCI_REG8(0x30AE), 0x22 },
	{ CCI_REG8(0x30AF), 0x0 },
	{ CCI_REG8(0x30B0), 0x0 },
	{ CCI_REG8(0x30B1), 0x0 },
	{ CCI_REG8(0x30B2), 0x0 },
	{ CCI_REG8(0x30B3), 0x0 },
	{ CCI_REG8(0x30BD), 0x5 },
	{ CCI_REG8(0x30BE), 0x9F },
	{ CCI_REG8(0x30BF), 0x0 },
	{ CCI_REG8(0x30C0), 0x7D },
	{ CCI_REG8(0x30C1), 0x0 },
	{ CCI_REG8(0x30C2), 0x0 },
	{ CCI_REG8(0x30C3), 0x0 },
	{ CCI_REG8(0x30C4), 0x0 },
	{ CCI_REG8(0x30C5), 0x0 },
	{ CCI_REG8(0x30B4), 0x4 },
	{ CCI_REG8(0x30B5), 0x9C },
	{ CCI_REG8(0x30B6), 0x0 },
	{ CCI_REG8(0x30B7), 0x7D },
	{ CCI_REG8(0x30B8), 0x0 },
	{ CCI_REG8(0x30B9), 0x0 },
	{ CCI_REG8(0x30BA), 0x0 },
	{ CCI_REG8(0x30BB), 0x0 },
	{ CCI_REG8(0x30BC), 0x0 },
	{ CCI_REG8(0x30FC), 0x5 },
	{ CCI_REG8(0x30FD), 0x0 },
	{ CCI_REG8(0x30FE), 0x0 },
	{ CCI_REG8(0x30FF), 0x0 },
	{ CCI_REG8(0x3100), 0x0 },
	{ CCI_REG8(0x3101), 0x0 },
	{ CCI_REG8(0x3102), 0x0 },
	{ CCI_REG8(0x3103), 0x0 },
	{ CCI_REG8(0x3104), 0x0 },
	{ CCI_REG8(0x3105), 0x5 },
	{ CCI_REG8(0x3106), 0x0 },
	{ CCI_REG8(0x3107), 0x0 },
	{ CCI_REG8(0x3108), 0x0 },
	{ CCI_REG8(0x3109), 0x0 },
	{ CCI_REG8(0x310A), 0x0 },
	{ CCI_REG8(0x310B), 0x0 },
	{ CCI_REG8(0x310C), 0x0 },
	{ CCI_REG8(0x310D), 0x0 },
	{ CCI_REG8(0x3099), 0x5 },
	{ CCI_REG8(0x309A), 0x96 },
	{ CCI_REG8(0x309B), 0x0 },
	{ CCI_REG8(0x309C), 0x6 },
	{ CCI_REG8(0x309D), 0x0 },
	{ CCI_REG8(0x309E), 0x0 },
	{ CCI_REG8(0x309F), 0x0 },
	{ CCI_REG8(0x30A0), 0x0 },
	{ CCI_REG8(0x30A1), 0x0 },
	{ CCI_REG8(0x310E), 0x5 },
	{ CCI_REG8(0x310F), 0x2 },
	{ CCI_REG8(0x3110), 0x0 },
	{ CCI_REG8(0x3111), 0x2B },
	{ CCI_REG8(0x3112), 0x0 },
	{ CCI_REG8(0x3113), 0x0 },
	{ CCI_REG8(0x3114), 0x0 },
	{ CCI_REG8(0x3115), 0x0 },
	{ CCI_REG8(0x3116), 0x0 },
	{ CCI_REG8(0x3117), 0x5 },
	{ CCI_REG8(0x3118), 0x2 },
	{ CCI_REG8(0x3119), 0x0 },
	{ CCI_REG8(0x311A), 0x2C },
	{ CCI_REG8(0x311B), 0x0 },
	{ CCI_REG8(0x311C), 0x0 },
	{ CCI_REG8(0x311D), 0x0 },
	{ CCI_REG8(0x311E), 0x0 },
	{ CCI_REG8(0x311F), 0x0 },
	{ CCI_REG8(0x30EA), 0x0 },
	{ CCI_REG8(0x30EB), 0x0 },
	{ CCI_REG8(0x30EC), 0x0 },
	{ CCI_REG8(0x30ED), 0x0 },
	{ CCI_REG8(0x30EE), 0x0 },
	{ CCI_REG8(0x30EF), 0x0 },
	{ CCI_REG8(0x30F0), 0x0 },
	{ CCI_REG8(0x30F1), 0x0 },
	{ CCI_REG8(0x30F2), 0x0 },
	{ CCI_REG8(0x313B), 0x3 },
	{ CCI_REG8(0x313C), 0x31 },
	{ CCI_REG8(0x313D), 0x0 },
	{ CCI_REG8(0x313E), 0x7 },
	{ CCI_REG8(0x313F), 0x0 },
	{ CCI_REG8(0x3140), 0x68 },
	{ CCI_REG8(0x3141), 0x0 },
	{ CCI_REG8(0x3142), 0x34 },
	{ CCI_REG8(0x3143), 0x0 },
	{ CCI_REG8(0x31A0), 0x3 },
	{ CCI_REG8(0x31A1), 0x16 },
	{ CCI_REG8(0x31A2), 0x0 },
	{ CCI_REG8(0x31A3), 0x8 },
	{ CCI_REG8(0x31A4), 0x0 },
	{ CCI_REG8(0x31A5), 0x7E },
	{ CCI_REG8(0x31A6), 0x0 },
	{ CCI_REG8(0x31A7), 0x8 },
	{ CCI_REG8(0x31A8), 0x0 },
	{ CCI_REG8(0x31A9), 0x3 },
	{ CCI_REG8(0x31AA), 0x16 },
	{ CCI_REG8(0x31AB), 0x0 },
	{ CCI_REG8(0x31AC), 0x8 },
	{ CCI_REG8(0x31AD), 0x0 },
	{ CCI_REG8(0x31AE), 0x7E },
	{ CCI_REG8(0x31AF), 0x0 },
	{ CCI_REG8(0x31B0), 0x8 },
	{ CCI_REG8(0x31B1), 0x0 },
	{ CCI_REG8(0x31B2), 0x3 },
	{ CCI_REG8(0x31B3), 0x16 },
	{ CCI_REG8(0x31B4), 0x0 },
	{ CCI_REG8(0x31B5), 0x8 },
	{ CCI_REG8(0x31B6), 0x0 },
	{ CCI_REG8(0x31B7), 0x7E },
	{ CCI_REG8(0x31B8), 0x0 },
	{ CCI_REG8(0x31B9), 0x8 },
	{ CCI_REG8(0x31BA), 0x0 },
	{ CCI_REG8(0x3120), 0x5 },
	{ CCI_REG8(0x3121), 0x45 },
	{ CCI_REG8(0x3122), 0x0 },
	{ CCI_REG8(0x3123), 0x1D },
	{ CCI_REG8(0x3124), 0x0 },
	{ CCI_REG8(0x3125), 0xA9 },
	{ CCI_REG8(0x3126), 0x0 },
	{ CCI_REG8(0x3127), 0x6D },
	{ CCI_REG8(0x3128), 0x0 },
	{ CCI_REG8(0x3129), 0x5 },
	{ CCI_REG8(0x312A), 0x15 },
	{ CCI_REG8(0x312B), 0x0 },
	{ CCI_REG8(0x312C), 0xA },
	{ CCI_REG8(0x312D), 0x0 },
	{ CCI_REG8(0x312E), 0x45 },
	{ CCI_REG8(0x312F), 0x0 },
	{ CCI_REG8(0x3130), 0x1D },
	{ CCI_REG8(0x3131), 0x0 },
	{ CCI_REG8(0x3132), 0x5 },
	{ CCI_REG8(0x3133), 0x7D },
	{ CCI_REG8(0x3134), 0x0 },
	{ CCI_REG8(0x3135), 0xA },
	{ CCI_REG8(0x3136), 0x0 },
	{ CCI_REG8(0x3137), 0xA9 },
	{ CCI_REG8(0x3138), 0x0 },
	{ CCI_REG8(0x3139), 0x6D },
	{ CCI_REG8(0x313A), 0x0 },
	{ CCI_REG8(0x3144), 0x5 },
	{ CCI_REG8(0x3145), 0x0 },
	{ CCI_REG8(0x3146), 0x0 },
	{ CCI_REG8(0x3147), 0x30 },
	{ CCI_REG8(0x3148), 0x0 },
	{ CCI_REG8(0x3149), 0x0 },
	{ CCI_REG8(0x314A), 0x0 },
	{ CCI_REG8(0x314B), 0x0 },
	{ CCI_REG8(0x314C), 0x0 },
	{ CCI_REG8(0x314D), 0x3 },
	{ CCI_REG8(0x314E), 0x0 },
	{ CCI_REG8(0x314F), 0x0 },
	{ CCI_REG8(0x3150), 0x31 },
	{ CCI_REG8(0x3151), 0x0 },
	{ CCI_REG8(0x3152), 0x0 },
	{ CCI_REG8(0x3153), 0x0 },
	{ CCI_REG8(0x3154), 0x0 },
	{ CCI_REG8(0x3155), 0x0 },
	{ CCI_REG8(0x31D8), 0x5 },
	{ CCI_REG8(0x31D9), 0x3A },
	{ CCI_REG8(0x31DA), 0x0 },
	{ CCI_REG8(0x31DB), 0x2E },
	{ CCI_REG8(0x31DC), 0x0 },
	{ CCI_REG8(0x31DD), 0x9E },
	{ CCI_REG8(0x31DE), 0x0 },
	{ CCI_REG8(0x31DF), 0x7E },
	{ CCI_REG8(0x31E0), 0x0 },
	{ CCI_REG8(0x31E1), 0x5 },
	{ CCI_REG8(0x31E2), 0x4 },
	{ CCI_REG8(0x31E3), 0x0 },
	{ CCI_REG8(0x31E4), 0x4 },
	{ CCI_REG8(0x31E5), 0x0 },
	{ CCI_REG8(0x31E6), 0x73 },
	{ CCI_REG8(0x31E7), 0x0 },
	{ CCI_REG8(0x31E8), 0x4 },
	{ CCI_REG8(0x31E9), 0x0 },
	{ CCI_REG8(0x31EA), 0x5 },
	{ CCI_REG8(0x31EB), 0x0 },
	{ CCI_REG8(0x31EC), 0x0 },
	{ CCI_REG8(0x31ED), 0x0 },
	{ CCI_REG8(0x31EE), 0x0 },
	{ CCI_REG8(0x31EF), 0x0 },
	{ CCI_REG8(0x31F0), 0x0 },
	{ CCI_REG8(0x31F1), 0x0 },
	{ CCI_REG8(0x31F2), 0x0 },
	{ CCI_REG8(0x31F3), 0x0 },
	{ CCI_REG8(0x31F4), 0x0 },
	{ CCI_REG8(0x31F5), 0x0 },
	{ CCI_REG8(0x31F6), 0x0 },
	{ CCI_REG8(0x31F7), 0x0 },
	{ CCI_REG8(0x31F8), 0x0 },
	{ CCI_REG8(0x31F9), 0x0 },
	{ CCI_REG8(0x31FA), 0x0 },
	{ CCI_REG8(0x31FB), 0x5 },
	{ CCI_REG8(0x31FC), 0x0 },
	{ CCI_REG8(0x31FD), 0x0 },
	{ CCI_REG8(0x31FE), 0x0 },
	{ CCI_REG8(0x31FF), 0x0 },
	{ CCI_REG8(0x3200), 0x0 },
	{ CCI_REG8(0x3201), 0x0 },
	{ CCI_REG8(0x3202), 0x0 },
	{ CCI_REG8(0x3203), 0x0 },
	{ CCI_REG8(0x3204), 0x0 },
	{ CCI_REG8(0x3205), 0x0 },
	{ CCI_REG8(0x3206), 0x0 },
	{ CCI_REG8(0x3207), 0x0 },
	{ CCI_REG8(0x3208), 0x0 },
	{ CCI_REG8(0x3209), 0x0 },
	{ CCI_REG8(0x320A), 0x0 },
	{ CCI_REG8(0x320B), 0x0 },
	{ CCI_REG8(0x3164), 0x5 },
	{ CCI_REG8(0x3165), 0x14 },
	{ CCI_REG8(0x3166), 0x0 },
	{ CCI_REG8(0x3167), 0xC },
	{ CCI_REG8(0x3168), 0x0 },
	{ CCI_REG8(0x3169), 0x44 },
	{ CCI_REG8(0x316A), 0x0 },
	{ CCI_REG8(0x316B), 0x1F },
	{ CCI_REG8(0x316C), 0x0 },
	{ CCI_REG8(0x316D), 0x5 },
	{ CCI_REG8(0x316E), 0x7C },
	{ CCI_REG8(0x316F), 0x0 },
	{ CCI_REG8(0x3170), 0xC },
	{ CCI_REG8(0x3171), 0x0 },
	{ CCI_REG8(0x3172), 0xA8 },
	{ CCI_REG8(0x3173), 0x0 },
	{ CCI_REG8(0x3174), 0x6F },
	{ CCI_REG8(0x3175), 0x0 },
	{ CCI_REG8(0x31C4), 0x5 },
	{ CCI_REG8(0x31C5), 0x24 },
	{ CCI_REG8(0x31C6), 0x1 },
	{ CCI_REG8(0x31C7), 0x4 },
	{ CCI_REG8(0x31C8), 0x0 },
	{ CCI_REG8(0x31C9), 0x5 },
	{ CCI_REG8(0x31CA), 0x24 },
	{ CCI_REG8(0x31CB), 0x1 },
	{ CCI_REG8(0x31CC), 0x4 },
	{ CCI_REG8(0x31CD), 0x0 },
	{ CCI_REG8(0x31CE), 0x5 },
	{ CCI_REG8(0x31CF), 0x24 },
	{ CCI_REG8(0x31D0), 0x1 },
	{ CCI_REG8(0x31D1), 0x4 },
	{ CCI_REG8(0x31D2), 0x0 },
	{ CCI_REG8(0x31D3), 0x5 },
	{ CCI_REG8(0x31D4), 0x73 },
	{ CCI_REG8(0x31D5), 0x0 },
	{ CCI_REG8(0x31D6), 0xB1 },
	{ CCI_REG8(0x31D7), 0x0 },
	{ CCI_REG8(0x3176), 0x5 },
	{ CCI_REG8(0x3177), 0x10 },
	{ CCI_REG8(0x3178), 0x0 },
	{ CCI_REG8(0x3179), 0x56 },
	{ CCI_REG8(0x317A), 0x0 },
	{ CCI_REG8(0x317B), 0x0 },
	{ CCI_REG8(0x317C), 0x0 },
	{ CCI_REG8(0x317D), 0x0 },
	{ CCI_REG8(0x317E), 0x0 },
	{ CCI_REG8(0x317F), 0x5 },
	{ CCI_REG8(0x3180), 0x6A },
	{ CCI_REG8(0x3181), 0x0 },
	{ CCI_REG8(0x3182), 0xAD },
	{ CCI_REG8(0x3183), 0x0 },
	{ CCI_REG8(0x3184), 0x0 },
	{ CCI_REG8(0x3185), 0x0 },
	{ CCI_REG8(0x3186), 0x0 },
	{ CCI_REG8(0x3187), 0x0 },
	{ CCI_REG8(0x100C), 0x7E },
	{ CCI_REG8(0x100D), 0x0 },
	{ CCI_REG8(0x1012), 0xDF },
	{ CCI_REG8(0x1013), 0x2B },
	{ CCI_REG8(0x1002), 0x4 },
	/* Sensor control mode */
	{ CCI_REG8(0x0043), 0x0 }, //  Sensor Control Mode.SLEEP_POWER_MODE(0)
	{ CCI_REG8(0x0043), 0x0 }, //  Sensor Control Mode.IDLE_POWER_MODE(0)
	{ CCI_REG8(0x0043), 0x4 }, //  Sensor Control Mode.SYSTEM_CLOCK_ENABLE(0)
	{ CCI_REG8(0x0043), 0xC }, //  Sensor Control Mode.SRAM_CLOCK_ENABLE(0)
	{ CCI_REG8(0x1002), 0x4 }, //  Sensor Control Mode.IMAGER_RUN_CONT(0)
	{ CCI_REG8(0x1001), 0x41 }, //  Sensor Control Mode.EXT_EVENT_SEL(0)
	{ CCI_REG8(0x10F2), 0x1 }, //  Sensor Control Mode.NB_OF_FRAMES_A(0)
	{ CCI_REG8(0x10F3), 0x0 }, //  Sensor Control Mode.NB_OF_FRAMES_A(1)
	{ CCI_REG8(0x1111), 0x1 }, //  Sensor Control Mode.NB_OF_FRAMES_B(0)
	{ CCI_REG8(0x1112), 0x0 }, //  Sensor Control Mode.NB_OF_FRAMES_B(1)
	{ CCI_REG8(0x0012), 0x0 }, //  IO Drive Strength.DIG_DRIVE_STRENGTH(0)
	{ CCI_REG8(0x0012), 0x0 }, //  IO Drive Strength.CCI_DRIVE_STRENGTH(0)
	{ CCI_REG8(0x1001), 0x41 }, //  Readout && Exposure.EXT_EXP_PW_SEL(0)
	{ CCI_REG8(0x10D0), 0x0 }, //  Readout && Exposure.EXT_EXP_PW_DELAY(0)
	{ CCI_REG8(0x10D1), 0x0 }, //  Readout && Exposure.EXT_EXP_PW_DELAY(1)
	{ CCI_REG8(0x1012), 0x91 }, //  Readout && Exposure.VBLANK_A(0)
	{ CCI_REG8(0x1013), 0xD }, //  Readout && Exposure.VBLANK_A(1)
	{ CCI_REG8(0x1103), 0x91 }, //  Readout && Exposure.VBLANK_B(0)
	{ CCI_REG8(0x1104), 0xD }, //  Readout && Exposure.VBLANK_B(1)
	{ CCI_REG8(0x100C), 0x80 }, //  Readout && Exposure.EXP_TIME_A(0)
	{ CCI_REG8(0x100D), 0x0 }, //  Readout && Exposure.EXP_TIME_A(1)
	{ CCI_REG8(0x1115), 0x80 }, //  Readout && Exposure.EXP_TIME_B(0)
	{ CCI_REG8(0x1116), 0x0 }, //  Readout && Exposure.EXP_TIME_B(1)
	{ CCI_REG8(0x102B), 0x30 }, //  Readout && Exposure.ROW_LENGTH_A(0)
	{ CCI_REG8(0x102C), 0x1 }, //  Readout && Exposure.ROW_LENGTH_A(1)
	{ CCI_REG8(0x1113), 0x30 }, //  Readout && Exposure.ROW_LENGTH_B(0)
	{ CCI_REG8(0x1114), 0x1 }, //  Readout && Exposure.ROW_LENGTH_B(1)
	/* ROI */
	{ CCI_REG8(0x2008), 0x20 }, //  Horizontal ROI.HSIZE_A(0)
	{ CCI_REG8(0x2009), 0x3 }, //  Horizontal ROI.HSIZE_A(1)
	{ CCI_REG8(0x2098), 0x20 }, //  Horizontal ROI.HSIZE_B(0)
	{ CCI_REG8(0x2099), 0x3 }, //  Horizontal ROI.HSIZE_B(1)
	{ CCI_REG8(0x200A), 0x0 }, //  Horizontal ROI.HSTART_A(0)
	{ CCI_REG8(0x200B), 0x0 }, //  Horizontal ROI.HSTART_A(1)
	{ CCI_REG8(0x209A), 0x0 }, //  Horizontal ROI.HSTART_B(0)
	{ CCI_REG8(0x209B), 0x0 }, //  Horizontal ROI.HSTART_B(1)
	{ CCI_REG8(0x207D), 0x40 }, //  Horizontal ROI.MIPI_HSIZE(0)
	{ CCI_REG8(0x207E), 0x6 }, //  Horizontal ROI.MIPI_HSIZE(1)
	{ CCI_REG8(0x107D), 0x0 }, //  Vertical ROI.VSTART0_A(0)
	{ CCI_REG8(0x107E), 0x0 }, //  Vertical ROI.VSTART0_A(1)
	{ CCI_REG8(0x1087), 0x78 }, //  Vertical ROI.VSIZE0_A(0)
	{ CCI_REG8(0x1088), 0x5 }, //  Vertical ROI.VSIZE0_A(1)
	{ CCI_REG8(0x1105), 0x0 }, //  Vertical ROI.VSTART0_B(0)
	{ CCI_REG8(0x1106), 0x0 }, //  Vertical ROI.VSTART0_B(1)
	{ CCI_REG8(0x110A), 0x78 }, //  Vertical ROI.VSIZE0_B(0)
	{ CCI_REG8(0x110B), 0x5 }, //  Vertical ROI.VSIZE0_B(1)
	{ CCI_REG8(0x107D), 0x0 }, //  Vertical ROI.VSTART1_A(0)
	{ CCI_REG8(0x107E), 0x0 }, //  Vertical ROI.VSTART1_A(1)
	{ CCI_REG8(0x107F), 0x0 }, //  Vertical ROI.VSTART1_A(2)
	{ CCI_REG8(0x1087), 0x78 }, //  Vertical ROI.VSIZE1_A(0)
	{ CCI_REG8(0x1088), 0x5 }, //  Vertical ROI.VSIZE1_A(1)
	{ CCI_REG8(0x1089), 0x0 }, //  Vertical ROI.VSIZE1_A(2)
	{ CCI_REG8(0x1105), 0x0 }, //  Vertical ROI.VSTART1_B(0)
	{ CCI_REG8(0x1106), 0x0 }, //  Vertical ROI.VSTART1_B(1)
	{ CCI_REG8(0x1107), 0x0 }, //  Vertical ROI.VSTART1_B(2)
	{ CCI_REG8(0x110A), 0x78 }, //  Vertical ROI.VSIZE1_B(0)
	{ CCI_REG8(0x110B), 0x5 }, //  Vertical ROI.VSIZE1_B(1)
	{ CCI_REG8(0x110C), 0x0 }, //  Vertical ROI.VSIZE1_B(2)
	{ CCI_REG8(0x107D), 0x0 }, //  Vertical ROI.VSTART2_A(0)
	{ CCI_REG8(0x107E), 0x0 }, //  Vertical ROI.VSTART2_A(1)
	{ CCI_REG8(0x107F), 0x0 }, //  Vertical ROI.VSTART2_A(2)
	{ CCI_REG8(0x1080), 0x0 }, //  Vertical ROI.VSTART2_A(3)
	{ CCI_REG8(0x1081), 0x0 }, //  Vertical ROI.VSTART2_A(4)
	{ CCI_REG8(0x1087), 0x78 }, //  Vertical ROI.VSIZE2_A(0)
	{ CCI_REG8(0x1088), 0x5 }, //  Vertical ROI.VSIZE2_A(1)
	{ CCI_REG8(0x1089), 0x0 }, //  Vertical ROI.VSIZE2_A(2)
	{ CCI_REG8(0x108A), 0x0 }, //  Vertical ROI.VSIZE2_A(3)
	{ CCI_REG8(0x108B), 0x0 }, //  Vertical ROI.VSIZE2_A(4)
	{ CCI_REG8(0x1105), 0x0 }, //  Vertical ROI.VSTART2_B(0)
	{ CCI_REG8(0x1106), 0x0 }, //  Vertical ROI.VSTART2_B(1)
	{ CCI_REG8(0x1107), 0x0 }, //  Vertical ROI.VSTART2_B(2)
	{ CCI_REG8(0x1108), 0x0 }, //  Vertical ROI.VSTART2_B(3)
	{ CCI_REG8(0x1109), 0x0 }, //  Vertical ROI.VSTART2_B(4)
	{ CCI_REG8(0x110A), 0x78 }, //  Vertical ROI.VSIZE2_B(0)
	{ CCI_REG8(0x110B), 0x5 }, //  Vertical ROI.VSIZE2_B(1)
	{ CCI_REG8(0x110C), 0x0 }, //  Vertical ROI.VSIZE2_B(2)
	{ CCI_REG8(0x110D), 0x0 }, //  Vertical ROI.VSIZE2_B(3)
	{ CCI_REG8(0x110E), 0x0 }, //  Vertical ROI.VSIZE2_B(4)
	/* Mirror and Flip */
	{ CCI_REG8(0x209C), 0x0 }, //  Mirroring && Flipping.HFLIP_A(0)
	{ CCI_REG8(0x209D), 0x0 }, //  Mirroring && Flipping.HFLIP_B(0)
	{ CCI_REG8(0x1095), 0x0 }, //  Mirroring && Flipping.VFLIP(0)
	{ CCI_REG8(0x2063), 0x0 }, //  Mirroring && Flipping.BIT_ORDER(0)
	/* MIPI */
	{ CCI_REG8(0x6006), 0x0 }, //  MIPI.TX_CTRL_EN(0)
	{ CCI_REG8(0x5004), 0x1 }, //  MIPI.datarate
	{ CCI_REG8(0x5086), 0x2 }, //  MIPI.datarate
	{ CCI_REG8(0x5087), 0x4E }, //  MIPI.datarate
	{ CCI_REG8(0x5088), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x5090), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x5091), 0x8 }, //  MIPI.datarate
	{ CCI_REG8(0x5092), 0x14 }, //  MIPI.datarate
	{ CCI_REG8(0x5093), 0xF }, //  MIPI.datarate
	{ CCI_REG8(0x5094), 0x6 }, //  MIPI.datarate
	{ CCI_REG8(0x5095), 0x32 }, //  MIPI.datarate
	{ CCI_REG8(0x5096), 0xE }, //  MIPI.datarate
	{ CCI_REG8(0x5097), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x5098), 0x11 }, //  MIPI.datarate
	{ CCI_REG8(0x5004), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x2066), 0x6C }, //  MIPI.datarate
	{ CCI_REG8(0x2067), 0x7 }, //  MIPI.datarate
	{ CCI_REG8(0x206E), 0x7E }, //  MIPI.datarate
	{ CCI_REG8(0x206F), 0x6 }, //  MIPI.datarate
	{ CCI_REG8(0x20AC), 0x7E }, //  MIPI.datarate
	{ CCI_REG8(0x20AD), 0x6 }, //  MIPI.datarate
	{ CCI_REG8(0x2076), 0xC8 }, //  MIPI.datarate
	{ CCI_REG8(0x2077), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x20B4), 0xC8 }, //  MIPI.datarate
	{ CCI_REG8(0x20B5), 0x0 }, //  MIPI.datarate
	{ CCI_REG8(0x2078), 0x1E }, //  MIPI.datarate
	{ CCI_REG8(0x2079), 0x4 }, //  MIPI.datarate
	{ CCI_REG8(0x20B6), 0x1E }, //  MIPI.datarate
	{ CCI_REG8(0x20B7), 0x4 }, //  MIPI.datarate
	{ CCI_REG8(0x207A), 0xD4 }, //  MIPI.datarate
	{ CCI_REG8(0x207B), 0x4 }, //  MIPI.datarate
	{ CCI_REG8(0x20B8), 0xD4 }, //  MIPI.datarate
	{ CCI_REG8(0x20B9), 0x4 }, //  MIPI.datarate
	{ CCI_REG8(0x208D), 0x4 }, //  MIPI.CSI2_DTYPE(0)
	{ CCI_REG8(0x208E), 0x0 }, //  MIPI.CSI2_DTYPE(1)
	{ CCI_REG8(0x207C), 0x0 }, //  MIPI.VC_ID(0)
	{ CCI_REG8(0x6001), 0x7 }, //  MIPI.TINIT(0)
	{ CCI_REG8(0x6002), 0xD8 }, //  MIPI.TINIT(1)
	{ CCI_REG8(0x6010), 0x0 }, //  MIPI.FRAME_MODE(0)
	{ CCI_REG8(0x6010), 0x0 }, //  MIPI.EMBEDDED_FRAME_MODE(0)
	{ CCI_REG8(0x6011), 0x0 }, //  MIPI.DATA_ENABLE_POLARITY(0)
	{ CCI_REG8(0x6011), 0x0 }, //  MIPI.HSYNC_POLARITY(0)
	{ CCI_REG8(0x6011), 0x0 }, //  MIPI.VSYNC_POLARITY(0)
	{ CCI_REG8(0x6012), 0x1 }, //  MIPI.LANE(0)
	{ CCI_REG8(0x6013), 0x0 }, //  MIPI.CLK_MODE(0)
	{ CCI_REG8(0x6016), 0x0 }, //  MIPI.FRAME_COUNTER(0)
	{ CCI_REG8(0x6017), 0x0 }, //  MIPI.FRAME_COUNTER(1)
	{ CCI_REG8(0x6037), 0x1 }, //  MIPI.LINE_COUNT_RAW8(0)
	{ CCI_REG8(0x6037), 0x3 }, //  MIPI.LINE_COUNT_RAW10(0)
	{ CCI_REG8(0x6037), 0x7 }, //  MIPI.LINE_COUNT_RAW12(0)
	{ CCI_REG8(0x6039), 0x1 }, //  MIPI.LINE_COUNT_EMB(0)
	{ CCI_REG8(0x6018), 0x0 }, //  MIPI.CCI_READ_INTERRUPT_EN(0)
	{ CCI_REG8(0x6018), 0x0 }, //  MIPI.CCI_WRITE_INTERRUPT_EN(0)
	{ CCI_REG8(0x6065), 0x0 }, //  MIPI.TWAKE_TIMER(0)
	{ CCI_REG8(0x6066), 0x0 }, //  MIPI.TWAKE_TIMER(1)
	{ CCI_REG8(0x601C), 0x0 }, //  MIPI.SKEW_CAL_EN(0)
	{ CCI_REG8(0x601D), 0x0 }, //  MIPI.SKEW_COUNT(0)
	{ CCI_REG8(0x601E), 0x22 }, //  MIPI.SKEW_COUNT(1)
	{ CCI_REG8(0x601F), 0x0 }, //  MIPI.SCRAMBLING_EN(0)
	{ CCI_REG8(0x6003), 0x1 }, //  MIPI.INIT_SKEW_EN(0)
	{ CCI_REG8(0x6004), 0x7A }, //  MIPI.INIT_SKEW(0)
	{ CCI_REG8(0x6005), 0x12 }, //  MIPI.INIT_SKEW(1)
	{ CCI_REG8(0x6006), 0x1 }, //  MIPI.TX_CTRL_EN(0)
	/* Processing */
	{ CCI_REG8(0x4006), 0x8 }, //  Processing.BSP(0)
	{ CCI_REG8(0x209E), 0x2 }, //  Processing.BIT_DEPTH(0)
	{ CCI_REG8(0x2045), 0x1 }, //  Processing.CDS_RNC(0)
	{ CCI_REG8(0x2048), 0x1 }, //  Processing.CDS_IMG(0)
	{ CCI_REG8(0x204B), 0x3 }, //  Processing.RNC_EN(0)
	{ CCI_REG8(0x205B), 0x64 }, //  Processing.RNC_DARK_TARGET(0)
	{ CCI_REG8(0x205C), 0x0 }, //  Processing.RNC_DARK_TARGET(1)
	{ CCI_REG8(0x24DC), 0x12 }, //  Defect Pixel Correction.DC_ENABLE(0)
	{ CCI_REG8(0x24DC), 0x10 }, //  Defect Pixel Correction.DC_MODE(0)
	{ CCI_REG8(0x24DC),	0x0 }, //  Defect Pixel Correction.DC_REPLACEMENT_VALUE(0)
	{ CCI_REG8(0x24DD), 0x0 }, //  Defect Pixel Correction.DC_LIMIT_LOW(0)
	{ CCI_REG8(0x24DE), 0x0 }, //  Defect Pixel Correction.DC_LIMIT_HIGH(0)
	{ CCI_REG8(0x24DF),	0x0 }, //  Defect Pixel Correction.DC_LIMIT_HIGH_MODE(0)
	/* Illumination */
	{ CCI_REG8(0x10D7), 0x1 }, //  Illumination Trigger.ILLUM_EN(0)
	{ CCI_REG8(0x10D8), 0x2 }, //  Illumination Trigger.ILLUM_POL(0)
	/* Histogram */
	{ CCI_REG8(0x205D), 0x0 }, //  Histogram.HIST_EN(0)
	{ CCI_REG8(0x205E), 0x0 }, //  Histogram.HIST_USAGE_RATIO(0)
	{ CCI_REG8(0x2063), 0x0 }, //  Histogram.PIXEL_DATA_SUPP(0)
	{ CCI_REG8(0x2063), 0x0 }, //  Histogram.PIXEL_TRANSMISSION(0)
	/* TP */
	{ CCI_REG8(0x2091), 0x0 }, //  Test Pattern Generator.TPG_EN(0)
	{ CCI_REG8(0x2091), 0x0 }, //  Test Pattern Generator.TPG_CONFIG(0)
};

static const char *const mira220_test_pattern_menu[] = {
	"Disabled",
	"Vertial Gradient",
};

static const int mira220_test_pattern_val[] = {
	MIRA220_TEST_PATTERN_DISABLE,
	MIRA220_TEST_PATTERN_VERTICAL_GRADIENT,
};

/* regulator supplies */
static const char *const mira220_supply_name[] = {
	/* Supplies can be enabled in any order */
	"vana", /* Analog (2.8V) supply */
	"vdig", /* Digital Core (1.8V) supply */
	"vddl", /* IF (1.2V) supply */
};

#define MIRA220_NUM_SUPPLIES ARRAY_SIZE(mira220_supply_name)


// Mira220 comes in monochrome and RGB variants. This driver implements the RGB variant.
/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 mira220_mbus_formats[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,

	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,

	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,

};

/* Mode configs */
static const struct mira220_mode supported_modes[] = {
	/* 2 MPx 30fps 12bpp mode */
	{
		.width = 1600,
		.height = 1400,
		.crop = {
			.left = MIRA220_PIXEL_ARRAY_LEFT,
			.top = MIRA220_PIXEL_ARRAY_TOP,
			.width = 1600,
			.height = 1400
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(full_1600_1400_1500_12b_2lanes_reg_new),
			.regs = full_1600_1400_1500_12b_2lanes_reg_new,
		},
		// vblank is ceil(MIRA220_GLOB_NUM_CLK_CYCLES / ROW_LENGTH)  + 11
		// ROW_LENGTH is configured by register 0x102B, 0x102C.
		.row_length = 304,
		.pixel_rate = MIRA220_PIXEL_RATE,
		.min_vblank = 20,
		.max_vblank = 50000,
		.hblank = MIRA220_HBLANK_1600x1400_304,
	},
};

struct mira220 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA220 */
	u32 xclk_freq;

	struct regulator_bulk_data supplies[MIRA220_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;

	/* Current mode */
	const struct mira220_mode *mode;

	struct mutex mutex;

	struct regmap *regmap;
};

static inline struct mira220 *to_mira220(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira220, sd);
}

/* Power/clock management functions */
static int mira220_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	int ret = -EINVAL;

	ret = regulator_bulk_enable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto reg_off;
	}
	ret = clk_prepare_enable(mira220->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n", __func__);
		goto clk_off;
	}
	fsleep(MIRA220_XCLR_MIN_DELAY_US);

	return 0;

clk_off:
	clk_disable_unprepare(mira220->xclk);
reg_off:
	ret = regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);
	return ret;
}

static int mira220_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);
	(void)mira220;

	clk_disable_unprepare(mira220->xclk);
	regulator_bulk_disable(MIRA220_NUM_SUPPLIES, mira220->supplies);

	return 0;
}

static int mira220_write_start_streaming_regs(struct mira220 *mira220)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	// Setting master control
	ret = cci_write(mira220->regmap, MIRA220_IMAGER_STATE_REG,
			MIRA220_IMAGER_STATE_MASTER_CONTROL, NULL);
	if (ret) {
		dev_err(&client->dev, "Error setting master control");
		return ret;
	}

	// Enable continuous streaming
	ret = cci_write(mira220->regmap, MIRA220_IMAGER_RUN_CONT_REG,
			MIRA220_IMAGER_RUN_CONT_ENABLE, NULL);
	if (ret) {
		dev_err(&client->dev, "Error enabling continuous streaming");
		return ret;
	}

	ret = cci_write(mira220->regmap, MIRA220_IMAGER_RUN_REG,
			MIRA220_IMAGER_RUN_START, NULL);
	if (ret) {
		dev_err(&client->dev, "Error setting internal trigger");
		return ret;
	}

	return ret;
}

static int mira220_write_stop_streaming_regs(struct mira220 *mira220)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	ret = cci_write(mira220->regmap, MIRA220_IMAGER_STATE_REG,
			MIRA220_IMAGER_STATE_STOP_AT_ROW, NULL);

	if (ret) {
		dev_err(&client->dev,
			"Error setting stop-at-row imager state after multiple attempts. Exiting.");
		return ret;
	}

	ret = cci_write(mira220->regmap, MIRA220_IMAGER_RUN_REG,
			MIRA220_IMAGER_RUN_STOP, NULL);
	if (ret) {
		dev_err(&client->dev, "Error setting run reg to stop");
		return ret;
	}

	fsleep(40000);

	return ret;
}

// Returns the maximum exposure time in row_length (reg value).
// Calculation is baded on Mira220 datasheet Section 9.2.
static u32 mira220_calculate_max_exposure_time(u32 height, u32 vblank,
					       u32 row_length)
{
	return (height + vblank) -
	       (int)(MIRA220_GLOB_NUM_CLK_CYCLES / row_length);
}

static int mira220_write_exposure_reg(struct mira220 *mira220, u32 exposure)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira220->sd);
	const u32 max_exposure = mira220_calculate_max_exposure_time(
		mira220->mode->height, mira220->vblank->val,
		mira220->mode->row_length);
	u32 ret = 0;

	u32 capped_exposure = exposure;

	if (exposure > max_exposure)
		capped_exposure = max_exposure;


	ret = cci_write(mira220->regmap, MIRA220_EXP_TIME_REG, capped_exposure,
			NULL);


	if (ret) {
		dev_err_ratelimited(&client->dev,
				    "Error setting exposure time to %d",
				    capped_exposure);
		return -EINVAL;
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 mira220_get_format_code(struct mira220 *mira220, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mira220_mbus_formats); i++)
		if (mira220_mbus_formats[i] == code)
			break;

	if (i >= ARRAY_SIZE(mira220_mbus_formats))
		i = 0;

	i = (i & ~3) | (mira220->vflip->val ? 2 : 0) | (mira220->hflip->val ? 0 : 1);


	return mira220_mbus_formats[i];
}


static int mira220_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira220 *mira220 =
		container_of(ctrl->handler, struct mira220, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira220_calculate_max_exposure_time(
			mira220->mode->height, ctrl->val,
			mira220->mode->row_length);
		exposure_def = (exposure_max < MIRA220_DEFAULT_EXPOSURE) ?
				       exposure_max :
				       MIRA220_DEFAULT_EXPOSURE;
		__v4l2_ctrl_modify_range(mira220->exposure,
					 mira220->exposure->minimum,
					 exposure_max, mira220->exposure->step,
					 exposure_def);
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */

	if (pm_runtime_get_if_in_use(&client->dev) == 0) {
		dev_info(
			&client->dev,
			"device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
			ctrl->id, ctrl->val);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		break;
	case V4L2_CID_EXPOSURE:
		ret = mira220_write_exposure_reg(mira220, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(mira220->regmap, MIRA220_REG_TEST_PATTERN,
				mira220_test_pattern_val[ctrl->val], NULL);
		break;
	case V4L2_CID_HFLIP:
		ret = cci_write(mira220->regmap, MIRA220_HFLIP_REG, mira220->hflip->val,
		NULL);
		break;

	case V4L2_CID_VFLIP:
		ret = cci_write(mira220->regmap, MIRA220_VFLIP_REG, mira220->vflip->val,
				NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(mira220->regmap, MIRA220_VBLANK_REG, ctrl->val,
				NULL);

		break;
	case V4L2_CID_HBLANK:
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n", ctrl->id,
			 ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops mira220_ctrl_ops = {
	.s_ctrl = mira220_set_ctrl,
};


static void mira220_update_pad_format(struct mira220 *mira220,
	const struct mira220_mode *mode,
	struct v4l2_mbus_framefmt *fmt, u32 code)
{
	/* Bayer order varies with flips */
	fmt->code = mira220_get_format_code(mira220, code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int mira220_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct mira220 *mira220 = to_mira220(sd);
	const struct mira220_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	u32 max_exposure = 0, default_exp = 0;

	// /* Validate format or use default */


	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mira220_update_pad_format(mira220, mode, &fmt->format, fmt->format.code);

	format = v4l2_subdev_state_get_format(state, 0);
	*format = fmt->format;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->width = format->width * 1;
	crop->height = format->height * 1;
	crop->left = MIRA220_PIXEL_ARRAY_LEFT;
	crop->top = MIRA220_PIXEL_ARRAY_TOP;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		// mira220->fmt = fmt->format;
		// mira220->mode = mode;

		// Update controls based on new mode (range and current value).
		max_exposure = mira220_calculate_max_exposure_time(
			mira220->mode->height, mira220->mode->min_vblank,
			mira220->mode->row_length);
		default_exp = (max_exposure < MIRA220_DEFAULT_EXPOSURE) ?
				      max_exposure :
				      MIRA220_DEFAULT_EXPOSURE;
		__v4l2_ctrl_modify_range(mira220->exposure,
					 MIRA220_EXPOSURE_MIN, max_exposure, 1,
					 default_exp);

		// Update pixel rate based on new mode.
		__v4l2_ctrl_modify_range(mira220->pixel_rate,
					 mira220->mode->pixel_rate,
					 mira220->mode->pixel_rate, 1,
					 mira220->mode->pixel_rate);

		// Update hblank based on new mode.
		__v4l2_ctrl_modify_range(mira220->hblank, mira220->mode->hblank,
					 mira220->mode->hblank, 1,
					 mira220->mode->hblank);

		__v4l2_ctrl_modify_range(mira220->vblank,
					 mira220->mode->min_vblank,
					 mira220->mode->max_vblank, 1,
					 mira220->mode->min_vblank);

		__v4l2_ctrl_s_ctrl(mira220->vblank, mira220->mode->min_vblank);
	}

	return 0;
}

// This function should enumerate all the media bus formats for the requested pads. If the requested
// format index is beyond the number of avaialble formats it shall return -EINVAL;
static int mira220_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira220 *mira220 = to_mira220(sd);

	if (code->index >= (ARRAY_SIZE(mira220_mbus_formats) / 4))
		return -EINVAL;

	code->code = mira220_get_format_code(
		mira220, mira220_mbus_formats[code->index * 4]);

	return 0;
}

static int mira220_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira220 *mira220 = to_mira220(sd);
	u32 code;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code = mira220_get_format_code(mira220, fse->code);
	if (fse->code != code)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;


	return 0;
}

static int mira220_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_SGRBG12_1X12,
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	mira220_set_pad_format(sd, state, &fmt);

	return 0;
}


static int mira220_set_framefmt(struct mira220 *mira220,
	struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	int ret = 0;

	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
	switch (format->code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		cci_write(mira220->regmap, MIRA220_BIT_DEPTH_REG,
			  MIRA220_BIT_DEPTH_8_BIT, NULL);
		cci_write(mira220->regmap, MIRA220_CSI_DATA_TYPE_REG,
			  MIRA220_CSI_DATA_TYPE_8_BIT, NULL);
		break;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		cci_write(mira220->regmap, MIRA220_BIT_DEPTH_REG,
			  MIRA220_BIT_DEPTH_10_BIT, NULL);
		cci_write(mira220->regmap, MIRA220_CSI_DATA_TYPE_REG,
			  MIRA220_CSI_DATA_TYPE_10_BIT, NULL);

		break;
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		cci_write(mira220->regmap, MIRA220_BIT_DEPTH_REG,
			  MIRA220_BIT_DEPTH_12_BIT, NULL);
		cci_write(mira220->regmap, MIRA220_CSI_DATA_TYPE_REG,
			  MIRA220_CSI_DATA_TYPE_12_BIT, NULL);

		break;
	default:
			ret = -EINVAL;
			break;
	}

	return ret;
}


static int mira220_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA220_NATIVE_WIDTH;
		sel->r.height = MIRA220_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA220_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA220_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA220_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA220_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int mira220_start_streaming(struct mira220 *mira220,
				struct v4l2_subdev_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	const struct mira220_reg_list *reg_list;
	int ret;
	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */

	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0) {
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Apply default values of current mode */
	/* Stop treaming before uploading register sequence */
	ret = mira220_write_stop_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}

	reg_list = &mira220->mode->reg_list;
	ret = cci_multi_reg_write(mira220->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		goto err_rpm_put;
	}

	ret = mira220_set_framefmt(mira220, state);
	if (ret) {
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
			__func__, ret);
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(mira220->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	ret = mira220_write_start_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}
	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(mira220->hflip, true);
	__v4l2_ctrl_grab(mira220->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira220_stop_streaming(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret = 0;

	ret = mira220_write_stop_streaming_regs(mira220);
	if (ret) {
		dev_err(&client->dev,
			"Could not write the stream-off sequence");
	}
	__v4l2_ctrl_grab(mira220->hflip, false);
	__v4l2_ctrl_grab(mira220->vflip, false);
	pm_runtime_put(&client->dev);
}

static int mira220_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira220 *mira220 = to_mira220(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (enable)
		ret = mira220_start_streaming(mira220, state);
	else
		mira220_stop_streaming(mira220);

	v4l2_subdev_unlock_state(state);
	return ret;
}

static int mira220_get_regulators(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	unsigned int i;

	for (i = 0; i < MIRA220_NUM_SUPPLIES; i++)
		mira220->supplies[i].supply = mira220_supply_name[i];

	return devm_regulator_bulk_get(&client->dev, MIRA220_NUM_SUPPLIES,
				       mira220->supplies);
}

/* OTP power on */
static void mira220_otp_power_on(struct mira220 *mira220)
{
	int ret;

	ret = cci_write(mira220->regmap, MIRA220_OTP_CMD_REG,
			MIRA220_OTP_CMD_UP, NULL);

}

/* OTP power off */
static void mira220_otp_power_off(struct mira220 *mira220)
{
	int ret;

	ret = cci_write(mira220->regmap, MIRA220_OTP_CMD_REG,
			MIRA220_OTP_CMD_DOWN, NULL);

}

/* OTP power on */
static int mira220_otp_read(struct mira220 *mira220, u8 addr, u8 offset,
			    u8 *val)
{
	int ret;
	u64 readback;

	ret = cci_write(mira220->regmap, CCI_REG8(0x0086), addr, NULL);
	ret = cci_write(mira220->regmap, CCI_REG8(0x0080), 0x02, NULL);
	ret = cci_read(mira220->regmap, CCI_REG8(0x0082 + offset), &readback,
		       NULL);
	*val = readback & 0xFF;

	return ret;
}

/* Verify chip ID */
static int mira220_identify_module(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	int ret;
	u8 val;

	mira220_otp_power_on(mira220);

	fsleep(100);

	ret = mira220_otp_read(mira220, 0x0d, 0, &val);
	dev_err(&client->dev, "Read OTP add 0x0d with val %x\n", val);
	ret = mira220_otp_read(mira220, 0x19, 0, &val);
	dev_err(&client->dev, "Read OTP add 0x19 with val %x\n", val);
	ret = mira220_otp_read(mira220, 0x19, 1, &val);
	dev_err(&client->dev, "Read OTP add 0x19+1 with val %x\n", val);

	mira220_otp_power_off(mira220);

	return ret;
}

static const struct v4l2_subdev_core_ops mira220_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira220_video_ops = {
	.s_stream = mira220_set_stream,
};

static const struct v4l2_subdev_pad_ops mira220_pad_ops = {
	.enum_mbus_code = mira220_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mira220_set_pad_format,
	.get_selection = mira220_get_selection,
	.enum_frame_size = mira220_enum_frame_size,
};

static const struct v4l2_subdev_ops mira220_subdev_ops = {
	.core = &mira220_core_ops,
	.video = &mira220_video_ops,
	.pad = &mira220_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira220_internal_ops = {
	.init_state = mira220_init_state,
};

/* Initialize control handlers */
static int mira220_init_controls(struct mira220 *mira220)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira220->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;

	u32 max_exposure = 0;

	ctrl_hdlr = &mira220->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 9);
	if (ret)
		return ret;

	mutex_init(&mira220->mutex);
	ctrl_hdlr->lock = &mira220->mutex;
	/* By default, PIXEL_RATE is read only */
	mira220->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						mira220->mode->pixel_rate,
						mira220->mode->pixel_rate, 1,
						mira220->mode->pixel_rate);

	mira220->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					    V4L2_CID_VBLANK,
					    mira220->mode->min_vblank,
					    mira220->mode->max_vblank, 1,
					    mira220->mode->min_vblank);

	mira220->hblank =
		v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops, V4L2_CID_HBLANK,
				  mira220->mode->hblank, mira220->mode->hblank,
				  1, mira220->mode->hblank);

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	max_exposure = mira220_calculate_max_exposure_time(
		mira220->mode->height, mira220->vblank->val,
		mira220->mode->row_length);

	mira220->exposure =
		v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
				  V4L2_CID_EXPOSURE, MIRA220_EXPOSURE_MIN,
				  max_exposure, 1, MIRA220_DEFAULT_EXPOSURE);

	mira220->gain = v4l2_ctrl_new_std(
		ctrl_hdlr, &mira220_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
		MIRA220_ANALOG_GAIN_MIN, MIRA220_ANALOG_GAIN_MAX,
		MIRA220_ANALOG_GAIN_STEP, MIRA220_ANALOG_GAIN_DEFAULT);

	mira220->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (mira220->hflip)
		mira220->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	mira220->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira220_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (mira220->vflip)
		mira220->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira220_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mira220_test_pattern_menu) - 1,
				     0, 0, mira220_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n", __func__,
			ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira220_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	mira220->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira220->mutex);

	return ret;
}

static void mira220_free_controls(struct mira220 *mira220)
{
	v4l2_ctrl_handler_free(mira220->sd.ctrl_handler);
	mutex_destroy(&mira220->mutex);
}

static int mira220_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira220 *mira220;
	int ret;

	mira220 = devm_kzalloc(&client->dev, sizeof(*mira220), GFP_KERNEL);
	if (!mira220)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira220->sd, client, &mira220_subdev_ops);
	mira220->sd.internal_ops = &mira220_internal_ops;

	mira220->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(mira220->regmap))
		return dev_err_probe(dev, PTR_ERR(mira220->regmap),
				     "failed to initialize CCI\n");
	/* Get system clock (xclk) */
	mira220->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira220->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira220->xclk);
	}
	mira220->xclk_freq = clk_get_rate(mira220->xclk);
	if (mira220->xclk_freq != MIRA220_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira220->xclk_freq);
		return -EINVAL;
	}

	ret = mira220_get_regulators(mira220);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	fsleep(10000);

	// The sensor must be powered for mira220_identify_module()
	// to be able to read the CHIP_ID register

	 ret = mira220_power_on(dev);
	if (ret)
		return ret;

	fsleep(100000);

	ret = mira220_identify_module(mira220);
	if (ret)
		goto error_power_off;

	/* Set default mode to max resolution */
	mira220->mode = &supported_modes[0];

	ret = mira220_init_controls(mira220);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira220->sd.internal_ops = &mira220_internal_ops;
	mira220->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			     V4L2_SUBDEV_FL_HAS_EVENTS;
	mira220->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira220->pad.flags = MEDIA_PAD_FL_SOURCE;



	ret = media_entity_pads_init(&mira220->sd.entity, 1, &mira220->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto error_handler_free;
	}

	mira220->sd.state_lock = mira220->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&mira220->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto error_media_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&mira220->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret,
			      "failed to register sensor sub-device\n");
		goto error_subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_subdev_cleanup:
	v4l2_subdev_cleanup(&mira220->sd);

error_media_entity:
	media_entity_cleanup(&mira220->sd.entity);

error_handler_free:
	mira220_free_controls(mira220);

error_power_off:
	mira220_power_off(dev);

	return ret;
}

static void mira220_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira220 *mira220 = to_mira220(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira220_free_controls(mira220);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira220_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops mira220_pm_ops = {
		SET_RUNTIME_PM_OPS(mira220_power_off, mira220_power_on, NULL)
};

static const struct of_device_id mira220_dt_ids[] = {
	{ .compatible = "ams,mira220" },
	{ /* sentinel */ } };
MODULE_DEVICE_TABLE(of, mira220_dt_ids);

static struct i2c_driver mira220_i2c_driver = {
	.driver = {
		.name = "mira220",
		.of_match_table	= mira220_dt_ids,
		.pm = &mira220_pm_ops,
	},
	.probe = mira220_probe,
	.remove = mira220_remove,
};

module_i2c_driver(mira220_i2c_driver);

MODULE_AUTHOR("Philippe Baetens <philippe.baetens@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA220 sensor driver");
MODULE_LICENSE("GPL");
