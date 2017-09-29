/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2017 NXP
 *
 ******************************************************************************
 *
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * vic_table.h
 *
 ******************************************************************************
 */

#ifndef VIC_TABLE_H_
#define VIC_TABLE_H_

#define PROGRESSIVE 0
#define INTERLACED 1

#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1

typedef enum {
	H_TOTAL,
	H_ACTIVE,
	H_BLANK,
	HSYNC,
	FRONT_PORCH,
	BACK_PORCH,
	/* H_FREQ_KHZ, */
	V_TOTAL,
	V_ACTIVE,
	V_BLANK,
	VSYNC,
	TYPE_EOF,
	SOF,
	V_FREQ_HZ,
	PIXEL_FREQ_KHZ,
	I_P,
	HSYNC_POL,
	VSYNC_POL,
	START_OF_F0,
	START_OF_F1,
	VSYNC_START_INTERLACED_F0,
	VSYNC_END_INTERLACED_F0,
	VSYNC_START_INTERLACED_F1,
	VSYNC_END_INTERLACED_F1,
	VIC,
	VIC_R3_0,
	VIC_PR,
} MSA_PARAM;

typedef enum {
	NUM_OF_LANES_1 = 1,
	NUM_OF_LANES_2 = 2,
	NUM_OF_LANES_4 = 4,
} VIC_NUM_OF_LANES;

typedef enum {
	RATE_1_6 = 162,
	RATE_2_7 = 270,
	RATE_5_4 = 540,
	RATE_8_1 = 810,
} VIC_SYMBOL_RATE;

typedef enum {
	PXL_RGB = 0x1,
	YCBCR_4_4_4 = 0x2,
	YCBCR_4_2_2 = 0x4,
	YCBCR_4_2_0 = 0x8,
	Y_ONLY = 0x10,
} VIC_PXL_ENCODING_FORMAT;

typedef enum {
	BCS_6 = 0x1,
	BCS_8 = 0x2,
	BCS_10 = 0x4,
	BCS_12 = 0x8,
	BCS_16 = 0x10,
} VIC_COLOR_DEPTH;

typedef enum {
	STEREO_VIDEO_LEFT = 0x0,
	STEREO_VIDEO_RIGHT = 0x1,
} STEREO_VIDEO_ATTR;

typedef enum {
	BT_601 = 0x0,
	BT_709 = 0x1,
} BT_TYPE;

typedef enum {
	VIC_MODE_3_59_94Hz,
	VIC_MODE_4_60Hz,
	VIC_MODE_16_60Hz,
	VIC_MODE_97_60Hz,
	VIC_MODE_95_30Hz,
	VIC_MODE_COUNT
} VIC_MODES;

extern const unsigned int vic_table[VIC_MODE_COUNT][27];

#endif
